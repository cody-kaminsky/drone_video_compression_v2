# M4 (revised): HLS as director + hand-VHDL dataflow

**Status: planning. No code change in this commit.**

This document is the authoritative starting point for a new session
working on M4. It captures:

1. Where we are (current state, all commits, validation results).
2. Why we're pivoting from "HLS does everything" to "HLS directs, VHDL
   does the dataflow".
3. The new architecture and a per-block plan with individual testbench
   for each.
4. The development workflow.
5. The order of operations.

If you're starting a new session, read this file end-to-end first.

---

## 1. Current state

### Commits (most recent first)

| SHA | Title | Outcome |
|---|---|---|
| `e69f6dc` | M4 phase 1.5: stub `cavlc_encode_block` under `__SYNTHESIS__` (preview only) | preview LUT 153% |
| `d99259b` | M4 phase 1: VHDL CAVLC engine scaffold + tooling | VHDL skeleton |
| `e28cd9a` | HLS iteration 4: drop I_4x4 candidate + force kernel sharing | LUT 201% |
| `7389df8` | HLS iteration 3: skip per-frame NAL writes + CAVLC heuristic for synth | LUT 225% |
| `794d074` | HLS iteration 2: line-buffer all cross-MB state, relax pragmas | LUT 300% (BRAM 10%) |
| `8fcfa1d` | HLS: replace line-buffer pointer-swap with index flip | csynth runs clean |
| `08be0ed` | M3.2: HLS pragmas + Vitis HLS driver scripts | first synth attempt |
| `acf57bf` | M3 phase 1: HLS port with line-buffered recon (src/hls/) | byte-exact port |
| `0a31b53` | Fix I_4x4 boundary bug; M2 cleanups for HLS port | byte-exact restored |
| earlier... | M0 / M1 / M2: C reference + bug fixes + M2 cleanups | byte-exact baseline |

### Build status

- `dcc_encoder` (C reference): **24/24 Kodak QP-30 byte-exact** vs ffmpeg.
  Untouched by M3/M4 work. This is the golden.
- `dcc_hls` (HLS port, gcc-built): **24/24 Kodak + 15/15 drone_sample
  byte-exact**, bit-identical bitstream and recon to `dcc_encoder`. The
  `__SYNTHESIS__`-only optimizations don't run in gcc, so the test bench
  validates the architecture, not the eventual synthesized hardware.
- VHDL CAVLC scaffold: not yet validated in simulation. `coeff_token_
  encoder.vhd` is fully implemented but un-tested.

### Last synthesis report (with CAVLC stubbed for preview)

```
                    Estimate     Budget (xc7z030)
BRAM_18K              49 (9%)     530       ✅
DSP                  107 (26%)    400       ✅
FF                 94022 (59%)    157200    ✅
LUT               121029 (153%)   78600     ⚠️ over
Clock                 4.83 ns     5.0 ns    ✅ 200 MHz
```

With real CAVLC inline (no stub): LUT was 158,656 (201%). The stub
gives us the post-M4 optimistic-floor preview.

### What works end-to-end

- Build `dcc_encoder` and `dcc_hls`: `make`
- Run validation: `bash tools/validate_byteexact.sh datasets/kodak/ 30`
- Self-decode debug build: `CFLAGS='... -DMB_SELFDECODE' make -B`;
  the gcc binary verifies its own bitstream every MB.
- HLS C synth: open in Vitis IDE, point at the sources in `src/hls/` +
  shared modules in `src/`, run C Synthesis. See
  [docs/hls-synthesis.md](hls-synthesis.md) for the IDE flow.
- CAVLC test-vector generation: `make vectors` produces 3243
  (input, output) pairs in `build/cavlc_vectors_*.txt`.

---

## 2. Why pivot

The HLS port was supposed to be the v1 hardware target. After four
iterations of pragma tuning + structural refactoring (line buffer,
sentinel cleanup, mode-decide simplification, kernel sharing), the
LUT count plateaus at:

- **201%** with HLS doing everything (iteration 4).
- **153%** with HLS-side CAVLC stubbed out (M4 phase 2 preview).
- **~80%** projected after a full M4 with VHDL CAVLC + dropping the
  4-way I_16x16 trial + dropping chroma Plane mode + simplifying mode
  decision to "DC always" or "SATD-only".

To hit **~80%** we'd have to drop ~30% of the encoder's mode-decision
intelligence. That's a real BD-rate cost (~5-10% bitrate at constant
PSNR per architecture.txt §1) and means losing functionality.

Comparable hand-VHDL Baseline I-only encoders ship in the **20-30k LUT**
range (~30% utilization on this part) **with all modes intact**. The
gap isn't the algorithm — it's HLS generating 6-9× the hardware that's
actually needed because:

- Every `complete` ARRAY_PARTITION pushes data into LUT-RAM rather
  than BRAM. We've already walked many back to `cyclic factor=4`, but
  HLS still over-replicates.
- Every PIPELINE II=1 spawns a parallel datapath. We have ~2000
  cycles of headroom per MB at 1080p30 / 200 MHz; we don't *need*
  II=1 in inner loops.
- Function cloning at call sites. ALLOCATION pragmas help but PIPELINE
  overrides them in some cases (see `Pipeline_LOOP_631` in iter 4).
- Variable-length code emission (CAVLC, NAL header building, RBSP
  escape) is the canonical anti-pattern for HLS — generic dispatch
  hardware versus hand-written state machines.

**The decision: keep all functionality, hand-write the dataflow in
VHDL, and demote HLS to orchestration.**

---

## 3. New architecture

### Top-level diagram

```
                    Zynq PS (ARM, software)
                           │
                           │ AXI-Lite control
                           ▼
       ┌─────────────────────────────────────────────────┐
       │   HLS Director (encode_frame_h264_top)           │
       │                                                  │
       │   - reads MB samples from src AXI master         │
       │   - sequences MBs in raster order                │
       │   - per MB: kicks off mode-decide, gathers        │
       │     neighbor data, sequences predict→            │
       │     transform→quant→cavlc                        │
       │   - maintains line buffer (recon edges, nC,       │
       │     mode4) — same shape as src/hls/line_buffer.h │
       │                                                  │
       │   Estimated size: 5-10k LUT, mostly state         │
       │   machines and AXI fabric.                       │
       └────────────────┬─────────────────────────────────┘
                        │ AXI-Stream level packets
                        │ + control sidebands
                        ▼
       ┌─────────────────────────────────────────────────┐
       │   VHDL Dataflow Engines (one shared instance     │
       │   each, shared by every MB in raster order)      │
       │                                                  │
       │   transform_engine    DCT 4×4, iDCT 4×4,         │
       │                       Hadamard 4×4 + 2×2.        │
       │                       ~1500 LUT.                 │
       │                                                  │
       │   quant_engine        quant_4x4, iquant_4x4,     │
       │                       quant_dc, iquant_dc.       │
       │                       ~2000 LUT, uses DSP.       │
       │                                                  │
       │   predict_4x4_engine  9 modes, sample-fed.       │
       │                       ~3000 LUT.                 │
       │                                                  │
       │   predict_16x16_engine 4 modes (V/H/DC/Plane).   │
       │                        ~1500 LUT.                │
       │                                                  │
       │   predict_chroma_engine 4 modes for 8x8 chroma.  │
       │                         ~1500 LUT.               │
       │                                                  │
       │   cavlc_engine         coeff_token + levels +    │
       │                        total_zeros + run_before  │
       │                        + bit packer.             │
       │                        ~5000 LUT.                │
       │                                                  │
       │   bitstream_packer     bit-level → byte-stream,  │
       │                        with NAL escape.          │
       │                        ~500 LUT.                 │
       │                                                  │
       │   Subtotal: ~15-20k LUT.                          │
       └────────────────┬─────────────────────────────────┘
                        │ AXI-Stream bytes
                        ▼
                ┌──────────────────┐
                │  AXI HP master    │
                │  to DDR           │
                └──────────────────┘
```

**Total estimated: 20-30k LUT.** Same range as published designs.

### Why one shared instance per kernel

For 1080p30 we have 2040 cycles per MB at 200 MHz. A single per-MB
encode pass needs roughly:

- 16 luma 4×4 DCTs + 16 iDCTs = 32 transform calls × ~10 cycles each
  = 320 cycles
- 16 luma 4×4 quants + 16 iquants = 32 quant calls × ~16 cycles each
  = 512 cycles
- 16 4×4 predicts per MB at ~10 cycles each = 160 cycles
- ~16 CAVLC blocks per MB at ~15 cycles each = 240 cycles
- chroma path: ~200 cycles total
- pipeline / handoff / control: ~200 cycles

**Total: ~1600 cycles per MB sequential.** Comfortably under 2040. So
sharing one instance of each engine across all per-MB calls works,
even at II=1 inside each engine.

This is the main efficiency: HLS was instantiating 4-16 copies of
each engine in parallel with II=1, when one shared copy at sequential
schedule is plenty.

---

## 4. Per-block plan

Each block is a self-contained VHDL entity with its own testbench.
Test vectors are generated from the C reference (same approach as
`tools/gen_cavlc_vectors.c`) and bit-compared in simulation.

### Files per block

For block `<name>`:

- `src/vhdl/<name>.vhd` — entity + architecture
- `src/vhdl/<name>_tb.vhd` — testbench, reads vectors, drives engine,
  compares output
- `tools/gen_<name>_vectors.c` — vector generator wrapping the C
  reference's golden function
- `src/vhdl/run_<name>_tb.tcl` — Vivado xsim run script

### Block list

| # | Block | Status | Vector source (C reference) | Estimated LUT |
|---|---|---|---|---|
| 1 | `cavlc_engine` | scaffold + 1 sub-module done | `cavlc_encode_block` | 5000 |
| 2 | `bit_packer` | TODO | (test pattern based) | 500 |
| 3 | `transform_engine` | TODO | `dct4x4` + `idct4x4` + `hadamard*` | 1500 |
| 4 | `quant_engine` | TODO | `quant_4x4` + `iquant_4x4` + DC variants | 2000 |
| 5 | `predict_4x4_engine` | TODO | `predict_4x4` (in `src/intra.c`) | 3000 |
| 6 | `predict_16x16_engine` | TODO | `predict_16x16` | 1500 |
| 7 | `predict_chroma_engine` | TODO | `predict_chroma_8x8` | 1500 |
| 8 | `mode_decide_controller` | TODO | `mb_mode_decide` orchestration | 3000 |
| 9 | `mb_pipeline_controller` | TODO | `encode_mb_emit` orchestration | 2000 |
| 10 | (HLS director — minimal) | TODO | `encode_frame_h264_hls` outer loop | ~3000 (HLS-generated) |

Total estimate: **~23,000 LUT** (29% utilization on xc7z030). Well in
budget with margin.

### Block 1: `cavlc_engine` (started)

**Status**: phase 1 scaffold committed. `coeff_token_encoder.vhd` is
fully implemented; the rest is TODO.

**Submodules** (each its own .vhd + testbench):
- `coeff_token_encoder.vhd` — DONE
- `levels_encoder.vhd` — TODO. Tracks `suffix_length`. Hardest.
- `total_zeros_encoder.vhd` — TODO. ROM lookup.
- `run_before_encoder.vhd` — TODO. ROM lookup loop.
- `bit_packer.vhd` — shared with bitstream packer (block 2).

**Vector source**: existing `tools/gen_cavlc_vectors.c` produces 3243
vectors. Build with `make vectors`.

**Testbench**: `src/vhdl/cavlc_engine_tb.vhd` — reads
`build/cavlc_vectors_in.txt` and `build/cavlc_vectors_out.txt`,
drives the engine, asserts output match.

**See**: [docs/m4-cavlc-vhdl.md](m4-cavlc-vhdl.md) for the detailed
sub-module plan.

### Block 2: `bit_packer`

**Function**: shift register that accumulates variable-length bit
fields and emits bytes to an AXI-Stream output.

**Interface**:
```vhdl
entity bit_packer is
    port (
        clk      : in  std_logic;
        rst_n    : in  std_logic;
        bits_i   : in  unsigned(31 downto 0);  -- right-aligned
        length_i : in  unsigned(5 downto 0);   -- 0..32
        valid_i  : in  std_logic;
        ready_o  : out std_logic;
        flush_i  : in  std_logic;              -- emit any partial byte
        out_data  : out unsigned(7 downto 0);
        out_valid : out std_logic;
        out_ready : in  std_logic
    );
end;
```

**Vector source**: a simple test harness in C that calls
`bs_put_bits` with a deterministic sequence and dumps both the input
(bits, length) tuples and the output bytes.

**Used by**: `cavlc_engine`, `bitstream_packer` (slice header /
trailing bits).

### Block 3: `transform_engine`

**Function**: 4×4 forward DCT, 4×4 inverse DCT, 4×4 Hadamard, 2×2
Hadamard + inverses. One shared core that selects which transform
based on a control input.

**Interface**:
```vhdl
type transform_op_t is (TX_DCT4, TX_IDCT4, TX_HAD4, TX_IHAD4, TX_HAD2, TX_IHAD2);

entity transform_engine is
    port (
        clk    : in  std_logic;
        rst_n  : in  std_logic;
        op_i   : in  transform_op_t;
        in_data  : in  signed_array_t(0 to 15);  -- up to 16 elements
        in_valid : in  std_logic;
        in_ready : out std_logic;
        out_data  : out signed_array_t(0 to 15);
        out_valid : out std_logic
    );
end;
```

**Vector source**: `tools/gen_transform_vectors.c` calls `dct4x4`,
`idct4x4`, `hadamard4x4`, `hadamard2x2`, `ihadamard4x4`, `ihadamard2x2`
from `src/transform.c` with random inputs (xorshift PRNG).

**Note**: each transform takes ~10 cycles fully pipelined. Could be
made II=1 if needed for throughput; with sharing, II=4 is plenty.

### Block 4: `quant_engine`

**Function**: forward quant + inverse quant, both 4×4 AC, 4×4 DC, 2×2
chroma DC. QP-driven scaling factors via small ROM.

**Interface**: similar to transform_engine, with an extra QP input.

**Vector source**: `tools/gen_quant_vectors.c` from `src/quant.c`
functions.

**DSP usage**: this is where most multipliers go. Estimate ~16-20
DSPs (well within 400 budget).

### Block 5: `predict_4x4_engine`

**Function**: 9 intra-4×4 prediction modes per spec 8.3.1.2. Inputs:
mode index, 13 reference samples (top[0..7], left[0..3], top-left).
Output: 16 predicted samples.

**Interface**:
```vhdl
entity predict_4x4_engine is
    port (
        clk      : in  std_logic;
        rst_n    : in  std_logic;
        mode_i   : in  unsigned(3 downto 0);
        top_i    : in  unsigned8_array_t(0 to 7);
        left_i   : in  unsigned8_array_t(0 to 3);
        tl_i     : in  unsigned(7 downto 0);
        avail_top_i, avail_left_i, avail_tl_i : in std_logic;
        in_valid : in  std_logic;
        out_pred  : out unsigned8_array_t(0 to 15);
        out_valid : out std_logic
    );
end;
```

**Vector source**: `tools/gen_predict_4x4_vectors.c` calls
`predict_4x4` from `src/intra.c` with random reference samples.

**Note**: this implements the prediction itself, NOT mode selection.
The mode-decide controller (block 8) iterates modes and computes SATD
costs externally.

### Block 6: `predict_16x16_engine`

**Function**: 4 modes (V/H/DC/Plane), 16×16 output samples. Plane
mode is the expensive one; H/V/DC are trivial.

**Vector source**: `tools/gen_predict_16x16_vectors.c`.

### Block 7: `predict_chroma_engine`

**Function**: 4 modes (DC/V/H/Plane) on 8×8 chroma.

**Vector source**: `tools/gen_predict_chroma_vectors.c`.

### Block 8: `mode_decide_controller`

**Function**: orchestrate the per-MB mode trial. Reads source samples
+ neighbor edges, dispatches predict + transform + quant for each
candidate, picks the winner by bit-estimate, emits the winning mode +
levels.

This is mostly state machine — a few thousand LUT.

**Substates**:
- I_16x16 mode trial: try V, H, DC, Plane → pick best
- I_4x4 mode trial: per-block, try 9 modes → pick best per block
- Chroma mode trial: try DC, V, H, Plane → pick best
- Final: emit selected mode + transformed/quantized coefficients

**Validation**: this is the place where bit-exact equivalence with the
C reference is hardest, because mode-decision tie-breaks may differ.
Strategy: dump full per-MB candidate evaluations from the C reference;
the testbench asserts the controller picks the same winner.

### Block 9: `mb_pipeline_controller`

**Function**: top-level state machine that sequences the per-MB stages:
fetch source → mode_decide → reconstruct → cavlc_emit → commit recon
to line buffer → next MB.

Maintains the line buffer (recon edges, nC, mode4) — same shape as
[src/hls/line_buffer.h](../src/hls/line_buffer.h).

### Block 10: HLS director

**Function**: top-level frame loop. Reads source from DDR via AXI
master, hands MBs to `mb_pipeline_controller`, writes the output
bitstream to DDR. Handles AXI-Lite control plane.

**Why HLS for this**: AXI master / AXI-Lite glue + frame-level outer
loops are exactly what HLS is good at. Variable-length emit is gone
(it's in CAVLC now), so there's nothing HLS-hostile here.

This becomes a tiny shell — maybe 200 lines of C.

---

## 5. Development workflow

### Per block

1. **Define the interface** (signal list / record / port).
2. **Write the test-vector generator** in C, using the existing
   reference functions from `src/`. Output two text files (input
   vectors, golden output).
3. **Write the VHDL entity** + architecture.
4. **Write the testbench**: clock + reset, file-IO loop reading
   vectors, driving the entity, asserting output match.
5. **Run xsim** in Vivado: `vivado -mode batch -source run_<name>_tb.tcl`
   or open the project and click "Run Simulation".
6. **Iterate** until all vectors pass.

### Tooling that already exists

- `tools/gen_cavlc_tables_vhd.py` — C tables → VHDL constants
  (idempotent, re-run when C tables change).
- `tools/gen_cavlc_vectors.c` — vector generator pattern. Copy/adapt
  for each new block.
- `Makefile`'s `vectors` target builds the CAVLC vector generator.
  Add similar targets for each block.

### Vivado xsim setup

A minimal `run_<name>_tb.tcl`:

```tcl
create_project -in_memory -part xc7z030-sbg485-3
add_files -norecurse src/vhdl/cavlc_pkg.vhd src/vhdl/cavlc_tables.vhd
add_files -norecurse src/vhdl/<name>.vhd
add_files -norecurse src/vhdl/<name>_tb.vhd
set_property top <name>_tb [get_filesets sim_1]
launch_simulation
run all
```

(The testbench self-asserts; xsim exits 0 on pass, non-zero on fail.)

---

## 6. Order of operations

Recommended sequence (each step is a self-contained commit):

1. **Bit packer** (block 2). Smallest, generic, no spec details.
   Write it first to validate the workflow (vector gen → VHDL entity
   → testbench → xsim pass). ~1 day.
2. **Transform engine** (block 3). Pure dataflow, well-specified by
   the spec, no state. Good "second test of the workflow". ~2 days.
3. **Quant engine** (block 4). Same shape as transform. ~2 days.
4. **Predict 4×4** (block 5). 9 modes, each is a few formulas. Big
   case statement. ~3 days.
5. **Predict 16×16** (block 6) and **chroma** (block 7). Smaller. ~2
   days each.
6. **CAVLC engine** finish (block 1). The biggest single block;
   already started. ~1 week.
7. **Mode-decide controller** (block 8). Orchestrates the above. ~1
   week (most of it spent making the C-reference comparison robust).
8. **MB pipeline controller** (block 9). Smaller; uses the
   line-buffer structure already designed in src/hls/line_buffer.h.
   ~3 days.
9. **HLS director** (block 10). Strip src/hls/encoder.c down to the
   frame-level outer loop and AXI plumbing. Compile, csynth, cosim
   against the assembled VHDL. ~1 week.
10. **Vivado integration**: M5 phase work — block design, timing
    closure, on-board bring-up. Already planned in
    [docs/hls-synthesis.md](hls-synthesis.md).

**Total estimate: 5-7 weeks for someone fluent in VHDL and Vivado.**
Realistic for a part-time project: 3-4 months.

---

## 7. Where to start a new session

In a new Claude session, paste this opening prompt:

> I'm working on an H.264 baseline I-only encoder for Zynq-7030. The
> project pivoted from "HLS does everything" to "HLS as director,
> hand-VHDL for dataflow" — see `docs/m4-architecture-pivot.md` for
> the full plan. The C reference (`src/encoder.c`) is the byte-exact
> golden. The HLS port (`src/hls/encoder.c`) was the M3 attempt and
> is still useful for orchestration. The VHDL scaffold lives in
> `src/vhdl/`; only `coeff_token_encoder.vhd` is implemented so far.
> I want to start on **block 2 (`bit_packer`)** next — please read
> the architecture-pivot doc and the existing CAVLC scaffold, then
> propose the bit_packer interface, vector format, and testbench
> approach before writing code.

That gives Claude:
- Where to read first.
- The current state.
- The next concrete task.
- Permission to design before coding.

### Files to read first in any new session

1. This file: `docs/m4-architecture-pivot.md`
2. `architecture.txt` §1, §10, §15 — top-level constraints / register
   map / milestone plan.
3. `docs/m4-cavlc-vhdl.md` — the CAVLC sub-architecture.
4. `src/hls/line_buffer.h` — the cross-MB state shape; the same
   structure will live inside `mb_pipeline_controller`.
5. The relevant C-reference functions in `src/` for whichever block
   is next.

---

## 8. Decisions deferred

These don't block M4 but should be revisited in M5:

- **Deblocking filter**: currently disabled in the bitstream
  (`disable_deblocking_filter_idc = 1` in the slice header). For
  visible quality at low QP, deblocking is needed. Deferred to a
  separate M-step.
- **Rate control**: the encoder is fixed-QP. Real-world deployments
  need a frame-level QP feedback loop. Deferred per architecture.txt
  §16 R3.
- **P-frames**: I-only is locked for v1 per architecture.txt header.
  4K30 deferred per R1.
