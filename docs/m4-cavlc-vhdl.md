# M4: CAVLC engine in VHDL

## Why VHDL

CAVLC encoding emits variable-length codes (1–32 bits per symbol) whose
length depends on the value being encoded *and* on neighbour state (`nC`).
Variable-length output is the canonical "bad fit for HLS" workload —
synthesizing it through Vitis HLS produced the largest single LUT block in
the encoder kernel (~12k LUT for `cavlc_encode_block`, plus another ~25k
of dispatch / pipeline overhead in `mb_cavlc_emit`). A hand-written VHDL
implementation hits the same task in ~5k LUT (per published H.264 baseline
designs).

This is the main lever to bring the kernel from ~150% to <100% LUT
utilization on Zynq-7030. Per [architecture.txt §15](../architecture.txt),
this was always M4 work; the M3 HLS pass was the prerequisite.

## Interface

The HLS kernel (everything except CAVLC) produces a stream of "level
packets" — quantized coefficients in zigzag order plus the metadata CAVLC
needs. The VHDL engine consumes that stream and produces a bitstream.

**Level packet** (defined in [src/vhdl/cavlc_pkg.vhd](../src/vhdl/cavlc_pkg.vhd)):

```vhdl
type level_packet_t is record
    block_type : block_type_t;          -- LUMA_DC_16x16 / LUMA_AC / LUMA_FULL / CHROMA_DC / CHROMA_AC
    n_coefs    : unsigned(4 downto 0);  -- 4, 15, or 16
    nC         : unsigned(4 downto 0);  -- 0..16, 0xFF (=31) for chroma DC sub-table
    levels     : level_array_t;          -- 16 × signed(15 downto 0), zigzag-ordered
end record;
```

In hardware this becomes an AXI-Stream port: each packet is one beat of a
~36-byte payload. The HLS encoder writes packets via `hls::stream<>`; Vitis
synthesizes that to AXI4-Stream. The VHDL engine consumes the same stream.

**Output**: AXI-Stream of bytes (the CAVLC-encoded slice data, ready to
be wrapped in NAL framing on the host PS).

## File inventory

| File | Status | Role |
|---|---|---|
| `src/vhdl/cavlc_pkg.vhd` | scaffold | Types, constants, function declarations |
| `src/vhdl/cavlc_tables.vhd` | scaffold (generated) | ROM constants for coeff_token, total_zeros, run_before |
| `src/vhdl/coeff_token_encoder.vhd` | proof-of-concept | First sub-block: emits the `coeff_token` VLC given (nC, TotalCoeff, TrailingOnes) |
| `src/vhdl/cavlc_engine.vhd` | scaffold | Top-level entity + state machine skeleton |
| `src/vhdl/cavlc_engine_tb.vhd` | TODO | Testbench — feeds vectors, compares output |
| `tools/gen_cavlc_tables_vhd.py` | done | Translates `src/cavlc_tables.h` → VHDL |
| `tools/gen_cavlc_vectors.c` | done | Wraps the C reference's `cavlc_encode_block` to dump test vectors |

## CAVLC encode order (spec 9.2.1)

For one residual block:

1. **`coeff_token`** (1–16 bits). Encodes `(TotalCoeff, TrailingOnes)`. ROM
   lookup, sub-table selected by `nC` range (`{0-1}`, `{2-3}`, `{4-7}`,
   `{8-16}`, or chroma_dc).
2. **TrailingOnes signs** (0–3 bits). One bit per trailing-one
   coefficient. Trivial.
3. **Levels** (variable, per coefficient). Suffix-length-encoded magnitude.
   The trickiest part: `suffix_length` mutates as levels are emitted.
4. **`total_zeros`** (1–9 bits). Only if `TotalCoeff < n_coefs`. ROM
   lookup, sub-table selected by `TotalCoeff`.
5. **`run_before`** for each non-zero coef except the last (variable).
   ROM lookup, sub-table selected by remaining `zeros_left`.

The C reference at [src/cavlc.c](../src/cavlc.c) implements all of this in
~250 lines — study it as the canonical reference for the VHDL.

## State machine outline

```
            ┌─────────────┐
            │    IDLE     │
            └──────┬──────┘
                   │ packet arrives on input stream
                   ▼
            ┌─────────────┐
            │ COUNT_COEFS │  scan levels[], find TotalCoeff and
            └──────┬──────┘  TrailingOnes (1 cycle, combinational)
                   ▼
            ┌─────────────┐
            │ COEFF_TOKEN │  ROM lookup → emit VLC bits
            └──────┬──────┘  (1 cycle, ROM is registered)
                   ▼
            ┌─────────────┐
            │  ONES_SIGN  │  emit 0..3 sign bits
            └──────┬──────┘
                   ▼
            ┌─────────────┐
            │    LEVELS   │  loop over (TotalCoeff − TrailingOnes) levels.
            └──────┬──────┘  Each level is 2..N bits, suffix_length tracked.
                   │         O(TotalCoeff) cycles.
                   ▼
            ┌─────────────┐
            │ TOTAL_ZEROS │  ROM lookup → emit
            └──────┬──────┘
                   ▼
            ┌─────────────┐
            │  RUN_BEFORE │  loop over (TotalCoeff − 1) runs.
            └──────┬──────┘  Each run is 1..11 bits.
                   │         O(TotalCoeff) cycles.
                   ▼
            ┌─────────────┐
            │     DONE    │  signal complete; back to IDLE.
            └─────────────┘
```

A typical 4×4 block with TotalCoeff=4 takes ~15 cycles; an empty block
(TotalCoeff=0) takes ~3. At 200 MHz and ~16 blocks per MB plus chroma,
that's ~250-400 cycles per MB for CAVLC alone — well under the
~2000-cycle MB budget for 1080p30.

## Bit packer

A separate sub-module accumulates bits into an output byte register.
Inputs: bits + length + valid. Outputs: byte + valid (with appropriate
backpressure on the AXI-Stream).

This is generic, ~200 lines of VHDL. Standard pattern.

## Testbench strategy

The C reference's `cavlc_encode_block` is the golden model. The vector
generator (`tools/gen_cavlc_vectors.c`) wraps it: input = a level_packet,
output = the resulting bytes. We dump thousands of (input, output) pairs
covering all block types, nC ranges, and TotalCoeff values.

The VHDL testbench reads these files, drives the engine, and bit-compares
the output. Pass/fail per vector. With the existing 563 round-trip
vectors from `tests/test_cavlc_roundtrip.c`, plus generated cases for
edge conditions (TotalCoeff=0, all trailing ones, max levels), we can
get high confidence the VHDL matches the C ref.

Once the VHDL passes its testbench, M5 integrates it into the Vivado
block design and we run cosim against the HLS kernel.

## What's done in this commit (M4 phase 1)

- Directory + design doc (this file).
- VHDL package types and constants.
- Table ROM generator script.
- Generated `cavlc_tables.vhd`.
- `coeff_token_encoder.vhd` — first concrete sub-module, fully implemented
  and ready for its own testbench.
- `cavlc_engine.vhd` — entity declaration + state machine skeleton with
  TODO markers for each substate.
- Test-vector generator (C source + makefile target).

## What's next (M4 phase 2 and beyond)

In rough order:

1. **Bit packer module** — small, generic. ~200 lines.
2. **Levels emitter** — the trickiest substate (suffix_length tracking).
   ~150 lines.
3. **`total_zeros` emitter** — straightforward ROM lookup. ~50 lines.
4. **`run_before` emitter** — ROM lookup loop. ~80 lines.
5. **State machine glue** in `cavlc_engine.vhd`. ~150 lines.
6. **Testbench** — populates vectors, drives clock, asserts output match.
7. **HLS encoder refactor** — replace `cavlc_encode_block` calls with
   stream writes. Verify byte-exact in cosim.

Estimated ~1500-2000 lines of VHDL plus ~500 lines of testbench. About
1-2 weeks of work for someone fluent in VHDL synthesis.
