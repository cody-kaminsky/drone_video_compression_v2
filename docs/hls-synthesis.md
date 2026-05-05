# HLS synthesis flow

Quick-start for getting `src/hls/` through Vitis HLS, all the way to a
Vivado-ready IP package. Assumes a working Vitis HLS install (2023.x or
later; older versions may need pragma syntax tweaks).

## What's in `src/hls/`

| File | Role |
|---|---|
| `hls_top.c` | Synthesis entry point. AXI master / AXI-Lite interface pragmas. |
| `encoder.c` | Line-buffered per-MB encoder. Mirrors `src/encoder.c` but reads neighbors from edge buffers, not a full plane. |
| `encoder_hls.h` | Public API declaration. |
| `line_buffer.{c,h}` | Edge-buffer struct + neighbor gathers. Replaces the W*H static recon plane. |
| `hls_pragmas.h` | `HLS_PRAGMA()` macro — inert in gcc, expands to `#pragma HLS ...` under `__SYNTHESIS__`. |
| `csynth.tcl` | Vitis HLS project + synthesis driver. |
| `directives.tcl` | Per-function pragmas for shared modules (`src/transform.c`, etc.). |
| `main.c` | CLI driver for the gcc-built `dcc_hls` binary, used for parity testing vs `dcc_encoder`. |

The shared kernel modules (`src/transform.c`, `src/quant.c`, `src/intra.c`,
`src/cavlc.c`, `src/bitstream.c`, `src/nal.c`) are identical to what the
C reference uses — Vitis HLS just compiles them with synthesis pragmas
applied via `directives.tcl`.

## Verifying the C side first (no Vitis needed)

The HLS port has to be byte-exact with the C reference before you spend
time on synthesis. From the repo root:

```bash
make                                                       # builds both
bash tools/validate_byteexact.sh datasets/kodak/ 30                          # C ref
bash tools/validate_byteexact.sh datasets/kodak/ 30 build/dcc_hls.exe        # HLS port
```

Both should report `120 pass, 0 fail` (24 images x 5 QPs by default;
pass the QP list as `30` for a quicker sanity-check). The two binaries
also produce bit-identical bitstreams — verify with:

```bash
build/dcc_encoder.exe in.yuv 768 512 30 ref_recon.yuv ref.264
build/dcc_hls.exe     in.yuv 768 512 30 hls_recon.yuv hls.264
cmp ref.264 hls.264                # should be silent
cmp ref_recon.yuv hls_recon.yuv    # should be silent
```

If those don't match, **stop and debug** — Vitis can't fix a
behavioral bug, and cosim will only confirm whatever the C source
already does.

## Running C synthesis

From the repo root:

```bash
vitis_hls -p src/hls/csynth.tcl
```

This creates `hls_proj/solution1/`. Watch the console for:

- **`Synthesis pragma errors`** — usually a malformed `HLS_PRAGMA()` in
  `src/hls/encoder.c` or a typo in `directives.tcl`. Fix and re-run.
- **`Unable to schedule`** — a loop has a carried dependency Vitis
  can't pipeline. Common spots: the per-block `for blk = 0..15` loop in
  `try_path_i4x4` (each block's recon feeds the next block's neighbors).
  Acceptable to leave at II>1; check that the achieved II is still fast
  enough for 1080p30.
- **`Resource exceeded`** — partitioning was too aggressive. Drop
  `complete` to `cyclic factor=N` for some arrays.

After it finishes, the report:

```
hls_proj/solution1/syn/report/encode_frame_h264_hls_top_csynth.rpt
```

## Reading the synthesis report

The per-loop / per-function table at the top tells you achieved II,
latency, and resource usage. Targets:

| Metric | Target | Notes |
|---|---|---|
| Top-level Fmax | ≥ 200 MHz | Set by `create_clock -period 5` in csynth.tcl. |
| Per-MB latency | ≤ ~10K cycles @ 200 MHz | 1080p30 has 8160 MB/frame * 30 fps = 245k MB/s; 200 MHz / 245k = 816 cycles per MB max for II=1. With pipelined MBs and parallelism, headroom is real but not huge. |
| Resource usage | LUT ≤ 78600, FF ≤ 157200, BRAM18Kb ≤ 86, DSP ≤ 400 | xc7z030 budget per architecture.txt §1. |

## Iterating

Each csynth pass takes 5–15 minutes on this design. Typical loop:

1. Read the report. Find the worst II / longest critical path / largest
   resource line.
2. Adjust `directives.tcl` (most common) or add a `HLS_PRAGMA()` in
   `src/hls/encoder.c` or `src/hls/line_buffer.c` (less common — those
   are mostly already annotated).
3. `vitis_hls -p src/hls/csynth.tcl` again.

Keep `git diff` small per iteration so you can bisect when something
gets worse.

## C/RTL cosim (next step)

After csynth is clean:

1. Add a test bench at `tests/tb_hls_kernel.c` that:
   - Reads a known-good golden output (produced by `dcc_hls.exe`).
   - Calls `encode_frame_h264_hls_top` with the same input.
   - Compares output bytes; returns 0 on match, nonzero on mismatch.
2. Uncomment the `add_files -tb ...` and `cosim_design` lines in
   `csynth.tcl`.
3. Re-run `vitis_hls -p src/hls/csynth.tcl`. Cosim is much slower than
   csynth (10x–100x for our kernel). Use a small frame (e.g. 256x256
   synthesized via `tools/make_test_frame.py`) to iterate.

## Exporting the IP

Once csynth and cosim both pass:

```tcl
# Append to csynth.tcl or run manually after open_solution:
export_design -format ip_catalog -ipname dcc_h264_encoder -version 1.0.0
```

This drops a `.zip` under `hls_proj/solution1/impl/ip/` that you add to
the Vivado IP repository (Tools → Settings → IP → Repository in the
Vivado GUI). From there it's an ordinary block-design IP.

## What's NOT going through HLS

Per [architecture.txt §15](../architecture.txt) M4: the CAVLC engine
(variable-length output, table-driven coeff/run/level coding) is a poor
HLS target and will be hand-VHDL. The current `directives.tcl` still
synthesizes `cavlc.c` so we can validate the rest of the kernel; once
the VHDL CAVLC IP is ready, we'll refactor the encoder to emit a level
stream over an AXI-Stream port and remove `cavlc.c` from the HLS source
list.

The host-side PSNR computation in `src/psnr.c` is not part of the
synthesized kernel — it runs on the Zynq PS / dev host.
