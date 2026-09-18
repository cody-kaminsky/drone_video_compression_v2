# Per-MB rate control in the kernel: what it costs

Follow-up to [rate-control-mb-vs-frame.md](rate-control-mb-vs-frame.md),
which measured that frame-level control alone does not hold the receiver
bucket and that the per-MB correction needs the previous frame's bit map.
This is the RTL of that path, its verification, and what it costs in
cycles, logic and compression. Measured 2026-09-18 on `claude/p-frames`
after merging `main` (the M5 kernel, VERSION 1.2).

## What was built

The host keeps the frame-level controller (bucket, frame model). The
kernel gets the MB loop:

- `rc_mb_engine.vhd` (new): the integer form of the C reference's per-MB
  step (`rc_mb == 3`, `--rc-hw`), which the C now carries so the two are
  bit-exact. Per MB marker from the merger it accumulates the bits spent
  and the expected spend (previous frame's per-MB bits times a host-supplied
  scale, from a 16-bit-per-MB map in block RAM), and steps the QP one
  towards `clamp(qp_frame + round(6 log2(spent/expect)) +
  trunc(4 (spent - expect) / target))`. One shared 32x16 multiplier; the
  log2 is a 5-step binary search over 32 thresholds, the division a
  4-step search; about 30 cycles per MB, off the critical path.
- `cavlc_dispatch.vhd`: an end-of-MB marker item (kind 3) and two ports
  that report every field or byte entering the output packer, so the
  engine counts each MB's bits exactly as the C does.
- `mb_header_engine.vhd`: a real `mb_qp_delta` (se(v), wrapped into
  [-26, 25]) with QP_Y,PRED tracked per spec: it moves only when a delta is
  transmitted, i.e. I_16x16 always, I_4x4 only with residual.
- `mb_pipeline_controller.vhd`: takes the QP for each MB from the engine
  at the decider start, hands the MB's QP to the header engine with the
  decision, pushes the marker after the MB's last packet.
- `encoder_axi_top.vhd`: VERSION 1.3, registers RC_CTRL 0x24 (enable, map
  valid, qp_min, qp_max, lag), RC_TARGET 0x28, RC_SCALE 0x2C, RC_WTOTAL
  0x30 (read: the frame's MB bits, the next frame's w_total). Latched at
  START like CONFIG. With RC_EN clear the stream is 1.2's, bit for bit.
- `host/h264_host.h`: the register offsets and the two helpers the host
  needs (`h264_rc_ctrl_word`, `h264_rc_scale`). `board_main.c` does not
  drive them yet.

**The lag.** The decision of MB i starts while MB i-1 is being emitted, so
the kernel can only know the bits through MB i-1-lag. The engine waits for
the merger's marker of that MB; lag is a register (default 2 = the
pipeline depth). The C model uses the same lag, so the streams match.

## Verification

`encoder_axi_top_rc_tb.vhd` drives the AXI top the way the host will: per
frame, CONFIG and the four RC registers from `params.txt` (the values the C
reference's frame-level controller produced, dumped with `DCC_DUMP_RC`),
START, pixels in, payload compared byte for byte with the C reference's
(`DCC_DUMP_SLICE_SEQ`), then BYTES, CYCLES and RC_WTOTAL read back
(RC_WTOTAL checked against the stop-bit position of the payload). Vectors:
`make rc_vectors` (foreman CIF, 3 intra frames, 3 Mbps, QP 16..40; the
first frame runs on the uniform expectation and swings QP 24..32, frames 2
and 3 use the map).

| Run                                   | Frames | Result                      |
|---------------------------------------|--------|-----------------------------|
| per-MB on, lag 2, DMA stalls          | 3      | bit-exact, RC_WTOTAL exact  |
| per-MB on, lag 2, tidy                | 3      | bit-exact                   |
| per-MB on, lag 1 and lag 3, both      | 12     | bit-exact                   |
| per-MB off (RC_EN = 0), both          | 6      | bit-exact with `--rc-frame-only` |
| `encoder_axi_top_tb` (1.2 regression) | 2      | bit-exact, unchanged stream |

"DMA stalls" is the testbench's adversarial mode: input and output stalls
of up to 128 cycles from an LFSR, which is why those runs sit at 466
cycles/MB instead of 342.

## Latency and throughput

Cycles per MB, 3 CIF frames, N_ENGINES = 2:

| Backpressure | off   | lag 2 | lag 3 | lag 1 |
|--------------|-------|-------|-------|-------|
| tidy         | 342.9 | 342.9 (+0.05) | 342.9 (+0.06) | 343.4 (+0.5) |
| DMA stalls   | 466.4 | 467.6 (+1.2)  | 467.5 (+1.1)  | 467.5 (+1.1) |

Cycles the front sequencer spent waiting for a QP (decider idle, no QP yet),
per 396-MB frame:

| Backpressure | lag 2            | lag 3      | lag 1                 |
|--------------|------------------|------------|-----------------------|
| tidy         | 0 / 0 / 0        | 0 / 0 / 0  | 110 / 215 / 241       |
| DMA stalls   | 4228 / 6931 / 8592 | 0 / 135 / 0 | 13817 / 18285 / 19726 |

So at lag 2 the path is free when the DMA keeps up, and under DMA stalls
the waits it does take (11 to 22 cycles/MB) almost all overlap stalls the
pipeline was already taking: the net cost is 1.2 cycles/MB, 0.26 percent.
Lag 3 buys nothing; lag 1 costs half a cycle/MB tidy and is otherwise
absorbed the same way. Fixed costs: one marker cycle per MB in the back
sequencer, and the engine's ~30 cycles per marker, which never reach the
front sequencer at lag 2.

There is a second-order throughput effect the testbench does not show:
CAVLC time rises below QP 22 (QP 14 was +16 percent on hardware), and the
per-MB QP can now dip to qp_min inside a frame. qp_min is the throughput
guard; keep it at 18 or above at 1080p.

## Logic and timing

Out of context, xc7z030sbg485-3 at 200 MHz, placed and routed, same
script (`make impl_ooc`), MAX_W 1920, N_ENGINES 2:

| Resource       | 1.2 kernel | with per-MB RC | delta  | of which rc_mb_engine |
|----------------|-----------:|---------------:|-------:|----------------------:|
| LUTs           | 23355      | 24074          | +719   | 841 (the rest is noise: the decider moved -86) |
| Registers      | 14545      | 15272          | +727   | 510                   |
| Block RAM      | 29.5       | 33.5           | +4 RAMB36 | 4 (8192 x 16 map)  |
| DSP            | 49         | 51             | +2     | 2 (the shared multiplier) |
| WNS            | +0.019 ns  | +0.073 ns      |        | worst path now in the engine |

Per block: `mb_header_engine` 140 -> 166 LUTs (the se(v) field and QP_Y,PRED),
`cavlc_dispatch` 2194 -> 2195. The first cut of the engine missed timing
by 0.210 ns on the final QP step (sum, clamp and compare in one cycle);
registering the clamped target fixed it. The map could be halved to 2
RAMB36 by storing 8-bit-shifted weights, not done.

Zybo Z7-20 (xc7z020clg400-1) at 100 MHz, the board build's clock:

| Resource   | 1.2 kernel | with per-MB RC | delta |
|------------|-----------:|---------------:|------:|
| LUTs       | 23208 | 23936 | +728 (+3.1%) |
| Registers  | 14320 | 15001 | +681 |
| Block RAM  | 29.5 | 33.5 | +4 |
| DSP        | 49 | 51 | +2 |
| WNS        | +0.173 ns | +0.191 ns | |

The in-system build (DMA, interconnect) was 49 percent of the Z7-20's LUTs
before this; the addition is 1.4 percent of the part.

## Compression

The integer model against the floating-point controller it replaces
(C reference, same sequences as the earlier doc):

| Case                                  | float per-MB     | hardware model (lag 2) |
|---------------------------------------|------------------|------------------------|
| foreman CIF 1 Mbps, IPPP              | -3.2%, 37.14 dB, 12.7% frame err | -1.5%, 37.21 dB, 11.3% |
| foreman CIF intra only 3 Mbps, QP 16..40 | -4.6%, 38.36 dB, 5.0%        | -2.2%, 38.54 dB, 3.3%  |

The lag costs nothing measurable and the exact thresholds are slightly
better than the polynomial log2 they replace. Against frame-level control
the numbers are those of the earlier doc: per-frame error down three to
five times, bucket held, rate within 4 percent.

## To use it from the host

Per frame: run the frame-level controller as `encode_frame_h264_ext` does
(bucket, frame model, qp_frame, frame_target); `w_total` = RC_WTOTAL of the
previous frame if it had the same MB count, else the MB count with
MAP_VALID clear; write CONFIG.qp = qp_frame, RC_CTRL, RC_TARGET,
RC_SCALE = `h264_rc_scale(target, w_total)`; START. The slice header's
`slice_qp_delta` is qp_frame - pic_init_qp as before; `mb_qp_delta` is in
the payload.
