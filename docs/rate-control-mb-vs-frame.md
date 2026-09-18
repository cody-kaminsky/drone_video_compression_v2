# Rate control: what the per-MB QP correction is worth

The RTL kernel latches one QP per frame at START, so frame-level rate
control needs no RTL change: the host writes CONFIG before each START.
The per-MB correction in the C reference (`encode_cfg_t.rc_mb`) needs the
kernel's MB layer to emit a nonzero `mb_qp_delta`, plus a bit counter and a
per-MB bit map from the previous frame. This measures whether that is
worth building, before building it.

Measured 2026-09-18 on `claude/p-frames` with `build/dcc_encoder`,
`tools/verify_sequence.sh` byte-exact in ffmpeg for every mode.

## The three modes

| `rc_mb` | CLI               | What it does                                                            | RTL cost                                    |
|---------|-------------------|-------------------------------------------------------------------------|---------------------------------------------|
| 0       | `--rc-frame-only` | One QP per frame from the bucket and the bits ~ C * 2^(-QP/6) model     | none (host side only)                       |
| 2       | `--rc-mb-linear`  | Per-MB QP corrected against a straight-line expectation of the spend    | bit counter, QP stepper, `mb_qp_delta`      |
| 1       | default           | Per-MB QP corrected against the previous frame's per-MB bit map          | as 2, plus one 16-bit word per MB in BRAM   |

Inside a frame the QP moves one step per MB towards the target the error
implies, within [qp_min, qp_max].

## Results

Columns: achieved rate against target; mean luma PSNR; the mean of
|frame bits - frame target| / frame target over the sequence; the largest
bucket fill in milliseconds of link; and the cumulative bits the bucket
could not absorb (`STAT RC_OVERFLOW`). A nonzero overflow means the stream
does **not** fit the link at the requested latency, whatever the PSNR says.

Bucket = 4 frame budgets (133 ms at 30 fps) unless stated. GOP 30, refresh
band 1 column, deblocking on, strict refresh.

| Case                                    | Mode  | Rate      | PSNR-Y | frame err | max bucket | overflow |
|-----------------------------------------|-------|-----------|--------|-----------|------------|----------|
| foreman CIF 300 kbps, qp-max 40         | frame | -2.5%     | 31.75  | 45.1%     | 133.3 ms   | 9.5 kbit |
|                                         | lin   | -2.1%     | 31.70  | 30.5%     | 129.5 ms   | 0        |
|                                         | MB    | -2.9%     | 31.68  | 27.8%     | 117.2 ms   | 0        |
| foreman CIF 1 Mbps                      | frame | -3.3%     | 37.16  | 34.3%     | 133.3 ms   | 21 kbit  |
|                                         | lin   | -2.5%     | 37.13  | 13.6%     | 101.3 ms   | 0        |
|                                         | MB    | -3.2%     | 37.14  | 12.7%     | 101.4 ms   | 0        |
| foreman 300 kbps, seed QP 22 (wrong)    | frame | +1.1%     | 31.84  | 53.2%     | 133.3 ms   | 116 kbit |
|                                         | lin   | -1.7%     | 31.71  | 32.4%     | 133.3 ms   | 14 kbit  |
|                                         | MB    | -2.4%     | 31.70  | 30.1%     | 133.3 ms   | 14 kbit  |
| foreman 300 kbps, bucket 1 frame        | frame | -0.5%     | 31.81  | 25.6%     | 33.3 ms    | 111 kbit |
|                                         | lin   | -1.2%     | 31.64  | 8.4%      | 33.3 ms    | 41 kbit  |
|                                         | MB    | -2.4%     | 31.59  | 8.8%      | 33.3 ms    | 40 kbit  |
| 1080p zoom, 30 frames, 10 Mbps          | frame | -5.7%     | 39.38  | 38.3%     | 133.3 ms   | 387 kbit |
|                                         | lin   | -9.0%     | 38.85  | 10.9%     | 117.4 ms   | 0        |
|                                         | MB    | -3.8%     | 39.05  | 9.6%      | 117.4 ms   | 0        |
| 1080p zoom, intra only, 40 Mbps         | frame | -1.6%     | 36.47  | 8.2%      | 22.5 ms    | 0        |
|                                         | lin   | -13.1%    | 35.60  | 13.7%     | 4.3 ms     | 0        |
|                                         | MB    | -3.7%     | 36.33  | 5.9%      | 5.8 ms     | 0        |
| 1080p pan, 8 frames, 10 Mbps            | frame | +7.6%     | 38.91  | 55.6%     | 113.1 ms   | 0        |
|                                         | lin   | +4.6%     | 38.65  | 17.7%     | 125.8 ms   | 0        |
|                                         | MB    | -0.0%     | 38.67  | 7.8%      | 125.8 ms   | 0        |

## What it says

**Frame-level control alone does not hold the bucket.** Its per-frame error
is 35 to 55 percent on the P-frame cases, it overflows the receiver buffer
in five of the eight runs, and on the 8-frame pan it is 7.6 percent over
the rate with nothing to repay it. The frame model is fitted on one
previous frame, and one frame is not enough to predict the next within the
tolerance a 133 ms bucket leaves.

**Its higher PSNR is not real.** On the zoom the frame-only IDR spends 1.54
times its target (257 kB against 188 kB), which is exactly the 387 kbit the
bucket could not absorb, and every P frame after it inherits that better
reference. The per-MB controller held the IDR to 1.13 times target and
took 2.4 dB less on that frame. Spend the same bits and the difference
goes away; the bits are not there to spend.

**The per-MB correction cuts the per-frame error by three to five times**
(9.6 vs 38.3 percent on the zoom, 7.8 vs 55.6 on the pan, 12.7 vs 34.3 on
foreman at 1 Mbps), keeps the bucket below capacity on every case where it
can be kept (the 1-frame-bucket and wrong-seed foreman cases overflow on
the IDR, which no QP in range can shrink enough), and lands within 4
percent of the rate everywhere.

**The straight-line expectation is not a substitute for the bit map.** It
matches full per-MB control on foreman, whose complexity is spread
evenly, and fails on the 1080p zoom: 9 percent under the rate on IPPP and
13 percent under on intra only, with 0.7 dB lost. The zoom's detail is
concentrated in the middle of the picture, so a straight line predicts
too much spend in the flat top rows, the controller raises QP there, and
the one-step-per-MB slew cannot give it back in time. The previous frame's
map knows where the bits go. One 16-bit word per MB is 8160 words at
1080p, which is four 18 kbit BRAMs on a 7-series part, and it is the
cheapest thing in this table.

## For the RTL

Build the per-MB path with the bit map, not the linear variant. What it
needs in the kernel:

- a per-frame `qp_frame` and `frame_target` from the host as now (the frame
  model and the bucket stay on the host; they run once per frame);
- a running bit count of the slice payload, which the bit packer already
  has;
- the previous frame's per-MB bit map (write on every MB, read one ahead
  in raster order, so a simple dual-port BRAM), plus its running prefix
  sum against the total, which is one accumulator;
- the QP step: `adj = round(6 log2(spent / expect)) + floor(4 (spent -
  expect) / target)`, then one step of `qp_mb` towards `qp_frame + adj`
  clamped to [qp_min, qp_max]. The log2 is of a ratio near 1 and only its
  integer-ish result matters, so a small lookup on the leading bits of the
  quotient will do; it does not need a divider if `expect` is tracked as
  a threshold the counter is compared against;
- `mb_qp_delta = qp_mb - qp_prev` on MBs that code residual, and
  `qp_prev` updated only then (spec 7.4.5: QP_Y,PRED is the QP of the
  previous MB in decoding order, and an MB without residual carries the
  predicted QP forward unchanged);
- the chroma QP table indexed per MB instead of per frame; the deblocking
  filter, when it reaches the RTL, averages the QPs of the two MBs across
  the edge (8.7.2.2), which the C reference already does.

The reference implementation is the MB loop in `encode_frame_h264_ext`
(`src/encoder.c`) and the `rc_mb` field in `src/encoder.h`.
