# Drone Video Compression — H.264 Reference Encoder

A from-scratch H.264 baseline-profile **intra-only** encoder in plain C, written
to be a faithful reference model for an FPGA hardware implementation
targeting drone telemetry compression on a Xilinx Zynq-7030 (xc7z030).

The output bitstream is **byte-exact decodable by ffmpeg** at all practical QPs
(10–51) — verified across 4K drone photography and the Kodak image set.

## Status

| Test set | Resolution | QPs validated | Result |
|---|---|---|---|
| 4K drone images (5 photos) | up to 3840×2576 | 18, 22, 26, 30, 38 | **25/25 byte-exact** |
| Kodak set (24 images) | 768×512 | 18, 22, 26, 30, 38, 46 | **144/144 byte-exact** |
| Synthetic complex content | 16×16 to 768×128 | all 10–51 | byte-exact |

"Byte-exact" means: ffmpeg's decoded YUV output matches our internal
reconstruction byte-for-byte.

### Sample output (cappadocia 3840×2560)

| QP | PSNR_Y | bpp | bytes (vs 14.7 MB raw) | compression |
|---|---|---|---|---|
| 10 | 51.29 dB | 5.12 | 6.30 MB | 2.3× |
| 18 | 44.23 dB | 3.31 | 4.06 MB | 3.6× |
| 22 | 40.69 dB | 2.54 | 3.12 MB | 4.7× |
| 26 | 37.01 dB | 1.81 | 2.23 MB | 6.6× |
| 30 | 33.60 dB | 1.22 | 1.50 MB | 9.8× |
| 38 | 28.17 dB | 0.46 | 568 kB | 25.9× |
| 46 | 24.22 dB | 0.16 | 198 kB | 74.3× |
| 51 | 22.30 dB | 0.08 | 101 kB | 146× |

## What's implemented

- **NAL/RBSP framing** — Annex B start codes, emulation-prevention bytes
- **Sequence/Picture Parameter Sets** — Baseline profile, level 4.0/5.0/5.2 auto-pick
- **Slice header** — IDR, slice_type=7 (I-only), deblocking-disabled signaling
- **Macroblock layer** — I_16x16 luma + Intra_8x8 chroma (4:2:0)
- **4×4 integer transform** + Hadamard for DC blocks
- **Quantization** — full intra/inter rounding offsets, per-position MF tables
- **CAVLC entropy coding** — coeff_token, level_prefix/suffix, total_zeros,
  run_before; all tables verified against x264 reference
- **Intra prediction** — I_16x16 (V/H/DC/Plane) with availability fallbacks
- **Chroma 8×8 intra** prediction (4 modes)
- **Reconstruction loop** with neighbor-sample buffers for prediction

## Not implemented (deferred)

- I_4x4 luma intra (mb_type=0) — encoder picks I_16x16 only
- In-loop deblocking filter — disabled in stream via PPS
- B/P slices, motion estimation
- CABAC

## Layout

```
src/
  main.c          CLI driver
  encoder.c       per-MB encode loop, mode decision, recon
  cavlc.c         CAVLC encode/decode (the entropy coder)
  cavlc_tables.h  spec Tables 9-5/9-7/9-8/9-9/9-10 (verified vs x264)
  intra.c         I_16x16 + chroma 8x8 prediction
  quant.c         forward/inverse quant for AC, DC luma, DC chroma
  transform.c     4x4 integer DCT + Hadamard butterflies
  bitstream.c     bit-level writer/reader
  nal.c           SPS/PPS/IDR NAL framing
tests/            Unit tests for transform/quant/CAVLC round-trip
tools/
  validate.py             dataset PSNR/BPP sweep
  validate_byteexact.sh   ffmpeg byte-exact verification
  diff_x264.py            compare our CAVLC tables vs x264 reference
  check_ct.py             compare our coeff_token vs spec image
  check_prefix.py         verify all VLC tables are prefix-free
  parse_cavlc_tables.py   auto-generate tables from spec text dump
  make_test_frame.py      synthesize NV12 test frames
9-*.png                   spec Tables 9-5/9-7/9-8/9-9/9-10 (image references)
architecture.txt          hardware-side architecture notes
Makefile                  builds build/dcc_encoder
```

## Build

Linux/macOS or MSYS2 on Windows:

```sh
make
```

Produces `build/dcc_encoder`.

Requirements: gcc/clang with C99, `make`, optionally Python 3 + ffmpeg/ffprobe
for the validation harnesses.

## Usage

```sh
# Encode a raw NV12 frame to H.264 Annex B + write recon
build/dcc_encoder <in.yuv> <width> <height> <qp> [recon.yuv [bitstream.264]]

# Examples
build/dcc_encoder pic.yuv 1920 1080 28 recon.yuv out.264
ffmpeg -i out.264 -pix_fmt nv12 -f rawvideo decoded.yuv
cmp recon.yuv decoded.yuv  # should be byte-exact

# Sweep PSNR/BPP across an image set
python tools/validate.py datasets/kodak --qp 22,28,34 --out results.csv

# Verify byte-exact ffmpeg decode across a dataset
bash tools/validate_byteexact.sh datasets/kodak 18,22,26,30,38
```

The encoder reads NV12 (luma plane followed by interleaved CbCr at half
resolution, 4:2:0). To convert from a still image:

```sh
ffmpeg -i input.png -vf 'crop=trunc(iw/16)*16:trunc(ih/16)*16:0:0' \
       -pix_fmt nv12 -f rawvideo input.yuv
```

Width and height must be multiples of 16.

## Validation

`tools/validate_byteexact.sh` runs the encoder on a dataset, decodes with
ffmpeg, and `cmp`-compares ffmpeg's YUV output against our internal
reconstruction. Any non-zero diff or ffmpeg error is a regression.

`tools/diff_x264.py` parses both our `cavlc_tables.h` and x264's
`common/tables.c` and reports any mismatched entries — the canonical way to
catch table bugs (this is how the final remaining `total_zeros[5][9..10]`
mismatch was found).

## Datasets (not in repo)

The `datasets/` directory is gitignored to keep the repo small. Populate it
with:

- **Kodak** — http://r0k.us/graphics/kodak/ (24 images, 768×512, ~15 MB total)
- **Drone samples** — any 4K aerial JPEGs

```
datasets/
  kodak/      kodim01.png .. kodim24.png
  drone_sample/  *.jpg
```

## Known limitations

- **QP < 10 with high-residual content**: at extreme low QPs the quantized AC
  level magnitudes can exceed CAVLC's Baseline profile max encodable abs
  (≈2528 at suffix_length=6). The H.264 FRExt profiles widen `level_suffix`
  to handle this; we don't yet. Not a problem at any practical operating QP
  since QP<10 produces files larger than the raw input.
- **Mode set**: only I_16x16 + Intra_8x8_chroma. I_4x4 is implemented but
  estimate-only — not in the bitstream-emitting path.
- **No deblocking filter**: signaled disabled in PPS; bitstream remains spec-
  compliant.

## References

- ITU-T Rec. H.264 (05/2003), §7 (syntax) and §9 (parsing/CAVLC)
- ISO/IEC 14496-10
- x264 reference encoder — `common/vlc.c`, `common/tables.c`
- JM reference software — https://iphome.hhi.de/suehring/tml/

## License

TBD.
