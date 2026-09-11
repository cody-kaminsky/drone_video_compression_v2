#!/bin/bash
# Encode an NV12 sequence with the C reference (IDR + P frames, rolling intra
# refresh) and check that ffmpeg's decode of the stream equals the encoder's
# own reconstruction byte for byte, frame by frame.
# Usage: verify_sequence.sh <in.yuv> <width> <height> <qp> [encoder options...]
set -u
IN="$1"; W="$2"; H="$3"; QP="$4"; shift 4
ENC="${ENCODER:-build/dcc_encoder.exe}"
OUT="${OUT:-build/seq}"
mkdir -p "$OUT"
bn=$(basename "$IN" .yuv)
"$ENC" "$IN" "$W" "$H" "$QP" "$OUT/${bn}_rec.yuv" "$OUT/${bn}.264" "$@" > "$OUT/${bn}.log" || { echo "encode failed"; cat "$OUT/${bn}.log"; exit 1; }
grep -E "STAT FRAME [0-9]" "$OUT/${bn}.log" | awk '{printf "  frame %s %s bytes=%s psnr_y=%s intra=%s inter=%s skip=%s\n", $3, $5, $7, $9, $15, $17, $19}'
grep -E "STAT (FRAMES|BYTES_OUT|PSNR_Y|MBS)" "$OUT/${bn}.log" | sed 's/^/  /'
ffmpeg -y -loglevel error -i "$OUT/${bn}.264" -f rawvideo -pix_fmt nv12 "$OUT/${bn}_dec.yuv" || { echo "ffmpeg decode FAILED"; exit 1; }
fsz=$((W * H * 3 / 2))
n=$(( $(stat -c %s "$OUT/${bn}_rec.yuv") / fsz ))
nd=$(( $(stat -c %s "$OUT/${bn}_dec.yuv") / fsz ))
if [ "$n" -ne "$nd" ]; then echo "FAIL: ffmpeg decoded $nd frames, encoder produced $n"; exit 1; fi
if cmp -s "$OUT/${bn}_rec.yuv" "$OUT/${bn}_dec.yuv"; then
  echo "PASS: $n frames, ffmpeg decode byte-exact with the encoder reconstruction"
else
  first=$(cmp "$OUT/${bn}_rec.yuv" "$OUT/${bn}_dec.yuv" | grep -o "byte [0-9]*" | grep -o "[0-9]*")
  echo "FAIL: first difference at byte $first = frame $(( (first - 1) / fsz ))"
  exit 1
fi
