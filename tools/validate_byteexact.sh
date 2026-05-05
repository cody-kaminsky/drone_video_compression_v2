#!/bin/bash
# Validate byte-exact ffmpeg decode against our internal recon for a dataset.
# Usage: validate_byteexact.sh <image_dir> [qp1,qp2,...] [binary]
#   binary defaults to build/dcc_encoder.exe; pass build/dcc_hls.exe to
#   test the HLS port.
set -e
DIR="$1"
QPS="${2:-18,22,26,30,38}"
ENCODER="${3:-build/dcc_encoder.exe}"
PATTERN="${4:-*.png *.jpg *.jpeg}"

PASS_TOTAL=0
FAIL_TOTAL=0

for img in $DIR/*.png $DIR/*.jpg $DIR/*.jpeg; do
  [ -f "$img" ] || continue
  bn=$(basename "$img")
  bn_noext="${bn%.*}"

  dims=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=p=0 "$img" 2>/dev/null)
  iw=$(echo "$dims" | cut -d, -f1)
  ih=$(echo "$dims" | cut -d, -f2)
  w=$((iw / 16 * 16))
  h=$((ih / 16 * 16))

  if [ "$w" -lt 16 ] || [ "$h" -lt 16 ]; then
    echo "SKIP $bn: too small ($iw x $ih)"
    continue
  fi

  yuv="build/v_${bn_noext}.yuv"
  ffmpeg -y -loglevel error -i "$img" -vf "crop=${w}:${h}:0:0" -pix_fmt nv12 -f rawvideo "$yuv" > /dev/null 2>&1

  printf "%-30s %dx%d  " "$bn" "$w" "$h"

  IFS=',' read -ra QP_ARR <<< "$QPS"
  for qp in "${QP_ARR[@]}"; do
    enc_log=$("$ENCODER" "$yuv" "$w" "$h" "$qp" "build/v_${bn_noext}_recon.yuv" "build/v_${bn_noext}.264" 2>&1)
    psnr_y=$(echo "$enc_log" | grep PSNR_Y | awk '{print $3}')
    bpp=$(echo "$enc_log" | grep "STAT BPP" | awk '{print $3}')

    ffmpeg -y -loglevel error -i "build/v_${bn_noext}.264" -pix_fmt nv12 -f rawvideo "build/v_${bn_noext}_ff.yuv" > /tmp/ff.log 2>&1

    diffs=$(cmp -l "build/v_${bn_noext}_recon.yuv" "build/v_${bn_noext}_ff.yuv" 2>/dev/null | wc -l)
    ff_errs=$(grep -cE "error|invalid" /tmp/ff.log 2>/dev/null || true)

    if [ "$diffs" = "0" ] && [ "$ff_errs" = "0" ]; then
      printf "QP%s:OK(%s) " "$qp" "$psnr_y"
      PASS_TOTAL=$((PASS_TOTAL+1))
    else
      printf "QP%s:FAIL(%s/%s) " "$qp" "$diffs" "$ff_errs"
      FAIL_TOTAL=$((FAIL_TOTAL+1))
    fi

    rm -f "build/v_${bn_noext}.264" "build/v_${bn_noext}_recon.yuv" "build/v_${bn_noext}_ff.yuv"
  done
  echo ""
  rm -f "$yuv"
done
echo ""
echo "TOTAL: $PASS_TOTAL pass, $FAIL_TOTAL fail"
