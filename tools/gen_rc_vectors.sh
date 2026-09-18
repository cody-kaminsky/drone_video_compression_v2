#!/bin/bash
# Vectors for encoder_axi_top_rc_tb: a short intra-only sequence coded by the
# C reference under rate control, with the per-frame register values the host
# would write, the pixel stream of every frame and the golden payload of
# every frame.
#
# Usage: gen_rc_vectors.sh <in.yuv> <width> <height> <frames> <outdir> <hw|off> [encoder options]
#   hw   the hardware model (--rc-hw): per-MB QP in the kernel
#   off  frame QP only (--rc-frame-only): the same frames, RC_EN clear
# The testbench reads <outdir>/config.txt (written here) to find the rest.
set -eu
IN="$1"; W="$2"; H="$3"; N="$4"; OUT="$5"; MODE="$6"; shift 6
ENC="${ENCODER:-build/dcc_encoder.exe}"
mkdir -p "$OUT"
rm -f "$OUT"/payload*.txt "$OUT"/stream*.txt "$OUT"/params.txt "$OUT"/mbqp.txt
fsz=$((W * H * 3 / 2))
head -c $((N * fsz)) "$IN" > "$OUT/src.yuv"
case "$MODE" in
  hw)  rcopt="--rc-hw" ;;
  off) rcopt="--rc-frame-only" ;;
  *)   echo "mode must be hw or off"; exit 1 ;;
esac
DCC_DUMP_SLICE_SEQ="$OUT/payload" DCC_DUMP_RC="$OUT/params.txt" DCC_DUMP_MBQP="$OUT/mbqp.txt" \
  "$ENC" "$OUT/src.yuv" "$W" "$H" 26 "$OUT/rec.yuv" "$OUT/seq.264" --intra-only --no-deblock $rcopt "$@" > "$OUT/enc.log"
for ((k = 0; k < N; k++)); do
  python tools/gen_frame_stream.py "$OUT/src.yuv" "$W" "$H" "$OUT/stream$k.txt" "$k" > /dev/null
done
printf '%s\n%d\n%d\n%d\n' "$OUT/" "$N" $((W / 16)) $((H / 16)) > "$OUT/config.txt"
grep -E "STAT FRAME|STAT RC" "$OUT/enc.log" | sed 's/^/  /'
echo "params ($OUT/params.txt): qp_frame target scale qp_min qp_max map_valid lag en"
cat "$OUT/params.txt" | sed 's/^/  /'
ffmpeg -y -loglevel error -i "$OUT/seq.264" -f rawvideo -pix_fmt nv12 "$OUT/dec.yuv"
if cmp -s "$OUT/rec.yuv" "$OUT/dec.yuv"; then echo "ffmpeg decode byte-exact"; else echo "ffmpeg decode DIFFERS"; exit 1; fi
