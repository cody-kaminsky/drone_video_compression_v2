#!/usr/bin/env python3
"""Validate the codec across a dataset of images.

For each image in <dataset_dir>:
  1. Convert to NV12 YUV420 with ffmpeg (cropped to multiple of 16).
  2. Run the encoder at the requested QP.
  3. Parse the encoder's STAT lines for PSNR / BPP / total_bits.
  4. Append a CSV row.

Optionally sweeps QP across a list (e.g. --qp 22,28,34,40) and produces
one row per (image, QP) pair — handy for rate-distortion plots.

Requirements: ffmpeg + ffprobe in PATH, Python 3.7+.

Usage:
  python tools/validate.py <dataset_dir> [options]

Options:
  --qp 22,28,34,40        QPs to sweep (default: 30)
  --out results.csv       output file (default: results.csv)
  --encoder build/dcc_encoder   path to the C encoder binary
  --max-images N          limit to N images
  --pattern '*.png'       glob pattern (can repeat)
"""
import argparse
import csv
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

STAT_RE = re.compile(r'^STAT\s+(\w+):\s+(.+)$')

def have_tool(name: str) -> bool:
    from shutil import which
    return which(name) is not None

def get_dims(img_path: Path) -> tuple[int, int]:
    """Return (width, height) using ffprobe."""
    out = subprocess.run(
        ["ffprobe", "-v", "error",
         "-select_streams", "v:0",
         "-show_entries", "stream=width,height",
         "-of", "csv=p=0", str(img_path)],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    parts = out.split(',')
    return int(parts[0]), int(parts[1])

def to_yuv(img_path: Path, yuv_path: Path, w: int, h: int) -> None:
    """Convert image to NV12 YUV420 of given dimensions (crop to top-left)."""
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-i", str(img_path),
        "-vf", f"crop={w}:{h}:0:0",
        "-pix_fmt", "nv12",
        "-f", "rawvideo",
        str(yuv_path),
    ]
    subprocess.run(cmd, check=True)

def parse_stats(stdout: str) -> dict:
    out = {}
    for line in stdout.splitlines():
        m = STAT_RE.match(line)
        if not m:
            continue
        k, v = m.group(1), m.group(2).strip()
        try:
            if '.' in v:
                out[k] = float(v)
            else:
                out[k] = int(v)
        except ValueError:
            out[k] = v
    return out

def encode_one(encoder: Path, yuv: Path, w: int, h: int, qp: int) -> dict:
    cmd = [str(encoder), str(yuv), str(w), str(h), str(qp)]
    res = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return parse_stats(res.stdout)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset_dir")
    ap.add_argument("--qp", default="30",
                    help="QP value or comma-separated list (e.g. 22,28,34,40)")
    ap.add_argument("--out", default="results.csv")
    ap.add_argument("--encoder", default="build/dcc_encoder",
                    help="path to the C encoder binary")
    ap.add_argument("--max-images", type=int, default=0,
                    help="limit number of images (0 = no limit)")
    ap.add_argument("--pattern", action="append", default=None,
                    help="glob pattern (default: *.png, *.jpg, *.jpeg, *.bmp)")
    args = ap.parse_args()

    if not have_tool("ffmpeg") or not have_tool("ffprobe"):
        sys.exit("error: ffmpeg and ffprobe must be in PATH")

    encoder = Path(args.encoder)
    if not encoder.exists():
        sys.exit(f"error: encoder not found: {encoder}\n"
                 "build it first with `make`")

    dataset = Path(args.dataset_dir)
    if not dataset.is_dir():
        sys.exit(f"error: dataset_dir not found: {dataset}")

    patterns = args.pattern or ["*.png", "*.jpg", "*.jpeg", "*.bmp"]
    images: list[Path] = []
    for pat in patterns:
        images.extend(sorted(dataset.glob(pat)))
    if args.max_images > 0:
        images = images[: args.max_images]
    if not images:
        sys.exit(f"error: no images matched in {dataset}")

    qps = [int(q) for q in args.qp.split(',')]

    fields = [
        "image", "width", "height", "qp",
        "psnr_y", "psnr_u", "psnr_v", "psnr_avg",
        "bpp", "total_bits", "bytes_out",
        "compressed_kbps_at_30fps",
    ]
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(out_path, "w", newline="") as cf:
        writer = csv.DictWriter(cf, fieldnames=fields)
        writer.writeheader()

        rows_written = 0
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            for img in images:
                try:
                    iw, ih = get_dims(img)
                except subprocess.CalledProcessError:
                    print(f"skip {img.name}: ffprobe failed", file=sys.stderr)
                    continue

                w = (iw // 16) * 16
                h = (ih // 16) * 16
                if w < 16 or h < 16:
                    print(f"skip {img.name}: too small after crop ({iw}x{ih})",
                          file=sys.stderr)
                    continue

                yuv = tmp / (img.stem + ".yuv")
                try:
                    to_yuv(img, yuv, w, h)
                except subprocess.CalledProcessError as e:
                    print(f"skip {img.name}: ffmpeg failed ({e})", file=sys.stderr)
                    continue

                for qp in qps:
                    try:
                        stats = encode_one(encoder, yuv, w, h, qp)
                    except subprocess.CalledProcessError as e:
                        print(f"  qp={qp}: encoder failed ({e})", file=sys.stderr)
                        continue

                    bpp        = stats.get("BPP", 0.0)
                    total_bits = stats.get("TOTAL_BITS", 0)
                    kbps_30    = total_bits * 30 / 1000.0

                    row = {
                        "image":     img.name,
                        "width":     w,
                        "height":    h,
                        "qp":        qp,
                        "psnr_y":    stats.get("PSNR_Y", 0.0),
                        "psnr_u":    stats.get("PSNR_U", 0.0),
                        "psnr_v":    stats.get("PSNR_V", 0.0),
                        "psnr_avg":  stats.get("PSNR_AVG", 0.0),
                        "bpp":       bpp,
                        "total_bits": total_bits,
                        "bytes_out": stats.get("BYTES_OUT", 0),
                        "compressed_kbps_at_30fps": round(kbps_30, 2),
                    }
                    writer.writerow(row)
                    cf.flush()
                    rows_written += 1
                    print(f"{img.name}  {w}x{h}  qp={qp}: "
                          f"psnr_y={row['psnr_y']:.2f} dB  "
                          f"bpp={bpp:.4f}  "
                          f"@30fps={kbps_30/1000:.2f} Mbps")

                yuv.unlink(missing_ok=True)

    # Summary by QP.
    print()
    print(f"wrote {rows_written} rows to {out_path}")
    print()
    print("=== summary by QP ===")
    import collections
    agg = collections.defaultdict(list)
    with open(out_path) as cf:
        reader = csv.DictReader(cf)
        for row in reader:
            agg[int(row["qp"])].append((float(row["psnr_y"]),
                                        float(row["bpp"]),
                                        float(row["compressed_kbps_at_30fps"])))
    print(f"{'QP':>4}  {'N':>4}  {'avg_PSNR_Y':>11}  {'avg_BPP':>9}  {'avg_kbps@30':>13}")
    for qp in sorted(agg):
        rows = agg[qp]
        n = len(rows)
        ay = sum(r[0] for r in rows) / n
        ab = sum(r[1] for r in rows) / n
        ak = sum(r[2] for r in rows) / n
        print(f"{qp:>4}  {n:>4}  {ay:>11.2f}  {ab:>9.4f}  {ak:>13.1f}")

if __name__ == "__main__":
    main()
