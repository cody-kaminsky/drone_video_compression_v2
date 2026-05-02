#!/usr/bin/env python3
"""Synthesize a small NV12 test frame for smoke-testing the encoder.

Usage: make_test_frame.py <out.yuv> <width> <height>

Generates a 2D gradient luma plane and a uniform chroma plane.
"""
import sys
from pathlib import Path

def main():
    if len(sys.argv) != 4:
        sys.exit("usage: make_test_frame.py <out.yuv> <width> <height>")
    out_path = Path(sys.argv[1])
    w = int(sys.argv[2])
    h = int(sys.argv[3])
    if w % 16 or h % 16:
        sys.exit("width and height must be multiples of 16")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        # Luma: smooth diagonal gradient with light noise pattern.
        y_buf = bytearray(w * h)
        for i in range(h):
            for j in range(w):
                v = (i * 256 // h + j * 256 // w) // 2
                # add a subtle 8x8 checkerboard so encoder has detail to work on
                if ((i // 8) + (j // 8)) & 1:
                    v += 8
                y_buf[i * w + j] = max(0, min(255, v))
        f.write(bytes(y_buf))

        # Chroma (NV12): half-height interleaved CbCr; uniform 128 = neutral.
        uv_rows = h // 2
        uv_buf = bytearray(uv_rows * w)
        for i in range(uv_rows):
            for j in range(w // 2):
                # Slight chroma gradient on Cb, neutral Cr.
                cb = 128 + (j - w // 4) * 32 // w
                cr = 128
                uv_buf[i * w + j*2 + 0] = max(0, min(255, cb))
                uv_buf[i * w + j*2 + 1] = cr
        f.write(bytes(uv_buf))

    print(f"wrote {out_path} ({w}x{h} NV12, {w*h*3//2} bytes)")

if __name__ == "__main__":
    main()
