#!/usr/bin/env python3
"""Write the frame_io input stream for an NV12 frame: per MB row, 16 luma
lines then 8 chroma lines, 4 bytes per beat, one beat per line as an
8-digit hex 32-bit little-endian word (byte 0 in bits 7:0).
Usage: gen_frame_stream.py <in.yuv> <width> <height> <out.txt> [frame]"""
import sys
path, w, h, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
frame = int(sys.argv[5]) if len(sys.argv) > 5 else 0
fsz = w * h * 3 // 2
data = open(path, "rb").read()[frame * fsz : (frame + 1) * fsz]
y = data[: w * h]
uv = data[w * h : w * h + w * (h // 2)]
with open(out, "w") as f:
    for mbr in range(h // 16):
        for line in range(16):
            row = y[(mbr * 16 + line) * w : (mbr * 16 + line + 1) * w]
            for x in range(0, w, 4):
                f.write("%08X\n" % int.from_bytes(row[x : x + 4], "little"))
        for line in range(8):
            row = uv[(mbr * 8 + line) * w : (mbr * 8 + line + 1) * w]
            for x in range(0, w, 4):
                f.write("%08X\n" % int.from_bytes(row[x : x + 4], "little"))
print("wrote", out)
