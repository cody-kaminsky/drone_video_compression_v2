#!/usr/bin/env python3
"""gen_board_sequence.py — build a board test sequence.

Produces, for a set of (frame, QP) pairs:

  frame_NNN.bin          each distinct source frame, once, in stream order
  golden_fNNN_qQQ.bin    the payload the C reference says the kernel must emit
  manifest.bin           geometry, repeats, and one record per pair
  addrs.tcl              data-only loader, sourced by scripts/run_board.tcl
  load.tcl / load.bat    standalone loader, for an already-running application

Frames are pre-shuffled here rather than staged on the board, so the board
does no copying: MM2S reads each frame where it lies.

QP lives in the record, not the manifest, so one sequence can sweep QP with a
single load. Frame data does not depend on QP, so a sweep stores each frame
once and one golden per pair -- eight QPs of a 480x272 frame cost 196 kB of
frame plus a few small goldens.

Because the encoder is intra only, frames are independent: the kernel holds no
state across them. A short sequence cycled many times therefore exercises
restart and sustained throughput as well as a long one, and loads in seconds.

Usage:
  gen_board_sequence.py <in.yuv> <width> <height> <qp> [options]
    --frames N      frames to take from the file (default: all)
    --qp-sweep LIST comma-separated QPs; every frame is encoded at each
    --repeats R     times the board should cycle the sequence (default 1)
    --out DIR       output directory (default build/seq_board)
    --aclk HZ       PL clock, written into the manifest (default 100000000)
    --xsdb PATH     xsdb.bat, for the generated .bat wrapper
"""

import argparse
import os
import struct
import subprocess
import sys

# Must match host/dcc_memmap.h.
MANIFEST_ADDR = 0x03000000
FRAMES_ADDR   = 0x04000000
GOLDEN_ADDR   = 0x20000000
PAYLOAD_ADDR  = 0x30000000
FRAMES_ROOM   = GOLDEN_ADDR - FRAMES_ADDR
GOLDEN_ROOM   = PAYLOAD_ADDR - GOLDEN_ADDR
MAGIC         = 0x4D434344
VERSION       = 2

DEFAULT_XSDB = os.environ.get(
    "XSDB", "C:/AMDDesignTools/2025.2/Vitis/bin/xsdb.bat")

ENCODER = os.path.join("build", "dcc_encoder.exe")
SHUFFLE = os.path.join("build", "gen_stream_frame.exe")


def align(x, a=64):
    return (x + a - 1) // a * a


def mod8_of(payload):
    """(macroblock-layer bits + stop bit) mod 8. Zero is the alignment that
    used to lose the frame's final byte; it happens on one frame in eight."""
    last = len(payload) - 1
    while last >= 0 and payload[last] == 0:
        last -= 1
    if last < 0:
        return -1
    tz, v = 0, payload[last]
    while not v & 1:
        v >>= 1
        tz += 1
    return ((last + 1) * 8 - tz) % 8


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yuv")
    ap.add_argument("width", type=int)
    ap.add_argument("height", type=int)
    ap.add_argument("qp", type=int)
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--qp-sweep", default="")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--out", default=os.path.join("build", "seq_board"))
    ap.add_argument("--aclk", type=int, default=100_000_000)
    ap.add_argument("--xsdb", default=DEFAULT_XSDB)
    args = ap.parse_args()

    # Absolute: on this Python build CreateProcess refuses a relative path with
    # forward slashes even when the file is plainly there.
    enc = os.path.abspath(ENCODER if os.path.isfile(ENCODER) else ENCODER[:-4])
    shf = os.path.abspath(SHUFFLE if os.path.isfile(SHUFFLE) else SHUFFLE[:-4])
    for tool, how in ((enc, "make ref"), (shf, "make board_seq_tools")):
        if not os.path.isfile(tool):
            sys.exit("missing %s -- run `%s`" % (tool, how))

    if args.width % 16 or args.height % 16:
        sys.exit("width and height must be multiples of 16")

    qps = ([int(q) for q in args.qp_sweep.split(",") if q.strip()]
           if args.qp_sweep else [args.qp])
    for q in qps:
        if not 0 <= q <= 51:
            sys.exit("QP %d out of range" % q)

    frame_bytes = args.width * args.height * 3 // 2
    avail = os.path.getsize(args.yuv) // frame_bytes
    if avail == 0:
        sys.exit("%s is smaller than one %dx%d frame (%d bytes)"
                 % (args.yuv, args.width, args.height, frame_bytes))
    n_src = avail if args.frames <= 0 else min(args.frames, avail)

    os.makedirs(args.out, exist_ok=True)
    frame_stride = align(frame_bytes)
    if n_src * frame_stride > FRAMES_ROOM:
        sys.exit("%d frames need %.0f MB, layout gives %.0f MB"
                 % (n_src, n_src * frame_stride / 2**20, FRAMES_ROOM / 2**20))

    print("%dx%d, %d frame(s) x %d QP(s) = %d records, %d repeats"
          % (args.width, args.height, n_src, len(qps), n_src * len(qps),
             args.repeats))

    # ---- each distinct source frame, once, in stream order ----
    for i in range(n_src):
        fbin = os.path.join(args.out, "frame_%03d.bin" % i)
        subprocess.run([shf, args.yuv, str(args.width), str(args.height),
                        str(i), fbin], check=True)

    # ---- one golden per (frame, QP) pair ----
    tmp_yuv = os.path.join(args.out, "_one.yuv")
    gtxt = os.path.join(args.out, "_golden.txt")
    pairs = []          # (src_index, qp, golden_filename, golden_len)
    aligned = 0
    for i in range(n_src):
        with open(args.yuv, "rb") as src, open(tmp_yuv, "wb") as dst:
            src.seek(i * frame_bytes)
            dst.write(src.read(frame_bytes))
        for q in qps:
            env = dict(os.environ, DCC_DUMP_SLICE=gtxt)
            subprocess.run([enc, tmp_yuv, str(args.width), str(args.height),
                            str(q)], check=True, env=env,
                           stdout=subprocess.DEVNULL)
            with open(gtxt) as f:
                payload = bytes(int(t) for t in f.read().split())
            name = "golden_f%03d_q%02d.bin" % (i, q)
            with open(os.path.join(args.out, name), "wb") as f:
                f.write(payload)
            pairs.append((i, q, name, len(payload)))
            m = mod8_of(payload)
            if m == 0:
                aligned += 1
            print("  frame %d QP %-3d %8d bytes  mod8=%d%s"
                  % (i, q, len(payload), m,
                     "  <-- byte-aligned payload" if m == 0 else ""))

    for p in (tmp_yuv, gtxt):
        if os.path.isfile(p):
            os.remove(p)

    golden_stride = align(max(p[3] for p in pairs))
    if len(pairs) * golden_stride > GOLDEN_ROOM:
        sys.exit("goldens need %.0f MB, layout gives %.0f MB"
                 % (len(pairs) * golden_stride / 2**20, GOLDEN_ROOM / 2**20))

    # ---- manifest ----
    blob = struct.pack("<8I", MAGIC, VERSION, args.width, args.height,
                       args.qp, len(pairs), args.repeats, args.aclk)
    for k, (i, q, _, glen) in enumerate(pairs):
        blob += struct.pack("<6I",
                            FRAMES_ADDR + i * frame_stride, frame_bytes,
                            GOLDEN_ADDR + k * golden_stride, glen, q, 0)
    with open(os.path.join(args.out, "manifest.bin"), "wb") as f:
        f.write(blob)

    here = os.path.abspath(args.out).replace("\\", "/")

    def dow_lines():
        out = []
        for i in range(n_src):
            out.append("dow -data %s/frame_%03d.bin 0x%08X"
                       % (here, i, FRAMES_ADDR + i * frame_stride))
        for k, (_, _, name, _) in enumerate(pairs):
            out.append("dow -data %s/%s 0x%08X"
                       % (here, name, GOLDEN_ADDR + k * golden_stride))
        out.append("dow -data %s/manifest.bin 0x%08X" % (here, MANIFEST_ADDR))
        return out

    with open(os.path.join(args.out, "addrs.tcl"), "w") as f:
        f.write("# Generated by tools/gen_board_sequence.py. Data only.\n")
        f.write("\n".join(dow_lines()) + "\n")

    # Standalone loader, for when the application is already running and
    # waiting. The manifest goes last: its magic is what tells the application
    # the rest of the data really arrived.
    with open(os.path.join(args.out, "load.tcl"), "w") as f:
        f.write("# Generated by tools/gen_board_sequence.py.\n")
        f.write("# Launch the application first; it polls for the manifest.\n")
        f.write("connect\n")
        f.write('targets -set -filter {name =~ "ARM*#0"}\n')
        f.write("stop\n")
        f.write("\n".join(dow_lines()) + "\n")
        f.write("con\n")

    with open(os.path.join(args.out, "load.bat"), "w") as f:
        f.write("@echo off\r\n")
        f.write('"%s" "%%~dp0load.tcl"\r\n' % args.xsdb)
        f.write("pause\r\n")

    mb = (n_src * frame_stride + len(pairs) * golden_stride) / 2**20
    print("\nwrote %s" % here)
    print("  %d frame(s)  at 0x%08X stride 0x%X" % (n_src, FRAMES_ADDR, frame_stride))
    print("  %d golden(s) at 0x%08X stride 0x%X" % (len(pairs), GOLDEN_ADDR, golden_stride))
    print("  %d of %d payloads are byte-aligned (the 1-in-8 tlast case)"
          % (aligned, len(pairs)))
    print("  %.1f MB to load" % mb)
    print("\n  xsdb scripts/run_board.tcl %s" % here)
    return 0


if __name__ == "__main__":
    sys.exit(main())
