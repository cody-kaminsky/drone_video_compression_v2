#!/usr/bin/env python3
"""gen_board_sequence.py — build an L5 test sequence for the board.

For each frame of an NV12 file this produces two binaries and one manifest
entry:

  frame_NNN.bin   the frame in the kernel's stream order (no staging needed)
  golden_NNN.bin  the payload the C reference says the kernel must emit

plus manifest.bin describing the whole set, and load.tcl to push it all into
DDR over JTAG. The board application reads the manifest, so changing the
sequence needs a reload but never a recompile.

Because the encoder is intra only, frames are independent: the kernel holds no
state across them, which the AXI testbench showed by running one frame twice
and getting identical bytes and identical cycle counts. So a short sequence
cycled many times exercises restart and sustained throughput exactly as well
as a long one, and loads in seconds rather than minutes over JTAG.

Usage:
  gen_board_sequence.py <in.yuv> <width> <height> <qp> [options]
    --frames N     frames to take from the file (default: all)
    --repeats R    times the board should cycle the sequence (default 1)
    --out DIR      output directory (default build/seq_board)
    --aclk HZ      PL clock, written into the manifest (default 100000000)
"""

import argparse
import os
import struct
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Must match host/dcc_memmap.h.
MANIFEST_ADDR = 0x03000000
FRAMES_ADDR   = 0x04000000
GOLDEN_ADDR   = 0x20000000
PAYLOAD_ADDR  = 0x30000000
FRAMES_ROOM   = GOLDEN_ADDR - FRAMES_ADDR
GOLDEN_ROOM   = PAYLOAD_ADDR - GOLDEN_ADDR
MAGIC         = 0x4D434344
VERSION       = 1

ENCODER = os.path.join("build", "dcc_encoder.exe")
SHUFFLE = os.path.join("build", "gen_stream_frame.exe")


def align(x, a=64):
    return (x + a - 1) // a * a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yuv")
    ap.add_argument("width", type=int)
    ap.add_argument("height", type=int)
    ap.add_argument("qp", type=int)
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--out", default=os.path.join("build", "seq_board"))
    ap.add_argument("--aclk", type=int, default=100_000_000)
    args = ap.parse_args()

    # Absolute paths: on this Python build CreateProcess refuses a relative
    # path with forward slashes even when the file is plainly there.
    enc = os.path.abspath(ENCODER if os.path.isfile(ENCODER) else ENCODER[:-4])
    shf = os.path.abspath(SHUFFLE if os.path.isfile(SHUFFLE) else SHUFFLE[:-4])
    for tool, how in ((enc, "make ref"), (shf, "make board_seq_tools")):
        if not os.path.isfile(tool):
            sys.exit("missing %s -- run `%s`" % (tool, how))

    if args.width % 16 or args.height % 16:
        sys.exit("width and height must be multiples of 16")

    frame_bytes = args.width * args.height * 3 // 2
    total = os.path.getsize(args.yuv)
    avail = total // frame_bytes
    if avail == 0:
        sys.exit("%s is %d bytes, smaller than one %dx%d frame (%d bytes)"
                 % (args.yuv, total, args.width, args.height, frame_bytes))
    n = avail if args.frames <= 0 else min(args.frames, avail)

    os.makedirs(args.out, exist_ok=True)
    frame_stride = align(frame_bytes)
    if n * frame_stride > FRAMES_ROOM:
        sys.exit("%d frames need %.0f MB but the layout gives %.0f MB; "
                 "use fewer frames and more --repeats"
                 % (n, n * frame_stride / 2**20, FRAMES_ROOM / 2**20))

    print("%dx%d QP%d, %d of %d frames, %d repeats"
          % (args.width, args.height, args.qp, n, avail, args.repeats))

    recs, goldens = [], []
    tmp_yuv = os.path.join(args.out, "_one.yuv")
    for i in range(n):
        # frame in stream order, via the one implementation of the ordering
        fbin = os.path.join(args.out, "frame_%03d.bin" % i)
        subprocess.run([shf, args.yuv, str(args.width), str(args.height),
                        str(i), fbin], check=True)

        # golden payload: the reference encodes this frame on its own
        with open(args.yuv, "rb") as src, open(tmp_yuv, "wb") as dst:
            src.seek(i * frame_bytes)
            dst.write(src.read(frame_bytes))
        gtxt = os.path.join(args.out, "_golden.txt")
        env = dict(os.environ, DCC_DUMP_SLICE=gtxt)
        subprocess.run([enc, tmp_yuv, str(args.width), str(args.height),
                        str(args.qp)], check=True, env=env,
                       stdout=subprocess.DEVNULL)
        with open(gtxt) as f:
            payload = bytes(int(line) for line in f if line.strip())
        gbin = os.path.join(args.out, "golden_%03d.bin" % i)
        with open(gbin, "wb") as f:
            f.write(payload)
        goldens.append(len(payload))
        recs.append((fbin, gbin, len(payload)))
        print("  frame %3d: %d payload bytes" % (i, len(payload)))

    for p in (tmp_yuv, os.path.join(args.out, "_golden.txt")):
        if os.path.isfile(p):
            os.remove(p)

    golden_stride = align(max(goldens))
    if n * golden_stride > GOLDEN_ROOM:
        sys.exit("goldens need %.0f MB, layout gives %.0f MB"
                 % (n * golden_stride / 2**20, GOLDEN_ROOM / 2**20))

    # ---- manifest ----
    blob = struct.pack("<8I", MAGIC, VERSION, args.width, args.height,
                       args.qp, n, args.repeats, args.aclk)
    for i, (_, _, glen) in enumerate(recs):
        blob += struct.pack("<4I",
                            FRAMES_ADDR + i * frame_stride, frame_bytes,
                            GOLDEN_ADDR + i * golden_stride, glen)
    man = os.path.join(args.out, "manifest.bin")
    with open(man, "wb") as f:
        f.write(blob)

    # ---- xsdb loader ----
    tcl = os.path.join(args.out, "load.tcl")
    here = os.path.abspath(args.out).replace("\\", "/")
    with open(tcl, "w") as f:
        f.write("# Generated by tools/gen_board_sequence.py. Load with:\n")
        f.write("#   xsdb %s/load.tcl\n" % here)
        f.write("#\n")
        f.write("# Order: launch the application from Vitis FIRST. It polls for\n")
        f.write("# the manifest and prints 'waiting for a manifest'. Then run\n")
        f.write("# this: it halts the core, writes the data, and resumes into\n")
        f.write("# that poll. No breakpoint needed.\n")
        f.write("#\n")
        f.write("# The manifest is written LAST on purpose: its magic number is\n")
        f.write("# what tells the application the rest of the data really\n")
        f.write("# arrived, so a load that dies halfway leaves it waiting\n")
        f.write("# rather than running on garbage.\n")
        f.write("connect\n")
        f.write('targets -set -filter {name =~ "ARM*#0"}\n')
        f.write("stop\n")
        f.write('puts "loading %d frames, %d repeats"\n' % (n, args.repeats))
        for i, (fbin, gbin, _) in enumerate(recs):
            f.write('dow -data %s/%s 0x%08X\n'
                    % (here, os.path.basename(fbin), FRAMES_ADDR + i * frame_stride))
            f.write('dow -data %s/%s 0x%08X\n'
                    % (here, os.path.basename(gbin), GOLDEN_ADDR + i * golden_stride))
            f.write('puts "  frame %d loaded"\n' % i)
        f.write('dow -data %s/manifest.bin 0x%08X\n' % (here, MANIFEST_ADDR))
        f.write('puts "manifest written at 0x%08X, resuming"\n' % MANIFEST_ADDR)
        f.write("con\n")

    # Data-only loader: just the dow -data lines, no connect/stop/con, so a
    # bigger script (scripts/run_board.tcl) can source it after it has
    # already programmed the PL and downloaded the ELF.
    addrs = os.path.join(args.out, "addrs.tcl")
    with open(addrs, "w") as f:
        f.write("# Generated by tools/gen_board_sequence.py. Data only.\n")
        for i, (fbin, gbin, _) in enumerate(recs):
            f.write('dow -data %s/%s 0x%08X\n'
                    % (here, os.path.basename(fbin), FRAMES_ADDR + i * frame_stride))
            f.write('dow -data %s/%s 0x%08X\n'
                    % (here, os.path.basename(gbin), GOLDEN_ADDR + i * golden_stride))
        f.write('dow -data %s/manifest.bin 0x%08X\n' % (here, MANIFEST_ADDR))

    mb = n * (frame_stride + golden_stride) / 2**20
    print("\nwrote %s" % args.out)
    print("  manifest.bin   %d bytes -> 0x%08X" % (len(blob), MANIFEST_ADDR))
    print("  frames         %d x %d bytes -> 0x%08X stride 0x%X"
          % (n, frame_bytes, FRAMES_ADDR, frame_stride))
    print("  goldens        %d, max %d bytes -> 0x%08X stride 0x%X"
          % (n, max(goldens), GOLDEN_ADDR, golden_stride))
    print("  total to load  %.1f MB (JTAG at ~1 MB/s is about %.0f s)" % (mb, mb))
    print("\n  xsdb %s/load.tcl" % here)
    return 0


if __name__ == "__main__":
    sys.exit(main())
