#!/usr/bin/env python3
"""Build build/decoder_frame_vectors.txt: a whole frame for decoder_top.

The payload is the encoder's own DCC_DUMP_SLICE output, which is the
macroblock layer re-aligned to bit 0 -- byte for byte what mb_pipeline_
controller emits and what decoder_top consumes. The expectation is the
reconstruction the C decoder produces from that same stream, which
`make dec_test` already shows is byte-exact against both the encoder's
reconstruction and ffmpeg.

So the test is end to end on real data: real coefficient statistics, real
mode decisions, every macroblock position including all four edges, and a
golden that has been checked against an implementation sharing no code with
this project.

The expectation is written per block rather than per frame on purpose. A
frame-level comparison says only that something is wrong; the testbench
needs to name the first macroblock and the first 4x4 block that differs,
because in an intra frame one wrong sample propagates into every block that
predicts from it and the visible damage starts far from the cause.

    python tools/gen_decoder_frame_vectors.py W H QP payload.txt recon.yuv out.txt

Output:
    <mbs_w> <mbs_h> <qp> <chroma_qp_offset> <n_payload_bytes>
    one payload byte per line
    then 24 lines per macroblock, raster macroblock order:
      Y blocks 0..15 (raster inside the macroblock), U 0..3, V 0..3,
      each line 16 decimal samples, raster inside the 4x4
"""

import sys


def main():
    if len(sys.argv) != 7:
        sys.stderr.write(__doc__)
        return 2
    w, h, qp = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
    payload_path, recon_path, out_path = sys.argv[4], sys.argv[5], sys.argv[6]

    payload = [int(x) for x in open(payload_path).read().split()]
    buf = open(recon_path, "rb").read()

    y_size = w * h
    uv_size = w * (h // 2)
    if len(buf) < y_size + uv_size:
        sys.stderr.write("%s: %d bytes, expected at least %d\n"
                         % (recon_path, len(buf), y_size + uv_size))
        return 1
    Y = buf[:y_size]
    UV = buf[y_size:y_size + uv_size]

    mbs_w, mbs_h = w // 16, h // 16
    if mbs_w * 16 != w or mbs_h * 16 != h:
        sys.stderr.write("frame is not a whole number of macroblocks\n")
        return 1

    out = open(out_path, "w")
    # The encoder's PPS writes chroma_qp_index_offset as 0; the decoder reads
    # it from the PPS, so carry it explicitly rather than assume it here.
    out.write("%d %d %d %d %d\n" % (mbs_w, mbs_h, qp, 0, len(payload)))
    for b in payload:
        out.write("%d\n" % b)

    cw = w                      # NV12: two bytes per chroma sample column
    for mb_r in range(mbs_h):
        for mb_c in range(mbs_w):
            for k in range(16):
                br, bc = k // 4, k % 4
                x0, y0 = mb_c * 16 + bc * 4, mb_r * 16 + br * 4
                px = [Y[(y0 + j // 4) * w + x0 + (j % 4)] for j in range(16)]
                out.write(" ".join(str(p) for p in px) + "\n")
            for comp in range(2):
                for k in range(4):
                    br, bc = k // 2, k % 2
                    xc, yc = mb_c * 8 + bc * 4, mb_r * 8 + br * 4
                    px = [UV[(yc + j // 4) * cw + (xc + (j % 4)) * 2 + comp]
                          for j in range(16)]
                    out.write(" ".join(str(p) for p in px) + "\n")
    out.close()
    sys.stderr.write("wrote %s: %dx%d macroblocks, %d payload bytes\n"
                     % (out_path, mbs_w, mbs_h, len(payload)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
