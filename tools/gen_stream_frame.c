/* gen_stream_frame.c — write one NV12 frame out in the kernel's stream order.
 *
 * This exists so that the board-side ordering has exactly one implementation.
 * It calls h264_nv12_to_stream(), the same function the board would have used
 * to stage a frame and the same one tools/gen_frame_stream.py is checked
 * against by `make host_test`. Doing the reorder here, once, on the
 * workstation is what lets the board skip staging entirely.
 *
 * Usage: gen_stream_frame <in.yuv> <width> <height> <frame_index> <out.bin>
 */

#include "h264_host.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    const char *in_path, *out_path;
    int width, height, index;
    uint32_t frame_bytes;
    unsigned char *nv12, *stream;
    FILE *f;

    if (argc != 6) {
        fprintf(stderr, "usage: %s <in.yuv> <w> <h> <frame_index> <out.bin>\n", argv[0]);
        return 1;
    }
    in_path  = argv[1];
    width    = atoi(argv[2]);
    height   = atoi(argv[3]);
    index    = atoi(argv[4]);
    out_path = argv[5];

    if (width % 16 || height % 16) {
        fprintf(stderr, "width and height must be multiples of 16\n");
        return 1;
    }
    frame_bytes = h264_frame_bytes(width, height);

    nv12   = malloc(frame_bytes);
    stream = malloc(frame_bytes);
    if (!nv12 || !stream) { fprintf(stderr, "out of memory\n"); return 2; }

    f = fopen(in_path, "rb");
    if (!f) { perror(in_path); return 2; }
    if (fseek(f, (long)index * (long)frame_bytes, SEEK_SET) != 0) {
        fprintf(stderr, "%s: cannot seek to frame %d\n", in_path, index);
        return 2;
    }
    if (fread(nv12, 1, frame_bytes, f) != frame_bytes) {
        fprintf(stderr, "%s: short read at frame %d (wanted %u bytes)\n",
                in_path, index, frame_bytes);
        return 2;
    }
    fclose(f);

    h264_nv12_to_stream(stream, nv12, width, height);

    f = fopen(out_path, "wb");
    if (!f) { perror(out_path); return 2; }
    if (fwrite(stream, 1, frame_bytes, f) != frame_bytes) {
        fprintf(stderr, "%s: short write\n", out_path);
        return 2;
    }
    fclose(f);

    free(nv12); free(stream);
    return 0;
}
