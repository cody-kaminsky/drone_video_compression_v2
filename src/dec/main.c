/* main.c — CLI for the matched decoder.
 *
 *   dcc_decoder <in.264> [out.yuv]
 *
 * Reads an Annex B stream produced by this project's encoder or by the FPGA
 * kernel plus its host, writes the reconstruction as NV12, and reports what
 * it found. With no output path it decodes and reports without writing, which
 * is the quick way to check a capture parses.
 *
 * The point of this pairing: `cmp` the output against the encoder's own
 * recon, or against ffmpeg's decode of the same stream. Three independent
 * reconstructions of one bitstream is a strong check on all three.
 */

#include "decoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    FILE *out;
    int frames;
    long bytes_out;
} sink_t;

static void on_frame(const u8 *y, const u8 *uv, int w, int h, void *vctx)
{
    sink_t *s = (sink_t *)vctx;
    s->frames++;
    if (!s->out) return;
    fwrite(y,  1, (size_t)w * h,       s->out);
    fwrite(uv, 1, (size_t)w * (h / 2), s->out);
    s->bytes_out += (long)w * h * 3 / 2;
}

int main(int argc, char **argv)
{
    const char *in_path, *out_path;
    FILE *f;
    long len;
    u8 *data;
    sink_t sink;
    dec_stats_t st;
    int rc;

    if (argc < 2 || argc > 3) {
        fprintf(stderr, "usage: %s <in.264> [out.yuv]\n", argv[0]);
        return 1;
    }
    in_path  = argv[1];
    out_path = (argc == 3) ? argv[2] : NULL;

    f = fopen(in_path, "rb");
    if (!f) { perror(in_path); return 2; }
    fseek(f, 0, SEEK_END); len = ftell(f); fseek(f, 0, SEEK_SET);
    if (len <= 0) { fprintf(stderr, "%s is empty\n", in_path); fclose(f); return 2; }
    data = malloc((size_t)len);
    if (!data || fread(data, 1, (size_t)len, f) != (size_t)len) {
        fprintf(stderr, "%s: short read\n", in_path);
        fclose(f); free(data); return 2;
    }
    fclose(f);

    memset(&sink, 0, sizeof sink);
    if (out_path) {
        sink.out = fopen(out_path, "wb");
        if (!sink.out) { perror(out_path); free(data); return 2; }
    }

    rc = dcc_decode_stream(data, (int)len, on_frame, &sink, &st);

    if (sink.out) fclose(sink.out);
    free(data);

    printf("STAT IN_BYTES: %ld\n", (long)len);
    printf("STAT FRAMES: %d\n", st.frames);
    if (st.frames > 0) {
        printf("STAT WIDTH:  %d\n", st.width);
        printf("STAT HEIGHT: %d\n", st.height);
        printf("STAT MB_COUNT: %d\n", st.mbs_w * st.mbs_h);
        printf("STAT SLICE_QP: %d\n", st.slice_qp);
        printf("STAT BPP: %.6f\n",
               (double)len * 8 / ((double)st.width * st.height * st.frames));
    }
    if (rc != 0) {
        printf("STAT RESULT: FAILED after %d frame(s)\n", st.frames);
        return 3;
    }
    printf("STAT RESULT: OK\n");
    return 0;
}
