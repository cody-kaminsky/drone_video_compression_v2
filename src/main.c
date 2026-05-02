/* main.c — CLI driver for the C reference encoder.
 *
 * Usage:
 *   dcc_encoder <in.yuv> <width> <height> <qp> [recon.yuv]
 *
 *   in.yuv     : raw NV12 YUV420 (luma plane, then interleaved CbCr).
 *                Size = width * height * 3 / 2 bytes.
 *   recon.yuv  : optional output of reconstructed NV12 frame.
 *
 * Stdout:
 *   STAT key: value lines suitable for parsing by tools/validate.py.
 *   Final line is JSON for one-shot consumption.
 */

#include "encoder.h"
#include "psnr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int read_file(const char *path, u8 *buf, size_t n)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    size_t got = fread(buf, 1, n, f);
    fclose(f);
    if (got != n) {
        fprintf(stderr, "%s: short read (%zu / %zu)\n", path, got, n);
        return -2;
    }
    return 0;
}

static int write_file(const char *path, const u8 *buf, size_t n)
{
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return -1; }
    size_t put = fwrite(buf, 1, n, f);
    fclose(f);
    if (put != n) {
        fprintf(stderr, "%s: short write (%zu / %zu)\n", path, put, n);
        return -2;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 5) {
        fprintf(stderr,
            "usage: %s <in.yuv> <width> <height> <qp> [recon.yuv [bitstream.264]]\n"
            "  bitstream.264 : optional H.264 Annex B output (M2 path)\n",
            argv[0]);
        return 1;
    }
    const char *in_path  = argv[1];
    int width  = atoi(argv[2]);
    int height = atoi(argv[3]);
    int qp     = atoi(argv[4]);
    const char *recon_path = (argc >= 6) ? argv[5] : NULL;
    const char *bs_path    = (argc >= 7) ? argv[6] : NULL;

    if (width  % 16 != 0) { fprintf(stderr, "width must be multiple of 16\n");  return 2; }
    if (height % 16 != 0) { fprintf(stderr, "height must be multiple of 16\n"); return 2; }
    if (qp < 0 || qp > 51) { fprintf(stderr, "qp must be 0..51\n"); return 2; }

    size_t y_size  = (size_t)width * height;
    size_t uv_size = (size_t)width * (height / 2);
    size_t fr_size = y_size + uv_size;

    u8 *frame = (u8*)malloc(fr_size);
    if (!frame) { fprintf(stderr, "oom\n"); return 3; }

    if (read_file(in_path, frame, fr_size) != 0) {
        free(frame); return 4;
    }

    u8 *recon = NULL;
    if (recon_path) {
        recon = (u8*)malloc(fr_size);
        if (!recon) { free(frame); fprintf(stderr, "oom\n"); return 3; }
    }

    encode_stats_t stats;
    int rc;
    u8 *bs_buf = NULL;
    int bs_len = 0;

    if (bs_path) {
        /* M2 bitstream-emitting path.
         * At low QPs (<10) the bitstream can exceed the source size due to
         * large quant levels needing many CAVLC bits. Cap at 4x source. */
        int bs_cap = (int)fr_size * 4 + 1024;
        bs_buf = (u8*)malloc((size_t)bs_cap);
        if (!bs_buf) { fprintf(stderr, "oom\n"); free(frame); free(recon); return 3; }
        rc = encode_frame_h264(width, height, qp,
                               frame,           width,
                               frame + y_size,  width,
                               recon,           recon ? width : 0,
                               recon ? recon + y_size : NULL, recon ? width : 0,
                               bs_buf, bs_cap, 0,
                               &stats);
        if (rc == 0) bs_len = stats.bytes_out;
    } else {
        rc = encode_frame(width, height, qp,
                          frame,           width,
                          frame + y_size,  width,
                          recon,           recon ? width : 0,
                          recon ? recon + y_size : NULL, recon ? width : 0,
                          &stats);
    }
    if (rc != 0) {
        fprintf(stderr, "encode failed: %d\n", rc);
        free(frame); free(recon); free(bs_buf);
        return 5;
    }

    /* Bitstream-emit path leaves PSNR / bpp unfilled (kernel is int-only,
     * matching the FPGA register contract). Compute them host-side from the
     * recon plane. The prototype encode_frame path still fills them itself. */
    if (bs_path && recon) {
        const u8 *src_y_p  = frame;
        const u8 *src_uv_p = frame + y_size;
        const u8 *rec_y_p  = recon;
        const u8 *rec_uv_p = recon + y_size;
        stats.psnr_y = psnr_plane(src_y_p, width, rec_y_p, width, width, height);
        stats.psnr_u = psnr_chroma_component(src_uv_p, width, rec_uv_p, width,
                                             width / 2, height / 2, 0);
        stats.psnr_v = psnr_chroma_component(src_uv_p, width, rec_uv_p, width,
                                             width / 2, height / 2, 1);
        stats.psnr_avg = (stats.psnr_y * 6.0 + stats.psnr_u + stats.psnr_v) / 8.0;
        stats.bpp      = (double)stats.total_bits / ((double)width * height);
    }

    if (recon_path) {
        write_file(recon_path, recon, fr_size);
    }
    if (bs_path && bs_buf && bs_len > 0) {
        write_file(bs_path, bs_buf, (size_t)bs_len);
    }

    /* Machine-readable stats. */
    printf("STAT WIDTH:  %d\n", width);
    printf("STAT HEIGHT: %d\n", height);
    printf("STAT QP:     %d\n", qp);
    printf("STAT MB_COUNT: %d\n", stats.mb_count);
    printf("STAT PSNR_Y: %.4f\n", stats.psnr_y);
    printf("STAT PSNR_U: %.4f\n", stats.psnr_u);
    printf("STAT PSNR_V: %.4f\n", stats.psnr_v);
    printf("STAT PSNR_AVG: %.4f\n", stats.psnr_avg);
    printf("STAT TOTAL_BITS: %d\n", stats.total_bits);
    printf("STAT BYTES_OUT: %d\n", stats.bytes_out);
    printf("STAT BPP: %.6f\n", stats.bpp);

    /* JSON one-liner for tools that want it. */
    printf("{\"width\":%d,\"height\":%d,\"qp\":%d,"
           "\"psnr_y\":%.4f,\"psnr_u\":%.4f,\"psnr_v\":%.4f,\"psnr_avg\":%.4f,"
           "\"total_bits\":%d,\"bytes_out\":%d,\"bpp\":%.6f}\n",
           width, height, qp,
           stats.psnr_y, stats.psnr_u, stats.psnr_v, stats.psnr_avg,
           stats.total_bits, stats.bytes_out, stats.bpp);

    free(bs_buf);
    free(frame);
    free(recon);
    return 0;
}
