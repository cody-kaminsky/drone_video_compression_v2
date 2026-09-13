/* main.c — CLI driver for the C reference encoder.
 *
 * Usage:
 *   dcc_encoder <in.yuv> <width> <height> <qp> [recon.yuv [bitstream.264]] [options]
 *
 *   in.yuv     : raw NV12 YUV420 frames (luma plane, then interleaved CbCr),
 *                width * height * 3 / 2 bytes per frame; all frames in the
 *                file are coded unless --frames says otherwise.
 *   recon.yuv  : optional output of the reconstructed NV12 frames.
 *   bitstream.264 : optional H.264 Annex B output.
 *
 * Options (P frames):
 *   --frames N        code at most N frames (default: all in the file)
 *   --intra-only      every frame an IDR frame (the pre-P behaviour)
 *   --gop G           an IDR every G frames (default 0: only the first)
 *   --refresh-cols C  intra refresh band width in MB columns per P frame
 *                     (default 1; 0 disables the refresh)
 *   --intra-budget B  extra intra MBs allowed per P frame (default 64)
 *   --me-range R      integer search range in samples (default 16)
 *   --no-strict-refresh  let MBs left of the band reference unrefreshed area
 *   --no-deblock      in-loop deblocking off (the I-only hardware kernel's mode)
 * Rate control (MB level):
 *   --bitrate BPS     target / ceiling bit rate; <qp> is then the start QP
 *   --fps N           frames per second (default 30)
 *   --qp-min A --qp-max B   MB QP range (default 22..32)
 *   --rc-buffer F     bucket size in frame budgets (default 4). This is the
 *                     receiver buffering the link must have; it also sets how
 *                     many bits the IDR may draw ahead.
 *
 * Stdout:
 *   STAT key: value lines per frame and totals, suitable for parsing.
 */

#include "encoder.h"
#include "psnr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int write_file(FILE *f, const u8 *buf, size_t n)
{
    size_t put = fwrite(buf, 1, n, f);
    return put == n ? 0 : -2;
}

int main(int argc, char **argv)
{
    if (argc < 5) {
        fprintf(stderr,
            "usage: %s <in.yuv> <width> <height> <qp> [recon.yuv [bitstream.264]] [options]\n"
            "  --frames N  --intra-only  --gop G  --refresh-cols C  --intra-budget B  --me-range R\n"
            "  --no-strict-refresh  --no-deblock\n"
            "  --bitrate BPS --fps N --qp-min A --qp-max B --rc-buffer F\n",
            argv[0]);
        return 1;
    }
    const char *in_path  = argv[1];
    int width  = atoi(argv[2]);
    int height = atoi(argv[3]);
    int qp     = atoi(argv[4]);
    const char *recon_path = NULL, *bs_path = NULL;
    int max_frames = -1, intra_only = 0, gop = 0, refresh_cols = 1, intra_budget = 64, me_range = 16;
    int strict = 1, deblock = 1;
    long bitrate = 0; int fps = 30, qp_min = 22, qp_max = 32, rc_buffer = 4;
    int npos = 0;
    for (int i = 5; i < argc; i++) {
        if (!strcmp(argv[i], "--frames") && i + 1 < argc)            max_frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--intra-only"))                   intra_only = 1;
        else if (!strcmp(argv[i], "--gop") && i + 1 < argc)          gop = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--refresh-cols") && i + 1 < argc) refresh_cols = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--intra-budget") && i + 1 < argc) intra_budget = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--me-range") && i + 1 < argc)     me_range = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--no-strict-refresh"))            strict = 0;
        else if (!strcmp(argv[i], "--no-deblock"))                   deblock = 0;
        else if (!strcmp(argv[i], "--bitrate") && i + 1 < argc)      bitrate = atol(argv[++i]);
        else if (!strcmp(argv[i], "--fps") && i + 1 < argc)          fps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--qp-min") && i + 1 < argc)       qp_min = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--qp-max") && i + 1 < argc)       qp_max = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--rc-buffer") && i + 1 < argc)    rc_buffer = atoi(argv[++i]);
        else if (argv[i][0] == '-') { fprintf(stderr, "unknown option %s\n", argv[i]); return 1; }
        else if (npos == 0) { recon_path = argv[i]; npos++; }
        else if (npos == 1) { bs_path = argv[i]; npos++; }
    }

    if (width  % 16 != 0) { fprintf(stderr, "width must be multiple of 16\n");  return 2; }
    if (height % 16 != 0) { fprintf(stderr, "height must be multiple of 16\n"); return 2; }
    if (qp < 0 || qp > 51) { fprintf(stderr, "qp must be 0..51\n"); return 2; }

    size_t y_size  = (size_t)width * height;
    size_t uv_size = (size_t)width * (height / 2);
    size_t fr_size = y_size + uv_size;

    FILE *fin = fopen(in_path, "rb");
    if (!fin) { perror(in_path); return 4; }
    fseek(fin, 0, SEEK_END);
    long fsz = ftell(fin);
    fseek(fin, 0, SEEK_SET);
    int nframes = (int)(fsz / (long)fr_size);
    if (nframes < 1) { fprintf(stderr, "%s: shorter than one frame\n", in_path); return 4; }
    if (max_frames > 0 && max_frames < nframes) nframes = max_frames;

    u8 *frame = (u8*)malloc(fr_size);
    u8 *recon = (u8*)malloc(fr_size);
    int bs_cap = (int)fr_size * 4 + 1024;
    u8 *bs_buf = (u8*)malloc((size_t)bs_cap);
    if (!frame || !recon || !bs_buf) { fprintf(stderr, "oom\n"); return 3; }

    FILE *frec = recon_path ? fopen(recon_path, "wb") : NULL;
    FILE *fbs  = bs_path ? fopen(bs_path, "wb") : NULL;

    int mbs_w = width / 16;
    long total_bytes = 0;
    double sum_psnr_y = 0;
    int refresh_col = 0, frame_num = 0;
    long tot_intra = 0, tot_inter = 0, tot_skip = 0;
    long max_fill = 0, over_budget = 0, rc_overflow = 0;
    long frame_budget = bitrate > 0 ? bitrate / fps : 0;

    for (int fi = 0; fi < nframes; fi++) {
        if (fread(frame, 1, fr_size, fin) != fr_size) { fprintf(stderr, "short read at frame %d\n", fi); break; }
        encode_cfg_t cfg;
        int idr = (fi == 0) || intra_only || (gop > 0 && (fi % gop) == 0);
        cfg.frame_type = idr ? 0 : 1;
        cfg.frame_num = idr ? 0 : frame_num;
        cfg.me_range = me_range;
        cfg.refresh_cols = refresh_cols;
        cfg.refresh_col = refresh_col;
        cfg.intra_budget = intra_budget;
        cfg.refresh_strict = strict;
        cfg.deblock = deblock;
        cfg.rc_bps = bitrate; cfg.rc_fps = fps; cfg.rc_qp_min = qp_min; cfg.rc_qp_max = qp_max;
        cfg.rc_bucket_frames = rc_buffer; cfg.rc_reset = (fi == 0);
        cfg.rc_gop = intra_only ? 1 : (gop > 0 ? gop : nframes);

        encode_stats_t stats;
        encode_pstats_t ps = {0};
        int rc = encode_frame_h264_ext(width, height, qp,
                                       frame, width, frame + y_size, width,
                                       recon, width, recon + y_size, width,
                                       bs_buf, bs_cap, &cfg, &stats, &ps);
        if (rc != 0) { fprintf(stderr, "encode failed at frame %d: %d\n", fi, rc); return 5; }
        if (idr) { frame_num = 1; }
        else {
            frame_num = (frame_num + 1) & 0xF;
            if (refresh_cols > 0) { refresh_col += refresh_cols; if (refresh_col >= mbs_w) refresh_col = 0; }
        }

        double py = psnr_plane(frame, width, recon, width, width, height);
        double pu = psnr_chroma_component(frame + y_size, width, recon + y_size, width, width / 2, height / 2, 0);
        double pv = psnr_chroma_component(frame + y_size, width, recon + y_size, width, width / 2, height / 2, 1);
        if (frec) write_file(frec, recon, fr_size);
        if (fbs) write_file(fbs, bs_buf, (size_t)stats.bytes_out);
        total_bytes += stats.bytes_out;
        sum_psnr_y += py;
        tot_intra += idr ? stats.mb_count : ps.mbs_intra;
        tot_inter += ps.mbs_inter; tot_skip += ps.mbs_skip;
        printf("STAT FRAME %d: type %s bytes %d psnr_y %.4f psnr_u %.4f psnr_v %.4f intra %d inter %d skip %d",
               fi, idr ? "IDR" : "P", stats.bytes_out, py, pu, pv,
               idr ? stats.mb_count : ps.mbs_intra, ps.mbs_inter, ps.mbs_skip);
        if (bitrate > 0) {
            printf(" qp %d avg %.2f range %d-%d target %ld bucket %ld/%ld",
                   ps.qp_frame, ps.qp_avg100 / 100.0, ps.qp_lo, ps.qp_hi,
                   ps.frame_target, ps.bucket_fill, ps.bucket_cap);
            if (ps.bucket_fill > max_fill) max_fill = ps.bucket_fill;
            if ((long)stats.bytes_out * 8 > frame_budget) over_budget++;
            rc_overflow = ps.rc_overflow;
        }
        printf("\n");
    }
    if (frec) fclose(frec);
    if (fbs) fclose(fbs);
    fclose(fin);

    printf("STAT WIDTH:  %d\n", width);
    printf("STAT HEIGHT: %d\n", height);
    printf("STAT QP:     %d\n", qp);
    printf("STAT FRAMES: %d\n", nframes);
    printf("STAT MB_COUNT: %d\n", mbs_w * (height / 16));
    printf("STAT PSNR_Y: %.4f\n", sum_psnr_y / nframes);
    printf("STAT TOTAL_BITS: %ld\n", total_bytes * 8);
    printf("STAT BYTES_OUT: %ld\n", total_bytes);
    printf("STAT BPP: %.6f\n", (double)total_bytes * 8 / ((double)width * height * nframes));
    printf("STAT MBS: intra %ld inter %ld skip %ld\n", tot_intra, tot_inter, tot_skip);
    if (bitrate > 0) {
        printf("STAT RC: target %ld bps, achieved %.0f bps, frames over budget %ld/%d, max bucket %ld bits (%.1f ms of link)\n",
               bitrate, (double)total_bytes * 8 * fps / nframes, over_budget, nframes, max_fill, 1000.0 * max_fill / bitrate);
        printf("STAT RC_OVERFLOW: %ld bits\n", rc_overflow);
        if (rc_overflow > 0)
            printf("STAT RC_WARN: bucket overflowed by %ld bits -- qp-max %d cannot hold %ld bps on this content\n",
                   rc_overflow, qp_max, bitrate);
    }

    free(bs_buf); free(frame); free(recon);
    return 0;
}
