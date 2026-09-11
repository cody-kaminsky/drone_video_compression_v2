/* encoder.h — main encoder API.
 *
 * v1 SCOPE
 *   - I-frame only.
 *   - Single frame per call (no GOP state between frames).
 *   - I_16x16 luma + Intra_Chroma 8x8 only (no I_4x4 yet).
 *   - SAD-based mode decision among the 4 luma modes and 4 chroma modes.
 *   - CAVLC bit-length estimation (no actual bitstream output).
 *   - Internal reconstruction matches what an H.264 decoder would produce
 *     for an equivalent valid stream → PSNR is meaningful.
 *
 * FPGA: this API mirrors the eventual hardware register interface
 *       (see architecture.txt §10):
 *         - Inputs: pointers + strides for src; pointers + strides for recon
 *         - QP setpoint
 *         - Output: stats struct with bytes_out, perf counters
 *       The C reference can later wrap a hardware-accelerated path with
 *       the same API.
 */
#ifndef DCC_ENCODER_H
#define DCC_ENCODER_H

#include "types.h"

typedef struct {
    /* Per-frame metrics — populated on return. */
    double psnr_y;       /* dB */
    double psnr_u;
    double psnr_v;
    double psnr_avg;     /* luma-weighted (Y*6 + U + V) / 8 */
    int    total_bits;   /* CAVLC residual + estimated header overhead */
    double bpp;          /* total_bits / (width * height) */
    int    mb_count;
    int    bytes_out;    /* (total_bits + 7) / 8 */
} encode_stats_t;

/* Encode one frame and emit a real H.264 Annex B bitstream.
 *
 *   width, height : pixel dimensions (must be multiples of 16, <= MAX_W/MAX_H).
 *   qp            : luma QP, 0..51. Constant across the frame.
 *   src_y         : luma plane (width*height bytes, stride = stride_y).
 *   src_uv        : interleaved CbCr (NV12), height/2 rows of width bytes,
 *                   stride = stride_uv.
 *   recon_y_out / recon_uv_out: optional reconstructed planes (may be NULL).
 *                   Used by the test bench to compute PSNR.
 *   bs_out        : destination buffer for the bitstream (SPS+PPS+IDR slice).
 *   bs_max_size   : capacity of bs_out in bytes.
 *   frame_num     : frame number for the slice header (mod 16 used).
 *
 * Each MB is coded as I_16x16 or I_4x4 — picked per-MB by the mode-decision
 * stage. stats->bytes_out is the actual bitstream byte count. PSNR fields
 * are NOT populated by the kernel (it's int-only); see psnr.h to compute
 * them host-side from the recon planes.
 *
 * Returns 0 on success, negative on error. */
int encode_frame_h264(int width, int height, int qp,
                      const u8 *src_y,  int stride_y,
                      const u8 *src_uv, int stride_uv,
                      u8 *recon_y_out,  int recon_stride_y,
                      u8 *recon_uv_out, int recon_stride_uv,
                      u8 *bs_out, int bs_max_size, int frame_num,
                      encode_stats_t *stats);

/* ===== P frames with rolling intra refresh =====
 *
 * A sequence is coded as one IDR frame followed by P frames that predict
 * from the previous frame's reconstruction (one reference, 16x16 vectors,
 * quarter-sample motion compensation). Instead of periodic I frames, a
 * band of refresh_cols MB columns is forced intra in every P frame and
 * walks across the picture (refresh_col is the band's first column), so
 * every MB is refreshed once per mbs_w / refresh_cols frames and the frame
 * size stays flat. Outside the band an MB may still be coded intra when
 * that is cheaper, but only intra_budget MBs per frame may do so, which
 * bounds the per-frame work of the hardware pipeline.
 *
 * Every P_Skip / P_L0_16x16 / intra decision is made from estimated
 * CAVLC bits, as the intra paths do. */
typedef struct {
    int frame_type;      /* 0 = IDR, 1 = P */
    int frame_num;       /* frame_num for the slice header (IDR: also idr_pic_id) */
    int me_range;        /* integer search range in samples around the predictor */
    int refresh_col;     /* first MB column of the intra refresh band (P frames) */
    int refresh_cols;    /* width of the band in MB columns (0 = no refresh) */
    int intra_budget;    /* extra intra MBs allowed per P frame outside the band */
} encode_cfg_t;

typedef struct {
    int mbs_intra;       /* per-frame decision counts */
    int mbs_inter;
    int mbs_skip;
} encode_pstats_t;

/* Encode one frame per cfg. The reconstruction of the previous call is the
 * reference for a P frame (the encoder keeps it internally). An IDR frame
 * writes SPS + PPS + IDR NAL; a P frame writes one non-IDR slice NAL.
 * pstats may be NULL. */
int encode_frame_h264_ext(int width, int height, int qp,
                          const u8 *src_y,  int stride_y,
                          const u8 *src_uv, int stride_uv,
                          u8 *recon_y_out,  int recon_stride_y,
                          u8 *recon_uv_out, int recon_stride_uv,
                          u8 *bs_out, int bs_max_size,
                          const encode_cfg_t *cfg,
                          encode_stats_t *stats, encode_pstats_t *pstats);

#endif
