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

/* Encode one frame.
 *
 *   width, height : pixel dimensions (must be multiples of 16).
 *   qp            : luma QP, 0..51. Constant across the frame.
 *   src_y         : luma plane (width * height bytes, stride = stride_y).
 *   src_uv        : interleaved CbCr (NV12), height/2 rows of width bytes,
 *                   stride = stride_uv.
 *   recon_y_out   : optional output for reconstructed luma plane. May be
 *                   NULL to skip writing recon. If non-NULL, must hold
 *                   width * height bytes with stride = recon_stride_y.
 *   recon_uv_out  : optional reconstructed CbCr (NV12).
 *   stats         : output statistics.
 *
 * Returns 0 on success, negative on error.
 */
int encode_frame(int width, int height, int qp,
                 const u8 *src_y,  int stride_y,
                 const u8 *src_uv, int stride_uv,
                 u8 *recon_y_out,  int recon_stride_y,
                 u8 *recon_uv_out, int recon_stride_uv,
                 encode_stats_t *stats);

/* Encode one frame and emit a real H.264 Annex B bitstream.
 *
 *   bs_out       : destination buffer for the bitstream (SPS+PPS+IDR slice).
 *                  May be NULL — in which case no bitstream is emitted (same
 *                  behavior as encode_frame()).
 *   bs_max_size  : capacity of bs_out in bytes.
 *   frame_num    : frame number for the slice header (mod 16 used).
 *
 * On success, stats->bytes_out is the actual bitstream byte count.
 * v1 emits ONLY I_16x16-coded macroblocks (no I_4x4 in the stream yet).
 *
 * Returns 0 on success, negative on error. */
int encode_frame_h264(int width, int height, int qp,
                      const u8 *src_y,  int stride_y,
                      const u8 *src_uv, int stride_uv,
                      u8 *recon_y_out,  int recon_stride_y,
                      u8 *recon_uv_out, int recon_stride_uv,
                      u8 *bs_out, int bs_max_size, int frame_num,
                      encode_stats_t *stats);

#endif
