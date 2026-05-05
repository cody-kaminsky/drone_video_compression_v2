/* encoder_hls.h — public entry for the HLS-port encoder.
 *
 * Mirrors src/encoder.h's encode_frame_h264 API so the same test bench
 * driver can target either implementation. The HLS port produces
 * byte-identical output to the C reference on the validation corpus.
 */
#ifndef DCC_ENCODER_HLS_H
#define DCC_ENCODER_HLS_H

#include "encoder.h"     /* shared encode_stats_t */

int encode_frame_h264_hls(int width, int height, int qp,
                          const u8 *src_y,  int stride_y,
                          const u8 *src_uv, int stride_uv,
                          u8 *recon_y_out,  int recon_stride_y,
                          u8 *recon_uv_out, int recon_stride_uv,
                          u8 *bs_out, int bs_max_size, int frame_num,
                          encode_stats_t *stats);

#endif
