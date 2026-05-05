/* hls_top.c — top-level kernel for the HLS-port encoder (M3 scaffold).
 *
 * This is the synthesis entry point. It wraps the C-reference-shape
 * encode_frame_h264_hls() with the HLS interface pragmas that Vitis
 * needs to generate AXI master/stream/lite ports. For now it is a thin
 * passthrough; the per-stage dataflow decomposition (mb_fetch ->
 * mb_mode_decide -> ...) and per-array partition pragmas come in M3.2.
 *
 * Interface plan (matches architecture.txt §10 register map):
 *
 *   src_y, src_uv, recon_y_out, recon_uv_out, bs_out
 *     - AXI HP master (m_axi). Frame-sized, lives in DDR. depth/offset
 *       parameters allow a single physical port to be re-used.
 *   width, height, qp, frame_num
 *     - AXI-Lite (s_axilite). Control-plane registers.
 *   stats
 *     - AXI-Lite return register block.
 *   return
 *     - AXI-Lite return code.
 *
 * To synthesize:
 *   vitis_hls -p drone_h264_top.tcl
 * where the .tcl invokes csynth on encode_frame_h264_hls_top.
 *
 * Cosim path: build this same .c file with gcc against the same C
 * reference modules and run validate_byteexact_hls.sh; if that passes
 * byte-exact, the synthesized version starts from a known-good C source.
 */

#include "encoder_hls.h"
#include "hls_pragmas.h"

int encode_frame_h264_hls_top(int width, int height, int qp,
                              const u8 *src_y,  int stride_y,
                              const u8 *src_uv, int stride_uv,
                              u8 *recon_y_out,  int recon_stride_y,
                              u8 *recon_uv_out, int recon_stride_uv,
                              u8 *bs_out, int bs_max_size, int frame_num,
                              encode_stats_t *stats)
{
    /* === Interface pragmas — inert outside synthesis. === */
    HLS_PRAGMA(INTERFACE m_axi     port=src_y        offset=slave bundle=gmem0 depth=8294400);
    HLS_PRAGMA(INTERFACE m_axi     port=src_uv       offset=slave bundle=gmem0 depth=4147200);
    HLS_PRAGMA(INTERFACE m_axi     port=recon_y_out  offset=slave bundle=gmem1 depth=8294400);
    HLS_PRAGMA(INTERFACE m_axi     port=recon_uv_out offset=slave bundle=gmem1 depth=4147200);
    HLS_PRAGMA(INTERFACE m_axi     port=bs_out       offset=slave bundle=gmem2 depth=4194304);
    HLS_PRAGMA(INTERFACE m_axi     port=stats        offset=slave bundle=gmem2 depth=1);
    HLS_PRAGMA(INTERFACE s_axilite port=src_y           bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=src_uv          bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=recon_y_out     bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=recon_uv_out    bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=bs_out          bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=stats           bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=width           bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=height          bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=qp              bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=stride_y        bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=stride_uv       bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=recon_stride_y  bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=recon_stride_uv bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=bs_max_size     bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=frame_num       bundle=control);
    HLS_PRAGMA(INTERFACE s_axilite port=return          bundle=control);

    return encode_frame_h264_hls(width, height, qp,
                                 src_y,  stride_y,
                                 src_uv, stride_uv,
                                 recon_y_out,  recon_stride_y,
                                 recon_uv_out, recon_stride_uv,
                                 bs_out, bs_max_size, frame_num, stats);
}
