/* deblock.h — in-loop deblocking filter, spec 8.7 (frame MBs, 4:2:0,
 * constant QP per frame, filter offsets 0).
 *
 * The filter runs as a post-pass over the whole reconstructed frame in MB
 * raster order, vertical edges then horizontal edges per MB, exactly the
 * order a decoder uses; because intra prediction reads samples prior to
 * deblocking (spec 8.3), the encoder keeps predicting from the unfiltered
 * plane and filters a copy that becomes the reference and the output. */
#ifndef DCC_DEBLOCK_H
#define DCC_DEBLOCK_H

#include "types.h"

/* Per-MB information the boundary strength derivation needs. */
typedef struct {
    u8  intra;         /* intra coded */
    u16 nz;            /* bit (br*4+bc): luma 4x4 block has nonzero levels (inter MBs) */
    i16 mvx, mvy;      /* vector in quarter samples (inter MBs) */
} dbk_mb_t;

/* Filter an NV12 frame in place. info is one entry per MB in raster order.
 * qp_y / qp_c are the (constant) luma and chroma QPs of the picture. */
void deblock_frame(u8 *y, int stride_y, u8 *uv, int stride_uv,
                   int mbs_w, int mbs_h, const dbk_mb_t *info, int qp_y, int qp_c);

#endif
