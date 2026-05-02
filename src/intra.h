/* intra.h — intra prediction modes for I-only encoding.
 *
 * v1 implements I_16x16 (luma) and Intra_Chroma 8x8 (chroma).
 * I_4x4 (9 modes per 4x4 sub-block) is deferred to the next iteration.
 *
 * FPGA: prediction is small combinational logic plus neighbor-buffer reads.
 *       The tight dependency chain is read-recon-pixels-of-prior-MB into
 *       this MB's predictors — see architecture.txt §4 (intra dep chain).
 */
#ifndef DCC_INTRA_H
#define DCC_INTRA_H

#include "types.h"

/* Luma I_16x16 modes (spec 8.3.3). */
enum {
    I16_VERTICAL   = 0,
    I16_HORIZONTAL = 1,
    I16_DC         = 2,
    I16_PLANE      = 3,
};

/* Luma I_4x4 modes (spec 8.3.1).
 *
 * Modes use up to 13 reference samples around the 4x4 block:
 *   - top:  8 pixels (row above, including 4 to the right of the block).
 *           Top-right (pixels 4..7) may be unavailable for some block
 *           positions; replicate top[3] in that case.
 *   - left: 4 pixels (column to the left).
 *   - tl:   1 pixel (top-left corner).
 */
enum {
    I4_VERTICAL          = 0,  /* needs top */
    I4_HORIZONTAL        = 1,  /* needs left */
    I4_DC                = 2,  /* always available */
    I4_DIAG_DOWN_LEFT    = 3,  /* needs top (top-right replicated if absent) */
    I4_DIAG_DOWN_RIGHT   = 4,  /* needs top, left, tl */
    I4_VERTICAL_RIGHT    = 5,  /* needs top, left, tl */
    I4_HORIZONTAL_DOWN   = 6,  /* needs top, left, tl */
    I4_VERTICAL_LEFT     = 7,  /* needs top (top-right replicated if absent) */
    I4_HORIZONTAL_UP     = 8,  /* needs left */
};

/* Intra chroma 8x8 modes (spec 8.3.4). */
enum {
    IC_DC          = 0,
    IC_HORIZONTAL  = 1,
    IC_VERTICAL    = 2,
    IC_PLANE       = 3,
};

/* Predict a 16x16 luma block.
 *  top[0..15]  — row of 16 reconstructed pixels above the block.
 *  left[0..15] — column of 16 reconstructed pixels to the left of the block.
 *  tl          — single pixel at the top-left corner.
 *  avail_*     — 1 if that neighbor is available (within frame, and was
 *                already encoded), 0 otherwise.
 *  pred[0..255]— output, raster order (row-major). */
void predict_16x16(int mode,
                   const u8 top[16], const u8 left[16], u8 tl,
                   int avail_top, int avail_left, int avail_tl,
                   u8 pred[256]);

/* Predict an 8x8 chroma block. */
void predict_chroma_8x8(int mode,
                        const u8 top[8], const u8 left[8], u8 tl,
                        int avail_top, int avail_left, int avail_tl,
                        u8 pred[64]);

/* Predict a 4x4 luma block.
 *  top[0..7]   — 8 pixels of the row above the block. If avail_topright
 *                is 0 but avail_top is 1, the caller must replicate top[3]
 *                into top[4..7] before calling (matches spec 8.3.1.2.4).
 *  left[0..3]  — 4 pixels of the column to the left.
 *  tl          — top-left corner pixel.
 *  pred[0..15] — output, raster order. */
void predict_4x4(int mode,
                 const u8 top[8], const u8 left[4], u8 tl,
                 int avail_top, int avail_left, int avail_tl,
                 u8 pred[16]);

#endif
