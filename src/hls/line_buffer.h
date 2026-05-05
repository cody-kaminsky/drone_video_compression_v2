/* line_buffer.h — recon-plane edge buffer for intra prediction.
 *
 * Replaces the full W*H recon plane with O(W) edge storage. For 1080p
 * NV12 this drops the recon footprint from ~3.1 MB to ~23 KB — small
 * enough to live in BRAM for the HLS port (architecture.txt §17).
 *
 * What we keep:
 *   top_y/top_uv      — bottom row of the MB row immediately above (read).
 *   next_y/next_uv    — bottom row of the current MB row (written as MBs
 *                       finish; swapped with top_y/top_uv at row end).
 *   left_y/left_uv    — right column of the MB just to the left in the
 *                       current MB row.
 *   top_valid/left_valid — availability flags for the very first MB row /
 *                       very first MB column (where neighbors are off-edge
 *                       and reads must default to 128 per spec 8.3.x).
 *
 * What we drop: every other row/column of the recon plane. Outside the
 * window described above, no MB ever reads recon for prediction (verified
 * by inspection of every gather_neighbors_* call).
 */
#ifndef DCC_LINE_BUFFER_H
#define DCC_LINE_BUFFER_H

#include "types.h"

typedef struct {
    /* Edge buffers — caller-provided storage of size `width` (luma) or
     * `width` (chroma — NV12 interleaves UV at full stride). */
    u8 *top_y;
    u8 *top_uv;
    u8 *next_y;
    u8 *next_uv;

    /* Right column of MB(r, mb_c-1). Repopulated each MB by lb_commit_mb. */
    u8 left_y [16];
    u8 left_uv[16];   /* 8 chroma rows x 2 (UV interleaved) */

    int width;        /* frame width in luma samples (= chroma byte stride) */
    int top_valid;    /* 1 once row 0 is finished; reads default to 128 if 0 */
    int left_valid;   /* 1 once mb_c >= 1 in the current row */
} line_buffer_t;

/* Initialize. Caller owns the four edge buffers (each at least `width`
 * bytes). After init, both top_valid and left_valid are 0. */
void lb_init(line_buffer_t *lb, int width,
             u8 *top_y, u8 *top_uv, u8 *next_y, u8 *next_uv);

/* Begin a new MB row. Promotes next_y/next_uv to top_y/top_uv (pointer
 * swap), invalidates left_y/left_uv. Call this once per MB row, before
 * encoding the first MB of the row. */
void lb_begin_mb_row(line_buffer_t *lb);

/* Commit a finished MB. Writes the MB's bottom row to next_y/next_uv at
 * position [mb_c*16 .. mb_c*16+15], copies the right column into left_y/
 * left_uv. recon_uv is NV12-interleaved (16 bytes per row x 8 rows). */
void lb_commit_mb(line_buffer_t *lb, int mb_c,
                  const u8 recon_y[256],
                  const u8 recon_uv[128]);

/* === Neighbor accessors ===
 * Each fills the same output arrays that the legacy gather_neighbors_*
 * functions populated, with identical avail/default semantics. Drop-in
 * replacements at the mb_fetch call site. */

void lb_gather_luma_16x16(const line_buffer_t *lb, int mb_c,
                          u8 top[16], u8 left[16], u8 *tl,
                          int *avail_top, int *avail_left, int *avail_tl);

void lb_gather_chroma_8x8(const line_buffer_t *lb, int mb_c,
                          u8 top_u[8], u8 left_u[8], u8 *tl_u,
                          u8 top_v[8], u8 left_v[8], u8 *tl_v,
                          int *avail_top, int *avail_left, int *avail_tl);

/* For I_4x4 per-block neighbor gather. Fills the same outputs as the
 * legacy gather_neighbors_4x4. The local recon_mb_y argument carries
 * neighbors that come from blocks already coded WITHIN the current MB;
 * the line buffer covers the cross-MB neighbors (br=0 top row, bc=0 left
 * column, top-left, top-right). */
void lb_gather_4x4(const line_buffer_t *lb, int blk_idx,
                   int mb_c, int mbs_w,
                   const u8 recon_mb_y[256],
                   u8 top[8], u8 left[4], u8 *tl,
                   int *avail_top, int *avail_left, int *avail_tl);

#endif
