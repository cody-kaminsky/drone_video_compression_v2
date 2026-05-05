/* line_buffer.h — recon-plane edge buffer for intra prediction.
 *
 * Replaces the full W*H recon plane with O(W) edge storage. For 1080p
 * NV12 this drops the recon footprint from ~3.1 MB to ~15 KB — small
 * enough to live in BRAM for the HLS port (architecture.txt §17).
 *
 * Layout:
 *   buf_y[2][MAX_W]   — two ping-pong banks of bottom-edge luma rows.
 *   buf_uv[2][MAX_W]  — same, for NV12-interleaved chroma.
 *   top_idx           — which bank holds the row-above-current (read side).
 *                       The other bank (1 - top_idx) is the write side for
 *                       the current MB row. lb_begin_mb_row() flips this.
 *   left_y/left_uv    — right column of the MB just to the left in the
 *                       current MB row.
 *   top_valid/left_valid — availability flags for the very first MB row /
 *                       very first MB column (where neighbors are off-edge
 *                       and reads must default to 128 per spec 8.3.x).
 *
 * Why the index instead of pointer-swapping the two banks: HLS rejects
 * pointer phi nodes ("Pointer points to an unknown underlying object").
 * An integer selector into a fixed 2-banked array is HLS-friendly.
 */
#ifndef DCC_LINE_BUFFER_H
#define DCC_LINE_BUFFER_H

#include "types.h"

typedef struct {
    /* Two ping-pong edge banks. Bank `top_idx` is the row above the
     * current MB row (read side); bank `1 - top_idx` accumulates the
     * current row (write side). lb_begin_mb_row() flips top_idx. */
    u8 buf_y [2][MAX_W];
    u8 buf_uv[2][MAX_W];

    /* Right column of MB(r, mb_c-1). Repopulated each MB by lb_commit_mb. */
    u8 left_y [16];
    u8 left_uv[16];   /* 8 chroma rows x 2 (UV interleaved) */

    int width;        /* frame width in luma samples (= chroma byte stride) */
    int top_idx;      /* 0 or 1 — selects which bank is "top" */
    int top_valid;    /* 1 once row 0 is finished; reads default to 128 if 0 */
    int left_valid;   /* 1 once mb_c >= 1 in the current row */
} line_buffer_t;

/* Initialize. Both banks start at undefined contents; the gather functions
 * consult top_valid before reading them. */
void lb_init(line_buffer_t *lb, int width);

/* Begin a new MB row. Flips top_idx so the bank we just filled becomes
 * the read side for the next row, and the previous read side is reused
 * for writes. Invalidates left_y/left_uv. Call once per MB row, before
 * encoding the first MB of the row. NOT called before row 0. */
void lb_begin_mb_row(line_buffer_t *lb);

/* Commit a finished MB. Writes the MB's bottom row to bank (1-top_idx)
 * at position [mb_c*16 .. mb_c*16+15] and copies the right column into
 * left_y/left_uv. recon_uv is NV12-interleaved (16 bytes per row x 8 rows). */
void lb_commit_mb(line_buffer_t *lb, int mb_c,
                  const u8 recon_y[256],
                  const u8 recon_uv[128]);

/* === Neighbor accessors ===
 * Each fills the same output arrays that the legacy gather_neighbors_*
 * functions populated, with identical avail/default semantics. */

void lb_gather_luma_16x16(const line_buffer_t *lb, int mb_c,
                          u8 top[16], u8 left[16], u8 *tl,
                          int *avail_top, int *avail_left, int *avail_tl);

void lb_gather_chroma_8x8(const line_buffer_t *lb, int mb_c,
                          u8 top_u[8], u8 left_u[8], u8 *tl_u,
                          u8 top_v[8], u8 left_v[8], u8 *tl_v,
                          int *avail_top, int *avail_left, int *avail_tl);

/* For I_4x4 per-block neighbor gather. The local recon_mb_y argument
 * carries neighbors that come from blocks already coded WITHIN the
 * current MB; the line buffer covers the cross-MB neighbors. */
void lb_gather_4x4(const line_buffer_t *lb, int blk_idx,
                   int mb_c, int mbs_w,
                   const u8 recon_mb_y[256],
                   u8 top[8], u8 left[4], u8 *tl,
                   int *avail_top, int *avail_left, int *avail_tl);

#endif
