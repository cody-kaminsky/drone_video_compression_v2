/* line_buffer.h — frame-edge state for intra prediction and CAVLC.
 *
 * Replaces ALL of the previous frame-scale "arenas" (recon plane, nC
 * grids, mode4 grid) with O(W) edge storage. Every read pattern in the
 * encoder only ever touches the row immediately above and the column
 * immediately left of the current MB; the rest of the frame is dead
 * data. So we keep just those edges.
 *
 * Footprint at MAX_W=3840:
 *   recon banks   2 * MAX_W * 2  = 15 KB
 *   nC luma       2 * MAX_W/4    = 1.9 KB
 *   nC chroma     2 * MAX_W/8 *2 = 1.9 KB
 *   mode4         2 * MAX_W/4    = 1.9 KB
 *   left regs     ~50 B
 *   total         ~21 KB
 *
 * vs previous arenas: ~3.3 MB at MAX_W. About 150x reduction.
 *
 * Banking: two banks per dimension, ping-pong via top_idx (0/1).
 * Bank top_idx = "the row above" (read side). Bank top_idx^1 = "the
 * row being filled" (write side). lb_begin_mb_row() flips top_idx.
 *
 * Why an integer index instead of pointer-swapping the banks: HLS
 * rejects pointer phi nodes ("Pointer points to an unknown underlying
 * object"). An integer selector into a fixed 2-banked array binds
 * cleanly to BRAM.
 */
#ifndef DCC_LINE_BUFFER_H
#define DCC_LINE_BUFFER_H

#include "types.h"

/* Per-row counts of 4x4 blocks. Frame width / 4 luma blocks per row,
 * /8 chroma (NV12 chroma is half-height so chroma 4x4 grid is half
 * resolution in both dims; here we just track per-row size). */
#define LB_LUMA_W4   (MAX_W / 4)
#define LB_CHROMA_W4 (MAX_W / 8)

typedef struct {
    /* === Recon edge banks ===
     * Bottom-row pixels of the MB row above (read by current row's MBs)
     * and the MB row being built (will become top after lb_begin_mb_row). */
    u8 buf_y [2][MAX_W];
    u8 buf_uv[2][MAX_W];

    /* Right column of luma/chroma pixels of the just-finished MB in the
     * current row. Read by the next MB's left-neighbor accesses. */
    u8 left_y [16];
    u8 left_uv[16];   /* 8 chroma rows x 2 (UV interleaved) */

    /* === nC line buffers ===
     * Per-4x4-block CAVLC TotalCoeff context. Same banking as recon. */
    u8 nc_y_top [2][LB_LUMA_W4];     /* 4 luma 4x4 blocks per MB-col */
    u8 nc_u_top [2][LB_CHROMA_W4];   /* 2 chroma 4x4 U blocks per MB-col */
    u8 nc_v_top [2][LB_CHROMA_W4];

    /* Right column of nC values for the just-finished MB in current row.
     * 4 luma rows + 2 chroma rows of 4x4 blocks. */
    u8 nc_y_left [4];
    u8 nc_u_left [2];
    u8 nc_v_left [2];

    /* === mode4 line buffer ===
     * Per-4x4-block intraMxMPredMode (0..8) for I_4x4 mode prediction.
     * For 4x4 blocks inside an I_16x16 MB the stored value is I4_DC
     * (= 2) per spec 8.3.1.1 — "I_16x16 neighbor contributes effective
     * mode 2 (DC) to the Min(top, left) computation". */
    u8 mode4_y_top [2][LB_LUMA_W4];
    u8 mode4_y_left[4];

    /* === Common === */
    int width;        /* frame width in luma samples (= chroma byte stride) */
    int top_idx;      /* 0 or 1 — selects which bank is "top" */
    int top_valid;    /* 1 once row 0 is finished; reads default to 0/128 */
    int left_valid;   /* 1 once mb_c >= 1 in the current row */
} line_buffer_t;

/* Initialize. After init, top_valid = left_valid = 0. */
void lb_init(line_buffer_t *lb, int width);

/* Begin a new MB row. Flips top_idx so the bank we just filled becomes
 * the read side, invalidates left edge. Call once per MB row, before
 * encoding the first MB of the row. NOT called before row 0. */
void lb_begin_mb_row(line_buffer_t *lb);

/* Commit a finished MB's recon pixels into the line buffer.
 * recon_uv is NV12-interleaved (16 bytes per row x 8 rows). */
void lb_commit_recon(line_buffer_t *lb, int mb_c,
                     const u8 recon_y[256],
                     const u8 recon_uv[128]);

/* Commit a finished MB's nC and mode4 values into the line buffer.
 *   nc_y_local[16]      — per-4x4-block luma nC, raster (br*4+bc).
 *   nc_u_local[4]       — per-4x4-block chroma U nC, raster (br*2+bc).
 *   nc_v_local[4]       — per-4x4-block chroma V nC.
 *   mode4_y_local[16]   — per-4x4-block intraMxMPredMode. */
void lb_commit_nc(line_buffer_t *lb, int mb_c,
                  const u8 nc_y_local   [16],
                  const u8 nc_u_local   [4],
                  const u8 nc_v_local   [4],
                  const u8 mode4_y_local[16]);

/* === Recon neighbor gathers (unchanged from earlier) === */

void lb_gather_luma_16x16(const line_buffer_t *lb, int mb_c,
                          u8 top[16], u8 left[16], u8 *tl,
                          int *avail_top, int *avail_left, int *avail_tl);

void lb_gather_chroma_8x8(const line_buffer_t *lb, int mb_c,
                          u8 top_u[8], u8 left_u[8], u8 *tl_u,
                          u8 top_v[8], u8 left_v[8], u8 *tl_v,
                          int *avail_top, int *avail_left, int *avail_tl);

void lb_gather_4x4(const line_buffer_t *lb, int blk_idx,
                   int mb_c, int mbs_w,
                   const u8 recon_mb_y[256],
                   u8 top[8], u8 left[4], u8 *tl,
                   int *avail_top, int *avail_left, int *avail_tl);

/* === nC / mode4 readers ===
 * Inline because they're hot — every CAVLC block emit reads them at
 * least twice. col_x4 is in 4x4-block units (= mb_c*4 + bc). */

static inline int lb_nc_y_top(const line_buffer_t *lb, int col_x4) {
    return lb->top_valid ? lb->nc_y_top[lb->top_idx][col_x4] : 0;
}
static inline int lb_nc_u_top(const line_buffer_t *lb, int col_x4) {
    return lb->top_valid ? lb->nc_u_top[lb->top_idx][col_x4] : 0;
}
static inline int lb_nc_v_top(const line_buffer_t *lb, int col_x4) {
    return lb->top_valid ? lb->nc_v_top[lb->top_idx][col_x4] : 0;
}
static inline int lb_mode4_y_top(const line_buffer_t *lb, int col_x4) {
    /* When top neighbor doesn't exist, return I4_DC = 2 (spec default). */
    return lb->top_valid ? lb->mode4_y_top[lb->top_idx][col_x4] : 2;
}

#endif
