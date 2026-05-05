/* line_buffer.c — see line_buffer.h for design. */

#include "line_buffer.h"
#include <string.h>

/* I_4x4 scan tables — duplicated from encoder.c. The line buffer only
 * needs (br, bc) per scan idx, no other state. */
static const u8 lb_i4_scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
static const u8 lb_i4_scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};

/* Top-right availability — same rule as encoder.c's i4_topright_avail.
 * Block 5 is the only one whose top-right comes from the MB diagonally
 * above-right; we don't have mb_r in the line buffer (top_valid stands
 * in for "row above exists"), so we encode the rule via mb_c only. */
static int lb_i4_tr_avail(int blk_idx, int mb_c, int mbs_w, int top_valid)
{
    switch (blk_idx) {
        case 3: case 7: case 11: case 13: case 15: return 0;
        case 5: return top_valid && (mb_c < mbs_w - 1);
        default: return 1;
    }
}

void lb_init(line_buffer_t *lb, int width,
             u8 *top_y, u8 *top_uv, u8 *next_y, u8 *next_uv)
{
    lb->top_y      = top_y;
    lb->top_uv     = top_uv;
    lb->next_y     = next_y;
    lb->next_uv    = next_uv;
    lb->width      = width;
    lb->top_valid  = 0;
    lb->left_valid = 0;
    /* Edge buffers don't need clearing; gather functions consult top_valid
     * before reading them. */
}

void lb_begin_mb_row(line_buffer_t *lb)
{
    /* Promote next → top by pointer swap. The just-promoted top now holds
     * the bottom row of the MB row we just finished; the just-cleared next
     * is the buffer the upcoming row will fill. */
    u8 *swap;
    swap = lb->top_y;  lb->top_y  = lb->next_y;  lb->next_y  = swap;
    swap = lb->top_uv; lb->top_uv = lb->next_uv; lb->next_uv = swap;
    /* After the first row finishes, top_valid stays 1 forever. */
    lb->top_valid  = lb->top_valid || (lb->next_y != NULL);
    lb->left_valid = 0;
    /* Note: lb_init starts both flags at 0; the first call to lb_begin_mb_row
     * (before encoding row 0) will flip top_valid -- but we want top_valid
     * to remain 0 for row 0, since there is no row above. So callers should
     * NOT call lb_begin_mb_row before row 0; instead call it BEFORE row 1
     * onwards. See encoder.c usage. */
}

void lb_commit_mb(line_buffer_t *lb, int mb_c,
                  const u8 recon_y[256],
                  const u8 recon_uv[128])
{
    /* Bottom row of luma: recon_y[15 * 16 + j] for j = 0..15. */
    memcpy(&lb->next_y[mb_c * 16], &recon_y[15 * 16], 16);

    /* Bottom row of NV12 chroma: recon_uv at chroma row 7, all 16 bytes
     * (= 8 UV pairs). */
    memcpy(&lb->next_uv[mb_c * 16], &recon_uv[7 * 16], 16);

    /* Right column of luma (col 15 of every row). */
    for (int r = 0; r < 16; r++)
        lb->left_y[r] = recon_y[r * 16 + 15];

    /* Right column of chroma — last UV pair of every chroma row.
     * NV12: each chroma row is 16 bytes; cols 14, 15 are the last UV pair. */
    for (int r = 0; r < 8; r++) {
        lb->left_uv[r * 2 + 0] = recon_uv[r * 16 + 14];  /* U */
        lb->left_uv[r * 2 + 1] = recon_uv[r * 16 + 15];  /* V */
    }

    lb->left_valid = 1;
}

/* === Neighbor gathers === */

void lb_gather_luma_16x16(const line_buffer_t *lb, int mb_c,
                          u8 top[16], u8 left[16], u8 *tl,
                          int *avail_top, int *avail_left, int *avail_tl)
{
    *avail_top  = lb->top_valid;
    *avail_left = (mb_c > 0) && lb->left_valid;
    *avail_tl   = (*avail_top && *avail_left);

    if (*avail_top)
        memcpy(top, &lb->top_y[mb_c * 16], 16);
    if (*avail_left)
        memcpy(left, lb->left_y, 16);
    *tl = (*avail_tl) ? lb->top_y[mb_c * 16 - 1] : (u8)128;
}

void lb_gather_chroma_8x8(const line_buffer_t *lb, int mb_c,
                          u8 top_u[8], u8 left_u[8], u8 *tl_u,
                          u8 top_v[8], u8 left_v[8], u8 *tl_v,
                          int *avail_top, int *avail_left, int *avail_tl)
{
    *avail_top  = lb->top_valid;
    *avail_left = (mb_c > 0) && lb->left_valid;
    *avail_tl   = (*avail_top && *avail_left);

    if (*avail_top) {
        const u8 *row = &lb->top_uv[mb_c * 16];   /* 8 UV pairs = 16 bytes */
        for (int j = 0; j < 8; j++) {
            top_u[j] = row[j * 2 + 0];
            top_v[j] = row[j * 2 + 1];
        }
    }
    if (*avail_left) {
        for (int i = 0; i < 8; i++) {
            left_u[i] = lb->left_uv[i * 2 + 0];
            left_v[i] = lb->left_uv[i * 2 + 1];
        }
    }
    if (*avail_tl) {
        /* Top-left UV pair: 2 bytes immediately preceding the current MB
         * in the top_uv row. */
        *tl_u = lb->top_uv[mb_c * 16 - 2];
        *tl_v = lb->top_uv[mb_c * 16 - 1];
    } else {
        *tl_u = 128; *tl_v = 128;
    }
}

void lb_gather_4x4(const line_buffer_t *lb, int blk_idx,
                   int mb_c, int mbs_w,
                   const u8 recon_mb_y[256],
                   u8 top[8], u8 left[4], u8 *tl,
                   int *avail_top, int *avail_left, int *avail_tl)
{
    int br = lb_i4_scan_br[blk_idx];
    int bc = lb_i4_scan_bc[blk_idx];

    *avail_top  = (br > 0) || lb->top_valid;
    *avail_left = (bc > 0) || ((mb_c > 0) && lb->left_valid);
    *avail_tl   = (*avail_top && *avail_left);

    /* top[0..3]: 4 pixels above the block. */
    if (*avail_top) {
        if (br > 0) {
            int ly = br*4 - 1;
            for (int j = 0; j < 4; j++)
                top[j] = recon_mb_y[ly * 16 + bc*4 + j];
        } else {
            const u8 *row = &lb->top_y[mb_c * 16 + bc*4];
            for (int j = 0; j < 4; j++) top[j] = row[j];
        }
    }

    /* top[4..7]: top-right 4 pixels (or replicated). */
    int tr_avail = lb_i4_tr_avail(blk_idx, mb_c, mbs_w, lb->top_valid);
    if (*avail_top && tr_avail) {
        if (br > 0) {
            int ly = br*4 - 1;
            int sx = bc*4 + 4;
            if (sx + 3 < 16) {
                for (int j = 0; j < 4; j++)
                    top[4 + j] = recon_mb_y[ly * 16 + sx + j];
            } else {
                /* Block 5 case: split between local MB and the line buffer
                 * (bottom row of MB above-right). */
                for (int j = 0; j < 4; j++) {
                    int gx = bc*4 + 4 + j;
                    if (gx < 16) top[4 + j] = recon_mb_y[ly * 16 + gx];
                    else         top[4 + j] = lb->top_y[mb_c * 16 + gx];
                }
            }
        } else {
            const u8 *row = &lb->top_y[mb_c * 16 + bc*4 + 4];
            for (int j = 0; j < 4; j++) top[4 + j] = row[j];
        }
    } else if (*avail_top) {
        u8 v = top[3];
        top[4] = v; top[5] = v; top[6] = v; top[7] = v;
    }

    /* left[0..3]: 4 pixels to the left of the block. */
    if (*avail_left) {
        if (bc > 0) {
            int lx = bc*4 - 1;
            for (int i = 0; i < 4; i++)
                left[i] = recon_mb_y[(br*4 + i) * 16 + lx];
        } else {
            for (int i = 0; i < 4; i++)
                left[i] = lb->left_y[br*4 + i];
        }
    }

    /* Top-left sample. */
    if (*avail_tl) {
        if (br > 0 && bc > 0) {
            *tl = recon_mb_y[(br*4 - 1) * 16 + (bc*4 - 1)];
        } else if (br > 0) {
            /* (br > 0, bc == 0): TL is the right column of the MB to the
             * left at row (br*4 - 1). */
            *tl = lb->left_y[br*4 - 1];
        } else if (bc > 0) {
            /* (br == 0, bc > 0): TL is in the row above, at column bc*4-1
             * within the current MB's column range. */
            *tl = lb->top_y[mb_c * 16 + bc*4 - 1];
        } else {
            /* (br == 0, bc == 0): TL is the corner pixel just above-left
             * of the MB. In the row above it's the last pixel of the
             * left-MB-above's bottom row, i.e., top_y[mb_c*16 - 1]. */
            *tl = lb->top_y[mb_c * 16 - 1];
        }
    } else {
        *tl = 128;
    }
}
