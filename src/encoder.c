/* encoder.c — main encoder loop and per-MB processing.
 *
 * v2: tries both I_16x16 and I_4x4 per MB, picks the path that produces
 * fewer estimated bits. Uses SATD (Hadamard-domain sum of abs) for the
 * intra mode decision within each path — closer to true coding cost than
 * SAD with similar compute. Chroma path is independent (always 8x8 DC).
 */

#include "encoder.h"
#include "transform.h"
#include "quant.h"
#include "intra.h"
#include "cavlc.h"
#include "bitstream.h"
#include "nal.h"

#include <string.h>
#include <math.h>

/* ===== static arena =====
 * Bounded per-frame buffers, sized for the architecture max (MAX_W x MAX_H).
 * Replacing malloc/free in the encode path is the M2 transition step toward
 * FPGA-friendly C: fixed memory footprint, no allocator dependency, and
 * predictable static analysis for the HLS port.
 */

#define ARENA_LUMA_W4     (MAX_W / 4)
#define ARENA_LUMA_H4     (MAX_H / 4)
#define ARENA_CHROMA_W4   (MAX_W / 8)
#define ARENA_CHROMA_H4   (MAX_H / 8)

/* RBSP slice payload buffer.
 * 8 MB comfortably holds 4K at sane QPs and any 1080p QP. The bitstream-
 * emitting path returns -6 on overflow if very low QPs blow the budget. */
#define ARENA_RBSP_BYTES  (8 * 1024 * 1024)

static u8  arena_recon_y    [MAX_W * MAX_H];
static u8  arena_recon_uv   [MAX_W * (MAX_H / 2)];
static int arena_luma_nc    [ARENA_LUMA_W4   * ARENA_LUMA_H4];
static int arena_chroma_u_nc[ARENA_CHROMA_W4 * ARENA_CHROMA_H4];
static int arena_chroma_v_nc[ARENA_CHROMA_W4 * ARENA_CHROMA_H4];
static u8  arena_rbsp       [ARENA_RBSP_BYTES];

/* ===== local helpers ===== */

static int clip_u8(int x)
{
    if (x < 0)   return 0;
    if (x > 255) return 255;
    return x;
}

static int abs_i(int x) { return x < 0 ? -x : x; }

static void residual_4x4(const u8 *src16, const u8 *pred16,
                         int br, int bc, i16 out[16])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            int idx = (br*4 + r) * 16 + (bc*4 + c);
            out[r*4 + c] = (i16)((int)src16[idx] - (int)pred16[idx]);
        }
}

static void residual_4x4_8x8(const u8 *src8, const u8 *pred8,
                             int br, int bc, i16 out[16])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            int idx = (br*4 + r) * 8 + (bc*4 + c);
            out[r*4 + c] = (i16)((int)src8[idx] - (int)pred8[idx]);
        }
}

static void recon_4x4(u8 *dst16, const u8 *pred16,
                      int br, int bc, const i32 res[16])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            int idx = (br*4 + r) * 16 + (bc*4 + c);
            int v = pred16[idx] + ((res[r*4 + c] + 32) >> 6);
            dst16[idx] = (u8)clip_u8(v);
        }
}

static void recon_4x4_8x8(u8 *dst8, const u8 *pred8,
                          int br, int bc, const i32 res[16])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            int idx = (br*4 + r) * 8 + (bc*4 + c);
            int v = pred8[idx] + ((res[r*4 + c] + 32) >> 6);
            dst8[idx] = (u8)clip_u8(v);
        }
}

/* I_4x4 variant: pred is a 16-element 4x4 block (NOT a 16x16 plane).
 * Writes the reconstructed block into dst_mb at MB position (br, bc). */
static void recon_4x4_local(u8 *dst_mb16x16, const u8 pred[16],
                            int br, int bc, const i32 res[16])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            int v = pred[r*4 + c] + ((res[r*4 + c] + 32) >> 6);
            dst_mb16x16[(br*4 + r) * 16 + (bc*4 + c)] = (u8)clip_u8(v);
        }
}

static void zigzag_4x4(const i16 in[16], i16 out[16])
{
    for (int i = 0; i < 16; i++)
        out[i] = in[zz_scan_4x4[i]];
}

static int sad_n(const u8 *a, const u8 *b, int n)
{
    int s = 0;
    for (int i = 0; i < n; i++) s += abs_i((int)a[i] - (int)b[i]);
    return s;
}

/* SATD on a 4x4 residual (sum of abs of Hadamard coefficients).
 * Approximates coding cost much better than SAD. */
static int satd_4x4(const u8 *src16, const u8 *pred16, int br, int bc, int stride)
{
    i16 res[16];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            int idx = (br*4 + r) * stride + (bc*4 + c);
            res[r*4 + c] = (i16)((int)src16[idx] - (int)pred16[idx]);
        }
    /* Cast residual into i32 buffer for ihadamard4x4 (which now takes i32). */
    i32 tmp_in[16];
    for (int i = 0; i < 16; i++) tmp_in[i] = res[i];
    i32 tmp_out[16];
    ihadamard4x4(tmp_in, tmp_out);  /* same butterfly as Hadamard */
    int s = 0;
    for (int i = 0; i < 16; i++) s += abs_i(tmp_out[i]);
    /* Standard SATD divides by 2; we omit since only relative values matter. */
    return s;
}

static int satd_16x16(const u8 *src256, const u8 *pred256)
{
    int s = 0;
    for (int br = 0; br < 4; br++)
        for (int bc = 0; bc < 4; bc++)
            s += satd_4x4(src256, pred256, br, bc, 16);
    return s;
}

/* ===== neighbor gather ===== */

static void gather_neighbors_luma(const u8 *recon_y, int stride,
                                  int mb_r, int mb_c, int width, int height,
                                  u8 top[16], u8 left[16], u8 *tl,
                                  int *avail_top, int *avail_left, int *avail_tl)
{
    int x = mb_c * 16, y = mb_r * 16;
    *avail_top  = (mb_r > 0);
    *avail_left = (mb_c > 0);
    *avail_tl   = (mb_r > 0 && mb_c > 0);
    if (*avail_top)  for (int j = 0; j < 16; j++) top[j]  = recon_y[(y - 1) * stride + x + j];
    if (*avail_left) for (int i = 0; i < 16; i++) left[i] = recon_y[(y + i) * stride + (x - 1)];
    *tl = (*avail_tl) ? recon_y[(y - 1) * stride + (x - 1)] : 128;
    (void)width; (void)height;
}

static void gather_neighbors_chroma(const u8 *recon_uv, int stride,
                                    int mb_r, int mb_c,
                                    u8 top_u[8], u8 left_u[8], u8 *tl_u,
                                    u8 top_v[8], u8 left_v[8], u8 *tl_v,
                                    int *avail_top, int *avail_left,
                                    int *avail_tl)
{
    int xc = mb_c * 8, yc = mb_r * 8;
    *avail_top  = (mb_r > 0);
    *avail_left = (mb_c > 0);
    *avail_tl   = (mb_r > 0 && mb_c > 0);
    if (*avail_top) {
        for (int j = 0; j < 8; j++) {
            top_u[j] = recon_uv[(yc - 1) * stride + (xc + j) * 2 + 0];
            top_v[j] = recon_uv[(yc - 1) * stride + (xc + j) * 2 + 1];
        }
    }
    if (*avail_left) {
        for (int i = 0; i < 8; i++) {
            left_u[i] = recon_uv[(yc + i) * stride + (xc - 1) * 2 + 0];
            left_v[i] = recon_uv[(yc + i) * stride + (xc - 1) * 2 + 1];
        }
    }
    *tl_u = (*avail_tl) ? recon_uv[(yc - 1) * stride + (xc - 1) * 2 + 0] : 128;
    *tl_v = (*avail_tl) ? recon_uv[(yc - 1) * stride + (xc - 1) * 2 + 1] : 128;
}

static void copy_in_mb_luma(const u8 *src, int stride, int mb_r, int mb_c, u8 out[256])
{
    int x = mb_c * 16, y = mb_r * 16;
    for (int i = 0; i < 16; i++) memcpy(&out[i*16], &src[(y + i) * stride + x], 16);
}

static void copy_in_mb_chroma_split(const u8 *src_uv, int stride,
                                    int mb_r, int mb_c, u8 out_u[64], u8 out_v[64])
{
    int xc = mb_c * 8, yc = mb_r * 8;
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++) {
            out_u[i*8 + j] = src_uv[(yc + i) * stride + (xc + j) * 2 + 0];
            out_v[i*8 + j] = src_uv[(yc + i) * stride + (xc + j) * 2 + 1];
        }
}

static void copy_out_mb_luma(u8 *dst, int stride, int mb_r, int mb_c, const u8 in[256])
{
    int x = mb_c * 16, y = mb_r * 16;
    for (int i = 0; i < 16; i++) memcpy(&dst[(y + i) * stride + x], &in[i*16], 16);
}

static void copy_out_mb_chroma_combine(u8 *dst_uv, int stride, int mb_r, int mb_c,
                                       const u8 in_u[64], const u8 in_v[64])
{
    int xc = mb_c * 8, yc = mb_r * 8;
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++) {
            dst_uv[(yc + i) * stride + (xc + j) * 2 + 0] = in_u[i*8 + j];
            dst_uv[(yc + i) * stride + (xc + j) * 2 + 1] = in_v[i*8 + j];
        }
}

/* ===== I_4x4 helpers =====
 *
 * 4x4 luma blocks within an MB are scanned in the order:
 *
 *      0  1 |  4  5
 *      2  3 |  6  7
 *      -----|-------
 *      8  9 | 12 13
 *     10 11 | 14 15
 *
 * Each block's neighbors come from a mix of:
 *   - the global recon_y buffer (for blocks bordering the MB edge)
 *   - the local recon_mb_y buffer (for blocks bordering already-coded
 *     blocks within the same MB)
 *
 * Top-right availability per block index (within current MB):
 *   blocks 3, 7, 11, 13, 15 — top-right is a not-yet-coded block,
 *   so the caller must replicate top[3] into top[4..7].
 */

/* Per-block (br, bc) in scan order. */
static const u8 i4_scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
static const u8 i4_scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};

/* Does block (br, bc) have natural top-right access (within MB or from
 * already-coded MB above-right)? blk_idx is the block scan index 0..15. */
static int i4_topright_avail(int blk_idx, int mb_r, int mb_c, int mbs_w)
{
    /* Blocks where top-right is NOT available regardless. */
    switch (blk_idx) {
        case 3: case 7: case 11: case 13: case 15:
            return 0;
        case 5:
            /* top-right is in MB above-right: available iff that MB exists */
            return (mb_r > 0 && mb_c < mbs_w - 1);
        default:
            /* Block 0,1,4: from MB above (avail_top); 2,6,8,9,10,12,14: from
             * already-coded blocks within current MB. Caller has confirmed
             * that the necessary neighbor exists. */
            return 1;
    }
}

/* Gather neighbors for a 4x4 block within the MB at scan index blk_idx.
 * Reads from global recon_y for outer-MB pixels and from local recon_mb_y
 * for inner-MB pixels (4x4 blocks already processed in this MB). */
static void gather_neighbors_4x4(int blk_idx, int mb_r, int mb_c, int mbs_w,
                                 const u8 *recon_y, int stride_recon,
                                 const u8 *recon_mb_y,
                                 u8 top[8], u8 left[4], u8 *tl,
                                 int *avail_top, int *avail_left, int *avail_tl)
{
    int br = i4_scan_br[blk_idx];
    int bc = i4_scan_bc[blk_idx];
    int x  = mb_c * 16 + bc * 4;       /* global x of block top-left */
    int y  = mb_r * 16 + br * 4;       /* global y of block top-left */

    *avail_top  = (br > 0) || (mb_r > 0);
    *avail_left = (bc > 0) || (mb_c > 0);
    *avail_tl   = (*avail_top && *avail_left);

    /* Top samples top[0..3]: 4 pixels above the block. */
    if (*avail_top) {
        if (br > 0) {
            /* From local recon_mb_y at (br*4 - 1, bc*4 + 0..3) */
            int ly = br*4 - 1;
            for (int j = 0; j < 4; j++)
                top[j] = recon_mb_y[ly * 16 + bc*4 + j];
        } else {
            /* From global recon_y at row (y-1), col x..x+3 */
            for (int j = 0; j < 4; j++)
                top[j] = recon_y[(y - 1) * stride_recon + x + j];
        }
    }

    /* Top-right samples top[4..7]: 4 pixels above and to the right. */
    int tr_avail = i4_topright_avail(blk_idx, mb_r, mb_c, mbs_w);
    if (*avail_top && tr_avail) {
        if (br > 0) {
            /* Inside MB or right edge of MB-above */
            int ly = br*4 - 1;
            int sx = bc*4 + 4;        /* may be >= 16 for block 5 only */
            if (sx + 3 < 16) {
                /* All 4 pixels are in local buffer */
                for (int j = 0; j < 4; j++)
                    top[4 + j] = recon_mb_y[ly * 16 + sx + j];
            } else {
                /* Some inside, some outside (only block-5-like cases). */
                for (int j = 0; j < 4; j++) {
                    int gx = bc*4 + 4 + j;
                    if (gx < 16) top[4 + j] = recon_mb_y[ly * 16 + gx];
                    else         top[4 + j] = recon_y[(y - 1) * stride_recon + (mb_c*16 + gx)];
                }
            }
        } else {
            /* Outside MB (row above) */
            for (int j = 0; j < 4; j++)
                top[4 + j] = recon_y[(y - 1) * stride_recon + x + 4 + j];
        }
    } else if (*avail_top) {
        /* Replicate top[3] per spec 8.3.1.2.4 */
        u8 v = top[3];
        top[4] = v; top[5] = v; top[6] = v; top[7] = v;
    }

    /* Left samples left[0..3]: 4 pixels to the left of the block. */
    if (*avail_left) {
        if (bc > 0) {
            int lx = bc*4 - 1;
            for (int i = 0; i < 4; i++)
                left[i] = recon_mb_y[(br*4 + i) * 16 + lx];
        } else {
            for (int i = 0; i < 4; i++)
                left[i] = recon_y[(y + i) * stride_recon + (x - 1)];
        }
    }

    /* Top-left sample. */
    if (*avail_tl) {
        if (br > 0 && bc > 0) {
            *tl = recon_mb_y[(br*4 - 1) * 16 + (bc*4 - 1)];
        } else if (br > 0) {
            *tl = recon_y[(mb_r*16 + br*4 - 1) * stride_recon + (mb_c*16 - 1)];
        } else if (bc > 0) {
            *tl = recon_y[(mb_r*16 - 1) * stride_recon + (mb_c*16 + bc*4 - 1)];
        } else {
            *tl = recon_y[(y - 1) * stride_recon + (x - 1)];
        }
    } else {
        *tl = 128;
    }
}

/* ===== try_i16x16_luma =====
 * Returns: filled recon_mb_y, returns estimated luma residual bits.
 * Side effects: writes per-block totalcoef counts to nc_out (16 entries). */
typedef struct {
    int total_bits;           /* CAVLC residual bits estimate (luma only) */
    int totalcoef[16];        /* per 4x4 block (raster order) */
    int dc_totalcoef;         /* I_16x16 DC block (16 coefs) */
    int mb_type;              /* 0=I_16x16, 1=I_4x4 */
    int mode16;               /* I_16x16 prediction mode (0..3) */
    int modes4[16];           /* I_4x4 per-block mode (raster order) */
} luma_path_t;

static void try_i16x16_luma(const u8 src_mb[256],
                            const u8 *recon_y, int stride_recon,
                            int mb_r, int mb_c, int width, int height,
                            int qp, u8 recon_mb_y[256], luma_path_t *out)
{
    u8 top[16], left[16], tl;
    int at, al, atl;
    gather_neighbors_luma(recon_y, stride_recon, mb_r, mb_c, width, height,
                          top, left, &tl, &at, &al, &atl);

    /* Pick I_16x16 mode by SATD. */
    int best_mode = I16_DC, best_cost = INT32_MAX;
    u8 pred_y[256], best_pred[256];
    for (int m = 0; m < 4; m++) {
        if (m == I16_VERTICAL   && !at)  continue;
        if (m == I16_HORIZONTAL && !al)  continue;
        if (m == I16_PLANE      && !(at && al && atl)) continue;
        predict_16x16(m, top, left, tl, at, al, atl, pred_y);
        int cost = satd_16x16(src_mb, pred_y);
        if (cost < best_cost) {
            best_cost = cost;
            best_mode = m;
            memcpy(best_pred, pred_y, 256);
        }
    }
    out->mode16 = best_mode;
    out->mb_type = 0;

    /* Forward path: per-4x4 DCT, extract DC, Hadamard+quant DC, quant AC.
     * IMPORTANT: quant_4x4 uses position-indexed MF tables, so the input
     * must be in raster order, not zigzag. Zigzag is only applied later
     * for CAVLC bit estimation. */
    i16 luma_dc[16];
    i16 ac_levels_raster[16][16];   /* raster-order quantized AC levels */
    i16 dc_levels[16];

    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            i16 res[16], dct[16];
            residual_4x4(src_mb, best_pred, br, bc, res);
            dct4x4(res, dct);
            int idx = br*4 + bc;
            luma_dc[idx] = dct[0];
            i16 dct_zd[16];
            memcpy(dct_zd, dct, sizeof(dct));
            dct_zd[0] = 0;
            quant_4x4(dct_zd, ac_levels_raster[idx], qp, 1);
        }
    }
    i32 dc_had[16];
    hadamard4x4(luma_dc, dc_had);
    quant_dc_4x4(dc_had, dc_levels, qp, 1);

    /* Inverse path. */
    i32 dc_dq[16], dc_recon[16];
    iquant_dc_4x4(dc_levels, dc_dq, qp);
    ihadamard4x4(dc_dq, dc_recon);

    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            int idx = br*4 + bc;
            i32 ac_dq[16];
            iquant_4x4(ac_levels_raster[idx], ac_dq, qp);
            ac_dq[0] = dc_recon[idx];
            i32 res[16];
            idct4x4(ac_dq, res);
            recon_4x4(recon_mb_y, best_pred, br, bc, res);
        }
    }

    /* Bit estimate.
     * MB header for I_16x16: mb_type + intra_chroma_mode + mb_qp_delta. */
    int bits = 6 + 3 + 1;

    /* DC block (16 coefs) — already in raster grid; zigzag for CAVLC. */
    {
        i16 zz[16];
        zigzag_4x4(dc_levels, zz);
        bits += cavlc_estimate_block_bits(zz, 16, BLK_LUMA_DC_16x16, 0);
    }

    /* 16 AC blocks: zigzag the raster levels for CAVLC, skip DC slot. */
    for (int idx = 0; idx < 16; idx++) {
        i16 zz[16];
        zigzag_4x4(ac_levels_raster[idx], zz);
        bits += cavlc_estimate_block_bits(&zz[1], 15, BLK_LUMA_AC, 0);
        out->totalcoef[idx] = count_nonzero(&zz[1], 15);
    }

    out->total_bits = bits;
}

/* try_i4x4_luma: the I_4x4 path. */
static void try_i4x4_luma(const u8 src_mb[256],
                          const u8 *recon_y, int stride_recon,
                          int mb_r, int mb_c, int mbs_w,
                          int qp, u8 recon_mb_y[256], luma_path_t *out)
{
    int bits = 0;
    /* MB header: mb_type=0 (I_4x4) is shorter, plus per-block mode signaling. */
    bits += 1;            /* mb_type Exp-Golomb */
    bits += 3;            /* intra_chroma_pred_mode */
    bits += 1;            /* mb_qp_delta */

    out->mb_type = 1;

    for (int blk = 0; blk < 16; blk++) {
        int br = i4_scan_br[blk];
        int bc = i4_scan_bc[blk];

        u8 top[8] = {0}, left[4] = {0}, tl = 128;
        int at, al, atl;
        gather_neighbors_4x4(blk, mb_r, mb_c, mbs_w, recon_y, stride_recon,
                             recon_mb_y, top, left, &tl, &at, &al, &atl);

        /* Pick best mode by SATD over a 4x4 block. */
        int best_mode = I4_DC, best_cost = INT32_MAX;
        u8 best_pred[16];
        u8 pred[16];

        for (int m = 0; m < 9; m++) {
            /* Mode availability checks (spec 8.3.1.2). */
            if ((m == I4_VERTICAL          || m == I4_DIAG_DOWN_LEFT  ||
                 m == I4_VERTICAL_LEFT) && !at)  continue;
            if ((m == I4_HORIZONTAL        || m == I4_HORIZONTAL_UP) && !al) continue;
            if ((m == I4_DIAG_DOWN_RIGHT   || m == I4_VERTICAL_RIGHT ||
                 m == I4_HORIZONTAL_DOWN) && !(at && al && atl)) continue;

            predict_4x4(m, top, left, tl, at, al, atl, pred);

            /* SATD on residual. */
            i16 r[16];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++) {
                    int idx = (br*4 + i) * 16 + (bc*4 + j);
                    r[i*4 + j] = (i16)((int)src_mb[idx] - (int)pred[i*4 + j]);
                }
            i32 ri32[16], satd_out[16];
            for (int k = 0; k < 16; k++) ri32[k] = r[k];
            ihadamard4x4(ri32, satd_out);
            int cost = 0;
            for (int k = 0; k < 16; k++) cost += abs_i(satd_out[k]);

            /* Mode-signaling cost (rough): 1 bit if "predicted" mode used,
             * else 4 bits. We can't know predicted-mode cheaply here, so
             * apply a flat 4-bit penalty for non-best modes — close enough. */
            cost += 4 << (qp / 6);   /* lambda-scaled bias against mode bits */

            if (cost < best_cost) {
                best_cost = cost;
                best_mode = m;
                memcpy(best_pred, pred, 16);
            }
        }

        out->modes4[blk] = best_mode;

        /* Forward: residual → DCT → quant (in RASTER order). */
        i16 res[16], dct[16], levels_raster[16];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                int idx = (br*4 + i) * 16 + (bc*4 + j);
                res[i*4 + j] = (i16)((int)src_mb[idx] - (int)best_pred[i*4 + j]);
            }
        dct4x4(res, dct);
        quant_4x4(dct, levels_raster, qp, 1);

        /* Inverse: dequant → iDCT → recon. best_pred is a 16-element 4x4
         * (the I_4x4 prediction), NOT a 16x16 plane — use the _local
         * variant that indexes pred by r*4+c. */
        i32 dq[16];
        iquant_4x4(levels_raster, dq, qp);
        i32 res_recon[16];
        idct4x4(dq, res_recon);
        recon_4x4_local(recon_mb_y, best_pred, br, bc, res_recon);

        /* Bit estimate. CAVLC operates on zigzag-ordered levels. */
        bits += 4;     /* mode signaling: ~1 bit (predicted) or 4 (explicit) */
        i16 zz[16];
        zigzag_4x4(levels_raster, zz);
        bits += cavlc_estimate_block_bits(zz, 16, BLK_LUMA_FULL, 0);

        out->totalcoef[br*4 + bc] = count_nonzero(zz, 16);
    }

    out->total_bits = bits;
}

/* ===== nC state ===== */
typedef struct {
    int *luma_nc;
    int *chroma_u_nc;
    int *chroma_v_nc;
    int luma_w4, luma_h4;
    int chroma_w4, chroma_h4;
} nc_state_t;

/* Encode one MB. Returns total bits (luma + chroma + header). */
static int encode_mb(const u8 *src_y,  int stride_y,
                     const u8 *src_uv, int stride_uv,
                     u8 *recon_y, int recon_stride_y,
                     u8 *recon_uv, int recon_stride_uv,
                     int mb_r, int mb_c, int width, int height, int mbs_w,
                     int qp_y, int qp_c, nc_state_t *ncs)
{
    /* Source MB. */
    u8 src_mb_y[256];
    u8 src_mb_u[64], src_mb_v[64];
    copy_in_mb_luma(src_y, stride_y, mb_r, mb_c, src_mb_y);
    copy_in_mb_chroma_split(src_uv, stride_uv, mb_r, mb_c, src_mb_u, src_mb_v);

    /* ===== try I_16x16 ===== */
    u8 recon_mb_a[256];
    luma_path_t info_a = {0};
    try_i16x16_luma(src_mb_y, recon_y, recon_stride_y,
                    mb_r, mb_c, width, height, qp_y, recon_mb_a, &info_a);

    /* ===== try I_4x4 ===== */
    u8 recon_mb_b[256];
    luma_path_t info_b = {0};
    try_i4x4_luma(src_mb_y, recon_y, recon_stride_y, mb_r, mb_c, mbs_w,
                  qp_y, recon_mb_b, &info_b);

    /* Pick winner by total estimated bits. */
    int pick_b = (info_b.total_bits < info_a.total_bits);
    luma_path_t *winner = pick_b ? &info_b : &info_a;
    u8 *recon_mb_y     = pick_b ? recon_mb_b : recon_mb_a;

    copy_out_mb_luma(recon_y, recon_stride_y, mb_r, mb_c, recon_mb_y);

    /* Update luma nC counts (per 4x4 block, raster order). */
    int luma_w4 = ncs->luma_w4;
    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            int gx = mb_c * 4 + bc;
            int gy = mb_r * 4 + br;
            ncs->luma_nc[gy * luma_w4 + gx] = winner->totalcoef[br*4 + bc];
        }
    }

    /* ===== chroma path (independent of luma path choice) ===== */
    u8 ctop_u[8], cleft_u[8], ctl_u;
    u8 ctop_v[8], cleft_v[8], ctl_v;
    int cat, cal, catl;
    gather_neighbors_chroma(recon_uv, recon_stride_uv, mb_r, mb_c,
                            ctop_u, cleft_u, &ctl_u, ctop_v, cleft_v, &ctl_v,
                            &cat, &cal, &catl);

    int best_mode_c = IC_DC, best_sad_c = INT32_MAX;
    u8 pred_u[64], pred_v[64], best_pred_u[64], best_pred_v[64];
    for (int m = 0; m < 4; m++) {
        if (m == IC_VERTICAL   && !cat) continue;
        if (m == IC_HORIZONTAL && !cal) continue;
        if (m == IC_PLANE      && !(cat && cal && catl)) continue;
        predict_chroma_8x8(m, ctop_u, cleft_u, ctl_u, cat, cal, catl, pred_u);
        predict_chroma_8x8(m, ctop_v, cleft_v, ctl_v, cat, cal, catl, pred_v);
        int s = sad_n(src_mb_u, pred_u, 64) + sad_n(src_mb_v, pred_v, 64);
        if (s < best_sad_c) {
            best_sad_c  = s;
            best_mode_c = m;
            memcpy(best_pred_u, pred_u, 64);
            memcpy(best_pred_v, pred_v, 64);
        }
    }
    (void)best_mode_c;

    i16 c_dc[2][4];
    i16 c_ac_levels[2][4][16];
    i16 c_dc_levels[2][4];
    const u8 *src_c[2]  = { src_mb_u,    src_mb_v };
    const u8 *pred_c[2] = { best_pred_u, best_pred_v };
    u8 recon_mb_c[2][64];

    for (int comp = 0; comp < 2; comp++) {
        for (int br = 0; br < 2; br++) {
            for (int bc = 0; bc < 2; bc++) {
                i16 res[16], dct[16];
                residual_4x4_8x8(src_c[comp], pred_c[comp], br, bc, res);
                dct4x4(res, dct);
                c_dc[comp][br*2 + bc] = dct[0];
                i16 dct_zd[16];
                memcpy(dct_zd, dct, sizeof(dct));
                dct_zd[0] = 0;
                /* Quant in RASTER order (matches MF table positions). */
                quant_4x4(dct_zd, c_ac_levels[comp][br*2 + bc], qp_c, 1);
            }
        }
        i16 dc_had[4];
        hadamard2x2(c_dc[comp], dc_had);
        quant_dc_2x2(dc_had, c_dc_levels[comp], qp_c, 1);

        i32 dc_dq[4], dc_recon[4];
        iquant_dc_2x2(c_dc_levels[comp], dc_dq, qp_c);
        ihadamard2x2(dc_dq, dc_recon);

        for (int br = 0; br < 2; br++) {
            for (int bc = 0; bc < 2; bc++) {
                int idx = br*2 + bc;
                i32 ac_dq[16];
                iquant_4x4(c_ac_levels[comp][idx], ac_dq, qp_c);
                ac_dq[0] = dc_recon[idx];
                i32 res[16];
                idct4x4(ac_dq, res);
                recon_4x4_8x8(recon_mb_c[comp], pred_c[comp], br, bc, res);
            }
        }
    }
    copy_out_mb_chroma_combine(recon_uv, recon_stride_uv, mb_r, mb_c,
                               recon_mb_c[0], recon_mb_c[1]);

    /* Chroma bits. */
    int bits = winner->total_bits;
    bits += cavlc_estimate_block_bits(c_dc_levels[0], 4, BLK_CHROMA_DC, -1);
    bits += cavlc_estimate_block_bits(c_dc_levels[1], 4, BLK_CHROMA_DC, -1);

    int chroma_w4 = ncs->chroma_w4;
    for (int comp = 0; comp < 2; comp++) {
        int *cnc = (comp == 0) ? ncs->chroma_u_nc : ncs->chroma_v_nc;
        for (int br = 0; br < 2; br++) {
            for (int bc = 0; bc < 2; bc++) {
                int idx = br*2 + bc;
                int gx = mb_c * 2 + bc;
                int gy = mb_r * 2 + br;
                int top_nc  = (gy > 0) ? cnc[(gy - 1) * chroma_w4 + gx] : 0;
                int left_nc = (gx > 0) ? cnc[gy * chroma_w4 + (gx - 1)] : 0;
                int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);
                /* Zigzag for CAVLC; skip DC slot (position 0). */
                i16 zz[16];
                zigzag_4x4(c_ac_levels[comp][idx], zz);
                bits += cavlc_estimate_block_bits(&zz[1], 15, BLK_CHROMA_AC, nC);
                cnc[gy * chroma_w4 + gx] = count_nonzero(&zz[1], 15);
            }
        }
    }

    return bits;
}

/* ===== PSNR helpers ===== */

static double psnr_plane(const u8 *a, int stride_a,
                         const u8 *b, int stride_b, int w, int h)
{
    i64 sse = 0;
    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++) {
            int d = (int)a[i * stride_a + j] - (int)b[i * stride_b + j];
            sse += (i64)(d * d);
        }
    if (sse == 0) return 99.0;
    double mse = (double)sse / ((double)w * h);
    return 10.0 * log10((255.0 * 255.0) / mse);
}

static double psnr_chroma_component(const u8 *a_uv, int stride_a,
                                    const u8 *b_uv, int stride_b,
                                    int w, int h, int comp_offset)
{
    i64 sse = 0;
    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++) {
            int d = (int)a_uv[i * stride_a + j*2 + comp_offset]
                  - (int)b_uv[i * stride_b + j*2 + comp_offset];
            sse += (i64)(d * d);
        }
    if (sse == 0) return 99.0;
    double mse = (double)sse / ((double)w * h);
    return 10.0 * log10((255.0 * 255.0) / mse);
}

/* ===== top-level encode_frame ===== */

int encode_frame(int width, int height, int qp,
                 const u8 *src_y,  int stride_y,
                 const u8 *src_uv, int stride_uv,
                 u8 *recon_y_out,  int recon_stride_y,
                 u8 *recon_uv_out, int recon_stride_uv,
                 encode_stats_t *stats)
{
    if (width  % 16 != 0) return -1;
    if (height % 16 != 0) return -1;
    if (qp < 0 || qp > 51) return -2;
    if (width > MAX_W || height > MAX_H) return -3;

    int mbs_w = width  / 16;
    int mbs_h = height / 16;
    int mb_count = mbs_w * mbs_h;
    int luma_w4 = mbs_w * 4;
    int luma_h4 = mbs_h * 4;
    int chroma_w4 = mbs_w * 2;
    int chroma_h4 = mbs_h * 2;

    u8 *recon_y_int  = arena_recon_y;
    u8 *recon_uv_int = arena_recon_uv;
    memset(recon_y_int,  128, (size_t)width * height);
    memset(recon_uv_int, 128, (size_t)width * (height / 2));

    nc_state_t ncs;
    ncs.luma_nc     = arena_luma_nc;
    ncs.chroma_u_nc = arena_chroma_u_nc;
    ncs.chroma_v_nc = arena_chroma_v_nc;
    ncs.luma_w4     = luma_w4;
    ncs.luma_h4     = luma_h4;
    ncs.chroma_w4   = chroma_w4;
    ncs.chroma_h4   = chroma_h4;
    memset(ncs.luma_nc,     0, (size_t)luma_w4   * luma_h4   * sizeof(int));
    memset(ncs.chroma_u_nc, 0, (size_t)chroma_w4 * chroma_h4 * sizeof(int));
    memset(ncs.chroma_v_nc, 0, (size_t)chroma_w4 * chroma_h4 * sizeof(int));

    int qp_c = chroma_qp(qp, 0);

    int total_residual_bits = 0;

    for (int r = 0; r < mbs_h; r++) {
        for (int c = 0; c < mbs_w; c++) {
            total_residual_bits += encode_mb(
                src_y, stride_y, src_uv, stride_uv,
                recon_y_int, width, recon_uv_int, width,
                r, c, width, height, mbs_w,
                qp, qp_c, &ncs);
        }
    }

    int header_bits = 30 + (150 / 30) + 64;
    int total_bits  = total_residual_bits + header_bits;

    double psnr_y = psnr_plane(src_y,  stride_y,  recon_y_int, width, width, height);
    double psnr_u = psnr_chroma_component(src_uv, stride_uv,
                                          recon_uv_int, width,
                                          width / 2, height / 2, 0);
    double psnr_v = psnr_chroma_component(src_uv, stride_uv,
                                          recon_uv_int, width,
                                          width / 2, height / 2, 1);

    if (recon_y_out)
        for (int i = 0; i < height; i++)
            memcpy(&recon_y_out[i * recon_stride_y], &recon_y_int[i * width], width);
    if (recon_uv_out)
        for (int i = 0; i < height / 2; i++)
            memcpy(&recon_uv_out[i * recon_stride_uv], &recon_uv_int[i * width], width);

    if (stats) {
        stats->psnr_y    = psnr_y;
        stats->psnr_u    = psnr_u;
        stats->psnr_v    = psnr_v;
        stats->psnr_avg  = (psnr_y * 6.0 + psnr_u + psnr_v) / 8.0;
        stats->total_bits = total_bits;
        stats->bpp       = (double)total_bits / ((double)width * height);
        stats->mb_count  = mb_count;
        stats->bytes_out = (total_bits + 7) / 8;
    }

    return 0;
}

/* ============================================================================
 * BITSTREAM-EMITTING ENCODER (M2)
 * ============================================================================
 * encode_mb_emit() processes one MB and writes a real H.264 slice payload
 * for it. Restricted to I_16x16 mode for v1 of M2 — no I_4x4 in the
 * emitted bitstream (yet). The internal forward+inverse paths still apply
 * the same math as the estimate-only path, so PSNR matches the decoder's
 * reconstruction.
 */

static int encode_mb_emit(const u8 *src_y,  int stride_y,
                          const u8 *src_uv, int stride_uv,
                          u8 *recon_y, int recon_stride_y,
                          u8 *recon_uv, int recon_stride_uv,
                          int mb_r, int mb_c, int width, int height,
                          int qp_y, int qp_c, nc_state_t *ncs,
                          bitstream_t *bs)
{
    /* === Source MB === */
    u8 src_mb_y[256];
    u8 src_mb_u[64], src_mb_v[64];
    copy_in_mb_luma(src_y, stride_y, mb_r, mb_c, src_mb_y);
    copy_in_mb_chroma_split(src_uv, stride_uv, mb_r, mb_c, src_mb_u, src_mb_v);

    /* === I_16x16 mode decision === */
    u8 top[16], left[16], tl;
    int at, al, atl;
    gather_neighbors_luma(recon_y, recon_stride_y, mb_r, mb_c, width, height,
                          top, left, &tl, &at, &al, &atl);

    int mode16 = I16_DC, best_cost = INT32_MAX;
    u8 pred_y[256], best_pred[256];
    for (int m = 0; m < 4; m++) {
        if (m == I16_VERTICAL   && !at)  continue;
        if (m == I16_HORIZONTAL && !al)  continue;
        if (m == I16_PLANE      && !(at && al && atl)) continue;
        predict_16x16(m, top, left, tl, at, al, atl, pred_y);
        int cost = satd_16x16(src_mb_y, pred_y);
        if (cost < best_cost) { best_cost = cost; mode16 = m;
                                memcpy(best_pred, pred_y, 256); }
    }

    /* === Forward + inverse luma === */
    i16 luma_dc[16];
    i16 ac_levels_raster[16][16];
    i16 dc_levels[16];

    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            i16 res[16], dct[16];
            residual_4x4(src_mb_y, best_pred, br, bc, res);
            dct4x4(res, dct);
            int idx = br*4 + bc;
            luma_dc[idx] = dct[0];
            i16 dct_zd[16];
            memcpy(dct_zd, dct, sizeof(dct));
            dct_zd[0] = 0;
            quant_4x4(dct_zd, ac_levels_raster[idx], qp_y, 1);
        }
    }
    i32 dc_had[16];
    hadamard4x4(luma_dc, dc_had);
    quant_dc_4x4(dc_had, dc_levels, qp_y, 1);

    /* Inverse path → recon_mb_y */
    i32 dc_dq[16], dc_recon[16];
    iquant_dc_4x4(dc_levels, dc_dq, qp_y);
    ihadamard4x4(dc_dq, dc_recon);

    u8 recon_mb_y[256];
    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            int idx = br*4 + bc;
            i32 ac_dq[16];
            iquant_4x4(ac_levels_raster[idx], ac_dq, qp_y);
            ac_dq[0] = dc_recon[idx];
            i32 res[16];
            idct4x4(ac_dq, res);
            recon_4x4(recon_mb_y, best_pred, br, bc, res);
        }
    }
    copy_out_mb_luma(recon_y, recon_stride_y, mb_r, mb_c, recon_mb_y);

    /* === Chroma === */
    u8 ctop_u[8], cleft_u[8], ctl_u, ctop_v[8], cleft_v[8], ctl_v;
    int cat, cal, catl;
    gather_neighbors_chroma(recon_uv, recon_stride_uv, mb_r, mb_c,
                            ctop_u, cleft_u, &ctl_u, ctop_v, cleft_v, &ctl_v,
                            &cat, &cal, &catl);

    int mode_c = IC_DC, best_sad_c = INT32_MAX;
    u8 pred_u[64], pred_v[64], best_pred_u[64], best_pred_v[64];
    for (int m = 0; m < 4; m++) {
        if (m == IC_VERTICAL   && !cat) continue;
        if (m == IC_HORIZONTAL && !cal) continue;
        if (m == IC_PLANE      && !(cat && cal && catl)) continue;
        predict_chroma_8x8(m, ctop_u, cleft_u, ctl_u, cat, cal, catl, pred_u);
        predict_chroma_8x8(m, ctop_v, cleft_v, ctl_v, cat, cal, catl, pred_v);
        int s = sad_n(src_mb_u, pred_u, 64) + sad_n(src_mb_v, pred_v, 64);
        if (s < best_sad_c) { best_sad_c = s; mode_c = m;
                              memcpy(best_pred_u, pred_u, 64);
                              memcpy(best_pred_v, pred_v, 64); }
    }

    i16 c_dc[2][4];
    i16 c_ac_levels[2][4][16];
    i16 c_dc_levels[2][4];
    const u8 *src_c[2]  = { src_mb_u,    src_mb_v };
    const u8 *pred_c[2] = { best_pred_u, best_pred_v };
    u8 recon_mb_c[2][64];

    for (int comp = 0; comp < 2; comp++) {
        for (int br = 0; br < 2; br++) {
            for (int bc = 0; bc < 2; bc++) {
                i16 res[16], dct[16];
                residual_4x4_8x8(src_c[comp], pred_c[comp], br, bc, res);
                dct4x4(res, dct);
                c_dc[comp][br*2 + bc] = dct[0];
                i16 dct_zd[16];
                memcpy(dct_zd, dct, sizeof(dct));
                dct_zd[0] = 0;
                quant_4x4(dct_zd, c_ac_levels[comp][br*2 + bc], qp_c, 1);
            }
        }
        i16 dc_had_c[4];
        hadamard2x2(c_dc[comp], dc_had_c);
        quant_dc_2x2(dc_had_c, c_dc_levels[comp], qp_c, 1);

        i32 dc_dq_c[4], dc_recon_c[4];
        iquant_dc_2x2(c_dc_levels[comp], dc_dq_c, qp_c);
        ihadamard2x2(dc_dq_c, dc_recon_c);

        for (int br = 0; br < 2; br++) {
            for (int bc = 0; bc < 2; bc++) {
                int idx = br*2 + bc;
                i32 ac_dq[16];
                iquant_4x4(c_ac_levels[comp][idx], ac_dq, qp_c);
                ac_dq[0] = dc_recon_c[idx];
                i32 res[16];
                idct4x4(ac_dq, res);
                recon_4x4_8x8(recon_mb_c[comp], pred_c[comp], br, bc, res);
            }
        }
    }
    copy_out_mb_chroma_combine(recon_uv, recon_stride_uv, mb_r, mb_c,
                               recon_mb_c[0], recon_mb_c[1]);

    /* === Compute CBPs === */
    int cbp_luma = 0;
    for (int idx = 0; idx < 16 && !cbp_luma; idx++) {
        for (int k = 1; k < 16; k++) {
            if (ac_levels_raster[idx][k] != 0) { cbp_luma = 1; break; }
        }
    }
    int chroma_dc_nz = 0, chroma_ac_nz = 0;
    for (int comp = 0; comp < 2; comp++) {
        for (int k = 0; k < 4; k++) if (c_dc_levels[comp][k] != 0) chroma_dc_nz = 1;
        for (int idx = 0; idx < 4; idx++)
            for (int k = 1; k < 16; k++)
                if (c_ac_levels[comp][idx][k] != 0) chroma_ac_nz = 1;
    }
    int cbp_chroma = chroma_ac_nz ? 2 : (chroma_dc_nz ? 1 : 0);

    int start_bits = bs->byte_pos * 8 + bs->n_in_cur;

    /* === Emit MB syntax ===
     * Spec Table 7-11: mb_type = 1 + PredMode + 4*CBPChroma + 12*CBPLuma
     * (note: NOT 4*CBPLuma + 8*CBPChroma — got this wrong initially). */
    int mb_type = 1 + mode16 + 4 * cbp_chroma + 12 * cbp_luma;
    bs_put_ue(bs, mb_type);
    bs_put_ue(bs, mode_c);                 /* intra_chroma_pred_mode */
    bs_put_se(bs, 0);                      /* mb_qp_delta = 0 */

    /* === Emit luma DC block (always for I_16x16) ===
     * Spec 9.2.1.1: DC block uses luma4x4BlkIdx=0's neighbors for nC.
     * Block 0 lives at global 4x4 position (mb_r*4, mb_c*4); its left
     * neighbor is the rightmost 4x4 of the MB to the left, top is the
     * bottom 4x4 of the MB above. */
    int luma_w4_dc = ncs->luma_w4;
    {
        int gx0 = mb_c * 4;
        int gy0 = mb_r * 4;
        int top_nc_dc  = (gy0 > 0) ? ncs->luma_nc[(gy0 - 1) * luma_w4_dc + gx0] : 0;
        int left_nc_dc = (gx0 > 0) ? ncs->luma_nc[gy0 * luma_w4_dc + (gx0 - 1)] : 0;
        int nC_dc = cavlc_compute_nC(top_nc_dc, left_nc_dc, gy0 > 0, gx0 > 0);

        i16 zz[16];
        for (int k = 0; k < 16; k++) zz[k] = dc_levels[zz_scan_4x4[k]];
        cavlc_encode_block(bs, zz, 16, BLK_LUMA_DC_16x16, nC_dc);
    }

    /* === Emit luma AC blocks if cbp_luma ===
     * Spec 8.5.6: blocks within an MB are scanned in the I_4x4 zigzag scan
     * order (NOT raster). Each block's nC is derived from physical neighbors
     * (top, left), which by scan order have already been encoded. */
    int luma_w4 = ncs->luma_w4;
    static const int blk_scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
    static const int blk_scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};
    for (int s = 0; s < 16; s++) {
        int br = blk_scan_br[s];
        int bc = blk_scan_bc[s];
        int idx = br*4 + bc;
        i16 zz[16];
        for (int k = 0; k < 16; k++)
            zz[k] = ac_levels_raster[idx][zz_scan_4x4[k]];

        int gx = mb_c * 4 + bc;
        int gy = mb_r * 4 + br;
        int top_nc  = (gy > 0) ? ncs->luma_nc[(gy - 1) * luma_w4 + gx] : 0;
        int left_nc = (gx > 0) ? ncs->luma_nc[gy * luma_w4 + (gx - 1)] : 0;
        int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);

        if (cbp_luma) {
            cavlc_encode_block(bs, &zz[1], 15, BLK_LUMA_AC, nC);
        }
        ncs->luma_nc[gy * luma_w4 + gx] = count_nonzero(&zz[1], 15);
    }

    /* === Emit chroma DC blocks if cbp_chroma >= 1 === */
    if (cbp_chroma >= 1) {
        for (int comp = 0; comp < 2; comp++) {
            cavlc_encode_block(bs, c_dc_levels[comp], 4, BLK_CHROMA_DC, -1);
        }
    }

    /* === Emit chroma AC blocks if cbp_chroma == 2 === */
    int chroma_w4 = ncs->chroma_w4;
    for (int comp = 0; comp < 2; comp++) {
        int *cnc = (comp == 0) ? ncs->chroma_u_nc : ncs->chroma_v_nc;
        for (int br = 0; br < 2; br++) {
            for (int bc = 0; bc < 2; bc++) {
                int idx = br*2 + bc;
                i16 zz[16];
                for (int k = 0; k < 16; k++)
                    zz[k] = c_ac_levels[comp][idx][zz_scan_4x4[k]];

                int gx = mb_c * 2 + bc;
                int gy = mb_r * 2 + br;
                int top_nc  = (gy > 0) ? cnc[(gy - 1) * chroma_w4 + gx] : 0;
                int left_nc = (gx > 0) ? cnc[gy * chroma_w4 + (gx - 1)] : 0;
                int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);

                if (cbp_chroma == 2) {
                    cavlc_encode_block(bs, &zz[1], 15, BLK_CHROMA_AC, nC);
                }
                cnc[gy * chroma_w4 + gx] = count_nonzero(&zz[1], 15);
            }
        }
    }

    return bs->byte_pos * 8 + bs->n_in_cur - start_bits;
}

int encode_frame_h264(int width, int height, int qp,
                      const u8 *src_y,  int stride_y,
                      const u8 *src_uv, int stride_uv,
                      u8 *recon_y_out,  int recon_stride_y,
                      u8 *recon_uv_out, int recon_stride_uv,
                      u8 *bs_out, int bs_max_size, int frame_num,
                      encode_stats_t *stats)
{
    if (width  % 16 != 0) return -1;
    if (height % 16 != 0) return -1;
    if (qp < 0 || qp > 51) return -2;
    if (!bs_out)           return -3;
    if (width > MAX_W || height > MAX_H) return -3;

    int mbs_w = width  / 16;
    int mbs_h = height / 16;
    int mb_count = mbs_w * mbs_h;
    int luma_w4 = mbs_w * 4;
    int luma_h4 = mbs_h * 4;
    int chroma_w4 = mbs_w * 2;
    int chroma_h4 = mbs_h * 2;

    /* === SPS + PPS === */
    int dst_pos = 0;
    int n = nal_write_sps(bs_out + dst_pos, bs_max_size - dst_pos,
                          width, height, qp);
    if (n < 0) return -4;
    dst_pos += n;
    n = nal_write_pps(bs_out + dst_pos, bs_max_size - dst_pos, qp);
    if (n < 0) return -4;
    dst_pos += n;

    /* === Slice RBSP === */
    bitstream_t bs;
    bs_init(&bs, arena_rbsp, ARENA_RBSP_BYTES);

    /* Slice header */
    bs_put_ue(&bs, 0);                       /* first_mb_in_slice */
    bs_put_ue(&bs, 7);                       /* slice_type = 7 (I) */
    bs_put_ue(&bs, 0);                       /* pic_parameter_set_id */
    bs_put_bits(&bs, frame_num & 0xF, 4);    /* frame_num */
    bs_put_ue(&bs, frame_num & 0xF);         /* idr_pic_id */
    bs_put_bits(&bs, 0, 1);                  /* no_output_of_prior_pics_flag */
    bs_put_bits(&bs, 0, 1);                  /* long_term_reference_flag */
    bs_put_se(&bs, 0);                       /* slice_qp_delta */
    /* disable_deblocking_filter_idc = 1: in-loop deblocking is OFF.
     * Our internal recon doesn't deblock (deblock module deferred), so to
     * stay bit-exact with the decoder we tell the decoder not to deblock
     * either. Cost: some block-edge artifacts; deblocking is M3 work. */
    bs_put_ue(&bs, 1);                       /* disable_deblocking_filter_idc */
    /* When idc != 1, alpha/beta offsets are NOT signaled. */

    /* Recon buffers from the static arena. */
    u8 *recon_y_int  = arena_recon_y;
    u8 *recon_uv_int = arena_recon_uv;
    memset(recon_y_int,  128, (size_t)width * height);
    memset(recon_uv_int, 128, (size_t)width * (height / 2));

    nc_state_t ncs;
    ncs.luma_nc     = arena_luma_nc;
    ncs.chroma_u_nc = arena_chroma_u_nc;
    ncs.chroma_v_nc = arena_chroma_v_nc;
    ncs.luma_w4     = luma_w4;
    ncs.luma_h4     = luma_h4;
    ncs.chroma_w4   = chroma_w4;
    ncs.chroma_h4   = chroma_h4;
    memset(ncs.luma_nc,     0, (size_t)luma_w4   * luma_h4   * sizeof(int));
    memset(ncs.chroma_u_nc, 0, (size_t)chroma_w4 * chroma_h4 * sizeof(int));
    memset(ncs.chroma_v_nc, 0, (size_t)chroma_w4 * chroma_h4 * sizeof(int));

    int qp_c = chroma_qp(qp, 0);

    /* Per-MB encoding into the slice RBSP */
    for (int r = 0; r < mbs_h; r++) {
        for (int c = 0; c < mbs_w; c++) {
            encode_mb_emit(src_y, stride_y, src_uv, stride_uv,
                           recon_y_int, width, recon_uv_int, width,
                           r, c, width, height,
                           qp, qp_c, &ncs, &bs);
        }
    }

    bs_rbsp_trailing(&bs);
    int rbsp_len = bs_byte_count(&bs);
    if (bs.overflow) return -6;

    /* Wrap in IDR NAL */
    n = nal_emit_idr(bs_out + dst_pos, bs_max_size - dst_pos, arena_rbsp, rbsp_len);
    if (n < 0) return -6;
    dst_pos += n;

    /* PSNR vs source */
    double psnr_y = psnr_plane(src_y, stride_y, recon_y_int, width, width, height);
    double psnr_u = psnr_chroma_component(src_uv, stride_uv, recon_uv_int, width,
                                          width/2, height/2, 0);
    double psnr_v = psnr_chroma_component(src_uv, stride_uv, recon_uv_int, width,
                                          width/2, height/2, 1);

    if (recon_y_out)
        for (int i = 0; i < height; i++)
            memcpy(&recon_y_out[i * recon_stride_y], &recon_y_int[i * width], width);
    if (recon_uv_out)
        for (int i = 0; i < height/2; i++)
            memcpy(&recon_uv_out[i * recon_stride_uv], &recon_uv_int[i * width], width);

    if (stats) {
        stats->psnr_y    = psnr_y;
        stats->psnr_u    = psnr_u;
        stats->psnr_v    = psnr_v;
        stats->psnr_avg  = (psnr_y * 6.0 + psnr_u + psnr_v) / 8.0;
        stats->bytes_out = dst_pos;
        stats->total_bits = dst_pos * 8;
        stats->bpp       = (double)(dst_pos * 8) / ((double)width * height);
        stats->mb_count  = mb_count;
    }

    return 0;
}
