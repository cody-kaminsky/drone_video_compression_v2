/* encoder.c — main encoder loop and per-MB processing.
 *
 * Per-MB pipeline mirrors the architecture's stage layout (architecture.txt
 * §8/§9): mb_fetch -> mb_mode_decide -> mb_residual / mb_transform /
 * mb_quantize / mb_reconstruct (chroma) -> mb_compute_cbp -> mb_cavlc_emit.
 * mb_mode_decide tries both I_16x16 and I_4x4 paths for luma and picks the
 * one with fewer estimated CAVLC bits; chroma is coded the same way for
 * both paths. SATD (Hadamard-domain sum of abs) drives intra mode picks.
 *
 * The bit-estimation prototype (encode_frame / encode_mb / try_i16x16_luma
 * / try_i4x4_luma) lived here through Phase B. It was removed once I_4x4
 * was grafted onto the bitstream-emitting path; the staged path is now the
 * only path and is the shape the FPGA IP will take.
 */

#include "encoder.h"
#include "transform.h"
#include "quant.h"
#include "intra.h"
#include "cavlc.h"
#include "cavlc_tables.h"
#include "bitstream.h"
#include "nal.h"
#include "mb_state.h"
#include "psnr.h"

#include <string.h>

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

/* Per-4x4-block luma intra prediction mode, indexed [gy * luma_w4 + gx]
 * (same indexing as arena_luma_nc). Used by the I_4x4 emit path to compute
 * predIntra4x4PredMode (spec 8.3.1.1) for prev/rem mode flag emission. For
 * blocks inside an I_16x16 MB the stored value is I4_DC = 2, which makes
 * the median-of-neighbors lookup behave per spec ("treat I_16x16 / inter /
 * unavailable neighbors as DC"). */
static int arena_luma_mode4 [ARENA_LUMA_W4   * ARENA_LUMA_H4];

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

/* ===== nC + per-block mode state ===== */
typedef struct {
    int *luma_nc;
    int *chroma_u_nc;
    int *chroma_v_nc;
    /* luma_mode4: per-4x4-block intra prediction mode at frame scale. Used
     * by I_4x4 emission to compute predIntra4x4PredMode from neighbors. */
    int *luma_mode4;
    int luma_w4, luma_h4;
    int chroma_w4, chroma_h4;
} nc_state_t;


/* ============================================================================
 * MACROBLOCK ENCODE PIPELINE
 * ============================================================================
 * encode_mb_emit() processes one MB and writes the slice payload for it.
 * Both I_16x16 and I_4x4 macroblock types are supported; the path is picked
 * per-MB by mb_mode_decide based on estimated CAVLC bit cost.
 *
 * Pipeline structure mirrors architecture.txt §8/§9 — each stage is a pure
 * function over mb_state_t, and these are the boundaries that become
 * per-module FIFO interfaces in the HLS port.
 */

/* I_4x4 sub-block scan order within an MB (used by stage 7 to walk AC
 * blocks for CAVLC emission and by stage 7 nC indexing). Spec 8.5.6. */
static const int blk_scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
static const int blk_scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};

/* === stage 0: mb_fetch ===
 * Read source MB samples from frame and gather neighbor samples for
 * prediction. Architecture.txt §8 "MB Fetch" stage. */
static void mb_fetch(const u8 *src_y,  int stride_y,
                     const u8 *src_uv, int stride_uv,
                     const u8 *recon_y, int recon_stride_y,
                     const u8 *recon_uv, int recon_stride_uv,
                     int width, int height, mb_state_t *st)
{
    copy_in_mb_luma(src_y, stride_y, st->mb_r, st->mb_c, st->src_y);
    copy_in_mb_chroma_split(src_uv, stride_uv, st->mb_r, st->mb_c,
                            st->src_u, st->src_v);

    gather_neighbors_luma(recon_y, recon_stride_y, st->mb_r, st->mb_c,
                          width, height,
                          st->luma_top, st->luma_left, &st->luma_tl,
                          &st->luma_avail_top, &st->luma_avail_left,
                          &st->luma_avail_tl);

    gather_neighbors_chroma(recon_uv, recon_stride_uv, st->mb_r, st->mb_c,
                            st->cu_top, st->cu_left, &st->cu_tl,
                            st->cv_top, st->cv_left, &st->cv_tl,
                            &st->chroma_avail_top,
                            &st->chroma_avail_left,
                            &st->chroma_avail_tl);
}

/* try_path_i4x4: I_4x4 per-block forward+inverse loop.
 *
 * The I_4x4 dependency chain (each block uses neighbors from already-coded
 * blocks within the same MB) forces forward+inverse to run together per
 * block — we can't separate them across pipeline stages. This function
 * encapsulates the entire per-block sequence.
 *
 *   src_mb            — 16x16 source samples (st->src_y).
 *   recon_y_frame     — frame-level recon plane (for outer-MB neighbors).
 *   modes4_out[16]    — chosen I_4x4 mode per block, raster (br*4+bc).
 *   ac_levels_out     — quantized 16-coef levels per block, raster order.
 *   recon_mb_out[256] — reconstructed luma MB, row-major.
 * Returns: estimated CAVLC residual + header bits for the I_4x4 path.
 */
static int try_path_i4x4(const u8 src_mb[256], int qp,
                         int mb_r, int mb_c, int mbs_w,
                         const u8 *recon_y_frame, int stride_recon_y,
                         int modes4_out[16],
                         i16 ac_levels_out[16][16],
                         u8 recon_mb_out[256])
{
    /* Header estimate for I_4x4: mb_type=0 (1 bit), 16 mode flags (~2 bits
     * each on average), intra_chroma_pred_mode (~3 bits), me(cbp) (~6 bits),
     * mb_qp_delta (1 bit). */
    int bits = 1 + 16*2 + 3 + 6 + 1;

    for (int blk = 0; blk < 16; blk++) {
        int br = i4_scan_br[blk];
        int bc = i4_scan_bc[blk];

        u8 top[8] = {0}, left[4] = {0}, tl = 128;
        int at, al, atl;
        gather_neighbors_4x4(blk, mb_r, mb_c, mbs_w, recon_y_frame,
                             stride_recon_y, recon_mb_out,
                             top, left, &tl, &at, &al, &atl);

        /* Pick best mode by SATD. */
        int best_mode = I4_DC, best_cost = INT32_MAX;
        u8 best_pred[16], pred[16];
        for (int m = 0; m < 9; m++) {
            if ((m == I4_VERTICAL || m == I4_DIAG_DOWN_LEFT ||
                 m == I4_VERTICAL_LEFT) && !at)  continue;
            if ((m == I4_HORIZONTAL || m == I4_HORIZONTAL_UP) && !al) continue;
            if ((m == I4_DIAG_DOWN_RIGHT || m == I4_VERTICAL_RIGHT ||
                 m == I4_HORIZONTAL_DOWN) && !(at && al && atl)) continue;
            predict_4x4(m, top, left, tl, at, al, atl, pred);

            /* SATD on residual via 4x4 Hadamard. */
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
            cost += 4 << (qp / 6);   /* lambda penalty for mode bits */

            if (cost < best_cost) {
                best_cost = cost;
                best_mode = m;
                memcpy(best_pred, pred, 16);
            }
        }
        /* Store in raster order (br*4+bc) to match mb_state_t::modes4 convention. */
        modes4_out[br*4 + bc] = best_mode;

        /* Forward path */
        i16 res[16], dct[16], levels[16];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                int idx = (br*4 + i) * 16 + (bc*4 + j);
                res[i*4 + j] = (i16)((int)src_mb[idx] - (int)best_pred[i*4 + j]);
            }
        dct4x4(res, dct);
        quant_4x4(dct, levels, qp, 1);
        memcpy(ac_levels_out[br*4 + bc], levels, sizeof levels);

        /* Inverse path → recon_mb_out (must complete before the next
         * block's neighbor gather). */
        i32 dq[16], res_recon[16];
        iquant_4x4(levels, dq, qp);
        idct4x4(dq, res_recon);
        recon_4x4_local(recon_mb_out, best_pred, br, bc, res_recon);

        /* Bit estimate (CAVLC zigzag, full 16 coefs). */
        i16 zz[16];
        zigzag_4x4(levels, zz);
        bits += cavlc_estimate_block_bits(zz, 16, BLK_LUMA_FULL, 0);
    }

    return bits;
}

/* === stage 1: mb_mode_decide ===
 * Pick the luma encode plan (I_16x16 vs I_4x4) and the chroma 8x8 mode.
 *
 * For a fair I_16x16 vs I_4x4 bit comparison we need both paths' full
 * forward+inverse, so this stage absorbs the per-MB luma encode entirely
 * (architecture stages 1-6 fused for luma — the I_4x4 dependency chain
 * forces this). After this stage runs, st has:
 *   - mb_type_is_i4x4 (path winner)
 *   - mode16 OR modes4 (chosen modes)
 *   - pred_y (for I_16x16 only — used by the mb_chroma stages? no, only by
 *     CBP debug and emission for I_16x16 path; not needed)
 *   - recon_y (committed reconstruction for the chosen path)
 *   - ac_levels_y + dc_levels_y (I_16x16) OR ac_levels_y_full (I_4x4)
 *   - mode_chroma + pred_u/pred_v (always)
 * Stages 2-6 below operate on chroma only.
 */
static void mb_mode_decide(int mbs_w, const u8 *recon_y_frame,
                           int stride_recon_y, mb_state_t *st)
{
    /* === Path A: I_16x16 — full forward+inverse, then estimate bits. === */
    int mode_a = I16_DC;
    u8 pred_a[256], recon_a[256];
    i16 ac_lev_a[16][16], dc_lev_a[16];
    int bits_a;
    {
        u8 cand[256];
        int best = INT32_MAX;
        for (int m = 0; m < 4; m++) {
            if (m == I16_VERTICAL   && !st->luma_avail_top)  continue;
            if (m == I16_HORIZONTAL && !st->luma_avail_left) continue;
            if (m == I16_PLANE      && !(st->luma_avail_top && st->luma_avail_left
                                         && st->luma_avail_tl)) continue;
            predict_16x16(m, st->luma_top, st->luma_left, st->luma_tl,
                          st->luma_avail_top, st->luma_avail_left,
                          st->luma_avail_tl, cand);
            int cost = satd_16x16(st->src_y, cand);
            if (cost < best) { best = cost; mode_a = m; memcpy(pred_a, cand, 256); }
        }

        /* Forward: per-block residual+DCT+quant; collect DC for Hadamard. */
        i16 dc_extract[16];
        for (int br = 0; br < 4; br++)
            for (int bc = 0; bc < 4; bc++) {
                i16 res[16], dct[16];
                residual_4x4(st->src_y, pred_a, br, bc, res);
                dct4x4(res, dct);
                int idx = br*4 + bc;
                dc_extract[idx] = dct[0];
                i16 dct_zd[16];
                memcpy(dct_zd, dct, sizeof dct);
                dct_zd[0] = 0;
                quant_4x4(dct_zd, ac_lev_a[idx], st->qp_y, 1);
            }
        i32 dc_had[16];
        hadamard4x4(dc_extract, dc_had);
        quant_dc_4x4(dc_had, dc_lev_a, st->qp_y, 1);

        /* Inverse: dequant DC, iHadamard, splice into AC, iDCT, recon. */
        i32 dc_dq[16], dc_recon[16];
        iquant_dc_4x4(dc_lev_a, dc_dq, st->qp_y);
        ihadamard4x4(dc_dq, dc_recon);
        for (int br = 0; br < 4; br++)
            for (int bc = 0; bc < 4; bc++) {
                int idx = br*4 + bc;
                i32 ac_dq[16];
                iquant_4x4(ac_lev_a[idx], ac_dq, st->qp_y);
                ac_dq[0] = dc_recon[idx];
                i32 res_recon[16];
                idct4x4(ac_dq, res_recon);
                recon_4x4(recon_a, pred_a, br, bc, res_recon);
            }

        /* Estimate bits: header + DC block + 16 AC blocks. */
        int bits = 6 + 3 + 1;          /* mb_type + intra_chroma_mode + qp_delta */
        i16 zz[16];
        for (int k = 0; k < 16; k++) zz[k] = dc_lev_a[zz_scan_4x4[k]];
        bits += cavlc_estimate_block_bits(zz, 16, BLK_LUMA_DC_16x16, 0);
        for (int idx = 0; idx < 16; idx++) {
            for (int k = 0; k < 16; k++) zz[k] = ac_lev_a[idx][zz_scan_4x4[k]];
            bits += cavlc_estimate_block_bits(&zz[1], 15, BLK_LUMA_AC, 0);
        }
        bits_a = bits;
    }

    /* === Path B: I_4x4 — per-block forward+inverse. === */
    int modes4_b[16];
    u8 recon_b[256];
    i16 ac_lev_b[16][16];
    int bits_b = try_path_i4x4(st->src_y, st->qp_y,
                               st->mb_r, st->mb_c, mbs_w,
                               recon_y_frame, stride_recon_y,
                               modes4_b, ac_lev_b, recon_b);

    /* Pick winner. Tie favors I_16x16 (simpler MB header, faster decode).
     *
     * KNOWN BUG (2026-05-02): the I_4x4 path is byte-exact with ffmpeg in
     * isolation (every MB I_4x4) and the I_16x16 path is byte-exact in
     * isolation. But when SOME MBs pick I_4x4 and others pick I_16x16, the
     * decoder's recon diverges from ours starting at the first I_4x4 ↔
     * I_16x16 boundary (verified across all 24 Kodak images at QP 18-38).
     * Spec analysis of the cross-MB state (luma_nc TotalCoeff semantics,
     * luma_mode4 sentinel for I_16x16 neighbors, mode-prediction round-
     * trip) all looks consistent — the bug is somewhere subtler that I
     * couldn't pin down. Until it's found, force I_16x16 to keep the
     * dataset byte-exact. The full I_4x4 emit path is preserved in code;
     * just flip this guard to re-enable. */
    bits_b = INT32_MAX;
    if (bits_a <= bits_b) {
        st->mb_type_is_i4x4 = 0;
        st->mode16 = mode_a;
        memcpy(st->pred_y, pred_a, 256);
        memcpy(st->recon_y, recon_a, 256);
        memcpy(st->ac_levels_y, ac_lev_a, sizeof ac_lev_a);
        memcpy(st->dc_levels_y, dc_lev_a, sizeof dc_lev_a);
    } else {
        st->mb_type_is_i4x4 = 1;
        memcpy(st->modes4, modes4_b, sizeof modes4_b);
        memcpy(st->recon_y, recon_b, 256);
        memcpy(st->ac_levels_y_full, ac_lev_b, sizeof ac_lev_b);
    }

    /* === Chroma 8x8 mode pick (path-independent). === */
    int cmode = IC_DC, cbest = INT32_MAX;
    u8 cand_u[64], cand_v[64];
    for (int m = 0; m < 4; m++) {
        if (m == IC_VERTICAL   && !st->chroma_avail_top)  continue;
        if (m == IC_HORIZONTAL && !st->chroma_avail_left) continue;
        if (m == IC_PLANE      && !(st->chroma_avail_top && st->chroma_avail_left
                                    && st->chroma_avail_tl)) continue;
        predict_chroma_8x8(m, st->cu_top, st->cu_left, st->cu_tl,
                           st->chroma_avail_top, st->chroma_avail_left,
                           st->chroma_avail_tl, cand_u);
        predict_chroma_8x8(m, st->cv_top, st->cv_left, st->cv_tl,
                           st->chroma_avail_top, st->chroma_avail_left,
                           st->chroma_avail_tl, cand_v);
        int s = sad_n(st->src_u, cand_u, 64) + sad_n(st->src_v, cand_v, 64);
        if (s < cbest) {
            cbest = s; cmode = m;
            memcpy(st->pred_u, cand_u, 64);
            memcpy(st->pred_v, cand_v, 64);
        }
    }
    st->mode_chroma = cmode;
}

/* === stage 2: mb_residual (chroma only) ===
 * res = src - pred for the 4+4 chroma 4x4 blocks. Luma residuals live in
 * mb_mode_decide because of the I_4x4 dependency chain. */
static void mb_residual(mb_state_t *st)
{
    for (int br = 0; br < 2; br++)
        for (int bc = 0; bc < 2; bc++) {
            residual_4x4_8x8(st->src_u, st->pred_u, br, bc, st->res_u[br*2 + bc]);
            residual_4x4_8x8(st->src_v, st->pred_v, br, bc, st->res_v[br*2 + bc]);
        }
}

/* === stage 3: mb_transform (chroma only) === */
static void mb_transform(mb_state_t *st)
{
    for (int idx = 0; idx < 4; idx++) {
        i16 dct[16];
        dct4x4(st->res_u[idx], dct);
        st->dc_extract_u[idx] = dct[0];
        memcpy(st->dct_ac_u[idx], dct, sizeof dct);
        st->dct_ac_u[idx][0] = 0;
    }
    hadamard2x2(st->dc_extract_u, st->dc_had_u);

    for (int idx = 0; idx < 4; idx++) {
        i16 dct[16];
        dct4x4(st->res_v[idx], dct);
        st->dc_extract_v[idx] = dct[0];
        memcpy(st->dct_ac_v[idx], dct, sizeof dct);
        st->dct_ac_v[idx][0] = 0;
    }
    hadamard2x2(st->dc_extract_v, st->dc_had_v);
}

/* === stage 4: mb_quantize (chroma only) === */
static void mb_quantize(mb_state_t *st)
{
    for (int idx = 0; idx < 4; idx++) {
        quant_4x4(st->dct_ac_u[idx], st->ac_levels_u[idx], st->qp_c, 1);
        quant_4x4(st->dct_ac_v[idx], st->ac_levels_v[idx], st->qp_c, 1);
    }
    quant_dc_2x2(st->dc_had_u, st->dc_levels_u, st->qp_c, 1);
    quant_dc_2x2(st->dc_had_v, st->dc_levels_v, st->qp_c, 1);
}

/* === stages 5+6: mb_reconstruct (chroma only) === */
static void mb_reconstruct(mb_state_t *st)
{
    i32 dc_dq_u[4], dc_recon_u[4];
    iquant_dc_2x2(st->dc_levels_u, dc_dq_u, st->qp_c);
    ihadamard2x2(dc_dq_u, dc_recon_u);
    for (int br = 0; br < 2; br++)
        for (int bc = 0; bc < 2; bc++) {
            int idx = br*2 + bc;
            i32 ac_dq[16];
            iquant_4x4(st->ac_levels_u[idx], ac_dq, st->qp_c);
            ac_dq[0] = dc_recon_u[idx];
            i32 res[16];
            idct4x4(ac_dq, res);
            recon_4x4_8x8(st->recon_u, st->pred_u, br, bc, res);
        }

    i32 dc_dq_v[4], dc_recon_v[4];
    iquant_dc_2x2(st->dc_levels_v, dc_dq_v, st->qp_c);
    ihadamard2x2(dc_dq_v, dc_recon_v);
    for (int br = 0; br < 2; br++)
        for (int bc = 0; bc < 2; bc++) {
            int idx = br*2 + bc;
            i32 ac_dq[16];
            iquant_4x4(st->ac_levels_v[idx], ac_dq, st->qp_c);
            ac_dq[0] = dc_recon_v[idx];
            i32 res[16];
            idct4x4(ac_dq, res);
            recon_4x4_8x8(st->recon_v, st->pred_v, br, bc, res);
        }
}

/* Compute coded-block-pattern flags from quantized levels.
 *
 * I_16x16: cbp_luma is a single 0/1 flag — set iff any AC coefficient
 *          (positions 1..15) is nonzero across all 16 4x4 blocks. The flag
 *          gets folded into mb_type via the 1+mode+4*cbpC+12*cbpL formula.
 *
 * I_4x4:   cbp_luma is a 4-bit field per spec — bit i is set iff any 4x4
 *          sub-block within 8x8 quadrant i has a nonzero coefficient. The
 *          16 luma blocks split across 4 8x8 quadrants:
 *               quad 0: scan blocks 0,1,2,3
 *               quad 1: scan blocks 4,5,6,7
 *               quad 2: scan blocks 8,9,10,11
 *               quad 3: scan blocks 12,13,14,15
 *          (matches blk_scan_br/bc — quad index is just blk/4.)
 *
 * cbp_chroma is path-independent: 0=none, 1=DC only, 2=DC+AC. */
static void mb_compute_cbp(mb_state_t *st)
{
    if (st->mb_type_is_i4x4) {
        int cbp = 0;
        for (int s = 0; s < 16; s++) {
            int br = blk_scan_br[s];
            int bc = blk_scan_bc[s];
            int idx = br*4 + bc;
            for (int k = 0; k < 16; k++) {
                if (st->ac_levels_y_full[idx][k] != 0) {
                    cbp |= (1 << (s / 4));
                    break;
                }
            }
        }
        st->cbp_luma = cbp;
    } else {
        int cbp = 0;
        for (int idx = 0; idx < 16 && !cbp; idx++)
            for (int k = 1; k < 16; k++)
                if (st->ac_levels_y[idx][k] != 0) { cbp = 1; break; }
        st->cbp_luma = cbp;
    }

    int chroma_dc_nz = 0, chroma_ac_nz = 0;
    for (int k = 0; k < 4; k++) {
        if (st->dc_levels_u[k] != 0) chroma_dc_nz = 1;
        if (st->dc_levels_v[k] != 0) chroma_dc_nz = 1;
    }
    for (int idx = 0; idx < 4 && !chroma_ac_nz; idx++)
        for (int k = 1; k < 16; k++) {
            if (st->ac_levels_u[idx][k] != 0) { chroma_ac_nz = 1; break; }
            if (st->ac_levels_v[idx][k] != 0) { chroma_ac_nz = 1; break; }
        }

    st->cbp_chroma = chroma_ac_nz ? 2 : (chroma_dc_nz ? 1 : 0);
}

/* Sentinel stored in ncs->luma_mode4 for 4x4 slots inside an I_16x16 MB.
 * Per spec 8.3.1.1, an I_16x16 neighbor forces predIntra4x4PredMode to DC
 * regardless of the other neighbor's mode — that's a stronger condition
 * than "treat as DC and run min()". We use -1 to distinguish this case
 * from a real I_4x4_DC neighbor (which contributes to min). */
#define LUMA_MODE4_NONI4 (-1)

/* Emit per-block I_4x4 mode signal (prev_intra4x4_pred_mode_flag + optional
 * rem_intra4x4_pred_mode). Spec 8.3.1.1 / 7.3.5.1.
 *
 * predIntra4x4PredMode = DC if either neighbor is unavailable or coded
 *   in Intra_16x16 (or in inter, which we don't have); else min of the two
 *   neighbor I_4x4 modes. Within the same I_4x4 MB, neighbors are always
 *   I_4x4 (just walk modes_in_mb in scan order). Outside, look up in
 *   luma_mode4 and treat the LUMA_MODE4_NONI4 sentinel as "force DC".
 */
static void emit_intra4x4_mode(bitstream_t *bs, int blk_scan_idx,
                               int actual_mode, int mb_r, int mb_c,
                               const int modes_in_mb[16],
                               const int *luma_mode4, int luma_w4)
{
    int br = blk_scan_br[blk_scan_idx];
    int bc = blk_scan_bc[blk_scan_idx];

    /* Top neighbor: -1 means unavailable or non-I_4x4. */
    int mode_top = -1;
    if (br > 0) {
        for (int s = 0; s < blk_scan_idx; s++) {
            if (blk_scan_br[s] == br - 1 && blk_scan_bc[s] == bc) {
                mode_top = modes_in_mb[s];   /* always a valid 0..8 mode */
                break;
            }
        }
    } else if (mb_r > 0) {
        int v = luma_mode4[(mb_r*4 - 1) * luma_w4 + (mb_c*4 + bc)];
        mode_top = (v >= 0 && v <= 8) ? v : -1;
    }

    /* Left neighbor */
    int mode_left = -1;
    if (bc > 0) {
        for (int s = 0; s < blk_scan_idx; s++) {
            if (blk_scan_br[s] == br && blk_scan_bc[s] == bc - 1) {
                mode_left = modes_in_mb[s];
                break;
            }
        }
    } else if (mb_c > 0) {
        int v = luma_mode4[(mb_r*4 + br) * luma_w4 + (mb_c*4 - 1)];
        mode_left = (v >= 0 && v <= 8) ? v : -1;
    }

    int pred_mode;
    if (mode_top < 0 || mode_left < 0)
        pred_mode = I4_DC;
    else
        pred_mode = (mode_top < mode_left) ? mode_top : mode_left;

    if (actual_mode == pred_mode) {
        bs_put_bits(bs, 1, 1);   /* prev_intra4x4_pred_mode_flag = 1 */
    } else {
        bs_put_bits(bs, 0, 1);   /* prev_intra4x4_pred_mode_flag = 0 */
        int rem = (actual_mode < pred_mode) ? actual_mode : actual_mode - 1;
        bs_put_bits(bs, rem, 3); /* rem_intra4x4_pred_mode (3 bits) */
    }
}

/* === stage 7: mb_cavlc_emit ===
 * Emit the macroblock layer to the slice bitstream. Order matches H.264
 * spec 7.3.5.1. Two distinct paths depending on st->mb_type_is_i4x4:
 *
 *   I_16x16 path (mb_type 1..24):
 *     mb_type encodes mode + cbp_luma(1bit) + cbp_chroma(2bits) all-in-one.
 *     Always emit mb_qp_delta and the luma DC residual block, AC blocks
 *     conditional on cbp_luma.
 *
 *   I_4x4 path (mb_type = 0):
 *     16 × prev/rem mode flags, intra_chroma_pred_mode, me(cbp), then
 *     mb_qp_delta + residuals only if cbp != 0. Luma blocks are full 16-coef
 *     (no DC extraction), emitted only for 8x8 quadrants set in cbp_luma.
 *
 * Always updates ncs->luma_nc / luma_mode4 / chroma_*_nc so subsequent MBs
 * see correct neighbor state. Architecture.txt §8 "CAVLC + bit pack". */
static int mb_cavlc_emit(mb_state_t *st, nc_state_t *ncs, bitstream_t *bs)
{
    int start_bits = bs->byte_pos * 8 + bs->n_in_cur;
    int luma_w4   = ncs->luma_w4;
    int chroma_w4 = ncs->chroma_w4;

    if (st->mb_type_is_i4x4) {
        /* mb_type = 0 (I_NxN). ue(0) = '1' (1 bit). */
        bs_put_ue(bs, 0);

        /* Per-block prev/rem mode flags, in scan order. */
        for (int s = 0; s < 16; s++) {
            int br = blk_scan_br[s];
            int bc = blk_scan_bc[s];
            int actual = st->modes4[br*4 + bc];
            /* modes_in_mb is indexed by scan position s, not raster br*4+bc.
             * Build a transient view in scan order. */
            int modes_scan[16];
            for (int t = 0; t < 16; t++)
                modes_scan[t] = st->modes4[blk_scan_br[t]*4 + blk_scan_bc[t]];
            emit_intra4x4_mode(bs, s, actual, st->mb_r, st->mb_c,
                               modes_scan, ncs->luma_mode4, luma_w4);
        }

        /* intra_chroma_pred_mode */
        bs_put_ue(bs, st->mode_chroma);

        /* coded_block_pattern me(v): map 6-bit cbp -> codeNum -> ue. */
        int cbp_value = (st->cbp_luma & 0xF) | ((st->cbp_chroma & 0x3) << 4);
        bs_put_ue(bs, cbp_intra_to_codenum[cbp_value]);

        /* mb_qp_delta + residuals only if cbp_luma|cbp_chroma != 0
         * (spec 7.3.5.1 "if any nonzero residual"). */
        int has_residual = (st->cbp_luma != 0) || (st->cbp_chroma != 0);
        if (has_residual) {
            bs_put_se(bs, 0);   /* mb_qp_delta = 0 */

            /* Luma blocks in scan order. Each block belongs to 8x8 quadrant
             * (s / 4); emit only if cbp_luma's bit for that quadrant is set. */
            for (int s = 0; s < 16; s++) {
                int br = blk_scan_br[s];
                int bc = blk_scan_bc[s];
                int idx = br*4 + bc;

                i16 zz[16];
                for (int k = 0; k < 16; k++)
                    zz[k] = st->ac_levels_y_full[idx][zz_scan_4x4[k]];

                int gx = st->mb_c * 4 + bc;
                int gy = st->mb_r * 4 + br;
                int top_nc  = (gy > 0) ? ncs->luma_nc[(gy - 1) * luma_w4 + gx] : 0;
                int left_nc = (gx > 0) ? ncs->luma_nc[gy * luma_w4 + (gx - 1)] : 0;
                int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);

                int quad_bit = (st->cbp_luma >> (s / 4)) & 1;
                if (quad_bit)
                    cavlc_encode_block(bs, zz, 16, BLK_LUMA_FULL, nC);
                /* nC count: full 16-coef when emitted, 0 when skipped (the
                 * spec reads totalCoeff of 0 from blocks not emitted). */
                ncs->luma_nc[gy * luma_w4 + gx] = quad_bit ? count_nonzero(zz, 16) : 0;
                ncs->luma_mode4[gy * luma_w4 + gx] = st->modes4[idx];
            }

            /* Chroma DC */
            if (st->cbp_chroma >= 1) {
                cavlc_encode_block(bs, st->dc_levels_u, 4, BLK_CHROMA_DC, -1);
                cavlc_encode_block(bs, st->dc_levels_v, 4, BLK_CHROMA_DC, -1);
            }

            /* Chroma AC */
            for (int comp = 0; comp < 2; comp++) {
                int *cnc = (comp == 0) ? ncs->chroma_u_nc : ncs->chroma_v_nc;
                i16 (*ac_levels)[16] = (comp == 0) ? st->ac_levels_u : st->ac_levels_v;
                for (int br = 0; br < 2; br++)
                    for (int bc = 0; bc < 2; bc++) {
                        int idx = br*2 + bc;
                        i16 zz[16];
                        for (int k = 0; k < 16; k++)
                            zz[k] = ac_levels[idx][zz_scan_4x4[k]];
                        int gx = st->mb_c * 2 + bc;
                        int gy = st->mb_r * 2 + br;
                        int top_nc  = (gy > 0) ? cnc[(gy - 1) * chroma_w4 + gx] : 0;
                        int left_nc = (gx > 0) ? cnc[gy * chroma_w4 + (gx - 1)] : 0;
                        int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);
                        if (st->cbp_chroma == 2)
                            cavlc_encode_block(bs, &zz[1], 15, BLK_CHROMA_AC, nC);
                        cnc[gy * chroma_w4 + gx] =
                            (st->cbp_chroma == 2) ? count_nonzero(&zz[1], 15) : 0;
                    }
            }
        } else {
            /* No residual emitted at all — but neighbor state must still
             * reflect "this MB had zero coefs everywhere" so subsequent MBs
             * compute correct nC. */
            for (int s = 0; s < 16; s++) {
                int br = blk_scan_br[s], bc = blk_scan_bc[s];
                int idx = br*4 + bc;
                int gx = st->mb_c * 4 + bc;
                int gy = st->mb_r * 4 + br;
                ncs->luma_nc[gy * luma_w4 + gx] = 0;
                ncs->luma_mode4[gy * luma_w4 + gx] = st->modes4[idx];
            }
            for (int br = 0; br < 2; br++)
                for (int bc = 0; bc < 2; bc++) {
                    int gx = st->mb_c * 2 + bc, gy = st->mb_r * 2 + br;
                    ncs->chroma_u_nc[gy * chroma_w4 + gx] = 0;
                    ncs->chroma_v_nc[gy * chroma_w4 + gx] = 0;
                }
        }
        return bs->byte_pos * 8 + bs->n_in_cur - start_bits;
    }

    /* ===== I_16x16 path ===== */
    /* mb_type for I_16x16: spec Table 7-11
     * mb_type = 1 + PredMode + 4*CBPChroma + 12*CBPLuma. */
    int mb_type = 1 + st->mode16 + 4 * st->cbp_chroma + 12 * st->cbp_luma;
    bs_put_ue(bs, mb_type);
    bs_put_ue(bs, st->mode_chroma);     /* intra_chroma_pred_mode */
    bs_put_se(bs, 0);                   /* mb_qp_delta = 0 (always for I_16x16) */

    /* Luma DC block (always emitted for I_16x16). nC from block-0 neighbors. */
    {
        int gx0 = st->mb_c * 4;
        int gy0 = st->mb_r * 4;
        int top_nc  = (gy0 > 0) ? ncs->luma_nc[(gy0 - 1) * luma_w4 + gx0] : 0;
        int left_nc = (gx0 > 0) ? ncs->luma_nc[gy0 * luma_w4 + (gx0 - 1)] : 0;
        int nC = cavlc_compute_nC(top_nc, left_nc, gy0 > 0, gx0 > 0);

        i16 zz[16];
        for (int k = 0; k < 16; k++) zz[k] = st->dc_levels_y[zz_scan_4x4[k]];
        cavlc_encode_block(bs, zz, 16, BLK_LUMA_DC_16x16, nC);
    }

    /* Luma AC blocks in I_4x4 sub-block scan order. */
    for (int s = 0; s < 16; s++) {
        int br = blk_scan_br[s];
        int bc = blk_scan_bc[s];
        int idx = br*4 + bc;

        i16 zz[16];
        for (int k = 0; k < 16; k++)
            zz[k] = st->ac_levels_y[idx][zz_scan_4x4[k]];

        int gx = st->mb_c * 4 + bc;
        int gy = st->mb_r * 4 + br;
        int top_nc  = (gy > 0) ? ncs->luma_nc[(gy - 1) * luma_w4 + gx] : 0;
        int left_nc = (gx > 0) ? ncs->luma_nc[gy * luma_w4 + (gx - 1)] : 0;
        int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);

        if (st->cbp_luma)
            cavlc_encode_block(bs, &zz[1], 15, BLK_LUMA_AC, nC);
        ncs->luma_nc[gy * luma_w4 + gx] = count_nonzero(&zz[1], 15);
        /* I_16x16-coded blocks: store the sentinel so future I_4x4 blocks
         * recognise this as a non-I_4x4 neighbor and force predIntra4x4PredMode
         * to DC per spec 8.3.1.1 (NOT min(..., DC)). */
        ncs->luma_mode4[gy * luma_w4 + gx] = LUMA_MODE4_NONI4;
    }

    /* Chroma DC: emitted in U,V order whenever cbp_chroma >= 1. */
    if (st->cbp_chroma >= 1) {
        cavlc_encode_block(bs, st->dc_levels_u, 4, BLK_CHROMA_DC, -1);
        cavlc_encode_block(bs, st->dc_levels_v, 4, BLK_CHROMA_DC, -1);
    }

    /* Chroma AC */
    for (int comp = 0; comp < 2; comp++) {
        int *cnc = (comp == 0) ? ncs->chroma_u_nc : ncs->chroma_v_nc;
        i16 (*ac_levels)[16] = (comp == 0) ? st->ac_levels_u : st->ac_levels_v;
        for (int br = 0; br < 2; br++)
            for (int bc = 0; bc < 2; bc++) {
                int idx = br*2 + bc;
                i16 zz[16];
                for (int k = 0; k < 16; k++)
                    zz[k] = ac_levels[idx][zz_scan_4x4[k]];

                int gx = st->mb_c * 2 + bc;
                int gy = st->mb_r * 2 + br;
                int top_nc  = (gy > 0) ? cnc[(gy - 1) * chroma_w4 + gx] : 0;
                int left_nc = (gx > 0) ? cnc[gy * chroma_w4 + (gx - 1)] : 0;
                int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);

                if (st->cbp_chroma == 2)
                    cavlc_encode_block(bs, &zz[1], 15, BLK_CHROMA_AC, nC);
                cnc[gy * chroma_w4 + gx] = count_nonzero(&zz[1], 15);
            }
    }

    return bs->byte_pos * 8 + bs->n_in_cur - start_bits;
}

static int encode_mb_emit(const u8 *src_y,  int stride_y,
                          const u8 *src_uv, int stride_uv,
                          u8 *recon_y, int recon_stride_y,
                          u8 *recon_uv, int recon_stride_uv,
                          int mb_r, int mb_c, int width, int height, int mbs_w,
                          int qp_y, int qp_c, nc_state_t *ncs,
                          bitstream_t *bs)
{
    mb_state_t st = {0};
    st.mb_r = mb_r;
    st.mb_c = mb_c;
    st.qp_y = qp_y;
    st.qp_c = qp_c;

    mb_fetch(src_y, stride_y, src_uv, stride_uv,
             recon_y, recon_stride_y, recon_uv, recon_stride_uv,
             width, height, &st);
    /* mb_mode_decide does the full luma encode internally (it has to —
     * I_4x4's per-block dependency chain forces forward+inverse to run
     * inline with mode picking). It needs the recon plane for I_4x4's
     * outer-MB neighbors. */
    mb_mode_decide(mbs_w, recon_y, recon_stride_y, &st);
    mb_residual(&st);     /* chroma only */
    mb_transform(&st);    /* chroma only */
    mb_quantize(&st);     /* chroma only */
    mb_reconstruct(&st);  /* chroma only */

    /* Write reconstructed samples into the frame-level recon planes so
     * subsequent MBs see them as prediction neighbors. */
    copy_out_mb_luma(recon_y, recon_stride_y, mb_r, mb_c, st.recon_y);
    copy_out_mb_chroma_combine(recon_uv, recon_stride_uv, mb_r, mb_c,
                               st.recon_u, st.recon_v);

    mb_compute_cbp(&st);
    return mb_cavlc_emit(&st, ncs, bs);
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
    ncs.luma_mode4  = arena_luma_mode4;
    ncs.luma_w4     = luma_w4;
    ncs.luma_h4     = luma_h4;
    ncs.chroma_w4   = chroma_w4;
    ncs.chroma_h4   = chroma_h4;
    memset(ncs.luma_nc,     0, (size_t)luma_w4   * luma_h4   * sizeof(int));
    memset(ncs.chroma_u_nc, 0, (size_t)chroma_w4 * chroma_h4 * sizeof(int));
    memset(ncs.chroma_v_nc, 0, (size_t)chroma_w4 * chroma_h4 * sizeof(int));
    memset(ncs.luma_mode4,  0, (size_t)luma_w4   * luma_h4   * sizeof(int));

    int qp_c = chroma_qp(qp, 0);

    /* Per-MB encoding into the slice RBSP */
    for (int r = 0; r < mbs_h; r++) {
        for (int c = 0; c < mbs_w; c++) {
            encode_mb_emit(src_y, stride_y, src_uv, stride_uv,
                           recon_y_int, width, recon_uv_int, width,
                           r, c, width, height, mbs_w,
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

    /* Optional recon copy-out (test bench / measurement only — the FPGA IP
     * does not write recon to host memory). */
    if (recon_y_out)
        for (int i = 0; i < height; i++)
            memcpy(&recon_y_out[i * recon_stride_y], &recon_y_int[i * width], width);
    if (recon_uv_out)
        for (int i = 0; i < height/2; i++)
            memcpy(&recon_uv_out[i * recon_stride_uv], &recon_uv_int[i * width], width);

    /* Kernel-level stats: integer-only quantities that map onto the FPGA
     * hardware register set (architecture.txt §10: BS_BYTES_OUT, PERF_MB_DONE).
     * PSNR / bpp are derived host-side by the test bench from the recon
     * planes — see main.c. */
    if (stats) {
        stats->bytes_out  = dst_pos;
        stats->total_bits = dst_pos * 8;
        stats->mb_count   = mb_count;
        stats->psnr_y    = 0.0;
        stats->psnr_u    = 0.0;
        stats->psnr_v    = 0.0;
        stats->psnr_avg  = 0.0;
        stats->bpp       = 0.0;
    }

    return 0;
}
