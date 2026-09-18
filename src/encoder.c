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
#include "rd_tables.h"
#include "inter.h"
#include "deblock.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

/* nc / mode4 arenas use u8 storage: nc values are TotalCoeff counts in
 * 0..16 and modes are 0..8 — both fit in 8 bits, and the lookups widen
 * to int via implicit promotion. 4× BRAM savings vs int[] for the HLS
 * port (1080p = 130 KB total at u8 vs 518 KB at int). */
static u8 arena_recon_y    [MAX_W * MAX_H];
static u8 arena_recon_uv   [MAX_W * (MAX_H / 2)];
static u8 arena_luma_nc    [ARENA_LUMA_W4   * ARENA_LUMA_H4];
static u8 arena_chroma_u_nc[ARENA_CHROMA_W4 * ARENA_CHROMA_H4];
static u8 arena_chroma_v_nc[ARENA_CHROMA_W4 * ARENA_CHROMA_H4];
static u8 arena_rbsp       [ARENA_RBSP_BYTES];

/* Per-4x4-block luma intra prediction mode, indexed [gy * luma_w4 + gx]
 * (same indexing as arena_luma_nc). Used by the I_4x4 emit path to compute
 * predIntra4x4PredMode (spec 8.3.1.1) for prev/rem mode flag emission. For
 * blocks inside an I_16x16 MB the stored value is I4_DC = 2: spec says an
 * I_16x16 neighbor contributes effective mode 2 (DC) to the Min(top, left)
 * computation. Storing I4_DC directly avoids a sentinel + special-case
 * lookup path. */
static u8  arena_luma_mode4 [ARENA_LUMA_W4   * ARENA_LUMA_H4];

/* ===== P-frame state =====
 * The previous frame's reconstruction (the reference), its padded copies
 * for motion compensation, and the current picture's vector field. */
#define ARENA_MBS ((MAX_W / 16) * (MAX_H / 16))
static u8  arena_ref_y   [MAX_W * MAX_H];
static u8  arena_ref_uv  [MAX_W * (MAX_H / 2)];
static u8  arena_pad_y   [(MAX_W + 2 * REF_PAD) * (MAX_H + 2 * REF_PAD)];
static u8  arena_pad_u   [(MAX_W / 2 + 2 * REF_PAD_C) * (MAX_H / 2 + 2 * REF_PAD_C)];
static u8  arena_pad_v   [(MAX_W / 2 + 2 * REF_PAD_C) * (MAX_H / 2 + 2 * REF_PAD_C)];
static i16 arena_mvx     [ARENA_MBS];
static i16 arena_mvy     [ARENA_MBS];
static u8  arena_mb_intra[ARENA_MBS];
static int ref_valid = 0;          /* a reference frame exists */
static int ref_w = 0, ref_h = 0;
static dbk_mb_t arena_dbk[ARENA_MBS];   /* per-MB deblocking info of the current picture */

/* ===== MB-level rate control state ===== */
static struct {
    long   fill;              /* leaky bucket, bits */
    double c_i, c_p;          /* complexity: bits * 2^(qp/6) of the last I / P frame */
    int    have_i, have_p;
    int    qp_last;           /* frame QP of the previous frame */
    int    pps_qp;            /* pic_init_qp written in the PPS */
    long   total_prev;        /* bits of the previous frame (for the MB weights) */
    int    prev_mbs;
    long   overflow;          /* bits the bucket could not absorb since the reset */
} rc;
static u32 arena_mb_bits[ARENA_MBS];      /* bits per MB of the previous frame */
static u32 arena_mb_bits_cur[ARENA_MBS];  /* bits per MB of the current frame */

static int ilog2_x6(double r)             /* round(6 * log2(r)) */
{
    double v = 0; int n = 0;
    if (r <= 0) return 0;
    while (r >= 2.0) { r /= 2.0; n++; }
    while (r < 1.0)  { r *= 2.0; n--; }
    /* log2(r) for r in [1,2) by a short series on (r-1) */
    v = (r - 1.0) * (1.4427 - 0.7213 * (r - 1.0) + 0.4809 * (r - 1.0) * (r - 1.0));
    v = (v + n) * 6.0;
    return (int)(v > 0 ? v + 0.5 : v - 0.5);
}

/* Hardware model of the per-MB QP step (rc_mb == 3): the integer arithmetic
 * of rc_mb_engine.vhd, bit for bit.
 *   round(6 log2(spent / expect)) as a threshold count: RC_T12[j] =
 *   round(4096 * 2^((j - 15.5) / 6)) for j = 0..31, A = -16 + #{j : spent
 *   * 4096 >= expect * RC_T12[j]} (A = 0 at spent == expect, 6 at 2x).
 *   trunc(4 (spent - expect) / target) as |4e| / target capped at 15. */
static const u32 RC_T12[32] = {
    683, 767, 861, 967, 1085, 1218, 1367, 1534, 1722, 1933, 2170, 2435, 2734, 3069, 3444, 3866,
    4340, 4871, 5468, 6137, 6889, 7732, 8679, 9742, 10935, 12274, 13777, 15464, 17358, 19484, 21870, 24548 };

static int rc_hw_log2x6(u32 spent, u32 expect)
{
    int a = -16;
    for (int j = 0; j < 32; j++)
        if (((uint64_t)spent << 12) >= (uint64_t)expect * RC_T12[j]) a = j - 15; else break;
    return a;
}

static int rc_hw_div4(int64_t e, u32 target)
{
    uint64_t m = (uint64_t)(e < 0 ? -e : e) * 4;
    uint64_t b = target ? m / target : 15;
    if (b > 15) b = 15;
    return e < 0 ? -(int)b : (int)b;
}

static int dump_idx = 0;                  /* frame index for the *_SEQ dumps */

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

/* ===== nC + per-block mode state =====
 * Storage type is u8: counts 0..16 and modes 0..8 fit in 8 bits. Reads
 * widen to int via implicit promotion. */
typedef struct {
    u8 *luma_nc;
    u8 *chroma_u_nc;
    u8 *chroma_v_nc;
    /* luma_mode4: per-4x4-block intra prediction mode at frame scale. Used
     * by I_4x4 emission to compute predIntra4x4PredMode from neighbors. */
    u8 *luma_mode4;
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
                         u8 recon_mb_out[256], int no_topright_modes)
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

        /* ---- I_4x4 mode decision: SATD screen -> shortlist -> RD ----
         * See rd_tables.h for the policy and constants. Everything is
         * integer so the VHDL mode decider can match it exactly.
         * The screen is "open-loop": it predicts from reconstructed samples
         * where the neighbour is another MB and from SOURCE samples where it
         * is a block of this MB, so the ranking does not depend on this MB's
         * reconstruction; the shortlist is then evaluated closed-loop. */
        u8 stop[8] = {0}, sleft[4] = {0}, stl = 128;
        int sat, sal, satl;
        gather_neighbors_4x4(blk, mb_r, mb_c, mbs_w, recon_y_frame,
                             stride_recon_y, src_mb,
                             stop, sleft, &stl, &sat, &sal, &satl);
        (void)sat; (void)sal; (void)satl;
        int cand_mode[9], cand_cost[9], ncand = 0;
        u8  cand_pred[9][16];

        /* predIntra4x4PredMode (spec 8.3.1.1) for the mode-bit penalty. */
        int pred_mode;
        {
            int top_ok = 0, left_ok = 0, mode_top = I4_DC, mode_left = I4_DC;
            if (br > 0)        { mode_top = modes4_out[(br-1)*4 + bc]; top_ok = 1; }
            else if (mb_r > 0) { mode_top = arena_luma_mode4[(mb_r*4 - 1) * (mbs_w * 4) + (mb_c*4 + bc)]; top_ok = 1; }
            if (bc > 0)        { mode_left = modes4_out[br*4 + bc - 1]; left_ok = 1; }
            else if (mb_c > 0) { mode_left = arena_luma_mode4[(mb_r*4 + br) * (mbs_w * 4) + (mb_c*4 - 1)]; left_ok = 1; }
            pred_mode = (!top_ok || !left_ok) ? I4_DC : (mode_top < mode_left ? mode_top : mode_left);
        }

        for (int m = 0; m < 9; m++) {
            if ((m == I4_VERTICAL || m == I4_DIAG_DOWN_LEFT ||
                 m == I4_VERTICAL_LEFT) && !at)  continue;
            if ((m == I4_HORIZONTAL || m == I4_HORIZONTAL_UP) && !al) continue;
            if ((m == I4_DIAG_DOWN_RIGHT || m == I4_VERTICAL_RIGHT ||
                 m == I4_HORIZONTAL_DOWN) && !(at && al && atl)) continue;
            /* strict refresh: block 5's top-right samples come from the MB
             * above-right, which is not yet refreshed */
            if (no_topright_modes && blk == 5 &&
                (m == I4_DIAG_DOWN_LEFT || m == I4_VERTICAL_LEFT)) continue;
            u8 *pred = cand_pred[ncand];
            predict_4x4(m, top, left, tl, at, al, atl, pred);       /* closed-loop, for the RD pass */
            u8 spred[16];
            predict_4x4(m, stop, sleft, stl, at, al, atl, spred);   /* open-loop, for the screen */
            i32 ri32[16], satd_out[16];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++) {
                    int idx = (br*4 + i) * 16 + (bc*4 + j);
                    ri32[i*4 + j] = (int)src_mb[idx] - (int)spred[i*4 + j];
                }
            ihadamard4x4(ri32, satd_out);
            int cost = 0;
            for (int k = 0; k < 16; k++) cost += abs_i(satd_out[k]);
            cost += (RD_SLAM16[qp] * ((m == pred_mode) ? 1 : 4) + 8) >> 4;
            cand_mode[ncand] = m; cand_cost[ncand] = cost; ncand++;
        }
        /* Stable insertion sort by screen cost (ties keep mode order). */
        int order[9];
        for (int k = 0; k < ncand; k++) {
            int p = k;
            while (p > 0 && cand_cost[order[p-1]] > cand_cost[k]) { order[p] = order[p-1]; p--; }
            order[p] = k;
        }
        int nfull = (RD_I4_SHORTLIST < ncand) ? RD_I4_SHORTLIST : ncand;

        int best_mode = I4_DC, best_bits = 0;
        long best_j = 0x7fffffffL;
        i16 best_levels[16];
        u8  best_recon[16];
        for (int k = 0; k < nfull; k++) {
            int ci = order[k];
            int m = cand_mode[ci];
            const u8 *pred = cand_pred[ci];
            i16 res[16], dct[16], levels[16], zz[16];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++) {
                    int idx = (br*4 + i) * 16 + (bc*4 + j);
                    res[i*4 + j] = (i16)((int)src_mb[idx] - (int)pred[i*4 + j]);
                }
            dct4x4(res, dct);
            quant_4x4(dct, levels, qp, 1);
            zigzag_4x4(levels, zz);
            int rbits = cavlc_estimate_block_bits(zz, 16, BLK_LUMA_FULL, 0);
            int mbits = (m == pred_mode) ? 1 : 4;
            i32 dq[16], rr[16];
            iquant_4x4(levels, dq, qp);
            idct4x4(dq, rr);
            u8 rec[16];
            long ssd = 0;
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++) {
                    int idx = (br*4 + i) * 16 + (bc*4 + j);
                    int v = clip_u8(pred[i*4 + j] + ((rr[i*4 + j] + 32) >> 6));
                    rec[i*4 + j] = (u8)v;
                    int d = v - (int)src_mb[idx];
                    ssd += (long)d * d;
                }
            long j = 16L * ssd + (long)RD_LAM16[qp] * (rbits + mbits);
            if (j < best_j) {
                best_j = j; best_mode = m; best_bits = rbits;
                memcpy(best_levels, levels, sizeof levels);
                memcpy(best_recon, rec, sizeof rec);
            }
        }
        modes4_out[br*4 + bc] = best_mode;
        memcpy(ac_levels_out[br*4 + bc], best_levels, sizeof best_levels);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                recon_mb_out[(br*4 + i) * 16 + (bc*4 + j)] = best_recon[i*4 + j];
        bits += best_bits;
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
                               modes4_b, ac_lev_b, recon_b, st->no_topright_modes);

    st->dbg_bits_a = bits_a;
    st->dbg_bits_b = bits_b;
    /* Pick winner. Tie favors I_16x16 (simpler MB header, faster decode). */
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
    int intra = !st->is_inter;
    for (int idx = 0; idx < 4; idx++) {
        quant_4x4(st->dct_ac_u[idx], st->ac_levels_u[idx], st->qp_c, intra);
        quant_4x4(st->dct_ac_v[idx], st->ac_levels_v[idx], st->qp_c, intra);
    }
    quant_dc_2x2(st->dc_had_u, st->dc_levels_u, st->qp_c, intra);
    quant_dc_2x2(st->dc_had_v, st->dc_levels_v, st->qp_c, intra);
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
    if (st->mb_type_is_i4x4 || st->is_inter) {
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

/* Emit per-block I_4x4 mode signal (prev_intra4x4_pred_mode_flag + optional
 * rem_intra4x4_pred_mode). Spec 8.3.1.1 / 7.3.5.1.
 *
 * Per spec 8.3.1.1:
 *   - If EITHER neighbor is UNAVAILABLE (off-frame edge), pred = DC.
 *   - Otherwise, pred = Min(intraMxMPredMode_top, intraMxMPredMode_left),
 *     where intraMxMPredMode for an I_4x4 neighbor is its block mode, and
 *     for an I_16x16 (or other non-I_NxN) neighbor is 2 (DC).
 *
 * Storage convention: luma_mode4 holds the effective intraMxMPredMode for
 * each 4x4 slot — actual block mode (0..8) for I_4x4, I4_DC=2 for I_16x16.
 * No sentinel; lookups read the value directly. Availability is tracked
 * positionally (br/bc + mb_r/mb_c bounds).
 */
static void emit_intra4x4_mode(bitstream_t *bs, int blk_scan_idx,
                               int actual_mode, int mb_r, int mb_c,
                               const int modes_in_mb[16],
                               const u8 *luma_mode4, int luma_w4)
{
    int br = blk_scan_br[blk_scan_idx];
    int bc = blk_scan_bc[blk_scan_idx];

    int top_avail = 0,  mode_top  = I4_DC;
    int left_avail = 0, mode_left = I4_DC;

    if (br > 0) {
        for (int s = 0; s < blk_scan_idx; s++)
            if (blk_scan_br[s] == br - 1 && blk_scan_bc[s] == bc) {
                mode_top = modes_in_mb[s]; top_avail = 1; break;
            }
    } else if (mb_r > 0) {
        mode_top = luma_mode4[(mb_r*4 - 1) * luma_w4 + (mb_c*4 + bc)];
        top_avail = 1;
    }

    if (bc > 0) {
        for (int s = 0; s < blk_scan_idx; s++)
            if (blk_scan_br[s] == br && blk_scan_bc[s] == bc - 1) {
                mode_left = modes_in_mb[s]; left_avail = 1; break;
            }
    } else if (mb_c > 0) {
        mode_left = luma_mode4[(mb_r*4 + br) * luma_w4 + (mb_c*4 - 1)];
        left_avail = 1;
    }

    int pred_mode;
    if (!top_avail || !left_avail)
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

/* DCC_DUMP_SRC=<path>: per MB, 24 lines of 16 samples: the source blocks in
 * the order the mb_pipeline_controller takes them (Y 0..15 raster, U 0..3,
 * V 0..3; sample (r,c) of a block at position 4r+c). */
static void dump_mb_src(const mb_state_t *st)
{
    static FILE *f = NULL; static int checked = 0;
    if (!checked) { checked = 1; const char *p = getenv("DCC_DUMP_SRC"); if (p) f = fopen(p, "w"); }
    if (!f) return;
    for (int b = 0; b < 16; b++) {
        for (int k = 0; k < 16; k++)
            fprintf(f, "%d ", st->src_y[((b / 4) * 4 + k / 4) * 16 + (b % 4) * 4 + (k % 4)]);
        fprintf(f, "\n");
    }
    for (int pl = 0; pl < 2; pl++) {
        const u8 *src = pl ? st->src_v : st->src_u;
        for (int b = 0; b < 4; b++) {
            for (int k = 0; k < 16; k++)
                fprintf(f, "%d ", src[((b / 2) * 4 + k / 4) * 8 + (b % 2) * 4 + (k % 4)]);
            fprintf(f, "\n");
        }
    }
    fflush(f);
}

/* ===== Per-MB vector dump for the VHDL mode_decide_engine testbench =====
 * Enabled by DCC_DUMP_MB=<path> (append), at most DCC_DUMP_N MBs (default
 * 200). One record per MB, all values decimal:
 *   MB r c qp_y qp_c
 *   SY 256 | SU 64 | SV 64                        source samples (row-major)
 *   NY at al atl atr top16 left16 tl tr4          luma neighbours
 *   NC at al atl cu_top8 cu_left8 cu_tl cv_top8 cv_left8 cv_tl
 *   NM m4top4 m4left4                             (2 = DC where unavailable)
 *   O is4 mode16 mode_chroma cbp_luma cbp_chroma modes4x16(raster)
 *   LY 16x16 (raster blocks, raster coefs; I_4x4 full, I_16x16 AC with [0]=0)
 *   LD 16 (I_16x16 DC levels, raster; zeros for I_4x4)
 *   LU 4x16 | LV 4x16 | DU 4 | DV 4
 *   RY 256 | RU 64 | RV 64                        reconstruction */
#include <stdio.h>
#include <stdlib.h>
static void dump_arr_u8(FILE *f, const char *tag, const u8 *a, int n)
{
    fprintf(f, "%s", tag);
    for (int i = 0; i < n; i++) fprintf(f, " %d", a[i]);
    fprintf(f, "\n");
}
static void dump_mb_vector(const mb_state_t *st, const u8 *recon_y, int stride,
                           int mbs_w, const nc_state_t *ncs)
{
    static FILE *f = NULL;
    static int  checked = 0, limit = 200, count = 0;
    if (!checked) {
        checked = 1;
        const char *p = getenv("DCC_DUMP_MB");
        if (p) f = fopen(p, "a");
        const char *n = getenv("DCC_DUMP_N");
        if (n) limit = atoi(n);
    }
    if (!f || count >= limit) return;
    count++;
    fprintf(f, "MB %d %d %d %d\n", st->mb_r, st->mb_c, st->qp_y, st->qp_c);
    dump_arr_u8(f, "SY", st->src_y, 256);
    dump_arr_u8(f, "SU", st->src_u, 64);
    dump_arr_u8(f, "SV", st->src_v, 64);
    int atr = (st->mb_r > 0 && st->mb_c < mbs_w - 1);
    fprintf(f, "NY %d %d %d %d", st->luma_avail_top, st->luma_avail_left, st->luma_avail_tl, atr);
    for (int i = 0; i < 16; i++) fprintf(f, " %d", st->luma_avail_top ? st->luma_top[i] : 0);
    for (int i = 0; i < 16; i++) fprintf(f, " %d", st->luma_avail_left ? st->luma_left[i] : 0);
    fprintf(f, " %d", st->luma_tl);
    for (int j = 0; j < 4; j++)
        fprintf(f, " %d", atr ? recon_y[(st->mb_r*16 - 1) * stride + st->mb_c*16 + 16 + j] : 0);
    fprintf(f, "\n");
    fprintf(f, "NC %d %d %d", st->chroma_avail_top, st->chroma_avail_left, st->chroma_avail_tl);
    for (int i = 0; i < 8; i++) fprintf(f, " %d", st->chroma_avail_top ? st->cu_top[i] : 0);
    for (int i = 0; i < 8; i++) fprintf(f, " %d", st->chroma_avail_left ? st->cu_left[i] : 0);
    fprintf(f, " %d", st->cu_tl);
    for (int i = 0; i < 8; i++) fprintf(f, " %d", st->chroma_avail_top ? st->cv_top[i] : 0);
    for (int i = 0; i < 8; i++) fprintf(f, " %d", st->chroma_avail_left ? st->cv_left[i] : 0);
    fprintf(f, " %d\n", st->cv_tl);
    fprintf(f, "NM");
    for (int bc = 0; bc < 4; bc++)
        fprintf(f, " %d", st->mb_r > 0 ? ncs->luma_mode4[(st->mb_r*4 - 1) * ncs->luma_w4 + st->mb_c*4 + bc] : 2);
    for (int br = 0; br < 4; br++)
        fprintf(f, " %d", st->mb_c > 0 ? ncs->luma_mode4[(st->mb_r*4 + br) * ncs->luma_w4 + st->mb_c*4 - 1] : 2);
    fprintf(f, "\n");
    fprintf(f, "B %d %d\n", st->dbg_bits_a, st->dbg_bits_b);
    fprintf(f, "O %d %d %d %d %d", st->mb_type_is_i4x4, st->mode16, st->mode_chroma, st->cbp_luma, st->cbp_chroma);
    for (int i = 0; i < 16; i++) fprintf(f, " %d", st->mb_type_is_i4x4 ? st->modes4[i] : 0);
    fprintf(f, "\n");
    fprintf(f, "LY");
    for (int b = 0; b < 16; b++)
        for (int k = 0; k < 16; k++)
            fprintf(f, " %d", st->mb_type_is_i4x4 ? st->ac_levels_y_full[b][k] : st->ac_levels_y[b][k]);
    fprintf(f, "\nLD");
    for (int k = 0; k < 16; k++) fprintf(f, " %d", st->mb_type_is_i4x4 ? 0 : st->dc_levels_y[k]);
    fprintf(f, "\nLU");
    for (int b = 0; b < 4; b++) for (int k = 0; k < 16; k++) fprintf(f, " %d", st->ac_levels_u[b][k]);
    fprintf(f, "\nLV");
    for (int b = 0; b < 4; b++) for (int k = 0; k < 16; k++) fprintf(f, " %d", st->ac_levels_v[b][k]);
    fprintf(f, "\nDU");
    for (int k = 0; k < 4; k++) fprintf(f, " %d", st->dc_levels_u[k]);
    fprintf(f, "\nDV");
    for (int k = 0; k < 4; k++) fprintf(f, " %d", st->dc_levels_v[k]);
    fprintf(f, "\n");
    dump_arr_u8(f, "RY", st->recon_y, 256);
    dump_arr_u8(f, "RU", st->recon_u, 64);
    dump_arr_u8(f, "RV", st->recon_v, 64);
    fflush(f);
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
/* DCC_DUMP_ITEMS=<path>: every residual block handed to the CAVLC encoder,
 * in emission order: "B <bt> <n> <nC> <levels...>" (DCC_DUMP_ITEMS_N blocks). */
static int dbg_encode_block(bitstream_t *bs, const i16 *c, int n, block_type_t bt, int nC)
{
    static FILE *f = NULL; static int checked = 0, limit = 200, cnt = 0;
    if (!checked) { checked = 1; const char *p = getenv("DCC_DUMP_ITEMS"); if (p) f = fopen(p, "w");
                    const char *l = getenv("DCC_DUMP_ITEMS_N"); if (l) limit = atoi(l); }
    if (f && cnt < limit) {
        cnt++;
        fprintf(f, "B %d %d %d", (int)bt, n, nC);
        for (int i = 0; i < n; i++) fprintf(f, " %d", c[i]);
        fprintf(f, "\n"); fflush(f);
    }
    return cavlc_encode_block(bs, c, n, bt, nC);
}

static int mb_cavlc_emit(mb_state_t *st, nc_state_t *ncs, bitstream_t *bs, int mb_type_off, int qp_delta)
{
    int start_bits = bs->byte_pos * 8 + bs->n_in_cur;
    int luma_w4   = ncs->luma_w4;
    int chroma_w4 = ncs->chroma_w4;

    if (st->mb_type_is_i4x4) {
        /* mb_type = 0 (I_NxN); in a P slice the intra types start at 5. */
        bs_put_ue(bs, 0 + mb_type_off);

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
            bs_put_se(bs, qp_delta);   /* mb_qp_delta */

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
                    dbg_encode_block(bs, zz, 16, BLK_LUMA_FULL, nC);
                /* nC count: full 16-coef when emitted, 0 when skipped (the
                 * spec reads totalCoeff of 0 from blocks not emitted). */
                ncs->luma_nc[gy * luma_w4 + gx] = quad_bit ? count_nonzero(zz, 16) : 0;
                ncs->luma_mode4[gy * luma_w4 + gx] = st->modes4[idx];
            }

            /* Chroma DC */
            if (st->cbp_chroma >= 1) {
                dbg_encode_block(bs, st->dc_levels_u, 4, BLK_CHROMA_DC, -1);
                dbg_encode_block(bs, st->dc_levels_v, 4, BLK_CHROMA_DC, -1);
            }

            /* Chroma AC */
            for (int comp = 0; comp < 2; comp++) {
                u8 *cnc = (comp == 0) ? ncs->chroma_u_nc : ncs->chroma_v_nc;
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
                            dbg_encode_block(bs, &zz[1], 15, BLK_CHROMA_AC, nC);
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
    bs_put_ue(bs, mb_type + mb_type_off);
    bs_put_ue(bs, st->mode_chroma);     /* intra_chroma_pred_mode */
    bs_put_se(bs, qp_delta);            /* mb_qp_delta (always present for I_16x16) */

    /* Luma DC block (always emitted for I_16x16). nC from block-0 neighbors. */
    {
        int gx0 = st->mb_c * 4;
        int gy0 = st->mb_r * 4;
        int top_nc  = (gy0 > 0) ? ncs->luma_nc[(gy0 - 1) * luma_w4 + gx0] : 0;
        int left_nc = (gx0 > 0) ? ncs->luma_nc[gy0 * luma_w4 + (gx0 - 1)] : 0;
        int nC = cavlc_compute_nC(top_nc, left_nc, gy0 > 0, gx0 > 0);

        i16 zz[16];
        for (int k = 0; k < 16; k++) zz[k] = st->dc_levels_y[zz_scan_4x4[k]];
        dbg_encode_block(bs, zz, 16, BLK_LUMA_DC_16x16, nC);
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
            dbg_encode_block(bs, &zz[1], 15, BLK_LUMA_AC, nC);
        ncs->luma_nc[gy * luma_w4 + gx] = count_nonzero(&zz[1], 15);
        /* I_16x16 blocks store I4_DC: spec 8.3.1.1 says an I_16x16 neighbor
         * contributes effective intraMxMPredMode = 2 (DC) to the Min(). */
        ncs->luma_mode4[gy * luma_w4 + gx] = I4_DC;
    }

    /* Chroma DC: emitted in U,V order whenever cbp_chroma >= 1. */
    if (st->cbp_chroma >= 1) {
        dbg_encode_block(bs, st->dc_levels_u, 4, BLK_CHROMA_DC, -1);
        dbg_encode_block(bs, st->dc_levels_v, 4, BLK_CHROMA_DC, -1);
    }

    /* Chroma AC */
    for (int comp = 0; comp < 2; comp++) {
        u8 *cnc = (comp == 0) ? ncs->chroma_u_nc : ncs->chroma_v_nc;
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
                    dbg_encode_block(bs, &zz[1], 15, BLK_CHROMA_AC, nC);
                cnc[gy * chroma_w4 + gx] = count_nonzero(&zz[1], 15);
            }
    }

    return bs->byte_pos * 8 + bs->n_in_cur - start_bits;
}

#ifdef MB_SELFDECODE
/* ============================================================================
 * Per-MB self-decoder
 * ============================================================================
 * Walks the bitstream after each mb_cavlc_emit call. Re-parses the MB's
 * mb_type, mode flags, CBP, mb_qp_delta, and residual blocks from scratch,
 * comparing each parsed value to the encoder's intent. Maintains parallel
 * decoder state (luma_nc, luma_mode4, etc.) advanced after each MB to mirror
 * what a spec-compliant decoder would track.
 *
 * On any mismatch — parsed value != intent, OR residual decode fails, OR
 * neighbor counts diverge — emits a diagnostic to stderr.
 *
 * Compile with -DMB_SELFDECODE to enable. Adds ~6.5 MB of .bss and one
 * full re-parse per MB.
 */
#include <stdio.h>
#include <stdlib.h>

/* Inverse Table 9-4(a): codeNum -> cbp_value for I_NxN. */
static u8 codenum_to_cbp_intra[48];
static int codenum_inv_done = 0;
static void init_codenum_inv(void) {
    if (codenum_inv_done) return;
    for (int v = 0; v < 48; v++)
        codenum_to_cbp_intra[cbp_intra_to_codenum[v]] = (u8)v;
    codenum_inv_done = 1;
}

static struct {
    int initialized;
    u8 luma_nc[ARENA_LUMA_W4 * ARENA_LUMA_H4];
    u8 luma_mode4[ARENA_LUMA_W4 * ARENA_LUMA_H4];
    u8 chroma_u_nc[ARENA_CHROMA_W4 * ARENA_CHROMA_H4];
    u8 chroma_v_nc[ARENA_CHROMA_W4 * ARENA_CHROMA_H4];
    int luma_w4, chroma_w4;
    int n_failures;
} dec_state;

static void dec_state_init(int luma_w4, int chroma_w4)
{
    init_codenum_inv();
    memset(dec_state.luma_nc,     0, sizeof dec_state.luma_nc);
    memset(dec_state.luma_mode4,  0, sizeof dec_state.luma_mode4);
    memset(dec_state.chroma_u_nc, 0, sizeof dec_state.chroma_u_nc);
    memset(dec_state.chroma_v_nc, 0, sizeof dec_state.chroma_v_nc);
    dec_state.luma_w4 = luma_w4;
    dec_state.chroma_w4 = chroma_w4;
    dec_state.initialized = 1;
    dec_state.n_failures = 0;
}

/* Called after each MB verify. Compares dec_state.luma_nc and luma_mode4
 * to encoder's ncs. Reports first divergence. */
static int verify_state_match(const mb_state_t *st, const nc_state_t *ncs)
{
    int luma_w4 = dec_state.luma_w4, chroma_w4 = dec_state.chroma_w4;
    int mb_r = st->mb_r, mb_c = st->mb_c;
    /* Check the 16 4x4 luma slots of THIS MB */
    for (int br = 0; br < 4; br++)
        for (int bc = 0; bc < 4; bc++) {
            int gx = mb_c*4 + bc, gy = mb_r*4 + br;
            int e = ncs->luma_nc[gy * luma_w4 + gx];
            int d = dec_state.luma_nc[gy * luma_w4 + gx];
            if (e != d) {
                fprintf(stderr, "STATE MISMATCH MB(%d,%d) luma_nc[%d,%d] enc=%d dec=%d\n",
                        mb_r, mb_c, br, bc, e, d);
                return -1;
            }
            int em = ncs->luma_mode4[gy * luma_w4 + gx];
            int dm = dec_state.luma_mode4[gy * luma_w4 + gx];
            if (em != dm) {
                fprintf(stderr, "STATE MISMATCH MB(%d,%d) luma_mode4[%d,%d] enc=%d dec=%d\n",
                        mb_r, mb_c, br, bc, em, dm);
                return -1;
            }
        }
    /* Check chroma 4x4 slots */
    for (int br = 0; br < 2; br++)
        for (int bc = 0; bc < 2; bc++) {
            int gx = mb_c*2 + bc, gy = mb_r*2 + br;
            int eu = ncs->chroma_u_nc[gy * chroma_w4 + gx];
            int du = dec_state.chroma_u_nc[gy * chroma_w4 + gx];
            int ev = ncs->chroma_v_nc[gy * chroma_w4 + gx];
            int dv = dec_state.chroma_v_nc[gy * chroma_w4 + gx];
            if (eu != du || ev != dv) {
                fprintf(stderr, "STATE MISMATCH MB(%d,%d) chroma_nc[%d,%d] enc=(%d,%d) dec=(%d,%d)\n",
                        mb_r, mb_c, br, bc, eu, ev, du, dv);
                return -1;
            }
        }
    return 0;
}

#define VERIFY_FAIL(fmt, ...) do { \
    fprintf(stderr, "verify FAIL MB(%d,%d): " fmt "\n", st->mb_r, st->mb_c, ##__VA_ARGS__); \
    dec_state.n_failures++; \
    if (dec_state.n_failures >= 3) exit(1); \
    return -1; \
} while(0)

/* Verify the just-emitted MB. Returns 0 on match, -1 on mismatch.
 *
 *   bs              — the bitstream we emitted to
 *   mb_start_bit    — bit offset in bs where this MB started (before mb_cavlc_emit)
 *   st              — encoder's intended MB state (modes, levels, recon, cbp)
 *   ncs             — encoder's intended neighbor state (already updated post-emit)
 */
static int verify_mb_at(bitstream_t *bs, int mb_start_bit,
                        const mb_state_t *st, const nc_state_t *ncs)
{
    int mb_r = st->mb_r, mb_c = st->mb_c;
    int luma_w4 = dec_state.luma_w4, chroma_w4 = dec_state.chroma_w4;

    /* Build a scratch buffer covering committed bytes + leftover bits in bs->cur. */
    static u8 scratch[1 << 17];   /* 128 KB; one MB at QP=0 lossless ~2 KB */
    int mb_start_byte = mb_start_bit / 8;
    int bit_off = mb_start_bit % 8;
    int n_committed = bs->byte_pos - mb_start_byte;
    if (n_committed < 0 || n_committed >= (int)sizeof scratch - 8)
        VERIFY_FAIL("scratch overflow n_committed=%d", n_committed);
    memcpy(scratch, bs->buf + mb_start_byte, (size_t)n_committed);
    /* Append leftover bits from bs->cur (MSB-first) as best-effort bytes. */
    if (bs->n_in_cur > 0) {
        u32 c = bs->cur;
        int rem = bs->n_in_cur;
        int idx = n_committed;
        while (rem >= 8) {
            scratch[idx++] = (u8)((c >> 24) & 0xFF);
            c <<= 8; rem -= 8;
        }
        if (rem > 0) {
            scratch[idx++] = (u8)((c >> 24) & 0xFF);
        }
        n_committed = idx;
    }

    bitreader_t br;
    br_init(&br, scratch, n_committed);
    if (bit_off > 0) br_get_bits(&br, bit_off);

    /* === mb_type === */
    int mb_type = (int)br_get_ue(&br);
    int dec_is_i4x4 = (mb_type == 0);
    int dec_mode16 = -1, dec_cbpL_flag = -1, dec_cbpC = -1;
    if (!dec_is_i4x4) {
        if (mb_type < 1 || mb_type > 24)
            VERIFY_FAIL("invalid mb_type=%d", mb_type);
        int t = mb_type - 1;
        dec_mode16    = t % 4;
        dec_cbpC      = (t / 4) % 3;
        dec_cbpL_flag = t / 12;
    }

    if (dec_is_i4x4 != st->mb_type_is_i4x4)
        VERIFY_FAIL("mb_type_is_i4x4 enc=%d dec=%d (mb_type=%d)",
                    st->mb_type_is_i4x4, dec_is_i4x4, mb_type);
    if (!dec_is_i4x4) {
        if (dec_mode16 != st->mode16)
            VERIFY_FAIL("mode16 enc=%d dec=%d", st->mode16, dec_mode16);
        if (dec_cbpC != st->cbp_chroma)
            VERIFY_FAIL("I16 cbp_chroma enc=%d dec=%d", st->cbp_chroma, dec_cbpC);
        if (dec_cbpL_flag != st->cbp_luma)
            VERIFY_FAIL("I16 cbp_luma_flag enc=%d dec=%d", st->cbp_luma, dec_cbpL_flag);
    }

    /* === I_NxN: 16 prev/rem mode flags === */
    int dec_modes4[16] = {0};
    if (dec_is_i4x4) {
        for (int s = 0; s < 16; s++) {
            int br_ = blk_scan_br[s], bc_ = blk_scan_bc[s];

            /* Compute pred_mode (matches emit_intra4x4_mode — spec 8.3.1.1). */
            int top_avail = 0, mode_top = I4_DC;
            int left_avail = 0, mode_left = I4_DC;
            if (br_ > 0) {
                for (int t = 0; t < s; t++)
                    if (blk_scan_br[t] == br_ - 1 && blk_scan_bc[t] == bc_) {
                        mode_top = dec_modes4[t]; top_avail = 1; break;
                    }
            } else if (mb_r > 0) {
                mode_top = dec_state.luma_mode4[(mb_r*4 - 1) * luma_w4 + (mb_c*4 + bc_)];
                top_avail = 1;
            }
            if (bc_ > 0) {
                for (int t = 0; t < s; t++)
                    if (blk_scan_br[t] == br_ && blk_scan_bc[t] == bc_ - 1) {
                        mode_left = dec_modes4[t]; left_avail = 1; break;
                    }
            } else if (mb_c > 0) {
                mode_left = dec_state.luma_mode4[(mb_r*4 + br_) * luma_w4 + (mb_c*4 - 1)];
                left_avail = 1;
            }
            int pred_mode;
            if (!top_avail || !left_avail) pred_mode = I4_DC;
            else pred_mode = (mode_top < mode_left) ? mode_top : mode_left;

            int prev_flag = (int)br_get_bits(&br, 1);
            int actual;
            if (prev_flag) actual = pred_mode;
            else {
                int rem = (int)br_get_bits(&br, 3);
                actual = (rem < pred_mode) ? rem : rem + 1;
            }
            dec_modes4[s] = actual;

            int idx = br_*4 + bc_;
            if (actual != st->modes4[idx])
                VERIFY_FAIL("blk_scan=%d (br=%d bc=%d) mode enc=%d dec=%d (pred=%d top=%d left=%d prev=%d)",
                            s, br_, bc_, st->modes4[idx], actual,
                            pred_mode, mode_top, mode_left, prev_flag);
        }
    }

    /* === intra_chroma_pred_mode === */
    int dec_mode_chroma = (int)br_get_ue(&br);
    if (dec_mode_chroma != st->mode_chroma)
        VERIFY_FAIL("mode_chroma enc=%d dec=%d", st->mode_chroma, dec_mode_chroma);

    /* === me(cbp) for I_NxN === */
    int dec_cbp_luma = dec_is_i4x4 ? 0 : dec_cbpL_flag;
    int dec_cbp_chroma = dec_is_i4x4 ? 0 : dec_cbpC;
    if (dec_is_i4x4) {
        int cbp_codenum = (int)br_get_ue(&br);
        if (cbp_codenum < 0 || cbp_codenum >= 48)
            VERIFY_FAIL("invalid cbp codeNum=%d", cbp_codenum);
        int cbp_value = codenum_to_cbp_intra[cbp_codenum];
        dec_cbp_luma   = cbp_value & 0xF;
        dec_cbp_chroma = (cbp_value >> 4) & 3;
        if (dec_cbp_luma != st->cbp_luma)
            VERIFY_FAIL("I4 cbp_luma enc=%d dec=%d (codenum=%d cbp_value=%d)",
                        st->cbp_luma, dec_cbp_luma, cbp_codenum, cbp_value);
        if (dec_cbp_chroma != st->cbp_chroma)
            VERIFY_FAIL("I4 cbp_chroma enc=%d dec=%d", st->cbp_chroma, dec_cbp_chroma);
    }

    /* === mb_qp_delta + residuals === */
    int has_residual = !dec_is_i4x4 || (dec_cbp_luma != 0) || (dec_cbp_chroma != 0);
    if (has_residual) {
        int qp_delta = (int)br_get_se(&br);
        if (qp_delta != 0)
            VERIFY_FAIL("qp_delta enc=0 dec=%d", qp_delta);
    }

    if (!has_residual) {
        /* No residual; update neighbor state to zero and modes4 sentinel. */
        for (int s = 0; s < 16; s++) {
            int br_ = blk_scan_br[s], bc_ = blk_scan_bc[s];
            int gx = mb_c*4 + bc_, gy = mb_r*4 + br_;
            dec_state.luma_nc[gy * luma_w4 + gx] = 0;
            dec_state.luma_mode4[gy * luma_w4 + gx] = dec_is_i4x4 ? dec_modes4[s] : I4_DC;
        }
        for (int br_ = 0; br_ < 2; br_++)
            for (int bc_ = 0; bc_ < 2; bc_++) {
                int gx = mb_c*2 + bc_, gy = mb_r*2 + br_;
                dec_state.chroma_u_nc[gy * chroma_w4 + gx] = 0;
                dec_state.chroma_v_nc[gy * chroma_w4 + gx] = 0;
            }
        return 0;
    }

    /* === Luma residuals === */
    if (!dec_is_i4x4) {
        /* Luma DC block (always for I_16x16). nC from block-0 neighbors. */
        int gx0 = mb_c*4, gy0 = mb_r*4;
        int top_nc  = (gy0 > 0) ? dec_state.luma_nc[(gy0 - 1) * luma_w4 + gx0] : 0;
        int left_nc = (gx0 > 0) ? dec_state.luma_nc[gy0 * luma_w4 + (gx0 - 1)] : 0;
        int nC = cavlc_compute_nC(top_nc, left_nc, gy0 > 0, gx0 > 0);
        i16 dc_dec[16];
        if (cavlc_decode_block(&br, dc_dec, 16, BLK_LUMA_DC_16x16, nC) < 0)
            VERIFY_FAIL("luma DC decode error (nC=%d)", nC);
        for (int k = 0; k < 16; k++) {
            i16 expected = st->dc_levels_y[zz_scan_4x4[k]];
            if (dc_dec[k] != expected)
                VERIFY_FAIL("luma DC[zz=%d raster=%d] enc=%d dec=%d (nC=%d)",
                            k, zz_scan_4x4[k], expected, dc_dec[k], nC);
        }

        /* Luma AC blocks in scan order. */
        for (int s = 0; s < 16; s++) {
            int br_ = blk_scan_br[s], bc_ = blk_scan_bc[s];
            int idx = br_*4 + bc_;
            int gx = mb_c*4 + bc_, gy = mb_r*4 + br_;
            int t_nc = (gy > 0) ? dec_state.luma_nc[(gy - 1) * luma_w4 + gx] : 0;
            int l_nc = (gx > 0) ? dec_state.luma_nc[gy * luma_w4 + (gx - 1)] : 0;
            int nC = cavlc_compute_nC(t_nc, l_nc, gy > 0, gx > 0);

            i16 ac_dec[16] = {0};
            int dec_count = 0;
            if (dec_cbp_luma) {
                if (cavlc_decode_block(&br, ac_dec, 15, BLK_LUMA_AC, nC) < 0)
                    VERIFY_FAIL("blk_scan=%d luma AC decode error (nC=%d)", s, nC);
                for (int k = 0; k < 15; k++) {
                    /* Encoder's emit: zz[1..15] from raster order; decoder reads
                     * 15 coefs into positions [0..14] of ac_dec. */
                    i16 expected = st->ac_levels_y[idx][zz_scan_4x4[k+1]];
                    if (ac_dec[k] != expected)
                        VERIFY_FAIL("blk_scan=%d (br=%d bc=%d raster=%d) luma AC[zz=%d raster=%d] enc=%d dec=%d (nC=%d top=%d left=%d)",
                                    s, br_, bc_, idx, k+1, zz_scan_4x4[k+1],
                                    expected, ac_dec[k], nC, t_nc, l_nc);
                }
                for (int k = 0; k < 15; k++) if (ac_dec[k] != 0) dec_count++;
            }
            dec_state.luma_nc[gy * luma_w4 + gx] = dec_count;
            dec_state.luma_mode4[gy * luma_w4 + gx] = I4_DC;  /* effective mode for I_16x16 neighbor */
        }
    } else {
        /* I_4x4: 16 full blocks in scan order, conditional on quad bit.
         * Also reconstruct each block from the parsed levels and compare to
         * the encoder's internal recon (st->recon_y). */
        for (int s = 0; s < 16; s++) {
            int br_ = blk_scan_br[s], bc_ = blk_scan_bc[s];
            int idx = br_*4 + bc_;
            int gx = mb_c*4 + bc_, gy = mb_r*4 + br_;
            int t_nc = (gy > 0) ? dec_state.luma_nc[(gy - 1) * luma_w4 + gx] : 0;
            int l_nc = (gx > 0) ? dec_state.luma_nc[gy * luma_w4 + (gx - 1)] : 0;
            int nC = cavlc_compute_nC(t_nc, l_nc, gy > 0, gx > 0);

            int quad_bit = (dec_cbp_luma >> (s / 4)) & 1;
            i16 ac_dec[16] = {0};
            int dec_count = 0;
            if (quad_bit) {
                if (cavlc_decode_block(&br, ac_dec, 16, BLK_LUMA_FULL, nC) < 0)
                    VERIFY_FAIL("blk_scan=%d (br=%d bc=%d raster=%d) luma FULL decode error (nC=%d top=%d left=%d)",
                                s, br_, bc_, idx, nC, t_nc, l_nc);
                for (int k = 0; k < 16; k++) {
                    i16 expected = st->ac_levels_y_full[idx][zz_scan_4x4[k]];
                    if (ac_dec[k] != expected)
                        VERIFY_FAIL("blk_scan=%d (br=%d bc=%d raster=%d) luma FULL[zz=%d raster=%d] enc=%d dec=%d (nC=%d top_nc=%d left_nc=%d)",
                                    s, br_, bc_, idx, k, zz_scan_4x4[k],
                                    expected, ac_dec[k], nC, t_nc, l_nc);
                }
                for (int k = 0; k < 16; k++) if (ac_dec[k] != 0) dec_count++;
            }
            dec_state.luma_nc[gy * luma_w4 + gx] = dec_count;
            dec_state.luma_mode4[gy * luma_w4 + gx] = dec_modes4[s];
        }
    }

    /* === Chroma DC (UV) === */
    if (dec_cbp_chroma >= 1) {
        i16 udc_dec[4], vdc_dec[4];
        if (cavlc_decode_block(&br, udc_dec, 4, BLK_CHROMA_DC, -1) < 0)
            VERIFY_FAIL("chroma U DC decode error");
        if (cavlc_decode_block(&br, vdc_dec, 4, BLK_CHROMA_DC, -1) < 0)
            VERIFY_FAIL("chroma V DC decode error");
        for (int k = 0; k < 4; k++) {
            if (udc_dec[k] != st->dc_levels_u[k])
                VERIFY_FAIL("chroma U DC[%d] enc=%d dec=%d", k, st->dc_levels_u[k], udc_dec[k]);
            if (vdc_dec[k] != st->dc_levels_v[k])
                VERIFY_FAIL("chroma V DC[%d] enc=%d dec=%d", k, st->dc_levels_v[k], vdc_dec[k]);
        }
    }

    /* === Chroma AC === */
    for (int comp = 0; comp < 2; comp++) {
        u8 *cnc = (comp == 0) ? dec_state.chroma_u_nc : dec_state.chroma_v_nc;
        const i16 (*enc_ac)[16] = (comp == 0) ? st->ac_levels_u : st->ac_levels_v;
        for (int br_ = 0; br_ < 2; br_++)
            for (int bc_ = 0; bc_ < 2; bc_++) {
                int idx = br_*2 + bc_;
                int gx = mb_c*2 + bc_, gy = mb_r*2 + br_;
                int t_nc = (gy > 0) ? cnc[(gy - 1) * chroma_w4 + gx] : 0;
                int l_nc = (gx > 0) ? cnc[gy * chroma_w4 + (gx - 1)] : 0;
                int nC = cavlc_compute_nC(t_nc, l_nc, gy > 0, gx > 0);
                i16 ac_dec[16] = {0};
                int dec_count = 0;
                if (dec_cbp_chroma == 2) {
                    if (cavlc_decode_block(&br, ac_dec, 15, BLK_CHROMA_AC, nC) < 0)
                        VERIFY_FAIL("chroma %c AC blk(%d,%d) decode error (nC=%d)",
                                    comp ? 'V' : 'U', br_, bc_, nC);
                    for (int k = 0; k < 15; k++) {
                        i16 expected = enc_ac[idx][zz_scan_4x4[k+1]];
                        if (ac_dec[k] != expected)
                            VERIFY_FAIL("chroma %c AC blk(%d,%d) [zz=%d] enc=%d dec=%d (nC=%d)",
                                        comp ? 'V' : 'U', br_, bc_, k+1,
                                        expected, ac_dec[k], nC);
                    }
                    for (int k = 0; k < 15; k++) if (ac_dec[k] != 0) dec_count++;
                }
                cnc[gy * chroma_w4 + gx] = dec_count;
            }
    }

    return 0;
}
#endif /* MB_SELFDECODE */

static int encode_mb_emit(const u8 *src_y,  int stride_y,
                          const u8 *src_uv, int stride_uv,
                          u8 *recon_y, int recon_stride_y,
                          u8 *recon_uv, int recon_stride_uv,
                          int mb_r, int mb_c, int width, int height, int mbs_w,
                          int qp_y, int qp_c, nc_state_t *ncs,
                          bitstream_t *bs, int *qp_prev)
{
    mb_state_t st = {0};
    st.mb_r = mb_r;
    st.mb_c = mb_c;
    st.qp_y = qp_y;
    st.qp_c = qp_c;

    mb_fetch(src_y, stride_y, src_uv, stride_uv,
             recon_y, recon_stride_y, recon_uv, recon_stride_uv,
             width, height, &st);
    dump_mb_src(&st);
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
    /* the MB's QP takes effect only when mb_qp_delta is transmitted */
    int emits_delta = !st.mb_type_is_i4x4 || st.cbp_luma != 0 || st.cbp_chroma != 0;
    int qp_eff = emits_delta ? qp_y : *qp_prev;
    arena_dbk[mb_r * mbs_w + mb_c].intra = 1;
    arena_dbk[mb_r * mbs_w + mb_c].qp = (u8)qp_eff;
    arena_dbk[mb_r * mbs_w + mb_c].nz = 0;
    arena_dbk[mb_r * mbs_w + mb_c].mvx = 0;
    arena_dbk[mb_r * mbs_w + mb_c].mvy = 0;

    dump_mb_vector(&st, recon_y, recon_stride_y, mbs_w, ncs);

#ifdef MB_SELFDECODE
    int mb_start_bit = bs->byte_pos * 8 + bs->n_in_cur;
#endif
    int rc = mb_cavlc_emit(&st, ncs, bs, 0, qp_y - *qp_prev);
    *qp_prev = qp_eff;
#ifdef MB_SELFDECODE
    if (dec_state.initialized) {
        verify_mb_at(bs, mb_start_bit, &st, ncs);
        verify_state_match(&st, ncs);
    }
#endif
    return rc;
}


/* ============================================================================
 * P frames: 16x16 motion compensation from the previous frame, P_Skip,
 * intra MBs (refresh band + budget), and the P slice syntax.
 * ============================================================================ */

/* lambda for the motion search (SAD / SATD domain) */
static int me_lambda(int qp)
{
    /* 0.92 * 2^((qp-12)/6), integer */
    static const int tab[52] = {
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,4,4,5,5,6,7,8,9,10,12,13,15,
        16,18,21,23,26,29,33,37,42,47,52,59,66,74,83,93 };
    return tab[qp < 0 ? 0 : (qp > 51 ? 51 : qp)];
}

/* estimated CAVLC bits of the chroma residual of st (levels already quantized) */
static int chroma_bits_estimate(const mb_state_t *st)
{
    int bits = 0;
    i16 zz[16];
    for (int k = 0; k < 4; k++) zz[k] = st->dc_levels_u[k];
    bits += cavlc_estimate_block_bits(zz, 4, BLK_CHROMA_DC, -1);
    for (int k = 0; k < 4; k++) zz[k] = st->dc_levels_v[k];
    bits += cavlc_estimate_block_bits(zz, 4, BLK_CHROMA_DC, -1);
    for (int idx = 0; idx < 4; idx++) {
        for (int k = 0; k < 16; k++) zz[k] = st->ac_levels_u[idx][zz_scan_4x4[k]];
        bits += cavlc_estimate_block_bits(&zz[1], 15, BLK_CHROMA_AC, 0);
        for (int k = 0; k < 16; k++) zz[k] = st->ac_levels_v[idx][zz_scan_4x4[k]];
        bits += cavlc_estimate_block_bits(&zz[1], 15, BLK_CHROMA_AC, 0);
    }
    return bits;
}

/* chroma stages 2..6 for the prediction already in st->pred_u / pred_v */
static void chroma_code(mb_state_t *st)
{
    mb_residual(st);
    mb_transform(st);
    mb_quantize(st);
    mb_reconstruct(st);
}

/* Inter luma coding for vector (st->mvx, st->mvy): motion-compensated
 * prediction, 16 x (residual, DCT, inter quant, dequant, IDCT, recon).
 * Returns the estimated CAVLC bits of the 16 luma blocks. */
static int inter_luma_code(mb_state_t *st, const ref_planes_t *rp)
{
    mc_luma_16x16(rp, st->mb_r, st->mb_c, st->mvx, st->mvy, st->pred_y);
    int bits = 0;
    for (int br = 0; br < 4; br++)
        for (int bc = 0; bc < 4; bc++) {
            int idx = br * 4 + bc;
            i16 res[16], dct[16], zz[16];
            residual_4x4(st->src_y, st->pred_y, br, bc, res);
            dct4x4(res, dct);
            quant_4x4(dct, st->ac_levels_y_full[idx], st->qp_y, 0);
            zigzag_4x4(st->ac_levels_y_full[idx], zz);
            bits += cavlc_estimate_block_bits(zz, 16, BLK_LUMA_FULL, 0);
            i32 dq[16], rr[16];
            iquant_4x4(st->ac_levels_y_full[idx], dq, st->qp_y);
            idct4x4(dq, rr);
            recon_4x4(st->recon_y, st->pred_y, br, bc, rr);
        }
    return bits;
}

/* ue(v) length of a coded_block_pattern codeNum */
static int ue_bits(int codenum)
{
    int n = 0; unsigned v = (unsigned)codenum + 1;
    while (v > 1) { v >>= 1; n++; }
    return 2 * n + 1;
}

/* One MB of a P slice. Returns 0 on success. */
static int encode_mb_p(const u8 *src_y,  int stride_y,
                       const u8 *src_uv, int stride_uv,
                       u8 *recon_y, int recon_stride_y,
                       u8 *recon_uv, int recon_stride_uv,
                       int mb_r, int mb_c, int width, int height, int mbs_w,
                       int qp_y, int qp_c, nc_state_t *ncs, const ref_planes_t *rp,
                       mv_field_t *mf, const encode_cfg_t *cfg, int *intra_budget,
                       bitstream_t *bs, int *skip_run, encode_pstats_t *ps, int *qp_prev)
{
    mb_state_t st = {0};
    st.mb_r = mb_r; st.mb_c = mb_c; st.qp_y = qp_y; st.qp_c = qp_c;
    mb_fetch(src_y, stride_y, src_uv, stride_uv, recon_y, recon_stride_y,
             recon_uv, recon_stride_uv, width, height, &st);

    int in_band = (cfg->refresh_cols > 0) &&
                  (mb_c >= cfg->refresh_col && mb_c < cfg->refresh_col + cfg->refresh_cols);
    /* strict refresh: left of the band only the refreshed columns of the
     * previous frame may be referenced; an intra MB whose above-right
     * neighbour is unrefreshed must not read it */
    int strict = cfg->refresh_strict && cfg->refresh_cols > 0;
    int clean_right = (strict && mb_c < cfg->refresh_col) ? cfg->refresh_col * 16 : 0;
    st.no_topright_modes = strict && (mb_c + 1 < mbs_w) &&
                           (mb_c + 1 >= cfg->refresh_col + cfg->refresh_cols) &&
                           (mb_c < cfg->refresh_col + cfg->refresh_cols);

    /* ---- inter candidate: predictor, search, code ---- */
    int pred_x = 0, pred_y = 0, skip_x = 0, skip_y = 0;
    mv_predict_16x16(mf, mb_r, mb_c, &pred_x, &pred_y);
    mv_skip_16x16(mf, mb_r, mb_c, &skip_x, &skip_y);
    int bits_inter = 0x7fffffff;
    mb_state_t si;              /* inter-coded copy */
    if (!in_band) {
        si = st;
        si.is_inter = 1;
        me_params_t mp = { cfg->me_range, me_lambda(qp_y), clean_right };
        me_search_16x16(rp, si.src_y, mb_r, mb_c, pred_x, pred_y, &mp, &si.mvx, &si.mvy);
        si.mvd_x = si.mvx - pred_x; si.mvd_y = si.mvy - pred_y;
        int luma_bits = inter_luma_code(&si, rp);
        mc_chroma_8x8(rp, mb_r, mb_c, si.mvx, si.mvy, si.pred_u, si.pred_v);
        chroma_code(&si);
        mb_compute_cbp(&si);
        int cbp = (si.cbp_luma & 0xF) | ((si.cbp_chroma & 3) << 4);
        bits_inter = 1 + mvd_bits(si.mvd_x) + mvd_bits(si.mvd_y) + ue_bits(cbp_inter_to_codenum[cbp]);
        if (cbp) bits_inter += 1 + luma_bits + chroma_bits_estimate(&si);
        /* P_Skip: the skip vector with nothing to code costs the skip run only */
        if (si.mvx == skip_x && si.mvy == skip_y && cbp == 0) { si.is_skip = 1; bits_inter = 1; }
    }

    /* ---- intra candidate: forced in the band, otherwise within the budget ---- */
    int use_intra = in_band;
    if (!in_band && *intra_budget > 0) {
        mb_mode_decide(mbs_w, recon_y, recon_stride_y, &st);
        chroma_code(&st);
        int bits_intra = (st.dbg_bits_a < st.dbg_bits_b ? st.dbg_bits_a : st.dbg_bits_b) +
                         chroma_bits_estimate(&st);
        if (bits_intra < bits_inter) { use_intra = 1; (*intra_budget)--; }
    } else if (in_band) {
        mb_mode_decide(mbs_w, recon_y, recon_stride_y, &st);
        chroma_code(&st);
    }

    mb_state_t *w = use_intra ? &st : &si;
    dbk_mb_t *dk = &arena_dbk[mb_r * mbs_w + mb_c];
    int emits_delta;
    if (use_intra) {
        mb_compute_cbp(w);
        emits_delta = !w->mb_type_is_i4x4 || w->cbp_luma != 0 || w->cbp_chroma != 0;
        mf->is_intra[mb_r * mbs_w + mb_c] = 1;
        mf->mvx[mb_r * mbs_w + mb_c] = 0; mf->mvy[mb_r * mbs_w + mb_c] = 0;
        dk->intra = 1; dk->nz = 0; dk->mvx = 0; dk->mvy = 0;
        if (ps) ps->mbs_intra++;
    } else {
        emits_delta = !w->is_skip && (w->cbp_luma != 0 || w->cbp_chroma != 0);
        mf->is_intra[mb_r * mbs_w + mb_c] = 0;
        mf->mvx[mb_r * mbs_w + mb_c] = (i16)w->mvx; mf->mvy[mb_r * mbs_w + mb_c] = (i16)w->mvy;
        dk->intra = 0; dk->mvx = (i16)w->mvx; dk->mvy = (i16)w->mvy; dk->nz = 0;
        for (int idx = 0; idx < 16; idx++)
            for (int k = 0; k < 16; k++)
                if (w->ac_levels_y_full[idx][k]) { dk->nz |= (u16)(1u << idx); break; }
        if (ps) { if (w->is_skip) ps->mbs_skip++; else ps->mbs_inter++; }
    }
    dk->qp = (u8)(emits_delta ? qp_y : *qp_prev);
    /* mb_qp_delta is in [-26, 25] (7.4.5); the decoder adds it mod 52 */
    int qp_delta = qp_y - *qp_prev;
    if (qp_delta > 25) qp_delta -= 52; else if (qp_delta < -26) qp_delta += 52;
    if (emits_delta) *qp_prev = qp_y;

    copy_out_mb_luma(recon_y, recon_stride_y, mb_r, mb_c, w->recon_y);
    copy_out_mb_chroma_combine(recon_uv, recon_stride_uv, mb_r, mb_c, w->recon_u, w->recon_v);

    /* ---- emission ---- */
    int luma_w4 = ncs->luma_w4, chroma_w4 = ncs->chroma_w4;
    if (use_intra) {
        bs_put_ue(bs, *skip_run); *skip_run = 0;
        mb_cavlc_emit(w, ncs, bs, 5, qp_delta);
        return 0;
    }
    if (w->is_skip) {
        (*skip_run)++;
        /* neighbour state: nothing coded, not intra 4x4 */
        for (int br = 0; br < 4; br++)
            for (int bc = 0; bc < 4; bc++) {
                int gx = mb_c * 4 + bc, gy = mb_r * 4 + br;
                ncs->luma_nc[gy * luma_w4 + gx] = 0;
                ncs->luma_mode4[gy * luma_w4 + gx] = I4_DC;
            }
        for (int br = 0; br < 2; br++)
            for (int bc = 0; bc < 2; bc++) {
                int gx = mb_c * 2 + bc, gy = mb_r * 2 + br;
                ncs->chroma_u_nc[gy * chroma_w4 + gx] = 0;
                ncs->chroma_v_nc[gy * chroma_w4 + gx] = 0;
            }
        return 0;
    }
    /* P_L0_16x16 */
    bs_put_ue(bs, *skip_run); *skip_run = 0;
    bs_put_ue(bs, 0);                         /* mb_type P_L0_16x16 */
    bs_put_se(bs, w->mvd_x);                  /* mvd_l0 (ref_idx omitted: one reference) */
    bs_put_se(bs, w->mvd_y);
    int cbp = (w->cbp_luma & 0xF) | ((w->cbp_chroma & 3) << 4);
    bs_put_ue(bs, cbp_inter_to_codenum[cbp]);
    if (cbp) bs_put_se(bs, qp_delta);         /* mb_qp_delta */
    /* luma 4x4 blocks in scan order, per coded 8x8 quadrant */
    for (int s = 0; s < 16; s++) {
        int br = blk_scan_br[s], bc = blk_scan_bc[s], idx = br * 4 + bc;
        i16 zz[16];
        for (int k = 0; k < 16; k++) zz[k] = w->ac_levels_y_full[idx][zz_scan_4x4[k]];
        int gx = mb_c * 4 + bc, gy = mb_r * 4 + br;
        int top_nc  = (gy > 0) ? ncs->luma_nc[(gy - 1) * luma_w4 + gx] : 0;
        int left_nc = (gx > 0) ? ncs->luma_nc[gy * luma_w4 + (gx - 1)] : 0;
        int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);
        int quad_bit = (w->cbp_luma >> (s / 4)) & 1;
        if (quad_bit) cavlc_encode_block(bs, zz, 16, BLK_LUMA_FULL, nC);
        ncs->luma_nc[gy * luma_w4 + gx] = quad_bit ? count_nonzero(zz, 16) : 0;
        ncs->luma_mode4[gy * luma_w4 + gx] = I4_DC;
    }
    if (w->cbp_chroma >= 1) {
        cavlc_encode_block(bs, w->dc_levels_u, 4, BLK_CHROMA_DC, -1);
        cavlc_encode_block(bs, w->dc_levels_v, 4, BLK_CHROMA_DC, -1);
    }
    for (int comp = 0; comp < 2; comp++) {
        u8 *cnc = (comp == 0) ? ncs->chroma_u_nc : ncs->chroma_v_nc;
        i16 (*ac_levels)[16] = (comp == 0) ? w->ac_levels_u : w->ac_levels_v;
        for (int br = 0; br < 2; br++)
            for (int bc = 0; bc < 2; bc++) {
                int idx = br * 2 + bc;
                i16 zz[16];
                for (int k = 0; k < 16; k++) zz[k] = ac_levels[idx][zz_scan_4x4[k]];
                int gx = mb_c * 2 + bc, gy = mb_r * 2 + br;
                int top_nc  = (gy > 0) ? cnc[(gy - 1) * chroma_w4 + gx] : 0;
                int left_nc = (gx > 0) ? cnc[gy * chroma_w4 + (gx - 1)] : 0;
                int nC = cavlc_compute_nC(top_nc, left_nc, gy > 0, gx > 0);
                if (w->cbp_chroma == 2) cavlc_encode_block(bs, &zz[1], 15, BLK_CHROMA_AC, nC);
                cnc[gy * chroma_w4 + gx] = (w->cbp_chroma == 2) ? count_nonzero(&zz[1], 15) : 0;
            }
    }
    return 0;
}

/* keep the reconstruction as the next frame's reference */
static void keep_reference(const u8 *recon_y, const u8 *recon_uv, int width, int height)
{
    memcpy(arena_ref_y,  recon_y,  (size_t)width * height);
    memcpy(arena_ref_uv, recon_uv, (size_t)width * (height / 2));
    ref_valid = 1; ref_w = width; ref_h = height;
}

int encode_frame_h264(int width, int height, int qp,
                      const u8 *src_y,  int stride_y,
                      const u8 *src_uv, int stride_uv,
                      u8 *recon_y_out,  int recon_stride_y,
                      u8 *recon_uv_out, int recon_stride_uv,
                      u8 *bs_out, int bs_max_size, int frame_num,
                      encode_stats_t *stats)
{
    encode_cfg_t cfg = { 0, frame_num, 16, 0, 0, 0, 1, 1, 0, 30, 0, 51, 4, 1, 1, 1, 0 };
    return encode_frame_h264_ext(width, height, qp, src_y, stride_y, src_uv, stride_uv,
                                 recon_y_out, recon_stride_y, recon_uv_out, recon_stride_uv,
                                 bs_out, bs_max_size, &cfg, stats, NULL);
}

int encode_frame_h264_ext(int width, int height, int qp,
                          const u8 *src_y,  int stride_y,
                          const u8 *src_uv, int stride_uv,
                          u8 *recon_y_out,  int recon_stride_y,
                          u8 *recon_uv_out, int recon_stride_uv,
                          u8 *bs_out, int bs_max_size,
                          const encode_cfg_t *cfg,
                          encode_stats_t *stats, encode_pstats_t *pstats)
{
    int frame_num = cfg->frame_num;
    int is_p = (cfg->frame_type == 1);
    if (is_p && !(ref_valid && ref_w == width && ref_h == height)) return -7;
    if (width  % 16 != 0) return -1;
    int rc_on = cfg->rc_bps > 0;
    if (cfg->rc_reset || !is_p) {
        if (cfg->rc_reset) {
            rc.fill = 0; rc.have_i = 0; rc.have_p = 0; rc.prev_mbs = 0; rc.overflow = 0;
            /* No history yet, so the caller's qp argument is the seed (this is
             * what --bitrate documents). The frame model slews at most 3 QP per
             * frame, so a seed far from the right answer costs several frames
             * of ramp; the MB loop still corrects inside the first frame. */
            rc.qp_last = qp;
            if (rc_on) {
                if (rc.qp_last < cfg->rc_qp_min) rc.qp_last = cfg->rc_qp_min;
                if (rc.qp_last > cfg->rc_qp_max) rc.qp_last = cfg->rc_qp_max;
            }
        }
        rc.pps_qp = qp;     /* the PPS is rewritten with every IDR */
    }
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

    int dst_pos = 0;
    int n;
    if (!is_p) {
        /* === SPS + PPS (IDR only) === */
        n = nal_write_sps(bs_out + dst_pos, bs_max_size - dst_pos, width, height, qp);
        if (n < 0) return -4;
        dst_pos += n;
        n = nal_write_pps(bs_out + dst_pos, bs_max_size - dst_pos, rc.pps_qp);
        if (n < 0) return -4;
        dst_pos += n;
    }

    /* ---- rate control: frame target and frame QP ---- */
    long frame_budget = 0, frame_target = 0, bucket_cap = 0;
    int qp_frame = qp;
    double rc_w_total = 0;
    if (rc_on) {
        frame_budget = cfg->rc_bps / (cfg->rc_fps > 0 ? cfg->rc_fps : 30);
        bucket_cap = (long)(cfg->rc_bucket_frames > 0 ? cfg->rc_bucket_frames : 4) * frame_budget;
        /* Every frame repays the backlog over ~2 frames. On top of that an
         * intra frame may draw ahead, because it costs several P frames and a
         * flat target would guarantee an overshoot the bucket absorbs blind.
         * The draw is bounded twice: by the room left in the bucket, and by
         * what the P frames of this GOP can plausibly give back. With a GOP of
         * 1 (intra only) nothing can repay, so there is no draw and the intra
         * frame lives on the plain per-frame budget like any other. */
        long extra = 0;
        if (!is_p) {
            int gop = cfg->rc_gop > 1 ? cfg->rc_gop : 1;
            long room  = bucket_cap - rc.fill;
            long repay = (long)(gop - 1) * frame_budget / 2;
            extra = room > 0 ? room * 3 / 4 : 0;
            if (extra > repay) extra = repay;
        }
        frame_target = frame_budget - rc.fill / 2 + extra;
        if (frame_target < frame_budget / 4) frame_target = frame_budget / 4;
        double c = is_p ? rc.c_p : rc.c_i;
        int have = is_p ? rc.have_p : rc.have_i;
        if (!have && is_p && rc.have_i) { c = rc.c_i / 4.0; have = 1; }   /* first P: a guess */
        if (have) {
            qp_frame = ilog2_x6(c / (double)frame_target);
            if (qp_frame > rc.qp_last + 3) qp_frame = rc.qp_last + 3;
            if (qp_frame < rc.qp_last - 3) qp_frame = rc.qp_last - 3;
        } else {
            qp_frame = rc.qp_last;
        }
        if (qp_frame < cfg->rc_qp_min) qp_frame = cfg->rc_qp_min;
        if (qp_frame > cfg->rc_qp_max) qp_frame = cfg->rc_qp_max;
        for (int i = 0; i < rc.prev_mbs; i++) rc_w_total += arena_mb_bits[i];
    }
    qp = qp_frame;
    if (cfg->rc_reset) dump_idx = 0;

    /* Hardware model (rc_mb == 3): what the host computes once per frame and
     * writes to the kernel's RC registers. The map weights are the previous
     * frame's per-MB bits saturated to 16 bits (what the kernel stores), or 1
     * per MB when there is no usable map; scale = target * 2^16 / w_total. */
    int      hw_lag = cfg->rc_lag > 0 ? cfg->rc_lag : 2;
    int      hw_map = (rc_on && cfg->rc_mb == 3 && rc.prev_mbs == mb_count);
    uint64_t hw_wtotal = 0;
    u32      hw_scale = 0;
    if (rc_on && cfg->rc_mb == 3) {
        if (hw_map) for (int i = 0; i < mb_count; i++) hw_wtotal += arena_mb_bits[i] > 65535 ? 65535 : arena_mb_bits[i];
        else hw_wtotal = mb_count;
        if (hw_wtotal == 0) hw_wtotal = 1;
        uint64_t s = ((uint64_t)frame_target << 16) / hw_wtotal;
        hw_scale = s > 0xFFFFFFFFu ? 0xFFFFFFFFu : (u32)s;
    }
    /* DCC_DUMP_RC=<path>: append the per-frame register values the host
     * writes to the kernel -- qp_frame target scale(hex) qp_min qp_max
     * map_valid lag en -- for the AXI testbench to replay. */
    if (rc_on) {
        const char *rp = getenv("DCC_DUMP_RC");
        if (rp) {
            FILE *rf = fopen(rp, "a");
            if (rf) { fprintf(rf, "%d %ld %08X %d %d %d %d %d\n", qp_frame, frame_target, hw_scale,
                              cfg->rc_qp_min, cfg->rc_qp_max, hw_map, hw_lag, cfg->rc_mb == 3); fclose(rf); }
        }
    }
    FILE *mbqp_f = NULL;
    { const char *mp = getenv("DCC_DUMP_MBQP"); if (mp) mbqp_f = fopen(mp, "a"); }

    /* === Slice RBSP === */
    bitstream_t bs;
    bs_init(&bs, arena_rbsp, ARENA_RBSP_BYTES);

    /* Slice header */
    bs_put_ue(&bs, 0);                       /* first_mb_in_slice */
    bs_put_ue(&bs, is_p ? 5 : 7);            /* slice_type: 5 = P, 7 = I (all slices same) */
    bs_put_ue(&bs, 0);                       /* pic_parameter_set_id */
    bs_put_bits(&bs, frame_num & 0xF, 4);    /* frame_num */
    if (!is_p) {
        bs_put_ue(&bs, frame_num & 0xF);     /* idr_pic_id */
        bs_put_bits(&bs, 0, 1);              /* no_output_of_prior_pics_flag */
        bs_put_bits(&bs, 0, 1);              /* long_term_reference_flag */
    } else {
        bs_put_bits(&bs, 0, 1);              /* num_ref_idx_active_override_flag */
        bs_put_bits(&bs, 0, 1);              /* ref_pic_list_reordering_flag_l0 */
        bs_put_bits(&bs, 0, 1);              /* adaptive_ref_pic_marking_mode_flag (sliding window) */
    }
    bs_put_se(&bs, qp - rc.pps_qp);          /* slice_qp_delta */
    if (cfg->deblock) {
        bs_put_ue(&bs, 0);                   /* disable_deblocking_filter_idc = 0: filter on */
        bs_put_se(&bs, 0);                   /* slice_alpha_c0_offset_div2 */
        bs_put_se(&bs, 0);                   /* slice_beta_offset_div2 */
    } else {
        /* the decoder must not deblock either, so its output matches the
         * unfiltered reconstruction (the I-only hardware kernel's mode) */
        bs_put_ue(&bs, 1);                   /* disable_deblocking_filter_idc = 1 */
    }

    /* Recon buffers from the static arena. */
    u8 *recon_y_int  = arena_recon_y;
    u8 *recon_uv_int = arena_recon_uv;
    memset(recon_y_int,  128, (size_t)width * height);
    memset(recon_uv_int, 128, (size_t)width * (height / 2));

    /* P frame: padded reference from the previous reconstruction, and the
     * vector field of this picture */
    ref_planes_t rp;
    mv_field_t mf = { arena_mvx, arena_mvy, arena_mb_intra, mbs_w, mbs_h };
    if (is_p) {
        ref_build(&rp, arena_pad_y, arena_pad_u, arena_pad_v,
                  arena_ref_y, width, arena_ref_uv, width, width, height);
        memset(arena_mb_intra, 0, (size_t)mb_count);
        memset(arena_mvx, 0, (size_t)mb_count * sizeof(i16));
        memset(arena_mvy, 0, (size_t)mb_count * sizeof(i16));
    }
    if (pstats) { pstats->mbs_intra = 0; pstats->mbs_inter = 0; pstats->mbs_skip = 0; }

    nc_state_t ncs;
    ncs.luma_nc     = arena_luma_nc;
    ncs.chroma_u_nc = arena_chroma_u_nc;
    ncs.chroma_v_nc = arena_chroma_v_nc;
    ncs.luma_mode4  = arena_luma_mode4;
    ncs.luma_w4     = luma_w4;
    ncs.luma_h4     = luma_h4;
    ncs.chroma_w4   = chroma_w4;
    ncs.chroma_h4   = chroma_h4;
    memset(ncs.luma_nc,     0, (size_t)luma_w4   * luma_h4);
    memset(ncs.chroma_u_nc, 0, (size_t)chroma_w4 * chroma_h4);
    memset(ncs.chroma_v_nc, 0, (size_t)chroma_w4 * chroma_h4);
    memset(ncs.luma_mode4,  0, (size_t)luma_w4   * luma_h4);

    int qp_c = chroma_qp(qp, 0);

#ifdef MB_SELFDECODE
    if (!is_p) dec_state_init(luma_w4, chroma_w4);
#endif

    /* Per-MB encoding into the slice RBSP */
    int payload_start_bit = bs.byte_pos * 8 + bs.n_in_cur;
    int skip_run = 0;
    int intra_budget = cfg->intra_budget;
    int qp_prev = qp;              /* QP_Y,PRED: the slice QP, then the last transmitted MB QP */
    int qp_mb = qp;
    long qp_sum = 0; int qp_lo = qp, qp_hi = qp;
    double w_cum = 0;
    u32 hw_spent = 0; uint64_t hw_acc = 0;
    for (int r = 0; r < mbs_h; r++) {
        for (int c = 0; c < mbs_w; c++) {
            int i = r * mbs_w + c;
            long bits_before = bs.byte_pos * 8L + bs.n_in_cur;
            /* ---- MB QP from the spend so far vs the expected spend ---- */
            if (rc_on && cfg->rc_mb == 3) {
                /* Hardware model: the kernel knows the bits of MB k only once
                 * the merger has emitted it, which is hw_lag MBs behind the
                 * decision, so MB i is stepped on the bits through MB i-1-lag
                 * and the map weights through the same MB. */
                if (i > hw_lag) {
                    int k = i - 1 - hw_lag;
                    u32 wk = hw_map ? (arena_mb_bits[k] > 65535 ? 65535 : arena_mb_bits[k]) : 1;
                    hw_spent += arena_mb_bits_cur[k];
                    hw_acc   += (uint64_t)wk * hw_scale;
                    uint64_t expect64 = hw_acc >> 16;
                    u32 expect = expect64 > 0xFFFFFFFFu ? 0xFFFFFFFFu : (u32)expect64;
                    int adj = 0;
                    if ((uint64_t)expect * 100 > (uint64_t)frame_target && hw_spent > 0)
                        adj = rc_hw_log2x6(hw_spent, expect)
                            + rc_hw_div4((int64_t)hw_spent - (int64_t)expect, (u32)frame_target);
                    int qp_target = qp_frame + adj;
                    if (qp_target < cfg->rc_qp_min) qp_target = cfg->rc_qp_min;
                    if (qp_target > cfg->rc_qp_max) qp_target = cfg->rc_qp_max;
                    if (qp_target > qp_mb) qp_mb++; else if (qp_target < qp_mb) qp_mb--;
                }
                if (mbqp_f) fprintf(mbqp_f, "%d %d %d\n", dump_idx, i, qp_mb);
            } else if (rc_on && cfg->rc_mb && i > 0) {
                long spent = bits_before - payload_start_bit;
                double expect;
                if (rc.prev_mbs == mb_count && rc_w_total > 0 && cfg->rc_mb == 1) expect = (double)frame_target * (w_cum / rc_w_total);
                else expect = (double)frame_target * ((double)i / mb_count);
                int adj = 0;
                if (expect > frame_target * 0.01 && spent > 0) {
                    adj = ilog2_x6((double)spent / expect);          /* rate ratio -> QP steps */
                    adj += (int)(4.0 * (spent - expect) / (double)frame_target);  /* accumulated error */
                }
                int qp_target = qp_frame + adj;
                if (qp_target < cfg->rc_qp_min) qp_target = cfg->rc_qp_min;
                if (qp_target > cfg->rc_qp_max) qp_target = cfg->rc_qp_max;
                if (qp_target > qp_mb) qp_mb++; else if (qp_target < qp_mb) qp_mb--;
            }
            int qp_c_mb = rc_on ? chroma_qp(qp_mb, 0) : qp_c;
            if (is_p)
                encode_mb_p(src_y, stride_y, src_uv, stride_uv,
                            recon_y_int, width, recon_uv_int, width,
                            r, c, width, height, mbs_w, qp_mb, qp_c_mb, &ncs, &rp, &mf, cfg,
                            &intra_budget, &bs, &skip_run, pstats, &qp_prev);
            else
                encode_mb_emit(src_y, stride_y, src_uv, stride_uv,
                               recon_y_int, width, recon_uv_int, width,
                               r, c, width, height, mbs_w,
                               qp_mb, qp_c_mb, &ncs, &bs, &qp_prev);
            long bits_after = bs.byte_pos * 8L + bs.n_in_cur;
            arena_mb_bits_cur[i] = (u32)(bits_after - bits_before);
            if (rc.prev_mbs == mb_count) w_cum += arena_mb_bits[i];
            qp_sum += qp_mb;
            if (qp_mb < qp_lo) qp_lo = qp_mb;
            if (qp_mb > qp_hi) qp_hi = qp_mb;
        }
    }
    if (is_p && skip_run > 0) bs_put_ue(&bs, skip_run);   /* trailing skipped MBs */

    /* ---- rate control: fit the model on this frame, update the bucket ---- */
    {
        long frame_bits = bs.byte_pos * 8L + bs.n_in_cur - payload_start_bit + 64;   /* + header, NAL */
        double qp_avg = (double)qp_sum / mb_count;
        double p2 = 1.0; for (int k = 0; k < (int)(qp_avg + 0.5); k++) p2 *= 1.122462;   /* 2^(qp/6) */
        if (is_p) { rc.c_p = frame_bits * p2; rc.have_p = 1; }
        else      { rc.c_i = frame_bits * p2; rc.have_i = 1; }
        rc.qp_last = qp_frame;
        if (rc_on) {
            rc.fill += frame_bits - frame_budget;
            if (rc.fill < 0) rc.fill = 0;
            /* The bucket is the receiver's buffer: what does not fit is what
             * the link cannot carry. Clamp and count it -- a nonzero overflow
             * means the QP ceiling cannot meet the requested bit rate. */
            if (rc.fill > bucket_cap) { rc.overflow += rc.fill - bucket_cap; rc.fill = bucket_cap; }
        }
        memcpy(arena_mb_bits, arena_mb_bits_cur, (size_t)mb_count * sizeof(u32));
        rc.prev_mbs = mb_count;
        if (pstats) {
            pstats->qp_frame = qp_frame; pstats->qp_avg100 = (int)(qp_avg * 100 + 0.5);
            pstats->qp_lo = qp_lo; pstats->qp_hi = qp_hi; pstats->bucket_fill = rc.fill;
            pstats->frame_target = frame_target; pstats->bucket_cap = bucket_cap;
            pstats->rc_overflow = rc.overflow;
        }
    }

    /* DCC_DUMP_SLICE=<path>: the MB-layer bits of this slice, re-aligned to
     * start at bit 0, with the rbsp stop bit and zero padding -- exactly
     * the byte stream mb_pipeline_controller produces. One decimal byte
     * per line. */
    {
        const char *dp = getenv("DCC_DUMP_SLICE");
        const char *dps = getenv("DCC_DUMP_SLICE_SEQ");   /* <prefix><frame>.txt, one per frame */
        char dpath[512];
        if (!dp && dps) { snprintf(dpath, sizeof dpath, "%s%d.txt", dps, dump_idx); dp = dpath; }
        if (mbqp_f) fclose(mbqp_f);
        if (dp) {
            FILE *df = fopen(dp, "w");
            int end_bit = bs.byte_pos * 8 + bs.n_in_cur;
            /* flush the accumulator into the buffer without disturbing it */
            bitstream_t tmp = bs;
            bs_put_bits(&tmp, 0, 32);
            u8 acc = 0; int nacc = 0;
            for (int b = payload_start_bit; b < end_bit; b++) {
                int bit = (arena_rbsp[b >> 3] >> (7 - (b & 7))) & 1;
                acc = (u8)((acc << 1) | bit); nacc++;
                if (nacc == 8) { fprintf(df, "%d\n", acc); acc = 0; nacc = 0; }
            }
            acc = (u8)((acc << 1) | 1); nacc++;                     /* stop bit */
            while (nacc < 8) { acc = (u8)(acc << 1); nacc++; }
            fprintf(df, "%d\n", acc);
            fclose(df);
        }
    }

    bs_rbsp_trailing(&bs);
    int rbsp_len = bs_byte_count(&bs);
    if (bs.overflow) return -6;

    /* Wrap in the slice NAL */
    if (is_p) n = nal_emit_slice(bs_out + dst_pos, bs_max_size - dst_pos, arena_rbsp, rbsp_len);
    else      n = nal_emit_idr(bs_out + dst_pos, bs_max_size - dst_pos, arena_rbsp, rbsp_len);
    if (n < 0) return -6;
    dst_pos += n;

    /* The reference (and the output) is the deblocked picture; intra
     * prediction inside the frame used the unfiltered samples, as the spec
     * defines. */
    keep_reference(recon_y_int, recon_uv_int, width, height);
    if (cfg->deblock)
        deblock_frame(arena_ref_y, width, arena_ref_uv, width, mbs_w, mbs_h, arena_dbk);

    /* Optional recon copy-out (test bench / measurement only — the FPGA IP
     * does not write recon to host memory). */
    if (recon_y_out)
        for (int i = 0; i < height; i++)
            memcpy(&recon_y_out[i * recon_stride_y], &arena_ref_y[i * width], width);
    if (recon_uv_out)
        for (int i = 0; i < height/2; i++)
            memcpy(&recon_uv_out[i * recon_stride_uv], &arena_ref_uv[i * width], width);

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

    dump_idx++;
    return 0;
}
