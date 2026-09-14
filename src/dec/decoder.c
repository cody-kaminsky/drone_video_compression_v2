/* decoder.c — see decoder.h. */

#include "decoder.h"
#include "dec_nal.h"
#include "bitstream.h"
#include "cavlc.h"
#include "cavlc_tables.h"
#include "intra.h"
#include "transform.h"
#include "quant.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Build limits, named apart from the encoder's own. */
#define DEC_MAX_W 4096
#define DEC_MAX_H 2304

#define FAIL(fmt, ...) do { \
    fprintf(stderr, "decode: " fmt "\n", ##__VA_ARGS__); \
    return -1; \
} while (0)

/* The 4x4 block scan inside a macroblock, spec 6.4.3. Must match the
 * encoder's blk_scan_br/bc exactly or every nC and every residual lands in
 * the wrong place. */
static const int scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
static const int scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};

/* Zigzag: cavlc_decode_block returns coefficients in scan order, dequant and
 * the inverse transform want raster. zz_scan_4x4 is the encoder's own table
 * (src/cavlc.c), used rather than a second copy that could drift from it. */
#define zigzag4 zz_scan_4x4

/* ------------------------------------------------------------- state ----- */

typedef struct {
    int width, height, mbs_w, mbs_h;
    int luma_w4, chroma_w4;

    u8 *recon_y;            /* width * height            */
    u8 *recon_uv;           /* width * (height/2), NV12  */

    /* Per-4x4 total_coeff, for nC of the next block (spec 9.2.1). */
    u8 *luma_nc;            /* luma_w4  * (mbs_h*4) */
    u8 *chroma_u_nc;        /* chroma_w4 * (mbs_h*2) */
    u8 *chroma_v_nc;
    /* Per-4x4 intra mode, for predIntra4x4PredMode (spec 8.3.1.1). */
    u8 *luma_mode4;
    /* Per-MB "is I_4x4", since an I_16x16 neighbour predicts as DC. */
    u8 *mb_is_i4x4;

    int qp;                 /* running QP_Y, updated by mb_qp_delta */
    int chroma_qp_offset;
} dec_ctx_t;

static int clip_u8(int x) { return x < 0 ? 0 : (x > 255 ? 255 : x); }

/* --------------------------------------------------- neighbour samples --- */
/* Reconstructed neighbours come from the frame buffer itself. Intra
 * prediction reads unfiltered samples by spec, and this decoder does not
 * deblock, so the frame buffer is exactly the right source. */

static void gather_4x4(const dec_ctx_t *c, int x0, int y0,
                       u8 top[8], u8 left[4], u8 *tl,
                       int *avail_top, int *avail_left, int *avail_tl,
                       int *avail_tr)
{
    int i;
    *avail_top  = (y0 > 0);
    *avail_left = (x0 > 0);
    *avail_tl   = (x0 > 0 && y0 > 0);
    /* Top-right exists only if it is inside the frame AND already decoded.
     * Within a macroblock the scan order decides that; the caller passes the
     * result in via avail_tr, which we only use to replicate. */
    if (*avail_top)
        for (i = 0; i < 4; i++) top[i] = c->recon_y[(y0 - 1) * c->width + x0 + i];
    else
        for (i = 0; i < 4; i++) top[i] = 0;

    if (*avail_top && *avail_tr && x0 + 4 + 3 < c->width)
        for (i = 0; i < 4; i++) top[4 + i] = c->recon_y[(y0 - 1) * c->width + x0 + 4 + i];
    else
        for (i = 0; i < 4; i++) top[4 + i] = top[3];   /* spec 8.3.1.2.4 */

    if (*avail_left)
        for (i = 0; i < 4; i++) left[i] = c->recon_y[(y0 + i) * c->width + x0 - 1];
    else
        for (i = 0; i < 4; i++) left[i] = 0;

    *tl = *avail_tl ? c->recon_y[(y0 - 1) * c->width + x0 - 1] : 0;
}

/* Is the 4x4 block above-right of scan position s available? Mirrors the
 * encoder: inside a macroblock, blocks 5, 7, 13 and 15 of the scan have no
 * decoded above-right neighbour, and nor does anything on the frame's right
 * edge or top row. */
static int tr_avail_4x4(int s, int mb_r, int mb_c, int mbs_w)
{
    int br = scan_br[s], bc = scan_bc[s];
    int x4 = mb_c * 4 + bc, y4 = mb_r * 4 + br;
    if (y4 == 0) return 0;
    if (x4 + 1 >= mbs_w * 4) return 0;
    /* Within the macroblock the above-right block must already be decoded.
     * In the spec's scan those are exactly the odd-column blocks of rows 1
     * and 3 of the 4x4 grid, i.e. scan positions 3, 7, 11, 13, 15 have a
     * neighbour that comes later; 5 and 13 straddle the 8x8 boundary. The
     * simple correct rule: the above-right block's scan index must be lower. */
    if (br > 0) {
        int t, want_x = bc + 1, want_y = br - 1;
        /* Column 3 with a row above inside this macroblock: the above-right
         * block belongs to the NEXT macroblock, which is not decoded yet.
         * Only the top row of the macroblock can reach right, because there
         * it lands in the already-decoded row above. */
        if (want_x > 3) return 0;
        for (t = 0; t < 16; t++)
            if (scan_br[t] == want_y && scan_bc[t] == want_x) return t < s;
    }
    return 1;   /* br == 0: the row above is in the macroblock above */
}

/* --------------------------------------------------- residual helpers ---- */

/* Decode one 4x4 residual block and add it to pred, writing recon. */
static void add_residual_4x4(const i16 zz[16], int qp, int skip_dc, i32 dc,
                             const u8 *pred, int pred_stride,
                             u8 *dst, int dst_stride)
{
    i16 lev[16];
    i32 coef[16], res[16];
    int i;

    /* zigzag -> raster */
    memset(lev, 0, sizeof lev);
    for (i = 0; i < 16; i++) lev[zigzag4[i]] = zz[i];

    iquant_4x4(lev, coef, qp);
    if (skip_dc) coef[0] = dc;          /* DC comes from the Hadamard plane */
    idct4x4(coef, res);

    /* idct4x4 leaves its result scaled by 64; the encoder's recon_4x4 folds
     * the rounding and the shift in here, and the two must agree exactly or
     * every reconstructed sample is wrong. */
    for (i = 0; i < 16; i++) {
        int r = i >> 2, cc = i & 3;
        dst[r * dst_stride + cc] =
            (u8)clip_u8(pred[r * pred_stride + cc] + ((res[i] + 32) >> 6));
    }
}

/* ------------------------------------------------------- macroblock ------ */

/* The macroblock header of spec 7.3.5, for the I macroblocks this decoder
 * handles. Split out of decode_mb so the VHDL mb_header_dec_engine can be
 * held to the routine that reconstructs real streams byte-exactly, rather
 * than to a second implementation written from the same spec paragraph and
 * free to misread it the same way.
 *
 * Neighbour modes come in rather than being looked up, because that is the
 * shape the hardware has: a line buffer hands over the row above and the
 * column to the left, and everything inside the macroblock comes from blocks
 * this routine has already decoded. Unavailable neighbours should be passed
 * as DC (2), though avail_top / avail_left decide the outcome anyway. */
int dec_mb_header(bitreader_t *br, mb_header_t *h)
{
    int mb_type, s, i;

    h->mode16 = 0;
    h->cbp_luma = 0;
    h->cbp_chroma = 0;
    for (i = 0; i < 16; i++) h->modes4[i] = 2;

    mb_type = (int)br_get_ue(br);
    if (mb_type == 0) {
        h->is_i4x4 = 1;
    } else if (mb_type >= 1 && mb_type <= 24) {
        h->is_i4x4 = 0;
        /* I_16x16: mb_type - 1 packs mode, cbp_chroma and cbp_luma. */
        h->mode16     = (mb_type - 1) % 4;
        h->cbp_chroma = ((mb_type - 1) / 4) % 3;
        h->cbp_luma   = ((mb_type - 1) / 12) ? 15 : 0;
    } else {
        FAIL("mb_type %d; this decoder handles I macroblocks only", mb_type);
    }

    if (h->is_i4x4) {
        for (s = 0; s < 16; s++) {
            int bcr = scan_br[s], bcc = scan_bc[s];
            int a_up   = (bcr > 0) || h->avail_top;
            int a_left = (bcc > 0) || h->avail_left;
            int up_mode, left_mode, pred_mode, flag;

            /* predIntra4x4PredMode, spec 8.3.1.1: an unavailable or
             * non-I_4x4 neighbour contributes DC (2). */
            up_mode   = (bcr > 0) ? h->modes4[(bcr - 1) * 4 + bcc]
                                  : h->mode4_top[bcc];
            left_mode = (bcc > 0) ? h->modes4[bcr * 4 + bcc - 1]
                                  : h->mode4_left[bcr];
            pred_mode = up_mode < left_mode ? up_mode : left_mode;
            if (!a_up || !a_left) pred_mode = 2;

            flag = (int)br_get_bits(br, 1);
            if (flag) {
                h->modes4[bcr * 4 + bcc] = pred_mode;
            } else {
                int rem = (int)br_get_bits(br, 3);
                h->modes4[bcr * 4 + bcc] = rem < pred_mode ? rem : rem + 1;
            }
        }
    }

    h->mode_chroma = (int)br_get_ue(br);
    if (h->mode_chroma > 3)
        FAIL("intra_chroma_pred_mode %d", h->mode_chroma);

    if (h->is_i4x4) {
        int codenum = (int)br_get_ue(br);
        int cbp = -1;
        if (codenum < 0 || codenum >= 48)
            FAIL("coded_block_pattern codeNum %d", codenum);
        /* Invert the encoder's table rather than carry a second one. */
        for (i = 0; i < 48; i++)
            if (cbp_intra_to_codenum[i] == codenum) { cbp = i; break; }
        if (cbp < 0) FAIL("no CBP for codeNum %d", codenum);
        h->cbp_luma   = cbp & 0xF;
        h->cbp_chroma = (cbp >> 4) & 3;
    }

    h->has_residual = h->is_i4x4 ? (h->cbp_luma || h->cbp_chroma) : 1;
    h->qp_out = h->qp_in;
    if (h->has_residual) {
        int delta = (int)br_get_se(br);
        h->qp_out = ((h->qp_in + delta + 52 + 2 * 26) % 52);
        if (h->qp_out < 0 || h->qp_out > 51)
            FAIL("QP %d out of range", h->qp_out);
    }
    if (br->overflow) FAIL("header ran past the end of the slice");
    return 0;
}

/* Every residual block of one macroblock, in bitstream order, and the
 * neighbour total_coeff context the nC derivation needs. Split out of
 * decode_mb for the same reason as dec_mb_header: so the VHDL block
 * sequencer can be held to the routine the golden decoder itself uses.
 *
 * The split also matches the shape of the hardware. Parsing reads bits and
 * nothing else; reconstruction reads neighbouring samples and writes the
 * frame. Interleaving them, as this function's caller used to, hid the fact
 * that the entropy layer never depends on a reconstructed sample.
 *
 * Neighbour counts come in and go out rather than being looked up, because a
 * line buffer is what the hardware has: the row above and the column to the
 * left, with everything inside the macroblock coming from blocks this
 * routine has already decoded.
 *
 * Coefficients come out in zigzag order with the I_16x16 and chroma AC shift
 * already applied, so every block is a full 16-coefficient vector whatever
 * its type. */
int dec_mb_residual(bitreader_t *br, mb_residual_t *r)
{
    int s, i, k, comp;
    i16 zz[16];

    memset(r->luma_dc, 0, sizeof r->luma_dc);
    memset(r->luma, 0, sizeof r->luma);
    memset(r->chroma_dc, 0, sizeof r->chroma_dc);
    memset(r->chroma_ac, 0, sizeof r->chroma_ac);
    memset(r->nc_out, 0, sizeof r->nc_out);
    memset(r->ncu_out, 0, sizeof r->ncu_out);
    memset(r->ncv_out, 0, sizeof r->ncv_out);

    /* Luma DC: 16 Hadamard coefficients, present for every I_16x16
     * macroblock whatever the coded_block_pattern says. nC comes from the
     * neighbours of block 0. */
    if (!r->is_i4x4) {
        int nA = r->avail_left ? r->nc_left[0] : 0;
        int nB = r->avail_top  ? r->nc_top[0]  : 0;
        int nC = cavlc_compute_nC(nB, nA, r->avail_top, r->avail_left);
        if (cavlc_decode_block(br, r->luma_dc, 16, BLK_LUMA_DC_16x16, nC) != 0)
            FAIL("luma DC block malformed");
    }

    for (s = 0; s < 16; s++) {
        int bcr = scan_br[s], bcc = scan_bc[s];
        int pos = bcr * 4 + bcc;
        int coded = (r->cbp_luma >> (s / 4)) & 1;
        int n_coefs = r->is_i4x4 ? 16 : 15;
        int total = 0;

        memset(zz, 0, sizeof zz);
        if (coded) {
            int a_left = (bcc > 0) || r->avail_left;
            int a_top  = (bcr > 0) || r->avail_top;
            int nA = a_left ? ((bcc > 0) ? r->nc_out[pos - 1] : r->nc_left[bcr]) : 0;
            int nB = a_top  ? ((bcr > 0) ? r->nc_out[pos - 4] : r->nc_top[bcc])  : 0;
            int nC = cavlc_compute_nC(nB, nA, a_top, a_left);
            if (cavlc_decode_block(br, zz, n_coefs,
                                   r->is_i4x4 ? BLK_LUMA_FULL : BLK_LUMA_AC, nC) != 0)
                FAIL("luma block %d malformed", s);
            for (i = 0; i < n_coefs; i++) if (zz[i]) total++;
        }
        /* An uncoded block still contributes 0 to its neighbours' nC. */
        r->nc_out[pos] = total;

        /* I_16x16 AC blocks carry 15 coefficients starting at index 1. */
        if (!r->is_i4x4) {
            for (i = 15; i >= 1; i--) zz[i] = zz[i - 1];
            zz[0] = 0;
        }
        memcpy(r->luma[pos], zz, sizeof zz);
    }

    /* Chroma DC: two 2x2 Hadamard blocks, present when cbp_chroma != 0.
     * Their total_coeff feeds no neighbour: chroma DC has no nC. */
    if (r->cbp_chroma) {
        for (comp = 0; comp < 2; comp++)
            if (cavlc_decode_block(br, r->chroma_dc[comp], 4, BLK_CHROMA_DC, -1) != 0)
                FAIL("chroma DC block malformed");
    }

    for (comp = 0; comp < 2; comp++) {
        const int *ntop  = comp ? r->ncv_top  : r->ncu_top;
        const int *nleft = comp ? r->ncv_left : r->ncu_left;
        int *nout = comp ? r->ncv_out : r->ncu_out;
        for (i = 0; i < 4; i++) {
            int bcr = i >> 1, bcc = i & 1;
            int total = 0;

            memset(zz, 0, sizeof zz);
            if (r->cbp_chroma == 2) {
                int a_left = (bcc > 0) || r->avail_left;
                int a_top  = (bcr > 0) || r->avail_top;
                int nA = a_left ? ((bcc > 0) ? nout[i - 1] : nleft[bcr]) : 0;
                int nB = a_top  ? ((bcr > 0) ? nout[i - 2] : ntop[bcc])  : 0;
                int nC = cavlc_compute_nC(nB, nA, a_top, a_left);
                if (cavlc_decode_block(br, zz, 15, BLK_CHROMA_AC, nC) != 0)
                    FAIL("chroma AC block %d malformed", comp * 4 + i);
                for (k = 0; k < 15; k++) if (zz[k]) total++;
            }
            nout[i] = total;

            for (k = 15; k >= 1; k--) zz[k] = zz[k - 1];
            zz[0] = 0;
            memcpy(r->chroma_ac[comp][i], zz, sizeof zz);
        }
    }

    if (br->overflow) FAIL("residual ran past the end of the slice");
    return 0;
}

static int decode_mb(dec_ctx_t *c, bitreader_t *br, int mb_r, int mb_c)
{
    int s, i, k;
    int is_i4x4, mode16, mode_chroma, cbp_luma, cbp_chroma;
    int modes4[16];                      /* raster inside the macroblock */
    int x_mb = mb_c * 16, y_mb = mb_r * 16;
    int x4b = mb_c * 4, y4b = mb_r * 4;
    int x2b = mb_c * 2, y2b = mb_r * 2;
    int qp_c;
    mb_residual_t r;

    /* ---- header ---- */
    {
        mb_header_t h;
        h.qp_in      = c->qp;
        h.avail_top  = (mb_r > 0);
        h.avail_left = (mb_c > 0);
        for (i = 0; i < 4; i++) {
            h.mode4_top[i]  = h.avail_top
                            ? c->luma_mode4[(y4b - 1) * c->luma_w4 + x4b + i] : 2;
            h.mode4_left[i] = h.avail_left
                            ? c->luma_mode4[(y4b + i) * c->luma_w4 + x4b - 1] : 2;
        }
        if (dec_mb_header(br, &h) != 0)
            FAIL("MB(%d,%d): malformed macroblock header", mb_r, mb_c);

        is_i4x4     = h.is_i4x4;
        mode16      = h.mode16;
        mode_chroma = h.mode_chroma;
        cbp_luma    = h.cbp_luma;
        cbp_chroma  = h.cbp_chroma;
        c->qp       = h.qp_out;
        for (i = 0; i < 16; i++) modes4[i] = h.modes4[i];

        /* An I_16x16 macroblock predicts as DC for its neighbours' purposes. */
        for (i = 0; i < 16; i++)
            c->luma_mode4[(y4b + i / 4) * c->luma_w4 + x4b + (i % 4)] =
                (u8)(is_i4x4 ? modes4[i] : 2);
        c->mb_is_i4x4[mb_r * c->mbs_w + mb_c] = (u8)is_i4x4;
    }
    qp_c = chroma_qp(c->qp, c->chroma_qp_offset);

    /* ---- residual ---- */
    r.is_i4x4    = is_i4x4;
    r.cbp_luma   = cbp_luma;
    r.cbp_chroma = cbp_chroma;
    r.avail_top  = (mb_r > 0);
    r.avail_left = (mb_c > 0);
    for (i = 0; i < 4; i++) {
        r.nc_top[i]  = r.avail_top
                     ? c->luma_nc[(y4b - 1) * c->luma_w4 + x4b + i] : 0;
        r.nc_left[i] = r.avail_left
                     ? c->luma_nc[(y4b + i) * c->luma_w4 + x4b - 1] : 0;
    }
    for (i = 0; i < 2; i++) {
        r.ncu_top[i]  = r.avail_top
                      ? c->chroma_u_nc[(y2b - 1) * c->chroma_w4 + x2b + i] : 0;
        r.ncu_left[i] = r.avail_left
                      ? c->chroma_u_nc[(y2b + i) * c->chroma_w4 + x2b - 1] : 0;
        r.ncv_top[i]  = r.avail_top
                      ? c->chroma_v_nc[(y2b - 1) * c->chroma_w4 + x2b + i] : 0;
        r.ncv_left[i] = r.avail_left
                      ? c->chroma_v_nc[(y2b + i) * c->chroma_w4 + x2b - 1] : 0;
    }
    if (dec_mb_residual(br, &r) != 0)
        FAIL("MB(%d,%d): malformed residual", mb_r, mb_c);

    for (i = 0; i < 16; i++)
        c->luma_nc[(y4b + i / 4) * c->luma_w4 + x4b + (i % 4)] = (u8)r.nc_out[i];
    for (i = 0; i < 4; i++) {
        c->chroma_u_nc[(y2b + i / 2) * c->chroma_w4 + x2b + (i % 2)] = (u8)r.ncu_out[i];
        c->chroma_v_nc[(y2b + i / 2) * c->chroma_w4 + x2b + (i % 2)] = (u8)r.ncv_out[i];
    }

    /* ---- luma reconstruction ---- */
    {
        i32 dc_coef[16];

        memset(dc_coef, 0, sizeof dc_coef);
        if (!is_i4x4) {
            i16 dc_lev[16];
            i32 tmp[16];
            memset(dc_lev, 0, sizeof dc_lev);
            for (i = 0; i < 16; i++) dc_lev[zigzag4[i]] = r.luma_dc[i];
            iquant_dc_4x4(dc_lev, tmp, c->qp);
            ihadamard4x4(tmp, dc_coef);
        }

        if (is_i4x4) {
            /* Block by block, in scan order: block s+1 predicts from block
             * s's reconstruction. */
            for (s = 0; s < 16; s++) {
                int bcr = scan_br[s], bcc = scan_bc[s];
                int x0 = x_mb + bcc * 4, y0 = y_mb + bcr * 4;
                u8 top[8], left[4], tl, pred[16];
                int at, al, atl;
                int atr = tr_avail_4x4(s, mb_r, mb_c, c->mbs_w);
                gather_4x4(c, x0, y0, top, left, &tl, &at, &al, &atl, &atr);
                predict_4x4(modes4[bcr * 4 + bcc], top, left, tl, at, al, atl, pred);
                add_residual_4x4(r.luma[bcr * 4 + bcc], c->qp, 0, 0, pred, 4,
                                 &c->recon_y[y0 * c->width + x0], c->width);
            }
        } else {
            u8 top16[16], left16[16], tl16, pred16[256];
            int at = (y_mb > 0), al = (x_mb > 0), atl = (x_mb > 0 && y_mb > 0);
            for (i = 0; i < 16; i++)
                top16[i] = at ? c->recon_y[(y_mb - 1) * c->width + x_mb + i] : 0;
            for (i = 0; i < 16; i++)
                left16[i] = al ? c->recon_y[(y_mb + i) * c->width + x_mb - 1] : 0;
            tl16 = atl ? c->recon_y[(y_mb - 1) * c->width + x_mb - 1] : 0;
            predict_16x16(mode16, top16, left16, tl16, at, al, atl, pred16);

            for (k = 0; k < 16; k++) {
                int kbr = k / 4, kbc = k % 4;
                int kx = x_mb + kbc * 4, ky = y_mb + kbr * 4;
                add_residual_4x4(r.luma[k], c->qp, 1, dc_coef[k],
                                 &pred16[(kbr * 4) * 16 + kbc * 4], 16,
                                 &c->recon_y[ky * c->width + kx], c->width);
            }
        }
    }

    /* ---- chroma reconstruction ---- */
    {
        u8 pred_u[64], pred_v[64];
        i32 dcu[4], dcv[4];
        int comp;
        u8 top[8], left[8], tl;
        int at = (y_mb > 0), al = (x_mb > 0), atl = (x_mb > 0 && y_mb > 0);
        int xc = x_mb / 2, yc = y_mb / 2;          /* chroma sample origin */
        int cw = c->width;                          /* NV12: 2 bytes per chroma x */

        memset(dcu, 0, sizeof dcu);
        memset(dcv, 0, sizeof dcv);

        for (comp = 0; comp < 2; comp++) {
            u8 *pred = comp ? pred_v : pred_u;
            for (i = 0; i < 8; i++)
                top[i] = at ? c->recon_uv[(yc - 1) * cw + (xc + i) * 2 + comp] : 0;
            for (i = 0; i < 8; i++)
                left[i] = al ? c->recon_uv[(yc + i) * cw + (xc - 1) * 2 + comp] : 0;
            tl = atl ? c->recon_uv[(yc - 1) * cw + (xc - 1) * 2 + comp] : 0;
            predict_chroma_8x8(mode_chroma, top, left, tl, at, al, atl, pred);
        }

        if (cbp_chroma) {
            for (comp = 0; comp < 2; comp++) {
                i16 dl[4];
                i32 out[4];
                for (i = 0; i < 4; i++) dl[i] = r.chroma_dc[comp][i];
                iquant_dc_2x2(dl, out, qp_c);
                ihadamard2x2(out, comp ? dcv : dcu);
            }
        }

        for (comp = 0; comp < 2; comp++) {
            u8 *pred = comp ? pred_v : pred_u;
            i32 *dc = comp ? dcv : dcu;
            for (i = 0; i < 4; i++) {
                int bcr = i >> 1, bcc = i & 1;
                u8 tmp[16];
                int r2, c2;
                add_residual_4x4(r.chroma_ac[comp][i], qp_c, 1, dc[i],
                                 &pred[(bcr * 4) * 8 + bcc * 4], 8, tmp, 4);
                for (r2 = 0; r2 < 4; r2++)
                    for (c2 = 0; c2 < 4; c2++)
                        c->recon_uv[(yc + bcr * 4 + r2) * cw
                                    + (xc + bcc * 4 + c2) * 2 + comp] =
                            tmp[r2 * 4 + c2];
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------ stream ----- */

int dcc_decode_stream(const u8 *data, int len,
                      void (*on_frame)(const u8 *, const u8 *, int, int, void *),
                      void *ctx, dec_stats_t *stats)
{
    dec_sps_t sps;
    dec_pps_t pps;
    dec_slice_t sh;
    dec_nal_t nal;
    dec_ctx_t c;
    bitreader_t br;
    u8 *rbsp = NULL;
    int pos = 0, have_sps = 0, have_pps = 0, rc = 0, frames = 0;
    int rbsp_cap = len + 64;

    memset(&c, 0, sizeof c);
    memset(stats, 0, sizeof *stats);

    rbsp = malloc((size_t)rbsp_cap);
    if (!rbsp) FAIL("out of memory for the RBSP buffer");

    while ((rc = dec_next_nal(data, len, &pos, rbsp, rbsp_cap, &nal)) == 1) {
        if (nal.nal_unit_type == 7) {
            if (dec_parse_sps(&nal, &sps) != 0) { rc = -1; goto done; }
            if (sps.width > DEC_MAX_W || sps.height > DEC_MAX_H) {
                fprintf(stderr, "decode: %dx%d exceeds the %dx%d build limit\n",
                        sps.width, sps.height, DEC_MAX_W, DEC_MAX_H);
                rc = -1; goto done;
            }
            if (have_sps && (sps.width != c.width || sps.height != c.height)) {
                fprintf(stderr, "decode: resolution changed mid-stream\n");
                rc = -1; goto done;
            }
            if (!have_sps) {
                c.width  = sps.width;   c.height = sps.height;
                c.mbs_w  = sps.mbs_w;   c.mbs_h  = sps.mbs_h;
                c.luma_w4   = c.mbs_w * 4;
                c.chroma_w4 = c.mbs_w * 2;
                c.recon_y     = calloc((size_t)c.width * c.height, 1);
                c.recon_uv    = calloc((size_t)c.width * (c.height / 2), 1);
                c.luma_nc     = calloc((size_t)c.luma_w4 * c.mbs_h * 4, 1);
                c.chroma_u_nc = calloc((size_t)c.chroma_w4 * c.mbs_h * 2, 1);
                c.chroma_v_nc = calloc((size_t)c.chroma_w4 * c.mbs_h * 2, 1);
                c.luma_mode4  = calloc((size_t)c.luma_w4 * c.mbs_h * 4, 1);
                c.mb_is_i4x4  = calloc((size_t)c.mbs_w * c.mbs_h, 1);
                if (!c.recon_y || !c.recon_uv || !c.luma_nc || !c.chroma_u_nc
                    || !c.chroma_v_nc || !c.luma_mode4 || !c.mb_is_i4x4) {
                    fprintf(stderr, "decode: out of memory for %dx%d\n",
                            c.width, c.height);
                    rc = -1; goto done;
                }
            }
            have_sps = 1;
            continue;
        }
        if (nal.nal_unit_type == 8) {
            if (dec_parse_pps(&nal, &pps) != 0) { rc = -1; goto done; }
            have_pps = 1;
            continue;
        }
        if (nal.nal_unit_type == 9) continue;          /* access unit delimiter */
        if (nal.nal_unit_type != 1 && nal.nal_unit_type != 5) continue;

        if (!have_sps || !have_pps) {
            fprintf(stderr, "decode: slice before SPS/PPS\n");
            rc = -1; goto done;
        }
        if (dec_parse_slice_header(&br, &nal, &sps, &pps, &sh) != 0) {
            rc = -1; goto done;
        }

        /* Every picture here is intra, so nothing carries over between them
         * except the buffers themselves. Clearing the contexts makes a lost
         * picture affect only itself. */
        memset(c.luma_nc,     0, (size_t)c.luma_w4 * c.mbs_h * 4);
        memset(c.chroma_u_nc, 0, (size_t)c.chroma_w4 * c.mbs_h * 2);
        memset(c.chroma_v_nc, 0, (size_t)c.chroma_w4 * c.mbs_h * 2);
        memset(c.luma_mode4,  0, (size_t)c.luma_w4 * c.mbs_h * 4);
        memset(c.recon_y,  128, (size_t)c.width * c.height);
        memset(c.recon_uv, 128, (size_t)c.width * (c.height / 2));

        c.qp = sh.slice_qp;
        c.chroma_qp_offset = pps.chroma_qp_index_offset;

        {
            int mb_r, mb_c;
            for (mb_r = 0; mb_r < c.mbs_h; mb_r++)
                for (mb_c = 0; mb_c < c.mbs_w; mb_c++)
                    if (decode_mb(&c, &br, mb_r, mb_c) != 0) { rc = -1; goto done; }
        }
        if (br.overflow) {
            fprintf(stderr, "decode: slice data ended early in frame %d\n", frames);
            rc = -1; goto done;
        }

        frames++;
        stats->slice_qp = sh.slice_qp;
        if (on_frame) on_frame(c.recon_y, c.recon_uv, c.width, c.height, ctx);
    }
    if (rc < 0) goto done;
    rc = 0;

done:
    stats->width  = c.width;   stats->height = c.height;
    stats->mbs_w  = c.mbs_w;   stats->mbs_h  = c.mbs_h;
    stats->frames = frames;
    stats->bytes_in = len;
    free(rbsp);
    free(c.recon_y); free(c.recon_uv);
    free(c.luma_nc); free(c.chroma_u_nc); free(c.chroma_v_nc);
    free(c.luma_mode4); free(c.mb_is_i4x4);
    return rc;
}
