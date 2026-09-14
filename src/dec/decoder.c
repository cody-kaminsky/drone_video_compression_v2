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

static int decode_mb(dec_ctx_t *c, bitreader_t *br, int mb_r, int mb_c)
{
    int mb_type, s, i, k;
    int is_i4x4, mode16 = 0, mode_chroma, cbp_luma, cbp_chroma;
    int modes4[16];                      /* raster inside the macroblock */
    int x_mb = mb_c * 16, y_mb = mb_r * 16;
    i16 zz[16];
    int qp_c;

    mb_type = (int)br_get_ue(br);
    if (mb_type == 0) {
        is_i4x4 = 1;
    } else if (mb_type >= 1 && mb_type <= 24) {
        is_i4x4 = 0;
        /* I_16x16: mb_type - 1 packs mode, cbp_chroma and cbp_luma. */
        mode16     = (mb_type - 1) % 4;
        cbp_chroma = ((mb_type - 1) / 4) % 3;
        cbp_luma   = ((mb_type - 1) / 12) ? 15 : 0;
    } else {
        FAIL("MB(%d,%d): mb_type %d; this decoder handles I macroblocks only",
             mb_r, mb_c, mb_type);
    }

    /* ---- intra modes ---- */
    if (is_i4x4) {
        for (s = 0; s < 16; s++) {
            int bcr = scan_br[s], bcc = scan_bc[s];
            int x4 = mb_c * 4 + bcc, y4 = mb_r * 4 + bcr;
            int a_up   = (y4 > 0);
            int a_left = (x4 > 0);
            int up_mode, left_mode, pred_mode, flag;

            /* predIntra4x4PredMode, spec 8.3.1.1: an unavailable or
             * non-I_4x4 neighbour contributes DC (2). */
            up_mode   = a_up   ? c->luma_mode4[(y4 - 1) * c->luma_w4 + x4] : 2;
            left_mode = a_left ? c->luma_mode4[y4 * c->luma_w4 + x4 - 1] : 2;
            pred_mode = up_mode < left_mode ? up_mode : left_mode;
            if (!a_up || !a_left) pred_mode = 2;

            flag = (int)br_get_bits(br, 1);
            if (flag) {
                modes4[bcr * 4 + bcc] = pred_mode;
            } else {
                int rem = (int)br_get_bits(br, 3);
                modes4[bcr * 4 + bcc] = rem < pred_mode ? rem : rem + 1;
            }
            c->luma_mode4[y4 * c->luma_w4 + x4] = (u8)modes4[bcr * 4 + bcc];
        }
    } else {
        /* An I_16x16 macroblock predicts as DC for its neighbours' purposes. */
        for (i = 0; i < 16; i++) {
            int x4 = mb_c * 4 + (i & 3), y4 = mb_r * 4 + (i >> 2);
            c->luma_mode4[y4 * c->luma_w4 + x4] = 2;
        }
    }
    c->mb_is_i4x4[mb_r * c->mbs_w + mb_c] = (u8)is_i4x4;

    mode_chroma = (int)br_get_ue(br);
    if (mode_chroma > 3)
        FAIL("MB(%d,%d): intra_chroma_pred_mode %d", mb_r, mb_c, mode_chroma);

    if (is_i4x4) {
        int codenum = (int)br_get_ue(br);
        int cbp;
        if (codenum < 0 || codenum >= 48)
            FAIL("MB(%d,%d): coded_block_pattern codeNum %d", mb_r, mb_c, codenum);
        /* Invert the encoder's table rather than carry a second one. */
        cbp = -1;
        for (i = 0; i < 48; i++)
            if (cbp_intra_to_codenum[i] == codenum) { cbp = i; break; }
        if (cbp < 0) FAIL("MB(%d,%d): no CBP for codeNum %d", mb_r, mb_c, codenum);
        cbp_luma   = cbp & 0xF;
        cbp_chroma = (cbp >> 4) & 3;
    }

    /* ---- QP ---- */
    {
        int has_residual = is_i4x4 ? (cbp_luma || cbp_chroma) : 1;
        if (has_residual) {
            int delta = (int)br_get_se(br);
            c->qp = ((c->qp + delta + 52 + 2 * 26) % 52);
            if (c->qp < 0 || c->qp > 51)
                FAIL("MB(%d,%d): QP %d out of range", mb_r, mb_c, c->qp);
        }
    }
    qp_c = chroma_qp(c->qp, c->chroma_qp_offset);

    /* ---- luma ---- */
    {
        i32 dc_coef[16];
        i16 dc_lev[16];
        int have_dc = 0;

        memset(dc_coef, 0, sizeof dc_coef);

        if (!is_i4x4) {
            /* Luma DC: 16 Hadamard coefficients, always present. nC uses the
             * block-0 neighbours. */
            int x4 = mb_c * 4, y4 = mb_r * 4;
            int nA = (x4 > 0) ? c->luma_nc[y4 * c->luma_w4 + x4 - 1] : 0;
            int nB = (y4 > 0) ? c->luma_nc[(y4 - 1) * c->luma_w4 + x4] : 0;
            int nC = cavlc_compute_nC(nB, nA, y4 > 0, x4 > 0);
            i32 tmp[16];
            if (cavlc_decode_block(br, zz, 16, BLK_LUMA_DC_16x16, nC) != 0)
                FAIL("MB(%d,%d): luma DC block malformed", mb_r, mb_c);
            memset(dc_lev, 0, sizeof dc_lev);
            for (i = 0; i < 16; i++) dc_lev[zigzag4[i]] = zz[i];
            iquant_dc_4x4(dc_lev, tmp, c->qp);
            ihadamard4x4(tmp, dc_coef);
            have_dc = 1;
        }

        for (s = 0; s < 16; s++) {
            int bcr = scan_br[s], bcc = scan_bc[s];
            int x4 = mb_c * 4 + bcc, y4 = mb_r * 4 + bcr;
            int x0 = x_mb + bcc * 4, y0 = y_mb + bcr * 4;
            int quad = s / 4;
            int coded = (cbp_luma >> quad) & 1;
            int n_coefs = is_i4x4 ? 16 : 15;
            int total = 0;
            u8 pred[16];

            memset(zz, 0, sizeof zz);
            if (coded) {
                int nA = (x4 > 0) ? c->luma_nc[y4 * c->luma_w4 + x4 - 1] : 0;
                int nB = (y4 > 0) ? c->luma_nc[(y4 - 1) * c->luma_w4 + x4] : 0;
                int nC = cavlc_compute_nC(nB, nA, y4 > 0, x4 > 0);
                if (cavlc_decode_block(br, zz, n_coefs,
                                       is_i4x4 ? BLK_LUMA_FULL : BLK_LUMA_AC, nC) != 0)
                    FAIL("MB(%d,%d) blk %d: luma block malformed", mb_r, mb_c, s);
                for (i = 0; i < n_coefs; i++) if (zz[i]) total++;
            }
            c->luma_nc[y4 * c->luma_w4 + x4] = (u8)total;

            /* I_16x16 AC blocks carry 15 coefficients starting at index 1. */
            if (!is_i4x4) {
                for (i = 15; i >= 1; i--) zz[i] = zz[i - 1];
                zz[0] = 0;
            }

            /* Prediction, then residual. For I_4x4 this must happen block by
             * block: block s+1 predicts from block s's reconstruction. */
            if (is_i4x4) {
                u8 top[8], left[4], tl;
                int at, al, atl;
                int atr = tr_avail_4x4(s, mb_r, mb_c, c->mbs_w);
                gather_4x4(c, x0, y0, top, left, &tl, &at, &al, &atl, &atr);
                predict_4x4(modes4[bcr * 4 + bcc], top, left, tl, at, al, atl, pred);
                add_residual_4x4(zz, c->qp, 0, 0, pred, 4,
                                 &c->recon_y[y0 * c->width + x0], c->width);
            } else {
                /* I_16x16: predict the whole macroblock once, below. Stash
                 * the residual by decoding it into the frame after the
                 * prediction pass, so keep the levels for now. */
                static i16 keep[16][16];
                memcpy(keep[s], zz, sizeof zz);
                if (s == 15) {
                    u8 top16[16], left16[16], tl16, pred16[256];
                    int at = (y_mb > 0), al = (x_mb > 0), atl = (x_mb > 0 && y_mb > 0);
                    for (i = 0; i < 16; i++)
                        top16[i] = at ? c->recon_y[(y_mb - 1) * c->width + x_mb + i] : 0;
                    for (i = 0; i < 16; i++)
                        left16[i] = al ? c->recon_y[(y_mb + i) * c->width + x_mb - 1] : 0;
                    tl16 = atl ? c->recon_y[(y_mb - 1) * c->width + x_mb - 1] : 0;
                    predict_16x16(mode16, top16, left16, tl16, at, al, atl, pred16);

                    for (k = 0; k < 16; k++) {
                        int kbr = scan_br[k], kbc = scan_bc[k];
                        int kx = x_mb + kbc * 4, ky = y_mb + kbr * 4;
                        add_residual_4x4(keep[k], c->qp, 1,
                                         have_dc ? dc_coef[kbr * 4 + kbc] : 0,
                                         &pred16[(kbr * 4) * 16 + kbc * 4], 16,
                                         &c->recon_y[ky * c->width + kx], c->width);
                    }
                }
            }
        }
    }

    /* ---- chroma ---- */
    {
        u8 pred_u[64], pred_v[64];
        i32 dcu[4], dcv[4];
        i16 dl[4];
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

        /* Chroma DC: two 2x2 Hadamard blocks, present when cbp_chroma != 0. */
        if (cbp_chroma) {
            for (comp = 0; comp < 2; comp++) {
                i16 z4[4];
                i32 out[4];
                if (cavlc_decode_block(br, z4, 4, BLK_CHROMA_DC, -1) != 0)
                    FAIL("MB(%d,%d): chroma DC malformed", mb_r, mb_c);
                for (i = 0; i < 4; i++) dl[i] = z4[i];
                iquant_dc_2x2(dl, out, qp_c);
                ihadamard2x2(out, comp ? dcv : dcu);
            }
        }

        for (comp = 0; comp < 2; comp++) {
            u8 *pred = comp ? pred_v : pred_u;
            u8 *ncbuf = comp ? c->chroma_v_nc : c->chroma_u_nc;
            i32 *dc = comp ? dcv : dcu;
            for (i = 0; i < 4; i++) {
                int bcr = i >> 1, bcc = i & 1;
                int x4 = mb_c * 2 + bcc, y4 = mb_r * 2 + bcr;
                int total = 0;
                u8 tmp[16];
                int r2, c2;

                memset(zz, 0, sizeof zz);
                if (cbp_chroma == 2) {
                    int nA = (x4 > 0) ? ncbuf[y4 * c->chroma_w4 + x4 - 1] : 0;
                    int nB = (y4 > 0) ? ncbuf[(y4 - 1) * c->chroma_w4 + x4] : 0;
                    int nC = cavlc_compute_nC(nB, nA, y4 > 0, x4 > 0);
                    if (cavlc_decode_block(br, zz, 15, BLK_CHROMA_AC, nC) != 0)
                        FAIL("MB(%d,%d): chroma AC malformed", mb_r, mb_c);
                    for (k = 0; k < 15; k++) if (zz[k]) total++;
                }
                ncbuf[y4 * c->chroma_w4 + x4] = (u8)total;

                for (k = 15; k >= 1; k--) zz[k] = zz[k - 1];
                zz[0] = 0;

                add_residual_4x4(zz, qp_c, 1, dc[i],
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
