/* inter.c — motion compensation, vector prediction and motion estimation
 * for 16x16 P-slice macroblocks with one reference picture. See inter.h. */

#include "inter.h"
#include "transform.h"
#include <string.h>

static int clip_u8(int x) { return x < 0 ? 0 : (x > 255 ? 255 : x); }
static int abs_i(int x)   { return x < 0 ? -x : x; }
static int min3(int a, int b, int c)
{
    int m = a < b ? a : b;
    return m < c ? m : c;
}
static int max3(int a, int b, int c)
{
    int m = a > b ? a : b;
    return m > c ? m : c;
}
static int median3(int a, int b, int c) { return a + b + c - min3(a, b, c) - max3(a, b, c); }

/* ------------------------------------------------------------------ */
/* padded reference                                                    */
/* ------------------------------------------------------------------ */
void ref_build(ref_planes_t *rp, u8 *pad_y, u8 *pad_u, u8 *pad_v,
               const u8 *recon_y, int stride_y,
               const u8 *recon_uv, int stride_uv, int width, int height)
{
    int w = width, h = height, cw = width / 2, ch = height / 2;
    rp->stride_y = w + 2 * REF_PAD;
    rp->stride_c = cw + 2 * REF_PAD_C;
    rp->y = pad_y + REF_PAD * rp->stride_y + REF_PAD;
    rp->u = pad_u + REF_PAD_C * rp->stride_c + REF_PAD_C;
    rp->v = pad_v + REF_PAD_C * rp->stride_c + REF_PAD_C;
    rp->width = w; rp->height = h;

    /* luma: copy then replicate edges */
    for (int y = -REF_PAD; y < h + REF_PAD; y++) {
        int sy = y < 0 ? 0 : (y >= h ? h - 1 : y);
        u8 *row = rp->y + y * rp->stride_y;
        const u8 *src = recon_y + sy * stride_y;
        memcpy(row, src, (size_t)w);
        memset(row - REF_PAD, src[0], REF_PAD);
        memset(row + w, src[w - 1], REF_PAD);
    }
    /* chroma: de-interleave NV12 then replicate */
    for (int y = -REF_PAD_C; y < ch + REF_PAD_C; y++) {
        int sy = y < 0 ? 0 : (y >= ch ? ch - 1 : y);
        const u8 *src = recon_uv + sy * stride_uv;
        u8 *ru = rp->u + y * rp->stride_c;
        u8 *rv = rp->v + y * rp->stride_c;
        for (int x = 0; x < cw; x++) { ru[x] = src[2 * x]; rv[x] = src[2 * x + 1]; }
        memset(ru - REF_PAD_C, ru[0], REF_PAD_C);  memset(ru + cw, ru[cw - 1], REF_PAD_C);
        memset(rv - REF_PAD_C, rv[0], REF_PAD_C);  memset(rv + cw, rv[cw - 1], REF_PAD_C);
    }
}

/* ------------------------------------------------------------------ */
/* luma interpolation, spec 8.4.2.2.1                                  */
/* ------------------------------------------------------------------ */
/* 6-tap filter on integer samples, unscaled (b1 / h1 in the spec) */
static int tap6(int e, int f, int g, int h, int i, int j)
{
    return e - 5 * f + 20 * g + 20 * h - 5 * i + j;
}

/* half sample b (horizontal) at integer position (x, y), scaled */
static int half_h(const u8 *p, int stride, int x, int y)
{
    const u8 *r = p + y * stride;
    return clip_u8((tap6(r[x - 2], r[x - 1], r[x], r[x + 1], r[x + 2], r[x + 3]) + 16) >> 5);
}
/* half sample h (vertical), scaled */
static int half_v(const u8 *p, int stride, int x, int y)
{
    const u8 *c = p + x;
    return clip_u8((tap6(c[(y - 2) * stride], c[(y - 1) * stride], c[y * stride],
                         c[(y + 1) * stride], c[(y + 2) * stride], c[(y + 3) * stride]) + 16) >> 5);
}
/* unscaled vertical intermediate h1 at (x, y) */
static int half_v1(const u8 *p, int stride, int x, int y)
{
    const u8 *c = p + x;
    return tap6(c[(y - 2) * stride], c[(y - 1) * stride], c[y * stride],
                c[(y + 1) * stride], c[(y + 2) * stride], c[(y + 3) * stride]);
}
/* centre sample j at (x, y): 6-tap over the unscaled vertical intermediates */
static int half_c(const u8 *p, int stride, int x, int y)
{
    int j1 = tap6(half_v1(p, stride, x - 2, y), half_v1(p, stride, x - 1, y), half_v1(p, stride, x, y),
                  half_v1(p, stride, x + 1, y), half_v1(p, stride, x + 2, y), half_v1(p, stride, x + 3, y));
    return clip_u8((j1 + 512) >> 10);
}

/* one predicted luma sample for fractional position (xf, yf) at integer
 * base (x, y); Table 8-12 */
static int luma_sample(const u8 *p, int stride, int x, int y, int xf, int yf)
{
    switch (yf * 4 + xf) {
        case 0:  return p[y * stride + x];                                          /* G */
        case 1:  return (p[y * stride + x] + half_h(p, stride, x, y) + 1) >> 1;      /* a */
        case 2:  return half_h(p, stride, x, y);                                    /* b */
        case 3:  return (p[y * stride + x + 1] + half_h(p, stride, x, y) + 1) >> 1;  /* c */
        case 4:  return (p[y * stride + x] + half_v(p, stride, x, y) + 1) >> 1;      /* d */
        case 5:  return (half_h(p, stride, x, y) + half_v(p, stride, x, y) + 1) >> 1;          /* e */
        case 6:  return (half_h(p, stride, x, y) + half_c(p, stride, x, y) + 1) >> 1;          /* f */
        case 7:  return (half_h(p, stride, x, y) + half_v(p, stride, x + 1, y) + 1) >> 1;      /* g */
        case 8:  return half_v(p, stride, x, y);                                    /* h */
        case 9:  return (half_v(p, stride, x, y) + half_c(p, stride, x, y) + 1) >> 1;          /* i */
        case 10: return half_c(p, stride, x, y);                                    /* j */
        case 11: return (half_c(p, stride, x, y) + half_v(p, stride, x + 1, y) + 1) >> 1;      /* k */
        case 12: return (p[(y + 1) * stride + x] + half_v(p, stride, x, y) + 1) >> 1; /* n */
        case 13: return (half_v(p, stride, x, y) + half_h(p, stride, x, y + 1) + 1) >> 1;      /* p */
        case 14: return (half_c(p, stride, x, y) + half_h(p, stride, x, y + 1) + 1) >> 1;      /* q */
        default: return (half_v(p, stride, x + 1, y) + half_h(p, stride, x, y + 1) + 1) >> 1;  /* r */
    }
}

void mc_luma_16x16(const ref_planes_t *rp, int mb_r, int mb_c,
                   int mvx, int mvy, u8 pred[256])
{
    int xf = mvx & 3, yf = mvy & 3;
    int x0 = mb_c * 16 + (mvx >> 2), y0 = mb_r * 16 + (mvy >> 2);
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++)
            pred[r * 16 + c] = (u8)luma_sample(rp->y, rp->stride_y, x0 + c, y0 + r, xf, yf);
}

/* ------------------------------------------------------------------ */
/* chroma interpolation, spec 8.4.2.2.2                                */
/* ------------------------------------------------------------------ */
static void mc_chroma_plane(const u8 *p, int stride, int x0, int y0, int xf, int yf, u8 pred[64])
{
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++) {
            const u8 *a = p + (y0 + r) * stride + (x0 + c);
            int v = (8 - xf) * (8 - yf) * a[0] + xf * (8 - yf) * a[1] +
                    (8 - xf) * yf * a[stride] + xf * yf * a[stride + 1];
            pred[r * 8 + c] = (u8)((v + 32) >> 6);
        }
}

void mc_chroma_8x8(const ref_planes_t *rp, int mb_r, int mb_c,
                   int mvx, int mvy, u8 pred_u[64], u8 pred_v[64])
{
    int xf = mvx & 7, yf = mvy & 7;
    int x0 = mb_c * 8 + (mvx >> 3), y0 = mb_r * 8 + (mvy >> 3);
    mc_chroma_plane(rp->u, rp->stride_c, x0, y0, xf, yf, pred_u);
    mc_chroma_plane(rp->v, rp->stride_c, x0, y0, xf, yf, pred_v);
}

/* ------------------------------------------------------------------ */
/* vector prediction, spec 8.4.1.3 (16x16, refIdx 0 only)              */
/* ------------------------------------------------------------------ */
/* neighbour N: avail = MB exists; ref = 0 if inter, -1 if intra / absent */
static void neighbour(const mv_field_t *mf, int r, int c, int *avail, int *ref, int *x, int *y)
{
    if (r < 0 || c < 0 || c >= mf->mbs_w) { *avail = 0; *ref = -1; *x = 0; *y = 0; return; }
    int i = r * mf->mbs_w + c;
    *avail = 1;
    if (mf->is_intra[i]) { *ref = -1; *x = 0; *y = 0; }
    else { *ref = 0; *x = mf->mvx[i]; *y = mf->mvy[i]; }
}

void mv_predict_16x16(const mv_field_t *mf, int mb_r, int mb_c,
                      int *pred_x, int *pred_y)
{
    int aa, ar, ax, ay, ba, br, bx, by, ca, cr, cx, cy;
    neighbour(mf, mb_r, mb_c - 1, &aa, &ar, &ax, &ay);          /* A: left */
    neighbour(mf, mb_r - 1, mb_c, &ba, &br, &bx, &by);          /* B: above */
    neighbour(mf, mb_r - 1, mb_c + 1, &ca, &cr, &cx, &cy);      /* C: above right */
    if (!ca) neighbour(mf, mb_r - 1, mb_c - 1, &ca, &cr, &cx, &cy);   /* D: above left */
    /* 8.4.1.3.1: B and C both unavailable, A available -> use A for all */
    if (!ba && !ca && aa) { bx = ax; by = ay; br = ar; cx = ax; cy = ay; cr = ar; }
    int na = (ar == 0), nb = (br == 0), nc = (cr == 0);
    if (na + nb + nc == 1) {
        if (na) { *pred_x = ax; *pred_y = ay; }
        else if (nb) { *pred_x = bx; *pred_y = by; }
        else { *pred_x = cx; *pred_y = cy; }
    } else {
        *pred_x = median3(ax, bx, cx);
        *pred_y = median3(ay, by, cy);
    }
}

void mv_skip_16x16(const mv_field_t *mf, int mb_r, int mb_c, int *mvx, int *mvy)
{
    int aa, ar, ax, ay, ba, br, bx, by;
    neighbour(mf, mb_r, mb_c - 1, &aa, &ar, &ax, &ay);
    neighbour(mf, mb_r - 1, mb_c, &ba, &br, &bx, &by);
    if (!aa || !ba || (ar == 0 && ax == 0 && ay == 0) || (br == 0 && bx == 0 && by == 0)) {
        *mvx = 0; *mvy = 0;
    } else {
        mv_predict_16x16(mf, mb_r, mb_c, mvx, mvy);
    }
}

int mvd_bits(int d)
{
    unsigned k = d > 0 ? (unsigned)(2 * d - 1) : (unsigned)(-2 * d);   /* se -> codeNum */
    int n = 0;
    unsigned v = k + 1;
    while (v > 1) { v >>= 1; n++; }
    return 2 * n + 1;
}

/* ------------------------------------------------------------------ */
/* motion estimation                                                   */
/* ------------------------------------------------------------------ */
static int sad_16x16_int(const ref_planes_t *rp, const u8 src[256], int x0, int y0)
{
    int s = 0;
    for (int r = 0; r < 16; r++) {
        const u8 *p = rp->y + (y0 + r) * rp->stride_y + x0;
        const u8 *q = src + r * 16;
        for (int c = 0; c < 16; c++) s += abs_i((int)q[c] - (int)p[c]);
    }
    return s;
}

static int satd_16x16_pred(const u8 src[256], const u8 pred[256])
{
    int total = 0;
    for (int br = 0; br < 4; br++)
        for (int bc = 0; bc < 4; bc++) {
            i16 d[16]; i32 h[16];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++) {
                    int i = (br * 4 + r) * 16 + bc * 4 + c;
                    d[r * 4 + c] = (i16)((int)src[i] - (int)pred[i]);
                }
            hadamard4x4(d, h);
            for (int k = 0; k < 16; k++) total += abs_i(h[k]);
        }
    return total;
}

/* keep the block (plus filter margin) inside the padded plane */
static int mv_ok(const ref_planes_t *rp, int mb_r, int mb_c, int mvx, int mvy)
{
    int x0 = mb_c * 16 + (mvx >> 2), y0 = mb_r * 16 + (mvy >> 2);
    int m = REF_PAD - 4;
    return x0 >= -m && x0 + 16 <= rp->width + m && y0 >= -m && y0 + 16 <= rp->height + m;
}

int me_search_16x16(const ref_planes_t *rp, const u8 src[256],
                    int mb_r, int mb_c, int pred_x, int pred_y,
                    const me_params_t *mp, int *best_x, int *best_y)
{
    /* integer full search around the predictor (rounded to integer) */
    int cx = (pred_x + 2) >> 2, cy = (pred_y + 2) >> 2;   /* centre in integer samples */
    int bx = 0, by = 0, best = 0x7fffffff;
    /* always evaluate the zero vector and the predictor first (ties keep them) */
    for (int dy = -mp->range; dy <= mp->range; dy++)
        for (int dx = -mp->range; dx <= mp->range; dx++) {
            int ix = cx + dx, iy = cy + dy;
            int mvx = ix * 4, mvy = iy * 4;
            if (!mv_ok(rp, mb_r, mb_c, mvx, mvy)) continue;
            int cost = sad_16x16_int(rp, src, mb_c * 16 + ix, mb_r * 16 + iy) +
                       mp->lambda * (mvd_bits(mvx - pred_x) + mvd_bits(mvy - pred_y));
            if (cost < best || (cost == best && abs_i(mvx) + abs_i(mvy) < abs_i(bx) + abs_i(by))) {
                best = cost; bx = mvx; by = mvy;
            }
        }
    /* half then quarter refinement on SATD (the integer winner is re-scored
     * with SATD so the comparison is consistent) */
    u8 pred[256];
    mc_luma_16x16(rp, mb_r, mb_c, bx, by, pred);
    best = satd_16x16_pred(src, pred) + mp->lambda * (mvd_bits(bx - pred_x) + mvd_bits(by - pred_y));
    for (int step = 2; step >= 1; step >>= 1) {
        int ox = bx, oy = by;
        for (int dy = -step; dy <= step; dy += step)
            for (int dx = -step; dx <= step; dx += step) {
                if (dx == 0 && dy == 0) continue;
                int mvx = ox + dx, mvy = oy + dy;
                if (!mv_ok(rp, mb_r, mb_c, mvx, mvy)) continue;
                mc_luma_16x16(rp, mb_r, mb_c, mvx, mvy, pred);
                int cost = satd_16x16_pred(src, pred) +
                           mp->lambda * (mvd_bits(mvx - pred_x) + mvd_bits(mvy - pred_y));
                if (cost < best) { best = cost; bx = mvx; by = mvy; }
            }
    }
    *best_x = bx; *best_y = by;
    return best;
}
