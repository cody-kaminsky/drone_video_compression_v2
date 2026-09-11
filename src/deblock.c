/* deblock.c — H.264 deblocking filter, spec 8.7. See deblock.h. */

#include "deblock.h"

/* Table 8-16: alpha' and beta' by indexA / indexB (0..51) */
static const u8 ALPHA[52] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     4,  4,  5,  6,  7,  8,  9, 10, 12, 13, 15, 17, 20, 22, 25, 28,
    32, 36, 40, 45, 50, 56, 63, 71, 80, 90,101,113,127,144,162,182,
   203,226,255,255 };
static const u8 BETA[52] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     2,  2,  2,  3,  3,  3,  3,  4,  4,  4,  6,  6,  7,  7,  8,  8,
     9,  9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18 };
/* Table 8-17: tC0 by indexA for bS = 1, 2, 3 */
static const u8 TC0[52][3] = {
    {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
    {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
    {0,0,0},{0,0,1},{0,0,1},{0,0,1},{0,0,1},{0,1,1},{0,1,1},{1,1,1},
    {1,1,1},{1,1,1},{1,1,1},{1,1,2},{1,1,2},{1,1,2},{1,1,2},{1,2,3},
    {1,2,3},{2,2,3},{2,2,4},{2,3,4},{2,3,4},{3,3,5},{3,4,6},{3,4,6},
    {4,5,7},{4,5,8},{4,6,9},{5,7,10},{6,8,11},{6,8,13},{7,10,14},{8,11,16},
    {9,12,18},{10,13,20},{11,15,23},{13,17,25} };

static int clip3(int lo, int hi, int v) { return v < lo ? lo : (v > hi ? hi : v); }
static int clip1(int v) { return clip3(0, 255, v); }
static int abs_i(int v) { return v < 0 ? -v : v; }

/* Filter one line of samples across an edge: p[-i-1] = p_i, p[i] = q_i with
 * sample step `step` (1 for a vertical edge, stride for a horizontal edge).
 * spec 8.7.2.3 (bS < 4) and 8.7.2.4 (bS = 4). */
static void filter_line(u8 *q0p, int step, int bs, int alpha, int beta, int tc0, int chroma)
{
    int p0 = q0p[-step], p1 = q0p[-2 * step], q0 = q0p[0], q1 = q0p[step];
    if (!(abs_i(p0 - q0) < alpha && abs_i(p1 - p0) < beta && abs_i(q1 - q0) < beta)) return;
    if (chroma) {
        if (bs < 4) {
            int tc = tc0 + 1;
            int delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
            q0p[-step] = (u8)clip1(p0 + delta);
            q0p[0]     = (u8)clip1(q0 - delta);
        } else {
            q0p[-step] = (u8)((2 * p1 + p0 + q1 + 2) >> 2);
            q0p[0]     = (u8)((2 * q1 + q0 + p1 + 2) >> 2);
        }
        return;
    }
    int p2 = q0p[-3 * step], q2 = q0p[2 * step];
    int ap = abs_i(p2 - p0), aq = abs_i(q2 - q0);
    if (bs < 4) {
        int tc = tc0 + (ap < beta) + (aq < beta);
        int delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
        q0p[-step] = (u8)clip1(p0 + delta);
        q0p[0]     = (u8)clip1(q0 - delta);
        if (ap < beta) q0p[-2 * step] = (u8)(p1 + clip3(-tc0, tc0, (p2 + ((p0 + q0 + 1) >> 1) - (p1 << 1)) >> 1));
        if (aq < beta) q0p[step]      = (u8)(q1 + clip3(-tc0, tc0, (q2 + ((p0 + q0 + 1) >> 1) - (q1 << 1)) >> 1));
    } else {
        int p3 = q0p[-4 * step], q3 = q0p[3 * step];
        int strong = abs_i(p0 - q0) < ((alpha >> 2) + 2);
        if (ap < beta && strong) {
            q0p[-step]     = (u8)((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3);
            q0p[-2 * step] = (u8)((p2 + p1 + p0 + q0 + 2) >> 2);
            q0p[-3 * step] = (u8)((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3);
        } else {
            q0p[-step]     = (u8)((2 * p1 + p0 + q1 + 2) >> 2);
        }
        if (aq < beta && strong) {
            q0p[0]        = (u8)((p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3);
            q0p[step]     = (u8)((p0 + q0 + q1 + q2 + 2) >> 2);
            q0p[2 * step] = (u8)((2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3);
        } else {
            q0p[0]        = (u8)((2 * q1 + q0 + p1 + 2) >> 2);
        }
    }
}

/* boundary strength between 4x4 luma blocks (spec 8.7.2.1), frame MBs,
 * one reference picture: mb_edge = the edge is an MB boundary */
static int bs_of(const dbk_mb_t *p, int pblk, const dbk_mb_t *q, int qblk, int mb_edge)
{
    if (p->intra || q->intra) return mb_edge ? 4 : 3;
    if (((p->nz >> pblk) & 1) || ((q->nz >> qblk) & 1)) return 2;
    if (abs_i(p->mvx - q->mvx) >= 4 || abs_i(p->mvy - q->mvy) >= 4) return 1;
    return 0;
}

void deblock_frame(u8 *y, int stride_y, u8 *uv, int stride_uv,
                   int mbs_w, int mbs_h, const dbk_mb_t *info, int qp_y, int qp_c)
{
    int ia = clip3(0, 51, qp_y), ic = clip3(0, 51, qp_c);
    int alpha = ALPHA[ia], beta = BETA[ia];
    int alpha_c = ALPHA[ic], beta_c = BETA[ic];

    for (int mr = 0; mr < mbs_h; mr++)
        for (int mc = 0; mc < mbs_w; mc++) {
            const dbk_mb_t *cur = &info[mr * mbs_w + mc];
            const dbk_mb_t *left = mc > 0 ? &info[mr * mbs_w + mc - 1] : 0;
            const dbk_mb_t *top  = mr > 0 ? &info[(mr - 1) * mbs_w + mc] : 0;
            u8 *ly = y + (mr * 16) * stride_y + mc * 16;
            u8 *cuv = uv + (mr * 8) * stride_uv + mc * 16;    /* U at even bytes, V at odd */
            int bsv[4][4], bsh[4][4];                          /* [edge][segment] */

            /* strengths: vertical edges e (x = 4e), segment = block row */
            for (int e = 0; e < 4; e++)
                for (int s = 0; s < 4; s++) {
                    if (e == 0) bsv[e][s] = left ? bs_of(left, s * 4 + 3, cur, s * 4 + 0, 1) : 0;
                    else        bsv[e][s] = bs_of(cur, s * 4 + e - 1, cur, s * 4 + e, 0);
                }
            for (int e = 0; e < 4; e++)
                for (int s = 0; s < 4; s++) {
                    if (e == 0) bsh[e][s] = top ? bs_of(top, 12 + s, cur, s, 1) : 0;
                    else        bsh[e][s] = bs_of(cur, (e - 1) * 4 + s, cur, e * 4 + s, 0);
                }

            /* luma vertical edges, then horizontal edges */
            for (int e = 0; e < 4; e++) {
                if (e == 0 && !left) continue;
                for (int r = 0; r < 16; r++) {
                    int bs = bsv[e][r >> 2];
                    if (bs) filter_line(ly + r * stride_y + 4 * e, 1, bs, alpha, beta, bs < 4 ? TC0[ia][bs - 1] : 0, 0);
                }
            }
            for (int e = 0; e < 4; e++) {
                if (e == 0 && !top) continue;
                for (int c = 0; c < 16; c++) {
                    int bs = bsh[e][c >> 2];
                    if (bs) filter_line(ly + (4 * e) * stride_y + c, stride_y, bs, alpha, beta, bs < 4 ? TC0[ia][bs - 1] : 0, 0);
                }
            }
            /* chroma: edges at 0 and 4 (chroma samples) use the luma strengths
             * of edges 0 and 2; chroma sample k maps to luma segment k >> 1 */
            for (int comp = 0; comp < 2; comp++) {
                u8 *cp = cuv + comp;
                for (int e = 0; e < 2; e++) {
                    if (e == 0 && !left) continue;
                    for (int r = 0; r < 8; r++) {
                        int bs = bsv[2 * e][r >> 1];
                        if (bs) filter_line(cp + r * stride_uv + 2 * (4 * e), 2, bs, alpha_c, beta_c, bs < 4 ? TC0[ic][bs - 1] : 0, 1);
                    }
                }
                for (int e = 0; e < 2; e++) {
                    if (e == 0 && !top) continue;
                    for (int c = 0; c < 8; c++) {
                        int bs = bsh[2 * e][c >> 1];
                        if (bs) filter_line(cp + (4 * e) * stride_uv + 2 * c, stride_uv, bs, alpha_c, beta_c, bs < 4 ? TC0[ic][bs - 1] : 0, 1);
                    }
                }
            }
        }
}
