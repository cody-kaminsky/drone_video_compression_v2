/* intra.c — intra prediction (luma I_16x16 and chroma 8x8). */
#include "intra.h"
#include <string.h>

static int clip_u8(int x)
{
    if (x < 0)   return 0;
    if (x > 255) return 255;
    return x;
}

static int dc_with_neighbors_16(const u8 top[16], const u8 left[16],
                                int avail_top, int avail_left)
{
    int sum = 0;
    if (avail_top) {
        for (int j = 0; j < 16; j++) sum += top[j];
    }
    if (avail_left) {
        for (int i = 0; i < 16; i++) sum += left[i];
    }
    if (avail_top && avail_left) return (sum + 16) >> 5;
    if (avail_top || avail_left) return (sum + 8) >> 4;
    return 128;
}

void predict_16x16(int mode,
                   const u8 top[16], const u8 left[16], u8 tl,
                   int avail_top, int avail_left, int avail_tl,
                   u8 pred[256])
{
    switch (mode) {

    case I16_VERTICAL:
        if (!avail_top) {
            /* Spec: VERTICAL requires top — caller should not have selected
             * this. Fallback to DC. */
            int dc = dc_with_neighbors_16(top, left, avail_top, avail_left);
            for (int k = 0; k < 256; k++) pred[k] = (u8)dc;
            return;
        }
        for (int i = 0; i < 16; i++)
            memcpy(&pred[i * 16], top, 16);
        return;

    case I16_HORIZONTAL:
        if (!avail_left) {
            int dc = dc_with_neighbors_16(top, left, avail_top, avail_left);
            for (int k = 0; k < 256; k++) pred[k] = (u8)dc;
            return;
        }
        for (int i = 0; i < 16; i++)
            memset(&pred[i * 16], left[i], 16);
        return;

    case I16_DC: {
        int dc = dc_with_neighbors_16(top, left, avail_top, avail_left);
        for (int k = 0; k < 256; k++) pred[k] = (u8)dc;
        return;
    }

    case I16_PLANE: {
        if (!(avail_top && avail_left && avail_tl)) {
            int dc = dc_with_neighbors_16(top, left, avail_top, avail_left);
            for (int k = 0; k < 256; k++) pred[k] = (u8)dc;
            return;
        }
        /* Spec 8.3.3.4 — plane prediction. */
        int top_ext[17], left_ext[17];
        top_ext[0]  = tl;
        for (int j = 0; j < 16; j++) top_ext[j + 1]  = top[j];
        left_ext[0] = tl;
        for (int i = 0; i < 16; i++) left_ext[i + 1] = left[i];

        int H = 0, V = 0;
        for (int i = 0; i < 8; i++) {
            H += (i + 1) * (top_ext[8 + 1 + i]  - top_ext[8 - 1 - i]);
            V += (i + 1) * (left_ext[8 + 1 + i] - left_ext[8 - 1 - i]);
        }
        int b = (5 * H + 32) >> 6;
        int c = (5 * V + 32) >> 6;
        int a = 16 * (left_ext[16] + top_ext[16]);
        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 16; j++) {
                int v = (a + b * (j - 7) + c * (i - 7) + 16) >> 5;
                pred[i * 16 + j] = (u8)clip_u8(v);
            }
        }
        return;
    }
    }
}

/* Helper: average of N neighbor pixels with rounding. */
static int avg_n(const u8 *p, int n)
{
    int s = 0;
    for (int k = 0; k < n; k++) s += p[k];
    return (s + (n / 2)) / n;
}

/* IC_DC computation — extracted so IC_PLANE's unavailable-neighbor fallback
 * can call it directly (avoiding self-recursion in predict_chroma_8x8). */
static void chroma_8x8_dc(const u8 top[8], const u8 left[8],
                          int avail_top, int avail_left, u8 pred[64])
{
    /* Spec 8.3.4.2 — chroma DC has 4 4x4 quadrants with different
     * averaging rules depending on neighbor availability. */
    int dc[4]; /* index by quadrant: 0=TL, 1=TR, 2=BL, 3=BR */

    if (avail_top && avail_left) {
        int s = 0;
        for (int k = 0; k < 4; k++) s += top[k] + left[k];
        dc[0] = (s + 4) >> 3;
    } else if (avail_top)        dc[0] = avg_n(top,  4);
      else if (avail_left)       dc[0] = avg_n(left, 4);
      else                       dc[0] = 128;

    if (avail_top)               dc[1] = avg_n(top + 4, 4);
    else if (avail_left)         dc[1] = avg_n(left,    4);
    else                         dc[1] = 128;

    if (avail_left)              dc[2] = avg_n(left + 4, 4);
    else if (avail_top)          dc[2] = avg_n(top,      4);
    else                         dc[2] = 128;

    if (avail_top && avail_left) {
        int s = 0;
        for (int k = 4; k < 8; k++) s += top[k] + left[k];
        dc[3] = (s + 4) >> 3;
    } else if (avail_top)        dc[3] = avg_n(top + 4,  4);
      else if (avail_left)       dc[3] = avg_n(left + 4, 4);
      else                       dc[3] = 128;

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            int q = (i / 4) * 2 + (j / 4);
            pred[i * 8 + j] = (u8)dc[q];
        }
    }
}

void predict_chroma_8x8(int mode,
                        const u8 top[8], const u8 left[8], u8 tl,
                        int avail_top, int avail_left, int avail_tl,
                        u8 pred[64])
{
    switch (mode) {

    case IC_HORIZONTAL:
        if (!avail_left) {
            /* fallback */
            int dc = avail_top ? avg_n(top, 8) : 128;
            for (int k = 0; k < 64; k++) pred[k] = (u8)dc;
            return;
        }
        for (int i = 0; i < 8; i++)
            memset(&pred[i * 8], left[i], 8);
        return;

    case IC_VERTICAL:
        if (!avail_top) {
            int dc = avail_left ? avg_n(left, 8) : 128;
            for (int k = 0; k < 64; k++) pred[k] = (u8)dc;
            return;
        }
        for (int i = 0; i < 8; i++)
            memcpy(&pred[i * 8], top, 8);
        return;

    case IC_DC:
        chroma_8x8_dc(top, left, avail_top, avail_left, pred);
        return;

    case IC_PLANE: {
        if (!(avail_top && avail_left && avail_tl)) {
            chroma_8x8_dc(top, left, avail_top, avail_left, pred);
            return;
        }
        int top_ext[9], left_ext[9];
        top_ext[0]  = tl;
        for (int j = 0; j < 8; j++) top_ext[j + 1]  = top[j];
        left_ext[0] = tl;
        for (int i = 0; i < 8; i++) left_ext[i + 1] = left[i];
        int H = 0, V = 0;
        for (int i = 0; i < 4; i++) {
            H += (i + 1) * (top_ext[4 + 1 + i]  - top_ext[4 - 1 - i]);
            V += (i + 1) * (left_ext[4 + 1 + i] - left_ext[4 - 1 - i]);
        }
        int b = (34 * H + 32) >> 6;
        int c = (34 * V + 32) >> 6;
        int a = 16 * (left_ext[8] + top_ext[8]);
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                int v = (a + b * (j - 3) + c * (i - 3) + 16) >> 5;
                pred[i * 8 + j] = (u8)clip_u8(v);
            }
        }
        return;
    }
    }
}

/* ============================================================
 * I_4x4 prediction — 9 modes (spec 8.3.1.2)
 * ============================================================
 *
 * Reference sample layout (after replication of unavailable top-right):
 *
 *      X | A B C D E F G H
 *      ---+----------------
 *      I |  *  *  *  *
 *      J |  *  *  *  *
 *      K |  *  *  *  *
 *      L |  *  *  *  *
 *
 *   X = tl          A..D = top[0..3]      E..H = top[4..7]
 *   I..L = left[0..3]
 *
 * Each predicted sample p[r][c] is a small linear combination of these
 * 13 reference values.
 */
void predict_4x4(int mode,
                 const u8 top[8], const u8 left[4], u8 tl,
                 int avail_top, int avail_left, int avail_tl,
                 u8 pred[16])
{
    /* Local aliases for readability — match spec naming. */
    int A = avail_top  ? top[0]  : 128;
    int B = avail_top  ? top[1]  : 128;
    int C = avail_top  ? top[2]  : 128;
    int D = avail_top  ? top[3]  : 128;
    int E = avail_top  ? top[4]  : 128;
    int F = avail_top  ? top[5]  : 128;
    int G = avail_top  ? top[6]  : 128;
    int H = avail_top  ? top[7]  : 128;
    int I = avail_left ? left[0] : 128;
    int J = avail_left ? left[1] : 128;
    int K = avail_left ? left[2] : 128;
    int L = avail_left ? left[3] : 128;
    int X = avail_tl   ? tl      : 128;

    /* Helper macro to set p[r][c] = val with clip. */
    #define P(r,c,val) (pred[(r)*4 + (c)] = (u8)clip_u8(val))

    switch (mode) {

    case I4_VERTICAL: {
        /* p[r][c] = top[c] for all r. */
        for (int r = 0; r < 4; r++) {
            P(r, 0, A); P(r, 1, B); P(r, 2, C); P(r, 3, D);
        }
        return;
    }

    case I4_HORIZONTAL: {
        for (int c = 0; c < 4; c++) {
            P(0, c, I); P(1, c, J); P(2, c, K); P(3, c, L);
        }
        return;
    }

    case I4_DC: {
        int dc;
        if (avail_top && avail_left)      dc = (A+B+C+D + I+J+K+L + 4) >> 3;
        else if (avail_top)               dc = (A+B+C+D + 2) >> 2;
        else if (avail_left)              dc = (I+J+K+L + 2) >> 2;
        else                              dc = 128;
        for (int i = 0; i < 16; i++) pred[i] = (u8)dc;
        return;
    }

    case I4_DIAG_DOWN_LEFT: {
        /* Spec 8.3.1.2.4 — needs top + (optionally) top-right.
         * Caller is responsible for replicating top[3] into top[4..7] if
         * top-right is unavailable. */
        P(0, 0, (A + 2*B + C + 2) >> 2);
        P(0, 1, (B + 2*C + D + 2) >> 2); P(1, 0, pred[1]);
        P(0, 2, (C + 2*D + E + 2) >> 2); P(1, 1, pred[2]); P(2, 0, pred[2]);
        P(0, 3, (D + 2*E + F + 2) >> 2); P(1, 2, pred[3]); P(2, 1, pred[3]); P(3, 0, pred[3]);
        P(1, 3, (E + 2*F + G + 2) >> 2); P(2, 2, pred[7]); P(3, 1, pred[7]);
        P(2, 3, (F + 2*G + H + 2) >> 2); P(3, 2, pred[11]);
        P(3, 3, (G + 3*H + 2) >> 2);
        return;
    }

    case I4_DIAG_DOWN_RIGHT: {
        /* Spec 8.3.1.2.5 */
        P(3, 0, (J + 2*K + L + 2) >> 2);
        P(2, 0, (I + 2*J + K + 2) >> 2); P(3, 1, pred[8]);
        P(1, 0, (X + 2*I + J + 2) >> 2); P(2, 1, pred[4]); P(3, 2, pred[4]);
        P(0, 0, (I + 2*X + A + 2) >> 2); P(1, 1, pred[0]); P(2, 2, pred[0]); P(3, 3, pred[0]);
        P(0, 1, (X + 2*A + B + 2) >> 2); P(1, 2, pred[1]); P(2, 3, pred[1]);
        P(0, 2, (A + 2*B + C + 2) >> 2); P(1, 3, pred[2]);
        P(0, 3, (B + 2*C + D + 2) >> 2);
        return;
    }

    case I4_VERTICAL_RIGHT: {
        /* Spec 8.3.1.2.6 */
        P(0, 0, (X + A + 1) >> 1);                      P(2, 1, pred[0]);
        P(0, 1, (A + B + 1) >> 1);                      P(2, 2, pred[1]);
        P(0, 2, (B + C + 1) >> 1);                      P(2, 3, pred[2]);
        P(0, 3, (C + D + 1) >> 1);
        P(1, 0, (I + 2*X + A + 2) >> 2);                P(3, 1, pred[4]);
        P(1, 1, (X + 2*A + B + 2) >> 2);                P(3, 2, pred[5]);
        P(1, 2, (A + 2*B + C + 2) >> 2);                P(3, 3, pred[6]);
        P(1, 3, (B + 2*C + D + 2) >> 2);
        P(2, 0, (X + 2*I + J + 2) >> 2);
        P(3, 0, (I + 2*J + K + 2) >> 2);
        return;
    }

    case I4_HORIZONTAL_DOWN: {
        /* Spec 8.3.1.2.7 */
        P(0, 0, (X + I + 1) >> 1);                      P(1, 2, pred[0]);
        P(1, 0, (I + J + 1) >> 1);                      P(2, 2, pred[4]);
        P(2, 0, (J + K + 1) >> 1);                      P(3, 2, pred[8]);
        P(3, 0, (K + L + 1) >> 1);
        P(0, 1, (I + 2*X + A + 2) >> 2);                P(1, 3, pred[1]);
        P(0, 2, (X + 2*A + B + 2) >> 2);
        P(0, 3, (A + 2*B + C + 2) >> 2);
        P(1, 1, (X + 2*I + J + 2) >> 2);                P(2, 3, pred[5]);
        P(2, 1, (I + 2*J + K + 2) >> 2);                P(3, 3, pred[9]);
        P(3, 1, (J + 2*K + L + 2) >> 2);
        return;
    }

    case I4_VERTICAL_LEFT: {
        /* Spec 8.3.1.2.8 */
        P(0, 0, (A + B + 1) >> 1);
        P(0, 1, (B + C + 1) >> 1);                      P(2, 0, pred[1]);
        P(0, 2, (C + D + 1) >> 1);                      P(2, 1, pred[2]);
        P(0, 3, (D + E + 1) >> 1);                      P(2, 2, pred[3]);
        P(1, 0, (A + 2*B + C + 2) >> 2);
        P(1, 1, (B + 2*C + D + 2) >> 2);                P(3, 0, pred[5]);
        P(1, 2, (C + 2*D + E + 2) >> 2);                P(3, 1, pred[6]);
        P(1, 3, (D + 2*E + F + 2) >> 2);                P(3, 2, pred[7]);
        P(2, 3, (E + F + 1) >> 1);
        P(3, 3, (E + 2*F + G + 2) >> 2);
        return;
    }

    case I4_HORIZONTAL_UP: {
        /* Spec 8.3.1.2.9 */
        P(0, 0, (I + J + 1) >> 1);
        P(0, 1, (I + 2*J + K + 2) >> 2);
        P(0, 2, (J + K + 1) >> 1);                      P(1, 0, pred[2]);
        P(0, 3, (J + 2*K + L + 2) >> 2);                P(1, 1, pred[3]);
        P(1, 2, (K + L + 1) >> 1);                      P(2, 0, pred[6]);
        P(1, 3, (K + 3*L + 2) >> 2);                    P(2, 1, pred[7]);
        /* Bottom-right region: replicate L. */
        P(2, 2, L); P(2, 3, L);
        P(3, 0, L); P(3, 1, L); P(3, 2, L); P(3, 3, L);
        return;
    }

    } /* switch */

    #undef P
}
