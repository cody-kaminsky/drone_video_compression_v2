/* quant.c — H.264 quantization tables and forward/inverse routines.
 *
 * FPGA: the multiplies here are the only DSP users in the front-end pipeline.
 *       16 DSPs in parallel give one 4x4 quant per cycle. Same for iquant.
 *       The divisions by 3 / 6 in the rounding offset are precomputed at
 *       run-time per QP — for hardware, build a small ROM of (1<<qbits)/3
 *       indexed by QP and remove the division entirely.
 */
#include "quant.h"

/* Forward quantization scale matrix MF[qp%6][pos]. Spec 8.5.9. */
static const i32 quant_coef[6][16] = {
    {13107, 8066,13107, 8066,
      8066, 5243, 8066, 5243,
     13107, 8066,13107, 8066,
      8066, 5243, 8066, 5243},
    {11916, 7490,11916, 7490,
      7490, 4660, 7490, 4660,
     11916, 7490,11916, 7490,
      7490, 4660, 7490, 4660},
    {10082, 6554,10082, 6554,
      6554, 4194, 6554, 4194,
     10082, 6554,10082, 6554,
      6554, 4194, 6554, 4194},
    { 9362, 5825, 9362, 5825,
      5825, 3647, 5825, 3647,
      9362, 5825, 9362, 5825,
      5825, 3647, 5825, 3647},
    { 8192, 5243, 8192, 5243,
      5243, 3355, 5243, 3355,
      8192, 5243, 8192, 5243,
      5243, 3355, 5243, 3355},
    { 7282, 4559, 7282, 4559,
      4559, 2893, 4559, 2893,
      7282, 4559, 7282, 4559,
      4559, 2893, 4559, 2893},
};

/* Inverse dequantization scale V[qp%6][pos]. Spec 8.5.9. */
static const i32 dequant_coef[6][16] = {
    {10,13,10,13, 13,16,13,16, 10,13,10,13, 13,16,13,16},
    {11,14,11,14, 14,18,14,18, 11,14,11,14, 14,18,14,18},
    {13,16,13,16, 16,20,16,20, 13,16,13,16, 16,20,16,20},
    {14,18,14,18, 18,23,18,23, 14,18,14,18, 18,23,18,23},
    {16,20,16,20, 20,25,20,25, 16,20,16,20, 20,25,20,25},
    {18,23,18,23, 23,29,23,29, 18,23,18,23, 23,29,23,29},
};

/* Spec table 8-9: chroma QP from (luma_qp + offset) clipped to 0..51. */
static const int chroma_qp_table[52] = {
     0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,
    12,13,14,15,16,17,18,19,20,21,22,23,
    24,25,26,27,28,29,29,30,31,32,32,33,
    34,34,35,35,36,36,37,37,37,38,38,38,
    39,39,39,39
};

int chroma_qp(int luma_qp, int chroma_qp_index_offset)
{
    int q = luma_qp + chroma_qp_index_offset;
    if (q < 0)  q = 0;
    if (q > 51) q = 51;
    return chroma_qp_table[q];
}

void quant_4x4(const i16 coef[16], i16 level[16], int qp, int is_intra)
{
    int qbits = 15 + qp / 6;
    int qmod  = qp % 6;
    /* Rounding offset: intra uses 1/3, inter 1/6. */
    int f = (1 << qbits) / (is_intra ? 3 : 6);
    for (int i = 0; i < 16; i++) {
        i32 c = coef[i];
        i32 abs_c = c < 0 ? -c : c;
        i32 lev = (abs_c * quant_coef[qmod][i] + f) >> qbits;
        level[i] = (i16)(c < 0 ? -lev : lev);
    }
}

void iquant_4x4(const i16 level[16], i32 coef[16], int qp)
{
    /* Per spec/JM: coef = level * V[qmod][i] << qdiv. The subsequent iDCT
     * and the (+32)>>6 in reconstruction recover the residual.
     *
     * Output is i32 because dequantized coefficients can exceed i16 at
     * high QP (e.g., level=200 at QP=42 produces coef ≈ 600k). */
    int qdiv = qp / 6;
    int qmod = qp % 6;
    for (int i = 0; i < 16; i++)
        coef[i] = ((i32)level[i] * dequant_coef[qmod][i]) << qdiv;
}

void quant_dc_4x4(const i32 coef[16], i16 level[16], int qp, int is_intra)
{
    /* Spec 8.5.10: inverse uses coef = T_iDC * V << (qdiv-2). For round-trip
     * equivalence with the AC chain (which uses q_bits = 15+qdiv), the DC
     * forward quant must use q_bits = 15+qdiv+2 (i.e., 2 extra right shifts
     * beyond AC, not 1). The original "+1" was off by one. */
    int qbits = 15 + qp / 6 + 2;
    int qmod  = qp % 6;
    int f = (1 << qbits) / (is_intra ? 3 : 6);
    i32 mf = quant_coef[qmod][0];
    for (int i = 0; i < 16; i++) {
        i32 c = coef[i];
        i32 abs_c = c < 0 ? -c : c;
        i32 lev = (abs_c * mf + f) >> qbits;
        level[i] = (i16)(c < 0 ? -lev : lev);
    }
}

void iquant_dc_4x4(const i16 level[16], i32 coef[16], int qp)
{
    /* Spec 8.5.10: f = (T_iDC * V) << (qdiv - 2)        for qdiv >= 2
     *              f = (T_iDC * V + 2^(1-qdiv)) >> (2-qdiv)   for qdiv < 2
     * (LevelScale4x4 in the spec equals V — the quoted "* 16" cancels with
     * a corresponding -4 in the shift, simplifying to the above.) */
    int qdiv = qp / 6;
    int qmod = qp % 6;
    i32 dq = dequant_coef[qmod][0];
    if (qdiv >= 2) {
        int sh = qdiv - 2;
        for (int i = 0; i < 16; i++)
            coef[i] = ((i32)level[i] * dq) << sh;
    } else {
        int sh = 2 - qdiv;
        i32 round = 1 << (sh - 1);
        for (int i = 0; i < 16; i++)
            coef[i] = (((i32)level[i] * dq) + round) >> sh;
    }
}

void quant_dc_2x2(const i16 coef[4], i16 level[4], int qp, int is_intra)
{
    /* Spec 8.5.11: chroma DC inverse shift = qdiv-1 (qdiv>=1) or >>1 (qdiv=0).
     * Forward must use q_bits = 15+qdiv+1 (one extra shift beyond AC; less
     * than luma DC's +2 because the 2x2 Hadamard scale is 4 not 16). */
    int qbits = 15 + qp / 6 + 1;
    int qmod  = qp % 6;
    int f = (1 << qbits) / (is_intra ? 3 : 6);
    i32 mf = quant_coef[qmod][0];
    for (int i = 0; i < 4; i++) {
        i32 c = coef[i];
        i32 abs_c = c < 0 ? -c : c;
        i32 lev = (abs_c * mf + f) >> qbits;
        level[i] = (i16)(c < 0 ? -lev : lev);
    }
}

void iquant_dc_2x2(const i16 level[4], i32 coef[4], int qp)
{
    /* Spec 8.5.11.1: dcC' = T_iDC * V << (qdiv-1)   for qdiv >= 1
     *                dcC' = (T_iDC * V) >> 1         for qdiv = 0  */
    int qdiv = qp / 6;
    int qmod = qp % 6;
    i32 dq = dequant_coef[qmod][0];
    if (qdiv >= 1) {
        int sh = qdiv - 1;
        for (int i = 0; i < 4; i++)
            coef[i] = ((i32)level[i] * dq) << sh;
    } else {
        for (int i = 0; i < 4; i++)
            coef[i] = ((i32)level[i] * dq) >> 1;
    }
}
