/* rd_tables.h — integer rate-distortion constants shared by the C reference
 * and the VHDL mode decider (mode_decide_engine). Both must use these
 * exact tables so hardware decisions match the reference bit for bit.
 *
 * I_4x4 mode decision policy (chosen from the 2026-09 shortlist study):
 *   1. screen all available modes by SATD plus a mode-signalling penalty
 *        cost = SATD + ((RD_SLAM16[qp] * mode_bits + 8) >> 4)
 *      where mode_bits = 1 if the mode equals predIntra4x4PredMode, else 4
 *      (RD_SLAM16 = 16 * 2 * 2^((qp-12)/6), rounded);
 *   2. keep the RD_I4_SHORTLIST best (ties: lower mode index first);
 *   3. evaluate those fully: transform, quantize, CAVLC bit estimate,
 *      reconstruct, and pick the smallest
 *        J = 16 * SSD + RD_LAM16[qp] * (residual_bits + mode_bits)
 *      (RD_LAM16 = 16 * 0.425 * 2^((qp-12)/3), rounded; ties: earlier in
 *      shortlist order wins).
 */
#ifndef DCC_RD_TABLES_H
#define DCC_RD_TABLES_H

#define RD_I4_SHORTLIST 3

static const int RD_SLAM16[52] = {
       8,    9,   10,   11,   13,   14,   16,   18,   20,   23,   25,   29,   32,
      36,   40,   45,   51,   57,   64,   72,   81,   91,  102,  114,  128,  144,
     161,  181,  203,  228,  256,  287,  323,  362,  406,  456,  512,  575,  645,
     724,  813,  912, 1024, 1149, 1290, 1448, 1625, 1825, 2048, 2299, 2580, 2896 };

static const int RD_LAM16[52] = {
       0,     1,     1,     1,     1,     1,     2,     2,     3,     3,     4,     5,     7,
       9,    11,    14,    17,    22,    27,    34,    43,    54,    69,    86,   109,   137,
     173,   218,   274,   345,   435,   548,   691,   870,  1097,  1382,  1741,  2193,  2763,
    3482,  4387,  5527,  6963,  8773, 11053, 13926, 17546, 22107, 27853, 35092, 44214, 55706 };

#endif
