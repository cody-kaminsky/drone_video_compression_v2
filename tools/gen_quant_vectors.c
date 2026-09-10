/* gen_quant_vectors.c — golden vectors for the VHDL quant_engine.
 *
 * Links against quant.c. Writes build/quant_vectors.txt:
 *   M <mode> <qp>
 *   I <v0> .. <v15>
 *   O <v0> .. <v15>
 * Modes: 0 quant_4x4, 1 iquant_4x4, 2 quant_dc_4x4, 3 iquant_dc_4x4,
 *        4 quant_dc_2x2, 5 iquant_dc_2x2 (2x2 modes use indices 0..3).
 * is_intra is always 1 (I-only encoder).
 *
 * Input ranges follow what the encoder can actually produce:
 *   AC coef      : |c| <= 4080   (4x4 DCT of 9-bit residual)
 *   luma DC coef : |c| <= 65280  (Hadamard of 16 DC coefs)
 *   chroma DC    : |c| <= 16320  (2x2 Hadamard of 4 DC coefs)
 *   levels       : |l| <= 2063   (CAVLC escape limit)
 * plus a few full-i16 stress cases for the AC forward path.
 */
#include "types.h"
#include "quant.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned rng_state = 0xC0FFEE11;
static unsigned xorshift32(void)
{
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static int rnd(int range) { return (int)(xorshift32() % (2u * range + 1u)) - range; }

static void write_test(FILE *f, int mode, int qp, const i32 in[16], const i32 out[16])
{
    fprintf(f, "M %d %d\nI", mode, qp);
    for (int i = 0; i < 16; i++) fprintf(f, " %d", (int)in[i]);
    fprintf(f, "\nO");
    for (int i = 0; i < 16; i++) fprintf(f, " %d", (int)out[i]);
    fprintf(f, "\n");
}

static void gen(FILE *f, int mode, int qp, const i32 in[16])
{
    i32 out[16] = {0};
    i16 in16[16], lev16[16];
    i32 coef32[16];
    for (int i = 0; i < 16; i++) in16[i] = (i16)in[i];
    switch (mode) {
    case 0: quant_4x4(in16, lev16, qp, 1);      for (int i = 0; i < 16; i++) out[i] = lev16[i]; break;
    case 1: iquant_4x4(in16, coef32, qp);       for (int i = 0; i < 16; i++) out[i] = coef32[i]; break;
    case 2: quant_dc_4x4(in, lev16, qp, 1);     for (int i = 0; i < 16; i++) out[i] = lev16[i]; break;
    case 3: iquant_dc_4x4(in16, coef32, qp);    for (int i = 0; i < 16; i++) out[i] = coef32[i]; break;
    case 4: quant_dc_2x2(in16, lev16, qp, 1);   for (int i = 0; i < 4; i++) out[i] = lev16[i]; break;
    case 5: iquant_dc_2x2(in16, coef32, qp);    for (int i = 0; i < 4; i++) out[i] = coef32[i]; break;
    }
    write_test(f, mode, qp, in, out);
}

int main(void)
{
    FILE *f = fopen("build/quant_vectors.txt", "w");
    if (!f) { perror("build/quant_vectors.txt"); return 1; }
    static const int qps[] = {0, 1, 5, 6, 11, 12, 17, 18, 23, 24, 29, 30, 35, 36, 41, 42, 47, 51};
    const int nqp = sizeof qps / sizeof qps[0];
    const int in_range[6]  = {4080, 2063, 65280, 2063, 16320, 2063};
    int count = 0;
    for (int mode = 0; mode < 6; mode++) {
        int n = (mode >= 4) ? 4 : 16;
        for (int q = 0; q < nqp; q++) {
            int qp = qps[q];
            i32 in[16];
            /* zeros */
            for (int i = 0; i < 16; i++) in[i] = 0;
            gen(f, mode, qp, in); count++;
            /* +-1 and small values */
            for (int i = 0; i < 16; i++) in[i] = (i < n) ? ((i & 1) ? -1 : 1) : 0;
            gen(f, mode, qp, in); count++;
            for (int i = 0; i < 16; i++) in[i] = (i < n) ? rnd(8) : 0;
            gen(f, mode, qp, in); count++;
            /* extremes of the realistic range */
            for (int i = 0; i < 16; i++) in[i] = (i < n) ? ((i & 1) ? -in_range[mode] : in_range[mode]) : 0;
            gen(f, mode, qp, in); count++;
            /* random within range */
            for (int r = 0; r < 6; r++) {
                for (int i = 0; i < 16; i++) in[i] = (i < n) ? rnd(in_range[mode]) : 0;
                gen(f, mode, qp, in); count++;
            }
            /* sparse random (typical) */
            for (int r = 0; r < 4; r++) {
                for (int i = 0; i < 16; i++)
                    in[i] = (i < n && (xorshift32() & 3) == 0) ? rnd(in_range[mode] / 8 + 1) : 0;
                gen(f, mode, qp, in); count++;
            }
        }
    }
    /* AC forward full-i16 stress (abs*MF still fits i32) */
    for (int r = 0; r < 20; r++) {
        i32 in[16];
        for (int i = 0; i < 16; i++) in[i] = rnd(32767);
        gen(f, 0, qps[r % nqp], in); count++;
    }
    fclose(f);
    printf("wrote %d quant vectors to build/quant_vectors.txt\n", count);
    return 0;
}
