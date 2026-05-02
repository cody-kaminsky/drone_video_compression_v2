/* test_roundtrip.c — unit tests for the transform + quant round-trip.
 *
 * Builds three test scenarios:
 *   1. AC-only path: dct → quant_4x4 → iquant_4x4 → idct → recon
 *   2. DC path:      hadamard4x4 → quant_dc_4x4 → iquant_dc_4x4 → ihadamard4x4
 *   3. Full I_16x16 path: combination, with DC extracted/Hadamard'd
 *
 * For each, prints mean abs error vs the original samples.
 */
#include "../src/types.h"
#include "../src/transform.h"
#include "../src/quant.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int clip_u8(int x) { return x<0?0:(x>255?255:x); }

static void print_block_i16(const char *name, const i16 b[16])
{
    printf("%s:\n", name);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            printf("%6d ", b[i*4 + j]);
        printf("\n");
    }
}

static void print_block_i32(const char *name, const i32 b[16])
{
    printf("%s:\n", name);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            printf("%8d ", b[i*4 + j]);
        printf("\n");
    }
}

static void print_block_u8(const char *name, const u8 b[16])
{
    printf("%s:\n", name);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            printf("%4d ", b[i*4 + j]);
        printf("\n");
    }
}

/* Test 1: AC-only round-trip on a single 4x4 block.
 * Use a residual with no DC component, simulating an AC-only coding path
 * (not what I_16x16 does, but a useful baseline). */
static void test_ac_roundtrip(int qp)
{
    printf("\n=== test_ac_roundtrip QP=%d ===\n", qp);

    /* Synthetic 4x4 residual with mixed AC content. */
    i16 residual[16] = {
         50, -10,   5, -3,
        -20,  30, -15, 10,
         15, -20,  10, -5,
         -5,  10,  -8, 12
    };
    print_block_i16("residual", residual);

    i16 dct[16];
    dct4x4(residual, dct);
    print_block_i16("dct(residual)", dct);

    i16 lev[16];
    quant_4x4(dct, lev, qp, 1);
    print_block_i16("quant(dct)", lev);

    i32 dq[16];
    iquant_4x4(lev, dq, qp);
    print_block_i32("iquant", dq);

    i32 res_recon[16];
    idct4x4(dq, res_recon);
    print_block_i32("idct(iquant)", res_recon);

    /* Recon: assume pred=0 for this test, so recon residual = (val+32)>>6. */
    int sum_abs_err = 0;
    int max_abs_err = 0;
    for (int i = 0; i < 16; i++) {
        int rr = (res_recon[i] + 32) >> 6;
        int err = rr - residual[i];
        if (err < 0) err = -err;
        sum_abs_err += err;
        if (err > max_abs_err) max_abs_err = err;
    }
    printf("AC round-trip: mean abs err = %.2f, max abs err = %d\n",
           sum_abs_err / 16.0, max_abs_err);
}

/* Test 2: full I_16x16 path on a 4x4-of-DCs scenario.
 * Synthesizes 16 4x4 sub-blocks of a 16x16 MB, runs the full DC+AC encode
 * and decode path, and measures pixel-level error.
 *
 * The 16x16 source is a constant block (so all DCs identical, all ACs zero),
 * and prediction is zero — residual equals source. */
static void test_i16x16_roundtrip(int qp, int constant_value)
{
    printf("\n=== test_i16x16_roundtrip QP=%d val=%d ===\n", qp, constant_value);

    /* 16x16 residual: all elements = constant_value. */
    i16 residual_16x16[256];
    for (int i = 0; i < 256; i++) residual_16x16[i] = constant_value;

    /* Forward path. */
    i16 luma_dc[16];                  /* one DC per 4x4 sub-block */
    i16 ac_levels[16][16];

    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            /* Extract 4x4 sub-block. */
            i16 sub[16];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    sub[r*4 + c] = residual_16x16[(br*4 + r)*16 + (bc*4 + c)];

            i16 dct[16];
            dct4x4(sub, dct);
            int idx = br*4 + bc;
            luma_dc[idx] = dct[0];

            i16 dct_zero_dc[16];
            memcpy(dct_zero_dc, dct, sizeof(dct));
            dct_zero_dc[0] = 0;
            quant_4x4(dct_zero_dc, ac_levels[idx], qp, 1);
        }
    }

    i32 dc_had[16];
    i16 dc_levels[16];
    hadamard4x4(luma_dc, dc_had);
    quant_dc_4x4(dc_had, dc_levels, qp, 1);

    print_block_i16("luma_dc (raw)",      luma_dc);
    print_block_i32("luma_dc (hadamard)", dc_had);
    print_block_i16("luma_dc (levels)",   dc_levels);

    /* Inverse path. */
    i32 dc_dq[16], dc_recon[16];
    iquant_dc_4x4(dc_levels, dc_dq, qp);
    ihadamard4x4(dc_dq, dc_recon);
    print_block_i32("dc_dq",    dc_dq);
    print_block_i32("dc_recon", dc_recon);

    i16 recon_16x16[256];
    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            int idx = br*4 + bc;
            i32 ac_dq[16];
            iquant_4x4(ac_levels[idx], ac_dq, qp);
            ac_dq[0] = dc_recon[idx];
            i32 res_recon[16];
            idct4x4(ac_dq, res_recon);
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    int v = (res_recon[r*4 + c] + 32) >> 6;
                    recon_16x16[(br*4 + r)*16 + (bc*4 + c)] = (i16)v;
                }
            }
        }
    }

    /* Compute error. */
    int sum_abs_err = 0;
    int max_abs_err = 0;
    for (int i = 0; i < 256; i++) {
        int err = recon_16x16[i] - constant_value;
        if (err < 0) err = -err;
        sum_abs_err += err;
        if (err > max_abs_err) max_abs_err = err;
    }
    printf("I_16x16 round-trip: mean abs err = %.2f, max abs err = %d, recon[0]=%d\n",
           sum_abs_err / 256.0, max_abs_err, recon_16x16[0]);
}

/* Test 3: forward + inverse full chain with non-zero prediction.
 * This is what the encoder actually does. */
static void test_i16x16_with_pred(int qp)
{
    printf("\n=== test_i16x16_with_pred QP=%d ===\n", qp);

    /* Source: gradient. */
    u8 src[256];
    for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
            src[i*16 + j] = (u8)(50 + i*8 + j);

    /* Prediction: DC = 128. */
    u8 pred[256];
    for (int i = 0; i < 256; i++) pred[i] = 128;

    /* Residual = src - pred. */
    i16 res[256];
    for (int i = 0; i < 256; i++) res[i] = (i16)((int)src[i] - 128);

    /* Forward + inverse using same code structure as the encoder. */
    i16 luma_dc[16];
    i16 ac_levels[16][16];
    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            i16 sub[16];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    sub[r*4 + c] = res[(br*4 + r)*16 + (bc*4 + c)];
            i16 dct[16];
            dct4x4(sub, dct);
            int idx = br*4 + bc;
            luma_dc[idx] = dct[0];
            i16 dct_zd[16];
            memcpy(dct_zd, dct, sizeof(dct));
            dct_zd[0] = 0;
            quant_4x4(dct_zd, ac_levels[idx], qp, 1);
        }
    }
    i32 dc_had[16];
    i16 dc_levels[16];
    hadamard4x4(luma_dc, dc_had);
    quant_dc_4x4(dc_had, dc_levels, qp, 1);

    i32 dc_dq[16], dc_recon[16];
    iquant_dc_4x4(dc_levels, dc_dq, qp);
    ihadamard4x4(dc_dq, dc_recon);

    u8 recon[256];
    for (int br = 0; br < 4; br++) {
        for (int bc = 0; bc < 4; bc++) {
            int idx = br*4 + bc;
            i32 ac_dq[16];
            iquant_4x4(ac_levels[idx], ac_dq, qp);
            ac_dq[0] = dc_recon[idx];
            i32 r2[16];
            idct4x4(ac_dq, r2);
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    int v = pred[(br*4 + r)*16 + (bc*4 + c)] +
                            ((r2[r*4 + c] + 32) >> 6);
                    recon[(br*4 + r)*16 + (bc*4 + c)] = (u8)clip_u8(v);
                }
            }
        }
    }

    /* PSNR. */
    long sse = 0;
    int max_err = 0;
    for (int i = 0; i < 256; i++) {
        int err = (int)src[i] - (int)recon[i];
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
        sse += err * err;
    }
    double mse = sse / 256.0;
    double psnr = (mse > 0) ? 10 * log10(255.0*255.0 / mse) : 99.0;
    printf("16x16 PSNR = %.2f dB, max err = %d\n", psnr, max_err);

    /* Show a corner of recon for visual inspection. */
    printf("src[0..15]:   ");
    for (int j = 0; j < 16; j++) printf("%3d ", src[j]);
    printf("\nrecon[0..15]: ");
    for (int j = 0; j < 16; j++) printf("%3d ", recon[j]);
    printf("\n");
}

int main(void)
{
    /* Test 1 — AC only at typical QPs. */
    for (int qp = 0; qp <= 42; qp += 6)
        test_ac_roundtrip(qp);

    /* Test 2 — DC handling at multiple QPs. Constant residual exercises only
     * the DC path; AC levels should all be zero. */
    test_i16x16_roundtrip(0,  100);
    test_i16x16_roundtrip(12, 100);
    test_i16x16_roundtrip(24, 100);
    test_i16x16_roundtrip(30, 100);
    test_i16x16_roundtrip(36, 100);

    /* Test 3 — realistic encoder scenario. */
    for (int qp = 0; qp <= 42; qp += 6)
        test_i16x16_with_pred(qp);

    return 0;
}
