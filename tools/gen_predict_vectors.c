/* gen_predict_vectors.c — golden vectors for the three VHDL intra
 * prediction engines. Links against intra.c. Writes:
 *
 * build/predict4x4_vectors.txt
 *   M <mode> <avail_top> <avail_left> <avail_tl>
 *   I <top0..7> <left0..3> <tl>                       (13 values)
 *   O <p0..p15>                                        (raster 4x4)
 *
 * build/predict16_vectors.txt
 *   M <mode> <avail_top> <avail_left> <avail_tl> <blk>  blk 0..15 raster
 *   I <top0..15> <left0..15> <tl>                      (33 values)
 *   O <p0..p15>   the 4x4 block (blk%4, blk/4) of the 16x16 prediction
 *
 * build/predict_chroma_vectors.txt
 *   M <mode> <avail_top> <avail_left> <avail_tl> <blk>  blk 0..3 (quadrant)
 *   I <top0..7> <left0..7> <tl>                        (17 values)
 *   O <p0..p15>   the 4x4 quadrant (blk%2, blk/2) of the 8x8 prediction
 *
 * Availability flags are exercised in all combinations for every mode;
 * the C functions define a fallback for every combination.
 */
#include "types.h"
#include "intra.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned rng_state = 0x1234ABCD;
static unsigned xorshift32(void)
{
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static u8 rpix(void) { return (u8)(xorshift32() & 0xFF); }

/* Neighbour patterns: 0 random, 1 flat, 2 ramp, 3 extremes */
static void fill(u8 *p, int n, int pattern)
{
    for (int i = 0; i < n; i++) {
        switch (pattern) {
        case 1:  p[i] = 128; break;
        case 2:  p[i] = (u8)(i * 255 / (n > 1 ? n - 1 : 1)); break;
        case 3:  p[i] = (i & 1) ? 255 : 0; break;
        default: p[i] = rpix(); break;
        }
    }
}

static int gen4x4(void)
{
    FILE *f = fopen("build/predict4x4_vectors.txt", "w");
    if (!f) { perror("build/predict4x4_vectors.txt"); return -1; }
    int count = 0;
    for (int pat = 0; pat < 4; pat++)
    for (int rep = 0; rep < (pat == 0 ? 6 : 1); rep++)
    for (int av = 0; av < 8; av++)
    for (int mode = 0; mode < 9; mode++) {
        u8 top[8], left[4], tl, pred[16];
        fill(top, 8, pat); fill(left, 4, pat); tl = (pat == 0) ? rpix() : (u8)(pat == 3 ? 255 : 128);
        int at = av & 1, al = (av >> 1) & 1, atl = (av >> 2) & 1;
        predict_4x4(mode, top, left, tl, at, al, atl, pred);
        fprintf(f, "M %d %d %d %d\nI", mode, at, al, atl);
        for (int i = 0; i < 8; i++) fprintf(f, " %d", top[i]);
        for (int i = 0; i < 4; i++) fprintf(f, " %d", left[i]);
        fprintf(f, " %d\nO", tl);
        for (int i = 0; i < 16; i++) fprintf(f, " %d", pred[i]);
        fprintf(f, "\n");
        count++;
    }
    fclose(f);
    return count;
}

static int gen16(void)
{
    FILE *f = fopen("build/predict16_vectors.txt", "w");
    if (!f) { perror("build/predict16_vectors.txt"); return -1; }
    int count = 0;
    for (int pat = 0; pat < 4; pat++)
    for (int rep = 0; rep < (pat == 0 ? 3 : 1); rep++)
    for (int av = 0; av < 8; av++)
    for (int mode = 0; mode < 4; mode++) {
        u8 top[16], left[16], tl, pred[256];
        fill(top, 16, pat); fill(left, 16, pat); tl = (pat == 0) ? rpix() : (u8)(pat == 3 ? 255 : 128);
        int at = av & 1, al = (av >> 1) & 1, atl = (av >> 2) & 1;
        predict_16x16(mode, top, left, tl, at, al, atl, pred);
        for (int blk = 0; blk < 16; blk++) {
            int bx = blk % 4, by = blk / 4;
            fprintf(f, "M %d %d %d %d %d\nI", mode, at, al, atl, blk);
            for (int i = 0; i < 16; i++) fprintf(f, " %d", top[i]);
            for (int i = 0; i < 16; i++) fprintf(f, " %d", left[i]);
            fprintf(f, " %d\nO", tl);
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    fprintf(f, " %d", pred[(by * 4 + r) * 16 + bx * 4 + c]);
            fprintf(f, "\n");
            count++;
        }
    }
    fclose(f);
    return count;
}

static int genchroma(void)
{
    FILE *f = fopen("build/predict_chroma_vectors.txt", "w");
    if (!f) { perror("build/predict_chroma_vectors.txt"); return -1; }
    int count = 0;
    for (int pat = 0; pat < 4; pat++)
    for (int rep = 0; rep < (pat == 0 ? 6 : 1); rep++)
    for (int av = 0; av < 8; av++)
    for (int mode = 0; mode < 4; mode++) {
        u8 top[8], left[8], tl, pred[64];
        fill(top, 8, pat); fill(left, 8, pat); tl = (pat == 0) ? rpix() : (u8)(pat == 3 ? 255 : 128);
        int at = av & 1, al = (av >> 1) & 1, atl = (av >> 2) & 1;
        predict_chroma_8x8(mode, top, left, tl, at, al, atl, pred);
        for (int blk = 0; blk < 4; blk++) {
            int bx = blk % 2, by = blk / 2;
            fprintf(f, "M %d %d %d %d %d\nI", mode, at, al, atl, blk);
            for (int i = 0; i < 8; i++) fprintf(f, " %d", top[i]);
            for (int i = 0; i < 8; i++) fprintf(f, " %d", left[i]);
            fprintf(f, " %d\nO", tl);
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    fprintf(f, " %d", pred[(by * 4 + r) * 8 + bx * 4 + c]);
            fprintf(f, "\n");
            count++;
        }
    }
    fclose(f);
    return count;
}

int main(void)
{
    int a = gen4x4(), b = gen16(), c = genchroma();
    if (a < 0 || b < 0 || c < 0) return 1;
    printf("wrote %d predict4x4, %d predict16, %d predict_chroma vectors\n", a, b, c);
    return 0;
}
