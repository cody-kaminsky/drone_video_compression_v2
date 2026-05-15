/* gen_transform_vectors.c — generate golden test vectors for the VHDL
 * transform engine testbench.
 *
 * Links against the real transform.c so vectors are byte-exact with the
 * C reference.  Writes build/transform_vectors.txt consumed by
 * transform_engine_tb.vhd.
 *
 * Build:  (done automatically by `make transform_vectors`)
 *   gcc -std=c99 -O2 -Isrc -o build/gen_transform_vectors \
 *       tools/gen_transform_vectors.c build/transform.o -lm
 *
 * Vector file format (one test per three lines):
 *   M <mode>                                -- 0..5
 *   I <v0> <v1> ... <v15>                   -- input  (decimal, 32-bit signed)
 *   O <v0> <v1> ... <v15>                   -- output (decimal, 32-bit signed)
 *
 * Modes:
 *   0  dct4x4          i16 in → i16 out  (sign-extended to i32 in file)
 *   1  idct4x4         i32 in → i32 out
 *   2  hadamard4x4     i16 in → i32 out
 *   3  ihadamard4x4    i32 in → i32 out
 *   4  hadamard2x2     i16 in → i16 out  (only indices 0..3 used)
 *   5  ihadamard2x2    i32 in → i32 out  (only indices 0..3 used)
 */
#include "types.h"
#include "transform.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NUM_RANDOM_TESTS  64

static unsigned rng_state = 0xDEADBEEF;

static unsigned xorshift32(void)
{
    unsigned x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static i16 rand_i16(int range)
{
    return (i16)((int)(xorshift32() % (2 * range + 1)) - range);
}

static i32 rand_i32(int range)
{
    return (i32)((int)(xorshift32() % (2 * range + 1)) - range);
}

static void write_test(FILE *f, int mode, const i32 in[16], const i32 out[16],
                        int count)
{
    fprintf(f, "M %d\n", mode);
    fprintf(f, "I");
    for (int i = 0; i < count; i++)
        fprintf(f, " %d", (int)in[i]);
    for (int i = count; i < 16; i++)
        fprintf(f, " 0");
    fprintf(f, "\n");
    fprintf(f, "O");
    for (int i = 0; i < count; i++)
        fprintf(f, " %d", (int)out[i]);
    for (int i = count; i < 16; i++)
        fprintf(f, " 0");
    fprintf(f, "\n");
}

static void gen_dct4x4(FILE *f, const i16 in16[16])
{
    i16 out16[16];
    i32 in32[16], out32[16];
    dct4x4(in16, out16);
    for (int i = 0; i < 16; i++) {
        in32[i]  = in16[i];
        out32[i] = out16[i];
    }
    write_test(f, 0, in32, out32, 16);
}

static void gen_idct4x4(FILE *f, const i32 in32[16])
{
    i32 out32[16];
    idct4x4(in32, out32);
    write_test(f, 1, in32, out32, 16);
}

static void gen_hadamard4x4(FILE *f, const i16 in16[16])
{
    i32 out32[16], in32[16];
    hadamard4x4(in16, out32);
    for (int i = 0; i < 16; i++)
        in32[i] = in16[i];
    write_test(f, 2, in32, out32, 16);
}

static void gen_ihadamard4x4(FILE *f, const i32 in32[16])
{
    i32 out32[16];
    ihadamard4x4(in32, out32);
    write_test(f, 3, in32, out32, 16);
}

static void gen_hadamard2x2(FILE *f, const i16 in16[4])
{
    i16 out16[4];
    i32 in32[16] = {0}, out32[16] = {0};
    hadamard2x2(in16, out16);
    for (int i = 0; i < 4; i++) {
        in32[i]  = in16[i];
        out32[i] = out16[i];
    }
    write_test(f, 4, in32, out32, 4);
}

static void gen_ihadamard2x2(FILE *f, const i32 in4[4])
{
    i32 out4[4];
    i32 in32[16] = {0}, out32[16] = {0};
    ihadamard2x2(in4, out4);
    for (int i = 0; i < 4; i++) {
        in32[i]  = in4[i];
        out32[i] = out4[i];
    }
    write_test(f, 5, in32, out32, 4);
}

int main(void)
{
    FILE *f = fopen("build/transform_vectors.txt", "w");
    if (!f) { perror("build/transform_vectors.txt"); return 1; }

    int count = 0;

    /* --- Fixed corner-case vectors --- */

    /* All zeros */
    {
        i16 z16[16] = {0};
        i32 z32[16] = {0};
        gen_dct4x4(f, z16);       count++;
        gen_idct4x4(f, z32);      count++;
        gen_hadamard4x4(f, z16);  count++;
        gen_ihadamard4x4(f, z32); count++;
    }

    /* All ones */
    {
        i16 o16[16]; i32 o32[16];
        for (int i = 0; i < 16; i++) { o16[i] = 1; o32[i] = 1; }
        gen_dct4x4(f, o16);       count++;
        gen_idct4x4(f, o32);      count++;
        gen_hadamard4x4(f, o16);  count++;
        gen_ihadamard4x4(f, o32); count++;
    }

    /* DC constant (typical pixel residual) */
    {
        i16 dc16[16]; i32 dc32[16];
        for (int i = 0; i < 16; i++) { dc16[i] = 100; dc32[i] = 100; }
        gen_dct4x4(f, dc16);       count++;
        gen_hadamard4x4(f, dc16);  count++;
    }

    /* Identity-like: single nonzero element */
    {
        i16 s16[16] = {0}; s16[0] = 255;
        i32 s32[16] = {0}; s32[0] = 255;
        gen_dct4x4(f, s16);       count++;
        gen_idct4x4(f, s32);      count++;
        gen_hadamard4x4(f, s16);  count++;
        gen_ihadamard4x4(f, s32); count++;
    }

    /* Alternating pattern */
    {
        i16 a16[16]; i32 a32[16];
        for (int i = 0; i < 16; i++) {
            a16[i] = (i % 2 == 0) ? 50 : -50;
            a32[i] = a16[i];
        }
        gen_dct4x4(f, a16);       count++;
        gen_idct4x4(f, a32);      count++;
        gen_hadamard4x4(f, a16);  count++;
        gen_ihadamard4x4(f, a32); count++;
    }

    /* Max positive i16 */
    {
        i16 m16[16]; i32 m32[16];
        for (int i = 0; i < 16; i++) { m16[i] = 32767; m32[i] = 32767; }
        gen_dct4x4(f, m16);       count++;
        gen_hadamard4x4(f, m16);  count++;
    }

    /* Max negative i16 */
    {
        i16 m16[16]; i32 m32[16];
        for (int i = 0; i < 16; i++) { m16[i] = -32768; m32[i] = -32768; }
        gen_dct4x4(f, m16);       count++;
        gen_hadamard4x4(f, m16);  count++;
    }

    /* 2x2 corner cases */
    {
        i16 z4[4] = {0};
        i32 z4_32[4] = {0};
        gen_hadamard2x2(f, z4);    count++;
        gen_ihadamard2x2(f, z4_32); count++;

        i16 h4[4] = {100, -50, 30, -20};
        i32 h4_32[4] = {100, -50, 30, -20};
        gen_hadamard2x2(f, h4);    count++;
        gen_ihadamard2x2(f, h4_32); count++;
    }

    /* Round-trip vectors: dct then idct (validates the pair) */
    {
        i16 res[16] = {50,-10,5,-3, -20,30,-15,10, 15,-20,10,-5, -5,10,-8,12};
        i16 dct_out[16];
        dct4x4(res, dct_out);
        i32 dct_i32[16];
        for (int i = 0; i < 16; i++) dct_i32[i] = dct_out[i];
        gen_dct4x4(f, res);         count++;
        gen_idct4x4(f, dct_i32);    count++;
    }

    /* Round-trip: hadamard4x4 then ihadamard4x4 */
    {
        i16 dc[16] = {400,400,400,400, 400,400,400,400,
                      400,400,400,400, 400,400,400,400};
        i32 had_out[16];
        hadamard4x4(dc, had_out);
        i32 in32[16];
        for (int i = 0; i < 16; i++) in32[i] = dc[i];
        gen_hadamard4x4(f, dc);      count++;
        gen_ihadamard4x4(f, had_out); count++;
    }

    /* --- Random vectors --- */
    for (int t = 0; t < NUM_RANDOM_TESTS; t++) {
        /* dct4x4: residuals typically in [-255, 255] */
        {
            i16 r[16];
            for (int i = 0; i < 16; i++) r[i] = rand_i16(255);
            gen_dct4x4(f, r);  count++;
        }
        /* idct4x4: dequantized coefficients, wider range */
        {
            i32 r[16];
            for (int i = 0; i < 16; i++) r[i] = rand_i32(2048);
            gen_idct4x4(f, r); count++;
        }
        /* hadamard4x4: DC values from DCT, range ~[-4096, 4096] */
        {
            i16 r[16];
            for (int i = 0; i < 16; i++) r[i] = rand_i16(4096);
            gen_hadamard4x4(f, r);  count++;
        }
        /* ihadamard4x4 */
        {
            i32 r[16];
            for (int i = 0; i < 16; i++) r[i] = rand_i32(8192);
            gen_ihadamard4x4(f, r); count++;
        }
        /* hadamard2x2 */
        {
            i16 r[4];
            for (int i = 0; i < 4; i++) r[i] = rand_i16(2048);
            gen_hadamard2x2(f, r);  count++;
        }
        /* ihadamard2x2 */
        {
            i32 r[4];
            for (int i = 0; i < 4; i++) r[i] = rand_i32(4096);
            gen_ihadamard2x2(f, r); count++;
        }
    }

    fclose(f);
    printf("wrote %d test vectors to build/transform_vectors.txt\n", count);
    return 0;
}
