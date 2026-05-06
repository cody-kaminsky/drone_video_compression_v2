/* gen_cavlc_vectors.c
 *
 * Generates input/output test vectors for the VHDL CAVLC engine, using the
 * C reference's cavlc_encode_block as the golden model. Each vector covers
 * one residual block (one packet on the level-stream interface).
 *
 * Output format: two text files, easy to parse from VHDL.
 *
 *   build/cavlc_vectors_in.txt   — one vector per line:
 *     <block_type> <n_coefs> <nC> <level_0> <level_1> ... <level_15>
 *
 *   build/cavlc_vectors_out.txt  — one vector per line, matching index:
 *     <bit_count> <hex_byte_0> <hex_byte_1> ...
 *
 * The VHDL testbench reads both, drives the engine with the input,
 * compares the output bits.
 *
 * Build: this file is added to the Makefile as a separate utility binary.
 *   make gen_cavlc_vectors
 *   build/gen_cavlc_vectors > /dev/null   # writes the two files in build/
 */

#include "cavlc.h"
#include "bitstream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Generate vectors covering all (block_type, nC, total_coef) combinations
 * we expect the engine to see. For each, build a representative levels[]
 * array (random within a controlled magnitude range) and emit. */

static u8 scratch[64];

static void emit_vector(FILE *fin, FILE *fout,
                        int block_type, int n_coefs, int nC,
                        const i16 levels[16])
{
    /* Input line: block_type n_coefs nC levels[0..15] */
    fprintf(fin, "%d %d %d", block_type, n_coefs, nC);
    for (int k = 0; k < 16; k++) fprintf(fin, " %d", levels[k]);
    fprintf(fin, "\n");

    /* Run the C reference and capture the bytes. */
    bitstream_t bs;
    bs_init(&bs, scratch, sizeof scratch);
    cavlc_encode_block(&bs, levels, n_coefs, block_type, nC);
    int bit_count = bs.byte_pos * 8 + bs.n_in_cur;

    fprintf(fout, "%d", bit_count);
    int n_bytes = (bit_count + 7) / 8;
    /* Flush any leftover bits in bs->cur into the scratch tail so we can
     * dump them. bitstream_t leaves <8 bits in bs->cur until forced. */
    if (bs.n_in_cur > 0 && n_bytes > bs.byte_pos) {
        scratch[bs.byte_pos] = (u8)((bs.cur >> 24) & 0xFF);
    }
    for (int b = 0; b < n_bytes; b++) fprintf(fout, " %02X", scratch[b]);
    fprintf(fout, "\n");
}

/* xorshift PRNG so we get reproducible vectors without depending on rand(). */
static unsigned long xrng_state = 0x12345678ul;
static int xrng_next(int hi) {
    xrng_state ^= xrng_state << 13;
    xrng_state ^= xrng_state >> 17;
    xrng_state ^= xrng_state << 5;
    return (int)(xrng_state % (unsigned long)hi);
}

static int random_signed(int max_mag) {
    int m = xrng_next(max_mag) + 1;
    return (xrng_next(2) == 0) ? m : -m;
}

int main(void)
{
    FILE *fin  = fopen("build/cavlc_vectors_in.txt",  "w");
    FILE *fout = fopen("build/cavlc_vectors_out.txt", "w");
    if (!fin || !fout) { perror("open"); return 1; }

    int n = 0;

    /* Iterate over the relevant (block_type, n_coefs) shapes. */
    struct { int bt; int n_coefs; } shapes[] = {
        { BLK_LUMA_DC_16x16, 16 },
        { BLK_LUMA_AC,       15 },
        { BLK_LUMA_FULL,     16 },
        { BLK_CHROMA_DC,      4 },
        { BLK_CHROMA_AC,     15 },
    };
    int nshapes = sizeof shapes / sizeof shapes[0];

    /* For each shape: cover nC = 0..16 (or -1 sentinel for chroma DC),
     * a few TotalCoeff values, with random magnitudes. */
    for (int s = 0; s < nshapes; s++) {
        int nC_lo = (shapes[s].bt == BLK_CHROMA_DC) ? -1 : 0;
        int nC_hi = (shapes[s].bt == BLK_CHROMA_DC) ? -1 : 16;
        for (int nC = nC_lo; nC <= nC_hi; nC++) {
            for (int tc = 0; tc <= shapes[s].n_coefs; tc++) {
                /* All-zero block: special case, emit once with TC=0. */
                i16 levels[16];
                memset(levels, 0, sizeof levels);
                if (tc == 0) {
                    emit_vector(fin, fout, shapes[s].bt, shapes[s].n_coefs, nC, levels);
                    n++;
                    continue;
                }
                /* Place tc nonzero values in the LAST tc positions of the
                 * zigzag (most realistic — high-frequency coefs are sparse).
                 * Mix in a few trailing-ones cases. */
                for (int rep = 0; rep < 3; rep++) {
                    memset(levels, 0, sizeof levels);
                    int max_mag = (rep == 0) ? 1 : (rep == 1) ? 4 : 32;
                    int start = shapes[s].n_coefs - tc;
                    for (int i = 0; i < tc; i++)
                        levels[start + i] = (i16)random_signed(max_mag);
                    emit_vector(fin, fout, shapes[s].bt, shapes[s].n_coefs, nC, levels);
                    n++;
                }
            }
        }
    }

    fclose(fin);
    fclose(fout);
    fprintf(stderr, "wrote %d vectors\n", n);
    return 0;
}
