/* gen_cavlc_cost_vectors.c — golden vectors for the VHDL cavlc_cost_engine
 * (the mode-decision bit estimator). Wraps cavlc_estimate_block_bits from
 * cavlc.c, approximations included: the hardware must rank candidates
 * exactly as the C reference does. Writes build/cavlc_cost_vectors.txt:
 *   M <bt> <n_coefs> <nC>        nC = 31 for chroma DC (packet convention)
 *   I <l0> .. <l15>
 *   O <bits>
 */
#include "types.h"
#include "cavlc.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned rng_state = 0x5EED1234;
static unsigned xorshift32(void)
{
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static int rnd(int range) { return (int)(xorshift32() % (2u * range + 1u)) - range; }

static int emit(FILE *f, block_type_t bt, int n, int nC, const i16 c[16])
{
    int bits = cavlc_estimate_block_bits(c, n, bt, (bt == BLK_CHROMA_DC) ? -1 : nC);
    fprintf(f, "M %d %d %d\nI", (int)bt, n, (bt == BLK_CHROMA_DC) ? 31 : nC);
    for (int i = 0; i < 16; i++) fprintf(f, " %d", (int)c[i]);
    fprintf(f, "\nO %d\n", bits);
    return 1;
}

/* Level magnitude distributions: 0 ones, 1 small, 2 medium, 3 large, 4 huge */
static i16 rlevel(int dist)
{
    int mag;
    switch (dist) {
    case 0:  mag = 1; break;
    case 1:  mag = 1 + (int)(xorshift32() % 3); break;
    case 2:  mag = 1 + (int)(xorshift32() % 20); break;
    case 3:  mag = 1 + (int)(xorshift32() % 200); break;
    default: mag = 1 + (int)(xorshift32() % 2063); break;
    }
    return (i16)((xorshift32() & 1) ? -mag : mag);
}

int main(void)
{
    FILE *f = fopen("build/cavlc_cost_vectors.txt", "w");
    if (!f) { perror("build/cavlc_cost_vectors.txt"); return 1; }
    static const block_type_t bts[] = {BLK_LUMA_AC, BLK_LUMA_FULL, BLK_LUMA_DC_16x16,
                                       BLK_CHROMA_AC, BLK_CHROMA_DC};
    static const int ns[] = {15, 16, 16, 15, 4};
    static const int nCs[] = {0, 1, 2, 3, 4, 7, 8, 16};
    int count = 0;
    for (int b = 0; b < 5; b++) {
        int n = ns[b];
        for (int q = 0; q < 8; q++) {
            int nC = nCs[q];
            if (bts[b] == BLK_CHROMA_DC && q > 0) break;
            i16 c[16];
            /* empty */
            for (int i = 0; i < 16; i++) c[i] = 0;
            count += emit(f, bts[b], n, nC, c);
            /* every nonzero count 1..n with all-ones at the top, and with
             * structured trailing-ones counts */
            for (int tc = 1; tc <= n; tc++) {
                for (int dist = 0; dist < 5; dist++) {
                    for (int rep = 0; rep < 2; rep++) {
                        for (int i = 0; i < 16; i++) c[i] = 0;
                        /* choose tc distinct positions */
                        int placed = 0;
                        while (placed < tc) {
                            int p = (int)(xorshift32() % (unsigned)n);
                            if (c[p] == 0) { c[p] = rlevel(dist); placed++; }
                        }
                        /* force 0..3 trailing ones at the top sometimes */
                        int t1 = (int)(xorshift32() % 4);
                        int seen = 0;
                        for (int i = n - 1; i >= 0 && seen < t1; i--)
                            if (c[i] != 0) { c[i] = (c[i] < 0) ? -1 : 1; seen++; }
                        count += emit(f, bts[b], n, nC, c);
                    }
                }
            }
            /* dense random */
            for (int rep = 0; rep < 8; rep++) {
                for (int i = 0; i < 16; i++) c[i] = (i < n) ? (i16)rnd(6) : 0;
                count += emit(f, bts[b], n, nC, c);
            }
        }
    }
    fclose(f);
    printf("wrote %d cavlc cost vectors to build/cavlc_cost_vectors.txt\n", count);
    return 0;
}
