/* gen_dispatch_vectors.c — golden vectors for the VHDL cavlc_dispatch
 * (coefficient FIFO + N CAVLC engines + in-order merge). Builds a few
 * slice-like item streams (header fields, residual blocks, stop bit,
 * flush) and the byte stream the C reference produces for them.
 *
 *   build/dispatch_vectors_in.txt   one item per line
 *     F <len> <bits>                 raw field (value <= 8 bits, right-aligned)
 *     B <bt> <n_coefs> <nC> <l0..l15>   level packet (nC = 31 for chroma DC)
 *     X                              flush (end of group)
 *   build/dispatch_vectors_out.txt  one byte per line
 *     Y <byte>  /  L <byte>          L = last byte of a flush group
 */
#include "cavlc.h"
#include "bitstream.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned rng_state = 0xD15BA7C4;
static unsigned xorshift32(void)
{
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}

static i16 rlevel(void)
{
    int mag;
    switch (xorshift32() % 5) {
    case 0:  mag = 1; break;
    case 1:  mag = 1 + (int)(xorshift32() % 3); break;
    case 2:  mag = 1 + (int)(xorshift32() % 20); break;
    case 3:  mag = 1 + (int)(xorshift32() % 200); break;
    default: mag = 1 + (int)(xorshift32() % 2063); break;
    }
    return (i16)((xorshift32() & 1) ? -mag : mag);
}

static u8 buf[1 << 16];

int main(void)
{
    FILE *fi = fopen("build/dispatch_vectors_in.txt", "w");
    FILE *fo = fopen("build/dispatch_vectors_out.txt", "w");
    if (!fi || !fo) { perror("build/dispatch_vectors_*.txt"); return 1; }
    static const block_type_t bts[] = {BLK_LUMA_FULL, BLK_LUMA_AC, BLK_LUMA_DC_16x16, BLK_CHROMA_AC, BLK_CHROMA_DC};
    static const int ns[]  = {16, 15, 16, 15, 4};
    static const int nCs[] = {0, 1, 2, 3, 4, 5, 7, 8, 16};
    bitstream_t bs;
    bs_init(&bs, buf, sizeof buf);
    int nblk = 0, nbytes = 0;
    for (int g = 0; g < 3; g++) {
        int start = bs.byte_pos;
        int mbs = 8;
        for (int m = 0; m < mbs; m++) {
            int nf = 1 + (int)(xorshift32() % 4);
            for (int i = 0; i < nf; i++) {
                int len = 1 + (int)(xorshift32() % 11);
                u32 bits = xorshift32() & ((1u << len) - 1u);
                bits &= 0xFF;                          /* field values are <= 8 bits; leading zeros implied */
                if (xorshift32() & 1) bits &= 0x3F;
                fprintf(fi, "F %d %u\n", len, bits);
                bs_put_bits(&bs, bits, len);
            }
            int nb = 1 + (int)(xorshift32() % 26);
            for (int b = 0; b < nb; b++) {
                int t = (int)(xorshift32() % 5);
                int n = ns[t];
                int nC = nCs[xorshift32() % 9];
                i16 lv[16] = {0};
                int nnz = (int)(xorshift32() % (n + 1));
                if ((xorshift32() % 4) == 0) nnz = 0;
                for (int k = 0; k < nnz; k++) lv[xorshift32() % n] = rlevel();
                fprintf(fi, "B %d %d %d", (int)bts[t], n, (bts[t] == BLK_CHROMA_DC) ? 31 : nC);
                for (int k = 0; k < 16; k++) fprintf(fi, " %d", (int)lv[k]);
                fprintf(fi, "\n");
                cavlc_encode_block(&bs, lv, n, bts[t], (bts[t] == BLK_CHROMA_DC) ? -1 : nC);
                nblk++;
            }
        }
        fprintf(fi, "F 1 1\n");
        bs_put_bits(&bs, 1, 1);
        fprintf(fi, "X\n");
        int end = bs_byte_count(&bs);
        for (int i = start; i < end; i++) {
            fprintf(fo, "%c %d\n", (i == end - 1) ? 'L' : 'Y', buf[i]);
            nbytes++;
        }
    }
    fclose(fi); fclose(fo);
    printf("wrote %d blocks, %d bytes (3 groups)\n", nblk, nbytes);
    return 0;
}
