/* gen_cavlc_dec_vectors.c — golden vectors for the VHDL cavlc_dec_engine.
 *
 * Every vector is a real encode/decode round trip: a block of coefficients is
 * encoded with cavlc_encode_block, then decoded with cavlc_decode_block, and
 * the decoded result is what the RTL must reproduce from the same bits. That
 * means the golden is the C decoder which has already reconstructed real
 * streams byte-exactly at every QP from 10 to 51 -- not a hand-written
 * expectation that could be wrong in the same way the RTL is.
 *
 * Coverage is chosen for the cases that break CAVLC decoders rather than for
 * volume: empty blocks, all four trailing-one counts, the suffix-length
 * escalation that only appears with large magnitudes, the 14- and 15-prefix
 * escape codes, every block type, and the nC ranges that select each
 * coeff_token sub-table including the nC >= 8 fixed-length path.
 *
 * Output, build/cavlc_dec_vectors.txt, one vector per line:
 *   <nC> <btype> <n_coefs> <nbits> <nbytes> <bytes...> <total_coeff> <16 coefs>
 * Bytes are decimal, coefficients decimal and signed.
 */

#include "cavlc.h"
#include "bitstream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static u32 rs = 0x1234567u;
static u32 rnd(void)
{
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
}

static int n_emitted = 0;

/* Encode coefs, decode them back, and emit the pair if the round trip held.
 * A round trip that fails here is a C-side bug and should stop the run: it
 * would otherwise become an RTL "mismatch" against a golden that is itself
 * wrong. */
static int emit(FILE *f, const i16 *coefs, int n_coefs, block_type_t bt, int nC)
{
    u8 buf[64];
    bitstream_t bs;
    bitreader_t br;
    i16 back[16];
    int nbits, nbytes, i, tc = 0;

    memset(buf, 0, sizeof buf);
    bs_init(&bs, buf, sizeof buf);
    nbits = cavlc_encode_block(&bs, coefs, n_coefs, bt, nC);
    if (nbits <= 0) return 0;
    bs_put_bits(&bs, 0, 32);              /* flush the accumulator */
    nbytes = (nbits + 7) / 8;

    memset(back, 0, sizeof back);
    br_init(&br, buf, nbytes + 4);
    if (cavlc_decode_block(&br, back, n_coefs, bt, nC) != 0) {
        fprintf(stderr, "round trip FAILED to decode (nC=%d bt=%d n=%d)\n",
                nC, (int)bt, n_coefs);
        return -1;
    }
    for (i = 0; i < n_coefs; i++) {
        if (back[i] != coefs[i]) {
            fprintf(stderr, "round trip MISMATCH at %d (nC=%d bt=%d)\n",
                    i, nC, (int)bt);
            return -1;
        }
        if (coefs[i]) tc++;
    }

    fprintf(f, "%d %d %d %d %d", nC, (int)bt, n_coefs, nbits, nbytes);
    for (i = 0; i < nbytes; i++) fprintf(f, " %d", buf[i]);
    fprintf(f, " %d", tc);
    for (i = 0; i < 16; i++) fprintf(f, " %d", i < n_coefs ? back[i] : 0);
    fprintf(f, "\n");
    n_emitted++;
    return 0;
}

/* A block with `nz` nonzeros, magnitudes drawn up to max_mag, placed so that
 * runs of zeros appear between them. */
static void make_block(i16 *c, int n, int nz, int max_mag)
{
    int i, placed = 0;
    memset(c, 0, sizeof(i16) * 16);
    for (i = 0; i < n && placed < nz; i++) {
        if ((int)(rnd() % (u32)(n - i)) < nz - placed) {
            int mag = 1 + (int)(rnd() % (u32)max_mag);
            c[i] = (i16)((rnd() & 1) ? -mag : mag);
            placed++;
        }
    }
}

int main(void)
{
    FILE *f = fopen("build/cavlc_dec_vectors.txt", "w");
    i16 c[16];
    int nC, nz, mag, t, rep;
    static const block_type_t types[] = {
        BLK_LUMA_FULL, BLK_LUMA_AC, BLK_LUMA_DC_16x16, BLK_CHROMA_AC
    };
    static const int ncoefs_of[] = { 16, 15, 16, 15 };

    if (!f) { perror("build/cavlc_dec_vectors.txt"); return 2; }

    /* Empty blocks, every table. */
    memset(c, 0, sizeof c);
    for (t = 0; t < 4; t++)
        for (nC = 0; nC <= 8; nC += 2)
            if (emit(f, c, ncoefs_of[t], types[t], nC) < 0) return 1;
    if (emit(f, c, 4, BLK_CHROMA_DC, -1) < 0) return 1;

    /* Trailing ones: 1, 2 and 3, which take different paths. */
    for (t = 0; t < 4; t++) {
        for (nz = 1; nz <= 3; nz++) {
            int i;
            memset(c, 0, sizeof c);
            for (i = 0; i < nz; i++) c[i] = (i16)((i & 1) ? -1 : 1);
            for (nC = 0; nC <= 10; nC += 2)
                if (emit(f, c, ncoefs_of[t], types[t], nC) < 0) return 1;
        }
    }

    /* Magnitudes that drive the suffix-length escalation and the escape
     * prefixes. A block whose levels stay small never reaches either. */
    for (t = 0; t < 4; t++)
        for (mag = 1; mag <= 2048; mag *= 4)
            for (nz = 1; nz <= ncoefs_of[t]; nz += 3)
                for (nC = 0; nC <= 10; nC += 5) {
                    make_block(c, ncoefs_of[t], nz, mag);
                    if (emit(f, c, ncoefs_of[t], types[t], nC) < 0) return 1;
                }

    /* Chroma DC, its own coeff_token and total_zeros tables. */
    for (nz = 0; nz <= 4; nz++)
        for (mag = 1; mag <= 256; mag *= 4)
            for (rep = 0; rep < 4; rep++) {
                make_block(c, 4, nz, mag);
                if (emit(f, c, 4, BLK_CHROMA_DC, -1) < 0) return 1;
            }

    /* Random fill, to reach combinations the structured cases miss. */
    for (rep = 0; rep < 2000; rep++) {
        t  = (int)(rnd() % 4);
        nz = (int)(rnd() % (u32)(ncoefs_of[t] + 1));
        mag = 1 + (int)(rnd() % 64);
        nC = (int)(rnd() % 14);
        make_block(c, ncoefs_of[t], nz, mag);
        if (emit(f, c, ncoefs_of[t], types[t], nC) < 0) return 1;
    }

    fclose(f);
    fprintf(stderr, "wrote build/cavlc_dec_vectors.txt: %d vectors\n", n_emitted);
    return 0;
}
