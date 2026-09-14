/* gen_mb_residual_dec_vectors.c — golden vectors for the VHDL
 * mb_residual_dec_engine, the block sequencer.
 *
 * Each vector is a whole macroblock's residual, written with the same block
 * order and the same nC derivation src/encoder.c uses, then read back with
 * dec_mb_residual -- the routine the golden decoder itself uses, proven by
 * `make dec_test` to reconstruct real streams byte-exactly against ffmpeg.
 *
 * The round trip checks more than it looks. The nC of every block comes from
 * its neighbours' total_coeff, so if this generator's neighbour rule and the
 * decoder's disagree by one block, the two sides pick different coeff_token
 * tables and the coefficients come back wrong. A round trip that holds is
 * therefore evidence about the nC bookkeeping, not just about the bits.
 *
 * Coefficients are emitted as the decoder returns them: zigzag, with the
 * I_16x16 and chroma AC shift already applied, so every block is a full
 * 16-coefficient vector. The blocks fed to the encoder are the unshifted
 * 15-coefficient ones. Writing the expectation by hand would have recorded
 * the wrong one.
 *
 * Output, build/mb_residual_dec_vectors.txt, one macroblock per line:
 *   <is_i4x4> <cbp_luma> <cbp_chroma> <avail_top> <avail_left>
 *   <nc_top x4> <nc_left x4> <ncu_top x2> <ncu_left x2> <ncv_top x2> <ncv_left x2>
 *   <nbits> <nbytes> <bytes...>
 *   <luma_dc x16>
 *   <luma x16 blocks of 16, raster block order>
 *   <chroma_dc x2 blocks of 4>
 *   <chroma_ac x8 blocks of 16, U then V, raster>
 *   <nc_out x16> <ncu_out x4> <ncv_out x4>
 */

#include "types.h"
#include "bitstream.h"
#include "cavlc.h"
#include "dec/decoder.h"
#include <stdio.h>
#include <string.h>

static const int scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
static const int scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};

static u32 rs = 0x13579BDFu;
static u32 rnd(void)
{
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
}

static int n_emitted = 0;

/* A block with `nz` nonzeros among the first n, magnitudes up to max_mag. */
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

/* Write one macroblock's residual as the encoder does, read it back with the
 * golden decoder, and emit the pair. */
static int emit(FILE *f, int is4, int cbpl, int cbpc, int at, int al,
                const int nc_top[4], const int nc_left[4],
                const int ncu_top[2], const int ncu_left[2],
                const int ncv_top[2], const int ncv_left[2],
                i16 ldc[16], i16 luma[16][16],
                i16 cdc[2][4], i16 cac[2][4][16])
{
    u8 buf[2048];
    bitstream_t bs;
    bitreader_t br;
    mb_residual_t r;
    int nbits, nbytes, s, i, k, comp;
    int nc_enc[16];               /* total_coeff as the encoder computes it */
    int ncc_enc[2][4];

    memset(buf, 0, sizeof buf);
    bs_init(&bs, buf, sizeof buf);
    memset(nc_enc, 0, sizeof nc_enc);
    memset(ncc_enc, 0, sizeof ncc_enc);

    if (!is4) {
        int nA = al ? nc_left[0] : 0;
        int nB = at ? nc_top[0]  : 0;
        if (cavlc_encode_block(&bs, ldc, 16, BLK_LUMA_DC_16x16,
                               cavlc_compute_nC(nB, nA, at, al)) < 0) return 0;
    }

    for (s = 0; s < 16; s++) {
        int bcr = scan_br[s], bcc = scan_bc[s];
        int pos = bcr * 4 + bcc;
        int coded = (cbpl >> (s / 4)) & 1;
        int n_coefs = is4 ? 16 : 15;
        int total = 0;
        if (coded) {
            int a_left = (bcc > 0) || al;
            int a_top  = (bcr > 0) || at;
            int nA = a_left ? ((bcc > 0) ? nc_enc[pos - 1] : nc_left[bcr]) : 0;
            int nB = a_top  ? ((bcr > 0) ? nc_enc[pos - 4] : nc_top[bcc])  : 0;
            if (cavlc_encode_block(&bs, luma[pos], n_coefs,
                                   is4 ? BLK_LUMA_FULL : BLK_LUMA_AC,
                                   cavlc_compute_nC(nB, nA, a_top, a_left)) < 0)
                return 0;
            for (i = 0; i < n_coefs; i++) if (luma[pos][i]) total++;
        }
        nc_enc[pos] = total;
    }

    if (cbpc) {
        for (comp = 0; comp < 2; comp++)
            if (cavlc_encode_block(&bs, cdc[comp], 4, BLK_CHROMA_DC, -1) < 0)
                return 0;
    }
    for (comp = 0; comp < 2; comp++) {
        const int *ntop  = comp ? ncv_top  : ncu_top;
        const int *nleft = comp ? ncv_left : ncu_left;
        for (i = 0; i < 4; i++) {
            int bcr = i >> 1, bcc = i & 1, total = 0;
            if (cbpc == 2) {
                int a_left = (bcc > 0) || al;
                int a_top  = (bcr > 0) || at;
                int nA = a_left ? ((bcc > 0) ? ncc_enc[comp][i - 1] : nleft[bcr]) : 0;
                int nB = a_top  ? ((bcr > 0) ? ncc_enc[comp][i - 2] : ntop[bcc])  : 0;
                if (cavlc_encode_block(&bs, cac[comp][i], 15, BLK_CHROMA_AC,
                                       cavlc_compute_nC(nB, nA, a_top, a_left)) < 0)
                    return 0;
                for (k = 0; k < 15; k++) if (cac[comp][i][k]) total++;
            }
            ncc_enc[comp][i] = total;
        }
    }

    nbits = bs.byte_pos * 8 + bs.n_in_cur;
    if (nbits == 0) return 0;                 /* nothing coded; nothing to test */
    bs_put_bits(&bs, 0, 32);                  /* flush the accumulator */
    nbytes = (nbits + 7) / 8;

    memset(&r, 0, sizeof r);
    r.is_i4x4 = is4; r.cbp_luma = cbpl; r.cbp_chroma = cbpc;
    r.avail_top = at; r.avail_left = al;
    for (i = 0; i < 4; i++) { r.nc_top[i] = nc_top[i]; r.nc_left[i] = nc_left[i]; }
    for (i = 0; i < 2; i++) {
        r.ncu_top[i] = ncu_top[i]; r.ncu_left[i] = ncu_left[i];
        r.ncv_top[i] = ncv_top[i]; r.ncv_left[i] = ncv_left[i];
    }
    br_init(&br, buf, nbytes + 4);
    if (dec_mb_residual(&br, &r) != 0) {
        fprintf(stderr, "round trip FAILED (is4=%d cbp=%d/%d)\n", is4, cbpl, cbpc);
        return -1;
    }
    if (br.byte_pos * 8 + br.bit_in_byte != nbits) {
        fprintf(stderr, "round trip LENGTH %d, wrote %d (is4=%d cbp=%d/%d)\n",
                br.byte_pos * 8 + br.bit_in_byte, nbits, is4, cbpl, cbpc);
        return -1;
    }
    for (i = 0; i < 16; i++) if (r.nc_out[i] != nc_enc[i]) {
        fprintf(stderr, "round trip nC MISMATCH at luma %d\n", i);
        return -1;
    }
    for (comp = 0; comp < 2; comp++)
        for (i = 0; i < 4; i++) {
            int got = comp ? r.ncv_out[i] : r.ncu_out[i];
            if (got != ncc_enc[comp][i]) {
                fprintf(stderr, "round trip nC MISMATCH at chroma %d/%d\n", comp, i);
                return -1;
            }
        }

    fprintf(f, "%d %d %d %d %d", is4, cbpl, cbpc, at, al);
    for (i = 0; i < 4; i++) fprintf(f, " %d", nc_top[i]);
    for (i = 0; i < 4; i++) fprintf(f, " %d", nc_left[i]);
    for (i = 0; i < 2; i++) fprintf(f, " %d", ncu_top[i]);
    for (i = 0; i < 2; i++) fprintf(f, " %d", ncu_left[i]);
    for (i = 0; i < 2; i++) fprintf(f, " %d", ncv_top[i]);
    for (i = 0; i < 2; i++) fprintf(f, " %d", ncv_left[i]);
    fprintf(f, " %d %d", nbits, nbytes);
    for (i = 0; i < nbytes; i++) fprintf(f, " %d", buf[i]);
    for (i = 0; i < 16; i++) fprintf(f, " %d", r.luma_dc[i]);
    for (s = 0; s < 16; s++)
        for (i = 0; i < 16; i++) fprintf(f, " %d", r.luma[s][i]);
    for (comp = 0; comp < 2; comp++)
        for (i = 0; i < 4; i++) fprintf(f, " %d", r.chroma_dc[comp][i]);
    for (comp = 0; comp < 2; comp++)
        for (s = 0; s < 4; s++)
            for (i = 0; i < 16; i++) fprintf(f, " %d", r.chroma_ac[comp][s][i]);
    for (i = 0; i < 16; i++) fprintf(f, " %d", r.nc_out[i]);
    for (i = 0; i < 4; i++) fprintf(f, " %d", r.ncu_out[i]);
    for (i = 0; i < 4; i++) fprintf(f, " %d", r.ncv_out[i]);
    fprintf(f, "\n");
    n_emitted++;
    return 0;
}

/* One macroblock's worth of coefficients and neighbour context. */
static int build(FILE *f, int is4, int cbpl, int cbpc, int at, int al,
                 int max_mag, int density)
{
    i16 ldc[16], luma[16][16], cdc[2][4], cac[2][4][16];
    int nc_top[4], nc_left[4], ncu_top[2], ncu_left[2], ncv_top[2], ncv_left[2];
    int i, comp, n_coefs = is4 ? 16 : 15;

    memset(ldc, 0, sizeof ldc);
    memset(luma, 0, sizeof luma);
    memset(cdc, 0, sizeof cdc);
    memset(cac, 0, sizeof cac);

    /* Neighbour counts are total_coeff values, so 0..16 for luma and 0..15
     * for chroma AC. An out-of-range one would silently pick a different
     * coeff_token table. */
    for (i = 0; i < 4; i++) {
        nc_top[i]  = at ? (int)(rnd() % 17) : 0;
        nc_left[i] = al ? (int)(rnd() % 17) : 0;
    }
    for (i = 0; i < 2; i++) {
        ncu_top[i]  = at ? (int)(rnd() % 16) : 0;
        ncu_left[i] = al ? (int)(rnd() % 16) : 0;
        ncv_top[i]  = at ? (int)(rnd() % 16) : 0;
        ncv_left[i] = al ? (int)(rnd() % 16) : 0;
    }

    if (!is4) make_block(ldc, 16, (int)(rnd() % 17), max_mag);
    for (i = 0; i < 16; i++)
        make_block(luma[i], n_coefs, (int)(rnd() % (u32)(density + 1)), max_mag);
    for (comp = 0; comp < 2; comp++) {
        make_block(cdc[comp], 4, (int)(rnd() % 5), max_mag);
        for (i = 0; i < 4; i++)
            make_block(cac[comp][i], 15, (int)(rnd() % (u32)(density + 1)), max_mag);
    }

    return emit(f, is4, cbpl, cbpc, at, al, nc_top, nc_left,
                ncu_top, ncu_left, ncv_top, ncv_left, ldc, luma, cdc, cac);
}

int main(void)
{
    FILE *f = fopen("build/mb_residual_dec_vectors.txt", "w");
    int cbpl, cbpc, at, al, rep, mag;

    if (!f) { perror("build/mb_residual_dec_vectors.txt"); return 2; }

    /* Every coded_block_pattern an I_4x4 macroblock can carry, at every
     * availability combination: the pattern decides which blocks are in the
     * stream at all, and availability decides where their nC comes from. */
    for (cbpl = 0; cbpl < 16; cbpl++)
        for (cbpc = 0; cbpc < 3; cbpc++)
            for (at = 0; at < 2; at++)
                for (al = 0; al < 2; al++)
                    if (build(f, 1, cbpl, cbpc, at, al, 8, 16) < 0) return 1;

    /* I_16x16: luma DC is always in the stream, the AC blocks carry 15
     * coefficients, and cbp_luma is all or nothing. */
    for (cbpl = 0; cbpl < 2; cbpl++)
        for (cbpc = 0; cbpc < 3; cbpc++)
            for (at = 0; at < 2; at++)
                for (al = 0; al < 2; al++)
                    for (rep = 0; rep < 4; rep++)
                        if (build(f, 0, cbpl ? 15 : 0, cbpc, at, al, 8, 15) < 0)
                            return 1;

    /* Magnitudes that drive the suffix-length escalation and the escape
     * prefixes, which only appear in dense high-energy blocks. */
    for (mag = 1; mag <= 2048; mag *= 8)
        for (rep = 0; rep < 60; rep++) {
            int is4 = (int)(rnd() & 1);
            if (build(f, is4, is4 ? (int)(rnd() % 16) : 15,
                      (int)(rnd() % 3), (int)(rnd() & 1), (int)(rnd() & 1),
                      mag, is4 ? 16 : 15) < 0) return 1;
        }

    /* Sparse macroblocks, where most blocks decode to nothing and the nC
     * chain is carried by zeros. */
    for (rep = 0; rep < 400; rep++) {
        int is4 = (int)(rnd() & 1);
        if (build(f, is4, is4 ? (int)(rnd() % 16) : ((rnd() & 1) ? 15 : 0),
                  (int)(rnd() % 3), (int)(rnd() & 1), (int)(rnd() & 1),
                  4, 2) < 0) return 1;
    }

    /* Random fill. */
    for (rep = 0; rep < 1200; rep++) {
        int is4 = (int)(rnd() & 1);
        if (build(f, is4, is4 ? (int)(rnd() % 16) : ((rnd() & 1) ? 15 : 0),
                  (int)(rnd() % 3), (int)(rnd() & 1), (int)(rnd() & 1),
                  1 + (int)(rnd() % 64), is4 ? 16 : 15) < 0) return 1;
    }

    fclose(f);
    fprintf(stderr, "wrote build/mb_residual_dec_vectors.txt: %d macroblocks\n",
            n_emitted);
    return 0;
}
