/* gen_mb_header_dec_vectors.c — golden vectors for the VHDL
 * mb_header_dec_engine.
 *
 * Each vector is a real round trip. The header is written with the same
 * sequence of syntax elements src/encoder.c emits (and that the encoder's own
 * mb_header_engine is already validated against), then read back with
 * dec_mb_header -- the routine the golden decoder itself uses, proven by
 * `make dec_test` to reconstruct real streams byte-exactly against ffmpeg.
 * So neither side of the expectation is a fresh reading of the spec.
 *
 * The round trip is not an identity, and that is the point: an I_16x16
 * macroblock encodes "some luma is coded" as one bit inside mb_type and
 * decodes it as cbp_luma = 15. Writing the expectation by hand would have
 * recorded 1.
 *
 * Coverage is chosen for what breaks header parsers: every mb_type, all 48
 * coded_block_pattern values, both prediction-flag paths at every 4x4 block,
 * the four neighbour-availability combinations, and mb_qp_delta across its
 * whole range rather than the 0 a fixed-QP encoder always emits.
 *
 * Output, build/mb_header_dec_vectors.txt, one vector per line:
 *   <qp_in> <avail_top> <avail_left> <mode4_top x4> <mode4_left x4>
 *   <nbits> <nbytes> <bytes...>
 *   <is_i4x4> <mode16> <mode_chroma> <cbp_luma> <cbp_chroma> <has_residual>
 *   <qp_out> <modes4 x16 raster>
 */

#include "types.h"
#include "bitstream.h"
#include "cavlc_tables.h"
#include "dec/decoder.h"
#include <stdio.h>
#include <string.h>

static const int scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
static const int scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};

static u32 rs = 0x5EED1234u;
static u32 rnd(void)
{
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
}

static int n_emitted = 0;

/* Write one header exactly as the encoder does, read it back with the golden
 * decoder, and emit the pair. A round trip that does not hold is a C-side
 * bug and stops the run: it would otherwise become an RTL "mismatch" against
 * a golden that is itself wrong. */
static int emit(FILE *f, int qp_in, int is4, int m16, int mc,
                const int modes4[16], int cbpl, int cbpc, int delta,
                int at, int al, const int m4t[4], const int m4l[4])
{
    u8 buf[64];
    bitstream_t bs;
    bitreader_t br;
    mb_header_t h;
    int nbits, nbytes, i, has_res;

    memset(buf, 0, sizeof buf);
    bs_init(&bs, buf, sizeof buf);

    if (is4) {
        bs_put_ue(&bs, 0);
        for (i = 0; i < 16; i++) {
            int r = scan_br[i], c = scan_bc[i];
            int actual = modes4[r * 4 + c];
            int mt, ml, tok, lok, pm;
            if (r > 0) { mt = modes4[(r - 1) * 4 + c]; tok = 1; }
            else       { mt = m4t[c];                  tok = at; }
            if (c > 0) { ml = modes4[r * 4 + c - 1];   lok = 1; }
            else       { ml = m4l[r];                  lok = al; }
            pm = (tok && lok) ? (mt < ml ? mt : ml) : 2;
            if (actual == pm) {
                bs_put_bits(&bs, 1, 1);
            } else {
                bs_put_bits(&bs, 0, 1);
                bs_put_bits(&bs, (u32)((actual < pm) ? actual : actual - 1), 3);
            }
        }
        bs_put_ue(&bs, (u32)mc);
        bs_put_ue(&bs, cbp_intra_to_codenum[(cbpl & 0xF) | (cbpc << 4)]);
        has_res = (cbpl || cbpc);
    } else {
        bs_put_ue(&bs, (u32)(1 + m16 + 4 * cbpc + 12 * (cbpl ? 1 : 0)));
        bs_put_ue(&bs, (u32)mc);
        has_res = 1;
    }
    if (has_res) bs_put_se(&bs, delta);

    nbits  = bs.byte_pos * 8 + bs.n_in_cur;
    bs_put_bits(&bs, 0, 32);              /* flush the accumulator */
    nbytes = (nbits + 7) / 8;

    memset(&h, 0, sizeof h);
    h.qp_in = qp_in; h.avail_top = at; h.avail_left = al;
    for (i = 0; i < 4; i++) { h.mode4_top[i] = m4t[i]; h.mode4_left[i] = m4l[i]; }
    br_init(&br, buf, nbytes + 4);
    if (dec_mb_header(&br, &h) != 0) {
        fprintf(stderr, "round trip FAILED to decode (is4=%d m16=%d cbp=%d/%d)\n",
                is4, m16, cbpl, cbpc);
        return -1;
    }
    if ((br.byte_pos * 8 + br.bit_in_byte) != nbits) {
        fprintf(stderr, "round trip LENGTH %d, wrote %d (is4=%d)\n",
                (br.byte_pos * 8 + br.bit_in_byte), nbits, is4);
        return -1;
    }
    if (h.is_i4x4 != is4 || h.mode_chroma != mc ||
        (is4 && h.cbp_luma != cbpl) || h.cbp_chroma != cbpc ||
        (!is4 && h.mode16 != m16)) {
        fprintf(stderr, "round trip MISMATCH (is4=%d m16=%d cbp=%d/%d)\n",
                is4, m16, cbpl, cbpc);
        return -1;
    }
    if (is4) {
        for (i = 0; i < 16; i++) if (h.modes4[i] != modes4[i]) {
            fprintf(stderr, "round trip MODE MISMATCH at %d\n", i);
            return -1;
        }
    }

    fprintf(f, "%d %d %d", qp_in, at, al);
    for (i = 0; i < 4; i++) fprintf(f, " %d", m4t[i]);
    for (i = 0; i < 4; i++) fprintf(f, " %d", m4l[i]);
    fprintf(f, " %d %d", nbits, nbytes);
    for (i = 0; i < nbytes; i++) fprintf(f, " %d", buf[i]);
    fprintf(f, " %d %d %d %d %d %d %d",
            h.is_i4x4, h.mode16, h.mode_chroma, h.cbp_luma, h.cbp_chroma,
            h.has_residual, h.qp_out);
    for (i = 0; i < 16; i++) fprintf(f, " %d", h.modes4[i]);
    fprintf(f, "\n");
    n_emitted++;
    return 0;
}

/* A delta that keeps QP inside 0..51, which is all a sane encoder emits, but
 * drawn across the whole legal range rather than left at the 0 a fixed-QP
 * encoder always writes. */
static int pick_delta(int qp_in)
{
    int lo = -qp_in, hi = 51 - qp_in;
    if (lo < -26) lo = -26;
    if (hi > 25)  hi = 25;
    return lo + (int)(rnd() % (u32)(hi - lo + 1));
}

int main(void)
{
    FILE *f = fopen("build/mb_header_dec_vectors.txt", "w");
    int modes4[16], m4t[4], m4l[4];
    int i, s, qp, m16, mc, cbp, at, al, rep;

    if (!f) { perror("build/mb_header_dec_vectors.txt"); return 2; }

    /* Every I_16x16 mb_type, at both availability extremes. */
    for (i = 0; i < 16; i++) modes4[i] = 2;
    for (i = 0; i < 4; i++) { m4t[i] = 2; m4l[i] = 2; }
    for (m16 = 0; m16 < 4; m16++)
        for (cbp = 0; cbp < 3; cbp++)
            for (i = 0; i < 2; i++)
                for (mc = 0; mc < 4; mc++) {
                    qp = (int)(rnd() % 52);
                    if (emit(f, qp, 0, m16, mc, modes4, i, cbp,
                             pick_delta(qp), 1, 1, m4t, m4l) < 0) return 1;
                }

    /* Every coded_block_pattern an I_4x4 macroblock can carry. */
    for (cbp = 0; cbp < 48; cbp++) {
        for (i = 0; i < 16; i++) modes4[i] = (int)(rnd() % 9);
        for (i = 0; i < 4; i++) { m4t[i] = (int)(rnd() % 9); m4l[i] = (int)(rnd() % 9); }
        qp = (int)(rnd() % 52);
        if (emit(f, qp, 1, 0, (int)(rnd() % 4), modes4, cbp & 0xF, cbp >> 4,
                 pick_delta(qp), 1, 1, m4t, m4l) < 0) return 1;
    }

    /* The four availability combinations, which decide whether the mode
     * prediction falls back to DC at the macroblock edge. */
    for (at = 0; at < 2; at++)
        for (al = 0; al < 2; al++)
            for (rep = 0; rep < 32; rep++) {
                for (i = 0; i < 16; i++) modes4[i] = (int)(rnd() % 9);
                for (i = 0; i < 4; i++) {
                    m4t[i] = at ? (int)(rnd() % 9) : 2;
                    m4l[i] = al ? (int)(rnd() % 9) : 2;
                }
                qp = (int)(rnd() % 52);
                if (emit(f, qp, 1, 0, (int)(rnd() % 4), modes4,
                         (int)(rnd() % 16), (int)(rnd() % 3),
                         pick_delta(qp), at, al, m4t, m4l) < 0) return 1;
            }

    /* mb_qp_delta at both ends of its range. */
    for (qp = 0; qp <= 51; qp++) {
        for (i = 0; i < 16; i++) modes4[i] = (int)(rnd() % 9);
        for (i = 0; i < 4; i++) { m4t[i] = (int)(rnd() % 9); m4l[i] = (int)(rnd() % 9); }
        for (s = 0; s < 2; s++) {
            int d = s ? (51 - qp > 25 ? 25 : 51 - qp) : (qp > 26 ? -26 : -qp);
            if (emit(f, qp, 1, 0, 1, modes4, 15, 2, d, 1, 1, m4t, m4l) < 0) return 1;
        }
    }

    /* Random fill, half of it biased so each block takes the one-bit
     * "same as predicted" path, which a uniform draw reaches only 1 time in 9. */
    for (rep = 0; rep < 3000; rep++) {
        int is4 = (int)(rnd() & 1);
        at = (int)(rnd() & 1); al = (int)(rnd() & 1);
        for (i = 0; i < 16; i++) modes4[i] = (int)(rnd() % 9);
        for (i = 0; i < 4; i++) {
            m4t[i] = at ? (int)(rnd() % 9) : 2;
            m4l[i] = al ? (int)(rnd() % 9) : 2;
        }
        if (rnd() & 1) {
            for (s = 0; s < 16; s++) {
                int r = scan_br[s], c = scan_bc[s];
                int mt, ml, tok, lok, pm;
                if (r > 0) { mt = modes4[(r - 1) * 4 + c]; tok = 1; }
                else       { mt = m4t[c];                  tok = at; }
                if (c > 0) { ml = modes4[r * 4 + c - 1];   lok = 1; }
                else       { ml = m4l[r];                  lok = al; }
                pm = (tok && lok) ? (mt < ml ? mt : ml) : 2;
                if (rnd() & 1) modes4[r * 4 + c] = pm;
            }
        }
        qp = (int)(rnd() % 52);
        if (emit(f, qp, is4, (int)(rnd() % 4), (int)(rnd() % 4), modes4,
                 is4 ? (int)(rnd() % 16) : (int)(rnd() & 1),
                 (int)(rnd() % 3), pick_delta(qp), at, al, m4t, m4l) < 0)
            return 1;
    }

    fclose(f);
    fprintf(stderr, "wrote build/mb_header_dec_vectors.txt: %d vectors\n", n_emitted);
    return 0;
}
