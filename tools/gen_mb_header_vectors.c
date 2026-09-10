/* gen_mb_header_vectors.c — golden vectors for the VHDL mb_header_engine.
 * Mirrors mb_compute_cbp + the header part of mb_cavlc_emit in
 * src/encoder.c (I slices, fixed QP so mb_qp_delta = 0) using the C
 * bitstream writer and the cbp_intra_to_codenum table.
 * Writes build/mb_header_vectors.txt:
 *   M <is_i4x4> <mode16> <mode_chroma> <modes4 x16 raster> <luma_nz x16 raster>
 *     <chroma_dc_nz> <chroma_ac_nz> <avail_top> <avail_left> <mode4_top x4> <mode4_left x4>
 *   E <cbp_luma> <cbp_chroma> <has_residual> <nbits> <24 hex chars, bits MSB-first>
 */
#include "types.h"
#include "bitstream.h"
#include "cavlc_tables.h"
#include <stdio.h>
#include <string.h>

static unsigned rng_state = 0x4EAD5EED;
static unsigned xorshift32(void)
{
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static const int scan_br[16] = {0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3};
static const int scan_bc[16] = {0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3};

int main(void)
{
    FILE *f = fopen("build/mb_header_vectors.txt", "w");
    if (!f) { perror("build/mb_header_vectors.txt"); return 1; }
    int count = 0;
    for (int n = 0; n < 3000; n++) {
        int is4 = (xorshift32() & 1);
        int m16 = (int)(xorshift32() % 4);
        int mc  = (int)(xorshift32() % 4);
        int modes4[16], nz[16], m4t[4], m4l[4];
        int allzero = (xorshift32() % 4) == 0;
        for (int i = 0; i < 16; i++) { modes4[i] = (int)(xorshift32() % 9); nz[i] = allzero ? 0 : (int)(xorshift32() & 1); }
        int cdc = allzero ? 0 : (int)(xorshift32() & 1);
        int cac = allzero ? 0 : (int)((xorshift32() % 3) == 0);
        int at = (int)(xorshift32() & 1), al = (int)(xorshift32() & 1);
        for (int i = 0; i < 4; i++) { m4t[i] = at ? (int)(xorshift32() % 9) : 2; m4l[i] = al ? (int)(xorshift32() % 9) : 2; }
        /* bias towards predicted modes so the 1-bit path is exercised */
        if (xorshift32() & 1) {
            for (int s = 0; s < 16; s++) {
                int br = scan_br[s], bc = scan_bc[s];
                int mt, ml, tok, lok;
                if (br > 0) { mt = modes4[(br-1)*4+bc]; tok = 1; } else { mt = m4t[bc]; tok = at; }
                if (bc > 0) { ml = modes4[br*4+bc-1]; lok = 1; } else { ml = m4l[br]; lok = al; }
                int pm = (tok && lok) ? (mt < ml ? mt : ml) : 2;
                if (xorshift32() & 1) modes4[br*4+bc] = pm;
            }
        }

        /* cbp */
        int cbpl = 0, cbpc;
        if (is4) {
            for (int s = 0; s < 16; s++)
                if (nz[scan_br[s]*4 + scan_bc[s]]) cbpl |= 1 << (s / 4);
        } else {
            for (int i = 0; i < 16; i++) if (nz[i]) cbpl = 1;
        }
        cbpc = cac ? 2 : (cdc ? 1 : 0);

        /* expected bits */
        u8 buf[64];
        bitstream_t bs;
        bs_init(&bs, buf, sizeof buf);
        if (is4) {
            bs_put_ue(&bs, 0);
            for (int s = 0; s < 16; s++) {
                int br = scan_br[s], bc = scan_bc[s];
                int actual = modes4[br*4+bc];
                int mt, ml, tok, lok;
                if (br > 0) { mt = modes4[(br-1)*4+bc]; tok = 1; } else { mt = m4t[bc]; tok = at; }
                if (bc > 0) { ml = modes4[br*4+bc-1]; lok = 1; } else { ml = m4l[br]; lok = al; }
                int pm = (tok && lok) ? (mt < ml ? mt : ml) : 2;
                if (actual == pm) bs_put_bits(&bs, 1, 1);
                else { bs_put_bits(&bs, 0, 1); bs_put_bits(&bs, (actual < pm) ? actual : actual - 1, 3); }
            }
            bs_put_ue(&bs, (u32)mc);
            bs_put_ue(&bs, cbp_intra_to_codenum[(cbpl & 0xF) | (cbpc << 4)]);
            if (cbpl || cbpc) bs_put_se(&bs, 0);
        } else {
            bs_put_ue(&bs, (u32)(1 + m16 + 4 * cbpc + 12 * cbpl));
            bs_put_ue(&bs, (u32)mc);
            bs_put_se(&bs, 0);
        }
        int nbits = bs.byte_pos * 8 + bs.n_in_cur;
        int nbytes = bs_byte_count(&bs);
        fprintf(f, "M %d %d %d", is4, m16, mc);
        for (int i = 0; i < 16; i++) fprintf(f, " %d", modes4[i]);
        for (int i = 0; i < 16; i++) fprintf(f, " %d", nz[i]);
        fprintf(f, " %d %d %d %d", cdc, cac, at, al);
        for (int i = 0; i < 4; i++) fprintf(f, " %d", m4t[i]);
        for (int i = 0; i < 4; i++) fprintf(f, " %d", m4l[i]);
        fprintf(f, "\nE %d %d %d %d ", cbpl, cbpc, (cbpl || cbpc) ? 1 : 0, nbits);
        for (int i = 0; i < 12; i++) fprintf(f, "%02X", i < nbytes ? buf[i] : 0);
        fprintf(f, "\n");
        count++;
    }
    fclose(f);
    printf("wrote %d mb header vectors\n", count);
    return 0;
}
