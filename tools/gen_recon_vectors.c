/* gen_recon_vectors.c — golden vectors for the VHDL recon_engine.
 * Mirrors recon_4x4 in src/encoder.c: recon = clip8(pred + ((res + 32) >> 6))
 * and the distortion ssd = sum((recon - src)^2). Writes build/recon_vectors.txt:
 *   P <p0..p15>      prediction samples
 *   R <r0..r15>      residual (inverse-transform output, before rounding)
 *   S <s0..s15>      source samples
 *   O <o0..o15> <ssd>
 */
#include "types.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned rng_state = 0x0C0FFEE5;
static unsigned xorshift32(void)
{
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}

static int clip_u8(int x) { return x < 0 ? 0 : (x > 255 ? 255 : x); }

int main(void)
{
    FILE *f = fopen("build/recon_vectors.txt", "w");
    if (!f) { perror("build/recon_vectors.txt"); return 1; }
    int count = 0;
    for (int dist = 0; dist < 6; dist++) {
        for (int n = 0; n < 400; n++) {
            int pred[16], res[16], src[16], out[16];
            long ssd = 0;
            for (int k = 0; k < 16; k++) {
                pred[k] = (int)(xorshift32() & 255);
                src[k]  = (int)(xorshift32() & 255);
                int mag;
                switch (dist) {
                case 0:  mag = 0; break;                                   /* zero residual */
                case 1:  mag = (int)(xorshift32() % 64); break;            /* tiny */
                case 2:  mag = (int)(xorshift32() % 2048); break;          /* typical */
                case 3:  mag = (int)(xorshift32() % 20000); break;         /* large, clips often */
                case 4:  mag = (int)(xorshift32() % 524288); break;        /* full 20-bit range */
                default: mag = 63 + (int)(xorshift32() % 3) - 1; break;    /* rounding edge: 62..64 */
                }
                res[k] = (xorshift32() & 1) ? -mag : mag;
                if (res[k] < -524288) res[k] = -524288;
                if (res[k] >  524287) res[k] =  524287;
                out[k] = clip_u8(pred[k] + ((res[k] + 32) >> 6));
                ssd += (long)(out[k] - src[k]) * (out[k] - src[k]);
            }
            fprintf(f, "P"); for (int k = 0; k < 16; k++) fprintf(f, " %d", pred[k]);
            fprintf(f, "\nR"); for (int k = 0; k < 16; k++) fprintf(f, " %d", res[k]);
            fprintf(f, "\nS"); for (int k = 0; k < 16; k++) fprintf(f, " %d", src[k]);
            fprintf(f, "\nO"); for (int k = 0; k < 16; k++) fprintf(f, " %d", out[k]);
            fprintf(f, " %ld\n", ssd);
            count++;
        }
    }
    fclose(f);
    printf("wrote %d recon vectors\n", count);
    return 0;
}
