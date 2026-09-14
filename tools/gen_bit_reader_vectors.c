/* gen_bit_reader_vectors.c — golden vectors for the VHDL bit_reader.
 *
 * The golden is src/bitstream.c's bitreader_t, the same reader the C decoder
 * uses and therefore the same one that has already been shown to parse real
 * streams byte-exactly. Testing the RTL against it means the two halves of
 * the decoder agree on what "the next n bits" means, which is the only thing
 * a bit reader has to get right.
 *
 * Two streams are emitted:
 *   - pseudorandom bytes with pseudorandom field widths, which hits widths
 *     and alignments a real stream would take a long time to reach;
 *   - a real slice payload if one is given, read with the same width
 *     pattern, so the block sees the byte statistics it will actually meet
 *     (long runs of zeros, escape codes, and so on).
 *
 * Output, build/bit_reader_vectors.txt:
 *   <n_bytes>            followed by n lines of one decimal byte
 *   <n_ops>              followed by n lines of "<width> <value in hex>"
 *
 * Values are hex because a VHDL integer is 32-bit signed and a textio read
 * of a field >= 2^31 overflows.
 *
 * Usage: gen_bit_reader_vectors [payload.bin]
 */

#include "bitstream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N_BYTES 4096
#define N_OPS   3000

static u32 rng_state = 0xC0FFEEu;
static u32 rng(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

/* Widths are biased towards the small ones a real parser uses most, with
 * enough long reads to exercise the wide end of the peek window. */
static int pick_width(void)
{
    u32 r = rng() % 100;
    if (r < 40) return 1 + (int)(rng() % 3);      /* 1..3  flags, small fields */
    if (r < 70) return 4 + (int)(rng() % 5);      /* 4..8  typical VLC lengths */
    if (r < 90) return 9 + (int)(rng() % 8);      /* 9..16 long prefixes */
    return 17 + (int)(rng() % 16);                /* 17..32 escape suffixes */
}

static int emit(FILE *f, const u8 *buf, int nbytes, const char *label)
{
    bitreader_t br;
    int i, n_ops = 0;
    int widths[N_OPS];
    u32 vals[N_OPS];
    long bits_used = 0;

    br_init(&br, buf, nbytes);
    for (i = 0; i < N_OPS; i++) {
        int w = pick_width();
        /* Stop well clear of the end: the block's underrun behaviour is
         * tested separately, and mixing it in here would only obscure the
         * width and alignment coverage this stream is for. */
        if (bits_used + w + 64 > (long)nbytes * 8) break;
        widths[n_ops] = w;
        vals[n_ops]   = br_get_bits(&br, w);
        bits_used += w;
        n_ops++;
    }
    if (br.overflow) { fprintf(stderr, "%s: golden reader overflowed\n", label); return -1; }

    /* Counts bare, values in hex. A VHDL integer is 32-bit SIGNED, so a
     * textio read of anything >= 2^31 overflows; hread into an unsigned
     * is the only safe way to carry a full 32-bit field. */
    fprintf(f, "%d\n", nbytes);
    for (i = 0; i < nbytes; i++) fprintf(f, "%d\n", buf[i]);
    fprintf(f, "%d\n", n_ops);
    for (i = 0; i < n_ops; i++) fprintf(f, "%d %08X\n", widths[i], vals[i]);
    fprintf(stderr, "%s: %d bytes, %d ops, %ld bits consumed\n",
            label, nbytes, n_ops, bits_used);
    return 0;
}

int main(int argc, char **argv)
{
    u8 *buf;
    FILE *f, *pf;
    int i, n;

    buf = malloc(N_BYTES);
    if (!buf) return 2;

    f = fopen("build/bit_reader_vectors.txt", "w");
    if (!f) { perror("build/bit_reader_vectors.txt"); return 2; }
    for (i = 0; i < N_BYTES; i++) buf[i] = (u8)(rng() & 0xFF);
    if (emit(f, buf, N_BYTES, "random") != 0) return 1;
    fclose(f);

    /* A real payload, if one was given. */
    if (argc > 1) {
        pf = fopen(argv[1], "rb");
        if (!pf) { perror(argv[1]); free(buf); return 2; }
        n = (int)fread(buf, 1, N_BYTES, pf);
        fclose(pf);
        if (n < 256) { fprintf(stderr, "%s: too short\n", argv[1]); free(buf); return 2; }
        f = fopen("build/bit_reader_vectors_real.txt", "w");
        if (!f) { perror("build/bit_reader_vectors_real.txt"); free(buf); return 2; }
        if (emit(f, buf, n, "real payload") != 0) return 1;
        fclose(f);
    }

    free(buf);
    return 0;
}
