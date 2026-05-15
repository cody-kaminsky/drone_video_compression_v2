/* gen_bit_packer_vectors.c
 *
 * Generates test vectors for the bit_packer VHDL entity using the C
 * reference's bs_put_bits as the golden model. Each test "group" is a
 * sequence of bs_put_bits calls followed by a flush (zero-pad to byte
 * boundary). All groups are concatenated; the bit_packer is reset only
 * between major scenarios (we use a single contiguous run for simplicity).
 *
 * Output files:
 *
 *   build/bit_packer_vectors_in.txt
 *     One driver beat per line:
 *       B <length_dec> <bits_dec>     -- one bs_put_bits call
 *       F                             -- flush_i pulse, valid_i = 0
 *
 *   build/bit_packer_vectors_out.txt
 *     One byte per line:
 *       Y <byte_dec>                  -- regular byte
 *       L <byte_dec>                  -- last byte of a flush group
 *
 * The VHDL testbench reads both files line-by-line, drives the DUT with
 * `in` lines (honoring ready_o), and compares each emitted byte against
 * the next `out` line. After the final F, it waits for flushed_o and
 * asserts no expected bytes remain.
 *
 * Build:  make bit_packer_vectors
 * Run:    build/gen_bit_packer_vectors           (writes both files)
 */

#include "bitstream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define BUF_CAP (1 << 20)
#define MAX_BEATS_PER_GROUP 32768

/* Per-test driver beats. */
typedef struct {
    int length;     /* 0..32 */
    uint32_t bits;  /* right-aligned */
} beat_t;

typedef struct {
    beat_t beats[MAX_BEATS_PER_GROUP];
    int    n_beats;
} group_t;

static void g_reset(group_t *g) { g->n_beats = 0; }

static void g_put(group_t *g, int length, uint32_t bits)
{
    if (g->n_beats >= MAX_BEATS_PER_GROUP) {
        fprintf(stderr, "group overflow\n");
        exit(2);
    }
    g->beats[g->n_beats].length = length;
    g->beats[g->n_beats].bits   = bits;
    g->n_beats++;
}

/* xorshift32 PRNG, deterministic across platforms. */
static uint32_t xs_state = 0xDEADBEEFu;
static uint32_t xs32(void)
{
    uint32_t x = xs_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xs_state = x;
    return x;
}

/* Apply group g to a bs writer, then return number of bytes produced after
 * a zero-pad to byte boundary (no rbsp_trailing semantics — bit_packer
 * only zero-pads). */
static int run_group_through_ref(const group_t *g, uint8_t *out, int cap)
{
    bitstream_t bs;
    bs_init(&bs, out, cap);
    for (int i = 0; i < g->n_beats; i++) {
        bs_put_bits(&bs, g->beats[i].bits, g->beats[i].length);
    }
    /* Zero-pad to byte boundary, matching bit_packer's flush behavior. */
    return bs_byte_count(&bs);
}

static void emit_group(FILE *fin, FILE *fout, const group_t *g)
{
    /* Drive lines */
    for (int i = 0; i < g->n_beats; i++) {
        fprintf(fin, "B %d %u\n", g->beats[i].length, g->beats[i].bits);
    }
    fprintf(fin, "F\n");

    /* Expected bytes */
    static uint8_t buf[BUF_CAP];
    int n = run_group_through_ref(g, buf, BUF_CAP);
    for (int i = 0; i < n; i++) {
        char tag = (i == n - 1) ? 'L' : 'Y';
        fprintf(fout, "%c %u\n", tag, (unsigned)buf[i]);
    }
    /* If the group produced no bytes (e.g. nothing was written), still
     * record nothing — the testbench will just see flushed_o pulse with
     * no preceding byte and no out_last assertion. */
}

int main(void)
{
    FILE *fin  = fopen("build/bit_packer_vectors_in.txt",  "w");
    FILE *fout = fopen("build/bit_packer_vectors_out.txt", "w");
    if (!fin || !fout) {
        fprintf(stderr, "could not open output files in build/\n");
        return 1;
    }

    group_t g;
    int total_groups = 0;
    int total_bytes  = 0;

    /* Scenario 1: single byte built from 8 one-bit beats. */
    g_reset(&g);
    for (int i = 0; i < 8; i++) g_put(&g, 1, (i & 1));
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 2: one 8-bit beat. */
    g_reset(&g);
    g_put(&g, 8, 0xA5);
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 3: 7 + 7 = 14 bits → 1 full byte + 6 partial; flush pads. */
    g_reset(&g);
    g_put(&g, 7, 0x55);   /* 1010101 */
    g_put(&g, 7, 0x2A);   /* 0101010 */
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 4: 32-bit single beat crossing 4 bytes. */
    g_reset(&g);
    g_put(&g, 32, 0xDEADBEEF);
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 5: aligned width sweep — every length 1..32, packed back-to-back. */
    g_reset(&g);
    for (int L = 1; L <= 32; L++) {
        uint32_t mask = (L == 32) ? 0xFFFFFFFFu : ((1u << L) - 1u);
        g_put(&g, L, xs32() & mask);
    }
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 6: escape level pattern as three sub-fields — exact spec
     * encoding (see src/cavlc.c lines 286-288 for the equivalent path). */
    g_reset(&g);
    g_put(&g, 15, 0x0000);   /* 15 zeros */
    g_put(&g, 1,  0x1);      /* single 1 */
    g_put(&g, 12, 0xABC);    /* escape suffix */
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 7: equivalent escape encoded as a single 28-bit field.
     * The bytes should match scenario 6 exactly (after rounding/padding).
     * We build a 28-bit value: bits = (0 << 13) | (1 << 12) | 0xABC
     *                        = 0x01ABC, padded to 28 bits as 0x0001ABC. */
    g_reset(&g);
    g_put(&g, 28, 0x0001ABCu);
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 8: many 1-bit beats (alternating) to exercise drain cadence. */
    g_reset(&g);
    for (int i = 0; i < 200; i++) g_put(&g, 1, i & 1);
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 9: stress — 4096 random beats, lengths uniform in [1, 32]. */
    g_reset(&g);
    for (int i = 0; i < 4096; i++) {
        int L = 1 + (xs32() % 32);
        uint32_t mask = (L == 32) ? 0xFFFFFFFFu : ((1u << L) - 1u);
        g_put(&g, L, xs32() & mask);
    }
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 10: empty group — flush with no prior bits. */
    g_reset(&g);
    emit_group(fin, fout, &g);
    total_groups++;

    /* Scenario 11: another stress with different lengths skewed toward
     * the boundary cases (1, 7, 8, 9, 16, 24, 32). */
    g_reset(&g);
    static const int boundary_lens[] = {1, 2, 7, 8, 9, 15, 16, 17, 23, 24, 25, 31, 32};
    int n_lens = sizeof(boundary_lens) / sizeof(boundary_lens[0]);
    for (int i = 0; i < 2000; i++) {
        int L = boundary_lens[xs32() % n_lens];
        uint32_t mask = (L == 32) ? 0xFFFFFFFFu : ((1u << L) - 1u);
        g_put(&g, L, xs32() & mask);
    }
    emit_group(fin, fout, &g);
    total_groups++;

    /* Count total expected bytes (not strictly needed; tb tracks itself). */
    /* Best-effort recompute: not critical, just for the summary. */
    static uint8_t scratch[BUF_CAP];
    (void)scratch;

    fclose(fin);
    fclose(fout);

    fprintf(stderr, "wrote %d groups to build/bit_packer_vectors_{in,out}.txt\n",
            total_groups);
    (void)total_bytes;
    return 0;
}
