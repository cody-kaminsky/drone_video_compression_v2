/* test_cavlc_roundtrip.c — encode + decode + verify match.
 *
 * Tests cavlc_encode_block / cavlc_decode_block agreement on a wide range
 * of block patterns. This catches LOGIC bugs (encoder/decoder mismatch) but
 * not table-VALUE bugs (since both sides use the same tables).
 *
 * Usage: ./build/test_cavlc_roundtrip
 */
#include "../src/types.h"
#include "../src/cavlc.h"
#include "../src/bitstream.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int total = 0;
static int failed = 0;

static const char *bt_name(block_type_t bt)
{
    switch (bt) {
        case BLK_LUMA_AC:       return "LUMA_AC";
        case BLK_LUMA_FULL:     return "LUMA_FULL";
        case BLK_LUMA_DC_16x16: return "LUMA_DC_16x16";
        case BLK_CHROMA_AC:     return "CHROMA_AC";
        case BLK_CHROMA_DC:     return "CHROMA_DC";
    }
    return "?";
}

static int test_block(const i16 *in, int n, block_type_t bt, int nC,
                      const char *label)
{
    total++;
    u8 buf[256] = {0};
    bitstream_t bs;
    bs_init(&bs, buf, sizeof buf);
    int bits_written = cavlc_encode_block(&bs, in, n, bt, nC);
    bs_byte_count(&bs);   /* flush */

    if (bits_written < 0) {
        printf("FAIL[%s][%s nC=%d] encode failed\n", label, bt_name(bt), nC);
        failed++;
        return -1;
    }

    bitreader_t br;
    br_init(&br, buf, sizeof buf);
    i16 out[16] = {0};
    int rc = cavlc_decode_block(&br, out, n, bt, nC);
    if (rc < 0) {
        printf("FAIL[%s][%s nC=%d] decode failed\n", label, bt_name(bt), nC);
        failed++;
        return -1;
    }

    int bits_read = br_bit_pos(&br);
    /* The reader will have consumed the encoded bits; the rest is padding.
     * The actual bits_written may differ from bits_read if there were
     * trailing zero bits in our buffer that the reader misinterpreted —
     * but for valid CAVLC the count should match. */

    int mismatch = 0;
    for (int i = 0; i < n; i++) {
        if (in[i] != out[i]) { mismatch = 1; break; }
    }
    if (mismatch || bits_written != bits_read) {
        printf("FAIL[%s][%s nC=%d] in=[", label, bt_name(bt), nC);
        for (int i = 0; i < n; i++) printf("%d ", in[i]);
        printf("] out=[");
        for (int i = 0; i < n; i++) printf("%d ", out[i]);
        printf("] bits_w=%d bits_r=%d\n", bits_written, bits_read);
        failed++;
        return -1;
    }
    return 0;
}

int main(void)
{
    /* Empty block */
    i16 zero[16] = {0};
    test_block(zero, 16, BLK_LUMA_AC, 0, "empty-AC");
    test_block(zero, 16, BLK_LUMA_FULL, 0, "empty-FULL");
    test_block(zero, 4,  BLK_CHROMA_DC, -1, "empty-CDC");

    /* Single nonzero at various positions */
    for (int pos = 0; pos < 16; pos++) {
        for (int v = -3; v <= 3; v++) {
            if (v == 0) continue;
            i16 b[16] = {0};
            b[pos] = v;
            char lab[32];
            snprintf(lab, sizeof lab, "pos%d_v%d", pos, v);
            test_block(b, 16, BLK_LUMA_FULL, 0, lab);
        }
    }

    /* Two nonzeros (DC + last) */
    {
        i16 b[16] = {0};
        b[0] = 5; b[15] = -1;
        test_block(b, 16, BLK_LUMA_FULL, 0, "DC5_last-1");
    }

    /* All trailing ones */
    {
        i16 b[16] = {0};
        b[13] = 1; b[14] = -1; b[15] = 1;
        test_block(b, 16, BLK_LUMA_FULL, 0, "3-T1");
    }

    /* Common case: small DC + a couple of low-freq AC */
    {
        i16 b[16] = {0};
        b[0] = 10; b[1] = -3; b[2] = 2;
        test_block(b, 16, BLK_LUMA_FULL, 0, "DC10_AC-3_2");
    }

    /* Larger levels (escape territory) */
    {
        i16 b[16] = {0};
        b[0] = 25; b[1] = -15;
        test_block(b, 16, BLK_LUMA_FULL, 0, "DC25_-15");
    }

    /* Random blocks */
    srand(12345);
    for (int trial = 0; trial < 200; trial++) {
        i16 b[16] = {0};
        int n_nz = (rand() % 8) + 1;
        for (int k = 0; k < n_nz; k++) {
            int p = rand() % 16;
            int v = (rand() % 9) - 4;
            if (v == 0) v = 1;
            b[p] = v;
        }
        char lab[32];
        snprintf(lab, sizeof lab, "rand%d", trial);
        test_block(b, 16, BLK_LUMA_FULL, 0, lab);
        test_block(b, 16, BLK_LUMA_AC, 0, lab);
    }

    /* Chroma DC blocks (4 coefs) */
    for (int trial = 0; trial < 50; trial++) {
        i16 b[4] = {0};
        for (int k = 0; k < 4; k++) {
            b[k] = (i16)((rand() % 7) - 3);
        }
        char lab[32];
        snprintf(lab, sizeof lab, "cdc%d", trial);
        test_block(b, 4, BLK_CHROMA_DC, -1, lab);
    }

    /* Different nC values */
    for (int nC = 0; nC < 10; nC++) {
        i16 b[16] = {0};
        b[0] = 3; b[1] = -2; b[3] = 1;
        char lab[32];
        snprintf(lab, sizeof lab, "nC%d", nC);
        test_block(b, 16, BLK_LUMA_FULL, nC, lab);
    }

    printf("\n%d/%d passed (%d failures)\n", total - failed, total, failed);
    return failed ? 1 : 0;
}
