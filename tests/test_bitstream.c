/* test_bitstream.c — unit tests for bitstream writer.
 *
 * Verifies Exp-Golomb codes against H.264 spec Table 9-1 and emulation
 * prevention against spec 7.4.1.1.
 */
#include "../src/types.h"
#include "../src/bitstream.h"

#include <stdio.h>
#include <string.h>

static int failed = 0;

#define CHECK(cond, fmt, ...) do { \
    if (!(cond)) { \
        printf("FAIL: " fmt "\n", ##__VA_ARGS__); \
        failed++; \
    } \
} while (0)

/* Spec Table 9-1 reference Exp-Golomb codes (binary, MSB-first). */
static const struct { u32 val; const char *bits; } ue_cases[] = {
    {0, "1"},
    {1, "010"},
    {2, "011"},
    {3, "00100"},
    {4, "00101"},
    {5, "00110"},
    {6, "00111"},
    {7, "0001000"},
    {8, "0001001"},
    {9, "0001010"},
    {14, "0001111"},
    {15, "000010000"},
    {239, "000000011110000"},   /* pic_width_in_mbs_minus1 = 239 (4K-1) */
    {119, "0000001111000"},     /* 1080p pic_width_in_mbs_minus1 = 119 */
    {0, NULL}
};

static const struct { i32 val; const char *bits; } se_cases[] = {
    {0,  "1"},
    {1,  "010"},
    {-1, "011"},
    {2,  "00100"},
    {-2, "00101"},
    {3,  "00110"},
    {-3, "00111"},
    {4,  "0001000"},
    {0, NULL}
};

static void bytes_to_bitstring(const u8 *buf, int n_bits, char *out)
{
    for (int i = 0; i < n_bits; i++)
        out[i] = ((buf[i >> 3] >> (7 - (i & 7))) & 1) ? '1' : '0';
    out[n_bits] = 0;
}

int main(void)
{
    /* ---- Exp-Golomb unsigned ---- */
    for (int i = 0; ue_cases[i].bits; i++) {
        u8 buf[16] = {0};
        bitstream_t bs;
        bs_init(&bs, buf, sizeof buf);
        bs_put_ue(&bs, ue_cases[i].val);
        bs_byte_count(&bs);    /* flush partial byte (zero-padded) */
        int nbits = (int)strlen(ue_cases[i].bits);
        char got[64];
        bytes_to_bitstring(buf, nbits, got);
        CHECK(strcmp(got, ue_cases[i].bits) == 0,
              "ue(%u): expected '%s' got '%s'",
              ue_cases[i].val, ue_cases[i].bits, got);
    }

    /* ---- Exp-Golomb signed ---- */
    for (int i = 0; se_cases[i].bits; i++) {
        u8 buf[16] = {0};
        bitstream_t bs;
        bs_init(&bs, buf, sizeof buf);
        bs_put_se(&bs, se_cases[i].val);
        bs_byte_count(&bs);
        int nbits = (int)strlen(se_cases[i].bits);
        char got[64];
        bytes_to_bitstring(buf, nbits, got);
        CHECK(strcmp(got, se_cases[i].bits) == 0,
              "se(%d): expected '%s' got '%s'",
              se_cases[i].val, se_cases[i].bits, got);
    }

    /* ---- bs_put_bits + alignment ---- */
    {
        u8 buf[8] = {0};
        bitstream_t bs;
        bs_init(&bs, buf, sizeof buf);
        bs_put_bits(&bs, 0b1010, 4);
        bs_put_bits(&bs, 0b1100, 4);
        bs_put_bits(&bs, 0xAB, 8);
        int n = bs_byte_count(&bs);
        CHECK(n == 2, "bytes after 16 bits: expected 2 got %d", n);
        CHECK(buf[0] == 0xAC, "byte0 expected 0xAC got 0x%02X", buf[0]);
        CHECK(buf[1] == 0xAB, "byte1 expected 0xAB got 0x%02X", buf[1]);
    }

    /* ---- RBSP emulation prevention ---- */
    {
        const u8 src[] = {0x00, 0x00, 0x00, 0x01, 0xAB, 0x00, 0x00, 0x02, 0xCD};
        u8 dst[16] = {0};
        int n = rbsp_emulation_prevent(dst, sizeof dst, src, sizeof src);
        /* Expected: 0x00 0x00 0x03 0x00 0x01 0xAB 0x00 0x00 0x03 0x02 0xCD */
        const u8 expected[] = {0x00,0x00,0x03,0x00,0x01,0xAB,0x00,0x00,0x03,0x02,0xCD};
        CHECK(n == (int)sizeof expected, "escape len: expected %zu got %d",
              sizeof expected, n);
        for (size_t i = 0; i < sizeof expected && i < (size_t)n; i++) {
            CHECK(dst[i] == expected[i],
                  "escape byte %zu: expected 0x%02X got 0x%02X",
                  i, expected[i], dst[i]);
        }
    }

    /* ---- rbsp_trailing_bits ---- */
    {
        u8 buf[4] = {0};
        bitstream_t bs;
        bs_init(&bs, buf, sizeof buf);
        bs_put_bits(&bs, 0b1010, 4);   /* 4 bits */
        bs_rbsp_trailing(&bs);          /* '1' + 3 zeros = 1000 -> total 8 */
        int n = bs_byte_count(&bs);
        CHECK(n == 1, "rbsp_trailing bytes: expected 1 got %d", n);
        CHECK(buf[0] == 0xA8, "rbsp_trailing byte: expected 0xA8 got 0x%02X", buf[0]);
    }

    if (failed == 0) printf("ALL TESTS PASSED\n");
    else             printf("%d FAILURES\n", failed);
    return failed ? 1 : 0;
}
