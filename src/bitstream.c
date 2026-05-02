/* bitstream.c — bit-level writer for H.264 Annex B output. */

#include "bitstream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void bs_init(bitstream_t *bs, u8 *buf, int cap)
{
    bs->buf       = buf;
    bs->cap       = cap;
    bs->byte_pos  = 0;
    bs->cur       = 0;
    bs->n_in_cur  = 0;
    bs->overflow  = 0;
}

/* Flush whole bytes from the accumulator to the buffer. */
static void bs_flush_bytes(bitstream_t *bs)
{
    while (bs->n_in_cur >= 8) {
        if (bs->byte_pos >= bs->cap) {
            bs->overflow = 1;
            bs->n_in_cur = 0;
            return;
        }
        /* Top byte of cur (positions 31..24) is the next byte. */
        bs->buf[bs->byte_pos++] = (u8)((bs->cur >> 24) & 0xFF);
        bs->cur <<= 8;
        bs->n_in_cur -= 8;
    }
}

void bs_put_bits(bitstream_t *bs, u32 val, int nbits)
{
    if (nbits <= 0) return;
    if (nbits > 32) nbits = 32;

    /* Mask val to nbits and place it MSB-aligned in a 32-bit word, then
     * shift it down into the appropriate position above n_in_cur. */
    u32 mask = (nbits == 32) ? 0xFFFFFFFFu : ((1u << nbits) - 1u);
    val &= mask;

    /* Debug: log every emission. Set BS_DEBUG=1 to enable. */
    {
        static int debug = -1;
        if (debug < 0) {
            const char *e = getenv("BS_DEBUG");
            debug = e && *e ? 1 : 0;
        }
        if (debug) {
            fprintf(stderr, "bs_put_bits val=0x%X nbits=%d at bit_pos=%d\n",
                    val, nbits, bs->byte_pos * 8 + bs->n_in_cur);
        }
    }

    /* Place val starting at bit (31 - n_in_cur), going downward. */
    int free_bits = 32 - bs->n_in_cur;

    if (nbits <= free_bits) {
        bs->cur |= (val << (free_bits - nbits));
        bs->n_in_cur += nbits;
    } else {
        /* Split across overflow: put high (free_bits) bits, flush, then put rest. */
        int hi = nbits - free_bits;
        u32 hi_part = val >> hi;
        bs->cur |= hi_part;
        bs->n_in_cur = 32;
        bs_flush_bytes(bs);
        u32 lo_part = val & ((1u << hi) - 1u);
        bs->cur |= (lo_part << (32 - hi));
        bs->n_in_cur = hi;
    }

    bs_flush_bytes(bs);
}

void bs_put_ue(bitstream_t *bs, u32 val)
{
    /* Exp-Golomb unsigned: prefix of N zeros, then '1', then val+1 in
     * (N+1)-bit binary minus the leading 1. Equivalently:
     *   code = (val+1) in binary, with (bits-1) leading zeros padded to
     *   make the total length 2*bits-1 where bits = floor(log2(val+1))+1.
     * Simpler form: emit val+1 with leading zeros to total length 2k+1
     * where k = floor(log2(val+1)). */
    u32 code = val + 1;
    int len = 0;
    u32 tmp = code;
    while (tmp) { len++; tmp >>= 1; }
    /* Total code length = 2*len - 1. Leading zeros = len - 1, then code. */
    bs_put_bits(bs, 0, len - 1);
    bs_put_bits(bs, code, len);
}

void bs_put_se(bitstream_t *bs, i32 val)
{
    /* Map to unsigned: 0 -> 0, 1 -> 1, -1 -> 2, 2 -> 3, -2 -> 4, ... */
    u32 u;
    if (val <= 0) u = (u32)(-val) * 2u;        /* 0,2,4,... */
    else          u = (u32)val * 2u - 1u;      /* 1,3,5,... */
    bs_put_ue(bs, u);
}

void bs_rbsp_trailing(bitstream_t *bs)
{
    bs_put_bits(bs, 1, 1);
    /* Pad with zeros to the next byte boundary. */
    if (bs->n_in_cur != 0) {
        int pad = 8 - (bs->n_in_cur % 8);
        if (pad < 8) bs_put_bits(bs, 0, pad);
    }
    bs_flush_bytes(bs);
}

int bs_byte_count(bitstream_t *bs)
{
    /* Pad partial byte with zeros (caller should have called
     * bs_rbsp_trailing first if RBSP semantics matter). */
    if (bs->n_in_cur > 0) {
        int pad = 8 - bs->n_in_cur;
        if (pad > 0 && pad < 8) bs_put_bits(bs, 0, pad);
    }
    bs_flush_bytes(bs);
    return bs->byte_pos;
}

void bs_put_startcode(bitstream_t *bs)
{
    /* Caller must ensure byte alignment. */
    bs_put_bits(bs, 0x00000001, 32);
}

/* ===== Bit reader ===== */

void br_init(bitreader_t *br, const u8 *buf, int len)
{
    br->buf = buf;
    br->buf_len = len;
    br->byte_pos = 0;
    br->bit_in_byte = 0;
    br->overflow = 0;
}

u32 br_get_bits(bitreader_t *br, int nbits)
{
    u32 v = 0;
    while (nbits > 0) {
        if (br->byte_pos >= br->buf_len) { br->overflow = 1; return v; }
        int avail = 8 - br->bit_in_byte;
        int take  = (nbits < avail) ? nbits : avail;
        u8 byte = br->buf[br->byte_pos];
        u32 bits = (byte >> (avail - take)) & ((1u << take) - 1u);
        v = (v << take) | bits;
        br->bit_in_byte += take;
        if (br->bit_in_byte == 8) { br->bit_in_byte = 0; br->byte_pos++; }
        nbits -= take;
    }
    return v;
}

u32 br_get_ue(bitreader_t *br)
{
    int zeros = 0;
    while (!br_get_bits(br, 1)) {
        if (br->overflow) return 0;
        if (++zeros > 32) { br->overflow = 1; return 0; }
    }
    if (zeros == 0) return 0;
    u32 suf = br_get_bits(br, zeros);
    return (1u << zeros) - 1 + suf;
}

i32 br_get_se(bitreader_t *br)
{
    u32 u = br_get_ue(br);
    if (u & 1) return (i32)((u + 1) >> 1);
    return -(i32)(u >> 1);
}

int br_bit_pos(const bitreader_t *br)
{
    return br->byte_pos * 8 + br->bit_in_byte;
}

int rbsp_emulation_prevent(u8 *dst, int dst_cap, const u8 *src, int src_len)
{
    /* Insert 0x03 after any 0x00 0x00 0x{00,01,02,03}.
     * Walk src, copy to dst, inserting escapes as needed. */
    int di = 0;
    int zeros = 0;
    for (int i = 0; i < src_len; i++) {
        u8 b = src[i];
        if (zeros >= 2 && b <= 0x03) {
            if (di >= dst_cap) return -1;
            dst[di++] = 0x03;
            zeros = 0;
        }
        if (di >= dst_cap) return -1;
        dst[di++] = b;
        if (b == 0) zeros++;
        else        zeros = 0;
    }
    return di;
}
