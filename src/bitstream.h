/* bitstream.h — bit-level writer with Exp-Golomb and Annex B framing.
 *
 * Used by M2 to emit a real H.264 bitstream that ffmpeg can decode.
 *
 * The writer accumulates into a byte buffer, MSB-first inside each byte,
 * which matches H.264 conventions. Two finalization paths:
 *   1. bs_finalize_rbsp() — pad with rbsp_trailing_bits (a 1 bit + zeros
 *      to byte align), then apply emulation-prevention escapes. Used for
 *      SPS, PPS, and slice payloads.
 *   2. bs_byte_count() — raw byte count, no escapes. Used for AUDs and
 *      simple cases.
 *
 * FPGA: the bit writer is the inner loop of the CAVLC engine. In HW this
 *       is a 64-bit shift register + barrel shifter; the C reference
 *       optimizes for clarity.
 */
#ifndef DCC_BITSTREAM_H
#define DCC_BITSTREAM_H

#include "types.h"

typedef struct {
    u8 *buf;          /* destination buffer */
    int cap;          /* capacity in bytes */
    int byte_pos;     /* next byte to write */
    u32 cur;          /* current accumulator (top bit at position 31) */
    int n_in_cur;     /* number of bits currently held in cur (0..31) */
    int overflow;     /* set to 1 if writes exceed cap */
} bitstream_t;

/* Initialize a writer over a caller-supplied buffer. */
void bs_init(bitstream_t *bs, u8 *buf, int cap);

/* Append nbits LSB-aligned bits of val. nbits must be in 0..32. */
void bs_put_bits(bitstream_t *bs, u32 val, int nbits);

/* Append unsigned Exp-Golomb codeword for val (val >= 0).
 * Code length = 2*floor(log2(val+1))+1 bits. */
void bs_put_ue(bitstream_t *bs, u32 val);

/* Append signed Exp-Golomb codeword for val. */
void bs_put_se(bitstream_t *bs, i32 val);

/* Append rbsp_trailing_bits: '1' followed by zero or more '0' bits to
 * pad to the next byte boundary. */
void bs_rbsp_trailing(bitstream_t *bs);

/* Flush any partial byte (zero-padded). Returns total bytes written.
 * Does NOT apply emulation prevention. */
int bs_byte_count(bitstream_t *bs);

/* Apply RBSP emulation prevention IN-PLACE on bytes [start..end) of buf.
 * Insert 0x03 after any 0x00 0x00 0x{00,01,02,03} pattern.
 *
 * Returns new end offset (always >= original end).
 *
 * out_buf must have room for end + n_inserted bytes; if it doesn't fit,
 * sets overflow flag and returns -1.
 *
 * If src and dst differ, the operation copies and escapes simultaneously.
 * If src == dst (in place), the buffer is shifted in chunks. */
int rbsp_emulation_prevent(u8 *dst, int dst_cap, const u8 *src, int src_len);

/* Append an Annex B start code (0x00 00 00 01) at the current byte
 * boundary. Caller must ensure bs is byte-aligned (bs_n_in_cur() == 0). */
void bs_put_startcode(bitstream_t *bs);

/* ===== Bit reader (for testing / decoder support) ===== */

typedef struct {
    const u8 *buf;
    int buf_len;       /* bytes available */
    int byte_pos;
    int bit_in_byte;   /* 0..7, MSB-first within byte (0 = MSB) */
    int overflow;      /* set if read past end */
} bitreader_t;

void br_init(bitreader_t *br, const u8 *buf, int len);
u32  br_get_bits(bitreader_t *br, int nbits);   /* up to 32 */
u32  br_get_ue(bitreader_t *br);
i32  br_get_se(bitreader_t *br);
int  br_bit_pos(const bitreader_t *br);          /* total bits consumed */

#endif
