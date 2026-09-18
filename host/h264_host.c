/* h264_host.c — see h264_host.h.
 *
 * This file links against the reference encoder's own src/nal.c and
 * src/bitstream.c rather than reimplementing them. That is deliberate: the
 * parameter sets and the slice header are fiddly, they are already verified
 * byte-exact against ffmpeg, and a second implementation on the board would
 * be a second thing that can be wrong. Both are free of malloc and stdio, so
 * they compile for bare-metal unchanged. */

#include "h264_host.h"
#include "nal.h"
#include "bitstream.h"
#include <string.h>

int h264_stream_chunks(int width, int height, h264_chunk_t *out, int max)
{
    int mbs_h = height / 16;
    uint32_t y_size = (uint32_t)width * (uint32_t)height;
    int n = 0;

    if (max < 2 * mbs_h) return -1;
    for (int r = 0; r < mbs_h; r++) {
        out[n].offset = (uint32_t)r * 16u * (uint32_t)width;   /* 16 luma lines */
        out[n].length = 16u * (uint32_t)width;
        n++;
        out[n].offset = y_size + (uint32_t)r * 8u * (uint32_t)width;  /* 8 chroma lines */
        out[n].length = 8u * (uint32_t)width;
        n++;
    }
    return n;
}

void h264_nv12_to_stream(uint8_t *dst, const uint8_t *nv12, int width, int height)
{
    int mbs_h = height / 16;
    uint32_t y_size = (uint32_t)width * (uint32_t)height;
    uint32_t o = 0;

    for (int r = 0; r < mbs_h; r++) {
        memcpy(dst + o, nv12 + (uint32_t)r * 16u * (uint32_t)width,
               16u * (size_t)width);
        o += 16u * (uint32_t)width;
        memcpy(dst + o, nv12 + y_size + (uint32_t)r * 8u * (uint32_t)width,
               8u * (size_t)width);
        o += 8u * (uint32_t)width;
    }
}

int h264_payload_bits(const uint8_t *payload, int payload_len)
{
    int last = payload_len - 1;
    uint8_t b;
    int tz = 0;

    if (payload_len <= 0) return -1;
    /* The stop bit is the last set bit. Padding is zeros, so walk back over
     * any all-zero tail first; a well-formed payload has none, but a short
     * or truncated DMA does, and saying so beats returning a wrong count. */
    while (last >= 0 && payload[last] == 0) last--;
    if (last < 0) return -1;

    b = payload[last];
    while ((b & 1u) == 0u) { b >>= 1; tz++; }
    return (last + 1) * 8 - 1 - tz;
}

int h264_assemble_idr(uint8_t *dst, int dst_cap,
                      uint8_t *scratch, int scratch_cap,
                      const uint8_t *payload, int payload_len,
                      int width, int height, int qp, int frame_num,
                      int with_sps_pps)
{
    /* The slice RBSP is built in scratch, then escaped into dst by
     * nal_emit_idr. Escaping can expand by up to half, so dst is sized from
     * the payload rather than assumed to be "obviously big enough". */
    bitstream_t bs;
    uint8_t *rbsp = scratch;
    int dst_pos = 0, n, mb_bits, full, rem, i;

    if (payload_len <= 0) return -1;
    if (scratch_cap < payload_len + H264_ASSEMBLE_SLACK) return -1;
    if (dst_cap < H264_ASSEMBLE_DST_MIN(payload_len)) return -1;

    if (with_sps_pps) {
        n = nal_write_sps(dst + dst_pos, dst_cap - dst_pos, width, height, qp);
        if (n < 0) return -2;
        dst_pos += n;
        n = nal_write_pps(dst + dst_pos, dst_cap - dst_pos, qp);
        if (n < 0) return -2;
        dst_pos += n;
    }

    mb_bits = h264_payload_bits(payload, payload_len);
    if (mb_bits < 0) return -3;

    bs_init(&bs, rbsp, scratch_cap);

    /* Slice header. Must match src/encoder.c exactly, including
     * disable_deblocking_filter_idc = 1: the kernel does not deblock, so the
     * decoder must not either, or its reconstruction diverges from ours. */
    bs_put_ue(&bs, 0);                       /* first_mb_in_slice */
    bs_put_ue(&bs, 7);                       /* slice_type = 7 (I, all slices) */
    bs_put_ue(&bs, 0);                       /* pic_parameter_set_id */
    bs_put_bits(&bs, (u32)(frame_num & 0xF), 4);
    bs_put_ue(&bs, (u32)(frame_num & 0xF));  /* idr_pic_id */
    bs_put_bits(&bs, 0, 1);                  /* no_output_of_prior_pics_flag */
    bs_put_bits(&bs, 0, 1);                  /* long_term_reference_flag */
    bs_put_se(&bs, 0);                       /* slice_qp_delta */
    bs_put_ue(&bs, 1);                       /* disable_deblocking_filter_idc */

    /* Macroblock layer, bit-shifted onto whatever offset the header ended at.
     * Byte at a time for the bulk, so this costs one call per payload byte
     * rather than one per bit. */
    full = mb_bits >> 3;
    rem  = mb_bits & 7;
    for (i = 0; i < full; i++) bs_put_bits(&bs, payload[i], 8);
    if (rem) bs_put_bits(&bs, (u32)(payload[full] >> (8 - rem)), rem);

    bs_rbsp_trailing(&bs);
    if (bs.overflow) return -4;

    n = nal_emit_idr(dst + dst_pos, dst_cap - dst_pos, rbsp, bs_byte_count(&bs));
    if (n < 0) return -5;
    return dst_pos + n;
}
