/* h264_host.h — the H.264-specific half of the host: everything the generic
 * driver in codec_kernel.h deliberately does not know.
 *
 * Three jobs:
 *   1. build the CONFIG word the kernel latches at START
 *   2. present an NV12 frame in the order the kernel's input stream expects
 *   3. wrap the kernel's payload back into a decodable Annex B stream
 *
 * Jobs 2 and 3 are pure functions over buffers, so they are testable on a
 * workstation against the C reference before any board exists. That is the
 * point: by the time this runs on hardware, the only untested thing left is
 * the hardware.
 */
#ifndef DCC_H264_HOST_H
#define DCC_H264_HOST_H

#include <stdint.h>
#include "codec_kernel.h"

/* ------------------------------------------------------------- config --- */
/* CONFIG layout: [7:0] mbs_w, [15:8] mbs_h, [21:16] qp.
 * width and height must be multiples of 16 and at most 4080; qp is 0..51. */
static inline uint32_t h264_config_word(int width, int height, int qp)
{
    uint32_t mbs_w = (uint32_t)(width  / 16);
    uint32_t mbs_h = (uint32_t)(height / 16);
    return (mbs_w & 0xFFu) | ((mbs_h & 0xFFu) << 8) | (((uint32_t)qp & 0x3Fu) << 16);
}

/* ------------------------------------------------- per-MB rate control ---
 * Kernel 1.3 (VERSION 0x00010003) adds the per-MB QP correction of the C
 * reference's rc_mb == 3 (src/encoder.c, docs/rate-control-rtl.md). The host
 * keeps the frame-level controller (bucket, frame model) and writes these
 * before START; with RC_EN clear the kernel is 1.2 again. RC_WTOTAL after a
 * frame is the next frame's w_total; scale = target * 2^16 / w_total, or
 * target * 2^16 / mb_count with MAP_VALID clear (first frame, or after a
 * size change). CONFIG.qp is the frame QP as before. */
#define DCC_H264_REG_RC_CTRL    0x24u   /* [0] RC_EN [1] MAP_VALID [13:8] qp_min [21:16] qp_max [27:24] lag */
#define DCC_H264_REG_RC_TARGET  0x28u   /* frame target, bits */
#define DCC_H264_REG_RC_SCALE   0x2Cu   /* target * 2^16 / w_total */
#define DCC_H264_REG_RC_WTOTAL  0x30u   /* R: sum of the last frame's MB bits (16-bit saturated per MB) */
#define DCC_H264_RC_LAG_DEFAULT 2       /* the pipeline depth; 1 stalls, 3 gains nothing */

static inline uint32_t h264_rc_ctrl_word(int enable, int map_valid, int qp_min, int qp_max, int lag)
{
    return (enable ? 1u : 0u) | (map_valid ? 2u : 0u) | (((uint32_t)qp_min & 0x3Fu) << 8)
         | (((uint32_t)qp_max & 0x3Fu) << 16) | (((uint32_t)lag & 0xFu) << 24);
}

static inline uint32_t h264_rc_scale(uint32_t target_bits, uint32_t w_total)
{
    uint64_t s = ((uint64_t)target_bits << 16) / (w_total ? w_total : 1u);
    return s > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)s;
}

/* ------------------------------------------------------- input ordering ---
 * The kernel reads a frame as, per macroblock row, 16 luma lines then 8
 * interleaved-chroma lines, each the full frame width. Both runs are
 * contiguous inside an NV12 frame, so the whole reordering is an alternation
 * of two contiguous chunks -- which is why it costs either one memcpy pass or
 * two scatter-gather descriptors per macroblock row, and never a per-line
 * gather.
 *
 * h264_stream_chunks is the single source of truth for that order. The
 * simple-DMA path copies the chunks into a staging buffer; the scatter-
 * gather path turns each chunk into a descriptor and copies nothing. */
typedef struct {
    uint32_t offset;   /* byte offset into the NV12 frame */
    uint32_t length;   /* bytes */
} h264_chunk_t;

/* Fill out[] with the chunks of one frame, in stream order. Returns the
 * number written, or -1 if max is too small. The count is 2 per macroblock
 * row, so 136 for 1080p. */
int h264_stream_chunks(int width, int height, h264_chunk_t *out, int max);

/* Bytes the kernel expects for one frame: width * height * 3 / 2. */
static inline uint32_t h264_frame_bytes(int width, int height)
{
    return (uint32_t)width * (uint32_t)height * 3u / 2u;
}

/* Copy an NV12 frame into kernel stream order. dst must hold
 * h264_frame_bytes(width, height). Use this for the simple-DMA bring-up
 * path; once the stream is proven, move to scatter-gather and drop it. */
void h264_nv12_to_stream(uint8_t *dst, const uint8_t *nv12, int width, int height);

/* ---------------------------------------------------- output assembly --- */
/* The kernel emits the macroblock layer byte-aligned, terminated by a stop
 * bit and zero padding -- exactly rbsp_trailing_bits. The byte count alone
 * therefore does not give the bit count, but the padding is all zeros, so
 * the last set bit in the payload *is* the stop bit and the bit count falls
 * out of it. Returns the macroblock-layer bit count, or -1 if the payload is
 * empty or all zero (which means the kernel produced nothing valid). */
int h264_payload_bits(const uint8_t *payload, int payload_len);

/* Build a complete Annex B access unit from one kernel payload.
 *
 *   dst/dst_cap          destination byte stream
 *   scratch/scratch_cap  working buffer for the un-escaped slice RBSP
 *   payload              the bytes read off m_axis for this frame
 *   payload_len          BYTES as reported by the kernel
 *   with_sps_pps         prepend SPS and PPS (send on the first frame, and
 *                        periodically after, so a late receiver can join)
 *
 * Buffer sizing. The RBSP is the payload plus a slice header, so scratch
 * needs payload_len + H264_ASSEMBLE_SLACK. For dst, note that nal_emit_idr
 * refuses up front unless it has twice the RBSP length: emulation prevention
 * cannot actually double anything (its worst case is one inserted byte per
 * two source bytes), but the check is the check, so size to it rather than to
 * the true bound. Both capacities are verified here; neither is assumed.
 *
 * Returns bytes written, or negative on error. The result is byte-for-byte
 * what the C reference encoder produces for the same frame and QP, which is
 * what makes an on-board comparison meaningful. */
#define H264_ASSEMBLE_SLACK 64
#define H264_ASSEMBLE_DST_MIN(payload_len)     (2 * ((payload_len) + H264_ASSEMBLE_SLACK) + 4 * H264_ASSEMBLE_SLACK)

int h264_assemble_idr(uint8_t *dst, int dst_cap,
                      uint8_t *scratch, int scratch_cap,
                      const uint8_t *payload, int payload_len,
                      int width, int height, int qp, int frame_num,
                      int with_sps_pps);

#endif
