/* nal.h — emit SPS/PPS/slice header NAL units (Annex B byte stream).
 *
 * Each writer takes a destination buffer and returns the number of bytes
 * written (including the 4-byte start code, NAL header byte, and any
 * emulation-prevention bytes).
 *
 * Configuration is fixed for our 4K/1080p I-only profile:
 *   - profile_idc = 66 (Baseline)
 *   - level_idc auto-selected (40 for 1080p30, 52 for 4K30)
 *   - chroma_format_idc = 1 (4:2:0)
 *   - log2_max_frame_num_minus4 = 0
 *   - pic_order_cnt_type = 2 (no reorder)
 *   - num_ref_frames = 1
 *   - frame_mbs_only_flag = 1
 *   - entropy_coding_mode_flag = 0 (CAVLC)
 */
#ifndef DCC_NAL_H
#define DCC_NAL_H

#include "types.h"

/* Write an SPS NAL unit (with start code) for an I-only Baseline stream.
 *   width, height : pixel dimensions (must be MB-aligned).
 * Returns bytes written, or -1 on overflow. */
int nal_write_sps(u8 *dst, int dst_cap, int width, int height, int qp_init);

/* Write a PPS NAL unit (with start code). */
int nal_write_pps(u8 *dst, int dst_cap, int qp_init);

/* Write the slice header for an IDR frame. The caller must follow up by
 * appending CAVLC slice data (with emulation prevention applied as a
 * combined NAL operation — see nal_finalize_idr_slice).
 *
 * Returns bytes written into the *internal* RBSP buffer (no escape yet). */
int nal_write_slice_header(u8 *rbsp_dst, int dst_cap, int frame_num,
                           int idr_pic_id, int qp_init);

/* Wrap an RBSP payload (slice header + slice data) into a complete IDR
 * NAL unit: prepend Annex B start code + NAL header byte, apply
 * emulation prevention to the RBSP body, write into dst.
 *
 *   rbsp:     pointer to RBSP buffer (header + data, with rbsp_trailing_bits
 *             already appended via bs_rbsp_trailing).
 *   rbsp_len: byte length of RBSP.
 * Returns bytes written, or -1 on overflow. */
int nal_emit_idr(u8 *dst, int dst_cap, const u8 *rbsp, int rbsp_len);

#endif
