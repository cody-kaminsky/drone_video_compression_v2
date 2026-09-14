/* dec_nal.h — Annex B framing and header parsing for the matched decoder.
 *
 * This decoder handles the streams *our* encoder produces, not H.264 in
 * general. That is a deliberate scope: Baseline profile, intra only, CAVLC,
 * one slice per picture, 4:2:0, no deblocking, no interlace, no reordering.
 * Anything outside that is rejected with a reason rather than mis-decoded,
 * because a decoder that quietly guesses is worse than one that stops.
 *
 * Where the encoder writes a field as a constant (src/nal.c), the decoder
 * reads it and checks it. That is what makes the pair a test of each other
 * rather than two programs that happen to agree.
 */
#ifndef DCC_DEC_NAL_H
#define DCC_DEC_NAL_H

#include "types.h"
#include "bitstream.h"

typedef struct {
    int profile_idc;
    int level_idc;
    int width;              /* derived from pic_width_in_mbs_minus1 */
    int height;             /* derived from pic_height_in_map_units_minus1 */
    int mbs_w, mbs_h;
    int log2_max_frame_num;
    int pic_order_cnt_type;
    int num_ref_frames;
} dec_sps_t;

typedef struct {
    int pic_init_qp;                        /* pic_init_qp_minus26 + 26 */
    int chroma_qp_index_offset;
    int entropy_coding_mode_flag;           /* must be 0: CAVLC */
    int deblocking_filter_control_present;
} dec_pps_t;

typedef struct {
    int first_mb_in_slice;
    int slice_type;                         /* 7 or 2 for I */
    int frame_num;
    int idr_pic_id;
    int slice_qp;                           /* pic_init_qp + slice_qp_delta */
    int disable_deblocking_filter_idc;
} dec_slice_t;

/* One NAL unit located in an Annex B stream. `rbsp` points into a caller
 * buffer holding the payload with emulation-prevention bytes removed. */
typedef struct {
    int nal_ref_idc;
    int nal_unit_type;      /* 1 non-IDR, 5 IDR, 7 SPS, 8 PPS, 9 AUD */
    const u8 *rbsp;
    int rbsp_len;
} dec_nal_t;

/* Find the next NAL unit at or after *pos in an Annex B byte stream, strip
 * emulation-prevention bytes into rbsp_buf, and fill nal.
 * Returns 1 if one was found, 0 at end of stream, negative on a malformed
 * start code. *pos is advanced past the unit. */
int dec_next_nal(const u8 *data, int len, int *pos,
                 u8 *rbsp_buf, int rbsp_cap, dec_nal_t *nal);

/* Parse. Each returns 0, or negative with a message on stderr naming the
 * field and the value, since "unsupported stream" on its own is useless. */
int dec_parse_sps(const dec_nal_t *nal, dec_sps_t *sps);
int dec_parse_pps(const dec_nal_t *nal, dec_pps_t *pps);

/* Parse the slice header and leave br positioned at the first macroblock. */
int dec_parse_slice_header(bitreader_t *br, const dec_nal_t *nal,
                           const dec_sps_t *sps, const dec_pps_t *pps,
                           dec_slice_t *sh);

#endif
