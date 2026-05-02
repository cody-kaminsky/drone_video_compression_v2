/* nal.c — SPS/PPS/slice header emission for the I-only Baseline profile. */

#include "nal.h"
#include "bitstream.h"
#include <string.h>

/* Pick a level_idc per spec Annex A based on (mbs_per_sec, max_frame_size).
 * Conservative choices for our targets. */
static int pick_level_idc(int width, int height)
{
    int mbs = (width / 16) * (height / 16);
    /* Common resolution → level mapping for 30 fps:
     *   1080p (8160 MBs/frame, 244800 MBs/s) → Level 4.0 (max 245760 MBs/s)
     *   4K30  (32400 MBs/frame, 972000 MBs/s) → Level 5.2 (max 2073600 MBs/s) */
    if (mbs <= 8192)  return 40;   /* up to 1080p30 */
    if (mbs <= 22080) return 50;   /* up to 2560x1664 */
    return 52;                      /* 4K30 needs 5.2 */
}

int nal_write_sps(u8 *dst, int dst_cap, int width, int height, int qp_init)
{
    /* Build RBSP into a scratch buffer, then escape + frame. */
    u8 rbsp[64];
    bitstream_t bs;
    bs_init(&bs, rbsp, sizeof rbsp);

    bs_put_bits(&bs, 66, 8);          /* profile_idc = Baseline */
    bs_put_bits(&bs, 1, 1);           /* constraint_set0_flag */
    bs_put_bits(&bs, 0, 1);           /* constraint_set1_flag */
    bs_put_bits(&bs, 0, 1);           /* constraint_set2_flag */
    bs_put_bits(&bs, 0, 1);           /* constraint_set3_flag */
    bs_put_bits(&bs, 0, 1);           /* constraint_set4_flag */
    bs_put_bits(&bs, 0, 1);           /* constraint_set5_flag */
    bs_put_bits(&bs, 0, 2);           /* reserved_zero_2bits */
    bs_put_bits(&bs, pick_level_idc(width, height), 8);
    bs_put_ue(&bs, 0);                /* seq_parameter_set_id */
    bs_put_ue(&bs, 0);                /* log2_max_frame_num_minus4 (4-bit frame_num) */
    bs_put_ue(&bs, 2);                /* pic_order_cnt_type = 2 */
    bs_put_ue(&bs, 1);                /* num_ref_frames (legal min for I-only) */
    bs_put_bits(&bs, 0, 1);           /* gaps_in_frame_num_value_allowed_flag */
    bs_put_ue(&bs, (width  / 16) - 1); /* pic_width_in_mbs_minus1 */
    bs_put_ue(&bs, (height / 16) - 1); /* pic_height_in_map_units_minus1 */
    bs_put_bits(&bs, 1, 1);           /* frame_mbs_only_flag */
    bs_put_bits(&bs, 1, 1);           /* direct_8x8_inference_flag */
    bs_put_bits(&bs, 0, 1);           /* frame_cropping_flag */
    bs_put_bits(&bs, 0, 1);           /* vui_parameters_present_flag */
    bs_rbsp_trailing(&bs);
    int rbsp_len = bs_byte_count(&bs);
    if (bs.overflow) return -1;
    (void)qp_init;

    /* Write to dst: start code + NAL header + escaped RBSP. */
    if (dst_cap < 5 + rbsp_len * 2) return -1;
    dst[0] = 0x00; dst[1] = 0x00; dst[2] = 0x00; dst[3] = 0x01;
    dst[4] = 0x67;                    /* nal_ref_idc=3, type=7 */
    int n = rbsp_emulation_prevent(dst + 5, dst_cap - 5, rbsp, rbsp_len);
    if (n < 0) return -1;
    return 5 + n;
}

int nal_write_pps(u8 *dst, int dst_cap, int qp_init)
{
    u8 rbsp[32];
    bitstream_t bs;
    bs_init(&bs, rbsp, sizeof rbsp);

    bs_put_ue(&bs, 0);                /* pic_parameter_set_id */
    bs_put_ue(&bs, 0);                /* seq_parameter_set_id */
    bs_put_bits(&bs, 0, 1);           /* entropy_coding_mode_flag = CAVLC */
    bs_put_bits(&bs, 0, 1);           /* bottom_field_pic_order_in_frame_present_flag */
    bs_put_ue(&bs, 0);                /* num_slice_groups_minus1 = 0 */
    bs_put_ue(&bs, 0);                /* num_ref_idx_l0_default_active_minus1 */
    bs_put_ue(&bs, 0);                /* num_ref_idx_l1_default_active_minus1 */
    bs_put_bits(&bs, 0, 1);           /* weighted_pred_flag */
    bs_put_bits(&bs, 0, 2);           /* weighted_bipred_idc */
    bs_put_se(&bs, qp_init - 26);     /* pic_init_qp_minus26 */
    bs_put_se(&bs, 0);                /* pic_init_qs_minus26 */
    bs_put_se(&bs, 0);                /* chroma_qp_index_offset */
    bs_put_bits(&bs, 1, 1);           /* deblocking_filter_control_present_flag */
    bs_put_bits(&bs, 0, 1);           /* constrained_intra_pred_flag */
    bs_put_bits(&bs, 0, 1);           /* redundant_pic_cnt_present_flag */
    bs_rbsp_trailing(&bs);
    int rbsp_len = bs_byte_count(&bs);
    if (bs.overflow) return -1;

    if (dst_cap < 5 + rbsp_len * 2) return -1;
    dst[0] = 0x00; dst[1] = 0x00; dst[2] = 0x00; dst[3] = 0x01;
    dst[4] = 0x68;                    /* nal_ref_idc=3, type=8 */
    int n = rbsp_emulation_prevent(dst + 5, dst_cap - 5, rbsp, rbsp_len);
    if (n < 0) return -1;
    return 5 + n;
}

int nal_write_slice_header(u8 *rbsp_dst, int dst_cap, int frame_num,
                           int idr_pic_id, int qp_init)
{
    bitstream_t bs;
    bs_init(&bs, rbsp_dst, dst_cap);

    bs_put_ue(&bs, 0);                /* first_mb_in_slice = 0 */
    bs_put_ue(&bs, 7);                /* slice_type = 7 (I, all-same) */
    bs_put_ue(&bs, 0);                /* pic_parameter_set_id */
    bs_put_bits(&bs, frame_num & 0xF, 4);  /* frame_num (4-bit per SPS) */
    /* IDR: emit idr_pic_id (since this is an IDR slice) */
    bs_put_ue(&bs, idr_pic_id);
    /* pic_order_cnt_type=2 → no pic_order_cnt_lsb */
    /* IDR slice: dec_ref_pic_marking */
    bs_put_bits(&bs, 0, 1);           /* no_output_of_prior_pics_flag */
    bs_put_bits(&bs, 0, 1);           /* long_term_reference_flag */
    bs_put_se(&bs, 0);                /* slice_qp_delta = 0 */
    bs_put_ue(&bs, 0);                /* disable_deblocking_filter_idc = 0 */
    bs_put_se(&bs, 0);                /* slice_alpha_c0_offset_div2 */
    bs_put_se(&bs, 0);                /* slice_beta_offset_div2 */

    (void)qp_init;
    /* Note: do NOT call bs_rbsp_trailing here. The slice_data follows in
     * the same RBSP and the trailing bits are appended after slice_data. */
    /* Return bit count so far so caller can continue appending. */
    return bs.byte_pos * 8 + bs.n_in_cur;
}

int nal_emit_idr(u8 *dst, int dst_cap, const u8 *rbsp, int rbsp_len)
{
    if (dst_cap < 5 + rbsp_len * 2) return -1;
    dst[0] = 0x00; dst[1] = 0x00; dst[2] = 0x00; dst[3] = 0x01;
    dst[4] = 0x65;                    /* nal_ref_idc=3, type=5 (IDR slice) */
    int n = rbsp_emulation_prevent(dst + 5, dst_cap - 5, rbsp, rbsp_len);
    if (n < 0) return -1;
    return 5 + n;
}
