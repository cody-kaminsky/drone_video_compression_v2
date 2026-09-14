/* dec_nal.c — see dec_nal.h. */

#include "dec_nal.h"
#include <stdio.h>
#include <string.h>

#define REJECT(fmt, ...) do { \
    fprintf(stderr, "decode: unsupported stream: " fmt "\n", ##__VA_ARGS__); \
    return -1; \
} while (0)

/* ---------------------------------------------------------- Annex B ----- */

/* Length of the start code at p, or 0 if there is none. */
static int start_code_len(const u8 *p, int avail)
{
    if (avail >= 4 && p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1) return 4;
    if (avail >= 3 && p[0] == 0 && p[1] == 0 && p[2] == 1) return 3;
    return 0;
}

int dec_next_nal(const u8 *data, int len, int *pos,
                 u8 *rbsp_buf, int rbsp_cap, dec_nal_t *nal)
{
    int i = *pos, sc, start, end, zeros, di;

    /* Scan to the next start code. A conforming stream begins with one, but
     * scanning rather than assuming lets a truncated capture resynchronise. */
    while (i < len && (sc = start_code_len(data + i, len - i)) == 0) i++;
    if (i >= len) { *pos = len; return 0; }
    start = i + sc;
    if (start >= len) { *pos = len; return 0; }

    /* The unit runs to the next start code or the end of the stream. */
    end = start;
    while (end < len && start_code_len(data + end, len - end) == 0) end++;

    nal->nal_ref_idc   = (data[start] >> 5) & 3;
    nal->nal_unit_type =  data[start] & 0x1F;
    if ((data[start] & 0x80) != 0)
        REJECT("forbidden_zero_bit set in the NAL header byte");

    /* Strip emulation prevention: 00 00 03 -> 00 00, the inverse of what
     * rbsp_emulation_prevent inserted. */
    di = 0;
    zeros = 0;
    for (i = start + 1; i < end; i++) {
        if (zeros >= 2 && data[i] == 0x03) {
            /* The 0x03 itself is removed; the byte after it is kept. */
            zeros = 0;
            continue;
        }
        if (di >= rbsp_cap) REJECT("NAL payload larger than the %d byte buffer", rbsp_cap);
        rbsp_buf[di++] = data[i];
        zeros = (data[i] == 0) ? zeros + 1 : 0;
    }
    nal->rbsp     = rbsp_buf;
    nal->rbsp_len = di;
    *pos = end;
    return 1;
}

/* ------------------------------------------------------------- SPS ------ */

int dec_parse_sps(const dec_nal_t *nal, dec_sps_t *sps)
{
    bitreader_t br;
    int constraint, reserved, i;
    int w_mbs, h_map, frame_mbs_only, crop;

    br_init(&br, nal->rbsp, nal->rbsp_len);
    sps->profile_idc = (int)br_get_bits(&br, 8);
    if (sps->profile_idc != 66)
        REJECT("profile_idc %d, only Baseline (66) is handled", sps->profile_idc);

    constraint = (int)br_get_bits(&br, 6);
    reserved   = (int)br_get_bits(&br, 2);
    (void)constraint;
    if (reserved != 0) REJECT("reserved_zero_2bits = %d", reserved);

    sps->level_idc = (int)br_get_bits(&br, 8);
    if (br_get_ue(&br) != 0) REJECT("seq_parameter_set_id must be 0");

    sps->log2_max_frame_num = (int)br_get_ue(&br) + 4;
    sps->pic_order_cnt_type = (int)br_get_ue(&br);
    if (sps->pic_order_cnt_type != 2)
        REJECT("pic_order_cnt_type %d, only 2 (no reordering) is handled",
               sps->pic_order_cnt_type);

    sps->num_ref_frames = (int)br_get_ue(&br);
    br_get_bits(&br, 1);                        /* gaps_in_frame_num_allowed */

    w_mbs  = (int)br_get_ue(&br) + 1;
    h_map  = (int)br_get_ue(&br) + 1;
    frame_mbs_only = (int)br_get_bits(&br, 1);
    if (!frame_mbs_only) REJECT("frame_mbs_only_flag is 0; interlace is not handled");
    br_get_bits(&br, 1);                        /* direct_8x8_inference_flag */

    crop = (int)br_get_bits(&br, 1);
    if (crop) REJECT("frame_cropping_flag is 1; cropping is not handled");
    /* vui_parameters_present_flag and the rest are not needed. */

    sps->mbs_w  = w_mbs;
    sps->mbs_h  = h_map;
    sps->width  = w_mbs * 16;
    sps->height = h_map * 16;
    if (br.overflow) REJECT("SPS ended early");
    (void)i;
    return 0;
}

/* ------------------------------------------------------------- PPS ------ */

int dec_parse_pps(const dec_nal_t *nal, dec_pps_t *pps)
{
    bitreader_t br;
    int n_slice_groups;

    br_init(&br, nal->rbsp, nal->rbsp_len);
    if (br_get_ue(&br) != 0) REJECT("pic_parameter_set_id must be 0");
    if (br_get_ue(&br) != 0) REJECT("seq_parameter_set_id must be 0");

    pps->entropy_coding_mode_flag = (int)br_get_bits(&br, 1);
    if (pps->entropy_coding_mode_flag)
        REJECT("entropy_coding_mode_flag is 1; CABAC is not handled");

    br_get_bits(&br, 1);                        /* bottom_field_pic_order... */
    n_slice_groups = (int)br_get_ue(&br) + 1;
    if (n_slice_groups != 1)
        REJECT("num_slice_groups is %d; only 1 is handled", n_slice_groups);

    br_get_ue(&br);                             /* num_ref_idx_l0_default */
    br_get_ue(&br);                             /* num_ref_idx_l1_default */
    br_get_bits(&br, 1);                        /* weighted_pred_flag */
    br_get_bits(&br, 2);                        /* weighted_bipred_idc */

    pps->pic_init_qp = (int)br_get_se(&br) + 26;
    br_get_se(&br);                             /* pic_init_qs_minus26 */
    pps->chroma_qp_index_offset = (int)br_get_se(&br);
    pps->deblocking_filter_control_present = (int)br_get_bits(&br, 1);
    br_get_bits(&br, 1);                        /* constrained_intra_pred_flag */
    br_get_bits(&br, 1);                        /* redundant_pic_cnt_present */

    if (pps->pic_init_qp < 0 || pps->pic_init_qp > 51)
        REJECT("pic_init_qp %d out of range", pps->pic_init_qp);
    if (br.overflow) REJECT("PPS ended early");
    return 0;
}

/* ---------------------------------------------------------- slice ------- */

int dec_parse_slice_header(bitreader_t *br, const dec_nal_t *nal,
                           const dec_sps_t *sps, const dec_pps_t *pps,
                           dec_slice_t *sh)
{
    int is_idr = (nal->nal_unit_type == 5);

    br_init(br, nal->rbsp, nal->rbsp_len);
    sh->first_mb_in_slice = (int)br_get_ue(br);
    if (sh->first_mb_in_slice != 0)
        REJECT("first_mb_in_slice %d; only one slice per picture is handled",
               sh->first_mb_in_slice);

    sh->slice_type = (int)br_get_ue(br);
    /* 2 and 7 are both I; 7 means every slice in the picture is I. */
    if (sh->slice_type != 2 && sh->slice_type != 7)
        REJECT("slice_type %d; this decoder handles I slices only", sh->slice_type);

    if (br_get_ue(br) != 0) REJECT("pic_parameter_set_id must be 0");

    sh->frame_num = (int)br_get_bits(br, sps->log2_max_frame_num);

    if (is_idr) {
        sh->idr_pic_id = (int)br_get_ue(br);
        /* pic_order_cnt_lsb is absent: pic_order_cnt_type is 2. */
        br_get_bits(br, 1);                     /* no_output_of_prior_pics */
        br_get_bits(br, 1);                     /* long_term_reference_flag */
    } else {
        sh->idr_pic_id = -1;
        if (nal->nal_ref_idc != 0)
            br_get_bits(br, 1);                 /* adaptive_ref_pic_marking */
    }

    sh->slice_qp = pps->pic_init_qp + (int)br_get_se(br);
    if (sh->slice_qp < 0 || sh->slice_qp > 51)
        REJECT("slice QP %d out of range", sh->slice_qp);

    sh->disable_deblocking_filter_idc = 0;
    if (pps->deblocking_filter_control_present) {
        sh->disable_deblocking_filter_idc = (int)br_get_ue(br);
        if (sh->disable_deblocking_filter_idc != 1) {
            /* Our encoder always signals 1. A stream asking for filtering
             * would decode to different samples than it reconstructed, so
             * say so rather than produce something subtly wrong. */
            br_get_se(br);                      /* slice_alpha_c0_offset_div2 */
            br_get_se(br);                      /* slice_beta_offset_div2 */
            REJECT("disable_deblocking_filter_idc %d; the in-loop filter is "
                   "not implemented", sh->disable_deblocking_filter_idc);
        }
    }
    if (br->overflow) REJECT("slice header ended early");
    return 0;
}
