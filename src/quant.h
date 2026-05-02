/* quant.h — H.264 quantization (forward and inverse). */
#ifndef DCC_QUANT_H
#define DCC_QUANT_H

#include "types.h"

/* AC blocks (4x4 transformed coefficients).
 * is_intra: rounding offset is (1<<qbits)/3 for intra, /6 for inter (we are
 * I-only so always pass 1). */
/* Forward path takes i16 (residuals/coefs/levels all fit easily).
 * Inverse path returns i32 — at high QP the dequantized coefficients
 * exceed i16 range; subsequent iDCT/iHadamard preserves enough precision
 * that the final >>6 in reconstruction stays within 8-bit pixel range. */
void quant_4x4   (const i16 coef[16], i16 level[16], int qp, int is_intra);
void iquant_4x4  (const i16 level[16], i32 coef[16], int qp);

void quant_dc_4x4 (const i32 coef[16], i16 level[16], int qp, int is_intra);
void iquant_dc_4x4(const i16 level[16], i32 coef[16], int qp);

void quant_dc_2x2 (const i16 coef[4], i16 level[4], int qp, int is_intra);
void iquant_dc_2x2(const i16 level[4], i32 coef[4], int qp);

/* Spec table 8-9: maps (luma_qp + chroma_qp_index_offset) clipped to 0..51
 * to the chroma QP value. */
int chroma_qp(int luma_qp, int chroma_qp_index_offset);

#endif
