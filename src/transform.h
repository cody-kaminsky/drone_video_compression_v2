/* transform.h — H.264 4x4 integer DCT and DC Hadamard transforms. */
#ifndef DCC_TRANSFORM_H
#define DCC_TRANSFORM_H

#include "types.h"

/* All 4x4 blocks are stored as 16-element row-major arrays.
 * Index layout: i16 blk[16] where blk[r*4 + c] = element at (row r, col c).
 *
 * FPGA: these functions are pure data-flow (no state). One 4x4 per cycle is
 *       trivially achievable when the body is unrolled. No DSPs needed —
 *       H.264 forward/inverse transforms are shifts and adds only.
 */

/* Forward transforms operate on i16 (residuals fit easily). */
void dct4x4(const i16 in[16], i16 out[16]);
void hadamard4x4(const i16 in[16], i32 out[16]);
void hadamard2x2(const i16 in[4], i16 out[4]);

/* Inverse transforms take i32 (dequantized coefficients can overflow i16
 * at high QP) and return i32 (residual prior to (+32)>>6 in recon). */
void idct4x4    (const i32 in[16], i32 out[16]);
void ihadamard4x4(const i32 in[16], i32 out[16]);
void ihadamard2x2(const i32 in[4],  i32 out[4]);

#endif
