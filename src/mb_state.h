/* mb_state.h — per-macroblock pipeline state.
 *
 * mb_state_t is a typed snapshot of every value that flows along the per-MB
 * encode pipeline. The encode_mb_emit path mutates exactly one mb_state_t
 * through eight stages (architecture.txt §8/§9):
 *
 *   stage 0  mb_fetch              src_y/u/v, neighbor gather
 *   stage 1  mb_mode_decide        mode16, mode_chroma, pred_y/u/v
 *   stage 2  mb_residual           res_y/u/v = src - pred
 *   stage 3  mb_transform          dct_ac_*, dc_extract_*, dc_had_*
 *   stage 4  mb_quantize           ac_levels_*, dc_levels_*
 *   stage 5  (iQ + iT, internal)
 *   stage 6  mb_reconstruct        recon_y/u/v
 *   stage 7  mb_cavlc_emit         emit bits, update neighbor nC
 *
 * In hardware each field becomes a FIFO entry between stages — the fields
 * are NOT all live at the same time, the C model just makes them all
 * available so the stage interfaces stay typed and verifiable.
 *
 * Indexing convention for arrays of 4x4 blocks:
 *   luma:    block_idx = br*4 + bc, br/bc in 0..3   (16 blocks per MB)
 *   chroma:  block_idx = br*2 + bc, br/bc in 0..1   (4 blocks per 8x8)
 *   each 4x4 stored as 16 elements in row-major order: [r*4 + c]
 */
#ifndef DCC_MB_STATE_H
#define DCC_MB_STATE_H

#include "types.h"

typedef struct {
    /* ===== position + QP ===== */
    int mb_r, mb_c;
    int qp_y, qp_c;

    /* ===== stage 0 outputs: source samples ===== */
    u8 src_y[256];    /* 16x16 luma, row-major */
    u8 src_u[64];     /* 8x8 chroma U */
    u8 src_v[64];     /* 8x8 chroma V */

    /* ===== stage 0 outputs: prediction neighbors ===== */
    u8 luma_top[16];
    u8 luma_left[16];
    u8 luma_tl;
    int luma_avail_top;
    int luma_avail_left;
    int luma_avail_tl;

    u8 cu_top[8],  cu_left[8],  cu_tl;
    u8 cv_top[8],  cv_left[8],  cv_tl;
    int chroma_avail_top;
    int chroma_avail_left;
    int chroma_avail_tl;

    /* ===== stage 1 outputs: chosen modes + predicted samples ===== */
    int mode16;        /* I_16x16 luma mode (I16_VERTICAL/.../I16_PLANE) */
    int mode_chroma;   /* chroma 8x8 mode (IC_DC/.../IC_PLANE) */
    u8 pred_y[256];
    u8 pred_u[64];
    u8 pred_v[64];

    /* ===== stage 2 outputs: residuals ===== */
    i16 res_y[16][16];   /* 16 luma blocks of 16 elements */
    i16 res_u[4][16];    /* 4 chroma U blocks */
    i16 res_v[4][16];    /* 4 chroma V blocks */

    /* ===== stage 3 outputs: transform coefficients ===== */
    /* dct_ac_*: per-block DCT output with the DC slot ([0]) zeroed.
     * dc_extract_*: extracted DC coefficient per block (pre-Hadamard).
     * dc_had_*: DC coefficients after the Hadamard transform.
     *   - luma: hadamard4x4 returns i32 (range can exceed i16)
     *   - chroma: hadamard2x2 stays in i16 (smaller dynamic range)         */
    i16 dct_ac_y[16][16];
    i16 dc_extract_y[16];
    i32 dc_had_y[16];

    i16 dct_ac_u[4][16];
    i16 dct_ac_v[4][16];
    i16 dc_extract_u[4];
    i16 dc_extract_v[4];
    i16 dc_had_u[4];
    i16 dc_had_v[4];

    /* ===== stage 4 outputs: quantized levels (raster order) ===== */
    i16 ac_levels_y[16][16];
    i16 dc_levels_y[16];
    i16 ac_levels_u[4][16];
    i16 ac_levels_v[4][16];
    i16 dc_levels_u[4];
    i16 dc_levels_v[4];

    /* ===== stage 6 outputs: reconstructed samples ===== */
    u8 recon_y[256];
    u8 recon_u[64];
    u8 recon_v[64];

    /* ===== derived flags used by stage 7 (cavlc) ===== */
    int cbp_luma;     /* 0 or 1 (any nonzero luma AC) */
    int cbp_chroma;   /* 0=none, 1=DC only, 2=AC */
} mb_state_t;

#endif
