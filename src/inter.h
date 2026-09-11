/* inter.h — inter prediction for P slices: motion compensation (spec
 * 8.4.2.2), motion vector prediction (8.4.1.3), motion estimation.
 *
 * Motion vectors are in quarter-sample units for luma; chroma uses the same
 * vector at eighth-sample resolution (4:2:0). Only 16x16 partitions and a
 * single reference (the previous frame) are used: the hardware target is a
 * one-reference, low-latency encoder with rolling intra refresh.
 *
 * The reference planes are padded copies of the previous reconstruction so
 * that every sample fetch inside the search window is a plain load; the
 * padding replicates edge samples, which is exactly the coordinate clamping
 * the spec prescribes for vectors that point outside the picture.
 */
#ifndef DCC_INTER_H
#define DCC_INTER_H

#include "types.h"

/* Padding around the reference planes, in luma samples. Vectors are kept
 * so that the 16x16 block plus the 6-tap margin stays inside. */
#define REF_PAD      64
#define REF_PAD_C    (REF_PAD / 2)

typedef struct {
    u8 *y;  int stride_y;      /* padded luma plane, (0,0) = picture origin */
    u8 *u;  int stride_c;      /* padded chroma planes (separate U and V) */
    u8 *v;
    int width, height;         /* picture size in luma samples */
} ref_planes_t;

/* Build the padded reference planes from a reconstructed NV12 frame.
 * pad_y must hold (width + 2*REF_PAD) * (height + 2*REF_PAD) bytes,
 * pad_u / pad_v (width/2 + 2*REF_PAD_C) * (height/2 + 2*REF_PAD_C) each. */
void ref_build(ref_planes_t *rp, u8 *pad_y, u8 *pad_u, u8 *pad_v,
               const u8 *recon_y, int stride_y,
               const u8 *recon_uv, int stride_uv, int width, int height);

/* Motion-compensated prediction of the 16x16 luma block at MB (mb_r, mb_c)
 * for vector (mvx, mvy) in quarter samples: spec 8.4.2.2.1 (6-tap half
 * samples, averaged quarter samples). pred[256] row-major. */
void mc_luma_16x16(const ref_planes_t *rp, int mb_r, int mb_c,
                   int mvx, int mvy, u8 pred[256]);

/* Chroma 8x8 prediction for both planes, spec 8.4.2.2.2 (bilinear, 1/8). */
void mc_chroma_8x8(const ref_planes_t *rp, int mb_r, int mb_c,
                   int mvx, int mvy, u8 pred_u[64], u8 pred_v[64]);

/* Per-MB motion data of the current picture, for vector prediction. */
typedef struct {
    i16 *mvx;          /* quarter samples, per MB (raster) */
    i16 *mvy;
    u8  *is_intra;     /* 1 = intra coded (no vector), 0 = inter */
    int  mbs_w, mbs_h;
} mv_field_t;

/* Predicted vector (spec 8.4.1.3, single reference index 0, 16x16). All
 * neighbours of the current MB must already be coded. */
void mv_predict_16x16(const mv_field_t *mf, int mb_r, int mb_c,
                      int *pred_x, int *pred_y);

/* Vector inferred for P_Skip (spec 8.4.1.1). */
void mv_skip_16x16(const mv_field_t *mf, int mb_r, int mb_c,
                   int *mvx, int *mvy);

/* Bits of se(v) for a vector difference component. */
int mvd_bits(int d);

/* Motion estimation: full search over integer vectors within +/- range
 * samples of the predictor, then half- and quarter-sample refinement.
 * Cost = SAD (integer) / SATD (fractional) + lambda * mvd bits.
 * Returns the best vector in quarter samples and its cost. */
typedef struct {
    int range;         /* integer search range in samples (e.g. 16) */
    int lambda;        /* rate weight for the vector bits */
} me_params_t;

int me_search_16x16(const ref_planes_t *rp, const u8 src[256],
                    int mb_r, int mb_c, int pred_x, int pred_y,
                    const me_params_t *mp, int *best_x, int *best_y);

#endif
