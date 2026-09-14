/* decoder.h — a decoder matched to this project's encoder.
 *
 * Scope, deliberately narrow: Baseline profile, intra only, CAVLC, one slice
 * per picture, 4:2:0 8-bit, no deblocking. That is exactly what src/encoder.c
 * emits and what the FPGA kernel emits. Anything else is rejected with a
 * reason.
 *
 * Almost none of the signal processing is new. Prediction (src/intra.c),
 * the inverse transforms (src/transform.c), dequantisation (src/quant.c) and
 * CAVLC block decoding (src/cavlc.c) are the encoder's own routines, used
 * unchanged. What the decoder adds is the parse: turning a byte stream back
 * into the macroblock decisions the encoder made.
 *
 * That reuse is the point. A decoder written from the same kernels cannot
 * catch a bug in those kernels -- it would make the same mistake twice. What
 * it does catch is everything in the bitstream layer: syntax, ordering, nC
 * derivation, mode prediction, CBP mapping. Those are where the encoder's
 * bugs have actually been. For independent confirmation of the kernels there
 * is still ffmpeg, which shares no code with us at all.
 */
#ifndef DCC_DECODER_H
#define DCC_DECODER_H

#include "types.h"
#include "bitstream.h"

/* One macroblock header, and everything the parse needs to know about its
 * neighbours. Exposed so hardware vector generators can be held to the same
 * routine the golden decoder uses; see dec_mb_header below. */
typedef struct {
    /* in */
    int qp_in;
    int mode4_top[4];     /* 4x4 modes of the row above, by column; DC(2) if absent */
    int mode4_left[4];    /* 4x4 modes of the column to the left, by row */
    int avail_top, avail_left;
    /* out */
    int is_i4x4;
    int mode16;           /* I_16x16 luma mode; 0 for I_4x4 */
    int modes4[16];       /* 4x4 modes, raster inside the macroblock */
    int mode_chroma;
    int cbp_luma;         /* 4 bits, one per 8x8 quadrant */
    int cbp_chroma;       /* 0 none, 1 DC only, 2 DC and AC */
    int has_residual;     /* whether mb_qp_delta was present */
    int qp_out;
} mb_header_t;

/* Parse the macroblock header of spec 7.3.5. Returns 0, or -1 with the reason
 * on stderr for syntax this decoder does not handle. */
int dec_mb_header(bitreader_t *br, mb_header_t *h);

/* Every residual block of one macroblock, in bitstream order, with the
 * neighbour total_coeff context the nC derivation of spec 9.2.1.1 needs.
 * Coefficients come out in zigzag order with the I_16x16 and chroma AC shift
 * already applied, so every block is a full 16-coefficient vector. */
typedef struct {
    /* in */
    int is_i4x4, cbp_luma, cbp_chroma;
    int avail_top, avail_left;
    int nc_top[4], nc_left[4];       /* luma, in 4x4 units */
    int ncu_top[2], ncu_left[2];     /* chroma U */
    int ncv_top[2], ncv_left[2];     /* chroma V */
    /* out */
    i16 luma_dc[16];                 /* I_16x16 only, zigzag */
    i16 luma[16][16];                /* [raster block][zigzag coefficient] */
    i16 chroma_dc[2][4];             /* [component][coefficient] */
    i16 chroma_ac[2][4][16];         /* [component][raster block][zigzag] */
    int nc_out[16];                  /* luma total_coeff, raster */
    int ncu_out[4], ncv_out[4];
} mb_residual_t;

/* Parse one macroblock's residual blocks. Returns 0, or -1 with the reason on
 * stderr. Reads bits and nothing else: the entropy layer never depends on a
 * reconstructed sample. */
int dec_mb_residual(bitreader_t *br, mb_residual_t *r);

typedef struct {
    int width, height;
    int mbs_w, mbs_h;
    int frames;            /* pictures decoded */
    int slice_qp;          /* QP of the last slice header */
    long bytes_in;         /* Annex B bytes consumed */
} dec_stats_t;

/* Decode every access unit in an Annex B byte stream.
 *
 * Calls on_frame after each picture with the reconstructed NV12 planes:
 * luma width*height, then interleaved CbCr at width*(height/2). The buffers
 * belong to the decoder and are valid until the callback returns.
 *
 * Returns 0, or negative on a stream this decoder does not handle (with the
 * reason on stderr). Partial output is kept: a truncated stream decodes the
 * frames it does contain. */
int dcc_decode_stream(const u8 *data, int len,
                      void (*on_frame)(const u8 *y, const u8 *uv,
                                       int width, int height, void *ctx),
                      void *ctx, dec_stats_t *stats);

#endif
