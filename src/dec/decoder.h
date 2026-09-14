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
