/* psnr.h — PSNR measurement for the test bench.
 *
 * PSNR is a measurement function applied AFTER encoding to compare
 * source vs reconstruction. The FPGA encoder IP does not compute it
 * (architecture.txt §10 register map only exposes BS_BYTES_OUT and perf
 * counters). Hence PSNR lives outside the kernel — host-side only.
 */
#ifndef DCC_PSNR_H
#define DCC_PSNR_H

#include "types.h"

/* Y-plane PSNR (dB). Returns 99.0 if buffers are identical. */
double psnr_plane(const u8 *a, int stride_a,
                  const u8 *b, int stride_b, int w, int h);

/* PSNR of one component (Cb or Cr) inside an interleaved NV12 plane.
 *   comp_offset: 0 for Cb, 1 for Cr.
 *   w, h:        sub-sampled chroma dimensions (width/2, height/2 for 4:2:0). */
double psnr_chroma_component(const u8 *a_uv, int stride_a,
                             const u8 *b_uv, int stride_b,
                             int w, int h, int comp_offset);

#endif
