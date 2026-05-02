/* psnr.c — PSNR helpers for the test bench.
 * See psnr.h for rationale (off-kernel measurement function).
 */
#include "psnr.h"
#include <math.h>

double psnr_plane(const u8 *a, int stride_a,
                  const u8 *b, int stride_b, int w, int h)
{
    i64 sse = 0;
    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++) {
            int d = (int)a[i * stride_a + j] - (int)b[i * stride_b + j];
            sse += (i64)(d * d);
        }
    if (sse == 0) return 99.0;
    double mse = (double)sse / ((double)w * h);
    return 10.0 * log10((255.0 * 255.0) / mse);
}

double psnr_chroma_component(const u8 *a_uv, int stride_a,
                             const u8 *b_uv, int stride_b,
                             int w, int h, int comp_offset)
{
    i64 sse = 0;
    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++) {
            int d = (int)a_uv[i * stride_a + j*2 + comp_offset]
                  - (int)b_uv[i * stride_b + j*2 + comp_offset];
            sse += (i64)(d * d);
        }
    if (sse == 0) return 99.0;
    double mse = (double)sse / ((double)w * h);
    return 10.0 * log10((255.0 * 255.0) / mse);
}
