/* transform.c — H.264 4x4 transforms.
 *
 * Reference: H.264 spec sections 8.5.6, 8.5.10, 8.5.11.
 * The "core" forward DCT here matches what JM/x264 use; the post-scaling
 * factors are absorbed into the quantization tables in quant.c.
 */
#include "transform.h"

void dct4x4(const i16 in[16], i16 out[16])
{
    i32 t[16];
    /* Row pass */
    for (int i = 0; i < 4; i++) {
        const i16 *r = &in[i * 4];
        i32 a = r[0] + r[3];
        i32 b = r[1] + r[2];
        i32 c = r[1] - r[2];
        i32 d = r[0] - r[3];
        t[i*4 + 0] = a + b;
        t[i*4 + 1] = (d << 1) + c;
        t[i*4 + 2] = a - b;
        t[i*4 + 3] = d - (c << 1);
    }
    /* Column pass */
    for (int j = 0; j < 4; j++) {
        i32 a = t[0*4 + j] + t[3*4 + j];
        i32 b = t[1*4 + j] + t[2*4 + j];
        i32 c = t[1*4 + j] - t[2*4 + j];
        i32 d = t[0*4 + j] - t[3*4 + j];
        out[0*4 + j] = (i16)(a + b);
        out[1*4 + j] = (i16)((d << 1) + c);
        out[2*4 + j] = (i16)(a - b);
        out[3*4 + j] = (i16)(d - (c << 1));
    }
}

void idct4x4(const i32 in[16], i32 out[16])
{
    i32 t[16];
    /* Row pass */
    for (int i = 0; i < 4; i++) {
        const i32 *r = &in[i * 4];
        i32 a = r[0] + r[2];
        i32 b = r[0] - r[2];
        i32 c = (r[1] >> 1) - r[3];
        i32 d = r[1] + (r[3] >> 1);
        t[i*4 + 0] = a + d;
        t[i*4 + 1] = b + c;
        t[i*4 + 2] = b - c;
        t[i*4 + 3] = a - d;
    }
    /* Column pass — output is the residual scaled by 64. Caller adds
     * (val + 32) >> 6 to predicted samples for final reconstruction. */
    for (int j = 0; j < 4; j++) {
        i32 a = t[0*4 + j] + t[2*4 + j];
        i32 b = t[0*4 + j] - t[2*4 + j];
        i32 c = (t[1*4 + j] >> 1) - t[3*4 + j];
        i32 d = t[1*4 + j] + (t[3*4 + j] >> 1);
        out[0*4 + j] = a + d;
        out[1*4 + j] = b + c;
        out[2*4 + j] = b - c;
        out[3*4 + j] = a - d;
    }
}

/* 4x4 Hadamard. Same butterfly as DCT but without the doubling factors,
 * since DC coefficients are already in a smaller dynamic range. */
void hadamard4x4(const i16 in[16], i32 out[16])
{
    i32 t[16];
    for (int i = 0; i < 4; i++) {
        const i16 *r = &in[i * 4];
        i32 a = r[0] + r[3];
        i32 b = r[1] + r[2];
        i32 c = r[1] - r[2];
        i32 d = r[0] - r[3];
        t[i*4 + 0] = a + b;
        t[i*4 + 1] = d + c;
        t[i*4 + 2] = a - b;
        t[i*4 + 3] = d - c;
    }
    for (int j = 0; j < 4; j++) {
        i32 a = t[0*4 + j] + t[3*4 + j];
        i32 b = t[1*4 + j] + t[2*4 + j];
        i32 c = t[1*4 + j] - t[2*4 + j];
        i32 d = t[0*4 + j] - t[3*4 + j];
        out[0*4 + j] = a + b;
        out[1*4 + j] = d + c;
        out[2*4 + j] = a - b;
        out[3*4 + j] = d - c;
    }
}

void ihadamard4x4(const i32 in[16], i32 out[16])
{
    /* Hadamard is its own inverse up to scaling that the dequantizer
     * handles. Implement directly on i32 buffers. */
    i32 t[16];
    for (int i = 0; i < 4; i++) {
        const i32 *r = &in[i * 4];
        i32 a = r[0] + r[3];
        i32 b = r[1] + r[2];
        i32 c = r[1] - r[2];
        i32 d = r[0] - r[3];
        t[i*4 + 0] = a + b;
        t[i*4 + 1] = d + c;
        t[i*4 + 2] = a - b;
        t[i*4 + 3] = d - c;
    }
    for (int j = 0; j < 4; j++) {
        i32 a = t[0*4 + j] + t[3*4 + j];
        i32 b = t[1*4 + j] + t[2*4 + j];
        i32 c = t[1*4 + j] - t[2*4 + j];
        i32 d = t[0*4 + j] - t[3*4 + j];
        out[0*4 + j] = a + b;
        out[1*4 + j] = d + c;
        out[2*4 + j] = a - b;
        out[3*4 + j] = d - c;
    }
}

void hadamard2x2(const i16 in[4], i16 out[4])
{
    i32 a = in[0] + in[1];
    i32 b = in[0] - in[1];
    i32 c = in[2] + in[3];
    i32 d = in[2] - in[3];
    out[0] = (i16)(a + c);
    out[1] = (i16)(b + d);
    out[2] = (i16)(a - c);
    out[3] = (i16)(b - d);
}

void ihadamard2x2(const i32 in[4], i32 out[4])
{
    i32 a = in[0] + in[1];
    i32 b = in[0] - in[1];
    i32 c = in[2] + in[3];
    i32 d = in[2] - in[3];
    out[0] = a + c;
    out[1] = b + d;
    out[2] = a - c;
    out[3] = b - d;
}
