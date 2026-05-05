/* hls_pragmas.h — Vitis HLS pragma macros, inert outside synthesis.
 *
 * Vitis HLS recognizes `#pragma HLS ...` directives directly. When the
 * code is compiled by gcc/clang for the C reference test bench, those
 * pragmas would normally trigger an "unknown pragma" warning. We wrap
 * them in macros that expand to `_Pragma(...)` only under the
 * vitis_hls-defined `__SYNTHESIS__` symbol; otherwise they expand to
 * nothing.
 *
 * Usage: HLS_PRAGMA(PIPELINE II=1);
 *        HLS_PRAGMA(ARRAY_PARTITION variable=foo dim=1 complete);
 *
 * Currently this header is only included by hls_top.c. The other HLS
 * source files (encoder.c, line_buffer.c) compile to bit-exact C and
 * don't need pragmas at this stage — pragmas come in once we extract
 * per-stage IPs (M3.2).
 */
#ifndef DCC_HLS_PRAGMAS_H
#define DCC_HLS_PRAGMAS_H

#ifdef __SYNTHESIS__
  #define HLS_PRAGMA_STR(x) _Pragma(#x)
  #define HLS_PRAGMA(x)     HLS_PRAGMA_STR(HLS x)
#else
  #define HLS_PRAGMA(x)
#endif

#endif
