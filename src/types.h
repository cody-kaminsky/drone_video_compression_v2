/* types.h — common typedefs and constants for the codec.
 *
 * FPGA: keep this header free of malloc/stdio so it can be reused in the
 *       FPGA-friendly C variant unchanged.
 */
#ifndef DCC_TYPES_H
#define DCC_TYPES_H

#include <stdint.h>
#include <stddef.h>

typedef int16_t  i16;
typedef int32_t  i32;
typedef int64_t  i64;
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define MB_W   16
#define MB_H   16
#define BLK    4

/* Maximum frame: sized to cover the architecture's 4K UHD target plus the
 * tall drone aerial samples in the validation set (e.g. agadir 3400x2671,
 * alicudi 3840x2586). The static arena in encoder.c allocates by these. */
#define MAX_W  3840
#define MAX_H  2688

#endif /* DCC_TYPES_H */
