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

/* Maximum frame in our v1 plan: 4K UHD. Used for static-sized buffers in
 * the FPGA-friendly variant. v1 uses dynamic alloc. */
#define MAX_W  3840
#define MAX_H  2160

#endif /* DCC_TYPES_H */
