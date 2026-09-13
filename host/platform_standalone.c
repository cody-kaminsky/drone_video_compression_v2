/* platform_standalone.c — the Xilinx bare-metal implementation of the three
 * platform calls declared in codec_kernel.h. Nothing else in the host code
 * touches hardware, so porting to Linux/UIO or to a different SoC means
 * replacing this file and nothing more.
 *
 * Build: add to a Vitis standalone application against the XSA produced by
 * scripts/build_zybo_bd.tcl. Not built by the workstation Makefile -- it
 * needs the BSP headers.
 */

#include "codec_kernel.h"
#include "xil_io.h"
#include "xtime_l.h"

uint32_t dcc_mmio_read(uintptr_t base, uint32_t off)
{
    return Xil_In32(base + off);
}

void dcc_mmio_write(uintptr_t base, uint32_t off, uint32_t val)
{
    Xil_Out32(base + off, val);
}

uint64_t dcc_time_us(void)
{
    XTime t;
    XTime_GetTime(&t);
    /* The A9 global timer runs at half the CPU clock. COUNTS_PER_SECOND is
     * defined by the BSP for this part; do the divide in 64 bits or a 1080p
     * frame's worth of counts overflows. */
    return (uint64_t)t / (COUNTS_PER_SECOND / 1000000ull);
}
