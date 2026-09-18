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
/* xiltimer.h, not xtime_l.h. The latter exists on UltraScale+ (A53/R5) but
 * not in a Cortex-A9 system-device-tree BSP, where the same XTime_GetTime and
 * COUNTS_PER_SECOND come from xiltimer.h (which pulls in xtimer_config.h). */
#include "xiltimer.h"

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
    /* COUNTS_PER_SECOND is XPAR_CPU_CORE_CLOCK_FREQ_HZ/2 here: the A9 global
     * timer runs at half the CPU clock. Parenthesise it, because the macro
     * expands to a bare division and would otherwise reassociate. Do the
     * divide in 64 bits or a frame's worth of counts overflows. */
    return (uint64_t)t / ((uint64_t)(COUNTS_PER_SECOND) / 1000000ull);
}
