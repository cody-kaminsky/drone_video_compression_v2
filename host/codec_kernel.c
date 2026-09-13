/* codec_kernel.c — the generic half of the host driver. See codec_kernel.h.
 *
 * Nothing here knows what codec is behind the registers. Keep it that way:
 * anything that needs to know belongs in the codec's own host module. */

#include "codec_kernel.h"

int dcc_kernel_probe(dcc_kernel_t *k, uintptr_t base, uint32_t want_id,
                     uint32_t aclk_hz)
{
    k->base    = base;
    k->aclk_hz = aclk_hz;
    k->id      = dcc_mmio_read(base, DCC_REG_ID);
    k->version = dcc_mmio_read(base, DCC_REG_VERSION);
    return (k->id == want_id) ? 0 : -1;
}

int dcc_kernel_reset(const dcc_kernel_t *k)
{
    uint64_t t0;
    dcc_mmio_write(k->base, DCC_REG_CTRL, DCC_CTRL_SOFT_RESET);
    /* The kernel holds itself in reset for a fixed number of cycles, so the
     * wait is short; the timeout is here to turn a dead bus into a message
     * rather than a hang. */
    t0 = dcc_time_us();
    while (dcc_mmio_read(k->base, DCC_REG_STATUS) & DCC_STATUS_BUSY) {
        if (dcc_time_us() - t0 > 1000u) return -1;
    }
    dcc_mmio_write(k->base, DCC_REG_DONE_CLR, 1u);
    return 0;
}

void dcc_kernel_configure(const dcc_kernel_t *k, uint32_t config)
{
    dcc_mmio_write(k->base, DCC_REG_CONFIG, config);
}

void dcc_kernel_start(const dcc_kernel_t *k)
{
    dcc_mmio_write(k->base, DCC_REG_CTRL, DCC_CTRL_START);
}

int dcc_kernel_wait_done(const dcc_kernel_t *k, uint32_t timeout_us)
{
    uint64_t t0 = dcc_time_us();
    for (;;) {
        if (dcc_mmio_read(k->base, DCC_REG_STATUS) & DCC_STATUS_DONE) return 0;
        if (dcc_time_us() - t0 > (uint64_t)timeout_us) return -1;
    }
}

void dcc_kernel_clear_done(const dcc_kernel_t *k)
{
    dcc_mmio_write(k->base, DCC_REG_DONE_CLR, 1u);
}

void dcc_kernel_perf(const dcc_kernel_t *k, dcc_kernel_perf_t *p)
{
    p->units  = dcc_mmio_read(k->base, DCC_REG_UNITS);
    p->cycles = dcc_mmio_read(k->base, DCC_REG_CYCLES);
    p->bytes  = dcc_mmio_read(k->base, DCC_REG_BYTES);
}
