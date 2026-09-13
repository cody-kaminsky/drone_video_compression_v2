/* board_main.c — L4/L5: run a sequence through the real kernel and compare
 * every frame's payload, byte for byte, against what the C reference says it
 * must be.
 *
 * Driven by a manifest in DDR (host/dcc_memmap.h), written by
 * tools/gen_board_sequence.py and loaded over JTAG. Changing the test
 * sequence means reloading, never recompiling.
 *
 * Two things this does NOT do, both deliberately:
 *
 *   - No staging copy. Frames are already in the kernel's stream order,
 *     because the reordering is deterministic and was done once on the
 *     workstation by the same h264_nv12_to_stream() that `make host_test`
 *     checks. MM2S reads each frame where it lies.
 *
 *   - No buffers in bss. The payload, scratch and Annex B buffers live at
 *     fixed DDR addresses from the memory map, so frame size is a runtime
 *     property rather than something baked into the ELF.
 *
 * Every wait in here is bounded by an iteration count rather than by
 * dcc_time_us(). If the A9 global timer is not running, a time-based timeout
 * silently becomes infinite, and an infinite wait looks exactly like a
 * hardware hang. A stall prints the kernel and DMA state instead.
 *
 * NOT BUILT BY THE WORKSTATION MAKEFILE: needs the Vitis BSP. Everything it
 * calls into is exercised on x86 by `make host_test`.
 */

#include "codec_kernel.h"
#include "h264_host.h"
#include "dcc_memmap.h"
#include "xparameters.h"
#include "xaxidma.h"
#include "xil_cache.h"
#include <stdio.h>
#include <string.h>

/* Addresses assigned by scripts/build_zybo_bd.tcl, printed as BD_ADDR lines. */
#ifndef DCC_ENC_BASE
#define DCC_ENC_BASE   0x43C00000u      /* SEG_enc_reg0 */
#endif
#ifndef DCC_DMA_BASE
#define DCC_DMA_BASE   0x40400000u      /* SEG_dma_Reg  */
#endif

/* Roughly a second or two of spinning on a 667 MHz A9. Generous: a 1080p
 * frame is 27.6 ms, so anything near this limit is stuck, not slow. */
#define DCC_SPIN_LIMIT  300000000u

static XAxiDma dma;

static uint8_t *const payload = (uint8_t *)DCC_PAYLOAD_ADDR;
static uint8_t *const scratch = (uint8_t *)DCC_SCRATCH_ADDR;
static uint8_t *const annexb  = (uint8_t *)DCC_ANNEXB_ADDR;

static uint32_t dma_reg(uint32_t chan_off, uint32_t reg_off)
{
    return XAxiDma_ReadReg(DCC_DMA_BASE, chan_off + reg_off);
}

/* Everything worth knowing when something stops moving. The DMA status bits
 * are the ones that actually localise a stall: Halted means the channel
 * stopped, Idle means it finished, and the error bits say whose fault it was. */
static void dump_state(const dcc_kernel_t *k, const char *where,
                       uint32_t rep, uint32_t frame)
{
    uint32_t st   = dcc_mmio_read(k->base, DCC_REG_STATUS);
    uint32_t tsr  = dma_reg(XAXIDMA_TX_OFFSET, XAXIDMA_SR_OFFSET);
    uint32_t rsr  = dma_reg(XAXIDMA_RX_OFFSET, XAXIDMA_SR_OFFSET);

    printf("STALL in %s, repeat %lu frame %lu\n",
           where, (unsigned long)rep, (unsigned long)frame);
    printf("  kernel STATUS %08lx  busy=%lu done=%lu s_axis_tready=%lu m_axis_tvalid=%lu\n",
           (unsigned long)st,
           (unsigned long)((st >> 0) & 1u), (unsigned long)((st >> 1) & 1u),
           (unsigned long)((st >> 2) & 1u), (unsigned long)((st >> 3) & 1u));
    printf("  kernel FRAMES %lu  CYCLES %lu  BYTES %lu\n",
           (unsigned long)dcc_mmio_read(k->base, DCC_REG_UNITS),
           (unsigned long)dcc_mmio_read(k->base, DCC_REG_CYCLES),
           (unsigned long)dcc_mmio_read(k->base, DCC_REG_BYTES));
    printf("  MM2S CR %08lx SR %08lx  halted=%lu idle=%lu err=%lx\n",
           (unsigned long)dma_reg(XAXIDMA_TX_OFFSET, XAXIDMA_CR_OFFSET),
           (unsigned long)tsr, (unsigned long)(tsr & 1u),
           (unsigned long)((tsr >> 1) & 1u), (unsigned long)((tsr >> 4) & 7u));
    printf("  S2MM CR %08lx SR %08lx  halted=%lu idle=%lu err=%lx\n",
           (unsigned long)dma_reg(XAXIDMA_RX_OFFSET, XAXIDMA_CR_OFFSET),
           (unsigned long)rsr, (unsigned long)(rsr & 1u),
           (unsigned long)((rsr >> 1) & 1u), (unsigned long)((rsr >> 4) & 7u));
}

static int dma_init(void)
{
#ifdef XPAR_XAXIDMA_0_BASEADDR
    XAxiDma_Config *cfg = XAxiDma_LookupConfig(DCC_DMA_BASE);
#else
    XAxiDma_Config *cfg = XAxiDma_LookupConfig(XPAR_AXIDMA_0_DEVICE_ID);
#endif
    if (!cfg) { printf("FAIL: no DMA config at %08lx\n",
                       (unsigned long)DCC_DMA_BASE); return -1; }
    if (XAxiDma_CfgInitialize(&dma, cfg) != XST_SUCCESS) {
        printf("FAIL: DMA init\n"); return -1;
    }
    if (XAxiDma_HasSg(&dma)) {
        printf("FAIL: DMA is built for scatter-gather, expected simple mode\n");
        return -1;
    }
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    return 0;
}

/* Encode one frame already sitting in DDR in stream order. Returns the
 * kernel's payload length, or negative. */
static int encode_one(const dcc_kernel_t *k, const dcc_manifest_t *m,
                      const dcc_frame_rec_t *r, dcc_kernel_perf_t *perf,
                      uint32_t rep, uint32_t frame)
{
    uint32_t spin;
    /* Arm S2MM for exactly the expected length rather than the whole
     * buffer, so the channel completes on byte count instead of tlast.
     *
     * This works around a known kernel bug: bit_packer does not assert
     * out_last when the flush finds an empty accumulator, which happens
     * whenever the payload bit count including the stop bit is a multiple
     * of 8 -- one frame in eight. m_axis_tlast then never fires and a
     * length-unbounded S2MM waits forever. See docs/m5-hw-validation.md.
     *
     * It is a validation workaround, not a fix: a real capture path does
     * not know the length in advance. It is sound here because the test
     * already knows what the payload must be, and a kernel that emits a
     * different length is still caught by the BYTES check below. */
    uint32_t rx_len = r->golden_len;

    /* The HP port is not coherent with the A9 data cache. The frame was
     * written by JTAG straight to DDR and the CPU never touches it, so it
     * needs nothing. The payload buffer does: the DMA writes it behind the
     * cache's back. */
    Xil_DCacheInvalidateRange((UINTPTR)payload, DCC_PAYLOAD_MAX);

    dcc_kernel_clear_done(k);
    /* Per-record QP: CONFIG is latched at START, so a sequence can sweep QP
     * without reloading anything. */
    dcc_kernel_configure(k, h264_config_word((int)m->width, (int)m->height,
                                             (int)r->qp));

    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)payload, rx_len,
                               XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
        printf("FAIL: S2MM start refused\n");
        dump_state(k, "S2MM start", rep, frame);
        return -1;
    }
    /* MM2S armed before START. s_axis_tready is held low until START, so an
     * early transfer just stalls harmlessly, and arming first keeps DMA setup
     * latency out of the CYCLES measurement. */
    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)r->frame_addr, r->frame_len,
                               XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
        printf("FAIL: MM2S start refused\n");
        dump_state(k, "MM2S start", rep, frame);
        return -1;
    }
    dcc_kernel_start(k);

    for (spin = 0; ; spin++) {
        if (dcc_mmio_read(k->base, DCC_REG_STATUS) & DCC_STATUS_DONE) break;
        if (spin > DCC_SPIN_LIMIT) {
            printf("FAIL: kernel never asserted DONE\n");
            dump_state(k, "wait for DONE", rep, frame);
            return -1;
        }
    }

    /* Let S2MM retire the beats after tlast. Bounded: this is the wait that
     * has no natural end if the channel errored. */
    for (spin = 0; XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA); spin++) {
        if (spin > DCC_SPIN_LIMIT) {
            printf("FAIL: S2MM never went idle after the kernel finished\n");
            dump_state(k, "wait for S2MM idle", rep, frame);
            return -1;
        }
    }
    /* MM2S too: if it did not drain, the next frame's SimpleTransfer is
     * refused and the failure appears one frame later than its cause. */
    for (spin = 0; XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE); spin++) {
        if (spin > DCC_SPIN_LIMIT) {
            printf("FAIL: MM2S never went idle after the kernel finished\n");
            dump_state(k, "wait for MM2S idle", rep, frame);
            return -1;
        }
    }

    dcc_kernel_perf(k, perf);
    Xil_DCacheInvalidateRange((UINTPTR)payload, DCC_PAYLOAD_MAX);

    /* Simple-mode S2MM cannot report how many bytes it received. It does not
     * have to: the kernel counts its own output and reports it in BYTES. */
    return (int)perf->bytes;
}

static int compare(const uint8_t *got, const uint8_t *want, int n, uint32_t frame)
{
    int i;
    for (i = 0; i < n; i++) {
        if (got[i] != want[i]) {
            printf("FAIL frame %lu: first difference at payload byte %d "
                   "(got %02X, want %02X)\n",
                   (unsigned long)frame, i, got[i], want[i]);
            printf("      that is bit %d of the macroblock layer; re-run the\n"
                   "      reference with DCC_DUMP_MB to find which macroblock\n"
                   "      spans it. See docs/m5-hw-validation.md 5.4.\n", i * 8);
            return -1;
        }
    }
    return 0;
}

int main(void)
{
    const dcc_manifest_t *m = (const dcc_manifest_t *)DCC_MANIFEST_ADDR;
    const dcc_frame_rec_t *recs;
    dcc_kernel_t k;
    dcc_kernel_perf_t perf;
    uint32_t rep, i, mbs, encoded = 0, repeats, spin;
    uint64_t cyc_total = 0, t_a, t_b;
    uint32_t cyc_min = 0xFFFFFFFFu, cyc_max = 0;
    long bytes_total = 0;

    Xil_DCacheEnable();
    printf("\n=== DCC codec kernel, sequence run (L5) ===\n");

    /* Does the global timer actually advance? Nothing below depends on it,
     * but the answer decides whether a reported millisecond figure means
     * anything, and a dead timer is worth knowing about once rather than
     * guessing at later. */
    t_a = dcc_time_us();
    for (spin = 0; spin < 2000000u; spin++) { __asm__ volatile ("nop"); }
    t_b = dcc_time_us();
    printf("timer:    %s (%lu us over 2M nops)\n",
           t_b > t_a ? "running" : "NOT RUNNING -- ms figures are meaningless",
           (unsigned long)(t_b - t_a));

    /* Wait for a manifest rather than failing when it is not there yet. The
     * loader halts the core, writes DDR over JTAG, and resumes into this
     * loop. Invalidate on every pass: JTAG writes DDR behind the data cache,
     * so a cached line would never show the magic arriving. */
    {
        int announced = 0;
        for (spin = 0; ; spin++) {
            Xil_DCacheInvalidateRange((UINTPTR)DCC_MANIFEST_ADDR, 64u * 1024u);
            if (m->magic == DCC_MANIFEST_MAGIC) break;
            if (!announced) {
                printf("waiting for a manifest at %08lx ...\n"
                       "  run the generated load.tcl now\n",
                       (unsigned long)DCC_MANIFEST_ADDR);
                announced = 1;
            }
            if (spin > 20u * DCC_SPIN_LIMIT) {
                printf("FAIL: no manifest (magic reads %08lx, want %08lx)\n",
                       (unsigned long)m->magic, (unsigned long)DCC_MANIFEST_MAGIC);
                return 1;
            }
        }
    }

    if (m->version != DCC_MANIFEST_VER) {
        printf("FAIL: manifest version %lu, this build expects %lu\n",
               (unsigned long)m->version, (unsigned long)DCC_MANIFEST_VER);
        return 1;
    }

    /* Consume the magic. DRAM cells hold their charge for a while, so a
     * power cycle does not reliably clear DDR and a manifest can outlive
     * the data it describes -- which would silently run the next session
     * on stale frames. Clearing it here makes every run require a fresh
     * load, so the sequence on the board is always the one just pushed.
     * Flush it: the write must reach DDR, not sit in the cache. */
    {
        volatile uint32_t *magic = (volatile uint32_t *)DCC_MANIFEST_ADDR;
        *magic = 0u;
        Xil_DCacheFlushRange((UINTPTR)DCC_MANIFEST_ADDR, 64u);
    }
    recs    = (const dcc_frame_rec_t *)(m + 1);
    mbs     = (m->width / 16u) * (m->height / 16u);
    repeats = m->repeats ? m->repeats : 1u;

    if (dcc_kernel_probe(&k, DCC_ENC_BASE, DCC_ID_H264, m->aclk_hz) != 0) {
        printf("FAIL: ID at %08lx reads %08lx, expected %08lx ('H264').\n",
               (unsigned long)DCC_ENC_BASE, (unsigned long)k.id,
               (unsigned long)DCC_ID_H264);
        return 1;
    }
    printf("kernel:   ID 'H264' version %lu.%lu at %08lx, %lu MHz\n",
           (unsigned long)(k.version >> 16), (unsigned long)(k.version & 0xFFFF),
           (unsigned long)DCC_ENC_BASE, (unsigned long)(m->aclk_hz / 1000000u));
    printf("sequence: %lux%lu QP%lu nominal, %lu records x %lu repeats, %lu MBs/frame\n",
           (unsigned long)m->width, (unsigned long)m->height,
           (unsigned long)m->qp, (unsigned long)m->n_frames,
           (unsigned long)repeats, (unsigned long)mbs);

    if (dma_init() != 0) return 1;
    if (dcc_kernel_reset(&k) != 0) { printf("FAIL: kernel reset timeout\n"); return 1; }

    /* Frame records come from DDR written over JTAG, so the same staleness
     * argument as the manifest applies to them. */
    Xil_DCacheInvalidateRange((UINTPTR)recs,
                              m->n_frames * sizeof(dcc_frame_rec_t) + 64u);

    for (rep = 0; rep < repeats; rep++) {
        for (i = 0; i < m->n_frames; i++) {
            const dcc_frame_rec_t *r = &recs[i];
            const uint8_t *golden = (const uint8_t *)r->golden_addr;
            int n;

            if (r->golden_len == 0u || r->golden_len > DCC_PAYLOAD_MAX) {
                printf("FAIL frame %lu: golden_len %lu is not sane "
                       "(buffer is %lu). Manifest or load is bad.\n",
                       (unsigned long)i, (unsigned long)r->golden_len,
                       (unsigned long)DCC_PAYLOAD_MAX);
                return 1;
            }
            Xil_DCacheInvalidateRange((UINTPTR)golden, r->golden_len);

            n = encode_one(&k, m, r, &perf, rep, i);
            if (n < 0) return 1;

            if (n != (int)r->golden_len) {
                printf("FAIL frame %lu: payload is %d bytes, golden is %lu\n",
                       (unsigned long)i, n, (unsigned long)r->golden_len);
                dump_state(&k, "length mismatch", rep, i);
                return 1;
            }
            if (compare(payload, golden, n, i) != 0) return 1;

            cyc_total += perf.cycles;
            if (perf.cycles < cyc_min) cyc_min = perf.cycles;
            if (perf.cycles > cyc_max) cyc_max = perf.cycles;
            bytes_total += n;
            encoded++;

            if (rep == 0)
                printf("  rec %2lu: QP%-3lu %8d bytes, %8lu cycles (%.2f ms, %.0f cy/MB) OK\n",
                       (unsigned long)i, (unsigned long)r->qp, n,
                       (unsigned long)perf.cycles,
                       perf.cycles * 1000.0 / m->aclk_hz,
                       (double)perf.cycles / mbs);
        }
        if (repeats > 1u && ((rep + 1u) % 10u == 0u))
            printf("  ... %lu of %lu repeats, all matching\n",
                   (unsigned long)(rep + 1u), (unsigned long)repeats);
    }

    {
        double avg_cy = (double)cyc_total / encoded;
        double avg_ms = avg_cy * 1000.0 / m->aclk_hz;
        printf("\nPASS: %lu frames encoded, every payload byte-exact with the "
               "C reference\n", (unsigned long)encoded);
        printf("  cycles   avg %.0f  min %lu  max %lu  (%.1f cy/MB)\n",
               avg_cy, (unsigned long)cyc_min, (unsigned long)cyc_max, avg_cy / mbs);
        printf("  frame    %.2f ms -> %.1f fps at %lu MHz\n",
               avg_ms, 1000.0 / avg_ms, (unsigned long)(m->aclk_hz / 1000000u));
        printf("  bitrate  %ld bytes total, %.2f Mbps at 30 fps\n",
               bytes_total, (double)bytes_total / encoded * 8.0 * 30.0 / 1e6);
        printf("  30 fps   %s (needs <= 33.33 ms/frame)\n",
               avg_ms <= 33.33 ? "MET" : "MISSED");
    }

    {
        const dcc_frame_rec_t *r = &recs[m->n_frames - 1u];
        int au = h264_assemble_idr(annexb, (int)DCC_ANNEXB_MAX,
                                   scratch, (int)DCC_SCRATCH_MAX,
                                   payload, (int)r->golden_len,
                                   (int)m->width, (int)m->height, (int)m->qp, 0, 1);
        if (au < 0)
            printf("  assemble returned %d\n", au);
        else
            printf("  last frame assembled into %d bytes of Annex B at %08lx\n",
                   au, (unsigned long)DCC_ANNEXB_ADDR);
    }
    printf("=== SEQUENCE PASSED ===\n");
    return 0;
}
