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
 *     checks. MM2S reads each frame where it lies. That removes about 3 MB of
 *     memcpy per 1080p frame, which was the main thing eating the 33 ms
 *     budget at 30 fps -- and the main argument for scatter-gather.
 *
 *   - No buffers in bss. The payload, scratch and Annex B buffers live at
 *     fixed DDR addresses from the memory map, so frame size is a runtime
 *     property rather than something baked into the ELF.
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

/* Addresses assigned by scripts/build_zybo_bd.tcl, printed as BD_ADDR lines.
 * Literals rather than XPAR_* symbols, whose spelling depends on the block
 * design cell name and the Vitis release. */
#ifndef DCC_ENC_BASE
#define DCC_ENC_BASE   0x43C00000u      /* SEG_enc_reg0 */
#endif
#ifndef DCC_DMA_BASE
#define DCC_DMA_BASE   0x40400000u      /* SEG_dma_Reg  */
#endif

static XAxiDma dma;

static uint8_t *const payload = (uint8_t *)DCC_PAYLOAD_ADDR;
static uint8_t *const scratch = (uint8_t *)DCC_SCRATCH_ADDR;
static uint8_t *const annexb  = (uint8_t *)DCC_ANNEXB_ADDR;

static int dma_init(void)
{
    /* Vitis moved this driver from device-id to base-address lookup around
     * 2023.2. Both spellings are here because which one compiles depends on
     * the BSP. */
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
                      const dcc_frame_rec_t *r, dcc_kernel_perf_t *perf)
{
    /* Cache maintenance. The HP port is not coherent with the A9 data cache.
     * The frame was written by JTAG straight to DDR and the CPU never touches
     * it, so it needs nothing. The payload buffer does: the DMA writes it
     * behind the cache's back, so stale lines must go before we read it. */
    Xil_DCacheInvalidateRange((UINTPTR)payload, DCC_PAYLOAD_MAX);

    dcc_kernel_clear_done(k);
    dcc_kernel_configure(k, h264_config_word((int)m->width, (int)m->height,
                                             (int)m->qp));

    /* Receive side armed first: the kernel emits as soon as it has a
     * macroblock, so S2MM must already be listening. */
    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)payload, DCC_PAYLOAD_MAX,
                               XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
        printf("FAIL: S2MM start\n"); return -1;
    }
    /* MM2S armed before START, not after. s_axis_tready is held low until
     * START so an early transfer just stalls harmlessly, and arming first
     * keeps DMA setup latency out of the CYCLES measurement. */
    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)r->frame_addr, r->frame_len,
                               XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
        printf("FAIL: MM2S start\n"); return -1;
    }
    dcc_kernel_start(k);

    if (dcc_kernel_wait_done(k, 5000000u) != 0) {
        printf("FAIL: kernel timeout. STATUS=%08lx, MM2S busy=%lu, S2MM busy=%lu\n",
               (unsigned long)dcc_mmio_read(k->base, DCC_REG_STATUS),
               (unsigned long)XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE),
               (unsigned long)XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA));
        return -1;
    }
    while (XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA)) { }

    dcc_kernel_perf(k, perf);
    Xil_DCacheInvalidateRange((UINTPTR)payload, DCC_PAYLOAD_MAX);

    /* Simple-mode S2MM cannot report how many bytes it received. It does not
     * have to: the kernel counts its own output and reports it in BYTES. */
    return (int)perf->bytes;
}

static int compare(const uint8_t *got, const uint8_t *want, int n, int frame)
{
    int i;
    for (i = 0; i < n; i++) {
        if (got[i] != want[i]) {
            printf("FAIL frame %d: first difference at payload byte %d "
                   "(got %02X, want %02X)\n", frame, i, got[i], want[i]);
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
    uint32_t rep, i, mbs, encoded = 0, repeats;
    uint64_t cyc_total = 0;
    uint32_t cyc_min = 0xFFFFFFFFu, cyc_max = 0;
    long bytes_total = 0;

    Xil_DCacheEnable();
    printf("\n=== DCC codec kernel, sequence run (L5) ===\n");

    /* The manifest arrived by JTAG straight into DDR, so any cache line the
     * CPU still holds for it is stale from a previous run. */
    Xil_DCacheInvalidateRange((UINTPTR)DCC_MANIFEST_ADDR, 64u * 1024u);

    if (m->magic != DCC_MANIFEST_MAGIC) {
        printf("FAIL: no manifest at %08lx (magic reads %08lx, want %08lx).\n"
               "      Load one with the generated load.tcl before resuming.\n",
               (unsigned long)DCC_MANIFEST_ADDR, (unsigned long)m->magic,
               (unsigned long)DCC_MANIFEST_MAGIC);
        return 1;
    }
    if (m->version != DCC_MANIFEST_VER) {
        printf("FAIL: manifest version %lu, this build expects %lu\n",
               (unsigned long)m->version, (unsigned long)DCC_MANIFEST_VER);
        return 1;
    }
    recs    = (const dcc_frame_rec_t *)(m + 1);
    mbs     = (m->width / 16u) * (m->height / 16u);
    repeats = m->repeats ? m->repeats : 1u;

    if (dcc_kernel_probe(&k, DCC_ENC_BASE, DCC_ID_H264, m->aclk_hz) != 0) {
        printf("FAIL: ID at %08lx reads %08lx, expected %08lx ('H264').\n"
               "      Check the bitstream is programmed, PL reset released,\n"
               "      and the base address matches the BD_ADDR output.\n",
               (unsigned long)DCC_ENC_BASE, (unsigned long)k.id,
               (unsigned long)DCC_ID_H264);
        return 1;
    }
    printf("kernel:   ID 'H264' version %lu.%lu at %08lx, %lu MHz\n",
           (unsigned long)(k.version >> 16), (unsigned long)(k.version & 0xFFFF),
           (unsigned long)DCC_ENC_BASE, (unsigned long)(m->aclk_hz / 1000000u));
    printf("sequence: %lux%lu QP%lu, %lu frames x %lu repeats, %lu MBs/frame\n",
           (unsigned long)m->width, (unsigned long)m->height,
           (unsigned long)m->qp, (unsigned long)m->n_frames,
           (unsigned long)repeats, (unsigned long)mbs);

    if (dma_init() != 0) return 1;
    if (dcc_kernel_reset(&k) != 0) { printf("FAIL: kernel reset timeout\n"); return 1; }

    for (rep = 0; rep < repeats; rep++) {
        for (i = 0; i < m->n_frames; i++) {
            const dcc_frame_rec_t *r = &recs[i];
            const uint8_t *golden = (const uint8_t *)r->golden_addr;
            int n;

            if (r->golden_len > DCC_PAYLOAD_MAX) {
                printf("FAIL frame %lu: golden is %lu bytes, buffer is %lu\n",
                       (unsigned long)i, (unsigned long)r->golden_len,
                       (unsigned long)DCC_PAYLOAD_MAX);
                return 1;
            }
            Xil_DCacheInvalidateRange((UINTPTR)golden, r->golden_len);

            n = encode_one(&k, m, r, &perf);
            if (n < 0) {
                printf("       (frame %lu of repeat %lu)\n",
                       (unsigned long)i, (unsigned long)rep);
                return 1;
            }
            if (n != (int)r->golden_len) {
                printf("FAIL frame %lu: payload is %d bytes, golden is %lu\n",
                       (unsigned long)i, n, (unsigned long)r->golden_len);
                return 1;
            }
            if (compare(payload, golden, n, (int)i) != 0) return 1;

            cyc_total += perf.cycles;
            if (perf.cycles < cyc_min) cyc_min = perf.cycles;
            if (perf.cycles > cyc_max) cyc_max = perf.cycles;
            bytes_total += n;
            encoded++;

            if (rep == 0)
                printf("  frame %2lu: %6d bytes, %7lu cycles (%.2f ms, %.0f cy/MB) OK\n",
                       (unsigned long)i, n, (unsigned long)perf.cycles,
                       perf.cycles * 1000.0 / m->aclk_hz,
                       (double)perf.cycles / mbs);
        }
        if (repeats > 1u && ((rep + 1u) % 10u == 0u))
            printf("  ... %lu of %lu repeats, all matching\n",
                   (unsigned long)(rep + 1u), (unsigned long)repeats);
    }

    /* ---- what the run says about running at rate ---- */
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

    /* ---- and the stream a decoder would see, for the last frame ---- */
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
