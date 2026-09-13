/* board_main.c — L4 of the validation ladder: run a frame through the real
 * kernel on the board and compare the payload, byte for byte, against the
 * bytes the C reference produced for the same frame and QP.
 *
 * NOT BUILT BY THE WORKSTATION MAKEFILE. This needs the Vitis BSP headers and
 * an XSA from scripts/build_zybo_bd.tcl, so it is the one file here that has
 * never been compiled. Everything it calls into (h264_host.c, codec_kernel.c,
 * src/nal.c, src/bitstream.c) is exercised by `make host_test` on x86.
 *
 * Setup is scripted:  make board_vectors  then  scripts/vitis_setup.py.
 * That creates the platform from the XSA and an application whose sources are
 * this file, host/codec_kernel.c, host/h264_host.c,
 * host/platform_standalone.c, and src/nal.c + src/bitstream.c verbatim from
 * the reference encoder.
 *
 * The input frame and the golden payload are linked in as C arrays, generated
 * by `make board_vectors`. For 480x272 that is 196 kB of frame and 19 kB of
 * golden, which fits DDR trivially and removes every file-I/O variable from
 * the first run. Move to SD or JTAG-loaded buffers only once this passes, and
 * only because 1080p is too big to link in comfortably.
 */

#include "codec_kernel.h"
#include "h264_host.h"
#include "xparameters.h"
#include "xaxidma.h"
#include "xil_cache.h"
#include <stdio.h>
#include <string.h>

/* ---- what the block design produced ----------------------------------------
 * These are the addresses scripts/build_zybo_bd.tcl assigned, printed as
 * BD_ADDR lines when it ran. They are literals rather than XPAR_* symbols
 * because the generated symbol name depends on the BD cell name and the
 * Vitis release, and a wrong guess is a compile error at best and a silent
 * read of the wrong peripheral at worst. If you rename cells or re-run the
 * BD script, take the new values from its BD_ADDR output or xparameters.h. */
#ifndef DCC_ENC_BASE
#define DCC_ENC_BASE   0x43C00000u      /* SEG_enc_reg0  */
#endif
#ifndef DCC_DMA_BASE
#define DCC_DMA_BASE   0x40400000u      /* SEG_dma_Reg   */
#endif

/* Must match the PL clock the BD actually synthesised, not the one you asked
 * for. FCLK0 is the IO PLL divided by two integers, so from 1000 MHz the only
 * reachable values near 110 MHz are 1000/9 = 111.111 and 1000/10 = 100: ask
 * for 110 and you silently get 111.111. build_zybo_bd prints the achieved
 * value as BD_FCLK and warns when it differs from the request. Every
 * cycles-to-milliseconds number below is wrong if this is wrong. */
#define DCC_ACLK_HZ    100000000u

/* ---- the test frame, linked in. See the header comment. ---- */
extern const unsigned char frame_nv12[];
extern const unsigned int  frame_nv12_len;
extern const unsigned char golden_payload[];
extern const unsigned int  golden_payload_len;

#define FRAME_W   480
#define FRAME_H   272
#define FRAME_QP  26

/* Staging and receive buffers. Aligned to a cache line so the flush and
 * invalidate below act on these buffers and nothing that shares a line with
 * them -- invalidating a partial line discards whatever else lives there. */
static uint8_t stage[FRAME_W * FRAME_H * 3 / 2]   __attribute__((aligned(64)));
static uint8_t payload[256 * 1024]                __attribute__((aligned(64)));
static uint8_t annexb[512 * 1024];
static uint8_t scratch[256 * 1024 + 64];

static XAxiDma dma;

static int dma_init(void)
{
    /* Vitis moved the DMA driver from device-id lookup to base-address
     * lookup around 2023.2. Both spellings are here because which one
     * compiles depends on the BSP, and this is the most likely first
     * build error. */
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
        /* The bring-up path is simple mode on purpose; an SG-configured DMA
         * would silently ignore the simple transfers below. */
        printf("FAIL: DMA is built for scatter-gather, expected simple mode\n");
        return -1;
    }
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    return 0;
}

/* Encode one frame. Returns the kernel's payload length, or negative. */
static int encode_one(dcc_kernel_t *k, int width, int height, int qp,
                      const uint8_t *nv12, uint8_t *out, uint32_t out_cap,
                      dcc_kernel_perf_t *perf)
{
    uint32_t frame_bytes = h264_frame_bytes(width, height);

    h264_nv12_to_stream(stage, nv12, width, height);

    /* Cache maintenance. The HP port is not coherent with the A9 data cache:
     * without the flush the DMA reads stale DDR and the failure is
     * intermittent and data-dependent, which reads exactly like an encoder
     * bug. Without the invalidate the CPU reads a stale payload. Neither is
     * optional and neither fails loudly. */
    Xil_DCacheFlushRange((UINTPTR)stage, frame_bytes);
    Xil_DCacheInvalidateRange((UINTPTR)out, out_cap);

    dcc_kernel_clear_done(k);
    dcc_kernel_configure(k, h264_config_word(width, height, qp));

    /* Receive side armed first: the kernel can start emitting as soon as it
     * has a macroblock, and S2MM must already be listening. */
    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)out, out_cap,
                               XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
        printf("FAIL: S2MM start\n"); return -1;
    }

    /* START before the input DMA. s_axis_tready is held low until START, so
     * the order is not critical, but this way the kernel is never the thing
     * that is late. */
    dcc_kernel_start(k);

    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)stage, frame_bytes,
                               XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
        printf("FAIL: MM2S start\n"); return -1;
    }

    if (dcc_kernel_wait_done(k, 2000000u) != 0) {
        printf("FAIL: kernel timeout. STATUS=%08lx, MM2S busy=%lu, S2MM busy=%lu\n",
               (unsigned long)dcc_mmio_read(k->base, DCC_REG_STATUS),
               (unsigned long)XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE),
               (unsigned long)XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA));
        return -1;
    }
    /* Let S2MM retire the last beats after tlast before reading the buffer. */
    while (XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA)) { }

    dcc_kernel_perf(k, perf);
    Xil_DCacheInvalidateRange((UINTPTR)out, out_cap);

    /* Simple-mode S2MM cannot report how many bytes it received, but it does
     * not have to: the kernel counts its own output and reports it in BYTES.
     * That is why simple mode is usable here at all. */
    return (int)perf->bytes;
}

int main(void)
{
    dcc_kernel_t k;
    dcc_kernel_perf_t perf;
    int n, i, au_len;
    double ms, mb_cy;

    Xil_DCacheEnable();
    printf("\n=== DCC codec kernel bring-up (L4) ===\n");

    if (dcc_kernel_probe(&k, DCC_ENC_BASE, DCC_ID_H264, DCC_ACLK_HZ) != 0) {
        /* Absent, held in reset, and wrong base address all look the same
         * from here, which is precisely why ID is read before anything else. */
        printf("FAIL: ID at %08lx reads %08lx, expected %08lx ('H264').\n"
               "      Check the base address in xparameters.h, that the\n"
               "      bitstream is loaded, and that PL reset is released.\n",
               (unsigned long)DCC_ENC_BASE, (unsigned long)k.id,
               (unsigned long)DCC_ID_H264);
        return 1;
    }
    printf("kernel: ID 'H264' version %lu.%lu at %08lx\n",
           (unsigned long)(k.version >> 16), (unsigned long)(k.version & 0xFFFF),
           (unsigned long)DCC_ENC_BASE);

    if (dma_init() != 0) return 1;
    if (dcc_kernel_reset(&k) != 0) { printf("FAIL: kernel reset timeout\n"); return 1; }

    if (frame_nv12_len != h264_frame_bytes(FRAME_W, FRAME_H)) {
        printf("FAIL: linked frame is %u bytes, expected %u\n",
               frame_nv12_len, (unsigned)h264_frame_bytes(FRAME_W, FRAME_H));
        return 1;
    }

    n = encode_one(&k, FRAME_W, FRAME_H, FRAME_QP, frame_nv12,
                   payload, sizeof payload, &perf);
    if (n < 0) return 1;

    ms    = (double)perf.cycles * 1000.0 / (double)DCC_ACLK_HZ;
    mb_cy = (double)perf.cycles / (double)((FRAME_W / 16) * (FRAME_H / 16));
    printf("frame: %d payload bytes, %lu cycles (%.2f ms, %.0f cycles/MB)\n",
           n, (unsigned long)perf.cycles, ms, mb_cy);

    /* ---- the comparison that the whole exercise exists for ---- */
    if (n != (int)golden_payload_len) {
        printf("FAIL: payload is %d bytes, golden is %u\n", n, golden_payload_len);
        return 1;
    }
    for (i = 0; i < n; i++) {
        if (payload[i] != golden_payload[i]) {
            int mb_bits_in = i * 8;
            printf("FAIL: first difference at payload byte %d (got %02X, want %02X)\n",
                   i, payload[i], golden_payload[i]);
            printf("      that is bit %d of the macroblock layer; re-run the\n"
                   "      reference with DCC_DUMP_MB to find which macroblock\n"
                   "      spans it. See docs/m5-hw-validation.md 5.4.\n", mb_bits_in);
            return 1;
        }
    }
    printf("PASS: payload byte-exact with the C reference (%d bytes)\n", n);

    /* ---- and the stream a decoder would actually see ---- */
    au_len = h264_assemble_idr(annexb, sizeof annexb, scratch, sizeof scratch,
                               payload, n, FRAME_W, FRAME_H, FRAME_QP, 0, 1);
    if (au_len < 0) { printf("FAIL: assemble returned %d\n", au_len); return 1; }
    printf("access unit: %d bytes of Annex B, ready to pull off and decode\n", au_len);
    printf("=== L4 PASSED ===\n");
    return 0;
}
