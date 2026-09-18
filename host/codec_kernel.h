/* codec_kernel.h — the contract every DCC codec kernel presents to its host.
 *
 * This header is deliberately codec-agnostic. It describes the socket, not
 * the encoder: a host that knows only this file can find a kernel, identify
 * it, reset it, run one unit of work, and read back how long that took and
 * how many bytes came out. Everything specific to H.264 lives in
 * h264_host.h, and a future codec adds its own equivalent without touching
 * this one.
 *
 * ---------------------------------------------------------------- hardware
 * A conforming kernel presents exactly three AXI interfaces:
 *
 *   s_axi   AXI4-Lite slave, 256-byte register map (below)
 *   s_axis  AXI4-Stream slave, input samples, kernel-defined order
 *   m_axis  AXI4-Stream master, output payload, tlast on the final beat
 *           and tkeep marking the valid bytes of that beat
 *
 * plus a level-high `irq` asserted while DONE is set and IRQ_EN is set.
 *
 * ------------------------------------------------------------ register map
 * Offsets 0x00 and 0x08..0x23 are common to every codec. Offset 0x04 and
 * everything from 0x24 up belong to the codec: one config word is enough
 * for H.264, a codec needing more takes 0x24 and beyond.
 *
 *   0x00  CTRL      W   START, SOFT_RESET      RW  IRQ_EN
 *   0x04  CONFIG    RW  codec-defined, latched at START
 *   0x08  STATUS    R   BUSY, DONE, and stream-health bits
 *   0x0C  DONE_CLR  W   write 1 to clear DONE and the interrupt
 *   0x10  UNITS     R   units (frames) completed since reset
 *   0x14  CYCLES    R   aclk cycles of the last unit, START to done
 *   0x18  BYTES     R   payload bytes of the last unit
 *   0x1C  ID        R   fourcc identifying the codec, e.g. 'H264'
 *   0x20  VERSION   R   [31:16] major, [15:0] minor
 *
 * ------------------------------------------------------------- the split
 * The kernel produces the *payload* only: for H.264 that is the macroblock
 * layer, byte-aligned, with a trailing stop bit. Container work -- parameter
 * sets, slice or frame headers, start codes, escaping -- stays on the host,
 * where it is cheap to change and runs once per frame rather than once per
 * macroblock. Keep this split for any future codec: it is what lets the same
 * verified header code run both in the x86 reference and on the board.
 */
#ifndef DCC_CODEC_KERNEL_H
#define DCC_CODEC_KERNEL_H

#include <stdint.h>

/* ------------------------------------------------------------- registers */
#define DCC_REG_CTRL      0x00u
#define DCC_REG_CONFIG    0x04u
#define DCC_REG_STATUS    0x08u
#define DCC_REG_DONE_CLR  0x0Cu
#define DCC_REG_UNITS     0x10u
#define DCC_REG_CYCLES    0x14u
#define DCC_REG_BYTES     0x18u
#define DCC_REG_ID        0x1Cu
#define DCC_REG_VERSION   0x20u

/* CTRL bits */
#define DCC_CTRL_START       (1u << 0)
#define DCC_CTRL_SOFT_RESET  (1u << 1)
#define DCC_CTRL_IRQ_EN      (1u << 8)

/* STATUS bits */
#define DCC_STATUS_BUSY        (1u << 0)
#define DCC_STATUS_DONE        (1u << 1)   /* sticky, cleared via DONE_CLR */
#define DCC_STATUS_SAXIS_READY (1u << 2)
#define DCC_STATUS_MAXIS_VALID (1u << 3)

/* Known codec identities. A kernel that does not answer with one of these
 * is either absent, held in reset, or at the wrong base address -- all three
 * look identical from software, so always check ID before anything else. */
#define DCC_ID_H264  0x48323634u   /* 'H264' */

/* ------------------------------------------------------- platform shim ---
 * Three things a platform provides. Bare-metal Zynq and a host-side stub
 * both implement these; nothing else in the host code touches hardware. */
uint32_t dcc_mmio_read (uintptr_t base, uint32_t off);
void     dcc_mmio_write(uintptr_t base, uint32_t off, uint32_t val);
/* Microseconds since an arbitrary epoch, monotonic. Used for timeouts and
 * for the wall-clock side of the throughput report. */
uint64_t dcc_time_us(void);

/* ------------------------------------------------------------- the API --- */
typedef struct {
    uintptr_t base;        /* AXI4-Lite base address */
    uint32_t  id;          /* as read from ID */
    uint32_t  version;     /* as read from VERSION */
    uint32_t  aclk_hz;     /* PL clock, for turning CYCLES into time */
} dcc_kernel_t;

/* Read ID and VERSION and fill k. Returns 0 if ID matches want_id, -1 if it
 * does not (k->id still carries what was actually read, which is the useful
 * thing to print). */
int dcc_kernel_probe(dcc_kernel_t *k, uintptr_t base, uint32_t want_id,
                     uint32_t aclk_hz);

/* Pulse SOFT_RESET and wait for the kernel to go idle. Returns 0, or -1 on
 * timeout. */
int dcc_kernel_reset(const dcc_kernel_t *k);

/* Write the codec's CONFIG word. Latched by the kernel at START, so it must
 * be written before the start pulse and must not change during a unit. */
void dcc_kernel_configure(const dcc_kernel_t *k, uint32_t config);

/* Pulse START. The input stream may be running before or after this call:
 * the kernel holds s_axis_tready low until START, so an early DMA is safe. */
void dcc_kernel_start(const dcc_kernel_t *k);

/* Poll STATUS until DONE, up to timeout_us. Returns 0 on done, -1 on
 * timeout. Does not clear DONE. */
int dcc_kernel_wait_done(const dcc_kernel_t *k, uint32_t timeout_us);

/* Clear the sticky DONE bit and the interrupt. */
void dcc_kernel_clear_done(const dcc_kernel_t *k);

/* Per-unit results, read after DONE. */
typedef struct {
    uint32_t units;        /* units completed since reset */
    uint32_t cycles;       /* aclk cycles of the last unit */
    uint32_t bytes;        /* payload bytes of the last unit */
} dcc_kernel_perf_t;

void dcc_kernel_perf(const dcc_kernel_t *k, dcc_kernel_perf_t *p);

#endif
