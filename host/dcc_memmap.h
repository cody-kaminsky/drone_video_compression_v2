/* dcc_memmap.h — the DDR layout the host loader and the board application
 * both agree on, and the manifest that describes what was loaded.
 *
 * The point of the manifest is that the board application does not need
 * recompiling when the test sequence changes. The host writes frames, goldens
 * and a manifest into DDR over JTAG; the application reads the manifest and
 * runs whatever it describes. Change the sequence, reload, re-run.
 *
 * Frames are stored in the kernel's own stream order -- per macroblock row,
 * 16 luma lines then 8 chroma lines -- not planar NV12. The reordering is
 * deterministic, so it is done once on the workstation by the same
 * h264_nv12_to_stream() the x86 test covers. The board therefore does no
 * staging copy at all: MM2S reads the frame where it lies.
 *
 * Layout (Zybo Z7-20, 1 GB DDR, application links at 0x00100000):
 *
 *   0x0010_0000  application text / data / bss        (a few MB)
 *   0x0300_0000  manifest                             (1 MB)
 *   0x0400_0000  frames, stream order, stride-packed  (448 MB, 149 x 1080p)
 *   0x2000_0000  golden payloads, stride-packed       (256 MB)
 *   0x3000_0000  payload receive buffer               (8 MB)
 *   0x3100_0000  slice RBSP scratch                   (8 MB)
 *   0x3200_0000  assembled Annex B                    (16 MB)
 *                                                     ends before 0x4000_0000
 */
#ifndef DCC_MEMMAP_H
#define DCC_MEMMAP_H

#include <stdint.h>

#define DCC_MANIFEST_ADDR   0x03000000u
#define DCC_FRAMES_ADDR     0x04000000u
#define DCC_GOLDEN_ADDR     0x20000000u
#define DCC_PAYLOAD_ADDR    0x30000000u
#define DCC_SCRATCH_ADDR    0x31000000u
#define DCC_ANNEXB_ADDR     0x32000000u

#define DCC_PAYLOAD_MAX     (8u  * 1024u * 1024u)
#define DCC_SCRATCH_MAX     (8u  * 1024u * 1024u)
#define DCC_ANNEXB_MAX      (16u * 1024u * 1024u)

/* Room the layout gives each region, so the loader can refuse to overrun. */
#define DCC_FRAMES_ROOM     (DCC_GOLDEN_ADDR  - DCC_FRAMES_ADDR)
#define DCC_GOLDEN_ROOM     (DCC_PAYLOAD_ADDR - DCC_GOLDEN_ADDR)

#define DCC_MANIFEST_MAGIC  0x4D434344u   /* 'DCCM' little-endian */
#define DCC_MANIFEST_VER    1u

typedef struct {
    uint32_t frame_addr;    /* frame in kernel stream order */
    uint32_t frame_len;     /* width * height * 3 / 2 */
    uint32_t golden_addr;   /* expected payload bytes */
    uint32_t golden_len;
} dcc_frame_rec_t;

typedef struct {
    uint32_t magic;         /* DCC_MANIFEST_MAGIC */
    uint32_t version;       /* DCC_MANIFEST_VER */
    uint32_t width;
    uint32_t height;
    uint32_t qp;
    uint32_t n_frames;      /* records that follow */
    uint32_t repeats;       /* times to cycle the sequence; 0 means 1 */
    uint32_t aclk_hz;       /* PL clock, so timing is right without a rebuild */
    /* n_frames records follow immediately */
} dcc_manifest_t;

#endif
