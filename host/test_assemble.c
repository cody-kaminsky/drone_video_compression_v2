/* test_assemble.c — prove the board-side stream assembly on a workstation.
 *
 * The board will take the kernel's payload bytes and wrap them into an Annex
 * B access unit with h264_assemble_idr. That wrapping is pure arithmetic over
 * buffers, so it can be checked here, against the very bytes the reference
 * encoder produced, long before a board exists. If this passes and the RTL
 * testbench passes, a hardware mismatch can only be the hardware.
 *
 * Two checks:
 *   1. h264_nv12_to_stream produces exactly the word order the RTL testbench
 *      is fed (tools/gen_frame_stream.py), so the DMA will present the frame
 *      the way the kernel reads it.
 *   2. h264_assemble_idr, given the reference's own payload dump, reproduces
 *      the reference's .264 byte for byte.
 *
 * Usage:
 *   test_assemble <in.yuv> <width> <height> <qp> <payload.txt> <stream.txt> <ref.264>
 */

#include "h264_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned char *slurp(const char *path, long *len)
{
    FILE *f = fopen(path, "rb");
    unsigned char *b;
    if (!f) { perror(path); return NULL; }
    fseek(f, 0, SEEK_END); *len = ftell(f); fseek(f, 0, SEEK_SET);
    b = malloc((size_t)*len);
    if (fread(b, 1, (size_t)*len, f) != (size_t)*len) { fclose(f); free(b); return NULL; }
    fclose(f);
    return b;
}

/* DCC_DUMP_SLICE writes one decimal byte per line. */
static unsigned char *read_payload(const char *path, int *len)
{
    FILE *f = fopen(path, "r");
    unsigned char *b;
    int cap = 1 << 16, n = 0, v;
    if (!f) { perror(path); return NULL; }
    b = malloc((size_t)cap);
    while (fscanf(f, "%d", &v) == 1) {
        if (n == cap) { cap *= 2; b = realloc(b, (size_t)cap); }
        b[n++] = (unsigned char)v;
    }
    fclose(f);
    *len = n;
    return b;
}

int main(int argc, char **argv)
{
    const char *yuv_path, *payload_path, *stream_path, *ref_path;
    int width, height, qp, payload_len, fails = 0;
    long yuv_len, ref_len;
    unsigned char *yuv, *payload, *ref, *stream, *out;
    FILE *fs;
    int mb_bits, out_len;

    if (argc != 8) {
        fprintf(stderr, "usage: %s <in.yuv> <w> <h> <qp> <payload.txt> <stream.txt> <ref.264>\n", argv[0]);
        return 1;
    }
    yuv_path = argv[1];
    width  = atoi(argv[2]);
    height = atoi(argv[3]);
    qp     = atoi(argv[4]);
    payload_path = argv[5];
    stream_path  = argv[6];
    ref_path     = argv[7];

    yuv = slurp(yuv_path, &yuv_len);
    if (!yuv) return 2;
    payload = read_payload(payload_path, &payload_len);
    if (!payload) return 2;
    ref = slurp(ref_path, &ref_len);
    if (!ref) return 2;

    /* ---- 1. input ordering matches what the RTL testbench is fed ---- */
    {
        uint32_t n = h264_frame_bytes(width, height);
        uint32_t i = 0, beats = 0, mism = 0, first = 0xFFFFFFFFu;
        unsigned w;
        if ((long)n != yuv_len) {
            printf("FAIL ordering: %s is %ld bytes, expected %u\n", yuv_path, yuv_len, n);
            fails++;
        }
        stream = malloc(n);
        h264_nv12_to_stream(stream, yuv, width, height);

        fs = fopen(stream_path, "r");
        if (!fs) { perror(stream_path); return 2; }
        while (fscanf(fs, "%x", &w) == 1) {
            uint32_t got;
            if (i + 4 > n) { mism++; break; }
            got = (uint32_t)stream[i] | ((uint32_t)stream[i+1] << 8)
                | ((uint32_t)stream[i+2] << 16) | ((uint32_t)stream[i+3] << 24);
            if (got != w) {
                if (first == 0xFFFFFFFFu) first = beats;
                mism++;
            }
            i += 4; beats++;
        }
        fclose(fs);
        if (i != n) {
            printf("FAIL ordering: covered %u of %u bytes (%u beats)\n", i, n, beats);
            fails++;
        } else if (mism) {
            printf("FAIL ordering: %u of %u beats differ, first at beat %u\n", mism, beats, first);
            fails++;
        } else {
            printf("PASS ordering: %u beats match gen_frame_stream.py\n", beats);
        }
        free(stream);
    }

    /* ---- 2. the stop-bit scan recovers the macroblock-layer bit count ---- */
    mb_bits = h264_payload_bits(payload, payload_len);
    if (mb_bits < 0 || mb_bits > payload_len * 8 - 1
        || mb_bits <= (payload_len - 1) * 8 - 1) {
        printf("FAIL bit count: %d bits from a %d byte payload\n", mb_bits, payload_len);
        fails++;
    } else {
        printf("PASS bit count: %d macroblock-layer bits in %d payload bytes\n",
               mb_bits, payload_len);
    }

    /* ---- 3. assembled access unit equals the reference stream ---- */
    {
        int dst_cap = H264_ASSEMBLE_DST_MIN(payload_len);
        int scr_cap = payload_len + H264_ASSEMBLE_SLACK;
        unsigned char *scratch = malloc((size_t)scr_cap);
        out = malloc((size_t)dst_cap);
        out_len = h264_assemble_idr(out, dst_cap, scratch, scr_cap,
                                    payload, payload_len, width, height, qp, 0, 1);
        free(scratch);
    }
    if (out_len < 0) {
        printf("FAIL assemble: h264_assemble_idr returned %d\n", out_len);
        fails++;
    } else if (out_len != (int)ref_len) {
        printf("FAIL assemble: %d bytes, reference is %ld\n", out_len, ref_len);
        fails++;
    } else if (memcmp(out, ref, (size_t)out_len) != 0) {
        int i;
        for (i = 0; i < out_len && out[i] == ref[i]; i++) { }
        printf("FAIL assemble: first difference at byte %d (got %02X, want %02X)\n",
               i, out[i], ref[i]);
        fails++;
    } else {
        printf("PASS assemble: %d bytes byte-exact with the reference stream\n", out_len);
    }

    free(yuv); free(payload); free(ref); free(out);
    printf(fails ? "TEST FAILED (%d)\n" : "TEST PASSED (%d)\n", fails);
    return fails ? 1 : 0;
}
