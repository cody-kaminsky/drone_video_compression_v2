/* cavlc.h — CAVLC bit-length estimation.
 *
 * v1 SCOPE: this module returns the *number of bits* a CAVLC-encoded
 * residual block would consume, but does not emit the actual bits. That
 * gives us accurate BPP for architecture validation without the complexity
 * of bit packing, RBSP, emulation prevention, and NAL framing.
 *
 * M2 will:
 *   1. Replace these length-only computations with full encode (length +
 *      codeword), wired into a bitstream.h writer.
 *   2. Add Exp-Golomb encoding for slice header / mb_type / mb_qp_delta.
 *   3. Add emulation prevention and NAL framing per Annex B.
 *
 * Spec reference: H.264 section 9.2 (CAVLC).
 *
 * FPGA: this module's algorithm maps cleanly to the 4-engine CAVLC design
 *       in architecture.txt §8. The bit-length calculations done here are
 *       a strict subset of what HW will need — see notes inline.
 */
#ifndef DCC_CAVLC_H
#define DCC_CAVLC_H

#include "types.h"
#include "bitstream.h"

/* Block type — affects scan order, coefficient count, and which length
 * tables are used. */
typedef enum {
    BLK_LUMA_AC,        /* 15 coefs (positions 1..15 only); used when DC was
                         * extracted (I_16x16 luma residual blocks) */
    BLK_LUMA_FULL,      /* 16 coefs (positions 0..15); used for I_4x4 (M2) */
    BLK_LUMA_DC_16x16,  /* 16 DC coefs after 4x4 Hadamard (I_16x16 only) */
    BLK_CHROMA_AC,      /* 15 coefs (positions 1..15); chroma residual */
    BLK_CHROMA_DC,      /* 4 DC coefs after 2x2 Hadamard */
} block_type_t;

/* Estimate the CAVLC bit length for one residual block (approximate, fast).
 *
 *   coefs:    coefficient array in zig-zag scan order (already reordered).
 *   n_coefs:  array length (16, 15, or 4 depending on block_type).
 *   nC:       neighbor-coeff predictor for coeff_token table selection.
 *             Use cavlc_compute_nC() below; pass -1 for chroma DC blocks.
 *
 * Returns: number of bits the block would consume in a real CAVLC stream.
 */
int cavlc_estimate_block_bits(const i16 *coefs, int n_coefs,
                              block_type_t bt, int nC);

/* Real CAVLC encoder — emits bits using spec Tables 9-5/9-7/9-9/9-10.
 * Returns number of bits written. */
int cavlc_encode_block(bitstream_t *bs, const i16 *coefs, int n_coefs,
                       block_type_t bt, int nC);

/* CAVLC decoder — mirror of cavlc_encode_block.
 * Reads bits from br and writes coefficients (in zigzag scan order — same
 * order the encoder received them) into out_coefs[0..n_coefs-1].
 * Returns 0 on success, -1 on malformed code (no matching VLC). */
int cavlc_decode_block(bitreader_t *br, i16 *out_coefs, int n_coefs,
                       block_type_t bt, int nC);

/* Compute nC for a luma/chroma-AC block from its top and left neighbors'
 * total-coeff counts. avail_* says whether each neighbor exists.
 *
 * Spec 9.2.1.1: nC = (nA + nB + 1) >> 1, with edge-availability rules. */
int cavlc_compute_nC(int top_total_coef, int left_total_coef,
                     int avail_top, int avail_left);

/* Count the nonzero coefficients in a block — useful for updating the
 * neighbor "totalcoeff" buffer that nC reads. */
int count_nonzero(const i16 *coefs, int n);

/* Zig-zag scan order for 4x4 blocks (spec table 8-12). The encoder uses
 * this to reorder a row-major 4x4 array into scan order before CAVLC. */
extern const u8 zz_scan_4x4[16];

/* Zig-zag for chroma DC 2x2 (trivial). */
extern const u8 zz_scan_2x2[4];

#endif
