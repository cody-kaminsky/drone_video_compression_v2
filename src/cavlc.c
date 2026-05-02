/* cavlc.c — CAVLC bit-length estimation + actual bit emission. */
#include "cavlc_tables.h"
/* Original header below: */
/* cavlc.c — CAVLC bit-length estimation.
 *
 * v1 SCOPE: bit-length estimation only. We follow the structural CAVLC
 * algorithm (count nonzeros, trailing ones, code levels in reverse, then
 * total_zeros and run_before) but the per-symbol bit lengths are computed
 * from closed-form approximations rather than the spec's exact lookup
 * tables. Expected accuracy: within ~10–15% of true CAVLC bit count,
 * with the same scaling behavior across QP and content.
 *
 * M2 work: replace the *_bits_approx() helpers with table-9-5/9-7/9-9/9-10
 * lookups and switch from "estimate" to "encode" so bits are emitted to a
 * real bitstream. The surrounding algorithm stays the same.
 */
#include "cavlc.h"

const u8 zz_scan_4x4[16] = {
    0,  1,  4,  8,
    5,  2,  3,  6,
    9, 12, 13, 10,
    7, 11, 14, 15
};

const u8 zz_scan_2x2[4] = { 0, 1, 2, 3 };

int count_nonzero(const i16 *coefs, int n)
{
    int c = 0;
    for (int i = 0; i < n; i++)
        if (coefs[i] != 0) c++;
    return c;
}

int cavlc_compute_nC(int top_total_coef, int left_total_coef,
                     int avail_top, int avail_left)
{
    /* Spec 9.2.1.1.
     * nC = nA + nB if both available
     *      = (nA + nB + 1) >> 1 if both available (corrected, spec uses
     *      this form)
     * Edge cases: missing neighbor uses 0. */
    if (avail_top && avail_left)
        return (top_total_coef + left_total_coef + 1) >> 1;
    if (avail_top)  return top_total_coef;
    if (avail_left) return left_total_coef;
    return 0;
}

/* ----- approximate bit-length helpers ----- */

/* coeff_token: encodes (TotalCoeff, TrailingOnes) with one of 5 sub-tables
 * picked by nC. Real lengths range from 1 to 16 bits. The approximation
 * below tracks the spec's general shape: more coefficients → longer code,
 * higher nC → shorter code (because high nC means high activity, expected). */
static int coeff_token_bits_approx(int total_coef, int trailing_ones, int nC)
{
    (void)trailing_ones;
    if (total_coef == 0)
        return (nC == -1) ? 2 : 1;            /* short "no coef" code */
    if (nC == -1) {
        /* Chroma DC sub-table (Table 9-5 last column). */
        if (total_coef <= 1) return 2 + total_coef * 4;
        return 6 + total_coef;
    }
    if (nC >= 8) return 6;                    /* FLC sub-table */
    int base;
    if (nC < 2)      base = 6;
    else if (nC < 4) base = 4;
    else             base = 3;                /* nC 4..7 */
    int b = base + (total_coef > 1 ? total_coef - 1 : 0);
    if (total_coef > 4)  b += 1;
    if (total_coef > 8)  b += 1;
    if (total_coef > 12) b += 1;
    if (b > 16) b = 16;
    return b;
}

/* level encoding bits.
 * Following spec 9.2.2: level uses level_prefix (unary) + level_suffix (FLC).
 * suffix_length adapts as we encode levels in reverse scan order. */
static int level_bits(int abs_level, int suffix_length, int *next_suffix)
{
    /* "Level - 1" sign-magnitude shift used in spec — for bit length we
     * only need the magnitude. */
    if (suffix_length == 0) {
        /* Special table 9-6: short 1-bit prefix for ±1, longer for larger. */
        if (abs_level <= 7)   { *next_suffix = (abs_level > 3) ? 2 : 1;
                                return abs_level * 2 - 1 + 1; /* prefix+sign */ }
        if (abs_level <= 15)  { *next_suffix = 2;
                                return 14 + 1 + 4; /* prefix=14, suffix=4 */ }
        /* escape */
        *next_suffix = 2;
        /* large escape: 16-bit prefix-of-15 + 12-bit value */
        return 16 + 12;
    } else {
        int level_code_idx = abs_level - 1;
        int prefix = level_code_idx >> suffix_length;
        if (prefix < 15) {
            int b = (prefix + 1) + suffix_length + 1; /* +1 sign */
            /* Update suffix_length per spec 9.2.2.1. */
            if (abs_level > (3 << (suffix_length - 1)))
                *next_suffix = (suffix_length < 6) ? suffix_length + 1
                                                   : suffix_length;
            else
                *next_suffix = suffix_length;
            return b;
        }
        /* escape */
        *next_suffix = (suffix_length < 6) ? suffix_length + 1 : suffix_length;
        return 28; /* approximation: 16 prefix + 12 escape suffix */
    }
}

/* total_zeros bit length. Spec table 9-7 (luma) or 9-9 (chroma DC).
 * Approximation: 1 + ceil(log2(16-TotalCoeff)) plus small adjustment. */
static int total_zeros_bits_approx(int total_zeros, int total_coef,
                                   int is_chroma_dc, int max_coefs)
{
    if (total_zeros == 0) return 1;
    int max_zeros = max_coefs - total_coef;
    if (max_zeros <= 0) return 0;
    int b = 1;
    int v = max_zeros;
    while (v > 0) { v >>= 1; b++; }
    /* chroma DC has shorter codes */
    if (is_chroma_dc) b = (b > 3) ? 3 : b;
    if (b > 9) b = 9;
    return b;
}

/* run_before bit length. Spec table 9-10. */
static int run_before_bits_approx(int run_before, int zeros_left)
{
    if (zeros_left == 0) return 0;
    if (zeros_left == 1) return 1;
    if (zeros_left <= 6) {
        int b = 1;
        int v = zeros_left;
        while (v > 1) { v >>= 1; b++; }
        return b;
    }
    /* zeros_left >= 7: longer codes (truncated unary up to 11 bits). */
    if (run_before < 7) return 3;
    return 3 + (run_before - 6);
}

int cavlc_estimate_block_bits(const i16 *coefs, int n_coefs,
                              block_type_t bt, int nC)
{
    /* Find last nonzero. */
    int last_nz = -1;
    for (int i = 0; i < n_coefs; i++)
        if (coefs[i] != 0) last_nz = i;

    if (last_nz < 0) {
        /* Empty block — only coeff_token needed. */
        int chroma_dc = (bt == BLK_CHROMA_DC);
        return coeff_token_bits_approx(0, 0, chroma_dc ? -1 : nC);
    }

    /* Walk coefs (in zig-zag scan order, which is how they arrive). */
    int total_coef = 0;
    int trailing_ones = 0;
    int t1_done = 0;        /* set when we hit a non-(±1) non-zero */
    int sign_bits = 0;
    int run_before_total = 0;

    /* Pass 1: count, find trailing ones (in reverse from last_nz). */
    for (int i = last_nz; i >= 0; i--) {
        if (coefs[i] == 0) continue;
        total_coef++;
        if (!t1_done && (coefs[i] == 1 || coefs[i] == -1) && trailing_ones < 3) {
            trailing_ones++;
        } else {
            t1_done = 1;
        }
    }

    int chroma_dc = (bt == BLK_CHROMA_DC);
    int max_coefs = n_coefs;

    int bits = 0;

    /* coeff_token */
    bits += coeff_token_bits_approx(total_coef, trailing_ones,
                                    chroma_dc ? -1 : nC);

    /* trailing_ones_sign_flag — 1 bit per trailing one */
    sign_bits += trailing_ones;

    /* Levels (non-trailing-ones). Encoded in reverse scan order. */
    int suffix_length = 0;
    if (total_coef > 10 && trailing_ones < 3) suffix_length = 1;

    int t1_seen = 0;
    for (int i = last_nz; i >= 0; i--) {
        if (coefs[i] == 0) continue;
        if (!t1_seen && (coefs[i] == 1 || coefs[i] == -1) && t1_seen < trailing_ones) {
            /* This is a trailing one, sign already counted. */
            t1_seen++;
            if (t1_seen >= trailing_ones) t1_seen = trailing_ones;
            continue;
        }
        int abs_v = coefs[i] < 0 ? -coefs[i] : coefs[i];
        int next_sl;
        bits += level_bits(abs_v, suffix_length, &next_sl);
        suffix_length = next_sl;
    }

    bits += sign_bits;

    /* total_zeros (only if total_coef < max_coefs). */
    int n_zeros = (last_nz + 1) - total_coef;
    if (total_coef < max_coefs) {
        bits += total_zeros_bits_approx(n_zeros, total_coef, chroma_dc, max_coefs);
    }

    /* run_before for each nonzero except the lowest-frequency one. */
    int zeros_left = n_zeros;
    int rb_count = 0;
    int prev_idx = last_nz + 1;
    for (int i = last_nz; i >= 0 && zeros_left > 0 && rb_count < total_coef - 1; i--) {
        if (coefs[i] == 0) continue;
        int run = (prev_idx - 1) - i;
        bits += run_before_bits_approx(run, zeros_left);
        zeros_left -= run;
        rb_count++;
        prev_idx = i;
    }
    (void)run_before_total;

    return bits;
}

/* ===========================================================================
 * REAL CAVLC ENCODER — emits actual bits using spec tables.
 * =========================================================================== */

/* Helper: emit a vlc_t. */
static void emit_vlc(bitstream_t *bs, vlc_t v)
{
    bs_put_bits(bs, v.code, v.len);
}

/* Pick coeff_token sub-table by (block type, nC). */
static const vlc_t *coeff_token_lookup(int total_coef, int trailing_ones,
                                       block_type_t bt, int nC)
{
    if (bt == BLK_CHROMA_DC) {
        return &coeff_token_chroma_dc[total_coef][trailing_ones];
    }
    if (nC < 2)  return &coeff_token_nc01[total_coef][trailing_ones];
    if (nC < 4)  return &coeff_token_nc23[total_coef][trailing_ones];
    if (nC < 8)  return &coeff_token_nc47[total_coef][trailing_ones];
    /* nC >= 8: 6-bit FLC. Spec Table 9-5 last column: TC=0 is special
     * (code=000011=3); for TC>0, code = ((TC-1)<<2) | T1.
     * Verified by reading spec rows for TC=8..15. */
    static vlc_t flc;
    if (total_coef == 0)
        flc.code = 0x03, flc.len = 6;
    else
        flc.code = ((total_coef - 1) << 2) | trailing_ones, flc.len = 6;
    return &flc;
}

/* Emit one level using spec 9.2.2 unary-prefix + suffix encoding.
 * Returns updated suffix_length. */
static int emit_level(bitstream_t *bs, int level_code, int suffix_length)
{
    int level_prefix;

    if (suffix_length == 0) {
        if (level_code < 14) {
            /* Prefix-only: level_prefix zeros + '1'. */
            bs_put_bits(bs, 0, level_code);
            bs_put_bits(bs, 1, 1);
        } else if (level_code < 30) {
            /* prefix=14 + 4-bit suffix */
            bs_put_bits(bs, 0, 14);
            bs_put_bits(bs, 1, 1);
            bs_put_bits(bs, level_code - 14, 4);
        } else {
            /* Escape: prefix=15 + 12-bit suffix */
            bs_put_bits(bs, 0, 15);
            bs_put_bits(bs, 1, 1);
            bs_put_bits(bs, level_code - 30, 12);
        }
    } else {
        level_prefix = level_code >> suffix_length;
        if (level_prefix < 15) {
            bs_put_bits(bs, 0, level_prefix);
            bs_put_bits(bs, 1, 1);
            bs_put_bits(bs, level_code & ((1 << suffix_length) - 1),
                        suffix_length);
        } else {
            /* Escape for non-zero suffix_length */
            bs_put_bits(bs, 0, 15);
            bs_put_bits(bs, 1, 1);
            int escape = level_code - (15 << suffix_length);
            bs_put_bits(bs, escape, 12);
        }
    }
    return suffix_length;
}

/* Emit total_zeros given block-type-aware table. */
static void emit_total_zeros(bitstream_t *bs, int total_zeros,
                             int total_coef, block_type_t bt)
{
    if (bt == BLK_CHROMA_DC) {
        emit_vlc(bs, total_zeros_chroma_dc[total_coef - 1][total_zeros]);
    } else {
        emit_vlc(bs, total_zeros_4x4[total_coef - 1][total_zeros]);
    }
}

/* Emit run_before. */
static void emit_run_before(bitstream_t *bs, int run, int zeros_left)
{
    int idx = (zeros_left > 6) ? 6 : (zeros_left - 1);
    emit_vlc(bs, run_before_tab[idx][run]);
}

int cavlc_encode_block(bitstream_t *bs, const i16 *coefs, int n_coefs,
                       block_type_t bt, int nC)
{
    int start_bits = bs->byte_pos * 8 + bs->n_in_cur;

    /* Find last nonzero, count total. */
    int last_nz = -1;
    for (int i = 0; i < n_coefs; i++)
        if (coefs[i] != 0) last_nz = i;

    if (last_nz < 0) {
        /* Empty block: emit "no coef" coeff_token. */
        emit_vlc(bs, *coeff_token_lookup(0, 0, bt, nC));
        return bs->byte_pos * 8 + bs->n_in_cur - start_bits;
    }

    int total_coef = 0;
    for (int i = 0; i <= last_nz; i++)
        if (coefs[i] != 0) total_coef++;

    /* Trailing ones: ±1 values from last_nz back, max 3, contiguous. */
    int t1 = 0;
    int t1_signs[3] = {0,0,0};   /* 0=positive, 1=negative; emitted in
                                   * scan order from highest freq */
    for (int i = last_nz; i >= 0; i--) {
        if (coefs[i] == 0) continue;
        if (t1 < 3 && (coefs[i] == 1 || coefs[i] == -1)) {
            t1_signs[t1++] = (coefs[i] < 0) ? 1 : 0;
        } else {
            break;
        }
    }

    /* 1. coeff_token */
    emit_vlc(bs, *coeff_token_lookup(total_coef, t1, bt, nC));

    /* 2. trailing_ones_sign_flags (highest freq first). */
    for (int i = 0; i < t1; i++) {
        bs_put_bits(bs, t1_signs[i], 1);
    }

    /* 3. Levels: walk from last_nz down, skip the t1 trailing ones already
     *    emitted as signs. */
    int suffix_length = (total_coef > 10 && t1 < 3) ? 1 : 0;
    int t1_seen = 0;
    int first_non_t1 = 1;

    for (int i = last_nz; i >= 0; i--) {
        if (coefs[i] == 0) continue;
        if (t1_seen < t1 && (coefs[i] == 1 || coefs[i] == -1)) {
            t1_seen++;
            continue;
        }
        int level = coefs[i];
        int abs_level = level < 0 ? -level : level;

        /* level_code = 2*(abs-1) + (sign==neg) */
        int level_code = (abs_level - 1) * 2 + (level < 0 ? 1 : 0);

        if (first_non_t1 && t1 < 3) {
            level_code -= 2;     /* bias: smallest first-non-T1 has abs>=2 */
        }
        first_non_t1 = 0;

        emit_level(bs, level_code, suffix_length);

        /* Update suffix_length per spec 9.2.2.1 step 6. */
        if (suffix_length == 0) suffix_length = 1;
        if (abs_level > (3 << (suffix_length - 1)) && suffix_length < 6)
            suffix_length++;
    }

    /* 4. total_zeros — only if TotalCoeff < block max. */
    int max_coefs = n_coefs;     /* matches the count we accept */
    int n_zeros = (last_nz + 1) - total_coef;
    if (total_coef < max_coefs) {
        emit_total_zeros(bs, n_zeros, total_coef, bt);
    }

    /* 5. run_before for nonzeros, emitted from highest-freq down to (but not
     *    including) lowest-freq. Spec semantics: run_before[i] = zeros
     *    between nz_{i-1} and nz_i in scan order, where nz_0 is lowest-freq.
     *    My old code emitted "zeros after nz_i toward last_nz", which had
     *    the runs offset by one nonzero.
     */
    {
        int nz_pos[16];
        int n_nz = 0;
        for (int i = 0; i <= last_nz; i++) {
            if (coefs[i] != 0) nz_pos[n_nz++] = i;
        }
        int zeros_left = n_zeros;
        for (int idx = total_coef - 1; idx >= 1 && zeros_left > 0; idx--) {
            int run = nz_pos[idx] - nz_pos[idx - 1] - 1;
            emit_run_before(bs, run, zeros_left);
            zeros_left -= run;
        }
    }

    return bs->byte_pos * 8 + bs->n_in_cur - start_bits;
}

/* ===========================================================================
 * CAVLC DECODER
 * =========================================================================== */

/* Linear search a table column for a (code, length) match. Returns the row
 * index, or -1 if no match found (after consuming up to max_len bits). */
static int decode_vlc_search(bitreader_t *br, const vlc_t *table, int rows,
                             int cols, int col, int max_len)
{
    /* Try each row, accumulate bits one-by-one, match against shortest first. */
    /* We do a bit-by-bit walk: for each candidate length L (from 1 to
     * max_len), read L bits and check if any row has (code, L) matching. */
    u32 acc = 0;
    int acc_len = 0;

    while (acc_len < max_len) {
        acc = (acc << 1) | br_get_bits(br, 1);
        acc_len++;
        if (br->overflow) return -1;

        /* Find any row with this exact (code=acc, length=acc_len). */
        for (int r = 0; r < rows; r++) {
            const vlc_t *e = &table[r * cols + col];
            if (e->len == acc_len && e->code == acc) return r;
        }
    }
    return -1;
}

static int decode_coeff_token(bitreader_t *br, block_type_t bt, int nC,
                              int *out_total_coef, int *out_t1)
{
    const vlc_t *tab;
    int rows, cols = 4;
    int max_len;
    if (bt == BLK_CHROMA_DC) {
        tab = &coeff_token_chroma_dc[0][0];
        rows = 5;
        max_len = 8;
    } else if (nC < 2) {
        tab = &coeff_token_nc01[0][0];
        rows = 17;
        max_len = 16;
    } else if (nC < 4) {
        tab = &coeff_token_nc23[0][0];
        rows = 17;
        max_len = 16;
    } else if (nC < 8) {
        tab = &coeff_token_nc47[0][0];
        rows = 17;
        max_len = 10;
    } else {
        /* nC >= 8: 6-bit FLC. code=3 (000011) → (TC=0, T1=0).
         * Else TC = (code>>2)+1, T1 = code&3. Inverse of encoder above. */
        u32 v = br_get_bits(br, 6);
        if (v == 0x03) { *out_total_coef = 0; *out_t1 = 0; return 0; }
        int tc = (v >> 2) + 1;
        int t1 = v & 3;
        *out_total_coef = tc;
        *out_t1 = t1;
        return 0;
    }

    /* Walk all 4 columns simultaneously: try each possible (TC, T1) match
     * by accumulating bits and looking up. */
    u32 acc = 0;
    int acc_len = 0;
    while (acc_len < max_len) {
        acc = (acc << 1) | br_get_bits(br, 1);
        acc_len++;
        if (br->overflow) return -1;
        for (int tc = 0; tc < rows; tc++) {
            for (int t1 = 0; t1 < cols; t1++) {
                const vlc_t *e = &tab[tc * cols + t1];
                if (e->len == acc_len && e->code == acc) {
                    *out_total_coef = tc;
                    *out_t1 = t1;
                    return 0;
                }
            }
        }
    }
    return -1;
}

static int decode_total_zeros(bitreader_t *br, int total_coef,
                              block_type_t bt, int *out_tz)
{
    if (bt == BLK_CHROMA_DC) {
        const vlc_t *tab = &total_zeros_chroma_dc[total_coef - 1][0];
        int idx = decode_vlc_search(br, tab, 4, 1, 0, 3);
        if (idx < 0) return -1;
        *out_tz = idx;
        return 0;
    }
    const vlc_t *tab = &total_zeros_4x4[total_coef - 1][0];
    int idx = decode_vlc_search(br, tab, 16, 1, 0, 9);
    if (idx < 0) return -1;
    *out_tz = idx;
    return 0;
}

static int decode_run_before(bitreader_t *br, int zeros_left, int *out_run)
{
    int row = (zeros_left > 6) ? 6 : (zeros_left - 1);
    if (zeros_left <= 6) {
        const vlc_t *tab = &run_before_tab[row][0];
        int max_len = (row + 1);   /* 1..6 -> max 1..3-ish bits, conservative */
        int idx = decode_vlc_search(br, tab, 16, 1, 0, 12);
        if (idx < 0) return -1;
        *out_run = idx;
        (void)max_len;
        return 0;
    }
    /* zL >= 7: prefix unary-style with extension */
    u32 v = br_get_bits(br, 3);
    if (v != 0) {
        /* run_before in 0..6 */
        const u8 lookup[8] = {0, 0,0, 0,0, 0,0, 0};   /* unused */
        (void)lookup;
        /* run_before = 7 - v (since codes are 7,6,5,4,3,2,1 for runs 0..6) */
        *out_run = 7 - v;
        return 0;
    }
    /* v == 0 → continue reading more zeros for runs >= 7 */
    int more = 0;
    while (br_get_bits(br, 1) == 0) {
        if (br->overflow) return -1;
        if (++more > 11) return -1;
    }
    *out_run = 7 + more;
    return 0;
}

static int decode_level(bitreader_t *br, int suffix_length, int *out_code)
{
    /* Read level_prefix (unary): count zeros until we hit a 1. */
    int zeros = 0;
    while (br_get_bits(br, 1) == 0) {
        if (br->overflow) return -1;
        if (++zeros > 25) return -1;
    }
    int level_prefix = zeros;
    int level_code;
    if (suffix_length == 0) {
        if (level_prefix < 14) {
            level_code = level_prefix;
        } else if (level_prefix == 14) {
            u32 suf = br_get_bits(br, 4);
            level_code = 14 + suf;
        } else { /* level_prefix == 15 */
            u32 suf = br_get_bits(br, 12);
            level_code = 30 + suf;
        }
    } else {
        if (level_prefix < 15) {
            u32 suf = br_get_bits(br, suffix_length);
            level_code = (level_prefix << suffix_length) | suf;
        } else {
            u32 suf = br_get_bits(br, 12);
            level_code = (15 << suffix_length) + suf;
        }
    }
    *out_code = level_code;
    return 0;
}

int cavlc_decode_block(bitreader_t *br, i16 *out_coefs, int n_coefs,
                       block_type_t bt, int nC)
{
    /* Zero-fill output. */
    for (int i = 0; i < n_coefs; i++) out_coefs[i] = 0;

    int total_coef, t1;
    if (decode_coeff_token(br, bt, nC, &total_coef, &t1) < 0) return -1;
    if (total_coef == 0) return 0;

    /* Decode trailing-ones signs (highest-freq first). */
    i16 levels[16];   /* in encoded order (highest freq → lowest freq) */
    for (int i = 0; i < t1; i++) {
        u32 sign = br_get_bits(br, 1);
        levels[i] = sign ? -1 : +1;
    }

    /* Decode non-T1 levels. */
    int suffix_length = (total_coef > 10 && t1 < 3) ? 1 : 0;
    int first_non_t1 = 1;
    for (int i = t1; i < total_coef; i++) {
        int level_code;
        if (decode_level(br, suffix_length, &level_code) < 0) return -1;

        if (first_non_t1 && t1 < 3) level_code += 2;
        first_non_t1 = 0;

        int abs_level = (level_code >> 1) + 1;
        int sign = (level_code & 1) ? -1 : +1;
        levels[i] = (i16)(sign * abs_level);

        if (suffix_length == 0) suffix_length = 1;
        if (abs_level > (3 << (suffix_length - 1)) && suffix_length < 6)
            suffix_length++;
    }

    /* Decode total_zeros. */
    int tz = 0;
    if (total_coef < n_coefs) {
        if (decode_total_zeros(br, total_coef, bt, &tz) < 0) return -1;
    }

    /* Decode run_before for each nonzero except the last (lowest-freq). */
    int runs[16] = {0};
    int zeros_left = tz;
    for (int i = 0; i < total_coef - 1; i++) {
        if (zeros_left == 0) { runs[i] = 0; continue; }
        int run;
        if (decode_run_before(br, zeros_left, &run) < 0) return -1;
        runs[i] = run;
        zeros_left -= run;
    }
    runs[total_coef - 1] = zeros_left;

    /* Place levels in the output array.
     * Encoded order: highest-freq → lowest-freq.
     * Run sequence: runs[i] is zeros AFTER level i (in encoded order).
     * In zigzag scan order (output array), the lowest-freq nonzero is at
     * index 0; highest-freq is at index (total_coef + tz - 1). */
    int last_nz_zz = total_coef + tz - 1;   /* index in zigzag array */
    int pos = last_nz_zz;
    for (int i = 0; i < total_coef; i++) {
        out_coefs[pos] = levels[i];
        pos -= runs[i] + 1;
    }
    return 0;
}

