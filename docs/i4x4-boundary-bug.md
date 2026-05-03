# I_4x4 ↔ I_16x16 Boundary Bug — Investigation Notes

## Status

**Bug live, workaround in place.** The I_4x4 macroblock-emit path was
grafted onto the staged pipeline (commit `8be8485`). The path works in
isolation: every MB I_4x4 is byte-exact against ffmpeg's decoder, every
MB I_16x16 is byte-exact, but **mixing the two is not** — when the path
selector picks I_4x4 for some MBs and I_16x16 for others, the decoder's
recon diverges from ours starting at the first I_4x4 ↔ I_16x16
macroblock boundary.

The current shipping baseline (`bits_b = INT32_MAX` in
`mb_mode_decide`, commit `1b01aac`) forces I_16x16 selection. All 145
dataset byte-exact tests pass with that workaround. The full I_4x4 emit
path (Table 9-4 CBP, mode prediction, full-coef CAVLC blocks, etc.)
remains compiled and intact.

## Symptoms

`tools/validate_byteexact.sh` against `datasets/kodak/`:

- **I_16x16-only forced** (`bits_b = INT32_MAX`): 120/120 byte-exact.
- **I_4x4-only forced** (`bits_a = INT32_MAX`): byte-exact on every
  case I sampled (kodim01 at QP 18/26/38).
- **Selector active** (no force): 0/120 byte-exact. ffmpeg accepts the
  bitstream cleanly (no parser errors), but recon diverges.

The first divergence is consistently at the **first byte of the first
I_16x16 MB whose neighbor was I_4x4**. For kodim01 at QP 30, that's
MB (0, 7) — left neighbor MB (0, 6) is I_4x4, the diff is at byte 113
(row 0, col 112 = MB-relative col 0). Our recon shows 143, ffmpeg's
shows 137. Bytes 0..111 are byte-exact, including byte 111 (the
rightmost col of MB (0, 6)) which is the input to MB (0, 7)'s prediction.

So:
- prediction inputs match between our encoder and ffmpeg's decoder
- yet MB (0, 7) reconstructs to a different result

## What Has Been Ruled Out

### CAVLC encode/decode roundtrip
`tests/test_cavlc_roundtrip.c` passes 563/563 across all block types
including `BLK_LUMA_FULL` (16-coef I_4x4 blocks).

Additionally I added a `#ifdef CAVLC_SELFCHECK` block in `cavlc.c`
that, on every `cavlc_encode_block` call, encodes to a scratch buffer
and decodes it back, comparing levels in/out. Built with `-DCAVLC_SELFCHECK`
and ran the failing case (kodim01 QP30 with I_4x4 selector active):
**zero CAVLC mismatches reported**. The CAVLC layer is consistent given
the encoder's nC.

### Table 9-4 CBP intra mapping
Cross-checked the 48-entry inverse map I added to `cavlc_tables.h`
against the spec PDF (T-REC-H.264-200305-S, page 156, Table 9-4). All
codeNum → cbp_value entries match.

### `mb_type` formula for I_16x16
`mb_type = 1 + mode + 4*cbpC + 12*cbpL` matches Table 7-11. For I_NxN,
`mb_type = 0`. Both round-trip through ue(v) correctly — a separate
`bs_put_ue / br_get_ue` test confirmed.

### Mode-prediction round-trip (spec 8.3.1.1)
For each block in an I_4x4 MB, traced `predIntra4x4PredMode` derivation
on both encoder and decoder side under the assumption that the decoder
implements the spec verbatim. With:
- `mode_top` from `luma_mode4[gy_above*w4 + gx]` if above neighbor in
  same MB or in a previous I_4x4 MB; else `LUMA_MODE4_NONI4 = -1` for
  I_16x16 neighbors.
- `mode_left` symmetric.
- `pred = (top<0 || left<0) ? DC : min(top, left)`.

The encoder emits `prev_intra4x4_pred_mode_flag = (actual==pred)` and
`rem = actual<pred ? actual : actual-1` when prev=0. Decoder recovers
`actual = rem<pred ? rem : rem+1`. All cases I worked through round-trip
correctly.

### nC / TotalCoeff semantics (spec 9.2.1.1)
The spec says `nN` for the neighbor block is `TotalCoeff(coeff_token)`
of that block, with these special cases:

- If neighbor is unavailable, or its corresponding cbp_luma bit is 0,
  `nN = 0`.
- If neighbor is in an Intra_16x16 macroblock, the spec note says
  "nA and nB is the number of decoded non-zero AC transform coefficient
  levels" — i.e. AC-only count (positions 1..15).
- Otherwise (I_4x4, P, etc.): full count (positions 0..15 for I_4x4).

The encoder stores:
- I_16x16 emit: `count_nonzero(&zz[1], 15)` — AC-only ✓
- I_4x4 emit: `count_nonzero(zz, 16)` — full ✓
- I_4x4 emit with `quad_bit==0`: stores 0 ✓

These match spec. The decoder uses the same convention.

### Residual emission order
For I_4x4 the 16 luma blocks emit in scan order using
`blk_scan_br/blk_scan_bc`. Each block emitted only if its 8x8
quadrant's cbp_luma bit (= s/4) is set. Decoder reads in the same
scan order. Verified.

### `try_path_i4x4` side effects
Inspected the function — it takes `const u8 *recon_y_frame` and writes
only to its own output buffers (`modes4_out`, `ac_levels_out`,
`recon_mb_out`). No global side effects. The selector picks the winner
and copies into `mb_state_t` fields with `memcpy`. The losing path's
intermediate state is discarded.

### Path selection symmetry
With `bits_b = INT32_MAX`, the workaround forces path A (I_16x16)
always, and dataset is byte-exact. With `bits_a = INT32_MAX`, force
path B (I_4x4) — also byte-exact (sampled). The bug only manifests
when the selector actually mixes paths.

### Per-MB forcing — bug is content/mode-specific
Forcing only **MB (0, 2)** to I_4x4 (rest I_16x16) on kodim01 at QP 30
triggers the bug: 64 byte diffs in MB (0, 2) at rows 12..15 (the
br=3 sub-blocks).

But forcing only **MB (0, 1)** to I_4x4 (same neighbor structure —
left neighbor MB (0, 0) is I_16x16) is **byte-exact**. Same with
forcing only MB (0, 3). Same with forcing MB (0, 1) + MB (0, 2)
together (MB (0, 2)'s left neighbor becomes I_4x4 in this case).

Conclusion: the bug is NOT simply "I_4x4 with I_16x16 left neighbor".
It depends on the specific 4x4 modes selected for the bottom-row
blocks of the MB and the relationship to the I_16x16 neighbor's state.

### Localization to the br=3 row of an I_4x4 MB
On the failing MB (0, 2), the diffs are confined to rows 12..15 of
the MB — i.e., the four blocks at br=3 (raster idx 12, 13, 14, 15
= scan idx 9, 10, 14, 15 — the bottom-most 4x4 row).

Of those four blocks in MB (0, 2)'s case, the chosen modes (raster
order at idx 12..15) were `[1, 1, 8, 1]` — three HORIZONTAL and one
HORIZONTAL_UP.

For block 10 (raster 12 = (br=3, bc=0), mode 1 HORIZONTAL):
- our recon row 12 col 32 = 109, ffmpeg = 123 (diff +14)
- our recon row 13 col 32 = 65, ffmpeg = 116 (diff +51)
- our recon row 14 col 32 = 148, ffmpeg = 136 (diff -12)
- our recon row 15 col 32 = 144, ffmpeg = 129 (diff -15)

Pred (HORIZONTAL = left[r] for all c) is the SAME on both sides
(left = [109, 80, 134, 143] from MB (0, 1) col 31, byte-exact).
CAVLC levels emitted are roundtrip-correct. So either the dequant
formula or the iDCT formula differs between us and ffmpeg, OR ffmpeg
is decoding from a DIFFERENT BIT POSITION (mb_type / cbp parsing
misalignment that doesn't surface as a parser error).

The hand-traced math for our internal recon at this block matches
exactly: pred=109, levels=[0,0,0,0,-1,0,0,0,0,0,0,0,1,0,0,0],
dq col-0 = [0, -416, 0, 416], iDCT row 0 = -208, recon row 0 col 0
= 109 + (-208+32)>>6 = 109 - 3 = 106. (Or 109 in this specific
forced-pattern test where pred differs slightly due to recon plane
state.) The math is internally consistent.

The remaining most likely cause: **ffmpeg is parsing a different
mb_type or cbp_luma than what we emit**, putting its bitstream
parser at a different bit offset for the residual data. This would
cause it to read DIFFERENT levels (with the same nC sub-table) and
produce a different recon. The CAVLC self-check doesn't catch this
because it only validates that our own decoder agrees with our own
encoder at the call-site nC.

To confirm, the next investigative step is a full per-MB self-decoder
that parses the emitted bitstream from the start of the slice through
the failing MB and reports the parsed mb_type, cbp_luma, modes, and
levels at each step.

## Plausible Remaining Causes

These haven't been fully verified and would be the next things to check.

### 1. Subtle nC misread for the I_16x16 luma DC block

The first emission inside an I_16x16 MB is the luma DC block
(`BLK_LUMA_DC_16x16`, 16 coefs). nC for this block is special — spec
9.2.1.1's note says it reads from "non-zero transform coefficient levels
in adjacent 4x4 blocks", i.e. the 4x4 luma_nc grid, NOT from neighbor
DC blocks. My code:

```c
int gx0 = st->mb_c * 4;
int gy0 = st->mb_r * 4;
int top_nc  = (gy0 > 0) ? ncs->luma_nc[(gy0 - 1) * luma_w4 + gx0] : 0;
int left_nc = (gx0 > 0) ? ncs->luma_nc[gy0 * luma_w4 + (gx0 - 1)] : 0;
int nC = cavlc_compute_nC(top_nc, left_nc, gy0 > 0, gx0 > 0);
```

This reads from `(gy0-1, gx0)` (top) and `(gy0, gx0-1)` (left) — i.e.
the 4x4 block above MB-relative (0,0) and left of MB-relative (0,0).

When the left neighbor MB is I_4x4, that 4x4 grid slot has the
**16-coef** TotalCoeff stored. Spec rule says: for an I_16x16
**neighbor** use AC-only; for I_4x4 neighbor use full. Default
applies — full count. So encoder reads the 16-coef value, decoder
likewise. Should match.

**But** — what if the spec's "AC-only when looking at I_16x16 neighbor"
rule applies to the **CURRENT block's nC interpretation**, not just to
the neighbor? I.e., does an I_16x16 block looking at an I_4x4 neighbor
need to use a different transformation of the stored count?

This is worth re-reading the spec literally and tracing through ffmpeg's
implementation.

### 2. Off-by-one in luma_nc grid indexing for the I_16x16 DC block

Spec says block 0's neighbors are "the top-left 4x4 block of the upper
neighbor macroblock and the bottom-left 4x4 block of the left neighbor
macroblock". My code reads `(gy0-1, gx0)` and `(gy0, gx0-1)`.

For MB (mb_r, mb_c): block 0 is at MB-relative (br=0, bc=0). Its
top neighbor 4x4 is at MB-relative (-1, 0) — the **bottom-left** of
the MB above. Its left neighbor 4x4 is at MB-relative (0, -1) — the
**top-right** of the MB to the left.

Hmm — but the spec phrase says "bottom-left of the LEFT neighbor MB"
for block 0's nC. Let me re-check that. Actually looking at spec 9.2.1.1
combined with spec 8.5.6 (DC residual block): the DC block is conceptually
a 4x4 block at position (0,0) of the MB. Its neighbor 4x4 blocks come
from the spec 6.4.10 process. For luma4x4BlkIdx=0: neighbor A (left) is
at MB-relative (0, -1) which is in the MB to the left's top-right 4x4
position. Neighbor B (top) is at MB-relative (-1, 0) which is the MB
above's bottom-left 4x4 position.

My code's indexing matches: `gx0-1 = mb_c*4 - 1` is the rightmost 4x4
column of MB to the left, and `gy0` (same row) means the top row of
that MB. So `luma_nc[gy0 * w4 + (gx0-1)] = luma_nc[mb_r*4 + (mb_c*4-1)]`
is at the top-right 4x4 of MB-to-the-left. ✓

Actually wait — the spec phrase I wrote says "bottom-left of the LEFT
neighbor MB". That's MB-relative (3, -1) of the left MB, NOT (0, -1).
That would be `luma_nc[(gy0+3) * w4 + (gx0-1)]`, which my code does NOT
read. Need to verify the spec text precisely.

**This is a real candidate** — the I_16x16 luma DC block's nC neighbor
indexing might be wrong, but only manifests when the neighbor is I_4x4
(because in I_16x16-only the bug would cancel out: every 4x4 slot has
the same count semantics).

Wait — that doesn't quite hold. If the indexing is wrong, then in
I_16x16-only the bug should ALSO show because the encoder and decoder
would both make the same mistake... unless they make the SAME wrong
indexing and so happen to agree.

This needs careful checking against the spec figure (Figure 6-11) and
ideally ffmpeg's source.

### 3. cbp_chroma encoding interaction

For I_4x4: `cbp = cbp_luma | (cbp_chroma << 4)`, then me() through
Table 9-4. For I_16x16: cbp is folded into mb_type, no me().

If my I_4x4 cbp_value computation gets the chroma bits wrong, the
decoder reads a different cbp_chroma → emits/skips chroma blocks
inconsistent with our state → mis-aligns the bitstream.

But ffmpeg accepts the bitstream cleanly with no parser error. If
chroma block count was wrong, ffmpeg would over- or under-read residual
blocks and likely throw a parse error.

Lower-probability cause but worth verifying with a self-decode.

### 4. I_4x4 luma block positional dequant

The spec uses different quantization scaling for "DC of I_16x16 luma"
vs "AC of I_16x16 luma" vs "I_4x4 luma" (which is 16-position, no DC
extraction). My `iquant_4x4` applies `level[i] * V[qmod][i] << qdiv`
uniformly across all 16 positions.

For I_4x4, the spec says position 0 (DC of the 4x4) uses the same
positional scaling factor as the AC positions (since I_4x4 doesn't
separate DC). Verified that `iquant_4x4` does this — `V[qmod][0]` is
present in the table.

For I_16x16, position 0 is replaced by `iHadamard(quant_dc)` results
before iDCT. That's what `mb_reconstruct` does in path A. Verified.

So forward+inverse should round-trip in both paths. And in fact they do
in isolation — that's why forced I_4x4 and forced I_16x16 are byte-exact.

The bug is specifically the cross-MB interaction.

## Debugging Strategies (Ranked by Expected Yield)

### Strategy A (highest yield): self-decode harness

Add a debug-only function that, after each `mb_cavlc_emit` call,
re-decodes the bytes just emitted using `cavlc_decode_block` and
compares the decoded levels to what was emitted. If a mismatch shows
up, the bug is in the bit-level emission. If decoded levels match, the
bug is downstream (recon, neighbor state).

This is straightforward to add — `bitreader_t` and `cavlc_decode_block`
already exist in the codebase (`tests/test_cavlc_roundtrip.c` uses
them). Wrap the decode in a `#ifdef DEBUG_SELFCHECK` block so it's
opt-in.

The harness would walk the bitstream from the start of the IDR slice
each time (since `bs_put_bits` doesn't yield bit-position-anchored
checkpoints), or instrument `cavlc_encode_block` to record start/end
offsets per call.

### Strategy B: spec re-read with exact section references

Re-read spec 9.2.1.1 with the spec PDF open, transcribing the exact
text — not paraphrasing — for these conditions:
- nC computation for I_16x16 luma DC block
- nC computation for I_16x16 luma AC blocks at the MB boundary
- nC computation for I_4x4 blocks looking at I_16x16 neighbor
- Neighbor 4x4 block indexing per spec 6.4.10 (Figure 6-11)

Compare against my implementation line by line.

### Strategy C: ffmpeg source comparison

Read ffmpeg's `libavcodec/h264_cavlc.c` (specifically `decode_residual`,
`pred_non_zero_count`, and `decode_mb_cavlc` for I_4x4). It's the
canonical decoder; anywhere my encoder differs from its expectation,
that's the bug.

The repo's `tools/diff_x264.py` already cross-checks CAVLC tables
against x264. A similar cross-check against ffmpeg's mode prediction
or CBP encoding might find the divergence.

### Strategy D: bisect by forcing patterns

Instead of "any MB I_4x4 if it scores better", force specific patterns:
- Every other MB I_4x4 (alternating) — does the bug appear at the 2nd MB?
- Every 2nd row I_4x4 — does the bug appear at the row boundary?
- A single I_4x4 MB at position (k, k) for k=1..N — does it always fail
  on the immediate-right I_16x16 neighbor?

Narrowing the failure pattern would point at what state crosses the
boundary. For example: if the bug only triggers on horizontally-adjacent
boundaries and not vertically, the issue is specifically left-neighbor
state (luma_nc at gx-1, recon at col mb_c*16-1).

### Strategy E: minimum reproducer + JM reference

Construct a synthetic 32×16 frame (two MBs side by side) where MB 0
chooses I_4x4 and MB 1 chooses I_16x16. Encode. If even this minimal
case fails, we have a tiny stream to dump byte-by-byte and trace through
JM's reference decoder (which is more verbose than ffmpeg) to find
where parsing diverges from our intent.

## Recommended Order

1. **Strategy A (self-decode)** — likely catches the bug in an hour
   of work, doesn't require external references.
2. If A reveals the bug is in bit emission: done, fix it.
3. If A says emission is correct: **Strategy B (spec re-read)** to find
   the semantic gap, focused on the I_16x16 luma DC block's neighbor
   indexing.
4. If B is inconclusive: **Strategy C (ffmpeg source)** to find the
   diff.

Strategy D is useful for narrowing if A/B/C don't pinpoint, but it's
slower since each pattern requires a code change.

## What I Tried This Session

- **Strategy A (CAVLC self-check)** ✓ — added `#ifdef CAVLC_SELFCHECK`,
  no mismatches caught. CAVLC layer is consistent given the encoder's nC.
- **Strategy B (spec re-read)** ✓ — confirmed nC TotalCoeff semantics,
  predIntra4x4PredMode rules, neighbor 4x4 derivation per 6.4.7.3.
  All match my implementation on paper.
- **Strategy D (per-MB forcing bisect)** ✓ — narrowed to: bug is
  content/mode-specific, not just any I_4x4-with-I_16x16-neighbor case.
  See "Per-MB forcing — bug is content/mode-specific" above.

## What's Most Promising Next

A **block-level self-decoder** that walks the emitted bitstream after
each MB and re-parses everything: mb_type, prev/rem mode flags,
intra_chroma_pred_mode, me(CBP), mb_qp_delta, and each residual block.
For each, compare to what we INTENDED to emit and dump the first
mismatch.

This is the only way to definitively distinguish:
- bit emission off (mb_type / cbp / mode flag wrong)
- bitstream alignment off (one syntax element corrupting the next)
- nC computation off (subtle spec interpretation)
- residual encoding off (less likely, since CAVLC selfcheck is clean)

It needs a moderate amount of code: enough state to parse a slice
header (already have `nal_write_slice_header` mirror), then per-MB
parsing logic. Could fit in ~200 lines reusing `bitreader_t` +
`cavlc_decode_block`.

## Out of Scope for This Document

- The `MAX_H` bump from 2160 → 2688 — that was a separate fix for the
  static-arena bounds check on tall drone images, unrelated to the
  mode-decision bug. Already shipped (`1b01aac`).
- Other potential I_4x4 issues (e.g. `intra_chroma_pred_mode` in the
  I_4x4 path's emission ordering, mb_qp_delta conditional) — these
  appear correct on inspection, but a thorough verification would happen
  alongside the self-decode harness.
