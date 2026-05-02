#!/usr/bin/env python3
"""Parse CAVLC tables (9-5/9-7/9-8/9-9/9-10) from H.264 spec text dump.

Strategy: walk the text in line order. The pdftotext dump puts each cell of
the table on its own line, but sometimes consecutive cells with short codes
get merged onto a single line separated by a single space (which is also used
inside cells for readability every 4 bits). We disambiguate using the known
shape of each table.

For Table 9-5 we use the FLC trick: column 4 (nC>=8) is always a 6-bit FLC
with a known value: 0x03 for (TC=0,T1=0); else (TC-1)*4 + T1.
"""
from __future__ import annotations
import re
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding='utf-8')
SPEC = Path(__file__).parent.parent / "build/spec_plain.txt"

BIN_TOK = re.compile(r"^[01]+$")
BIN_LINE = re.compile(r"^[01 ]+$")


def bin_to_vlc(s: str):
    s = s.replace(" ", "")
    if not s or any(c not in "01" for c in s):
        return None
    return (int(s, 2), len(s))


def find_real_table(text: str, marker: str) -> int:
    """Find the position of the real table (skip TOC entry)."""
    starts = [m.start() for m in re.finditer(re.escape(marker), text)]
    for s in starts:
        chunk = text[s:s + 250]
        if "...." in chunk or re.search(r"\.{3,}\s*\d{2,3}", chunk):
            continue
        return s
    return -1


# ============================================================================
# Table 9-5: coeff_token
# ============================================================================
def parse_coeff_token(text: str):
    """Parse Table 9-5. Returns (nc01, nc23, nc47, cdc) where each is a
    [TC][T1] table; nc01/nc23/nc47 are 17x4, cdc is 5x4. Entries are
    (code_int, len_bits) or None.
    """
    nc01 = [[None] * 4 for _ in range(17)]
    nc23 = [[None] * 4 for _ in range(17)]
    nc47 = [[None] * 4 for _ in range(17)]
    cdc = [[None] * 4 for _ in range(5)]

    # First, collapse text into clean lines, dropping empty lines and obvious
    # page-header noise.
    lines = []
    for raw in text.splitlines():
        s = raw.strip()
        if not s:
            continue
        if s.startswith("ITU-T Rec."):
            continue
        # Drop bare page-number lines but ONLY if they're 3-digit (page-like).
        # Single-digit numbers are valid T1/TC values.
        if re.fullmatch(r"\d{3}", s):
            continue
        lines.append(s)

    # Now walk: each row consists of a T1 line (0-3), TC line (0-16), then
    # one or more code lines until we hit a "next T1 then TC" pair.
    def is_tc_line(idx: int) -> bool:
        if idx < 0 or idx >= len(lines):
            return False
        return bool(re.fullmatch(r"\d{1,2}", lines[idx]))

    def is_t1_then_tc(idx: int) -> bool:
        # T1 in 0..3, TC in 0..16, T1 <= TC (T1 = trailing ones <= TC).
        # When TC=0, T1=0 only.
        if idx + 1 >= len(lines):
            return False
        a, b = lines[idx], lines[idx + 1]
        if not (re.fullmatch(r"\d", a) and re.fullmatch(r"\d{1,2}", b)):
            return False
        t1 = int(a)
        tc = int(b)
        if t1 > 3 or tc > 16:
            return False
        if tc == 0 and t1 != 0:
            return False
        if tc >= 1 and t1 > min(3, tc):
            return False
        return True

    i = 0
    # Find first valid T1=0,TC=0 row.
    while i < len(lines):
        if is_t1_then_tc(i) and lines[i] == "0" and lines[i + 1] == "0":
            break
        i += 1
    if i >= len(lines):
        return nc01, nc23, nc47, cdc

    while i < len(lines):
        if not is_t1_then_tc(i):
            i += 1
            continue
        t1 = int(lines[i])
        tc = int(lines[i + 1])
        i += 2

        # Collect code tokens until the next T1/TC pair or non-bin line.
        toks = []
        while i < len(lines):
            if is_t1_then_tc(i):
                break
            # End of table marker: 9.2.2 etc.
            if not BIN_LINE.match(lines[i]) and lines[i] != "-":
                break
            if lines[i] == "-":
                toks.append("-")
                i += 1
                continue
            # Split on spaces; each part is a chunk of bits. Empty cells (a
            # single "-") will appear as their own entry.
            for part in lines[i].split():
                if part == "-" or BIN_TOK.match(part):
                    toks.append(part)
            i += 1

        # toks is now a flat list like ["0", "0", "1", "11", "1111",
        # "0000", "11", "01"] for TC=0 row.
        # We need to split into 5 cells: c1, c2, c3, c4, c5.
        # c4 is always 6 bits, value computed from (TC, T1).
        # Strategy: compute the bitstream bits of each token sequentially,
        # find the position where the next 6 bits = expected FLC. The tokens
        # before that point are c1 c2 c3 (split by token boundaries somehow);
        # tokens spanning the 6-bit window form c4; tokens after form c5.

        if tc == 0 and t1 == 0:
            flc_val = 0x03
        else:
            flc_val = (tc - 1) * 4 + t1
        flc_bits = format(flc_val, "06b")

        # Cumulative bit positions per token.
        def all_dash(tlist):
            return all(t == "-" for t in tlist)

        # Build list of (token, start_bit, end_bit) for binary tokens.
        bit_positions = []
        cumbit = 0
        for idx_tok, t in enumerate(toks):
            if t == "-":
                bit_positions.append((idx_tok, t, cumbit, cumbit))
            else:
                bit_positions.append((idx_tok, t, cumbit, cumbit + len(t)))
                cumbit += len(t)

        # Concatenate full bitstring of all binary tokens.
        full = "".join(t for t in toks if t != "-")

        # Find FLC position. There may be multiple matches. Prefer the one
        # aligned with token boundaries.
        candidates = []
        bit_idx = 0
        # Map bit position -> token index where that bit lies.
        # token_starts[k] = bit position of start of token k (-1 for "-")
        cum = 0
        token_bit_start = []  # parallel to toks
        for t in toks:
            if t == "-":
                token_bit_start.append(-1)
            else:
                token_bit_start.append(cum)
                cum += len(t)
        total_bits = cum

        # Find all FLC start positions in full.
        positions = []
        p = 0
        while True:
            j = full.find(flc_bits, p)
            if j < 0:
                break
            positions.append(j)
            p = j + 1

        # For each FLC position, determine the "k" such that tokens[0..k-1]
        # cover bits [0..j) and there's an integer m such that tokens[k..m-1]
        # cover bits [j..j+6). If found, prefer this candidate.
        best = None
        for j in positions:
            # Find token index k where bit position j starts.
            k = None
            for idx_tok, t in enumerate(toks):
                if t == "-":
                    continue
                if token_bit_start[idx_tok] == j:
                    k = idx_tok
                    break
            if k is None:
                continue
            # Find m such that token bit positions cover j..j+6.
            m = None
            cur = j
            for mi in range(k, len(toks)):
                if toks[mi] == "-":
                    continue
                cur += len(toks[mi])
                if cur == j + 6:
                    m = mi + 1
                    break
                if cur > j + 6:
                    break
            if m is None:
                continue
            # tokens[0..k-1] form c1+c2+c3 with their own splits.
            # tokens[k..m-1] form c4.
            # tokens[m..end] form c5 or "-".
            c4_bits = "".join(toks[a] for a in range(k, m) if toks[a] != "-")
            c5_tokens = toks[m:]
            # c5: if all "-" or empty -> "-". Else concatenate binary tokens.
            if not c5_tokens or all_dash(c5_tokens):
                c5 = "-"
            else:
                c5_bits = "".join(t for t in c5_tokens if t != "-")
                c5 = c5_bits if c5_bits else "-"

            # tokens[0..k-1]: split into c1, c2, c3.
            # If k is 0 (TC=0, T1=0 row) this whole prefix is empty and we
            # have no c1/c2/c3 — only the special row case.
            pre_tokens = [t for t in toks[:k] if t != "-"]
            if k == 0 or not pre_tokens:
                if tc == 0 and t1 == 0:
                    # Only c4 and c5 are valid for the TC=0 row.
                    c1 = c2 = c3 = None
                else:
                    # Shouldn't happen
                    c1 = c2 = c3 = None
            elif len(pre_tokens) == 3:
                # Each token is one cell.
                c1, c2, c3 = pre_tokens
            else:
                # Multiple tokens, need to split by lengths. Cells are listed
                # left-to-right as nC<2, 2<=nC<4, 4<=nC<8. Codes get shorter
                # as nC increases (higher activity prior). Use a heuristic:
                # try all 2-cut combinations. Pick one with len(c1) >=
                # len(c2) >= len(c3). Tiebreak by maximizing c1 length.
                cands = []
                n = len(pre_tokens)
                for a in range(1, n):
                    for b in range(a + 1, n + 1):
                        c1s = "".join(pre_tokens[:a])
                        c2s = "".join(pre_tokens[a:b])
                        c3s = "".join(pre_tokens[b:])
                        if not c1s or not c2s or not c3s:
                            continue
                        if len(c1s) >= len(c2s) >= len(c3s):
                            cands.append((c1s, c2s, c3s))
                if not cands:
                    # Try without monotone constraint
                    for a in range(1, n):
                        for b in range(a + 1, n + 1):
                            c1s = "".join(pre_tokens[:a])
                            c2s = "".join(pre_tokens[a:b])
                            c3s = "".join(pre_tokens[b:])
                            if c1s and c2s and c3s:
                                cands.append((c1s, c2s, c3s))
                if not cands:
                    c1 = c2 = c3 = None
                else:
                    # Prefer split closest to "spec column lengths".
                    # We don't know lengths precisely, but biggest-first works.
                    cands.sort(key=lambda x: (-len(x[0]), -len(x[1])))
                    c1, c2, c3 = cands[0]

            best = (c1, c2, c3, c4_bits, c5)
            break  # take first valid FLC alignment

        if best is None:
            # Couldn't parse. Print warning and continue.
            sys.stderr.write(f"WARN: failed to parse TC={tc} T1={t1} toks={toks}\n")
            continue

        c1, c2, c3, c4, c5 = best

        # Store
        if c1:
            v = bin_to_vlc(c1)
            if v:
                nc01[tc][t1] = v
        if c2:
            v = bin_to_vlc(c2)
            if v:
                nc23[tc][t1] = v
        if c3:
            v = bin_to_vlc(c3)
            if v:
                nc47[tc][t1] = v
        if c5 and c5 != "-":
            v = bin_to_vlc(c5)
            if v and tc <= 4:
                cdc[tc][t1] = v

    # Special: TC=0,T1=0 row — encoder uses 0x01, len 1 for nc01; 0x03, len 2
    # for nc23; 0xF, len 4 for nc47; chroma_dc 0x01, len 2.
    # The text dump has these on the very first row: "1", "11", "1111", and
    # chroma DC "01". Verify they got picked up.
    return nc01, nc23, nc47, cdc


# ============================================================================
# Tables 9-7/9-8: total_zeros for 4x4 blocks
# ============================================================================
def parse_total_zeros_4x4(text: str):
    """Returns out[TC-1][tz] where TC in 1..15."""
    out = [[None] * 16 for _ in range(15)]

    # Find Table 9-7 and Table 9-8 separately.
    pos7 = find_real_table(text, "Table 9-7")
    pos8 = find_real_table(text, "Table 9-8")
    pos9 = find_real_table(text, "Table 9-9")

    if pos7 < 0 or pos8 < 0 or pos9 < 0:
        return out

    text7 = text[pos7:pos8]
    text8 = text[pos8:pos9]

    parse_tz_block(text7, 1, 7, out)
    parse_tz_block(text8, 8, 15, out)
    return out


def parse_tz_block(text: str, tc_lo: int, tc_hi: int, out):
    """Parse a total_zeros sub-block. Each row is keyed by tz value, has
    (tc_hi - tc_lo + 1) code cells.

    The dump format: a header row with "<tc_lo>", "<tc_lo+1>", ..., "<tc_hi>".
    Then rows: <tz>, <code for tc_lo>, <code for tc_lo+1>, ..., <code for
    tc_hi>. Multiple cells may appear on a single line separated by spaces;
    individual code cells may have internal spaces too (every 4 bits).
    """
    n_tc = tc_hi - tc_lo + 1
    lines = []
    for raw in text.splitlines():
        s = raw.strip()
        if not s:
            continue
        if s.startswith("ITU-T Rec."):
            continue
        if re.fullmatch(r"\d{3}", s):
            continue
        lines.append(s)

    # Skip until we reach the data. The data starts after the row of TC
    # column labels. Find the start by looking for the line "0" followed by
    # binary lines.
    # The real first data row is tz=0 with codes for each TC.
    i = 0
    # Skip header: find "<tc_lo>" appearing alone on a line, then "<tc_lo+1>"
    # ...
    # Simpler: find position where we have "0" (tz=0) followed by binary
    # content.
    while i < len(lines) - 1:
        if lines[i] == "0":
            j = i + 1
            # Check if next few lines are binary or split-form for cell data.
            if j < len(lines) and (BIN_LINE.match(lines[j]) or lines[j] == "-"):
                break
        i += 1

    # Now walk row by row. Each row: a single-digit (tz value) line, then
    # n_tc cells. Cells may be split across lines, or several cells may be
    # bundled on one line.
    while i < len(lines):
        # Line should be the tz value.
        if not re.fullmatch(r"\d{1,2}", lines[i]):
            i += 1
            continue
        tz = int(lines[i])
        if tz < 0 or tz > 15:
            i += 1
            continue
        i += 1

        # Collect tokens until the next tz line (a small-int alone) or
        # non-binary line.
        toks = []
        while i < len(lines):
            s = lines[i]
            # If this is a small-int line that could be next tz, AND next
            # line is binary, treat as tz.
            if re.fullmatch(r"\d{1,2}", s):
                v = int(s)
                if v == tz + 1 and v <= 15:
                    # next tz
                    break
                # Could be a number embedded in the data? In total_zeros
                # tables data is binary only, so a stray digit is a tz.
                if v <= 15:
                    break
            if not BIN_LINE.match(s) and s != "-":
                break
            if s == "-":
                toks.append("-")
            else:
                for part in s.split():
                    if part == "-" or BIN_TOK.match(part):
                        toks.append(part)
            i += 1

        # Greedily consume tokens for n_tc columns.
        # Each cell is one or more consecutive tokens. Cell boundary is
        # ambiguous when codes are short.
        # Strategy: We don't know boundaries. So we use a different approach:
        # the tokens form a sequence of bit-chunks. Each cell is some prefix
        # of those tokens.
        # For total_zeros, valid range for column tc is 0..(16-tc). So we can
        # validate by: tz must be <= 16-tc for cell to be non-"-".
        # If cell is invalid (tz > 16-tc), the spec dump shows NOTHING (no
        # entry, not even "-"). It just skips.
        # Hmm, but it's hard to tell which columns are skipped.
        # Better: assume there's exactly one binary cell per valid column,
        # consumed left-to-right. Skip invalid columns (no token).
        valid_cols = [c for c in range(n_tc) if tz <= 16 - (tc_lo + c)]
        # Combine "-" tokens into individual cells too (they're literal "no
        # entry" markers).
        # Walk tokens: for each cell, consume one token. But cells with
        # codes of >4 bits get split into multiple tokens.
        # We don't know cell lengths a priori.
        # Heuristic: The vast majority of total_zeros codes are 1-9 bits.
        # If consecutive tokens form a >9-bit string, they might still be one
        # cell. So we just match counts: if tokens count == valid columns,
        # one token per cell. If more, lump consecutive tokens into cells
        # such that total cells = valid columns.

        cells = assign_cells(toks, len(valid_cols))
        for col_idx, val in zip(valid_cols, cells):
            if val == "-" or val == "":
                continue
            v = bin_to_vlc(val)
            if v is not None:
                tc = tc_lo + col_idx
                out[tc - 1][tz] = v


def assign_cells(toks, n_cells):
    """Assign tokens to cells. Use a heuristic: try to group tokens into
    n_cells cells, prefer assignments where consecutive cell lengths are
    monotonically non-decreasing (codes get longer for higher tz, lower TC).
    """
    if not toks or n_cells == 0:
        return []
    # Filter dashes -- they are "no entry" markers and count as cells.
    # Hmm, for total_zeros there shouldn't be "-" since invalid columns are
    # just skipped.
    if len(toks) == n_cells:
        return list(toks)
    if len(toks) < n_cells:
        # Pad with "-"
        return list(toks) + ["-"] * (n_cells - len(toks))

    # Need to group len(toks) tokens into n_cells groups. Use DP/heuristic:
    # try all combinations of (n_cells - 1) cut points. Score each by:
    # - all cells should be 1..16 bits (length of concatenated bits).
    # - Slight preference for smaller cells first / monotone.
    n = len(toks)
    best = None
    best_score = -1

    from itertools import combinations
    # Cut points are token indices in 1..n-1; choose n_cells-1 of them.
    for cuts in combinations(range(1, n), n_cells - 1):
        boundaries = [0] + list(cuts) + [n]
        cells = []
        ok = True
        for ci in range(n_cells):
            chunk = "".join(toks[boundaries[ci]:boundaries[ci + 1]])
            if not chunk or len(chunk) > 16:
                ok = False
                break
            cells.append(chunk)
        if not ok:
            continue
        # Score: prefer the assignment most likely to match spec. Spec total_
        # zeros codes get LONGER as TC increases for same tz, but actually it
        # depends. Without prior structure info, just take ANY valid split.
        # Tie-break by minimum variance in cell length (codes for one tz row
        # in total_zeros are usually similar length).
        lens = [len(c) for c in cells]
        score = -max(lens) - (max(lens) - min(lens))  # prefer compact, even
        if score > best_score:
            best_score = score
            best = cells

    return best if best else list(toks[:n_cells])


# ============================================================================
# Table 9-9: total_zeros for chroma DC 2x2 blocks
# ============================================================================
def parse_chroma_dc_tz(text: str):
    """Returns out[TC-1][tz] for TC in 1..3."""
    pos9 = find_real_table(text, "Table 9-9")
    pos10 = find_real_table(text, "Table 9-10")
    if pos9 < 0 or pos10 < 0:
        return [[None] * 4 for _ in range(3)]

    sub = text[pos9:pos10]
    out = [[None] * 4 for _ in range(3)]
    parse_tz_block_cdc(sub, out)
    return out


def parse_tz_block_cdc(text: str, out):
    n_tc = 3
    lines = []
    for raw in text.splitlines():
        s = raw.strip()
        if not s:
            continue
        if s.startswith("ITU-T Rec."):
            continue
        if re.fullmatch(r"\d{3}", s):
            continue
        lines.append(s)

    i = 0
    while i < len(lines) - 1:
        if lines[i] == "0":
            j = i + 1
            if j < len(lines) and BIN_LINE.match(lines[j]):
                break
        i += 1

    while i < len(lines):
        if not re.fullmatch(r"\d{1,2}", lines[i]):
            i += 1
            continue
        tz = int(lines[i])
        if tz < 0 or tz > 3:
            i += 1
            continue
        i += 1

        toks = []
        while i < len(lines):
            s = lines[i]
            if re.fullmatch(r"\d{1,2}", s):
                v = int(s)
                if v == tz + 1 and v <= 3:
                    break
                if v <= 3:
                    break
            if not BIN_LINE.match(s) and s != "-":
                break
            if s == "-":
                toks.append("-")
            else:
                for part in s.split():
                    if part == "-" or BIN_TOK.match(part):
                        toks.append(part)
            i += 1

        valid_cols = [c for c in range(n_tc) if tz <= 4 - (1 + c)]
        cells = assign_cells(toks, len(valid_cols))
        for col_idx, val in zip(valid_cols, cells):
            if val == "-" or val == "":
                continue
            v = bin_to_vlc(val)
            if v is not None:
                tc = 1 + col_idx
                out[tc - 1][tz] = v


# ============================================================================
# Table 9-10: run_before
# ============================================================================
def parse_run_before(text: str):
    """Returns out[zL_idx][run] for zL_idx in 0..6 (zL=1..6, then >=7).
    """
    pos10 = find_real_table(text, "Table 9-10")
    if pos10 < 0:
        return [[None] * 16 for _ in range(7)]

    end = text.find("9.2.4", pos10)
    if end < 0:
        end = len(text)
    sub = text[pos10:end]

    out = [[None] * 16 for _ in range(7)]

    lines = []
    for raw in sub.splitlines():
        s = raw.strip()
        if not s:
            continue
        if s.startswith("ITU-T Rec."):
            continue
        if re.fullmatch(r"\d{3}", s):
            continue
        lines.append(s)

    # Find data start
    i = 0
    while i < len(lines) - 1:
        if lines[i] == "0":
            j = i + 1
            if j < len(lines) and (BIN_LINE.match(lines[j]) or lines[j] == "-"):
                break
        i += 1

    while i < len(lines):
        if not re.fullmatch(r"\d{1,2}", lines[i]):
            i += 1
            continue
        run = int(lines[i])
        if run < 0 or run > 14:
            i += 1
            continue
        i += 1

        toks = []
        while i < len(lines):
            s = lines[i]
            if re.fullmatch(r"\d{1,2}", s):
                v = int(s)
                if v == run + 1 and v <= 14:
                    break
                if v <= 14:
                    break
            if not BIN_LINE.match(s) and s != "-":
                break
            if s == "-":
                toks.append("-")
            else:
                for part in s.split():
                    if part == "-" or BIN_TOK.match(part):
                        toks.append(part)
            i += 1

        # 7 columns: zL=1..6 then >=7. For each column zL, valid runs:
        # zL=1: run in 0..1
        # zL=2: run in 0..2
        # zL=k (1..6): run in 0..k
        # zL>=7: run in 0..14
        # If run > zL, the cell is "-" or absent.
        # The text dump explicitly uses "-" for invalid cells in this table.
        # So we have exactly 7 cells per row (some may be "-").

        # Parse: walk tokens, treating "-" as a cell, otherwise a binary
        # token may be one cell or split across multiple tokens.
        # We have 7 expected cells. Number of cells contributed by tokens
        # = 7. So we group binary tokens into cells using assign_cells but
        # taking dashes into account.

        # Split tokens into cells: each "-" is its own cell, binary runs
        # collected into cells using assign_cells.
        cells = []
        cur_bin = []
        # We don't know how many bin cells until we know how many "-"s there
        # are. So: count "-" tokens; remaining cells are binary; each binary
        # cell may span multiple tokens.
        n_dash = sum(1 for t in toks if t == "-")
        n_bin_cells = 7 - n_dash
        if n_bin_cells < 0:
            n_bin_cells = 0
        # Filter out dashes for grouping
        bin_toks = [t for t in toks if t != "-"]
        if n_bin_cells > 0 and bin_toks:
            bin_cells = assign_cells(bin_toks, n_bin_cells)
        else:
            bin_cells = []

        # Reconstruct cells in order, replacing dashes back.
        result = []
        bi = 0
        for t in toks:
            if t == "-":
                result.append("-")
            else:
                pass
        # Rebuild: walk toks, lump binary stretches together as one cell
        # each, but we need to know how many cells per stretch.
        # Simpler: assume each "stretch" of binary tokens between dashes is
        # one cell — usually true but spec may have multi-token cells.
        # For run_before, codes max 11 bits = 3 tokens of 4 bits. If we have
        # one stretch with 3 tokens and need 1 cell, lump them all.
        cells = []
        cur = []
        for t in toks:
            if t == "-":
                if cur:
                    cells.append("".join(cur))
                    cur = []
                cells.append("-")
            else:
                cur.append(t)
        if cur:
            cells.append("".join(cur))

        # If we got != 7 cells, try a different lumping (grouping binary
        # tokens differently to hit 7 cells).
        if len(cells) != 7:
            cells = []
            bin_idx = 0
            # Walk again, cell by cell, deciding whether to take 1 token or
            # combine tokens for binary cells.
            cells = redistribute_cells(toks, 7)

        if len(cells) != 7:
            sys.stderr.write(f"WARN run_before tz={run} cells={cells} toks={toks}\n")

        for col, val in enumerate(cells[:7]):
            if val == "-":
                continue
            zL = col + 1 if col < 6 else 7
            # Validate run range
            if col < 6 and run > zL:
                continue
            if col == 6 and run > 14:
                continue
            v = bin_to_vlc(val)
            if v is not None:
                out[col][run] = v


def redistribute_cells(toks, n_cells):
    """Distribute toks into exactly n_cells cells, where dash tokens are
    individual cells and binary tokens may be combined."""
    # Count dashes - they're fixed cells
    n_dash = sum(1 for t in toks if t == "-")
    n_bin_cells = n_cells - n_dash
    bin_toks = [t for t in toks if t != "-"]
    if n_bin_cells <= 0:
        # Just emit cells with "-" interleaved
        return list(toks)
    if n_bin_cells > len(bin_toks):
        # Pad
        bin_cells = bin_toks + ["-"] * (n_bin_cells - len(bin_toks))
    elif n_bin_cells == len(bin_toks):
        bin_cells = list(bin_toks)
    else:
        # Need to group bin_toks into n_bin_cells cells
        bin_cells = assign_cells(bin_toks, n_bin_cells)

    # Interleave back: walk toks, emit dash where dash, else pop from
    # bin_cells.
    result = []
    bi = 0
    seen_bin_in_run = False
    # Actually we need a more careful approach: position dashes where they
    # appeared. Since we changed the binary grouping, the binary cells
    # collectively represent the same span as bin_toks did.
    # Walk through original toks, tracking which "binary cell" we're in.
    cells_out = []
    bin_cell_idx = 0
    in_bin_run = False
    for t in toks:
        if t == "-":
            if in_bin_run:
                # End the current binary cell run; should have placed cells
                # already
                in_bin_run = False
            cells_out.append("-")
        else:
            in_bin_run = True
    # That's not quite right. Let me just do simpler: dashes interspersed
    # with binary runs. Each binary run gets divided into some number of
    # cells. Output order: dashes appear where they appear; binary cells
    # appear in order between/around dashes.

    # Groups: list of ("dash",) or ("bin", n_toks_in_run)
    groups = []
    cur_run = 0
    for t in toks:
        if t == "-":
            if cur_run:
                groups.append(("bin", cur_run))
                cur_run = 0
            groups.append(("dash",))
        else:
            cur_run += 1
    if cur_run:
        groups.append(("bin", cur_run))

    # Distribute n_bin_cells among bin groups. If only one bin group, all
    # cells go there. If multiple, distribute proportionally.
    n_bin_groups = sum(1 for g in groups if g[0] == "bin")
    if n_bin_groups == 0:
        return list(toks)
    if n_bin_groups == 1:
        # All cells in this group
        result = []
        ti = 0
        for g in groups:
            if g[0] == "dash":
                result.append("-")
            else:
                _, ntoks = g
                run_toks = bin_toks[ti:ti + ntoks]
                ti += ntoks
                grp_cells = assign_cells(run_toks, n_bin_cells)
                result.extend(grp_cells)
        return result
    else:
        # Multiple bin groups — assume each contributes equal share, plus
        # remainder to first.
        per = n_bin_cells // n_bin_groups
        rem = n_bin_cells - per * n_bin_groups
        bin_grp_idx = 0
        result = []
        ti = 0
        for g in groups:
            if g[0] == "dash":
                result.append("-")
            else:
                _, ntoks = g
                run_toks = bin_toks[ti:ti + ntoks]
                ti += ntoks
                want = per + (1 if bin_grp_idx < rem else 0)
                bin_grp_idx += 1
                if want <= 0:
                    want = 1
                grp_cells = assign_cells(run_toks, want)
                result.extend(grp_cells)
        return result


# ============================================================================
# Output generation
# ============================================================================
def fmt_vlc(v):
    if v is None:
        return "{0,0}"
    return f"{{0x{v[0]:X},{v[1]}}}"


def emit_header(nc01, nc23, nc47, cdc, tz, cdc_tz, rb, fp):
    p = lambda *a, **k: print(*a, **k, file=fp)
    p("/* AUTO-GENERATED from H.264 spec Tables 9-5/9-7/9-8/9-9/9-10. */")
    p("#ifndef DCC_CAVLC_TABLES_H")
    p("#define DCC_CAVLC_TABLES_H")
    p('#include "types.h"')
    p("typedef struct { u16 code; u8 len; } vlc_t;")
    p("")

    for name, tab in [("coeff_token_nc01", nc01),
                      ("coeff_token_nc23", nc23),
                      ("coeff_token_nc47", nc47)]:
        p(f"static const vlc_t {name}[17][4] = {{")
        for tc in range(17):
            entries = ", ".join(fmt_vlc(tab[tc][t1]) for t1 in range(4))
            p(f"  /* TC={tc:2d} */ {{ {entries} }},")
        p("};")
        p("")

    p("static const vlc_t coeff_token_chroma_dc[5][4] = {")
    for tc in range(5):
        entries = ", ".join(fmt_vlc(cdc[tc][t1]) for t1 in range(4))
        p(f"  /* TC={tc} */ {{ {entries} }},")
    p("};")
    p("")

    p("static const vlc_t total_zeros_4x4[15][16] = {")
    for tc in range(1, 16):
        entries = ", ".join(fmt_vlc(tz[tc - 1][t]) for t in range(16))
        p(f"  /* TC={tc:2d} */ {{ {entries} }},")
    p("};")
    p("")

    p("static const vlc_t total_zeros_chroma_dc[3][4] = {")
    for tc in range(3):
        entries = ", ".join(fmt_vlc(cdc_tz[tc][t]) for t in range(4))
        p(f"  /* TC={tc + 1} */ {{ {entries} }},")
    p("};")
    p("")

    p("static const vlc_t run_before_tab[7][16] = {")
    labels = ["zL=1", "zL=2", "zL=3", "zL=4", "zL=5", "zL=6", "zL>=7"]
    for col in range(7):
        entries = ", ".join(fmt_vlc(rb[col][r]) for r in range(16))
        p(f"  /* {labels[col]} */ {{ {entries} }},")
    p("};")
    p("")

    p("#endif")


def main():
    text = SPEC.read_text(encoding='utf-8', errors='replace')

    pos95 = find_real_table(text, "Table 9-5")
    end_95 = text.find("9.2.2", pos95)

    sub_95 = text[pos95:end_95]

    nc01, nc23, nc47, cdc = parse_coeff_token(sub_95)

    tz = parse_total_zeros_4x4(text)
    cdc_tz = parse_chroma_dc_tz(text)
    rb = parse_run_before(text)

    out_path = Path(__file__).parent.parent / "src/cavlc_tables.h"
    with open(out_path, "w", encoding="utf-8") as fp:
        emit_header(nc01, nc23, nc47, cdc, tz, cdc_tz, rb, fp)
    sys.stderr.write(f"Wrote {out_path}\n")


if __name__ == "__main__":
    main()
