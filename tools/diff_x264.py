#!/usr/bin/env python3
"""Diff our cavlc_tables.h against x264's tables.c."""
import re
from pathlib import Path

X264_TABLES = Path(r"C:\Users\kamin\Documents\Projects\x264\common\tables.c").read_text()
OUR_TABLES = Path("src/cavlc_tables.h").read_text()


def parse_x264_table(text, name):
    """Parse x264 vlc_t table. Returns list of lists of (code, len) or None."""
    start = text.index(f"const vlc_t {name}")
    # Find end of declaration: closing };
    depth = 0
    i = start
    in_table = False
    while i < len(text):
        c = text[i]
        if c == '{':
            depth += 1
            in_table = True
        elif c == '}':
            depth -= 1
            if in_table and depth == 0:
                end = i + 1
                break
        i += 1
    block = text[start:end]
    # Split by top-level { /* table N */ ... }
    # Each row is { /* i_total N */ entries }
    # Each entry is { 0xN, M }
    rows = []
    current_table = []
    current_row = []
    depth = 0
    pos = block.index('{')  # outermost
    i = pos + 1
    table_open = False
    row_open = False
    entries = []
    # Simpler: find all { 0xN, M } pairs and group by row
    # Track depth via { and }
    out = []
    cur_table = []
    cur_row = []
    depth = 0
    j = 0
    while j < len(block):
        c = block[j]
        if c == '{':
            depth += 1
            j += 1
            continue
        if c == '}':
            depth -= 1
            if depth == 1 and cur_row:
                # Just closed an entry — but entries are { 0xN, M } at depth 3 typically
                pass
            j += 1
            continue
        # Try to match entry pattern
        m = re.match(r'\s*0x([0-9a-fA-F]+)\s*,\s*(\d+)\s*', block[j:])
        if m:
            code = int(m.group(1), 16)
            length = int(m.group(2))
            cur_row.append((code, length))
            j += m.end()
            continue
        # Detect row boundary via comment "/* i_total N */"
        m = re.match(r'/\* i_total (\d+)', block[j:])
        if m:
            if cur_row:
                cur_table.append(cur_row)
                cur_row = []
            j += m.end()
            continue
        # Detect table boundary
        m = re.match(r'/\* table (\d+)', block[j:])
        if m:
            if cur_row:
                cur_table.append(cur_row)
                cur_row = []
            if cur_table:
                out.append(cur_table)
                cur_table = []
            j += m.end()
            continue
        j += 1
    if cur_row:
        cur_table.append(cur_row)
    if cur_table:
        out.append(cur_table)
    return out


def parse_our_table(text, name, rows, cols):
    start = text.index(f'static const vlc_t {name}[{rows}][{cols}] = ' + '{')
    end = text.index('};', start)
    block = text[start:end]
    entries = re.findall(r'\{\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*(\d+)\s*\}', block)
    parsed = []
    for code_s, len_s in entries:
        code = int(code_s, 16) if code_s.startswith('0x') else int(code_s)
        length = int(len_s)
        parsed.append((code, length) if length > 0 else None)
    return [parsed[i*cols:(i+1)*cols] for i in range(rows)]


def parse_our_total_zeros(text, name, rows, cols):
    return parse_our_table(text, name, rows, cols)


# x264 has 6 tables in coeff_token: [0]=nc01, [1]=nc23, [2]=nc47, [3]=nc>=8 (FLC),
# [4]=chroma_dc 4x4, [5]=chroma_dc 2x4 (not used).
x264_ct = parse_x264_table(X264_TABLES, "x264_coeff_token")
print(f"Parsed x264_coeff_token: {len(x264_ct)} sub-tables")
for i, t in enumerate(x264_ct):
    print(f"  table {i}: {len(t)} TC rows")

# Compare nc01 (x264 table 0)
our_nc01 = parse_our_table(OUR_TABLES, 'coeff_token_nc01', 17, 4)
print("\n=== coeff_token_nc01 ===")
# our_nc01[0] is TC=0 (special), our_nc01[1..16] is TC=1..16.
# x264_ct[0][0..15] is TC=1..16.
for tc in range(1, 17):
    x_row = x264_ct[0][tc-1] if tc-1 < len(x264_ct[0]) else []
    our_row = our_nc01[tc]
    for t1 in range(min(len(x_row), 4)):
        x = x_row[t1]
        o = our_row[t1] if our_row[t1] else None
        if o is None or x != o:
            print(f"  TC={tc} T={t1}: x264={x} ours={o}")

our_nc23 = parse_our_table(OUR_TABLES, 'coeff_token_nc23', 17, 4)
print("\n=== coeff_token_nc23 ===")
for tc in range(1, 17):
    x_row = x264_ct[1][tc-1] if tc-1 < len(x264_ct[1]) else []
    our_row = our_nc23[tc]
    for t1 in range(min(len(x_row), 4)):
        x = x_row[t1]
        o = our_row[t1] if our_row[t1] else None
        if o is None or x != o:
            print(f"  TC={tc} T={t1}: x264={x} ours={o}")

our_nc47 = parse_our_table(OUR_TABLES, 'coeff_token_nc47', 17, 4)
print("\n=== coeff_token_nc47 ===")
for tc in range(1, 17):
    x_row = x264_ct[2][tc-1] if tc-1 < len(x264_ct[2]) else []
    our_row = our_nc47[tc]
    for t1 in range(min(len(x_row), 4)):
        x = x_row[t1]
        o = our_row[t1] if our_row[t1] else None
        if o is None or x != o:
            print(f"  TC={tc} T={t1}: x264={x} ours={o}")

# Chroma DC: x264 table 4
our_cdc = parse_our_table(OUR_TABLES, 'coeff_token_chroma_dc', 5, 4)
print("\n=== coeff_token_chroma_dc ===")
for tc in range(1, 5):
    x_row = x264_ct[4][tc-1] if tc-1 < len(x264_ct[4]) else []
    our_row = our_cdc[tc]
    for t1 in range(min(len(x_row), 4)):
        x = x_row[t1]
        o = our_row[t1] if our_row[t1] else None
        if o is None or x != o:
            print(f"  TC={tc} T={t1}: x264={x} ours={o}")

# Total zeros
x264_tz = parse_x264_table(X264_TABLES, "x264_total_zeros")
print(f"\nParsed x264_total_zeros: {len(x264_tz)} tables, first table has {len(x264_tz[0])} rows")
# x264_total_zeros has 15 rows for TC=1..15
our_tz = parse_our_table(OUR_TABLES, 'total_zeros_4x4', 15, 16)
print("\n=== total_zeros_4x4 ===")
for tc in range(1, 16):
    if tc-1 >= len(x264_tz[0]):
        break
    x_row = x264_tz[0][tc-1]
    our_row = our_tz[tc-1]
    for tz_idx in range(16):
        x = x_row[tz_idx] if tz_idx < len(x_row) else None
        o = our_row[tz_idx] if tz_idx < len(our_row) else None
        if x != o:
            print(f"  TC={tc} tz={tz_idx}: x264={x} ours={o}")

# Chroma DC total zeros
x264_cdc_tz = parse_x264_table(X264_TABLES, "x264_total_zeros_2x2_dc")
print(f"\nParsed x264_total_zeros_2x2_dc: {len(x264_cdc_tz)} tables")
our_cdc_tz = parse_our_table(OUR_TABLES, 'total_zeros_chroma_dc', 3, 4)
print("\n=== total_zeros_chroma_dc ===")
for tc in range(1, 4):
    if tc-1 >= len(x264_cdc_tz[0]):
        break
    x_row = x264_cdc_tz[0][tc-1]
    our_row = our_cdc_tz[tc-1]
    for tz_idx in range(4):
        x = x_row[tz_idx] if tz_idx < len(x_row) else None
        o = our_row[tz_idx] if tz_idx < len(our_row) else None
        if x != o:
            print(f"  TC={tc} tz={tz_idx}: x264={x} ours={o}")

# Run before
x264_rb = parse_x264_table(X264_TABLES, "x264_run_before_init")
print(f"\nParsed x264_run_before_init: {len(x264_rb)} tables, first has {len(x264_rb[0])} rows")
# x264_run_before_init[7][16]: indexed by [zL_idx][run]
# We have run_before_tab[7][16]: same indexing
our_rb = parse_our_table(OUR_TABLES, 'run_before_tab', 7, 16)
print("\n=== run_before_tab ===")
for zL in range(7):
    if zL >= len(x264_rb[0]):
        break
    x_row = x264_rb[0][zL]
    our_row = our_rb[zL]
    for run in range(16):
        x = x_row[run] if run < len(x_row) else None
        o = our_row[run] if run < len(our_row) else None
        if x != o:
            print(f"  zL_idx={zL} run={run}: x264={x} ours={o}")

print("\nDone.")
