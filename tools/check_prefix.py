#!/usr/bin/env python3
"""Check that all codes within each TC row are prefix-free."""
import re
content = open('src/cavlc_tables.h').read()

def parse_table(name, rows, cols):
    start = content.index(f'static const vlc_t {name}[{rows}][{cols}] = ' + '{')
    end = content.index('};', start)
    block = content[start:end]
    entries = re.findall(r'\{\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*(\d+)\s*\}', block)
    parsed = []
    for code_s, len_s in entries:
        code = int(code_s, 16) if code_s.startswith('0x') else int(code_s)
        length = int(len_s)
        parsed.append((code, length) if length > 0 else None)
    return [parsed[i*cols:(i+1)*cols] for i in range(rows)]

def code_to_bits(code, length):
    return format(code, f'0{length}b')

def check_prefix_free(name, table):
    print(f"\nChecking {name}:")
    for tc, row in enumerate(table):
        bits_list = []
        for i, e in enumerate(row):
            if e is None: continue
            bs = code_to_bits(e[0], e[1])
            bits_list.append((i, bs))
        # Check no entry is prefix of another
        for i, (idx1, bs1) in enumerate(bits_list):
            for idx2, bs2 in bits_list[i+1:]:
                if bs1 == bs2:
                    print(f"  CONFLICT TC={tc} idx1={idx1} idx2={idx2}: same code {bs1}")
                elif bs1.startswith(bs2):
                    print(f"  PREFIX TC={tc} idx2={idx2}({bs2}) is prefix of idx1={idx1}({bs1})")
                elif bs2.startswith(bs1):
                    print(f"  PREFIX TC={tc} idx1={idx1}({bs1}) is prefix of idx2={idx2}({bs2})")

check_prefix_free('coeff_token_nc01', parse_table('coeff_token_nc01', 17, 4))
check_prefix_free('coeff_token_nc23', parse_table('coeff_token_nc23', 17, 4))
check_prefix_free('coeff_token_nc47', parse_table('coeff_token_nc47', 17, 4))
check_prefix_free('coeff_token_chroma_dc', parse_table('coeff_token_chroma_dc', 5, 4))

# Check coeff_token across ALL TCs for a single sub-table, since decoder
# scans across all TCs in the same sub-table.
print("\nNow checking coeff_token sub-tables across all TC rows:")
for name in ['coeff_token_nc01', 'coeff_token_nc23', 'coeff_token_nc47', 'coeff_token_chroma_dc']:
    rows = 5 if 'chroma' in name else 17
    table = parse_table(name, rows, 4)
    all_codes = []
    for tc, row in enumerate(table):
        for t1, e in enumerate(row):
            if e is None: continue
            bs = code_to_bits(e[0], e[1])
            all_codes.append((tc, t1, bs))
    print(f"\n  {name}:")
    for i, (tc1, t11, bs1) in enumerate(all_codes):
        for tc2, t12, bs2 in all_codes[i+1:]:
            if bs1 == bs2:
                print(f"    CONFLICT (TC={tc1},T={t11}) and (TC={tc2},T={t12}): same code {bs1}")
            elif bs1.startswith(bs2):
                print(f"    PREFIX (TC={tc2},T={t12}) [{bs2}] prefix of (TC={tc1},T={t11}) [{bs1}]")
            elif bs2.startswith(bs1):
                print(f"    PREFIX (TC={tc1},T={t11}) [{bs1}] prefix of (TC={tc2},T={t12}) [{bs2}]")

# Check total_zeros prefix within each TC row
print("\nChecking total_zeros_4x4 prefix-free within each TC:")
tz = parse_table('total_zeros_4x4', 15, 16)
for tc, row in enumerate(tz):
    bits_list = []
    for i, e in enumerate(row):
        if e is None: continue
        bs = code_to_bits(e[0], e[1])
        bits_list.append((i, bs))
    for i, (idx1, bs1) in enumerate(bits_list):
        for idx2, bs2 in bits_list[i+1:]:
            if bs1 == bs2:
                print(f"  CONFLICT TC={tc+1} tz1={idx1} tz2={idx2}: same code {bs1}")
            elif bs1.startswith(bs2):
                print(f"  PREFIX TC={tc+1} tz2={idx2}({bs2}) prefix of tz1={idx1}({bs1})")
            elif bs2.startswith(bs1):
                print(f"  PREFIX TC={tc+1} tz1={idx1}({bs1}) prefix of tz2={idx2}({bs2})")

# Check run_before per zL
print("\nChecking run_before_tab prefix-free within each zL:")
rb = parse_table('run_before_tab', 7, 16)
for zL_idx, row in enumerate(rb):
    bits_list = []
    for i, e in enumerate(row):
        if e is None: continue
        bs = code_to_bits(e[0], e[1])
        bits_list.append((i, bs))
    for i, (idx1, bs1) in enumerate(bits_list):
        for idx2, bs2 in bits_list[i+1:]:
            if bs1 == bs2:
                print(f"  CONFLICT zL_idx={zL_idx} run1={idx1} run2={idx2}: same code {bs1}")
            elif bs1.startswith(bs2):
                print(f"  PREFIX zL_idx={zL_idx} run2={idx2}({bs2}) prefix of run1={idx1}({bs1})")
            elif bs2.startswith(bs1):
                print(f"  PREFIX zL_idx={zL_idx} run1={idx1}({bs1}) prefix of run2={idx2}({bs2})")
