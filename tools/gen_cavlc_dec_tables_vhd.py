#!/usr/bin/env python3
"""Build src/vhdl/cavlc_dec_tables.vhd from src/cavlc_tables.h.

The decoder needs the inverse of the encoder's tables: bits in, symbol out.
Rather than hand-write a second set of code words that could drift from the
first, this inverts the same C header the encoder's VHDL tables come from.
One source of truth, two directions.

    python tools/gen_cavlc_dec_tables_vhd.py

Why the tables have this shape. Every CAVLC table resolves as a run of
leading zeros followed by a short tail; measured across all of them the tail
is at most 4 bits, even where the whole code reaches 16. So the decode key is
(leading zeros, next 4 bits) and each variant is 16 x 16 entries, rather than
the 65536 a direct 16-bit index would need. Decoding is then a priority
encoder feeding a small ROM, one symbol per cycle.

An entry carries the symbol and the TOTAL code length so the consumer retires
exactly what matched. A length of 0 means no code matches: a malformed
stream, reported rather than silently decoded as symbol 0.
"""

import re
import sys
from pathlib import Path

SRC = Path("src/cavlc_tables.h")
DST = Path("src/vhdl/cavlc_dec_tables.vhd")

KEY_BITS = 4
MAX_LZ = 15


def parse_table(text, name, rows, cols):
    pattern = re.compile(
        rf"static const vlc_t {re.escape(name)}\[{rows}\]\[{cols}\]\s*=\s*\{{(.*?)\}};",
        re.DOTALL,
    )
    m = pattern.search(text)
    if not m:
        raise SystemExit("could not find table %s in %s" % (name, SRC))
    entries = re.findall(r"\{\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*(\d+)\s*\}", m.group(1))
    if len(entries) != rows * cols:
        raise SystemExit("table %s: expected %d entries, got %d"
                         % (name, rows * cols, len(entries)))
    out = []
    for code_s, len_s in entries:
        code = int(code_s, 16) if code_s.startswith("0x") else int(code_s)
        out.append((code, int(len_s)))
    return [out[i * cols:(i + 1) * cols] for i in range(rows)]


def leading_zeros(code, length):
    lz = 0
    for b in range(length - 1, -1, -1):
        if (code >> b) & 1:
            break
        lz += 1
    return lz


def build(entries, label):
    """entries: list of (code, length, symbol). Returns [lz][key] -> (sym,len)."""
    tab = [[(0, 0)] * (1 << KEY_BITS) for _ in range(MAX_LZ + 1)]
    for code, length, sym in entries:
        if length == 0:
            continue
        lz = leading_zeros(code, length)
        tail_len = length - lz
        if lz > MAX_LZ:
            raise SystemExit("%s: code 0x%X len %d has %d leading zeros, over %d"
                             % (label, code, length, lz, MAX_LZ))
        if tail_len > KEY_BITS:
            raise SystemExit("%s: code 0x%X len %d needs %d bits after the zero "
                             "run, over KEY_BITS=%d" % (label, code, length,
                                                        tail_len, KEY_BITS))
        tail = code & ((1 << tail_len) - 1)
        # Every key whose top tail_len bits equal tail resolves to this symbol.
        span = 1 << (KEY_BITS - tail_len)
        base = tail * span
        for j in range(span):
            prev = tab[lz][base + j]
            if prev[1] != 0 and prev != (sym, length):
                raise SystemExit("%s: key (lz=%d,%d) claimed by two codes"
                                 % (label, lz, base + j))
            tab[lz][base + j] = (sym, length)
    return tab


def emit(name, tab, comment):
    lines = ["    -- %s" % comment,
             "    constant %s : dec_tab_t := (" % name]
    for lz in range(MAX_LZ + 1):
        cells = ", ".join("(%d,%d)" % (s, l) for s, l in tab[lz])
        comma = "," if lz < MAX_LZ else ""
        lines.append("        %-2d => ( %s )%s" % (lz, cells, comma))
    lines.append("    );")
    lines.append("")
    return "\n".join(lines)


def main():
    text = SRC.read_text()

    # coeff_token: [total_coeff 0..16][trailing_ones 0..3].
    # Symbol packs both: tc * 4 + t1, which is how the engine already keys it.
    ct_tables = []
    for cname, rows, label in (("coeff_token_nc01", 17, "nC 0..1"),
                               ("coeff_token_nc23", 17, "nC 2..3"),
                               ("coeff_token_nc47", 17, "nC 4..7"),
                               ("coeff_token_chroma_dc", 5, "chroma DC")):
        t = parse_table(text, cname, rows, 4)
        ents = [(t[tc][t1][0], t[tc][t1][1], tc * 4 + t1)
                for tc in range(rows) for t1 in range(4)]
        ct_tables.append((cname.upper(), build(ents, label), "coeff_token, " + label))

    # total_zeros 4x4: [tzVlcIndex 1..15][total_zeros 0..15]. Symbol = total_zeros.
    tz = parse_table(text, "total_zeros_4x4", 15, 16)
    tz_tabs = []
    for i in range(15):
        ents = [(tz[i][v][0], tz[i][v][1], v) for v in range(16)]
        tz_tabs.append(build(ents, "total_zeros tzVlcIndex %d" % (i + 1)))

    # total_zeros chroma DC: [tzVlcIndex 1..3][total_zeros 0..3].
    tzc = parse_table(text, "total_zeros_chroma_dc", 3, 4)
    tzc_tabs = []
    for i in range(3):
        ents = [(tzc[i][v][0], tzc[i][v][1], v) for v in range(4)]
        tzc_tabs.append(build(ents, "total_zeros chroma DC index %d" % (i + 1)))

    # run_before: [zerosLeft 1..7 (6 = >6)][run 0..15]. Symbol = run.
    rb = parse_table(text, "run_before_tab", 7, 16)
    rb_tabs = []
    for i in range(7):
        ents = [(rb[i][v][0], rb[i][v][1], v) for v in range(16)]
        rb_tabs.append(build(ents, "run_before zerosLeft %d" % (i + 1)))

    out = []
    out.append("""--------------------------------------------------------------------------------
-- cavlc_dec_tables.vhd — GENERATED by tools/gen_cavlc_dec_tables_vhd.py
-- from src/cavlc_tables.h. Do not edit; regenerate.
--
-- Decode-direction lookup: bits in, symbol out. Built by inverting the same C
-- header the encoder's VHDL tables come from, so the two directions cannot
-- drift apart.
--
-- Key is (leading zeros, next %d bits). Every CAVLC table resolves as a run of
-- leading zeros followed by a tail of at most %d bits, even where the whole
-- code reaches 16, so each variant is %d x %d entries rather than the 65536 a
-- direct 16-bit index would need.
--
-- An entry carries the symbol and the TOTAL code length, so a consumer
-- retires exactly what matched. len = 0 means no code matches: a malformed
-- stream, to be reported rather than decoded as symbol 0.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package cavlc_dec_tables is

    constant KEY_BITS : integer := %d;
    constant MAX_LZ   : integer := %d;

    type dec_entry_t is record
        sym : integer range 0 to 68;
        len : integer range 0 to 31;
    end record;

    type dec_tab_t is array (0 to MAX_LZ, 0 to 2**KEY_BITS - 1) of dec_entry_t;
    type dec_tab_arr_t is array (natural range <>) of dec_tab_t;
""" % (KEY_BITS, KEY_BITS, MAX_LZ + 1, 1 << KEY_BITS, KEY_BITS, MAX_LZ))

    for name, tab, comment in ct_tables:
        out.append(emit("DEC_" + name, tab, comment))

    out.append("    -- total_zeros, 4x4 blocks, indexed by tzVlcIndex - 1")
    out.append("    constant DEC_TOTAL_ZEROS_4x4 : dec_tab_arr_t(0 to 14) := (")
    for i, t in enumerate(tz_tabs):
        cells = []
        for lz in range(MAX_LZ + 1):
            cells.append("%d => ( %s )" % (lz, ", ".join("(%d,%d)" % (s, l) for s, l in t[lz])))
        out.append("        %-2d => ( %s )%s" % (i, ", ".join(cells), "," if i < 14 else ""))
    out.append("    );\n")

    out.append("    -- total_zeros, chroma DC, indexed by tzVlcIndex - 1")
    out.append("    constant DEC_TOTAL_ZEROS_CHROMA_DC : dec_tab_arr_t(0 to 2) := (")
    for i, t in enumerate(tzc_tabs):
        cells = []
        for lz in range(MAX_LZ + 1):
            cells.append("%d => ( %s )" % (lz, ", ".join("(%d,%d)" % (s, l) for s, l in t[lz])))
        out.append("        %-2d => ( %s )%s" % (i, ", ".join(cells), "," if i < 2 else ""))
    out.append("    );\n")

    out.append("    -- run_before, indexed by zerosLeft - 1 (index 6 covers zerosLeft > 6)")
    out.append("    constant DEC_RUN_BEFORE : dec_tab_arr_t(0 to 6) := (")
    for i, t in enumerate(rb_tabs):
        cells = []
        for lz in range(MAX_LZ + 1):
            cells.append("%d => ( %s )" % (lz, ", ".join("(%d,%d)" % (s, l) for s, l in t[lz])))
        out.append("        %-2d => ( %s )%s" % (i, ", ".join(cells), "," if i < 6 else ""))
    out.append("    );\n")

    out.append("end package;")
    DST.write_text("\n".join(out) + "\n")
    print("wrote %s" % DST)
    return 0


if __name__ == "__main__":
    sys.exit(main())
