#!/usr/bin/env python3
"""Translate src/cavlc_tables.h → src/vhdl/cavlc_tables.vhd.

The C tables are vlc_t entries (each = {code, length}). We emit them as
VHDL constant arrays of vlc_entry_t. Run from the repo root:

    python tools/gen_cavlc_tables_vhd.py

Idempotent — re-running should produce the same file. Re-run whenever
src/cavlc_tables.h changes.
"""
import re
from pathlib import Path

SRC = Path("src/cavlc_tables.h")
DST = Path("src/vhdl/cavlc_tables.vhd")


def parse_table(text, name, rows, cols):
    """Pull a rows×cols vlc_t table out of cavlc_tables.h."""
    pattern = re.compile(
        rf"static const vlc_t {re.escape(name)}\[{rows}\]\[{cols}\]\s*=\s*\{{(.*?)\}};",
        re.DOTALL,
    )
    m = pattern.search(text)
    if not m:
        raise SystemExit(f"could not find table {name} in {SRC}")
    body = m.group(1)
    entries = re.findall(r"\{\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*(\d+)\s*\}", body)
    if len(entries) != rows * cols:
        raise SystemExit(
            f"table {name}: expected {rows*cols} entries, got {len(entries)}"
        )
    parsed = []
    for code_s, len_s in entries:
        code = int(code_s, 16) if code_s.startswith("0x") else int(code_s)
        parsed.append((code, int(len_s)))
    return [parsed[i * cols : (i + 1) * cols] for i in range(rows)]


def emit_2d(name, table, comment):
    """Emit a 2-D constant array as VHDL."""
    rows = len(table)
    cols = len(table[0])
    lines = [
        f"    -- {comment}",
        f"    -- {rows} rows x {cols} cols.",
        f"    type {name}_t is array (0 to {rows - 1}, 0 to {cols - 1}) of vlc_entry_t;",
        f"    constant {name} : {name}_t := (",
    ]
    for r, row in enumerate(table):
        cells = ", ".join(
            f'(code => x"{c:04X}", length => "{l:05b}")' for c, l in row
        )
        comma = "," if r < rows - 1 else ""
        lines.append(f"        {r} => ( {cells} ){comma}")
    lines.append("    );")
    lines.append("")
    return "\n".join(lines)


def main():
    text = SRC.read_text()

    out = []
    out.append(
        "--------------------------------------------------------------------------------"
    )
    out.append("-- cavlc_tables.vhd")
    out.append("--")
    out.append("-- AUTO-GENERATED from src/cavlc_tables.h by")
    out.append("-- tools/gen_cavlc_tables_vhd.py — do not hand-edit.")
    out.append("--")
    out.append(
        "-- Source ITU-T Rec. H.264 (05/2003) Tables 9-5 (coeff_token), 9-7 (total_zeros),"
    )
    out.append(
        "-- 9-9 (run_before), 9-10 (run_before_init by zerosLeft), and chroma DC variants."
    )
    out.append(
        "--------------------------------------------------------------------------------"
    )
    out.append("library ieee;")
    out.append("use ieee.std_logic_1164.all;")
    out.append("use ieee.numeric_std.all;")
    out.append("")
    out.append("use work.cavlc_pkg.all;")
    out.append("")
    out.append("package cavlc_tables is")
    out.append("")

    # coeff_token sub-tables (5 of them, each 17 rows × 4 cols).
    # Row 0 corresponds to TotalCoeff=0 (special "no nonzero" code), rows
    # 1..16 to TotalCoeff=1..16. Cols are TrailingOnes 0..3.
    out.append(
        emit_2d(
            "COEFF_TOKEN_NC01",
            parse_table(text, "coeff_token_nc01", 17, 4),
            "Coeff_token Table 9-5(a) — nC in 0..1",
        )
    )
    out.append(
        emit_2d(
            "COEFF_TOKEN_NC23",
            parse_table(text, "coeff_token_nc23", 17, 4),
            "Coeff_token Table 9-5(b) — nC in 2..3",
        )
    )
    out.append(
        emit_2d(
            "COEFF_TOKEN_NC47",
            parse_table(text, "coeff_token_nc47", 17, 4),
            "Coeff_token Table 9-5(c) — nC in 4..7",
        )
    )
    out.append(
        emit_2d(
            "COEFF_TOKEN_CHROMA_DC",
            parse_table(text, "coeff_token_chroma_dc", 5, 4),
            "Coeff_token chroma_dc — nC = sentinel (0x1F)",
        )
    )

    # total_zeros (Table 9-7): 15 rows × 16 cols. Row r = TotalCoeff (1..15);
    # Col z = total_zeros value (0..15). Some cells are reserved (length=0).
    out.append(
        emit_2d(
            "TOTAL_ZEROS_4x4",
            parse_table(text, "total_zeros_4x4", 15, 16),
            "Total_zeros for 4x4 luma blocks. Indexed [TotalCoeff-1][total_zeros].",
        )
    )

    # total_zeros for chroma DC (smaller — Table 9-7 chroma column).
    out.append(
        emit_2d(
            "TOTAL_ZEROS_CHROMA_DC",
            parse_table(text, "total_zeros_chroma_dc", 3, 4),
            "Total_zeros for chroma DC. Indexed [TotalCoeff-1][total_zeros].",
        )
    )

    # run_before (Table 9-10): 7 rows × 16 cols. Row r = zeros_left index,
    # col c = run value.
    out.append(
        emit_2d(
            "RUN_BEFORE_TAB",
            parse_table(text, "run_before_tab", 7, 16),
            "Run_before. Indexed [zeros_left_idx][run].",
        )
    )

    out.append("end package;")
    out.append("")

    DST.write_text("\n".join(out))
    print(f"wrote {DST} ({DST.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
