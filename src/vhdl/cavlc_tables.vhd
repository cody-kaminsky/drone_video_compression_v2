--------------------------------------------------------------------------------
-- cavlc_tables.vhd
--
-- AUTO-GENERATED from src/cavlc_tables.h by
-- tools/gen_cavlc_tables_vhd.py — do not hand-edit.
--
-- Source ITU-T Rec. H.264 (05/2003) Tables 9-5 (coeff_token), 9-7 (total_zeros),
-- 9-9 (run_before), 9-10 (run_before_init by zerosLeft), and chroma DC variants.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

package cavlc_tables is

    -- Coeff_token Table 9-5(a) — nC in 0..1
    -- 17 rows x 4 cols.
    type COEFF_TOKEN_NC01_t is array (0 to 16, 0 to 3) of vlc_entry_t;
    constant COEFF_TOKEN_NC01 : COEFF_TOKEN_NC01_t := (
        0 => ( (code => x"0001", length => "00001"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        1 => ( (code => x"0005", length => "00110"), (code => x"0001", length => "00010"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        2 => ( (code => x"0007", length => "01000"), (code => x"0004", length => "00110"), (code => x"0001", length => "00011"), (code => x"0000", length => "00000") ),
        3 => ( (code => x"0007", length => "01001"), (code => x"0006", length => "01000"), (code => x"0005", length => "00111"), (code => x"0003", length => "00101") ),
        4 => ( (code => x"0007", length => "01010"), (code => x"0006", length => "01001"), (code => x"0005", length => "01000"), (code => x"0003", length => "00110") ),
        5 => ( (code => x"0007", length => "01011"), (code => x"0006", length => "01010"), (code => x"0005", length => "01001"), (code => x"0004", length => "00111") ),
        6 => ( (code => x"000F", length => "01101"), (code => x"0006", length => "01011"), (code => x"0005", length => "01010"), (code => x"0004", length => "01000") ),
        7 => ( (code => x"000B", length => "01101"), (code => x"000E", length => "01101"), (code => x"0005", length => "01011"), (code => x"0004", length => "01001") ),
        8 => ( (code => x"0008", length => "01101"), (code => x"000A", length => "01101"), (code => x"000D", length => "01101"), (code => x"0004", length => "01010") ),
        9 => ( (code => x"000F", length => "01110"), (code => x"000E", length => "01110"), (code => x"0009", length => "01101"), (code => x"0004", length => "01011") ),
        10 => ( (code => x"000B", length => "01110"), (code => x"000A", length => "01110"), (code => x"000D", length => "01110"), (code => x"000C", length => "01101") ),
        11 => ( (code => x"000F", length => "01111"), (code => x"000E", length => "01111"), (code => x"0009", length => "01110"), (code => x"000C", length => "01110") ),
        12 => ( (code => x"000B", length => "01111"), (code => x"000A", length => "01111"), (code => x"000D", length => "01111"), (code => x"0008", length => "01110") ),
        13 => ( (code => x"000F", length => "10000"), (code => x"0001", length => "01111"), (code => x"0009", length => "01111"), (code => x"000C", length => "01111") ),
        14 => ( (code => x"000B", length => "10000"), (code => x"000E", length => "10000"), (code => x"000D", length => "10000"), (code => x"0008", length => "01111") ),
        15 => ( (code => x"0007", length => "10000"), (code => x"000A", length => "10000"), (code => x"0009", length => "10000"), (code => x"000C", length => "10000") ),
        16 => ( (code => x"0004", length => "10000"), (code => x"0006", length => "10000"), (code => x"0005", length => "10000"), (code => x"0008", length => "10000") )
    );

    -- Coeff_token Table 9-5(b) — nC in 2..3
    -- 17 rows x 4 cols.
    type COEFF_TOKEN_NC23_t is array (0 to 16, 0 to 3) of vlc_entry_t;
    constant COEFF_TOKEN_NC23 : COEFF_TOKEN_NC23_t := (
        0 => ( (code => x"0003", length => "00010"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        1 => ( (code => x"000B", length => "00110"), (code => x"0002", length => "00010"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        2 => ( (code => x"0007", length => "00110"), (code => x"0007", length => "00101"), (code => x"0003", length => "00011"), (code => x"0000", length => "00000") ),
        3 => ( (code => x"0007", length => "00111"), (code => x"000A", length => "00110"), (code => x"0009", length => "00110"), (code => x"0005", length => "00100") ),
        4 => ( (code => x"0007", length => "01000"), (code => x"0006", length => "00110"), (code => x"0005", length => "00110"), (code => x"0004", length => "00100") ),
        5 => ( (code => x"0004", length => "01000"), (code => x"0006", length => "00111"), (code => x"0005", length => "00111"), (code => x"0006", length => "00101") ),
        6 => ( (code => x"0007", length => "01001"), (code => x"0006", length => "01000"), (code => x"0005", length => "01000"), (code => x"0008", length => "00110") ),
        7 => ( (code => x"000F", length => "01011"), (code => x"0006", length => "01001"), (code => x"0005", length => "01001"), (code => x"0004", length => "00110") ),
        8 => ( (code => x"000B", length => "01011"), (code => x"000E", length => "01011"), (code => x"000D", length => "01011"), (code => x"0004", length => "00111") ),
        9 => ( (code => x"000F", length => "01100"), (code => x"000A", length => "01011"), (code => x"0009", length => "01011"), (code => x"0004", length => "01001") ),
        10 => ( (code => x"000B", length => "01100"), (code => x"000E", length => "01100"), (code => x"000D", length => "01100"), (code => x"000C", length => "01011") ),
        11 => ( (code => x"0008", length => "01100"), (code => x"000A", length => "01100"), (code => x"0009", length => "01100"), (code => x"0008", length => "01011") ),
        12 => ( (code => x"000F", length => "01101"), (code => x"000E", length => "01101"), (code => x"000D", length => "01101"), (code => x"000C", length => "01100") ),
        13 => ( (code => x"000B", length => "01101"), (code => x"000A", length => "01101"), (code => x"0009", length => "01101"), (code => x"000C", length => "01101") ),
        14 => ( (code => x"0007", length => "01101"), (code => x"000B", length => "01110"), (code => x"0006", length => "01101"), (code => x"0008", length => "01101") ),
        15 => ( (code => x"0009", length => "01110"), (code => x"0008", length => "01110"), (code => x"000A", length => "01110"), (code => x"0001", length => "01101") ),
        16 => ( (code => x"0007", length => "01110"), (code => x"0006", length => "01110"), (code => x"0005", length => "01110"), (code => x"0004", length => "01110") )
    );

    -- Coeff_token Table 9-5(c) — nC in 4..7
    -- 17 rows x 4 cols.
    type COEFF_TOKEN_NC47_t is array (0 to 16, 0 to 3) of vlc_entry_t;
    constant COEFF_TOKEN_NC47 : COEFF_TOKEN_NC47_t := (
        0 => ( (code => x"000F", length => "00100"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        1 => ( (code => x"000F", length => "00110"), (code => x"000E", length => "00100"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        2 => ( (code => x"000B", length => "00110"), (code => x"000F", length => "00101"), (code => x"000D", length => "00100"), (code => x"0000", length => "00000") ),
        3 => ( (code => x"0008", length => "00110"), (code => x"000C", length => "00101"), (code => x"000E", length => "00101"), (code => x"000C", length => "00100") ),
        4 => ( (code => x"000F", length => "00111"), (code => x"000A", length => "00101"), (code => x"000B", length => "00101"), (code => x"000B", length => "00100") ),
        5 => ( (code => x"000B", length => "00111"), (code => x"0008", length => "00101"), (code => x"0009", length => "00101"), (code => x"000A", length => "00100") ),
        6 => ( (code => x"0009", length => "00111"), (code => x"000E", length => "00110"), (code => x"000D", length => "00110"), (code => x"0009", length => "00100") ),
        7 => ( (code => x"0008", length => "00111"), (code => x"000A", length => "00110"), (code => x"0009", length => "00110"), (code => x"0008", length => "00100") ),
        8 => ( (code => x"000F", length => "01000"), (code => x"000E", length => "00111"), (code => x"000D", length => "00111"), (code => x"000D", length => "00101") ),
        9 => ( (code => x"000B", length => "01000"), (code => x"000E", length => "01000"), (code => x"000A", length => "00111"), (code => x"000C", length => "00110") ),
        10 => ( (code => x"000F", length => "01001"), (code => x"000A", length => "01000"), (code => x"000D", length => "01000"), (code => x"000C", length => "00111") ),
        11 => ( (code => x"000B", length => "01001"), (code => x"000E", length => "01001"), (code => x"0009", length => "01000"), (code => x"000C", length => "01000") ),
        12 => ( (code => x"0008", length => "01001"), (code => x"000A", length => "01001"), (code => x"000D", length => "01001"), (code => x"0008", length => "01000") ),
        13 => ( (code => x"000D", length => "01010"), (code => x"0007", length => "01001"), (code => x"0009", length => "01001"), (code => x"000C", length => "01001") ),
        14 => ( (code => x"0009", length => "01010"), (code => x"000C", length => "01010"), (code => x"000B", length => "01010"), (code => x"000A", length => "01010") ),
        15 => ( (code => x"0005", length => "01010"), (code => x"0008", length => "01010"), (code => x"0007", length => "01010"), (code => x"0006", length => "01010") ),
        16 => ( (code => x"0001", length => "01010"), (code => x"0004", length => "01010"), (code => x"0003", length => "01010"), (code => x"0002", length => "01010") )
    );

    -- Coeff_token chroma_dc — nC = sentinel (0x1F)
    -- 5 rows x 4 cols.
    type COEFF_TOKEN_CHROMA_DC_t is array (0 to 4, 0 to 3) of vlc_entry_t;
    constant COEFF_TOKEN_CHROMA_DC : COEFF_TOKEN_CHROMA_DC_t := (
        0 => ( (code => x"0001", length => "00010"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        1 => ( (code => x"0007", length => "00110"), (code => x"0001", length => "00001"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        2 => ( (code => x"0004", length => "00110"), (code => x"0006", length => "00110"), (code => x"0001", length => "00011"), (code => x"0000", length => "00000") ),
        3 => ( (code => x"0003", length => "00110"), (code => x"0003", length => "00111"), (code => x"0002", length => "00111"), (code => x"0005", length => "00110") ),
        4 => ( (code => x"0002", length => "00110"), (code => x"0003", length => "01000"), (code => x"0002", length => "01000"), (code => x"0000", length => "00111") )
    );

    -- Total_zeros for 4x4 luma blocks. Indexed [TotalCoeff-1][total_zeros].
    -- 15 rows x 16 cols.
    type TOTAL_ZEROS_4x4_t is array (0 to 14, 0 to 15) of vlc_entry_t;
    constant TOTAL_ZEROS_4x4 : TOTAL_ZEROS_4x4_t := (
        0 => ( (code => x"0001", length => "00001"), (code => x"0003", length => "00011"), (code => x"0002", length => "00011"), (code => x"0003", length => "00100"), (code => x"0002", length => "00100"), (code => x"0003", length => "00101"), (code => x"0002", length => "00101"), (code => x"0003", length => "00110"), (code => x"0002", length => "00110"), (code => x"0003", length => "00111"), (code => x"0002", length => "00111"), (code => x"0003", length => "01000"), (code => x"0002", length => "01000"), (code => x"0003", length => "01001"), (code => x"0002", length => "01001"), (code => x"0001", length => "01001") ),
        1 => ( (code => x"0007", length => "00011"), (code => x"0006", length => "00011"), (code => x"0005", length => "00011"), (code => x"0004", length => "00011"), (code => x"0003", length => "00011"), (code => x"0005", length => "00100"), (code => x"0004", length => "00100"), (code => x"0003", length => "00100"), (code => x"0002", length => "00100"), (code => x"0003", length => "00101"), (code => x"0002", length => "00101"), (code => x"0003", length => "00110"), (code => x"0002", length => "00110"), (code => x"0001", length => "00110"), (code => x"0000", length => "00110"), (code => x"0000", length => "00000") ),
        2 => ( (code => x"0005", length => "00100"), (code => x"0007", length => "00011"), (code => x"0006", length => "00011"), (code => x"0005", length => "00011"), (code => x"0004", length => "00100"), (code => x"0003", length => "00100"), (code => x"0004", length => "00011"), (code => x"0003", length => "00011"), (code => x"0002", length => "00100"), (code => x"0003", length => "00101"), (code => x"0002", length => "00101"), (code => x"0001", length => "00110"), (code => x"0001", length => "00101"), (code => x"0000", length => "00110"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        3 => ( (code => x"0003", length => "00101"), (code => x"0007", length => "00011"), (code => x"0005", length => "00100"), (code => x"0004", length => "00100"), (code => x"0006", length => "00011"), (code => x"0005", length => "00011"), (code => x"0004", length => "00011"), (code => x"0003", length => "00100"), (code => x"0003", length => "00011"), (code => x"0002", length => "00100"), (code => x"0002", length => "00101"), (code => x"0001", length => "00101"), (code => x"0000", length => "00101"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        4 => ( (code => x"0005", length => "00100"), (code => x"0004", length => "00100"), (code => x"0003", length => "00100"), (code => x"0007", length => "00011"), (code => x"0006", length => "00011"), (code => x"0005", length => "00011"), (code => x"0004", length => "00011"), (code => x"0003", length => "00011"), (code => x"0002", length => "00100"), (code => x"0001", length => "00101"), (code => x"0001", length => "00100"), (code => x"0000", length => "00101"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        5 => ( (code => x"0001", length => "00110"), (code => x"0001", length => "00101"), (code => x"0007", length => "00011"), (code => x"0006", length => "00011"), (code => x"0005", length => "00011"), (code => x"0004", length => "00011"), (code => x"0003", length => "00011"), (code => x"0002", length => "00011"), (code => x"0001", length => "00100"), (code => x"0001", length => "00011"), (code => x"0000", length => "00110"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        6 => ( (code => x"0001", length => "00110"), (code => x"0001", length => "00101"), (code => x"0005", length => "00011"), (code => x"0004", length => "00011"), (code => x"0003", length => "00011"), (code => x"0003", length => "00010"), (code => x"0002", length => "00011"), (code => x"0001", length => "00100"), (code => x"0001", length => "00011"), (code => x"0000", length => "00110"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        7 => ( (code => x"0001", length => "00110"), (code => x"0001", length => "00100"), (code => x"0001", length => "00101"), (code => x"0003", length => "00011"), (code => x"0003", length => "00010"), (code => x"0002", length => "00010"), (code => x"0002", length => "00011"), (code => x"0001", length => "00011"), (code => x"0000", length => "00110"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        8 => ( (code => x"0001", length => "00110"), (code => x"0000", length => "00110"), (code => x"0001", length => "00100"), (code => x"0003", length => "00010"), (code => x"0002", length => "00010"), (code => x"0001", length => "00011"), (code => x"0001", length => "00010"), (code => x"0001", length => "00101"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        9 => ( (code => x"0001", length => "00101"), (code => x"0000", length => "00101"), (code => x"0001", length => "00011"), (code => x"0003", length => "00010"), (code => x"0002", length => "00010"), (code => x"0001", length => "00010"), (code => x"0001", length => "00100"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        10 => ( (code => x"0000", length => "00100"), (code => x"0001", length => "00100"), (code => x"0001", length => "00011"), (code => x"0002", length => "00011"), (code => x"0001", length => "00001"), (code => x"0003", length => "00011"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        11 => ( (code => x"0000", length => "00100"), (code => x"0001", length => "00100"), (code => x"0001", length => "00010"), (code => x"0001", length => "00001"), (code => x"0001", length => "00011"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        12 => ( (code => x"0000", length => "00011"), (code => x"0001", length => "00011"), (code => x"0001", length => "00001"), (code => x"0001", length => "00010"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        13 => ( (code => x"0000", length => "00010"), (code => x"0001", length => "00010"), (code => x"0001", length => "00001"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        14 => ( (code => x"0000", length => "00001"), (code => x"0001", length => "00001"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") )
    );

    -- Total_zeros for chroma DC. Indexed [TotalCoeff-1][total_zeros].
    -- 3 rows x 4 cols.
    type TOTAL_ZEROS_CHROMA_DC_t is array (0 to 2, 0 to 3) of vlc_entry_t;
    constant TOTAL_ZEROS_CHROMA_DC : TOTAL_ZEROS_CHROMA_DC_t := (
        0 => ( (code => x"0001", length => "00001"), (code => x"0001", length => "00010"), (code => x"0001", length => "00011"), (code => x"0000", length => "00011") ),
        1 => ( (code => x"0001", length => "00001"), (code => x"0001", length => "00010"), (code => x"0000", length => "00010"), (code => x"0000", length => "00000") ),
        2 => ( (code => x"0001", length => "00001"), (code => x"0000", length => "00001"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") )
    );

    -- Run_before. Indexed [zeros_left_idx][run].
    -- 7 rows x 16 cols.
    type RUN_BEFORE_TAB_t is array (0 to 6, 0 to 15) of vlc_entry_t;
    constant RUN_BEFORE_TAB : RUN_BEFORE_TAB_t := (
        0 => ( (code => x"0001", length => "00001"), (code => x"0000", length => "00001"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        1 => ( (code => x"0001", length => "00001"), (code => x"0001", length => "00010"), (code => x"0000", length => "00010"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        2 => ( (code => x"0003", length => "00010"), (code => x"0002", length => "00010"), (code => x"0001", length => "00010"), (code => x"0000", length => "00010"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        3 => ( (code => x"0003", length => "00010"), (code => x"0002", length => "00010"), (code => x"0001", length => "00010"), (code => x"0001", length => "00011"), (code => x"0000", length => "00011"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        4 => ( (code => x"0003", length => "00010"), (code => x"0002", length => "00010"), (code => x"0003", length => "00011"), (code => x"0002", length => "00011"), (code => x"0001", length => "00011"), (code => x"0000", length => "00011"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        5 => ( (code => x"0003", length => "00010"), (code => x"0000", length => "00011"), (code => x"0001", length => "00011"), (code => x"0003", length => "00011"), (code => x"0002", length => "00011"), (code => x"0005", length => "00011"), (code => x"0004", length => "00011"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000"), (code => x"0000", length => "00000") ),
        6 => ( (code => x"0007", length => "00011"), (code => x"0006", length => "00011"), (code => x"0005", length => "00011"), (code => x"0004", length => "00011"), (code => x"0003", length => "00011"), (code => x"0002", length => "00011"), (code => x"0001", length => "00011"), (code => x"0001", length => "00100"), (code => x"0001", length => "00101"), (code => x"0001", length => "00110"), (code => x"0001", length => "00111"), (code => x"0001", length => "01000"), (code => x"0001", length => "01001"), (code => x"0001", length => "01010"), (code => x"0001", length => "01011"), (code => x"0000", length => "00000") )
    );

end package;
