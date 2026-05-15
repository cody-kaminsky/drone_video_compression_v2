--------------------------------------------------------------------------------
-- cavlc_pkg.vhd
--
-- Common types and constants for the CAVLC engine. See
-- docs/m4-cavlc-vhdl.md for the design overview.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package cavlc_pkg is

    --------------------------------------------------------------------
    -- Block-type encoding (must match BLK_* enum in src/cavlc.h).
    --
    -- LUMA_DC_16x16  : 16-coef block, I_16x16 luma DC.
    -- LUMA_AC        : 15-coef block, I_16x16 luma AC.
    -- LUMA_FULL      : 16-coef block, I_4x4 luma (DC and AC together).
    -- CHROMA_DC      : 4-coef block, chroma DC. Uses chroma_dc table.
    -- CHROMA_AC      : 15-coef block, chroma AC.
    --------------------------------------------------------------------
    constant BLK_LUMA_DC_16x16 : integer := 0;
    constant BLK_LUMA_AC       : integer := 1;
    constant BLK_LUMA_FULL     : integer := 2;
    constant BLK_CHROMA_DC     : integer := 3;
    constant BLK_CHROMA_AC     : integer := 4;

    subtype block_type_t is unsigned(2 downto 0);

    --------------------------------------------------------------------
    -- Coefficient level. Spec range is +/-2048 for QP up to 51 baseline;
    -- 12 bits would suffice but we use 16 for safety / NV12 alignment.
    --------------------------------------------------------------------
    subtype level_t is signed(15 downto 0);
    type    level_array_t is array (0 to 15) of level_t;

    --------------------------------------------------------------------
    -- Level packet — one residual block crossing the HLS-to-VHDL
    -- boundary. The HLS encoder produces these via hls::stream<>; Vitis
    -- synthesizes the stream as AXI4-Stream which lands here on the
    -- VHDL side as one valid+ready handshake per packet.
    --------------------------------------------------------------------
    type level_packet_t is record
        block_type : block_type_t;          -- BLK_* constant
        n_coefs    : unsigned(4 downto 0);  -- 4, 15, or 16
        nC         : unsigned(4 downto 0);  -- 0..16; 31 (=0x1F) for chroma DC sub-table
        levels     : level_array_t;          -- zigzag-ordered coefficients
    end record;

    --------------------------------------------------------------------
    -- VLC (variable-length code) table entry. code is right-aligned in
    -- a 16-bit field; length tells how many low bits to emit MSB-first.
    -- A length of 0 means "no entry" (reserved cell in a 17×4 table).
    --------------------------------------------------------------------
    type vlc_entry_t is record
        code   : unsigned(15 downto 0);
        length : unsigned(4 downto 0);
    end record;

    --------------------------------------------------------------------
    -- nC sub-table selector for coeff_token. Mirrors the C reference's
    -- five sub-tables (Table 9-5 a-d + chroma_dc).
    --------------------------------------------------------------------
    constant NC_TABLE_01      : integer := 0;   -- nC in 0..1
    constant NC_TABLE_23      : integer := 1;   -- nC in 2..3
    constant NC_TABLE_47      : integer := 2;   -- nC in 4..7
    constant NC_TABLE_8PLUS   : integer := 3;   -- nC >= 8 (FLC)
    constant NC_TABLE_CHROMA  : integer := 4;   -- chroma DC

    function nC_to_table_sel(nC : unsigned) return integer;

    --------------------------------------------------------------------
    -- Bit packer ports. The bit packer is a separate sub-module; this
    -- record bundles the producer-side signals so submodules can drive
    -- it cleanly.
    --
    -- Width: 32-bit bits / 6-bit length (range 0..32). Wide enough for
    -- the longest single CAVLC field (28-bit escape level: 15 zeros +
    -- '1' + 12-bit suffix per spec 9.2.2). One handshake = one logical
    -- field; sub-modules don't have to split escape levels into multiple
    -- packer beats.
    --------------------------------------------------------------------
    type bp_in_t is record
        bits   : unsigned(31 downto 0);
        length : unsigned(5 downto 0);
        valid  : std_logic;
    end record;

    constant BP_IN_IDLE : bp_in_t := (
        bits   => (others => '0'),
        length => (others => '0'),
        valid  => '0'
    );

end package;

package body cavlc_pkg is

    function nC_to_table_sel(nC : unsigned) return integer is
        variable n : integer;
    begin
        n := to_integer(nC);
        if n = 31 then
            return NC_TABLE_CHROMA;     -- 0x1F sentinel = chroma DC
        elsif n < 2 then
            return NC_TABLE_01;
        elsif n < 4 then
            return NC_TABLE_23;
        elsif n < 8 then
            return NC_TABLE_47;
        else
            return NC_TABLE_8PLUS;
        end if;
    end function;

end package body;
