--------------------------------------------------------------------------------
-- coeff_token_encoder.vhd
--
-- First sub-block of the CAVLC engine. Given (nC, TotalCoeff, TrailingOnes)
-- it looks up the variable-length coeff_token code per spec Table 9-5 and
-- presents (code, length) on its output, asserted for one cycle when valid_o
-- is high. Pipelined: takes 1 cycle from valid_i to valid_o.
--
-- Sub-table selection (per spec):
--   nC in 0..1  → Table 9-5(a)  → COEFF_TOKEN_NC01
--   nC in 2..3  → Table 9-5(b)  → COEFF_TOKEN_NC23
--   nC in 4..7  → Table 9-5(c)  → COEFF_TOKEN_NC47
--   nC >= 8     → 6-bit FLC (no ROM, computed inline)
--   nC = 0x1F   → chroma DC      → COEFF_TOKEN_CHROMA_DC
--
-- Special case: TotalCoeff=0 gets a one-bit code, picked from the
-- TotalCoeff=0 row of the active sub-table. The C reference handles this
-- via vlc_t coeff0_token[5]; we fold it into the same ROM lookup by
-- reading row 0 of each table (cells [0][0] hold the TC=0 code).
--
-- This module is purely combinational with one register stage on the output;
-- it does NOT depend on any external state beyond its inputs. It can be
-- instantiated once and shared across all CAVLC dispatch sites.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;
use work.cavlc_tables.all;

entity coeff_token_encoder is
    port (
        clk         : in  std_logic;
        rst_n       : in  std_logic;

        -- Request side
        valid_i     : in  std_logic;
        nC_i        : in  unsigned(4 downto 0);   -- 0..16 normal, 0x1F = chroma DC
        total_coef_i: in  unsigned(4 downto 0);   -- 0..16
        t1_i        : in  unsigned(1 downto 0);   -- TrailingOnes 0..3 (capped at 3)

        -- Output side: code + length valid for one cycle when valid_o = '1'
        valid_o     : out std_logic;
        code_o      : out unsigned(15 downto 0);
        length_o    : out unsigned(4 downto 0)
    );
end entity;

architecture rtl of coeff_token_encoder is

    signal code_q   : unsigned(15 downto 0);
    signal length_q : unsigned(4 downto 0);
    signal valid_q  : std_logic;

begin

    process(clk)
        variable tab_sel : integer range 0 to 4;
        variable tc      : integer range 0 to 16;
        variable t1      : integer range 0 to 3;
        variable e       : vlc_entry_t;
        variable nc_int  : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                valid_q  <= '0';
                code_q   <= (others => '0');
                length_q <= (others => '0');
            else
                valid_q <= valid_i;

                if valid_i = '1' then
                    tab_sel := nC_to_table_sel(nC_i);
                    tc      := to_integer(total_coef_i);
                    t1      := to_integer(t1_i);
                    nc_int  := to_integer(nC_i);

                    case tab_sel is
                        when NC_TABLE_01 =>
                            e := COEFF_TOKEN_NC01(tc, t1);
                        when NC_TABLE_23 =>
                            e := COEFF_TOKEN_NC23(tc, t1);
                        when NC_TABLE_47 =>
                            e := COEFF_TOKEN_NC47(tc, t1);
                        when NC_TABLE_CHROMA =>
                            -- chroma DC has only 5 rows (TotalCoeff 0..4).
                            -- The HLS encoder guarantees TC<=4 for chroma_dc
                            -- so we don't bound-check.
                            e := COEFF_TOKEN_CHROMA_DC(tc, t1);
                        when NC_TABLE_8PLUS =>
                            -- FLC (Fixed-Length Code) per spec 9.2.1.1:
                            --   coeff_token = (TotalCoeff << 2) | TrailingOnes,
                            --   transmitted as a 6-bit code with a +1 nudge:
                            --   actual_code = (TC << 2) | T1; if TC == 0 then
                            --   code = 0b000011 (special), else
                            --   code = ((TC-1) << 2) | T1, transmitted as 6 bits.
                            -- See ITU-T Rec. H.264 §9.2.1.1, Table 9-5(d).
                            if tc = 0 then
                                e.code   := x"0003";
                                e.length := "00110";
                            else
                                e.code   := resize(((to_unsigned(tc, 5) - 1)
                                                     & to_unsigned(t1, 2)),
                                                   16);
                                e.length := "00110";
                            end if;
                        when others =>
                            e.code   := (others => '0');
                            e.length := (others => '0');
                    end case;

                    code_q   <= e.code;
                    length_q <= e.length;
                end if;
            end if;
        end if;
    end process;

    valid_o  <= valid_q;
    code_o   <= code_q;
    length_o <= length_q;

end architecture;
