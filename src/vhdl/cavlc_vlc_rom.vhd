--------------------------------------------------------------------------------
-- cavlc_vlc_rom.vhd
--
-- Single-port synchronous ROM holding the coeff_token (Table 9-5 a/b/c and
-- chroma DC) and total_zeros (Tables 9-7 / 9-9) VLC entries, so that they
-- map to one block RAM instead of ~200 LUTs per engine. The engine muxes
-- the address: coeff_token while waiting for that code, total_zeros
-- otherwise (the two lookups are never needed in the same cycle). A
-- single read port is used deliberately: Vivado infers two reads of a
-- constant array as two ROMs and only one of them reaches block RAM.
--
-- Contents are built at elaboration from the cavlc_tables constants, so
-- this file needs no regeneration when the tables change.
--
-- Address map (9 bits, 512 entries so the 21-bit word fits one RAMB18 in
-- 512x36 mode; a 1024-deep map would only get 18 bits into the BRAM):
--   coeff_token :   0 + tc*4 + t1   nC 0..1   (68 entries)
--                  68 + tc*4 + t1   nC 2..3
--                 136 + tc*4 + t1   nC 4..7
--                 204 + tc*4 + t1   chroma DC (tc 0..4, 20 entries)
--   total_zeros : 256 + (tc-1)*16 + tz   luma 4x4  (240 entries)
--                 496 + (tc-1)*4  + tz   chroma DC (12 entries)
--
-- Data (21 bits): length(20 downto 16) & code(15 downto 0).
-- One cycle of latency; the output holds while the address holds.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;
use work.cavlc_tables.all;

entity cavlc_vlc_rom is
    port (
        clk      : in  std_logic;
        addr     : in  unsigned(8 downto 0);
        data     : out vlc_entry_t
    );
end entity;

architecture rtl of cavlc_vlc_rom is

    subtype word_t is std_logic_vector(20 downto 0);
    type rom_t is array (0 to 511) of word_t;

    function pack(e : vlc_entry_t) return word_t is
    begin
        return std_logic_vector(e.length) & std_logic_vector(e.code);
    end function;

    function build_rom return rom_t is
        variable r : rom_t := (others => (others => '0'));
        variable e : vlc_entry_t;
    begin
        -- coeff_token: 3 tables x 17 rows x 4 cols, then chroma DC 5 x 4
        for tab in 0 to 2 loop
            for tc in 0 to 16 loop
                for t1 in 0 to 3 loop
                    case tab is
                        when 0      => e := COEFF_TOKEN_NC01(tc, t1);
                        when 1      => e := COEFF_TOKEN_NC23(tc, t1);
                        when others => e := COEFF_TOKEN_NC47(tc, t1);
                    end case;
                    r(tab * 68 + tc * 4 + t1) := pack(e);
                end loop;
            end loop;
        end loop;
        for tc in 0 to 4 loop
            for t1 in 0 to 3 loop
                r(204 + tc * 4 + t1) := pack(COEFF_TOKEN_CHROMA_DC(tc, t1));
            end loop;
        end loop;
        -- total_zeros: luma 15 x 16, chroma DC 3 x 4
        for tcm1 in 0 to 14 loop
            for tz in 0 to 15 loop
                r(256 + tcm1 * 16 + tz) := pack(TOTAL_ZEROS_4x4(tcm1, tz));
            end loop;
        end loop;
        for tcm1 in 0 to 2 loop
            for tz in 0 to 3 loop
                r(496 + tcm1 * 4 + tz) := pack(TOTAL_ZEROS_CHROMA_DC(tcm1, tz));
            end loop;
        end loop;
        return r;
    end function;

    signal rom : rom_t := build_rom;
    attribute rom_style : string;
    attribute rom_style of rom : signal is "block";

    signal q : word_t;

begin

    process(clk)
    begin
        if rising_edge(clk) then
            q <= rom(to_integer(addr));
        end if;
    end process;

    data.code   <= unsigned(q(15 downto 0));
    data.length <= unsigned(q(20 downto 16));

end architecture;
