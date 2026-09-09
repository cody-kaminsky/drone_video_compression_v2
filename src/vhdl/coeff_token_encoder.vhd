--------------------------------------------------------------------------------
-- coeff_token_encoder.vhd
--
-- First sub-block of the CAVLC engine. Given (nC, TotalCoeff, TrailingOnes)
-- it looks up the variable-length coeff_token code per spec Table 9-5 and
-- presents (code, length) on its output, asserted for one cycle when valid_o
-- is high. Pipelined: takes 1 cycle from valid_i to valid_o. code_o and
-- length_o hold their value after the pulse as long as the inputs hold.
--
-- Sub-table selection (per spec):
--   nC in 0..1  → Table 9-5(a)  → ROM tab 0
--   nC in 2..3  → Table 9-5(b)  → ROM tab 1
--   nC in 4..7  → Table 9-5(c)  → ROM tab 2
--   nC >= 8     → 6-bit FLC (no ROM, computed inline)
--   nC = 0x1F   → chroma DC      → ROM tab 3
--
-- The table itself lives in cavlc_vlc_rom (block RAM, shared with the
-- total_zeros lookup); this module only forms the address and muxes in the
-- FLC case. Special case: TotalCoeff=0 gets a one-bit code, held in row 0
-- of each sub-table.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

entity coeff_token_encoder is
    port (
        clk         : in  std_logic;
        rst_n       : in  std_logic;

        -- Request side
        valid_i     : in  std_logic;
        nC_i        : in  unsigned(4 downto 0);   -- 0..16 normal, 0x1F = chroma DC
        total_coef_i: in  unsigned(4 downto 0);   -- 0..16
        t1_i        : in  unsigned(1 downto 0);   -- TrailingOnes 0..3 (capped at 3)

        -- ROM port (cavlc_vlc_rom port A, 1-cycle latency)
        rom_addr_o  : out unsigned(8 downto 0);
        rom_data_i  : in  vlc_entry_t;

        -- Output side: code + length valid for one cycle when valid_o = '1'
        valid_o     : out std_logic;
        code_o      : out unsigned(15 downto 0);
        length_o    : out unsigned(4 downto 0)
    );
end entity;

architecture rtl of coeff_token_encoder is

    signal valid_q   : std_logic;
    signal use_flc_q : std_logic;
    signal flc_q     : unsigned(5 downto 0);
    signal tab_base  : unsigned(7 downto 0);

begin

    -- ROM sub-table base (68 entries per table, 17 rows x 4; chroma DC at
    -- 204). nC >= 8 uses the FLC path and the ROM address is don't-care.
    tab_base <= to_unsigned(204, 8) when nC_i = to_unsigned(31, 5) else
                to_unsigned(0, 8)   when nC_i < 2 else
                to_unsigned(68, 8)  when nC_i < 4 else
                to_unsigned(136, 8);

    rom_addr_o <= '0' & (tab_base + (total_coef_i & t1_i));

    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                valid_q   <= '0';
                use_flc_q <= '0';
                flc_q     <= (others => '0');
            else
                valid_q <= valid_i;

                -- FLC (Fixed-Length Code) per spec 9.2.1.1 for nC >= 8:
                --   TC = 0  -> 000011
                --   else    -> ((TC-1) << 2) | T1, 6 bits
                if nC_i >= 8 and nC_i /= to_unsigned(31, 5) then
                    use_flc_q <= '1';
                else
                    use_flc_q <= '0';
                end if;
                if total_coef_i = 0 then
                    flc_q <= "000011";
                else
                    flc_q <= resize(total_coef_i - 1, 4) & t1_i;
                end if;
            end if;
        end if;
    end process;

    valid_o  <= valid_q;
    code_o   <= resize(flc_q, 16) when use_flc_q = '1' else rom_data_i.code;
    length_o <= "00110"           when use_flc_q = '1' else rom_data_i.length;

end architecture;
