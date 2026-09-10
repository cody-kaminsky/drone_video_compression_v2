--------------------------------------------------------------------------------
-- mb_header_engine_tb.vhd — self-checking testbench for mb_header_engine.
-- Reads build/mb_header_vectors.txt (tools/gen_mb_header_vectors.c),
-- collects the emitted fields into a bit string (with a stalling field
-- sink) and compares it, the CBP and the bit count with the C reference.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity mb_header_engine_tb is
    generic (VEC_FILE : string := "build/mb_header_vectors.txt");
end entity;

architecture sim of mb_header_engine_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal start_i, ready_o : std_logic := '0';
    signal is_i4x4_i : std_logic := '0';
    signal mode16_i, mode_chroma_i : unsigned(1 downto 0) := (others => '0');
    signal modes4_i : std_logic_vector(63 downto 0) := (others => '0');
    signal luma_nz_i, mode4_top_i, mode4_left_i : std_logic_vector(15 downto 0) := (others => '0');
    signal chroma_dc_nz_i, chroma_ac_nz_i, avail_top_i, avail_left_i : std_logic := '0';
    signal fbits_o : unsigned(15 downto 0);
    signal flen_o  : unsigned(5 downto 0);
    signal fvalid_o, fready_i, done_o, has_residual_o : std_logic;
    signal hdr_bits_o : unsigned(7 downto 0);
    signal cbp_luma_o : unsigned(3 downto 0);
    signal cbp_chroma_o : unsigned(1 downto 0);

    signal bitbuf  : std_logic_vector(0 to 127) := (others => '0');
    signal nbits_c : integer := 0;
    signal collecting : boolean := false;
    signal cycle : natural := 0;
begin
    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.mb_header_engine
        port map (clk => clk, rst_n => rst_n, start_i => start_i, ready_o => ready_o,
                  is_i4x4_i => is_i4x4_i, mode16_i => mode16_i, modes4_i => modes4_i,
                  mode_chroma_i => mode_chroma_i, luma_nz_i => luma_nz_i,
                  chroma_dc_nz_i => chroma_dc_nz_i, chroma_ac_nz_i => chroma_ac_nz_i,
                  mode4_top_i => mode4_top_i, mode4_left_i => mode4_left_i,
                  avail_top_i => avail_top_i, avail_left_i => avail_left_i,
                  fbits_o => fbits_o, flen_o => flen_o, fvalid_o => fvalid_o, fready_i => fready_i,
                  done_o => done_o, hdr_bits_o => hdr_bits_o, cbp_luma_o => cbp_luma_o,
                  cbp_chroma_o => cbp_chroma_o, has_residual_o => has_residual_o);

    -- stalling sink that appends accepted fields MSB-first
    sink_p : process(clk)
        variable n : integer;
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            if (cycle mod 5) = 2 or (cycle mod 7) = 4 then fready_i <= '0'; else fready_i <= '1'; end if;
            if fvalid_o = '1' and fready_i = '1' then
                n := to_integer(flen_o);
                for i in 0 to n - 1 loop
                    bitbuf(nbits_c + i) <= fbits_o(n - 1 - i);
                end loop;
                nbits_c <= nbits_c + n;
            end if;
            if collecting = false then
                nbits_c <= 0;
                bitbuf  <= (others => '0');
            end if;
        end if;
    end process;

    main_p : process
        file vec_f : text;
        variable L : line;
        variable tag : character;
        variable val, e_cl, e_cc, e_hr, e_nb : integer;
        variable open_status : file_open_status;
        variable k, mc : natural := 0;
        variable v64 : std_logic_vector(63 downto 0);
        variable v16 : std_logic_vector(15 downto 0);
        variable ehex : std_logic_vector(95 downto 0);
        variable ebits : std_logic_vector(0 to 95);
        variable ok : boolean;
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        file_open(open_status, vec_f, VEC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & VEC_FILE severity failure;
        while not endfile(vec_f) loop
            readline(vec_f, L);
            if L'length = 0 then next; end if;
            read(L, tag); assert tag = 'M' report "expected M" severity failure;
            read(L, val); if val = 1 then is_i4x4_i <= '1'; else is_i4x4_i <= '0'; end if;
            read(L, val); mode16_i <= to_unsigned(val, 2);
            read(L, val); mode_chroma_i <= to_unsigned(val, 2);
            for i in 0 to 15 loop read(L, val); v64(4*i+3 downto 4*i) := std_logic_vector(to_unsigned(val, 4)); end loop;
            modes4_i <= v64;
            for i in 0 to 15 loop read(L, val); if val = 1 then v16(i) := '1'; else v16(i) := '0'; end if; end loop;
            luma_nz_i <= v16;
            read(L, val); if val = 1 then chroma_dc_nz_i <= '1'; else chroma_dc_nz_i <= '0'; end if;
            read(L, val); if val = 1 then chroma_ac_nz_i <= '1'; else chroma_ac_nz_i <= '0'; end if;
            read(L, val); if val = 1 then avail_top_i <= '1'; else avail_top_i <= '0'; end if;
            read(L, val); if val = 1 then avail_left_i <= '1'; else avail_left_i <= '0'; end if;
            for i in 0 to 3 loop read(L, val); v16(4*i+3 downto 4*i) := std_logic_vector(to_unsigned(val, 4)); end loop;
            mode4_top_i <= v16;
            for i in 0 to 3 loop read(L, val); v16(4*i+3 downto 4*i) := std_logic_vector(to_unsigned(val, 4)); end loop;
            mode4_left_i <= v16;
            readline(vec_f, L); read(L, tag); assert tag = 'E' report "expected E" severity failure;
            read(L, e_cl); read(L, e_cc); read(L, e_hr); read(L, e_nb);
            hread(L, ehex);
            for i in 0 to 95 loop ebits(i) := ehex(95 - i); end loop;

            collecting <= true;
            wait until rising_edge(clk);
            start_i <= '1';
            wait until rising_edge(clk);
            start_i <= '0';
            loop wait until rising_edge(clk); exit when done_o = '1'; end loop;
            loop wait until rising_edge(clk); exit when fvalid_o = '0'; end loop;
            wait until rising_edge(clk);
            wait for 1 ns;
            ok := (nbits_c = e_nb) and (to_integer(hdr_bits_o) = e_nb) and
                  (to_integer(cbp_luma_o) = e_cl) and (to_integer(cbp_chroma_o) = e_cc) and
                  ((has_residual_o = '1') = (e_hr = 1));
            if ok then
                for i in 0 to e_nb - 1 loop
                    if bitbuf(i) /= ebits(i) then ok := false; end if;
                end loop;
            end if;
            if not ok then
                mc := mc + 1;
                report "MISMATCH test " & integer'image(k) & ": nbits got " & integer'image(nbits_c) &
                       " exp " & integer'image(e_nb) & " cbp got " & integer'image(to_integer(cbp_luma_o)) &
                       "/" & integer'image(to_integer(cbp_chroma_o)) & " exp " & integer'image(e_cl) & "/" &
                       integer'image(e_cc) severity error;
            end if;
            collecting <= false;
            wait until rising_edge(clk);
            k := k + 1;
        end loop;
        file_close(vec_f);
        if mc = 0 then
            report "PASS - " & integer'image(k) & " mb_header tests verified" severity note;
        else
            report "FAIL - " & integer'image(mc) & " mismatches over " & integer'image(k) & " tests"
                severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process begin
        wait for 20 ms;
        report "watchdog timeout" severity failure;
        std.env.finish;
        wait;
    end process;
end architecture;
