--------------------------------------------------------------------------------
-- cavlc_cost_engine_tb.vhd — self-checking testbench.
-- Reads build/cavlc_cost_vectors.txt:
--   M <bt> <n_coefs> <nC>     (nC = 31 for chroma DC)
--   I <l0..l15>
--   O <bits>
-- Vectors are streamed back-to-back (one per cycle) to exercise the
-- pipeline at full rate; results are checked in order as they emerge.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;
use work.cavlc_pkg.all;

entity cavlc_cost_engine_tb is
    generic (VEC_FILE : string := "build/cavlc_cost_vectors.txt");
end entity;

architecture sim of cavlc_cost_engine_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal n_coefs_i : unsigned(4 downto 0) := (others => '0');
    signal nC_i      : unsigned(4 downto 0) := (others => '0');
    signal levels_i  : level_array_t := (others => (others => '0'));
    signal valid_i   : std_logic := '0';
    signal ready_o, valid_o : std_logic;
    signal ready_i   : std_logic := '1';
    signal bits_o    : unsigned(9 downto 0);
    signal sent, checked, mismatches : natural := 0;
    signal stim_done : boolean := false;

    -- expected results queue (in order)
    type int_arr is array (0 to 8191) of integer;
    signal exp_q : int_arr;
    signal exp_bt, exp_n, exp_nc : int_arr;
begin
    clk <= not clk after CLK_PERIOD / 2;

    rst_p : process begin
        rst_n <= '0'; wait for 5 * CLK_PERIOD; wait until rising_edge(clk); rst_n <= '1'; wait;
    end process;

    dut : entity work.cavlc_cost_engine
        port map (clk => clk, rst_n => rst_n, n_coefs_i => n_coefs_i, nC_i => nC_i,
                  levels_i => levels_i, valid_i => valid_i, ready_o => ready_o,
                  bits_o => bits_o, valid_o => valid_o, ready_i => ready_i);

    stim_p : process
        file vec_f : text;
        variable L : line;
        variable tag : character;
        variable bt_v, n_v, nc_v, val : integer;
        variable open_status : file_open_status;
        variable k : natural := 0;
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);
        file_open(open_status, vec_f, VEC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & VEC_FILE severity failure;
        while not endfile(vec_f) loop
            readline(vec_f, L);
            if L'length = 0 then next; end if;
            read(L, tag); assert tag = 'M' report "expected M" severity failure;
            read(L, bt_v); read(L, n_v); read(L, nc_v);
            readline(vec_f, L); read(L, tag); assert tag = 'I' report "expected I" severity failure;
            for i in 0 to 15 loop read(L, val); levels_i(i) <= to_signed(val, 16); end loop;
            readline(vec_f, L); read(L, tag); assert tag = 'O' report "expected O" severity failure;
            read(L, val);
            exp_q(k) <= val; exp_bt(k) <= bt_v; exp_n(k) <= n_v; exp_nc(k) <= nc_v;
            n_coefs_i <= to_unsigned(n_v, 5);
            nC_i      <= to_unsigned(nc_v, 5);
            valid_i   <= '1';
            loop wait until rising_edge(clk); exit when ready_o = '1'; end loop;
            k := k + 1;
        end loop;
        valid_i <= '0';
        file_close(vec_f);
        sent <= k;
        stim_done <= true;
        wait;
    end process;

    check_p : process
        variable c, mc : natural := 0;
    begin
        loop
            wait until rising_edge(clk);
            if valid_o = '1' then
                if to_integer(bits_o) /= exp_q(c) then
                    mc := mc + 1;
                    report "MISMATCH test " & integer'image(c) & " bt " & integer'image(exp_bt(c)) &
                           " n " & integer'image(exp_n(c)) & " nC " & integer'image(exp_nc(c)) &
                           ": expected " & integer'image(exp_q(c)) & " got " &
                           integer'image(to_integer(bits_o)) severity error;
                end if;
                c := c + 1;
                checked <= c; mismatches <= mc;
            end if;
            exit when stim_done and c = sent;
        end loop;
        wait for 2 * CLK_PERIOD;
        if mc = 0 then
            report "PASS - " & integer'image(c) & " cavlc_cost tests verified" severity note;
        else
            report "FAIL - " & integer'image(mc) & " mismatches over " & integer'image(c) & " tests"
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
