--------------------------------------------------------------------------------
-- quant_engine_tb.vhd — self-checking testbench. Reads build/quant_vectors.txt
-- (M <mode> <qp> / I 16 values / O 16 values), drives quant_engine, compares.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity quant_engine_tb is
    generic (VEC_FILE : string := "build/quant_vectors.txt");
end entity;

architecture sim of quant_engine_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal mode_i  : unsigned(2 downto 0) := (others => '0');
    signal qp_i    : unsigned(5 downto 0) := (others => '0');
    signal valid_i : std_logic := '0';
    signal ready_o, valid_o : std_logic;
    signal ready_i : std_logic := '1';
    type block_t is array (0 to 15) of signed(31 downto 0);
    signal din  : block_t := (others => (others => '0'));
    signal dout : block_t;
    signal test_count, mismatch_count : natural := 0;
    signal done : boolean := false;
begin
    clk <= not clk after CLK_PERIOD / 2;

    rst_p : process begin
        rst_n <= '0'; wait for 5 * CLK_PERIOD; wait until rising_edge(clk); rst_n <= '1'; wait;
    end process;

    dut : entity work.quant_engine
        port map (
            clk => clk, rst_n => rst_n, mode_i => mode_i, qp_i => qp_i,
            din_0 => din(0), din_1 => din(1), din_2 => din(2), din_3 => din(3),
            din_4 => din(4), din_5 => din(5), din_6 => din(6), din_7 => din(7),
            din_8 => din(8), din_9 => din(9), din_10 => din(10), din_11 => din(11),
            din_12 => din(12), din_13 => din(13), din_14 => din(14), din_15 => din(15),
            valid_i => valid_i, ready_o => ready_o,
            dout_0 => dout(0), dout_1 => dout(1), dout_2 => dout(2), dout_3 => dout(3),
            dout_4 => dout(4), dout_5 => dout(5), dout_6 => dout(6), dout_7 => dout(7),
            dout_8 => dout(8), dout_9 => dout(9), dout_10 => dout(10), dout_11 => dout(11),
            dout_12 => dout(12), dout_13 => dout(13), dout_14 => dout(14), dout_15 => dout(15),
            valid_o => valid_o, ready_i => ready_i);

    stim_p : process
        file vec_f : text;
        variable L : line;
        variable tag : character;
        variable mode_v, qp_v, val : integer;
        variable exp_v : block_t;
        variable din_v : block_t;
        variable open_status : file_open_status;
        variable tc, mc : natural := 0;
        variable n : integer;
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);
        file_open(open_status, vec_f, VEC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & VEC_FILE severity failure;
        while not endfile(vec_f) loop
            readline(vec_f, L);
            if L'length = 0 then next; end if;
            read(L, tag); assert tag = 'M' report "expected M" severity failure;
            read(L, mode_v); read(L, qp_v);
            readline(vec_f, L); read(L, tag); assert tag = 'I' report "expected I" severity failure;
            for i in 0 to 15 loop read(L, val); din_v(i) := to_signed(val, 32); end loop;
            readline(vec_f, L); read(L, tag); assert tag = 'O' report "expected O" severity failure;
            for i in 0 to 15 loop read(L, val); exp_v(i) := to_signed(val, 32); end loop;

            mode_i <= to_unsigned(mode_v, 3);
            qp_i   <= to_unsigned(qp_v, 6);
            din    <= din_v;
            valid_i <= '1';
            loop wait until rising_edge(clk); exit when ready_o = '1'; end loop;
            valid_i <= '0';
            loop wait until rising_edge(clk); exit when valid_o = '1'; end loop;

            if mode_v >= 4 then n := 4; else n := 16; end if;
            for i in 0 to n - 1 loop
                if dout(i) /= exp_v(i) then
                    mc := mc + 1;
                    report "MISMATCH test " & integer'image(tc) & " mode " & integer'image(mode_v) &
                           " qp " & integer'image(qp_v) & " elem " & integer'image(i) &
                           ": expected " & integer'image(to_integer(exp_v(i))) &
                           " got " & integer'image(to_integer(dout(i))) severity error;
                end if;
            end loop;
            tc := tc + 1;
        end loop;
        file_close(vec_f);
        test_count <= tc; mismatch_count <= mc; done <= true;
        wait;
    end process;

    final_p : process begin
        wait until done;
        wait for 2 * CLK_PERIOD;
        if mismatch_count = 0 then
            report "PASS - " & integer'image(test_count) & " quant tests verified" severity note;
        else
            report "FAIL - " & integer'image(mismatch_count) & " mismatches over " &
                   integer'image(test_count) & " tests" severity failure;
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
