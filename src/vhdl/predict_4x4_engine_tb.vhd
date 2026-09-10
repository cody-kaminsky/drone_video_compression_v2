--------------------------------------------------------------------------------
-- predict_4x4_engine_tb.vhd — self-checking testbench.
-- Reads build/predict4x4_vectors.txt:
--   M <mode> <avail_top> <avail_left> <avail_tl>
--   I <top0..7> <left0..3> <tl>
--   O <p0..p15>
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity predict_4x4_engine_tb is
    generic (VEC_FILE : string := "build/predict4x4_vectors.txt");
end entity;

architecture sim of predict_4x4_engine_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal mode_i  : unsigned(3 downto 0) := (others => '0');
    signal top_i   : std_logic_vector(63 downto 0) := (others => '0');
    signal left_i  : std_logic_vector(31 downto 0) := (others => '0');
    signal tl_i    : std_logic_vector(7 downto 0) := (others => '0');
    signal at_i, al_i, atl_i : std_logic := '0';
    signal valid_i : std_logic := '0';
    signal ready_o, valid_o : std_logic;
    signal ready_i : std_logic := '1';
    signal pred_o  : std_logic_vector(127 downto 0);
    signal test_count, mismatch_count : natural := 0;
    signal done : boolean := false;
begin
    clk <= not clk after CLK_PERIOD / 2;

    rst_p : process begin
        rst_n <= '0'; wait for 5 * CLK_PERIOD; wait until rising_edge(clk); rst_n <= '1'; wait;
    end process;

    dut : entity work.predict_4x4_engine
        port map (clk => clk, rst_n => rst_n, mode_i => mode_i, top_i => top_i, left_i => left_i,
                  tl_i => tl_i, avail_top_i => at_i, avail_left_i => al_i, avail_tl_i => atl_i,
                  valid_i => valid_i, ready_o => ready_o, pred_o => pred_o, valid_o => valid_o,
                  ready_i => ready_i);

    stim_p : process
        file vec_f : text;
        variable L : line;
        variable tag : character;
        variable mode_v, at_v, al_v, atl_v, val : integer;
        variable exp_v : integer_vector(0 to 15);
        variable open_status : file_open_status;
        variable tc, mc : natural := 0;
        variable got : integer;
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);
        file_open(open_status, vec_f, VEC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & VEC_FILE severity failure;
        while not endfile(vec_f) loop
            readline(vec_f, L);
            if L'length = 0 then next; end if;
            read(L, tag); assert tag = 'M' report "expected M" severity failure;
            read(L, mode_v); read(L, at_v); read(L, al_v); read(L, atl_v);
            readline(vec_f, L); read(L, tag); assert tag = 'I' report "expected I" severity failure;
            for i in 0 to 7 loop read(L, val); top_i(8*i+7 downto 8*i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            for i in 0 to 3 loop read(L, val); left_i(8*i+7 downto 8*i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            read(L, val); tl_i <= std_logic_vector(to_unsigned(val, 8));
            readline(vec_f, L); read(L, tag); assert tag = 'O' report "expected O" severity failure;
            for i in 0 to 15 loop read(L, val); exp_v(i) := val; end loop;

            mode_i <= to_unsigned(mode_v, 4);
            if at_v = 1 then at_i <= '1'; else at_i <= '0'; end if;
            if al_v = 1 then al_i <= '1'; else al_i <= '0'; end if;
            if atl_v = 1 then atl_i <= '1'; else atl_i <= '0'; end if;
            valid_i <= '1';
            loop wait until rising_edge(clk); exit when ready_o = '1'; end loop;
            valid_i <= '0';
            loop wait until rising_edge(clk); exit when valid_o = '1'; end loop;

            for i in 0 to 15 loop
                got := to_integer(unsigned(pred_o(8*i+7 downto 8*i)));
                if got /= exp_v(i) then
                    mc := mc + 1;
                    report "MISMATCH test " & integer'image(tc) & " mode " & integer'image(mode_v) &
                           " avail " & integer'image(at_v) & integer'image(al_v) & integer'image(atl_v) &
                           " elem " & integer'image(i) & ": expected " & integer'image(exp_v(i)) &
                           " got " & integer'image(got) severity error;
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
            report "PASS - " & integer'image(test_count) & " predict_4x4 tests verified" severity note;
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
