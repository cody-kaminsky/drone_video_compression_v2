--------------------------------------------------------------------------------
-- recon_engine_tb.vhd — self-checking testbench for recon_engine.
-- Reads build/recon_vectors.txt (see tools/gen_recon_vectors.c). Vectors
-- are streamed back-to-back with a periodic downstream stall; recon and
-- ssd results are checked in order as they emerge.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity recon_engine_tb is
    generic (
        VEC_FILE      : string  := "build/recon_vectors.txt";
        STALL_EVERY_N : natural := 7
    );
end entity;

architecture sim of recon_engine_tb is
    constant CLK_PERIOD : time := 5 ns;
    constant RES_W : positive := 20;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal pred_i, src_i : std_logic_vector(127 downto 0) := (others => '0');
    signal res_i   : std_logic_vector(16 * RES_W - 1 downto 0) := (others => '0');
    signal valid_i : std_logic := '0';
    signal ready_o, valid_o : std_logic;
    signal ready_i : std_logic := '1';
    signal recon_o : std_logic_vector(127 downto 0);
    signal ssd_o   : unsigned(19 downto 0);
    signal sent, checked_r, checked_s, mismatches : natural := 0;
    signal stim_done : boolean := false;

    type px_arr is array (0 to 4095, 0 to 15) of integer;
    type int_arr is array (0 to 4095) of integer;
    signal exp_px  : px_arr;
    signal exp_ssd : int_arr;
    signal cycle : natural := 0;
begin
    clk <= not clk after CLK_PERIOD / 2;

    rst_p : process begin
        rst_n <= '0'; wait for 5 * CLK_PERIOD; wait until rising_edge(clk); rst_n <= '1'; wait;
    end process;

    dut : entity work.recon_engine
        generic map (RES_W => RES_W, WITH_SSD => true)
        port map (clk => clk, rst_n => rst_n, pred_i => pred_i, res_i => res_i, src_i => src_i,
                  valid_i => valid_i, ready_o => ready_o, recon_o => recon_o, valid_o => valid_o,
                  ssd_o => ssd_o, ready_i => ready_i);

    -- downstream stall pattern
    stall_p : process(clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            if STALL_EVERY_N > 0 and (cycle mod STALL_EVERY_N) = 3 then
                ready_i <= '0';
            else
                ready_i <= '1';
            end if;
        end if;
    end process;

    stim_p : process
        file vec_f : text;
        variable L : line;
        variable tag : character;
        variable val : integer;
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
            read(L, tag); assert tag = 'P' report "expected P" severity failure;
            for i in 0 to 15 loop read(L, val); pred_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            readline(vec_f, L); read(L, tag); assert tag = 'R' report "expected R" severity failure;
            for i in 0 to 15 loop read(L, val); res_i(RES_W * (i + 1) - 1 downto RES_W * i) <= std_logic_vector(to_signed(val, RES_W)); end loop;
            readline(vec_f, L); read(L, tag); assert tag = 'S' report "expected S" severity failure;
            for i in 0 to 15 loop read(L, val); src_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            readline(vec_f, L); read(L, tag); assert tag = 'O' report "expected O" severity failure;
            for i in 0 to 15 loop read(L, val); exp_px(k, i) <= val; end loop;
            read(L, val); exp_ssd(k) <= val;
            valid_i <= '1';
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
        variable cr, cs, mc : natural := 0;
        variable got : integer;
    begin
        loop
            wait until rising_edge(clk);
            if valid_o = '1' and ready_i = '1' then
                for i in 0 to 15 loop
                    got := to_integer(unsigned(recon_o(8 * i + 7 downto 8 * i)));
                    if got /= exp_px(cr, i) then
                        mc := mc + 1;
                        report "MISMATCH recon test " & integer'image(cr) & " sample " & integer'image(i) &
                               ": expected " & integer'image(exp_px(cr, i)) & " got " & integer'image(got)
                            severity error;
                    end if;
                end loop;
                cr := cr + 1;
                got := to_integer(ssd_o);
                if got /= exp_ssd(cs) then
                    mc := mc + 1;
                    report "MISMATCH ssd test " & integer'image(cs) & ": expected " &
                           integer'image(exp_ssd(cs)) & " got " & integer'image(got) severity error;
                end if;
                cs := cs + 1;
            end if;
            checked_r <= cr; checked_s <= cs; mismatches <= mc;
            exit when stim_done and cr = sent and cs = sent;
        end loop;
        wait for 2 * CLK_PERIOD;
        if mc = 0 then
            report "PASS - " & integer'image(cr) & " recon tests verified (recon + ssd)" severity note;
        else
            report "FAIL - " & integer'image(mc) & " mismatches over " & integer'image(cr) & " tests"
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
