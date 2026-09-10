--------------------------------------------------------------------------------
-- mb_pipeline_controller_tb.vhd — full-frame integration test.
-- Feeds build/frame_src_words.txt (24 source words per MB, from the C
-- reference with DCC_DUMP_SRC) through the pipeline controller and checks
-- the slice payload bytes against build/slice_payload.txt (DCC_DUMP_SLICE).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity mb_pipeline_controller_tb is
    generic (
        SRC_FILE : string := "build/frame_src_words.txt";
        OUT_FILE : string := "build/slice_payload.txt";
        MBS_W    : natural := 30;
        MBS_H    : natural := 17;
        QP       : natural := 26
    );
end entity;

architecture sim of mb_pipeline_controller_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal frame_start_i, busy_o, frame_done_o : std_logic := '0';
    signal mbs_w_i, mbs_h_i : unsigned(7 downto 0) := (others => '0');
    signal qp_i : unsigned(5 downto 0) := (others => '0');
    signal src_valid_i, src_ready_o : std_logic := '0';
    signal src_data_i : std_logic_vector(127 downto 0) := (others => '0');
    signal out_valid, out_ready, out_last : std_logic := '0';
    signal out_data : unsigned(7 downto 0);
    signal cycle : natural := 0;
    signal nbytes : natural := 0;
    signal done_seen : boolean := false;
begin
    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.mb_pipeline_controller
        generic map (MAX_MB_COLS => 32, N_ENGINES => 1, PKT_DEPTH => 32, ORDER_DEPTH => 64)
        port map (clk => clk, rst_n => rst_n, frame_start_i => frame_start_i, mbs_w_i => mbs_w_i,
                  mbs_h_i => mbs_h_i, qp_i => qp_i, busy_o => busy_o, frame_done_o => frame_done_o,
                  src_valid_i => src_valid_i, src_ready_o => src_ready_o, src_data_i => src_data_i,
                  out_valid => out_valid, out_ready => out_ready, out_data => out_data, out_last => out_last);

    cyc_p : process(clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            if (cycle mod 7) = 3 then out_ready <= '0'; else out_ready <= '1'; end if;
        end if;
    end process;

    -- source feeder: one word per line, honours src_ready
    src_p : process
        file f : text;
        variable L : line;
        variable val : integer;
        variable open_status : file_open_status;
        variable w : std_logic_vector(127 downto 0);
    begin
        wait until rst_n = '1';
        wait until frame_start_i = '1';
        file_open(open_status, f, SRC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & SRC_FILE severity failure;
        while not endfile(f) loop
            readline(f, L);
            if L'length = 0 then next; end if;
            for k in 0 to 15 loop read(L, val); w(8 * k + 7 downto 8 * k) := std_logic_vector(to_unsigned(val, 8)); end loop;
            src_data_i <= w;
            src_valid_i <= '1';
            loop wait until rising_edge(clk); exit when src_ready_o = '1'; end loop;
            src_valid_i <= '0';
        end loop;
        file_close(f);
        wait;
    end process;

    main_p : process
        file f : text;
        variable L : line;
        variable val : integer;
        variable open_status : file_open_status;
        variable n, mc : natural := 0;
        variable exp_last : boolean;
        variable t0 : natural;
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        file_open(open_status, f, OUT_FILE, read_mode);
        assert open_status = open_ok report "could not open " & OUT_FILE severity failure;
        mbs_w_i <= to_unsigned(MBS_W, 8); mbs_h_i <= to_unsigned(MBS_H, 8); qp_i <= to_unsigned(QP, 6);
        frame_start_i <= '1';
        wait until rising_edge(clk);
        frame_start_i <= '0';
        t0 := cycle;
        loop
            wait until rising_edge(clk);
            if out_valid = '1' and out_ready = '1' then
                if endfile(f) then
                    mc := mc + 1;
                    report "MISMATCH: extra byte " & integer'image(to_integer(out_data)) & " at " & integer'image(n) severity error;
                else
                    readline(f, L); read(L, val);
                    exp_last := endfile(f);
                    if to_integer(out_data) /= val or ((out_last = '1') /= exp_last) then
                        mc := mc + 1;
                        if mc <= 20 then
                            report "MISMATCH byte " & integer'image(n) & ": expected " & integer'image(val) & " got " &
                                   integer'image(to_integer(out_data)) & " last=" & std_logic'image(out_last) severity error;
                        end if;
                    end if;
                end if;
                n := n + 1;
                nbytes <= n;
            end if;
            if frame_done_o = '1' then done_seen <= true; end if;
            exit when done_seen;
        end loop;
        wait for 10 * CLK_PERIOD;
        if not endfile(f) then
            mc := mc + 1;
            report "MISMATCH: stream ended early after " & integer'image(n) & " bytes" severity error;
        end if;
        report "frame: " & integer'image(n) & " bytes, " & integer'image(cycle - t0) & " cycles, " &
               integer'image((cycle - t0) / (MBS_W * MBS_H)) & " cycles/MB" severity note;
        if mc = 0 then
            report "PASS - slice payload bit-exact (" & integer'image(n) & " bytes)" severity note;
        else
            report "FAIL - " & integer'image(mc) & " mismatches" severity failure;
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
