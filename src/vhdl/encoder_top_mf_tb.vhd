--------------------------------------------------------------------------------
-- encoder_top_mf_tb.vhd — multi-frame whole-kernel test: FRAMES frames are
-- pushed back to back through encoder_top (frame_start per frame, pixel
-- stream per MB row as a host DMA would deliver it) and every frame's slice
-- payload is compared byte for byte with the C reference.
--   build/mf/stream<k>.txt   pixel beats (gen_frame_stream.py), frame k
--   build/mf/payload<k>.txt  MB-layer payload bytes (DCC_DUMP_SLICE), frame k
-- QP per frame from the QPS generic string ("24,28,32").
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity encoder_top_mf_tb is
    generic (
        FRAMES    : natural := 3;
        DIR       : string  := "build/mf/";
        MBS_W     : natural := 120;
        MBS_H     : natural := 68;
        QP0       : natural := 24;
        QP1       : natural := 28;
        QP2       : natural := 32;
        N_ENGINES : natural := 2
    );
end entity;

architecture sim of encoder_top_mf_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal frame_start_i, busy_o, frame_done_o : std_logic := '0';
    signal mbs_w_i, mbs_h_i : unsigned(7 downto 0) := (others => '0');
    signal qp_i : unsigned(5 downto 0) := (others => '0');
    signal s_valid_i, s_ready_o : std_logic := '0';
    signal s_data_i : std_logic_vector(31 downto 0) := (others => '0');
    signal o_valid_o, o_ready_i, o_last_o : std_logic := '0';
    signal o_data_o : std_logic_vector(31 downto 0);
    signal o_keep_o : std_logic_vector(3 downto 0);
    signal cycle : natural := 0;
    signal cur_frame : integer := -1;      -- frame being fed / checked
    signal feed_done : boolean := false;

    function qp_of(k : natural) return natural is
    begin
        case k is
            when 0 => return QP0;
            when 1 => return QP1;
            when others => return QP2;
        end case;
    end function;
begin
    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.encoder_top
        generic map (MAX_W => MBS_W * 16, N_ENGINES => N_ENGINES)
        port map (clk => clk, rst_n => rst_n, frame_start_i => frame_start_i, mbs_w_i => mbs_w_i,
                  mbs_h_i => mbs_h_i, qp_i => qp_i, busy_o => busy_o, frame_done_o => frame_done_o,
                  s_valid_i => s_valid_i, s_ready_o => s_ready_o, s_data_i => s_data_i,
                  o_valid_o => o_valid_o, o_ready_i => o_ready_i, o_data_o => o_data_o,
                  o_keep_o => o_keep_o, o_last_o => o_last_o);

    cyc_p : process(clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            if (cycle mod 5) = 2 then o_ready_i <= '0'; else o_ready_i <= '1'; end if;
        end if;
    end process;

    -- pixel feeder with random gaps, one file per frame
    src_p : process
        file f : text;
        variable L : line;
        variable w32 : std_logic_vector(31 downto 0);
        variable open_status : file_open_status;
        variable gap : natural := 0;
        variable k : integer := 0;
    begin
        wait until rst_n = '1';
        for k in 0 to FRAMES - 1 loop
            wait until frame_start_i = '1';
            feed_done <= false;
            file_open(open_status, f, DIR & "stream" & integer'image(k) & ".txt", read_mode);
            assert open_status = open_ok report "could not open stream file " & integer'image(k) severity failure;
            while not endfile(f) loop
                readline(f, L);
                if L'length = 0 then next; end if;
                hread(L, w32);
                s_data_i <= w32;
                s_valid_i <= '1';
                loop wait until rising_edge(clk); exit when s_ready_o = '1'; end loop;
                s_valid_i <= '0';
                gap := (gap * 5 + 1) mod 4;
                if gap = 3 then wait until rising_edge(clk); end if;
            end loop;
            file_close(f);
            feed_done <= true;
        end loop;
        wait;
    end process;

    main_p : process
        file f : text;
        variable L : line;
        variable val : integer;
        variable open_status : file_open_status;
        variable n, mc, mc_tot : natural := 0;
        variable nb : integer;
        variable t0, tsum : natural := 0;
        variable got : integer;
        variable at_end, done_seen, last_seen : boolean;
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        mbs_w_i <= to_unsigned(MBS_W, 8); mbs_h_i <= to_unsigned(MBS_H, 8);
        for k in 0 to FRAMES - 1 loop
            cur_frame <= k;
            file_open(open_status, f, DIR & "payload" & integer'image(k) & ".txt", read_mode);
            assert open_status = open_ok report "could not open payload file " & integer'image(k) severity failure;
            qp_i <= to_unsigned(qp_of(k), 6);
            wait until rising_edge(clk);
            frame_start_i <= '1';
            wait until rising_edge(clk);
            frame_start_i <= '0';
            t0 := cycle; n := 0; mc := 0; done_seen := false; last_seen := false;
            loop
                wait until rising_edge(clk);
                if o_valid_o = '1' and o_ready_i = '1' then
                    if o_keep_o = "1111" then nb := 4; elsif o_keep_o = "0111" then nb := 3; elsif o_keep_o = "0011" then nb := 2; else nb := 1; end if;
                    for i in 0 to nb - 1 loop
                        got := to_integer(unsigned(o_data_o(8 * i + 7 downto 8 * i)));
                        if endfile(f) then
                            mc := mc + 1;
                            if mc <= 10 then report "frame " & integer'image(k) & " MISMATCH: extra byte " & integer'image(got) severity error; end if;
                        else
                            readline(f, L); read(L, val);
                            at_end := endfile(f);
                            if got /= val then
                                mc := mc + 1;
                                if mc <= 10 then report "frame " & integer'image(k) & " MISMATCH byte " & integer'image(n) & ": expected " & integer'image(val) & " got " & integer'image(got) severity error; end if;
                            end if;
                            if at_end and (o_last_o = '0' or i /= nb - 1) then
                                mc := mc + 1; report "frame " & integer'image(k) & " MISMATCH: tlast not on the final byte" severity error;
                            end if;
                        end if;
                        n := n + 1;
                    end loop;
                    if o_last_o = '1' then last_seen := true; end if;
                end if;
                if frame_done_o = '1' then done_seen := true; end if;
                exit when done_seen and last_seen and o_valid_o = '0';
            end loop;
            if not endfile(f) then
                mc := mc + 1;
                report "frame " & integer'image(k) & " MISMATCH: stream ended early after " & integer'image(n) & " bytes" severity error;
            end if;
            file_close(f);
            tsum := tsum + (cycle - t0);
            report "frame " & integer'image(k) & " (QP " & integer'image(qp_of(k)) & "): " & integer'image(n) & " bytes, " &
                   integer'image(cycle - t0) & " cycles, " & integer'image((cycle - t0) / (MBS_W * MBS_H)) & " cycles/MB, " &
                   integer'image(mc) & " mismatches" severity note;
            mc_tot := mc_tot + mc;
            -- let the feeder finish its file (it should already have) before the next frame
            wait until rising_edge(clk);
            while not feed_done loop wait until rising_edge(clk); end loop;
            wait for 20 * CLK_PERIOD;
        end loop;
        report "total: " & integer'image(FRAMES) & " frames, " & integer'image(tsum) & " cycles, " &
               integer'image(tsum / (FRAMES * MBS_W * MBS_H)) & " cycles/MB avg" severity note;
        if mc_tot = 0 then
            report "PASS - all " & integer'image(FRAMES) & " frames bit-exact" severity note;
        else
            report "FAIL - " & integer'image(mc_tot) & " mismatches" severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process begin
        wait for 400 ms;
        report "watchdog timeout" severity failure;
        std.env.finish;
        wait;
    end process;
end architecture;
