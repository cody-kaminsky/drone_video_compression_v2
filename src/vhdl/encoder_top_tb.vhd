--------------------------------------------------------------------------------
-- encoder_top_tb.vhd — whole-kernel test: pixels in (build/frame_stream.txt,
-- 32-bit beats per MB row as a host DMA would deliver them), slice payload
-- out, compared byte for byte with the C reference (build/slice_payload.txt).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity encoder_top_tb is
    generic (
        IN_FILE  : string := "build/frame_stream.txt";
        OUT_FILE : string := "build/slice_payload.txt";
        MBS_W    : natural := 30;
        MBS_H    : natural := 17;
        QP       : natural := 26
    );
end entity;

architecture sim of encoder_top_tb is
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
    signal done_seen : boolean := false;
begin
    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.encoder_top
        generic map (MAX_W => 480, N_ENGINES => 1)
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

    -- pixel feeder with random gaps
    src_p : process
        file f : text;
        variable L : line;
        variable val : integer;
        variable w32 : std_logic_vector(31 downto 0);
        variable open_status : file_open_status;
        variable gap : natural := 0;
    begin
        wait until rst_n = '1';
        wait until frame_start_i = '1';
        file_open(open_status, f, IN_FILE, read_mode);
        assert open_status = open_ok report "could not open " & IN_FILE severity failure;
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
        wait;
    end process;

    main_p : process
        file f : text;
        variable L : line;
        variable val : integer;
        variable open_status : file_open_status;
        variable n, mc : natural := 0;
        variable nb : integer;
        variable t0 : natural;
        variable got : integer;
        variable at_end : boolean;
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
            if o_valid_o = '1' and o_ready_i = '1' then
                if o_keep_o = "1111" then nb := 4; elsif o_keep_o = "0111" then nb := 3; elsif o_keep_o = "0011" then nb := 2; else nb := 1; end if;
                for i in 0 to nb - 1 loop
                    got := to_integer(unsigned(o_data_o(8 * i + 7 downto 8 * i)));
                    if endfile(f) then
                        mc := mc + 1;
                        if mc <= 10 then report "MISMATCH: extra byte " & integer'image(got) severity error; end if;
                    else
                        readline(f, L); read(L, val);
                        at_end := endfile(f);
                        if got /= val then
                            mc := mc + 1;
                            if mc <= 10 then report "MISMATCH byte " & integer'image(n) & ": expected " & integer'image(val) & " got " & integer'image(got) severity error; end if;
                        end if;
                        if at_end and (o_last_o = '0' or i /= nb - 1) then
                            mc := mc + 1; report "MISMATCH: tlast not on the final byte" severity error;
                        end if;
                    end if;
                    n := n + 1;
                end loop;
            end if;
            if frame_done_o = '1' then done_seen <= true; end if;
            exit when done_seen and o_valid_o = '0';
        end loop;
        wait for 10 * CLK_PERIOD;
        if not endfile(f) then
            mc := mc + 1;
            report "MISMATCH: stream ended early after " & integer'image(n) & " bytes" severity error;
        end if;
        report "frame: " & integer'image(n) & " bytes, " & integer'image(cycle - t0) & " cycles, " &
               integer'image((cycle - t0) / (MBS_W * MBS_H)) & " cycles/MB" severity note;
        if mc = 0 then
            report "PASS - encoder_top slice payload bit-exact (" & integer'image(n) & " bytes)" severity note;
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
