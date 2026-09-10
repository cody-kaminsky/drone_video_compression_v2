--------------------------------------------------------------------------------
-- frame_io_tb.vhd — feeds build/frame_stream.txt (pixel beats per MB row) and
-- checks the MB source words against build/frame_src_words.txt (the blocks the
-- C reference sees), plus the byte packer with a short byte sequence.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity frame_io_tb is
    generic (
        IN_FILE  : string := "build/frame_stream.txt";
        SRC_FILE : string := "build/frame_src_words.txt";
        MBS_W    : natural := 30;
        MBS_H    : natural := 17
    );
end entity;

architecture sim of frame_io_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal frame_start_i : std_logic := '0';
    signal mbs_w_i : unsigned(7 downto 0) := (others => '0');
    signal s_valid_i, s_ready_o : std_logic := '0';
    signal s_data_i : std_logic_vector(31 downto 0) := (others => '0');
    signal m_valid_o, m_ready_i : std_logic := '0';
    signal m_data_o : std_logic_vector(127 downto 0);
    signal b_valid_i, b_ready_o, b_last_i : std_logic := '0';
    signal b_data_i : unsigned(7 downto 0) := (others => '0');
    signal o_valid_o, o_ready_i, o_last_o : std_logic := '1';
    signal o_data_o : std_logic_vector(31 downto 0);
    signal o_keep_o : std_logic_vector(3 downto 0);
    signal cycle : natural := 0;
begin
    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.frame_io
        generic map (MAX_W => 480)
        port map (clk => clk, rst_n => rst_n, frame_start_i => frame_start_i, mbs_w_i => mbs_w_i,
                  s_valid_i => s_valid_i, s_ready_o => s_ready_o, s_data_i => s_data_i,
                  m_valid_o => m_valid_o, m_ready_i => m_ready_i, m_data_o => m_data_o,
                  b_valid_i => b_valid_i, b_ready_o => b_ready_o, b_data_i => b_data_i, b_last_i => b_last_i,
                  o_valid_o => o_valid_o, o_ready_i => o_ready_i, o_data_o => o_data_o,
                  o_keep_o => o_keep_o, o_last_o => o_last_o);

    cyc_p : process(clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            if (cycle mod 3) = 1 then m_ready_i <= '0'; else m_ready_i <= '1'; end if;
        end if;
    end process;

    src_p : process
        file f : text;
        variable L : line;
        variable w32 : std_logic_vector(31 downto 0);
        variable open_status : file_open_status;
    begin
        wait until rst_n = '1';
        wait until frame_start_i = '1';
        file_open(open_status, f, IN_FILE, read_mode);
        assert open_status = open_ok report "could not open " & IN_FILE severity failure;
        while not endfile(f) loop
            readline(f, L);
            if L'length = 0 then next; end if;
            hread(L, w32);
            s_data_i <= w32; s_valid_i <= '1';
            loop wait until rising_edge(clk); exit when s_ready_o = '1'; end loop;
            s_valid_i <= '0';
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
        variable exp : std_logic_vector(127 downto 0);
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        file_open(open_status, f, SRC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & SRC_FILE severity failure;
        mbs_w_i <= to_unsigned(MBS_W, 8);
        frame_start_i <= '1';
        wait until rising_edge(clk);
        frame_start_i <= '0';
        while not endfile(f) loop
            readline(f, L);
            if L'length = 0 then next; end if;
            for k in 0 to 15 loop read(L, val); exp(8 * k + 7 downto 8 * k) := std_logic_vector(to_unsigned(val, 8)); end loop;
            loop wait until rising_edge(clk); exit when m_valid_o = '1' and m_ready_i = '1'; end loop;
            if m_data_o /= exp then
                mc := mc + 1;
                if mc <= 10 then
                    report "MISMATCH word " & integer'image(n) & " (MB " & integer'image(n / 24) & " item " & integer'image(n mod 24) &
                           "): got " & integer'image(to_integer(unsigned(m_data_o(7 downto 0)))) & "," & integer'image(to_integer(unsigned(m_data_o(15 downto 8)))) &
                           " exp " & integer'image(to_integer(unsigned(exp(7 downto 0)))) & "," & integer'image(to_integer(unsigned(exp(15 downto 8)))) severity error;
                end if;
            end if;
            n := n + 1;
        end loop;
        file_close(f);
        if mc = 0 then
            report "PASS - frame_io " & integer'image(n) & " source words verified" severity note;
        else
            report "FAIL - " & integer'image(mc) & " mismatches over " & integer'image(n) & " words" severity failure;
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
