--------------------------------------------------------------------------------
-- cavlc_dispatch_tb.vhd — self-checking testbench for cavlc_dispatch.
-- Two DUTs (1 engine and 3 engines) each replay build/dispatch_vectors_in.txt
-- with random input gaps and output stalls; their byte streams are checked
-- against build/dispatch_vectors_out.txt, including out_last on the final
-- byte of every flush group.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;
use work.cavlc_pkg.all;

entity cavlc_dispatch_tb is
    generic (
        IN_FILE  : string := "build/dispatch_vectors_in.txt";
        OUT_FILE : string := "build/dispatch_vectors_out.txt"
    );
end entity;

architecture sim of cavlc_dispatch_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    type nat_arr is array (0 to 1) of natural;
    signal passed : std_logic_vector(1 downto 0) := "00";
    signal failed : std_logic_vector(1 downto 0) := "00";
    constant NENG : nat_arr := (1, 3);
begin
    clk <= not clk after CLK_PERIOD / 2;

    rst_p : process begin
        rst_n <= '0'; wait for 5 * CLK_PERIOD; wait until rising_edge(clk); rst_n <= '1'; wait;
    end process;

    gen_dut : for g in 0 to 1 generate
        signal in_valid, in_ready : std_logic := '0';
        signal in_kind  : unsigned(1 downto 0) := (others => '0');
        signal in_fbits : unsigned(7 downto 0) := (others => '0');
        signal in_flen  : unsigned(5 downto 0) := (others => '0');
        signal in_pkt   : level_packet_t := (block_type => (others => '0'), n_coefs => (others => '0'),
                                             nC => (others => '0'), levels => (others => (others => '0')));
        signal out_valid, out_ready, out_last, flushed_o : std_logic := '0';
        signal out_data : unsigned(7 downto 0);
        signal stim_done : boolean := false;
        signal cycle : natural := 0;
    begin
        dut : entity work.cavlc_dispatch
            generic map (N_ENGINES => NENG(g), PKT_DEPTH => 4, ORDER_DEPTH => 32)
            port map (clk => clk, rst_n => rst_n, in_valid => in_valid, in_ready => in_ready,
                      in_kind => in_kind, in_fbits => in_fbits, in_flen => in_flen, in_pkt => in_pkt,
                      out_valid => out_valid, out_ready => out_ready, out_data => out_data,
                      out_last => out_last, flushed_o => flushed_o);

        stall_p : process(clk)
        begin
            if rising_edge(clk) then
                cycle <= cycle + 1;
                if ((cycle * (g + 3)) mod 11) = 5 then out_ready <= '0'; else out_ready <= '1'; end if;
            end if;
        end process;

        stim_p : process
            file in_f : text;
            variable L : line;
            variable tag : character;
            variable val, len, bits : integer;
            variable open_status : file_open_status;
            variable gap : natural := 0;
        begin
            wait until rst_n = '1';
            wait until rising_edge(clk);
            file_open(open_status, in_f, IN_FILE, read_mode);
            assert open_status = open_ok report "could not open " & IN_FILE severity failure;
            while not endfile(in_f) loop
                readline(in_f, L);
                if L'length = 0 then next; end if;
                read(L, tag);
                case tag is
                    when 'F' =>
                        read(L, len); read(L, bits);
                        in_kind  <= "00";
                        in_flen  <= to_unsigned(len, 6);
                        in_fbits <= to_unsigned(bits, 8);
                    when 'B' =>
                        in_kind <= "01";
                        read(L, val); in_pkt.block_type <= to_unsigned(val, 3);
                        read(L, val); in_pkt.n_coefs    <= to_unsigned(val, 5);
                        read(L, val); in_pkt.nC         <= to_unsigned(val, 5);
                        for i in 0 to 15 loop read(L, val); in_pkt.levels(i) <= to_signed(val, 16); end loop;
                    when 'X' =>
                        in_kind <= "10";
                    when others =>
                        report "bad tag" severity failure;
                end case;
                in_valid <= '1';
                loop wait until rising_edge(clk); exit when in_ready = '1'; end loop;
                in_valid <= '0';
                -- random gaps
                gap := (gap * 7 + 3) mod 5;
                for i in 1 to gap loop wait until rising_edge(clk); end loop;
            end loop;
            file_close(in_f);
            stim_done <= true;
            wait;
        end process;

        check_p : process
            file out_f : text;
            variable L : line;
            variable tag : character;
            variable val : integer;
            variable open_status : file_open_status;
            variable n, mc : natural := 0;
            variable have : boolean := false;
        begin
            file_open(open_status, out_f, OUT_FILE, read_mode);
            assert open_status = open_ok report "could not open " & OUT_FILE severity failure;
            loop
                wait until rising_edge(clk);
                if out_valid = '1' and out_ready = '1' then
                    have := false;
                    while not endfile(out_f) loop
                        readline(out_f, L);
                        if L'length > 0 then read(L, tag); read(L, val); have := true; exit; end if;
                    end loop;
                    if not have then
                        mc := mc + 1;
                        report "DUT" & integer'image(NENG(g)) & " MISMATCH: extra byte " &
                               integer'image(to_integer(out_data)) severity error;
                    elsif to_integer(out_data) /= val or ((out_last = '1') /= (tag = 'L')) then
                        mc := mc + 1;
                        report "DUT" & integer'image(NENG(g)) & " MISMATCH byte " & integer'image(n) &
                               ": expected " & integer'image(val) & " (" & tag & ") got " &
                               integer'image(to_integer(out_data)) & " last=" & std_logic'image(out_last)
                            severity error;
                    end if;
                    n := n + 1;
                end if;
                exit when stim_done and endfile(out_f) and out_valid = '0' and cycle mod 64 = 0;
            end loop;
            -- allow the pipeline to drain a little more and catch stragglers
            for i in 1 to 200 loop
                wait until rising_edge(clk);
                if out_valid = '1' and out_ready = '1' then
                    mc := mc + 1;
                    report "DUT" & integer'image(NENG(g)) & " MISMATCH: byte after end" severity error;
                end if;
            end loop;
            if mc = 0 then
                report "PASS - DUT" & integer'image(NENG(g)) & " " & integer'image(n) & " bytes verified" severity note;
                passed(g) <= '1';
            else
                report "FAIL - DUT" & integer'image(NENG(g)) & " " & integer'image(mc) & " mismatches over " &
                       integer'image(n) & " bytes" severity error;
                failed(g) <= '1';
            end if;
            wait;
        end process;
    end generate;

    finish_p : process
    begin
        loop
            wait until rising_edge(clk);
            exit when (passed or failed) = "11";
        end loop;
        wait for 2 * CLK_PERIOD;
        if failed = "00" then
            report "PASS - cavlc_dispatch both configurations verified" severity note;
        else
            report "FAIL - cavlc_dispatch" severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process begin
        wait for 50 ms;
        report "watchdog timeout" severity failure;
        std.env.finish;
        wait;
    end process;
end architecture;
