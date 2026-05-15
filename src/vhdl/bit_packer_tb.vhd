--------------------------------------------------------------------------------
-- bit_packer_tb.vhd
--
-- Self-checking testbench for bit_packer. Reads two text files generated
-- by tools/gen_bit_packer_vectors:
--
--   build/bit_packer_vectors_in.txt   -- driver beats:
--                                        "B <length> <bits>"  bs_put_bits
--                                        "F"                  flush_i pulse
--   build/bit_packer_vectors_out.txt  -- expected bytes:
--                                        "Y <byte>"           regular byte
--                                        "L <byte>"           last byte of flush
--
-- One process drives stimuli (honoring ready_o, with optional periodic
-- stalls on out_ready to exercise backpressure). One process consumes
-- emitted bytes and asserts each matches the expected sequence, including
-- out_last on "L"-marked bytes.
--
-- The testbench self-asserts and exits via std.env.finish on success;
-- xsim returns 0 on pass, non-zero on assertion failure.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.textio.all;
use std.env.all;

entity bit_packer_tb is
    generic (
        IN_FILE       : string  := "build/bit_packer_vectors_in.txt";
        OUT_FILE      : string  := "build/bit_packer_vectors_out.txt";
        STALL_EVERY_N : natural := 0    -- 0 = always-ready; N = stall 1 cycle every N
    );
end entity;

architecture sim of bit_packer_tb is

    constant CLK_PERIOD : time := 5 ns;     -- 200 MHz

    signal clk       : std_logic := '0';
    signal rst_n     : std_logic := '0';

    signal bits_i    : unsigned(31 downto 0) := (others => '0');
    signal length_i  : unsigned(5 downto 0)  := (others => '0');
    signal valid_i   : std_logic := '0';
    signal ready_o   : std_logic;
    signal flush_i   : std_logic := '0';
    signal flushed_o : std_logic;

    signal out_data  : unsigned(7 downto 0);
    signal out_valid : std_logic;
    signal out_ready : std_logic := '1';
    signal out_last  : std_logic;

    signal driver_done   : boolean := false;
    signal expected_done : boolean := false;
    signal byte_count    : natural := 0;
    signal mismatch_count: natural := 0;

begin

    ----------------------------------------------------------------------
    -- Clock and reset
    ----------------------------------------------------------------------
    clk <= not clk after CLK_PERIOD / 2;

    rst_p : process
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait;
    end process;

    ----------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------
    dut : entity work.bit_packer
        port map (
            clk       => clk,
            rst_n     => rst_n,
            bits_i    => bits_i,
            length_i  => length_i,
            valid_i   => valid_i,
            ready_o   => ready_o,
            flush_i   => flush_i,
            flushed_o => flushed_o,
            out_data  => out_data,
            out_valid => out_valid,
            out_ready => out_ready,
            out_last  => out_last
        );

    ----------------------------------------------------------------------
    -- Producer: read input file, drive beats / flush.
    ----------------------------------------------------------------------
    drive_p : process
        file in_f      : text;
        variable L     : line;
        variable tag   : character;
        variable len   : integer;
        variable val   : integer;
        variable open_status : file_open_status;
    begin
        valid_i  <= '0';
        flush_i  <= '0';
        bits_i   <= (others => '0');
        length_i <= (others => '0');

        wait until rst_n = '1';
        wait until rising_edge(clk);

        file_open(open_status, in_f, IN_FILE, read_mode);
        assert open_status = open_ok
            report "could not open " & IN_FILE
            severity failure;

        while not endfile(in_f) loop
            readline(in_f, L);
            if L'length = 0 then
                next;
            end if;
            read(L, tag);

            if tag = 'B' then
                read(L, len);
                read(L, val);
                bits_i   <= to_unsigned(val, 32);
                length_i <= to_unsigned(len, 6);
                valid_i  <= '1';
                flush_i  <= '0';
                -- wait for handshake
                loop
                    wait until rising_edge(clk);
                    exit when ready_o = '1';
                end loop;
                valid_i  <= '0';

            elsif tag = 'F' then
                bits_i   <= (others => '0');
                length_i <= (others => '0');
                valid_i  <= '0';
                flush_i  <= '1';
                wait until rising_edge(clk);
                flush_i  <= '0';
                -- wait for the flush to fully complete
                loop
                    wait until rising_edge(clk);
                    exit when flushed_o = '1';
                end loop;
            end if;
        end loop;

        file_close(in_f);
        driver_done <= true;
        wait;
    end process;

    ----------------------------------------------------------------------
    -- out_ready generator. STALL_EVERY_N = 0 → always ready.
    ----------------------------------------------------------------------
    ready_p : process
        variable counter : natural := 0;
    begin
        if STALL_EVERY_N = 0 then
            out_ready <= '1';
            wait;
        else
            out_ready <= '1';
            wait until rst_n = '1';
            loop
                wait until rising_edge(clk);
                counter := counter + 1;
                if counter mod STALL_EVERY_N = 0 then
                    out_ready <= '0';
                else
                    out_ready <= '1';
                end if;
            end loop;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Consumer: read expected file, compare each emitted byte.
    ----------------------------------------------------------------------
    consume_p : process
        file out_f     : text;
        variable L     : line;
        variable tag   : character;
        variable val   : integer;
        variable expected_byte  : unsigned(7 downto 0);
        variable expected_last  : std_logic;
        variable open_status    : file_open_status;
        variable mc             : natural := 0;
    begin
        wait until rst_n = '1';

        file_open(open_status, out_f, OUT_FILE, read_mode);
        assert open_status = open_ok
            report "could not open " & OUT_FILE
            severity failure;

        while not endfile(out_f) loop
            readline(out_f, L);
            if L'length = 0 then
                next;
            end if;
            read(L, tag);
            read(L, val);
            expected_byte := to_unsigned(val, 8);
            if tag = 'L' then
                expected_last := '1';
            else
                expected_last := '0';
            end if;

            -- Wait for the next emitted byte.
            loop
                wait until rising_edge(clk);
                exit when out_valid = '1' and out_ready = '1';
            end loop;

            if out_data /= expected_byte then
                mc := mc + 1;
                report "byte mismatch at index " & integer'image(byte_count) &
                       ": expected " & integer'image(val) &
                       ", got "      & integer'image(to_integer(out_data))
                    severity error;
            end if;
            if out_last /= expected_last then
                mc := mc + 1;
                report "out_last mismatch at index " & integer'image(byte_count) &
                       ": expected " & std_logic'image(expected_last) &
                       ", got "      & std_logic'image(out_last)
                    severity error;
            end if;
            byte_count <= byte_count + 1;
        end loop;

        file_close(out_f);
        mismatch_count <= mc;
        expected_done  <= true;
        wait;
    end process;

    ----------------------------------------------------------------------
    -- Watchdog + finalizer.
    ----------------------------------------------------------------------
    final_p : process
    begin
        wait until driver_done and expected_done;
        wait for 10 * CLK_PERIOD;
        if mismatch_count = 0 then
            report "MATCH PASS - " & integer'image(byte_count) &
                   " bytes verified" severity note;
        else
            report "FAIL - " & integer'image(mismatch_count) &
                   " mismatches over " & integer'image(byte_count) & " bytes"
                severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process
    begin
        wait for 5 ms;     -- generous; 4096-beat group at <1us/byte is way under
        report "watchdog timeout - test did not complete"
            severity failure;
        std.env.finish;
        wait;
    end process;

end architecture;
