--------------------------------------------------------------------------------
-- bit_reader_tb.vhd — drive bit_reader against the C bitreader_t golden.
--
-- Reads build/bit_reader_vectors.txt (tools/gen_bit_reader_vectors.c):
--   n_bytes, then n decimal bytes
--   n_ops,   then n lines of "<width> <value in hex>"
-- Values are hex because a VHDL integer is 32-bit signed and a textio read
-- of a field >= 2^31 overflows -- a gotcha this project has hit before.
--
-- For each op it checks the top `width` bits of peek_o against the value the
-- C reader produced, then consumes that many. A mismatch is reported with the
-- op index and the bit offset, because "wrong at bit 9137" localises a
-- shift-by-one far faster than a wrong sample at the end of a frame does.
--
-- The byte source deliberately stalls: a reader that only works when bytes
-- arrive every cycle is a reader that works in a testbench and nowhere else.
-- IN_BP selects a tidy pattern or LFSR-driven gaps of up to 128 cycles, the
-- same treatment that found three faults on the encoder's output path.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity bit_reader_tb is
    generic (
        VEC_FILE : string  := "build/bit_reader_vectors.txt";
        PEEK_W   : positive := 32;
        IN_BP    : natural := 1        -- 0 tidy, 1 long stalls
    );
end entity;

architecture sim of bit_reader_tb is
    constant CLK_PERIOD : time := 5 ns;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal in_data  : unsigned(7 downto 0) := (others => '0');
    signal in_valid : std_logic := '0';
    signal in_ready : std_logic;
    signal in_last  : std_logic := '0';

    signal peek     : unsigned(PEEK_W - 1 downto 0);
    signal avail    : std_logic;
    signal consume  : std_logic := '0';
    signal consume_n: unsigned(5 downto 0) := (others => '0');
    signal bitpos   : unsigned(31 downto 0);
    signal underrun : std_logic;

    signal feed_done : boolean := false;
    signal mism      : natural := 0;

    type byte_arr is array (natural range <>) of integer;
    -- Sized for the generator's 4096-byte streams with headroom.
    shared variable bytes_v : byte_arr(0 to 65535);
    shared variable n_bytes : natural := 0;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.bit_reader
        generic map (PEEK_W => PEEK_W)
        port map (
            clk => clk, rst_n => rst_n,
            in_data => in_data, in_valid => in_valid,
            in_ready => in_ready, in_last => in_last,
            peek_o => peek, avail_o => avail,
            consume_i => consume, consume_n_i => consume_n,
            bitpos_o => bitpos, underrun_o => underrun);

    ----------------------------------------------------------------------
    -- Byte source
    ----------------------------------------------------------------------
    feed_p : process
        variable lfsr  : unsigned(15 downto 0) := x"1234";
        variable stall : integer := 0;
        variable i     : natural;
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);
        i := 0;
        while i < n_bytes loop
            -- optional gap before offering the byte
            if IN_BP /= 0 then
                lfsr := lfsr(14 downto 0) &
                        (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
                if lfsr(3 downto 0) = "0000" then
                    stall := 1 + to_integer(lfsr(6 downto 0));
                    in_valid <= '0';
                    for z in 1 to stall loop wait until rising_edge(clk); end loop;
                end if;
            end if;
            in_data  <= to_unsigned(bytes_v(i), 8);
            in_valid <= '1';
            if i = n_bytes - 1 then in_last <= '1'; else in_last <= '0'; end if;
            loop
                wait until rising_edge(clk);
                exit when in_ready = '1';
            end loop;
            i := i + 1;
        end loop;
        in_valid <= '0';
        in_last  <= '0';
        feed_done <= true;
        wait;
    end process;

    ----------------------------------------------------------------------
    -- Checker
    ----------------------------------------------------------------------
    main_p : process
        file f : text;
        variable L : line;
        variable n_ops, i, w : integer;
        variable expv : unsigned(31 downto 0);
        variable mismatches : natural := 0;
        variable open_status : file_open_status;
        variable win : unsigned(PEEK_W - 1 downto 0);
        variable got : unsigned(31 downto 0);
    begin
        file_open(open_status, f, VEC_FILE, read_mode);
        assert open_status = open_ok
            report "cannot open " & VEC_FILE severity failure;

        readline(f, L); read(L, n_bytes);
        for i in 0 to n_bytes - 1 loop
            readline(f, L); read(L, bytes_v(i));
        end loop;
        report "bit_reader_tb: " & integer'image(n_bytes) & " bytes" severity note;

        rst_n <= '0';
        wait for 10 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);

        readline(f, L); read(L, n_ops);
        report "bit_reader_tb: " & integer'image(n_ops) & " ops" severity note;

        for i in 0 to n_ops - 1 loop
            readline(f, L); read(L, w); hread(L, expv);

            -- Wait until the window holds enough bits.
            loop
                wait until rising_edge(clk);
                exit when avail = '1';
            end loop;

            -- Top w bits of the peek window, as an integer.
            win := peek;
            got := (others => '0');
            got(w - 1 downto 0) := win(PEEK_W - 1 downto PEEK_W - w);

            if got /= expv then
                mismatches := mismatches + 1;
                if mismatches < 10 then
                    report "MISMATCH op " & integer'image(i)
                         & " width " & integer'image(w)
                         & " at bit " & integer'image(to_integer(bitpos))
                         & ": expected " & to_hstring(expv)
                         & " got " & to_hstring(got)
                        severity error;
                end if;
            end if;

            consume   <= '1';
            consume_n <= to_unsigned(w, 6);
            wait until rising_edge(clk);
            consume   <= '0';
            consume_n <= (others => '0');
        end loop;

        file_close(f);

        assert underrun = '0'
            report "underrun asserted during the checked region" severity error;

        if mismatches = 0 then
            report "PASS - bit_reader: " & integer'image(n_ops)
                 & " reads match the C bitreader_t, "
                 & integer'image(to_integer(bitpos)) & " bits consumed"
                severity note;
        else
            report "FAIL - bit_reader: " & integer'image(mismatches) & " mismatches"
                severity error;
        end if;
        finish;
    end process;

end architecture;
