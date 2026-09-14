--------------------------------------------------------------------------------
-- cavlc_dec_engine_tb.vhd — drive cavlc_dec_engine against the C decoder.
--
-- Each vector in build/cavlc_dec_vectors.txt is a real round trip: a block
-- encoded by cavlc_encode_block and decoded by cavlc_decode_block, so the
-- expectation comes from the routine that has already reconstructed real
-- streams byte-exactly, not from a hand-written guess that could be wrong in
-- the same way the RTL is.
--
--   <nC> <btype> <n_coefs> <nbits> <nbytes> <bytes...> <total_coeff> <16 coefs>
--
-- The engine is instantiated with a real bit_reader rather than a stub,
-- because the peek/consume contract between them is where a length error
-- would hide: consume one bit too few and the next block still decodes to
-- something, just not the right thing.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

use work.cavlc_pkg.all;

entity cavlc_dec_engine_tb is
    generic (
        VEC_FILE : string := "build/cavlc_dec_vectors.txt";
        -- Starve the bit reader pseudorandomly. With the feeder running flat
        -- out, avail_i is high for the whole block and the engine never has
        -- to hold a decision across a gap -- which is precisely the shape of
        -- bug that cost this project four silicon-visible defects on the
        -- encode side. 0 disables the stalls.
        IN_BP    : integer := 1
    );
end entity;

architecture sim of cavlc_dec_engine_tb is
    constant CLK_PERIOD : time := 5 ns;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    -- bit reader
    signal in_data  : unsigned(7 downto 0) := (others => '0');
    signal in_valid : std_logic := '0';
    signal in_ready : std_logic;
    signal in_last  : std_logic := '0';
    signal peek     : unsigned(31 downto 0);
    signal avail    : std_logic;
    signal consume  : std_logic;
    signal consume_n: unsigned(5 downto 0);
    signal bitpos   : unsigned(31 downto 0);
    signal underrun : std_logic;

    -- engine
    signal start_i  : std_logic := '0';
    signal ready_o  : std_logic;
    signal nc_i     : signed(7 downto 0) := (others => '0');
    signal btype_i  : block_type_t := (others => '0');
    signal ncoef_i  : unsigned(4 downto 0) := (others => '0');
    signal done_o   : std_logic;
    signal err_o    : std_logic;
    signal errc_o   : unsigned(3 downto 0);
    signal tc_o     : unsigned(4 downto 0);
    signal coefs_o  : std_logic_vector(255 downto 0);

    -- one vector's bytes, handed to the feeder
    type byte_arr is array (0 to 63) of integer;
    shared variable vbytes : byte_arr;
    shared variable vn     : integer := 0;
    -- Bumped once per vector. The feeder waits on a CHANGE of this, which is
    -- unambiguous; waiting on a level like rst_n is not, because `wait until`
    -- needs an event and the signal may already hold that value.
    signal vec_id : integer := 0;

    -- Stall pattern for the feeder. An LFSR rather than a fixed duty cycle:
    -- a fixed one lines up with the block's own rhythm and can miss the
    -- alignment that breaks.
    signal lfsr : unsigned(15 downto 0) := x"ACE1";

    -- Proof the stalls bite. A back-pressure test that never actually starves
    -- the reader passes for the wrong reason, so count the cycles the engine
    -- spent waiting and fail the run if there were none.
    signal starved : integer := 0;

begin

    clk <= not clk after CLK_PERIOD / 2;

    rd : entity work.bit_reader
        generic map (PEEK_W => 32)
        port map (clk => clk, rst_n => rst_n,
                  in_data => in_data, in_valid => in_valid,
                  in_ready => in_ready, in_last => in_last,
                  peek_o => peek, avail_o => avail,
                  consume_i => consume, consume_n_i => consume_n,
                  bitpos_o => bitpos, underrun_o => underrun);

    dut : entity work.cavlc_dec_engine
        port map (clk => clk, rst_n => rst_n,
                  start_i => start_i, ready_o => ready_o,
                  nc_i => nc_i, btype_i => btype_i, n_coefs_i => ncoef_i,
                  peek_i => peek, avail_i => avail,
                  consume_o => consume, consume_n_o => consume_n,
                  done_o => done_o, err_o => err_o, err_code_o => errc_o,
                  total_coeff_o => tc_o, coefs_o => coefs_o);

    ----------------------------------------------------------------------
    -- Byte feeder: pushes the current vector's bytes, then pads. The padding
    -- matters: a block's last code can sit inside the final byte, and the
    -- reader needs a full window to peek at.
    ----------------------------------------------------------------------
    -- Clocked, and re-armed by a CHANGE of vec_id, because the obvious
    -- sequential feeder deadlocks: the engine stops consuming as soon as it
    -- has its block, so the reader stays full, `wait until in_ready` never
    -- returns, and the feeder is still blocked inside the PREVIOUS vector's
    -- push loop when the next vector starts. It then misses the vec_id event
    -- entirely and resumes pushing at a stale index, so the new vector never
    -- receives its first bytes. Every vector after the first large one
    -- decodes from the wrong bits, which looks exactly like an RTL bug.
    feed_p : process(clk)
        variable i      : integer := 0;
        variable id     : integer := -1;
        variable active : boolean := false;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                active   := false;
                in_valid <= '0';
                id       := vec_id;
            else
                lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13)
                                             xor lfsr(12) xor lfsr(10));
                if vec_id /= id then
                    id := vec_id; i := 0; active := true;
                elsif active and in_valid = '1' and in_ready = '1' then
                    i := i + 1;
                end if;
                -- Pad past the vector's own bytes: a block's last code can sit
                -- inside the final byte and the reader needs a full window.
                if active and i < vn + 8
                   and (IN_BP = 0 or lfsr(3 downto 0) = "0000") then
                    in_valid <= '1';
                    if i < vn then
                        in_data <= to_unsigned(vbytes(i), 8);
                    else
                        in_data <= (others => '0');
                    end if;
                else
                    in_valid <= '0';
                end if;
            end if;
        end if;
    end process;

    starve_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '1' and avail = '0' then
                starved <= starved + 1;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    main_p : process
        file f : text;
        variable L : line;
        variable open_status : file_open_status;
        variable v_nc, v_bt, v_ncoef, v_nbits, v_nbytes, v_tc : integer;
        variable exp_c : integer;
        variable got   : signed(15 downto 0);
        variable i, nvec, bad : integer := 0;
        variable exp_coefs : integer_vector(0 to 15);
    begin
        file_open(open_status, f, VEC_FILE, read_mode);
        assert open_status = open_ok
            report "cannot open " & VEC_FILE severity failure;

        nvec := 0; bad := 0;

        while not endfile(f) loop
            readline(f, L);
            if L'length > 0 then
                read(L, v_nc); read(L, v_bt); read(L, v_ncoef);
                read(L, v_nbits); read(L, v_nbytes);
                for i in 0 to v_nbytes - 1 loop read(L, vbytes(i)); end loop;
                read(L, v_tc);
                for i in 0 to 15 loop read(L, exp_coefs(i)); end loop;
                vn := v_nbytes;

                -- Reset both blocks so each vector starts at bit 0.
                rst_n <= '0';
                wait until rising_edge(clk);
                wait until rising_edge(clk);
                rst_n <= '1';
                wait until rising_edge(clk);

                -- Release the feeder only once reset is high and the vector's
                -- bytes are in place.
                vec_id <= vec_id + 1;
                wait until rising_edge(clk);

                nc_i    <= to_signed(v_nc, 8);
                btype_i <= to_unsigned(v_bt, 3);
                ncoef_i <= to_unsigned(v_ncoef, 5);
                start_i <= '1';
                wait until rising_edge(clk);
                start_i <= '0';

                -- Wait for the block, with a bound so a stuck FSM reports
                -- rather than hanging the run.
                i := 0;
                while done_o = '0' and i < 20000 loop
                    wait until rising_edge(clk);
                    i := i + 1;
                end loop;

                if done_o = '0' then
                    bad := bad + 1;
                    if bad <= 10 then
                        report "vector " & integer'image(nvec)
                             & ": engine never asserted done (nC="
                             & integer'image(v_nc) & " bt=" & integer'image(v_bt) & ")"
                            severity error;
                    end if;
                elsif err_o = '1' then
                    bad := bad + 1;
                    if bad <= 10 then
                        report "vector " & integer'image(nvec)
                             & ": engine reported err code "
                             & integer'image(to_integer(errc_o))
                             & " (1,2=coeff_token 3=level_prefix 4,5=total_zeros "
                             & "6,7=run_before 8=total_coeff>n_coefs) nC=" & integer'image(v_nc)
                             & " bt=" & integer'image(v_bt)
                             & " at bit " & integer'image(to_integer(bitpos))
                             & " of " & integer'image(v_nbits)
                             & ", tc=" & integer'image(to_integer(tc_o))
                            severity error;
                    end if;
                else
                    if to_integer(tc_o) /= v_tc then
                        bad := bad + 1;
                        if bad <= 10 then
                            report "vector " & integer'image(nvec)
                                 & ": total_coeff expected " & integer'image(v_tc)
                                 & " got " & integer'image(to_integer(tc_o))
                                severity error;
                        end if;
                    end if;
                    for i in 0 to 15 loop
                        got := signed(coefs_o(i * 16 + 15 downto i * 16));
                        if to_integer(got) /= exp_coefs(i) then
                            bad := bad + 1;
                            if bad <= 10 then
                                report "vector " & integer'image(nvec)
                                     & " coef " & integer'image(i)
                                     & ": expected " & integer'image(exp_coefs(i))
                                     & " got " & integer'image(to_integer(got))
                                     & " (nC=" & integer'image(v_nc)
                                     & " bt=" & integer'image(v_bt) & ")"
                                    severity error;
                            end if;
                            exit;
                        end if;
                    end loop;
                end if;

                wait until rising_edge(clk);
                nvec := nvec + 1;
            end if;
        end loop;
        file_close(f);

        if IN_BP /= 0 and starved = 0 then
            report "back-pressure was requested but the reader never ran dry; "
                 & "the stall pattern is too gentle to prove anything"
                severity error;
        end if;

        if bad = 0 then
            report "PASS - cavlc_dec_engine: " & integer'image(nvec)
                 & " blocks match the C decoder, "
                 & integer'image(starved) & " cycles waiting on the reader"
                severity note;
        else
            report "FAIL - cavlc_dec_engine: " & integer'image(bad)
                 & " of " & integer'image(nvec) & " vectors wrong" severity error;
        end if;
        finish;
    end process;

end architecture;
