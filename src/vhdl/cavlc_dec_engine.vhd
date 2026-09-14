--------------------------------------------------------------------------------
-- cavlc_dec_engine.vhd
--
-- Decodes one 4x4 CAVLC block: bits in from a bit_reader, up to 16
-- coefficients out in zigzag order, plus the total_coeff the caller needs for
-- the next block's nC.
--
-- This is the mirror of cavlc_engine, and it is the harder direction. Encoding
-- a symbol is a table lookup whose length is known before the write; decoding
-- one is a prefix match whose length is only known after the match, and that
-- length decides where the next symbol starts. So the four stages of a block
--
--     coeff_token -> trailing-one signs and levels -> total_zeros -> runs
--
-- are strictly sequential: nothing here can be pipelined against itself. The
-- parallelism has to come from several blocks in flight, which is the mirror
-- of what cavlc_dispatch does on the encode side.
--
-- Interface to the bit reader is peek/consume rather than get(n), for exactly
-- the reason above: the engine looks at a window, decides what it is worth,
-- and retires that much.
--
--   start_i    with nc_i and btype_i valid; ready_o must be high
--   done_o     one cycle, with coef_* and total_coeff_o valid
--   err_o      with done_o: the stream did not decode. Raised rather than
--              emitting plausible coefficients, because a CAVLC decoder that
--              guesses desynchronises the whole slice and the damage shows up
--              far from the cause.
--
-- Coefficient output is a flat vector of 16 signed 16-bit values in zigzag
-- order, matching what cavlc_decode_block writes in the C reference.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;
use work.cavlc_dec_tables.all;

entity cavlc_dec_engine is
    port (
        clk         : in  std_logic;
        rst_n       : in  std_logic;

        -- Job
        start_i     : in  std_logic;
        ready_o     : out std_logic;
        nc_i        : in  signed(7 downto 0);        -- -1 selects chroma DC
        -- Carried for interface symmetry with cavlc_engine and for debug.
        -- Deliberately NOT indexed on: see the note in S_TZ.
        btype_i     : in  block_type_t;
        n_coefs_i   : in  unsigned(4 downto 0);      -- 4, 15 or 16

        -- Bit reader
        peek_i      : in  unsigned(31 downto 0);
        avail_i     : in  std_logic;
        consume_o   : out std_logic;
        consume_n_o : out unsigned(5 downto 0);

        -- Result
        done_o      : out std_logic;
        err_o       : out std_logic;
        -- Which stage rejected the stream. A bare error flag on a
        -- four-stage sequential decoder localises nothing.
        --   2 coeff_token, 3 level_prefix, 5 total_zeros, 7 run_before,
        --   8 total_coeff larger than the block holds.
        err_code_o  : out unsigned(3 downto 0);
        total_coeff_o : out unsigned(4 downto 0);
        coefs_o     : out std_logic_vector(16 * 16 - 1 downto 0)
    );
end entity;

architecture rtl of cavlc_dec_engine is

    -- The magnitude at which the level suffix widens, spec 9.2.2: 3 << (n-1)
    -- for suffix_length n. Written out as constants rather than computed,
    -- because computing it puts a variable shift and a multiply in series
    -- with the level arithmetic that produces the magnitude being compared,
    -- and that chain was the last path over 5 ns at 200 MHz. Built by the
    -- same expression it replaces so the two cannot disagree.
    type suf_thr_t is array (1 to 6) of integer range 0 to 127;
    function mk_suf_thresh return suf_thr_t is
        variable r : suf_thr_t;
    begin
        for i in 1 to 6 loop r(i) := 3 * 2 ** (i - 1); end loop;
        return r;
    end function;
    constant SUF_THRESH : suf_thr_t := mk_suf_thresh;

    type state_t is (S_IDLE, S_TOKEN, S_T1, S_LEVEL_PFX, S_LEVEL_SFX,
                     S_TZ, S_RUN, S_PLACE_INIT, S_PLACE, S_DONE);
    signal st : state_t := S_IDLE;

    -- Job
    signal nc      : signed(7 downto 0) := (others => '0');
    signal btype   : block_type_t := (others => '0');
    signal n_coefs : unsigned(4 downto 0) := (others => '0');

    -- Decoded fields
    signal total_coeff : unsigned(4 downto 0) := (others => '0');
    signal t1          : unsigned(2 downto 0) := (others => '0');
    signal tz          : unsigned(4 downto 0) := (others => '0');
    signal zeros_left  : unsigned(4 downto 0) := (others => '0');

    -- Levels in encoded order (highest frequency first) and their runs.
    type lev_arr_t is array (0 to 15) of signed(15 downto 0);
    type run_arr_t is array (0 to 15) of unsigned(4 downto 0);
    signal levels : lev_arr_t := (others => (others => '0'));
    signal runs   : run_arr_t := (others => (others => '0'));

    signal idx        : unsigned(4 downto 0) := (others => '0');
    signal suffix_len : unsigned(2 downto 0) := (others => '0');
    signal first_nt1  : std_logic := '0';
    signal lvl_prefix : unsigned(5 downto 0) := (others => '0');

    signal coefs_q : std_logic_vector(16 * 16 - 1 downto 0) := (others => '0');

    -- Scatter cursor. The placement walks backwards from the last nonzero
    -- position, one coefficient per cycle. Doing all sixteen in one cycle is
    -- the obvious coding and it costs 105 ns of logic: each position is
    -- computed from the one before it, so it synthesises as a 16-deep chain
    -- of dependent subtractions feeding sixteen variable writes into a
    -- 256-bit register. Spreading it costs at most sixteen cycles against the
    -- sixty or more the block's entropy decode already takes.
    signal pl_i   : integer range 0 to 15 := 0;
    signal pl_pos : integer range -32 to 31 := 0;
    signal err_q   : std_logic := '0';
    signal errc_q  : unsigned(3 downto 0) := (others => '0');

    -- Combinational view of the peek window. lz is the true leading-zero
    -- count, which level_prefix needs unbounded; lzk is it clamped to the
    -- table depth, which is what a table lookup needs. The two differ for a
    -- code that is ALL zeros -- run_before "0" with one zero left, and
    -- several total_zeros codes -- because such a code has no terminating 1,
    -- so the window runs on into the next symbol's zeros and reports more
    -- leading zeros than the code itself has. The generated tables replicate
    -- those codes up to MAX_LZ, so clamping lands on the right row.
    signal lz  : integer range 0 to 32;
    signal lzk : integer range 0 to MAX_LZ;

    -- consume_o is a registered output, so it asserts the cycle AFTER the FSM
    -- decides, the reader shifts at the end of that cycle, and the new window
    -- is only valid the cycle after that. Without this the FSM reads peek one
    -- cycle early and every symbol after the first is decoded from stale bits
    -- -- which shows up as a wrong sign or a wrong level, not as a crash.
    signal settling : std_logic := '0';

    -- Leading zeros of the 32-bit window, capped at 32.
    function count_lz(v : unsigned(31 downto 0)) return integer is
    begin
        for i in 31 downto 0 loop
            if v(i) = '1' then return 31 - i; end if;
        end loop;
        return 32;
    end function;

    -- The same count, but only over the bits a table code can occupy. No
    -- CAVLC code in these tables runs past MAX_LZ leading zeros, so searching
    -- all 32 bits builds a priority encoder twice the size it needs to be.
    -- Beyond the range the answer is MAX_LZ, which is where the tables
    -- already replicate their all-zero codes.
    function count_lzk(v : unsigned(31 downto 0)) return integer is
    begin
        for i in 31 downto 31 - MAX_LZ loop
            if v(i) = '1' then return 31 - i; end if;
        end loop;
        return MAX_LZ;
    end function;

    -- One table lookup, with the row selected AFTER the read rather than
    -- before it. Each row's key is a fixed slice of the window, so all
    -- MAX_LZ + 1 rows can be read at once from constant addresses and the
    -- leading-zero count only has to pick among the answers. Forming the key
    -- first and then addressing the table puts the priority encoder, the
    -- slice select and the table in series, and that chain was the last path
    -- over 5 ns at 200 MHz.
    function lookup(t : dec_tab_t; v : unsigned(31 downto 0); z : integer)
        return dec_entry_t is
        variable e : dec_entry_t := (0, 0);
    begin
        for i in 0 to MAX_LZ loop
            if z = i then
                e := t(i, to_integer(v(31 - i downto 31 - i - KEY_BITS + 1)));
            end if;
        end loop;
        return e;
    end function;



    -- coeff_token sub-table from nC, mirroring the encoder's selector.
    function ct_lookup(n : signed(7 downto 0); v : unsigned(31 downto 0);
                       z : integer) return dec_entry_t is
    begin
        if n < 0 then
            return lookup(DEC_COEFF_TOKEN_CHROMA_DC, v, z);
        elsif n < 2 then
            return lookup(DEC_COEFF_TOKEN_NC01, v, z);
        elsif n < 4 then
            return lookup(DEC_COEFF_TOKEN_NC23, v, z);
        else
            return lookup(DEC_COEFF_TOKEN_NC47, v, z);
        end if;
    end function;

begin

    lz  <= count_lz(peek_i);
    lzk <= count_lzk(peek_i);

    ready_o       <= '1' when st = S_IDLE else '0';
    done_o        <= '1' when st = S_DONE else '0';
    err_o         <= err_q;
    err_code_o    <= errc_q;
    total_coeff_o <= total_coeff;
    coefs_o       <= coefs_q;

    main_p : process(clk)
        variable e        : dec_entry_t;
        -- All bounded. An unbounded integer here is a 32-bit datapath, and
        -- these sit between the bit reader and a register.
        variable k        : integer range 0 to 63;
        variable lc       : integer range 0 to 8191;   -- level_code
        variable absl     : integer range 0 to 4096;
        variable i        : integer range 0 to 15;
        variable nc8      : integer range -128 to 127;
        variable want     : integer range 0 to 32;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                st          <= S_IDLE;
                consume_o   <= '0';
                consume_n_o <= (others => '0');
                settling    <= '0';
                err_q       <= '0';
                errc_q      <= (others => '0');
                coefs_q     <= (others => '0');
                total_coeff <= (others => '0');
            else
                consume_o   <= '0';
                consume_n_o <= (others => '0');

                if settling = '1' then
                    -- One dead cycle while the reader retires the bits and
                    -- presents the next window.
                    settling <= '0';
                else
                case st is

                ----------------------------------------------------------
                when S_IDLE =>
                    if start_i = '1' then
                        nc      <= nc_i;
                        btype   <= btype_i;
                        n_coefs <= n_coefs_i;
                        err_q   <= '0';
                        coefs_q <= (others => '0');
                        levels  <= (others => (others => '0'));
                        runs    <= (others => (others => '0'));
                        st      <= S_TOKEN;
                    end if;

                ----------------------------------------------------------
                -- coeff_token. nC >= 8 is a 6-bit fixed-length code, not a
                -- VLC, which is why it is handled apart from the tables.
                when S_TOKEN =>
                    if avail_i = '1' then
                        nc8 := to_integer(nc);
                        if nc8 >= 8 then
                            -- 6 bits: tc-1 in the top 4, t1 in the low 2,
                            -- with the all-zero pattern meaning tc = 0.
                            k := to_integer(peek_i(31 downto 26));
                            if k = 3 then
                                total_coeff <= (others => '0');
                                t1          <= (others => '0');
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(6, 6);
                                st          <= S_PLACE_INIT;
                            elsif k / 4 + 1 > to_integer(n_coefs) then
                                err_q <= '1'; errc_q <= x"8"; st <= S_DONE;
                            else
                                total_coeff <= to_unsigned(k / 4 + 1, 5);
                                t1          <= to_unsigned(k mod 4, 3);
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(6, 6);
                                st          <= S_T1;
                            end if;
                        else
                            e := ct_lookup(nc, peek_i, lzk);
                            if e.len = 0 then
                                err_q <= '1'; errc_q <= x"2"; st <= S_DONE;
                            elsif e.sym / 4 > to_integer(n_coefs) then
                                -- More coefficients than the block holds.
                                -- Caught here rather than left to index a
                                -- total_zeros table out of range, which in
                                -- simulation is a crash and in hardware is
                                -- whatever the ROM happens to hold.
                                err_q <= '1'; errc_q <= x"8"; st <= S_DONE;
                            else
                                total_coeff <= to_unsigned(e.sym / 4, 5);
                                t1          <= to_unsigned(e.sym mod 4, 3);
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(e.len, 6);
                                if e.sym / 4 = 0 then
                                    st <= S_PLACE_INIT;   -- no coefficients
                                else
                                    st <= S_T1;
                                end if;
                            end if;
                        end if;
                        idx        <= (others => '0');
                        first_nt1  <= '1';
                    end if;

                ----------------------------------------------------------
                -- Trailing-one signs: one bit each, highest frequency first.
                when S_T1 =>
                    if avail_i = '1' then
                        if idx < t1 then
                            if peek_i(31) = '1' then
                                levels(to_integer(idx)) <= to_signed(-1, 16);
                            else
                                levels(to_integer(idx)) <= to_signed(1, 16);
                            end if;
                            consume_o   <= '1'; settling <= '1';
                            consume_n_o <= to_unsigned(1, 6);
                            idx <= idx + 1;
                        else
                            -- suffix_length starts at 1 for dense blocks with
                            -- few trailing ones, else 0.
                            if total_coeff > 10 and t1 < 3 then
                                suffix_len <= to_unsigned(1, 3);
                            else
                                suffix_len <= (others => '0');
                            end if;
                            if idx >= total_coeff then
                                st <= S_TZ;
                            else
                                st <= S_LEVEL_PFX;
                            end if;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- level_prefix: a unary run of zeros terminated by a 1.
                when S_LEVEL_PFX =>
                    if avail_i = '1' then
                        if lz > 25 then
                            err_q <= '1'; errc_q <= x"3"; st <= S_DONE;
                        else
                            lvl_prefix  <= to_unsigned(lz, 6);
                            consume_o   <= '1'; settling <= '1';
                            consume_n_o <= to_unsigned(lz + 1, 6);
                            st <= S_LEVEL_SFX;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- level_suffix, whose width depends on both suffix_len and
                -- the prefix just read. Escape cases use 12 bits.
                when S_LEVEL_SFX =>
                    if avail_i = '1' then
                        want := 0;
                        if suffix_len = 0 then
                            if lvl_prefix < 14 then
                                lc := to_integer(lvl_prefix);
                            elsif lvl_prefix = 14 then
                                want := 4;
                                lc := 14 + to_integer(peek_i(31 downto 28));
                            else
                                want := 12;
                                lc := 30 + to_integer(peek_i(31 downto 20));
                            end if;
                        else
                            if lvl_prefix < 15 then
                                want := to_integer(suffix_len);
                                lc := to_integer(shift_left(
                                          resize(lvl_prefix, 20),
                                          to_integer(suffix_len)))
                                      + to_integer(peek_i(31 downto 32 - want));
                            else
                                want := 12;
                                lc := to_integer(shift_left(
                                          to_unsigned(15, 20),
                                          to_integer(suffix_len)))
                                      + to_integer(peek_i(31 downto 20));
                            end if;
                        end if;

                        -- The first non-trailing-one level cannot be +-1 when
                        -- fewer than three trailing ones were signalled, so
                        -- the encoder biased it down by 2.
                        if first_nt1 = '1' and t1 < 3 then lc := lc + 2; end if;
                        first_nt1 <= '0';

                        -- Magnitude and sign, not magnitude times sign: a
                        -- variable integer sign of -1 or 1 multiplying a
                        -- 13-bit magnitude synthesises as a real multiplier,
                        -- eight carry chains deep, and it sat on the critical
                        -- path at 200 MHz. The low bit of level_code IS the
                        -- sign, so this is a conditional negate.
                        absl := lc / 2 + 1;
                        if (lc mod 2) = 1 then
                            levels(to_integer(idx)) <= -to_signed(absl, 16);
                        else
                            levels(to_integer(idx)) <= to_signed(absl, 16);
                        end if;

                        -- Widen the suffix as magnitudes grow, spec 9.2.2.
                        if suffix_len = 0 then
                            suffix_len <= to_unsigned(1, 3);
                            if absl > 3 then suffix_len <= to_unsigned(2, 3); end if;
                        elsif suffix_len < 6 and
                              absl > SUF_THRESH(to_integer(suffix_len)) then
                            suffix_len <= suffix_len + 1;
                        end if;

                        if want > 0 then
                            consume_o   <= '1'; settling <= '1';
                            consume_n_o <= to_unsigned(want, 6);
                        end if;

                        if idx + 1 >= total_coeff then
                            st <= S_TZ;
                        else
                            st <= S_LEVEL_PFX;
                        end if;
                        idx <= idx + 1;
                    end if;

                ----------------------------------------------------------
                -- total_zeros, absent when the block is full.
                when S_TZ =>
                    if avail_i = '1' then
                        if total_coeff = n_coefs then
                            tz         <= (others => '0');
                            zeros_left <= (others => '0');
                            idx        <= (others => '0');
                            st         <= S_RUN;
                        else
                            -- Chroma DC is identified by shape, not by the
                            -- block_type code. The C enum in src/cavlc.h and
                            -- the VHDL constants in cavlc_pkg.vhd number the
                            -- BLK_* symbols DIFFERENTLY, which has been
                            -- harmless only because the encode path never
                            -- indexes on block_type -- it carries it through
                            -- and discriminates on nC and n_coefs instead.
                            -- Depending on the numbering here would make this
                            -- the first thing in the project to break on that
                            -- disagreement. n_coefs = 4 is unambiguous.
                            if n_coefs = to_unsigned(4, 5) then
                                e := lookup(DEC_TOTAL_ZEROS_CHROMA_DC(
                                        to_integer(total_coeff) - 1), peek_i, lzk);
                            else
                                e := lookup(DEC_TOTAL_ZEROS_4x4(
                                        to_integer(total_coeff) - 1), peek_i, lzk);
                            end if;
                            if e.len = 0 then
                                err_q <= '1'; errc_q <= x"5"; st <= S_DONE;
                            else
                                tz          <= to_unsigned(e.sym, 5);
                                zeros_left  <= to_unsigned(e.sym, 5);
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(e.len, 6);
                                idx         <= (others => '0');
                                st          <= S_RUN;
                            end if;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- run_before for every coefficient but the last, which takes
                -- whatever zeros remain.
                when S_RUN =>
                    if avail_i = '1' then
                        if idx + 1 >= total_coeff then
                            runs(to_integer(total_coeff) - 1) <= zeros_left;
                            st <= S_PLACE_INIT;
                        elsif zeros_left = 0 then
                            runs(to_integer(idx)) <= (others => '0');
                            idx <= idx + 1;
                        else
                            if zeros_left > 6 then
                                e := lookup(DEC_RUN_BEFORE(6), peek_i, lzk);
                            else
                                e := lookup(DEC_RUN_BEFORE(
                                        to_integer(zeros_left) - 1), peek_i, lzk);
                            end if;
                            if e.len = 0 then
                                err_q <= '1'; errc_q <= x"7"; st <= S_DONE;
                            else
                                runs(to_integer(idx)) <= to_unsigned(e.sym, 5);
                                zeros_left  <= zeros_left - e.sym;
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(e.len, 6);
                                idx <= idx + 1;
                            end if;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- Scatter the levels into zigzag positions. Encoded order is
                -- highest frequency first, so the walk runs backwards from the
                -- last nonzero position, one coefficient per cycle.
                when S_PLACE_INIT =>
                    pl_i   <= 0;
                    pl_pos <= to_integer(total_coeff) + to_integer(tz) - 1;
                    if total_coeff = 0 then
                        st <= S_DONE;
                    else
                        st <= S_PLACE;
                    end if;

                when S_PLACE =>
                    if pl_i < to_integer(total_coeff)
                       and pl_pos >= 0 and pl_pos < 16 then
                        coefs_q(pl_pos * 16 + 15 downto pl_pos * 16) <=
                            std_logic_vector(levels(pl_i));
                        pl_pos <= pl_pos - to_integer(runs(pl_i)) - 1;
                    end if;
                    if pl_i = 15 or pl_i + 1 >= to_integer(total_coeff) then
                        st <= S_DONE;
                    else
                        pl_i <= pl_i + 1;
                    end if;

                ----------------------------------------------------------
                when S_DONE =>
                    st <= S_IDLE;

                end case;
                end if;
            end if;
        end if;
    end process;

end architecture;
