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
-- are strictly sequential: nothing here can be pipelined against itself.
--
-- The cost of a block is therefore its symbol count, and every symbol costs
-- two cycles: one to decide its length and one for the reader to retire it,
-- and at 200 MHz those two cannot share a cycle. Everything else about this
-- state machine follows from that. Anything that can be read as one symbol
-- is: all the trailing-one signs together, and each level's prefix and
-- suffix together, the suffix bits being latched in the deciding cycle and
-- turned into a level during the retiring one. Anything that reads no bits
-- costs no wait: a run once no zeros remain, a block that fills its
-- positions, a total_zeros that cannot exist. And the coefficients are
-- placed as the runs are decoded, at the position the previous run already
-- fixed, so there is no placement pass afterwards.
--
-- Interface to the bit reader is peek/consume rather than get(n): the engine
-- looks at a window, decides what it is worth, and retires that much.
--
--   start_i    with nc_i and n_coefs_i valid; ready_o must be high
--   done_o     one cycle, with coefs_o and total_coeff_o valid, and start_i
--              is accepted in that same cycle
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
        -- Deliberately NOT indexed on: see the note at S_TZ.
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

    -- The magnitude at which the level suffix widens, spec 9.2.2: |level| >
    -- 3 << (n-1) for suffix_length n, and |level| > 3 when it is 0. Tested
    -- on level_code rather than on the magnitude: |level| = lc/2 + 1, so
    -- |level| > T is lc >= 2T, and the compare no longer waits on the adder
    -- that makes the magnitude. Constants rather than a runtime shift, built
    -- from the expression they stand for.
    type lc_thr_t is array (0 to 6) of integer range 0 to 255;
    function mk_lc_thresh return lc_thr_t is
        variable r : lc_thr_t;
    begin
        r(0) := 6;
        for i in 1 to 6 loop r(i) := 2 * (3 * 2 ** (i - 1)); end loop;
        return r;
    end function;
    constant LC_THRESH : lc_thr_t := mk_lc_thresh;

    -- Longest level_prefix this profile emits: 15 is the escape, and the
    -- extension beyond it belongs to High profiles this encoder never uses.
    constant MAX_PFX : integer := 15;

    type state_t is (S_IDLE, S_TOKEN, S_T1, S_LEVEL_A, S_LEVEL_B,
                     S_TZ, S_RUN, S_DONE);
    signal st : state_t := S_IDLE;

    -- Job
    signal nc      : signed(7 downto 0) := (others => '0');
    signal n_coefs : unsigned(4 downto 0) := (others => '0');

    -- Decoded fields
    signal total_coeff : unsigned(4 downto 0) := (others => '0');
    signal t1          : unsigned(2 downto 0) := (others => '0');
    signal zeros_left  : unsigned(4 downto 0) := (others => '0');

    -- Levels in encoded order (highest frequency first).
    type lev_arr_t is array (0 to 15) of signed(15 downto 0);
    signal levels : lev_arr_t := (others => (others => '0'));

    signal idx        : unsigned(4 downto 0) := (others => '0');
    signal suffix_len : unsigned(2 downto 0) := (others => '0');
    signal first_nt1  : std_logic := '0';

    -- A level's prefix and suffix, latched in the deciding cycle.
    signal lv_pfx  : integer range 0 to MAX_PFX := 0;
    signal lv_fld  : unsigned(11 downto 0) := (others => '0');
    signal lv_want : integer range 0 to 12 := 0;

    -- Placement cursor: where the level being run-decoded goes.
    signal pl_pos : integer range -32 to 31 := 0;

    signal coefs_q : std_logic_vector(16 * 16 - 1 downto 0) := (others => '0');
    signal err_q   : std_logic := '0';
    signal errc_q  : unsigned(3 downto 0) := (others => '0');

    -- Combinational view of the peek window. lz is the true leading-zero
    -- count; lzk is it clamped to the table depth. The two differ for a code
    -- that is ALL zeros -- run_before "0" with one zero left, and several
    -- total_zeros codes -- because such a code has no terminating 1, so the
    -- window runs on into the next symbol's zeros. The generated tables
    -- replicate those codes up to MAX_LZ, so clamping lands on the right row.
    signal lz  : integer range 0 to 32;
    signal lzk : integer range 0 to MAX_LZ;

    -- consume_o is a registered output: it asserts the cycle AFTER the FSM
    -- decides, the reader shifts at the end of that cycle, and the new window
    -- is only valid the cycle after that. `settling` marks the shift cycle.
    -- States that read the window wait it out; states that do not, run.
    signal settling : std_logic := '0';

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

    -- The twelve bits after a level prefix's terminating one: the widest
    -- suffix there is. A select among the prefix lengths, not a shift.
    function field_after(v : unsigned(31 downto 0); p : integer)
        return unsigned is
        variable f : unsigned(11 downto 0) := (others => '0');
    begin
        for i in 0 to MAX_PFX loop
            if p = i then f := v(30 - i downto 19 - i); end if;
        end loop;
        return f;
    end function;

begin

    lz  <= count_lz(peek_i);
    lzk <= count_lzk(peek_i);

    ready_o       <= '1' when st = S_IDLE or st = S_DONE else '0';
    done_o        <= '1' when st = S_DONE else '0';
    err_o         <= err_q;
    err_code_o    <= errc_q;
    total_coeff_o <= total_coeff;
    coefs_o       <= coefs_q;

    main_p : process(clk)
        variable e     : dec_entry_t;
        variable k     : integer range 0 to 63;
        variable lc    : integer range 0 to 8191;    -- level_code
        variable absl  : integer range 0 to 4096;
        variable suf   : integer range 0 to 4095;
        variable want  : integer range 0 to 12;
        variable nc8   : integer range -128 to 127;
        variable tc    : integer range 0 to 16;
        variable wok   : boolean;                    -- window readable now
        variable go_tz : boolean;
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
                settling    <= '0';
                wok   := (settling = '0') and (avail_i = '1');
                go_tz := false;

                case st is

                ----------------------------------------------------------
                when S_IDLE | S_DONE =>
                    if start_i = '1' then
                        nc      <= nc_i;
                        n_coefs <= n_coefs_i;
                        err_q   <= '0';
                        coefs_q <= (others => '0');
                        st      <= S_TOKEN;
                    elsif st = S_DONE then
                        st <= S_IDLE;
                    end if;

                ----------------------------------------------------------
                -- coeff_token. nC >= 8 is a 6-bit fixed-length code, not a
                -- VLC, which is why it is handled apart from the tables.
                -- suffix_length's starting value depends only on what the
                -- token says, so it is settled here rather than later.
                when S_TOKEN =>
                    if wok then
                        nc8 := to_integer(nc);
                        if nc8 >= 8 then
                            k := to_integer(peek_i(31 downto 26));
                            if k = 3 then
                                total_coeff <= (others => '0');
                                t1          <= (others => '0');
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(6, 6);
                                st          <= S_DONE;
                            elsif k / 4 + 1 > to_integer(n_coefs) then
                                err_q <= '1'; errc_q <= x"8"; st <= S_DONE;
                            else
                                tc := k / 4 + 1;
                                total_coeff <= to_unsigned(tc, 5);
                                t1          <= to_unsigned(k mod 4, 3);
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(6, 6);
                                if tc > 10 and (k mod 4) < 3 then
                                    suffix_len <= to_unsigned(1, 3);
                                else
                                    suffix_len <= (others => '0');
                                end if;
                                if (k mod 4) = 0 then st <= S_LEVEL_A;
                                else                  st <= S_T1;
                                end if;
                            end if;
                        else
                            e := ct_lookup(nc, peek_i, lzk);
                            if e.len = 0 then
                                err_q <= '1'; errc_q <= x"2"; st <= S_DONE;
                            elsif e.sym / 4 > to_integer(n_coefs) then
                                -- More coefficients than the block holds.
                                -- Caught here rather than left to index a
                                -- total_zeros table out of range.
                                err_q <= '1'; errc_q <= x"8"; st <= S_DONE;
                            else
                                tc := e.sym / 4;
                                total_coeff <= to_unsigned(tc, 5);
                                t1          <= to_unsigned(e.sym mod 4, 3);
                                consume_o   <= '1'; settling <= '1';
                                consume_n_o <= to_unsigned(e.len, 6);
                                if tc > 10 and (e.sym mod 4) < 3 then
                                    suffix_len <= to_unsigned(1, 3);
                                else
                                    suffix_len <= (others => '0');
                                end if;
                                if tc = 0 then
                                    st <= S_DONE;          -- no coefficients
                                elsif (e.sym mod 4) = 0 then
                                    st <= S_LEVEL_A;
                                else
                                    st <= S_T1;
                                end if;
                            end if;
                        end if;
                        idx       <= (others => '0');
                        first_nt1 <= '1';
                    end if;

                ----------------------------------------------------------
                -- Trailing-one signs: one bit each, highest frequency first,
                -- all of them in one symbol.
                when S_T1 =>
                    if wok then
                        for i in 0 to 2 loop
                            if i < to_integer(t1) then
                                if peek_i(31 - i) = '1' then
                                    levels(i) <= to_signed(-1, 16);
                                else
                                    levels(i) <= to_signed(1, 16);
                                end if;
                            end if;
                        end loop;
                        consume_o   <= '1'; settling <= '1';
                        consume_n_o <= resize(t1, 6);
                        idx <= resize(t1, 5);
                        if resize(t1, 5) = total_coeff then
                            go_tz := true;
                        else
                            st <= S_LEVEL_A;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- A level in two cycles. A: the prefix is a unary run of
                -- zeros; its length with suffix_length fixes how many suffix
                -- bits follow, so prefix and suffix retire as one symbol and
                -- the suffix bits are latched. B: the level is computed from
                -- the latched bits while the reader is retiring them.
                when S_LEVEL_A =>
                    if wok then
                        if lz > MAX_PFX then
                            err_q <= '1'; errc_q <= x"3"; st <= S_DONE;
                        else
                            want := 0;
                            if suffix_len = 0 then
                                if    lz = 14 then want := 4;
                                elsif lz = 15 then want := 12;
                                end if;
                            else
                                if lz < 15 then want := to_integer(suffix_len);
                                else            want := 12;
                                end if;
                            end if;
                            lv_pfx  <= lz;
                            lv_want <= want;
                            lv_fld  <= field_after(peek_i, lz);
                            consume_o   <= '1'; settling <= '1';
                            consume_n_o <= to_unsigned(lz + 1 + want, 6);
                            st <= S_LEVEL_B;
                        end if;
                    end if;

                when S_LEVEL_B =>
                    -- The suffix is the top lv_want bits of the latched field.
                    suf := to_integer(shift_right(lv_fld, 12 - lv_want));
                    if suffix_len = 0 then
                        if    lv_pfx < 14 then lc := lv_pfx;
                        elsif lv_pfx = 14 then lc := 14 + suf;
                        else                   lc := 30 + suf;
                        end if;
                    else
                        if lv_pfx < 15 then
                            lc := to_integer(shift_left(to_unsigned(lv_pfx, 13),
                                                        to_integer(suffix_len))) + suf;
                        else
                            lc := to_integer(shift_left(to_unsigned(15, 13),
                                                        to_integer(suffix_len))) + suf;
                        end if;
                    end if;

                    -- The first non-trailing-one level cannot be +-1 when
                    -- fewer than three trailing ones were signalled, so the
                    -- encoder biased it down by 2.
                    if first_nt1 = '1' and t1 < 3 then lc := lc + 2; end if;
                    first_nt1 <= '0';

                    -- level = (lc/2 + 1) with the sign in the low bit of lc.
                    -- The negative case, -(h + 1), is the bitwise complement
                    -- of h in two's complement: no adder and no negate, which
                    -- took this from seven carry chains in series to four and
                    -- was the last path over 5 ns at 200 MHz.
                    absl := lc / 2 + 1;
                    if (lc mod 2) = 1 then
                        levels(to_integer(idx)) <= not to_signed(lc / 2, 16);
                    else
                        levels(to_integer(idx)) <= to_signed(absl, 16);
                    end if;

                    -- Widen the suffix as magnitudes grow, spec 9.2.2.
                    if suffix_len = 0 then
                        suffix_len <= to_unsigned(1, 3);
                        if lc >= LC_THRESH(0) then suffix_len <= to_unsigned(2, 3); end if;
                    elsif suffix_len < 6 and
                          lc >= LC_THRESH(to_integer(suffix_len)) then
                        suffix_len <= suffix_len + 1;
                    end if;

                    idx <= idx + 1;
                    if idx + 1 >= total_coeff then
                        go_tz := true;
                    else
                        st <= S_LEVEL_A;
                    end if;

                ----------------------------------------------------------
                -- total_zeros. Reached only when the block is not full; a full
                -- block goes straight to the runs, which are then all zero.
                when S_TZ =>
                    if wok then
                        -- Chroma DC is identified by shape, not by the
                        -- block_type code. The C enum in src/cavlc.h and the
                        -- VHDL constants in cavlc_pkg.vhd once numbered the
                        -- BLK_* symbols differently, harmlessly, because the
                        -- encode path never indexes on block_type. n_coefs = 4
                        -- is unambiguous either way.
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
                            zeros_left  <= to_unsigned(e.sym, 5);
                            pl_pos      <= to_integer(total_coeff) + e.sym - 1;
                            consume_o   <= '1'; settling <= '1';
                            consume_n_o <= to_unsigned(e.len, 6);
                            idx         <= (others => '0');
                            st          <= S_RUN;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- One level per step: place it at the cursor the previous
                -- run fixed, then decode its run_before to move the cursor.
                -- The last level takes whatever zeros remain, and once no
                -- zeros remain there is nothing to read and nothing to wait
                -- for. Neither of those steps touches the window, so both run
                -- through the reader's shift cycle.
                when S_RUN =>
                    if idx + 1 >= total_coeff then
                        if pl_pos >= 0 and pl_pos < 16 then
                            coefs_q(pl_pos * 16 + 15 downto pl_pos * 16) <=
                                std_logic_vector(levels(to_integer(idx)));
                        end if;
                        st <= S_DONE;
                    elsif zeros_left = 0 then
                        if pl_pos >= 0 and pl_pos < 16 then
                            coefs_q(pl_pos * 16 + 15 downto pl_pos * 16) <=
                                std_logic_vector(levels(to_integer(idx)));
                        end if;
                        pl_pos <= pl_pos - 1;
                        idx    <= idx + 1;
                    elsif wok then
                        if zeros_left > 6 then
                            e := lookup(DEC_RUN_BEFORE(6), peek_i, lzk);
                        else
                            e := lookup(DEC_RUN_BEFORE(
                                    to_integer(zeros_left) - 1), peek_i, lzk);
                        end if;
                        if e.len = 0 then
                            err_q <= '1'; errc_q <= x"7"; st <= S_DONE;
                        else
                            if pl_pos >= 0 and pl_pos < 16 then
                                coefs_q(pl_pos * 16 + 15 downto pl_pos * 16) <=
                                    std_logic_vector(levels(to_integer(idx)));
                            end if;
                            zeros_left  <= zeros_left - e.sym;
                            pl_pos      <= pl_pos - e.sym - 1;
                            consume_o   <= '1'; settling <= '1';
                            consume_n_o <= to_unsigned(e.len, 6);
                            idx         <= idx + 1;
                        end if;
                    end if;

                end case;

                -- Levels done: total_zeros next, unless the block is full,
                -- in which case there are none and the runs are all zero.
                if go_tz then
                    if total_coeff = n_coefs then
                        zeros_left <= (others => '0');
                        pl_pos     <= to_integer(total_coeff) - 1;
                        idx        <= (others => '0');
                        st         <= S_RUN;
                    else
                        st <= S_TZ;
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture;
