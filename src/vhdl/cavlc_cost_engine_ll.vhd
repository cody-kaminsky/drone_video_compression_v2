--------------------------------------------------------------------------------
-- cavlc_cost_engine_ll.vhd
--
-- Low-latency CAVLC bit-cost estimator. Same function and interface as
-- cavlc_cost_engine (reproduces cavlc_estimate_block_bits() in src/cavlc.c
-- exactly, quirks included) but 5 cycles latency instead of 19, so it can
-- sit on the I_4x4 mode-decision dependency chain.
--
-- Where the systolic version walks the 16 positions one per stage, this
-- version computes everything position-parallel:
--
--   * suffix_length: it only ever grows, by one per level (two from 0),
--     and the growth thresholds are nested (|L| > 3*2^(s-1)). So
--     "suffix_length >= s+1 after position q" is a prefix-OR over the
--     positions above of (level active AND |L| clears threshold s AND
--     suffix_length >= s before that position). Layers 2..6 never look at
--     layer 1 (any active level gives sl >= 1), so they start in the
--     first stage; only the final count needs the tc/t1-dependent initial
--     value. Level bits then follow from a barrel shift of |L|-1 by the
--     suffix_length entering each position.
--   * the topmost level is skipped iff TrailingOnes >= 1, which is the
--     same as "the topmost nonzero is a +/-1" -- known in stage 1.
--   * run_before: the run at a nonzero position is the count of zeros
--     immediately above it, and zeros_left is the count of zeros below it
--     plus that run. Both are pure functions of the nonzero mask.
--   * bits = seed + sum(level bits) + sum(run bits) via adder trees.
--
-- Pipeline (II=1, single global stall):
--   s1  |L|-1, nonzero / one / active masks, threshold flags, sl-0 level
--       bits, sl layer 2
--   s2  TotalCoeff, last, TrailingOnes, total_zeros, zeros-below and
--       run-above per position, sl layers 3-4
--   s3  seed bits, sl layers 5-6, suffix_length per position, run_before
--       bits per position
--   s4  level bits per position, partial sums, run-bits sum
--   s5  final sum
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

entity cavlc_cost_engine_ll is
    port (
        clk       : in  std_logic;
        rst_n     : in  std_logic;
        n_coefs_i : in  unsigned(4 downto 0);   -- 4, 15 or 16
        nC_i      : in  unsigned(4 downto 0);   -- 0..16, 31 = chroma DC
        levels_i  : in  level_array_t;
        valid_i   : in  std_logic;
        ready_o   : out std_logic;
        bits_o    : out unsigned(9 downto 0);
        valid_o   : out std_logic;
        ready_i   : in  std_logic
    );
end entity;

architecture rtl of cavlc_cost_engine_ll is

    subtype mask_t is std_logic_vector(15 downto 0);
    subtype am1_t  is unsigned(11 downto 0);
    type am1_arr   is array (0 to 15) of am1_t;
    subtype k_t    is std_logic_vector(5 downto 1);      -- |L| > 3*2^(s-1), s = 1..5
    type k_arr     is array (0 to 15) of k_t;
    subtype b5_t   is unsigned(4 downto 0);
    type b5_arr    is array (0 to 15) of b5_t;
    subtype b4_t   is unsigned(3 downto 0);
    type b4_arr    is array (0 to 15) of b4_t;
    subtype b3_t   is unsigned(2 downto 0);
    type b3_arr    is array (0 to 15) of b3_t;

    function popcount(m : std_logic_vector) return integer is
        variable c : integer range 0 to 16 := 0;
    begin
        for i in m'range loop
            if m(i) = '1' then c := c + 1; end if;
        end loop;
        return c;
    end function;

    function highest_set(m : mask_t) return integer is
        variable r : integer range 0 to 15 := 0;
    begin
        for i in 0 to 15 loop
            if m(i) = '1' then r := i; end if;
        end loop;
        return r;
    end function;

    function above_mask(p : integer range 0 to 15) return mask_t is
        variable r : mask_t;
    begin
        for i in 0 to 15 loop
            if i > p then r(i) := '1'; else r(i) := '0'; end if;
        end loop;
        return r;
    end function;

    function is_zero(m : std_logic_vector) return boolean is
    begin
        return m = (m'range => '0');
    end function;

    -- zeros in positions [0, p)
    function zeros_below(nz : mask_t; p : integer range 0 to 15) return integer is
        variable c : integer range 0 to 15 := 0;
    begin
        for i in 0 to 14 loop
            if i < p and nz(i) = '0' then c := c + 1; end if;
        end loop;
        return c;
    end function;

    -- consecutive zeros immediately above p (stops at the first '1')
    function run_above(nz : mask_t; p : integer range 0 to 15) return integer is
        variable c     : integer range 0 to 15 := 0;
        variable found : boolean := false;
    begin
        for i in 1 to 15 loop
            if i > p and not found then
                if nz(i) = '1' then found := true; else c := c + 1; end if;
            end if;
        end loop;
        return c;
    end function;

    function ilog2(v : integer) return integer is
    begin
        if v >= 16 then return 4;
        elsif v >= 8 then return 3;
        elsif v >= 4 then return 2;
        elsif v >= 2 then return 1;
        else return 0;
        end if;
    end function;

    function ct_bits(tc, t1 : integer; nc : integer; chroma : boolean) return integer is
        variable base, b : integer;
    begin
        if tc = 0 then
            if chroma then return 2; else return 1; end if;
        end if;
        if chroma then
            if tc <= 1 then return 2 + tc * 4; else return 6 + tc; end if;
        end if;
        if nc >= 8 then return 6; end if;
        if nc < 2 then base := 6; elsif nc < 4 then base := 4; else base := 3; end if;
        if tc > 1 then b := base + tc - 1; else b := base; end if;
        if tc > 4 then b := b + 1; end if;
        if tc > 8 then b := b + 1; end if;
        if tc > 12 then b := b + 1; end if;
        if b > 16 then b := 16; end if;
        return b;
    end function;

    function tz_bits(n_zeros, tc, max_coefs : integer; chroma : boolean) return integer is
        variable b : integer;
    begin
        if n_zeros = 0 then return 1; end if;
        b := 2 + ilog2(max_coefs - tc);
        if chroma and b > 3 then b := 3; end if;
        if b > 9 then b := 9; end if;
        return b;
    end function;

    function rb_bits(run, zl : integer) return integer is
    begin
        if zl = 0 then return 0; end if;
        if zl = 1 then return 1; end if;
        if zl <= 6 then return 1 + ilog2(zl); end if;
        if run < 7 then return 3; end if;
        return run - 3;
    end function;

    -- level bits for suffix_length 0 (|L| <= 7: 2|L|, <= 15: 19, else 28),
    -- from the low 3 bits of |L|-1 and the two range flags (am1 <= 6,
    -- am1 <= 14) computed in stage 1.
    subtype rf_t is std_logic_vector(1 downto 0);
    type rf_arr is array (0 to 15) of rf_t;

    function lb0_of(am1lo : unsigned(2 downto 0); rf : rf_t) return b5_t is
    begin
        if rf(0) = '1' then
            return resize(shift_left(resize(am1lo, 5) + 1, 1), 5);
        elsif rf(1) = '1' then
            return to_unsigned(19, 5);
        else
            return to_unsigned(28, 5);
        end if;
    end function;

    -- thermometer (t6 => t5 => ... => t1) to binary count
    function therm2bin(t : std_logic_vector(6 downto 1)) return b3_t is
        variable r : b3_t;
    begin
        r(2) := t(4);
        r(1) := t(2) xor t(4) xor t(6);
        r(0) := t(1) xor t(2) xor t(3) xor t(4) xor t(5) xor t(6);
        return r;
    end function;

    -- s1
    signal nz1, one1, act1, b2_1 : mask_t;
    signal am1_1   : am1_arr;
    signal k1      : k_arr;
    signal rf1     : rf_arr;
    signal ncoef1  : integer range 0 to 16;
    signal nc1     : integer range 0 to 31;
    signal v1      : std_logic;
    -- s2
    signal nz2, act2, b2_2, b3_2, b4_2 : mask_t;
    signal am1_2   : am1_arr;
    signal k2      : k_arr;
    signal rf2     : rf_arr;
    signal tc2     : integer range 0 to 16;
    signal last2   : integer range 0 to 15;
    signal t1_2    : integer range 0 to 3;
    signal nzer2   : integer range 0 to 16;
    signal zb2     : b4_arr;                 -- zeros below position
    signal ra2     : b4_arr;                 -- run above position
    signal hb2     : mask_t;                 -- has a nonzero below
    signal ncoef2  : integer range 0 to 16;
    signal nc2     : integer range 0 to 31;
    signal v2      : std_logic;
    -- s3
    signal act3    : mask_t;
    signal am1_3   : am1_arr;
    signal rf3     : rf_arr;
    signal sl3     : b3_arr;                 -- suffix_length entering position
    signal rbt3    : b4_arr;                 -- run_before bits per position
    signal seed3   : unsigned(9 downto 0);
    signal v3      : std_logic;
    -- s4
    type p4_arr is array (0 to 3) of unsigned(6 downto 0);
    signal part4   : p4_arr;
    signal rbsum4  : unsigned(7 downto 0);
    signal seed4   : unsigned(9 downto 0);
    signal v4      : std_logic;
    -- s5
    signal bits5   : unsigned(9 downto 0);
    signal v5      : std_logic;

    signal advance : std_logic;

    -- Multi-stage delay lines in flip-flops (plentiful) rather than LUT
    -- shift registers, which count against the LUT budget.
    attribute shreg_extract : string;
    attribute shreg_extract of am1_2 : signal is "no";
    attribute shreg_extract of am1_3 : signal is "no";
    attribute shreg_extract of act2  : signal is "no";
    attribute shreg_extract of act3  : signal is "no";
    attribute shreg_extract of rf2   : signal is "no";
    attribute shreg_extract of rf3   : signal is "no";
    attribute shreg_extract of k2    : signal is "no";

begin

    advance <= ready_i or not v5;
    ready_o <= advance;

    process(clk, rst_n)
        variable lv     : signed(12 downto 0);
        variable am1v   : am1_arr;
        variable kv     : k_arr;
        variable nz, one, big, above, act : mask_t;
        variable topfree : std_logic;
        variable tc, last, t1 : integer range 0 to 16;
        variable chroma : boolean;
        variable sb     : unsigned(9 downto 0);
        variable sl0    : std_logic;
        variable c2, c3, c4, c5, c6, anyact : std_logic;
        variable b3, b4, b5, b6 : mask_t;
        variable t      : std_logic_vector(6 downto 1);
        variable run, zl : integer range 0 to 31;
        variable rb     : integer range 0 to 15;
        variable sh     : am1_t;
        variable lb     : unsigned(4 downto 0);
        variable rs     : unsigned(7 downto 0);
        variable ps     : p4_arr;
    begin
        if rst_n = '0' then
            v1 <= '0'; v2 <= '0'; v3 <= '0'; v4 <= '0'; v5 <= '0';
        elsif rising_edge(clk) then
            if advance = '1' then
                ----------------------------------------------------------
                -- s1
                ----------------------------------------------------------
                for i in 0 to 15 loop
                    lv := levels_i(i)(12 downto 0);
                    -- |L|-1: ~L for L < 0, L-1 for L > 0. One conditional
                    -- decrement: (L xor sign) - (not sign).
                    am1v(i) := (unsigned(lv(11 downto 0)) xor (11 downto 0 => lv(12)))
                               - ("00000000000" & not lv(12));
                    if i < to_integer(n_coefs_i) and lv /= 0 then nz(i) := '1'; else nz(i) := '0'; end if;
                    -- |L| = 1  <=>  am1 = 0 (L = 0 gives am1 = 4095)
                    if am1v(i) = 0 then one(i) := '1'; else one(i) := '0'; end if;
                    -- |L| > 3*2^(s-1)  <=>  am1 >= 3*2^(s-1)
                    if am1v(i) >= 3  then kv(i)(1) := '1'; else kv(i)(1) := '0'; end if;
                    if am1v(i) >= 6  then kv(i)(2) := '1'; else kv(i)(2) := '0'; end if;
                    if am1v(i) >= 12 then kv(i)(3) := '1'; else kv(i)(3) := '0'; end if;
                    if am1v(i) >= 24 then kv(i)(4) := '1'; else kv(i)(4) := '0'; end if;
                    if am1v(i) >= 48 then kv(i)(5) := '1'; else kv(i)(5) := '0'; end if;
                    am1_1(i) <= am1v(i);
                    if am1v(i) <= 6  then rf1(i)(0) <= '1'; else rf1(i)(0) <= '0'; end if;
                    if am1v(i) <= 14 then rf1(i)(1) <= '1'; else rf1(i)(1) <= '0'; end if;
                    k1(i)    <= kv(i);
                end loop;
                -- active = nonzero, except the topmost nonzero when it is a +/-1
                -- (that is exactly TrailingOnes >= 1). Layer 2 of the
                -- suffix_length chain: sl >= 2 before position i.
                topfree := '1';
                c2 := '0';
                for i in 15 downto 0 loop
                    act(i) := nz(i) and not (topfree and one(i));
                    if nz(i) = '1' then topfree := '0'; end if;
                    b2_1(i) <= c2;
                    c2 := c2 or (act(i) and kv(i)(1));
                end loop;
                nz1    <= nz;
                one1   <= one;
                act1   <= act;
                ncoef1 <= to_integer(n_coefs_i);
                nc1    <= to_integer(nC_i);
                v1     <= valid_i;

                ----------------------------------------------------------
                -- s2: counts, positional zero statistics, sl layers 3-4
                ----------------------------------------------------------
                tc   := popcount(nz1);
                last := highest_set(nz1);
                big  := nz1 and not one1;
                if is_zero(big) then
                    above := nz1;
                else
                    above := nz1 and above_mask(highest_set(big));
                end if;
                t1 := popcount(above);
                if t1 > 3 then t1 := 3; end if;
                tc2   <= tc;
                last2 <= last;
                t1_2  <= t1;
                if tc > 0 then nzer2 <= (last + 1) - tc; else nzer2 <= 0; end if;
                for i in 0 to 15 loop
                    zb2(i) <= to_unsigned(zeros_below(nz1, i), 4);
                    ra2(i) <= to_unsigned(run_above(nz1, i), 4);
                    if i = 0 then
                        hb2(i) <= '0';
                    elsif is_zero(nz1(i - 1 downto 0)) then
                        hb2(i) <= '0';
                    else
                        hb2(i) <= '1';
                    end if;
                end loop;
                c3 := '0'; c4 := '0';
                for i in 15 downto 0 loop
                    b3(i) := c3;
                    b4(i) := c4;
                    c4 := c4 or (act1(i) and k1(i)(3) and c3);
                    c3 := c3 or (act1(i) and k1(i)(2) and b2_1(i));
                end loop;
                b3_2   <= b3;
                b4_2   <= b4;
                b2_2   <= b2_1;
                nz2    <= nz1;
                act2   <= act1;
                am1_2  <= am1_1;
                k2     <= k1;
                rf2    <= rf1;
                ncoef2 <= ncoef1;
                nc2    <= nc1;
                v2     <= v1;

                ----------------------------------------------------------
                -- s3: seed, sl layers 5-6 + count, run_before bits
                ----------------------------------------------------------
                chroma := (nc2 = 31);
                sb := to_unsigned(ct_bits(tc2, t1_2, nc2, chroma), 10);
                if tc2 > 0 then
                    sb := sb + t1_2;
                    if tc2 < ncoef2 then
                        sb := sb + tz_bits(nzer2, tc2, ncoef2, chroma);
                    end if;
                end if;
                seed3 <= sb;
                if tc2 > 10 and t1_2 < 3 then sl0 := '1'; else sl0 := '0'; end if;

                c5 := '0'; c6 := '0'; anyact := sl0;
                for i in 15 downto 0 loop
                    t(1) := anyact;
                    t(2) := b2_2(i);
                    t(3) := b3_2(i);
                    t(4) := b4_2(i);
                    t(5) := c5;
                    t(6) := c6;
                    sl3(i) <= therm2bin(t);
                    c6 := c6 or (act2(i) and k2(i)(5) and c5);
                    c5 := c5 or (act2(i) and k2(i)(4) and b4_2(i));
                    anyact := anyact or act2(i);
                end loop;

                for i in 0 to 15 loop
                    -- run_before: every nonzero except the lowest one, while zeros remain
                    if i = last2 then run := 0; else run := to_integer(ra2(i)); end if;
                    zl := to_integer(zb2(i)) + run;
                    if nz2(i) = '1' and hb2(i) = '1' and zl > 0 then
                        rb := rb_bits(run, zl);
                    else
                        rb := 0;
                    end if;
                    rbt3(i) <= to_unsigned(rb, 4);
                end loop;
                act3  <= act2;
                am1_3 <= am1_2;
                rf3   <= rf2;
                v3    <= v2;

                ----------------------------------------------------------
                -- s4: level bits per position, partial sums, run sum
                ----------------------------------------------------------
                for g in 0 to 3 loop
                    ps(g) := (others => '0');
                    for j in 0 to 3 loop
                        lb := (others => '0');
                        if act3(g * 4 + j) = '1' then
                            if sl3(g * 4 + j) = 0 then
                                lb := lb0_of(am1_3(g * 4 + j)(2 downto 0), rf3(g * 4 + j));
                            else
                                sh := shift_right(am1_3(g * 4 + j), to_integer(sl3(g * 4 + j)));
                                if sh >= 15 then
                                    lb := to_unsigned(28, 5);
                                else
                                    lb := resize(sh(3 downto 0), 5) + 2 + resize(sl3(g * 4 + j), 5);
                                end if;
                            end if;
                        end if;
                        ps(g) := ps(g) + lb;
                    end loop;
                end loop;
                part4 <= ps;
                rs := (others => '0');
                for i in 0 to 15 loop
                    rs := rs + rbt3(i);
                end loop;
                rbsum4 <= rs;
                seed4  <= seed3;
                v4     <= v3;

                ----------------------------------------------------------
                -- s5
                ----------------------------------------------------------
                bits5 <= (seed4 + resize(rbsum4, 10))
                       + ((resize(part4(0), 10) + resize(part4(1), 10))
                        + (resize(part4(2), 10) + resize(part4(3), 10)));
                v5    <= v4;
            end if;
        end if;
    end process;

    bits_o  <= bits5;
    valid_o <= v5;

end architecture;
