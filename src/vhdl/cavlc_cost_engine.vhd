--------------------------------------------------------------------------------
-- cavlc_cost_engine.vhd
--
-- CAVLC bit-cost estimator for mode decision. Reproduces
-- cavlc_estimate_block_bits() in src/cavlc.c EXACTLY, approximations and
-- quirks included (only the topmost trailing one is skipped in the level
-- pass; the run_before pass starts with a zero-length run at the top and
-- stops after TotalCoeff-1 runs). Mode decision must rank candidates the
-- same way the C reference does, so this is the golden behaviour, not the
-- spec's exact code lengths.
--
-- Throughput: one block per cycle (the I_4x4 search evaluates 144
-- candidates per MB). Latency 19 cycles.
--
-- Structure:
--   stage A   per-position: nonzero flag (within n_coefs), |level|-1 (for
--             L<0 this is just ~L, no negate).
--   stage B1  TotalCoeff, last nonzero, TrailingOnes (nonzeros above the
--             highest non-±1 level, capped at 3), total_zeros.
--   stage B2  coeff_token bits, total_zeros bits, sign bits, suffix_length
--             init -> seed of the accumulator.
--   16 systolic stages, one per position from 15 down to 0, each carrying
--             (bits, suffix_length, zeros_left, run count, previous
--             position). A stage adds its position's level bits and
--             run_before bits.
--
-- Input levels are treated as sign-extended 13-bit values (|L| <= 2063
-- after CAVLC's own limits).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

entity cavlc_cost_engine is
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

architecture rtl of cavlc_cost_engine is

    constant NSTAGE : integer := 16;

    subtype mask_t is std_logic_vector(15 downto 0);

    -- Per-position payload that travels down the systolic pipeline
    type pos_t is record
        am1  : unsigned(11 downto 0);    -- |level| - 1
    end record;
    type pos_arr is array (0 to 15) of pos_t;

    function popcount(m : mask_t) return integer is
        variable c : integer range 0 to 16 := 0;
    begin
        for i in 0 to 15 loop
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

    function is_zero(m : mask_t) return boolean is
    begin
        return m = (m'range => '0');
    end function;

    -- floor(log2(v)) for 1 <= v <= 16
    function ilog2(v : integer) return integer is
    begin
        if v >= 16 then return 4;
        elsif v >= 8 then return 3;
        elsif v >= 4 then return 2;
        elsif v >= 2 then return 1;
        else return 0;
        end if;
    end function;

    -- coeff_token_bits_approx
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

    -- total_zeros_bits_approx (only called when tc < max_coefs)
    function tz_bits(n_zeros, tc, max_coefs : integer; chroma : boolean) return integer is
        variable b : integer;
    begin
        if n_zeros = 0 then return 1; end if;
        b := 2 + ilog2(max_coefs - tc);
        if chroma and b > 3 then b := 3; end if;
        if b > 9 then b := 9; end if;
        return b;
    end function;

    -- run_before_bits_approx
    function rb_bits(run, zl : integer) return integer is
    begin
        if zl = 0 then return 0; end if;
        if zl = 1 then return 1; end if;
        if zl <= 6 then return 1 + ilog2(zl); end if;
        if run < 7 then return 3; end if;
        return run - 3;
    end function;

    -- Pipeline carry record
    type carry_t is record
        valid   : std_logic;
        bits    : unsigned(9 downto 0);
        sl      : integer range 0 to 6;
        zl      : integer range 0 to 15;
        rb_left : integer range 0 to 16;   -- run codes still allowed (tc-1 at start)
        prev    : integer range 0 to 16;   -- previous nonzero position (+1 at start)
        last_nz : integer range 0 to 15;
        skip1   : std_logic;               -- TrailingOnes >= 1: skip topmost level
    end record;
    type carry_arr is array (0 to NSTAGE) of carry_t;
    signal stage : carry_arr;

    -- Stage A registers
    signal nz_a    : mask_t;
    signal pos_a   : pos_arr;
    signal ncoef_a : integer range 0 to 16;
    signal nc_a    : integer range 0 to 31;
    signal va      : std_logic;
    -- Stage B1 registers
    signal nz_b    : mask_t;
    signal pos_b   : pos_arr;
    signal tc_b    : integer range 0 to 16;
    signal last_b  : integer range 0 to 15;
    signal t1_b    : integer range 0 to 3;
    signal nzeros_b : integer range 0 to 16;
    signal ncoef_b : integer range 0 to 16;
    signal nc_b    : integer range 0 to 31;
    signal vb      : std_logic;

    -- Delay lines into / along the systolic stages
    type pos_dl is array (0 to NSTAGE) of pos_arr;
    type nz_dl  is array (0 to NSTAGE) of mask_t;
    signal dl_pos : pos_dl;
    signal dl_nz  : nz_dl;

    signal advance : std_logic;

begin

    advance <= ready_i;
    ready_o <= advance;

    ------------------------------------------------------------------
    -- Stages A, B1, B2
    ------------------------------------------------------------------
    process(clk, rst_n)
        variable lv    : signed(12 downto 0);
        variable am1   : unsigned(11 downto 0);
        variable nz, one, big, above : mask_t;
        variable tc, last, t1 : integer range 0 to 16;
        variable chroma : boolean;
        variable seed  : carry_t;
    begin
        if rst_n = '0' then
            va <= '0'; vb <= '0';
            stage(0).valid <= '0';
        elsif rising_edge(clk) then
            if advance = '1' then
                -- Stage A
                for i in 0 to 15 loop
                    lv := levels_i(i)(12 downto 0);
                    -- |L| - 1: for L < 0 it is ~L (no negate); for L > 0, L - 1
                    if lv(12) = '1' then
                        am1 := unsigned(not lv(11 downto 0));
                    else
                        am1 := unsigned(lv(11 downto 0)) - 1;
                    end if;
                    if i < to_integer(n_coefs_i) and lv /= 0 then nz(i) := '1'; else nz(i) := '0'; end if;
                    pos_a(i).am1 <= am1;
                end loop;
                nz_a    <= nz;
                ncoef_a <= to_integer(n_coefs_i);
                nc_a    <= to_integer(nC_i);
                va      <= valid_i;

                -- Stage B1: counts
                for i in 0 to 15 loop
                    if pos_a(i).am1 = 0 then one(i) := '1'; else one(i) := '0'; end if;
                end loop;
                tc   := popcount(nz_a);
                last := highest_set(nz_a);
                big  := nz_a and not one;
                if is_zero(big) then
                    above := nz_a;
                else
                    above := nz_a and above_mask(highest_set(big));
                end if;
                t1 := popcount(above);
                if t1 > 3 then t1 := 3; end if;
                tc_b   <= tc;
                last_b <= last;
                t1_b   <= t1;
                if tc > 0 then nzeros_b <= (last + 1) - tc; else nzeros_b <= 0; end if;
                nz_b    <= nz_a;
                pos_b   <= pos_a;
                ncoef_b <= ncoef_a;
                nc_b    <= nc_a;
                vb      <= va;

                -- Stage B2: seed
                chroma := (nc_b = 31);
                seed.valid   := vb;
                seed.bits    := to_unsigned(ct_bits(tc_b, t1_b, nc_b, chroma), 10);
                seed.last_nz := last_b;
                if tc_b > 0 then
                    seed.bits := seed.bits + t1_b;            -- sign bits
                    if tc_b < ncoef_b then
                        seed.bits := seed.bits + tz_bits(nzeros_b, tc_b, ncoef_b, chroma);
                    end if;
                    seed.rb_left := tc_b - 1;
                else
                    seed.rb_left := 0;
                end if;
                if tc_b > 10 and t1_b < 3 then seed.sl := 1; else seed.sl := 0; end if;
                seed.zl   := nzeros_b;
                seed.prev := last_b + 1;
                if t1_b >= 1 then seed.skip1 := '1'; else seed.skip1 := '0'; end if;
                stage(0)  <= seed;
                dl_pos(0) <= pos_b;
                dl_nz(0)  <= nz_b;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Systolic level / run_before stages. Stage k processes position
    -- p = 15 - k using dl_*(k) and produces stage(k+1), dl_*(k+1).
    ------------------------------------------------------------------
    gen_stages : for k in 0 to NSTAGE - 1 generate
        constant P : integer := 15 - k;
    begin
        process(clk, rst_n)
            variable c      : carry_t;
            variable ps     : pos_t;
            variable pre4   : unsigned(3 downto 0);
            variable esc    : boolean;
            variable grow   : boolean;
            variable lb     : integer range 0 to 28;
            variable run    : integer range 0 to 16;
            variable nsl    : integer range 0 to 6;
            variable sh     : unsigned(11 downto 0);
        begin
            if rst_n = '0' then
                stage(k + 1).valid <= '0';
            elsif rising_edge(clk) then
                if advance = '1' then
                    c  := stage(k);
                    ps := dl_pos(k)(P);
                    if dl_nz(k)(P) = '1' then
                        ------------------------------------------------
                        -- level bits (level_bits in cavlc.c)
                        ------------------------------------------------
                        if not (P = c.last_nz and c.skip1 = '1') then
                            if c.sl = 0 then
                                -- |L| <= 7 : 2|L| bits ; <= 15 : 19 ; else 28
                                if ps.am1 <= 6 then
                                    lb := 2 * (to_integer(ps.am1(2 downto 0)) + 1);
                                    if ps.am1 >= 3 then nsl := 2; else nsl := 1; end if;
                                elsif ps.am1 <= 14 then
                                    lb := 19; nsl := 2;
                                else
                                    lb := 28; nsl := 2;
                                end if;
                            else
                                -- prefix = (|L|-1) >> sl; escape when >= 15,
                                -- i.e. |L|-1 >= 15 << sl. Growth test
                                -- |L| > 3<<(sl-1)  <=>  |L|-1 >= 3<<(sl-1).
                                sh   := shift_right(ps.am1, c.sl);
                                pre4 := sh(3 downto 0);
                                esc  := (ps.am1 >= to_unsigned(15 * (2 ** c.sl), 12));
                                grow := (ps.am1 >= to_unsigned(3 * (2 ** (c.sl - 1)), 12));
                                if not esc then
                                    lb := to_integer(pre4) + 2 + c.sl;
                                    if grow and c.sl < 6 then nsl := c.sl + 1; else nsl := c.sl; end if;
                                else
                                    lb := 28;
                                    if c.sl < 6 then nsl := c.sl + 1; else nsl := c.sl; end if;
                                end if;
                            end if;
                            c.bits := c.bits + lb;
                            c.sl   := nsl;
                        end if;
                        ------------------------------------------------
                        -- run_before bits
                        ------------------------------------------------
                        if c.zl > 0 and c.rb_left > 0 then
                            run    := (c.prev - 1) - P;
                            c.bits := c.bits + rb_bits(run, c.zl);
                            c.zl   := c.zl - run;
                            c.rb_left := c.rb_left - 1;
                            c.prev := P;
                        end if;
                    end if;
                    stage(k + 1) <= c;
                    dl_pos(k + 1) <= dl_pos(k);
                    dl_nz(k + 1)  <= dl_nz(k);
                end if;
            end if;
        end process;
    end generate;

    bits_o  <= stage(NSTAGE).bits;
    valid_o <= stage(NSTAGE).valid;

end architecture;
