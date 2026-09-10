--------------------------------------------------------------------------------
-- predict_16x16_engine.vhd
--
-- H.264 intra 16x16 luma prediction (spec 8.3.3), matching predict_16x16 in
-- src/intra.c including its fallbacks (V without top, H without left, PLANE
-- without top+left+tl all fall back to DC).
--
-- The engine produces one 4x4 block of the 16x16 prediction per cycle,
-- selected by blk_i (raster: bx = blk mod 4, by = blk / 4), so it feeds the
-- 4x4 transform/quant/cost pipeline directly. 16 requests cover an MB.
--
-- Sample buses: sample k at bits (8k+7 downto 8k).
--   top_i : 16 samples   left_i : 16 samples   tl_i : 1   pred_o : 16
--
-- Pipeline: 8 cycles latency (input register + 7 stages), II=1, single global stall. The plane
-- parameters are a long dependent chain (8-term weighted sums, x5, the
-- block offset, then per-sample sums), so they are spread over shallow
-- stages with explicit adder trees:
--   s1  neighbour diffs, DC, effective mode, block row/col samples, a+16
--   s2  H, V weighted sums (balanced trees)
--   s3  b = (5H+32)>>6, c = (5V+32)>>6
--   s4  b+c, 3b, 3c
--   s5  base = a+16 - 7(b+c) + 4*bx*b + 4*by*c
--   s6  row bases  base + {0,c,2c,3c}
--   s7  samples    (rowbase + {0,b,2b,3b}) >> 5, clip; mode mux; output
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity predict_16x16_engine is
    port (
        clk          : in  std_logic;
        rst_n        : in  std_logic;
        mode_i       : in  unsigned(1 downto 0);   -- 0 V, 1 H, 2 DC, 3 PLANE
        blk_i        : in  unsigned(3 downto 0);
        top_i        : in  std_logic_vector(127 downto 0);
        left_i       : in  std_logic_vector(127 downto 0);
        tl_i         : in  std_logic_vector(7 downto 0);
        avail_top_i  : in  std_logic;
        avail_left_i : in  std_logic;
        avail_tl_i   : in  std_logic;
        valid_i      : in  std_logic;
        ready_o      : out std_logic;
        pred_o       : out std_logic_vector(127 downto 0);
        valid_o      : out std_logic;
        ready_i      : in  std_logic
    );
end entity;

architecture rtl of predict_16x16_engine is

    constant NST : integer := 7;

    subtype px_t is unsigned(7 downto 0);
    type px16_t is array (0 to 15) of px_t;
    type px4_t  is array (0 to 3)  of px_t;
    subtype d_t  is signed(9 downto 0);          -- neighbour difference
    type d8_t   is array (0 to 7) of d_t;
    subtype w_t  is signed(17 downto 0);         -- working width
    type w4_t   is array (0 to 3) of w_t;

    function get16(v : std_logic_vector(127 downto 0)) return px16_t is
        variable r : px16_t;
    begin
        for k in 0 to 15 loop r(k) := unsigned(v(8*k+7 downto 8*k)); end loop;
        return r;
    end function;

    function clip8(x : signed) return px_t is
    begin
        if x < 0 then return x"00";
        elsif x > 255 then return x"FF";
        else return unsigned(x(7 downto 0));
        end if;
    end function;

    function sd(a, b : px_t) return d_t is
    begin
        return resize(signed('0' & a), 10) - resize(signed('0' & b), 10);
    end function;

    -- Control that just rides along the pipeline
    type ctl_t is record
        valid : std_logic;
        mode  : unsigned(1 downto 0);
        bx    : unsigned(1 downto 0);
        by    : unsigned(1 downto 0);
        dc    : px_t;
        vcol  : px4_t;
        hrow  : px4_t;
    end record;
    type ctl_arr is array (1 to NST) of ctl_t;
    signal ctl : ctl_arr;

    -- per-stage data
    signal dt1, dl1 : d8_t;                       -- s1
    signal a16_1    : w_t;
    signal a16_2, a16_3, a16_4 : w_t;
    signal H2, V2   : signed(15 downto 0);        -- s2
    signal b3, c3   : signed(13 downto 0);        -- s3
    signal b4, c4, b3x4, c3x4, bc4 : signed(15 downto 0);   -- s4 (b, c, 3b, 3c, b+c)
    signal base5    : w_t;                        -- s5
    signal b5, b2_5, b3_5, c5, c2_5, c3_5 : signed(15 downto 0);
    signal rb6      : w4_t;                       -- s6 row bases
    signal b6, b2_6, b3_6 : signed(15 downto 0);
    signal out_q    : px16_t := (others => (others => '0'));

    signal advance : std_logic;

    -- input register stage: the DC / plane sums start from registers, not
    -- from whatever drives the ports (keeps the s1 adder trees off any
    -- upstream mux path). Adds one cycle of latency (8 total).
    signal top_r, left_r : std_logic_vector(127 downto 0) := (others => '0');
    signal tl_r          : std_logic_vector(7 downto 0) := (others => '0');
    signal mode_r        : unsigned(1 downto 0) := (others => '0');
    signal blk_r         : unsigned(3 downto 0) := (others => '0');
    signal at_r, al_r, atl_r, v_r : std_logic := '0';
    signal st1, sl1 : unsigned(12 downto 0) := (others => '0');
    signal at1, al1 : std_logic := '0';

begin

    advance <= ready_i or not ctl(NST).valid;
    ready_o <= advance;

    process(clk, rst_n)
        variable top, lft : px16_t;
        variable tl       : px_t;
        variable st, sl   : unsigned(12 downto 0);
        variable c1       : ctl_t;
        variable bx, by   : integer range 0 to 3;
        variable m1, m2, m3, m4, n1, n2, n3, n4 : signed(15 downto 0);
        variable bt, ct   : signed(19 downto 0);
        variable bc       : signed(15 downto 0);
        variable mbx, mby : signed(15 downto 0);
        variable v        : w_t;
        variable p        : px16_t;
        variable coloff   : signed(15 downto 0);
    begin
        if rst_n = '0' then
            for k in 1 to NST loop ctl(k).valid <= '0'; end loop;
            v_r <= '0';
        elsif rising_edge(clk) then
            if advance = '1' then
                top_r <= top_i; left_r <= left_i; tl_r <= tl_i;
                mode_r <= mode_i; blk_r <= blk_i;
                at_r <= avail_top_i; al_r <= avail_left_i; atl_r <= avail_tl_i;
                v_r <= valid_i;
                ----------------------------------------------------------
                -- s1
                ----------------------------------------------------------
                top := get16(top_r);
                lft := get16(left_r);
                tl  := unsigned(tl_r);
                bx  := to_integer(blk_r(1 downto 0));
                by  := to_integer(blk_r(3 downto 2));

                -- DC: the two 16-sample sums here, the combine in s2
                st := (others => '0'); sl := (others => '0');
                for k in 0 to 15 loop
                    st := st + resize(top(k), 13);
                    sl := sl + resize(lft(k), 13);
                end loop;
                st1 <= st; sl1 <= sl; at1 <= at_r; al1 <= al_r;
                c1.dc := x"80";

                c1.mode := mode_r;
                if (mode_r = 0 and at_r = '0') or
                   (mode_r = 1 and al_r = '0') or
                   (mode_r = 3 and not (at_r = '1' and al_r = '1'
                                        and atl_r = '1')) then
                    c1.mode := "10";
                end if;
                c1.bx := blk_r(1 downto 0);
                c1.by := blk_r(3 downto 2);
                for k in 0 to 3 loop
                    c1.vcol(k) := top(4 * bx + k);
                    c1.hrow(k) := lft(4 * by + k);
                end loop;
                c1.valid := v_r;
                ctl(1) <= c1;

                for i in 0 to 6 loop
                    dt1(i) <= sd(top(8 + i), top(6 - i));
                    dl1(i) <= sd(lft(8 + i), lft(6 - i));
                end loop;
                dt1(7) <= sd(top(15), tl);
                dl1(7) <= sd(lft(15), tl);
                -- a + 16 = 16*(left[15] + top[15]) + 16
                a16_1 <= resize(shift_left(resize(signed('0' & lft(15)), 18) +
                                           resize(signed('0' & top(15)), 18), 4), 18) + 16;

                ----------------------------------------------------------
                -- s2: H = sum (i+1)*dt(i), balanced tree of shift-adds
                ----------------------------------------------------------
                m1 := resize(dt1(0), 16) + shift_left(resize(dt1(1), 16), 1);                        -- d0 + 2d1
                m2 := resize(dt1(2), 16) + shift_left(resize(dt1(2), 16), 1)
                    + shift_left(resize(dt1(3), 16), 2);                                              -- 3d2 + 4d3
                m3 := resize(dt1(4), 16) + shift_left(resize(dt1(4), 16), 2)
                    + shift_left(resize(dt1(5), 16), 1) + shift_left(resize(dt1(5), 16), 2);        -- 5d4 + 6d5
                m4 := shift_left(resize(dt1(6), 16), 3) - resize(dt1(6), 16)
                    + shift_left(resize(dt1(7), 16), 3);                                              -- 7d6 + 8d7
                n1 := resize(dl1(0), 16) + shift_left(resize(dl1(1), 16), 1);
                n2 := resize(dl1(2), 16) + shift_left(resize(dl1(2), 16), 1)
                    + shift_left(resize(dl1(3), 16), 2);
                n3 := resize(dl1(4), 16) + shift_left(resize(dl1(4), 16), 2)
                    + shift_left(resize(dl1(5), 16), 1) + shift_left(resize(dl1(5), 16), 2);
                n4 := shift_left(resize(dl1(6), 16), 3) - resize(dl1(6), 16)
                    + shift_left(resize(dl1(7), 16), 3);
                H2 <= (m1 + m2) + (m3 + m4);
                V2 <= (n1 + n2) + (n3 + n4);
                a16_2 <= a16_1;
                ctl(2) <= ctl(1);
                if at1 = '1' and al1 = '1' then
                    st := st1 + sl1 + 16;  ctl(2).dc <= st(12 downto 5);
                elsif at1 = '1' then
                    st := st1 + 8;         ctl(2).dc <= st(11 downto 4);
                elsif al1 = '1' then
                    sl := sl1 + 8;         ctl(2).dc <= sl(11 downto 4);
                else
                    ctl(2).dc <= x"80";
                end if;

                ----------------------------------------------------------
                -- s3: b = (5H + 32) >> 6, c likewise
                ----------------------------------------------------------
                bt := shift_right(resize(H2, 20) + shift_left(resize(H2, 20), 2) + 32, 6);
                ct := shift_right(resize(V2, 20) + shift_left(resize(V2, 20), 2) + 32, 6);
                b3 <= bt(13 downto 0);
                c3 <= ct(13 downto 0);
                a16_3 <= a16_2;
                ctl(3) <= ctl(2);

                ----------------------------------------------------------
                -- s4: b+c, 3b, 3c
                ----------------------------------------------------------
                b4   <= resize(b3, 16);
                c4   <= resize(c3, 16);
                b3x4 <= resize(b3, 16) + shift_left(resize(b3, 16), 1);
                c3x4 <= resize(c3, 16) + shift_left(resize(c3, 16), 1);
                bc4  <= resize(b3, 16) + resize(c3, 16);
                a16_4 <= a16_3;
                ctl(4) <= ctl(3);

                ----------------------------------------------------------
                -- s5: base = a+16 - 7(b+c) + 4*bx*b + 4*by*c
                ----------------------------------------------------------
                case ctl(4).bx is
                    when "00"   => mbx := (others => '0');
                    when "01"   => mbx := b4;
                    when "10"   => mbx := shift_left(b4, 1);
                    when others => mbx := b3x4;
                end case;
                case ctl(4).by is
                    when "00"   => mby := (others => '0');
                    when "01"   => mby := c4;
                    when "10"   => mby := shift_left(c4, 1);
                    when others => mby := c3x4;
                end case;
                bc := shift_left(bc4, 3) - bc4;                       -- 7(b+c)
                base5 <= (a16_4 - resize(bc, 18)) +
                         (shift_left(resize(mbx, 18), 2) + shift_left(resize(mby, 18), 2));
                b5 <= b4; b2_5 <= shift_left(b4, 1); b3_5 <= b3x4;
                c5 <= c4; c2_5 <= shift_left(c4, 1); c3_5 <= c3x4;
                ctl(5) <= ctl(4);

                ----------------------------------------------------------
                -- s6: row bases
                ----------------------------------------------------------
                rb6(0) <= base5;
                rb6(1) <= base5 + resize(c5, 18);
                rb6(2) <= base5 + resize(c2_5, 18);
                rb6(3) <= base5 + resize(c3_5, 18);
                b6 <= b5; b2_6 <= b2_5; b3_6 <= b3_5;
                ctl(6) <= ctl(5);

                ----------------------------------------------------------
                -- s7: samples + mode mux
                ----------------------------------------------------------
                for r in 0 to 3 loop
                    for cc in 0 to 3 loop
                        case cc is
                            when 0      => coloff := (others => '0');
                            when 1      => coloff := b6;
                            when 2      => coloff := b2_6;
                            when others => coloff := b3_6;
                        end case;
                        case ctl(6).mode is
                            when "00" => p(r*4+cc) := ctl(6).vcol(cc);
                            when "01" => p(r*4+cc) := ctl(6).hrow(r);
                            when "10" => p(r*4+cc) := ctl(6).dc;
                            when others =>
                                v := rb6(r) + resize(coloff, 18);
                                p(r*4+cc) := clip8(shift_right(v, 5));
                        end case;
                    end loop;
                end loop;
                out_q  <= p;
                ctl(7) <= ctl(6);
            end if;
        end if;
    end process;

    gen_out : for k in 0 to 15 generate
        pred_o(8*k+7 downto 8*k) <= std_logic_vector(out_q(k));
    end generate;
    valid_o <= ctl(NST).valid;

end architecture;
