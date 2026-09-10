--------------------------------------------------------------------------------
-- predict_chroma_engine.vhd
--
-- H.264 intra chroma 8x8 prediction (spec 8.3.4), matching
-- predict_chroma_8x8 in src/intra.c including its fallbacks:
--   H without left  -> flat (avg of top if available else 128)
--   V without top   -> flat (avg of left if available else 128)
--   PLANE without top+left+tl -> the 4-quadrant chroma DC
-- One 4x4 quadrant per cycle, selected by blk_i (bx = blk(0), by = blk(1)).
-- Each chroma plane (U, V) is a separate request with its own neighbours.
--
-- Sample buses: sample k at bits (8k+7 downto 8k).
--   top_i : 8 samples   left_i : 8 samples   tl_i : 1   pred_o : 16
--
-- Pipeline: 6 cycles latency, II=1, single global stall, same staging
-- idea as predict_16x16_engine:
--   s1  diffs, quadrant DC / flat value, effective mode, row/col samples
--   s2  H, V (4-term weighted sums)
--   s3  b = (34H+32)>>6, c likewise
--   s4  base = a+16 - 3(b+c) + 4*bx*b + 4*by*c ; 2b, 3b, 2c, 3c
--   s5  row bases
--   s6  samples, mode mux, output
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity predict_chroma_engine is
    port (
        clk          : in  std_logic;
        rst_n        : in  std_logic;
        mode_i       : in  unsigned(1 downto 0);   -- 0 DC, 1 H, 2 V, 3 PLANE
        blk_i        : in  unsigned(1 downto 0);
        top_i        : in  std_logic_vector(63 downto 0);
        left_i       : in  std_logic_vector(63 downto 0);
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

architecture rtl of predict_chroma_engine is

    constant NST : integer := 6;

    subtype px_t is unsigned(7 downto 0);
    type px16_t is array (0 to 15) of px_t;
    type px8_t  is array (0 to 7)  of px_t;
    type px4_t  is array (0 to 3)  of px_t;
    subtype d_t  is signed(9 downto 0);
    type d4_t   is array (0 to 3) of d_t;
    subtype w_t  is signed(17 downto 0);
    type w4_t   is array (0 to 3) of w_t;

    function get8(v : std_logic_vector(63 downto 0)) return px8_t is
        variable r : px8_t;
    begin
        for k in 0 to 7 loop r(k) := unsigned(v(8*k+7 downto 8*k)); end loop;
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

    function avg4(p : px8_t; base : integer) return px_t is
        variable s : unsigned(10 downto 0);
    begin
        s := resize(p(base),11) + resize(p(base+1),11) + resize(p(base+2),11) + resize(p(base+3),11) + 2;
        return s(9 downto 2);
    end function;

    function avg8(p : px8_t) return px_t is
        variable s : unsigned(11 downto 0);
    begin
        s := (others => '0');
        for k in 0 to 7 loop s := s + resize(p(k), 12); end loop;
        s := s + 4;
        return s(10 downto 3);
    end function;

    type ctl_t is record
        valid : std_logic;
        mode  : unsigned(1 downto 0);   -- effective: 0 flat/DC, 1 H, 2 V, 3 PLANE
        bx    : std_logic;
        by    : std_logic;
        dc    : px_t;
        vcol  : px4_t;
        hrow  : px4_t;
    end record;
    type ctl_arr is array (1 to NST) of ctl_t;
    signal ctl : ctl_arr;

    signal dt1, dl1 : d4_t;
    signal a16_1, a16_2, a16_3 : w_t;
    signal H2, V2   : signed(13 downto 0);
    signal b3, c3   : signed(13 downto 0);
    signal base4    : w_t;
    signal b4, b2_4, b3_4, c4, c2_4, c3_4 : signed(15 downto 0);
    signal rb5      : w4_t;
    signal b5, b2_5, b3_5 : signed(15 downto 0);
    signal out_q    : px16_t := (others => (others => '0'));

    signal advance : std_logic;

begin

    advance <= ready_i or not ctl(NST).valid;
    ready_o <= advance;

    process(clk, rst_n)
        variable top, lft : px8_t;
        variable tl       : px_t;
        variable dcq      : px4_t;
        variable s        : unsigned(11 downto 0);
        variable c1       : ctl_t;
        variable at, al   : boolean;
        variable bx, by   : integer range 0 to 1;
        variable bt, ct   : signed(21 downto 0);
        variable b16, c16, bc, mbx, mby, b3x, c3x : signed(15 downto 0);
        variable v        : w_t;
        variable p        : px16_t;
        variable coloff   : signed(15 downto 0);
    begin
        if rst_n = '0' then
            for k in 1 to NST loop ctl(k).valid <= '0'; end loop;
        elsif rising_edge(clk) then
            if advance = '1' then
                ----------------------------------------------------------
                -- s1
                ----------------------------------------------------------
                top := get8(top_i);
                lft := get8(left_i);
                tl  := unsigned(tl_i);
                bx  := to_integer(blk_i(0 downto 0));
                by  := to_integer(blk_i(1 downto 1));
                at  := (avail_top_i = '1');
                al  := (avail_left_i = '1');

                if at and al then
                    s := (others => '0');
                    for k in 0 to 3 loop s := s + resize(top(k), 12) + resize(lft(k), 12); end loop;
                    s := s + 4; dcq(0) := s(10 downto 3);
                elsif at then dcq(0) := avg4(top, 0);
                elsif al then dcq(0) := avg4(lft, 0);
                else          dcq(0) := x"80";
                end if;
                if at then    dcq(1) := avg4(top, 4);
                elsif al then dcq(1) := avg4(lft, 0);
                else          dcq(1) := x"80";
                end if;
                if al then    dcq(2) := avg4(lft, 4);
                elsif at then dcq(2) := avg4(top, 0);
                else          dcq(2) := x"80";
                end if;
                if at and al then
                    s := (others => '0');
                    for k in 4 to 7 loop s := s + resize(top(k), 12) + resize(lft(k), 12); end loop;
                    s := s + 4; dcq(3) := s(10 downto 3);
                elsif at then dcq(3) := avg4(top, 4);
                elsif al then dcq(3) := avg4(lft, 4);
                else          dcq(3) := x"80";
                end if;

                c1.mode := mode_i;
                c1.dc   := dcq(by * 2 + bx);
                if mode_i = 1 and not al then
                    c1.mode := "00";
                    if at then c1.dc := avg8(top); else c1.dc := x"80"; end if;
                elsif mode_i = 2 and not at then
                    c1.mode := "00";
                    if al then c1.dc := avg8(lft); else c1.dc := x"80"; end if;
                elsif mode_i = 3 and not (at and al and avail_tl_i = '1') then
                    c1.mode := "00";
                end if;
                c1.bx := blk_i(0);
                c1.by := blk_i(1);
                for k in 0 to 3 loop
                    c1.vcol(k) := top(4 * bx + k);
                    c1.hrow(k) := lft(4 * by + k);
                end loop;
                c1.valid := valid_i;
                ctl(1) <= c1;

                for i in 0 to 2 loop
                    dt1(i) <= sd(top(4 + i), top(2 - i));
                    dl1(i) <= sd(lft(4 + i), lft(2 - i));
                end loop;
                dt1(3) <= sd(top(7), tl);
                dl1(3) <= sd(lft(7), tl);
                a16_1 <= resize(shift_left(resize(signed('0' & lft(7)), 18) +
                                           resize(signed('0' & top(7)), 18), 4), 18) + 16;

                ----------------------------------------------------------
                -- s2: H = d0 + 2d1 + 3d2 + 4d3
                ----------------------------------------------------------
                H2 <= (resize(dt1(0), 14) + shift_left(resize(dt1(1), 14), 1))
                    + (resize(dt1(2), 14) + shift_left(resize(dt1(2), 14), 1)
                       + shift_left(resize(dt1(3), 14), 2));
                V2 <= (resize(dl1(0), 14) + shift_left(resize(dl1(1), 14), 1))
                    + (resize(dl1(2), 14) + shift_left(resize(dl1(2), 14), 1)
                       + shift_left(resize(dl1(3), 14), 2));
                a16_2 <= a16_1;
                ctl(2) <= ctl(1);

                ----------------------------------------------------------
                -- s3: b = (34H + 32) >> 6 = ((H<<5) + (H<<1) + 32) >> 6
                ----------------------------------------------------------
                bt := shift_right(shift_left(resize(H2, 22), 5) + shift_left(resize(H2, 22), 1) + 32, 6);
                ct := shift_right(shift_left(resize(V2, 22), 5) + shift_left(resize(V2, 22), 1) + 32, 6);
                b3 <= bt(13 downto 0);
                c3 <= ct(13 downto 0);
                a16_3 <= a16_2;
                ctl(3) <= ctl(2);

                ----------------------------------------------------------
                -- s4: base = a+16 - 3(b+c) + 4*bx*b + 4*by*c
                ----------------------------------------------------------
                b16 := resize(b3, 16);
                c16 := resize(c3, 16);
                bc  := b16 + c16;
                bc  := bc + shift_left(bc, 1);                        -- 3(b+c)
                if ctl(3).bx = '1' then mbx := shift_left(b16, 2); else mbx := (others => '0'); end if;
                if ctl(3).by = '1' then mby := shift_left(c16, 2); else mby := (others => '0'); end if;
                base4 <= (a16_3 - resize(bc, 18)) + (resize(mbx, 18) + resize(mby, 18));
                b3x := b16 + shift_left(b16, 1);
                c3x := c16 + shift_left(c16, 1);
                b4 <= b16; b2_4 <= shift_left(b16, 1); b3_4 <= b3x;
                c4 <= c16; c2_4 <= shift_left(c16, 1); c3_4 <= c3x;
                ctl(4) <= ctl(3);

                ----------------------------------------------------------
                -- s5: row bases
                ----------------------------------------------------------
                rb5(0) <= base4;
                rb5(1) <= base4 + resize(c4, 18);
                rb5(2) <= base4 + resize(c2_4, 18);
                rb5(3) <= base4 + resize(c3_4, 18);
                b5 <= b4; b2_5 <= b2_4; b3_5 <= b3_4;
                ctl(5) <= ctl(4);

                ----------------------------------------------------------
                -- s6: samples + mode mux
                ----------------------------------------------------------
                for r in 0 to 3 loop
                    for cc in 0 to 3 loop
                        case cc is
                            when 0      => coloff := (others => '0');
                            when 1      => coloff := b5;
                            when 2      => coloff := b2_5;
                            when others => coloff := b3_5;
                        end case;
                        case ctl(5).mode is
                            when "00" => p(r*4+cc) := ctl(5).dc;
                            when "01" => p(r*4+cc) := ctl(5).hrow(r);
                            when "10" => p(r*4+cc) := ctl(5).vcol(cc);
                            when others =>
                                v := rb5(r) + resize(coloff, 18);
                                p(r*4+cc) := clip8(shift_right(v, 5));
                        end case;
                    end loop;
                end loop;
                out_q  <= p;
                ctl(6) <= ctl(5);
            end if;
        end if;
    end process;

    gen_out : for k in 0 to 15 generate
        pred_o(8*k+7 downto 8*k) <= std_logic_vector(out_q(k));
    end generate;
    valid_o <= ctl(NST).valid;

end architecture;
