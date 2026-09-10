--------------------------------------------------------------------------------
-- quant_engine.vhd
--
-- H.264 forward / inverse quantizer, 16 lanes, one 4x4 block per cycle.
--
--   Mode 0: quant_4x4       AC forward   level = (|c|*MF + f) >> (15+qdiv)
--   Mode 1: iquant_4x4      AC inverse   coef  = l*V << qdiv
--   Mode 2: quant_dc_4x4    luma DC fwd  >> (15+qdiv+2), MF/V of position 0
--   Mode 3: iquant_dc_4x4   luma DC inv  << (qdiv-2), or (+round) >> (2-qdiv)
--   Mode 4: quant_dc_2x2    chroma DC fwd >> (15+qdiv+1)  (lanes 0..3)
--   Mode 5: iquant_dc_2x2   chroma DC inv << (qdiv-1), or >> 1 (lanes 0..3)
--   is_intra is always 1 (rounding offset f = (1<<qbits)/3, from a ROM).
--
-- Pipeline (4 cycles latency, II=1, single global stall):
--   stage 0  operand a (signed coefficient or level); qp/mode decode.
--   stage 1  operand b (MF or V<<ls) and per-lane operand c from ROMs.
--   stage 2  one signed multiply-add per lane, a*b + c  -> DSP48E1.
--   stage 3  arithmetic right shift (two-level mux for the forward path,
--            0..2 for inverse), output register.
--   stage 4  fused AC dequantisation of the forward level (mode 0 only):
--            deq = level * (V << qdiv), exactly what mode 1 would return
--            for that level, one cycle after dout (deq_valid_o). Lets a
--            closed-loop evaluation skip the inverse pass.
--
-- Sign handling without |c| and without a final negate: the C reference
-- computes sign(c) * ((|c|*MF + f) >> q). For c < 0 that equals
-- floor((c*MF - f + 2^q - 1) / 2^q), so the forward path multiplies the
-- SIGNED coefficient and adds either f (c >= 0) or 2^q - 1 - f (c < 0),
-- then does one arithmetic shift. Both constants come from a ROM.
--
-- Area notes:
--   - One multiplier per lane serves both directions (operands muxed).
--   - The inverse left shift (<< qdiv etc.) is folded into the multiplier
--     constant: the ROM holds V << ls, so no barrel shifter on that path.
--   - Widths follow the encoder's real ranges: forward inputs |c| < 2^17
--     (luma DC Hadamard max 65280), levels within 16 bits.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity quant_engine is
    port (
        clk     : in  std_logic;
        rst_n   : in  std_logic;
        mode_i  : in  unsigned(2 downto 0);
        qp_i    : in  unsigned(5 downto 0);
        din_0   : in  signed(31 downto 0);
        din_1   : in  signed(31 downto 0);
        din_2   : in  signed(31 downto 0);
        din_3   : in  signed(31 downto 0);
        din_4   : in  signed(31 downto 0);
        din_5   : in  signed(31 downto 0);
        din_6   : in  signed(31 downto 0);
        din_7   : in  signed(31 downto 0);
        din_8   : in  signed(31 downto 0);
        din_9   : in  signed(31 downto 0);
        din_10  : in  signed(31 downto 0);
        din_11  : in  signed(31 downto 0);
        din_12  : in  signed(31 downto 0);
        din_13  : in  signed(31 downto 0);
        din_14  : in  signed(31 downto 0);
        din_15  : in  signed(31 downto 0);
        valid_i : in  std_logic;
        ready_o : out std_logic;
        dout_0  : out signed(31 downto 0);
        dout_1  : out signed(31 downto 0);
        dout_2  : out signed(31 downto 0);
        dout_3  : out signed(31 downto 0);
        dout_4  : out signed(31 downto 0);
        dout_5  : out signed(31 downto 0);
        dout_6  : out signed(31 downto 0);
        dout_7  : out signed(31 downto 0);
        dout_8  : out signed(31 downto 0);
        dout_9  : out signed(31 downto 0);
        dout_10 : out signed(31 downto 0);
        dout_11 : out signed(31 downto 0);
        dout_12 : out signed(31 downto 0);
        dout_13 : out signed(31 downto 0);
        dout_14 : out signed(31 downto 0);
        dout_15 : out signed(31 downto 0);
        valid_o : out std_logic;
        ready_i : in  std_logic;
        -- fused dequantised block (forward AC mode), latency 5
        deq_0   : out signed(31 downto 0);
        deq_1   : out signed(31 downto 0);
        deq_2   : out signed(31 downto 0);
        deq_3   : out signed(31 downto 0);
        deq_4   : out signed(31 downto 0);
        deq_5   : out signed(31 downto 0);
        deq_6   : out signed(31 downto 0);
        deq_7   : out signed(31 downto 0);
        deq_8   : out signed(31 downto 0);
        deq_9   : out signed(31 downto 0);
        deq_10  : out signed(31 downto 0);
        deq_11  : out signed(31 downto 0);
        deq_12  : out signed(31 downto 0);
        deq_13  : out signed(31 downto 0);
        deq_14  : out signed(31 downto 0);
        deq_15  : out signed(31 downto 0);
        deq_valid_o : out std_logic
    );
end entity;

architecture rtl of quant_engine is

    constant AW : integer := 18;   -- operand a (signed coef within +/-2^17, or 16-bit level)
    constant BW : integer := 15;   -- operand b (MF up to 13107, or V<<8 up to 7424)
    constant CW : integer := 26;   -- operand c (up to 22369621)
    constant PW : integer := AW + BW + 1;

    type a_arr is array (0 to 15) of signed(AW - 1 downto 0);
    type b_arr is array (0 to 15) of signed(BW - 1 downto 0);
    type c_arr is array (0 to 15) of signed(CW - 1 downto 0);
    type p_arr is array (0 to 15) of signed(PW - 1 downto 0);
    type o_arr is array (0 to 15) of signed(31 downto 0);

    -- Spec 8.5.9 tables by (qp mod 6, position class):
    --   class 0: (row even, col even)   class 2: (row odd, col odd)   class 1: else
    type tab_t is array (0 to 5, 0 to 2) of integer;
    constant MF_TAB : tab_t := (
        (13107, 8066, 5243), (11916, 7490, 4660), (10082, 6554, 4194),
        ( 9362, 5825, 3647), ( 8192, 5243, 3355), ( 7282, 4559, 2893));
    constant V_TAB : tab_t := (
        (10, 13, 16), (11, 14, 18), (13, 16, 20),
        (14, 18, 23), (16, 20, 25), (18, 23, 29));

    function pos_class(i : integer) return integer is
        variable r, c : integer;
    begin
        r := i / 4; c := i mod 4;
        if (r mod 2 = 0) and (c mod 2 = 0) then return 0;
        elsif (r mod 2 = 1) and (c mod 2 = 1) then return 2;
        else return 1;
        end if;
    end function;

    -- f = (1 << qbits) / 3 for qbits 15..25, and f_neg = 2^qbits - 1 - f
    function f_rom(qbits : integer; neg : boolean) return integer is
        variable f : integer;
    begin
        case qbits is
            when 15 => f := 10922;
            when 16 => f := 21845;
            when 17 => f := 43690;
            when 18 => f := 87381;
            when 19 => f := 174762;
            when 20 => f := 349525;
            when 21 => f := 699050;
            when 22 => f := 1398101;
            when 23 => f := 2796202;
            when 24 => f := 5592405;
            when others => f := 11184810;
        end case;
        if neg then
            return (2 ** qbits) - 1 - f;
        else
            return f;
        end if;
    end function;

    -- Stage 0 registers
    signal a0_q    : a_arr;
    signal neg0_q  : std_logic_vector(15 downto 0);
    signal qdiv0_q : integer range 0 to 10;
    signal qmod0_q : integer range 0 to 5;
    signal dcadd0_q : integer range 0 to 2;
    signal ls0_q   : integer range 0 to 8;
    signal rs0_q   : integer range 0 to 2;
    signal rnd0_q  : integer range 0 to 2;
    signal fwd0_q  : std_logic;
    signal four0_q : std_logic;
    signal v0_q    : std_logic;
    -- Stage 1 registers
    signal a1_q    : a_arr;
    signal b1_q    : b_arr;
    signal c1_q    : c_arr;
    signal sh1_q   : unsigned(3 downto 0);
    signal rs1_q   : unsigned(1 downto 0);
    signal fwd1_q  : std_logic;
    signal four1_q : std_logic;
    signal v1_q    : std_logic;
    -- Stage 2 registers
    signal p_q     : p_arr;
    signal sh2_q   : unsigned(3 downto 0);
    signal rs2_q   : unsigned(1 downto 0);
    signal fwd2_q  : std_logic;
    signal four2_q : std_logic;
    signal v2_q    : std_logic;
    -- Stage 3 registers
    signal out_q   : o_arr;
    signal v3_q    : std_logic;
    signal qmod1_q, qmod2_q, qmod3_q : integer range 0 to 5;
    signal qdiv1_q, qdiv2_q, qdiv3_q : integer range 0 to 10;
    signal ac0_q, ac1_q, ac2_q, ac3_q : std_logic;   -- forward AC mode flag, aligned with v0..v3
    -- Stage 4 registers (fused dequant)
    type d_arr is array (0 to 15) of signed(31 downto 0);
    signal deq_q   : d_arr;
    signal v4_q    : std_logic;

    attribute use_dsp : string;
    attribute use_dsp of p_q : signal is "yes";
    attribute use_dsp of deq_q : signal is "yes";

    signal advance : std_logic;

begin

    advance <= ready_i or not v3_q;
    ready_o <= advance;

    process(clk, rst_n)
        variable din   : o_arr;
        variable qp    : integer range 0 to 63;
        variable qdiv  : integer range 0 to 10;
        variable ls    : integer range 0 to 8;
        variable rs    : integer range 0 to 2;
        variable rnd   : integer range 0 to 2;
        variable cls   : integer range 0 to 2;
        variable t, t1 : signed(PW - 1 downto 0);
        variable vb    : unsigned(BW - 1 downto 0);
        variable qb    : integer range 15 to 25;
    begin
        if rst_n = '0' then
            v0_q <= '0'; v1_q <= '0'; v2_q <= '0'; v3_q <= '0';
        elsif rising_edge(clk) then
            if advance = '1' then
                ----------------------------------------------------------
                -- Stage 0: operands a + control decode
                ----------------------------------------------------------
                din(0)  := din_0;   din(1)  := din_1;   din(2)  := din_2;   din(3)  := din_3;
                din(4)  := din_4;   din(5)  := din_5;   din(6)  := din_6;   din(7)  := din_7;
                din(8)  := din_8;   din(9)  := din_9;   din(10) := din_10;  din(11) := din_11;
                din(12) := din_12;  din(13) := din_13;  din(14) := din_14;  din(15) := din_15;

                qp   := to_integer(qp_i);
                qdiv := qp / 6;
                qdiv0_q <= qdiv;
                qmod0_q <= qp mod 6;
                case mode_i is
                    when "010" | "011" => dcadd0_q <= 2;
                    when "100" | "101" => dcadd0_q <= 1;
                    when others        => dcadd0_q <= 0;
                end case;
                ls := 0; rs := 0; rnd := 0;
                case mode_i is
                    when "001" => ls := qdiv;
                    when "011" =>
                        if qdiv >= 2 then ls := qdiv - 2;
                        else rs := 2 - qdiv; rnd := 2 ** (rs - 1); end if;
                    when "101" =>
                        if qdiv >= 1 then ls := qdiv - 1;
                        else rs := 1; end if;
                    when others => null;
                end case;
                ls0_q <= ls; rs0_q <= rs; rnd0_q <= rnd;
                fwd0_q  <= not mode_i(0);
                four0_q <= mode_i(2);
                if mode_i = "000" then ac0_q <= valid_i; else ac0_q <= '0'; end if;
                for i in 0 to 15 loop
                    if mode_i(0) = '0' then
                        a0_q(i) <= din(i)(AW - 1 downto 0);
                    else
                        a0_q(i) <= resize(din(i)(15 downto 0), AW);
                    end if;
                    neg0_q(i) <= din(i)(31);
                end loop;
                v0_q <= valid_i;

                ----------------------------------------------------------
                -- Stage 1: operand ROMs
                ----------------------------------------------------------
                qb := 15 + qdiv0_q + dcadd0_q;
                for i in 0 to 15 loop
                    if dcadd0_q /= 0 then cls := 0; else cls := pos_class(i); end if;
                    if fwd0_q = '1' then
                        b1_q(i) <= to_signed(MF_TAB(qmod0_q, cls), BW);
                        c1_q(i) <= to_signed(f_rom(qb, neg0_q(i) = '1'), CW);
                    else
                        vb := shift_left(to_unsigned(V_TAB(qmod0_q, cls), BW), ls0_q);
                        b1_q(i) <= signed(vb);
                        c1_q(i) <= to_signed(rnd0_q, CW);
                    end if;
                end loop;
                a1_q   <= a0_q;
                qmod1_q <= qmod0_q; qdiv1_q <= qdiv0_q; ac1_q <= ac0_q;
                sh1_q  <= to_unsigned(qdiv0_q + dcadd0_q, 4);
                rs1_q  <= to_unsigned(rs0_q, 2);
                fwd1_q <= fwd0_q; four1_q <= four0_q;
                v1_q   <= v0_q;

                ----------------------------------------------------------
                -- Stage 2: multiply-add (DSP, post-adder)
                ----------------------------------------------------------
                for i in 0 to 15 loop
                    p_q(i) <= resize(a1_q(i) * b1_q(i) + resize(c1_q(i), PW - 1), PW);
                end loop;
                sh2_q <= sh1_q; rs2_q <= rs1_q; fwd2_q <= fwd1_q; four2_q <= four1_q;
                qmod2_q <= qmod1_q; qdiv2_q <= qdiv1_q; ac2_q <= ac1_q;
                v2_q <= v1_q;

                ----------------------------------------------------------
                -- Stage 3: shift, output
                ----------------------------------------------------------
                for i in 0 to 15 loop
                    if fwd2_q = '1' then
                        t := shift_right(p_q(i), 15);
                        -- two-level shifter: 0..3 then 0/4/8
                        case sh2_q(1 downto 0) is
                            when "00"   => t1 := t;
                            when "01"   => t1 := shift_right(t, 1);
                            when "10"   => t1 := shift_right(t, 2);
                            when others => t1 := shift_right(t, 3);
                        end case;
                        case sh2_q(3 downto 2) is
                            when "00"   => t := t1;
                            when "01"   => t := shift_right(t1, 4);
                            when others => t := shift_right(t1, 8);
                        end case;
                        out_q(i) <= resize(t(15 downto 0), 32);
                    else
                        t := shift_right(p_q(i), to_integer(rs2_q));
                        out_q(i) <= resize(t, 32);
                    end if;
                    if four2_q = '1' and i >= 4 then
                        out_q(i) <= (others => '0');
                    end if;
                end loop;
                v3_q <= v2_q;
                qmod3_q <= qmod2_q; qdiv3_q <= qdiv2_q; ac3_q <= ac2_q;

                ----------------------------------------------------------
                -- Stage 4: fused dequantisation of the forward AC level
                ----------------------------------------------------------
                for i in 0 to 15 loop
                    vb := shift_left(to_unsigned(V_TAB(qmod3_q, pos_class(i)), BW), qdiv3_q);
                    deq_q(i) <= resize(out_q(i)(15 downto 0) * signed(vb), 32);
                end loop;
                v4_q <= ac3_q;
            end if;
        end if;
    end process;

    deq_0  <= deq_q(0);   deq_1  <= deq_q(1);   deq_2  <= deq_q(2);   deq_3  <= deq_q(3);
    deq_4  <= deq_q(4);   deq_5  <= deq_q(5);   deq_6  <= deq_q(6);   deq_7  <= deq_q(7);
    deq_8  <= deq_q(8);   deq_9  <= deq_q(9);   deq_10 <= deq_q(10);  deq_11 <= deq_q(11);
    deq_12 <= deq_q(12);  deq_13 <= deq_q(13);  deq_14 <= deq_q(14);  deq_15 <= deq_q(15);
    deq_valid_o <= v4_q;

    dout_0  <= out_q(0);   dout_1  <= out_q(1);   dout_2  <= out_q(2);   dout_3  <= out_q(3);
    dout_4  <= out_q(4);   dout_5  <= out_q(5);   dout_6  <= out_q(6);   dout_7  <= out_q(7);
    dout_8  <= out_q(8);   dout_9  <= out_q(9);   dout_10 <= out_q(10);  dout_11 <= out_q(11);
    dout_12 <= out_q(12);  dout_13 <= out_q(13);  dout_14 <= out_q(14);  dout_15 <= out_q(15);
    valid_o <= v3_q;

end architecture;
