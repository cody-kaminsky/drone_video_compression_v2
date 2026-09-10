--------------------------------------------------------------------------------
-- predict_4x4_engine.vhd
--
-- H.264 intra 4x4 prediction, all 9 modes (spec 8.3.1.2). Matches
-- predict_4x4 in src/intra.c: unavailable neighbours are replaced by 128,
-- top-right replication (top[4..7]) is the caller's job.
--
-- Sample buses: sample k occupies bits (8k+7 downto 8k).
--   top_i  : 8 samples A..H     left_i : 4 samples I..L     tl_i : X
--   pred_o : 16 samples, raster (row*4 + col)
--
-- One block per cycle, one cycle latency, registered output. The 2-tap and
-- 3-tap averages are computed once per unique neighbour triple and shared
-- across modes; the mode only selects.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity predict_4x4_engine is
    port (
        clk          : in  std_logic;
        rst_n        : in  std_logic;
        mode_i       : in  unsigned(3 downto 0);
        top_i        : in  std_logic_vector(63 downto 0);
        left_i       : in  std_logic_vector(31 downto 0);
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

architecture rtl of predict_4x4_engine is

    subtype px_t is unsigned(7 downto 0);
    type pred_t is array (0 to 15) of px_t;

    -- (a + b + 1) >> 1
    function avg2(a, b : px_t) return px_t is
        variable s : unsigned(9 downto 0);
    begin
        s := resize(a, 10) + resize(b, 10) + 1;
        return s(8 downto 1);
    end function;

    -- (a + 2b + c + 2) >> 2
    function avg3(a, b, c : px_t) return px_t is
        variable s : unsigned(10 downto 0);
    begin
        s := resize(a, 11) + resize(b, 11) + resize(b, 11) + resize(c, 11) + 2;
        return s(9 downto 2);
    end function;

    signal out_q     : pred_t := (others => (others => '0'));
    signal out_valid : std_logic := '0';
    signal advance   : std_logic;

begin

    advance <= ready_i or not out_valid;
    ready_o <= advance;

    process(clk, rst_n)
        variable A, B, C, D, E, F, G, H, I, J, K, L, X : px_t;
        variable p : pred_t;
        variable dc : px_t;
        variable s, sT, sL : unsigned(11 downto 0);
        -- shared kernels
        variable xAB, ABC, BCD, CDE, DEF, EFG, FGH, GHH : px_t;  -- 3-tap along top
        variable IXA, XIJ, IJK, JKL, KLL                : px_t;  -- 3-tap around corner/left
        variable hXA, hAB, hBC, hCD, hDE, hEF           : px_t;  -- 2-tap top
        variable hXI, hIJ, hJK, hKL                     : px_t;  -- 2-tap left
    begin
        if rst_n = '0' then
            out_valid <= '0';
        elsif rising_edge(clk) then
            if advance = '1' then
                if avail_top_i = '1' then
                    A := unsigned(top_i(7 downto 0));   B := unsigned(top_i(15 downto 8));
                    C := unsigned(top_i(23 downto 16)); D := unsigned(top_i(31 downto 24));
                    E := unsigned(top_i(39 downto 32)); F := unsigned(top_i(47 downto 40));
                    G := unsigned(top_i(55 downto 48)); H := unsigned(top_i(63 downto 56));
                else
                    A := x"80"; B := x"80"; C := x"80"; D := x"80";
                    E := x"80"; F := x"80"; G := x"80"; H := x"80";
                end if;
                if avail_left_i = '1' then
                    I := unsigned(left_i(7 downto 0));   J := unsigned(left_i(15 downto 8));
                    K := unsigned(left_i(23 downto 16)); L := unsigned(left_i(31 downto 24));
                else
                    I := x"80"; J := x"80"; K := x"80"; L := x"80";
                end if;
                if avail_tl_i = '1' then X := unsigned(tl_i); else X := x"80"; end if;

                -- DC (sums as balanced trees to keep the path short)
                sT := (resize(A,12) + resize(B,12)) + (resize(C,12) + resize(D,12));
                sL := (resize(I,12) + resize(J,12)) + (resize(K,12) + resize(L,12));
                if avail_top_i = '1' and avail_left_i = '1' then
                    s  := (sT + sL) + 4;
                    dc := s(10 downto 3);
                elsif avail_top_i = '1' then
                    s  := sT + 2;
                    dc := s(9 downto 2);
                elsif avail_left_i = '1' then
                    s  := sL + 2;
                    dc := s(9 downto 2);
                else
                    dc := x"80";
                end if;

                -- Shared kernels
                xAB := avg3(X, A, B);  ABC := avg3(A, B, C);  BCD := avg3(B, C, D);
                CDE := avg3(C, D, E);  DEF := avg3(D, E, F);  EFG := avg3(E, F, G);
                FGH := avg3(F, G, H);  GHH := avg3(G, H, H);
                IXA := avg3(I, X, A);  XIJ := avg3(X, I, J);  IJK := avg3(I, J, K);
                JKL := avg3(J, K, L);  KLL := avg3(K, L, L);
                hXA := avg2(X, A); hAB := avg2(A, B); hBC := avg2(B, C);
                hCD := avg2(C, D); hDE := avg2(D, E); hEF := avg2(E, F);
                hXI := avg2(X, I); hIJ := avg2(I, J); hJK := avg2(J, K); hKL := avg2(K, L);

                p := (others => dc);
                case to_integer(mode_i) is
                    when 0 =>  -- VERTICAL
                        for r in 0 to 3 loop
                            p(r*4+0) := A; p(r*4+1) := B; p(r*4+2) := C; p(r*4+3) := D;
                        end loop;
                    when 1 =>  -- HORIZONTAL
                        for c in 0 to 3 loop
                            p(0*4+c) := I; p(1*4+c) := J; p(2*4+c) := K; p(3*4+c) := L;
                        end loop;
                    when 2 =>  -- DC
                        null;
                    when 3 =>  -- DIAG_DOWN_LEFT
                        p(0) := ABC;
                        p(1) := BCD; p(4) := BCD;
                        p(2) := CDE; p(5) := CDE; p(8) := CDE;
                        p(3) := DEF; p(6) := DEF; p(9) := DEF; p(12) := DEF;
                        p(7) := EFG; p(10) := EFG; p(13) := EFG;
                        p(11) := FGH; p(14) := FGH;
                        p(15) := GHH;
                    when 4 =>  -- DIAG_DOWN_RIGHT
                        p(12) := JKL;
                        p(8) := IJK; p(13) := IJK;
                        p(4) := XIJ; p(9) := XIJ; p(14) := XIJ;
                        p(0) := IXA; p(5) := IXA; p(10) := IXA; p(15) := IXA;
                        p(1) := xAB; p(6) := xAB; p(11) := xAB;
                        p(2) := ABC; p(7) := ABC;
                        p(3) := BCD;
                    when 5 =>  -- VERTICAL_RIGHT
                        p(0) := hXA; p(9)  := hXA;
                        p(1) := hAB; p(10) := hAB;
                        p(2) := hBC; p(11) := hBC;
                        p(3) := hCD;
                        p(4) := IXA; p(13) := IXA;
                        p(5) := xAB; p(14) := xAB;
                        p(6) := ABC; p(15) := ABC;
                        p(7) := BCD;
                        p(8) := XIJ;
                        p(12) := IJK;
                    when 6 =>  -- HORIZONTAL_DOWN
                        p(0) := hXI; p(6)  := hXI;
                        p(4) := hIJ; p(10) := hIJ;
                        p(8) := hJK; p(14) := hJK;
                        p(12) := hKL;
                        p(1) := IXA; p(7) := IXA;
                        p(2) := xAB;
                        p(3) := ABC;
                        p(5) := XIJ; p(11) := XIJ;
                        p(9) := IJK; p(15) := IJK;
                        p(13) := JKL;
                    when 7 =>  -- VERTICAL_LEFT
                        p(0) := hAB;
                        p(1) := hBC; p(8)  := hBC;
                        p(2) := hCD; p(9)  := hCD;
                        p(3) := hDE; p(10) := hDE;
                        p(4) := ABC;
                        p(5) := BCD; p(12) := BCD;
                        p(6) := CDE; p(13) := CDE;
                        p(7) := DEF; p(14) := DEF;
                        p(11) := hEF;
                        p(15) := EFG;
                    when others =>  -- 8: HORIZONTAL_UP
                        p(0) := hIJ;
                        p(1) := IJK;
                        p(2) := hJK; p(4) := hJK;
                        p(3) := JKL; p(5) := JKL;
                        p(6) := hKL; p(8) := hKL;
                        p(7) := KLL; p(9) := KLL;
                        p(10) := L; p(11) := L;
                        p(12) := L; p(13) := L; p(14) := L; p(15) := L;
                end case;

                out_q     <= p;
                out_valid <= valid_i;
            end if;
        end if;
    end process;

    gen_out : for k in 0 to 15 generate
        pred_o(8*k+7 downto 8*k) <= std_logic_vector(out_q(k));
    end generate;
    valid_o <= out_valid;

end architecture;
