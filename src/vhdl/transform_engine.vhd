--------------------------------------------------------------------------------
-- transform_engine.vhd
--
-- H.264 4x4 integer transform engine (single-cycle, area-optimized).
--
--   Mode 0: dct4x4          (forward DCT,  spec 8.5.6)
--   Mode 1: idct4x4         (inverse DCT,  spec 8.5.6)
--   Mode 2: hadamard4x4     (forward Hadamard, DC luma)
--   Mode 3: ihadamard4x4    (inverse Hadamard, DC luma)
--   Mode 4: hadamard2x2     (forward Hadamard, DC chroma)
--   Mode 5: ihadamard2x2    (inverse Hadamard, DC chroma)
--
-- Architecture: 8 instances of one shared mode-muxed butterfly (4 row +
-- 4 column).  Single-cycle combinational core with registered output.
-- One block per cycle throughput, one cycle latency.
-- Pure shift-and-add — no DSPs.
--
-- Data layout: 16-element row-major array, matching the C reference.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity transform_engine is
    port (
        clk     : in  std_logic;
        rst_n   : in  std_logic;
        -- Input side
        mode_i  : in  unsigned(2 downto 0);
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
        -- Output side
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
        ready_i : in  std_logic
    );
end entity;

architecture rtl of transform_engine is

    -- Internal width: 21 bits covers worst-case intermediate
    -- (forward DCT: 4 * 2 * max_i16 = 4*2*32767 = 262136, fits in 19 bits,
    --  column pass doubles again: fits in 21 bits before i16 truncation).
    -- Inverse transforms need full 32 bits at input but intermediates
    -- still fit in 32. We use 32 throughout for correctness on inverse
    -- paths; the synthesizer trims unused MSBs on forward-only paths.
    subtype elem_t is signed(31 downto 0);
    type block_t is array (0 to 15) of elem_t;

    constant MODE_DCT4  : unsigned(2 downto 0) := "000";
    constant MODE_IDCT4 : unsigned(2 downto 0) := "001";
    constant MODE_HAD2  : unsigned(2 downto 0) := "100";

    signal result_comb : block_t;
    signal out_reg     : block_t := (others => (others => '0'));
    signal out_valid   : std_logic := '0';
    signal accept      : std_logic;

    -- Shared butterfly: one implementation, mode-selected
    procedure butterfly(
        mode                 : in  unsigned(2 downto 0);
        r0, r1, r2, r3      : in  elem_t;
        o0, o1, o2, o3      : out elem_t) is
        variable a, b, c, d : elem_t;
    begin
        if mode = MODE_IDCT4 then
            -- Inverse DCT butterfly
            a := r0 + r2;
            b := r0 - r2;
            c := shift_right(r1, 1) - r3;
            d := r1 + shift_right(r3, 1);
            o0 := a + d;
            o1 := b + c;
            o2 := b - c;
            o3 := a - d;
        elsif mode = MODE_DCT4 then
            -- Forward DCT butterfly
            a := r0 + r3;  b := r1 + r2;
            c := r1 - r2;  d := r0 - r3;
            o0 := a + b;
            o1 := shift_left(d, 1) + c;
            o2 := a - b;
            o3 := d - shift_left(c, 1);
        else
            -- Hadamard butterfly (forward = inverse)
            a := r0 + r3;  b := r1 + r2;
            c := r1 - r2;  d := r0 - r3;
            o0 := a + b;
            o1 := d + c;
            o2 := a - b;
            o3 := d - c;
        end if;
    end procedure;

    function trunc16(x : elem_t) return elem_t is
    begin
        return resize(signed(x(15 downto 0)), 32);
    end function;

begin

    accept  <= valid_i and (ready_i or not out_valid);
    ready_o <= ready_i or not out_valid;

    --------------------------------------------------------------------
    -- Combinational transform core
    --------------------------------------------------------------------
    comb_p : process(all)
        variable d   : block_t;
        variable t   : block_t;
        variable res : block_t;
        variable va, vb, vc, vd : elem_t;
    begin
        -- Wire inputs
        d(0)  := din_0;   d(1)  := din_1;   d(2)  := din_2;   d(3)  := din_3;
        d(4)  := din_4;   d(5)  := din_5;   d(6)  := din_6;   d(7)  := din_7;
        d(8)  := din_8;   d(9)  := din_9;   d(10) := din_10;  d(11) := din_11;
        d(12) := din_12;  d(13) := din_13;  d(14) := din_14;  d(15) := din_15;

        t   := (others => (others => '0'));
        res := (others => (others => '0'));

        if mode_i = MODE_HAD2 or mode_i = "101" then
            -- 2x2 Hadamard (self-inverse): direct computation
            va := d(0) + d(1);
            vb := d(0) - d(1);
            vc := d(2) + d(3);
            vd := d(2) - d(3);
            res(0) := va + vc;
            res(1) := vb + vd;
            res(2) := va - vc;
            res(3) := vb - vd;
            -- Truncate for forward 2x2 (mode 4)
            if mode_i = MODE_HAD2 then
                res(0) := trunc16(res(0));
                res(1) := trunc16(res(1));
                res(2) := trunc16(res(2));
                res(3) := trunc16(res(3));
            end if;
        else
            -- 4x4 transforms: row pass (4 butterflies)
            for i in 0 to 3 loop
                butterfly(mode_i,
                          d(i*4+0), d(i*4+1), d(i*4+2), d(i*4+3),
                          t(i*4+0), t(i*4+1), t(i*4+2), t(i*4+3));
            end loop;

            -- 4x4 transforms: column pass (4 butterflies)
            for j in 0 to 3 loop
                butterfly(mode_i,
                          t(0*4+j), t(1*4+j), t(2*4+j), t(3*4+j),
                          res(0*4+j), res(1*4+j), res(2*4+j), res(3*4+j));
            end loop;

            -- Truncate for forward DCT (mode 0)
            if mode_i = MODE_DCT4 then
                for i in 0 to 15 loop
                    res(i) := trunc16(res(i));
                end loop;
            end if;
        end if;

        result_comb <= res;
    end process;

    --------------------------------------------------------------------
    -- Output register with handshake
    --------------------------------------------------------------------
    reg_p : process(clk, rst_n)
    begin
        if rst_n = '0' then
            out_valid <= '0';
            out_reg   <= (others => (others => '0'));
        elsif rising_edge(clk) then
            if ready_i = '1' then
                out_valid <= '0';
            end if;
            if accept = '1' then
                out_reg   <= result_comb;
                out_valid <= '1';
            end if;
        end if;
    end process;

    -- Unpack output register to ports
    dout_0  <= out_reg(0);   dout_1  <= out_reg(1);
    dout_2  <= out_reg(2);   dout_3  <= out_reg(3);
    dout_4  <= out_reg(4);   dout_5  <= out_reg(5);
    dout_6  <= out_reg(6);   dout_7  <= out_reg(7);
    dout_8  <= out_reg(8);   dout_9  <= out_reg(9);
    dout_10 <= out_reg(10);  dout_11 <= out_reg(11);
    dout_12 <= out_reg(12);  dout_13 <= out_reg(13);
    dout_14 <= out_reg(14);  dout_15 <= out_reg(15);
    valid_o <= out_valid;

end architecture;
