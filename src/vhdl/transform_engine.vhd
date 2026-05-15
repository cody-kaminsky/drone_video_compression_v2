--------------------------------------------------------------------------------
-- transform_engine.vhd
--
-- H.264 4x4 integer transform engine.  Implements all six transforms from
-- transform.c as a single combinational core with registered I/O:
--
--   Mode 0: dct4x4          (forward DCT,  spec 8.5.6)
--   Mode 1: idct4x4         (inverse DCT,  spec 8.5.6)
--   Mode 2: hadamard4x4     (forward Hadamard, DC luma)
--   Mode 3: ihadamard4x4    (inverse Hadamard, DC luma)
--   Mode 4: hadamard2x2     (forward Hadamard, DC chroma)
--   Mode 5: ihadamard2x2    (inverse Hadamard, DC chroma)
--
-- Interface: valid/ready handshake.  One block in, one block out, single-
-- cycle throughput with one-cycle latency.  Pure shift-and-add; no DSPs.
--
-- Data layout: 16-element row-major array, matching the C reference.
--   index = row*4 + col.
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

    type block_t is array (0 to 15) of signed(31 downto 0);

    constant MODE_DCT4      : unsigned(2 downto 0) := "000";
    constant MODE_IDCT4     : unsigned(2 downto 0) := "001";
    constant MODE_HAD4      : unsigned(2 downto 0) := "010";
    constant MODE_IHAD4     : unsigned(2 downto 0) := "011";
    constant MODE_HAD2      : unsigned(2 downto 0) := "100";
    constant MODE_IHAD2     : unsigned(2 downto 0) := "101";

    signal din_block   : block_t;
    signal result_comb : block_t;

    signal out_reg   : block_t := (others => (others => '0'));
    signal out_valid : std_logic := '0';

    signal accept : std_logic;

    --------------------------------------------------------------------
    -- 4-point butterfly for forward DCT (one row or column).
    --   a = r0+r3,  b = r1+r2,  c = r1-r2,  d = r0-r3
    --   out = [a+b,  2d+c,  a-b,  d-2c]
    --------------------------------------------------------------------
    procedure butterfly_dct(
        r0, r1, r2, r3 : in  signed(31 downto 0);
        o0, o1, o2, o3 : out signed(31 downto 0)) is
        variable a, b, c, d : signed(31 downto 0);
    begin
        a := r0 + r3;
        b := r1 + r2;
        c := r1 - r2;
        d := r0 - r3;
        o0 := a + b;
        o1 := shift_left(d, 1) + c;
        o2 := a - b;
        o3 := d - shift_left(c, 1);
    end procedure;

    --------------------------------------------------------------------
    -- 4-point butterfly for inverse DCT.
    --   a = r0+r2,  b = r0-r2
    --   c = (r1>>1)-r3,  d = r1+(r3>>1)
    --   out = [a+d,  b+c,  b-c,  a-d]
    --------------------------------------------------------------------
    procedure butterfly_idct(
        r0, r1, r2, r3 : in  signed(31 downto 0);
        o0, o1, o2, o3 : out signed(31 downto 0)) is
        variable a, b, c, d : signed(31 downto 0);
    begin
        a := r0 + r2;
        b := r0 - r2;
        c := shift_right(r1, 1) - r3;
        d := r1 + shift_right(r3, 1);
        o0 := a + d;
        o1 := b + c;
        o2 := b - c;
        o3 := a - d;
    end procedure;

    --------------------------------------------------------------------
    -- 4-point butterfly for Hadamard (same for forward & inverse).
    --   a = r0+r3,  b = r1+r2,  c = r1-r2,  d = r0-r3
    --   out = [a+b,  d+c,  a-b,  d-c]
    --------------------------------------------------------------------
    procedure butterfly_had(
        r0, r1, r2, r3 : in  signed(31 downto 0);
        o0, o1, o2, o3 : out signed(31 downto 0)) is
        variable a, b, c, d : signed(31 downto 0);
    begin
        a := r0 + r3;
        b := r1 + r2;
        c := r1 - r2;
        d := r0 - r3;
        o0 := a + b;
        o1 := d + c;
        o2 := a - b;
        o3 := d - c;
    end procedure;

begin

    -- Pack input ports into array
    din_block(0)  <= din_0;   din_block(1)  <= din_1;
    din_block(2)  <= din_2;   din_block(3)  <= din_3;
    din_block(4)  <= din_4;   din_block(5)  <= din_5;
    din_block(6)  <= din_6;   din_block(7)  <= din_7;
    din_block(8)  <= din_8;   din_block(9)  <= din_9;
    din_block(10) <= din_10;  din_block(11) <= din_11;
    din_block(12) <= din_12;  din_block(13) <= din_13;
    din_block(14) <= din_14;  din_block(15) <= din_15;

    accept  <= valid_i and (ready_i or not out_valid);
    ready_o <= ready_i or not out_valid;

    --------------------------------------------------------------------
    -- Combinational transform core
    --------------------------------------------------------------------
    comb_p : process(all)
        variable d   : block_t;
        variable t   : block_t;
        variable res : block_t;
        variable va, vb, vc, vd : signed(31 downto 0);
    begin
        d   := din_block;
        t   := (others => (others => '0'));
        res := (others => (others => '0'));

        case mode_i is

        when MODE_DCT4 =>
            -- Row pass
            for i in 0 to 3 loop
                butterfly_dct(d(i*4+0), d(i*4+1), d(i*4+2), d(i*4+3),
                              t(i*4+0), t(i*4+1), t(i*4+2), t(i*4+3));
            end loop;
            -- Column pass
            for j in 0 to 3 loop
                butterfly_dct(t(0*4+j), t(1*4+j), t(2*4+j), t(3*4+j),
                              res(0*4+j), res(1*4+j), res(2*4+j), res(3*4+j));
            end loop;

        when MODE_IDCT4 =>
            -- Row pass
            for i in 0 to 3 loop
                butterfly_idct(d(i*4+0), d(i*4+1), d(i*4+2), d(i*4+3),
                               t(i*4+0), t(i*4+1), t(i*4+2), t(i*4+3));
            end loop;
            -- Column pass
            for j in 0 to 3 loop
                butterfly_idct(t(0*4+j), t(1*4+j), t(2*4+j), t(3*4+j),
                               res(0*4+j), res(1*4+j), res(2*4+j), res(3*4+j));
            end loop;

        when MODE_HAD4 | MODE_IHAD4 =>
            -- Row pass
            for i in 0 to 3 loop
                butterfly_had(d(i*4+0), d(i*4+1), d(i*4+2), d(i*4+3),
                              t(i*4+0), t(i*4+1), t(i*4+2), t(i*4+3));
            end loop;
            -- Column pass
            for j in 0 to 3 loop
                butterfly_had(t(0*4+j), t(1*4+j), t(2*4+j), t(3*4+j),
                              res(0*4+j), res(1*4+j), res(2*4+j), res(3*4+j));
            end loop;

        when MODE_HAD2 | MODE_IHAD2 =>
            va := d(0) + d(1);
            vb := d(0) - d(1);
            vc := d(2) + d(3);
            vd := d(2) - d(3);
            res(0) := va + vc;
            res(1) := vb + vd;
            res(2) := va - vc;
            res(3) := vb - vd;

        when others =>
            null;

        end case;

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
