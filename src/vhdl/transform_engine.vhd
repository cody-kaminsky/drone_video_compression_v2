--------------------------------------------------------------------------------
-- transform_engine.vhd
--
-- H.264 4x4 integer transform engine (two-stage pipeline, area-optimized).
--
--   Mode 0: dct4x4          (forward DCT,  spec 8.5.6)
--   Mode 1: idct4x4         (inverse DCT,  spec 8.5.6)
--   Mode 2: hadamard4x4     (forward Hadamard, DC luma)
--   Mode 3: ihadamard4x4    (inverse Hadamard, DC luma)
--   Mode 4: hadamard2x2     (forward Hadamard, DC chroma)
--   Mode 5: ihadamard2x2    (inverse Hadamard, DC chroma)
--
-- Architecture: 8 instances of ONE shared butterfly (4 row + 4 column),
-- each built from 8 add/sub units whose operands are muxed by mode. The
-- mode select never duplicates an adder. The 2x2 Hadamard is the 4-point
-- Hadamard butterfly with inputs permuted (r0,r1,r2,r3) = (d0,d2,d3,d1),
-- so it reuses row butterfly 0 and skips the column pass (dout_4..15 are
-- zero in the 2x2 modes, as in the C reference).
--
-- Pipeline: row pass registered, column pass registered. One block per
-- cycle throughput, two cycles latency, single global stall (ready_i).
-- Pure shift-and-add — no DSPs.
--
-- Width: ports stay 32-bit for compatibility with the C reference (i32
-- inverse paths). Internally the datapath is W bits (generic, default 20).
-- For inputs within the 16-bit range that the 8-bit profile guarantees
-- (spec 8.5.12 constrains inverse-transform inputs and intermediates to
-- 16 bits), 20 bits is exact: two passes each grow the magnitude by at
-- most 4x. Results for the truncating modes (0 and 4) are exact for any
-- W >= 16. Inputs outside +/-2^(W-1) wrap differently from the i32 C model.
--
-- Data layout: 16-element row-major array, matching the C reference.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity transform_engine is
    generic (
        W   : positive := 20;   -- internal datapath width (see header)
        -- "both": all six modes. "fwd": modes 0/2/4 only, "inv": modes
        -- 1/3/5 only, "had": mode 3 only (a 4x4 Hadamard lane; mode_i is
        -- ignored). A single-direction instance drops the unused operand
        -- muxes; the pipeline has separate T and iT stages, so each can be
        -- specialized. Unsupported modes give garbage.
        DIR : string   := "both"
    );
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

    subtype elem_t is signed(W - 1 downto 0);
    type block_t is array (0 to 15) of elem_t;

    constant MODE_DCT4  : unsigned(2 downto 0) := "000";
    constant MODE_IDCT4 : unsigned(2 downto 0) := "001";
    constant MODE_HAD2  : unsigned(2 downto 0) := "100";
    constant MODE_IHAD2 : unsigned(2 downto 0) := "101";

    -- Pipeline registers
    signal row_reg   : block_t := (others => (others => '0'));
    signal row_mode  : unsigned(2 downto 0) := (others => '0');
    signal row_valid : std_logic := '0';
    signal out_reg   : block_t := (others => (others => '0'));
    signal out_valid : std_logic := '0';
    signal advance   : std_logic;

    -- x + y, or x - y when sub = '1'. Written so that synthesis builds a
    -- single carry chain with the subtract folded into the operand XOR
    -- and the carry-in.
    function addsub(x, y : elem_t; sub : std_logic) return elem_t is
        variable ym  : elem_t;
        variable cin : elem_t := (others => '0');
    begin
        ym     := y xor (elem_t'range => sub);
        cin(0) := sub;
        return x + ym + cin;
    end function;

    function mux(sel : boolean; a, b : elem_t) return elem_t is
    begin
        if sel then
            return a;
        else
            return b;
        end if;
    end function;

    function to_std_logic(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

    -- Shared butterfly: 8 add/sub units, operands selected by mode.
    --   IDCT : a=r0+r2  b=r0-r2  c=(r1>>1)-r3  d=r1+(r3>>1)
    --          o0=a+d   o1=b+c   o2=b-c        o3=a-d
    --   DCT  : a=r0+r3  b=r1+r2  c=r1-r2       d=r0-r3
    --          o0=a+b   o1=2d+c  o2=a-b        o3=d-2c
    --   HAD  : stage 1 as DCT; o0=a+b  o1=d+c  o2=a-b  o3=d-c
    procedure butterfly(
        mode                : in  unsigned(2 downto 0);
        r0, r1, r2, r3      : in  elem_t;
        o0, o1, o2, o3      : out elem_t) is
        variable idct, dct  : boolean;
        variable a, b, c, d : elem_t;
        variable sub_d      : std_logic;
    begin
        -- Mode decode; constant-folded away in single-direction instances.
        if DIR = "had" then
            idct := false;
            dct  := false;
        elsif DIR = "fwd" then
            idct := false;
            dct  := (mode(2) = '0');            -- 0 = DCT, 2 = Hadamard
        elsif DIR = "inv" then
            idct := (mode(2 downto 1) = "00");  -- 1 = IDCT, 3 = iHadamard
            dct  := false;
        else
            idct := (mode = MODE_IDCT4);
            dct  := (mode = MODE_DCT4);
        end if;
        if idct then sub_d := '0'; else sub_d := '1'; end if;

        a := addsub(r0,                            mux(idct, r2, r3),                 '0');
        b := addsub(mux(idct, r0, r1),             r2,                                to_std_logic(idct));
        c := addsub(mux(idct, shift_right(r1, 1), r1), mux(idct, r3, r2),             '1');
        d := addsub(mux(idct, r1, r0),             mux(idct, shift_right(r3, 1), r3), sub_d);

        o0 := addsub(a,                                   mux(idct, d, b),                            '0');
        o1 := addsub(mux(idct, b, mux(dct, shift_left(d, 1), d)), c,                                  '0');
        o2 := addsub(mux(idct, b, a),                     mux(idct, c, b),                            '1');
        o3 := addsub(mux(idct, a, d),                     mux(idct, d, mux(dct, shift_left(c, 1), c)), '1');
    end procedure;

    function trunc16(x : elem_t) return elem_t is
    begin
        return resize(signed(x(15 downto 0)), W);
    end function;

begin

    -- Single global stall: everything moves when the output slot is free
    -- or being consumed this cycle.
    advance <= ready_i or not out_valid;
    ready_o <= advance;

    --------------------------------------------------------------------
    -- Pipeline
    --------------------------------------------------------------------
    pipe_p : process(clk, rst_n)
        variable d   : block_t;
        variable t   : block_t;
        variable res : block_t;
        variable is_2x2 : boolean;
    begin
        if rst_n = '0' then
            row_valid <= '0';
            out_valid <= '0';
            row_mode  <= (others => '0');
        elsif rising_edge(clk) then
            if advance = '1' then
                ------------------------------------------------------
                -- Stage 1: row pass (or the whole 2x2 Hadamard on
                -- butterfly 0 with permuted inputs).
                ------------------------------------------------------
                d(0)  := resize(din_0, W);   d(1)  := resize(din_1, W);
                d(2)  := resize(din_2, W);   d(3)  := resize(din_3, W);
                d(4)  := resize(din_4, W);   d(5)  := resize(din_5, W);
                d(6)  := resize(din_6, W);   d(7)  := resize(din_7, W);
                d(8)  := resize(din_8, W);   d(9)  := resize(din_9, W);
                d(10) := resize(din_10, W);  d(11) := resize(din_11, W);
                d(12) := resize(din_12, W);  d(13) := resize(din_13, W);
                d(14) := resize(din_14, W);  d(15) := resize(din_15, W);

                is_2x2 := (DIR /= "had") and ((mode_i = MODE_HAD2) or (mode_i = MODE_IHAD2));

                if is_2x2 then
                    -- 2x2 Hadamard == 4-point Hadamard of (d0, d2, d3, d1)
                    butterfly(MODE_HAD2, d(0), d(2), d(3), d(1),
                              t(0), t(1), t(2), t(3));
                else
                    butterfly(mode_i, d(0), d(1), d(2), d(3),
                              t(0), t(1), t(2), t(3));
                end if;
                for i in 1 to 3 loop
                    butterfly(mode_i,
                              d(i*4+0), d(i*4+1), d(i*4+2), d(i*4+3),
                              t(i*4+0), t(i*4+1), t(i*4+2), t(i*4+3));
                end loop;

                row_reg   <= t;
                row_mode  <= mode_i;
                row_valid <= valid_i;

                ------------------------------------------------------
                -- Stage 2: column pass (bypassed for 2x2), truncation
                -- for the forward modes 0 and 4.
                ------------------------------------------------------
                if DIR /= "had" and (row_mode = MODE_HAD2 or row_mode = MODE_IHAD2) then
                    -- 2x2 result is already complete after the row pass;
                    -- elements 4..15 are zero as in the C reference.
                    -- (Leaving them undefined was tried and synthesized
                    -- larger, so the gate stays.)
                    res := (others => (others => '0'));
                    res(0) := row_reg(0);  res(1) := row_reg(1);
                    res(2) := row_reg(2);  res(3) := row_reg(3);
                else
                    for j in 0 to 3 loop
                        butterfly(row_mode,
                                  row_reg(0*4+j), row_reg(1*4+j),
                                  row_reg(2*4+j), row_reg(3*4+j),
                                  res(0*4+j), res(1*4+j),
                                  res(2*4+j), res(3*4+j));
                    end loop;
                end if;

                if DIR /= "had" and (row_mode = MODE_DCT4 or row_mode = MODE_HAD2) then
                    for i in 0 to 15 loop
                        res(i) := trunc16(res(i));
                    end loop;
                end if;

                out_reg   <= res;
                out_valid <= row_valid;
            end if;
        end if;
    end process;

    -- Unpack output register to ports
    dout_0  <= resize(out_reg(0), 32);   dout_1  <= resize(out_reg(1), 32);
    dout_2  <= resize(out_reg(2), 32);   dout_3  <= resize(out_reg(3), 32);
    dout_4  <= resize(out_reg(4), 32);   dout_5  <= resize(out_reg(5), 32);
    dout_6  <= resize(out_reg(6), 32);   dout_7  <= resize(out_reg(7), 32);
    dout_8  <= resize(out_reg(8), 32);   dout_9  <= resize(out_reg(9), 32);
    dout_10 <= resize(out_reg(10), 32);  dout_11 <= resize(out_reg(11), 32);
    dout_12 <= resize(out_reg(12), 32);  dout_13 <= resize(out_reg(13), 32);
    dout_14 <= resize(out_reg(14), 32);  dout_15 <= resize(out_reg(15), 32);
    valid_o <= out_valid;

end architecture;
