--------------------------------------------------------------------------------
-- mb_header_engine.vhd
--
-- Macroblock header for I slices: computes coded_block_pattern from the
-- per-block nonzero flags and emits the header syntax elements of spec
-- 7.3.5 as variable-length fields for a bit_packer, exactly as
-- mb_compute_cbp + mb_cavlc_emit in src/encoder.c do:
--
--   I_4x4   : mb_type ue(0); 16 x prev_intra4x4_pred_mode_flag [+ 3-bit
--             rem_intra4x4_pred_mode] in block scan order; intra_chroma_
--             pred_mode ue(v); coded_block_pattern me(v) (Table 9-4 intra
--             column); mb_qp_delta se(0) only if cbp != 0.
--   I_16x16 : mb_type ue(1 + mode + 4*cbp_chroma + 12*cbp_luma);
--             intra_chroma_pred_mode ue(v); mb_qp_delta se(0).
--
-- mb_qp_delta = qp_i - QP_Y,PRED, wrapped into [-26, 25] (7.4.5), where
-- QP_Y,PRED is the slice QP (slice_qp_i at frame_start_i) and then the QP of
-- the last MB that transmitted a delta -- an MB without residual carries
-- the predicted QP forward unchanged. With one QP per frame every delta is
-- 0 ('1'). Exp-Golomb codes are emitted as one field: value v+1 with
-- length 2*bits(v+1)-1; the bit_packer treats the missing high bits as the
-- leading zeros.
--
-- predIntra4x4PredMode (spec 8.3.1.1) uses the in-MB modes for inner
-- blocks and the line buffer's top/left modes at the MB edge; if either
-- neighbour is unavailable the prediction is DC.
--
-- luma_nz_i(k) (raster br*4+bc) must be "block k has a nonzero level"
-- over the 16 coefficients for I_4x4, or over the 15 AC coefficients for
-- I_16x16 -- the caller knows which levels it has.
--
-- One field per cycle while fready_i; a header takes at most 20 cycles.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mb_header_engine is
    port (
        clk           : in  std_logic;
        rst_n         : in  std_logic;
        start_i       : in  std_logic;
        ready_o       : out std_logic;
        is_i4x4_i     : in  std_logic;
        mode16_i      : in  unsigned(1 downto 0);
        modes4_i      : in  std_logic_vector(63 downto 0);   -- 16 x 4 bits, raster
        mode_chroma_i : in  unsigned(1 downto 0);
        luma_nz_i     : in  std_logic_vector(15 downto 0);   -- raster
        chroma_dc_nz_i: in  std_logic;
        chroma_ac_nz_i: in  std_logic;
        mode4_top_i   : in  std_logic_vector(15 downto 0);   -- 4 x 4 bits (bc)
        mode4_left_i  : in  std_logic_vector(15 downto 0);   -- 4 x 4 bits (br)
        avail_top_i   : in  std_logic;
        avail_left_i  : in  std_logic;
        -- QP: slice QP at the frame start, then the MB's QP with each start
        frame_start_i : in  std_logic := '0';
        slice_qp_i    : in  unsigned(5 downto 0) := (others => '0');
        qp_i          : in  unsigned(5 downto 0) := (others => '0');
        -- field stream
        fbits_o       : out unsigned(15 downto 0);
        flen_o        : out unsigned(5 downto 0);
        fvalid_o      : out std_logic;
        fready_i      : in  std_logic;
        -- results
        done_o        : out std_logic;
        hdr_bits_o    : out unsigned(7 downto 0);
        cbp_luma_o    : out unsigned(3 downto 0);
        cbp_chroma_o  : out unsigned(1 downto 0);
        has_residual_o: out std_logic
    );
end entity;

architecture rtl of mb_header_engine is

    type u8_tab is array (0 to 47) of integer range 0 to 47;
    constant CBP_CODENUM : u8_tab := (
         3, 29, 30, 17, 31, 18, 37,  8, 32, 38, 19,  9, 20, 10, 11,  2,
        16, 33, 34, 21, 35, 22, 39,  4, 36, 40, 23,  5, 24,  6,  7,  1,
        41, 42, 43, 25, 44, 26, 46, 12, 45, 47, 27, 13, 28, 14, 15,  0);

    type i16_tab is array (0 to 15) of integer range 0 to 3;
    constant SCAN_BR : i16_tab := (0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3);
    constant SCAN_BC : i16_tab := (0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3);

    -- Exp-Golomb ue(v) as (value, length): value = v+1, length = 2*bits-1
    procedure ue_field(v : in integer range 0 to 63;
                       bits : out unsigned(15 downto 0); len : out unsigned(5 downto 0)) is
        variable code : unsigned(6 downto 0);
        variable nb   : integer range 1 to 7;
    begin
        code := to_unsigned(v + 1, 7);
        if    code(6) = '1' then nb := 7;
        elsif code(5) = '1' then nb := 6;
        elsif code(4) = '1' then nb := 5;
        elsif code(3) = '1' then nb := 4;
        elsif code(2) = '1' then nb := 3;
        elsif code(1) = '1' then nb := 2;
        else                     nb := 1;
        end if;
        bits := resize(code, 16);
        len  := to_unsigned(2 * nb - 1, 6);
    end procedure;

    -- Exp-Golomb se(v): codeNum = 2v-1 for v > 0, -2v otherwise
    procedure se_field(v : in integer range -26 to 25;
                       bits : out unsigned(15 downto 0); len : out unsigned(5 downto 0)) is
    begin
        if v > 0 then ue_field(2 * v - 1, bits, len); else ue_field(-2 * v, bits, len); end if;
    end procedure;

    function mode_of(v : std_logic_vector; k : integer) return integer is
    begin
        return to_integer(unsigned(v(4*k+3 downto 4*k)));
    end function;

    -- Modes are held in block SCAN order and shifted out one per step; the
    -- neighbours a block needs are then fixed taps into a short history of
    -- the modes already emitted: top is 2 back (odd row of a quadrant) or
    -- 6 back (top row of quadrants 2/3), left is 1 back (odd column) or
    -- 3 back (blocks 4,6,12,14). No 16:1 muxes.
    type mode_arr is array (0 to 15) of unsigned(3 downto 0);
    type hist_arr is array (1 to 6) of unsigned(3 downto 0);

    signal busy      : std_logic := '0';
    signal step      : integer range 0 to 20 := 0;
    signal i4        : std_logic := '0';
    signal m16       : unsigned(1 downto 0) := (others => '0');
    signal sr        : mode_arr := (others => (others => '0'));
    signal hist      : hist_arr := (others => (others => '0'));
    signal mchroma   : unsigned(1 downto 0) := (others => '0');
    signal cbp_l     : unsigned(3 downto 0) := (others => '0');
    signal cbp_c     : unsigned(1 downto 0) := (others => '0');
    signal m4top, m4left : std_logic_vector(15 downto 0) := (others => '0');
    signal atop, aleft : std_logic := '0';
    signal fbits_q   : unsigned(15 downto 0) := (others => '0');
    signal flen_q    : unsigned(5 downto 0) := (others => '0');
    signal fvalid_q  : std_logic := '0';
    signal done_q    : std_logic := '0';
    signal nbits     : unsigned(7 downto 0) := (others => '0');
    signal qp_prev   : unsigned(5 downto 0) := (others => '0');   -- QP_Y,PRED
    signal dq        : integer range -26 to 25 := 0;              -- this MB's mb_qp_delta

begin

    ready_o        <= not busy;
    fbits_o        <= fbits_q;
    flen_o         <= flen_q;
    fvalid_o       <= fvalid_q;
    done_o         <= done_q;
    hdr_bits_o     <= nbits;
    cbp_luma_o     <= cbp_l;
    cbp_chroma_o   <= cbp_c;
    has_residual_o <= '1' when (cbp_l /= 0 or cbp_c /= 0) else '0';

    process(clk, rst_n)
        variable cl      : unsigned(3 downto 0);
        variable cc      : unsigned(1 downto 0);
        variable s, br, bc : integer range 0 to 15;
        variable mt, ml, pm, am, rem_m : integer range 0 to 15;
        variable tok, lok : boolean;
        variable fb      : unsigned(15 downto 0);
        variable fl      : unsigned(5 downto 0);
        variable emit    : boolean;
        variable last    : boolean;
        variable can     : boolean;
        variable mbtype  : integer range 0 to 63;
        variable d       : integer range -63 to 63;
    begin
        if rst_n = '0' then
            busy <= '0'; fvalid_q <= '0'; done_q <= '0'; step <= 0;
        elsif rising_edge(clk) then
            done_q <= '0';
            can := (fvalid_q = '0') or (fready_i = '1');
            if fready_i = '1' then fvalid_q <= '0'; end if;
            if frame_start_i = '1' then qp_prev <= slice_qp_i; end if;

            if busy = '0' then
                if start_i = '1' then
                    -- coded_block_pattern
                    if is_i4x4_i = '1' then
                        cl(0) := luma_nz_i(0)  or luma_nz_i(1)  or luma_nz_i(4)  or luma_nz_i(5);
                        cl(1) := luma_nz_i(2)  or luma_nz_i(3)  or luma_nz_i(6)  or luma_nz_i(7);
                        cl(2) := luma_nz_i(8)  or luma_nz_i(9)  or luma_nz_i(12) or luma_nz_i(13);
                        cl(3) := luma_nz_i(10) or luma_nz_i(11) or luma_nz_i(14) or luma_nz_i(15);
                    else
                        cl := "000" & (or luma_nz_i);
                    end if;
                    if chroma_ac_nz_i = '1' then cc := "10";
                    elsif chroma_dc_nz_i = '1' then cc := "01";
                    else cc := "00";
                    end if;
                    cbp_l <= cl; cbp_c <= cc;
                    -- the delta is transmitted for I_16x16 always, for I_4x4
                    -- only with residual; QP_Y,PRED moves only then
                    d := to_integer(qp_i) - to_integer(qp_prev);
                    if d > 25 then d := d - 52; elsif d < -26 then d := d + 52; end if;
                    dq <= d;
                    if is_i4x4_i = '0' or cl /= 0 or cc /= 0 then qp_prev <= qp_i; end if;
                    i4 <= is_i4x4_i; m16 <= mode16_i;
                    for s in 0 to 15 loop
                        sr(s) <= unsigned(modes4_i(4 * (SCAN_BR(s) * 4 + SCAN_BC(s)) + 3 downto 4 * (SCAN_BR(s) * 4 + SCAN_BC(s))));
                    end loop;
                    mchroma <= mode_chroma_i;
                    m4top <= mode4_top_i; m4left <= mode4_left_i;
                    atop <= avail_top_i; aleft <= avail_left_i;
                    nbits <= (others => '0');
                    step  <= 0;
                    busy  <= '1';
                end if;
            elsif can then
                emit := true;
                last := false;
                fb := (others => '0'); fl := (others => '0');
                if i4 = '1' then
                    if step = 0 then
                        fb := to_unsigned(1, 16); fl := to_unsigned(1, 6);          -- mb_type ue(0)
                    elsif step <= 16 then
                        s  := step - 1;
                        br := SCAN_BR(s); bc := SCAN_BC(s);
                        am := to_integer(sr(0));
                        if br = 0 then
                            mt := mode_of(m4top, bc); tok := (atop = '1');
                        elsif s = 8 or s = 9 or s = 12 or s = 13 then
                            mt := to_integer(hist(6)); tok := true;
                        else
                            mt := to_integer(hist(2)); tok := true;
                        end if;
                        if bc = 0 then
                            ml := mode_of(m4left, br); lok := (aleft = '1');
                        elsif s = 4 or s = 6 or s = 12 or s = 14 then
                            ml := to_integer(hist(3)); lok := true;
                        else
                            ml := to_integer(hist(1)); lok := true;
                        end if;
                        -- advance the scan-order register and the history
                        sr(0 to 14) <= sr(1 to 15);
                        hist(1) <= sr(0);
                        hist(2 to 6) <= hist(1 to 5);
                        if not (tok and lok) then pm := 2;
                        elsif mt < ml then pm := mt;
                        else pm := ml;
                        end if;
                        if am = pm then
                            fb := to_unsigned(1, 16); fl := to_unsigned(1, 6);      -- prev flag = 1
                        else
                            if am < pm then rem_m := am; else rem_m := am - 1; end if;
                            fb := to_unsigned(rem_m, 16); fl := to_unsigned(4, 6);  -- '0' + rem(3)
                        end if;
                    elsif step = 17 then
                        ue_field(to_integer(mchroma), fb, fl);
                    elsif step = 18 then
                        ue_field(CBP_CODENUM(to_integer(cbp_c & cbp_l)), fb, fl);
                        if cbp_l = 0 and cbp_c = 0 then last := true; end if;
                    else
                        se_field(dq, fb, fl);                                       -- mb_qp_delta se(v)
                        last := true;
                    end if;
                else
                    if step = 0 then
                        mbtype := 1 + to_integer(m16) + 4 * to_integer(cbp_c) + 12 * to_integer(cbp_l(0 downto 0));
                        ue_field(mbtype, fb, fl);
                    elsif step = 1 then
                        ue_field(to_integer(mchroma), fb, fl);
                    else
                        se_field(dq, fb, fl);                                       -- mb_qp_delta se(v)
                        last := true;
                    end if;
                end if;

                if emit then
                    fbits_q  <= fb; flen_q <= fl; fvalid_q <= '1';
                    nbits    <= nbits + resize(fl, 8);
                end if;
                if last then
                    busy   <= '0';
                    done_q <= '1';
                else
                    step <= step + 1;
                end if;
            end if;
        end if;
    end process;

end architecture;
