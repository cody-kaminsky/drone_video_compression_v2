--------------------------------------------------------------------------------
-- mb_header_dec_engine.vhd
--
-- Parses one macroblock header of spec 7.3.5 for the I macroblocks this
-- codec emits: the mirror of mb_header_engine.
--
--   I_4x4   : mb_type ue(0); 16 x prev_intra4x4_pred_mode_flag [+ 3-bit
--             rem_intra4x4_pred_mode] in block scan order; intra_chroma_
--             pred_mode ue(v); coded_block_pattern me(v); mb_qp_delta se(v)
--             only when the pattern is nonzero.
--   I_16x16 : mb_type ue(1 + mode + 4*cbp_chroma + 12*cbp_luma), which packs
--             the luma pattern into a single bit; intra_chroma_pred_mode
--             ue(v); mb_qp_delta se(v), always present.
--
-- Note the asymmetry that makes this worth testing against the real decoder
-- rather than against the encoder's inputs: I_16x16 writes "some luma is
-- coded" as one bit and reads it back as cbp_luma = 15.
--
-- predIntra4x4PredMode (spec 8.3.1.1) needs each block's top and left
-- neighbour. Because blocks are decoded in the same scan order the encoder
-- wrote them, those neighbours are fixed taps into a short history of the
-- modes already decoded -- top is 2 back, or 6 back for the blocks that open
-- the lower quadrants; left is 1 back, or 3 back for the blocks that open an
-- odd quadrant column. Only the macroblock edge reaches outside, to the line
-- buffer's top and left modes. No 16:1 muxes, exactly as on the encode side.
--
-- Interface to the bit reader is peek/consume, not get(n): a variable-length
-- code's length is only known after the match, and that length decides where
-- the next element starts.
--
--   start_i    with qp_i and the neighbour context valid; ready_o must be high
--   done_o     one cycle, with every result port valid
--   err_o      with done_o: the header did not parse. Raised rather than
--              emitting a plausible macroblock, because a header that
--              desynchronises takes the rest of the slice with it and the
--              damage shows up far from the cause.
--     1 mb_type outside the I range   2 Exp-Golomb prefix too long
--     3 intra_chroma_pred_mode > 3    4 coded_block_pattern codeNum >= 48
--     5 mb_qp_delta out of range
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mb_header_dec_engine is
    port (
        clk          : in  std_logic;
        rst_n        : in  std_logic;

        -- Job
        start_i      : in  std_logic;
        ready_o      : out std_logic;
        qp_i         : in  unsigned(5 downto 0);            -- QP before this MB
        mode4_top_i  : in  std_logic_vector(15 downto 0);   -- 4 x 4 bits (bc)
        mode4_left_i : in  std_logic_vector(15 downto 0);   -- 4 x 4 bits (br)
        avail_top_i  : in  std_logic;
        avail_left_i : in  std_logic;

        -- Bit reader
        peek_i       : in  unsigned(31 downto 0);
        avail_i      : in  std_logic;
        consume_o    : out std_logic;
        consume_n_o  : out unsigned(5 downto 0);

        -- Result
        done_o       : out std_logic;
        err_o        : out std_logic;
        err_code_o   : out unsigned(3 downto 0);
        is_i4x4_o    : out std_logic;
        mode16_o     : out unsigned(1 downto 0);
        modes4_o     : out std_logic_vector(63 downto 0);   -- 16 x 4 bits, raster
        mode_chroma_o: out unsigned(1 downto 0);
        cbp_luma_o   : out unsigned(3 downto 0);
        cbp_chroma_o : out unsigned(1 downto 0);
        has_residual_o : out std_logic;
        qp_o         : out unsigned(5 downto 0);            -- QP after mb_qp_delta
        hdr_bits_o   : out unsigned(7 downto 0)
    );
end entity;

architecture rtl of mb_header_dec_engine is

    constant MAX_K : integer := 15;     -- longest Exp-Golomb prefix accepted

    type state_t is (S_IDLE, S_MBTYPE, S_MODES, S_CHROMA, S_CBP, S_QPD, S_DONE);
    signal st : state_t := S_IDLE;

    type u48_tab is array (0 to 47) of integer range 0 to 47;
    -- coded_block_pattern -> codeNum, Table 9-4 intra column. The same
    -- constant the encoder carries; the decode direction is derived from it
    -- at elaboration rather than written out a second time, so the two cannot
    -- disagree.
    constant CBP_CODENUM : u48_tab := (
         3, 29, 30, 17, 31, 18, 37,  8, 32, 38, 19,  9, 20, 10, 11,  2,
        16, 33, 34, 21, 35, 22, 39,  4, 36, 40, 23,  5, 24,  6,  7,  1,
        41, 42, 43, 25, 44, 26, 46, 12, 45, 47, 27, 13, 28, 14, 15,  0);

    function invert(t : u48_tab) return u48_tab is
        variable r : u48_tab := (others => 0);
    begin
        for i in 0 to 47 loop
            r(t(i)) := i;
        end loop;
        return r;
    end function;

    constant CODENUM_CBP : u48_tab := invert(CBP_CODENUM);

    type i16_tab is array (0 to 15) of integer range 0 to 3;
    constant SCAN_BR : i16_tab := (0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3);
    constant SCAN_BC : i16_tab := (0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3);

    -- Job context
    signal qp_in   : unsigned(5 downto 0) := (others => '0');
    signal m4top   : std_logic_vector(15 downto 0) := (others => '0');
    signal m4left  : std_logic_vector(15 downto 0) := (others => '0');
    signal atop    : std_logic := '0';
    signal aleft   : std_logic := '0';

    -- Results
    signal i4      : std_logic := '0';
    signal m16     : unsigned(1 downto 0) := (others => '0');
    signal mchroma : unsigned(1 downto 0) := (others => '0');
    signal cbp_l   : unsigned(3 downto 0) := (others => '0');
    signal cbp_c   : unsigned(1 downto 0) := (others => '0');
    signal has_res : std_logic := '0';
    signal qp_q    : unsigned(5 downto 0) := (others => '0');
    signal nbits   : unsigned(7 downto 0) := (others => '0');
    signal err_q   : std_logic := '0';
    signal errc_q  : unsigned(3 downto 0) := (others => '0');

    type mode_arr is array (0 to 15) of unsigned(3 downto 0);
    type hist_arr is array (1 to 6) of unsigned(3 downto 0);
    signal modes   : mode_arr := (others => to_unsigned(2, 4));
    signal hist    : hist_arr := (others => (others => '0'));
    signal sidx    : integer range 0 to 15 := 0;

    -- consume_o is registered, so it asserts the cycle AFTER the decision,
    -- the reader shifts at the end of that cycle, and the next window is only
    -- valid the cycle after that. Without the dead cycle every element after
    -- the first is parsed from stale bits.
    signal settling : std_logic := '0';

    -- Leading zeros of the peek window, which is the Exp-Golomb prefix length.
    function count_lz(v : unsigned(31 downto 0)) return integer is
    begin
        for i in 31 downto 0 loop
            if v(i) = '1' then return 31 - i; end if;
        end loop;
        return 32;
    end function;

    -- ue(v) given its prefix length: k zeros, a 1, then k suffix bits.
    function ue_of(v : unsigned(31 downto 0); k : integer) return integer is
        variable sh : unsigned(31 downto 0);
    begin
        if k = 0 then return 0; end if;
        sh := shift_left(v, k + 1);
        return 2 ** k - 1 + to_integer(sh(31 downto 32 - k));
    end function;

    function mode_of(v : std_logic_vector; k : integer) return integer is
    begin
        return to_integer(unsigned(v(4 * k + 3 downto 4 * k)));
    end function;

    signal lz : integer range 0 to 32;

begin

    lz <= count_lz(peek_i);

    ready_o        <= '1' when st = S_IDLE else '0';
    -- Held off for the settling cycle, so the LAST element's consume has
    -- reached the reader before the caller is told the header is done. Assert
    -- it a cycle early and the reader's bit position still points one element
    -- back, which is invisible here and fatal to the residual blocks that
    -- read from it next.
    done_o         <= '1' when (st = S_DONE and settling = '0') else '0';
    err_o          <= err_q;
    err_code_o     <= errc_q;
    is_i4x4_o      <= i4;
    mode16_o       <= m16;
    mode_chroma_o  <= mchroma;
    cbp_luma_o     <= cbp_l;
    cbp_chroma_o   <= cbp_c;
    has_residual_o <= has_res;
    qp_o           <= qp_q;
    hdr_bits_o     <= nbits;

    g_out : for i in 0 to 15 generate
        modes4_o(4 * i + 3 downto 4 * i) <= std_logic_vector(modes(i));
    end generate;

    main_p : process(clk)
        variable k      : integer range 0 to 32;
        variable v      : integer;
        variable s      : integer range 0 to 15;
        variable br, bc : integer range 0 to 3;
        variable mt, ml : integer range 0 to 15;
        variable pm, am : integer range 0 to 15;
        variable rm     : integer range 0 to 7;
        variable tok    : boolean;
        variable lok    : boolean;
        variable used   : integer range 0 to 63;
        variable delta  : integer range -32 to 32;
        variable qsum   : integer range 0 to 255;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                st          <= S_IDLE;
                consume_o   <= '0';
                consume_n_o <= (others => '0');
                settling    <= '0';
                err_q       <= '0';
                errc_q      <= (others => '0');
                nbits       <= (others => '0');
            else
                consume_o   <= '0';
                consume_n_o <= (others => '0');
                used        := 0;

                if settling = '1' then
                    settling <= '0';
                else
                case st is

                ----------------------------------------------------------
                when S_IDLE =>
                    if start_i = '1' then
                        qp_in   <= qp_i;
                        qp_q    <= qp_i;
                        m4top   <= mode4_top_i;
                        m4left  <= mode4_left_i;
                        atop    <= avail_top_i;
                        aleft   <= avail_left_i;
                        -- An I_16x16 macroblock counts as DC for its
                        -- neighbours, so DC is also the right default for a
                        -- header that never writes these.
                        modes   <= (others => to_unsigned(2, 4));
                        m16     <= (others => '0');
                        cbp_l   <= (others => '0');
                        cbp_c   <= (others => '0');
                        has_res <= '0';
                        err_q   <= '0';
                        errc_q  <= (others => '0');
                        nbits   <= (others => '0');
                        sidx    <= 0;
                        st      <= S_MBTYPE;
                    end if;

                ----------------------------------------------------------
                when S_MBTYPE =>
                    if avail_i = '1' then
                        k := lz;
                        if k > MAX_K then
                            err_q <= '1'; errc_q <= x"2"; st <= S_DONE;
                        else
                            v := ue_of(peek_i, k);
                            used := 2 * k + 1;
                            if v = 0 then
                                i4 <= '1';
                                st <= S_MODES;
                            elsif v <= 24 then
                                i4  <= '0';
                                -- mb_type - 1 packs mode, cbp_chroma and the
                                -- single luma bit.
                                m16 <= to_unsigned((v - 1) mod 4, 2);
                                cbp_c <= to_unsigned(((v - 1) / 4) mod 3, 2);
                                if (v - 1) / 12 /= 0 then
                                    cbp_l <= to_unsigned(15, 4);
                                else
                                    cbp_l <= (others => '0');
                                end if;
                                st <= S_CHROMA;
                            else
                                err_q <= '1'; errc_q <= x"1"; st <= S_DONE;
                                used := 0;
                            end if;
                        end if;
                        sidx <= 0;
                    end if;

                ----------------------------------------------------------
                -- One 4x4 prediction mode per cycle, in block scan order.
                when S_MODES =>
                    if avail_i = '1' then
                        s  := sidx;
                        br := SCAN_BR(s);
                        bc := SCAN_BC(s);

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

                        if not (tok and lok) then pm := 2;
                        elsif mt < ml then pm := mt;
                        else pm := ml;
                        end if;

                        if peek_i(31) = '1' then
                            am   := pm;                  -- same as predicted
                            used := 1;
                        else
                            rm := to_integer(peek_i(30 downto 28));
                            if rm < pm then am := rm; else am := rm + 1; end if;
                            used := 4;
                        end if;

                        modes(br * 4 + bc) <= to_unsigned(am, 4);
                        hist(1)      <= to_unsigned(am, 4);
                        hist(2 to 6) <= hist(1 to 5);

                        if s = 15 then
                            st <= S_CHROMA;
                        else
                            sidx <= sidx + 1;
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_CHROMA =>
                    if avail_i = '1' then
                        k := lz;
                        if k > MAX_K then
                            err_q <= '1'; errc_q <= x"2"; st <= S_DONE;
                        else
                            v := ue_of(peek_i, k);
                            if v > 3 then
                                err_q <= '1'; errc_q <= x"3"; st <= S_DONE;
                            else
                                mchroma <= to_unsigned(v, 2);
                                used    := 2 * k + 1;
                                if i4 = '1' then
                                    st <= S_CBP;
                                else
                                    -- I_16x16 always carries mb_qp_delta.
                                    has_res <= '1';
                                    st <= S_QPD;
                                end if;
                            end if;
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_CBP =>
                    if avail_i = '1' then
                        k := lz;
                        if k > MAX_K then
                            err_q <= '1'; errc_q <= x"2"; st <= S_DONE;
                        else
                            v := ue_of(peek_i, k);
                            if v > 47 then
                                err_q <= '1'; errc_q <= x"4"; st <= S_DONE;
                            else
                                cbp_l <= to_unsigned(CODENUM_CBP(v) mod 16, 4);
                                cbp_c <= to_unsigned(CODENUM_CBP(v) / 16, 2);
                                used  := 2 * k + 1;
                                if CODENUM_CBP(v) = 0 then
                                    has_res <= '0';
                                    st <= S_DONE;
                                else
                                    has_res <= '1';
                                    st <= S_QPD;
                                end if;
                            end if;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- mb_qp_delta se(v). The spec's wrap is a modulo, not a
                -- clamp, so it is written as one here even though a sane
                -- encoder never reaches it.
                when S_QPD =>
                    if avail_i = '1' then
                        k := lz;
                        if k > MAX_K then
                            err_q <= '1'; errc_q <= x"2"; st <= S_DONE;
                        else
                            v := ue_of(peek_i, k);
                            if v > 52 then
                                err_q <= '1'; errc_q <= x"5"; st <= S_DONE;
                            else
                                if (v mod 2) = 1 then
                                    delta := (v + 1) / 2;
                                else
                                    delta := -(v / 2);
                                end if;
                                qsum := to_integer(qp_in) + delta + 104;
                                if    qsum >= 156 then qsum := qsum - 156;
                                elsif qsum >= 104 then qsum := qsum - 104;
                                elsif qsum >= 52  then qsum := qsum - 52;
                                end if;
                                qp_q <= to_unsigned(qsum, 6);
                                used := 2 * k + 1;
                                st   <= S_DONE;
                            end if;
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_DONE =>
                    st <= S_IDLE;

                end case;

                if used > 0 then
                    consume_o   <= '1';
                    consume_n_o <= to_unsigned(used, 6);
                    settling    <= '1';
                    nbits       <= nbits + to_unsigned(used, 8);
                end if;
                end if;
            end if;
        end if;
    end process;

end architecture;
