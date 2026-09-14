--------------------------------------------------------------------------------
-- mb_residual_dec_engine.vhd
--
-- Walks one macroblock's residual blocks in bitstream order, deriving each
-- one's nC from its neighbours and driving a cavlc_dec_engine. The decode-side
-- counterpart of what cavlc_dispatch does for the encoder -- but not its
-- mirror image, because the two directions have opposite parallelism.
-- Encoding a block needs no knowledge of the block before it, so the encoder
-- runs several engines at once and merges. Decoding one cannot start until
-- the previous block's length is known, so there is exactly one engine here
-- and the sequencer's whole job is bookkeeping.
--
-- Block order, spec 7.3.5.3:
--   luma DC (I_16x16 only), 16 luma blocks in scan order, two chroma DC
--   blocks, then eight chroma AC blocks, U before V.
--
-- The stream out is a fixed 27 blocks whatever the macroblock contains.
-- Blocks the coded_block_pattern leaves out are emitted as zeros rather than
-- skipped, so the consumer's loop does not have to know the pattern -- and
-- an uncoded block still contributes its 0 to its neighbours' nC, which is a
-- rule easier to get right when the block is present than when it is absent.
--
-- Coefficients come out in zigzag order with the I_16x16 and chroma AC shift
-- already applied, so every block is a full 16-coefficient vector whatever
-- its type. That matches dec_mb_residual in the golden decoder.
--
-- Neighbour total_coeff comes in and goes out rather than being looked up: a
-- line buffer holds the row above and the column to the left, and everything
-- inside the macroblock comes from blocks this engine has already decoded.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

entity mb_residual_dec_engine is
    port (
        clk          : in  std_logic;
        rst_n        : in  std_logic;

        -- Job
        start_i      : in  std_logic;
        ready_o      : out std_logic;
        is_i4x4_i    : in  std_logic;
        cbp_luma_i   : in  unsigned(3 downto 0);
        cbp_chroma_i : in  unsigned(1 downto 0);
        avail_top_i  : in  std_logic;
        avail_left_i : in  std_logic;
        -- Neighbour total_coeff. Luma: 4 x 5 bits, top by column and left by
        -- row. Chroma: 2 x 5 bits each, per component.
        nc_top_i     : in  unsigned(19 downto 0);
        nc_left_i    : in  unsigned(19 downto 0);
        ncu_top_i    : in  unsigned(9 downto 0);
        ncu_left_i   : in  unsigned(9 downto 0);
        ncv_top_i    : in  unsigned(9 downto 0);
        ncv_left_i   : in  unsigned(9 downto 0);

        -- Bit reader
        peek_i       : in  unsigned(31 downto 0);
        avail_i      : in  std_logic;
        consume_o    : out std_logic;
        consume_n_o  : out unsigned(5 downto 0);

        -- Block stream out, 27 blocks per macroblock
        blk_valid_o  : out std_logic;
        blk_ready_i  : in  std_logic;
        blk_kind_o   : out unsigned(1 downto 0);   -- 0 lumaDC 1 luma 2 chrDC 3 chrAC
        blk_comp_o   : out std_logic;              -- 0 U, 1 V
        blk_pos_o    : out unsigned(3 downto 0);   -- raster within the MB
        blk_total_o  : out unsigned(4 downto 0);
        blk_coefs_o  : out std_logic_vector(16 * 16 - 1 downto 0);

        -- Result
        done_o       : out std_logic;
        err_o        : out std_logic;
        err_code_o   : out unsigned(3 downto 0);
        -- total_coeff of every block in the macroblock, packed exactly as
        -- line_buffer's commit ports expect: raster order, entry k at
        -- (5k+4 downto 5k). The line buffer derives the row and column the
        -- neighbours need, so handing it the edges separately would be a
        -- second chance to pack them wrongly.
        nc_y_o       : out std_logic_vector(79 downto 0);
        nc_u_o       : out std_logic_vector(19 downto 0);
        nc_v_o       : out std_logic_vector(19 downto 0)
    );
end entity;

architecture rtl of mb_residual_dec_engine is

    type state_t is (S_IDLE, S_SETUP, S_WAIT, S_EMIT, S_DONE);
    signal st : state_t := S_IDLE;

    type i16_tab is array (0 to 15) of integer range 0 to 3;
    constant SCAN_BR : i16_tab := (0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3);
    constant SCAN_BC : i16_tab := (0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3);

    -- Job context
    signal i4      : std_logic := '0';
    signal cbp_l   : unsigned(3 downto 0) := (others => '0');
    signal cbp_c   : unsigned(1 downto 0) := (others => '0');
    signal atop    : std_logic := '0';
    signal aleft   : std_logic := '0';
    signal nct, ncl   : unsigned(19 downto 0) := (others => '0');
    signal ncut, ncul : unsigned(9 downto 0) := (others => '0');
    signal ncvt, ncvl : unsigned(9 downto 0) := (others => '0');

    -- total_coeff of this macroblock's own blocks
    type lnc_arr is array (0 to 15) of unsigned(4 downto 0);
    type cnc_arr is array (0 to 3)  of unsigned(4 downto 0);
    signal lnc  : lnc_arr := (others => (others => '0'));
    signal cncu : cnc_arr := (others => (others => '0'));
    signal cncv : cnc_arr := (others => (others => '0'));

    signal seq : integer range 0 to 27 := 0;

    -- cavlc_dec_engine
    signal e_start : std_logic := '0';
    signal e_ready : std_logic;
    signal e_nc    : signed(7 downto 0) := (others => '0');
    signal e_btype : block_type_t := (others => '0');
    signal e_ncoef : unsigned(4 downto 0) := (others => '0');
    signal e_done  : std_logic;
    signal e_err   : std_logic;
    signal e_errc  : unsigned(3 downto 0);
    signal e_total : unsigned(4 downto 0);
    signal e_coefs : std_logic_vector(255 downto 0);

    -- Output registers
    signal b_valid : std_logic := '0';
    signal b_kind  : unsigned(1 downto 0) := (others => '0');
    signal b_comp  : std_logic := '0';
    signal b_pos   : unsigned(3 downto 0) := (others => '0');
    signal b_total : unsigned(4 downto 0) := (others => '0');
    signal b_coefs : std_logic_vector(255 downto 0) := (others => '0');

    signal err_q  : std_logic := '0';
    signal errc_q : unsigned(3 downto 0) := (others => '0');

    -- nC from the two neighbours, spec 9.2.1.1.
    function calc_nc(nb, na : integer; a_top, a_left : boolean) return integer is
    begin
        if a_top and a_left then return (nb + na + 1) / 2; end if;
        if a_top  then return nb; end if;
        if a_left then return na; end if;
        return 0;
    end function;

    function slice5(v : unsigned; k : integer) return integer is
    begin
        return to_integer(v(5 * k + 4 downto 5 * k));
    end function;

begin

    eng : entity work.cavlc_dec_engine
        port map (clk => clk, rst_n => rst_n,
                  start_i => e_start, ready_o => e_ready,
                  nc_i => e_nc, btype_i => e_btype, n_coefs_i => e_ncoef,
                  peek_i => peek_i, avail_i => avail_i,
                  consume_o => consume_o, consume_n_o => consume_n_o,
                  done_o => e_done, err_o => e_err, err_code_o => e_errc,
                  total_coeff_o => e_total, coefs_o => e_coefs);

    ready_o     <= '1' when st = S_IDLE else '0';
    done_o      <= '1' when st = S_DONE else '0';
    err_o       <= err_q;
    err_code_o  <= errc_q;
    blk_valid_o <= b_valid;
    blk_kind_o  <= b_kind;
    blk_comp_o  <= b_comp;
    blk_pos_o   <= b_pos;
    blk_total_o <= b_total;
    blk_coefs_o <= b_coefs;

    g_nc : for i in 0 to 15 generate
        nc_y_o(5 * i + 4 downto 5 * i) <= std_logic_vector(lnc(i));
    end generate;
    g_ncc : for i in 0 to 3 generate
        nc_u_o(5 * i + 4 downto 5 * i) <= std_logic_vector(cncu(i));
        nc_v_o(5 * i + 4 downto 5 * i) <= std_logic_vector(cncv(i));
    end generate;

    main_p : process(clk)
        variable s      : integer range 0 to 15;
        variable br, bc : integer range 0 to 3;
        variable pos    : integer range 0 to 15;
        variable na, nb : integer range 0 to 31;
        variable a_top  : boolean;
        variable a_left : boolean;
        variable coded  : boolean;
        variable ci     : integer range 0 to 3;
        variable comp   : integer range 0 to 1;
        variable shifted : std_logic_vector(255 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                st      <= S_IDLE;
                e_start <= '0';
                b_valid <= '0';
                err_q   <= '0';
                errc_q  <= (others => '0');
                seq     <= 0;
            else
                e_start <= '0';

                case st is

                ----------------------------------------------------------
                when S_IDLE =>
                    if start_i = '1' then
                        i4     <= is_i4x4_i;
                        cbp_l  <= cbp_luma_i;
                        cbp_c  <= cbp_chroma_i;
                        atop   <= avail_top_i;
                        aleft  <= avail_left_i;
                        nct    <= nc_top_i;   ncl  <= nc_left_i;
                        ncut   <= ncu_top_i;  ncul <= ncu_left_i;
                        ncvt   <= ncv_top_i;  ncvl <= ncv_left_i;
                        lnc    <= (others => (others => '0'));
                        cncu   <= (others => (others => '0'));
                        cncv   <= (others => (others => '0'));
                        err_q  <= '0';
                        errc_q <= (others => '0');
                        seq    <= 0;
                        st     <= S_SETUP;
                    end if;

                ----------------------------------------------------------
                -- Work out this block's type, size and nC, then either start
                -- the engine or emit zeros.
                when S_SETUP =>
                    coded := false;
                    na := 0; nb := 0; a_top := false; a_left := false;
                    b_comp <= '0';
                    b_pos  <= (others => '0');

                    if seq = 0 then
                        -- Luma DC, present for every I_16x16 macroblock
                        -- whatever the coded_block_pattern says. nC comes from
                        -- the neighbours of block 0.
                        b_kind  <= to_unsigned(0, 2);
                        e_btype <= to_unsigned(BLK_LUMA_DC_16x16, 3);
                        e_ncoef <= to_unsigned(16, 5);
                        a_top   := (atop = '1');
                        a_left  := (aleft = '1');
                        if a_top  then nb := slice5(nct, 0); end if;
                        if a_left then na := slice5(ncl, 0); end if;
                        coded := (i4 = '0');

                    elsif seq <= 16 then
                        s   := seq - 1;
                        br  := SCAN_BR(s);
                        bc  := SCAN_BC(s);
                        pos := br * 4 + bc;
                        b_kind <= to_unsigned(1, 2);
                        b_pos  <= to_unsigned(pos, 4);
                        if i4 = '1' then
                            e_btype <= to_unsigned(BLK_LUMA_FULL, 3);
                            e_ncoef <= to_unsigned(16, 5);
                        else
                            e_btype <= to_unsigned(BLK_LUMA_AC, 3);
                            e_ncoef <= to_unsigned(15, 5);
                        end if;
                        a_left := (bc > 0) or (aleft = '1');
                        a_top  := (br > 0) or (atop = '1');
                        if a_left then
                            if bc > 0 then na := to_integer(lnc(pos - 1));
                            else            na := slice5(ncl, br);
                            end if;
                        end if;
                        if a_top then
                            if br > 0 then nb := to_integer(lnc(pos - 4));
                            else            nb := slice5(nct, bc);
                            end if;
                        end if;
                        coded := (cbp_l(s / 4) = '1');

                    elsif seq <= 18 then
                        -- Chroma DC has no nC of its own and feeds no
                        -- neighbour: its total_coeff is never stored.
                        b_kind  <= to_unsigned(2, 2);
                        if seq = 18 then b_comp <= '1'; else b_comp <= '0'; end if;
                        e_btype <= to_unsigned(BLK_CHROMA_DC, 3);
                        e_ncoef <= to_unsigned(4, 5);
                        coded   := (cbp_c /= 0);

                    else
                        if seq <= 22 then
                            comp := 0; ci := seq - 19;
                        else
                            comp := 1; ci := seq - 23;
                        end if;
                        br := ci / 2;
                        bc := ci mod 2;
                        b_kind  <= to_unsigned(3, 2);
                        if comp = 1 then b_comp <= '1'; else b_comp <= '0'; end if;
                        b_pos   <= to_unsigned(ci, 4);
                        e_btype <= to_unsigned(BLK_CHROMA_AC, 3);
                        e_ncoef <= to_unsigned(15, 5);
                        a_left := (bc > 0) or (aleft = '1');
                        a_top  := (br > 0) or (atop = '1');
                        if a_left then
                            if bc > 0 then
                                if comp = 0 then na := to_integer(cncu(ci - 1));
                                else             na := to_integer(cncv(ci - 1));
                                end if;
                            else
                                if comp = 0 then na := slice5(ncul, br);
                                else             na := slice5(ncvl, br);
                                end if;
                            end if;
                        end if;
                        if a_top then
                            if br > 0 then
                                if comp = 0 then nb := to_integer(cncu(ci - 2));
                                else             nb := to_integer(cncv(ci - 2));
                                end if;
                            else
                                if comp = 0 then nb := slice5(ncut, bc);
                                else             nb := slice5(ncvt, bc);
                                end if;
                            end if;
                        end if;
                        coded := (cbp_c = 2);
                    end if;

                    if seq >= 17 and seq <= 18 then
                        e_nc <= to_signed(-1, 8);      -- chroma DC table
                    else
                        e_nc <= to_signed(calc_nc(nb, na, a_top, a_left), 8);
                    end if;

                    if coded then
                        e_start <= '1';
                        st      <= S_WAIT;
                    else
                        b_total <= (others => '0');
                        b_coefs <= (others => '0');
                        b_valid <= '1';
                        st      <= S_EMIT;
                    end if;

                ----------------------------------------------------------
                when S_WAIT =>
                    if e_done = '1' then
                        if e_err = '1' then
                            err_q  <= '1';
                            errc_q <= e_errc;
                            st     <= S_DONE;
                        else
                            -- I_16x16 luma AC and chroma AC carry 15
                            -- coefficients that start at zigzag index 1.
                            if (seq >= 1 and seq <= 16 and i4 = '0')
                               or seq >= 19 then
                                shifted := e_coefs(239 downto 0) & x"0000";
                            else
                                shifted := e_coefs;
                            end if;
                            b_coefs <= shifted;
                            b_total <= e_total;
                            b_valid <= '1';
                            st      <= S_EMIT;
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_EMIT =>
                    if blk_ready_i = '1' then
                        b_valid <= '0';
                        -- An uncoded block still contributes its 0.
                        if seq >= 1 and seq <= 16 then
                            lnc(to_integer(b_pos)) <= b_total;
                        elsif seq >= 19 and seq <= 22 then
                            cncu(to_integer(b_pos)) <= b_total;
                        elsif seq >= 23 then
                            cncv(to_integer(b_pos)) <= b_total;
                        end if;
                        if seq = 26 then
                            st <= S_DONE;
                        else
                            seq <= seq + 1;
                            st  <= S_SETUP;
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_DONE =>
                    st <= S_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture;
