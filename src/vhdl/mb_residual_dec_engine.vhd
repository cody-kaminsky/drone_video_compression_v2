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

    type state_t is (S_IDLE, S_SETUP, S_WAIT, S_FLUSH, S_DONE);
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

    -- The block the engine is working on, so it can be presented when the
    -- engine finishes even though the NEXT block's setup has by then
    -- overwritten the engine's inputs.
    signal cur_kind : unsigned(1 downto 0) := (others => '0');
    signal cur_comp : std_logic := '0';
    signal cur_pos  : unsigned(3 downto 0) := (others => '0');
    -- The engine finished but the consumer had not taken the previous block.
    signal pend     : std_logic := '0';

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
        variable l      : lnc_arr;
        variable cu, cv : cnc_arr;
        variable k      : unsigned(1 downto 0);
        variable c      : std_logic;
        variable ps     : unsigned(3 downto 0);
        variable ncv    : signed(7 downto 0);
        variable bt     : block_type_t;
        variable ncf    : unsigned(4 downto 0);
        variable coded  : boolean;
        variable free   : boolean;
        variable nxt    : integer range 0 to 27;

        -- Everything about block number s of the 27: its tag, the engine's
        -- job for it, and whether the coded_block_pattern says it is in the
        -- stream at all. nC comes from the arrays passed in, which lets the
        -- caller bypass a count the arrays do not hold yet.
        procedure setup(s : in integer; l : in lnc_arr; cu, cv : in cnc_arr;
                        k : out unsigned(1 downto 0); c : out std_logic;
                        ps : out unsigned(3 downto 0); ncv : out signed(7 downto 0);
                        bt : out block_type_t; ncf : out unsigned(4 downto 0);
                        coded : out boolean) is
            variable si, br, bc, pos : integer range 0 to 15;
            variable na, nb : integer range 0 to 31;
            variable a_top, a_left : boolean;
            variable ci   : integer range 0 to 3;
            variable comp : integer range 0 to 1;
        begin
            na := 0; nb := 0; a_top := false; a_left := false;
            c := '0'; ps := (others => '0');
            if s = 0 then
                -- Luma DC, present for every I_16x16 macroblock whatever the
                -- coded_block_pattern says. nC from the neighbours of block 0.
                k := "00"; bt := to_unsigned(BLK_LUMA_DC_16x16, 3);
                ncf := to_unsigned(16, 5);
                a_top := (atop = '1'); a_left := (aleft = '1');
                if a_top  then nb := slice5(nct, 0); end if;
                if a_left then na := slice5(ncl, 0); end if;
                coded := (i4 = '0');
            elsif s <= 16 then
                si := s - 1; br := SCAN_BR(si); bc := SCAN_BC(si);
                pos := br * 4 + bc;
                k := "01"; ps := to_unsigned(pos, 4);
                if i4 = '1' then
                    bt := to_unsigned(BLK_LUMA_FULL, 3); ncf := to_unsigned(16, 5);
                else
                    bt := to_unsigned(BLK_LUMA_AC, 3);   ncf := to_unsigned(15, 5);
                end if;
                a_left := (bc > 0) or (aleft = '1');
                a_top  := (br > 0) or (atop = '1');
                if a_left then
                    if bc > 0 then na := to_integer(l(pos - 1));
                    else            na := slice5(ncl, br);
                    end if;
                end if;
                if a_top then
                    if br > 0 then nb := to_integer(l(pos - 4));
                    else            nb := slice5(nct, bc);
                    end if;
                end if;
                coded := (cbp_l(si / 4) = '1');
            elsif s <= 18 then
                -- Chroma DC has no nC of its own and feeds no neighbour.
                k := "10"; if s = 18 then c := '1'; end if;
                bt := to_unsigned(BLK_CHROMA_DC, 3); ncf := to_unsigned(4, 5);
                coded := (cbp_c /= 0);
            else
                if s <= 22 then comp := 0; ci := s - 19;
                else            comp := 1; ci := s - 23;
                end if;
                br := ci / 2; bc := ci mod 2;
                k := "11"; if comp = 1 then c := '1'; end if;
                ps := to_unsigned(ci, 4);
                bt := to_unsigned(BLK_CHROMA_AC, 3); ncf := to_unsigned(15, 5);
                a_left := (bc > 0) or (aleft = '1');
                a_top  := (br > 0) or (atop = '1');
                if a_left then
                    if bc > 0 then
                        if comp = 0 then na := to_integer(cu(ci - 1));
                        else             na := to_integer(cv(ci - 1));
                        end if;
                    else
                        if comp = 0 then na := slice5(ncul, br);
                        else             na := slice5(ncvl, br);
                        end if;
                    end if;
                end if;
                if a_top then
                    if br > 0 then
                        if comp = 0 then nb := to_integer(cu(ci - 2));
                        else             nb := to_integer(cv(ci - 2));
                        end if;
                    else
                        if comp = 0 then nb := slice5(ncut, bc);
                        else             nb := slice5(ncvt, bc);
                        end if;
                    end if;
                end if;
                coded := (cbp_c = 2);
            end if;
            if s = 17 or s = 18 then
                ncv := to_signed(-1, 8);
            else
                ncv := to_signed(calc_nc(nb, na, a_top, a_left), 8);
            end if;
        end procedure;

        -- Record a finished block's count where its neighbours will look.
        procedure store(k : in unsigned(1 downto 0); c : in std_logic;
                        ps : in unsigned(3 downto 0); t : in unsigned(4 downto 0)) is
        begin
            if k = "01" then
                lnc(to_integer(ps)) <= t;
            elsif k = "11" then
                if c = '0' then cncu(to_integer(ps(1 downto 0))) <= t;
                else            cncv(to_integer(ps(1 downto 0))) <= t;
                end if;
            end if;
        end procedure;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                st      <= S_IDLE;
                e_start <= '0';
                b_valid <= '0';
                pend    <= '0';
                err_q   <= '0';
                errc_q  <= (others => '0');
                seq     <= 0;
            else
                e_start <= '0';
                -- The consumer takes the presented block at this edge, or
                -- there is none: either way a new one can be presented now.
                free := (b_valid = '0') or (blk_ready_i = '1');
                if b_valid = '1' and blk_ready_i = '1' then b_valid <= '0'; end if;

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
                        pend   <= '0';
                        seq    <= 0;
                        st     <= S_SETUP;
                    end if;

                ----------------------------------------------------------
                -- Set up block `seq` from the counts as stored. A coded block
                -- goes to the engine; an uncoded one is presented as zeros
                -- right here, one per cycle, and still contributes its 0.
                when S_SETUP =>
                    setup(seq, lnc, cncu, cncv, k, c, ps, ncv, bt, ncf, coded);
                    if coded then
                        e_nc <= ncv; e_btype <= bt; e_ncoef <= ncf;
                        e_start  <= '1';
                        cur_kind <= k; cur_comp <= c; cur_pos <= ps;
                        st <= S_WAIT;
                    elsif free then
                        b_kind <= k; b_comp <= c; b_pos <= ps;
                        b_total <= (others => '0');
                        b_coefs <= (others => '0');
                        b_valid <= '1';
                        store(k, c, ps, (others => '0'));
                        if seq = 26 then
                            st <= S_FLUSH;
                        else
                            seq <= seq + 1;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- The engine has the block. When it finishes, present the
                -- result and, in the same cycle, set up the next block with
                -- this one's count bypassed into the neighbour arithmetic,
                -- so a coded block follows a coded block with one idle cycle
                -- rather than three.
                when S_WAIT =>
                    if (e_done = '1' or pend = '1') and e_err = '1' then
                        err_q  <= '1';
                        errc_q <= e_errc;
                        pend   <= '0';
                        st     <= S_DONE;
                    elsif (e_done = '1' or pend = '1') and free then
                        pend <= '0';
                        b_kind <= cur_kind; b_comp <= cur_comp; b_pos <= cur_pos;
                        b_total <= e_total;
                        -- I_16x16 luma AC and chroma AC carry 15 coefficients
                        -- that start at zigzag index 1.
                        if (cur_kind = "01" and i4 = '0') or cur_kind = "11" then
                            b_coefs <= e_coefs(239 downto 0) & x"0000";
                        else
                            b_coefs <= e_coefs;
                        end if;
                        b_valid <= '1';
                        store(cur_kind, cur_comp, cur_pos, e_total);
                        if seq = 26 then
                            st <= S_FLUSH;
                        else
                            nxt := seq + 1;
                            l := lnc; cu := cncu; cv := cncv;
                            if cur_kind = "01" then
                                l(to_integer(cur_pos)) := e_total;
                            elsif cur_kind = "11" then
                                if cur_comp = '0' then cu(to_integer(cur_pos(1 downto 0))) := e_total;
                                else                   cv(to_integer(cur_pos(1 downto 0))) := e_total;
                                end if;
                            end if;
                            setup(nxt, l, cu, cv, k, c, ps, ncv, bt, ncf, coded);
                            seq <= nxt;
                            if coded then
                                e_nc <= ncv; e_btype <= bt; e_ncoef <= ncf;
                                e_start  <= '1';
                                cur_kind <= k; cur_comp <= c; cur_pos <= ps;
                            else
                                st <= S_SETUP;
                            end if;
                        end if;
                    elsif e_done = '1' then
                        -- Finished, but the previous block is still waiting
                        -- to be taken. The engine's outputs hold until its
                        -- next start, so remember and present when free.
                        pend <= '1';
                    end if;

                ----------------------------------------------------------
                -- done_o means every block has been TAKEN, not merely
                -- presented: the caller reads the counts at done and the
                -- consumer may still be holding the last block off.
                when S_FLUSH =>
                    if b_valid = '0' or blk_ready_i = '1' then
                        st <= S_DONE;
                    end if;

                when S_DONE =>
                    st <= S_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture;
