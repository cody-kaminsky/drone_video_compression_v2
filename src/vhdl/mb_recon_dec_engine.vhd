--------------------------------------------------------------------------------
-- mb_recon_dec_engine.vhd
--
-- Reconstructs one macroblock from the block stream mb_residual_dec_engine
-- produces, doing what decode_mb in src/dec/decoder.c does after the parse:
--
--   luma DC (I_16x16) : iquant_dc_4x4 then ihadamard4x4, giving one DC per
--                       4x4 block
--   chroma DC         : iquant_dc_2x2 then ihadamard2x2, per component
--   every 4x4 block   : predict, iquant_4x4 (DC substituted from the
--                       Hadamard plane where there is one), idct4x4, then
--                       clip8(pred + ((res + 32) >> 6))
--
-- None of that arithmetic is new. quant_engine already has the three inverse
-- modes because the encoder reconstructs; transform_engine already has the
-- three inverse transforms; recon_engine already folds the rounding, the
-- shift and the clip. They are instantiated here with DIR = "inv" and
-- WITH_SSD = false, which drops the forward operand muxes and the distortion
-- adder tree the decoder has no use for.
--
-- The I_4x4 neighbour gather is lifted from mode_decide_engine's own
-- reconstruction path rather than rewritten, including the two rules that
-- are easy to get wrong: the above-right 4x4 exists only for scan positions
-- other than 3, 7, 11, 13 and 15, and when it does not exist the four
-- samples are replicated from the last one of the row above (spec 8.3.1.2.4)
-- rather than treated as unavailable.
--
-- Blocks are processed one at a time, each walking predict -> dequantise ->
-- inverse transform -> reconstruct before the next begins. I_4x4 forces that
-- serialisation anyway -- block s+1 predicts from block s's reconstruction --
-- and the entropy decode ahead of it is the slower half, so there is nothing
-- to gain here yet. If that changes, the I_16x16 and chroma paths have no
-- such dependency and can be pipelined without touching the interface.
--
-- The reconstruction comes out as 24 blocks, Y raster 0..15 then U 0..3 then
-- V 0..3, which is the stream line_buffer's commit path already consumes.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mb_recon_dec_engine is
    port (
        clk           : in  std_logic;
        rst_n         : in  std_logic;

        -- Job
        start_i       : in  std_logic;
        ready_o       : out std_logic;
        done_o        : out std_logic;
        is_i4x4_i     : in  std_logic;
        mode16_i      : in  unsigned(1 downto 0);
        modes4_i      : in  std_logic_vector(63 downto 0);   -- 16 x 4 bits, raster
        mode_chroma_i : in  unsigned(1 downto 0);
        qp_y_i        : in  unsigned(5 downto 0);
        qp_c_i        : in  unsigned(5 downto 0);

        -- Neighbours, the line_buffer bundle
        top_y_i       : in  std_logic_vector(127 downto 0);
        tr_y_i        : in  std_logic_vector(31 downto 0);
        tl_y_i        : in  std_logic_vector(7 downto 0);
        left_y_i      : in  std_logic_vector(127 downto 0);
        top_u_i       : in  std_logic_vector(63 downto 0);
        left_u_i      : in  std_logic_vector(63 downto 0);
        tl_u_i        : in  std_logic_vector(7 downto 0);
        top_v_i       : in  std_logic_vector(63 downto 0);
        left_v_i      : in  std_logic_vector(63 downto 0);
        tl_v_i        : in  std_logic_vector(7 downto 0);
        avail_top_i   : in  std_logic;
        avail_left_i  : in  std_logic;
        avail_tl_i    : in  std_logic;
        avail_tr_i    : in  std_logic;

        -- Block stream from mb_residual_dec_engine, 27 blocks
        blk_valid_i   : in  std_logic;
        blk_ready_o   : out std_logic;
        blk_kind_i    : in  unsigned(1 downto 0);
        blk_comp_i    : in  std_logic;
        blk_pos_i     : in  unsigned(3 downto 0);
        blk_coefs_i   : in  std_logic_vector(16 * 16 - 1 downto 0);

        -- Reconstruction, 24 blocks: Y raster 0..15, U 0..3, V 0..3
        rec_valid_o   : out std_logic;
        rec_ready_i   : in  std_logic;
        rec_plane_o   : out unsigned(1 downto 0);
        rec_idx_o     : out unsigned(3 downto 0);
        rec_data_o    : out std_logic_vector(127 downto 0)
    );
end entity;

architecture rtl of mb_recon_dec_engine is

    type int16_t is array (0 to 15) of integer;
    -- Raster position of zigzag coefficient k, so raster(ZIGZAG(k)) = zz(k).
    constant ZIGZAG  : int16_t := (0,1,4,8,5,2,3,6,9,12,13,10,7,11,14,15);
    constant SCAN_BR : int16_t := (0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3);
    constant SCAN_BC : int16_t := (0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3);

    -- The above-right 4x4 of a block, in the spec's scan, belongs to a block
    -- that has not been decoded yet at exactly these positions.
    function tr_avail_blk(s : integer) return boolean is
    begin
        return not (s = 3 or s = 7 or s = 11 or s = 13 or s = 15);
    end function;

    function byte_of(v : std_logic_vector; k : integer) return std_logic_vector is
    begin
        return v(8 * k + 7 downto 8 * k);
    end function;

    type c16_t  is array (0 to 15) of signed(31 downto 0);
    type blk_t  is array (0 to 15) of std_logic_vector(127 downto 0);
    type c4_t   is array (0 to 3)  of std_logic_vector(127 downto 0);

    type state_t is (S_IDLE, S_TAKE,
                     S_PRED_ISSUE, S_PRED_WAIT,
                     S_Q_ISSUE, S_Q_WAIT,
                     S_T_ISSUE, S_T_WAIT,
                     S_R_ISSUE, S_R_WAIT,
                     S_EMIT, S_DONE);
    signal st : state_t := S_IDLE;

    -- Job context
    signal i4      : std_logic := '0';
    signal m16     : unsigned(1 downto 0) := (others => '0');
    signal modes4  : std_logic_vector(63 downto 0) := (others => '0');
    signal mchroma : unsigned(1 downto 0) := (others => '0');
    signal qp_y, qp_c : unsigned(5 downto 0) := (others => '0');
    signal top_y, left_y : std_logic_vector(127 downto 0) := (others => '0');
    signal tr_y  : std_logic_vector(31 downto 0) := (others => '0');
    signal tl_y  : std_logic_vector(7 downto 0) := (others => '0');
    signal top_u, left_u, top_v, left_v : std_logic_vector(63 downto 0) := (others => '0');
    signal tl_u, tl_v : std_logic_vector(7 downto 0) := (others => '0');
    signal a_top, a_left, a_tl, a_tr : std_logic := '0';

    -- Current block
    signal cur_kind : unsigned(1 downto 0) := (others => '0');
    signal cur_comp : std_logic := '0';
    signal cur_pos  : unsigned(3 downto 0) := (others => '0');
    signal cur_s    : integer range 0 to 15 := 0;   -- scan index, luma only
    signal nblk     : integer range 0 to 27 := 0;
    signal lev      : c16_t := (others => (others => '0'));   -- raster levels
    signal pred_q   : std_logic_vector(127 downto 0) := (others => '0');
    signal coef_q   : c16_t := (others => (others => '0'));
    signal res_q    : std_logic_vector(16 * 20 - 1 downto 0) := (others => '0');

    -- Results
    signal r4  : blk_t := (others => (others => '0'));
    signal rcu : c4_t  := (others => (others => '0'));
    signal rcv : c4_t  := (others => (others => '0'));
    signal dcy : c16_t := (others => (others => '0'));
    signal dcu : c16_t := (others => (others => '0'));
    signal dcv : c16_t := (others => (others => '0'));

    signal emit_i : integer range 0 to 24 := 0;

    -- quant_engine
    signal q_mode  : unsigned(2 downto 0) := (others => '0');
    signal q_qp    : unsigned(5 downto 0) := (others => '0');
    signal q_valid : std_logic := '0';
    signal q_ready : std_logic;
    signal q_vo    : std_logic;
    signal q_din, q_dout : c16_t;

    -- transform_engine
    signal t_mode  : unsigned(2 downto 0) := (others => '0');
    signal t_valid : std_logic := '0';
    signal t_ready : std_logic;
    signal t_vo    : std_logic;
    signal t_din, t_dout : c16_t;

    -- recon_engine
    signal rc_valid : std_logic := '0';
    signal rc_ready : std_logic;
    signal rc_vo    : std_logic;
    signal rc_out   : std_logic_vector(127 downto 0);

    -- predictors
    signal p4_mode  : unsigned(3 downto 0) := (others => '0');
    signal p4_top   : std_logic_vector(63 downto 0) := (others => '0');
    signal p4_left  : std_logic_vector(31 downto 0) := (others => '0');
    signal p4_tl    : std_logic_vector(7 downto 0) := (others => '0');
    signal p4_at, p4_al, p4_atl : std_logic := '0';
    signal p4_valid : std_logic := '0';
    signal p4_ready, p4_vo : std_logic;
    signal p4_pred  : std_logic_vector(127 downto 0);

    signal p16_blk   : unsigned(3 downto 0) := (others => '0');
    signal p16_valid : std_logic := '0';
    signal p16_ready, p16_vo : std_logic;
    signal p16_pred  : std_logic_vector(127 downto 0);

    signal pc_blk   : unsigned(1 downto 0) := (others => '0');
    signal pc_top, pc_left : std_logic_vector(63 downto 0) := (others => '0');
    signal pc_tl    : std_logic_vector(7 downto 0) := (others => '0');
    signal pc_valid : std_logic := '0';
    signal pc_ready, pc_vo : std_logic;
    signal pc_pred  : std_logic_vector(127 downto 0);

    signal b_valid : std_logic := '0';
    signal b_plane : unsigned(1 downto 0) := (others => '0');
    signal b_idx   : unsigned(3 downto 0) := (others => '0');
    signal b_data  : std_logic_vector(127 downto 0) := (others => '0');

begin

    ready_o     <= '1' when st = S_IDLE else '0';
    done_o      <= '1' when st = S_DONE else '0';
    blk_ready_o <= '1' when st = S_TAKE else '0';
    rec_valid_o <= b_valid;
    rec_plane_o <= b_plane;
    rec_idx_o   <= b_idx;
    rec_data_o  <= b_data;

    ------------------------------------------------------------------
    quant : entity work.quant_engine
        port map (clk => clk, rst_n => rst_n,
                  mode_i => q_mode, qp_i => q_qp,
                  valid_i => q_valid, ready_o => q_ready,
                  valid_o => q_vo, ready_i => '1',
                  din_0 => q_din(0),   din_1 => q_din(1),
                  din_2 => q_din(2),   din_3 => q_din(3),
                  din_4 => q_din(4),   din_5 => q_din(5),
                  din_6 => q_din(6),   din_7 => q_din(7),
                  din_8 => q_din(8),   din_9 => q_din(9),
                  din_10 => q_din(10), din_11 => q_din(11),
                  din_12 => q_din(12), din_13 => q_din(13),
                  din_14 => q_din(14), din_15 => q_din(15),
                  dout_0 => q_dout(0),   dout_1 => q_dout(1),
                  dout_2 => q_dout(2),   dout_3 => q_dout(3),
                  dout_4 => q_dout(4),   dout_5 => q_dout(5),
                  dout_6 => q_dout(6),   dout_7 => q_dout(7),
                  dout_8 => q_dout(8),   dout_9 => q_dout(9),
                  dout_10 => q_dout(10), dout_11 => q_dout(11),
                  dout_12 => q_dout(12), dout_13 => q_dout(13),
                  dout_14 => q_dout(14), dout_15 => q_dout(15),
                  deq_0 => open,  deq_1 => open,  deq_2 => open,  deq_3 => open,
                  deq_4 => open,  deq_5 => open,  deq_6 => open,  deq_7 => open,
                  deq_8 => open,  deq_9 => open,  deq_10 => open, deq_11 => open,
                  deq_12 => open, deq_13 => open, deq_14 => open, deq_15 => open,
                  deq_valid_o => open);

    xform : entity work.transform_engine
        generic map (W => 20, DIR => "inv")
        port map (clk => clk, rst_n => rst_n,
                  mode_i => t_mode,
                  valid_i => t_valid, ready_o => t_ready,
                  valid_o => t_vo, ready_i => '1',
                  din_0 => t_din(0),   din_1 => t_din(1),
                  din_2 => t_din(2),   din_3 => t_din(3),
                  din_4 => t_din(4),   din_5 => t_din(5),
                  din_6 => t_din(6),   din_7 => t_din(7),
                  din_8 => t_din(8),   din_9 => t_din(9),
                  din_10 => t_din(10), din_11 => t_din(11),
                  din_12 => t_din(12), din_13 => t_din(13),
                  din_14 => t_din(14), din_15 => t_din(15),
                  dout_0 => t_dout(0),   dout_1 => t_dout(1),
                  dout_2 => t_dout(2),   dout_3 => t_dout(3),
                  dout_4 => t_dout(4),   dout_5 => t_dout(5),
                  dout_6 => t_dout(6),   dout_7 => t_dout(7),
                  dout_8 => t_dout(8),   dout_9 => t_dout(9),
                  dout_10 => t_dout(10), dout_11 => t_dout(11),
                  dout_12 => t_dout(12), dout_13 => t_dout(13),
                  dout_14 => t_dout(14), dout_15 => t_dout(15));

    recon : entity work.recon_engine
        generic map (RES_W => 20, WITH_SSD => false)
        port map (clk => clk, rst_n => rst_n,
                  pred_i => pred_q, res_i => res_q,
                  src_i => (others => '0'),
                  valid_i => rc_valid, ready_o => rc_ready,
                  recon_o => rc_out, ssd_o => open,
                  valid_o => rc_vo, ready_i => '1');

    p4 : entity work.predict_4x4_engine
        port map (clk => clk, rst_n => rst_n,
                  mode_i => p4_mode, top_i => p4_top, left_i => p4_left,
                  tl_i => p4_tl, avail_top_i => p4_at, avail_left_i => p4_al,
                  avail_tl_i => p4_atl,
                  valid_i => p4_valid, ready_o => p4_ready,
                  pred_o => p4_pred, valid_o => p4_vo, ready_i => '1');

    p16 : entity work.predict_16x16_engine
        port map (clk => clk, rst_n => rst_n,
                  mode_i => m16, blk_i => p16_blk,
                  top_i => top_y, left_i => left_y, tl_i => tl_y,
                  avail_top_i => a_top, avail_left_i => a_left,
                  avail_tl_i => a_tl,
                  valid_i => p16_valid, ready_o => p16_ready,
                  pred_o => p16_pred, valid_o => p16_vo, ready_i => '1');

    pchroma : entity work.predict_chroma_engine
        port map (clk => clk, rst_n => rst_n,
                  mode_i => mchroma, blk_i => pc_blk,
                  top_i => pc_top, left_i => pc_left, tl_i => pc_tl,
                  avail_top_i => a_top, avail_left_i => a_left,
                  avail_tl_i => a_tl,
                  valid_i => pc_valid, ready_o => pc_ready,
                  pred_o => pc_pred, valid_o => pc_vo, ready_i => '1');

    ------------------------------------------------------------------
    main_p : process(clk)
        variable s, br, bc, pos, k : integer range 0 to 15;
        variable topv  : std_logic_vector(63 downto 0);
        variable leftv : std_logic_vector(31 downto 0);
        variable tlv   : std_logic_vector(7 downto 0);
        variable at, al, atl : std_logic;
        variable rb    : std_logic_vector(127 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                st       <= S_IDLE;
                q_valid  <= '0';
                t_valid  <= '0';
                rc_valid <= '0';
                p4_valid <= '0';
                p16_valid<= '0';
                pc_valid <= '0';
                b_valid  <= '0';
                nblk     <= 0;
                emit_i   <= 0;
            else
                q_valid  <= '0';
                t_valid  <= '0';
                rc_valid <= '0';
                p4_valid <= '0';
                p16_valid<= '0';
                pc_valid <= '0';

                case st is

                ----------------------------------------------------------
                when S_IDLE =>
                    if start_i = '1' then
                        i4      <= is_i4x4_i;
                        m16     <= mode16_i;
                        modes4  <= modes4_i;
                        mchroma <= mode_chroma_i;
                        qp_y    <= qp_y_i;
                        qp_c    <= qp_c_i;
                        top_y   <= top_y_i;  left_y <= left_y_i;
                        tr_y    <= tr_y_i;   tl_y   <= tl_y_i;
                        top_u   <= top_u_i;  left_u <= left_u_i; tl_u <= tl_u_i;
                        top_v   <= top_v_i;  left_v <= left_v_i; tl_v <= tl_v_i;
                        a_top   <= avail_top_i;  a_left <= avail_left_i;
                        a_tl    <= avail_tl_i;   a_tr   <= avail_tr_i;
                        nblk    <= 0;
                        emit_i  <= 0;
                        st      <= S_TAKE;
                    end if;

                ----------------------------------------------------------
                -- Take one block and put its coefficients into raster order.
                when S_TAKE =>
                    if blk_valid_i = '1' then
                        cur_kind <= blk_kind_i;
                        cur_comp <= blk_comp_i;
                        cur_pos  <= blk_pos_i;
                        -- Zigzag to raster -- except for a chroma DC block,
                        -- whose four coefficients are already in order and
                        -- must land in lanes 0..3. Permuting them sends
                        -- coefficients 2 and 3 to lanes 4 and 8, which the 2x2
                        -- Hadamard never reads: the block then reconstructs
                        -- flat and slightly wrong, and only when one of those
                        -- two is nonzero.
                        for k in 0 to 15 loop
                            if blk_kind_i = 2 then
                                lev(k) <= resize(signed(
                                    blk_coefs_i(k * 16 + 15 downto k * 16)), 32);
                            else
                                lev(ZIGZAG(k)) <= resize(signed(
                                    blk_coefs_i(k * 16 + 15 downto k * 16)), 32);
                            end if;
                        end loop;
                        -- Luma blocks arrive in scan order; the scan index is
                        -- what the above-right rule is stated in terms of.
                        if nblk >= 1 and nblk <= 16 then
                            cur_s <= nblk - 1;
                        else
                            cur_s <= 0;
                        end if;
                        if blk_kind_i = 1 or blk_kind_i = 3 then
                            st <= S_PRED_ISSUE;
                        else
                            st <= S_Q_ISSUE;     -- a DC block has no prediction
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_PRED_ISSUE =>
                    if cur_kind = 1 then
                        s   := cur_s;
                        br  := SCAN_BR(s);
                        bc  := SCAN_BC(s);
                        if i4 = '1' then
                            -- Neighbour gather, the same one the encoder's
                            -- reconstruction path uses.
                            at := '1'; al := '1';
                            if br = 0 then at := a_top;  end if;
                            if bc = 0 then al := a_left; end if;
                            if br > 0 and bc > 0 then atl := '1';
                            elsif br > 0 then atl := a_left;
                            elsif bc > 0 then atl := a_top;
                            else atl := a_tl;
                            end if;

                            if br > 0 then
                                topv(31 downto 0) := r4((br - 1) * 4 + bc)(127 downto 96);
                            else
                                topv(31 downto 0) := top_y(bc * 32 + 31 downto bc * 32);
                            end if;
                            if tr_avail_blk(s) and ((s /= 5) or a_tr = '1')
                               and at = '1' then
                                if br > 0 then
                                    topv(63 downto 32) :=
                                        r4((br - 1) * 4 + bc + 1)(127 downto 96);
                                elsif s = 5 then
                                    topv(63 downto 32) := tr_y;
                                else
                                    topv(63 downto 32) :=
                                        top_y((bc + 1) * 32 + 31 downto (bc + 1) * 32);
                                end if;
                            else
                                -- Spec 8.3.1.2.4: replicate, do not disable.
                                topv(63 downto 32) := topv(31 downto 24) &
                                    topv(31 downto 24) & topv(31 downto 24) &
                                    topv(31 downto 24);
                            end if;
                            if bc > 0 then
                                leftv := byte_of(r4(br * 4 + bc - 1), 15) &
                                         byte_of(r4(br * 4 + bc - 1), 11) &
                                         byte_of(r4(br * 4 + bc - 1), 7) &
                                         byte_of(r4(br * 4 + bc - 1), 3);
                            else
                                leftv := left_y(br * 32 + 31 downto br * 32);
                            end if;
                            if br > 0 and bc > 0 then
                                tlv := byte_of(r4((br - 1) * 4 + bc - 1), 15);
                            elsif br > 0 then
                                tlv := byte_of(left_y, br * 4 - 1);
                            elsif bc > 0 then
                                tlv := byte_of(top_y, bc * 4 - 1);
                            else
                                tlv := tl_y;
                            end if;

                            pos := br * 4 + bc;
                            p4_mode  <= unsigned(modes4(4 * pos + 3 downto 4 * pos));
                            p4_top   <= topv;
                            p4_left  <= leftv;
                            p4_tl    <= tlv;
                            p4_at    <= at;
                            p4_al    <= al;
                            p4_atl   <= atl;
                            p4_valid <= '1';
                        else
                            p16_blk   <= cur_pos;
                            p16_valid <= '1';
                        end if;
                    else
                        pc_blk <= cur_pos(1 downto 0);
                        if cur_comp = '0' then
                            pc_top <= top_u; pc_left <= left_u; pc_tl <= tl_u;
                        else
                            pc_top <= top_v; pc_left <= left_v; pc_tl <= tl_v;
                        end if;
                        pc_valid <= '1';
                    end if;
                    st <= S_PRED_WAIT;

                ----------------------------------------------------------
                when S_PRED_WAIT =>
                    if cur_kind = 1 and i4 = '1' then
                        if p4_vo = '1' then
                            pred_q <= p4_pred;  st <= S_Q_ISSUE;
                        end if;
                    elsif cur_kind = 1 then
                        if p16_vo = '1' then
                            pred_q <= p16_pred; st <= S_Q_ISSUE;
                        end if;
                    else
                        if pc_vo = '1' then
                            pred_q <= pc_pred;  st <= S_Q_ISSUE;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- Dequantise. A DC block uses its own inverse mode and the
                -- 2x2 one only looks at lanes 0..3.
                when S_Q_ISSUE =>
                    for k in 0 to 15 loop q_din(k) <= lev(k); end loop;
                    if cur_kind = 0 then
                        q_mode <= to_unsigned(3, 3); q_qp <= qp_y;
                    elsif cur_kind = 2 then
                        q_mode <= to_unsigned(5, 3); q_qp <= qp_c;
                    elsif cur_kind = 1 then
                        q_mode <= to_unsigned(1, 3); q_qp <= qp_y;
                    else
                        q_mode <= to_unsigned(1, 3); q_qp <= qp_c;
                    end if;
                    q_valid <= '1';
                    st <= S_Q_WAIT;

                when S_Q_WAIT =>
                    if q_vo = '1' then
                        for k in 0 to 15 loop coef_q(k) <= q_dout(k); end loop;
                        -- The DC of an I_16x16 luma block or a chroma block
                        -- comes from the Hadamard plane, not from this
                        -- block's own coefficient 0.
                        if cur_kind = 1 and i4 = '0' then
                            coef_q(0) <= dcy(to_integer(cur_pos));
                        elsif cur_kind = 3 then
                            if cur_comp = '0' then
                                coef_q(0) <= dcu(to_integer(cur_pos));
                            else
                                coef_q(0) <= dcv(to_integer(cur_pos));
                            end if;
                        end if;
                        st <= S_T_ISSUE;
                    end if;

                ----------------------------------------------------------
                when S_T_ISSUE =>
                    for k in 0 to 15 loop t_din(k) <= coef_q(k); end loop;
                    if cur_kind = 0 then
                        t_mode <= to_unsigned(3, 3);     -- ihadamard4x4
                    elsif cur_kind = 2 then
                        t_mode <= to_unsigned(5, 3);     -- ihadamard2x2
                    else
                        t_mode <= to_unsigned(1, 3);     -- idct4x4
                    end if;
                    t_valid <= '1';
                    st <= S_T_WAIT;

                when S_T_WAIT =>
                    if t_vo = '1' then
                        if cur_kind = 0 then
                            for k in 0 to 15 loop dcy(k) <= t_dout(k); end loop;
                            st <= S_TAKE;
                            nblk <= nblk + 1;
                        elsif cur_kind = 2 then
                            for k in 0 to 3 loop
                                if cur_comp = '0' then dcu(k) <= t_dout(k);
                                else                   dcv(k) <= t_dout(k);
                                end if;
                            end loop;
                            st <= S_TAKE;
                            nblk <= nblk + 1;
                        else
                            for k in 0 to 15 loop
                                res_q(20 * k + 19 downto 20 * k) <=
                                    std_logic_vector(t_dout(k)(19 downto 0));
                            end loop;
                            st <= S_R_ISSUE;
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_R_ISSUE =>
                    rc_valid <= '1';
                    st <= S_R_WAIT;

                when S_R_WAIT =>
                    if rc_vo = '1' then
                        if cur_kind = 1 then
                            r4(to_integer(cur_pos)) <= rc_out;
                        elsif cur_comp = '0' then
                            rcu(to_integer(cur_pos(1 downto 0))) <= rc_out;
                        else
                            rcv(to_integer(cur_pos(1 downto 0))) <= rc_out;
                        end if;
                        if nblk = 26 then
                            st <= S_EMIT;
                        else
                            nblk <= nblk + 1;
                            st   <= S_TAKE;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- Y raster 0..15, U 0..3, V 0..3: the order line_buffer's
                -- commit path already consumes.
                when S_EMIT =>
                    if b_valid = '0' then
                        if emit_i < 16 then
                            b_plane <= to_unsigned(0, 2);
                            b_idx   <= to_unsigned(emit_i, 4);
                            b_data  <= r4(emit_i);
                        elsif emit_i < 20 then
                            b_plane <= to_unsigned(1, 2);
                            b_idx   <= to_unsigned(emit_i - 16, 4);
                            b_data  <= rcu(emit_i - 16);
                        else
                            b_plane <= to_unsigned(2, 2);
                            b_idx   <= to_unsigned(emit_i - 20, 4);
                            b_data  <= rcv(emit_i - 20);
                        end if;
                        b_valid <= '1';
                    elsif rec_ready_i = '1' then
                        b_valid <= '0';
                        if emit_i = 23 then
                            st <= S_DONE;
                        else
                            emit_i <= emit_i + 1;
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
