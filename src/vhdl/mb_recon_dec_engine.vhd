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
-- WITH_SSD = false.
--
-- Structure: a dataflow, not a sequence. Every block is issued into the
-- dequantise -> inverse transform chain the cycle it is taken, and its
-- prediction is issued separately; the two meet at the reconstruction step,
-- residuals waiting in a small FIFO for their prediction. Each of the five
-- engines is one block per cycle with a fixed latency, so this runs at the
-- rate the entropy decoder can feed it. The first version of this block
-- walked one block at a time through all four stages with a handshake at
-- each, and cost more than the entropy decode it sat behind.
--
-- Three things constrain the flow, and each is a gate rather than a state:
--
--   A DC block's result is the DC of the AC blocks after it, so no block is
--   taken while a DC block is in the chain. That is at most three short
--   waits per macroblock.
--
--   An I_4x4 block predicts from the reconstruction of the block before it,
--   so for those the prediction is not issued until every earlier block has
--   been written. The residual chain runs ahead regardless; only the
--   predict -> reconstruct -> write loop is serial, four cycles a block.
--
--   Predictions come out of whichever engine they went into, and the three
--   engines have different latencies, so a prediction is not issued to a
--   different engine than the last one until the last one's outputs have
--   all been consumed. The block order makes that a single wait per
--   macroblock, between the luma and the chroma.
--
-- The I_4x4 neighbour gather is lifted from mode_decide_engine's own
-- reconstruction path rather than rewritten, including the above-right rule
-- and the sample replication of spec 8.3.1.2.4.
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
    -- Scan index of raster block k, the inverse of the two above.
    constant RASTER_TO_SCAN : int16_t := (0,1,4,5, 2,3,6,7, 8,9,12,13, 10,11,14,15);

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

    constant DEPTH : integer := 8;     -- blocks in flight, and the residual FIFO

    type c16_t  is array (0 to 15) of signed(31 downto 0);
    type blk_t  is array (0 to 15) of std_logic_vector(127 downto 0);
    type c4_t   is array (0 to 3)  of std_logic_vector(127 downto 0);
    subtype res_t is std_logic_vector(16 * 20 - 1 downto 0);
    type res_fifo_t is array (0 to DEPTH - 1) of res_t;

    -- One non-DC block, as taken. Indexed by the block's ordinal among the
    -- non-DC blocks, which is also its order through every pipeline here.
    type desc_t is record
        kind : unsigned(1 downto 0);
        comp : std_logic;
        pos  : unsigned(3 downto 0);
    end record;
    type desc_ring_t is array (0 to 15) of desc_t;

    -- What is inside the dequantiser and the transform, in order.
    type tag_t is record
        is_dc : std_logic;
        kind  : unsigned(1 downto 0);
        comp  : std_logic;
    end record;
    type tag_ring_t is array (0 to 7) of tag_t;

    type state_t is (S_IDLE, S_RUN, S_EMIT, S_DONE);
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

    -- Bookkeeping. Counts of non-DC blocks past each point, in order:
    -- taken, prediction issued, prediction consumed (reconstruction issued),
    -- reconstruction written. Rings are indexed by the low bits.
    signal desc    : desc_ring_t := (others => (kind => "00", comp => '0', pos => "0000"));
    signal n_all   : integer range 0 to 27 := 0;    -- taken, DC included
    signal n_take  : unsigned(4 downto 0) := (others => '0');
    signal n_pred  : unsigned(4 downto 0) := (others => '0');
    signal n_pout  : unsigned(4 downto 0) := (others => '0');
    signal n_rec   : unsigned(4 downto 0) := (others => '0');
    signal n_qout  : unsigned(4 downto 0) := (others => '0');  -- past the DC override
    signal dc_wait : std_logic := '0';
    signal pred_kind_v : std_logic := '0';          -- a prediction has been issued
    signal pred_kind   : unsigned(1 downto 0) := (others => '0');

    signal qtag : tag_ring_t := (others => (is_dc => '0', kind => "00", comp => '0'));
    signal ttag : tag_ring_t := (others => (is_dc => '0', kind => "00", comp => '0'));
    signal q_wr, q_rd, t_wr, t_rd : unsigned(2 downto 0) := (others => '0');

    signal rfifo : res_fifo_t;
    signal r_wr, r_rd : unsigned(3 downto 0) := (others => '0');
    signal r_count : unsigned(3 downto 0) := (others => '0');

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
    signal q_din, q_dout : c16_t := (others => (others => '0'));

    -- transform_engine
    signal t_mode  : unsigned(2 downto 0) := (others => '0');
    signal t_valid : std_logic := '0';
    signal t_ready : std_logic;
    signal t_vo    : std_logic;
    signal t_din, t_dout : c16_t := (others => (others => '0'));

    -- recon_engine, driven combinationally from the join
    signal rc_valid : std_logic;
    signal rc_ready : std_logic;
    signal rc_vo    : std_logic;
    signal rc_out   : std_logic_vector(127 downto 0);
    signal rc_pred  : std_logic_vector(127 downto 0);
    signal rc_res   : res_t;
    signal pred_now : std_logic;
    signal res_now  : std_logic;

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

    signal take_ok : std_logic;

begin

    ready_o     <= '1' when st = S_IDLE else '0';
    done_o      <= '1' when st = S_DONE else '0';
    rec_valid_o <= b_valid;
    rec_plane_o <= b_plane;
    rec_idx_o   <= b_idx;
    rec_data_o  <= b_data;

    -- A block can be taken when nothing DC is in the chain and there is room
    -- for its residual to wait.
    take_ok <= '1' when st = S_RUN and dc_wait = '0' and n_all < 27
                    and (n_take - n_rec) < DEPTH else '0';
    blk_ready_o <= take_ok;

    ------------------------------------------------------------------
    -- The join. A prediction is consumed the cycle it appears, provided its
    -- residual is already waiting; otherwise the predictor is held. Only one
    -- predictor has outputs in flight at a time, by construction.
    ------------------------------------------------------------------
    pred_now <= p4_vo or p16_vo or pc_vo;
    res_now  <= '1' when r_count /= 0 else '0';
    rc_valid <= pred_now and res_now;
    rc_pred <= p4_pred when p4_vo = '1' else p16_pred when p16_vo = '1' else pc_pred;
    rc_res  <= rfifo(to_integer(r_rd(2 downto 0)));

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
                  pred_i => rc_pred, res_i => rc_res,
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
                  pred_o => p4_pred, valid_o => p4_vo, ready_i => res_now);

    p16 : entity work.predict_16x16_engine
        port map (clk => clk, rst_n => rst_n,
                  mode_i => m16, blk_i => p16_blk,
                  top_i => top_y, left_i => left_y, tl_i => tl_y,
                  avail_top_i => a_top, avail_left_i => a_left,
                  avail_tl_i => a_tl,
                  valid_i => p16_valid, ready_o => p16_ready,
                  pred_o => p16_pred, valid_o => p16_vo, ready_i => res_now);

    pchroma : entity work.predict_chroma_engine
        port map (clk => clk, rst_n => rst_n,
                  mode_i => mchroma, blk_i => pc_blk,
                  top_i => pc_top, left_i => pc_left, tl_i => pc_tl,
                  avail_top_i => a_top, avail_left_i => a_left,
                  avail_tl_i => a_tl,
                  valid_i => pc_valid, ready_o => pc_ready,
                  pred_o => pc_pred, valid_o => pc_vo, ready_i => res_now);

    ------------------------------------------------------------------
    main_p : process(clk)
        variable s, br, bc, pos : integer range 0 to 15;
        variable e     : integer range 0 to 24;
        variable topv  : std_logic_vector(63 downto 0);
        variable leftv : std_logic_vector(31 downto 0);
        variable tlv   : std_logic_vector(7 downto 0);
        variable at, al, atl : std_logic;
        variable d     : desc_t;
        variable tg    : tag_t;
        variable can_issue : boolean;
        variable eng_free  : boolean;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                st       <= S_IDLE;
                q_valid  <= '0';
                t_valid  <= '0';
                p4_valid <= '0';
                p16_valid<= '0';
                pc_valid <= '0';
                b_valid  <= '0';
                n_all    <= 0;
                n_take   <= (others => '0');
                n_pred   <= (others => '0');
                n_pout   <= (others => '0');
                n_rec    <= (others => '0');
                n_qout   <= (others => '0');
                q_wr <= (others => '0'); q_rd <= (others => '0');
                t_wr <= (others => '0'); t_rd <= (others => '0');
                r_wr <= (others => '0'); r_rd <= (others => '0');
                r_count  <= (others => '0');
                dc_wait  <= '0';
                pred_kind_v <= '0';
                emit_i   <= 0;
            else
                -- Registered issues last one cycle unless held below.
                q_valid <= '0';
                t_valid <= '0';
                if p4_ready  = '1' then p4_valid  <= '0'; end if;
                if p16_ready = '1' then p16_valid <= '0'; end if;
                if pc_ready  = '1' then pc_valid  <= '0'; end if;

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
                        n_all   <= 0;
                        n_take  <= (others => '0');
                        n_pred  <= (others => '0');
                        n_pout  <= (others => '0');
                        n_rec   <= (others => '0');
                        n_qout  <= (others => '0');
                        q_wr <= (others => '0'); q_rd <= (others => '0');
                        t_wr <= (others => '0'); t_rd <= (others => '0');
                        r_wr <= (others => '0'); r_rd <= (others => '0');
                        r_count <= (others => '0');
                        dc_wait <= '0';
                        pred_kind_v <= '0';
                        emit_i  <= 0;
                        st      <= S_RUN;
                    end if;

                ----------------------------------------------------------
                when S_RUN =>
                    ------------------------------------------------------
                    -- 1. Take a block: straight into the dequantiser.
                    ------------------------------------------------------
                    if blk_valid_i = '1' and take_ok = '1' then
                        -- Zigzag to raster, except for a chroma DC block
                        -- whose four coefficients are already in order and
                        -- must land in lanes 0..3.
                        for k in 0 to 15 loop
                            if blk_kind_i = 2 then
                                q_din(k) <= resize(signed(
                                    blk_coefs_i(k * 16 + 15 downto k * 16)), 32);
                            else
                                q_din(ZIGZAG(k)) <= resize(signed(
                                    blk_coefs_i(k * 16 + 15 downto k * 16)), 32);
                            end if;
                        end loop;
                        if blk_kind_i = 0 then
                            q_mode <= to_unsigned(3, 3); q_qp <= qp_y;
                        elsif blk_kind_i = 2 then
                            q_mode <= to_unsigned(5, 3); q_qp <= qp_c;
                        elsif blk_kind_i = 1 then
                            q_mode <= to_unsigned(1, 3); q_qp <= qp_y;
                        else
                            q_mode <= to_unsigned(1, 3); q_qp <= qp_c;
                        end if;
                        q_valid <= '1';

                        tg.is_dc := '0';
                        if blk_kind_i = 0 or blk_kind_i = 2 then tg.is_dc := '1'; end if;
                        tg.kind := blk_kind_i;
                        tg.comp := blk_comp_i;
                        qtag(to_integer(q_wr)) <= tg;
                        q_wr <= q_wr + 1;

                        if tg.is_dc = '1' then
                            dc_wait <= '1';
                        else
                            desc(to_integer(n_take(3 downto 0))) <=
                                (kind => blk_kind_i, comp => blk_comp_i, pos => blk_pos_i);
                            n_take <= n_take + 1;
                        end if;
                        n_all <= n_all + 1;
                    end if;

                    ------------------------------------------------------
                    -- 2. Dequantiser out, transform in, with the DC of an
                    --    I_16x16 luma or a chroma block substituted from the
                    --    Hadamard plane.
                    ------------------------------------------------------
                    if q_vo = '1' then
                        tg := qtag(to_integer(q_rd));
                        q_rd <= q_rd + 1;
                        for k in 0 to 15 loop t_din(k) <= q_dout(k); end loop;
                        if tg.is_dc = '1' then
                            if tg.kind = 0 then t_mode <= to_unsigned(3, 3);
                            else                t_mode <= to_unsigned(5, 3);
                            end if;
                        else
                            t_mode <= to_unsigned(1, 3);
                            d := desc(to_integer(n_qout(3 downto 0)));
                            if d.kind = 1 and i4 = '0' then
                                t_din(0) <= dcy(to_integer(d.pos));
                            elsif d.kind = 3 then
                                if d.comp = '0' then t_din(0) <= dcu(to_integer(d.pos));
                                else                 t_din(0) <= dcv(to_integer(d.pos));
                                end if;
                            end if;
                            n_qout <= n_qout + 1;
                        end if;
                        t_valid <= '1';
                        ttag(to_integer(t_wr)) <= tg;
                        t_wr <= t_wr + 1;
                    end if;

                    ------------------------------------------------------
                    -- 3. Transform out: a DC plane is captured, a residual
                    --    waits for its prediction.
                    ------------------------------------------------------
                    if t_vo = '1' then
                        tg := ttag(to_integer(t_rd));
                        t_rd <= t_rd + 1;
                        if tg.is_dc = '1' then
                            if tg.kind = 0 then
                                for k in 0 to 15 loop dcy(k) <= t_dout(k); end loop;
                            else
                                for k in 0 to 3 loop
                                    if tg.comp = '0' then dcu(k) <= t_dout(k);
                                    else                  dcv(k) <= t_dout(k);
                                    end if;
                                end loop;
                            end if;
                            dc_wait <= '0';
                        else
                            for k in 0 to 15 loop
                                rfifo(to_integer(r_wr(2 downto 0)))
                                    (20 * k + 19 downto 20 * k) <=
                                    std_logic_vector(t_dout(k)(19 downto 0));
                            end loop;
                            r_wr <= r_wr + 1;
                        end if;
                    end if;

                    ------------------------------------------------------
                    -- 4. Issue a prediction for the next taken block, when
                    --    its engine is free and the gates allow.
                    ------------------------------------------------------
                    d := desc(to_integer(n_pred(3 downto 0)));
                    can_issue := n_pred /= n_take;
                    -- Not to a different engine while another has work out.
                    if pred_kind_v = '1' and d.kind /= pred_kind and n_pred /= n_pout then
                        can_issue := false;
                    end if;
                    -- I_4x4 luma predicts from the block before it.
                    if d.kind = 1 and i4 = '1' and n_pred /= n_rec then
                        can_issue := false;
                    end if;
                    if d.kind = 1 and i4 = '1' then
                        eng_free := (p4_valid = '0' or p4_ready = '1');
                    elsif d.kind = 1 then
                        eng_free := (p16_valid = '0' or p16_ready = '1');
                    else
                        eng_free := (pc_valid = '0' or pc_ready = '1');
                    end if;

                    if can_issue and eng_free then
                        pos := to_integer(d.pos);
                        if d.kind = 1 and i4 = '1' then
                            s  := RASTER_TO_SCAN(pos);
                            br := SCAN_BR(s);
                            bc := SCAN_BC(s);
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
                            p4_mode  <= unsigned(modes4(4 * pos + 3 downto 4 * pos));
                            p4_top   <= topv;
                            p4_left  <= leftv;
                            p4_tl    <= tlv;
                            p4_at    <= at;
                            p4_al    <= al;
                            p4_atl   <= atl;
                            p4_valid <= '1';
                        elsif d.kind = 1 then
                            p16_blk   <= d.pos;
                            p16_valid <= '1';
                        else
                            pc_blk <= d.pos(1 downto 0);
                            if d.comp = '0' then
                                pc_top <= top_u; pc_left <= left_u; pc_tl <= tl_u;
                            else
                                pc_top <= top_v; pc_left <= left_v; pc_tl <= tl_v;
                            end if;
                            pc_valid <= '1';
                        end if;
                        pred_kind   <= d.kind;
                        pred_kind_v <= '1';
                        n_pred      <= n_pred + 1;
                    end if;

                    ------------------------------------------------------
                    -- 5. The join fired (combinational above): retire the
                    --    residual and count the prediction as consumed.
                    ------------------------------------------------------
                    if rc_valid = '1' then
                        r_rd   <= r_rd + 1;
                        n_pout <= n_pout + 1;
                    end if;
                    -- One in from the transform, one out to the join, or
                    -- both, or neither.
                    if (t_vo = '1' and ttag(to_integer(t_rd)).is_dc = '0')
                       and rc_valid = '0' then
                        r_count <= r_count + 1;
                    elsif not (t_vo = '1' and ttag(to_integer(t_rd)).is_dc = '0')
                          and rc_valid = '1' then
                        r_count <= r_count - 1;
                    end if;

                    ------------------------------------------------------
                    -- 6. Reconstruction out: write it where it belongs.
                    ------------------------------------------------------
                    if rc_vo = '1' then
                        d := desc(to_integer(n_rec(3 downto 0)));
                        if d.kind = 1 then
                            r4(to_integer(d.pos)) <= rc_out;
                        elsif d.comp = '0' then
                            rcu(to_integer(d.pos(1 downto 0))) <= rc_out;
                        else
                            rcv(to_integer(d.pos(1 downto 0))) <= rc_out;
                        end if;
                        n_rec <= n_rec + 1;
                    end if;

                    if n_all = 27 and n_rec = 24 then
                        st <= S_EMIT;
                    end if;

                ----------------------------------------------------------
                -- Y raster 0..15, U 0..3, V 0..3, one per cycle: the order
                -- line_buffer's commit path already consumes.
                ----------------------------------------------------------
                when S_EMIT =>
                    -- Valid stays up between blocks; the data advances on
                    -- ready. Dropping valid between blocks made this two
                    -- cycles a block, 24 cycles a macroblock for nothing.
                    if b_valid = '0' or rec_ready_i = '1' then
                        if b_valid = '1' and emit_i = 23 then
                            b_valid <= '0';
                            st <= S_DONE;
                        else
                            if b_valid = '0' then e := 0; else e := emit_i + 1; end if;
                            emit_i <= e;
                            if e < 16 then
                                b_plane <= to_unsigned(0, 2);
                                b_idx   <= to_unsigned(e, 4);
                                b_data  <= r4(e);
                            elsif e < 20 then
                                b_plane <= to_unsigned(1, 2);
                                b_idx   <= to_unsigned(e - 16, 4);
                                b_data  <= rcu(e - 16);
                            else
                                b_plane <= to_unsigned(2, 2);
                                b_idx   <= to_unsigned(e - 20, 4);
                                b_data  <= rcv(e - 20);
                            end if;
                            b_valid <= '1';
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
