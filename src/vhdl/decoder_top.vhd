--------------------------------------------------------------------------------
-- decoder_top.vhd
--
-- Frame-level macroblock loop for an intra slice: the mirror of
-- mb_pipeline_controller. Walks the macroblocks in raster order and drives
--
--   line_buffer              neighbour samples, nC counts and intra modes
--   mb_header_dec_engine     mb_type, intra modes, CBP, mb_qp_delta
--   mb_residual_dec_engine   the residual blocks, each with its nC
--   mb_recon_dec_engine      prediction, dequantisation, inverse transform,
--                            reconstruction
--
-- line_buffer is the encoder's, unchanged. Nothing about holding the row
-- above and the column to the left is direction-specific, and it already
-- carries exactly the three things a decoder needs from its neighbours:
-- reconstructed samples for prediction, total_coeff for nC, and 4x4 modes
-- for the most-probable-mode rule. It even packs them the way the decode
-- engines want them.
--
-- The slice header, picture and sequence parameter sets, and the Annex B
-- framing stay outside, on the host, exactly as they do on the encode side.
-- They are read once per frame, so putting them in the fabric would buy no
-- latency worth the area. What arrives here is the macroblock layer of one
-- slice, with emulation prevention already removed, plus the slice QP.
--
-- Output is the reconstruction as 24 blocks per macroblock, Y raster 0..15
-- then U 0..3 then V 0..3, in macroblock raster order. Turning that into
-- NV12 rows in memory is frame_io's job on the encode side and a separate
-- block here too.
--
-- One macroblock at a time: header, then residual, then reconstruction. The
-- three cannot overlap within a macroblock because each needs the one before
-- it, and across macroblocks the entropy decode of the next cannot start
-- until this one's bits are retired. The reconstruction of macroblock n
-- could overlap the parse of n+1, which is the first thing to try if the
-- rate is short; it would need a second neighbour bundle and a deeper commit.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decoder_top is
    generic (
        MAX_MB_COLS : positive := 120        -- 1920/16; 240 for 4K
    );
    port (
        clk           : in  std_logic;
        rst_n         : in  std_logic;

        -- Frame control
        frame_start_i : in  std_logic;
        mbs_w_i       : in  unsigned(7 downto 0);
        mbs_h_i       : in  unsigned(7 downto 0);
        qp_i          : in  unsigned(5 downto 0);        -- slice QP
        chroma_qp_offset_i : in signed(5 downto 0);
        busy_o        : out std_logic;
        frame_done_o  : out std_logic;
        err_o         : out std_logic;
        err_code_o    : out unsigned(3 downto 0);
        err_mb_o      : out unsigned(15 downto 0);

        -- Slice payload bytes, emulation prevention already removed
        in_valid      : in  std_logic;
        in_ready      : out std_logic;
        in_data       : in  unsigned(7 downto 0);
        in_last       : in  std_logic;

        -- Reconstruction: 24 blocks per macroblock, raster macroblock order
        rec_valid_o   : out std_logic;
        rec_ready_i   : in  std_logic;
        rec_mb_o      : out unsigned(15 downto 0);
        rec_plane_o   : out unsigned(1 downto 0);
        rec_idx_o     : out unsigned(3 downto 0);
        rec_data_o    : out std_logic_vector(127 downto 0)
    );
end entity;

architecture rtl of decoder_top is

    type qpc_tab_t is array (0 to 51) of integer range 0 to 39;
    constant QPC_TAB : qpc_tab_t := (
         0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,
        29,30,31,32,32,33,34,34,35,35,36,36,37,37,37,38,38,38,39,39,39,39);

    type state_t is (S_IDLE, S_FETCH, S_HDR, S_RES, S_RECON_WAIT, S_COMMIT, S_DONE);
    signal st : state_t := S_IDLE;

    signal mbs_w, mbs_h : unsigned(7 downto 0) := (others => '0');
    signal mb_c  : unsigned(7 downto 0) := (others => '0');
    signal mb_r  : unsigned(7 downto 0) := (others => '0');
    signal mb_n  : unsigned(15 downto 0) := (others => '0');
    signal qp_q  : unsigned(5 downto 0) := (others => '0');
    signal cqo   : signed(5 downto 0) := (others => '0');
    signal qp_c  : unsigned(5 downto 0) := (others => '0');

    signal err_q  : std_logic := '0';
    signal errc_q : unsigned(3 downto 0) := (others => '0');
    signal errmb_q: unsigned(15 downto 0) := (others => '0');

    -- bit reader
    signal peek     : unsigned(31 downto 0);
    signal bavail   : std_logic;
    signal consume  : std_logic;
    signal consume_n: unsigned(5 downto 0);
    signal bitpos   : unsigned(31 downto 0);
    signal underrun : std_logic;

    -- header engine
    signal h_start : std_logic := '0';
    signal h_ready, h_done, h_err : std_logic;
    signal h_errc  : unsigned(3 downto 0);
    signal h_i4    : std_logic;
    signal h_m16   : unsigned(1 downto 0);
    signal h_modes4: std_logic_vector(63 downto 0);
    signal h_mc    : unsigned(1 downto 0);
    signal h_cbpl  : unsigned(3 downto 0);
    signal h_cbpc  : unsigned(1 downto 0);
    signal h_hasres: std_logic;
    signal h_qp    : unsigned(5 downto 0);
    signal h_cons  : std_logic;
    signal h_consn : unsigned(5 downto 0);

    -- latched header, because the engines below are restarted and the
    -- header engine's outputs are only guaranteed until its next start
    signal j_i4    : std_logic := '0';
    signal j_m16   : unsigned(1 downto 0) := (others => '0');
    signal j_modes4: std_logic_vector(63 downto 0) := (others => '0');
    signal j_mc    : unsigned(1 downto 0) := (others => '0');
    signal j_cbpl  : unsigned(3 downto 0) := (others => '0');
    signal j_cbpc  : unsigned(1 downto 0) := (others => '0');

    -- residual sequencer
    signal r_start : std_logic := '0';
    signal r_ready, r_done, r_err : std_logic;
    signal r_errc  : unsigned(3 downto 0);
    signal r_cons  : std_logic;
    signal r_consn : unsigned(5 downto 0);
    signal r_bvalid, r_bready : std_logic;
    signal r_bkind : unsigned(1 downto 0);
    signal r_bcomp : std_logic;
    signal r_bpos  : unsigned(3 downto 0);
    signal r_btotal: unsigned(4 downto 0);
    signal r_bcoefs: std_logic_vector(255 downto 0);
    signal r_ncy   : std_logic_vector(79 downto 0);
    signal r_ncu, r_ncv : std_logic_vector(19 downto 0);

    -- reconstruction engine
    signal k_start : std_logic := '0';
    signal k_ready, k_done : std_logic;
    signal k_rvalid, k_rready : std_logic;
    signal k_rplane : unsigned(1 downto 0);
    signal k_ridx   : unsigned(3 downto 0);
    signal k_rdata  : std_logic_vector(127 downto 0);

    -- line buffer
    signal lb_frame_start, lb_row_start : std_logic := '0';
    signal lb_fetch_valid : std_logic := '0';
    signal lb_fetch_ready, lb_nb_valid : std_logic;
    signal lb_top_y, lb_left_y : std_logic_vector(127 downto 0);
    signal lb_tr_y : std_logic_vector(31 downto 0);
    signal lb_tl_y : std_logic_vector(7 downto 0);
    signal lb_top_u, lb_top_v, lb_left_u, lb_left_v : std_logic_vector(63 downto 0);
    signal lb_tl_u, lb_tl_v : std_logic_vector(7 downto 0);
    signal lb_nc_y_top, lb_nc_y_left : std_logic_vector(19 downto 0);
    signal lb_nc_u_top, lb_nc_u_left : std_logic_vector(9 downto 0);
    signal lb_nc_v_top, lb_nc_v_left : std_logic_vector(9 downto 0);
    signal lb_mode4_top, lb_mode4_left : std_logic_vector(15 downto 0);
    signal lb_a_top, lb_a_left, lb_a_tl, lb_a_tr : std_logic;
    signal lb_commit_valid : std_logic := '0';
    signal lb_commit_ready : std_logic;
    signal lb_rec_y_bot, lb_rec_uv_bot : std_logic_vector(127 downto 0) := (others => '0');
    signal lb_rec_y_right, lb_rec_uv_right : std_logic_vector(127 downto 0) := (others => '0');
    signal lb_mode4 : std_logic_vector(63 downto 0) := (others => '0');

    -- The neighbour bundle, held for the whole macroblock: line_buffer
    -- presents it around the fetch and the engines need it much later.
    signal nb_top_y, nb_left_y : std_logic_vector(127 downto 0) := (others => '0');
    signal nb_tr_y : std_logic_vector(31 downto 0) := (others => '0');
    signal nb_tl_y : std_logic_vector(7 downto 0) := (others => '0');
    signal nb_top_u, nb_top_v, nb_left_u, nb_left_v :
        std_logic_vector(63 downto 0) := (others => '0');
    signal nb_tl_u, nb_tl_v : std_logic_vector(7 downto 0) := (others => '0');
    signal nb_ncy_top, nb_ncy_left : std_logic_vector(19 downto 0) := (others => '0');
    signal nb_ncu_top, nb_ncu_left : std_logic_vector(9 downto 0) := (others => '0');
    signal nb_ncv_top, nb_ncv_left : std_logic_vector(9 downto 0) := (others => '0');
    signal nb_m4top, nb_m4left : std_logic_vector(15 downto 0) := (others => '0');
    signal nb_at, nb_al, nb_atl, nb_atr : std_logic := '0';

    -- The macroblock's own reconstruction, gathered on the way past so the
    -- line-buffer commit has the bottom row and the right column.
    signal cm_y_bot, cm_uv_bot : std_logic_vector(127 downto 0) := (others => '0');
    signal cm_y_right, cm_uv_right : std_logic_vector(127 downto 0) := (others => '0');

    function byte_of(v : std_logic_vector; k : integer) return std_logic_vector is
    begin
        return v(8 * k + 7 downto 8 * k);
    end function;

begin

    busy_o       <= '0' when st = S_IDLE else '1';
    frame_done_o <= '1' when st = S_DONE else '0';
    err_o        <= err_q;
    err_code_o   <= errc_q;
    err_mb_o     <= errmb_q;

    rec_valid_o <= k_rvalid;
    rec_mb_o    <= mb_n;
    rec_plane_o <= k_rplane;
    rec_idx_o   <= k_ridx;
    rec_data_o  <= k_rdata;
    -- The commit needs every block, so the reconstruction stream is only
    -- taken when the consumer takes it too.
    k_rready    <= rec_ready_i;

    -- Only one of the two parsers owns the reader at a time, and neither
    -- asserts consume outside its own phase.
    consume   <= h_cons  when st = S_HDR else r_cons;
    consume_n <= h_consn when st = S_HDR else r_consn;

    ------------------------------------------------------------------
    rd : entity work.bit_reader
        generic map (PEEK_W => 32)
        port map (clk => clk, rst_n => rst_n,
                  in_data => in_data, in_valid => in_valid,
                  in_ready => in_ready, in_last => in_last,
                  peek_o => peek, avail_o => bavail,
                  consume_i => consume, consume_n_i => consume_n,
                  bitpos_o => bitpos, underrun_o => underrun);

    hdr : entity work.mb_header_dec_engine
        port map (clk => clk, rst_n => rst_n,
                  start_i => h_start, ready_o => h_ready,
                  qp_i => qp_q,
                  mode4_top_i => nb_m4top, mode4_left_i => nb_m4left,
                  avail_top_i => nb_at, avail_left_i => nb_al,
                  peek_i => peek, avail_i => bavail,
                  consume_o => h_cons, consume_n_o => h_consn,
                  done_o => h_done, err_o => h_err, err_code_o => h_errc,
                  is_i4x4_o => h_i4, mode16_o => h_m16, modes4_o => h_modes4,
                  mode_chroma_o => h_mc, cbp_luma_o => h_cbpl,
                  cbp_chroma_o => h_cbpc, has_residual_o => h_hasres,
                  qp_o => h_qp, hdr_bits_o => open);

    res : entity work.mb_residual_dec_engine
        port map (clk => clk, rst_n => rst_n,
                  start_i => r_start, ready_o => r_ready,
                  is_i4x4_i => j_i4, cbp_luma_i => j_cbpl,
                  cbp_chroma_i => j_cbpc,
                  avail_top_i => nb_at, avail_left_i => nb_al,
                  nc_top_i => unsigned(nb_ncy_top),
                  nc_left_i => unsigned(nb_ncy_left),
                  ncu_top_i => unsigned(nb_ncu_top),
                  ncu_left_i => unsigned(nb_ncu_left),
                  ncv_top_i => unsigned(nb_ncv_top),
                  ncv_left_i => unsigned(nb_ncv_left),
                  peek_i => peek, avail_i => bavail,
                  consume_o => r_cons, consume_n_o => r_consn,
                  blk_valid_o => r_bvalid, blk_ready_i => r_bready,
                  blk_kind_o => r_bkind, blk_comp_o => r_bcomp,
                  blk_pos_o => r_bpos, blk_total_o => r_btotal,
                  blk_coefs_o => r_bcoefs,
                  done_o => r_done, err_o => r_err, err_code_o => r_errc,
                  nc_y_o => r_ncy, nc_u_o => r_ncu, nc_v_o => r_ncv);

    rec : entity work.mb_recon_dec_engine
        port map (clk => clk, rst_n => rst_n,
                  start_i => k_start, ready_o => k_ready, done_o => k_done,
                  is_i4x4_i => j_i4, mode16_i => j_m16, modes4_i => j_modes4,
                  mode_chroma_i => j_mc,
                  qp_y_i => qp_q, qp_c_i => qp_c,
                  top_y_i => nb_top_y, tr_y_i => nb_tr_y, tl_y_i => nb_tl_y,
                  left_y_i => nb_left_y,
                  top_u_i => nb_top_u, left_u_i => nb_left_u, tl_u_i => nb_tl_u,
                  top_v_i => nb_top_v, left_v_i => nb_left_v, tl_v_i => nb_tl_v,
                  avail_top_i => nb_at, avail_left_i => nb_al,
                  avail_tl_i => nb_atl, avail_tr_i => nb_atr,
                  blk_valid_i => r_bvalid, blk_ready_o => r_bready,
                  blk_kind_i => r_bkind, blk_comp_i => r_bcomp,
                  blk_pos_i => r_bpos, blk_coefs_i => r_bcoefs,
                  rec_valid_o => k_rvalid, rec_ready_i => k_rready,
                  rec_plane_o => k_rplane, rec_idx_o => k_ridx,
                  rec_data_o => k_rdata);

    lb : entity work.line_buffer
        generic map (MAX_MB_COLS => MAX_MB_COLS)
        port map (clk => clk, rst_n => rst_n,
                  frame_start_i => lb_frame_start, row_start_i => lb_row_start,
                  mbs_w_i => mbs_w,
                  fetch_valid_i => lb_fetch_valid, fetch_mb_c_i => mb_c,
                  fetch_ready_o => lb_fetch_ready, nb_valid_o => lb_nb_valid,
                  top_y_o => lb_top_y, tr_y_o => lb_tr_y, tl_y_o => lb_tl_y,
                  left_y_o => lb_left_y,
                  top_u_o => lb_top_u, top_v_o => lb_top_v,
                  tl_u_o => lb_tl_u, tl_v_o => lb_tl_v,
                  left_u_o => lb_left_u, left_v_o => lb_left_v,
                  nc_y_top_o => lb_nc_y_top, nc_y_left_o => lb_nc_y_left,
                  nc_u_top_o => lb_nc_u_top, nc_u_left_o => lb_nc_u_left,
                  nc_v_top_o => lb_nc_v_top, nc_v_left_o => lb_nc_v_left,
                  mode4_top_o => lb_mode4_top, mode4_left_o => lb_mode4_left,
                  avail_top_o => lb_a_top, avail_left_o => lb_a_left,
                  avail_tl_o => lb_a_tl, avail_tr_o => lb_a_tr,
                  commit_valid_i => lb_commit_valid,
                  commit_ready_o => lb_commit_ready,
                  commit_mb_c_i => mb_c,
                  rec_y_bot_i => cm_y_bot, rec_uv_bot_i => cm_uv_bot,
                  rec_y_right_i => cm_y_right, rec_uv_right_i => cm_uv_right,
                  nc_y_i => r_ncy, nc_u_i => r_ncu, nc_v_i => r_ncv,
                  mode4_i => lb_mode4);

    ------------------------------------------------------------------
    -- Gather the macroblock's bottom row and right column out of the
    -- reconstruction stream as it goes past, so the commit has them without
    -- a second copy of the macroblock.
    ------------------------------------------------------------------
    gather_p : process(clk)
        variable idx : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            if k_rvalid = '1' and k_rready = '1' then
                idx := to_integer(k_ridx);
                if k_rplane = 0 then
                    if idx >= 12 then
                        cm_y_bot((idx - 12) * 32 + 31 downto (idx - 12) * 32) <=
                            k_rdata(127 downto 96);
                    end if;
                    if (idx mod 4) = 3 then
                        cm_y_right((idx / 4) * 32 + 31 downto (idx / 4) * 32) <=
                            byte_of(k_rdata, 15) & byte_of(k_rdata, 11) &
                            byte_of(k_rdata, 7)  & byte_of(k_rdata, 3);
                    end if;
                else
                    -- The chroma buses are NV12-interleaved, not split into a
                    -- U half and a V half: each of the eight positions takes
                    -- 16 bits, U in the low byte and V in the high one. Same
                    -- packing the encoder commits, and the same packing
                    -- line_buffer hands back as left_u / left_v.
                    if idx >= 2 then
                        for j in 0 to 3 loop
                            if k_rplane = 1 then
                                cm_uv_bot(16 * ((idx - 2) * 4 + j) + 7
                                          downto 16 * ((idx - 2) * 4 + j)) <=
                                    byte_of(k_rdata, 12 + j);
                            else
                                cm_uv_bot(16 * ((idx - 2) * 4 + j) + 15
                                          downto 16 * ((idx - 2) * 4 + j) + 8) <=
                                    byte_of(k_rdata, 12 + j);
                            end if;
                        end loop;
                    end if;
                    if (idx mod 2) = 1 then
                        for j in 0 to 3 loop
                            if k_rplane = 1 then
                                cm_uv_right(16 * ((idx / 2) * 4 + j) + 7
                                            downto 16 * ((idx / 2) * 4 + j)) <=
                                    byte_of(k_rdata, 4 * j + 3);
                            else
                                cm_uv_right(16 * ((idx / 2) * 4 + j) + 15
                                            downto 16 * ((idx / 2) * 4 + j) + 8) <=
                                    byte_of(k_rdata, 4 * j + 3);
                            end if;
                        end loop;
                    end if;
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    main_p : process(clk)
        variable q : integer;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                st              <= S_IDLE;
                h_start         <= '0';
                r_start         <= '0';
                k_start         <= '0';
                lb_frame_start  <= '0';
                lb_row_start    <= '0';
                lb_fetch_valid  <= '0';
                lb_commit_valid <= '0';
                err_q           <= '0';
            else
                h_start        <= '0';
                r_start        <= '0';
                k_start        <= '0';
                lb_frame_start <= '0';
                lb_row_start   <= '0';

                case st is

                ----------------------------------------------------------
                when S_IDLE =>
                    if frame_start_i = '1' then
                        mbs_w  <= mbs_w_i;
                        mbs_h  <= mbs_h_i;
                        qp_q   <= qp_i;
                        cqo    <= chroma_qp_offset_i;
                        mb_c   <= (others => '0');
                        mb_r   <= (others => '0');
                        mb_n   <= (others => '0');
                        err_q  <= '0';
                        errc_q <= (others => '0');
                        lb_frame_start <= '1';
                        lb_row_start   <= '1';
                        lb_fetch_valid <= '1';
                        st <= S_FETCH;
                    end if;

                ----------------------------------------------------------
                -- Neighbour bundle for this macroblock. Held for the whole
                -- macroblock rather than read where it is needed: the line
                -- buffer presents it around the fetch, and the reconstruction
                -- runs long after.
                when S_FETCH =>
                    if lb_fetch_ready = '1' then
                        lb_fetch_valid <= '0';
                    end if;
                    if lb_nb_valid = '1' then
                        lb_fetch_valid <= '0';
                        nb_top_y  <= lb_top_y;   nb_left_y <= lb_left_y;
                        nb_tr_y   <= lb_tr_y;    nb_tl_y   <= lb_tl_y;
                        nb_top_u  <= lb_top_u;   nb_left_u <= lb_left_u;
                        nb_tl_u   <= lb_tl_u;
                        nb_top_v  <= lb_top_v;   nb_left_v <= lb_left_v;
                        nb_tl_v   <= lb_tl_v;
                        nb_ncy_top <= lb_nc_y_top; nb_ncy_left <= lb_nc_y_left;
                        nb_ncu_top <= lb_nc_u_top; nb_ncu_left <= lb_nc_u_left;
                        nb_ncv_top <= lb_nc_v_top; nb_ncv_left <= lb_nc_v_left;
                        nb_m4top   <= lb_mode4_top; nb_m4left <= lb_mode4_left;
                        nb_at   <= lb_a_top;  nb_al  <= lb_a_left;
                        nb_atl  <= lb_a_tl;   nb_atr <= lb_a_tr;
                        h_start <= '1';
                        st <= S_HDR;
                    end if;

                ----------------------------------------------------------
                when S_HDR =>
                    if h_done = '1' then
                        if h_err = '1' then
                            err_q   <= '1';
                            errc_q  <= h_errc;
                            errmb_q <= mb_n;
                            st      <= S_DONE;
                        else
                            j_i4    <= h_i4;
                            j_m16   <= h_m16;
                            j_modes4<= h_modes4;
                            j_mc    <= h_mc;
                            j_cbpl  <= h_cbpl;
                            j_cbpc  <= h_cbpc;
                            qp_q    <= h_qp;
                            q := to_integer(h_qp) + to_integer(cqo);
                            if q < 0  then q := 0;  end if;
                            if q > 51 then q := 51; end if;
                            qp_c    <= to_unsigned(QPC_TAB(q), 6);
                            -- An I_16x16 macroblock predicts as DC for its
                            -- neighbours, whatever its own luma modes say.
                            if h_i4 = '1' then
                                lb_mode4 <= h_modes4;
                            else
                                lb_mode4 <= x"2222222222222222";
                            end if;
                            r_start <= '1';
                            k_start <= '1';
                            st      <= S_RES;
                        end if;
                    end if;

                ----------------------------------------------------------
                -- The sequencer and the reconstruction engine run together:
                -- the block stream flows straight from one into the other.
                when S_RES =>
                    if r_done = '1' then
                        if r_err = '1' then
                            err_q   <= '1';
                            errc_q  <= r_errc;
                            errmb_q <= mb_n;
                            st      <= S_DONE;
                        else
                            st <= S_RECON_WAIT;
                        end if;
                    end if;

                ----------------------------------------------------------
                when S_RECON_WAIT =>
                    if k_done = '1' then
                        lb_commit_valid <= '1';
                        st <= S_COMMIT;
                    end if;

                ----------------------------------------------------------
                when S_COMMIT =>
                    if lb_commit_ready = '1' then
                        lb_commit_valid <= '0';
                        if mb_c + 1 = mbs_w then
                            if mb_r + 1 = mbs_h then
                                st <= S_DONE;
                            else
                                mb_c <= (others => '0');
                                mb_r <= mb_r + 1;
                                mb_n <= mb_n + 1;
                                lb_row_start   <= '1';
                                lb_fetch_valid <= '1';
                                st <= S_FETCH;
                            end if;
                        else
                            mb_c <= mb_c + 1;
                            mb_n <= mb_n + 1;
                            lb_fetch_valid <= '1';
                            st <= S_FETCH;
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
