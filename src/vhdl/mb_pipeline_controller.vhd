--------------------------------------------------------------------------------
-- mb_pipeline_controller.vhd
--
-- Frame-level macroblock pipeline: walks the MBs of an I slice in raster
-- order and drives the dataflow blocks the way encode_frame_h264 /
-- encode_mb_emit in src/encoder.c do:
--
--   line_buffer         neighbour samples, nC counts and intra modes
--   mode_decide_engine  luma / chroma decision, levels, reconstruction
--   mb_header_engine    CBP + Exp-Golomb MB header fields
--   cavlc_dispatch      packets to N CAVLC engines, in-order byte merge
--
-- Two sequencers:
--
--   front  per MB: fetch the neighbour bundle (needs the previous MB
--          committed), start the decider and stream it the 24 source
--          blocks, take the decision at done_o (together with the
--          neighbour context the emission needs), collect the
--          reconstruction stream into the line-buffer commit (bottom rows,
--          right columns) and commit with the decider's TotalCoeff counts
--          and modes; then straight on to the next MB.
--   back   per decision: run the header engine (fields go to the
--          dispatcher first), then turn the level stream into CAVLC
--          packets: zigzag, n_coefs, nC from the neighbour counts (spec
--          9.2.1), emitted or not per the coded_block_pattern.
--
-- The back sequencer of MB n therefore runs while the decider works on MB
-- n+1. After the last MB the rbsp stop bit is pushed and the dispatcher is
-- flushed; frame_done_o pulses when the last byte has been taken.
--
-- Source blocks come from a 24-word stream per MB (Y 0..15 raster, U 0..3,
-- V 0..3, sample k of a block at byte k) and are double-buffered so the
-- next MB's blocks arrive while the current one is being decided.
--
-- The slice header, SPS, PPS and NAL framing stay outside (host side), as
-- in the reference: the output is the MB-layer bit stream of one slice,
-- byte-aligned with the stop bit, exactly what the reference emits between
-- its slice header and the NAL end.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

entity mb_pipeline_controller is
    generic (
        MAX_MB_COLS : positive := 120;
        N_ENGINES   : positive := 1;
        PKT_DEPTH   : positive := 32;
        ORDER_DEPTH : positive := 64;
        DEBUG       : boolean  := false
    );
    port (
        clk           : in  std_logic;
        rst_n         : in  std_logic;
        -- frame control
        frame_start_i : in  std_logic;
        mbs_w_i       : in  unsigned(7 downto 0);
        mbs_h_i       : in  unsigned(7 downto 0);
        qp_i          : in  unsigned(5 downto 0);
        busy_o        : out std_logic;
        frame_done_o  : out std_logic;
        -- source MB stream (24 x 128-bit words per MB, raster MB order)
        src_valid_i   : in  std_logic;
        src_ready_o   : out std_logic;
        src_data_i    : in  std_logic_vector(127 downto 0);
        -- slice payload bytes
        out_valid     : out std_logic;
        out_ready     : in  std_logic;
        out_data      : out unsigned(7 downto 0);
        out_last      : out std_logic
    );
end entity;

architecture rtl of mb_pipeline_controller is

    type int16_t is array (0 to 15) of integer;
    constant ZIGZAG : int16_t := (0,1,4,8,5,2,3,6,9,12,13,10,7,11,14,15);
    type qpc_tab_t is array (0 to 51) of integer range 0 to 39;
    constant QPC_TAB : qpc_tab_t := (
         0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,
        29,30,31,32,32,33,34,34,35,35,36,36,37,37,37,38,38,38,39,39,39,39);

    subtype px128 is std_logic_vector(127 downto 0);
    function byte_of(v : std_logic_vector; k : integer) return std_logic_vector is
    begin
        return v(8 * k + 7 downto 8 * k);
    end function;

    ------------------------------------------------------------------
    -- configuration / position
    ------------------------------------------------------------------
    signal mbs_w, mbs_h : unsigned(7 downto 0) := (others => '0');
    signal mb_r, mb_c   : unsigned(7 downto 0) := (others => '0');
    signal cm_col       : unsigned(7 downto 0) := (others => '0');   -- column being committed
    signal qp_y, qp_c   : unsigned(5 downto 0) := (others => '0');

    ------------------------------------------------------------------
    -- source double buffer
    ------------------------------------------------------------------
    type src_buf_t is array (0 to 63) of px128;
    signal src_buf : src_buf_t;
    attribute ram_style : string;
    attribute ram_style of src_buf : signal is "distributed";
    signal fill_bank : std_logic := '0';
    signal fill_cnt  : integer range 0 to 24 := 0;
    signal bank_full : std_logic_vector(1 downto 0) := "00";
    signal use_bank  : std_logic := '0';
    signal src_wa, src_ra : integer range 0 to 63;
    signal src_rd    : px128;
    signal src_fire  : std_logic;
    signal md_word   : integer range 0 to 24 := 24;

    ------------------------------------------------------------------
    -- line buffer
    ------------------------------------------------------------------
    signal lb_frame_start, lb_row_start, lb_fetch_valid, lb_fetch_ready, lb_nb_valid : std_logic;
    signal lb_commit_valid, lb_commit_ready : std_logic;
    signal nb_top_y, nb_left_y : px128;
    signal nb_tr_y : std_logic_vector(31 downto 0);
    signal nb_tl_y, nb_tl_u, nb_tl_v : std_logic_vector(7 downto 0);
    signal nb_top_u, nb_top_v, nb_left_u, nb_left_v : std_logic_vector(63 downto 0);
    signal nb_ncy_top, nb_ncy_left : std_logic_vector(19 downto 0);
    signal nb_ncu_top, nb_ncu_left, nb_ncv_top, nb_ncv_left : std_logic_vector(9 downto 0);
    signal nb_m4_top, nb_m4_left : std_logic_vector(15 downto 0);
    signal nb_at, nb_al, nb_atl, nb_atr : std_logic;
    signal cm_y_bot, cm_uv_bot, cm_y_right, cm_uv_right : px128 := (others => '0');
    signal cm_ncy : std_logic_vector(79 downto 0) := (others => '0');
    signal cm_ncu, cm_ncv : std_logic_vector(19 downto 0) := (others => '0');
    signal cm_m4 : std_logic_vector(63 downto 0) := (others => '0');

    ------------------------------------------------------------------
    -- mode decider
    ------------------------------------------------------------------
    signal md_start, md_busy, md_done, md_sdone, md_sbusy : std_logic;
    signal md_src : px128;
    signal md_is4, md_cdc_nz, md_cac_nz : std_logic;
    signal md_mode16, md_modec : unsigned(1 downto 0);
    signal md_modes4 : std_logic_vector(63 downto 0);
    signal md_lnz : std_logic_vector(15 downto 0);
    signal md_bits_a, md_bits_b : unsigned(15 downto 0);
    signal md_tcy : std_logic_vector(79 downto 0);
    signal md_tcu, md_tcv : std_logic_vector(19 downto 0);
    signal md_blk_valid, md_blk_ready, md_rec_valid, md_rec_ready : std_logic;
    signal md_blk_plane, md_rec_plane : unsigned(1 downto 0);
    signal md_blk_kind : std_logic;
    signal md_blk_idx, md_rec_idx : unsigned(3 downto 0);
    signal md_blk_levels : level_array_t;
    signal md_rec_data : px128;
    signal md_done_f : std_logic := '0';       -- decision presented, not yet taken
    -- latched decision + the neighbour context the emission needs
    signal d_is4 : std_logic := '0';
    signal d_mode16, d_modec : unsigned(1 downto 0) := (others => '0');
    signal d_modes4 : std_logic_vector(63 downto 0) := (others => '0');
    signal d_lnz : std_logic_vector(15 downto 0) := (others => '0');
    signal d_cdc, d_cac : std_logic := '0';
    signal d_tcy : std_logic_vector(79 downto 0) := (others => '0');
    signal d_tcu, d_tcv : std_logic_vector(19 downto 0) := (others => '0');
    signal d_m4_top, d_m4_left : std_logic_vector(15 downto 0) := (others => '0');
    signal d_at, d_al : std_logic := '0';
    signal d_ncy_top, d_ncy_left : std_logic_vector(19 downto 0) := (others => '0');
    signal d_ncu_top, d_ncu_left, d_ncv_top, d_ncv_left : std_logic_vector(9 downto 0) := (others => '0');

    ------------------------------------------------------------------
    -- header engine
    ------------------------------------------------------------------
    signal hd_start, hd_ready, hd_done, hd_fvalid, hd_fready, hd_hasres : std_logic;
    signal hd_fbits : unsigned(15 downto 0);
    signal hd_flen : unsigned(5 downto 0);
    signal hd_cbpl : unsigned(3 downto 0);
    signal hd_cbpc : unsigned(1 downto 0);
    signal cbpl_q : unsigned(3 downto 0) := (others => '0');
    signal cbpc_q : unsigned(1 downto 0) := (others => '0');

    ------------------------------------------------------------------
    -- dispatcher
    ------------------------------------------------------------------
    signal dp_valid, dp_ready, dp_flushed : std_logic;
    signal dp_kind : unsigned(1 downto 0);
    signal dp_fbits : unsigned(7 downto 0);
    signal dp_flen : unsigned(5 downto 0);
    signal dp_pkt : level_packet_t;

    ------------------------------------------------------------------
    -- per-MB CAVLC context (emission side)
    ------------------------------------------------------------------
    type nc16_t is array (0 to 15) of integer range 0 to 16;
    type nc4_t  is array (0 to 3)  of integer range 0 to 16;
    signal ncy_loc : nc16_t := (others => 0);
    signal ncu_loc, ncv_loc : nc4_t := (others => 0);

    type st_t is (S_IDLE, S_WAIT_BANK, S_ROW, S_FETCH, S_FETCH_WAIT, S_START, S_SRC_PRE, S_SRC, S_WAIT_MD,
                  S_WAIT_REC, S_STOP, S_FLUSH, S_FLUSH_WAIT);
    signal st : st_t := S_IDLE;
    type est_t is (E_IDLE, E_HDR, E_HDR_WAIT, E_LEVELS, E_LEVEL_BUILD, E_LEVEL_PUSH);
    signal est : est_t := E_IDLE;
    signal em_start : std_logic := '0';
    signal frame_done_q : std_logic := '0';
    signal rec_cnt : integer range 0 to 24 := 0;
    signal lvl_cnt : integer range 0 to 31 := 0;
    signal rec_done : std_logic := '0';

    -- packet under construction; its TotalCoeff is counted from the
    -- registered packet (keeps the level-store read off the count path)
    signal pk_pkt : level_packet_t;
    signal pk_emit : std_logic := '0';
    signal pk_pl : integer range 0 to 2 := 0;
    signal pk_ix : integer range 0 to 15 := 0;
    signal pk_kind : std_logic := '0';
    signal pk_cnt_pend : std_logic := '0';
    -- level item captured from the decider's stream (keeps the decider's
    -- stream index off the nC / packet path)
    signal it_plane : integer range 0 to 2 := 0;
    signal it_idx : integer range 0 to 15 := 0;
    signal it_kind : std_logic := '0';
    signal it_levels : level_array_t := (others => (others => '0'));

begin

    ------------------------------------------------------------------
    -- Instances
    ------------------------------------------------------------------
    lb : entity work.line_buffer
        generic map (MAX_MB_COLS => MAX_MB_COLS)
        port map (clk => clk, rst_n => rst_n, frame_start_i => lb_frame_start, row_start_i => lb_row_start,
                  mbs_w_i => mbs_w, fetch_valid_i => lb_fetch_valid, fetch_mb_c_i => mb_c,
                  fetch_ready_o => lb_fetch_ready, nb_valid_o => lb_nb_valid,
                  top_y_o => nb_top_y, tr_y_o => nb_tr_y, tl_y_o => nb_tl_y, left_y_o => nb_left_y,
                  top_u_o => nb_top_u, top_v_o => nb_top_v, tl_u_o => nb_tl_u, tl_v_o => nb_tl_v,
                  left_u_o => nb_left_u, left_v_o => nb_left_v,
                  nc_y_top_o => nb_ncy_top, nc_y_left_o => nb_ncy_left, nc_u_top_o => nb_ncu_top,
                  nc_u_left_o => nb_ncu_left, nc_v_top_o => nb_ncv_top, nc_v_left_o => nb_ncv_left,
                  mode4_top_o => nb_m4_top, mode4_left_o => nb_m4_left,
                  avail_top_o => nb_at, avail_left_o => nb_al, avail_tl_o => nb_atl, avail_tr_o => nb_atr,
                  commit_valid_i => lb_commit_valid, commit_ready_o => lb_commit_ready, commit_mb_c_i => cm_col,
                  rec_y_bot_i => cm_y_bot, rec_uv_bot_i => cm_uv_bot, rec_y_right_i => cm_y_right,
                  rec_uv_right_i => cm_uv_right, nc_y_i => cm_ncy, nc_u_i => cm_ncu, nc_v_i => cm_ncv,
                  mode4_i => cm_m4);

    md : entity work.mode_decide_engine
        generic map (DEBUG => DEBUG)
        port map (clk => clk, rst_n => rst_n, start_i => md_start, busy_o => md_busy, done_o => md_done,
                  stream_done_o => md_sdone, qp_y_i => qp_y, qp_c_i => qp_c,
                  src_data_i => md_src, stream_busy_o => md_sbusy,
                  top_y_i => nb_top_y, tr_y_i => nb_tr_y, tl_y_i => nb_tl_y, left_y_i => nb_left_y,
                  avail_top_i => nb_at, avail_left_i => nb_al, avail_tl_i => nb_atl, avail_tr_i => nb_atr,
                  top_u_i => nb_top_u, left_u_i => nb_left_u, tl_u_i => nb_tl_u,
                  top_v_i => nb_top_v, left_v_i => nb_left_v, tl_v_i => nb_tl_v,
                  mode4_top_i => nb_m4_top, mode4_left_i => nb_m4_left,
                  is_i4x4_o => md_is4, mode16_o => md_mode16, modes4_o => md_modes4,
                  mode_chroma_o => md_modec, luma_nz_o => md_lnz,
                  chroma_dc_nz_o => md_cdc_nz, chroma_ac_nz_o => md_cac_nz,
                  bits_a_o => md_bits_a, bits_b_o => md_bits_b,
                  tc_y_o => md_tcy, tc_u_o => md_tcu, tc_v_o => md_tcv, dbg_j_o => open,
                  blk_valid_o => md_blk_valid, blk_ready_i => md_blk_ready, blk_plane_o => md_blk_plane,
                  blk_kind_o => md_blk_kind, blk_idx_o => md_blk_idx, blk_levels_o => md_blk_levels,
                  rec_valid_o => md_rec_valid, rec_ready_i => md_rec_ready, rec_plane_o => md_rec_plane,
                  rec_idx_o => md_rec_idx, rec_data_o => md_rec_data);

    hd : entity work.mb_header_engine
        port map (clk => clk, rst_n => rst_n, start_i => hd_start, ready_o => hd_ready,
                  is_i4x4_i => d_is4, mode16_i => d_mode16, modes4_i => d_modes4, mode_chroma_i => d_modec,
                  luma_nz_i => d_lnz, chroma_dc_nz_i => d_cdc, chroma_ac_nz_i => d_cac,
                  mode4_top_i => d_m4_top, mode4_left_i => d_m4_left,
                  avail_top_i => d_at, avail_left_i => d_al,
                  fbits_o => hd_fbits, flen_o => hd_flen, fvalid_o => hd_fvalid, fready_i => hd_fready,
                  done_o => hd_done, hdr_bits_o => open, cbp_luma_o => hd_cbpl, cbp_chroma_o => hd_cbpc,
                  has_residual_o => hd_hasres);

    dp : entity work.cavlc_dispatch
        generic map (N_ENGINES => N_ENGINES, PKT_DEPTH => PKT_DEPTH, ORDER_DEPTH => ORDER_DEPTH)
        port map (clk => clk, rst_n => rst_n, in_valid => dp_valid, in_ready => dp_ready, in_kind => dp_kind,
                  in_fbits => dp_fbits, in_flen => dp_flen, in_pkt => dp_pkt,
                  out_valid => out_valid, out_ready => out_ready, out_data => out_data, out_last => out_last,
                  flushed_o => dp_flushed);

    ------------------------------------------------------------------
    -- Source double buffer: fill one bank from the input stream while
    -- the decider is fed from the other
    ------------------------------------------------------------------
    -- no source words while idle: frame_start resets the fill counters
    src_ready_o <= '1' when (st /= S_IDLE and fill_cnt < 24 and bank_full(to_integer(unsigned'("" & fill_bank))) = '0') else '0';
    src_fire    <= src_valid_i and src_ready_o;
    src_wa      <= to_integer(unsigned'("" & fill_bank)) * 32 + fill_cnt;
    src_ra      <= to_integer(unsigned'("" & use_bank)) * 32 + md_word;
    src_rd      <= src_buf(src_ra);
    md_src      <= src_rd;

    src_buf_p : process(clk)
    begin
        if rising_edge(clk) then
            if src_fire = '1' then src_buf(src_wa) <= src_data_i; end if;
        end if;
    end process;

    fill_p : process(clk, rst_n)
    begin
        if rst_n = '0' then
            fill_bank <= '0'; fill_cnt <= 0; bank_full <= "00";
        elsif rising_edge(clk) then
            if st = S_IDLE and frame_start_i = '1' then
                fill_bank <= '0'; fill_cnt <= 0; bank_full <= "00";
            else
                if src_fire = '1' then
                    if fill_cnt = 23 then
                        bank_full(to_integer(unsigned'("" & fill_bank))) <= '1';
                        fill_bank <= not fill_bank;
                        fill_cnt <= 0;
                    else
                        fill_cnt <= fill_cnt + 1;
                    end if;
                end if;
                -- a bank is released once the decider has taken all 24 words
                if st = S_SRC and md_word = 23 then
                    bank_full(to_integer(unsigned'("" & use_bank))) <= '0';
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Reconstruction stream -> line-buffer commit registers
    ------------------------------------------------------------------
    md_rec_ready <= '1';
    rec_p : process(clk)
        variable ix : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            if md_rec_valid = '1' then
                ix := to_integer(md_rec_idx);
                if md_rec_plane = 0 then
                    if ix >= 12 then
                        cm_y_bot(32 * (ix - 12) + 31 downto 32 * (ix - 12)) <= md_rec_data(127 downto 96);
                    end if;
                    if (ix mod 4) = 3 then
                        cm_y_right(32 * (ix / 4) + 31 downto 32 * (ix / 4)) <=
                            byte_of(md_rec_data, 15) & byte_of(md_rec_data, 11) & byte_of(md_rec_data, 7) & byte_of(md_rec_data, 3);
                    end if;
                else
                    -- chroma: U at even bytes, V at odd bytes (NV12)
                    if ix >= 2 then
                        for j in 0 to 3 loop
                            if md_rec_plane = 1 then
                                cm_uv_bot(16 * ((ix - 2) * 4 + j) + 7 downto 16 * ((ix - 2) * 4 + j)) <= byte_of(md_rec_data, 12 + j);
                            else
                                cm_uv_bot(16 * ((ix - 2) * 4 + j) + 15 downto 16 * ((ix - 2) * 4 + j) + 8) <= byte_of(md_rec_data, 12 + j);
                            end if;
                        end loop;
                    end if;
                    if (ix mod 2) = 1 then
                        for r in 0 to 3 loop
                            if md_rec_plane = 1 then
                                cm_uv_right(16 * ((ix / 2) * 4 + r) + 7 downto 16 * ((ix / 2) * 4 + r)) <= byte_of(md_rec_data, 4 * r + 3);
                            else
                                cm_uv_right(16 * ((ix / 2) * 4 + r) + 15 downto 16 * ((ix / 2) * 4 + r) + 8) <= byte_of(md_rec_data, 4 * r + 3);
                            end if;
                        end loop;
                    end if;
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Dispatcher input mux: header fields, then packets, then stop/flush
    ------------------------------------------------------------------
    hd_fready <= dp_ready when est = E_HDR_WAIT else '0';

    dp_mux_p : process(all)
    begin
        dp_valid <= '0'; dp_kind <= "00"; dp_fbits <= (others => '0'); dp_flen <= (others => '0');
        dp_pkt <= pk_pkt;
        if est = E_HDR_WAIT then
            dp_valid <= hd_fvalid;
            dp_fbits <= hd_fbits(7 downto 0);
            dp_flen  <= hd_flen;
        elsif est = E_LEVEL_PUSH then
            dp_valid <= pk_emit;
            dp_kind  <= "01";
        elsif st = S_STOP and est = E_IDLE and em_start = '0' then
            dp_valid <= '1';
            dp_fbits <= x"01";
            dp_flen  <= to_unsigned(1, 6);
        elsif st = S_FLUSH then
            dp_valid <= '1';
            dp_kind  <= "10";
        end if;
    end process;

    md_blk_ready <= '1' when est = E_LEVELS else '0';

    ------------------------------------------------------------------
    -- Front sequencer: fetch, decide, commit
    ------------------------------------------------------------------
    front_p : process(clk, rst_n)
    begin
        if rst_n = '0' then
            st <= S_IDLE; frame_done_q <= '0';
            lb_frame_start <= '0'; lb_row_start <= '0'; lb_fetch_valid <= '0'; lb_commit_valid <= '0';
            md_start <= '0'; md_word <= 24; use_bank <= '0'; md_done_f <= '0'; em_start <= '0';
            rec_done <= '0'; rec_cnt <= 0;
        elsif rising_edge(clk) then
            frame_done_q <= '0';
            lb_frame_start <= '0'; lb_row_start <= '0';
            lb_commit_valid <= '0';
            md_start <= '0'; em_start <= '0';
            if md_done = '1' then md_done_f <= '1'; end if;

            -- reconstruction stream bookkeeping (independent of the state)
            if md_rec_valid = '1' then
                if rec_cnt = 23 then rec_done <= '1'; rec_cnt <= 0; else rec_cnt <= rec_cnt + 1; end if;
            end if;

            case st is
                when S_IDLE =>
                    if frame_start_i = '1' then
                        mbs_w <= mbs_w_i; mbs_h <= mbs_h_i;
                        qp_y <= qp_i; qp_c <= to_unsigned(QPC_TAB(to_integer(qp_i)), 6);
                        mb_r <= (others => '0'); mb_c <= (others => '0');
                        use_bank <= '0'; rec_done <= '0'; rec_cnt <= 0; md_done_f <= '0';
                        lb_frame_start <= '1';
                        st <= S_WAIT_BANK;
                    end if;
                when S_WAIT_BANK =>
                    if bank_full(to_integer(unsigned'("" & use_bank))) = '1' then st <= S_ROW; end if;
                when S_ROW =>
                    if mb_c = 0 and mb_r > 0 then lb_row_start <= '1'; end if;
                    st <= S_FETCH;
                when S_FETCH =>
                    -- one cycle after a row start so top_idx has flipped
                    if lb_fetch_ready = '1' then lb_fetch_valid <= '1'; st <= S_FETCH_WAIT; end if;
                when S_FETCH_WAIT =>
                    lb_fetch_valid <= '0';
                    if lb_nb_valid = '1' then st <= S_START; end if;
                when S_START =>
                    -- the decider must be idle; its reconstruction stream is out
                    -- (we committed it) though its level stream may still run
                    if md_busy = '0' then
                        md_start <= '1'; md_word <= 0;
                        rec_done <= '0'; rec_cnt <= 0; md_done_f <= '0';
                        st <= S_SRC_PRE;
                    end if;
                when S_SRC_PRE =>
                    -- md_start is seen by the decider on this edge; word 0 must be
                    -- on the bus during the following cycle, so hold md_word = 0
                    st <= S_SRC;
                when S_SRC =>
                    -- word md_word is on md_src this cycle
                    if md_word = 23 then
                        md_word <= 24; use_bank <= not use_bank; st <= S_WAIT_MD;
                    else
                        md_word <= md_word + 1;
                    end if;
                when S_WAIT_MD =>
                    -- take the decision once the emission of the previous MB is
                    -- done with the latched context
                    if md_done_f = '1' and est = E_IDLE and em_start = '0' then
                        d_is4 <= md_is4; d_mode16 <= md_mode16; d_modes4 <= md_modes4; d_modec <= md_modec;
                        d_lnz <= md_lnz; d_cdc <= md_cdc_nz; d_cac <= md_cac_nz;
                        d_tcy <= md_tcy; d_tcu <= md_tcu; d_tcv <= md_tcv;
                        d_m4_top <= nb_m4_top; d_m4_left <= nb_m4_left; d_at <= nb_at; d_al <= nb_al;
                        d_ncy_top <= nb_ncy_top; d_ncy_left <= nb_ncy_left;
                        d_ncu_top <= nb_ncu_top; d_ncu_left <= nb_ncu_left;
                        d_ncv_top <= nb_ncv_top; d_ncv_left <= nb_ncv_left;
                        em_start <= '1';
                        md_done_f <= '0';
                        st <= S_WAIT_REC;
                    end if;
                when S_WAIT_REC =>
                    -- commit this MB to the line buffer once its reconstruction
                    -- stream has been collected
                    if rec_done = '1' and lb_commit_ready = '1' then
                        cm_ncy <= d_tcy; cm_ncu <= d_tcu; cm_ncv <= d_tcv;
                        if d_is4 = '1' then cm_m4 <= d_modes4; else cm_m4 <= x"2222222222222222"; end if;
                        lb_commit_valid <= '1';
                        cm_col <= mb_c;
                        rec_done <= '0';
                        if mb_c = mbs_w - 1 then
                            mb_c <= (others => '0');
                            if mb_r = mbs_h - 1 then st <= S_STOP; else mb_r <= mb_r + 1; st <= S_WAIT_BANK; end if;
                        else
                            mb_c <= mb_c + 1; st <= S_WAIT_BANK;
                        end if;
                    end if;
                when S_STOP =>
                    -- after the last MB's emission: stop bit, then flush
                    if est = E_IDLE and em_start = '0' and dp_ready = '1' then st <= S_FLUSH; end if;
                when S_FLUSH =>
                    if dp_ready = '1' then st <= S_FLUSH_WAIT; end if;
                when S_FLUSH_WAIT =>
                    if dp_flushed = '1' then frame_done_q <= '1'; st <= S_IDLE; end if;
            end case;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Back sequencer: header fields, then the level stream as packets
    ------------------------------------------------------------------
    back_p : process(clk, rst_n)
        variable pl, ix, br, bc : integer range 0 to 15;
        variable nt, nl, ncv : integer range 0 to 16;
        variable tok, lok : boolean;
        variable zz : level_array_t;
        variable cnt : integer range 0 to 16;
        variable emit : boolean;
        variable p : level_packet_t;
    begin
        if rst_n = '0' then
            est <= E_IDLE; hd_start <= '0'; pk_emit <= '0'; pk_cnt_pend <= '0';
        elsif rising_edge(clk) then
            hd_start <= '0';
            case est is
                when E_IDLE =>
                    if em_start = '1' then
                        ncy_loc <= (others => 0); ncu_loc <= (others => 0); ncv_loc <= (others => 0);
                        est <= E_HDR;
                    end if;
                when E_HDR =>
                    if hd_ready = '1' then hd_start <= '1'; est <= E_HDR_WAIT; end if;
                when E_HDR_WAIT =>
                    -- fields flow to the dispatcher; done when the engine is idle again
                    -- and its last field has been accepted
                    if hd_ready = '1' and hd_fvalid = '0' and hd_start = '0' then
                        cbpl_q <= hd_cbpl; cbpc_q <= hd_cbpc;
                        lvl_cnt <= 0;
                        est <= E_LEVELS;
                    end if;
                when E_LEVELS =>
                    if md_blk_valid = '1' then
                        it_plane <= to_integer(md_blk_plane); it_idx <= to_integer(md_blk_idx);
                        it_kind <= md_blk_kind; it_levels <= md_blk_levels;
                        lvl_cnt <= lvl_cnt + 1;
                        est <= E_LEVEL_BUILD;
                    elsif md_sdone = '1' or (lvl_cnt > 0 and md_sbusy = '0') then
                        est <= E_IDLE;
                    end if;
                when E_LEVEL_BUILD =>
                        pl := it_plane; ix := it_idx;
                        br := ix / 4; bc := ix mod 4;
                        -- nC from the neighbours' TotalCoeff (spec 9.2.1)
                        if pl = 0 then
                            if it_kind = '1' then br := 0; bc := 0; end if;
                            if br > 0 then nt := ncy_loc((br - 1) * 4 + bc); tok := true;
                            else nt := to_integer(unsigned(d_ncy_top(5 * bc + 4 downto 5 * bc))); tok := (d_at = '1'); end if;
                            if bc > 0 then nl := ncy_loc(br * 4 + bc - 1); lok := true;
                            else nl := to_integer(unsigned(d_ncy_left(5 * br + 4 downto 5 * br))); lok := (d_al = '1'); end if;
                        else
                            br := ix / 2; bc := ix mod 2;
                            if pl = 1 then
                                if br > 0 then nt := ncu_loc((br - 1) * 2 + bc); tok := true;
                                else nt := to_integer(unsigned(d_ncu_top(5 * bc + 4 downto 5 * bc))); tok := (d_at = '1'); end if;
                                if bc > 0 then nl := ncu_loc(br * 2 + bc - 1); lok := true;
                                else nl := to_integer(unsigned(d_ncu_left(5 * br + 4 downto 5 * br))); lok := (d_al = '1'); end if;
                            else
                                if br > 0 then nt := ncv_loc((br - 1) * 2 + bc); tok := true;
                                else nt := to_integer(unsigned(d_ncv_top(5 * bc + 4 downto 5 * bc))); tok := (d_at = '1'); end if;
                                if bc > 0 then nl := ncv_loc(br * 2 + bc - 1); lok := true;
                                else nl := to_integer(unsigned(d_ncv_left(5 * br + 4 downto 5 * br))); lok := (d_al = '1'); end if;
                            end if;
                        end if;
                        if tok and lok then ncv := (nt + nl + 1) / 2;
                        elsif tok then ncv := nt;
                        elsif lok then ncv := nl;
                        else ncv := 0;
                        end if;
                        -- zigzag and packet shape
                        for k in 0 to 15 loop zz(k) := it_levels(ZIGZAG(k)); end loop;
                        p.nC := to_unsigned(ncv, 5);
                        p.levels := (others => (others => '0'));
                        if it_kind = '1' then
                            if pl = 0 then
                                p.block_type := to_unsigned(BLK_LUMA_DC_16x16, 3); p.n_coefs := to_unsigned(16, 5);
                                p.levels := zz;
                                emit := true;
                            else
                                p.block_type := to_unsigned(BLK_CHROMA_DC, 3); p.n_coefs := to_unsigned(4, 5);
                                p.nC := to_unsigned(31, 5);
                                for k in 0 to 3 loop p.levels(k) := it_levels(k); end loop;
                                emit := (cbpc_q >= 1);
                            end if;
                        elsif pl = 0 and d_is4 = '1' then
                            p.block_type := to_unsigned(BLK_LUMA_FULL, 3); p.n_coefs := to_unsigned(16, 5);
                            p.levels := zz;
                            -- quadrant of the block: (br/2)*2 + bc/2 in raster terms
                            emit := (cbpl_q((br / 2) * 2 + (bc / 2)) = '1');
                        elsif pl = 0 then
                            p.block_type := to_unsigned(BLK_LUMA_AC, 3); p.n_coefs := to_unsigned(15, 5);
                            for k in 0 to 14 loop p.levels(k) := zz(k + 1); end loop;
                            emit := (cbpl_q(0) = '1');
                        else
                            p.block_type := to_unsigned(BLK_CHROMA_AC, 3); p.n_coefs := to_unsigned(15, 5);
                            for k in 0 to 14 loop p.levels(k) := zz(k + 1); end loop;
                            emit := (cbpc_q = 2);
                        end if;
                        pk_pkt <= p;
                        pk_pl <= pl; pk_ix <= ix; pk_kind <= it_kind; pk_cnt_pend <= '1';
                        if emit then pk_emit <= '1'; else pk_emit <= '0'; end if;
                        est <= E_LEVEL_PUSH;
                when E_LEVEL_PUSH =>
                    -- TotalCoeff for the neighbours (0 when the block is not coded)
                    if pk_cnt_pend = '1' then
                        cnt := 0;
                        for k in 0 to 15 loop
                            if k < to_integer(pk_pkt.n_coefs) and pk_pkt.levels(k) /= 0 then cnt := cnt + 1; end if;
                        end loop;
                        if pk_emit = '0' then cnt := 0; end if;
                        if pk_kind = '0' then
                            if pk_pl = 0 then ncy_loc(pk_ix) <= cnt;
                            elsif pk_pl = 1 then ncu_loc(pk_ix) <= cnt;
                            else ncv_loc(pk_ix) <= cnt;
                            end if;
                        end if;
                        pk_cnt_pend <= '0';
                    end if;
                    if pk_emit = '0' or dp_ready = '1' then
                        pk_emit <= '0';
                        if lvl_cnt = 27 or (d_is4 = '1' and lvl_cnt = 26) then est <= E_IDLE; else est <= E_LEVELS; end if;
                    end if;
            end case;
        end if;
    end process;

    busy_o       <= '0' when st = S_IDLE else '1';
    frame_done_o <= frame_done_q;

end architecture;
