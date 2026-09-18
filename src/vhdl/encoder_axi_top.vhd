--------------------------------------------------------------------------------
-- encoder_axi_top.vhd — the encoder kernel behind standard AXI interfaces,
-- ready to drop into a Zynq block design:
--
--   s_axi   AXI4-Lite slave, control / status registers (below)
--   s_axis  AXI4-Stream slave, NV12 pixels: per MB row 16 luma lines then 8
--           chroma lines, 4 bytes per beat (a DMA transfer per MB row, or
--           one per frame, both work; tlast / tkeep are ignored). tready is
--           held low until START, so the DMA may be started before or after
--           the frame is started.
--   m_axis  AXI4-Stream master, the slice payload of each frame (MB-layer
--           bits, byte-aligned with the rbsp stop bit), tkeep on the final
--           beat, tlast on the final beat of the frame
--   irq     level interrupt, asserted while DONE is set and IRQ_EN is set
--
-- Register map (byte offsets, 32-bit):
--   0x00 CTRL     W  bit 0 START      pulse a frame start (ignored while BUSY)
--                    bit 1 SOFT_RESET  reset the kernel (16 cycles)
--                 RW bit 8 IRQ_EN
--   0x04 CONFIG   RW [7:0] mbs_w  [15:8] mbs_h  [21:16] qp   (latched at START)
--   0x08 STATUS   R  bit 0 BUSY  bit 1 DONE (sticky; set once the frame
--                    has been EMITTED, not merely once the kernel stopped --
--                    the two differ under input backpressure)  bit 2 s_axis_tready
--                    bit 3 m_axis_tvalid
--   0x0C DONE_CLR W  write 1 clears DONE (and the interrupt)
--   0x10 FRAMES   R  frames completed since reset
--   0x14 CYCLES   R  aclk cycles of the last frame, START to frame done
--   0x18 BYTES    R  payload bytes of the last frame
--   0x1C ID       R  0x48323634 ("H264")
--   0x20 VERSION  R  0x00010003
--   0x24 RC_CTRL  RW bit 0 RC_EN      per-MB QP correction on (latched at START)
--                    bit 1 MAP_VALID  weight the expected spend by the previous
--                                     frame's per-MB bit map (kept in the
--                                     kernel); else every MB weighs 1
--                    [13:8] qp_min  [21:16] qp_max  [27:24] lag (MBs between a
--                    decision and the bits it can see; 2 = the pipeline depth)
--   0x28 RC_TARGET RW frame target, bits (latched at START)
--   0x2C RC_SCALE  RW target * 2^16 / w_total, w_total = sum of the weights
--                    (RC_WTOTAL of the previous frame, or the MB count)
--   0x30 RC_WTOTAL R  sum of the MB-layer bits of the last frame, per MB
--                    saturated to 16 bits: the next frame's w_total
--        The frame QP is CONFIG.qp as before; with RC_EN clear the stream is
--        the one 1.2 produced. The host keeps the frame-level controller
--        (bucket, frame model); see docs/rate-control-mb-vs-frame.md.
--        1.3 adds the per-MB rate control (registers 0x24..0x30).
--        1.2 adds: DONE waits for the output to drain, so BYTES is
--        right and a host may start the next frame on DONE.
--        1.0 = original. 1.1 = the output-path fixes: bit_packer
--        HOLD_LAST (tlast on a payload that is a whole number of
--        bytes), the merger block-trim latch (4 bits inserted under
--        output stalls), and bytes_last no longer dropping the final
--        beat. Bump this on any change software can observe: PL
--        configuration survives an ELF reload, so without a version
--        there is no way to tell which bitstream is really loaded.
--
-- The slice header, SPS/PPS and NAL framing are done by the host around
-- the payload, exactly as the C reference splits them (src/nal.c).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity encoder_axi_top is
    generic (
        MAX_W     : positive := 1920;
        N_ENGINES : positive := 2;
        MAX_MBS   : positive := 8192     -- per-MB bit map depth (1080p = 8160)
    );
    port (
        aclk            : in  std_logic;
        aresetn         : in  std_logic;
        -- AXI4-Lite slave
        s_axi_awaddr    : in  std_logic_vector(7 downto 0);
        s_axi_awvalid   : in  std_logic;
        s_axi_awready   : out std_logic;
        s_axi_wdata     : in  std_logic_vector(31 downto 0);
        s_axi_wstrb     : in  std_logic_vector(3 downto 0);
        s_axi_wvalid    : in  std_logic;
        s_axi_wready    : out std_logic;
        s_axi_bresp     : out std_logic_vector(1 downto 0);
        s_axi_bvalid    : out std_logic;
        s_axi_bready    : in  std_logic;
        s_axi_araddr    : in  std_logic_vector(7 downto 0);
        s_axi_arvalid   : in  std_logic;
        s_axi_arready   : out std_logic;
        s_axi_rdata     : out std_logic_vector(31 downto 0);
        s_axi_rresp     : out std_logic_vector(1 downto 0);
        s_axi_rvalid    : out std_logic;
        s_axi_rready    : in  std_logic;
        -- AXI4-Stream slave: pixels
        s_axis_tdata    : in  std_logic_vector(31 downto 0);
        s_axis_tkeep    : in  std_logic_vector(3 downto 0);
        s_axis_tlast    : in  std_logic;
        s_axis_tvalid   : in  std_logic;
        s_axis_tready   : out std_logic;
        -- AXI4-Stream master: slice payload
        m_axis_tdata    : out std_logic_vector(31 downto 0);
        m_axis_tkeep    : out std_logic_vector(3 downto 0);
        m_axis_tlast    : out std_logic;
        m_axis_tvalid   : out std_logic;
        m_axis_tready   : in  std_logic;
        -- interrupt
        irq             : out std_logic
    );
end entity;

architecture rtl of encoder_axi_top is

    -- kernel reset: asynchronous assert, synchronous release, plus the
    -- register-driven soft reset
    signal rst_sync   : std_logic_vector(1 downto 0) := "00";
    signal soft_cnt   : unsigned(4 downto 0) := (others => '0');
    signal krst_n     : std_logic;

    -- registers
    signal cfg_w, cfg_h  : unsigned(7 downto 0) := (others => '0');
    signal cfg_qp        : unsigned(5 downto 0) := (others => '0');
    signal irq_en        : std_logic := '0';
    signal start_pulse   : std_logic := '0';
    signal done_flag     : std_logic := '0';
    signal frames        : unsigned(31 downto 0) := (others => '0');
    signal cyc_run       : unsigned(31 downto 0) := (others => '0');
    signal cyc_last      : unsigned(31 downto 0) := (others => '0');
    signal bytes_run     : unsigned(31 downto 0) := (others => '0');
    signal bytes_last    : unsigned(31 downto 0) := (others => '0');
    signal counting      : std_logic := '0';
    -- The kernel can finish while its last output beat is still in flight,
    -- so "kernel done" and "frame emitted" are two different events. DONE
    -- waits for both: a host that starts the next frame on DONE would
    -- otherwise do so with a beat still pending.
    signal kdone_q       : std_logic := '0';   -- kernel reported done
    signal drained_q     : std_logic := '0';   -- tlast beat handshaken
    signal frame_active  : std_logic := '0';   -- START seen, frame not yet done: pixels accepted

    -- kernel
    signal k_busy, k_done : std_logic;
    signal k_s_ready, k_o_valid, k_o_last : std_logic;
    signal k_o_data : std_logic_vector(31 downto 0);
    signal k_o_keep : std_logic_vector(3 downto 0);
    signal mbs_w_q, mbs_h_q : unsigned(7 downto 0) := (others => '0');
    signal qp_q : unsigned(5 downto 0) := (others => '0');
    -- rate control registers and their START-latched copies
    signal cfg_rc_en, cfg_rc_map : std_logic := '0';
    signal cfg_rc_qpmin, cfg_rc_qpmax : unsigned(5 downto 0) := (others => '0');
    signal cfg_rc_lag : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal cfg_rc_target, cfg_rc_scale : unsigned(31 downto 0) := (others => '0');
    signal rc_en_q, rc_map_q : std_logic := '0';
    signal rc_qpmin_q, rc_qpmax_q : unsigned(5 downto 0) := (others => '0');
    signal rc_lag_q : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal rc_target_q, rc_scale_q : unsigned(31 downto 0) := (others => '0');
    signal k_wtotal : unsigned(31 downto 0);

    -- AXI-Lite
    signal awready_q, wready_q, bvalid_q, arready_q, rvalid_q : std_logic := '0';
    signal rdata_q : std_logic_vector(31 downto 0) := (others => '0');
    signal wr_addr : std_logic_vector(7 downto 0) := (others => '0');

    function keep_count(k : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case k is
            when "1111" => return to_unsigned(4, 3);
            when "0111" => return to_unsigned(3, 3);
            when "0011" => return to_unsigned(2, 3);
            when others => return to_unsigned(1, 3);
        end case;
    end function;

begin

    ------------------------------------------------------------------
    -- reset
    ------------------------------------------------------------------
    rst_p : process(aclk, aresetn)
    begin
        if aresetn = '0' then
            rst_sync <= "00";
        elsif rising_edge(aclk) then
            rst_sync <= rst_sync(0) & '1';
        end if;
    end process;

    soft_p : process(aclk)
    begin
        if rising_edge(aclk) then
            if rst_sync(1) = '0' then
                soft_cnt <= (others => '0');
            elsif s_axi_wvalid = '1' and s_axi_awvalid = '1' and awready_q = '1' and
                  s_axi_awaddr(7 downto 2) = "000000" and s_axi_wstrb(0) = '1' and s_axi_wdata(1) = '1' then
                soft_cnt <= to_unsigned(16, 5);
            elsif soft_cnt /= 0 then
                soft_cnt <= soft_cnt - 1;
            end if;
        end if;
    end process;

    krst_n <= rst_sync(1) and not (or soft_cnt);

    ------------------------------------------------------------------
    -- kernel
    ------------------------------------------------------------------
    core : entity work.encoder_top
        generic map (MAX_W => MAX_W, N_ENGINES => N_ENGINES, MAX_MBS => MAX_MBS, DEBUG => false)
        port map (clk => aclk, rst_n => krst_n,
                  frame_start_i => start_pulse, mbs_w_i => mbs_w_q, mbs_h_i => mbs_h_q, qp_i => qp_q,
                  busy_o => k_busy, frame_done_o => k_done,
                  rc_en_i => rc_en_q, rc_map_valid_i => rc_map_q, rc_qp_min_i => rc_qpmin_q,
                  rc_qp_max_i => rc_qpmax_q, rc_target_i => rc_target_q, rc_scale_i => rc_scale_q,
                  rc_lag_i => rc_lag_q, rc_wtotal_o => k_wtotal,
                  s_valid_i => s_axis_tvalid, s_ready_o => k_s_ready, s_data_i => s_axis_tdata,
                  o_valid_o => k_o_valid, o_ready_i => m_axis_tready, o_data_o => k_o_data,
                  o_keep_o => k_o_keep, o_last_o => k_o_last);

    s_axis_tready <= k_s_ready and frame_active;
    m_axis_tvalid <= k_o_valid;
    m_axis_tdata  <= k_o_data;
    m_axis_tkeep  <= k_o_keep;
    m_axis_tlast  <= k_o_last;

    ------------------------------------------------------------------
    -- AXI4-Lite slave: both address and data are accepted together;
    -- one outstanding transaction per direction
    ------------------------------------------------------------------
    s_axi_awready <= awready_q;
    s_axi_wready  <= wready_q;
    s_axi_bvalid  <= bvalid_q;
    s_axi_bresp   <= "00";
    s_axi_arready <= arready_q;
    s_axi_rvalid  <= rvalid_q;
    s_axi_rdata   <= rdata_q;
    s_axi_rresp   <= "00";

    axi_p : process(aclk)
        variable wr : boolean;
        variable a : std_logic_vector(5 downto 0);
        variable bytes_tot : unsigned(31 downto 0);
        variable beat, lastbeat : boolean;
    begin
        if rising_edge(aclk) then
            if rst_sync(1) = '0' then
                awready_q <= '0'; wready_q <= '0'; bvalid_q <= '0'; arready_q <= '0'; rvalid_q <= '0';
                cfg_w <= (others => '0'); cfg_h <= (others => '0'); cfg_qp <= (others => '0');
                irq_en <= '0'; start_pulse <= '0'; done_flag <= '0';
                frames <= (others => '0'); cyc_run <= (others => '0'); cyc_last <= (others => '0');
                bytes_run <= (others => '0'); bytes_last <= (others => '0'); counting <= '0';
                kdone_q <= '0'; drained_q <= '0';
                frame_active <= '0';
                mbs_w_q <= (others => '0'); mbs_h_q <= (others => '0'); qp_q <= (others => '0');
                cfg_rc_en <= '0'; cfg_rc_map <= '0'; cfg_rc_qpmin <= (others => '0'); cfg_rc_qpmax <= (others => '0');
                cfg_rc_lag <= to_unsigned(2, 4); cfg_rc_target <= (others => '0'); cfg_rc_scale <= (others => '0');
                rc_en_q <= '0'; rc_map_q <= '0'; rc_qpmin_q <= (others => '0'); rc_qpmax_q <= (others => '0');
                rc_lag_q <= to_unsigned(2, 4); rc_target_q <= (others => '0'); rc_scale_q <= (others => '0');
            else
                start_pulse <= '0';

                -- write channel
                wr := false;
                if awready_q = '1' then
                    awready_q <= '0'; wready_q <= '0'; bvalid_q <= '1';
                    wr := true;
                elsif s_axi_awvalid = '1' and s_axi_wvalid = '1' and bvalid_q = '0' then
                    awready_q <= '1'; wready_q <= '1';
                end if;
                if bvalid_q = '1' and s_axi_bready = '1' then bvalid_q <= '0'; end if;
                if wr then
                    a := s_axi_awaddr(7 downto 2);
                    case a is
                        when "000000" =>     -- CTRL
                            if s_axi_wstrb(0) = '1' and s_axi_wdata(0) = '1' and k_busy = '0' then
                                start_pulse <= '1';
                                mbs_w_q <= cfg_w; mbs_h_q <= cfg_h; qp_q <= cfg_qp;
                                rc_en_q <= cfg_rc_en; rc_map_q <= cfg_rc_map;
                                rc_qpmin_q <= cfg_rc_qpmin; rc_qpmax_q <= cfg_rc_qpmax; rc_lag_q <= cfg_rc_lag;
                                rc_target_q <= cfg_rc_target; rc_scale_q <= cfg_rc_scale;
                                cyc_run <= (others => '0'); bytes_run <= (others => '0'); counting <= '1';
                                kdone_q <= '0'; drained_q <= '0';
                                frame_active <= '1';
                            end if;
                            if s_axi_wstrb(1) = '1' then irq_en <= s_axi_wdata(8); end if;
                        when "000001" =>     -- CONFIG
                            if s_axi_wstrb(0) = '1' then cfg_w <= unsigned(s_axi_wdata(7 downto 0)); end if;
                            if s_axi_wstrb(1) = '1' then cfg_h <= unsigned(s_axi_wdata(15 downto 8)); end if;
                            if s_axi_wstrb(2) = '1' then cfg_qp <= unsigned(s_axi_wdata(21 downto 16)); end if;
                        when "000011" =>     -- DONE_CLR
                            if s_axi_wstrb(0) = '1' and s_axi_wdata(0) = '1' then done_flag <= '0'; end if;
                        when "001001" =>     -- RC_CTRL
                            if s_axi_wstrb(0) = '1' then cfg_rc_en <= s_axi_wdata(0); cfg_rc_map <= s_axi_wdata(1); end if;
                            if s_axi_wstrb(1) = '1' then cfg_rc_qpmin <= unsigned(s_axi_wdata(13 downto 8)); end if;
                            if s_axi_wstrb(2) = '1' then cfg_rc_qpmax <= unsigned(s_axi_wdata(21 downto 16)); end if;
                            if s_axi_wstrb(3) = '1' then cfg_rc_lag <= unsigned(s_axi_wdata(27 downto 24)); end if;
                        when "001010" =>     -- RC_TARGET
                            if s_axi_wstrb = "1111" then cfg_rc_target <= unsigned(s_axi_wdata); end if;
                        when "001011" =>     -- RC_SCALE
                            if s_axi_wstrb = "1111" then cfg_rc_scale <= unsigned(s_axi_wdata); end if;
                        when others => null;
                    end case;
                end if;

                -- read channel
                if arready_q = '1' then
                    arready_q <= '0';
                elsif s_axi_arvalid = '1' and rvalid_q = '0' and arready_q = '0' then
                    arready_q <= '1';
                    rvalid_q <= '1';
                    case s_axi_araddr(7 downto 2) is
                        when "000000" => rdata_q <= (8 => irq_en, others => '0');
                        when "000001" => rdata_q <= x"00" & "00" & std_logic_vector(cfg_qp) & std_logic_vector(cfg_h) & std_logic_vector(cfg_w);
                        when "000010" => rdata_q <= (0 => k_busy, 1 => done_flag, 2 => k_s_ready and frame_active, 3 => k_o_valid, others => '0');
                        when "000100" => rdata_q <= std_logic_vector(frames);
                        when "000101" => rdata_q <= std_logic_vector(cyc_last);
                        when "000110" => rdata_q <= std_logic_vector(bytes_last);
                        when "000111" => rdata_q <= x"48323634";
                        when "001000" => rdata_q <= x"00010003";
                        when "001001" => rdata_q <= x"0" & std_logic_vector(cfg_rc_lag) & "00" & std_logic_vector(cfg_rc_qpmax)
                                                    & "00" & std_logic_vector(cfg_rc_qpmin) & "000000" & cfg_rc_map & cfg_rc_en;
                        when "001010" => rdata_q <= std_logic_vector(cfg_rc_target);
                        when "001011" => rdata_q <= std_logic_vector(cfg_rc_scale);
                        when "001100" => rdata_q <= std_logic_vector(k_wtotal);
                        when others   => rdata_q <= (others => '0');
                    end case;
                end if;
                if rvalid_q = '1' and s_axi_rready = '1' then rvalid_q <= '0'; end if;

                -- frame bookkeeping.
                --
                -- bytes_run and bytes_last are both signals, so an assignment
                -- to bytes_run here is not visible to a read of it in the same
                -- process. When the frame's final beat handshakes on the very
                -- cycle k_done arrives -- which is exactly when it is most
                -- likely to -- a plain `bytes_last <= bytes_run` captures the
                -- count from before that beat and reports one beat short.
                -- Compute the total once and use it for both.
                if counting = '1' then cyc_run <= cyc_run + 1; end if;
                -- Assign bytes_run only on a handshake. START clears it in the
                -- register-write section above, and the last assignment in a
                -- process wins, so an unconditional assignment here would
                -- silently defeat that clear and accumulate across frames.
                beat     := (k_o_valid = '1' and m_axis_tready = '1');
                lastbeat := beat and (k_o_last = '1');

                bytes_tot := bytes_run;
                if beat then
                    bytes_tot := bytes_tot + keep_count(k_o_keep);
                    bytes_run <= bytes_tot;
                end if;
                if lastbeat then drained_q <= '1'; end if;

                if k_done = '1' then
                    kdone_q      <= '1';
                    cyc_last     <= cyc_run + 1;
                    counting     <= '0';
                    frame_active <= '0';
                end if;

                -- DONE means the frame has been emitted, not merely that the
                -- kernel stopped. Under input backpressure k_done arrives
                -- before the final beat has handshaken, so capturing here
                -- rather than at k_done is what makes BYTES right.
                if done_flag = '0'
                   and (kdone_q = '1' or k_done = '1')
                   and (drained_q = '1' or lastbeat) then
                    done_flag  <= '1';
                    frames     <= frames + 1;
                    bytes_last <= bytes_tot;
                    kdone_q    <= '0';
                    drained_q  <= '0';
                end if;
            end if;
        end if;
    end process;

    irq <= done_flag and irq_en;

end architecture;
