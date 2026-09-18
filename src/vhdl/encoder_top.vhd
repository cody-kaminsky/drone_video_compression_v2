--------------------------------------------------------------------------------
-- encoder_top.vhd
--
-- The encoder kernel: frame_io (row buffer, MB tiling, output packer) +
-- mb_pipeline_controller (line buffer, mode decision, header, CAVLC).
-- Pixels in as a 32-bit AXI-Stream (per MB row: 16 luma lines then 8
-- NV12 chroma lines), slice payload out as a 32-bit AXI-Stream. Slice
-- header, SPS/PPS and NAL framing are done by the host around this
-- payload, exactly as the C reference splits them.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity encoder_top is
    generic (
        MAX_W     : positive := 1920;
        N_ENGINES : positive := 1;
        MAX_MBS   : positive := 8192;
        DEBUG     : boolean  := false
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
        -- per-MB rate control; the defaults give one QP per frame
        rc_en_i        : in  std_logic := '0';
        rc_map_valid_i : in  std_logic := '0';
        rc_qp_min_i    : in  unsigned(5 downto 0) := (others => '0');
        rc_qp_max_i    : in  unsigned(5 downto 0) := (others => '1');
        rc_target_i    : in  unsigned(31 downto 0) := (others => '0');
        rc_scale_i     : in  unsigned(31 downto 0) := (others => '0');
        rc_lag_i       : in  unsigned(3 downto 0) := to_unsigned(2, 4);
        rc_wtotal_o    : out unsigned(31 downto 0);
        -- pixel input
        s_valid_i     : in  std_logic;
        s_ready_o     : out std_logic;
        s_data_i      : in  std_logic_vector(31 downto 0);
        -- bitstream output
        o_valid_o     : out std_logic;
        o_ready_i     : in  std_logic;
        o_data_o      : out std_logic_vector(31 downto 0);
        o_keep_o      : out std_logic_vector(3 downto 0);
        o_last_o      : out std_logic
    );
end entity;

architecture rtl of encoder_top is
    signal m_valid, m_ready : std_logic;
    signal m_data : std_logic_vector(127 downto 0);
    signal b_valid, b_ready, b_last : std_logic;
    signal b_data : unsigned(7 downto 0);
begin

    io : entity work.frame_io
        generic map (MAX_W => MAX_W)
        port map (clk => clk, rst_n => rst_n, frame_start_i => frame_start_i, mbs_w_i => mbs_w_i,
                  s_valid_i => s_valid_i, s_ready_o => s_ready_o, s_data_i => s_data_i,
                  m_valid_o => m_valid, m_ready_i => m_ready, m_data_o => m_data,
                  b_valid_i => b_valid, b_ready_o => b_ready, b_data_i => b_data, b_last_i => b_last,
                  o_valid_o => o_valid_o, o_ready_i => o_ready_i, o_data_o => o_data_o,
                  o_keep_o => o_keep_o, o_last_o => o_last_o);

    ctl : entity work.mb_pipeline_controller
        generic map (MAX_MB_COLS => MAX_W / 16, N_ENGINES => N_ENGINES, PKT_DEPTH => 32, ORDER_DEPTH => 64,
                     MAX_MBS => MAX_MBS, DEBUG => DEBUG)
        port map (clk => clk, rst_n => rst_n, frame_start_i => frame_start_i, mbs_w_i => mbs_w_i,
                  mbs_h_i => mbs_h_i, qp_i => qp_i, busy_o => busy_o, frame_done_o => frame_done_o,
                  rc_en_i => rc_en_i, rc_map_valid_i => rc_map_valid_i, rc_qp_min_i => rc_qp_min_i,
                  rc_qp_max_i => rc_qp_max_i, rc_target_i => rc_target_i, rc_scale_i => rc_scale_i,
                  rc_lag_i => rc_lag_i, rc_wtotal_o => rc_wtotal_o,
                  src_valid_i => m_valid, src_ready_o => m_ready, src_data_i => m_data,
                  out_valid => b_valid, out_ready => b_ready, out_data => b_data, out_last => b_last);

end architecture;
