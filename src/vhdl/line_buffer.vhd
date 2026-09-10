--------------------------------------------------------------------------------
-- line_buffer.vhd
--
-- Frame-edge neighbour storage for intra prediction and CAVLC, the hardware
-- form of line_buffer_t in src/hls/line_buffer.[ch] (the O(width) scheme:
-- only the row above and the column to the left of the current MB are ever
-- read, so only those are kept).
--
-- Storage:
--   * two banks of bottom-row reconstructed samples per MB column, luma
--     (16 bytes) and NV12 chroma (16 bytes, U/V interleaved) -- block RAM,
--     one 128-bit word per MB column;
--   * two banks of per-4x4-block CAVLC TotalCoeff counts (luma x4, U x2,
--     V x2) and luma intra 4x4 modes (x4) for the bottom block row of
--     each MB -- block RAM, one 56-bit word per MB column;
--   * the right column of the just-finished MB (luma 16, chroma 8 U/V
--     pairs, nC luma x4 / U x2 / V x2, modes x4) in registers.
--
-- The 128-bit words cost 2 RAMB36 per sample plane at 1080p (port width
-- limit 72); halving the words to 64 bits would fit one RAMB36 per plane
-- but needs a 2-cycle commit with a data mux, about +200 LUTs. Block RAM
-- is the cheaper resource here, so the wide organisation is kept.
--
-- Banking: bank top_idx is the row above (read side), bank top_idx^1 is
-- the row being built (write side). row_start_i flips them and marks the
-- top row valid; frame_start_i clears everything. row_start_i must NOT be
-- pulsed before the first row of a frame (same contract as
-- lb_begin_mb_row).
--
-- Fetch: fetch_valid_i with the MB column returns, nb_valid_o cycles
-- later, the complete neighbour bundle for that MB, held until the next
-- fetch. Reads take three block-RAM accesses (columns c-1, c, c+1 for the
-- top-left sample, the top row and block 5's top-right samples), so a
-- fetch occupies 5 cycles. Defaults follow the C code: nC = 0 and mode =
-- DC (2) where a neighbour is unavailable, top-left samples = 128.
--
-- Commit: one cycle, always accepted (commit_ready_o is constant '1' and
-- exists so a narrower-RAM variant can share the interface). Writes the
-- MB's bottom rows into the write bank and its right columns into the left
-- registers, and marks the left neighbour valid.
--
-- Bus layouts: sample k at bits (8k+7 downto 8k); nC entry k at
-- (5k+4 downto 5k); mode entry k at (4k+3 downto 4k). nc_y_i / mode4_i are
-- raster (br*4+bc) over the MB's 16 blocks; nc_u_i / nc_v_i raster
-- (br*2+bc) over 4 blocks. rec_uv_right_i holds U(r) at byte 2r and V(r)
-- at byte 2r+1 for chroma rows r = 0..7.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity line_buffer is
    generic (
        MAX_MB_COLS : positive := 120     -- 1920/16; 240 for 4K
    );
    port (
        clk            : in  std_logic;
        rst_n          : in  std_logic;
        frame_start_i  : in  std_logic;
        row_start_i    : in  std_logic;
        mbs_w_i        : in  unsigned(7 downto 0);
        -- fetch
        fetch_valid_i  : in  std_logic;
        fetch_mb_c_i   : in  unsigned(7 downto 0);
        fetch_ready_o  : out std_logic;
        nb_valid_o     : out std_logic;
        top_y_o        : out std_logic_vector(127 downto 0);
        tr_y_o         : out std_logic_vector(31 downto 0);
        tl_y_o         : out std_logic_vector(7 downto 0);
        left_y_o       : out std_logic_vector(127 downto 0);
        top_u_o        : out std_logic_vector(63 downto 0);
        top_v_o        : out std_logic_vector(63 downto 0);
        tl_u_o         : out std_logic_vector(7 downto 0);
        tl_v_o         : out std_logic_vector(7 downto 0);
        left_u_o       : out std_logic_vector(63 downto 0);
        left_v_o       : out std_logic_vector(63 downto 0);
        nc_y_top_o     : out std_logic_vector(19 downto 0);
        nc_y_left_o    : out std_logic_vector(19 downto 0);
        nc_u_top_o     : out std_logic_vector(9 downto 0);
        nc_u_left_o    : out std_logic_vector(9 downto 0);
        nc_v_top_o     : out std_logic_vector(9 downto 0);
        nc_v_left_o    : out std_logic_vector(9 downto 0);
        mode4_top_o    : out std_logic_vector(15 downto 0);
        mode4_left_o   : out std_logic_vector(15 downto 0);
        avail_top_o    : out std_logic;
        avail_left_o   : out std_logic;
        avail_tl_o     : out std_logic;
        avail_tr_o     : out std_logic;   -- block 5 top-right (MB above-right)
        -- commit
        commit_valid_i : in  std_logic;
        commit_ready_o : out std_logic;
        commit_mb_c_i  : in  unsigned(7 downto 0);
        rec_y_bot_i    : in  std_logic_vector(127 downto 0);
        rec_uv_bot_i   : in  std_logic_vector(127 downto 0);
        rec_y_right_i  : in  std_logic_vector(127 downto 0);
        rec_uv_right_i : in  std_logic_vector(127 downto 0);
        nc_y_i         : in  std_logic_vector(79 downto 0);
        nc_u_i         : in  std_logic_vector(19 downto 0);
        nc_v_i         : in  std_logic_vector(19 downto 0);
        mode4_i        : in  std_logic_vector(63 downto 0)
    );
end entity;

architecture rtl of line_buffer is

    constant DEPTH : integer := 2 * MAX_MB_COLS;

    type px_mem_t is array (0 to DEPTH - 1) of std_logic_vector(127 downto 0);
    type nc_mem_t is array (0 to DEPTH - 1) of std_logic_vector(55 downto 0);
    signal ram_y  : px_mem_t;
    signal ram_uv : px_mem_t;
    signal ram_nc : nc_mem_t;
    attribute ram_style : string;
    attribute ram_style of ram_y  : signal is "block";
    attribute ram_style of ram_uv : signal is "block";
    attribute ram_style of ram_nc : signal is "block";

    signal raddr    : integer range 0 to DEPTH - 1 := 0;
    signal q_y, q_uv : std_logic_vector(127 downto 0);
    signal q_nc     : std_logic_vector(55 downto 0);

    signal top_idx    : std_logic := '0';
    signal top_valid  : std_logic := '0';
    signal left_valid : std_logic := '0';

    -- left registers
    signal left_y_q, left_uv_q : std_logic_vector(127 downto 0) := (others => '0');
    signal nc_y_left_q : std_logic_vector(19 downto 0) := (others => '0');
    signal nc_u_left_q, nc_v_left_q : std_logic_vector(9 downto 0) := (others => '0');
    signal mode4_left_q : std_logic_vector(15 downto 0) := (others => '0');

    type fstate_t is (S_IDLE, S_F1, S_F2, S_F3, S_F4);
    signal fstate : fstate_t := S_IDLE;
    signal mbc    : unsigned(7 downto 0) := (others => '0');

    -- output registers
    signal top_y_q   : std_logic_vector(127 downto 0) := (others => '0');
    signal top_uv_q  : std_logic_vector(127 downto 0) := (others => '0');
    signal tr_y_q    : std_logic_vector(31 downto 0)  := (others => '0');
    signal tl_y_q, tl_u_q, tl_v_q : std_logic_vector(7 downto 0) := (others => '0');
    signal nc_word_q : std_logic_vector(55 downto 0) := (others => '0');
    signal a_top, a_left, a_tl, a_tr : std_logic := '0';
    signal nb_valid_q : std_logic := '0';

    function bank_addr(bank : std_logic; col : integer) return integer is
    begin
        if bank = '1' then return MAX_MB_COLS + col; else return col; end if;
    end function;

begin

    fetch_ready_o  <= '1' when fstate = S_IDLE else '0';
    commit_ready_o <= '1';
    nb_valid_o     <= nb_valid_q;

    ------------------------------------------------------------------
    -- Block RAMs: synchronous read on raddr, one write site each.
    ------------------------------------------------------------------
    ram_rd : process(clk)
    begin
        if rising_edge(clk) then
            q_y  <= ram_y(raddr);
            q_uv <= ram_uv(raddr);
            q_nc <= ram_nc(raddr);
        end if;
    end process;

    ram_wr : process(clk)
        variable wa : integer range 0 to DEPTH - 1;
        variable w  : std_logic_vector(55 downto 0);
    begin
        if rising_edge(clk) then
            if commit_valid_i = '1' then
                wa := bank_addr(not top_idx, to_integer(commit_mb_c_i));
                ram_y(wa)  <= rec_y_bot_i;
                ram_uv(wa) <= rec_uv_bot_i;
                -- bottom block row: luma blocks 12..15, chroma blocks 2..3
                w(19 downto 0)  := nc_y_i(79 downto 60);
                w(29 downto 20) := nc_u_i(19 downto 10);
                w(39 downto 30) := nc_v_i(19 downto 10);
                w(55 downto 40) := mode4_i(63 downto 48);
                ram_nc(wa) <= w;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Control, left registers, fetch sequencer
    ------------------------------------------------------------------
    ctl : process(clk, rst_n)
        variable c    : integer range 0 to 255;
        variable cp1  : integer range 0 to 255;
    begin
        if rst_n = '0' then
            top_idx    <= '0';
            top_valid  <= '0';
            left_valid <= '0';
            fstate     <= S_IDLE;
            nb_valid_q <= '0';
        elsif rising_edge(clk) then
            nb_valid_q <= '0';

            if frame_start_i = '1' then
                top_idx    <= '0';
                top_valid  <= '0';
                left_valid <= '0';
            elsif row_start_i = '1' then
                top_idx    <= not top_idx;
                top_valid  <= '1';
                left_valid <= '0';
            end if;

            if commit_valid_i = '1' then
                left_y_q  <= rec_y_right_i;
                left_uv_q <= rec_uv_right_i;
                -- right block column: luma blocks 3,7,11,15; chroma blocks 1,3
                nc_y_left_q <= nc_y_i(79 downto 75) & nc_y_i(59 downto 55) &
                               nc_y_i(39 downto 35) & nc_y_i(19 downto 15);
                nc_u_left_q <= nc_u_i(19 downto 15) & nc_u_i(9 downto 5);
                nc_v_left_q <= nc_v_i(19 downto 15) & nc_v_i(9 downto 5);
                mode4_left_q <= mode4_i(63 downto 60) & mode4_i(47 downto 44) &
                                mode4_i(31 downto 28) & mode4_i(15 downto 12);
                left_valid <= '1';
            end if;

            c := to_integer(mbc);
            if c < MAX_MB_COLS - 1 then cp1 := c + 1; else cp1 := c; end if;

            case fstate is
                when S_IDLE =>
                    if fetch_valid_i = '1' then
                        mbc <= fetch_mb_c_i;
                        if fetch_mb_c_i > 0 then
                            raddr <= bank_addr(top_idx, to_integer(fetch_mb_c_i) - 1);
                        else
                            raddr <= bank_addr(top_idx, 0);
                        end if;
                        fstate <= S_F1;
                    end if;
                when S_F1 =>
                    raddr  <= bank_addr(top_idx, c);
                    fstate <= S_F2;
                when S_F2 =>
                    -- q_* = column c-1: top-left samples
                    tl_y_q <= q_y(127 downto 120);
                    tl_u_q <= q_uv(119 downto 112);
                    tl_v_q <= q_uv(127 downto 120);
                    raddr  <= bank_addr(top_idx, cp1);
                    fstate <= S_F3;
                when S_F3 =>
                    -- q_* = column c: top rows, nC / modes
                    top_y_q   <= q_y;
                    top_uv_q  <= q_uv;
                    nc_word_q <= q_nc;
                    fstate    <= S_F4;
                when S_F4 =>
                    -- q_y = column c+1: block 5 top-right
                    tr_y_q <= q_y(31 downto 0);
                    a_top  <= top_valid;
                    if c > 0 and left_valid = '1' then a_left <= '1'; else a_left <= '0'; end if;
                    if top_valid = '1' and c > 0 and left_valid = '1' then a_tl <= '1'; else a_tl <= '0'; end if;
                    if top_valid = '1' and mbc < mbs_w_i - 1 then a_tr <= '1'; else a_tr <= '0'; end if;
                    nb_valid_q <= '1';
                    fstate <= S_IDLE;
            end case;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Output formatting with the C defaults
    ------------------------------------------------------------------
    top_y_o  <= top_y_q;
    tr_y_o   <= tr_y_q;
    left_y_o <= left_y_q;
    gen_c : for k in 0 to 7 generate
        top_u_o(8*k+7 downto 8*k)  <= top_uv_q(16*k+7 downto 16*k);
        top_v_o(8*k+7 downto 8*k)  <= top_uv_q(16*k+15 downto 16*k+8);
        left_u_o(8*k+7 downto 8*k) <= left_uv_q(16*k+7 downto 16*k);
        left_v_o(8*k+7 downto 8*k) <= left_uv_q(16*k+15 downto 16*k+8);
    end generate;
    tl_y_o <= tl_y_q when a_tl = '1' else x"80";
    tl_u_o <= tl_u_q when a_tl = '1' else x"80";
    tl_v_o <= tl_v_q when a_tl = '1' else x"80";

    nc_y_top_o  <= nc_word_q(19 downto 0)  when a_top = '1' else (others => '0');
    nc_u_top_o  <= nc_word_q(29 downto 20) when a_top = '1' else (others => '0');
    nc_v_top_o  <= nc_word_q(39 downto 30) when a_top = '1' else (others => '0');
    mode4_top_o <= nc_word_q(55 downto 40) when a_top = '1' else x"2222";
    nc_y_left_o  <= nc_y_left_q  when a_left = '1' else (others => '0');
    nc_u_left_o  <= nc_u_left_q  when a_left = '1' else (others => '0');
    nc_v_left_o  <= nc_v_left_q  when a_left = '1' else (others => '0');
    mode4_left_o <= mode4_left_q when a_left = '1' else x"2222";

    avail_top_o  <= a_top;
    avail_left_o <= a_left;
    avail_tl_o   <= a_tl;
    avail_tr_o   <= a_tr;

end architecture;
