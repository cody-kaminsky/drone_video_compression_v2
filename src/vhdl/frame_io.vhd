--------------------------------------------------------------------------------
-- frame_io.vhd
--
-- Frame-side I/O for the encoder kernel:
--
--   * pixel input: a 32-bit AXI-Stream carrying one MB row at a time in
--     raster order -- 16 luma lines, then the 8 NV12 (U/V interleaved)
--     chroma lines of the same MB row, 4 samples per beat. That is what a
--     host DMA produces with two transfers per MB row (one per plane).
--     The block re-tiles the row into 4x4 blocks: a 128-bit RAM word holds
--     4 lines x 4 bytes, written with byte enables as the beats arrive, so
--     every 4x4 block is one read. Two MB-row banks (ping-pong) let the
--     next row stream in while the current one is being encoded.
--   * MB source output: the 24-word stream mb_pipeline_controller expects
--     per MB (Y blocks 0..15 raster, U 0..3, V 0..3), MBs left to right,
--     rows top to bottom.
--   * bitstream output: the controller's byte stream packed into a 32-bit
--     AXI-Stream (byte 0 in bits 7:0) with TKEEP on the final partial word
--     and TLAST from the controller's last byte.
--
-- Storage: 2 banks x (4 luma + 2 chroma) line-groups x (MAX_W/4) words x
-- 128 bits; 90 KB at MAX_W = 1920, block RAM (separate luma and chroma
-- RAMs so their depths, 3840 and 1920, fill block RAMs without padding).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity frame_io is
    generic (
        MAX_W : positive := 1920      -- luma width in samples, multiple of 16
    );
    port (
        clk           : in  std_logic;
        rst_n         : in  std_logic;
        -- configuration
        frame_start_i : in  std_logic;
        mbs_w_i       : in  unsigned(7 downto 0);
        -- pixel input (AXI-Stream, 4 samples per beat)
        s_valid_i     : in  std_logic;
        s_ready_o     : out std_logic;
        s_data_i      : in  std_logic_vector(31 downto 0);
        -- MB source stream to the pipeline controller
        m_valid_o     : out std_logic;
        m_ready_i     : in  std_logic;
        m_data_o      : out std_logic_vector(127 downto 0);
        -- slice payload bytes from the pipeline controller
        b_valid_i     : in  std_logic;
        b_ready_o     : out std_logic;
        b_data_i      : in  unsigned(7 downto 0);
        b_last_i      : in  std_logic;
        -- packed bitstream output (AXI-Stream, 32-bit)
        o_valid_o     : out std_logic;
        o_ready_i     : in  std_logic;
        o_data_o      : out std_logic_vector(31 downto 0);
        o_keep_o      : out std_logic_vector(3 downto 0);
        o_last_o      : out std_logic
    );
end entity;

architecture rtl of frame_io is

    constant WPL    : integer := MAX_W / 4;         -- words per line-group
    constant YBANK  : integer := 4 * WPL;           -- luma words per bank
    constant CBANK  : integer := 2 * WPL;           -- chroma words per bank
    constant YDEPTH : integer := 2 * YBANK;
    constant CDEPTH : integer := 2 * CBANK;

    subtype word_t is std_logic_vector(127 downto 0);
    -- separate luma / chroma RAMs, and one 32-bit RAM per line-of-group
    -- lane: a beat writes exactly one lane (constant slice per RAM), a
    -- block read collects the four lanes at one address
    subtype lane_t is std_logic_vector(31 downto 0);
    type ylane_t is array (0 to YDEPTH - 1) of lane_t;
    type clane_t is array (0 to CDEPTH - 1) of lane_t;
    type ymem_t is array (0 to 3) of ylane_t;
    type cmem_t is array (0 to 3) of clane_t;
    signal ymem : ymem_t;
    signal cmem : cmem_t;
    attribute ram_style : string;
    attribute ram_style of ymem : signal is "block";
    attribute ram_style of cmem : signal is "block";
    signal yq, cq : word_t;

    -- write side
    signal wr_bank : std_logic := '0';
    signal wr_line : integer range 0 to 23 := 0;
    signal wr_xq   : integer range 0 to WPL - 1 := 0;
    signal bank_full : std_logic_vector(1 downto 0) := "00";
    signal wr_addr : integer range 0 to YDEPTH - 1 := 0;   -- registered, tracks (bank, group, xq)
    signal grp_base : integer range 0 to YDEPTH - 1 := 0;
    signal wr_is_c : std_logic;
    signal wr_lane : integer range 0 to 3;
    signal s_fire  : std_logic;
    signal xq_max  : integer range 0 to WPL - 1 := 0;

    -- read side
    signal rd_addr : integer range 0 to YDEPTH - 1 := 0;
    signal rd_is_c : std_logic := '0';
    signal rd_q    : word_t;
    signal rd_bank : std_logic := '0';
    signal rd_mb   : unsigned(7 downto 0) := (others => '0');
    signal rd_item : integer range 0 to 24 := 0;     -- 0..15 Y, 16..19 U quadrants, 20..23 V from stash
    type rs_t is (R_IDLE, R_ADDR, R_WAIT, R_ADDR2, R_WAIT2, R_OUT, R_STASH);
    signal rs : rs_t := R_IDLE;
    type stash_t is array (0 to 3) of word_t;
    signal vstash : stash_t := (others => (others => '0'));
    signal out_q   : word_t := (others => '0');
    signal out_v   : std_logic := '0';
    signal wa_q    : word_t := (others => '0');
    signal mbs_w   : unsigned(7 downto 0) := (others => '0');

    -- byte packer
    signal pk_data : std_logic_vector(31 downto 0) := (others => '0');
    signal pk_n    : integer range 0 to 4 := 0;
    signal pk_last : std_logic := '0';
    signal pk_v    : std_logic := '0';

    function group_of(line : integer) return integer is
    begin
        if line < 16 then return line / 4; else return 4 + (line - 16) / 4; end if;
    end function;

begin

    ------------------------------------------------------------------
    -- Input: write 4 bytes into lane (line mod 4) of word (group, xq)
    ------------------------------------------------------------------
    -- (not on the frame_start edge: the counters reset there)
    s_ready_o <= '1' when (frame_start_i = '0' and bank_full(to_integer(unsigned'("" & wr_bank))) = '0') else '0';
    s_fire    <= s_valid_i and s_ready_o;
    wr_is_c   <= '1' when wr_line >= 16 else '0';
    wr_lane   <= wr_line mod 4;

    gen_lanes : for k in 0 to 3 generate
        ymem_wr : process(clk)
        begin
            if rising_edge(clk) then
                if s_fire = '1' and wr_is_c = '0' and wr_lane = k then
                    ymem(k)(wr_addr) <= s_data_i;
                end if;
                yq(32 * k + 31 downto 32 * k) <= ymem(k)(rd_addr);
            end if;
        end process;

        cmem_wr : process(clk)
        begin
            if rising_edge(clk) then
                if s_fire = '1' and wr_is_c = '1' and wr_lane = k then
                    cmem(k)(wr_addr mod CDEPTH) <= s_data_i;
                end if;
                cq(32 * k + 31 downto 32 * k) <= cmem(k)(rd_addr mod CDEPTH);
            end if;
        end process;
    end generate;
    rd_q <= cq when rd_is_c = '1' else yq;

    wr_ctl : process(clk, rst_n)
    begin
        if rst_n = '0' then
            wr_bank <= '0'; wr_line <= 0; wr_xq <= 0; bank_full <= "00";
        elsif rising_edge(clk) then
            if frame_start_i = '1' then
                wr_bank <= '0'; wr_line <= 0; wr_xq <= 0; bank_full <= "00";
                wr_addr <= 0; grp_base <= 0;
                mbs_w <= mbs_w_i;
                xq_max <= to_integer(mbs_w_i) * 4 - 1;
            else
                if s_fire = '1' then
                    -- the write address walks the line; at a line end it goes
                    -- back to the group start, or to the next group (luma
                    -- groups 0..3 then chroma groups 0..1 of the same bank)
                    if wr_xq = xq_max then
                        wr_xq <= 0;
                        if wr_line = 23 then
                            wr_line <= 0;
                            bank_full(to_integer(unsigned'("" & wr_bank))) <= '1';
                            wr_bank <= not wr_bank;
                            if wr_bank = '0' then grp_base <= YBANK; wr_addr <= YBANK; else grp_base <= 0; wr_addr <= 0; end if;
                        else
                            wr_line <= wr_line + 1;
                            if wr_line = 15 then
                                -- chroma groups start at the bank's chroma base
                                if wr_bank = '0' then grp_base <= 0; wr_addr <= 0; else grp_base <= CBANK; wr_addr <= CBANK; end if;
                            elsif (wr_line mod 4) = 3 then
                                grp_base <= grp_base + WPL; wr_addr <= grp_base + WPL;
                            else
                                wr_addr <= grp_base;
                            end if;
                        end if;
                    else
                        wr_xq <= wr_xq + 1;
                        wr_addr <= wr_addr + 1;
                    end if;
                end if;
                -- release a bank when its last MB has been read out
                if rs = R_OUT and out_v = '1' and m_ready_i = '1' and rd_item = 23 and rd_mb = mbs_w - 1 then
                    bank_full(to_integer(unsigned'("" & rd_bank))) <= '0';
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Output: 24 words per MB. Luma block = one word. Chroma quadrant =
    -- two words de-interleaved into a U block (sent now) and a V block
    -- (stashed, sent after U 0..3).
    ------------------------------------------------------------------
    m_valid_o <= out_v;
    m_data_o  <= out_q;

    rd_ctl : process(clk, rst_n)
        variable q, qr, qc : integer range 0 to 3;
        variable ub, vb : word_t;
        variable a, b : word_t;
    begin
        if rst_n = '0' then
            rs <= R_IDLE; rd_bank <= '0'; rd_mb <= (others => '0'); rd_item <= 0; out_v <= '0';
        elsif rising_edge(clk) then
            if frame_start_i = '1' then
                rs <= R_IDLE; rd_bank <= '0'; rd_mb <= (others => '0'); rd_item <= 0; out_v <= '0';
            else
                case rs is
                    when R_IDLE =>
                        if bank_full(to_integer(unsigned'("" & rd_bank))) = '1' then
                            rd_item <= 0; rs <= R_ADDR;
                        end if;
                    when R_ADDR =>
                        if rd_item < 16 then
                            rd_is_c <= '0';
                            rd_addr <= to_integer(unsigned'("" & rd_bank)) * YBANK + (rd_item / 4) * WPL
                                       + to_integer(rd_mb) * 4 + (rd_item mod 4);
                            rs <= R_WAIT;
                        elsif rd_item < 20 then
                            q := rd_item - 16; qr := q / 2; qc := q mod 2;
                            rd_is_c <= '1';
                            rd_addr <= to_integer(unsigned'("" & rd_bank)) * CBANK + qr * WPL
                                       + to_integer(rd_mb) * 4 + qc * 2;
                            rs <= R_WAIT;
                        else
                            out_q <= vstash(rd_item - 20); out_v <= '1'; rs <= R_OUT;
                        end if;
                    when R_WAIT =>
                        -- rd_q valid next cycle
                        if rd_item < 16 then
                            rs <= R_OUT;
                        else
                            rd_addr <= rd_addr + 1;
                            rs <= R_ADDR2;
                        end if;
                        if rd_item < 16 then out_q <= (others => '0'); end if;
                        rs <= R_WAIT2;
                    when R_WAIT2 =>
                        if rd_item < 16 then
                            out_q <= rd_q; out_v <= '1'; rs <= R_OUT;
                        else
                            wa_q <= rd_q;          -- word A (samples 0..1 of each line)
                            rs <= R_ADDR2;
                        end if;
                    when R_ADDR2 =>
                        rs <= R_STASH;             -- word B read is in flight
                    when R_STASH =>
                        a := wa_q; b := rd_q;
                        for r in 0 to 3 loop
                            -- line r: A = U0 V0 U1 V1, B = U2 V2 U3 V3
                            ub(32 * r + 7 downto 32 * r)       := a(32 * r + 7 downto 32 * r);
                            ub(32 * r + 15 downto 32 * r + 8)  := a(32 * r + 23 downto 32 * r + 16);
                            ub(32 * r + 23 downto 32 * r + 16) := b(32 * r + 7 downto 32 * r);
                            ub(32 * r + 31 downto 32 * r + 24) := b(32 * r + 23 downto 32 * r + 16);
                            vb(32 * r + 7 downto 32 * r)       := a(32 * r + 15 downto 32 * r + 8);
                            vb(32 * r + 15 downto 32 * r + 8)  := a(32 * r + 31 downto 32 * r + 24);
                            vb(32 * r + 23 downto 32 * r + 16) := b(32 * r + 15 downto 32 * r + 8);
                            vb(32 * r + 31 downto 32 * r + 24) := b(32 * r + 31 downto 32 * r + 24);
                        end loop;
                        vstash(rd_item - 16) <= vb;
                        out_q <= ub; out_v <= '1'; rs <= R_OUT;
                    when R_OUT =>
                        if m_ready_i = '1' then
                            out_v <= '0';
                            if rd_item = 23 then
                                rd_item <= 0;
                                if rd_mb = mbs_w - 1 then
                                    rd_mb <= (others => '0'); rd_bank <= not rd_bank; rs <= R_IDLE;
                                else
                                    rd_mb <= rd_mb + 1; rs <= R_ADDR;
                                end if;
                            else
                                rd_item <= rd_item + 1; rs <= R_ADDR;
                            end if;
                        end if;
                end case;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Byte -> 32-bit packer (byte 0 in bits 7:0)
    ------------------------------------------------------------------
    b_ready_o <= '1' when (pk_v = '0' or o_ready_i = '1') and not (pk_n = 4 and pk_v = '1' and o_ready_i = '0') else '0';
    o_valid_o <= pk_v;
    o_data_o  <= pk_data;
    o_last_o  <= pk_last;
    o_keep_o  <= "1111" when pk_n = 4 else "0111" when pk_n = 3 else "0011" when pk_n = 2 else "0001";

    pack_p : process(clk, rst_n)
        variable n : integer range 0 to 4;
        variable d : std_logic_vector(31 downto 0);
    begin
        if rst_n = '0' then
            pk_v <= '0'; pk_n <= 0; pk_last <= '0'; pk_data <= (others => '0');
        elsif rising_edge(clk) then
            n := pk_n; d := pk_data;
            if pk_v = '1' and o_ready_i = '1' then
                pk_v <= '0'; n := 0; d := (others => '0');
            end if;
            if b_valid_i = '1' and b_ready_o = '1' then
                d(8 * n + 7 downto 8 * n) := std_logic_vector(b_data_i);
                n := n + 1;
                if n = 4 or b_last_i = '1' then
                    pk_v <= '1'; pk_last <= b_last_i;
                end if;
            end if;
            pk_n <= n; pk_data <= d;
        end if;
    end process;

end architecture;
