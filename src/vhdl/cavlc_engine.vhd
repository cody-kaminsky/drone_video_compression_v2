--------------------------------------------------------------------------------
-- cavlc_engine.vhd
--
-- Top-level CAVLC encoder. Consumes level packets on an AXI-Stream-shaped
-- input; produces a CAVLC bitstream as a byte-stream output via bit_packer.
--
-- Implements the full H.264 spec 9.2 encoding:
--   1. coeff_token (ROM lookup via coeff_token_encoder)
--   2. trailing_ones sign flags (1 bit each)
--   3. level codes (unary prefix + suffix, suffix_length tracking)
--   4. total_zeros (ROM lookup)
--   5. run_before (ROM lookup loop)
--
-- Design:
--   - One coeff_token_encoder instance (1-cycle latency, shared).
--   - Level encoding done inline in state machine (no separate entity).
--   - Bit packer instance emits bytes from variable-length fields.
--   - Backpressure: stalls when bit_packer's ready_o deasserts.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;
use work.cavlc_tables.all;

entity cavlc_engine is
    port (
        clk     : in  std_logic;
        rst_n   : in  std_logic;

        -- Input: level packets (AXI-Stream-style handshake).
        in_valid  : in  std_logic;
        in_ready  : out std_logic;
        in_data   : in  level_packet_t;
        in_last   : in  std_logic;

        -- Output: CAVLC bitstream as bytes (AXI-Stream).
        out_valid : out std_logic;
        out_ready : in  std_logic;
        out_data  : out unsigned(7 downto 0);
        out_last  : out std_logic
    );
end entity;

architecture rtl of cavlc_engine is

    --------------------------------------------------------------------
    -- State machine
    --------------------------------------------------------------------
    type state_t is (
        S_IDLE,
        S_COUNT,
        S_COEFF_TOKEN,
        S_EMIT_CT,
        S_ONES_SIGN,
        S_LEVELS,
        S_TOTAL_ZEROS,
        S_RUN_BEFORE,
        S_DRAIN,
        S_DONE
    );

    signal state : state_t;

    --------------------------------------------------------------------
    -- Captured packet
    --------------------------------------------------------------------
    signal pkt_q     : level_packet_t;
    signal pkt_last  : std_logic;

    --------------------------------------------------------------------
    -- Per-block working registers (populated in S_COUNT)
    --------------------------------------------------------------------
    signal total_coef    : integer range 0 to 16;
    signal trailing_ones : integer range 0 to 3;
    signal total_zeros   : integer range 0 to 15;
    signal last_nz       : integer range -1 to 15;

    -- Nonzero positions for run_before computation
    type nz_pos_array_t is array (0 to 15) of integer range 0 to 15;
    signal nz_pos : nz_pos_array_t;

    -- Levels stored in reverse order (from last_nz toward DC) for encoding
    type level_val_array_t is array (0 to 15) of signed(15 downto 0);
    signal level_vals : level_val_array_t;

    -- Loop counters
    signal idx : integer range 0 to 15;

    -- Suffix length for level encoding (spec 9.2.2)
    signal suffix_length : integer range 0 to 6;
    signal first_non_t1  : std_logic;

    --------------------------------------------------------------------
    -- Coeff_token encoder ports
    --------------------------------------------------------------------
    signal ct_valid_i, ct_valid_o : std_logic;
    signal ct_code   : unsigned(15 downto 0);
    signal ct_length : unsigned(4 downto 0);

    --------------------------------------------------------------------
    -- Bit packer ports
    --------------------------------------------------------------------
    signal bp_bits    : unsigned(31 downto 0);
    signal bp_length  : unsigned(5 downto 0);
    signal bp_valid   : std_logic;
    signal bp_ready   : std_logic;
    signal bp_flush   : std_logic;
    signal bp_flushed : std_logic;

    signal bp_out_data  : unsigned(7 downto 0);
    signal bp_out_valid : std_logic;
    signal bp_out_last  : std_logic;

begin

    ------------------------------------------------------------------
    -- Sub-module: coeff_token_encoder
    ------------------------------------------------------------------
    coeff_token_inst : entity work.coeff_token_encoder
        port map (
            clk          => clk,
            rst_n        => rst_n,
            valid_i      => ct_valid_i,
            nC_i         => pkt_q.nC,
            total_coef_i => to_unsigned(total_coef, 5),
            t1_i         => to_unsigned(trailing_ones, 2),
            valid_o      => ct_valid_o,
            code_o       => ct_code,
            length_o     => ct_length
        );

    ------------------------------------------------------------------
    -- Sub-module: bit_packer
    ------------------------------------------------------------------
    bit_packer_inst : entity work.bit_packer
        port map (
            clk       => clk,
            rst_n     => rst_n,
            bits_i    => bp_bits,
            length_i  => bp_length,
            valid_i   => bp_valid,
            ready_o   => bp_ready,
            flush_i   => bp_flush,
            flushed_o => bp_flushed,
            out_data  => bp_out_data,
            out_valid => bp_out_valid,
            out_ready => out_ready,
            out_last  => bp_out_last
        );

    out_data  <= bp_out_data;
    out_valid <= bp_out_valid;
    out_last  <= bp_out_last;

    ------------------------------------------------------------------
    -- Handshake
    ------------------------------------------------------------------
    in_ready <= '1' when state = S_IDLE else '0';

    ------------------------------------------------------------------
    -- Main state machine
    ------------------------------------------------------------------
    process(clk)
        variable v_total_coef    : integer range 0 to 16;
        variable v_trailing_ones : integer range 0 to 3;
        variable v_last_nz       : integer range -1 to 15;
        variable v_total_zeros   : integer range 0 to 15;
        variable v_nz_count      : integer range 0 to 16;
        variable v_t1_counting   : boolean;
        variable v_level         : signed(15 downto 0);
        variable v_abs_level     : integer range 0 to 2048;
        variable v_level_code    : integer range 0 to 65535;
        variable v_level_prefix  : integer range 0 to 65535;
        variable v_suffix_bits   : integer range 0 to 12;
        variable v_suffix_val    : integer range 0 to 4095;
        variable v_emit_len      : integer range 0 to 28;
        variable v_emit_bits     : unsigned(31 downto 0);
        variable v_new_sl        : integer range 0 to 6;
        variable v_zeros_left    : integer range 0 to 15;
        variable v_run           : integer range 0 to 15;
        variable v_zl_idx        : integer range 0 to 6;
        variable v_vlc           : vlc_entry_t;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                state        <= S_IDLE;
                pkt_last     <= '0';
                total_coef   <= 0;
                trailing_ones <= 0;
                total_zeros  <= 0;
                last_nz      <= -1;
                idx          <= 0;
                suffix_length <= 0;
                first_non_t1 <= '0';
                ct_valid_i   <= '0';
                bp_bits      <= (others => '0');
                bp_length    <= (others => '0');
                bp_valid     <= '0';
                bp_flush     <= '0';
            else
                -- Defaults: deassert one-shot signals
                ct_valid_i <= '0';
                bp_valid   <= '0';
                bp_flush   <= '0';

                case state is

                --------------------------------------------------------
                -- S_IDLE: wait for input packet
                --------------------------------------------------------
                when S_IDLE =>
                    if in_valid = '1' then
                        pkt_q    <= in_data;
                        pkt_last <= in_last;
                        state    <= S_COUNT;
                    end if;

                --------------------------------------------------------
                -- S_COUNT: scan levels to find TotalCoeff, TrailingOnes,
                -- total_zeros, nonzero positions, and level values in
                -- reverse order.
                --------------------------------------------------------
                when S_COUNT =>
                    v_total_coef    := 0;
                    v_trailing_ones := 0;
                    v_last_nz       := -1;
                    v_nz_count      := 0;
                    v_t1_counting   := true;

                    -- Find last nonzero and count total
                    for i in 0 to 15 loop
                        if i < to_integer(pkt_q.n_coefs) then
                            if pkt_q.levels(i) /= to_signed(0, 16) then
                                v_last_nz    := i;
                                v_total_coef := v_total_coef + 1;
                            end if;
                        end if;
                    end loop;

                    -- Find trailing ones (from last_nz backward, contiguous ±1)
                    v_trailing_ones := 0;
                    if v_last_nz >= 0 then
                        v_t1_counting := true;
                        for i in 15 downto 0 loop
                            if i <= v_last_nz and v_t1_counting then
                                if pkt_q.levels(i) /= to_signed(0, 16) then
                                    if v_trailing_ones < 3 and
                                       (pkt_q.levels(i) = to_signed(1, 16) or
                                        pkt_q.levels(i) = to_signed(-1, 16)) then
                                        v_trailing_ones := v_trailing_ones + 1;
                                    else
                                        v_t1_counting := false;
                                    end if;
                                end if;
                            end if;
                        end loop;
                    end if;

                    -- Build nonzero position array and level values in
                    -- reverse order (highest freq first for encoding)
                    v_nz_count := 0;
                    for i in 0 to 15 loop
                        if i <= v_last_nz then
                            if pkt_q.levels(i) /= to_signed(0, 16) then
                                nz_pos(v_nz_count) <= i;
                                v_nz_count := v_nz_count + 1;
                            end if;
                        end if;
                    end loop;

                    -- Store levels in reverse order (last_nz down to 0, nonzero only)
                    v_nz_count := 0;
                    for i in 15 downto 0 loop
                        if i <= v_last_nz then
                            if pkt_q.levels(i) /= to_signed(0, 16) then
                                level_vals(v_nz_count) <= pkt_q.levels(i);
                                v_nz_count := v_nz_count + 1;
                            end if;
                        end if;
                    end loop;

                    total_coef    <= v_total_coef;
                    trailing_ones <= v_trailing_ones;
                    last_nz       <= v_last_nz;
                    if v_last_nz >= 0 then
                        v_total_zeros := (v_last_nz + 1) - v_total_coef;
                    else
                        v_total_zeros := 0;
                    end if;
                    total_zeros <= v_total_zeros;

                    -- Initialize suffix_length per spec
                    if v_total_coef > 10 and v_trailing_ones < 3 then
                        suffix_length <= 1;
                    else
                        suffix_length <= 0;
                    end if;
                    first_non_t1 <= '1';
                    idx <= 0;

                    state <= S_COEFF_TOKEN;

                --------------------------------------------------------
                -- S_COEFF_TOKEN: trigger coeff_token_encoder lookup
                --------------------------------------------------------
                when S_COEFF_TOKEN =>
                    ct_valid_i <= '1';
                    state <= S_EMIT_CT;

                --------------------------------------------------------
                -- S_EMIT_CT: wait for result, push to bit packer
                --------------------------------------------------------
                when S_EMIT_CT =>
                    if ct_valid_o = '1' and bp_ready = '1' then
                        bp_bits   <= resize(ct_code, 32);
                        bp_length <= resize(ct_length, 6);
                        bp_valid  <= '1';
                        if total_coef = 0 then
                            -- Empty block: done
                            if pkt_last = '1' then
                                state <= S_DRAIN;
                            else
                                state <= S_DONE;
                            end if;
                        else
                            state <= S_ONES_SIGN;
                            idx   <= 0;
                        end if;
                    end if;

                --------------------------------------------------------
                -- S_ONES_SIGN: emit trailing_ones sign bits (1 per cycle)
                --------------------------------------------------------
                when S_ONES_SIGN =>
                    if idx >= trailing_ones then
                        -- Done with signs, move to levels
                        idx   <= trailing_ones;
                        state <= S_LEVELS;
                    elsif bp_ready = '1' then
                        -- level_vals(idx) is from reverse order (highest freq first)
                        -- Sign bit: 1 = negative, 0 = positive
                        if level_vals(idx) < 0 then
                            bp_bits <= to_unsigned(1, 32);
                        else
                            bp_bits <= to_unsigned(0, 32);
                        end if;
                        bp_length <= to_unsigned(1, 6);
                        bp_valid  <= '1';
                        idx <= idx + 1;
                    end if;

                --------------------------------------------------------
                -- S_LEVELS: emit level codes with suffix_length tracking
                -- idx starts at trailing_ones (skip T1s already emitted as signs)
                --------------------------------------------------------
                when S_LEVELS =>
                    if idx >= total_coef then
                        -- All levels emitted
                        if total_coef < to_integer(pkt_q.n_coefs) then
                            state <= S_TOTAL_ZEROS;
                        else
                            -- No total_zeros needed (TC == n_coefs)
                            idx   <= total_coef - 1;
                            state <= S_RUN_BEFORE;
                        end if;
                    elsif bp_ready = '1' then
                        v_level := level_vals(idx);
                        if v_level < 0 then
                            v_abs_level := to_integer(-v_level);
                        else
                            v_abs_level := to_integer(v_level);
                        end if;

                        -- level_code = 2*(abs-1) + (sign==neg)
                        v_level_code := (v_abs_level - 1) * 2;
                        if v_level < 0 then
                            v_level_code := v_level_code + 1;
                        end if;

                        -- Bias first non-T1 level if T1 < 3
                        if first_non_t1 = '1' and trailing_ones < 3 then
                            v_level_code := v_level_code - 2;
                        end if;
                        first_non_t1 <= '0';

                        -- Encode based on suffix_length
                        if suffix_length = 0 then
                            if v_level_code < 14 then
                                -- Prefix-only: level_code zeros + '1'
                                v_emit_len := v_level_code + 1;
                                v_emit_bits := to_unsigned(1, 32);
                            elsif v_level_code < 30 then
                                -- prefix=14 + 4-bit suffix
                                v_emit_len := 19;  -- 14 zeros + 1 + 4 suffix
                                v_emit_bits := resize(
                                    to_unsigned(1, 5) & to_unsigned(v_level_code - 14, 4),
                                    32);
                            else
                                -- Escape: prefix=15 + 12-bit suffix
                                v_emit_len := 28;  -- 15 zeros + 1 + 12 suffix
                                v_emit_bits := resize(
                                    to_unsigned(1, 13) & to_unsigned(v_level_code - 30, 12),
                                    32);
                            end if;
                        else
                            v_level_prefix := v_level_code / (2**suffix_length);
                            v_suffix_val   := v_level_code mod (2**suffix_length);
                            if v_level_prefix < 15 then
                                -- prefix zeros + '1' + suffix bits
                                v_emit_len := v_level_prefix + 1 + suffix_length;
                                v_emit_bits := resize(
                                    shift_left(to_unsigned(1, 28),
                                               suffix_length) or
                                    to_unsigned(v_suffix_val, 28),
                                    32);
                            else
                                -- Escape: 15 zeros + 1 + 12-bit suffix
                                v_emit_len := 28;
                                v_emit_bits := resize(
                                    to_unsigned(1, 13) &
                                    to_unsigned(v_level_code - 15 * (2**suffix_length), 12),
                                    32);
                            end if;
                        end if;

                        bp_bits   <= v_emit_bits;
                        bp_length <= to_unsigned(v_emit_len, 6);
                        bp_valid  <= '1';

                        -- Update suffix_length per spec 9.2.2.1 step 6:
                        -- First promote 0→1, then check threshold against
                        -- the NEW value. Both must apply in one cycle.
                        v_new_sl := suffix_length;
                        if v_new_sl = 0 then
                            v_new_sl := 1;
                        end if;
                        if v_abs_level > 3 * (2**(v_new_sl - 1)) and v_new_sl < 6 then
                            v_new_sl := v_new_sl + 1;
                        end if;
                        suffix_length <= v_new_sl;

                        idx <= idx + 1;
                    end if;

                --------------------------------------------------------
                -- S_TOTAL_ZEROS: ROM lookup and emit
                --------------------------------------------------------
                when S_TOTAL_ZEROS =>
                    if bp_ready = '1' then
                        if pkt_q.nC = to_unsigned(31, 5) then
                            -- Chroma DC table
                            v_vlc := TOTAL_ZEROS_CHROMA_DC(total_coef - 1, total_zeros);
                        else
                            -- 4x4 luma table
                            v_vlc := TOTAL_ZEROS_4x4(total_coef - 1, total_zeros);
                        end if;
                        bp_bits   <= resize(v_vlc.code, 32);
                        bp_length <= resize(v_vlc.length, 6);
                        bp_valid  <= '1';
                        idx       <= total_coef - 1;
                        state     <= S_RUN_BEFORE;
                    end if;

                --------------------------------------------------------
                -- S_RUN_BEFORE: emit run codes from highest-freq down
                -- idx counts from (total_coef-1) down to 1
                --------------------------------------------------------
                when S_RUN_BEFORE =>
                    if idx <= 1 or total_zeros = 0 then
                        -- Done with run_before
                        if pkt_last = '1' then
                            state <= S_DRAIN;
                        else
                            state <= S_DONE;
                        end if;
                    elsif bp_ready = '1' then
                        -- Compute zeros_left (remaining zeros to distribute)
                        -- and run for this position
                        v_run := nz_pos(idx) - nz_pos(idx - 1) - 1;
                        -- zeros_left = total_zeros minus runs already emitted
                        -- We track this via total_zeros signal (decremented each iter)
                        v_zeros_left := total_zeros;
                        v_zl_idx := total_zeros;
                        if v_zl_idx > 6 then
                            v_zl_idx := 6;
                        else
                            v_zl_idx := v_zl_idx - 1;
                        end if;
                        v_vlc := RUN_BEFORE_TAB(v_zl_idx, v_run);
                        bp_bits   <= resize(v_vlc.code, 32);
                        bp_length <= resize(v_vlc.length, 6);
                        bp_valid  <= '1';
                        total_zeros <= total_zeros - v_run;
                        idx <= idx - 1;
                    end if;

                --------------------------------------------------------
                -- S_DRAIN: flush bit packer (end of slice)
                --------------------------------------------------------
                when S_DRAIN =>
                    bp_flush <= '1';
                    if bp_flushed = '1' then
                        state <= S_DONE;
                    end if;

                --------------------------------------------------------
                -- S_DONE: single tick, return to idle
                --------------------------------------------------------
                when S_DONE =>
                    state <= S_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture;
