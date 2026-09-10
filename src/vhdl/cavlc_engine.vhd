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
--   - Backpressure: stalls when bit_packer's ready_o deasserts. Every push
--     is held until accepted (AXI-style), so a field is never dropped.
--
-- Area notes:
--   - The nonzero levels are NOT compacted into a separate array. The
--     engine walks the 16-bit nonzero mask from the highest position down
--     with a position pointer and a leading-one detector, and reads each
--     level straight out of the captured packet. That removes the 16x16
--     one-hot compaction muxes and their 320 flops.
--   - All variable-length fields have a VALUE that fits in 16 bits (the
--     leading zeros of unary prefixes are implied by the length), so the
--     bit packer is instantiated with a 16-bit data path.
--   - Level arithmetic and the nonzero / ±1 detection are sized for the
--     spec range |level| <= 2048: levels are treated as sign-extended
--     13-bit values (bits 15..13 of level_t must equal bit 12).
--   - level_code is formed without a negate: for L < 0, 2|L|-1 = 2*(~L)+1.
--     The suffix_length threshold |L| > 3<<(sl-1) is tested as
--     level_code >= 3<<sl on the unbiased code, so |L| is never computed.
--   - coeff_token and total_zeros tables live in one block RAM
--     (cavlc_vlc_rom) behind a state-muxed address. The total_zeros entry
--     is fetched during the level states, so the lookup costs no cycles.
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
        out_last  : out std_logic;
        -- Bits in the current flush group (see bit_packer.block_bits_o) and
        -- the packer's flush-complete pulse (one cycle after the group's
        -- last byte was taken). Together they let a merger find a block's
        -- boundary and trim its zero-padded last byte.
        block_bits_o : out unsigned(15 downto 0);
        flushed_o    : out std_logic
    );
end entity;

architecture rtl of cavlc_engine is

    --------------------------------------------------------------------
    -- State machine
    --------------------------------------------------------------------
    type state_t is (
        S_IDLE,
        S_COUNT,        -- stage 1: per-position nonzero / ±1 flags
        S_COUNT2,       -- stage 2: TotalCoeff, TrailingOnes, total_zeros
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
    -- Per-block working registers (populated in S_COUNT / S_COUNT2)
    --------------------------------------------------------------------
    signal total_coef    : integer range 0 to 16;
    signal trailing_ones : integer range 0 to 3;
    signal total_zeros   : integer range 0 to 15;
    signal last_nz       : integer range 0 to 15;

    subtype mask_t is std_logic_vector(15 downto 0);
    signal nz_mask  : mask_t;   -- level(i) /= 0 and i < n_coefs
    signal one_mask : mask_t;   -- level(i) = ±1

    -- Walk pointer: position of the nonzero currently being processed.
    -- Levels and run_before are both emitted from the highest-frequency
    -- nonzero downward, so one pointer serves both loops.
    signal pos : integer range 0 to 15;
    -- level at pos, captured when pos is set so the 16:1 level mux is off
    -- the level-code path
    signal cur_level : signed(12 downto 0);

    -- Loop counter (nonzeros processed so far / remaining)
    signal idx : integer range 0 to 15;

    -- Suffix length for level encoding (spec 9.2.2)
    signal suffix_length : integer range 0 to 6;
    signal first_non_t1  : std_logic;

    -- coeff_token result seen (code/length registers hold after the
    -- one-cycle valid_o pulse, so we only need to remember the pulse).
    signal ct_seen : std_logic;

    --------------------------------------------------------------------
    -- Coeff_token encoder ports
    --------------------------------------------------------------------
    signal ct_valid_i, ct_valid_o : std_logic;
    signal ct_code   : unsigned(15 downto 0);
    signal ct_length : unsigned(4 downto 0);

    --------------------------------------------------------------------
    -- VLC ROM. One read port, address muxed by state: the coeff_token
    -- address while that code is being looked up / waited for, the
    -- total_zeros address (from the per-block registers) otherwise.
    --------------------------------------------------------------------
    signal rom_addr_ct, rom_addr_tz, rom_addr : unsigned(8 downto 0);
    signal rom_data  : vlc_entry_t;
    signal tz_chroma : std_logic;

    --------------------------------------------------------------------
    -- Bit packer ports (16-bit data path, see header)
    --------------------------------------------------------------------
    constant BP_W : positive := 16;
    signal bp_bits    : unsigned(BP_W - 1 downto 0);
    signal bp_length  : unsigned(5 downto 0);
    signal bp_valid   : std_logic;
    signal bp_ready   : std_logic;
    signal bp_flush   : std_logic;
    signal flush_sent : std_logic;
    signal bp_flushed : std_logic;

    signal bp_out_data  : unsigned(7 downto 0);
    signal bp_out_valid : std_logic;
    signal bp_out_last  : std_logic;

    --------------------------------------------------------------------
    -- Mask helpers
    --------------------------------------------------------------------
    -- Index of the highest set bit (0 if none; callers check m /= 0).
    function highest_set(m : mask_t) return integer is
        variable r : integer range 0 to 15 := 0;
    begin
        for i in 0 to 15 loop
            if m(i) = '1' then
                r := i;
            end if;
        end loop;
        return r;
    end function;

    -- Bits strictly below position p.
    function below_mask(p : integer range 0 to 15) return mask_t is
        variable r : mask_t;
    begin
        for i in 0 to 15 loop
            if i < p then
                r(i) := '1';
            else
                r(i) := '0';
            end if;
        end loop;
        return r;
    end function;

    -- Bits strictly above position p.
    function above_mask(p : integer range 0 to 15) return mask_t is
        variable r : mask_t;
    begin
        for i in 0 to 15 loop
            if i > p then
                r(i) := '1';
            else
                r(i) := '0';
            end if;
        end loop;
        return r;
    end function;

    function popcount(m : mask_t) return integer is
        variable c : integer range 0 to 16 := 0;
    begin
        for i in 0 to 15 loop
            if m(i) = '1' then
                c := c + 1;
            end if;
        end loop;
        return c;
    end function;

    function is_zero(m : mask_t) return boolean is
    begin
        return m = (m'range => '0');
    end function;

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
            rom_addr_o   => rom_addr_ct,
            rom_data_i   => rom_data,
            valid_o      => ct_valid_o,
            code_o       => ct_code,
            length_o     => ct_length
        );

    ------------------------------------------------------------------
    -- Sub-module: VLC ROM (block RAM)
    ------------------------------------------------------------------
    vlc_rom_inst : entity work.cavlc_vlc_rom
        port map (
            clk  => clk,
            addr => rom_addr,
            data => rom_data
        );

    -- The coeff_token result is consumed in S_EMIT_CT; S_TOTAL_ZEROS is
    -- reached at least three cycles later (S_ONES_SIGN exit, one level,
    -- S_LEVELS exit), so the total_zeros entry is ready when needed.
    rom_addr <= rom_addr_ct when (state = S_COEFF_TOKEN or state = S_EMIT_CT)
                else rom_addr_tz;

    -- total_zeros address: valid from S_COUNT2 onward (total_coef >= 1
    -- whenever S_TOTAL_ZEROS is reached; total_zeros is only decremented
    -- later, in S_RUN_BEFORE).
    -- Address map: see cavlc_vlc_rom. Luma: 256 + (TC-1)*16 + tz.
    -- Chroma DC: 496 + (TC-1)*4 + tz (TC <= 3, tz <= 3).
    tz_chroma   <= '1' when pkt_q.nC = to_unsigned(31, 5) else '0';
    rom_addr_tz <= (others => '0') when total_coef = 0 else
                  "11111" & to_unsigned(total_coef - 1, 5)(1 downto 0) &
                            to_unsigned(total_zeros, 4)(1 downto 0)
                      when tz_chroma = '1' else
                  '1' & to_unsigned(total_coef - 1, 5)(3 downto 0) &
                        to_unsigned(total_zeros, 4);

    ------------------------------------------------------------------
    -- Sub-module: bit_packer
    ------------------------------------------------------------------
    bit_packer_inst : entity work.bit_packer
        generic map (
            DATA_W => BP_W
        )
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
            out_last  => bp_out_last,
            block_bits_o => block_bits_o
        );

    out_data  <= bp_out_data;
    out_valid <= bp_out_valid;
    out_last  <= bp_out_last;
    flushed_o <= bp_flushed;

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
        variable v_last_nz       : integer range 0 to 15;
        variable v_big           : mask_t;
        variable v_above         : mask_t;
        variable v_next_mask     : mask_t;
        variable v_next_pos      : integer range 0 to 15;
        variable v_level         : signed(12 downto 0);
        variable v_lc_raw        : integer range 0 to 4095;  -- unbiased level_code
        variable v_level_code    : integer range 0 to 4095;
        variable v_level_prefix  : integer range 0 to 4095;
        variable v_suffix_val    : integer range 0 to 63;
        variable v_esc_off       : integer range 0 to 960;
        variable v_emit_len      : integer range 0 to 28;
        variable v_emit_bits     : unsigned(BP_W - 1 downto 0);
        variable v_new_sl        : integer range 0 to 6;
        variable v_run           : integer range 0 to 15;
        variable v_zl_idx        : integer range 0 to 6;
        variable v_vlc           : vlc_entry_t;
        -- True when a new field may be presented to the bit packer this
        -- cycle: either nothing is pending, or the pending field is being
        -- accepted on this edge. bp_valid/bp_bits/bp_length are held
        -- until bp_ready is seen (AXI-style), so a push is never dropped
        -- when the packer's ready falls after a long (up to 28-bit) field.
        variable v_can_push      : boolean;
        variable v_pos_new  : integer range 0 to 15;
        variable v_pos_upd  : boolean;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                state        <= S_IDLE;
                pkt_last     <= '0';
                total_coef   <= 0;
                trailing_ones <= 0;
                total_zeros  <= 0;
                last_nz      <= 0;
                nz_mask      <= (others => '0');
                one_mask     <= (others => '0');
                pos          <= 0;
                idx          <= 0;
                suffix_length <= 0;
                first_non_t1 <= '0';
                ct_seen      <= '0';
                ct_valid_i   <= '0';
                bp_bits      <= (others => '0');
                bp_length    <= (others => '0');
                bp_valid     <= '0';
                bp_flush     <= '0';
                flush_sent   <= '0';
            else
                -- Defaults: deassert one-shot signals
                ct_valid_i <= '0';
                bp_flush   <= '0';

                -- bp_valid is held until the packer accepts the field.
                v_pos_upd  := false;
                v_pos_new  := 0;
                v_can_push := (bp_valid = '0') or (bp_ready = '1');
                if bp_ready = '1' then
                    bp_valid <= '0';
                end if;

                -- Walk: next nonzero position below the current one.
                -- (Shared by S_ONES_SIGN, S_LEVELS and S_RUN_BEFORE.)
                v_next_mask := nz_mask and below_mask(pos);
                v_next_pos  := highest_set(v_next_mask);

                case state is

                --------------------------------------------------------
                -- S_IDLE: wait for input packet
                --------------------------------------------------------
                when S_IDLE =>
                    if in_valid = '1' then
                        pkt_q    <= in_data;
                        pkt_last <= in_last;
                        state    <= S_COUNT;
                        -- synthesis translate_off
                        report "ENGINE: S_IDLE->S_COUNT at " & time'image(now) severity note;
                        -- synthesis translate_on
                    end if;

                --------------------------------------------------------
                -- S_COUNT: per-position flags. Only 16-bit compares,
                -- no cross-position dependency.
                --------------------------------------------------------
                when S_COUNT =>
                    for i in 0 to 15 loop
                        -- Levels are sign-extended 13-bit values (see header).
                        if i < to_integer(pkt_q.n_coefs) and
                           pkt_q.levels(i)(12 downto 0) /= to_signed(0, 13) then
                            nz_mask(i) <= '1';
                        else
                            nz_mask(i) <= '0';
                        end if;
                        -- ±1  <=>  bit 0 set and bits 12..1 all equal
                        -- (all-zero => +1, all-one => -1).
                        if pkt_q.levels(i)(0) = '1' and
                           (pkt_q.levels(i)(12 downto 1) xor
                            (12 downto 1 => pkt_q.levels(i)(1))) = 0 then
                            one_mask(i) <= '1';
                        else
                            one_mask(i) <= '0';
                        end if;
                    end loop;
                    state <= S_COUNT2;

                --------------------------------------------------------
                -- S_COUNT2: TotalCoeff, last nonzero, TrailingOnes,
                -- total_zeros; initialise the walk.
                --------------------------------------------------------
                when S_COUNT2 =>
                    v_total_coef := popcount(nz_mask);
                    v_last_nz    := highest_set(nz_mask);

                    -- TrailingOnes = number of nonzeros above the highest
                    -- level that is not ±1, capped at 3. If every nonzero
                    -- is ±1, all of them count (still capped at 3).
                    v_big := nz_mask and not one_mask;
                    if is_zero(v_big) then
                        v_above := nz_mask;
                    else
                        v_above := nz_mask and above_mask(highest_set(v_big));
                    end if;
                    if popcount(v_above) >= 3 then
                        v_trailing_ones := 3;
                    else
                        v_trailing_ones := popcount(v_above);
                    end if;

                    total_coef    <= v_total_coef;
                    trailing_ones <= v_trailing_ones;
                    last_nz       <= v_last_nz;
                    if v_total_coef > 0 then
                        total_zeros <= (v_last_nz + 1) - v_total_coef;
                    else
                        total_zeros <= 0;
                    end if;

                    -- Initialize suffix_length per spec
                    if v_total_coef > 10 and v_trailing_ones < 3 then
                        suffix_length <= 1;
                    else
                        suffix_length <= 0;
                    end if;
                    first_non_t1 <= '1';
                    pos <= v_last_nz;
                    v_pos_new := v_last_nz; v_pos_upd := true;
                    idx <= 0;

                    state <= S_COEFF_TOKEN;
                    -- synthesis translate_off
                    report "ENGINE: S_COUNT->S_COEFF_TOKEN TC=" &
                           integer'image(v_total_coef) &
                           " T1=" & integer'image(v_trailing_ones) &
                           " TZ=" & integer'image((v_last_nz + 1) - v_total_coef)
                           severity note;
                    -- synthesis translate_on

                --------------------------------------------------------
                -- S_COEFF_TOKEN: trigger coeff_token_encoder lookup
                --------------------------------------------------------
                when S_COEFF_TOKEN =>
                    ct_valid_i <= '1';
                    ct_seen    <= '0';
                    state <= S_EMIT_CT;

                --------------------------------------------------------
                -- S_EMIT_CT: wait for result, push to bit packer
                --------------------------------------------------------
                when S_EMIT_CT =>
                    if ct_valid_o = '1' then
                        ct_seen <= '1';
                    end if;
                    if (ct_valid_o = '1' or ct_seen = '1') and v_can_push then
                        ct_seen   <= '0';
                        bp_bits   <= resize(ct_code, BP_W);
                        bp_length <= resize(ct_length, 6);
                        bp_valid  <= '1';
                        -- synthesis translate_off
                        report "ENGINE: S_EMIT_CT pushed ct len=" &
                               integer'image(to_integer(ct_length)) &
                               " code=" & integer'image(to_integer(ct_code)) &
                               " bp_ready=" & std_logic'image(bp_ready) severity note;
                        -- synthesis translate_on
                        if total_coef = 0 then
                            -- Empty block: done
                            if pkt_last = '1' then
                                state <= S_DRAIN;
                                -- synthesis translate_off
                                report "ENGINE: TC=0 pkt_last->S_DRAIN" severity note;
                                -- synthesis translate_on
                            else
                                state <= S_DONE;
                            end if;
                        else
                            state <= S_ONES_SIGN;
                        end if;
                    end if;

                --------------------------------------------------------
                -- S_ONES_SIGN: emit trailing_ones sign bits (1 per cycle),
                -- walking down from the highest nonzero.
                --------------------------------------------------------
                when S_ONES_SIGN =>
                    if idx >= trailing_ones then
                        -- Done with signs; pos already points at the
                        -- first non-T1 level.
                        state <= S_LEVELS;
                    elsif v_can_push then
                        -- Sign bit: 1 = negative, 0 = positive
                        bp_bits    <= (others => '0');
                        bp_bits(0) <= cur_level(12);
                        bp_length  <= to_unsigned(1, 6);
                        bp_valid   <= '1';
                        idx <= idx + 1;
                        pos <= v_next_pos;
                        v_pos_new := v_next_pos; v_pos_upd := true;
                    end if;

                --------------------------------------------------------
                -- S_LEVELS: emit level codes with suffix_length tracking
                --------------------------------------------------------
                when S_LEVELS =>
                    if idx >= total_coef then
                        -- All levels emitted; restart the walk for
                        -- run_before (idx counts down from TC-1 to 1).
                        pos <= last_nz;
                        v_pos_new := last_nz; v_pos_upd := true;
                        idx <= total_coef - 1;
                        if total_coef < to_integer(pkt_q.n_coefs) then
                            state <= S_TOTAL_ZEROS;
                        else
                            state <= S_RUN_BEFORE;
                        end if;
                    elsif v_can_push then
                        v_level := cur_level;

                        -- level_code = 2*(|L|-1) + (L<0), without a negate:
                        --   L > 0 : 2L - 2
                        --   L < 0 : 2|L| - 1 = 2*(~L) + 1   (~L = -L-1 >= 0)
                        if v_level(12) = '1' then
                            v_lc_raw := to_integer(
                                unsigned(not v_level(11 downto 0)) & '1');
                        else
                            v_lc_raw := to_integer(
                                unsigned(v_level(11 downto 0)) & '0') - 2;
                        end if;

                        -- Bias first non-T1 level if T1 < 3
                        if first_non_t1 = '1' and trailing_ones < 3 then
                            v_level_code := v_lc_raw - 2;
                        else
                            v_level_code := v_lc_raw;
                        end if;
                        first_non_t1 <= '0';

                        -- Encode per spec 9.2.2.1. Only the code VALUE is
                        -- stored; leading zeros come from the length.
                        --   prefix < 15 (sl > 0)   : prefix zeros, '1', sl suffix bits
                        --   prefix < 14 (sl = 0)   : prefix zeros, '1'
                        --   prefix = 14, sl = 0    : 14 zeros, '1', 4-bit suffix
                        --   otherwise (escape)     : 15 zeros, '1', 12-bit suffix
                        v_level_prefix := v_level_code / (2**suffix_length);
                        v_suffix_val   := v_level_code mod (2**suffix_length);
                        if suffix_length = 0 then
                            v_esc_off := 30;
                        else
                            v_esc_off := 15 * (2**suffix_length);
                        end if;

                        if (suffix_length = 0 and v_level_code < 14) or
                           (suffix_length > 0 and v_level_prefix < 15) then
                            v_emit_len  := v_level_prefix + 1 + suffix_length;
                            v_emit_bits := resize(
                                shift_left(to_unsigned(1, 7), suffix_length) or
                                to_unsigned(v_suffix_val, 7), BP_W);
                        elsif suffix_length = 0 and v_level_code < 30 then
                            v_emit_len  := 19;
                            v_emit_bits := resize(
                                "1" & to_unsigned(v_level_code - 14, 4), BP_W);
                        else
                            v_emit_len  := 28;
                            v_emit_bits := resize(
                                "1" & to_unsigned(v_level_code - v_esc_off, 12),
                                BP_W);
                        end if;

                        bp_bits   <= v_emit_bits;
                        bp_length <= to_unsigned(v_emit_len, 6);
                        bp_valid  <= '1';

                        -- Update suffix_length per spec 9.2.2.1 step 6:
                        -- First promote 0→1, then check threshold against
                        -- the NEW value. Both must apply in one cycle.
                        -- |L| > 3<<(sl-1)  <=>  unbiased level_code >= 3<<sl.
                        v_new_sl := suffix_length;
                        if v_new_sl = 0 then
                            v_new_sl := 1;
                        end if;
                        if v_lc_raw >= 3 * (2**v_new_sl) and v_new_sl < 6 then
                            v_new_sl := v_new_sl + 1;
                        end if;
                        suffix_length <= v_new_sl;

                        idx <= idx + 1;
                        pos <= v_next_pos;
                        v_pos_new := v_next_pos; v_pos_upd := true;
                    end if;

                --------------------------------------------------------
                -- S_TOTAL_ZEROS: ROM lookup and emit
                --------------------------------------------------------
                when S_TOTAL_ZEROS =>
                    if v_can_push then
                        -- Entry was fetched from the ROM during the
                        -- level states; luma/chroma selected by address.
                        bp_bits   <= resize(rom_data.code, BP_W);
                        bp_length <= resize(rom_data.length, 6);
                        bp_valid  <= '1';
                        state     <= S_RUN_BEFORE;
                    end if;

                --------------------------------------------------------
                -- S_RUN_BEFORE: emit run codes from highest-freq down.
                -- idx counts from (total_coef-1) down to 1; pos walks the
                -- nonzero positions in step with it.
                --------------------------------------------------------
                when S_RUN_BEFORE =>
                    if idx < 1 or total_zeros = 0 then
                        -- Done with run_before
                        if pkt_last = '1' then
                            state <= S_DRAIN;
                        else
                            state <= S_DONE;
                        end if;
                    elsif v_can_push then
                        -- run = zeros between this nonzero and the next
                        -- lower one. zeros_left is tracked in total_zeros
                        -- (decremented each iteration).
                        v_run := pos - v_next_pos - 1;
                        v_zl_idx := total_zeros;
                        if v_zl_idx > 6 then
                            v_zl_idx := 6;
                        else
                            v_zl_idx := v_zl_idx - 1;
                        end if;
                        v_vlc := RUN_BEFORE_TAB(v_zl_idx, v_run);
                        bp_bits   <= resize(v_vlc.code, BP_W);
                        bp_length <= resize(v_vlc.length, 6);
                        bp_valid  <= '1';
                        total_zeros <= total_zeros - v_run;
                        idx <= idx - 1;
                        pos <= v_next_pos;
                        v_pos_new := v_next_pos; v_pos_upd := true;
                    end if;

                --------------------------------------------------------
                -- S_DRAIN: flush bit packer (end of slice)
                --------------------------------------------------------
                when S_DRAIN =>
                    -- Flush only once no field is pending (packer requires
                    -- flush_i with valid_i = '0'), and only once: a second
                    -- pulse after flushed_o would start a spurious empty
                    -- flush group (and a second flushed_o pulse).
                    if bp_valid = '0' and flush_sent = '0' then
                        bp_flush   <= '1';
                        flush_sent <= '1';
                    end if;
                    if bp_flushed = '1' then
                        flush_sent <= '0';
                        -- synthesis translate_off
                        report "ENGINE: S_DRAIN->S_DONE (flushed) at " &
                               time'image(now) severity note;
                        -- synthesis translate_on
                        state <= S_DONE;
                    end if;

                --------------------------------------------------------
                -- S_DONE: single tick, return to idle
                --------------------------------------------------------
                when S_DONE =>
                    -- synthesis translate_off
                    report "ENGINE: S_DONE->S_IDLE" severity note;
                    -- synthesis translate_on
                    state <= S_IDLE;

                end case;

                -- One shared level mux: capture the level of the position
                -- just selected, so the level-code path starts from a
                -- register instead of a 16:1 mux.
                if v_pos_upd then
                    cur_level <= pkt_q.levels(v_pos_new)(12 downto 0);
                end if;
            end if;
        end if;
    end process;

end architecture;
