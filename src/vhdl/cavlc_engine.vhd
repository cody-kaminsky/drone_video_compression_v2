--------------------------------------------------------------------------------
-- cavlc_engine.vhd
--
-- Top-level CAVLC encoder. Consumes level packets on an AXI-Stream-shaped
-- input; produces a CAVLC bitstream as a byte-stream output.
--
-- This file is the SCAFFOLD for M4 phase 1: the entity declaration and a
-- state machine outline are in place; the per-state encoders (levels,
-- total_zeros, run_before, bit packer) are TODO. See docs/m4-cavlc-vhdl.md
-- for the per-state plan and substate encoders.
--
-- Design principles:
--   - One coeff_token_encoder, one levels_encoder, one total_zeros_encoder,
--     one run_before_encoder. All shared. No replication per call site.
--   - Bit packer accumulates a 32-bit shift register and emits bytes when
--     8 or more bits have accumulated.
--   - Backpressure: the engine stalls if the output AXI-Stream isn't ready.
--   - Latency: ~3-30 cycles per block depending on TotalCoeff.
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

        ----------------------------------------------------------------
        -- Input: level packets (AXI-Stream-style handshake).
        --
        -- A single beat carries a complete level_packet_t. The HLS
        -- encoder side packs the record into AXI-Stream tdata; the
        -- shim that adapts that to this VHDL signal lives in
        -- whatever wraps the engine into the Vivado block design.
        ----------------------------------------------------------------
        in_valid  : in  std_logic;
        in_ready  : out std_logic;
        in_data   : in  level_packet_t;
        in_last   : in  std_logic;        -- end-of-slice marker

        ----------------------------------------------------------------
        -- Output: CAVLC bitstream as bytes (AXI-Stream).
        ----------------------------------------------------------------
        out_valid : out std_logic;
        out_ready : in  std_logic;
        out_data  : out unsigned(7 downto 0);
        out_last  : out std_logic         -- end-of-slice marker passes through
    );
end entity;

architecture rtl of cavlc_engine is

    --------------------------------------------------------------------
    -- State machine. See docs/m4-cavlc-vhdl.md state-machine outline.
    --------------------------------------------------------------------
    type state_t is (
        S_IDLE,         -- waiting for a level packet
        S_COUNT,        -- scan levels[] → TotalCoeff, TrailingOnes, magnitudes
        S_COEFF_TOKEN,  -- emit coeff_token VLC
        S_ONES_SIGN,    -- emit 0..3 sign bits for trailing ones
        S_LEVELS,       -- emit (TotalCoeff - TrailingOnes) level codes
        S_TOTAL_ZEROS,  -- emit total_zeros VLC (skipped if TC == n_coefs)
        S_RUN_BEFORE,   -- emit (TotalCoeff - 1) run codes
        S_DRAIN,        -- flush the bit packer at end of slice (in_last)
        S_DONE          -- single-cycle "block complete" tick → S_IDLE
    );

    signal state, state_next : state_t;

    --------------------------------------------------------------------
    -- Captured packet (held while the state machine processes it).
    --------------------------------------------------------------------
    signal pkt_q : level_packet_t;

    --------------------------------------------------------------------
    -- Per-block working registers — populated in S_COUNT.
    --------------------------------------------------------------------
    signal total_coef    : unsigned(4 downto 0);
    signal trailing_ones : unsigned(1 downto 0);
    signal total_zeros   : unsigned(4 downto 0);   -- = (last_nz_idx + 1) - TC
    -- magnitudes / signs stored alongside levels, in scan order from high to low
    -- (CAVLC encodes from the last nonzero coefficient back toward index 0).

    --------------------------------------------------------------------
    -- Sub-module ports (signals — modules instantiated below).
    --
    -- TODO M4 phase 2: implement the missing modules and wire them up.
    --------------------------------------------------------------------
    signal ct_valid_i, ct_valid_o   : std_logic;
    signal ct_code                  : unsigned(15 downto 0);
    signal ct_length                : unsigned(4 downto 0);

    -- Bit packer producer-side bus. Each substate drives this when
    -- emitting; one of them is selected at a time via the state.
    signal bp_in : bp_in_t;

begin

    ------------------------------------------------------------------
    -- Sub-module instances.
    ------------------------------------------------------------------
    coeff_token_inst : entity work.coeff_token_encoder
        port map (
            clk          => clk,
            rst_n        => rst_n,
            valid_i      => ct_valid_i,
            nC_i         => pkt_q.nC,
            total_coef_i => total_coef,
            t1_i         => trailing_ones,
            valid_o      => ct_valid_o,
            code_o       => ct_code,
            length_o     => ct_length
        );

    -- TODO: levels_encoder_inst    — emits per-level VLC with suffix tracking
    -- TODO: total_zeros_inst       — ROM lookup on (TotalCoeff, total_zeros)
    -- TODO: run_before_inst        — ROM lookup on (zeros_left_idx, run)
    -- TODO: bit_packer_inst        — 32-bit shift register + byte emit

    ------------------------------------------------------------------
    -- State register.
    ------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                state <= S_IDLE;
                pkt_q <= (
                    block_type => (others => '0'),
                    n_coefs    => (others => '0'),
                    nC         => (others => '0'),
                    levels     => (others => (others => '0'))
                );
                total_coef    <= (others => '0');
                trailing_ones <= (others => '0');
                total_zeros   <= (others => '0');
            else
                state <= state_next;

                if state = S_IDLE and in_valid = '1' then
                    pkt_q <= in_data;
                end if;

                -- TODO M4 phase 2: count TotalCoeff, TrailingOnes, etc. on
                -- entry to S_COUNT. The C reference's logic in
                -- src/cavlc.c::cavlc_encode_block (lines 380-440 area)
                -- is the model.
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Next-state combinational logic.
    --
    -- Sketch only — the substate transitions are TODO for phase 2.
    ------------------------------------------------------------------
    process(state, in_valid, in_last, ct_valid_o)
    begin
        state_next <= state;

        case state is
            when S_IDLE =>
                if in_valid = '1' then
                    if in_last = '1' then
                        state_next <= S_DRAIN;
                    else
                        state_next <= S_COUNT;
                    end if;
                end if;

            when S_COUNT =>
                state_next <= S_COEFF_TOKEN;

            when S_COEFF_TOKEN =>
                if ct_valid_o = '1' then
                    state_next <= S_ONES_SIGN;
                end if;

            -- TODO M4 phase 2: wire the rest of the state transitions.
            when S_ONES_SIGN    => state_next <= S_LEVELS;
            when S_LEVELS       => state_next <= S_TOTAL_ZEROS;
            when S_TOTAL_ZEROS  => state_next <= S_RUN_BEFORE;
            when S_RUN_BEFORE   => state_next <= S_DONE;

            when S_DRAIN =>
                state_next <= S_DONE;

            when S_DONE =>
                state_next <= S_IDLE;
        end case;
    end process;

    ------------------------------------------------------------------
    -- Handshake outputs (placeholder).
    ------------------------------------------------------------------
    in_ready  <= '1' when state = S_IDLE else '0';
    out_valid <= '0';                           -- TODO: drive from bit packer
    out_data  <= (others => '0');
    out_last  <= '0';

    -- Drive coeff_token request when entering S_COEFF_TOKEN.
    ct_valid_i <= '1' when state = S_COEFF_TOKEN else '0';

    -- TODO: bp_in driven by whichever substate is active.
    bp_in <= BP_IN_IDLE;

end architecture;
