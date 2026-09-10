--------------------------------------------------------------------------------
-- bit_packer.vhd
--
-- Variable-length bit-field accumulator. Producer side accepts up to 32-bit
-- right-aligned fields; consumer side emits bytes MSB-first to an AXI-Stream
-- output. Used by the CAVLC engine and (later) the bitstream packer.
--
-- Internal state:
--   accum       : 64-bit shift register, MSB-first. Bit 63 is the next bit
--                 to be emitted; bits 63..(64 - n_in_accum) are valid.
--   n_in_accum  : number of valid bits currently held (0..64).
--
-- Producer rules:
--   - bits_i is right-aligned in DATA_W bits; only the low length_i bits
--     matter. CALLER MUST ENSURE bits above length_i are zero. length_i
--     may exceed DATA_W (the extra high bits are implied zeros).
--   - length_i in 0..32. length_i = 0 with valid_i = '1' is a no-op handshake.
--   - ready_o = '1' iff state = NORMAL and n_in_accum <= 32. With max input
--     of 32 bits, that guarantees room (n_in_accum + 32 <= 64).
--
-- Consumer rules:
--   - When >= 8 bits are buffered and the output slot is free, the top byte
--     is presented on out_data + out_valid. Held until out_ready = '1'.
--
-- Flush:
--   - Pulse flush_i with valid_i = '0'. Engine transitions to FLUSHING:
--       * partial byte (1..7 bits) is zero-padded and emitted
--       * full bytes already buffered drain normally
--       * out_last is asserted on the LAST byte of this flush group
--       * flushed_o pulses for one cycle once the last byte is consumed
--     Engine returns to NORMAL after the pulse. flush_i with empty accum
--     pulses flushed_o the next cycle (no byte emitted, out_last not set).
--
-- No RBSP emulation prevention here; that's the bitstream-packer's job.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bit_packer is
    generic (
        -- Width of the producer data path. The field VALUE must fit in
        -- DATA_W bits; length_i may still be up to 32 (leading zeros are
        -- implied). CAVLC fields never exceed 16 significant bits.
        DATA_W : positive := 32
    );
    port (
        clk       : in  std_logic;
        rst_n     : in  std_logic;

        -- Producer side: variable-length bit field
        bits_i    : in  unsigned(DATA_W - 1 downto 0);
        length_i  : in  unsigned(5 downto 0);
        valid_i   : in  std_logic;
        ready_o   : out std_logic;

        -- End-of-stream / byte-align flush
        flush_i   : in  std_logic;
        flushed_o : out std_logic;

        -- AXI-Stream byte output
        out_data  : out unsigned(7 downto 0);
        out_valid : out std_logic;
        out_ready : in  std_logic;
        out_last  : out std_logic;
        -- Bits accepted since the last flush completed. Final (no pushes
        -- are accepted while flushing) from the moment flush_i is seen
        -- until flushed_o, so a merger can trim the zero-padded last byte
        -- of a flush group: valid bits in it = block_bits_o mod 8 (8 if 0).
        block_bits_o : out unsigned(15 downto 0)
    );
end entity;

architecture rtl of bit_packer is

    type state_t is (S_NORMAL, S_FLUSHING);
    signal state : state_t;

    signal accum      : unsigned(63 downto 0);
    signal n_in_accum : integer range 0 to 64;

    signal out_byte_q  : unsigned(7 downto 0);
    signal out_valid_q : std_logic;
    signal out_last_q  : std_logic;
    signal flushed_q   : std_logic;

    signal ready_int : std_logic;
    signal blk_bits  : unsigned(15 downto 0);

begin

    ready_int <= '1' when (state = S_NORMAL and n_in_accum <= 32) else '0';
    ready_o   <= ready_int;
    out_data  <= out_byte_q;
    out_valid <= out_valid_q;
    out_last  <= out_last_q;
    flushed_o <= flushed_q;
    block_bits_o <= blk_bits;

    process(clk)
        variable n_v       : integer range 0 to 64;
        variable accum_v   : unsigned(63 downto 0);
        variable drain     : boolean;
        variable can_emit  : boolean;
        variable did_emit  : boolean;
        variable last_byte : boolean;
        variable len_int   : integer range 0 to 32;
        variable shift_amt : integer range 0 to 64;
        variable do_accept : boolean;
        variable blk_base  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                state       <= S_NORMAL;
                accum       <= (others => '0');
                n_in_accum  <= 0;
                out_byte_q  <= (others => '0');
                out_valid_q <= '0';
                out_last_q  <= '0';
                flushed_q   <= '0';
                blk_bits    <= (others => '0');
            else
                flushed_q <= '0';
                -- Per-flush-group bit count: restart the cycle after the
                -- flush completes (a push may be accepted that same cycle).
                if flushed_q = '1' then blk_base := (others => '0'); else blk_base := blk_bits; end if;
                if valid_i = '1' and ready_int = '1' then
                    blk_bits <= blk_base + resize(length_i, 16);
                else
                    blk_bits <= blk_base;
                end if;

                accum_v := accum;
                n_v     := n_in_accum;

                ----------------------------------------------------------
                -- 1. Did downstream consume the pending byte?
                ----------------------------------------------------------
                drain := (out_valid_q = '1' and out_ready = '1');
                if drain then
                    out_valid_q <= '0';
                    out_last_q  <= '0';
                end if;

                ----------------------------------------------------------
                -- 2. Emit a new byte if the slot is free or just freed.
                ----------------------------------------------------------
                can_emit := (out_valid_q = '0') or drain;
                did_emit := false;
                last_byte := false;

                if can_emit then
                    if n_v >= 8 then
                        out_byte_q  <= accum_v(63 downto 56);
                        out_valid_q <= '1';
                        accum_v := shift_left(accum_v, 8);
                        n_v     := n_v - 8;
                        did_emit := true;
                        if (state = S_FLUSHING or flush_i = '1') and n_v = 0 then
                            last_byte := true;
                        end if;
                    elsif state = S_FLUSHING and n_v > 0 then
                        -- Partial-byte zero-pad: top n_v bits are valid,
                        -- low (8 - n_v) bits of accum_v(63..56) are already
                        -- zero (writes only ever set bits within n_v).
                        out_byte_q  <= accum_v(63 downto 56);
                        out_valid_q <= '1';
                        accum_v := (others => '0');
                        n_v     := 0;
                        did_emit := true;
                        last_byte := true;
                    end if;
                end if;

                if last_byte then
                    out_last_q <= '1';
                end if;

                ----------------------------------------------------------
                -- 3. Try to accept a producer beat.
                ----------------------------------------------------------
                do_accept := (valid_i = '1' and ready_int = '1');
                if do_accept then
                    len_int := to_integer(length_i);
                    if len_int > 0 then
                        shift_amt := 64 - n_v - len_int;
                        accum_v := accum_v or
                                   shift_left(resize(bits_i, 64), shift_amt);
                    end if;
                    n_v := n_v + len_int;
                end if;

                ----------------------------------------------------------
                -- 4. Flush state machine.
                ----------------------------------------------------------
                if state = S_NORMAL then
                    if flush_i = '1' then
                        state <= S_FLUSHING;
                    end if;
                else
                    -- We're done flushing once the accumulator is empty AND
                    -- the most recently emitted byte (if any) has been
                    -- consumed by the sink.
                    if n_v = 0 and not did_emit and out_valid_q = '0' then
                        state     <= S_NORMAL;
                        flushed_q <= '1';
                    end if;
                end if;

                accum      <= accum_v;
                n_in_accum <= n_v;
            end if;
        end if;
    end process;

end architecture;
