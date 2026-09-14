--------------------------------------------------------------------------------
-- bit_reader.vhd
--
-- The inverse of bit_packer: bytes in, bit fields out. It is the first half
-- of a decoder's entropy path, and everything above it -- Exp-Golomb, CAVLC,
-- the macroblock parser -- is built on the two primitives here.
--
--   peek_o   the next PEEK_W bits of the stream, MSB first, always valid
--            when avail_o says so. A variable-length code is decoded by
--            looking at this window and then consuming what it turned out
--            to be worth, which is why peek and consume are separate.
--   consume_i / consume_n_i
--            retire n bits (1..32). Takes effect on the next cycle.
--
-- That split is the whole point. A packer knows the length before it writes;
-- a reader cannot know it until it has looked. Every VLC decode is therefore
-- "peek a window, match it, consume the matched length", and a reader that
-- only offered get(n) would force the consumer to guess.
--
-- Byte input is a simple valid/ready stream. The reader keeps a 64-bit
-- accumulator and refills it whenever there is room for a whole byte, so a
-- consumer that peeks and consumes every cycle never stalls as long as the
-- source keeps up.
--
-- End of stream: assert last_i with the final byte. Once the accumulator
-- drains past the real data the reader pads with zeros and raises
-- underrun_o, rather than stalling. A truncated stream then fails in the
-- parser with a syntax error, which is a far better diagnosis than a hang.
--
-- No emulation-prevention handling here. Removing 0x03 bytes is a property
-- of the NAL layer, not of bit reading, and doing it upstream keeps this
-- block usable for any byte stream.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bit_reader is
    generic (
        -- Width of the peek window. It must be at least as wide as the
        -- longest code the consumer needs to see at once. CAVLC's worst case
        -- is a 16-bit zero prefix plus a 12-bit suffix on an escape level,
        -- so 32 covers every field this decoder parses.
        PEEK_W : positive := 32
    );
    port (
        clk        : in  std_logic;
        rst_n      : in  std_logic;

        -- Byte input
        in_data    : in  unsigned(7 downto 0);
        in_valid   : in  std_logic;
        in_ready   : out std_logic;
        in_last    : in  std_logic;   -- final byte of the stream

        -- Peek / consume
        peek_o     : out unsigned(PEEK_W - 1 downto 0);
        avail_o    : out std_logic;   -- at least PEEK_W bits are held
        consume_i  : in  std_logic;
        consume_n_i: in  unsigned(5 downto 0);   -- 1..32

        -- Bits retired since reset, so a parser can report a bit offset and
        -- a caller can check it landed where the encoder said it would.
        bitpos_o   : out unsigned(31 downto 0);
        underrun_o : out std_logic    -- sticky: zeros were fabricated
    );
end entity;

architecture rtl of bit_reader is

    -- Accumulator, MSB-aligned: the next bit out is always bit 63.
    signal acc    : unsigned(63 downto 0) := (others => '0');
    signal n_bits : integer range 0 to 64 := 0;

    signal saw_last : std_logic := '0';
    signal under_q  : std_logic := '0';
    signal bitpos   : unsigned(31 downto 0) := (others => '0');

    -- Room for another byte, and nothing to wait for.
    signal can_fill : std_logic;

begin

    can_fill <= '1' when n_bits <= 56 else '0';
    in_ready <= can_fill and not saw_last;

    peek_o  <= acc(63 downto 64 - PEEK_W);
    avail_o <= '1' when (n_bits >= PEEK_W or saw_last = '1') else '0';

    bitpos_o   <= bitpos;
    underrun_o <= under_q;

    main_p : process(clk)
        variable a    : unsigned(63 downto 0);
        variable n    : integer range 0 to 64;
        variable take : integer range 0 to 32;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                acc      <= (others => '0');
                n_bits   <= 0;
                saw_last <= '0';
                under_q  <= '0';
                bitpos   <= (others => '0');
            else
                a := acc;
                n := n_bits;

                -- 1. Retire bits first, so the refill below sees the room.
                if consume_i = '1' then
                    take := to_integer(consume_n_i);
                    if take > 0 then
                        if take <= n then
                            a := shift_left(a, take);
                            n := n - take;
                        else
                            -- Past the end of the data. Shift in zeros and say
                            -- so; the parser will fail on the resulting syntax
                            -- rather than deadlock waiting for bytes that are
                            -- never coming.
                            a := (others => '0');
                            n := 0;
                            under_q <= '1';
                        end if;
                        bitpos <= bitpos + take;
                    end if;
                end if;

                -- 2. Refill. One byte per cycle is enough: the consumer can
                -- retire at most 32 bits per cycle only in bursts, and the
                -- accumulator carries 64.
                if in_valid = '1' and can_fill = '1' and saw_last = '0' then
                    a := a or shift_left(resize(in_data, 64), 56 - n);
                    n := n + 8;
                    if in_last = '1' then saw_last <= '1'; end if;
                end if;

                acc    <= a;
                n_bits <= n;
            end if;
        end if;
    end process;

end architecture;
