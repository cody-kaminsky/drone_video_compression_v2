--------------------------------------------------------------------------------
-- cavlc_dispatch.vhd
--
-- Coefficient FIFO + CAVLC dispatch + in-order merge.
--
-- Decouples mode decision from entropy coding and lets N cavlc_engine
-- instances work on different residual blocks at once while the output
-- stays a single, in-order H.264 bit stream.
--
-- Input is one ordered stream of items:
--   kind 0  FIELD : a raw variable-length field (MB header syntax, the
--                   rbsp stop bit, ...) -- goes straight to the output
--                   packer when its turn comes. Only the low 8 bits of a
--                   field carry a value (Exp-Golomb codes up to ue(254));
--                   the bit_packer implies the leading zeros from in_flen.
--   kind 1  BLOCK : a level packet -- queued for engine (round-robin).
--   kind 2  FLUSH : byte-align and terminate the stream (end of slice);
--                   flushed_o pulses when the last byte has been taken.
--
-- Per engine there is a PKT_DEPTH-deep packet FIFO (the coefficient FIFO,
-- distributed RAM, levels stored at 13 bits); every packet is sent with
-- in_last so the engine's packer flushes after each block. An ORDER_DEPTH-
-- deep order FIFO remembers the item sequence; the merger walks it,
-- pushing fields, or copying a block's bytes from the engine that has it
-- into the output bit_packer. Because a packer emits a full byte as soon as
-- it has one, its out_last cannot mark a block whose last byte left before
-- the flush was requested; so the merger holds one byte back and pushes it
-- as a full byte when the next byte appears, or trimmed to
-- block_bits mod 8 when the engine's flushed_o says the block is complete.
-- The merged stream is bit-identical to a single engine's.
--
-- Throughput: the merger moves one byte (8 bits) per cycle; engines run at
-- their own pace. Blocks are assigned round-robin, so the output order and
-- the assignment order agree without any tagging.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

entity cavlc_dispatch is
    generic (
        N_ENGINES   : positive := 2;
        PKT_DEPTH   : positive := 8;
        ORDER_DEPTH : positive := 64
    );
    port (
        clk       : in  std_logic;
        rst_n     : in  std_logic;
        -- ordered item stream
        in_valid  : in  std_logic;
        in_ready  : out std_logic;
        in_kind   : in  unsigned(1 downto 0);
        in_fbits  : in  unsigned(7 downto 0);    -- field VALUE (leading zeros implied by in_flen)
        in_flen   : in  unsigned(5 downto 0);
        in_pkt    : in  level_packet_t;
        -- merged byte stream
        out_valid : out std_logic;
        out_ready : in  std_logic;
        out_data  : out unsigned(7 downto 0);
        out_last  : out std_logic;
        flushed_o : out std_logic
    );
end entity;

architecture rtl of cavlc_dispatch is

    -- Levels are stored at 13 bits: the engine treats them as sign-extended
    -- 13-bit values anyway (|L| <= 4095), so this is exact.
    constant LVL_W : integer := 13;
    constant PKT_W : integer := 3 + 5 + 5 + 16 * LVL_W;
    subtype pkt_slv is std_logic_vector(PKT_W - 1 downto 0);

    function pack(p : level_packet_t) return pkt_slv is
        variable r : pkt_slv;
    begin
        r(2 downto 0)   := std_logic_vector(p.block_type);
        r(7 downto 3)   := std_logic_vector(p.n_coefs);
        r(12 downto 8)  := std_logic_vector(p.nC);
        for i in 0 to 15 loop
            r(13 + LVL_W*i + LVL_W - 1 downto 13 + LVL_W*i) := std_logic_vector(p.levels(i)(LVL_W - 1 downto 0));
        end loop;
        return r;
    end function;

    function unpack(r : pkt_slv) return level_packet_t is
        variable p : level_packet_t;
    begin
        p.block_type := unsigned(r(2 downto 0));
        p.n_coefs    := unsigned(r(7 downto 3));
        p.nC         := unsigned(r(12 downto 8));
        for i in 0 to 15 loop
            p.levels(i) := resize(signed(r(13 + LVL_W*i + LVL_W - 1 downto 13 + LVL_W*i)), 16);
        end loop;
        return p;
    end function;

    -- order FIFO entry: kind(2) | engine(4) | flen(6) | fbits(8)
    constant ORD_W : integer := 20;
    subtype ord_slv is std_logic_vector(ORD_W - 1 downto 0);
    type ord_mem_t is array (0 to ORDER_DEPTH - 1) of ord_slv;
    signal ord_mem : ord_mem_t;
    attribute ram_style : string;
    attribute ram_style of ord_mem : signal is "distributed";
    signal ord_wp, ord_rp : integer range 0 to ORDER_DEPTH - 1 := 0;
    signal ord_cnt : integer range 0 to ORDER_DEPTH := 0;
    signal ord_head : ord_slv;

    -- per-engine packet FIFOs live in the engine generate (one 1-D memory
    -- each so they map to distributed RAM)
    signal pkt_full : std_logic_vector(N_ENGINES - 1 downto 0);
    signal rr : integer range 0 to N_ENGINES - 1 := 0;

    -- engine interfaces
    type pkt_arr is array (0 to N_ENGINES - 1) of level_packet_t;
    signal eng_in_data  : pkt_arr;
    signal eng_in_valid, eng_in_ready : std_logic_vector(N_ENGINES - 1 downto 0);
    signal eng_out_valid, eng_out_ready, eng_out_last : std_logic_vector(N_ENGINES - 1 downto 0);
    type byte_arr is array (0 to N_ENGINES - 1) of unsigned(7 downto 0);
    signal eng_out_data : byte_arr;
    type bits_arr is array (0 to N_ENGINES - 1) of unsigned(15 downto 0);
    signal eng_blk_bits : bits_arr;
    signal eng_flushed  : std_logic_vector(N_ENGINES - 1 downto 0);

    -- one-byte hold on the block path (see header)
    signal hold_byte  : unsigned(7 downto 0) := (others => '0');
    signal hold_valid : std_logic := '0';
    signal blk_done   : std_logic := '0';
    -- Trim length of the block's last byte, captured when the engine's
    -- flushed pulse arrives. It must be a register: block_bits_o is only
    -- valid from flush_i until flushed_o, and the packer restarts the
    -- count immediately after, so reading it combinationally on any later
    -- cycle -- which is what happens the moment op_ready is low -- returns
    -- 0 and the merger pushes 8 bits where it should push block_bits mod 8.
    signal blk_r      : unsigned(2 downto 0) := (others => '0');
    signal consume    : std_logic;

    -- output packer
    signal op_bits   : unsigned(7 downto 0);
    signal op_len    : unsigned(5 downto 0);
    signal op_valid  : std_logic;
    signal op_ready  : std_logic;
    signal op_flush  : std_logic;
    signal op_flushed: std_logic;

    -- merger
    type mstate_t is (S_POP, S_FIELD, S_BLOCK, S_FLUSH, S_FLUSH_WAIT);
    signal mstate : mstate_t := S_POP;
    signal cur_e  : integer range 0 to N_ENGINES - 1 := 0;
    signal cur_bits : unsigned(7 downto 0);
    signal cur_len  : unsigned(5 downto 0);
    signal ord_pop  : std_logic;
    signal flushed_q : std_logic := '0';

    signal in_fire  : std_logic;
    signal pkt_full_rr : std_logic;

begin

    ------------------------------------------------------------------
    -- Input side
    ------------------------------------------------------------------
    pkt_full_rr <= pkt_full(rr);
    in_ready <= '1' when (ord_cnt < ORDER_DEPTH) and (in_kind /= 1 or pkt_full_rr = '0') else '0';
    in_fire  <= in_valid and in_ready;

    ord_head <= ord_mem(ord_rp);

    ord_wr : process(clk)
        variable ent : ord_slv;
    begin
        if rising_edge(clk) then
            if in_fire = '1' then
                ent := (others => '0');
                ent(1 downto 0)   := std_logic_vector(in_kind);
                ent(5 downto 2)   := std_logic_vector(to_unsigned(rr, 4));
                ent(11 downto 6)  := std_logic_vector(in_flen);
                ent(19 downto 12) := std_logic_vector(in_fbits);
                ord_mem(ord_wp) <= ent;
            end if;
        end if;
    end process;

    fifo_p : process(clk, rst_n)
    begin
        if rst_n = '0' then
            ord_wp <= 0; ord_rp <= 0; ord_cnt <= 0;
            rr <= 0;
        elsif rising_edge(clk) then
            if in_fire = '1' then
                if ord_wp = ORDER_DEPTH - 1 then ord_wp <= 0; else ord_wp <= ord_wp + 1; end if;
            end if;
            if ord_pop = '1' then
                if ord_rp = ORDER_DEPTH - 1 then ord_rp <= 0; else ord_rp <= ord_rp + 1; end if;
            end if;
            if in_fire = '1' and ord_pop = '0' then ord_cnt <= ord_cnt + 1;
            elsif in_fire = '0' and ord_pop = '1' then ord_cnt <= ord_cnt - 1;
            end if;
            if in_fire = '1' and in_kind = 1 then
                if rr = N_ENGINES - 1 then rr <= 0; else rr <= rr + 1; end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Engines, each with its packet FIFO
    ------------------------------------------------------------------
    gen_eng : for k in 0 to N_ENGINES - 1 generate
        type pkt_mem_t is array (0 to PKT_DEPTH - 1) of pkt_slv;
        signal mem : pkt_mem_t;
        attribute ram_style of mem : signal is "distributed";
        signal wp, rp : integer range 0 to PKT_DEPTH - 1 := 0;
        signal cnt    : integer range 0 to PKT_DEPTH := 0;
        signal push, pop : std_logic;
    begin
        push <= '1' when (in_fire = '1' and in_kind = 1 and rr = k) else '0';
        pop  <= eng_in_valid(k) and eng_in_ready(k);

        mem_wr : process(clk)
        begin
            if rising_edge(clk) then
                if push = '1' then mem(wp) <= pack(in_pkt); end if;
            end if;
        end process;

        ptr_p : process(clk, rst_n)
        begin
            if rst_n = '0' then
                wp <= 0; rp <= 0; cnt <= 0;
            elsif rising_edge(clk) then
                if push = '1' then
                    if wp = PKT_DEPTH - 1 then wp <= 0; else wp <= wp + 1; end if;
                end if;
                if pop = '1' then
                    if rp = PKT_DEPTH - 1 then rp <= 0; else rp <= rp + 1; end if;
                end if;
                if push = '1' and pop = '0' then cnt <= cnt + 1;
                elsif push = '0' and pop = '1' then cnt <= cnt - 1;
                end if;
            end if;
        end process;

        pkt_full(k)     <= '1' when cnt = PKT_DEPTH else '0';
        eng_in_valid(k) <= '1' when cnt > 0 else '0';
        eng_in_data(k)  <= unpack(mem(rp));

        eng : entity work.cavlc_engine
            port map (
                clk       => clk,
                rst_n     => rst_n,
                in_valid  => eng_in_valid(k),
                in_ready  => eng_in_ready(k),
                in_data   => eng_in_data(k),
                in_last   => '1',
                out_valid => eng_out_valid(k),
                out_ready => eng_out_ready(k),
                out_data  => eng_out_data(k),
                out_last  => eng_out_last(k),
                block_bits_o => eng_blk_bits(k),
                flushed_o => eng_flushed(k)
            );
        eng_out_ready(k) <= consume when (mstate = S_BLOCK and cur_e = k) else '0';
    end generate;

    ------------------------------------------------------------------
    -- Merger
    ------------------------------------------------------------------
    merge_comb : process(all)
        variable r : integer range 0 to 7;
        variable fin : std_logic;
    begin
        op_bits  <= (others => '0');
        op_len   <= (others => '0');
        op_valid <= '0';
        ord_pop  <= '0';
        consume  <= '0';
        case mstate is
            when S_FIELD =>
                op_bits  <= cur_bits;
                op_len   <= cur_len;
                op_valid <= '1';
                if op_ready = '1' then ord_pop <= '1'; end if;
            when S_BLOCK =>
                -- a flushed pulse with nothing held cannot be this block's
                fin := blk_done or (eng_flushed(cur_e) and hold_valid);
                -- On the flushed pulse the live count is correct; from the next
                -- cycle on it has already been restarted, so use what was
                -- captured with blk_done.
                if blk_done = '1' then
                    r := to_integer(blk_r);
                else
                    r := to_integer(eng_blk_bits(cur_e)(2 downto 0));
                end if;
                if fin = '1' then
                    -- last byte of the block: only block_bits mod 8 bits are real
                    if r /= 0 then
                        op_bits <= shift_right(hold_byte, 8 - r);
                        op_len  <= to_unsigned(r, 6);
                    else
                        op_bits <= hold_byte;
                        op_len  <= to_unsigned(8, 6);
                    end if;
                    op_valid <= hold_valid;
                    if op_ready = '1' then ord_pop <= '1'; end if;
                else
                    op_bits  <= hold_byte;
                    op_len   <= to_unsigned(8, 6);
                    op_valid <= hold_valid and eng_out_valid(cur_e);
                    if eng_out_valid(cur_e) = '1' and (hold_valid = '0' or op_ready = '1') then
                        consume <= '1';
                    end if;
                end if;
            when S_FLUSH_WAIT =>
                if op_flushed = '1' then ord_pop <= '1'; end if;
            when others => null;
        end case;
    end process;

    merge_seq : process(clk, rst_n)
    begin
        if rst_n = '0' then
            mstate <= S_POP; op_flush <= '0'; flushed_q <= '0'; cur_e <= 0;
            hold_valid <= '0'; blk_done <= '0';
        elsif rising_edge(clk) then
            op_flush  <= '0';
            flushed_q <= '0';
            case mstate is
                when S_POP =>
                    if ord_cnt > 0 then
                        cur_e    <= to_integer(unsigned(ord_head(5 downto 2))) mod N_ENGINES;
                        cur_len  <= unsigned(ord_head(11 downto 6));
                        cur_bits <= unsigned(ord_head(19 downto 12));
                        case ord_head(1 downto 0) is
                            when "00"   => mstate <= S_FIELD;
                            when "01"   => mstate <= S_BLOCK;
                            when others => mstate <= S_FLUSH;
                        end case;
                    end if;
                when S_FIELD =>
                    if op_ready = '1' then mstate <= S_POP; end if;
                when S_BLOCK =>
                    if consume = '1' then
                        hold_byte  <= eng_out_data(cur_e);
                        hold_valid <= '1';
                    end if;
                    if eng_flushed(cur_e) = '1' and hold_valid = '1' then
                        blk_done <= '1';
                        blk_r    <= eng_blk_bits(cur_e)(2 downto 0);
                    end if;
                    if (blk_done = '1' or (eng_flushed(cur_e) = '1' and hold_valid = '1')) and op_ready = '1' then
                        hold_valid <= '0';
                        blk_done   <= '0';
                        mstate     <= S_POP;
                    end if;
                when S_FLUSH =>
                    if op_ready = '1' then
                        op_flush <= '1';
                        mstate   <= S_FLUSH_WAIT;
                    end if;
                when S_FLUSH_WAIT =>
                    if op_flushed = '1' then
                        flushed_q <= '1';
                        mstate    <= S_POP;
                    end if;
            end case;
        end if;
    end process;

    packer : entity work.bit_packer
        -- HOLD_LAST: this is the packer whose out_last becomes
        -- m_axis_tlast, so it must always have a byte left to mark when
        -- the frame flush arrives. The engine-side packers keep the default.
        generic map (DATA_W => 8, HOLD_LAST => true)
        port map (
            clk       => clk,
            rst_n     => rst_n,
            bits_i    => op_bits,
            length_i  => op_len,
            valid_i   => op_valid,
            ready_o   => op_ready,
            flush_i   => op_flush,
            flushed_o => op_flushed,
            out_data  => out_data,
            out_valid => out_valid,
            out_ready => out_ready,
            out_last  => out_last,
            block_bits_o => open
        );

    flushed_o <= flushed_q;

end architecture;
