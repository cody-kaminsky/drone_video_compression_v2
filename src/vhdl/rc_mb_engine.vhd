--------------------------------------------------------------------------------
-- rc_mb_engine.vhd
--
-- Per-macroblock QP for the rate control: the MB loop of
-- encode_frame_h264_ext (src/encoder.c, rc_mb == 3) in integer form.
--
-- The host runs the frame-level controller (bucket, frame model) and writes
-- per frame: qp_frame, [qp_min, qp_max], the frame target in bits, and
-- scale = target * 2^16 / w_total, where w_total is the sum of the weights
-- the kernel will use -- the previous frame's per-MB bit map, stored here
-- (16 bits per MB, saturated), or 1 per MB when map_valid_i is clear.
--
-- The kernel knows an MB's bits only once the merger has emitted it, which
-- is lag_i MBs behind the decision. So the QP of MB i is stepped on the
-- bits through MB i-1-lag:
--
--   spent  = sum of the bits of MBs 0 .. k          (k = i-1-lag)
--   expect = (sum of w[0..k] * scale) >> 16
--   if expect * 100 > target and spent > 0:
--       adj = A + B
--       A   = round(6 log2(spent / expect)) as -16 + #{j : spent * 4096 >=
--             expect * T[j]}, T[j] = round(4096 * 2^((j - 15.5) / 6))
--       B   = trunc(4 (spent - expect) / target), |B| <= 15
--   qp_target = clamp(qp_frame + adj, qp_min, qp_max)
--   qp        = qp of MB i-1, stepped one towards qp_target
--
-- MBs 0 .. lag get qp_frame. The QPs come out in MB order through a small
-- FIFO: the controller pops one per MB start (take_i), and every MB marker
-- from the merger (mb_end_i, with the bits pushed since the previous one)
-- computes and pushes the QP of MB k+1+lag. The FIFO never holds more than
-- lag+1 entries, because a marker for MB k can only arrive after MB k was
-- started.
--
-- With en_i clear every MB gets qp_frame and the stream is the one the
-- frame-QP kernel produced; the map is still written, so a host may turn
-- the correction on for the next frame with a valid map.
--
-- One 32x16 multiplier serves the weight accumulation, the log2 threshold
-- search and the division; a marker takes about 30 cycles to process, well
-- under the ~40 cycles the merger needs for the smallest MB.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity rc_mb_engine is
    generic (
        MAX_MBS : positive := 8192
    );
    port (
        clk           : in  std_logic;
        rst_n         : in  std_logic;
        -- per frame, latched on frame_start_i
        frame_start_i : in  std_logic;
        en_i          : in  std_logic;
        map_valid_i   : in  std_logic;
        qp_frame_i    : in  unsigned(5 downto 0);
        qp_min_i      : in  unsigned(5 downto 0);
        qp_max_i      : in  unsigned(5 downto 0);
        target_i      : in  unsigned(31 downto 0);
        scale_i       : in  unsigned(31 downto 0);
        lag_i         : in  unsigned(3 downto 0);
        -- bit accounting from the merger
        push_valid_i  : in  std_logic;
        push_len_i    : in  unsigned(5 downto 0);
        mb_end_i      : in  std_logic;
        -- QP of the next MB to start
        take_i        : in  std_logic;
        qp_o          : out unsigned(5 downto 0);
        qp_valid_o    : out std_logic;
        -- sum of this frame's MB bits (the next frame's w_total)
        wtotal_o      : out unsigned(31 downto 0)
    );
end entity;

architecture rtl of rc_mb_engine is

    -- round(4096 * 2^((j - 15.5) / 6)), j = 0 .. 31
    type t12_t is array (0 to 31) of unsigned(14 downto 0);
    constant T12 : t12_t := (
        to_unsigned(683, 15),   to_unsigned(767, 15),   to_unsigned(861, 15),   to_unsigned(967, 15),
        to_unsigned(1085, 15),  to_unsigned(1218, 15),  to_unsigned(1367, 15),  to_unsigned(1534, 15),
        to_unsigned(1722, 15),  to_unsigned(1933, 15),  to_unsigned(2170, 15),  to_unsigned(2435, 15),
        to_unsigned(2734, 15),  to_unsigned(3069, 15),  to_unsigned(3444, 15),  to_unsigned(3866, 15),
        to_unsigned(4340, 15),  to_unsigned(4871, 15),  to_unsigned(5468, 15),  to_unsigned(6137, 15),
        to_unsigned(6889, 15),  to_unsigned(7732, 15),  to_unsigned(8679, 15),  to_unsigned(9742, 15),
        to_unsigned(10935, 15), to_unsigned(12274, 15), to_unsigned(13777, 15), to_unsigned(15464, 15),
        to_unsigned(17358, 15), to_unsigned(19484, 15), to_unsigned(21870, 15), to_unsigned(24548, 15));

    -- frame configuration
    signal en, map_valid : std_logic := '0';
    signal qp_frame, qp_min, qp_max : unsigned(5 downto 0) := (others => '0');
    signal target, scale : unsigned(31 downto 0) := (others => '0');
    signal lag : unsigned(3 downto 0) := (others => '0');

    -- bit accounting
    signal bits_acc : unsigned(31 downto 0) := (others => '0');   -- since the last marker
    signal wtotal   : unsigned(31 downto 0) := (others => '0');
    signal done_cnt : integer range 0 to MAX_MBS := 0;            -- markers seen (write address)

    -- previous-frame bit map (simple dual port: read at done_cnt, written one
    -- cycle after the marker at the same address, from the captured value)
    type map_t is array (0 to MAX_MBS - 1) of unsigned(15 downto 0);
    signal map_mem : map_t;
    attribute ram_style : string;
    attribute ram_style of map_mem : signal is "block";
    signal map_rd_addr : integer range 0 to MAX_MBS - 1 := 0;
    signal map_q  : unsigned(15 downto 0) := (others => '0');
    signal map_we : std_logic := '0';
    signal map_wa : integer range 0 to MAX_MBS - 1 := 0;
    signal map_wd : unsigned(15 downto 0) := (others => '0');

    -- marker queue into the compute FSM: (bits, weight), 4 deep
    type mq_bits_t is array (0 to 3) of unsigned(31 downto 0);
    type mq_w_t    is array (0 to 3) of unsigned(15 downto 0);
    signal mq_bits : mq_bits_t := (others => (others => '0'));
    signal mq_w    : mq_w_t    := (others => (others => '0'));
    signal mq_wp, mq_rp : integer range 0 to 3 := 0;
    signal mq_cnt  : integer range 0 to 4 := 0;
    signal mq_pop  : std_logic;

    -- compute FSM
    type cst_t is (C_IDLE, C_MUL_W, C_ACC, C_EXPECT, C_GUARD,
                   C_A_MUL, C_A_CMP, C_B_MUL, C_B_CMP, C_TARGET, C_STEP);
    signal cst : cst_t := C_IDLE;
    signal spent  : unsigned(31 downto 0) := (others => '0');
    signal acc    : unsigned(47 downto 0) := (others => '0');
    signal expect : unsigned(31 downto 0) := (others => '0');
    signal e_neg  : std_logic := '0';
    signal m4     : unsigned(34 downto 0) := (others => '0');    -- |4 (spent - expect)|
    signal guard  : std_logic := '0';
    signal cur_bits : unsigned(31 downto 0) := (others => '0');
    signal cur_w    : unsigned(15 downto 0) := (others => '0');
    signal lo, hi, mid : integer range 0 to 32 := 0;
    signal a_cnt  : integer range 0 to 32 := 0;
    signal b_cnt  : integer range 0 to 15 := 0;
    signal qt_r   : integer range 0 to 63 := 0;     -- clamped qp_target
    signal qp_cur : unsigned(5 downto 0) := (others => '0');

    -- shared multiplier, one register stage
    signal mul_a : unsigned(31 downto 0) := (others => '0');
    signal mul_b : unsigned(15 downto 0) := (others => '0');
    signal prod  : unsigned(47 downto 0) := (others => '0');

    -- QP FIFO to the controller
    type qf_t is array (0 to 15) of unsigned(5 downto 0);
    signal qf : qf_t := (others => (others => '0'));
    signal qf_wp, qf_rp : integer range 0 to 15 := 0;
    signal qf_cnt : integer range 0 to 16 := 0;
    signal qf_push : std_logic := '0';
    signal qf_din  : unsigned(5 downto 0) := (others => '0');
    signal seed_left : integer range 0 to 16 := 0;    -- qp_frame entries still to push

    function sat16(v : unsigned(31 downto 0)) return unsigned is
    begin
        if v(31 downto 16) /= 0 then return x"FFFF"; else return v(15 downto 0); end if;
    end function;

begin

    wtotal_o <= wtotal;

    ------------------------------------------------------------------
    -- Bit accounting and the map
    ------------------------------------------------------------------
    map_rd_addr <= done_cnt when done_cnt < MAX_MBS else 0;

    map_p : process(clk)
    begin
        if rising_edge(clk) then
            map_q <= map_mem(map_rd_addr);
            if map_we = '1' then map_mem(map_wa) <= map_wd; end if;
        end if;
    end process;

    acct_p : process(clk, rst_n)
        variable cur : unsigned(31 downto 0);
    begin
        if rst_n = '0' then
            bits_acc <= (others => '0'); wtotal <= (others => '0'); done_cnt <= 0;
            map_we <= '0'; mq_wp <= 0; mq_cnt <= 0;
        elsif rising_edge(clk) then
            map_we <= '0';
            if frame_start_i = '1' then
                bits_acc <= (others => '0'); wtotal <= (others => '0'); done_cnt <= 0;
                mq_wp <= 0; mq_cnt <= 0;
            else
                if mb_end_i = '1' then
                    cur := bits_acc;
                    if push_valid_i = '1' then bits_acc <= resize(push_len_i, 32); else bits_acc <= (others => '0'); end if;
                    wtotal <= wtotal + cur;
                    -- this frame's bits into the map, the previous frame's out
                    map_we <= '1'; map_wa <= map_rd_addr; map_wd <= sat16(cur);
                    if done_cnt < MAX_MBS then done_cnt <= done_cnt + 1; end if;
                    mq_bits(mq_wp) <= cur;
                    if map_valid = '1' then mq_w(mq_wp) <= map_q; else mq_w(mq_wp) <= to_unsigned(1, 16); end if;
                    -- synthesis translate_off
                    assert mq_cnt < 4 report "rc_mb_engine: marker queue overflow" severity failure;
                    -- synthesis translate_on
                    if mq_wp = 3 then mq_wp <= 0; else mq_wp <= mq_wp + 1; end if;
                elsif push_valid_i = '1' then
                    bits_acc <= bits_acc + resize(push_len_i, 32);
                end if;
                if mb_end_i = '1' and mq_pop = '0' then mq_cnt <= mq_cnt + 1;
                elsif mb_end_i = '0' and mq_pop = '1' then mq_cnt <= mq_cnt - 1;
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Shared multiplier
    ------------------------------------------------------------------
    mul_p : process(clk)
    begin
        if rising_edge(clk) then
            prod <= mul_a * mul_b;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Compute FSM: one marker -> one QP
    ------------------------------------------------------------------
    mq_pop <= '1' when (cst = C_IDLE and mq_cnt > 0 and frame_start_i = '0' and seed_left = 0) else '0';

    comp_p : process(clk, rst_n)
        variable e     : signed(33 downto 0);
        variable adj   : integer range -31 to 31;
        variable qt    : integer range -64 to 127;
        variable ok    : boolean;
        variable e100  : unsigned(38 downto 0);
    begin
        if rst_n = '0' then
            cst <= C_IDLE; qf_push <= '0'; mq_rp <= 0; seed_left <= 0;
        elsif rising_edge(clk) then
            qf_push <= '0';
            if frame_start_i = '1' then
                en <= en_i; map_valid <= map_valid_i;
                qp_frame <= qp_frame_i; qp_min <= qp_min_i; qp_max <= qp_max_i;
                target <= target_i; scale <= scale_i; lag <= lag_i;
                spent <= (others => '0'); acc <= (others => '0');
                qp_cur <= qp_frame_i;
                mq_rp <= 0;
                cst <= C_IDLE;
                seed_left <= to_integer(lag_i) + 1;      -- MBs 0 .. lag get qp_frame
            elsif seed_left > 0 then
                qf_push <= en; qf_din <= qp_frame;      -- nothing is ever popped with en clear
                seed_left <= seed_left - 1;
            else
                case cst is
                    when C_IDLE =>
                        if mq_pop = '1' then
                            cur_bits <= mq_bits(mq_rp); cur_w <= mq_w(mq_rp);
                            if mq_rp = 3 then mq_rp <= 0; else mq_rp <= mq_rp + 1; end if;
                            mul_a <= scale; mul_b <= mq_w(mq_rp);
                            cst <= C_MUL_W;
                        end if;
                    when C_MUL_W =>
                        -- prod = w * scale lands this cycle
                        cst <= C_ACC;
                    when C_ACC =>
                        acc   <= acc + prod;
                        spent <= spent + cur_bits;
                        cst <= C_EXPECT;
                    when C_EXPECT =>
                        -- acc < 2^48, so acc >> 16 fits the 32 bits
                        expect <= acc(47 downto 16);
                        cst <= C_GUARD;
                    when C_GUARD =>
                        -- expect * 100 > target and spent > 0
                        e100 := (resize(expect, 39) sll 6) + (resize(expect, 39) sll 5) + (resize(expect, 39) sll 2);
                        ok := (e100 > resize(target, 39)) and (spent /= 0);
                        e := signed(resize(spent, 34)) - signed(resize(expect, 34));
                        if e < 0 then e_neg <= '1'; m4 <= unsigned(resize(-e, 35)) sll 2;
                        else           e_neg <= '0'; m4 <= unsigned(resize(e, 35)) sll 2;
                        end if;
                        if ok then
                            guard <= '1';
                            lo <= 0; hi <= 32; mid <= 16;
                            mul_a <= expect; mul_b <= resize(T12(16), 16);
                            cst <= C_A_MUL;
                        else
                            guard <= '0'; a_cnt <= 16; b_cnt <= 0;
                            cst <= C_TARGET;
                        end if;
                    -- A: binary search for the count of j with spent*4096 >= expect*T[j]
                    when C_A_MUL =>
                        cst <= C_A_CMP;
                    when C_A_CMP =>
                        if (resize(spent, 48) sll 12) >= prod then
                            lo <= mid + 1;
                            if (mid + 1) < hi then
                                mid <= (mid + 1 + hi) / 2;
                                mul_a <= expect; mul_b <= resize(T12((mid + 1 + hi) / 2), 16);
                                cst <= C_A_MUL;
                            else
                                a_cnt <= mid + 1;
                                lo <= 1; hi <= 16; mid <= 8;
                                mul_a <= target; mul_b <= to_unsigned(8, 16);
                                cst <= C_B_MUL;
                            end if;
                        else
                            hi <= mid;
                            if lo < mid then
                                mid <= (lo + mid) / 2;
                                mul_a <= expect; mul_b <= resize(T12((lo + mid) / 2), 16);
                                cst <= C_A_MUL;
                            else
                                a_cnt <= lo;
                                lo <= 1; hi <= 16; mid <= 8;
                                mul_a <= target; mul_b <= to_unsigned(8, 16);
                                cst <= C_B_MUL;
                            end if;
                        end if;
                    -- B: largest b in 1..15 with b*target <= |4e| (0 if none)
                    when C_B_MUL =>
                        cst <= C_B_CMP;
                    when C_B_CMP =>
                        if resize(m4, 48) >= prod then
                            lo <= mid + 1;
                            if (mid + 1) < hi then
                                mid <= (mid + 1 + hi) / 2;
                                mul_a <= target; mul_b <= to_unsigned((mid + 1 + hi) / 2, 16);
                                cst <= C_B_MUL;
                            else
                                b_cnt <= mid;          -- = lo_new - 1
                                cst <= C_TARGET;
                            end if;
                        else
                            hi <= mid;
                            if lo < mid then
                                mid <= (lo + mid) / 2;
                                mul_a <= target; mul_b <= to_unsigned((lo + mid) / 2, 16);
                                cst <= C_B_MUL;
                            else
                                b_cnt <= lo - 1;
                                cst <= C_TARGET;
                            end if;
                        end if;
                    when C_TARGET =>
                        -- qp_target = clamp(qp_frame + adj); registered, the
                        -- step compare is the next cycle (one cycle at 200 MHz
                        -- does not hold the sum, the clamp and the compare)
                        if guard = '1' then
                            if e_neg = '1' then adj := (a_cnt - 16) - b_cnt; else adj := (a_cnt - 16) + b_cnt; end if;
                        else
                            adj := 0;
                        end if;
                        qt := to_integer(qp_frame) + adj;
                        if qt < to_integer(qp_min) then qt := to_integer(qp_min); end if;
                        if qt > to_integer(qp_max) then qt := to_integer(qp_max); end if;
                        qt_r <= qt;
                        cst <= C_STEP;
                    when C_STEP =>
                        qt := qt_r;
                        if qt > to_integer(qp_cur) then
                            qp_cur <= qp_cur + 1; qf_din <= qp_cur + 1;
                        elsif qt < to_integer(qp_cur) then
                            qp_cur <= qp_cur - 1; qf_din <= qp_cur - 1;
                        else
                            qf_din <= qp_cur;
                        end if;
                        qf_push <= en;
                        cst <= C_IDLE;
                end case;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- QP FIFO to the controller
    ------------------------------------------------------------------
    qf_p : process(clk, rst_n)
    begin
        if rst_n = '0' then
            qf_wp <= 0; qf_rp <= 0; qf_cnt <= 0;
        elsif rising_edge(clk) then
            if frame_start_i = '1' then
                qf_wp <= 0; qf_rp <= 0; qf_cnt <= 0;
            else
                if qf_push = '1' then
                    qf(qf_wp) <= qf_din;
                    if qf_wp = 15 then qf_wp <= 0; else qf_wp <= qf_wp + 1; end if;
                end if;
                if take_i = '1' and en = '1' then
                    if qf_rp = 15 then qf_rp <= 0; else qf_rp <= qf_rp + 1; end if;
                end if;
                if qf_push = '1' and not (take_i = '1' and en = '1') then qf_cnt <= qf_cnt + 1;
                elsif qf_push = '0' and (take_i = '1' and en = '1') then qf_cnt <= qf_cnt - 1;
                end if;
                -- synthesis translate_off
                assert not (qf_push = '1' and qf_cnt = 16) report "rc_mb_engine: QP FIFO overflow" severity failure;
                assert not (take_i = '1' and en = '1' and qf_cnt = 0) report "rc_mb_engine: QP taken while empty" severity failure;
                -- synthesis translate_on
            end if;
        end if;
    end process;

    qp_o       <= qf(qf_rp) when en = '1' else qp_frame;
    qp_valid_o <= '1' when (en = '0' or qf_cnt > 0) else '0';

end architecture;
