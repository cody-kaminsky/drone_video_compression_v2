--------------------------------------------------------------------------------
-- mode_decide_engine.vhd
--
-- Per-macroblock intra mode decision and luma/chroma coding, the hardware
-- form of mb_mode_decide + try_path_i4x4 + the chroma stages (mb_residual
-- .. mb_reconstruct) in src/encoder.c, with the I_4x4 policy of
-- src/rd_tables.h:
--
--   screen   every 4x4 block's nine modes are ranked by SATD against
--            "open-loop" neighbours: reconstructed samples where the
--            neighbour is another MB (available at MB start), SOURCE
--            samples where it is a block of this MB. This takes the mode
--            ranking out of the reconstruction loop.
--   rank     when a block's neighbours have been reconstructed and their
--            modes are known, the predicted mode's bit penalty is applied
--            (1 vs 4 mode bits, RD_SLAM16) and the best RD_I4_SHORTLIST
--            modes are kept.
--   chain    the shortlist is evaluated closed-loop (true prediction,
--            DCT, quant, CAVLC bits, dequant, IDCT, reconstruction, SSD)
--            and the block commits J = 16*SSD + lambda*(bits + mode bits).
--
-- One shared datapath, free-running (no back-pressure inside):
--
--   predict_4x4 -> residual A -> Hadamard lane -> abs-sum   (screen)
--   predict_4x4 -> residual A -> transform (DIR both) -> quant (fwd/inv)
--     -> cavlc_cost_engine_ll  |  loop-back -> quant (inv) -> transform
--     (inv) -> recon_engine (+SSD)                           (chain)
--   predict_16x16 / predict_chroma -> residual B -> Hadamard lane or
--     transform / quant / cost / recon as above              (background)
--
-- Sequencers sharing it:
--
--   screen      the 16 blocks in wavefront order, 4-cycle neighbour prep
--               from the source RAM (double-buffered), nine beats each.
--   chain       the I_4x4 wavefront (10 steps of one or two blocks): prep
--               true neighbours, rank (2 cycles), issue the shortlist,
--               wait for the last J, commit. The second block of a pair
--               preps and ranks while the first block's candidates issue.
--   background  the I_16x16 screen, forward, DC Hadamard and inverse, then
--               the chroma screen, forward, DC chain and inverse. Owns the
--               16x16 / chroma predictors, the DC-vector transform port and
--               the level-store inverse port.
--
-- Every beat carries a tag through delay lines that match each engine's
-- fixed latency. Because the latencies are fixed, an issue reserves its
-- future slots on the shared ports (16x16/chroma residual merge, Hadamard
-- lane, transform, quant, abs-sum tree) in a scoreboard; an issue that
-- would collide waits a cycle. The chain's candidate bursts have priority:
-- background beats are only issued when their slots precede the next burst.
--
-- Then the decision is presented (done_o) and the reconstruction blocks
-- and the level blocks are streamed out from their own FSM. The next MB
-- may start as soon as the reconstruction stream is out: the I_4x4 level
-- store is banked by MB parity, and the background's first level-store
-- write (I_16x16 forward) waits for the level stream to finish.
--
-- The source MB arrives as a 24-word stream right after start; it lives in
-- a LUT RAM (four read ports: two residual stages, recon, screen prep).
--
-- Sample buses: block k (raster br*4+bc) at bits 128k+127 downto 128k,
-- sample (r,c) of a block at byte 4r+c. Level blocks are streamed in
-- raster coefficient order (the consumer applies the zigzag).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cavlc_pkg.all;

entity mode_decide_engine is
    generic (
        DEBUG : boolean := false
    );
    port (
        clk            : in  std_logic;
        rst_n          : in  std_logic;
        -- control
        start_i        : in  std_logic;
        busy_o         : out std_logic;
        done_o         : out std_logic;     -- decision + stores complete
        stream_done_o  : out std_logic;     -- all output items taken
        qp_y_i         : in  unsigned(5 downto 0);
        qp_c_i         : in  unsigned(5 downto 0);
        -- source MB: 24 blocks (Y 0..15 raster, U 0..3, V 0..3) presented one
        -- per cycle, consecutively, starting the cycle after start_i is taken
        src_data_i     : in  std_logic_vector(127 downto 0);
        stream_busy_o  : out std_logic;     -- output streams still draining
        -- luma neighbours (line_buffer bundle)
        top_y_i        : in  std_logic_vector(127 downto 0);
        tr_y_i         : in  std_logic_vector(31 downto 0);
        tl_y_i         : in  std_logic_vector(7 downto 0);
        left_y_i       : in  std_logic_vector(127 downto 0);
        avail_top_i    : in  std_logic;
        avail_left_i   : in  std_logic;
        avail_tl_i     : in  std_logic;
        avail_tr_i     : in  std_logic;
        -- chroma neighbours
        top_u_i        : in  std_logic_vector(63 downto 0);
        left_u_i       : in  std_logic_vector(63 downto 0);
        tl_u_i         : in  std_logic_vector(7 downto 0);
        top_v_i        : in  std_logic_vector(63 downto 0);
        left_v_i       : in  std_logic_vector(63 downto 0);
        tl_v_i         : in  std_logic_vector(7 downto 0);
        -- neighbour intra 4x4 modes (2 = DC where unavailable)
        mode4_top_i    : in  std_logic_vector(15 downto 0);
        mode4_left_i   : in  std_logic_vector(15 downto 0);
        -- decision (valid from done_o until the next start)
        is_i4x4_o      : out std_logic;
        mode16_o       : out unsigned(1 downto 0);
        modes4_o       : out std_logic_vector(63 downto 0);
        mode_chroma_o  : out unsigned(1 downto 0);
        luma_nz_o      : out std_logic_vector(15 downto 0);   -- raster
        chroma_dc_nz_o : out std_logic;
        chroma_ac_nz_o : out std_logic;
        bits_a_o       : out unsigned(15 downto 0);
        bits_b_o       : out unsigned(15 downto 0);
        -- TotalCoeff of every coded block for the CAVLC neighbours (0 when
        -- the block is not emitted per the coded_block_pattern): 5 bits per
        -- block, luma raster 0..15, chroma 0..3 per plane
        tc_y_o         : out std_logic_vector(79 downto 0);
        tc_u_o         : out std_logic_vector(19 downto 0);
        tc_v_o         : out std_logic_vector(19 downto 0);
        dbg_j_o        : out unsigned(27 downto 0);   -- last RD winner's J (debug)
        -- level block stream (emission order)
        blk_valid_o    : out std_logic;
        blk_ready_i    : in  std_logic;
        blk_plane_o    : out unsigned(1 downto 0);   -- 0 Y, 1 U, 2 V
        blk_kind_o     : out std_logic;              -- 0 coefficients, 1 DC block
        blk_idx_o      : out unsigned(3 downto 0);   -- raster block index
        blk_levels_o   : out level_array_t;          -- raster coefficient order
        -- reconstruction stream (Y 0..15, U 0..3, V 0..3)
        rec_valid_o    : out std_logic;
        rec_ready_i    : in  std_logic;
        rec_plane_o    : out unsigned(1 downto 0);
        rec_idx_o      : out unsigned(3 downto 0);
        rec_data_o     : out std_logic_vector(127 downto 0)
    );
end entity;

architecture rtl of mode_decide_engine is

    ------------------------------------------------------------------
    -- constants
    ------------------------------------------------------------------
    type int16_t is array (0 to 15) of integer;
    constant SCAN_BR : int16_t := (0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3);
    constant SCAN_BC : int16_t := (0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3);
    constant ZZ      : int16_t := (0,1,4,8,5,2,3,6,9,12,13,10,7,11,14,15);
    -- quadrant of a raster block (coded_block_pattern luma bit)
    constant QUAD    : int16_t := (0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3);
    -- screen order: the wavefront steps flattened (scan indices)
    constant SCREEN_ORDER : int16_t := (0,1,4,2,5,3,6,8,7,9,12,10,13,11,14,15);

    type qp_tab_t is array (0 to 51) of integer;
    constant RD_SLAM16 : qp_tab_t := (
           8,    9,   10,   11,   13,   14,   16,   18,   20,   23,   25,   29,   32,
          36,   40,   45,   51,   57,   64,   72,   81,   91,  102,  114,  128,  144,
         161,  181,  203,  228,  256,  287,  323,  362,  406,  456,  512,  575,  645,
         724,  813,  912, 1024, 1149, 1290, 1448, 1625, 1825, 2048, 2299, 2580, 2896);
    constant RD_LAM16 : qp_tab_t := (
           0,     1,     1,     1,     1,     1,     2,     2,     3,     3,     4,     5,     7,
           9,    11,    14,    17,    22,    27,    34,    43,    54,    69,    86,   109,   137,
         173,   218,   274,   345,   435,   548,   691,   870,  1097,  1382,  1741,  2193,  2763,
        3482,  4387,  5527,  6963,  8773, 11053, 13926, 17546, 22107, 27853, 35092, 44214, 55706);
    constant SHORTLIST : integer := 3;

    -- result latencies (cycles after the last issue) to wait per phase
    constant W_SCR16 : integer := 14;
    constant W_FWD16 : integer := 12;
    constant W_HAD16 : integer := 14;
    constant W_INV16 : integer := 4;
    constant W_FULL4 : integer := 16;   -- the last J lands as the wait expires; commit reads it the cycle after
    constant W_SCRC  : integer := 10;
    constant W_FWDC  : integer := 10;   -- +1: the DC vector is registered at the HADC issue
    constant W_HADC  : integer := 14;
    constant W_INVC  : integer := 11;

    -- Scoreboard slot offsets, in cycles after the issue cycle (the cycle
    -- in which the head tag is visible):
    --   R16 16x16/chroma predictor merge   H  Hadamard lane input (4x4 screen)
    --   T   transform input                Q  quantizer input
    --   A   4x4 abs-sum tree               B  16x16/chroma abs-sum tree
    --   SCR4  : H 3, A 5              FULL4 : T 3, Q 5, T 10 (fused dequant)
    --   SCR16 : R16 8, B 11 and T 9 (via the transform, tag slot 0) or
    --           H 9 (via the Hadamard lane, tag slot 1)
    --                                 FWD16 : R16 8, T 9, Q 11
    --   HAD16 / HADC : T 0, Q 2, Q 7, T 11
    --   INV16 / INVC : Q 0, T 4
    --   SCRC  : R16 6, B 7            FWDC  : R16 6, T 7, Q 9
    -- (the 4x4 predictor path issues at most one beat per cycle, so its
    -- own merge never conflicts). The chain's candidate bursts have
    -- priority: the cycles until the next burst's first head (fb) are
    -- known from the chain state, and a burst of up to BURST_LEN issues
    -- owns T in [fb+3, fb+3+BURST_LEN) u [fb+10, fb+10+BURST_LEN) and Q in
    -- [fb+5, fb+5+BURST_LEN); a background beat
    -- that uses the transform or quant must keep its slots out of those
    -- windows. The 16x16 screen goes through the transform when the burst
    -- windows leave it free, and through the Hadamard lane only in cycles
    -- the 4x4 screen cannot use (a candidate owns the 4x4 head six cycles
    -- later, or the 4x4 screen is finished).
    constant BURST_LEN : integer := 8;
    constant FB_NONE   : integer := 40;

    ------------------------------------------------------------------
    -- types
    ------------------------------------------------------------------
    subtype px128 is std_logic_vector(127 downto 0);
    type px128_arr is array (natural range <>) of px128;
    subtype s32 is signed(31 downto 0);
    type vec16_t is array (0 to 15) of s32;
    subtype lvl208 is std_logic_vector(207 downto 0);
    type lvl208_arr is array (natural range <>) of lvl208;
    subtype s20 is signed(19 downto 0);
    type s20_arr16 is array (0 to 15) of s20;
    type s16_arr16 is array (0 to 15) of signed(15 downto 0);
    type s16_arr4  is array (0 to 3)  of signed(15 downto 0);
    type s20_arr4  is array (0 to 3)  of s20;
    type u24_arr4  is array (0 to 3)  of unsigned(23 downto 0);
    subtype u5 is unsigned(4 downto 0);
    type u5_arr16 is array (0 to 15) of u5;
    type u5_arr8  is array (0 to 7)  of u5;
    type u5_arr6  is array (0 to 5)  of u5;
    subtype sbv_t is std_logic_vector(0 to 15);
    subtype u24 is unsigned(23 downto 0);

    type op_t is (OP_NONE, OP_SCR4, OP_FULL4, OP_SCR16, OP_FWD16, OP_HAD16, OP_INV16,
                  OP_SCRC, OP_FWDC, OP_HADC, OP_INVC);
    type tag_t is record
        valid : std_logic;
        op    : op_t;
        plane : integer range 0 to 2;
        blk   : integer range 0 to 15;
        mode  : integer range 0 to 8;
        slot  : integer range 0 to 2;     -- candidate slot; SCR4: block parity
        inv   : std_logic;                -- inverse beat; SCR4: last mode of the block
        sub   : integer range 0 to 2;     -- wavefront sub-block; 2 = screen neighbours
    end record;
    constant TAG_NONE : tag_t := ('0', OP_NONE, 0, 0, 0, 0, '0', 0);
    type tag_arr is array (natural range <>) of tag_t;

    ------------------------------------------------------------------
    -- helper functions
    ------------------------------------------------------------------
    function byte_of(v : std_logic_vector; k : integer) return std_logic_vector is
    begin
        return v(8 * k + 7 downto 8 * k);
    end function;

    function pack13(x : vec16_t) return lvl208 is
        variable r : lvl208;
    begin
        for i in 0 to 15 loop
            r(13 * i + 12 downto 13 * i) := std_logic_vector(x(i)(12 downto 0));
        end loop;
        return r;
    end function;

    function unpack13(r : lvl208) return level_array_t is
        variable p : level_array_t;
    begin
        for i in 0 to 15 loop
            p(i) := resize(signed(r(13 * i + 12 downto 13 * i)), 16);
        end loop;
        return p;
    end function;

    function unpack13_v(r : lvl208) return vec16_t is
        variable p : vec16_t;
    begin
        for i in 0 to 15 loop
            p(i) := resize(signed(r(13 * i + 12 downto 13 * i)), 32);
        end loop;
        return p;
    end function;

    function any_nz(x : vec16_t; lo : integer) return std_logic is
        variable r : std_logic := '0';
    begin
        for i in lo to 15 loop
            if x(i) /= 0 then r := '1'; end if;
        end loop;
        return r;
    end function;

    -- number of nonzero levels (levels fit 13 bits)
    function popnz(x : vec16_t) return u5 is
        variable c : u5 := (others => '0');
    begin
        for i in 0 to 15 loop
            if x(i)(12 downto 0) /= 0 then c := c + 1; end if;
        end loop;
        return c;
    end function;

    -- level store address for a tag (I_16x16 AC 0..15, DC 16, chroma AC
    -- 48..55, chroma DC 56/57; the I_4x4 levels live in their own store)
    function lvl_addr(t : tag_t) return integer is
    begin
        case t.op is
            when OP_FWD16 | OP_INV16 => return t.blk;
            when OP_HAD16 => return 16;
            when OP_FWDC | OP_INVC => return 48 + (t.plane - 1) * 4 + t.blk;
            when others => return 56 + (t.plane - 1);                 -- HADC
        end case;
    end function;

    -- 16x16 / chroma prediction store slot
    function pred16_slot(t : tag_t) return integer is
    begin
        if t.op = OP_FWD16 or t.op = OP_INV16 then return t.blk; end if;
        return 16 + (t.plane - 1) * 4 + t.blk;
    end function;

    function tr_avail_blk(s : integer) return boolean is
    begin
        return not (s = 3 or s = 7 or s = 11 or s = 13 or s = 15);
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

    -- mode availability for a 4x4 block
    function mode_ok(m : integer; at, al : std_logic) return boolean is
    begin
        if (m = 0 or m = 3 or m = 7) and at = '0' then return false; end if;
        if (m = 1 or m = 8) and al = '0' then return false; end if;
        if (m = 4 or m = 5 or m = 6) and not (at = '1' and al = '1') then return false; end if;
        return true;
    end function;

    -- (cost, mode) ordering: smaller cost first, ties by lower mode
    function before(ca : u24; ma : integer; cb : u24; mb : integer) return boolean is
    begin
        return (ca < cb) or (ca = cb and ma < mb);
    end function;

    -- a background transform slot k (cycles after its issue cycle) stays
    -- out of the planned candidate burst that starts fb cycles from now
    function t_free(k, fb : integer) return boolean is
        variable c : integer;
    begin
        c := k + 1;
        return c < fb + 3 or c >= fb + 10 + BURST_LEN or (c >= fb + 3 + BURST_LEN and c < fb + 10);
    end function;
    function q_free(k, fb : integer) return boolean is
        variable c : integer;
    begin
        c := k + 1;
        return c < fb + 5 or c >= fb + 5 + BURST_LEN;
    end function;

    ------------------------------------------------------------------
    -- registered MB inputs
    ------------------------------------------------------------------
    attribute ram_style : string;
    signal qp_y, qp_c : unsigned(5 downto 0);
    type src_mem_t is array (0 to 31) of px128;
    signal src_mem : src_mem_t;
    attribute ram_style of src_mem : signal is "distributed";
    signal src_ld : integer range 0 to 24 := 24;
    signal src_ra_r4, src_ra_r16, src_ra_rec, src_ra_nb : integer range 0 to 31;
    signal src_rd_r4, src_rd_r16, src_rd_rec, src_rd_nb : px128;
    signal top_y, left_y : px128;
    signal tr_y : std_logic_vector(31 downto 0);
    signal tl_y, tl_u, tl_v : std_logic_vector(7 downto 0);
    signal top_u, left_u, top_v, left_v : std_logic_vector(63 downto 0);
    signal a_top, a_left, a_tl, a_tr : std_logic;
    signal m4top, m4left : std_logic_vector(15 downto 0);
    signal pen1, pen4 : unsigned(11 downto 0);
    signal lam : unsigned(15 downto 0);

    ------------------------------------------------------------------
    -- sequencers
    ------------------------------------------------------------------
    type st_t is (S_IDLE, S_I4_PREP, S_I4_RK1, S_I4_RK2, S_I4_FULL, S_I4_COMMIT, S_PICK, S_DONE);
    signal st : st_t := S_IDLE;
    type bst_t is (B_IDLE, B_SCR16, B_CSCR, B_PICK, B_PICK2, B_FWD16, B_CPICK, B_CPICK2, B_CFWD,
                   B_HAD16, B_CHAD, B_INV16, B_CINV, B_DONE);
    signal bst : bst_t := B_IDLE;
    signal wait_cnt : integer range 0 to 31 := 0;
    signal ck : integer range 0 to 7 := 0;      -- candidates issued (both sub-blocks)
    signal ci : integer range 0 to 1 := 0;      -- sub-block whose candidates are issuing
    signal bwait : integer range 0 to 31 := 0;      -- I_16x16 chain latency wait
    signal bwaitc : integer range 0 to 31 := 0;     -- chroma chain latency wait
    signal bcm : integer range 0 to 4 := 0;     -- background mode counter
    signal bcb : integer range 0 to 16 := 0;    -- background block counter
    signal bcp : integer range 0 to 2 := 1;     -- background plane counter
    signal issuing : std_logic := '0';          -- a 4x4 beat left the head this cycle
    signal head_tag : tag_t := TAG_NONE;
    signal bissuing : std_logic := '0';         -- a background beat left the head this cycle
    signal bhead_tag : tag_t := TAG_NONE;
    -- scoreboard: bit j = port used j cycles from now
    signal sb_r, sb_h, sb_t, sb_q, sb_a, sb_b : sbv_t := (others => '0');

    -- screen sequencer
    signal sp_idx : integer range 0 to 16 := 16;    -- block being prepped (screen order position)
    signal sp_step : integer range 0 to 4 := 0;
    signal si_idx : integer range 0 to 16 := 16;    -- block being issued
    signal sm : integer range 0 to 9 := 0;          -- screen mode counter
    signal nbs_ready : std_logic_vector(1 downto 0) := "00";   -- prepped neighbours per parity
    type nbtop_t is array (0 to 1) of std_logic_vector(63 downto 0);
    type nbleft_t is array (0 to 1) of std_logic_vector(31 downto 0);
    type nbtl_t is array (0 to 1) of std_logic_vector(7 downto 0);
    signal nbs_top : nbtop_t := (others => (others => '0'));
    signal nbs_left : nbleft_t := (others => (others => '0'));
    signal nbs_tl : nbtl_t := (others => (others => '0'));
    signal nbs_at, nbs_al : std_logic_vector(1 downto 0) := (others => '0');
    signal sp_top : std_logic_vector(63 downto 0) := (others => '0');   -- prep in progress
    -- screen rank lists (top 4 by raw SATD), one per block parity, and the stores
    type sk_cost_t is array (0 to 7) of u24;
    type sk_mode_t is array (0 to 7) of integer range 0 to 8;
    type sk_n_t is array (0 to 1) of integer range 0 to 4;
    signal sk_cost : sk_cost_t := (others => (others => '0'));
    signal sk_mode : sk_mode_t := (others => 0);
    signal sk_n : sk_n_t := (others => 0);
    signal sk_wr : std_logic := '0';                -- write the finished list next cycle
    signal sk_wr_par : integer range 0 to 1 := 0;
    signal sk_wr_blk : integer range 0 to 15 := 0;
    subtype sklist_t is std_logic_vector(114 downto 0);   -- 4 x (24 cost + 4 mode) + 3 count
    type scr_top_t is array (0 to 15) of sklist_t;
    signal scr_top : scr_top_t;
    attribute ram_style of scr_top : signal is "distributed";
    signal scr_top_rd : sklist_t;
    signal scr_top_ra : integer range 0 to 15;
    type scr_raw_t is array (0 to 255) of std_logic_vector(23 downto 0);
    signal scr_raw : scr_raw_t;
    attribute ram_style of scr_raw : signal is "distributed";
    signal scr_raw_rd : std_logic_vector(23 downto 0);
    signal scr_raw_ra : integer range 0 to 255;
    signal scr_done : std_logic_vector(15 downto 0) := (others => '0');

    -- decision state
    signal mode16  : integer range 0 to 3 := 2;
    signal modec   : integer range 0 to 3 := 0;
    signal modes4  : std_logic_vector(63 downto 0) := (others => '0');
    signal is_i4   : std_logic := '0';
    signal bits_a, bits_b : unsigned(15 downto 0) := (others => '0');
    signal nz16, nz4 : std_logic_vector(15 downto 0) := (others => '0');
    signal cdc_nz, cac_nz : std_logic := '0';
    signal acc16 : u24_arr4 := (others => (others => '0'));
    signal accc  : u24_arr4 := (others => (others => '0'));
    signal tcnt16 : u5_arr16 := (others => (others => '0'));
    signal tcnt4  : u5_arr16 := (others => (others => '0'));
    signal tcntc  : u5_arr8  := (others => (others => '0'));

    -- I4 wavefront: up to two independent blocks per step (sub 0 / sub 1)
    type step_tab_t is array (0 to 9, 0 to 1) of integer range -1 to 15;   -- scan indices
    constant STEP_TAB : step_tab_t := (
        (0, -1), (1, -1), (4, 2), (5, 3), (6, 8), (7, 9), (12, 10), (13, 11), (14, -1), (15, -1));
    type i2_t is array (0 to 1) of integer range 0 to 15;
    type m2_t is array (0 to 1) of integer range 0 to 8;
    signal step : integer range 0 to 9 := 0;
    signal nsub : integer range 1 to 2 := 1;
    signal cc : integer range 0 to 1 := 0;          -- sub-block cursor (prep / rank / commit)
    signal cur_blk : i2_t := (others => 0);
    signal pred_mode : m2_t := (others => 2);
    signal nb_top : nbtop_t := (others => (others => '0'));
    signal nb_left : nbleft_t := (others => (others => '0'));
    signal nb_tl : nbtl_t := (others => (others => '0'));
    signal nb_at, nb_al : std_logic_vector(1 downto 0) := (others => '0');
    signal p4_top : std_logic_vector(63 downto 0);
    signal p4_left : std_logic_vector(31 downto 0);
    signal p4_tl : std_logic_vector(7 downto 0);
    signal p4_at, p4_al : std_logic;
    -- registered predictor inputs (the neighbour-set mux is off the predictor's path)
    signal p4_top_q : std_logic_vector(63 downto 0) := (others => '0');
    signal p4_left_q : std_logic_vector(31 downto 0) := (others => '0');
    signal p4_tl_q : std_logic_vector(7 downto 0) := (others => '0');
    signal p4_at_q, p4_al_q, p4_valid_q : std_logic := '0';
    signal p4_mode_q : unsigned(3 downto 0) := (others => '0');
    -- chain rank: compacted list (pred mode removed) then the pred mode inserted
    type rk_mode_t is array (0 to 5) of integer range 0 to 8;
    type rk_n_t is array (0 to 1) of integer range 0 to 3;
    signal rk_mode : rk_mode_t := (others => 0);
    signal nfull : rk_n_t := (others => 0);
    signal rk_ready : std_logic_vector(1 downto 0) := "00";
    type rc3_t is array (0 to 2) of u24;
    type rm3_t is array (0 to 2) of integer range 0 to 8;
    signal rk1_cost : rc3_t := (others => (others => '0'));
    signal rk1_mode : rm3_t := (others => 0);
    signal rk1_n : integer range 0 to 3 := 0;
    signal rk1_pcost : u24 := (others => '0');
    type cand_bits_t is array (0 to 5) of unsigned(9 downto 0);
    type cand_mb_t is array (0 to 5) of integer range 1 to 4;
    type cand_lvl_t is array (0 to 7) of lvl208;
    type cand_rec_t is array (0 to 7) of px128;
    signal cand_lvl : cand_lvl_t;
    signal cand_rec : cand_rec_t;
    attribute ram_style of cand_lvl : signal is "distributed";
    attribute ram_style of cand_rec : signal is "distributed";
    signal cand_lvl_rd : lvl208;
    signal cand_rec_rd : px128;
    signal cand_ra : integer range 0 to 7;
    signal cand_bits : cand_bits_t := (others => (others => '0'));
    signal cand_mbits : cand_mb_t := (others => 1);
    signal cand_nz : std_logic_vector(5 downto 0) := (others => '0');
    signal cand_cnt : u5_arr6 := (others => (others => '0'));
    type bj_t is array (0 to 1) of unsigned(27 downto 0);
    type bs_t is array (0 to 1) of integer range 0 to 2;
    signal best_j : bj_t := (others => (others => '1'));
    signal best_slot : bs_t := (others => 0);
    -- rate term lambda*(bits + mode bits): the bit sum is registered as the
    -- estimate lands (two cycles before the SSD), multiplied the next cycle,
    -- and meets the SSD the cycle after
    signal jb_q : unsigned(11 downto 0) := (others => '0');
    signal jp1 : unsigned(27 downto 0) := (others => '0');
    attribute use_dsp : string;
    attribute use_dsp of jp1 : signal is "yes";
    -- the J add / compare stays in fabric: without this the tool folds the
    -- recon engine's SSD register into the multiplier's C register and puts
    -- the SSD adder tree on the same cycle
    attribute dont_touch : string;
    attribute dont_touch of jp1 : signal is "true";

    -- reconstruction / DC state
    type r4_mem_t is array (0 to 15) of px128;
    signal r4 : r4_mem_t;
    attribute ram_style of r4 : signal is "distributed";
    signal r4_stream : px128;
    -- pairwise pick pipeline
    signal pk_a, pk_b : integer range 0 to 3 := 0;
    signal pk_va, pk_vb : u24 := (others => '0');
    type rec_mem_t is array (0 to 31) of px128;
    signal rec_mem : rec_mem_t;                     -- R16 at 0..15, U 16..19, V 20..23
    attribute ram_style of rec_mem : signal is "distributed";
    signal rec_we : std_logic;
    signal rec_wa : integer range 0 to 31;
    signal rec_wd : px128;
    signal rec_ra : integer range 0 to 31 := 0;
    signal dc16 : s16_arr16 := (others => (others => '0'));
    signal dcrec16 : s20_arr16 := (others => (others => '0'));
    type dcc_t is array (0 to 1) of s16_arr4;
    type dccr_t is array (0 to 1) of s20_arr4;
    signal dcc : dcc_t := (others => (others => (others => '0')));
    signal dccrec : dccr_t := (others => (others => (others => '0')));

    -- level store (background results) and the banked I_4x4 level store
    type lvl_mem_t is array (0 to 63) of lvl208;
    signal lvl_mem : lvl_mem_t;
    attribute ram_style of lvl_mem : signal is "distributed";
    signal lvl_we : std_logic;
    signal lvl_wa : integer range 0 to 63;
    signal lvl_wd : lvl208;
    signal lvl_ra : integer range 0 to 63 := 0;
    signal lvl_ra_stream : integer range 0 to 63 := 0;
    signal lvl_rd : lvl208;
    type lvl4_mem_t is array (0 to 31) of lvl208;
    signal lvl4_mem : lvl4_mem_t;
    attribute ram_style of lvl4_mem : signal is "distributed";
    signal lvl4_bank : std_logic := '0';
    signal lvl4_ra : integer range 0 to 31 := 0;
    signal lvl4_rd : lvl208;

    -- prediction stores
    type pred4_mem_t is array (0 to 7) of px128;
    signal pred4_mem : pred4_mem_t;
    attribute ram_style of pred4_mem : signal is "distributed";
    type pred16_mem_t is array (0 to 31) of px128;
    signal pred16_mem : pred16_mem_t;
    attribute ram_style of pred16_mem : signal is "distributed";
    signal pred4_ra : integer range 0 to 7;
    signal pred16_ra : integer range 0 to 31;

    ------------------------------------------------------------------
    -- engines
    ------------------------------------------------------------------
    signal p4_valid_i, p4_valid_o, p16_valid_i, p16_valid_o, pc_valid_i, pc_valid_o : std_logic;
    signal p4_mode : unsigned(3 downto 0);
    signal p16_mode, pc_mode : unsigned(1 downto 0);
    signal p16_blk : unsigned(3 downto 0);
    signal pc_blk : unsigned(1 downto 0);
    -- chroma predictor neighbours, plane-selected when the beat is issued
    signal pc_top, pc_left : std_logic_vector(63 downto 0) := (others => '0');
    signal pc_tl : std_logic_vector(7 downto 0) := (others => '0');
    signal p4_pred, p16_pred, pc_pred : px128;
    signal tp4 : tag_arr(1 to 2) := (others => TAG_NONE);
    signal tp16 : tag_arr(1 to 8) := (others => TAG_NONE);
    signal tpc : tag_arr(1 to 6) := (others => TAG_NONE);

    -- residual stages: A (4x4 predictor), B (16x16 / chroma predictors)
    signal res4_q : s20_arr16 := (others => (others => '0'));
    signal res4_tag : tag_t := TAG_NONE;
    signal res4_pred : px128 := (others => '0');
    signal res16_q : s20_arr16 := (others => (others => '0'));
    signal res16_tag : tag_t := TAG_NONE;
    signal res16_pred : px128 := (others => '0');

    -- Hadamard lane (both screens), input select decided a cycle ahead
    signal h_din, h_dout : vec16_t;
    signal h_valid_i, h_valid_o : std_logic;
    signal h_sel4_q, h_sel16_q : std_logic := '0';
    signal h_tag_q : tag_t := TAG_NONE;
    signal th : tag_arr(1 to 2) := (others => TAG_NONE);

    -- transform (input select, mode and tag are decided one cycle ahead)
    signal t_din, t_dout : vec16_t;
    signal t_valid_i, t_valid_o : std_logic;
    signal t_sel_lb_q, t_sel_dq_q, t_sel_dc_q, t_sel_r4_q, t_sel_r16_q : std_logic := '0';
    signal t_mode_q : unsigned(2 downto 0) := (others => '0');
    signal t_tag_q : tag_t := TAG_NONE;
    signal t_dc0_q : s32 := (others => '0');
    signal t_dc0_en_q : std_logic := '0';
    signal t_dcv_q : s16_arr16 := (others => (others => '0'));   -- DC vector, loaded at the HAD issue
    signal tt : tag_arr(1 to 2) := (others => TAG_NONE);

    -- quant (q_deq: fused dequantised block, one cycle after q_dout)
    signal q_din, q_dout, q_deq : vec16_t;
    signal q_deq_valid : std_logic;
    signal q_mode : unsigned(2 downto 0);
    signal q_qp : unsigned(5 downto 0);
    signal q_valid_i, q_valid_o : std_logic;
    signal q_tag_in : tag_t;
    signal tq : tag_arr(1 to 5) := (others => TAG_NONE);

    -- loop-back register (quant forward output -> quant inverse input)
    signal lb_data : vec16_t := (others => (others => '0'));
    signal lb_tag : tag_t := TAG_NONE;

    -- cost
    signal c_levels : level_array_t;
    signal c_n : unsigned(4 downto 0);
    signal c_valid_i, c_valid_o : std_logic;
    signal c_bits : unsigned(9 downto 0);
    signal tc : tag_arr(1 to 5) := (others => TAG_NONE);

    -- recon
    signal r_pred, r_src, r_recon : px128;
    signal r_res : std_logic_vector(16 * 20 - 1 downto 0);
    signal r_valid_i, r_valid_o : std_logic;
    signal r_ssd : unsigned(19 downto 0);
    signal r_tag_in : tag_t;
    signal r_pred_q, r_src_q : px128 := (others => '0');
    signal r_res_q : std_logic_vector(16 * 20 - 1 downto 0) := (others => '0');
    signal r_valid_q : std_logic := '0';
    signal r_tag_q : tag_t := TAG_NONE;
    signal trc : tag_arr(1 to 3) := (others => TAG_NONE);

    -- abs-sum trees: A for the 4x4 screen (Hadamard lane output), B for
    -- the 16x16 screen (transform output) and the chroma screen (residual B)
    type u20_arr16 is array (0 to 15) of unsigned(19 downto 0);
    signal ab1 : u20_arr16 := (others => (others => '0'));
    signal ab1_tag : tag_t := TAG_NONE;
    signal ab2 : u24 := (others => '0');
    signal ab2_tag : tag_t := TAG_NONE;
    signal bb1 : u20_arr16 := (others => (others => '0'));
    signal bb1_tag : tag_t := TAG_NONE;
    signal bb2 : u24 := (others => '0');
    signal bb2_tag : tag_t := TAG_NONE;

    -- output streams (own FSM so the next MB can start while they drain)
    type ost_t is (S_OIDLE, S_OREC, S_OBLK);
    signal ost : ost_t := S_OIDLE;
    signal o_item : integer range 0 to 31 := 0;
    signal o_is4 : std_logic := '0';
    signal o_bank : std_logic := '0';
    signal o_start : std_logic := '0';
    signal blk_valid_q : std_logic := '0';
    signal rec_valid_q : std_logic := '0';
    signal done_q, sdone_q : std_logic := '0';
    -- synthesis translate_off
    signal stat_bg, stat_ost, stat_mb, stat_fs, stat_bgdone, stat_scr, stat_rk, stat_t_scr16, stat_t_cscr, stat_t_fwd16, stat_t_cfwd, stat_t_inv16 : natural := 0;
    -- synthesis translate_on

begin

    ------------------------------------------------------------------
    -- Engine instances
    ------------------------------------------------------------------
    p4_nb_p : process(all)
    begin
        if head_tag.sub = 2 then
            p4_top <= nbs_top(head_tag.slot mod 2); p4_left <= nbs_left(head_tag.slot mod 2);
            p4_tl <= nbs_tl(head_tag.slot mod 2);
            p4_at <= nbs_at(head_tag.slot mod 2); p4_al <= nbs_al(head_tag.slot mod 2);
        else
            p4_top <= nb_top(head_tag.sub mod 2); p4_left <= nb_left(head_tag.sub mod 2);
            p4_tl <= nb_tl(head_tag.sub mod 2);
            p4_at <= nb_at(head_tag.sub mod 2); p4_al <= nb_al(head_tag.sub mod 2);
        end if;
    end process;

    p4_in_reg : process(clk)
    begin
        if rising_edge(clk) then
            p4_top_q <= p4_top; p4_left_q <= p4_left; p4_tl_q <= p4_tl;
            p4_at_q <= p4_at; p4_al_q <= p4_al; p4_mode_q <= p4_mode; p4_valid_q <= p4_valid_i;
        end if;
    end process;

    p4 : entity work.predict_4x4_engine
        port map (clk => clk, rst_n => rst_n, mode_i => p4_mode_q, top_i => p4_top_q, left_i => p4_left_q,
                  tl_i => p4_tl_q, avail_top_i => p4_at_q, avail_left_i => p4_al_q, avail_tl_i => p4_at_q and p4_al_q,
                  valid_i => p4_valid_q, ready_o => open, pred_o => p4_pred, valid_o => p4_valid_o,
                  ready_i => '1');

    p16 : entity work.predict_16x16_engine
        port map (clk => clk, rst_n => rst_n, mode_i => p16_mode, blk_i => p16_blk, top_i => top_y,
                  left_i => left_y, tl_i => tl_y, avail_top_i => a_top, avail_left_i => a_left,
                  avail_tl_i => a_tl, valid_i => p16_valid_i, ready_o => open, pred_o => p16_pred,
                  valid_o => p16_valid_o, ready_i => '1');

    pc : entity work.predict_chroma_engine
        port map (clk => clk, rst_n => rst_n, mode_i => pc_mode, blk_i => pc_blk, top_i => pc_top,
                  left_i => pc_left, tl_i => pc_tl, avail_top_i => a_top, avail_left_i => a_left,
                  avail_tl_i => a_tl, valid_i => pc_valid_i, ready_o => open, pred_o => pc_pred,
                  valid_o => pc_valid_o, ready_i => '1');

    -- Hadamard lane: a transform instance specialised to mode 3
    -- (ihadamard4x4, as the C SATD does)
    hd : entity work.transform_engine
        generic map (W => 14, DIR => "had")   -- 9-bit residuals: two Hadamard passes fit 13 bits
        port map (clk => clk, rst_n => rst_n, mode_i => "011",
                  din_0 => h_din(0), din_1 => h_din(1), din_2 => h_din(2), din_3 => h_din(3),
                  din_4 => h_din(4), din_5 => h_din(5), din_6 => h_din(6), din_7 => h_din(7),
                  din_8 => h_din(8), din_9 => h_din(9), din_10 => h_din(10), din_11 => h_din(11),
                  din_12 => h_din(12), din_13 => h_din(13), din_14 => h_din(14), din_15 => h_din(15),
                  valid_i => h_valid_i, ready_o => open,
                  dout_0 => h_dout(0), dout_1 => h_dout(1), dout_2 => h_dout(2), dout_3 => h_dout(3),
                  dout_4 => h_dout(4), dout_5 => h_dout(5), dout_6 => h_dout(6), dout_7 => h_dout(7),
                  dout_8 => h_dout(8), dout_9 => h_dout(9), dout_10 => h_dout(10), dout_11 => h_dout(11),
                  dout_12 => h_dout(12), dout_13 => h_dout(13), dout_14 => h_dout(14), dout_15 => h_dout(15),
                  valid_o => h_valid_o, ready_i => '1');

    tr : entity work.transform_engine
        generic map (W => 20, DIR => "both")
        port map (clk => clk, rst_n => rst_n, mode_i => t_mode_q,
                  din_0 => t_din(0), din_1 => t_din(1), din_2 => t_din(2), din_3 => t_din(3),
                  din_4 => t_din(4), din_5 => t_din(5), din_6 => t_din(6), din_7 => t_din(7),
                  din_8 => t_din(8), din_9 => t_din(9), din_10 => t_din(10), din_11 => t_din(11),
                  din_12 => t_din(12), din_13 => t_din(13), din_14 => t_din(14), din_15 => t_din(15),
                  valid_i => t_valid_i, ready_o => open,
                  dout_0 => t_dout(0), dout_1 => t_dout(1), dout_2 => t_dout(2), dout_3 => t_dout(3),
                  dout_4 => t_dout(4), dout_5 => t_dout(5), dout_6 => t_dout(6), dout_7 => t_dout(7),
                  dout_8 => t_dout(8), dout_9 => t_dout(9), dout_10 => t_dout(10), dout_11 => t_dout(11),
                  dout_12 => t_dout(12), dout_13 => t_dout(13), dout_14 => t_dout(14), dout_15 => t_dout(15),
                  valid_o => t_valid_o, ready_i => '1');

    qu : entity work.quant_engine
        port map (clk => clk, rst_n => rst_n, mode_i => q_mode, qp_i => q_qp,
                  din_0 => q_din(0), din_1 => q_din(1), din_2 => q_din(2), din_3 => q_din(3),
                  din_4 => q_din(4), din_5 => q_din(5), din_6 => q_din(6), din_7 => q_din(7),
                  din_8 => q_din(8), din_9 => q_din(9), din_10 => q_din(10), din_11 => q_din(11),
                  din_12 => q_din(12), din_13 => q_din(13), din_14 => q_din(14), din_15 => q_din(15),
                  valid_i => q_valid_i, ready_o => open,
                  dout_0 => q_dout(0), dout_1 => q_dout(1), dout_2 => q_dout(2), dout_3 => q_dout(3),
                  dout_4 => q_dout(4), dout_5 => q_dout(5), dout_6 => q_dout(6), dout_7 => q_dout(7),
                  dout_8 => q_dout(8), dout_9 => q_dout(9), dout_10 => q_dout(10), dout_11 => q_dout(11),
                  dout_12 => q_dout(12), dout_13 => q_dout(13), dout_14 => q_dout(14), dout_15 => q_dout(15),
                  valid_o => q_valid_o, ready_i => '1',
                  deq_0 => q_deq(0), deq_1 => q_deq(1), deq_2 => q_deq(2), deq_3 => q_deq(3),
                  deq_4 => q_deq(4), deq_5 => q_deq(5), deq_6 => q_deq(6), deq_7 => q_deq(7),
                  deq_8 => q_deq(8), deq_9 => q_deq(9), deq_10 => q_deq(10), deq_11 => q_deq(11),
                  deq_12 => q_deq(12), deq_13 => q_deq(13), deq_14 => q_deq(14), deq_15 => q_deq(15),
                  deq_valid_o => q_deq_valid);

    co : entity work.cavlc_cost_engine_ll
        port map (clk => clk, rst_n => rst_n, n_coefs_i => c_n, nC_i => (others => '0'),
                  levels_i => c_levels, valid_i => c_valid_i, ready_o => open, bits_o => c_bits,
                  valid_o => c_valid_o, ready_i => '1');

    rc : entity work.recon_engine
        generic map (RES_W => 20, WITH_SSD => true)
        port map (clk => clk, rst_n => rst_n, pred_i => r_pred_q, res_i => r_res_q, src_i => r_src_q,
                  valid_i => r_valid_q, ready_o => open, recon_o => r_recon, ssd_o => r_ssd,
                  valid_o => r_valid_o, ready_i => '1');

    rec_in_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_pred_q <= r_pred; r_src_q <= r_src; r_res_q <= r_res;
            r_valid_q <= r_valid_i; r_tag_q <= r_tag_in;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Heads: predictor inputs (4x4: screen + chain; background: 16x16, chroma)
    ------------------------------------------------------------------
    p4_valid_i  <= issuing when (head_tag.op = OP_SCR4 or head_tag.op = OP_FULL4) else '0';
    p4_mode     <= to_unsigned(head_tag.mode, 4);
    p16_valid_i <= bissuing when (bhead_tag.op = OP_SCR16 or bhead_tag.op = OP_FWD16) else '0';
    p16_mode    <= to_unsigned(bhead_tag.mode, 2);
    p16_blk     <= to_unsigned(bhead_tag.blk, 4);
    pc_valid_i  <= bissuing when (bhead_tag.op = OP_SCRC or bhead_tag.op = OP_FWDC) else '0';
    pc_mode     <= to_unsigned(bhead_tag.mode, 2);
    pc_blk      <= to_unsigned(bhead_tag.blk, 2);

    ------------------------------------------------------------------
    -- Tag delay lines (free-running)
    ------------------------------------------------------------------
    tags_p : process(clk)
    begin
        if rising_edge(clk) then
            if p4_valid_i = '1' then tp4(1) <= head_tag; else tp4(1) <= TAG_NONE; end if;
            tp4(2) <= tp4(1);
            if p16_valid_i = '1' then tp16(1) <= bhead_tag; else tp16(1) <= TAG_NONE; end if;
            tp16(2 to 8) <= tp16(1 to 7);
            if pc_valid_i = '1' then tpc(1) <= bhead_tag; else tpc(1) <= TAG_NONE; end if;
            tpc(2 to 6) <= tpc(1 to 5);
            if h_valid_i = '1' then th(1) <= h_tag_q; else th(1) <= TAG_NONE; end if;
            th(2) <= th(1);
            if t_valid_i = '1' then tt(1) <= t_tag_q; else tt(1) <= TAG_NONE; end if;
            tt(2) <= tt(1);
            if q_valid_i = '1' then tq(1) <= q_tag_in; else tq(1) <= TAG_NONE; end if;
            tq(2 to 5) <= tq(1 to 4);
            if c_valid_i = '1' then tc(1) <= tq(4); else tc(1) <= TAG_NONE; end if;
            tc(2 to 5) <= tc(1 to 4);
            if r_valid_q = '1' then trc(1) <= r_tag_q; else trc(1) <= TAG_NONE; end if;
            trc(2 to 3) <= trc(1 to 2);
        end if;
    end process;

    ------------------------------------------------------------------
    -- Residual stages: subtract the source block from the prediction,
    -- keep the prediction for the reconstruction later.
    ------------------------------------------------------------------
    src_addr_p : process(all)
        variable tg : tag_t;
    begin
        src_ra_r4 <= tp4(2).blk;
        tg := TAG_NONE;
        if p16_valid_o = '1' then tg := tp16(8);
        elsif pc_valid_o = '1' then tg := tpc(6);
        end if;
        case tg.plane is
            when 0 => src_ra_r16 <= tg.blk;
            when 1 => src_ra_r16 <= 16 + (tg.blk mod 4);
            when others => src_ra_r16 <= 20 + (tg.blk mod 4);
        end case;
        case tt(2).plane is
            when 0 => src_ra_rec <= tt(2).blk;
            when 1 => src_ra_rec <= 16 + (tt(2).blk mod 4);
            when others => src_ra_rec <= 20 + (tt(2).blk mod 4);
        end case;
    end process;

    res_p : process(clk)
        variable tg : tag_t;
        variable pr : px128;
        variable sb : px128;
    begin
        if rising_edge(clk) then
            -- synthesis translate_off
            assert (p16_valid_o = '0' or pc_valid_o = '0') report "16x16/chroma residual merge collision" severity failure;
            -- synthesis translate_on
            -- A: 4x4 predictor
            tg := TAG_NONE; pr := (others => '0');
            if p4_valid_o = '1' then tg := tp4(2); pr := p4_pred; end if;
            sb := src_rd_r4;
            for k in 0 to 15 loop
                res4_q(k) <= resize(signed('0' & byte_of(sb, k)), 20) - resize(signed('0' & byte_of(pr, k)), 20);
            end loop;
            res4_tag  <= tg;
            res4_pred <= pr;
            -- B: 16x16 / chroma predictors
            tg := TAG_NONE; pr := (others => '0');
            if p16_valid_o = '1' then tg := tp16(8); pr := p16_pred;
            elsif pc_valid_o = '1' then tg := tpc(6); pr := pc_pred;
            end if;
            sb := src_rd_r16;
            for k in 0 to 15 loop
                res16_q(k) <= resize(signed('0' & byte_of(sb, k)), 20) - resize(signed('0' & byte_of(pr, k)), 20);
            end loop;
            res16_tag  <= tg;
            res16_pred <= pr;
        end if;
    end process;

    -- source block RAM: loaded from the 24-word stream after start
    src_wr : process(clk)
    begin
        if rising_edge(clk) then
            if src_ld < 24 then
                src_mem(src_ld) <= src_data_i;
            end if;
        end if;
    end process;
    src_rd_r4  <= src_mem(src_ra_r4);
    src_rd_r16 <= src_mem(src_ra_r16);
    src_rd_rec <= src_mem(src_ra_rec);
    src_rd_nb  <= src_mem(src_ra_nb);

    -- prediction stores: written for anything that will be reconstructed
    pred_wr : process(clk)
    begin
        if rising_edge(clk) then
            if res4_tag.valid = '1' and res4_tag.op = OP_FULL4 then
                pred4_mem(res4_tag.sub * 3 + res4_tag.slot) <= res4_pred;
            end if;
            if res16_tag.valid = '1' and (res16_tag.op = OP_FWD16 or res16_tag.op = OP_FWDC) then
                pred16_mem(pred16_slot(res16_tag)) <= res16_pred;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Hadamard lane input: 4x4 screen from residual A, 16x16 screen from
    -- residual B (selects registered a cycle ahead)
    ------------------------------------------------------------------
    h_valid_i <= h_sel4_q or h_sel16_q;
    h_in_p : process(all)
    begin
        for k in 0 to 15 loop
            if h_sel4_q = '1' then h_din(k) <= resize(res4_q(k), 32);
            else h_din(k) <= resize(res16_q(k), 32);
            end if;
        end loop;
    end process;

    ------------------------------------------------------------------
    -- Transform input mux: loop-back (DC inverse) / fused dequant (4x4
    -- candidate inverse) / DC vectors / residual A (4x4 candidates) /
    -- residual B (16x16 and chroma forward). The select, mode and tag
    -- were registered a cycle ahead (main_p).
    ------------------------------------------------------------------
    t_valid_i <= t_sel_lb_q or t_sel_dq_q or t_sel_dc_q or t_sel_r4_q or t_sel_r16_q;

    t_in_p : process(all)
    begin
        for k in 0 to 15 loop t_din(k) <= (others => '0'); end loop;
        if t_sel_lb_q = '1' then
            for k in 0 to 15 loop t_din(k) <= q_dout(k); end loop;
            if t_dc0_en_q = '1' then t_din(0) <= t_dc0_q; end if;
        elsif t_sel_dq_q = '1' then
            for k in 0 to 15 loop t_din(k) <= q_deq(k); end loop;
        elsif t_sel_dc_q = '1' then
            for k in 0 to 15 loop t_din(k) <= resize(t_dcv_q(k), 32); end loop;
        elsif t_sel_r4_q = '1' then
            for k in 0 to 15 loop t_din(k) <= resize(res4_q(k), 32); end loop;
        elsif t_sel_r16_q = '1' then
            for k in 0 to 15 loop t_din(k) <= resize(res16_q(k), 32); end loop;
        end if;
    end process;

    -- synthesis translate_off
    t_chk_p : process(clk)
        variable raw_lb, raw_dq, raw_dc, raw_r4, raw_r16, raw_h4, raw_h16 : std_logic;
        variable n : integer;
    begin
        if rising_edge(clk) then
            raw_lb := q_valid_o and tq(4).inv;
            raw_dq := q_deq_valid and bit_of(tq(5).op = OP_FULL4);
            if raw_dq = '1' then n := 1; else n := 0; end if;
            assert t_sel_dq_q = raw_dq report "transform dequant select misaligned" severity failure;
            raw_dc := bit_of(bissuing = '1' and (bhead_tag.op = OP_HAD16 or bhead_tag.op = OP_HADC));
            raw_r4 := bit_of(res4_tag.valid = '1' and res4_tag.op = OP_FULL4);
            raw_r16 := bit_of(res16_tag.valid = '1' and (res16_tag.op = OP_FWD16 or res16_tag.op = OP_FWDC or (res16_tag.op = OP_SCR16 and res16_tag.slot = 0)));
            if raw_lb = '1' then n := n + 1; end if;
            if raw_dc = '1' then n := n + 1; end if;
            if raw_r4 = '1' then n := n + 1; end if;
            if raw_r16 = '1' then n := n + 1; end if;
            assert n <= 1 report "transform input collision" severity failure;
            assert t_sel_lb_q = raw_lb report "transform loop-back select misaligned" severity failure;
            assert t_sel_dc_q = raw_dc report "transform DC select misaligned" severity failure;
            assert t_sel_r4_q = raw_r4 report "transform residual-A select misaligned" severity failure;
            assert t_sel_r16_q = raw_r16 report "transform residual-B select misaligned" severity failure;
            raw_h4 := bit_of(res4_tag.valid = '1' and res4_tag.op = OP_SCR4);
            raw_h16 := bit_of(res16_tag.valid = '1' and res16_tag.op = OP_SCR16 and res16_tag.slot = 1);
            assert not (raw_h4 = '1' and raw_h16 = '1') report "Hadamard lane collision" severity failure;
            assert h_sel4_q = raw_h4 report "Hadamard select A misaligned" severity failure;
            assert h_sel16_q = raw_h16 report "Hadamard select B misaligned" severity failure;
        end if;
    end process;
    -- synthesis translate_on

    ------------------------------------------------------------------
    -- Quant input mux: forward from the transform, inverse from the
    -- loop-back register or the level store (background head)
    ------------------------------------------------------------------
    lvl_rd  <= lvl_mem(lvl_ra);
    lvl4_rd <= lvl4_mem(lvl4_ra);

    q_in_p : process(all)
        variable tt2 : tag_t;
        variable fwd_ok : std_logic;
        variable inv_hd : std_logic;
        variable lv : vec16_t;
    begin
        tt2 := tt(2);
        fwd_ok := '0';
        if t_valid_o = '1' and tt2.inv = '0' and
           (tt2.op = OP_FULL4 or tt2.op = OP_FWD16 or tt2.op = OP_FWDC or tt2.op = OP_HAD16 or tt2.op = OP_HADC) then
            fwd_ok := '1';
        end if;
        inv_hd := bit_of(bissuing = '1' and (bhead_tag.op = OP_INV16 or bhead_tag.op = OP_INVC));
        -- synthesis translate_off
        assert (lb_tag.valid = '0' or inv_hd = '0') and (lb_tag.valid = '0' or fwd_ok = '0') and (inv_hd = '0' or fwd_ok = '0')
            report "quant input collision" severity failure;
        -- synthesis translate_on
        q_valid_i <= '0';
        q_mode    <= "000";
        q_tag_in  <= TAG_NONE;
        q_qp      <= qp_y;
        for k in 0 to 15 loop q_din(k) <= (others => '0'); end loop;
        if lb_tag.valid = '1' then
            q_valid_i <= '1';
            q_tag_in  <= lb_tag;
            for k in 0 to 15 loop q_din(k) <= lb_data(k); end loop;
            case lb_tag.op is
                when OP_HAD16 => q_mode <= "011";
                when OP_HADC  => q_mode <= "101";
                when others   => q_mode <= "001";
            end case;
            if lb_tag.plane /= 0 then q_qp <= qp_c; end if;
        elsif inv_hd = '1' then
            q_valid_i <= '1';
            q_tag_in  <= bhead_tag;
            lv := unpack13_v(lvl_rd);
            for k in 0 to 15 loop q_din(k) <= lv(k); end loop;
            q_mode <= "001";
            if bhead_tag.plane /= 0 then q_qp <= qp_c; end if;
        elsif fwd_ok = '1' then
            q_valid_i <= '1';
            q_tag_in  <= tt2;
            for k in 0 to 15 loop q_din(k) <= t_dout(k); end loop;
            case tt2.op is
                when OP_HAD16 => q_mode <= "010";
                when OP_HADC  => q_mode <= "100";
                when others   => q_mode <= "000";
            end case;
            if tt2.op = OP_FWD16 or tt2.op = OP_FWDC then q_din(0) <= (others => '0'); end if;
            if tt2.plane /= 0 then q_qp <= qp_c; end if;
        end if;
    end process;

    -- level-store read address: inverse beats from the background head, else the streamer
    lvl_ra <= lvl_addr(bhead_tag) when (bissuing = '1' and (bhead_tag.op = OP_INV16 or bhead_tag.op = OP_INVC))
              else lvl_ra_stream;

    ------------------------------------------------------------------
    -- Quant output: level store, cost engine feed, loop-back register
    ------------------------------------------------------------------
    cost_in_p : process(all)
        variable qt : tag_t;
    begin
        qt := tq(4);
        c_valid_i <= '0';
        c_n <= to_unsigned(16, 5);
        for k in 0 to 15 loop c_levels(k) <= (others => '0'); end loop;
        if q_valid_o = '1' and qt.inv = '0' then
            if qt.op = OP_FULL4 or qt.op = OP_HAD16 then
                c_valid_i <= '1';
                for k in 0 to 15 loop c_levels(k) <= resize(q_dout(ZZ(k)), 16); end loop;
            elsif qt.op = OP_FWD16 then
                c_valid_i <= '1';
                c_n <= to_unsigned(15, 5);
                for k in 0 to 14 loop c_levels(k) <= resize(q_dout(ZZ(k + 1)), 16); end loop;
            end if;
        end if;
    end process;

    -- background level store: one write site (quant forward results)
    lvl_wr_p : process(all)
        variable qt : tag_t;
    begin
        qt := tq(4);
        lvl_we <= '0'; lvl_wa <= 0; lvl_wd <= pack13(q_dout);
        if q_valid_o = '1' and qt.inv = '0' and
           (qt.op = OP_FWD16 or qt.op = OP_HAD16 or qt.op = OP_FWDC or qt.op = OP_HADC) then
            lvl_we <= '1';
            lvl_wa <= lvl_addr(qt);
        end if;
    end process;

    lvl_mem_p : process(clk)
    begin
        if rising_edge(clk) then
            if lvl_we = '1' then lvl_mem(lvl_wa) <= lvl_wd; end if;
        end if;
    end process;

    -- I_4x4 level store: written at commit into this MB's bank
    lvl4_mem_p : process(clk)
    begin
        if rising_edge(clk) then
            if st = S_I4_COMMIT then
                if lvl4_bank = '1' then lvl4_mem(16 + cur_blk(cc)) <= cand_lvl_rd;
                else lvl4_mem(cur_blk(cc)) <= cand_lvl_rd;
                end if;
            end if;
        end if;
    end process;

    -- candidate stores (I4 shortlist): written by the quant / recon results,
    -- read once at commit
    cand_wr_p : process(clk)
    begin
        if rising_edge(clk) then
            if q_valid_o = '1' and tq(4).inv = '0' and tq(4).op = OP_FULL4 then
                cand_lvl(tq(4).sub * 3 + tq(4).slot) <= pack13(q_dout);
            end if;
            if r_valid_o = '1' and trc(3).op = OP_FULL4 then
                cand_rec(trc(3).sub * 3 + trc(3).slot) <= r_recon;
            end if;
        end if;
    end process;
    cand_ra <= cc * 3 + best_slot(cc);
    cand_lvl_rd <= cand_lvl(cand_ra);
    cand_rec_rd <= cand_rec(cand_ra);

    -- I4 reconstruction buffer: one write site (commit), several read ports
    r4_wr_p : process(clk)
    begin
        if rising_edge(clk) then
            if st = S_I4_COMMIT then r4(cur_blk(cc)) <= cand_rec_rd; end if;
        end if;
    end process;
    r4_stream <= r4(o_item mod 16);

    -- screen rank stores: finished top-4 list per block, raw SATD per (block, mode)
    scr_wr_p : process(clk)
        variable l : sklist_t;
    begin
        if rising_edge(clk) then
            if sk_wr = '1' then
                l := (others => '0');
                for i in 0 to 3 loop
                    l(28 * i + 23 downto 28 * i) := std_logic_vector(sk_cost(sk_wr_par * 4 + i));
                    l(28 * i + 27 downto 28 * i + 24) := std_logic_vector(to_unsigned(sk_mode(sk_wr_par * 4 + i), 4));
                end loop;
                l(114 downto 112) := std_logic_vector(to_unsigned(sk_n(sk_wr_par), 3));
                scr_top(sk_wr_blk) <= l;
            end if;
            if ab2_tag.valid = '1' and ab2_tag.op = OP_SCR4 then
                scr_raw(ab2_tag.blk * 16 + ab2_tag.mode) <= std_logic_vector(ab2);
            end if;
        end if;
    end process;
    scr_top_ra <= cur_blk(cc);
    scr_top_rd <= scr_top(scr_top_ra);
    scr_raw_ra <= cur_blk(cc) * 16 + pred_mode(cc);
    scr_raw_rd <= scr_raw(scr_raw_ra);

    ------------------------------------------------------------------
    -- Recon input: inverse transform output + stored prediction
    ------------------------------------------------------------------
    rec_in_p : process(all)
        variable t2 : tag_t;
    begin
        t2 := tt(2);
        r_valid_i <= '0';
        r_tag_in  <= TAG_NONE;
        pred4_ra  <= 0;
        pred16_ra <= 0;
        r_pred    <= (others => '0');
        r_src     <= (others => '0');
        for k in 0 to 15 loop
            r_res(20 * k + 19 downto 20 * k) <= std_logic_vector(t_dout(k)(19 downto 0));
        end loop;
        if t_valid_o = '1' and t2.inv = '1' and (t2.op = OP_FULL4 or t2.op = OP_INV16 or t2.op = OP_INVC) then
            r_valid_i <= '1';
            r_tag_in  <= t2;
            r_src     <= src_rd_rec;
            if t2.op = OP_FULL4 then
                pred4_ra <= t2.sub * 3 + t2.slot;
                r_pred   <= pred4_mem(t2.sub * 3 + t2.slot);
            else
                pred16_ra <= pred16_slot(t2);
                r_pred    <= pred16_mem(pred16_slot(t2));
            end if;
        end if;
    end process;

    -- recon store (I16 luma, chroma) with one write site
    rec_wr_p : process(all)
        variable t3 : tag_t;
    begin
        t3 := trc(3);
        rec_we <= '0'; rec_wa <= 0; rec_wd <= r_recon;
        if r_valid_o = '1' and t3.op = OP_INV16 then
            rec_we <= '1'; rec_wa <= t3.blk;
        elsif r_valid_o = '1' and t3.op = OP_INVC then
            rec_we <= '1'; rec_wa <= 16 + (t3.plane - 1) * 4 + t3.blk;
        end if;
    end process;

    rec_mem_p : process(clk)
    begin
        if rising_edge(clk) then
            if rec_we = '1' then rec_mem(rec_wa) <= rec_wd; end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- abs-sum trees: A = 4x4 SATD from the Hadamard lane; B = 16x16 SATD
    -- from the transform (Hadamard mode) and chroma SAD from residual B
    ------------------------------------------------------------------
    ab_p : process(clk)
        variable s : u24;
        variable v : s20;
    begin
        if rising_edge(clk) then
            ab1_tag <= TAG_NONE;
            if h_valid_o = '1' and th(2).op = OP_SCR4 then
                ab1_tag <= th(2);
                for k in 0 to 15 loop
                    v := h_dout(k)(19 downto 0);
                    if v < 0 then ab1(k) <= unsigned(-v); else ab1(k) <= unsigned(v); end if;
                end loop;
            end if;
            s := (others => '0');
            for k in 0 to 15 loop s := s + resize(ab1(k), 24); end loop;
            ab2     <= s;
            ab2_tag <= ab1_tag;
        end if;
    end process;

    bb_p : process(clk)
        variable s : u24;
        variable v : s20;
        variable from_t, from_h, from_r : boolean;
    begin
        if rising_edge(clk) then
            from_t := t_valid_o = '1' and tt(2).inv = '0' and tt(2).op = OP_SCR16;
            from_h := h_valid_o = '1' and th(2).op = OP_SCR16;
            from_r := res16_tag.valid = '1' and res16_tag.op = OP_SCRC;
            -- synthesis translate_off
            assert not ((from_t and from_r) or (from_t and from_h) or (from_h and from_r)) report "abs-sum tree B collision" severity failure;
            -- synthesis translate_on
            bb1_tag <= TAG_NONE;
            if from_t then
                bb1_tag <= tt(2);
                for k in 0 to 15 loop
                    v := t_dout(k)(19 downto 0);
                    if v < 0 then bb1(k) <= unsigned(-v); else bb1(k) <= unsigned(v); end if;
                end loop;
            elsif from_h then
                bb1_tag <= th(2);
                for k in 0 to 15 loop
                    v := h_dout(k)(19 downto 0);
                    if v < 0 then bb1(k) <= unsigned(-v); else bb1(k) <= unsigned(v); end if;
                end loop;
            elsif from_r then
                bb1_tag <= res16_tag;
                for k in 0 to 15 loop
                    v := res16_q(k);
                    if v < 0 then bb1(k) <= unsigned(-v); else bb1(k) <= unsigned(v); end if;
                end loop;
            end if;
            s := (others => '0');
            for k in 0 to 15 loop s := s + resize(bb1(k), 24); end loop;
            bb2     <= s;
            bb2_tag <= bb1_tag;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Result collectors and the sequencers
    ------------------------------------------------------------------
    main_p : process(clk, rst_n)
        variable qt, t2, ct, rt : tag_t;
        variable c : u24;
        variable p : integer range 0 to 4;
        variable ok : boolean;
        variable br, bc, s : integer range 0 to 15;
        variable jv : unsigned(27 downto 0);
        variable jd : signed(28 downto 0);
        variable mt, ml : integer range 0 to 15;
        variable tok, lok : boolean;
        variable at, al : std_logic;
        variable topv : std_logic_vector(63 downto 0);
        variable leftv : std_logic_vector(31 downto 0);
        variable tlv : std_logic_vector(7 downto 0);
        variable issue, bissue : boolean;
        variable ht, bht, nt4, nt16 : tag_t;
        variable kc : integer range 0 to 2;
        variable sidx : integer range -1 to 15;
        variable vr, vh, vt, vq, va, vb : sbv_t;
        variable vr_i, vh_i, vt_i, vq_i, vb_i : sbv_t;   -- registered view for the background (its slots never coincide with a same-cycle foreground issue)
        variable fb : integer range 0 to 63;
        variable bl : integer range 0 to 15;   -- candidate issues still to come in the next burst
        variable cand6 : boolean;              -- a candidate will own the 4x4 head six cycles from now
        variable qcnt : u5;
        variable cand_ok : boolean;
        variable lm : integer range 0 to 8;
        variable l : sklist_t;
        variable lc : rc3_t;
        variable lmd : rm3_t;
        variable ln : integer range 0 to 7;
        variable pk : u24;
        variable pos : integer range 0 to 3;
        variable pp : integer range 0 to 1;
        variable im1 : integer range 0 to 2;
    begin
        if rst_n = '0' then
            st <= S_IDLE; bst <= B_IDLE;
            issuing <= '0'; head_tag <= TAG_NONE; bissuing <= '0'; bhead_tag <= TAG_NONE;
            done_q <= '0'; o_start <= '0';
            lb_tag <= TAG_NONE; src_ld <= 24;
            sb_r <= (others => '0'); sb_h <= (others => '0'); sb_t <= (others => '0');
            sb_q <= (others => '0'); sb_a <= (others => '0'); sb_b <= (others => '0');
            t_sel_lb_q <= '0'; t_sel_dq_q <= '0'; t_sel_dc_q <= '0'; t_sel_r4_q <= '0'; t_sel_r16_q <= '0'; t_dc0_en_q <= '0';
            t_mode_q <= (others => '0'); t_tag_q <= TAG_NONE;
            h_sel4_q <= '0'; h_sel16_q <= '0'; h_tag_q <= TAG_NONE;
            lvl4_bank <= '0';
            sp_idx <= 16; si_idx <= 16; sp_step <= 0; sm <= 0; nbs_ready <= "00"; sk_wr <= '0';
            scr_done <= (others => '0'); rk_ready <= "00";
        elsif rising_edge(clk) then
            done_q <= '0'; o_start <= '0';
            issuing <= '0'; bissuing <= '0';
            sk_wr <= '0';
            if src_ld < 24 then src_ld <= src_ld + 1; end if;
            ht := TAG_NONE; issue := false;
            bht := TAG_NONE; bissue := false;
            -- scoreboard as it will read next cycle (before this cycle's issues)
            vr := sb_r(1 to 15) & '0';
            vh := sb_h(1 to 15) & '0';
            vt := sb_t(1 to 15) & '0';
            vq := sb_q(1 to 15) & '0';
            va := sb_a(1 to 15) & '0';
            vb := sb_b(1 to 15) & '0';
            vr_i := vr; vh_i := vh; vt_i := vt; vq_i := vq; vb_i := vb;
            fb := FB_NONE; bl := 0;
            cand_ok := false;

            ----------------------------------------------------------
            -- loop-back register: quant forward output of HAD16 / HADC goes
            -- back into the quantizer as an inverse beat (4x4 candidates use
            -- the fused dequant instead)
            ----------------------------------------------------------
            qt := tq(4);
            rt := trc(3);
            lb_tag <= TAG_NONE;
            if q_valid_o = '1' and qt.inv = '0' and (qt.op = OP_HAD16 or qt.op = OP_HADC) then
                lb_tag <= qt; lb_tag.inv <= '1';
                lb_data <= q_dout;
            end if;

            ----------------------------------------------------------
            -- quant forward results: candidate levels / nz flags / counts
            ----------------------------------------------------------
            if q_valid_o = '1' and qt.inv = '0' then
                qcnt := popnz(q_dout);
                case qt.op is
                    when OP_FULL4 =>
                        cand_nz(qt.sub * 3 + qt.slot)  <= any_nz(q_dout, 0);
                        cand_cnt(qt.sub * 3 + qt.slot) <= qcnt;
                    when OP_FWD16 =>
                        nz16(qt.blk) <= any_nz(q_dout, 1);
                        tcnt16(qt.blk) <= qcnt;
                    when OP_FWDC  =>
                        cac_nz <= cac_nz or any_nz(q_dout, 1);
                        tcntc((qt.plane - 1) * 4 + qt.blk) <= qcnt;
                    when OP_HADC  => cdc_nz <= cdc_nz or any_nz(q_dout, 0);
                    when others => null;
                end case;
            end if;

            ----------------------------------------------------------
            -- transform results: DC capture (forward), DC recon (inverse)
            ----------------------------------------------------------
            t2 := tt(2);
            -- synthesis translate_off
            if DEBUG and issuing = '1' then
                report "ISSUE op=" & op_t'image(head_tag.op) & " blk=" & integer'image(head_tag.blk) & " mode=" & integer'image(head_tag.mode) &
                       " slot=" & integer'image(head_tag.slot) & " sub=" & integer'image(head_tag.sub) severity note;
            end if;
            if DEBUG and bissuing = '1' then
                report "BISSUE op=" & op_t'image(bhead_tag.op) & " blk=" & integer'image(bhead_tag.blk) & " mode=" & integer'image(bhead_tag.mode) &
                       " plane=" & integer'image(bhead_tag.plane) severity note;
            end if;
            -- synthesis translate_on
            if t_valid_o = '1' then
                if t2.inv = '0' and t2.op = OP_FWD16 then dc16(t2.blk) <= t_dout(0)(15 downto 0); end if;
                if t2.inv = '0' and t2.op = OP_FWDC then dcc(t2.plane - 1)(t2.blk) <= t_dout(0)(15 downto 0); end if;
                if t2.inv = '1' and t2.op = OP_HAD16 then
                    for k in 0 to 15 loop dcrec16(k) <= t_dout(k)(19 downto 0); end loop;
                end if;
                if t2.inv = '1' and t2.op = OP_HADC then
                    for k in 0 to 3 loop dccrec(t2.plane - 1)(k) <= t_dout(k)(19 downto 0); end loop;
                end if;
            end if;

            ----------------------------------------------------------
            -- cost results; the candidate's rate term starts its multiply
            -- here (two cycles ahead of its SSD)
            ----------------------------------------------------------
            ct := tc(5);
            if c_valid_o = '1' then
                if ct.op = OP_FULL4 then cand_bits(ct.sub * 3 + ct.slot) <= c_bits;
                else bits_a <= bits_a + resize(c_bits, 16);
                end if;
            end if;
            jb_q <= to_unsigned(to_integer(c_bits) + cand_mbits(ct.sub * 3 + ct.slot), 12);
            jp1 <= resize(lam * jb_q, 28);

            ----------------------------------------------------------
            -- recon results: J = 16*SSD + rate, compared as the SSD lands
            ----------------------------------------------------------
            if r_valid_o = '1' and rt.op = OP_FULL4 then
                jv := resize(r_ssd & "0000", 28) + jp1;
                -- jv < best as one three-operand subtraction (single carry chain)
                jd := signed('0' & resize(r_ssd & "0000", 28)) + signed('0' & jp1) - signed('0' & best_j(rt.sub));
                if jd < 0 then
                    best_j(rt.sub) <= jv; best_slot(rt.sub) <= rt.slot;
                end if;
            end if;

            ----------------------------------------------------------
            -- SATD / SAD results
            ----------------------------------------------------------
            if bb2_tag.valid = '1' then
                case bb2_tag.op is
                    when OP_SCR16 => acc16(bb2_tag.mode) <= acc16(bb2_tag.mode) + bb2;
                    when OP_SCRC  => accc(bb2_tag.mode)  <= accc(bb2_tag.mode)  + bb2;
                    when others => null;
                end case;
            end if;
            if ab2_tag.valid = '1' then
                case ab2_tag.op is
                    when OP_SCR4 =>
                        -- raw SATD into the block's top-4 list (ties after equal costs)
                        pp := ab2_tag.slot mod 2;
                        c := ab2;
                        p := 0;
                        if sk_n(pp) > 0 and sk_cost(pp * 4 + 0) <= c then p := 1; end if;
                        if sk_n(pp) > 1 and sk_cost(pp * 4 + 1) <= c then p := 2; end if;
                        if sk_n(pp) > 2 and sk_cost(pp * 4 + 2) <= c then p := 3; end if;
                        if sk_n(pp) > 3 and sk_cost(pp * 4 + 3) <= c then p := 4; end if;
                        if p < 4 then
                            if p <= 2 then sk_cost(pp * 4 + 3) <= sk_cost(pp * 4 + 2); sk_mode(pp * 4 + 3) <= sk_mode(pp * 4 + 2); end if;
                            if p <= 1 then sk_cost(pp * 4 + 2) <= sk_cost(pp * 4 + 1); sk_mode(pp * 4 + 2) <= sk_mode(pp * 4 + 1); end if;
                            if p = 0 then sk_cost(pp * 4 + 1) <= sk_cost(pp * 4 + 0); sk_mode(pp * 4 + 1) <= sk_mode(pp * 4 + 0); end if;
                            sk_cost(pp * 4 + p) <= c; sk_mode(pp * 4 + p) <= ab2_tag.mode;
                            if sk_n(pp) < 4 then sk_n(pp) <= sk_n(pp) + 1; end if;
                        end if;
                        if ab2_tag.inv = '1' then
                            -- last mode of the block: publish the list next cycle
                            sk_wr <= '1'; sk_wr_par <= pp; sk_wr_blk <= ab2_tag.blk;
                        end if;
                    when others => null;
                end case;
            end if;
            if sk_wr = '1' then
                scr_done(sk_wr_blk) <= '1';
                sk_n(sk_wr_par) <= 0;
            end if;

            ----------------------------------------------------------
            -- chain sequencer: the I_4x4 wavefront
            ----------------------------------------------------------
            case st is
                when S_IDLE =>
                    if start_i = '1' and ost /= S_OREC then
                        qp_y <= qp_y_i; qp_c <= qp_c_i;
                        src_ld <= 0;
                        top_y <= top_y_i; left_y <= left_y_i; tr_y <= tr_y_i; tl_y <= tl_y_i;
                        top_u <= top_u_i; left_u <= left_u_i; tl_u <= tl_u_i;
                        top_v <= top_v_i; left_v <= left_v_i; tl_v <= tl_v_i;
                        a_top <= avail_top_i; a_left <= avail_left_i; a_tl <= avail_tl_i; a_tr <= avail_tr_i;
                        m4top <= mode4_top_i; m4left <= mode4_left_i;
                        pen1 <= to_unsigned((RD_SLAM16(to_integer(qp_y_i)) + 8) / 16, 12);
                        pen4 <= to_unsigned((RD_SLAM16(to_integer(qp_y_i)) * 4 + 8) / 16, 12);
                        lam  <= to_unsigned(RD_LAM16(to_integer(qp_y_i)), 16);
                        bits_a <= to_unsigned(10, 16);
                        bits_b <= to_unsigned(43, 16);
                        acc16 <= (others => (others => '0'));
                        accc  <= (others => (others => '0'));
                        nz16 <= (others => '0'); nz4 <= (others => '0');
                        cdc_nz <= '0'; cac_nz <= '0';
                        modes4 <= (others => '0');
                        step <= 0; cc <= 0; ck <= 0; ci <= 0; rk_ready <= "00";
                        bcm <= 0; bcb <= 0; bcp <= 1; bwait <= 0; bwaitc <= 0;
                        lvl4_bank <= not lvl4_bank;
                        -- screen from block 0
                        sp_idx <= 0; sp_step <= 0; si_idx <= 0; sm <= 0; nbs_ready <= "00";
                        scr_done <= (others => '0'); sk_n <= (others => 0);
                        st <= S_I4_PREP;
                        bst <= B_SCR16;
                    end if;

                when S_I4_PREP =>
                    -- true (reconstructed) neighbours of sub-block cc + predIntra4x4PredMode
                    sidx := STEP_TAB(step, cc);
                    if sidx < 0 then s := 0; else s := sidx; end if;
                    br := SCAN_BR(s); bc := SCAN_BC(s);
                    cur_blk(cc) <= br * 4 + bc;
                    at := '1'; al := '1';
                    if br = 0 then at := a_top; end if;
                    if bc = 0 then al := a_left; end if;
                    nb_at(cc) <= at; nb_al(cc) <= al;
                    if br > 0 then
                        topv(31 downto 0) := r4((br - 1) * 4 + bc)(127 downto 96);
                    else
                        topv(31 downto 0) := top_y(bc * 32 + 31 downto bc * 32);
                    end if;
                    if tr_avail_blk(s) and ((s /= 5) or a_tr = '1') and at = '1' then
                        if br > 0 then
                            topv(63 downto 32) := r4((br - 1) * 4 + bc + 1)(127 downto 96);
                        elsif s = 5 then
                            topv(63 downto 32) := tr_y;
                        else
                            topv(63 downto 32) := top_y((bc + 1) * 32 + 31 downto (bc + 1) * 32);
                        end if;
                    else
                        topv(63 downto 32) := topv(31 downto 24) & topv(31 downto 24) & topv(31 downto 24) & topv(31 downto 24);
                    end if;
                    if bc > 0 then
                        leftv := byte_of(r4(br * 4 + bc - 1), 15) & byte_of(r4(br * 4 + bc - 1), 11) &
                                 byte_of(r4(br * 4 + bc - 1), 7) & byte_of(r4(br * 4 + bc - 1), 3);
                    else
                        leftv := left_y(br * 32 + 31 downto br * 32);
                    end if;
                    if br > 0 and bc > 0 then tlv := byte_of(r4((br - 1) * 4 + bc - 1), 15);
                    elsif br > 0 then tlv := byte_of(left_y, br * 4 - 1);
                    elsif bc > 0 then tlv := byte_of(top_y, bc * 4 - 1);
                    else tlv := tl_y;
                    end if;
                    nb_top(cc) <= topv; nb_left(cc) <= leftv; nb_tl(cc) <= tlv;
                    if br > 0 then mt := to_integer(unsigned(modes4(4 * ((br - 1) * 4 + bc) + 3 downto 4 * ((br - 1) * 4 + bc)))); tok := true;
                    else mt := to_integer(unsigned(m4top(4 * bc + 3 downto 4 * bc))); tok := (a_top = '1');
                    end if;
                    if bc > 0 then ml := to_integer(unsigned(modes4(4 * (br * 4 + bc - 1) + 3 downto 4 * (br * 4 + bc - 1)))); lok := true;
                    else ml := to_integer(unsigned(m4left(4 * br + 3 downto 4 * br))); lok := (a_left = '1');
                    end if;
                    if not (tok and lok) then pred_mode(cc) <= 2;
                    elsif mt < ml then pred_mode(cc) <= mt;
                    else pred_mode(cc) <= ml;
                    end if;
                    if cc = 0 then
                        if STEP_TAB(step, 1) >= 0 then nsub <= 2; else nsub <= 1; end if;
                    end if;
                    st <= S_I4_RK1;
                    if cc = 0 then fb := 4; else fb := 1; cand_ok := true; end if;
                    if nsub = 2 or (cc = 0 and STEP_TAB(step, 1) >= 0) then bl := 7; else bl := 3; end if;

                when S_I4_RK1 =>
                    -- the block's screen list, minus the predicted mode
                    -- synthesis translate_off
                    if scr_done(cur_blk(cc)) = '0' then stat_rk <= stat_rk + 1; end if;
                    -- synthesis translate_on
                    if scr_done(cur_blk(cc)) = '1' then
                        l := scr_top_rd;
                        ln := to_integer(unsigned(l(114 downto 112)));
                        lc := (others => (others => '0')); lmd := (others => 0);
                        p := 0;
                        for i in 0 to 3 loop
                            if i < ln and to_integer(unsigned(l(28 * i + 27 downto 28 * i + 24))) /= pred_mode(cc) and p < 3 then
                                lc(p) := unsigned(l(28 * i + 23 downto 28 * i));
                                lmd(p) := to_integer(unsigned(l(28 * i + 27 downto 28 * i + 24)));
                                p := p + 1;
                            end if;
                        end loop;
                        rk1_cost <= lc; rk1_mode <= lmd; rk1_n <= p;
                        rk1_pcost <= unsigned(scr_raw_rd);
                        st <= S_I4_RK2;
                    end if;
                    if cc = 0 then fb := 3; else fb := 1; cand_ok := true; end if;
                    if nsub = 2 then bl := 7; else bl := 3; end if;

                when S_I4_RK2 =>
                    -- insert the predicted mode (penalty 1) against the rest (penalty 4);
                    -- same order as sorting the nine penalised costs with ties by mode
                    pk := rk1_pcost + resize(pen1, 24);
                    pos := rk1_n;
                    for i in 0 to 2 loop
                        if i < rk1_n and pos = rk1_n and before(pk, pred_mode(cc), rk1_cost(i) + resize(pen4, 24), rk1_mode(i)) then
                            pos := i;
                        end if;
                    end loop;
                    for i in 0 to 2 loop
                        im1 := 0;
                        if i > 0 then im1 := i - 1; end if;
                        if i < pos then rk_mode(cc * 3 + i) <= rk1_mode(i);
                        elsif i = pos then rk_mode(cc * 3 + i) <= pred_mode(cc);
                        elsif i > 0 and im1 < rk1_n then rk_mode(cc * 3 + i) <= rk1_mode(im1);
                        else rk_mode(cc * 3 + i) <= 2;
                        end if;
                    end loop;
                    if rk1_n + 1 < SHORTLIST then nfull(cc) <= rk1_n + 1; else nfull(cc) <= SHORTLIST; end if;
                    rk_ready(cc) <= '1';
                    if cc = 0 then
                        best_j <= (others => (others => '1')); best_slot <= (others => 0);
                        if nsub = 2 then cc <= 1; st <= S_I4_PREP; else st <= S_I4_FULL; end if;
                        fb := 2;
                    else
                        cc <= 0; st <= S_I4_FULL;
                        fb := 1; cand_ok := true;
                    end if;
                    if nsub = 2 then bl := 7; else bl := 3; end if;

                when S_I4_FULL =>
                    cand_ok := true;
                    fb := 1;
                    if nsub = 2 then bl := 7; else bl := 3; end if;
                    if ck < bl then bl := bl - ck; else bl := 0; end if;
                    if (nsub = 1 and ck >= nfull(0)) or (nsub = 2 and rk_ready(1) = '1' and ck >= nfull(0) + nfull(1)) then
                        cand_ok := false;
                        if wait_cnt = 0 then
                            cc <= 0; st <= S_I4_COMMIT;
                        else
                            wait_cnt <= wait_cnt - 1;
                            fb := wait_cnt + 6;
                        end if;
                    end if;

                when S_I4_COMMIT =>
                    modes4(4 * cur_blk(cc) + 3 downto 4 * cur_blk(cc)) <= std_logic_vector(to_unsigned(rk_mode(cc * 3 + best_slot(cc)), 4));
                    bits_b <= bits_b + resize(cand_bits(cc * 3 + best_slot(cc)), 16);
                    nz4(cur_blk(cc)) <= cand_nz(cc * 3 + best_slot(cc));
                    tcnt4(cur_blk(cc)) <= cand_cnt(cc * 3 + best_slot(cc));
                    fb := 5;
                    if cc = 0 and nsub = 2 then
                        cc <= 1;
                    elsif step = 9 then
                        st <= S_PICK;
                    else
                        step <= step + 1; cc <= 0; ck <= 0; ci <= 0; rk_ready <= "00"; st <= S_I4_PREP;
                    end if;

                when S_PICK =>
                    -- path pick by estimated bits, once the background chain and
                    -- the previous MB's level stream are done
                    if bst = B_DONE and ost = S_OIDLE then
                        if bits_a <= bits_b then is_i4 <= '0'; else is_i4 <= '1'; end if;
                        st <= S_DONE;
                    end if;

                when S_DONE =>
                    done_q  <= '1';
                    o_start <= '1';
                    st <= S_IDLE;
                    bst <= B_IDLE;
            end case;

            ----------------------------------------------------------
            -- candidate issue (chain): sub-block ci's shortlist, one per
            -- cycle when the transform / quant slots are free
            ----------------------------------------------------------
            if cand_ok then
                if ci = 0 and ck >= nfull(0) and rk_ready(0) = '1' and nsub = 2 then
                    ci <= 1;
                elsif ci = 0 and ck < nfull(0) and rk_ready(0) = '1' then
                    kc := ck;
                    if vt(3) = '0' and vq(5) = '0' and vt(10) = '0' then
                        ht := ('1', OP_FULL4, 0, cur_blk(0), rk_mode(kc), kc, '0', 0); issue := true;
                        vt(3) := '1'; vq(5) := '1'; vt(10) := '1';
                        if rk_mode(kc) = pred_mode(0) then cand_mbits(kc) <= 1; else cand_mbits(kc) <= 4; end if;
                        wait_cnt <= W_FULL4;
                        ck <= ck + 1;
                    -- synthesis translate_off
                    else stat_fs <= stat_fs + 1;
                    -- synthesis translate_on
                    end if;
                elsif ci = 1 and rk_ready(1) = '1' and ck < nfull(0) + nfull(1) then
                    kc := ck - nfull(0);
                    if vt(3) = '0' and vq(5) = '0' and vt(10) = '0' then
                        ht := ('1', OP_FULL4, 0, cur_blk(1), rk_mode(3 + kc), kc, '0', 1); issue := true;
                        vt(3) := '1'; vq(5) := '1'; vt(10) := '1';
                        if rk_mode(3 + kc) = pred_mode(1) then cand_mbits(3 + kc) <= 1; else cand_mbits(3 + kc) <= 4; end if;
                        wait_cnt <= W_FULL4;
                        ck <= ck + 1;
                    -- synthesis translate_off
                    else stat_fs <= stat_fs + 1;
                    -- synthesis translate_on
                    end if;
                end if;
            end if;

            ----------------------------------------------------------
            -- background sequencer: both screens first (they share the
            -- Hadamard lane / abs tree with the 4x4 screen and only take
            -- them while the 4x4 screen is ahead of the chain), then the
            -- I_16x16 and chroma chains interleaved so their latency waits
            -- overlap: FWD16, chroma FWD, HAD16, HADC, INV16, INVC.
            -- bwait counts the I_16x16 chain's latency, bwaitc the chroma's.
            ----------------------------------------------------------
            if bwait > 0 then bwait <= bwait - 1; end if;
            if bwaitc > 0 then bwaitc <= bwaitc - 1; end if;
            cand6 := (fb <= 6) and (6 < fb + bl);
            case bst is
                when B_IDLE | B_DONE => null;

                -- I_16x16 screen: modes 0..3 (available ones), 16 blocks each
                when B_SCR16 =>
                    ok := (bcm = 2) or (bcm = 0 and a_top = '1') or (bcm = 1 and a_left = '1') or
                          (bcm = 3 and a_top = '1' and a_left = '1' and a_tl = '1');
                    if bcm > 3 then
                        bcm <= 0; bcp <= 1; bcb <= 0; bst <= B_CSCR;
                    elsif not ok then
                        bcm <= bcm + 1;
                    elsif t_free(9, fb) and vr_i(8) = '0' and vt_i(9) = '0' and vb_i(11) = '0' then
                        bht := ('1', OP_SCR16, 0, bcb, bcm, 0, '0', 0); bissue := true;
                        vr(8) := '1'; vt(9) := '1'; vb(11) := '1';
                        bwait <= W_SCR16;
                        if bcb = 15 then bcb <= 0; bcm <= bcm + 1; else bcb <= bcb + 1; end if;
                    elsif (si_idx = 16 or cand6) and vr_i(8) = '0' and vh_i(9) = '0' and vb_i(11) = '0' then
                        bht := ('1', OP_SCR16, 0, bcb, bcm, 1, '0', 0); bissue := true;
                        vr(8) := '1'; vh(9) := '1'; vb(11) := '1';
                        bwait <= W_SCR16;
                        if bcb = 15 then bcb <= 0; bcm <= bcm + 1; else bcb <= bcb + 1; end if;
                    end if;
                -- chroma screen: modes 0..3 (available ones), both planes, 4 blocks
                when B_CSCR =>
                    ok := (bcm = 0) or (bcm = 1 and a_left = '1') or (bcm = 2 and a_top = '1') or
                          (bcm = 3 and a_top = '1' and a_left = '1' and a_tl = '1');
                    if bcm > 3 then
                        bst <= B_PICK;
                    elsif not ok then
                        bcm <= bcm + 1;
                    elsif vr_i(6) = '0' and vb_i(7) = '0' then
                        bht := ('1', OP_SCRC, bcp, bcb, bcm, 0, '0', 0); bissue := true;
                        vr(6) := '1'; vb(7) := '1';
                        bwaitc <= W_SCRC;
                        if bcb = 3 then
                            bcb <= 0;
                            if bcp = 2 then bcp <= 1; bcm <= bcm + 1; else bcp <= 2; end if;
                        else
                            bcb <= bcb + 1;
                        end if;
                    end if;
                when B_PICK =>
                    -- same result as the C scan (modes 0,1,2,3 in order, strict
                    -- '<', ties keep the earlier mode) done pairwise in two
                    -- cycles; an unavailable mode counts as +inf
                    if bwait = 0 then
                        pk_a <= 0; pk_va <= (others => '1');
                        if a_top = '1' then pk_va <= acc16(0); end if;
                        if a_left = '1' and (a_top = '0' or acc16(1) < acc16(0)) then pk_a <= 1; pk_va <= acc16(1); end if;
                        pk_b <= 2; pk_vb <= acc16(2);
                        if a_top = '1' and a_left = '1' and a_tl = '1' and acc16(3) < acc16(2) then pk_b <= 3; pk_vb <= acc16(3); end if;
                        bst <= B_PICK2;
                    end if;
                when B_PICK2 =>
                    if pk_vb < pk_va then mode16 <= pk_b; else mode16 <= pk_a; end if;
                    bcb <= 0;
                    bst <= B_FWD16;
                when B_FWD16 =>
                    -- first level-store write of this MB: the previous MB's level
                    -- stream must be out
                    if bcb > 15 then
                        bcb <= 0; bst <= B_CPICK;
                    elsif ost = S_OIDLE and t_free(9, fb) and q_free(11, fb) and vr_i(8) = '0' and vt_i(9) = '0' and vq_i(11) = '0' then
                        bht := ('1', OP_FWD16, 0, bcb, mode16, 0, '0', 0); bissue := true;
                        vr(8) := '1'; vt(9) := '1'; vq(11) := '1';
                        bwait <= W_FWD16;
                        bcb <= bcb + 1;
                    end if;
                when B_CPICK =>
                    if bwaitc = 0 then
                        pk_a <= 0; pk_va <= accc(0);
                        if a_left = '1' and accc(1) < accc(0) then pk_a <= 1; pk_va <= accc(1); end if;
                        pk_b <= 2; pk_vb <= (others => '1');
                        if a_top = '1' then pk_vb <= accc(2); end if;
                        if a_top = '1' and a_left = '1' and a_tl = '1' and (a_top = '0' or accc(3) < accc(2)) then pk_b <= 3; pk_vb <= accc(3); end if;
                        bst <= B_CPICK2;
                    end if;
                when B_CPICK2 =>
                    if pk_vb < pk_va then modec <= pk_b; else modec <= pk_a; end if;
                    bcp <= 1; bcb <= 0;
                    bst <= B_CFWD;
                when B_CFWD =>
                    if bcp = 0 then
                        bcb <= 0; bst <= B_HAD16;
                    elsif t_free(7, fb) and q_free(9, fb) and vr_i(6) = '0' and vt_i(7) = '0' and vq_i(9) = '0' then
                        bht := ('1', OP_FWDC, bcp, bcb, modec, 0, '0', 0); bissue := true;
                        vr(6) := '1'; vt(7) := '1'; vq(9) := '1';
                        bwaitc <= W_FWDC;
                        if bcb = 3 then
                            bcb <= 0;
                            if bcp = 2 then bcp <= 0; else bcp <= 2; end if;
                        else
                            bcb <= bcb + 1;
                        end if;
                    end if;
                when B_HAD16 =>
                    if bwait = 0 and t_free(0, fb) and q_free(2, fb) and q_free(7, fb) and t_free(11, fb) and vt_i(0) = '0' and vq_i(2) = '0' and vq_i(7) = '0' and vt_i(11) = '0' then
                        bht := ('1', OP_HAD16, 0, 0, 0, 0, '0', 0); bissue := true;
                        vt(0) := '1'; vq(2) := '1'; vq(7) := '1'; vt(11) := '1';
                        bwait <= W_HAD16; bcp <= 1; bst <= B_CHAD;
                    end if;
                when B_CHAD =>
                    if bcp = 0 then
                        bcb <= 0; bst <= B_INV16;
                    elsif bwaitc = 0 and t_free(0, fb) and q_free(2, fb) and q_free(7, fb) and t_free(11, fb) and vt_i(0) = '0' and vq_i(2) = '0' and vq_i(7) = '0' and vt_i(11) = '0' then
                        bht := ('1', OP_HADC, bcp, 0, 0, 0, '0', 0); bissue := true;
                        vt(0) := '1'; vq(2) := '1'; vq(7) := '1'; vt(11) := '1';
                        if bcp = 2 then bcp <= 0; bwaitc <= W_HADC; else bcp <= 2; end if;
                    end if;
                when B_INV16 =>
                    if bcb > 15 then
                        bcp <= 1; bcb <= 0; bst <= B_CINV;
                    elsif bwait = 0 and q_free(0, fb) and t_free(4, fb) and vq_i(0) = '0' and vt_i(4) = '0' then
                        bht := ('1', OP_INV16, 0, bcb, 0, 0, '1', 0); bissue := true;
                        vq(0) := '1'; vt(4) := '1';
                        if bcb = 15 then bwait <= W_INV16; end if;
                        bcb <= bcb + 1;
                    end if;
                when B_CINV =>
                    if bcp = 0 then
                        if bwait = 0 and bwaitc = 0 then bst <= B_DONE; end if;
                    elsif bwaitc = 0 and q_free(0, fb) and t_free(4, fb) and vq_i(0) = '0' and vt_i(4) = '0' then
                        bht := ('1', OP_INVC, bcp, bcb, 0, 0, '1', 0); bissue := true;
                        vq(0) := '1'; vt(4) := '1';
                        if bcb = 3 then
                            bcb <= 0;
                            if bcp = 2 then bcp <= 0; bwaitc <= W_INVC; else bcp <= 2; end if;
                        else
                            bcb <= bcb + 1;
                        end if;
                    end if;
            end case;

            ----------------------------------------------------------
            -- screen sequencer: neighbour prep (source RAM, 4 cycles) for
            -- block sp_idx, nine screen beats for block si_idx whenever the
            -- 4x4 head is free
            ----------------------------------------------------------
            if sp_idx < 16 and nbs_ready(sp_idx mod 2) = '0' then
                s := SCREEN_ORDER(sp_idx);
                br := SCAN_BR(s); bc := SCAN_BC(s);
                pp := sp_idx mod 2;
                at := '1'; al := '1';
                if br = 0 then at := a_top; end if;
                if bc = 0 then al := a_left; end if;
                case sp_step is
                    when 0 =>
                        -- top row: source block above, or the bundle
                        if br > 0 then sp_top(31 downto 0) <= src_rd_nb(127 downto 96);
                        else sp_top(31 downto 0) <= top_y(bc * 32 + 31 downto bc * 32);
                        end if;
                        nbs_at(pp) <= at; nbs_al(pp) <= al;
                        if s = 0 then
                            -- block 0: every neighbour is in the bundle, one cycle
                            nbs_top(pp)(31 downto 0) <= top_y(31 downto 0);
                            if tr_avail_blk(0) and at = '1' then nbs_top(pp)(63 downto 32) <= top_y(63 downto 32);
                            else nbs_top(pp)(63 downto 32) <= top_y(31 downto 24) & top_y(31 downto 24) & top_y(31 downto 24) & top_y(31 downto 24);
                            end if;
                            nbs_left(pp) <= left_y(31 downto 0);
                            nbs_tl(pp) <= tl_y;
                            nbs_ready(pp) <= '1';
                        end if;
                    when 1 =>
                        -- top-right
                        if tr_avail_blk(s) and ((s /= 5) or a_tr = '1') and at = '1' then
                            if br > 0 then sp_top(63 downto 32) <= src_rd_nb(127 downto 96);
                            elsif s = 5 then sp_top(63 downto 32) <= tr_y;
                            else sp_top(63 downto 32) <= top_y((bc + 1) * 32 + 31 downto (bc + 1) * 32);
                            end if;
                        else
                            sp_top(63 downto 32) <= sp_top(31 downto 24) & sp_top(31 downto 24) & sp_top(31 downto 24) & sp_top(31 downto 24);
                        end if;
                    when 2 =>
                        -- left column
                        if bc > 0 then
                            nbs_left(pp) <= byte_of(src_rd_nb, 15) & byte_of(src_rd_nb, 11) & byte_of(src_rd_nb, 7) & byte_of(src_rd_nb, 3);
                        else
                            nbs_left(pp) <= left_y(br * 32 + 31 downto br * 32);
                        end if;
                    when 3 =>
                        -- top-left
                        if br > 0 and bc > 0 then nbs_tl(pp) <= byte_of(src_rd_nb, 15);
                        elsif br > 0 then nbs_tl(pp) <= byte_of(left_y, br * 4 - 1);
                        elsif bc > 0 then nbs_tl(pp) <= byte_of(top_y, bc * 4 - 1);
                        else nbs_tl(pp) <= tl_y;
                        end if;
                        nbs_top(pp) <= sp_top;
                        nbs_ready(pp) <= '1';
                    when others => null;
                end case;
                if sp_step = 3 or s = 0 then sp_step <= 0; sp_idx <= sp_idx + 1; else sp_step <= sp_step + 1; end if;
            end if;

            if si_idx < 16 and nbs_ready(si_idx mod 2) = '1' then
                s := SCREEN_ORDER(si_idx);
                pp := si_idx mod 2;
                if nbs_al(pp) = '1' then lm := 8; elsif nbs_at(pp) = '1' then lm := 7; else lm := 2; end if;
                if sm > 8 then
                    -- all modes issued: release the neighbour set
                    nbs_ready(pp) <= '0';
                    sm <= 0; si_idx <= si_idx + 1;
                elsif not mode_ok(sm, nbs_at(pp), nbs_al(pp)) then
                    sm <= sm + 1;
                elsif not issue and vh(3) = '0' and va(5) = '0' then
                    ht := ('1', OP_SCR4, 0, SCAN_BR(s) * 4 + SCAN_BC(s), sm, pp, bit_of(sm = lm), 2); issue := true;
                    vh(3) := '1'; va(5) := '1';
                    sm <= sm + 1;
                -- synthesis translate_off
                else
                    stat_scr <= stat_scr + 1;
                -- synthesis translate_on
                end if;
            end if;

            sb_r <= vr; sb_h <= vh; sb_t <= vt; sb_q <= vq; sb_a <= va; sb_b <= vb;
            if issue then
                issuing <= '1';
                head_tag <= ht;
            end if;
            if bissue then
                bissuing <= '1';
                bhead_tag <= bht;
                if bht.plane = 1 then pc_top <= top_u; pc_left <= left_u; pc_tl <= tl_u;
                else pc_top <= top_v; pc_left <= left_v; pc_tl <= tl_v;
                end if;
                if bht.op = OP_HAD16 then
                    t_dcv_q <= dc16;
                elsif bht.op = OP_HADC then
                    t_dcv_q <= (others => (others => '0'));
                    for k in 0 to 3 loop t_dcv_q(k) <= dcc(bht.plane - 1)(k); end loop;
                end if;
            end if;

            ----------------------------------------------------------
            -- next-cycle input selects for the Hadamard lane and the
            -- transform: loop-back (quant tag one stage before its output),
            -- DC vectors (this cycle's background issue), residual A / B
            -- (a predictor output now)
            ----------------------------------------------------------
            nt4 := TAG_NONE;
            if p4_valid_o = '1' then nt4 := tp4(2); end if;
            nt16 := TAG_NONE;
            if p16_valid_o = '1' then nt16 := tp16(8);
            elsif pc_valid_o = '1' then nt16 := tpc(6);
            end if;
            h_sel4_q <= '0'; h_sel16_q <= '0'; h_tag_q <= TAG_NONE;
            if nt4.valid = '1' and nt4.op = OP_SCR4 then
                h_sel4_q <= '1'; h_tag_q <= nt4;
            elsif nt16.valid = '1' and nt16.op = OP_SCR16 and nt16.slot = 1 then
                h_sel16_q <= '1'; h_tag_q <= nt16;
            end if;
            t_sel_lb_q <= '0'; t_sel_dq_q <= '0'; t_sel_dc_q <= '0'; t_sel_r4_q <= '0'; t_sel_r16_q <= '0'; t_dc0_en_q <= '0';
            t_mode_q <= "000"; t_tag_q <= TAG_NONE;
            if tq(4).valid = '1' and tq(4).inv = '0' and tq(4).op = OP_FULL4 then
                -- the fused dequant of this cycle's forward level lands next cycle
                t_sel_dq_q <= '1'; t_tag_q <= tq(4); t_tag_q.inv <= '1'; t_mode_q <= "001";
            elsif tq(3).valid = '1' and tq(3).inv = '1' then
                t_sel_lb_q <= '1'; t_tag_q <= tq(3);
                case tq(3).op is
                    when OP_HAD16 => t_mode_q <= "011";
                    when OP_HADC  => t_mode_q <= "101";
                    when OP_INV16 => t_mode_q <= "001"; t_dc0_en_q <= '1'; t_dc0_q <= resize(dcrec16(tq(3).blk), 32);
                    when OP_INVC  => t_mode_q <= "001"; t_dc0_en_q <= '1'; t_dc0_q <= resize(dccrec(tq(3).plane - 1)(tq(3).blk), 32);
                    when others   => t_mode_q <= "001";
                end case;
            elsif bissue and (bht.op = OP_HAD16 or bht.op = OP_HADC) then
                t_sel_dc_q <= '1'; t_tag_q <= bht;
                if bht.op = OP_HAD16 then t_mode_q <= "010"; else t_mode_q <= "100"; end if;
            elsif nt4.valid = '1' and nt4.op = OP_FULL4 then
                t_sel_r4_q <= '1'; t_tag_q <= nt4;
            elsif nt16.valid = '1' and (nt16.op = OP_FWD16 or nt16.op = OP_FWDC or (nt16.op = OP_SCR16 and nt16.slot = 0)) then
                t_sel_r16_q <= '1'; t_tag_q <= nt16;
                if nt16.op = OP_SCR16 then t_mode_q <= "011"; end if;
            end if;

            -- synthesis translate_off
            if st = S_IDLE then stat_mb <= 0; stat_bg <= 0; stat_ost <= 0; stat_fs <= 0; stat_scr <= 0; stat_rk <= 0; else stat_mb <= stat_mb + 1; end if;
            if bst = B_SCR16 then stat_t_scr16 <= stat_mb + 1; end if;
            if bst = B_CSCR then stat_t_cscr <= stat_mb + 1; end if;
            if bst = B_FWD16 then stat_t_fwd16 <= stat_mb + 1; end if;
            if bst = B_CFWD then stat_t_cfwd <= stat_mb + 1; end if;
            if bst = B_INV16 then stat_t_inv16 <= stat_mb + 1; end if;
            if st = S_PICK and bst /= B_DONE then stat_bg <= stat_bg + 1; end if;
            if st = S_PICK and bst = B_DONE and ost /= S_OIDLE then stat_ost <= stat_ost + 1; end if;
            if bst /= B_DONE and bst /= B_IDLE then stat_bgdone <= stat_mb + 1; end if;
            if st = S_DONE then
                report "MDSTAT mb=" & integer'image(stat_mb) & " bgwait=" & integer'image(stat_bg) &
                       " ostwait=" & integer'image(stat_ost) & " fgstall=" & integer'image(stat_fs) & " bgdone=" & integer'image(stat_bgdone) &
                       " scrstall=" & integer'image(stat_scr) & " rkwait=" & integer'image(stat_rk) &
                       " t_scr16=" & integer'image(stat_t_scr16) & " t_cscr=" & integer'image(stat_t_cscr) & " t_fwd16=" & integer'image(stat_t_fwd16) &
                       " t_cfwd=" & integer'image(stat_t_cfwd) & " t_inv16=" & integer'image(stat_t_inv16) severity note;
            end if;
            -- synthesis translate_on
        end if;
    end process;

    -- screen prep read address (source RAM): the neighbour block for the current prep step
    sp_addr_p : process(all)
        variable s, br, bc : integer range 0 to 15;
        variable a : integer range 0 to 31;
    begin
        s := SCREEN_ORDER(sp_idx mod 16);
        br := SCAN_BR(s); bc := SCAN_BC(s);
        a := 0;
        case sp_step is
            when 0 => if br > 0 then a := (br - 1) * 4 + bc; end if;
            when 1 => if br > 0 and bc < 3 then a := (br - 1) * 4 + bc + 1; end if;
            when 2 => if bc > 0 then a := br * 4 + bc - 1; end if;
            when others => if br > 0 and bc > 0 then a := (br - 1) * 4 + bc - 1; end if;
        end case;
        src_ra_nb <= a;
    end process;

    ------------------------------------------------------------------
    -- Output streaming FSM: reconstruction blocks first (the line buffer
    -- needs them before the next MB), then the level blocks.
    ------------------------------------------------------------------
    ost_p : process(clk, rst_n)
        variable last : boolean;
    begin
        if rst_n = '0' then
            ost <= S_OIDLE; o_item <= 0; blk_valid_q <= '0'; rec_valid_q <= '0'; sdone_q <= '0';
        elsif rising_edge(clk) then
            sdone_q <= '0';
            case ost is
                when S_OIDLE =>
                    if o_start = '1' then
                        o_is4 <= is_i4; o_bank <= lvl4_bank; o_item <= 0; rec_valid_q <= '1'; ost <= S_OREC;
                    end if;
                when S_OREC =>
                    if rec_ready_i = '1' then
                        if o_item = 23 then
                            rec_valid_q <= '0'; blk_valid_q <= '1'; o_item <= 0; ost <= S_OBLK;
                        else
                            o_item <= o_item + 1;
                        end if;
                    end if;
                when S_OBLK =>
                    if blk_ready_i = '1' then
                        if o_is4 = '1' then last := (o_item = 25); else last := (o_item = 26); end if;
                        if last then
                            blk_valid_q <= '0'; sdone_q <= '1'; ost <= S_OIDLE;
                        else
                            o_item <= o_item + 1;
                        end if;
                    end if;
            end case;
        end if;
    end process;

    stream_p : process(all)
        variable it : integer range 0 to 31;
        variable s  : integer range 0 to 15;
        variable pl : integer range 0 to 2;
        variable kd : std_logic;
        variable ix : integer range 0 to 15;
        variable la : integer range 0 to 63;
    begin
        it := o_item;
        pl := 0; kd := '0'; ix := 0; la := 0;
        if o_is4 = '1' then
            if it < 16 then
                s := it; ix := SCAN_BR(s) * 4 + SCAN_BC(s);
            elsif it = 16 then pl := 1; kd := '1'; la := 56;
            elsif it = 17 then pl := 2; kd := '1'; la := 57;
            elsif it < 22 then pl := 1; ix := it - 18; la := 48 + ix;
            else pl := 2; ix := it - 22; la := 52 + ix;
            end if;
        else
            if it = 0 then kd := '1'; la := 16;
            elsif it < 17 then
                s := it - 1; ix := SCAN_BR(s) * 4 + SCAN_BC(s); la := ix;
            elsif it = 17 then pl := 1; kd := '1'; la := 56;
            elsif it = 18 then pl := 2; kd := '1'; la := 57;
            elsif it < 23 then pl := 1; ix := it - 19; la := 48 + ix;
            else pl := 2; ix := it - 23; la := 52 + ix;
            end if;
        end if;
        blk_plane_o <= to_unsigned(pl, 2);
        blk_kind_o  <= kd;
        blk_idx_o   <= to_unsigned(ix, 4);
        lvl_ra_stream <= la;
        if o_bank = '1' then lvl4_ra <= 16 + ix; else lvl4_ra <= ix; end if;
        -- recon items: Y 0..15 (r4 or rec_mem), U 16..19, V 20..23
        if it < 16 then
            rec_plane_o <= "00"; rec_idx_o <= to_unsigned(it, 4); rec_ra <= it;
        elsif it < 20 then
            rec_plane_o <= "01"; rec_idx_o <= to_unsigned(it - 16, 4); rec_ra <= it;
        else
            rec_plane_o <= "10"; rec_idx_o <= to_unsigned(it - 20, 4); rec_ra <= it;
        end if;
    end process;

    blk_levels_o <= unpack13(lvl4_rd) when (o_is4 = '1' and o_item < 16) else unpack13(lvl_rd);
    rec_data_o   <= r4_stream when (o_is4 = '1' and o_item < 16) else rec_mem(rec_ra);
    blk_valid_o  <= blk_valid_q;
    rec_valid_o  <= rec_valid_q;
    stream_busy_o <= '0' when ost = S_OIDLE else '1';

    ------------------------------------------------------------------
    -- Decision outputs
    ------------------------------------------------------------------
    busy_o         <= '0' when st = S_IDLE else '1';
    done_o         <= done_q;
    stream_done_o  <= sdone_q;
    is_i4x4_o      <= is_i4;
    mode16_o       <= to_unsigned(mode16, 2);
    modes4_o       <= modes4;
    mode_chroma_o  <= to_unsigned(modec, 2);
    luma_nz_o      <= nz4 when is_i4 = '1' else nz16;
    chroma_dc_nz_o <= cdc_nz;
    chroma_ac_nz_o <= cac_nz;
    bits_a_o       <= bits_a;
    bits_b_o       <= bits_b;
    dbg_j_o        <= best_j(0);

    -- TotalCoeff per block, zero where the coded_block_pattern drops the block
    tc_out_p : process(all)
        variable qnz : std_logic_vector(3 downto 0);
        variable any16 : std_logic;
    begin
        qnz(0) := nz4(0)  or nz4(1)  or nz4(4)  or nz4(5);
        qnz(1) := nz4(2)  or nz4(3)  or nz4(6)  or nz4(7);
        qnz(2) := nz4(8)  or nz4(9)  or nz4(12) or nz4(13);
        qnz(3) := nz4(10) or nz4(11) or nz4(14) or nz4(15);
        any16 := or nz16;
        for k in 0 to 15 loop
            tc_y_o(5 * k + 4 downto 5 * k) <= (others => '0');
            if is_i4 = '1' then
                if qnz(QUAD(k)) = '1' then tc_y_o(5 * k + 4 downto 5 * k) <= std_logic_vector(tcnt4(k)); end if;
            elsif any16 = '1' then
                tc_y_o(5 * k + 4 downto 5 * k) <= std_logic_vector(tcnt16(k));
            end if;
        end loop;
        for k in 0 to 3 loop
            tc_u_o(5 * k + 4 downto 5 * k) <= (others => '0');
            tc_v_o(5 * k + 4 downto 5 * k) <= (others => '0');
            if cac_nz = '1' then
                tc_u_o(5 * k + 4 downto 5 * k) <= std_logic_vector(tcntc(k));
                tc_v_o(5 * k + 4 downto 5 * k) <= std_logic_vector(tcntc(4 + k));
            end if;
        end loop;
    end process;

end architecture;
