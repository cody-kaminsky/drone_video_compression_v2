--------------------------------------------------------------------------------
-- mode_decide_engine.vhd
--
-- Per-macroblock intra mode decision and luma/chroma coding, the hardware
-- form of mb_mode_decide + try_path_i4x4 + the chroma stages (mb_residual
-- .. mb_reconstruct) in src/encoder.c, with the I_4x4 policy of
-- src/rd_tables.h (SATD screen with mode-bit penalty, RD_I4_SHORTLIST
-- candidates evaluated by J = 16*SSD + lambda*(bits + mode bits)).
--
-- One shared datapath, free-running (no back-pressure inside):
--
--   predict_4x4 / predict_16x16 / predict_chroma  -> residual (src - pred)
--     -> transform_engine (DIR both) -> quant_engine (fwd/inv)
--     -> cavlc_cost_engine_ll (bit estimate)  |  loop-back -> quant (inv)
--     -> transform (inv) -> recon_engine (+SSD)
--
-- The transform and quantizer are single instances serving both
-- directions: the sequencer issues forward beats (anything that reaches
-- the transform from the predictors or the DC vectors) on even cycles and
-- inverse beats (quantizer input from the loop-back register or the level
-- store) on odd cycles, so a forward block and an inverse block never
-- compete for a port. Every beat carries a tag through delay lines that
-- match each engine's fixed latency; the tag says what to do with the
-- result (accumulate SATD, store levels, capture DC, reconstruct, ...).
--
-- Sequence per MB (v1, I_4x4 blocks strictly in scan order):
--   I16 screen (4 modes x 16 blocks SATD) -> forward (DCT, AC quant, DC
--   Hadamard + quant, bit estimate) -> inverse (DC chain, AC dequant with
--   DC splice, IDCT, recon)  |  I4: per block screen 9 modes, rank, full
--   RD on the shortlist, pick, commit  |  path pick by estimated bits  |
--   chroma screen (4 modes SAD) -> forward -> DC 2x2 chain -> inverse.
--   Then the decision is presented (done_o) and the reconstruction blocks
--   and the level blocks are streamed out; the streams run from their own
--   FSM so that a consumer that drains them quickly loses no time.
--   The source MB arrives as a 24-word stream right after start (block b
--   is first needed 8 cycles after its issue, so the load overlaps the
--   I16 screen); it lives in a small LUT RAM instead of 3 K flip-flops.
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
    constant W_SCR4  : integer := 8;
    constant W_FULL4 : integer := 21;
    constant W_SCRC  : integer := 10;
    constant W_FWDC  : integer := 9;
    constant W_HADC  : integer := 14;
    constant W_INVC  : integer := 11;

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

    type op_t is (OP_NONE, OP_SCR4, OP_FULL4, OP_SCR16, OP_FWD16, OP_HAD16, OP_INV16,
                  OP_SCRC, OP_FWDC, OP_HADC, OP_INVC);
    type tag_t is record
        valid : std_logic;
        op    : op_t;
        plane : integer range 0 to 2;
        blk   : integer range 0 to 15;
        mode  : integer range 0 to 8;
        slot  : integer range 0 to 2;
        inv   : std_logic;
        sub   : integer range 0 to 1;     -- which block of a wavefront pair
    end record;
    constant TAG_NONE : tag_t := ('0', OP_NONE, 0, 0, 0, 0, '0', 0);
    type tag_arr is array (natural range <>) of tag_t;

    ------------------------------------------------------------------
    -- helper functions
    ------------------------------------------------------------------
    function blk_of(v : std_logic_vector; k : integer) return px128 is
    begin
        return v(128 * k + 127 downto 128 * k);
    end function;

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

    -- pred RAM slot for a tag
    function pred_slot(t : tag_t) return integer is
    begin
        case t.op is
            when OP_FULL4 => return t.sub * 3 + t.slot;
            when OP_FWD16 | OP_INV16 => return 6 + t.blk;
            when others => return 22 + (t.plane - 1) * 4 + t.blk;   -- chroma
        end case;
    end function;

    -- level store address for a tag
    function lvl_addr(t : tag_t) return integer is
    begin
        case t.op is
            when OP_FWD16 | OP_INV16 => return t.blk;
            when OP_HAD16 => return 16;
            when OP_FWDC | OP_INVC => return 48 + (t.plane - 1) * 4 + t.blk;
            when others => return 56 + (t.plane - 1);                 -- HADC
        end case;
    end function;

    function tr_avail_blk(s : integer) return boolean is
    begin
        return not (s = 3 or s = 7 or s = 11 or s = 13 or s = 15);
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
    signal src_ra_res, src_ra_rec : integer range 0 to 31;
    signal src_rd_res, src_rd_rec : px128;
    signal top_y, left_y : px128;
    signal tr_y : std_logic_vector(31 downto 0);
    signal tl_y, tl_u, tl_v : std_logic_vector(7 downto 0);
    signal top_u, left_u, top_v, left_v : std_logic_vector(63 downto 0);
    signal a_top, a_left, a_tl, a_tr : std_logic;
    signal m4top, m4left : std_logic_vector(15 downto 0);
    signal pen1, pen4 : unsigned(11 downto 0);
    signal lam : unsigned(15 downto 0);

    ------------------------------------------------------------------
    -- sequencer
    ------------------------------------------------------------------
    type st_t is (S_IDLE, S_I16_SCR, S_I16_PICK, S_I16_PICK2, S_I16_FWD, S_I16_HAD, S_I16_INV,
                  S_I4_PREP, S_I4_SCR, S_I4_FULL, S_I4_COMMIT,
                  S_PICK, S_C_SCR, S_C_PICK, S_C_PICK2, S_C_FWD, S_C_HAD, S_C_INV,
                  S_DONE);
    signal st : st_t := S_IDLE;
    signal phase : std_logic := '0';           -- toggles every cycle; '0' = even
    signal wait_cnt : integer range 0 to 31 := 0;
    signal cm : integer range 0 to 9 := 0;      -- mode counter (runs one past the last mode)
    signal cb : integer range 0 to 16 := 0;     -- block / quadrant counter (runs one past the last block)
    signal cp : integer range 0 to 2 := 0;      -- plane counter
    signal cs : integer range 0 to 15 := 0;     -- scan index (I4)
    signal ck : integer range 0 to 7 := 0;      -- shortlist counter (up to 2 x 3 candidates)
    signal issuing : std_logic := '0';          -- a beat left the head this cycle
    signal head_tag : tag_t := TAG_NONE;

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

    -- I4 wavefront: up to two independent blocks per step (sub 0 / sub 1)
    type step_tab_t is array (0 to 9, 0 to 1) of integer range -1 to 15;   -- scan indices
    constant STEP_TAB : step_tab_t := (
        (0, -1), (1, -1), (4, 2), (5, 3), (6, 8), (7, 9), (12, 10), (13, 11), (14, -1), (15, -1));
    type i2_t is array (0 to 1) of integer range 0 to 15;
    type m2_t is array (0 to 1) of integer range 0 to 8;
    type nbtop_t is array (0 to 1) of std_logic_vector(63 downto 0);
    type nbleft_t is array (0 to 1) of std_logic_vector(31 downto 0);
    type nbtl_t is array (0 to 1) of std_logic_vector(7 downto 0);
    signal step : integer range 0 to 9 := 0;
    signal nsub : integer range 1 to 2 := 1;
    signal cc : integer range 0 to 1 := 0;          -- sub-block cursor (prep / commit)
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
    type rk_cost_t is array (0 to 5) of unsigned(23 downto 0);
    type rk_mode_t is array (0 to 5) of integer range 0 to 8;
    type rk_n_t is array (0 to 1) of integer range 0 to 3;
    signal rk_cost : rk_cost_t := (others => (others => '0'));
    signal rk_mode : rk_mode_t := (others => 0);
    signal rk_n : rk_n_t := (others => 0);
    signal nfull : rk_n_t := (others => 0);
    signal nfull_tot : integer range 0 to 6 := 0;
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
    type bj_t is array (0 to 1) of unsigned(27 downto 0);
    type bs_t is array (0 to 1) of integer range 0 to 2;
    signal best_j : bj_t := (others => (others => '1'));
    signal best_slot : bs_t := (others => 0);
    signal j_prod : unsigned(27 downto 0) := (others => '0');
    signal j_ssd16 : unsigned(27 downto 0) := (others => '0');
    signal j_pend : std_logic := '0';
    signal j_pend_idx : integer range 0 to 5 := 0;
    signal j_pend_sub : integer range 0 to 1 := 0;
    -- chroma screen issued in the I4 idle windows
    signal csm : integer range 0 to 4 := 0;
    signal csp : integer range 1 to 2 := 1;
    signal csq : integer range 0 to 3 := 0;
    signal cs_done : std_logic := '0';
    attribute use_dsp : string;
    attribute use_dsp of j_prod : signal is "yes";

    -- reconstruction / DC state
    type r4_mem_t is array (0 to 15) of px128;
    signal r4 : r4_mem_t;
    attribute ram_style of r4 : signal is "distributed";
    signal r4_stream : px128;
    -- pairwise pick pipeline
    signal pk_a, pk_b : integer range 0 to 3 := 0;
    signal pk_va, pk_vb : unsigned(23 downto 0) := (others => '0');
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

    -- level store
    type lvl_mem_t is array (0 to 63) of lvl208;
    signal lvl_mem : lvl_mem_t;
    attribute ram_style of lvl_mem : signal is "distributed";
    signal lvl_we : std_logic;
    signal lvl_wa : integer range 0 to 63;
    signal lvl_wd : lvl208;
    signal lvl_ra : integer range 0 to 63 := 0;
    signal lvl_ra_stream : integer range 0 to 63 := 0;
    signal lvl_rd : lvl208;

    -- pred store
    type pred_mem_t is array (0 to 31) of px128;
    signal pred_mem : pred_mem_t;
    attribute ram_style of pred_mem : signal is "distributed";
    signal pred_rd : px128;
    signal pred_ra : integer range 0 to 31;

    ------------------------------------------------------------------
    -- engines
    ------------------------------------------------------------------
    signal p4_valid_i, p4_valid_o, p16_valid_i, p16_valid_o, pc_valid_i, pc_valid_o : std_logic;
    signal p4_mode : unsigned(3 downto 0);
    signal p16_mode, pc_mode : unsigned(1 downto 0);
    signal p16_blk : unsigned(3 downto 0);
    signal pc_blk : unsigned(1 downto 0);
    signal pc_top, pc_left : std_logic_vector(63 downto 0);
    signal pc_tl : std_logic_vector(7 downto 0);
    signal p4_pred, p16_pred, pc_pred : px128;
    signal tp4 : tag_arr(1 to 1) := (others => TAG_NONE);
    signal tp16 : tag_arr(1 to 8) := (others => TAG_NONE);
    signal tpc : tag_arr(1 to 6) := (others => TAG_NONE);

    -- residual stage
    signal res_q : s20_arr16 := (others => (others => '0'));   -- 10-bit residual, sign-extended
    signal res_tag : tag_t := TAG_NONE;
    signal res_pred : px128 := (others => '0');

    -- transform
    signal t_din, t_dout : vec16_t;
    signal t_mode : unsigned(2 downto 0);
    signal t_valid_i, t_valid_o : std_logic;
    signal t_tag_in : tag_t;
    signal tt : tag_arr(1 to 2) := (others => TAG_NONE);

    -- quant
    signal q_din, q_dout : vec16_t;
    signal q_mode : unsigned(2 downto 0);
    signal q_qp : unsigned(5 downto 0);
    signal q_valid_i, q_valid_o : std_logic;
    signal q_tag_in : tag_t;
    signal tq : tag_arr(1 to 4) := (others => TAG_NONE);

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
    -- registered recon inputs (the tag-addressed prediction RAM read plus
    -- the recon adders do not fit one cycle)
    signal r_pred_q, r_src_q : px128 := (others => '0');
    signal r_res_q : std_logic_vector(16 * 20 - 1 downto 0) := (others => '0');
    signal r_valid_q : std_logic := '0';
    signal r_tag_q : tag_t := TAG_NONE;
    signal trc : tag_arr(1 to 3) := (others => TAG_NONE);

    -- abs-sum tree (SATD / SAD)
    signal ab_in : s20_arr16;
    signal ab_tag_in : tag_t;
    signal ab_valid_in : std_logic;
    type u20_arr16 is array (0 to 15) of unsigned(19 downto 0);
    signal ab1 : u20_arr16 := (others => (others => '0'));
    signal ab1_tag : tag_t := TAG_NONE;
    signal ab2 : unsigned(23 downto 0) := (others => '0');
    signal ab2_tag : tag_t := TAG_NONE;
    -- penalised cost registered before the rank insert
    signal ab3 : unsigned(23 downto 0) := (others => '0');
    signal ab3_tag : tag_t := TAG_NONE;

    -- output streams (own FSM so the next MB can start while they drain)
    type ost_t is (S_OIDLE, S_OREC, S_OBLK);
    signal ost : ost_t := S_OIDLE;
    signal o_item : integer range 0 to 31 := 0;
    signal o_is4 : std_logic := '0';
    signal o_start : std_logic := '0';
    signal blk_valid_q : std_logic := '0';
    signal rec_valid_q : std_logic := '0';
    signal done_q, sdone_q : std_logic := '0';

begin

    ------------------------------------------------------------------
    -- Engine instances
    ------------------------------------------------------------------
    p4_top  <= nb_top(head_tag.sub);
    p4_left <= nb_left(head_tag.sub);
    p4_tl   <= nb_tl(head_tag.sub);
    p4_at   <= nb_at(head_tag.sub);
    p4_al   <= nb_al(head_tag.sub);

    p4 : entity work.predict_4x4_engine
        port map (clk => clk, rst_n => rst_n, mode_i => p4_mode, top_i => p4_top, left_i => p4_left,
                  tl_i => p4_tl, avail_top_i => p4_at, avail_left_i => p4_al, avail_tl_i => p4_at and p4_al,
                  valid_i => p4_valid_i, ready_o => open, pred_o => p4_pred, valid_o => p4_valid_o,
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

    tr : entity work.transform_engine
        generic map (W => 20, DIR => "both")
        port map (clk => clk, rst_n => rst_n, mode_i => t_mode,
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
                  valid_o => q_valid_o, ready_i => '1');

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
    -- Head: predictor inputs
    ------------------------------------------------------------------
    p4_valid_i  <= issuing when (head_tag.op = OP_SCR4 or head_tag.op = OP_FULL4) else '0';
    p4_mode     <= to_unsigned(head_tag.mode, 4);
    p16_valid_i <= issuing when (head_tag.op = OP_SCR16 or head_tag.op = OP_FWD16) else '0';
    p16_mode    <= to_unsigned(head_tag.mode, 2);
    p16_blk     <= to_unsigned(head_tag.blk, 4);
    pc_valid_i  <= issuing when (head_tag.op = OP_SCRC or head_tag.op = OP_FWDC) else '0';
    pc_mode     <= to_unsigned(head_tag.mode, 2);
    pc_blk      <= to_unsigned(head_tag.blk, 2);
    pc_top      <= top_u  when head_tag.plane = 1 else top_v;
    pc_left     <= left_u when head_tag.plane = 1 else left_v;
    pc_tl       <= tl_u   when head_tag.plane = 1 else tl_v;

    ------------------------------------------------------------------
    -- Tag delay lines (free-running)
    ------------------------------------------------------------------
    tags_p : process(clk)
    begin
        if rising_edge(clk) then
            if p4_valid_i = '1' then tp4(1) <= head_tag; else tp4(1) <= TAG_NONE; end if;
            if p16_valid_i = '1' then tp16(1) <= head_tag; else tp16(1) <= TAG_NONE; end if;
            tp16(2 to 8) <= tp16(1 to 7);
            if pc_valid_i = '1' then tpc(1) <= head_tag; else tpc(1) <= TAG_NONE; end if;
            tpc(2 to 6) <= tpc(1 to 5);
            if t_valid_i = '1' then tt(1) <= t_tag_in; else tt(1) <= TAG_NONE; end if;
            tt(2) <= tt(1);
            if q_valid_i = '1' then tq(1) <= q_tag_in; else tq(1) <= TAG_NONE; end if;
            tq(2 to 4) <= tq(1 to 3);
            if c_valid_i = '1' then tc(1) <= tq(4); else tc(1) <= TAG_NONE; end if;
            tc(2 to 5) <= tc(1 to 4);
            if r_valid_q = '1' then trc(1) <= r_tag_q; else trc(1) <= TAG_NONE; end if;
            trc(2 to 3) <= trc(1 to 2);
        end if;
    end process;

    ------------------------------------------------------------------
    -- Residual stage: merge predictor outputs, subtract the source block,
    -- store the prediction for the reconstruction later.
    ------------------------------------------------------------------
    src_addr_p : process(all)
        variable tg : tag_t;
    begin
        tg := TAG_NONE;
        if p4_valid_o = '1' then tg := tp4(1);
        elsif p16_valid_o = '1' then tg := tp16(8);
        elsif pc_valid_o = '1' then tg := tpc(6);
        end if;
        case tg.plane is
            when 0 => src_ra_res <= tg.blk;
            when 1 => src_ra_res <= 16 + (tg.blk mod 4);
            when others => src_ra_res <= 20 + (tg.blk mod 4);
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
            tg := TAG_NONE; pr := (others => '0');
            if p4_valid_o = '1' then tg := tp4(1); pr := p4_pred;
            elsif p16_valid_o = '1' then tg := tp16(8); pr := p16_pred;
            elsif pc_valid_o = '1' then tg := tpc(6); pr := pc_pred;
            end if;
            sb := src_rd_res;
            for k in 0 to 15 loop
                res_q(k) <= resize(signed('0' & byte_of(sb, k)), 20) - resize(signed('0' & byte_of(pr, k)), 20);
            end loop;
            res_tag  <= tg;
            res_pred <= pr;
        end if;
    end process;

    -- source block RAM: loaded from the 24-word stream after start
    src_wr : process(clk)
    begin
        if rising_edge(clk) then
            if src_ld < 24 then
                src_mem(src_ld) <= src_data_i;
                -- synthesis translate_off
                if DEBUG then
                    report "SRC " & integer'image(src_ld) & ": " & integer'image(to_integer(unsigned(src_data_i(7 downto 0)))) & " " &
                           integer'image(to_integer(unsigned(src_data_i(15 downto 8)))) & " " & integer'image(to_integer(unsigned(src_data_i(23 downto 16)))) & " " &
                           integer'image(to_integer(unsigned(src_data_i(31 downto 24)))) & " .. " & integer'image(to_integer(unsigned(src_data_i(127 downto 120)))) severity note;
                end if;
                -- synthesis translate_on
            end if;
        end if;
    end process;
    src_rd_res <= src_mem(src_ra_res);
    src_rd_rec <= src_mem(src_ra_rec);

    -- prediction store: written for anything that will be reconstructed
    pred_wr : process(clk)
    begin
        if rising_edge(clk) then
            if res_tag.valid = '1' and (res_tag.op = OP_FULL4 or res_tag.op = OP_FWD16 or res_tag.op = OP_FWDC) then
                pred_mem(pred_slot(res_tag)) <= res_pred;
            end if;
        end if;
    end process;
    pred_rd <= pred_mem(pred_ra);

    ------------------------------------------------------------------
    -- Transform input mux: loop-back (inverse) > DC vectors > residual
    ------------------------------------------------------------------
    t_in_p : process(all)
        variable lb_to_t : std_logic;
        variable qt : tag_t;
    begin
        qt := tq(4);
        lb_to_t := q_valid_o and qt.inv;
        t_valid_i <= '0';
        t_mode    <= "000";
        t_tag_in  <= TAG_NONE;
        for k in 0 to 15 loop t_din(k) <= (others => '0'); end loop;
        if lb_to_t = '1' then
            t_valid_i <= '1';
            t_tag_in  <= qt;
            for k in 0 to 15 loop t_din(k) <= q_dout(k); end loop;
            case qt.op is
                when OP_HAD16 => t_mode <= "011";
                when OP_HADC  => t_mode <= "101";
                when OP_INV16 => t_mode <= "001"; t_din(0) <= resize(dcrec16(qt.blk), 32);
                when OP_INVC  => t_mode <= "001"; t_din(0) <= resize(dccrec(qt.plane - 1)(qt.blk), 32);
                when others   => t_mode <= "001";
            end case;
        elsif issuing = '1' and (head_tag.op = OP_HAD16 or head_tag.op = OP_HADC) then
            t_valid_i <= '1';
            t_tag_in  <= head_tag;
            if head_tag.op = OP_HAD16 then
                t_mode <= "010";
                for k in 0 to 15 loop t_din(k) <= resize(dc16(k), 32); end loop;
            else
                t_mode <= "100";
                for k in 0 to 3 loop t_din(k) <= resize(dcc(head_tag.plane - 1)(k), 32); end loop;
            end if;
        elsif res_tag.valid = '1' and res_tag.op /= OP_SCRC then
            t_valid_i <= '1';
            t_tag_in  <= res_tag;
            for k in 0 to 15 loop t_din(k) <= resize(res_q(k), 32); end loop;
            if res_tag.op = OP_SCR4 or res_tag.op = OP_SCR16 then t_mode <= "011"; else t_mode <= "000"; end if;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Quant input mux: forward from the transform (even cycles),
    -- inverse from the loop-back register or the level store (odd)
    ------------------------------------------------------------------
    lvl_rd <= lvl_mem(lvl_ra);

    q_in_p : process(all)
        variable tt2 : tag_t;
        variable fwd_ok : std_logic;
        variable lv : vec16_t;
    begin
        tt2 := tt(2);
        fwd_ok := '0';
        if t_valid_o = '1' and tt2.inv = '0' and
           (tt2.op = OP_FULL4 or tt2.op = OP_FWD16 or tt2.op = OP_FWDC or tt2.op = OP_HAD16 or tt2.op = OP_HADC) then
            fwd_ok := '1';
        end if;
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
        elsif issuing = '1' and (head_tag.op = OP_INV16 or head_tag.op = OP_INVC) then
            q_valid_i <= '1';
            q_tag_in  <= head_tag;
            lv := unpack13_v(lvl_rd);
            for k in 0 to 15 loop q_din(k) <= lv(k); end loop;
            q_mode <= "001";
            if head_tag.plane /= 0 then q_qp <= qp_c; end if;
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

    -- level-store read address: inverse beats from the head, else the output streamer
    lvl_ra <= lvl_addr(head_tag) when (issuing = '1' and (head_tag.op = OP_INV16 or head_tag.op = OP_INVC))
              else lvl_ra_stream;

    ------------------------------------------------------------------
    -- Quant output: level store, DC capture is at the transform output,
    -- cost engine feed, loop-back register
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

    lvl_wr_p : process(all)
        variable qt : tag_t;
    begin
        qt := tq(4);
        lvl_we <= '0'; lvl_wa <= 0; lvl_wd <= (others => '0');
        if st = S_I4_COMMIT then
            lvl_we <= '1';
            lvl_wa <= 32 + cur_blk(cc);
            lvl_wd <= cand_lvl_rd;
        elsif q_valid_o = '1' and qt.inv = '0' and
              (qt.op = OP_FWD16 or qt.op = OP_HAD16 or qt.op = OP_FWDC or qt.op = OP_HADC) then
            lvl_we <= '1';
            lvl_wa <= lvl_addr(qt);
            lvl_wd <= pack13(q_dout);
        end if;
    end process;

    lvl_mem_p : process(clk)
    begin
        if rising_edge(clk) then
            if lvl_we = '1' then lvl_mem(lvl_wa) <= lvl_wd; end if;
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

    ------------------------------------------------------------------
    -- Recon input: inverse transform output + stored prediction
    ------------------------------------------------------------------
    rec_in_p : process(all)
        variable t2 : tag_t;
    begin
        t2 := tt(2);
        r_valid_i <= '0';
        r_tag_in  <= TAG_NONE;
        pred_ra   <= 0;
        r_pred    <= (others => '0');
        r_src     <= (others => '0');
        for k in 0 to 15 loop
            r_res(20 * k + 19 downto 20 * k) <= std_logic_vector(t_dout(k)(19 downto 0));
        end loop;
        if t_valid_o = '1' and t2.inv = '1' and (t2.op = OP_FULL4 or t2.op = OP_INV16 or t2.op = OP_INVC) then
            r_valid_i <= '1';
            r_tag_in  <= t2;
            pred_ra   <= pred_slot(t2);
            r_pred    <= pred_rd;
            r_src     <= src_rd_rec;
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
    -- abs-sum tree: SATD from the transform (Hadamard) output, SAD from
    -- the residual (chroma screen)
    ------------------------------------------------------------------
    ab_p : process(clk)
        variable s : unsigned(23 downto 0);
        variable v : s20;
    begin
        if rising_edge(clk) then
            ab1_tag <= TAG_NONE;
            if t_valid_o = '1' and tt(2).inv = '0' and (tt(2).op = OP_SCR4 or tt(2).op = OP_SCR16) then
                ab1_tag <= tt(2);
                for k in 0 to 15 loop
                    v := t_dout(k)(19 downto 0);
                    if v < 0 then ab1(k) <= unsigned(-v); else ab1(k) <= unsigned(v); end if;
                end loop;
            elsif res_tag.valid = '1' and res_tag.op = OP_SCRC then
                ab1_tag <= res_tag;
                for k in 0 to 15 loop
                    v := res_q(k);
                    if v < 0 then ab1(k) <= unsigned(-v); else ab1(k) <= unsigned(v); end if;
                end loop;
            end if;
            s := (others => '0');
            for k in 0 to 15 loop s := s + resize(ab1(k), 24); end loop;
            ab2     <= s;
            ab2_tag <= ab1_tag;
        end if;
    end process;

    ------------------------------------------------------------------
    -- Result collectors and the sequencer
    ------------------------------------------------------------------
    main_p : process(clk, rst_n)
        variable qt, t2, ct, rt : tag_t;
        variable c : unsigned(23 downto 0);
        variable p : integer range 0 to 3;
        variable m : integer range 0 to 8;
        variable ok : boolean;
        variable br, bc, s : integer range 0 to 15;
        variable jv : unsigned(27 downto 0);
        variable mt, ml : integer range 0 to 15;
        variable tok, lok : boolean;
        variable at, al : std_logic;
        variable topv : std_logic_vector(63 downto 0);
        variable leftv : std_logic_vector(31 downto 0);
        variable tlv : std_logic_vector(7 downto 0);
        variable best : integer range 0 to 3;
        variable bestc : unsigned(23 downto 0);
        variable issue : boolean;
        variable ht : tag_t;
        variable last : boolean;
        variable sb : integer range 0 to 1;
        variable kc : integer range 0 to 2;
        variable sidx : integer range -1 to 15;
        variable nf : integer range 0 to 3;
        variable cs_ok : boolean;
        variable cs_issue : boolean;
    begin
        if rst_n = '0' then
            st <= S_IDLE; phase <= '0'; issuing <= '0'; head_tag <= TAG_NONE;
            done_q <= '0'; o_start <= '0';
            lb_tag <= TAG_NONE; j_pend <= '0'; src_ld <= 24;
        elsif rising_edge(clk) then
            phase <= not phase;
            done_q <= '0'; o_start <= '0';
            issuing <= '0';
            if src_ld < 24 then src_ld <= src_ld + 1; end if;
            ht := TAG_NONE;
            issue := false;

            ----------------------------------------------------------
            -- loop-back register: quant forward output of FULL4 / HAD16 /
            -- HADC goes back into the quantizer as an inverse beat
            ----------------------------------------------------------
            qt := tq(4);
            rt := trc(3);
            lb_tag <= TAG_NONE;
            if q_valid_o = '1' and qt.inv = '0' and (qt.op = OP_FULL4 or qt.op = OP_HAD16 or qt.op = OP_HADC) then
                lb_tag <= qt; lb_tag.inv <= '1';
                lb_data <= q_dout;
            end if;

            ----------------------------------------------------------
            -- quant forward results: candidate levels / nz flags
            ----------------------------------------------------------
            if q_valid_o = '1' and qt.inv = '0' then
                case qt.op is
                    when OP_FULL4 =>
                        cand_nz(qt.sub * 3 + qt.slot)  <= any_nz(q_dout, 0);
                    when OP_FWD16 => nz16(qt.blk) <= any_nz(q_dout, 1);
                    when OP_FWDC  => cac_nz <= cac_nz or any_nz(q_dout, 1);
                    when OP_HADC  => cdc_nz <= cdc_nz or any_nz(q_dout, 0);
                    when others => null;
                end case;
            end if;

            ----------------------------------------------------------
            -- transform results: DC capture (forward), DC recon (inverse)
            ----------------------------------------------------------
            t2 := tt(2);
            -- synthesis translate_off
            if DEBUG and t_valid_o = '1' then
                report "T out op=" & op_t'image(t2.op) & " inv=" & std_logic'image(t2.inv) & " blk=" & integer'image(t2.blk) &
                       " d0..3=" & integer'image(to_integer(t_dout(0))) & "," & integer'image(to_integer(t_dout(1))) & "," &
                       integer'image(to_integer(t_dout(2))) & "," & integer'image(to_integer(t_dout(3))) severity note;
            end if;
            if DEBUG and q_valid_o = '1' then
                report "Q out op=" & op_t'image(qt.op) & " inv=" & std_logic'image(qt.inv) & " blk=" & integer'image(qt.blk) &
                       " d0..3=" & integer'image(to_integer(q_dout(0))) & "," & integer'image(to_integer(q_dout(1))) & "," &
                       integer'image(to_integer(q_dout(2))) & "," & integer'image(to_integer(q_dout(3))) severity note;
            end if;
            if DEBUG and q_valid_i = '1' then
                report "Q in  op=" & op_t'image(q_tag_in.op) & " inv=" & std_logic'image(q_tag_in.inv) & " mode=" & integer'image(to_integer(q_mode)) &
                       " d0..3=" & integer'image(to_integer(q_din(0))) & "," & integer'image(to_integer(q_din(1))) & "," &
                       integer'image(to_integer(q_din(2))) & "," & integer'image(to_integer(q_din(3))) severity note;
            end if;
            if DEBUG and r_valid_o = '1' then
                report "R out op=" & op_t'image(rt.op) & " blk=" & integer'image(rt.blk) & " r0=" &
                       integer'image(to_integer(unsigned(r_recon(7 downto 0)))) & " ssd=" & integer'image(to_integer(r_ssd)) severity note;
            end if;
            if DEBUG and issuing = '1' then
                report "ISSUE op=" & op_t'image(head_tag.op) & " blk=" & integer'image(head_tag.blk) & " mode=" & integer'image(head_tag.mode) &
                       " slot=" & integer'image(head_tag.slot) & " phase=" & std_logic'image(phase) severity note;
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
            -- cost results
            ----------------------------------------------------------
            ct := tc(5);
            if c_valid_o = '1' then
                if ct.op = OP_FULL4 then cand_bits(ct.sub * 3 + ct.slot) <= c_bits;
                else bits_a <= bits_a + resize(c_bits, 16);
                end if;
            end if;

            ----------------------------------------------------------
            -- recon results
            ----------------------------------------------------------
            rt := trc(3);
            j_pend <= '0';
            if r_valid_o = '1' and rt.op = OP_FULL4 then
                j_ssd16 <= resize(r_ssd & "0000", 28);
                j_pend  <= '1';
                j_pend_idx <= rt.sub * 3 + rt.slot;
                j_pend_sub <= rt.sub;
                j_prod <= resize(lam * to_unsigned(to_integer(cand_bits(rt.sub * 3 + rt.slot)) + cand_mbits(rt.sub * 3 + rt.slot), 12), 28);
            end if;
            if j_pend = '1' then
                jv := j_ssd16 + j_prod;
                if jv < best_j(j_pend_sub) then
                    best_j(j_pend_sub) <= jv; best_slot(j_pend_sub) <= j_pend_idx - j_pend_sub * 3;
                end if;
            end if;

            ----------------------------------------------------------
            -- SATD / SAD results
            ----------------------------------------------------------
            if ab2_tag.valid = '1' then
                case ab2_tag.op is
                    when OP_SCR16 => acc16(ab2_tag.mode) <= acc16(ab2_tag.mode) + ab2;
                    when OP_SCRC  => accc(ab2_tag.mode)  <= accc(ab2_tag.mode)  + ab2;
                    when others => null;
                end case;
            end if;
            -- SCR4: penalise, then insert next cycle
            ab3_tag <= TAG_NONE;
            if ab2_tag.valid = '1' and ab2_tag.op = OP_SCR4 then
                ab3_tag <= ab2_tag;
                if ab2_tag.mode = pred_mode(ab2_tag.sub) then ab3 <= ab2 + resize(pen1, 24); else ab3 <= ab2 + resize(pen4, 24); end if;
            end if;
            if ab3_tag.valid = '1' then
                case ab3_tag.op is
                    when OP_SCR4 =>
                        sb := ab3_tag.sub;
                        c := ab3;
                        -- sorted insert (ties go after equal costs), keep the best 3
                        p := 0;
                        if rk_n(sb) > 0 and rk_cost(sb * 3 + 0) <= c then p := 1; end if;
                        if rk_n(sb) > 1 and rk_cost(sb * 3 + 1) <= c then p := 2; end if;
                        if rk_n(sb) > 2 and rk_cost(sb * 3 + 2) <= c then p := 3; end if;
                        if p < 3 then
                            if p <= 1 then rk_cost(sb * 3 + 2) <= rk_cost(sb * 3 + 1); rk_mode(sb * 3 + 2) <= rk_mode(sb * 3 + 1); end if;
                            if p = 0 then rk_cost(sb * 3 + 1) <= rk_cost(sb * 3 + 0); rk_mode(sb * 3 + 1) <= rk_mode(sb * 3 + 0); end if;
                            if p = 2 then rk_cost(sb * 3 + 2) <= c; rk_mode(sb * 3 + 2) <= ab3_tag.mode;
                            elsif p = 1 then rk_cost(sb * 3 + 1) <= c; rk_mode(sb * 3 + 1) <= ab3_tag.mode;
                            else rk_cost(sb * 3 + 0) <= c; rk_mode(sb * 3 + 0) <= ab3_tag.mode;
                            end if;
                            if rk_n(sb) < 3 then rk_n(sb) <= rk_n(sb) + 1; end if;
                        end if;
                    when others => null;
                end case;
            end if;

            ----------------------------------------------------------
            -- sequencer
            ----------------------------------------------------------
            case st is
                when S_IDLE =>
                    if start_i = '1' and ost = S_OIDLE then
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
                        cm <= 0; cb <= 0; cp <= 0; cs <= 0;
                        csm <= 0; csp <= 1; csq <= 0; cs_done <= '0';
                        st <= S_I16_SCR;
                    end if;

                -- I_16x16 screen: modes 0..3 (available ones), 16 blocks each
                when S_I16_SCR =>
                    ok := (cm = 2) or (cm = 0 and a_top = '1') or (cm = 1 and a_left = '1') or
                          (cm = 3 and a_top = '1' and a_left = '1' and a_tl = '1');
                    if cm > 3 then
                        if wait_cnt = 0 then st <= S_I16_PICK; else wait_cnt <= wait_cnt - 1; end if;
                    elsif ok then
                        ht := ('1', OP_SCR16, 0, cb, cm, 0, '0', 0); issue := true;
                        if cb = 15 then cb <= 0; cm <= cm + 1; wait_cnt <= W_SCR16; else cb <= cb + 1; end if;
                    else
                        cm <= cm + 1;
                    end if;
                when S_I16_PICK =>
                    -- same result as the C scan (modes 0,1,2,3 in order, strict
                    -- '<', ties keep the earlier mode) done pairwise in two
                    -- cycles; an unavailable mode counts as +inf
                    pk_a <= 0; pk_va <= (others => '1');
                    if a_top = '1' then pk_va <= acc16(0); end if;
                    if a_left = '1' and (a_top = '0' or acc16(1) < acc16(0)) then pk_a <= 1; pk_va <= acc16(1); end if;
                    pk_b <= 2; pk_vb <= acc16(2);
                    if a_top = '1' and a_left = '1' and a_tl = '1' and acc16(3) < acc16(2) then pk_b <= 3; pk_vb <= acc16(3); end if;
                    st <= S_I16_PICK2;
                when S_I16_PICK2 =>
                    if pk_vb < pk_va then mode16 <= pk_b; else mode16 <= pk_a; end if;
                    cb <= 0;
                    st <= S_I16_FWD;
                when S_I16_FWD =>
                    if cb > 15 then
                        if wait_cnt = 0 then cb <= 0; st <= S_I16_HAD; else wait_cnt <= wait_cnt - 1; end if;
                    else
                        ht := ('1', OP_FWD16, 0, cb, mode16, 0, '0', 0); issue := true;
                        if cb = 15 then wait_cnt <= W_FWD16; end if;
                        cb <= cb + 1;
                    end if;
                when S_I16_HAD =>
                    if cb = 0 then
                        if phase = '0' then
                            ht := ('1', OP_HAD16, 0, 0, 0, 0, '0', 0); issue := true;
                            wait_cnt <= W_HAD16; cb <= 1;
                        end if;
                    elsif wait_cnt = 0 then
                        cb <= 0; st <= S_I16_INV;
                    else
                        wait_cnt <= wait_cnt - 1;
                    end if;
                when S_I16_INV =>
                    if cb > 15 then
                        if wait_cnt = 0 then step <= 0; cc <= 0; st <= S_I4_PREP; else wait_cnt <= wait_cnt - 1; end if;
                    else
                        ht := ('1', OP_INV16, 0, cb, 0, 0, '1', 0); issue := true;
                        if cb = 15 then wait_cnt <= W_INV16; end if;
                        cb <= cb + 1;
                    end if;

                -- I_4x4: wavefront steps of one or two independent blocks
                when S_I4_PREP =>
                    -- cc = 0: first block of the step; cc = 1: second (if any)
                    sidx := STEP_TAB(step, cc);
                    if sidx < 0 then s := 0; else s := sidx; end if;
                    br := SCAN_BR(s); bc := SCAN_BC(s);
                    cur_blk(cc) <= br * 4 + bc;
                    at := '1'; al := '1';
                    if br = 0 then at := a_top; end if;
                    if bc = 0 then al := a_left; end if;
                    nb_at(cc) <= at; nb_al(cc) <= al;
                    -- top 0..3
                    if br > 0 then
                        topv(31 downto 0) := r4((br - 1) * 4 + bc)(127 downto 96);
                    else
                        topv(31 downto 0) := top_y(bc * 32 + 31 downto bc * 32);
                    end if;
                    -- top-right 4..7
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
                    -- left
                    if bc > 0 then
                        leftv := byte_of(r4(br * 4 + bc - 1), 15) & byte_of(r4(br * 4 + bc - 1), 11) &
                                 byte_of(r4(br * 4 + bc - 1), 7) & byte_of(r4(br * 4 + bc - 1), 3);
                    else
                        leftv := left_y(br * 32 + 31 downto br * 32);
                    end if;
                    -- top-left
                    if br > 0 and bc > 0 then tlv := byte_of(r4((br - 1) * 4 + bc - 1), 15);
                    elsif br > 0 then tlv := byte_of(left_y, br * 4 - 1);
                    elsif bc > 0 then tlv := byte_of(top_y, bc * 4 - 1);
                    else tlv := tl_y;
                    end if;
                    nb_top(cc) <= topv; nb_left(cc) <= leftv; nb_tl(cc) <= tlv;
                    -- predIntra4x4PredMode
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
                    rk_n(cc) <= 0;
                    if cc = 0 and STEP_TAB(step, 1) >= 0 then
                        cc <= 1; nsub <= 2;
                    else
                        if cc = 0 then nsub <= 1; end if;
                        cc <= 0; cm <= 0;
                        st <= S_I4_SCR;
                    end if;
                when S_I4_SCR =>
                    -- cc = sub-block being screened, cm = mode
                    if cm > 8 then
                        if cc = 0 and nsub = 2 then
                            cc <= 1; cm <= 0;
                        elsif wait_cnt = 0 then
                            for k in 0 to 1 loop
                                if rk_n(k) < SHORTLIST then nfull(k) <= rk_n(k); else nfull(k) <= SHORTLIST; end if;
                            end loop;
                            if nsub = 1 then
                                if rk_n(0) < SHORTLIST then nfull_tot <= rk_n(0); else nfull_tot <= SHORTLIST; end if;
                            else
                                nf := SHORTLIST; if rk_n(0) < SHORTLIST then nf := rk_n(0); end if;
                                if rk_n(1) < SHORTLIST then nfull_tot <= nf + rk_n(1); else nfull_tot <= nf + SHORTLIST; end if;
                            end if;
                            ck <= 0; cc <= 0;
                            best_j <= (others => (others => '1')); best_slot <= (others => 0);
                            st <= S_I4_FULL;
                        else
                            wait_cnt <= wait_cnt - 1;
                        end if;
                    else
                        ok := true;
                        if (cm = 0 or cm = 3 or cm = 7) and nb_at(cc) = '0' then ok := false; end if;
                        if (cm = 1 or cm = 8) and nb_al(cc) = '0' then ok := false; end if;
                        if (cm = 4 or cm = 5 or cm = 6) and not (nb_at(cc) = '1' and nb_al(cc) = '1') then ok := false; end if;
                        if ok then
                            ht := ('1', OP_SCR4, 0, cur_blk(cc), cm, 0, '0', cc); issue := true;
                        end if;
                        if cm = 8 then wait_cnt <= W_SCR4; end if;
                        cm <= cm + 1;
                    end if;
                when S_I4_FULL =>
                    -- ck counts issued candidates over both sub-blocks; cc = current sub
                    if ck >= nfull_tot then
                        if wait_cnt = 0 then
                            cc <= 0; st <= S_I4_COMMIT;
                        else
                            wait_cnt <= wait_cnt - 1;
                            -- chroma screen beats in the idle window (no p4 beats in flight)
                            if cs_done = '0' and wait_cnt >= 4 then
                                cs_ok := (csm = 0) or (csm = 1 and a_left = '1') or (csm = 2 and a_top = '1') or
                                         (csm = 3 and a_top = '1' and a_left = '1' and a_tl = '1');
                                if csm > 3 then
                                    cs_done <= '1';
                                elsif cs_ok then
                                    ht := ('1', OP_SCRC, csp, csq, csm, 0, '0', 0); issue := true;
                                    if csq = 3 then
                                        csq <= 0;
                                        if csp = 2 then csp <= 1; csm <= csm + 1; else csp <= 2; end if;
                                    else
                                        csq <= csq + 1;
                                    end if;
                                else
                                    csm <= csm + 1;
                                end if;
                            end if;
                        end if;
                    elsif nfull_tot = 0 then
                        wait_cnt <= 0;
                    elsif phase = '0' then
                        if cc = 0 and ck >= nfull(0) then
                            cc <= 1;
                        else
                            kc := ck - cc * nfull(0);
                            ht := ('1', OP_FULL4, 0, cur_blk(cc), rk_mode(cc * 3 + kc), kc, '0', cc); issue := true;
                            if rk_mode(cc * 3 + kc) = pred_mode(cc) then cand_mbits(cc * 3 + kc) <= 1; else cand_mbits(cc * 3 + kc) <= 4; end if;
                            if ck = nfull_tot - 1 then wait_cnt <= W_FULL4; end if;
                            ck <= ck + 1;
                        end if;
                    end if;
                when S_I4_COMMIT =>
                    modes4(4 * cur_blk(cc) + 3 downto 4 * cur_blk(cc)) <= std_logic_vector(to_unsigned(rk_mode(cc * 3 + best_slot(cc)), 4));
                    bits_b <= bits_b + resize(cand_bits(cc * 3 + best_slot(cc)), 16);
                    nz4(cur_blk(cc)) <= cand_nz(cc * 3 + best_slot(cc));
                    if cc = 0 and nsub = 2 then
                        cc <= 1;
                    elsif step = 9 then
                        st <= S_PICK;
                    else
                        step <= step + 1; cc <= 0; st <= S_I4_PREP;
                    end if;

                when S_PICK =>
                    if bits_a <= bits_b then is_i4 <= '0'; else is_i4 <= '1'; end if;
                    wait_cnt <= W_SCRC;
                    st <= S_C_SCR;

                -- chroma screen: whatever the I4 idle windows did not cover
                when S_C_SCR =>
                    if csm > 3 or cs_done = '1' then
                        if wait_cnt = 0 then st <= S_C_PICK; else wait_cnt <= wait_cnt - 1; end if;
                    else
                        cs_ok := (csm = 0) or (csm = 1 and a_left = '1') or (csm = 2 and a_top = '1') or
                                 (csm = 3 and a_top = '1' and a_left = '1' and a_tl = '1');
                        if cs_ok then
                            ht := ('1', OP_SCRC, csp, csq, csm, 0, '0', 0); issue := true;
                            if csq = 3 then
                                csq <= 0;
                                if csp = 2 then csp <= 1; csm <= csm + 1; else csp <= 2; end if;
                            else
                                csq <= csq + 1;
                            end if;
                        else
                            csm <= csm + 1;
                        end if;
                        wait_cnt <= W_SCRC;
                    end if;
                when S_C_PICK =>
                    pk_a <= 0; pk_va <= accc(0);
                    if a_left = '1' and accc(1) < accc(0) then pk_a <= 1; pk_va <= accc(1); end if;
                    pk_b <= 2; pk_vb <= (others => '1');
                    if a_top = '1' then pk_vb <= accc(2); end if;
                    if a_top = '1' and a_left = '1' and a_tl = '1' and (a_top = '0' or accc(3) < accc(2)) then pk_b <= 3; pk_vb <= accc(3); end if;
                    st <= S_C_PICK2;
                when S_C_PICK2 =>
                    if pk_vb < pk_va then modec <= pk_b; else modec <= pk_a; end if;
                    cp <= 1; cb <= 0;
                    st <= S_C_FWD;
                when S_C_FWD =>
                    if cp = 0 then
                        if wait_cnt = 0 then cp <= 1; cb <= 0; st <= S_C_HAD; else wait_cnt <= wait_cnt - 1; end if;
                    else
                        ht := ('1', OP_FWDC, cp, cb, modec, 0, '0', 0); issue := true;
                        if cb = 3 then
                            cb <= 0;
                            if cp = 2 then cp <= 0; wait_cnt <= W_FWDC; else cp <= 2; end if;
                        else
                            cb <= cb + 1;
                        end if;
                    end if;
                when S_C_HAD =>
                    if cp = 0 then
                        if wait_cnt = 0 then cp <= 1; cb <= 0; st <= S_C_INV; else wait_cnt <= wait_cnt - 1; end if;
                    elsif phase = '0' then
                        ht := ('1', OP_HADC, cp, 0, 0, 0, '0', 0); issue := true;
                        if cp = 2 then cp <= 0; wait_cnt <= W_HADC; else cp <= 2; end if;
                    end if;
                when S_C_INV =>
                    if cp = 0 then
                        if wait_cnt = 0 then st <= S_DONE; else wait_cnt <= wait_cnt - 1; end if;
                    else
                        ht := ('1', OP_INVC, cp, cb, 0, 0, '1', 0); issue := true;
                        if cb = 3 then
                            cb <= 0;
                            if cp = 2 then cp <= 0; wait_cnt <= W_INVC; else cp <= 2; end if;
                        else
                            cb <= cb + 1;
                        end if;
                    end if;

                when S_DONE =>
                    done_q  <= '1';
                    o_start <= '1';
                    st <= S_IDLE;
            end case;

            if issue then
                issuing <= '1';
                head_tag <= ht;
            end if;
        end if;
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
                        o_is4 <= is_i4; o_item <= 0; rec_valid_q <= '1'; ost <= S_OREC;
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
                s := it; ix := SCAN_BR(s) * 4 + SCAN_BC(s); la := 32 + ix;
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
        -- recon items: Y 0..15 (r4 or rec_mem), U 16..19, V 20..23
        if it < 16 then
            rec_plane_o <= "00"; rec_idx_o <= to_unsigned(it, 4); rec_ra <= it;
        elsif it < 20 then
            rec_plane_o <= "01"; rec_idx_o <= to_unsigned(it - 16, 4); rec_ra <= it;
        else
            rec_plane_o <= "10"; rec_idx_o <= to_unsigned(it - 20, 4); rec_ra <= it;
        end if;
    end process;

    blk_levels_o <= unpack13(lvl_rd);
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

end architecture;
