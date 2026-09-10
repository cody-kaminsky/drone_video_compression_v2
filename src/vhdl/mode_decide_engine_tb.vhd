--------------------------------------------------------------------------------
-- mode_decide_engine_tb.vhd — self-checking testbench for mode_decide_engine.
-- Replays build/mode_decide_vectors.txt (per-MB records dumped by the C
-- reference with DCC_DUMP_MB) and checks the decision, every level block
-- and every reconstructed block against the reference.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;
use work.cavlc_pkg.all;

entity mode_decide_engine_tb is
    generic (
        VEC_FILE : string  := "build/mode_decide_vectors.txt";
        MAX_MBS  : natural := 100000;
        DEBUG    : boolean := false
    );
end entity;

architecture sim of mode_decide_engine_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal start_i, busy_o, done_o, stream_done_o : std_logic := '0';
    signal qp_y_i, qp_c_i : unsigned(5 downto 0) := (others => '0');
    signal src_y_i : std_logic_vector(2047 downto 0) := (others => '0');
    signal src_u_i, src_v_i : std_logic_vector(511 downto 0) := (others => '0');
    signal src_data_i : std_logic_vector(127 downto 0) := (others => '0');
    signal stream_busy_o : std_logic;
    signal top_y_i, left_y_i : std_logic_vector(127 downto 0) := (others => '0');
    signal tr_y_i : std_logic_vector(31 downto 0) := (others => '0');
    signal tl_y_i, tl_u_i, tl_v_i : std_logic_vector(7 downto 0) := (others => '0');
    signal avail_top_i, avail_left_i, avail_tl_i, avail_tr_i : std_logic := '0';
    signal top_u_i, left_u_i, top_v_i, left_v_i : std_logic_vector(63 downto 0) := (others => '0');
    signal mode4_top_i, mode4_left_i : std_logic_vector(15 downto 0) := (others => '0');
    signal is_i4x4_o, chroma_dc_nz_o, chroma_ac_nz_o : std_logic;
    signal mode16_o, mode_chroma_o : unsigned(1 downto 0);
    signal modes4_o : std_logic_vector(63 downto 0);
    signal luma_nz_o : std_logic_vector(15 downto 0);
    signal bits_a_o, bits_b_o : unsigned(15 downto 0);
    signal blk_valid_o, blk_ready_i, rec_valid_o, rec_ready_i : std_logic := '0';
    signal blk_plane_o, rec_plane_o : unsigned(1 downto 0);
    signal blk_kind_o : std_logic;
    signal blk_idx_o, rec_idx_o : unsigned(3 downto 0);
    signal blk_levels_o : level_array_t;
    signal rec_data_o : std_logic_vector(127 downto 0);
    signal cycle : natural := 0;
    signal mb_cycles : natural := 0;
    signal total_cycles : natural := 0;
begin
    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.mode_decide_engine
        generic map (DEBUG => DEBUG)
        port map (clk => clk, rst_n => rst_n, start_i => start_i, busy_o => busy_o, done_o => done_o,
                  stream_done_o => stream_done_o, qp_y_i => qp_y_i, qp_c_i => qp_c_i,
                  src_data_i => src_data_i, stream_busy_o => stream_busy_o,
                  top_y_i => top_y_i, tr_y_i => tr_y_i, tl_y_i => tl_y_i, left_y_i => left_y_i,
                  avail_top_i => avail_top_i, avail_left_i => avail_left_i, avail_tl_i => avail_tl_i,
                  avail_tr_i => avail_tr_i, top_u_i => top_u_i, left_u_i => left_u_i, tl_u_i => tl_u_i,
                  top_v_i => top_v_i, left_v_i => left_v_i, tl_v_i => tl_v_i,
                  mode4_top_i => mode4_top_i, mode4_left_i => mode4_left_i,
                  is_i4x4_o => is_i4x4_o, mode16_o => mode16_o, modes4_o => modes4_o,
                  mode_chroma_o => mode_chroma_o, luma_nz_o => luma_nz_o,
                  chroma_dc_nz_o => chroma_dc_nz_o, chroma_ac_nz_o => chroma_ac_nz_o,
                  bits_a_o => bits_a_o, bits_b_o => bits_b_o, dbg_j_o => open,
                  blk_valid_o => blk_valid_o, blk_ready_i => blk_ready_i, blk_plane_o => blk_plane_o,
                  blk_kind_o => blk_kind_o, blk_idx_o => blk_idx_o, blk_levels_o => blk_levels_o,
                  rec_valid_o => rec_valid_o, rec_ready_i => rec_ready_i, rec_plane_o => rec_plane_o,
                  rec_idx_o => rec_idx_o, rec_data_o => rec_data_o);

    cyc_p : process(clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            if (cycle mod 3) = 1 then blk_ready_i <= '0'; rec_ready_i <= '0'; else blk_ready_i <= '1'; rec_ready_i <= '1'; end if;
        end if;
    end process;

    main_p : process
        file vec_f : text;
        variable L : line;
        variable tag : string(1 to 2);
        variable val : integer;
        variable open_status : file_open_status;
        variable nmb, mc : natural := 0;
        variable mb_r, mb_c, qpy, qpc : integer;
        variable e_is4, e_m16, e_mc, e_cbpl, e_cbpc, e_ba, e_bb : integer;
        type i16x16 is array (0 to 15, 0 to 15) of integer;
        type i4x16  is array (0 to 3, 0 to 15) of integer;
        variable e_modes4 : integer_vector(0 to 15);
        variable e_ly : i16x16;
        variable e_ld : integer_vector(0 to 15);
        variable e_lu, e_lv : i4x16;
        variable e_du, e_dv : integer_vector(0 to 3);
        variable e_ry : integer_vector(0 to 255);
        variable e_ru, e_rv : integer_vector(0 to 63);
        variable e_nz : std_logic_vector(15 downto 0);
        variable e_cdc, e_cac : std_logic;
        variable v, b, k, cnt, it, pl, ix : integer;
        variable got : integer;
        variable bad : boolean;
        variable seen : boolean;
        variable t0 : natural;

        procedure rd_bytes_into(sig_hi : integer; n : integer; signal dst : out std_logic_vector) is
        begin
            null;
        end procedure;
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        file_open(open_status, vec_f, VEC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & VEC_FILE severity failure;
        while not endfile(vec_f) and nmb < MAX_MBS loop
            readline(vec_f, L);
            if L'length = 0 then next; end if;
            read(L, tag);
            assert tag = "MB" report "expected MB, got " & tag severity failure;
            read(L, mb_r); read(L, mb_c); read(L, qpy); read(L, qpc);
            qp_y_i <= to_unsigned(qpy, 6); qp_c_i <= to_unsigned(qpc, 6);
            -- SY: 256 samples row-major -> block words
            readline(vec_f, L); read(L, tag);
            for i in 0 to 255 loop
                read(L, val);
                b := (i / 16 / 4) * 4 + ((i mod 16) / 4);
                k := ((i / 16) mod 4) * 4 + (i mod 4);
                src_y_i(128 * b + 8 * k + 7 downto 128 * b + 8 * k) <= std_logic_vector(to_unsigned(val, 8));
            end loop;
            readline(vec_f, L); read(L, tag);
            for i in 0 to 63 loop
                read(L, val);
                b := (i / 8 / 4) * 2 + ((i mod 8) / 4);
                k := ((i / 8) mod 4) * 4 + (i mod 4);
                src_u_i(128 * b + 8 * k + 7 downto 128 * b + 8 * k) <= std_logic_vector(to_unsigned(val, 8));
            end loop;
            readline(vec_f, L); read(L, tag);
            for i in 0 to 63 loop
                read(L, val);
                b := (i / 8 / 4) * 2 + ((i mod 8) / 4);
                k := ((i / 8) mod 4) * 4 + (i mod 4);
                src_v_i(128 * b + 8 * k + 7 downto 128 * b + 8 * k) <= std_logic_vector(to_unsigned(val, 8));
            end loop;
            -- NY
            readline(vec_f, L); read(L, tag);
            read(L, val); if val = 1 then avail_top_i <= '1'; else avail_top_i <= '0'; end if;
            read(L, val); if val = 1 then avail_left_i <= '1'; else avail_left_i <= '0'; end if;
            read(L, val); if val = 1 then avail_tl_i <= '1'; else avail_tl_i <= '0'; end if;
            read(L, val); if val = 1 then avail_tr_i <= '1'; else avail_tr_i <= '0'; end if;
            for i in 0 to 15 loop read(L, val); top_y_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            for i in 0 to 15 loop read(L, val); left_y_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            read(L, val); tl_y_i <= std_logic_vector(to_unsigned(val, 8));
            for i in 0 to 3 loop read(L, val); tr_y_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            -- NC
            readline(vec_f, L); read(L, tag);
            read(L, val); read(L, val); read(L, val);
            for i in 0 to 7 loop read(L, val); top_u_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            for i in 0 to 7 loop read(L, val); left_u_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            read(L, val); tl_u_i <= std_logic_vector(to_unsigned(val, 8));
            for i in 0 to 7 loop read(L, val); top_v_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            for i in 0 to 7 loop read(L, val); left_v_i(8 * i + 7 downto 8 * i) <= std_logic_vector(to_unsigned(val, 8)); end loop;
            read(L, val); tl_v_i <= std_logic_vector(to_unsigned(val, 8));
            -- NM
            readline(vec_f, L); read(L, tag);
            for i in 0 to 3 loop read(L, val); mode4_top_i(4 * i + 3 downto 4 * i) <= std_logic_vector(to_unsigned(val, 4)); end loop;
            for i in 0 to 3 loop read(L, val); mode4_left_i(4 * i + 3 downto 4 * i) <= std_logic_vector(to_unsigned(val, 4)); end loop;
            -- B
            readline(vec_f, L); read(L, tag);
            read(L, e_ba); read(L, e_bb);
            -- O
            readline(vec_f, L); read(L, tag);
            read(L, e_is4); read(L, e_m16); read(L, e_mc); read(L, e_cbpl); read(L, e_cbpc);
            for i in 0 to 15 loop read(L, e_modes4(i)); end loop;
            -- LY LD LU LV DU DV
            readline(vec_f, L); read(L, tag);
            for bb in 0 to 15 loop for kk in 0 to 15 loop read(L, e_ly(bb, kk)); end loop; end loop;
            readline(vec_f, L); read(L, tag);
            for kk in 0 to 15 loop read(L, e_ld(kk)); end loop;
            readline(vec_f, L); read(L, tag);
            for bb in 0 to 3 loop for kk in 0 to 15 loop read(L, e_lu(bb, kk)); end loop; end loop;
            readline(vec_f, L); read(L, tag);
            for bb in 0 to 3 loop for kk in 0 to 15 loop read(L, e_lv(bb, kk)); end loop; end loop;
            readline(vec_f, L); read(L, tag);
            for kk in 0 to 3 loop read(L, e_du(kk)); end loop;
            readline(vec_f, L); read(L, tag);
            for kk in 0 to 3 loop read(L, e_dv(kk)); end loop;
            -- RY RU RV
            readline(vec_f, L); read(L, tag);
            for i in 0 to 255 loop read(L, e_ry(i)); end loop;
            readline(vec_f, L); read(L, tag);
            for i in 0 to 63 loop read(L, e_ru(i)); end loop;
            readline(vec_f, L); read(L, tag);
            for i in 0 to 63 loop read(L, e_rv(i)); end loop;

            -- expected nz flags
            e_nz := (others => '0'); e_cdc := '0'; e_cac := '0';
            for bb in 0 to 15 loop
                for kk in 0 to 15 loop
                    if e_ly(bb, kk) /= 0 and (e_is4 = 1 or kk > 0) then e_nz(bb) := '1'; end if;
                end loop;
            end loop;
            for bb in 0 to 3 loop
                for kk in 1 to 15 loop
                    if e_lu(bb, kk) /= 0 or e_lv(bb, kk) /= 0 then e_cac := '1'; end if;
                end loop;
            end loop;
            for kk in 0 to 3 loop if e_du(kk) /= 0 or e_dv(kk) /= 0 then e_cdc := '1'; end if; end loop;

            -- run: one loop handles the decision (at done_o) and both output
            -- streams, since the first item can be taken on the edge where
            -- done_o is first visible
            wait until rising_edge(clk);
            t0 := cycle;
            start_i <= '1';
            wait until rising_edge(clk);
            start_i <= '0';
            bad := false;
            seen := false;
            cnt := 0;
            it := 0;
            loop
                -- source block stream: Y 0..15, U 0..3, V 0..3, one per cycle
                if it < 16 then src_data_i <= src_y_i(128 * it + 127 downto 128 * it);
                elsif it < 20 then src_data_i <= src_u_i(128 * (it - 16) + 127 downto 128 * (it - 16));
                elsif it < 24 then src_data_i <= src_v_i(128 * (it - 20) + 127 downto 128 * (it - 20));
                else src_data_i <= (others => '0');
                end if;
                it := it + 1;
                wait until rising_edge(clk);
                if done_o = '1' and not seen then
                    seen := true;
                    mb_cycles <= cycle - t0;
                    total_cycles <= total_cycles + (cycle - t0);
                    if to_integer(bits_a_o) /= e_ba or to_integer(bits_b_o) /= e_bb then
                        bad := true;
                        report "MB " & integer'image(nmb) & " bits mismatch got " & integer'image(to_integer(bits_a_o)) & "/" &
                               integer'image(to_integer(bits_b_o)) & " exp " & integer'image(e_ba) & "/" & integer'image(e_bb) severity error;
                    end if;
                    if (is_i4x4_o = '1') /= (e_is4 = 1) then bad := true; report "MB " & integer'image(nmb) & " is_i4x4 mismatch" severity error; end if;
                    if e_is4 = 0 and to_integer(mode16_o) /= e_m16 then bad := true; report "MB " & integer'image(nmb) & " mode16 mismatch got " & integer'image(to_integer(mode16_o)) & " exp " & integer'image(e_m16) severity error; end if;
                    if to_integer(mode_chroma_o) /= e_mc then bad := true; report "MB " & integer'image(nmb) & " mode_chroma mismatch got " & integer'image(to_integer(mode_chroma_o)) & " exp " & integer'image(e_mc) severity error; end if;
                    if e_is4 = 1 then
                        for i in 0 to 15 loop
                            if to_integer(unsigned(modes4_o(4 * i + 3 downto 4 * i))) /= e_modes4(i) then
                                bad := true; report "MB " & integer'image(nmb) & " modes4[" & integer'image(i) & "] got " &
                                    integer'image(to_integer(unsigned(modes4_o(4 * i + 3 downto 4 * i)))) & " exp " & integer'image(e_modes4(i)) severity error;
                            end if;
                        end loop;
                    end if;
                    if luma_nz_o /= e_nz then bad := true; report "MB " & integer'image(nmb) & " luma_nz mismatch" severity error; end if;
                    if chroma_dc_nz_o /= e_cdc or chroma_ac_nz_o /= e_cac then bad := true; report "MB " & integer'image(nmb) & " chroma nz mismatch" severity error; end if;
                end if;
                if blk_valid_o = '1' and blk_ready_i = '1' then
                    pl := to_integer(blk_plane_o); ix := to_integer(blk_idx_o);
                    for kk in 0 to 15 loop
                        got := to_integer(blk_levels_o(kk));
                        if pl = 0 then
                            if blk_kind_o = '1' then v := e_ld(kk); else v := e_ly(ix, kk); end if;
                        elsif pl = 1 then
                            if blk_kind_o = '1' then if kk < 4 then v := e_du(kk); else v := 0; end if; else v := e_lu(ix, kk); end if;
                        else
                            if blk_kind_o = '1' then if kk < 4 then v := e_dv(kk); else v := 0; end if; else v := e_lv(ix, kk); end if;
                        end if;
                        if got /= v then
                            bad := true;
                            report "MB " & integer'image(nmb) & " level mismatch item " & integer'image(cnt) & " plane " & integer'image(pl) &
                                   " kind " & std_logic'image(blk_kind_o) & " idx " & integer'image(ix) & " k " & integer'image(kk) &
                                   " got " & integer'image(got) & " exp " & integer'image(v) severity error;
                        end if;
                    end loop;
                    cnt := cnt + 1;
                end if;
                if rec_valid_o = '1' and rec_ready_i = '1' then
                    pl := to_integer(rec_plane_o); ix := to_integer(rec_idx_o);
                    for kk in 0 to 15 loop
                        got := to_integer(unsigned(rec_data_o(8 * kk + 7 downto 8 * kk)));
                        if pl = 0 then
                            v := e_ry(((ix / 4) * 4 + kk / 4) * 16 + (ix mod 4) * 4 + (kk mod 4));
                        elsif pl = 1 then
                            v := e_ru(((ix / 2) * 4 + kk / 4) * 8 + (ix mod 2) * 4 + (kk mod 4));
                        else
                            v := e_rv(((ix / 2) * 4 + kk / 4) * 8 + (ix mod 2) * 4 + (kk mod 4));
                        end if;
                        if got /= v then
                            bad := true;
                            report "MB " & integer'image(nmb) & " recon mismatch plane " & integer'image(pl) & " idx " & integer'image(ix) &
                                   " k " & integer'image(kk) & " got " & integer'image(got) & " exp " & integer'image(v) severity error;
                        end if;
                    end loop;
                end if;
                exit when stream_done_o = '1';
            end loop;
            if (e_is4 = 1 and cnt /= 26) or (e_is4 = 0 and cnt /= 27) then
                bad := true; report "MB " & integer'image(nmb) & " block count " & integer'image(cnt) severity error;
            end if;
            if bad then mc := mc + 1; end if;
            nmb := nmb + 1;
            if (nmb mod 20) = 0 then
                report "progress: " & integer'image(nmb) & " MBs, " & integer'image(mc) & " bad, last " &
                       integer'image(mb_cycles) & " cycles" severity note;
            end if;
        end loop;
        file_close(vec_f);
        report "cycles per MB (decision only, avg): " & integer'image(total_cycles / nmb) severity note;
        if mc = 0 then
            report "PASS - " & integer'image(nmb) & " macroblocks verified" severity note;
        else
            report "FAIL - " & integer'image(mc) & " bad macroblocks over " & integer'image(nmb) severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process begin
        wait for 200 ms;
        report "watchdog timeout" severity failure;
        std.env.finish;
        wait;
    end process;
end architecture;
