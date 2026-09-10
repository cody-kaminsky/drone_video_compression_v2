--------------------------------------------------------------------------------
-- line_buffer_tb.vhd — self-checking testbench for line_buffer.
-- Replays build/line_buffer_vectors.txt (tools/gen_line_buffer_vectors.c):
-- frame/row starts, fetches (compared against the C line buffer's
-- gathers) and commits.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity line_buffer_tb is
    generic (VEC_FILE : string := "build/line_buffer_vectors.txt");
end entity;

architecture sim of line_buffer_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal frame_start_i, row_start_i : std_logic := '0';
    signal mbs_w_i : unsigned(7 downto 0) := (others => '0');
    signal fetch_valid_i : std_logic := '0';
    signal fetch_mb_c_i  : unsigned(7 downto 0) := (others => '0');
    signal fetch_ready_o, nb_valid_o : std_logic;
    signal top_y_o, left_y_o : std_logic_vector(127 downto 0);
    signal tr_y_o : std_logic_vector(31 downto 0);
    signal tl_y_o, tl_u_o, tl_v_o : std_logic_vector(7 downto 0);
    signal top_u_o, top_v_o, left_u_o, left_v_o : std_logic_vector(63 downto 0);
    signal nc_y_top_o, nc_y_left_o : std_logic_vector(19 downto 0);
    signal nc_u_top_o, nc_u_left_o, nc_v_top_o, nc_v_left_o : std_logic_vector(9 downto 0);
    signal mode4_top_o, mode4_left_o : std_logic_vector(15 downto 0);
    signal avail_top_o, avail_left_o, avail_tl_o, avail_tr_o : std_logic;
    signal commit_valid_i : std_logic := '0';
    signal commit_ready_o : std_logic;
    signal commit_mb_c_i  : unsigned(7 downto 0) := (others => '0');
    signal rec_y_bot_i, rec_uv_bot_i, rec_y_right_i, rec_uv_right_i : std_logic_vector(127 downto 0) := (others => '0');
    signal nc_y_i : std_logic_vector(79 downto 0) := (others => '0');
    signal nc_u_i, nc_v_i : std_logic_vector(19 downto 0) := (others => '0');
    signal mode4_i : std_logic_vector(63 downto 0) := (others => '0');
begin
    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.line_buffer
        generic map (MAX_MB_COLS => 8)
        port map (clk => clk, rst_n => rst_n, frame_start_i => frame_start_i, row_start_i => row_start_i,
                  mbs_w_i => mbs_w_i, fetch_valid_i => fetch_valid_i, fetch_mb_c_i => fetch_mb_c_i,
                  fetch_ready_o => fetch_ready_o, nb_valid_o => nb_valid_o,
                  top_y_o => top_y_o, tr_y_o => tr_y_o, tl_y_o => tl_y_o, left_y_o => left_y_o,
                  top_u_o => top_u_o, top_v_o => top_v_o, tl_u_o => tl_u_o, tl_v_o => tl_v_o,
                  left_u_o => left_u_o, left_v_o => left_v_o,
                  nc_y_top_o => nc_y_top_o, nc_y_left_o => nc_y_left_o, nc_u_top_o => nc_u_top_o,
                  nc_u_left_o => nc_u_left_o, nc_v_top_o => nc_v_top_o, nc_v_left_o => nc_v_left_o,
                  mode4_top_o => mode4_top_o, mode4_left_o => mode4_left_o,
                  avail_top_o => avail_top_o, avail_left_o => avail_left_o, avail_tl_o => avail_tl_o,
                  avail_tr_o => avail_tr_o,
                  commit_valid_i => commit_valid_i, commit_ready_o => commit_ready_o, commit_mb_c_i => commit_mb_c_i,
                  rec_y_bot_i => rec_y_bot_i, rec_uv_bot_i => rec_uv_bot_i, rec_y_right_i => rec_y_right_i,
                  rec_uv_right_i => rec_uv_right_i, nc_y_i => nc_y_i, nc_u_i => nc_u_i, nc_v_i => nc_v_i,
                  mode4_i => mode4_i);

    main_p : process
        file vec_f : text;
        variable L : line;
        variable tag : character;
        variable val, c, at, al, atl, atr : integer;
        variable open_status : file_open_status;
        variable nfetch, mc : natural := 0;
        variable exp8  : std_logic_vector(7 downto 0);
        variable exp_top, exp_left : std_logic_vector(127 downto 0);
        variable exp_tr : std_logic_vector(31 downto 0);
        variable exp64 : std_logic_vector(63 downto 0);
        variable e_tu, e_tv, e_lu, e_lv : std_logic_vector(63 downto 0);
        variable e20a, e20b : std_logic_vector(19 downto 0);
        variable e10a, e10b, e10c, e10d : std_logic_vector(9 downto 0);
        variable e16a, e16b : std_logic_vector(15 downto 0);
        variable v80 : std_logic_vector(79 downto 0);
        variable v20 : std_logic_vector(19 downto 0);
        variable v64 : std_logic_vector(63 downto 0);
        variable v128 : std_logic_vector(127 downto 0);

        procedure chk(name : string; got, exp : std_logic_vector; en : boolean) is
        begin
            if en and got /= exp then
                mc := mc + 1;
                report "MISMATCH fetch " & integer'image(nfetch) & " mb_c " & integer'image(c) & " " & name
                    severity error;
            end if;
        end procedure;
        procedure chkb(name : string; got : std_logic; exp : integer) is
        begin
            if (got = '1') /= (exp = 1) then
                mc := mc + 1;
                report "MISMATCH fetch " & integer'image(nfetch) & " mb_c " & integer'image(c) & " " & name
                    severity error;
            end if;
        end procedure;
        procedure rd_bytes(n : integer; v : out std_logic_vector) is
        begin
            for i in 0 to n - 1 loop
                read(L, val);
                v(8 * i + 7 downto 8 * i) := std_logic_vector(to_unsigned(val, 8));
            end loop;
        end procedure;
        procedure rd_nc(n : integer; v : out std_logic_vector) is
        begin
            for i in 0 to n - 1 loop
                read(L, val);
                v(5 * i + 4 downto 5 * i) := std_logic_vector(to_unsigned(val, 5));
            end loop;
        end procedure;
        procedure rd_m4(n : integer; v : out std_logic_vector) is
        begin
            for i in 0 to n - 1 loop
                read(L, val);
                v(4 * i + 3 downto 4 * i) := std_logic_vector(to_unsigned(val, 4));
            end loop;
        end procedure;
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        file_open(open_status, vec_f, VEC_FILE, read_mode);
        assert open_status = open_ok report "could not open " & VEC_FILE severity failure;
        while not endfile(vec_f) loop
            readline(vec_f, L);
            if L'length = 0 then next; end if;
            read(L, tag);
            case tag is
                when 'S' =>
                    read(L, val);
                    mbs_w_i <= to_unsigned(val, 8);
                    frame_start_i <= '1';
                    wait until rising_edge(clk);
                    frame_start_i <= '0';
                    wait until rising_edge(clk);
                when 'R' =>
                    row_start_i <= '1';
                    wait until rising_edge(clk);
                    row_start_i <= '0';
                    wait until rising_edge(clk);
                when 'F' =>
                    read(L, c); read(L, at); read(L, al); read(L, atl); read(L, atr);
                    rd_bytes(16, exp_top); rd_bytes(4, exp_tr);
                    read(L, val); exp8 := std_logic_vector(to_unsigned(val, 8));
                    rd_bytes(16, exp_left);
                    rd_bytes(8, e_tu); rd_bytes(8, e_tv);
                    fetch_mb_c_i  <= to_unsigned(c, 8);
                    fetch_valid_i <= '1';
                    loop wait until rising_edge(clk); exit when fetch_ready_o = '1'; end loop;
                    fetch_valid_i <= '0';
                    loop wait until rising_edge(clk); exit when nb_valid_o = '1'; end loop;
                    wait for 1 ns;
                    chkb("avail_top", avail_top_o, at);
                    chkb("avail_left", avail_left_o, al);
                    chkb("avail_tl", avail_tl_o, atl);
                    chkb("avail_tr", avail_tr_o, atr);
                    chk("top_y", top_y_o, exp_top, at = 1);
                    chk("tr_y", tr_y_o, exp_tr, atr = 1);
                    chk("tl_y", tl_y_o, exp8, true);
                    chk("left_y", left_y_o, exp_left, al = 1);
                    chk("top_u", top_u_o, e_tu, at = 1);
                    chk("top_v", top_v_o, e_tv, at = 1);
                    read(L, val); exp8 := std_logic_vector(to_unsigned(val, 8)); chk("tl_u", tl_u_o, exp8, true);
                    read(L, val); exp8 := std_logic_vector(to_unsigned(val, 8)); chk("tl_v", tl_v_o, exp8, true);
                    rd_bytes(8, e_lu); rd_bytes(8, e_lv);
                    chk("left_u", left_u_o, e_lu, al = 1);
                    chk("left_v", left_v_o, e_lv, al = 1);
                    rd_nc(4, e20a); rd_nc(4, e20b);
                    chk("nc_y_top", nc_y_top_o, e20a, true);
                    chk("nc_y_left", nc_y_left_o, e20b, true);
                    rd_nc(2, e10a); rd_nc(2, e10b); rd_nc(2, e10c); rd_nc(2, e10d);
                    chk("nc_u_top", nc_u_top_o, e10a, true);
                    chk("nc_u_left", nc_u_left_o, e10b, true);
                    chk("nc_v_top", nc_v_top_o, e10c, true);
                    chk("nc_v_left", nc_v_left_o, e10d, true);
                    rd_m4(4, e16a); rd_m4(4, e16b);
                    chk("mode4_top", mode4_top_o, e16a, true);
                    chk("mode4_left", mode4_left_o, e16b, true);
                    nfetch := nfetch + 1;
                when 'C' =>
                    read(L, c);
                    rd_bytes(16, v128); rec_y_bot_i <= v128;
                    rd_bytes(16, v128); rec_uv_bot_i <= v128;
                    rd_bytes(16, v128); rec_y_right_i <= v128;
                    rd_bytes(16, v128); rec_uv_right_i <= v128;
                    rd_nc(16, v80); nc_y_i <= v80;
                    rd_nc(4, v20); nc_u_i <= v20;
                    rd_nc(4, v20); nc_v_i <= v20;
                    rd_m4(16, v64); mode4_i <= v64;
                    commit_mb_c_i  <= to_unsigned(c, 8);
                    commit_valid_i <= '1';
                    loop wait until rising_edge(clk); exit when commit_ready_o = '1'; end loop;
                    commit_valid_i <= '0';
                when others =>
                    report "bad tag" severity failure;
            end case;
        end loop;
        file_close(vec_f);
        wait for 2 * CLK_PERIOD;
        if mc = 0 then
            report "PASS - " & integer'image(nfetch) & " line_buffer fetches verified" severity note;
        else
            report "FAIL - " & integer'image(mc) & " mismatches over " & integer'image(nfetch) & " fetches"
                severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process begin
        wait for 5 ms;
        report "watchdog timeout" severity failure;
        std.env.finish;
        wait;
    end process;
end architecture;
