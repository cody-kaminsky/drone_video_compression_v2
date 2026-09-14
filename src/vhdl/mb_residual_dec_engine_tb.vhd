--------------------------------------------------------------------------------
-- mb_residual_dec_engine_tb.vhd — drive the block sequencer against the C
-- decoder's own residual parser.
--
-- Each vector in build/mb_residual_dec_vectors.txt is a whole macroblock's
-- residual, written with the encoder's block order and nC derivation and read
-- back with dec_mb_residual.
--
-- Three things are checked, and the last two are the ones that matter. The
-- coefficients, obviously. Then the neighbour total_coeff the macroblock
-- hands on, because every following block's nC depends on it and an error
-- there decodes the NEXT macroblock wrongly while this one looks perfect.
-- Then the number of bits retired, because a sequencer can produce entirely
-- correct blocks while leaving the reader one bit out of step.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity mb_residual_dec_engine_tb is
    generic (
        VEC_FILE : string := "build/mb_residual_dec_vectors.txt";
        -- Starve the bit reader hard: the reader must actually run dry, not
        -- merely pause, or the sequencer is never made to hold a decision
        -- across a gap. 0 disables the stalls.
        IN_BP    : integer := 1
    );
end entity;

architecture sim of mb_residual_dec_engine_tb is
    constant CLK_PERIOD : time := 5 ns;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal in_data  : unsigned(7 downto 0) := (others => '0');
    signal in_valid : std_logic := '0';
    signal in_ready : std_logic;
    signal in_last  : std_logic := '0';
    signal peek     : unsigned(31 downto 0);
    signal avail    : std_logic;
    signal consume  : std_logic;
    signal consume_n: unsigned(5 downto 0);
    signal bitpos   : unsigned(31 downto 0);
    signal underrun : std_logic;

    signal start_i   : std_logic := '0';
    signal ready_o   : std_logic;
    signal i4_i      : std_logic := '0';
    signal cbpl_i    : unsigned(3 downto 0) := (others => '0');
    signal cbpc_i    : unsigned(1 downto 0) := (others => '0');
    signal at_i      : std_logic := '0';
    signal al_i      : std_logic := '0';
    signal nct_i, ncl_i   : unsigned(19 downto 0) := (others => '0');
    signal ncut_i, ncul_i : unsigned(9 downto 0) := (others => '0');
    signal ncvt_i, ncvl_i : unsigned(9 downto 0) := (others => '0');

    signal bvalid : std_logic;
    signal bready : std_logic := '0';
    signal bkind  : unsigned(1 downto 0);
    signal bcomp  : std_logic;
    signal bpos   : unsigned(3 downto 0);
    signal btotal : unsigned(4 downto 0);
    signal bcoefs : std_logic_vector(255 downto 0);

    signal done_o : std_logic;
    signal err_o  : std_logic;
    signal errc_o : unsigned(3 downto 0);
    signal ncbot, ncright   : unsigned(19 downto 0);
    signal ncubot, ncuright : unsigned(9 downto 0);
    signal ncvbot, ncvright : unsigned(9 downto 0);

    type byte_arr is array (0 to 2047) of integer;
    shared variable vbytes : byte_arr;
    shared variable vn     : integer := 0;
    signal vec_id : integer := 0;

    signal lfsr    : unsigned(15 downto 0) := x"C0DE";
    signal starved : integer := 0;

begin

    clk <= not clk after CLK_PERIOD / 2;

    rd : entity work.bit_reader
        generic map (PEEK_W => 32)
        port map (clk => clk, rst_n => rst_n,
                  in_data => in_data, in_valid => in_valid,
                  in_ready => in_ready, in_last => in_last,
                  peek_o => peek, avail_o => avail,
                  consume_i => consume, consume_n_i => consume_n,
                  bitpos_o => bitpos, underrun_o => underrun);

    dut : entity work.mb_residual_dec_engine
        port map (clk => clk, rst_n => rst_n,
                  start_i => start_i, ready_o => ready_o,
                  is_i4x4_i => i4_i, cbp_luma_i => cbpl_i,
                  cbp_chroma_i => cbpc_i,
                  avail_top_i => at_i, avail_left_i => al_i,
                  nc_top_i => nct_i, nc_left_i => ncl_i,
                  ncu_top_i => ncut_i, ncu_left_i => ncul_i,
                  ncv_top_i => ncvt_i, ncv_left_i => ncvl_i,
                  peek_i => peek, avail_i => avail,
                  consume_o => consume, consume_n_o => consume_n,
                  blk_valid_o => bvalid, blk_ready_i => bready,
                  blk_kind_o => bkind, blk_comp_o => bcomp,
                  blk_pos_o => bpos, blk_total_o => btotal,
                  blk_coefs_o => bcoefs,
                  done_o => done_o, err_o => err_o, err_code_o => errc_o,
                  nc_bot_o => ncbot, nc_right_o => ncright,
                  ncu_bot_o => ncubot, ncu_right_o => ncuright,
                  ncv_bot_o => ncvbot, ncv_right_o => ncvright);

    ----------------------------------------------------------------------
    feed_p : process(clk)
        variable i      : integer := 0;
        variable id     : integer := -1;
        variable active : boolean := false;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                active   := false;
                in_valid <= '0';
                id       := vec_id;
            else
                lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13)
                                             xor lfsr(12) xor lfsr(10));
                if vec_id /= id then
                    id := vec_id; i := 0; active := true;
                elsif active and in_valid = '1' and in_ready = '1' then
                    i := i + 1;
                end if;
                if active and i < vn + 8
                   and (IN_BP = 0 or lfsr(3 downto 0) = "0000") then
                    in_valid <= '1';
                    if i < vn then
                        in_data <= to_unsigned(vbytes(i), 8);
                    else
                        in_data <= (others => '0');
                    end if;
                else
                    in_valid <= '0';
                end if;
            end if;
        end if;
    end process;

    starve_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '1' and avail = '0' then
                starved <= starved + 1;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    main_p : process
        file f : text;
        variable L : line;
        variable open_status : file_open_status;
        variable v_i4, v_cbpl, v_cbpc, v_at, v_al : integer;
        variable v_nbits, v_nbytes : integer;
        variable nt, nl : integer_vector(0 to 3);
        variable ut, ul, vt, vl : integer_vector(0 to 1);
        variable e_ldc   : integer_vector(0 to 15);
        variable e_luma  : integer_vector(0 to 255);
        variable e_cdc   : integer_vector(0 to 7);
        variable e_cac   : integer_vector(0 to 127);
        variable e_nc    : integer_vector(0 to 15);
        variable e_ncu, e_ncv : integer_vector(0 to 3);
        variable i, j, nvec, bad, blk : integer := 0;
        variable wrong : boolean;
        variable base  : integer;
        variable expc  : integer;
        variable got   : integer;
        variable nc_exp : integer;
    begin
        file_open(open_status, f, VEC_FILE, read_mode);
        assert open_status = open_ok
            report "cannot open " & VEC_FILE severity failure;

        nvec := 0; bad := 0;

        while not endfile(f) loop
            readline(f, L);
            if L'length > 0 then
                read(L, v_i4); read(L, v_cbpl); read(L, v_cbpc);
                read(L, v_at); read(L, v_al);
                for i in 0 to 3 loop read(L, nt(i)); end loop;
                for i in 0 to 3 loop read(L, nl(i)); end loop;
                for i in 0 to 1 loop read(L, ut(i)); end loop;
                for i in 0 to 1 loop read(L, ul(i)); end loop;
                for i in 0 to 1 loop read(L, vt(i)); end loop;
                for i in 0 to 1 loop read(L, vl(i)); end loop;
                read(L, v_nbits); read(L, v_nbytes);
                for i in 0 to v_nbytes - 1 loop read(L, vbytes(i)); end loop;
                for i in 0 to 15  loop read(L, e_ldc(i));  end loop;
                for i in 0 to 255 loop read(L, e_luma(i)); end loop;
                for i in 0 to 7   loop read(L, e_cdc(i));  end loop;
                for i in 0 to 127 loop read(L, e_cac(i));  end loop;
                for i in 0 to 15  loop read(L, e_nc(i));   end loop;
                for i in 0 to 3   loop read(L, e_ncu(i));  end loop;
                for i in 0 to 3   loop read(L, e_ncv(i));  end loop;
                vn := v_nbytes;
                wrong := false;

                rst_n <= '0';
                wait until rising_edge(clk);
                wait until rising_edge(clk);
                rst_n <= '1';
                wait until rising_edge(clk);

                vec_id <= vec_id + 1;
                wait until rising_edge(clk);

                i4_i   <= '1' when v_i4 = 1 else '0';
                cbpl_i <= to_unsigned(v_cbpl, 4);
                cbpc_i <= to_unsigned(v_cbpc, 2);
                at_i   <= '1' when v_at = 1 else '0';
                al_i   <= '1' when v_al = 1 else '0';
                for i in 0 to 3 loop
                    nct_i(5 * i + 4 downto 5 * i) <= to_unsigned(nt(i), 5);
                    ncl_i(5 * i + 4 downto 5 * i) <= to_unsigned(nl(i), 5);
                end loop;
                for i in 0 to 1 loop
                    ncut_i(5 * i + 4 downto 5 * i) <= to_unsigned(ut(i), 5);
                    ncul_i(5 * i + 4 downto 5 * i) <= to_unsigned(ul(i), 5);
                    ncvt_i(5 * i + 4 downto 5 * i) <= to_unsigned(vt(i), 5);
                    ncvl_i(5 * i + 4 downto 5 * i) <= to_unsigned(vl(i), 5);
                end loop;
                start_i <= '1';
                wait until rising_edge(clk);
                start_i <= '0';

                ------------------------------------------------------------
                -- Collect the 27 blocks. Holding ready low between them is
                -- deliberate: the sequencer must stall rather than run on.
                blk := 0;
                i   := 0;
                while blk < 27 and done_o = '0' and i < 200000 loop
                    wait until rising_edge(clk);
                    i := i + 1;
                    if bvalid = '1' then
                        -- Check before accepting.
                        -- The tag as well as the payload: a consumer routes
                        -- on kind, component and position, so a block that is
                        -- right but mislabelled lands in the wrong place.
                        if blk = 0 then
                            if to_integer(bkind) /= 0 then wrong := true; end if;
                            base := -1;
                        elsif blk <= 16 then
                            if to_integer(bkind) /= 1 then wrong := true; end if;
                            base := to_integer(bpos) * 16;
                        elsif blk <= 18 then
                            if to_integer(bkind) /= 2 then wrong := true; end if;
                            if (blk = 18) /= (bcomp = '1') then wrong := true; end if;
                            base := -2;
                        else
                            if to_integer(bkind) /= 3 then wrong := true; end if;
                            if (blk >= 23) /= (bcomp = '1') then wrong := true; end if;
                            if to_integer(bpos) /= (blk - 19) mod 4 then
                                wrong := true;
                            end if;
                            base := -3;
                        end if;
                        if wrong and bad < 10 then
                            report "vector " & integer'image(nvec)
                                 & " block " & integer'image(blk)
                                 & ": wrong tag (kind "
                                 & integer'image(to_integer(bkind))
                                 & " pos " & integer'image(to_integer(bpos))
                                 & ")" severity error;
                        end if;

                        for j in 0 to 15 loop
                            got := to_integer(signed(
                                       bcoefs(j * 16 + 15 downto j * 16)));
                            if blk = 0 then
                                expc := e_ldc(j);
                            elsif blk <= 16 then
                                expc := e_luma(base + j);
                            elsif blk <= 18 then
                                if j < 4 then
                                    expc := e_cdc((blk - 17) * 4 + j);
                                else
                                    expc := 0;
                                end if;
                            else
                                expc := e_cac((blk - 19) * 16 + j);
                            end if;
                            if got /= expc then
                                wrong := true;
                                if bad < 10 then
                                    report "vector " & integer'image(nvec)
                                         & " block " & integer'image(blk)
                                         & " coef " & integer'image(j)
                                         & ": expected " & integer'image(expc)
                                         & " got " & integer'image(got)
                                        severity error;
                                end if;
                                exit;
                            end if;
                        end loop;

                        -- The block's own total_coeff, which is what the next
                        -- block's nC is derived from.
                        if blk >= 1 and blk <= 16 then
                            nc_exp := e_nc(to_integer(bpos));
                        elsif blk >= 19 and blk <= 22 then
                            nc_exp := e_ncu(to_integer(bpos));
                        elsif blk >= 23 then
                            nc_exp := e_ncv(to_integer(bpos));
                        else
                            nc_exp := to_integer(btotal);   -- DC: not stored
                        end if;
                        if to_integer(btotal) /= nc_exp then
                            wrong := true;
                            if bad < 10 then
                                report "vector " & integer'image(nvec)
                                     & " block " & integer'image(blk)
                                     & ": total_coeff expected "
                                     & integer'image(nc_exp) & " got "
                                     & integer'image(to_integer(btotal))
                                    severity error;
                            end if;
                        end if;

                        bready <= '1';
                        wait until rising_edge(clk);
                        bready <= '0';
                        blk := blk + 1;
                    end if;
                end loop;

                ------------------------------------------------------------
                i := 0;
                while done_o = '0' and i < 20000 loop
                    wait until rising_edge(clk);
                    i := i + 1;
                end loop;

                if done_o = '0' then
                    wrong := true;
                    if bad < 10 then
                        report "vector " & integer'image(nvec)
                             & ": engine never asserted done" severity error;
                    end if;
                elsif err_o = '1' then
                    wrong := true;
                    if bad < 10 then
                        report "vector " & integer'image(nvec)
                             & ": engine reported err code "
                             & integer'image(to_integer(errc_o))
                             & " at bit " & integer'image(to_integer(bitpos))
                             & " of " & integer'image(v_nbits)
                             & " (block " & integer'image(blk) & ")"
                            severity error;
                    end if;
                else
                    if blk /= 27 then
                        wrong := true;
                        if bad < 10 then
                            report "vector " & integer'image(nvec)
                                 & ": emitted " & integer'image(blk)
                                 & " blocks, expected 27" severity error;
                        end if;
                    end if;
                    -- The macroblock's edge counts, which the next macroblock
                    -- and the next row read.
                    for i in 0 to 3 loop
                        if to_integer(ncbot(5 * i + 4 downto 5 * i)) /= e_nc(12 + i)
                           or to_integer(ncright(5 * i + 4 downto 5 * i))
                              /= e_nc(i * 4 + 3) then
                            wrong := true;
                            if bad < 10 then
                                report "vector " & integer'image(nvec)
                                     & ": luma edge nC " & integer'image(i)
                                     & " wrong" severity error;
                            end if;
                            exit;
                        end if;
                    end loop;
                    for i in 0 to 1 loop
                        if to_integer(ncubot(5 * i + 4 downto 5 * i)) /= e_ncu(2 + i)
                           or to_integer(ncuright(5 * i + 4 downto 5 * i))
                              /= e_ncu(i * 2 + 1)
                           or to_integer(ncvbot(5 * i + 4 downto 5 * i)) /= e_ncv(2 + i)
                           or to_integer(ncvright(5 * i + 4 downto 5 * i))
                              /= e_ncv(i * 2 + 1) then
                            wrong := true;
                            if bad < 10 then
                                report "vector " & integer'image(nvec)
                                     & ": chroma edge nC " & integer'image(i)
                                     & " wrong" severity error;
                            end if;
                            exit;
                        end if;
                    end loop;
                    if to_integer(bitpos) /= v_nbits then
                        wrong := true;
                        if bad < 10 then
                            report "vector " & integer'image(nvec)
                                 & ": consumed " & integer'image(to_integer(bitpos))
                                 & " bits, residual is " & integer'image(v_nbits)
                                severity error;
                        end if;
                    end if;
                end if;

                if wrong then bad := bad + 1; end if;
                wait until rising_edge(clk);
                nvec := nvec + 1;
            end if;
        end loop;
        file_close(f);

        if IN_BP /= 0 and starved = 0 then
            report "back-pressure was requested but the reader never ran dry"
                severity error;
        end if;

        if bad = 0 then
            report "PASS - mb_residual_dec_engine: " & integer'image(nvec)
                 & " macroblocks match the C decoder, "
                 & integer'image(starved) & " cycles waiting on the reader"
                severity note;
        else
            report "FAIL - mb_residual_dec_engine: " & integer'image(bad)
                 & " of " & integer'image(nvec) & " macroblocks wrong"
                severity error;
        end if;
        finish;
    end process;

end architecture;
