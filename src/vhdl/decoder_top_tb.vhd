--------------------------------------------------------------------------------
-- decoder_top_tb.vhd — decode a whole real frame and compare every sample.
--
-- The payload is the encoder's own macroblock-layer byte stream and the
-- expectation is the C decoder's reconstruction of it, which `make dec_test`
-- already shows byte-exact against both the encoder's own reconstruction and
-- ffmpeg. So this is end to end on real data: real coefficient statistics,
-- real mode decisions, every macroblock position including all four edges,
-- and a golden checked against an implementation that shares no code with
-- this project.
--
-- The comparison is per 4x4 block and it stops at the first difference. In an
-- intra frame one wrong sample propagates into every block that predicts from
-- it, so a report of the last mismatch, or of a count, would describe the
-- damage rather than the cause. What the first one names is the block that
-- actually went wrong.
--
-- The reconstruction stream arrives in exactly the order the vector file
-- lists it -- macroblock raster order, then Y raster 0..15, U 0..3, V 0..3 --
-- so the expectations are read one line per block as they come rather than
-- held in memory.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity decoder_top_tb is
    generic (
        VEC_FILE : string := "build/decoder_frame_vectors.txt";
        -- Stall the payload feeder and the reconstruction consumer. A frame
        -- that decodes only when the bits are always there and the sink never
        -- pushes back has not been tested at its interfaces.
        BP       : integer := 1
    );
end entity;

architecture sim of decoder_top_tb is
    constant CLK_PERIOD : time := 5 ns;
    constant MAX_BYTES  : integer := 262144;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal frame_start : std_logic := '0';
    signal mbs_w, mbs_h : unsigned(7 downto 0) := (others => '0');
    signal qp     : unsigned(5 downto 0) := (others => '0');
    signal cqo    : signed(5 downto 0) := (others => '0');
    signal busy, frame_done, err : std_logic;
    signal errc   : unsigned(3 downto 0);
    signal errmb  : unsigned(15 downto 0);

    signal in_valid : std_logic := '0';
    signal in_ready : std_logic;
    signal in_data  : unsigned(7 downto 0) := (others => '0');
    signal in_last  : std_logic := '0';

    signal rec_valid : std_logic;
    signal rec_ready : std_logic := '0';
    signal rec_mb    : unsigned(15 downto 0);
    signal rec_plane : unsigned(1 downto 0);
    signal rec_idx   : unsigned(3 downto 0);
    signal rec_data  : std_logic_vector(127 downto 0);

    type byte_arr is array (0 to MAX_BYTES - 1) of integer;
    shared variable pay : byte_arr;
    shared variable pay_n : integer := 0;
    signal go : std_logic := '0';

    signal lfsr : unsigned(15 downto 0) := x"F00D";
    signal starved : integer := 0;
    -- Cycles the decoder was busy, so the cost of a timing change shows up
    -- here rather than being argued about.
    signal busy_cy : integer := 0;

    -- Where the cycles go. Probed through the hierarchy rather than added
    -- to the design's ports: this is a question about the design, not part
    -- of it.
    alias p_hstart is <<signal .decoder_top_tb.dut.h_start  : std_logic>>;
    alias p_hdone  is <<signal .decoder_top_tb.dut.h_done   : std_logic>>;
    alias p_rstart is <<signal .decoder_top_tb.dut.r_start  : std_logic>>;
    alias p_rdone  is <<signal .decoder_top_tb.dut.r_done   : std_logic>>;
    alias p_kdone  is <<signal .decoder_top_tb.dut.k_done   : std_logic>>;
    alias p_bvalid is <<signal .decoder_top_tb.dut.r_bvalid : std_logic>>;
    alias p_bready is <<signal .decoder_top_tb.dut.r_bready : std_logic>>;
    signal cy_hdr, cy_res, cy_rectail, cy_res_wait_rec, cy_rec_wait_res : integer := 0;
    -- Inside the sequencer: cycles its CAVLC engine is busy, cycles spent
    -- on symbols (consumes), and cycles the sequencer itself spends between
    -- blocks.
    alias p_estart is <<signal .decoder_top_tb.dut.res.e_start : std_logic>>;
    alias p_edone  is <<signal .decoder_top_tb.dut.res.e_done  : std_logic>>;
    alias p_cons   is <<signal .decoder_top_tb.dut.r_cons      : std_logic>>;
    signal cy_eng, n_cons, n_blk_coded, in_eng : integer := 0;
    signal in_hdr, in_res, in_tail : boolean := false;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.decoder_top
        generic map (MAX_MB_COLS => 120)
        port map (clk => clk, rst_n => rst_n,
                  frame_start_i => frame_start,
                  mbs_w_i => mbs_w, mbs_h_i => mbs_h,
                  qp_i => qp, chroma_qp_offset_i => cqo,
                  busy_o => busy, frame_done_o => frame_done,
                  err_o => err, err_code_o => errc, err_mb_o => errmb,
                  in_valid => in_valid, in_ready => in_ready,
                  in_data => in_data, in_last => in_last,
                  rec_valid_o => rec_valid, rec_ready_i => rec_ready,
                  rec_mb_o => rec_mb, rec_plane_o => rec_plane,
                  rec_idx_o => rec_idx, rec_data_o => rec_data);

    ----------------------------------------------------------------------
    -- Payload feeder. in_last on the final byte matters: the slice ends with
    -- the stop bit and a few padding bits, so without it the reader would
    -- wait forever for a full window that the stream never supplies.
    ----------------------------------------------------------------------
    feed_p : process(clk)
        variable i : integer := 0;
        variable armed : boolean := false;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                i := 0; armed := false; in_valid <= '0'; in_last <= '0';
            else
                lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13)
                                             xor lfsr(12) xor lfsr(10));
                if go = '1' then armed := true; end if;
                if armed and in_valid = '1' and in_ready = '1' then
                    i := i + 1;
                end if;
                if armed and i < pay_n
                   and (BP = 0 or lfsr(2 downto 0) /= "000") then
                    in_valid <= '1';
                    in_data  <= to_unsigned(pay(i), 8);
                    if i = pay_n - 1 then in_last <= '1'; else in_last <= '0'; end if;
                else
                    in_valid <= '0';
                end if;
            end if;
        end if;
    end process;

    prof_p : process(clk)
    begin
        if rising_edge(clk) then
            if p_hstart = '1' then in_hdr <= true; end if;
            if p_hdone  = '1' then in_hdr <= false; end if;
            if p_rstart = '1' then in_res <= true; end if;
            if p_rdone  = '1' then in_res <= false; in_tail <= true; end if;
            if p_kdone  = '1' then in_tail <= false; end if;
            if in_hdr  then cy_hdr     <= cy_hdr + 1; end if;
            if in_res  then cy_res     <= cy_res + 1; end if;
            if in_tail then cy_rectail <= cy_rectail + 1; end if;
            -- A block offered and not taken: the sequencer waits on the
            -- reconstruction. Taken-ready and nothing offered: the other way.
            if p_bvalid = '1' and p_bready = '0' then
                cy_res_wait_rec <= cy_res_wait_rec + 1;
            end if;
            if in_res and p_bvalid = '0' and p_bready = '1' then
                cy_rec_wait_res <= cy_rec_wait_res + 1;
            end if;
            if p_estart = '1' then in_eng <= 1; n_blk_coded <= n_blk_coded + 1; end if;
            if p_edone  = '1' then in_eng <= 0; end if;
            if in_eng = 1 or p_estart = '1' then cy_eng <= cy_eng + 1; end if;
            if p_cons = '1' then n_cons <= n_cons + 1; end if;
        end if;
    end process;

    starve_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '1' and busy = '1' then
                busy_cy <= busy_cy + 1;
                if in_valid = '0' then starved <= starved + 1; end if;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    main_p : process
        file f : text;
        variable L : line;
        variable open_status : file_open_status;
        variable v_mbw, v_mbh, v_qp, v_cqo, v_n : integer;
        variable i, j, nblk, guard : integer;
        variable expv : integer;
        variable got  : integer;
        variable bad  : integer := 0;
        variable exp_mb, exp_plane, exp_idx : integer;
        variable expblk : integer_vector(0 to 15);
        variable total_blocks : integer;
    begin
        file_open(open_status, f, VEC_FILE, read_mode);
        assert open_status = open_ok
            report "cannot open " & VEC_FILE severity failure;

        readline(f, L);
        read(L, v_mbw); read(L, v_mbh); read(L, v_qp); read(L, v_cqo); read(L, v_n);
        assert v_n <= MAX_BYTES
            report "payload larger than the testbench buffer" severity failure;
        for i in 0 to v_n - 1 loop
            readline(f, L);
            read(L, pay(i));
        end loop;
        pay_n := v_n;
        total_blocks := v_mbw * v_mbh * 24;

        report "decoding " & integer'image(v_mbw) & "x" & integer'image(v_mbh)
             & " macroblocks from " & integer'image(v_n) & " payload bytes"
            severity note;

        rst_n <= '0';
        wait until rising_edge(clk);
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);

        mbs_w <= to_unsigned(v_mbw, 8);
        mbs_h <= to_unsigned(v_mbh, 8);
        qp    <= to_unsigned(v_qp, 6);
        cqo   <= to_signed(v_cqo, 6);
        wait until rising_edge(clk);
        go <= '1';
        frame_start <= '1';
        wait until rising_edge(clk);
        frame_start <= '0';

        nblk  := 0;
        guard := 0;
        while nblk < total_blocks and bad = 0 and guard < 200000000 loop
            rec_ready <= '1';
            wait until rising_edge(clk);
            guard := guard + 1;
            if rec_valid = '1' then
                exp_mb    := nblk / 24;
                exp_plane := 0;
                exp_idx   := nblk mod 24;
                if exp_idx >= 20 then
                    exp_plane := 2; exp_idx := exp_idx - 20;
                elsif exp_idx >= 16 then
                    exp_plane := 1; exp_idx := exp_idx - 16;
                end if;
                if to_integer(rec_mb) /= exp_mb
                   or to_integer(rec_plane) /= exp_plane
                   or to_integer(rec_idx) /= exp_idx then
                    report "block " & integer'image(nblk)
                         & ": expected MB " & integer'image(exp_mb)
                         & " plane " & integer'image(exp_plane)
                         & " idx " & integer'image(exp_idx)
                         & ", got MB " & integer'image(to_integer(rec_mb))
                         & " plane " & integer'image(to_integer(rec_plane))
                         & " idx " & integer'image(to_integer(rec_idx))
                        severity error;
                    bad := bad + 1;
                else
                    readline(f, L);
                    for j in 0 to 15 loop
                        read(L, expblk(j));
                    end loop;
                    for j in 0 to 15 loop
                        if to_integer(unsigned(rec_data(j * 8 + 7 downto j * 8)))
                           /= expblk(j) then
                            bad := bad + 1;
                        end if;
                    end loop;
                    if bad > 0 then
                        -- The whole block, not just the first sample: a flat
                        -- offset points at the prediction or the DC, a
                        -- scattered one at the residual, and which of those it
                        -- is decides where to look next.
                        report "MB " & integer'image(exp_mb)
                             & " (row " & integer'image(exp_mb / v_mbw)
                             & " col " & integer'image(exp_mb mod v_mbw)
                             & ") plane " & integer'image(exp_plane)
                             & " block " & integer'image(exp_idx)
                            severity error;
                        for j in 0 to 15 loop
                            got := to_integer(unsigned(
                                       rec_data(j * 8 + 7 downto j * 8)));
                            report "   sample " & integer'image(j)
                                 & " expected " & integer'image(expblk(j))
                                 & " got " & integer'image(got)
                                 & "  (diff " & integer'image(got - expblk(j)) & ")"
                                severity note;
                        end loop;
                    end if;
                end if;
                nblk := nblk + 1;
            end if;
            if err = '1' then
                report "decoder reported err code "
                     & integer'image(to_integer(errc))
                     & " at MB " & integer'image(to_integer(errmb))
                    severity error;
                bad := bad + 1;
            end if;
            -- Push back now and then, so the reconstruction path has to hold
            -- a block rather than assume the sink is always listening.
            if BP /= 0 and (guard mod 37) = 0 then
                rec_ready <= '0';
                wait until rising_edge(clk);
                wait until rising_edge(clk);
            end if;
        end loop;
        rec_ready <= '0';

        if bad = 0 then
            if nblk /= total_blocks then
                report "FAIL - decoder_top: " & integer'image(nblk) & " of "
                     & integer'image(total_blocks) & " blocks emitted"
                    severity error;
            else
                guard := 0;
                while frame_done = '0' and guard < 100000 loop
                    wait until rising_edge(clk);
                    guard := guard + 1;
                end loop;
                if frame_done = '0' then
                    report "FAIL - decoder_top: never reported frame_done"
                        severity error;
                else
                    report "PASS - decoder_top: " & integer'image(v_mbw * v_mbh)
                         & " macroblocks reconstructed sample-exact in "
                         & integer'image(busy_cy) & " cycles, "
                         & integer'image(busy_cy / (v_mbw * v_mbh))
                         & " per macroblock ("
                         & integer'image(starved)
                         & " of them with no payload byte offered)"
                        severity note;
                    report "  per macroblock: header " & integer'image(cy_hdr / (v_mbw * v_mbh))
                         & ", residual+recon " & integer'image(cy_res / (v_mbw * v_mbh))
                         & " (of which sequencer waiting on recon "
                         & integer'image(cy_res_wait_rec / (v_mbw * v_mbh))
                         & ", recon waiting on sequencer "
                         & integer'image(cy_rec_wait_res / (v_mbw * v_mbh))
                         & "), recon tail after last block "
                         & integer'image(cy_rectail / (v_mbw * v_mbh))
                         & ", other " & integer'image((busy_cy - cy_hdr - cy_res - cy_rectail) / (v_mbw * v_mbh))
                        severity note;
                    report "  CAVLC engine per macroblock: busy " & integer'image(cy_eng / (v_mbw * v_mbh))
                         & " cycles over " & integer'image(n_blk_coded / (v_mbw * v_mbh))
                         & " coded blocks, " & integer'image(n_cons / (v_mbw * v_mbh))
                         & " symbols consumed; sequencer overhead "
                         & integer'image((cy_res - cy_eng) / (v_mbw * v_mbh))
                        severity note;
                end if;
            end if;
        else
            report "FAIL - decoder_top: stopped at block "
                 & integer'image(nblk) & " of " & integer'image(total_blocks)
                severity error;
        end if;

        file_close(f);
        finish;
    end process;

end architecture;
