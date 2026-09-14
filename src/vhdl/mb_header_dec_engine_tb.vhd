--------------------------------------------------------------------------------
-- mb_header_dec_engine_tb.vhd — drive mb_header_dec_engine against the C
-- decoder's own header parser.
--
-- Each vector in build/mb_header_dec_vectors.txt is a real round trip: a
-- header written with the syntax-element sequence src/encoder.c emits and
-- read back with dec_mb_header, the routine the golden decoder itself uses.
--
--   <qp_in> <avail_top> <avail_left> <mode4_top x4> <mode4_left x4>
--   <nbits> <nbytes> <bytes...>
--   <is_i4x4> <mode16> <mode_chroma> <cbp_luma> <cbp_chroma> <has_residual>
--   <qp_out> <modes4 x16>
--
-- The engine runs against a real bit_reader rather than a stub, and the
-- consumed bit count is checked against the encoder's own length. That check
-- matters more than it looks: every result can be right while the engine has
-- retired one bit too few, and the damage then lands on the residual blocks
-- that follow, far from the cause.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity mb_header_dec_engine_tb is
    generic (
        VEC_FILE : string := "build/mb_header_dec_vectors.txt";
        -- Starve the bit reader pseudorandomly. With the feeder running flat
        -- out the engine never has to hold a decision across a gap, which is
        -- the shape of bug that cost this project four silicon-visible
        -- defects on the encode side. 0 disables the stalls.
        IN_BP    : integer := 1
    );
end entity;

architecture sim of mb_header_dec_engine_tb is
    constant CLK_PERIOD : time := 5 ns;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    -- bit reader
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

    -- engine
    signal start_i   : std_logic := '0';
    signal ready_o   : std_logic;
    signal qp_i      : unsigned(5 downto 0) := (others => '0');
    signal m4top_i   : std_logic_vector(15 downto 0) := (others => '0');
    signal m4left_i  : std_logic_vector(15 downto 0) := (others => '0');
    signal at_i      : std_logic := '0';
    signal al_i      : std_logic := '0';
    signal done_o    : std_logic;
    signal err_o     : std_logic;
    signal errc_o    : unsigned(3 downto 0);
    signal i4_o      : std_logic;
    signal m16_o     : unsigned(1 downto 0);
    signal modes4_o  : std_logic_vector(63 downto 0);
    signal mc_o      : unsigned(1 downto 0);
    signal cbpl_o    : unsigned(3 downto 0);
    signal cbpc_o    : unsigned(1 downto 0);
    signal hasres_o  : std_logic;
    signal qp_o      : unsigned(5 downto 0);
    signal hbits_o   : unsigned(7 downto 0);

    type byte_arr is array (0 to 63) of integer;
    shared variable vbytes : byte_arr;
    shared variable vn     : integer := 0;
    signal vec_id : integer := 0;

    signal lfsr    : unsigned(15 downto 0) := x"BEEF";
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

    dut : entity work.mb_header_dec_engine
        port map (clk => clk, rst_n => rst_n,
                  start_i => start_i, ready_o => ready_o,
                  qp_i => qp_i,
                  mode4_top_i => m4top_i, mode4_left_i => m4left_i,
                  avail_top_i => at_i, avail_left_i => al_i,
                  peek_i => peek, avail_i => avail,
                  consume_o => consume, consume_n_o => consume_n,
                  done_o => done_o, err_o => err_o, err_code_o => errc_o,
                  is_i4x4_o => i4_o, mode16_o => m16_o, modes4_o => modes4_o,
                  mode_chroma_o => mc_o, cbp_luma_o => cbpl_o,
                  cbp_chroma_o => cbpc_o, has_residual_o => hasres_o,
                  qp_o => qp_o, hdr_bits_o => hbits_o);

    ----------------------------------------------------------------------
    -- Clocked and re-armed by a CHANGE of vec_id. A sequential feeder
    -- deadlocks here: the engine stops consuming as soon as it has its
    -- header, so the reader stays full and the feeder is still blocked in the
    -- previous vector's push loop when the next vector starts.
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
                -- Pad past the vector's own bytes: the last element can sit
                -- inside the final byte and the reader needs a full window.
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
        variable v_qp, v_at, v_al, v_nbits, v_nbytes : integer;
        variable v_i4, v_m16, v_mc, v_cbpl, v_cbpc, v_hr, v_qpo : integer;
        variable m4t, m4l : integer_vector(0 to 3);
        variable exp_modes : integer_vector(0 to 15);
        variable i, nvec, bad : integer := 0;
        variable got : integer;
    begin
        file_open(open_status, f, VEC_FILE, read_mode);
        assert open_status = open_ok
            report "cannot open " & VEC_FILE severity failure;

        nvec := 0; bad := 0;

        while not endfile(f) loop
            readline(f, L);
            if L'length > 0 then
                read(L, v_qp); read(L, v_at); read(L, v_al);
                for i in 0 to 3 loop read(L, m4t(i)); end loop;
                for i in 0 to 3 loop read(L, m4l(i)); end loop;
                read(L, v_nbits); read(L, v_nbytes);
                for i in 0 to v_nbytes - 1 loop read(L, vbytes(i)); end loop;
                read(L, v_i4); read(L, v_m16); read(L, v_mc);
                read(L, v_cbpl); read(L, v_cbpc); read(L, v_hr); read(L, v_qpo);
                for i in 0 to 15 loop read(L, exp_modes(i)); end loop;
                vn := v_nbytes;

                rst_n <= '0';
                wait until rising_edge(clk);
                wait until rising_edge(clk);
                rst_n <= '1';
                wait until rising_edge(clk);

                vec_id <= vec_id + 1;
                wait until rising_edge(clk);

                qp_i <= to_unsigned(v_qp, 6);
                at_i <= '1' when v_at = 1 else '0';
                al_i <= '1' when v_al = 1 else '0';
                for i in 0 to 3 loop
                    m4top_i(4 * i + 3 downto 4 * i)  <=
                        std_logic_vector(to_unsigned(m4t(i), 4));
                    m4left_i(4 * i + 3 downto 4 * i) <=
                        std_logic_vector(to_unsigned(m4l(i), 4));
                end loop;
                start_i <= '1';
                wait until rising_edge(clk);
                start_i <= '0';

                i := 0;
                while done_o = '0' and i < 20000 loop
                    wait until rising_edge(clk);
                    i := i + 1;
                end loop;

                if done_o = '0' then
                    bad := bad + 1;
                    if bad <= 10 then
                        report "vector " & integer'image(nvec)
                             & ": engine never asserted done" severity error;
                    end if;
                elsif err_o = '1' then
                    bad := bad + 1;
                    if bad <= 10 then
                        report "vector " & integer'image(nvec)
                             & ": engine reported err code "
                             & integer'image(to_integer(errc_o))
                             & " (1=mb_type 2=ue prefix 3=chroma mode "
                             & "4=cbp codeNum 5=mb_qp_delta) at bit "
                             & integer'image(to_integer(bitpos))
                             & " of " & integer'image(v_nbits)
                            severity error;
                    end if;
                else
                    got := 0;
                    if (v_i4 = 1) /= (i4_o = '1') then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": is_i4x4 expected " & integer'image(v_i4)
                            severity error;
                    elsif to_integer(m16_o) /= v_m16 then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": mode16 expected " & integer'image(v_m16)
                             & " got " & integer'image(to_integer(m16_o))
                            severity error;
                    elsif to_integer(mc_o) /= v_mc then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": chroma mode expected " & integer'image(v_mc)
                             & " got " & integer'image(to_integer(mc_o))
                            severity error;
                    elsif to_integer(cbpl_o) /= v_cbpl then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": cbp_luma expected " & integer'image(v_cbpl)
                             & " got " & integer'image(to_integer(cbpl_o))
                            severity error;
                    elsif to_integer(cbpc_o) /= v_cbpc then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": cbp_chroma expected " & integer'image(v_cbpc)
                             & " got " & integer'image(to_integer(cbpc_o))
                            severity error;
                    elsif (v_hr = 1) /= (hasres_o = '1') then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": has_residual expected " & integer'image(v_hr)
                            severity error;
                    elsif to_integer(qp_o) /= v_qpo then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": QP expected " & integer'image(v_qpo)
                             & " got " & integer'image(to_integer(qp_o))
                            severity error;
                    elsif to_integer(bitpos) /= v_nbits then
                        -- Every field can be right while the engine has
                        -- retired the wrong number of bits; the residual
                        -- blocks that follow would then be read from the
                        -- wrong offset.
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": consumed " & integer'image(to_integer(bitpos))
                             & " bits, header is " & integer'image(v_nbits)
                            severity error;
                    elsif to_integer(hbits_o) /= v_nbits then
                        got := 1;
                        report "vector " & integer'image(nvec)
                             & ": hdr_bits reported "
                             & integer'image(to_integer(hbits_o))
                             & ", header is " & integer'image(v_nbits)
                            severity error;
                    else
                        for i in 0 to 15 loop
                            if to_integer(unsigned(
                                   modes4_o(4 * i + 3 downto 4 * i)))
                               /= exp_modes(i) then
                                got := 1;
                                report "vector " & integer'image(nvec)
                                     & " mode " & integer'image(i)
                                     & ": expected " & integer'image(exp_modes(i))
                                     & " got " & integer'image(to_integer(unsigned(
                                           modes4_o(4 * i + 3 downto 4 * i))))
                                    severity error;
                                exit;
                            end if;
                        end loop;
                    end if;
                    if got = 1 then bad := bad + 1; end if;
                end if;

                wait until rising_edge(clk);
                nvec := nvec + 1;
            end if;
        end loop;
        file_close(f);

        if IN_BP /= 0 and starved = 0 then
            report "back-pressure was requested but the reader never ran dry; "
                 & "the stall pattern is too gentle to prove anything"
                severity error;
        end if;

        if bad = 0 then
            report "PASS - mb_header_dec_engine: " & integer'image(nvec)
                 & " headers match the C decoder, "
                 & integer'image(starved) & " cycles waiting on the reader"
                severity note;
        else
            report "FAIL - mb_header_dec_engine: " & integer'image(bad)
                 & " of " & integer'image(nvec) & " vectors wrong"
                severity error;
        end if;
        finish;
    end process;

end architecture;
