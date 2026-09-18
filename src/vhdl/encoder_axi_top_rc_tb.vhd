--------------------------------------------------------------------------------
-- encoder_axi_top_rc_tb.vhd — the per-MB rate control through the AXI top,
-- driven the way the host will: for every frame of a short sequence, write
-- CONFIG (frame QP) and the RC registers with the values the C reference's
-- frame-level controller produced (params.txt), START, feed the pixels
-- (stream<k>.txt), compare the payload byte for byte with the reference's
-- (payload<k>.txt), then read BYTES / CYCLES / RC_WTOTAL back.
--
-- The vectors come from tools/gen_rc_vectors.sh, which also writes
-- build/rc_tb/config.txt: the vector directory, the frame count and the
-- MB dimensions (a file rather than generics, because generic overrides do
-- not survive the xelab .bat wrapper on this machine).
--
-- With RC_EN clear in params.txt this is the frame-QP kernel on the same
-- frames, so the two runs give the cycle cost of the per-MB path directly.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity encoder_axi_top_rc_tb is
    generic (
        CFG_FILE : string := "build/rc_tb/config.txt";
        BP_MODE  : natural := 1;    -- output backpressure: 0 tidy, 1 long stalls
        IN_BP    : natural := 1     -- input stalls:        0 tidy, 1 long stalls
    );
end entity;

architecture sim of encoder_axi_top_rc_tb is
    constant CLK_PERIOD : time := 5 ns;
    signal aclk : std_logic := '0';
    signal aresetn : std_logic := '0';
    signal awaddr, araddr : std_logic_vector(7 downto 0) := (others => '0');
    signal awvalid, awready, wvalid, wready, bvalid, bready, arvalid, arready, rvalid, rready : std_logic := '0';
    signal wdata, rdata : std_logic_vector(31 downto 0) := (others => '0');
    signal wstrb : std_logic_vector(3 downto 0) := (others => '0');
    signal bresp, rresp : std_logic_vector(1 downto 0);
    signal s_tdata : std_logic_vector(31 downto 0) := (others => '0');
    signal s_tvalid, s_tready : std_logic := '0';
    signal m_tdata : std_logic_vector(31 downto 0);
    signal m_tkeep : std_logic_vector(3 downto 0);
    signal m_tlast, m_tvalid, m_tready : std_logic := '0';
    signal irq : std_logic;
    signal cycle : natural := 0;
    signal feed_go : boolean := false;
    signal feed_done : boolean := false;
    signal feed_k : natural := 0;

    -- configuration read from CFG_FILE
    shared variable cfg_dir    : line;
    shared variable cfg_frames : natural := 0;
    shared variable cfg_mbs_w  : natural := 0;
    shared variable cfg_mbs_h  : natural := 0;
    signal cfg_ready : boolean := false;
    -- optional lines 5 and 6 of the config override the backpressure generics
    signal bp_mode_s : natural := BP_MODE;
    signal in_bp_s   : natural := IN_BP;

    procedure axi_write(signal clk : in std_logic; addr : in natural; data : in std_logic_vector(31 downto 0);
                        signal awaddr_s : out std_logic_vector(7 downto 0); signal awvalid_s : out std_logic;
                        signal wdata_s : out std_logic_vector(31 downto 0); signal wstrb_s : out std_logic_vector(3 downto 0);
                        signal wvalid_s : out std_logic; signal bready_s : out std_logic;
                        signal awready_s : in std_logic; signal bvalid_s : in std_logic) is
    begin
        awaddr_s <= std_logic_vector(to_unsigned(addr, 8)); awvalid_s <= '1';
        wdata_s <= data; wstrb_s <= "1111"; wvalid_s <= '1'; bready_s <= '1';
        loop wait until rising_edge(clk); exit when awready_s = '1'; end loop;
        awvalid_s <= '0'; wvalid_s <= '0';
        if bvalid_s /= '1' then
            loop wait until rising_edge(clk); exit when bvalid_s = '1'; end loop;
        end if;
        bready_s <= '0';
        wait until rising_edge(clk);
    end procedure;

    procedure axi_read(signal clk : in std_logic; addr : in natural; data : out std_logic_vector(31 downto 0);
                       signal araddr_s : out std_logic_vector(7 downto 0); signal arvalid_s : out std_logic;
                       signal rready_s : out std_logic; signal arready_s : in std_logic;
                       signal rvalid_s : in std_logic; signal rdata_s : in std_logic_vector(31 downto 0)) is
    begin
        araddr_s <= std_logic_vector(to_unsigned(addr, 8)); arvalid_s <= '1'; rready_s <= '1';
        loop wait until rising_edge(clk); exit when arready_s = '1'; end loop;
        arvalid_s <= '0';
        if rvalid_s /= '1' then
            loop wait until rising_edge(clk); exit when rvalid_s = '1'; end loop;
        end if;
        data := rdata_s;
        rready_s <= '0';
        wait until rising_edge(clk);
    end procedure;

    function ctz8(v : integer) return integer is
        variable u : unsigned(7 downto 0);
    begin
        u := to_unsigned(v, 8);
        for i in 0 to 7 loop
            if u(i) = '1' then return i; end if;
        end loop;
        return 8;
    end function;
begin
    aclk <= not aclk after CLK_PERIOD / 2;

    dut : entity work.encoder_axi_top
        generic map (MAX_W => 480, N_ENGINES => 2, MAX_MBS => 1024)
        port map (aclk => aclk, aresetn => aresetn,
                  s_axi_awaddr => awaddr, s_axi_awvalid => awvalid, s_axi_awready => awready,
                  s_axi_wdata => wdata, s_axi_wstrb => wstrb, s_axi_wvalid => wvalid, s_axi_wready => wready,
                  s_axi_bresp => bresp, s_axi_bvalid => bvalid, s_axi_bready => bready,
                  s_axi_araddr => araddr, s_axi_arvalid => arvalid, s_axi_arready => arready,
                  s_axi_rdata => rdata, s_axi_rresp => rresp, s_axi_rvalid => rvalid, s_axi_rready => rready,
                  s_axis_tdata => s_tdata, s_axis_tkeep => "1111", s_axis_tlast => '0',
                  s_axis_tvalid => s_tvalid, s_axis_tready => s_tready,
                  m_axis_tdata => m_tdata, m_axis_tkeep => m_tkeep, m_axis_tlast => m_tlast,
                  m_axis_tvalid => m_tvalid, m_axis_tready => m_tready, irq => irq);

    cyc_p : process(aclk)
        variable lfsr  : unsigned(15 downto 0) := x"ACE1";
        variable stall : integer := 0;
    begin
        if rising_edge(aclk) then
            cycle <= cycle + 1;
            if bp_mode_s = 0 then
                if (cycle mod 5) = 2 then m_tready <= '0'; else m_tready <= '1'; end if;
            else
                lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
                if stall > 0 then
                    stall := stall - 1;
                    m_tready <= '0';
                elsif lfsr(3 downto 0) = "0000" then
                    stall := 1 + to_integer(lfsr(6 downto 0));
                    m_tready <= '0';
                else
                    m_tready <= '1';
                end if;
            end if;
        end if;
    end process;

    src_p : process
        file f : text;
        variable L : line;
        variable w32 : std_logic_vector(31 downto 0);
        variable open_status : file_open_status;
        variable gap : natural := 0;
        variable ilfsr : unsigned(15 downto 0) := x"BEEF";
        variable istall : integer := 0;
    begin
        wait until cfg_ready;
        loop
            wait until feed_go;
            feed_done <= false;
            file_open(open_status, f, cfg_dir.all & "stream" & integer'image(feed_k) & ".txt", read_mode);
            assert open_status = open_ok report "could not open stream " & integer'image(feed_k) severity failure;
            while not endfile(f) loop
                readline(f, L);
                if L'length = 0 then next; end if;
                hread(L, w32);
                s_tdata <= w32; s_tvalid <= '1';
                loop wait until rising_edge(aclk); exit when s_tready = '1'; end loop;
                s_tvalid <= '0';
                if in_bp_s = 0 then
                    gap := (gap * 5 + 1) mod 4;
                    if gap = 3 then wait until rising_edge(aclk); end if;
                else
                    ilfsr := ilfsr(14 downto 0) & (ilfsr(15) xor ilfsr(13) xor ilfsr(12) xor ilfsr(10));
                    if ilfsr(3 downto 0) = "0000" then
                        istall := 1 + to_integer(ilfsr(6 downto 0));
                        for z in 1 to istall loop wait until rising_edge(aclk); end loop;
                    end if;
                end if;
            end loop;
            file_close(f);
            feed_done <= true;
            wait until not feed_go;
        end loop;
    end process;

    main_p : process
        file f, fp, fc : text;
        file fdump : text;
        variable Ld, L : line;
        variable val, got, nb : integer;
        variable open_status : file_open_status;
        variable n, mc, mc_tot, k : natural := 0;
        variable r : std_logic_vector(31 downto 0);
        variable last_seen : boolean;
        variable t0 : natural;
        variable p_qp, p_target, p_qpmin, p_qpmax, p_map, p_lag, p_en : integer;
        variable p_scale : std_logic_vector(31 downto 0);
        variable rc_ctrl : std_logic_vector(31 downto 0);
        variable last_nz_idx, last_nz_val : integer;
        variable wtot_exp : integer;
        variable cyc_tot : natural := 0;
    begin
        -- configuration
        file_open(open_status, fc, CFG_FILE, read_mode);
        assert open_status = open_ok report "could not open " & CFG_FILE severity failure;
        readline(fc, cfg_dir);
        readline(fc, L); read(L, cfg_frames);
        readline(fc, L); read(L, cfg_mbs_w);
        readline(fc, L); read(L, cfg_mbs_h);
        if not endfile(fc) then readline(fc, L); read(L, val); bp_mode_s <= val; end if;
        if not endfile(fc) then readline(fc, L); read(L, val); in_bp_s <= val; end if;
        file_close(fc);
        wait until rising_edge(aclk);
        cfg_ready <= true;
        report "config: " & cfg_dir.all & " frames " & integer'image(cfg_frames) & " MBs " &
               integer'image(cfg_mbs_w) & "x" & integer'image(cfg_mbs_h) &
               " bp_mode " & integer'image(bp_mode_s) & " in_bp " & integer'image(in_bp_s) severity note;

        aresetn <= '0';
        wait for 10 * CLK_PERIOD;
        wait until rising_edge(aclk);
        aresetn <= '1';
        wait for 10 * CLK_PERIOD;
        wait until rising_edge(aclk);
        axi_read(aclk, 16#1C#, r, araddr, arvalid, rready, arready, rvalid, rdata);
        assert r = x"48323634" report "ID register mismatch" severity failure;
        axi_read(aclk, 16#20#, r, araddr, arvalid, rready, arready, rvalid, rdata);
        assert r = x"00010003" report "VERSION is not 1.3" severity failure;
        axi_write(aclk, 16#00#, x"00000100", awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);   -- IRQ_EN

        file_open(open_status, fp, cfg_dir.all & "params.txt", read_mode);
        assert open_status = open_ok report "could not open params.txt" severity failure;

        for k in 0 to cfg_frames - 1 loop
            -- the host's per-frame values
            readline(fp, L);
            read(L, p_qp); read(L, p_target); hread(L, p_scale);
            read(L, p_qpmin); read(L, p_qpmax); read(L, p_map); read(L, p_lag); read(L, p_en);
            axi_write(aclk, 16#04#, x"00" & "00" & std_logic_vector(to_unsigned(p_qp, 6)) &
                      std_logic_vector(to_unsigned(cfg_mbs_h, 8)) & std_logic_vector(to_unsigned(cfg_mbs_w, 8)),
                      awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);
            rc_ctrl := (others => '0');
            rc_ctrl(0) := '1' when p_en = 1 else '0';
            rc_ctrl(1) := '1' when p_map = 1 else '0';
            rc_ctrl(13 downto 8)  := std_logic_vector(to_unsigned(p_qpmin, 6));
            rc_ctrl(21 downto 16) := std_logic_vector(to_unsigned(p_qpmax, 6));
            rc_ctrl(27 downto 24) := std_logic_vector(to_unsigned(p_lag, 4));
            axi_write(aclk, 16#24#, rc_ctrl, awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);
            axi_write(aclk, 16#28#, std_logic_vector(to_unsigned(p_target, 32)), awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);
            axi_write(aclk, 16#2C#, p_scale, awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);
            axi_read(aclk, 16#24#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            assert r = rc_ctrl report "RC_CTRL readback mismatch" severity failure;

            file_open(open_status, f, cfg_dir.all & "payload" & integer'image(k) & ".txt", read_mode);
            assert open_status = open_ok report "could not open payload " & integer'image(k) severity failure;
            file_open(open_status, fdump, cfg_dir.all & "got" & integer'image(k) & ".txt", write_mode);
            n := 0; mc := 0; last_seen := false; last_nz_idx := 0; last_nz_val := 0;
            t0 := cycle;
            feed_k <= k;
            feed_go <= true;
            axi_write(aclk, 16#00#, x"00000101", awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);   -- START + IRQ_EN
            loop
                wait until rising_edge(aclk);
                if m_tvalid = '1' and m_tready = '1' then
                    if m_tkeep = "1111" then nb := 4; elsif m_tkeep = "0111" then nb := 3; elsif m_tkeep = "0011" then nb := 2; else nb := 1; end if;
                    for i in 0 to nb - 1 loop
                        got := to_integer(unsigned(m_tdata(8 * i + 7 downto 8 * i)));
                        write(Ld, got); writeline(fdump, Ld);
                        if got /= 0 then last_nz_idx := n; last_nz_val := got; end if;
                        if endfile(f) then
                            mc := mc + 1;
                            if mc <= 10 then report "MISMATCH: extra byte " & integer'image(got) severity error; end if;
                        else
                            readline(f, L); read(L, val);
                            if got /= val then
                                mc := mc + 1;
                                if mc <= 10 then report "MISMATCH frame " & integer'image(k) & " byte " & integer'image(n) & ": expected " & integer'image(val) & " got " & integer'image(got) severity error; end if;
                            end if;
                            if endfile(f) and (m_tlast = '0' or i /= nb - 1) then
                                mc := mc + 1; report "MISMATCH: tlast not on the final byte" severity error;
                            end if;
                        end if;
                        n := n + 1;
                    end loop;
                    if m_tlast = '1' then last_seen := true; end if;
                end if;
                exit when last_seen and irq = '1';
            end loop;
            if not endfile(f) then mc := mc + 1; report "MISMATCH: stream ended early" severity error; end if;
            file_close(f);
            file_close(fdump);
            -- the stop bit is the last set bit of the payload; everything before it is MB bits
            wtot_exp := 8 * last_nz_idx + 7 - ctz8(last_nz_val);
            axi_read(aclk, 16#08#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            assert r(1) = '1' report "DONE not set" severity error;
            axi_read(aclk, 16#18#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            assert to_integer(unsigned(r)) = n report "BYTES counter wrong: " & integer'image(to_integer(unsigned(r))) severity error;
            axi_read(aclk, 16#30#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            if to_integer(unsigned(r)) /= wtot_exp then
                mc := mc + 1;
                report "MISMATCH: RC_WTOTAL " & integer'image(to_integer(unsigned(r))) & " expected " & integer'image(wtot_exp) severity error;
            end if;
            axi_read(aclk, 16#14#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            cyc_tot := cyc_tot + to_integer(unsigned(r));
            report "RCSTAT frame " & integer'image(k) & ": rc_en " & integer'image(p_en) & " qp_frame " & integer'image(p_qp) &
                   " bytes " & integer'image(n) & " mb_bits " & integer'image(wtot_exp) &
                   " cycles " & integer'image(to_integer(unsigned(r))) &
                   " cycles/MB " & integer'image(to_integer(unsigned(r)) / (cfg_mbs_w * cfg_mbs_h)) &
                   " mismatches " & integer'image(mc) severity note;
            axi_write(aclk, 16#0C#, x"00000001", awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);   -- DONE_CLR
            mc_tot := mc_tot + mc;
            while not feed_done loop wait until rising_edge(aclk); end loop;
            feed_go <= false;
            wait for 20 * CLK_PERIOD;
        end loop;
        file_close(fp);
        report "RCSTAT total cycles " & integer'image(cyc_tot) & " over " & integer'image(cfg_frames) & " frames, " &
               integer'image(cyc_tot / (cfg_frames * cfg_mbs_w * cfg_mbs_h)) & " cycles/MB" severity note;
        if mc_tot = 0 then
            report "PASS - encoder_axi_top_rc_tb: " & integer'image(cfg_frames) & " frames bit-exact through AXI" severity note;
        else
            report "FAIL - " & integer'image(mc_tot) & " mismatches" severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process begin
        wait for 20 ms;
        report "watchdog timeout" severity failure;
        std.env.finish;
        wait;
    end process;
end architecture;
