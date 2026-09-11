--------------------------------------------------------------------------------
-- encoder_axi_top_tb.vhd — drives encoder_axi_top the way the host will:
-- AXI4-Lite register writes (CONFIG, CTRL.START), pixels over AXI4-Stream
-- (build/frame_stream.txt), payload back over AXI4-Stream compared byte for
-- byte with build/slice_payload.txt, then STATUS / FRAMES / CYCLES / BYTES
-- read back. Two frames are run back to back to exercise the restart.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.all;

entity encoder_axi_top_tb is
    generic (
        IN_FILE  : string := "build/frame_stream.txt";
        OUT_FILE : string := "build/slice_payload.txt";
        MBS_W    : natural := 30;
        MBS_H    : natural := 17;
        QP       : natural := 26;
        FRAMES   : natural := 2
    );
end entity;

architecture sim of encoder_axi_top_tb is
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
        -- rvalid may come with arready (same cycle) or later
        if rvalid_s /= '1' then
            loop wait until rising_edge(clk); exit when rvalid_s = '1'; end loop;
        end if;
        data := rdata_s;
        rready_s <= '0';
        wait until rising_edge(clk);
    end procedure;
begin
    aclk <= not aclk after CLK_PERIOD / 2;

    dut : entity work.encoder_axi_top
        generic map (MAX_W => 480, N_ENGINES => 2)
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
    begin
        if rising_edge(aclk) then
            cycle <= cycle + 1;
            if (cycle mod 5) = 2 then m_tready <= '0'; else m_tready <= '1'; end if;
        end if;
    end process;

    -- pixel feeder (DMA model) with random gaps
    src_p : process
        file f : text;
        variable L : line;
        variable w32 : std_logic_vector(31 downto 0);
        variable open_status : file_open_status;
        variable gap : natural := 0;
    begin
        for k in 0 to FRAMES - 1 loop
            wait until feed_go;
            feed_done <= false;
            file_open(open_status, f, IN_FILE, read_mode);
            assert open_status = open_ok report "could not open " & IN_FILE severity failure;
            while not endfile(f) loop
                readline(f, L);
                if L'length = 0 then next; end if;
                hread(L, w32);
                s_tdata <= w32; s_tvalid <= '1';
                loop wait until rising_edge(aclk); exit when s_tready = '1'; end loop;
                s_tvalid <= '0';
                gap := (gap * 5 + 1) mod 4;
                if gap = 3 then wait until rising_edge(aclk); end if;
            end loop;
            file_close(f);
            feed_done <= true;
            wait until not feed_go;
        end loop;
        wait;
    end process;

    main_p : process
        file f : text;
        variable L : line;
        variable val, got, nb : integer;
        variable open_status : file_open_status;
        variable n, mc, mc_tot : natural := 0;
        variable r : std_logic_vector(31 downto 0);
        variable last_seen : boolean;
        variable t0 : natural;
    begin
        aresetn <= '0';
        wait for 10 * CLK_PERIOD;
        wait until rising_edge(aclk);
        aresetn <= '1';
        wait for 10 * CLK_PERIOD;
        wait until rising_edge(aclk);
        report "AXI: reading ID" severity note;
        axi_read(aclk, 16#1C#, r, araddr, arvalid, rready, arready, rvalid, rdata);
        assert r = x"48323634" report "ID register mismatch" severity failure;
        report "AXI: ID ok, writing CONFIG" severity note;
        axi_write(aclk, 16#04#, x"00" & "00" & std_logic_vector(to_unsigned(QP, 6)) &
                  std_logic_vector(to_unsigned(MBS_H, 8)) & std_logic_vector(to_unsigned(MBS_W, 8)),
                  awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);
        axi_read(aclk, 16#04#, r, araddr, arvalid, rready, arready, rvalid, rdata);
        assert to_integer(unsigned(r(7 downto 0))) = MBS_W and to_integer(unsigned(r(21 downto 16))) = QP
            report "CONFIG readback mismatch" severity failure;
        axi_write(aclk, 16#00#, x"00000100", awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);   -- IRQ_EN

        for k in 0 to FRAMES - 1 loop
            file_open(open_status, f, OUT_FILE, read_mode);
            assert open_status = open_ok report "could not open " & OUT_FILE severity failure;
            n := 0; mc := 0; last_seen := false;
            t0 := cycle;
            feed_go <= true;
            report "AXI: START frame " & integer'image(k) severity note;
            axi_write(aclk, 16#00#, x"00000101", awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);   -- START + IRQ_EN
            report "AXI: START accepted, waiting for payload" severity note;
            loop
                wait until rising_edge(aclk);
                if m_tvalid = '1' and m_tready = '1' then
                    if m_tkeep = "1111" then nb := 4; elsif m_tkeep = "0111" then nb := 3; elsif m_tkeep = "0011" then nb := 2; else nb := 1; end if;
                    for i in 0 to nb - 1 loop
                        got := to_integer(unsigned(m_tdata(8 * i + 7 downto 8 * i)));
                        if endfile(f) then
                            mc := mc + 1;
                            if mc <= 10 then report "MISMATCH: extra byte " & integer'image(got) severity error; end if;
                        else
                            readline(f, L); read(L, val);
                            if got /= val then
                                mc := mc + 1;
                                if mc <= 10 then report "MISMATCH byte " & integer'image(n) & ": expected " & integer'image(val) & " got " & integer'image(got) severity error; end if;
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
            -- status readback
            axi_read(aclk, 16#08#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            assert r(1) = '1' report "DONE not set" severity error;
            assert r(0) = '0' report "BUSY still set after done" severity error;
            axi_read(aclk, 16#10#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            assert to_integer(unsigned(r)) = k + 1 report "FRAMES counter wrong" severity error;
            axi_read(aclk, 16#18#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            assert to_integer(unsigned(r)) = n report "BYTES counter wrong: " & integer'image(to_integer(unsigned(r))) severity error;
            axi_read(aclk, 16#14#, r, araddr, arvalid, rready, arready, rvalid, rdata);
            report "frame " & integer'image(k) & ": " & integer'image(n) & " bytes, CYCLES reg " & integer'image(to_integer(unsigned(r))) &
                   " (tb " & integer'image(cycle - t0) & "), " & integer'image(to_integer(unsigned(r)) / (MBS_W * MBS_H)) & " cycles/MB, " &
                   integer'image(mc) & " mismatches" severity note;
            axi_write(aclk, 16#0C#, x"00000001", awaddr, awvalid, wdata, wstrb, wvalid, bready, awready, bvalid);   -- DONE_CLR
            assert irq = '0' report "irq still set after DONE_CLR" severity error;
            mc_tot := mc_tot + mc;
            while not feed_done loop wait until rising_edge(aclk); end loop;
            feed_go <= false;
            wait for 20 * CLK_PERIOD;
        end loop;
        if mc_tot = 0 then
            report "PASS - encoder_axi_top: " & integer'image(FRAMES) & " frames bit-exact through AXI" severity note;
        else
            report "FAIL - " & integer'image(mc_tot) & " mismatches" severity failure;
        end if;
        std.env.finish;
        wait;
    end process;

    watchdog_p : process begin
        wait for 8 ms;
        report "watchdog timeout" severity failure;
        std.env.finish;
        wait;
    end process;
end architecture;
