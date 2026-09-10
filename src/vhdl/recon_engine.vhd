--------------------------------------------------------------------------------
-- recon_engine.vhd
--
-- 4x4 reconstruction: recon = clip8(pred + ((res + 32) >> 6)), matching
-- recon_4x4 / recon_4x4_local in src/encoder.c. One block per cycle,
-- latency 1 for the reconstructed samples.
--
-- Optional distortion (WITH_SSD): ssd = sum((recon - src)^2) over the 16
-- samples, for the rate-distortion mode decision. The 16 squares use
-- DSP48 slices; the sum is an adder tree. With WITH_SSD the reconstructed
-- samples are delayed to line up with ssd_o, so both come out together
-- under one valid/ready at latency 3. Without it the latency is 1.
--
-- Sample buses: sample k at bits (8k+7 downto 8k). Residual bus: sample k
-- at bits (RES_W*(k+1)-1 downto RES_W*k), two's complement. RES_W = 20
-- matches the transform engine's internal width; feed it the low 20 bits
-- of its 32-bit outputs.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity recon_engine is
    generic (
        RES_W    : positive := 20;
        WITH_SSD : boolean  := true
    );
    port (
        clk         : in  std_logic;
        rst_n       : in  std_logic;
        pred_i      : in  std_logic_vector(127 downto 0);
        res_i       : in  std_logic_vector(16 * RES_W - 1 downto 0);
        src_i       : in  std_logic_vector(127 downto 0);   -- only used with WITH_SSD
        valid_i     : in  std_logic;
        ready_o     : out std_logic;
        recon_o     : out std_logic_vector(127 downto 0);
        ssd_o       : out unsigned(19 downto 0);   -- valid with valid_o (WITH_SSD only)
        valid_o     : out std_logic;
        ready_i     : in  std_logic
    );
end entity;

architecture rtl of recon_engine is

    subtype px_t is unsigned(7 downto 0);
    type px16_t is array (0 to 15) of px_t;
    subtype d_t is signed(8 downto 0);
    type d16_t is array (0 to 15) of d_t;
    subtype sq_t is unsigned(15 downto 0);
    type sq16_t is array (0 to 15) of sq_t;

    signal rec1, rec2, rec3 : px16_t := (others => (others => '0'));
    signal diff1  : d16_t  := (others => (others => '0'));
    signal v1     : std_logic := '0';
    signal sq2    : sq16_t := (others => (others => '0'));
    signal v2     : std_logic := '0';
    signal ssd3   : unsigned(19 downto 0) := (others => '0');
    signal v3     : std_logic := '0';
    signal vlast  : std_logic;

    attribute use_dsp : string;
    attribute use_dsp of sq2 : signal is "yes";
    -- Keep the 2-stage recon delay in flip-flops (plentiful) rather than
    -- LUT shift registers, which count against the LUT budget.
    attribute shreg_extract : string;
    attribute shreg_extract of rec2 : signal is "no";
    attribute shreg_extract of rec3 : signal is "no";

    signal advance : std_logic;

begin

    gen_vl_ssd : if WITH_SSD generate
        vlast <= v3;
    end generate;
    gen_vl_nossd : if not WITH_SSD generate
        vlast <= v1;
    end generate;

    advance <= ready_i or not vlast;
    ready_o <= advance;

    process(clk, rst_n)
        variable p    : unsigned(7 downto 0);
        variable r    : signed(RES_W - 1 downto 0);
        variable rr   : signed(RES_W - 7 downto 0);
        variable big  : boolean;
        variable v    : signed(10 downto 0);
        variable px   : px_t;
        variable prod : signed(17 downto 0);
        variable s    : unsigned(19 downto 0);
    begin
        if rst_n = '0' then
            v1 <= '0'; v2 <= '0'; v3 <= '0';
        elsif rising_edge(clk) then
            if advance = '1' then
                ----------------------------------------------------------
                -- s1: round, add, clip
                ----------------------------------------------------------
                for k in 0 to 15 loop
                    p  := unsigned(pred_i(8 * k + 7 downto 8 * k));
                    r  := signed(res_i(RES_W * (k + 1) - 1 downto RES_W * k));
                    -- (r + 32) >> 6 == (r >> 6) + r(5): fold the rounding into
                    -- the carry-in of the pred add instead of a second adder.
                    rr := r(RES_W - 1 downto 6);
                    -- |rr| >= 512 clips regardless of pred; keep 10 bits otherwise
                    big := (rr(RES_W - 7 downto 9) /= (RES_W - 7 downto 9 => rr(RES_W - 7)));
                    if big then
                        if rr(RES_W - 7) = '1' then px := x"00"; else px := x"FF"; end if;
                    else
                        v := resize(signed('0' & p), 11) + resize(rr(9 downto 0), 11)
                             + resize(signed('0' & r(5 downto 5)), 11);
                        if v < 0 then px := x"00";
                        elsif v > 255 then px := x"FF";
                        else px := unsigned(v(7 downto 0));
                        end if;
                    end if;
                    rec1(k)  <= px;
                    diff1(k) <= resize(signed('0' & px), 9) - resize(signed('0' & unsigned(src_i(8 * k + 7 downto 8 * k))), 9);
                end loop;
                v1 <= valid_i;

                ----------------------------------------------------------
                -- s2 / s3: SSD
                ----------------------------------------------------------
                if WITH_SSD then
                    for k in 0 to 15 loop
                        -- |diff| <= 255 so the square fits 16 bits; take the low
                        -- slice of the 18-bit product (resize would keep the
                        -- sign bit and drop bit 15/16).
                        prod := diff1(k) * diff1(k);
                        sq2(k) <= unsigned(prod(15 downto 0));
                    end loop;
                    rec2 <= rec1;
                    v2 <= v1;
                    s := (others => '0');
                    for k in 0 to 15 loop
                        s := s + resize(sq2(k), 20);
                    end loop;
                    ssd3 <= s;
                    rec3 <= rec2;
                    v3 <= v2;
                end if;
            end if;
        end if;
    end process;

    gen_ssd : if WITH_SSD generate
        gen_out : for k in 0 to 15 generate
            recon_o(8 * k + 7 downto 8 * k) <= std_logic_vector(rec3(k));
        end generate;
        ssd_o   <= ssd3;
        valid_o <= v3;
    end generate;
    gen_nossd : if not WITH_SSD generate
        gen_out : for k in 0 to 15 generate
            recon_o(8 * k + 7 downto 8 * k) <= std_logic_vector(rec1(k));
        end generate;
        ssd_o   <= (others => '0');
        valid_o <= v1;
    end generate;

end architecture;
