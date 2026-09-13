# Bring-up runbook — Zybo Z7-20

The practical sequence, with the values this project's scripts actually
produced. The reasoning behind it all is in `docs/m5-hw-validation.md`; this
file is the checklist.

Everything here up to step 4 has been run on this machine. Steps 5 and 6 need
the board.

**Known values.** Referred to throughout, all confirmed from the build:

| Thing | Value |
|---|---|
| IP | `dcc:codec:dcc_h264_enc:1.0` |
| IP repository | `build/ip` |
| Board part | `digilentinc.com:zybo-z7-20:part0:1.2` |
| Board files | `C:/Users/kamin/vivado-boards/new/board_files` |
| Part | `xc7z020clg400-1` |
| PL clock | **100 MHz** (see the clock note below) |
| Encoder registers | `0x43C0_0000` |
| DMA registers | `0x4040_0000` |
| CPU / domain | `ps7_cortexa9_0` / `standalone_ps7_cortexa9_0` |

---

## 0. Prerequisites

Digilent's board files are **already installed** at
`C:/Users/kamin/vivado-boards/new/board_files` (cloned from
`github.com/Digilent/vivado-boards`). Every Vivado invocation below needs
`BOARD_FILES` pointing at it, as a **Windows-style path** — Vivado does not
understand an MSYS `/c/...` path and the failure message is a confusing
"board part not found".

```sh
export BOARD_FILES="C:/Users/kamin/vivado-boards/new/board_files"
```

To make it permanent instead, copy the `zybo-z7-20` directory into
`<Vivado>/data/boards/board_files/` and drop the env var.

---

## 1. Package the IP

```sh
make ip         # -> build/ip/dcc_h264_enc/component.xml
make ip_check   # prove it is usable before you rely on it
```

`make ip_check` instantiates the core in a throwaway block design, asserts
that `s_axi`, `s_axis` and `m_axis` all came through as bus interfaces rather
than loose pins, sets both generics from IP Integrator, and synthesizes. It
takes a couple of minutes and it is the difference between finding a
packaging problem now and finding it as an inexplicable elaboration error in
the middle of a board design.

Expect one critical warning about packaging a VHDL-2008 top file. For this
core it is benign — the entity boundary is plain `std_logic` and `positive`
generics, and `ip_check` passing is the evidence.

---

## 2. Add the IP to a project

### Scripted (what `make zybo` does)

```tcl
set_property ip_repo_paths build/ip [current_project]
update_ip_catalog -rebuild
create_bd_cell -type ip -vlnv dcc:codec:dcc_h264_enc:1.0 enc
set_property -dict [list CONFIG.MAX_W {1920} CONFIG.N_ENGINES {2}] [get_bd_cells enc]
```

### By hand in the GUI

1. **Settings → IP → Repository → `+`**, add the `build/ip` directory. The
   catalog should report one IP found.
2. In the block design, **Add IP** and search for *DCC H.264 Intra Encoder*.
3. Double-click it to set `MAX_W` (frame-width ceiling, costs BRAM) and
   `N_ENGINES` (CAVLC engines; 2 is enough for 1080p30).

Either way the IP appears under *Video and Image Processing*.

**If you re-package the IP, bump the version or refresh.** Vivado caches
IP by VLNV, so re-running `make ip` without changing `1.0` leaves an existing
project pointing at the stale copy. In the GUI that is **Reports → IP Status →
Upgrade Selected**; from Tcl, `update_ip_catalog -rebuild` then
`upgrade_ip [get_ips]`.

---

## 3. Build the block design and bitstream

```sh
BOARD_FILES="C:/Users/kamin/vivado-boards/new/board_files" make zybo
```

That is a full synth, implementation and bitstream: budget 30–60 minutes.

### The clock: you cannot have 110 MHz

FCLK0 is the IO PLL divided by two integers. From the 1000 MHz PLL the only
values reachable near 110 MHz are **1000/9 = 111.111** and **1000/10 = 100**.
Ask for 110 and the PS quietly gives you 111.111, and
`PCW_FPGA0_PERIPHERAL_FREQMHZ` keeps reporting 110, so the request looks
honoured. The achieved value is `PCW_CLK0_FREQ`, and that is what the timing
constraint is actually built from.

This is not cosmetic. The kernel's out-of-context margin at 9.091 ns (110 MHz)
was **+0.048 ns**. At 111.111 MHz the constraint tightens to 9.0 ns, which
puts the kernel roughly 40 ps *negative* before the DMA and the interconnect
have taken their share.

So the build targets **100 MHz**, a real PS frequency with about a nanosecond
of margin. The cost is small: at 337 cycles per macroblock a 1080p frame is
27.5 ms, so 36 fps, and 1080p30 is still met with 21% to spare.

| FCLK | Period | Kernel OOC slack | 1080p fps |
|---|---|---|---|
| 111.111 MHz | 9.000 ns | about -0.04 ns | 40.4 |
| 100 MHz | 10.000 ns | roughly +1 ns | 36.4 |

Chase 111.111 MHz later if you want the extra 4 fps. Not during bring-up.

To check a change to the script without the wait, validate only:

```sh
BOARD_FILES="..." vivado -mode batch -source scripts/build_zybo_bd.tcl \
    -tclargs build/zybo_val build/ip 100 0
```

The trailing `0` stops after `validate_bd_design`. That takes about two
minutes and catches every automation and connection error.

**What the script prints, and why you should read it:**

- `BD_BOARD` — the resolved board part, version included.
- `BD_FCLK requested=... actual=...` — read this one carefully; see the clock
  note below. If they disagree the script also prints `BD_WARN`.
- `BD_ADDR` lines — the assigned register addresses. These are hard-coded in
  `board_main.c`; if you rename BD cells they move.
- `BD_DONE ... wns=` — post-route slack for the whole system. The kernel alone
  closed at +0.048 ns, so this is the number to watch: the DMA and
  interconnect share the same clock and can eat that margin.

Expect four critical warnings about negative `PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY`
values. Those come from Digilent's own board file and are normal for Zybo.

Output: `build/zybo/dcc_enc.xsa`, containing the bitstream and the hardware
handoff.

---

## 4. Vitis setup

Generate the test vectors first — the frame and the golden payload are linked
into the application as C arrays, so there is no file I/O on the first run:

```sh
make board_vectors      # -> build/board/{frame_data.c,golden_data.c,golden.bin}
```

Then create the workspace, platform and application:

```sh
/c/AMDDesignTools/2025.2/Vitis/bin/vitis -s scripts/vitis_setup.py \
    --xsa build/zybo/dcc_enc.xsa --workspace build/vitis
```

This is the 2025.2 unified flow (`vitis -s` runs a Python script against the
Vitis server; the old XSCT/Eclipse project model is gone). The script creates
a standalone platform on `ps7_cortexa9_0`, creates an empty application, and
imports the sources **flat into one directory** so `#include "nal.h"` resolves
with no include paths to configure:

```
board_main.c  codec_kernel.c/.h  h264_host.c/.h  platform_standalone.c
nal.c/.h  bitstream.c/.h  types.h          <- verbatim from src/
frame_data.c  golden_data.c                <- from make board_vectors
```

Note what is *not* being reimplemented. `src/nal.c` and `src/bitstream.c` are
the same files that produce the byte-exact reference stream on the
workstation. The board emits its parameter sets and slice header with exactly
that code, so the container cannot disagree with itself.

### By hand in the IDE instead

```sh
/c/AMDDesignTools/2025.2/Vitis/bin/vitis -w build/vitis
```

1. **Create Platform Component** from `build/zybo/dcc_enc.xsa`, OS
   `standalone`, processor `ps7_cortexa9_0`. Build it.
2. **Create Application Component** against that platform, domain
   `standalone_ps7_cortexa9_0`, template *Empty Application (C)*.
3. Drag the twelve files above into the component's `src/`.
4. Set optimization to `-O2` in the component's build settings. At `-O0` the
   staging memcpy and the payload compare dominate the timings the run
   reports, which makes the numbers meaningless.

---

## 5. Run it

1. Set **JP5 to JTAG** and connect the micro-USB (PROG/UART) port. Power on.
2. Open a terminal on the Zybo's serial port at **115200 8N1**.
3. In Vitis, **Run → Launch on Hardware**, with *Program FPGA* enabled in the
   run configuration so the bitstream is loaded before the ELF.

Expected output:

```
=== DCC codec kernel bring-up (L4) ===
kernel: ID 'H264' version 1.0 at 43c00000
frame: 18717 payload bytes, 174xxx cycles (1.7x ms, ~342 cycles/MB)
PASS: payload byte-exact with the C reference (18717 bytes)
access unit: 18744 bytes of Annex B, ready to pull off and decode
=== L4 PASSED ===
```

The two numbers that must match exactly are **18717 payload bytes** and the
byte-for-byte compare. The cycle count is informational; on the 480×272 frame
the simulation measured 342 cycles/MB, so expect roughly 174,000 cycles and
about 1.6 ms.

---

## 6. After it passes

In order, each step changing one thing:

1. **1080p.** Too big to link in, so load the frame over JTAG
   (`dow -data frame.bin 0x10000000` from xsdb) or from SD. Confirms the line
   buffers at full width and gives the real frame time.
2. **Multiple frames back to back.** Confirms the restart path, which
   simulation covers with two frames but hardware has never done.
3. **Pull the Annex B stream off and decode it with ffmpeg.** The on-board
   compare already proves the payload; this proves the container the board
   builds around it.
4. **Scatter-gather.** Drops the 3.1 MB staging memcpy by pointing two
   descriptors per macroblock row straight at the source buffer. Rebuild the
   DMA with `c_include_sg 1`; `h264_stream_chunks()` already generates the
   descriptor offsets and lengths.
5. **Frame-level rate control.** Free: `CONFIG` is latched at `START`, so the
   host writes a new QP per frame with no RTL change. The controller is
   already written and measured on `claude/p-frames`.

---

## Troubleshooting

**`ID` reads something other than `H264`.** Absent, held in reset, and wrong
base address all look identical from software, which is why the probe reads
`ID` before anything else. Check the bitstream actually programmed, that the
address matches the `BD_ADDR` output, and that `FCLK_RESET0_N` is released.

**Kernel never asserts `DONE`.** Read `STATUS`. Bit 2 is `s_axis_tready`: if
it is low the kernel is not being offered data, so look at MM2S. If it is high
and nothing is moving, the DMA never started — check the transfer length
against `h264_frame_bytes()` and that the buffer length register was built at
26 bits, since the 14-bit default silently caps a transfer at 16 kB.

**Payload correct for a while, then diverges and stays wrong.** A dropped or
duplicated input beat. The kernel has no way to notice, so the symptom appears
far from the cause.

**Sporadic, different every run.** Cache maintenance. The HP port is not
coherent with the A9 data cache, so the staged frame must be flushed before
MM2S reads it and the payload buffer invalidated before the CPU reads it.
This is the most likely first-run failure that is not in the kernel.

**Compile error on `XAxiDma_LookupConfig`.** Vitis moved that driver from
device-id to base-address lookup around 2023.2. `dma_init()` carries both
spellings behind `#ifdef XPAR_XAXIDMA_0_BASEADDR`; if neither matches your
BSP, check what `xparameters.h` actually defines.
