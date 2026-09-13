# Bring-up runbook — Zybo Z7-20

The practical sequence, with the values this project's scripts actually
produced. The reasoning behind it all is in `docs/m5-hw-validation.md`; this
file is the checklist.

**L4 passed on hardware on 2026-09-13.** The kernel produced a payload
byte-exact with the C reference on a Zybo Z7-20 at 100 MHz. Everything in this
file has now been run for real except the follow-on work in section 6.

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

| FCLK | Period | Kernel OOC slack | In-system slack | 1080p fps |
|---|---|---|---|---|
| 111.111 MHz | 9.000 ns | about -0.04 ns | about -0.9 ns | 40.4 |
| 100 MHz | 10.000 ns | n/a | **+0.094 ns (built)** | 36.4 |

**Out-of-context slack does not extrapolate to in-system slack.** The built
design closed at only +0.094 ns, not the nanosecond of margin a linear
extrapolation from the OOC run suggests. The worst path is the mode decider's
transform input select into the transform row register -- the same path the
OOC run found -- but it went from about 9.04 ns standalone to 9.767 ns in the
system, and 66% of that is routing, not logic.

The reason is congestion. The encoder is 24k LUTs and the plumbing adds
another 1.9k, so the device sits at 49% and the placer has much less freedom
than it does with the kernel alone. Nothing about the kernel changed; its
routing just got worse.

Two things follow. First, 100 MHz was not a conservative choice, it was the
necessary one: at 111.111 MHz this design would have missed by roughly 0.9 ns,
not the 0.04 ns the OOC number implied. Second, re-measure in-system after any
RTL change; `make impl_ooc` is a fast proxy for the kernel in isolation but it
flatters the real design by most of a nanosecond on this part.

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

**Built result**, for comparison when you rebuild:

| | Value |
|---|---|
| WNS / WHS | +0.094 ns / +0.024 ns, no failing endpoints |
| LUTs | 25,945 of 53,200 (49%) |
| Flip-flops | 18,183 of 106,400 (17%) |
| BRAM tiles | 34.5 of 140 (25%) |
| DSPs | 49 of 220 (22%) |

The plumbing -- DMA, two interconnects, reset and concat -- costs about 1,900
LUTs and 5 BRAM tiles on top of the kernel's 24,063.

---

## 4. Vitis setup

Generate the test sequence first. It is loaded into DDR over JTAG rather than
linked into the ELF, so the application is resolution-independent and changing
the sequence never needs a recompile:

```sh
make board_seq_tools
make board_seq SEQ=build/zoom_1080p.yuv W=1920 H=1088 QP=26 FRAMES=4 REPEATS=25
```

That writes, into `build/seq_board`:

| File | Contents |
|---|---|
| `frame_NNN.bin` | the frame in the kernel's **stream order** |
| `golden_NNN.bin` | the payload the C reference says the kernel must emit |
| `manifest.bin` | geometry, QP, repeats, and the address of every frame |
| `load.tcl` | an xsdb script that pushes it all into DDR |

Frames are pre-shuffled on the workstation by the same `h264_nv12_to_stream()`
that `make host_test` checks, so **the board does no staging copy at all** --
MM2S reads each frame where it lies. At 1080p that removes about 3 MB of
memcpy per frame, which was the main thing eating the 33 ms budget at 30 fps
and the main argument for scatter-gather.

Because the encoder is intra only, frames are independent: the kernel carries
no state between them. So four frames cycled twenty-five times exercises
restart and sustained throughput exactly as well as a hundred distinct
frames, and loads in thirteen seconds instead of minutes.

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
board_main.c  codec_kernel.c/.h  h264_host.c/.h  dcc_memmap.h
platform_standalone.c
nal.c/.h  bitstream.c/.h  types.h          <- verbatim from src/
```

Twelve files, and no generated data: the frames and goldens live in DDR, not
in the ELF. The application comes out at about 69 kB of text and 25 kB of bss,
whatever the resolution.

Note what is *not* being reimplemented. `src/nal.c` and `src/bitstream.c` are
the same files that produce the byte-exact reference stream on the
workstation. The board emits its parameter sets and slice header with exactly
that code, so the container cannot disagree with itself.

### By hand in the IDE instead

```sh
/c/AMDDesignTools/2025.2/Vitis/bin/vitis.bat -w build/vitis
```

1. **File -> New Component -> Platform.** Name `dcc_plat`, hardware design
   `build/zybo/dcc_enc.xsa`, OS `standalone`, processor `ps7_cortexa9_0`.
   Finish, then select it and **Build**. A few minutes; it also generates an
   FSBL you do not need for JTAG bring-up.
2. **File -> New Component -> Application.** Name `dcc_l4`, platform
   `dcc_plat`, domain `standalone_ps7_cortexa9_0`, template
   *Empty Application (C)*.
3. **Copy the twelve files into `dcc_l4/src/`.** The component's
   `CMakeLists.txt` calls `aux_source_directory` on that directory, so
   anything dropped there is compiled with no registration step -- a plain
   filesystem copy plus a refresh is enough. (The scripted path puts them in
   the component root instead and registers them through
   `USER_COMPILE_SOURCES`; both work, `src/` is the simpler one by hand.)
4. **Set `-O2`** in the component's build settings. At `-O0` the staging
   memcpy and the payload compare dominate the timings the run reports.
5. **Build.**

### What actually goes wrong first

The application has now been built on this machine, and two things needed
fixing that are worth knowing about because both are Cortex-A9 specific:

- **`xtime_l.h: No such file or directory`.** That header exists on
  UltraScale+ (A53/R5) but not in a Cortex-A9 system-device-tree BSP. The
  same `XTime_GetTime` and `COUNTS_PER_SECOND` come from **`xiltimer.h`**,
  which pulls in `xtimer_config.h`. Already fixed in
  `host/platform_standalone.c`.
- **`XAxiDma_Busy` returns `u32`, not `int`**, so `%d` is a `-Wformat` error
  under the default `-Wall -Wextra`. Already fixed in `host/board_main.c`.

Note also that `COUNTS_PER_SECOND` expands to a bare
`XPAR_CPU_CORE_CLOCK_FREQ_HZ/2` with no parentheses, so parenthesise it before
dividing again or it reassociates.

Result: `build/vitis/dcc_l4/build/dcc_l4.elf`, about 554 kB, 281 kB of text
and 1.27 MB of bss (the staging, payload, Annex B and scratch buffers). To
rebuild after editing a source without going through the IDE:

```sh
cd build/vitis/dcc_l4/build && ninja
```

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
byte-for-byte compare.

**Measured on hardware**, for comparison on future runs:

| | Simulation | Hardware |
|---|---|---|
| Payload bytes | 18,717 | 18,717 |
| Cycles | 174,913 | **174,350** |
| Cycles/MB | 342 | 342 |
| Frame time at 100 MHz | - | 1.74 ms |

Hardware came in 563 cycles (0.32%) *faster* than simulation, which is worth
understanding rather than shrugging at. Two effects cancel. `board_main.c`
pulses START before programming the MM2S transfer, so the kernel idles while
the CPU writes DMA registers, which should inflate the count. Against that,
the real DMA streams from DDR with fewer gaps than the testbench's feed, so
the kernel stalls less on input. The second effect is the larger one.

The useful conclusion: **the kernel is compute-bound on hardware, not
input-bound**, so the data path has margin. A count far above this, say
200,000+, would mean the opposite and would point at the DMA rather than the
encoder.

Extrapolating to 1080p at 342 cycles/MB: 8,160 macroblocks is 27.9 ms, so
35.8 fps. 1080p30 with about 19% to spare.

---

## 5b. Loading the sequence and running it

The ELF and the data are loaded separately, and the application waits for the
data rather than needing a breakpoint.

1. In Vitis, **Run -> Launch on Hardware** with *Program FPGA* enabled. Let it
   run. The UART prints:

   ```
   waiting for a manifest at 03000000 ...
     run the generated load.tcl now:
       xsdb <seq dir>/load.tcl
   ```

2. In a second terminal:

   ```sh
   xsdb build/seq_board_1080/load.tcl
   ```

The script connects, halts core 0, `dow -data`s every frame and golden to its
address, writes the manifest last, and resumes into the poll. The application
sees the magic appear and starts encoding.

Two details that matter. The manifest is written **last** because its magic
number is what tells the application the rest of the data really arrived, so a
load that dies halfway leaves it waiting rather than running on garbage. And
the poll **invalidates the cache on every pass**: JTAG writes DDR behind the
data cache, so without that the CPU would hold the stale line forever and
never see the manifest arrive.

The wait times out after 300 seconds.

Expected output for a 4-frame 1080p sequence at QP 26:

```
=== DCC codec kernel, sequence run (L5) ===
kernel:   ID 'H264' version 1.0 at 43c00000, 100 MHz
sequence: 1920x1088 QP26, 4 frames x 25 repeats, 8160 MBs/frame
  frame  0: 256713 bytes, 2750000 cycles (27.50 ms, 337 cy/MB) OK
  frame  1: 253657 bytes, ...
  ...
PASS: 100 frames encoded, every payload byte-exact with the C reference
  cycles   avg ...  min ...  max ...  (337.0 cy/MB)
  frame    27.50 ms -> 36.4 fps at 100 MHz
  bitrate  ... bytes total, 61.0 Mbps at 30 fps
  30 fps   MET (needs <= 33.33 ms/frame)
```

The payload byte counts are fixed by the reference and must match exactly:
**256713, 253657, 252840, 252415** for the first four frames of this clip.
The cycle counts are the measurement.

Note the bitrate line. At QP 26 a 1080p frame is about 254 kB, which is
61 Mbps at 30 fps -- twice the 30 Mbps the project targets. That is expected
for intra-only at this QP and is what the rate control on `claude/p-frames`
exists to fix; it is not a bring-up problem.

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
