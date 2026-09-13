# M5 — packaging the kernel and validating it on hardware

Status: 2026-09-13. Target board for bring-up is a Zybo Z7-20
(`xc7z020clg400-1`) at 110 MHz, bare-metal.

This document covers two things that are easy to conflate. The first is
concrete: how the I-only H.264 kernel gets from an RTL directory onto a board
and how we know its output is right. The second is structural: which parts of
that are about H.264 and which parts are a socket the next codec can drop
into unchanged. Section 6 is the second thing; everything before it is the
first.

---

## 1. What is already true

The kernel is further along than "untested RTL". Before designing anything,
here is what has actually been measured, because it determines how much of
the bring-up is discovery and how much is confirmation.

**It fits the board, with room for the plumbing.** Out-of-context
implementation for `xc7z020clg400-1`:

| Resource | Used | Available | Share |
|---|---|---|---|
| LUTs | 24,050 | 53,200 | 45% |
| Flip-flops | 14,267 | 106,400 | 13% |
| BRAM tiles | 29.5 | 140 | 21% |
| DSPs | 49 | 220 | 22% |

The AXI DMA, the interconnect and the PS glue add roughly 6–8K LUTs, which
lands the whole design near 60%. That is comfortable.

**It is fast enough.** The whole-kernel measurement on three 1080p frames is
337 cycles per macroblock. At 110 MHz a 1080p frame is 8,160 macroblocks, so
25.0 ms, or 40 fps. 1080p30 is met with about a third to spare. This is worth
stating plainly because the assumption going in was that dropping from a
7030-3 at 200 MHz to a 7020-1 at 110 MHz would cost frame rate — it does not,
at 1080p. The margin that was spent was the 200 MHz headroom, not the frame
rate.

That headroom is the kernel's alone, though. It says nothing yet about
whether the PS can stage frames and drain payloads fast enough alongside it,
which is what L5 is for.

**Its output is already known-correct in simulation.**
`encoder_axi_top_tb` runs two frames through the real AXI interfaces and
compares the payload byte for byte against the C reference. It passes. The
C reference in turn decodes byte-exact under ffmpeg. So there is an unbroken
chain from the standard to the RTL — through simulation.

The open question is therefore narrow, and worth stating as such: **does the
hardware do what the simulation did.** Everything below is built to make the
answer to that question unambiguous, and to make a "no" point at the cause.

---

## 2. The packaging decision: what goes in the IP

The kernel presents three AXI interfaces and nothing else:

```
        AXI4-Lite  s_axi     control and status registers
        AXI4-Stream s_axis   NV12 samples in
        AXI4-Stream m_axis   slice payload out, tlast on the last beat
        irq                  level-high while DONE and IRQ_EN
```

The important choice here is not the interfaces, it is the **split between
what the PL produces and what the host produces**. The kernel emits the
macroblock layer only. Parameter sets, the slice header, start codes and
emulation prevention all stay in software.

That split is worth defending because it looks at first like leaving work on
the table. Three reasons it is right:

- **Frequency.** Header work happens once per frame; macroblock work happens
  8,160 times per frame. Hardware should own the inner loop and nothing else.
  The headers cost the A9 microseconds.
- **Volatility.** Parameter sets are where profile, level, VUI, cropping and
  colour signalling live. Those change for product reasons, not codec
  reasons. Changing a constant in C is free; changing it in RTL is a rebuild
  and a re-verification.
- **Shared source.** `src/nal.c` and `src/bitstream.c` are free of `malloc`
  and `stdio`, so the *same files* that produce the byte-exact reference
  stream on a workstation compile for bare-metal and run on the board. The
  container code is therefore not a second implementation that can disagree
  with the first. This is the single highest-value property of the split and
  it should be preserved for every future codec.

The kernel is packaged as a versioned Vivado IP (`scripts/package_ip.tcl`)
rather than added to the block design as a module reference, so that it
carries an identity and a fixed contract. That identity is readable at
runtime from the `ID` register, which matters more than it sounds like it
does — see section 5.

---

## 3. Two integration details that are easy to get wrong

These are the parts where the interface documentation is not enough and the
answer has to be derived. Both are now settled and both are covered by the
x86 test.

### 3.1 The frame arrives in macroblock-row order, not planar order

The kernel reads a frame as, per macroblock row, 16 luma lines then 8
interleaved-chroma lines, each the full frame width. That is not the order an
NV12 frame sits in memory.

It is, however, nearly free to produce, because **both runs are contiguous**.
Luma lines `16r … 16r+15` are one block of `16w` bytes; chroma lines `8r …
8r+7` are one block of `8w` bytes. So the reordering is an alternation of two
contiguous chunks, two per macroblock row — 136 chunks for 1080p — and never
a per-line gather.

That gives two implementations of the same ordering:

- **Bring-up:** one memcpy pass into a staging buffer, then a single
  contiguous DMA transfer. About 3.1 MB of copy per 1080p frame, so a few
  milliseconds on the A9. Wasteful, and correct, and has exactly one moving
  part.
- **Production:** two scatter-gather descriptors per macroblock row pointing
  straight at the camera buffer. No copy at all.

`h264_stream_chunks()` is the single source of truth for the order and both
paths are built from it, so they cannot drift apart.

Start with the memcpy. The point of the first bring-up run is to answer one
question, and a 136-descriptor ring is a second question wearing the same
clothes.

### 3.2 The payload's bit length is recoverable from the payload

The kernel emits the macroblock layer byte-aligned, terminated by a stop bit
and zero padding — exactly `rbsp_trailing_bits`. To splice it after a slice
header, which ends at an arbitrary bit offset, the host needs the *bit*
count, and the `BYTES` register only gives the byte count.

The bit count is not lost, though. The padding is all zeros, so **the last set
bit in the payload is the stop bit**:

```
mb_bits = 8 * (index of last nonzero byte + 1) - 1 - trailing_zeros(that byte)
```

This is exact, needs no extra register, and degrades usefully: a truncated or
never-written payload is all zeros and the function reports failure rather
than a plausible wrong number. Adding a `BITS` register would be marginally
cleaner, but it is not worth an RTL change and a re-verification.

Verified on the reference's own payload dump: 18,717 payload bytes → 149,730
macroblock-layer bits, and the reassembled access unit is byte-identical to
the reference `.264`.

---

## 4. The validation ladder

This is the core of the method. Each rung compares hardware-produced bytes
against **the same golden artifact**, and each rung adds exactly one new
thing that can be wrong. When a rung fails, the thing it added is the
suspect.

| Rung | What runs | What is compared | Status |
|---|---|---|---|
| L0 | C reference vs ffmpeg | decoded YUV vs our reconstruction | passing |
| L1 | per-block RTL testbenches | block vectors from the C reference | passing |
| L2 | `encoder_axi_top_tb` | slice payload vs `DCC_DUMP_SLICE` | passing |
| L3 | host stream assembly on x86 | reassembled `.264` vs reference `.264` | passing |
| L4 | board, staged frame, simple DMA | payload vs `DCC_DUMP_SLICE`, on-board | **passing** |
| L5 | board, scatter-gather, at rate | same, plus cycles and dropped frames | next |

L3 is the rung that was missing and is now closed. It matters more than its
position suggests: it means that when L4 runs for the first time, the input
ordering, the bit-count recovery, the header emission and the NAL escaping
are all already known-good. **The only untested thing left at L4 is the
hardware itself** — which is the entire point of building the ladder in this
order.

Run it with `make host_test`.

The invariant that makes this work is that L2, L3 and L4 all consume the same
golden bytes, produced once by the software reference. Keep that. The moment
a rung gets its own bespoke expected-output, the ladder stops being a ladder
and becomes four unrelated tests.

---

## 5. Bring-up plan for the Zybo

### 5.1 Build

```sh
make ip         # package as dcc:codec:dcc_h264_enc:1.0 into build/ip
make ip_check   # prove it instantiates and synthesizes from a block design
make zybo       # block design, bitstream, XSA  (needs board files, below)
```

`make ip_check` is not ceremony. Vivado raises a critical warning that
packaging a component with a VHDL-2008 top file is "not fully supported", and
the way a bad package fails is at block-design elaboration, long after you
have stopped thinking about packaging. The check instantiates the core,
asserts that all three interfaces came through as bus interfaces rather than
loose pins, sets both generics from IPI, and synthesizes.

For this core the warning turns out to be benign — `check_ip.tcl` passes, and
the design synthesizes through IP Integrator with the real hierarchy intact.
That is because the entity's own boundary is plain `std_logic` and `positive`
generics; the 2008 constructs are all internal. Keep the boundary that way in
any future kernel and the warning stays harmless. The check is what tells you
whether it still is.

**Board files are required, deliberately.** The PS7 DDR and MIO
configuration is board-specific. A wrong DDR configuration does not fail the
build — it produces a bitstream that boots and then corrupts memory under
load, which is among the worst things to debug because it looks like a codec
bug. `build_zybo_bd.tcl` refuses to run without the board files rather than
guessing. Install them through Vivado's XHub Store, or clone
`github.com/Digilent/vivado-boards` and set `BOARD_FILES` to its
`new/board_files` directory.

### 5.2 The first run

Target 480×272 first, not 1080p. It is the frame the whole vector chain
already uses, it fits comfortably in on-chip staging, and it finishes fast
enough to iterate. Move to 1080p only after it is byte-exact.

Per frame, the host:

1. writes `CONFIG` with `mbs_w`, `mbs_h`, `qp`
2. flushes the staged frame buffer from the data cache
3. starts the S2MM channel on the payload buffer
4. pulses `CTRL.START`
5. starts the MM2S channel on the staged frame
6. waits for `STATUS.DONE`
7. invalidates the payload buffer, reads `BYTES`
8. compares against the golden payload, and assembles the `.264`

`s_axis_tready` is held low until `START`, so steps 3–5 can be reordered
freely; the DMA may be armed before the kernel is started without corrupting
macroblock 0.

Note what step 7 does *not* need. Simple-mode S2MM cannot report how many
bytes it received — a real limitation of the AXI DMA, and normally the reason
people reach for scatter-gather earlier than they want to. It does not bite
here, because the kernel counts its own output and reports it in `BYTES`. That
is worth keeping in the contract for future codecs: **a kernel that reports
its own payload length lets the simplest possible DMA mode work**, and the
simplest DMA mode is what you want on the first day.

### 5.3 The cache flush is not optional

The HP port is not coherent with the A9 data cache. The staged frame must be
flushed before MM2S reads it, and the payload buffer must be invalidated
before the host reads it. Skipping either produces *intermittent*,
*data-dependent* corruption that looks exactly like an encoder bug and will
consume a day. This is the single most likely cause of a first-run mismatch
that is not in the kernel.

### 5.4 Reading a failure

Because the comparison is byte-for-byte against the golden payload, the index
of the first differing byte localises the fault. Convert it to a macroblock
by re-running the reference with `DCC_DUMP_MB` and finding which macroblock
spans that bit offset. Then:

- **Mismatch at byte 0, or payload length 0** — the stream never arrived.
  Look at `STATUS` bit 2 (`s_axis_tready`) and the DMA status registers, not
  at the codec.
- **Correct for a while, then diverges and stays wrong** — a dropped or
  duplicated input beat. The kernel has no way to notice, so the symptom
  appears far from the cause. Check DMA transfer length against
  `h264_frame_bytes()`.
- **Byte-exact but `BYTES` disagrees** — the tail of the payload was not
  drained. S2MM terminates on `tlast`; check the receive buffer was large
  enough that the DMA did not stop early.
- **Sporadic, changes run to run** — cache maintenance, or a timing failure.
  Check the post-route WNS before suspecting logic.

---

## 6. The reusable part

Everything in sections 2–5 that is not the letters "H.264" is the framework.
Concretely, three things are worth carrying forward.

### 6.1 The socket

A conforming codec kernel presents `s_axi`, `s_axis`, `m_axis`, `irq`, and a
256-byte register map whose **common prefix is codec-independent**:

```
0x00 CTRL     START / SOFT_RESET / IRQ_EN
0x04 CONFIG   codec-defined, latched at START     <- the codec's window
0x08 STATUS   BUSY / DONE / stream health
0x0C DONE_CLR
0x10 UNITS    units completed since reset
0x14 CYCLES   aclk cycles of the last unit
0x18 BYTES    payload bytes of the last unit
0x1C ID       fourcc, e.g. 'H264'
0x20 VERSION  major.minor
0x24+         codec-defined
```

The existing kernel already conforms, so this costs nothing to adopt. The
codec's own window is `0x04` and `0x24` upward; a codec needing more than one
config word takes `0x24+` and leaves the prefix alone.

`ID` and `VERSION` are what make this more than a naming convention. A kernel
that is absent, held in reset, or at the wrong base address looks *identical*
from software — all three give you garbage. Probing `ID` first turns all
three into one clear message, and it is the reason `dcc_kernel_probe()` is
the only way into the driver.

`host/codec_kernel.[ch]` implements this and knows nothing about H.264.
A second codec reuses it unchanged and writes its own `*_host.c`.

### 6.2 The split, again

Payload in hardware, container in software, container code shared verbatim
between the reference and the board. This is the property that made L3
possible. Any future codec that puts its header generation in RTL gives up
the ability to test the board's software path before the board exists, and
gives up having one implementation of the container instead of two.

### 6.3 The ladder

One golden artifact per test frame, produced by the software reference,
consumed identically by the RTL testbench, the x86 host test, and the board.
New codec, same shape: get the software reference byte-exact against a
reference decoder first, then make it dump the exact bytes the kernel should
produce, then never generate expected output any other way.

The reference needs exactly one hook for this, and it is cheap:
`DCC_DUMP_SLICE` in `src/encoder.c` is nine lines.

---

## 6b. Bugs hardware found that simulation did not

L4 passed on a single frame. Running a *sequence* broke, and the three faults
below are worth recording because of what each says about the test that missed
it.

### B1. `m_axis_tlast` is lost on one frame in eight (fatal with a real DMA)

**Symptom.** Frame 0 of a 1080p sequence encoded perfectly. Frame 1 completed
inside the kernel -- `FRAMES` incremented, `DONE` set, `BUSY` clear -- but
`m_axis_tvalid` was low and S2MM never went idle, so the host hung waiting for
a transfer that could never finish. `BYTES` read 253656 against a golden of
253657.

**The final byte is lost, not merely unmarked.** That distinction took a wrong
turn to find. Arming S2MM for exactly the golden length, so the channel
completes on byte count rather than `tlast`, did *not* help: it still waited,
because the byte genuinely never arrives. `frame_io`'s byte-to-word packer
only marks a word valid when `n = 4 or b_last_i = '1'`, so with `out_last`
lost, a final partial word is never presented at all. 253657 mod 4 = 1, so
exactly one byte stayed stuck in the register. One root cause, three
symptoms: a short payload, no `tlast`, and a hung DMA.

**Cause.** `bit_packer` emits a byte as soon as it has eight bits. Its own
header documents the consequence: *"flush_i with empty accum pulses flushed_o
the next cycle (no byte emitted, out_last not set)"*. So when the payload's bit
count including the RBSP stop bit is an exact multiple of 8, the last byte has
already left by the time the flush arrives, nothing is emitted, and `out_last`
is never asserted. That signal is wired straight through `cavlc_dispatch` and
`frame_io` to `m_axis_tlast`.

**Why it is one frame in eight.** The condition is purely
`(macroblock_layer_bits + 1) mod 8 == 0`, which is uniform over content. The
four 1080p frames tested gave 6, **0**, 4, 7 -- and exactly the one that gave 0
hung.

**Why simulation missed it.** The testbench checks `tlast` properly. It simply
never ran a frame that triggered the condition: both 480x272 frames it uses are
mod 8 = 3, and it ran the *same* frame twice, so it sampled one value of an
eight-valued variable.

**Reproduction, confirmed.** Encode the same 480x272 test frame at QP 23
instead of QP 26. That gives a 24451-byte payload with mod 8 = 0. Regenerate
the vectors, set the testbench's `QP` generic to 23, and run it:

```sh
DCC_DUMP_SRC=build/frame_src_words.txt DCC_DUMP_SLICE=build/slice_payload.txt     build/dcc_encoder.exe build/md_frame.yuv 480 272 23
python tools/gen_frame_stream.py build/md_frame.yuv 480 272 build/frame_stream.txt
```

Result: **zero byte mismatches** and `Failure: watchdog timeout` at 8 ms. Every
payload byte is correct and the testbench hangs waiting for `last_seen`, which
is exactly what the board did. This is the regression test for the fix -- it
must report PASS at QP 23 as well as QP 26.

**Fix direction.** Give `bit_packer` a `HOLD_LAST` generic, default false so
nothing else changes, and set it true only for the output packer in
`cavlc_dispatch`. With it set, emit only when `n_v >= 16` in normal operation,
so a byte is always held back and the flush always has one left to mark. The
one-byte latency is irrelevant at frame scale. Do not change the default: the
merger already compensates for per-block flush behaviour and would double up.

**Fixed.** `bit_packer` gained a `HOLD_LAST` generic, default false, enabled
only on the output packer in `cavlc_dispatch`. Validated in simulation:

| Case | (bits+stop) mod 8 | Before | After |
|---|---|---|---|
| QP 23, 24451 bytes | 0 | watchdog hang | PASS, 0 mismatches |
| QP 26, 18717 bytes | 3 | PASS | PASS, cycle counts unchanged |

`board_main.c` still arms S2MM for exactly the expected payload length. That
was written as a workaround and does not work as one -- the byte is missing,
not just unmarked -- but it is kept because completing on a known length is
the stricter check: a kernel that emits the wrong number of bytes hangs the
channel rather than quietly passing a truncated compare.

### B2. Data corruption under long output stalls

Changing the testbench's output backpressure from a tidy one-cycle-in-five
pattern to LFSR-driven stalls of up to 128 cycles produces byte mismatches
*early* in the frame, at byte 461 of 24451. That is not an end-of-frame
effect and it is not the same bug as B1.

A real AXI DMA does not deassert `tready` politely every fifth cycle; it
disappears for tens of cycles when its FIFO fills or DDR is busy. The regular
pattern exercised the handshake but never the sustained stall. `BP_MODE` on
`encoder_axi_top_tb` selects between the two; this is unfixed and needs
tracking down before the kernel can be trusted with a real capture path.

### B3. `bytes_last` undercounts by one beat (cosmetic)

In `encoder_axi_top.vhd` the byte accumulator and the capture into
`bytes_last` are separate signal assignments in the same clocked process:

```vhdl
if k_o_valid = '1' and m_axis_tready = '1' then
    bytes_run <= bytes_run + keep_count(k_o_keep);
end if;
if k_done = '1' then
    bytes_last <= bytes_run;     -- pre-update value
```

When the final beat handshakes in the same cycle `k_done` arrives,
`bytes_last` captures the value from before that beat. The hardware reported
253656 for a 253657-byte payload, and 253657 mod 4 = 1, so the missing beat
carried exactly one byte. Fix by adding `keep_count` when both conditions hold
in the same cycle.

---

## 7. What this does not yet cover

Honest list of what is still open, in the order it will probably matter.

- ~~The board has not run anything.~~ **L4 passed on 2026-09-13**: a Zybo
  Z7-20 at 100 MHz produced a payload byte-exact with the C reference, in
  174,350 cycles against simulation's 174,913. What remains untested is
  everything past a single 480x272 frame: multiple frames back to back, full
  width, and running at rate.
- **Frame ingest is a file, not a camera.** Bring-up loads a frame over JTAG
  or from SD. A real source needs the SG path and a capture front end, and
  the Zybo Z7-20 has no camera interface worth using for 1080p.
- **Rate control is host-side only.** `CONFIG` is latched at `START`, so the
  host can set a new QP every frame for free — frame-level rate control needs
  no RTL change at all. The per-macroblock QP refinement developed on
  `claude/p-frames` does need RTL work, because `mb_qp_delta` is currently
  always zero in the kernel's macroblock layer. Worth measuring what the
  refinement is actually worth before building it.
- **No deblocking.** The slice header says
  `disable_deblocking_filter_idc = 1` and the kernel does not filter, so the
  decoder must not either. For an I-only stream this is a quality choice, not
  a correctness one, and it can be revisited without touching the kernel by
  letting the decoder filter — intra prediction reads unfiltered samples by
  spec, so the reconstruction stays valid. That changes the byte-exactness
  check though, so do it after L4, not before.
- **Single clock domain.** The design runs the kernel and the AXI plumbing on
  one PL clock. If the camera front end brings its own clock, the crossing is
  new work.
- **No error signalling.** `STATUS` has no bit for "input stream ended
  early" or "payload buffer overflowed". Simulation cannot hit these; a real
  DMA can. Adding two sticky bits is cheap and would convert a class of
  confusing hardware failures into a register read.

---

## 8. Files

```
scripts/package_ip.tcl      package encoder_axi_top as dcc:codec:dcc_h264_enc:1.0
scripts/check_ip.tcl        prove the packaged IP is usable in IP Integrator
scripts/build_zybo_bd.tcl   Zybo Z7-20 block design, bitstream, XSA
scripts/impl_ooc.tcl        out-of-context implementation, for timing and area
host/codec_kernel.h/.c      codec-independent contract and driver
host/h264_host.h/.c         config word, stream ordering, payload to Annex B
host/test_assemble.c        L3: the x86 check of the board's software path
host/platform_standalone.c  the three platform calls, Xilinx bare-metal
host/board_main.c           L4: the board application  (never compiled yet)
tools/gen_board_vectors.py  frame and golden payload as C arrays
```

Everything above is built and passing except `board_main.c` and
`platform_standalone.c`, which need the Vitis BSP and are therefore the two
files that have never seen a compiler. That is deliberate and it is the whole
shape of the exercise: the untested surface has been squeezed down to the
board application and the board itself.

| Target | What it does |
|---|---|
| `make host_test` | run L3, the x86 check of the board's software path |
| `make ip` | package the kernel as `dcc:codec:dcc_h264_enc:1.0` |
| `make ip_check` | prove the packaged IP works in IP Integrator |
| `make zybo` | block design, bitstream and XSA (needs board files) |
| `make board_vectors` | the L4 frame and golden payload as linkable C arrays |
| `make impl_ooc` | re-measure timing and area after an RTL change |
