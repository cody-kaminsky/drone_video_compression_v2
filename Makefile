# Makefile for the drone video compression encoders.
#
# Builds two binaries:
#   build/dcc_encoder   - C reference (M2). Full recon plane in static memory.
#   build/dcc_hls       - HLS port (M3). Line-buffered recon, otherwise
#                         byte-identical with the C reference. Uses the same
#                         transform / quant / intra / cavlc modules.
#
# Targets:
#   make            - build both binaries
#   make ref        - build only build/dcc_encoder
#   make hls        - build only build/dcc_hls
#   make test       - run a smoke test on the C reference
#   make clean      - remove build artifacts
#
# Works with gcc/clang on Linux/macOS and mingw-w64 on Windows.

CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable
LDFLAGS ?=
LDLIBS  ?= -lm

SRC_DIR := src
HLS_DIR := src/hls
BUILD   := build

# Modules shared by both binaries. These compile to the same .o files
# regardless of which encoder front-end links them.
SHARED_SRCS := $(SRC_DIR)/transform.c $(SRC_DIR)/quant.c $(SRC_DIR)/intra.c \
               $(SRC_DIR)/cavlc.c $(SRC_DIR)/bitstream.c $(SRC_DIR)/nal.c \
               $(SRC_DIR)/psnr.c $(SRC_DIR)/inter.c $(SRC_DIR)/deblock.c
SHARED_OBJS := $(patsubst $(SRC_DIR)/%.c,$(BUILD)/%.o,$(SHARED_SRCS))

# C-reference-only sources: top-level encoder loop + driver.
REF_SRCS := $(SRC_DIR)/encoder.c $(SRC_DIR)/main.c
REF_OBJS := $(patsubst $(SRC_DIR)/%.c,$(BUILD)/%.o,$(REF_SRCS))

# HLS-port sources.
HLS_SRCS := $(HLS_DIR)/encoder.c $(HLS_DIR)/line_buffer.c \
            $(HLS_DIR)/hls_top.c $(HLS_DIR)/main.c
HLS_OBJS := $(patsubst $(HLS_DIR)/%.c,$(BUILD)/hls/%.o,$(HLS_SRCS))

BIN_REF := $(BUILD)/dcc_encoder
BIN_HLS := $(BUILD)/dcc_hls

.PHONY: rc_vectors impl_ooc host_test ip ip_check zybo board_vectors board_seq_tools board_seq all ref hls clean test vectors bit_packer_vectors transform_vectors quant_vectors predict_vectors cavlc_cost_vectors recon_vectors line_buffer_vectors mb_header_vectors dispatch_vectors mode_decide_vectors pipeline_vectors

all: $(BIN_REF) $(BIN_HLS)
ref: $(BIN_REF)
hls: $(BIN_HLS)
vectors: $(BUILD)/gen_cavlc_vectors
bit_packer_vectors: $(BUILD)/bit_packer_vectors_in.txt
transform_vectors: $(BUILD)/transform_vectors.txt
quant_vectors: $(BUILD)/quant_vectors.txt
predict_vectors: $(BUILD)/predict4x4_vectors.txt
cavlc_cost_vectors: $(BUILD)/cavlc_cost_vectors.txt
recon_vectors: $(BUILD)/recon_vectors.txt
line_buffer_vectors: $(BUILD)/line_buffer_vectors.txt
mb_header_vectors: $(BUILD)/mb_header_vectors.txt
dispatch_vectors: $(BUILD)/dispatch_vectors_in.txt
mode_decide_vectors: $(BUILD)/mode_decide_vectors.txt
pipeline_vectors: $(BUILD)/slice_payload.txt

# CAVLC vector generator for the VHDL CAVLC engine testbench. Links against
# the shared kernel (just needs cavlc.c + bitstream.c).
$(BUILD)/gen_cavlc_vectors: tools/gen_cavlc_vectors.c \
                            $(BUILD)/cavlc.o $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

# bit_packer vector generator. Just needs bitstream.c (drives bs_put_bits).
$(BUILD)/gen_bit_packer_vectors: tools/gen_bit_packer_vectors.c \
                                  $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

# Transform vector generator. Links against transform.c only.
$(BUILD)/gen_transform_vectors: tools/gen_transform_vectors.c \
                                 $(BUILD)/transform.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

$(BUILD)/transform_vectors.txt: $(BUILD)/gen_transform_vectors
	./$<

# Quant vector generator. Links against quant.c only.
$(BUILD)/gen_quant_vectors: tools/gen_quant_vectors.c $(BUILD)/quant.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

$(BUILD)/quant_vectors.txt: $(BUILD)/gen_quant_vectors
	./$<

# Intra prediction vector generator (4x4, 16x16, chroma). Links intra.c.
$(BUILD)/gen_predict_vectors: tools/gen_predict_vectors.c $(BUILD)/intra.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

$(BUILD)/predict4x4_vectors.txt: $(BUILD)/gen_predict_vectors
	./$<

# CAVLC cost (mode-decision bit estimate) vector generator.
$(BUILD)/gen_cavlc_cost_vectors: tools/gen_cavlc_cost_vectors.c                                  $(BUILD)/cavlc.o $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

$(BUILD)/cavlc_cost_vectors.txt: $(BUILD)/gen_cavlc_cost_vectors
	./$<

# Reconstruction (+SSD) vector generator.
$(BUILD)/gen_recon_vectors: tools/gen_recon_vectors.c | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $<

$(BUILD)/recon_vectors.txt: $(BUILD)/gen_recon_vectors
	./$<

# Line buffer vectors: drives the HLS line_buffer_t directly.
$(BUILD)/gen_line_buffer_vectors: tools/gen_line_buffer_vectors.c $(HLS_DIR)/line_buffer.c | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I$(HLS_DIR) -o $@ $^

$(BUILD)/line_buffer_vectors.txt: $(BUILD)/gen_line_buffer_vectors
	./$<

# MB header vectors.
$(BUILD)/gen_mb_header_vectors: tools/gen_mb_header_vectors.c $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -Wno-unused-const-variable -Wno-unused-variable -I$(SRC_DIR) -o $@ $^

$(BUILD)/mb_header_vectors.txt: $(BUILD)/gen_mb_header_vectors
	./$<

# CAVLC dispatch / merge vectors.
$(BUILD)/gen_dispatch_vectors: tools/gen_dispatch_vectors.c $(BUILD)/cavlc.o $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

$(BUILD)/dispatch_vectors_in.txt: $(BUILD)/gen_dispatch_vectors
	./$<

# Mode-decision vectors: per-MB records dumped by the reference encoder on a
# natural test frame (needs ffmpeg for the PNG -> NV12 conversion).
$(BUILD)/mode_decide_vectors.txt: $(BIN_REF) tools/frames/old_town_cross_480x272.png
	ffmpeg -y -loglevel error -i tools/frames/old_town_cross_480x272.png -pix_fmt nv12 -f rawvideo $(BUILD)/md_frame.yuv
	rm -f $@
	for qp in 26 20 34; do DCC_DUMP_MB=$@ DCC_DUMP_N=510 $(BIN_REF) $(BUILD)/md_frame.yuv 480 272 $$qp > /dev/null; done

# Frame-level vectors for mb_pipeline_controller_tb / encoder_top_tb: the
# source blocks, the pixel stream and the slice payload the reference emits.
$(BUILD)/slice_payload.txt: $(BIN_REF) tools/frames/old_town_cross_480x272.png tools/gen_frame_stream.py
	ffmpeg -y -loglevel error -i tools/frames/old_town_cross_480x272.png -pix_fmt nv12 -f rawvideo $(BUILD)/md_frame.yuv
	DCC_DUMP_SRC=$(BUILD)/frame_src_words.txt DCC_DUMP_SLICE=$@ $(BIN_REF) $(BUILD)/md_frame.yuv 480 272 26 > /dev/null
	python tools/gen_frame_stream.py $(BUILD)/md_frame.yuv 480 272 $(BUILD)/frame_stream.txt

# Generate vectors. The C tool writes both files; we touch one to mark
# completion (the tool runs in $(BUILD)/.. since paths in the tool are
# relative, so invoke it from the project root).
$(BUILD)/bit_packer_vectors_in.txt: $(BUILD)/gen_bit_packer_vectors
	./$<

$(BIN_REF): $(SHARED_OBJS) $(REF_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BIN_HLS): $(SHARED_OBJS) $(HLS_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/%.o: $(SRC_DIR)/%.c | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -c -o $@ $<

$(BUILD)/hls/%.o: $(HLS_DIR)/%.c | $(BUILD)/hls
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I$(HLS_DIR) -c -o $@ $<

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/hls:
	@mkdir -p $(BUILD)/hls

# Smoke test on the C reference.
test: $(BIN_REF)
	@python tools/make_test_frame.py $(BUILD)/test.yuv 256 256
	$(BIN_REF) $(BUILD)/test.yuv 256 256 30 $(BUILD)/test_recon.yuv

# Out-of-context implementation (synth, place, route) of the AXI wrapper;
# reports and checkpoints in build/impl. Needs vivado on PATH.
impl_ooc:
	@mkdir -p $(BUILD)/impl
	vivado -mode batch -source scripts/impl_ooc.tcl -log $(BUILD)/impl/vivado.log -journal $(BUILD)/impl/vivado.jou -tclargs $(BUILD)/impl 1920 2

clean:
	rm -rf $(BUILD)

# ---------------------------------------------------------------- host ---
# Board-side stream assembly, checked on a workstation against the reference.
# The same h264_host.c compiles for bare-metal; only the platform shim differs.
HOST_DIR := host

$(BUILD)/test_assemble: $(HOST_DIR)/test_assemble.c $(HOST_DIR)/h264_host.c \
                        $(BUILD)/nal.o $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I$(HOST_DIR) -o $@ $^ $(LDLIBS)

# Encode the test frame, dump the payload the kernel would produce and the
# pixel stream it would be fed, then check the host path reproduces both.
host_test: $(BUILD)/test_assemble $(BIN_REF) tools/frames/old_town_cross_480x272.png
	@ffmpeg -y -loglevel error -i tools/frames/old_town_cross_480x272.png \
	        -pix_fmt nv12 -f rawvideo $(BUILD)/md_frame.yuv
	@DCC_DUMP_SLICE=$(BUILD)/ht_payload.txt $(BIN_REF) $(BUILD)/md_frame.yuv 480 272 26 \
	        $(BUILD)/ht_recon.yuv $(BUILD)/ht_ref.264 --no-deblock > /dev/null
	@python tools/gen_frame_stream.py $(BUILD)/md_frame.yuv 480 272 $(BUILD)/ht_stream.txt > /dev/null
	./$(BUILD)/test_assemble $(BUILD)/md_frame.yuv 480 272 26 \
	        $(BUILD)/ht_payload.txt $(BUILD)/ht_stream.txt $(BUILD)/ht_ref.264

# --------------------------------------------------- hardware packaging ---
# VIVADO is not on PATH in a default install; override if yours differs.
VIVADO ?= /c/AMDDesignTools/2025.2/Vivado/bin/vivado.bat
PART   ?= xc7z020clg400-1
CLKMHZ ?= 110

# Package the kernel as dcc:codec:dcc_h264_enc:1.0 into build/ip.
ip:
	@mkdir -p $(BUILD)
	$(VIVADO) -mode batch -source scripts/package_ip.tcl 	    -log $(BUILD)/ip_pkg.log -journal $(BUILD)/ip_pkg.jou 	    -tclargs $(BUILD)/ip $(PART)

# Prove the packaged IP instantiates, elaborates and synthesizes from a block
# design. Run after `make ip`, before trusting it in a board design.
ip_check:
	$(VIVADO) -mode batch -source scripts/check_ip.tcl 	    -log $(BUILD)/ip_check.log -journal $(BUILD)/ip_check.jou 	    -tclargs $(BUILD)/ip $(PART)

# Zybo Z7-20 block design, bitstream and XSA. Needs Digilent board files;
# see docs/m5-hw-validation.md 5.1.
zybo:
	$(VIVADO) -mode batch -source scripts/build_zybo_bd.tcl 	    -log $(BUILD)/zybo.log -journal $(BUILD)/zybo.jou 	    -tclargs $(BUILD)/zybo $(BUILD)/ip $(CLKMHZ)

# L4 vectors: the input frame and the golden payload as C arrays, to link
# into the Vitis application. See docs/m5-hw-validation.md 5.2.
board_vectors: $(BIN_REF) tools/frames/old_town_cross_480x272.png
	@mkdir -p $(BUILD)/board
	@ffmpeg -y -loglevel error -i tools/frames/old_town_cross_480x272.png 	        -pix_fmt nv12 -f rawvideo $(BUILD)/md_frame.yuv
	@DCC_DUMP_SLICE=$(BUILD)/board/payload.txt $(BIN_REF) 	        $(BUILD)/md_frame.yuv 480 272 26 > /dev/null
	python tools/gen_board_vectors.py $(BUILD)/md_frame.yuv 	        $(BUILD)/board/payload.txt $(BUILD)/board

# Tool that writes a frame in the kernel's stream order, using the same
# h264_nv12_to_stream() the x86 test covers. One implementation of the order.
board_seq_tools: $(BUILD)/gen_stream_frame

$(BUILD)/gen_stream_frame: tools/gen_stream_frame.c $(HOST_DIR)/h264_host.c                            $(BUILD)/nal.o $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I$(HOST_DIR) -o $@ $^ $(LDLIBS)

# L5 sequence for the board: frames in kernel stream order, golden payloads,
# a manifest and an xsdb loader. Override any of these on the command line.
#   make board_seq SEQ=build/zoom_1080p.yuv W=1920 H=1088 QP=26 FRAMES=4 REPEATS=25
SEQ     ?= build/zoom_1080p.yuv
W       ?= 1920
H       ?= 1088
QP      ?= 26
FRAMES  ?= 4
REPEATS ?= 25
SEQOUT  ?= $(BUILD)/seq_board

board_seq: $(BIN_REF) $(BUILD)/gen_stream_frame
	python tools/gen_board_sequence.py $(SEQ) $(W) $(H) $(QP) 	    --frames $(FRAMES) --repeats $(REPEATS) --out $(SEQOUT)

# ------------------------------------------------------- rate control ---
# Vectors for encoder_axi_top_rc_tb: RC_SEQ is a raw NV12 file (foreman CIF,
# 352x288, is what the numbers in docs/rate-control-rtl.md were taken on);
# rc_vectors writes build/rc_hw (per-MB QP in the kernel) and build/rc_off
# (frame QP only, the same frames). Copy one config.txt to build/rc_tb/ and
# run src/vhdl/run_encoder_axi_top_rc_tb.tcl.
RC_SEQ ?= build/rc_src.yuv
RC_W   ?= 352
RC_H   ?= 288
RC_N   ?= 3
RC_OPTS ?= --bitrate 3000000 --qp-min 16 --qp-max 40
rc_vectors: $(BIN_REF) tools/gen_rc_vectors.sh
	bash tools/gen_rc_vectors.sh $(RC_SEQ) $(RC_W) $(RC_H) $(RC_N) $(BUILD)/rc_hw hw $(RC_OPTS)
	bash tools/gen_rc_vectors.sh $(RC_SEQ) $(RC_W) $(RC_H) $(RC_N) $(BUILD)/rc_off off $(RC_OPTS)
