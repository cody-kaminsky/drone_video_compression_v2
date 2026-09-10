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
               $(SRC_DIR)/psnr.c
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

.PHONY: all ref hls clean test vectors bit_packer_vectors transform_vectors quant_vectors predict_vectors cavlc_cost_vectors recon_vectors line_buffer_vectors mb_header_vectors dispatch_vectors mode_decide_vectors pipeline_vectors

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

clean:
	rm -rf $(BUILD)
