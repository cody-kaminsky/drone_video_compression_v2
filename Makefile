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

.PHONY: all ref hls clean test vectors

all: $(BIN_REF) $(BIN_HLS)
ref: $(BIN_REF)
hls: $(BIN_HLS)
vectors: $(BUILD)/gen_cavlc_vectors

# CAVLC vector generator for the VHDL CAVLC engine testbench. Links against
# the shared kernel (just needs cavlc.c + bitstream.c).
$(BUILD)/gen_cavlc_vectors: tools/gen_cavlc_vectors.c \
                            $(BUILD)/cavlc.o $(BUILD)/bitstream.o | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDLIBS)

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
