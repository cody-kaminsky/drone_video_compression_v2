# Makefile for the drone video compression C reference encoder.
#
# Targets:
#   make            - build build/dcc_encoder
#   make test       - run a smoke test on a synthetic frame
#   make clean      - remove build artifacts
#
# Works with gcc/clang on Linux/macOS and mingw-w64 on Windows.
# On Windows under git-bash you can also use this Makefile directly.

CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable
LDFLAGS ?=
LDLIBS  ?= -lm

SRC_DIR := src
BUILD   := build
BIN     := $(BUILD)/dcc_encoder

SRCS    := $(wildcard $(SRC_DIR)/*.c)
OBJS    := $(patsubst $(SRC_DIR)/%.c,$(BUILD)/%.o,$(SRCS))

.PHONY: all clean test

all: $(BIN)

$(BIN): $(OBJS)
	@mkdir -p $(BUILD)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/%.o: $(SRC_DIR)/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD):
	@mkdir -p $(BUILD)

# Smoke test: synthesize a 256x256 NV12 gradient frame, encode at QP 30,
# print stats. Verifies compile + a single end-to-end pass.
test: $(BIN)
	@python tools/make_test_frame.py $(BUILD)/test.yuv 256 256
	$(BIN) $(BUILD)/test.yuv 256 256 30 $(BUILD)/test_recon.yuv

clean:
	rm -rf $(BUILD)
