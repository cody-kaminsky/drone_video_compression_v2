# run_mb_header_dec_engine_tb.tcl
#
# Vivado xsim driver for the mb_header_dec_engine testbench.
#
# Usage (from project root -- the testbench opens the vector file by a path
# relative to the working directory):
#   xvhdl --2008 src/vhdl/bit_reader.vhd
#   xvhdl --2008 src/vhdl/mb_header_dec_engine.vhd
#   xvhdl --2008 src/vhdl/mb_header_dec_engine_tb.vhd
#   xelab work.mb_header_dec_engine_tb -snapshot hdr_tb -debug off
#   xsim hdr_tb -runall
#
# Prerequisite: vectors are generated.
#   make mb_header_dec_vectors
#
# The testbench starves the bit reader pseudorandomly by default (IN_BP = 1)
# and fails the run if the reader never actually ran dry. It also checks the
# number of bits retired against the encoder's own header length, which no
# result port would reveal: every field can be right while the engine is one
# bit out of step, and the damage then lands on the residual blocks.
