# run_cavlc_dec_engine_tb.tcl
#
# Vivado xsim driver for the cavlc_dec_engine testbench.
#
# Usage (from project root -- the testbench opens the vector file by a path
# relative to the working directory):
#   xvhdl --2008 src/vhdl/cavlc_pkg.vhd
#   xvhdl --2008 src/vhdl/cavlc_dec_tables.vhd
#   xvhdl --2008 src/vhdl/bit_reader.vhd
#   xvhdl --2008 src/vhdl/cavlc_dec_engine.vhd
#   xvhdl --2008 src/vhdl/cavlc_dec_engine_tb.vhd
#   xelab work.cavlc_dec_engine_tb -snapshot dec_tb -debug off
#   xsim dec_tb -runall
#
# Prerequisite: vectors are generated.
#   make cavlc_dec_vectors
#
# The testbench starves the bit reader pseudorandomly by default (IN_BP = 1)
# and fails the run if the reader never actually ran dry, so a pass means the
# engine held its decisions across gaps in the input rather than merely
# working when the bits were always there. Set IN_BP = 0 for the easy case.
# Note the .bat wrappers mangle -generic_top NAME=VAL from a shell; change the
# default in the entity if you need the other setting.
