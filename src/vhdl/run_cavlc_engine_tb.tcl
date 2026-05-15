# run_cavlc_engine_tb.tcl
#
# Vivado xsim driver for cavlc_engine testbench.
#
# Usage (from project root):
#   vivado -mode batch -source src/vhdl/run_cavlc_engine_tb.tcl
#
# Or standalone xsim commands:
#   xvhdl --2008 src/vhdl/cavlc_pkg.vhd
#   xvhdl --2008 src/vhdl/cavlc_tables.vhd
#   xvhdl --2008 src/vhdl/coeff_token_encoder.vhd
#   xvhdl --2008 src/vhdl/bit_packer.vhd
#   xvhdl --2008 src/vhdl/cavlc_engine.vhd
#   xvhdl --2008 src/vhdl/cavlc_engine_tb.vhd
#   xelab cavlc_engine_tb -snapshot cavlc_test -debug typical
#   xsim cavlc_test -runall
#
# Prerequisite: vectors are generated.
#   make vectors   (or: build/gen_cavlc_vectors)
