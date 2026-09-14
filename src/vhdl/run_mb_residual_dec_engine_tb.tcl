# run_mb_residual_dec_engine_tb.tcl
#
# Vivado xsim driver for the decode-side block sequencer testbench.
#
# Usage (from project root -- the testbench opens the vector file by a path
# relative to the working directory):
#   xvhdl --2008 src/vhdl/cavlc_pkg.vhd
#   xvhdl --2008 src/vhdl/cavlc_dec_tables.vhd
#   xvhdl --2008 src/vhdl/bit_reader.vhd
#   xvhdl --2008 src/vhdl/cavlc_dec_engine.vhd
#   xvhdl --2008 src/vhdl/mb_residual_dec_engine.vhd
#   xvhdl --2008 src/vhdl/mb_residual_dec_engine_tb.vhd
#   xelab work.mb_residual_dec_engine_tb -snapshot res_tb -debug off
#   xsim res_tb -runall
#
# Prerequisite: vectors are generated.
#   make mb_residual_dec_vectors
#
# Beyond the coefficients the testbench checks the neighbour total_coeff the
# macroblock hands on, because an error there decodes the NEXT macroblock
# wrongly while this one looks perfect, and the number of bits retired,
# because a sequencer can emit entirely correct blocks while leaving the
# reader one bit out of step.
