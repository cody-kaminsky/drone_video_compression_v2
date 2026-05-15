# run_transform_tb.tcl
#
# Standalone xsim driver for the transform_engine testbench.
# Uses xvhdl/xelab/xsim directly — no Vivado project needed.
#
# Usage (from Vivado Tcl console, cd to project root first):
#   cd C:/path/to/drone_video_compression_v2
#   source src/vhdl/run_transform_tb.tcl
#
# Prerequisite: vectors must be generated first.
#   make transform_vectors

xvhdl --2008 src/vhdl/transform_engine.vhd
xvhdl --2008 src/vhdl/transform_engine_tb.vhd
exec xelab transform_engine_tb -snapshot transform_test -debug typical
xsim transform_test -runall
