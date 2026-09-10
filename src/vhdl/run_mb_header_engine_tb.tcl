# run_mb_header_engine_tb.tcl — xsim driver for mb_header_engine_tb (run from project
# root, after `make mb_header_vectors`).
xvhdl --2008 src/vhdl/mb_header_engine.vhd
xvhdl --2008 src/vhdl/mb_header_engine_tb.vhd
exec xelab mb_header_engine_tb -snapshot mb_header_engine_snap -debug typical
exec xsim mb_header_engine_snap -runall
