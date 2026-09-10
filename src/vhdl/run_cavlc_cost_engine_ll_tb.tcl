# run_cavlc_cost_engine_ll_tb.tcl — xsim driver for cavlc_cost_engine_ll_tb (run from project root,
# after `make cavlc_cost_vectors`).
xvhdl --2008 src/vhdl/cavlc_pkg.vhd
xvhdl --2008 src/vhdl/cavlc_cost_engine_ll.vhd
xvhdl --2008 src/vhdl/cavlc_cost_engine_ll_tb.vhd
exec xelab cavlc_cost_engine_ll_tb -snapshot cavlc_cost_engine_ll_snap -debug typical
exec xsim cavlc_cost_engine_ll_snap -runall
