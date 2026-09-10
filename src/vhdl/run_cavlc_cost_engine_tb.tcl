# run_cavlc_cost_engine_tb.tcl — xsim driver for cavlc_cost_engine_tb (run from project root,
# after `make cavlc_cost_vectors`).
xvhdl --2008 src/vhdl/cavlc_pkg.vhd
xvhdl --2008 src/vhdl/cavlc_cost_engine.vhd
xvhdl --2008 src/vhdl/cavlc_cost_engine_tb.vhd
exec xelab cavlc_cost_engine_tb -snapshot cavlc_cost_engine_snap -debug typical
exec xsim cavlc_cost_engine_snap -runall
