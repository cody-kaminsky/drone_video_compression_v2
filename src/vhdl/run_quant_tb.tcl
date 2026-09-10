# run_quant_tb.tcl — xsim driver for quant_engine_tb (run from project root,
# after `make quant_vectors`).
xvhdl --2008 src/vhdl/quant_engine.vhd
xvhdl --2008 src/vhdl/quant_engine_tb.vhd
exec xelab quant_engine_tb -snapshot quant_test -debug typical
exec xsim quant_test -runall
