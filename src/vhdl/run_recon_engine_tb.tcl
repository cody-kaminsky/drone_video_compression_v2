# run_recon_engine_tb.tcl — xsim driver for recon_engine_tb (run from project root,
# after `make recon_vectors`).
xvhdl --2008 src/vhdl/recon_engine.vhd
xvhdl --2008 src/vhdl/recon_engine_tb.vhd
exec xelab recon_engine_tb -snapshot recon_engine_snap -debug typical
exec xsim recon_engine_snap -runall
