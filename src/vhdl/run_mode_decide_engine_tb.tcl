# run_mode_decide_engine_tb.tcl — xsim driver for mode_decide_engine_tb (run from
# project root, after `make mode_decide_vectors`).
xvhdl --2008 src/vhdl/cavlc_pkg.vhd
xvhdl --2008 src/vhdl/transform_engine.vhd
xvhdl --2008 src/vhdl/quant_engine.vhd
xvhdl --2008 src/vhdl/predict_4x4_engine.vhd
xvhdl --2008 src/vhdl/predict_16x16_engine.vhd
xvhdl --2008 src/vhdl/predict_chroma_engine.vhd
xvhdl --2008 src/vhdl/cavlc_cost_engine_ll.vhd
xvhdl --2008 src/vhdl/recon_engine.vhd
xvhdl --2008 src/vhdl/mode_decide_engine.vhd
xvhdl --2008 src/vhdl/mode_decide_engine_tb.vhd
exec xelab mode_decide_engine_tb -snapshot mode_decide_engine_snap -debug typical
exec xsim mode_decide_engine_snap -runall
