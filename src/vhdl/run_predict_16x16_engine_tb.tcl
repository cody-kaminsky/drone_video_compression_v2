# run_predict_16x16_engine_tb.tcl — xsim driver for predict_16x16_engine_tb (run from project root,
# after `make predict_vectors`).
xvhdl --2008 src/vhdl/predict_16x16_engine.vhd
xvhdl --2008 src/vhdl/predict_16x16_engine_tb.vhd
exec xelab predict_16x16_engine_tb -snapshot predict_16x16_engine_snap -debug typical
exec xsim predict_16x16_engine_snap -runall
