# run_predict_chroma_engine_tb.tcl — xsim driver for predict_chroma_engine_tb (run from project root,
# after `make predict_vectors`).
xvhdl --2008 src/vhdl/predict_chroma_engine.vhd
xvhdl --2008 src/vhdl/predict_chroma_engine_tb.vhd
exec xelab predict_chroma_engine_tb -snapshot predict_chroma_engine_snap -debug typical
exec xsim predict_chroma_engine_snap -runall
