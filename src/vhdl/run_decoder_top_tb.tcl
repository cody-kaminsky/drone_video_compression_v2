# run_decoder_top_tb.tcl
#
# Vivado xsim driver for the whole decoder: decode a real frame and compare
# every reconstructed sample against the C decoder.
#
# Usage (from project root -- the testbench opens the vector file by a path
# relative to the working directory):
#   foreach f {cavlc_pkg cavlc_dec_tables bit_reader cavlc_dec_engine
#              mb_residual_dec_engine mb_header_dec_engine transform_engine
#              quant_engine recon_engine predict_4x4_engine
#              predict_16x16_engine predict_chroma_engine line_buffer
#              mb_recon_dec_engine decoder_top decoder_top_tb} {
#       xvhdl --2008 src/vhdl/$f.vhd
#   }
#   xelab work.decoder_top_tb -snapshot dtop_tb -debug off
#   xsim dtop_tb -runall
#
# Prerequisite: vectors are generated.
#   make decoder_frame_vectors            (QP 26)
#   make DFV_QP=14 decoder_frame_vectors  (or any other QP)
#
# The payload is the encoder's own DCC_DUMP_SLICE output and the expectation
# is the C decoder's reconstruction of that same stream, which dec_test shows
# byte-exact against both the encoder and ffmpeg. The comparison is per 4x4
# block and stops at the first difference: in an intra frame one wrong sample
# propagates into everything that predicts from it, so the last mismatch
# describes the damage and the first one names the cause.
