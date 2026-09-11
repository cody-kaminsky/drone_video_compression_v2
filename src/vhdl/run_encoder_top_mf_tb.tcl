# run_encoder_top_mf_tb.tcl — three 1080p frames back to back through the
# kernel (run from project root; data in build/mf/: stream<k>.txt from
# tools/gen_frame_stream.py, payload<k>.txt from DCC_DUMP_SLICE).
foreach f {cavlc_pkg cavlc_tables cavlc_vlc_rom coeff_token_encoder bit_packer cavlc_engine
           cavlc_dispatch transform_engine quant_engine predict_4x4_engine predict_16x16_engine
           predict_chroma_engine cavlc_cost_engine_ll recon_engine mode_decide_engine
           line_buffer mb_header_engine mb_pipeline_controller frame_io encoder_top
           encoder_top_mf_tb} {
    xvhdl --2008 src/vhdl/$f.vhd
}
exec xelab encoder_top_mf_tb -snapshot encoder_top_mf_snap -debug typical
exec xsim encoder_top_mf_snap -runall
