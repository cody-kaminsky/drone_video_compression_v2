# run_encoder_axi_top_tb.tcl — AXI wrapper test (run from project root, after
# `make pipeline_vectors`): registers over AXI4-Lite, pixels / payload over
# AXI4-Stream, two frames back to back.
foreach f {cavlc_pkg cavlc_tables cavlc_vlc_rom coeff_token_encoder bit_packer cavlc_engine
           cavlc_dispatch transform_engine quant_engine predict_4x4_engine predict_16x16_engine
           predict_chroma_engine cavlc_cost_engine_ll recon_engine mode_decide_engine
           line_buffer mb_header_engine mb_pipeline_controller frame_io encoder_top
           encoder_axi_top encoder_axi_top_tb} {
    xvhdl --2008 src/vhdl/$f.vhd
}
exec xelab encoder_axi_top_tb -snapshot encoder_axi_top_snap -debug typical
exec xsim encoder_axi_top_snap -runall
