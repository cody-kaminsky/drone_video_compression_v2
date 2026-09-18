# run_mb_pipeline_controller_tb.tcl — full-frame integration test (run from project
# root, after `make pipeline_vectors`).
foreach f {cavlc_pkg cavlc_tables cavlc_vlc_rom coeff_token_encoder bit_packer cavlc_engine
           cavlc_dispatch transform_engine quant_engine predict_4x4_engine predict_16x16_engine
           predict_chroma_engine cavlc_cost_engine_ll recon_engine mode_decide_engine
           line_buffer mb_header_engine rc_mb_engine mb_pipeline_controller mb_pipeline_controller_tb} {
    xvhdl --2008 src/vhdl/$f.vhd
}
exec xelab mb_pipeline_controller_tb -snapshot mb_pipeline_snap -debug typical
exec xsim mb_pipeline_snap -runall
