# run_encoder_axi_top_rc_tb.tcl — per-MB rate control through the AXI wrapper
# (run from the project root, after `make rc_vectors` and a copy of
# build/rc_hw/config.txt or build/rc_off/config.txt to build/rc_tb/config.txt):
# registers and RC registers over AXI4-Lite per frame, pixels / payload over
# AXI4-Stream, three frames back to back, byte-exact against the C reference's
# hardware model (--rc-hw). The RCSTAT lines give cycles per MB per frame and
# the RCSTALL lines the cycles spent waiting for a MB QP.
foreach f {cavlc_pkg cavlc_tables cavlc_vlc_rom coeff_token_encoder bit_packer cavlc_engine
           cavlc_dispatch transform_engine quant_engine predict_4x4_engine predict_16x16_engine
           predict_chroma_engine cavlc_cost_engine_ll recon_engine mode_decide_engine
           line_buffer mb_header_engine rc_mb_engine mb_pipeline_controller frame_io encoder_top
           encoder_axi_top encoder_axi_top_rc_tb} {
    xvhdl --2008 src/vhdl/$f.vhd
}
exec xelab encoder_axi_top_rc_tb -snapshot encoder_axi_top_rc_snap -debug typical
exec xsim encoder_axi_top_rc_snap -runall
