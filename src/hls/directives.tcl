# directives.tcl — externalized HLS pragmas for the encoder kernel.
#
# Sourced by csynth.tcl after open_solution. Pragmas applied here go to
# functions in the SHARED modules (src/transform.c, src/quant.c, etc.)
# so we don't have to add HLS-specific pragmas inline (the shared code
# is also used by the C reference build).
#
# Pragmas that ARE inline in src/hls/encoder.c and src/hls/line_buffer.c
# are gated by the HLS_PRAGMA() macro and don't need to be repeated here.

# === transform.c ===
# 4x4 forward and inverse DCT. Function-level pipelining at II=1.
set_directive_pipeline -II 1 dct4x4
set_directive_pipeline -II 1 idct4x4
set_directive_array_partition -dim 1 -type complete dct4x4 in
set_directive_array_partition -dim 1 -type complete dct4x4 out
set_directive_array_partition -dim 1 -type complete idct4x4 in
set_directive_array_partition -dim 1 -type complete idct4x4 out

set_directive_pipeline -II 1 hadamard4x4
set_directive_pipeline -II 1 hadamard2x2
set_directive_pipeline -II 1 ihadamard4x4
set_directive_pipeline -II 1 ihadamard2x2

# === quant.c ===
set_directive_pipeline -II 1 quant_4x4
set_directive_pipeline -II 1 iquant_4x4
set_directive_pipeline -II 1 quant_dc_4x4
set_directive_pipeline -II 1 iquant_dc_4x4
set_directive_pipeline -II 1 quant_dc_2x2
set_directive_pipeline -II 1 iquant_dc_2x2
set_directive_array_partition -dim 1 -type complete quant_4x4 in
set_directive_array_partition -dim 1 -type complete quant_4x4 out
set_directive_array_partition -dim 1 -type complete iquant_4x4 in
set_directive_array_partition -dim 1 -type complete iquant_4x4 out

# === intra.c ===
# Pipeline at function level — small predict kernels.
set_directive_pipeline -II 1 predict_4x4
set_directive_pipeline -II 1 predict_chroma_8x8
# predict_16x16 internally has 256-iter inner loops — leave function-level
# unset; iterate based on report. (Was causing memory-dependency II=2 in the
# first synth run.)

# === cavlc.c ===
# Bit-cost ESTIMATE only (used by mode decision). The actual variable-
# length CAVLC encoder is planned to be hand-VHDL in M4.
set_directive_pipeline -II 1 cavlc_estimate_block_bits

# === bitstream.c ===
# bs_byte_count and bs_put_bits run once per slice / per syntax element
# — NOT in inner loops. Don't pipeline (causes massive LUT explosion when
# inlined into nal_write_sps/pps).
# Leaving these unset on purpose — Vitis defaults are fine.

# === nal.c ===
# nal_write_sps and nal_write_pps run ONCE PER FRAME. Stay sequential.
set_directive_inline -off nal_write_sps
set_directive_inline -off nal_write_pps
set_directive_inline -off nal_emit_idr

# === line_buffer.c ===
set_directive_pipeline -II 1 lb_gather_luma_16x16
set_directive_pipeline -II 1 lb_gather_chroma_8x8
set_directive_pipeline -II 1 lb_gather_4x4
set_directive_pipeline -II 1 lb_commit_recon
set_directive_pipeline -II 1 lb_commit_nc

# === encoder.c — top-level guidance ===
set_directive_inline -off encode_mb_emit
set_directive_inline -off mb_fetch
set_directive_inline -off mb_residual
set_directive_inline -off mb_transform
set_directive_inline -off mb_quantize
set_directive_inline -off mb_reconstruct
set_directive_inline -off mb_compute_cbp
set_directive_inline -off mb_cavlc_emit
set_directive_inline -off mb_mode_decide
set_directive_inline -off try_path_i4x4
