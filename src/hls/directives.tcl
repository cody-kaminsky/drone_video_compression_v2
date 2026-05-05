# directives.tcl — externalized HLS pragmas for the encoder kernel.
#
# Sourced by csynth.tcl after open_solution. Pragmas applied here go to
# functions in the SHARED modules (src/transform.c, src/quant.c, etc.)
# so we don't have to add HLS-specific pragmas inline (the shared code
# is also used by the C reference build).
#
# Pragmas that ARE inline in src/hls/encoder.c and src/hls/line_buffer.c
# are gated by the HLS_PRAGMA() macro and don't need to be repeated here.
#
# After the first csynth_design report, iterate this file based on:
#   - II reported per loop in <proj>/<sol>/syn/report/*_csynth.rpt
#   - any "unable to schedule" / "carried dependency" warnings
#   - resource estimate vs the budget in architecture.txt §1

# === transform.c ===
# 4x4 forward and inverse DCT. 16-element arrays, fully unroll the inner
# arithmetic; pipeline at the function level.
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
# Intra prediction is a small load-and-combine kernel; pipeline at fn level.
set_directive_pipeline -II 1 predict_4x4
set_directive_pipeline -II 1 predict_16x16
set_directive_pipeline -II 1 predict_chroma_8x8

# === cavlc.c ===
# We pipeline the bit-cost ESTIMATE used for mode decision (no variable-
# length output, just a counter). The actual cavlc_encode_block is
# expected to be replaced by a hand-VHDL CAVLC engine in M4; for the M3
# csynth pass it's still in HLS.
set_directive_pipeline -II 1 cavlc_estimate_block_bits

# === line_buffer.c ===
set_directive_pipeline -II 1 lb_gather_luma_16x16
set_directive_pipeline -II 1 lb_gather_chroma_8x8
set_directive_pipeline -II 1 lb_gather_4x4
set_directive_pipeline -II 1 lb_commit_mb

# === encoder.c — top-level guidance ===
# These complement the HLS_PRAGMA() macros already inside src/hls/encoder.c.
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
