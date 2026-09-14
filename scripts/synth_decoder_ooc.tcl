# synth_decoder_ooc.tcl — out-of-context synthesis of decoder_top, for an
# area and timing number on the same part the encoder was measured on.
# Run from the project root:
#   vivado -mode batch -source scripts/synth_decoder_ooc.tcl \
#          -tclargs [<outdir> [<part> [<period_ns> [MAX_MB_COLS]]]]
# Defaults: build/dec_synth, xc7z020clg400-1, 9.091 ns (110 MHz), 120 cols.
#
# Synthesis only. That is enough to answer "does it fit alongside the
# encoder", which is the question; a routed number needs a real top with the
# AXI plumbing, and there is no decode-side AXI wrapper yet.
set outd   [expr {[llength $argv] > 0 ? [lindex $argv 0] : "build/dec_synth"}]
set part   [expr {[llength $argv] > 1 ? [lindex $argv 1] : "xc7z020clg400-1"}]
set period [expr {[llength $argv] > 2 ? [lindex $argv 2] : 9.091}]
set cols   [expr {[llength $argv] > 3 ? [lindex $argv 3] : 120}]
file mkdir $outd

set srcs {
    cavlc_pkg cavlc_dec_tables bit_reader cavlc_dec_engine
    mb_header_dec_engine mb_residual_dec_engine
    transform_engine quant_engine recon_engine
    predict_4x4_engine predict_16x16_engine predict_chroma_engine
    line_buffer mb_recon_dec_engine decoder_top
}
create_project -in_memory -part $part
foreach s $srcs { read_vhdl -vhdl2008 src/vhdl/$s.vhd }

set fh [open "$outd/ooc.xdc" w]
puts $fh "create_clock -period $period -name clk \[get_ports clk\]"
puts $fh {set_input_delay  -clock clk 1.000 [get_ports -filter {DIRECTION == IN  && NAME !~ "clk"}]}
puts $fh {set_output_delay -clock clk 1.000 [get_ports -filter {DIRECTION == OUT}]}
puts $fh {set_false_path -from [get_ports rst_n]}
close $fh
read_xdc -mode out_of_context $outd/ooc.xdc

synth_design -top decoder_top -part $part -mode out_of_context \
    -generic MAX_MB_COLS=$cols
report_utilization -file $outd/post_synth_util.rpt
report_utilization -hierarchical -file $outd/post_synth_util_hier.rpt
report_timing_summary -max_paths 5 -file $outd/post_synth_timing.rpt
write_checkpoint -force $outd/post_synth.dcp
puts "DECODER SYNTH DONE -> $outd"
