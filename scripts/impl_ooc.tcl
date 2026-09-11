# impl_ooc.tcl — full out-of-context implementation of encoder_axi_top:
# synthesis, opt, place, phys_opt, route, timing + utilization reports and
# a checkpoint. Run from the project root:
#   vivado -mode batch -source scripts/impl_ooc.tcl [-tclargs <outdir> [MAX_W N_ENGINES [part [period_ns]]]]
# Defaults: build/impl, MAX_W=1920, N_ENGINES=2, xc7z030sbg485-3, 5.000 ns.
set outd   [expr {[llength $argv] > 0 ? [lindex $argv 0] : "build/impl"}]
set max_w  [expr {[llength $argv] > 1 ? [lindex $argv 1] : 1920}]
set n_eng  [expr {[llength $argv] > 2 ? [lindex $argv 2] : 2}]
set part   [expr {[llength $argv] > 3 ? [lindex $argv 3] : "xc7z030sbg485-3"}]
set period [expr {[llength $argv] > 4 ? [lindex $argv 4] : 5.000}]
file mkdir $outd

set srcs {
    cavlc_pkg cavlc_tables cavlc_vlc_rom coeff_token_encoder bit_packer cavlc_engine cavlc_dispatch
    transform_engine quant_engine predict_4x4_engine predict_16x16_engine predict_chroma_engine
    cavlc_cost_engine_ll recon_engine mode_decide_engine line_buffer mb_header_engine
    mb_pipeline_controller frame_io encoder_top encoder_axi_top
}
create_project -in_memory -part $part
foreach s $srcs { read_vhdl -vhdl2008 src/vhdl/$s.vhd }
# the checked-in XDC is the 200 MHz one; a different period is written here
if {$period == 5.000} {
    read_xdc -mode out_of_context constraints/encoder_axi_top_ooc.xdc
} else {
    set fh [open "$outd/ooc.xdc" w]
    puts $fh "create_clock -period $period -name aclk \[get_ports aclk\]"
    puts $fh {set_input_delay  -clock aclk 1.000 [get_ports -filter {DIRECTION == IN  && NAME !~ "aclk"}]}
    puts $fh {set_output_delay -clock aclk 1.000 [get_ports -filter {DIRECTION == OUT}]}
    puts $fh {set_false_path -from [get_ports aresetn]}
    close $fh
    read_xdc -mode out_of_context $outd/ooc.xdc
}

synth_design -top encoder_axi_top -part $part -mode out_of_context \
    -generic MAX_W=$max_w -generic N_ENGINES=$n_eng
report_utilization -file $outd/post_synth_util.rpt
report_timing_summary -max_paths 5 -file $outd/post_synth_timing.rpt
write_checkpoint -force $outd/post_synth.dcp

opt_design
place_design
phys_opt_design
report_timing_summary -max_paths 5 -file $outd/post_place_timing.rpt
route_design
phys_opt_design
report_timing_summary -max_paths 10 -file $outd/post_route_timing.rpt
report_utilization -file $outd/post_route_util.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file $outd/post_route_util_hier.rpt
report_power -file $outd/post_route_power.rpt
write_checkpoint -force $outd/post_route.dcp

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "IMPL_DONE part=$part period=$period WNS=$wns WHS=$whs"
