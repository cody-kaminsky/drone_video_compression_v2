# impl_decoder_ooc.tcl — place and route the decoder from its synthesis
# checkpoint, for a real timing number.
#
#   vivado -mode batch -source scripts/impl_decoder_ooc.tcl \
#          -tclargs [<dir with post_synth.dcp>]
#
# Defaults to build/dec_synth200.
#
# Worth running rather than reading the post-synthesis number: before
# placement Vivado estimates routing from fanout alone, and on this design
# that estimate is five times the logic delay. It is a bound, not a result.
# A second argument of "explore" raises placement and routing effort. Worth a
# couple of tenths of a nanosecond on a design whose remaining critical path
# is routing rather than logic.
set outd [expr {[llength $argv] > 0 ? [lindex $argv 0] : "build/dec_synth200"}]
set eff  [expr {[llength $argv] > 1 ? [lindex $argv 1] : "default"}]

open_checkpoint $outd/post_synth.dcp
if {$eff eq "explore"} {
    opt_design
    place_design -directive Explore
    phys_opt_design -directive AggressiveExplore
    route_design -directive Explore
    phys_opt_design -directive AggressiveExplore
} else {
    opt_design
    place_design
    phys_opt_design
    route_design
}
report_utilization -file $outd/post_route_util.rpt
report_utilization -hierarchical -file $outd/post_route_util_hier.rpt
report_timing_summary -max_paths 10 -file $outd/post_route_timing.rpt
write_checkpoint -force $outd/post_route.dcp
puts "DECODER IMPL DONE -> $outd"
