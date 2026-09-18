# check_ip.tcl — prove the packaged IP is usable in IP Integrator.
#
# Vivado warns that packaging a component with a VHDL-2008 top file is "not
# fully supported". The entity's own boundary is plain std_logic and positive
# generics, so there is no 2008 construct at the port list, but the warning is
# not something to wave away: if IPI cannot elaborate the core, that is far
# better discovered here than halfway through a board design.
#
# So: drop the IP into a block design on its own, connect a clock and reset,
# validate, build a wrapper, and synthesize it out of context. If this passes,
# the packaging is good.
#
#   vivado -mode batch -source scripts/check_ip.tcl [-tclargs <ip_root> [part]]

set ip_root [expr {[llength $argv] > 0 ? [lindex $argv 0] : "build/ip"}]
set part    [expr {[llength $argv] > 1 ? [lindex $argv 1] : "xc7z020clg400-1"}]

set root [pwd]
set outd [file join $root build ip_check]
file delete -force $outd

create_project -force ip_check $outd -part $part
set_property ip_repo_paths [file join $root $ip_root] [current_project]
update_ip_catalog -rebuild

if {[llength [get_ipdefs -quiet dcc:codec:dcc_h264_enc:1.0]] == 0} {
    error "CHECK_FAIL: dcc:codec:dcc_h264_enc:1.0 not found in $ip_root"
}

create_bd_design check
set enc [create_bd_cell -type ip -vlnv dcc:codec:dcc_h264_enc:1.0 enc]

# Every interface must have come through as a real bus interface, not as a
# bag of loose pins. A missed inference is the actual failure mode here.
foreach bif {s_axi s_axis m_axis} {
    if {[llength [get_bd_intf_pins -quiet enc/$bif]] == 0} {
        error "CHECK_FAIL: enc/$bif is not a bus interface on the instantiated core"
    }
}
puts "CHECK: s_axi, s_axis, m_axis all present as bus interfaces"

# The generics must be settable from IPI, or the core is stuck at its defaults.
set_property -dict [list CONFIG.MAX_W {1920} CONFIG.N_ENGINES {2}] $enc
puts "CHECK: MAX_W and N_ENGINES are settable from IPI"

make_bd_intf_pins_external  [get_bd_intf_pins enc/s_axi]
make_bd_intf_pins_external  [get_bd_intf_pins enc/s_axis]
make_bd_intf_pins_external  [get_bd_intf_pins enc/m_axis]
make_bd_pins_external       [get_bd_pins enc/aclk]
make_bd_pins_external       [get_bd_pins enc/aresetn]
make_bd_pins_external       [get_bd_pins enc/irq]

validate_bd_design
save_bd_design
puts "CHECK: block design validates"

# Global synthesis, not the default per-IP out-of-context mode: we want the
# core's RTL elaborated here and now. In OOC mode the block design refers to a
# synthesis checkpoint that only launch_runs would produce, and synth_design
# then fails with "module not found" -- which looks like a packaging fault and
# is not one.
set_property synth_checkpoint_mode None [get_files check.bd]
generate_target all [get_files check.bd]

make_wrapper -files [get_files check.bd] -top -import
set_property top check_wrapper [current_fileset]

synth_design -top check_wrapper -part $part -mode out_of_context
report_utilization -file [file join $outd util.rpt]
puts "CHECK_DONE: the packaged IP elaborates and synthesizes from a block design"
