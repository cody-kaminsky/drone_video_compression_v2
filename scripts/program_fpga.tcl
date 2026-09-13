# program_fpga.tcl — configure the PL explicitly, bypassing Vitis.
#
#   xsdb scripts/program_fpga.tcl [path/to.bit]
#
# Why this exists. PL configuration survives an ELF download and a PS reset,
# so a run configuration with "Program FPGA" unticked happily runs new software
# against whatever bitstream was loaded hours ago. Nothing reports this: the
# registers read back fine, the kernel encodes, and only the behaviour is old.
# Programming explicitly removes the question.
#
# Run this, then launch from Vitis with "Program FPGA" OFF.

set bit [expr {[llength $argv] > 0 ? [lindex $argv 0] : \
    "build/zybo/dcc_enc_proj/dcc_enc_proj.runs/impl_1/dcc_enc_wrapper.bit"}]

if {![file exists $bit]} {
    puts "ERROR: no bitstream at $bit"
    exit 1
}
puts "bitstream: $bit"
puts "  [file size $bit] bytes, modified [clock format [file mtime $bit] -format {%Y-%m-%d %H:%M:%S}]"

connect
targets -set -filter {name =~ "xc7z*"}
fpga -file $bit
puts "PL configured"

# Release the PS side and leave the core halted so a sequence can be loaded.
targets -set -filter {name =~ "ARM*#0"}
rst -processor
puts "core 0 reset and halted -- launch the ELF now with Program FPGA OFF"
