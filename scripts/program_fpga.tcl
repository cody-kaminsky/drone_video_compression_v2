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

# Candidates in preference order. The Vitis platform's exported copy comes
# first on purpose: the Vivado run directory is recreated by every rebuild, so
# a bitstream inside it disappears the moment another build starts, while the
# platform's copy is stable and is what the board was actually programmed with.
set candidates {
    "C:/Users/kamin/Vivado_Projects/zybo_encoder_test/dcc_plat/export/dcc_plat/hw/dcc_enc.bit"
    "build/zybo/dcc_enc_proj/dcc_enc_proj.runs/impl_1/dcc_enc_wrapper.bit"
}

set bit ""
if {[llength $argv] > 0} {
    set bit [lindex $argv 0]
    if {![file exists $bit]} {
        puts "ERROR: no bitstream at $bit"
        exit 1
    }
} else {
    foreach c $candidates {
        if {[file exists $c]} { set bit $c; break }
    }
    if {$bit eq ""} {
        puts "ERROR: no bitstream found. Tried:"
        foreach c $candidates { puts "  $c" }
        puts "Pass one explicitly: xsdb scripts/program_fpga.tcl <path.bit>"
        exit 1
    }
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
