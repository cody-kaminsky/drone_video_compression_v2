# run_board.tcl — program the PL, boot the PS, load the app and the test
# sequence, and run. The whole L5 flow in one command, with no IDE involved.
#
#   xsdb scripts/run_board.tcl [seq_dir [elf [bitstream [xsa]]]]
#
# Defaults to the 480x272 sequence, which is the one to try first.
# For 1080p:
#   xsdb scripts/run_board.tcl build/seq_board_1080
#
# Why this exists. Driving the board from the IDE means a run configuration
# whose "Program FPGA" setting decides whether the PL is reconfigured, and PL
# configuration survives an ELF download and a PS reset. Get it wrong and new
# software runs against a bitstream from hours ago, reporting nothing unusual.
# This script always programs, so the question cannot arise.
#
# Watch the UART at 115200 8N1 for the results.

proc arg {i default} {
    global argv
    if {[llength $argv] > $i} { return [lindex $argv $i] }
    return $default
}

# Absolute: xsdb does not start in the project root, so a relative default
# would resolve against wherever the shell happened to be.
set root "C:/Users/kamin/OneDrive/Documents/drone_video_compression_v2"
set seq  [arg 0 "$root/build/seq_board_480"]
set elf  [arg 1 "C:/Users/kamin/Vivado_Projects/zybo_encoder_test/dcc_14/build/dcc_14.elf"]
set bit  [arg 2 "C:/Users/kamin/Vivado_Projects/zybo_encoder_test/dcc_plat/export/dcc_plat/hw/dcc_enc.bit"]
set xsa  [arg 3 "C:/Users/kamin/Vivado_Projects/zybo_encoder_test/dcc_plat/hw/dcc_enc.xsa"]

foreach {what path} [list bitstream $bit ELF $elf "sequence dir" $seq] {
    if {![file exists $path]} {
        puts "ERROR: no $what at $path"
        exit 1
    }
}

proc stamp {path} {
    return "[file size $path] bytes, [clock format [file mtime $path] -format {%H:%M:%S}]"
}
puts "bitstream : $bit"
puts "            [stamp $bit]"
puts "ELF       : $elf"
puts "            [stamp $elf]"
puts "sequence  : $seq"

connect

# ---- configure the PL -------------------------------------------------------
targets -set -filter {name =~ "xc7z*"}
fpga -file $bit
puts "PL configured"

# ---- bring up the PS --------------------------------------------------------
# loadhw supplies ps7_init from the hardware handoff; without it DDR is not
# initialised and every load below would land in the void.
targets -set -filter {name =~ "ARM*#0"}
rst -processor
if {[file exists $xsa]} {
    loadhw -hw $xsa -mem-ranges [list {0x40000000 0xbfffffff}] -regs
}
catch { ps7_init }
catch { ps7_post_config }
puts "PS initialised"

# ---- application ------------------------------------------------------------
dow $elf
puts "ELF downloaded"

# ---- test data --------------------------------------------------------------
# Same addresses as host/dcc_memmap.h. The manifest goes last on purpose: its
# magic is what tells the application the rest of the data really arrived, so a
# load that dies halfway leaves it waiting rather than running on garbage.
source [file join $seq addrs.tcl]
puts "sequence loaded"

con
puts "running -- watch the UART at 115200"
