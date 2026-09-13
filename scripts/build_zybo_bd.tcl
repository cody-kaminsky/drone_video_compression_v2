# build_zybo_bd.tcl — Zybo Z7-20 block design for encoder kernel bring-up.
#
#   PS7  --GP0--> AXI-Lite: encoder registers, DMA registers
#   PS7  <--HP0-- AXI DMA:  MM2S reads the frame, S2MM writes the payload
#   DMA  --MM2S--> encoder s_axis      encoder m_axis --> DMA S2MM
#   encoder irq --> PS IRQ_F2P[0]
#
# Run from the project root, after scripts/package_ip.tcl:
#   vivado -mode batch -source scripts/build_zybo_bd.tcl \
#          [-tclargs <outdir> [ip_root [clk_mhz]]]
# Defaults: build/zybo, build/ip, 110.
#
# Board files: this needs Digilent's Zybo Z7-20 board file for the PS7 DDR and
# MIO configuration. Those numbers are board-specific and getting them wrong
# does not fail the build -- it produces a bitstream that boots and then
# corrupts memory under load, which is a miserable thing to debug. So the
# script refuses to guess. Install them once, either through Vivado's
# Tools > XHub Store > Board Files, or by cloning
#   https://github.com/Digilent/vivado-boards
# and pointing BOARD_FILES at its new/board_files directory.

set outd    [expr {[llength $argv] > 0 ? [lindex $argv 0] : "build/zybo"}]
set ip_root [expr {[llength $argv] > 1 ? [lindex $argv 1] : "build/ip"}]
set clk_mhz [expr {[llength $argv] > 2 ? [lindex $argv 2] : 110}]

set BOARD_PART "digilentinc.com:zybo-z7-20:part0:1.0"
set PART       "xc7z020clg400-1"
set BD         "dcc_enc"

set root [pwd]
file mkdir $outd

# ------------------------------------------------------------ board check ---
if {[info exists ::env(BOARD_FILES)]} {
    set_param board.repoPaths $::env(BOARD_FILES)
}
if {[llength [get_board_parts -quiet $BOARD_PART]] == 0} {
    puts "ERROR: board part $BOARD_PART not found."
    puts "  The PS7 DDR and MIO settings for this board come from Digilent's"
    puts "  board files. Without them the design would build with a wrong"
    puts "  memory controller configuration and fail intermittently on"
    puts "  hardware rather than at build time."
    puts "  Install via Vivado Tools > XHub Store > Board Files, or:"
    puts "    git clone https://github.com/Digilent/vivado-boards"
    puts "    BOARD_FILES=<clone>/new/board_files vivado -mode batch -source $argv0"
    exit 1
}

create_project -force ${BD}_proj [file join $root $outd ${BD}_proj] -part $PART
set_property board_part $BOARD_PART [current_project]
set_property ip_repo_paths [file join $root $ip_root] [current_project]
update_ip_catalog -rebuild

create_bd_design $BD

# --------------------------------------------------------------------- PS ---
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 ps7]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps

set_property -dict [list \
    CONFIG.PCW_USE_S_AXI_HP0        {1} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
    CONFIG.PCW_IRQ_F2P_INTR         {1} \
    CONFIG.PCW_EN_CLK0_PORT         {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ $clk_mhz \
] $ps

# ---------------------------------------------------------------- encoder ---
set enc [create_bd_cell -type ip -vlnv dcc:codec:dcc_h264_enc:1.0 enc]
set_property -dict [list CONFIG.MAX_W {1920} CONFIG.N_ENGINES {2}] $enc

# -------------------------------------------------------------------- DMA ---
# One AXI DMA carries both directions. Two settings matter and both default
# wrong for this job:
#   - the buffer length register is 14 bits by default, so a transfer caps at
#     16 kB. A 1080p frame is 3.1 MB, so widen it to 26 bits (64 MB).
#   - scatter-gather off. Simple mode means one descriptor-free transfer per
#     call, which is what the bring-up path wants: the host stages the frame
#     in stream order and sends it as one contiguous block. Turn SG on later
#     to feed straight from the camera buffer with no staging copy.
set dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma dma]
set_property -dict [list \
    CONFIG.c_include_sg               {0} \
    CONFIG.c_sg_length_width          {26} \
    CONFIG.c_include_mm2s             {1} \
    CONFIG.c_include_s2mm             {1} \
    CONFIG.c_m_axi_mm2s_data_width    {64} \
    CONFIG.c_m_axis_mm2s_tdata_width  {32} \
    CONFIG.c_m_axi_s2mm_data_width    {64} \
    CONFIG.c_s_axis_s2mm_tdata_width  {32} \
    CONFIG.c_mm2s_burst_size          {128} \
    CONFIG.c_s2mm_burst_size          {128} \
] $dma

# ------------------------------------------------------------ connections ---
connect_bd_intf_net [get_bd_intf_pins dma/M_AXIS_MM2S] [get_bd_intf_pins enc/s_axis]
connect_bd_intf_net [get_bd_intf_pins enc/m_axis]      [get_bd_intf_pins dma/S_AXIS_S2MM]

# control plane: PS GP0 -> encoder registers and DMA registers
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config [list Master "/ps7/M_AXI_GP0" Clk "Auto"] [get_bd_intf_pins enc/s_axi]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config [list Master "/ps7/M_AXI_GP0" Clk "Auto"] [get_bd_intf_pins dma/S_AXI_LITE]

# data plane: DMA -> PS HP0
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config [list Master "/dma/M_AXI_MM2S" Clk "Auto"] [get_bd_intf_pins ps7/S_AXI_HP0]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config [list Master "/dma/M_AXI_S2MM" Clk "Auto"] [get_bd_intf_pins ps7/S_AXI_HP0]

# interrupts: the encoder's frame-done plus both DMA channels, so the host can
# wait on any of the three rather than polling.
set cc [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat irq_cat]
set_property CONFIG.NUM_PORTS {3} $cc
connect_bd_net [get_bd_pins enc/irq]              [get_bd_pins irq_cat/In0]
connect_bd_net [get_bd_pins dma/mm2s_introut]     [get_bd_pins irq_cat/In1]
connect_bd_net [get_bd_pins dma/s2mm_introut]     [get_bd_pins irq_cat/In2]
connect_bd_net [get_bd_pins irq_cat/dout]         [get_bd_pins ps7/IRQ_F2P]

assign_bd_address
regenerate_bd_layout
validate_bd_design
save_bd_design

# ------------------------------------------------------------------ build ---
make_wrapper -files [get_files ${BD}.bd] -top -import
set_property top ${BD}_wrapper [current_fileset]

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

set impl_dir [get_property DIRECTORY [get_runs impl_1]]
set wns [get_property STATS.WNS [get_runs impl_1]]
puts "BD_WNS=$wns"

# The XSA carries the bitstream and the hardware handoff that the bare-metal
# platform is generated from, including the register base addresses.
write_hw_platform -fixed -include_bit -force [file join $root $outd ${BD}.xsa]
puts "BD_DONE xsa=[file join $outd ${BD}.xsa] wns=$wns"
