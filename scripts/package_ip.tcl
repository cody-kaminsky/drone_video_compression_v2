# package_ip.tcl — package encoder_axi_top as a Vivado IP.
#
# The kernel is packaged rather than added to a block design as a module
# reference so that it carries an identity (vendor / library / name / version)
# and a fixed interface contract. Any host that knows the contract can drive
# any codec kernel that presents it; see docs/m5-hw-validation.md.
#
# Run from the project root:
#   vivado -mode batch -source scripts/package_ip.tcl [-tclargs <ip_root> [part]]
# Defaults: build/ip, xc7z020clg400-1.
#
# Produces <ip_root>/dcc_h264_enc/component.xml. Add <ip_root> to a project's
# ip_repo_paths to use it.

set ip_root [expr {[llength $argv] > 0 ? [lindex $argv 0] : "build/ip"}]
set part    [expr {[llength $argv] > 1 ? [lindex $argv 1] : "xc7z020clg400-1"}]

set VENDOR  "dcc"
set LIBRARY "codec"
set NAME    "dcc_h264_enc"
set VERSION "1.0"
set DISPLAY "DCC H.264 Intra Encoder"

set srcs {
    cavlc_pkg cavlc_tables cavlc_vlc_rom coeff_token_encoder bit_packer
    cavlc_engine cavlc_dispatch transform_engine quant_engine
    predict_4x4_engine predict_16x16_engine predict_chroma_engine
    cavlc_cost_engine_ll recon_engine mode_decide_engine line_buffer
    mb_header_engine mb_pipeline_controller frame_io encoder_top
    encoder_axi_top
}

set root [pwd]
set proj_dir [file join $root build ip_proj]
file delete -force $proj_dir
file mkdir $ip_root
set ip_dir [file join $root $ip_root $NAME]
file delete -force $ip_dir
file mkdir $ip_dir

create_project -force ip_proj $proj_dir -part $part
set_property target_language VHDL [current_project]
foreach s $srcs { read_vhdl -vhdl2008 [file join $root src vhdl $s.vhd] }
set_property top encoder_axi_top [current_fileset]
update_compile_order -fileset sources_1

# ---------------------------------------------------------------- package ---
ipx::package_project -root_dir $ip_dir -vendor $VENDOR -library $LIBRARY \
    -taxonomy /Video_and_Image_Processing -import_files -force
set core [ipx::current_core]

set_property name           $NAME     $core
set_property version        $VERSION  $core
set_property display_name   $DISPLAY  $core
set_property description    "H.264 baseline intra-only encoder kernel. AXI4-Lite control, AXI4-Stream NV12 samples in, AXI4-Stream slice payload out. The host emits SPS/PPS, the slice header and the NAL framing." $core
set_property vendor_display_name "drone_video_compression" $core
set_property company_url    "https://example.invalid/dcc" $core
set_property supported_families {zynq Production qzynq Production} $core

# ------------------------------------------------------------- interfaces ---
# The port names follow the AXI naming conventions, so inference finds
# s_axi / s_axis / m_axis on its own. Assert it rather than assume it: a
# missed inference shows up much later as an un-connectable block.
foreach bif {s_axi s_axis m_axis} {
    if {[llength [ipx::get_bus_interfaces $bif -of_objects $core]] == 0} {
        error "PACKAGE_FAIL: bus interface '$bif' was not inferred"
    }
}

# AXI4-Lite address space: 256 bytes is the whole register map.
set mm [ipx::get_memory_maps s_axi -of_objects $core]
if {[llength $mm] == 0} { set mm [ipx::add_memory_map s_axi $core] }
set_property slave_memory_map_ref s_axi [ipx::get_bus_interfaces s_axi -of_objects $core]
set blk [ipx::get_address_blocks reg0 -of_objects $mm]
if {[llength $blk] == 0} { set blk [ipx::add_address_block reg0 $mm] }
set_property base_address 0 $blk
set_property range 256     $blk
set_property width 32      $blk

# aclk drives every interface; without ASSOCIATED_BUSIF the block automation
# leaves the streams unclocked and the design silently builds wrong.
ipx::associate_bus_interfaces -busif s_axi  -clock aclk $core
ipx::associate_bus_interfaces -busif s_axis -clock aclk $core
ipx::associate_bus_interfaces -busif m_axis -clock aclk $core
set rstif [ipx::get_bus_interfaces aresetn -of_objects $core]
if {[llength $rstif] > 0} {
    set_property value ACTIVE_LOW [ipx::add_bus_parameter POLARITY $rstif]
}

# irq as a real interrupt so IPI connects it to the PS IRQ_F2P port.
if {[llength [ipx::get_bus_interfaces irq -of_objects $core]] == 0} {
    ipx::infer_bus_interface irq xilinx.com:signal:interrupt_rtl:1.0 $core
}
set_property value LEVEL_HIGH \
    [ipx::add_bus_parameter SENSITIVITY [ipx::get_bus_interfaces irq -of_objects $core]]

# --------------------------------------------------------------- generics ---
# MAX_W sizes the neighbour line buffers, so it is the frame-width ceiling and
# it costs BRAM. N_ENGINES is the CAVLC engine count. Both are build-time.
# Note: 'tooltip' is not a user_parameter property; it belongs to the GUI
# parameter, which only exists after create_xgui_files. Hence the two passes.
foreach {p disp} {
    MAX_W     "Max frame width"
    N_ENGINES "CAVLC engines"
} {
    set up [ipx::get_user_parameters $p -of_objects $core]
    if {[llength $up] > 0} { set_property display_name $disp $up }
}

ipx::create_xgui_files $core

foreach {p tip} {
    MAX_W     "Widest frame the line buffers can hold, in pixels. Multiple of 16."
    N_ENGINES "Parallel CAVLC engines. 2 is enough for 1080p30."
} {
    set gp [ipgui::get_guiparamspec -name $p -component $core -quiet]
    if {[llength $gp] > 0} { set_property tooltip $tip $gp }
}
ipx::update_checksums $core
ipx::check_integrity $core
ipx::save_core $core

close_project
puts "PACKAGE_DONE root=$ip_root core=${VENDOR}:${LIBRARY}:${NAME}:${VERSION}"
