# csynth.tcl — Vitis HLS synthesis driver for the drone H.264 encoder.
#
# Run from the repo root with:
#   vitis_hls -p src/hls/csynth.tcl
#
# Produces a project at hls_proj/ with one solution. After this finishes
# clean, drop into the GUI for waveform inspection:
#   vitis_hls hls_proj
#
# To export the IP for Vivado integration after synthesis closes timing:
#   vitis_hls -p src/hls/export.tcl   (separate script, not yet written)
#
# Target: Xilinx Zynq-7030 SBG485 speed grade -3, 200 MHz.
# (See architecture.txt §1 for budget and §10 for register-map plan.)

# === Project setup ===
set proj_dir   hls_proj
set sol_name   solution1
set top_func   encode_frame_h264_hls_top
set part       xc7z030sbg485-3
set clk_period 5            ;# ns -> 200 MHz

# Create or open the project.
if { [file isdirectory $proj_dir] } {
    puts "INFO: opening existing project $proj_dir"
    open_project $proj_dir
} else {
    puts "INFO: creating new project $proj_dir"
    open_project -reset $proj_dir
}

set_top $top_func

# === Source files ===
# Top-level synthesis kernel + line-buffered encoder.
add_files src/hls/hls_top.c       -cflags "-Isrc -Isrc/hls -std=c99"
add_files src/hls/encoder.c       -cflags "-Isrc -Isrc/hls -std=c99"
add_files src/hls/line_buffer.c   -cflags "-Isrc -Isrc/hls -std=c99"

# Shared kernel modules — the same code the C reference uses.
add_files src/transform.c -cflags "-Isrc -std=c99"
add_files src/quant.c     -cflags "-Isrc -std=c99"
add_files src/intra.c     -cflags "-Isrc -std=c99"
add_files src/cavlc.c     -cflags "-Isrc -std=c99"
add_files src/bitstream.c -cflags "-Isrc -std=c99"
add_files src/nal.c       -cflags "-Isrc -std=c99"
# psnr.c is host-side only; do not include in synthesis.

# === Test bench ===
# A dedicated cosim test bench lives at tests/tb_hls_kernel.c (TODO).
# Once that file exists, uncomment the next line and the cosim_design
# block at the bottom.
# add_files -tb tests/tb_hls_kernel.c -cflags "-Isrc -Isrc/hls -std=c99"

# === Solution + part + clock ===
open_solution -reset $sol_name
set_part $part
create_clock -period $clk_period -name default

# === External directives (per-function pragmas for shared modules) ===
source src/hls/directives.tcl

# === C synthesis ===
puts "INFO: running csynth_design ..."
csynth_design

puts "INFO: csynth complete. Reports under $proj_dir/$sol_name/syn/report/"
puts "INFO: top-level report: $proj_dir/$sol_name/syn/report/${top_func}_csynth.rpt"

# === C/RTL cosim (uncomment once the test bench is in place) ===
# puts "INFO: running cosim_design ..."
# cosim_design -trace_level all
