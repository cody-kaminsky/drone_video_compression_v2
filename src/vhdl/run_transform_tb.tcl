# run_transform_tb.tcl
#
# Vivado xsim driver for transform_engine testbench.  Uses an in-memory
# project (no on-disk project directory) — just local RAM.
#
# Usage (from Vivado Tcl console, cd to project root first):
#   cd C:/path/to/drone_video_compression_v2
#   source src/vhdl/run_transform_tb.tcl
#
# Or from a shell:
#   vivado -mode batch -source src/vhdl/run_transform_tb.tcl
#
# Prerequisite: vectors must be generated first.
#   make transform_vectors

create_project -in_memory -part xc7z030-sbg485-3

# Sources
add_files -norecurse src/vhdl/transform_engine.vhd
add_files -norecurse src/vhdl/transform_engine_tb.vhd

# VHDL-2008 for std.env.finish and process(all)
set_property file_type {VHDL 2008} [get_files src/vhdl/transform_engine.vhd]
set_property file_type {VHDL 2008} [get_files src/vhdl/transform_engine_tb.vhd]

set_property top transform_engine_tb [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {0} -objects [get_filesets sim_1]

puts "===== Running transform_engine testbench ====="
launch_simulation
run all
close_sim

puts "Transform engine simulation completed."
