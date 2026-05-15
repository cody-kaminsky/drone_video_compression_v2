# run_bit_packer_tb.tcl
#
# Vivado xsim driver for bit_packer's testbench. Runs both the
# always-ready pass and a backpressure pass.
#
# Usage:
#   vivado -mode batch -source src/vhdl/run_bit_packer_tb.tcl
#
# Prerequisite: vectors are generated.
#   make bit_packer_vectors

create_project -in_memory -part xc7z030-sbg485-3

# Sources
add_files -norecurse src/vhdl/bit_packer.vhd
add_files -norecurse src/vhdl/bit_packer_tb.vhd

# VHDL-2008 for std.env.finish
set_property file_type {VHDL 2008} [get_files src/vhdl/bit_packer.vhd]
set_property file_type {VHDL 2008} [get_files src/vhdl/bit_packer_tb.vhd]

# Pass 1: always ready (validates correctness)
set_property top bit_packer_tb [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {0} -objects [get_filesets sim_1]
set_property -name {xsim.elaborate.xelab.more_options} \
    -value {-generic_top STALL_EVERY_N=0} \
    -objects [get_filesets sim_1]

puts "===== Pass 1: STALL_EVERY_N = 0 (always ready) ====="
launch_simulation
run all
close_sim

# Pass 2: backpressure every 5 cycles (exercises stall paths)
set_property -name {xsim.elaborate.xelab.more_options} \
    -value {-generic_top STALL_EVERY_N=5} \
    -objects [get_filesets sim_1]

puts "===== Pass 2: STALL_EVERY_N = 5 (periodic backpressure) ====="
launch_simulation
run all
close_sim

puts "All bit_packer simulation passes completed."
