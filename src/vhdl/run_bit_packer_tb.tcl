# run_bit_packer_tb.tcl
#
# Standalone xsim driver for bit_packer's testbench. Runs both the
# always-ready pass and a backpressure pass.
# Uses xvhdl/xelab/xsim directly — no Vivado project needed.
#
# Usage (from Vivado Tcl console, cd to project root first):
#   cd C:/path/to/drone_video_compression_v2
#   source src/vhdl/run_bit_packer_tb.tcl
#
# Prerequisite: vectors are generated.
#   make bit_packer_vectors

xvhdl --2008 src/vhdl/bit_packer.vhd
xvhdl --2008 src/vhdl/bit_packer_tb.vhd

# Pass 1: always ready (validates correctness)
puts "===== Pass 1: STALL_EVERY_N = 0 (always ready) ====="
exec xelab bit_packer_tb -snapshot bp_test_p1 -debug typical \
    -generic_top STALL_EVERY_N=0
exec xsim bp_test_p1 -runall

# Pass 2: backpressure every 5 cycles (exercises stall paths)
puts "===== Pass 2: STALL_EVERY_N = 5 (periodic backpressure) ====="
exec xelab bit_packer_tb -snapshot bp_test_p2 -debug typical \
    -generic_top STALL_EVERY_N=5
exec xsim bp_test_p2 -runall

puts "All bit_packer simulation passes completed."
