# encoder_axi_top, out-of-context implementation constraints.
# The module is meant to sit in a Zynq block design clocked by FCLK at
# 200 MHz; in the block design the PS clock constraint replaces this
# create_clock and the AXI interfaces are internal, so only this file's
# clock matters there.
create_clock -period 5.000 -name aclk [get_ports aclk]

# AXI ports: budget for the interconnect / DMA on the other side
set_input_delay  -clock aclk 1.000 [get_ports -filter {DIRECTION == IN  && NAME !~ "aclk"}]
set_output_delay -clock aclk 1.000 [get_ports -filter {DIRECTION == OUT}]
set_false_path -from [get_ports aresetn]
