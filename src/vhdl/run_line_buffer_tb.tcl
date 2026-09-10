# run_line_buffer_tb.tcl — xsim driver for line_buffer_tb (run from project root,
# after `make line_buffer_vectors`).
xvhdl --2008 src/vhdl/line_buffer.vhd
xvhdl --2008 src/vhdl/line_buffer_tb.vhd
exec xelab line_buffer_tb -snapshot line_buffer_snap -debug typical
exec xsim line_buffer_snap -runall
