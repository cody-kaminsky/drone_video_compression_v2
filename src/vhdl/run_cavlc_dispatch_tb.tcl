# run_cavlc_dispatch_tb.tcl — xsim driver for cavlc_dispatch_tb (run from project
# root, after `make dispatch_vectors`).
xvhdl --2008 src/vhdl/cavlc_pkg.vhd
xvhdl --2008 src/vhdl/cavlc_tables.vhd
xvhdl --2008 src/vhdl/cavlc_vlc_rom.vhd
xvhdl --2008 src/vhdl/coeff_token_encoder.vhd
xvhdl --2008 src/vhdl/bit_packer.vhd
xvhdl --2008 src/vhdl/cavlc_engine.vhd
xvhdl --2008 src/vhdl/cavlc_dispatch.vhd
xvhdl --2008 src/vhdl/cavlc_dispatch_tb.vhd
exec xelab cavlc_dispatch_tb -snapshot cavlc_dispatch_snap -debug typical
exec xsim cavlc_dispatch_snap -runall
