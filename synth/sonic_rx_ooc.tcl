# Compare packet-RAM timing changes without attributing an OOC result to the
# whole FPGA. Args: source .sv, output directory. Use identical constraints
# for baseline and candidate; full-board routed timing is still required.
if {[llength $argv] != 2} { error "usage: sonic_rx_ooc.tcl source.sv output_dir" }
set source_file [file normalize [lindex $argv 0]]
set out_dir [file normalize [lindex $argv 1]]
file mkdir $out_dir
set_param general.maxThreads 8
create_project -in_memory -part xcku5p-ffvb676-2-i
read_verilog -sv $source_file
# The board instantiates MAX_CHUNKS=32 (default), ACCEPT_ALL=0 explicitly.
synth_design -top q700_sonic_rx -mode out_of_context -part xcku5p-ffvb676-2-i \
    -generic MAX_CHUNKS=32 -generic ACCEPT_ALL=0
create_clock -period 5.000 -name core_clk [get_ports clk]
set_input_delay 1.000 -clock core_clk [get_ports -filter {DIRECTION == IN && NAME != clk}]
set_output_delay 1.000 -clock core_clk [get_ports -filter {DIRECTION == OUT}]
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 20 -file $out_dir/timing.rpt
report_utilization -file $out_dir/utilization.rpt
report_timing -to [get_pins -hier -filter {REF_PIN_NAME =~ DIN*}] \
    -max_paths 10 -file $out_dir/bram_input_timing.rpt
write_checkpoint -force $out_dir/routed.dcp
