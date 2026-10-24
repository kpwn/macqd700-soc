# Standalone, TQTT GMII transmit/receive physical-link probe.
# Usage: vivado -mode batch -source synth/vivado_tqtt_eth_tx_probe.tcl -tclargs OUTPUT_DIR
set output_dir [file normalize [lindex $argv 0]]
set proj_root [file normalize [file dirname [info script]]/..]
set part xcku5p-ffvb676-2-i
if {[lindex $argv 0] eq ""} { error "output directory required" }
file mkdir $output_dir
file mkdir $output_dir/ip
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints
set_param general.maxThreads 8

file delete -force $output_dir/ip/tqtt_eth_tx_vio
foreach old_ip [glob -nocomplain $output_dir/ip/tqtt_eth_tx_vio_*] {
    file delete -force $old_ip
}
create_project -in_memory -part $part -force
create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name tqtt_eth_tx_vio -dir $output_dir/ip
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN {4} \
    CONFIG.C_NUM_PROBE_OUT {0} \
    CONFIG.C_PROBE_IN0_WIDTH {32} \
    CONFIG.C_PROBE_IN1_WIDTH {32} \
    CONFIG.C_PROBE_IN2_WIDTH {32} \
    CONFIG.C_PROBE_IN3_WIDTH {32} \
] [get_ips tqtt_eth_tx_vio]
generate_target {synthesis} [get_ips tqtt_eth_tx_vio]
synth_ip [get_ips tqtt_eth_tx_vio]
close_project

read_verilog $proj_root/rtl/soc/tqtt_eth_tx_probe.v
read_ip $output_dir/ip/tqtt_eth_tx_vio/tqtt_eth_tx_vio.xci
read_xdc $proj_root/synth/tqtt_eth_probe.xdc
synth_design -top tqtt_eth_tx_probe -part $part
# Only the first stages cross between the 125 MHz TX domain and 100 MHz
# JTAG/status domain. Keep each synchronizer's second stage timed normally.
set gray_meta [get_cells -hier -filter {NAME =~ *tx_gray_meta_reg*}]
set ready_meta [get_cells -hier -regexp {.*phy_ready_tx_reg\[0\]$}]
if {[llength $gray_meta] < 1 || [llength $ready_meta] != 1} {
    error "TX/status CDC first stages not found: gray=$gray_meta ready=$ready_meta"
}
set_false_path -to [get_pins -of_objects $gray_meta -filter {REF_PIN_NAME == D}]
set_false_path -to [get_pins -of_objects $ready_meta -filter {REF_PIN_NAME == D}]
create_generated_clock -name tqtt_gmii_txclk -source [get_pins u_gtx_clock_fwd/C] \
    -divide_by 1 [get_ports eth_gtx_clk]
set_output_delay -clock tqtt_gmii_txclk -max 2.000 \
    [get_ports {eth_txd[*] eth_txen eth_txer}]
set_output_delay -clock tqtt_gmii_txclk -min -0.500 \
    [get_ports {eth_txd[*] eth_txen eth_txer}]
write_checkpoint -force $output_dir/checkpoints/synth.dcp
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
if {[llength [get_timing_paths -setup -max_paths 1 -slack_lesser_than 0.0]] ||
    [llength [get_timing_paths -hold -max_paths 1 -slack_lesser_than 0.0]]} {
    error "TQTT GMII TX timing failed; bitstream withheld"
}
write_checkpoint -force $output_dir/checkpoints/route.dcp
write_bitstream -force $output_dir/tqtt_eth_tx_probe.bit
write_debug_probes -force $output_dir/tqtt_eth_tx_probe.ltx
puts "TQTT Ethernet TX probe complete: $output_dir/tqtt_eth_tx_probe.bit"
