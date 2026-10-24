# Standalone, receive-only TQTT GMII physical-link probe.
# Usage: vivado -mode batch -source synth/vivado_tqtt_eth_probe.tcl -tclargs OUTPUT_DIR
set output_dir [file normalize [lindex $argv 0]]
set proj_root [file normalize [file dirname [info script]]/..]
set part xcku5p-ffvb676-2-i
if {[lindex $argv 0] eq ""} { error "output directory required" }
file mkdir $output_dir
file mkdir $output_dir/ip
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints
set_param general.maxThreads 8

file delete -force $output_dir/ip/tqtt_eth_vio
foreach old_ip [glob -nocomplain $output_dir/ip/tqtt_eth_vio_*] {
    file delete -force $old_ip
}
create_project -in_memory -part $part -force
create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name tqtt_eth_vio -dir $output_dir/ip
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN {3} \
    CONFIG.C_NUM_PROBE_OUT {0} \
    CONFIG.C_PROBE_IN0_WIDTH {32} \
    CONFIG.C_PROBE_IN1_WIDTH {32} \
    CONFIG.C_PROBE_IN2_WIDTH {32} \
] [get_ips tqtt_eth_vio]
generate_target {synthesis} [get_ips tqtt_eth_vio]
synth_ip [get_ips tqtt_eth_vio]
close_project

read_verilog $proj_root/rtl/soc/tqtt_eth_probe.v
read_ip $output_dir/ip/tqtt_eth_vio/tqtt_eth_vio.xci
read_xdc $proj_root/synth/tqtt_eth_probe.xdc
synth_design -top tqtt_eth_probe -part $part
write_checkpoint -force $output_dir/checkpoints/synth.dcp
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
write_checkpoint -force $output_dir/checkpoints/route.dcp
write_bitstream -force $output_dir/tqtt_eth_probe.bit
write_debug_probes -force $output_dir/tqtt_eth_probe.ltx
puts "TQTT Ethernet probe complete: $output_dir/tqtt_eth_probe.bit"
