# Standalone TQTT PMOD0 USB HID host probe.
# Usage: vivado -mode batch -source synth/vivado_tqtt_usb_host_probe.tcl -tclargs OUTPUT_DIR
if {[lindex $argv 0] eq ""} { error "output directory required" }
set output_dir [file normalize [lindex $argv 0]]
set proj_root [file normalize [file dirname [info script]]/..]
set part xcku5p-ffvb676-2-i
file mkdir $output_dir
file mkdir $output_dir/ip
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints
set_param general.maxThreads 8

create_project -in_memory -part $part -force
create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name tqtt_usb_vio -dir $output_dir/ip
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN {5} \
    CONFIG.C_NUM_PROBE_OUT {0} \
    CONFIG.C_PROBE_IN0_WIDTH {32} \
    CONFIG.C_PROBE_IN1_WIDTH {64} \
    CONFIG.C_PROBE_IN2_WIDTH {32} \
    CONFIG.C_PROBE_IN3_WIDTH {32} \
    CONFIG.C_PROBE_IN4_WIDTH {64} \
] [get_ips tqtt_usb_vio]
generate_target {synthesis} [get_ips tqtt_usb_vio]
synth_ip [get_ips tqtt_usb_vio]
close_project

set host_dir $proj_root/rtl/third_party/m1nl_usb_hid_host
read_verilog $host_dir/usb_hid_host.v
read_verilog $host_dir/usb_hid_host_rom.v
read_verilog $proj_root/rtl/soc/tqtt_usb_host_probe.v
read_ip $output_dir/ip/tqtt_usb_vio/tqtt_usb_vio.xci
read_xdc $proj_root/synth/tqtt_usb_host_probe.xdc
synth_design -top tqtt_usb_host_probe -part $part
write_checkpoint -force $output_dir/checkpoints/synth.dcp
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -report_unconstrained \
    -file $output_dir/reports/timing_final.rpt
report_drc -file $output_dir/reports/drc.rpt
write_checkpoint -force $output_dir/checkpoints/route.dcp
write_bitstream -force $output_dir/tqtt_usb_host_probe.bit
write_debug_probes -force $output_dir/tqtt_usb_host_probe.ltx
puts "TQTT USB host probe complete: $output_dir/tqtt_usb_host_probe.bit"
