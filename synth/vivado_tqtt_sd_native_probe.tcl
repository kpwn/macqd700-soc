# Small native four-bit SD controller probe with direct VIO CSR access.
set output_dir [lindex $argv 0]
set proj_root [file normalize [file dirname [info script]]/..]
set part xcku5p-ffvb676-2-i
set_param general.maxThreads 16
file mkdir $output_dir
file mkdir $output_dir/ip
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints

set ip_dir $output_dir/ip
file delete -force $ip_dir/native_sd_vio
foreach d [glob -nocomplain $ip_dir/native_sd_vio_*] {file delete -force $d}
create_project -in_memory -part $part -force
create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name native_sd_vio -dir $ip_dir
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN {5} \
    CONFIG.C_NUM_PROBE_OUT {4} \
    CONFIG.C_PROBE_IN0_WIDTH {32} \
    CONFIG.C_PROBE_IN1_WIDTH {32} \
    CONFIG.C_PROBE_IN2_WIDTH {32} \
    CONFIG.C_PROBE_IN3_WIDTH {32} \
    CONFIG.C_PROBE_IN4_WIDTH {32} \
    CONFIG.C_PROBE_OUT0_WIDTH {32} \
    CONFIG.C_PROBE_OUT1_WIDTH {32} \
    CONFIG.C_PROBE_OUT2_WIDTH {4} \
    CONFIG.C_PROBE_OUT3_WIDTH {12} \
    CONFIG.C_PROBE_OUT0_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT1_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT2_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT3_INIT_VAL {0x0} \
] [get_ips native_sd_vio]
generate_target {synthesis} [get_ips native_sd_vio]
synth_ip [get_ips native_sd_vio]
close_project

read_verilog $proj_root/third_party/litesdcard/litesdcard_core.v
read_verilog $proj_root/rtl/soc/tqtt_sd_native_probe.v
read_ip $ip_dir/native_sd_vio/native_sd_vio.xci
read_xdc $proj_root/synth/tqtt_sd_native_probe.xdc
synth_design -top tqtt_sd_native_probe -part $part
write_checkpoint -force $output_dir/checkpoints/synth.dcp
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
write_checkpoint -force $output_dir/checkpoints/route.dcp
write_bitstream -force $output_dir/tqtt_sd_native_probe.bit
write_debug_probes -force $output_dir/tqtt_sd_native_probe.ltx
set f [open $output_dir/tqtt_sd_native_probe.buildinfo w]
puts $f "part=$part"
puts $f "board=tqtt"
puts $f "top=tqtt_sd_native_probe"
puts $f "core=LiteSDCard-2024.12"
puts $f "clock_hz=100000000"
puts $f "generated_utc=[clock format [clock seconds] -gmt 1 -format {%Y-%m-%dT%H:%M:%SZ}]"
close $f
