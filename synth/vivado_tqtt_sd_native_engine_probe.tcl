# Standalone TQTT native SD hardware-init and read-block probe.
set output_dir [lindex $argv 0]
set proj_root [file normalize [file dirname [info script]]/..]
set part xcku5p-ffvb676-2-i
set high_speed 0
if {[info exists ::env(TQTT_NATIVE_SD_HS)]} {
    set high_speed $::env(TQTT_NATIVE_SD_HS)
}
if {$high_speed ni {0 1}} {error "TQTT_NATIVE_SD_HS must be 0 or 1"}
set overclock_100 0
if {[info exists ::env(TQTT_NATIVE_SD_OVERCLOCK_100)]} {
    set overclock_100 $::env(TQTT_NATIVE_SD_OVERCLOCK_100)
}
if {$overclock_100 ni {0 1} || ($overclock_100 && !$high_speed)} {
    error "100 MHz experiment requires TQTT_NATIVE_SD_HS=1 and TQTT_NATIVE_SD_OVERCLOCK_100=1"
}
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
foreach source {
    sd_native_csr_master.v sd_native_command.v sd_native_init.v
    sd_native_init_controller.v sd_native_block_engine.v
    sd_native_hs_switch.v sd_native_backend.v
    tqtt_sd_native_engine_probe.v
} {
    read_verilog $proj_root/rtl/soc/$source
}
read_ip $ip_dir/native_sd_vio/native_sd_vio.xci
read_xdc $proj_root/synth/tqtt_sd_native_probe.xdc
set synth_args [list -top tqtt_sd_native_engine_probe -part $part]
if {$high_speed} {lappend synth_args -verilog_define TQTT_NATIVE_SD_HS}
if {$overclock_100} {lappend synth_args -verilog_define TQTT_NATIVE_SD_OVERCLOCK_100}
synth_design {*}$synth_args
write_checkpoint -force $output_dir/checkpoints/synth.dcp
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
write_checkpoint -force $output_dir/checkpoints/route.dcp
write_bitstream -force $output_dir/tqtt_sd_native_engine_probe.bit
write_debug_probes -force $output_dir/tqtt_sd_native_engine_probe.ltx
set f [open $output_dir/tqtt_sd_native_engine_probe.buildinfo w]
puts $f "part=$part"
puts $f "board=tqtt"
puts $f "top=tqtt_sd_native_engine_probe"
puts $f "core=LiteSDCard-2024.12"
puts $f "clock_hz=[expr {$overclock_100 ? 200000000 : 100000000}]"
puts $f "high_speed=$high_speed"
puts $f "experimental_sd100=$overclock_100"
puts $f "generated_utc=[clock format [clock seconds] -gmt 1 -format {%Y-%m-%dT%H:%M:%SZ}]"
close $f
