# Program the standalone TQTT DDR4 probe and read its VIO result.
# Usage: vivado -nojournal -nolog -mode batch -source tools/tqtt_ddr4_probe_status.tcl
#          -tclargs build/tqtt_ddr4_probe/tqtt_ddr4_probe.bit
#                   build/tqtt_ddr4_probe/tqtt_ddr4_probe.ltx
if {[llength $argv] != 2} { error "expected bitstream and LTX paths" }
set bit_file [file normalize [lindex $argv 0]]
set ltx_file [file normalize [lindex $argv 1]]
open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} { error "expected one JTAG target, got: $targets" }
current_hw_target [lindex $targets 0]
open_hw_target
set devices [get_hw_devices]
if {[llength $devices] != 1} { error "expected one FPGA, got: $devices" }
set dev [lindex $devices 0]
set_property PROGRAM.FILE $bit_file $dev
set_property PROBES.FILE $ltx_file $dev
set_property FULL_PROBES.FILE $ltx_file $dev
if {![info exists ::env(TQTT_DDR4_NO_PROGRAM)] || $::env(TQTT_DDR4_NO_PROGRAM) ne "1"} {
    program_hw_devices $dev
}
refresh_hw_device -update_hw_probes true $dev
set vios [get_hw_vios -quiet]
if {[llength $vios] != 1} { error "expected one VIO, got: $vios" }
set vio [lindex $vios 0]
after 3000
refresh_hw_vio $vio
set observed [dict create]
foreach probe [get_hw_probes -of_objects $vio] {
    set name [get_property NAME $probe]
    set value [get_property INPUT_VALUE $probe]
    puts "PROBE $name $value"
    if {$name in {cal_done phase verified error_code error_addr error_xor}} {
        scan $value %x decoded
        dict set observed $name $decoded
    }
}
foreach name {cal_done phase verified error_code error_addr error_xor} {
    if {![dict exists $observed $name]} { error "missing DDR4 status probe: $name" }
}
if {[dict get $observed cal_done] != 1 ||
    [dict get $observed phase] != 5 ||
    [dict get $observed verified] != 8192 ||
    [dict get $observed error_code] != 0} {
    error "DDR4 probe failed or incomplete: $observed"
}
puts "DDR4_RESULT PASS calibrated=1 verified=8192 error_code=0"
