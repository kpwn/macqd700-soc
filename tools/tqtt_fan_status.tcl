# Read die temperature and, when the new bitstream is loaded, fan tach RPM.
# Usage: vivado -nojournal -nolog -mode batch -source tools/tqtt_fan_status.tcl \
#          -tclargs <matching fpga_top.ltx>
set ltx_file [file normalize [lindex $argv 0]]
if {![file exists $ltx_file]} {error "Missing probes file: $ltx_file"}
open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} {error "Expected one JTAG target, got: $targets"}
current_hw_target [lindex $targets 0]
open_hw_target
set devices [get_hw_devices]
if {[llength $devices] != 1} {error "Expected one FPGA, got: $devices"}
set dev [lindex $devices 0]
current_hw_device $dev
set_property PROBES.FILE $ltx_file $dev
set_property FULL_PROBES.FILE $ltx_file $dev
refresh_hw_device $dev
foreach mon [get_hw_sysmons -of_objects $dev -quiet] {
    refresh_hw_sysmon $mon
    puts "TQTT_DIE_TEMP_C=[get_property TEMPERATURE $mon]"
}
foreach vio [get_hw_vios -of_objects $dev -quiet] {
    refresh_hw_vio $vio
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[string match *vio_fan_rpm* [get_property NAME $p]]} {
            refresh_hw_vio $vio
            set raw_rpm [get_property INPUT_VALUE $p]
            scan $raw_rpm %x rpm
            puts "TQTT_FAN_RPM=$rpm"
        }
        if {[string match *vio_fan_duty* [get_property NAME $p]]} {
            puts "TQTT_FAN_DUTY_RAW=[get_property OUTPUT_VALUE $p]"
        }
    }
}
