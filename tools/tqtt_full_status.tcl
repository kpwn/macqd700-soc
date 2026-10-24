# Read the running full Mac VIO without reprogramming or changing reset.
# Usage: vivado -nojournal -nolog -mode batch -source tools/tqtt_full_status.tcl \
#          -tclargs <fpga_top.ltx>
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

set vios [get_hw_vios -of_objects $dev -quiet]
if {[llength $vios] != 1} {error "Expected one VIO, got: $vios"}
set vio [lindex $vios 0]
refresh_hw_vio $vio
puts "TQTT_FULL_STATUS_BEGIN"
foreach p [get_hw_probes -of_objects $vio -quiet] {
    set name [get_property NAME $p]
    if {[catch {set value [get_property INPUT_VALUE $p]}]} {continue}
    if {$value ne ""} {puts "$name=$value"}
}
puts "TQTT_FULL_STATUS_END"
