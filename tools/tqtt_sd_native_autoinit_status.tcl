# Load the read-only TQTT native-SD initialization probe and inspect its
# final command event. The card must have been physically power-cycled out of
# SPI mode before this runs. Reprograms FPGA SRAM only; never writes the card.
# Usage: vivado -nojournal -nolog -mode batch -source this_file -tclargs bit ltx
if {[llength $argv] != 2} {error "usage: tqtt_sd_native_autoinit_status.tcl <bit> <ltx>"}
if {![info exists ::env(NATIVE_SD_CARD_POWER_CYCLED)] ||
    $::env(NATIVE_SD_CARD_POWER_CYCLED) ne "1"} {
    error "power-cycle the SD card out of SPI mode first"
}
set bit [file normalize [lindex $argv 0]]
set ltx [file normalize [lindex $argv 1]]
foreach path [list $bit $ltx] {if {![file isfile $path]} {error "missing $path"}}

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} {error "expected one JTAG target, got $targets"}
current_hw_target [lindex $targets 0]
open_hw_target
set devices [get_hw_devices]
if {[llength $devices] != 1} {error "expected one FPGA, got $devices"}
set dev [lindex $devices 0]
set_property PROGRAM.FILE $bit $dev
program_hw_devices $dev
set_property PROBES.FILE $ltx $dev
refresh_hw_device -update_hw_probes true $dev
set vios [get_hw_vios -of_objects $dev -quiet]
if {[llength $vios] != 1} {error "expected one native-SD VIO, got $vios"}
set vio [lindex $vios 0]

proc probe_value {vio name} {
    foreach probe [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $probe] eq $name} {
            scan [get_property INPUT_VALUE $probe] %x value
            return $value
        }
    }
    error "probe $name missing from LTX"
}

# Native initialization normally finishes in well under a second. Sample
# until ready/error or ten seconds, retaining the final command status.
set deadline [expr {[clock milliseconds] + 10000}]
while {1} {
    refresh_hw_vio $vio
    set state [probe_value $vio vio_state]
    set phase [probe_value $vio vio_phase]
    set event [probe_value $vio vio_event]
    if {$state & 3 || [clock milliseconds] > $deadline} {break}
    after 20
}
set capacity [probe_value $vio card_num_lbas]
set rca [probe_value $vio rca]
puts [format "NATIVE_AUTOINIT phase=%u ready=%u error=%u event=0x%X timeout=%u capacity_lbas=%u rca=0x%08X" \
    $phase [expr {$state & 1}] [expr {($state >> 1) & 1}] \
    [expr {$event & 15}] [expr {($event >> 4) & 1}] $capacity $rca]
close_hw_target
disconnect_hw_server
close_hw_manager
if {!($state & 3)} {error "native SD initialization did not finish"}
