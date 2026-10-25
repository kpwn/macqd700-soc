# Load the read-only native SD block-engine probe into FPGA SRAM only.
# Program before reseating a card that was last used in SPI mode. This
# script does not reset the card, issue a block command, or write SPI flash.
#
# vivado -nojournal -nolog -mode batch \
#   -source tools/tqtt_sd_native_engine_load.tcl -tclargs <bit> <ltx>
if {[llength $argv] != 2} {
    error "usage: tqtt_sd_native_engine_load.tcl <bit> <ltx>"
}
set bit [file normalize [lindex $argv 0]]
set ltx [file normalize [lindex $argv 1]]
foreach path [list $bit $ltx] {
    if {![file isfile $path]} {error "missing $path"}
}

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} {error "expected one JTAG target"}
current_hw_target [lindex $targets 0]
open_hw_target
set devices [get_hw_devices]
if {[llength $devices] != 1} {error "expected one FPGA device"}
set dev [lindex $devices 0]
set_property PROGRAM.FILE $bit $dev
program_hw_devices $dev
set_property PROBES.FILE $ltx $dev
refresh_hw_device -update_hw_probes true $dev
set vios [get_hw_vios -quiet]
if {[llength $vios] != 1} {error "expected native probe VIO, found [llength $vios]"}
puts "NATIVE_ENGINE_PROBE_LOADED bit=$bit ltx=$ltx"
puts "Reseat the SD card now; after its power cycle, run tqtt_sd_native_engine_read.tcl with NATIVE_SD_CARD_POWER_CYCLED=1 and NATIVE_SD_NO_PROGRAM=1."
close_hw_target
disconnect_hw_server
close_hw_manager
