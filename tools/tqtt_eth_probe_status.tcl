# Program the standalone TQTT GMII receive probe and sample it twice.
# Usage: vivado -nojournal -nolog -mode batch -source tools/tqtt_eth_probe_status.tcl \
#          -tclargs build/tqtt_eth_probe/tqtt_eth_probe.bit build/tqtt_eth_probe/tqtt_eth_probe.ltx
if {[llength $argv] != 2} { error "expected bitstream and LTX paths" }
set bit_file [file normalize [lindex $argv 0]]
set ltx_file [file normalize [lindex $argv 1]]
open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} { error "expected exactly one JTAG target, got: $targets" }
current_hw_target [lindex $targets 0]
open_hw_target
set devices [get_hw_devices]
if {[llength $devices] != 1} { error "expected exactly one FPGA, got: $devices" }
set dev [lindex $devices 0]
set_property PROGRAM.FILE $bit_file $dev
set_property PROBES.FILE $ltx_file $dev
set_property FULL_PROBES.FILE $ltx_file $dev
if {![info exists ::env(TQTT_ETH_NO_PROGRAM)] || $::env(TQTT_ETH_NO_PROGRAM) ne "1"} {
    program_hw_devices $dev
}
refresh_hw_device -update_hw_probes true $dev
set vios [get_hw_vios -quiet]
if {[llength $vios] != 1} { error "expected one VIO, got: $vios" }
set vio [lindex $vios 0]

proc gray_to_binary {value} {
    set result $value
    foreach shift {1 2 4 8 16} {
        set result [expr {$result ^ ($result >> $shift)}]
    }
    return [expr {$result & 0xffffffff}]
}

proc sample_counts {vio} {
    refresh_hw_vio $vio
    set values [dict create]
    foreach probe [get_hw_probes -of_objects $vio] {
        set name [get_property NAME $probe]
        scan [get_property INPUT_VALUE $probe] %x gray
        dict set values $name [gray_to_binary $gray]
    }
    return $values
}

after 1000
set first [sample_counts $vio]
set first_ms [clock milliseconds]
after 2000
set second [sample_counts $vio]
set second_ms [clock milliseconds]
puts "ELAPSED_MS [expr {$second_ms - $first_ms}]"
puts "FIRST  $first"
puts "SECOND $second"
foreach name [dict keys $first] {
    if {[dict exists $second $name]} {
        set delta [expr {([dict get $second $name] - [dict get $first $name]) & 0xffffffff}]
        puts "DELTA $name $delta"
    }
}
