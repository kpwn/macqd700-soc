# Inspect the already-loaded native SD engine probe without resetting its
# controller, issuing SD commands, or reprogramming the FPGA.
# Usage: vivado -mode batch -nojournal -nolog -source \
#   tools/tqtt_sd_native_engine_status.tcl -tclargs <matching probe.ltx>
if {[llength $argv] != 1} {error "usage: tqtt_sd_native_engine_status.tcl <ltx>"}
set ltx [file normalize [lindex $argv 0]]
if {![file isfile $ltx]} {error "missing probes file: $ltx"}

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} {error "expected one JTAG target, got: $targets"}
current_hw_target [lindex $targets 0]
open_hw_target
set devices [get_hw_devices]
if {[llength $devices] != 1} {error "expected one FPGA, got: $devices"}
set dev [lindex $devices 0]
set_property PROBES.FILE $ltx $dev
refresh_hw_device -update_hw_probes true $dev
set vios [get_hw_vios -of_objects $dev -quiet]
if {[llength $vios] != 1} {error "expected one native SD VIO, got: $vios"}
set vio [lindex $vios 0]

proc read_probe {vio name} {
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq $name} {
            scan [get_property INPUT_VALUE $p] %x value
            return $value
        }
    }
    error "native SD probe $name missing; check the matching LTX"
}

# The first refresh after attaching can return an incoherent snapshot; retain
# several samples so an unstable debug path is visible rather than reporting
# one transient value as a card result.
set samples {}
for {set i 0} {$i < 5} {incr i} {
    refresh_hw_vio $vio
    set raw [read_probe $vio status]
    set sample_crc [read_probe $vio result_crc]
    set sample_capacity [read_probe $vio card_num_lbas]
    lappend samples [list $raw $sample_crc $sample_capacity]
    puts [format "NATIVE_SD_SAMPLE%d status=0x%08X crc=0x%08X capacity_lbas=%u" \
        $i $raw $sample_crc $sample_capacity]
    after 600
}
set status   [read_probe $vio status]
set capacity [read_probe $vio card_num_lbas]
set rca      [read_probe $vio rca]
set crc      [read_probe $vio result_crc]
set cycles   [read_probe $vio result_cycles]
puts [format "NATIVE_SD_STATUS raw=0x%08X ready=%d init_error=%d phase=%d high_speed=%d capacity_lbas=%u rca=0x%08X last_crc=0x%08X last_cycles=%u" \
    $status [expr {($status >> 3) & 1}] [expr {($status >> 4) & 1}] \
    [expr {($status >> 5) & 31}] [expr {($status >> 10) & 1}] \
    $capacity $rca $crc $cycles]
set stable 1
foreach sample $samples {
    if {$sample ne [list $status $crc $capacity]} {set stable 0}
}
puts "NATIVE_SD_STATUS_STABLE=$stable"
close_hw_target
disconnect_hw_server
close_hw_manager
