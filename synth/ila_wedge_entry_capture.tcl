# synth/ila_wedge_entry_capture.tcl — capture the retired-PC trace leading
# INTO the STM/CTE wedge on real hardware (task #98, 2026-07-05).
#
# Sim investigation (checksum bug retracted+fixed) proved the wedge is
# HW-only: a 20M-instruction deep bare-boot sim never reaches any of the
# known wedge PCs (0x4084a844/a966/afca/af9c/aa08), matching MAME cleanly
# the whole way. Real hardware DOES reach this wedge every time
# (exc_count~0x1322, confirmed via many halt-status reads this session).
# This capture triggers on rob_pc==0x4084a844 (one of the ring addresses)
# with a deep pre-trigger buffer, to see the retired-PC trajectory
# leading UP TO first entry into this region -- i.e. what real HW is
# doing right before it diverges from the now-validated-clean sim/MAME
# baseline.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_wedge_entry_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_wedge_entry_capture.tcl <ltx_file> <jcmd_path> \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set timeout_min [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 2.0}]

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
if {[llength $targets] == 0} {
    puts stderr "ERROR: no JTAG targets discovered."
    exit 2
}
current_hw_target [lindex $targets 0]
open_hw_target

set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx_file $dev
refresh_hw_device $dev -update_hw_probes true

set ilas [get_hw_ilas -of_objects $dev]
if {[llength $ilas] == 0} {
    puts stderr "ERROR: no hw_ila core found on device."
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p13 [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p13] == 0} {
    puts stderr "ERROR: expected probe dbg_ila_rob_pc_w not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

# Deep pre-trigger: capture the ~3900 retired-PCs BEFORE first entering
# the ring, not just the ring itself (which we've already seen many
# times).
set_property CONTROL.TRIGGER_POSITION 3900 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h4084a844 $p13
puts "=== Trigger: rob_pc==0x4084a844 (STM/CTE ring entry point) ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

after 300
puts "=== Firing reset via: $jcmd_path reset ==="
if {[catch {exec $jcmd_path reset} reset_out]} {
    puts "WARN: reset exec returned error/nonzero: $reset_out"
} else {
    puts "reset output:\n$reset_out"
}

puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: TRIGGERED ==="

catch {write_hw_ila_data -csv_file /tmp/ila_wedge_entry_capture.csv -force $data}
puts "=== Wrote /tmp/ila_wedge_entry_capture.csv (if supported) ==="

puts "=== DONE ==="
