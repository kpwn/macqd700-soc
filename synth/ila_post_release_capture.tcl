# synth/ila_post_release_capture.tcl — capture whatever the ROB head PC
# does the moment it moves away from a known halt PC, for the 2026-08-27
# engineered-T2-already-pending experiment
# (docs/BUG_calibration_word_misplaced_0d00.md Part 10). Two prior
# attempts showed: (1) a PC-entry trigger on the real loop's own address
# (0x40800888) never fired within 3 real minutes after release, and (2) a
# live status snapshot right after release showed pc_live=0x0000a8c0, an
# unexplained low-memory address -- neither the temp ISR (0x40800866) nor
# the permanent ISR (0x40847d06). This capture triggers the INSTANT the
# ROB-head PC changes away from the known halt PC (passed as an argument),
# with a small pretrigger, to see the actual PC sequence immediately
# following release -- does it enter an exception vector fetch, redirect
# somewhere unexpected, or something else entirely.
#
# PROBE MAPPING: see ila_rob_irq_recognition_capture.tcl's header.
#   dbg_ila_rob_pc_w      <- RobPlugin's ROB-head PC (p0.pc)  (THIS SCRIPT'S TRIGGER)
#   ila_rob_pop_bundle[2] <- RobPlugin.flushing
#   ila_cdb2_pack[8]      <- RobPlugin.interruptPending
#   ila_cdb0_pack[8]      <- RobPlugin.branchRedirect
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_post_release_capture.tcl \
#       -tclargs <ltx_file> <halt_pc_hex_no_0x> [timeout_min]
#
# Does NOT reset or halt the board itself -- arm this while the CPU is
# ALREADY halted at halt_pc (e.g. via reset-and-break-pc), then release
# separately via the jtag_repl.tcl FIFO.

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_post_release_capture.tcl <ltx_file> <halt_pc_hex_no_0x> \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set halt_pc     [lindex $argv 1]
set timeout_min [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 3.0}]

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

set p_trig [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p_trig] == 0} {
    puts stderr "ERROR: expected trigger probe (dbg_ila_rob_pc_w) not found."
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 100 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE neq32'h$halt_pc $p_trig
puts "=== Trigger: dbg_ila_rob_pc_w != 0x$halt_pc, position=100/4096 ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="
puts "=== Waiting up to ${timeout_min} min for trigger (release the halt now) ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) -- PC never left 0x$halt_pc ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== RESULT: ATTEMPTING_DUMP ==="
catch {write_hw_ila_data -csv_file /tmp/ila_post_release_capture.csv -force $data}
puts "=== Wrote /tmp/ila_post_release_capture.csv (if supported) ==="
puts "=== DONE ==="
