# synth/ila_stuck_loop_capture.tcl — capture RobPlugin's interrupt-recognition
# internals for ~4000 cycles AFTER the CPU enters the known stuck calibration
# `dbf` loop, for the 2026-08-27 boot investigation (see
# docs/BUG_calibration_word_misplaced_0d00.md Parts 6-8).
#
# This is the companion/sibling of ila_rob_irq_recognition_capture.tcl, which
# triggers on interruptPending RISING (catches successful recognition
# elsewhere in boot -- confirmed working on the first live run of that
# script, PC=0x40847bfc, an already-known-good repeating VIA1 timer ISR
# call site, NOT the stuck loop). This script instead triggers on ROB-head
# PC entering the loop body (0x40800888, the shared `dbf D0,0x40800888`
# instruction -- see the doc's Part 6 disassembly) and captures mostly
# POST-trigger, so the buffer shows what normalIrqGate/flushing/excIdle/
# iplActive/interruptPending actually DO while the CPU is confirmed stuck
# retiring that one instruction over and over. If interruptPending never
# rises anywhere in the ~4000-cycle post-trigger window despite iplActive
# staying 1, that IS the bug, captured directly from the RTL's own signals
# rather than inferred from an external JTAG-halt side effect.
#
# PROBE MAPPING: see ila_rob_irq_recognition_capture.tcl's header for the
# full corrected mapping (fpga_top_debug_vio.vh packs several of these into
# wider vectors before handing them to the debug_ila IP):
#   ila_rob_pop_bundle[3] <- RobPlugin.normalIrqGate
#   ila_rob_pop_bundle[2] <- RobPlugin.flushing
#   dbg_ila_flush_en_w    <- RobPlugin.excIdle
#   ila_axi_w_snap[0]     <- RobPlugin.iplActive
#   ila_cdb0_pack[8]      <- RobPlugin.branchRedirect
#   ila_cdb0_pack[7]      <- RobPlugin.p0.first
#   ila_cdb1_pack[8]      <- RobPlugin.preciseDrainBusyIn
#   ila_cdb1_pack[7]      <- RobPlugin.inhibitedLoadBusyIn
#   ila_cdb2_pack[8]      <- RobPlugin.interruptPending
#   dbg_ila_rob_pc_w      <- RobPlugin's ROB-head PC (p0.pc)  (THIS SCRIPT'S TRIGGER)
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_stuck_loop_capture.tcl \
#       -tclargs <ltx_file> [timeout_min]
#
# Does NOT reset the board itself -- run this armed BEFORE or immediately
# after issuing a `reset` via the jtag_repl.tcl FIFO (`echo reset >
# /tmp/jtag_in`), since round 1 of the calibration loop is very early boot
# and the trigger needs to be armed before the CPU reaches 0x40800888.

if {[llength $argv] < 1} {
    puts stderr "ERROR: usage: ila_stuck_loop_capture.tcl <ltx_file> \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set timeout_min [expr {[llength $argv] >= 2 ? [lindex $argv 1] : 5.0}]

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
    puts stderr "ERROR: no hw_ila core found on device -- was ENABLE_ILA=1 CPU=m68k040 used, and is fpga_top.ltx current?"
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p_trig [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p_trig] == 0} {
    puts stderr "ERROR: expected trigger probe (dbg_ila_rob_pc_w) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

# Small pretrigger (position near the START of the 4096-sample depth) so
# ~3900 samples are POST-trigger -- what we actually want to see is what
# happens AFTER entering the loop, not the run-up to it.
set_property CONTROL.TRIGGER_POSITION 200 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h40800888 $p_trig
puts "=== Trigger: dbg_ila_rob_pc_w == 0x40800888 (dbf loop body), position=200/4096 ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="
puts "=== Waiting up to ${timeout_min} min for trigger (issue a board reset now if not already mid-boot) ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) -- loop body PC was never reached in the window ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP (caller must grep for 'No data to upload' WARNING above to confirm real vs empty) ==="

catch {write_hw_ila_data -csv_file /tmp/ila_stuck_loop_capture.csv -force $data}
puts "=== Wrote /tmp/ila_stuck_loop_capture.csv (if supported) ==="

puts "=== DONE ==="
