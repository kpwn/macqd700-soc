# synth/ila_ipl_rising_capture.tcl — capture RobPlugin's interrupt-
# recognition internals around the moment iplActive FIRST rises during
# natural boot (i.e. the moment the interrupt controller asserts an
# unmasked pending IPL to the CPU), for the 2026-08-27 boot investigation
# (docs/BUG_calibration_word_misplaced_0d00.md Parts 6-8).
#
# Sibling of ila_stuck_loop_capture.tcl, which triggers on ROB-head PC
# entering the calibration dbf loop (0x40800888) -- that capture's first
# live run showed the loop spinning for its entire ~4096-cycle window with
# iplActive=0 throughout (round 1's VIA1 Timer2 hadn't fired yet -- too
# early/short a window relative to the real timer period). This script
# instead triggers on iplActive's rising edge directly, wherever/whenever
# that happens in natural boot -- no guessing about which PC or which
# round. TRIGGER_POSITION is mid-buffer so the capture shows a little
# leadup (was the CPU already in the dbf loop when IPL asserted?) and
# ~3500 cycles of aftermath (did normalIrqGate/interruptPending follow
# promptly, or stay stuck low despite iplActive=1?).
#
# PROBE MAPPING: see ila_rob_irq_recognition_capture.tcl's header for the
# full corrected mapping.
#   ila_axi_w_snap[0]     <- RobPlugin.iplActive        (THIS SCRIPT'S TRIGGER)
#   ila_rob_pop_bundle[3] <- RobPlugin.normalIrqGate
#   ila_rob_pop_bundle[2] <- RobPlugin.flushing
#   dbg_ila_flush_en_w    <- RobPlugin.excIdle
#   ila_cdb2_pack[8]      <- RobPlugin.interruptPending
#   dbg_ila_rob_pc_w      <- RobPlugin's ROB-head PC (p0.pc)
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_ipl_rising_capture.tcl \
#       -tclargs <ltx_file> [timeout_min]
#
# Does NOT reset the board -- arm this against a board that is either
# already mid-boot, or about to be reset externally via the jtag_repl.tcl
# FIFO (`echo reset > /tmp/jtag_in`).

if {[llength $argv] < 1} {
    puts stderr "ERROR: usage: ila_ipl_rising_capture.tcl <ltx_file> \[timeout_min\]"
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

set p_trig [get_hw_probes ila_axi_w_snap -of_objects $ila]
if {[llength $p_trig] == 0} {
    puts stderr "ERROR: expected trigger probe (ila_axi_w_snap) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

# iplActive is bit[0] (LSB) of the 40-bit ila_axi_w_snap probe -- the
# other 39 bits (raw_daxi_* AXI W-channel snapshot) are all tied to 0 in
# this cpu040 build, so any nonzero value on this probe IS iplActive=1.
set_property CONTROL.TRIGGER_POSITION 500 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq40'h0000000001 $p_trig
puts "=== Trigger: ila_axi_w_snap\[0\] (iplActive) == 1, position=500/4096 ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="
puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) -- iplActive never rose in the window ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP (caller must grep for 'No data to upload' WARNING above to confirm real vs empty) ==="

catch {write_hw_ila_data -csv_file /tmp/ila_ipl_rising_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ipl_rising_capture.csv (if supported) ==="

puts "=== DONE ==="
