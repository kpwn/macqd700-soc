# synth/ila_rob_irq_recognition_capture.tcl — capture RobPlugin's interrupt-
# recognition internals around the moment `interruptPending` finally fires,
# for the 2026-08-27 boot investigation (see
# docs/BUG_calibration_word_misplaced_0d00.md Part 8): a genuinely
# pending+unmasked+enabled interrupt was NOT recognized for the entire
# natural duration of a tight `dbf` loop on real hardware, then recognized
# instantly once the loop was broken externally via a JTAG halt/resume. Four
# synthetic sim repro attempts (see the cpu040 repo's ExecuteLockStepSpec
# "probe: does a real dbf loop..." test) all showed CORRECT behavior in
# isolation, so the bug depends on real boot-time context this capture is
# meant to expose directly from the RTL's own internal signals.
#
# PROBE MAPPING (these dbg_ila_*_w wires are REPURPOSED 1-bit slots from the
# v1-era 44-probe map -- their v1 names/semantics do NOT apply to this
# capture; see rtl/soc/fpga_top_debug_ctrl.vh's 2026-08-27 comment at the
# CPU_M68K040 instantiation for the authoritative wiring).
#
# CORRECTED 2026-08-27 (first live attempt found the real probe list):
# fpga_top_debug_vio.vh packs several of these 1-bit slots into wider
# vector probes before handing them to the debug_ila IP, so the hw_ila
# probe names are NOT 1:1 with the dbg_ila_*_w wire names -- listed here
# against the real `get_hw_probes` output:
#   ila_rob_pop_bundle[3] <- dbg_ila_rob_pop_w         <- RobPlugin.normalIrqGate
#   ila_rob_pop_bundle[2] <- dbg_ila_rob_is_last_uop_w <- RobPlugin.flushing
#   dbg_ila_flush_en_w    (standalone)                 <- RobPlugin.excIdle
#   ila_axi_w_snap[0]     <- dbg_ila_dc_aw_is_evict_w  <- RobPlugin.iplActive
#     (NOT ila_wb_addr[0] -- that copy is ANDed with raw_daxi_awvalid,
#     which this build ties to 0, so it would always read 0.)
#   ila_cdb0_pack[8]      <- dbg_ila_cdb0_en_w         <- RobPlugin.branchRedirect
#   ila_cdb0_pack[7]      <- dbg_ila_cdb0_has_dst_w    <- RobPlugin.p0.first
#   ila_cdb1_pack[8]      <- dbg_ila_cdb1_en_w         <- RobPlugin.preciseDrainBusyIn
#   ila_cdb1_pack[7]      <- dbg_ila_cdb1_has_dst_w    <- RobPlugin.inhibitedLoadBusyIn
#   ila_cdb2_pack[8]      <- dbg_ila_cdb2_en_w         <- RobPlugin.interruptPending  (TRIGGER)
#   dbg_ila_rob_pc_w      (standalone, 32b)            <- RobPlugin's ROB-head PC (p0.pc)
#     for correlation against the known calibration-loop addresses
#     (0x40800852 store, 0x40800888 dbf loop body, 0x40800866 temp-ISR entry).
# ila_cdb{0,1,2}_pack[6:0] and ila_rob_pop_bundle[4] / [1:0] are v1-only
# concepts this build ties to 0 or leaves as unrelated A7/commit-bundle
# taps -- ignore them when reading a capture.
#
# Trigger: ila_cdb2_pack bit[8] (interruptPending) rising to 1 -- the FIRST
# time the CPU recognizes ANY interrupt after arming. TRIGGER_POSITION is
# set near the END of the 4096-sample depth so most of the buffer is
# PRE-trigger context (~3800 samples leading up to recognition) -- exactly
# the window that matters: what were normalIrqGate/flushing/excIdle/
# branchRedirect/p0.first doing in the run-up to (finally) recognizing the
# interrupt.
#
# This does NOT reprogram the device and does NOT reset by default (unlike
# ila_a7_capture.tcl's pattern) -- pass -tclargs ... reset only if you want
# a fresh cold boot; the natural real-hardware repro needs no external
# intervention to REACH the stall (round 1's DIVU-by-zero-adjacent
# calibration loop is very early boot), but per the known finding, the
# interrupt is NOT naturally recognized inside the loop at all -- if this
# capture times out with no trigger, that itself reproduces the bug
# (interruptPending genuinely never asserted), and the next step is to
# break the loop externally via jtag_repl.tcl's `reset-and-break-pc`/
# `continue` sequence (see docs/BUG_calibration_word_misplaced_0d00.md
# Part 6/8) WHILE this capture is still armed and waiting -- that external
# break is expected to make ila_cdb2_en fire, which this capture will catch.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_rob_irq_recognition_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_rob_irq_recognition_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set do_reset    [expr {[llength $argv] >= 3 ? [lindex $argv 2] : "noreset"}]
set timeout_min [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 5.0}]

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

# interruptPending lives at bit[8] (MSB) of the packed 9-bit
# ila_cdb2_pack = {cdb2_en, cdb2_has_dst, cdb2_phys[6:0]} probe -- there
# is no standalone ila_cdb2_en probe (see the corrected mapping above).
set p_trig [get_hw_probes ila_cdb2_pack -of_objects $ila]
if {[llength $p_trig] == 0} {
    puts stderr "ERROR: expected trigger probe (ila_cdb2_pack) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 3800 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq9'b1XXXXXXXX $p_trig
puts "=== Trigger: ila_cdb2_pack\[8\] (interruptPending) rising to 1, position=3800/4096 ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

if {$do_reset eq "reset"} {
    after 300
    puts "=== Firing reset via: $jcmd_path reset ==="
    if {[catch {exec $jcmd_path reset} reset_out]} {
        puts "WARN: reset exec returned error/nonzero: $reset_out"
    } else {
        puts "reset output:\n$reset_out"
    }
} else {
    puts "=== NOT resetting (noreset) -- assumes the board is already mid-boot ==="
    puts "=== or that the operator will break the loop externally via jtag_repl.tcl ==="
}

puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP (caller must grep for 'No data to upload' WARNING above to confirm real vs empty) ==="

catch {write_hw_ila_data -csv_file /tmp/ila_rob_irq_recognition_capture.csv -force $data}
puts "=== Wrote /tmp/ila_rob_irq_recognition_capture.csv (if supported) ==="

puts "=== FULL CAPTURE TABLE ==="
display_hw_ila_data $data

puts "=== DONE ==="
