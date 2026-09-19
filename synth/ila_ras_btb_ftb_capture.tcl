# synth/ila_ras_btb_ftb_capture.tcl -- capture cpu040's RAS occupancy and
# BTB/FTB training payload around the moment `0x40800284`'s `bsrw
# 0x408010f0` retires to a WRONG target, for
# docs/BUG_calibration_word_misplaced_0d00.md Part 24.
#
# BACKGROUND (Parts 19/22/23): the wrong target is reproducibly (3/3
# captures) the fall-through/return address of a DIFFERENT, unrelated,
# already-completed call earlier in the SAME boot -- RAS-shaped, despite
# BSR being structurally, source-confirmed gated away from ever consulting
# the RAS (FetchAlignPlugin.scala's `predTargetSel` mux). Eight independent
# SpinalSim repro attempts (synthetic, byte-exact real-ROM-bytes, three
# AXI-latency-swept configs including chaos/randomized timing, and two
# topology-directed distant-call/indirect-jsr shapes) ALL FAILED to
# reproduce it. This capture adds real hardware visibility the prior ILA
# probe set did not have: RAS predValid/predTarget/count, the BTB's OWN
# combinational query-time hit/target for the CURRENT fetch PC, the shared
# BTB+FTB retire-time training bus (valid/pc/target), and the FTB's own
# separate C+1 token-matched lookup response (valid/hit/framedOk/target).
#
# PROBE MAPPING (cpu040 repo commit 1799dd6 / this repo's
# rtl/soc/fpga_top_debug_ctrl.vh 2026-08-28 update -- 12 NEW repurposed
# `dbg_ila_*_w` slots, ALL taken from wires that were previously tied to a
# hardcoded 0 for CPU_M68K040 and verified to have no other consumer logic;
# does NOT touch any of the existing Part 8/19/23 probe mapping below,
# which stays live in the SAME capture for cross-checking):
#
#   Existing (Part 8/19/23, UNCHANGED):
#     dbg_ila_rob_pc_w          <- ROB head PC (TRIGGER)
#     ila_cdb0_pack[8]          <- branchRedirect
#     ila_cdb2_pack[8]          <- interruptPending
#     ila_axi_w_snap[0]         <- iplActive
#     dbg_ila_rob_brt_w / dbg_ila_rob_brtgt_w -- REPURPOSED this session,
#       see below (Part 23 read these as always-0/flat under the OLD
#       mapping; that meaning no longer applies to a capture taken with
#       this build).
#
#   NEW this session (Part 24):
#     dbg_ila_effective_halt_w    <- rasPredValid   (RAS non-empty)
#     dbg_ila_real_a7_val_w       <- rasPredTarget   (RAS top-of-stack, 32b)
#     dbg_ila_rob_phys_dst_w      <- rasCount        (occupancy, 0..16, 7b)
#     dbg_ila_take_rte_finalize_w <- btbPredHitComb  (query-time BTB hit for
#                                    the CURRENT fetch PC)
#     dbg_ila_prf01_w             <- btbPredTargetComb (query-time BTB
#                                    predicted target, 32b)
#     dbg_ila_supervisor_mode_w   <- btbUpdValid     (retire-time BTB+FTB
#                                    shared training-write valid)
#     dbg_ila_if_pc_w             <- btbUpdPc        (PC being trained, 32b)
#     dbg_ila_pred_pc_w           <- btbUpdTarget    (target value written
#                                    into that entry, 32b)
#     dbg_ila_take_finalize_w     <- ftbRspValid     (FTB's own C+1
#                                    token-matched lookup response valid)
#     dbg_ila_rob_brt_w           <- ftbRspHit
#     dbg_ila_sp_slot_write_en_w  <- ftbRspFramedOk
#     dbg_ila_exc_held_fault_pc_w <- ftbRspTarget    (32b)
#
# WHAT TO LOOK FOR in a decoded capture: at/around sample where
# dbg_ila_rob_pc_w == 0x40800284 (stable ~10 cycles, matching Parts 19/23),
# then transitions to a wrong target on the FOLLOWING sample --
#   * If btbUpdValid pulses with btbUpdPc == 0x40800284-ish and btbUpdTarget
#     matching one of the previously observed wrong targets (fall-through of
#     an earlier call), THAT is the smoking gun: something is training the
#     BTB/FTB entry for this cold PC with a stale fall-through/return value.
#   * If instead btbPredHitComb reads 1 with btbPredTargetComb already
#     matching a wrong target BEFORE any local btbUpdValid pulse, the entry
#     was trained EARLIER (need to scan backward through the whole capture
#     for the training write that planted it).
#   * If rasPredValid/rasPredTarget/rasCount show the RAS non-empty with
#     top-of-stack == the wrong target at the moment of the bad redirect,
#     that is direct confirmation of a RAS-consultation leak DESPITE the
#     source-level BSR/RAS gate (i.e. the gate itself has a real bug, not
#     just "BSR reads RAS" as literally written).
#   * If NONE of the above show anything anomalous, the mechanism is
#     neither the BTB/FTB training path nor a RAS leak as directly
#     observable at these tap points -- report exactly that.
#
# METHOD: identical two-phase natural-boot discipline as Parts 19/23 (NOT
# halt-based -- see Part 19's own retracted Capture 1 for why): confirm
# `halt-release`/`effective=0` first, THEN arm this trigger, THEN a plain
# `reset` pulse, then wait undisturbed.
#
# Usage (interactively, via jtag_repl.tcl's `tcl` passthrough, in the SAME
# session that already holds hw_target open):
#   tcl set ::ila0 [lindex [get_hw_ilas -of_objects $::hw_dev] 0]
#   tcl set ::p_pc0 [get_hw_probes dbg_ila_rob_pc_w -of_objects $::ila0]
#   tcl set_property CONTROL.TRIGGER_POSITION 50 $::ila0
#   tcl set_property CONTROL.TRIGGER_CONDITION AND $::ila0
#   tcl set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $::p_pc0
#   tcl run_hw_ila $::ila0
#   reset
#   tcl wait_on_hw_ila -timeout 5 $::ila0
#   tcl upload_hw_ila_data $::ila0; set ::d [get_hw_ila_data -of_objects $::ila0]; write_hw_ila_data -csv_file /tmp/ila_ras_btb_ftb_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process -- only if NOT
# already attached via jtag_repl.tcl, to avoid a second contending JTAG
# connection):
#   vivado -mode batch -nojournal -nolog -source synth/ila_ras_btb_ftb_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_ras_btb_ftb_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set do_reset    [expr {[llength $argv] >= 3 ? [lindex $argv 2] : "reset"}]
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

set p_trig [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p_trig] == 0} {
    puts stderr "ERROR: expected trigger probe (dbg_ila_rob_pc_w) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 50 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $p_trig
puts "=== Trigger: dbg_ila_rob_pc_w == 0x40800284 (the bsrw 0x408010f0 call site), position=50/4096 ==="

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
    puts "=== NOT resetting (noreset) -- assumes the board will reach 0x40800284 on its own ==="
}

puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) -- 0x40800284 was never reached in the window ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP ==="

catch {write_hw_ila_data -csv_file /tmp/ila_ras_btb_ftb_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ras_btb_ftb_capture.csv (if supported) ==="

puts "=== DONE ==="
