# synth/ila_ftq_probes_capture.tcl -- capture cpu040's FTQ head-entry state
# and the FTB lookup-command's live query address around the moment
# `0x40800284`'s `bsrw 0x408010f0` diverges to a wrong target, for
# docs/BUG_calibration_word_misplaced_0d00.md Part 26.
#
# BACKGROUND: Part 25's re-analysis of the ALREADY-CAPTURED Part 19/23/24
# CSVs found that `dbg_ila_rob_pc_w == 0x40800284` during the "held" window
# is a STALE-PAYLOAD artifact (the ROB is genuinely EMPTY, count==0, for
# that whole window -- `payload.readAsync(h0)` has no occupancy gate) --
# NOT evidence of a live, valid, retiring 0x40800284 ROB entry. The first
# genuinely LIVE (count>0-backed) ROB entry after the redirect already
# carries the WRONG pc. This reframes the fault locus from "BSR target
# computation" (already well-eliminated, Parts 19/23/24) toward the
# fetch-recovery path itself: how the redirect's target is actually
# consumed by FetchAlignPlugin's `commonRedirect` and turned into a real
# fetch/decode, and specifically `ftqConfirm` (FetchAlignPlugin.scala:952),
# which matches an FTQ head entry to ANY simple instruction of the same
# LENGTH as `ftqHeadE.brLen`, with NO check that the current instruction is
# actually the same branch (or a branch at all) that trained that entry.
#
# TRIGGER: kept on the SAME proven probe as Parts 19/23/24
# (`dbg_ila_rob_pc_w == 0x40800284`) rather than switching to the new
# `decodePc` tap -- `decodePc` (wired to `raw_daxi_awaddr`) is NOT its own
# addressable hw_ila probe: it only reaches the debug_ila IP packed inside
# TWO composite MARK_DEBUG buses (`ila_wb_addr[32:1]`, `ila_axi_w_snap
# [39:8]`), so it is available as an ANALYSIS COLUMN in the capture but
# constructing a reliable bit-sliced TRIGGER_COMPARE_VALUE against a packed
# bus is a real, demonstrated failure class in this exact investigation
# (Part 24's own "probe-wiring mistake" postmortem) -- not worth the risk
# under time pressure when `dbg_ila_rob_pc_w` is already proven reliable
# across 6 real captures. Cross-referencing `decodePc` (via `ila_wb_addr`)
# against `dbg_ila_rob_pc_w` in the SAME capture is exactly what directly
# tests Part 25's "stale ROB payload, live decodePc" reframing.
#
# PROBE MAPPING (cpu040 repo commit 37fdd15 / this repo's
# rtl/soc/fpga_top_debug_ctrl.vh 2026-08-28 round-3 update -- 6 NEW
# repurposed `dbg_ila_*_w`/`raw_daxi_awaddr` slots, ALL taken from wires
# previously tied to a hardcoded 0 for CPU_M68K040 with a single MARK_DEBUG
# probe consumer each; does NOT touch the Part 8/19/23/24 mapping, which
# stays live in the SAME capture for cross-checking):
#
#   Existing (Part 8/19/23/24, UNCHANGED):
#     dbg_ila_rob_pc_w             <- ROB head PC (stale-prone, kept for
#                                      cross-reference against decodePc)
#     ila_cdb0_pack[8]             <- branchRedirect
#     ila_rob_pop_bundle[2]        <- flushing (Part 25's TRUE general
#                                      flush/redirect/squash signal)
#     dbg_ila_effective_halt_w     <- rasPredValid
#     dbg_ila_real_a7_val_w        <- rasPredTarget
#     dbg_ila_take_rte_finalize_w  <- btbPredHitComb
#     dbg_ila_prf01_w              <- btbPredTargetComb
#     dbg_ila_supervisor_mode_w    <- btbUpdValid
#     dbg_ila_if_pc_w              <- btbUpdPc
#     dbg_ila_pred_pc_w            <- btbUpdTarget
#     dbg_ila_take_finalize_w      <- ftbRspValid
#     dbg_ila_rob_brt_w            <- ftbRspHit
#     dbg_ila_exc_held_fault_pc_w  <- ftbRspTarget
#
#   NEW this session (Part 26):
#     dbg_ila_rob_brtgt_w          <- ftbCmdWindowPc (FTB lookup command's
#                                      LIVE query address, 32b -- Part 24's
#                                      own #1 recommended next step)
#     raw_daxi_awaddr              <- decodePc (32b, ALSO the trigger)
#     dbg_ila_dc_rdata_w           <- ftqHeadBrPc (FTQ head entry's own
#                                      trained PC, 32b)
#     dbg_ila_snap_ea_w            <- ftqHeadTarget (FTQ head entry's own
#                                      trained/predicted target, 32b)
#     dbg_ila_rob_arch_dst_w[4:1]  <- ftqHeadBrLen (4b)
#     dbg_ila_rob_arch_dst_w[0]    <- ftqConfirm (1b -- the decode-time
#                                      confirm gate itself)
#     dbg_ila_rob_phys_old_w[6:1]  <- ftqCount (6b, FTQ occupancy)
#     dbg_ila_rob_phys_old_w[0]    <- ftbCmdValid (1b)
#
# WHAT TO LOOK FOR in a decoded capture: find the sample where decodePc
# first reads 0x40800284 (the trigger). From there:
#   * Does `ftqConfirm` (dbg_ila_rob_arch_dst_w[0]) pulse HIGH at or near
#     that same cycle? If so, what does `ftqHeadBrPc` read at that exact
#     moment -- does it equal 0x40800284 (a genuine match) or something
#     ELSE (proving `ftqConfirm`'s length-only check matched a
#     STALE/WRONG-WINDOW FTQ head entry)? And does `ftqHeadTarget` at that
#     moment equal one of the previously observed wrong targets
#     (0x40884074, 0x40800634, 0x4088159a, 0x408001c0, 0x4080b1ee)? If
#     BOTH: this is the smoking gun -- hypothesis (a) confirmed directly.
#   * If `ftqConfirm` does NOT fire (ftqCount==0 / not-near at that PC),
#     the fallback BTB/decode-time path is in control instead -- cross-
#     check btbPredHitComb/btbPredTargetComb at the same cycle.
#   * Compare `ftbCmdWindowPc` (the FTB lookup command's live query
#     address) against 0x408010f0's actual window and against
#     0x40800284's own window, at the cycles surrounding the trigger --
#     directly answers whether the frontend ever issued an FTB lookup for
#     the CORRECT post-redirect window at all, closing Part 24's own open
#     "ftbRspHit@sample-52 coincidence" thread (command-side visibility
#     that session's probe set lacked).
#   * Cross-check `dbg_ila_rob_pc_w` (stale-prone) against `decodePc`
#     (live) across the same window -- directly confirms or refutes Part
#     25's "stale ROB payload artifact" reading with a second, independent
#     capture.
#
# METHOD: identical two-phase natural-boot discipline as Parts 19/23/24
# (NOT halt-based -- see Part 19's own retracted Capture 1 for why):
# confirm `halt-release`/`effective=0` first, THEN arm this trigger, THEN a
# plain `reset` pulse, then wait undisturbed. TRIGGER_POSITION is set
# mid-window (2048/4096) so a SINGLE capture shows both the approach (the
# preceding flush/redirect) and the aftermath (ftqConfirm's own behavior,
# the wrong-target transition) without needing separate post/pre-trigger
# runs like Parts 19+23 did.
#
# Usage (interactively, via jtag_repl.tcl's `tcl` passthrough, in the SAME
# session that already holds hw_target open):
#   tcl set ::ila0 [lindex [get_hw_ilas -of_objects $::hw_dev] 0]
#   tcl set ::p_pc0 [get_hw_probes dbg_ila_rob_pc_w -of_objects $::ila0]
#   tcl set_property CONTROL.TRIGGER_POSITION 2048 $::ila0
#   tcl set_property CONTROL.TRIGGER_CONDITION AND $::ila0
#   tcl set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $::p_pc0
#   tcl run_hw_ila $::ila0
#   reset
#   tcl wait_on_hw_ila -timeout 5 $::ila0
#   tcl upload_hw_ila_data $::ila0; set ::d [get_hw_ila_data -of_objects $::ila0]; write_hw_ila_data -csv_file /tmp/ila_ftq_probes_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process -- only if NOT
# already attached via jtag_repl.tcl, to avoid a second contending JTAG
# connection):
#   vivado -mode batch -nojournal -nolog -source synth/ila_ftq_probes_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_ftq_probes_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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

set_property CONTROL.TRIGGER_POSITION 2048 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $p_trig
puts "=== Trigger: dbg_ila_rob_pc_w == 0x40800284 (proven probe, Parts 19/23/24), position=2048/4096 -- decodePc (ila_wb_addr\[32:1\]) captured alongside for cross-reference ==="

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

catch {write_hw_ila_data -csv_file /tmp/ila_ftq_probes_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ftq_probes_capture.csv (if supported) ==="

puts "=== DONE ==="
