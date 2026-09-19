# synth/ila_branch_eu_s1_capture.tcl -- capture BranchEuPlugin's own S1-stage
# internals (does the 0x40800284 bsrw uop even ARRIVE at the branch EU, what
# does it carry, what does the EU compute) plus RobPlugin's retire-time
# redirect-gating chain (the three ANDed terms of
# `branchRedirect = retire0 && p0.retireAlone && mispredictStore(h0)`), for
# docs/BUG_calibration_word_misplaced_0d00.md Part 27.
#
# BACKGROUND: Part 26 found, with direct live hardware evidence (4/4
# independent occurrences), that decode-time fetch of the `bsrw
# 0x408010f0` at ROM PC 0x40800284 ALWAYS falls straight through to
# 0x40800288 -- no redirect of ANY kind (correct target, or any of the 6
# previously catalogued wrong targets) ever reaches the frontend. Every
# prior round's probes are either pre-decode (fetch-side: decodePc, FTQ,
# FTB, BTB, RAS) or post-retire (dbg_ila_rob_pc_w, the ROB head PC
# display) -- nobody has yet directly instrumented what BranchEuPlugin
# itself computes for this specific cracked-BSR branch micro-op, NOR the
# retire-time path a correctly-computed EU redirect must still travel
# through before it can ever reach FetchAlignPlugin's `mispredictRedirect`
# input. That path is NOT a direct wire:
#   BranchEuPlugin.completionPort (S1, out-of-order) only WRITES
#   RobPlugin.mispredictStore(robId); the actual frontend-visible flush,
#   RobPlugin.logic.branchRedirect = retire0 && p0.retireAlone &&
#   mispredictStore(h0), is a SEPARATE, LATER, IN-ORDER read of that same
#   storage, gated by TWO more conditions. Either condition silently false
#   at the moment this specific entry is at the retire head would exactly
#   reproduce Part 26's "no redirect ever reaches the frontend" finding
#   even if the EU computed everything correctly. `p.retireAlone :=
#   u.isBranch`, written once at ALLOC from the decode/rename-carried
#   `isBranch` field, is a plausible corruption/mis-set point for the
#   SECOND micro-op of a crack pair (BSR cracks into a store-phase uop
#   THEN a branch-phase uop) that no prior round has directly observed.
#
# TRIGGER: kept on the SAME proven probe as Parts 19/23/24/26
# (`dbg_ila_rob_pc_w == 0x40800284`), reliable across 7 prior real
# captures.
#
# PROBE MAPPING (cpu040 repo commit 75503f3 on
# debug/branch-eu-s1-ila-probes / this repo's
# rtl/soc/fpga_top_debug_ctrl.vh 2026-08-28 round-4 update -- 20 NEW
# repurposed BITS, packed into 3 already-existing, currently-dead-for-
# CPU_M68K040 slots; does NOT touch the Part 8/19/23/24/26 mapping, which
# stays live in the SAME capture for cross-checking):
#
#   Existing (Part 8/19/23/24/26, UNCHANGED):
#     dbg_ila_rob_pc_w             <- ROB head PC (TRIGGER; stale-prone,
#                                      kept for cross-reference)
#     ila_cdb0_pack[8]             <- branchRedirect (retire-time actual
#                                      flush -- ALREADY KNOWN to never fire
#                                      at this PC, per Part 26)
#     raw_daxi_awaddr              <- decodePc (32b, live frontend PC)
#     ila_rob_pop_bundle[2]        <- flushing
#
#   NEW this session (Part 27, round 4):
#     dbg_ila_snap_prf_idx_w[0]     <- beuS1Valid   (does the uop ARRIVE
#                                       at the branch EU's S1 stage at all)
#     dbg_ila_snap_prf_idx_w[4:1]   <- beuCond      (u1.cond -- should read
#                                       0=T for the BSR crack's branch half,
#                                       per MicroOpAssembler's explicit
#                                       cond=B(0,4 bits) force)
#     dbg_ila_snap_prf_idx_w[5]     <- beuIbranch   (u1.ibranch -- should
#                                       read 0, BSR is PC-relative not
#                                       indirect)
#     dbg_ila_snap_prf_idx_w[6]     <- beuTaken     (evaluated cccc
#                                       condition -- should read 1)
#     dbg_ila_exc_held_a7_phys_w[0] <- beuRawRedirect
#     dbg_ila_exc_held_a7_phys_w[1] <- beuAddrErr   (should read 0 --
#                                       0x408010f0 is even)
#     dbg_ila_exc_held_a7_phys_w[2] <- beuRedirect  (rawRedirect &&
#                                       !addrErr -- the actual "take this
#                                       branch" verdict the EU computes)
#     dbg_ila_exc_held_a7_phys_w[3] <- beuMispredict (should read 1 -- a
#                                       cold/never-predicted branch)
#     dbg_ila_exc_held_a7_phys_w[4] <- robRetire0   (RobPlugin retire stage
#                                       even attempting retire this cycle)
#     dbg_ila_exc_held_a7_phys_w[5] <- robHeadRetireAlone (p0.retireAlone
#                                       at the CURRENT retire head)
#     dbg_ila_exc_held_a7_phys_w[6] <- robHeadMispredictStore
#                                       (mispredictStore(h0) at the CURRENT
#                                       retire head -- did the EU's
#                                       completion write survive to retire)
#     dbg_ila_snap_tag_w[5:0]       <- beuCompRobId (the completing robId,
#                                       6b -- correlate THIS EU completion
#                                       to the SAME entry's later retire-
#                                       head appearance above, unambiguous)
#
# WHAT TO LOOK FOR in a decoded capture:
#   1. Find every sample where beuS1Valid==1 AND beuCond==4'h0 (=T) AND
#      beuIbranch==0 -- these are candidate BSR-crack branch-half
#      completions. Cross-reference against decodePc's own 4 known
#      0x40800284 occurrences (samples relative to THIS capture's own
#      trigger position) to find the ONE that corresponds to our PC (the
#      EU is fixed single-cycle S0->S1 latency after issue, so it should
#      land within a small, roughly-fixed window after the matching
#      decodePc sample -- rename+dispatch+IQ latency, typically a handful
#      of cycles).
#   2. At that sample: does beuTaken read 1? Does beuRedirect read 1? Does
#      beuMispredict read 1? If ALL THREE ARE 1 -- the EU computed
#      EVERYTHING correctly, and the bug is downstream, in the retire-time
#      gating chain (step 3). If ANY is 0, the bug is IN the EU itself
#      (u1.cond not actually surviving as the constant 0 the assembler
#      set, or the ibranch/taken/redirect computation misfiring for this
#      specific uop) -- a genuinely new, much harder-to-explain class of
#      finding (a literal RTL constant not surviving intact through
#      rename/issue under real hardware timing).
#   3. If step 2 shows a clean EU verdict (taken=1, redirect=1,
#      mispredict=1): watch beuCompRobId at that same sample, then scan
#      FORWARD for the sample(s) where dbg_ila_rob_pc_w == 0x40800284 (the
#      trigger condition) -- at THOSE samples, read robRetire0,
#      robHeadRetireAlone, robHeadMispredictStore. All three must be 1 for
#      branchRedirect to fire; ila_cdb0_pack[8] (branchRedirect itself) is
#      already known to read 0 throughout (Part 26) -- these three bits
#      pinpoint EXACTLY which ANDed term is the one suppressing it.
#      - robHeadMispredictStore==0 despite beuMispredict==1 at completion
#        -> the write was lost/overwritten before retire (a real storage-
#        hazard bug, e.g. the entry got reallocated, or a same-cycle-
#        alloc-vs-completion race clobbered it).
#      - robHeadRetireAlone==0 -> `p.retireAlone := u.isBranch` did not
#        survive for this uop (an alloc-time isBranch-classification bug,
#        plausibly specific to the SECOND micro-op of the BSR crack pair).
#      - robRetire0==0 -> retire itself is blocked for an unrelated reason
#        (headReady/faultedStore/isRte/sysOp/interruptPending/
#        privViolation/stopped/tracePendingFire/sysAuxRdy0/
#        haltAfterDue/haltAfterRetireBlock/debugBreakpointBoundaryHit) --
#        a completely different, upstream-of-branch-EU explanation.
#   4. If step 1 finds NO candidate sample at all (beuS1Valid never pairs
#      with cond==0/ibranch==0 anywhere near the known decodePc
#      occurrences) -- the uop never reaches the branch EU's S1 stage in
#      the first place, pointing to an issue/dispatch-side drop (a
#      completely different investigative angle: IssueQueuePlugin/
#      dispatch for this specific PC).
#
# METHOD: identical two-phase natural-boot discipline as Parts 19/23/24/26
# (NOT halt-based): confirm `halt-release`/`effective=0` first, THEN arm
# this trigger, THEN a plain `reset` pulse, then wait undisturbed.
# TRIGGER_POSITION is mid-window (2048/4096) so a single capture shows
# both the approach and the aftermath without needing separate runs.
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
#   tcl upload_hw_ila_data $::ila0; set ::d [get_hw_ila_data -of_objects $::ila0]; write_hw_ila_data -csv_file /tmp/ila_branch_eu_s1_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process -- only if NOT
# already attached via jtag_repl.tcl, to avoid a second contending JTAG
# connection):
#   vivado -mode batch -nojournal -nolog -source synth/ila_branch_eu_s1_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_branch_eu_s1_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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
puts "=== Trigger: dbg_ila_rob_pc_w == 0x40800284 (proven probe, Parts 19/23/24/26), position=2048/4096 ==="

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

catch {write_hw_ila_data -csv_file /tmp/ila_branch_eu_s1_capture.csv -force $data}
puts "=== Wrote /tmp/ila_branch_eu_s1_capture.csv (if supported) ==="

puts "=== DONE ==="
