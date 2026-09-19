# synth/ila_sq_exc_drain_capture.tcl -- round 6: real-hardware ILA capture
# of StoreQueue's own internal ring-occupancy state + ExceptionUnit's
# commit-time sysOp sequencer state, at the moment of a LIVE, undisturbed
# `0x40887126` CPUSHL/dbf cache-maintenance-loop hang --
# docs/BUG_calibration_word_misplaced_0d00.md Part 52/53.
#
# BACKGROUND: Part 36 narrowed this hang to ExceptionUnit's `S_DRAIN` state
# (`src/main/scala/m68k040/exception/ExceptionUnit.scala`) never seeing
# `sqDrained && dcQuiesced` go true -- a debug-injected maintenance push
# (which bypasses S_DRAIN entirely) always completes cleanly while parked at
# the hang, ruling OUT DcachePlugin's own AXI/quiescence tracking
# (`dcQuiesced`) as the blocker and narrowing suspicion to `sqDrained`
# (== `StoreQueue.io.empty`) being stuck permanently false -- most likely a
# "phantom entry never drains" class bug, the same general failure mode
# already fixed once in this exact file (Part 37, `efcd953`) for a DIFFERENT
# (debug-flush) trigger. Part 38 proved Part 37's fix does NOT clear the
# real-hardware stall (still reproduces byte-identical). Part 52 built 3
# directed sim tests at increasing realism (isolated maintenance engine,
# real StoreQueue occupancy, real MMU-translated COPYBACK dirty lines) --
# ALL THREE PASS cleanly in sim, meaning the natural trigger could not be
# reproduced off real hardware's own richer concurrent boot activity. This
# capture is the direct, ground-truth follow-up Part 36/52 both recommended:
# read StoreQueue's actual internal ring state DIRECTLY at the moment of a
# real, undisturbed hang, rather than inferring it from external symptoms.
#
# PROBE MAPPING (cpu040 repo commit d5b2ab25 on
# debug/branch-eu-s1-ila-probes-on-rasfix-sqfix, round 6, on top of round 5's
# 88bc4920 / this repo's rtl/soc/fpga_top_debug_ctrl.vh 2026-08-30 round-6
# update -- 2 round-6 repurposed WHOLE 32b wires; does NOT touch any prior
# round's mapping, which stays live in the SAME capture for cross-checking):
#
#   Existing (Parts 8/19/23/24/26, UNCHANGED, used here as the TRIGGER):
#     dbg_ila_rob_pc_w             <- ROB head PC (TRIGGER == 0x40887126,
#                                      the `cpushl bc,(a1)` opword's own PC,
#                                      per Part 36 S2.2's ROM disassembly)
#
#   NEW this session (round 6):
#     dbg_ila_cdb1_data_w (probe21) <- sqState (StoreQueue.io.dbgState,
#       32b, packed):
#         [31:29] reserved (0)
#         [28]    ackPhaseB
#         [27]    sendPhaseB
#         [26]    empty            <-- THE decisive bit: == sqDrained
#                                      upstream, the value S_DRAIN gates on
#         [25]    drainBusy
#         [24:22] sendPtr
#         [21:19] tail
#         [18:16] head
#         [15:8]  committed[7:0]   <-- per-entry committed bits
#         [7:0]   valids[7:0]      <-- per-entry valid bits (occupancy)
#
#     dbg_ila_a7_writeback_val_w (probe22) <- excSqState (ExceptionUnit's
#       commit-time sysOp sequencer state, 32b, packed):
#         [31:23] reserved (0)
#         [22:7]  maintCmdOut.payload.addr[15:0] (the CPUSHL target line
#                                      address S_APPLY last pulsed, if any)
#         [6]     sqDrained        <-- ExceptionUnit's OWN copy of the same
#                                      value (cross-check against sqState[26]
#                                      -- if they ever disagree, that is
#                                      itself a real, separate bug: a wiring/
#                                      timing gap between StoreQueue.io.empty
#                                      and exc.sqDrained, e.g. LsEuPlugin's
#                                      `sqEmptySig := sq.io.empty` cross-
#                                      plugin assignment landing a cycle late)
#         [5]     maintCmdOut.valid (S_APPLY's 1-cycle CPUSH pulse -- should
#                                      read 0 while parked, confirming the
#                                      hang is genuinely upstream of S_APPLY)
#         [4]     dcQuiesced       <-- already known True from Part 36's
#                                      debug-push discriminator; included for
#                                      direct confirmation on THIS exact
#                                      capture
#         [3]     fsmIsSRedir
#         [2]     fsmIsSMaintWait
#         [1]     fsmIsSApply
#         [0]     fsmIsSDrain      <-- should read 1 continuously if Part
#                                      36's S_DRAIN hypothesis is correct
#
# WHAT TO LOOK FOR in a decoded capture:
#   1. At the trigger sample (dbg_ila_rob_pc_w == 0x40887126) and every
#      sample after it in the capture window: does excSqState bit[0]
#      (fsmIsSDrain) read 1 continuously? If NOT -- the FSM is NOT parked in
#      S_DRAIN at all, and the whole S_DRAIN/sqDrained hypothesis from Part
#      36 is WRONG for the undisturbed case; check which OTHER fsmIs* bit
#      (or none) is set instead, and re-root-cause from there.
#   2. If fsmIsSDrain reads 1: does sqState bit[26] (empty/sqDrained) read 0
#      continuously? If YES -- Part 36's hypothesis is DIRECTLY CONFIRMED
#      with real hardware ground truth: StoreQueue never reports empty, so
#      S_DRAIN can never advance to S_APPLY. Proceed to step 3 to find WHY.
#      If sqState[26] instead reads 1 (empty IS true) while fsmIsSDrain
#      also reads 1 -- that is a DIFFERENT, more surprising bug: the FSM is
#      seeing its OWN gating condition true but not advancing, which would
#      point at a combinational/registration bug in the `when(sqDrained &&
#      dcQuiesced) { goto(S_APPLY) }` statement itself or a `dcQuiesced`
#      that reads 1 in isolation (excSqState bit[4]) but the AND somehow
#      still gates false -- re-inspect ExceptionUnit.scala's exact S_DRAIN
#      code around line 1426 against this capture's dcQuiesced/sqDrained
#      bits together.
#   3. If sqState[26]==0 confirmed: read sqState[7:0] (valids) and
#      sqState[15:8] (committed) directly. A single stuck bit (e.g. only
#      valids[k]==1 for one k, unchanging across the whole post-trigger
#      window) is the "phantom entry" signature Part 37 already fixed once
#      for a different trigger -- identifies EXACTLY which ring slot is
#      wedged. Cross-reference sqState[18:16] (head) against that k: if
#      head == k, the phantom entry is AT THE HEAD (blocking the whole ring
#      from ever reporting empty, matching `io.empty := !valids.reduce(_ ||
#      _) && !drainBusy` exactly). If head != k, the entry is NOT at head --
#      a DIFFERENT, more surprising shape than Part 37's fix covered (that
#      fix's own `headDrainInFlight` guard is specifically head-relative).
#      Also check sqState[25] (drainBusy) and [28]/[27]
#      (ackPhaseB/sendPhaseB) -- `io.empty := !valids.reduce(...) &&
#      !drainBusy`, so `drainBusy` stuck true with ALL valids clear is a
#      SEPARATE possible root cause the "phantom valid entry" framing above
#      does not cover; check `acceptedHalves =/= 0` never resolving instead.
#   4. Cross-check excSqState[6] (exc's own sqDrained copy) against
#      sqState[26] (StoreQueue's raw io.empty) at every sample. Any
#      disagreement between these two bits (which should be electrically
#      the same signal, `exc.sqDrained := lsEu.sqEmptySig := sq.io.empty`)
#      is itself a real, separate, actionable finding -- a genuine wiring/
#      CDC-class bug between LsEuPlugin and ExceptionUnit, independent of
#      whatever StoreQueue itself is doing internally.
#
# METHOD: PASSIVE ILA trigger arming does NOT interact with CPU execution
# (unlike a JTAG hardware breakpoint) -- Part 40's finding that an ARMED
# BREAKPOINT suppresses the early bsrw-collision race does NOT apply here;
# this capture uses only the hw_ila's own free-running probe-compare logic,
# which never asserts anything back onto the CPU. Same two-phase
# undisturbed-boot discipline as every prior round (NOT halt-based): confirm
# `halt-release`/`effective=0` first, THEN arm this trigger, THEN a plain
# `reset` pulse (full `load-bit` reprogram per this doc's own established
# per-trial discipline, done OUTSIDE this script before each attempt), then
# wait undisturbed. Only ~9-40% of cold boots escape the early bsrw-
# collision race at all (Part 33/51), and only about half of THOSE land on
# THIS specific stall (the other half land on the VIA2/`0x0000a8c0` stall
# instead, per Part 51's even 2/2 split) -- budget several reset attempts
# per successful capture. TRIGGER_POSITION is kept SMALL (128/4096) since
# the hang is already independently proven PERMANENT (Part 36 S2.2, static
# across 90+ real seconds) -- a large post-trigger window directly tests
# whether the packed state is ALSO static within the ILA's own much shorter
# (~41us at 100MHz) capture window, which is itself informative (any change
# within that window would mean the state is NOT simply frozen, contradicting
# the byte-static register evidence).
#
# Usage (interactively, via jtag_repl.tcl's `tcl` passthrough, in the SAME
# session that already holds hw_target open):
#   tcl set ::ila0 [lindex [get_hw_ilas -of_objects $::hw_dev] 0]
#   tcl set ::p_pc0 [get_hw_probes dbg_ila_rob_pc_w -of_objects $::ila0]
#   tcl set_property CONTROL.TRIGGER_POSITION 128 $::ila0
#   tcl set_property CONTROL.TRIGGER_CONDITION AND $::ila0
#   tcl set_property TRIGGER_COMPARE_VALUE eq32'h40887126 $::p_pc0
#   tcl run_hw_ila $::ila0
#   reset
#   tcl wait_on_hw_ila -timeout 5 $::ila0
#   tcl upload_hw_ila_data $::ila0; set ::d [get_hw_ila_data -of_objects $::ila0]; write_hw_ila_data -csv_file /tmp/ila_sq_exc_drain_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process -- only if NOT
# already attached via jtag_repl.tcl, to avoid a second contending JTAG
# connection):
#   vivado -mode batch -nojournal -nolog -source synth/ila_sq_exc_drain_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_sq_exc_drain_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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

set_property CONTROL.TRIGGER_POSITION 128 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h40887126 $p_trig
puts "=== Trigger: dbg_ila_rob_pc_w == 0x40887126 (proven probe, Parts 19/23/24/26; PC == the cpushl bc,(a1) opword's own address per Part 36 S2.2), position=128/4096 ==="

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
    puts "=== NOT resetting (noreset) -- assumes the board will reach 0x40887126 on its own ==="
}

puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) -- 0x40887126 was never reached in the window ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP ==="

catch {write_hw_ila_data -csv_file /tmp/ila_sq_exc_drain_capture.csv -force $data}
puts "=== Wrote /tmp/ila_sq_exc_drain_capture.csv (if supported) ==="

puts "=== DONE ==="
