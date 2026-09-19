# synth/ila_robplugin_headready_capture.tcl -- round 7: real-hardware ILA
# capture of RobPlugin's own headReady/sysRetire gate + coreHalted/
# haltReason/DcachePlugin-diagFault attribution, at the moment of a LIVE,
# undisturbed `0x40887126` CPUSHL/dbf cache-maintenance-loop hang --
# docs/BUG_calibration_word_misplaced_0d00.md Part 53's precise closing
# recommendation.
#
# BACKGROUND: Part 53's round-6 capture DEFINITIVELY REFUTED the standing
# S_DRAIN/`sqDrained` hypothesis (Parts 36/38/52): a live-hang re-arm capture
# showed `StoreQueue.io.empty`/`sqDrained` reading TRUE and `dcQuiesced`
# reading TRUE the WHOLE 4096-sample window, while ALL FOUR of
# ExceptionUnit's S_DRAIN/S_APPLY/S_MAINTWAIT/S_REDIR FSM bits read 0 --
# the commit-time sysOp sequencer never even leaves IDLE. Cross-referencing
# every OTHER already-tapped gating term in the SAME capture (`excIdle`,
# `flushing`, `iplActive`, `branchRedirect`, `preciseDrainBusyIn`,
# `inhibitedLoadBusyIn`, `interruptPending`) showed all of them clean/
# non-blocking too, narrowing the real blocker to RobPlugin's own gating
# chain (`RobPlugin.scala`, read directly):
#
#   val headReady = (count > 0) && completes(h0) && !flushing &&
#                    !coreHalted && !debugHalted
#   val sysRetire = headReady && p0.sysOp && sysValRdyStore(h0) &&
#                   !faultedStore(h0) && !p0.isRte && excIdle
#   sysTriggerSig := sysRetire && (exc.ss.s || sysUserOk)
#
# Since `excIdle`/`!flushing` are already confirmed satisfied, the real
# blocker is one of `completes(h0)`, `sysValRdyStore(h0)`, `coreHalted`, or
# `debugHalted`/`count>0`. `coreHalted` (a STICKY fatal latch, `RegInit
# (False)`, set by `dc.diagFault || exc.fsXlateFault || arbWedge || rvHalt`
# in `FullCoreSynth.scala`) was flagged the single strongest concrete lead
# -- it ties directly to Part 38's own unconfirmed side finding (a debug
# maintenance push returning a real AXI writeback error at this EXACT PC).
#
# PROBE MAPPING (cpu040 repo commit b7494b98 on
# debug/branch-eu-s1-ila-probes-on-rasfix-sqfix, round 7, on top of round 6's
# d5b2ab25 / this repo's rtl/soc/fpga_top_debug_ctrl.vh 2026-08-30 round-7
# update -- 2 BRAND NEW `debug_ila` probes, 44/45, C_NUM_OF_PROBES bumped
# 44->46; does NOT touch any prior round's mapping, which stays live in the
# SAME capture for cross-checking):
#
#   Existing (Parts 8/19/23/24/26, UNCHANGED, used here as the TRIGGER):
#     dbg_ila_rob_pc_w             <- ROB head PC (TRIGGER == 0x40887126,
#                                      the `cpushl bc,(a1)` opword's own PC,
#                                      per Part 36 S2.2's ROM disassembly)
#
#   NEW this session (round 7):
#     probe44 <- robGateState (RobPlugin/DcachePlugin packed word, 32b):
#         [0]     headReady        <-- the top-level gate ALL retirement
#                                      (sysRetire included) needs
#         [1]     completesH0      (= completes(h0), the ROB-head entry's
#                                      own EU-writeback-landed bit)
#         [2]     sysValRdyH0      (= sysValRdyStore(h0), a SEPARATE latch
#                                      tracking whether the EU writeback
#                                      VALUE specifically landed)
#         [3]     coreHalted       <-- THE decisive bit: a sticky fatal
#                                      D-cache/AXI-error/DTLB/arbiter latch
#         [4]     debugHalted
#         [5]     countGt0         (= rob.logic.count > 0)
#         [8:6]   haltReason[2:0]  (HaltReason enum: 0=NONE 1=DCACHE_DIAG
#                                      2=FS_XLATE 3=RESET_VECTOR
#                                      4=ARBITER_WEDGE -- first-wins
#                                      attribution of WHICH coreHaltedIn
#                                      disjunct fired, if any)
#         [9]     dcDiagFaultValid (DcachePlugin's own diagFaultValid copy)
#         [12:10] dcDiagFaultKind[2:0] (0=WT-beat/INHIBITED-drain,
#                                      1=drain-miss write-allocate refill,
#                                      2=eviction writeback, 3=CPUSH
#                                      maintenance writeback -- kind=3 is
#                                      the EXACT match for a CPUSHL-
#                                      triggered writeback error)
#         [14:13] dcDiagFaultResp[1:0] (raw AXI B-channel resp code:
#                                      0=OKAY 1=EXOKAY 2=SLVERR 3=DECERR)
#         [31:15] reserved (0)
#
#     probe45 <- diagFaultAddr[31:0] (DcachePlugin's own diagFaultAddr,
#                                      verbatim -- the physical address of
#                                      whichever AXI writeback errored, if
#                                      coreHalted/haltReason==DCACHE_DIAG
#                                      turns out to be the answer)
#
# WHAT TO LOOK FOR in a decoded capture:
#   1. At the trigger sample (dbg_ila_rob_pc_w == 0x40887126) and every
#      sample after it: does probe44 bit[3] (coreHalted) read 1
#      CONTINUOUSLY? This is the single question this round exists to
#      answer.
#   2. If coreHalted == 0 for the ENTIRE capture -- the coreHalted
#      hypothesis is REFUTED. Look at probe44 bits[1]/[2] (completesH0/
#      sysValRdyH0) instead: whichever reads 0 while headReady (bit[0])
#      also reads 0 is the real blocker. If BOTH completesH0 and
#      sysValRdyH0 read 1 but headReady (bit[0]) STILL reads 0, re-derive
#      headReady's own formula against bit[5] (countGt0) and bit[4]
#      (debugHalted) -- one of those, or a genuine RTL bug in the
#      combinational `headReady` assignment itself, is the answer, and a
#      round 8 with a direct h0/head-index probe may be needed.
#   3. If coreHalted == 1 -- CONFIRMED. Read probe44 bits[8:6] (haltReason)
#      to see WHICH producer fired (0=NONE is impossible if coreHalted==1;
#      1=DCACHE_DIAG, 2=FS_XLATE, 3=RESET_VECTOR, 4=ARBITER_WEDGE). If
#      haltReason==1 (DCACHE_DIAG), read bits[12:10] (dcDiagFaultKind) --
#      kind=3 (CPUSH maintenance writeback) directly confirms Part 38's own
#      unconfirmed side finding; read probe45 (diagFaultAddr) for the exact
#      faulting physical address and bits[14:13] (dcDiagFaultResp) for the
#      raw AXI response code (2=SLVERR, 3=DECERR). Cross-reference
#      diagFaultAddr against the SoC's L2C/DRAM timing docs
#      (l2c_ctrl.v/l2c.v comments, AXI reset absorber logic) for a
#      plausible root cause BEFORE writing any RTL fix.
#   4. Regardless of the coreHalted verdict: since coreHalted is STICKY
#      (RegInit(False), no self-clearing path short of a real CPU reset),
#      a `coreHalted==1` reading does NOT by itself prove the CPUSHL at
#      0x40887126 caused it -- it only proves the core is ALREADY halted by
#      the time this capture's window starts. If probe45/haltReason point
#      at an address/kind that has NOTHING to do with cache-maintenance
#      traffic near this PC, treat that as a genuine, separate finding
#      (some EARLIER boot activity halted the core, and 0x40887126 is
#      merely the last PC the ROB head happened to display before
#      `headReady` went permanently false) rather than forcing the
#      CPUSHL-writeback-error narrative onto data that does not support it.
#
# METHOD: same two-phase undisturbed-boot discipline as every prior round
# (NOT halt-based). PASSIVE ILA trigger arming does NOT interact with CPU
# execution (Part 40's armed-breakpoint finding does not apply). Given the
# hang is already independently proven PERMANENT (Part 36 S2.2, static
# across 90+ real seconds) AND Part 53 showed the board can be RE-ARMED
# live against an ALREADY-frozen CPU with zero additional cold-boot risk --
# prefer that technique whenever a board from a prior trial is still
# sitting at 0x40887126 (check with a `halt-status` PC read FIRST). Only
# fall back to a fresh cold-boot + reset if no board is already stuck there.
# TRIGGER_POSITION kept SMALL (128/4096) for the same reason as round 6 --
# the hang is proven static, so this window is really testing "is the
# packed state ALSO static within the ILA's own ~41us window", itself
# informative.
#
# **`wait_on_hw_ila -timeout N` is in MINUTES, not seconds** -- Part 53
# self-caught a false "TRIGGERED" from misreading this as seconds. Keep the
# Vivado-side timeout SHORT (this script's default 0.05 min = 3s) and rely
# on the caller's OWN `pc_live` polling (which observes 0x40887126 directly,
# essentially proof-positive the hardware trigger ALSO already latched,
# since the polling has gaps but the ILA comparator does not) as the
# primary signal, with a post-hoc `grep -q 40887126` on the written CSV as
# the final gate before ever declaring success.
#
# Usage (interactively, via jtag_repl.tcl's `tcl` passthrough, in the SAME
# session that already holds hw_target open):
#   tcl set ::ila0 [lindex [get_hw_ilas -of_objects $::hw_dev] 0]
#   tcl set ::p_pc0 [get_hw_probes dbg_ila_rob_pc_w -of_objects $::ila0]
#   tcl set_property CONTROL.TRIGGER_POSITION 128 $::ila0
#   tcl set_property CONTROL.TRIGGER_CONDITION AND $::ila0
#   tcl set_property TRIGGER_COMPARE_VALUE eq32'h40887126 $::p_pc0
#   tcl run_hw_ila $::ila0
#   [reset, or just wait if re-arming against an already-frozen CPU]
#   tcl wait_on_hw_ila -timeout 0.05 $::ila0
#   tcl upload_hw_ila_data $::ila0; set ::d [get_hw_ila_data -of_objects $::ila0]; write_hw_ila_data -csv_file /tmp/ila_robplugin_headready_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process -- only if NOT
# already attached via jtag_repl.tcl, to avoid a second contending JTAG
# connection):
#   vivado -mode batch -nojournal -nolog -source synth/ila_robplugin_headready_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_robplugin_headready_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set do_reset    [expr {[llength $argv] >= 3 ? [lindex $argv 2] : "reset"}]
set timeout_min [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 0.05}]

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
    puts "=== NOT resetting (noreset) -- assumes the board is ALREADY stuck at 0x40887126 (re-arm-against-live-hang technique, Part 53 S6) ==="
}

puts "=== Waiting up to ${timeout_min} min for trigger (NOTE: MINUTES, not seconds -- Part 53's own self-caught methodology bug) ==="
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

catch {write_hw_ila_data -csv_file /tmp/ila_robplugin_headready_capture.csv -force $data}
puts "=== Wrote /tmp/ila_robplugin_headready_capture.csv (if supported) ==="

puts "=== DONE ==="
