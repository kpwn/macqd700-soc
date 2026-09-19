# synth/ila_0db0_gate_capture.tcl — capture the actual ROB-head PC sequence
# cpu040 executes across the boot-device-driver-open call chain's real entry
# point, for the 2026-08-28 follow-up session on
# docs/BUG_calibration_word_misplaced_0d00.md Parts 15/17/18 (the `$0DB0`
# sentinel that never gets stamped, gating the operator-monitor-loop hang).
#
# BACKGROUND: Parts 15/17/18 traced the skipped call chain 5 levels deep
# (0x40800bf0 <- 0x40801000 <- 0x4080114c <- 0x408010f0 <- 0x40800284) and
# confirmed via break-pc that EVERY PC in that chain is never reached on real
# cpu040 hardware -- but the true fork point (what decides not to enter it)
# was never located. This capture answers that directly: is the call site
# itself (0x40800284: `bsrw 0x408010f0`) ever reached, and if so, what PC
# does the CPU actually execute next?
#
# REUSES the same ILA infrastructure/probe mapping as
# synth/ila_rob_irq_recognition_capture.tcl and synth/ila_post_release_capture.tcl
# (see their headers for the full corrected packed-probe mapping and the
# Chipscope 16-213 tie-off-fix background) -- same bitstream
# (build/vivado_ila_irq_investigation/fpga_top.bit, build_id 0x77c93ead, a
# 100MHz ENABLE_ILA=1 CPU=m68k040 build), same `dbg_ila_rob_pc_w` 32-bit
# standalone probe (RobPlugin's ROB-head PC / p0.pc), no probe-set changes
# and no rebuild needed. Verified via `git log --since <ila build time>` that
# no RTL affecting the CPU/SoC (beyond the already-known, orthogonal, and at
# the time not-yet-landed cpuBootThrottleEn throttle feature) changed between
# that build and this session, so behavior is representative of current HEAD.
#
# METHOD (two-phase, both confirmed live 2026-08-28):
#
# Phase 1 -- confirm reachability first (cheap, via jtag_repl.tcl, not this
# script): `reset-and-break-pc 0x40800260 0 <wait_ms>` and
# `reset-and-break-pc 0x40800284 0 <wait_ms>` both FIRE on real hardware
# (effective=1, hit=<pc>) -- NEW finding, neither had been directly confirmed
# reached before this session (only the *callee* 0x408010f0 had been
# confirmed NEVER reached). Wait times needed were much longer than the
# jtag_repl default (0x40800284 took 132224ms real wall-clock in one run) --
# budget several minutes, not the ~2s default.
#
# Phase 2 -- THIS script's trigger: arm BEFORE a fresh `reset` (plain pulse,
# not reset-and-break-pc/halt-based -- a halt-then-release construction was
# tried first and is a known confound, see docs Part 19 for why: pending
# real-time-paced events accumulate during an artificial multi-minute halt
# and produce a misleading capture) so the CPU reaches 0x40800284 completely
# undisturbed, exactly as Part 10 of this doc recommends ("reproduce the
# original observation methodology first...without any breakpoint/halt").
#
# TRIGGER_POSITION is small (near the start of the 4096-sample depth) so
# ~4000 samples are POST-trigger -- this captures forward from the exact
# moment 0x40800284 is fetched, which is what answers the open question.
#
# Usage (interactively, via jtag_repl.tcl's `tcl` passthrough command, in the
# SAME session that already holds hw_target open -- avoids a second,
# contending JTAG connection, see docs/ila_a7_drift_probes.md's `tcl`
# passthrough note):
#   tcl set ::ila0 [lindex [get_hw_ilas -of_objects $::hw_dev] 0]
#   tcl set ::p_pc0 [get_hw_probes dbg_ila_rob_pc_w -of_objects $::ila0]
#   tcl set_property CONTROL.TRIGGER_POSITION 50 $::ila0
#   tcl set_property CONTROL.TRIGGER_CONDITION AND $::ila0
#   tcl set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $::p_pc0
#   tcl run_hw_ila $::ila0
#   reset                                    ;# plain pulse, NOT reset-and-break-pc
#   tcl wait_on_hw_ila -timeout 2 $::ila0
#   tcl upload_hw_ila_data $::ila0; set ::d [get_hw_ila_data -of_objects $::ila0]; write_hw_ila_data -csv_file /tmp/ila_0db0_gate_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process, same pattern as the
# other ila_*_capture.tcl scripts in this directory -- only use this if NOT
# already attached via jtag_repl.tcl, to avoid a second contending JTAG
# connection):
#   vivado -mode batch -nojournal -nolog -source synth/ila_0db0_gate_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_0db0_gate_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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

catch {write_hw_ila_data -csv_file /tmp/ila_0db0_gate_capture.csv -force $data}
puts "=== Wrote /tmp/ila_0db0_gate_capture.csv (if supported) ==="

puts "=== DONE ==="
