# synth/ila_0db0_pretrigger_capture.tcl — same trigger recipe as
# synth/ila_0db0_gate_capture.tcl (Part 19 of
# docs/BUG_calibration_word_misplaced_0d00.md), but with TRIGGER_POSITION
# moved to the END of the 4096-sample ILA depth instead of the start, so the
# capture shows PRE-trigger PC HISTORY leading up to 0x40800284 instead of
# what happens after it. Part 19's capture had ~4000 post-trigger samples
# and almost no pre-trigger context (TRIGGER_POSITION=50); this script sets
# TRIGGER_POSITION=4044, giving ~4044 pre-trigger samples and ~52
# post-trigger samples (enough to re-confirm the divergence itself, per
# Part 19, while making room for the history that answers "what did cpu040
# do in the ~40us immediately before this bsrw").
#
# See docs/BUG_calibration_word_misplaced_0d00.md Part 22 for the capture
# results and analysis. Same bitstream as Part 19
# (build/vivado_ila_irq_investigation/fpga_top.bit, build_id 0x77c93ead,
# 100MHz ENABLE_ILA=1 CPU=m68k040), same dbg_ila_rob_pc_w trigger probe
# (and the same richer 2026-08-28 packed-probe set that build carries —
# ila_cdb0/1/2_pack, ila_exc_summary, ila_via1_snap, mig_cal_done, etc. —
# see the CSV header of any capture for the full list), no rebuild needed.
#
# METHOD (identical two-phase discipline as ila_0db0_gate_capture.tcl,
# see that script's header for the full write-up):
#
# Phase 1 -- confirm reachability first via jtag_repl.tcl's own
# `reset-and-break-pc <pc> 0 <wait_ms>` (not this script) if you want to
# re-verify 0x40800260/0x40800284 are still reached fresh on the current
# board/SD-card state before spending ILA time.
#
# Phase 2 -- THIS script's trigger: arm BEFORE a fresh plain `reset`
# (pulse, NOT reset-and-break-pc/halt-based) so the CPU reaches 0x40800284
# completely undisturbed. A halt-based construction is a KNOWN CONFOUND
# (Part 10's methodology note, re-confirmed by Part 19's own retracted
# Capture 1) — real-time-paced peripheral events (VIA1 ticks etc.)
# accumulate during an artificial halt and produce a misleading capture.
#
# Usage (interactively, via jtag_repl.tcl's `tcl` passthrough, in the SAME
# session that already holds hw_target open):
#   tcl set ::ila0 [lindex [get_hw_ilas -of_objects $::hw_dev] 0]
#   tcl set ::p_pc0 [get_hw_probes dbg_ila_rob_pc_w -of_objects $::ila0]
#   tcl set_property CONTROL.TRIGGER_POSITION 4044 $::ila0
#   tcl set_property CONTROL.TRIGGER_CONDITION AND $::ila0
#   tcl set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $::p_pc0
#   break-pc off
#   halt-release
#   tcl run_hw_ila $::ila0
#   reset                                    ;# plain pulse, NOT reset-and-break-pc
#   tcl wait_on_hw_ila -timeout 1.0 $::ila0
#   tcl upload_hw_ila_data $::ila0
#   tcl set ::d [get_hw_ila_data -of_objects $::ila0]
#   tcl write_hw_ila_data -csv_file /tmp/ila_0db0_pretrigger_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process, only if NOT already
# attached via jtag_repl.tcl):
#   vivado -mode batch -nojournal -nolog -source synth/ila_0db0_pretrigger_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_0db0_pretrigger_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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

# The ONLY substantive change from ila_0db0_gate_capture.tcl: TRIGGER_POSITION
# moved from 50 (near the start, mostly post-trigger) to 4044 (near the end
# of the 4096-deep buffer, mostly PRE-trigger) -- same trigger condition.
set_property CONTROL.TRIGGER_POSITION 4044 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $p_trig
puts "=== Trigger: dbg_ila_rob_pc_w == 0x40800284 (the bsrw 0x408010f0 call site), position=4044/4096 (PRETRIGGER capture) ==="

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

catch {write_hw_ila_data -csv_file /tmp/ila_0db0_pretrigger_capture.csv -force $data}
puts "=== Wrote /tmp/ila_0db0_pretrigger_capture.csv (if supported) ==="

puts "=== DONE ==="
