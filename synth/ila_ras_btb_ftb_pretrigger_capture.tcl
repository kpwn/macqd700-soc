# synth/ila_ras_btb_ftb_pretrigger_capture.tcl -- same trigger as
# ila_ras_btb_ftb_capture.tcl (dbg_ila_rob_pc_w == 0x40800284) but
# TRIGGER_POSITION near the END of the 4096-sample depth (mirrors Part 23's
# ila_0db0_pretrigger_capture.tcl), so ~4000 samples are PRE-trigger: the
# RAS/BTB/FTB history LEADING UP TO the bad prediction, not just its
# aftermath.
#
# WHY THIS EXISTS (coordinator-directed, mid-session, Part 24): a sharp new
# hypothesis -- RAS updates SPECULATIVELY at fetch time (`FetchAlignPlugin`'s
# `rasPushValid/rasPopValid := feed.fire && ...`, confirmed pre-retire) and
# is NEVER rolled back on a flush. Confirmed at the RTL level this session:
# `FullCoreSynth.scala` wires `ras.logic.invalidateAll :=
# host[IcachePlugin].logic.invalidateAll` -- the ONLY thing that ever clears
# RAS state is a full I-cache invalidate. ALL FIVE of FetchAlignPlugin's
# redirect arms (`redirect`, `resetRedirect`, `resume`, `vioRedirect`,
# `mispredictRedirect`) share one `commonRedirect(newPc, ...)` helper
# (~line 1264) that flushes `ibuf`/marks the fetch ring stale/resets
# `decodePc`/`fetchPc`/`pendingDrop`/`faultHold` -- and NEVER touches the
# RAS. So: a wrong-path speculative excursion (down ANY of these five
# redirect classes, not just a branch misprediction) that fetches through
# real call/return-shaped opcodes before being caught leaves genuine
# leftover push/pop activity in the RAS that nothing ever rolls back. This
# is a REAL, CONFIRMED gap -- see docs/BUG_calibration_word_misplaced_0d00.md
# Part 24 for the full writeup, including why source review could not
# construct a direct data-path from this corruption to the SPECIFIC
# `0x40800284` bsrw's own wrong target (its `target` in
# `BranchEuPlugin.scala` is `u1.pc + 2 + u1.branchDisp` -- pure decode-time
# fields, no RAS/register/memory dependency at all) -- but real hardware can
# show what source review cannot, and this capture is aimed exactly at that:
# does the RAS show anomalous pointer/content activity (a jump inconsistent
# with the surrounding architectural call depth) anywhere in the ~40us
# BEFORE 0x40800284 is even fetched, and does its content at any point
# match one of the already-observed wrong targets?
#
# Usage: identical two-phase natural-boot discipline as Parts 19/23/24's
# other capture (confirm halt-release/effective=0, THEN arm, THEN plain
# `reset`, then wait undisturbed) -- see ila_ras_btb_ftb_capture.tcl's own
# usage block for the exact jtag_repl.tcl `tcl` passthrough commands; only
# difference here is TRIGGER_POSITION.
#
#   vivado -mode batch -nojournal -nolog -source synth/ila_ras_btb_ftb_pretrigger_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_ras_btb_ftb_pretrigger_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 4044 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h40800284 $p_trig
puts "=== Trigger: dbg_ila_rob_pc_w == 0x40800284, position=4044/4096 (mostly PRE-trigger) ==="

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

catch {write_hw_ila_data -csv_file /tmp/ila_ras_btb_ftb_pretrigger_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ras_btb_ftb_pretrigger_capture.csv (if supported) ==="

puts "=== DONE ==="
