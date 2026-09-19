# p141 -- capture the cycles BEFORE the 0x40806b68 walker stall.
#
#   vivado -mode batch -nojournal -nolog \
#       -source synth/ila_p141_walker_stall_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]
#
# Or, preferred, paste the `tcl ...` lines into a live jtag_repl session so a
# second JTAG connection does not contend with it (jtag_repl.tcl:5717-5725
# exposes Vivado's ILA API verbatim through its `tcl` passthrough).
#
# ── THE TRIGGER, AND WHY IT IS THE RIGHT ONE ────────────────────────────────
# The wedge freezes retire at a BIT-IDENTICAL count on every boot: 0x0543222C,
# measured 6/6 on p138c and 6/6 on p140. probe50 carries that exact counter
# (RobPlugin's DebugCommitService.macroCount[31:0] -- the same one OFF_INST_LO
# reads), so an equality match fires on the precise cycle the machine stops.
# No PC trigger, no heuristic, no unit conversion.
#
# TRIGGER_POSITION IS LATE ON PURPOSE. The stall is structural and persists
# unchanged once entered, so post-trigger samples are worthless -- they record
# a machine doing nothing. Position 3968 of 4096 keeps ~39.7 us of the APPROACH
# and only ~1.3 us after. The approach is the entire point: a frozen read (the
# CSRs at 0x5090101C..0x50901028, which this same bitstream also carries)
# already gives the terminal state, and it CANNOT distinguish
#   (a) a quiesce term that never cleared, from
#   (b) one that cleared and was RE-ARMED by a grant hand-over, from
#   (c) a hand-over that landed one cycle late relative to drain entry.
# Those imply different fixes and have identical terminal state. Only the
# sequence separates them.
#
# ── PROBE MAP (repurposed slots; see rtl/soc/fpga_top_debug_ctrl.vh) ────────
#   probe20  dbg_ila_cdb0_data_w        stallDc      17 dcIdleForMaint terms
#   probe21  dbg_ila_cdb1_data_w        stallGrant   port ownership + grants
#   probe22  dbg_ila_a7_writeback_val_w stallExc     ExceptionUnit FSM one-hot
#   probe45  ila_diag_fault_addr        stallWalk    DTLB [15:0], ITLB [31:16]
#   probe50  ila_ic_mshr_snap           macroCountLo THE TRIGGER
#
# Bit layouts are documented in cpu040
# docs/superpowers/specs/2026-09-05-p141-measure-the-walker-stall-on-silicon.md
# and are IDENTICAL to the live CSRs, so tools/p141_decode_stall.sh decodes a
# CSV row unchanged.
#
# POLARITY TRAP, restated because getting it wrong inverts the conclusion:
# stallDc's terms are NOT all the same polarity. Bits [13:0] block when SET;
# bits [17:14] (stAwDone/stWDone/evictAwDone/evictWDone) block when CLEAR.

set ltx_file   [lindex $argv 0]
set jcmd_path  [lindex $argv 1]
set do_reset   [expr {[llength $argv] > 2 ? [lindex $argv 2] : "reset"}]
set timeout_min [expr {[llength $argv] > 3 ? [lindex $argv 3] : 8}]

# The measured freeze value. Override on the command line if a future build
# lands on a different count -- but DO NOT guess it: read it from
# OFF_INST_LO (0x50901008) on a wedged boot first.
set FREEZE_COUNT [expr {[info exists ::env(P141_FREEZE_COUNT)]
                        ? $::env(P141_FREEZE_COUNT) : 0x0543222C}]

open_hw_manager
connect_hw_server -allow_non_jtag
current_hw_target [lindex [get_hw_targets] 0]
open_hw_target
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx_file $dev
refresh_hw_device -update_hw_probes true $dev

set ilas [get_hw_ilas -of_objects $dev]
if {[llength $ilas] == 0} {
    puts "ERROR: no ILA core on the device."
    puts "       Was this bitstream built with ENABLE_ILA=1 CPU=m68k040,"
    puts "       and is fpga_top.ltx the one from build/vivado_ila?"
    exit 3
}
set ila [lindex $ilas 0]

set p_trig  [get_hw_probes ila_ic_mshr_snap       -of_objects $ila]
set p_dc    [get_hw_probes dbg_ila_cdb0_data_w    -of_objects $ila]
set p_grant [get_hw_probes dbg_ila_cdb1_data_w    -of_objects $ila]
set p_exc   [get_hw_probes dbg_ila_a7_writeback_val_w -of_objects $ila]
set p_walk  [get_hw_probes ila_diag_fault_addr    -of_objects $ila]

# Keep the approach, discard the (static) aftermath.
set_property CONTROL.TRIGGER_POSITION 3968 $ila
set_property CONTROL.TRIGGER_CONDITION AND  $ila
set_property TRIGGER_COMPARE_VALUE \
    [format "eq32'h%08X" $FREEZE_COUNT] $p_trig

puts "=== armed: trigger on macroCountLo == [format 0x%08X $FREEZE_COUNT] ==="
puts "=== trigger position 3968/4096 (keeps the approach, not the aftermath) ==="
run_hw_ila $ila

if {$do_reset eq "reset"} {
    puts "=== pulsing board reset via $jcmd_path ==="
    catch {exec $jcmd_path reset} r
    puts $r
}

puts "=== waiting up to ${timeout_min} min for the trigger ==="
if {[catch {wait_on_hw_ila -timeout $timeout_min $ila} err]} {
    puts "ERROR: wait_on_hw_ila failed: $err"
}
upload_hw_ila_data $ila
set d [get_hw_ila_data -of_objects $ila]
set status [get_property CONTROL.STATUS $ila]
puts "=== ILA status: $status ==="
if {[string match -nocase "*idle*" $status] == 0 &&
    [string match -nocase "*full*" $status] == 0} {
    puts "WARNING: the ILA may not have triggered; the CSV may be pre-trigger junk."
    puts "         Cross-check against OFF_INST_LO -- if retire is NOT frozen at"
    puts "         [format 0x%08X $FREEZE_COUNT] this boot simply did not wedge,"
    puts "         which is itself a finding (perturbation sensitivity)."
}
write_hw_ila_data -csv_file /tmp/p141_walker_stall.csv -force $d
puts "=== wrote /tmp/p141_walker_stall.csv ==="
puts "=== decode a row with: tools/p141_decode_stall.sh <dc> <grant> <exc> <walk> ==="
