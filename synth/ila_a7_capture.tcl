# synth/ila_a7_capture.tcl — arm hw_ila to bracket a take_rte_finalize
# (probe33) event at a specific exc_count (probe12), for the live A7/RTE
# investigation documented in docs/ila_a7_drift_probes.md.
#
# Rationale: the HW bus-error wedge occurs when an RTS instruction's own
# stack-pop LOAD finds A7 == the RTS's own PC (0x408701b2).  The
# exception ring shows the immediately-preceding ring entry is an
# autovector level-1 IRQ (vec=0x19) taken AT that same PC, and the live
# bus-error's exc_count reads 0x12d2.  Working theory: the RTE that
# returns from that vec=0x19 handler (exc_count still 0x12d1, since
# exc_count increments on exception ENTRY not RTE) is where A7 gets
# corrupted.  This script triggers on take_rte_finalize while exc_count
# == ONE candidate value per invocation, issuing a fresh cold reset (via
# the jtag_repl.tcl REPL's `reset` command over jcmd.sh) after arming.
#
# NOTE on Vivado hw_ila API quirks discovered empirically (2025.2):
#   - hw_ila objects have NO "STATUS" property (unlike some doc examples).
#   - wait_on_hw_ila -timeout takes MINUTES, not seconds.
#   - wait_on_hw_ila does not raise a Tcl error on timeout -- it just
#     returns.  On timeout it auto-stops the core and prints
#     "WARNING: [Labtools 27-157] hw_ila [...] stopped. No data to
#     upload." -- that WARNING text is the only reliable signal that the
#     capture did NOT trigger.  We grep our own transcript for it since
#     there's no clean property to test.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_a7_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> <candidate_hex_or_any> <timeout_min>
#
# candidate_hex_or_any: a hex string like "12d1" (may include a trailing
#   'x' nibble wildcard, e.g. "12dx"), or the literal word "any" to trigger
#   on take_rte_finalize alone with no exc_count qualifier.
#
# Assumes the bitstream is ALREADY programmed and booting (via a
# separate jtag_repl.tcl session against the same target) with debug
# probes matching ltx_file.  This script does NOT reprogram — it only
# attaches, arms, fires ONE reset through jcmd_path, waits, and dumps.

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_a7_capture.tcl <ltx_file> <jcmd_path> \[candidate_hex|any\] \[timeout_min\]"
    exit 2
}

set ltx_file   [lindex $argv 0]
set jcmd_path  [lindex $argv 1]
set candidate  [expr {[llength $argv] >= 3 ? [lindex $argv 2] : "12d1"}]
set timeout_min [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 1.5}]

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
    puts stderr "ERROR: no hw_ila core found on device -- was ENABLE_ILA=1 used, and is fpga_top.ltx current?"
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p33 [get_hw_probes ila_take_rte_finalize -of_objects $ila]
set p12 [get_hw_probes ila_exc_count_lo -of_objects $ila]
if {[llength $p33] == 0 || [llength $p12] == 0} {
    puts stderr "ERROR: expected probes (ila_take_rte_finalize, ila_exc_count_lo) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 2048 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila

set_property TRIGGER_COMPARE_VALUE eq1'b1 $p33
if {$candidate eq "any"} {
    set_property TRIGGER_COMPARE_VALUE eq16'hxxxx $p12
    puts "=== Trigger: probe33(take_rte_finalize)==1, exc_count UNGATED ==="
} else {
    set_property TRIGGER_COMPARE_VALUE eq16'h$candidate $p12
    puts "=== Trigger: probe33(take_rte_finalize)==1 AND exc_count==0x$candidate ==="
}

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

after 300
puts "=== Firing reset via: $jcmd_path reset ==="
if {[catch {exec $jcmd_path reset} reset_out]} {
    puts "WARN: reset exec returned error/nonzero: $reset_out"
} else {
    puts "reset output:\n$reset_out"
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

# Sanity: does the data object actually have samples?  If the core never
# triggered, Vivado prints "stopped. No data to upload." just above this
# point (see NOTE) and upload_hw_ila_data still "succeeds" but yields an
# effectively empty/garbage data object.  We rely on the caller grepping
# this transcript for that WARNING text as the authoritative signal;
# still attempt the dump here in case it IS valid, so the caller doesn't
# need a second round-trip.
puts "=== RESULT: ATTEMPTING_DUMP (caller must grep for 'No data to upload' WARNING above to confirm real vs empty) ==="

catch {write_hw_ila_data -csv_file /tmp/ila_a7_capture.csv -force $data}
puts "=== Wrote /tmp/ila_a7_capture.csv (if supported) ==="

puts "=== FULL CAPTURE TABLE ==="
display_hw_ila_data $data

puts "=== DONE ==="
