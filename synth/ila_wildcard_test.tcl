# synth/ila_rte_acc1_capture.tcl — arm hw_ila on take_rte_finalize
# qualified by redirect target landing in the ROM checksum region
# (task #101, 2026-07-07).
#
# This session's live-HW pc-trace bisection found the RTE-pop resume PC
# landing exactly 14 bytes past correct after an IRQ taken mid-ROM-
# checksum-loop (0x40870048-0x408701b2), skipping the interrupted
# routine's own moveml/move-sr restore and landing straight on its
# trailing rts (0x408701b2), which then bus-faults. commit.v computes
# saved_pc_w = {rte_acc[0][15:0], rte_acc[1][31:16]} at
# take_rte_finalize; the ILA v9 probe bundle adds rte_acc[1] (probe55)
# and the registered redirect_pc (probe56) alongside the existing
# take_rte_finalize (probe33) and rte_acc[0] (probe35) taps.
#
# take_rte_finalize fires on EVERY normal RTE during boot (not just the
# buggy one) -- an unqualified trigger would almost certainly catch some
# unrelated, correct RTE instead of the specific occurrence in question.
# Qualify the trigger on redirect_pc's high 16 bits == 0x4087 (the ROM
# checksum region) to filter for RTEs resuming into that specific 64KB
# window.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_rte_acc1_capture.tcl \
#       -tclargs <ltx_file> [timeout_min]

if {[llength $argv] < 1} {
    puts stderr "ERROR: usage: ila_rte_acc1_capture.tcl <ltx_file> \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set timeout_min [expr {[llength $argv] >= 2 ? [lindex $argv 1] : 10.0}]

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
    puts stderr "ERROR: no hw_ila core found on device."
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p_finalize [get_hw_probes ila_take_rte_finalize -of_objects $ila]
set p_redirect [get_hw_probes ila_redirect_pc        -of_objects $ila]
set p_rte_acc0 [get_hw_probes ila_rte_acc0           -of_objects $ila]
set p_rte_acc1 [get_hw_probes ila_rte_acc1           -of_objects $ila]
set p_rob_pc   [get_hw_probes dbg_ila_rob_pc_w        -of_objects $ila]

foreach {name probe} [list ila_take_rte_finalize $p_finalize \
                            ila_redirect_pc        $p_redirect \
                            ila_rte_acc0           $p_rte_acc0 \
                            ila_rte_acc1           $p_rte_acc1 \
                            dbg_ila_rob_pc_w        $p_rob_pc] {
    if {[llength $probe] == 0} {
        puts stderr "ERROR: expected probe $name not found."
        puts stderr "Available probes:"
        foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
        exit 3
    }
}

set_property CONTROL.TRIGGER_POSITION 512 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila

# Sanity pass (2026-07-07, earlier this run) confirmed plumbing works
# AND revealed saved_pc_w = {rte_acc[0][15:0], rob_br_target[31:16]} --
# NOT rte_acc[1] as originally hypothesized (commit.v:2964-2965 bypasses
# directly from the live rob_br_target on the finalize cycle itself,
# since finalize IS the word_idx=1 retirement). rte_acc[1] register is
# written but never read back for this computation. Reframed focus:
# check whether rob_br_target itself is wrong at the exact finalize
# cycle (already-probed via probe38, cross-referenced with cmpl1_stale/
# rob_head_idx from v7 probes35-43).
set_property TRIGGER_COMPARE_VALUE eq1'b1 $p_finalize
set_property TRIGGER_COMPARE_VALUE {eq32'h4084xxxx} $p_redirect

puts "=== Trigger: take_rte_finalize==1 AND redirect_pc[31:16]==0x4084 (SYNTAX TEST) ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

# Force a fresh cold reset directly via hw_axi so the boot has a full
# pass to naturally reach the checksum-loop IRQ window from a clean
# start (the CPU had already been free-running well past this point
# before the ILA armed -- an unqualified wait would very likely time
# out against the terminal ADB-stall loop instead).  Mirrors
# tools/jtag_repl.tcl's unified_reset: pulse DBG_CONTROL.cold_reset_pulse
# (bit 5) at DBG_BASE+OFF_CONTROL (0x50900000+0x008), no hold.
set axi [lindex [get_hw_axis -of_objects $dev] 0]
if {$axi eq ""} {
    puts stderr "ERROR: no hw_axi master found -- cannot force reset."
    exit 5
}
puts "=== hw_axi master: $axi ==="

proc dbg_wr_raw {axi addr data} {
    create_hw_axi_txn -quiet -force _w $axi -type WRITE \
        -address [format %08X $addr] -data [format %08X $data]
    run_hw_axi -quiet _w
}

# Plain cold_reset_pulse (v4 attempt) timed out twice (10min, 5min) --
# this session established that a PLAIN reset statistically favors the
# "good" boot path that never reaches the checksum-loop excursion at
# all (4/4 clean via JTAG REPL earlier). The ONLY mechanism that
# reliably reproduced the bug (3/3) was `reset-halt-exc 2` (arm
# halt-on-vector-2 from a fresh reset) -- replicate that exact sequence
# here via raw hw_axi writes (mirrors tools/jtag_repl.tcl's
# reset_and_halt_exc proc) since the REPL can't run concurrently with
# this ILA capture (JTAG session conflict).
#   DBG_BASE=0x50900000, OFF_CONTROL=0x008, OFF_HALT_CTL=0x03C,
#   OFF_HALT_EXC_MASK=0x060 (8x32b lanes), CTL_COLD_RESET_HOLD=0x10,
#   CTL_COLD_RESET_PULSE=0x20, HALT_CLEAR=0x4, HALT_EXC_EN=0x40.
#   vec=2 (bus error): lane=(2>>5)&7=0, bit=2&0x1f=2, mask=1<<2=0x4.
set DBG_BASE 0x50900000
dbg_wr_raw $axi [expr {$DBG_BASE + 0x008}] [expr {0x10 | 0x20}]
for {set lane 0} {$lane < 8} {incr lane} {
    dbg_wr_raw $axi [expr {$DBG_BASE + 0x060 + $lane*4}] 0
}
dbg_wr_raw $axi [expr {$DBG_BASE + 0x060}] 0x4
dbg_wr_raw $axi [expr {$DBG_BASE + 0x03C}] [expr {0x40 | 0x4}]
dbg_wr_raw $axi [expr {$DBG_BASE + 0x008}] 0
puts "=== Forced reset-halt-exc(vec=2) sequence at [clock format [clock seconds]] ==="

puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

catch {upload_hw_ila_data $ila} uerr
puts "=== upload_hw_ila_data result: $uerr ==="

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: '$data' ==="

if {$data ne ""} {
    catch {write_hw_ila_data -csv_file /tmp/ila_rte_acc1_capture.csv -force $data} werr
    puts "=== write_hw_ila_data result: $werr ==="
}
puts "=== DONE ==="
