# synth/ila_rte_acc1_capture.tcl — arm hw_ila on take_rte_finalize,
# iteratively re-arming past unrelated RTEs until redirect_pc lands in
# the ROM checksum region (task #101, 2026-07-07).
#
# This session's live-HW pc-trace bisection found the RTE-pop resume PC
# landing exactly 14 bytes past correct after an IRQ taken mid-ROM-
# checksum-loop (0x40870048-0x408701b2), skipping the interrupted
# routine's own moveml/move-sr restore and landing straight on its
# trailing rts (0x408701b2), which then bus-faults.
#
# A sanity-check capture this session confirmed the ILA plumbing works
# AND revealed the real mechanism: commit.v:2964-2965 computes
#   saved_pc_w = {rte_acc[0][15:0], rob_br_target[31:16]}
# -- NOT rte_acc[1] as originally hypothesized (rte_acc[1] is written
# but never read back for this; the low 16 bits bypass directly from
# the live rob_br_target on the finalize cycle itself, since finalize
# IS the word_idx=1 retirement). So the real question is whether
# rob_br_target is wrong at the exact finalize cycle -- cross-reference
# with the existing v7 cmpl1_en/cmpl1_tag/cmpl1_alloc_epoch/cmpl1_stale/
# rob_head_idx probes (35-43) to see if a stale/dropped LSU completion
# corrupted it.
#
# Vivado's multi-probe wildcard trigger syntax (both 'b with x-bits and
# 'h with x-nibbles) failed to fire even against a value KNOWN to occur
# within seconds -- rather than keep fighting exact trigger-condition
# API syntax, this script re-arms an UNQUALIFIED take_rte_finalize==1
# trigger in a loop (confirmed reliable), checking redirect_pc after
# each hit and only stopping once it lands in 0x4087xxxx.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_rte_acc1_capture.tcl \
#       -tclargs <ltx_file> [max_iters]

if {[llength $argv] < 1} {
    puts stderr "ERROR: usage: ila_rte_acc1_capture.tcl <ltx_file> \[max_iters\]"
    exit 2
}

set ltx_file  [lindex $argv 0]
set max_iters [expr {[llength $argv] >= 2 ? [lindex $argv 1] : 500}]

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
set p_rob_br   [get_hw_probes ila_rob_br_target      -of_objects $ila]
set p_cmpl1_stale [get_hw_probes ila_cmpl1_stale     -of_objects $ila]
set p_rob_pc   [get_hw_probes dbg_ila_rob_pc_w        -of_objects $ila]

foreach {name probe} [list ila_take_rte_finalize $p_finalize \
                            ila_redirect_pc        $p_redirect \
                            ila_rte_acc0           $p_rte_acc0 \
                            ila_rob_br_target      $p_rob_br \
                            ila_cmpl1_stale        $p_cmpl1_stale \
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
set_property TRIGGER_COMPARE_VALUE eq1'b1 $p_finalize
puts "=== Trigger: take_rte_finalize==1 (unqualified, re-armed in a loop) ==="

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

# Force reset-halt-exc(vec=2) -- the ONLY mechanism that reliably
# reproduced the bug (3/3 via JTAG REPL earlier this session). A plain
# reset statistically favors the "good" path that never reaches the
# checksum-loop excursion at all (timed out twice against it).
set DBG_BASE 0x50900000
dbg_wr_raw $axi [expr {$DBG_BASE + 0x008}] [expr {0x10 | 0x20}]
# CRITICAL: tools/jtag_repl.tcl's unified_reset does `after 800` here --
# "Allow time for the 1024-cy stretcher + boot_fsm SD->DDR copy." A
# first attempt without this delay produced ZERO take_rte_finalize
# events in 4+ minutes (vs ~3s normally) -- the CPU was very likely
# still held/unstable when the mask-config writes and release fired.
after 800
for {set lane 0} {$lane < 8} {incr lane} {
    dbg_wr_raw $axi [expr {$DBG_BASE + 0x060 + $lane*4}] 0
}
dbg_wr_raw $axi [expr {$DBG_BASE + 0x060}] 0x4
dbg_wr_raw $axi [expr {$DBG_BASE + 0x03C}] [expr {0x40 | 0x4}]
dbg_wr_raw $axi [expr {$DBG_BASE + 0x008}] 0
puts "=== Forced reset-halt-exc(vec=2) sequence at [clock format [clock seconds]] ==="

# Arm the very first capture now, immediately after the reset sequence,
# so we don't miss early RTEs.
run_hw_ila $ila
puts "=== ARMED (iter 0) at [clock format [clock seconds]] ==="

# Read back the trigger-sample's redirect_pc column from a just-written
# CSV via plain Tcl file I/O -- avoids uncertain hw_ila_data probe-read
# API guessing, relies only on the CONFIRMED-WORKING write_hw_ila_data
# + CSV mechanism from the earlier sanity-check capture this session.
proc read_trigger_redirect_pc {csv_path} {
    set fh [open $csv_path r]
    set header_line [gets $fh]
    close $fh
    set header [split $header_line ","]
    set col -1
    for {set i 0} {$i < [llength $header]} {incr i} {
        if {[lindex $header $i] eq "ila_redirect_pc\[31:0\]"} { set col $i; break }
    }
    if {$col < 0} { return "" }
    set trig_col -1
    for {set i 0} {$i < [llength $header]} {incr i} {
        if {[lindex $header $i] eq "TRIGGER"} { set trig_col $i; break }
    }
    set fh [open $csv_path r]
    gets $fh; gets $fh
    set result ""
    while {[gets $fh line] >= 0} {
        set parts [split $line ","]
        if {$trig_col >= 0 && [lindex $parts $trig_col] eq "1"} {
            set result [lindex $parts $col]
            break
        }
    }
    close $fh
    return $result
}

set found 0
for {set iter 0} {$iter < $max_iters} {incr iter} {
    wait_on_hw_ila -timeout 0.3 $ila
    if {[catch {upload_hw_ila_data $ila} uerr]} {
        if {$iter % 10 == 0} { puts "=== iter=$iter: no trigger yet (upload threw) ===" }
        run_hw_ila $ila
        continue
    }
    set data [get_hw_ila_data -of_objects $ila]
    if {$data eq ""} {
        if {$iter % 10 == 0} { puts "=== iter=$iter: no trigger yet (empty data) ===" }
        run_hw_ila $ila
        continue
    }
    if {[catch {write_hw_ila_data -csv_file /tmp/ila_rte_acc1_iter.csv -force $data} werr]} {
        run_hw_ila $ila
        continue
    }
    # Distinguish a real (if uninteresting) capture from a bare
    # timeout-with-no-data CSV (header + radix line only, 2 lines).
    set fh [open /tmp/ila_rte_acc1_iter.csv r]
    set nlines 0
    while {[gets $fh dummy] >= 0} { incr nlines }
    close $fh
    if {$nlines <= 2} {
        if {$iter % 10 == 0} { puts "=== iter=$iter: no trigger yet (2-line/empty CSV) ===" }
        run_hw_ila $ila
        continue
    }
    set rp [read_trigger_redirect_pc /tmp/ila_rte_acc1_iter.csv]
    puts "=== iter=$iter (nlines=$nlines) redirect_pc=$rp ==="
    if {[string length $rp] == 8 && [string range $rp 0 3] eq "4087"} {
        puts "=== MATCH at iter=$iter -- checksum-region RTE found ==="
        set found 1
        file copy -force /tmp/ila_rte_acc1_iter.csv /tmp/ila_rte_acc1_capture.csv
        puts "=== Wrote /tmp/ila_rte_acc1_capture.csv ==="
        break
    }
    run_hw_ila $ila
}

if {!$found} {
    puts "=== RESULT: NOT FOUND after $max_iters iterations ==="
    catch {file copy -force /tmp/ila_rte_acc1_iter.csv /tmp/ila_rte_acc1_capture_last.csv}
    puts "=== Wrote /tmp/ila_rte_acc1_capture_last.csv (last non-matching capture) ==="
    exit 4
}

puts "=== DONE ==="
