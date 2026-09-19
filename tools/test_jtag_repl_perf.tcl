#!/usr/bin/env tclsh
##
## Self-checking test for the `perf` / `perf-clear` REPL commands in jtag_repl.tcl.
##
## WHY THIS EXISTS.  `perf` is a host-side renderer for a hardware register block, and a
## renderer that has never been run is the same hazard as a register that has never been
## read: it looks finished and it is not.  Writing this test immediately found that
## `perf_report` referenced `$::OFF_INST_LO`, a variable jtag_repl.tcl had NEVER defined
## -- the command would have thrown `can't read "::OFF_INST_LO": no such variable` the
## first time anyone ran it on the board, after a full bitstream build.
##
## It sources the real jtag_repl.tcl procs against a MODELLED register block: `rd`/`dbg_wr`
## are replaced with an array that behaves the way the RTL does (bit 1 is the RUN level,
## bit 0 zeroes every counter).  No hardware, no Vivado.
##
## Run: tclsh tools/test_jtag_repl_perf.tcl        (exit 0 = all checks pass)

set HERE [file dirname [file normalize [info script]]]
set REPL [file join $HERE jtag_repl.tcl]

# ---- extract the perf offsets + procs from the REPL, verbatim ------------------
set fh [open $REPL r]; set src [read $fh]; close $fh
set offStart [string first "# ---- windowed performance counters (core commit: feat/perf-counters) ----------" $src]
set offEnd   [string first "set OFF_PC_TRACE_BASE" $src]
set prStart  [string first "# ══ windowed performance counters " $src]
set prEnd    [string first "proc dbg_caps_report \{\} \{" $src]
foreach {n v} [list offStart $offStart offEnd $offEnd prStart $prStart prEnd $prEnd] {
    if {$v < 0} { puts "FAIL  could not locate '$n' in jtag_repl.tcl -- the extraction\
markers moved; fix this test rather than deleting it"; exit 1 }
}
set extracted "[string range $src $offStart [expr {$offEnd-1}]]\n[string range $src $prStart [expr {$prEnd-1}]]"

# ---- the modelled register block ----------------------------------------------
set ::DBG_BASE 0x50900000
array set ::MEM {}
set ::WRITES {}
proc rd {addr} {
    set off [expr {$addr - $::DBG_BASE}]
    if {[info exists ::MEM($off)]} { return [format %08X $::MEM($off)] }
    return "00000000"
}
proc dbg_rd {off} { return [rd [expr {$::DBG_BASE + $off}]] }
proc rdx_or_empty {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return "" }
    return [expr {"0x$v"}]
}
proc dbg_wr {off data} {
    set off [expr {$off}]
    lappend ::WRITES [list [format 0x%X $off] [format 0x%X $data]]
    if {$off == [expr {$::OFF_PERF_CTL}]} {
        # Model the RTL: bit 1 is the RUN LEVEL (taken from the written bit, no
        # read-modify-write); bit 0 zeroes every windowed counter.
        set ::MEM($off) [expr {($::MEM($off) & ~0x2) | ($data & 0x2)}]
        if {$data & 0x1} {
            foreach k [array names ::MEM] {
                if {$k != [expr {$::OFF_PERF_CTL}]} { set ::MEM($k) 0 }
            }
        }
    }
}
eval $extracted

set ::fails 0
proc check {what cond} {
    if {$cond} { puts "PASS  $what" } else { puts "FAIL  $what"; incr ::fails }
}
proc capture {script} {
    rename puts ::_real_puts
    set ::CAP ""
    proc puts {args} { append ::CAP "[lindex $args end]\n" }
    uplevel 1 $script
    rename puts {}
    rename ::_real_puts puts
    return $::CAP
}
proc reset_mem {} { array unset ::MEM; array set ::MEM {}; set ::WRITES {} }
proc ctlWord {counters producers {run 1}} {
    return [expr {($run ? 0x2 : 0x0) | ($counters << 8) | ($producers << 16)}]
}

# ---- 1. an ABSENT block must REFUSE, not render a table of zeros ---------------
reset_mem
set out [capture {perf_report}]
check "perf REFUSES when OFF_PERF_CTL reports 0 counters" \
      [expr {[string match "*REFUSED*" $out] && ![string match "*IPC*" $out]}]
check "the refusal explains that the zeros are unsourced, not measured" \
      [string match "*read 0 because nothing drives them*" $out]
check "perf-clear ERRORS on an absent block rather than dropping the write" \
      [catch {perf_clear}]

# ---- 2. a full block renders every lane and the derived metrics ----------------
reset_mem
set ::MEM([expr {$::OFF_PERF_CTL}])          [ctlWord 12 0x1F]
set ::MEM([expr {$::OFF_PERF_CYCLE_LO}])     1000000
set ::MEM([expr {$::OFF_PERF_INST_LO}])      1500000
set ::MEM([expr {$::OFF_MISPRED_COUNT}])     3000
set ::MEM([expr {$::OFF_FLUSH_COUNT}])       3300
set ::MEM([expr {$::OFF_PERF_BRANCH}])       150000
set ::MEM([expr {$::OFF_PERF_DC_MISS}])      6000
set ::MEM([expr {$::OFF_PERF_IC_MISS}])      750
set ::MEM([expr {$::OFF_PERF_DTLB_WALK}])    300
set ::MEM([expr {$::OFF_PERF_ITLB_WALK}])    30
set ::MEM([expr {$::OFF_PERF_STALL_RETIRE}]) 250000
set ::MEM([expr {$::OFF_PERF_STALL_DC}])     90000
set ::MEM([expr {$::OFF_PERF_STALL_WALK}])   4500
set ::WRITES {}
set out [capture {perf_report}]
check "all five producer presence bits are decoded" \
      [string match "*rob:1 dcache:1 icache:1 dtlb:1 itlb:1*" $out]
check "IPC = inst/cycles = 1.5000" [string match "*IPC = 1.5000*" $out]
check "MPKI: dc-miss = 6000/1500 kinst = 4.000" [string match "*dc-miss=4.000*" $out]
check "MPKI: ic-miss = 750/1500 kinst = 0.500" [string match "*ic-miss=0.500*" $out]
check "MPKI: dtlb-walk = 300/1500 kinst = 0.200" [string match "*dtlb-walk=0.200*" $out]
check "MPKI: itlb-walk is labelled itlb, not ic" [string match "*itlb-walk=0.020*" $out]
check "mispredict rate = 3000/150000 = 2.00%" [string match "*mispredict rate = 2.00%*" $out]
check "retire-stall = 250000/1000000 = 25.00%" [string match "*retire-stall = 25.00%*" $out]
check "thousands separators are applied" [string match "*1,500,000*" $out]
check "the read FROZE and then RESUMED the counters" \
      [expr {$::WRITES eq {{0x1080 0x0} {0x1080 0x2}}}]

# ---- 3. a counter with NO producer renders `--`, never 0 -----------------------
reset_mem
set ::MEM([expr {$::OFF_PERF_CTL}])      [ctlWord 12 0x19]   ;# rob+dtlb+itlb, no caches
set ::MEM([expr {$::OFF_PERF_CYCLE_LO}]) 100000
set ::MEM([expr {$::OFF_PERF_INST_LO}])  50000
set out [capture {perf_report}]
check "an absent D-cache producer renders as -- not as 0" \
      [string match "*D-cache load misses *--*" $out]
check "an absent I-cache producer renders as -- not as 0" \
      [string match "*I-cache demand misses *--*" $out]
check "a PRESENT producer with a zero count still renders 0" \
      [string match "*DTLB table walks *0*" $out]
check "no dc-miss ratio is derived from an absent producer" \
      [expr {![string match "*dc-miss=*" $out]}]

# ---- 4. perf-clear writes the right control word and really zeroes -------------
reset_mem
set ::MEM([expr {$::OFF_PERF_CTL}])     [ctlWord 12 0x1F]
set ::MEM([expr {$::OFF_PERF_DC_MISS}]) 999
set ::WRITES {}
perf_clear
check "perf-clear writes CLEAR+RUN (0x3)" [expr {$::WRITES eq {{0x1080 0x3}}}]
check "perf-clear leaves the counters at zero" \
      [expr {$::MEM([expr {$::OFF_PERF_DC_MISS}]) == 0}]
set ::WRITES {}
perf_clear 1
check "perf-clear hold writes CLEAR with RUN=0 (0x1)" [expr {$::WRITES eq {{0x1080 0x1}}}]
check "perf-clear hold leaves the block FROZEN" \
      [expr {($::MEM([expr {$::OFF_PERF_CTL}]) & 0x2) == 0}]

# ---- 5. `perf live` must WARN that the 64-bit pairs can tear -------------------
reset_mem
set ::MEM([expr {$::OFF_PERF_CTL}])      [ctlWord 12 0x1F]
set ::MEM([expr {$::OFF_PERF_CYCLE_LO}]) 10
set ::MEM([expr {$::OFF_PERF_INST_LO}])  5
set ::WRITES {}
set out [capture {perf_report 0 0}]
check "perf live issues NO control writes" [expr {$::WRITES eq {}}]
check "perf live warns about tearing" [string match "*can TEAR*" $out]

# ---- 6. the 64-bit pair is assembled HI<<32 | LO -------------------------------
reset_mem
set ::MEM([expr {$::OFF_PERF_CTL}])      [ctlWord 12 0x1F]
set ::MEM([expr {$::OFF_PERF_CYCLE_LO}]) 0x00000002
set ::MEM([expr {$::OFF_PERF_CYCLE_HI}]) 0x00000001
check "perf_rd64 assembles HI<<32|LO" \
      [expr {[perf_rd64 $::OFF_PERF_CYCLE_LO $::OFF_PERF_CYCLE_HI] == 0x100000002}]

puts ""
if {$::fails == 0} { puts "All checks passed."; exit 0 }
puts "$::fails check(s) FAILED."
exit 1
