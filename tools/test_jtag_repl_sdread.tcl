#!/usr/bin/env tclsh
##
## Self-checking test for the `sd-read` REPL command (sd_read_file and its
## sdp_* helpers in jtag_repl.tcl) against a MODELLED sd_bulk_writer
## (CAPS v2): register window + staging BRAM + a card whose bytes are a
## deterministic function of (lba, offset).  No hardware, no Vivado.
##
## The model returns burst-read DATA in REVERSED word order (one of the two
## orders Vivado versions produce) so the read-order calibration is
## exercised, and injects the two transient faults sd-read must survive:
##   * a JTAG burst that corrupts one word (dump CRC != hardware CRC);
##   * an SD re-read pass whose CRC differs once (unstable wire read).
## The output file must still equal the modelled card byte for byte.
##
## Run: tclsh tools/test_jtag_repl_sdread.tcl        (exit 0 = all checks pass)

set HERE [file dirname [file normalize [info script]]]
set REPL [file join $HERE jtag_repl.tcl]

set fh [open $REPL r]; set src [read $fh]; close $fh
proc cut {src a b} {
    set s [string first $a $src]
    set e [string first $b $src $s]
    if {$s < 0 || $e < 0} {
        puts "FAIL  extraction marker not found ('$a' .. '$b') -- the markers moved; fix this test"
        exit 1
    }
    return [string range $src $s [expr {$e - 1}]]
}
set consts [cut $src "set SDP_REG_BASE " "# ── vHDD"]
set procs  [cut $src "proc sdp_addr \{off\}" "proc sd_fast_status \{\} \{"]

# ---- the modelled bulk writer -------------------------------------------------
set ::STAGE_SECTORS 256
array set ::REG {lba 0 blkcnt 1 status 0x102 crc 0}
array set ::STG {}
set ::TXN {}
set ::burst_reads 0
set ::corrupt_burst_at -1      ;# corrupt one word of this (0-based) burst read
set ::verify_lie_at -1         ;# make this (0-based) GO_VERIFY report a bad CRC
set ::n_verify 0
set ::n_read_ops 0
set ::axi AXI

proc card_byte {lba k} { return [expr {($lba * 131 + $k * 7 + ($lba >> 4) * 29 + 0x5A) & 0xFF}] }
proc card_bytes {lba nsec} {
    set out {}
    for {set s 0} {$s < $nsec} {incr s} {
        for {set k 0} {$k < 512} {incr k} { lappend out [card_byte [expr {$lba + $s}] $k] }
    }
    return [binary format c* $out]
}

proc model_go {ctrl} {
    set lba $::REG(lba); set n $::REG(blkcnt)
    if {$n < 1 || $n > $::STAGE_SECTORS} { set ::REG(status) [expr {0x100 | 0x2 | 0x4 | (0xE << 4)}]; return }
    set data [card_bytes $lba $n]
    set crc [expr {[zlib crc32 $data] & 0xFFFFFFFF}]
    if {$ctrl == 0x5DB00003} {
        incr ::n_read_ops
        binary scan $data iu* words
        set i 0
        foreach w $words { set ::STG($i) $w; incr i }
    } elseif {$ctrl == 0x5DB00002} {
        if {$::n_verify == $::verify_lie_at} { set crc [expr {$crc ^ 0x1}] }
        incr ::n_verify
    } else {
        error "model: unexpected CTRL [format 0x%08X $ctrl] (sd-read must never write)"
    }
    set ::REG(crc) $crc
    set ::REG(status) 0x102
}

proc wr {addr data} {
    set addr [expr {$addr}]; set data [expr {$data & 0xFFFFFFFF}]
    if {$addr & 0x100000} { set ::STG([expr {($addr & 0xFFFFF) >> 2}]) $data; return }
    switch -- [format 0x%X $addr] {
        0x0 { set ::REG(lba) $data }
        0x4 { model_go $data }
        0xC { set ::REG(blkcnt) $data }
        default {}
    }
}
proc rd {addr} {
    set addr [expr {$addr}]
    if {$addr & 0x100000} {
        set i [expr {($addr & 0xFFFFF) >> 2}]
        return [format %08X [expr {[info exists ::STG($i)] ? $::STG($i) : 0}]]
    }
    switch -- [format 0x%X $addr] {
        0x0  { return [format %08X $::REG(lba)] }
        0x8  { return [format %08X $::REG(status)] }
        0xC  { return [format %08X $::REG(blkcnt)] }
        0x10 { return [format %08X $::REG(crc)] }
        0x14 { return 5DB70001 }
        0x18 { return [format %08X [expr {(2 << 16) | $::STAGE_SECTORS}]] }
    }
    return 00000000
}
proc create_hw_axi_txn {args} {
    set ::TXN [dict create]
    for {set i 0} {$i < [llength $args]} {incr i} {
        set k [lindex $args $i]
        if {$k in {-quiet -force}} { continue }
        if {[string match -* $k]} { dict set ::TXN $k [lindex $args [incr i]] }
    }
}
proc run_hw_axi {args} {}
proc get_hw_axi_txns {name} { return $name }
proc get_property {prop obj} {
    if {$prop ne "DATA"} { error "model: get_property $prop" }
    if {[dict get $::TXN -burst] ne "INCR"} { error "model: burst read without -burst INCR" }
    set base [expr {"0x[dict get $::TXN -address]" & 0xFFFFF}]
    set n [dict get $::TXN -len]
    set words {}
    for {set i 0} {$i < $n} {incr i} {
        set j [expr {($base >> 2) + $i}]
        lappend words [expr {[info exists ::STG($j)] ? $::STG($j) : 0}]
    }
    if {$::burst_reads == $::corrupt_burst_at} {
        lset words 17 [expr {[lindex $words 17] ^ 0x00100000}]
    }
    incr ::burst_reads
    set hex ""
    foreach w [lreverse $words] { append hex [format %08X $w] }
    return $hex
}
proc after {ms} {}
proc sdp_reprogram_current {} { error "model: recovery reprogram should not be needed" }

eval $consts
eval $procs

set fails 0
proc check {what cond} {
    if {$cond} { puts "PASS  $what" } else { puts "FAIL  $what"; incr ::fails }
}

set tmpdir [expr {[info exists ::env(TMPDIR)] ? $::env(TMPDIR) : "/tmp"}]
set out [file join $tmpdir "sdread_model_[pid].bin"]

# 600 sectors = 256 + 256 + 88: exercises the short final batch.
# Burst 3 = second burst of batch 1's FIRST dump (after 1 calibration burst
# that succeeds) -> one dump retry.  Second GO_VERIFY lies -> one card re-read.
set ::corrupt_burst_at 3
set ::verify_lie_at 1
set lba0 12345
set n [sd_read_file $lba0 600 $out]
check "returned sector count" [expr {$n == 600}]
check "read order calibrated as rev" [expr {$::sdp_rd_order eq "rev"}]
set f [open $out rb]; set got [read $f]; close $f
check "output size" [expr {[string length $got] == 600 * 512}]
check "output == card bytes" [expr {$got eq [card_bytes $lba0 600]}]
check "unstable re-read retried the card (4 GO_READ ops = 3 batches + 1)" \
    [expr {$::n_read_ops == 4}]

# noreread: no GO_VERIFY at all.
set ::n_verify 0; set ::corrupt_burst_at -1; set ::verify_lie_at -1
set n [sd_read_file 7 3 $out 0]
set f [open $out rb]; set got [read $f]; close $f
check "noreread: 3 sectors byte-exact" [expr {$n == 3 && $got eq [card_bytes 7 3]}]
check "noreread: no GO_VERIFY issued" [expr {$::n_verify == 0}]

file delete $out
if {$fails} { puts "\n$fails check(s) FAILED"; exit 1 }
puts "\nall sd-read checks passed"
exit 0
