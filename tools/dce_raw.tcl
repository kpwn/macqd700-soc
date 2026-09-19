# dce_raw.tcl — raw-memory helpers for Device-Manager spelunking.
#
# Written 2026-07-28 after dce_probe's dCtlRefNum self-check rejected EVERY
# unit-table entry under both the handle and the pointer interpretation.  When
# a structured decoder disagrees with itself that completely, the next step is
# to stop decoding and LOOK at the bytes -- otherwise you just keep inventing
# new layout guesses and grading them against each other.
#
#   tcl source tools/dce_raw.tcl
#   tcl lowmem_sanity      — prove the read path works using values we ALREADY
#                            know (CurApName should literally spell "Finder")
#   tcl hexdump <addr> <n> — n longs, with ASCII, so structure is visible
#   tcl utable_raw [n]     — first n unit-table entries + what each points to

proc _dr_u32 {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return -1 }
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}

# Assemble a 32-bit value at an address that may NOT be 4-byte aligned.
# `rd` SILENTLY ALIGNS DOWN, so _dr_u32 on e.g. $02AE (ROMBase, 2 mod 4)
# returns bytes 2AC..2AF -- a window straddling two different fields.  On
# 2026-07-28 that made ROMBase read 0xDEB84080 (the real 0x40800000 shifted
# into the low half) and made Ticks look FROZEN, because the compared bytes
# were Ticks' HIGH half, which only changes every ~18 minutes.
proc _dr_u32u {addr} {
    set lo [expr {$addr & ~3}]
    set sh [expr {8 * ($addr & 3)}]
    if {$sh == 0} { return [_dr_u32 $lo] }
    set a [_dr_u32 $lo]
    set b [_dr_u32 [expr {$lo + 4}]]
    if {$a < 0 || $b < 0} { return -1 }
    return [expr {(($a << $sh) | ($b >> (32 - $sh))) & 0xFFFFFFFF}]
}

proc _dr_ascii {word} {
    set s ""
    for {set i 3} {$i >= 0} {incr i -1} {
        set c [expr {($word >> (8*$i)) & 0xFF}]
        append s [expr {($c >= 0x20 && $c < 0x7F) ? [format %c $c] : "."}]
    }
    return $s
}

proc hexdump {addr n} {
    dcache-op push
    for {set i 0} {$i < $n} {incr i 4} {
        set line ""
        set asc ""
        for {set j 0} {$j < 4 && ($i+$j) < $n} {incr j} {
            set w [_dr_u32 [expr {$addr + 4*($i+$j)}]]
            if {$w < 0} { append line "-------- "; append asc "????" } \
            else        { append line [format "%08X " $w]; append asc [_dr_ascii $w] }
        }
        puts [format "  %08X  %-36s |%s|" [expr {$addr + 4*$i}] $line $asc]
    }
}

# Prove the JTAG->Mac-RAM read path before trusting ANY structured decode.
# CurApName is a Pascal string; on this machine it is known to be "Finder"
# (verified 2026-07-28), so if this does not spell Finder the problem is the
# read path or the RAM window, NOT whatever struct you were decoding.
proc lowmem_sanity {} {
    dcache-op push
    puts "> lowmem_sanity — validating the JTAG read path against KNOWN values"
    puts ""
    puts "  CurApName \$0910 (Pascal string, expect \"Finder\"):"
    hexdump 0x0910 8
    puts ""
    foreach {name addr} {
        MemTop      0x0108
        BufPtr      0x010C
        StkLowPt    0x0110
        HeapEnd     0x0114
        TheZone     0x0118
        UTableBase  0x011C
        SysZone     0x02A6
        ApplZone    0x02AA
        ROMBase     0x02AE
        RAMBase     0x02B2
        Ticks       0x016A
    } {
        puts [format "  %-11s %s = 0x%08X" $name $addr [_dr_u32u $addr]]
    }
    set c1 [_dr_u32u 0x016A]
    after 500
    dcache-op push
    set c2 [_dr_u32u 0x016A]
    puts [format "  Ticks advancing: %d -> %d  %s" $c1 $c2 \
             [expr {$c2 > $c1 ? "(OS alive)" : "<- FROZEN"}]]
}

# Dump the unit table with NO interpretation: the entry, what it points to,
# and what THAT points to, side by side.  Whichever column contains something
# shaped like a DCE tells you the right indirection level.
proc utable_raw {{n 16}} {
    dcache-op push
    set utable [_dr_u32 0x011C]
    puts [format "> utable_raw — UTableBase = 0x%08X" $utable]
    puts "  unit  entry@utable   \[entry\]        \[\[entry\]\]      note"
    for {set u 0} {$u < $n} {incr u} {
        set e [_dr_u32 [expr {$utable + 4*$u}]]
        if {$e <= 0} { continue }
        set d1 [_dr_u32 $e]
        set d2 [expr {$d1 > 0 ? [_dr_u32 $d1] : -1}]
        # dCtlRefNum lives at +24 in a DCE and MUST read -(unit+1).
        set note ""
        foreach {lbl cand} [list entry $e {[entry]} $d1] {
            if {$cand <= 0} { continue }
            set l [_dr_u32 [expr {($cand + 24) & ~3}]]
            if {$l < 0} { continue }
            set rn [expr {($cand + 24) & 2 ? $l & 0xFFFF : ($l >> 16) & 0xFFFF}]
            set srn [expr {$rn > 0x7FFF ? $rn - 0x10000 : $rn}]
            if {$srn == [expr {-($u+1)}]} { append note "DCE is $lbl " }
        }
        puts [format "  %-5d 0x%08X    0x%08X    0x%08X    %s" $u $e $d1 $d2 $note]
    }
}

puts "> dce_raw.tcl loaded (lowmem_sanity | hexdump <addr> <nlongs> | utable_raw \[n\])"
