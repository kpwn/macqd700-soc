# unstick.tcl — release the ioTrap-0x2E DCE deadlock that blocks the boot.
#
# WHAT THIS IS: a WORKAROUND, not a fix.  The real bug is that two chained
# requests queue on the same DCE and neither dispatches: the outer request's
# work routine is itself sitting in the ioResult poll after making a nested FS
# call, and the inner cannot run because the outer still holds the DCE.
# Clearing both ioResult words breaks the cycle; the machine then runs Mac OS
# normally (verified 2026-07-28: exc_count 35k -> 5.1M, Ticks at 60.2 Hz, no
# re-stall).
#
# Addresses are DERIVED, not hardcoded — the param blocks move between boots
# and between bitstreams (0x00383D10 on one boot, 0x001FC5A4 on another).
#
#   tcl source .../unstick.tcl ; unstick
#
# Notes:
#  * The driver queue header lives at 0x00016E78 and is a QHdr:
#        qFlags (w @0x16E78)  qHead (l @0x16E7A)  qTail (l @0x16E7E)
#    Both are ODD-aligned longs — read them byte-wise, not with an aligned rd.
#  * ioResult is a WORD at param_block+0x10.  We clear the whole long, which
#    also zeroes the high half of ioNamePtr; on every boot observed that half
#    was already 0.
#  * `dcache-op inv` afterwards is MANDATORY: JTAG writes go to memory and the
#    CPU would otherwise keep reading its cached ioResult=1 forever.

proc _u32 {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return -1 }
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}

# Read a 32-bit big-endian value at an arbitrary (possibly odd) address by
# combining two aligned reads.
proc _u32_unaligned {addr} {
    set base [expr {$addr & ~3}]
    set sh   [expr {$addr - $base}]
    set lo [_u32 $base]
    set hi [_u32 [expr {$base + 4}]]
    if {$lo < 0 || $hi < 0} { return -1 }
    set both [expr {($lo << 32) | $hi}]
    return [expr {($both >> ((4 - $sh) * 8)) & 0xFFFFFFFF}]
}

# Walk the driver queue and clear ioResult on every entry that is still
# "in progress" (>0).  Returns the number of blocks released.
proc unstick {{qhdr 0x00016E78}} {
    dbg_wr $::OFF_CONTROL 0x0            ;# make sure we are not halted-stuck
    set head [_u32_unaligned [expr {$qhdr + 2}]]
    set tail [_u32_unaligned [expr {$qhdr + 6}]]
    puts [format "> unstick: queue head=0x%08X tail=0x%08X" $head $tail]

    set n 0
    set pb $head
    for {set guard 0} {$guard < 16} {incr guard} {
        if {$pb <= 0 || $pb == 0xFFFFFFFF} { break }
        set trapw [_u32 [expr {$pb + 4}]]      ;# qType<<16 | ioTrap
        set res   [_u32 [expr {$pb + 0x10}]]   ;# ioResult<<16 | ioNamePtr-hi
        set ior   [expr {($res >> 16) & 0xFFFF}]
        puts [format "  block 0x%08X  qType=0x%04X ioTrap=0x%04X ioResult=0x%04X" \
                 $pb [expr {($trapw >> 16) & 0xFFFF}] [expr {$trapw & 0xFFFF}] $ior]
        if {$ior != 0 && $ior < 0x8000} {      ;# positive => still in progress
            wr [expr {$pb + 0x10}] [expr {$res & 0x0000FFFF}]
            incr n
            puts "    -> cleared ioResult"
        }
        set nxt [_u32 $pb]                     ;# qLink
        if {$nxt == $pb} { break }
        set pb $nxt
    }

    if {$n > 0} {
        # MANDATORY: the CPU's write-back D-cache still holds ioResult=1.
        dbg_wr $::OFF_CONTROL 0x0
        puts "> unstick: released $n block(s) — issue `dcache-op inv` then `halt-release`"
    } else {
        puts "> unstick: nothing in progress; the machine is not in this deadlock"
    }
    return $n
}

# ── wait_eff — wait for an EFFECTIVE HALT before reading registers ──────────
# Register reads at a breakpoint are only valid once HALT_REASON bit 3 (0x08)
# is set.  The break-pc LATCH (HALT_CTL bit 5) sets EARLIER, and reading in
# that window returns stale snap-chain values — jtag_repl's own live-arch
# refuses to read without bit 3 ("Stable only when CPU is halted; racy under
# free-run").
#
# Measured 2026-07-28 capturing SCSI CDBs at 0x00009CBA:
#     polling the break-pc latch : 57% valid (A6 stale 43% of the time)
#     gating on effective halt   : 93% valid
# The stale reads do NOT fail loudly — they fabricate plausible data.  They
# produced a phantom "169 TEST UNIT READY commands" finding (real count: 6) and
# a phantom "constant +2736 block offset vs MAME".  Always gate.
proc wait_eff {{tmo 6000}} {
    set w 0
    while {$w < $tmo} {
        set r [_u32 -1]
        set rv [dbg_rd $::OFF_HALT_REASON]
        if {[scan $rv %x r] == 1 && ($r & 0x08) != 0} { return 1 }
        after 10 ; incr w 10
    }
    return 0
}


# ── autounstick — poll for the stall, then clear it, unattended ─────────────
# Boot reaches the ioResult poll ~20-25 s after reset and wedges there.  This
# watches for that condition and releases it automatically, so a cold boot
# reaches the Finder without hand-holding.
#
# Detection: the CPU sits in the 3-instruction poll (0x0002E8A4/A8/AA) across
# several consecutive samples AND the driver queue is non-empty.  Requiring
# BOTH avoids firing on a healthy transit of the poll — a normal boot enters
# it ~1400 times and leaves each time.
proc autounstick {{timeout_s 90}} {
    set t 0
    set inpoll 0
    while {$t < $timeout_s} {
        set pc [_u32_pc]
        if {$pc >= 0x0002E8A4 && $pc <= 0x0002E8AA} { incr inpoll } else { set inpoll 0 }
        if {$inpoll >= 4} {
            set h [_u32_unaligned 0x00016E7A]
            if {$h > 0x1000 && $h < 0x400000} {
                puts "> autounstick: stall detected (pc=0x[format %08X $pc], queue head=0x[format %08X $h])"
                set n [unstick]
                if {$n > 0} {
                    dcache_inv_and_release
                    puts "> autounstick: released $n block(s); machine resumed"
                    return $n
                }
            }
            set inpoll 0
        }
        after 500 ; incr t
    }
    puts "> autounstick: no stall seen within ${timeout_s}s"
    return 0
}
proc _u32_pc {} {
    set v [dbg_rd $::OFF_PC]
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}
proc dcache_inv_and_release {} {
    # MANDATORY: JTAG writes land in memory; the CPU would keep reading its
    # cached ioResult=1 forever without an invalidate.  Reuse the REPL's own
    # dcache-op (it also polls the done bit) rather than poking the register.
    dcache-op inv
    clear_auto_halt
    dbg_wr $::OFF_CONTROL 0x0
}

puts "> unstick.tcl loaded (unstick / autounstick / wait_eff)"
