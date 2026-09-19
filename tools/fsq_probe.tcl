# fsq_probe.tcl — File System queue + FSBusy semaphore state (task #140).
#
# THE MECHANISM (established 2026-07-28 by symbolizing the RAM patch).
# The synchronous File Manager path is:
#
#   _FSDispatch  0x0002E938:
#       move.w  SR,-(A7)
#       ori.w   #$0700,SR        ; mask ALL interrupts
#       lea     (FSQHdr).w,A1    ; $0360
#       _Enqueue                 ; put this param block on the FS queue
#       bset    #0,(FSQHdr).w    ; test-and-set FSBusy
#       beq.s   run_it           ; bit WAS clear -> we own the FS, run it
#       move.w  (A7)+,SR         ; bit was ALREADY SET -> someone else owns it,
#       moveq   #0,D0            ;   so leave our PB queued and return,
#       rts                      ;   trusting the owner to drain the queue
#
#   caller (_VSyncWaitPoll 0x0002E8A4):
#       move.w  (0x10,A0),D0     ; poll ioResult
#       bgt.s   -6               ; spin while > 0
#
# So a caller that loses the bset race does NOT run its own request -- it
# depends entirely on whoever holds FSBusy to drain the queue and post
# ioResult.  If FSBusy is ever left SET with no owner actually running, every
# later _FSDispatch enqueues and returns, and every caller spins forever on an
# ioResult nobody will post.  That matches the observed bug exactly, including
# why clearing one ioResult by hand frees that one caller and the stall
# immediately comes back for the next request.
#
# WHY THE DEVICE MANAGER PROBE CAME BACK EMPTY.  dce_probe reports 0 drivers
# active / 0 queues non-empty during this stall, which is CORRECT and not a
# failed measurement: the stuck param block is on the FILE SYSTEM queue at
# $0360, not on any driver's queue.  Do not go looking for a wedged driver.
#
# HEALTHY BASELINE (idle machine, verified 2026-07-28):
#   $0360 = 00000000 00000000  -> FSBusy=0, qHead=0, qTail=0
#
# THE STALL SIGNATURE TO CONFIRM:
#   FSBusy=1 AND qHead != 0, PERSISTENTLY (re-sample seconds apart).  FSBusy=1
#   for a single sample is NORMAL -- it is set for the duration of every File
#   Manager call.  Only persistence makes it a wedge.
#
# QHdr layout: qFlags word at +0, qHead long at +2, qTail long at +6.
# NOTE +2 and +6 are ≡2 mod 4 and `rd` SILENTLY ALIGNS DOWN, so they MUST be
# read with the unaligned assembler, not a bare rd.
#
#   tcl source tools/fsq_probe.tcl ; tcl fsq_probe [samples]

proc _fsq_u32 {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return -1 }
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}

# Unaligned-safe 32-bit read (see the align-down note above).
proc _fsq_u32u {addr} {
    set lo [expr {$addr & ~3}]
    set sh [expr {8 * ($addr & 3)}]
    if {$sh == 0} { return [_fsq_u32 $lo] }
    set a [_fsq_u32 $lo]
    set b [_fsq_u32 [expr {$lo + 4}]]
    if {$a < 0 || $b < 0} { return -1 }
    return [expr {(($a << $sh) | ($b >> (32 - $sh))) & 0xFFFFFFFF}]
}

proc _fsq_pb {pb {limit 6}} {
    set n 0
    while {$pb > 0 && $n < $limit} {
        set iores [expr {[_fsq_u32u [expr {$pb + 16}]] >> 16}]
        set sres  [expr {$iores > 0x7FFF ? $iores - 0x10000 : $iores}]
        puts [format "      PB#%d @0x%08X  ioTrap=0x%04X ioResult=%d %s" \
                 $n $pb \
                 [expr {[_fsq_u32u [expr {$pb + 6}]] >> 16}] \
                 $sres \
                 [expr {$sres > 0 ? "<- IN PROGRESS (a caller is spinning on this)" : ""}]]
        set pb [_fsq_u32u $pb]
        incr n
    }
}

proc fsq_probe {{samples 3}} {
    puts "> fsq_probe — File System queue (FSQHdr \$0360) + FSBusy semaphore"
    set prev_busy -1
    set stuck 1
    for {set i 0} {$i < $samples} {incr i} {
        dcache-op push
        set qflags [expr {[_fsq_u32 0x0360] >> 16}]
        set qhead  [_fsq_u32u 0x0362]
        set qtail  [_fsq_u32u 0x0366]
        set busy   [expr {$qflags & 1}]
        puts [format "  \[%d\] qFlags=0x%04X FSBusy=%d  qHead=0x%08X qTail=0x%08X" \
                 $i $qflags $busy $qhead $qtail]
        if {$busy && $qhead > 0} { _fsq_pb $qhead } else { set stuck 0 }
        if {$i + 1 < $samples} { after 1000 }
    }
    puts ""
    if {$stuck} {
        puts "> verdict: FSBusy STUCK SET with a non-empty queue across all samples."
        puts ">          Nothing is draining the File System queue, so every caller"
        puts ">          that lost the bset race is spinning on an ioResult that will"
        puts ">          never be posted.  The bug is whoever owned FSBusy and"
        puts ">          returned/unwound WITHOUT clearing bit 0 of \$0360 and"
        puts ">          draining the queue."
    } else {
        puts "> verdict: not stuck (FSBusy clear, or queue empty, in at least one"
        puts ">          sample).  FSBusy=1 for a single sample is NORMAL -- it is"
        puts ">          held for the duration of every File Manager call."
    }
}

# Find every instruction that ACQUIRES or RELEASES the FSBusy semaphore.
#
# The acquire site is known (bset #0,(FSQHdr).w at 0x0002E944).  What matters
# for the wedge is the RELEASE site(s): whoever owns FSBusy must clear bit 0 of
# $0360 and drain the queue, and the bug is a path that returns or unwinds
# without doing so.  Knowing every release site turns "something forgot to
# release it" into a small set of breakpointable addresses.
#
# Encodings searched (bit number 0 as immediate, operand = $0360):
#   bclr #0,($0360).w   08B8 0000 0360     release, short absolute
#   bclr #0,($0360).l   08B9 0000 0360     release, long absolute
#   bset #0,($0360).w   08F8 0000 0360     acquire, short absolute
#   bset #0,($0360).l   08F9 0000 0360     acquire, long absolute
#
# This deliberately does NOT try to catch every conceivable way to clear a bit
# (andi.b #$FE, a move of a computed byte, a bclr through an address register).
# A miss here is a FALSE NEGATIVE, not a false positive -- if the release site
# is not found, do not conclude there isn't one.
proc fsq_sites {{start 0x0002E000} {end 0x0002F400}} {
    dcache-op push
    puts [format "> fsq_sites — scanning 0x%08X..0x%08X for FSBusy acquire/release" $start $end]
    set found 0
    for {set a $start} {$a < $end} {incr a 2} {
        set w [_fsq_u32u $a]
        if {$w < 0} { continue }
        set opc [expr {($w >> 16) & 0xFFFF}]
        if {$opc != 0x08B8 && $opc != 0x08B9 && $opc != 0x08F8 && $opc != 0x08F9} { continue }
        set bitno [expr {$w & 0xFFFF}]
        set operand [expr {[_fsq_u32u [expr {$a + 4}]] >> 16}]
        if {$bitno != 0 || $operand != 0x0360} { continue }
        set kind [expr {($opc == 0x08B8 || $opc == 0x08B9) ? "RELEASE bclr" : "ACQUIRE bset"}]
        puts [format "  0x%08X  %s #0,(\$0360)" $a $kind]
        incr found
    }
    puts ""
    if {$found == 0} {
        puts "> no acquire/release site found in that range.  This is a FALSE"
        puts ">   NEGATIVE risk, not proof: the release may use another encoding"
        puts ">   (andi.b, a byte move, bclr via An) or live outside the range."
        puts ">   Widen the range before concluding anything."
    } else {
        puts [format "> %d site(s).  Breakpoint the RELEASE site(s) and confirm they are" $found]
        puts ">   reached on a normal File Manager call; the wedge is a path that"
        puts ">   owns FSBusy and returns without hitting one."
    }
}

# Is the param block the CPU is spinning on actually ON the FS queue?
#
# This is THE discriminating measurement for task #140.  Run it with the A0
# read at the poll breakpoint.  The three outcomes are three DIFFERENT bugs and
# must not be conflated:
#
#   on-chain + FSBusy=1   the owner never released the semaphore.  Look for a
#                         path between the acquire (ROM 0x4080F00E / patch
#                         0x0002E944) and the release (ROM 0x4080F126) that
#                         exits without reaching the release.
#
#   on-chain + FSBusy=0   nobody owns the FS and the request is still queued:
#                         the drain loop at ROM 0x4080F154 decided the queue
#                         was empty when it was not.  That test is
#                         `tstl 0x362` -- a MISALIGNED 32-bit load (qHead is at
#                         $0362, which is 2 mod 4).
#
#   NOT on-chain          the PB was orphaned: dequeued (or lost from the
#                         chain) without its ioResult ever being posted, so
#                         nothing can ever complete it.  Suspect the misaligned
#                         32-bit STORES that maintain the chain
#                         (`movel %a0@,0x362` at ROM 0x4080F138,
#                          `movel %a0,0x366` at ROM 0x4080F176).
#
# The misaligned angle is a HYPOTHESIS, not a finding: $0362/$0366 are long-
# misaligned by construction, and the machine executes these constantly while
# running fine, so plain misaligned access clearly works.  Only a specific
# corner would fail.  Do not report it as the cause without evidence from here.
proc fsq_check_pb {pb {limit 32}} {
    dcache-op push
    set qflags [expr {[_fsq_u32 0x0360] >> 16}]
    set busy   [expr {$qflags & 1}]
    set qhead  [_fsq_u32u 0x0362]
    set qtail  [_fsq_u32u 0x0366]
    puts [format "> fsq_check_pb — is 0x%08X on the FS queue?" $pb]
    puts [format "  FSBusy=%d  qHead=0x%08X  qTail=0x%08X" $busy $qhead $qtail]

    set cur $qhead
    set n 0
    set found 0
    while {$cur > 0 && $n < $limit} {
        if {$cur == $pb} { set found 1 }
        puts [format "    chain\[%d\] = 0x%08X%s" $n $cur \
                 [expr {$cur == $pb ? "   <- THE POLLED PARAM BLOCK" : ""}]]
        set cur [_fsq_u32u $cur]
        incr n
    }
    if {$n >= $limit} { puts "    (chain longer than $limit — truncated or circular)" }
    puts ""
    if {$found && $busy} {
        puts "> verdict: ON-CHAIN + FSBusy SET — the owner never released."
        puts ">          Breakpoint ROM 0x4080F126 (clrw \$0360, the ONLY release"
        puts ">          site) and find the path that skips it."
    } elseif {$found} {
        puts "> verdict: ON-CHAIN + FSBusy CLEAR — nobody owns the FS yet the"
        puts ">          request is still queued.  The drain loop at 0x4080F154"
        puts ">          (`tstl 0x362`, a MISALIGNED 32-bit load) concluded the"
        puts ">          queue was empty when it was not."
    } else {
        puts "> verdict: NOT ON-CHAIN — the param block was ORPHANED.  It is"
        puts ">          unreachable from qHead, so nothing will ever post its"
        puts ">          ioResult.  Suspect the misaligned 32-bit stores that"
        puts ">          maintain the chain (0x4080F138 / 0x4080F176)."
    }
}

puts "> fsq_probe.tcl loaded (fsq_probe \[n\] | fsq_sites \[a\] \[b\] | fsq_check_pb <pb>)"
