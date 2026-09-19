# dce_probe.tcl — walk the Device Manager unit table and find the stuck driver.
#
# WHY THIS EXISTS
# ────────────────────────────────────────────────────────────────────────
# Task #140: the machine periodically wedges in a synchronous ioResult poll
#     move.w (0x10,A0),D0 ; bgt.s -6
# i.e. it is spinning until ioResult stops being > 0 ("still in progress").
# Writing 0 to [A0+0x10] + `dcache-op inv` always frees it, so the CPU and the
# poll are fine — SOMETHING NEVER POSTS COMPLETION.  That is a Device Manager
# state question, not a CPU question, and until now it was only ever probed by
# reading ioResult itself, which cannot distinguish the possible causes.
#
# WHAT THIS ANSWERS THAT ioResult ALONE CANNOT
#   * WHICH driver owns the stalled request (ioRefNum -> unit -> DCE)
#   * whether the DCE is marked drvrActive (a request is genuinely running)
#   * whether requests are QUEUED BEHIND it (the self-deadlock signature)
#   * whether the head request has a completion routine at all
#
# THE DECISIVE SIGNATURE.  The Device Manager runs ONE request per DCE at a
# time: if drvrActive is set, a new call is queued instead of started, and it
# only starts when IODone fires for the current one.  So a DCE that is
# PERMANENTLY drvrActive with a non-empty queue means IODone never ran for the
# head request — everything behind it is stranded forever.  If the culprit DCE
# instead shows drvrActive CLEAR with a non-empty queue, the bug is the
# opposite one (a request was dequeued/never started), and this is a DIFFERENT
# root cause — do not conflate them.
#
# NEGATIVE RESULT IS ALSO INFORMATIVE: if no DCE is active and no queue is
# non-empty while the CPU is provably spinning on the poll, then the param
# block being polled was never queued on a driver at all — which would point
# at the File Manager / a RAM patch rather than the Device Manager.
#
# READ RULES (each of these has silently corrupted data on this board before)
#   * JTAG reads BYPASS the write-back D-cache -> `dcache-op push` first, or
#     you read stale DRAM instead of what the CPU actually wrote.
#   * `rd` SILENTLY ALIGNS DOWN on unaligned addresses -> never read an odd
#     address; assemble words/bytes out of the aligned long containing them.
#   * A failed read returns the literal string BADA0BAD, which scan %x would
#     otherwise turn into a plausible-looking number.
#
#   tcl source tools/dce_probe.tcl ; dce_probe
#
# Low-memory globals used (Inside Macintosh: Devices / Operating System Utils)
#   $011C UTableBase   long   base of the unit table (DCE handles/pointers)
#   $01D2 UnitNtryCnt  word   number of unit table entries
#
# DCtlEntry layout (Inside Macintosh: Devices, "Device Control Entry")
#   +0  dCtlDriver   long    driver handle/pointer
#   +4  dCtlFlags    word    low byte bit5 dOpened, bit6 dRAMBased,
#                            bit7 drvrActive   <- THE bit this tool exists for
#   +6  dCtlQHdr     10by    qFlags(word) qHead(long) qTail(long)
#   +16 dCtlPosition long
#   +20 dCtlStorage  long
#   +24 dCtlRefNum   word
#
# ParamBlockHeader (the queue elements hanging off dCtlQHdr.qHead)
#   +0  qLink long   +4 qType word   +6 ioTrap word   +8  ioCmdAddr long
#   +12 ioCompletion long           +16 ioResult word  <- what the CPU polls
#   +18 ioNamePtr long  +22 ioVRefNum word  +24 ioRefNum word  +26 csCode word

proc _dce_u32 {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return -1 }
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}

# Read a 16-bit word WITHOUT tripping `rd`'s silent align-down.
proc _dce_u16 {addr} {
    set l [_dce_u32 [expr {$addr & ~3}]]
    if {$l < 0} { return -1 }
    if {$addr & 2} { return [expr {$l & 0xFFFF}] }
    return [expr {($l >> 16) & 0xFFFF}]
}

proc _dce_flagstr {flags} {
    set lo [expr {$flags & 0xFF}]
    set s ""
    if {$lo & 0x20} { append s "dOpened " }
    if {$lo & 0x40} { append s "dRAMBased " }
    if {$lo & 0x80} { append s "drvrACTIVE " }
    if {$s eq ""} { set s "(none) " }
    return [string trimright $s]
}

# Walk one DCE's request queue, printing each param block.  Bounded so a
# corrupt/circular qLink cannot spin the REPL forever.
proc _dce_walk_queue {qhead {limit 8}} {
    set pb $qhead
    set n 0
    while {$pb > 0 && $n < $limit} {
        set qtype  [_dce_u16 [expr {$pb + 4}]]
        set iotrap [_dce_u16 [expr {$pb + 6}]]
        set iocomp [_dce_u32 [expr {$pb + 12}]]
        set iores  [_dce_u16 [expr {$pb + 16}]]
        set iorefn [_dce_u16 [expr {$pb + 24}]]
        set cscode [_dce_u16 [expr {$pb + 26}]]
        # ioResult is SIGNED: >0 = in progress, 0 = done ok, <0 = error.
        set sres [expr {$iores > 0x7FFF ? $iores - 0x10000 : $iores}]
        puts [format "      PB#%d @0x%08X  qType=0x%04X ioTrap=0x%04X csCode=%-5d" \
                 $n $pb $qtype $iotrap $cscode]
        puts [format "                        ioResult=%-6d %s   ioCompletion=0x%08X %s" \
                 $sres \
                 [expr {$sres > 0 ? "<- IN PROGRESS (this is what the CPU polls)" : "(complete)"}] \
                 $iocomp \
                 [expr {$iocomp == 0 ? "(none -> synchronous caller must poll)" : ""}]]
        puts [format "                        ioRefNum=%d" \
                 [expr {$iorefn > 0x7FFF ? $iorefn - 0x10000 : $iorefn}]]
        set pb [_dce_u32 $pb]
        incr n
        if {$pb == $qhead} { puts "      (qLink loops back to head — circular queue)"; break }
    }
    if {$n >= $limit} { puts "      (stopped at $limit entries — queue longer or corrupt)" }
}

proc dce_probe {} {
    dcache-op push
    puts "> dce_probe — Device Manager unit table / driver queue state"

    set utable [_dce_u32 0x011C]
    set ncount [_dce_u16 0x01D2]
    if {$utable <= 0 || $ncount <= 0} {
        puts [format "  UTableBase \$011C = 0x%08X  UnitNtryCnt \$01D2 = %d  <- not initialised" \
                 $utable $ncount]
        return
    }
    puts [format "  UTableBase \$011C = 0x%08X   UnitNtryCnt \$01D2 = %d" $utable $ncount]
    puts ""

    set n_active 0
    set n_queued 0
    set suspects {}

    # The unit table holds HANDLES to device control entries, not pointers
    # (Inside Macintosh: Devices).  Dereferencing only once yields a master
    # pointer block and every field reads as heap garbage -- on 2026-07-28 that
    # produced qHead=0xB6DB6DB6 (a repeating bit pattern) for half the table and
    # two different units "sharing" one DCE.
    #
    # dCtlRefNum at +24 MUST equal -(unit+1) by definition, so it is a free
    # self-check on whether we dereferenced correctly.  Entries that fail it are
    # SKIPPED rather than printed with a warning: a mis-dereferenced entry is not
    # evidence of a corrupt unit table, it is evidence of a bad read, and
    # printing it invites reading meaning into noise.
    set n_bad 0
    for {set u 0} {$u < $ncount} {incr u} {
        set h [_dce_u32 [expr {$utable + 4*$u}]]
        if {$h <= 0} { continue }

        # This machine runs 24-BIT ADDRESSING (verified 2026-07-28: BufPtr
        # 0x3E3718 / HeapEnd 0x3CBF98 on the 4 MiB default window), so a
        # dereferenced handle is a MASTER POINTER whose HIGH BYTE holds Memory
        # Manager flag bits, not address bits -- [entry] reads 0x800052D0 and
        # the real DCE is at 0x0052D0.  Masking to 24 bits is therefore part of
        # the dereference, not an optional cleanup.
        #
        # Try every plausible interpretation and let dCtlRefNum pick the winner
        # rather than hardcoding one: handle-masked, handle-raw, pointer-masked,
        # pointer-raw.  (A 32-bit-clean machine would validate on a *-raw form,
        # so this keeps working if the OS is ever switched to 32-bit mode.)
        set dce -1
        set hderef [_dce_u32 $h]
        foreach cand [list \
                [expr {$hderef > 0 ? $hderef & 0x00FFFFFF : -1}] \
                $hderef \
                [expr {$h & 0x00FFFFFF}] \
                $h] {
            if {$cand <= 0} { continue }
            set r [_dce_u16 [expr {$cand + 24}]]
            if {$r < 0} { continue }
            set sr [expr {$r > 0x7FFF ? $r - 0x10000 : $r}]
            if {$sr == [expr {-($u+1)}]} { set dce $cand; break }
        }
        if {$dce <= 0} { incr n_bad; continue }

        set flags [_dce_u16 [expr {$dce + 4}]]
        if {$flags < 0} { continue }
        set qhead [_dce_u32 [expr {$dce + 8}]]
        set qtail [_dce_u32 [expr {$dce + 12}]]
        set active [expr {($flags & 0x80) ? 1 : 0}]
        set hasq   [expr {$qhead > 0 ? 1 : 0}]

        # Only print entries that are doing something — a full 32-unit dump
        # buries the one interesting line.
        if {!$active && !$hasq} { continue }

        puts [format "  unit %-2d  refNum %-4d  DCE @0x%08X" $u [expr {-($u+1)}] $dce]
        # NOTE the \[ \] escapes: bare [%s] inside a quoted Tcl string is a
        # command substitution, not literal brackets.
        puts [format "          dCtlFlags=0x%04X  \[%s\]" $flags [_dce_flagstr $flags]]
        puts [format "          dCtlQHdr qHead=0x%08X qTail=0x%08X" $qhead $qtail]
        if {$hasq} { _dce_walk_queue $qhead }
        puts ""

        if {$active} { incr n_active }
        if {$hasq}   { incr n_queued }
        if {$active && $hasq} { lappend suspects $u }
    }

    puts [format "  summary: %d DCE(s) drvrActive, %d with a non-empty queue" $n_active $n_queued]
    if {$n_bad > 0} {
        puts [format "           (%d unit-table entr%s did not validate as a DCE and were skipped)" \
                 $n_bad [expr {$n_bad == 1 ? "y" : "ies"}]]
    }
    puts ""
    if {[llength $suspects] > 0} {
        puts "> verdict: DCE self-deadlock signature PRESENT on unit(s): $suspects"
        puts ">          drvrActive is set AND requests are queued behind it, so"
        puts ">          IODone never ran for the head request.  Find who was"
        puts ">          supposed to call IODone for that driver — for a disk"
        puts ">          driver that is normally the SCSI completion interrupt."
        puts ">          Re-run this a few seconds apart: if the SAME unit stays"
        puts ">          active with the SAME qHead, it is wedged, not merely busy."
    } elseif {$n_queued > 0} {
        puts "> verdict: requests are QUEUED but no DCE is drvrActive."
        puts ">          That is NOT the self-deadlock — it means a request was"
        puts ">          left on a queue that nothing will ever start.  Different"
        puts ">          root cause; do not conflate the two."
    } else {
        puts "> verdict: no driver is active and nothing is queued."
        puts ">          If the CPU is provably spinning on the ioResult poll right"
        puts ">          now, then the polled param block was NEVER handed to the"
        puts ">          Device Manager — look at the File Manager / RAM patch that"
        puts ">          owns it, not at driver completion."
    }
}

puts "> dce_probe.tcl loaded (run: dce_probe)"
