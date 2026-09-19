# iostall_watch.tcl — detect the task-#140 ioResult stall the moment it starts.
#
# WHY IN-REPL.  The stall is intermittent (minutes of healthy running between
# occurrences), so detection is a WAITING problem.  Polling `pc` over the
# /tmp/jtag_in FIFO costs a round trip per sample and races with any other
# command; a Tcl loop inside the REPL samples in ~0.1s and cannot interleave.
#
# WHAT COUNTS AS A STALL.  The File Manager's _FSDispatch RAM patch does
#     bsr.w   <issue the request>
#     movea.l (SP)+,A0
#     0x0002E8A4  move.w (0x10,A0),D0     <- POLL_PC
#     0x0002E8A8  bgt.s  -6
# and it enters this poll CONSTANTLY in normal operation -- a break-pc on
# POLL_PC fires immediately on a perfectly healthy machine, so "we saw the poll
# PC once" means nothing.  What distinguishes the stall is DWELL: a healthy
# request completes in microseconds, so N consecutive samples spread over
# seconds that ALL land in the 2-instruction poll is decisive.
#
# POLL_PC IS RAM-SIZE DEPENDENT (the patch relocates with the heap):
#     4 MiB -> 0x0002E8A4    64 MiB -> 0x000324A4    128 MiB/1 GiB -> 0x00036484
# so it is VERIFIED BY OPCODE here rather than trusted: [POLL_PC] must read
# 0x30280010 and [POLL_PC+4] 0x6EFA4A40.  If a future boot moves it, this
# refuses to watch the wrong address instead of silently never firing.
#
#   tcl source tools/iostall_watch.tcl
#   tcl iostall_watch                 — defaults: 4 MiB PC, ~10 min
#   tcl iostall_watch 0x000324A4      — 64 MiB build
#   tcl iostall_watch 0x0002E8A4 2000 8
#        args: poll_pc, max_samples, consecutive-hits required
#
# On detection it STOPS and tells you the capture sequence; it deliberately
# does NOT halt the CPU itself, because halting is destructive to the very
# state you then want to read, and the operator may want to look at the screen
# or the DAFB first.

proc _iow_u32 {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return -1 }
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}

proc iostall_watch {{poll_pc 0x0002E8A4} {max_samples 2000} {need 8}} {
    # ── Verify we are watching the right address, by opcode ──────────────
    dcache-op push
    set op0 [_iow_u32 $poll_pc]
    set op1 [_iow_u32 [expr {$poll_pc + 4}]]
    puts [format "> iostall_watch — POLL_PC 0x%08X" $poll_pc]
    puts [format "  \[POLL_PC\]   = 0x%08X  (expect 0x30280010  move.w (0x10,A0),D0)" $op0]
    puts [format "  \[POLL_PC+4\] = 0x%08X  (expect 0x6EFA4A40  bgt.s -6 ; tst.w D0)" $op1]
    if {$op0 != 0x30280010 || $op1 != 0x6EFA4A40} {
        puts "  !! OPCODES DO NOT MATCH — this is not the poll loop on this boot."
        puts "     The _FSDispatch patch relocates with the heap, so a different"
        puts "     RAM size means a different POLL_PC.  Find it before watching:"
        puts "       4 MiB 0x0002E8A4 | 64 MiB 0x000324A4 | 128 MiB+ 0x00036484"
        puts "     Refusing to watch the wrong address."
        return
    }
    puts "  opcodes verified — watching."
    puts ""

    set run 0
    set hits 0
    set best 0
    for {set i 0} {$i < $max_samples} {incr i} {
        set pc [_iow_u32 [expr {$::DBG_BASE + $::OFF_PC}]]
        if {$pc == $poll_pc || $pc == [expr {$poll_pc + 4}]} {
            incr run
            incr hits
            if {$run > $best} { set best $run }
            if {$run >= $need} {
                puts ""
                puts [format "> STALL DETECTED after %d samples (%d consecutive in the poll)" $i $run]
                puts ">   The CPU has been sitting in the 2-instruction ioResult poll"
                puts ">   continuously.  A healthy request clears in microseconds, so"
                puts ">   this is the wedge, not a request that happens to be in flight."
                puts ">"
                puts "> CAPTURE NOW, in this order (halting is destructive — do it last):"
                puts ">   tcl dce_probe                 <- Device Manager state FIRST"
                puts [format ">   break-pc 0x%08X 8000" $poll_pc]
                puts ">   halt-status                   <- require effective=1"
                puts ">   live-arch                     <- A0 = the param block"
                puts ">   tcl hexdump <A0> 16           <- compare vs the healthy"
                puts ">                                    reference: qType=5,"
                puts ">                                    ioCompletion=0, ioResult=0"
                puts ">"
                puts "> Healthy baseline for reference (captured 2026-07-28):"
                puts ">   A0=0x003D1864 qType=0x0005 ioTrap=0x0208"
                puts ">   ioCmdAddr=0x40811CF0 ioCompletion=0 ioResult=0"
                return 1
            }
        } else {
            set run 0
        }
        if {($i % 100) == 0 && $i > 0} {
            puts [format "  ... %d samples, %d in-poll (%.1f%%), longest run %d — still healthy" \
                     $i $hits [expr {100.0*$hits/$i}] $best]
        }
        after 250
    }
    puts ""
    puts [format "> no stall in %d samples: %d in-poll (%.1f%%), longest run %d" \
             $max_samples $hits [expr {100.0*$hits/$max_samples}] $best]
    puts ">   A nonzero in-poll percentage is NORMAL and expected — the patch"
    puts ">   polls on every synchronous File Manager call.  Only a long"
    puts ">   CONSECUTIVE run means wedged."
    return 0
}

puts "> iostall_watch.tcl loaded (run: iostall_watch \[poll_pc\] \[max_samples\] \[need\])"
