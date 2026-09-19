# slot_irq_health.tcl — one-shot verdict on the level-2 (VIA2 slot) IRQ chain.
#
# WHY THIS EXISTS
# ────────────────────────────────────────────────────────────────────────
# On 2026-07-28 the cursor was frozen and I/O completion was stalling, and
# it took most of a session to establish that BOTH were downstream of a
# stopped HDMI scanout.  Every link had to be measured by hand.  This
# encodes that chain so the next session gets the answer in one command.
#
# THE CHAIN (each link is a real measurement, in causal order):
#
#   pclk VBL pulse            DAFB REG_IRQ_STATUS +0x20 bit0 (vblank_pending)
#     -> dafb_vbl_level
#     -> via2_ca1_in = ~(dafb_vbl_level & dafb_irq_pb_sync)   <- the AND that
#        makes ALL slot IRQs depend on the display VTG (task #147)
#     -> VIA2 IFR.CA1                       IFR bit1, and bit7 = IRQ asserted
#     -> level-2 autovector (vector 26)
#     -> ROM slot dispatcher
#     -> DoVBLTask(slot 0) at 0x4080A332    per-slot counter at [$0D10 + 10]
#     -> JCrsrTask at 0x4082DF94            only call site for the cursor task
#     -> RawMouse tracks MTemp
#
# THE ONE-LINE TRIAGE: the DoVBLTask COUNTER is the liveness signal.  If it is
# frozen, the slot-IRQ chain is dead and nothing below it can work -- do not
# chase the cursor, the VBL guard or the ioResult stall as CPU/OS bugs until it
# advances.  The usual cause is an in-place JTAG bitstream reload leaving the
# pixel-clock MMCM unrelocked; the fix is a BOARD POWER CYCLE, not a JTAG retry.
#
# DO NOT use `vblank_pending` for this.  It is a STICKY PENDING bit that
# software CLEARS on ack, so a HEALTHY machine servicing VBLs promptly reads 0
# almost always -- it cannot tell "no frames" from "frames handled correctly".
# Keying the verdict off it produced a false "SCANOUT DEAD" on 2026-07-28 while
# DoVBLTask was advancing at ~2500/s.  It is corroborating evidence only.
#
#   tcl source tools/slot_irq_health.tcl ; slot_irq_health
#
# READ RULES baked in (each of these silently corrupted data before):
#   * JTAG reads bypass the write-back D-cache -> `dcache-op push` first.
#   * `rd` SILENTLY ALIGNS DOWN on unaligned addresses -> never read an odd
#     address directly; assemble bytes from the aligned long that contains it.
#   * VIA register reads replicate the byte across all four lanes (0x48 reads
#     back as 0x48484848) -- take the low byte.
#   * Do NOT probe SCSI 53C96 regs here: reg 0x02 pops the FIFO and 0x05
#     clears the pending IRQ, so a "probe" destroys the thing it measures.

proc _sih_u32 {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return -1 }
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}

proc _sih_byte {addr} {
    set w [_sih_u32 [expr {$addr & ~3}]]
    if {$w < 0} { return -1 }
    return [expr {($w >> ((3 - ($addr & 3)) * 8)) & 0xFF}]
}

proc _sih_yn {cond good bad} {
    return [expr {$cond ? $good : $bad}]
}

proc slot_irq_health {{dafb 0xF9800000} {via2 0x50F02000}} {
    dcache-op push
    puts "> slot_irq_health — level-2 (VIA2 slot) IRQ chain"

    set verdict ""
    set dovbl_live -1     ;# -1 unknown, 0 frozen, 1 advancing

    # ── Link 1: is the scanout actually producing frames? ───────────────
    set st [_sih_u32 [expr {$dafb + 0x20}]]
    if {$st < 0} {
        puts "  DAFB REG_IRQ_STATUS +0x20 = <read failed>"
        return
    }
    set vblank_pending [expr {$st & 1}]
    set irq_observable [expr {($st >> 1) & 1}]
    puts [format "  DAFB +0x20 IRQ_STATUS   = 0x%08X   vblank_pending=%d  irq_observable=%d" \
             $st $vblank_pending $irq_observable]
    puts [format "  DAFB +0x108 swatch      = 0x%08X   (bit2 = cursor-scanline pending)" \
             [_sih_u32 [expr {$dafb + 0x108}]]]
    puts [format "  DAFB +0x118 CURSOR_LINE = %d" [_sih_u32 [expr {$dafb + 0x118}]]]
    # NOTE +0x104 SWATCH_CTRL is WRITE-ONLY (reads 0) and +0x1C is
    # REG_IRQ_ENABLE on write but MON_SENSE on read — neither tells you
    # anything about enables.  Don't be fooled by either.

    # ── Link 2: VIA2 — is CA1 armed, flagged, and asserting? ────────────
    set ifr [_sih_u32 [expr {$via2 + 13*0x200}]]
    set ier [_sih_u32 [expr {$via2 + 14*0x200}]]
    if {$ifr >= 0 && $ier >= 0} {
        set ifr_b [expr {$ifr & 0xFF}]
        set ier_b [expr {$ier & 0xFF}]
        puts [format "  VIA2 IFR                = 0x%02X   CA1(bit1)=%d  IRQ(bit7)=%d" \
                 $ifr_b [expr {($ifr_b >> 1) & 1}] [expr {($ifr_b >> 7) & 1}]]
        puts [format "  VIA2 IER                = 0x%02X   CA1 armed=%d" \
                 $ier_b [expr {($ier_b >> 1) & 1}]]
        if {$vblank_pending && [expr {($ier_b >> 1) & 1}] && ![expr {($ifr_b >> 1) & 1}]} {
            append verdict "CA1 armed and the scanout is live, but IFR.CA1 never latched —\n"
            append verdict "           suspect the via2_ca1_in edge itself (task #147)."
        }
    }

    # ── Link 3: VBL guard — stranded guard silently kills ALL deferred
    # tasks, and Ticks++ happens BEFORE the guard test so 60Hz Ticks does
    # NOT prove deferred tasks are running. ─────────────────────────────
    set guard [_sih_byte 0x0160]
    if {$guard >= 0} {
        puts [format "  VBL guard \$0160         = 0x%02X   bit6=%d %s" \
                 $guard [expr {($guard >> 6) & 1}] \
                 [_sih_yn [expr {($guard >> 6) & 1}] \
                    "<- STRANDED: all deferred tasks gated off" "(clear, ok)"]]
    }

    # ── Link 4: is DoVBLTask actually running for the cursor's slot? ────
    # $0D10 holds the VBL queue pointer of the slot that owns the cursor
    # (slot $0 = built-in video).  0x4080A334 does `addql #1,%a1@(10)` on
    # every dispatch, so [$0D10 + 10] is a live service counter.
    set cslot [_sih_u32 0x0D10]
    if {$cslot <= 0} {
        puts "  cursor slot \$0D10       = 0x00000000   <- cursor never registered"
    } else {
        set c1 [_sih_u32 [expr {$cslot + 10}]]
        after 500
        dcache-op push
        set c2 [_sih_u32 [expr {$cslot + 10}]]
        puts [format "  cursor slot \$0D10       = 0x%08X" $cslot]
        set dovbl_live [expr {$c2 > $c1 ? 1 : 0}]
        puts [format "  DoVBLTask count         = %d -> %d %s" $c1 $c2 \
                 [_sih_yn $dovbl_live "(advancing, ok)" \
                    "<- FROZEN: slot VBL never dispatched"]]
        if {!$dovbl_live} {
            append verdict "SLOT-IRQ CHAIN DEAD (DoVBLTask frozen): no level-2 is being\n"
            append verdict "           dispatched, so the cursor AND any deferred-task/ioResult\n"
            append verdict "           stall are EXPECTED consequences, not CPU or OS bugs.\n"
            append verdict "           => try a BOARD POWER CYCLE first (a JTAG reload leaves the\n"
            append verdict "              pixel-clock MMCM unrelocked), then re-run this."
        }
    }

    # ── Link 5: the observable symptom ──────────────────────────────────
    set mt [_sih_u32 0x0828]
    set rm [_sih_u32 0x082C]
    puts [format "  MTemp \$0828             = 0x%08X   (ADB writes here)" $mt]
    puts [format "  RawMouse \$082C          = 0x%08X   %s" $rm \
             [_sih_yn [expr {$mt == $rm}] "(tracking, ok)" "<- not tracking MTemp"]]

    puts ""
    if {$verdict eq ""} {
        puts "> verdict: level-2 chain looks HEALTHY — a cursor/ioResult failure"
        puts ">          here is NOT explained by the slot-IRQ path; dig elsewhere."
    } else {
        puts "> verdict: $verdict"
    }
}

puts "> slot_irq_health.tcl loaded (run: slot_irq_health)"
