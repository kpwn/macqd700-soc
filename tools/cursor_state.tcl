# cursor_state.tcl — dump the Mac OS cursor/mouse globals in one shot.
#
# WHY: with the machine at the Finder (see tools/unstick.tcl), injected ADB
# motion moves MTemp but NOT RawMouse — the cursor task is not propagating
# MTemp -> RawMouse -> screen.  The hardware side is already cleared:
#   * the Mac arrow is SOFTWARE-drawn by QuickDraw (no sprite plane needed,
#     and rtl/mac/video.v correctly has none);
#   * DAFB's "cursor" registers are a SCANLINE INTERRUPT and are implemented
#     correctly (video.v: swatch_cursor_pending / ACK at +0x10C).
# So the remaining question is OS state, which is what this dumps.
#
#   tcl source tools/cursor_state.tcl ; cursor_state
#
# READ RULES baked in (each of these silently corrupted data earlier):
#   * `rd` SILENTLY ALIGNS DOWN on unaligned addresses — never read an odd
#     address directly; assemble from aligned longs.
#   * JTAG reads bypass the write-back D-cache — `dcache-op push` first.
#   * Several of these globals are ODD-ALIGNED (JCrsrTask at $08EE), so they
#     straddle two aligned longs.

proc _cs_u32 {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return -1 }
    if {[scan $v %x out] != 1} { return -1 }
    return $out
}

# Byte at an arbitrary address, via the aligned long that contains it.
proc _cs_byte {addr} {
    set w [_cs_u32 [expr {$addr & ~3}]]
    if {$w < 0} { return -1 }
    return [expr {($w >> ((3 - ($addr & 3)) * 8)) & 0xFF}]
}

# 32-bit big-endian value at a possibly-odd address.
proc _cs_long {addr} {
    set b0 [_cs_byte $addr]
    set b1 [_cs_byte [expr {$addr + 1}]]
    set b2 [_cs_byte [expr {$addr + 2}]]
    set b3 [_cs_byte [expr {$addr + 3}]]
    if {$b0 < 0 || $b1 < 0 || $b2 < 0 || $b3 < 0} { return -1 }
    return [expr {($b0 << 24) | ($b1 << 16) | ($b2 << 8) | $b3}]
}

proc _cs_point {addr name} {
    set v [_cs_long $addr]
    if {$v < 0} { puts [format "  %-11s \$%04X = <read failed>" $name $addr] ; return }
    puts [format "  %-11s \$%04X = 0x%08X   v=%d h=%d" \
             $name $addr $v [expr {($v >> 16) & 0xFFFF}] [expr {$v & 0xFFFF}]]
}

proc cursor_state {} {
    dcache-op push
    puts "> cursor_state:"
    _cs_point 0x0828 MTemp        ;# low-level interim mouse location (ADB writes here)
    _cs_point 0x082C RawMouse     ;# un-jerkified mouse location (cursor task writes)
    _cs_point 0x0830 Mouse        ;# processed mouse location
    foreach {addr name note} {
        0x08CE CrsrNew    "cursor changed flag"
        0x08CF CrsrCouple "MUST be non-zero for the cursor to track the mouse"
        0x08D0 CrsrState  "negative = cursor hidden"
        0x08D2 CrsrObscure "obscured flag"
        0x08D3 CrsrScale  "scaling flag"
    } {
        set b [_cs_byte $addr]
        if {$b < 0} {
            puts [format "  %-11s \$%04X = <read failed>" $name $addr]
        } else {
            puts [format "  %-11s \$%04X = 0x%02X   (%s)" $name $addr $b $note]
        }
    }
    foreach {addr name} {0x08EE JCrsrTask 0x0B22 CrsrThresh} {
        set v [_cs_long $addr]
        if {$v < 0} {
            puts [format "  %-11s \$%04X = <read failed>" $name $addr]
        } else {
            puts [format "  %-11s \$%04X = 0x%08X" $name $addr $v]
        }
    }
    set t [_cs_long 0x016A]
    puts [format "  %-11s \$%04X = %d   (should climb ~60/s)" Ticks 0x016A $t]
}

# ── dafb_cursor_state — the HARDWARE half of the cursor question ────────────
# The Mac cursor task is driven by DAFB's CURSOR SCANLINE interrupt, not by
# VBL (see rtl/mac/video.v: "Mac OS drives the built-in video slot interrupt
# from the CURSOR SCANLINE source ... acks it by writing +0x10C").  Ticks comes
# from VIA1 and keeps running at 60 Hz regardless, so the cursor task can be
# dead while every other liveness signal looks healthy.
#
# If SWATCH_CTRL bit 2 (cursor IRQ enable) is CLEAR, the interrupt never fires
# and the cursor task never runs — which is exactly the observed symptom
# (MTemp moves, RawMouse frozen).
#
# NOTE the countdown in video.v is a phase-BLIND linear approximation:
#     cursor_line == 0 -> 199521 cycles  (~2.0 ms @100 MHz = ~500 Hz!)
#     else             -> cursor_line*2751 + 196770  (line 500 -> ~15.7 ms)
# so a cursor_line of 0 would generate interrupts ~8x faster than the frame
# rate.  Worth knowing what the OS actually programmed.
proc dafb_cursor_state {{base 0xF9800000}} {
    puts "> dafb_cursor_state (base [format 0x%08X $base]):"
    foreach {off name note} {
        0x0104 SWATCH_CTRL   "bit0=VBL enable  bit2=CURSOR IRQ enable"
        0x010C CURSOR_ACK    "write to ack the cursor-scanline IRQ"
        0x0114 VBL_ACK       "write to ack VBL"
        0x0118 CURSOR_LINE   "scanline the cursor IRQ is scheduled at"
        0x001C IRQ_SENSE     "MAME: reads back monitor sense, NOT an enable"
    } {
        set v [rd [expr {$base + $off}]]
        puts [format "  %-12s +%04X = 0x%s   (%s)" $name $off $v $note]
    }
}

puts "> cursor_state.tcl loaded (cursor_state / dafb_cursor_state)"
