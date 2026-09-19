# synth/adb_inject_verify.tcl — end-to-end ADB liveness check (2026-07-25)
#
# Verifies the LISTEN-payload fix (adb_phy.v ST_LSN_GAP/ST_LSN_LOW,
# commit d7c57d2) actually made the ADB chain work on real hardware.
#
# Before that fix adb_phy never captured the payload the host sends after
# a LISTEN command, so dev_listen_valid was permanently 0 and devices
# could never leave their default addresses.  Mac OS's ADBReInit
# RELOCATES devices via LISTEN R3, so its device table named addresses
# our devices never moved to, every poll hit a dead address, and an
# injected mouse event would set event_pending and then sit there
# forever because no TALK R0 ever reached the mouse to consume it.
#
# THE PASS CRITERION IS THE DRAIN, NOT THE INJECT.  Writing MOUSE_DX
# always sets event_pending=1 (that is just the inject register doing its
# job, and it did that even when ADB was completely dead).  What proves
# the chain is alive is event_pending returning to 0 on its own, because
# that only happens when a TALK R0 actually reaches the mouse and
# consumes the report.
#
# ADB inject MMIO — word-aligned aliases (rtl/mac/adb_inject.v).  The
# byte-granular map at +0x00..0x05 is unusable from here: JTAG-AXI is a
# word-only master and cannot issue the unaligned byte writes it needs.
#   0x50011010  W KBD_ENQUEUE   R KBD_STATUS
#   0x50011014  W MOUSE_BTN     R MOUSE_STATUS  (bit0 = event_pending)
#   0x50011018  W MOUSE_DX      (signed 8-bit)
#   0x5001101C  W MOUSE_DY      (signed 8-bit)
#
# Usage (from the jtag_repl.tcl REPL, which already holds the target):
#   tcl source synth/adb_inject_verify.tcl
# or standalone:
#   vivado -mode batch -source synth/adb_inject_verify.tcl -tclargs <bit> <ltx>

set ADBINJ_KBD    0x50011010
set ADBINJ_MSBTN  0x50011014
set ADBINJ_MSDX   0x50011018
set ADBINJ_MSDY   0x5001101C

# Decode of vio_adb_dbg (fpga_top_debug_vio.vh probe_in21... probe_in20):
#   [19:16] adb_dbg_state  -- NOTE: this is u_adb_modem's state, and
#           fpga_top_peripherals.vh documents u_adb_modem as INERT / not
#           on the real boot path.  It reads 0 always; ignore it.
#   [15]    dev_cmd_valid
#   [14:11] dev_cmd_addr   <- driven by u_adb_phy, the real path
#   [10:8]  dev_cmd_op     <- 0=RESET 1=FLUSH 2=LISTEN_R0 3=LISTEN_R3
#                             4=TALK_R0 5=TALK_R3
#   [7]kbd_resp_valid [6]kbd_resp_empty [5]kbd_srq
#   [4]ms_resp_valid  [3]ms_resp_empty  [2]ms_srq
#   [1]dev_listen_valid  <- was ALWAYS 0 before the fix
proc decode_adb_dbg {hex} {
    scan $hex %x v
    set state [expr {($v >> 16) & 0xF}]
    set cmdv  [expr {($v >> 15) & 0x1}]
    set addr  [expr {($v >> 11) & 0xF}]
    set op    [expr {($v >>  8) & 0x7}]
    set opname [lindex {RESET FLUSH LISTEN_R0 LISTEN_R3 TALK_R0 TALK_R3 op6 op7} $op]
    return [format \
        "cmd_valid=%d addr=%2d op=%d(%s) | kbd{v=%d e=%d srq=%d} ms{v=%d e=%d srq=%d} listen_valid=%d (modem_state=%d, inert)" \
        $cmdv $addr $op $opname \
        [expr {($v >> 7) & 1}] [expr {($v >> 6) & 1}] [expr {($v >> 5) & 1}] \
        [expr {($v >> 4) & 1}] [expr {($v >> 3) & 1}] [expr {($v >> 2) & 1}] \
        [expr {($v >> 1) & 1}] $state]
}

proc adb_probe {} {
    set vios [get_hw_vios -quiet]
    if {[llength $vios] == 0} { return "NO_VIO" }
    set vio [lindex $vios 0]
    refresh_hw_vio $vio
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq "vio_adb_dbg"} {
            return [get_property INPUT_VALUE $p]
        }
    }
    return "NOT_FOUND"
}

# Sample the ADB command stream for a few seconds and report which
# addresses/ops the firmware is actually issuing.  A healthy bus shows
# TALK R0 aimed at the addresses the devices really hold; the broken
# state showed TALK R0 to address 1, where nothing lives.
proc adb_watch {{n 12}} {
    puts "=== ADB command stream ($n samples) ==="
    for {set i 0} {$i < $n} {incr i} {
        set raw [adb_probe]
        if {$raw eq "NOT_FOUND" || $raw eq "NO_VIO"} {
            puts "  vio_adb_dbg: $raw (bitstream predates probe_in20?)"
            return
        }
        puts [format "  \[%2d\] 0x%s  %s" $i $raw [decode_adb_dbg $raw]]
        after 250
    }
}

proc ms_status {} { return [rd $::ADBINJ_MSBTN] }

# Inject a mouse delta and watch event_pending drain.  Returns 1 if it
# drained (ADB chain alive), 0 if it stuck (still broken).
proc adb_inject_mouse {{dx 20} {dy 10} {settle_ms 3000}} {
    puts "=== injecting mouse delta dx=$dx dy=$dy ==="
    set before [ms_status]
    puts "  MOUSE_STATUS before inject: 0x$before"

    wr $::ADBINJ_MSDX [format 0x%08X [expr {$dx & 0xFF}]]
    wr $::ADBINJ_MSDY [format 0x%08X [expr {$dy & 0xFF}]]

    set armed [ms_status]
    puts "  MOUSE_STATUS after  inject: 0x$armed  (bit0=1 expected -- the"
    puts "                                          inject register alone"
    puts "                                          does this even when"
    puts "                                          ADB is dead)"

    # Poll for the drain.  This is the real signal: event_pending only
    # clears when a TALK R0 genuinely reaches the mouse.
    set step 250
    set waited 0
    while {$waited < $settle_ms} {
        after $step
        incr waited $step
        set now [ms_status]
        scan $now %x nv
        if {($nv & 1) == 0} {
            puts "  MOUSE_STATUS drained to 0x$now after ${waited}ms"
            puts "  RESULT: PASS -- a TALK R0 reached the mouse and consumed"
            puts "          the report.  The ADB chain is alive."
            return 1
        }
    }
    set final [ms_status]
    puts "  MOUSE_STATUS still 0x$final after ${settle_ms}ms"
    puts "  RESULT: FAIL -- event_pending never drained, so no TALK R0 is"
    puts "          reaching the mouse.  Check adb_watch output for which"
    puts "          address the firmware is polling."
    return 0
}

proc adb_verify {} {
    adb_watch 12
    set ok [adb_inject_mouse 20 10 3000]
    puts "=== post-inject ADB command stream ==="
    adb_watch 8
    return $ok
}

puts "adb_inject_verify.tcl loaded."
puts "  adb_watch \[n\]                 -- sample the ADB command stream"
puts "  adb_inject_mouse \[dx dy ms\]   -- inject + wait for the drain"
puts "  adb_verify                     -- watch, inject, watch"
