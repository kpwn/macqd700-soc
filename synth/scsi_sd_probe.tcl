# synth/scsi_sd_probe.tcl — is the SCSI→SD disk read actually completing?
#
# Companion to the vio_scsi_sd probe (probe_in22, rtl/soc/fpga_top_debug_vio.vh).
#
# WHY THIS EXISTS
# ---------------
# Mac OS boot stalls forever polling ioResult on a File Manager disk read
# (the request's param block has ioCompletion=0, so nothing but the driver
# writing the result can ever release it).  The open question was whether
# the SCSI→SD backing store ever finishes those reads — and that state was
# completely unobservable:
#
#   • JTAG-AXI CANNOT read the SCSI registers.  peripheral_bus exposes them
#     byte-granular (scsi_addr[8:0], 8-bit data) and JTAG-AXI is a word-only
#     master — the same limitation adb_inject_verify.tcl documents for the
#     ADB byte map.  Word reads there return zeros that mean nothing.
#   • NCR5380 register reads have SIDE EFFECTS (reg 7 clears parity/IRQ), so
#     probing them from JTAG would perturb the driver mid-transaction.
#
# Hence a passive VIO probe instead.
#
# HOW TO READ THE RESULT
# ----------------------
# scsi.v issues a backing-store request with sd_go and waits for sd_done or
# sd_error. The probe counts completions and errors and freezes sd_ctrl's
# first error details before later requests can overwrite them. Sample it
# twice while stalled: a frozen busy bit indicates an uncompleted request;
# a valid error snapshot identifies a request that explicitly failed.
#
# Counters are edge-detected and SATURATING (they stop at max rather than
# wrapping) — a wrapped small number after a long boot reads exactly like
# "barely any I/O", which is the wrong conclusion.
#
# Usage (from the jtag_repl.tcl REPL, which already holds the target):
#   tcl source synth/scsi_sd_probe.tcl
#   scsi-sd
# or standalone:
#   vivado -mode batch -source synth/scsi_sd_probe.tcl -tclargs <bit> <ltx>

proc scsi_sd_raw {} {
    set vios [get_hw_vios -quiet]
    if {[llength $vios] == 0} { return "NO_VIO" }
    set vio [lindex $vios 0]
    refresh_hw_vio $vio
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq "vio_scsi_sd"} {
            return [get_property INPUT_VALUE $p]
        }
    }
    return "NOT_FOUND"
}

# vio_scsi_sd layout (76 bits), MSB..LSB:
#   [75:44] err_lba       first failing SD LBA
#   [43:28] done_count    completions seen
#   [27:20] err_count     errors seen
#   [19:16] err_cause     sd_ctrl ERR_* classification
#   [15:12] err_cmd       sd_ctrl SC_CMD* encoding
#   [11:4]  err_detail    R1, read token poll count, or write response byte
#   [3]     err_valid     first-error snapshot valid
#   [2]     sd_busy       backing store busy right now
#   [1]     scsi_irq      SCSI IRQ to VIA2
#   [0]     scsi_drq      SCSI DRQ
proc scsi_sd_decode {hex} {
    # 76 bits does not fit a Tcl 64-bit int; slice the hex string instead.
    set h [string trim $hex]
    regsub {^0[xX]} $h "" h
    set h [format %019s $h]
    set h [string map {" " "0"} $h]

    set v 0
    foreach ch [split $h ""] { set v [expr {$v * 16 + [scan $ch %x]}] }

    set err_lba  [expr {($v >> 44) & 0xFFFFFFFF}]
    set done     [expr {($v >> 28) & 0xFFFF}]
    set err      [expr {($v >> 20) & 0xFF}]
    set cause    [expr {($v >> 16) & 0xF}]
    set cmd      [expr {($v >> 12) & 0xF}]
    set detail   [expr {($v >>  4) & 0xFF}]
    set valid    [expr {($v >>  3) & 1}]
    set busy     [expr {($v >>  2) & 1}]
    set irq      [expr {($v >>  1) & 1}]
    set drq      [expr {$v & 1}]

    if {$valid} {
        if {$cause == 4} {
            if {$detail == 0x0B} {
                set verdict "FIRST ERROR CAPTURED: SD rejected write CRC"
            } elseif {$detail == 0x0D} {
                set verdict "FIRST ERROR CAPTURED: SD rejected write data"
            } elseif {$detail == 0xFF} {
                set verdict "FIRST ERROR CAPTURED: SD write response timed out"
            } else {
                set verdict "FIRST ERROR CAPTURED: unexpected SD write response"
            }
        } else {
            set verdict "FIRST ERROR CAPTURED: decode cause/cmd/detail; vio_boot_crc holds computed/received SCSI CRC16"
        }
    } elseif {$busy} {
        set verdict "backing-store request is still live; sample again to distinguish progress from a wedge"
    } else {
        set verdict "no backing-store error captured"
    }

    return [format \
"  err_lba = %u (0x%08X)
  done=%u  err=%u  cause=%u  cmd=%u  detail=0x%02X  valid=%d
  busy=%d irq=%d drq=%d
  => %s" \
        $err_lba $err_lba $done $err $cause $cmd $detail $valid \
        $busy $irq $drq $verdict]
}

proc scsi-sd {} {
    set raw [scsi_sd_raw]
    if {$raw eq "NOT_FOUND"} {
        puts "> vio_scsi_sd not present -- bitstream predates probe_in22.
  Rebuild with ENABLE_VIO=1 (and note synth/vivado.tcl's probe_map=v20
  marker forces the VIO IP to regenerate)."
        return
    }
    if {$raw eq "NO_VIO"} { puts "> no VIO on target"; return }
    puts "> vio_scsi_sd = 0x$raw"
    puts [scsi_sd_decode $raw]
}

# Sample twice with a delay to distinguish "abandoned" from "retrying".
proc scsi-sd-watch {{n 4} {delay_ms 2000}} {
    for {set i 0} {$i < $n} {incr i} {
        puts "--- sample [expr {$i + 1}]/$n ---"
        scsi-sd
        if {$i < $n - 1} { after $delay_ms }
    }
}

puts "scsi_sd_probe.tcl loaded."
puts "  scsi-sd                  -- decode the SCSI->SD counters once"
puts "  scsi-sd-watch \[n\] \[ms\]   -- sample repeatedly (frozen == abandoned)"
