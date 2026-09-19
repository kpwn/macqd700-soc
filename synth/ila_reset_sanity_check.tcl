# synth/ila_reset_sanity_check.tcl — verify raw hw_axi reset-halt-exc(2)
# replication actually halts the CPU the same way jtag_repl.tcl's
# version does (task #101, 2026-07-07). No ILA involved -- just fires
# the reset sequence, waits, and reads back HALT_REASON/HALT_HIT_PC/
# EXC_VEC/STATUS directly via hw_axi reads.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_reset_sanity_check.tcl \
#       -tclargs <ltx_file> [wait_s]

set ltx_file [lindex $argv 0]
set wait_s   [expr {[llength $argv] >= 2 ? [lindex $argv 1] : 10}]

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
current_hw_target [lindex $targets 0]
open_hw_target
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx_file $dev
refresh_hw_device $dev -update_hw_probes true

set axi [lindex [get_hw_axis -of_objects $dev] 0]
puts "=== hw_axi master: $axi ==="

proc dbg_wr_raw {axi addr data} {
    create_hw_axi_txn -quiet -force _w $axi -type WRITE \
        -address [format %08X $addr] -data [format %08X $data]
    run_hw_axi -quiet _w
}
proc dbg_rd_raw {axi addr} {
    create_hw_axi_txn -quiet -force _r $axi -type READ \
        -address [format %08X $addr]
    run_hw_axi -quiet _r
    return [get_property DATA [get_hw_axi_txns _r]]
}

set DBG_BASE 0x50900000
puts "=== Firing reset-halt-exc(vec=2) at [clock format [clock seconds]] ==="
dbg_wr_raw $axi [expr {$DBG_BASE + 0x008}] [expr {0x10 | 0x20}]
after 800
for {set lane 0} {$lane < 8} {incr lane} {
    dbg_wr_raw $axi [expr {$DBG_BASE + 0x060 + $lane*4}] 0
    after 20
}
dbg_wr_raw $axi [expr {$DBG_BASE + 0x060}] 0x4
after 20
dbg_wr_raw $axi [expr {$DBG_BASE + 0x03C}] [expr {0x40 | 0x4}]
after 20
# Read back HALT_CTL immediately to confirm the arm actually landed
# BEFORE releasing hold -- if this doesn't read back 0x44, the arm
# itself silently failed regardless of any release-timing issue.
set halt_ctl_check [dbg_rd_raw $axi [expr {$DBG_BASE + 0x03C}]]
puts "=== HALT_CTL readback before release: $halt_ctl_check (expect 00000044) ==="
dbg_wr_raw $axi [expr {$DBG_BASE + 0x008}] 0
puts "=== Reset sequence complete at [clock format [clock seconds]], waiting ${wait_s}s ==="

after [expr {$wait_s * 1000}]

set status     [dbg_rd_raw $axi [expr {$DBG_BASE + 0x00C}]]
set halt_reason [dbg_rd_raw $axi [expr {$DBG_BASE + 0x040}]]
set halt_hit_pc [dbg_rd_raw $axi [expr {$DBG_BASE + 0x044}]]
set exc_vec     [dbg_rd_raw $axi [expr {$DBG_BASE + 0x024}]]

puts "=== STATUS=$status HALT_REASON=$halt_reason HALT_HIT_PC=$halt_hit_pc EXC_VEC=$exc_vec ==="
puts "=== DONE ==="
