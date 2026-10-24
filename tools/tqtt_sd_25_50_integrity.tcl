# Repeated read-only 25/50 MHz SPI comparison on the standalone TQTT SD probe.
# Usage: vivado -nojournal -nolog -mode batch -source this_file \
#          -tclargs <sdmin_50.bit> <sdmin_50.ltx> ?runs_per_mode?
# Each run reloads the probe's 2048-sector ROM window and compares its CRC32.
# This checks that window only; it does not qualify the full card or writes.

if {[llength $argv] < 2 || [llength $argv] > 3} {
    error "usage: tqtt_sd_25_50_integrity.tcl <bit> <ltx> ?runs_per_mode?"
}
set bit_file [file normalize [lindex $argv 0]]
set ltx_file [file normalize [lindex $argv 1]]
set runs [expr {[llength $argv] == 3 ? [lindex $argv 2] : 5}]
if {![string is integer -strict $runs] || $runs < 1} {
    error "runs_per_mode must be a positive integer"
}
foreach path [list $bit_file $ltx_file] {
    if {![file isfile $path]} { error "missing probe artifact: $path" }
}

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} { error "expected one JTAG target, found [llength $targets]" }
current_hw_target [lindex $targets 0]
open_hw_target
set devices [get_hw_devices]
if {[llength $devices] != 1} { error "expected one JTAG device, found [llength $devices]" }
set dev [lindex $devices 0]
current_hw_device $dev
set_property PROGRAM.FILE $bit_file $dev
set_property PROBES.FILE $ltx_file $dev
set_property FULL_PROBES.FILE $ltx_file $dev
program_hw_devices $dev
refresh_hw_device -update_hw_probes true $dev
set vios [get_hw_vios -quiet]
if {[llength $vios] != 1} { error "expected one VIO, found [llength $vios]" }
set vio [lindex $vios 0]

proc probe {vio name} {
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq $name} { return $p }
    }
    error "missing VIO probe $name"
}
set ctrl [probe $vio vio_ctrl]
set flags [probe $vio vio_flags]
set diag [probe $vio vio_diag]
set crc16 [probe $vio vio_crc]
set crc32 [probe $vio rom_crc32_final]

proc ctrl_write {p val} {
    set_property OUTPUT_VALUE [format %x $val] $p
    commit_hw_vio [list $p]
}
proc sample {vio flags diag crc16 crc32} {
    refresh_hw_vio $vio
    foreach {name p} [list flags $flags diag $diag crc16 $crc16 crc32 $crc32] {
        set hex [get_property INPUT_VALUE $p]
        if {[scan $hex %x value] != 1} { error "invalid $name VIO value: $hex" }
        dict set result $name $value
    }
    return $result
}

set reference_crc ""
foreach {mode cfg} {25 0 50 4} {
    for {set i 1} {$i <= $runs} {incr i} {
        # Bit 0 is soft reset; bit 2 permits the negotiated CMD6 HS clock.
        ctrl_write $ctrl [expr {$cfg | 1}]
        after 100
        ctrl_write $ctrl $cfg
        set start [clock milliseconds]
        set deadline [expr {$start + 30000}]
        while {1} {
            set s [sample $vio $flags $diag $crc16 $crc32]
            set f [dict get $s flags]
            set d [dict get $s diag]
            if {$f & 0x20} {
                error [format "%d MHz run %d: SD error, sector=%d cause=%d" \
                    $mode $i [expr {$d & 0xffff}] [expr {($d >> 16) & 7}]]
            }
            if {$f & 0x40} { break }
            if {[clock milliseconds] > $deadline} {
                error [format "%d MHz run %d: timeout, sector=%d" \
                    $mode $i [expr {$d & 0xffff}]]
            }
            after 25
        }
        set elapsed [expr {[clock milliseconds] - $start}]
        if {($d & 0xffff) != 2048} {
            error [format "%d MHz run %d: loaded after only %d sectors" \
                $mode $i [expr {$d & 0xffff}]]
        }
        set retries [expr {($d >> 19) & 15}]
        if {$retries != 0} {
            error [format "%d MHz run %d: %d whole-run CMD18 retries" \
                $mode $i $retries]
        }
        set actual_hs [expr {($f >> 3) & 1}]
        if {$actual_hs != ($mode == 50)} {
            error [format "%d MHz run %d: unexpected actual HS mode %d" \
                $mode $i $actual_hs]
        }
        set block_crc [dict get $s crc16]
        if {($block_crc & 0xffff) != (($block_crc >> 16) & 0xffff)} {
            error [format "%d MHz run %d: last block CRC16 mismatch 0x%08x" \
                $mode $i $block_crc]
        }
        set c [dict get $s crc32]
        if {$reference_crc eq ""} { set reference_crc $c }
        if {$c != $reference_crc} {
            error [format "%d MHz run %d: CRC32 mismatch 0x%08x vs 0x%08x" \
                $mode $i $c $reference_crc]
        }
        puts [format "PASS mode=%dMHz run=%d sectors=%d crc32=0x%08x elapsed_ms=%d" \
            $mode $i [expr {$d & 0xffff}] $c $elapsed]
        flush stdout
    }
}
puts "PASS same 2048-sector CRC32 at 25 and 50 MHz across $runs runs per mode"
