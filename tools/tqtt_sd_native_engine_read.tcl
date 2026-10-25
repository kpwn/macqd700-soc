# Read sectors with the standalone hardware-init/native-block TQTT probe.
# The SD card must be power-cycled after any SPI-mode use. The script never
# writes card data. It reports the CRC32 computed over the byte stream read
# from the LiteSDCard DMA RAM, plus the transfer time in 100 MHz cycles.
#
# NATIVE_SD_CARD_POWER_CYCLED=1 vivado -nojournal -nolog -mode batch \
#   -source tools/tqtt_sd_native_engine_read.tcl -tclargs \
#   build/tqtt_sd_native_engine_probe/tqtt_sd_native_engine_probe.bit \
#   build/tqtt_sd_native_engine_probe/tqtt_sd_native_engine_probe.ltx 0 32
# Set NATIVE_SD_NO_PROGRAM=1 when this exact probe is already in FPGA SRAM.
if {[llength $argv] < 2 || [llength $argv] > 4} {
    error "usage: tqtt_sd_native_engine_read.tcl <bit> <ltx> ?lba? ?blocks=32?"
}
if {![info exists ::env(NATIVE_SD_CARD_POWER_CYCLED)] ||
    $::env(NATIVE_SD_CARD_POWER_CYCLED) ne "1"} {
    error "card must be power-cycled out of SPI mode; set NATIVE_SD_CARD_POWER_CYCLED=1 only afterward"
}
set bit [file normalize [lindex $argv 0]]
set ltx [file normalize [lindex $argv 1]]
set lba [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 0}]
set blocks [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 32}]
if {$lba < 0 || $lba > 0xffffffff || $blocks < 1 || $blocks > 32} {
    error "LBA must fit 32 bits and blocks must be 1..32"
}
foreach path [list $bit $ltx] {
    if {![file isfile $path]} {error "missing $path"}
}

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] != 1} {error "expected one JTAG target"}
current_hw_target [lindex $targets 0]
open_hw_target
set dev [lindex [get_hw_devices] 0]
if {![info exists ::env(NATIVE_SD_NO_PROGRAM)] ||
    $::env(NATIVE_SD_NO_PROGRAM) ne "1"} {
    set_property PROGRAM.FILE $bit $dev
    program_hw_devices $dev
} else {
    puts "Attaching to already loaded native block-engine probe"
}
set_property PROBES.FILE $ltx $dev
refresh_hw_device -update_hw_probes true $dev
set vio [lindex [get_hw_vios -quiet] 0]
if {$vio eq ""} {error "native SD VIO missing"}
proc probe {name} {
    foreach item [get_hw_probes -of_objects $::vio -quiet] {
        if {[get_property NAME $item] eq $name} {return $item}
    }
    error "VIO probe $name missing"
}
set p_lba [probe vio_lba]
set p_ctrl [probe vio_control]
set p_capacity [probe card_num_lbas]
set p_rca [probe rca]
set p_status [probe status]
set p_crc [probe result_crc]
set p_cycles [probe result_cycles]
# The probe can be programmed before the physical card reseat. Its automatic
# first init may then have failed on an absent/SPI-mode card; restart the
# controller only after the caller has confirmed the card power cycle.
set_property OUTPUT_VALUE 00000080 $p_ctrl
commit_hw_vio [list $p_ctrl]
after 5
set_property OUTPUT_VALUE 00000000 $p_ctrl
commit_hw_vio [list $p_ctrl]
proc sample {} {
    refresh_hw_vio $::vio
    foreach {key item} [list status $::p_status capacity $::p_capacity \
                              rca $::p_rca crc $::p_crc cycles $::p_cycles] {
        scan [get_property INPUT_VALUE $item] %x value
        dict set result $key $value
    }
    return $result
}
set deadline [expr {[clock milliseconds] + 10000}]
while {1} {
    set snap [sample]
    set status [dict get $snap status]
    if {$status & 0x10} {error [format "native SD init failed: status=0x%08x" $status]}
    if {$status & 0x08} {break}
    if {[clock milliseconds] > $deadline} {
        error [format "native SD init timeout: status=0x%08x" $status]
    }
    after 20
}
set capacity [dict get $snap capacity]
if {$lba + $blocks > $capacity} {
    error [format "request LBA %u + %u exceeds card capacity %u" $lba $blocks $capacity]
}
puts [format "CARD sectors=%u RCA=0x%08x" $capacity [dict get $snap rca]]
set high_speed [expr {($status >> 10) & 1}]
if {[info exists ::env(NATIVE_SD_EXPECT_HS)]} {
    set expected_hs $::env(NATIVE_SD_EXPECT_HS)
    if {$expected_hs ni {0 1}} {error "NATIVE_SD_EXPECT_HS must be 0 or 1"}
    if {$high_speed != $expected_hs} {
        error [format "native SD speed mismatch: expected High Speed=%s, observed=%u, status=0x%08x" \
            $expected_hs $high_speed $status]
    }
}
puts [format "CARD high_speed=%u sd_clk_mhz=%u" $high_speed [expr {$high_speed ? 50 : 25}]]
puts [format "NATIVE_SD_PRE status=0x%08x crc=0x%08x cycles=%u" \
    [dict get $snap status] [dict get $snap crc] [dict get $snap cycles]]

set initial_toggle [expr {$status & 1}]
scan [get_property OUTPUT_VALUE $p_ctrl] %x old_control
set request_toggle [expr {($old_control ^ 0x40) & 0x40}]
set_property OUTPUT_VALUE [format %08x $lba] $p_lba
set_property OUTPUT_VALUE [format %08x [expr {$blocks | ($old_control & 0x40)}]] $p_ctrl
commit_hw_vio [list $p_lba $p_ctrl]
set_property OUTPUT_VALUE [format %08x [expr {$blocks | $request_toggle}]] $p_ctrl
commit_hw_vio [list $p_ctrl]

set deadline [expr {[clock milliseconds] + 10000}]
while {1} {
    set snap [sample]
    set status [dict get $snap status]
    if {($status & 1) != $initial_toggle} {break}
    if {[clock milliseconds] > $deadline} {
        error [format "native SD read timeout: status=0x%08x" $status]
    }
    after 20
}
# VIO readback can catch separate probe values from adjacent hardware cycles.
# A completed transfer is static, so require two identical complete samples.
set deadline [expr {[clock milliseconds] + 2000}]
while {1} {
    after 20
    set again [sample]
    if {$again eq $snap} {break}
    if {[clock milliseconds] > $deadline} {error "native SD result never stabilized"}
    set snap $again
}
set status [dict get $snap status]
puts [format "NATIVE_SD_POST status=0x%08x crc=0x%08x cycles=%u capacity=%u rca=0x%08x" \
    $status [dict get $snap crc] [dict get $snap cycles] \
    [dict get $snap capacity] [dict get $snap rca]]
set received [expr {$status >> 17}]
if {$status & 2} {error [format "native SD block error: status=0x%08x" $status]}
if {$received != $blocks * 512} {
    error [format "short native read: %u of %u bytes" $received [expr {$blocks * 512}]]
}
if {[dict get $snap cycles] == 0} {error "native SD cycle counter is zero"}
puts [format "PASS LBA=%u blocks=%u bytes=%u CRC32=0x%08x DMA_cycles=%u DMA_MBps=%.3f" \
    $lba $blocks $received [dict get $snap crc] [dict get $snap cycles] \
    [expr {$received * 100.0 / [dict get $snap cycles]}]]
close_hw_target
disconnect_hw_server
close_hw_manager
