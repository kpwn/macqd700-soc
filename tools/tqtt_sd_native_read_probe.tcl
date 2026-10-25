# Read-only native four-bit TQTT SD probe.
# Run only after SD card power was cycled since any SPI-mode use. Set
# NATIVE_SD_NO_PROGRAM=1 to attach to an already loaded native probe; this
# lets the Mac image be replaced first, then the card safely reseated.
# Usage: vivado -nojournal -nolog -mode batch -source this_file \
#   -tclargs <native.bit> <native.ltx> ?start_lba? ?blocks? ?mhz?
# Blocks must fit the 16 KiB local RAM. This script never writes card data.
if {[llength $argv] < 2 || [llength $argv] > 5} {
    error "usage: tqtt_sd_native_read_probe.tcl <bit> <ltx> ?start_lba? ?blocks<=32? ?25|50?"
}
set bit [file normalize [lindex $argv 0]]
set ltx [file normalize [lindex $argv 1]]
set start_lba [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 0}]
set blocks [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 32}]
set speed [expr {[llength $argv] >= 5 ? [lindex $argv 4] : 25}]
if {$blocks < 1 || $blocks > 32 || $speed ni {25 50}} {
    error "blocks must be 1..32 and speed must be 25 or 50"
}
foreach file [list $bit $ltx] {if {![file isfile $file]} {error "missing $file"}}

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
    puts "Attaching to existing native SD probe without programming SRAM"
}
set_property PROBES.FILE $ltx $dev
refresh_hw_device -update_hw_probes true $dev
set vio [lindex [get_hw_vios -quiet] 0]
if {$vio eq ""} {error "native SD VIO missing"}
proc p {name} {
    foreach probe [get_hw_probes -of_objects $::vio -quiet] {
        if {[get_property NAME $probe] eq $name} {return $probe}
    }
    error "VIO probe $name missing"
}
set addr_probe [p vio_addr]
set data_probe [p vio_wdata]
set ctrl_probe [p vio_ctrl]
set rsp_probe [p wb_rdata]
set status_probe [p vio_status]
set crc_probe [p vio_crc]
set cycles_probe [p vio_dma_cycles]
set mem_addr_probe [p vio_mem_addr]
set mem_data_probe [p vio_mem_data]
set ::toggle 0

proc sample_status {} {
    refresh_hw_vio $::vio
    scan [get_property INPUT_VALUE $::status_probe] %x value
    return $value
}
proc csr {address {value ""}} {
    set_property OUTPUT_VALUE [format %08x $address] $::addr_probe
    if {$value ne ""} {
        set_property OUTPUT_VALUE [format %08x $value] $::data_probe
        set we 2
        commit_hw_vio [list $::addr_probe $::data_probe]
    } else {
        set we 0
        commit_hw_vio [list $::addr_probe]
    }
    set ::toggle [expr {$::toggle ^ 1}]
    set_property OUTPUT_VALUE [format %x [expr {$we | $::toggle}]] $::ctrl_probe
    commit_hw_vio [list $::ctrl_probe]
    set deadline [expr {[clock milliseconds] + 5000}]
    while {1} {
        set status [sample_status]
        if {(($status >> 2) & 1) == $::toggle} {break}
        if {[clock milliseconds] > $deadline} {
            error [format "CSR timeout addr=0x%04x status=0x%08x" $address $status]
        }
        after 10
    }
    if {($status >> 3) & 1} {error [format "CSR bus error addr=0x%04x" $address]}
    scan [get_property INPUT_VALUE $::rsp_probe] %x result
    return $result
}
proc cmd {index arg response {transfer 0}} {
    csr 0x1000 $arg
    csr 0x1004 [expr {($index << 8) | ($transfer << 5) | $response}]
    csr 0x1008 1
    set deadline [expr {[clock milliseconds] + 3000}]
    while {1} {
        set event [csr 0x101c]
        if {$event & 1} {break}
        if {[clock milliseconds] > $deadline} {
            error [format "CMD%d timeout waiting for event" $index]
        }
    }
    if {$event & 0xc} {error [format "CMD%d error event=0x%x" $index $event]}
    return [csr 0x1018]
}
proc data_done {} {
    set deadline [expr {[clock milliseconds] + 10000}]
    while {1} {
        set event [csr 0x1020]
        if {$event & 1} {break}
        if {[clock milliseconds] > $deadline} {error "SD data timeout"}
    }
    if {$event & 0xc} {error [format "SD data error event=0x%x" $event]}
}
proc dma_setup {bytes} {
    csr 0x080c 0
    csr 0x0800 0
    csr 0x0804 0
    csr 0x0808 $bytes
    csr 0x080c 1
}
proc wait_dma {} {
    set deadline [expr {[clock milliseconds] + 10000}]
    while {![csr 0x0810]} {
        if {[clock milliseconds] > $deadline} {error "SD DMA timeout"}
    }
}
proc reset_dma_stats {} {
    set_property OUTPUT_VALUE [format %x [expr {$::toggle | 8}]] $::ctrl_probe
    commit_hw_vio [list $::ctrl_probe]
    after 1
    set_property OUTPUT_VALUE [format %x $::toggle] $::ctrl_probe
    commit_hw_vio [list $::ctrl_probe]
}
proc ram_word {index} {
    set_property OUTPUT_VALUE [format %03x $index] $::mem_addr_probe
    commit_hw_vio [list $::mem_addr_probe]
    refresh_hw_vio $::vio
    scan [get_property INPUT_VALUE $::mem_data_probe] %x value
    return $value
}

puts "Native SD reset; card must have been power-cycled since SPI mode"
csr 0x2804 256
csr 0x2808 1
after 5
cmd 0 0 0
cmd 8 0x1aa 1
csr 0x2804 4 ;# 100 MHz / 4 = 25 MHz
set ready 0
for {set attempt 0} {$attempt < 1000} {incr attempt} {
    cmd 55 0 1
    set ocr [cmd 41 0x70ff8000 3]
    if {$ocr & 0x80000000} {set ready 1; break}
    after 1
}
if {!$ready} {error "ACMD41 card never became ready"}
puts [format "OCR=0x%08x" $ocr]
cmd 2 0 2
set rca_response [cmd 3 0 1]
set rca [expr {$rca_response & 0xffff0000}]
puts [format "RCA=0x%08x" $rca]
cmd 7 $rca 3
cmd 55 $rca 1
cmd 6 2 1 ;# ACMD6, four data wires

if {$speed == 50} {
    dma_setup 64
    csr 0x1024 64
    csr 0x1028 1
    cmd 6 0x80fffff1 1 1 ;# switch access mode to High Speed
    data_done
    wait_dma
    set group_support [expr {([ram_word 3] >> 16) & 0xff}]
    set selected [expr {([ram_word 4] >> 24) & 0x0f}]
    if {!(($group_support & 2) && $selected == 1)} {
        error [format "CMD6 did not select High Speed: support=0x%02x selected=%d" \
            $group_support $selected]
    }
    csr 0x2804 2 ;# 100 MHz / 2 = 50 MHz
}
cmd 16 512 1

set bytes [expr {$blocks * 512}]
reset_dma_stats
dma_setup $bytes
csr 0x1024 512
csr 0x1028 $blocks
set start [clock milliseconds]
if {$blocks == 1} {
    cmd 17 $start_lba 1 1
} else {
    cmd 18 $start_lba 1 1
}
data_done
wait_dma
if {$blocks > 1} {cmd 12 0 3}
set elapsed [expr {[clock milliseconds] - $start}]
set status [sample_status]
set words [expr {$status >> 8}]
if {$words != ($bytes / 4)} {
    error [format "DMA word count=%d expected=%d" $words [expr {$bytes / 4}]]
}
scan [get_property INPUT_VALUE $crc_probe] %x crc
scan [get_property INPUT_VALUE $cycles_probe] %x cycles
set wire_mib_s [expr {$cycles > 0 ? ($bytes * 100.0 / $cycles) / 1.048576 : 0}]
puts [format "PASS native=%dMHz lba=%d blocks=%d bytes=%d crc32=0x%08x host_elapsed_ms=%d wire_cycles=%d wire_MiBps=%.2f" \
    $speed $start_lba $blocks $bytes $crc $elapsed $cycles $wire_mib_s]
close_hw_target
disconnect_hw_server
close_hw_manager
