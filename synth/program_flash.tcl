## program_flash.tcl — write the bitstream into the KU5P SPIx4 boot flash
##
## Usage:
##   vivado -mode batch -nojournal -nolog \
##          -source synth/program_flash.tcl \
##          -tclargs <path/to/fpga_top.bit> [<cfgmem-part>]
##
## Default cfgmem part: mx25u51245g-spi-x1_x2_x4 (Macronix MX25U51245G,
## 512 Mb / 64 MB QSPI — confirmed 2026-04-30 to match this board).
## Override with the second tclargs slot if your board has a different flash.
## Earlier default `mt25qu256-spi-x1_x2_x4` (Micron 256 Mb) was wrong for
## our board and triggered "Flash Programming Unsuccessful: Failure to
## set flash parameters" (Labtools 27-3347).
##
## What it does:
##   1. Translates fpga_top.bit -> .mcs at 0x0 with SPIx4 stitching.
##   2. Opens hw_manager, connects to local hw_server, attaches the KU5P.
##   3. Erases + programs + verifies the entire .mcs into the cfgmem.
##   4. Boots the FPGA from flash (boot_hw_device).  No power-cycle needed.
##
## After it finishes, the device is configured from flash AND the new image
## persists across power cycles.  To reload from flash again later, just
## power-cycle the board or send a JTAG `boot_hw_device`.

if {[llength $argv] < 1} {
    puts stderr "ERR: usage: program_flash.tcl <fpga_top.bit> \[cfgmem-part\]"
    exit 1
}

set bit       [lindex $argv 0]
set part_glob [expr {[llength $argv] >= 2 ? [lindex $argv 1] : "mx25u51245g-spi-x1_x2_x4"}]

if {![file exists $bit]} {
    puts stderr "ERR: bitstream not found: $bit"
    exit 1
}

set bit_dir [file dirname  $bit]
set mcs     [file join $bit_dir [file rootname [file tail $bit]].mcs]
set prm     [file join $bit_dir [file rootname [file tail $bit]].prm]

puts "── stage 1: write_cfgmem ──────────────────────────────────────────"
puts "  bit  = $bit"
puts "  mcs  = $mcs"
puts "  part = $part_glob"

# write_cfgmem stitches the .bit into a flash image at offset 0 with
# CONFIG_MODE = SPIx4 (matches synth/ku5p.xdc).  -force overwrites any
# stale .mcs from a prior flash attempt.
write_cfgmem -force -format mcs \
             -interface SPIx4 -size 32 \
             -loadbit "up 0x0 $bit" \
             -file $mcs

if {![file exists $mcs]} {
    puts stderr "ERR: write_cfgmem did not produce $mcs"
    exit 1
}

puts ""
puts "── stage 2: connect to hw_server ──────────────────────────────────"
open_hw_manager
connect_hw_server -url localhost:3121 -allow_non_jtag
current_hw_target [lindex [get_hw_targets] 0]
set_property PARAM.FREQUENCY 6000000 [current_hw_target]
open_hw_target

set dev [lindex [get_hw_devices xcku5p_*] 0]
if {$dev eq ""} {
    puts stderr "ERR: no xcku5p device found on JTAG chain.  Got:"
    puts stderr "      [get_hw_devices]"
    exit 1
}
current_hw_device $dev
puts "  device   = [get_property PART $dev]"

set part [lindex [get_cfgmem_parts $part_glob] 0]
if {$part eq ""} {
    puts stderr "ERR: cfgmem part '$part_glob' not found.  Try:"
    puts stderr "     [get_cfgmem_parts mt25qu*]"
    puts stderr "     [get_cfgmem_parts s25fl*]"
    exit 1
}
puts "  cfgmem   = $part"

# Detach any prior cfgmem instance so create_hw_cfgmem succeeds idempotently.
foreach old [get_hw_cfgmems] {
    delete_hw_cfgmem $old
}
create_hw_cfgmem -hw_device $dev -mem_dev $part

set cfg [lindex [get_hw_cfgmems] 0]
set_property PROGRAM.BLANK_CHECK    0       $cfg
set_property PROGRAM.ERASE          1       $cfg
set_property PROGRAM.CFG_PROGRAM    1       $cfg
set_property PROGRAM.VERIFY         1       $cfg
set_property PROGRAM.CHECKSUM       0       $cfg
set_property PROGRAM.ADDRESS_RANGE  {use_file} $cfg
set_property PROGRAM.FILES          [list $mcs] $cfg
if {[file exists $prm]} {
    set_property PROGRAM.PRM_FILE   $prm    $cfg
}

puts ""
puts "── stage 3: erase + program + verify (this takes 1-3 min) ─────────"
set t0 [clock seconds]
program_hw_cfgmem -hw_cfgmem $cfg
set dt [expr {[clock seconds] - $t0}]
puts "  done in ${dt}s"

puts ""
puts "── stage 4: boot device from flash ────────────────────────────────"
boot_hw_device $dev

puts ""
puts "=== FLASH PROGRAMMING COMPLETE ==="
puts "Bitstream is now persistent in SPI flash.  Power-cycle to confirm."

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
