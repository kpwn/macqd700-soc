# Synthetic-only tool-chain test; NEVER program these test images on a board.
# Small standalone ROM implementation, not a full SoC build.
# vivado -mode batch -source synth/pic_bram_roundtrip.tcl -tclargs build/pic-roundtrip
# To reuse that test placement and check compressed output:
# ... -tclargs build/pic-roundtrip-compressed build/pic-roundtrip/rom.dcp TRUE
if {[catch {
set root [file normalize [file dirname [info script]]/..]
set output [file normalize [lindex $argv 0]]
file mkdir $output
set_param general.maxThreads 4
if {[lindex $argv 1] ne ""} {
    open_checkpoint [lindex $argv 1]
} else {
    create_project -in_memory -part xcku5p-ffvb676-2-i
    read_verilog $root/rtl/mac/pic16c5x.v
    synth_design -top pic16c5x_program_rom -part xcku5p-ffvb676-2-i
    create_clock -name clk -period 20.0 [get_ports clk]
    set_property IOSTANDARD LVCMOS18 [get_ports *]
    opt_design
    place_design
    route_design
}
# Test-only unconstrained IO assignments: no image from this script is a board
# design. Never weaken these DRCs in a production implementation flow.
set_property SEVERITY Warning [get_drc_checks UCIO-1]
set compression [lindex $argv 2]
if {$compression eq ""} { set compression FALSE }
set_property BITSTREAM.GENERAL.COMPRESS $compression [current_design]
write_checkpoint -force $output/rom.dcp
source $root/synth/adb_firmware_mmi.tcl
validate_adb_firmware_bram
write_bitstream -force $output/blank.bit
export_adb_firmware_bundle $output/blank.bit
set cell [get_cells -hier -filter {REF_NAME == RAMB36E2}]
set bytes {}
for {set i 0} {$i < 512} {incr i} {
    set word [expr {($i * 17) & 0xfff}]
    lappend bytes [expr {$word & 255}] [expr {$word >> 8}]
}
set dump [open $output/synthetic.bin wb]
puts -nonewline $dump [binary format c* $bytes]
close $dump
for {set init 0} {$init < 128} {incr init} {
    set value ""
    for {set lane 7} {$lane >= 0} {incr lane -1} {
        set index [expr {$init * 8 + $lane}]
        set word [expr {$index < 512 ? ($index * 17) & 0xfff : 0}]
        append value [format %08x $word]
    }
    set_property [format INIT_%02X $init] "256'h$value" $cell
}
write_bitstream -force $output/reference.bit
puts [exec python3 $root/tools/patch_adb_bitstream.py patch \
    --bit $output/blank.bit --mmi $output/blank.adb.mmi \
    --manifest $output/blank.adb.json --firmware $output/synthetic.bin \
    --output $output/patched.bit]
puts [exec python3 $root/tools/check_adb_roundtrip.py $output/reference.bit $output/patched.bit]
} failure]} {
    puts stderr $failure
    exit 1
}
