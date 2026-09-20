# Export a Python-patcher base from an existing, routed checkpoint. No synth,
# placement or routing. Usage: vivado -mode batch -source this.tcl -tclargs
#     path/to/route.dcp output-directory
if {[catch {
    set root [file normalize [file dirname [info script]]/..]
    set output [file normalize [lindex $argv 1]]
    if {[llength $argv] != 2 || [file exists $output]} {
        error "expected a routed checkpoint and a NEW output directory"
    }
    file mkdir $output
    open_checkpoint [lindex $argv 0]
    source $root/synth/adb_firmware_mmi.tcl
    validate_adb_firmware_bram
    # A fixed uncompressed packet layout lets a release-specific map identify
    # individual INIT bits without implementing device-wide decompression.
    set_property BITSTREAM.GENERAL.COMPRESS FALSE [current_design]
    write_bitstream $output/fpga_top.python.bit
    export_adb_firmware_bundle $output/fpga_top.python.bit
    puts "ADB_PYTHON_BASE_EXPORTED"
} failure]} {
    puts stderr $failure
    exit 1
}
