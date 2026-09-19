# Source in an open, implemented design BEFORE writing its blank bitstream.
# Export only the explicit, fixed-layout PIC memory; never infer arbitrary ROM
# packing from a netlist. The companion manifest binds this map to this image.
set adb_firmware_tools [file normalize [file dirname [info script]]/../tools]
proc validate_adb_firmware_bram {} {
    set cells [get_cells -hier -quiet -filter {NAME =~ "*u_firmware_bram" && REF_NAME == RAMB36E2}]
    if {[llength $cells] != 1} {
        error "Expected exactly one patchable ADB RAMB36E2; found [llength $cells]. Old LUTROM images cannot use this flow."
    }
    set cell [lindex $cells 0]
    foreach {property expected} {READ_WIDTH_A 36 DOA_REG 0 EN_ECC_READ FALSE EN_ECC_WRITE FALSE} {
        if {[get_property $property $cell] ne $expected} {
            error "ADB BRAM $property mismatch: expected $expected"
        }
    }
    if {![regexp {^(1|1'b1|TRUE)$} [get_property IS_CLKARDCLK_INVERTED $cell]]} {
        error "ADB BRAM must fetch on the falling edge"
    }
    for {set i 0} {$i < 128} {incr i} {
        set value [get_property [format "INIT_%02X" $i] $cell]
        if {![regexp {^(256'h)?0+$} $value]} { error "ADB BRAM is not blank; refusing to publish patch bundle" }
    }
    for {set i 0} {$i < 16} {incr i} {
        set value [get_property [format "INITP_%02X" $i] $cell]
        if {![regexp {^(256'h)?0+$} $value]} { error "ADB BRAM parity is not blank" }
    }
    set loc [get_property LOC $cell]
    if {![regexp {^RAMB36_(X[0-9]+Y[0-9]+)$} $loc -> placement]} {
        error "ADB BRAM has no valid placement: $loc"
    }
    return $cell
}
proc export_adb_firmware_bundle {bitfile} {
    global adb_firmware_tools
    set cell [validate_adb_firmware_bram]
    regexp {^RAMB36_(X[0-9]+Y[0-9]+)$} [get_property LOC $cell] -> placement
    set mmi [file rootname $bitfile].adb.mmi
    set out [open $mmi w]
    puts $out {<?xml version="1.0" encoding="UTF-8"?>}
    puts $out {<MemInfo Version="1" Minor="0">}
    puts $out {  <Processor Endianness="Little" InstPath="adb_pic">}
    puts $out {    <AddressSpace Name="program" Begin="0" End="4095"><BusBlock>}
    # MMI v1.0 calls the 32-data-bit RAMB36 layout RAMB32 (not RAMB36).
    puts $out "      <BitLane MemType=\"RAMB32\" Placement=\"$placement\">"
    puts $out {        <DataWidth MSB="31" LSB="0"/>}
    puts $out {        <AddressRange Begin="0" End="1023"/>}
    puts $out {        <Parity ON="false" NumBits="0"/>}
    puts $out {      </BitLane>}
    puts $out {    </BusBlock></AddressSpace>}
    puts $out {  </Processor>}
    puts $out "  <Config><Option Name=\"Part\" Val=\"[get_property PART [current_design]]\"/></Config>"
    puts $out {</MemInfo>}
    close $out
    set manifest [file rootname $bitfile].adb.json
    # Regenerating this companion is safe: it contains only public file hashes.
    file delete -force $manifest
    puts [exec python3 [file join $adb_firmware_tools patch_adb_bitstream.py] seal \
        --bit $bitfile --mmi $mmi --manifest $manifest]
}
