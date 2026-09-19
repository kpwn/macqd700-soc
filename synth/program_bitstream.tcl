# program_bitstream.tcl -- program one FPGA bitstream without requiring probes.
#
# Usage:
#   vivado -nojournal -nolog -mode batch \
#          -source synth/program_bitstream.tcl \
#          -tclargs <bitstream.bit> [probes.ltx]
#
# Optional environment overrides:
#   HW_TARGET_RE  Tcl regexp matched against get_hw_targets.
#   HW_DEVICE_RE  Tcl regexp matched against get_hw_devices.

if {[llength $argv] < 1 || [llength $argv] > 2} {
    puts stderr "ERROR: usage: program_bitstream.tcl <bitstream.bit> [probes.ltx]"
    exit 2
}

set bit_file [file normalize [lindex $argv 0]]
set ltx_file ""
if {[llength $argv] == 2} {
    set ltx_file [file normalize [lindex $argv 1]]
}

if {![file exists $bit_file]} {
    puts stderr "ERROR: bitstream not found: $bit_file"
    exit 1
}
if {$ltx_file ne "" && ![file exists $ltx_file]} {
    puts stderr "ERROR: probe file not found: $ltx_file"
    exit 1
}

proc first_matching {items env_name label} {
    if {[info exists ::env($env_name)] && $::env($env_name) ne ""} {
        set re $::env($env_name)
        foreach item $items {
            if {[regexp -- $re $item]} {
                return $item
            }
        }
        puts stderr "ERROR: no $label matched $env_name='$re'"
        puts stderr "Available $label values:"
        foreach item $items { puts stderr "  $item" }
        exit 2
    }
    return [lindex $items 0]
}

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
if {[llength $targets] == 0} {
    puts stderr "ERROR: no JTAG targets discovered. Is the KU5P powered and cabled?"
    exit 2
}
puts "=== Hardware targets ==="
foreach t $targets { puts "  $t" }

set target [first_matching $targets HW_TARGET_RE "hardware target"]
puts "=== Opening target: $target ==="
current_hw_target $target
open_hw_target

set devices [get_hw_devices]
if {[llength $devices] == 0} {
    puts stderr "ERROR: target opened but no hardware devices were discovered."
    exit 2
}
puts "=== Hardware devices ==="
foreach d $devices { puts "  $d" }

set dev [first_matching $devices HW_DEVICE_RE "hardware device"]
puts "=== Programming device: $dev ==="
current_hw_device $dev
set_property PROGRAM.FILE $bit_file $dev
if {$ltx_file ne ""} {
    set_property PROBES.FILE $ltx_file $dev
    set_property FULL_PROBES.FILE $ltx_file $dev
    puts "=== Probe file attached: $ltx_file ==="
} else {
    puts "=== No probe file supplied; programming bitstream only ==="
}
program_hw_devices $dev
refresh_hw_device $dev

puts "=== PROGRAM COMPLETE ==="
puts "Bitstream: $bit_file"
