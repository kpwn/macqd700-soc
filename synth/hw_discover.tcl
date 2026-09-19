# hw_discover.tcl -- list Vivado hw_manager targets/devices without programming.
#
# Usage:
#   vivado -nojournal -nolog -mode batch -source synth/hw_discover.tcl
#
# Optional environment override:
#   HW_TARGET_RE  Tcl regexp matched against get_hw_targets before opening.

proc prop_or_unknown {obj prop} {
    set value "<unknown>"
    if {![catch {set value [get_property $prop $obj]}]} {
        if {$value eq ""} { set value "<empty>" }
    }
    return $value
}

proc target_matches {target} {
    if {![info exists ::env(HW_TARGET_RE)] || $::env(HW_TARGET_RE) eq ""} {
        return 1
    }
    return [regexp -- $::env(HW_TARGET_RE) $target]
}

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
puts "Targets discovered: [llength $targets]"
if {[llength $targets] == 0} {
    puts stderr "ERROR: no JTAG targets discovered. Check board power, USB/JTAG cabling, and permissions."
    exit 2
}

foreach target $targets {
    puts "Target: $target"
    if {![target_matches $target]} {
        puts "  skipped by HW_TARGET_RE"
        continue
    }

    current_hw_target $target
    if {[catch {open_hw_target} err]} {
        puts stderr "  ERROR: could not open target: $err"
        continue
    }

    set devices [get_hw_devices]
    puts "  Devices: [llength $devices]"
    foreach dev $devices {
        puts "    $dev"
        puts "      PART:   [prop_or_unknown $dev PART]"
        puts "      IDCODE: [prop_or_unknown $dev IDCODE_REGISTER]"
        puts "      DNA:    [prop_or_unknown $dev REGISTER.EFUSE.FUSE_DNA]"
    }
}

puts "=== DISCOVERY COMPLETE ==="
