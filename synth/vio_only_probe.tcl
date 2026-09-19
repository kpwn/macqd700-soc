# synth/vio_only_probe.tcl — connect and read ONLY VIO probes, skipping
# any hw_axi/JTAG-AXI transaction entirely. Diagnostic for 2026-07-24
# L2C-hang investigation: the main jtag_repl.tcl REPL's startup does an
# unconditional AXI-Lite build_id read before ever reaching its command
# loop, and that read appears to hang indefinitely on this build
# (suspected: boot_fsm's stuck write into l2c wedges shared fabric that
# debug_ctrl's own AXI-Lite path also depends on). This script never
# touches hw_axi at all, to see whether VIO readback still works while
# the main AXI fabric is wedged.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/vio_only_probe.tcl \
#       -tclargs <bit_file> <ltx_file>

set bit_file [lindex $argv 0]
set ltx_file [lindex $argv 1]

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
if {[llength $targets] == 0} {
    puts stderr "ERROR: no JTAG targets discovered."
    exit 2
}
current_hw_target [lindex $targets 0]
open_hw_target

set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROGRAM.FILE $bit_file $dev
set_property PROBES.FILE  $ltx_file $dev
program_hw_devices $dev
puts "=== Programmed $bit_file at [clock format [clock seconds]] ==="
refresh_hw_device $dev -update_hw_probes true
puts "=== refresh_hw_device returned at [clock format [clock seconds]] ==="

set vios [get_hw_vios -of_objects $dev -quiet]
if {[llength $vios] == 0} {
    puts stderr "ERROR: no hw_vio core found on device."
    exit 3
}
set vio [lindex $vios 0]
puts "=== VIO core: $vio ==="

puts "=== All probe names ==="
foreach p [get_hw_probes -of_objects $vio -quiet] {
    puts "  [get_property NAME $p]"
}

proc read_vio {vio name} {
    refresh_hw_vio $vio
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq $name} {
            return [get_property INPUT_VALUE $p]
        }
    }
    return "NOT_FOUND"
}

foreach name {vio_rst_bundle s0_wready dbg_committed dbg_pc hdmi_mmcm_locked} {
    puts "=== $name = [read_vio $vio $name] ==="
}

puts "=== DONE at [clock format [clock seconds]] ==="
