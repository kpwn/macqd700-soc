# synth/vio_bootfsm_probe.tcl — read boot_fsm's own SD-level debug state
# via VIO, bypassing hw_axi entirely. 2026-07-24 investigation: L2C's
# s_awvalid/mf_arvalid never fire on the stub_l2c_ila bitstream -- check
# whether boot_fsm's own SD init/read state machine is stuck BEFORE ever
# reaching the point of issuing a DDR write at all.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/vio_bootfsm_probe.tcl \
#       -tclargs <bit_file> <ltx_file>

set bit_file [lindex $argv 0]
set ltx_file [lindex $argv 1]

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets]
current_hw_target [lindex $targets 0]
open_hw_target

set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROGRAM.FILE $bit_file $dev
set_property PROBES.FILE  $ltx_file $dev
program_hw_devices $dev
puts "=== Programmed $bit_file at [clock format [clock seconds]] ==="
refresh_hw_device $dev -update_hw_probes true

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

# Give the board a moment to run its natural post-configuration boot
# sequence before sampling.
after 2000

foreach name {vio_boot_diag vio_boot_crc vio_rst_bundle s0_wready mig_cal_done dbg_committed} {
    puts "=== $name = [read_vio $vio $name] ==="
}
# Sample again after a further delay to see if anything changes over
# time (e.g. sector count advancing, or a stuck value).
after 3000
puts "--- second sample, 3s later ---"
foreach name {vio_boot_diag vio_boot_crc vio_rst_bundle s0_wready mig_cal_done dbg_committed} {
    puts "=== $name = [read_vio $vio $name] ==="
}

puts "=== DONE at [clock format [clock seconds]] ==="
