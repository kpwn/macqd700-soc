# synth/ila_l2c_forced_reset_capture.tcl -- L2C narrow-write SLVERR capture,
# take 2 (2026-07-24). The prior two attempts (triggering on mf_arvalid,
# then on s_awvalid) both timed out with zero data because boot_fsm's
# fatal first write happens within a few hundred cycles of reset release
# -- well before Vivado finishes program_hw_devices + refresh_hw_device +
# arming the ILA (~9s). Root cause has since been found and fixed
# (l2c_ctrl.v's front door SLVERR'd any AXI transfer narrower than 16B;
# fixed commit d4c6d80) -- this capture is confirmatory: get an actual
# waveform of the SLVERR sequence on the STILL-FLASHED pre-fix bitstream
# before it gets replaced by the fixed rebuild.
#
# Key difference from the prior attempts: arm the ILA FIRST, THEN force a
# fresh reset via the VIO-controlled debug_full_reset bit
# (vio_boot_ctrl[3], probe_out0) -- so the fault happens on OUR schedule,
# safely after the trigger is already armed, instead of racing the
# natural post-configuration boot sequence.
#
# Also reads the sticky first-SLVERR address latches (err_s0_aw_addr_r /
# err_s0_ar_addr_r, VIO-readable, no ILA needed) as a fast independent
# corroboration -- these should already show 0x00000000 (boot_fsm's
# first RAM zero-fill write) from the CURRENT (pre-reset) run.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_l2c_forced_reset_capture.tcl \
#       -tclargs <bit_file> <ltx_file> [timeout_sec]

set bit_file    [lindex $argv 0]
set ltx_file    [lindex $argv 1]
set timeout_sec [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 30}]

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

set vios [get_hw_vios -of_objects $dev -quiet]
if {[llength $vios] == 0} {
    puts stderr "ERROR: no hw_vio core found."
    exit 3
}
set vio [lindex $vios 0]

proc read_vio {vio name} {
    refresh_hw_vio $vio
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq $name} {
            return [get_property INPUT_VALUE $p]
        }
    }
    return "NOT_FOUND"
}

# --- Fast independent corroboration: sticky first-SLVERR address latch,
# no ILA needed, reflects whatever happened since the last soc_full_rst
# (i.e. since this bitstream was originally programmed, pre-dating this
# script's own forced reset below). ---
puts "=== PRE-RESET sticky first-error addresses (from the ORIGINAL boot attempt) ==="
puts "=== err_s0_aw_addr_r = [read_vio $vio err_s0_aw_addr_r] ==="
puts "=== err_s0_ar_addr_r = [read_vio $vio err_s0_ar_addr_r] ==="
puts "=== vio_boot_diag    = [read_vio $vio vio_boot_diag] ==="

set ilas [get_hw_ilas -of_objects $dev -quiet]
if {[llength $ilas] == 0} {
    puts stderr "ERROR: no hw_ila core found."
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p41 [get_hw_probes dbg_l2c_write_snap -of_objects $ila -quiet]
set p42 [get_hw_probes dbg_l2c_master_snap -of_objects $ila -quiet]
set p43 [get_hw_probes mig_cal_done -of_objects $ila -quiet]
if {[llength $p41] == 0 || [llength $p42] == 0 || [llength $p43] == 0} {
    puts stderr "ERROR: expected probes not found. Available:"
    foreach p [get_hw_probes -of_objects $ila -quiet] { puts stderr "  $p" }
    exit 3
}

# Trigger on s_awvalid (dbg_l2c_write_snap bit[10]) -- the very first
# thing that happens when boot_fsm's write reaches L2C's front door.
# Modest pre-trigger (we want post-trigger visibility into st[1:0]
# cycling IDLE->WAIT->LOOKUP->HITRESP->IDLE, the tell-tale fast-reject
# signature) but not zero, in case the AW and the illegal_c resolution
# race within a couple cycles of each other.
set_property CONTROL.TRIGGER_POSITION 200 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq11'b1XXXXXXXXXX $p41
puts "=== Trigger: dbg_l2c_write_snap bit\[10\] (s_awvalid) == 1 ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

# Now force the fault to happen on OUR schedule: pulse the VIO debug
# full-reset bit (probe_out0 / vio_boot_ctrl[3]). Rising-edge detected +
# pulse-stretched in fpga_top_clocks.vh, so a brief assert-then-release
# is sufficient -- no need to hold it.
proc get_boot_ctrl_probe {vio} {
    foreach name {probe_out0 vio_boot_ctrl vio_boot_ctrl_1} {
        set p [get_hw_probes -quiet -of_objects $vio $name]
        if {[llength $p] != 0} { return [lindex $p 0] }
    }
    return ""
}
set po [get_boot_ctrl_probe $vio]
if {$po eq ""} {
    puts stderr "ERROR: VIO boot control output probe not found."
    exit 3
}
puts "=== Pulsing VIO debug-full-reset (probe_out0=0x8) to force a fresh boot_fsm run ==="
set_property OUTPUT_VALUE 8 $po
commit_hw_vio $vio
after 100
set_property OUTPUT_VALUE 0 $po
commit_hw_vio $vio
puts "=== Reset pulse issued at [clock format [clock seconds]] ==="

puts "=== Waiting up to ${timeout_sec}s for trigger ==="
wait_on_hw_ila -timeout [expr {$timeout_sec / 60.0}] $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
set nsamples 0
catch { set nsamples [get_property DATA.SAMPLES.WINDOW_SIZE $data] }
puts "=== data object: $data (window samples reported: $nsamples) ==="

catch {write_hw_ila_data -csv_file /tmp/ila_l2c_forced_reset_capture.csv -force $data}
puts "=== Wrote /tmp/ila_l2c_forced_reset_capture.csv ==="

# Post-capture: re-read the sticky error latches (should now be
# populated if they weren't already, since our forced reset just
# reproduced the fault).
puts "=== POST-CAPTURE sticky first-error addresses ==="
puts "=== err_s0_aw_addr_r = [read_vio $vio err_s0_aw_addr_r] ==="
puts "=== err_s0_ar_addr_r = [read_vio $vio err_s0_ar_addr_r] ==="
puts "=== vio_boot_diag    = [read_vio $vio vio_boot_diag] ==="

puts "=== DONE ==="
