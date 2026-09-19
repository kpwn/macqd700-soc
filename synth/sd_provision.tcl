# sd_provision.tcl — Non-project Vivado flow for the DEDICATED SD-card
# provisioning bitstream (rtl/soc/sd_provision_top.v).
#
# Usage (from Makefile):
#   vivado -mode batch -source synth/sd_provision.tcl -tclargs synth_only <output_dir>
#   vivado -mode batch -source synth/sd_provision.tcl -tclargs full_impl  <output_dir>
#
# This is a deliberately tiny standalone design (JTAG-to-AXI master +
# sd_bulk_writer + boot_fsm card init + sd_spi — no CPU, no DDR, no
# video, no peripheral bus), so a full impl takes minutes, not the
# ~35-45 min of the fpga_top flow.  The resulting bitstream is loaded
# transiently over JTAG for `sd-write-fast` (tools/jtag_repl.tcl), then
# the board is re-programmed with the normal fpga_top bitstream; nothing
# persists on the board.
#
# The JTAG-to-AXI IP here (prov_jtag_axi) differs from fpga_top's
# debug_jtag_axi in ONE crucial config: CONFIG.M_HAS_BURST=1.  That lets
# one create_hw_axi_txn/run_hw_axi round-trip carry an INCR burst of up
# to 256 x 32-bit beats (1 KiB) instead of a single word — the entire
# speed story of the fast SD-write path.

set mode       [lindex $argv 0]
set output_dir [lindex $argv 1]

set_param general.maxThreads 16

if {$mode ne "synth_only" && $mode ne "full_impl"} {
    puts stderr "ERROR: mode must be one of {synth_only, full_impl}, got: $mode"
    exit 1
}

set proj_root [file normalize [file dirname [info script]]/..]
set rtl_dir   $proj_root/rtl
set synth_dir $proj_root/synth
set part      "xcku5p-ffvb676-2-i"

file mkdir $output_dir
file mkdir $output_dir/reports

# ──────────────────────────────────────────────────────────────────────
# JTAG-to-AXI master IP — burst-capable (M_HAS_BURST=1).
# ──────────────────────────────────────────────────────────────────────
proc gen_prov_jtag_axi_ip {output_dir part} {
    set ip_dir    $output_dir/ip
    set ip_xci    $ip_dir/prov_jtag_axi/prov_jtag_axi.xci
    set ip_dcp    $ip_dir/prov_jtag_axi/prov_jtag_axi.dcp
    set ip_marker $ip_dir/prov_jtag_axi/sd_provision_jtag_axi_v1.txt
    file mkdir $ip_dir

    set marker_ok 0
    if {[file exists $ip_marker]} {
        set fh [open $ip_marker r]
        set txt [read $fh]
        close $fh
        set marker_ok [expr {[string first "prov_jtag_axi=v1" $txt] >= 0 &&
                             [string first "part=$part" $txt] >= 0}]
    }
    if {[file exists $ip_dcp] && $marker_ok} {
        puts "=== Reusing cached prov_jtag_axi IP at $ip_dcp ==="
        return $ip_xci
    } elseif {[file exists [file dirname $ip_xci]]} {
        puts "=== Existing prov_jtag_axi IP is missing/stale/wrong-part; regenerating ==="
        file delete -force $ip_dir/prov_jtag_axi
    }

    puts "=== Generating prov_jtag_axi IP OOC into $ip_dir ==="
    create_project -in_memory -part $part -force
    create_ip -name jtag_axi -vendor xilinx.com -library ip -version 1.2 \
        -module_name prov_jtag_axi -dir $ip_dir
    set_property -dict [list \
        CONFIG.PROTOCOL {0} \
        CONFIG.M_HAS_BURST {1} \
        CONFIG.M_AXI_DATA_WIDTH {32} \
        CONFIG.M_AXI_ADDR_WIDTH {32} \
        CONFIG.M_AXI_ID_WIDTH {1} \
        CONFIG.RD_TXN_QUEUE_LENGTH {1} \
        CONFIG.WR_TXN_QUEUE_LENGTH {1} \
    ] [get_ips prov_jtag_axi]
    generate_target {synthesis} [get_ips prov_jtag_axi]
    synth_ip [get_ips prov_jtag_axi]
    set fh [open $ip_marker w]
    puts $fh "prov_jtag_axi=v1"
    puts $fh "part=$part"
    puts $fh "data_width=32"
    puts $fh "host_contract=burst_up_to_256_beats"
    close $fh
    close_project
    return $ip_xci
}

set jtag_xci [gen_prov_jtag_axi_ip $output_dir $part]

# ──────────────────────────────────────────────────────────────────────
# Read RTL + constraints
# ──────────────────────────────────────────────────────────────────────
create_project -in_memory -part $part -force

read_ip $jtag_xci
puts "=== prov_jtag_axi IP read: $jtag_xci ==="

read_verilog [list \
    $rtl_dir/board/sd_spi.v \
    $rtl_dir/board/sd_spi_mux.v \
    $rtl_dir/board/sd_ctrl.v \
    $rtl_dir/board/sd_bulk_writer.v \
    $rtl_dir/board/clk_rst.v \
    $rtl_dir/board/reset_debounce.v \
    $rtl_dir/soc/boot_fsm.v \
    $rtl_dir/soc/sd_provision_core.v \
    $rtl_dir/soc/sd_provision_top.v \
]

read_xdc $synth_dir/sd_provision.xdc

# ──────────────────────────────────────────────────────────────────────
# Synthesis
# ──────────────────────────────────────────────────────────────────────
puts "=== SYNTHESIS (sd_provision_top) ==="
synth_design -top sd_provision_top -part $part \
    -verilog_define PROV_JTAG_AXI \
    -flatten_hierarchy rebuilt

write_checkpoint -force $output_dir/post_synth.dcp
report_utilization -file $output_dir/reports/utilization_synth.rpt

if {$mode eq "synth_only"} {
    report_timing_summary -file $output_dir/reports/timing_synth.rpt
    puts "=== synth_only done — checkpoint at $output_dir/post_synth.dcp ==="
    exit 0
}

# ──────────────────────────────────────────────────────────────────────
# Implementation + bitstream
# ──────────────────────────────────────────────────────────────────────
puts "=== OPT / PLACE / ROUTE ==="
opt_design
place_design
phys_opt_design
route_design

write_checkpoint -force $output_dir/post_route.dcp
report_utilization    -file $output_dir/reports/utilization_route.rpt
report_timing_summary -file $output_dir/reports/timing_route.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "=== Post-route WNS: $wns ns ==="
if {$wns < 0} {
    puts "WARNING: negative slack — inspect $output_dir/reports/timing_route.rpt"
}

puts "=== WRITING BITSTREAM ==="
write_bitstream -force $output_dir/sd_provision_top.bit

puts "=== DONE: $output_dir/sd_provision_top.bit ==="
puts "Load it with tools/jtag_repl.tcl (load-bit) or the usual program"
puts "flow, run sd-write-fast, then re-program the normal fpga_top"
puts "bitstream.  Nothing persists on the board."
exit 0
