# vivado_sdmin.tcl — minimal standalone synth+impl+bitstream flow for the
# fpga_top_sdmin SD/CRC test harness (rtl/soc/fpga_top_sdmin.v).
#
# Purpose: the full fpga_top build (synth/vivado.tcl, DDR4 MIG + HDMI + the
# whole CPU) takes 35-45 minutes per cycle — far too slow for iterating on
# the CMD18/CRC16 boot-blocker investigation (2026-07-23). This is a
# deliberately small, separate, self-contained flow (no CDC constraint
# passes, no incremental-place reuse, no PCIe/HDMI/DDR branches) for a
# design with no DDR4 MIG and no CPU to place/route — should complete in a
# few minutes.
#
# Usage:
#   vivado -mode batch -source synth/vivado_sdmin.tcl -tclargs <output_dir>

set output_dir [lindex $argv 0]
set proj_root  [file normalize [file dirname [info script]]/..]
set rtl_dir    $proj_root/rtl
set synth_dir  $proj_root/synth
set part       "xcku5p-ffvb676-2-i"

set_param general.maxThreads 16
file mkdir $output_dir
file mkdir $output_dir/ip
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints

# ──────────────────────────────────────────────────────────────────────
# Generate the sdmin_vio IP (throwaway in-memory project — same pattern
# as synth/vivado.tcl's gen_debug_vio_ip, just smaller/renamed to avoid
# any collision with the main SoC's debug_vio IP).
# ──────────────────────────────────────────────────────────────────────
set ip_dir $output_dir/ip
set ip_xci $ip_dir/sdmin_vio/sdmin_vio.xci

# Always regenerate from scratch — this script has no cache-invalidation
# marker (unlike synth/vivado.tcl's gen_debug_vio_ip), so a stale IP dir
# left over from a prior run makes Vivado auto-version the new one
# (sdmin_vio_1, sdmin_vio_2, ...) instead of overwriting in place, which
# silently breaks the hardcoded $ip_xci path above (2026-07-24: cost a
# whole rebuild+flash+test cycle before this was caught — probe reads
# quietly landed on a stale 4-probe IP from an earlier iteration).
file delete -force $ip_dir/sdmin_vio
foreach d [glob -nocomplain $ip_dir/sdmin_vio_*] { file delete -force $d }

puts "=== Generating sdmin_vio IP ==="
create_project -in_memory -part $part -force
create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name sdmin_vio -dir $ip_dir
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN     {13} \
    CONFIG.C_NUM_PROBE_OUT    {1} \
    CONFIG.C_PROBE_IN0_WIDTH  {32} \
    CONFIG.C_PROBE_IN1_WIDTH  {32} \
    CONFIG.C_PROBE_IN2_WIDTH  {32} \
    CONFIG.C_PROBE_IN3_WIDTH  {32} \
    CONFIG.C_PROBE_IN4_WIDTH  {32} \
    CONFIG.C_PROBE_IN5_WIDTH  {32} \
    CONFIG.C_PROBE_IN6_WIDTH  {32} \
    CONFIG.C_PROBE_IN7_WIDTH  {32} \
    CONFIG.C_PROBE_IN8_WIDTH  {32} \
    CONFIG.C_PROBE_IN9_WIDTH  {32} \
    CONFIG.C_PROBE_IN10_WIDTH {32} \
    CONFIG.C_PROBE_IN11_WIDTH {32} \
    CONFIG.C_PROBE_IN12_WIDTH {32} \
    CONFIG.C_PROBE_OUT0_WIDTH {3} \
    CONFIG.C_PROBE_OUT0_INIT_VAL {0x0} \
    CONFIG.C_EN_PROBE_IN_ACTIVITY {1} \
] [get_ips sdmin_vio]
generate_target {synthesis} [get_ips sdmin_vio]
synth_ip [get_ips sdmin_vio]
close_project

# ──────────────────────────────────────────────────────────────────────
# Non-project synth of the harness RTL + IP netlist. Only the files this
# minimal design actually needs — no sd_spi_mux (single SPI consumer),
# no CPU, no DDR4/HDMI.
# ──────────────────────────────────────────────────────────────────────
puts "=== Reading RTL ==="
read_verilog $rtl_dir/soc/fpga_top_sdmin.v
read_verilog $rtl_dir/soc/boot_fsm.v
read_verilog $rtl_dir/board/sd_ctrl.v
read_verilog $rtl_dir/board/sd_spi.v
read_ip $ip_xci
read_xdc $synth_dir/fpga_top_sdmin.xdc

puts "=== synth_design ==="
synth_design -top fpga_top_sdmin -part $part \
    -include_dirs [list $rtl_dir $rtl_dir/soc $rtl_dir/board]

write_checkpoint -force $output_dir/checkpoints/synth.dcp

puts "=== opt/place/phys_opt/route ==="
opt_design
place_design
phys_opt_design
route_design

report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
write_checkpoint -force $output_dir/checkpoints/route.dcp

write_bitstream -force $output_dir/fpga_top_sdmin.bit
write_debug_probes -force $output_dir/fpga_top_sdmin.ltx

set buildinfo_fh [open $output_dir/fpga_top_sdmin.buildinfo w]
puts $buildinfo_fh "part=$part"
puts $buildinfo_fh "top=fpga_top_sdmin"
puts $buildinfo_fh "bitstream=fpga_top_sdmin.bit"
puts $buildinfo_fh "debug_probes=fpga_top_sdmin.ltx"
puts $buildinfo_fh "generated_utc=[clock format [clock seconds] -gmt 1 -format {%Y-%m-%dT%H:%M:%SZ}]"
close $buildinfo_fh

puts "=== SDMIN BUILD COMPLETE ==="
puts "Bitstream: $output_dir/fpga_top_sdmin.bit"
puts "Probes:    $output_dir/fpga_top_sdmin.ltx"
