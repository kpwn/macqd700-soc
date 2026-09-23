# vivado.tcl — Non-project Vivado flow for m68k-ooo
#
# Usage (from Makefile):
#   vivado -mode batch -source vivado.tcl -tclargs synth_only <output_dir>
#   vivado -mode batch -source vivado.tcl -tclargs full_impl  <output_dir>
#   vivado -mode batch -source vivado.tcl -tclargs dry_run    <output_dir>
#   vivado -mode batch -source vivado.tcl -tclargs clock_report <output_dir>
#   vivado -mode batch -source vivado.tcl -tclargs ddr_pincheck <output_dir>
#
# Steps: read_verilog → synth_design → insert JTAG VIO debug core →
#        opt_design → place_design → phys_opt_design → route_design →
#        report_timing_summary → write_bitstream + write_debug_probes
#
# `dry_run` mode reads all RTL + XDC files, performs `synth_design -rtl`
# to parse the design tree, then exits before full synthesis.  Used by
# agents to lint-check TCL + RTL set without burning a ~30-min synth
# run (the Vivado mutex would also block).  Output goes to <output_dir>
# but no checkpoint is written.
#
# `ddr_pincheck` mode is the explicit smoke path for the DDR4 real-hardware
# shell: it forces the non-SIM_MODEL port set, reads ddr4.xdc, runs RTL
# elaboration only, and exits before synthesis or bitstream generation.
# Full real-MIG synth/impl is handled by USE_REAL_MIG=1 and stitches the
# repo-generated DDR4 MIG OOC DCP into ddr_ctrl's black-box instance.
#
# Top module: `fpga_top` (real-hardware SoC).  `mac_top` remains the
# Verilator testbench target — this flow does NOT compile it.
#
# JTAG VIO debug probes (IP-generation flow):
#   When ENABLE_VIO=1 env var is set, a pre-synth step runs `create_ip
#   -name vio` to generate an out-of-context `debug_vio` IP checkpoint.
#   The generated `debug_vio` module is then instantiated directly in
#   `rtl/fpga_top.v` inside an `ifdef VIO_ENABLE` block (see
#   `-verilog_define VIO_ENABLE` passed to synth_design below) and the
#   core_clk-domain probes are wired to it as first-class module ports.
#   The resulting `fpga_top.ltx` probe file pairs with the bitstream
#   for the Vivado hw_manager JTAG dashboard — see
#   `synth/vio_dashboard.tcl` for the convenience open-hw script.
#
#   Why the IP flow (and not `create_debug_core -type vio`)?
#   Vivado 2023.1's `create_debug_core` silently coerces the `vio` type
#   to `ila`, so the probe ports mismatch at connect_debug_port time
#   and synth fails.  The `create_ip` flow generates the IP correctly
#   and is the pattern we use elsewhere (cf. sd-hdmi-bringup/synth/
#   vivado.tcl).  The reference project instantiates `debug_vio` as a
#   normal module — we follow the same convention here.

set mode       [lindex $argv 0]
set output_dir [lindex $argv 1]

set_param general.maxThreads 4

if {$mode ne "synth_only" && $mode ne "full_impl" && $mode ne "place_only" && $mode ne "dry_run" && $mode ne "clock_report" && $mode ne "ddr_pincheck"} {
    puts stderr "ERROR: mode must be one of {synth_only, full_impl, dry_run, clock_report, ddr_pincheck}, got: $mode"
    exit 1
}

set proj_root  [file normalize [file dirname [info script]]/..]
set rtl_dir    $proj_root/rtl
set synth_dir  $proj_root/synth

# ADB PIC BRAM is deliberately blank during synthesis. Users insert their own
# firmware after implementation; see docs/adb_firmware_bitstream.md.

# One identity for both the SoC top-level manifest and cpu040's generated
# debug controller. M68kSocketTop has no Verilog BUILD_ID parameter, so its
# Scala elaboration must see this value before read_all_rtl consumes the file.
if {[catch {exec git -C $proj_root rev-parse --short=8 HEAD 2>/dev/null} git_sha]} {
    set git_sha ""
}
if {[string length $git_sha] == 8 && [string is xdigit $git_sha]} {
    set build_id "0x${git_sha}"
} else {
    set build_id [format "0x%08X" [expr {[clock seconds] & 0xFFFFFFFF}]]
}

# ──────────────────────────────────────────────────────────────────────────────
# CPU socket select — mirrors the Makefile's CPU=stub|m68k|m68k040 flow
# (Makefile:1786-1925, "CPU build select").
# CPU=stub (default): the socket binds rtl/soc/cpu_stub.v (idle occupant).
# CPU=m68k: the socket binds cpu/rtl/core/m68k_axi_wrapper.v (the real
# m68k_core + if_to_axi + axi_narrow_to_wide + debug_ctrl glue) selected at
# elaboration via -verilog_define CPU_M68K (see rtl/soc/fpga_top_debug_ctrl.vh).
# CPU=m68k040: the socket binds cpu040/generated/M68kSocketTop.v (the
# m68k-core-040-ooo SpinalHDL "v2" CPU) selected at elaboration via
# -verilog_define CPU_M68K040 (see rtl/soc/fpga_top_debug_ctrl.vh).  Unlike
# CPU=m68k this is ONE generated file, not a source-tree glob, so it is
# regenerated fresh here (unconditionally — correctness over incremental
# speed for a real synth/impl run) via the exact command the Makefile's
# cpu040-gen rule uses: `cd cpu040 && sbt "runMain
# m68k040.top.GenSocketTopVerilog"`.  axi_i is native 256b for this CPU
# with no CPU-side downconverter, so CPU=m68k040 unconditionally forces
# L2C_ENABLE + VRAM_IN_DDR on below (task #269's dedicated 256b l2c fetch
# port is the only axi_i binding this CPU can use safely — see the `error`
# backstop at rtl/soc/fpga_top_cpu.vh's CPU_AXI_I_DW guard, which still
# catches anyone who bypasses this script entirely).
# ──────────────────────────────────────────────────────────────────────────────
# DEFAULT = m68k040 (was "stub").  A stub build is a bitstream with NO CPU and
# the symptom is silent -- the board simply never executes.  Stub stays reachable
# with an explicit CPU=stub for standalone SoC builds.
set cpu_sel [expr {[info exists ::env(CPU)] ? $::env(CPU) : "m68k040"}]
if {$cpu_sel ne "stub" && $cpu_sel ne "m68k040"} {
    puts stderr "ERROR: CPU must be one of {stub, m68k040}; got: $cpu_sel"
    exit 1
}
set cpu_m68k040 [expr {$cpu_sel eq "m68k040"}]
set cpu_ipc_profile [expr {[info exists ::env(CPU_IPC_PROFILE)] ? $::env(CPU_IPC_PROFILE) : "baseline"}]
if {$cpu_ipc_profile ni {baseline throughput-v1 throughput-v2}} { error "Invalid CPU_IPC_PROFILE=$cpu_ipc_profile" }
if {!$cpu_m68k040 && $cpu_ipc_profile ne "baseline"} { error "CPU_IPC_PROFILE requires CPU=m68k040" }
set perf_detail_enable [expr {[info exists ::env(PERF_DETAIL_ENABLE)] ? $::env(PERF_DETAIL_ENABLE) : 0}]
set cpu_debug_profile [expr {[info exists ::env(CPU_DEBUG_PROFILE)] ? $::env(CPU_DEBUG_PROFILE) : "full"}]
if {$cpu_debug_profile ni {full reduced}} { error "Invalid CPU_DEBUG_PROFILE=$cpu_debug_profile" }
if {!$cpu_m68k040 && $cpu_debug_profile ne "full"} { error "CPU_DEBUG_PROFILE requires CPU=m68k040" }
puts "=== CPU DEBUG PROFILE: $cpu_debug_profile ==="
set enable_ipc_ila [expr {[info exists ::env(ENABLE_IPC_ILA)] ? $::env(ENABLE_IPC_ILA) : 0}]
if {$perf_detail_enable ni {0 1} || $enable_ipc_ila ni {0 1}} {
    error "PERF_DETAIL_ENABLE and ENABLE_IPC_ILA must be 0 or 1"
}
if {$enable_ipc_ila && (!$perf_detail_enable || !$cpu_m68k040)} {
    error "ENABLE_IPC_ILA requires CPU=m68k040 and PERF_DETAIL_ENABLE=1"
}
puts "=== CPU SOCKET: CPU=$cpu_sel ==="

set cpu040_dir      $proj_root/cpu040
set cpu_m68k040_v   $cpu040_dir/generated/M68kSocketTop.v
if {$cpu_m68k040} {
    if {![file isdirectory $cpu040_dir]} {
        puts stderr "ERROR: CPU=m68k040 but $cpu040_dir not found."
        puts stderr "       Is the cpu040 submodule initialised?  Run: git submodule update --init"
        exit 1
    }
    puts "=== CPU=m68k040: regenerating $cpu_m68k040_v ==="
    puts "=== CPU=m68k040: cd $cpu040_dir && sbt \"runMain m68k040.top.GenSocketTopVerilog\" ==="
    if {[catch {exec env DBG_BUILD_ID=$build_id PERF_DETAIL_ENABLE=$perf_detail_enable CPU_IPC_PROFILE=$cpu_ipc_profile CPU_DEBUG_PROFILE=$cpu_debug_profile sh -c "cd $cpu040_dir && sbt \"runMain m68k040.top.GenSocketTopVerilog\"" 2>@1} sbt_out]} {
        puts stderr "ERROR: cpu040 M68kSocketTop.v regeneration failed:"
        puts stderr $sbt_out
        exit 1
    }
    puts $sbt_out
    if {![file exists $cpu_m68k040_v]} {
        puts stderr "ERROR: cpu040 regeneration reported success but $cpu_m68k040_v is still missing."
        exit 1
    }
    puts "=== CPU=m68k040: regeneration OK, using $cpu_m68k040_v ==="
}

# Shared include-dir list for every synth_design invocation below.  The
# pre-split $rtl_dir/core paths are gone; CPU=m68k appends the submodule's
# include roots (mirrors CPU_M68K_IDIRS in the Makefile).  CPU=m68k040
# needs no extra include dirs — M68kSocketTop.v is one flat, self-contained
# generated file with no `include`s (mirrors the Makefile's empty
# CPU_M68K040_IDIRS).
set rtl_include_dirs [list $rtl_dir $rtl_dir/soc $rtl_dir/board $rtl_dir/board/video_phy]

# ──────────────────────────────────────────────────────────────────────────────
# Part / board constants
# ──────────────────────────────────────────────────────────────────────────────
set part "xcku5p-ffvb676-2-i"

proc ensure_project_part {part} {
    if {[catch {current_project} project_obj]} {
        puts "INFO: no active Vivado project while setting PART=$part; explicit command-line -part arguments remain in use."
        return
    }

    set current_part ""
    catch {set current_part [get_property PART $project_obj]}
    if {$current_part ne $part} {
        puts "=== Setting active Vivado project part to $part (was '$current_part') ==="
        set_property PART $part $project_obj
    }
}

proc parse_int_env {name default_value} {
    if {[info exists ::env($name)]} {
        set raw [set ::env($name)]
    } else {
        set raw $default_value
    }
    set cleaned [string map {_ ""} $raw]
    if {![string is integer -strict $cleaned]} {
        puts stderr "ERROR: $name must be an integer; got '$raw'."
        exit 1
    }
    return [expr {int($cleaned)}]
}

proc parse_bool_env {name default_value} {
    set value [parse_int_env $name $default_value]
    if {$value != 0 && $value != 1} {
        puts stderr "ERROR: $name must be 0 or 1; got '$value'."
        exit 1
    }
    return $value
}

# ──────────────────────────────────────────────────────────────────────────────
# VIO gate.  ENABLE_VIO=1 turns on the JTAG VIO dashboard IP.  The
# gate has two effects:
#   1. Pre-synth we generate the `debug_vio` IP out-of-context and
#      read its XCI into the design.
#   2. `VIO_ENABLE` is passed to synth_design as a Verilog define so
#      the `ifdef VIO_ENABLE` block at the bottom of fpga_top.v
#      instantiates the `debug_vio` module.
# When disabled (default) the IP is skipped entirely and fpga_top.v
# falls through the `ifdef` without any debug_vio instance — no IP
# is required on the Vivado side and the bitstream is leaner.
# ──────────────────────────────────────────────────────────────────────────────
set enable_vio [parse_bool_env ENABLE_VIO 1]
set enable_jtag_axi [parse_bool_env ENABLE_JTAG_AXI 1]
set enable_pcie_xdma [parse_bool_env ENABLE_PCIE_XDMA 0]
# ILA gate.  ENABLE_ILA=1 turns on the JTAG ILA capture core.  Same IP
# flow as ENABLE_VIO: pre-synth generates `debug_ila` OOC, fpga_top.v
# instantiates it under `ifdef ILA_ENABLE`.  Default OFF — the ILA
# consumes ~28-32 BRAMs and a few thousand LUTs, only enable while
# actively bisecting the A7-drift / boot-wedge HW-only race.
# See synth/debug_ila.tcl + docs/ila_a7_drift_probes.md.
set enable_ila [parse_bool_env ENABLE_ILA 0]
# L2C URAM floorplan gate.  L2C_URAM_FLOORPLAN=0 drops the slice-major LOC on
# the 64 L2 data-array URAMs (applied after synth_design, see the block right
# before opt_design) so a build can be A/B'd against an unconstrained one.
# Inert when the L2C itself is off.  Recorded in fpga_top.buildinfo.
set l2c_uram_floorplan [parse_bool_env L2C_URAM_FLOORPLAN 1]
# Ethernet ON by default in its SONIC DMA form -- the configuration the board
# runs.  ETH_ICMP_RESPONDER=1 selects the OTHER q700_eth_link branch (standalone
# ICMP responder, no SONIC DMA), so defaulting it to 1 made a plain ETH_ENABLE=1
# build quietly produce the wrong ethernet.
set eth_enable [parse_bool_env ETH_ENABLE 1]
set eth_icmp_responder [parse_bool_env ETH_ICMP_RESPONDER 0]
set eth_debug_enable [parse_bool_env ETH_DEBUG_ENABLE 0]
# The taxi/rk5-eth sources are VENDORED IN-TREE at vendor/rk5-eth, which mirrors
# the original rk5-eth layout exactly, so every $eth_rk5_dir-relative path below
# resolves without a sibling checkout.  ETH_RK5_DIR still overrides, to build
# against an external rk5-eth tree.
set eth_rk5_dir [file normalize [expr {[info exists ::env(ETH_RK5_DIR)] ? $::env(ETH_RK5_DIR) : "$proj_root/vendor/rk5-eth"}]]


# 53C96 register-access trace ring gate.  ENABLE_SCSI_TRACE=1 (DEFAULT)
# passes `SCSI_TRACE_ENABLE` so fpga_top_peripherals.vh instantiates
# rtl/soc/scsi_trace_ring.v as a snoop on the peripheral-bus port into
# rtl/mac/scsi.v; readout is through u_vhdd_ctrl at 0x5010_0020..0x28
# (tools/jtag_repl.tcl `scsi-trace`).
#
# Unlike ENABLE_VIO / ENABLE_ILA this needs no Vivado IP -- it is plain
# RTL already in the read_verilog list -- so the gate is a define only.
#
# Default ON: reading the live 53C96 over JTAG is destructive (a reg-2
# read pops the FIFO, a reg-5 read clears the pending IRQ), so without
# this ring the register traffic behind a storage hang is unobservable.
# COST: one 4096x32 ring (~4 RAMB36) plus its capture/CDC logic, roughly
# a hundred LUTs.  This design has historically routed close to the edge
# (~83% LUT, congestion level 6) and the ring was removed twice for
# exactly that reason -- if a build fails to route, set
# ENABLE_SCSI_TRACE=0 and rebuild; nothing else needs to change.
set enable_scsi_trace [parse_bool_env ENABLE_SCSI_TRACE 1]
set allow_undebuggable_fpga_build [parse_bool_env ALLOW_UNDEBUGGABLE_FPGA_BUILD 0]

if {$eth_debug_enable && (!$eth_enable || $eth_icmp_responder)} {
    error "ETH_DEBUG_ENABLE=1 requires ETH_ENABLE=1 and ETH_ICMP_RESPONDER=0 (the SONIC DMA datapath)"
}

# ──────────────────────────────────────────────────────────────────────────────
# Incremental-compile reference DCP.
#
# When INCREMENTAL_REF_DCP points at an existing routed checkpoint, opt_design /
# place_design / route_design will reuse placement+routing for unchanged
# hierarchy — typical impl-time savings are 40–60 % on peripheral-only RTL
# changes.  After a successful route, we copy the new route.dcp back to the
# reference path so the next iteration can incrementally consume it.
#
# To force a clean run (e.g. after a major top-level reshape), either delete
# the file at INCREMENTAL_REF_DCP or set NO_INCREMENTAL=1.
# ──────────────────────────────────────────────────────────────────────────────
set no_incremental [parse_bool_env NO_INCREMENTAL 0]
set incremental_ref_dcp ""
if {[info exists ::env(INCREMENTAL_REF_DCP)] && \
        $::env(INCREMENTAL_REF_DCP) ne ""} {
    set incremental_ref_dcp [file normalize $::env(INCREMENTAL_REF_DCP)]
}

# DDR/MIG path selection.  Default generic synth/impl invocations keep using
# ddr_ctrl's small SIM_MODEL so tiny smoke images can be built without the
# Xilinx MIG IP.  Real FPGA bitstream targets set REAL_FPGA_BUILD=1 and
# USE_REAL_MIG=1, selecting the non-SIM_MODEL DDR4/MIG path.  ddr_pincheck
# also forces that shell for a parse/XDC smoke check.
set use_sim_model [expr {[info exists ::env(USE_REAL_MIG)] && $::env(USE_REAL_MIG) == "0"}]
if {$mode eq "ddr_pincheck"} {
    set use_sim_model 0
}
set real_fpga_build [expr {![info exists ::env(REAL_FPGA_BUILD)] || $::env(REAL_FPGA_BUILD) != "0"}]
if {$real_fpga_build && $use_sim_model} {
    puts stderr "ERROR: REAL_FPGA_BUILD=1 requires USE_REAL_MIG=1; refusing to define SIM_MODEL for a real FPGA build."
    exit 1
}
if {$enable_pcie_xdma && $use_sim_model} {
    puts stderr "ERROR: ENABLE_PCIE_XDMA=1 requires USE_REAL_MIG=1; PCIe ports and the XDMA reference clock exist only in the real-hardware shell."
    exit 1
}
if {$enable_pcie_xdma && $enable_jtag_axi} {
    puts stderr "ERROR: ENABLE_PCIE_XDMA=1 and ENABLE_JTAG_AXI=1 both drive xbar M1; enable only one host master."
    exit 1
}
if {$real_fpga_build && $mode eq "full_impl" && !$allow_undebuggable_fpga_build} {
    if {!$enable_vio} {
        puts stderr "ERROR: REAL_FPGA_BUILD=1 full implementation now requires ENABLE_VIO=1."
        puts stderr "       Use the canonical debug-capable target 'make fpga-100mhz-jtag-bitstream-dram',"
        puts stderr "       or set ALLOW_UNDEBUGGABLE_FPGA_BUILD=1 only for an intentional non-debug image."
        exit 1
    }
    if {!$enable_jtag_axi && !$enable_pcie_xdma} {
        puts stderr "ERROR: REAL_FPGA_BUILD=1 full implementation now requires a host debug path."
        puts stderr "       Enable ENABLE_JTAG_AXI=1 or ENABLE_PCIE_XDMA=1, or set ALLOW_UNDEBUGGABLE_FPGA_BUILD=1"
        puts stderr "       only for an intentional non-debug image."
        exit 1
    }
}

# Fabric clock used by clk_rst/video_top.  SIM_MODEL keeps using the 200 MHz
# T24/U24 sys_clk pair as a fabric clock.  Real-MIG builds leave T24/U24
# exclusively on the MIG DDR reference input and use the AB7/AB6 100 MHz
# MGTREFCLK0 pair through IBUFDS_GTE4.ODIV2.
set board_clk_hz [expr {$use_sim_model ? 200000000 : 100000000}]

if {[info exists ::env(PCIE_TEST_DIR)]} {
    set pcie_test_dir $::env(PCIE_TEST_DIR)
} elseif {[file exists /offlinenas/share/FPGA/pcie_test]} {
    set pcie_test_dir /offlinenas/share/FPGA/pcie_test
} else {
    set pcie_test_dir [file join $::env(HOME) FPGA pcie_test]
}
set pcie_test_mig_dcp [file normalize [file join $pcie_test_dir \
    pcie_test.runs design_1_ddr4_0_1_synth_1 design_1_ddr4_0_1.dcp]]
set pcie_test_xdma_xci [file normalize [file join $pcie_test_dir \
    pcie_test.srcs sources_1 bd design_1 ip design_1_xdma_0_0 design_1_xdma_0_0.xci]]
if {[info exists ::env(DDR4_MIG_DCP)]} {
    set ddr4_mig_dcp [file normalize $::env(DDR4_MIG_DCP)]
    set ddr4_mig_dcp_source "DDR4_MIG_DCP override"
} else {
    set ddr4_mig_dcp [file normalize [file join $proj_root \
        build ddr4_mig design_1_ddr4_0_1.dcp]]
    set ddr4_mig_dcp_source "repo-generated build/ddr4_mig"
}
set allow_pcie_test_mig_dcp [expr {
    [info exists ::env(USE_PCIE_TEST_MIG_DCP)] &&
    $::env(USE_PCIE_TEST_MIG_DCP) == "1"
}]
if {![info exists ::env(DDR4_MIG_DCP)] &&
    ![file exists $ddr4_mig_dcp] &&
    $allow_pcie_test_mig_dcp &&
    [file exists $pcie_test_mig_dcp]} {
    set ddr4_mig_dcp $pcie_test_mig_dcp
    set ddr4_mig_dcp_source "legacy pcie_test fallback"
}
if {[info exists ::env(PCIE_XDMA_XCI)]} {
    set pcie_xdma_xci [file normalize $::env(PCIE_XDMA_XCI)]
    set pcie_xdma_source "PCIE_XDMA_XCI override"
} else {
    # Prefer the in-tree IP XCI (output products adjacent) so read_ip can
    # resolve the generated netlist/DCP; the root-level copy is a detached
    # artifact that Vivado 2025.2 treats as "moved" (products not found).
    set pcie_xdma_xci_intree [file normalize [file join $proj_root \
        build pcie_xdma ip design_1_xdma_0_0 design_1_xdma_0_0.xci]]
    if {[file exists $pcie_xdma_xci_intree]} {
        set pcie_xdma_xci $pcie_xdma_xci_intree
        set pcie_xdma_source "repo-generated build/pcie_xdma (in-tree IP dir)"
    } else {
        set pcie_xdma_xci [file normalize [file join $proj_root \
            build pcie_xdma design_1_xdma_0_0.xci]]
        set pcie_xdma_source "repo-generated build/pcie_xdma"
    }
}
if {[info exists ::env(PCIE_XDMA_DCP)]} {
    set pcie_xdma_dcp [file normalize $::env(PCIE_XDMA_DCP)]
} elseif {[info exists ::env(PCIE_XDMA_XCI)]} {
    set pcie_xdma_dcp [file normalize [file rootname $pcie_xdma_xci].dcp]
} else {
    set pcie_xdma_dcp [file normalize [file join $proj_root \
        build pcie_xdma design_1_xdma_0_0.dcp]]
}
set allow_pcie_test_xdma_xci [expr {
    [info exists ::env(USE_PCIE_TEST_XDMA_XCI)] &&
    $::env(USE_PCIE_TEST_XDMA_XCI) == "1"
}]
if {![info exists ::env(PCIE_XDMA_XCI)] &&
    ![file exists $pcie_xdma_xci] &&
    $allow_pcie_test_xdma_xci &&
    [file exists $pcie_test_xdma_xci]} {
    set pcie_xdma_xci $pcie_test_xdma_xci
    set pcie_xdma_source "legacy pcie_test fallback"
}
if {($mode eq "synth_only" || $mode eq "full_impl") &&
    $ddr4_mig_dcp_source eq "legacy pcie_test fallback"} {
    puts stderr "ERROR: USE_PCIE_TEST_MIG_DCP=1 is reference-only and is no longer accepted for synth/impl."
    puts stderr "       Generate the repo-owned MIG under build/ddr4_mig with 'make ddr4-mig',"
    puts stderr "       or point DDR4_MIG_DCP at an explicit artifact you intend to build with."
    exit 1
}
if {($mode eq "synth_only" || $mode eq "full_impl") &&
    $pcie_xdma_source eq "legacy pcie_test fallback"} {
    puts stderr "ERROR: USE_PCIE_TEST_XDMA_XCI=1 is reference-only and is no longer accepted for synth/impl."
    puts stderr "       Generate the repo-owned XDMA IP under build/pcie_xdma with 'make pcie-xdma',"
    puts stderr "       or point PCIE_XDMA_XCI/PCIE_XDMA_DCP at explicit artifacts you intend to build with."
    exit 1
}

set allow_sim_model_rom_wrap [expr {
    [info exists ::env(ALLOW_SIM_MODEL_ROM_WRAP)] &&
    $::env(ALLOW_SIM_MODEL_ROM_WRAP) == "1"
}]

# ──────────────────────────────────────────────────────────────────────────────
# Target clock and first-board runtime generics
# ──────────────────────────────────────────────────────────────────────────────
# CORE_CLK_DIVIDE is the first-board clock-plan knob.  Keep it to the
# legal BUFGCE_DIV set so the 50 MHz board test is explicit instead of
# accidental.  When omitted, the core simply passes through the board clock.
set core_clk_divide [parse_int_env CORE_CLK_DIVIDE 1]
if {[lsearch -exact {1 2 4 8} $core_clk_divide] < 0} {
    puts stderr "ERROR: CORE_CLK_DIVIDE must be one of {1, 2, 4, 8}; got $core_clk_divide."
    exit 1
}
if {[info exists ::env(CORE_CLK_HZ)]} {
    set core_clk_hz [parse_int_env CORE_CLK_HZ 0]
} else {
    set core_clk_hz [expr {$board_clk_hz / $core_clk_divide}]
}
# CORE_MMCM — run the core ABOVE the board oscillator.
#
# The board reference is a fixed 100 MHz MGTREFCLK and a BUFG_GT can only
# DIVIDE, so the divide-only guard below is correct for every topology
# EXCEPT this one.  With CORE_MMCM=1, rtl/soc/fpga_top_clocks.vh adds an
# MMCM (100 MHz in, VCO 1200 MHz) whose CLKOUT0 is the core clock and
# whose CLKOUT1 is pb at EXACTLY 1200/24 = 50.000 MHz.  Only the four
# frequencies that divide 1200 MHz cleanly are accepted, because an
# awkward intermediate would either miss the VCO or make pb non-exact --
# and every Mac peripheral timebase in this SoC (VIA phi2, SCC BRG, ADB,
# audio PWM, peripheral-bus timeouts) is derived from PB_CLK_HZ.
set core_mmcm [parse_bool_env CORE_MMCM 0]
if {$core_mmcm} {
    if {[lsearch -exact {100000000 120000000 150000000 200000000} $core_clk_hz] < 0} {
        puts stderr "ERROR: CORE_MMCM=1 supports CORE_CLK_HZ in {100000000, 120000000, 150000000, 200000000}"
        puts stderr "       (the MMCM VCO is 1200 MHz and pb must stay exactly 50 MHz = 1200/24); got $core_clk_hz."
        exit 1
    }
    if {$core_clk_divide != 1} {
        puts stderr "ERROR: CORE_MMCM=1 and CORE_CLK_DIVIDE=$core_clk_divide are mutually exclusive:"
        puts stderr "       the MMCM replaces the BUFG_GT divider entirely.  Leave CORE_CLK_DIVIDE unset."
        exit 1
    }
    puts "=== CORE_MMCM=1: core clock from MMCM (VCO 1200 MHz), CORE_CLK_HZ=$core_clk_hz, pb fixed at 50.000 MHz ==="
    # The -verilog_define itself is appended where synth_def_args is built
    # (it is created further down, so a lappend here would be discarded).
} else {
    set expected_core_clk_hz [expr {$board_clk_hz / $core_clk_divide}]
    if {$core_clk_hz != $expected_core_clk_hz} {
        puts stderr "ERROR: CORE_CLK_HZ must equal ${expected_core_clk_hz} when CORE_CLK_DIVIDE=$core_clk_divide on a ${board_clk_hz} Hz board clock."
        exit 1
    }
}
set pb_clk_hz [parse_int_env PB_CLK_HZ 50000000]
if {$pb_clk_hz != 50000000} {
    puts stderr "ERROR: PB_CLK_HZ is currently fixed at 50000000; got $pb_clk_hz."
    exit 1
}
set video_smoke [parse_int_env VIDEO_SMOKE 0]
if {$video_smoke != 0 && $video_smoke != 1} {
    puts stderr "ERROR: VIDEO_SMOKE must be 0 or 1; got $video_smoke."
    exit 1
}
set boot_rom_sectors [parse_int_env BOOT_ROM_SECTORS 2048]
set sd_safe_cmd25 [parse_int_env SD_SAFE_CMD25 0]
if {$sd_safe_cmd25 ni {0 1}} {error "SD_SAFE_CMD25 must be 0 or 1"}
if {$sd_safe_cmd25 && [info exists ::env(ENABLE_SD_JTAG_WRITER)]} {
    error "SD_SAFE_CMD25 requires the preemptive provisioning writer to be disabled"
}
if {$boot_rom_sectors <= 0 || $boot_rom_sectors > 65535} {
    puts stderr "ERROR: BOOT_ROM_SECTORS must be in the range 1..65535; got $boot_rom_sectors."
    exit 1
}
set sim_model_ddr_bytes [expr {1 << (12 + 4)}]
set boot_rom_bytes [expr {$boot_rom_sectors * 512}]
if {$use_sim_model &&
    ($mode eq "synth_only" || $mode eq "full_impl") &&
    $boot_rom_bytes > $sim_model_ddr_bytes &&
    !$allow_sim_model_rom_wrap} {
    puts stderr "ERROR: SIM_MODEL DDR is only ${sim_model_ddr_bytes} bytes, but BOOT_ROM_SECTORS=$boot_rom_sectors needs ${boot_rom_bytes} bytes."
    puts stderr "       That build would wrap the ROM image and corrupt the reset-vector area."
    puts stderr "       Use USE_REAL_MIG=1 for a DRAM-backed ROM image, reduce BOOT_ROM_SECTORS for a tiny smoke image,"
    puts stderr "       or set ALLOW_SIM_MODEL_ROM_WRAP=1 only for an intentional negative test."
    exit 1
}
if {!$use_sim_model && ![file exists $ddr4_mig_dcp] &&
    ($mode eq "synth_only" || $mode eq "full_impl")} {
    puts stderr "ERROR: USE_REAL_MIG=1 needs the repo-generated DDR4 MIG DCP, but it was not found:"
    puts stderr "       $ddr4_mig_dcp"
    puts stderr "       Run 'make ddr4-mig' to generate it under build/ddr4_mig, or set DDR4_MIG_DCP to an explicit DCP."
    puts stderr "       For one-off reference comparison only, USE_PCIE_TEST_MIG_DCP=1 can fall back to:"
    puts stderr "       $pcie_test_mig_dcp"
    exit 1
}
if {$enable_pcie_xdma && ![file exists $pcie_xdma_xci]} {
    puts stderr "ERROR: ENABLE_PCIE_XDMA=1 needs the repo-generated XDMA XCI, but it was not found:"
    puts stderr "       $pcie_xdma_xci"
    puts stderr "       Run 'make pcie-xdma-validate' or 'make pcie-xdma' to generate it under build/pcie_xdma."
    puts stderr "       For one-off reference comparison only, USE_PCIE_TEST_XDMA_XCI=1 can fall back to:"
    puts stderr "       $pcie_test_xdma_xci"
    exit 1
}
if {$enable_pcie_xdma &&
    ($mode eq "synth_only" || $mode eq "full_impl") &&
    ![file exists $pcie_xdma_dcp] &&
    !$allow_pcie_test_xdma_xci} {
    puts stderr "ERROR: ENABLE_PCIE_XDMA=1 synthesis/implementation needs the repo-generated XDMA DCP, but it was not found:"
    puts stderr "       $pcie_xdma_dcp"
    puts stderr "       Run 'make pcie-xdma' to generate it under build/pcie_xdma, or set PCIE_XDMA_DCP to an explicit DCP."
    exit 1
}
# TARGET_FREQ_MHZ is a timing budget, not a new board clock.  The board
# oscillator on sys_clk_p is physically 200 MHz and stays constrained at
# 5.000 ns.  The active core clock may be a direct BUFG pass-through or a
# BUFGCE_DIV output from clk_rst, but the 200 MHz input clock itself does
# not change.  TARGET_FREQ_MHZ is translated into a set_max_delay exception
# on the active core clock so Vivado judges core-internal timing against the
# requested budget without inventing a different board oscillator.
set target_freq_mhz [expr {[info exists ::env(TARGET_FREQ_MHZ)] ? double($::env(TARGET_FREQ_MHZ)) : 200.0}]
set target_period_ns [expr {1000.0 / $target_freq_mhz}]

puts "Target: $part @ ${target_freq_mhz} MHz (${target_period_ns} ns)"
puts "Mode: $mode"
puts "Output: $output_dir"
puts "Clock note: active fabric clock is ${board_clk_hz} Hz; CORE_CLK_DIVIDE=$core_clk_divide gives CORE_CLK_HZ=$core_clk_hz Hz."
puts "Clock note: Mac peripheral bus clock PB_CLK_HZ=$pb_clk_hz Hz."
puts "Clock note: TARGET_FREQ_MHZ only relaxes timing; it does not create a new fabric clock."
puts "First-board generics: PB_CLK_HZ=$pb_clk_hz VIDEO_SMOKE=$video_smoke BOOT_ROM_SECTORS=$boot_rom_sectors"
puts "Debug IP: ENABLE_VIO=$enable_vio ENABLE_JTAG_AXI=$enable_jtag_axi ENABLE_PCIE_XDMA=$enable_pcie_xdma ENABLE_ILA=$enable_ila ENABLE_SCSI_TRACE=$enable_scsi_trace"
puts "Ethernet: ETH_ENABLE=$eth_enable ETH_ICMP_RESPONDER=$eth_icmp_responder ETH_DEBUG_ENABLE=$eth_debug_enable"
puts "Real FPGA build guard: REAL_FPGA_BUILD=$real_fpga_build ALLOW_UNDEBUGGABLE_FPGA_BUILD=$allow_undebuggable_fpga_build"
if {$enable_pcie_xdma} {
    puts "PCIe/XDMA path: $pcie_xdma_xci ($pcie_xdma_source)"
}
if {$use_sim_model} {
    puts "DDR path: SIM_MODEL behavioural RAM"
} elseif {[file exists $ddr4_mig_dcp]} {
    puts "DDR path: real DDR4 MIG via $ddr4_mig_dcp ($ddr4_mig_dcp_source)"
} else {
    puts "DDR path: real DDR4 MIG shell; DCP target $ddr4_mig_dcp ($ddr4_mig_dcp_source, not required for $mode)"
}

file mkdir $output_dir
file mkdir $output_dir/checkpoints
file mkdir $output_dir/reports

# ──────────────────────────────────────────────────────────────────────────────
# Read RTL sources
# ──────────────────────────────────────────────────────────────────────────────
proc read_taxi_filelist {file seen_name} {
    upvar 1 $seen_name seen
    set file [file normalize $file]
    if {[dict exists $seen $file]} { return }
    dict set seen $file 1
    if {![file exists $file]} { error "Taxi source-list entry not found: $file" }
    foreach raw [split [read [set fh [open $file r]]] "\n"] {
        set line [string trim [lindex [split $raw "#"] 0]]
        if {$line eq ""} { continue }
        set entry [file normalize [file join [file dirname $file] $line]]
        if {[file extension $entry] eq ".f"} {
            read_taxi_filelist $entry seen
        } else {
            if {![file exists $entry]} { error "Taxi RTL source not found: $entry" }
            read_verilog -sv $entry
        }
    }
    close $fh
}

proc read_all_rtl {rtl_dir} {
    global use_sim_model cpu_m68k040 cpu_m68k040_v eth_enable eth_rk5_dir

    # ── CPU socket occupant ──────────────────────────────────────────────
    # CPU=m68k: the m68k-ooo submodule supplies the whole OoO CPU
    # (m68k_core pipeline + m68k_axi_wrapper socket adapter + debug_ctrl).
    # Mirror the Makefile's CPU_M68K_SRCS glob: every .v under
    # cpu/rtl/core, with NO exclusions.  There used to be a "module-name
    # collision guard" skipping debug_stop_manager.v because
    # rtl/soc/fpga_top.v defined an inline duplicate; that duplicate was
    # deleted 2026-08-19 and cpu/ now holds the single definition, so
    # skipping it here would leave m68k_axi_wrapper's `u_debug_stop`
    # instance undefined at elaboration.  Shared modules
    # that also live in the SoC tree (if_to_axi, axi_narrow_to_wide,
    # irq_agg) sit OUTSIDE cpu/rtl/core (cpu/rtl/sys, cpu/rtl/mac) and
    # are deliberately not read — the SoC copies below win.
    #
    # CPU=m68k040: mirror the Makefile's CPU_M68K040_SRCS — a single
    # generated, self-contained Verilog file (no glob, no shared-module
    # collision to avoid).  Regenerated fresh above, before read_all_rtl
    # is called.
    if {$cpu_m68k040} {
        read_verilog $cpu_m68k040_v
    } else {
        # CPU=stub: standalone SoC build; idle socket occupant.
        read_verilog $rtl_dir/soc/cpu_stub.v
    }

    # Mac peripherals.  VIA1/VIA2/SCC/SCSI/ASC and ENET/SONIC have RTL
    # behavior; Orwell and SWIM/IWM are still conservative stubs.
    read_verilog $rtl_dir/mac/via1.v
    read_verilog $rtl_dir/mac/rtc.v
    read_verilog $rtl_dir/mac/via2.v
    read_verilog $rtl_dir/mac/scsi.v
    read_verilog $rtl_dir/mac/scc.v
    read_verilog $rtl_dir/mac/asc.v
    read_verilog $rtl_dir/mac/q700_eth_sonic.v
    read_verilog -sv $rtl_dir/mac/q700_sonic_cdc.sv
    read_verilog -sv $rtl_dir/mac/q700_sonic_rx_cdc.sv
    read_verilog -sv $rtl_dir/mac/q700_sonic_rx.sv
    read_verilog -sv $rtl_dir/mac/q700_sonic_tx.sv
    read_verilog $rtl_dir/mac/orwell_stub.v
    read_verilog $rtl_dir/mac/iwm_stub.v
    read_verilog $rtl_dir/mac/irq_agg.v
    read_verilog $rtl_dir/mac/video.v
    # DAFB stage 2: the ONLY place the mode arithmetic lives.
    # Instantiated by video.v; Vivado has no module search path here,
    # so it must be read explicitly or elaboration dies at
    # `module 'mode_decode' not found`.
    read_verilog $rtl_dir/mac/mode_decode.v
    read_verilog $rtl_dir/mac/glue.v

    # ADB stack — PIC1654S ADB-modem (runs the real 342s0440-b firmware
    # over VIA1 CB1/CB2) plus the legacy byte-level bridge devices, and
    # the minimal ADB-PHY (no-device behavior) attached to the PIC's
    # external 3-wire bus.
    read_verilog $rtl_dir/mac/pic16c5x.v
    read_verilog $rtl_dir/mac/adb_pic_modem.v
    read_verilog $rtl_dir/mac/adb_phy.v
    read_verilog $rtl_dir/mac/adb_modem.v
    read_verilog $rtl_dir/mac/adb_keyboard.v
    read_verilog $rtl_dir/mac/adb_mouse.v
    read_verilog $rtl_dir/mac/adb_inject.v

    # HDMI video pipeline (rtl/mac/video/*)
    read_verilog $rtl_dir/board/video_phy/mmcm_hdmi.v
    read_verilog $rtl_dir/board/video_phy/vtg.v
    read_verilog $rtl_dir/board/video_phy/i2c_init.v
    # Scan-out stage 3: the ONLY place an admission inequality lives.
    # Instantiated by scanout_placement_sync.v and scanout_fetch.v; same
    # no-search-path rule as mode_decode above -- without this line
    # elaboration dies at `module 'mode_admit' not found`.
    read_verilog $rtl_dir/board/video_phy/mode_admit.v
    read_verilog $rtl_dir/board/video_phy/place_plan.v
    read_verilog $rtl_dir/board/video_phy/scanout_placement_sync.v
    read_verilog $rtl_dir/board/video_phy/fb_reader.v
    read_verilog $rtl_dir/board/video_phy/scanout_fetch.v
    read_verilog $rtl_dir/board/video_phy/pixel_unpack.v
    read_verilog $rtl_dir/board/video_phy/clut.v
    read_verilog $rtl_dir/board/video_phy/upscale.v
    read_verilog $rtl_dir/board/video_phy/compositor.v
    read_verilog $rtl_dir/board/video_phy/scanout_display.v
    read_verilog $rtl_dir/board/video_phy/linebuf_scanout.v
    read_verilog $rtl_dir/board/video_phy/video_top.v
    read_verilog $rtl_dir/board/video_phy/vram_smoke.v

    # System (clocks, AXI, DDR, VRAM, SD)
    read_verilog $rtl_dir/board/clk_rst.v
    read_verilog $rtl_dir/board/reset_debounce.v
    read_verilog $rtl_dir/soc/axi_xbar.v
    read_verilog $rtl_dir/soc/axi_w_skid.v
    # THE DEBUG BUS.  One JTAG-AXI bridge lands on it; the 0x5090_0000 debug
    # window is served locally (CPU debug CSRs / eth debug regs) and everything
    # else is mastered onto the SoC crossbar above.  Instantiated in
    # fpga_top_debug_host.vh under `ifdef JTAG_AXI_ENABLE -- which every
    # production bitstream defines (ENABLE_JTAG_AXI defaults to 1) -- so it is
    # NOT optional here.  It was allowlisted out of this list while it sat
    # out of the datapath; that allowlist entry is gone.
    read_verilog $rtl_dir/soc/axi_dbg_bus.v
    read_verilog $rtl_dir/board/axi_ddr4_mig_bridge.v
    read_verilog $rtl_dir/board/ddr_ctrl.v
    read_verilog $rtl_dir/board/vram.v
    read_verilog $rtl_dir/board/sd_spi.v
    read_verilog $rtl_dir/board/sd_spi_mux.v
    read_verilog $rtl_dir/board/sd_ctrl.v
    read_verilog $rtl_dir/board/sd_jtag_writer.v
    # PRAM persistence to SD LBA 8191 (manual JTAG save/load).  NOT gated
    # by DISABLE_SD_JTAG_WRITER — it shares that slave window via
    # axil_split2 but is a runtime feature, so it ships in every bitstream.
    read_verilog $rtl_dir/soc/axil_split2.v
    read_verilog $rtl_dir/soc/pram_cdc.v
    read_verilog $rtl_dir/soc/pram_sd.v
    read_verilog $rtl_dir/soc/sd_scsi_lba_mapper.v
    read_verilog $rtl_dir/soc/vhdd_sd.v
    # vHDD CSR block and the two-target mux.  These are UNGATED mainline
    # logic -- vhdd_mux is instantiated by rtl/mac/scsi.v and vhdd_ctrl by
    # fpga_top_dma.vh -- so they must be read regardless of defines.
    #
    # They were missing from this list when the vHDD seam first landed, and
    # synthesis died at `module 'vhdd_ctrl' not found` AFTER the modules
    # had passed lint and every unit tb: the Makefile finds sources with
    # `find`, this list is explicit, so an RTL file can be fully verified
    # and still be invisible to the only flow that builds a bitstream.
    # `make check-synth-sources` now fails on exactly that gap.
    read_verilog $rtl_dir/soc/vhdd_ctrl.v
    # 53C96 register-access trace ring.  Instantiated by
    # fpga_top_peripherals.vh under `ifdef SCSI_TRACE_ENABLE (env
    # ENABLE_SCSI_TRACE, default 1).  Read UNCONDITIONALLY: an
    # unreferenced module is pruned, and making the read conditional
    # would turn a define typo into `module not found` instead of a
    # silently-absent ring.
    read_verilog $rtl_dir/soc/scsi_trace_ring.v
    read_verilog $rtl_dir/soc/vhdd_mux.v
    read_verilog $rtl_dir/soc/vhdd_readahead.v
    read_verilog $rtl_dir/soc/sd_scsi_bridge.v
    read_verilog $rtl_dir/soc/boot_fsm.v
    read_verilog $rtl_dir/soc/peripheral_reset_sequencer.v
    # T14/T16 DDR-path additions.  axi_bridge_stale_sink is UNGATED
    # mainline logic (instantiated unconditionally since T14 landed) —
    # it must be read regardless of defines.  The l2c*/scanout/mux files
    # are gated behind L2C_ENABLE / VRAM_IN_DDR; reading them
    # unconditionally is safe (unreferenced modules are pruned).
    read_verilog $rtl_dir/soc/axi_bridge_stale_sink.v
    read_verilog $rtl_dir/soc/axi_bridge_w_pad.v
    read_verilog $rtl_dir/soc/axi_vram_priority_mux3.v
    read_verilog $rtl_dir/soc/axi_vram_smoke_mux.v
    read_verilog $rtl_dir/soc/axil_null_slave.v
    read_verilog $rtl_dir/soc/scanout_ddr_reader.v
    read_verilog $rtl_dir/soc/scanout_line_fetch.v
    read_verilog $rtl_dir/soc/l2c.v
    read_verilog $rtl_dir/soc/ifetch_window_guard.v
    read_verilog $rtl_dir/soc/l2c_ctrl.v
    read_verilog $rtl_dir/soc/l2c_tags.v
    read_verilog $rtl_dir/soc/l2c_data.v
    read_verilog $rtl_dir/soc/l2c_mshr.v
    read_verilog $rtl_dir/soc/l2c_victim.v
    read_verilog $rtl_dir/soc/l2c_victim_sel.v
    read_verilog $rtl_dir/soc/l2c_bypass.v
    read_verilog $rtl_dir/soc/l2c_pri8.v
    read_verilog $rtl_dir/soc/l2c_reset.v
    # sd_provision.v was removed — boot is via JTAG-AXI now

    # CDC / DMA / audio (added alongside the VIO-probe landing — these
    # files existed under rtl/sys but were never appended to the
    # read_verilog list when their parent commits landed.  Without them
    # synth_design fails RTL elab on `module 'audio_i2s' not found`
    # etc.  See follow-up on missing-read cleanup.
    read_verilog $rtl_dir/board/async_fifo.v
    read_verilog $rtl_dir/soc/axi_async_bridge.v
    read_verilog $rtl_dir/soc/axi_pb_lane_shim.v
    read_verilog $rtl_dir/soc/axi_pb_s1_cdc.v
    read_verilog $rtl_dir/soc/axil_async_bridge.v
    read_verilog $rtl_dir/soc/axi_wide_to_axilite.v
    read_verilog $rtl_dir/soc/dma_ctrl.v
    read_verilog -sv $rtl_dir/soc/dma_engine.sv
    read_verilog -sv $rtl_dir/soc/eth_debug_regs.sv
    # SONIC register-access trace ring — gated behind ETH_DEBUG_ENABLE
    # (instantiated only inside that `ifdef in fpga_top_dma.vh).  Reading
    # it unconditionally is safe: an unreferenced module is pruned.
    read_verilog $rtl_dir/soc/sonic_trace_ring.v
    # net-vHDD: the Ethernet-backed volume, gated behind ENABLE_NET_VHDD
    # (instantiated only inside that `ifdef in fpga_top_peripherals.vh).
    # Reading them unconditionally is safe -- unreferenced modules are pruned.
    read_verilog -sv $rtl_dir/board/net_block_framer.sv
    read_verilog -sv $rtl_dir/board/vhdd_net.sv
    read_verilog $rtl_dir/board/audio_i2s.v
    read_verilog $rtl_dir/board/audio_hdmi_bridge.v
    read_verilog $rtl_dir/board/audio_pwm.v
    read_verilog $rtl_dir/board/uart_byte_bridge.v
    read_verilog $rtl_dir/board/pulse_cdc.v
    if {$eth_enable} {
        if {![file isdirectory $eth_rk5_dir]} {
            error "ETH_ENABLE=1 requires rk5-eth at $eth_rk5_dir (set ETH_RK5_DIR to override)"
        }
        set taxi_seen [dict create]
        read_taxi_filelist $eth_rk5_dir/third_party/taxi/src/eth/rtl/taxi_eth_mac_1g_rgmii_fifo.f taxi_seen
        read_verilog -sv $eth_rk5_dir/third_party/taxi/src/sync/rtl/taxi_sync_reset.sv
        read_verilog -sv $eth_rk5_dir/rtl/icmp_echo_responder.sv
        read_verilog -sv $rtl_dir/board/q700_eth_link.sv
    }

    # Integration glue + adapters
    read_verilog $rtl_dir/soc/if_to_axi.v
    read_verilog $rtl_dir/soc/axi_narrow_to_wide.v
    read_verilog $rtl_dir/soc/peripheral_bus.v

    # Top
    if {!$use_sim_model} {
        read_verilog $rtl_dir/board/vendor/design_1_ddr4_0_1_stub.v
    }
    read_verilog $rtl_dir/soc/fpga_top.v
}

read_all_rtl $rtl_dir
ensure_project_part $part

# Vivado's `set_property generic` parses `0x...`-prefixed values as STRINGS
# (Verilog only knows the `'h` / `'d` / `'b` radix prefixes — `0x` is a Tcl
# convention).  When BUILD_ID was previously passed as `BUILD_ID=0x3b701463`
# Vivado would silently truncate it: "Parameter BUILD_ID bound to: 1463 -
# type: string".  Convert to an explicit Verilog 32-bit decimal literal so
# the synth tool binds it as the 32-bit integer the RTL declares.  Keep
# `$build_id` as the canonical `0x...` form for the buildinfo file and the
# `puts` banner so the JTAG REPL's regex check still works.
set build_id_int [expr {$build_id & 0xFFFFFFFF}]
set build_id_generic "32'd${build_id_int}"
puts "jtag_repl build_id: $build_id (generic = $build_id_generic)"

# Top-level generics.  CORE_CLK_DIVIDE lets the first board test drop the
# core clock without changing RTL topology.  CORE_CLK_HZ must stay consistent
# with the active fabric clock: 200 MHz for SIM_MODEL, 100 MHz for real MIG.
set top_generic_list [list \
    CORE_CLK_HZ=$core_clk_hz \
    CORE_CLK_DIVIDE=$core_clk_divide \
    PB_CLK_HZ=$pb_clk_hz \
    VIDEO_SMOKE=$video_smoke \
    ETH_ICMP_RESPONDER=$eth_icmp_responder \
    BOOT_ROM_SECTORS=$boot_rom_sectors \
    SD_SAFE_CMD25=$sd_safe_cmd25 \
    BUILD_ID=$build_id_generic
]
set_property generic [join $top_generic_list " "] [current_fileset]
puts "=== TOP GENERICS: CORE_CLK_HZ=$core_clk_hz CORE_CLK_DIVIDE=$core_clk_divide PB_CLK_HZ=$pb_clk_hz VIDEO_SMOKE=$video_smoke ETH_ICMP_RESPONDER=$eth_icmp_responder BOOT_ROM_SECTORS=$boot_rom_sectors BUILD_ID=$build_id_generic (canonical=$build_id) ==="

# ──────────────────────────────────────────────────────────────────────────────
# Debug VIO IP — generated out-of-context once and cached under
# $output_dir/ip/debug_vio.  Probe widths match the wires brought out
# to the debug_vio instance at the bottom of rtl/fpga_top.v:
#
# Compact map v27 is documented in synth/vio_dashboard.tcl.
# Values and reset outputs remain enabled; input activity tracking is off.
# ──────────────────────────────────────────────────────────────────────────────
# v15 (2026-07-24): the marker_ok check below only verifies probe
# COUNT/WIDTH, not which actual net each probe_inN resolves to. The
# L2C/VRAM-in-DDR merge changed enough hierarchy that a build reused a
# stale-but-count-matching cached debug_vio IP, producing duplicate-
# suffixed probe names ("s0_wready"/"s0_wready_1",
# "vio_rst_bundle"/"vio_rst_bundle_1") in the .ltx and readbacks that
# didn't reflect the new netlist. Bumping the marker forces a clean
# regenerate. If this recurs after a future hierarchy change with an
# unchanged probe count/width, that's the same class of bug again --
# bump the version rather than trusting a "probe_count matches" cache
# hit blindly.
proc gen_debug_vio_ip {output_dir part} {
    set ip_dir $output_dir/ip
    set ip_xci $ip_dir/debug_vio/debug_vio.xci
    set ip_dcp $ip_dir/debug_vio/debug_vio.dcp
    # Compact map v27. Exact configuration is the cache key, including
    # all probe widths and the disabled activity-detector setting.
    set widths {32 6 10 8 16 32 32 32 32 68 187 48 96 160 84}
    set config [list CONFIG.C_NUM_PROBE_IN [llength $widths] \
        CONFIG.C_NUM_PROBE_OUT 2 CONFIG.C_PROBE_OUT0_WIDTH 5 \
        CONFIG.C_PROBE_OUT0_INIT_VAL 0x0 CONFIG.C_PROBE_OUT1_WIDTH 1 \
        CONFIG.C_PROBE_OUT1_INIT_VAL 0x0 CONFIG.C_EN_PROBE_IN_ACTIVITY 0]
    set n 0
    foreach width $widths {
        lappend config CONFIG.C_PROBE_IN${n}_WIDTH $width
        incr n
    }
    set ip_marker $ip_dir/debug_vio/probe_config.txt
    set signature [list probe_map=v27 part=$part config=$config]
    file mkdir $ip_dir
    set marker_ok 0
    if {[file exists $ip_marker]} {
        set fh [open $ip_marker r]
        set marker_ok [expr {[string trim [read $fh]] eq $signature}]
        close $fh
    }
    if {[file exists $ip_xci] && [file exists $ip_dcp] && $marker_ok} {
        puts "=== Reusing cached debug_vio IP at $ip_dcp ==="
        return [list $ip_xci $ip_dcp]
    } elseif {[file exists [file dirname $ip_xci]]} {
        puts "=== Existing debug_vio IP is missing/stale/wrong-part; regenerating ==="
        file delete -force $ip_dir/debug_vio
    }
    puts "=== Generating debug_vio IP OOC into $ip_dir ==="
    create_project -in_memory -part $part -force
    create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
        -module_name debug_vio -dir $ip_dir
    set_property -dict $config [get_ips debug_vio]
    generate_target {synthesis} [get_ips debug_vio]
    synth_ip [get_ips debug_vio]
    set fh [open $ip_marker w]
    puts $fh $signature
    close $fh
    close_project
    return [list $ip_xci $ip_dcp]
}

if {$enable_vio} {
    set ip_paths [gen_debug_vio_ip $output_dir $part]
    set vio_xci [lindex $ip_paths 0]
    ensure_project_part $part
    read_ip $vio_xci
    puts "=== debug_vio IP read: $vio_xci ==="
}

# ──────────────────────────────────────────────────────────────────────────────
# Debug ILA IP — same pre-synth IP-gen pattern as VIO.  Generator lives in
# synth/debug_ila.tcl (sourced here, called when ENABLE_ILA=1).  See
# docs/ila_a7_drift_probes.md for the probe map + HW Manager trigger
# recipes.
# ──────────────────────────────────────────────────────────────────────────────
source $synth_dir/debug_ila.tcl
if {$enable_ipc_ila} {
    source $synth_dir/ipc_ila.tcl
    read_ip [gen_ipc_ila_ip $output_dir $part]
}
if {$enable_ila} {
    set ip_paths [gen_debug_ila_ip $output_dir $part]
    set ila_xci [lindex $ip_paths 0]
    ensure_project_part $part
    read_ip $ila_xci
    puts "=== debug_ila IP read: $ila_xci ==="
}

# JTAG-to-AXI IP — optional host AXI master for first-board ROM patch/load.
# The RTL connects this to xbar M1 through the existing 32→128 adapter.  Use
# single 32-bit hw_axi transactions until the broader DDR burst bridge lands.
proc gen_jtag_axi_ip {output_dir part} {
    set ip_dir $output_dir/ip
    set ip_xci $ip_dir/debug_jtag_axi/debug_jtag_axi.xci
    set ip_dcp $ip_dir/debug_jtag_axi/debug_jtag_axi.dcp
    set ip_marker $ip_dir/debug_jtag_axi/m68k_ooo_jtag_axi_v2.txt
    file mkdir $ip_dir

    set marker_ok 0
    if {[file exists $ip_marker]} {
        set marker_fh [open $ip_marker r]
        set marker_txt [read $marker_fh]
        close $marker_fh
        set marker_ok [expr {[string first "jtag_axi=v2" $marker_txt] >= 0 &&
                             [string first "part=$part" $marker_txt] >= 0}]
    }
    if {[file exists $ip_dcp] && $marker_ok} {
        puts "=== Reusing cached debug_jtag_axi IP at $ip_dcp ==="
        return [list $ip_xci $ip_dcp]
    } elseif {[file exists [file dirname $ip_xci]]} {
        puts "=== Existing debug_jtag_axi IP is missing/stale/wrong-part; regenerating ==="
        file delete -force $ip_dir/debug_jtag_axi
    }

    puts "=== Generating debug_jtag_axi IP OOC into $ip_dir ==="
    create_project -in_memory -part $part -force
    create_ip -name jtag_axi -vendor xilinx.com -library ip -version 1.2 \
        -module_name debug_jtag_axi -dir $ip_dir
    set_property -dict [list \
        CONFIG.PROTOCOL {0} \
        CONFIG.M_HAS_BURST {0} \
        CONFIG.M_AXI_DATA_WIDTH {32} \
        CONFIG.M_AXI_ADDR_WIDTH {32} \
        CONFIG.M_AXI_ID_WIDTH {1} \
        CONFIG.RD_TXN_QUEUE_LENGTH {1} \
        CONFIG.WR_TXN_QUEUE_LENGTH {1} \
    ] [get_ips debug_jtag_axi]
    generate_target {synthesis} [get_ips debug_jtag_axi]
    synth_ip [get_ips debug_jtag_axi]
    set marker_fh [open $ip_marker w]
    puts $marker_fh "jtag_axi=v2"
    puts $marker_fh "part=$part"
    puts $marker_fh "data_width=32"
    puts $marker_fh "host_contract=single_word_transactions"
    close $marker_fh
    close_project
    return [list $ip_xci $ip_dcp]
}

if {$enable_jtag_axi} {
    set ip_paths [gen_jtag_axi_ip $output_dir $part]
    set jtag_xci [lindex $ip_paths 0]
    ensure_project_part $part
    read_ip $jtag_xci
    puts "=== debug_jtag_axi IP read: $jtag_xci ==="
}

# PCIe/XDMA host-access IP -- optional, real-hardware only.  The normal path
# consumes the repo-generated XCI/DCP under build/pcie_xdma.  pcie_test is used
# only as the parameter reference for the generator and an explicit fallback
# when USE_PCIE_TEST_XDMA_XCI=1 is set for comparison work.
proc assert_xdma_xci_contract {xci_path} {
    if {![file exists $xci_path]} {
        puts stderr "ERROR: ENABLE_PCIE_XDMA=1 needs the XDMA XCI, but it was not found:"
        puts stderr "       $xci_path"
        exit 1
    }
    set fh [open $xci_path r]
    set txt [read $fh]
    close $fh
    set needles [list]
    lappend needles {"component_reference": "xilinx.com:ip:xdma:4.2"}
    lappend needles {"ip_revision": "2"}
    lappend needles "\"pl_link_cap_max_link_width\": \[ \{ \"value\": \"X4\""
    lappend needles "\"pl_link_cap_max_link_speed\": \[ \{ \"value\": \"8.0_GT/s\""
    lappend needles "\"axi_data_width\": \[ \{ \"value\": \"128_bit\""
    lappend needles "\"axisten_freq\": \[ \{ \"value\": \"250\""
    lappend needles "\"axist_bypass_en\": \[ \{ \"value\": \"true\""
    lappend needles "\"axi_bypass_64bit_en\": \[ \{ \"value\": \"true\""
    lappend needles "\"select_quad\": \[ \{ \"value\": \"GTY_Quad_224\""
    foreach needle $needles {
        if {[string first $needle $txt] < 0} {
            puts stderr "ERROR: XDMA XCI does not match the expected host-access reference."
            puts stderr "       Missing marker: $needle"
            puts stderr "       XCI: $xci_path"
            exit 1
        }
    }
}

if {$enable_pcie_xdma} {
    assert_xdma_xci_contract $pcie_xdma_xci
    read_ip $pcie_xdma_xci
    puts "=== PCIe/XDMA IP read: $pcie_xdma_xci ==="
}

# Constraint files.  Read fpga_top.xdc FIRST so the base clock constraint
# exists before any dependent exceptions; then add the real-MIG-only fabric
# clock / DDR pin constraints when USE_REAL_MIG=1, then HDMI.  In SIM_MODEL
# mode the DDR4 and AB7/AB6 fabric-clock ports are not present and Vivado
# would reject those `get_ports` lookups.

# XDC read.  fpga_top.xdc constrains sys_clk_p at 200 MHz for SIM_MODEL and
# for the MIG DDR reference.  In real-MIG builds it also constrains the
# separate AB7/AB6 fabric clock at 100 MHz.  TARGET_FREQ_MHZ remains a timing
# budget; the physical input clocks stay at their board frequencies.
read_xdc $synth_dir/fpga_top.xdc
if {!$use_sim_model} {
    read_xdc $synth_dir/fpga_top_real_mig.xdc
    read_xdc $synth_dir/ddr4.xdc
}
if {$enable_pcie_xdma} {
    read_xdc $synth_dir/pcie_xdma.xdc
}
read_xdc $synth_dir/hdmi.xdc
# audio_pwm.xdc constrains the Σ-Δ PWM audio outputs on AN9134 NC pins
# J1.35 / J1.36.  See docs/superpowers/specs/2026-05-04-an9134-pwm-audio-
# design.md.  The package_pin lines are commented out in audio_pwm.xdc
# until the carrier-side FPGA pins are confirmed; the IOSTANDARD / DRIVE
# / SLEW / false_path constraints still apply once pins are uncommented.
read_xdc $synth_dir/audio_pwm.xdc
if {$eth_enable} {
    read_xdc $synth_dir/ethernet_rgmii.xdc
}

# ──────────────────────────────────────────────────────────────────────────────
# ddr_pincheck: explicit DDR4/MIG-shell smoke build.
#
# This mode exists to make first-hardware DDR readiness testable without
# implying that the full SoC can boot from real DDR yet.  It forces the
# real-hardware fpga_top port list, reads ddr4.xdc, elaborates the RTL
# tree, and stops before synthesis/checkpoint/bitstream output.
# ──────────────────────────────────────────────────────────────────────────────
if {$mode eq "ddr_pincheck"} {
    puts "=== DDR PINCHECK: RTL ELAB + DDR4 XDC ONLY ==="
    set pincheck_def_args [list]
    if {$cpu_m68k040} {
        # Not optional: axi_i is native 256b for this CPU with no
        # CPU-side downconverter (see the CPU socket select comment above).
        lappend pincheck_def_args -verilog_define CPU_M68K040 \
            -verilog_define L2C_ENABLE -verilog_define VRAM_IN_DDR
    }
    if {$enable_vio} { lappend pincheck_def_args -verilog_define VIO_ENABLE }
    if {$enable_ila} { lappend pincheck_def_args -verilog_define ILA_ENABLE }
    if {$perf_detail_enable} { lappend pincheck_def_args -verilog_define PERF_DETAIL_ENABLE }
    if {$enable_ipc_ila} { lappend pincheck_def_args -verilog_define IPC_ILA_ENABLE }
    if {$enable_scsi_trace} { lappend pincheck_def_args -verilog_define SCSI_TRACE_ENABLE }
    if {$enable_jtag_axi} { lappend pincheck_def_args -verilog_define JTAG_AXI_ENABLE }
    if {$enable_pcie_xdma} { lappend pincheck_def_args -verilog_define PCIE_XDMA_ENABLE }
    if {$eth_enable} { lappend pincheck_def_args -verilog_define ETH_ENABLE }
    if {$eth_debug_enable} { lappend pincheck_def_args -verilog_define ETH_DEBUG_ENABLE }
    # debug_ctrl.v's local cycle/inst-count fallback is for unit testing only;
    # the integrated bitstream always feeds real 64-bit external counters.
    lappend pincheck_def_args -verilog_define DEBUG_CTRL_NO_LOCAL_FALLBACK
    synth_design \
        -rtl \
        -top fpga_top \
        -part $part \
        -include_dirs $rtl_include_dirs \
        {*}$pincheck_def_args
    puts "=== DDR PINCHECK: non-SIM_MODEL DDR4 pin shell + constraints parse OK ==="
    puts "=== DDR PINCHECK: no synthesis, checkpoint, or bitstream emitted ==="
    exit 0
}

# ──────────────────────────────────────────────────────────────────────────────
# dry_run: parse-only lint (no full synth).  Walks the elaborated design
# tree via `synth_design -rtl` which performs RTL elab + basic check
# without running full synthesis.  Used by agents to validate the TCL
# + the complete RTL read list without burning a 30-min synth run.
# ──────────────────────────────────────────────────────────────────────────────
if {$mode eq "dry_run"} {
    puts "=== DRY-RUN: RTL ELAB ONLY ==="
    set dry_def_args [list]
    if {$cpu_m68k040} {
        lappend dry_def_args -verilog_define CPU_M68K040 \
            -verilog_define L2C_ENABLE -verilog_define VRAM_IN_DDR
    }
    if {$use_sim_model} { lappend dry_def_args -verilog_define SIM_MODEL }
    if {$enable_vio}    { lappend dry_def_args -verilog_define VIO_ENABLE }
    if {$enable_ila}    { lappend dry_def_args -verilog_define ILA_ENABLE }
    if {$perf_detail_enable} { lappend dry_def_args -verilog_define PERF_DETAIL_ENABLE }
    if {$enable_ipc_ila} { lappend dry_def_args -verilog_define IPC_ILA_ENABLE }
    if {$enable_scsi_trace} { lappend dry_def_args -verilog_define SCSI_TRACE_ENABLE }
    if {$enable_jtag_axi} { lappend dry_def_args -verilog_define JTAG_AXI_ENABLE }
    if {$enable_pcie_xdma} { lappend dry_def_args -verilog_define PCIE_XDMA_ENABLE }
    if {$eth_enable} { lappend dry_def_args -verilog_define ETH_ENABLE }
    if {$eth_debug_enable} { lappend dry_def_args -verilog_define ETH_DEBUG_ENABLE }
    if {$core_mmcm} { lappend dry_def_args -verilog_define CORE_MMCM }
    lappend dry_def_args -verilog_define DEBUG_CTRL_NO_LOCAL_FALLBACK
    synth_design \
        -rtl \
        -top fpga_top \
        -part $part \
        -include_dirs $rtl_include_dirs \
        {*}$dry_def_args
    puts "=== DRY-RUN: TCL + RTL parse OK ==="
    exit 0
}

# ──────────────────────────────────────────────────────────────────────────────
# clock_report: parse-only clock audit.  Uses the same RTL/XDC read path as
# dry_run, then prints the clocks Vivado can currently see so first-board
# bring-up can distinguish the fixed 200 MHz sysclk200 input from any real
# generated clocks (for example the HDMI MMCM pclk).
# ──────────────────────────────────────────────────────────────────────────────
if {$mode eq "clock_report"} {
    puts "=== CLOCK REPORT: RTL ELAB ONLY ==="
    set clock_def_args [list]
    if {$cpu_m68k040} {
        lappend clock_def_args -verilog_define CPU_M68K040 \
            -verilog_define L2C_ENABLE -verilog_define VRAM_IN_DDR
    }
    if {$use_sim_model} { lappend clock_def_args -verilog_define SIM_MODEL }
    if {$enable_vio}    { lappend clock_def_args -verilog_define VIO_ENABLE }
    if {$enable_ila}    { lappend clock_def_args -verilog_define ILA_ENABLE }
    if {$perf_detail_enable} { lappend clock_def_args -verilog_define PERF_DETAIL_ENABLE }
    if {$enable_ipc_ila} { lappend clock_def_args -verilog_define IPC_ILA_ENABLE }
    if {$enable_scsi_trace} { lappend clock_def_args -verilog_define SCSI_TRACE_ENABLE }
    if {$enable_jtag_axi} { lappend clock_def_args -verilog_define JTAG_AXI_ENABLE }
    if {$enable_pcie_xdma} { lappend clock_def_args -verilog_define PCIE_XDMA_ENABLE }
    if {$eth_enable} { lappend clock_def_args -verilog_define ETH_ENABLE }
    if {$eth_debug_enable} { lappend clock_def_args -verilog_define ETH_DEBUG_ENABLE }
    lappend clock_def_args -verilog_define DEBUG_CTRL_NO_LOCAL_FALLBACK
    synth_design \
        -rtl \
        -top fpga_top \
        -part $part \
        -include_dirs $rtl_include_dirs \
        {*}$clock_def_args

    set clock_names [lsort -unique [get_clocks]]
    if {[llength $clock_names] == 0} {
        puts "INFO: no clocks resolved yet"
    } else {
        puts "=== CLOCK REPORT: resolved clocks ==="
        foreach clk $clock_names {
            puts "  $clk"
        }
    }

    report_clocks -file $output_dir/reports/clocks.rpt
    puts "=== CLOCK REPORT: wrote $output_dir/reports/clocks.rpt ==="
    puts "=== CLOCK REPORT: CORE_CLK_DIVIDE=$core_clk_divide CORE_CLK_HZ=$core_clk_hz ==="
    puts "=== CLOCK REPORT: TARGET_FREQ_MHZ is a timing budget only; it does not create a new board clock ==="
    exit 0
}

# ──────────────────────────────────────────────────────────────────────────────
# Synthesis
# ──────────────────────────────────────────────────────────────────────────────
puts "=== SYNTHESISING ==="
set synth_def_args [list]
if {$cpu_m68k040}   { lappend synth_def_args -verilog_define CPU_M68K040 }
if {$use_sim_model} { lappend synth_def_args -verilog_define SIM_MODEL }
# Macro-op fusion kill switch (env NO_MACRO_FUSION=1).  Fusion is a pure
# performance optimisation — a flag-producing TST.L/ADDQ.L/SUBQ.L collapsed
# with a following Bcc into one uop — so disabling it is architecturally
# invisible (973/973 directed + 200/200 fuzz both ways) at ~+2.6% cycles.
# Two reasons to be able to turn it off in a real bitstream:
#   1. A fused pair emits ONE uop carrying the FIRST instruction's PC, so a
#      PC breakpoint on the Bcc can never match — it makes branches in fused
#      pairs invisible to break-pc and to pc-trace.
#   2. It is the cheapest available test of whether fusion itself is behind
#      the 7.5.3 Finder guard misbranch.
# I-cache debug probe kill switch (env NO_ICACHE_PROBE=1).
# BISECTED 2026-08-18: the probe is the boot regression -- with it, cold boots
# succeed 4/15; without it (cpu 483e7d28) 15/15, and the archive 15/15.  It is a
# DEBUG-ONLY facility, so production bitstreams must build with it disabled
# until its defect is found.  Inspection of probe_launch (which includes !req)
# and the rd_set mux found nothing, so the fault is subtler than the read path.
if {[info exists ::env(NO_ICACHE_PROBE)] && $::env(NO_ICACHE_PROBE) ne "0"} {
    lappend synth_def_args -verilog_define CORE_NO_ICACHE_PROBE
    puts "=== I-CACHE DEBUG PROBE DISABLED (CORE_NO_ICACHE_PROBE) ==="
}
if {[info exists ::env(NO_MACRO_FUSION)] && $::env(NO_MACRO_FUSION) ne "0"} {
    lappend synth_def_args -verilog_define CORE_NO_MACRO_FUSION
    puts "=== MACRO-OP FUSION DISABLED (CORE_NO_MACRO_FUSION) ==="
}
if {$enable_vio}    { lappend synth_def_args -verilog_define VIO_ENABLE }
if {$enable_ila}    { lappend synth_def_args -verilog_define ILA_ENABLE }
if {$perf_detail_enable} { lappend synth_def_args -verilog_define PERF_DETAIL_ENABLE }
if {$enable_ipc_ila} { lappend synth_def_args -verilog_define IPC_ILA_ENABLE }
if {$enable_scsi_trace} { lappend synth_def_args -verilog_define SCSI_TRACE_ENABLE }
if {$enable_jtag_axi} { lappend synth_def_args -verilog_define JTAG_AXI_ENABLE }
if {$enable_pcie_xdma} { lappend synth_def_args -verilog_define PCIE_XDMA_ENABLE }
if {$eth_enable} { lappend synth_def_args -verilog_define ETH_ENABLE }
if {$eth_debug_enable} { lappend synth_def_args -verilog_define ETH_DEBUG_ENABLE }
if {$core_mmcm} { lappend synth_def_args -verilog_define CORE_MMCM }
# debug_ctrl.v's local cycle/inst-count fallback is unit-tb only; the
# integrated bitstream feeds real 64-bit external counters from
# fpga_top_debug_ctrl.vh.  Drop ~256 FFs + two 64-bit adders + two
# 64-bit zero-compare muxes by setting this define.
lappend synth_def_args -verilog_define DEBUG_CTRL_NO_LOCAL_FALLBACK
# PC trace ring is enabled in the bitstream at depth 64 (overridden in
# fpga_top_debug_ctrl.vh) for ~200-400 LUTs of distributed LUTRAM.  The
# 1024-entry default is reserved for unit-tb instances of debug_ctrl.
# Phase 2 cutover (2026-05-03): exception entry routes through LSU
# inject µops instead of the legacy dc_req shadow port (see
# docs/exception_uop_refactor.md and Makefile EXC_UOP_INJECT default).
# Both modes give identical 569/8 sweep counts — keep them in sync
# between sim and bitstream so HW debug matches sim debug.
lappend synth_def_args -verilog_define EXC_UOP_INJECT
# 2026-05-22 — Gate the JTAG SD-card writer out of production bitstreams.
# It burns ~17.5K LUTs (≈8% of KU5P) on the runtime path even when never
# used.  The writer is host-side provisioning only (one-shot CMD24 over
# JTAG); production boot needs only sd_ctrl_scsi.  Override with
# `ENABLE_SD_JTAG_WRITER=1` env to build a provisioning bitstream.
if {![info exists ::env(ENABLE_SD_JTAG_WRITER)]} {
    lappend synth_def_args -verilog_define DISABLE_SD_JTAG_WRITER
    puts "=== SD-JTAG writer gated (saves ~17.5K LUTs).  Set ENABLE_SD_JTAG_WRITER=1 to include for provisioning. ==="
}
# ──────────────────────────────────────────────────────────────────────────
# L2C_ENABLE (T13, docs/l2c_spec.md) — DO NOT UNCOMMENT YET.
#
# l2c (rtl/soc/l2c.v + submodules) is wired into fpga_top's DDR path
# behind `L2C_ENABLE` (see rtl/soc/fpga_top_ddr.vh) but is NOT part of
# any synth/bitstream build today.  l2c's 8-way 2 MB data array needs
# ~64 URAM288 blocks (docs/l2c_spec.md S3 URAM-budget note); the
# existing 2 MB Q700-silicon-matched VRAM array (rtl/mac/video.v) is
# ALSO URAM-backed and already claims ~57 URAM288 -- together they
# exceed the KU5P's 80 URAM288 budget (docs/tracks/platform.md).
# L2C_ENABLE must stay undefined here until the VRAM-in-DDR migration
# task lands and frees VRAM's URAM allocation.  When that lands, this
# needs BOTH the `-verilog_define L2C_ENABLE` below AND `read_verilog`
# lines for l2c*.v (currently absent from this file's file list --
# grep read_verilog $rtl_dir/soc/l2c* to confirm before uncommenting).
# Cutover gates (2026-07-24): both may now be enabled together — the
# VRAM-in-DDR migration (T14/T16) frees VRAM's URAM so l2c's data array
# fits the KU5P budget.  Enable via environment: L2C_ENABLE=1 VRAM_IN_DDR=1.
#
# CPU=m68k040 forces both ON regardless of env — NOT optional for this
# CPU (see the CPU socket select comment block above / task #269).  This
# is a real force, not just a default: it also overrides an explicit
# opt-out attempt (e.g. a caller passing L2C_ENABLE=0), because the
# Makefile's L2C_ENABLE=0 filter suppresses forwarding the env var
# entirely rather than forwarding a falsey value (see Makefile:2498-2523),
# so this script cannot tell "unset" from "explicitly disabled" — forcing
# it here is the only way to make the "not optional" guarantee hold
# whether this script is invoked via the Makefile or directly.  The RTL
# elaboration-time `error` backstop at rtl/soc/fpga_top_cpu.vh still
# catches any path that bypasses this script entirely.
# SECOND, LESS OBVIOUS CONSEQUENCE OF FORCING L2C_ENABLE FOR cpu040.  It is not
# only a datapath-width decision.  With L2C_ENABLE defined, axi_i (ifa_*) binds
# straight to l2c's dedicated fetch port (fpga_top_ddr.vh:273-279) and the
# crossbar's M2 slot is tied to a constant never-valid master
# (fpga_top_xbar.vh:371-378) -- and the crossbar holds the ONLY decode of the
# 0x5000_0000 peripheral window (axi_xbar.v:1249-1250).  So on a cpu040 build a
# speculative instruction fetch physically cannot reach a device register, and
# cpu040's I-side speculation guard is correspondingly built DISABLED
# (IcachePlugin's `iFetchCanReachMmio` defaults false).
#
#   *** IF THIS LINE IS EVER RELAXED so a cpu040 build can take the `else` branch
#   *** in fpga_top_xbar.vh -- putting instruction fetch back on the crossbar --
#   *** cpu040 MUST be rebuilt with iFetchCanReachMmio = true, or wrong-path
#   *** instruction fetches will issue 64-byte bursts into device space and fire
#   *** read-to-clear side effects the program never asked for.
#
# See cpu040 docs/superpowers/specs/2026-09-05-p138c-fetch-gate-deadlock-fix.md.
set l2c_enable_effective  [expr {$cpu_m68k040 || [info exists ::env(L2C_ENABLE)]}]
set vram_in_ddr_effective [expr {$cpu_m68k040 || [info exists ::env(VRAM_IN_DDR)]}]
if {$l2c_enable_effective}  { lappend synth_def_args -verilog_define L2C_ENABLE }
if {$vram_in_ddr_effective} { lappend synth_def_args -verilog_define VRAM_IN_DDR }
if {$cpu_m68k040} {
    puts "=== CPU=m68k040: forcing L2C_ENABLE + VRAM_IN_DDR on (axi_i is native 256b; no CPU-side downconverter) ==="
}

# ──────────────────────────────────────────────────────────────────────────
# CPU physical-register-file size knob (cpu/rtl/core/decode/uop_pkg.v)
#
# `make impl CPU=m68k PRF_INT=64` shrinks the OoO core's integer PRF from
# its 96-entry default, reclaiming FPGA area for new peripherals.  64 is
# the interesting stop: it is the largest size that fits a 6-bit physical
# tag, so every phys-tag field through the CPU's ROB, its three issue
# queues, the RAT and all four CDBs narrows by a bit as well.  Measured
# IPC is flat from 96 down to 64 on the CPU repo's bench set.
#
# The tag width is DERIVED here rather than passed in, so callers only
# ever set one number.  Getting the pair wrong is not a compile error in
# Verilog — it silently truncates phys tags — so rat.v / fp_rat.v carry a
# hard elaboration-time check that fails synthesis outright on a
# mismatch.  See the knob comment block in uop_pkg.v.
#
# Floors, as ACTUALLY enforced by that elaboration check (re-read from
# the RTL 2026-08-19 — the numbers here previously said "PRF_FP >= 9
# structurally, >= 16 recommended", which would have failed the build):
#   PRF_INT >= 24  (rat.v g_prf_int_floor; >= 64 recommended)
#   PRF_FP  >= 32  (fp_rat.v g_prf_fp_floor — 8 pinned FP0-7 plus the 24
#                   speculative destinations a full-width
#                   FMOVEM.X (mem),FP0-FP7 needs before the macro
#                   boundary can commit any of them)
# The check instantiates a module that does not exist, so the module name
# is the error message; grep `u_prf_size_check` to find all six sites.
proc prf_tag_width {n} {
    set w 1
    while {(1 << $w) < $n} { incr w }
    return $w
}
# CPU=m68k040 has no PHYS_INT_REGS/PHYS_FP_REGS build-time knob the way
# cpu/'s uop_pkg.v does — M68kSocketTop.v is a single pre-generated file
# with no Verilog parameters, so PRF_INT/PRF_FP are silent no-ops for it
# (mirrors the Makefile's CPU_PRF_DEFINES comment).  Not an error; just
# tell the caller their env var did nothing.
if {$cpu_m68k040 && ((([info exists ::env(PRF_INT)]) && $::env(PRF_INT) ne "") || \
                     (([info exists ::env(PRF_FP)])  && $::env(PRF_FP) ne ""))} {
    puts "=== CPU PRF: PRF_INT/PRF_FP are no-ops for CPU=m68k040 (no build-time PRF-size knob) ==="
}

# -no_lc was REMOVED here on 2026-08-01 (task #190).  LUT combining is now ON.
#
# It had been a DIAGNOSTIC AID, never a design requirement: with LUT combining
# off each LUT holds one function, so utilization reports attribute directly
# back to source and path delays are not confounded by packing.  That
# characterisation work is done, and the flag only cost area afterwards --
# LUT combining typically recovers 5-15% of logic LUTs, order 8,000-23,000
# sites against the 152,058 measured on b69a1a4.
#
# `git log -S` places the flag in the initial scaffolding commit c0cc7a2 with
# no rationale anywhere in docs/ or synth/; this comment is the only surviving
# record that it existed and why.  If a future build has to put it back, say
# so here with the measurement that forced it -- do not restore it silently.
#
# Its neighbour -keep_equivalent_registers is DIFFERENT and IS load-bearing for
# Fmax (docs/fmax_retime_log.md:46).  It stays.
# ⛔ -fanout_limit 400 WAS TRIED AND REVERTED (2026-09-14) -- it made timing WORSE.
#
# The 200 MHz build's worst core path was failing by 0.465 ns, and per-node fanout
# on that path showed TWO nets carrying most of the delay:
#
#     fanout 941  0.737 ns   DecodeStage_logic_queue/...
#     fanout 688  0.447 ns   FetchAlignPlugin_logic_ibuf/...
#
# 1.18 ns of a path that misses by 0.465. One LUT output reaching ~1000 loads across
# the die is a LOCALITY problem, and it is the same disease `9ca2a430` fixed by hand
# for a fanout-113 signal -- these are ~8x worse. The broadcasts are the instruction
# buffer's flush/shift and the decode queue's equivalent: inherently every-entry
# controls, so the answer is to let the tool DUPLICATE the driver per region rather
# than to restructure them away.
#
# MEASURED RESULT, same netlist, same flow:
#
#            WNS      TNS      failing setup EPs
#   without  -0.465   -2254    11846
#   with     -1.209  -12363    30105     <- 2.5x WORSE
#
# So replicating those drivers cost MORE in congestion than it saved in fanout
# delay. The reasoning (1.18 ns of a 0.465 ns miss sits on two nets of fanout 941
# and 688) was sound; the remedy was not. Do not retry the GLOBAL threshold.
# If this is revisited, target ONLY those two nets -- e.g. a MAX_FANOUT attribute
# on the specific RTL signals (the instruction buffer's flush/shift and the decode
# queue's equivalent) -- so the rest of the design is not replicated along with them.
#
# ⚠️ An earlier attempt at `phys_opt_design -force_replication_on_nets` did NOTHING
# ("no rep_nets matched") because it needed net names, which are synthesis-generated
# and unstable. A threshold needs no names.
synth_design \
    -top fpga_top \
    -part $part \
    -include_dirs $rtl_include_dirs \
    -directive PerformanceOptimized \
    -keep_equivalent_registers \
    -flatten_hierarchy rebuilt \
    {*}$synth_def_args

# Taxi's constraint Tcl files operate on elaborated cells, so source them
# only after synthesis has created the hierarchy they query.  They are kept
# verbatim from the pinned physical-link baseline.
if {$eth_enable} {
    foreach tcl_file [list \
            $eth_rk5_dir/third_party/taxi/src/eth/syn/vivado/taxi_rgmii_phy_if.tcl \
            $eth_rk5_dir/third_party/taxi/src/eth/syn/vivado/taxi_eth_mac_fifo.tcl \
            $eth_rk5_dir/third_party/taxi/src/axis/syn/vivado/taxi_axis_async_fifo.tcl \
            $eth_rk5_dir/third_party/taxi/src/sync/syn/vivado/taxi_sync_reset.tcl \
            $eth_rk5_dir/third_party/taxi/src/sync/syn/vivado/taxi_sync_signal.tcl \
            $synth_dir/ethernet_rgmii_post.tcl] {
        if {![file exists $tcl_file]} { error "Taxi constraint file not found: $tcl_file" }
        source $tcl_file
    }
}
if {$eth_debug_enable} {
    source $synth_dir/eth_debug_cdc.tcl
}

# ──────────────────────────────────────────────────────────────────────────────
# Post-synth fanout hint pass — belt-and-suspenders for the high-fanout
# broadcast FFs that the RTL `(* MAX_FANOUT = 256 *)` attribute targets.
# Some Vivado flows do not honour the Verilog attribute uniformly across
# `synth_design -directive PerformanceOptimized + -keep_equivalent_registers`
# (the latter, in particular, can suppress automatic replication on
# attributed FFs that look "equivalent" to other FFs).  Re-asserting
# MAX_FANOUT as a TCL property after synth ensures the placer still has
# the directive in hand, regardless of synth flow quirks.
#
# IMPORTANT: the reset broadcasts (`soc_full_rst*`, `core_rst*`,
# `pb_full_rst*`, `via1_rst_pb*`) used to live in this list; they have
# been moved to the xpm_cdc_async_rst + explicit BUFG distribution
# pattern (UG974) in clk_rst.v / fpga_top_peripherals.vh.  An explicit
# BUFG drives the broadcast directly; replicating the BUFG output via
# MAX_FANOUT would defeat the global-clock-network distribution we
# specifically wanted, so those patterns are intentionally NOT here.
#
# Remaining targets: per-clock event broadcasts that are NOT resets and
# still need user-FF replication.  See agent/fanout-vivado-managed for
# the original analysis and agent/xpm-reset-distribution for the reset-
# specific drop.
#
# ──────────────────────────────────────────────────────────────────────────────
set fanout_offenders [get_cells -quiet -hier -filter \
    {NAME =~ "*dbg_overlay_q*" || \
     NAME =~ "*crat_rb*" || \
     NAME =~ "*video*rst*"}]
if {[llength $fanout_offenders] > 0} {
    puts "=== Setting MAX_FANOUT=256 on [llength $fanout_offenders] high-fanout broadcast FFs ==="
    set_property MAX_FANOUT 256 $fanout_offenders
}

# `vif_rdata` belt-and-suspenders pass added 2026-07-12 (route_design
# congestion investigation).  if_mmu_xlate.v's 128-bit registered
# icache-line output (u_cpu/u_cpu/u_if_xlate/vif_rdata_reg) — read
# broadly across the whole predecode/decode front end every cycle —
# now carries its own `(* MAX_FANOUT = 64 *)` RTL attribute (see that
# file's port comment).  Kept in a SEPARATE block from the
# dbg_overlay_q/crat_rb/video_rst set above (which uses 256) rather
# than folding it into that filter, so this re-assertion uses the SAME
# value (64) as the RTL attribute — matching, not overriding, it —
# rather than silently loosening it to 256 the way appending it to the
# block above would have.  Empirically confirmed (via
# report_route_status / route_design verification-failure overlap
# listings) to be a reproducible residual-congestion contributor across
# BOTH the baseline (ExtraPostPlacementOpt) and AltSpreadLogic_high
# place_design runs, with its share of the (shrinking) overlap set
# growing as AltSpreadLogic_high cleared generic PRF/CDB-crossbar
# congestion elsewhere (2/10 top overlaps -> 3/10 across two full
# route_design attempts) — the signature of a genuinely under-
# replicated wide bus, not incidental noise.  Unrelated to any RTL
# change in the current fix batch (icache.v's diff never touches the
# rdata/hit_data path this signal is sourced from).
set vif_rdata_cells [get_cells -quiet -hier -filter {NAME =~ "*vif_rdata*"}]
if {[llength $vif_rdata_cells] > 0} {
    puts "=== Setting MAX_FANOUT=64 on [llength $vif_rdata_cells] vif_rdata cells (matches RTL attribute) ==="
    set_property MAX_FANOUT 64 $vif_rdata_cells
}

proc stitch_real_mig_dcp {mig_dcp} {
    set mig_cells [get_cells -quiet -hier -filter {REF_NAME == design_1_ddr4_0_1}]
    if {[llength $mig_cells] != 1} {
        puts stderr "ERROR: expected exactly one design_1_ddr4_0_1 black-box cell, found [llength $mig_cells]: $mig_cells"
        exit 1
    }
    set mig_cell [lindex $mig_cells 0]
    puts "=== STITCHING REAL DDR4 MIG DCP ==="
    puts "MIG cell: $mig_cell"
    puts "MIG DCP:  $mig_dcp"
    read_checkpoint -cell $mig_cell $mig_dcp
}

proc force_real_mig_impl_contract {} {
    puts "=== REAL MIG IMPL CONTRACT FIXUPS ==="

    # Vivado's MIG/Advanced IO DRC wants the memory IP pins to collapse cleanly
    # to top-level ports.  HDL attributes and MARK_DEBUG can freeze the ddr_ctrl
    # hierarchy before opt_design gets a chance to do that, so explicitly loosen
    # those cells for the real-MIG flow after the DCP is stitched.
    foreach pattern {u_mig_ddr4 u_ddr u_ddr/u_mig_ddr4} {
        foreach cell [get_cells -quiet -hier $pattern] {
            catch {set_property DONT_TOUCH false $cell}
            catch {set_property KEEP_HIERARCHY false $cell}
        }
    }

    # The pcie_test-generated MIG implementation XDC applies these standards.
    # When the OOC DCP is stitched into our non-project flow, Vivado can otherwise
    # arrive at opt_design with bank defaults such as LVCMOS18/DIFF_HSTL_I_18,
    # which the MIG DRC correctly rejects.
    set sstl_ports [get_ports -quiet {ddr4_act_n ddr4_adr[*] ddr4_ba[*] ddr4_bg[*] ddr4_cke[*] ddr4_odt[*] ddr4_cs_n[*]}]
    if {[llength $sstl_ports] > 0} {
        set_property IOSTANDARD SSTL12_DCI $sstl_ports
    }

    set pod_ports [get_ports -quiet {ddr4_dq[*] ddr4_dm_dbi_n[*]}]
    if {[llength $pod_ports] > 0} {
        set_property IOSTANDARD POD12_DCI $pod_ports
    }

    set dqs_ports [get_ports -quiet {ddr4_dqs_t[*] ddr4_dqs_c[*]}]
    if {[llength $dqs_ports] > 0} {
        set_property IOSTANDARD DIFF_POD12_DCI $dqs_ports
    }

    set ck_ports [get_ports -quiet {ddr4_ck_t[*] ddr4_ck_c[*]}]
    if {[llength $ck_ports] > 0} {
        set_property IOSTANDARD DIFF_SSTL12_DCI $ck_ports
    }

    set reset_ports [get_ports -quiet ddr4_reset_n]
    if {[llength $reset_ports] > 0} {
        set_property IOSTANDARD LVCMOS12 $reset_ports
        set_property DRIVE 8 $reset_ports
    }
}

if {!$use_sim_model} {
    stitch_real_mig_dcp $ddr4_mig_dcp
    force_real_mig_impl_contract
}

# Relax core-clock timing to TARGET_FREQ_MHZ via set_max_delay.  This keeps
# the board oscillator constrained at 200 MHz while telling STA to accept
# paths up to (target_period) on the active core clock (core_clk when it is
# resolved, otherwise sysclk200 in the direct-pass case).  At 200 MHz this is
# a no-op.
#
# ── Part 132 (2026-09-04): `-datapath_only` REMOVED, and the block is now
#    skipped when it would buy nothing. ────────────────────────────────────
#
# `set_max_delay -datapath_only` is a CROSS-DOMAIN construct.  Applied FROM a
# clock TO ITSELF it does two things nobody intended:
#
#   (1) it SUPPRESSES THE MIN (HOLD) REQUIREMENT on every path it covers, and
#   (2) it drops clock skew and clock uncertainty from the setup check.
#
# At CORE_CLK_DIVIDE=1 -- the production 100 MHz shape -- `u_fabric_core_bufg_gt`
# has DIV=0, so Vivado creates no new clock object and `fabric_clk100` IS the
# core clock, carrying the entire CPU.  The block therefore aimed this exception
# at the whole processor.  Measured, verbatim, in the routed p123-reseed design
# (build/vivado/timing_summary.rpt), a bitstream that wedges on silicon:
#
#     From Clock:  fabric_clk100
#       To Clock:  fabric_clk100
#     Setup :  0  Failing Endpoints,  Worst Slack  0.912ns,  Total Violation 0.000ns
#     Hold  : NA  Failing Endpoints,  Worst Slack     NA  ,  Total Violation    NA
#     ...
#     Timing Exception: MaxDelay Path 10.000ns -datapath_only
#
# `Hold: NA` over 281 916 endpoints.  Vivado analysed -- and therefore FIXED --
# no hold at all inside the CPU, while every other domain in the same design
# sits at WHS +0.012..+0.025 ns, the signature of a router that had to work to
# close hold.  Hold failures are FREQUENCY-INDEPENDENT, which is exactly the
# otherwise-unexplained shape of the 0x4084BECE wedge (identical at 100, 25 and
# 12.5 MHz) and of its netlist-to-netlist lottery.
#
# It was also buying nothing: `create_clock -period 10.000 fabric_clk100` and
# TARGET_FREQ_MHZ=100 give target_period_ns == 10.0 == the clock period, so the
# "relaxation" was exactly zero while costing all hold analysis.
#
# The comment this replaces argued the block was "a no-op at CORE_CLK_DIVIDE != 1
# (it lands on the endpoint-less `fabric_clk100`)".  True at DIVIDE != 1, and
# irrelevant: the DEPLOYED builds are DIVIDE=1, where it lands on the real core
# clock with every endpoint in the design behind it.
#
# Kept: the ability to genuinely relax a slower-than-nominal core clock.  Now it
# only fires when the requested budget is actually LOOSER than the clock's own
# period, and it uses a plain `set_max_delay` so that hold analysis and clock
# skew/uncertainty accounting both survive.
if {$target_freq_mhz != 200.0} {
    set core_budget_clks [get_clocks -quiet core_clk]
    if {[llength $core_budget_clks] == 0} {
        if {$use_sim_model} {
            set core_budget_clks [get_clocks -quiet sysclk200]
        } else {
            set core_budget_clks [get_clocks -quiet fabric_clk100]
        }
    }
    # The tightest period among the resolved clocks is what a max_delay would
    # have to beat to be a relaxation at all.
    set core_period_ns 0.0
    foreach c $core_budget_clks {
        set p [get_property -quiet PERIOD $c]
        if {$p ne "" && ($core_period_ns == 0.0 || $p < $core_period_ns)} {
            set core_period_ns $p
        }
    }
    if {$core_period_ns > 0.0 && $target_period_ns <= [expr {$core_period_ns + 0.001}]} {
        puts "=== SKIPPING core-clock set_max_delay: budget ${target_period_ns}ns is not looser"
        puts "    than the clock's own period ${core_period_ns}ns on ($core_budget_clks)."
        puts "    Applying it would relax nothing and would disable hold analysis. ==="
    } else {
        puts "=== Applying set_max_delay $target_period_ns ns on $core_budget_clks → $core_budget_clks ==="
        puts "    (NO -datapath_only: this is a SAME-CLOCK relaxation, so the hold"
        puts "     requirement and clock skew/uncertainty must both stay live.)"
        set_max_delay -from $core_budget_clks -to $core_budget_clks $target_period_ns
    }
    puts "NOTE: TARGET_FREQ_MHZ does not generate a new clock; it only relaxes timing on the active core clock."
}

# ── Fabric clock resolution (pin-based, name-independent) ─────────────
#
# BUG FIXED HERE (documented but deliberately left unfixed in
# docs/BUG_calibration_word_misplaced_0d00.md Part 17; see
# docs/timing_constraint_audit_2026-09-03.md for the full write-up):
#
# The previous implementation guessed the core clock by NAME, trying
# `core_clk`, then `fabric_clk100`, then `sysclk200` -- and never
# `sys_clk`, which is what Vivado auto-derives for the real post-BUFG_GT
# core clock whenever CORE_CLK_DIVIDE != 1.  Neither `core_clk` nor
# `sys_clk` is ever created by an explicit `create_clock`: the only
# explicit ones are `sysclk200` (on port sys_clk_p) and `fabric_clk100`
# (on port fabric_clk_p), everything else is auto-derived.
#
# Why the name lookup accidentally worked at CORE_CLK_DIVIDE=1 and
# silently broke everywhere else: `u_fabric_core_bufg_gt` (see
# rtl/soc/fpga_top_clocks.vh) is instantiated with
# DIV = FABRIC_GT_CORE_DIV, which is 3'd0 at CORE_CLK_DIVIDE=1.  A
# BUFG_GT with DIV=0 does not divide, so Vivado creates NO new generated
# clock and simply propagates `fabric_clk100` through it -- the name
# fallback lands on the right object by luck.  At DIVIDE 2/4/8 the
# BUFG_GT does divide, Vivado auto-derives a generated clock (named
# `sys_clk` after the driven net), `fabric_clk100` is left with ZERO
# endpoints, and every constraint built on top of this lookup was
# applied to an empty clock.
#
# Measured consequence (build archived as
# synth/timing_reports/timing_20260903_160349.rpt, a CORE_CLK_DIVIDE=8
# build): `sys_clk` carries 271892 endpoints while `fabric_clk100`
# carries none, the fabric<->MIG `set_clock_groups -asynchronous` was
# therefore built against an empty clock, and the routed design reports
# WNS -4.163 ns / 8146 failing endpoints with mmcm_clkout0 at
# -2.620 ns / 1451 endpoints.  Compare the CORE_CLK_DIVIDE=1 build of
# the same design (build/vivado_divfix_100mhz): WNS +0.031 ns, zero
# failing endpoints.
#
# Fix: resolve the clock from the netlist PIN that physically generates
# it, which is correct at every divide value and immune to Vivado's
# auto-naming.  This is the same technique `apply_video_cdc_constraints`
# already used for the pixel clock (`get_pins u_video/u_mmcm/u_bufg_pclk/O`).
# Name lookups are kept only as a fallback for build shapes that have no
# fpga_top_clocks.vh at all (CPU-only OOC runs, SIM_MODEL), and `sys_clk`
# is now among them.
proc _clocks_on_pin {pin_pattern} {
    set pins [get_pins -quiet -hierarchical -filter "NAME =~ \"$pin_pattern\""]
    if {[llength $pins] == 0} {
        return {}
    }
    return [get_clocks -quiet -of_objects $pins]
}

# ── Part 132 guard: the core clock must never lose its hold analysis. ───────
# Any `-datapath_only` exception with the core clock on BOTH ends silently turns
# off min-delay analysis for the whole processor.  That went unnoticed across
# every bitstream this repo has shipped; refuse to build that way again.
proc p132_assert_core_hold_analysed {clks} {
    if {[llength $clks] == 0} { return }
    # Property-name-independent screen first: `report_exceptions` prints one row
    # per exception with its type and its -from/-to, so a datapath_only row that
    # names the core clock on both sides is caught even if the object property
    # names differ across Vivado releases.
    set rep ""
    catch {set rep [report_exceptions -no_header -return_string]}
    foreach c $clks {
        set nm [get_property -quiet NAME $c]
        if {$nm eq ""} { continue }
        foreach line [split $rep "\n"] {
            if {![string match -nocase "*datapath_only*" $line]} { continue }
            # both ends must name this clock for it to be an intra-domain kill
            set hits [regexp -all -- "\\y$nm\\y" $line]
            if {$hits >= 2} {
                puts "ERROR: \[P132\] report_exceptions row suppresses hold on $nm -> $nm:"
                puts "       $line"
                error "P132: core clock hold analysis is disabled by a -datapath_only exception"
            }
        }
    }
    # P133: `get_timing_exceptions` does NOT exist in Vivado 2025.2 (verified by
    # scanning every shipped library); calling it raises "invalid command name"
    # and would abort the build.  The report_exceptions screen above is the real
    # check; this object-based pass is belt-and-braces, so make it non-fatal.
    set _p133_excs {}
    catch {set _p133_excs [get_timing_exceptions -quiet]}
    foreach e $_p133_excs {
        if {[get_property -quiet DATAPATH_ONLY $e] ne "1"} { continue }
        set f [get_property -quiet FROM $e]
        set t [get_property -quiet TO   $e]
        foreach c $clks {
            set nm [get_property -quiet NAME $c]
            if {$nm eq ""} { continue }
            if {[string match "*$nm*" $f] && [string match "*$nm*" $t]} {
                puts "ERROR: \[P132\] a -datapath_only max_delay covers $nm -> $nm."
                puts "       That suppresses the HOLD requirement across the entire CPU"
                puts "       clock domain (Vivado reports 'Hold: NA'), so the router will"
                puts "       do no hold fixing there.  See Part 132.  Refusing to build."
                error "P132: core clock hold analysis is disabled by a -datapath_only exception"
            }
        }
    }
    puts "=== \[P132\] core-clock hold analysis is live on ($clks) ==="
}

proc get_active_core_clock {} {
    # Authoritative: whichever buffer actually drives `sys_clk` (core).
    # CORE_MMCM builds replace the BUFG_GT divider with an MMCM + BUFG, so
    # try that pin FIRST -- on a CORE_MMCM netlist the BUFG_GT instance
    # does not exist at all, and on a normal netlist the MMCM one does
    # not, so the order is only about which lookup is cheaper.  Resolving
    # by pin (not by name) is the whole point: see the comment on
    # get_active_pb_clock below for what a name lookup silently did.
    set core_clk [_clocks_on_pin "*u_core_mmcm_core_bufg/O"]
    if {[llength $core_clk] > 0} {
        return $core_clk
    }
    set core_clk [_clocks_on_pin "*u_fabric_core_bufg_gt/O"]
    if {[llength $core_clk] > 0} {
        return $core_clk
    }
    # Fallback for netlists without the fpga_top clock tree (OOC / SIM_MODEL).
    # Order matters: `sys_clk` before `fabric_clk100` so a divided core clock
    # wins over its own (endpoint-less) parent.
    foreach nm {core_clk sys_clk fabric_clk100 sysclk200} {
        set core_clk [get_clocks -quiet $nm]
        if {[llength $core_clk] > 0} {
            return $core_clk
        }
    }
    return {}
}

# Peripheral-bus clock (fixed 50 MHz, its own BUFG_GT with DIV=1, see
# `u_fabric_pb_bufg_gt` in rtl/soc/fpga_top_clocks.vh).
#
# Second instance of the same class of bug: every call site below looked
# this clock up as the literal name `pb_clk_src`, and the auto-derived
# name is NOT stable across builds.  Observed directly in this repo's own
# build logs -- synth/slowclock_diag_impl.out resolved `pb_clk_src` (the
# "VIDEO CDC FALSE_PATH" line printed, and the MIG group reads
# "(fabric_clk100 pb_clk_src)"), while synth/slowclock8_impl.out did NOT
# (no false-path line, MIG group reads "(fabric_clk100)" only).  In the
# current production netlist the clock is named `pb_clk`
# (build/vivado_divfix_100mhz/reports/timing_place.rpt Clock Summary), so
# `get_clocks pb_clk_src` returns empty and every pb_clk constraint below
# has been silently doing nothing.  Resolve by pin instead.
proc get_active_pb_clock {} {
    # CORE_MMCM builds take pb off the same MMCM (CLKOUT1, /24) instead of
    # the BUFG_GT divider -- same pin-not-name rule as the core clock.
    set pb [_clocks_on_pin "*u_core_mmcm_pb_bufg/O"]
    if {[llength $pb] > 0} {
        return $pb
    }
    set pb [_clocks_on_pin "*u_fabric_pb_bufg_gt/O"]
    if {[llength $pb] > 0} {
        return $pb
    }
    foreach nm {pb_clk_src pb_clk} {
        set pb [get_clocks -quiet $nm]
        if {[llength $pb] > 0} {
            return $pb
        }
    }
    return {}
}

# Whole clock family rooted at $clks, i.e. the clocks themselves plus every
# generated clock derived from them.  `set_clock_groups` does NOT walk
# generated clocks implicitly (UG903): naming only `fabric_clk100` leaves
# its children -- `sys_clk` (core, at DIVIDE != 1), `pb_clk`, `pclk_unbuf`,
# `al9134_clk_fwd` -- outside the group and still timed against the other
# group.  This is the second half of the same bug: even with the right root
# name, the exclusion was too narrow.
proc _clock_family {clks} {
    if {[llength $clks] == 0} {
        return {}
    }
    set fam [get_clocks -quiet -include_generated_clocks $clks]
    if {[llength $fam] == 0} {
        return $clks
    }
    return $fam
}

# ── Reset-broadcast STA exception (clk_rst.v BUFG-buffered nets) ───────
# clk_rst.v's xpm_cdc_async_rst + explicit BUFG methodology (see that
# file's "Fanout / placement notes" comment, post-vivado_main_d1c9f11
# autopsy) makes Vivado ROUTE core_rst_buf/soc_full_rst_buf on the global
# clock network (low, balanced skew) instead of generic fabric routing --
# but that is a PLACEMENT/ROUTING treatment only.  Nothing in this script
# declared an STA exception on paths that merely traverse the buffered
# net, so Vivado still setup/hold-checks every FF-to-FF path reachable
# through it against the full clock period, even though the signal only
# transitions once per reset event and is architecturally guaranteed to
# settle across many cycles (the whole point of the xpm_cdc_async_rst
# DEST_SYNC_FF=4 synchroniser depth) before any consumer begins real
# operation.  At synth-only (pre-place) stage this net reports as a huge
# `(unplaced)` net dominating several reported paths' delay purely
# because the wireload model can't see the real BUFG global-network skew
# it will get once placed (task #274 investigation, 2026-08-20, 18
# fabric_clk100 endpoints all traced to this one root cause) -- not a
# real critical path.  False-path it explicitly (through, not from/to,
# so it strips the check regardless of which clock domain reaches it on
# either side) so synth-only and post-route reporting agree and this
# noise doesn't keep recurring or masking genuine WNS on later builds.
set rst_bufg_pins [get_pins -quiet -hier -filter \
    {NAME =~ "*u_core_rst_bufg/O" || NAME =~ "*u_soc_full_rst_bufg/O"}]
if {[llength $rst_bufg_pins] > 0} {
    set rst_bufg_nets [get_nets -quiet -of_objects $rst_bufg_pins]
    puts "=== Reset-broadcast false-path: [llength $rst_bufg_nets] BUFG-buffered reset net(s) (-through) ==="
    set_false_path -through $rst_bufg_nets
} else {
    puts "INFO: no clk_rst.v BUFG reset-broadcast pins found; skipping reset false-path (expected for CPU-only OOC builds without fpga_top_clocks.vh)."
}

# ── Async-FIFO Gray-pointer bus-skew bound (T6) ─────────────────────────
# set_clock_groups -asynchronous (used below in each of the video / MIG /
# PCIe CDC procs) correctly tells STA not to time these crossings on a
# launch/capture clock-edge relationship, but the multi-bit Gray-coded
# pointer crossing (rptr_gray -> rptr_gray_w1_r, wptr_gray ->
# wptr_gray_r1_r in rtl/board/async_fifo.v) still needs its "at most one
# bit changes per update" guarantee protected: the different bits of the
# Gray word must all arrive at the destination register within
# min(source period, destination period) of each other, or the
# destination can sample a torn mix of old/new bits that was never a
# real pointer value.  The bound is the MIN of the two periods, not the
# destination period alone -- see the min() rationale comment at the
# call site below (matches Xilinx's own `xpm_cdc_gray.tcl` reference,
# which uses `[expr min($src_clk_period, $dest_clk_period)]`).
#
# An earlier version of this proc used `set_max_delay -to <reg> <period>
# -datapath_only` and self-reported success while actually applying
# nothing, for three compounding reasons (all confirmed against Vivado
# 2025.2 documentation during review):
#   (a) `set_max_delay -datapath_only` REQUIRES `-from` — a `-to`-only
#       invocation errors.
#   (b) that call also had `-quiet`, which makes the command return
#       TCL_OK regardless of the error, so the `catch` around it never
#       tripped and the WARNING branch was dead code.
#   (c) even with `-from` added, per UG903's exception-priority rules
#       `set_max_delay` is IGNORED between two clocks already declared
#       `-asynchronous` via `set_clock_groups` — which is exactly what
#       every call site below does immediately before invoking this
#       proc.
# `set_bus_skew` (UG906) is the correct instrument here: it is checked
# INDEPENDENTLY of the `-asynchronous` clock-group exclusion, and it is
# specifically meant to bound the SPREAD between multiple related bits
# of a bus arriving at their destination registers — i.e. exactly the
# Gray-pointer inter-bit-skew property this module's correctness
# depends on, not just a single-path max-delay.  No `-quiet` on the
# constraint command itself, so a real Tcl error from a bad invocation
# propagates to the `catch` around it instead of being silently
# swallowed.
#
# FRAGILITY NOTE: source/destination register pairs are matched purely
# by hierarchical NAME PATTERN across the WHOLE design (source leaf
# "wptr_gray_reg"/"rptr_gray_reg", destination leaf
# "wptr_gray_r1_r_reg"/"rptr_gray_w1_r_reg"), then re-grouped into
# per-instance buses by stripping the known leaf suffix from each
# matched cell's hierarchical path and pairing same-prefix groups — NOT
# by walking each async_fifo instance's sub-hierarchy explicitly via
# REF_NAME.  This is name-collision-fragile: any OTHER module that
# happens to declare registers literally named `.../wptr_gray_reg[n]` or
# `.../wptr_gray_r1_r_reg[n]` (or the rptr_gray equivalents) would be
# swept into this sweep and bus-skew-bounded as if it were an
# async_fifo instance — harmless if such a register doesn't actually
# exist, but silently wrong-scoped if it did.  Acceptable today because
# this naming convention is unique to rtl/board/async_fifo.v in this
# codebase; revisit (scope by walking REF_NAME == async_fifo hierarchy
# instead of name-pattern matching) if a second Gray-coded CDC primitive
# with similarly-named internal registers is ever added.
set async_fifo_cdc_bound_applied 0

# Group a flat cell collection by hierarchical instance prefix: for each
# cell whose name is "<prefix>/<leaf_suffix>[<n>]" (bit index optional),
# returns an `array get`-style flat list mapping <prefix> -> {cells}.
proc _t6_group_cells_by_instance_prefix {cells leaf_suffix} {
    array set groups {}
    # Plain `foreach`, not `foreach_in_collection`: by the time a Vivado
    # get_cells collection is passed as a procedure argument, it no
    # longer survives as a native collection object that
    # foreach_in_collection recognizes ("invalid command name" at
    # runtime) -- but it still behaves as a normal Tcl list, which plain
    # foreach iterates over correctly.
    foreach c $cells {
        set nm [get_property NAME $c]
        regsub {\[[0-9]+\]$} $nm {} nm_nobit
        set marker "/${leaf_suffix}"
        set idx [string last $marker $nm_nobit]
        if {$idx < 0} {
            continue
        }
        set prefix [string range $nm_nobit 0 [expr {$idx - 1}]]
        lappend groups($prefix) $c
    }
    return [array get groups]
}

# Resolve the clock driving a flop, given only that flop's hierarchical
# NAME as a bare Tcl string.
#
# BUG FIXED HERE.  The previous code did:
#     set src_anchor [lindex $src_bus 0]
#     set src_clk_pin [get_pins -quiet -of_objects $src_anchor -filter {IS_CLOCK}]
# `$src_bus` is a Vivado collection that has already been shimmered to a
# plain Tcl list (see the comment in _t6_group_cells_by_instance_prefix),
# so `lindex` yields a NAME STRING, not a cell object -- and
# `get_pins -of_objects <string>` glob-matches that string.  Every one of
# these names ends in a bit index, e.g.
#     u_dbg_pb_to_core/u_bridge/ar_fifo/wptr_gray_reg[0]
# and in a glob pattern `[0]` is a CHARACTER CLASS, so the pattern only
# matches a cell literally named `...wptr_gray_reg0`, which does not
# exist.  `-quiet` then swallowed the miss and the code took its
# "source clock unresolved" skip path for every instance.
#
# Result: the async-FIFO Gray-pointer `set_bus_skew` bound this file
# spends ~60 lines of comment justifying was applied to ZERO instances in
# EVERY build ever produced.  Verbatim from synth/slowclock8_impl.out and
# synth/slowclock_diag_impl.out, and reproduced directly against
# build/vivado_divfix_100mhz/checkpoints/synth.dcp on 2026-09-03:
#     === ASYNC_FIFO GRAY-POINTER BUS_SKEW BOUND (T6): 0 instance(s)/
#         direction(s) actually constrained, 30 attempted-but-skipped ===
# (30 = 15 async_fifo instances x 2 crossing directions.)
#
# Re-resolving the name to a real cell object first makes `-of_objects`
# behave; naming the clock pin explicitly is kept as a second route.
# Both were verified to return `pb_clk` for the anchor above on the
# synth checkpoint where the original returned nothing.
proc _t6_clock_of_cell_name {cell_name} {
    set cellobj [get_cells -quiet $cell_name]
    if {[llength $cellobj] > 0} {
        set p [get_pins -quiet -of_objects $cellobj -filter {IS_CLOCK}]
        if {[llength $p] > 0} {
            set clk [get_clocks -quiet -of_objects $p]
            if {[llength $clk] > 0} {
                return $clk
            }
        }
    }
    set p [get_pins -quiet "${cell_name}/C"]
    if {[llength $p] > 0} {
        return [get_clocks -quiet -of_objects $p]
    }
    return {}
}

# Apply set_bus_skew for one Gray-pointer crossing direction (source
# leaf name -> first-stage destination leaf name), per matched instance
# prefix.  Returns {n_applied n_skipped}.
proc _t6_apply_bus_skew_for_direction {src_leaf dst_leaf} {
    set src_cells [get_cells -quiet -hierarchical -filter \
        "NAME =~ \"*/${src_leaf}*\""]
    set dst_cells [get_cells -quiet -hierarchical -filter \
        "NAME =~ \"*/${dst_leaf}*\""]

    if {[llength $src_cells] == 0 || [llength $dst_cells] == 0} {
        puts "WARNING: T6 gray-pointer bus_skew: no cells matched for ${src_leaf} -> ${dst_leaf} (source found: [llength $src_cells], dest found: [llength $dst_cells]) -- skipping.  Expected only if no async_fifo instances survived synthesis for this build."
        return [list 0 0]
    }

    array set src_groups [_t6_group_cells_by_instance_prefix $src_cells $src_leaf]
    array set dst_groups [_t6_group_cells_by_instance_prefix $dst_cells $dst_leaf]

    set n_applied 0
    set n_skipped 0
    foreach prefix [array names dst_groups] {
        if {![info exists src_groups($prefix)]} {
            puts "WARNING: T6 gray-pointer bus_skew: no source-side match for instance prefix ${prefix} (looked for ${src_leaf}) -- skipping this instance."
            incr n_skipped
            continue
        }
        set dst_bus $dst_groups($prefix)
        set src_bus $src_groups($prefix)

        # Bound = min(source period, destination period), NOT
        # destination period alone -- matches Xilinx's own reference
        # (xpm_cdc_gray.tcl: `[expr min($src_clk_period,
        # $dest_clk_period)]`).  Rationale: bounding to <= one SOURCE
        # period guarantees only ADJACENT Gray updates can ever land
        # inside the skew window and get mixed -- a 1-bit difference is
        # still a real (if one-update-stale) pointer value.  Bounding to
        # the destination period alone is only correct when dest is the
        # slower clock; in the fast-source -> slow-destination direction
        # (e.g. ~333 MHz MIG UI -> a slower fabric clock) a dest-only
        # bound could let several source-clock periods' worth of updates
        # (and therefore several DIFFERENT multi-bit-apart Gray values)
        # land inside one destination period, producing a torn
        # combination that was never any single valid pointer state.
        set src_anchor [lindex $src_bus 0]
        set src_clk [_t6_clock_of_cell_name $src_anchor]
        if {[llength $src_clk] == 0} {
            puts "WARNING: T6 gray-pointer bus_skew: source clock unresolved for instance prefix ${prefix} -- skipping this instance."
            incr n_skipped
            continue
        }
        if {[catch {set src_period_ns [get_property PERIOD [lindex $src_clk 0]]} err]} {
            puts "WARNING: T6 gray-pointer bus_skew: source PERIOD lookup failed for instance prefix ${prefix}: $err -- skipping this instance."
            incr n_skipped
            continue
        }
        if {$src_period_ns eq "" || $src_period_ns <= 0} {
            puts "WARNING: T6 gray-pointer bus_skew: invalid/zero source PERIOD for instance prefix ${prefix} -- skipping this instance."
            incr n_skipped
            continue
        }

        # Destination-clock period: resolve from any one bit of the
        # destination bus (every bit of one first-stage sync register
        # shares the same destination clock).
        set dst_anchor [lindex $dst_bus 0]
        set dst_clk [_t6_clock_of_cell_name $dst_anchor]
        if {[llength $dst_clk] == 0} {
            puts "WARNING: T6 gray-pointer bus_skew: destination clock unresolved for instance prefix ${prefix} -- skipping this instance."
            incr n_skipped
            continue
        }
        if {[catch {set dst_period_ns [get_property PERIOD [lindex $dst_clk 0]]} err]} {
            puts "WARNING: T6 gray-pointer bus_skew: destination PERIOD lookup failed for instance prefix ${prefix}: $err -- skipping this instance."
            incr n_skipped
            continue
        }
        if {$dst_period_ns eq "" || $dst_period_ns <= 0} {
            puts "WARNING: T6 gray-pointer bus_skew: invalid/zero destination PERIOD for instance prefix ${prefix} -- skipping this instance."
            incr n_skipped
            continue
        }

        set period_ns [expr {($src_period_ns < $dst_period_ns) ? $src_period_ns : $dst_period_ns}]

        if {[catch {
            set_bus_skew -from $src_bus -to $dst_bus $period_ns
        } err]} {
            puts "WARNING: set_bus_skew failed for instance prefix ${prefix} (${src_leaf} -> ${dst_leaf}): $err"
            incr n_skipped
            continue
        }
        incr n_applied
    }
    return [list $n_applied $n_skipped]
}

proc apply_async_fifo_gray_cdc_bus_skew_bound {} {
    global async_fifo_cdc_bound_applied
    if {$async_fifo_cdc_bound_applied} {
        return
    }
    set async_fifo_cdc_bound_applied 1

    set r1 [_t6_apply_bus_skew_for_direction wptr_gray_reg wptr_gray_r1_r_reg]
    set r2 [_t6_apply_bus_skew_for_direction rptr_gray_reg rptr_gray_w1_r_reg]
    set n_applied [expr {[lindex $r1 0] + [lindex $r2 0]}]
    set n_skipped [expr {[lindex $r1 1] + [lindex $r2 1]}]
    puts "=== ASYNC_FIFO GRAY-POINTER BUS_SKEW BOUND (T6): $n_applied instance(s)/direction(s) actually constrained, $n_skipped attempted-but-skipped (no source/dest pair or clock unresolved) ==="
}

proc apply_video_cdc_constraints {} {
    set core_clk [get_active_core_clock]
    set pclk_clk [get_clocks -quiet -of_objects \
        [get_pins -quiet u_video/u_mmcm/u_bufg_pclk/O]]

    if {[llength $core_clk] == 0 || [llength $pclk_clk] == 0} {
        puts "INFO: video CDC clocks not fully resolved; skipped core_clk/pclk async grouping."
        return
    }

    set_clock_groups -asynchronous -group $core_clk -group $pclk_clk
    puts "=== VIDEO CDC CLOCK GROUPS: $core_clk async to $pclk_clk ==="
    apply_async_fifo_gray_cdc_bus_skew_bound

    # pb_clk (50 MHz, direct off PCIe ref BUFG_GT) is also physically async
    # to pclk_unbuf (148.5 MHz, MMCM CLKOUT0) — different source paths,
    # no defined phase relationship.  Without an explicit constraint, STA
    # picks the LCM-aligned edge pair (~660 ns alignment, 0.067 ns required
    # window) and produces a phantom -1.7 ns violation on the dafb_vbl_cdc
    # 2-flop synchroniser (rtl/sys/pulse_cdc.v with ASYNC_REG=TRUE).  The
    # path has 0 logic levels and 0.213 ns of data delay; the synchroniser
    # handles MTBF, STA just needs to stop checking setup.  We use
    # set_false_path rather than set_clock_groups to avoid clobbering the
    # synchronous fabric_clk100 <-> pb_clk paths (which are real, related
    # fabric clocks and meet timing fine when checked synchronously).
    # The auto-derived name of this clock is NOT stable (`pb_clk_src` in
    # some builds, `pb_clk` in others -- see get_active_pb_clock above), so
    # resolve it by pin.  Previously the hard-coded `pb_clk_src` lookup
    # returned empty in the current production netlist and this exception
    # silently did nothing.
    #
    # AUDIT NOTE (2026-09-03, docs/timing_constraint_audit_2026-09-03.md):
    # making this live removes ZERO endpoints from the report in
    # build/vivado_divfix_100mhz.  The only pclk_unbuf -> pb_clk crossing in
    # that netlist is the u_dafb_vbl_cdc pulse_cdc synchroniser, which is
    # already excluded at ENDPOINT granularity by synth/fpga_top.xdc
    # ("set_false_path -to ... *u_dafb_vbl_cdc/dst_meta_reg/D"), and there
    # are no pb_clk -> pclk_unbuf paths at all (neither direction appears in
    # the Inter Clock Table with that endpoint exception in place and this
    # one absent).  Kept as authored, but it is a CLOCK-level exception over
    # two clocks that do share a root (fabric_clk_p): if a future crossing
    # appears here it should get its own endpoint-scoped exception in
    # fpga_top.xdc rather than quietly inheriting this one.
    set pb_clk [get_active_pb_clock]
    if {[llength $pb_clk] != 0} {
        set_false_path -from $pclk_clk -to $pb_clk
        set_false_path -from $pb_clk   -to $pclk_clk
        puts "=== VIDEO CDC FALSE_PATH: $pclk_clk <-> $pb_clk ==="
    }
}
apply_video_cdc_constraints

proc apply_real_mig_cdc_constraints {} {
    global use_sim_model

    if {$use_sim_model} {
        return
    }

    set core_clk [get_active_core_clock]

    # The MIG infrastructure MMCM gets analysed twice when synth_design and
    # the stitched MIG OOC checkpoint each present `sys_clk_p` to STA — once
    # via the top-level pad and once via the MIG-internal `sysclk200` net.
    # Vivado names the second appearance of every MMCM output `*_1`, so the
    # MIG UI 333 MHz clock shows up as BOTH `mmcm_clkout0` and
    # `mmcm_clkout0_1`, each with thousands of endpoints in the UI domain
    # (same BUFG output, two clock objects).  The MIG PHY 166 MHz clock
    # similarly appears as `mmcm_clkout6` / `mmcm_clkout6_1`.  Looking up
    # via `get_pins -of u_bufg_divClk/O` only returns one of the pair, so
    # the previous proc only marked half the UI fanout async to fabric and
    # left ~26 k endpoints (mmcm_clkout0_1 → fabric) timed synchronously,
    # surfacing as a -4.3 ns CDC WNS post-place.
    #
    # Filter by name pattern to capture both occurrences of every MIG UI /
    # PHY clock the MMCM produces, not just one.
    set mig_ui_clks [get_clocks -quiet -filter {NAME =~ mmcm_clkout0*}]
    set mig_phy_clks [get_clocks -quiet -filter {NAME =~ mmcm_clkout6*}]
    set mig_clks [concat $mig_ui_clks $mig_phy_clks]

    if {[llength $core_clk] == 0 || [llength $mig_clks] == 0} {
        puts "INFO: real-MIG CDC clocks not fully resolved; skipped core_clk/MIG UI async grouping."
        return
    }

    # pb_clk is a sibling fabric clock derived from the GT refclk, sourced
    # at a physically different pin (fabric_clk_p / GTYE4_COMMON) from the
    # MIG sys_clk_p input.  STA still treats it as related to mmcm_clkout0
    # absent an async constraint, which surfaces as failing setup paths on
    # the MIG cal-done -> pb_clk meta synchroniser (calDone_gated_reg ->
    # ddr_cal_done_pb_meta_reg).  Group it alongside core_clk in the same
    # async constraint so both fabric domains relax against the MIG family.
    # Resolved by pin (get_active_pb_clock): the hard-coded `pb_clk_src`
    # name that used to be here resolves in some builds and not others
    # (synth/slowclock_diag_impl.out yes, synth/slowclock8_impl.out no,
    # build/vivado_divfix_100mhz no -- there the clock is named `pb_clk`).
    set pb_clk [get_active_pb_clock]

    # WHY THIS EXCLUSION IS GENUINE, not a convenience: the whole fabric
    # family is sourced from the `fabric_clk_p/n` MGTREFCLK pair on AB7/AB6
    # (rtl/soc/fpga_top_clocks.vh: IBUFDS_GTE4 -> fabric_clk_odiv2 -> three
    # separate BUFG_GTs for sys_clk / pb_clk_src / video_ref_clk), while the
    # entire MIG PHY/UI family is sourced from the physically separate
    # `sys_clk_p/n` 200 MHz pair on T24/U24 through the MIG's own MMCM.  Two
    # independent board oscillators with no phase relationship: asynchronous
    # by construction, not by assertion.  Every real crossing between them
    # goes through rtl/soc/axi_async_bridge.v (5x rtl/board/async_fifo.v,
    # Gray-coded pointers into ASYNC_REG 2-flop synchronisers) or a plain
    # 2-flop status synchroniser; the Gray-pointer inter-bit skew those
    # FIFOs actually depend on is bounded separately by the set_bus_skew
    # pass below, which set_clock_groups does not and cannot cover.
    #
    # _clock_family() expands each side to include its generated clocks.
    # set_clock_groups does NOT walk generated clocks implicitly (UG903), so
    # naming only `fabric_clk100` left `sys_clk` (the real core clock at any
    # CORE_CLK_DIVIDE != 1), `pb_clk`, `pclk_unbuf` and `al9134_clk_fwd`
    # OUTSIDE the group and still cross-timed against the 333 MHz MIG UI
    # domain.  Same on the MIG side for the `pll_clk[*]` XIPHY clocks
    # generated off mmcm_clkout0.
    set fabric_clks [_clock_family [concat $core_clk $pb_clk]]
    set mig_clks    [_clock_family $mig_clks]

    # ddr_ctrl crosses core_clk into the MIG UI clock through axi_async_bridge.
    # The MIG calibration-done status also feeds the core reset synchronizer.
    # Keep MIG-internal MMCM/PHY clocks constrained, but do not ask STA to time
    # the intentional core<->UI async boundary.
    set_clock_groups -asynchronous -group $fabric_clks -group $mig_clks

    # The MIG UI BUFG drives a single physical net but Vivado defines two
    # clock objects on it (`mmcm_clkout0` from sys_clk_p and `mmcm_clkout0_1`
    # from sysclk200 — same BUFG output, same fanout).  STA then times every
    # MIG-internal RAM->FF path twice — once for each name pair — and the
    # tiny intra-slice LUTRAM->flop hops fail hold by 1-2 ps because the
    # source-clock-name and dest-clock-name picks pick different MMCM
    # delay arcs even though the physical clock is identical.  Post-route
    # this surfaces as 7 hold-failing endpoints on
    # `wr_buffer_ram[N].RAM32M0/RAMC/CLK -> wr_buf_out_data_reg[M]/D`,
    # all in the same SLICE, all driven by the same BUFG.  Declare the
    # pairs physically_exclusive so STA stops cross-timing the duplicate
    # names; the same constraint is the Vivado-recommended pattern when
    # one BUFG output carries two create_clock objects (UG949).
    if {[llength $mig_ui_clks] >= 2} {
        set_clock_groups -physically_exclusive \
            -group [lindex $mig_ui_clks 0] \
            -group [lindex $mig_ui_clks 1]
        puts "=== REAL MIG UI DUPLICATE CLOCKS PHYSICALLY EXCLUSIVE: [lindex $mig_ui_clks 0] / [lindex $mig_ui_clks 1] ==="
    }
    if {[llength $mig_phy_clks] >= 2} {
        set_clock_groups -physically_exclusive \
            -group [lindex $mig_phy_clks 0] \
            -group [lindex $mig_phy_clks 1]
        puts "=== REAL MIG PHY DUPLICATE CLOCKS PHYSICALLY EXCLUSIVE: [lindex $mig_phy_clks 0] / [lindex $mig_phy_clks 1] ==="
    }

    puts "=== REAL MIG CDC CLOCK GROUPS: ($fabric_clks) async to MIG ($mig_clks) ==="
    apply_async_fifo_gray_cdc_bus_skew_bound
}
apply_real_mig_cdc_constraints

# PCIe/XDMA CDC — the XDMA IP's AXI clock (`pcie_axi_aclk`, 250 MHz) and
# PIPE clock are GT TXOUTCLK-derived and trace back to the same 100 MHz
# MGTREFCLK primary as the fabric BUFG_GT tree, so STA times
# pcie_axi_aclk <-> fabric_clk100 paths as synchronous by default.  Every
# real crossing goes through axi_async_bridge (XPM async FIFOs) in
# fpga_top_debug_host.vh, so declare the domains asynchronous — without
# this the routed design shows -1.5 ns setup / -0.6 ns hold across ~95 k
# endpoints on the bridge paths.
proc apply_pcie_xdma_cdc_constraints {} {
    # Same pin-based resolution + generated-clock expansion as the MIG proc
    # above: the hard-coded `fabric_clk100` / `pb_clk_src` names missed the
    # real core clock at CORE_CLK_DIVIDE != 1 and missed pb_clk entirely in
    # builds where Vivado named it `pb_clk`.
    set core_clk [get_active_core_clock]
    set pb_clk   [get_active_pb_clock]
    set fabric_clks [_clock_family [concat $core_clk $pb_clk]]
    set pcie_clks [get_clocks -quiet -filter \
        {NAME =~ pcie_axi_aclk* || NAME =~ pipe_clk* || NAME =~ *user_clk* || NAME =~ *userclk*}]
    if {[llength $fabric_clks] == 0 || [llength $pcie_clks] == 0} {
        puts "INFO: PCIe/XDMA CDC clocks not fully resolved; skipped fabric/PCIe async grouping."
        return
    }
    set_clock_groups -asynchronous -group $fabric_clks -group $pcie_clks
    puts "=== PCIE/XDMA CDC CLOCK GROUPS: ($fabric_clks) async to PCIe ($pcie_clks) ==="
    apply_async_fifo_gray_cdc_bus_skew_bound
}
if {$enable_pcie_xdma} {
    apply_pcie_xdma_cdc_constraints
}

# Fallback: guarantee the T6 gray-pointer bus_skew bound runs at least
# once even in build configs where all three CDC procs above
# short-circuit before reaching their internal call (e.g. SIM_MODEL with
# video/PCIe both unresolved).  Idempotent — see the proc's internal
# $async_fifo_cdc_bound_applied guard.
apply_async_fifo_gray_cdc_bus_skew_bound

# ──────────────────────────────────────────────────────────────────────────────
# JTAG VIO debug — live bring-up visibility via Vivado hw_manager.
#
# The `debug_vio` IP was generated + read_ip'd up-front (see
# `gen_debug_vio_ip` above).  Instantiation happens in RTL under
# `ifdef VIO_ENABLE` in `rtl/fpga_top.v` — the instance is named
# `u_dbg_vio` and is fed core_clk-domain probes directly from the
# top-level wires.  No post-synth `create_debug_core` or
# `connect_debug_port` calls are required with this flow.
#
# Probe map: see synth/vio_dashboard.tcl (compact v27).
#
# All probes land on the core_clk domain at the instance.  pclk-domain
# signals are sampled async via core_clk under the same core_clk/pclk
# clock-group exception used by the framebuffer reader CDC.
# ──────────────────────────────────────────────────────────────────────────────
proc setup_debug_vio {} {
    global use_sim_model
    puts "=== VIO IP INSTANTIATED IN RTL (see u_dbg_vio in fpga_top.v) ==="

    if {!$use_sim_model} {
        puts "INFO: real-MIG build leaves VIO probe nets unmarked so opt_design can flatten DDR/MIG hierarchy."
        return
    }

    # Mark core-clock probe nets so opt_design doesn't collapse them.
    # The IP instance ports are already wired in RTL, but MARK_DEBUG
    # prevents the optimiser from shoving sources through CLB packers
    # and losing signal identity at the hw_manager.
    foreach n {dbg_pc vio_rst_bundle vio_ddr_axi vio_boot_video vio_axi_error err_s0_aw_addr_r err_s0_ar_addr_r vio_boot_diag vio_boot_crc vio_l2c_stats vio_scsi_sd vio_fb_reader_stats video_dbg_snap video_dbg_place vio_scsi_c96} {
        set m [get_nets -quiet $n]
        if {[llength $m] > 0} {
            set_property MARK_DEBUG true $m
            set_property DONT_TOUCH true $m
        } else {
            puts "INFO: VIO probe net '$n' not resolved at synth time — skipped MARK_DEBUG."
        }
    }
    puts "=== VIO DEBUG CORE CONSTRAINTS APPLIED ==="
}
if {$enable_vio} {
    setup_debug_vio
} else {
    puts "=== SKIPPING JTAG VIO DEBUG CORE (set ENABLE_VIO=1 to enable) ==="
}

# ──────────────────────────────────────────────────────────────────────────────
# JTAG ILA debug — cycle-accurate capture via Vivado hw_manager.
#
# The `debug_ila` IP was generated + read_ip'd above via gen_debug_ila_ip.
# Instantiation happens in RTL under `ifdef ILA_ENABLE` in
# rtl/fpga_top_debug_vio.vh — the instance is named `u_dbg_ila` and is
# fed core_clk-domain probes directly from the top-level wires.
#
# Probe map — see synth/debug_ila.tcl and docs/ila_a7_drift_probes.md.
#
# Trigger setup is configured at runtime in Vivado HW Manager — once
# the bitstream is loaded the user attaches to hw_ila_1 and sets up
# a trigger (e.g. probe10 not equal to previous OR probe9 != 0 to
# catch any exception fire).  No static trigger is baked in.
# ──────────────────────────────────────────────────────────────────────────────
proc setup_debug_ila {} {
    global use_sim_model
    puts "=== ILA IP INSTANTIATED IN RTL (see u_dbg_ila in fpga_top_debug_vio.vh) ==="

    if {$use_sim_model} {
        # SIM_MODEL builds don't carry the DDR4/MIG hierarchy so it is
        # safe to mark every probe net.  Real-MIG builds leave probe
        # nets unmarked to let opt_design flatten cleanly through the
        # MIG (same constraint as the VIO setup above).
        foreach n {dbg_ila_commit_bundle_w
                   dbg_ila_rob_arch_dst_w dbg_ila_rob_phys_dst_w
                   dbg_ila_rob_phys_old_w dbg_ila_rob_pop_w
                   dbg_ila_rob_is_last_uop_w dbg_ila_flush_en_w
                   dbg_ila_rob_pc_w dbg_ila_arch_a7_w
                   dbg_ila_dc_aw_is_evict_w
                   ila_a7_illegitimate_step ila_wb_addr
                   ila_axi_w_snap ila_exc_count_lo} {
            set m [get_nets -quiet $n]
            if {[llength $m] > 0} {
                set_property MARK_DEBUG true $m
                set_property DONT_TOUCH true $m
            } else {
                puts "INFO: ILA probe net '$n' not resolved at synth time — skipped MARK_DEBUG."
            }
        }
    } else {
        puts "INFO: real-MIG build leaves ILA probe nets unmarked so opt_design can flatten DDR/MIG hierarchy."
    }
    puts "=== ILA DEBUG CORE CONSTRAINTS APPLIED ==="
}
if {$enable_ila} {
    setup_debug_ila
} else {
    puts "=== SKIPPING JTAG ILA DEBUG CORE (set ENABLE_ILA=1 to enable) ==="
}

write_checkpoint -force $output_dir/checkpoints/synth.dcp

# Hierarchical synthesis utilization is intentionally kept alongside the
# flat total.  Wide variable-index/shift logic (DMA assembly, L2 quadrant
# placement, video address generation) is otherwise difficult to distinguish
# from ordinary LUT growth until after the much slower implementation run.
report_utilization \
    -hierarchical \
    -file $output_dir/reports/utilization_synth.rpt
report_timing_summary \
    -max_paths 20 \
    -report_unconstrained \
    -file $output_dir/reports/timing_synth.rpt

puts "=== SYNTH TIMING SUMMARY ==="
report_timing_summary -max_paths 5

if {$mode eq "synth_only"} {
    puts "=== SYNTHESIS COMPLETE ==="
    puts "Checkpoint: $output_dir/checkpoints/synth.dcp"
    # Write debug probes at synth-complete so if a later impl_from_dcp
    # run writes a bitstream, the probe file is already staged.
    write_debug_probes -force $output_dir/fpga_top_synth.ltx
    exit 0
}

# ──────────────────────────────────────────────────────────────────────────────
# Implementation
# ──────────────────────────────────────────────────────────────────────────────
# Note: opt_design must run BEFORE read_checkpoint -incremental when the
# design contains MIG cores — Vivado refuses with "Found memory core that
# needs to be (re)generated.  Please run opt_design or implement_mig_cores
# prior to launching 'read_checkpoint -incremental'."  So we opt first and
# only THEN load the incremental reference for place_design / route_design
# to consume.
# Part 132: all timing constraints are in place by now, so this is the last
# honest moment to check that the processor still has a hold requirement.
p132_assert_core_hold_analysed [get_active_core_clock]

puts "=== OPTIMISING ==="
# ── CORE_BUDGET_NS — ask STA for a TIGHTER core budget (measurement hook) ──
#
# Inert unless the env var is set; never set it for a build you intend to
# flash.  It exists because there is otherwise NO way to make this project
# report "does the fabric close at 200 MHz":
#
#   * TARGET_FREQ_MHZ (the block near line 1510) only ever RELAXES, and its
#     whole body is guarded by `if {$target_freq_mhz != 200.0}` -- so asking
#     for exactly 200 skips it and measures nothing.
#   * `create_clock -period 5.000` on the fabric clock is WRONG here: the
#     HDMI MMCM is parameterised CLKIN1_PERIOD=10.000
#     (rtl/soc/fpga_top_video.vh), so redeclaring the input doubles every
#     derived video clock and contaminates the video and MIG numbers.
#
# So express the target as a plain max-delay budget on the core clock,
# leaving every clock FREQUENCY untouched.  No `-datapath_only`, so hold
# and clock uncertainty stay live.
#
# THE pb TRANSFERS ARE PART OF THE ANSWER.  core_clk <-> pb_clk are real
# synchronous transfers between two clocks off the same root.  At 100/50 MHz
# their tightest launch/capture pair is 10 ns; at 200/50 MHz it is 5 ns --
# the same budget as an intra-core path.  Constraining only intra-core
# paths therefore reports an OPTIMISTIC number: measured at 10 ns the two
# directions sat at +5.230 and +3.241, i.e. about +0.2 and -1.8 at 5 ns.
# Both clocks are resolved BY PIN (get_active_core_clock /
# get_active_pb_clock) because their auto-derived names are not stable
# across builds -- `get_clocks pb_clk` vs `pb_clk_src` has silently
# returned empty before, which is exactly how a constraint ends up
# measuring nothing.  Placed here, after every clock-constraint proc has
# run and before opt_design, so it sees the final clock set.  NOTE that
# this is AFTER reports/timing_synth.rpt is written, so only the place
# and route reports carry the tightened budget -- read timing_route.rpt.
if {[info exists ::env(CORE_BUDGET_NS)] && $::env(CORE_BUDGET_NS) ne ""} {
    set cb_ns   [expr {double($::env(CORE_BUDGET_NS))}]
    set cb_core [get_active_core_clock]
    set cb_pb   [get_active_pb_clock]
    if {[llength $cb_core] > 0} {
        set_max_delay -from $cb_core -to $cb_core $cb_ns
        puts "=== CORE_BUDGET_NS=$cb_ns : set_max_delay within ($cb_core) ==="
        if {[llength $cb_pb] > 0} {
            set_max_delay -from $cb_core -to $cb_pb $cb_ns
            set_max_delay -from $cb_pb   -to $cb_core $cb_ns
            puts "=== CORE_BUDGET_NS=$cb_ns : set_max_delay both ways ($cb_core) <-> ($cb_pb) ==="
        } else {
            puts "=== CORE_BUDGET_NS: WARNING no pb clock resolved; the reported number is OPTIMISTIC ==="
        }
    } else {
        puts "=== CORE_BUDGET_NS: no core clock resolved; nothing applied ==="
    }
}

# ── async_fifo GRAY-POINTER CROSSINGS ─────────────────────────────────────────
# MEASURED (2026-09-22): of the twenty *_hold_fix delay cells Vivado inserted to
# repair hold in the preceding build, FIFTEEN were in
# u_pb_s1_cdc/u_bridge/{aw,w,b,ar,r}_fifo/{w,r}ptr_gray* -- three per FIFO across
# all five AXI channel FIFOs. Three quarters of a FINITE hold-repair budget went
# into an async FIFO's gray pointers, and that build still exited with twelve
# hold endpoints failing. The inserted cells then show up in setup path dumps at
# 5.3-6.9 ns, so the repair is not free either.
#
# Those legs are timed only because core_clk and pb_clk are deliberately kept
# phase-related rather than declared asynchronous (fpga_top_clocks.vh: "a
# well-defined synchronous transfer rather than a CDC").  That is right for the
# DATA path and is NOT changed here.  It also drags in the gray pointers, which
# are gray-coded precisely so one bit changes per update and the receiver
# tolerates arbitrary skew -- a hold check on them is meaningless work.
#
# -datapath_only excludes clock phase/skew and drops the min requirement on these
# legs only, while still BOUNDING the path at one core period (5 ns, the shorter
# of the two clocks) so a pointer bit is stable before its source can change
# again.  The synchroniser's own interstage path stays normally timed.  This is
# NOT set_clock_groups -asynchronous: nothing else about core<->pb moves.
#
# WHY HERE AND NOT IN fpga_top.xdc.  That file is read while the design still has
# unresolved black boxes, so get_cells returns nothing and Vivado defers the whole
# constraint; and its parser rejects `if`/`puts` (Designutils 20-1307), so the
# match could not be asserted there.  A CDC constraint change has silently killed
# a boot on this board once (p183cdc, exc_count=0), so the match IS asserted here:
# a rename in async_fifo.v fails the build loudly instead of quietly disabling it.
# ── SONIC HANDSHAKE-QUALIFIED CDC PAYLOAD ────────────────────────────────────
# MEASURED: the router's "tight setup and hold" report names exactly one pin,
# build after build:
#
#   Launch Setup Clock | Launch Hold Clock | Pin
#   core_mmcm_clkout0  | core_mmcm_clkout1 | g_sonic_dma_client.u_q700_sonic_rx/state_reg[*]/D
#
# Setup is checked against the core clock and hold against the peripheral clock,
# so the window is whatever those two related clocks happen to leave -- a
# requirement no amount of routing effort can satisfy, and a direct obstacle to
# meeting WNS and WHS simultaneously.
#
# WHY IT IS SPURIOUS. q700_sonic_rx_cdc is a correct handshake CDC: the VALID is
# 2FF synchronised (cv1/cv2, ASYNC_REG) while the payload crosses combinationally
#     assign core_cfg_cdc = pb_cfg_cdc;   ... and 13 siblings
# The payload is written by the pb side and left stable; the core side may only
# consume it once the synchronised valid arrives, which is at least two core
# cycles later. So the payload legitimately needs no synchroniser -- but nothing
# in the constraints says so, and because core_clk and pb_clk are deliberately
# phase-related rather than asynchronous (fpga_top_clocks.vh), the tool times it
# as an ordinary single-cycle transfer.
#
# -datapath_only excludes clock phase/skew and drops the min requirement, while
# still BOUNDING the payload at one core period -- far tighter than the >=2 core
# cycles the handshake actually guarantees, so the bound is conservative.
# Scoped to the SONIC CDC crossings only; this is NOT set_clock_groups
# -asynchronous and nothing else about core<->pb changes.
#
# The owner has confirmed the Ethernet side is not latency critical.
set son_pb   [get_cells -quiet -hier -regexp {.*u_q700_eth_sonic.*} -filter {IS_SEQUENTIAL}]
set son_core [get_cells -quiet -hier -regexp {.*g_sonic_dma_client.*} -filter {IS_SEQUENTIAL}]
if {[llength $son_pb] == 0 || [llength $son_core] == 0} {
    puts stderr "ERROR: SONIC CDC constraint matched nothing (pb=[llength $son_pb] core=[llength $son_core]) -- did the sonic hierarchy get renamed?"
    exit 1
}
puts "=== SONIC CDC payload: [llength $son_pb] pb-side / [llength $son_core] core-side cells, set_max_delay -datapath_only 5.000 both ways ==="
set_max_delay -datapath_only 5.000 -from $son_pb   -to $son_core
set_max_delay -datapath_only 5.000 -from $son_core -to $son_pb

set gray_src [get_cells -quiet -hier -regexp {.*/(wptr_gray|rptr_gray)_reg\[\d+\]}]
set gray_dst [get_cells -quiet -hier -regexp {.*/(wptr_gray_r1_r|rptr_gray_w1_r)_reg\[\d+\]}]
if {[llength $gray_src] == 0 || [llength $gray_dst] == 0} {
    puts stderr "ERROR: async_fifo gray-pointer constraint matched nothing (src=[llength $gray_src] dst=[llength $gray_dst]) -- did async_fifo.v rename wptr_gray/rptr_gray/*_r1_r?"
    exit 1
}
puts "=== async_fifo gray pointers: [llength $gray_src] source / [llength $gray_dst] sync cells, set_max_delay -datapath_only 5.000 ==="
set_max_delay -datapath_only 5.000 -from $gray_src -to $gray_dst

# ── L2C DATA-ARRAY FLOORPLAN: slice-major LOC on the 64 URAM288s ─────────────
# The only floorplan constraint in this flow, and deliberately NOT a pblock.
# Whole-core pblocks were measured on this design and made it worse (the
# core's units are fused at the LUT level, so a box forces their tails in);
# this pins 64 hard macros and nothing else, and it exists because of a
# measured net census, not a hunch.
#
# MEASURED on the routed dcache-read-base checkpoint (2026-09-22, same flow,
# WNS -0.143), against the router's "NORTH global/long congestion" band
# INT_X32-55 / Y126-205 that terminates on URAM_URAM_FT_X51Y150:
#
#   * All 64 URAMs sit in ONE column (URAM288_X0Y0..Y63, tile column X51) that
#     spans the full 240-row die.  Vivado scattered the 8 ways of each 72-bit
#     slice over the whole column (way 0's eight URAMs were at site Y4, 16, 28,
#     36, 44, 45, 48, 56).
#   * Of ALL nets crossing Y=150 inside X32-60, 37% (1675 of 4493) touch a URAM
#     pin.  By driver, u_l2c/.../u_data + u_mshr are 38% of those crossings and
#     34% of the summed vertical span in the band -- from ~5% of the design's
#     LUTs.  The core's ROB/D-cache/DTLB/LSU are the CELLS in the band; the L2
#     data array is the WIRE.
#   * The 575 write-data/strobe nets (one per line bit + byte enable) each fan
#     out to the 8 ways of one slice: mean vertical span 150 rows, 413 of 575
#     cross Y=150.  The 4096 URAM read-data nets each feed one 8:1 way-mux LUT
#     whose eight sources are scattered: 171k row-tracks, 1126 cross Y=150.
#
# THE CONSTRAINT.  Place slice N, way W at URAM288_X0Y(8*N+W), so the eight
# ways of every slice are vertically adjacent (two URAM tiles, ~30 rows).  Then
# every write-data net's 8 loads are local and every way-mux LUT's 8 sources
# are local.  Simulated on the same checkpoint with all drivers held where
# they are (conservative -- the placer will move the mux LUTs): write-net mean
# span 150 -> 82 rows, Y=150 crossings 413 -> 170; the read side becomes
# ~15-row nets by construction.  The 12 read-address/CE broadcasts to all 64
# URAMs are unchanged and unavoidable.
#
# WHY SLICE-major and not WAY-major.  The L2's own worst paths on that
# checkpoint were req_tag_reg -> u_data/g_way[W].mem_reg_uram_N/EN_B (87%
# route): the hit-way write enable, 8 nets each reaching a way's 8 URAMs.
# Way-major would localise those 8 nets and scatter the other 4671; and it
# does not even shorten the enable's WORST leg, which is driver-to-farthest-
# group in either ordering.  uram_7 holds only bits 504..511, so it goes at
# the top, farthest from the (south-placed) write drivers.
#
# Netlist-dependent (get_cells), so it lives here after synth_design, not in
# an XDC.  The match is asserted: a rename in l2c_data.v or a change in the
# 8x8 geometry fails the build loudly rather than silently dropping the
# floorplan.  L2C_URAM_FLOORPLAN=0 disables it for a controlled A/B build.
# MEASURED (2026-09-22) by a controlled A/B: identical CPU/SoC revisions, ETH
# on, 200 MHz, AggressiveExplore in a split process, differing only in
# L2C_URAM_FLOORPLAN.  ON: WNS +0.006 / WHS +0.007, ZERO failing setup and hold
# endpoints out of 342,482 / 341,781.  OFF: WNS -0.046 with 183 failing setup
# endpoints, and the post-route phys_opt loop found -0.004 five times but drove
# hold negative each time and rolled back.  So the floorplan is worth ~52 ps and
# is what closes this design.  Its one cost is a slow-corner Min Skew violation
# of -0.018 on one DDR4 XIPHY bitslice site (BITSLICE_RX_TX_X0Y36, inside the
# MIG's own PHY), absent from every floorplan-off build; the fast corner on the
# equivalent site passes at +0.292.
# ($l2c_uram_floorplan is parsed with the other build knobs near ENABLE_ILA.)
if {$l2c_enable_effective && $l2c_uram_floorplan} {
    set uram_cells [get_cells -quiet -hier -regexp \
        {.*u_l2c/g_active\.u_ctrl/u_data/g_way\[[0-7]\]\.mem_reg_uram_[0-7]$} \
        -filter {REF_NAME == URAM288}]
    if {[llength $uram_cells] != 64} {
        puts stderr "ERROR: L2C URAM floorplan expected 64 URAM288 cells named u_l2c/g_active.u_ctrl/u_data/g_way\[W\].mem_reg_uram_N, found [llength $uram_cells] -- did l2c_data.v change geometry or naming? (L2C_URAM_FLOORPLAN=0 disables this)"
        exit 1
    }
    set uram_seen [dict create]
    foreach c $uram_cells {
        if {![regexp {g_way\[([0-7])\]\.mem_reg_uram_([0-7])$} [get_property NAME $c] -> uway uslice]} {
            puts stderr "ERROR: L2C URAM floorplan: cannot parse way/slice from [get_property NAME $c]"
            exit 1
        }
        set uy [expr {8 * $uslice + $uway}]
        set usite "URAM288_X0Y$uy"
        if {[llength [get_sites -quiet $usite]] != 1} {
            puts stderr "ERROR: L2C URAM floorplan: site $usite does not exist on $part"
            exit 1
        }
        if {[dict exists $uram_seen $uy]} {
            puts stderr "ERROR: L2C URAM floorplan: two cells map to $usite ([dict get $uram_seen $uy] and $c)"
            exit 1
        }
        dict set uram_seen $uy $c
        set_property LOC $usite $c
    }
    puts "=== L2C URAM floorplan: 64 URAM288 LOCed slice-major, URAM288_X0Y(8*slice+way) ==="
} else {
    puts "=== L2C URAM floorplan: skipped (l2c_enable=$l2c_enable_effective L2C_URAM_FLOORPLAN=$l2c_uram_floorplan) ==="
}

# ──────────────────────────────────────────────────────────────────────────────
# DDR4 XIPHY bitslice Min Skew: pin one MIG write-serializer flop near its byte
# lane.
#
# The last failing timing check on this design was not in our logic at all. The
# XIPHY RXTX_BITSLICE for byte lane 2 carries a Min Skew requirement between its
# D[1] and D[2] inputs -- the two arrivals must be at least 90 ps APART in the
# slow corner -- and the routed design delivered 72 ps, for WPWS -0.018 on one
# endpoint out of 113,769.
#
# WHAT DRIVES THOSE PINS. D[0..7] of the bitslice come from the MIG's own write
# serializer flops, u_ddr_mc_pi/u_ddr_mc_write/genByte[2].../genBit[7].../
# dReg_reg[0..7]. Nothing constrains where they go, and the placer scattered
# this bit's eight across SLICE_X3Y52, X4Y67, X7Y65, X4Y67, X7Y65, X4Y67, X6Y64
# and X3Y65. The skew between any two of them is then whatever the routing
# happened to give: MEASURED across the four lanes that carry this check, in one
# build, 0.072 / 0.543 / 0.270 / 0.144 ns against a 0.090 requirement. The same
# lane measured 0.161 in the otherwise-identical L2C_URAM_FLOORPLAN=0 build. It
# is a lottery, not a design property, and it is the reason this check moves for
# reasons that have nothing to do with the change under test.
#
# WHY A SINGLE LOC IS THE FIX. The requirement is a MINIMUM separation, so the
# lever is to make one of the two arrivals clearly different from the other, and
# the cheap direction is to pull D[1]'s driver close to the bitslice while D[2]'s
# stays out at X7: the byte lane's neighbourhood is otherwise full (zero wholly
# free slices in X4..X12 across Y30..Y79), so D[2] cannot drift inward to close
# the gap again. MEASURED on the routed checkpoint: moving this one flop to any
# of eight free slices near the lane (X1Y37, X2Y35, X1Y39, X0Y32, X2Y32, X2Y41,
# X2Y31, X1Y28) removed the violation, and all eight left WNS +0.006 and WHS
# +0.007 exactly unchanged. X1Y37 is the closest to the lane and is what is
# pinned here.
#
# WHAT WOULD INVALIDATE IT. A different MIG configuration or a regenerated core
# can rename or re-shape this hierarchy, and a byte lane other than 2 can be the
# one that comes out short. So the cell match is asserted rather than assumed,
# and the post-route report records WPWS in the buildinfo so a build that loses
# this check says so in its own manifest instead of being found out later.
# MIG_WR_FLOP_LOC=0 disables it for a controlled A/B.
# ──────────────────────────────────────────────────────────────────────────────
set mig_wr_flop_loc [parse_bool_env MIG_WR_FLOP_LOC 1]
set mig_wr_flop_site "SLICE_X1Y37"
if {!$use_sim_model && $mig_wr_flop_loc} {
    set mig_wr_cell [get_cells -quiet -hier -regexp \
        {.*u_ddr_mc_write/genByte\[2\]\.u_ddr_mc_wr_byte/genBit\[7\]\.u_ddr_mc_wr_bit/dReg_reg\[1\]$}]
    if {[llength $mig_wr_cell] != 1} {
        puts stderr "ERROR: DDR4 bitslice Min Skew fix expected exactly one MIG write-serializer flop matching .../u_ddr_mc_write/genByte\[2\].u_ddr_mc_wr_byte/genBit\[7\].u_ddr_mc_wr_bit/dReg_reg\[1\], found [llength $mig_wr_cell] -- did the DDR4 core get regenerated with a different geometry? (MIG_WR_FLOP_LOC=0 disables this)"
        exit 1
    }
    if {[llength [get_sites -quiet $mig_wr_flop_site]] != 1} {
        puts stderr "ERROR: DDR4 bitslice Min Skew fix: site $mig_wr_flop_site does not exist on $part"
        exit 1
    }
    # LOC only, no BEL: the slice is what sets the net length, and leaving the
    # flop's BEL free lets the placer use whichever FF in it suits the control
    # set rather than failing on a hardcoded one. (X1Y37 is a SLICEM on this
    # part, so a hardcoded SLICEL BEL would be wrong outright.)
    set_property LOC $mig_wr_flop_site $mig_wr_cell
    puts "=== DDR4 XIPHY Min Skew: byte-2 write-serializer dReg_reg\[1\] pinned to $mig_wr_flop_site ==="
} else {
    puts "=== DDR4 XIPHY Min Skew: flop LOC skipped (use_sim_model=$use_sim_model MIG_WR_FLOP_LOC=$mig_wr_flop_loc) ==="
}

opt_design -directive Explore

if {!$no_incremental && $incremental_ref_dcp ne "" && \
        [file exists $incremental_ref_dcp]} {
    puts "=== INCREMENTAL: reusing $incremental_ref_dcp as routed reference ==="
    read_checkpoint -incremental $incremental_ref_dcp
} elseif {$no_incremental} {
    puts "=== INCREMENTAL: disabled (NO_INCREMENTAL=1) — running full impl ==="
} elseif {$incremental_ref_dcp eq ""} {
    puts "=== INCREMENTAL: no INCREMENTAL_REF_DCP set — running full impl ==="
} else {
    puts "=== INCREMENTAL: reference $incremental_ref_dcp not found yet — running full impl, will stash on success ==="
}

set mig_bridge_fanout_nets [get_nets -hier -quiet -regexp \
    {^.*/u_ddr/u_repo_to_pcie_mig/wr_prev_mig_index(_bank[0-3])?_q_reg_n_0_\[[0-8]\]$}]
if {[llength $mig_bridge_fanout_nets] > 0} {
    puts "=== Applying FORCE_MAX_FANOUT=64 to [llength $mig_bridge_fanout_nets] DDR bridge index nets ==="
    set_property FORCE_MAX_FANOUT 64 $mig_bridge_fanout_nets
}

puts "=== PLACING ==="
# TEMPORARY DIAGNOSTIC (congestion investigation): allow overriding the
# place_design directive via PLACE_DIRECTIVE for A/B testing.  Default
# CHANGED 2026-07-12 (route_design congestion investigation) from
# ExtraPostPlacementOpt to AltSpreadLogic_high: full-chip route_design
# was failing with ~118-120 residual node overlaps (207 signals failing
# to route due to congestion, placer emitting "highly congested, may
# have difficulty routing" well before phys_opt/route even ran).
# AltSpreadLogic_high A/B-tested clean: it eliminates that placer
# congestion warning entirely AND improves WNS (0.230ns -> 0.358ns) at
# the placement checkpoint, and in a full route_design run reduced the
# residual node-overlap failure from 118 to 33 (72% reduction) with no
# RTL change.  Override via PLACE_DIRECTIVE=<name> still available for
# further A/B testing.
if {[info exists ::env(PLACE_DIRECTIVE)] && $::env(PLACE_DIRECTIVE) ne ""} {
    puts "=== DIAG: place_design directive overridden to $::env(PLACE_DIRECTIVE) ==="
    place_design -directive $::env(PLACE_DIRECTIVE)
} else {
    place_design -directive AltSpreadLogic_high
}

write_checkpoint -force $output_dir/checkpoints/place.dcp

# ── PLACE/ROUTE PROCESS SPLIT ────────────────────────────────────────────────
# `place_only` stops here so routing runs in a SEPARATE Vivado process, from
# this checkpoint, via synth/resume_from_place.tcl.
#
# This is not tidiness. route_design's post-routing leaf-clock programmable-delay
# optimisation ("Phase N.1.1 Leaf ClockOpt Init") segfaults on this design --
# `Abnormal program termination (11)` -- and it now does so DETERMINISTICALLY for
# BOTH `Explore` and `AggressiveExplore`, after routing itself has completed. The
# same placement routed by a freshly started Vivado gets through. The crash
# therefore tracks accumulated process state, not the placement or the directive,
# which is why splitting the processes is the fix rather than a workaround.
#
# The flow comment further down already prescribed re-opening place.dcp after a
# crash; this makes that the normal path instead of the recovery path, so an
# hour of synthesis and placement is never at risk from it.
if {$mode eq "place_only"} {
    report_timing_summary -max_paths 10 -file $output_dir/reports/timing_place.rpt
    puts "=== PLACE COMPLETE (process split): $output_dir/checkpoints/place.dcp ==="
    puts "=== route it with: vivado -mode batch -source synth/resume_from_place.tcl -tclargs $output_dir <directive> ==="
    exit 0
}
report_timing_summary \
    -max_paths 10 \
    -file $output_dir/reports/timing_place.rpt

puts "=== PHYS OPT ==="
phys_opt_design -directive AggressiveExplore

# ──────────────────────────────────────────────────────────────────────────────
# Post-phys_opt force-replication pass.  Vivado's BUFG-insertion log on
# vivado_main_4284d9b explicitly recommended `phys_opt_design
# -force_replication_on_nets <nets>` as the resolution path for the
# soc_full_rst / core_rst broadcast cones (msg [Place 46-32]: "BUFG
# insertion was skipped because the netlist editing failed").  After the
# xpm_cdc_async_rst + explicit BUFG migration (agent/xpm-reset-
# distribution) those reset cones travel on the global clock network
# directly, so force-replicating them here would actively undo the
# distribution.  Reset patterns are intentionally excluded; only non-
# reset event broadcasts that still need user-FF replication remain.
# ──────────────────────────────────────────────────────────────────────────────
# `*vif_rdata*` added 2026-07-12 alongside the MAX_FANOUT=256 property
# set above — see that block's comment for the full evidence trail
# (reproducible residual-overlap contributor across two different
# place_design directives, growing share as generic congestion clears).
set rep_nets [get_nets -quiet -hier -filter \
    {NAME =~ "*dbg_overlay_q*" || NAME =~ "*crat_rb*" || NAME =~ "*vif_rdata*"}]
if {[llength $rep_nets] > 0} {
    puts "=== phys_opt -force_replication_on_nets on [llength $rep_nets] nets ==="
    phys_opt_design -force_replication_on_nets $rep_nets
}

# ── TEMPORARY DIAGNOSTIC (congestion investigation, remove after use) ──
# When DIAG_STOP_AFTER_PHYSOPT=1 is set in the environment, checkpoint
# and report congestion right here (post-phys_opt, pre-route) and exit
# before committing to the multi-hour route_design run.  Not intended
# to be a permanent flow change.
if {[info exists ::env(DIAG_STOP_AFTER_PHYSOPT)] && $::env(DIAG_STOP_AFTER_PHYSOPT) eq "1"} {
    puts "=== DIAG: writing post-phys_opt checkpoint + congestion report ==="
    write_checkpoint -force $output_dir/checkpoints/physopt.dcp
    report_design_analysis -congestion -file $output_dir/reports/congestion_physopt.rpt
    report_design_analysis -complexity -file $output_dir/reports/complexity_physopt.rpt
    catch {
        report_high_fanout_nets -timing -load_types -fanout_greater_than 200 \
            -file $output_dir/reports/high_fanout_physopt.rpt
    } hfn_err
    if {$hfn_err ne ""} {
        puts "=== DIAG: report_high_fanout_nets failed: $hfn_err ==="
    }
    puts "=== DIAG STOP: exiting before route_design (DIAG_STOP_AFTER_PHYSOPT=1) ==="
    exit 0
}

puts "=== ROUTING ==="
# 2026-07-09: task #101 fmove-imm/fmovemx-mode6 decode fix build hit a
# congestion-driven route_design failure with AggressiveExplore (1 residual
# node overlap between u_lsu/cdb_phys_reg[6] and a PRF register bit after
# ~3.5h of rip-up/reroute — AggressiveExplore optimizes for timing closure
# via multiple placement attempts, which can worsen congestion on an
# already-marginal design). Explore is Xilinx's recommended directive for
# congested designs (balances timing + routability rather than chasing
# timing alone) — trying it here since the design was 1 net away from
# routing cleanly.
# Router directive, overridable via env ROUTE_DIRECTIVE (mirrors PLACE_DIRECTIVE
# above).  Default stays Explore -- every closing build to date used it.
# 2026-08-16: adding the I-cache debug probe pushed this design over the routing
# cliff -- 14587 signals failed to route at "Effective congestion level: 6", after
# the two prior builds closed with only +0.042 / +0.032 ns WNS, i.e. no headroom.
# For a congestion failure (as opposed to a timing one) try:
#     ROUTE_DIRECTIVE=AlternateCLBRouting   # spreads routing off congested CLBs
#     ROUTE_DIRECTIVE=AggressiveExplore     # slower, wider search
# Congestion is reported by route_design as "[Route 35-162] N signals failed to
# route"; a TIMING failure looks different (negative WNS, 0 unrouted) and this
# knob will not help there.
if {[info exists ::env(ROUTE_DIRECTIVE)] && $::env(ROUTE_DIRECTIVE) ne ""} {
    puts "=== route_design -directive $::env(ROUTE_DIRECTIVE) (env override) ==="
    route_design -directive $::env(ROUTE_DIRECTIVE)
} else {
    # AggressiveExplore, not Explore (2026-09-15).  TWO measured reasons:
    #
    # 1. CRASH.  `Explore` segfaults deterministically at "Phase 11.1.1 Leaf
    #    ClockOpt Init" (Abnormal program termination (11), make exit 139) on
    #    this design -- three separate netlists this session (items3+6, the SD
    #    read-ahead build, and read-ahead+via2-strand-fix).  It is
    #    NETLIST-DEPENDENT, not universal: earlier netlists routed through it
    #    fine, which is why it was not noticed sooner.  Routing itself always
    #    completed ("Phase 10 Depositing Routes"); the crash is in the
    #    post-route programmable-clock-delay optimisation.  There is no Vivado
    #    param to disable that phase -- I searched list_param for *clock*,
    #    *leaf*, *skew*, *progdelay*, *clkopt* and found nothing.
    #
    # 2. QUALITY.  On the same placement, measured post-route:
    #       Explore           segfault (no result)
    #       Default           WNS -1.504  -- completes but throws the placement away
    #       AggressiveExplore WNS -0.634, WHS +0.004, TNS -2582, 11316 failing EP
    #                         -- the best result of the campaign, and the first
    #                         hold-CLEAN 200 MHz route.
    #
    # If a future netlist makes AggressiveExplore crash instead, the recovery is
    # the same shape: the placement checkpoint survives, so re-open
    # <outdir>/checkpoints/place.dcp and route it with another directive rather
    # than rebuilding the ~50 minutes of synthesis and placement.
    route_design -directive AggressiveExplore
}

# ── Post-route physical optimisation ─────────────────────────────────────
# The flow previously went straight from route_design to reports, so the only
# phys_opt passes ran BEFORE routing and could not see final route delays.
# That left the last fraction of a nanosecond on the table, which is exactly
# where this design lives (the shipped non-L2C bitstream closed at WNS
# +0.019ns).
#
# Added 2026-07-28 after the first L2C + VRAM_IN_DDR build MISSED at
# WNS -0.334ns / TNS -109.557ns / 678 failing endpoints.  Every failing path
# was ROUTE-dominated, not logic-bound:
#   -0.334  u_ddr/u_repo_to_pcie_mig/rq_issue_ptr_reg[0]_replica
#             -> u_mig_ddr4/.../axi_ar_channel_0/.../axaddr_incr_reg[27]
#             9 levels, 66% route   (group mmcm_clkout0)
#   -0.140  u_cpu/.../u_iq_mem/e_valid_reg[0] -> e_is_rts_reg[1]/CE
#             25 levels, 74% route
#   -0.099  u_cpu/.../u_dec/ext1_f3_q_reg[15] -> q_arch_dst_reg[0]/D
#             29 levels, 73% route
# Route-dominated critical paths are precisely what post-route phys_opt
# targets (routing-aware replication / retiming / rewiring), so this is the
# cheapest possible fix and carries no RTL risk.
#
# Guarded: phys_opt_design can legitimately find nothing to do and it must
# never fail the build, so a failure here is a warning, not an error.  It is
# also skipped when timing already MET — no point perturbing a good route.
# ITERATE rather than run a fixed pair (2026-09-14). Each pass re-optimises the
# result of the last and moves the critical path somewhere new, so a different cost
# function then has fresh work to do. Two passes stop well short of the point of
# diminishing returns on a design that lives this close to the edge -- the 200 MHz
# SoC sits at WNS ~-0.45 with ~10k failing endpoints, all route-dominated, which is
# exactly what routing-aware replication/retiming/rewiring targets.
#
# Loop until a pass stops helping, capped so a pathological case cannot run forever.
# ⚠️ HOLD IS WATCHED, NOT ASSUMED. phys_opt optimises SETUP and can trade hold away,
# and a hold violation corrupts data at ANY clock rate -- it is what produced visible
# video glitching on this board before. If a pass drives WHS negative the loop stops
# immediately and says so, rather than banking a setup gain that is not real.
proc _pr_wns {} { return [get_property SLACK [get_timing_paths -delay_type max -max_paths 1 -nworst 1]] }
proc _pr_whs {} { return [get_property SLACK [get_timing_paths -delay_type min -max_paths 1 -nworst 1]] }
if {[get_property SLACK [get_timing_paths -delay_type min_max]] < 0} {
    set pr_dirs {AggressiveExplore AlternateReplication AggressiveFanoutOpt Explore AlternateReplication AggressiveExplore}
    set pr_max  [expr {[info exists ::env(POST_ROUTE_PHYSOPT_MAX)] ? $::env(POST_ROUTE_PHYSOPT_MAX) : 6}]
    set pr_prev [_pr_wns]
    set pr_whs0 [_pr_whs]
    set pr_ckpt [file join $output_dir checkpoints post_route_holdclean.dcp]
    file mkdir [file join $output_dir checkpoints]
    puts [format "=== POST-ROUTE PHYS_OPT: start WNS %.3f WHS %.3f (max %d passes) ===" $pr_prev $pr_whs0 $pr_max]
    set pr_i 0
    foreach pr_d $pr_dirs {
        if {$pr_i >= $pr_max} { puts "=== POST-ROUTE PHYS_OPT: pass cap reached ==="; break }
        incr pr_i
        # Snapshot the current (hold-clean) state so a pass that trades hold away can be
        # rolled back instead of being shipped.
        if {[catch {write_checkpoint -force $pr_ckpt} pr_ck_err]} {
            puts "WARN: could not write rollback checkpoint: $pr_ck_err"
        }
        if {[catch {phys_opt_design -directive $pr_d} pr_err]} {
            puts "WARN: post-route phys_opt_design ($pr_d) failed: $pr_err"
            continue
        }
        set pr_now [_pr_wns]; set pr_hold [_pr_whs]
        puts [format "=== POST-ROUTE PHYS_OPT pass %d (%s): WNS %.3f (was %.3f, delta %+.3f)  WHS %.3f ===" \
                     $pr_i $pr_d $pr_now $pr_prev [expr {$pr_now - $pr_prev}] $pr_hold]
        # Closure takes precedence over relative gain: a hold-only repair, or
        # reduced but still positive setup slack, is already a successful route.
        if {$pr_now >= 0.0 && $pr_hold >= 0.0} {
            puts "=== POST-ROUTE PHYS_OPT: setup and hold closed ==="
            break
        }
        # Judge hold against WHERE WE STARTED, not against zero.  Routing can (and on the
        # 2026-09-15 200 MHz run did) leave WHS already negative -- so there was no
        # hold-clean state to roll back to, and a `$pr_hold < 0` test rejected EVERY pass,
        # including ones that left hold untouched: it discarded +0.213 ns of real setup
        # gain and still finished hold-broken.  A pass is only bad if it makes hold WORSE
        # than the netlist we started from.
        if {$pr_hold < $pr_whs0 - 0.001} {
            # A pass traded hold away.  Two things were wrong with simply breaking here:
            #   1. It STOPPED THE WHOLE CAMPAIGN on the first hold blip, so passes 2..N
            #      never ran -- the 2026-09-14 200 MHz build earned +0.291 ns on pass 1
            #      and then quit with 5 directives unused.
            #   2. It did NOT UNDO the offending pass, so the flow SHIPPED the
            #      hold-violating netlist anyway (that build: WHS -0.077, 70 failing hold
            #      endpoints) -- exactly the thing the comment above says corrupts data.
            # Correct behaviour: try to repair hold, keep the setup gain if repair works,
            # otherwise ROLL BACK to the last hold-clean checkpoint and continue trying
            # other directives from there.
            puts "=== POST-ROUTE PHYS_OPT: pass $pr_i drove HOLD negative (WHS $pr_hold); attempting repair ==="
            if {[catch {phys_opt_design -hold_fix} pr_hf_err]} {
                puts "WARN: phys_opt_design -hold_fix failed: $pr_hf_err"
            }
            set pr_hold [_pr_whs]; set pr_now [_pr_wns]
            # A successful repair is sufficient even if its positive hold
            # margin is smaller than the one before this optimization pass.
            if {$pr_now >= 0.0 && $pr_hold >= 0.0} {
                puts "=== POST-ROUTE PHYS_OPT: setup and hold closed after repair ==="
                break
            }
            if {$pr_hold < $pr_whs0 - 0.001} {
                puts [format "=== POST-ROUTE PHYS_OPT: hold repair FAILED (WHS %.3f); rolling back to last hold-clean checkpoint ===" $pr_hold]
                if {[file exists $pr_ckpt]} {
                    close_design
                    open_checkpoint $pr_ckpt
                    puts [format "=== POST-ROUTE PHYS_OPT: restored WNS %.3f WHS %.3f ===" [_pr_wns] [_pr_whs]]
                } else {
                    puts "=== POST-ROUTE PHYS_OPT: no checkpoint to restore; stopping with hold NEGATIVE (investigate) ==="
                    break
                }
                continue
            }
            puts [format "=== POST-ROUTE PHYS_OPT: hold repaired (WHS %.3f), WNS now %.3f; continuing ===" $pr_hold $pr_now]
        }
        if {$pr_now <= $pr_prev + 0.001} {
            # One directive plateau does not predict the next directive.
            # Preserve the best setup state if this pass regressed it, but
            # still try the remaining bounded schedule (including fanout).
            if {$pr_now < $pr_prev - 0.001 && [file exists $pr_ckpt]} {
                close_design
                open_checkpoint $pr_ckpt
                puts "=== POST-ROUTE PHYS_OPT: setup regressed; restored prior checkpoint ==="
            }
            puts "=== POST-ROUTE PHYS_OPT: no gain; trying next directive ==="
            continue
        }
        set pr_prev $pr_now
    }
    # Final hold repair can regress setup even without improving hold.
    # Preserve the current result and roll back on either timing regression
    # or tool failure. A failed checkpoint write must stop before mutation.
    if {[_pr_whs] < 0} {
        set pr_final_wns [_pr_wns]
        set pr_final_whs [_pr_whs]
        set pr_final_checkpoint [file join $output_dir checkpoints pre_final_hold_repair.dcp]
        write_checkpoint -force $pr_final_checkpoint
        puts [format "=== POST-ROUTE PHYS_OPT: final hold repair attempt (WHS %.3f) ===" $pr_final_whs]
        set pr_final_failed [catch {phys_opt_design -hold_fix} pr_fh_err]
        if {$pr_final_failed || [_pr_wns] < $pr_final_wns - 0.001 ||
                                [_pr_whs] < $pr_final_whs - 0.001} {
            puts "=== POST-ROUTE PHYS_OPT: final repair failed/regressed; restoring prior checkpoint ==="
            if {$pr_final_failed} {puts "WARN final hold_fix: $pr_fh_err"}
            close_design
            open_checkpoint $pr_final_checkpoint
        }
        puts [format "=== POST-ROUTE PHYS_OPT: after final repair WNS %.3f WHS %.3f ===" [_pr_wns] [_pr_whs]]
    }
    puts [format "=== POST-ROUTE PHYS_OPT done: WNS %.3f WHS %.3f after %d passes ===" [_pr_wns] [_pr_whs] $pr_i]
} else {
    puts "=== POST-ROUTE PHYS_OPT skipped: timing already met ==="
}

write_checkpoint -force $output_dir/checkpoints/route.dcp

# The SPI pads used to be false-pathed. Fail loudly if they disappear from
# analysis again, and preserve explicit reports for the physical IO budget.
foreach sd_output {sd_clk sd_mosi sd_cs_n} {
    set sd_paths [get_timing_paths -to [get_ports $sd_output] -delay_type max -max_paths 1]
    if {[llength $sd_paths] == 0} {error "SPI output $sd_output is not timed"}
    if {abs([get_property REQUIREMENT $sd_paths] - 4.0) > 0.001} {
        error "SPI output $sd_output lost its 4ns latency allocation"
    }
    puts "SPI_IO_BUDGET $sd_output slack=[get_property SLACK $sd_paths]"
}
set sd_input_paths [get_timing_paths -from [get_ports sd_miso] -delay_type max -max_paths 1]
if {[llength $sd_input_paths] == 0} {error "SPI input sd_miso is not timed"}
if {abs([get_property REQUIREMENT $sd_input_paths] - 3.0) > 0.001} {
    error "SPI input sd_miso lost its 3ns latency allocation"
}
puts "SPI_IO_BUDGET sd_miso slack=[get_property SLACK $sd_input_paths]"
report_timing -from [get_ports sd_miso] -delay_type max -max_paths 4 \
    -file $output_dir/reports/sd_miso_timing.rpt
report_timing -to [get_ports {sd_clk sd_mosi sd_cs_n}] -delay_type max -max_paths 6 \
    -file $output_dir/reports/sd_output_timing.rpt

# Incremental-compile reuse report + stash for the next iteration.  Vivado's
# report_incremental_reuse summarises how much placement+routing was carried
# over from the reference DCP — a per-impl indicator of how productive the
# incremental flow was for this change set.
if {!$no_incremental && $incremental_ref_dcp ne ""} {
    if {[file exists $incremental_ref_dcp]} {
        puts "=== INCREMENTAL REUSE REPORT ==="
        if {[catch {report_incremental_reuse \
                -file $output_dir/reports/incremental_reuse.rpt} reuse_err]} {
            puts "WARN: report_incremental_reuse failed: $reuse_err"
        } else {
            report_incremental_reuse
        }
    }
    # ONLY stash a reference that actually MET TIMING.
    #
    # This copy used to be unconditional, which made the incremental flow a
    # downward ratchet: a build that missed timing overwrote the reference,
    # so the NEXT incremental build started from a failing floorplan and was
    # biased to miss again, and so on.  Measured trajectory that exposed it
    # (2026-08-04): fabric_clk100 WNS 0.000 -> -0.173 -> -0.117 across three
    # consecutive builds whose RTL deltas were small and, in the last case,
    # confined to the LSU/MMU handshake while the violations sat in the MIG
    # DDR4 controller's own internals — i.e. placement drift, not logic.
    #
    # The comment on the "not found yet" branch above already promised
    # "will stash on success"; this makes that true.  A failing build now
    # leaves the previous good reference intact, so the next attempt still
    # starts from the best-known floorplan instead of the worst-recent one.
    # get_property on an empty path collection ERRORS rather than returning
    # "", so wrap the query: any failure means "we don't know", and not
    # knowing must not overwrite a good reference.
    set stash_ok 1
    if {[catch {
            set stash_wns [get_property SLACK \
                [get_timing_paths -delay_type max -max_paths 1]]
            set stash_whs [get_property SLACK \
                [get_timing_paths -delay_type min -max_paths 1]]
        } stash_err]} {
        set stash_ok 0
        puts "WARN: slack query failed ($stash_err)"
    }
    if {!$stash_ok || ![string is double -strict $stash_wns] \
                   || ![string is double -strict $stash_whs]} {
        puts "=== INCREMENTAL: slack unknown — NOT stashing (fail safe) ==="
    } elseif {$stash_wns < 0 || $stash_whs < 0} {
        puts [format \
            "=== INCREMENTAL: TIMING NOT MET (WNS %.3f, WHS %.3f) — reference left UNCHANGED ===" \
            $stash_wns $stash_whs]
        puts "===   (a failing routed DCP must never become the incremental reference)"
    } else {
        set ref_dir [file dirname $incremental_ref_dcp]
        if {![file exists $ref_dir]} {
            file mkdir $ref_dir
        }
        file copy -force $output_dir/checkpoints/route.dcp $incremental_ref_dcp
        puts [format \
            "=== INCREMENTAL: timing met (WNS %.3f, WHS %.3f) — stashed routed DCP to %s ===" \
            $stash_wns $stash_whs $incremental_ref_dcp]
    }
}

# ──────────────────────────────────────────────────────────────────────────────
# Final reports
# ──────────────────────────────────────────────────────────────────────────────
puts "=== FINAL REPORTS ==="
report_timing_summary \
    -max_paths 50 \
    -report_unconstrained \
    -file $output_dir/timing_summary.rpt

# POST-ROUTE timing under reports/, matching the naming convention of the
# other per-stage reports (reports/timing_synth.rpt, reports/timing_place.rpt).
#
# Added 2026-09-03.  The post-route summary was already being written -- but
# to $output_dir/timing_summary.rpt (plus the timestamped archive copy under
# synth/timing_reports/), NOT to reports/ and NOT under a timing_route name.
# Result: reports/ held timing_synth + timing_place and no route-stage peer,
# so anyone auditing a shipped build naturally read timing_place.rpt and drew
# conclusions from PLACE-stage numbers.  That is systematically wrong in both
# directions -- the router and post-route phys_opt close hold violations by
# construction and recover a large fraction of setup, and route can also
# introduce problems place never saw.
#
# Concrete instance of the harm (build/vivado_divfix_100mhz, 2026-09-03):
# reports/timing_place.rpt says WNS -0.368 ns / 16 failing setup endpoints
# and WHS -0.949 ns / 1986 failing hold endpoints, while the actual routed
# result for the same build is WNS +0.031 ns / WHS +0.010 ns / ZERO failing
# setup or hold endpoints.  An investigation started from the place report
# spent its time on 2002 endpoints that do not exist in the shipped
# bitstream.
report_timing_summary \
    -max_paths 10 \
    -file $output_dir/reports/timing_route.rpt

# Bus skew is checked INDEPENDENTLY of set_clock_groups -asynchronous and is
# NOT folded into report_timing_summary's headline WNS/WHS/TNS numbers, so a
# failing Gray-pointer skew bound would otherwise be invisible in every
# report this build emits.  That matters as of 2026-09-03: the async-FIFO
# Gray-pointer set_bus_skew pass (T6, above) had been applying to zero
# instances in every build; now that it actually applies to all of them, its
# result has to be recorded somewhere.
if {[catch {report_bus_skew -max_paths 10 \
        -file $output_dir/reports/bus_skew_route.rpt} bskew_err]} {
    puts "WARN: report_bus_skew failed: $bskew_err"
}

report_utilization \
    -hierarchical \
    -file $output_dir/reports/utilization_route.rpt

report_power \
    -file $output_dir/reports/power.rpt

report_drc \
    -file $output_dir/reports/drc.rpt

# Copy timing summary to archive with timestamp
set ts [clock format [clock seconds] -format "%Y%m%d_%H%M%S"]
file copy $output_dir/timing_summary.rpt \
          $proj_root/synth/timing_reports/timing_${ts}.rpt

# Print WNS to stdout for easy CI parsing
puts "=== WNS SUMMARY ==="
report_timing_summary -max_paths 1

# Bus-skew worst slack alongside WNS/WHS: it is a real, separately-checked
# constraint class (see reports/bus_skew_route.rpt above) that report_timing_summary
# does not include, so print it here or a CI gate reading only WNS will pass a
# build whose Gray-pointer CDC skew bound is violated.
puts "=== BUS SKEW SUMMARY ==="
if {[catch {report_bus_skew -max_paths 1} bskew_sum_err]} {
    puts "WARN: report_bus_skew (stdout) failed: $bskew_sum_err"
}

# ──────────────────────────────────────────────────────────────────────────────
# Bitstream.  Vivado will emit a bitstream even if timing is violated
# (it issues a critical warning).  Setting bitstream.general.compress is
# done in fpga_top.xdc; here we just write_bitstream.
# ──────────────────────────────────────────────────────────────────────────────
puts "=== WRITING BITSTREAM ==="
source $synth_dir/adb_firmware_mmi.tcl
validate_adb_firmware_bram
write_bitstream -force $output_dir/fpga_top.blank.bit
export_adb_firmware_bundle $output_dir/fpga_top.blank.bit

# Export the debug probe metadata (.ltx) for Vivado hw_manager.  The
# user's JTAG session loads both the bitstream and this file to pick
# up the 10 VIO probes.  If the VIO insertion failed earlier (e.g. a
# net couldn't be resolved) this call still emits a sparse .ltx — the
# dashboard TCL warns the user about missing probes.
write_debug_probes -force $output_dir/fpga_top.ltx

set host_debug_mode "none"
if {$enable_pcie_xdma} {
    set host_debug_mode "pcie_xdma"
} elseif {$enable_jtag_axi} {
    set host_debug_mode "jtag_axi"
}
set buildinfo_file [file join $output_dir fpga_top.buildinfo]
set buildinfo_fh [open $buildinfo_file w]
puts $buildinfo_fh "part=$part"
puts $buildinfo_fh "mode=$mode"
puts $buildinfo_fh "real_fpga_build=$real_fpga_build"
puts $buildinfo_fh "ddr_path=[expr {$use_sim_model ? {sim_model} : {real_mig}}]"
puts $buildinfo_fh "enable_vio=$enable_vio"
puts $buildinfo_fh "perf_detail_enable=$perf_detail_enable"
puts $buildinfo_fh "enable_ipc_ila=$enable_ipc_ila"
puts $buildinfo_fh "enable_jtag_axi=$enable_jtag_axi"
puts $buildinfo_fh "enable_pcie_xdma=$enable_pcie_xdma"
puts $buildinfo_fh "host_debug=$host_debug_mode"
# Whether the 53C96 trace ring is in this bitstream.  Without this line
# a `scsi-trace status` reading all zeros is ambiguous -- an absent ring
# and a genuinely-empty ring look identical over JTAG -- and answering it
# otherwise means grepping a routed netlist for the module.
puts $buildinfo_fh "enable_scsi_trace=$enable_scsi_trace"
puts $buildinfo_fh "target_freq_mhz=$target_freq_mhz"
puts $buildinfo_fh "core_clk_hz=$core_clk_hz"
puts $buildinfo_fh "video_smoke=$video_smoke"
puts $buildinfo_fh "boot_rom_sectors=$boot_rom_sectors"
puts $buildinfo_fh "sd_safe_cmd25=$sd_safe_cmd25"
# Record the memory-system cutover knobs.  These were previously absent,
# so there was NO way to tell from a shipped artifact whether a bitstream
# had the L2C in it -- answering that question on 2026-08-03 required
# grepping the routed timing report for `l2c` cells (0 hits, against 734
# for `scsi` as a positive control).  A buildinfo that omits the two
# knobs which most change memory-system behaviour is a record that looks
# authoritative while missing the thing you need.
puts $buildinfo_fh "l2c_enable=[expr {$l2c_enable_effective ? 1 : 0}]"
puts $buildinfo_fh "vram_in_ddr=[expr {$vram_in_ddr_effective ? 1 : 0}]"
puts $buildinfo_fh "l2c_uram_floorplan=[expr {($l2c_enable_effective && $l2c_uram_floorplan) ? 1 : 0}]"
puts $buildinfo_fh "cpu=$cpu_sel"
puts $buildinfo_fh "cpu_ipc_profile=$cpu_ipc_profile"
if {$cpu_m68k040} {
    puts $buildinfo_fh "cpu_git_revision=[string trim [exec git -C $cpu040_dir rev-parse HEAD]]"
}
puts $buildinfo_fh "eth_enable=$eth_enable"
puts $buildinfo_fh "eth_icmp_responder=$eth_icmp_responder"
puts $buildinfo_fh "eth_debug_enable=$eth_debug_enable"
puts $buildinfo_fh "build_id=$build_id"
# The artifact's own timing verdict, all three check classes.  Without these the
# only record of whether a shipped .bit met timing lives in a report file next to
# it, and when the flow crashes between route and reports (which it has) that
# record does not exist -- the timing then has to be re-derived by reopening the
# checkpoint.  WPWS is included because pulse-width is a separate class that
# report_timing_summary's WNS/WHS pair does not cover, and this design has had a
# real failing pulse-width endpoint inside the DDR4 PHY while WNS and WHS were
# both positive.
set bi_wns [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set bi_whs [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
puts $buildinfo_fh "wns=$bi_wns"
puts $buildinfo_fh "whs=$bi_whs"
set bi_pw_file [file join $output_dir reports pulse_width.rpt]
report_pulse_width -all_violators -file $bi_pw_file
set bi_wpws "unknown"
set bi_pw_fails "unknown"
if {[catch {
        set bi_fh [open $bi_pw_file r]; set bi_txt [read $bi_fh]; close $bi_fh
        set bi_worst ""
        set bi_n 0
        foreach bi_line [split $bi_txt "\n"] {
            # Data rows carry a numeric slack in column 7 of the check table.
            if {[regexp {^(Min Skew|Max Skew|Min Period|Max Period|Low Pulse Width|High Pulse Width)\s+\S+\s+\S+\s+\S+\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s} $bi_line -> bi_ct bi_req bi_act bi_slk]} {
                if {$bi_worst eq "" || $bi_slk < $bi_worst} { set bi_worst $bi_slk }
                if {$bi_slk < 0} { incr bi_n }
            }
        }
        if {$bi_worst ne ""} { set bi_wpws $bi_worst; set bi_pw_fails $bi_n }
    } bi_err]} {
    puts "WARN: could not extract WPWS for buildinfo: $bi_err"
}
puts $buildinfo_fh "wpws=$bi_wpws"
puts $buildinfo_fh "pulse_width_failing_endpoints=$bi_pw_fails"
puts "=== TIMING VERDICT: WNS=$bi_wns WHS=$bi_whs WPWS=$bi_wpws pulse_width_failures=$bi_pw_fails ==="
puts $buildinfo_fh "bitstream=fpga_top.blank.bit"
puts $buildinfo_fh "adb_firmware=absent_requires_local_patch"
puts $buildinfo_fh "debug_probes=fpga_top.ltx"
puts $buildinfo_fh "generated_utc=[clock format [clock seconds] -gmt 1 -format {%Y-%m-%dT%H:%M:%SZ}]"
close $buildinfo_fh

puts "=== IMPLEMENTATION COMPLETE ==="
puts "Route checkpoint: $output_dir/checkpoints/route.dcp"
puts "Bitstream:        $output_dir/fpga_top.blank.bit"
puts "Debug probes:     $output_dir/fpga_top.ltx"
puts "Build info:       $buildinfo_file"
puts "Open in GUI:  vivado $output_dir/checkpoints/route.dcp"
puts "DO NOT PROGRAM THE BLANK IMAGE. Insert your ADB dump first; see docs/adb_firmware_bitstream.md."
puts "Then explicitly program the resulting fpga_top.local.bit with the matching fpga_top.ltx."
