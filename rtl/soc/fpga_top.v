// fpga_top.v — Real-hardware SoC top level for Vivado impl.
//
// This repo has no monorepo rtl/core/* tree any more (moved to the cpu/
// git submodule — see CPU_M68K_SRCS in the Makefile's "CPU build select"
// section).  `make sim` (the old flat `mac_top` Verilator top) is retired;
// the live Verilator sim path for this file is `make tb-fpga-top-rom`
// (CPU=m68k pulls cpu/rtl/core/* in alongside this file).
//
// ════════════════════════════════════════════════════════════════════════
// Topology (bring-up phase — several stubs, flagged inline)
// ════════════════════════════════════════════════════════════════════════
//
// AXI xbar is 3M x 6S (2026-07-16 master-count reduction — see
// fpga_top_xbar.vh and axi_xbar.v's header "M0/boot merge"):
//
//               ┌──────────────────────────────────────────────┐
//               │               m68k_core                       │
//               │   if_addr/req/rdata/rvalid      daxi_*        │
//               └──────────┬──────────────────────────┬─────────┘
//                          ▼                          ▼
//                 ┌────────────────┐           ┌──────────────────┐
//                 │  if_to_axi     │           │ axi_narrow_to_   │
//                 │ (128-bit RD)   │           │ wide (32↔128)    │
//                 └──────┬─────────┘           └─────┬────────────┘
//                        │                           │
//                        ▼ (M2, CPU IF, read-only)   ▼ (M0, CPU LSU)
//                                  ┌─────────┐
//                         ┌──M1 ──►│         │  (host debug: JTAG AXI
//                         │        │         │   when enabled, else stub)
//                         │        │  xbar   │
//                         │  M0 ──►│  3M×6S  │◄── boot_fsm (m0b sub-port:
//                         │        │         │     cpu_rst-selected mux
//                         │  M2 ──►│         │     onto M0, NOT a separate
//                         │        │         │     master — boot_fsm only
//                         │        └─┬─┬─┬─┬─┘     drives while CPU held
//                         │          │ │ │ │        in reset)
//                         │      S0 ─┘ │ │ └─ S3/S4/S5
//                         │      │     │ └─ S2 (DMA config; master port
//                         │      ▼     │      stubbed, see fpga_top_dma.vh)
//                         │   ddr_ctrl │
//                         │   (SIM_MODEL / real MIG)
//                         │            └─ S1 peripheral_bus (pb_clk)
//                         │
//                         │      S3 ── VRAM pixel aperture
//                         │      S4 ── DAFB register shim
//                         │      S5 ── SD JTAG writer
//                         │
//                         └──► host debug AXI (JTAG-to-AXI for first-board
//                              ROM patch/load, XDMA later; see
//                              docs/debug_pcie.md prince pattern)
//
//                         ┌──────────┐   ┌───────────┐   ┌──────────┐
//                         │  vram    │──►│ video_top │──►│  HDMI    │
//                         │ (URAM)   │   │(linebuf_  │   │ (AL9134) │
//                         │          │   │ scanout)  │   │          │
//                         └──────────┘   └───────────┘   └──────────┘
//                    (VRAM AXI slave is xbar S3 since task #147:
//                     CPU writes to 0xF900_0000..0xF91F_FFFF (2 MB,
//                     matches Q700 silicon) travel M0 -> xbar -> S3 ->
//                     vram URAM.  When VIDEO_SMOKE=1 the reset-time
//                     vram_smoke writer owns the slave until it's done.)
//
//                     ┌───────────┐          ┌────────────┐
//                     │  boot_fsm │───prov───│ sd_spi_mux │──► sd_spi ──► SD
//                     └─────┬─────┘          └────────────┘
//                           │
//                           ▼ AXI WR (via narrow→wide, m0b sub-port)
//                        xbar M0
//
// ════════════════════════════════════════════════════════════════════════
// Clocking tree
// ════════════════════════════════════════════════════════════════════════
//
//   SIM_MODEL:
//     sys_clk_p/n (200 MHz diff) ──► IBUFDS ──► sys_clk (200 MHz SE)
//                                       │
//                                       ├──► clk_rst ──► core_clk
//                                       ├──► clk_rst ──► pb_clk
//                                       └──► video_top MMCM ──► pclk
//
//   Real MIG:
//     sys_clk_p/n (200 MHz diff, T24/U24) ──► MIG c0_sys_clk_p/n only
//     fabric_clk_p/n (100 MHz MGTREFCLK0, AB7/AB6)
//         └──► IBUFDS_GTE4.ODIV2 ──► BUFG_GT ──► video_ref_clk (100 MHz)
//                                  ├──► BUFG_GT / CORE_CLK_DIVIDE
//                                                   ──► clk_rst reset pipe
//                                                   ──► core_clk
//                                  └──► BUFG_GT /2 ──► clk_rst reset pipe
//                                                   ──► pb_clk
//     video_ref_clk ──► video_top MMCM ──► pclk
//
//   The real-MIG path deliberately does not buffer T24/U24 in fpga_top:
//   those pins belong to the MIG black box.  AB7/AB6 are the same legal
//   fabric/utility clock pair used by ~/FPGA/pcie_test's util_ds_buf
//   (IBUFDS_GTE4, not a normal fabric IBUFDS).  ODIV2 cannot legally drive
//   fabric MMCM/BUFGCE_DIV loads directly on KU5P; it must enter fabric
//   through BUFG_GT first.
//
//   core_rst is released by clk_rst after cpu_resetn and ddr_cal_done are
//   both high and four core-clock edges have elapsed.  cpu_rst stays gated
//   until the ROM is ready and for a further 50 ms after the peripheral
//   island reset has released.  Normal builds use boot_rom_loaded from the
//   SD loader; JTAG can also hold the boot FSM in reset, let the host
//   populate the ROM window, then release the CPU from VIO without rebuilding.
//
// ════════════════════════════════════════════════════════════════════════
// Stubs / TODOs (all called out in the orchestrator handoff)
// ════════════════════════════════════════════════════════════════════════
// - Host debug AXI (M1): optional JTAG-to-AXI master for first-board ROM
//   patch/load iteration.  A gated XDMA skeleton can instead drive M1 via
//   the XDMA M_AXI_BYPASS port when PCIE_XDMA_ENABLE is defined; see
//   docs/debug_pcie.md.
// - CPU instruction fetch uses xbar M2 as an independent read-only master
//   (non-negotiable — never merge with CPU LSU / M0, per the 2026-07-16
//   master-count-reduction task).  The real HDMI path reads URAM directly
//   (see docs/memhier.md §"VRAM placement") and does not consume an xbar
//   master slot.
// - Boot FSM is time-multiplexed onto xbar M0 (CPU LSU) via a
//   cpu_rst-selected mux inside axi_xbar.v — see its header note
//   "M0/boot merge" (2026-07-16).  DMA's AXI4 master (was M4) is
//   stubbed out of the crossbar entirely — see fpga_top_dma.vh — pending
//   a real consumer; its AXI-Lite config slave (xbar S2) stays live.
// - VRAM AXI slave: wired to xbar S3 (task #147, "xbar-vram-wire").
//   CPU writes to the 0xF900_0000..0xF91F_FFFF aperture (2 MB, matches
//   Q700 silicon) flow M0 → xbar → S3 → vram.  The reset-time
//   VIDEO_SMOKE writer — when VIDEO_SMOKE
//   is enabled — is pre-multiplexed onto the vram slave via vram_smoke
//   because this bring-up phase does not yet share the slave port with
//   two masters simultaneously.  A future retune may promote smoke to
//   an xbar master; for now, enabling VIDEO_SMOKE effectively disables
//   the CPU→VRAM path (smoke owns the slave until it's done).
// - IRQ aggregator: `cpu_ipl[2:0]` tied to 0 — no peripherals inject
//   IRQs this round.  See docs/exceptions.md §"IRQ story".  Follow-up
//   ticket: `irq-aggregator`.
// - MIG: generic synth/impl can still use SIM_MODEL for tiny smoke images,
//   but real FPGA bitstream targets set USE_REAL_MIG=1 and instantiate the
//   pcie_test DDR4 MIG black box through ddr_ctrl.  Vivado stitches the OOC
//   MIG DCP during synth/impl; no real-hardware target should define
//   SIM_MODEL.
//
// Ready for the first 50 MHz board-test pass:
//   - SIM_MODEL: `TARGET_FREQ_MHZ=50 CORE_CLK_DIVIDE=4 make clock-report`
//   - real MIG:  `USE_REAL_MIG=1 TARGET_FREQ_MHZ=50 CORE_CLK_DIVIDE=2 make clock-report`
// Full impl still waits on the MIG integration gate.

`default_nettype none
`include "axi_defs.vh"

// NOTE: SIM_MODEL is passed via `synth_design -verilog_define SIM_MODEL`
// only for generic/tiny-smoke builds.  Real FPGA targets set
// REAL_FPGA_BUILD=1 USE_REAL_MIG=1, which makes synth/vivado.tcl refuse
// any accidental SIM_MODEL define.

// ═══════════════════════════════════════════════════════════════════════
// debug_stop_manager USED TO BE DEFINED HERE, and must not come back.
//
// It was an inline VERBATIM COPY of cpu/rtl/core/debug/debug_stop_manager.v
// whose header asked the next person to keep the two byte-identical when
// bumping the cpu/ submodule.  Nothing enforced that and nothing could
// notice it failing: synth/vivado.tcl and the Makefile's CPU_M68K_SRCS
// glob both SKIPPED the cpu/ file as a "module-name collision guard", so
// this copy was what every bitstream elaborated, while `make tb-debug-stop`
// validated it too — leaving the cpu/ copy, and the only tests that could
// see the difference, entirely out of the SoC's view.
//
// The copies did diverge, for three weeks, over two CPU commits
// (7fdcba29 2026-07-28, 0052e492 2026-08-06) carrying four halt-path
// fixes — most importantly restricting `dbg_precise_stop_req` to the
// double-fault source.  That overlay drives the UNREGISTERED `flush_en`
// in m68k_core_rename.vh while the SYNCHRONOUS flush network that
// actually clears ROB/IQ entries is driven from `commit_flush_en` alone
// (see the CONTRACT block at m68k_core_rename.vh:505-530).  Firing it on
// a registered halt decision masks µops that are still in the issue
// queues and F1 while their ROB entries survive — a permanent wedge.
//
// Deleted 2026-08-19.  cpu/rtl/core/debug/debug_stop_manager.v is now the
// single definition, both collision guards are gone, and `make
// tb-debug-stop` builds the cpu/ module against the cpu/ testbench.  Do
// not re-inline it: only the CPU instantiates it (m68k_axi_wrapper.v:574),
// so CPU=stub builds do not need it here.
// ═══════════════════════════════════════════════════════════════════════

module fpga_top #(
    // CORE_CLK_HZ tracks the active core clock after clk_rst.  Default
    // stays at 200 MHz passthrough; the first 50 MHz board test should
    // set CORE_CLK_DIVIDE=4 and CORE_CLK_HZ=50_000_000 together.
    parameter integer CORE_CLK_HZ = 200_000_000,
    parameter integer CORE_CLK_DIVIDE = 1,
    // Peripheral island clock.  Hardware keeps this fixed at 50 MHz:
    // SIM_MODEL divides 200 MHz by four, real-MIG divides the 100 MHz
    // AB7/AB6 fabric reference by two.
    parameter integer PB_CLK_HZ = 50_000_000,
    // Number of SD sectors copied by boot_fsm before CPU reset is
    // released.  Default is the full 1 MiB Quadra 700 ROM; first-board
    // checkerboard/small-ROM smoke images can override this to a short
    // sector count without editing boot_fsm or the ROM boot harness.
    parameter [31:0] BOOT_ROM_SECTORS = 32'd2048,
    // Opt-in: reset-safe staged CMD25 writes. Adds first-sector staging
    // latency; leave disabled until real-card throughput justifies it.
    parameter integer SD_SAFE_CMD25 = 0,
    // MAME's Quadra 700 model derives VIA clocks from C7M/10:
    // 31.3344 MHz / 4 / 10 = 783.36 kHz.
    parameter integer VIA_PHI2_HZ = 783_360,
    // ── Video smoke mode ─────────────────────────────────────────────
    // When VIDEO_SMOKE = 1, a reset-time AXI writer preloads VRAM with
    // SMPTE-style colour bars.  Use this to validate the VRAM preload
    // path before the CPU / ROM boot is healthy enough to paint.
    // When VIDEO_SMOKE = 0 the smoke writer is NOT
    // instantiated — the VRAM AXI slave ports are tied off exactly as
    // before.  Set at synth time (e.g. vivado tcl: set_param
    // general.maxThreads 8; set_property generic VIDEO_SMOKE=1 [current_fileset]).
    // See docs/video_smoke.md.
    parameter VIDEO_SMOKE = 0,
    // Phase 1 keeps the proven ARP/ICMP endpoint available as a physical-link
    // regression image.  Phase 3 changes this only after the packet adapter
    // exists; it must not make SONIC's current CR_TXP placeholder live.
    parameter ETH_ICMP_RESPONDER = 1,
    // Build identity — set at synth time from vivado.tcl (git_sha or
    // clock-based fallback).  Plumbed down to debug_ctrl so each
    // bitstream's identity is verifiable via `r 0x50900004` from JTAG.
    parameter [31:0] BUILD_ID = 32'h0000_0000
) (
    // ── DDR4 reference / SIM_MODEL system clock (200 MHz diff) ───────
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    input  wire        cpu_resetn,       // active-low board reset

`ifndef SIM_MODEL
    // ── Real-MIG fabric clock (100 MHz MGTREFCLK0 diff, AB7/AB6) ─────
    input  wire        fabric_clk_p,
    input  wire        fabric_clk_n,
`endif

    // ── LEDs / buttons for bring-up visibility ───────────────────────
    output wire [3:0]  led,
    input  wire [3:0]  btn,

    // ── Board UART routed to selected SCC channel ────────────────────
    input  wire        uart_rtl_0_rxd,
    output wire        uart_rtl_0_txd,

    // ── SD card (SPI) ────────────────────────────────────────────────
    output wire        sd_clk,
    output wire        sd_mosi,
    input  wire        sd_miso,
    output wire        sd_cs_n,

    // ── HDMI (AL9134) ────────────────────────────────────────────────
    inout  wire        al9134_scl,
    inout  wire        al9134_sda,
    output wire [23:0] al9134_d,
    output wire        al9134_hs,
    output wire        al9134_vs,
    output wire        al9134_de,
    output wire        al9134_clk,
    output wire        al9134_resetn,
    input  wire        al9134_int,

    // ── Σ-Δ PWM audio (AN9134 NC pins J1.35 / J1.36) ─────────────────
    // Drive an off-board class-D speaker amp / RC-filter line-out.
    // Tied off when AUDIO_PATH != AUDIO_PATH_PWM in fpga_top_peripherals.vh.
    // Pin assignments live in synth/audio_pwm.xdc.
    output wire        pwm_audio_l,
    output wire        pwm_audio_r

`ifdef ETH_ENABLE
    ,
    // ── RTL8211F RGMII Ethernet (Phase 1 physical-link regression) ──
    input  wire        phy_rx_clk,
    input  wire [3:0]  phy_rxd,
    input  wire        phy_rx_ctl,
    output wire        phy_tx_clk,
    output wire [3:0]  phy_txd,
    output wire        phy_tx_ctl
`endif

`ifndef SIM_MODEL
    ,
    // ── DDR4 MIG physical interface (real-hardware build only) ───────
    output wire [16:0] ddr4_adr,
    output wire [1:0]  ddr4_ba,
    output wire [0:0]  ddr4_bg,
    output wire [0:0]  ddr4_cke,
    output wire [0:0]  ddr4_odt,
    output wire [0:0]  ddr4_cs_n,
    output wire [0:0]  ddr4_ck_t,
    output wire [0:0]  ddr4_ck_c,
    output wire        ddr4_reset_n,
    output wire        ddr4_act_n,
    inout  wire [31:0] ddr4_dq,
    inout  wire [3:0]  ddr4_dqs_t,
    inout  wire [3:0]  ddr4_dqs_c,
    inout  wire [3:0]  ddr4_dm_dbi_n
`ifdef PCIE_XDMA_ENABLE
    ,
    // ── PCIe XDMA host/debug interface (real-hardware build only) ───
    // PERST# note: the slot's PERST# arrives on T19, the same physical
    // pin the base design already uses as cpu_resetn (fpga_top.xdc).
    // There is no separate pcie_perstn port — the XDMA sys_rst_n is fed
    // from cpu_resetn inside fpga_top_debug_host.vh, so a host-driven
    // PERST# also cold-resets the SoC (correct add-in-card semantics).
    input  wire [3:0]  pcie_rxp,
    input  wire [3:0]  pcie_rxn,
    output wire [3:0]  pcie_txp,
    output wire [3:0]  pcie_txn
`endif
`endif
);
    // ─────────────────────────────────────────────────────────────────
    // fpga_top body is split across include files. Each file is a slice of
    // the original module body and is included in original source order.
    // Cross-file wires are declared in the earlier include that first uses
    // them; later includes see them normally. No functional change.
    // ─────────────────────────────────────────────────────────────────

`include "fpga_top_clocks.vh"
`include "fpga_top_ethernet.vh"
`include "fpga_top_cpu.vh"
`include "fpga_top_boot_master.vh"
`include "fpga_top_debug_host.vh"
`include "fpga_top_dma.vh"
`include "fpga_top_xbar.vh"
`include "fpga_top_ddr.vh"
`include "fpga_top_peripherals.vh"
`include "fpga_top_debug_ctrl.vh"
`include "fpga_top_sd.vh"
`include "fpga_top_video.vh"
`include "fpga_top_debug_vio.vh"

endmodule

`default_nettype wire
