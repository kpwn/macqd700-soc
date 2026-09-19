// rtl/soc/cpu_socket.vh — CPU↔SoC socket contract (AUTHORITATIVE)
//
// This header is the single source of truth for the seam between the
// macqd700 SoC and whatever CPU plugs into it.  A DIFFERENT CPU must be
// able to drop in by speaking ONLY this socket — no CPU-specific (m68k)
// signal crosses the boundary.
//
// The seam is THREE AXI interfaces + an IRQ pair + a small SoC-fabric
// control group + clk/rst:
//
//   * axi_i_*    — CPU instruction-read MASTER (AR/R only) at AXI_I_DW.
//   * axi_d_*    — CPU data read/write MASTER (AW/W/B/AR/R) at AXI_D_DW,
//                  wstrb = AXI_D_DW/8.  AXI_I_DW and AXI_D_DW are
//                  independent per-master widths (task #266 / SOC-1) —
//                  they are NOT required to match.
//   * dbg_axi_*  — CPU-EXPORTED debug/control AXI(-Lite) SLAVE.  This is
//                  the WHOLE debug + control register map the CPU
//                  presents: build_id, live PC / arch state, halt /
//                  continue / step, breakpoints, arch-capture,
//                  dcache/icache probe + ops, ILA cfg, AND reset/halt
//                  via the debug-CSR.  The SoC's JTAG-AXI master routes
//                  its debug window (the historical 0x5090_0000 region)
//                  to this slave.  None of the ~130 discrete m68k debug
//                  taps cross the socket any more — they are private to
//                  the CPU repo behind this slave.
//
// All the m68k-specific glue (if_stage→axi_i adapter, 32→AXI_DW data
// widen, debug_ctrl + its ~130 taps + the ILA hub) lives CPU-side
// (Phase 4).  For the standalone SoC build, cpu_stub.v presents this
// socket with every master idle and the dbg_axi slave terminating OKAY.
//
// ─────────────────────────────────────────────────────────────────────
// SoC-fabric control group (CPU → SoC outputs)
// ─────────────────────────────────────────────────────────────────────
// A small set of platform-control signals the SoC fabric genuinely
// needs and that the CPU's debug-CSR (reached via dbg_axi) owns.  These
// REPLACE the discrete debug_ctrl reset/window outputs the SoC reset
// tree + xbar used to consume:
//
//   output        cpu_cold_reset_pulse  unified-reset trigger (1-cycle;
//                                        was DBG_CONTROL bit 5 →
//                                        dbg_cold_reset_pulse).  Feeds the
//                                        SoC dbg_rst_src_level edge detector.
//   output        cpu_cold_reset_hold   sticky CPU-hold level (was
//                                        DBG_CONTROL bit 4 →
//                                        dbg_cold_reset_hold).  ORs into
//                                        the SoC cpu_rst_or hold path.
//   output [5:0]  cpu_ram_window_lg2    DDR RAM-window size select
//                                        (was dbg_ram_window_lg2 → xbar).
//   output [6:0]  cpu_mon_sense         DAFB monitor-sense code, runtime
//                                        settable over JTAG (debug-CSR
//                                        OFF_MON_SENSE, 0x0005C).  Bit 6 =
//                                        "extended monitor" flag, bits
//                                        [5:0] = code; ALL SEVEN bits must
//                                        reach the SoC's video.v, which
//                                        interprets them in
//                                        sense_response().  Was a
//                                        compile-time MONITOR_TYPE
//                                        parameter; POR default 7'h06
//                                        preserves that behaviour, and the
//                                        value survives a CPU reset (it
//                                        lives in the debug reset domain)
//                                        so the operator can set a sense
//                                        code and then reset into it.
//   output        cpu_peripheral_reset  the 68k RESET instruction's
//                                        external indication (real-chip
//                                        semantics: RESET is
//                                        output-only, resets attached
//                                        peripherals, never the CPU
//                                        itself -- see the project's
//                                        2026-08-18 RESET-instruction
//                                        decision).  Already implemented
//                                        on the wrapper (v1:
//                                        m68k_axi_wrapper.v:140/683,
//                                        `.cpu_reset_out(...)`) and
//                                        consumed by the SoC
//                                        (fpga_top_debug_ctrl.vh:202,
//                                        fpga_top_sd.vh:80 `reset_req`)
//                                        -- SOC-2 (cpu040 design doc
//                                        §9.4/§11) is this doc catching
//                                        up to an already-shipping
//                                        signal, not new RTL.
//
// The CPU also CONSUMES the SoC's init-done (DDR cal) so its debug-CSR
// can gate boot:
//   input         init_done_seen        SoC reports DDR calibration done.
//
// ─────────────────────────────────────────────────────────────────────
// Socket parameters
// ─────────────────────────────────────────────────────────────────────
//   CPU_SOCKET_AXI_I_DW : axi_i (instruction-fetch master) data width.
//                       256.  The 040-OoO core ("v2") fetches natively
//                       at 256b (len=1/size=5, two 32B beats = one 64B
//                       line) and declined to build a 256->128
//                       downconverter CPU-side (cpu040 design doc
//                       2026-08-18-axi-socket-adapter-design.md D11);
//                       v1's if_to_axi.v instead presented 128b here.
//                       task #266 / SOC-1 split this out of the single
//                       shared CPU_SOCKET_AXI_DW below so a v2-shaped
//                       CPU and a v1-shaped CPU can each declare their
//                       own axi_i width without one distorting the
//                       other.  See §11 of that doc for the fabric-side
//                       companion work this still needs behind
//                       XBAR_M_CPUI (not part of this header).
//   CPU_SOCKET_AXI_D_DW : axi_d (data read/write master) data width.
//                       128.  Matches today's xbar fabric on both v1
//                       and v2.
//   CPU_SOCKET_AXI_AW : address width (flat 32-bit physical, post-MMU).
//   CPU_SOCKET_AXI_IW : AXI ID width (xbar needs distinct I vs D IDs).
//   CPU_SOCKET_DBG_AW : dbg_axi(-Lite) address width.  20 bits = the
//                       existing 0x5090_0000 debug window (1 MiB).
//   CPU_SOCKET_DBG_DW : dbg_axi(-Lite) data width = 32.
`ifndef CPU_SOCKET_PARAMS
`define CPU_SOCKET_PARAMS
  `ifndef CPU_SOCKET_AXI_I_DW
    `define CPU_SOCKET_AXI_I_DW 256
  `endif
  `ifndef CPU_SOCKET_AXI_D_DW
    `define CPU_SOCKET_AXI_D_DW 128
  `endif
  `define CPU_SOCKET_AXI_AW 32
  `define CPU_SOCKET_AXI_IW 4
  `define CPU_SOCKET_DBG_AW 20
  `define CPU_SOCKET_DBG_DW 32
`endif

// ─────────────────────────────────────────────────────────────────────
// Socket port groups (canonical reference — cpu_stub.v gives the
// concrete Verilog port declarations of the Phase-3 standalone stub;
// the Phase-4 m68k wrapper presents EXACTLY this same surface).
// ─────────────────────────────────────────────────────────────────────
//
// 1) Clock / reset
//    input   clk                         core clock (= core_clk in SoC)
//    input   rst                         synchronous, active-high core reset
//
// 2) Instruction read master  axi_i_*   (AR/R only — fetch never writes)
//    at AXI_I_DW (256 — task #266 / SOC-1; NOT AXI_D_DW)
//    output [AXI_IW-1:0]     axi_i_arid
//    output [AXI_AW-1:0]     axi_i_araddr
//    output [7:0]            axi_i_arlen
//    output [2:0]            axi_i_arsize
//    output [1:0]            axi_i_arburst
//    output                  axi_i_arvalid
//    input                   axi_i_arready
//    input  [AXI_IW-1:0]     axi_i_rid
//    input  [AXI_I_DW-1:0]   axi_i_rdata
//    input  [1:0]            axi_i_rresp
//    input                   axi_i_rlast
//    input                   axi_i_rvalid
//    output                  axi_i_rready
//
// 3) Data read/write master  axi_d_*    (full AW/W/B/AR/R at AXI_D_DW)
//    output [AXI_IW-1:0]     axi_d_awid
//    output [AXI_AW-1:0]     axi_d_awaddr
//    output [7:0]            axi_d_awlen
//    output [2:0]            axi_d_awsize
//    output [1:0]            axi_d_awburst
//    output                  axi_d_awvalid
//    input                   axi_d_awready
//    output [AXI_D_DW-1:0]   axi_d_wdata
//    output [AXI_D_DW/8-1:0] axi_d_wstrb
//    output                  axi_d_wlast
//    output                  axi_d_wvalid
//    input                   axi_d_wready
//    input  [AXI_IW-1:0]     axi_d_bid
//    input  [1:0]            axi_d_bresp
//    input                   axi_d_bvalid
//    output                  axi_d_bready
//    output [AXI_IW-1:0]     axi_d_arid
//    output [AXI_AW-1:0]     axi_d_araddr
//    output [7:0]            axi_d_arlen
//    output [2:0]            axi_d_arsize
//    output [1:0]            axi_d_arburst
//    output                  axi_d_arvalid
//    input                   axi_d_arready
//    input  [AXI_IW-1:0]     axi_d_rid
//    input  [AXI_D_DW-1:0]   axi_d_rdata
//    input  [1:0]            axi_d_rresp
//    input                   axi_d_rlast
//    input                   axi_d_rvalid
//    output                  axi_d_rready
//
// 4) Debug/control AXI(-Lite) SLAVE  dbg_axi_*  (32b data, 20b addr)
//    input  [DBG_AW-1:0]   dbg_axi_awaddr
//    input                 dbg_axi_awvalid
//    output                dbg_axi_awready
//    input  [DBG_DW-1:0]   dbg_axi_wdata
//    input  [DBG_DW/8-1:0] dbg_axi_wstrb
//    input                 dbg_axi_wvalid
//    output                dbg_axi_wready
//    output [1:0]          dbg_axi_bresp
//    output                dbg_axi_bvalid
//    input                 dbg_axi_bready
//    input  [DBG_AW-1:0]   dbg_axi_araddr
//    input                 dbg_axi_arvalid
//    output                dbg_axi_arready
//    output [DBG_DW-1:0]   dbg_axi_rdata
//    output [1:0]          dbg_axi_rresp
//    output                dbg_axi_rvalid
//    input                 dbg_axi_rready
//
// 5) Interrupt seam
//    input  [2:0]  cpu_ipl     prioritised level from peripheral irq_agg
//                              (0 = none, 1-6 = level, 7 = NMI)
//    output        ipl_ack     1-cycle pulse when CPU dispatches an IRQ
//                              exception (clears irq_agg NMI edge latch)
//
// 6) SoC-fabric control group (CPU → SoC; see top-of-file rationale)
//    output        cpu_cold_reset_pulse  unified-reset trigger (1-cycle)
//    output        cpu_cold_reset_hold   sticky CPU-hold level
//    output [5:0]  cpu_ram_window_lg2    DDR RAM-window size select
//    output [6:0]  cpu_mon_sense         DAFB monitor-sense code (bit 6 =
//                                        extended-monitor flag)
//    output        cpu_peripheral_reset  RESET instruction's external
//                                        indication (SOC-2; already
//                                        shipping, see top-of-file entry)
//    input         init_done_seen        SoC reports DDR cal done
//
// 7) ILA-only debug-export group — DELIBERATE EXCEPTION to the "no
//    CPU-specific signal crosses the socket" rule above.  Exists ONLY
//    when `ILA_ENABLE` is defined (i.e. `make impl ENABLE_ILA=1`); the
//    default build's port list is unaffected.  A polled dbg_axi
//    register read cannot give a real-time Vivado ILA the cycle-
//    accurate commit/PRF-write/A7 taps a HW bring-up bisect needs, so
//    this group promotes ~46 named `dbg_ila_*_w` wires (commit-bundle,
//    ROB pop/retire, PRF write-port snoop, true-A7/snap-mux-bypass,
//    IF/BPU PC, the A7/RTE-finalize group added 2026-07-04, and the v7
//    rte_acc[0]/ROB-completion-staleness group added 2026-07-04 for
//    task #89) as real
//    output ports on whichever module binds `u_cpu`.  Both
//    m68k_axi_wrapper.v (real CPU) and cpu_stub.v (standalone
//    placeholder, tied to constant 0) declare an IDENTICAL
//    `ifdef ILA_ENABLE`-gated port list so the fpga_top instantiation's
//    single shared port-connection list binds cleanly either way.  See
//    m68k_axi_wrapper.v's own port-list comment for the full signal
//    list, and synth/debug_ila.tcl / docs/ila_a7_drift_probes.md in the
//    SoC repo for the probe map that consumes them.
