// rtl/soc/cpu_socket.vh — CPU↔SoC socket contract (AUTHORITATIVE)
//
// This header is the single source of truth for the seam between the
// macqd700 SoC and whatever CPU plugs into it.  A DIFFERENT CPU must be
// able to drop in by speaking ONLY this socket — no CPU-specific (m68k)
// signal crosses the boundary.
//
// The seam has three baseline AXI interfaces, an optional P6 D-side hot-read
// interface, an IRQ pair and a small SoC-fabric
// control group + clk/rst:
//
//   * axi_i_*    — CPU instruction-read MASTER (AR/R only) at AXI_I_DW.
//   * axi_d_*    — CPU data read/write MASTER (AW/W/B/AR/R) at AXI_D_DW,
//                  wstrb = AXI_D_DW/8.  AXI_I_DW and AXI_D_DW are
//                  independent per-master widths (task #266 / SOC-1) —
//                  they are NOT required to match.
//   * axi_dh_*   — optional P6 D-side hot-read MASTER (AR/R only), 128 b,
//                  2-bit ID, present when CPU_AXI_DH=1 / L2C_DH_PORT.
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
//
// ── axi_dh (P6, the D-side hot door) -- optional socket group
//    (CPU side: flag CPU_AXI_DH=1; core ports axi_dh_ar*/axi_dh_r*
//    bind here to dha_ar*/dha_r*). CPU and SoC must enable the group
//    together; the Makefile and Vivado generator check that coupling.
//
//    Shape, as l2c.v's dh_axi_* port already expects it:
//      CPU_SOCKET_AXI_DH_DW = 128   (native, no decompose in l2c.v)
//      CPU_SOCKET_AXI_DH_IW = 2     (4 IDs = AxiIds.dRefill(0..3))
//      AR/R only -- read-only.  The D side's writes (copyback writebacks,
//      precise and cache-inhibited stores) stay on axi_d and the crossbar.
//      INCR and WRAP; WRAP is what lets a 64 B line fill be issued
//      demanded-sector-first as one burst.
//
//    Fabric side (rtl/soc/fpga_top_ddr.vh, behind `L2C_DH_PORT`): the
//    guard+door use `dha_ar*`/`dha_r*`, `CPU_AXI_DH_IW` beside
//    `CPU_AXI_IW` in fpga_top_cpu.vh, and `dha_rready_gated` with the
//    same JTAG-CPU-reset drain policy as `ifa_rready_gated`.
//
//    Core side, and these are HARD prerequisites, not niceties:
//      * a cache-inhibited access is a full barrier BOTH WAYS, launched
//        only at ROB head under dQuiesce (no MSHR valid, no outstanding
//        cold write, SQ drained of older stores), with no younger LS op
//        launching until it completes.  The fabric contributes no
//        cross-path ordering between the doors and does not need to;
//      * a per-line writeback-address gate: an AR for line X is held while
//        X sits in the L1D writeback buffer awaiting its B.  This is the
//        L1D analogue of l2c_victim's query_hit and it is the ONE
//        cross-path hazard the split creates.  What the fabric DOES
//        guarantee is that a line whose dirty copy is still in L2's victim
//        buffer is never answered from DRAM, and that once a cold write has
//        been accepted at the front door a subsequently accepted hot read
//        of the same set observes it.
//      * the routing key is the MMU cache mode on the D-cache command
//        (missCmode in {COPYBACK, WRITETHROUGH} -> hot), NEVER an address
//        decode.  The fabric's window guard is what makes a wrong
//        cacheable answer fail loudly instead of reading aliased DRAM.
//
//    See docs/l2c_spec.md "The hot/cold memory split (P6, DH_PORT_EN)".
//
// ── PLANNED, NOT YET PART OF THIS HEADER: axi_dh AW/W/B (P6W, the hot
//    door's WRITE half).  Same status and same two-repo rule as above.
//
//    Shape, as l2c.v's dw_axi_* port expects it (behind the write guard):
//      128 b data, 16 b strobes, CPU_SOCKET_AXI_DH_IW (2) ID bits -- the
//      write ID space is independent of the read one, as in AXI.
//      INCR only, awsize = 4.  awlen 0 (one 16 B L1D line) today; 3 for a
//      64 B sector writeback later.
//      bready MUST be held high (a refused B stalls the whole L2 front door).
//      AW may run ahead of W; W beats follow AW order; B is in order per ID.
//
//    What may go on it: CACHEABLE writes only -- COPYBACK dirty-victim
//    writebacks and full-line (noFill) store installs.  Writethrough stores
//    and every cache-inhibited access stay on axi_d.  Routing key: the
//    cache mode the line was allocated under, never an address decode; the
//    fabric guard answers RAM -> L2, ROM mirror -> dropped with OKAY (as the
//    crossbar drops a CPU ROM write), anything else -> DECERR.
//
//    What B means: the write has been accepted by L2's lookup pipeline and
//    EVERY read accepted afterwards, on any door, observes it.  Nothing
//    earlier (AW or W handshake) means that, and AW/W/B are NOT ordered
//    against axi_dh AR/R.  Core-side rules, all HARD:
//      * WB gate, extended: hold a read of line X -- an axi_dh AR, or a
//        cold AR on axi_d -- while any write of X, on axi_dh OR axi_d, has
//        been issued (or is queued to issue) and has no B.  Same CAM as the
//        P6 gate, over the WB buffer and the WT FIFO.
//      * No two writes of one line in flight on DIFFERENT doors (L2 orders
//        same-line writes by front-door accept order, which is not issue
//        order across doors).  Same-door same-line writes are ordered.
//      * D4: busQuiesced additionally requires no axi_dh write without its
//        B; while an inhibited access is outstanding, no axi_dh AW issues.
//        A CPUSH/CPUSHL push that goes out on this port completes (B) before
//        a following inhibited access launches -- so device DMA reading
//        memory through the crossbar sees the pushed data in L2.
//      * On a core reset nothing is needed on this port: the SoC guard
//        (abandon = cpu_rst) pads an abandoned burst with no-op beats and
//        swallows Bs owed to the dead core before it accepts a new AW.
//
//    Fabric side (fpga_top_ddr.vh, behind `L2C_DW_PORT`, which requires
//    `L2C_DH_PORT`): waits on `dha_aw*`/`dha_w*`/`dha_b*` and a
//    `dha_bready_gated` twin of `dha_rready_gated`.
//    See docs/l2c_spec.md "The hot door's write half (P6W, DW_PORT_EN)".
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
  // P6 hot door axi_dh (present on the CPU only when it is built with
  // CPU_AXI_DH=1; the SoC binds it only under `L2C_DH_PORT`, which the
  // Makefile/vivado.tcl define from the SAME knob and cross-check against
  // the generated M68kSocketTop.v -- see the Makefile's CPU_AXI_DH).
  `define CPU_SOCKET_AXI_DH_DW 128
  `define CPU_SOCKET_AXI_DH_IW 2
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
