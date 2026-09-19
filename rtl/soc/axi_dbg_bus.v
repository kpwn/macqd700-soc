// axi_dbg_bus.v — THE DEBUG BUS.  One JTAG-AXI bridge lands here; debug-window
// addresses are served locally, everything else is MASTERED onto the SoC bus.
//
// ┌─ WHAT THIS IS ─────────────────────────────────────────────────────────┐
//
//        JTAG-AXI          <-- exactly ONE bridge, one hw_axi core
//            |
//            v  s_*
//     ┌───────────────┐
//     │  axi_dbg_bus  │ ── d_*  ──► DEBUG-BUS LOCAL SLAVES
//     │ (this module) │             (CPU debug CSRs @ 0x5090_0000, eth dbg regs)
//     └───────┬───────┘             core_clk, NOT in the SoC reset domain
//             │ m_*
//             ▼
//        SoC BUS (axi_narrow_to_wide -> axi_xbar M1)
//             DDR / peripheral bus / VRAM / DAFB / DMA cfg
//
// This is the "two busses, the debug one able to master the SoC one, and just
// one axi jtag bridge" topology (owner, 2026-09-12), expressed as a 1x2
// address-decoding interconnect at the master.  It is NOT the dual-BSCAN
// arrangement that was reverted in 330ad925 -- that revert removed a SECOND
// `debug_jtag_axi` IP instance (52166283), not this module.  There is still
// exactly one JTAG-AXI bridge, one hw_axi core, and one host address space.
//
// ┌─ WHY IT EXISTS: two measured defects ──────────────────────────────────┐
//
// (1) THE INSTRUMENT DIED WITH THE PATIENT.  A host debug-register access used
//     to travel:
//
//       debug_jtag_axi (core_clk)
//         -> axi_narrow_to_wide        (shared, bounded outstanding)
//         -> axi_xbar M1               (shared with the CPU)
//         -> S1 / peripheral_bus       (pb_clk)  <-- stalls when a peripheral
//                                                    stops making progress
//         -> axil_async_bridge         (pb_clk -> core_clk)
//         -> cpu dbg_axi               (core_clk)
//
//     MEASURED 2026-09-06 (p150, build 0xE934ECC7): with the peripheral-bus ack
//     watchdog disabled, a stalled SCSI beat wedged the fabric and every
//     JTAG-AXI read failed -- debug CSR, DRAM and low RAM alike -- with
//     `halt-status` returning the poison value 0xBADA0BAD.  Total loss of
//     observability at the moment of the hang.
//
// (2) THE RESET STRANDED ITS OWN WRITE.  A JTAG `reset` IS a write to
//     DBG_CONTROL, i.e. a write into the 0x5090_0000 window -- which, on the
//     old path above, was a write THROUGH S1.  The soc_full_rst it triggers
//     swallowed that write's BRESP, and S1 is deliberately outside the
//     crossbar's flush domain, so `ws_flush_abort` could never release the
//     slot.  MEASURED on hardware 2026-09-12 via `vio_boot_diag`:
//       running:      0x01000800 -> xbar_s1_slot_busy = 0
//       after `reset`:0x80800800 -> xbar_s1_slot_busy = 1
//     with `xbar_slv_poisoned = 0x00` in BOTH states (so every fix premised on
//     the poison latches, including a93451a6, was aimed at a thing that never
//     fires).  That is a topology defect, not a logic bug: no watchdog tuning
//     can fix "the reset kills the path carrying the reset request".
//
// Routing the window here removes both at once.  Both ends are already on
// core_clk, so the old core->pb->core detour is DELETED rather than replaced:
// the new path is strictly shorter as well as unwedgeable, and the reset
// request never touches the fabric it resets.
//
// ┌─ CONTRACT ─────────────────────────────────────────────────────────────┐
//   * Whole transactions are routed by their AW/AR address; a burst never
//     splits across the two destinations.
//   * Read and write directions are independent.
//   * ⚠️ THE TWO BRANCHES ARE INDEPENDENT — one outstanding transaction
//     PER BRANCH, not per direction.  This is load-bearing and was got
//     WRONG first time: a single shared per-direction latch meant a
//     system transaction that never completes (exactly what a wedged
//     peripheral produces) held the latch set forever, s_arready/
//     s_awready went low permanently, and DEBUG accesses were blocked BY
//     THIS MODULE — reintroducing the head-of-line blocking it exists to
//     remove.  Caught on hardware 2026-09-07 (p152): one blocked
//     `r 0x50F0F040` killed every subsequent halt-status.
//     The original justification — "matches what the narrow-to-wide
//     adapter already enforced" — was precisely backwards: that
//     adapter's single-outstanding behaviour IS the p150 bug.
//   * B and R prefer the DEBUG branch when both have a response
//     pending, so a stuck system access can never starve the escape
//     hatch.
//   * The debug branch is a pure register file with no arbitration behind
//     it: it cannot wedge, which is the entire point.
//
// The system branch is bit-identical to the pre-split behaviour whenever
// the address is outside the debug window, so DRAM/peripheral access
// semantics are unchanged.
//
// RESET DOMAIN.  Drive `rst` from `core_rst`, NEVER from `soc_full_rst`.  The
// whole point is that this module and its local slaves sit UPSTREAM of the SoC
// reset: the host must be able to assert, observe and release a SoC reset
// through a path that reset does not touch.  See docs/bus_debug_split_plan.md.

`default_nettype none

module axi_dbg_bus #(
    parameter integer ID_WIDTH   = 4,
    parameter integer ADDR_WIDTH = 32,
    parameter integer DATA_WIDTH = 32,
    // Debug window base/mask, matched as (addr & MASK) == BASE.
    // Default is the historical 0x5090_0000 1 MiB debug window.
    parameter [31:0]  DBG_BASE   = 32'h5090_0000,
    parameter [31:0]  DBG_MASK   = 32'hFFF0_0000
) (
    input  wire                        clk,
    input  wire                        rst,

    // ── Slave: the JTAG-AXI master ──────────────────────────────────
    input  wire [ID_WIDTH-1:0]         s_awid,
    input  wire [ADDR_WIDTH-1:0]       s_awaddr,
    input  wire [7:0]                  s_awlen,
    input  wire [2:0]                  s_awsize,
    input  wire [1:0]                  s_awburst,
    input  wire                        s_awvalid,
    output wire                        s_awready,
    input  wire [DATA_WIDTH-1:0]       s_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]   s_wstrb,
    input  wire                        s_wlast,
    input  wire                        s_wvalid,
    output wire                        s_wready,
    output wire [ID_WIDTH-1:0]         s_bid,
    output wire [1:0]                  s_bresp,
    output wire                        s_bvalid,
    input  wire                        s_bready,
    input  wire [ID_WIDTH-1:0]         s_arid,
    input  wire [ADDR_WIDTH-1:0]       s_araddr,
    input  wire [7:0]                  s_arlen,
    input  wire [2:0]                  s_arsize,
    input  wire [1:0]                  s_arburst,
    input  wire                        s_arvalid,
    output wire                        s_arready,
    output wire [ID_WIDTH-1:0]         s_rid,
    output wire [DATA_WIDTH-1:0]       s_rdata,
    output wire [1:0]                  s_rresp,
    output wire                        s_rlast,
    output wire                        s_rvalid,
    input  wire                        s_rready,

    // ── Master 0: system path (narrow_to_wide -> xbar), unchanged ───
    output wire [ID_WIDTH-1:0]         m_awid,
    output wire [ADDR_WIDTH-1:0]       m_awaddr,
    output wire [7:0]                  m_awlen,
    output wire [2:0]                  m_awsize,
    output wire [1:0]                  m_awburst,
    output wire                        m_awvalid,
    input  wire                        m_awready,
    output wire [DATA_WIDTH-1:0]       m_wdata,
    output wire [(DATA_WIDTH/8)-1:0]   m_wstrb,
    output wire                        m_wlast,
    output wire                        m_wvalid,
    input  wire                        m_wready,
    input  wire [ID_WIDTH-1:0]         m_bid,
    input  wire [1:0]                  m_bresp,
    input  wire                        m_bvalid,
    output wire                        m_bready,
    output wire [ID_WIDTH-1:0]         m_arid,
    output wire [ADDR_WIDTH-1:0]       m_araddr,
    output wire [7:0]                  m_arlen,
    output wire [2:0]                  m_arsize,
    output wire [1:0]                  m_arburst,
    output wire                        m_arvalid,
    input  wire                        m_arready,
    input  wire [ID_WIDTH-1:0]         m_rid,
    input  wire [DATA_WIDTH-1:0]       m_rdata,
    input  wire [1:0]                  m_rresp,
    input  wire                        m_rlast,
    input  wire                        m_rvalid,
    output wire                        m_rready,

    // ── Master 1: PRIVATE debug path (direct to the CPU dbg slave) ──
    output wire [ID_WIDTH-1:0]         d_awid,
    output wire [ADDR_WIDTH-1:0]       d_awaddr,
    output wire [7:0]                  d_awlen,
    output wire [2:0]                  d_awsize,
    output wire [1:0]                  d_awburst,
    output wire                        d_awvalid,
    input  wire                        d_awready,
    output wire [DATA_WIDTH-1:0]       d_wdata,
    output wire [(DATA_WIDTH/8)-1:0]   d_wstrb,
    output wire                        d_wlast,
    output wire                        d_wvalid,
    input  wire                        d_wready,
    input  wire [ID_WIDTH-1:0]         d_bid,
    input  wire [1:0]                  d_bresp,
    input  wire                        d_bvalid,
    output wire                        d_bready,
    output wire [ID_WIDTH-1:0]         d_arid,
    output wire [ADDR_WIDTH-1:0]       d_araddr,
    output wire [7:0]                  d_arlen,
    output wire [2:0]                  d_arsize,
    output wire [1:0]                  d_arburst,
    output wire                        d_arvalid,
    input  wire                        d_arready,
    input  wire [ID_WIDTH-1:0]         d_rid,
    input  wire [DATA_WIDTH-1:0]       d_rdata,
    input  wire [1:0]                  d_rresp,
    input  wire                        d_rlast,
    input  wire                        d_rvalid,
    output wire                        d_rready,

    // Observability: which branch currently owns each direction.
    output wire                        dbg_wr_active,
    output wire                        dbg_rd_active
);

    // ── Address decode ──────────────────────────────────────────────
    // SINGLE-BEAT ONLY on the debug branch.  The local slaves behind d_*
    // are AXI-LITE (the CPU's dbg_axi CSR block, the eth debug regs): one
    // AW, one W, one B.  Hand them a burst and they would take the AW and
    // then see W beats they have no AW for.
    //
    // Before the debug window moved here it reached the CPU through the
    // crossbar, and a burst was HANDLED THERE, not by an AXI-Lite slave.
    // Routing a burst DOWN THE SYSTEM BRANCH restores that exactly, with
    // no state and no new response path in this module: the transaction
    // goes out m_*, through axi_narrow_to_wide, and decodes to
    // XBAR_SLV_IO like it always did.
    //
    // Be precise about what "handled there" means, because it is not one
    // behaviour.  axi_narrow_to_wide packs narrow beats into 128-bit ones
    // (w_awlen = (addr[3:2] + n_awlen) >> 2), so:
    //   * <= 4 narrow beats from a 16B-aligned address arrive at the
    //     crossbar as a SINGLE wide beat.  It is forwarded to S1 and
    //     peripheral_bus writes the addressed register from its lane; the
    //     other words are dropped.  Mangled, but answered -- and that is
    //     what this path has always done.
    //   * longer bursts arrive with awlen > 0 onto a lite-only slave and
    //     ARE rejected with a local SLVERR (is_lite_only_slv() includes
    //     XBAR_SLV_IO -- axi_xbar.v, "Burst legalization").
    // Either way the transaction gets a response and no AXI-Lite slave
    // ever sees W beats it has no AW for, which is the property that
    // matters here.
    //
    // Nothing legitimately bursts into this window: the REPL's debug-CSR
    // accesses are single words (`create_hw_axi_txn ... -len 1`), and the
    // only burst users -- dump-mem/rd_burst into DRAM and the SD staging
    // BRAM -- are outside it.
    wire aw_is_dbg = ((s_awaddr & DBG_MASK) == DBG_BASE) && (s_awlen == 8'd0);
    wire ar_is_dbg = ((s_araddr & DBG_MASK) == DBG_BASE) && (s_arlen == 8'd0);

    // ── Write direction ─────────────────────────────────────────────
    // wr_busy latches the destination for the whole AW+W+B sequence so a
    // burst cannot split, and so B is returned to the right requester.
    // ⚠️ PER-BRANCH state, NOT a shared busy flag.  A single shared
    // "one outstanding" latch was the original design here and it was
    // WRONG: a system transaction that never completes (exactly what a
    // wedged peripheral produces) left the shared latch set forever, so
    // s_awready/s_arready went low permanently and DEBUG accesses were
    // blocked BY THIS MODULE -- reintroducing the very head-of-line
    // blocking it exists to remove.  Caught on hardware 2026-09-07
    // (p152): a blocked `r 0x50F0F040` killed every later halt-status.
    // The branches must be independent so a stuck system access cannot
    // reach the debug path.
    reg wr_busy_sys, wr_busy_dbg;
    wire aw_fire = s_awvalid && s_awready;

    always @(posedge clk) begin
        if (rst) begin
            wr_busy_sys <= 1'b0;
            wr_busy_dbg <= 1'b0;
        end else begin
            if (aw_fire &&  aw_is_dbg) wr_busy_dbg <= 1'b1;
            else if (d_bvalid && d_bready) wr_busy_dbg <= 1'b0;
            if (aw_fire && !aw_is_dbg) wr_busy_sys <= 1'b1;
            else if (m_bvalid && m_bready) wr_busy_sys <= 1'b0;
        end
    end

    // Accept an AW only if THAT branch is free.
    assign d_awvalid = s_awvalid &&  aw_is_dbg && !wr_busy_dbg;
    assign m_awvalid = s_awvalid && !aw_is_dbg && !wr_busy_sys;
    assign s_awready = aw_is_dbg ? (!wr_busy_dbg && d_awready)
                                 : (!wr_busy_sys && m_awready);

    assign d_awid = s_awid;  assign d_awaddr = s_awaddr; assign d_awlen = s_awlen;
    assign d_awsize = s_awsize; assign d_awburst = s_awburst;
    assign m_awid = s_awid;  assign m_awaddr = s_awaddr; assign m_awlen = s_awlen;
    assign m_awsize = s_awsize; assign m_awburst = s_awburst;

    // W follows the branch that owns an in-flight write.  Debug wins if
    // both are somehow live: it is the escape hatch and must never be
    // starved by a stuck system write.
    wire w_to_dbg = wr_busy_dbg;
    assign d_wvalid = s_wvalid &&  w_to_dbg;
    assign m_wvalid = s_wvalid && !w_to_dbg && wr_busy_sys;
    assign s_wready = w_to_dbg ? d_wready : (wr_busy_sys && m_wready);
    assign d_wdata = s_wdata; assign d_wstrb = s_wstrb; assign d_wlast = s_wlast;
    assign m_wdata = s_wdata; assign m_wstrb = s_wstrb; assign m_wlast = s_wlast;

    // B: prefer the debug branch so a wedged system write cannot hide it.
    assign s_bvalid = d_bvalid ? 1'b1 : m_bvalid;
    assign s_bresp  = d_bvalid ? d_bresp : m_bresp;
    assign s_bid    = d_bvalid ? d_bid   : m_bid;
    assign d_bready = d_bvalid && s_bready;
    assign m_bready = !d_bvalid && s_bready;

    // ── Read direction ──────────────────────────────────────────────
    reg rd_busy_sys, rd_busy_dbg;
    wire ar_fire = s_arvalid && s_arready;

    always @(posedge clk) begin
        if (rst) begin
            rd_busy_sys <= 1'b0;
            rd_busy_dbg <= 1'b0;
        end else begin
            if (ar_fire &&  ar_is_dbg) rd_busy_dbg <= 1'b1;
            else if (d_rvalid && d_rready && d_rlast) rd_busy_dbg <= 1'b0;
            if (ar_fire && !ar_is_dbg) rd_busy_sys <= 1'b1;
            else if (m_rvalid && m_rready && m_rlast) rd_busy_sys <= 1'b0;
        end
    end

    assign d_arvalid = s_arvalid &&  ar_is_dbg && !rd_busy_dbg;
    assign m_arvalid = s_arvalid && !ar_is_dbg && !rd_busy_sys;
    assign s_arready = ar_is_dbg ? (!rd_busy_dbg && d_arready)
                                 : (!rd_busy_sys && m_arready);

    assign d_arid = s_arid;  assign d_araddr = s_araddr; assign d_arlen = s_arlen;
    assign d_arsize = s_arsize; assign d_arburst = s_arburst;
    assign m_arid = s_arid;  assign m_araddr = s_araddr; assign m_arlen = s_arlen;
    assign m_arsize = s_arsize; assign m_arburst = s_arburst;

    // R: debug has priority, for the same reason as B.
    assign s_rvalid = d_rvalid ? 1'b1 : m_rvalid;
    assign s_rdata  = d_rvalid ? d_rdata : m_rdata;
    assign s_rresp  = d_rvalid ? d_rresp : m_rresp;
    assign s_rlast  = d_rvalid ? d_rlast : m_rlast;
    assign s_rid    = d_rvalid ? d_rid   : m_rid;
    assign d_rready = d_rvalid && s_rready;
    assign m_rready = !d_rvalid && s_rready;

    assign dbg_wr_active = wr_busy_dbg;
    assign dbg_rd_active = rd_busy_dbg;

endmodule

`default_nettype wire
