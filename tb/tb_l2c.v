// tb_l2c.v -- thin Verilator-top wrapper around rtl/soc/l2c.v.
//
// Fixes the bypass-window test parameters at build time (a 1 MB window
// at 0x0100_0000, immediately after -- and disjoint from -- the 16 MB
// cacheable region the C++ harness in tb/tb_l2c.cpp exercises at
// 0x0000_0000+; kept adjacent rather than far away so the harness's
// host-side backing/golden arrays stay a modest ~17 MB) and forwards
// L2_BYPASS_ALL so the Makefile can build two variants: the normal
// active-cache model (tb-l2c) and a pass-through-only model
// (tb-l2c-bypass-all) for the L2_BYPASS_ALL equivalence check.
//
// Port names are a flat pass-through of l2c.v's AXI4 slave/master ports
// so the C++ harness can drive/observe them directly via Verilator's
// generated accessors.

`default_nettype none

module tb_l2c #(
    parameter L2_BYPASS_ALL = 0,
    // Overridable so the victim-buffer depth sweep in docs/l2c_perf.md can
    // be reproduced without editing RTL: verilator -GVICTIM_SLOTS=n.
    parameter VICTIM_SLOTS = 8,
    // Same, for the bypass-engine depth sweep (docs/l2c_perf.md S14),
    // overridden with a -GBYPASS_SLOTS=n build flag.  BYPASS_SLOTS=1
    // reproduces the pre-2026-08-20 serialized engine, which is the
    // sweep's reference point -- see tb-l2c-bypdepth-* in the Makefile.
    parameter BYPASS_SLOTS = 8,
    // ── P6: the D-side hot door ────────────────────────────────────────
    // DH_EN forwards to l2c's DH_PORT_EN.  Default 0, so `make tb-l2c` is
    // the OFF configuration and its output is the regression baseline for
    // "the OFF netlist/behaviour is unchanged".  `make tb-l2c-dh` builds
    // -GDH_EN=1 and is the only thing that runs the dh scenarios.
    parameter DH_EN = 0,
    parameter DH_R_FALL_THROUGH = 0,
    // Native ID width of the hot door: 2 = 4 IDs = AxiIds.dRefill(0..3).
    parameter DH_ID_WIDTH = 2,
    // Cold-path fairness bound; 0 = pure fixed priority.  The arbitration
    // tests sweep this.
    parameter DH_COLD_FAIR_N = 64,
    // P6W: the hot door's WRITE half (l2c DW_PORT_EN).  Requires DH_EN.
    parameter DW_EN = 0,
    parameter DW_ID_WIDTH = 2
) (
    input  wire clk, rst,
    input  wire [5:0]  s_axi_awid, input wire [31:0] s_axi_awaddr,
    input  wire [7:0]  s_axi_awlen, input wire [2:0] s_axi_awsize, input wire [1:0] s_axi_awburst,
    input  wire s_axi_awvalid, output wire s_axi_awready,
    input  wire [127:0] s_axi_wdata, input wire [15:0] s_axi_wstrb,
    input  wire s_axi_wlast, input wire s_axi_wvalid, output wire s_axi_wready,
    output wire [5:0] s_axi_bid, output wire [1:0] s_axi_bresp,
    output wire s_axi_bvalid, input wire s_axi_bready,
    input  wire [5:0] s_axi_arid, input wire [31:0] s_axi_araddr,
    input  wire [7:0] s_axi_arlen, input wire [2:0] s_axi_arsize, input wire [1:0] s_axi_arburst,
    input  wire s_axi_arvalid, output wire s_axi_arready,
    output wire [5:0] s_axi_rid, output wire [127:0] s_axi_rdata,
    output wire [1:0] s_axi_rresp, output wire s_axi_rlast, output wire s_axi_rvalid, input wire s_axi_rready,
    // Dedicated 256-bit fetch source.  This was previously omitted from
    // the unit wrapper, leaving the shipping dual-source front door wholly
    // unexercised by tb-l2c.
    input  wire [3:0] f_axi_arid, input wire [31:0] f_axi_araddr,
    input  wire [7:0] f_axi_arlen, input wire [2:0] f_axi_arsize,
    input  wire [1:0] f_axi_arburst, input wire f_axi_arvalid,
    output wire f_axi_arready,
    output wire [3:0] f_axi_rid, output wire [255:0] f_axi_rdata,
    output wire [1:0] f_axi_rresp, output wire f_axi_rlast,
    output wire f_axi_rvalid, input wire f_axi_rready,
    // ── P6: the hot D door, AS THE SoC WIRES IT ────────────────────────
    // These are the guard's UPSTREAM pins, not l2c's.  ifetch_window_guard
    // sits between them and l2c.dh_axi_*, exactly as fpga_top_ddr.vh puts
    // it between the CPU's axi_i and l2c.f_axi_* -- so the tb exercises the
    // composition that ships, and a test can prove a non-memory address is
    // refused at the fabric rather than served from (addr mod DRAM size).
    input  wire [DH_ID_WIDTH-1:0] dh_axi_arid, input wire [31:0] dh_axi_araddr,
    input  wire [7:0] dh_axi_arlen, input wire [2:0] dh_axi_arsize,
    input  wire [1:0] dh_axi_arburst, input wire dh_axi_arvalid,
    output wire dh_axi_arready,
    output wire [DH_ID_WIDTH-1:0] dh_axi_rid, output wire [127:0] dh_axi_rdata,
    output wire [1:0] dh_axi_rresp, output wire dh_axi_rlast,
    output wire dh_axi_rvalid, input wire dh_axi_rready,
    // Guard configuration + observability.  dh_guard_fault_count is the
    // tb-visible `OFF_DH_GUARD_HITS` the design doc asks for.
    input  wire [5:0] dh_ram_window_lg2,
    input  wire       dh_ar_is_rom,
    input  wire       dh_overlay_active,   // stand-in for the crossbar's reset ROM overlay
    output wire       dh_guard_fault_sticky,
    output wire [31:0] dh_guard_fault_addr,
    output wire [15:0] dh_guard_fault_count,
    // Compile-time configuration, readable from C++ so a scenario can skip
    // itself in the OFF build instead of silently passing.
    output wire       dbg_dh_en,
    // P6W: hot write door, driven at the WRITE GUARD's upstream pins.
    input  wire [DW_ID_WIDTH-1:0] dw_axi_awid, input wire [31:0] dw_axi_awaddr,
    input  wire [7:0] dw_axi_awlen, input wire [2:0] dw_axi_awsize,
    input  wire [1:0] dw_axi_awburst, input wire dw_axi_awvalid,
    output wire dw_axi_awready,
    input  wire [127:0] dw_axi_wdata, input wire [15:0] dw_axi_wstrb,
    input  wire dw_axi_wlast, input wire dw_axi_wvalid, output wire dw_axi_wready,
    output wire [DW_ID_WIDTH-1:0] dw_axi_bid, output wire [1:0] dw_axi_bresp,
    output wire dw_axi_bvalid, input wire dw_axi_bready,
    input  wire       dw_aw_is_rom,
    input  wire       dw_abandon,
    output wire [15:0] dw_guard_fault_count,
    output wire [15:0] dw_guard_drop_count,
    output wire       dbg_dw_en,
    output wire       dbg_acc_dw,
    output wire       dbg_dw_aw_have,
    output wire       dbg_hot_avail,
    output wire       dbg_dw_over_gfull,
    output wire [15:0] dbg_dh_cold_fair_n,
    output wire [5:0] m_axi_awid, output wire [31:0] m_axi_awaddr,
    output wire [7:0] m_axi_awlen, output wire [2:0] m_axi_awsize, output wire [1:0] m_axi_awburst,
    output wire m_axi_awvalid, input wire m_axi_awready,
    output wire [127:0] m_axi_wdata, output wire [15:0] m_axi_wstrb,
    output wire m_axi_wlast, output wire m_axi_wvalid, input wire m_axi_wready,
    input  wire [5:0] m_axi_bid, input wire [1:0] m_axi_bresp,
    input  wire m_axi_bvalid, output wire m_axi_bready,
    output wire [5:0] m_axi_arid, output wire [31:0] m_axi_araddr,
    output wire [7:0] m_axi_arlen, output wire [2:0] m_axi_arsize, output wire [1:0] m_axi_arburst,
    output wire m_axi_arvalid, input wire m_axi_arready,
    input  wire [5:0] m_axi_rid, input wire [127:0] m_axi_rdata,
    input  wire [1:0] m_axi_rresp, input wire m_axi_rlast, input wire m_axi_rvalid, output wire m_axi_rready,
    output wire [3:0] dbg_mshr_occupancy,
    // ── Eviction-pressure observability (sim-only hierarchical taps) ──────
    // dbg_vb_occ      -- l2c_victim slot occupancy, 0..SLOTS.
    // dbg_vb_slots    -- that module's SLOTS parameter, so the C++ side's
    //                    self-check ("evict_stall implies the buffer is
    //                    FULL") stays right when the depth changes.
    // dbg_evict_stall -- a request sitting in S_LOOKUP that would dispatch
    //                    its miss RIGHT NOW if only the victim buffer had a
    //                    free slot.  This is the exact cycle count eager
    //                    writeback is supposed to remove, so it is measured
    //                    rather than argued about.
    output wire [4:0] dbg_vb_occ,
    output wire [4:0] dbg_vb_slots,
    output wire       dbg_evict_stall,
    // dbg_vb_drain_busy -- at least one writeback is in flight or waiting
    // to start.  Since the engine was pipelined (2026-08-20) the sequencer
    // itself is idle most of a saturated run, so "the FSM is not S_IDLE" no
    // longer means what it used to; drain_busy_c is the honest successor.
    // dbg_vb_out        -- writebacks with an AW accepted and no B yet, i.e.
    //                      how much of the pipelining is actually being used.
    // dbg_vb_sink       -- of dbg_vb_out, how many are PRE-RESET strays whose
    //                      slots no longer exist (l2c_victim's sink_cnt).
    //                      Needed so the "outstanding <= occupancy"
    //                      invariant below stays true across a reset.
    // dbg_vb_seq_busy   -- the AW/W sequencer has something to send this
    //                      cycle.  This, NOT dbg_vb_drain_busy, is the
    //                      master-write-port occupancy: it is what a
    //                      background cleaner would have to compete for.
    output wire       dbg_vb_drain_busy,
    output wire       dbg_vb_seq_busy,
    output wire [4:0] dbg_vb_out,
    output wire [4:0] dbg_vb_sink,
    // dbg_vb_wb_err     -- sim-only count of writeback BRESPs that came back
    //                      non-OKAY.  A writeback has no requester to notify
    //                      (docs/l2c_spec.md Minor-16), so this counter is
    //                      the only way a test can assert the error was
    //                      surfaced rather than silently dropped.
    output wire [7:0] dbg_vb_wb_err,
    // ── Bypass-engine observability (2026-08-20 pipelining) ──────────────
    // dbg_by_slots    -- l2c_bypass's SLOTS parameter, as a wire, so the
    //                    C++ side's depth assertions do not have to
    //                    hierarchically reference a parameter.
    // dbg_by_occ      -- slots accepted from the front door but whose
    //                    response has not yet been delivered upstream.
    // dbg_by_inflight -- transactions PRESENTED to DRAM and still awaiting
    //                    their R/B.  This is the number the whole rewrite
    //                    is about: it was structurally <= 1 before, so a
    //                    test that watches it reach N is a direct check
    //                    that the pipelining exists rather than an
    //                    indirect inference from a cycle count.
    output wire [7:0] dbg_by_slots,
    output wire [5:0] dbg_by_occ,
    output wire [5:0] dbg_by_inflight,
    // ── FRONT-DOOR occupancy taps (2026-08-20, front-door depth study) ────
    // The question these answer: l2c_ctrl has exactly ONE aw_have/ar_have
    // register set, so it holds one AW burst and one AR burst at a time and
    // cannot latch the next header until the current burst's last beat has
    // been consumed into the tag pipeline.  Does that shape actually cost
    // throughput, given that the tag pipeline behind it is a 3-cycle
    // S_IDLE -> S_WAIT -> S_LOOKUP loop that can only start one beat every
    // three cycles anyway?
    //
    // dbg_fd_ar_stall / dbg_fd_aw_stall -- the master is presenting a
    //     header this cycle and the door refuses it.  This is the raw
    //     "cycles stalled on s_arready/s_awready low" the study asks for.
    //     NOTE it is NOT by itself a loss: a refused header costs nothing
    //     if the pipeline had no free accept slot to give it anyway.
    // dbg_fd_w_stall  -- same for the write-data channel.
    // dbg_fd_idle     -- the tag pipeline is at its accept point (S_IDLE,
    //     reset walk done).  This is the denominator: the number of
    //     opportunities to start a beat that physically exist.
    // dbg_fd_accept   -- an opportunity taken (cache path or bypass path).
    // dbg_fd_gather   -- a full-line-write beat swallowed by the flw
    //     gather.  Productive work that is deliberately NOT an accept, so
    //     it must be excluded from the bubble count or the gather looks
    //     like a stall.
    // The remaining three classify a WASTED opportunity (idle & !accept):
    // dbg_fd_nowork   -- the door held no beat to offer (!any_sel).  With a
    //     master that presents its next header the cycle after the previous
    //     one is taken, this is the ONLY class the door's depth can fix.
    // dbg_fd_idbusy   -- a beat was available but its AXI ID still has a
    //     live MSHR/bypass op (Critical-3's id_busy_c).  A deeper door
    //     cannot help; the next beat behind it has the same ID.
    // dbg_fd_bypnr    -- a beat was available, classified bypass, and the
    //     bypass engine was not ready.
    output wire dbg_fd_ar_stall,
    output wire dbg_fd_aw_stall,
    output wire dbg_fd_w_stall,
    output wire dbg_fd_idle,
    output wire dbg_fd_accept,
    output wire dbg_fd_gather,
    output wire dbg_fd_nowork,
    output wire dbg_fd_idbusy,
    output wire dbg_fd_bypnr,
    // dbg_fd_sethaz  -- a cache-bound beat was available but refused
    //                   because an older pipeline stage holds a set that
    //                   conflicts with it (the array RAW interlock).
    // dbg_s2_stall   -- the resolve stage held a request it could not
    //                   retire this cycle.
    // dbg_s2_reread  -- a skew-hazard re-read was armed this cycle.
    output wire dbg_fd_sethaz,
    output wire dbg_s2_stall,
    output wire dbg_s2_reread,
    // ── P6 front-door arbitration taps ────────────────────────────────
    // PER-SOURCE accepts, so an arbitration test counts what the door
    // actually did instead of inferring it from a cycle total.  These are
    // the only way to distinguish "hot > cold held" from "cold happened not
    // to be offering anything".
    output wire dbg_acc_dh,
    output wire dbg_acc_fetch,
    output wire dbg_acc_lsu,
    output wire dbg_acc_write,
    // Cold-path availability + the registered starvation flag, so the
    // fairness bound is measured rather than assumed.
    output wire dbg_cold_avail,
    output wire dbg_cold_starved,
    output wire dbg_dh_ar_have,
    // victim_query_hit at the resolve stage: the interlock that makes a
    // read of a line whose dirty copy is still in the writeback buffer
    // WAIT instead of reading DRAM behind it.  A coherence test asserts it
    // actually fired, so the test cannot pass vacuously.
    output wire dbg_victim_query_hit,
    output wire [15:0] dbg_dh_snap
);
    assign dbg_dh_en = (DH_EN != 0);
    assign dbg_dw_en = (DW_EN != 0);

    wire [DW_ID_WIDTH-1:0] dwg_awid;
    wire [31:0]            dwg_awaddr;
    wire [7:0]             dwg_awlen;
    wire [2:0]             dwg_awsize;
    wire [1:0]             dwg_awburst;
    wire                   dwg_awvalid, dwg_awready;
    wire [127:0]           dwg_wdata;
    wire [15:0]            dwg_wstrb;
    wire                   dwg_wlast, dwg_wvalid, dwg_wready;
    wire [DW_ID_WIDTH-1:0] dwg_bid;
    wire [1:0]             dwg_bresp;
    wire                   dwg_bvalid, dwg_bready;
    /* verilator lint_off PINCONNECTEMPTY */
    dh_write_window_guard #(
        .ID_WIDTH(DW_ID_WIDTH), .DATA_WIDTH(128)
    ) u_dw_guard (
        .clk(clk), .rst(rst),
        .ram_window_lg2(dh_ram_window_lg2),
        .s_aw_is_rom(dw_aw_is_rom),
        .abandon(dw_abandon),
        .s_awid(dw_axi_awid), .s_awaddr(dw_axi_awaddr), .s_awlen(dw_axi_awlen),
        .s_awsize(dw_axi_awsize), .s_awburst(dw_axi_awburst),
        .s_awvalid(dw_axi_awvalid), .s_awready(dw_axi_awready),
        .s_wdata(dw_axi_wdata), .s_wstrb(dw_axi_wstrb), .s_wlast(dw_axi_wlast),
        .s_wvalid(dw_axi_wvalid), .s_wready(dw_axi_wready),
        .s_bid(dw_axi_bid), .s_bresp(dw_axi_bresp), .s_bvalid(dw_axi_bvalid), .s_bready(dw_axi_bready),
        .m_awid(dwg_awid), .m_awaddr(dwg_awaddr), .m_awlen(dwg_awlen),
        .m_awsize(dwg_awsize), .m_awburst(dwg_awburst),
        .m_awvalid(dwg_awvalid), .m_awready(dwg_awready),
        .m_wdata(dwg_wdata), .m_wstrb(dwg_wstrb), .m_wlast(dwg_wlast),
        .m_wvalid(dwg_wvalid), .m_wready(dwg_wready),
        .m_bid(dwg_bid), .m_bresp(dwg_bresp), .m_bvalid(dwg_bvalid), .m_bready(dwg_bready),
        .fault_sticky(), .fault_addr(),
        .fault_count(dw_guard_fault_count), .drop_count(dw_guard_drop_count)
    );
    /* verilator lint_on PINCONNECTEMPTY */
    assign dbg_dh_cold_fair_n = DH_COLD_FAIR_N;

    // ── The window guard, in front of the hot door ─────────────────────
    // ID_WIDTH/DATA_WIDTH are the D side's (4 IDs, 128 b); the module is
    // otherwise the byte-for-byte one the I side ships.
    wire [DH_ID_WIDTH-1:0] dhg_arid;
    wire [31:0]            dhg_araddr;
    wire [7:0]             dhg_arlen;
    wire [2:0]             dhg_arsize;
    wire [1:0]             dhg_arburst;
    wire                   dhg_arvalid, dhg_arready;
    wire [DH_ID_WIDTH-1:0] dhg_rid;
    wire [127:0]           dhg_rdata;
    wire [1:0]             dhg_rresp;
    wire                   dhg_rlast, dhg_rvalid, dhg_rready;
    ifetch_window_guard #(
        .ID_WIDTH(DH_ID_WIDTH), .DATA_WIDTH(128), .OVERLAY_GUARD(1)
    ) u_dh_guard (
        .clk(clk), .rst(rst),
        .ram_window_lg2(dh_ram_window_lg2),
        .s_ar_is_rom(dh_ar_is_rom),
        .overlay_active(dh_overlay_active),
        .s_arid(dh_axi_arid), .s_araddr(dh_axi_araddr), .s_arlen(dh_axi_arlen),
        .s_arsize(dh_axi_arsize), .s_arburst(dh_axi_arburst),
        .s_arvalid(dh_axi_arvalid), .s_arready(dh_axi_arready),
        .s_rid(dh_axi_rid), .s_rdata(dh_axi_rdata), .s_rresp(dh_axi_rresp),
        .s_rlast(dh_axi_rlast), .s_rvalid(dh_axi_rvalid), .s_rready(dh_axi_rready),
        .m_arid(dhg_arid), .m_araddr(dhg_araddr), .m_arlen(dhg_arlen),
        .m_arsize(dhg_arsize), .m_arburst(dhg_arburst),
        .m_arvalid(dhg_arvalid), .m_arready(dhg_arready),
        .m_rid(dhg_rid), .m_rdata(dhg_rdata), .m_rresp(dhg_rresp),
        .m_rlast(dhg_rlast), .m_rvalid(dhg_rvalid), .m_rready(dhg_rready),
        .fault_sticky(dh_guard_fault_sticky), .fault_addr(dh_guard_fault_addr),
        .fault_count(dh_guard_fault_count)
    );

    l2c #(
        .ADDR_WIDTH(32), .DATA_WIDTH(128), .ID_WIDTH(6),
        .L2_BYPASS_ALL(L2_BYPASS_ALL),
        .DH_PORT_EN(DH_EN), .DH_ID_WIDTH(DH_ID_WIDTH),
        .DH_R_FALL_THROUGH(DH_R_FALL_THROUGH),
        .DH_COLD_FAIR_N(DH_COLD_FAIR_N),
        .DW_PORT_EN(DW_EN), .DW_ID_WIDTH(DW_ID_WIDTH),
        .VICTIM_SLOTS(VICTIM_SLOTS),
        .BYPASS_SLOTS(BYPASS_SLOTS),
        .NUM_BYPASS_WINDOWS(1),
        .BYP_WIN_BASE(32'h0100_0000), .BYP_WIN_MASK(32'hFFF0_0000), .BYP_WIN_EN(1'b1),
        .CACHEABLE_BASE(32'h0000_0000), .CACHEABLE_SIZE(32'h0100_0000)
    ) dut (
        .clk(clk), .rst(rst),
        .s_axi_awid(s_axi_awid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awlen(s_axi_awlen),
        .s_axi_awsize(s_axi_awsize), .s_axi_awburst(s_axi_awburst), .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wlast(s_axi_wlast),
        .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bid(s_axi_bid), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_arid(s_axi_arid), .s_axi_araddr(s_axi_araddr), .s_axi_arlen(s_axi_arlen),
        .s_axi_arsize(s_axi_arsize), .s_axi_arburst(s_axi_arburst), .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rid(s_axi_rid), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rlast(s_axi_rlast), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .f_axi_arid(f_axi_arid), .f_axi_araddr(f_axi_araddr), .f_axi_arlen(f_axi_arlen),
        .f_axi_arsize(f_axi_arsize), .f_axi_arburst(f_axi_arburst),
        .f_axi_arvalid(f_axi_arvalid), .f_axi_arready(f_axi_arready),
        .f_axi_rid(f_axi_rid), .f_axi_rdata(f_axi_rdata), .f_axi_rresp(f_axi_rresp),
        .f_axi_rlast(f_axi_rlast), .f_axi_rvalid(f_axi_rvalid), .f_axi_rready(f_axi_rready),
        .dh_axi_arid(dhg_arid), .dh_axi_araddr(dhg_araddr), .dh_axi_arlen(dhg_arlen),
        .dh_axi_arsize(dhg_arsize), .dh_axi_arburst(dhg_arburst),
        .dh_axi_arvalid(dhg_arvalid), .dh_axi_arready(dhg_arready),
        .dh_axi_rid(dhg_rid), .dh_axi_rdata(dhg_rdata), .dh_axi_rresp(dhg_rresp),
        .dh_axi_rlast(dhg_rlast), .dh_axi_rvalid(dhg_rvalid), .dh_axi_rready(dhg_rready),
        .dw_axi_awid(dwg_awid), .dw_axi_awaddr(dwg_awaddr), .dw_axi_awlen(dwg_awlen),
        .dw_axi_awsize(dwg_awsize), .dw_axi_awburst(dwg_awburst),
        .dw_axi_awvalid(dwg_awvalid), .dw_axi_awready(dwg_awready),
        .dw_axi_wdata(dwg_wdata), .dw_axi_wstrb(dwg_wstrb), .dw_axi_wlast(dwg_wlast),
        .dw_axi_wvalid(dwg_wvalid), .dw_axi_wready(dwg_wready),
        .dw_axi_bid(dwg_bid), .dw_axi_bresp(dwg_bresp), .dw_axi_bvalid(dwg_bvalid),
        .dw_axi_bready(dwg_bready),
        .dbg_dh_snap(dbg_dh_snap),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready),
        .dbg_hit_count(), .dbg_miss_count(), .dbg_mshr_occupancy(dbg_mshr_occupancy)
    );

    // Hierarchical references into the DUT.  Legal Verilog-2005, and this
    // is a testbench-only wrapper -- nothing here is ever synthesised.  The
    // g_active generate block does not exist in an L2_BYPASS_ALL build, so
    // the taps are guarded by the same parameter.
    generate
    if (L2_BYPASS_ALL == 0) begin : g_taps
        assign dbg_vb_occ   = dut.g_active.u_victim.used_c;
        assign dbg_vb_out   = dut.g_active.u_victim.out_cnt;
        assign dbg_vb_sink  = dut.g_active.u_victim.sink_cnt;
        assign dbg_vb_slots = dut.g_active.u_victim.slots_c[4:0];
        assign dbg_evict_stall = dut.g_active.u_ctrl.miss_no_hit_c &&
                                 !dut.g_active.u_ctrl.mshr_lu_hit &&
                                 dut.g_active.u_ctrl.need_evict_c &&
                                 dut.g_active.u_ctrl.victim_sel_ok &&
                                 !dut.g_active.u_ctrl.victim_push_ready;
        assign dbg_vb_drain_busy = dut.g_active.u_victim.drain_busy_c;
        // Independent old-style registered install payload. The implementation
        // may share its active-line register, but valid install beats must still
        // equal this separately captured pre-edge line plus dynamic byte merge.
        reg [511:0] install_shadow;
        integer install_byte;
        always @(posedge clk) begin
            if (!dut.g_active.u_ctrl.u_mshr.rst) begin
                if (dut.g_active.u_ctrl.u_mshr.inst_valid &&
                    dut.g_active.u_ctrl.u_mshr.inst_data !== install_shadow)
                    $fatal(1, "L2 install payload differs from registered reference");
                if (!dut.g_active.u_ctrl.u_mshr.m_fill_err[dut.g_active.u_ctrl.u_mshr.act]) begin
                    if (dut.g_active.u_ctrl.u_mshr.st == dut.g_active.u_ctrl.u_mshr.S_INSTALL) begin
                        install_shadow <= dut.g_active.u_ctrl.u_mshr.act_line;
                        if (dut.g_active.u_ctrl.u_mshr.p_wr[dut.g_active.u_ctrl.u_mshr.act])
                            for (install_byte = 0; install_byte < 16; install_byte = install_byte + 1)
                                if (dut.g_active.u_ctrl.u_mshr.p_wstrb_act_c[install_byte])
                                    install_shadow[dut.g_active.u_ctrl.u_mshr.p_qoff_act_c*128 + install_byte*8 +: 8]
                                        <= dut.g_active.u_ctrl.u_mshr.p_wdata_act_c[install_byte*8 +: 8];
                    end
                    if (dut.g_active.u_ctrl.u_mshr.st == dut.g_active.u_ctrl.u_mshr.S_SWR) begin
                        install_shadow <= dut.g_active.u_ctrl.u_mshr.act_line;
                        for (install_byte = 0; install_byte < 16; install_byte = install_byte + 1)
                            if (dut.g_active.u_ctrl.u_mshr.r_wstrb_act_c[install_byte])
                                install_shadow[dut.g_active.u_ctrl.u_mshr.r_qoff_act_c*128 + install_byte*8 +: 8]
                                    <= dut.g_active.u_ctrl.u_mshr.r_wdata_act_c[install_byte*8 +: 8];
                    end
                end
            end
        end
        assign dbg_vb_seq_busy   = dut.g_active.u_victim.seq_busy_c;
        assign dbg_vb_wb_err     = dut.g_active.u_victim.wb_err_count[7:0];
        assign dbg_by_slots      = dut.g_active.u_bypass.slots_c;
        assign dbg_by_occ        = dut.g_active.u_bypass.occ_c;
        assign dbg_by_inflight   = dut.g_active.u_bypass.inflight_c;
        // Front door.  `accept_slot_c` is l2c_ctrl's "stage 1 can take a
        // request this cycle" -- the pipelined replacement for the old
        // `st == S_IDLE`.  Since 2026-08-20 the lookup is a 3-STAGE
        // pipeline, so an accept slot exists nearly every cycle rather
        // than one cycle in three; `idle` is still the denominator that
        // the door is judged against, it is just a much bigger number.
        assign dbg_fd_idle   = !dut.g_active.u_ctrl.rst_busy &&
                                dut.g_active.u_ctrl.accept_slot_c;
        assign dbg_fd_accept = dbg_fd_idle && (dut.g_active.u_ctrl.do_accept_c ||
                                                dut.g_active.u_ctrl.do_bypass_c);
        assign dbg_fd_gather = dut.g_active.u_ctrl.flw_gath_c;
        assign dbg_fd_nowork = dbg_fd_idle && !dut.g_active.u_ctrl.any_sel;
        // A beat was available but refused at the door for an ORDERING
        // reason.  Only a bypass-bound beat can be refused for its id now
        // (the cache path's same-id check moved to the resolve stage), so
        // this counter is bypass-only by construction.
        assign dbg_fd_idbusy = dbg_fd_idle && dut.g_active.u_ctrl.any_sel &&
                                dut.g_active.u_ctrl.is_bypass_c &&
                                (dut.g_active.u_ctrl.id_busy_c ||
                                 dut.g_active.u_ctrl.pipe_id_haz_c);
        // NEW (pipelining): a cache-bound beat refused because an older
        // request in the pipeline holds a conflicting set.  This is the
        // cost of the array read-after-write interlock and it is the
        // number to watch if a workload stops scaling.
        assign dbg_fd_sethaz = dbg_fd_idle && dut.g_active.u_ctrl.any_sel &&
                                !dut.g_active.u_ctrl.is_bypass_c &&
                                dut.g_active.u_ctrl.set_haz_c;
        // Stage 2 held a request it could not retire this cycle.
        assign dbg_s2_stall  = dut.g_active.u_ctrl.req_v &&
                                !dut.g_active.u_ctrl.p2_done_c;
        assign dbg_s2_reread = dut.g_active.u_ctrl.rr_start_c;
        assign dbg_fd_bypnr  = dbg_fd_idle && dut.g_active.u_ctrl.any_sel &&
                                !dut.g_active.u_ctrl.id_busy_c &&
                                dut.g_active.u_ctrl.is_bypass_c &&
                                !dut.g_active.u_ctrl.byp_req_ready;
        // ── P6 taps ────────────────────────────────────────────────────
        assign dbg_acc_dh    = dut.g_active.u_ctrl.door_take_c && dut.g_active.u_ctrl.dh_read_sel;
        assign dbg_acc_fetch = dut.g_active.u_ctrl.door_take_c && dut.g_active.u_ctrl.fetch_read_sel;
        assign dbg_acc_lsu   = dut.g_active.u_ctrl.door_take_c && dut.g_active.u_ctrl.lsu_read_sel;
        assign dbg_acc_write = dut.g_active.u_ctrl.door_take_c && dut.g_active.u_ctrl.write_sel;
        assign dbg_cold_avail   = dut.g_active.u_ctrl.cold_avail_c;
        assign dbg_cold_starved = dut.g_active.u_ctrl.cold_starved_q;
        assign dbg_dh_ar_have   = dut.g_active.u_ctrl.dh_ar_have;
        assign dbg_acc_dw       = dut.g_active.u_ctrl.door_take_c && dut.g_active.u_ctrl.dw_write_sel;
        assign dbg_dw_aw_have   = dut.g_active.u_ctrl.dw_aw_have;
        assign dbg_hot_avail    = dut.g_active.u_ctrl.hot_read_avail_c ||
                                  dut.g_active.u_ctrl.dw_wr_avail_c;
        // A hot write accepted while a COLD gathered line sits at G_FULL --
        // the one cycle shape where the full-line masks must NOT apply.
        assign dbg_dw_over_gfull = dut.g_active.u_ctrl.door_take_c &&
                                   dut.g_active.u_ctrl.dw_write_sel &&
                                   (dut.g_active.u_ctrl.gst == 2'd2);
        assign dbg_victim_query_hit = dut.g_active.u_ctrl.req_v &&
                                      dut.g_active.u_ctrl.victim_query_hit;
    end else begin : g_no_taps
        assign dbg_acc_dh    = 1'b0;
        assign dbg_acc_fetch = 1'b0;
        assign dbg_acc_lsu   = 1'b0;
        assign dbg_acc_write = 1'b0;
        assign dbg_cold_avail   = 1'b0;
        assign dbg_cold_starved = 1'b0;
        assign dbg_dh_ar_have   = 1'b0;
        assign dbg_acc_dw       = 1'b0;
        assign dbg_dw_aw_have   = 1'b0;
        assign dbg_hot_avail    = 1'b0;
        assign dbg_dw_over_gfull = 1'b0;
        assign dbg_victim_query_hit = 1'b0;
        assign dbg_vb_occ        = 5'd0;
        assign dbg_vb_out        = 5'd0;
        assign dbg_vb_sink       = 5'd0;
        assign dbg_vb_slots      = 5'd0;
        assign dbg_evict_stall   = 1'b0;
        assign dbg_vb_drain_busy = 1'b0;
        assign dbg_vb_seq_busy   = 1'b0;
        assign dbg_vb_wb_err     = 8'd0;
        assign dbg_by_slots      = 8'd0;
        assign dbg_by_occ        = 6'd0;
        assign dbg_by_inflight   = 6'd0;
        assign dbg_fd_idle       = 1'b0;
        assign dbg_fd_accept     = 1'b0;
        assign dbg_fd_gather     = 1'b0;
        assign dbg_fd_nowork     = 1'b0;
        assign dbg_fd_idbusy     = 1'b0;
        assign dbg_fd_bypnr      = 1'b0;
        assign dbg_fd_sethaz     = 1'b0;
        assign dbg_s2_stall      = 1'b0;
        assign dbg_s2_reread     = 1'b0;
    end
    endgenerate

    // Channel-level stalls need no hierarchy -- they are the top-level
    // handshake, and they are meaningful in the L2_BYPASS_ALL build too.
    assign dbg_fd_ar_stall = s_axi_arvalid && !s_axi_arready;
    assign dbg_fd_aw_stall = s_axi_awvalid && !s_axi_awready;
    assign dbg_fd_w_stall  = s_axi_wvalid  && !s_axi_wready;

endmodule

`default_nettype wire
