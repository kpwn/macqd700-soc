// tb_n2w_watchdog.v — Verilog harness for the axi_narrow_to_wide
// abandonment watchdog.
//
// axi_narrow_to_wide sits between EVERY narrow (32-bit) master in this SoC
// and the 128-bit fabric.  It holds exactly one outstanding transaction per
// direction and gates n_awready/n_arready on that latch, so a wide side that
// never answers wedges the master permanently.  The abandonment watchdog is
// the only thing that stops that being a permanent SoC hang: after
// TIMEOUT_CYCLES with NO PROGRESS on any channel it force-clears the
// transaction and answers the narrow master with SLVERR.
//
// This harness exists so the watchdog can be driven directly, at
// cycle granularity, with a stallable wide side:
//
//   * TIMEOUT_CYCLES is overridden to a small value (default 64) so a
//     full expiry costs tens of cycles instead of two billion.
//   * Every narrow and wide AXI port is brought straight out to the
//     C++ driver (tb/tb_n2w_watchdog.cpp), which plays both the narrow
//     master and the wide slave and can stall either at will.
//   * The watchdog's INTERNAL state is tapped out as dbg_* outputs.  The
//     abandonment path's whole job is state teardown (aw_valid_q,
//     aw_beats_left_q, aw_wrx_left_q, wo_valid_q|wg_full_q, wraw_valid_q,
//     aw_drain_q, aw_err_q, and the read-side equivalents), and a
//     port-only view cannot tell "torn down correctly" from "happened to
//     look right this cycle".  Tapping the counters additionally lets the
//     driver place an event on an EXACT counter value, which is what the
//     boundary scenarios need.
//
// The taps are hierarchical reads only — nothing here drives the DUT's
// internals.
//
// Built via: make tb-n2w-watchdog

`default_nettype none

module tb_n2w_watchdog #(
    // The watchdog ships DISABLED (axi_narrow_to_wide defaults
    // ENABLE_ABANDON_TIMEOUT to 0 so a 20-second timer stays off the live
    // WREADY handshake).  The Makefile builds this harness twice: once at 1
    // to exercise the mechanism, once at 0 to pin the disabled contract.
    // MUST match N2W_WATCHDOG_ENABLED in tb_n2w_watchdog.cpp.
    parameter integer ENABLE_ABANDON_TIMEOUT = 1,
    // Small enough that a full expiry is tens of cycles, large enough
    // that a burst can make several progress events inside one window.
    // MUST match kTimeout in tb_n2w_watchdog.cpp.
    parameter [31:0] TIMEOUT_CYCLES = 32'd64
) (
    input  wire         clk,
    input  wire         rst,

    // ── Narrow (32-bit) slave side — driven by the C++ master ────────
    input  wire [31:0]  n_awaddr,
    input  wire [2:0]   n_awprot,
    input  wire [7:0]   n_awlen,
    input  wire         n_awvalid,
    output wire         n_awready,
    input  wire [31:0]  n_wdata,
    input  wire [3:0]   n_wstrb,
    input  wire         n_wlast,
    input  wire         n_wvalid,
    output wire         n_wready,
    output wire [1:0]   n_bresp,
    output wire         n_bvalid,
    input  wire         n_bready,
    input  wire [31:0]  n_araddr,
    input  wire [2:0]   n_arprot,
    input  wire [2:0]   n_arsize,
    input  wire [7:0]   n_arlen,
    input  wire         n_arvalid,
    output wire         n_arready,
    output wire [31:0]  n_rdata,
    output wire [1:0]   n_rresp,
    output wire         n_rlast,
    output wire         n_rvalid,
    input  wire         n_rready,

    // ── Wide (128-bit) master side — driven by the C++ slave ─────────
    output wire [3:0]   w_awid,
    output wire [31:0]  w_awaddr,
    output wire [7:0]   w_awlen,
    output wire [2:0]   w_awsize,
    output wire [1:0]   w_awburst,
    output wire         w_awvalid,
    input  wire         w_awready,
    output wire [127:0] w_wdata,
    output wire [15:0]  w_wstrb,
    output wire         w_wlast,
    output wire         w_wvalid,
    input  wire         w_wready,
    input  wire [3:0]   w_bid,
    input  wire [1:0]   w_bresp,
    input  wire         w_bvalid,
    output wire         w_bready,
    output wire [3:0]   w_arid,
    output wire [31:0]  w_araddr,
    output wire [7:0]   w_arlen,
    output wire [2:0]   w_arsize,
    output wire [1:0]   w_arburst,
    output wire         w_arvalid,
    input  wire         w_arready,
    input  wire [3:0]   w_rid,
    input  wire [127:0] w_rdata,
    input  wire [1:0]   w_rresp,
    input  wire         w_rlast,
    input  wire         w_rvalid,
    output wire         w_rready,

    // ── Watchdog state taps (read-only hierarchical references) ──────
    output wire [31:0]  dbg_aw_timeout_cnt,
    output wire         dbg_aw_valid_q,
    output wire         dbg_aw_accepted_q,
    output wire [8:0]   dbg_aw_beats_left_q,
    output wire [8:0]   dbg_aw_wrx_left_q,
    output wire [8:0]   dbg_aw_wswal_q,
    output wire         dbg_wg_valid_q,
    output wire         dbg_wraw_valid_q,
    output wire         dbg_aw_drain_q,
    output wire         dbg_aw_err_q,

    output wire [31:0]  dbg_ar_timeout_cnt,
    output wire         dbg_ar_valid_q,
    output wire         dbg_ar_accepted_q,
    output wire [8:0]   dbg_ar_beats_left_q,
    output wire [8:0]   dbg_ar_err_left_q,
    output wire         dbg_rg_valid_q,
    output wire         dbg_ar_drain_q
);

    axi_narrow_to_wide #(
        .ID_WIDTH               (4),
        .ID_TAG                 (4'h0),
        .ENABLE_ABANDON_TIMEOUT (ENABLE_ABANDON_TIMEOUT),
        .TIMEOUT_CYCLES         (TIMEOUT_CYCLES)
    ) u_dut (
        .clk        (clk),
        .rst        (rst),

        .n_awaddr   (n_awaddr),
        .n_awprot   (n_awprot),
        .n_awlen    (n_awlen),
        .n_awvalid  (n_awvalid),
        .n_awready  (n_awready),
        .n_wdata    (n_wdata),
        .n_wstrb    (n_wstrb),
        .n_wlast    (n_wlast),
        .n_wvalid   (n_wvalid),
        .n_wready   (n_wready),
        .n_bresp    (n_bresp),
        .n_bvalid   (n_bvalid),
        .n_bready   (n_bready),
        .n_araddr   (n_araddr),
        .n_arprot   (n_arprot),
        .n_arsize   (n_arsize),
        .n_arlen    (n_arlen),
        .n_arvalid  (n_arvalid),
        .n_arready  (n_arready),
        .n_rdata    (n_rdata),
        .n_rresp    (n_rresp),
        .n_rlast    (n_rlast),
        .n_rvalid   (n_rvalid),
        .n_rready   (n_rready),

        .w_awid     (w_awid),
        .w_awaddr   (w_awaddr),
        .w_awlen    (w_awlen),
        .w_awsize   (w_awsize),
        .w_awburst  (w_awburst),
        .w_awvalid  (w_awvalid),
        .w_awready  (w_awready),
        .w_wdata    (w_wdata),
        .w_wstrb    (w_wstrb),
        .w_wlast    (w_wlast),
        .w_wvalid   (w_wvalid),
        .w_wready   (w_wready),
        .w_bid      (w_bid),
        .w_bresp    (w_bresp),
        .w_bvalid   (w_bvalid),
        .w_bready   (w_bready),
        .w_arid     (w_arid),
        .w_araddr   (w_araddr),
        .w_arlen    (w_arlen),
        .w_arsize   (w_arsize),
        .w_arburst  (w_arburst),
        .w_arvalid  (w_arvalid),
        .w_arready  (w_arready),
        .w_rid      (w_rid),
        .w_rdata    (w_rdata),
        .w_rresp    (w_rresp),
        .w_rlast    (w_rlast),
        .w_rvalid   (w_rvalid),
        .w_rready   (w_rready)
    );

    assign dbg_aw_timeout_cnt  = u_dut.aw_timeout_cnt;
    assign dbg_aw_valid_q      = u_dut.aw_valid_q;
    assign dbg_aw_accepted_q   = u_dut.aw_accepted_q;
    assign dbg_aw_beats_left_q = u_dut.aw_beats_left_q;
    assign dbg_aw_wrx_left_q   = u_dut.aw_wrx_left_q;
    assign dbg_aw_wswal_q      = u_dut.aw_wswal_q;
    // "an assembled wide beat is pending somewhere in the gather path".
    // The gather output is double-buffered (2026-08-20): a completed beat
    // normally sits in the OUTPUT bank (wo_valid_q) and only parks in the
    // gather bank (wg_full_q) when the wide side backpressures.  The tap
    // keeps its historical meaning by covering both.
    assign dbg_wg_valid_q      = u_dut.wo_valid_q || u_dut.wg_full_q;
    assign dbg_wraw_valid_q    = u_dut.wraw_valid_q;
    assign dbg_aw_drain_q      = u_dut.aw_drain_q;
    assign dbg_aw_err_q        = u_dut.aw_err_q;

    assign dbg_ar_timeout_cnt  = u_dut.ar_timeout_cnt;
    assign dbg_ar_valid_q      = u_dut.ar_valid_q;
    assign dbg_ar_accepted_q   = u_dut.ar_accepted_q;
    assign dbg_ar_beats_left_q = u_dut.ar_beats_left_q;
    assign dbg_ar_err_left_q   = u_dut.ar_err_left_q;
    assign dbg_rg_valid_q      = u_dut.rg_valid_q;
    assign dbg_ar_drain_q      = u_dut.ar_drain_q;

endmodule

`default_nettype wire
