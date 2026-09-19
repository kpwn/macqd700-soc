// tb_n2w_pipe.v — Verilog harness for axi_narrow_to_wide's MULTI-OUTSTANDING
// behaviour (2026-08-20 rework).
//
// The adapter used to hold exactly one transaction per direction:
// n_awready was `!aw_valid_q` with aw_valid_q cleared only on B, and
// n_arready was `!ar_valid_q` cleared only on the last R.  Every narrow
// master behind it was therefore serialized at one full fabric round trip
// per transaction, whatever it was capable of issuing.
//
// Nothing in the tree could measure that, because every existing harness
// that drives this module is itself single-outstanding:
//
//   * tb_l2c_wstream.cpp's stream_writes() waits for B before presenting
//     the next AW ("single-outstanding narrow master" is printed in its
//     own banner), so it prices the fabric, not the adapter's ability to
//     overlap.
//   * tb_axi_widen.cpp / tb_axi_widen_watchdog.cpp drive one transaction
//     at a time by construction.
//   * tb_sd_boot_top.v does not contain this module at all — it models
//     the fabric with a B-latency parameter, so its cycles/word number is
//     a boot_fsm measurement, not an adapter one.
//
// This harness exists to (a) produce the depth curve the shipping
// WR_OUTSTANDING / RD_OUTSTANDING values were chosen from and (b) hold
// the directed multi-outstanding scenarios: several writes in flight,
// a read and a write in flight together, mixed burst lengths proving the
// gather buffer never interleaves two transactions into one wide beat,
// and the abandonment watchdog firing with N transactions outstanding.
//
// Both AXI sides are brought straight out to the C++ driver
// (tb/tb_n2w_pipe.cpp), which plays a pipelined narrow master and a wide
// slave with a configurable, multi-outstanding response latency.
//
// The DEPTH parameter is passed through to the DUT so the Makefile can
// build the same source at several depths; the driver prints which one it
// is running as.  Depth 1 is built as well and is the pin that the
// historical single-outstanding contract is still exactly reproducible.
//
// Built via: make tb-n2w-pipe

`default_nettype none

module tb_n2w_pipe #(
    // Outstanding-transaction depth under test.  MUST match kDepth in
    // tb_n2w_pipe.cpp (passed via -DN2W_PIPE_DEPTH).
    parameter integer DEPTH = 4,
    // Watchdog left ENABLED here (it ships disabled) with a short window,
    // so the multi-outstanding abandonment scenarios are reachable.  Perf
    // scenarios never stall, so the timer never gets near this.
    parameter integer ENABLE_ABANDON_TIMEOUT = 1,
    parameter [31:0]  TIMEOUT_CYCLES = 32'd512
) (
    input  wire         clk,
    input  wire         rst,

    // ── Narrow (32-bit) slave side — driven by the C++ master ────────
    input  wire [31:0]  n_awaddr,
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

    // ── State taps (read-only hierarchical references) ───────────────
    // The multi-outstanding bookkeeping is the thing under test, and a
    // port-only view cannot tell "two writes genuinely in flight" from
    // "one write and a coincidence".
    output wire [31:0]  dbg_wr_out_q,
    output wire [31:0]  dbg_ar_out_q,
    output wire         dbg_aw_valid_q,
    output wire         dbg_ar_valid_q,
    output wire         dbg_aw_err_q,
    output wire [8:0]   dbg_ar_err_left_q,
    output wire         dbg_ar_err_mode_q
);

    axi_narrow_to_wide #(
        .ID_WIDTH               (4),
        .ID_TAG                 (4'h0),
        .ENABLE_ABANDON_TIMEOUT (ENABLE_ABANDON_TIMEOUT),
        .TIMEOUT_CYCLES         (TIMEOUT_CYCLES),
        .WR_OUTSTANDING         (DEPTH),
        .RD_OUTSTANDING         (DEPTH)
    ) u_dut (
        .clk        (clk),
        .rst        (rst),

        .n_awaddr   (n_awaddr),
        .n_awprot   (3'b000),
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
        .n_arprot   (3'b000),
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

    assign dbg_wr_out_q      = {{(32-1){1'b0}}, 1'b0} | u_dut.wr_out_q;
    assign dbg_ar_out_q      = {{(32-1){1'b0}}, 1'b0} | u_dut.ar_out_q;
    assign dbg_aw_valid_q    = u_dut.aw_valid_q;
    assign dbg_ar_valid_q    = u_dut.ar_valid_q;
    assign dbg_aw_err_q      = u_dut.aw_err_q;
    assign dbg_ar_err_left_q = u_dut.ar_err_left_q;
    assign dbg_ar_err_mode_q = u_dut.ar_err_mode_q;

endmodule

`default_nettype wire
