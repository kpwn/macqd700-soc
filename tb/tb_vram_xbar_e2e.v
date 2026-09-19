// tb_vram_xbar_e2e.v — Verilator wrapper for the real xbar → vram path.
//
// Task #147 E2E harness
// ─────────────────────
// Proves that a CPU-style AXI4 write driven into the xbar's M0 port at
// a VRAM-aperture address (0xF900_XXXX) lands in the URAM-backed vram
// module at the correct stripped offset AND is visible on the scanner
// streaming-read port.  This is the full-stack variant of
// tb_vram_cpu_write which drove the vram slave directly.
//
// Topology
// ────────
//    [C++ AXI master on M0]
//           │
//           ▼
//    ┌──────────────┐
//    │  axi_xbar    │ (3M × 6S; only M0 + S3 active, S0/S1/S2/S4/S5 sinked)
//    │              │──► S3 (VRAM aperture, base stripped)
//    └──────────────┘        │
//                            ▼
//                     ┌──────────────┐
//                     │     vram     │ (128×48 px, BPP=8)
//                     └──────┬───────┘
//                            │
//                            ▼
//                     [C++ scanner samples rd_*]
//
// The non-VRAM slaves (S0 DDR, S1 IO, S2 DMA, S4 DAFB, S5 SD-JTAG) are
// present on the xbar's ports but tied-off ready so they can accept any
// spurious traffic.  M0B (boot FSM, write-only fan-in on the shared M0
// port), M1 (host debug) and M2 (CPU IF, read-only) are tied off inactive.
// The old M3/M4 seats no longer exist — the 2026-07-16 master-count
// reduction folded boot onto M0's m0b_* fan-in and stubbed DMA's master
// port out of the crossbar entirely.
//
// Verilog-2005, no SystemVerilog.

`default_nettype none

module tb_vram_xbar_e2e #(
    parameter FB_W   = 128,
    parameter FB_H   = 48,
    parameter FB_BPP = 8
) (
    input  wire         clk,
    input  wire         rst,

    // M0 AXI master (CPU LSU equivalent) — only channel we actually drive.
    input  wire [3:0]   m0_awid,
    input  wire [31:0]  m0_awaddr,
    input  wire [7:0]   m0_awlen,
    input  wire [2:0]   m0_awsize,
    input  wire [1:0]   m0_awburst,
    input  wire         m0_awvalid,
    output wire         m0_awready,
    input  wire [127:0] m0_wdata,
    input  wire [15:0]  m0_wstrb,
    input  wire         m0_wlast,
    input  wire         m0_wvalid,
    output wire         m0_wready,
    output wire [3:0]   m0_bid,
    output wire [1:0]   m0_bresp,
    output wire         m0_bvalid,
    input  wire         m0_bready,
    input  wire [3:0]   m0_arid,
    input  wire [31:0]  m0_araddr,
    input  wire [7:0]   m0_arlen,
    input  wire [2:0]   m0_arsize,
    input  wire [1:0]   m0_arburst,
    input  wire         m0_arvalid,
    output wire         m0_arready,
    output wire [3:0]   m0_rid,
    output wire [127:0] m0_rdata,
    output wire [1:0]   m0_rresp,
    output wire         m0_rlast,
    output wire         m0_rvalid,
    input  wire         m0_rready,

    // Scanner streaming read port (from vram module).  Width is fixed
    // large enough for the full Q700 VRAM image; smaller configs just
    // use the low bits.
    input  wire [19:0]  rd_addr,
    input  wire         rd_en,
    // 4-LANE group starting at rd_addr (vram.v RD_DATA_W == 4*BPP): the pixel
    // AT rd_addr occupies the TOP BPP bits and is valid at any alignment; the
    // three lanes below it are the pixels at +1/+2/+3 and are valid only when
    // rd_addr is 4-lane aligned.  At the FB_BPP=8 default that is the 4-byte
    // group {b0,b1,b2,b3} in 32 bits.
    output wire [4*FB_BPP-1:0] rd_data,
    output wire         rd_valid
);

    function integer clog2;
        input integer x;
        integer i;
        begin
            clog2 = 0;
            for (i = x - 1; i > 0; i = i >> 1) clog2 = clog2 + 1;
            if (clog2 == 0) clog2 = 1;
        end
    endfunction

    localparam integer RD_ADDR_W = clog2(FB_W * FB_H);

    // S0 DDR slave wires (tie off with ready=1, accept and drop writes).
    wire [5:0]   s0_awid, s0_bid, s0_arid, s0_rid;
    wire [31:0]  s0_awaddr, s0_araddr;
    wire [7:0]   s0_awlen, s0_arlen;
    wire [2:0]   s0_awsize, s0_arsize;
    wire [1:0]   s0_awburst, s0_arburst, s0_bresp, s0_rresp;
    wire         s0_awvalid, s0_awready, s0_wvalid, s0_wready, s0_bvalid, s0_bready,
                 s0_arvalid, s0_arready, s0_rvalid, s0_rready, s0_wlast, s0_rlast;
    wire [127:0] s0_wdata, s0_rdata;
    wire [15:0]  s0_wstrb;

    // S1 IO slave tied off similarly.
    wire [5:0]   s1_awid, s1_bid, s1_arid, s1_rid;
    wire [31:0]  s1_awaddr, s1_araddr;
    wire [7:0]   s1_awlen, s1_arlen;
    wire [2:0]   s1_awsize, s1_arsize;
    wire [1:0]   s1_awburst, s1_arburst, s1_bresp, s1_rresp;
    wire         s1_awvalid, s1_awready, s1_wvalid, s1_wready, s1_bvalid, s1_bready,
                 s1_arvalid, s1_arready, s1_rvalid, s1_rready, s1_wlast, s1_rlast;
    wire [127:0] s1_wdata, s1_rdata;
    wire [15:0]  s1_wstrb;

    // S2 DMA slave tied off similarly.
    wire [5:0]   s2_awid, s2_bid, s2_arid, s2_rid;
    wire [31:0]  s2_awaddr, s2_araddr;
    wire [7:0]   s2_awlen, s2_arlen;
    wire [2:0]   s2_awsize, s2_arsize;
    wire [1:0]   s2_awburst, s2_arburst, s2_bresp, s2_rresp;
    wire         s2_awvalid, s2_awready, s2_wvalid, s2_wready, s2_bvalid, s2_bready,
                 s2_arvalid, s2_arready, s2_rvalid, s2_rready, s2_wlast, s2_rlast;
    wire [127:0] s2_wdata, s2_rdata;
    wire [15:0]  s2_wstrb;

    // S3 VRAM slave — drives u_vram below.
    wire [5:0]   s3_awid, s3_bid, s3_arid, s3_rid;
    wire [31:0]  s3_awaddr, s3_araddr;
    wire [7:0]   s3_awlen, s3_arlen;
    wire [2:0]   s3_awsize, s3_arsize;
    wire [1:0]   s3_awburst, s3_arburst, s3_bresp, s3_rresp;
    wire         s3_awvalid, s3_awready, s3_wvalid, s3_wready,
                 s3_bvalid, s3_bready,
                 s3_arvalid, s3_arready, s3_rvalid, s3_rready,
                 s3_wlast, s3_rlast;
    wire [127:0] s3_wdata, s3_rdata;
    wire [15:0]  s3_wstrb;

    // S4 DAFB register shim tied off similarly.
    wire [5:0]   s4_awid, s4_bid, s4_arid, s4_rid;
    wire [31:0]  s4_awaddr, s4_araddr;
    wire [7:0]   s4_awlen, s4_arlen;
    wire [2:0]   s4_awsize, s4_arsize;
    wire [1:0]   s4_awburst, s4_arburst, s4_bresp, s4_rresp;
    wire         s4_awvalid, s4_awready, s4_wvalid, s4_wready,
                 s4_bvalid, s4_bready,
                 s4_arvalid, s4_arready, s4_rvalid, s4_rready,
                 s4_wlast, s4_rlast;
    wire [127:0] s4_wdata, s4_rdata;
    wire [15:0]  s4_wstrb;

    // S5 SD JTAG writer tied off similarly.
    wire [5:0]   s5_awid, s5_bid, s5_arid, s5_rid;
    wire [31:0]  s5_awaddr, s5_araddr;
    wire [7:0]   s5_awlen, s5_arlen;
    wire [2:0]   s5_awsize, s5_arsize;
    wire [1:0]   s5_awburst, s5_arburst, s5_bresp, s5_rresp;
    wire         s5_awvalid, s5_awready, s5_wvalid, s5_wready,
                 s5_bvalid, s5_bready,
                 s5_arvalid, s5_arready, s5_rvalid, s5_rready,
                 s5_wlast, s5_rlast;
    wire [127:0] s5_wdata, s5_rdata;
    wire [15:0]  s5_wstrb;

    // Simple ready/accept tie-off for S0, S1, S2, S4 — they must handshake
    // every AW/W and emit a BVALID so the xbar can clear its own state.
    // Since the xbar only routes 0xF9000000..0xF90FFFFF to S3, these
    // slaves see no traffic in the scenarios we care about, but we
    // make them robust anyway.
    tb_vram_xbar_sink_slave #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_s0_sink (
        .clk(clk), .rst(rst),
        .awid(s0_awid), .awaddr(s0_awaddr), .awlen(s0_awlen),
        .awsize(s0_awsize), .awburst(s0_awburst),
        .awvalid(s0_awvalid), .awready(s0_awready),
        .wdata(s0_wdata), .wstrb(s0_wstrb), .wlast(s0_wlast),
        .wvalid(s0_wvalid), .wready(s0_wready),
        .bid(s0_bid), .bresp(s0_bresp), .bvalid(s0_bvalid), .bready(s0_bready),
        .arid(s0_arid), .araddr(s0_araddr), .arlen(s0_arlen),
        .arsize(s0_arsize), .arburst(s0_arburst),
        .arvalid(s0_arvalid), .arready(s0_arready),
        .rid(s0_rid), .rdata(s0_rdata), .rresp(s0_rresp),
        .rlast(s0_rlast), .rvalid(s0_rvalid), .rready(s0_rready)
    );
    tb_vram_xbar_sink_slave #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_s1_sink (
        .clk(clk), .rst(rst),
        .awid(s1_awid), .awaddr(s1_awaddr), .awlen(s1_awlen),
        .awsize(s1_awsize), .awburst(s1_awburst),
        .awvalid(s1_awvalid), .awready(s1_awready),
        .wdata(s1_wdata), .wstrb(s1_wstrb), .wlast(s1_wlast),
        .wvalid(s1_wvalid), .wready(s1_wready),
        .bid(s1_bid), .bresp(s1_bresp), .bvalid(s1_bvalid), .bready(s1_bready),
        .arid(s1_arid), .araddr(s1_araddr), .arlen(s1_arlen),
        .arsize(s1_arsize), .arburst(s1_arburst),
        .arvalid(s1_arvalid), .arready(s1_arready),
        .rid(s1_rid), .rdata(s1_rdata), .rresp(s1_rresp),
        .rlast(s1_rlast), .rvalid(s1_rvalid), .rready(s1_rready)
    );
    tb_vram_xbar_sink_slave #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_s2_sink (
        .clk(clk), .rst(rst),
        .awid(s2_awid), .awaddr(s2_awaddr), .awlen(s2_awlen),
        .awsize(s2_awsize), .awburst(s2_awburst),
        .awvalid(s2_awvalid), .awready(s2_awready),
        .wdata(s2_wdata), .wstrb(s2_wstrb), .wlast(s2_wlast),
        .wvalid(s2_wvalid), .wready(s2_wready),
        .bid(s2_bid), .bresp(s2_bresp), .bvalid(s2_bvalid), .bready(s2_bready),
        .arid(s2_arid), .araddr(s2_araddr), .arlen(s2_arlen),
        .arsize(s2_arsize), .arburst(s2_arburst),
        .arvalid(s2_arvalid), .arready(s2_arready),
        .rid(s2_rid), .rdata(s2_rdata), .rresp(s2_rresp),
        .rlast(s2_rlast), .rvalid(s2_rvalid), .rready(s2_rready)
    );
    tb_vram_xbar_sink_slave #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_s4_sink (
        .clk(clk), .rst(rst),
        .awid(s4_awid), .awaddr(s4_awaddr), .awlen(s4_awlen),
        .awsize(s4_awsize), .awburst(s4_awburst),
        .awvalid(s4_awvalid), .awready(s4_awready),
        .wdata(s4_wdata), .wstrb(s4_wstrb), .wlast(s4_wlast),
        .wvalid(s4_wvalid), .wready(s4_wready),
        .bid(s4_bid), .bresp(s4_bresp), .bvalid(s4_bvalid), .bready(s4_bready),
        .arid(s4_arid), .araddr(s4_araddr), .arlen(s4_arlen),
        .arsize(s4_arsize), .arburst(s4_arburst),
        .arvalid(s4_arvalid), .arready(s4_arready),
        .rid(s4_rid), .rdata(s4_rdata), .rresp(s4_rresp),
        .rlast(s4_rlast), .rvalid(s4_rvalid), .rready(s4_rready)
    );
    tb_vram_xbar_sink_slave #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_s5_sink (
        .clk(clk), .rst(rst),
        .awid(s5_awid), .awaddr(s5_awaddr), .awlen(s5_awlen),
        .awsize(s5_awsize), .awburst(s5_awburst),
        .awvalid(s5_awvalid), .awready(s5_awready),
        .wdata(s5_wdata), .wstrb(s5_wstrb), .wlast(s5_wlast),
        .wvalid(s5_wvalid), .wready(s5_wready),
        .bid(s5_bid), .bresp(s5_bresp), .bvalid(s5_bvalid), .bready(s5_bready),
        .arid(s5_arid), .araddr(s5_araddr), .arlen(s5_arlen),
        .arsize(s5_arsize), .arburst(s5_arburst),
        .arvalid(s5_arvalid), .arready(s5_arready),
        .rid(s5_rid), .rdata(s5_rdata), .rresp(s5_rresp),
        .rlast(s5_rlast), .rvalid(s5_rvalid), .rready(s5_rready)
    );

    // ── xbar (3M × 6S) ──────────────────────────────────────────────
    axi_xbar #(
        .DATA_WIDTH(128),
        .ID_WIDTH  (4),
        .N_MASTERS (3),
        .N_SLAVES  (6)
    ) u_xbar (
        .clk(clk), .rst(rst),
        .cpu_overlay_active(1'b0),
        .cpu_overlay_reset(rst),
        .cpu_overlay_disabled(),
        .cpu_overlay_effective(),
        .dbg_slv_poisoned(),
        .dbg_s1_slot_busy(),
        .dbg_ram_window_lg2(6'd26),
        // M0 fan-in select: 0 = CPU LSU side (m0_*), which is the master
        // this tb drives.  The boot-FSM side (m0b_*) is tied off below.
        .cpu_held_in_reset(1'b0),
        // No slave-side reset domain in this harness, so never flush.
        .slv_flush(1'b0),
        // ...and no pb island either, so S1's far side is never warm-reset.
        .s1_far_reset(1'b0),
        // No boot-FSM master in this harness either.
        .m0b_master_reset(1'b0),
        // M0 — active CPU-style master
        .m0_awid(m0_awid), .m0_awaddr(m0_awaddr), .m0_awlen(m0_awlen),
        .m0_awsize(m0_awsize), .m0_awburst(m0_awburst),
        .m0_awvalid(m0_awvalid), .m0_awready(m0_awready),
        .m0_wdata(m0_wdata), .m0_wstrb(m0_wstrb),
        .m0_wlast(m0_wlast), .m0_wvalid(m0_wvalid), .m0_wready(m0_wready),
        .m0_bid(m0_bid), .m0_bresp(m0_bresp),
        .m0_bvalid(m0_bvalid), .m0_bready(m0_bready),
        .m0_arid(m0_arid), .m0_araddr(m0_araddr), .m0_arlen(m0_arlen),
        .m0_arsize(m0_arsize), .m0_arburst(m0_arburst),
        .m0_arvalid(m0_arvalid), .m0_arready(m0_arready),
        .m0_rid(m0_rid), .m0_rdata(m0_rdata),
        .m0_rresp(m0_rresp), .m0_rlast(m0_rlast),
        .m0_rvalid(m0_rvalid), .m0_rready(m0_rready),
        // M0B (boot FSM, write-only), M1 (host debug), M2 (CPU IF,
        // read-only) — all tied off.  The 2026-07-16 master-count
        // reduction retired the old M3/M4 seats: what this tb used to tie
        // off as "M3 (RO, HDMI)" is now M2, and "M2 (WO, boot)" is now the
        // m0b_* fan-in on the shared M0 physical port.
        .m0b_awid(4'd0), .m0b_awaddr(32'd0), .m0b_awlen(8'd0),
        .m0b_awsize(3'd0), .m0b_awburst(2'd0),
        .m0b_awvalid(1'b0), .m0b_awready(),
        .m0b_wdata(128'd0), .m0b_wstrb(16'd0),
        .m0b_wlast(1'b0), .m0b_wvalid(1'b0), .m0b_wready(),
        .m0b_bid(), .m0b_bresp(),
        .m0b_bvalid(), .m0b_bready(1'b1),
        .m1_awid(4'd0), .m1_awaddr(32'd0), .m1_awlen(8'd0),
        .m1_awsize(3'd0), .m1_awburst(2'd0),
        .m1_awvalid(1'b0), .m1_awready(),
        .m1_wdata(128'd0), .m1_wstrb(16'd0),
        .m1_wlast(1'b0), .m1_wvalid(1'b0), .m1_wready(),
        .m1_bid(), .m1_bresp(),
        .m1_bvalid(), .m1_bready(1'b1),
        .m1_arid(4'd0), .m1_araddr(32'd0), .m1_arlen(8'd0),
        .m1_arsize(3'd0), .m1_arburst(2'd0),
        .m1_arvalid(1'b0), .m1_arready(),
        .m1_rid(), .m1_rdata(),
        .m1_rresp(), .m1_rlast(),
        .m1_rvalid(), .m1_rready(1'b1),
        .m2_arid(4'd0), .m2_araddr(32'd0), .m2_arlen(8'd0),
        .m2_arsize(3'd0), .m2_arburst(2'd0),
        .m2_arvalid(1'b0), .m2_arready(),
        .m2_rid(), .m2_rdata(),
        .m2_rresp(), .m2_rlast(),
        .m2_rvalid(), .m2_rready(1'b1),
        // M3 (DDR-backed RAM-disk) was attached after this harness was
        // written.  Keep it quiescent so the xbar is fully specified and
        // the narrow-to-wide VRAM regression remains a meaningful build.
        .m3_awid(4'd0), .m3_awaddr(32'd0), .m3_awlen(8'd0),
        .m3_awsize(3'd0), .m3_awburst(2'd0),
        .m3_awvalid(1'b0), .m3_awready(),
        .m3_wdata(128'd0), .m3_wstrb(16'd0),
        .m3_wlast(1'b0), .m3_wvalid(1'b0), .m3_wready(),
        .m3_bid(), .m3_bresp(),
        .m3_bvalid(), .m3_bready(1'b1),
        .m3_arid(4'd0), .m3_araddr(32'd0), .m3_arlen(8'd0),
        .m3_arsize(3'd0), .m3_arburst(2'd0),
        .m3_arvalid(1'b0), .m3_arready(),
        .m3_rid(), .m3_rdata(),
        .m3_rresp(), .m3_rlast(),
        .m3_rvalid(), .m3_rready(1'b1),
        // Slaves
        .s0_awid(s0_awid), .s0_awaddr(s0_awaddr), .s0_awlen(s0_awlen),
        .s0_awsize(s0_awsize), .s0_awburst(s0_awburst),
        .s0_awvalid(s0_awvalid), .s0_awready(s0_awready),
        .s0_wdata(s0_wdata), .s0_wstrb(s0_wstrb),
        .s0_wlast(s0_wlast), .s0_wvalid(s0_wvalid), .s0_wready(s0_wready),
        .s0_bid(s0_bid), .s0_bresp(s0_bresp),
        .s0_bvalid(s0_bvalid), .s0_bready(s0_bready),
        .s0_arid(s0_arid), .s0_araddr(s0_araddr), .s0_arlen(s0_arlen),
        .s0_arsize(s0_arsize), .s0_arburst(s0_arburst),
        .s0_arvalid(s0_arvalid), .s0_arready(s0_arready),
        .s0_rid(s0_rid), .s0_rdata(s0_rdata),
        .s0_rresp(s0_rresp), .s0_rlast(s0_rlast),
        .s0_rvalid(s0_rvalid), .s0_rready(s0_rready),
        .s1_awid(s1_awid), .s1_awaddr(s1_awaddr), .s1_awlen(s1_awlen),
        .s1_awsize(s1_awsize), .s1_awburst(s1_awburst),
        .s1_awvalid(s1_awvalid), .s1_awready(s1_awready),
        .s1_wdata(s1_wdata), .s1_wstrb(s1_wstrb),
        .s1_wlast(s1_wlast), .s1_wvalid(s1_wvalid), .s1_wready(s1_wready),
        .s1_bid(s1_bid), .s1_bresp(s1_bresp),
        .s1_bvalid(s1_bvalid), .s1_bready(s1_bready),
        .s1_arid(s1_arid), .s1_araddr(s1_araddr), .s1_arlen(s1_arlen),
        .s1_arsize(s1_arsize), .s1_arburst(s1_arburst),
        .s1_arvalid(s1_arvalid), .s1_arready(s1_arready),
        .s1_rid(s1_rid), .s1_rdata(s1_rdata),
        .s1_rresp(s1_rresp), .s1_rlast(s1_rlast),
        .s1_rvalid(s1_rvalid), .s1_rready(s1_rready),
        .s2_awid(s2_awid), .s2_awaddr(s2_awaddr), .s2_awlen(s2_awlen),
        .s2_awsize(s2_awsize), .s2_awburst(s2_awburst),
        .s2_awvalid(s2_awvalid), .s2_awready(s2_awready),
        .s2_wdata(s2_wdata), .s2_wstrb(s2_wstrb),
        .s2_wlast(s2_wlast), .s2_wvalid(s2_wvalid), .s2_wready(s2_wready),
        .s2_bid(s2_bid), .s2_bresp(s2_bresp),
        .s2_bvalid(s2_bvalid), .s2_bready(s2_bready),
        .s2_arid(s2_arid), .s2_araddr(s2_araddr), .s2_arlen(s2_arlen),
        .s2_arsize(s2_arsize), .s2_arburst(s2_arburst),
        .s2_arvalid(s2_arvalid), .s2_arready(s2_arready),
        .s2_rid(s2_rid), .s2_rdata(s2_rdata),
        .s2_rresp(s2_rresp), .s2_rlast(s2_rlast),
        .s2_rvalid(s2_rvalid), .s2_rready(s2_rready),
        .s3_awid(s3_awid), .s3_awaddr(s3_awaddr), .s3_awlen(s3_awlen),
        .s3_awsize(s3_awsize), .s3_awburst(s3_awburst),
        .s3_awvalid(s3_awvalid), .s3_awready(s3_awready),
        .s3_wdata(s3_wdata), .s3_wstrb(s3_wstrb),
        .s3_wlast(s3_wlast), .s3_wvalid(s3_wvalid), .s3_wready(s3_wready),
        .s3_bid(s3_bid), .s3_bresp(s3_bresp),
        .s3_bvalid(s3_bvalid), .s3_bready(s3_bready),
        .s3_arid(s3_arid), .s3_araddr(s3_araddr), .s3_arlen(s3_arlen),
        .s3_arsize(s3_arsize), .s3_arburst(s3_arburst),
        .s3_arvalid(s3_arvalid), .s3_arready(s3_arready),
        .s3_rid(s3_rid), .s3_rdata(s3_rdata),
        .s3_rresp(s3_rresp), .s3_rlast(s3_rlast),
        .s3_rvalid(s3_rvalid), .s3_rready(s3_rready),
        .s4_awid(s4_awid), .s4_awaddr(s4_awaddr), .s4_awlen(s4_awlen),
        .s4_awsize(s4_awsize), .s4_awburst(s4_awburst),
        .s4_awvalid(s4_awvalid), .s4_awready(s4_awready),
        .s4_wdata(s4_wdata), .s4_wstrb(s4_wstrb),
        .s4_wlast(s4_wlast), .s4_wvalid(s4_wvalid), .s4_wready(s4_wready),
        .s4_bid(s4_bid), .s4_bresp(s4_bresp),
        .s4_bvalid(s4_bvalid), .s4_bready(s4_bready),
        .s4_arid(s4_arid), .s4_araddr(s4_araddr), .s4_arlen(s4_arlen),
        .s4_arsize(s4_arsize), .s4_arburst(s4_arburst),
        .s4_arvalid(s4_arvalid), .s4_arready(s4_arready),
        .s4_rid(s4_rid), .s4_rdata(s4_rdata),
        .s4_rresp(s4_rresp), .s4_rlast(s4_rlast),
        .s4_rvalid(s4_rvalid), .s4_rready(s4_rready),
        .s5_awid(s5_awid), .s5_awaddr(s5_awaddr), .s5_awlen(s5_awlen),
        .s5_awsize(s5_awsize), .s5_awburst(s5_awburst),
        .s5_awvalid(s5_awvalid), .s5_awready(s5_awready),
        .s5_wdata(s5_wdata), .s5_wstrb(s5_wstrb),
        .s5_wlast(s5_wlast), .s5_wvalid(s5_wvalid), .s5_wready(s5_wready),
        .s5_bid(s5_bid), .s5_bresp(s5_bresp),
        .s5_bvalid(s5_bvalid), .s5_bready(s5_bready),
        .s5_arid(s5_arid), .s5_araddr(s5_araddr), .s5_arlen(s5_arlen),
        .s5_arsize(s5_arsize), .s5_arburst(s5_arburst),
        .s5_arvalid(s5_arvalid), .s5_arready(s5_arready),
        .s5_rid(s5_rid), .s5_rdata(s5_rdata),
        .s5_rresp(s5_rresp), .s5_rlast(s5_rlast),
        .s5_rvalid(s5_rvalid), .s5_rready(s5_rready)
    );

    // ── vram @ S3 ────────────────────────────────────────────────────
    // The xbar strips VRAM_BASE before driving s3_*addr, so the vram
    // slave sees a zero-based offset — matching production fpga_top.v.
    vram #(
        .FB_WIDTH_PX (FB_W),
        .FB_HEIGHT_PX(FB_H),
        // BPP is the pixel packing / read-port address granularity;
        // RD_DATA_W is the (widened) read-port DATA width, and vram.v requires
        // it to be exactly 4*BPP (a 4-lane gather with every lane driven).
        .BPP         (FB_BPP),
        .RD_DATA_W   (4 * FB_BPP),
        // Size the aperture to exactly the harness framebuffer.  vram.v's
        // VRAM_BYTES default is the 2 MiB Q700 aperture, and PX_ADDR_W is
        // derived from max(VRAM_BYTES/(BPP/8), FB_W*FB_H) — so leaving the
        // default widened rd_addr to 21 bits while this wrapper's local
        // RD_ADDR_W (and tb_vram_xbar_e2e.cpp's 0x1800 out-of-bounds
        // scenario) both assume the 6144-byte FB sizing.  Passing it
        // explicitly is the sizing vram.v's own header documents for this
        // case, and it is what makes the OOB scenario actually out of
        // bounds.
        .VRAM_BYTES  (FB_W * FB_H * FB_BPP / 8),
        .DATA_WIDTH  (128),
        .ID_WIDTH    (6)
    ) u_vram (
        .clk       (clk),
        .rst       (rst),
        .clear_req (1'b0),
        .s_awid    (s3_awid), .s_awaddr(s3_awaddr),
        .s_awlen   (s3_awlen), .s_awsize(s3_awsize),
        .s_awburst (s3_awburst), .s_awvalid(s3_awvalid),
        .s_awready (s3_awready),
        .s_wdata   (s3_wdata), .s_wstrb(s3_wstrb),
        .s_wlast   (s3_wlast), .s_wvalid(s3_wvalid),
        .s_wready  (s3_wready),
        .s_bid     (s3_bid), .s_bresp(s3_bresp),
        .s_bvalid  (s3_bvalid), .s_bready(s3_bready),
        .s_arid    (s3_arid), .s_araddr(s3_araddr),
        .s_arlen   (s3_arlen), .s_arsize(s3_arsize),
        .s_arburst (s3_arburst), .s_arvalid(s3_arvalid),
        .s_arready (s3_arready),
        .s_rid     (s3_rid), .s_rdata(s3_rdata),
        .s_rresp   (s3_rresp), .s_rlast(s3_rlast),
        .s_rvalid  (s3_rvalid), .s_rready(s3_rready),
        .rd_clk    (clk), .rd_rst(rst),
        .rd_addr   (rd_addr[RD_ADDR_W-1:0]),
        .rd_en     (rd_en),
        .rd_data   (rd_data),
        .rd_valid  (rd_valid)
    );

endmodule

// ── Simple always-ready sink slave for xbar S0/S1/S2 tie-off ────────────
// Accepts any AW+W sequence, drops data, emits BVALID after W final beat.
// Accepts any AR, returns len+1 beats of zero RDATA.  No memory modelled —
// the tb only cares that the xbar does NOT block when those slaves are
// addressed (they aren't, by the scenarios below, but if routing ever
// regresses and a write leaks to DDR we'd rather it not deadlock the tb).
module tb_vram_xbar_sink_slave #(
    parameter ID_WIDTH   = 6,
    parameter DATA_WIDTH = 128
) (
    input  wire                    clk,
    input  wire                    rst,
    input  wire [ID_WIDTH-1:0]     awid,
    input  wire [31:0]             awaddr,
    input  wire [7:0]              awlen,
    input  wire [2:0]              awsize,
    input  wire [1:0]              awburst,
    input  wire                    awvalid,
    output reg                     awready,
    input  wire [DATA_WIDTH-1:0]   wdata,
    input  wire [DATA_WIDTH/8-1:0] wstrb,
    input  wire                    wlast,
    input  wire                    wvalid,
    output reg                     wready,
    output reg  [ID_WIDTH-1:0]     bid,
    output reg  [1:0]              bresp,
    output reg                     bvalid,
    input  wire                    bready,
    input  wire [ID_WIDTH-1:0]     arid,
    input  wire [31:0]             araddr,
    input  wire [7:0]              arlen,
    input  wire [2:0]              arsize,
    input  wire [1:0]              arburst,
    input  wire                    arvalid,
    output reg                     arready,
    output reg  [ID_WIDTH-1:0]     rid,
    output reg  [DATA_WIDTH-1:0]   rdata,
    output reg  [1:0]              rresp,
    output reg                     rlast,
    output reg                     rvalid,
    input  wire                    rready
);
    // Write side — 3-state: IDLE → WDATA → BRESP.
    reg [1:0] ws;
    reg [ID_WIDTH-1:0] ws_id;

    // Read side — 3-state: IDLE → R beats.
    reg [1:0] rs;
    reg [ID_WIDTH-1:0] rs_id;
    reg [7:0]          rs_left;

    always @(posedge clk) begin
        if (rst) begin
            ws <= 2'd0; awready <= 1'b0; wready <= 1'b0; bvalid <= 1'b0;
            bid <= {ID_WIDTH{1'b0}}; bresp <= 2'b00; ws_id <= {ID_WIDTH{1'b0}};
            rs <= 2'd0; arready <= 1'b0; rvalid <= 1'b0; rlast <= 1'b0;
            rid <= {ID_WIDTH{1'b0}}; rdata <= {DATA_WIDTH{1'b0}};
            rresp <= 2'b00; rs_id <= {ID_WIDTH{1'b0}}; rs_left <= 8'd0;
        end else begin
            case (ws)
                2'd0: begin
                    awready <= 1'b1;
                    wready  <= 1'b0;
                    bvalid  <= 1'b0;
                    if (awvalid && awready) begin
                        ws_id   <= awid;
                        awready <= 1'b0;
                        wready  <= 1'b1;
                        ws      <= 2'd1;
                    end
                end
                2'd1: begin
                    wready <= 1'b1;
                    if (wvalid && wready && wlast) begin
                        wready <= 1'b0;
                        bvalid <= 1'b1;
                        bid    <= ws_id;
                        bresp  <= 2'b00;
                        ws     <= 2'd2;
                    end
                end
                2'd2: begin
                    if (bvalid && bready) begin
                        bvalid <= 1'b0;
                        awready <= 1'b1;
                        ws      <= 2'd0;
                    end
                end
                default: ws <= 2'd0;
            endcase
            case (rs)
                2'd0: begin
                    arready <= 1'b1;
                    rvalid  <= 1'b0;
                    rlast   <= 1'b0;
                    if (arvalid && arready) begin
                        rs_id   <= arid;
                        rs_left <= arlen + 8'd1;
                        arready <= 1'b0;
                        rs      <= 2'd1;
                    end
                end
                2'd1: begin
                    rvalid <= 1'b1;
                    rid    <= rs_id;
                    rdata  <= {DATA_WIDTH{1'b0}};
                    rresp  <= 2'b00;
                    rlast  <= (rs_left == 8'd1);
                    if (rvalid && rready) begin
                        if (rs_left == 8'd1) begin
                            rvalid  <= 1'b0;
                            rlast   <= 1'b0;
                            arready <= 1'b1;
                            rs      <= 2'd0;
                        end else begin
                            rs_left <= rs_left - 8'd1;
                        end
                    end
                end
                default: rs <= 2'd0;
            endcase
        end
    end
endmodule

`default_nettype wire
