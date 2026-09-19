// tb_scanout_ddr_frames.v -- MULTI-FRAME scanout over the REAL fetch chain.
// ---------------------------------------------------------------------------
// WHY THIS EXISTS (the coverage hole this closes)
//
// Before this file there was NO test anywhere that ran the whole scanout
// chain.  The two halves were each covered, separately, and the seam between
// them was not:
//
//   tb-scanout-frames        vtg + linebuf_scanout, fetch port modelled in
//                            C++ (fixed latency, always-ordered, never
//                            line-granular).  Multi-frame, pixel-exact.
//   tb-fb-reader-ddr-chain   fb_reader + scanout_ddr_reader + the DDR chain,
//                            driven by a C++ request stream.  Explicitly
//                            "skips scanout_placement_sync/linebuf_scanout".
//   tb-video-smoke-ddr       vram_smoke -> DDR -> scanout_ddr_reader port,
//                            read back directly.  No scanner, no frames.
//
// The hardware symptom this file was written for -- whole frames rendering
// with the source origin displaced by exactly one 128 B ring line
// (scanout_line_fetch.v's LINE_BYTES), roughly one frame in ten, on an
// otherwise static framebuffer -- lives EXACTLY in that seam: it needs the
// scanner's per-frame resync walk (linebuf_scanout) driving the line-granular
// DDR ring (scanout_line_fetch) across a real frame boundary.  Neither half's
// tb can produce it alone.
//
// So this instantiates BOTH halves, unmodified, at the shipping geometry, and
// walks several consecutive frames.
//
// WHAT IT ASSERTS (see tb_scanout_ddr_frames.cpp)
//   1. FRAME-TO-FRAME IDENTITY.  The framebuffer is painted once and never
//      written again, so consecutive frames MUST be byte-identical.  This is
//      the assertion tb_scanout_frames never had, and it is the one that
//      catches an origin slip: a slipped frame is internally self-consistent,
//      so any single-frame checker that only looks at "is this line's colour
//      plausible" can pass it.
//   2. ABSOLUTE per-source-row correctness, which subsumes (1) but is stated
//      separately so a failure names WHICH of the two broke.
//
// CLOCKING.  Three genuinely independent clock nets at the production
// frequency RATIOS (pclk 148.5 MHz, core_clk 100 MHz, mig_clk 200 MHz).  The
// ratio is load-bearing here and not a detail: the scanner requests at pclk
// and the DDR ring services at core_clk, so pclk > core_clk is what makes the
// request stream outrun the fetch engine and back up in fb_reader's CDC FIFO.
// A 1:1 rig (which is what tb_fb_reader_ddr_chain uses, correctly, for its own
// much narrower purpose) does not reproduce that pressure.
//
// PLACEMENT.  scanout_placement_sync is omitted deliberately: every scenario
// holds base/stride/depth constant for the whole run, so the sync stage would
// be a pass-through.  tb-scanout-placement-sync covers it on its own.
// ---------------------------------------------------------------------------
`default_nettype none
`include "axi_defs.vh"

module tb_scanout_ddr_frames #(
    parameter SRC_W          = 1024,
    parameter SRC_H          = 768,
    parameter DST_W          = 1920,
    parameter DST_H          = 1080,
    parameter ADDR_W         = 22,             // 4 MB aperture (VRAM_IN_DDR)
    parameter FETCH_W        = 32,
    parameter FB_MAX_PIXELS  = 32'h0040_0000,
    parameter LINE_COUNT_LOG2 = 6,
    // scanout_ddr_reader's request-queue depth.  6 is PRODUCTION.  The
    // harness can raise it to measure the true peak occupancy without
    // tripping the RTL's own overflow $fatal.
    parameter QDEPTH_LOG2    = 6,
    // ── DDR read round trip (see below -- THIS IS AN ASSUMPTION) ──────
    // There is NO measured real-hardware DDR read latency anywhere in this
    // repo, and every scanout margin claim depends on one.  40 +/- 8 core
    // clocks is the realistic figure the T14 writeup assumed; the previous
    // version of this harness modelled ZERO (sim_mig_backend's default),
    // which made every scanout deadline trivially reachable and hid the
    // fact that v1's fetch engine was latency-bound.  Overridable with
    // -GDDR_READ_LATENCY_BASE=... so the conclusions can be shown to be
    // insensitive to it (the gate sweeps 0 / 40 / 200).
    parameter DDR_READ_LATENCY_BASE   = 40,
    parameter DDR_READ_LATENCY_JITTER = 8,
    // axi_vram_priority_mux3's non-scan admission bound.  Exposed so the
    // harness can prove where the safe ceiling is instead of asserting it.
    parameter MUX_MAX_BULK_AHEAD = 4,
    // Bulk (CPU L2 fill) burst length in 128-bit beats.  4 beats = one
    // 64 B L2C line, which is what l2c_mshr.v actually issues.
    parameter BULK_ARLEN = 3
) (
    input  wire                 pclk,
    input  wire                 core_clk,
    input  wire                 mig_clk,
    input  wire                 rst,          // fabric reset (mux/bridge/MIG)
    // Holds the SCANNER half (vtg, linebuf_scanout, fb_reader,
    // scanout_ddr_reader) in reset while the fabric is live, so the harness
    // can paint the framebuffer over the CPU write port without the scanner
    // competing for the same DDR port for tens of thousands of beats.
    input  wire                 scan_hold,
    input  wire                 mig_rst,

    // ── WORST-CASE ARBITRATION PRESSURE ───────────────────────────────
    // 1 = drive the mux's l2c_* AR port with a saturating CPU-miss storm
    // (back-to-back 4-beat reads, always pending, responses always
    // accepted).  This is the ONLY state in which MAX_BULK_AHEAD is
    // reachable, so it is the only state in which the scanout deadline is
    // actually under test.  Steady-state scenarios never produce it.
    input  wire                 bulk_pressure,

    // ── Runtime placement / depth (harness-driven, constant per run) ──
    input  wire [ADDR_W-1:0]    fb_base_px,
    input  wire [ADDR_W-1:0]    fb_stride_px,
    input  wire [2:0]           bpp_shift,
    input  wire [2:0]           bytes_per_px,
    input  wire [11:0]          hres,
    input  wire [11:0]          vres,
    // INTEGER output scale (place_plan.v's N): N display pixels per source
    // pixel on both axes.  The harness drives the value the SETTLED POLICY
    // picks for the scenario's geometry -- see policy_scale_n() in the .cpp
    // -- rather than a per-scenario ratio constant, so no scenario can state
    // a ratio the shipping policy would not choose.
    input  wire [2:0]           scale_n,

    // ── CLUT write port (pclk here; core_clk in production) ──────────
    input  wire                 clut_we,
    input  wire [7:0]           clut_waddr,
    input  wire [23:0]          clut_wdata,

    // ── Raw CPU-shaped AXI4 write port, straight into the carveout ────
    // Used only to paint the framebuffer before the frames start.  Same
    // arrangement (and same no-swap rationale) as tb_fb_reader_ddr_chain.v.
    input  wire [5:0]   cpu_awid,   input wire [31:0] cpu_awaddr,
    input  wire [7:0]   cpu_awlen,  input wire [2:0]  cpu_awsize,
    input  wire [1:0]   cpu_awburst,
    input  wire         cpu_awvalid, output wire cpu_awready,
    input  wire [127:0] cpu_wdata,  input wire [15:0] cpu_wstrb,
    input  wire         cpu_wlast,  input wire cpu_wvalid, output wire cpu_wready,
    output wire [5:0]   cpu_bid,    output wire [1:0] cpu_bresp,
    output wire         cpu_bvalid, input  wire cpu_bready,

    // ── Scanned-out video ─────────────────────────────────────────────
    output wire [23:0]          rgb,
    output wire                 de_out,
    output wire                 hs_out,
    output wire                 vs_out,
    output wire                 line_underflow_sticky,
    output wire                 fb_reader_underflow_sticky,

    output wire [11:0]          hcount,
    output wire [10:0]          vcount,
    output wire                 de_vtg,
    output wire                 cal_done,
    // The ELABORATED scanner bound.  The .cpp hardcodes nothing about it:
    // tb-scanout-first-pixel builds this same source at SRC 1152x1024, and a
    // scenario wider or taller than the bound would be silently CROPPED and
    // the checker would measure the crop instead of the mode.  Reporting the
    // bound lets the scenario table carry the whole Q700 mode set and skip,
    // loudly, the rows a given elaboration cannot represent.
    output wire [15:0]          dbg_src_w,
    output wire [15:0]          dbg_src_h,

    // ── Fetch-walk observability ──────────────────────────────────────
    output wire [2:0]           dbg_state,
    output wire [15:0]          dbg_credits,
    output wire [15:0]          dbg_outstanding,
    output wire [15:0]          dbg_stale_count,
    output wire [15:0]          dbg_req_y,
    output wire [15:0]          dbg_req_x,
    output wire [15:0]          dbg_rsp_y,
    output wire [15:0]          dbg_rsp_x,
    output wire [15:0]          dbg_y_src,
    // Byte-stream accounting at the scanner's own fetch port: how many
    // requests it has had ACCEPTED and how many responses it has CONSUMED
    // since reset.  A response stream that has slipped relative to the
    // request stream shows up here as a persistent difference that is not
    // explained by `outstanding`.
    output wire [31:0]          dbg_req_accepted,
    output wire [31:0]          dbg_rsp_consumed,
    // Peak occupancy of scanout_ddr_reader's request queue.  Nothing in the
    // chain enforces the "caller must not exceed the outstanding-request
    // contract" this queue documents, so this is the number that says whether
    // the production QDEPTH is actually big enough.
    output reg  [15:0]          dbg_q_peak,

    // ── Arbitration / slack observability (see tb_scanout_ddr_frames.cpp)
    // Deliberately built from PORT-VISIBLE signals only (AR/R handshakes
    // and the reader's own queue), never from the fetch engine's internal
    // state, so the SAME harness measures the pre-rework and post-rework
    // scanout_line_fetch.v and the numbers are comparable.
    output reg  [31:0]          dbg_bulk_bursts,     // completed bulk reads
    output reg  [31:0]          dbg_bulk_cycles,     // core_clk under pressure
    output reg  [31:0]          dbg_scan_bursts,     // scan ARs accepted
    output reg  [31:0]          dbg_scan_excursions, // idle -> busy transitions
    output reg  [31:0]          dbg_scan_stall,      // cycles the drain was starved
    output reg  [15:0]          dbg_scan_outst_max   // peak scan bursts in flight
);

    localparam SRC_Y_W = $clog2(SRC_H);
    localparam SRC_X_W = $clog2(SRC_W);

    wire scan_rst = rst || scan_hold;
    wire resetn = ~scan_rst;
    wire vtg_de, vtg_hs, vtg_vs;

    vtg #(
        .H_ACTIVE(DST_W),
        .V_ACTIVE(DST_H)
    ) u_vtg (
        .pclk   (pclk),
        .resetn (resetn),
        .hcount (hcount),
        .vcount (vcount),
        .de     (vtg_de),
        .hsync  (vtg_hs),
        .vsync  (vtg_vs)
    );
    assign de_vtg = vtg_de;
    localparam [15:0] SRC_W_16 = SRC_W;
    localparam [15:0] SRC_H_16 = SRC_H;
    assign dbg_src_w = SRC_W_16;
    assign dbg_src_h = SRC_H_16;

    // ── Scanner (pclk) ────────────────────────────────────────────────
    wire                sc_rd_en;
    wire [ADDR_W-1:0]   sc_rd_addr;
    wire                sc_rd_ready;
    wire [FETCH_W-1:0]  sc_rd_data;
    wire                sc_rd_valid;

    linebuf_scanout #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        .FB_MAX_PIXELS   (FB_MAX_PIXELS),
        .DST_W           (DST_W),
        .DST_H           (DST_H),
        .FETCH_W         (FETCH_W),
        .ADDR_W          (ADDR_W),
        .LINE_COUNT_LOG2 (LINE_COUNT_LOG2)
    ) u_scanout (
        .pclk        (pclk),
        .resetn      (resetn),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (vtg_de),
        .hs_in       (vtg_hs),
        .vs_in       (vtg_vs),
        .fb_base_px  (fb_base_px),
        .fb_stride_px(fb_stride_px),
        .bpp_shift   (bpp_shift),
        .bytes_per_px(bytes_per_px),
        .hres        (hres),
        .vres        (vres),
        .scale_n     (scale_n),
        // Existing scanout tbs cover the NORMAL video path, so the boot
        // splash is retired here (dafb_live=1).  tb_scanout_frames.v drives
        // it low to cover the splash itself.
        .dafb_live   (1'b1),
        .clut_wclk   (pclk),
        .clut_we     (clut_we),
        .clut_waddr  (clut_waddr),
        .clut_wdata  (clut_wdata),
        .fb_rd_en    (sc_rd_en),
        .fb_rd_addr  (sc_rd_addr),
        .fb_rd_ready (sc_rd_ready),
        .fb_rd_data  (sc_rd_data),
        .fb_rd_valid (sc_rd_valid),
        .rgb         (rgb),
        .de_out      (de_out),
        .hs_out      (hs_out),
        .vs_out      (vs_out),
        .line_underflow_sticky(line_underflow_sticky)
    );

    // ── fb_reader CDC bridge (pclk <-> core_clk) ──────────────────────
    // Parameters mirror video_top.v's PRODUCTION instantiation, including
    // its INFLIGHT_WIDTH (left at the default there).  Deliberately NOT
    // retuned to 6 the way tb_fb_reader_ddr_chain.v does: a rig that fixes
    // a parameter production does not fix is testing a design that does not
    // ship.
    wire              v_rd_en;
    wire [ADDR_W-1:0] v_rd_addr;
    wire [31:0]       v_rd_data;
    wire              v_rd_valid;

    fb_reader #(
        .ADDR_W        (ADDR_W),
        .DATA_W        (FETCH_W),
        .RETURN_LATENCY(48)
    ) u_fb_reader (
        .pclk       (pclk),
        .resetn     (resetn),
        .vram_clk   (core_clk),
        .vram_rst   (scan_rst),
        .s_rd_en    (sc_rd_en),
        .s_rd_addr  (sc_rd_addr),
        .s_rd_ready (sc_rd_ready),
        .s_rd_data  (sc_rd_data),
        .s_rd_valid (sc_rd_valid),
        .v_rd_en    (v_rd_en),
        .v_rd_addr  (v_rd_addr),
        .v_rd_data  (v_rd_data),
        .v_rd_valid (v_rd_valid),
        .underflow_sticky(fb_reader_underflow_sticky),
        .req_count  (),
        .rsp_count  (),
        .miss_count ()
    );

    // ── DDR-backed streaming read port (core_clk) ─────────────────────
    wire [5:0]   scan_arid;   wire [31:0]  scan_araddr; wire [7:0] scan_arlen;
    wire [2:0]   scan_arsize; wire [1:0]   scan_arburst;
    wire scan_arvalid, scan_arready;
    wire [5:0]   scan_rid;    wire [127:0] scan_rdata;  wire [1:0] scan_rresp;
    wire scan_rlast, scan_rvalid, scan_rready;

    scanout_ddr_reader #(
        .ADDR_W(ADDR_W), .BPP(8), .RD_DATA_W(32),
        .DATA_WIDTH(128), .ID_WIDTH(6), .AXI_ID(6'h00),
        .CARVEOUT_BASE(`AXI_VRAM_DDR_CARVEOUT_BASE),
        .CARVEOUT_SIZE(`AXI_VRAM_DDR_CARVEOUT_SIZE),
        .QDEPTH_LOG2(QDEPTH_LOG2)
    ) u_scan_reader (
        .clk(core_clk), .rst(scan_rst),
        .rd_clk(core_clk), .rd_rst(scan_rst),
        .rd_addr(v_rd_addr), .rd_en(v_rd_en),
        .rd_data(v_rd_data), .rd_valid(v_rd_valid),
        .m_arid(scan_arid), .m_araddr(scan_araddr), .m_arlen(scan_arlen),
        .m_arsize(scan_arsize), .m_arburst(scan_arburst),
        .m_arvalid(scan_arvalid), .m_arready(scan_arready),
        .m_rid(scan_rid), .m_rdata(scan_rdata), .m_rresp(scan_rresp),
        .m_rlast(scan_rlast), .m_rvalid(scan_rvalid), .m_rready(scan_rready)
    );

    // ── VRAM-lane arbiter + async bridge + MIG model ──────────────────
    wire [5:0]   m_awid;   wire [31:0]  m_awaddr;  wire [7:0] m_awlen;
    wire [2:0]   m_awsize; wire [1:0]   m_awburst; wire m_awvalid, m_awready;
    wire [127:0] m_wdata;  wire [15:0]  m_wstrb;   wire m_wlast, m_wvalid, m_wready;
    wire [5:0]   m_bid;    wire [1:0]   m_bresp;   wire m_bvalid, m_bready;
    wire [5:0]   m_arid;   wire [31:0]  m_araddr;  wire [7:0] m_arlen;
    wire [2:0]   m_arsize; wire [1:0]   m_arburst; wire m_arvalid, m_arready;
    wire [5:0]   m_rid;    wire [127:0] m_rdata;   wire [1:0] m_rresp;
    wire m_rlast, m_rvalid, m_rready;

    // ── Saturating bulk-read generator (the "CPU miss storm") ─────────
    // Back-to-back 4-beat INCR reads at a rolling 64 B-line address inside
    // the carveout (reads never disturb the painted framebuffer), presented
    // continuously so `l2c_arvalid` is ALWAYS high while enabled.  That is
    // what makes axi_vram_priority_mux3's MAX_BULK_AHEAD and MAX_SCAN_AHEAD
    // terms reachable; without it neither ever fires and the arbiter is
    // effectively untested.
    reg  [31:0] bulk_araddr;
    wire        bulk_arvalid = bulk_pressure;
    wire        bulk_arready;
    wire        bulk_rvalid, bulk_rlast;

    always @(posedge core_clk) begin
        if (rst) begin
            bulk_araddr <= `AXI_VRAM_DDR_CARVEOUT_BASE;
        end else if (bulk_arvalid && bulk_arready) begin
            // 64 B stride, wrapping inside the low 1 MB of the carveout.
            bulk_araddr <= `AXI_VRAM_DDR_CARVEOUT_BASE |
                           {12'd0, (bulk_araddr[19:0] + 20'd64)};
        end
    end

    axi_vram_priority_mux3 #(.ID_WIDTH(6), .ADDR_WIDTH(32), .DATA_WIDTH(128),
                             .MAX_BULK_AHEAD(MUX_MAX_BULK_AHEAD)) u_mux (
        .clk(core_clk), .rst(rst),
        .l2c_awid(cpu_awid), .l2c_awaddr(cpu_awaddr), .l2c_awlen(cpu_awlen),
        .l2c_awsize(cpu_awsize), .l2c_awburst(cpu_awburst),
        .l2c_awvalid(cpu_awvalid), .l2c_awready(cpu_awready),
        .l2c_wdata(cpu_wdata), .l2c_wstrb(cpu_wstrb), .l2c_wlast(cpu_wlast),
        .l2c_wvalid(cpu_wvalid), .l2c_wready(cpu_wready),
        .l2c_bid(cpu_bid), .l2c_bresp(cpu_bresp), .l2c_bvalid(cpu_bvalid),
        .l2c_bready(cpu_bready),
        .l2c_arid(6'd1), .l2c_araddr(bulk_araddr), .l2c_arlen(BULK_ARLEN[7:0]),
        .l2c_arsize(3'd4), .l2c_arburst(2'b01),
        .l2c_arvalid(bulk_arvalid), .l2c_arready(bulk_arready),
        .l2c_rid(), .l2c_rdata(), .l2c_rresp(), .l2c_rlast(bulk_rlast),
        .l2c_rvalid(bulk_rvalid),
        .l2c_rready(1'b1),
        .s3_awid(6'd0), .s3_awaddr(32'd0), .s3_awlen(8'd0),
        .s3_awsize(3'd0), .s3_awburst(2'd0),
        .s3_awvalid(1'b0), .s3_awready(),
        .s3_wdata(128'd0), .s3_wstrb(16'd0), .s3_wlast(1'b0),
        .s3_wvalid(1'b0), .s3_wready(),
        .s3_bid(), .s3_bresp(), .s3_bvalid(), .s3_bready(1'b1),
        .s3_arid(6'd0), .s3_araddr(32'd0), .s3_arlen(8'd0),
        .s3_arsize(3'd0), .s3_arburst(2'd0),
        .s3_arvalid(1'b0), .s3_arready(),
        .s3_rid(), .s3_rdata(), .s3_rresp(), .s3_rlast(), .s3_rvalid(),
        .s3_rready(1'b1),
        .scan_arid(scan_arid), .scan_araddr(scan_araddr), .scan_arlen(scan_arlen),
        .scan_arsize(scan_arsize), .scan_arburst(scan_arburst),
        .scan_arvalid(scan_arvalid), .scan_arready(scan_arready),
        .scan_rid(scan_rid), .scan_rdata(scan_rdata), .scan_rresp(scan_rresp),
        .scan_rlast(scan_rlast), .scan_rvalid(scan_rvalid), .scan_rready(scan_rready),
        .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen),
        .m_awsize(m_awsize), .m_awburst(m_awburst),
        .m_awvalid(m_awvalid), .m_awready(m_awready),
        .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast),
        .m_wvalid(m_wvalid), .m_wready(m_wready),
        .m_bid(m_bid), .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
        .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen),
        .m_arsize(m_arsize), .m_arburst(m_arburst),
        .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp),
        .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready)
    );

    wire [5:0]   ui_awid;   wire [31:0]  ui_awaddr;  wire [7:0] ui_awlen;
    wire [2:0]   ui_awsize; wire [1:0]   ui_awburst; wire ui_awvalid, ui_awready;
    wire [127:0] ui_wdata;  wire [15:0]  ui_wstrb;   wire ui_wlast, ui_wvalid, ui_wready;
    wire [5:0]   ui_bid;    wire [1:0]   ui_bresp;   wire ui_bvalid, ui_bready;
    wire [5:0]   ui_arid;   wire [31:0]  ui_araddr;  wire [7:0] ui_arlen;
    wire [2:0]   ui_arsize; wire [1:0]   ui_arburst; wire ui_arvalid, ui_arready;
    wire [5:0]   ui_rid;    wire [127:0] ui_rdata;   wire [1:0] ui_rresp;
    wire ui_rlast, ui_rvalid, ui_rready;

    axi_async_bridge #(.DATA_WIDTH(128), .ADDR_WIDTH(32), .ID_WIDTH(6), .USER_WIDTH(1)) u_bridge (
        .s_clk(core_clk), .s_rst(rst),
        .s_awid(m_awid), .s_awaddr(m_awaddr), .s_awlen(m_awlen),
        .s_awsize(m_awsize), .s_awburst(m_awburst), .s_awlock(1'b0),
        .s_awcache(4'b0011), .s_awprot(3'b000), .s_awqos(4'b0000),
        .s_awuser(1'b0), .s_awvalid(m_awvalid), .s_awready(m_awready),
        .s_wdata(m_wdata), .s_wstrb(m_wstrb), .s_wlast(m_wlast),
        .s_wuser(1'b0), .s_wvalid(m_wvalid), .s_wready(m_wready),
        .s_bid(m_bid), .s_bresp(m_bresp), .s_buser(), .s_bvalid(m_bvalid),
        .s_bready(m_bready),
        .s_arid(m_arid), .s_araddr(m_araddr), .s_arlen(m_arlen),
        .s_arsize(m_arsize), .s_arburst(m_arburst), .s_arlock(1'b0),
        .s_arcache(4'b0011), .s_arprot(3'b000), .s_arqos(4'b0000),
        .s_aruser(1'b0), .s_arvalid(m_arvalid), .s_arready(m_arready),
        .s_rid(m_rid), .s_rdata(m_rdata), .s_rresp(m_rresp), .s_rlast(m_rlast),
        .s_ruser(), .s_rvalid(m_rvalid), .s_rready(m_rready),

        .m_clk(mig_clk), .m_rst(mig_rst || !cal_done),
        .m_awid(ui_awid), .m_awaddr(ui_awaddr), .m_awlen(ui_awlen),
        .m_awsize(ui_awsize), .m_awburst(ui_awburst), .m_awlock(),
        .m_awcache(), .m_awprot(), .m_awqos(),
        .m_awuser(), .m_awvalid(ui_awvalid), .m_awready(ui_awready),
        .m_wdata(ui_wdata), .m_wstrb(ui_wstrb), .m_wlast(ui_wlast),
        .m_wuser(), .m_wvalid(ui_wvalid), .m_wready(ui_wready),
        .m_bid(ui_bid), .m_bresp(ui_bresp), .m_buser(1'b0),
        .m_bvalid(ui_bvalid), .m_bready(ui_bready),
        .m_arid(ui_arid), .m_araddr(ui_araddr), .m_arlen(ui_arlen),
        .m_arsize(ui_arsize), .m_arburst(ui_arburst), .m_arlock(),
        .m_arcache(), .m_arprot(), .m_arqos(),
        .m_aruser(), .m_arvalid(ui_arvalid), .m_arready(ui_arready),
        .m_rid(ui_rid), .m_rdata(ui_rdata), .m_rresp(ui_rresp),
        .m_rlast(ui_rlast), .m_ruser(1'b0), .m_rvalid(ui_rvalid),
        .m_rready(ui_rready)
    );

    wire [0:0]   mig_awid;  wire [30:0] mig_awaddr; wire [7:0] mig_awlen;
    wire [2:0]   mig_awsize; wire [1:0] mig_awburst; wire mig_awvalid, mig_awready;
    wire [255:0] mig_wdata; wire [31:0] mig_wstrb;   wire mig_wlast, mig_wvalid, mig_wready;
    wire [0:0]   mig_bid;   wire [1:0]  mig_bresp;   wire mig_bvalid, mig_bready;
    wire [0:0]   mig_arid;  wire [30:0] mig_araddr;  wire [7:0] mig_arlen;
    wire [2:0]   mig_arsize; wire [1:0] mig_arburst; wire mig_arvalid, mig_arready;
    wire [0:0]   mig_rid;   wire [255:0] mig_rdata;  wire [1:0] mig_rresp;
    wire mig_rlast, mig_rvalid, mig_rready;

    axi_ddr4_mig_bridge u_mig_bridge (
        .clk(mig_clk), .rst(mig_rst || !cal_done),
        .s_awid(ui_awid), .s_awaddr(ui_awaddr), .s_awlen(ui_awlen),
        .s_awsize(ui_awsize), .s_awburst(ui_awburst),
        .s_awvalid(ui_awvalid), .s_awready(ui_awready),
        .s_wdata(ui_wdata), .s_wstrb(ui_wstrb), .s_wlast(ui_wlast),
        .s_wvalid(ui_wvalid), .s_wready(ui_wready),
        .s_bid(ui_bid), .s_bresp(ui_bresp), .s_bvalid(ui_bvalid),
        .s_bready(ui_bready),
        .s_arid(ui_arid), .s_araddr(ui_araddr), .s_arlen(ui_arlen),
        .s_arsize(ui_arsize), .s_arburst(ui_arburst),
        .s_arvalid(ui_arvalid), .s_arready(ui_arready),
        .s_rid(ui_rid), .s_rdata(ui_rdata), .s_rresp(ui_rresp),
        .s_rlast(ui_rlast), .s_rvalid(ui_rvalid), .s_rready(ui_rready),

        .m_awid(mig_awid), .m_awaddr(mig_awaddr), .m_awlen(mig_awlen),
        .m_awsize(mig_awsize), .m_awburst(mig_awburst),
        .m_awvalid(mig_awvalid), .m_awready(mig_awready),
        .m_wdata(mig_wdata), .m_wstrb(mig_wstrb), .m_wlast(mig_wlast),
        .m_wvalid(mig_wvalid), .m_wready(mig_wready),
        .m_bid(mig_bid), .m_bresp(mig_bresp), .m_bvalid(mig_bvalid),
        .m_bready(mig_bready),
        .m_arid(mig_arid), .m_araddr(mig_araddr), .m_arlen(mig_arlen),
        .m_arsize(mig_arsize), .m_arburst(mig_arburst),
        .m_arvalid(mig_arvalid), .m_arready(mig_arready),
        .m_rid(mig_rid), .m_rdata(mig_rdata), .m_rresp(mig_rresp),
        .m_rlast(mig_rlast), .m_rvalid(mig_rvalid), .m_rready(mig_rready)
    );

    sim_mig_backend #(.BEATS_LOG2(20), .STALL_ENABLE(1), .STALL_SEED(32'hFACE_B00C),
                      .READ_LATENCY_BASE(DDR_READ_LATENCY_BASE),
                      .READ_LATENCY_JITTER(DDR_READ_LATENCY_JITTER)) u_mig_sim (
        .clk(mig_clk), .rst(mig_rst),
        .cal_done(cal_done),
        .awid(mig_awid), .awaddr(mig_awaddr), .awlen(mig_awlen),
        .awsize(mig_awsize), .awburst(mig_awburst),
        .awvalid(mig_awvalid), .awready(mig_awready),
        .wdata(mig_wdata), .wstrb(mig_wstrb), .wlast(mig_wlast),
        .wvalid(mig_wvalid), .wready(mig_wready),
        .bid(mig_bid), .bresp(mig_bresp), .bvalid(mig_bvalid), .bready(mig_bready),
        .arid(mig_arid), .araddr(mig_araddr), .arlen(mig_arlen),
        .arsize(mig_arsize), .arburst(mig_arburst),
        .arvalid(mig_arvalid), .arready(mig_arready),
        .rid(mig_rid), .rdata(mig_rdata), .rresp(mig_rresp),
        .rlast(mig_rlast), .rvalid(mig_rvalid), .rready(mig_rready)
    );

    // ── Fetch-walk observability ──────────────────────────────────────
    assign dbg_state       = {1'b0, u_scanout.fetch_state};
    assign dbg_credits     = {{(16-(LINE_COUNT_LOG2+1)){1'b0}}, u_scanout.u_fetch.credits};
    assign dbg_outstanding = {4'd0, u_scanout.u_fetch.outstanding};
    assign dbg_stale_count = {4'd0, u_scanout.u_fetch.stale_count};
    assign dbg_req_y       = {{(16-SRC_Y_W){1'b0}}, u_scanout.u_fetch.req_y};
    assign dbg_req_x       = {{(16-SRC_X_W){1'b0}}, u_scanout.u_fetch.req_x};
    assign dbg_rsp_y       = {{(16-SRC_Y_W){1'b0}}, u_scanout.u_fetch.rsp_y};
    assign dbg_rsp_x       = {{(16-SRC_X_W){1'b0}}, u_scanout.u_fetch.rsp_x};
    assign dbg_y_src       = {{(16-SRC_Y_W){1'b0}}, u_scanout.u_disp.y_src};

    reg [31:0] req_accepted_r, rsp_consumed_r;
    always @(posedge pclk) begin
        if (scan_rst) begin
            req_accepted_r <= 32'd0;
            rsp_consumed_r <= 32'd0;
        end else begin
            if (sc_rd_en && sc_rd_ready) req_accepted_r <= req_accepted_r + 32'd1;
            if (sc_rd_valid)             rsp_consumed_r <= rsp_consumed_r + 32'd1;
        end
    end
    assign dbg_req_accepted = req_accepted_r;
    assign dbg_rsp_consumed = rsp_consumed_r;

    wire [QDEPTH_LOG2:0] q_count_now = u_scan_reader.q_count;
    always @(posedge core_clk) begin
        if (scan_rst) dbg_q_peak <= 16'd0;
        else if ({{(16-(QDEPTH_LOG2+1)){1'b0}}, q_count_now} > dbg_q_peak)
            dbg_q_peak <= {{(16-(QDEPTH_LOG2+1)){1'b0}}, q_count_now};
    end

    // ── Arbitration / slack counters ──────────────────────────────────
    // `scan_outst` is rebuilt from the AR/R handshakes rather than read out
    // of the fetch engine, so this harness measures the OLD single-
    // outstanding engine and the NEW multi-outstanding one identically.
    reg [7:0] scan_outst;
    wire scan_ar_accept_c = scan_arvalid && scan_arready;
    wire scan_r_done_c    = scan_rvalid  && scan_rready && scan_rlast;
    // The drain is starved exactly when the reader holds requests it cannot
    // answer -- queue non-empty, no pop.  In CYCLES, which is the unit the
    // scanout budget is denominated in.
    wire scan_starved_c   = (q_count_now != {(QDEPTH_LOG2+1){1'b0}}) &&
                            !u_scan_reader.pop_now;

    always @(posedge core_clk) begin
        if (rst || scan_rst) begin
            scan_outst          <= 8'd0;
            dbg_bulk_bursts     <= 32'd0;
            dbg_bulk_cycles     <= 32'd0;
            dbg_scan_bursts     <= 32'd0;
            dbg_scan_excursions <= 32'd0;
            dbg_scan_stall      <= 32'd0;
            dbg_scan_outst_max  <= 16'd0;
        end else begin
            if (bulk_rvalid && bulk_rlast) dbg_bulk_bursts <= dbg_bulk_bursts + 32'd1;
            if (bulk_pressure)             dbg_bulk_cycles <= dbg_bulk_cycles + 32'd1;
            if (scan_ar_accept_c) begin
                dbg_scan_bursts <= dbg_scan_bursts + 32'd1;
                if (scan_outst == 8'd0)
                    dbg_scan_excursions <= dbg_scan_excursions + 32'd1;
            end
            case ({scan_ar_accept_c, scan_r_done_c})
                2'b10:   scan_outst <= scan_outst + 8'd1;
                2'b01:   scan_outst <= scan_outst - 8'd1;
                default: scan_outst <= scan_outst;
            endcase
            if ({8'd0, scan_outst} > dbg_scan_outst_max)
                dbg_scan_outst_max <= {8'd0, scan_outst};
            if (scan_starved_c) dbg_scan_stall <= dbg_scan_stall + 32'd1;
        end
    end

endmodule

`default_nettype wire
