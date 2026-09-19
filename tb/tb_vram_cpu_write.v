// tb_vram_cpu_write.v — Verilator wrapper for the CPU→VRAM→scanner E2E tb.
//
// Purpose
// ───────
// Prove the CPU-style AXI4 write path into `vram` (URAM-backed
// framebuffer) results in bytes that the streaming scanner port reads
// back byte-exact.  This is the last-mile validation for mac-logo:
// if the ROM eventually writes the logo via the CPU LSU and our AXI-W
// plumbing drops writes, the logo never appears.  tb-vram today covers
// the slave-side protocol but uses synthetic single-beat bursts; this
// tb hammers the same slave with CPU-representative burst patterns
// (multi-beat INCR, varied offsets, varied WSTRB) across the two
// legal BPP modes, and verifies every written pixel via the scanner
// port — exactly the downstream consumer in `video_top`.
//
// Topology (fully sim-only; mirrors the live fpga_top wiring for the
// VRAM AXI slave + scanner pair — does not include the xbar, because
// today the xbar does NOT route FB_BASE to a VRAM slave: FB_BASE is
// decoded to the DDR slave.  The VRAM AXI slave is driven ONLY by the
// smoke writer at the top-level.  This tb therefore exercises the
// VRAM slave directly with a CPU-style traffic pattern, so when the
// planned axi-xbar-vram retune wires a third xbar slave into the
// VRAM port, the slave is already known-good against the exact
// sequence a CPU LSU would issue.)
//
//    [C++ driver]  ──AXI4──►  vram (BPP=8)  ──rd_* ──►  [C++ scanner]
//    [C++ driver]  ──AXI4──►  vram (BPP=16) ──rd_* ──►  [C++ scanner]
//
// Both VRAM instances live in the same `clk` domain; their streaming
// read clocks are also tied to `clk` (matches production — see
// rtl/fpga_top.v: common-clock URAM constraint after task #132).
//
// Verilog-2005.  No SystemVerilog.

`default_nettype none

module tb_vram_cpu_write (
    input  wire                 clk,
    input  wire                 rst,

    // ── Instance 0: BPP=8 ─────────────────────────────────────────────
    // AXI4 slave
    input  wire [3:0]           i0_s_awid,
    input  wire [31:0]          i0_s_awaddr,
    input  wire [7:0]           i0_s_awlen,
    input  wire [2:0]           i0_s_awsize,
    input  wire [1:0]           i0_s_awburst,
    input  wire                 i0_s_awvalid,
    output wire                 i0_s_awready,
    input  wire [127:0]         i0_s_wdata,
    input  wire [15:0]          i0_s_wstrb,
    input  wire                 i0_s_wlast,
    input  wire                 i0_s_wvalid,
    output wire                 i0_s_wready,
    output wire [3:0]           i0_s_bid,
    output wire [1:0]           i0_s_bresp,
    output wire                 i0_s_bvalid,
    input  wire                 i0_s_bready,
    input  wire [3:0]           i0_s_arid,
    input  wire [31:0]          i0_s_araddr,
    input  wire [7:0]           i0_s_arlen,
    input  wire [2:0]           i0_s_arsize,
    input  wire [1:0]           i0_s_arburst,
    input  wire                 i0_s_arvalid,
    output wire                 i0_s_arready,
    output wire [3:0]           i0_s_rid,
    output wire [127:0]         i0_s_rdata,
    output wire [1:0]           i0_s_rresp,
    output wire                 i0_s_rlast,
    output wire                 i0_s_rvalid,
    input  wire                 i0_s_rready,
    // Streaming read
    // Widened to 21 bits: u_vram8 (BPP=8) below does not override
    // VRAM_BYTES, so its PX_ADDR_W is derived from the full 2 MiB
    // VRAM_BYTES aperture (2,097,152 pixels at BPP=8), not from this
    // tb's 128x48=6,144-pixel FB span (which alone would only need 13
    // bits) — see vram.v's PX_ADDR_SPAN comment.  The C++ driver only
    // ever sets the low ~13 meaningful bits; the rest stay zero.
    input  wire [20:0]          i0_rd_addr,
    input  wire                 i0_rd_en,
    // Streaming-read DATA bus is RD_DATA_W=32 wide at every BPP: it returns a
    // GROUP of RD_DATA_W/BPP consecutive pixels starting at rd_addr, with the
    // pixel AT rd_addr in the TOP BPP bits (bit-identical to the old BPP-wide
    // rd_data) and the following pixels below it.
    output wire [31:0]          i0_rd_data,
    output wire                 i0_rd_valid,

    // ── Instance 1: BPP=16 ────────────────────────────────────────────
    input  wire [3:0]           i1_s_awid,
    input  wire [31:0]          i1_s_awaddr,
    input  wire [7:0]           i1_s_awlen,
    input  wire [2:0]           i1_s_awsize,
    input  wire [1:0]           i1_s_awburst,
    input  wire                 i1_s_awvalid,
    output wire                 i1_s_awready,
    input  wire [127:0]         i1_s_wdata,
    input  wire [15:0]          i1_s_wstrb,
    input  wire                 i1_s_wlast,
    input  wire                 i1_s_wvalid,
    output wire                 i1_s_wready,
    output wire [3:0]           i1_s_bid,
    output wire [1:0]           i1_s_bresp,
    output wire                 i1_s_bvalid,
    input  wire                 i1_s_bready,
    input  wire [3:0]           i1_s_arid,
    input  wire [31:0]          i1_s_araddr,
    input  wire [7:0]           i1_s_arlen,
    input  wire [2:0]           i1_s_arsize,
    input  wire [1:0]           i1_s_arburst,
    input  wire                 i1_s_arvalid,
    output wire                 i1_s_arready,
    output wire [3:0]           i1_s_rid,
    output wire [127:0]         i1_s_rdata,
    output wire [1:0]           i1_s_rresp,
    output wire                 i1_s_rlast,
    output wire                 i1_s_rvalid,
    input  wire                 i1_s_rready,
    // Streaming read
    // u_vram16 (BPP=16) below does not override VRAM_BYTES either, so
    // its PX_ADDR_W is 2 MiB / 2 bytes-per-pixel = 1,048,576 pixels =
    // 20 bits — already matches this port's existing declared width.
    input  wire [19:0]          i1_rd_addr,
    input  wire                 i1_rd_en,
    // Same 4-LANE group contract as i0_rd_data above, but this instance is
    // BPP=16, and vram.v's RD_DATA_W defaults to 4*BPP -- so the group is FOUR
    // 16-bit pixels = 64 bits, with the pixel at rd_addr in [63:48].  (This
    // instance exists to exercise the AXI byte-enable write path at BPP=16;
    // the streaming read port is incidental here, but it still has to be
    // connected at the right width.)
    output wire [63:0]          i1_rd_data,
    output wire                 i1_rd_valid
);

    // Use a 128×48 frame — matches tb_video_smoke, keeps URAM word
    // count small so the sim runs in < 1 s wall-clock.
    localparam FB_W = 128;
    localparam FB_H = 48;

    // ── BPP=8 instance ───────────────────────────────────────────────
    vram #(
        .FB_WIDTH_PX (FB_W),
        .FB_HEIGHT_PX(FB_H),
        .BPP         (8),
        .RD_DATA_W   (32),
        .DATA_WIDTH  (128),
        .ID_WIDTH    (4)
    ) u_vram8 (
        .clk       (clk),
        .rst       (rst),
        .clear_req (1'b0),
        .s_awid    (i0_s_awid),
        .s_awaddr  (i0_s_awaddr),
        .s_awlen   (i0_s_awlen),
        .s_awsize  (i0_s_awsize),
        .s_awburst (i0_s_awburst),
        .s_awvalid (i0_s_awvalid),
        .s_awready (i0_s_awready),
        .s_wdata   (i0_s_wdata),
        .s_wstrb   (i0_s_wstrb),
        .s_wlast   (i0_s_wlast),
        .s_wvalid  (i0_s_wvalid),
        .s_wready  (i0_s_wready),
        .s_bid     (i0_s_bid),
        .s_bresp   (i0_s_bresp),
        .s_bvalid  (i0_s_bvalid),
        .s_bready  (i0_s_bready),
        .s_arid    (i0_s_arid),
        .s_araddr  (i0_s_araddr),
        .s_arlen   (i0_s_arlen),
        .s_arsize  (i0_s_arsize),
        .s_arburst (i0_s_arburst),
        .s_arvalid (i0_s_arvalid),
        .s_arready (i0_s_arready),
        .s_rid     (i0_s_rid),
        .s_rdata   (i0_s_rdata),
        .s_rresp   (i0_s_rresp),
        .s_rlast   (i0_s_rlast),
        .s_rvalid  (i0_s_rvalid),
        .s_rready  (i0_s_rready),
        .rd_clk    (clk),
        .rd_rst    (rst),
        .rd_addr   (i0_rd_addr),  // full 21-bit port; only low ~13 bits ever nonzero
        .rd_en     (i0_rd_en),
        .rd_data   (i0_rd_data),
        .rd_valid  (i0_rd_valid)
    );

    // ── BPP=16 instance ──────────────────────────────────────────────
    vram #(
        .FB_WIDTH_PX (FB_W),
        .FB_HEIGHT_PX(FB_H),
        .BPP         (16),
        // RD_DATA_W deliberately NOT overridden: it defaults to 4*BPP = 64,
        // which is the only self-consistent 4-lane width at BPP=16.  Forcing
        // 32 here would leave two of the four gather lanes undriven.
        .DATA_WIDTH  (128),
        .ID_WIDTH    (4)
    ) u_vram16 (
        .clk       (clk),
        .rst       (rst),
        .clear_req (1'b0),
        .s_awid    (i1_s_awid),
        .s_awaddr  (i1_s_awaddr),
        .s_awlen   (i1_s_awlen),
        .s_awsize  (i1_s_awsize),
        .s_awburst (i1_s_awburst),
        .s_awvalid (i1_s_awvalid),
        .s_awready (i1_s_awready),
        .s_wdata   (i1_s_wdata),
        .s_wstrb   (i1_s_wstrb),
        .s_wlast   (i1_s_wlast),
        .s_wvalid  (i1_s_wvalid),
        .s_wready  (i1_s_wready),
        .s_bid     (i1_s_bid),
        .s_bresp   (i1_s_bresp),
        .s_bvalid  (i1_s_bvalid),
        .s_bready  (i1_s_bready),
        .s_arid    (i1_s_arid),
        .s_araddr  (i1_s_araddr),
        .s_arlen   (i1_s_arlen),
        .s_arsize  (i1_s_arsize),
        .s_arburst (i1_s_arburst),
        .s_arvalid (i1_s_arvalid),
        .s_arready (i1_s_arready),
        .s_rid     (i1_s_rid),
        .s_rdata   (i1_s_rdata),
        .s_rresp   (i1_s_rresp),
        .s_rlast   (i1_s_rlast),
        .s_rvalid  (i1_s_rvalid),
        .s_rready  (i1_s_rready),
        .rd_clk    (clk),
        .rd_rst    (rst),
        .rd_addr   (i1_rd_addr),  // full 20-bit port; only low ~13 bits ever nonzero
        .rd_en     (i1_rd_en),
        .rd_data   (i1_rd_data),
        .rd_valid  (i1_rd_valid)
    );

endmodule

`default_nettype wire
