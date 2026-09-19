// tb_video_smoke.v — Verilator wrapper for the VRAM smoke-preload tb.
//
// Instantiates:
//   - vram_smoke (reset-time AXI writer with SMPTE-bar pattern)
//   - vram       (URAM-backed framebuffer with AXI slave + stream port)
// … wires the smoke-writer's AXI master directly into vram's slave port.
// The streaming read port is exposed to the C++ tb so it can walk
// through every pixel of the frame once the smoke writer signals done
// and compare against the golden bar pattern.
//
// Uses a small source frame (128 × 48, BPP=8) so a single full-frame
// write completes in < 2k cycles — keeps tb wall-clock under a second.
// BPP=8 also exercises the scaler's indexed-colour path: smoke pixels
// carry low-nibble DAFB CLUT indexes for the intended SMPTE bars.

`default_nettype none

module tb_video_smoke (
    input  wire        clk,
    input  wire        rst,

    // Streaming read port backdoor (consumed by the C++ harness)
    input  wire [12:0] rd_addr,    // 128*48 = 6144 < 2^13
    input  wire        rd_en,
    // 4-byte group starting at rd_addr (vram.v RD_DATA_W=32): [31:24] is the
    // byte at rd_addr (valid at any alignment), [23:0] the bytes at +1/+2/+3
    // (valid only when rd_addr[1:0]==0).
    output wire [31:0] rd_data,
    output wire        rd_valid,

    // Done indicator (smoke writer latched-high after last word lands)
    output wire        smoke_done
);

    localparam FB_W    = 128;
    localparam FB_H    = 48;
    localparam FB_BPP  = 8;
    localparam DATA_W  = 128;
    localparam ID_W    = 4;

    // AXI wires between smoke writer and vram slave
    wire [ID_W-1:0]   s_awid;
    wire [31:0]       s_awaddr;
    wire [7:0]        s_awlen;
    wire [2:0]        s_awsize;
    wire [1:0]        s_awburst;
    wire              s_awvalid;
    wire              s_awready;
    wire [DATA_W-1:0] s_wdata;
    wire [DATA_W/8-1:0] s_wstrb;
    wire              s_wlast;
    wire              s_wvalid;
    wire              s_wready;
    wire [ID_W-1:0]   s_bid;
    wire [1:0]        s_bresp;
    wire              s_bvalid;
    wire              s_bready;

    vram_smoke #(
        .FB_WIDTH_PX (FB_W),
        .FB_HEIGHT_PX(FB_H),
        .BPP         (FB_BPP),
        .DATA_WIDTH  (DATA_W),
        .ID_WIDTH    (ID_W)
    ) u_smoke (
        .clk       (clk),
        .rst       (rst),
        .done      (smoke_done),
        .m_awid    (s_awid),
        .m_awaddr  (s_awaddr),
        .m_awlen   (s_awlen),
        .m_awsize  (s_awsize),
        .m_awburst (s_awburst),
        .m_awvalid (s_awvalid),
        .m_awready (s_awready),
        .m_wdata   (s_wdata),
        .m_wstrb   (s_wstrb),
        .m_wlast   (s_wlast),
        .m_wvalid  (s_wvalid),
        .m_wready  (s_wready),
        .m_bid     (s_bid),
        .m_bresp   (s_bresp),
        .m_bvalid  (s_bvalid),
        .m_bready  (s_bready)
    );

    vram #(
        .FB_WIDTH_PX (FB_W),
        .FB_HEIGHT_PX(FB_H),
        // BPP is the pixel packing / read-port address granularity;
        // RD_DATA_W is the (widened) read-port DATA width, and vram.v requires
        // it to be exactly 4*BPP (32 at this tb's fixed FB_BPP=8, matching the
        // 32-bit rd_data port declared above).
        .BPP         (FB_BPP),
        .RD_DATA_W   (4 * FB_BPP),
        .DATA_WIDTH  (DATA_W),
        .ID_WIDTH    (ID_W)
    ) u_vram (
        .clk       (clk),
        .rst       (rst),
        .clear_req (1'b0),
        .s_awid    (s_awid),
        .s_awaddr  (s_awaddr),
        .s_awlen   (s_awlen),
        .s_awsize  (s_awsize),
        .s_awburst (s_awburst),
        .s_awvalid (s_awvalid),
        .s_awready (s_awready),
        .s_wdata   (s_wdata),
        .s_wstrb   (s_wstrb),
        .s_wlast   (s_wlast),
        .s_wvalid  (s_wvalid),
        .s_wready  (s_wready),
        .s_bid     (s_bid),
        .s_bresp   (s_bresp),
        .s_bvalid  (s_bvalid),
        .s_bready  (s_bready),
        // AR/R unused
        .s_arid    (4'd0), .s_araddr(32'd0),
        .s_arlen   (8'd0), .s_arsize(3'd0),
        .s_arburst (2'd0), .s_arvalid(1'b0),
        .s_arready (),
        .s_rid     (), .s_rdata(),
        .s_rresp   (), .s_rlast(),
        .s_rvalid  (), .s_rready(1'b1),
        // Streaming read port — same clock in the tb for simplicity
        .rd_clk    (clk),
        .rd_rst    (rst),
        .rd_addr   (rd_addr),
        .rd_en     (rd_en),
        .rd_data   (rd_data),
        .rd_valid  (rd_valid)
    );

endmodule

`default_nettype wire
