// axi_vram_smoke_mux.v -- VRAM-lane write-source mux + carveout translate.
//
// PURPOSE
//   Under `VRAM_IN_DDR` the VRAM aperture is a DDR carveout, not a URAM
//   array, so the `vram_smoke` reset-time SMPTE-bar writer can no longer
//   drive `vram.v`'s AXI slave port (that port does not exist in that
//   build).  This module is the replacement seam: it multiplexes the smoke
//   writer onto the SAME lane the CPU's VRAM-aperture writes take -- xbar
//   S3 -> axi_vram_priority_mux3's `s3_*` port -- so smoke traffic is
//   translated, arbitrated and delivered by exactly the path CPU writes
//   already use.  Nothing about `vram_smoke` is URAM-specific; it is a
//   plain AXI4 write master.
//
//   It also owns the S3 lane's address translate (the `+ CARVEOUT_BASE`
//   step that used to be two inline `wire` assignments in
//   fpga_top_ddr.vh), so that the select and the translate are one
//   testable unit and the smoke master is guaranteed to sit UPSTREAM of
//   the translate.  Both sources therefore present aperture-relative
//   (zero-based) addresses, exactly like the URAM slave always saw.
//
// BYTE ORDER -- THE LOAD-BEARING DETAIL
//   The CPU<->VRAM byte-lane swap (`vram_swap_words` / `vram_swap_strb`)
//   is applied INSIDE axi_xbar.v, on its S3 master ports
//   (axi_xbar.v:1660-1661).  The `s3_*` signals arriving HERE are already
//   post-swap, i.e. plain AXI byte-lane order: byte at aperture offset
//   (A + k) travels on s3_wdata[k*8 +: 8].  scanout_line_fetch.v fills its
//   ring with exactly that convention (`m_rdata[i*8 +: 8]` is the byte at
//   the burst address + i), and vram_smoke.v authors its words with lane 0
//   = lowest-address pixel.  Smoke therefore needs NO swap at this
//   boundary -- it is already on the same side as post-swap S3 traffic.
//   Adding one here would mirror every pixel within each 32-bit group.
//   tb_vram_ddr_chain's `smoke_matches_cpu_path_byte_order` scenario is
//   the executable proof: it pushes a known byte string through the real
//   xbar S3 path and compares it, byte for byte at the scanout read port,
//   against what smoke lands.
//
// ARBITRATION
//   Strictly sequential, not a real arbiter: `smoke_active` is
//   (VIDEO_SMOKE != 0) && !smoke_done, and `vram_smoke` only latches
//   `done` after the B response of its final word.  The two masters
//   therefore never have an AW outstanding at the same time and the select
//   can never flip mid-transaction.  This mirrors the URAM build's
//   smoke/S3 mux in fpga_top_video.vh one-for-one.
//
//   READ CHANNEL: smoke never reads, so AR/R pass straight through with
//   only the address translate applied.  This deliberately DIFFERS from
//   the URAM mux, which force-zeroes `s3_rvalid` while smoke is active.
//   Doing that here would drop R beats that axi_vram_priority_mux3 has
//   already committed to a locked grant, corrupting its single-outstanding
//   accounting.  Letting reads flow costs nothing: reading the aperture
//   while it is being painted is a don't-care by construction.
//
// LATENCY
//   Zero -- purely combinational.  No state, no clock, no reset.
//
`default_nettype none

module axi_vram_smoke_mux #(
    parameter ID_WIDTH   = 6,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 128,
    parameter [31:0] CARVEOUT_BASE = 32'h4600_0000
) (
    // 1 = the smoke writer owns the lane's write channels.
    input  wire                      smoke_active,

    // ── vram_smoke's AXI4 write master (aperture-relative addresses) ──
    input  wire [ID_WIDTH-1:0]       smk_awid,
    input  wire [ADDR_WIDTH-1:0]     smk_awaddr,
    input  wire [7:0]                smk_awlen,
    input  wire [2:0]                smk_awsize,
    input  wire [1:0]                smk_awburst,
    input  wire                      smk_awvalid,
    output wire                      smk_awready,
    input  wire [DATA_WIDTH-1:0]     smk_wdata,
    input  wire [DATA_WIDTH/8-1:0]   smk_wstrb,
    input  wire                      smk_wlast,
    input  wire                      smk_wvalid,
    output wire                      smk_wready,
    output wire [1:0]                smk_bresp,
    output wire                      smk_bvalid,
    input  wire                      smk_bready,

    // ── xbar S3 (aperture-relative addresses, already byte-swapped) ──
    input  wire [ID_WIDTH-1:0]       s3_awid,
    input  wire [ADDR_WIDTH-1:0]     s3_awaddr,
    input  wire [7:0]                s3_awlen,
    input  wire [2:0]                s3_awsize,
    input  wire [1:0]                s3_awburst,
    input  wire                      s3_awvalid,
    output wire                      s3_awready,
    input  wire [DATA_WIDTH-1:0]     s3_wdata,
    input  wire [DATA_WIDTH/8-1:0]   s3_wstrb,
    input  wire                      s3_wlast,
    input  wire                      s3_wvalid,
    output wire                      s3_wready,
    output wire [ID_WIDTH-1:0]       s3_bid,
    output wire [1:0]                s3_bresp,
    output wire                      s3_bvalid,
    input  wire                      s3_bready,
    input  wire [ID_WIDTH-1:0]       s3_arid,
    input  wire [ADDR_WIDTH-1:0]     s3_araddr,
    input  wire [7:0]                s3_arlen,
    input  wire [2:0]                s3_arsize,
    input  wire [1:0]                s3_arburst,
    input  wire                      s3_arvalid,
    output wire                      s3_arready,
    output wire [ID_WIDTH-1:0]       s3_rid,
    output wire [DATA_WIDTH-1:0]     s3_rdata,
    output wire [1:0]                s3_rresp,
    output wire                      s3_rlast,
    output wire                      s3_rvalid,
    input  wire                      s3_rready,

    // ── merged lane out (ABSOLUTE DDR addresses) ──────────────────────
    output wire [ID_WIDTH-1:0]       m_awid,
    output wire [ADDR_WIDTH-1:0]     m_awaddr,
    output wire [7:0]                m_awlen,
    output wire [2:0]                m_awsize,
    output wire [1:0]                m_awburst,
    output wire                      m_awvalid,
    input  wire                      m_awready,
    output wire [DATA_WIDTH-1:0]     m_wdata,
    output wire [DATA_WIDTH/8-1:0]   m_wstrb,
    output wire                      m_wlast,
    output wire                      m_wvalid,
    input  wire                      m_wready,
    input  wire [ID_WIDTH-1:0]       m_bid,
    input  wire [1:0]                m_bresp,
    input  wire                      m_bvalid,
    output wire                      m_bready,
    output wire [ID_WIDTH-1:0]       m_arid,
    output wire [ADDR_WIDTH-1:0]     m_araddr,
    output wire [7:0]                m_arlen,
    output wire [2:0]                m_arsize,
    output wire [1:0]                m_arburst,
    output wire                      m_arvalid,
    input  wire                      m_arready,
    input  wire [ID_WIDTH-1:0]       m_rid,
    input  wire [DATA_WIDTH-1:0]     m_rdata,
    input  wire [1:0]                m_rresp,
    input  wire                      m_rlast,
    input  wire                      m_rvalid,
    output wire                      m_rready
);

    // ── AW ────────────────────────────────────────────────────────────
    wire [ADDR_WIDTH-1:0] sel_awaddr = smoke_active ? smk_awaddr : s3_awaddr;
    assign m_awid    = smoke_active ? smk_awid    : s3_awid;
    assign m_awaddr  = sel_awaddr + CARVEOUT_BASE;
    assign m_awlen   = smoke_active ? smk_awlen   : s3_awlen;
    assign m_awsize  = smoke_active ? smk_awsize  : s3_awsize;
    assign m_awburst = smoke_active ? smk_awburst : s3_awburst;
    assign m_awvalid = smoke_active ? smk_awvalid : s3_awvalid;
    assign smk_awready = smoke_active ? m_awready : 1'b0;
    assign s3_awready  = smoke_active ? 1'b0      : m_awready;

    // ── W ─────────────────────────────────────────────────────────────
    // NO byte swap on either arm -- see the header.  s3_wdata is already
    // post-swap; smoke authors in the same (plain AXI byte-lane) order.
    assign m_wdata  = smoke_active ? smk_wdata  : s3_wdata;
    assign m_wstrb  = smoke_active ? smk_wstrb  : s3_wstrb;
    assign m_wlast  = smoke_active ? smk_wlast  : s3_wlast;
    assign m_wvalid = smoke_active ? smk_wvalid : s3_wvalid;
    assign smk_wready = smoke_active ? m_wready : 1'b0;
    assign s3_wready  = smoke_active ? 1'b0     : m_wready;

    // ── B ─────────────────────────────────────────────────────────────
    // Only one master has an AW outstanding at a time, so the response is
    // steered by the same select and the other side is masked.
    assign smk_bresp  = m_bresp;
    assign smk_bvalid = smoke_active ? m_bvalid : 1'b0;
    assign s3_bid     = m_bid;
    assign s3_bresp   = m_bresp;
    assign s3_bvalid  = smoke_active ? 1'b0     : m_bvalid;
    assign m_bready   = smoke_active ? smk_bready : s3_bready;

    // ── AR / R: S3 only, translate applied, never gated ───────────────
    assign m_arid    = s3_arid;
    assign m_araddr  = s3_araddr + CARVEOUT_BASE;
    assign m_arlen   = s3_arlen;
    assign m_arsize  = s3_arsize;
    assign m_arburst = s3_arburst;
    assign m_arvalid = s3_arvalid;
    assign s3_arready = m_arready;

    assign s3_rid    = m_rid;
    assign s3_rdata  = m_rdata;
    assign s3_rresp  = m_rresp;
    assign s3_rlast  = m_rlast;
    assign s3_rvalid = m_rvalid;
    assign m_rready  = s3_rready;

endmodule

`default_nettype wire
