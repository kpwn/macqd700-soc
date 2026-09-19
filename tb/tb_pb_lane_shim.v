// tb_pb_lane_shim.v — Verilator top for the S1 narrowed-payload CDC
// equivalence testbench (`make tb-pb-lane-shim`).
//
// Two structurally parallel chains, fed the identical transaction
// stream by tb/tb_pb_lane_shim.cpp:
//
//   REF:  wide AXI4 face -> axi_async_bridge #(128) -> pb_lane_slave_model
//   DUT:  wide AXI4 face -> axi_pb_s1_cdc  #(128)   -> pb_lane_slave_model
//
// `axi_pb_s1_cdc` is internally
//   axi_pb_lane_narrow (128->32) -> axi_async_bridge #(32) -> axi_pb_lane_widen
// so REF and DUT differ ONLY in the payload width actually carried
// across the core_clk -> pb_clk boundary.  Both chains share `s_clk`/
// `s_rst` (core domain, 100 MHz) and `m_clk`/`m_rst` (pb domain,
// 50 MHz); the C++ side generates that 2:1 ratio with non-coincident
// edges so the async_fifo synchronisers are genuinely exercised.
//
// Each chain gets its OWN copy of the AXI payload wires rather than
// sharing one set.  That deliberately decouples the two chains — REF
// may be several transactions ahead of DUT — so the equivalence result
// cannot be an artefact of lockstep driving.  Stimulus identity is
// guaranteed on the C++ side instead: both chains replay the same
// `txs[]` list, in the same order, element for element.
//
// ── pb_lane_slave_model ───────────────────────────────────────────────
// The downstream model replicates `rtl/soc/peripheral_bus.v`'s byte-lane
// policy EXACTLY, because that policy is the contract the two shims rely
// on:
//
//   * the active 32-bit lane is derived from AWADDR[3:2] / ARADDR[3:2],
//     never from WSTRB (peripheral_bus.v:662, :1232 — `s_aw_lane` /
//     `s_ar_lane`, captured into `wr_lane_q` / `rd_lane_q`);
//   * a write captures `wdata[lane*32 +: 32]` and `wstrb[lane*4 +: 4]`
//     through 4:1 muxes (peripheral_bus.v:665-674, `wr_lane_data` /
//     `wr_lane_strb`) and applies them byte-wise;
//   * a read returns the 32-bit word placed in lane ARADDR[3:2] with
//     the other three lanes HARD ZERO (peripheral_bus.v:1573-1576).
//     That zeroing is load-bearing — `axi_pb_lane_widen` recovers the
//     word with a 4-way OR-reduce;
//   * one outstanding read and one outstanding write (peripheral_bus.v
//     header, "One outstanding read and one outstanding write at a
//     time"), each with a small pseudo-random extra response latency so
//     both chains see real back-pressure.
//
// AWSIZE/ARSIZE note: `axi_pb_lane_narrow` clamps SIZE to 3'd2 (4 bytes)
// because the narrow bus is 32 bits.  peripheral_bus only ever tests
// `size == 3'd1` (the SONIC 16-bit path, peripheral_bus.v:723, :1387,
// :1439, :1481, :1566), and sizes 0/1/2 pass through the clamp
// unchanged, so the clamp is invisible downstream.  The models capture
// the SLAVE-SIDE size they observed (`*_dbg_last_awsize` /
// `*_dbg_last_arsize`) so the C++ side can assert exactly that:
// pass-through for size <= 2, clamp to 2 above it.
//
// Verilog-2005.

`default_nettype none

module tb_pb_lane_shim (
    input  wire         s_clk,      // core domain (fast)
    input  wire         s_rst,
    input  wire         m_clk,      // pb domain (half rate)
    input  wire         m_rst,

    // ── REF chain: wide AXI4 slave face ──────────────────────────────
    input  wire [5:0]   ref_awid,
    input  wire [31:0]  ref_awaddr,
    input  wire [7:0]   ref_awlen,
    input  wire [2:0]   ref_awsize,
    input  wire [1:0]   ref_awburst,
    input  wire         ref_awvalid,
    output wire         ref_awready,
    input  wire [127:0] ref_wdata,
    input  wire [15:0]  ref_wstrb,
    input  wire         ref_wlast,
    input  wire         ref_wvalid,
    output wire         ref_wready,
    output wire [5:0]   ref_bid,
    output wire [1:0]   ref_bresp,
    output wire         ref_bvalid,
    input  wire         ref_bready,
    input  wire [5:0]   ref_arid,
    input  wire [31:0]  ref_araddr,
    input  wire [7:0]   ref_arlen,
    input  wire [2:0]   ref_arsize,
    input  wire [1:0]   ref_arburst,
    input  wire         ref_arvalid,
    output wire         ref_arready,
    output wire [5:0]   ref_rid,
    output wire [127:0] ref_rdata,
    output wire [1:0]   ref_rresp,
    output wire         ref_rlast,
    output wire         ref_rvalid,
    input  wire         ref_rready,

    // ── DUT chain: wide AXI4 slave face ──────────────────────────────
    input  wire [5:0]   dut_awid,
    input  wire [31:0]  dut_awaddr,
    input  wire [7:0]   dut_awlen,
    input  wire [2:0]   dut_awsize,
    input  wire [1:0]   dut_awburst,
    input  wire         dut_awvalid,
    output wire         dut_awready,
    input  wire [127:0] dut_wdata,
    input  wire [15:0]  dut_wstrb,
    input  wire         dut_wlast,
    input  wire         dut_wvalid,
    output wire         dut_wready,
    output wire [5:0]   dut_bid,
    output wire [1:0]   dut_bresp,
    output wire         dut_bvalid,
    input  wire         dut_bready,
    input  wire [5:0]   dut_arid,
    input  wire [31:0]  dut_araddr,
    input  wire [7:0]   dut_arlen,
    input  wire [2:0]   dut_arsize,
    input  wire [1:0]   dut_arburst,
    input  wire         dut_arvalid,
    output wire         dut_arready,
    output wire [5:0]   dut_rid,
    output wire [127:0] dut_rdata,
    output wire [1:0]   dut_rresp,
    output wire         dut_rlast,
    output wire         dut_rvalid,
    input  wire         dut_rready,

    // ── backdoor into the two slave models' captured write memories ──
    input  wire [9:0]   dbg_idx,
    output wire [31:0]  ref_dbg_data,
    output wire [31:0]  dut_dbg_data,
    output wire [2:0]   ref_dbg_last_awsize,
    output wire [2:0]   ref_dbg_last_arsize,
    output wire [2:0]   dut_dbg_last_awsize,
    output wire [2:0]   dut_dbg_last_arsize
);

    // ═════════════════════════════════════════════════════════════════
    // REF chain
    // ═════════════════════════════════════════════════════════════════
    wire [5:0]   rp_awid;   wire [31:0]  rp_awaddr; wire [7:0] rp_awlen;
    wire [2:0]   rp_awsize; wire [1:0]   rp_awburst;
    wire         rp_awvalid, rp_awready;
    wire [127:0] rp_wdata;  wire [15:0]  rp_wstrb;
    wire         rp_wlast, rp_wvalid, rp_wready;
    wire [5:0]   rp_bid;    wire [1:0]   rp_bresp;
    wire         rp_bvalid, rp_bready;
    wire [5:0]   rp_arid;   wire [31:0]  rp_araddr; wire [7:0] rp_arlen;
    wire [2:0]   rp_arsize; wire [1:0]   rp_arburst;
    wire         rp_arvalid, rp_arready;
    wire [5:0]   rp_rid;    wire [127:0] rp_rdata;  wire [1:0] rp_rresp;
    wire         rp_rlast, rp_rvalid, rp_rready;

    // Unused USER/lock/cache/prot/qos taps on the bridge's m side.
    wire         rp_awlock_nc;  wire [3:0] rp_awcache_nc;
    wire [2:0]   rp_awprot_nc;  wire [3:0] rp_awqos_nc;
    wire         rp_awuser_nc,  rp_wuser_nc, rp_ruser_nc;
    wire         rp_arlock_nc;  wire [3:0] rp_arcache_nc;
    wire [2:0]   rp_arprot_nc;  wire [3:0] rp_arqos_nc;
    wire         rp_aruser_nc;
    wire         ref_buser_nc,  ref_ruser_nc;

    axi_async_bridge #(
        .DATA_WIDTH(128), .ADDR_WIDTH(32), .ID_WIDTH(6), .USER_WIDTH(1)
    ) u_ref_cdc (
        .s_clk(s_clk), .s_rst(s_rst),
        .s_awid(ref_awid), .s_awaddr(ref_awaddr), .s_awlen(ref_awlen),
        .s_awsize(ref_awsize), .s_awburst(ref_awburst),
        .s_awlock(1'b0), .s_awcache(4'd0), .s_awprot(3'd0), .s_awqos(4'd0),
        .s_awuser(1'b0),
        .s_awvalid(ref_awvalid), .s_awready(ref_awready),
        .s_wdata(ref_wdata), .s_wstrb(ref_wstrb), .s_wlast(ref_wlast),
        .s_wuser(1'b0), .s_wvalid(ref_wvalid), .s_wready(ref_wready),
        .s_bid(ref_bid), .s_bresp(ref_bresp), .s_buser(ref_buser_nc),
        .s_bvalid(ref_bvalid), .s_bready(ref_bready),
        .s_arid(ref_arid), .s_araddr(ref_araddr), .s_arlen(ref_arlen),
        .s_arsize(ref_arsize), .s_arburst(ref_arburst),
        .s_arlock(1'b0), .s_arcache(4'd0), .s_arprot(3'd0), .s_arqos(4'd0),
        .s_aruser(1'b0),
        .s_arvalid(ref_arvalid), .s_arready(ref_arready),
        .s_rid(ref_rid), .s_rdata(ref_rdata), .s_rresp(ref_rresp),
        .s_rlast(ref_rlast), .s_ruser(ref_ruser_nc),
        .s_rvalid(ref_rvalid), .s_rready(ref_rready),

        .m_clk(m_clk), .m_rst(m_rst),
        .m_awid(rp_awid), .m_awaddr(rp_awaddr), .m_awlen(rp_awlen),
        .m_awsize(rp_awsize), .m_awburst(rp_awburst),
        .m_awlock(rp_awlock_nc), .m_awcache(rp_awcache_nc),
        .m_awprot(rp_awprot_nc), .m_awqos(rp_awqos_nc),
        .m_awuser(rp_awuser_nc),
        .m_awvalid(rp_awvalid), .m_awready(rp_awready),
        .m_wdata(rp_wdata), .m_wstrb(rp_wstrb), .m_wlast(rp_wlast),
        .m_wuser(rp_wuser_nc), .m_wvalid(rp_wvalid), .m_wready(rp_wready),
        .m_bid(rp_bid), .m_bresp(rp_bresp), .m_buser(1'b0),
        .m_bvalid(rp_bvalid), .m_bready(rp_bready),
        .m_arid(rp_arid), .m_araddr(rp_araddr), .m_arlen(rp_arlen),
        .m_arsize(rp_arsize), .m_arburst(rp_arburst),
        .m_arlock(rp_arlock_nc), .m_arcache(rp_arcache_nc),
        .m_arprot(rp_arprot_nc), .m_arqos(rp_arqos_nc),
        .m_aruser(rp_aruser_nc),
        .m_arvalid(rp_arvalid), .m_arready(rp_arready),
        .m_rid(rp_rid), .m_rdata(rp_rdata), .m_rresp(rp_rresp),
        .m_rlast(rp_rlast), .m_ruser(1'b0),
        .m_rvalid(rp_rvalid), .m_rready(rp_rready)
    );

    pb_lane_slave_model #(.SEED(32'h1234_5678)) u_ref_slave (
        .clk(m_clk), .rst(m_rst),
        .s_awid(rp_awid), .s_awaddr(rp_awaddr), .s_awlen(rp_awlen),
        .s_awsize(rp_awsize), .s_awburst(rp_awburst),
        .s_awvalid(rp_awvalid), .s_awready(rp_awready),
        .s_wdata(rp_wdata), .s_wstrb(rp_wstrb), .s_wlast(rp_wlast),
        .s_wvalid(rp_wvalid), .s_wready(rp_wready),
        .s_bid(rp_bid), .s_bresp(rp_bresp),
        .s_bvalid(rp_bvalid), .s_bready(rp_bready),
        .s_arid(rp_arid), .s_araddr(rp_araddr), .s_arlen(rp_arlen),
        .s_arsize(rp_arsize), .s_arburst(rp_arburst),
        .s_arvalid(rp_arvalid), .s_arready(rp_arready),
        .s_rid(rp_rid), .s_rdata(rp_rdata), .s_rresp(rp_rresp),
        .s_rlast(rp_rlast), .s_rvalid(rp_rvalid), .s_rready(rp_rready),
        .dbg_idx(dbg_idx), .dbg_data(ref_dbg_data),
        .dbg_last_awsize(ref_dbg_last_awsize),
        .dbg_last_arsize(ref_dbg_last_arsize)
    );

    // ═════════════════════════════════════════════════════════════════
    // DUT chain
    // ═════════════════════════════════════════════════════════════════
    wire [5:0]   dp_awid;   wire [31:0]  dp_awaddr; wire [7:0] dp_awlen;
    wire [2:0]   dp_awsize; wire [1:0]   dp_awburst;
    wire         dp_awvalid, dp_awready;
    wire [127:0] dp_wdata;  wire [15:0]  dp_wstrb;
    wire         dp_wlast, dp_wvalid, dp_wready;
    wire [5:0]   dp_bid;    wire [1:0]   dp_bresp;
    wire         dp_bvalid, dp_bready;
    wire [5:0]   dp_arid;   wire [31:0]  dp_araddr; wire [7:0] dp_arlen;
    wire [2:0]   dp_arsize; wire [1:0]   dp_arburst;
    wire         dp_arvalid, dp_arready;
    wire [5:0]   dp_rid;    wire [127:0] dp_rdata;  wire [1:0] dp_rresp;
    wire         dp_rlast, dp_rvalid, dp_rready;

    wire         dp_awlock_nc;  wire [3:0] dp_awcache_nc;
    wire [2:0]   dp_awprot_nc;  wire [3:0] dp_awqos_nc;
    wire         dp_awuser_nc,  dp_wuser_nc, dp_ruser_nc;
    wire         dp_arlock_nc;  wire [3:0] dp_arcache_nc;
    wire [2:0]   dp_arprot_nc;  wire [3:0] dp_arqos_nc;
    wire         dp_aruser_nc;
    wire         dut_buser_nc,  dut_ruser_nc;

    axi_pb_s1_cdc #(
        .DATA_WIDTH(128), .ADDR_WIDTH(32), .ID_WIDTH(6), .USER_WIDTH(1)
    ) u_dut_cdc (
        .s_clk(s_clk), .s_rst(s_rst),
        .s_awid(dut_awid), .s_awaddr(dut_awaddr), .s_awlen(dut_awlen),
        .s_awsize(dut_awsize), .s_awburst(dut_awburst),
        .s_awlock(1'b0), .s_awcache(4'd0), .s_awprot(3'd0), .s_awqos(4'd0),
        .s_awuser(1'b0),
        .s_awvalid(dut_awvalid), .s_awready(dut_awready),
        .s_wdata(dut_wdata), .s_wstrb(dut_wstrb), .s_wlast(dut_wlast),
        .s_wuser(1'b0), .s_wvalid(dut_wvalid), .s_wready(dut_wready),
        .s_bid(dut_bid), .s_bresp(dut_bresp), .s_buser(dut_buser_nc),
        .s_bvalid(dut_bvalid), .s_bready(dut_bready),
        .s_arid(dut_arid), .s_araddr(dut_araddr), .s_arlen(dut_arlen),
        .s_arsize(dut_arsize), .s_arburst(dut_arburst),
        .s_arlock(1'b0), .s_arcache(4'd0), .s_arprot(3'd0), .s_arqos(4'd0),
        .s_aruser(1'b0),
        .s_arvalid(dut_arvalid), .s_arready(dut_arready),
        .s_rid(dut_rid), .s_rdata(dut_rdata), .s_rresp(dut_rresp),
        .s_rlast(dut_rlast), .s_ruser(dut_ruser_nc),
        .s_rvalid(dut_rvalid), .s_rready(dut_rready),

        .m_clk(m_clk), .m_rst(m_rst),
        .m_awid(dp_awid), .m_awaddr(dp_awaddr), .m_awlen(dp_awlen),
        .m_awsize(dp_awsize), .m_awburst(dp_awburst),
        .m_awlock(dp_awlock_nc), .m_awcache(dp_awcache_nc),
        .m_awprot(dp_awprot_nc), .m_awqos(dp_awqos_nc),
        .m_awuser(dp_awuser_nc),
        .m_awvalid(dp_awvalid), .m_awready(dp_awready),
        .m_wdata(dp_wdata), .m_wstrb(dp_wstrb), .m_wlast(dp_wlast),
        .m_wuser(dp_wuser_nc), .m_wvalid(dp_wvalid), .m_wready(dp_wready),
        .m_bid(dp_bid), .m_bresp(dp_bresp), .m_buser(1'b0),
        .m_bvalid(dp_bvalid), .m_bready(dp_bready),
        .m_arid(dp_arid), .m_araddr(dp_araddr), .m_arlen(dp_arlen),
        .m_arsize(dp_arsize), .m_arburst(dp_arburst),
        .m_arlock(dp_arlock_nc), .m_arcache(dp_arcache_nc),
        .m_arprot(dp_arprot_nc), .m_arqos(dp_arqos_nc),
        .m_aruser(dp_aruser_nc),
        .m_arvalid(dp_arvalid), .m_arready(dp_arready),
        .m_rid(dp_rid), .m_rdata(dp_rdata), .m_rresp(dp_rresp),
        .m_rlast(dp_rlast), .m_ruser(1'b0),
        .m_rvalid(dp_rvalid), .m_rready(dp_rready)
    );

    pb_lane_slave_model #(.SEED(32'h1234_5678)) u_dut_slave (
        .clk(m_clk), .rst(m_rst),
        .s_awid(dp_awid), .s_awaddr(dp_awaddr), .s_awlen(dp_awlen),
        .s_awsize(dp_awsize), .s_awburst(dp_awburst),
        .s_awvalid(dp_awvalid), .s_awready(dp_awready),
        .s_wdata(dp_wdata), .s_wstrb(dp_wstrb), .s_wlast(dp_wlast),
        .s_wvalid(dp_wvalid), .s_wready(dp_wready),
        .s_bid(dp_bid), .s_bresp(dp_bresp),
        .s_bvalid(dp_bvalid), .s_bready(dp_bready),
        .s_arid(dp_arid), .s_araddr(dp_araddr), .s_arlen(dp_arlen),
        .s_arsize(dp_arsize), .s_arburst(dp_arburst),
        .s_arvalid(dp_arvalid), .s_arready(dp_arready),
        .s_rid(dp_rid), .s_rdata(dp_rdata), .s_rresp(dp_rresp),
        .s_rlast(dp_rlast), .s_rvalid(dp_rvalid), .s_rready(dp_rready),
        .dbg_idx(dbg_idx), .dbg_data(dut_dbg_data),
        .dbg_last_awsize(dut_dbg_last_awsize),
        .dbg_last_arsize(dut_dbg_last_arsize)
    );

endmodule


// ═════════════════════════════════════════════════════════════════════
// pb_lane_slave_model — peripheral_bus.v's byte-lane policy, distilled.
//
// One outstanding write and one outstanding read.  Small pseudo-random
// response latency (LFSR) so the chains see genuine back-pressure.
// 1024 x 32-bit word memory addressed by addr[11:2] — the tb keeps all
// traffic inside one 4 KB window so there is no aliasing.
// ═════════════════════════════════════════════════════════════════════
module pb_lane_slave_model #(
    parameter [31:0] SEED      = 32'hACE1_2345,
    parameter        MEM_WORDS = 1024
) (
    input  wire         clk,
    input  wire         rst,

    input  wire [5:0]   s_awid,
    input  wire [31:0]  s_awaddr,
    input  wire [7:0]   s_awlen,
    input  wire [2:0]   s_awsize,
    input  wire [1:0]   s_awburst,
    input  wire         s_awvalid,
    output wire         s_awready,
    input  wire [127:0] s_wdata,
    input  wire [15:0]  s_wstrb,
    input  wire         s_wlast,
    input  wire         s_wvalid,
    output wire         s_wready,
    output wire [5:0]   s_bid,
    output wire [1:0]   s_bresp,
    output wire         s_bvalid,
    input  wire         s_bready,
    input  wire [5:0]   s_arid,
    input  wire [31:0]  s_araddr,
    input  wire [7:0]   s_arlen,
    input  wire [2:0]   s_arsize,
    input  wire [1:0]   s_arburst,
    input  wire         s_arvalid,
    output wire         s_arready,
    output wire [5:0]   s_rid,
    output wire [127:0] s_rdata,
    output wire [1:0]   s_rresp,
    output wire         s_rlast,
    output wire         s_rvalid,
    input  wire         s_rready,

    input  wire [9:0]   dbg_idx,
    output wire [31:0]  dbg_data,
    output wire [2:0]   dbg_last_awsize,
    output wire [2:0]   dbg_last_arsize
);
    localparam [1:0] WR_IDLE = 2'd0, WR_DATA = 2'd1,
                     WR_LAT  = 2'd2, WR_RESP = 2'd3;
    localparam [1:0] RD_IDLE = 2'd0, RD_LAT  = 2'd1, RD_RESP = 2'd2;

    reg [31:0] mem [0:MEM_WORDS-1];

    // ── shared latency LFSR (xnor-tapped, never all-ones) ────────────
    reg [31:0] lfsr;
    always @(posedge clk) begin
        if (rst) lfsr <= SEED;
        else     lfsr <= {lfsr[30:0],
                          lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
    end

    // ── write channel ────────────────────────────────────────────────
    reg [1:0]  wr_st;
    reg [5:0]  wr_id_q;
    reg [1:0]  wr_lane_q;
    reg [9:0]  wr_idx_q;
    reg [2:0]  wr_lat_q;
    reg [2:0]  wr_size_q;

    // 4:1 lane muxes on the CAPTURED AW lane — peripheral_bus.v:665-674.
    wire [31:0] wr_lane_data =
        (wr_lane_q == 2'd0) ? s_wdata[31:0]  :
        (wr_lane_q == 2'd1) ? s_wdata[63:32] :
        (wr_lane_q == 2'd2) ? s_wdata[95:64] :
                              s_wdata[127:96];
    wire [3:0]  wr_lane_strb =
        (wr_lane_q == 2'd0) ? s_wstrb[3:0]   :
        (wr_lane_q == 2'd1) ? s_wstrb[7:4]   :
        (wr_lane_q == 2'd2) ? s_wstrb[11:8]  :
                              s_wstrb[15:12];

    wire [31:0] wr_old = mem[wr_idx_q];
    wire [31:0] wr_new = {
        wr_lane_strb[3] ? wr_lane_data[31:24] : wr_old[31:24],
        wr_lane_strb[2] ? wr_lane_data[23:16] : wr_old[23:16],
        wr_lane_strb[1] ? wr_lane_data[15: 8] : wr_old[15: 8],
        wr_lane_strb[0] ? wr_lane_data[ 7: 0] : wr_old[ 7: 0]
    };

    assign s_awready = !rst && (wr_st == WR_IDLE);
    assign s_wready  = !rst && (wr_st == WR_DATA);
    assign s_bid     = wr_id_q;
    assign s_bresp   = 2'b00;
    assign s_bvalid  = (wr_st == WR_RESP);

    always @(posedge clk) begin
        if (rst) begin
            wr_st     <= WR_IDLE;
            wr_id_q   <= 6'd0;
            wr_lane_q <= 2'd0;
            wr_idx_q  <= 10'd0;
            wr_lat_q  <= 3'd0;
            wr_size_q <= 3'd0;
        end else begin
            case (wr_st)
                WR_IDLE: if (s_awvalid) begin
                    wr_id_q   <= s_awid;
                    wr_lane_q <= s_awaddr[3:2];
                    wr_idx_q  <= s_awaddr[11:2];
                    wr_size_q <= s_awsize;
                    wr_st     <= WR_DATA;
                end
                WR_DATA: if (s_wvalid) begin
                    // WSTRB=0 must be a genuine no-op.
                    if (|wr_lane_strb) mem[wr_idx_q] <= wr_new;
                    wr_lat_q <= lfsr[2:0];
                    wr_st    <= WR_LAT;
                end
                WR_LAT: begin
                    if (wr_lat_q == 3'd0) wr_st <= WR_RESP;
                    else                  wr_lat_q <= wr_lat_q - 3'd1;
                end
                WR_RESP: if (s_bready) wr_st <= WR_IDLE;
                default: wr_st <= WR_IDLE;
            endcase
        end
    end

    // ── read channel ─────────────────────────────────────────────────
    reg [1:0]  rd_st;
    reg [5:0]  rd_id_q;
    reg [1:0]  rd_lane_q;
    reg [31:0] rd_data_q;
    reg [2:0]  rd_lat_q;
    reg [2:0]  rd_size_q;

    assign s_arready = !rst && (rd_st == RD_IDLE);
    assign s_rid     = rd_id_q;
    // peripheral_bus.v:1573-1576 — selected lane driven, other three
    // HARD ZERO.  axi_pb_lane_widen's OR-reduce depends on this.
    assign s_rdata   = (rd_lane_q == 2'd0) ? {96'b0, rd_data_q} :
                       (rd_lane_q == 2'd1) ? {64'b0, rd_data_q, 32'b0} :
                       (rd_lane_q == 2'd2) ? {32'b0, rd_data_q, 64'b0} :
                                             {rd_data_q, 96'b0};
    assign s_rresp   = 2'b00;
    assign s_rlast   = 1'b1;
    assign s_rvalid  = (rd_st == RD_RESP);

    always @(posedge clk) begin
        if (rst) begin
            rd_st     <= RD_IDLE;
            rd_id_q   <= 6'd0;
            rd_lane_q <= 2'd0;
            rd_data_q <= 32'd0;
            rd_lat_q  <= 3'd0;
            rd_size_q <= 3'd0;
        end else begin
            case (rd_st)
                RD_IDLE: if (s_arvalid) begin
                    rd_id_q   <= s_arid;
                    rd_lane_q <= s_araddr[3:2];
                    rd_data_q <= mem[s_araddr[11:2]];
                    rd_size_q <= s_arsize;
                    rd_lat_q  <= lfsr[5:3];
                    rd_st     <= RD_LAT;
                end
                RD_LAT: begin
                    if (rd_lat_q == 3'd0) rd_st <= RD_RESP;
                    else                  rd_lat_q <= rd_lat_q - 3'd1;
                end
                RD_RESP: if (s_rready) rd_st <= RD_IDLE;
                default: rd_st <= RD_IDLE;
            endcase
        end
    end

    assign dbg_data        = mem[dbg_idx];
    assign dbg_last_awsize = wr_size_q;
    assign dbg_last_arsize = rd_size_q;

    // Unused burst attributes (single-beat only on this path).
    wire _unused = &{1'b0, s_awlen, s_awburst, s_arlen, s_arburst,
                     s_wlast, 1'b0};
endmodule

`default_nettype wire
