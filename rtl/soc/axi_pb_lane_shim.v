// axi_pb_lane_shim.v — 128b <-> 32b lane shims for the peripheral-bus (S1) path.
//
// Purpose
// ───────
// `u_pb_s1_cdc` (rtl/soc/fpga_top_peripherals.vh) used to carry a
// 128-bit AXI4 payload across the core_clk -> pb_clk boundary to reach
// peripherals whose widest register is 8 bits.  The W and R channel
// FIFOs are 16 entries deep, so that width cost ~285 LUT of LUTRAM and
// ~95 of the 131 timing-path endpoints this bridge contributes to a
// congested build (measured: `synth/timing_reports/timing_20260819_131707.rpt`,
// 58 endpoints in w_fifo + 49 in r_fifo, of which 95 are the
// `mem_reg_*` LUTRAM data arrays).
//
// These two shims sandwich the CDC so it only has to carry 32 data bits:
//
//   xbar S1 (128b) -> axi_pb_lane_narrow -> [32b CDC] ->
//                     axi_pb_lane_widen  -> peripheral_bus (128b)
//
// `peripheral_bus.v` is untouched and still sees exactly the 128-bit
// lane layout it always has.
//
// Why this and not `axi_wide_to_axilite`
// ──────────────────────────────────────
// `axi_wide_to_axilite` (used for S2/S4/S5) is AXI4 -> AXI-Lite and is
// ONE outstanding read + ONE outstanding write.  Putting it in front of
// the CDC would (a) drop the AXI ID that `axi_xbar` needs to route S1's
// R beats back to the right master slot (`axi_xbar.v:3408`,
// `s1_rtgt = s1_rid[XID_WIDTH-1 -: SLOT_W]`), and (b) prevent the CDC's
// 4-deep AW/AR FIFOs from pre-staging a second master's transaction
// while the first is being serviced.  These shims instead keep full
// AXI4 with the ID intact and change nothing but the data width.
//
// Both modules are PURELY COMBINATIONAL — no state, no reset, no
// ordering assumption.  That matters here: the S1 CDC is deliberately
// held on `core_rst_bank[1]` / `pb_core_rst_bank[3]` (the *core* banks,
// not the full-reset banks) so the host path to `debug_ctrl` survives a
// JTAG/button full reset.  A shim with no reset at all cannot perturb
// that property.
//
// Lane policy (matches peripheral_bus.v's header, "Byte-lane logic")
// ──────────────────────────────────────────────────────────────────
// A 32-bit word lives in the 128-bit lane selected by addr[3:2].
//   * Write, narrow side: the W channel carries no address, so the lane
//     is recovered from WSTRB — for a single-beat peripheral write
//     exactly one 32-bit lane is strobed.  This introduces no new
//     assumption: peripheral_bus already muxes WSTRB by addr[3:2], so a
//     master whose strobe lane disagreed with its address lane is
//     already broken today (it would present an all-zero lane strobe).
//   * Write, widen side: the 32-bit word and its 4 strobe bits are
//     REPLICATED into all four lanes.  peripheral_bus reads exactly one
//     lane (`wr_lane_data`/`wr_lane_strb`, 4:1 muxes on addr[3:2]) and
//     never inspects the others, so replication is invisible to it and
//     removes any need to correlate W with AW.
//   * Read, widen side: peripheral_bus drives the selected lane and
//     HARD-ZEROES the other three (`peripheral_bus.v:1573-1576`, single
//     driver), so a 4-way OR-reduce recovers the word exactly.
//   * Read, narrow side: the 32-bit result is REPLICATED into all four
//     lanes.  Every master on this path recovers its word by selecting
//     lane addr[3:2] (`axi_narrow_to_wide.v`), so replication is
//     observationally identical at every consumer.  This is the one
//     intentional difference from the 128-bit path, which returned
//     zeroes in the unselected lanes; `tb-pb-lane-shim` asserts the
//     selected lane matches bit-for-bit.
//
// Burst safety: S1 is burst-free by construction — `axi_xbar.v`'s
// `is_lite_only_slv()` (:1328) includes `XBAR_SLV_IO`, so any AWLEN/
// ARLEN != 0 aimed at S1 is answered locally with SLVERR and never
// forwarded.  These shims therefore never see a multi-beat burst; they
// pass LEN/LAST through unchanged rather than assuming anything.
//
// Verilog-2005.  No clk, no rst — combinational by construction.

`default_nettype none

// ─────────────────────────────────────────────────────────────────────
// axi_pb_lane_narrow — 128-bit AXI4 slave face, 32-bit AXI4 master face
// ─────────────────────────────────────────────────────────────────────
module axi_pb_lane_narrow #(
    parameter ID_WIDTH   = 6,
    parameter ADDR_WIDTH = 32,
    parameter WIDE_DW    = 128,
    parameter NARROW_DW  = 32
) (
    // Wide AXI4 slave face (from axi_xbar S1).
    input  wire [ID_WIDTH-1:0]     s_awid,
    input  wire [ADDR_WIDTH-1:0]   s_awaddr,
    input  wire [7:0]              s_awlen,
    input  wire [2:0]              s_awsize,
    input  wire [1:0]              s_awburst,
    input  wire                    s_awvalid,
    output wire                    s_awready,
    input  wire [WIDE_DW-1:0]      s_wdata,
    input  wire [WIDE_DW/8-1:0]    s_wstrb,
    input  wire                    s_wlast,
    input  wire                    s_wvalid,
    output wire                    s_wready,
    output wire [ID_WIDTH-1:0]     s_bid,
    output wire [1:0]              s_bresp,
    output wire                    s_bvalid,
    input  wire                    s_bready,
    input  wire [ID_WIDTH-1:0]     s_arid,
    input  wire [ADDR_WIDTH-1:0]   s_araddr,
    input  wire [7:0]              s_arlen,
    input  wire [2:0]              s_arsize,
    input  wire [1:0]              s_arburst,
    input  wire                    s_arvalid,
    output wire                    s_arready,
    output wire [ID_WIDTH-1:0]     s_rid,
    output wire [WIDE_DW-1:0]      s_rdata,
    output wire [1:0]              s_rresp,
    output wire                    s_rlast,
    output wire                    s_rvalid,
    input  wire                    s_rready,

    // Narrow AXI4 master face (into the CDC).
    output wire [ID_WIDTH-1:0]     m_awid,
    output wire [ADDR_WIDTH-1:0]   m_awaddr,
    output wire [7:0]              m_awlen,
    output wire [2:0]              m_awsize,
    output wire [1:0]              m_awburst,
    output wire                    m_awvalid,
    input  wire                    m_awready,
    output wire [NARROW_DW-1:0]    m_wdata,
    output wire [NARROW_DW/8-1:0]  m_wstrb,
    output wire                    m_wlast,
    output wire                    m_wvalid,
    input  wire                    m_wready,
    input  wire [ID_WIDTH-1:0]     m_bid,
    input  wire [1:0]              m_bresp,
    input  wire                    m_bvalid,
    output wire                    m_bready,
    output wire [ID_WIDTH-1:0]     m_arid,
    output wire [ADDR_WIDTH-1:0]   m_araddr,
    output wire [7:0]              m_arlen,
    output wire [2:0]              m_arsize,
    output wire [1:0]              m_arburst,
    output wire                    m_arvalid,
    input  wire                    m_arready,
    input  wire [ID_WIDTH-1:0]     m_rid,
    input  wire [NARROW_DW-1:0]    m_rdata,
    input  wire [1:0]              m_rresp,
    input  wire                    m_rlast,
    input  wire                    m_rvalid,
    output wire                    m_rready
);
    localparam [2:0] NARROW_SIZE_MAX = 3'd2;   // 32 bits = 4 bytes

    // ── AW / AR: straight through, size clamped to the narrow bus ────
    assign m_awid    = s_awid;
    assign m_awaddr  = s_awaddr;
    assign m_awlen   = s_awlen;
    assign m_awsize  = (s_awsize > NARROW_SIZE_MAX) ? NARROW_SIZE_MAX : s_awsize;
    assign m_awburst = s_awburst;
    assign m_awvalid = s_awvalid;
    assign s_awready = m_awready;

    assign m_arid    = s_arid;
    assign m_araddr  = s_araddr;
    assign m_arlen   = s_arlen;
    assign m_arsize  = (s_arsize > NARROW_SIZE_MAX) ? NARROW_SIZE_MAX : s_arsize;
    assign m_arburst = s_arburst;
    assign m_arvalid = s_arvalid;
    assign s_arready = m_arready;

    // ── W: recover the active 32-bit lane from WSTRB ─────────────────
    wire [3:0] lane_hit = {|s_wstrb[15:12], |s_wstrb[11:8],
                           |s_wstrb[ 7: 4], |s_wstrb[ 3: 0]};
    wire [1:0] w_lane = lane_hit[0] ? 2'd0 :
                        lane_hit[1] ? 2'd1 :
                        lane_hit[2] ? 2'd2 :
                        lane_hit[3] ? 2'd3 : 2'd0;

    assign m_wdata  = s_wdata[w_lane*32 +: 32];
    assign m_wstrb  = s_wstrb[w_lane*4  +: 4];

    // PRECONDITION: at most ONE 32-bit lane may be strobed.
    //
    // The lane is recovered from WSTRB because the W channel carries no
    // address.  That is exact when one lane is strobed, and it is what
    // every master on S1 produces today (`axi_narrow_to_wide` places a
    // single 32-bit word in the lane its address selects and strobes
    // only that lane; the xbar rejects bursts to S1 outright via
    // `is_lite_only_slv()`).
    //
    // It is NOT exact for a beat that strobes two lanes when the
    // ADDRESS lane is not the lowest strobed one: this shim would
    // forward the lowest, and `peripheral_bus` — which selects by
    // addr[3:2] — would commit the wrong word.  The 128-bit path was
    // correct there.  Note the information is not recoverable at 32
    // bits: the right answer is always "the address lane", which the W
    // channel cannot see without AW/W correlation state.  State was
    // deliberately NOT added here — this bridge's reset behaviour is
    // load-bearing (it stays on the core reset banks so the host path
    // to debug_ctrl survives a JTAG full reset), and a provably
    // unreachable case does not justify new reset-sensitive logic.
    //
    // So: assert it in simulation instead, which turns a silent
    // data-corruption risk into a loud failure the moment some future
    // master makes it reachable.  `tb-pb-lane-shim` documents how to
    // reproduce the failing case.
    // synthesis translate_off
    always @(*) begin
        if (s_wvalid && (s_wstrb != {(WIDE_DW/8){1'b0}})) begin
            if (!((lane_hit == 4'b0001) || (lane_hit == 4'b0010) ||
                  (lane_hit == 4'b0100) || (lane_hit == 4'b1000))) begin
                $display("axi_pb_lane_narrow: PRECONDITION VIOLATED at %0t: WSTRB=0x%h strobes %0d lanes; this shim can only carry one 32-bit lane and will forward the LOWEST, which peripheral_bus (selecting by addr[3:2]) may not agree with.",
                         $time, s_wstrb,
                         lane_hit[0] + lane_hit[1] + lane_hit[2] + lane_hit[3]);
                $stop;
            end
        end
    end
    // synthesis translate_on
    assign m_wlast  = s_wlast;
    assign m_wvalid = s_wvalid;
    assign s_wready = m_wready;

    // ── B: straight through ──────────────────────────────────────────
    assign s_bid    = m_bid;
    assign s_bresp  = m_bresp;
    assign s_bvalid = m_bvalid;
    assign m_bready = s_bready;

    // ── R: replicate the word into all four lanes (see header) ───────
    assign s_rid    = m_rid;
    assign s_rdata  = {4{m_rdata}};
    assign s_rresp  = m_rresp;
    assign s_rlast  = m_rlast;
    assign s_rvalid = m_rvalid;
    assign m_rready = s_rready;
endmodule

// ─────────────────────────────────────────────────────────────────────
// axi_pb_lane_widen — 32-bit AXI4 slave face, 128-bit AXI4 master face
// ─────────────────────────────────────────────────────────────────────
module axi_pb_lane_widen #(
    parameter ID_WIDTH   = 6,
    parameter ADDR_WIDTH = 32,
    parameter WIDE_DW    = 128,
    parameter NARROW_DW  = 32
) (
    // Narrow AXI4 slave face (out of the CDC).
    input  wire [ID_WIDTH-1:0]     s_awid,
    input  wire [ADDR_WIDTH-1:0]   s_awaddr,
    input  wire [7:0]              s_awlen,
    input  wire [2:0]              s_awsize,
    input  wire [1:0]              s_awburst,
    input  wire                    s_awvalid,
    output wire                    s_awready,
    input  wire [NARROW_DW-1:0]    s_wdata,
    input  wire [NARROW_DW/8-1:0]  s_wstrb,
    input  wire                    s_wlast,
    input  wire                    s_wvalid,
    output wire                    s_wready,
    output wire [ID_WIDTH-1:0]     s_bid,
    output wire [1:0]              s_bresp,
    output wire                    s_bvalid,
    input  wire                    s_bready,
    input  wire [ID_WIDTH-1:0]     s_arid,
    input  wire [ADDR_WIDTH-1:0]   s_araddr,
    input  wire [7:0]              s_arlen,
    input  wire [2:0]              s_arsize,
    input  wire [1:0]              s_arburst,
    input  wire                    s_arvalid,
    output wire                    s_arready,
    output wire [ID_WIDTH-1:0]     s_rid,
    output wire [NARROW_DW-1:0]    s_rdata,
    output wire [1:0]              s_rresp,
    output wire                    s_rlast,
    output wire                    s_rvalid,
    input  wire                    s_rready,

    // Wide AXI4 master face (into peripheral_bus).
    output wire [ID_WIDTH-1:0]     m_awid,
    output wire [ADDR_WIDTH-1:0]   m_awaddr,
    output wire [7:0]              m_awlen,
    output wire [2:0]              m_awsize,
    output wire [1:0]              m_awburst,
    output wire                    m_awvalid,
    input  wire                    m_awready,
    output wire [WIDE_DW-1:0]      m_wdata,
    output wire [WIDE_DW/8-1:0]    m_wstrb,
    output wire                    m_wlast,
    output wire                    m_wvalid,
    input  wire                    m_wready,
    input  wire [ID_WIDTH-1:0]     m_bid,
    input  wire [1:0]              m_bresp,
    input  wire                    m_bvalid,
    output wire                    m_bready,
    output wire [ID_WIDTH-1:0]     m_arid,
    output wire [ADDR_WIDTH-1:0]   m_araddr,
    output wire [7:0]              m_arlen,
    output wire [2:0]              m_arsize,
    output wire [1:0]              m_arburst,
    output wire                    m_arvalid,
    input  wire                    m_arready,
    input  wire [ID_WIDTH-1:0]     m_rid,
    input  wire [WIDE_DW-1:0]      m_rdata,
    input  wire [1:0]              m_rresp,
    input  wire                    m_rlast,
    input  wire                    m_rvalid,
    output wire                    m_rready
);
    // ── AW / AR: straight through ────────────────────────────────────
    assign m_awid    = s_awid;
    assign m_awaddr  = s_awaddr;
    assign m_awlen   = s_awlen;
    assign m_awsize  = s_awsize;
    assign m_awburst = s_awburst;
    assign m_awvalid = s_awvalid;
    assign s_awready = m_awready;

    assign m_arid    = s_arid;
    assign m_araddr  = s_araddr;
    assign m_arlen   = s_arlen;
    assign m_arsize  = s_arsize;
    assign m_arburst = s_arburst;
    assign m_arvalid = s_arvalid;
    assign s_arready = m_arready;

    // ── W: replicate word + strobes into all four lanes (see header) ─
    assign m_wdata  = {4{s_wdata}};
    assign m_wstrb  = {4{s_wstrb}};
    assign m_wlast  = s_wlast;
    assign m_wvalid = s_wvalid;
    assign s_wready = m_wready;

    // ── B: straight through ──────────────────────────────────────────
    assign s_bid    = m_bid;
    assign s_bresp  = m_bresp;
    assign s_bvalid = m_bvalid;
    assign m_bready = s_bready;

    // ── R: OR-reduce — peripheral_bus zeroes the unselected lanes ────
    assign s_rid    = m_rid;
    assign s_rdata  = m_rdata[31:0] | m_rdata[63:32] |
                      m_rdata[95:64] | m_rdata[127:96];
    assign s_rresp  = m_rresp;
    assign s_rlast  = m_rlast;
    assign s_rvalid = m_rvalid;
    assign m_rready = s_rready;
endmodule

`default_nettype wire
