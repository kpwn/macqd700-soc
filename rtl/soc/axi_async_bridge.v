// axi_async_bridge.v — AXI4 clock-domain-crossing bridge built on async_fifo.
//
// Purpose
// ───────
// Pass an AXI4 transaction stream intact from a master clocked at `s_clk`
// to a slave clocked at `m_clk` (or vice versa — the protocol is
// symmetric in this module's name; the slave port faces the upstream
// master, the master port faces the downstream slave).
//
// Each of the 5 AXI4 channels (AW, W, B, AR, R) is carried by an
// independent async_fifo:
//
//                ┌─────────────┐
//   AXI master ──│ AW fifo     │── AXI slave
//                │ W  fifo     │
//                │ B  fifo     │
//                │ AR fifo     │
//                │ R  fifo     │
//                └─────────────┘
//
// Each fifo maintains its own backpressure: a stalled downstream channel
// simply lets its FIFO fill → wr_full asserts → upstream sees !ready.
// Bursts pass unchanged: AWLEN / ARLEN / WLAST / RLAST are carried in
// the same payload as the data word, so the slave reconstructs the
// original burst.
//
// Parameters
// ──────────
//   DATA_WIDTH       : AXI data bus (default 64).
//   ADDR_WIDTH       : AXI address bus (default 32).
//   ID_WIDTH         : AXI ID bus (default 4).
//   USER_WIDTH       : AXI user bus (default 1 — we don't carry USER at
//                      the bridge level beyond 1 bit to keep the channel
//                      payloads minimal; wider USER can be added by bumping
//                      this parameter).
//   AW_DEPTH_LOG2    : AW channel FIFO depth-log2 (default 2 → 4 entries).
//   AR_DEPTH_LOG2    : AR channel FIFO depth-log2 (default 2).
//   W_DEPTH_LOG2     : W  channel FIFO depth-log2 (default 4 → 16 entries).
//   R_DEPTH_LOG2     : R  channel FIFO depth-log2 (default 4).
//   B_DEPTH_LOG2     : B  channel FIFO depth-log2 (default 2).
//
// Two sides of the bridge are fully decoupled: s_clk can be faster,
// slower, or completely asynchronous to m_clk.  Resets: `s_rst` resets
// the upstream side of each FIFO; `m_rst` resets the downstream side.
// Both resets are synchronous active-high in their own domain.  For a
// clean full-system restart, assert both at once (holding several cycles
// in each domain).
//
// T14 stale-beat protection (R and B channels)
// ─────────────────────────────────────────────
// Under a core-only reset (s_rst pulses, m_rst stays low — e.g. a JTAG
// debug reset that leaves the DDR MIG running), a burst already accepted
// by the downstream slave keeps being delivered by that slave (it was
// never told to abandon it) even though async_fifo's own T6
// coupled-reset handshake correctly zeroes both the R/B fifo's pointers.
// The slave's still-outstanding beats then get accepted into the
// freshly-zeroed fifo post-handshake and forwarded upstream as stale
// data.  `axi_bridge_stale_sink` (rtl/soc/axi_bridge_stale_sink.v)
// tracks outstanding AR/AW bursts entirely in the m_clk domain and
// sinks (discards, never writes into the R/B fifo) exactly that many
// owed completions after an s-side reset, before resuming normal
// forwarding.  See that file's header for the full root-cause writeup.
//
// T162 mid-W-burst abandonment (W channel)
// ─────────────────────────────────────────
// The stale sink only covers bursts that COMPLETED on the m-side.  A
// core-only reset landing MID-W-BURST leaves the slave parked on a
// burst whose remaining beats no longer exist, and because AW and W
// ride independent async_fifos a later, unrelated write's W beats then
// get consumed as that old burst's continuation and committed to the
// OLD address -- silently, permanently shifting the AW/W pairing for
// every subsequent write.  `axi_bridge_w_pad`
// (rtl/soc/axi_bridge_w_pad.v) completes the abandoned burst(s) with
// wstrb=0 filler beats and correct WLAST before any fresh W data is
// admitted; the resulting BRESPs are exactly the debt the stale sink
// already snapshotted, so they are sunk, not replayed.
//
// synth: instantiate once per boundary between clk_core and clk_pb,
// clk_core and clk_ddr_ui, etc.  See docs/clocking.md §14.
// sim: Verilator can drive s_clk and m_clk at different rates; unit tb
// tb_axi_async_bridge.cpp exercises multiple ratios (including the
// s-side-only-reset-with-outstanding-burst regression scenarios).

`default_nettype none

module axi_async_bridge #(
    parameter DATA_WIDTH    = 64,
    parameter ADDR_WIDTH    = 32,
    parameter ID_WIDTH      = 4,
    parameter USER_WIDTH    = 1,
    parameter AW_DEPTH_LOG2 = 2,
    parameter AR_DEPTH_LOG2 = 2,
    parameter W_DEPTH_LOG2  = 4,
    parameter R_DEPTH_LOG2  = 4,
    parameter B_DEPTH_LOG2  = 2
) (
    // ── Upstream side (slave port; master clocks it on s_clk) ──────
    input  wire                       s_clk,
    input  wire                       s_rst,
    // AW
    input  wire [ID_WIDTH-1:0]        s_awid,
    input  wire [ADDR_WIDTH-1:0]      s_awaddr,
    input  wire [7:0]                 s_awlen,
    input  wire [2:0]                 s_awsize,
    input  wire [1:0]                 s_awburst,
    input  wire                       s_awlock,
    input  wire [3:0]                 s_awcache,
    input  wire [2:0]                 s_awprot,
    input  wire [3:0]                 s_awqos,
    input  wire [USER_WIDTH-1:0]      s_awuser,
    input  wire                       s_awvalid,
    output wire                       s_awready,
    // W
    input  wire [DATA_WIDTH-1:0]      s_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]  s_wstrb,
    input  wire                       s_wlast,
    input  wire [USER_WIDTH-1:0]      s_wuser,
    input  wire                       s_wvalid,
    output wire                       s_wready,
    // B
    output wire [ID_WIDTH-1:0]        s_bid,
    output wire [1:0]                 s_bresp,
    output wire [USER_WIDTH-1:0]      s_buser,
    output wire                       s_bvalid,
    input  wire                       s_bready,
    // AR
    input  wire [ID_WIDTH-1:0]        s_arid,
    input  wire [ADDR_WIDTH-1:0]      s_araddr,
    input  wire [7:0]                 s_arlen,
    input  wire [2:0]                 s_arsize,
    input  wire [1:0]                 s_arburst,
    input  wire                       s_arlock,
    input  wire [3:0]                 s_arcache,
    input  wire [2:0]                 s_arprot,
    input  wire [3:0]                 s_arqos,
    input  wire [USER_WIDTH-1:0]      s_aruser,
    input  wire                       s_arvalid,
    output wire                       s_arready,
    // R
    output wire [ID_WIDTH-1:0]        s_rid,
    output wire [DATA_WIDTH-1:0]      s_rdata,
    output wire [1:0]                 s_rresp,
    output wire                       s_rlast,
    output wire [USER_WIDTH-1:0]      s_ruser,
    output wire                       s_rvalid,
    input  wire                       s_rready,

    // ── Downstream side (master port; slave clocks it on m_clk) ────
    input  wire                       m_clk,
    input  wire                       m_rst,
    // AW
    output wire [ID_WIDTH-1:0]        m_awid,
    output wire [ADDR_WIDTH-1:0]      m_awaddr,
    output wire [7:0]                 m_awlen,
    output wire [2:0]                 m_awsize,
    output wire [1:0]                 m_awburst,
    output wire                       m_awlock,
    output wire [3:0]                 m_awcache,
    output wire [2:0]                 m_awprot,
    output wire [3:0]                 m_awqos,
    output wire [USER_WIDTH-1:0]      m_awuser,
    output wire                       m_awvalid,
    input  wire                       m_awready,
    // W
    output wire [DATA_WIDTH-1:0]      m_wdata,
    output wire [(DATA_WIDTH/8)-1:0]  m_wstrb,
    output wire                       m_wlast,
    output wire [USER_WIDTH-1:0]      m_wuser,
    output wire                       m_wvalid,
    input  wire                       m_wready,
    // B
    input  wire [ID_WIDTH-1:0]        m_bid,
    input  wire [1:0]                 m_bresp,
    input  wire [USER_WIDTH-1:0]      m_buser,
    input  wire                       m_bvalid,
    output wire                       m_bready,
    // AR
    output wire [ID_WIDTH-1:0]        m_arid,
    output wire [ADDR_WIDTH-1:0]      m_araddr,
    output wire [7:0]                 m_arlen,
    output wire [2:0]                 m_arsize,
    output wire [1:0]                 m_arburst,
    output wire                       m_arlock,
    output wire [3:0]                 m_arcache,
    output wire [2:0]                 m_arprot,
    output wire [3:0]                 m_arqos,
    output wire [USER_WIDTH-1:0]      m_aruser,
    output wire                       m_arvalid,
    input  wire                       m_arready,
    // R
    input  wire [ID_WIDTH-1:0]        m_rid,
    input  wire [DATA_WIDTH-1:0]      m_rdata,
    input  wire [1:0]                 m_rresp,
    input  wire                       m_rlast,
    input  wire [USER_WIDTH-1:0]      m_ruser,
    input  wire                       m_rvalid,
    output wire                       m_rready
);

    // ── Channel payload widths ─────────────────────────────────────
    // AW: id + addr + len + size + burst + lock + cache + prot + qos + user
    localparam AW_W = ID_WIDTH + ADDR_WIDTH + 8 + 3 + 2 + 1 + 4 + 3 + 4 + USER_WIDTH;
    // W: data + strb + last + user
    localparam W_W  = DATA_WIDTH + (DATA_WIDTH/8) + 1 + USER_WIDTH;
    // B: id + resp + user
    localparam B_W  = ID_WIDTH + 2 + USER_WIDTH;
    // AR: same as AW
    localparam AR_W = AW_W;
    // R: id + data + resp + last + user
    localparam R_W  = ID_WIDTH + DATA_WIDTH + 2 + 1 + USER_WIDTH;

    wire aw_m_reset_engaged, w_m_reset_engaged, ar_m_reset_engaged;
    wire b_m_reset_engaged, r_m_reset_engaged;

    // ── T14: s-side-reset-event witness, crossed into m_clk ─────────
    // Plain edge detector on s_rst itself (s_clk domain) — deliberately
    // NOT gated by any reset of its own: this register's whole job is
    // to notice s_rst's rising edge, so tying its own reset to s_rst
    // would blind it to the very event it must report.
    reg s_rst_d;
    always @(posedge s_clk) s_rst_d <= s_rst;
    wire s_rst_rise = s_rst && !s_rst_d;

    // Toggle-based single-event CDC (rtl/board/pulse_cdc.v, already
    // used project-wide e.g. the DAFB VBL -> VIA1 CA1 path).  src_rst is
    // tied low for the same reason as s_rst_d above: this signal must
    // survive the very s_rst pulse it is reporting.  dst_rst = m_rst is
    // correct — a genuine m-side reset means the outstanding-burst
    // trackers below reset too (see axi_bridge_stale_sink.v header).
    wire s_rst_event_m;
    pulse_cdc u_srst_pulse_cdc (
        .src_clk(s_clk), .src_rst(1'b0), .src_pulse(s_rst_rise),
        .dst_clk(m_clk), .dst_rst(m_rst), .dst_pulse(s_rst_event_m)
    );

    // Do not snapshot stale-response or W-padding debt until every channel
    // FIFO has acknowledged the source reset in the m_clk domain. The reset
    // event and FIFO engagement use independent synchronizers; relying on a
    // nominal stage-count ordering lets hardware phase/metastability move a
    // pre-reset AR/AW across after the snapshot.
    wire all_s_fifos_frozen_m = aw_m_reset_engaged && w_m_reset_engaged &&
                                ar_m_reset_engaged && b_m_reset_engaged &&
                                r_m_reset_engaged;
    reg s_reset_pending_m;
    wire s_reset_recovery_event_m = s_reset_pending_m && all_s_fifos_frozen_m;
    always @(posedge m_clk) begin
        if (m_rst) begin
            s_reset_pending_m <= 1'b0;
        end else begin
            if (s_rst_event_m) s_reset_pending_m <= 1'b1;
            else if (s_reset_recovery_event_m) s_reset_pending_m <= 1'b0;
        end
    end

    // ── AW channel FIFO (s_clk → m_clk) ────────────────────────────
    wire [AW_W-1:0] aw_wdata = {s_awid, s_awaddr, s_awlen, s_awsize,
                                s_awburst, s_awlock, s_awcache,
                                s_awprot, s_awqos, s_awuser};
    wire [AW_W-1:0] aw_rdata;
    wire            aw_full, aw_empty;
    reg  [AW_W-1:0] aw_stage_data;
    reg             aw_stage_valid;

    // Registered elastic stage at the FIFO read boundary.  The async FIFO
    // is show-ahead, so exposing aw_rdata/aw_empty directly made its live
    // read-pointer decode part of every downstream AW acceptance path at
    // 333 MHz.  Pop into this stage instead; on an outgoing handshake it
    // can capture the next FIFO entry in the same cycle, preserving one AW
    // per cycle throughput.
    wire m_awvalid_i = aw_stage_valid && !w_pad_aw_block && !aw_m_reset_engaged;
    wire aw_fwd_c    = m_awvalid_i && m_awready;
    wire aw_fifo_pop = !aw_empty && (!aw_stage_valid || aw_fwd_c) &&
                       !aw_m_reset_engaged;

    async_fifo #(.WIDTH(AW_W), .DEPTH_LOG2(AW_DEPTH_LOG2)) aw_fifo (
        .wclk(s_clk), .wrst(s_rst),
        .wr_en(s_awvalid && !aw_full), .wr_data(aw_wdata),
        .wr_full(aw_full), .wr_almost_full(), .wr_reset_engaged(),
        .rclk(m_clk), .rrst(m_rst),
        .rd_en(aw_fifo_pop),
        .rd_data(aw_rdata),
        .rd_empty(aw_empty), .rd_almost_empty(),
        .rd_reset_engaged(aw_m_reset_engaged)
    );
    assign s_awready = !aw_full;

    always @(posedge m_clk) begin
        if (m_rst || aw_m_reset_engaged) begin
            aw_stage_data  <= {AW_W{1'b0}};
            aw_stage_valid <= 1'b0;
        end else if (!aw_stage_valid || aw_fwd_c) begin
            aw_stage_valid <= !aw_empty;
            if (!aw_empty)
                aw_stage_data <= aw_rdata;
        end
    end

    // T162: hold AW off while the W-burst tracker is at capacity (see
    // axi_bridge_w_pad.v -- ordinary AXI backpressure, never reachable
    // with the write acceptors this design actually has downstream).
    // AW handshake with the downstream slave — the single "a write burst
    // has been committed to the slave" event, shared by the B-channel
    // stale sink (debt) and the W-channel filler-completion tracker.
    assign m_awvalid = m_awvalid_i;
    assign {m_awid, m_awaddr, m_awlen, m_awsize,
            m_awburst, m_awlock, m_awcache,
            m_awprot, m_awqos, m_awuser} = aw_stage_data;

    // ── W channel FIFO (s_clk → m_clk) ─────────────────────────────
    // T162: a mid-W-burst s-side reset abandons a burst the downstream
    // slave is still parked on.  axi_bridge_w_pad completes it with
    // wstrb=0 filler beats before any fresh W data is admitted -- see
    // that file's header for the corruption mechanism this closes.
    wire [W_W-1:0] w_wdata = {s_wdata, s_wstrb, s_wlast, s_wuser};
    wire [W_W-1:0] w_rdata;
    wire           w_full, w_empty;

    wire w_pad_active, w_pad_wlast, w_pad_track_valid, w_pad_aw_block;

    axi_bridge_w_pad #(.TAIL_DEPTH_LOG2(3)) u_w_pad (
        .m_clk(m_clk), .m_rst(m_rst),
        .reset_event(s_reset_recovery_event_m),
        .aw_fwd(aw_fwd_c),
        .aw_len(m_awlen),
        .w_fire(m_wvalid && m_wready),
        .track_valid(w_pad_track_valid),
        .aw_block(w_pad_aw_block),
        .pad_active(w_pad_active),
        .pad_wlast(w_pad_wlast)
    );

    async_fifo #(.WIDTH(W_W), .DEPTH_LOG2(W_DEPTH_LOG2)) w_fifo (
        .wclk(s_clk), .wrst(s_rst),
        .wr_en(s_wvalid && !w_full), .wr_data(w_wdata),
        .wr_full(w_full), .wr_almost_full(), .wr_reset_engaged(),
        .rclk(m_clk), .rrst(m_rst),
        .rd_en(m_wvalid_i && m_wready && !w_pad_active),
        .rd_data(w_rdata),
        .rd_empty(w_empty), .rd_almost_empty(),
        .rd_reset_engaged(w_m_reset_engaged)
    );
    assign s_wready = !w_full;
    // Real W beats only flow once their own AW has been forwarded to
    // the slave (w_pad_track_valid) -- otherwise the beat would reach
    // the slave with no tracked burst length behind it, making the
    // filler count unknowable.  All of the mux selects below are plain
    // registers, so nothing here deepens the W datapath.
    wire m_wvalid_i = !w_empty && w_pad_track_valid;
    wire [DATA_WIDTH-1:0]     w_fifo_data;
    wire [(DATA_WIDTH/8)-1:0] w_fifo_strb;
    wire                      w_fifo_last;
    wire [USER_WIDTH-1:0]     w_fifo_user;
    assign {w_fifo_data, w_fifo_strb, w_fifo_last, w_fifo_user} = w_rdata;

    assign m_wvalid = w_pad_active ? 1'b1                          : m_wvalid_i;
    assign m_wdata  = w_pad_active ? {DATA_WIDTH{1'b0}}            : w_fifo_data;
    assign m_wstrb  = w_pad_active ? {(DATA_WIDTH/8){1'b0}}        : w_fifo_strb;
    assign m_wlast  = w_pad_active ? w_pad_wlast                   : w_fifo_last;
    assign m_wuser  = w_pad_active ? {USER_WIDTH{1'b0}}            : w_fifo_user;

    // ── B channel FIFO (m_clk → s_clk) ─────────────────────────────
    // T14: sink stale BRESPs owed to a write burst forwarded before an
    // s-side-only reset (see axi_bridge_stale_sink.v header + the
    // module-level comment above).  b_complete_c is gated on the FINAL
    // m_bready (post-sink) below, not on a raw m_bvalid, so the
    // outstanding tracker only counts genuine handshakes.
    wire b_aw_fwd_c   = aw_fwd_c;
    wire b_complete_c = m_bvalid && m_bready;
    wire b_sinking;

    axi_bridge_stale_sink #(.OUTSTANDING_W(4)) u_b_stale_sink (
        .m_clk(m_clk), .m_rst(m_rst),
        .reset_event(s_reset_recovery_event_m),
        .fwd_event(b_aw_fwd_c),
        .complete_event(b_complete_c),
        .sinking(b_sinking)
    );

    wire [B_W-1:0] b_wdata = {m_bid, m_bresp, m_buser};
    wire [B_W-1:0] b_rdata;
    wire           b_full, b_empty;

    async_fifo #(.WIDTH(B_W), .DEPTH_LOG2(B_DEPTH_LOG2)) b_fifo (
        .wclk(m_clk), .wrst(m_rst),
        .wr_en(m_bvalid && !b_full && !b_sinking), .wr_data(b_wdata),
        .wr_full(b_full), .wr_almost_full(),
        .wr_reset_engaged(b_m_reset_engaged),
        .rclk(s_clk), .rrst(s_rst),
        .rd_en(s_bvalid_i && s_bready),
        .rd_data(b_rdata),
        .rd_empty(b_empty), .rd_almost_empty(), .rd_reset_engaged()
    );
    // While sinking, always accept (and discard) the slave's stale
    // BRESP — no reason to backpressure it, we're not storing it.
    assign m_bready = b_sinking ? 1'b1 : !b_full;
    wire s_bvalid_i = !b_empty;
    assign s_bvalid = s_bvalid_i;
    assign {s_bid, s_bresp, s_buser} = b_rdata;

    // ── AR channel FIFO (s_clk → m_clk) ────────────────────────────
    wire [AR_W-1:0] ar_wdata = {s_arid, s_araddr, s_arlen, s_arsize,
                                s_arburst, s_arlock, s_arcache,
                                s_arprot, s_arqos, s_aruser};
    wire [AR_W-1:0] ar_rdata;
    wire            ar_full, ar_empty;

    async_fifo #(.WIDTH(AR_W), .DEPTH_LOG2(AR_DEPTH_LOG2)) ar_fifo (
        .wclk(s_clk), .wrst(s_rst),
        .wr_en(s_arvalid && !ar_full), .wr_data(ar_wdata),
        .wr_full(ar_full), .wr_almost_full(), .wr_reset_engaged(),
        .rclk(m_clk), .rrst(m_rst),
        .rd_en(m_arvalid_i && m_arready),
        .rd_data(ar_rdata),
        .rd_empty(ar_empty), .rd_almost_empty(),
        .rd_reset_engaged(ar_m_reset_engaged)
    );
    assign s_arready = !ar_full;
    wire m_arvalid_i = !ar_empty;
    assign m_arvalid = m_arvalid_i;
    assign {m_arid, m_araddr, m_arlen, m_arsize,
            m_arburst, m_arlock, m_arcache,
            m_arprot, m_arqos, m_aruser} = ar_rdata;

    // ── R channel FIFO (m_clk → s_clk) ─────────────────────────────
    // T14: sink stale R beats owed to a read burst forwarded before an
    // s-side-only reset (see axi_bridge_stale_sink.v header + the
    // module-level comment above).  r_complete_c is gated on the FINAL
    // m_rready (post-sink) below and on m_rlast, since a burst is only
    // "complete" (its debt repaid) on its last beat.
    wire r_ar_fwd_c   = m_arvalid_i && m_arready;
    wire r_complete_c = m_rvalid && m_rlast && m_rready;
    wire r_sinking;

    axi_bridge_stale_sink #(.OUTSTANDING_W(4)) u_r_stale_sink (
        .m_clk(m_clk), .m_rst(m_rst),
        .reset_event(s_reset_recovery_event_m),
        .fwd_event(r_ar_fwd_c),
        .complete_event(r_complete_c),
        .sinking(r_sinking)
    );

    wire [R_W-1:0] r_wdata = {m_rid, m_rdata, m_rresp, m_rlast, m_ruser};
    wire [R_W-1:0] r_rdata;
    wire           r_full, r_empty;

    async_fifo #(.WIDTH(R_W), .DEPTH_LOG2(R_DEPTH_LOG2)) r_fifo (
        .wclk(m_clk), .wrst(m_rst),
        .wr_en(m_rvalid && !r_full && !r_sinking), .wr_data(r_wdata),
        .wr_full(r_full), .wr_almost_full(),
        .wr_reset_engaged(r_m_reset_engaged),
        .rclk(s_clk), .rrst(s_rst),
        .rd_en(s_rvalid_i && s_rready),
        .rd_data(r_rdata),
        .rd_empty(r_empty), .rd_almost_empty(), .rd_reset_engaged()
    );
    // While sinking, always accept (and discard) the slave's stale R
    // beats -- no reason to backpressure it, we're not storing them.
    assign m_rready = r_sinking ? 1'b1 : !r_full;
    wire s_rvalid_i = !r_empty;
    assign s_rvalid = s_rvalid_i;
    assign {s_rid, s_rdata, s_rresp, s_rlast, s_ruser} = r_rdata;

endmodule

`default_nettype wire
