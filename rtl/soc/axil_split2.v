// axil_split2.v — two-way address demux for one AXI-Lite debug aperture.
//
// Why this exists
//   The JTAG-AXI debug fabric hands out one 1 MiB slave window per xbar
//   port, and adding a seventh xbar slave means touching axi_xbar.v's
//   flattened s0..s5 port arrays everywhere.  The AXI_SD_JTAG_BASE window
//   (0x50A0_0000) already holds exactly one 32-byte register file
//   (sd_jtag_writer), so there is a megabyte of address space going spare
//   behind it.  This module splits that window in two on a single address
//   bit so a second small register file (pram_sd) can live there without
//   reshaping the crossbar.
//
//   Default SEL_BIT = 8:
//       0x50A0_0000 .. 0x50A0_00FF  → d0   (sd_jtag_writer, unchanged)
//       0x50A0_0100 .. 0x50A0_01FF  → d1   (pram_sd)
//   and the pattern repeats every 512 bytes across the window, which is
//   harmless — both downstream register files already alias.
//
// Behaviour
//   Store-and-forward, ONE outstanding write and ONE outstanding read.
//   That is deliberate: the only master here is the JTAG-AXI REPL, which
//   is strictly sequential, and single-outstanding makes the routing
//   latch trivially correct (there is never a second transaction whose
//   target could disagree with the one in flight).
//
//   AW and W are buffered independently, so a master that presents W
//   before AW (legal in AXI-Lite) still works; the transaction is only
//   launched once both have landed and the target is therefore known.
//
// Bounded response
//   This module adds no unbounded wait of its own — it is a pure relay
//   and both downstream slaves are register files that always answer.
//   The backstop for a slave that somehow does not is the xbar's own
//   per-transaction watchdog upstream of this module (see axi_xbar.v).
//
// Verilog-2005, synchronous active-high reset.

`default_nettype none

module axil_split2 #(
    parameter integer ADDR_W  = 20,
    // Upstream address bit that selects d1.  Must sit above whatever the
    // downstream register files decode (both use addr[5:0]).
    parameter integer SEL_BIT = 8
) (
    input  wire                clk,
    input  wire                rst,

    // ── Upstream (slave face) ─────────────────────────────────────────
    input  wire [ADDR_W-1:0]   u_awaddr,
    input  wire                u_awvalid,
    output wire                u_awready,
    input  wire [31:0]         u_wdata,
    input  wire [3:0]          u_wstrb,
    input  wire                u_wvalid,
    output wire                u_wready,
    output wire [1:0]          u_bresp,
    output wire                u_bvalid,
    input  wire                u_bready,
    input  wire [ADDR_W-1:0]   u_araddr,
    input  wire                u_arvalid,
    output wire                u_arready,
    output wire [31:0]         u_rdata,
    output wire [1:0]          u_rresp,
    output wire                u_rvalid,
    input  wire                u_rready,

    // ── Downstream 0 (addr[SEL_BIT] == 0) ─────────────────────────────
    output wire [ADDR_W-1:0]   d0_awaddr,
    output wire                d0_awvalid,
    input  wire                d0_awready,
    output wire [31:0]         d0_wdata,
    output wire [3:0]          d0_wstrb,
    output wire                d0_wvalid,
    input  wire                d0_wready,
    input  wire [1:0]          d0_bresp,
    input  wire                d0_bvalid,
    output wire                d0_bready,
    output wire [ADDR_W-1:0]   d0_araddr,
    output wire                d0_arvalid,
    input  wire                d0_arready,
    input  wire [31:0]         d0_rdata,
    input  wire [1:0]          d0_rresp,
    input  wire                d0_rvalid,
    output wire                d0_rready,

    // ── Downstream 1 (addr[SEL_BIT] == 1) ─────────────────────────────
    output wire [ADDR_W-1:0]   d1_awaddr,
    output wire                d1_awvalid,
    input  wire                d1_awready,
    output wire [31:0]         d1_wdata,
    output wire [3:0]          d1_wstrb,
    output wire                d1_wvalid,
    input  wire                d1_wready,
    input  wire [1:0]          d1_bresp,
    input  wire                d1_bvalid,
    output wire                d1_bready,
    output wire [ADDR_W-1:0]   d1_araddr,
    output wire                d1_arvalid,
    input  wire                d1_arready,
    input  wire [31:0]         d1_rdata,
    input  wire [1:0]          d1_rresp,
    input  wire                d1_rvalid,
    output wire                d1_rready
);

    // ── Write path ────────────────────────────────────────────────────
    localparam [1:0] W_IDLE = 2'd0, W_SEND = 2'd1, W_RESP = 2'd2, W_ACK = 2'd3;

    reg [1:0]        wstate;
    reg [ADDR_W-1:0] awaddr_q;
    reg              aw_have_q;
    reg [31:0]       wdata_q;
    reg [3:0]        wstrb_q;
    reg              w_have_q;
    reg              wsel_q;
    reg              aw_sent_q;
    reg              w_sent_q;
    reg [1:0]        bresp_q;

    assign u_awready = (wstate == W_IDLE) && !aw_have_q;
    assign u_wready  = (wstate == W_IDLE) && !w_have_q;

    wire aw_fire = (wstate == W_SEND) && !aw_sent_q;
    wire w_fire  = (wstate == W_SEND) && !w_sent_q;

    assign d0_awaddr  = awaddr_q;
    assign d1_awaddr  = awaddr_q;
    assign d0_wdata   = wdata_q;
    assign d1_wdata   = wdata_q;
    assign d0_wstrb   = wstrb_q;
    assign d1_wstrb   = wstrb_q;
    assign d0_awvalid = aw_fire && !wsel_q;
    assign d1_awvalid = aw_fire &&  wsel_q;
    assign d0_wvalid  = w_fire  && !wsel_q;
    assign d1_wvalid  = w_fire  &&  wsel_q;
    assign d0_bready  = (wstate == W_RESP) && !wsel_q;
    assign d1_bready  = (wstate == W_RESP) &&  wsel_q;

    assign u_bvalid   = (wstate == W_ACK);
    assign u_bresp    = bresp_q;

    always @(posedge clk) begin
        if (rst) begin
            wstate    <= W_IDLE;
            awaddr_q  <= {ADDR_W{1'b0}};
            aw_have_q <= 1'b0;
            wdata_q   <= 32'd0;
            wstrb_q   <= 4'd0;
            w_have_q  <= 1'b0;
            wsel_q    <= 1'b0;
            aw_sent_q <= 1'b0;
            w_sent_q  <= 1'b0;
            bresp_q   <= 2'b00;
        end else begin
            case (wstate)
                W_IDLE: begin
                    if (u_awvalid && u_awready) begin
                        awaddr_q  <= u_awaddr;
                        aw_have_q <= 1'b1;
                    end
                    if (u_wvalid && u_wready) begin
                        wdata_q  <= u_wdata;
                        wstrb_q  <= u_wstrb;
                        w_have_q <= 1'b1;
                    end
                    // Launch once BOTH halves have landed — only then is
                    // the routing decision unambiguous.
                    if ((aw_have_q || (u_awvalid && u_awready)) &&
                        (w_have_q  || (u_wvalid  && u_wready ))) begin
                        wsel_q    <= aw_have_q ? awaddr_q[SEL_BIT]
                                               : u_awaddr[SEL_BIT];
                        aw_sent_q <= 1'b0;
                        w_sent_q  <= 1'b0;
                        wstate    <= W_SEND;
                    end
                end
                W_SEND: begin
                    if (aw_fire && (wsel_q ? d1_awready : d0_awready))
                        aw_sent_q <= 1'b1;
                    if (w_fire && (wsel_q ? d1_wready : d0_wready))
                        w_sent_q <= 1'b1;
                    if ((aw_sent_q || (aw_fire && (wsel_q ? d1_awready : d0_awready))) &&
                        (w_sent_q  || (w_fire  && (wsel_q ? d1_wready  : d0_wready )))) begin
                        aw_have_q <= 1'b0;
                        w_have_q  <= 1'b0;
                        wstate    <= W_RESP;
                    end
                end
                W_RESP: begin
                    if (wsel_q ? d1_bvalid : d0_bvalid) begin
                        bresp_q <= wsel_q ? d1_bresp : d0_bresp;
                        wstate  <= W_ACK;
                    end
                end
                W_ACK: begin
                    if (u_bready) wstate <= W_IDLE;
                end
                default: wstate <= W_IDLE;
            endcase
        end
    end

    // ── Read path ─────────────────────────────────────────────────────
    localparam [1:0] R_IDLE = 2'd0, R_SEND = 2'd1, R_WAIT = 2'd2, R_ACK = 2'd3;

    reg [1:0]        rstate;
    reg [ADDR_W-1:0] araddr_q;
    reg              rsel_q;
    reg [31:0]       rdata_q;
    reg [1:0]        rresp_q;

    assign u_arready = (rstate == R_IDLE);

    assign d0_araddr  = araddr_q;
    assign d1_araddr  = araddr_q;
    assign d0_arvalid = (rstate == R_SEND) && !rsel_q;
    assign d1_arvalid = (rstate == R_SEND) &&  rsel_q;
    assign d0_rready  = (rstate == R_WAIT) && !rsel_q;
    assign d1_rready  = (rstate == R_WAIT) &&  rsel_q;

    assign u_rvalid = (rstate == R_ACK);
    assign u_rdata  = rdata_q;
    assign u_rresp  = rresp_q;

    always @(posedge clk) begin
        if (rst) begin
            rstate   <= R_IDLE;
            araddr_q <= {ADDR_W{1'b0}};
            rsel_q   <= 1'b0;
            rdata_q  <= 32'd0;
            rresp_q  <= 2'b00;
        end else begin
            case (rstate)
                R_IDLE: begin
                    if (u_arvalid && u_arready) begin
                        araddr_q <= u_araddr;
                        rsel_q   <= u_araddr[SEL_BIT];
                        rstate   <= R_SEND;
                    end
                end
                R_SEND: begin
                    if (rsel_q ? d1_arready : d0_arready) rstate <= R_WAIT;
                end
                R_WAIT: begin
                    if (rsel_q ? d1_rvalid : d0_rvalid) begin
                        rdata_q <= rsel_q ? d1_rdata : d0_rdata;
                        rresp_q <= rsel_q ? d1_rresp : d0_rresp;
                        rstate  <= R_ACK;
                    end
                end
                R_ACK: begin
                    if (u_rready) rstate <= R_IDLE;
                end
                default: rstate <= R_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
