// axil_null_slave.v — AXI4-Lite terminator for a reserved address window
//
// Purpose
//   Keeps an address hole in the fabric *terminated* when the peripheral
//   that used to sit behind it is gone.  Accepts every write and answers
//   BRESP=OKAY; accepts every read and answers RDATA=0 / RRESP=OKAY.  The
//   point is that a stray access completes instead of parking a burst on
//   the interconnect forever — this SoC has a documented history of wild
//   accesses landing in the 0x5xxx_xxxx I/O region
//   (docs/diag-bus-fault-51001c00.md), and an unterminated slave port
//   turns one of those into an xbar wedge.
//
//   Used for the xbar S2 "DMA config" window (rtl/soc/fpga_top_dma.vh)
//   after dma_ctrl was removed: the window, the slot numbering and every
//   master's view of the address map stay exactly as they were.
//
// Key interfaces
//   Standard AXI4-Lite slave.  Address/strobe inputs are accepted and
//   discarded; ADDR_WIDTH exists only so the port list matches the
//   upstream bridge without width warnings.
//
//   AW and W are tracked INDEPENDENTLY.  rtl/soc/axi_wide_to_axilite.v
//   drives them from separate `wr_aw_done` / `wr_w_done` flags and can
//   present them in either order or in the same cycle, so a terminator
//   that required both handshakes together would deadlock it.
//
//   No combinational path from any *valid to any *ready: readiness is a
//   function of internal state only, so this cannot form a handshake
//   loop with an upstream bridge.
//
// Latency
//   One outstanding write and one outstanding read at a time.  BVALID
//   rises the cycle after the second of (AW, W) lands; RVALID rises the
//   cycle after AR lands.  Both hold until their *READY.
//
// Reset
//   Synchronous, active-high.  Clears any parked response so a
//   debug/cold reset cannot leave a dangling BVALID/RVALID.

module axil_null_slave #(
    parameter ADDR_WIDTH = 20,
    parameter DATA_WIDTH = 32
) (
    input  wire                     clk,
    input  wire                     rst,

    input  wire [ADDR_WIDTH-1:0]    awaddr,
    input  wire                     awvalid,
    output wire                     awready,

    input  wire [DATA_WIDTH-1:0]    wdata,
    input  wire [DATA_WIDTH/8-1:0]  wstrb,
    input  wire                     wvalid,
    output wire                     wready,

    output wire [1:0]               bresp,
    output wire                     bvalid,
    input  wire                     bready,

    input  wire [ADDR_WIDTH-1:0]    araddr,
    input  wire                     arvalid,
    output wire                     arready,

    output wire [DATA_WIDTH-1:0]    rdata,
    output wire [1:0]               rresp,
    output wire                     rvalid,
    input  wire                     rready
);

    reg aw_seen;
    reg w_seen;
    reg bvalid_r;
    reg rvalid_r;

    // Ready only while this half of the write has not been captured and
    // no response is still parked — one write in flight, no back-to-back
    // overlap to reason about.
    assign awready = !aw_seen  && !bvalid_r;
    assign wready  = !w_seen   && !bvalid_r;
    assign bvalid  = bvalid_r;
    assign bresp   = 2'b00;                       // OKAY

    assign arready = !rvalid_r;
    assign rvalid  = rvalid_r;
    assign rdata   = {DATA_WIDTH{1'b0}};
    assign rresp   = 2'b00;                       // OKAY

    always @(posedge clk) begin
        if (rst) begin
            aw_seen  <= 1'b0;
            w_seen   <= 1'b0;
            bvalid_r <= 1'b0;
            rvalid_r <= 1'b0;
        end else begin
            // Capture each write half as it lands.
            if (awvalid && awready) aw_seen <= 1'b1;
            if (wvalid  && wready ) w_seen  <= 1'b1;
            // Once both halves are in — this cycle or earlier — park B.
            // These assignments come LAST so their clears of aw_seen /
            // w_seen win over the captures above when both land together.
            if (!bvalid_r) begin
                if ((aw_seen || (awvalid && awready)) &&
                    (w_seen  || (wvalid  && wready ))) begin
                    bvalid_r <= 1'b1;
                    aw_seen  <= 1'b0;
                    w_seen   <= 1'b0;
                end
            end else if (bready) begin
                bvalid_r <= 1'b0;
            end
            // Reads: single beat, zero data.
            if (!rvalid_r) begin
                if (arvalid && arready) rvalid_r <= 1'b1;
            end else if (rready) begin
                rvalid_r <= 1'b0;
            end
        end
    end

    // Address / data inputs are intentionally discarded.
    // verilator lint_off UNUSED
    wire _unused = &{1'b0, awaddr, wdata, wstrb, araddr};
    // verilator lint_on UNUSED

endmodule
