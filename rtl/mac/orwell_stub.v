// orwell_stub.v - Probe-safe Quadra 700 Orwell controls stub.
//
// The Q700 ROM probes the Orwell controls page at 0x5000_E000.  The
// real controls are not implemented yet; this owned sink keeps the RTL
// platform behavior aligned with the ROM harness by acknowledging the
// narrow reset-value window, returning zero, and swallowing writes.

`default_nettype none

module orwell_stub (
    input  wire       cs,
    input  wire       rd,
    input  wire       wr,
    input  wire [7:0] addr,
    input  wire [7:0] wdata,
    output wire [7:0] rdata,
    output wire       ack
);

    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0] _unused_addr = addr;
    wire [7:0] _unused_wdata = wdata;
    /* verilator lint_on UNUSEDSIGNAL */

    assign rdata = (cs && rd) ? 8'h00 : 8'hFF;
    assign ack   = cs && (rd || wr);

endmodule

`default_nettype wire
