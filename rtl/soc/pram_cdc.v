// pram_cdc.v — single-byte clock-domain crossing for the RTC PRAM array.
//
// Purpose
//   rtl/mac/rtc.v (and therefore the 256-byte PRAM array) runs on pb_clk
//   (50 MHz).  pram_sd.v, which persists that array to the SD card, has
//   to live on core_clk (100 MHz) because that is where sd_spi / sd_ctrl
//   run.  This module is the ONLY crossing between the two, and it is a
//   textbook 4-phase MCP (multi-cycle-path) handshake: the payload wires
//   are held stable by the requester for the whole transaction and are
//   never sampled by the destination until a synchronised request edge
//   says they have been stable for >= 2 destination clocks.
//
//   One byte per handshake.  A full 256-byte PRAM snapshot or restore
//   therefore costs a few thousand core_clk cycles, which is irrelevant
//   for a manual JTAG operation and buys a CDC that is trivially
//   reviewable instead of an async FIFO.
//
// Contract (A side = core_clk requester)
//   * Drive a_addr / a_we / a_wdata, THEN raise a_req and hold it.
//   * Wait for a_ack to rise.  a_rdata is valid on and after that edge
//     (it is the PRAM byte at a_addr, captured on the B side).
//   * Drop a_req.  Wait for a_ack to fall before starting the next byte.
//   * a_req/a_ack are LEVELS, not pulses.  The caller is responsible for
//     bounding its own wait — this module has no watchdog of its own
//     because it has no way to make progress on a dead destination clock;
//     see pram_sd.v's CDC_WAIT_LOG2 timeout, which is that bound.
//
// Contract (B side = pb_clk, wired straight at rtc.v)
//   * b_addr / b_wdata are combinational passthroughs of the A-side
//     registers.  They are stable for the entire transaction — declare
//     them as false paths / max-delay if you ever constrain this.
//   * b_we is a genuine one-clock pulse in the B domain.
//   * b_rdata must be a combinational read of pram[b_addr].
//
// Verilog-2005, synchronous active-high resets (one per domain).

`default_nettype none

module pram_cdc (
    // ── A side: core_clk requester ────────────────────────────────────
    input  wire        a_clk,
    input  wire        a_rst,
    input  wire        a_req,
    input  wire        a_we,
    input  wire [7:0]  a_addr,
    input  wire [7:0]  a_wdata,
    output reg  [7:0]  a_rdata,
    output wire        a_ack,

    // ── B side: pb_clk / rtc.v PRAM array ─────────────────────────────
    input  wire        b_clk,
    input  wire        b_rst,
    output wire [7:0]  b_addr,
    output wire [7:0]  b_wdata,
    output reg         b_we,
    input  wire [7:0]  b_rdata
);

    // Payload is A-side-held for the whole transaction: pure passthrough.
    assign b_addr  = a_addr;
    assign b_wdata = a_wdata;

    // ── A → B: synchronise the request level ──────────────────────────
    (* ASYNC_REG = "TRUE" *) reg req_b_meta;
    (* ASYNC_REG = "TRUE" *) reg req_b_sync;
    reg                          req_b_q;
    reg                          ack_b;
    reg  [7:0]                   rdata_b;

    wire req_b_rise = req_b_sync && !req_b_q;
    wire req_b_fall = !req_b_sync && req_b_q;

    always @(posedge b_clk) begin
        if (b_rst) begin
            req_b_meta <= 1'b0;
            req_b_sync <= 1'b0;
            req_b_q    <= 1'b0;
            ack_b      <= 1'b0;
            rdata_b    <= 8'h00;
            b_we       <= 1'b0;
        end else begin
            req_b_meta <= a_req;
            req_b_sync <= req_b_meta;
            req_b_q    <= req_b_sync;

            // One-clock write strobe, exactly on the synchronised rising
            // edge of the request.  a_addr/a_wdata have been stable for
            // >= 2 b_clk edges by construction.
            b_we <= req_b_rise && a_we;

            if (req_b_rise) begin
                // Read capture happens on the same edge for both read and
                // write requests: b_rdata is the PRE-write value, which is
                // exactly what a read-modify-write caller would want and is
                // simply ignored by a writer.
                rdata_b <= b_rdata;
                ack_b   <= 1'b1;
            end else if (req_b_fall) begin
                ack_b   <= 1'b0;
            end
        end
    end

    // ── B → A: synchronise the acknowledge level ──────────────────────
    //
    // THREE flops, not two, and `a_ack` is the third — deliberately.
    //
    // rdata_b is captured into a_rdata on any edge where the 2-FF-synced
    // ack is high, and a_ack is that same signal delayed one further
    // cycle.  So by the time the requester observes a_ack, a_rdata has
    // ALREADY been updated: "a_rdata is valid on and after the a_ack
    // rising edge" is literally true.
    //
    // The obvious two-flop version (a_ack = ack_a_sync, capture on its
    // rising edge) is off by exactly one cycle: the capture happens on
    // the same edge that raises a_ack, so a requester sampling on that
    // cycle gets the PREVIOUS transaction's byte.  That produced a PRAM
    // snapshot shifted by one byte — which still checksums
    // self-consistently, so a save/load round trip using only this port
    // would have looked perfectly healthy while corrupting every byte.
    // Caught by tb_pram_sd.cpp's cross-path comparison against rtc.v's
    // own serial protocol; do not "simplify" this back to two flops.
    (* ASYNC_REG = "TRUE" *) reg ack_a_meta;
    (* ASYNC_REG = "TRUE" *) reg ack_a_sync;
    reg                          ack_a_q;

    assign a_ack = ack_a_q;

    always @(posedge a_clk) begin
        if (a_rst) begin
            ack_a_meta <= 1'b0;
            ack_a_sync <= 1'b0;
            ack_a_q    <= 1'b0;
            a_rdata    <= 8'h00;
        end else begin
            ack_a_meta <= ack_b;
            ack_a_sync <= ack_a_meta;
            ack_a_q    <= ack_a_sync;
            // rdata_b is held by the B side for as long as ack_b is high,
            // and ack_a_sync trails ack_b by 2 a_clk edges, so the value
            // sampled here has been stable for at least that long.
            if (ack_a_sync) a_rdata <= rdata_b;
        end
    end

endmodule

`default_nettype wire
