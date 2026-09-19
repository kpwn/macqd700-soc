// Four-phase CDC mailboxes for the SONIC register and packet-engine seam.
// Multi-bit payloads remain stable from valid assertion through acknowledge.
`default_nettype none
module q700_sonic_cdc (
    input wire pb_clk, input wire pb_rst,
    input wire pb_cmd_valid, output wire pb_cmd_ready,
    input wire [15:0] pb_cmd_dcr, pb_cmd_utda, pb_cmd_ctda,
    output wire pb_done_valid, input wire pb_done_ready,
    output wire pb_done_error,
    output wire pb_done_pint,
    output wire [15:0] pb_done_ctda, pb_done_tcr, pb_done_tps, pb_done_tfc,

    input wire core_clk, input wire core_rst,
    output wire core_cmd_valid, input wire core_cmd_ready,
    output wire [15:0] core_cmd_dcr, core_cmd_utda, core_cmd_ctda,
    input wire core_done_valid, output wire core_done_ready,
    input wire core_done_error,
    input wire core_done_pint,
    input wire pb_halt, output wire core_halt,
    input wire [15:0] core_done_ctda, core_done_tcr, core_done_tps, core_done_tfc
);
    (* ASYNC_REG="TRUE" *) reg cmd_v_c1, cmd_v_c2;
    (* ASYNC_REG="TRUE" *) reg cmd_ack_p1, cmd_ack_p2;
    reg cmd_ack_core;
    (* ASYNC_REG="TRUE" *) reg done_v_p1, done_v_p2;
    (* ASYNC_REG="TRUE" *) reg done_ack_c1, done_ack_c2;
    reg done_ack_pb;

    assign core_cmd_valid = cmd_v_c2 && !cmd_ack_core;
    assign pb_cmd_ready = cmd_ack_p2;
    assign core_cmd_dcr = pb_cmd_dcr;
    assign core_cmd_utda = pb_cmd_utda;
    assign core_cmd_ctda = pb_cmd_ctda;
    assign pb_done_valid = done_v_p2 && !done_ack_pb;
    assign core_done_ready = done_ack_c2;
    assign pb_done_error = core_done_error;
    assign pb_done_pint = core_done_pint;
    assign pb_done_ctda = core_done_ctda;
    assign pb_done_tcr = core_done_tcr;
    assign pb_done_tps = core_done_tps;
    assign pb_done_tfc = core_done_tfc;

    always @(posedge core_clk) begin
        if (core_rst) begin
            cmd_v_c1 <= 0; cmd_v_c2 <= 0; cmd_ack_core <= 0;
            done_ack_c1 <= 0; done_ack_c2 <= 0;
        end else begin
            cmd_v_c1 <= pb_cmd_valid; cmd_v_c2 <= cmd_v_c1;
            if (core_cmd_valid && core_cmd_ready) cmd_ack_core <= 1;
            else if (!cmd_v_c2) cmd_ack_core <= 0;
            done_ack_c1 <= done_ack_pb; done_ack_c2 <= done_ack_c1;
        end
    end
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            cmd_ack_p1 <= 0; cmd_ack_p2 <= 0;
            done_v_p1 <= 0; done_v_p2 <= 0; done_ack_pb <= 0;
        end else begin
            cmd_ack_p1 <= cmd_ack_core; cmd_ack_p2 <= cmd_ack_p1;
            done_v_p1 <= core_done_valid; done_v_p2 <= done_v_p1;
            if (pb_done_valid && pb_done_ready) done_ack_pb <= 1;
            else if (!done_v_p2) done_ack_pb <= 0;
        end
    end
    // CR_HTX crossing.  A LEVEL, not an event: the transmit engine samples
    // it once per descriptor, after writing TXpkt.status, so a two-flop
    // synchroniser is sufficient and a missed cycle only delays the halt to
    // the next descriptor boundary -- which is what "halts after the current
    // transmission has completed" (ds:1656-1659) already permits.
    (* ASYNC_REG = "TRUE" *) reg halt_meta, halt_sync;
    always @(posedge core_clk) begin
        if (core_rst) begin halt_meta <= 1'b0; halt_sync <= 1'b0; end
        else          begin halt_meta <= pb_halt; halt_sync <= halt_meta; end
    end
    assign core_halt = halt_sync;

endmodule
`default_nettype wire
