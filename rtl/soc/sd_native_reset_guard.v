// Hold a native SD controller reset until its current on-card command has
// completed. A request is latched, so a short debug reset pulse is not lost
// while waiting. This does not repair a card that itself timed out; a card
// power cycle may still be required after such an error.
`default_nettype none
module sd_native_reset_guard (
    input  wire clk,
    input  wire reset_req,
    input  wire card_session_active,
    output wire rst
);
    reg pending = 1'b1;
    reg [2:0] reset_pipe = 3'b111;
    always @(posedge clk) begin
        if (reset_req) pending <= 1'b1;
        if (pending && !card_session_active) begin
            reset_pipe <= 3'b111;
            if (!reset_req) pending <= 1'b0;
        end else if (reset_pipe != 3'b000) begin
            reset_pipe <= {reset_pipe[1:0], 1'b0};
        end
    end
    assign rst = reset_pipe[2];
endmodule
`default_nettype wire
