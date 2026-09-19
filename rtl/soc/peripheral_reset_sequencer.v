// peripheral_reset_sequencer.v - turn a 68040 RESET indication into a
// storage-safe warm reset of the SoC peripheral island.

`default_nettype none

module peripheral_reset_sequencer #(
    parameter integer PULSE_CYCLES = 128,
    parameter integer COUNT_W = (PULSE_CYCLES <= 1) ? 1 : $clog2(PULSE_CYCLES)
) (
    input  wire clk,
    input  wire rst,
    input  wire reset_req,
    input  wire storage_busy,
    output wire storage_reset_req,
    output reg  peripheral_reset
);
    reg [COUNT_W-1:0] count_q;
    reg               pending_q;
    reg               request_armed_q;

    wire new_request = reset_req && request_armed_q;
    wire start_reset = !peripheral_reset && !storage_busy &&
                       (pending_q || new_request);

    // Assert this immediately and retain it while waiting for storage to
    // close.  sd_ctrl consumes the level by completing any open CMD24/CMD25
    // session before honoring reset.
    assign storage_reset_req = reset_req || pending_q || peripheral_reset;

    always @(posedge clk) begin
        if (rst) begin
            count_q          <= {COUNT_W{1'b0}};
            pending_q        <= 1'b0;
            request_armed_q  <= 1'b1;
            peripheral_reset <= 1'b0;
        end else begin
            if (!reset_req)
                request_armed_q <= 1'b1;

            if (new_request) begin
                request_armed_q <= 1'b0;
                pending_q       <= 1'b1;
            end

            if (start_reset) begin
                pending_q        <= 1'b0;
                peripheral_reset <= 1'b1;
                count_q          <= PULSE_CYCLES - 1;
            end else if (peripheral_reset) begin
                if (count_q == {COUNT_W{1'b0}}) begin
                    peripheral_reset <= 1'b0;
                end else begin
                    count_q <= count_q - {{(COUNT_W-1){1'b0}}, 1'b1};
                end
            end
        end
    end

endmodule

`default_nettype wire
