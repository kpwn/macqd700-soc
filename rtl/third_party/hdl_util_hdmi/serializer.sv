// UltraScale+ 10:1 TMDS serializer for hdl-util/hdmi.
// Each of five clk_pixel_x5 cycles transfers two bits through ODDRE1.
module serializer #(
    parameter int NUM_CHANNELS = 3,
    parameter real VIDEO_RATE = 25.2e6
) (
    input logic clk_pixel,
    input logic clk_pixel_x5,
    input logic reset,
    input logic [9:0] tmds_internal [NUM_CHANNELS-1:0],
    output logic [2:0] tmds,
    output logic tmds_clock
);
    // Synchronize the pixel-clock level into the x5 domain. The rising-edge
    // pulse appears two or three x5 cycles after the TMDS encoders update,
    // when all ten symbol bits are stable. This is a control synchronizer;
    // the parallel symbol stays stable for a full pixel period.
    (* ASYNC_REG = "TRUE" *) logic pixel_meta = 1'b0;
    (* ASYNC_REG = "TRUE" *) logic pixel_sync = 1'b0;
    logic pixel_prev = 1'b0;
    wire pixel_start = pixel_sync && !pixel_prev;

    logic [9:0] shift_word [NUM_CHANNELS:0];
    always @(posedge clk_pixel_x5) begin
        if (reset) begin
            pixel_meta <= 1'b0;
            pixel_sync <= 1'b0;
            pixel_prev <= 1'b0;
            for (int i = 0; i <= NUM_CHANNELS; i++)
                shift_word[i] <= 10'b0000011111;
        end else begin
            pixel_meta <= clk_pixel;
            pixel_sync <= pixel_meta;
            pixel_prev <= pixel_sync;
            if (pixel_start) begin
                shift_word[NUM_CHANNELS] <= 10'b0000011111;
                for (int i = 0; i < NUM_CHANNELS; i++)
                    shift_word[i] <= tmds_internal[i];
            end else begin
                for (int i = 0; i <= NUM_CHANNELS; i++)
                    shift_word[i] <= {2'b00, shift_word[i][9:2]};
            end
        end
    end

    for (genvar i = 0; i <= NUM_CHANNELS; i++) begin : g_ddr
        wire serial_out;
        ODDRE1 #(.IS_C_INVERTED(1'b0), .IS_D1_INVERTED(1'b0),
                 .IS_D2_INVERTED(1'b0), .SRVAL(1'b0)) u_ddr (
            .C(clk_pixel_x5), .D1(shift_word[i][0]),
            .D2(shift_word[i][1]), .SR(reset), .Q(serial_out)
        );
        if (i == NUM_CHANNELS)
            assign tmds_clock = serial_out;
        else
            assign tmds[i] = serial_out;
    end
endmodule
