// Direct DVI-compatible TMDS output for the TQTT board's existing scanout.
module tqtt_tmds_video (
    input wire pclk,
    input wire pclk_x5,
    input wire reset,
    input wire [23:0] rgb,
    input wire de, hs, vs,
    output wire tmds_clk_p, tmds_clk_n,
    output wire [2:0] tmds_p, tmds_n
);
    wire [9:0] symbols [2:0];
    wire [2:0] serial_data;
    wire serial_clock;
    for (genvar i = 0; i < 3; i++) begin : lanes
        tmds_channel #(.CN(i)) encoder (
            .clk_pixel(pclk), .video_data(rgb[i*8 +: 8]),
            .data_island_data(4'd0),
            .control_data(i == 0 ? {vs, hs} : 2'b00),
            .mode(de ? 3'd1 : 3'd0), .tmds(symbols[i])
        );
        OBUFDS data_output (.I(serial_data[i]), .O(tmds_p[i]), .OB(tmds_n[i]));
    end
    serializer #(.NUM_CHANNELS(3), .VIDEO_RATE(74.25e6)) u_serializer (
        .clk_pixel(pclk), .clk_pixel_x5(pclk_x5), .reset(reset),
        .tmds_internal(symbols), .tmds(serial_data), .tmds_clock(serial_clock)
    );
    OBUFDS clock_output (.I(serial_clock), .O(tmds_clk_p), .OB(tmds_clk_n));
endmodule
