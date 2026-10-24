// Optional TQTT HDMI transmitter.  The existing scanout owns the 720p raster;
// this module inserts HDMI audio packets into its horizontal blanking period.
module tqtt_hdmi_audio_video (
    input  wire        pclk,
    input  wire        pclk_x5,
    input  wire        video_locked,
    input  wire [23:0] rgb,
    input  wire        de, hs, vs,
    input  wire [11:0] x,
    input  wire [10:0] y,
    input  wire        pb_clk,
    input  wire        pb_rst,
    input  wire        sample_valid,
    input  wire [15:0] sample_l, sample_r,
    output wire        tmds_clk_p, tmds_clk_n,
    output wire [2:0]  tmds_p, tmds_n
);
    // Exact average 48 kHz from 50 MHz: five 521-cycle half periods and
    // one 520-cycle half period.  The resulting 20 ns jitter is well below
    // one audio sample period.  This clock only feeds the audio packet side.
    reg [9:0] audio_half_count = 0;
    reg [2:0] audio_fraction = 0;
    reg audio_clk_div = 0;
    wire audio_clk;
    (* ASYNC_REG = "TRUE" *) reg [1:0] video_locked_pb = 0;
    always @(posedge pb_clk) begin
        if (pb_rst) video_locked_pb <= 0;
        else        video_locked_pb <= {video_locked_pb[0], video_locked};
    end
    wire audio_rst = pb_rst | !video_locked_pb[1];
    BUFG u_audio_bufg (.I(audio_clk_div), .O(audio_clk));
    always @(posedge pb_clk) begin
        if (audio_rst) begin
            audio_half_count <= 0;
            audio_fraction <= 0;
            audio_clk_div <= 0;
        end else if (audio_half_count == (audio_fraction == 3'd5 ? 10'd519 : 10'd520)) begin
            audio_half_count <= 0;
            audio_fraction <= audio_fraction == 3'd5 ? 3'd0 : audio_fraction + 1'b1;
            audio_clk_div <= !audio_clk_div;
        end else begin
            audio_half_count <= audio_half_count + 1'b1;
        end
    end

    wire [31:0] fifo_data;
    wire fifo_full, fifo_empty;
    async_fifo #(.WIDTH(32), .DEPTH_LOG2(4)) u_audio_fifo (
        .wclk(pb_clk), .wrst(audio_rst),
        .wr_en(sample_valid && !fifo_full), .wr_data({sample_l, sample_r}),
        .wr_full(fifo_full), .wr_almost_full(), .wr_reset_engaged(),
        .rclk(audio_clk), .rrst(audio_rst),
        .rd_en(!fifo_empty), .rd_data(fifo_data),
        .rd_empty(fifo_empty), .rd_almost_empty(), .rd_reset_engaged()
    );
    logic [15:0] audio_sample_word [1:0];
    reg [12:0] silence_count = 0;
    always @(posedge audio_clk) begin
        if (audio_rst) begin
            audio_sample_word[0] <= 0;
            audio_sample_word[1] <= 0;
            silence_count <= 0;
        end else if (!fifo_empty) begin
            audio_sample_word[0] <= fifo_data[31:16];
            audio_sample_word[1] <= fifo_data[15:0];
            silence_count <= 0;
        end else if (silence_count == 13'd4799) begin
            // A stopped ASC stream must not hold a nonzero sample forever.
            audio_sample_word[0] <= 0;
            audio_sample_word[1] <= 0;
        end else begin
            silence_count <= silence_count + 1'b1;
        end
    end

    localparam int SCREEN_W = 1280;
    localparam int SCREEN_H = 720;
    localparam int FRAME_W = 1650;
    localparam int FRAME_H = 750;
    localparam int PACKETS_PER_LINE = 10;
    wire di_now = x >= SCREEN_W + 14 && x < SCREEN_W + 14 + PACKETS_PER_LINE * 32;
    wire packet_enable = di_now && (5'(x + SCREEN_W + 18) == 5'd0);
    wire video_field_end = x == SCREEN_W - 1 && y == SCREEN_H - 1;
    wire vg_now = x >= FRAME_W - 2 && x < FRAME_W &&
                  (y == FRAME_H - 1 || y < SCREEN_H - 1);
    wire vp_now = x >= FRAME_W - 10 && x < FRAME_W - 2 &&
                  (y == FRAME_H - 1 || y < SCREEN_H - 1);
    wire dg_now = (x >= SCREEN_W + 12 && x < SCREEN_W + 14) ||
                  (x >= SCREEN_W + 14 + PACKETS_PER_LINE * 32 &&
                   x < SCREEN_W + 16 + PACKETS_PER_LINE * 32);
    wire dp_now = x >= SCREEN_W + 4 && x < SCREEN_W + 12;

    wire [23:0] packet_header;
    wire [55:0] packet_sub [3:0];
    wire [8:0] packet_data;
    wire [4:0] packet_pixel_counter;
    packet_picker #(
        .VIDEO_ID_CODE(4), .VIDEO_RATE(74.25e6), .IT_CONTENT(1'b1),
        .AUDIO_RATE(48000), .AUDIO_BIT_WIDTH(16),
        .VENDOR_NAME({"macqd", 24'd0}),
        .PRODUCT_DESCRIPTION({"macqd700", 64'd0})
    ) u_packet_picker (
        .clk_pixel(pclk), .clk_audio(audio_clk), .reset(!video_locked),
        .video_field_end(video_field_end), .packet_enable(packet_enable),
        .packet_pixel_counter(packet_pixel_counter),
        .audio_sample_word(audio_sample_word),
        .header(packet_header), .sub(packet_sub)
    );
    reg di_period = 0;
    packet_assembler u_packet_assembler (
        .clk_pixel(pclk), .reset(!video_locked),
        .data_island_period(di_period),
        .header(packet_header), .sub(packet_sub),
        .packet_data(packet_data), .counter(packet_pixel_counter)
    );

    reg video_guard = 0, video_preamble = 0;
    reg data_guard = 0, data_preamble = 0;
    reg [2:0] mode = 0;
    reg [23:0] video_data = 0;
    reg [5:0] control_data = 0;
    reg [11:0] island_data = 0;
    always @(posedge pclk) begin
        if (!video_locked) begin
            video_guard <= 0;
            video_preamble <= 0;
            data_guard <= 0;
            data_preamble <= 0;
            di_period <= 0;
            mode <= 0;
            video_data <= 0;
            control_data <= 0;
            island_data <= 0;
        end else begin
            video_guard <= vg_now;
            video_preamble <= vp_now;
            data_guard <= dg_now;
            data_preamble <= dp_now;
            di_period <= di_now;
            mode <= data_guard ? 3'd4 : di_period ? 3'd3 :
                    video_guard ? 3'd2 : de ? 3'd1 : 3'd0;
            video_data <= rgb;
            control_data <= {{1'b0, data_preamble},
                             {1'b0, video_preamble || data_preamble},
                             vs, hs};
            island_data[11:4] <= packet_data[8:1];
            island_data[3] <= x != 0;
            island_data[2] <= packet_data[0];
            island_data[1:0] <= {vs, hs};
        end
    end

    wire [9:0] symbols [2:0];
    wire [2:0] serial_data;
    wire serial_clock;
    for (genvar i = 0; i < 3; i++) begin : lanes
        tmds_channel #(.CN(i)) encoder (
            .clk_pixel(pclk), .video_data(video_data[i*8 +: 8]),
            .data_island_data(island_data[i*4 +: 4]),
            .control_data(control_data[i*2 +: 2]),
            .mode(mode), .tmds(symbols[i])
        );
        OBUFDS data_output (.I(serial_data[i]), .O(tmds_p[i]), .OB(tmds_n[i]));
    end
    serializer #(.NUM_CHANNELS(3), .VIDEO_RATE(74.25e6)) u_serializer (
        .clk_pixel(pclk), .clk_pixel_x5(pclk_x5), .reset(!video_locked),
        .tmds_internal(symbols), .tmds(serial_data), .tmds_clock(serial_clock)
    );
    OBUFDS clock_output (.I(serial_clock), .O(tmds_clk_p), .OB(tmds_clk_n));
endmodule
