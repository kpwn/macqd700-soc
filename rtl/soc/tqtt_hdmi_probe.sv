// Standalone 640x480p60 TQTT HDMI test pattern and optional 48 kHz PCM tone.
module tqtt_hdmi_probe #(
    parameter int HDMI_AUDIO = 0
) (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    output wire       hdmi_out_en,
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_p,
    output wire [2:0] tmds_n
);
    wire sys_clk_unbuf, sys_clk;
    IBUFDS u_sys_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_clk_unbuf));
    BUFG u_sys_bufg (.I(sys_clk_unbuf), .O(sys_clk));

    // 100 MHz / 5 * 63 = 1260 MHz VCO; 126 and 25.2 MHz outputs.
    wire pixel_x5_unbuf, pixel_unbuf, pixel_x5, pixel_clk, clkfb, locked;
    MMCME4_BASE #(
        .CLKIN1_PERIOD(10.0), .DIVCLK_DIVIDE(5), .CLKFBOUT_MULT_F(63.0),
        .CLKOUT0_DIVIDE_F(10.0), .CLKOUT1_DIVIDE(50),
        // Both pixel-clock edges fall between 126 MHz sample edges.
        .CLKOUT1_PHASE(36.0), .CLKOUT1_DUTY_CYCLE(0.4),
        .BANDWIDTH("OPTIMIZED"), .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKIN1(sys_clk), .CLKFBIN(clkfb), .CLKFBOUT(clkfb), .CLKFBOUTB(),
        .CLKOUT0(pixel_x5_unbuf), .CLKOUT0B(),
        .CLKOUT1(pixel_unbuf), .CLKOUT1B(),
        .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(),
        .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .RST(1'b0), .PWRDWN(1'b0), .LOCKED(locked)
    );
    BUFG u_x5_bufg (.I(pixel_x5_unbuf), .O(pixel_x5));
    BUFG u_pixel_bufg (.I(pixel_unbuf), .O(pixel_clk));
    wire reset = !locked;

    // 126 MHz / 2625 = 48 kHz exactly on average. Alternate half-cycle
    // lengths of 1312 and 1313 input clocks to keep the audio duty balanced.
    reg [10:0] audio_half_count = 0;
    reg audio_clk_div = 0;
    wire audio_clk;
    BUFG u_audio_bufg (.I(audio_clk_div), .O(audio_clk));
    always @(posedge pixel_x5) begin
        if (reset) begin
            audio_half_count <= 0;
            audio_clk_div <= 0;
        end else if (audio_half_count == (audio_clk_div ? 11'd1312 : 11'd1311)) begin
            audio_half_count <= 0;
            audio_clk_div <= !audio_clk_div;
        end else begin
            audio_half_count <= audio_half_count + 1'b1;
        end
    end

    // A quiet 1 kHz stereo square wave is easy to recognize in a captured
    // WAV. It is a source test, not yet the Mac ASC stream.
    reg [5:0] tone_phase = 0;
    logic [15:0] audio_sample_word [1:0];
    always @(posedge audio_clk) begin
        if (reset) begin
            tone_phase <= 0;
            audio_sample_word[0] <= 0;
            audio_sample_word[1] <= 0;
        end else begin
            tone_phase <= tone_phase == 6'd47 ? 6'd0 : tone_phase + 1'b1;
            audio_sample_word[0] <= tone_phase < 6'd24 ? 16'd4096 : -16'sd4096;
            audio_sample_word[1] <= tone_phase < 6'd24 ? 16'd4096 : -16'sd4096;
        end
    end

    wire [9:0] cx;
    wire [9:0] cy;
    reg [23:0] rgb = 24'd0;
    always @(posedge pixel_clk) begin
        if (reset)
            rgb <= 24'd0;
        else if (cx < 10'd80)  rgb <= 24'hff0000;
        else if (cx < 10'd160) rgb <= 24'h00ff00;
        else if (cx < 10'd240) rgb <= 24'h0000ff;
        else if (cx < 10'd320) rgb <= 24'hffffff;
        else if (cx < 10'd400) rgb <= 24'h00ffff;
        else if (cx < 10'd480) rgb <= 24'hff00ff;
        else if (cx < 10'd560) rgb <= 24'hffff00;
        else                    rgb <= 24'h000000;
    end

    wire [2:0] tmds_data;
    wire tmds_clock;
    hdmi #(
        .VIDEO_ID_CODE(1), .VIDEO_REFRESH_RATE(60.0),
        .AUDIO_RATE(48000), .AUDIO_BIT_WIDTH(16),
        .DVI_OUTPUT(HDMI_AUDIO == 0)
    ) u_hdmi (
        .clk_pixel_x5(pixel_x5), .clk_pixel(pixel_clk),
        .clk_audio(audio_clk), .reset(reset), .rgb(rgb),
        .audio_sample_word(audio_sample_word),
        .tmds(tmds_data), .tmds_clock(tmds_clock),
        .cx(cx), .cy(cy),
        .frame_width(), .frame_height(), .screen_width(), .screen_height()
    );

    assign hdmi_out_en = 1'b1;
    OBUFDS u_clk_obuf (.I(tmds_clock), .O(tmds_clk_p), .OB(tmds_clk_n));
    for (genvar i = 0; i < 3; i++) begin : g_out
        OBUFDS u_data_obuf (.I(tmds_data[i]), .O(tmds_p[i]), .OB(tmds_n[i]));
    end
endmodule
