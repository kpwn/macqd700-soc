// Exercise the full Mac's 720p HDMI packetizer without the Mac or DDR.
// This intentionally uses tqtt_hdmi_audio_video, not the standalone probe's
// upstream hdmi top, so a passing pattern validates the integration path.
module tqtt_hdmi_integration_probe #(
    parameter int USE_FABRIC_CLOCK = 0,
    parameter int CONTROL_SKEW_CYCLES = 0
) (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire       fabric_clk_50,
    output wire       hdmi_out_en,
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_p,
    output wire [2:0] tmds_n
);
    wire video_ref_clk, pb_clk, fabric_locked;
    if (USE_FABRIC_CLOCK) begin : fabric_source
        wire fabric_clk, video_unbuf, pb_unbuf, feedback;
        IBUF u_fabric_ibuf (.I(fabric_clk_50), .O(fabric_clk));
        MMCME4_BASE #(
            .BANDWIDTH("OPTIMIZED"), .CLKIN1_PERIOD(20.000),
            .DIVCLK_DIVIDE(1), .CLKFBOUT_MULT_F(20.000),
            .CLKOUT0_DIVIDE_F(10.000), .CLKOUT1_DIVIDE(20),
            .CLKOUT2_DIVIDE(10), .STARTUP_WAIT("FALSE")
        ) u_fabric_mmcm (
            .CLKIN1(fabric_clk), .CLKFBIN(feedback), .CLKFBOUT(feedback),
            .CLKOUT0(), .CLKOUT1(pb_unbuf), .CLKOUT2(video_unbuf),
            .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2B(),
            .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(),
            .CLKOUT6(), .CLKFBOUTB(),
            .LOCKED(fabric_locked), .PWRDWN(1'b0), .RST(1'b0)
        );
        BUFG u_video_bufg (.I(video_unbuf), .O(video_ref_clk));
        BUFG u_pb_bufg (.I(pb_unbuf), .O(pb_clk));
    end else begin : direct_source
        wire sys_clk;
        IBUFDS u_pb_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_clk));
        assign video_ref_clk = sys_clk;
        assign fabric_locked = 1'b1;
        reg pb_div = 0;
        always @(posedge sys_clk) pb_div <= !pb_div;
        BUFG u_pb_bufg (.I(pb_div), .O(pb_clk));
    end
    wire pclk, pclk_fwd, pclk_x5, locked;
    mmcm_hdmi #(
        .CLKIN1_PERIOD(10.000),
        .DIVCLK_DIVIDE(5),
        .CLKFBOUT_MULT_F(74.250),
        .CLKOUT0_DIVIDE_F(20.000),
        .CLKOUT1_DIVIDE_I(20),
        .EXTERNAL_IBUFDS(1)
    ) u_mmcm (
        .clk_in_p(video_ref_clk), .clk_in_n(1'b0), .resetn(fabric_locked),
        .pclk(pclk), .pclk_fwd(pclk_fwd), .pclk_x5(pclk_x5),
        .locked(locked)
    );

    reg [3:0] reset_pipe = 4'hf;
    always @(posedge pclk or negedge locked) begin
        if (!locked) reset_pipe <= 4'hf;
        else         reset_pipe <= {reset_pipe[2:0], 1'b0};
    end
    wire reset = reset_pipe[3];
    wire [11:0] x;
    wire [10:0] y;
    wire de, hs, vs;
    vtg #(
        .H_ACTIVE(1280), .H_FP(110), .H_SYNC(40), .H_BP(220),
        .V_ACTIVE(720), .V_FP(5), .V_SYNC(5), .V_BP(20)
    ) u_vtg (
        .pclk(pclk), .resetn(!reset),
        .hcount(x), .vcount(y), .de(de), .hsync(hs), .vsync(vs)
    );

    wire [23:0] bars = x < 12'd160  ? 24'hff0000 :
                       x < 12'd320  ? 24'h00ff00 :
                       x < 12'd480  ? 24'h0000ff :
                       x < 12'd640  ? 24'hffffff :
                       x < 12'd800  ? 24'h00ffff :
                       x < 12'd960  ? 24'hff00ff :
                       x < 12'd1120 ? 24'hffff00 : 24'h000000;
    reg [23:0] rgb_pipe = 0, rgb_out = 0;
    reg de_pipe = 0, de_out = 0;
    reg hs_pipe = 0, hs_out = 0;
    reg vs_pipe = 0, vs_out = 0;
    reg [11:0] x_pipe = 0, x_out = 0;
    reg [10:0] y_pipe = 0, y_out = 0;
    always @(posedge pclk) begin
        rgb_pipe <= bars;       rgb_out <= rgb_pipe;
        de_pipe <= de;          de_out <= de_pipe;
        hs_pipe <= hs;          hs_out <= hs_pipe;
        vs_pipe <= vs;          vs_out <= vs_pipe;
        x_pipe <= x;            x_out <= x_pipe;
        y_pipe <= y;            y_out <= y_pipe;
    end
    wire [23:0] hdmi_rgb;
    wire hdmi_de, hdmi_hs, hdmi_vs;
    if (CONTROL_SKEW_CYCLES == 0) begin : aligned_control
        assign hdmi_rgb = rgb_out;
        assign hdmi_de = de_out;
        assign hdmi_hs = hs_out;
        assign hdmi_vs = vs_out;
    end else begin : skewed_control
        reg [23:0] rgb_delay [0:CONTROL_SKEW_CYCLES-1];
        reg [CONTROL_SKEW_CYCLES-1:0] de_delay = '0;
        reg [CONTROL_SKEW_CYCLES-1:0] hs_delay = '0;
        reg [CONTROL_SKEW_CYCLES-1:0] vs_delay = '0;
        always @(posedge pclk) begin
            rgb_delay[0] <= rgb_out;
            de_delay <= {de_delay[CONTROL_SKEW_CYCLES-2:0], de_out};
            hs_delay <= {hs_delay[CONTROL_SKEW_CYCLES-2:0], hs_out};
            vs_delay <= {vs_delay[CONTROL_SKEW_CYCLES-2:0], vs_out};
            for (int i = 1; i < CONTROL_SKEW_CYCLES; i++)
                rgb_delay[i] <= rgb_delay[i-1];
        end
        assign hdmi_rgb = rgb_delay[CONTROL_SKEW_CYCLES-1];
        assign hdmi_de = de_delay[CONTROL_SKEW_CYCLES-1];
        assign hdmi_hs = hs_delay[CONTROL_SKEW_CYCLES-1];
        assign hdmi_vs = vs_delay[CONTROL_SKEW_CYCLES-1];
    end

    // The full Mac supplies a 50 MHz peripheral clock and PCM valid pulses.
    reg [1:0] pb_reset_pipe = 2'b11;
    always @(posedge pb_clk or negedge locked) begin
        if (!locked) pb_reset_pipe <= 2'b11;
        else         pb_reset_pipe <= {pb_reset_pipe[0], 1'b0};
    end
    reg [10:0] sample_div = 0;
    reg [5:0] tone_phase = 0;
    reg sample_valid = 0;
    always @(posedge pb_clk) begin
        sample_valid <= 0;
        if (sample_div == 11'd1041) begin
            sample_div <= 0;
            sample_valid <= 1;
            tone_phase <= tone_phase == 6'd47 ? 0 : tone_phase + 1'b1;
        end else sample_div <= sample_div + 1'b1;
    end
    wire [15:0] sample_word = tone_phase < 6'd24 ? 16'd4096 : -16'sd4096;
    tqtt_hdmi_audio_video u_hdmi (
        .pclk(pclk), .pclk_x5(pclk_x5), .video_locked(locked),
        .rgb(hdmi_rgb), .de(hdmi_de), .hs(hdmi_hs), .vs(hdmi_vs),
        .x(x_out), .y(y_out),
        .pb_clk(pb_clk), .pb_rst(pb_reset_pipe[1]),
        .sample_valid(sample_valid),
        .sample_l(sample_word), .sample_r(sample_word),
        .tmds_clk_p(tmds_clk_p), .tmds_clk_n(tmds_clk_n),
        .tmds_p(tmds_p), .tmds_n(tmds_n)
    );
    assign hdmi_out_en = 1'b1;
endmodule
