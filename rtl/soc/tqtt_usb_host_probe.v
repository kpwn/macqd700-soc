// Standalone TQTT PMOD0 USB HID host probe. No full Mac resources are used.
// J1 pin 1 = D+ (D10), pin 3 = D- (D9); see docs/tqtt_usb_host_plan.md.
`default_nettype none

module tqtt_usb_host_probe (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    inout  wire       usb_dp,
    inout  wire       usb_dm,
    output wire [3:0] led
);
    wire ref_clk_unbuf, ref_clk;
    IBUFDS u_ref_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(ref_clk_unbuf));
    BUFG u_ref_bufg (.I(ref_clk_unbuf), .O(ref_clk));

    // 100 MHz / 5 * 60 = 1200 MHz VCO; / 20 = 60 MHz host clock.
    wire usb_clk_unbuf, usb_clk, clkfb, locked;
    MMCME4_BASE #(
        .CLKIN1_PERIOD(10.0), .DIVCLK_DIVIDE(5),
        .CLKFBOUT_MULT_F(60.0), .CLKOUT0_DIVIDE_F(20.0),
        .BANDWIDTH("OPTIMIZED"), .STARTUP_WAIT("FALSE")
    ) u_usb_mmcm (
        .CLKIN1(ref_clk), .CLKFBIN(clkfb), .CLKFBOUT(clkfb), .CLKFBOUTB(),
        .CLKOUT0(usb_clk_unbuf), .CLKOUT0B(),
        .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .RST(1'b0), .PWRDWN(1'b0), .LOCKED(locked)
    );
    BUFG u_usb_bufg (.I(usb_clk_unbuf), .O(usb_clk));

    reg [7:0] reset_count = 0;
    always @(posedge usb_clk) begin
        if (!locked)
            reset_count <= 0;
        else if (!(&reset_count))
            reset_count <= reset_count + 1'b1;
    end
    wire usb_reset = !(&reset_count);

    wire dp_i, dm_i, dp_o, dm_o, usb_oe;
    IOBUF u_dp_buf (.I(dp_o), .T(!usb_oe), .O(dp_i), .IO(usb_dp));
    IOBUF u_dm_buf (.I(dm_o), .T(!usb_oe), .O(dm_i), .IO(usb_dm));

    wire [1:0] typ;
    wire full_report, connerr, busy;
    wire [7:0] key_modifiers, key_0, key_1, key_2;
    wire [2:0] mouse_btn;
    wire signed [7:0] mouse_dx, mouse_dy;
    wire [63:0] hid_report, hid_regs;
    wire [9:0] rom_addr;
    wire [3:0] rom_dout;
    wire rom_en;

    usb_hid_host #(
        .FULL_SPEED(1), .KEYBOARD_SUPPORT(1),
        .MOUSE_SUPPORT(1), .GAME_SUPPORT(0)
    ) u_host (
        .clk(usb_clk), .reset(usb_reset), .cs(1'b1),
        .usb_dm_i(dm_i), .usb_dp_i(dp_i),
        .usb_dm_o(dm_o), .usb_dp_o(dp_o), .usb_oe(usb_oe),
        .typ(typ), .full_report(full_report), .connerr(connerr), .busy(busy),
        .key_modifiers(key_modifiers),
        .key_0(key_0), .key_1(key_1), .key_2(key_2),
        .key_3(), .key_4(), .key_5(),
        .mouse_btn(mouse_btn), .mouse_dx(mouse_dx), .mouse_dy(mouse_dy),
        .game_l(), .game_r(), .game_u(), .game_d(),
        .game_a(), .game_b(), .game_x(), .game_y(),
        .game_sel(), .game_sta(), .game_extra(),
        .dbg_hid_report(hid_report), .dbg_hid_regs(hid_regs),
        .rom_addr(rom_addr), .rom_dout(rom_dout), .rom_en(rom_en)
    );
    usb_hid_host_rom u_rom (
        .clk(usb_clk), .addr(rom_addr), .dout(rom_dout), .en(rom_en)
    );

    reg [15:0] report_count = 0;
    reg report_toggle = 0;
    reg [63:0] last_report = 0;
    reg [31:0] last_keys = 0;
    reg [31:0] last_mouse = 0;
    always @(posedge usb_clk) begin
        if (usb_reset) begin
            report_count <= 0;
            report_toggle <= 0;
            last_report <= 0;
            last_keys <= 0;
            last_mouse <= 0;
        end else if (full_report) begin
            report_count <= report_count + 1'b1;
            report_toggle <= !report_toggle;
            last_report <= hid_report;
            last_keys <= {key_modifiers, key_0, key_1, key_2};
            last_mouse <= {13'd0, mouse_btn, mouse_dx, mouse_dy};
        end
    end

    wire [31:0] status = {
        report_count, 8'd0, typ, connerr, busy,
        report_toggle, dp_i, dm_i, usb_oe
    };
    tqtt_usb_vio u_vio (
        .clk(usb_clk),
        .probe_in0(status), .probe_in1(last_report),
        .probe_in2(last_keys), .probe_in3(last_mouse),
        .probe_in4(hid_regs)
    );

    assign led = {report_toggle, connerr, typ};
endmodule

`default_nettype wire
