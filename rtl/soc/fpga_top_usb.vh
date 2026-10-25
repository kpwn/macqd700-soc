// Optional single-port TQTT USB HID host on PMOD0/J1. Included after the
// peripheral island so pb_clk and pb_full_rst_bank are available. The USB-A
// connector, 5 V VBUS, data resistors, and protection are external hardware.
    wire usb_clk;
    wire usb_mmcm_locked = core_mmcm_locked && !tqtt_reset;
    BUFG u_usb_clk_bufg (.I(tqtt_usb_unbuf), .O(usb_clk));
    reg [7:0] usb_reset_count = 0;
    always @(posedge usb_clk) begin
        if (!usb_mmcm_locked) usb_reset_count <= 0;
        else if (!(&usb_reset_count)) usb_reset_count <= usb_reset_count + 1'b1;
    end
    wire usb_reset = !(&usb_reset_count);

    wire usb_dp_i, usb_dm_i, usb_dp_o, usb_dm_o, usb_oe;
    IOBUF u_usb_dp_buf (.I(usb_dp_o), .T(!usb_oe), .O(usb_dp_i), .IO(usb_dp));
    IOBUF u_usb_dm_buf (.I(usb_dm_o), .T(!usb_oe), .O(usb_dm_i), .IO(usb_dm));
    wire [1:0] usb_type;
    wire usb_full_report;
    wire [7:0] usb_modifiers, usb_key0, usb_key1, usb_key2;
    wire [7:0] usb_key3, usb_key4, usb_key5;
    wire [2:0] usb_mouse_btn;
    wire signed [7:0] usb_mouse_dx, usb_mouse_dy;
    wire [9:0] usb_rom_addr;
    wire [3:0] usb_rom_data;
    wire usb_rom_en;
    usb_hid_host #(
        .FULL_SPEED(1), .KEYBOARD_SUPPORT(1),
        .MOUSE_SUPPORT(1), .GAME_SUPPORT(0)
    ) u_usb_host (
        .clk(usb_clk), .reset(usb_reset), .cs(1'b1),
        .usb_dm_i(usb_dm_i), .usb_dp_i(usb_dp_i),
        .usb_dm_o(usb_dm_o), .usb_dp_o(usb_dp_o), .usb_oe(usb_oe),
        .typ(usb_type), .full_report(usb_full_report),
        .connerr(), .busy(),
        .key_modifiers(usb_modifiers),
        .key_0(usb_key0), .key_1(usb_key1), .key_2(usb_key2),
        .key_3(usb_key3), .key_4(usb_key4), .key_5(usb_key5),
        .mouse_btn(usb_mouse_btn),
        .mouse_dx(usb_mouse_dx), .mouse_dy(usb_mouse_dy),
        .game_l(), .game_r(), .game_u(), .game_d(),
        .game_a(), .game_b(), .game_x(), .game_y(),
        .game_sel(), .game_sta(), .game_extra(),
        .dbg_hid_report(), .dbg_hid_regs(),
        .rom_addr(usb_rom_addr), .rom_dout(usb_rom_data), .rom_en(usb_rom_en)
    );
    usb_hid_host_rom u_usb_rom (
        .clk(usb_clk), .addr(usb_rom_addr),
        .dout(usb_rom_data), .en(usb_rom_en)
    );

    wire usb_kbd_report_valid, usb_kbd_report_ready, usb_kbd_connected;
    wire [7:0] usb_kbd_modifiers;
    wire [47:0] usb_kbd_usages;
    usb_hid_adb_keyboard_cdc u_usb_kbd_cdc (
        .usb_clk(usb_clk), .usb_rst(usb_reset),
        .usb_type(usb_type), .usb_full_report(usb_full_report),
        .usb_modifiers(usb_modifiers),
        .usb_usages({usb_key0,usb_key1,usb_key2,usb_key3,usb_key4,usb_key5}),
        .report_coalesced(),
        .pb_clk(pb_clk), .pb_rst(pb_full_rst_bank[0]),
        .report_valid(usb_kbd_report_valid),
        .report_ready(usb_kbd_report_ready),
        .connected(usb_kbd_connected),
        .modifiers(usb_kbd_modifiers), .usages(usb_kbd_usages)
    );
    usb_hid_adb_keyboard u_usb_kbd_adb (
        .clk(pb_clk), .rst(pb_full_rst_bank[0]),
        .report_valid(usb_kbd_report_valid),
        .report_ready(usb_kbd_report_ready),
        .connected(usb_kbd_connected),
        .modifiers(usb_kbd_modifiers), .usages(usb_kbd_usages),
        .event_valid(usb_adb_kc_valid), .event_byte(usb_adb_kc_byte),
        .event_ready(usb_adb_kc_ready)
    );

    wire usb_ms_report_valid, usb_ms_report_ready, usb_ms_connected;
    wire usb_ms_button;
    wire signed [7:0] usb_ms_dx, usb_ms_dy;
    usb_hid_adb_mouse_cdc u_usb_mouse_cdc (
        .usb_clk(usb_clk), .usb_rst(usb_reset),
        .usb_type(usb_type), .usb_full_report(usb_full_report),
        .usb_button_left(usb_mouse_btn[0]),
        .usb_dx(usb_mouse_dx), .usb_dy(usb_mouse_dy),
        .report_coalesced(),
        .pb_clk(pb_clk), .pb_rst(pb_full_rst_bank[0]),
        .report_valid(usb_ms_report_valid),
        .report_ready(usb_ms_report_ready),
        .connected(usb_ms_connected), .button_left(usb_ms_button),
        .dx(usb_ms_dx), .dy(usb_ms_dy)
    );
    usb_hid_adb_mouse u_usb_mouse_adb (
        .clk(pb_clk), .rst(pb_full_rst_bank[0]),
        .report_valid(usb_ms_report_valid),
        .report_ready(usb_ms_report_ready),
        .connected(usb_ms_connected), .button_left(usb_ms_button),
        .dx(usb_ms_dx), .dy(usb_ms_dy),
        .event_valid(usb_adb_mouse_valid),
        .event_ready(usb_adb_mouse_ready),
        .event_button_valid(usb_adb_mouse_button_valid),
        .event_button(usb_adb_mouse_button),
        .event_dx(usb_adb_mouse_dx), .event_dy(usb_adb_mouse_dy)
    );
