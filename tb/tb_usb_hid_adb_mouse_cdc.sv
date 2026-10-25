`timescale 1ns/1ps
module tb_usb_hid_adb_mouse_cdc;
    reg usb_clk = 0, pb_clk = 0;
    always #8 usb_clk = ~usb_clk;
    always #10 pb_clk = ~pb_clk;
    reg usb_rst = 1, pb_rst = 1;
    reg [1:0] usb_type = 0;
    reg usb_full_report = 0, usb_button_left = 0;
    reg signed [7:0] usb_dx = 0, usb_dy = 0;
    wire report_coalesced, report_valid, report_ready, connected, button_left;
    wire signed [7:0] dx, dy, event_dx, event_dy;
    wire event_valid, event_button_valid, event_button;
    reg event_ready = 0;
    reg [17:0] expected [0:2];
    integer seen = 0;

    usb_hid_adb_mouse_cdc u_cdc (
        .usb_clk(usb_clk), .usb_rst(usb_rst),
        .usb_type(usb_type), .usb_full_report(usb_full_report),
        .usb_button_left(usb_button_left), .usb_dx(usb_dx), .usb_dy(usb_dy),
        .report_coalesced(report_coalesced),
        .pb_clk(pb_clk), .pb_rst(pb_rst),
        .report_valid(report_valid), .report_ready(report_ready),
        .connected(connected), .button_left(button_left), .dx(dx), .dy(dy)
    );
    usb_hid_adb_mouse u_translate (
        .clk(pb_clk), .rst(pb_rst),
        .report_valid(report_valid), .report_ready(report_ready),
        .connected(connected), .button_left(button_left), .dx(dx), .dy(dy),
        .event_valid(event_valid), .event_ready(event_ready),
        .event_button_valid(event_button_valid), .event_button(event_button),
        .event_dx(event_dx), .event_dy(event_dy)
    );
    always @(posedge pb_clk) begin
        if (!pb_rst && event_valid && event_ready) begin
            if (seen >= 3 || {event_button_valid, event_button, event_dx, event_dy}
                             !== expected[seen])
                $fatal(1, "mouse event %0d mismatch", seen);
            seen <= seen + 1;
        end
    end
    task automatic host_report(input bit button, input signed [7:0] x,
                               input signed [7:0] y);
        @(negedge usb_clk);
        usb_type = 2'd2;
        usb_button_left = button;
        usb_dx = x;
        usb_dy = y;
        usb_full_report = 1;
        @(negedge usb_clk);
        usb_full_report = 0;
    endtask
    initial begin
        expected[0] = {1'b1, 1'b1, 8'd10, 8'hfb};
        expected[1] = {1'b0, 1'b1, 8'hfd, 8'd4};
        expected[2] = {1'b1, 1'b0, 8'd0, 8'd0};
        repeat (20) @(negedge usb_clk);
        usb_rst = 0;
        pb_rst = 0;
        repeat (40) @(negedge usb_clk);
        host_report(1, 8'sd10, -8'sd5);
        wait(event_valid);
        repeat (3) begin
            @(negedge pb_clk);
            if (!event_valid || !event_button_valid || !event_button ||
                event_dx !== 8'sd10 || event_dy !== -8'sd5)
                $fatal(1, "mouse report changed under backpressure");
        end
        event_ready = 1;
        host_report(1, 0, 0); // held button and no motion: no ADB event
        host_report(1, -8'sd3, 8'sd4);
        @(negedge usb_clk);
        usb_type = 0; // unplug without another full report
        wait(seen == 3);
        repeat (40) @(negedge pb_clk);
        if (report_coalesced) $fatal(1, "unexpected mouse report coalescing");
        $display("PASS USB HID mouse CDC, motion, idle, and disconnect");
        $finish;
    end
    initial begin
        #200000;
        $fatal(1, "USB HID mouse CDC test timed out");
    end
endmodule
