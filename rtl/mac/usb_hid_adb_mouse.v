// Convert USB mouse reports in pb_clk into compound ADB mouse injections.
// Each accepted report may update button, X, and Y together. A disconnect
// releases the held button even if the USB host produces no final report.
`default_nettype none
module usb_hid_adb_mouse (
    input  wire               clk,
    input  wire               rst,
    input  wire               report_valid,
    output wire               report_ready,
    input  wire               connected,
    input  wire               button_left,
    input  wire signed [7:0]  dx,
    input  wire signed [7:0]  dy,
    output wire               event_valid,
    input  wire               event_ready,
    output wire               event_button_valid,
    output wire               event_button,
    output wire signed [7:0] event_dx,
    output wire signed [7:0] event_dy
);
    reg old_button;
    wire new_button = connected && button_left;
    assign event_button_valid = old_button != new_button;
    assign event_button = new_button;
    assign event_dx = connected ? dx : 8'sd0;
    assign event_dy = connected ? dy : 8'sd0;
    assign event_valid = report_valid &&
                         (event_button_valid || event_dx != 0 || event_dy != 0);
    assign report_ready = !event_valid || event_ready;
    always @(posedge clk) begin
        if (rst) old_button <= 1'b0;
        else if (report_valid && report_ready) old_button <= new_button;
    end
endmodule
`default_nettype wire
