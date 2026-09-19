// tb_adb.v — Verilator wrapper for the ADB unit test.
//
// Wires up adb_modem.v + adb_keyboard.v + adb_mouse.v on a shared 2-device
// device-bus, exposing the modem's VIA-side and the per-device MMIO
// injection ports to the C++ tb.
//
// All signals match adb_modem.v / adb_keyboard.v / adb_mouse.v exactly so
// the C++ tb can drive them directly via the dut handle.

`default_nettype none

module tb_adb (
    input  wire        clk,
    input  wire        rst,
    input  wire        phi2_tick,

    // VIA1-facing byte interface (driven by tb to mimic the host SR)
    output wire [7:0]  via_rx_byte,
    output wire        via_rx_valid,
    input  wire        via_rx_ready,
    input  wire [7:0]  via_tx_byte,
    input  wire        via_tx_valid,

    output wire        adb_irq_pending,

    // MMIO injection inputs (driven by tb)
    input  wire        inj_kc_valid,
    input  wire [7:0]  inj_kc_byte,
    output wire [7:0]  inj_kc_status,

    input  wire        inj_btn_valid,
    input  wire        inj_btn_state,
    input  wire        inj_dx_valid,
    input  wire signed [7:0] inj_dx,
    input  wire        inj_dy_valid,
    input  wire signed [7:0] inj_dy,
    output wire [7:0]  inj_mouse_status,

    output wire [3:0]  dbg_state
);

    // Bus from modem to devices.
    wire        dev_cmd_valid;
    wire [3:0]  dev_cmd_addr;
    wire [2:0]  dev_cmd_op;
    wire        dev_listen_valid;
    wire [7:0]  dev_listen_b0;
    wire [7:0]  dev_listen_b1;

    // Per-device responses (2-bit vector: [0]=keyboard, [1]=mouse).
    wire [1:0]  resp_valid;
    wire [1:0]  resp_empty;
    wire [15:0] resp_b0;
    wire [15:0] resp_b1;
    wire [1:0]  srq;

    // Keyboard
    wire        kbd_resp_valid;
    wire        kbd_resp_empty;
    wire [7:0]  kbd_resp_b0;
    wire [7:0]  kbd_resp_b1;
    wire        kbd_srq;
    adb_keyboard u_kbd (
        .clk             (clk),
        .rst             (rst),
        .dev_cmd_valid   (dev_cmd_valid),
        .dev_cmd_addr    (dev_cmd_addr),
        .dev_cmd_op      (dev_cmd_op),
        .dev_resp_valid  (kbd_resp_valid),
        .dev_resp_empty  (kbd_resp_empty),
        .dev_resp_b0     (kbd_resp_b0),
        .dev_resp_b1     (kbd_resp_b1),
        .dev_listen_valid(dev_listen_valid),
        .dev_listen_b0   (dev_listen_b0),
        .dev_listen_b1   (dev_listen_b1),
        .dev_srq         (kbd_srq),
        .inj_kc_valid    (inj_kc_valid),
        .inj_kc_byte     (inj_kc_byte),
        .inj_status      (inj_kc_status)
    );

    // Mouse
    wire        ms_resp_valid;
    wire        ms_resp_empty;
    wire [7:0]  ms_resp_b0;
    wire [7:0]  ms_resp_b1;
    wire        ms_srq;
    adb_mouse u_mouse (
        .clk             (clk),
        .rst             (rst),
        .dev_cmd_valid   (dev_cmd_valid),
        .dev_cmd_addr    (dev_cmd_addr),
        .dev_cmd_op      (dev_cmd_op),
        .dev_resp_valid  (ms_resp_valid),
        .dev_resp_empty  (ms_resp_empty),
        .dev_resp_b0     (ms_resp_b0),
        .dev_resp_b1     (ms_resp_b1),
        .dev_listen_valid(dev_listen_valid),
        .dev_listen_b0   (dev_listen_b0),
        .dev_listen_b1   (dev_listen_b1),
        .dev_srq         (ms_srq),
        .inj_btn_valid   (inj_btn_valid),
        .inj_btn_state   (inj_btn_state),
        .inj_dx_valid    (inj_dx_valid),
        .inj_dx          (inj_dx),
        .inj_dy_valid    (inj_dy_valid),
        .inj_dy          (inj_dy),
        .inj_status      (inj_mouse_status)
    );

    assign resp_valid = {ms_resp_valid, kbd_resp_valid};
    assign resp_empty = {ms_resp_empty, kbd_resp_empty};
    assign resp_b0    = {ms_resp_b0,    kbd_resp_b0};
    assign resp_b1    = {ms_resp_b1,    kbd_resp_b1};
    assign srq        = {ms_srq,        kbd_srq};

    adb_modem #(
        .NUM_DEVICES     (2),
        .NO_SERVICE_DELAY(4)
    ) u_modem (
        .clk             (clk),
        .rst             (rst),
        .phi2_tick       (phi2_tick),
        .via_rx_byte     (via_rx_byte),
        .via_rx_valid    (via_rx_valid),
        .via_rx_ready    (via_rx_ready),
        .via_tx_byte     (via_tx_byte),
        .via_tx_valid    (via_tx_valid),
        .adb_irq_pending (adb_irq_pending),
        .dev_cmd_valid   (dev_cmd_valid),
        .dev_cmd_addr    (dev_cmd_addr),
        .dev_cmd_op      (dev_cmd_op),
        .dev_listen_valid(dev_listen_valid),
        .dev_listen_b0   (dev_listen_b0),
        .dev_listen_b1   (dev_listen_b1),
        .dev_resp_valid  (resp_valid),
        .dev_resp_empty  (resp_empty),
        .dev_resp_b0     (resp_b0),
        .dev_resp_b1     (resp_b1),
        .dev_srq         (srq),
        .dbg_state       (dbg_state)
    );

endmodule

`default_nettype wire
