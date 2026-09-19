// tb_adb_inject.v — unit-tb wrapper for the host-side ADB event
// injection path: adb_inject.v (MMIO pb_* shim) + adb_keyboard.v +
// adb_mouse.v wired exactly as fpga_top_peripherals.vh wires them.
//
// The tb drives:
//   - the pb_* face with the same one-byte-per-write shape that
//     peripheral_bus.v's SLOT_ADBINJ delivers (this is what a JTAG-AXI
//     word write ultimately becomes), and
//   - the device-bus command face with the same dev_cmd_* pulses that
//     adb_phy.v's dispatcher emits when the PIC firmware polls the bus,
// and observes the device response bytes — i.e. it covers the full
// "host injects an event over MMIO → device model reports it on the
// next TALK poll" contract without needing bit-level ADB framing
// (tb_adb_phy covers that layer).
//
// Build: make tb-adb-inject

`default_nettype none

module tb_adb_inject (
    input  wire        clk,
    input  wire        rst,

    // pb_* face (as delivered by peripheral_bus SLOT_ADBINJ)
    input  wire [7:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output wire [7:0]  pb_rdata,
    output wire        pb_ack,

    // Device-bus command face (as driven by adb_phy's dispatcher)
    input  wire        dev_cmd_valid,
    input  wire [3:0]  dev_cmd_addr,
    input  wire [2:0]  dev_cmd_op,
    input  wire        dev_listen_valid,
    input  wire [7:0]  dev_listen_b0,
    input  wire [7:0]  dev_listen_b1,

    // Keyboard responses
    output wire        kbd_resp_valid,
    output wire        kbd_resp_empty,
    output wire [7:0]  kbd_resp_b0,
    output wire [7:0]  kbd_resp_b1,
    output wire        kbd_srq,

    // Mouse responses
    output wire        ms_resp_valid,
    output wire        ms_resp_empty,
    output wire [7:0]  ms_resp_b0,
    output wire [7:0]  ms_resp_b1,
    output wire        ms_srq
);

    wire        inj_kc_valid;
    wire [7:0]  inj_kc_byte;
    wire [7:0]  kbd_status;
    wire        inj_btn_valid;
    wire        inj_btn_state;
    wire        inj_dx_valid;
    wire signed [7:0] inj_dx;
    wire        inj_dy_valid;
    wire signed [7:0] inj_dy;
    wire [7:0]  mouse_status;

    adb_inject u_adb_inject (
        .clk           (clk),
        .rst           (rst),
        .pb_addr       (pb_addr),
        .pb_wdata      (pb_wdata),
        .pb_wr         (pb_wr),
        .pb_rd         (pb_rd),
        .pb_rdata      (pb_rdata),
        .pb_ack        (pb_ack),
        .inj_kc_valid  (inj_kc_valid),
        .inj_kc_byte   (inj_kc_byte),
        .kbd_status    (kbd_status),
        .inj_btn_valid (inj_btn_valid),
        .inj_btn_state (inj_btn_state),
        .inj_dx_valid  (inj_dx_valid),
        .inj_dx        (inj_dx),
        .inj_dy_valid  (inj_dy_valid),
        .inj_dy        (inj_dy),
        .mouse_status  (mouse_status)
    );

    adb_keyboard u_adb_kbd (
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
        .inj_status      (kbd_status)
    );

    adb_mouse u_adb_mouse (
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
        .inj_status      (mouse_status)
    );

endmodule

`default_nettype wire
