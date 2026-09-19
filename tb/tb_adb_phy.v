// tb_adb_phy.v — Verilator wrapper for the bit-level ADB PHY unit test.
//
// Wires up adb_phy.v (the real bit-level ADB device responder) +
// adb_keyboard.v + adb_mouse.v exactly as rtl/soc/fpga_top_peripherals.vh
// does: adb_phy is the sole driver of the {dev_cmd_valid, dev_cmd_addr,
// dev_cmd_op, dev_listen_*} device-bus, and consumes the keyboard/mouse
// dev_resp_* / dev_srq outputs.
//
// The C++ tb drives `pic_adb_out` with a hand-bit-banged ADB command
// frame (Attention + Sync + command byte + stop, one phi2_tick per
// microsecond — matching adb_phy.v's own _US timing constants) and
// samples the composite `adb_in` output to decode whatever response the
// addressed device drives back.  This exercises the exact round trip a
// real ADB bus transaction takes through the new bit-level PHY logic,
// independent of the byte-level adb_modem.v path tb_adb.cpp already
// covers.

`default_nettype none

module tb_adb_phy (
    input  wire        clk,
    input  wire        rst,
    input  wire        phi2_tick,

    // Host (PIC) side of the ADB bus — driven by the tb to synthesize
    // command frames.
    input  wire        pic_adb_out,
    output wire        adb_in,
    output wire        rtcc_in,

    // MMIO injection inputs (driven by tb to arm keyboard/mouse state)
    input  wire        inj_kc_valid,
    input  wire [7:0]  inj_kc_byte,
    output wire [7:0]  inj_kc_status,

    input  wire        inj_btn_valid,
    input  wire        inj_btn_state,
    input  wire        inj_dx_valid,
    input  wire signed [7:0] inj_dx,
    input  wire        inj_dy_valid,
    input  wire signed [7:0] inj_dy,
    output wire [7:0]  inj_mouse_status
);

    wire        dev_cmd_valid;
    wire [3:0]  dev_cmd_addr;
    wire [2:0]  dev_cmd_op;
    wire        dev_listen_valid;
    wire [7:0]  dev_listen_b0;
    wire [7:0]  dev_listen_b1;

    wire        kbd_resp_valid, kbd_resp_empty;
    wire [7:0]  kbd_resp_b0, kbd_resp_b1;
    wire        kbd_srq;

    wire        ms_resp_valid, ms_resp_empty;
    wire [7:0]  ms_resp_b0, ms_resp_b1;
    wire        ms_srq;

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

    adb_phy #(
        .CLK_MHZ(32)
    ) u_phy (
        .clk             (clk),
        .rst             (rst),
        .phi2_tick       (phi2_tick),
        .pic_adb_out     (pic_adb_out),
        .adb_in          (adb_in),
        .rtcc_in         (rtcc_in),
        .dev_cmd_valid   (dev_cmd_valid),
        .dev_cmd_addr    (dev_cmd_addr),
        .dev_cmd_op      (dev_cmd_op),
        .dev_listen_valid(dev_listen_valid),
        .dev_listen_b0   (dev_listen_b0),
        .dev_listen_b1   (dev_listen_b1),
        .dev_resp_valid  ({ms_resp_valid, kbd_resp_valid}),
        .dev_resp_empty  ({ms_resp_empty, kbd_resp_empty}),
        .dev_resp_b0     ({ms_resp_b0,    kbd_resp_b0}),
        .dev_resp_b1     ({ms_resp_b1,    kbd_resp_b1}),
        .dev_srq         ({ms_srq,        kbd_srq})
    );

endmodule

`default_nettype wire
