// tb_adb_pic_phy.v — Verilator wrapper for the REAL-FIRMWARE ADB chain
// integration test.
//
// This is the testbench that closes the gap tb_adb_phy.cpp leaves open:
// tb_adb_phy drives adb_phy.v with a hand-bit-banged host model whose
// timing mirrors adb_phy.v's own constants — it can never catch a
// mismatch between adb_phy.v and the REAL host, the PIC1654S running
// the genuine 342S0440-B firmware (rtl/mac/adb_pic_fw.hex).  Here the
// real PIC (adb_pic_modem.v + pic16c5x.v) IS the host, wired to
// adb_phy.v + adb_keyboard.v + adb_mouse.v exactly as
// rtl/soc/fpga_top_peripherals.vh wires them, and the C++ side plays
// the VIA1/68k role at the CB1/CB2/state-pin level (same edge phasing
// as via1.v's MAME-matched external-clock shift register).
//
// This is the regression for the 2026-07-21 "injected ADB mouse event
// never drains / boot stalls spinning on VIA1 SR" hardware bug: the
// firmware's autonomous idle-state autopoll (DecCou0 -> lb_063 in the
// 342S0440-B disassembly, see github.com/lampmerchant/macseadb88) must
// be decoded by adb_phy.v so a pending device event is consumed and
// the firmware notifies the 68k by clocking a byte into the VIA SR.

`default_nettype none

module tb_adb_pic_phy (
    input  wire        clk,
    input  wire        rst,
    input  wire        phi2_tick,

    // VIA1/68k role, driven by the C++ tb:
    input  wire [1:0]  adb_state,     // VIA1 ORB[5:4] -> PIC PORTA[1:0]
    input  wire        via_cb2_oe,    // VIA drives CB2 (SR shift-out mode)
    input  wire        via_cb2_out,

    // Observability:
    output wire        cb1,               // PIC -> VIA shift clock
    output wire        cb2,               // composite CB2 (open-drain mux)
    output wire        adb_irq_pending,   // PIC RB4 (inverted) = VIA PB3 IRQ
    output wire        adb_bus,           // composite open-drain ADB bus
    output wire        pic_adb_out_o,     // host-only drive (frame timing)
    output wire        dev_cmd_valid_o,   // adb_phy device-bus dispatch
    output wire [3:0]  dev_cmd_addr_o,
    output wire [2:0]  dev_cmd_op_o,

    // MMIO injection (same as adb_inject.v's accumulator outputs)
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

    wire        pic_cb1_out;
    wire        pic_cb2_out;
    wire        pic_cb2_oe;
    wire        pic_adb_out;
    wire        adb_bus_in;
    wire        adb_rtcc_in;

    // Same open-drain CB2 mux as rtl/soc/fpga_top_peripherals.vh.
    wire adb_cb2 = via_cb2_oe ? via_cb2_out :
                   (pic_cb2_oe ? pic_cb2_out : 1'b1);

    assign cb1           = pic_cb1_out;
    assign cb2           = adb_cb2;
    assign adb_bus       = adb_bus_in;
    assign pic_adb_out_o = pic_adb_out;

    adb_pic_modem u_pic (
        .clk            (clk),
        .rst            (rst),
        .phi2_tick      (phi2_tick),
        .cb1_out        (pic_cb1_out),
        .cb2_out        (pic_cb2_out),
        .cb2_oe         (pic_cb2_oe),
        .cb2_in         (adb_cb2),
        .adb_bus_in     (adb_bus_in),
        .rtcc_in        (adb_rtcc_in),
        .pic_adb_out    (pic_adb_out),
        .adb_state      (adb_state),
        .adb_irq_pending(adb_irq_pending)
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

    assign dev_cmd_valid_o = dev_cmd_valid;
    assign dev_cmd_addr_o  = dev_cmd_addr;
    assign dev_cmd_op_o    = dev_cmd_op;

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
        .adb_in          (adb_bus_in),
        .rtcc_in         (adb_rtcc_in),
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
