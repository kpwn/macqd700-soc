// adb_inject.v — host-side MMIO bridge for the ADB keyboard/mouse models.
//
// The host (testbench, JTAG-AXI agent, future USB→ADB bridge) writes
// keycodes / mouse deltas through this peripheral-bus shim.  Each write
// is forwarded as a one-cycle pulse on the matching device's injection
// port; reads expose status flags.
//
// Register map (byte offset within the 4 KB ADBINJ window starting at
// system 0x5000_0000 + mac-canonical offset 0x011000 — i.e. system
// address 0x5001_1000):
//
//   +0x00  W  KBD_ENQUEUE — write an ADB keycode (bit 7 = release).
//                            Pulses the keyboard's inj_kc_valid for one
//                            cycle.  Reads return KBD_STATUS.
//   +0x01  R  KBD_STATUS  — bit 0 = !empty, bits 6..1 = FIFO count low
//                            bits, bit 7 = full.
//   +0x02  W  MOUSE_BTN   — bit 0 = button state (1 = pressed).  Pulses
//                            inj_btn_valid for one cycle.
//   +0x03  W  MOUSE_DX    — signed 8-bit X delta.  Pulses inj_dx_valid.
//   +0x04  W  MOUSE_DY    — signed 8-bit Y delta.  Pulses inj_dy_valid.
//   +0x05  R  MOUSE_STATUS- bit 0 = event_pending.
//
// Word-aligned alias block (offsets 0x10..0x1C) — same side effects as
// the byte-granular map above, but every register sits at a 32-bit
// word boundary.  Needed because word-only AXI masters (the JTAG-AXI
// debug master is one; a future USB-HID bridge MCU may be another)
// cannot issue the unaligned single-byte writes that offsets
// 0x01..0x03 above require.  The byte map stays for byte-capable
// masters (the m68k itself) and for existing tbs:
//
//   +0x10  W  KBD_ENQUEUE alias   R  KBD_STATUS
//   +0x14  W  MOUSE_BTN   alias   R  MOUSE_STATUS
//   +0x18  W  MOUSE_DX    alias   R  0x00
//   +0x1C  W  MOUSE_DY    alias   R  0x00
//
// pb_* protocol: pb_wr / pb_rd are one-cycle pulses; we ack the same
// cycle (combinational ack) since the side-effect is a one-cycle pulse
// that the device latches synchronously.

`default_nettype none

module adb_inject (
    input  wire        clk,
    input  wire        rst,

    // peripheral_bus pb_* slave face
    input  wire [7:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output reg  [7:0]  pb_rdata,
    output reg         pb_ack,

    // To adb_keyboard.inj_*
    output reg         inj_kc_valid,
    output reg  [7:0]  inj_kc_byte,
    input  wire [7:0]  kbd_status,

    // To adb_mouse.inj_*
    output reg         inj_btn_valid,
    output reg         inj_btn_state,
    output reg         inj_dx_valid,
    output reg signed [7:0] inj_dx,
    output reg         inj_dy_valid,
    output reg signed [7:0] inj_dy,
    input  wire [7:0]  mouse_status
);

    localparam [7:0] ADBINJ_KBD_ENQ    = 8'h00;
    localparam [7:0] ADBINJ_KBD_STAT   = 8'h01;
    localparam [7:0] ADBINJ_MOUSE_BTN  = 8'h02;
    localparam [7:0] ADBINJ_MOUSE_DX   = 8'h03;
    localparam [7:0] ADBINJ_MOUSE_DY   = 8'h04;
    localparam [7:0] ADBINJ_MOUSE_STAT = 8'h05;
    // Word-aligned aliases (see header) — reachable by word-only AXI
    // masters such as the JTAG-AXI debug bridge.
    localparam [7:0] ADBINJ_KBD_ENQ_W    = 8'h10;
    localparam [7:0] ADBINJ_MOUSE_BTN_W  = 8'h14;
    localparam [7:0] ADBINJ_MOUSE_DX_W   = 8'h18;
    localparam [7:0] ADBINJ_MOUSE_DY_W   = 8'h1C;

    always @(posedge clk) begin
        if (rst) begin
            inj_kc_valid  <= 1'b0;
            inj_kc_byte   <= 8'h00;
            inj_btn_valid <= 1'b0;
            inj_btn_state <= 1'b0;
            inj_dx_valid  <= 1'b0;
            inj_dx        <= 8'sd0;
            inj_dy_valid  <= 1'b0;
            inj_dy        <= 8'sd0;
            pb_rdata      <= 8'h00;
            pb_ack        <= 1'b0;
        end else begin
            // Default one-cycle pulses.
            inj_kc_valid  <= 1'b0;
            inj_btn_valid <= 1'b0;
            inj_dx_valid  <= 1'b0;
            inj_dy_valid  <= 1'b0;
            pb_ack        <= pb_wr | pb_rd;

            if (pb_wr) begin
                case (pb_addr)
                    ADBINJ_KBD_ENQ,
                    ADBINJ_KBD_ENQ_W: begin
                        inj_kc_byte  <= pb_wdata;
                        inj_kc_valid <= 1'b1;
                    end
                    ADBINJ_MOUSE_BTN,
                    ADBINJ_MOUSE_BTN_W: begin
                        inj_btn_state <= pb_wdata[0];
                        inj_btn_valid <= 1'b1;
                    end
                    ADBINJ_MOUSE_DX,
                    ADBINJ_MOUSE_DX_W: begin
                        inj_dx       <= $signed(pb_wdata);
                        inj_dx_valid <= 1'b1;
                    end
                    ADBINJ_MOUSE_DY,
                    ADBINJ_MOUSE_DY_W: begin
                        inj_dy       <= $signed(pb_wdata);
                        inj_dy_valid <= 1'b1;
                    end
                    default: ;
                endcase
            end

            if (pb_rd) begin
                case (pb_addr)
                    ADBINJ_KBD_STAT,
                    ADBINJ_KBD_ENQ_W:   pb_rdata <= kbd_status;
                    ADBINJ_MOUSE_STAT,
                    ADBINJ_MOUSE_BTN_W: pb_rdata <= mouse_status;
                    default:            pb_rdata <= 8'h00;
                endcase
            end else begin
                pb_rdata <= 8'h00;
            end
        end
    end

endmodule

`default_nettype wire
