// adb_mouse.v — ADB mouse model (default address 3, handler ID 1)
//
// Mirrors the device-bus protocol of adb_keyboard.v.  Maintains an
// accumulated (button, dx, dy) state that the host injects via MMIO and
// the modem reads via TALK register 0.
//
// ADB register 0 mouse encoding (Apple ADB spec, 1985):
//   byte0:   bit 7 = button (0 = pressed)
//            bits 6..0 = signed Y delta (7-bit, two's complement)
//   byte1:   bit 7 = optional secondary button or always-1 for single-
//                    button mice (we keep it 1 for single-button per
//                    macadb.cpp lines that emit `0x80 | (...)`)
//            bits 6..0 = signed X delta
//
// macadb.cpp's adb_pollmouse + adb_talk path encodes deltas as
// `(NewY - LastY)` and `(NewX - LastX)`, clamped to ±63.  We do the
// same: each MMIO write to dx/dy adds into a saturating accumulator;
// the next TALK consumes and zeroes it.
//
// The FIFO_LOG2 from adb_keyboard.v is not needed here — mice don't
// queue events, they accumulate deltas.

`default_nettype none

module adb_mouse #(
    parameter [3:0] DEFAULT_ADDR    = 4'd3,
    parameter [7:0] DEFAULT_HANDLER = 8'd1
) (
    input  wire        clk,
    input  wire        rst,

    // Device-bus to adb_modem.v (same protocol as adb_keyboard.v)
    input  wire        dev_cmd_valid,
    input  wire [3:0]  dev_cmd_addr,
    input  wire [2:0]  dev_cmd_op,
    output reg         dev_resp_valid,
    output reg         dev_resp_empty,
    output reg  [7:0]  dev_resp_b0,
    output reg  [7:0]  dev_resp_b1,
    input  wire        dev_listen_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [7:0]  dev_listen_b0,
    input  wire [7:0]  dev_listen_b1,
    /* verilator lint_on UNUSEDSIGNAL */

    output wire        dev_srq,

    // MMIO injection
    input  wire        inj_btn_valid,
    input  wire        inj_btn_state,    // 1 = pressed
    input  wire        inj_dx_valid,
    input  wire signed [7:0] inj_dx,
    input  wire        inj_dy_valid,
    input  wire signed [7:0] inj_dy,
    output wire [7:0]  inj_status
);

    localparam [2:0] OP_RESET     = 3'd0;
    localparam [2:0] OP_FLUSH     = 3'd1;
    localparam [2:0] OP_LISTEN_R0 = 3'd2;
    localparam [2:0] OP_LISTEN_R3 = 3'd3;
    localparam [2:0] OP_TALK_R0   = 3'd4;
    localparam [2:0] OP_TALK_R3   = 3'd5;

    reg [3:0]  cur_addr;
    reg [7:0]  cur_handler;
    reg        srq_enable;
    reg        button_pressed;
    reg signed [7:0] acc_dx;
    reg signed [7:0] acc_dy;
    reg              event_pending;

    // Saturate to ±63 (7-bit signed range) for the next TALK report.
    function signed [6:0] sat7;
        input signed [7:0] v;
        begin
            if (v > 8'sd63)       sat7 = 7'sd63;
            else if (v < -8'sd64) sat7 = -7'sd64;
            else                  sat7 = v[6:0];
        end
    endfunction

    // Saturating add helper (avoids 8-bit wrap when many small deltas
    // pile up between reports).
    function signed [7:0] sat_add;
        input signed [7:0] a;
        input signed [7:0] b;
        begin
            // Compute on a 9-bit signed extension; clamp to ±63.
            if ($signed({a[7], a}) + $signed({b[7], b}) > 9'sd63)
                sat_add = 8'sd63;
            else if ($signed({a[7], a}) + $signed({b[7], b}) < -9'sd64)
                sat_add = -8'sd64;
            else
                sat_add = a + b;
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            cur_addr       <= DEFAULT_ADDR;
            cur_handler    <= DEFAULT_HANDLER;
            srq_enable     <= 1'b1;
            button_pressed <= 1'b0;
            acc_dx         <= 8'sd0;
            acc_dy         <= 8'sd0;
            event_pending  <= 1'b0;
            dev_resp_valid <= 1'b0;
            dev_resp_empty <= 1'b0;
            dev_resp_b0    <= 8'hFF;
            dev_resp_b1    <= 8'hFF;
        end else begin
            dev_resp_valid <= 1'b0;
            dev_resp_empty <= 1'b0;

            // ── MMIO injection ──────────────────────────────────────
            if (inj_btn_valid) begin
                button_pressed <= inj_btn_state;
                event_pending  <= 1'b1;
            end
            if (inj_dx_valid) begin
                acc_dx        <= sat_add(acc_dx, inj_dx);
                event_pending <= 1'b1;
            end
            if (inj_dy_valid) begin
                acc_dy        <= sat_add(acc_dy, inj_dy);
                event_pending <= 1'b1;
            end

            // ── Device-bus command ──────────────────────────────────
            if (dev_cmd_valid && (dev_cmd_addr == cur_addr)) begin
                case (dev_cmd_op)
                    OP_RESET: begin
                        cur_addr       <= DEFAULT_ADDR;
                        cur_handler    <= DEFAULT_HANDLER;
                        srq_enable     <= 1'b1;
                        button_pressed <= 1'b0;
                        acc_dx         <= 8'sd0;
                        acc_dy         <= 8'sd0;
                        event_pending  <= 1'b0;
                    end
                    OP_FLUSH: begin
                        acc_dx         <= 8'sd0;
                        acc_dy         <= 8'sd0;
                        event_pending  <= 1'b0;
                    end
                    OP_TALK_R0: begin
                        if (!event_pending) begin
                            dev_resp_empty <= 1'b1;
                        end else begin
                            dev_resp_valid <= 1'b1;
                            // byte0: bit 7 = ~button (0 = pressed).
                            dev_resp_b0 <= {~button_pressed, sat7(acc_dy)};
                            // byte1: bit 7 = 1 (no secondary button on a
                            // single-button mouse), bits 6..0 = X delta.
                            dev_resp_b1 <= {1'b1, sat7(acc_dx)};
                            // Consume: zero deltas; keep button_pressed
                            // (level signal) but clear pending unless the
                            // button is still down — Mac OS expects to keep
                            // seeing reports while held, but we only
                            // generate a fresh "pending" on each MMIO event.
                            acc_dx        <= 8'sd0;
                            acc_dy        <= 8'sd0;
                            event_pending <= 1'b0;
                        end
                    end
                    OP_TALK_R3: begin
                        dev_resp_valid <= 1'b1;
                        dev_resp_b0    <= {srq_enable, 3'b000, cur_addr};
                        dev_resp_b1    <= cur_handler;
                    end
                    default: ;
                endcase
            end

            // 2026-07-25: gated on OP_LISTEN_R3 — see the matching comment
            // in adb_keyboard.v.  adb_phy.v decodes LISTEN R1/R2 into
            // OP_LISTEN_R0, whose payload is device data rather than an
            // address/handler pair, so treating it as a relocate would
            // scramble cur_addr.
            if (dev_listen_valid && (dev_cmd_addr == cur_addr) &&
                (dev_cmd_op == OP_LISTEN_R3)) begin
                // srq_enable is deliberately NOT written from R3 byte0
                // bit 7 — see the matching note in adb_keyboard.v (byte0
                // 0x00 is a "change nothing" sentinel here, so its bit 7
                // isn't real data, and leaving SRQ enabled is what keeps
                // injected events deliverable).
                if (dev_listen_b1 != 8'hFE) begin
                    cur_handler <= dev_listen_b1;
                end
                if (dev_listen_b0[7:4] == 4'hE) begin
                    cur_addr <= dev_listen_b0[3:0];
                end else if (dev_listen_b0[3:0] != 4'h0) begin
                    cur_addr <= dev_listen_b0[3:0];
                end
            end
        end
    end

    assign dev_srq    = srq_enable && event_pending;
    assign inj_status = {7'h00, event_pending};

endmodule

`default_nettype wire
