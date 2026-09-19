// MAME reference/excerpt/adaptation attribution: Copyright R. Belmont.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// adb_modem.v — ADB modem (byte-level bridge between VIA1's shift
// register and a small set of ADB bus devices).
//
// The real Quadra 700 ADB modem (MAME `adbmodem_device`, MAME source
// `src/mame/apple/adbmodem.cpp`) is a PIC16C57 microcontroller running
// a small firmware that performs ADB bit-level signaling on a single
// open-drain line and bridges to VIA1 via CB1/CB2 at the bit level.  We
// do NOT model the PIC, the bit-level ADB physical layer, or the CB1/CB2
// shift handshake — VIA1's `adb_rx_byte/valid/ready` and
// `adb_tx_byte/valid` ports already abstract the byte boundary, and the
// only thing the host's ADB Manager observes once SR-attention timing is
// taken care of is the byte-level command/response stream:
//
//     Host → modem:  command byte [+ optional 2-byte LISTEN payload]
//     Modem → host:  response bytes for TALK; SRQ notify byte unsolicited
//
// Command byte format (Apple Desktop Bus Specification, 1985 — see also
// macadb.cpp `adb_talk` decode `(m_command >> 4)`/`(m_command & 0xC)`/
// `(m_command & 3)`):
//     bits 7..4 = ADB device address (4 bits)
//     bits 3..2 = operation:
//         00 = SendReset (broadcast, address ignored) /
//              Flush (per-device, when low 2 bits == 01)
//         01 = reserved
//         10 = LISTEN (host writes 2 bytes to reg N)
//         11 = TALK   (host reads  2 bytes from reg N)
//     bits 1..0 = register number (0..3)
//
// Special-case bytes called out by the project task description:
//     0x00 = SendReset / NOP wakeup (broadcast to addr 0)
//     0x01 = Flush (low bits 01) at addr 0 — collapses to flush family
//     0x03 = per-device flush (addr in upper nibble; here addr=0)
//
// State machine:
//
//   IDLE → [host TX command]                   → DECODE
//   DECODE
//     RESET / FLUSH:  broadcast cmd; ack byte  → SEND_BYTE (0x00) → DELAY
//     LISTEN R0/R3:                            → LISTEN_D0 → LISTEN_D1 →
//                                                LISTEN_DONE → SEND_BYTE
//                                                (0x00) → DELAY
//     TALK   R0/R3:   broadcast cmd            → TALK_WAIT → either:
//                                                  - SEND_BYTE(b0) →
//                                                    SEND_BYTE(b1) → DELAY
//                                                  - SEND_BYTE(0xFF) →
//                                                    DELAY
//   SEND_BYTE: hold via_rx_byte / via_rx_valid; transition to next_state
//              once via_rx_ready pulses (host has accepted).
//   DELAY:    park briefly before accepting the next host TX.
//
// adb_irq_pending output:
//   - level held while any device asserts dev_srq AND no transaction is
//     in flight, so the host's poll loop sees the pending notification.

`default_nettype none

module adb_modem #(
    parameter integer NUM_DEVICES = 2,
    // RX retry interval — when the host TALKs an empty register, we want
    // to defer the "no service" reply by a few phi2 ticks rather than
    // bursting empties back to back.  Default 16 ticks ≈ 16 µs at the
    // 783 kHz NCO rate.
    parameter integer NO_SERVICE_DELAY = 16
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        phi2_tick,

    // ── VIA1 byte-level shift-register interface ─────────────────────
    output reg  [7:0]  via_rx_byte,
    output reg         via_rx_valid,
    input  wire        via_rx_ready,
    input  wire [7:0]  via_tx_byte,
    input  wire        via_tx_valid,

    // ── Active-high IRQ pending (platform inverts to ORB[3]) ─────────
    output wire        adb_irq_pending,

    // ── Device-bus fan-out ───────────────────────────────────────────
    output reg         dev_cmd_valid,
    output reg  [3:0]  dev_cmd_addr,
    output reg  [2:0]  dev_cmd_op,
    output reg         dev_listen_valid,
    output reg  [7:0]  dev_listen_b0,
    output reg  [7:0]  dev_listen_b1,

    input  wire [NUM_DEVICES-1:0]   dev_resp_valid,
    input  wire [NUM_DEVICES-1:0]   dev_resp_empty,
    input  wire [NUM_DEVICES*8-1:0] dev_resp_b0,
    input  wire [NUM_DEVICES*8-1:0] dev_resp_b1,
    input  wire [NUM_DEVICES-1:0]   dev_srq,

    // Debug observability — current FSM state
    output wire [3:0]  dbg_state
);

    // ── ADB command-byte decode helpers ──────────────────────────────
    function [2:0] decode_op;
        input [7:0] cmd;
        begin
            // op[3:2] selects family; op[1:0] selects register / sub-op.
            if (cmd[3:2] == 2'b00) begin
                if (cmd[1:0] == 2'b00) decode_op = 3'd0;       // OP_RESET
                else                   decode_op = 3'd1;       // OP_FLUSH
            end
            else if (cmd[3:2] == 2'b10) begin
                if (cmd[1:0] == 2'b11) decode_op = 3'd3;       // OP_LISTEN_R3
                else                   decode_op = 3'd2;       // OP_LISTEN_R0
            end
            else if (cmd[3:2] == 2'b11) begin
                if (cmd[1:0] == 2'b11) decode_op = 3'd5;       // OP_TALK_R3
                else                   decode_op = 3'd4;       // OP_TALK_R0
            end
            else begin
                decode_op = 3'd1;  // reserved → treat as FLUSH (no-op)
            end
        end
    endfunction

    localparam [2:0] OP_RESET     = 3'd0;
    localparam [2:0] OP_FLUSH     = 3'd1;
    localparam [2:0] OP_LISTEN_R0 = 3'd2;
    localparam [2:0] OP_LISTEN_R3 = 3'd3;
    localparam [2:0] OP_TALK_R0   = 3'd4;
    localparam [2:0] OP_TALK_R3   = 3'd5;

    // ── State machine ────────────────────────────────────────────────
    localparam [3:0] S_IDLE        = 4'd0;
    localparam [3:0] S_DECODE      = 4'd1;
    localparam [3:0] S_LISTEN_D0   = 4'd2;
    localparam [3:0] S_LISTEN_D1   = 4'd3;
    localparam [3:0] S_LISTEN_DONE = 4'd4;
    localparam [3:0] S_TALK_WAIT   = 4'd5;
    localparam [3:0] S_SEND_BYTE   = 4'd6;
    localparam [3:0] S_TALK_LO     = 4'd7;
    localparam [3:0] S_DELAY       = 4'd8;

    reg [3:0]  state;
    reg [3:0]  next_after_send;
    reg [7:0]  cmd_byte;
    reg [3:0]  cmd_addr;
    reg [2:0]  cmd_op;
    reg [7:0]  listen_b0;
    reg [7:0]  listen_b1;
    reg [7:0]  reply_b0;
    reg [7:0]  reply_b1;
    reg [15:0] delay_cnt;
    reg        irq_level;

    // OR-reduce response bytes from the responding device.  At most one
    // device matches the addr, so this is an effectively-priority mux.
    integer di;
    reg [7:0] sel_b0;
    reg [7:0] sel_b1;
    always @(*) begin
        sel_b0 = 8'h00;
        sel_b1 = 8'h00;
        for (di = 0; di < NUM_DEVICES; di = di + 1) begin
            if (dev_resp_valid[di]) begin
                sel_b0 = dev_resp_b0[di*8 +: 8];
                sel_b1 = dev_resp_b1[di*8 +: 8];
            end
        end
    end

    wire any_resp_valid = |dev_resp_valid;
    wire any_resp_empty = |dev_resp_empty;

    always @(posedge clk) begin
        if (rst) begin
            state            <= S_IDLE;
            next_after_send  <= S_IDLE;
            cmd_byte         <= 8'h00;
            cmd_addr         <= 4'h0;
            cmd_op           <= 3'd0;
            listen_b0        <= 8'h00;
            listen_b1        <= 8'h00;
            reply_b0         <= 8'h00;
            reply_b1         <= 8'h00;
            via_rx_byte      <= 8'h00;
            via_rx_valid     <= 1'b0;
            dev_cmd_valid    <= 1'b0;
            dev_cmd_addr     <= 4'h0;
            dev_cmd_op       <= 3'd0;
            dev_listen_valid <= 1'b0;
            dev_listen_b0    <= 8'h00;
            dev_listen_b1    <= 8'h00;
            delay_cnt        <= 16'd0;
            irq_level        <= 1'b0;
        end else begin
            // Default one-cycle pulses for cmd/listen valid.
            dev_cmd_valid    <= 1'b0;
            dev_listen_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (via_tx_valid) begin
                        cmd_byte <= via_tx_byte;
                        cmd_addr <= via_tx_byte[7:4];
                        cmd_op   <= decode_op(via_tx_byte);
                        state    <= S_DECODE;
                    end
                end

                S_DECODE: begin
                    case (cmd_op)
                        OP_RESET, OP_FLUSH: begin
                            dev_cmd_valid <= 1'b1;
                            dev_cmd_addr  <= cmd_addr;
                            dev_cmd_op    <= cmd_op;
                            // Send a 0x00 ack byte to the host.
                            via_rx_byte   <= 8'h00;
                            via_rx_valid  <= 1'b1;
                            irq_level     <= 1'b1;
                            next_after_send <= S_DELAY;
                            state         <= S_SEND_BYTE;
                        end
                        OP_LISTEN_R0, OP_LISTEN_R3: begin
                            state <= S_LISTEN_D0;
                        end
                        OP_TALK_R0, OP_TALK_R3: begin
                            dev_cmd_valid <= 1'b1;
                            dev_cmd_addr  <= cmd_addr;
                            dev_cmd_op    <= cmd_op;
                            // Wait one cycle for the device's registered
                            // response to settle, then sample.
                            state <= S_TALK_WAIT;
                        end
                        default: state <= S_IDLE;
                    endcase
                end

                S_LISTEN_D0: begin
                    if (via_tx_valid) begin
                        listen_b0 <= via_tx_byte;
                        state     <= S_LISTEN_D1;
                    end
                end
                S_LISTEN_D1: begin
                    if (via_tx_valid) begin
                        listen_b1 <= via_tx_byte;
                        state     <= S_LISTEN_DONE;
                    end
                end
                S_LISTEN_DONE: begin
                    dev_cmd_valid    <= 1'b1;
                    dev_cmd_addr     <= cmd_addr;
                    dev_cmd_op       <= cmd_op;
                    dev_listen_valid <= 1'b1;
                    dev_listen_b0    <= listen_b0;
                    dev_listen_b1    <= listen_b1;
                    via_rx_byte      <= 8'h00;
                    via_rx_valid     <= 1'b1;
                    irq_level        <= 1'b1;
                    next_after_send  <= S_DELAY;
                    state            <= S_SEND_BYTE;
                end

                S_TALK_WAIT: begin
                    // Wait for the addressed device to either ack with
                    // data (any_resp_valid) or signal "empty" (any_resp_
                    // empty).  Devices register their reply one cycle
                    // after dev_cmd_valid; if no device matches the
                    // address, neither line ever rises and we time out
                    // after delay_cnt phi2 ticks.
                    if (any_resp_valid) begin
                        reply_b0        <= sel_b0;
                        reply_b1        <= sel_b1;
                        via_rx_byte     <= sel_b0;
                        via_rx_valid    <= 1'b1;
                        irq_level       <= 1'b1;
                        next_after_send <= S_TALK_LO;
                        state           <= S_SEND_BYTE;
                        delay_cnt       <= 16'd0;
                    end else if (any_resp_empty) begin
                        // Device matched but had no data — single 0xFF
                        // so the host's poll loop sees the SR-attention
                        // IRQ and reads "no service".
                        via_rx_byte     <= 8'hFF;
                        via_rx_valid    <= 1'b1;
                        irq_level       <= 1'b1;
                        next_after_send <= S_DELAY;
                        state           <= S_SEND_BYTE;
                        delay_cnt       <= 16'd0;
                    end else if (phi2_tick) begin
                        // Time-out for unaddressed targets — return
                        // 0xFF after a short window.
                        if (delay_cnt >= NO_SERVICE_DELAY[15:0] - 16'd1) begin
                            via_rx_byte     <= 8'hFF;
                            via_rx_valid    <= 1'b1;
                            irq_level       <= 1'b1;
                            next_after_send <= S_DELAY;
                            state           <= S_SEND_BYTE;
                            delay_cnt       <= 16'd0;
                        end else begin
                            delay_cnt <= delay_cnt + 16'd1;
                        end
                    end
                end

                S_SEND_BYTE: begin
                    // Hold via_rx_valid until VIA1 acknowledges by
                    // pulsing via_rx_ready (phi2-gated).  When it does,
                    // drop valid and advance.
                    if (via_rx_valid && via_rx_ready) begin
                        via_rx_valid    <= 1'b0;
                        delay_cnt       <= 16'd0;
                        state           <= next_after_send;
                    end
                end

                S_TALK_LO: begin
                    via_rx_byte     <= reply_b1;
                    via_rx_valid    <= 1'b1;
                    next_after_send <= S_DELAY;
                    state           <= S_SEND_BYTE;
                end

                S_DELAY: begin
                    if (phi2_tick) begin
                        if (delay_cnt >= NO_SERVICE_DELAY[15:0] - 16'd1) begin
                            delay_cnt <= 16'd0;
                            irq_level <= 1'b0;
                            state     <= S_IDLE;
                        end else begin
                            delay_cnt <= delay_cnt + 16'd1;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    wire any_srq = |dev_srq;
    assign adb_irq_pending = irq_level || (any_srq && (state == S_IDLE));
    assign dbg_state       = state;

    // Discard unused command-byte field (kept for debug/wave).
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_cmd_byte = |cmd_byte;
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
