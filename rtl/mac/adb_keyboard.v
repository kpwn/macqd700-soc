// adb_keyboard.v — ADB keyboard model (default address 2, handler ID 1)
//
// Bridges to adb_modem.v via a tiny 4-signal device-bus interface.  Stores
// keycodes in a 16-entry FIFO that the host fills via an MMIO injection
// register (see fpga_top_peripherals.vh — the adb_inject window).
//
// ADB protocol (Apple Desktop Bus Specification, 1985):
//   - Default address = 2 (ADB_KEYBOARD_DEFAULT_ADDR).
//   - Handler ID = 1 (standard ANSI keyboard, M0115/M3501-class).
//   - Register 0: 2 bytes returned on TALK.
//       byte0 = first key event (bit 7 = release, bits 6..0 = key code).
//       byte1 = second key event (or 0xFF if only one).
//     Reference: Inside Macintosh: Devices, ch. 5; macadb.cpp emits
//     `0x80 | code` for release, `code` (with bit 7 clear) for down.
//   - Register 1, 2: optional / device-specific.  Stubbed to 0xFFFF.
//   - Register 3: high byte = (SRQ_enable<<6) | (exceptional<<5) | addr;
//                low byte = handler ID.
//
// Service request (SRQ): when the FIFO is non-empty, we assert
// `dev_srq` so the modem knows to poll register 0.  Cleared once we have
// successfully delivered both bytes to the modem.
//
// Device-bus protocol (mirrored on adb_mouse.v):
//   1. Modem raises `dev_cmd_valid` for one cycle alongside
//      `dev_cmd_addr` (4-bit ADB address) and `dev_cmd_op` (3-bit op):
//         3'b000 = RESET     — clear FIFO, restore default addr/handler.
//         3'b001 = FLUSH     — clear FIFO only.
//         3'b010 = LISTEN R0 — modem is about to send 2 data bytes.
//         3'b011 = LISTEN R3 — modem is about to send 2 data bytes.
//         3'b100 = TALK   R0 — produce 2 data bytes, MSB first.
//         3'b101 = TALK   R3 — produce 2 data bytes (addr/handler).
//         (LISTEN R1/R2 and TALK R1/R2 ignored — return zero.)
//   2. For TALK ops: device drives `dev_resp_valid` for one cycle when
//      the 2 response bytes are ready; modem latches them off
//      `dev_resp_b0` / `dev_resp_b1`.  If the device has no data
//      (FIFO empty for kbd), it asserts `dev_resp_empty` instead, and
//      the modem reports "no service" up to the host.
//   3. For LISTEN ops: modem then issues `dev_listen_valid` for one
//      cycle with both bytes on `dev_listen_b0`/`dev_listen_b1`.
//      The device latches them.
//
// MMIO injection face:
//   - `inj_kc_valid` pulses when the host writes the keyboard inject
//     register; `inj_kc_byte` carries the 8-bit ADB keycode (bit 7 =
//     release).  We push it onto the FIFO (oldest discarded if full).
//   - `inj_status` exposes {full, almost_full, ..., not_empty}.

`default_nettype none

module adb_keyboard #(
    parameter [3:0] DEFAULT_ADDR    = 4'd2,
    parameter [7:0] DEFAULT_HANDLER = 8'd1,
    parameter integer FIFO_LOG2     = 4   // 16 entries
) (
    input  wire        clk,
    input  wire        rst,

    // Device-bus to adb_modem.v
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

    // Service request — high while FIFO has un-fetched events.
    output wire        dev_srq,

    // MMIO injection
    input  wire        inj_kc_valid,
    input  wire [7:0]  inj_kc_byte,
    output wire [7:0]  inj_status
);

    // ── Operation encoding (must match adb_modem.v) ─────────────────────
    localparam [2:0] OP_RESET     = 3'd0;
    localparam [2:0] OP_FLUSH     = 3'd1;
    localparam [2:0] OP_LISTEN_R0 = 3'd2;
    localparam [2:0] OP_LISTEN_R3 = 3'd3;
    localparam [2:0] OP_TALK_R0   = 3'd4;
    localparam [2:0] OP_TALK_R3   = 3'd5;

    // ── Device state ─────────────────────────────────────────────────────
    reg [3:0]  cur_addr;
    reg [7:0]  cur_handler;
    reg        srq_enable;

    // FIFO of pending keycodes.  Two bytes per ADB report — we always
    // emit 2 events, padding the second slot with 0xFF (no key) when
    // only one is available.
    localparam integer FIFO_DEPTH = (1 << FIFO_LOG2);
    reg [7:0] fifo[0:FIFO_DEPTH-1];
    reg [FIFO_LOG2:0] fifo_head;   // points to next-out
    reg [FIFO_LOG2:0] fifo_tail;   // points to next-in
    wire [FIFO_LOG2:0] fifo_count = fifo_tail - fifo_head;
    wire fifo_empty = (fifo_count == 0);
    wire fifo_full  = (fifo_count == FIFO_DEPTH[FIFO_LOG2:0]);

    // Combinational peek of next 1-2 bytes (independent of registered
    // updates this cycle).
    wire [FIFO_LOG2-1:0] peek_idx0 = fifo_head[FIFO_LOG2-1:0];
    wire [FIFO_LOG2-1:0] peek_idx1 = fifo_head[FIFO_LOG2-1:0] + {{(FIFO_LOG2-1){1'b0}}, 1'b1};
    wire [7:0] peek0 = fifo[peek_idx0];
    wire [7:0] peek1 = fifo[peek_idx1];

    // ── FIFO writes (injection) and reads (TALK R0) ─────────────────────
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            cur_addr        <= DEFAULT_ADDR;
            cur_handler     <= DEFAULT_HANDLER;
            srq_enable      <= 1'b1;
            fifo_head       <= {(FIFO_LOG2+1){1'b0}};
            fifo_tail       <= {(FIFO_LOG2+1){1'b0}};
            for (i = 0; i < FIFO_DEPTH; i = i + 1)
                fifo[i] <= 8'h00;
            dev_resp_valid  <= 1'b0;
            dev_resp_empty  <= 1'b0;
            dev_resp_b0     <= 8'hFF;
            dev_resp_b1     <= 8'hFF;
        end else begin
            // Default one-cycle pulses
            dev_resp_valid <= 1'b0;
            dev_resp_empty <= 1'b0;

            // ── MMIO injection: append to FIFO ──────────────────────
            if (inj_kc_valid && !fifo_full) begin
                fifo[fifo_tail[FIFO_LOG2-1:0]] <= inj_kc_byte;
                fifo_tail <= fifo_tail + {{FIFO_LOG2{1'b0}}, 1'b1};
            end

            // ── Device-bus command ──────────────────────────────────
            if (dev_cmd_valid && (dev_cmd_addr == cur_addr)) begin
                case (dev_cmd_op)
                    OP_RESET: begin
                        cur_addr     <= DEFAULT_ADDR;
                        cur_handler  <= DEFAULT_HANDLER;
                        srq_enable   <= 1'b1;
                        fifo_head    <= fifo_tail;  // drain
                    end
                    OP_FLUSH: begin
                        fifo_head    <= fifo_tail;  // drain
                    end
                    OP_TALK_R0: begin
                        if (fifo_empty) begin
                            dev_resp_empty <= 1'b1;
                        end else begin
                            dev_resp_valid <= 1'b1;
                            dev_resp_b0    <= peek0;
                            // Second byte: pop second event if present, else
                            // 0xFF (no second key — Apple convention).
                            if (fifo_count >= 'd2) begin
                                dev_resp_b1 <= peek1;
                                fifo_head   <= fifo_head + {{(FIFO_LOG2-1){1'b0}}, 2'd2};
                            end else begin
                                dev_resp_b1 <= 8'hFF;
                                fifo_head   <= fifo_head + {{FIFO_LOG2{1'b0}}, 1'b1};
                            end
                        end
                    end
                    OP_TALK_R3: begin
                        // ADB register 3 layout (per ADB spec; see
                        // Inside Macintosh: Devices, ch.5):
                        //   byte0 bit 7    = SRQ enable
                        //         bit 6    = exceptional event (0 here)
                        //         bits 5-4 = reserved (0)
                        //         bits 3-0 = device address
                        //   byte1          = handler ID
                        dev_resp_valid <= 1'b1;
                        dev_resp_b0    <= {srq_enable, 3'b000, cur_addr};
                        dev_resp_b1    <= cur_handler;
                    end
                    OP_LISTEN_R3: begin
                        // Wait for the 2-byte payload from the modem.
                    end
                    default: ;
                endcase
            end

            // ── LISTEN R3 payload ──────────────────────────────────
            // Encoding (host → device):
            //   byte0 bits 3:0  = new address (if meta byte == 0xFE = "set
            //                     address only"; if meta == 0x00 = no-op).
            //   byte1           = handler ID (0xFE means leave handler).
            // Real ADB has more nuances (collision arbitration via 0xFD/0xFE)
            // but for a single-keyboard config we just take the new addr.
            // Gated on dev_cmd_addr == cur_addr so other devices on the
            // shared listen bus don't accidentally latch our payload.
            // 2026-07-25: gated on OP_LISTEN_R3.  adb_phy.v only started
            // delivering LISTEN payloads at all in this same change; it
            // decodes R1/R2 into OP_LISTEN_R0, so without this gate an
            // R0/R1/R2 payload (which carries device data, NOT an
            // address/handler pair) would be misread as a relocate and
            // scramble cur_addr.
            if (dev_listen_valid && (dev_cmd_addr == cur_addr) &&
                (dev_cmd_op == OP_LISTEN_R3)) begin
                // NOTE: R3 byte0 bit 7 is architecturally the host's
                // SRQ-enable for this device, but we deliberately do NOT
                // write srq_enable from it.  This codebase treats a byte0
                // of 0x00 as a "change nothing" sentinel (see the address
                // logic below, and tb_adb.cpp's
                // test_listen_then_talk_handler, which sets a handler with
                // byte0=0x00 and expects SRQ to stay enabled) — so bit 7
                // of that sentinel is not real data.  Leaving SRQ always
                // enabled also keeps injected events deliverable, which is
                // the behaviour the Q700 boot path actually needs.
                if (dev_listen_b1 != 8'hFE) begin
                    cur_handler <= dev_listen_b1;
                end
                if (dev_listen_b0[7:4] == 4'hE) begin
                    // Apple-spec "change address only" code 0xE.
                    cur_addr <= dev_listen_b0[3:0];
                end else if (dev_listen_b0[3:0] != 4'h0) begin
                    cur_addr <= dev_listen_b0[3:0];
                end
            end
        end
    end

    assign dev_srq    = srq_enable && !fifo_empty;
    // inj_status: bit7=full, bits[6:1]=fifo count, bit0=not_empty.
    // fifo_count is FIFO_LOG2+1 bits wide (so it can express both 0 and
    // FIFO_DEPTH); we expose the LOW FIFO_LOG2 bits in the middle nibble.
    wire [5:0] count_pad = {{(6-FIFO_LOG2){1'b0}}, fifo_count[FIFO_LOG2-1:0]};
    assign inj_status = {fifo_full, count_pad, !fifo_empty};

endmodule

`default_nettype wire
