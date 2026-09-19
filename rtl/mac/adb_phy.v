// adb_phy.v - ADB bus PHY for the Q700 ADB PIC
//
// Bit-level ADB electrical model bridging the real PIC1654S firmware
// (rtl/mac/adb_pic_modem.v + pic16c5x.v) to the byte-level device
// register models in rtl/mac/adb_keyboard.v / rtl/mac/adb_mouse.v.
//
// History: this module started as a "no-device" bus watcher — it
// framed the host's Attention/command-byte sequence but never
// answered.  That left the firmware's post-SendReset TALK polling
// permanently unanswered (see project_via1_lockstep_deepdive.md).  It
// then became a bit-level responder (commit d092d97) with SRQ support
// (commit 5a9f453) — but its host-frame receiver was only ever
// validated against tb_adb_phy.cpp's SYNTHETIC host model, whose
// timing mirrored this module's own constants instead of the real
// firmware's.  The real 342S0440-B firmware produces a very different
// frame (2026-07-21 root cause of the "injected ADB mouse event never
// drains / boot stalls spinning on VIA1 SR reads" hardware bug):
//
//   * The PIC core retires one instruction per phi2_tick (783.36 kHz)
//     and this module counts the same ticks, so every firmware-timed
//     interval measures as its INSTRUCTION-CYCLE count here — roughly
//     half the old constants' nominal "microseconds" (the real
//     PIC1654S ran at 500 kHz / 2us per cycle).
//   * Measured frame (tb_adb_pic_phy, real firmware in the loop):
//     attention low ~365 ticks (old ATTN_MIN_US=500 rejected EVERY
//     real frame!), sync is a ~30-tick HIGH gap (the old FSM expected
//     a 50-120 LOW pulse the firmware never produces), bit cells are
//     ~46 ticks self-clocked by their falling edges ('1' = low ~16,
//     '0' = low ~30; the old fixed 100-tick free-running sampler
//     could never track them), and the stop bit (low ~30) follows the
//     8th bit immediately (the old FSM waited a 200-tick Tlt BEFORE
//     looking for a >=70-tick stop).
//
// This version's receiver is therefore SELF-CLOCKING: each bit cell is
// framed by its own falling edge and classified by low-pulse width,
// exactly how real ADB devices (and MAME's macadb) decode the line.
// It accepts the real firmware's frames and remains robust to modest
// retimings.
//
// The transmit (response) side is scaled to what the firmware's
// AdbRecvBit routine (see the 342S0440-B disassembly, commented port at
// github.com/lampmerchant/macseadb88) actually samples: ~50-tick cells,
// '0' low ~32 / '1' low ~17, response starting ~60 ticks after the
// host's stop-bit release — inside the firmware's ~26-round
// wait-for-start window (~10..130 cycles after release).
//
// Service Request (SRQ): a device wanting service stretches the
// COMMAND STOP BIT low.  The firmware samples this in its bounded
// post-stop release-wait loop (lb_065/SAdCmd0: 26 rounds, sets F_SRQ
// on any low sample) — this is the ONLY window it ever samples SRQ in,
// so the hold must overlap the stop bit itself and extend ~90 ticks
// past the host's release (long enough to be unmistakable, short
// enough that the loop exits by line-release and the firmware's
// response-start window stays open for a same-transaction response).
// On F_SRQ (or on autopoll data), the firmware notifies the 68k by
// asserting RB4/INT and clocking the stashed command byte into the
// VIA1 shift register — that unsolicited SR byte is what the 68k's
// ADB Manager blocks on.
//
// The firmware ALSO autopolls autonomously: while the VIA state pins
// sit in S3 (idle) it re-issues the last TALK command (stashed in
// GPR14) every other CDOWNA countdown expiry (~every few thousand
// cycles).  Mac OS never re-polls on its own — it relies entirely on
// this PIC autopoll + SRQ + SR-notification chain, which is why the
// old receiver's rejection of every firmware frame wedged the boot.
//
// LISTEN payload capture (2026-07-25).  Previously a scope cut, and the
// root cause of "ADB keyboard/mouse have never once worked on real
// hardware": after a LISTEN command's stop bit the host transmits a
// data frame (start bit + 16 data bits + stop bit) in the SAME
// self-clocked bit-cell encoding as the command byte.  That frame used
// to be dropped entirely — dev_listen_valid was wired but never
// asserted — so devices could never leave their default addresses
// (kbd 2, mouse 3).  That breaks Mac OS's ADB Manager startup outright:
// ADBReInit does not merely poll the defaults, it RELOCATES each device
// via LISTEN R3 to hand out unique addresses and detect collisions.
// With the payload dropped, the OS's device table ends up naming
// addresses our devices never moved to, so every subsequent poll /
// firmware autopoll is sent to a dead address and nothing ever answers
// (observed on real HW: the last dispatched command was TALK R0 to
// address 1, where no device lives).  ST_LSN_GAP / ST_LSN_LOW below now
// receive that payload using the same falling-edge framing and
// low-pulse-width classification the command receiver already uses, and
// pulse dev_listen_valid with the two captured bytes.
//
// NOT implemented (remaining documented scope cuts):
//   - decode_op collapses LISTEN R1/R2 into OP_LISTEN_R0, so e.g. a
//     keyboard LED write (LISTEN R2) arrives tagged as R0.  The device
//     models gate their R3 address/handler logic on the op, so this is
//     inert rather than corrupting — but a device needing real R1/R2
//     semantics would need the op encoding widened first.
//   - R3 handler-ID special values beyond 0xFE ("change address only,
//     keep handler", which the device models do honour): 0x00 / 0xFD /
//     0xFF self-test and activator variants are treated as ordinary
//     handler IDs.  Not exercised by the Q700 boot path.
//   - The ADB reset pulse (a single long low, no command byte —
//     what the firmware emits for a SendReset) is detected by length
//     and dispatched as OP_RESET with addr 0.  The device models match
//     on their own current address only, so a relocated device does not
//     see it; with LISTEN now live this is a real (if narrow) gap where
//     it used to be a no-op, since devices CAN now hold a non-default
//     address.  Broadcast-reset semantics would need an addr-agnostic
//     match in the device models.
//
// Electrical model (ticks == phi2_tick == one PIC instruction cycle):
//   Attention: host low >= ATTN_MIN_TICKS (firmware: ~365).
//   Reset:     host low >= RESET_MIN_TICKS (firmware: ~1800).
//   Sync:      high gap after attention (~30 ticks); no separate
//              framing — the first bit cell's falling edge starts it.
//   Data bit:  self-clocked; low-pulse width <= BIT1_MAX_TICKS is a
//              '1', longer is a '0'.
//   Stop:      one more low pulse right after bit 8 (host, ~30 low).
//   SRQ:       stop-bit low stretched SRQ_HOLD_TICKS past host release.
//   Response:  start bit ("0"-shaped) + 16 data bits (b0 then b1, MSB
//              first) + stop bit ("1"-shaped), 50-tick cells, driven
//              open-drain RESP_GAP_TICKS after stop release (or after
//              the SRQ hold ends).

module adb_phy #(
    parameter CLK_MHZ = 32
) (
    input  wire clk,
    input  wire rst,
    input  wire phi2_tick,
    input  wire pic_adb_out,
    output wire adb_in,
    output wire rtcc_in,

    // Device-bus fan-out to adb_keyboard.v / adb_mouse.v (same protocol
    // adb_modem.v uses — see adb_keyboard.v's header comment for the op
    // encoding: 0=RESET,1=FLUSH,2=LISTEN R0,3=LISTEN R3,4=TALK R0,5=TALK R3).
    output reg         dev_cmd_valid,
    output reg  [3:0]  dev_cmd_addr,
    output reg  [2:0]  dev_cmd_op,
    output reg         dev_listen_valid,
    output reg  [7:0]  dev_listen_b0,
    output reg  [7:0]  dev_listen_b1,

    // Per-device responses, 2-bit vectors: [0]=keyboard, [1]=mouse —
    // matches the {adb_ms_*, adb_kbd_*} concatenation convention used
    // for adb_modem.v at the fpga_top_peripherals.vh integration site.
    input  wire [1:0]  dev_resp_valid,
    input  wire [1:0]  dev_resp_empty,
    input  wire [15:0] dev_resp_b0,
    input  wire [15:0] dev_resp_b1,
    input  wire [1:0]  dev_srq
);
    localparam [3:0] ST_IDLE      = 4'd0;
    localparam [3:0] ST_ATTN      = 4'd1;
    localparam [3:0] ST_CMD_GAP   = 4'd2;
    localparam [3:0] ST_CMD_LOW   = 4'd3;
    localparam [3:0] ST_STOP_GAP  = 4'd4;
    localparam [3:0] ST_STOP_LOW  = 4'd5;
    localparam [3:0] ST_SRQ_HOLD  = 4'd6;
    localparam [3:0] ST_RESP_GAP  = 4'd7;
    localparam [3:0] ST_RESP_BITS = 4'd8;
    localparam [3:0] ST_RESP_DONE = 4'd9;
    localparam [3:0] ST_LSN_GAP   = 4'd10;
    localparam [3:0] ST_LSN_LOW   = 4'd11;

    // Receiver thresholds (ticks).  Firmware-measured values in
    // comments — see tb_adb_pic_phy.cpp.
    localparam [11:0] ATTN_MIN_TICKS  = 12'd250;   // firmware: ~365
    localparam [11:0] RESET_MIN_TICKS = 12'd1200;  // firmware: ~1800
    localparam [11:0] GAP_TIMEOUT     = 12'd200;   // sync/inter-bit high gap bound (fw: ~15-30)
    localparam [11:0] BIT1_MAX_TICKS  = 12'd22;    // fw '1': ~16 low, '0': ~30 low
    localparam [11:0] BIT_LOW_MAX     = 12'd80;    // malformed-bit abort guard
    localparam [11:0] STOP_LOW_MAX    = 12'd100;   // malformed-stop abort guard
    localparam [3:0]  CMD_BITS        = 4'd8;

    // Transmit (response) timing — scaled to the firmware's AdbRecvBit
    // sampling windows (one tick == one firmware instruction cycle).
    localparam [11:0] RESP_GAP_TICKS  = 12'd60;    // fw start window ~10..130 after stop release
    localparam [11:0] TX_CELL_TICKS   = 12'd50;
    localparam [11:0] TX_LOW_0_TICKS  = 12'd32;
    localparam [11:0] TX_LOW_1_TICKS  = 12'd17;
    localparam [4:0]  RESP_CELLS      = 5'd18;     // 1 start + 16 data + 1 stop

    // Service Request hold past the host's stop-bit release.  Must be
    // long enough that the firmware's 26-round release-wait loop
    // (~4-5 cycles/round) samples the line low at least once (it sets
    // F_SRQ on the FIRST low sample), and short enough that the loop
    // exits by line-release before the firmware's response-start
    // window closes.
    localparam [11:0] SRQ_HOLD_TICKS  = 12'd90;

    // LISTEN payload receive.  The host's stop-to-start turnaround (Tlt)
    // before it begins the payload frame is much longer than an
    // inter-bit gap (real ADB Tlt is 140-260us; at one tick per PIC
    // instruction cycle that lands around 70-130 ticks), so the payload's
    // FIRST cell gets its own generous bound rather than reusing
    // GAP_TIMEOUT.  It is still bounded: a host that sends no payload
    // (or a command we mis-decoded as LISTEN) falls back to ST_IDLE
    // instead of parking the receiver forever.
    localparam [11:0] LSN_START_MAX   = 12'd400;
    localparam [4:0]  LSN_CELLS       = 5'd18;     // 1 start + 16 data + 1 stop

    reg [3:0]  state;
    reg [11:0] low_cnt;
    reg [11:0] cell_cnt;
    reg [3:0]  bit_cnt;
    reg [7:0]  cmd_shift;
    reg        rtcc_toggle;
    reg        srq_active;     // any_srq latched at the stop bit's falling edge

    reg [2:0]  cmd_op_r;
    reg [15:0] resp_shift;
    reg [4:0]  resp_cell_idx;
    reg        resp_drive_low;

    // LISTEN payload receive state (see header).  listen_req is a level
    // raised by the phi2-gated FSM and turned into a one-fast-clk
    // dev_listen_valid pulse by the dispatch block below, exactly like
    // dispatch_req.
    reg [15:0] lsn_shift;
    reg [4:0]  lsn_cell_idx;
    reg        listen_req;
    reg        listen_req_d;

    // ── Device-bus dispatch: one clean fast-clk-cycle pulse ──────────
    // The phi2_tick-gated frame FSM below just raises `dispatch_req` (a
    // level) for at least one phi2_tick period when it wants to issue a
    // command; this always-posedge-clk block turns the RISING EDGE of
    // that level into a proper one-cycle dev_cmd_valid pulse —
    // adb_keyboard.v / adb_mouse.v sample their device-bus every
    // posedge clk (not phi2-gated), so a pulse held for a whole
    // phi2_tick period would re-trigger their TALK/FIFO-drain logic on
    // every intervening fast clk edge instead of once.  Mirrors
    // adb_modem.v's own (ungated) dispatch pattern.
    reg dispatch_req;
    reg dispatch_req_d;
    reg [3:0] dispatch_addr;
    reg [2:0] dispatch_op;

    reg        resp_latched_valid;
    reg        resp_latched_empty;
    reg [7:0]  resp_latched_b0;
    reg [7:0]  resp_latched_b1;
    reg        resp_consume;

    integer di;
    reg [7:0] sel_b0_r, sel_b1_r;
    always @(*) begin
        sel_b0_r = 8'h00;
        sel_b1_r = 8'h00;
        for (di = 0; di < 2; di = di + 1) begin
            if (dev_resp_valid[di]) begin
                sel_b0_r = dev_resp_b0[di*8 +: 8];
                sel_b1_r = dev_resp_b1[di*8 +: 8];
            end
        end
    end
    wire any_resp_valid = |dev_resp_valid;
    wire any_resp_empty = |dev_resp_empty;
    wire any_srq        = |dev_srq;

    always @(posedge clk) begin
        if (rst) begin
            dispatch_req_d     <= 1'b0;
            listen_req_d        <= 1'b0;
            dev_cmd_valid       <= 1'b0;
            dev_cmd_addr        <= 4'h0;
            dev_cmd_op          <= 3'd0;
            dev_listen_valid    <= 1'b0;
            dev_listen_b0       <= 8'h00;
            dev_listen_b1       <= 8'h00;
            resp_latched_valid  <= 1'b0;
            resp_latched_empty  <= 1'b0;
            resp_latched_b0     <= 8'h00;
            resp_latched_b1     <= 8'h00;
        end else begin
            dispatch_req_d   <= dispatch_req;
            listen_req_d     <= listen_req;
            dev_cmd_valid    <= 1'b0;
            dev_listen_valid <= 1'b0;

            // LISTEN payload complete.  dev_cmd_addr/dev_cmd_op still
            // hold this frame's own command (nothing rewrites them
            // between the command dispatch and here), so the device
            // models can match on address AND gate on the LISTEN
            // register, which is what makes an R3 relocate safe.
            if (listen_req && !listen_req_d) begin
                dev_listen_valid <= 1'b1;
                dev_listen_b0    <= lsn_shift[15:8];
                dev_listen_b1    <= lsn_shift[7:0];
            end

            if (dispatch_req && !dispatch_req_d) begin
                dev_cmd_valid <= 1'b1;
                dev_cmd_addr  <= dispatch_addr;
                dev_cmd_op    <= dispatch_op;
                // A fresh command invalidates any stale response latch
                // from an aborted earlier frame; the addressed device
                // re-answers within a couple of fast clks.
                resp_latched_valid <= 1'b0;
                resp_latched_empty <= 1'b0;
            end

            if (any_resp_valid) begin
                resp_latched_valid <= 1'b1;
                resp_latched_b0    <= sel_b0_r;
                resp_latched_b1    <= sel_b1_r;
            end
            if (any_resp_empty) begin
                resp_latched_empty <= 1'b1;
            end
            if (resp_consume) begin
                resp_latched_valid <= 1'b0;
                resp_latched_empty <= 1'b0;
            end
        end
    end

    // ── ADB command-byte decode (mirrors adb_modem.v's decode_op) ────
    function [2:0] decode_op;
        input [7:0] cmd;
        begin
            if (cmd[3:2] == 2'b00) begin
                if (cmd[1:0] == 2'b00) decode_op = 3'd0;   // RESET
                else                   decode_op = 3'd1;   // FLUSH
            end else if (cmd[3:2] == 2'b10) begin
                if (cmd[1:0] == 2'b11) decode_op = 3'd3;   // LISTEN R3
                else                   decode_op = 3'd2;   // LISTEN R0
            end else if (cmd[3:2] == 2'b11) begin
                if (cmd[1:0] == 2'b11) decode_op = 3'd5;   // TALK R3
                else                   decode_op = 3'd4;   // TALK R0
            end else begin
                decode_op = 3'd1;                          // reserved -> FLUSH
            end
        end
    endfunction
    localparam [2:0] OP_TALK_R0 = 3'd4;
    localparam [2:0] OP_TALK_R3 = 3'd5;

    localparam [2:0] OP_LSN_R0  = 3'd2;
    localparam [2:0] OP_LSN_R3  = 3'd3;

    wire is_talk   = (cmd_op_r == OP_TALK_R0) || (cmd_op_r == OP_TALK_R3);
    wire is_listen = (cmd_op_r == OP_LSN_R0)  || (cmd_op_r == OP_LSN_R3);

    // Open-drain wire-AND: whoever pulls low wins.  Command-framing
    // detection below keys off pic_adb_out (the host's own drive)
    // directly rather than this composite, so our own response drive
    // never aliases as host activity.
    assign adb_in  = pic_adb_out & ~resp_drive_low;
    assign rtcc_in = rtcc_toggle;

    // The received command byte with the final (in-flight) bit merged
    // in — used at the 8th bit's rising edge, when the last bit's value
    // is known but not yet registered.
    wire       cur_bit_is_one = (low_cnt <= BIT1_MAX_TICKS);
    wire [7:0] cmd_full       = {cmd_shift[6:0], cur_bit_is_one};

    always @(posedge clk) begin
        if (rst) begin
            state          <= ST_IDLE;
            low_cnt        <= 12'd0;
            cell_cnt       <= 12'd0;
            bit_cnt        <= 4'd0;
            cmd_shift      <= 8'h00;
            rtcc_toggle    <= 1'b0;
            srq_active     <= 1'b0;
            cmd_op_r       <= 3'd0;
            resp_shift     <= 16'h0000;
            resp_cell_idx  <= 5'd0;
            resp_drive_low <= 1'b0;
            dispatch_req   <= 1'b0;
            dispatch_addr  <= 4'h0;
            dispatch_op    <= 3'd0;
            resp_consume   <= 1'b0;
            lsn_shift      <= 16'h0000;
            lsn_cell_idx   <= 5'd0;
            listen_req     <= 1'b0;
        end else if (phi2_tick) begin
            rtcc_toggle  <= ~rtcc_toggle;
            resp_consume <= 1'b0;

            case (state)
                ST_IDLE: begin
                    low_cnt      <= 12'd0;
                    cell_cnt     <= 12'd0;
                    bit_cnt      <= 4'd0;
                    dispatch_req <= 1'b0;
                    listen_req   <= 1'b0;
                    if (!pic_adb_out) begin
                        state   <= ST_ATTN;
                        low_cnt <= 12'd1;
                    end
                end

                ST_ATTN: begin
                    if (!pic_adb_out) begin
                        if (low_cnt != 12'hFFF)
                            low_cnt <= low_cnt + 12'd1;
                    end else if (low_cnt >= RESET_MIN_TICKS) begin
                        // ADB reset pulse (firmware SendReset): one long
                        // low, no command byte.  Broadcast OP_RESET (see
                        // header scope-cut note) and return to idle.
                        dispatch_addr <= 4'h0;
                        dispatch_op   <= 3'd0;   // RESET
                        dispatch_req  <= 1'b1;
                        state         <= ST_IDLE;
                    end else if (low_cnt >= ATTN_MIN_TICKS) begin
                        state     <= ST_CMD_GAP;
                        cell_cnt  <= 12'd0;
                        bit_cnt   <= 4'd0;
                        cmd_shift <= 8'h00;
                    end else begin
                        state <= ST_IDLE;
                    end
                end

                // High gap before the next command bit (the ~30-tick
                // sync gap after attention, or the ~15-30-tick high
                // tail of the previous bit cell).  The next falling
                // edge starts a bit's low pulse.
                ST_CMD_GAP: begin
                    if (!pic_adb_out) begin
                        state    <= ST_CMD_LOW;
                        low_cnt  <= 12'd1;
                    end else if (cell_cnt >= GAP_TIMEOUT) begin
                        state <= ST_IDLE;
                    end else begin
                        cell_cnt <= cell_cnt + 12'd1;
                    end
                end

                // Measuring one command bit's low pulse; the rising
                // edge classifies it by width ('1' short, '0' long).
                ST_CMD_LOW: begin
                    if (!pic_adb_out) begin
                        if (low_cnt >= BIT_LOW_MAX) begin
                            state <= ST_IDLE;    // malformed bit
                        end else begin
                            low_cnt <= low_cnt + 12'd1;
                        end
                    end else begin
                        cmd_shift <= cmd_full;
                        if (bit_cnt >= (CMD_BITS - 4'd1)) begin
                            // Full command byte received.  Decode and
                            // dispatch to the device bus right away —
                            // devices register their reply within a
                            // handful of fast clk cycles, long before
                            // the stop bit and response gap elapse.
                            cmd_op_r      <= decode_op(cmd_full);
                            dispatch_addr <= cmd_full[7:4];
                            dispatch_op   <= decode_op(cmd_full);
                            dispatch_req  <= 1'b1;
                            state         <= ST_STOP_GAP;
                            cell_cnt      <= 12'd0;
                        end else begin
                            bit_cnt  <= bit_cnt + 4'd1;
                            state    <= ST_CMD_GAP;
                            cell_cnt <= 12'd0;
                        end
                    end
                end

                // High gap before the host's stop-bit low pulse.
                ST_STOP_GAP: begin
                    if (!pic_adb_out) begin
                        state      <= ST_STOP_LOW;
                        low_cnt    <= 12'd1;
                        // Latch the SRQ decision at the stop bit's
                        // falling edge (a device the dispatch just
                        // serviced has already dropped its dev_srq by
                        // now), and start our own low drive
                        // immediately so the hold overlaps the host's
                        // stop pulse — the firmware's SRQ sample loop
                        // begins right after ITS release.
                        srq_active     <= any_srq;
                        resp_drive_low <= any_srq;
                    end else if (cell_cnt >= GAP_TIMEOUT) begin
                        state <= ST_IDLE;
                    end else begin
                        cell_cnt <= cell_cnt + 12'd1;
                    end
                end

                // Host stop-bit low pulse; its rising edge is the
                // frame's end and the SRQ/response decision point.
                ST_STOP_LOW: begin
                    if (!pic_adb_out) begin
                        if (low_cnt >= STOP_LOW_MAX) begin
                            resp_drive_low <= 1'b0;
                            state          <= ST_IDLE;   // malformed stop
                        end else begin
                            low_cnt <= low_cnt + 12'd1;
                        end
                    end else begin
                        if (is_talk && resp_latched_empty &&
                            !resp_latched_valid) begin
                            // Device matched but had nothing to report
                            // — real ADB silence, no data frame.
                            resp_consume <= 1'b1;
                        end
                        cell_cnt <= 12'd0;
                        if (srq_active) begin
                            state <= ST_SRQ_HOLD;
                        end else if (is_talk && resp_latched_valid) begin
                            state <= ST_RESP_GAP;
                        end else if (is_listen) begin
                            // Host is about to transmit the payload frame
                            // for this LISTEN — receive it (see header).
                            lsn_shift    <= 16'h0000;
                            lsn_cell_idx <= 5'd0;
                            state        <= ST_LSN_GAP;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end
                end

                ST_SRQ_HOLD: begin
                    // Real ADB Service Request: keep the bus low for
                    // SRQ_HOLD_TICKS beyond the host's stop-bit
                    // release.  The firmware's bounded release-wait
                    // loop samples it low, sets F_SRQ, and later
                    // notifies the 68k (RB4/INT + unsolicited VIA-SR
                    // byte).
                    resp_drive_low <= 1'b1;
                    if (cell_cnt >= (SRQ_HOLD_TICKS - 12'd1)) begin
                        resp_drive_low <= 1'b0;
                        cell_cnt       <= 12'd0;
                        if (is_talk && resp_latched_valid) begin
                            state <= ST_RESP_GAP;
                        end else if (is_listen) begin
                            // A device may assert SRQ on a LISTEN too; the
                            // host still sends the payload afterwards, so
                            // fall into the receiver rather than idling.
                            // LSN_START_MAX absorbs the hold we just spent.
                            lsn_shift    <= 16'h0000;
                            lsn_cell_idx <= 5'd0;
                            state        <= ST_LSN_GAP;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end else begin
                        cell_cnt <= cell_cnt + 12'd1;
                    end
                end

                ST_RESP_GAP: begin
                    // Turnaround before the answering device may start
                    // driving its response frame — lands inside the
                    // firmware's response-start wait window.
                    resp_drive_low <= 1'b0;
                    if (cell_cnt >= (RESP_GAP_TICKS - 12'd1)) begin
                        state          <= ST_RESP_BITS;
                        cell_cnt       <= 12'd0;
                        resp_shift     <= {resp_latched_b0, resp_latched_b1};
                        resp_cell_idx  <= 5'd0;
                        resp_consume   <= 1'b1;
                        resp_drive_low <= 1'b1;   // start bit is "0"-shaped
                    end else begin
                        cell_cnt <= cell_cnt + 12'd1;
                    end
                end

                ST_RESP_BITS: begin
                    // resp_cell_idx: 0 = start bit ("0"-shaped), 1..16 =
                    // data bits MSB-first from {b0,b1}, 17 = stop bit
                    // ("1"-shaped).
                    if (resp_cell_idx == 5'd0) begin
                        resp_drive_low <= (cell_cnt < TX_LOW_0_TICKS);
                    end else if (resp_cell_idx <= 5'd16) begin
                        resp_drive_low <= resp_shift[15] ? (cell_cnt < TX_LOW_1_TICKS)
                                                          : (cell_cnt < TX_LOW_0_TICKS);
                    end else begin
                        resp_drive_low <= (cell_cnt < TX_LOW_1_TICKS);
                    end

                    if (cell_cnt >= (TX_CELL_TICKS - 12'd1)) begin
                        cell_cnt <= 12'd0;
                        if (resp_cell_idx >= 5'd1 && resp_cell_idx <= 5'd16) begin
                            resp_shift <= {resp_shift[14:0], 1'b0};
                        end
                        if (resp_cell_idx >= (RESP_CELLS - 5'd1)) begin
                            resp_drive_low <= 1'b0;
                            state          <= ST_RESP_DONE;
                        end else begin
                            resp_cell_idx <= resp_cell_idx + 5'd1;
                        end
                    end else begin
                        cell_cnt <= cell_cnt + 12'd1;
                    end
                end

                ST_RESP_DONE: begin
                    // Note: no SRQ chaining here.  The firmware only
                    // samples SRQ in its post-COMMAND-stop window
                    // (which ST_STOP_GAP/ST_STOP_LOW already served);
                    // it never re-checks the line after a response
                    // frame, so a post-response hold would be invisible
                    // to it and only delay the next frame.
                    resp_drive_low <= 1'b0;
                    state          <= ST_IDLE;
                end

                // High gap before the next LISTEN-payload bit cell.  The
                // first one spans the host's Tlt turnaround and so gets
                // the wider LSN_START_MAX bound; subsequent ones are
                // ordinary inter-bit gaps, comfortably inside it.
                ST_LSN_GAP: begin
                    if (!pic_adb_out) begin
                        state   <= ST_LSN_LOW;
                        low_cnt <= 12'd1;
                    end else if (cell_cnt >= LSN_START_MAX) begin
                        state <= ST_IDLE;   // no payload came — give up
                    end else begin
                        cell_cnt <= cell_cnt + 12'd1;
                    end
                end

                // One LISTEN-payload bit's low pulse, classified on its
                // rising edge by width exactly like a command bit.
                // lsn_cell_idx 0 = start bit (framing only), 1..16 = data
                // MSB-first, 17 = stop bit -> payload complete.
                ST_LSN_LOW: begin
                    if (!pic_adb_out) begin
                        if (low_cnt >= BIT_LOW_MAX) begin
                            state <= ST_IDLE;    // malformed bit
                        end else begin
                            low_cnt <= low_cnt + 12'd1;
                        end
                    end else begin
                        if (lsn_cell_idx >= 5'd1 && lsn_cell_idx <= 5'd16)
                            lsn_shift <= {lsn_shift[14:0], cur_bit_is_one};
                        if (lsn_cell_idx >= (LSN_CELLS - 5'd1)) begin
                            // Stop bit seen: lsn_shift holds all 16 data
                            // bits (the last one landed on the previous
                            // cell's rising edge).  Raise the level; the
                            // dispatch block turns it into a one-cycle
                            // dev_listen_valid pulse and samples the bytes.
                            listen_req <= 1'b1;
                            state      <= ST_IDLE;
                        end else begin
                            lsn_cell_idx <= lsn_cell_idx + 5'd1;
                            state        <= ST_LSN_GAP;
                            cell_cnt     <= 12'd0;
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] _unused_clk_mhz = CLK_MHZ;
    /* verilator lint_on UNUSEDSIGNAL */
endmodule
