// MAME reference/excerpt/adaptation attribution: Copyright Joakim Larsson Edstrom.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// scc.v — Zilog Z85C30 Serial Communications Controller (dual-channel)
//
// Implements the asynchronous RS-422 subset of the Z85C30 as wired in
// the Mac Quadra 700: two independent channels (A, B), each with its
// own TX hold / TX shifter / RX FIFO (3-deep) / baud-rate generator,
// plus a shared interrupt vector register (WR2), master interrupt
// control (WR9) and the per-channel interrupt-pending summary at RR3
// on channel A.  RR2 is a stable shared vector readback on both
// channels so ROM probe code sees deterministic configuration.
//
// The real Z85C30 has a "quirky" register-access scheme that multiplexes
// 16 write registers onto a single control port via a write-pointer:
// writes to WR0 with pointer = n select the next write as WRn (pointer
// 0 is the default after reset or after a completed WRn access).
// Reads follow the same pointer to select RR0..RR15.  The pointer is
// device-global on the Universal Bus variant, matching MAME's
// z80scc_device::m_wr0_ptrbits.
//
// Simplified Mac address layout accepted at the `pb_*` interface:
//
//   pb_addr[0] = data/ctrl   (0 = control port, 1 = data port)
//   pb_addr[1] = channel A/B (0 = channel B,    1 = channel A)
//
// The Q700 peripheral bus maps byte-offset bit 1 into pb_addr[1] and
// byte-offset bit 2 into pb_addr[0], matching MAME's dc_ab_r/w offset
// interpretation for the Universal Bus SCC variant.
//
// TX / RX semantics in this model (async RS-422 subset only):
//   - Writing WR8 (data) loads tx_hold and clears RR0[TX_BUF_EMPTY].
//   - The BRG (16-bit reload counter, 2-byte time constant in WR12/13,
//     clocked by an internal pre-scaler off `clk`) meters the serialiser.
//     Under-flow reloads from `{WR13,WR12} + 2` per datasheet/MAME
//     (z80scc.cpp:2787); the +2 is load-bearing for low TC values.
//     When tx_hold is set AND WR5[3] (TX enable) is high, the BRG
//     under-flow latches tx_hold into tx_shift, clears tx_hold_full, and
//     starts the async character frame.  The frame is metered by WR4's
//     x1/x16/x32/x64 clock mode, so an 8N1 byte takes 10 bit times and
//     each bit consumes the selected number of BRG output clocks.
//   - TX hold depth: 1 byte.  Per MAME z80scc.cpp:1051, the Z85C30
//     (TYPE_SCC85C30, in SET_CMOS) ships with a 1-byte TX FIFO; only
//     the ESCC family (Z85230 / Z85233 / Z80230, SET_ESCC) gets the
//     4-byte TX FIFO.  Mac Quadra 700 silicon and MAME's macquadra700
//     wiring both use SCC85C30, so 1-byte hold is correct here.  ROM
//     bring-up software polls RR0[TX_BUF_EMPTY] before each WR8 write,
//     matching the 1-byte protocol.
//   - RX path: rx_fifo is 3-deep.  Incoming bytes push; reads from the
//     data port pop.  RR0[RX_AVAIL] is set while fifo != empty.
//   - IRQs: WR1 per-bit enables + WR9 MIE master enable.  RR3 on chan A
//     gives the three-per-channel IP summary.
//
// Loopback: WR14[4] enables the datasheet's "local loopback" mode — a
// channel's own tx_done byte is pushed directly onto its own rx_fifo.
// The tb uses this to exercise the TX → RX → IRQ path without external
// wires.  No separate SIM_LOOPBACK define is needed.
//
// Reset: external `rst` or a WR9[7:6] write (01 = chan B reset, 10 =
// chan A reset, 11 = hardware reset) clears the affected registers.
// The reset-command bits self-clear the cycle after the write so
// software sees the chip back in "normal" state on the next read.
//
// Reference:
//   Zilog Z85C30 Serial Communications Controller Datasheet, §3 "Async
//   Mode Register Descriptions".  Apple Technical Note HW 01 "Serial
//   Port Software Access".  MAME scc8530.cpp (behavior verification).

`default_nettype none

module scc #(
    // Pre-scaler from `clk` to the BRG reference tick.  The Q700 PCLK is
    // 3.672 MHz, so PCLK_DIV must be round(clk_hz / 3.672e6).
    //
    // THIS DEFAULT IS FOR clk = 200 MHz AND IS WRONG FOR ANY OTHER CLOCK.
    // The SoC integration runs this module on pb_clk (50 MHz) and so
    // OVERRIDES it — see fpga_top_peripherals.vh, which derives it from
    // PB_CLK_HZ.  Leaving the default in place there gave a 926 kHz BRG
    // reference and every baud rate ~4x slow, undetected because tb-scc
    // overrides PCLK_DIV to 2 and never exercises the shipped value.
    //
    // PCLK_DIV = 54 (200e6 / 3.672e6 ≈ 54.5) is close enough — the
    // datasheet says the BRG feeds off PCLK/2 so we further divide by 2
    // internally.  The tb sets PCLK_DIV = 2 to make a byte time happen
    // in a handful of simulation cycles.
    parameter integer PCLK_DIV = 32'd54,
    // Simulation-only trace hook for ROM SCC probes.  Leave at 0 for
    // normal builds; set with Verilator -GLOG_PROBES=1 when correlating
    // ROM register traffic.
    parameter integer LOG_PROBES = 0
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [3:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output reg  [7:0]  pb_rdata,
    output reg         pb_ack,
    input  wire        rx_a_valid,
    input  wire [7:0]  rx_a_data,
    input  wire        rx_b_valid,
    input  wire [7:0]  rx_b_data,
    input  wire        cts_a_n,
    input  wire        dcd_a_n,
    input  wire        sync_a_n,
    input  wire        cts_b_n,
    input  wire        dcd_b_n,
    input  wire        sync_b_n,
    output wire        tx_a_valid,
    output wire [7:0]  tx_a_data,
    output wire        tx_b_valid,
    output wire [7:0]  tx_b_data,
    output wire        rts_a_n,
    output wire        dtr_a_n,
    output wire        rts_b_n,
    output wire        dtr_b_n,
    output wire        irq
);

    // ── Address decode ────────────────────────────────────────────────
    wire sel_data = pb_addr[0];    // 0 = ctrl port, 1 = data port
    wire sel_a    = pb_addr[1];    // 0 = chan B,    1 = chan A

    localparam [2:0] WR0_CMD_POINT_HIGH        = 3'b001;
    localparam [2:0] WR0_CMD_RESET_EXT_STATUS  = 3'b010;
    localparam [2:0] WR0_CMD_SEND_ABORT        = 3'b011;
    localparam [2:0] WR0_CMD_ENABLE_INT_NEXT_RX = 3'b100;
    localparam [2:0] WR0_CMD_RESET_TX_IP       = 3'b101;
    localparam [2:0] WR0_CMD_ERROR_RESET       = 3'b110;
    localparam [2:0] WR0_CMD_RESET_HIGHEST_IUS = 3'b111;
    localparam [7:0] RR1_RX_OVERRUN_ERROR      = 8'h20;

    // ── PCLK pre-scaler ──────────────────────────────────────────────
    // Produces a one-cycle `pclk_tick` at PCLK_DIV-cycle intervals.  The
    // BRG then further divides this by 2 (per datasheet).
    reg [31:0] pclk_div;
    wire pclk_tick = (pclk_div == (PCLK_DIV - 32'd1));
    always @(posedge clk) begin
        if (rst) pclk_div <= 32'd0;
        else     pclk_div <= pclk_tick ? 32'd0 : (pclk_div + 32'd1);
    end
    reg brg_phase;
    wire brg_ref = pclk_tick && brg_phase;
    always @(posedge clk) begin
        if (rst)             brg_phase <= 1'b0;
        else if (pclk_tick)  brg_phase <= ~brg_phase;
    end

    // ── Register banks, one per channel ──────────────────────────────
    // Channel A = suffix _a, channel B = suffix _b.  WR2 and WR9 are
    // architecturally shared — writes mirror into both banks.
    //
    // Storage is sparse — the read mux below only consumes a subset of
    // the 16-element WR space (WR2, WR4, WR5, WR9, WR11..WR13, WR15);
    // the rest of the protocol-relevant registers are decoded for their
    // side-effects only (TX-hold load on WR8, command decode on WR0,
    // chip-reset on WR9[7:6], BRG reload on WR12/WR13).  Slots that are
    // neither read back nor consumed by control logic — WR0, WR6, WR7,
    // WR8, WR10 — are intentionally not stored to keep Vivado from
    // dead-code-eliminating them after an unused-FF complaint.  Adding a
    // read path for any of those slots in future requires also adding
    // the corresponding `wra[i] <= pb_wdata` write below.
    reg [3:0] ptr;
    reg [7:0] wra [0:15];
    reg [7:0] wrb [0:15];
    localparam [7:0] WR15_RESET = 8'hF8;

    // Live-storage mask: 1 = the wra/wrb slot is actually consumed
    // (either read back via the case mux or fed into an FSM).  Used to
    // gate writes so dead slots don't emit FFs that the synth tool then
    // strips with a [Synth 8-6014] warning.
    function live_wr_slot;
        input [3:0] idx;
        begin
            // Live consumers (from rdata mux, IRQ enables, BRG, modem
            // outs, loopback gating):
            //   WR1  → tx/rx/ext IRQ enables
            //   WR2  → shared interrupt vector (mirrored both banks)
            //   WR3  → RX enable bit gates loopback / external RX push
            //   WR4  → readback (sync mode / clock mode)
            //   WR5  → TX enable bit, RTS/DTR pins, readback
            //   WR9  → MIE bit, readback
            //   WR11 → readback (clock mode)
            //   WR12 → BRG reload low + readback
            //   WR13 → BRG reload high + readback
            //   WR14 → BRG enable + local-loopback bit (NOT in rdata mux,
            //          but consumed in BRG / loopback FSM gating)
            //   WR15 → external-status enables + readback
            // Dead-by-design (write-only or unused):
            //   WR0  → command/pointer port (no storage)
            //   WR6  → SDLC sync byte 1 (no SDLC mode)
            //   WR7  → SDLC sync byte 2 (no SDLC mode)
            //   WR8  → TX hold mirror (TX path uses tx_hold_a/b directly)
            //   WR10 → misc TX/RX mode (no NRZI / mark-idle / CRC reset)
            case (idx)
                4'd1, 4'd2, 4'd3, 4'd4, 4'd5, 4'd9,
                4'd11, 4'd12, 4'd13, 4'd14, 4'd15:
                    live_wr_slot = 1'b1;
                default:
                    live_wr_slot = 1'b0;
            endcase
        end
    endfunction

    function [9:0] tx_frame_clocks;
        input [7:0] wr4;
        begin
            case (wr4[7:6])
                2'b00: tx_frame_clocks = 10'd10;   // x1
                2'b01: tx_frame_clocks = 10'd160;  // x16
                2'b10: tx_frame_clocks = 10'd320;  // x32
                default: tx_frame_clocks = 10'd640; // x64
            endcase
        end
    endfunction

    // WR0_CMD_RESET_HIGHEST_IUS: the Z85C30 IUS daisy chain has a FIXED
    // priority order — RxA > TxA > ExtA > RxB > TxB > ExtB — regardless
    // of which channel's control port carried the command byte.  Shared
    // by both the channel-A and channel-B WR0 command decode arms below
    // so the order can't drift between them again.
    task do_reset_highest_ius;
        begin
            if (rx_ip_a) begin
                rx_ip_a <= 1'b0;
            end else if (tx_ip_a) begin
                tx_ip_a <= 1'b0;
            end else if (ext_ip_a) begin
                ext_ip_a <= 1'b0;
            end else if (rx_ip_b) begin
                rx_ip_b <= 1'b0;
            end else if (tx_ip_b) begin
                tx_ip_b <= 1'b0;
            end else if (ext_ip_b) begin
                ext_ip_b <= 1'b0;
            end
        end
    endtask

    // ── BRG counters ─────────────────────────────────────────────────
    // Each channel's BRG counts down on brg_ref while WR14[0]=1.
    // Under-flow reloads from {WR13,WR12} + 2 and pulses the serialiser.
    //
    // Reload formula: per Z85C30 datasheet (and MAME z80scc.cpp:2787 +
    // :2822: `brg_const = 2 + (m_wr13 << 8 | m_wr12);`), the BRG output
    // bit period is (TC + 2) BRG-input clocks.  The "+2" comes from the
    // datasheet quote at z80scc.cpp:2155-2158: "the counter decrements
    // from N down to zero-plus-one-cycle for reloading the time
    // constant.  This is then fed to a toggle flip-flop to make the
    // output a square wave."  Earlier revisions of this RTL used `TC+1`
    // — close, but off-by-one vs MAME and the datasheet at low TC
    // values (most visibly at TC=0 where MAME divides by 2, not 1).
    reg [15:0] brg_a, brg_b;

    // ── TX path ──────────────────────────────────────────────────────
    reg [7:0] tx_hold_a,   tx_hold_b;
    reg       tx_hold_full_a, tx_hold_full_b;
    reg       tx_shifting_a, tx_shifting_b;
    reg [7:0] tx_shift_byte_a, tx_shift_byte_b;
    reg [9:0] tx_bits_left_a, tx_bits_left_b;
    reg       tx_done_a,    tx_done_b;      // one-cycle byte-complete pulse
    assign tx_a_valid = tx_done_a;
    assign tx_a_data  = tx_shift_byte_a;
    assign tx_b_valid = tx_done_b;
    assign tx_b_data  = tx_shift_byte_b;
    assign rts_a_n    = ~wra[5][1];
    assign dtr_a_n    = ~wra[5][7];
    assign rts_b_n    = ~wrb[5][1];
    assign dtr_b_n    = ~wrb[5][7];

    // ── RX path (3-deep FIFO + overrun latch) ───────────────────────
    reg [7:0] rxf_a0, rxf_a1, rxf_a2;
    reg [1:0] rxlvl_a;                       // 0..3
    reg       rxov_a;
    reg [7:0] rxf_b0, rxf_b1, rxf_b2;
    reg [1:0] rxlvl_b;
    reg       rxov_b;

    // ── RX FIFO push/pop wires (netted in the main sequential block) ─
    // A push (WR14[4] local loopback of a completed TX byte, or an
    // external rx_x_valid byte) and a CPU data-port pop (read of RR8)
    // can both want to touch rxlvl_x / rxf_x0..2 in the same cycle.
    // These wires are combined below in ONE case block per channel so
    // level + slot placement net correctly instead of two independent
    // non-blocking writes racing (last-in-program-order silently
    // winning).  Matches the idiom in asc.v:537-546 / scsi.v:2301-2315.
    wire       rx_a_loop_push = tx_done_a && wra[14][4] && wra[3][0];
    wire       rx_a_ext_push  = rx_a_valid && wra[3][0];
    wire       rx_a_push      = rx_a_loop_push || rx_a_ext_push;
    // On a same-cycle double-push (loopback + external both fire),
    // external data wins — matches the pre-existing accidental
    // last-write-wins priority (external's block ran after loopback's),
    // preserved here rather than silently changed as part of this fix.
    wire [7:0] rx_a_push_data = rx_a_ext_push ? rx_a_data : tx_shift_byte_a;
    wire       rx_a_pop       = pb_rd && !pb_wr && sel_a && sel_data &&
                                 (rxlvl_a != 2'd0);

    wire       rx_b_loop_push = tx_done_b && wrb[14][4] && wrb[3][0];
    wire       rx_b_ext_push  = rx_b_valid && wrb[3][0];
    wire       rx_b_push      = rx_b_loop_push || rx_b_ext_push;
    wire [7:0] rx_b_push_data = rx_b_ext_push ? rx_b_data : tx_shift_byte_b;
    wire       rx_b_pop       = pb_rd && !pb_wr && !sel_a && sel_data &&
                                 (rxlvl_b != 2'd0);

    // ── IRQ-pending latches (per channel) ───────────────────────────
    reg tx_ip_a, rx_ip_a, ext_ip_a;
    reg tx_ip_b, rx_ip_b, ext_ip_b;
    reg cts_a_n_q, dcd_a_n_q, sync_a_n_q;
    reg cts_b_n_q, dcd_b_n_q, sync_b_n_q;

    // ── SYNC/HUNT latch (RR0 bit 4), per channel ────────────────────
    // MAME z80scc semantics (z80scc.cpp do_sccreg_wr3 + sync_w):
    //   - SET by any WR3 write with bit4 (Enter Hunt Mode) OR bit0=0
    //     (receiver disabled): `if ((wr3 & ENTER_HUNT) || !(wr3 &
    //     RX_ENABLE)) rr0 |= SYNC_HUNT;`
    //   - Follows /SYNC pin transitions (bit = !pin).
    //   - On the real chip it is cleared when the receiver leaves hunt
    //     (opening flag/sync detected).  No SDLC receive path exists in
    //     this model (nor a peer on the board), so nothing clears it
    //     except a pin transition — matching MAME's steady-state during
    //     LocalTalk lapENQ address acquisition, where RR0 reads 0x54
    //     (underrun/EOM + SYNC/HUNT + TX-empty) throughout.
    //   - WR9 soft resets do NOT clear it (MAME channel reset only
    //     forces rr0 bits [6][2] set and [1:0] clear; hunt preserved).
    // Load-bearing for Mac OS boot: the ROM .MPP driver's LocalTalk
    // carrier-sense loop reads RR0 while hunting; with this bit stuck 0
    // (old behavior: raw ~sync pin, tied 1 at the board) the driver
    // never sees the line as idle and its transmit never completes —
    // the "Welcome to Macintosh" AppleTalk hang.
    reg hunt_a, hunt_b;

    // ── Interrupt-enable decode ─────────────────────────────────────
    // WR1[1] = TX int enable, WR1[4:3] = RX int mode (nonzero = enable),
    // WR1[0] = ext-status int enable.
    wire tx_ie_a = wra[1][1];
    wire tx_ie_b = wrb[1][1];
    wire rx_ie_a = |wra[1][4:3];
    wire rx_ie_b = |wrb[1][4:3];
    wire ex_ie_a = wra[1][0];
    wire ex_ie_b = wrb[1][0];
    wire mie     = wra[9][3];   // WR9 is shared — wra and wrb both hold it.

    wire pending_rx_a = rx_ip_a && rx_ie_a;
    wire pending_tx_a = tx_ip_a && tx_ie_a;
    wire pending_ex_a = ext_ip_a && ex_ie_a;
    wire pending_rx_b = rx_ip_b && rx_ie_b;
    wire pending_tx_b = tx_ip_b && tx_ie_b;
    wire pending_ex_b = ext_ip_b && ex_ie_b;
    wire ext_change_a = ((cts_a_n_q ^ cts_a_n) && wra[15][5]) |
                        ((sync_a_n_q ^ sync_a_n) && wra[15][4]) |
                        ((dcd_a_n_q ^ dcd_a_n) && wra[15][3]);
    wire ext_change_b = ((cts_b_n_q ^ cts_b_n) && wrb[15][5]) |
                        ((sync_b_n_q ^ sync_b_n) && wrb[15][4]) |
                        ((dcd_b_n_q ^ dcd_b_n) && wrb[15][3]);

    // ── RR0 per channel ─────────────────────────────────────────────
    // Bits: 0=RX avail, 1=zero-count(stub), 2=TX buffer empty,
    // 3=DCD(1), 4=SYNC/HUNT, 5=CTS(1), 6=TX under-run (no char + idle),
    // 7=BREAK.
    wire [7:0] rr0_a = { 1'b0,
                         1'b1,
                         ~cts_a_n,
                         hunt_a,
                         ~dcd_a_n,
                         (!tx_hold_full_a),
                         1'b0,
                         (rxlvl_a != 2'd0) };
    wire [7:0] rr0_b = { 1'b0,
                         1'b1,
                         ~cts_b_n,
                         hunt_b,
                         ~dcd_b_n,
                         (!tx_hold_full_b),
                         1'b0,
                         (rxlvl_b != 2'd0) };
    // RR1 exposes transmit all-sent status plus receive error state.
    // Bits [3:1] are the SDLC residue-code reset value; MAME's z80scc
    // model and ROM probes both observe 3'b011 here even in async mode.
    // Bit 0 is set when the transmit buffer and shifter are both empty;
    // bit 5 latches receive overrun until WR0 error-reset.
    wire [7:0] rr1_a = {2'b00, rxov_a, 1'b0, 3'b011, (!tx_hold_full_a && !tx_shifting_a)};
    wire [7:0] rr1_b = {2'b00, rxov_b, 1'b0, 3'b011, (!tx_hold_full_b && !tx_shifting_b)};
    wire [7:0] rr2_a = wra[2];
    wire [7:0] rr2_b = wrb[2];
    // RR3 IP summary (chan A only).
    wire [7:0] rr3 = { 2'b00,
                       rx_ip_a, tx_ip_a, ext_ip_a,
                       rx_ip_b, tx_ip_b, ext_ip_b };

    // ── Read data mux ────────────────────────────────────────────────
    // Control-port read honours the current pointer.  Data-port read
    // always returns the head of the RX FIFO (RR8).
    reg [7:0] rd_byte_a, rd_byte_b;
    always @(*) begin
        case (ptr)
            4'd0:    rd_byte_a = rr0_a;
            4'd1:    rd_byte_a = rr1_a;
            4'd2:    rd_byte_a = rr2_a;
            4'd3:    rd_byte_a = rr3;
            4'd4:    rd_byte_a = wra[4];
            4'd5:    rd_byte_a = wra[5];
            4'd8:    rd_byte_a = (rxlvl_a != 2'd0) ? rxf_a0 : 8'h00;
            4'd9:    rd_byte_a = wra[9];
            4'd10:   rd_byte_a = 8'h00;
            4'd11:   rd_byte_a = wra[11];
            4'd12:   rd_byte_a = wra[12];
            4'd13:   rd_byte_a = wra[13];
            4'd15:   rd_byte_a = wra[15] & 8'hFA;
            default: rd_byte_a = 8'h00;
        endcase
        case (ptr)
            4'd0:    rd_byte_b = rr0_b;
            4'd1:    rd_byte_b = rr1_b;
            4'd2:    rd_byte_b = rr2_b;
            4'd3:    rd_byte_b = 8'h00;      // RR3 meaningful only on chan A
            4'd4:    rd_byte_b = wrb[4];
            4'd5:    rd_byte_b = wrb[5];
            4'd8:    rd_byte_b = (rxlvl_b != 2'd0) ? rxf_b0 : 8'h00;
            4'd9:    rd_byte_b = wrb[9];
            4'd10:   rd_byte_b = 8'h00;
            4'd11:   rd_byte_b = wrb[11];
            4'd12:   rd_byte_b = wrb[12];
            4'd13:   rd_byte_b = wrb[13];
            4'd15:   rd_byte_b = wrb[15] & 8'hFA;
            default: rd_byte_b = 8'h00;
        endcase
    end
    wire [7:0] pb_read_preview =
        sel_data ? (sel_a ? ((rxlvl_a != 2'd0) ? rxf_a0 : 8'h00)
                          : ((rxlvl_b != 2'd0) ? rxf_b0 : 8'h00))
                 : (sel_a ? rd_byte_a : rd_byte_b);

`ifndef SYNTHESIS
    reg [63:0] log_cycle;
    always @(posedge clk) begin
        if (rst)
            log_cycle <= 64'd0;
        else
            log_cycle <= log_cycle + 64'd1;
    end
`endif

    // ── Main sequential logic ───────────────────────────────────────
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            pb_ack   <= 1'b0;
            ptr <= 4'd0;
            for (i = 0; i < 16; i = i + 1) begin
                if (live_wr_slot(i[3:0])) begin
                    wra[i] <= 8'h00;
                    wrb[i] <= 8'h00;
                end
            end
            wra[15] <= WR15_RESET;
            wrb[15] <= WR15_RESET;
            brg_a <= 16'h0000; brg_b <= 16'h0000;
            tx_hold_a <= 8'h00; tx_hold_b <= 8'h00;
            tx_hold_full_a <= 1'b0; tx_hold_full_b <= 1'b0;
            tx_shifting_a <= 1'b0; tx_shifting_b <= 1'b0;
            tx_shift_byte_a <= 8'h00; tx_shift_byte_b <= 8'h00;
            tx_bits_left_a <= 10'd0; tx_bits_left_b <= 10'd0;
            tx_done_a <= 1'b0; tx_done_b <= 1'b0;
            rxf_a0 <= 8'h00; rxf_a1 <= 8'h00; rxf_a2 <= 8'h00;
            rxf_b0 <= 8'h00; rxf_b1 <= 8'h00; rxf_b2 <= 8'h00;
            rxlvl_a <= 2'd0; rxlvl_b <= 2'd0;
            rxov_a  <= 1'b0; rxov_b  <= 1'b0;
            tx_ip_a <= 1'b0; tx_ip_b <= 1'b0;
            rx_ip_a <= 1'b0; rx_ip_b <= 1'b0;
            ext_ip_a <= 1'b0; ext_ip_b <= 1'b0;
            cts_a_n_q <= cts_a_n; dcd_a_n_q <= dcd_a_n; sync_a_n_q <= sync_a_n;
            cts_b_n_q <= cts_b_n; dcd_b_n_q <= dcd_b_n; sync_b_n_q <= sync_b_n;
            hunt_a <= ~sync_a_n;
            hunt_b <= ~sync_b_n;
        end else begin
            pb_ack    <= pb_wr | pb_rd;
            tx_done_a <= 1'b0;
            tx_done_b <= 1'b0;
            cts_a_n_q <= cts_a_n; dcd_a_n_q <= dcd_a_n; sync_a_n_q <= sync_a_n;
            cts_b_n_q <= cts_b_n; dcd_b_n_q <= dcd_b_n; sync_b_n_q <= sync_b_n;
            // SYNC/HUNT follows /SYNC pin transitions (MAME sync_w);
            // a same-cycle WR3 hunt/rx-disable write below overrides.
            if (sync_a_n_q ^ sync_a_n) hunt_a <= ~sync_a_n;
            if (sync_b_n_q ^ sync_b_n) hunt_b <= ~sync_b_n;
            if (ext_change_a)
                ext_ip_a <= 1'b1;
            if (ext_change_b)
                ext_ip_b <= 1'b1;

            // ═════════════════ WRITE HANDLING ═════════════════
            if (pb_wr) begin
                if (sel_a) begin
                    // ── Channel A ──
                    if (sel_data) begin
                        // Data-port write: WR8 / TX hold.
                        // wra[8] storage is dead — TX path uses tx_hold_a.
                        tx_hold_a      <= pb_wdata;
                        tx_hold_full_a <= wra[5][3] && !tx_shifting_a ? 1'b0 : 1'b1;
                        if (wra[5][3] && !tx_shifting_a) begin
                            tx_shift_byte_a <= pb_wdata;
                            tx_shifting_a   <= 1'b1;
                            tx_bits_left_a  <= tx_frame_clocks(wra[4]);
                        end
                        tx_ip_a        <= 1'b0;   // TX buf no longer empty
                    end else begin
                        if (ptr == 4'd0) begin
                            // WR0: pointer + command.  wra[0] storage is
                            // dead per Z85C30 spec — WR0 is command-only.
                            case (pb_wdata[5:3])
                                WR0_CMD_POINT_HIGH: ptr <= {1'b1, pb_wdata[2:0]};
                                WR0_CMD_RESET_EXT_STATUS: ext_ip_a <= 1'b0;
                                WR0_CMD_SEND_ABORT: ;
                                WR0_CMD_ENABLE_INT_NEXT_RX: ;
                                WR0_CMD_RESET_TX_IP: tx_ip_a <= 1'b0;
                                WR0_CMD_ERROR_RESET: rxov_a <= 1'b0;
                                WR0_CMD_RESET_HIGHEST_IUS: do_reset_highest_ius;
                                default: ;
                            endcase
                            if (pb_wdata[5:3] != WR0_CMD_POINT_HIGH && pb_wdata[2:0] != 3'd0)
                                ptr <= {1'b0, pb_wdata[2:0]};
                        end else begin
                            // Targeted WRn write.  Only store the slot
                            // when it has a live read or control consumer.
                            if (live_wr_slot(ptr))
                                wra[ptr] <= pb_wdata;
                            case (ptr)
                                4'd2: wrb[2] <= pb_wdata;   // shared vec
                                // WR3: Enter-Hunt command (bit4) or
                                // receiver-disable (bit0=0) sets the
                                // SYNC/HUNT latch — MAME do_sccreg_wr3.
                                4'd3: if (pb_wdata[4] || !pb_wdata[0])
                                          hunt_a <= 1'b1;
                                4'd8: begin
                                    tx_hold_a      <= pb_wdata;
                                    tx_hold_full_a <= wra[5][3] && !tx_shifting_a ? 1'b0 : 1'b1;
                                    if (wra[5][3] && !tx_shifting_a) begin
                                        tx_shift_byte_a <= pb_wdata;
                                        tx_shifting_a   <= 1'b1;
                                        tx_bits_left_a  <= tx_frame_clocks(wra[4]);
                                    end
                                    tx_ip_a        <= 1'b0;
                                end
                                4'd9: begin
                                    wrb[9] <= pb_wdata;
                                    case (pb_wdata[7:6])
                                        2'b01: begin          // chan B reset
                                            for (i = 0; i < 16; i = i + 1)
                                                if (live_wr_slot(i[3:0]))
                                                    wrb[i] <= 8'h00;
                                            wrb[15] <= WR15_RESET;
                                            brg_b <= 16'h0000;
                                            tx_hold_b <= 8'h00;
                                            tx_shift_byte_b <= 8'h00;
                                            tx_hold_full_b <= 1'b0;
                                            tx_shifting_b  <= 1'b0;
                                            tx_bits_left_b <= 10'd0;
                                            rxf_b0 <= 8'h00; rxf_b1 <= 8'h00; rxf_b2 <= 8'h00;
                                            rxlvl_b <= 2'd0; rxov_b <= 1'b0;
                                            tx_ip_b <= 1'b0; rx_ip_b <= 1'b0; ext_ip_b <= 1'b0;
                                            ptr   <= 4'd0;
                                            wrb[9] <= {2'b00, pb_wdata[5:0]};
                                        end
                                        2'b10: begin          // chan A reset
                                            for (i = 0; i < 16; i = i + 1)
                                                if (live_wr_slot(i[3:0]))
                                                    wra[i] <= 8'h00;
                                            wra[15] <= WR15_RESET;
                                            brg_a <= 16'h0000;
                                            tx_hold_a <= 8'h00;
                                            tx_shift_byte_a <= 8'h00;
                                            tx_hold_full_a <= 1'b0;
                                            tx_shifting_a  <= 1'b0;
                                            tx_bits_left_a <= 10'd0;
                                            rxf_a0 <= 8'h00; rxf_a1 <= 8'h00; rxf_a2 <= 8'h00;
                                            rxlvl_a <= 2'd0; rxov_a <= 1'b0;
                                            tx_ip_a <= 1'b0; rx_ip_a <= 1'b0; ext_ip_a <= 1'b0;
                                            ptr   <= 4'd0;
                                            wra[9] <= {2'b00, pb_wdata[5:0]};
                                            wrb[9] <= {2'b00, pb_wdata[5:0]};
                                        end
                                        2'b11: begin          // hardware reset
                                            for (i = 0; i < 16; i = i + 1) begin
                                                if (live_wr_slot(i[3:0])) begin
                                                    wra[i] <= 8'h00;
                                                    wrb[i] <= 8'h00;
                                                end
                                            end
                                            wra[15] <= WR15_RESET;
                                            wrb[15] <= WR15_RESET;
                                            brg_a <= 16'h0000; brg_b <= 16'h0000;
                                            tx_hold_a <= 8'h00; tx_hold_b <= 8'h00;
                                            tx_shift_byte_a <= 8'h00; tx_shift_byte_b <= 8'h00;
                                            tx_hold_full_a <= 1'b0; tx_hold_full_b <= 1'b0;
                                            tx_shifting_a  <= 1'b0; tx_shifting_b  <= 1'b0;
                                            tx_bits_left_a <= 10'd0; tx_bits_left_b <= 10'd0;
                                            rxf_a0 <= 8'h00; rxf_a1 <= 8'h00; rxf_a2 <= 8'h00;
                                            rxf_b0 <= 8'h00; rxf_b1 <= 8'h00; rxf_b2 <= 8'h00;
                                            rxlvl_a <= 2'd0; rxlvl_b <= 2'd0;
                                            rxov_a  <= 1'b0; rxov_b  <= 1'b0;
                                            tx_ip_a <= 1'b0; tx_ip_b <= 1'b0;
                                            rx_ip_a <= 1'b0; rx_ip_b <= 1'b0;
                                            ext_ip_a <= 1'b0; ext_ip_b <= 1'b0;
                                            ptr   <= 4'd0;
                                            wra[9] <= {2'b00, pb_wdata[5:0]};
                                            wrb[9] <= {2'b00, pb_wdata[5:0]};
                                        end
                                        default: ;
                                    endcase
                                end
                                4'd12: brg_a[7:0]  <= pb_wdata;  // reload BRG
                                4'd13: brg_a[15:8] <= pb_wdata;
                                default: ;
                            endcase
                            ptr <= 4'd0;
                        end
                    end
                end else begin
                    // ── Channel B ── (mirror of channel A above)
                    if (sel_data) begin
                        // wrb[8] storage is dead — TX path uses tx_hold_b.
                        tx_hold_b      <= pb_wdata;
                        tx_hold_full_b <= wrb[5][3] && !tx_shifting_b ? 1'b0 : 1'b1;
                        if (wrb[5][3] && !tx_shifting_b) begin
                            tx_shift_byte_b <= pb_wdata;
                            tx_shifting_b   <= 1'b1;
                            tx_bits_left_b  <= tx_frame_clocks(wrb[4]);
                        end
                        tx_ip_b        <= 1'b0;
                    end else begin
                        if (ptr == 4'd0) begin
                            // wrb[0] dead per Z85C30 spec — command-only.
                            case (pb_wdata[5:3])
                                WR0_CMD_POINT_HIGH: ptr <= {1'b1, pb_wdata[2:0]};
                                WR0_CMD_RESET_EXT_STATUS: ext_ip_b <= 1'b0;
                                WR0_CMD_SEND_ABORT: ;
                                WR0_CMD_ENABLE_INT_NEXT_RX: ;
                                WR0_CMD_RESET_TX_IP: tx_ip_b <= 1'b0;
                                WR0_CMD_ERROR_RESET: rxov_b <= 1'b0;
                                WR0_CMD_RESET_HIGHEST_IUS: do_reset_highest_ius;
                                default: ;
                            endcase
                            if (pb_wdata[5:3] != WR0_CMD_POINT_HIGH && pb_wdata[2:0] != 3'd0)
                                ptr <= {1'b0, pb_wdata[2:0]};
                        end else begin
                            if (live_wr_slot(ptr))
                                wrb[ptr] <= pb_wdata;
                            case (ptr)
                                4'd2: wra[2] <= pb_wdata;
                                // WR3 hunt/rx-disable → SYNC/HUNT latch
                                // (see channel-A arm / MAME do_sccreg_wr3).
                                4'd3: if (pb_wdata[4] || !pb_wdata[0])
                                          hunt_b <= 1'b1;
                                4'd8: begin
                                    tx_hold_b      <= pb_wdata;
                                    tx_hold_full_b <= wrb[5][3] && !tx_shifting_b ? 1'b0 : 1'b1;
                                    if (wrb[5][3] && !tx_shifting_b) begin
                                        tx_shift_byte_b <= pb_wdata;
                                        tx_shifting_b   <= 1'b1;
                                        tx_bits_left_b  <= tx_frame_clocks(wrb[4]);
                                    end
                                    tx_ip_b        <= 1'b0;
                                end
                                4'd9: begin
                                    wra[9] <= pb_wdata;
                                    case (pb_wdata[7:6])
                                        2'b01: begin
                                            for (i = 0; i < 16; i = i + 1)
                                                wrb[i] <= 8'h00;
                                            wrb[15] <= WR15_RESET;
                                            brg_b <= 16'h0000;
                                            tx_hold_b <= 8'h00;
                                            tx_shift_byte_b <= 8'h00;
                                            tx_hold_full_b <= 1'b0;
                                            tx_shifting_b  <= 1'b0;
                                            tx_bits_left_b <= 10'd0;
                                            rxf_b0 <= 8'h00; rxf_b1 <= 8'h00; rxf_b2 <= 8'h00;
                                            rxlvl_b <= 2'd0; rxov_b <= 1'b0;
                                            tx_ip_b <= 1'b0; rx_ip_b <= 1'b0; ext_ip_b <= 1'b0;
                                            ptr   <= 4'd0;
                                            wra[9] <= {2'b00, pb_wdata[5:0]};
                                            wrb[9] <= {2'b00, pb_wdata[5:0]};
                                        end
                                        2'b10: begin
                                            for (i = 0; i < 16; i = i + 1)
                                                wra[i] <= 8'h00;
                                            wra[15] <= WR15_RESET;
                                            brg_a <= 16'h0000;
                                            tx_hold_a <= 8'h00;
                                            tx_shift_byte_a <= 8'h00;
                                            tx_hold_full_a <= 1'b0;
                                            tx_shifting_a  <= 1'b0;
                                            tx_bits_left_a <= 10'd0;
                                            rxf_a0 <= 8'h00; rxf_a1 <= 8'h00; rxf_a2 <= 8'h00;
                                            rxlvl_a <= 2'd0; rxov_a <= 1'b0;
                                            tx_ip_a <= 1'b0; rx_ip_a <= 1'b0; ext_ip_a <= 1'b0;
                                            ptr   <= 4'd0;
                                            wra[9] <= {2'b00, pb_wdata[5:0]};
                                            wrb[9] <= {2'b00, pb_wdata[5:0]};
                                        end
                                        2'b11: begin
                                            for (i = 0; i < 16; i = i + 1) begin
                                                wra[i] <= 8'h00;
                                                wrb[i] <= 8'h00;
                                            end
                                            wra[15] <= WR15_RESET;
                                            wrb[15] <= WR15_RESET;
                                            brg_a <= 16'h0000; brg_b <= 16'h0000;
                                            tx_hold_a <= 8'h00; tx_hold_b <= 8'h00;
                                            tx_shift_byte_a <= 8'h00; tx_shift_byte_b <= 8'h00;
                                            tx_hold_full_a <= 1'b0; tx_hold_full_b <= 1'b0;
                                            tx_shifting_a  <= 1'b0; tx_shifting_b  <= 1'b0;
                                            tx_bits_left_a <= 10'd0; tx_bits_left_b <= 10'd0;
                                            rxf_a0 <= 8'h00; rxf_a1 <= 8'h00; rxf_a2 <= 8'h00;
                                            rxf_b0 <= 8'h00; rxf_b1 <= 8'h00; rxf_b2 <= 8'h00;
                                            rxlvl_a <= 2'd0; rxlvl_b <= 2'd0;
                                            rxov_a  <= 1'b0; rxov_b  <= 1'b0;
                                            tx_ip_a <= 1'b0; tx_ip_b <= 1'b0;
                                            rx_ip_a <= 1'b0; rx_ip_b <= 1'b0;
                                            ext_ip_a <= 1'b0; ext_ip_b <= 1'b0;
                                            ptr   <= 4'd0;
                                            wra[9] <= {2'b00, pb_wdata[5:0]};
                                            wrb[9] <= {2'b00, pb_wdata[5:0]};
                                        end
                                        default: ;
                                    endcase
                                end
                                4'd12: brg_b[7:0]  <= pb_wdata;
                                4'd13: brg_b[15:8] <= pb_wdata;
                                default: ;
                            endcase
                            ptr <= 4'd0;
                        end
                    end
                end
            end

            // ═════════════════ READ SIDE-EFFECTS ═════════════════
            // Reads never update the bank.  The data-port RX pop is
            // netted together with any same-cycle push in the RX FIFO
            // PUSH/POP section below (rx_a_pop / rx_b_pop wires) instead
            // of being applied here, so a concurrent push can't race it.
            // A read of a non-zero control-port register still resets
            // the pointer back to WR0 here.
            if (pb_rd && !pb_wr) begin
                if (sel_a) begin
                    if (!sel_data) begin
                        if (ptr != 4'd0)
                            ptr <= 4'd0;
                    end
                end else begin
                    if (!sel_data) begin
                        if (ptr != 4'd0)
                            ptr <= 4'd0;
                    end
                end
            end

            // ═════════════════ BRG + TX SERIALISER ═════════════════
            if (brg_ref) begin
                // Chan A
                if (wra[14][0]) begin
                    if (brg_a == 16'h0000) begin
                        brg_a <= {wra[13], wra[12]} + 16'd2;
                        if (tx_shifting_a) begin
                            if (tx_bits_left_a <= 10'd1) begin
                                tx_shifting_a  <= 1'b0;
                                tx_bits_left_a <= 10'd0;
                                tx_done_a      <= 1'b1;
                                if (!tx_hold_full_a && tx_ie_a)
                                    tx_ip_a <= 1'b1;
                            end else begin
                                tx_bits_left_a <= tx_bits_left_a - 10'd1;
                            end
                        end else if (tx_hold_full_a && wra[5][3]) begin
                            tx_shift_byte_a <= tx_hold_a;
                            tx_shifting_a   <= 1'b1;
                            tx_bits_left_a  <= tx_frame_clocks(wra[4]);
                            tx_hold_full_a  <= 1'b0;
                            if (tx_ie_a)
                                tx_ip_a <= 1'b1;
                        end
                    end else begin
                        brg_a <= brg_a - 16'd1;
                    end
                end
                // Chan B
                if (wrb[14][0]) begin
                    if (brg_b == 16'h0000) begin
                        brg_b <= {wrb[13], wrb[12]} + 16'd2;
                        if (tx_shifting_b) begin
                            if (tx_bits_left_b <= 10'd1) begin
                                tx_shifting_b  <= 1'b0;
                                tx_bits_left_b <= 10'd0;
                                tx_done_b      <= 1'b1;
                                if (!tx_hold_full_b && tx_ie_b)
                                    tx_ip_b <= 1'b1;
                            end else begin
                                tx_bits_left_b <= tx_bits_left_b - 10'd1;
                            end
                        end else if (tx_hold_full_b && wrb[5][3]) begin
                            tx_shift_byte_b <= tx_hold_b;
                            tx_shifting_b   <= 1'b1;
                            tx_bits_left_b  <= tx_frame_clocks(wrb[4]);
                            tx_hold_full_b  <= 1'b0;
                            if (tx_ie_b)
                                tx_ip_b <= 1'b1;
                        end
                    end else begin
                        brg_b <= brg_b - 16'd1;
                    end
                end
            end

            // ═════════════════ RX FIFO PUSH / POP (netted) ═════════════════
            // A push (WR14[4] local loopback of a completed TX byte, or an
            // external rx_x_valid byte per Z85C30 datasheet §3) and a CPU
            // data-port pop (RR8 read, rx_x_pop above) can land in the same
            // cycle.  Net level + slot placement in ONE case block per
            // channel instead of separate always-block writes stomping
            // each other — idiom per asc.v:537-546 / scsi.v:2301-2315.
            // Net effect of simultaneous push+pop: level unchanged, the
            // popped byte leaves, and the pushed byte lands at the
            // correct post-pop slot.
            if (rx_a_push && rx_a_pop) begin
                case (rxlvl_a)
                    2'd1: rxf_a0 <= rx_a_push_data;
                    2'd2: begin
                        rxf_a0 <= rxf_a1;
                        rxf_a1 <= rx_a_push_data;
                    end
                    2'd3: begin
                        rxf_a0 <= rxf_a1;
                        rxf_a1 <= rxf_a2;
                        rxf_a2 <= rx_a_push_data;
                    end
                    default: ; // rxlvl_a==0 can't pop; rx_a_pop gates on !=0
                endcase
                // Level nets to unchanged (pop -1, push +1); a byte is
                // still present, so the IP latch follows push semantics.
                if (rx_ie_a)
                    rx_ip_a <= 1'b1;
            end else if (rx_a_push) begin
                if (rxlvl_a == 2'd3) begin
                    rxov_a <= 1'b1;
                    if (rx_a_ext_push)
                        rxf_a2 <= rx_a_data;   // external overflow overwrites tail
                end else begin
                    case (rxlvl_a)
                        2'd0: rxf_a0 <= rx_a_push_data;
                        2'd1: rxf_a1 <= rx_a_push_data;
                        2'd2: rxf_a2 <= rx_a_push_data;
                        default: ;
                    endcase
                    rxlvl_a <= rxlvl_a + 2'd1;
                    if (rx_ie_a)
                        rx_ip_a <= 1'b1;
                end
            end else if (rx_a_pop) begin
                rxf_a0  <= rxf_a1;
                rxf_a1  <= rxf_a2;
                rxlvl_a <= rxlvl_a - 2'd1;
                if (rxlvl_a == 2'd1)
                    rx_ip_a <= 1'b0;
            end

            if (rx_b_push && rx_b_pop) begin
                case (rxlvl_b)
                    2'd1: rxf_b0 <= rx_b_push_data;
                    2'd2: begin
                        rxf_b0 <= rxf_b1;
                        rxf_b1 <= rx_b_push_data;
                    end
                    2'd3: begin
                        rxf_b0 <= rxf_b1;
                        rxf_b1 <= rxf_b2;
                        rxf_b2 <= rx_b_push_data;
                    end
                    default: ;
                endcase
                if (rx_ie_b)
                    rx_ip_b <= 1'b1;
            end else if (rx_b_push) begin
                if (rxlvl_b == 2'd3) begin
                    rxov_b <= 1'b1;
                    if (rx_b_ext_push)
                        rxf_b2 <= rx_b_data;   // external overflow overwrites tail
                end else begin
                    case (rxlvl_b)
                        2'd0: rxf_b0 <= rx_b_push_data;
                        2'd1: rxf_b1 <= rx_b_push_data;
                        2'd2: rxf_b2 <= rx_b_push_data;
                        default: ;
                    endcase
                    rxlvl_b <= rxlvl_b + 2'd1;
                    if (rx_ie_b)
                        rx_ip_b <= 1'b1;
                end
            end else if (rx_b_pop) begin
                rxf_b0  <= rxf_b1;
                rxf_b1  <= rxf_b2;
                rxlvl_b <= rxlvl_b - 2'd1;
                if (rxlvl_b == 2'd1)
                    rx_ip_b <= 1'b0;
            end

            // ═════════════════ WR9[7:6] SELF-CLEAR ═════════════════
            // The reset-command bits auto-clear after one cycle.
            if (wra[9][7:6] != 2'b00) wra[9][7:6] <= 2'b00;
            if (wrb[9][7:6] != 2'b00) wrb[9][7:6] <= 2'b00;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            pb_rdata <= 8'h00;
        end else if (pb_rd) begin
            if (sel_data) begin
                if (sel_a)
                    pb_rdata <= (rxlvl_a != 2'd0) ? rxf_a0 : 8'h00;
                else
                    pb_rdata <= (rxlvl_b != 2'd0) ? rxf_b0 : 8'h00;
            end else begin
                pb_rdata <= sel_a ? rd_byte_a : rd_byte_b;
            end
        end else begin
            pb_rdata <= 8'h00;
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!rst && (LOG_PROBES != 0) && (pb_wr || pb_rd)) begin
            if (pb_wr) begin
                $display("[scc-probe] cyc=%0d wr ch=%s port=%s addr=0x%0h data=0x%02x ptr=%0d rr0A=0x%02x rr0B=0x%02x irq=%0d",
                         log_cycle, sel_a ? "A" : "B", sel_data ? "data" : "ctrl",
                         pb_addr, pb_wdata, ptr, rr0_a, rr0_b, irq);
            end else begin
                $display("[scc-probe] cyc=%0d rd ch=%s port=%s addr=0x%0h data=0x%02x ptr=%0d rr0A=0x%02x rr0B=0x%02x irq=%0d",
                         log_cycle, sel_a ? "A" : "B", sel_data ? "data" : "ctrl",
                         pb_addr, pb_read_preview, ptr, rr0_a, rr0_b, irq);
            end
        end
    end
`endif

    // ── IRQ line ────────────────────────────────────────────────────
    wire any_ip_a = (tx_ip_a && tx_ie_a) |
                    (rx_ip_a && rx_ie_a) |
                    (ext_ip_a && ex_ie_a);
    wire any_ip_b = (tx_ip_b && tx_ie_b) |
                    (rx_ip_b && rx_ie_b) |
                    (ext_ip_b && ex_ie_b);
    assign irq = mie && (any_ip_a || any_ip_b);

endmodule

`default_nettype wire
