// MAME reference/excerpt/adaptation attribution: Copyright Peter Trauner; Mathis Rosenhauer.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// via2.v — Mac VIA2 (6522) peripheral (full Quadra 700 variant)
//
// Upgrade of the phase-2 minimum-viable stub to a full 6522 matching
// the feature set of `via1.v`.  Task #115.
// The VIA implementation drew inspiration from Shachar Shemesh's CompuSAR/6522 project.
// See THIRD_PARTY_NOTICES.md.
//
// On a Quadra 700, VIA2 services:
//   * NuBus slot interrupts — slots $9..$F aggregated onto PA[6:0]
//     (active-low per-slot sense lines; PA7 is hard-wired high on Q700).
//     Slot activity additionally triggers
//     CA1 (IFR bit 1) so a single line re-vector tells the CPU
//     "some slot is requesting service".
//   * PB status + control bits — PB[0]=VFC (vertical-frame-capture
//     mode from DAFB), PB[1]=SNDEXT (sound-extended), PB[2]=TM0A,
//     PB[3]=TM1A (test-mode bits), PB[4:6] reserved, PB[7]
//     Timer-1 PB7 output mode.
//   * Full 6522 feature set — ORA/ORB + DDR, T1 (one-shot / free-run
//     / PB7 pulse / PB7 square wave), T2 (timed / pulse-count),
//     shift register (external/T2/phi2-clock modes), IFR+IER with
//     write-1-clear / MSB set-clear semantics, CA1/CA2/CB1/CB2
//     edge/level control per PCR.
//
// Reference:
//   MOS 6522 VIA datasheet (MOS Technology 1977);
//   Apple "Guide to the Macintosh Family Hardware" §12 (VIA2);
//   MAME `src/devices/machine/6522via.cpp` + `src/mame/apple/macquadra.cpp`.
//
// Register map (A3..A0 == pb_addr[3:0]):
//    0  ORB      — port B output / input
//    1  ORA      — port A output / input (with CA2 handshake)
//    2  DDRB     — data direction B    (1=output)
//    3  DDRA     — data direction A
//    4  T1CL     — T1 counter low  (read)  / latch low  (write)
//    5  T1CH     — T1 counter high (read)  / latch high (write, starts T1)
//    6  T1LL     — T1 latch low
//    7  T1LH     — T1 latch high (write clears IFR[T1])
//    8  T2CL     — T2 counter low  (read)  / latch low  (write)
//    9  T2CH     — T2 counter high (read)  / latch high (write, starts T2)
//   10  SR       — shift register
//   11  ACR      — aux control  (T1 mode [7:6], T2 mode [5], SR [4:2])
//   12  PCR      — peripheral control (CA1 [0], CA2 [3:1], CB1 [4], CB2 [7:5])
//   13  IFR      — interrupt flags  (read: raw; write: write-1-clears)
//   14  IER      — interrupt enable (bit 7 = set/clear, [6:0] = mask)
//   15  ORA-NH   — port A without handshake (no CA2 pulse)
//
// IFR bit positions (standard 6522):
//   0 = CA2
//   1 = CA1  (NuBus slot IRQ aggregate — any PA slot-line falling edge)
//   2 = SR
//   3 = CB2
//   4 = CB1
//   5 = T2
//   6 = T1
//
// Peripheral bus:
//   pb_wr / pb_rd are one-cycle pulses; pb_ack is registered one
//   cycle after the request (matches via1.v timing).  pb_rdata is
//   registered.
//
// Clocking:
//   All state on `clk` (pb_clk); the TIMERS (T1/T2) and the shift
//   register are gated by `phi2_tick` (783 360 Hz), matching via1.v so
//   both VIAs share one timebase.
//
//   The CA1/CA2/CB1/CB2 EXTERNAL-PIN EDGE LATCHES are deliberately NOT
//   phi2-gated — they sample every `clk`, exactly as via1.v's already
//   do.  A device interrupt level latched into IFR later than the CPU
//   access that services it strands forever in PCR independent mode;
//   that was the 2026-09-15 53C96 level-2 exception storm.  See the
//   load-bearing comment on the edge-latch block below and
//   tb/tb_via2.cpp scenario 24.
//
// Reset values (Mac convention — Inside Macintosh: Hardware ch.7):
//   ORB    = 0x00
//   ORA    = 0x00
//   DDRB   = 0x00  (all inputs)
//   DDRA   = 0x00  (all inputs — slot-IRQ sensing requires inputs)
//   all other regs = 0.

module via2 (
    input  wire        clk,
    input  wire        rst,

    // 1 MHz VIA timebase (one-cycle pulse every ~200 clk edges)
    input  wire        phi2_tick,

    // Peripheral-bus slave
    input  wire [3:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output reg  [7:0]  pb_rdata,
    output reg         pb_ack,

    // ── Port A pins ──────────────────────────────────────────────────
    //
    // PA[6:0] = NuBus slot IRQs from slots $9..$F (active-low).  Idle
    // state is all-high; a slot asserts low when it requires service.
    // PA[6] is the built-in video slot ($F) in this SoC integration.
    // PA[7] is hard-wired high.
    // The top-level supplies pa_in with the raw slot sense lines; the
    // VIA samples them on every clk for edge detection on CA1 (NOT on
    // phi2_tick — see the Clocking note in the header).
    input  wire [7:0]  pa_in,
    output wire [7:0]  pa_out,
    output wire [7:0]  pa_mask,

    // ── Port B pins ──────────────────────────────────────────────────
    //
    // PB[0] = VFC  (vertical-frame-capture; DAFB status, idle 0)
    // PB[1] = SNDEXT (sound-extended — input from audio path)
    // PB[2] = TM0A   (test mode — Q700 descriptor idle high)
    // PB[3] = TM1A   (test mode — tie low at board level)
    // PB[4] = RAM-parity reset out (output when DDRB[4]=1)
    // PB[5] = RAM-parity fault in  (input)
    // PB[6] = reserved/status input
    // PB[7] = Timer-1 PB7 output  (when ACR[7]=1)
    //
    // Any bits set as outputs in DDRB return orb on read; inputs return
    // pb_in.  The top level wires DAFB + audio + ram-parity to pb_in.
    input  wire [7:0]  pb_in,
    output wire [7:0]  pb_out,
    output wire [7:0]  pb_mask,

    // ── CA / CB control pins ─────────────────────────────────────────
    //
    // CA1 on the Quadra 700 is driven internally by the slot-IRQ
    // aggregate — any falling edge on PA[6:0] that was previously
    // high triggers CA1 edge detection.  It is still exposed as an
    // input so board-level test harnesses can inject an edge directly.
    // CA2/CB2 are available for future slot-manager use; tie off at
    // the board for now.  CB1 is used for edge-latch coverage and SR
    // clocking mode selection.
    input  wire        ca1_in,
    input  wire        ca2_in,
    input  wire        cb1_in,
    input  wire        cb2_in,
    output wire        ca2_out,
    output wire        cb1_out,
    output wire        cb2_out,

    // Aggregated IRQ line to irq_agg.v (level 2)
    output wire        irq
);

    // ── Register file ─────────────────────────────────────────────────
    reg [7:0]  orb;          //  0
    reg [7:0]  ora;          //  1
    reg [7:0]  ddrb;         //  2
    reg [7:0]  ddra;         //  3
    reg [15:0] t1c;          //  4/5 counter
    reg [15:0] t1l;          //  6/7 latch
    reg [15:0] t2c;          //  8/9 counter
    reg [7:0]  t2l_lo;       //     T2 latch low  (T2 high written straight to t2c[15:8])
    reg [7:0]  sr;           // 10
    reg [7:0]  acr;          // 11
    reg [7:0]  pcr;          // 12
    reg [6:0]  ifr;          // 13 (bit 7 derived)
    reg [6:0]  ier;          // 14 (bit 7 = set/clear selector, not stored)

    // Timer arm + state
    reg        t1_armed;
    reg        t2_armed;
    reg        t1_pb7;           // Timer-1 PB7 output flip-flop
    reg        t1_fired_once;    // One-shot has already fired?
    reg        pb6_prev;         // PB6 pulse-count edge detector
    reg        ca2_prev;         // CA2 independent-IRQ edge detector
    reg        cb2_prev;         // CB2 independent-IRQ edge detector

    // Shift register: bit counter + activity flag
    reg  [3:0] sr_bits_left;
    reg        sr_active;

    // CA1 edge detector — latches slot-IRQ aggregate.
    reg  [6:0] pa_prev;       // previous sampled slot IRQ inputs
    reg        ca1_prev;      // previous sampled ca1_in
    reg        cb1_prev;      // previous sampled cb1_in

    // ── IFR bit positions (standard 6522) ─────────────────────────────
    localparam [2:0] IFR_CA2 = 3'd0;
    localparam [2:0] IFR_CA1 = 3'd1;
    localparam [2:0] IFR_SR  = 3'd2;
    localparam [2:0] IFR_CB2 = 3'd3;
    localparam [2:0] IFR_CB1 = 3'd4;
    localparam [2:0] IFR_T2  = 3'd5;
    localparam [2:0] IFR_T1  = 3'd6;

    // IFR summary bit — mirrors real 6522: bit 7 is the OR of (ifr & ier).
    wire       ifr_any = |(ifr & ier);
    wire [7:0] ifr_rd  = {ifr_any, ifr};

    // ── ACR decode ───────────────────────────────────────────────────
    // ACR[7:6] = Timer 1 mode:
    //   00  one-shot, no PB7 output
    //   01  free-run (continuous), no PB7 output
    //   10  one-shot, PB7 goes low for one phi2 period on wrap
    //   11  free-run, PB7 toggles on each wrap (square wave)
    // ACR[5]   = Timer 2 mode:
    //   0   timed interrupt (one-shot)
    //   1   pulse-count — decrement on PB6 falling edge (Mac stubs this
    //       off; we accept writes but leave counting gated by phi2 to
    //       keep the behaviour deterministic)
    // ACR[4:2] = Shift register mode:
    //   000 disabled
    //   001 shift in   under T2
    //   010 shift in   under phi2
    //   011 shift in   under external clock (CB1)
    //   100 shift out  free-run under T2
    //   101 shift out  under T2
    //   110 shift out  under phi2
    //   111 shift out  under external clock
    // ACR[1]  = PB latch enable (ignored here — we read live pb_in)
    // ACR[0]  = PA latch enable (ignored)

    wire       acr_t1_continuous = acr[6];
    wire       acr_t1_pb7        = acr[7];
    wire       acr_t2_pulsecount = acr[5];
    wire [2:0] acr_sr_mode       = acr[4:2];
    wire       acr_sr_enable     = (acr_sr_mode != 3'b000);
    wire       acr_sr_out        = acr_sr_mode[2];       // 1 = shift-out direction

    // ── PCR decode ───────────────────────────────────────────────────
    // PCR[0]   = CA1 control:      0 = negative-edge, 1 = positive-edge
    // PCR[3:1] = CA2 control
    // PCR[4]   = CB1 control
    // PCR[7:5] = CB2 control
    // For the Q700 VIA2 the ROM programs CA1 for negative-edge slot-IRQ
    // latching (slots drive their sense line low).  We implement the
    // edge-selector; the handshake / pulse / manual modes of CA2/CB2
    // drive out the corresponding _out pins combinationally below.
    wire       pcr_ca1_pos_edge = pcr[0];
    wire       pcr_ca2_pos_edge = pcr[2];
    wire       pcr_cb1_pos_edge = pcr[4];
    wire       pcr_cb2_pos_edge = pcr[6];
    wire       pcr_ca2_ind_irq  = ((pcr & 8'h0a) == 8'h02);
    wire       pcr_cb2_ind_irq  = ((pcr & 8'ha0) == 8'h20);

    // CA2/CB2 idle high unless configured as outputs.
    assign     ca2_out = pcr[3] ? pcr[1] : 1'b1;
    assign     cb2_out = pcr[7] ? pcr[5] : 1'b1;
    // CB1 is shift-clock when SR is in phi2/T2 out mode; otherwise high.
    assign     cb1_out = (acr_sr_mode == 3'b101) || (acr_sr_mode == 3'b110) ||
                         (acr_sr_mode == 3'b100) ? 1'b0 : 1'b1;

    // Unused PCR / CA-CB inputs (kept for future manager wiring).
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_ca2 = ca2_in;
    wire _unused_cb2 = cb2_in;
    /* verilator lint_on UNUSEDSIGNAL */

    // ── Slot-IRQ / CA1 edge detection ────────────────────────────────
    //
    // On Q700 the NuBus slots drive PA[6:0] low when asserting IRQ.
    // CA1 edge-detection (per PCR[0]) latches into IFR[CA1] on the
    // programmed edge.  MAME drives the VIA CA1 pin explicitly from
    // nubus_slot_interrupt(); this RTL also watches the PA slot-state bits
    // so the standalone VIA model catches the same active-low slot event.
    // We OR two sources:
    //   (a) the bit-wise edge on any PA[6:0] matching the PCR[0] edge
    //       polarity (the slot-aggregate path);
    //   (b) the raw ca1_in external port (for direct stimulation).
    //
    // Sampled EVERY pb_clk, not on phi2_tick — see the load-bearing
    // comment on the edge-latch block in the always below (an IFR flag
    // that is latched later than the CPU access meant to clear it strands
    // forever in PCR independent mode).
    wire [6:0] pa_slots      = pa_in[6:0];
    wire [6:0] pa_slots_prev = pa_prev;
    wire [6:0] pa_neg_edge   = pa_slots_prev & ~pa_slots;  // 1->0 transitions
    wire [6:0] pa_pos_edge   = ~pa_slots_prev & pa_slots;  // 0->1 transitions
    wire       slot_edge     = pcr_ca1_pos_edge ? |pa_pos_edge : |pa_neg_edge;
    wire       ca1_edge_ext  = pcr_ca1_pos_edge ? (ca1_in && !ca1_prev)
                                                : (!ca1_in && ca1_prev);
    wire       ca1_trigger   = slot_edge || ca1_edge_ext;
    wire       cb1_edge_ext  = pcr_cb1_pos_edge ? (cb1_in && !cb1_prev)
                                                : (!cb1_in && cb1_prev);
    wire       ca2_edge_ext  = pcr_ca2_pos_edge ? (ca2_in && !ca2_prev)
                                                : (!ca2_in && ca2_prev);
    wire       cb2_edge_ext  = pcr_cb2_pos_edge ? (cb2_in && !cb2_prev)
                                                : (!cb2_in && cb2_prev);

    // ── Write / state advance ────────────────────────────────────────
    always @(posedge clk) begin
        if (rst) begin
            orb        <= 8'h00;
            ora        <= 8'h00;
            ddrb       <= 8'h00;
            ddra       <= 8'h00;
            t1c        <= 16'h0000;
            t1l        <= 16'h0000;
            t2c        <= 16'h0000;
            t2l_lo     <= 8'h00;
            sr         <= 8'h00;
            acr        <= 8'h00;
            pcr        <= 8'h00;
            ifr        <= 7'h00;
            ier        <= 7'h00;
            t1_armed   <= 1'b0;
            t2_armed   <= 1'b0;
            t1_pb7     <= 1'b1;
            t1_fired_once <= 1'b0;
            sr_bits_left  <= 4'd0;
            sr_active     <= 1'b0;
            pb6_prev      <= pb_in[6];
            ca2_prev      <= ca2_in;
            cb2_prev      <= cb2_in;
            // Slot IRQs are active-low levels feeding an edge-latched VIA.
            // Start the previous samples at idle-high so a slot line that is
            // already asserted when reset releases still latches CA1 on the
            // first phi2_tick.  This matters for debug resets where DAFB
            // Swatch control state can survive and reassert the cursor IRQ
            // before VIA2 has observed an idle slot line.
            pa_prev    <= 7'h7F;
            ca1_prev   <= 1'b1;
            cb1_prev   <= cb1_in;
        end else begin
            // ─── CPU write ───────────────────────────────────────────
            if (pb_wr) begin
                case (pb_addr)
                    4'd0:  begin
                        orb <= pb_wdata;                                // ORB
                        // Writing ORB acknowledges PB-side handshakes.
                        ifr[IFR_CB1] <= 1'b0;
                        if (!pcr_cb2_ind_irq)
                            ifr[IFR_CB2] <= 1'b0;
                    end
                    4'd1:  begin
                        ora <= pb_wdata;                                // ORA (w/ hs)
                        // Writing ORA clears IFR[CA1]/[CA2] (handshake)
                        ifr[IFR_CA1] <= 1'b0;
                        if (!pcr_ca2_ind_irq)
                            ifr[IFR_CA2] <= 1'b0;
                    end
                    4'd2:  ddrb     <= pb_wdata;                       // DDRB
                    4'd3:  ddra     <= pb_wdata;                       // DDRA

                    // T1CL write → T1 latch low (not counter)
                    4'd4:  t1l[7:0] <= pb_wdata;
                    // T1CH write → T1 latch high + counter load + clear IFR,
                    // arm timer, reset one-shot-fired flag.  Per 6522
                    // spec, writing T1CH in PB7-pulse mode (ACR[7:6]=10)
                    // drops PB7 low; it returns high on underflow.
                    4'd5:  begin
                        t1l[15:8]     <= pb_wdata;
                        t1c           <= {pb_wdata, t1l[7:0]};
                        ifr[IFR_T1]   <= 1'b0;
                        t1_armed      <= 1'b1;
                        t1_fired_once <= 1'b0;
                        // Timer-output mode drives PB7 low when T1CH arms
                        // the counter; continuous mode then toggles it on
                        // each underflow.
                        if (acr_t1_pb7)
                            t1_pb7 <= 1'b0;
                    end
                    // T1 latch aliases — do not arm; high-write clears IFR.
                    4'd6:  t1l[7:0]  <= pb_wdata;
                    4'd7:  begin
                        t1l[15:8]    <= pb_wdata;
                        ifr[IFR_T1]  <= 1'b0;
                    end

                    // T2CL = latch low; T2CH = load counter + arm + clear IFR.
                    4'd8:  t2l_lo    <= pb_wdata;
                    4'd9:  begin
                        t2c          <= {pb_wdata, t2l_lo};
                        ifr[IFR_T2]  <= 1'b0;
                        t2_armed     <= 1'b1;
                    end

                    // SR write — clears IFR[SR]; if ACR is in a shift-out
                    // mode, kick off an 8-bit shift.
                    4'd10: begin
                        sr          <= pb_wdata;
                        ifr[IFR_SR] <= 1'b0;
                        if (acr_sr_enable) begin
                            sr_bits_left <= 4'd8;
                            sr_active    <= 1'b1;
                        end
                    end

                    4'd11: acr       <= pb_wdata;
                    4'd12: pcr       <= pb_wdata;

                    // IFR: write-1-clears (low 7 bits).  Bit 7 read-only.
                    4'd13: ifr       <= ifr & ~pb_wdata[6:0];

                    // IER: MSB=1 sets bits; MSB=0 clears bits.
                    4'd14: begin
                        if (pb_wdata[7]) ier <= ier |  pb_wdata[6:0];
                        else             ier <= ier & ~pb_wdata[6:0];
                    end

                    // ORA-no-handshake — write ORA without touching CA
                    // IFR bits (distinct from reg 1 which clears them).
                    4'd15: ora       <= pb_wdata;
                    default: ;
                endcase
            end

            // ─── Side-effect-ful reads ───────────────────────────────
            //
            //  reading ORA    (reg 1)  clears IFR[CA1] and IFR[CA2]
            //  reading ORB    (reg 0)  clears IFR[CB1] and IFR[CB2]
            //                          (6522 convention — via1 shares it)
            //  reading T1CL   (reg 4)  clears IFR[T1]
            //  reading T2CL   (reg 8)  clears IFR[T2]
            //  reading SR     (reg 10) clears IFR[SR]
            //  reading ORA-NH (reg 15) does NOT touch IFR (no handshake)
            if (pb_rd && !pb_wr) begin
                case (pb_addr)
                    4'd0:  begin
                        ifr[IFR_CB1] <= 1'b0;
                        if (!pcr_cb2_ind_irq)
                            ifr[IFR_CB2] <= 1'b0;
                    end
                    4'd1:  begin
                        ifr[IFR_CA1] <= 1'b0;
                        if (!pcr_ca2_ind_irq)
                            ifr[IFR_CA2] <= 1'b0;
                    end
                    4'd4:  ifr[IFR_T1]  <= 1'b0;
                    4'd8:  ifr[IFR_T2]  <= 1'b0;
                    4'd10: ifr[IFR_SR]  <= 1'b0;
                    default: ;
                endcase
            end

            // ─── Timer 1 countdown (phi2-gated) ──────────────────────
            // Gated against a same-cycle CPU write to T1CH (reg 5): the
            // CPU-write branch above and this countdown live in the same
            // always block with the countdown ordered last, so without
            // this gate a T1CH write that lands on the same clk edge as
            // phi2_tick would be silently overridden by the countdown's
            // non-blocking assignment (wrap-to-zero could even set
            // IFR_T1 / disarm a one-shot the CPU just rearmed).
            if (phi2_tick && t1_armed && !(pb_wr && pb_addr == 4'd5)) begin
                if (t1c == 16'h0000) begin
                    if (acr_t1_continuous) begin
                        t1c           <= t1l;
                        ifr[IFR_T1]   <= 1'b1;
                        // Square-wave mode: toggle PB7 each underflow.
                        if (acr_t1_pb7) t1_pb7 <= ~t1_pb7;
                    end else begin
                        // One-shot: fire exactly once per arm; after
                        // that counter free-runs but we don't re-fire.
                        t1c <= 16'hFFFF;
                        if (!t1_fired_once) begin
                            ifr[IFR_T1]   <= 1'b1;
                            t1_fired_once <= 1'b1;
                            // PB7 pulse mode: return high on underflow.
                            if (acr_t1_pb7) t1_pb7 <= 1'b1;
                        end
                    end
                end else begin
                    t1c <= t1c - 16'd1;
                end
            end

            // ─── Timer 2 countdown ──────────────────────────────────
            // ACR[5]=0: timed interrupt (one-shot) on phi2.
            // ACR[5]=1: pulse-count on PB6 falling edges.
            // Gated against a same-cycle CPU write to T2CH (reg 9) for
            // the same reason as the T1 gate above.
            if (phi2_tick && t2_armed && !acr_t2_pulsecount && !(pb_wr && pb_addr == 4'd9)) begin
                if (t2c == 16'h0000) begin
                    ifr[IFR_T2] <= 1'b1;
                    t2c         <= 16'hFFFF;
                    t2_armed    <= 1'b0;
                end else begin
                    t2c <= t2c - 16'd1;
                end
            end
            if (t2_armed && acr_t2_pulsecount && pb6_prev && !pb_in[6]) begin
                if (t2c == 16'h0000) begin
                    ifr[IFR_T2] <= 1'b1;
                    t2c         <= 16'hFFFF;
                    t2_armed    <= 1'b0;
                end else begin
                    t2c <= t2c - 16'd1;
                end
            end

            // ─── Shift register — fires IFR_SR after 8 bits ─────────
            if (phi2_tick && sr_active && acr_sr_enable) begin
                if (sr_bits_left == 4'd1) begin
                    sr_bits_left <= 4'd0;
                    sr_active    <= 1'b0;
                    ifr[IFR_SR]  <= 1'b1;
                    // For shift-in under phi2/T2 modes we consume the
                    // external CB2 line — tied 0 at the board, so SR
                    // just rotates its own content.
                    if (!acr_sr_out) sr <= {sr[6:0], cb2_in};
                    else             sr <= {sr[6:0], sr[7]};
                end else if (sr_bits_left != 4'd0) begin
                    sr_bits_left <= sr_bits_left - 4'd1;
                    if (!acr_sr_out) sr <= {sr[6:0], cb2_in};
                    else             sr <= {sr[6:0], sr[7]};
                end
            end

            // ─── External input edge detection (CA1 slot aggregate,
            //     CA2, CB1, CB2) — EVERY pb_clk, NOT phi2-gated ───────
            //
            // ⚠️ THIS SAMPLING RATE IS LOAD-BEARING.  Do not "restore"
            //    the phi2_tick gate this block used to carry.
            //
            // WHY (root cause of the 2026-09-15 level-2 exception storm,
            // ~277k exc/s with the 53C96 idle):
            //
            // These four pins carry DEVICE INTERRUPT LEVELS — CB2 is the
            // 53C96 IRQ (fpga_top_peripherals.vh:1763, `.cb2_in
            // (~scsi_irq_pb)`, matching MAME macquadra700.cpp:773), CB1
            // is the ASC, CA1 the NuBus slot aggregate.  The 6522 turns
            // those LEVELS into EDGE-LATCHED IFR flags, and in the
            // PCR=0x22 independent mode the Mac programs, ONLY a write to
            // IFR clears the flag (see the reg-13 write arm and the reg-0
            // read/write arms above).
            //
            // The driver's measured ISR/poll protocol (captured from a
            // healthy MAME 7.5.3 boot, tools-side tap on VIA2 + 53C96) is
            //
            //     C96 R4  -> 0x80   ; 53C96 status, S_INTR set
            //     VIA2 W IFR <- 0x88; clear IFR.CB2   <-- clears FIRST
            //     C96 R6            ; seq step
            //     C96 R5  -> 0x20   ; istatus read: the 53C96 IRQ FALLS
            //
            // and — measured over a whole boot — the ISR clears IFR.CB2
            // ONLY when the 53C96 says S_INTR (9/9 CB2 interrupt entries).
            // So an IFR.CB2 flag that outlives its source is FATAL: the
            // ISR reads 53C96 status, sees 0x00, claims nothing, never
            // writes IFR, and the level-sensitive via2_irq re-fires
            // forever.  irq_agg's strict priority then starves the 60 Hz
            // VIA1 tick and the machine livelocks.
            //
            // With the edge detector gated on phi2_tick, IFR.CB2 was set
            // up to ONE PHI2 PERIOD (1/783360 s = 1.28 us = ~64 pb_clk
            // cycles) AFTER the 53C96 raised its IRQ.  On a real Q700
            // that can't hurt: the 6522 is clocked BY phi2, so every CPU
            // access to it costs a full phi2 cycle and the latch is
            // always ordered before the CPU access that follows the
            // interrupt.  Here the CPU reaches VIA2 through pb_clk at
            // 50 MHz and the whole `R4 / W IFR / R6 / R5` sequence fits
            // inside one phi2 period — so the deferred latch could land
            // in the window BETWEEN the driver's IFR clear and its
            // istatus read, and the istatus read then removed the only
            // evidence that the flag was real.  Stranded flag, idle chip,
            // permanent ipl=2.
            //
            // Sampling every pb_clk restores the ordering unconditionally
            // (the flag is set within one pb_clk of the pin moving, i.e.
            // always before any CPU access that could observe the same
            // interrupt through the device) and matches MAME's 6522,
            // which sets INT_CB2 synchronously inside write_cb2()
            // (6522via.cpp:1204-1220) with no phi2 quantisation at all.
            //
            // The 6522 TIMERS (T1/T2/SR) stay on phi2_tick above — they
            // are the parts that really do count phi2 cycles.  Only the
            // external-pin edge latches move.
            //
            // Ordering inside this always block is unchanged and still
            // load-bearing: this block runs AFTER the CPU write/read arms,
            // so an edge that lands on the same pb_clk as the ISR's IFR
            // write still wins and the interrupt is not lost.
            pa_prev  <= pa_in[6:0];
            ca1_prev <= ca1_in;
            cb1_prev <= cb1_in;
            if (ca1_trigger) ifr[IFR_CA1] <= 1'b1;
            if (cb1_edge_ext) ifr[IFR_CB1] <= 1'b1;
            if (pcr_ca2_ind_irq && ca2_edge_ext)
                ifr[IFR_CA2] <= 1'b1;
            if (pcr_cb2_ind_irq && cb2_edge_ext)
                ifr[IFR_CB2] <= 1'b1;
            ca2_prev <= ca2_in;
            cb2_prev <= cb2_in;

            pb6_prev <= pb_in[6];
        end
    end

    // ── Read path ────────────────────────────────────────────────────
    //
    // ORB read: bits with DDRB=1 return orb; bits with DDRB=0 return pb_in.
    // PB[7] is overridden to t1_pb7 when ACR[7]=1 (Timer-1 PB7 mode) —
    // the 6522 forces the PB7 output line even when DDRB[7]=0.
    wire [7:0] orb_rd_raw = (orb & ddrb) | (pb_in & ~ddrb);
    wire [7:0] orb_rd     = acr_t1_pb7 ? {t1_pb7, orb_rd_raw[6:0]}
                                       : orb_rd_raw;
    wire [7:0] ora_rd     = (ora & ddra) | (pa_in & ~ddra);

    always @(posedge clk) begin
        if (rst) begin
            pb_rdata <= 8'h00;
            pb_ack   <= 1'b0;
        end else begin
            pb_ack   <= pb_wr | pb_rd;
            if (pb_rd) begin
                case (pb_addr)
                    4'd0:    pb_rdata <= orb_rd;
                    4'd1:    pb_rdata <= ora_rd;
                    4'd2:    pb_rdata <= ddrb;
                    4'd3:    pb_rdata <= ddra;
                    4'd4:    pb_rdata <= t1c[7:0];
                    4'd5:    pb_rdata <= t1c[15:8];
                    4'd6:    pb_rdata <= t1l[7:0];
                    4'd7:    pb_rdata <= t1l[15:8];
                    4'd8:    pb_rdata <= t2c[7:0];
                    4'd9:    pb_rdata <= t2c[15:8];
                    4'd10:   pb_rdata <= sr;
                    4'd11:   pb_rdata <= acr;
                    4'd12:   pb_rdata <= pcr;
                    4'd13:   pb_rdata <= ifr_rd;
                    4'd14:   pb_rdata <= {1'b1, ier};
                    4'd15:   pb_rdata <= ora_rd;
                    default: pb_rdata <= 8'h00;
                endcase
            end else begin
                pb_rdata <= 8'h00;
            end
        end
    end

    // ── Port outputs ─────────────────────────────────────────────────
    //
    // PA / PB output values are the raw ORA / ORB; the direction mask
    // is DDRA / DDRB so upstream logic can tri-state accordingly.
    // For PB, overlay the PB7 timer output into pb_out so the board
    // sees the timer waveform on the same pin DDRB[7] would drive.
    assign pa_out  = ora;
    assign pa_mask = ddra;
    assign pb_out  = acr_t1_pb7 ? {t1_pb7, orb[6:0]} : orb;
    assign pb_mask = acr_t1_pb7 ? (ddrb | 8'b1000_0000) : ddrb;

    // ── IRQ output ───────────────────────────────────────────────────
    assign irq = ifr_any;

endmodule
