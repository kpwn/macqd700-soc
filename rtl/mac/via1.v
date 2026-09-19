// MAME reference/excerpt/adaptation attribution: Copyright Peter Trauner; Mathis Rosenhauer.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// via1.v — Mac VIA1 (6522) peripheral (full-boot variant)
//
// The VIA implementation drew inspiration from Shachar Shemesh's CompuSAR/6522 project.
// See THIRD_PARTY_NOTICES.md for component licensing and attribution.
//
// Ported and EXTENDED by the m68k-ooo project with:
//   * Peripheral-bus slave interface (matches via2/scc/asc shape).
//   * Full register file (16 regs) with standard 6522 read/write quirks.
//   * Timer 1 + Timer 2 countdown (gated by phi2_tick) + IFR set on wrap.
//   * Timer 2 pulse-count mode on PB6 edges.
//   * IFR / IER with read-clear / write-1-clear semantics + MSB-set write.
//   * Mac VIA1 ORB bit 3 overlay output, reset values per
//     docs/peripheral_arch.md.
//   * ADB shift register: SR-attention IRQ (IFR bit 2) + external byte-
//     level transceiver interface (adb_rx_* / adb_tx_*) + deterministic
//     idle-high empty-bus completion when no transceiver is wired.
//   * RTC side-channel: PB0 (rtcData), PB1 (rtcClk), PB2 (rtcEnb) fan-out
//     to the off-chip rtc.v peripheral.
//   * RTC clock-output input on CA2 (IFR bit 0), matching the Q700 board
//     wiring used by MAME.
//   * Minimal ADB modem idle-line source on CB2, matching the Q700 wiring
//     where the ADB modem PIC drives VIA1 CB1/CB2.  This is deliberately
//     below the byte-level ADB transport: it only creates the idle CB2 edge
//     that the ROM expects while no real ADB phy/modem is wired.
//   * CA1 edge input (IFR bit 1), with an optional standalone 60 Hz tick
//     for unit/bring-up tests.  Platform tops feed the Q700 VIA2 PB7
//     60.15 Hz chain through this pin.
//
// Reference: MOS/Rockwell/WDC 6522 VIA datasheet (MOS Technology, 1977);
//            Inside Macintosh: Hardware chapter 7 (ADB) and Device Manager
//            chapter 10 (real-time clock).
//
// Register map (A3..A0):
//   0  ORB     — port B output / input (ORB bit 3 = Mac ROM overlay)
//   1  ORA     — port A output / input (handshake)
//   2  DDRB    — data direction B   (1=output)
//   3  DDRA    — data direction A
//   4  T1CL    — timer 1 counter low  (read)  / latch low  (write)
//   5  T1CH    — timer 1 counter high (read)  / latch high (write, starts)
//   6  T1LL    — timer 1 latch low
//   7  T1LH    — timer 1 latch high
//   8  T2CL    — timer 2 counter low  (read)  / latch low  (write)
//   9  T2CH    — timer 2 counter high (read)  / latch high (write, starts)
//  10  SR      — shift register (ADB serial byte)
//  11  ACR     — aux control (T1 mode at [7:6], T2 mode at [5], SR [4:2])
//  12  PCR     — peripheral control
//  13  IFR     — interrupt flags (read: raw; write: write-1-clears)
//  14  IER     — interrupt enable (bit 7=set/clear, [6:0]=mask)
//  15  ORA-NH  — port A w/o handshake
//
// IFR bit positions (standard 6522):
//   0 = CA2, 1 = CA1 (60 Hz VBL), 2 = SR (ADB byte complete),
//   3 = CB2, 4 = CB1, 5 = T2, 6 = T1
//
// Peripheral bus:
//   - pb_wr / pb_rd are one-cycle pulses; pb_ack rises the same cycle
//     write commits / one cycle after read request is latched.
//   - pb_rdata is registered.
//
// Clocking:
//   - All state lives on the 200 MHz `clk`.
//   - Timer + SR + VBL countdown are gated by `phi2_tick` (one-cycle
//     pulse at ~1 MHz).
//
// Reset values (Mac convention, docs/peripheral_arch.md):
//   ORB    = 0x80  (bit 7 idle-high ADB interrupt; overlay
//                   is asserted while DDRB[3]=0, then follows ORB[3] once
//                   driven)
//   DDRB   = 0x00
//   all other regs = 0.

module via1 (
    input  wire        clk,
    input  wire        rst,

    // 1 MHz VIA timebase (1-cycle pulse every ~200 clk edges)
    input  wire        phi2_tick,

    // Peripheral-bus slave
    input  wire [3:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output reg  [7:0]  pb_rdata,
    output reg         pb_ack,

    // Port A / B pins (stubbed for sim — open inputs default to 0)
    input  wire [7:0]  pa_in,
    // pb_in[7:5] follow the normal 6522 DDR mux.  MAME's Q700 model does
    // not synthesize RAM-size pins here; forcing these bits high changes
    // the ROM's memory-sizing path.
    input  wire [7:0]  pb_in,
    output wire [7:0]  pa_out,
    output wire [7:0]  pa_mask,
    output wire [7:0]  pb_out,
    output wire [7:0]  pb_mask,

    // Mac-specific: ROM overlay flop output (drives glue.v decode mux)
    output wire        overlay_bit,

    // ── ADB transceiver (byte-level interface to adb_phy.v) ───────────
    //
    // adb_rx_byte / adb_rx_valid: external ADB transceiver presents a
    //   fully-framed byte.  Captured into SR on the next phi2_tick; sets
    //   IFR bit 2 (SR-attention).  Host sees adb_rx_ready go low for one
    //   tick — the transceiver should drop adb_rx_valid immediately.
    // adb_tx_byte / adb_tx_valid: pulses when the CPU writes SR in TX
    //   mode (ACR[4:2] = 3'b110 or 3'b111 — "shift out" modes) — the
    //   transceiver latches adb_tx_byte that cycle.  IFR bit 2 also
    //   sets after the pulse to indicate "transfer complete".
    input  wire [7:0]  adb_rx_byte,
    input  wire        adb_rx_valid,
    output reg         adb_rx_ready,
    output reg  [7:0]  adb_tx_byte,
    output reg         adb_tx_valid,

    // ── ADB modem PIC CB1/CB2 bit-level shift interface ──────────────
    input  wire        cb1_in,
    input  wire        cb2_in,
    output wire        cb2_out,
    output wire        cb2_oe,

    // ── RTC side-channel (PB0=data bidir, PB1=clk, PB2=enb) ───────────
    //
    // The off-chip rtc.v is shifted using PB0/PB1/PB2 bit-banging from
    // Mac OS; this module just forwards the GPIO as named pins so the
    // top-level can wire rtc directly.  rtc_data_in is sampled back into
    // ORB bit 0 when DDRB[0]=0 (input) — the standard 6522 read path
    // already does this through pb_in[0].
    output wire        rtc_enb,     // = ORB[2] (driven straight through);
                                     // rtc.v treats the incoming line as
                                     // active-low chip-enable itself.
    output wire        rtc_clk,     // ORB[1]
    output wire        rtc_data_o,  // ORB[0] when DDRB[0]=1
    output wire        rtc_data_oe, // DDRB[0]
    input  wire        rtc_data_i,  // sampled into pb_in[0] upstream
    input  wire        rtc_cko,     // RTC CKO -> VIA1 CA2

    // External CA1 level.  PCR[0] selects the active edge; Q700 platform
    // tops drive this from VIA2 PB7.
    input  wire        vblank_irq_in,

    // Aggregated IRQ line
    output wire        irq
);

    // ── Register file ─────────────────────────────────────────────────
    reg [7:0] orb;          //  0
    reg [7:0] ora;          //  1
    reg [7:0] ddrb;         //  2
    reg [7:0] ddra;         //  3
    reg [15:0] t1c;         //  4/5 counter
    reg [15:0] t1l;         //  6/7 latch
    reg [15:0] t2c;         //  8/9 counter
    reg [7:0]  t2l_lo;      //  T2 latch low (T2 high is written directly to counter)
    reg [7:0]  sr;          // 10
    reg [7:0]  acr;         // 11
    reg [7:0]  pcr;         // 12
    reg [6:0]  ifr;         // 13 (bit 7 derived)
    reg [6:0]  ier;         // 14 (bit 7 is set/clear selector, not stored)

    // Timer activity flags.  T1 counts while t1_armed; T2 while t2_armed.
    reg        t1_armed;
    reg        t2_armed;
    reg        pb6_prev;

    // ── IFR bit positions (standard 6522) ─────────────────────────────
    //   0 = CA2, 1 = CA1, 2 = SR, 3 = CB2, 4 = CB1, 5 = T2, 6 = T1
    localparam [2:0] IFR_CA2 = 3'd0;
    localparam [2:0] IFR_CA1 = 3'd1;
    localparam [2:0] IFR_SR  = 3'd2;
    localparam [2:0] IFR_CB2 = 3'd3;
    localparam [2:0] IFR_CB1 = 3'd4;
    localparam [2:0] IFR_T2  = 3'd5;
    localparam [2:0] IFR_T1  = 3'd6;

    // PCR-dependent clear paths mirror the MAME 6522 model: CA2 / CB2
    // interrupts are only cleared by ORA / ORB accesses when those pins are
    // not configured as independent IRQ inputs.
    wire pcr_ca1_low_to_high = pcr[0];
    wire pcr_ca1_high_to_low = !pcr[0];
    wire pcr_ca2_ind_irq = ((pcr & 8'h0a) == 8'h02);
    wire pcr_cb2_ind_irq = ((pcr & 8'ha0) == 8'h20);
    wire pcr_ca2_input = !pcr[3];
    // Q700 VIA1 uses PCR=0x22 for the RTC CKO path; treat CA2 mode
    // bits 3:1 = 001 as an independent positive-edge IRQ input.
    wire pcr_ca2_low_to_high = ((pcr & 8'h0e) == 8'h02) ||
                               ((pcr & 8'h0c) == 8'h04);
    wire pcr_ca2_high_to_low = ((pcr & 8'h0e) == 8'h00);
    wire pcr_cb1_low_to_high = pcr[4];
    wire pcr_cb1_high_to_low = !pcr[4];
    wire pcr_cb2_input = !pcr[7];
    wire pcr_cb2_low_to_high = ((pcr & 8'hc0) == 8'h40);
    wire pcr_cb2_high_to_low = ((pcr & 8'hc0) == 8'h00);

    // Combinational IFR summary bit — mirrors real 6522: bit 7 is the
    // OR of (ifr & ier).
    wire ifr_any    = |(ifr & ier);
    wire [7:0] ifr_rd = {ifr_any, ifr};

    // ── Internal VBL tick — only used when ENABLE_INTERNAL_VBL=1 ────
    //
    // The synth/board path passes ENABLE_INTERNAL_VBL=0 because DAFB
    // drives the real CA1 line off the HDMI VTG (true 60 Hz vsync).
    // This divider is therefore DEAD CODE on the FPGA and is kept only
    // for the standalone `tb-via1` unit test, which feeds phi2_tick at
    // the same NCO rate the integrator uses (VIA_PHI2_HZ = 783_360 Hz,
    // see rtl/fpga_top_clocks.vh and rtl/fpga_top.v).  At that rate
    // VBL_PHI2_DIV = 16666 yields ~47 Hz — fine for tb-via1's purpose
    // (it only checks the divider mechanism, not the rate).
    //
    // The pre-2026-05-06 comment claimed phi2_tick = 1 MHz which gave
    // a clean 60 Hz from 16666; that hasn't been true since the NCO
    // landed.  If you ever flip ENABLE_INTERNAL_VBL=1 in the board top
    // (e.g. boot before DAFB is up), pick VBL_PHI2_DIV = 13039 to
    // restore Q700-faithful 60.15 Hz at the 783_360 Hz NCO rate.
    parameter integer VBL_PHI2_DIV = 32'd16666;
    parameter         ENABLE_INTERNAL_VBL = 1'b1;
    // Wrap the divider in a generate so synthesis-target builds (which
    // pass ENABLE_INTERNAL_VBL=1'b0 — DAFB drives the real CA1 line)
    // don't carry a dead 32-bit counter that Vivado then strips with a
    // [Synth 8-6014] vbl_cnt_reg warning.  Standalone tb-via1 keeps the
    // internal divider so VBL→CA1 still ticks on the unit clock.
    wire vbl_pulse;
    generate
        if (ENABLE_INTERNAL_VBL) begin : g_internal_vbl
            reg [31:0] vbl_cnt;
            always @(posedge clk) begin
                if (rst) begin
                    vbl_cnt <= 32'd0;
                end else if (phi2_tick) begin
                    if (vbl_cnt >= VBL_PHI2_DIV - 32'd1)
                        vbl_cnt <= 32'd0;
                    else
                        vbl_cnt <= vbl_cnt + 32'd1;
                end
            end
            // Combinational VBL strobe so the IFR update lands on the
            // same clock edge that the counter rolls over.
            assign vbl_pulse = phi2_tick && (vbl_cnt >= VBL_PHI2_DIV - 32'd1);
        end else begin : g_no_internal_vbl
            assign vbl_pulse = 1'b0;
        end
    endgenerate

    // ── Shift register FSM ────────────────────────────────────────────
    //
    // We don't simulate the clocked bit-level SR; Mac OS uses the 6522 SR
    // as a byte-serialiser to the ADB transceiver.  Byte-level contract:
    //
    //   RX path (ACR[4:2] == 3'b011 — shift in under external clock):
    //     - External transceiver raises adb_rx_valid with adb_rx_byte.
    //     - We capture SR <= adb_rx_byte on the next phi2_tick, assert
    //       adb_rx_ready for one phi2 tick (handshake), and set IFR[SR].
    //
    //   TX path (ACR[4:2] == 3'b110 or 3'b111 — shift out):
    //     - CPU writes SR (addr 10).  We latch adb_tx_byte <= sr and
    //       pulse adb_tx_valid for one phi2_tick; then set IFR[SR] to
    //       signal "TX complete".
    //
    // The CPU can also just read SR to pick up the captured byte.
    //
    // A pending_tx flag queues a CPU SR-write until the next phi2_tick
    // so RX/TX events line up with the 1 MHz VIA timebase rather than
    // firing every peripheral-bus cycle.

    wire acr_sr_in       = (acr[4:2] == 3'b011);
    wire acr_sr_out      = (acr[4:2] == 3'b110) || (acr[4:2] == 3'b111);
    wire old_byte_test_mode = !cb1_in && !cb2_in;
    wire acr_old_sr_path = (acr[4:2] != 3'b111) &&
                           ((acr[4:2] != 3'b011) || old_byte_test_mode);

    reg        sr_tx_pending;
    reg  [7:0] sr_tx_byte;
    reg        adb_idle_rx_pending;
    reg  [7:0] adb_idle_rx_count;
    reg  [4:0] shift_counter;
    reg  [7:0] m_out_cb2_val;
    reg        vblank_irq_prev;
    reg        rtc_cko_prev;
    reg        adb_cb1_line;
    reg        adb_cb1_prev;
    reg        adb_cb2_line;
    reg        adb_cb2_prev;
    reg  [7:0] adb_cb1_idle_count;
    reg  [1:0] adb_cb1_idle_pulse_state;
    reg  [1:0] adb_cb2_idle_pulse_state;

    localparam [7:0] ADB_IDLE_BYTE = 8'hff;
    localparam [7:0] ADB_IDLE_RX_TICKS = 8'd16;
    localparam [7:0] ADB_CB1_IDLE_TICKS = 8'd64;
    localparam [1:0] ADB_CB1_PULSE_IDLE = 2'd0;
    localparam [1:0] ADB_CB1_PULSE_FALL = 2'd1;
    localparam [1:0] ADB_CB1_PULSE_RAISE = 2'd2;
    localparam [1:0] ADB_CB2_PULSE_IDLE = 2'd0;
    localparam [1:0] ADB_CB2_PULSE_RAISE = 2'd1;
    localparam [1:0] ADB_CB2_PULSE_FALL  = 2'd2;
    localparam [4:0] SR_EDGE_FIRST = 5'd0;
    localparam [4:0] SR_EDGE_LAST  = 5'd15;
    reg via1_sr_trace_en;
`ifndef SYNTHESIS
    initial via1_sr_trace_en = $test$plusargs("via1_sr_trace");
`else
    initial via1_sr_trace_en = 1'b0;
`endif

    // ── Write path ────────────────────────────────────────────────────
    always @(posedge clk) begin
        if (rst) begin
            orb      <= 8'h80;   // overlay=1, bit7 idle
            ora      <= 8'h00;
            ddrb     <= 8'h00;
            ddra     <= 8'h00;
            t1c      <= 16'h0000;
            t1l      <= 16'h0000;
            t2c      <= 16'h0000;
            t2l_lo   <= 8'h00;
            sr       <= 8'h00;
            acr      <= 8'h00;
            pcr      <= 8'h00;
            ifr      <= 7'h00;
            ier      <= 7'h00;
            t1_armed <= 1'b0;
            t2_armed <= 1'b0;
            pb6_prev  <= pb_in[6];
            sr_tx_pending <= 1'b0;
            sr_tx_byte    <= 8'h00;
            adb_idle_rx_pending <= 1'b0;
            adb_idle_rx_count   <= 8'h00;
            vblank_irq_prev <= vblank_irq_in;
            adb_rx_ready  <= 1'b0;
            adb_tx_byte   <= 8'h00;
            adb_tx_valid  <= 1'b0;
            rtc_cko_prev  <= rtc_cko;
            adb_cb1_line <= cb1_in;
            adb_cb1_prev <= cb1_in;
            adb_cb2_line <= cb2_in;
            adb_cb2_prev <= cb2_in;
            adb_cb1_idle_count <= 8'h00;
            adb_cb1_idle_pulse_state <= ADB_CB1_PULSE_IDLE;
            adb_cb2_idle_pulse_state <= ADB_CB2_PULSE_IDLE;
            shift_counter <= 5'h0f;
            m_out_cb2_val <= 8'hff;
        end else begin
            // Default one-clock pulses
            adb_rx_ready <= 1'b0;
            adb_tx_valid <= 1'b0;

            // ─── CPU write ──────────────────────────────────────────
            if (pb_wr) begin
                case (pb_addr)
                    4'd0:  begin
                        orb <= pb_wdata;                                  // ORB
                        ifr[IFR_CB1] <= 1'b0;                             // PB handshake ack
                        if (!pcr_cb2_ind_irq)
                            ifr[IFR_CB2] <= 1'b0;
                    end
                    4'd1:  begin
                        ora <= pb_wdata;                                  // ORA
                        ifr[IFR_CA1] <= 1'b0;                             // PA handshake ack
                        if (!pcr_ca2_ind_irq)
                            ifr[IFR_CA2] <= 1'b0;
                    end
                    4'd2:  ddrb     <= pb_wdata;                         // DDRB
                    4'd3:  ddra     <= pb_wdata;                         // DDRA

                    // Writing T1CL stores into the low latch (not counter)
                    4'd4:  t1l[7:0] <= pb_wdata;
                    // Writing T1CH stores into high latch, transfers
                    // latches to counter, clears IFR_T1, arms timer.
                    4'd5:  begin
                        t1l[15:8] <= pb_wdata;
                        t1c       <= {pb_wdata, t1l[7:0]};
                        ifr[IFR_T1] <= 1'b0;
                        t1_armed    <= 1'b1;
                    end
                    // T1 latch write aliases (regs 6/7) — do not arm.
                    4'd6:  t1l[7:0]  <= pb_wdata;
                    4'd7:  begin
                        t1l[15:8]   <= pb_wdata;
                        ifr[IFR_T1] <= 1'b0;  // real 6522 clears T1 IFR here too
                    end

                    // T2 low: store into T2 low latch.
                    4'd8:  t2l_lo <= pb_wdata;
                    // T2 high: load counter = {this, t2l_lo}, arm, clear IFR.
                    4'd9:  begin
                        t2c         <= {pb_wdata, t2l_lo};
                        ifr[IFR_T2] <= 1'b0;
                        t2_armed    <= 1'b1;
                    end

                    // SR write: update register; if ACR says TX mode,
                    // queue a TX pulse for the next phi2_tick.  If ACR
                    // is already in ADB receive mode, treat the write as
                    // a new empty-bus receive arm so ROM probes that keep
                    // ACR stable but rewrite SR still complete.
                    4'd10: begin
                        sr <= pb_wdata;
                        shift_counter <= SR_EDGE_FIRST;
`ifndef SYNTHESIS
                        if (via1_sr_trace_en)
                            $display("[via1sr] CPU SR write = %02x (acr[4:2]=%b)",
                                     pb_wdata, acr[4:2]);
`endif
                        if (acr_old_sr_path && acr_sr_out) begin
                            sr_tx_pending <= 1'b1;
                            sr_tx_byte    <= pb_wdata;
                            adb_idle_rx_pending <= 1'b0;
                        end else if (acr_old_sr_path && acr_sr_in) begin
                            adb_idle_rx_pending <= 1'b1;
                            adb_idle_rx_count   <= 8'h00;
                        end
                        // Writing SR clears SR-attention IFR (per 6522).
                        ifr[IFR_SR] <= 1'b0;
                    end

                    4'd11: begin
                        acr <= pb_wdata;
`ifndef SYNTHESIS
                        if (via1_sr_trace_en)
                            $display("[via1sr] ACR write = %02x (sr-mode acr[4:2]=%b)",
                                     pb_wdata, pb_wdata[4:2]);
`endif
                        if ((pb_wdata[4:2] == 3'b011) || (pb_wdata[4:2] == 3'b111))
                            shift_counter <= SR_EDGE_FIRST;
                        if (pb_wdata[4:2] == 3'b011 && old_byte_test_mode) begin
                            adb_idle_rx_pending <= 1'b1;
                            adb_idle_rx_count   <= 8'h00;
                        end else begin
                            adb_idle_rx_pending <= 1'b0;
                        end
                    end
                    4'd12: begin
                        pcr <= pb_wdata;
                        if (!pb_wdata[7] && ((pb_wdata & 8'hc0) == 8'h00))
                            adb_cb2_idle_pulse_state <= adb_cb2_line ? ADB_CB2_PULSE_FALL : ADB_CB2_PULSE_RAISE;
                    end

                    // IFR: write-1-clear per-bit.
                    4'd13: begin
                        ifr <= ifr & ~pb_wdata[6:0];
                        if (pb_wdata[IFR_CB2] && pcr_cb2_input && pcr_cb2_high_to_low)
                            adb_cb2_idle_pulse_state <= adb_cb2_line ? ADB_CB2_PULSE_FALL : ADB_CB2_PULSE_RAISE;
                    end

                    // IER: MSB=1 sets bits, MSB=0 clears bits.
                    4'd14: begin
                        if (pb_wdata[7])
                            ier <= ier |  pb_wdata[6:0];
                        else
                            ier <= ier & ~pb_wdata[6:0];
`ifndef SYNTHESIS
                        if (via1_sr_trace_en)
                            $display("[via1sr] CPU IER write = %02x -> ier %02x",
                                     pb_wdata, pb_wdata[7] ? (ier | pb_wdata[6:0])
                                                           : (ier & ~pb_wdata[6:0]));
`endif
                    end

                    4'd15: ora      <= pb_wdata;                          // ORA-NH
                    default: ;
                endcase
            end

            // ─── Reads that have side-effects ────────────────────────
            // Reading ORA clears IFR_CA1/IFR_CA2 (PA handshake side).
            // Reading ORB clears IFR_CB1/IFR_CB2 (PB handshake side).
            // ORA-NH (reg 15) has no handshake side effects.
            // Reading T1CL clears IFR_T1 (real 6522 behaviour).
            // Reading T2CL clears IFR_T2.
            // Reading SR clears IFR_SR.
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
                    4'd10: begin
                        ifr[IFR_SR]  <= 1'b0;
                        shift_counter <= SR_EDGE_FIRST;
`ifndef SYNTHESIS
                        if (via1_sr_trace_en)
                            $display("[via1sr] CPU SR read = %02x (clears IFR_SR)", sr);
`endif
                    end
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
                    // Wrap: set IFR_T1, reload from latch if continuous
                    ifr[IFR_T1] <= 1'b1;
                    if (acr[6]) begin
                        // continuous mode: auto-reload from latch
                        t1c <= t1l;
                    end else begin
                        // one-shot: keep decrementing (counter free-runs),
                        // disarm so we don't re-fire every wrap
                        t1c      <= 16'hFFFF;
                        t1_armed <= 1'b0;
                    end
                end else begin
                    t1c <= t1c - 16'd1;
                end
            end

            // ─── Timer 2 countdown ──────────────────────────────────
            // ACR[5]=0: one-shot on phi2; ACR[5]=1: pulse-count on PB6.
            // Gated against a same-cycle CPU write to T2CH (reg 9) for
            // the same reason as the T1 gate above.
            if (phi2_tick && t2_armed && !acr[5] && !(pb_wr && pb_addr == 4'd9)) begin
                if (t2c == 16'h0000) begin
                    ifr[IFR_T2] <= 1'b1;
                    t2c         <= 16'hFFFF;
                    t2_armed    <= 1'b0;
                end else begin
                    t2c <= t2c - 16'd1;
                end
            end
            if (t2_armed && acr[5] && pb6_prev && !pb_in[6]) begin
                if (t2c == 16'h0000) begin
                    ifr[IFR_T2] <= 1'b1;
                    t2c         <= 16'hFFFF;
                    t2_armed    <= 1'b0;
                end else begin
                    t2c <= t2c - 16'd1;
                end
            end

            // ─── Shift-register RX / TX (phi2-gated) ────────────────
            if (phi2_tick) begin
                // ADB CB1/CB2 idle-pulse generator GUTTED — temporary fix
                // for the IRQ storm prior to real ADB.
                //
                // Background: the Q700 ADB modem PIC drives VIA1 CB1 as
                // shift clock and CB2 as shift data.  The previous code
                // synthesised idle CB1/CB2 edges every phi2_tick (~1.3 µs)
                // so ROM VIA probes would see sticky ADB IFR bits.  In
                // practice this produced edges faster than any handler
                // could clear them — IFR[CB1]/IFR[CB2] re-asserted in the
                // same vector window, the IRQ aggregator re-fired
                // immediately on RTE, and the CPU never made forward
                // progress past the autovector handler.
                //
                // adb_cb1_line / adb_cb2_line are left at their reset
                // value of 1'b1 (idle high).  No edge → no IFR set →
                // no spurious IRQ.  Replace with a real ADB transceiver
                // when ADB lands.
                //
                // adb_cb1_idle_pulse_state, adb_cb2_idle_pulse_state,
                // adb_cb1_idle_count are still declared/reset above but
                // never advanced; they synthesise to constants.

                // RX byte arriving from the ADB transceiver.
                if (acr_old_sr_path && acr_sr_in && adb_rx_valid) begin
                    sr           <= adb_rx_byte;
                    adb_rx_ready <= 1'b1;
                    ifr[IFR_SR]  <= 1'b1;
                    adb_idle_rx_pending <= 1'b0;
                end else if (acr_old_sr_path && acr_sr_in && adb_idle_rx_pending) begin
                    if (adb_idle_rx_count >= ADB_IDLE_RX_TICKS - 8'd1) begin
                        sr                  <= ADB_IDLE_BYTE;
                        ifr[IFR_SR]         <= 1'b1;
                        adb_idle_rx_pending <= 1'b0;
                    end else begin
                        adb_idle_rx_count <= adb_idle_rx_count + 8'd1;
                    end
                end
                // TX pulse follows a CPU SR write.
                if (acr_old_sr_path && sr_tx_pending) begin
                    adb_tx_byte   <= sr_tx_byte;
                    adb_tx_valid  <= 1'b1;
                    sr_tx_pending <= 1'b0;
                    ifr[IFR_SR]   <= 1'b1;
                end
            end

            // ─── External-clock shift register on CB1/CB2 ─────────────
            if (adb_cb1_line != adb_cb1_prev) begin
`ifndef SYNTHESIS
                if (via1_sr_trace_en)
                    $display("[via1sr] CB1 edge cb1=%b acr[4:2]=%b sc=%0d sr=%02x cb2in=%b ocb2=%b ifrSR=%b",
                             adb_cb1_line, acr[4:2], shift_counter, sr,
                             cb2_in, m_out_cb2_val[0], ifr[IFR_SR]);
`endif
                // Edge phasing matches MAME via6522 shift_out/shift_in.
                // MAME's m_shift_counter counts DOWN from 0x0f and:
                //   shift_out shifts on ODD counter  -> edges #1,3,5,..15
                //   shift_in  shifts on EVEN counter -> edges #2,4,6,..16
                // Our shift_counter counts UP from 0, so the parity is
                // inverted vs MAME: shift_out on EVEN sc, shift_in on ODD
                // sc reproduces MAME's exact CB1-edge phase.  Getting this
                // backwards shifts the byte by one CB1 edge, so the PIC
                // modem samples CB2 a half-bit out of phase.
                if (acr[4:2] == 3'b111) begin
                    if (!shift_counter[0]) begin
                        m_out_cb2_val <= {7'b0000000, sr[7]};
                        sr <= {sr[6:0], sr[7]};
                    end
                    if (shift_counter == SR_EDGE_LAST) begin
                        ifr[IFR_SR] <= 1'b1;
                        shift_counter <= SR_EDGE_FIRST;
                    end else begin
                        shift_counter <= shift_counter + 5'd1;
                    end
                end else if (acr[4:2] == 3'b011) begin
                    if (shift_counter[0]) begin
                        sr <= {sr[6:0], cb2_in};
                    end
                    if (shift_counter == SR_EDGE_LAST) begin
                        ifr[IFR_SR] <= 1'b1;
                        shift_counter <= SR_EDGE_FIRST;
                    end else begin
                        shift_counter <= shift_counter + 5'd1;
                    end
                end
            end

            // ─── Board CA1 sources raise IFR[CA1] ───────────────────
            // The internal 60 Hz divider preserves standalone VIA bring-up
            // tests when enabled (vbl_pulse is the generate-gated tick;
            // it ties to 1'b0 when ENABLE_INTERNAL_VBL=1'b0).  Platform
            // tops route the real Q700 CA1 source into vblank_irq_in and
            // let PCR[0] choose the edge.
            if (vbl_pulse ||
                ((vblank_irq_in != vblank_irq_prev) &&
                 ((vblank_irq_in && pcr_ca1_low_to_high) ||
                  (!vblank_irq_in && pcr_ca1_high_to_low)))) begin
                ifr[IFR_CA1] <= 1'b1;
            end

            // Q700 board wiring routes the RTC CKO pin to VIA1 CA2.  MAME's
            // 6522 latches CA2 whenever PCR selects input mode and the edge
            // sense matches the external transition.
            if (pcr_ca2_input && (rtc_cko != rtc_cko_prev) &&
                ((rtc_cko && pcr_ca2_low_to_high) ||
                 (!rtc_cko && pcr_ca2_high_to_low))) begin
                ifr[IFR_CA2] <= 1'b1;
            end

            // ADB modem lines feed VIA1 CB1/CB2.  CB1 always edge-detects
            // through PCR[4]; CB2 only latches while PCR selects input mode.
            if ((adb_cb1_line != adb_cb1_prev) &&
                ((adb_cb1_line && pcr_cb1_low_to_high) ||
                 (!adb_cb1_line && pcr_cb1_high_to_low))) begin
                ifr[IFR_CB1] <= 1'b1;
            end
            if (pcr_cb2_input && (adb_cb2_line != adb_cb2_prev) &&
                ((adb_cb2_line && pcr_cb2_low_to_high) ||
                 (!adb_cb2_line && pcr_cb2_high_to_low))) begin
                ifr[IFR_CB2] <= 1'b1;
            end

            vblank_irq_prev <= vblank_irq_in;
            rtc_cko_prev <= rtc_cko;
            adb_cb1_line <= cb1_in;
            adb_cb2_line <= cb2_in;
            adb_cb1_prev <= adb_cb1_line;
            adb_cb2_prev <= adb_cb2_line;
            pb6_prev <= pb_in[6];
        end
    end

    // ── Read path ────────────────────────────────────────────────────
    //
    // ORB read: normal 6522 DDR mux per Synertek 6522 datasheet §3.2.
    // Bit 3 follows the standard DDR mux: when DDRB[3]=1 (output) the bit
    // returns the latched orb[3] (the ROM overlay latch in classic-Mac
    // convention; the ROM uses this readback to confirm the overlay clear).
    // When DDRB[3]=0 (input — Q700 pin direction at reset / after the ROM
    // tristates the line), the bit returns pb_in[3] which the platform top
    // wires to ~adb_irq_pending per `src/mame/apple/macquadra700.cpp::
    // via_in_b` ("if (!m_adb_irq_pending) val |= 0x08;").  Reads HIGH while
    // the ADB modem is idle, LOW while a transaction is pending — matches
    // MAME bit-for-bit.
    //
    // The internal `overlay_bit` output below stays decoupled and continues
    // to assert overlay (1) while DDRB[3]=0 so the address decoder still
    // aliases low memory to ROM through reset and pre-overlay-clear.  The
    // CPU's ORB[3] read thus sees the real Q700 hardware state, while
    // glue.v / xbar see the boot-overlay latch — these are physically
    // separate paths in real silicon (the overlay flop is internal to the
    // glue, not on the VIA1 PB3 pin) and we model them the same way.
    wire       overlay_live = (ddrb[3] ? orb[3] : 1'b1);
    wire [7:0] orb_rd_mux   = (orb & ddrb) | (pb_in & ~ddrb);
    // ORB read-back is the plain 6522 Port-B DDR mux — matches MAME's
    // via6522_device golden model.  Verified via `make tb-via1-lockstep`:
    // the previous bits-[2:1] override (always returning the orb latch)
    // diverged at event #55 — an ORB read while DDRB[1:2]=input, where
    // MAME returns the pin (pb_in[2:1]=0) not the latch.  The RTC
    // clk/enb side-channel (rtc_clk/rtc_enb below) reads the orb latch
    // directly, so it is unaffected by DDR state; the ROM makes DDRB[1:2]
    // outputs (DDRB=0xf7) before bit-banging the RTC, so the CPU still
    // reads the latched value back during the transaction proper.
    wire [7:0] orb_rd       = orb_rd_mux;

    // ORA read: outputs return ora, inputs return pa_in.
    wire [7:0] ora_rd = (ora & ddra) | (pa_in & ~ddra);

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
                    4'd13:   pb_rdata <= ifr_rd;             // summary bit + flags
                    4'd14:   pb_rdata <= {1'b1, ier};        // IER bit 7 reads as 1
                    4'd15:   pb_rdata <= ora_rd;
                    default: pb_rdata <= 8'h00;
                endcase
            end else begin
                pb_rdata <= 8'h00;
            end
        end
    end

    // ── Port outputs ─────────────────────────────────────────────────
    assign pa_out  = ora;
    assign pa_mask = ddra;
    assign pb_out  = orb;
    assign pb_mask = ddrb;

    // ── Mac overlay flop output ─────────────────────────────────────
    // The overlay is "active" (ROM aliased at 0) whenever ORB[3] reads as 1.
    // Real hardware: the pin is driven only when DDRB[3]=1 (output);
    // at reset DDRB[3]=0 so overlay remains asserted.  The ROM writes
    // DDRB[3]=1 and ORB[3]=0 to turn overlay off.  Expose the live bit;
    // glue.v uses it combinationally.
    assign overlay_bit = overlay_live;

    // ── RTC side-channel fan-out ─────────────────────────────────────
    //
    // PB0 = rtcData (bidir, gated by DDRB[0]), PB1 = rtcClk, PB2 = rtcEnb.
    // The Q700 ROM's fallback PRAM path toggles ORB[1:2] without first
    // making those bits outputs in DDRB, so route the RTC select/clock
    // side-channel from the ORB latch itself.  While selected, ORB[0] is
    // also the RTC host-data latch; rtc.v ignores that latch during read
    // data bits so the device can still drive PB0 back through pb_in[0].
    // rtc_data_i is sampled back into ORB via top-level pb_in[0].
    assign rtc_enb     = orb[2];
    assign rtc_clk     = orb[1];
    assign rtc_data_o  = orb[0];
    assign rtc_data_oe = ddrb[0] | !orb[2];
    assign cb2_oe      = (acr[4:2] == 3'b111);
    assign cb2_out     = m_out_cb2_val[0];
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_rtc_data_i = rtc_data_i;
    /* verilator lint_on UNUSEDSIGNAL */

    // ── IRQ ──────────────────────────────────────────────────────────
    assign irq = ifr_any;

endmodule
