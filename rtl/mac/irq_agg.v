// irq_agg.v — Peripheral IRQ aggregator (68040 autovector style)
//
// Mac-class OoO CPU has a single 3-bit `ipl[2:0]` input that the CPU's
// commit stage samples each cycle.  Seven peripheral IRQ lines, one per
// autovector level, feed this aggregator; the aggregator priority-encodes
// them into the highest-active-level value and presents it on `ipl`.
//
// Contract
//   Peripheral inputs are active-high level requests except `nmi_edge`.
//   This block does not contain SR.I masking state and does not clear
//   level IRQs on `ipl_ack`.  Commit compares `ipl` against SR.I and the
//   peripheral driver clears the source latch by touching the owning
//   peripheral register.  Therefore a lower still-asserted source becomes
//   visible immediately after a higher source de-asserts.
//
// Level assignments (docs/exceptions.md §"Asynchronous exceptions"):
//   level 1 (vec 25)  — VIA1   (60 Hz VBL, ADB, RTC, sound command)
//   level 2 (vec 26)  — VIA2   (NuBus slot IRQs aggregated)
//   level 3 (vec 27)  — SCSI   (NCR 5380 end-of-command)
//   level 4 (vec 28)  — SCC    (serial RX/TX ready)
//   level 5 (vec 29)  — ASC sound buffer IRQ
//   level 6 (vec 30)  — reserved / DMA class source
//   level 7 (vec 31)  — NMI (power key, edge-triggered)
//
// Priority encoder
//   Highest level wins.  Levels 1-6 are level-triggered: the aggregator
//   presents that level on `ipl` for as long as the peripheral holds
//   its `_irq` line high.  The CPU clears it indirectly by servicing
//   the peripheral in the IRQ handler (which writes to a peripheral
//   register that de-asserts the IRQ line).  Multiple active peripherals
//   at different levels are ordered by priority — the highest wins.
//
// NMI edge-trigger
//   Level 7 (NMI) is edge-triggered to avoid repeatedly retaking the
//   same NMI while the line stays high (Mac power key is level-held).
//   Rising edge of `nmi_edge` sets an internal latch `nmi_pending`;
//   while pending, the aggregator presents `ipl = 7`.  The latch is
//   cleared by `ipl_ack` pulsed by the CPU on the cycle it actually
//   dispatches the NMI (CPU drives ipl_ack high for one cycle when it
//   fires the exception sequencer with vec 31).  Until an ack comes,
//   the latch persists so the CPU can take the NMI as soon as it's
//   able (exc_active / lsu_busy gating at commit).
//
// Priority diagram (8 cases, ASCII)
//
//          +-----------+
//          | nmi_pend  |------------+--> ipl = 3'd7
//          +-----------+            |
//          +-----------+  no        |
//    rsvd6 |   irq6    |------- OR -+--> ipl = 3'd6
//          +-----------+                |
//          +-----------+  no            |
//    snd5  |   irq5    |------- OR -----+--> ipl = 3'd5
//          +-----------+                    |
//          ...                              |
//          +-----------+  no                |
//    via1  |   irq1    |------- OR ---------+--> ipl = 3'd1
//          +-----------+
//                                           none --> ipl = 3'd0
//
// Timing
//   Pure combinational from inputs to `ipl` except the NMI latch bit.
//   Fits inside any 5 ns budget (priority encoder is log2(7) levels).

module irq_agg (
    input  wire        clk,
    input  wire        rst,

    // ── Per-peripheral IRQ lines (level-triggered, active high) ──────
    input  wire        via1_irq,   // level 1 (vec 25)
    input  wire        via2_irq,   // level 2 (vec 26)
    input  wire        scsi_irq,   // level 3 (vec 27)
    input  wire        scc_irq,    // level 4 (vec 28)
    input  wire        snd_irq,    // level 5 (vec 29), ASC sound IRQ
    input  wire        rsvd_irq6,  // level 6
    input  wire        nmi_edge,   // level 7 (edge-triggered)

    // ── Handshake ────────────────────────────────────────────────────
    // ipl_ack: CPU pulses this high on the cycle it fires the
    // exception sequencer for this ipl.  For NMI (ipl == 7), this
    // clears the internal pending latch so subsequent NMIs require a
    // fresh rising edge.  For levels 1-6, ipl_ack is a no-op — those
    // are level-triggered and de-assert by peripheral-side action.
    input  wire        ipl_ack,

    // ── CPU interface ────────────────────────────────────────────────
    output wire [2:0]  ipl
);

    // ── NMI edge detect ──────────────────────────────────────────────
    // Sample nmi_edge and detect rising edges against the previous
    // cycle's value.  Set nmi_pending on rising edge; clear on
    // ipl_ack while the aggregator's output is level 7.
    reg nmi_edge_q;
    reg nmi_pending;

    wire nmi_rise = nmi_edge && !nmi_edge_q;

    always @(posedge clk) begin
        if (rst) begin
            nmi_edge_q  <= 1'b0;
            nmi_pending <= 1'b0;
        end else begin
            nmi_edge_q <= nmi_edge;
            // Set on rising edge.  If ack lands the same cycle as a
            // rising edge, prefer set (new NMI wins) — this matches
            // the 68k semantics that a freshly-arriving NMI overrides
            // an ack of a previous one.
            if (nmi_rise)
                nmi_pending <= 1'b1;
            else if (ipl_ack && ipl == 3'd7)
                nmi_pending <= 1'b0;
        end
    end

    // ── Priority encoder (highest level wins) ─────────────────────────
    // NMI (latched) overrides everything else.  rsvd_irq6 > snd_irq >
    // scc_irq > scsi_irq > via2_irq > via1_irq.  If none are active,
    // output 0 (meaning "no IRQ").
    assign ipl = nmi_pending ? 3'd7 :
                 rsvd_irq6   ? 3'd6 :
                 snd_irq     ? 3'd5 :
                 scc_irq     ? 3'd4 :
                 scsi_irq    ? 3'd3 :
                 via2_irq    ? 3'd2 :
                 via1_irq    ? 3'd1 :
                               3'd0;

endmodule
