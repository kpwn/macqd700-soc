// axi_bridge_stale_sink.v — T14 investigation fix: per-channel stale
// response-beat sink for axi_async_bridge.v.
//
// Problem
// ───────
// axi_async_bridge.v carries each AXI4 channel across a clock-domain
// crossing on an independent async_fifo.v instance.  async_fifo's own
// "T6" coupled-reset handshake (see rtl/board/async_fifo.v header)
// correctly RELAYS a one-sided reset to the far side and zeroes BOTH
// the write and read pointers together — no pointer desync, ever.  But
// that handshake operates purely on the FIFO's Gray-coded pointers; it
// has zero awareness of AXI *burst* semantics.
//
// Concretely: under a core-only (s-side-only) reset, `m_rst` never
// pulses, so the downstream slave (real MIG or sim_mig_backend) is
// never told to abandon whatever burst it already accepted.  While the
// T6 handshake is engaging/zeroing, this bridge's `m_rready`/`m_bready`
// get forced low (the FIFO looks "full" during the coupled-reset
// window) — which is legal, ordinary AXI backpressure from the slave's
// point of view.  The slave just holds RVALID/BVALID and its payload
// steady until ready returns.  Once the handshake settles and
// ready returns high, the slave's STILL-OUTSTANDING (pre-reset) beat(s)
// get accepted into the now-freshly-zeroed FIFO and forwarded upstream
// as if they were brand-new data — a stale-beat replay.  Root-caused
// via direct hierarchical Verilator memory probing in the T14
// investigation trail (see docs/../t14-report.md ROUND 2).
//
// Fix
// ───
// Track, per channel, how many AXI bursts are "outstanding" on the
// m-side (AR/AW forwarded to the slave, response not yet returned).
// This counter lives ENTIRELY in the m_clk domain (fwd_event and
// complete_event are both already m_clk-domain signals in
// axi_async_bridge.v — AR/AW-forward is the async_fifo's own m-side
// rd_en, and R/B-complete is the m-side accept of RLAST/BRESP), so no
// cross-domain counter is needed.
//
// On an s-side reset, snapshot the live "outstanding" count into a "debt"
// counter. axi_async_bridge.v delays this reset_event until every channel
// FIFO has explicitly acknowledged its source-side reset in the m_clk
// domain, so no pre-reset AR/AW can forward after the snapshot.
// While debt is nonzero, every response beat arriving from the m-side
// slave is discarded (sunk) — NEVER written into the R/B async_fifo —
// instead of being forwarded upstream.  RLAST (R) / any beat (B)
// decrements debt by one.  Once debt reaches zero, forwarding resumes
// normally.  This absorbs the abandoned burst's tail entirely on the
// m-side, before it ever crosses back into the s_clk domain, so the
// s-side never sees it — matching the "sink executed m-side" placement
// specified in the T14 ROUND 3 fix authorization.
//
// `outstanding` / `debt` are bus-protocol hygiene state, not
// architectural state — same class as l2c_victim.v's `stray_b_owed`
// (see docs/l2c_spec.md Important-A) and l2c_mshr.v's bounded
// S_DRAIN — deliberately NOT reset by `s_rst` (it isn't even wired to
// this module) so they survive exactly the reset event they exist to
// protect against.  They DO reset on `m_rst`, since a genuine m-side
// reset means the downstream slave itself is restarting and any
// tracked debt is moot.
//
// LOAD-BEARING ORDERING CONSTRAINT: the debt snapshot is only correct
// after async_fifo.v's coupled-reset handshake has frozen every channel.
// axi_async_bridge.v enforces this structurally by arming recovery from
// the crossed reset witness, waiting for all five FIFO engagement outputs,
// and only then pulsing `reset_event`. Do not connect this module directly
// to an unconstrained reset pulse.
//
// Companion module — the mid-W-burst case (task #162, FIXED, was the
// "known gap" written up below).  Everything the next paragraph
// describes is real and was reproduced beat-by-beat; it is no longer
// unhandled.  `axi_bridge_w_pad` (rtl/soc/axi_bridge_w_pad.v) sits on
// the same `reset_event` and the same AW-forward predicate as this
// module and completes any abandoned W burst with wstrb=0 filler beats
// (correct WLAST) before fresh W data is admitted, so the slave's write
// state machine returns to idle and the AW/W pairing never shifts.  The
// BRESP each padded burst then produces is exactly the debt THIS module
// already snapshotted, so it is sunk rather than replayed — the two
// mechanisms are counted off the same events by design.  Regression:
// tb_axi_async_bridge.cpp `sside_reset_mid_w_burst_pad`, which sweeps
// the reset across every landing point in an 8-beat burst.  Keep the
// writeup below: it is the root-cause record, and the failure mode is
// silent, so anyone touching either module needs to know what it looks
// like.
//
// Root-cause record (the case #162 fixes): if an s-side
// reset abandons a WRITE burst whose W beats were only partially sent
// (AW accepted, W incomplete), the downstream slave's write state
// machine does NOT return to idle — it stays parked waiting for that
// burst's remaining W beats, which will now never arrive (the s-side's
// own W-fifo pointers were reset too, and nothing here re-sends them).
// The REAL, honest consequence is worse than "debt stays elevated":
// the slave's AW/W channels are independent (separate async_fifo
// instances), so a fresh, UNRELATED write issued after the reset can
// still get its AW accepted (once the slave's aw-side is free) while
// its W beats flow into a slave that is still mid-burst on the OLD,
// abandoned write — AXI's W channel carries no address or ID, only
// FIFO-ordered data, so the slave has no way to tell "these beats
// belong to a different write" and simply consumes them as the
// continuation of the OLD burst, physically committing them to the
// OLD (wrong) address. Once the old burst's beat count is satisfied
// this way, the slave asserts BRESP for the OLD write's ID, and the
// AW/W pairing is now PERMANENTLY SHIFTED by however many beats got
// misattributed -- every subsequent write on that connection silently
// corrupts, not just the one caught by the reset. Reachable in
// practice whenever a JTAG core-only reset catches an l2c writeback/
// eviction mid-W. `outstanding`/`debt` (B channel) also stay
// permanently elevated for the abandoned burst as a secondary symptom,
// incorrectly sinking a later BRESP, but that is not the primary risk
// here. This module only handles the case verified in the T14
// investigation: a burst that DOES complete on the m-side (AW+W fully
// forwarded, or a full AR accepted) whose response is merely
// outstanding at reset time. The incomplete-W-burst case is handled by
// the companion W filler-completion module described above
// (rtl/soc/axi_bridge_w_pad.v) -- the same idiom l2c_victim.v's
// S_RSTDRAINW and axi_ddr4_mig_bridge.v's wr_pad_active pusher already
// use for their own abandoned-burst completion.
//
`default_nettype none

module axi_bridge_stale_sink #(
    parameter OUTSTANDING_W = 4   // up to 15 outstanding bursts tracked
) (
    input  wire                     m_clk,
    input  wire                     m_rst,

    // 1-cycle m_clk-domain pulse marking a fully-frozen s-side reset.
    input  wire                     reset_event,

    // AR/AW successfully forwarded to the downstream slave this cycle
    // (i.e. the async_fifo's m-side rd_en firing).
    input  wire                     fwd_event,

    // RLAST (R channel) / any beat (B channel) successfully ACCEPTED
    // from the downstream slave this cycle — gate this on the final,
    // post-sink m_rready/m_bready so the count only reflects real
    // handshakes, sunk or not.
    input  wire                     complete_event,

    // High while stale beats from before the last reset_event are still
    // being drained — caller must discard (not forward) beats while
    // this is asserted.
    output wire                     sinking
);

    reg [OUTSTANDING_W-1:0] outstanding; // fwd'd - completed, live count
    reg [OUTSTANDING_W-1:0] debt;        // bursts still owed a sink

    assign sinking = (debt != {OUTSTANDING_W{1'b0}});

    // outstanding_next_c: what `outstanding` WILL hold after this
    // cycle's own fwd_event/complete_event update (the same arithmetic
    // as the case statement below, computed combinationally so a
    // same-cycle reset_event can snapshot the up-to-date value instead
    // of the stale pre-cycle one).
    wire [OUTSTANDING_W-1:0] outstanding_next_c =
        outstanding
        + (fwd_event      ? {{(OUTSTANDING_W-1){1'b0}}, 1'b1} : {OUTSTANDING_W{1'b0}})
        - (complete_event ? {{(OUTSTANDING_W-1){1'b0}}, 1'b1} : {OUTSTANDING_W{1'b0}});
    wire [OUTSTANDING_W-1:0] debt_dec = (complete_event && sinking)
                                             ? {{(OUTSTANDING_W-1){1'b0}}, 1'b1}
                                             : {OUTSTANDING_W{1'b0}};

    always @(posedge m_clk) begin
        if (m_rst) begin
            outstanding <= {OUTSTANDING_W{1'b0}};
            debt        <= {OUTSTANDING_W{1'b0}};
        end else begin
            // fwd_event and complete_event can coincide (a new burst
            // forwarded the same cycle an old one completes) — the
            // case split below nets out correctly (+1-1=0 net change)
            // via the default (no-op) arm.
            case ({fwd_event, complete_event})
                2'b10:   outstanding <= outstanding + 1'b1;
                2'b01:   outstanding <= outstanding - 1'b1;
                default: ; // 2'b00: no change. 2'b11: net zero.
            endcase
            // I1 fix (full-branch-review): a `reset_event` SNAPSHOTS
            // debt to the current true outstanding count -- it does
            // NOT accumulate on top of whatever debt is still
            // undrained. The previous `debt + debt_inc - debt_dec`
            // form added `outstanding` EVERY time reset_event fired,
            // including a SECOND s-reset landing within the ~100-cycle
            // drain window of a first one (JTAG double-pulse
            // territory): with debt=1 (one burst still draining) and
            // outstanding=1 (the same burst, still genuinely
            // outstanding), a second reset_event computed debt=1+1=2,
            // double-counting the SAME undrained burst -- the sink then
            // over-sinks by one, silently eating the next FRESH
            // response after the real debt was already repaid. The fix
            // snapshots to `outstanding_next_c` (this cycle's accurate,
            // up-to-date outstanding value) on every reset_event,
            // discarding whatever old debt was still in flight --
            // correct because outstanding_next_c ALREADY reflects
            // every burst still genuinely forwarded-but-incomplete,
            // with no double-count possible. When NOT resetting this
            // cycle, debt simply drains normally via debt_dec, exactly
            // as before.
            if (reset_event) begin
                debt <= outstanding_next_c;
            end else begin
                debt <= debt - debt_dec;
            end
        end
    end

endmodule

`default_nettype wire
