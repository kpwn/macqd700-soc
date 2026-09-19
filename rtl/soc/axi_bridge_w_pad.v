// axi_bridge_w_pad.v — T162: W-channel filler-completion for write
// bursts abandoned mid-burst by an s-side-only reset, for
// axi_async_bridge.v.
//
// Problem (the gap axi_bridge_stale_sink.v documents but does NOT fix)
// ────────────────────────────────────────────────────────────────────
// axi_bridge_stale_sink.v handles a burst that COMPLETED on the m-side
// (AW+W fully forwarded, or a full AR accepted) whose response is merely
// outstanding when an s-side-only reset lands: it sinks the owed R/B
// beats so they never replay upstream.
//
// It does nothing for a reset that lands MID-W-BURST.  When that
// happens the downstream slave has accepted an AW and some (possibly
// zero) of that burst's W beats, and is parked waiting for the rest.
// Those remaining beats will never arrive — the s-side W fifo's own
// pointers were zeroed by async_fifo's T6 coupled-reset handshake, and
// nothing re-sends them.  Because AW and W are carried on independent
// async_fifo instances, a fresh, completely unrelated write issued
// after the reset can still get its AW accepted while its W beats flow
// into a slave that is still mid-burst on the OLD address.  The W
// channel carries no address and no ID — only FIFO-ordered data — so
// the slave consumes them as the continuation of the OLD burst and
// physically commits them to the OLD (wrong) address.  Once the old
// burst's beat count is satisfied that way, the AW/W pairing is
// PERMANENTLY SHIFTED and every subsequent write on that connection
// silently corrupts.  Reachable in practice on every JTAG core-only
// reset (`reset`, `reset-and-halt-after`) that catches an l2c writeback
// or a CPU store burst mid-W.
//
// Fix
// ───
// Complete the abandoned burst(s) on the m-side with wstrb=0 no-op
// filler beats (correct WLAST on the final beat of each) so the slave's
// write state machine genuinely finishes them and returns to idle
// BEFORE any fresh W data is admitted.  This is the same idiom
// l2c_victim.v's S_RSTDRAINW uses for its own abandoned writeback (see
// that file's IMPORTANT-A header note), and the same idiom
// axi_ddr4_mig_bridge.v's wr_pad_active pusher uses for a truncated
// descriptor.  The BRESP the slave then emits for each completed-by-
// padding burst is already accounted for by axi_bridge_stale_sink's
// `debt` (it snapshots the same AW-forwarded-but-uncompleted set), so
// it is sunk rather than replayed upstream — the two mechanisms are
// deliberately counted off the SAME event (`reset_event`) and the SAME
// fwd predicate (an AW handshake on the m-side).
//
// What has to be tracked
// ──────────────────────
// To emit the right number of filler beats with WLAST in the right
// place we need, per in-flight write burst, its AWLEN and how many of
// its beats have already been forwarded.  A head register (`head_beats`,
// decremented in place, so back-to-back bursts cost no bubble) plus a
// small tail FIFO of not-yet-started burst lengths covers it:
//
//   aw_fwd  → beats = AWLEN+1 pushed (to head if head is free this
//             cycle, else to the tail fifo)
//   w_fire  → head_beats decremented; at 1 the burst retires and the
//             head reloads from the tail fifo (or from a same-cycle
//             aw_fwd) in the SAME cycle.
//
// Burst boundaries are taken from AWLEN, not from the incoming WLAST,
// because AWLEN is what the downstream slave itself counts.  The
// forwarded W stream's own WLAST bit is still passed through verbatim
// in normal operation — a malformed upstream master that misplaces
// WLAST is neither corrected nor newly broken by this module.
//
// Two side effects, both deliberate:
//
//   * `track_valid` gates real W forwarding on "the corresponding AW
//     has already been forwarded to the slave".  Without it a W beat
//     could reach the slave with no AW in front of it and no tracked
//     length, leaving the pad count unknowable.  This costs one cycle
//     on the first beat of a burst and is what the downstream slave
//     already requires anyway (axi_ddr4_mig_bridge.v's
//     `s_wready = wr_data_active && ...` only accepts W for an
//     already-accepted descriptor).
//   * `aw_block` holds AW off when the tail fifo is full, bounding the
//     tracked set at TAIL_DEPTH+1 bursts.  This is NOT a throughput
//     throttle in practice: the deepest downstream write acceptor in
//     this design is axi_ddr4_mig_bridge.v with WMAX_OUTSTANDING=4,
//     below the default capacity of 9.  It cannot deadlock — the queued
//     bursts' W data flows independently and retires entries.
//
// Reset semantics match axi_bridge_stale_sink.v exactly: this is
// bus-protocol hygiene state, not architectural state.  It is NOT reset
// by s_rst (not even wired here) — it must survive precisely the event
// it exists to protect against — and IS reset by m_rst, since a genuine
// m-side reset means the slave itself is restarting and owes nothing.
//
// The same LOAD-BEARING ORDERING CONSTRAINT documented in
// axi_bridge_stale_sink.v applies verbatim: `reset_event` must arrive
// only after every async FIFO is frozen, or the pad snapshot could miss
// a burst forwarded after the snapshot. axi_async_bridge.v enforces this
// with explicit FIFO reset-engagement acknowledgements.
//
// Verilog-2005, synchronous active-high reset, all control registered
// (nothing here adds combinational depth to the W datapath beyond a
// single constant-vs-fifo mux with a registered select).

`default_nettype none

module axi_bridge_w_pad #(
    // Tracked-burst capacity is TAIL_DEPTH + 1 (head register).  8 -> 9,
    // comfortably above the deepest downstream write acceptor
    // (axi_ddr4_mig_bridge.v, WMAX_OUTSTANDING=4).
    parameter TAIL_DEPTH_LOG2 = 3
) (
    input  wire        m_clk,
    input  wire        m_rst,

    // 1-cycle m_clk-domain pulse marking a fully-frozen s-side reset.
    input  wire        reset_event,

    // AW successfully forwarded to the downstream slave this cycle,
    // with that AW's AWLEN.
    input  wire        aw_fwd,
    input  wire [7:0]  aw_len,

    // W beat successfully accepted by the downstream slave this cycle —
    // real OR filler; both consume one beat of the slave's burst.
    input  wire        w_fire,

    // High once an AW has been forwarded whose W burst is still owed
    // beats.  Caller must gate real W forwarding on this.
    output wire        track_valid,

    // High when the tracked-burst capacity is exhausted.  Caller must
    // hold AW off (ordinary AXI backpressure) while asserted.
    output wire        aw_block,

    // High while abandoned bursts are being completed with filler
    // beats.  Caller must present WVALID=1, WSTRB=0 and `pad_wlast`
    // instead of the W fifo's output, and must NOT advance the W fifo.
    output wire        pad_active,
    output wire        pad_wlast
);

    localparam TAIL_DEPTH = 1 << TAIL_DEPTH_LOG2;
    localparam BEATS_W    = 9;                    // AWLEN=255 -> 256 beats
    localparam TCNT_W     = TAIL_DEPTH_LOG2 + 1;  // tail occupancy
    localparam PCNT_W     = TAIL_DEPTH_LOG2 + 2;  // head + tail occupancy

    localparam [BEATS_W-1:0] BEATS_ONE  = 9'd1;
    localparam [BEATS_W-1:0] BEATS_TWO  = 9'd2;
    localparam [BEATS_W-1:0] BEATS_ZERO = 9'd0;
    localparam [TCNT_W-1:0]  TCNT_ZERO  = 0;
    localparam [TCNT_W-1:0]  TCNT_ONE   = 1;
    localparam [TCNT_W-1:0]  TCNT_MAX   = TAIL_DEPTH;
    localparam [PCNT_W-1:0]  PCNT_ZERO  = 0;
    localparam [PCNT_W-1:0]  PCNT_ONE   = 1;
    localparam [TAIL_DEPTH_LOG2-1:0] TPTR_ONE = 1;

    // ── Head: the burst the slave is currently consuming W beats for ──
    reg [BEATS_W-1:0]        head_beats;   // beats still owed, incl. current
    reg                      head_valid;
    reg                      head_is_last; // == (head_beats == 1), registered

    // ── Tail: lengths of AW-forwarded bursts not yet started on W ─────
    reg [BEATS_W-1:0]        tail_mem [0:TAIL_DEPTH-1];
    reg [TAIL_DEPTH_LOG2-1:0] tail_wp;
    reg [TAIL_DEPTH_LOG2-1:0] tail_rp;
    reg [TCNT_W-1:0]         tail_cnt;

    // ── Filler-completion state ───────────────────────────────────────
    reg [PCNT_W-1:0]         pad_left;     // bursts still owed completion
    reg                      pad_r;

    wire [BEATS_W-1:0] aw_beats   = {1'b0, aw_len} + BEATS_ONE;
    wire [BEATS_W-1:0] tail_head  = tail_mem[tail_rp];
    wire               tail_empty = (tail_cnt == TCNT_ZERO);
    wire               tail_full  = (tail_cnt == TCNT_MAX);

    assign track_valid = head_valid;
    assign aw_block    = tail_full;
    assign pad_active  = pad_r;
    assign pad_wlast   = head_is_last;

    // A burst retires on the beat that takes head_beats from 1 to 0.
    wire burst_done = w_fire && head_valid && head_is_last;

    // A newly-forwarded AW lands directly in the head register whenever
    // the head is free this cycle (either idle, or retiring with nothing
    // queued behind it) — that is what makes back-to-back bursts
    // bubble-free.  Otherwise it queues in the tail fifo.
    wire head_free_now = burst_done ? tail_empty : !head_valid;
    wire aw_to_head    = aw_fwd && head_free_now;
    wire tail_push     = aw_fwd && !aw_to_head;
    wire tail_pop      = burst_done && !tail_empty;

    // Post-update occupancy — snapshotted by reset_event, for the same
    // reason axi_bridge_stale_sink.v snapshots outstanding_next_c rather
    // than the stale pre-cycle value: a same-cycle aw_fwd/burst_done must
    // be reflected, and a second reset landing inside a first one's pad
    // window must re-snapshot rather than accumulate.
    wire head_valid_next = burst_done ? (!tail_empty || aw_fwd)
                                      : (head_valid  || aw_fwd);
    wire [TCNT_W-1:0] tail_cnt_next = tail_cnt
                                    + (tail_push ? TCNT_ONE : TCNT_ZERO)
                                    - (tail_pop  ? TCNT_ONE : TCNT_ZERO);
    wire [PCNT_W-1:0] occ_next = {1'b0, tail_cnt_next}
                               + (head_valid_next ? PCNT_ONE : PCNT_ZERO);

    wire [PCNT_W-1:0] pad_left_next =
        reset_event                 ? occ_next            :
        (pad_r && burst_done)       ? (pad_left - PCNT_ONE)
                                    : pad_left;

    // tail_mem is deliberately NOT reset — it is a fifo payload store,
    // read only at addresses the (reset) pointers/occupancy vouch for.
    always @(posedge m_clk) begin
        if (m_rst) begin
            head_beats   <= BEATS_ZERO;
            head_valid   <= 1'b0;
            head_is_last <= 1'b0;
            tail_wp      <= {TAIL_DEPTH_LOG2{1'b0}};
            tail_rp      <= {TAIL_DEPTH_LOG2{1'b0}};
            tail_cnt     <= TCNT_ZERO;
            pad_left     <= PCNT_ZERO;
            pad_r        <= 1'b0;
        end else begin
            // ── Head update ───────────────────────────────────────────
            if (burst_done) begin
                if (!tail_empty) begin
                    head_beats   <= tail_head;
                    head_is_last <= (tail_head == BEATS_ONE);
                    head_valid   <= 1'b1;
                end else if (aw_fwd) begin
                    head_beats   <= aw_beats;
                    head_is_last <= (aw_beats == BEATS_ONE);
                    head_valid   <= 1'b1;
                end else begin
                    head_beats   <= BEATS_ZERO;
                    head_is_last <= 1'b0;
                    head_valid   <= 1'b0;
                end
            end else if (w_fire && head_valid) begin
                head_beats   <= head_beats - BEATS_ONE;
                head_is_last <= (head_beats == BEATS_TWO);
            end else if (aw_to_head) begin
                head_beats   <= aw_beats;
                head_is_last <= (aw_beats == BEATS_ONE);
                head_valid   <= 1'b1;
            end

            // ── Tail update ───────────────────────────────────────────
            if (tail_push) begin
                tail_mem[tail_wp] <= aw_beats;
                tail_wp           <= tail_wp + TPTR_ONE;
            end
            if (tail_pop)
                tail_rp <= tail_rp + TPTR_ONE;
            tail_cnt <= tail_cnt_next;

            // ── Filler-completion update ──────────────────────────────
            pad_left <= pad_left_next;
            pad_r    <= (pad_left_next != PCNT_ZERO);

            // synthesis translate_off
            if (reset_event && (occ_next != PCNT_ZERO))
                $display("AXI_BRIDGE_W_PAD: s-side reset abandoned %0d write burst(s) mid-W; completing with wstrb=0 filler beats (head owes %0d beat(s))",
                         occ_next, head_beats);
            // synthesis translate_on
        end
    end

    // tail_full is a genuine bound, not a should-never-happen: it is
    // enforced by holding AW off via aw_block.  Flag any violation in
    // sim so a future depth change that silently overruns is loud.
    // synthesis translate_off
    always @(posedge m_clk) begin
        if (!m_rst && tail_push && tail_full)
            $display("AXI_BRIDGE_W_PAD: ERROR tail fifo overrun -- aw_block was ignored");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
