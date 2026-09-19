// l2c_victim.v -- L2 dirty-eviction victim/writeback buffer, N entries,
//                 PIPELINED (many writebacks outstanding at once).
//
// Holds evicted dirty lines until they drain to DRAM over a dedicated
// AXI-master write sub-port (arbitrated onto the shared physical master
// port by l2c.v alongside l2c_mshr's read sub-port and l2c_bypass).
// A fill may proceed before its victim drains (l2c.v copies the evicted
// line into this buffer combinationally at eviction time, independent of
// DRAM latency) -- see docs/l2c_spec.md S5/S6.
//
// WHY THIS MODULE WAS REWRITTEN (2026-08-20, measured in 54919fe).
// The v1 buffer had two slots and ONE drain FSM whose states were
// S_AW -> S_W -> S_B, i.e. it held the master write port for a full DRAM
// write round trip per evicted line.  l2c.v's AW/W/B arbiter compounded
// it by holding its grant from AW-PRESENTATION to B.  Measured on
// conflict-eviction traffic (every op misses AND evicts a dirty victim),
// against the deterministic model in tb_l2c.cpp:
//
//     cycles/op = 3.0 + 0.746 x DDR_write_latency        (R^2 ~ 1)
//     L=40 -> 32.85    L=200 -> 152.22    L~0 -> 3.75
//
// The second slot bought nothing at all: it could not begin draining
// until the first slot's B came back.  The L~0 point (B returned one
// cycle after WLAST) is the floor write pipelining can reach.
//
// So: SLOTS entries (default 8) and NO S_B in the drain path.  The
// sequencer is S_AW -> S_W -> S_AW: it presents an AW, streams that
// transaction's W beats, and immediately moves to the next entry.  B is
// tracked SEPARATELY, by count, and retires entries in order.
//
//   wptr -- next slot a push lands in
//   dptr -- slot the AW/W sequencer is working on; advances at WLAST
//   bptr -- oldest slot awaiting its B; advances at B
//
//   bptr <= dptr <= wptr  (mod 2*SLOTS), occupancy = wptr - bptr
//
// AW NEVER RUNS AHEAD OF ITS OWN W BURST.  A transaction's AW is
// presented only from S_AW, and S_AW is left for S_W on AW-accept, so at
// most ONE transaction can ever be "AW accepted, W not finished".  This
// is deliberate and it is what keeps the reset story below tractable:
// every OTHER outstanding transaction has had its W burst fully sent and
// owes nothing but a BRESP.  It costs nothing measurable -- the round
// trip being hidden is the B latency, not the one AW cycle.
//
// AXI ORDERING.  Every writeback uses the SAME AWID (wb_id = 0), so AXI4
// requires the slave to return their BRESPs in AW-acceptance order; that
// is what makes in-order retirement at bptr correct, and it holds for
// every path below this master (axi_ddr4_mig_bridge is a strict per-
// direction FIFO that preserves BID, docs/ddr4_mig_bridge_contract.md
// "Ordering guarantees"; axi_async_bridge is FIFO-based).  W beats are
// likewise emitted in AW-acceptance order, which AXI4 requires because
// there is no WID.  l2c.v's arbiter keeps the write port EXCLUSIVE to
// one source while any of its writes are outstanding, so l2c_bypass can
// never interleave a W burst into the middle of ours.
//
// query_hit exposes the hazard check the front-door pipeline uses: any
// new access to a line still resident here (valid, whether draining has
// started or not) must stall until that entry retires.  The compare is
// LINE-ALIGNED (addr[ADDR_WIDTH-1:6]) -- query_addr arrives as a full
// per-beat address (which may have a nonzero offset within the line),
// while the stored addresses are always line-aligned (pushed as
// {tag,set,6'b0}); comparing the raw full addresses for equality missed
// every query whose offset happened to be nonzero, letting an access at
// offset != 0 of a just-evicted dirty line fall through to a fresh-miss
// refetch from DRAM while the actual dirty data was still sitting,
// un-written-back, in this buffer (stale-data escape).
//
// THE QUERY MUST STAY CORRECT ACROSS THE PIPELINE.  l2c is the SoC-wide
// point of coherency and does not snoop, so this compare is the ONLY
// thing standing between a re-read of a just-evicted line and stale DRAM
// data.  An entry stays `v_valid` -- and therefore stays visible to the
// query -- from push until its B is accepted.  A write that has been
// PRESENTED, or even ACCEPTED, is not durable: freeing a slot at
// AW-accept (or at WLAST) would open exactly the window this compare
// exists to close, so retirement is keyed on B and nothing else.  It
// also keeps the buffer duplicate-free: a second eviction of the same
// line can only happen after the line is re-fetched, which the query
// blocks while the first copy is still here, so no two entries ever hold
// the same line and out-of-order draining between entries is moot.
//
// PAYLOAD IN LUTRAM, NOT FLOPS (878d002's precedent).  SLOTS x 512 b is
// 4,096 FF at SLOTS=8 -- about what 878d002 freed by moving the MSHR
// replay payload out of flops -- plus the SLOTS:1-over-512b read mux the
// flop array would need.  `v_pay` is a FLAT 1-D array of LINE_BITS-wide
// words with `(* ram_style = "distributed" *)`, which is the shape that
// actually infers here; a 2-D unpacked array does NOT (the same trap
// that leaves asc.v's FIFOs in LUTRAM despite ram_style="block", and the
// reason 878d002 had to flatten).  VERIFY, don't assume: Vivado prints
// "Trying to implement RAM 'X' in registers" when inference fails.
// The read is asynchronous, so the sequencer is cycle-for-cycle what the
// flop array would have been -- no pipeline stage, nothing new to get
// wrong.  Write-then-read of the same slot cannot collide: a slot is
// written at `wptr` and read at `dptr`, the read only happens in S_W,
// and S_W is reachable no earlier than two cycles after the push that
// filled that slot (push edge -> v_valid visible -> S_AW -> S_W).
// Metadata (v_valid/v_addr/v_dsec) stays in flops: the query CAMs the
// addresses combinationally every cycle and a RAM cannot be searched
// associatively -- the same reason 878d002 left r_id in flops.
//
// The width is what costs: SLOTS x 512 b is one LUT6 per bit (512) at any
// depth up to 32, because the array must absorb a whole line in one cycle
// (push_data is 512 b) even though it is read 128 b at a time.  Addressing
// it {slot, quadrant} instead -- 32 x 128 b -- would be 4x cheaper, and
// needs no staging register (l2c_ctrl holds victim_push_* stable until
// push_ready), but it turns the push into a four-cycle accept.  Costed and
// deliberately NOT taken: ~0.2% of the device's LUTs against a new
// multi-cycle accept path on the eviction critical path.  See
// docs/l2c_perf.md S13.5.
//
// Reset mid-drain (IMPORTANT-A): dirty-line CACHE state (v_valid etc.)
// is intentionally dropped on reset per the module's documented "dirty
// lines are lost on reset" contract -- but the physical m_axi WRITE
// TRANSACTIONS are a separate concern, bus-protocol hygiene rather than
// cache state (the same category as l2c_reset.v's tag-clear walker,
// which also keeps running post-reset for reasons independent of what
// gets cleared).  With a pipelined engine there may now be SEVERAL
// transactions outstanding when reset lands, so the accounting is a
// COUNT rather than a single flag:
//
//   out_cnt  -- AW-accepted minus B-received.  DRAM owes us exactly this
//               many BRESPs.  Deliberately does NOT clear on `rst`.
//   sink_cnt -- how many of those belong to PRE-reset transactions whose
//               slots no longer exist.  Loaded from out_cnt at reset.
//
// At most one of them can be mid-W-burst (see "AW never runs ahead"),
// and it is the one in S_W: S_RSTDRAINW completes that burst with
// wstrb=0 filler beats (data is irrelevant once no byte lane is enabled)
// so DRAM is not left waiting for beats forever.  S_RSTSINKB then
// consumes the sink_cnt BRESPs DRAM still owes, which must be sunk
// rather than left to arrive later and get mis-consumed as some LATER,
// unrelated writeback's own completion (a perpetual off-by-one B
// association).  While sink_cnt != 0 no new AW is presented, so every B
// arriving in that window is unambiguously a stray and frees no slot.
//
// SECTORED WRITEBACK (2026-08-19, docs/l2c_perf.md S12).  `push_dsec` is
// the evicted line's per-quadrant dirty mask, stored PER ENTRY.  Only
// the quadrants that were actually modified need to reach DRAM, so the
// burst covers the CONTIGUOUS SPAN from the lowest to the highest dirty
// quadrant -- awaddr = line base + first*16, awlen = last - first.  A
// line touched in one 16 B quadrant (which is one 68040 L1D line, and
// the shape a copyback L1 push presents) costs ONE beat instead of four.
//
// Three details that are not optional:
//   * A quadrant inside the span that is NOT dirty is sent with wstrb=0.
//     It may be INVALID -- never fetched, holding whatever the URAM had
//     -- and writing that to DRAM would be silent corruption.  Masks like
//     4'b1001 are the only ones where this costs a beat carrying nothing;
//     contiguous masks (the common ones) have no gap at all.
//   * dsec != 0 always: l2c_ctrl only pushes a victim when the line has
//     at least one dirty quadrant, so first/last are always defined.
//     `dirty ==> the whole 16 B quadrant holds valid data` is the tag
//     array's invariant (a partial-strobe write can only reach a quadrant
//     that is already valid), which is what makes a full-strobe beat
//     correct for every dirty quadrant.
//   * The mask is SNAPSHOTTED into act_dsec_r at AW-accept, not read live
//     from the slot, for the W phase.  IMPORTANT-A's reset path forces
//     dptr back to 0 while S_RSTDRAINW is still completing a burst DRAM
//     is owed; a live read would then take slot 0's mask and get the
//     filler burst's LENGTH wrong, which is exactly the desync the reset
//     story above exists to avoid.  The AW phase reads the mask live
//     (aw_dsec_c) because dptr is stable for the whole presentation.
//
// Verilog-2005, sync active-high rst, up to SLOTS writebacks (each a
// 1..4-beat, 128b/beat INCR burst) outstanding at a time.

`default_nettype none

module l2c_victim #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BITS  = 512,
    parameter ID_WIDTH   = 6,
    // Buffer depth.  MUST be a power of two >= 2 (the pointers wrap by
    // plain truncation).  8 costs ~512 LUT of distributed RAM for the
    // payload and ~300 FF of metadata; it is also the knob that grows the
    // query CAM feeding l2c_ctrl's hit-resolve cone, so grow it past 8
    // only with a timing report in hand.
    parameter SLOTS      = 8,
    // Set when an async boundary below this master owns one-sided-reset
    // completion. Replaying filler beats here would duplicate that padding.
    parameter EXTERNAL_RESET_RECOVERY = 0
) (
    input  wire                        clk,
    input  wire                        rst,

    // Push a newly-evicted dirty line.
    input  wire                        push_valid,
    output wire                        push_ready,
    input  wire [ADDR_WIDTH-1:0]       push_addr,
    input  wire [LINE_BITS-1:0]        push_data,
    input  wire [3:0]                  push_dsec,

    // Hazard query (combinational, line-aligned -- low 6 bits intentionally
    // unused, see header).
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ADDR_WIDTH-1:0]       query_addr,
    /* verilator lint_on UNUSEDSIGNAL */
    output wire                        query_hit,

    // AXI-master write sub-port.
    output wire [ID_WIDTH-1:0]         m_awid,
    output wire [ADDR_WIDTH-1:0]       m_awaddr,
    output wire [7:0]                  m_awlen,
    output wire [2:0]                  m_awsize,
    output wire [1:0]                  m_awburst,
    output wire                        m_awvalid,
    input  wire                        m_awready,
    output wire [127:0]                m_wdata,
    output wire [15:0]                 m_wstrb,
    output wire                        m_wlast,
    output wire                        m_wvalid,
    input  wire                        m_wready,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ID_WIDTH-1:0]         m_bid,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [1:0]                  m_bresp,
    input  wire                        m_bvalid,
    output wire                        m_bready
);

    localparam SB = (SLOTS <= 2)  ? 1 : (SLOTS <= 4)  ? 2 :
                    (SLOTS <= 8)  ? 3 : (SLOTS <= 16) ? 4 : 5;
    localparam CW = SB + 1;                     // pointer / counter width
    localparam [CW-1:0] SLOTS_V = SLOTS;
    localparam [CW-1:0] ZERO_C  = {CW{1'b0}};
    localparam [CW-1:0] ONE_C   = {{(CW-1){1'b0}}, 1'b1};

    integer i;

    // -- Entry metadata (flops -- the query CAMs v_addr, see header) ------
    reg                    v_valid [0:SLOTS-1];
    reg [ADDR_WIDTH-1:0]   v_addr  [0:SLOTS-1];
    reg [3:0]              v_dsec  [0:SLOTS-1];

    // -- Entry payload (LUTRAM, flat 1-D, see header) ---------------------
    (* ram_style = "distributed" *)
    reg [LINE_BITS-1:0]    v_pay   [0:SLOTS-1];

    reg [CW-1:0] wptr, dptr, bptr;

    wire [CW-1:0] used_c = wptr - bptr;
    wire          full_c = (used_c == SLOTS_V);
    wire          todo_c = (wptr != dptr);      // an entry not yet drained
    assign push_ready = !full_c;

    wire [SB-1:0] wslot_c = wptr[SB-1:0];
    wire [SB-1:0] dslot_c = dptr[SB-1:0];
    wire [SB-1:0] bslot_c = bptr[SB-1:0];

    // -- Hazard query (combinational, LINE-ALIGNED compare, see header) ---
    // One line-address compare per slot, OR-reduced.  This is the cone
    // that grows with SLOTS and it feeds l2c_ctrl's s_lookup_active, so it
    // is why SLOTS is a parameter rather than a constant.
    wire [SLOTS-1:0] q_match_c;
    genvar gq;
    generate
        for (gq = 0; gq < SLOTS; gq = gq + 1) begin : g_query
            assign q_match_c[gq] = v_valid[gq] &&
                (v_addr[gq][ADDR_WIDTH-1:6] == query_addr[ADDR_WIDTH-1:6]);
        end
    endgenerate
    assign query_hit = |q_match_c;

    // -- Contiguous span of dirty quadrants (see header) ------------------
    // dsec is never zero for a pushed entry, so both chains resolve.
    function [1:0] dsec_first;
        input [3:0] d;
        dsec_first = d[0] ? 2'd0 : d[1] ? 2'd1 : d[2] ? 2'd2 : 2'd3;
    endfunction
    function [1:0] dsec_last;
        input [3:0] d;
        dsec_last  = d[3] ? 2'd3 : d[2] ? 2'd2 : d[1] ? 2'd1 : 2'd0;
    endfunction

    // -- Sequencer --------------------------------------------------------
    // S_AW is also the IDLE state: with nothing to drain it simply holds
    // m_awvalid low.  There is no S_B -- completion is counted, not waited
    // on, which is the whole point of this rewrite.
    // S_RSTDRAINW/S_RSTSINKB (IMPORTANT-A) are entered only from the reset
    // branch below, never from normal operation.
    localparam S_AW = 2'd0, S_W = 2'd1, S_RSTDRAINW = 2'd2, S_RSTSINKB = 2'd3;
    reg [1:0]    st;
    reg [1:0]    beat;
    reg [1:0]    rdbeat;     // resume point for the abandoned-burst filler
    reg [3:0]    act_dsec_r; // dirty-quadrant mask of the W burst in progress
    reg [CW-1:0] out_cnt;    // AW-accepted minus B-received -- survives rst
    reg [CW-1:0] sink_cnt;   // of those, pre-reset strays owning no slot

    wire [ID_WIDTH-1:0] wb_id = {ID_WIDTH{1'b0}};

    // AW phase reads the entry live (dptr is stable for the whole
    // presentation); W phase reads the snapshot (see header).
    wire [ADDR_WIDTH-1:0] aw_addr_c  = v_addr[dslot_c];
    wire [3:0]            aw_dsec_c  = v_dsec[dslot_c];
    wire [1:0]            aw_first_c = dsec_first(aw_dsec_c);
    wire [1:0]            aw_span_c  = dsec_last(aw_dsec_c) - aw_first_c;

    wire [1:0] act_first = dsec_first(act_dsec_r);
    wire [1:0] act_span  = dsec_last(act_dsec_r) - act_first;
    wire [1:0] act_q     = act_first + beat;   // quadrant this beat carries
    wire [LINE_BITS-1:0] act_data_c = v_pay[dslot_c];

    assign m_awid    = wb_id;
    assign m_awaddr  = {aw_addr_c[ADDR_WIDTH-1:6], aw_first_c, 4'b0000};
    assign m_awlen   = {6'b0, aw_span_c};
    assign m_awsize  = 3'd4;
    assign m_awburst = 2'd1; // INCR

    // Every master-port output below is gated by !rst UNIFORMLY (not
    // just the S_RSTDRAINW/S_RSTSINKB terms) -- this was a real bug
    // found during bring-up: on the very cycle rst FIRST asserts, `st`
    // itself hasn't transitioned to S_RSTDRAINW yet (that's an NBA,
    // taking effect only on the NEXT edge), so an ungated `(st==S_W)`
    // term stayed live for one extra cycle with REAL (non-filler) data,
    // presenting one uncounted bonus beat.  If the mem/DRAM side (and
    // g_mem_force_ready in the tb) happened to accept it, its beat-count
    // desynced from l2c_victim's own `rdbeat<=beat` snapshot by exactly
    // one -- the drain then finished one beat short of what the far side
    // was still expecting, permanently hanging with m_wvalid asserted
    // and m_wready never coming (the far side had already closed the
    // burst).  Gating unconditionally means NOTHING is ever presented
    // during any cycle rst is asserted (rst may be held for several
    // cycles -- this module's own sequential progress, rdbeat and
    // out_cnt included, is frozen for exactly as long, so nothing it
    // would present during that window could be current anyway);
    // draining only ever actually progresses once rst has deasserted.
    // The same gating is what makes "an AW cannot be accepted on the
    // cycle rst asserts" true, which the reset case analysis relies on.
    assign m_awvalid = !rst && (st == S_AW) && todo_c;
    assign m_wdata   = act_data_c[act_q*128 +: 128];
    // A gap quadrant inside the span carries no enabled byte lane -- see
    // the header; it may never have been fetched at all.
    assign m_wstrb   = (st == S_RSTDRAINW) ? 16'h0000
                     : (act_dsec_r[act_q] ? 16'hffff : 16'h0000);
    assign m_wlast   = (st == S_RSTDRAINW) ? (rdbeat == act_span) : (beat == act_span);
    assign m_wvalid  = !rst && ((st == S_W) || (st == S_RSTDRAINW));
    // Always ready for a BRESP while any transaction is outstanding --
    // there is no state to wait in any more.
    assign m_bready  = !rst && (out_cnt != ZERO_C);

    wire aw_fire_c = m_awvalid && m_awready;
    wire b_fire_c  = m_bvalid  && m_bready;

    // -- Observability (sim taps; also the honest definition of "busy") ---
    // With a pipelined engine "busy" can no longer mean "the FSM is not
    // idle": the sequencer sits in S_AW for most of a saturated run
    // because it has already handed everything to DRAM.  drain_busy_c is
    // "at least one writeback is in flight or waiting to start", which is
    // what the pre-pipelining `st != S_IDLE` actually measured.
    // seq_busy_c is the narrower "the AW/W sequencer has something to do
    // this cycle", i.e. the port occupancy an eager cleaner would compete
    // for.
    /* verilator lint_off UNUSEDSIGNAL */
    wire drain_busy_c = (st != S_AW) || todo_c || (out_cnt != ZERO_C);
    wire seq_busy_c   = (st != S_AW) || todo_c;
    // Depth as a WIRE, so a testbench tap does not have to hierarchically
    // reference a parameter (which not every tool resolves).
    wire [7:0] slots_c = SLOTS;
    /* verilator lint_on UNUSEDSIGNAL */

    // synthesis translate_off
    // Sim-only visibility for Minor-16: a writeback BRESP has no
    // architectural error channel (no requester is waiting on it), so a
    // DRAM-side SLVERR/DECERR can only be surfaced here.  Counted as well
    // as printed so a testbench can assert on it instead of grepping stdout.
    integer wb_err_count;
    initial wb_err_count = 0;
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            // IMPORTANT-A: decide, from the PRE-reset st, whether DRAM is
            // owed anything before this engine goes fully idle.  A second
            // reset landing while still draining/sinking from an earlier
            // one must NOT restart or double-count -- rdbeat/out_cnt/
            // sink_cnt already reflect it.
            if (EXTERNAL_RESET_RECOVERY != 0) begin
                st <= S_AW;
                out_cnt  <= ZERO_C;
                sink_cnt <= ZERO_C;
            end else begin
                case (st)
                    S_W: begin
                        // One burst is mid-flight; finish it with filler
                        // beats, then sink every BRESP still owed.
                        st <= S_RSTDRAINW; rdbeat <= beat; sink_cnt <= out_cnt;
                    end
                    S_AW: begin
                        // Nothing mid-burst.  Anything already accepted
                        // still owes us a BRESP.  (An AW PRESENTED but not
                        // accepted promised DRAM nothing -- resetting clean
                        // there is what keeps l2c.v's arbiter from waiting
                        // forever for a B that is never coming.)
                        sink_cnt <= out_cnt;
                        st <= (out_cnt != ZERO_C) ? S_RSTSINKB : S_AW;
                    end
                    S_RSTDRAINW, S_RSTSINKB: begin
                        // stay -- already draining/sinking, state intact.
                    end
                    default: begin
                        // Unreachable in 2-state; this is the power-up X
                        // case, the only place out_cnt/sink_cnt are ever
                        // initialised rather than carried.
                        st <= S_AW;
                        out_cnt  <= ZERO_C;
                        sink_cnt <= ZERO_C;
                    end
                endcase
            end
            wptr <= ZERO_C; dptr <= ZERO_C; bptr <= ZERO_C;
            beat <= 2'd0;
            for (i = 0; i < SLOTS; i = i + 1) v_valid[i] <= 1'b0;
            // Deliberately NOT cleared when a burst is mid-flight: the
            // S_RSTDRAINW filler must finish the burst it already promised
            // DRAM, whose length is act_span, i.e. this mask.
            if ((st != S_W) && (st != S_RSTDRAINW) && (st != S_RSTSINKB))
                act_dsec_r <= 4'b1111;
        end else begin
            // -- Push -----------------------------------------------------
            if (push_valid && push_ready) begin
                v_valid[wslot_c] <= 1'b1;
                v_addr [wslot_c] <= push_addr;
                v_dsec [wslot_c] <= push_dsec;
                v_pay  [wslot_c] <= push_data;
                wptr <= wptr + ONE_C;
            end

            // -- AW / W sequencer -----------------------------------------
            case (st)
                S_AW: if (aw_fire_c) begin
                    act_dsec_r <= aw_dsec_c;   // snapshot for the W phase
                    beat <= 2'd0;
                    st   <= S_W;
                end
                S_W: if (m_wready) begin
                    if (beat == act_span) begin
                        dptr <= dptr + ONE_C;
                        st   <= S_AW;
                    end else begin
                        beat <= beat + 2'd1;
                    end
                end
                // IMPORTANT-A: complete an abandoned burst with wstrb=0
                // filler beats, then sink the BRESPs DRAM still owes.
                S_RSTDRAINW: if (m_wready) begin
                    if (rdbeat == act_span) st <= S_RSTSINKB;
                    else                    rdbeat <= rdbeat + 2'd1;
                end
                // Exiting the sink state is keyed on sink_cnt reaching
                // zero rather than on "this B was the last one", so it
                // cannot matter in which order this block and the B block
                // below happen to land on the same cycle.
                default: if (sink_cnt == ZERO_C) st <= S_AW;   // S_RSTSINKB
            endcase

            // -- Outstanding-write accounting ------------------------------
            case ({aw_fire_c, b_fire_c})
                2'b10:   out_cnt <= out_cnt + ONE_C;
                2'b01:   out_cnt <= out_cnt - ONE_C;
                default: out_cnt <= out_cnt;   // 00, or 11 = net zero
            endcase

            if (b_fire_c) begin
                // No requester is waiting on a writeback's own response
                // (it's an internal action, not tied to any live AXI
                // transaction) -- there's no architectural error channel
                // yet (Minor 16), so a DRAM-side SLVERR on an evicted
                // dirty line's writeback can only be surfaced here.  It is
                // NOT silently dropped: it is counted and printed, and the
                // entry still retires (there is nothing else to do with it
                // -- the line's only copy is gone either way).
                // synthesis translate_off
                if (m_bresp != 2'b00) begin
                    /* verilator lint_off BLKSEQ */
                    wb_err_count = wb_err_count + 1;   // sim-only counter
                    /* verilator lint_on BLKSEQ */
                    if (sink_cnt != ZERO_C)
                        $display("L2C_VICTIM: sunk post-reset stray BRESP=SLVERR/DECERR (%0d) for an abandoned writeback -- no cache state to notify", m_bresp);
                    else
                        $display("L2C_VICTIM: writeback BRESP=SLVERR/DECERR (%0d) for evicted line 0x%08x -- dirty data lost, no requester to notify (see docs/l2c_spec.md Minor-16 note)",
                                  m_bresp, v_addr[bslot_c]);
                end
                // synthesis translate_on
                if (sink_cnt != ZERO_C) begin
                    // A pre-reset stray: its slot no longer exists, so it
                    // frees nothing.  No new AW is presented while
                    // sink_cnt != 0, so every B here is unambiguously one.
                    sink_cnt <= sink_cnt - ONE_C;
                end else begin
                    v_valid[bslot_c] <= 1'b0;
                    bptr <= bptr + ONE_C;
                end
            end
        end
    end

endmodule

`default_nettype wire
