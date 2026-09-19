// axi_ddr4_mig_bridge.v -- repo DDR AXI contract to pcie_test MIG AXI contract.
//
// The repo memory fabric presents a 128-bit, 6-bit-ID, 32-bit-address AXI4 DDR
// port.  The known-good ~/FPGA/pcie_test DDR4 MIG exports a 256-bit, 1-bit-ID,
// 31-bit-address AXI4 slave at the MIG UI clock.  This shim is the non-Vivado
// contract bridge for that mismatch.
//
// ── Multi-outstanding / cut-through rework (task T11) ──────────────────
// The bridge used to be single-outstanding and store-and-forward: AW was
// only issued to the MIG after an ENTIRE write burst was buffered locally,
// and only one read was ever outstanding (the next AR waited for the
// previous RLAST).  That capped channel utilisation well under 20%.  This
// version pipelines both directions:
//
//   - Reads: up to RMAX_OUTSTANDING (default 8) AR commands may be
//     in-flight.  AR is issued to the MIG as fast as m_arready allows,
//     independent of whether earlier reads have finished draining their
//     data.  The MIG ui interface returns read data STRICTLY IN THE ORDER
//     commands were accepted -- this bridge always drives m_arid=0, so
//     the guarantee this relies on is the plain AXI4 same-ID in-order-
//     response rule (all outstanding reads share one ID, so the protocol
//     itself, not any MIG-specific behaviour, fixes their return order),
//     not an assumption about the pcie_test MIG's internal scheduling
//     (docs/ddr4_mig_generation.md is silent on that) -- so a small
//     descriptor queue plus a 2-deep intake skid on the return-data path
//     is enough to unpack
//     beats back to the repo bus without forcing m_rready low every time a
//     split (lower/upper-half) beat is draining -- the old single-hold-
//     register design dropped m_rready for that entire cycle even when the
//     MIG had another beat ready, which is the "drops a cycle" bug this
//     rework removes.
//   - Writes: up to WMAX_OUTSTANDING (default 4) AW commands may be
//     in-flight.  W beats are packed and pushed into a 2-deep output skid
//     toward the MIG as soon as a 256-bit MIG beat's worth of data is
//     assembled (cut-through -- no longer buffering an entire burst before
//     the first byte reaches the MIG).  AW for the next burst is accepted
//     as soon as there is a free command-queue slot, not gated on the
//     current burst's data.  Per MIG ui contract, write DATA is never
//     forwarded ahead of its own command (the "data no later than
//     command" conservative rule -- the ordering window is otherwise
//     unspecified by the pcie_test MIG docs, see the contract doc).
//     The repo-side B response fires once the burst's LAST W beat has been
//     accepted by the MIG (m_wvalid && m_wready on the final MIG beat) --
//     matching how a native app_wdf_rdy interface reports completion --
//     rather than waiting on a downstream AXI B handshake.  The bridge
//     still drains the real m_b channel (m_bready tied high) so the
//     downstream slave is never stalled, but does not re-propagate its
//     resp upstream (documented trade-off, see the contract doc).
//   - Ordering: same-ID ordering is preserved because each direction is a
//     strict FIFO (descriptor queues drain in acceptance order).
//     Cross-direction (write-then-read to the same 64B line) ordering is
//     enforced with an explicit hazard check: a queued read's AR is held
//     back from the MIG for as long as any outstanding write whose 64B
//     line range overlaps it has not yet received its REAL m_bvalid (see
//     the bq_ queue below) -- not merely had its data locally accepted
//     into the skid.  Local skid acceptance is all the early upstream B
//     response is keyed on (see above), but neither this bridge's own sim
//     backend nor docs/ddr4_mig_generation.md substantiates that as the
//     point a subsequent read is guaranteed to observe the write, so the
//     hazard uses the strictly stronger real-B signal instead.  The
//     reverse direction (a later write racing ahead of an earlier,
//     still-queued read) is not separately guarded -- out of scope per the
//     T11 brief, see docs/ddr4_mig_bridge_contract.md.
//
// Supported traffic (unchanged from the single-outstanding version):
//   - narrow accesses from axi_narrow_to_wide: len=0, INCR, any byte offset
//     consistent with the awsize/arsize alignment rule.  axi_narrow_to_wide
//     derives `awsize` from the narrow `wstrb`:
//       * size=0 (1 B) for one-bit-set wstrb (CPU MOVE.B store)
//       * size=1 (2 B) for two-bit-set contiguous wstrb (CPU MOVE.W store)
//       * size=2 (4 B) for any other pattern (sparse / full-word)
//     Reads always use size=2 with a 4 B-aligned araddr.  The bridge
//     accepts size 0/1/2 here and lets WSTRB drive the actual byte-lane
//     selection on the wide side.
//   - aligned 128-bit repo beats and bursts: size=4, INCR, addr[3:0]=0
//   - repo bit 31 must be zero, and 128-bit bursts must not cross into bit 31
//   - repo IDs are preserved locally and replayed on B/R responses
//
// 128-bit repo burst beats are packed into 256-bit MIG INCR beats.  A start
// address with addr[4]=1 consumes the upper half of the first MIG beat, so
// that first beat is upper-half-only (flushed alone, lower half zero);
// later repo beats alternate lower/upper halves, packing two repo beats per
// MIG beat.  A burst that ends on a lower-half beat with no partner is
// likewise flushed alone (upper half zero).
//
// Unsupported traffic is completed locally with SLVERR and is never
// forwarded to the MIG.  A malformed burst (WLAST in the wrong place) that
// is detected only partway through streaming may already have forwarded a
// well-formed prefix to the MIG before the mismatch is seen -- unlike the
// old store-and-forward design, cut-through cannot roll back beats already
// pushed.  The repo-side response is still SLVERR either way (documented
// trade-off, see the contract doc).

`default_nettype none

module axi_ddr4_mig_bridge #(
    parameter RMAX_OUTSTANDING = 8,
    parameter WMAX_OUTSTANDING = 4
) (
    input  wire         clk,
    input  wire         rst,

    // Repo DDR-side AXI slave: 128-bit data, 6-bit ID, 32-bit address.
    input  wire [5:0]   s_awid,
    input  wire [31:0]  s_awaddr,
    input  wire [7:0]   s_awlen,
    input  wire [2:0]   s_awsize,
    input  wire [1:0]   s_awburst,
    input  wire         s_awvalid,
    output wire         s_awready,

    input  wire [127:0] s_wdata,
    input  wire [15:0]  s_wstrb,
    input  wire         s_wlast,
    input  wire         s_wvalid,
    output wire         s_wready,

    output wire [5:0]   s_bid,
    output wire [1:0]   s_bresp,
    output wire         s_bvalid,
    input  wire         s_bready,

    input  wire [5:0]   s_arid,
    input  wire [31:0]  s_araddr,
    input  wire [7:0]   s_arlen,
    input  wire [2:0]   s_arsize,
    input  wire [1:0]   s_arburst,
    input  wire         s_arvalid,
    output wire         s_arready,

    output wire [5:0]   s_rid,
    output wire [127:0] s_rdata,
    output wire [1:0]   s_rresp,
    output wire         s_rlast,
    output wire         s_rvalid,
    input  wire         s_rready,

    // pcie_test MIG-side AXI master: 256-bit data, 1-bit ID, 31-bit address.
    output wire [0:0]   m_awid,
    output wire [30:0]  m_awaddr,
    output wire [7:0]   m_awlen,
    output wire [2:0]   m_awsize,
    output wire [1:0]   m_awburst,
    output wire         m_awvalid,
    input  wire         m_awready,

    output wire [255:0] m_wdata,
    output wire [31:0]  m_wstrb,
    output wire         m_wlast,
    output wire         m_wvalid,
    input  wire         m_wready,

    input  wire [0:0]   m_bid,
    input  wire [1:0]   m_bresp,
    input  wire         m_bvalid,
    output wire         m_bready,

    output wire [0:0]   m_arid,
    output wire [30:0]  m_araddr,
    output wire [7:0]   m_arlen,
    output wire [2:0]   m_arsize,
    output wire [1:0]   m_arburst,
    output wire         m_arvalid,
    input  wire         m_arready,

    input  wire [0:0]   m_rid,
    input  wire [255:0] m_rdata,
    input  wire [1:0]   m_rresp,
    input  wire         m_rlast,
    input  wire         m_rvalid,
    output wire         m_rready
);

localparam [1:0] RESP_OKAY   = 2'b00;
localparam [1:0] RESP_SLVERR = 2'b10;

localparam WPTRW = (WMAX_OUTSTANDING <= 1) ? 1 : $clog2(WMAX_OUTSTANDING);
localparam WCNTW = $clog2(WMAX_OUTSTANDING+1);
localparam RPTRW = (RMAX_OUTSTANDING <= 1) ? 1 : $clog2(RMAX_OUTSTANDING);
localparam RCNTW = $clog2(RMAX_OUTSTANDING+1);

// ════════════════════════════════════════════════════════════════════
// WRITE ENGINE
// ════════════════════════════════════════════════════════════════════
//
// Descriptor queue (WMAX_OUTSTANDING deep, indexed by physical slot):
//   wq_valid[i]     -- slot i holds a live, uncommitted AW.
//   wq_ok[i]        -- contract-legal (may be downgraded to 0 mid-stream
//                      on a WLAST-placement mismatch; see header note).
//   wq_touched[i]   -- at least one MIG beat has actually been pushed for
//                      i (review I1).  A descriptor can be touched while
//                      still !ok (downgraded mid-burst after a well-formed
//                      prefix already reached the skid) -- touched, not
//                      ok, is what gates "does this descriptor still owe
//                      the MIG anything" from here on.
//   wq_aw_issued[i] -- a REAL m_awvalid&&m_awready handshake has happened
//                      for i's CURRENT occupant (review round 3
//                      re-review: this flag must mean exactly this and
//                      nothing looser -- it used to also get set by the
//                      AW-issue skip path, which does NOT talk to the
//                      MIG at all; see wq_aw_advance / wq_aw_committed).
//   wq_data_done[i] -- all of i's repo W beats have been consumed from s_w.
//   wq_mig_done[i]  -- i's full promised MIG beat count (wq_mig_beats[i],
//                      real data + any wstrb=0 padding) has been accepted
//                      downstream, i.e. the skid has fully drained i.
//   wq_line_lo/hi[i]-- i's 64B-granule line range, registered once at
//                      accept (review I3: removes the addr+len-1 adder
//                      from the per-cycle hazard-compare path -- the
//                      hazard check below is now a plain registered
//                      compare).
// Four independent pointers walk the same ring in strict FIFO order:
//   wq_wptr (accept) -> wq_aw_ptr (issue AW) -> wq_data_ptr (stream W)
//   -> wq_commit_ptr (issue B once data_done && (!touched || mig_done)).
// wq_aw_ptr and wq_data_ptr are independent of each other by design (AW
// issuance does not wait on data, and cut-through data streaming only
// waits on aw_issued for the ONE descriptor it's currently draining) --
// that decoupling is what gives >1 outstanding write.
//
// Malformed-burst containment (review I1): a WLAST-placement mismatch
// mid-burst downgrades wq_ok, but if the descriptor was already touched
// (AW issued and/or beats pushed) the bridge MUST still complete the
// exact MIG beat count it already told the MIG to expect (m_awlen) --
// short-changing a MIG burst wedges its address counter for every
// subsequent transfer on a real controller.  The "pad pusher" below
// (wr_pad_active/remaining/idx) autonomously pushes wstrb=0 filler beats
// into the SAME descriptor's remaining MIG-beat budget once its (possibly
// truncated) real data stream ends, before any later descriptor's data is
// allowed to reach the skid (s_wready is held low for the whole pad
// window -- simple and correct; padding is an error-recovery path, not a
// throughput-sensitive one).  wq_aw_advance/m_awvalid never skip issuing
// AW for a touched descriptor (an un-issued AW with beats already sitting
// in the skid would present orphaned W data with no command in front of
// it). wq_commit_ready waits for wq_mig_done whenever touched, not merely
// while ok, so a downgraded-but-touched slot is never recycled while its
// skid beats (real or padding) are still draining.

reg [5:0]  wq_id         [0:WMAX_OUTSTANDING-1];
reg [31:0] wq_addr       [0:WMAX_OUTSTANDING-1];
reg [8:0]  wq_repo_beats [0:WMAX_OUTSTANDING-1];
reg [8:0]  wq_mig_beats  [0:WMAX_OUTSTANDING-1];
reg [25:0] wq_line_lo    [0:WMAX_OUTSTANDING-1];
reg [25:0] wq_line_hi    [0:WMAX_OUTSTANDING-1];
reg        wq_ok         [0:WMAX_OUTSTANDING-1];
reg        wq_touched    [0:WMAX_OUTSTANDING-1];
reg        wq_valid      [0:WMAX_OUTSTANDING-1];
reg        wq_aw_issued  [0:WMAX_OUTSTANDING-1];
reg        wq_data_done  [0:WMAX_OUTSTANDING-1];
reg        wq_mig_done   [0:WMAX_OUTSTANDING-1];

reg [WPTRW-1:0] wq_wptr;       // next free slot for AW accept
reg [WPTRW-1:0] wq_aw_ptr;     // next slot to issue m_aw for
reg [WPTRW-1:0] wq_data_ptr;   // slot currently consuming s_w beats
// Registered "one past wq_data_ptr" copy.  Exists purely so the cwd_*
// shadow load below can address the wq_ arrays with TWO pure register
// outputs (wq_data_ptr and wq_data_ptr_p1) instead of the combinational
// wq_data_ptr_nxt -- see the shadow block's TIMING note.  Advanced from
// the SAME wq_data_ptr_nxt expression the real pointer uses, so there is
// no second hand-copied advance formula that could drift; a sim-only
// check below proves it stays exactly one ahead.
reg [WPTRW-1:0] wq_data_ptr_p1;
reg [WPTRW-1:0] wq_commit_ptr; // oldest slot awaiting s_b response
reg [WCNTW-1:0] wq_count;      // occupied slot count (accept - commit)

wire wq_full = (wq_count == WMAX_OUTSTANDING[WCNTW-1:0]);
// AXI requires ready LOW while in reset.  Ungated, a handshake completing
// during rst is accepted on the bus while the queue push below sits in the
// `else` branch of `if (rst)` -- the request vanishes and the master is owed
// a response forever.  Demonstrated in tb_axi_ddr4_mig_bridge's
// probe_ar_accepted_during_reset_is_silently_dropped.  Not reachable via the
// current sole master (its AR/AW FIFOs report empty whenever this rst is
// high), but this module is a reusable contract shim.  s_wready was already
// gated via wr_data_active; only these two were not.
assign s_awready = !wq_full && !rst;

wire [32:0] aw_last_addr   = {1'b0, s_awaddr} + ({25'b0, s_awlen} << 4);
wire [8:0]  aw_repo_beats  = {1'b0, s_awlen} + 9'd1;
wire [8:0]  aw_mig_beats_c = ({8'b0, s_awaddr[4]} + aw_repo_beats + 9'd1) >> 1;
wire        aw_word_ok = !s_awaddr[31] &&
                         (s_awlen == 8'd0) &&
                         (s_awsize <= 3'd2) &&
                         (s_awburst == 2'b01);
wire        aw_beat_ok = !s_awaddr[31] &&
                         !aw_last_addr[32] && !aw_last_addr[31] &&
                         (s_awaddr[3:0] == 4'b0000) &&
                         (s_awsize == 3'd4) &&
                         (s_awburst == 2'b01);
wire        aw_ok   = aw_word_ok || aw_beat_ok;
wire        aw_fire = s_awvalid && s_awready;
wire [8:0]  aw_mig_beats = aw_word_ok ? 9'd1 : aw_mig_beats_c;

// 64B-granule line range for this AW, computed once here and registered
// at accept (review I3) instead of recomputed combinationally every cycle
// per outstanding write for the hazard compare.
wire [39:0] aw_line_bytes     = {31'b0, aw_repo_beats} << 4;
wire [39:0] aw_line_last_byte = {8'b0, s_awaddr} + aw_line_bytes - 40'd1;
wire [25:0] aw_line_lo_c      = s_awaddr[31:6];
wire [25:0] aw_line_hi_c      = aw_line_last_byte[31:6];

// -- AW issuance: independent of data streaming, flag-indexed (no pointer-
//    wrap ambiguity: wq_valid[i] is the sole source of truth for "slot i
//    is occupied", so checking it AT wq_aw_ptr directly answers "is there
//    a not-yet-issued descriptor here" with no separate occupancy count
//    needed). --
wire wq_aw_pending = wq_valid[wq_aw_ptr] && !wq_aw_issued[wq_aw_ptr];

assign m_awid    = 1'b0;
assign m_awaddr  = {wq_addr[wq_aw_ptr][30:5], 5'b00000};
assign m_awlen   = wq_mig_beats[wq_aw_ptr][7:0] - 8'd1;
assign m_awsize  = 3'd5;
assign m_awburst = 2'b01;
// A touched descriptor (review I1) must have its AW issued even if it has
// since been downgraded -- its beats are already committed to the skid
// and need a command in front of them.  Gated by !bq_full (review C2):
// every real AW issuance also enters the hazard-tracking bq_ queue below,
// which is sized to WMAX_OUTSTANDING and therefore can only ever be full
// when WMAX_OUTSTANDING real AWs are already in flight -- the natural
// bound on how many this pointer could possibly present anyway, so this
// never introduces a new stall in practice, only guards the invariant.
//
// CRITICAL FIX (review round 3, the AW twin of the C1 ar_presenting fix):
// m_awvalid must not deassert without a handshake either.  wq_ok[wq_aw_ptr]
// is a LIVE signal that can fall (WLAST-mismatch downgrade) WHILE this
// exact AW is presenting and unaccepted -- AW issuance is deliberately
// eager (independent of, and often ahead of, data streaming), so it is
// entirely possible for m_awvalid to have asserted using the pre-downgrade
// wq_ok=1, then have wq_ok flip to 0 a few stalled cycles later with
// wq_touched STILL 0 (no prefix pushed yet).  Sticky aw_presenting latches
// the admission decision the cycle it is made and holds m_awvalid
// regardless of a later wq_ok/wq_touched change, until the real m_awready
// handshake completes -- exactly mirroring ar_presenting.
//
// TIMING (round 2, 333 MHz post-route).  m_awvalid USED to be
// `aw_presenting || wq_aw_want`, i.e. a COMBINATIONAL function of four
// wq_aw_ptr-indexed array reads plus bq_full -- three LUT levels -- on a
// net with fanout 52 that leaves the module and lands deep inside the
// MIG's own AXI shim.  That single cone accounted for ~50 of the 72
// failing endpoints in two shapes:
//   wq_ok_reg[0]/C      -> m_awvalid -> u_ddr_axi/axi_{aw,ar,b}_channel_0
//                          .../{axaddr_incr,axlen_cnt,cnt_read}_reg/CE  (-0.029)
//   wq_touched_reg[3]/C -> m_awvalid -> m_wvalid -> wrout_pop -> s_wready
//                          -> u_core_to_mig_ui/u_w_pad/pad_r_reg/D      (-0.037)
// Neither is fixable inside the MIG, and the second one loops the same
// combinational cone back through our own skid into upstream s_wready.
//
// m_awvalid is now a PURE REGISTER OUTPUT (aw_presenting alone).  The
// admission decision still uses exactly the same wq_aw_want expression,
// it just takes effect on the following cycle, so AW is presented one
// cycle after the decision rather than in the same cycle.
//
// Correctness: this only ever DELAYS an AW.  aw_presenting was already
// the sticky "admission latched, hold m_awvalid until a real handshake"
// flop, so every downstream user of the AW-handshake event
// (wq_aw_advance, the wq_aw_issued set, wrout_head_aw_issuing_now,
// bq_push) still keys off the identical `m_awvalid && m_awready`
// condition and is unchanged.  wq_aw_want retains its `!aw_presenting`
// term, so the two else-if arms below stay mutually exclusive and a
// descriptor can never be admitted twice.  AXI-wise this is strictly
// MORE conservative: m_awvalid can now only ever deassert on a real
// handshake.
//
// Cost: peak AW issue rate falls from one per cycle to one per two
// cycles (the handshake cycle itself has aw_presenting=1, so wq_aw_want
// is masked and the next admission lands the cycle after).  That cannot
// bind here -- AW issuance is already capped at WMAX_OUTSTANDING=4 in
// flight by bq_full/wq_full, the repo side is a 100 MHz fabric domain
// behind a CDC FIFO, and every MIG burst this bridge emits occupies at
// least one 256-bit W beat anyway.
reg aw_presenting;
wire wq_aw_want = !aw_presenting && wq_aw_pending &&
                  (wq_ok[wq_aw_ptr] || wq_touched[wq_aw_ptr]) && !bq_full;
assign m_awvalid = aw_presenting;

always @(posedge clk) begin
    if (rst) begin
        aw_presenting <= 1'b0;
    end else if (m_awvalid && m_awready) begin
        aw_presenting <= 1'b0;
    end else if (wq_aw_want) begin
        aw_presenting <= 1'b1;
    end
end

wire wq_aw_advance = wq_aw_pending &&
                     ((!wq_ok[wq_aw_ptr] && !wq_touched[wq_aw_ptr] && !aw_presenting) ||
                      (m_awvalid && m_awready));

// -- W-side packing state for the descriptor currently at wq_data_ptr --
// AXI allows (and real masters routinely do) presenting AW and its first W
// beat on the SAME cycle.  wq_data_ptr's array entries are only valid
// starting the cycle AFTER acceptance (non-blocking writes), so when this
// cycle's AW accept targets the exact slot wq_data_ptr is about to
// consume (queue was empty / just caught up), bypass straight to the
// incoming s_aw* values instead of reading stale/undefined array state --
// mirrors the original single-outstanding design's WR_IDLE same-cycle
// AW+W special case.  wr_pad_active additionally excludes a fresh
// descriptor from being active while a PRIOR descriptor's padding is
// still draining (s_wready is held low for that whole window).
// wq_dp_eq_wp is a REGISTERED copy of (wq_data_ptr == wq_wptr) -- see the
// "current W descriptor context" block below for why, and for the
// sim-only self-check that proves it never drifts from the literal
// comparison.  It is used only here, on the timing-critical arm; the
// accept block below still writes its array entries using the plain
// literal comparison (those are ordinary register D paths, not the
// wrout_* write-enable path this fix is about).
wire wr_bypass = aw_fire && wq_dp_eq_wp;

wire wr_data_active = !wr_pad_active && (wr_bypass || cwd_active);

reg  [8:0]   wr_recv_count;      // repo beats consumed so far for wq_data_ptr
reg  [8:0]   wr_mig_pushed_count;// MIG beats actually pushed so far for
                                  // wq_data_ptr (real pushes only; used to
                                  // compute how much padding, if any, a
                                  // malformed burst still owes)
reg          wr_pack_valid;  // a lower-half beat is held, awaiting its
                              // upper-half partner
reg  [127:0] wr_pack_data;
reg  [15:0]  wr_pack_strb;

reg          wr_pad_active;      // padding a downgraded-but-touched burst
reg  [8:0]   wr_pad_remaining;   // MIG beats still owed
reg  [WPTRW-1:0] wr_pad_idx;     // which wq_ slot owes them

// TIMING FIX (333 MHz DDR clock, 96 of the 100 worst violated endpoints):
// the non-bypass arms used to be COMBINATIONAL wq_*[wq_data_ptr] array
// reads.  wq_data_ptr (fo=48) therefore drove a WMAX_OUTSTANDING-wide
// (4:1) mux over four arrays, and the mux output fed the whole packing
// decision -- wr_is_last_expected / wr_terminate_now / wr_completes_now
// / wr_real_need_push / s_wready -- which is precisely the write-enable
// for the wrout_data / wrout_strb distributed RAMs.  Reported path:
// wq_data_ptr_reg[0]/C -> 8 LUT levels -> wrout_strb_reg_0_1_0_13/RAM*/WE
// (and, via s_wready, out to u_core_to_mig_ui/u_w_pad/pad_r_reg).
//
// The non-bypass arms now come from cwd_* -- a registered shadow of the
// descriptor at wq_data_ptr, maintained by the block after the main write
// FSM.  wq_data_ptr no longer reaches the wrout_* write enables at all:
// the 4:1 mux moved behind the cwd_* flops, and the residual
// (wq_data_ptr == wq_wptr) term moved behind the wq_dp_eq_wp flop.
//
// The BYPASS arm stays combinational, deliberately.  wr_bypass is the
// same-cycle AW-accept + first-W-beat coincidence (see wr_bypass above):
// the descriptor's array entries do not exist yet that cycle, and neither
// does its shadow, so the values MUST come straight off the s_aw* inputs.
// Registering that arm would push s_wready low for a cycle on every
// isolated write -- a throughput and latency regression on the most
// common traffic pattern this bridge sees -- and would change the
// carefully-reasoned bypass behaviour the module header documents.  The
// bypass arm is also not the timing problem: s_awaddr/s_awvalid are
// module inputs, already registered upstream.
wire        cur_wdata_ok       = wr_bypass ? aw_ok         : cwd_ok;
wire        cur_wdata_addr4    = wr_bypass ? s_awaddr[4]   : cwd_addr4;
wire [8:0]  cur_wdata_repobeat = wr_bypass ? aw_repo_beats : cwd_repobeat;
wire [8:0]  cur_wdata_migbeat  = wr_bypass ? aw_mig_beats  : cwd_migbeat;

wire [8:0]  wr_half_index = {8'b0, cur_wdata_addr4} + wr_recv_count;
wire        wr_upper      = wr_half_index[0];
wire        wr_is_last_expected = (wr_recv_count == (cur_wdata_repobeat - 9'd1));
wire        wr_wlast_ok         = (s_wlast == wr_is_last_expected);
// Data reception for the current descriptor ends the moment EITHER the
// promised repo-beat count is reached OR the master asserts WLAST early
// (review I1: without the `|| s_wlast` term, a real WLAST arriving before
// the expected position would never be recognised as ending the burst --
// wr_recv_count would just keep counting against a length that will never
// be reached again, hanging s_wready low forever on that beat sequence).
// A WLAST that arrives LATER than expected (the promised position passes
// with s_wlast=0) still terminates AT the promised position, same as
// before -- matches the original single-outstanding design's containment
// for that sub-case.
wire wr_terminate_now = wr_is_last_expected || s_wlast;
// A MIG beat completes (is ready to push) whenever the incoming repo beat
// lands in the upper half of its 32B slot (normal pairing, or a lone
// upper-starting beat with no stored partner) -- OR data reception is
// ending regardless of parity (a lone trailing/truncated beat must still
// flush, zero-filling the missing half).
wire wr_completes_now = wr_upper || wr_terminate_now;

wire [127:0] wr_pack_base_data = wr_pack_valid ? wr_pack_data : 128'b0;
wire [15:0]  wr_pack_base_strb = wr_pack_valid ? wr_pack_strb : 16'b0;
wire [255:0] wr_push_data = wr_upper ? {s_wdata, wr_pack_base_data}
                                      : {wr_pack_base_data, s_wdata};
wire [31:0]  wr_push_strb = wr_upper ? {s_wstrb, wr_pack_base_strb}
                                      : {wr_pack_base_strb, s_wstrb};

// -- 2-deep output skid toward the MIG --------------------------------
// Small elastic buffer -- NOT a full-burst store; gives enough slack to
// absorb a couple of MIG beats' worth of cut-through data without forcing
// s_wready low on every push, while remaining fundamentally streaming
// (nothing waits for the whole burst, unlike the old design).
reg [255:0]     wrout_data [0:1];
reg [31:0]      wrout_strb [0:1];
reg             wrout_last [0:1];
reg [WPTRW-1:0] wrout_idx  [0:1];
reg             wrout_wptr, wrout_rptr;
reg [1:0]       wrout_cnt;

wire wrout_full2 = (wrout_cnt == 2'd2);
wire wrout_pop   = m_wvalid && m_wready;

// The beat that FIRST reveals a WLAST-placement mismatch is itself never
// pushed -- only a genuinely well-formed PRIOR prefix (already pushed on
// earlier beats, tracked via wr_mig_pushed_count) can make a descriptor
// "touched".  Without the `&& wr_wlast_ok` term here, a single-beat burst
// whose only beat has the wrong WLAST would still get real data pushed
// (using the pre-downgrade wq_ok, since the downgrade itself only
// registers at the end of this same cycle) and would incorrectly acquire
// a real AW/touched status identical to a legitimately-touched multi-beat
// burst -- there is no "already in the pipe" prefix to protect in that
// case, so it should stay a pure local SLVERR, exactly like an address-
// rejected write that never touches the MIG at all.
wire wr_real_need_push = wr_completes_now && cur_wdata_ok && wr_wlast_ok && !wr_pad_active;
wire wr_out_room       = !wrout_full2 || wrout_pop;
wire wrout_push_real   = s_wvalid && s_wready && wr_real_need_push;

// How many MIG beats this descriptor will have had pushed after THIS
// cycle (real pushes only), and whether that already completes its
// promised MIG-beat budget -- used both to trigger padding on a malformed
// termination and to tag the correct beat as the skid entry's WLAST
// (review I1: the old `wr_is_last_expected`-based tag no longer applies
// once a burst can be padded to completion by a LATER, synthetic beat).
wire [8:0] wr_pushed_total_after  = wr_mig_pushed_count + (wr_real_need_push ? 9'd1 : 9'd0);
wire       wr_real_push_is_final  = (wr_pushed_total_after == cur_wdata_migbeat);
// IMPORTANT FIX (review round 3): AW issuance is eager -- independent of,
// and often ahead of, data streaming -- so a descriptor's AW can already
// be issued (or irrevocably committed to issuing, via sticky
// aw_presenting) BEFORE any mismatch is even detected, with ZERO real
// beats ever pushed (e.g. WLAST asserted early inside what would have
// been the FIRST MIG-beat pair).  The original wr_mig_pushed_count-only
// trigger missed this entirely: a zero-prefix-but-committed AW would
// never get padded, leaving the MIG mid-burst forever (address desync
// for every subsequent transfer on a real controller), its bq_ entry
// never popping (real B never arrives -> any overlapping read is
// permanently hazard-blocked, and enough of these wedge bq_ full ->
// blocks all further writes too).
//
// FIX (review round 3 re-review, both flavors confirmed by reviewer
// repro): the naive "wq_aw_issued[wq_data_ptr] || same-cycle-presenting"
// form above LIED in two ways:
//   (a) wq_aw_issued used to also get set by the AW-issue SKIP path (an
//       accept-rejected-and-never-touched descriptor whose AW is never
//       actually sent to the MIG) -- so a rejected descriptor's later
//       malformed W data would read wq_aw_committed=1 and get padded to
//       the MIG with ZERO real AW handshake ever having occurred
//       (tb_orphan: misaligned addr + early WLAST -> W beats incl WLAST
//       reach the MIG with 0 AW handshakes).  Fixed below: the skip path
//       no longer touches wq_aw_issued at all (see "AW issuance"), so the
//       flag now means exactly what its name says -- a real m_awvalid &&
//       m_awready handshake happened for this physical slot's CURRENT
//       occupant.
//   (b) even with (a) fixed, wq_aw_issued[wq_data_ptr] is STALE during
//       the wr_bypass coincidence (wq_data_ptr == wq_wptr, a brand-new
//       slot being accepted THIS cycle): the array read this cycle still
//       returns the PREVIOUS occupant of that physical index's value
//       (NBA write hasn't landed yet), not "0 for a definitely-unissued
//       new descriptor" -- so a legal single-beat write landing on a
//       physical slot whose last occupant happened to have been really
//       issued would read a false wq_aw_committed=1 and get an orphan pad
//       beat with 0 AW handshake for THIS descriptor (tb_stale: after >=4
//       retired writes wrap the ring, one legal write with wlast=0 ->
//       1 orphan pad beat).  Command-less W beats desync a real MIG's
//       write FIFO, silently corrupting subsequent legal writes.
// Fixed with `!wr_bypass` (a brand-new slot's AW cannot possibly have
// issued yet -- kills (b) unconditionally) and `cur_wdata_ok` (the
// descriptor's OWN registered accept decision, pre-downgrade -- kills
// (a) as defense in depth: a rejected descriptor has cur_wdata_ok=0
// regardless of what (a) put in the array).  `!wr_bypass` also restores
// the wq_touched dual-writer agreement the sim-only self-check
// below depends on (see its header comment).
wire wq_aw_committed = !wr_bypass && cur_wdata_ok &&
                       (wq_aw_issued[wq_data_ptr] ||
                        ((wq_aw_ptr == wq_data_ptr) && (aw_presenting || wq_aw_want)));
// Padding requires !wr_wlast_ok, whereas wr_real_need_push requires
// wr_wlast_ok. Thus no real push can occur on a padding-trigger cycle:
// wr_pushed_total_after is exactly wr_mig_pushed_count on that cycle.
// Use that identity explicitly to keep the real-push decision and its
// increment out of the padding comparator/CE path (333 MHz MIG UI).
// No register, handshake, latency, or malformed-burst policy changes.
wire       wr_pad_trigger = wq_w_fire && wr_terminate_now && !wr_wlast_ok &&
                            ((wr_mig_pushed_count != 9'd0) || wq_aw_committed) &&
                            (wr_mig_pushed_count < cur_wdata_migbeat);
wire [8:0] wr_pad_count_needed = cur_wdata_migbeat - wr_mig_pushed_count;

// synthesis translate_off
wire wr_pad_trigger_reference = wq_w_fire && wr_terminate_now && !wr_wlast_ok &&
    (wr_real_need_push || (wr_mig_pushed_count != 9'd0) || wq_aw_committed) &&
    (wr_pushed_total_after < cur_wdata_migbeat);
wire [8:0] wr_pad_count_reference = cur_wdata_migbeat - wr_pushed_total_after;
always @(posedge clk) begin
    if (!rst) begin
        if (wr_pad_trigger !== wr_pad_trigger_reference)
            $fatal(1, "padding trigger equivalence failed");
        if (wr_pad_trigger && (wr_pad_count_needed !== wr_pad_count_reference))
            $fatal(1, "padding count equivalence failed");
    end
end
// synthesis translate_on

// Padding pusher: wstrb=0 filler beats, tagged onto the SAME descriptor
// (wr_pad_idx) that owes them, draining independently of upstream s_w
// activity (s_wready is held low for the whole window via wr_data_active
// above, so no other descriptor's real data can jump the queue ahead of
// this one's required completion).
wire wrout_push_pad = wr_pad_active && wr_out_room && (wr_pad_remaining != 9'd0);
wire [255:0] wr_pad_push_data = 256'b0;
wire [31:0]  wr_pad_push_strb = 32'b0;
wire         wr_pad_push_last = (wr_pad_remaining == 9'd1);

wire wrout_push = wrout_push_real || wrout_push_pad;

// s_wready: ready whenever there is a descriptor to stream (and no prior
// descriptor's padding is still draining -- see wr_data_active), AND
// either this beat doesn't need to push (just latches into wr_pack) or
// the output skid has room to accept the completed MIG beat.  The "data
// no later than command" ordering rule (see header note) is enforced
// downstream instead -- at the skid's DRAIN side (m_wvalid, below) -- so
// that a same-cycle AW+first-W-beat (routine, normal AXI traffic) is
// never forced to stall waiting for an AW that hasn't even had a chance
// to be issued yet.  The skid can hold data for an unissued AW; it just
// won't present it to the MIG until that AW has gone out.
assign s_wready = wr_data_active && (!wr_real_need_push || wr_out_room);

wire wq_w_fire = s_wvalid && s_wready;

// -- explicit next-state for the accept and data pointers ----------------
// Factored out (they used to be inlined in the always block) so that the
// registered shadow state below is guaranteed to be derived from the
// EXACT same expressions the pointers themselves use -- there is no
// second, hand-copied formula that could drift.  The always block now
// assigns `wq_wptr <= wq_wptr_nxt` / `wq_data_ptr <= wq_data_ptr_nxt`
// unconditionally, which is identical to the old conditional increments
// because both _nxt wires hold their pointer when their advance condition
// is false.
wire wq_accept_advance = aw_fire;
wire wq_data_advance   = wq_w_fire && wr_terminate_now;
wire [WPTRW-1:0] wq_wptr_nxt =
        wq_accept_advance
            ? ((wq_wptr == WMAX_OUTSTANDING-1) ? {WPTRW{1'b0}} : wq_wptr + 1'b1)
            : wq_wptr;
wire [WPTRW-1:0] wq_data_ptr_nxt =
        wq_data_advance
            ? ((wq_data_ptr == WMAX_OUTSTANDING-1) ? {WPTRW{1'b0}} : wq_data_ptr + 1'b1)
            : wq_data_ptr;
// One-past copy, derived from the SAME _nxt expression (no second formula).
wire [WPTRW-1:0] wq_data_ptr_p1_nxt =
        (wq_data_ptr_nxt == WMAX_OUTSTANDING-1) ? {WPTRW{1'b0}}
                                                : wq_data_ptr_nxt + 1'b1;

integer wi;
always @(posedge clk) begin
    if (rst) begin
        for (wi = 0; wi < WMAX_OUTSTANDING; wi = wi + 1) begin
            wq_valid[wi]     <= 1'b0;
            wq_ok[wi]        <= 1'b0;
            wq_touched[wi]   <= 1'b0;
            wq_aw_issued[wi] <= 1'b0;
            wq_data_done[wi] <= 1'b0;
            wq_mig_done[wi]  <= 1'b0;
        end
        wq_wptr             <= {WPTRW{1'b0}};
        wq_aw_ptr            <= {WPTRW{1'b0}};
        wq_data_ptr          <= {WPTRW{1'b0}};
        wq_data_ptr_p1       <= {{(WPTRW-1){1'b0}}, 1'b1};
        wq_commit_ptr        <= {WPTRW{1'b0}};
        wq_count             <= {WCNTW{1'b0}};
        wr_recv_count        <= 9'd0;
        wr_mig_pushed_count  <= 9'd0;
        wr_pack_valid        <= 1'b0;
        wr_pack_data         <= 128'b0;
        wr_pack_strb         <= 16'b0;
        wr_pad_active        <= 1'b0;
        wr_pad_remaining     <= 9'd0;
        wr_pad_idx           <= {WPTRW{1'b0}};
    end else begin
        // -- accept new AW --
        //
        // wq_ok[wq_wptr], wq_data_done[wq_wptr], and wq_touched[wq_wptr]
        // below are written using an explicit priority expression, NOT a
        // plain default, because a same-cycle AW+final-W-beat (the
        // wr_bypass case, routine AXI traffic -- see wr_bypass above) can
        // target this EXACT slot on THIS SAME cycle from the "W data
        // consume" block further down.  Two independent nonblocking
        // assignments to the same dynamically-indexed array element in
        // one always-block ARE well-defined by the LRM (last one executed
        // wins -- IEEE 1800-2017 10.4.2 / IEEE 1364-2005 5.4.1) but
        // relying on that across two separately-reasoned-about `if`
        // blocks is fragile.  Confirmed via an isolated repro
        // (scratchpad/nba_repro/) that this exact array-of-dynamic-index
        // pattern does NOT resolve as expected by the open-source sim
        // tool used here (first-executed wins instead) -- a genuine
        // sim/synth divergence risk, since Vivado follows the LRM.  Both
        // writers below compute the SAME final value instead of racing to
        // overwrite each other, so the result is correct under EITHER
        // resolution order.  See the sim-only self-check after this
        // block.
        if (aw_fire) begin
            wq_id[wq_wptr]         <= s_awid;
            wq_addr[wq_wptr]       <= s_awaddr;
            wq_repo_beats[wq_wptr] <= aw_repo_beats;
            wq_mig_beats[wq_wptr]  <= aw_mig_beats;
            wq_line_lo[wq_wptr]    <= aw_line_lo_c;
            wq_line_hi[wq_wptr]    <= aw_line_hi_c;
            wq_ok[wq_wptr]         <= (wq_w_fire && !wr_wlast_ok && (wq_data_ptr == wq_wptr))
                                       ? 1'b0 : aw_ok;
            wq_valid[wq_wptr]      <= 1'b1;
            wq_aw_issued[wq_wptr]  <= 1'b0;
            wq_data_done[wq_wptr]  <= (wq_w_fire && wr_terminate_now && (wq_data_ptr == wq_wptr))
                                       ? 1'b1 : 1'b0;
            wq_touched[wq_wptr]    <= (wq_data_ptr == wq_wptr) ? wrout_push_real : 1'b0;
            wq_mig_done[wq_wptr]   <= 1'b0;
        end
        wq_wptr <= wq_wptr_nxt;

        // -- AW issuance --
        // wq_aw_issued is set ONLY on a real m_awvalid&&m_awready handshake
        // -- split out from the pointer-advance below (review round 3
        // re-review Fix 1(a)): wq_aw_advance also fires on the SKIP branch
        // (a rejected-and-never-touched descriptor whose AW is never sent
        // to the MIG at all), and setting wq_aw_issued there used to make
        // the flag lie -- a later malformed W beat for that same
        // rejected-and-skipped descriptor would read "AW committed" and
        // get padded to the MIG despite zero AW handshake ever occurring
        // (tb_orphan).  The pointer still advances on skip (an
        // accept-rejected descriptor must not stall the AW-issue ring),
        // it just no longer claims an issuance that didn't happen.
        if (wq_aw_advance) begin
            wq_aw_ptr <= (wq_aw_ptr == WMAX_OUTSTANDING-1) ? {WPTRW{1'b0}} : wq_aw_ptr + 1'b1;
        end
        if (m_awvalid && m_awready) begin
            wq_aw_issued[wq_aw_ptr] <= 1'b1;
        end

        // -- W data consume --
        if (wq_w_fire) begin
            if (!wr_wlast_ok) begin
                wq_ok[wq_data_ptr] <= 1'b0;
            end
            // wr_real_need_push and wr_pad_trigger are mutually exclusive
            // (the former requires wr_wlast_ok, the latter !wr_wlast_ok),
            // so this never double-drives the same index for two
            // different reasons in the same cycle.  The wr_pad_trigger
            // arm covers the zero-prefix-but-AW-committed case (review
            // round 3 I1): wq_touched must still become 1 so the slot-
            // retirement gate below (wq_commit_ready) waits for the
            // padding to fully drain before recycling this slot.
            if (wr_real_need_push || wr_pad_trigger) begin
                wq_touched[wq_data_ptr] <= 1'b1;
            end
            if (wr_completes_now) begin
                wr_pack_valid <= 1'b0;
            end else begin
                wr_pack_data  <= s_wdata;
                wr_pack_strb  <= s_wstrb;
                wr_pack_valid <= 1'b1;
            end
            if (wr_pad_trigger) begin
                wr_pad_active    <= 1'b1;
                wr_pad_remaining <= wr_pad_count_needed;
                wr_pad_idx       <= wq_data_ptr;
            end
            if (wr_terminate_now) begin
                wq_data_done[wq_data_ptr] <= 1'b1;
                wr_recv_count       <= 9'd0;
                wr_mig_pushed_count <= 9'd0;
            end else begin
                wr_recv_count <= wr_recv_count + 9'd1;
                if (wr_real_need_push) begin
                    wr_mig_pushed_count <= wr_pushed_total_after;
                end
            end
        end
        wq_data_ptr    <= wq_data_ptr_nxt;
        wq_data_ptr_p1 <= wq_data_ptr_p1_nxt;

        // -- pad pusher: drains independently of s_w activity (s_wready
        //    is held low for the whole window, so this can never coincide
        //    with a fresh wr_pad_trigger -- see the header note). --
        if (wrout_push_pad) begin
            if (wr_pad_remaining == 9'd1) begin
                wr_pad_active <= 1'b0;
            end
            wr_pad_remaining <= wr_pad_remaining - 9'd1;
        end

        // -- mig-accepted (from output skid pop): tracks the FULL
        //    promised MIG beat count (real + any padding), since that is
        //    what the commit/retire logic below needs -- "has the skid
        //    fully drained this descriptor's obligation to the MIG". --
        if (wrout_pop && wrout_last[wrout_rptr]) begin
            wq_mig_done[wrout_idx[wrout_rptr]] <= 1'b1;
        end

        // -- commit pop (s_b handshake) --
        if (s_bvalid && s_bready) begin
            wq_valid[wq_commit_ptr] <= 1'b0;
            wq_commit_ptr <= (wq_commit_ptr == WMAX_OUTSTANDING-1) ? {WPTRW{1'b0}} : wq_commit_ptr + 1'b1;
        end

        // -- occupancy --
        case ({aw_fire, (s_bvalid && s_bready)})
            2'b10:   wq_count <= wq_count + 1'b1;
            2'b01:   wq_count <= wq_count - 1'b1;
            default: wq_count <= wq_count;
        endcase
    end
end

// ════════════════════════════════════════════════════════════════════
// Registered "current W descriptor" context (cwd_*) -- TIMING
// ════════════════════════════════════════════════════════════════════
//
// cwd_* is a registered shadow of the wq_ arrays AT wq_data_ptr, plus
// wq_dp_eq_wp, a registered copy of (wq_data_ptr == wq_wptr).  Together
// they take wq_data_ptr entirely off the wrout_data/wrout_strb
// write-enable path (see the cur_wdata_* comment above for the measured
// path this removes).  The shadow is reloaded EVERY cycle from the array
// at wq_data_ptr_nxt, so it is coherent by construction the cycle
// wq_data_ptr actually lands there -- there is no "load on advance only"
// state machine to get wrong.
//
// COHERENCE OBLIGATION.  An array read at index wq_data_ptr_nxt this
// cycle returns the PRE-nonblocking-assignment value, so any writer
// targeting that same index this cycle would be missed.  Enumerating
// every writer of every shadowed array:
//
//  1. accept (aw_fire, index wq_wptr) writes wq_id/addr/repo_beats/
//     mig_beats/ok/valid/data_done.  Handled EXPLICITLY by
//     cwd_load_bypass, which takes the incoming s_aw*/aw_* values
//     instead of the stale array read.
//     Under cwd_load_bypass, the accept block's own conditional terms
//     collapse: it writes wq_ok[wq_wptr] <= (wq_w_fire && !wr_wlast_ok
//     && wq_data_ptr==wq_wptr) ? 0 : aw_ok, and wq_data_done[wq_wptr] <=
//     (wq_w_fire && wr_terminate_now && wq_data_ptr==wq_wptr) ? 1 : 0.
//     Both extra terms require wr_terminate_now (note !wr_wlast_ok
//     IMPLIES wr_terminate_now: wr_terminate_now = wr_is_last_expected ||
//     s_wlast, and wr_wlast_ok = (s_wlast == wr_is_last_expected), so
//     !wr_wlast_ok means exactly one of the two is set).  wr_terminate_now
//     with wq_data_ptr == wq_wptr forces wq_data_ptr_nxt = wq_wptr + 1,
//     which contradicts cwd_load_bypass's (wq_wptr == wq_data_ptr_nxt).
//     So under the bypass the writes reduce to aw_ok and data_done=0 --
//     exactly what is loaded below.
//
//  2. W-consume (index wq_data_ptr) writes wq_ok (downgrade) and
//     wq_data_done (set).  BOTH are gated by conditions that imply
//     wr_terminate_now (see above), which implies wq_data_advance, which
//     makes wq_data_ptr_nxt = wq_data_ptr + 1 != wq_data_ptr.  So the
//     write index and the shadow's read index can never coincide.
//     *** This is the one place the shadow depends on the ring being
//     bigger than one slot; the generate-time check below enforces it. ***
//
//  3. commit-pop (index wq_commit_ptr) clears wq_valid.  It can only fire
//     when wq_commit_ready, which requires wq_data_done[wq_commit_ptr].
//     cwd_active loads (wq_valid[nxt] && !wq_data_done[nxt]); if
//     wq_commit_ptr == wq_data_ptr_nxt then wq_data_done[nxt] is already
//     1, so the stale wq_valid=1 is ANDed away and the loaded value (0)
//     matches the post-clear truth (0).  No bypass needed.
//
// The sim-only block after this one checks the resulting invariant
// directly against live array reads on every cycle of every scenario --
// including the 10k-op random scoreboard -- so this argument is not
// merely asserted in prose.
//
// TIMING (round 2, 333 MHz post-route: 20 of the 72 failing endpoints,
// including the worst at -0.040 ns, were cwd_repobeat/cwd_migbeat/
// cwd_addr4/cwd_active D pins).  The COHERENCE argument above is about
// WHICH INDEX is read; it says nothing about how that index is formed.
// The load used to be written literally as an array read at the
// COMBINATIONAL wq_data_ptr_nxt, and post-route that cost three extra
// logic levels on the shadow's own D path:
//     aw_fifo RAMD32 (s_awlen) -> aw_repo_beats -> cur_wdata_repobeat
//       -> wr_is_last_expected (9-bit compare) -> wr_terminate_now
//       -> wq_data_advance -> wq_data_ptr_nxt -> 4:1 array mux -> cwd_*
// (10 logic levels, 3.002 ns).  Note in particular that the old claim
// "the bypass arm is not the timing problem because s_aw* are already
// registered upstream" is FALSE in the real SoC: s_awaddr/s_awlen come
// out of a DISTRIBUTED-RAM async FIFO (axi_async_bridge's aw_fifo), so
// they arrive as a RAMD32 CLK->O plus routing, not as a flop Q.
//
// The fix is a pure select-vs-mux reassociation, NOT a semantic change:
//     wq_x[wq_data_ptr_nxt]
//   == wq_data_advance ? wq_x[wq_data_ptr_p1] : wq_x[wq_data_ptr]
//     cwd_load_bypass
//   == wq_data_advance ? cwd_bypass_p1        : cwd_bypass_cur
// Both array reads now use PURE REGISTER addresses, so both candidate
// values (already bypass-muxed) are settled early; the late-arriving
// wq_data_advance now feeds exactly ONE final 2:1 mux into the D pin
// instead of an address computation plus a 4:1 mux plus a bypass mux.
// The set of indices read, the values loaded, and therefore the entire
// coherence argument above are bit-for-bit unchanged -- and the sim-only
// self-check below (which compares the shadow to a LIVE array read at
// wq_data_ptr) still covers it, plus a new direct check that the
// reassociated bypass select equals the original expression.
reg        cwd_ok;
reg        cwd_addr4;
reg [8:0]  cwd_repobeat;
reg [8:0]  cwd_migbeat;
reg        cwd_active;
reg        wq_dp_eq_wp;

wire cwd_load_bypass = aw_fire && (wq_wptr == wq_data_ptr_nxt);

// Early (register-addressed) candidates: index wq_data_ptr, taken when
// wq_data_advance is 0.
wire       cwd_bypass_cur   = aw_fire && (wq_wptr == wq_data_ptr);
wire       cwd_cur_ok       = cwd_bypass_cur ? aw_ok         : wq_ok[wq_data_ptr];
wire       cwd_cur_addr4    = cwd_bypass_cur ? s_awaddr[4]   : wq_addr[wq_data_ptr][4];
wire [8:0] cwd_cur_repobeat = cwd_bypass_cur ? aw_repo_beats : wq_repo_beats[wq_data_ptr];
wire [8:0] cwd_cur_migbeat  = cwd_bypass_cur ? aw_mig_beats  : wq_mig_beats[wq_data_ptr];
wire       cwd_cur_active   = cwd_bypass_cur ? 1'b1
                                             : (wq_valid[wq_data_ptr] &&
                                                !wq_data_done[wq_data_ptr]);
wire       cwd_cur_dp_eq_wp = (wq_data_ptr == wq_wptr_nxt);

// Early (register-addressed) candidates: index wq_data_ptr_p1, taken
// when wq_data_advance is 1.
wire       cwd_bypass_p1    = aw_fire && (wq_wptr == wq_data_ptr_p1);
wire       cwd_p1_ok        = cwd_bypass_p1 ? aw_ok         : wq_ok[wq_data_ptr_p1];
wire       cwd_p1_addr4     = cwd_bypass_p1 ? s_awaddr[4]   : wq_addr[wq_data_ptr_p1][4];
wire [8:0] cwd_p1_repobeat  = cwd_bypass_p1 ? aw_repo_beats : wq_repo_beats[wq_data_ptr_p1];
wire [8:0] cwd_p1_migbeat   = cwd_bypass_p1 ? aw_mig_beats  : wq_mig_beats[wq_data_ptr_p1];
wire       cwd_p1_active    = cwd_bypass_p1 ? 1'b1
                                            : (wq_valid[wq_data_ptr_p1] &&
                                               !wq_data_done[wq_data_ptr_p1]);
wire       cwd_p1_dp_eq_wp  = (wq_data_ptr_p1 == wq_wptr_nxt);

always @(posedge clk) begin
    if (rst) begin
        cwd_ok       <= 1'b0;
        cwd_addr4    <= 1'b0;
        cwd_repobeat <= 9'd0;
        cwd_migbeat  <= 9'd0;
        cwd_active   <= 1'b0;
        // Both pointers reset to 0, so the registered equality must too.
        wq_dp_eq_wp  <= 1'b1;
    end else begin
        // Late signal (wq_data_advance) drives ONE mux stage; see note.
        cwd_ok       <= wq_data_advance ? cwd_p1_ok       : cwd_cur_ok;
        cwd_addr4    <= wq_data_advance ? cwd_p1_addr4    : cwd_cur_addr4;
        cwd_repobeat <= wq_data_advance ? cwd_p1_repobeat : cwd_cur_repobeat;
        cwd_migbeat  <= wq_data_advance ? cwd_p1_migbeat  : cwd_cur_migbeat;
        cwd_active   <= wq_data_advance ? cwd_p1_active   : cwd_cur_active;
        wq_dp_eq_wp  <= wq_data_advance ? cwd_p1_dp_eq_wp : cwd_cur_dp_eq_wp;
    end
end

// synthesis translate_off
// ── AW-channel protocol monitor ──────────────────────────────────────
// AXI4 A3.2.1: once AWVALID is asserted it must stay asserted, with the
// payload unchanged, until AWREADY.  m_awvalid became a pure register
// output as part of the 333 MHz timing fix above, which makes this
// structural -- this monitor is the end-to-end proof rather than a
// code-reading claim, and being in the RTL it runs across ALL tb
// scenarios (including the 10k-op random scoreboard) instead of one
// directed case.  Everything it reads is a flop, so sampling at posedge
// gives cycle N-1 vs cycle N-2 -- i.e. consecutive-cycle comparison.
reg        awmon_v;
reg        awmon_r;
reg [30:0] awmon_a;
reg [7:0]  awmon_l;
reg [0:0]  awmon_i;
always @(posedge clk) begin
    if (!rst && awmon_v && !awmon_r) begin
        if (m_awvalid !== 1'b1) begin
            $display("ASSERTION FAIL: m_awvalid deasserted without m_awready t=%0t",
                     $time);
        end
        if ((m_awaddr !== awmon_a) || (m_awlen !== awmon_l) ||
            (m_awid !== awmon_i)) begin
            $display("ASSERTION FAIL: m_aw payload changed while held: addr %h->%h len %0d->%0d t=%0t",
                     awmon_a, m_awaddr, awmon_l, m_awlen, $time);
        end
    end
    awmon_v <= rst ? 1'b0 : m_awvalid;
    awmon_r <= m_awready;
    awmon_a <= m_awaddr;
    awmon_l <= m_awlen;
    awmon_i <= m_awid;
end

// Positive-control self-check for the cwd_* shadow and wq_dp_eq_wp.
// These compare the REGISTERED shadow against a LIVE array read of the
// thing it shadows -- two independent expressions, not one expression
// compared to itself -- so they genuinely fire if the coherence argument
// above is ever violated (verified by temporarily breaking the shadow).
// Payload fields are only meaningful while the slot is occupied, hence
// the wq_valid gate; cwd_active and wq_dp_eq_wp are checked always.
initial begin
    if (WMAX_OUTSTANDING < 2) begin
        $display("ASSERTION FAIL: cwd_* shadow requires WMAX_OUTSTANDING >= 2 (got %0d)",
                 WMAX_OUTSTANDING);
        $finish;
    end
end
always @(posedge clk) begin
    if (!rst) begin
        // Reassociation positive controls (round-2 timing fix).  These
        // compare the NEW early-addressed formulation against the
        // ORIGINAL late-addressed expressions, combinationally, every
        // cycle.  If the select-vs-mux reassociation is ever wrong --
        // or wq_data_ptr_p1 drifts off wq_data_ptr+1 -- these fire
        // BEFORE the registered-shadow checks below would.
        if (wq_data_ptr_p1 !== ((wq_data_ptr == WMAX_OUTSTANDING-1)
                                ? {WPTRW{1'b0}} : wq_data_ptr + 1'b1)) begin
            $display("ASSERTION FAIL: wq_data_ptr_p1=%0d but wq_data_ptr=%0d t=%0t",
                     wq_data_ptr_p1, wq_data_ptr, $time);
        end
        if (cwd_load_bypass !== (wq_data_advance ? cwd_bypass_p1 : cwd_bypass_cur)) begin
            $display("ASSERTION FAIL: cwd_load_bypass=%0b but reassociated=%0b t=%0t",
                     cwd_load_bypass,
                     (wq_data_advance ? cwd_bypass_p1 : cwd_bypass_cur), $time);
        end
        if ((wq_data_advance ? cwd_p1_repobeat : cwd_cur_repobeat) !==
            (cwd_load_bypass ? aw_repo_beats : wq_repo_beats[wq_data_ptr_nxt])) begin
            $display("ASSERTION FAIL: cwd repobeat load mismatch new=%0d old=%0d t=%0t",
                     (wq_data_advance ? cwd_p1_repobeat : cwd_cur_repobeat),
                     (cwd_load_bypass ? aw_repo_beats : wq_repo_beats[wq_data_ptr_nxt]),
                     $time);
        end
        if ((wq_data_advance ? cwd_p1_active : cwd_cur_active) !==
            (cwd_load_bypass ? 1'b1 : (wq_valid[wq_data_ptr_nxt] &&
                                       !wq_data_done[wq_data_ptr_nxt]))) begin
            $display("ASSERTION FAIL: cwd_active load mismatch t=%0t", $time);
        end
        if (wq_dp_eq_wp !== (wq_data_ptr == wq_wptr)) begin
            $display("ASSERTION FAIL: wq_dp_eq_wp=%0b but (wq_data_ptr==wq_wptr)=%0b t=%0t",
                     wq_dp_eq_wp, (wq_data_ptr == wq_wptr), $time);
        end
        if (cwd_active !== (wq_valid[wq_data_ptr] && !wq_data_done[wq_data_ptr])) begin
            $display("ASSERTION FAIL: cwd_active=%0b but live=%0b ptr=%0d t=%0t",
                     cwd_active,
                     (wq_valid[wq_data_ptr] && !wq_data_done[wq_data_ptr]),
                     wq_data_ptr, $time);
        end
        if (wq_valid[wq_data_ptr]) begin
            if (cwd_ok !== wq_ok[wq_data_ptr]) begin
                $display("ASSERTION FAIL: cwd_ok=%0b but wq_ok[%0d]=%0b t=%0t",
                         cwd_ok, wq_data_ptr, wq_ok[wq_data_ptr], $time);
            end
            if (cwd_addr4 !== wq_addr[wq_data_ptr][4]) begin
                $display("ASSERTION FAIL: cwd_addr4=%0b but wq_addr[%0d][4]=%0b t=%0t",
                         cwd_addr4, wq_data_ptr, wq_addr[wq_data_ptr][4], $time);
            end
            if (cwd_repobeat !== wq_repo_beats[wq_data_ptr]) begin
                $display("ASSERTION FAIL: cwd_repobeat=%0d but wq_repo_beats[%0d]=%0d t=%0t",
                         cwd_repobeat, wq_data_ptr, wq_repo_beats[wq_data_ptr], $time);
            end
            if (cwd_migbeat !== wq_mig_beats[wq_data_ptr]) begin
                $display("ASSERTION FAIL: cwd_migbeat=%0d but wq_mig_beats[%0d]=%0d t=%0t",
                         cwd_migbeat, wq_data_ptr, wq_mig_beats[wq_data_ptr], $time);
            end
        end
    end
end
// synthesis translate_on

// Sim-only (sim-only-guarded) self-check (review headline hardening; FIXED round 3 per
// review Important-2, then re-fixed round 3 re-review). On a coincidence
// cycle (wr_bypass: the accept writer and the W-consume writer target the
// SAME physical slot), assert the two writers' INTENDED values agree --
// purely combinationally, same cycle, never routed through the register
// or any NBA resolution.
//
// History: v1 compared the LANDED REGISTER against a re-derivation of the
// ACCEPT writer's own formula -- tautological under Verilator's confirmed
// first-write-wins resolution for this pattern (register always equals
// the accept writer, so the check could never fire even if the consume
// writer's formula had drifted -- exactly the sim/synth divergence it
// existed to catch, since Vivado's LRM last-write-wins resolution takes
// the consume writer). v2 (this file, briefly) replaced it with three
// `expr !== expr` comparisons -- for wq_ok/wq_data_done that is because
// the accept and consume writers share the identical subexpression BY
// CONSTRUCTION (there is only one live condition, `wr_wlast_ok` /
// `wr_terminate_now`, and both writers reduce to the same ternary of it),
// so comparing the expression to itself is dead at elaboration -- not a
// bug in the writers, just a non-assertion.  Documented honestly below
// instead of faked as a check.  wq_touched's writers genuinely differ
// (accept: `wrout_push_real`; consume: `wr_real_need_push ||
// wr_pad_trigger`) -- THAT is the one comparison worth actually making,
// and is implemented below.
// synthesis translate_off
always @(posedge clk) begin
    if (!rst && aw_fire && wq_w_fire && (wq_data_ptr == wq_wptr)) begin
        // wq_ok -- accept ternary (this block, "accept new AW" above):
        //   (wq_w_fire && !wr_wlast_ok && coincidence) ? 1'b0 : aw_ok
        // consume write (this block, "W data consume" above):
        //   if (!wr_wlast_ok) wq_ok[wq_data_ptr] <= 1'b0;  (else: no
        //   opinion -- the coincidence slot's only other writer, accept,
        //   stands with aw_ok)
        // Both writers reduce to the SAME ternary of the single live
        // condition wr_wlast_ok -- there is no second independent
        // formula to disagree with, so no assertion is coded here (a
        // literal expr-vs-itself compare would be dead at elaboration).

        // wq_data_done -- accept ternary:
        //   (wq_w_fire && wr_terminate_now && coincidence) ? 1'b1 : 1'b0
        // consume write: if (wr_terminate_now) wq_data_done[..] <= 1'b1;
        // Same situation as wq_ok: both writers reduce to the same
        // ternary of the single live condition wr_terminate_now. No
        // assertion coded for the same reason.

        // wq_touched -- accept ternary: coincidence ? wrout_push_real : 0
        // consume write: if (wr_real_need_push || wr_pad_trigger)
        //   wq_touched[..] <= 1'b1;
        // These are GENUINELY DIFFERENT expressions (wrout_push_real vs.
        // wr_real_need_push || wr_pad_trigger) that happen to be equal
        // only because wr_pad_trigger is provably false under this exact
        // coincidence gate: wq_aw_committed (which wr_pad_trigger
        // requires) is unconditionally gated by `!wr_bypass` (review
        // round 3 re-review Fix 1), and wr_bypass is true by definition
        // whenever this coincidence gate (aw_fire && wq_w_fire &&
        // wq_data_ptr==wq_wptr) is true -- so wr_pad_trigger's
        // wq_aw_committed term is always 0 here, leaving `wr_real_need_push
        // || wr_pad_trigger` == wr_real_need_push == wrout_push_real
        // (wrout_push_real IS wr_real_need_push gated by wq_w_fire, which
        // this coincidence already implies). This is the one comparison
        // that is not dead at elaboration: it holds with Fix 1 in place,
        // and would fire if that invariant ever regressed.
        if (wrout_push_real !== (wr_real_need_push || wr_pad_trigger)) begin
            $display("ASSERTION FAIL: wq_touched accept/consume disagreement idx=%0d t=%0t",
                     wq_wptr, $time);
        end
    end
end
// synthesis translate_on

// -- output-skid state machine ------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        wrout_wptr <= 1'b0;
        wrout_rptr <= 1'b0;
        wrout_cnt  <= 2'd0;
    end else begin
        if (wrout_push) begin
            wrout_data[wrout_wptr] <= wrout_push_pad ? wr_pad_push_data : wr_push_data;
            wrout_strb[wrout_wptr] <= wrout_push_pad ? wr_pad_push_strb : wr_push_strb;
            wrout_last[wrout_wptr] <= wrout_push_pad ? wr_pad_push_last : wr_real_push_is_final;
            wrout_idx[wrout_wptr]  <= wrout_push_pad ? wr_pad_idx : wq_data_ptr;
            wrout_wptr <= ~wrout_wptr;
        end
        if (wrout_pop) begin
            wrout_rptr <= ~wrout_rptr;
        end
        case ({wrout_push, wrout_pop})
            2'b10:   wrout_cnt <= wrout_cnt + 2'd1;
            2'b01:   wrout_cnt <= wrout_cnt - 2'd1;
            default: wrout_cnt <= wrout_cnt;
        endcase
    end
end

assign m_wdata  = wrout_data[wrout_rptr];
assign m_wstrb  = wrout_strb[wrout_rptr];
assign m_wlast  = wrout_last[wrout_rptr];
// Conservative "data no later than command" ordering rule: don't present
// a buffered beat to the MIG until its owning descriptor's AW has been
// issued (or is being issued THIS SAME cycle -- "no later than" permits
// simultaneous, and gating strictly on the registered wq_aw_issued flag
// would otherwise force an artificial 1-cycle gap between AW and its
// first W beat even for a totally isolated, uncontended write, which is
// routine AXI traffic).  Safe to key on the head-of-skid entry alone: AW
// issuance (wq_aw_ptr) and skid draining are both strict FIFOs over the
// same descriptor order, so an unissued head can never be masking an
// issued entry behind it.
wire wrout_head_aw_issuing_now = (wq_aw_ptr == wrout_idx[wrout_rptr]) &&
                                 m_awvalid && m_awready;
assign m_wvalid = (wrout_cnt != 2'd0) &&
                  (wq_aw_issued[wrout_idx[wrout_rptr]] || wrout_head_aw_issuing_now);

// Downstream B is always sunk (never re-propagated -- see header note on
// early commit).  If a real HW MIG ever reports a genuine write error via
// bresp, it is invisible upstream by design of this early-ack contract
// (documented trade-off, see docs/ddr4_mig_bridge_contract.md).
assign m_bready = 1'b1;

// wq_valid's two writers (accept sets 1 at wq_wptr; commit-pop clears 0 at
// wq_commit_ptr) can only coincide on the same index at either the empty
// ring state (wq_wptr == wq_commit_ptr, nothing valid there, so no commit
// -pop event is even possible that cycle) or the full state (accept is
// already blocked that cycle by s_awready=!wq_full computed from the
// PRE-pop occupancy) -- safe by the queue's own occupancy invariant, no
// explicit agreement needed (documented per review headline hardening).
wire wq_commit_pending = wq_valid[wq_commit_ptr];
// Waits for wq_mig_done whenever the descriptor was ever touched, not
// merely while it is still ok (review I1) -- a downgraded-but-touched
// slot must not be recycled while real or padding beats are still
// draining through the skid.  This also RETROACTIVELY makes the
// wq_mig_done pair (accept resets to 0 at wq_wptr; the skid-drain event
// sets 1 at wrout_idx) provably single-writer-safe: a slot can only be
// freed (and therefore reused by a later accept) once wq_mig_done was
// ALREADY 1 for it in a strictly earlier cycle, so the set-event for any
// physical slot's PREVIOUS occupant always precedes the reset-event for
// its NEXT occupant -- they can never target the same index on the same
// cycle (documented per review headline hardening; no restructuring
// needed since I1's fix already eliminates the coincidence).
wire wq_commit_ready   = wq_commit_pending && wq_data_done[wq_commit_ptr] &&
                         (!wq_touched[wq_commit_ptr] || wq_mig_done[wq_commit_ptr]);

assign s_bid    = wq_id[wq_commit_ptr];
assign s_bresp  = wq_ok[wq_commit_ptr] ? RESP_OKAY : RESP_SLVERR;
assign s_bvalid = wq_commit_ready;

// -- bq_: hazard-tracking queue, real-B-response-cleared (review C2) ----
// Separate from the wq_ descriptor ring on purpose: wq_ slots retire
// (become reusable) as soon as the LOCAL skid has drained a descriptor
// (wq_mig_done, above), but the REAL downstream write -- and therefore
// the point a subsequent read is guaranteed to observe it on an in-order
// MIG -- isn't confirmed until the real m_bvalid arrives, which can lag
// well behind local skid-drain.  Tracking hazard state directly on the
// wq_ ring would let a slot recycle (and get overwritten by a totally
// unrelated later write) before its real B arrives, corrupting the
// hazard state for whichever write happens to land in that physical slot
// next.  bq_ instead holds its own independent, real-B-popped copy of
// each in-flight write's line range, pushed at the SAME moment m_awvalid
// is actually accepted (wq_aw_want, both ok and touched-downgraded
// writes) and popped on the real m_bvalid (single ID, m_bready tied high
// elsewhere -- B responses return in order per the AXI4 same-ID rule,
// independent of any MIG-specific ordering assumption).  Sized to
// WMAX_OUTSTANDING and never overflows: wq_aw_want's `!bq_full` term
// backpressures AW issuance itself, so bq_ can never be asked to hold
// more entries than it has room for.
reg               bq_valid   [0:WMAX_OUTSTANDING-1];
reg [25:0]        bq_line_lo [0:WMAX_OUTSTANDING-1];
reg [25:0]        bq_line_hi [0:WMAX_OUTSTANDING-1];
reg [WPTRW-1:0]   bq_wptr, bq_rptr;
reg [WCNTW-1:0]   bq_count;

wire bq_full = (bq_count == WMAX_OUTSTANDING[WCNTW-1:0]);
wire bq_push = m_awvalid && m_awready;
wire bq_pop  = m_bvalid && m_bready;

integer bi;
always @(posedge clk) begin
    if (rst) begin
        for (bi = 0; bi < WMAX_OUTSTANDING; bi = bi + 1) begin
            bq_valid[bi] <= 1'b0;
        end
        bq_wptr  <= {WPTRW{1'b0}};
        bq_rptr  <= {WPTRW{1'b0}};
        bq_count <= {WCNTW{1'b0}};
    end else begin
        // bq_push targets bq_wptr; bq_pop targets bq_rptr.  These can
        // only coincide on the same physical index at empty (bq_pop can't
        // fire -- nothing valid to report a real B for) or full (bq_push
        // is already blocked that cycle by bq_full computed from the
        // PRE-pop occupancy) -- same occupancy-invariant argument as
        // wq_valid above, no agreement needed.
        if (bq_push) begin
            bq_valid[bq_wptr]   <= 1'b1;
            bq_line_lo[bq_wptr] <= wq_line_lo[wq_aw_ptr];
            bq_line_hi[bq_wptr] <= wq_line_hi[wq_aw_ptr];
            bq_wptr <= (bq_wptr == WMAX_OUTSTANDING-1) ? {WPTRW{1'b0}} : bq_wptr + 1'b1;
        end
        if (bq_pop) begin
            bq_valid[bq_rptr] <= 1'b0;
            bq_rptr <= (bq_rptr == WMAX_OUTSTANDING-1) ? {WPTRW{1'b0}} : bq_rptr + 1'b1;
        end
        case ({bq_push, bq_pop})
            2'b10:   bq_count <= bq_count + 1'b1;
            2'b01:   bq_count <= bq_count - 1'b1;
            default: bq_count <= bq_count;
        endcase
    end
end

// ════════════════════════════════════════════════════════════════════
// READ ENGINE
// ════════════════════════════════════════════════════════════════════
//
// Same three-pointer-plus-flags structure as the write side, minus the
// early-ack commit complexity (s_r data IS the final answer, no local
// "done" bookkeeping needed beyond the drain walk itself):
//   rq_valid[i]  -- slot i holds a live, undrained AR.
//   rq_issued[i] -- m_ar has been issued (or skipped, for !ok) for i.
//   rq_line_lo/hi[i] -- i's 64B-granule line range, registered once at
//                       accept (review I3, mirrors wq_line_lo/hi).
// rq_wptr (accept) -> rq_issue_ptr (issue AR, hazard-gated) ->
// rq_drain_ptr (unpack MIG data / local SLVERR back to s_r).

reg [5:0]  rq_id         [0:RMAX_OUTSTANDING-1];
reg [31:0] rq_addr       [0:RMAX_OUTSTANDING-1];
reg [8:0]  rq_repo_beats [0:RMAX_OUTSTANDING-1];
reg [8:0]  rq_mig_beats  [0:RMAX_OUTSTANDING-1];
reg [25:0] rq_line_lo    [0:RMAX_OUTSTANDING-1];
reg [25:0] rq_line_hi    [0:RMAX_OUTSTANDING-1];
reg        rq_ok         [0:RMAX_OUTSTANDING-1];
reg        rq_valid      [0:RMAX_OUTSTANDING-1];
reg        rq_issued     [0:RMAX_OUTSTANDING-1];

reg [RPTRW-1:0] rq_wptr;       // next free slot for AR accept
reg [RPTRW-1:0] rq_issue_ptr;  // next slot to issue m_ar for
reg [RPTRW-1:0] rq_drain_ptr;  // slot currently draining to s_r
reg [RCNTW-1:0] rq_count;      // occupied slot count

wire rq_full = (rq_count == RMAX_OUTSTANDING[RCNTW-1:0]);
assign s_arready = !rq_full && !rst;

wire [32:0] ar_last_addr   = {1'b0, s_araddr} + ({25'b0, s_arlen} << 4);
wire [8:0]  ar_repo_beats  = {1'b0, s_arlen} + 9'd1;
wire [8:0]  ar_mig_beats_c = ({8'b0, s_araddr[4]} + ar_repo_beats + 9'd1) >> 1;
wire        ar_word_ok = !s_araddr[31] &&
                         (s_arlen == 8'd0) &&
                         (s_arsize <= 3'd2) &&
                         (s_arburst == 2'b01);
wire        ar_beat_ok = !s_araddr[31] &&
                         !ar_last_addr[32] && !ar_last_addr[31] &&
                         (s_araddr[3:0] == 4'b0000) &&
                         (s_arsize == 3'd4) &&
                         (s_arburst == 2'b01);
wire        ar_ok   = ar_word_ok || ar_beat_ok;
wire        ar_fire = s_arvalid && s_arready;

// 64B-granule line range for this AR, computed once here and registered
// at accept (review I3), mirroring the write side.
wire [39:0] ar_line_bytes     = {31'b0, ar_repo_beats} << 4;
wire [39:0] ar_line_last_byte = {8'b0, s_araddr} + ar_line_bytes - 40'd1;
wire [25:0] ar_line_lo_c      = s_araddr[31:6];
wire [25:0] ar_line_hi_c      = ar_line_last_byte[31:6];

// -- same-64B-line hazard check (review C2 / I3) -------------------------
// Block AR issuance while any WRITE STILL AWAITING ITS REAL m_bvalid
// (tracked in the bq_ queue above, popped only on the real B -- not
// merely local MIG-beat acceptance, which is all the old wq_mig_done
// signal this used to key on actually guaranteed) has a 64B-granule line
// range overlapping the candidate read's.  Both ranges are registered at
// accept (I3), so this reduces to a bank of registered 26-bit compares,
// not an adder chain evaluated combinationally every cycle.
wire rq_issue_pending = rq_valid[rq_issue_ptr] && !rq_issued[rq_issue_ptr];

wire [WMAX_OUTSTANDING-1:0] wr_hazard_hit;
genvar gi;
generate
    for (gi = 0; gi < WMAX_OUTSTANDING; gi = gi + 1) begin : g_hazard
        wire overlap = !(rq_line_hi[rq_issue_ptr] < bq_line_lo[gi] ||
                        rq_line_lo[rq_issue_ptr] > bq_line_hi[gi]);
        assign wr_hazard_hit[gi] = bq_valid[gi] && overlap;
    end
endgenerate
wire rd_hazard = |wr_hazard_hit;

// -- AR output stage: FULLY REGISTERED toward the MIG (timing) -----------
//
// TIMING FIX (333 MHz DDR clock, post-route WNS -0.213 ns, 4 of the 100
// worst violated endpoints): this used to be a purely combinational AR
// presentation --
//
//     assign m_araddr  = {rq_addr[rq_issue_ptr][30:5], 5'b00000};
//     assign m_arlen   = rq_mig_beats[rq_issue_ptr][7:0] - 8'd1;
//     assign m_arvalid = ar_presenting || ar_want;
//
// -- so rq_issue_ptr fanned out (fo=57) into an RMAX_OUTSTANDING-wide
// (8:1) mux over rq_addr / rq_mig_beats / rq_line_lo / rq_line_hi and,
// worse, straight through the 26-bit rd_hazard comparator bank into
// m_arvalid, which then drove the MIG's own AR command translator.  The
// reported worst path was 11 logic levels spanning both modules:
//
//   rq_issue_ptr_reg[2]_replica/C
//     -> rq_line_lo_reg_0_7_0_13/RAMC/O        (8:1 array mux, RAMD32)
//     -> g_hazard[2].overlap1_carry (LUT4 + 2x CARRY8)  (26-bit compares)
//     -> mig_arvalid_INST_0_i_3 / _i_1 -> ar_want -> m_arvalid (LUT6,LUT6,LUT2)
//     -> u_mig_ddr4/.../axi_mc_incr_cmd_0/axlen_cnt_reg[7]/D  (LUT5,LUT6,LUT6,LUT5)
//
// Vivado had already tried duplicating the POINTER (rq_issue_ptr_reg[2]
// _replica) on its own; that cannot help, because the cost is the mux and
// the comparator bank BEHIND the pointer, not the pointer's own fanout.
//
// The fix is a proper AXI register slice on AR: m_arvalid, m_araddr and
// m_arlen are now plain flip-flop outputs.  Everything that used to sit
// between rq_issue_ptr and the MIG (the 8:1 payload mux, the 26-bit
// hazard comparators, the ar_want reduction) is now confined to the D
// inputs of three local registers inside this module, and the MIG sees
// only Q pins.  Nothing from this module's arrays reaches the MIG's AR
// inputs combinationally any more.
//
// AXI4 conformance of the slice (A3.2.1/A3.2.2):
//   - m_arvalid is a register, so it is NOT a combinational function of
//     m_arready.  ar_want (the LOAD ENABLE) may look at m_arready -- that
//     is the standard register-slice/skid form and is explicitly legal;
//     only VALID itself must not depend on READY.
//   - Payload stability while VALID is high: ar_addr_q/ar_len_q change
//     only when ar_want fires, and ar_want requires ar_slot_free
//     (!ar_presenting || m_arready).  So while an AR is presented and
//     unaccepted, ar_want is 0 and the payload is frozen.
//   - VALID never drops without a handshake: ar_presenting clears only in
//     the `ar_presenting && m_arready` arm.  Reloading back-to-back on
//     the handshake cycle keeps it high, so full-rate (1 AR/cycle) issue
//     is preserved -- this costs latency, not throughput.
//
// Cost: exactly one extra cycle between "descriptor is issue-eligible"
// and "AR is presented to the MIG".  Against DDR4 CAS latency that is
// noise, and the tb's eight_pipelined_reads_speedup / back-to-back read
// scenarios still measure the same pipelined behaviour.
//
// Hazard semantics preserved (this is the old ar_presenting argument,
// unchanged in substance): rd_hazard is a level computed from CURRENTLY
// live outstanding writes and can RISE while an AR is already committed.
// That rise is provably irrelevant to an AR already in flight -- hazard
// terms only ever fall over time as writes retire (see bq_ above), so a
// hazard appearing AFTER this AR was admitted can only be a YOUNGER
// write, and a younger write must never gate an OLDER read.  Latching
// the admission decision (formerly "sticky ar_presenting", now "capture
// into the AR register") is what makes that safe, and the capture is if
// anything a stronger form of it: the payload is frozen too, not just
// the valid bit.
//
// Ordering note (documented widening of an already-unguarded window):
// the AW channel is deliberately NOT registered here (it does not appear
// anywhere in the violated-path set), so an AR now reaches the MIG one
// cycle later than an AW admitted on the same cycle.  The write-then-read
// direction is unaffected -- it is guarded by the bq_ hazard check, which
// is evaluated at capture and only ever gets MORE conservative if the AR
// is delayed.  The reverse direction (a later write racing ahead of an
// earlier, still-queued read) is already explicitly out of scope per the
// module header / docs/ddr4_mig_bridge_contract.md; this makes that
// pre-existing window one cycle wider, it does not create a new one.
reg         ar_presenting;   // == m_arvalid; an AR is registered & unaccepted
reg [30:0]  ar_addr_q;
reg [7:0]   ar_len_q;

// Load enable for the AR register.  May depend on m_arready (see above):
// this is the slice's load enable, not its VALID.
wire ar_slot_free = !ar_presenting || m_arready;
wire ar_want = ar_slot_free && rq_issue_pending && rq_ok[rq_issue_ptr] && !rd_hazard;

assign m_arid    = 1'b0;
assign m_araddr  = ar_addr_q;
assign m_arlen   = ar_len_q;
assign m_arsize  = 3'd5;
assign m_arburst = 2'b01;
assign m_arvalid = ar_presenting;

always @(posedge clk) begin
    if (rst) begin
        ar_presenting <= 1'b0;
        ar_addr_q     <= 31'b0;
        ar_len_q      <= 8'b0;
    end else if (ar_want) begin
        ar_presenting <= 1'b1;
        ar_addr_q     <= {rq_addr[rq_issue_ptr][30:5], 5'b00000};
        ar_len_q      <= rq_mig_beats[rq_issue_ptr][7:0] - 8'd1;
    end else if (ar_presenting && m_arready) begin
        ar_presenting <= 1'b0;
    end
end

// The descriptor is consumed at CAPTURE time, not at handshake time.
// This is mandatory with a registered AR: if rq_issue_ptr only advanced
// on m_arready, then on the handshake cycle ar_slot_free would be 1 while
// rq_issue_ptr still pointed at the descriptor already sitting in the
// register -- and ar_want would re-capture it, issuing the SAME AR twice.
// Advancing (and setting rq_issued) at capture makes "captured into the
// AR register" the single commit point.  rq_issued has no other consumer
// than rq_issue_pending, so nothing else observes the retiming.
wire rq_issue_advance = rq_issue_pending &&
                        (!rq_ok[rq_issue_ptr] || ar_want);

// -- 2-deep intake skid from the MIG return channel ----------------------
reg [255:0] rdin_data [0:1];
reg [1:0]   rdin_resp [0:1];
reg         rdin_last [0:1];
reg         rdin_wptr, rdin_rptr;
reg [1:0]   rdin_cnt;

wire rdin_full2 = (rdin_cnt == 2'd2);

wire [8:0] cur_rdrain_repobeat = rq_repo_beats[rq_drain_ptr];
wire [8:0] cur_rdrain_migbeat  = rq_mig_beats[rq_drain_ptr];
wire       cur_rdrain_ok       = rq_ok[rq_drain_ptr];
wire       cur_rdrain_addr4    = rq_addr[rq_drain_ptr][4];

reg         rd_cur_valid;
reg [255:0] rd_cur_data;
reg [1:0]   rd_cur_resp;
reg [8:0]   rd_repo_index;   // repo beats emitted so far for rq_drain_ptr
reg [8:0]   rd_mig_index;    // mig beats consumed so far for rq_drain_ptr

wire [8:0] rd_half_index = {8'b0, cur_rdrain_addr4} + rd_repo_index;
wire       rd_upper      = rd_half_index[0];
wire       rd_last_repo  = (rd_repo_index == (cur_rdrain_repobeat - 9'd1));
wire       rd_last_mig   = (rd_mig_index == (cur_rdrain_migbeat - 9'd1));
wire [1:0] rd_beat_resp  = (rdin_last[rdin_rptr] == rd_last_mig) ? rdin_resp[rdin_rptr] : RESP_SLVERR;

wire rd_drain_has_entry = rq_valid[rq_drain_ptr];
wire rd_drain_is_err    = rd_drain_has_entry && !cur_rdrain_ok;
wire rd_drain_is_ok     = rd_drain_has_entry && cur_rdrain_ok;

// OK-mode fill: rd_cur is refilled from the intake skid whenever it holds
// something, or -- when the skid is empty -- directly, combinationally,
// from this cycle's m_rdata ("bypass").  The bypass path is what avoids
// adding a mandatory cycle of latency to the very first beat of a read
// (which would otherwise be a strictly worse latency floor than the old
// single-outstanding design): its data reaches s_rdata the SAME cycle
// m_rvalid asserts, via the rd_eff_* combinational mux below, while
// SIMULTANEOUSLY being registered into rd_cur so a second (upper-half)
// repo beat, if this MIG beat needs one, is still available next cycle --
// it is not also pushed into the skid (that would duplicate it).  This
// never has to withhold m_rready just because a held upper-half is
// draining -- the skid independently accepts the next MIG beat as long as
// it has room (up to 2 in flight), and a subsequent MIG beat arriving
// while THIS one bypasses still lands safely in the empty skid via the
// same push logic.
wire rd_fill_from_skid = rd_drain_is_ok && !rd_cur_valid && (rdin_cnt != 2'd0);
wire rd_fill_from_mig  = rd_drain_is_ok && !rd_cur_valid && (rdin_cnt == 2'd0) && m_rvalid;
wire rdin_pop  = rd_fill_from_skid;
wire rdin_push = m_rvalid && m_rready && !rd_fill_from_mig;

assign m_rready = rd_drain_is_ok && (!rdin_full2 || rdin_pop);

// rd_bypass: this cycle's output is being served directly from whichever
// source is filling rd_cur (rd_cur itself has not yet registered it --
// that happens on this same edge).  Review I2: the skid-sourced fill
// (rd_fill_from_skid) used to be registered-only, costing a full extra
// cycle every time the skid (rather than a direct mig bypass) supplies
// the beat -- on a sustained back-to-back MIG return stream, that is a
// real, measurable throughput loss (roughly 3 cycles per 2 beats instead
// of 2, ~33%).  Extending the combinational mux to also serve the skid
// head directly removes that bubble: EVERY fill, regardless of source,
// reaches s_rdata the same cycle it lands, while still registering into
// rd_cur for a possible second (upper-half) repo beat next cycle.
wire       rd_bypass_mig  = rd_fill_from_mig;
wire       rd_bypass_skid = rd_fill_from_skid;
wire       rd_bypass      = rd_bypass_mig || rd_bypass_skid;
wire       rd_eff_valid = rd_cur_valid || rd_bypass;
wire [255:0] rd_eff_data = rd_cur_valid ? rd_cur_data :
                           (rd_bypass_skid ? rdin_data[rdin_rptr] : m_rdata);
wire [1:0] rd_eff_resp   = rd_cur_valid ? rd_cur_resp :
                           (rd_bypass_skid ? rd_beat_resp :
                            ((m_rlast == rd_last_mig) ? m_rresp : RESP_SLVERR));

wire rd_err_out_valid = rd_drain_is_err;
wire rd_ok_out_valid  = rd_drain_is_ok && rd_eff_valid;

assign s_rvalid = rd_err_out_valid || rd_ok_out_valid;
assign s_rid    = rq_id[rq_drain_ptr];
assign s_rdata  = rd_err_out_valid ? 128'b0 :
                  (rd_upper ? rd_eff_data[255:128] : rd_eff_data[127:0]);
assign s_rresp  = rd_err_out_valid ? RESP_SLVERR : rd_eff_resp;
// A locally-rejected read completes with the FULL arlen+1 beats the master
// asked for, every beat RRESP=SLVERR / RDATA=0 and RLAST on the last one --
// exactly the shape rd_last_repo already computes for the OK path, so the
// error path simply uses it too.
//
// This used to be a "fast-reject shortcut" that returned ONE truncated
// SLVERR beat with RLAST set regardless of arlen (task: xbar-burst-gaps,
// GAP 3).  A master that asked for N beats got 1, so it either half-filled
// a cache line and hung waiting for an RLAST that already went by, or
// silently proceeded with garbage.  That was tolerable only while nothing
// on this fabric could issue a rejected MULTI-beat read; it is a latent
// protocol violation regardless and is now closed.
//
// Single-beat behaviour is bit-identical: for arlen=0, rq_repo_beats=1 and
// rd_repo_index=0, so rd_last_repo is 1 on the one and only beat.
assign s_rlast  = rd_last_repo;

wire s_r_fire = s_rvalid && s_rready;
// Does this fire consume the entirety of the MIG beat currently backing
// the output (no more halves needed from it)?  True for the upper half of
// a pair, or any lone (edge) beat that is also the burst's last repo beat.
wire rd_cur_done_after_fire = rd_upper || rd_last_repo;
// GAP 3: the error path no longer pops after one beat -- it walks
// rd_repo_index through the whole requested burst like the OK path, so the
// descriptor retires on the SAME beat that carries RLAST in both modes.
wire rd_drain_last_beat = rd_last_repo;
wire rd_drain_pop = s_r_fire && rd_drain_last_beat;

integer ri;
always @(posedge clk) begin
    if (rst) begin
        for (ri = 0; ri < RMAX_OUTSTANDING; ri = ri + 1) begin
            rq_valid[ri]  <= 1'b0;
            rq_issued[ri] <= 1'b0;
        end
        rq_wptr       <= {RPTRW{1'b0}};
        rq_issue_ptr  <= {RPTRW{1'b0}};
        rq_drain_ptr  <= {RPTRW{1'b0}};
        rq_count      <= {RCNTW{1'b0}};
        rd_cur_valid  <= 1'b0;
        rd_cur_data   <= 256'b0;
        rd_cur_resp   <= RESP_OKAY;
        rd_repo_index <= 9'd0;
        rd_mig_index  <= 9'd0;
    end else begin
        // -- accept new AR --
        if (ar_fire) begin
            rq_id[rq_wptr]         <= s_arid;
            rq_addr[rq_wptr]       <= s_araddr;
            rq_repo_beats[rq_wptr] <= ar_repo_beats;
            rq_mig_beats[rq_wptr]  <= ar_word_ok ? 9'd1 : ar_mig_beats_c;
            rq_line_lo[rq_wptr]    <= ar_line_lo_c;
            rq_line_hi[rq_wptr]    <= ar_line_hi_c;
            rq_ok[rq_wptr]         <= ar_ok;
            rq_valid[rq_wptr]      <= 1'b1;
            rq_issued[rq_wptr]     <= 1'b0;
            rq_wptr <= (rq_wptr == RMAX_OUTSTANDING-1) ? {RPTRW{1'b0}} : rq_wptr + 1'b1;
        end

        // -- AR issuance --
        if (rq_issue_advance) begin
            rq_issued[rq_issue_ptr] <= 1'b1;
            rq_issue_ptr <= (rq_issue_ptr == RMAX_OUTSTANDING-1) ? {RPTRW{1'b0}} : rq_issue_ptr + 1'b1;
        end

        // -- fill rd_cur from the intake skid or directly from the MIG
        //    (OK-mode only); must run BEFORE the drain/consume block below
        //    so that a same-cycle fill-and-immediately-fully-consumed beat
        //    (e.g. a lone bypassed edge beat) ends the cycle correctly
        //    invalidated, not left spuriously valid. --
        if (rd_fill_from_skid) begin
            rd_cur_valid <= 1'b1;
            rd_cur_data  <= rdin_data[rdin_rptr];
            rd_cur_resp  <= rd_beat_resp;
        end else if (rd_fill_from_mig) begin
            rd_cur_valid <= 1'b1;
            rd_cur_data  <= m_rdata;
            rd_cur_resp  <= (m_rlast == rd_last_mig) ? m_rresp : RESP_SLVERR;
        end

        // -- drain to s_r --
        if (s_r_fire) begin
            if (rd_drain_is_ok && rd_cur_done_after_fire) begin
                rd_cur_valid <= 1'b0;
            end
            if (rd_drain_last_beat) begin
                rd_repo_index <= 9'd0;
                rd_mig_index  <= 9'd0;
                rq_valid[rq_drain_ptr] <= 1'b0;
                rq_drain_ptr <= (rq_drain_ptr == RMAX_OUTSTANDING-1) ? {RPTRW{1'b0}} : rq_drain_ptr + 1'b1;
            end else begin
                rd_repo_index <= rd_repo_index + 9'd1;
                if (rd_drain_is_ok && rd_upper) begin
                    rd_mig_index <= rd_mig_index + 9'd1;
                end
            end
        end

        // -- occupancy --
        case ({ar_fire, rd_drain_pop})
            2'b10:   rq_count <= rq_count + 1'b1;
            2'b01:   rq_count <= rq_count - 1'b1;
            default: rq_count <= rq_count;
        endcase
    end
end

// -- intake-skid state machine -------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        rdin_wptr <= 1'b0;
        rdin_rptr <= 1'b0;
        rdin_cnt  <= 2'd0;
    end else begin
        if (rdin_push) begin
            rdin_data[rdin_wptr] <= m_rdata;
            rdin_resp[rdin_wptr] <= m_rresp;
            rdin_last[rdin_wptr] <= m_rlast;
            rdin_wptr <= ~rdin_wptr;
        end
        if (rdin_pop) begin
            rdin_rptr <= ~rdin_rptr;
        end
        case ({rdin_push, rdin_pop})
            2'b10:   rdin_cnt <= rdin_cnt + 2'd1;
            2'b01:   rdin_cnt <= rdin_cnt - 2'd1;
            default: rdin_cnt <= rdin_cnt;
        endcase
    end
end

// The pcie_test MIG contract uses 1-bit IDs; the bridge tracks the wider
// repo IDs locally per descriptor instead.  Downstream B is intentionally
// discarded (see header note); m_bid/m_bresp/m_rid are therefore unused.
// synthesis translate_off
wire _unused = &{1'b0, m_bid, m_rid, 1'b0};
// synthesis translate_on

endmodule

`default_nettype wire
