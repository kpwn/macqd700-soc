// l2c_mshr.v -- L2 miss-status-holding-register table, N entries.
//
// SIZING -- DO NOT SHRINK N (measured 2026-08-19, docs/l2c_perf.md S4).
// Today every master that can reach xbar S0 presents ONE constant AXI ID
// and is ONE-OUTSTANDING (rtl/soc/if_to_axi.v:16,
// rtl/soc/axi_narrow_to_wide.v:72, rtl/soc/fpga_top_debug_host.vh:489),
// and l2c_ctrl.v's Critical-3 same-ID accept
// gate blocks a second live op per ID -- so the live table cannot exceed
// 3 entries and this module's 8 look unused.  THEY ARE NOT DEAD: the core
// this SoC exists to host (m68k-core-040-ooo) is multi-outstanding on both
// its I and D side, which makes all 8 reachable.  An earlier revision of
// this comment recommended shrinking N on the "unused" argument; that is
// WITHDRAWN.
//
// The thing that IS wrong is the accept gate in front of it: a
// multi-outstanding master that reuses one AXI ID gets ZERO benefit from
// these 8 entries -- measured 8.38 cyc/op with unique IDs vs 55.40 cyc/op
// with one ID, a 6.6x cliff (docs/l2c_perf.md S4.2, docs/soc_bus_review.md
// F4).  Fix the gate, do not shrink the table.
//
// Area context: post-route this module was 20,953 LUT / 10,168 FF, 87% of
// all of l2c -- see the act_line note below for where ~59% of that went
// and why depth is now much cheaper to add than it used to be.  The
// follow-up (2026-08-20) moved the replay PAYLOAD -- r_wdata/r_wstrb,
// 32 slots x 144 b = 4,608 FF, 75% of what was left after act_line -- out
// of flops and into a distributed-RAM array (`r_pay`).  The read mux went
// with it: selecting one of 32 slots' 144 b used to elaborate a flat
// 32:1 select; in LUTRAM that select IS the address decode.  MEASURED
// post-synth (Vivado 2025.2, xcku5p-ffvb676-2-i, CPU=m68k, same flags
// both sides):
//
//                 Total LUT   Logic LUT   LUTRAM      FF   BRAM
//   u_mshr before      7,651       6,529    1,122   6,600      0
//   u_mshr after       6,205       4,999    1,206   1,954      0
//                     -1,446      -1,530      +84  -4,646      0
//
// i.e. -70% of the module's flops and -19% of its LUTs for +84 LUTRAM and
// zero block RAMs; whole design 168,161 -> 166,321 LUT and 120,136 ->
// 115,238 FF, post-synth WNS unchanged at -0.669 ns.  Every sibling
// (u_data, u_victim, u_bypass, l2c_ctrl's own logic) is bit-identical
// across the two runs, so the delta is this change and nothing else.
//
// See r_pay's declaration for why distributed RAM and not block RAM, and
// why r_id explicitly did NOT move.
//
// Tracks up to 8 concurrent misses (primary + up to 4 same-line
// secondary merges each). Fill issue and fill return are pipelined:
// every entry has an issued bit, beat counter, error bit and 512-bit
// assembly buffer, and the MSHR index is carried as the AXI read ID.
// ARs can therefore be issued on consecutive clocks and returning beats
// are demultiplexed directly into their owning entry. The downstream
// SoC DDR path preserves AXI IDs and accepted-AR response order.
//
// m_line is a pure FILL-ASSEMBLY buffer: written only by returning R
// beats, read exactly once (S_SCAN snapshots it into act_line).  Every
// install/replay merge happens in act_line -- see the act_line comment
// below for why that matters so much for area.
//
// SECTORED INSTALL (2026-08-19, docs/l2c_perf.md S12).  A miss may now
// land on a line that is ALREADY PARTIALLY VALID -- a fully-strobed 16 B
// write installs its own quadrant with no fill, so a later access to a
// different quadrant of that line allocates an entry on a way that
// already holds real, possibly dirty, data.  Two consequences:
//
//   * `alloc_vsec_pre`/`alloc_dsec_pre` snapshot that way's quadrant
//     valid/dirty masks at allocation.  S_INSTALL's `inst_strb` is then
//     the byte expansion of the INVALID quadrants only, so the fetched
//     (and therefore stale) copy of a resident dirty quadrant never
//     overwrites it, and `inst_dsec` ORs the pre-existing dirty mask so
//     an unrelated dirty quadrant is not silently marked clean.
//   * A requester whose quadrant is ALREADY VALID never reaches this
//     module: l2c_ctrl.v retries such a write instead of merging it (see
//     that file's S_LOOKUP).  That is what keeps `merge_quad` sound --
//     it merges into `act_line`, which holds FETCHED data, correct only
//     for quadrants that were invalid.  Do not relax that gate here.
//
// Installation/replay remains a separate one-entry-at-a-time scheduler
// because l2c_tags/l2c_data intentionally expose one write port. That
// serialization is at the physical resource boundary rather than across
// the long DDR latency. inst_valid/inst_data/inst_strb pulses are the only
// way this module touches l2c_tags/l2c_data. S_INSTALL's pulse writes
// exactly the quadrants this fill had to fetch (`inst_strb_c`, the byte
// expansion of ~alloc_vsec_pre -- a full-line strobe whenever the way was
// wholly invalid, which is the common case); S_SWR's replay-write
// pulses use a QUADRANT-ONLY strobe (Critical-6: a full-line strobe here
// would clobber an unrelated hit-write that landed on this now-valid
// line between install and this replay step, since that hit-write's
// data never touched line_reg).
//
// idq_id/idq_busy: combinational "is this AXI ID tracked anywhere in the
// table" query (primary or any populated replay slot) -- l2c_ctrl.v's
// front door uses this to hold off accepting a new request sharing an ID
// with a still-unanswered MSHR-path op (Critical-3, AXI4 same-ID
// response ordering).
//
// S_DRAIN (entered on reset instead of straight to S_SCAN) sinks R beats
// still arriving from fills that DDR already accepted before the reset.
// It leaves when TWO conditions hold: a 128-cycle settle floor (kept
// verbatim, so nothing downstream shifts in time), AND `ar_debt == 0`.
//
// `ar_debt` is the only piece of state in this module that deliberately
// SURVIVES `rst`: an outstanding-read-burst counter, +1 on every accepted
// fill AR, -1 on every RLAST it consumes.  It exists because the reset
// destroys the evidence -- `m_issued[]` is cleared on the first reset
// cycle, and l2c_ctrl holds this module in reset for the WHOLE 4096-cycle
// tag-clear walk (`.rst(rst || rst_busy)`), so by the time the drain
// starts the table no longer remembers a single thing DDR still owes.
//
// The old exit condition was the cycle count ALONE, i.e. an assumption
// that DDR answers an accepted read inside
//     8 + 4096 (l2c_reset walk) + 128 (this drain) = 4232 cycles = 21 us
// @ 200 MHz.  Past that, a pre-reset burst arrives with `rid = 0` and
// either (a) a post-reset fill has re-allocated entry 0 and the stale
// line is installed as THAT fill's data, or (b) nothing is live, so
// `m_rready` is 0 forever and the shared DDR read channel is dead for
// every master.  Measured: PASS at 3000 and 4000 cycles of stall, at 4300
// the read returns the PREVIOUS burst's data and a quiet variant logs
// 9105 cycles of rvalid && !rready.  A MIG that stalls an accepted read
// past 21 us is not a fault -- it is a refresh storm plus arbitration --
// so the count was a bounded-time assumption about DRAM, and this
// replaces it with the debt the transactions themselves define.
//
// Note `m_arvalid` is suppressed for the whole of S_DRAIN, so no NEW fill
// can be issued while debt is outstanding: a stale beat can never be
// mistaken for a fresh entry's data no matter how late it is.
//
// This is the SAME pattern axi_bridge_stale_sink.v already uses one layer
// down, for the same class of bug and for the same reason -- read that
// file's header.  The difference is whose reset arms it: the bridge
// snapshots ITS debt on ITS s-side reset, which does not help a module
// that was reset without the bridge being told.
//
// REACHABILITY, stated honestly.  In the shipping SoC this is defence in
// depth, not a live bug: l2c is build-gated behind `L2C_ENABLE`, which
// synth/vivado.tcl does not define, and when it is enabled it sits on
// core_rst_bank[4] -- a copy of the same core_rst that arms the async
// bridge's own stale sink, so the one remaining trigger (a board-button
// reset while the MIG stalls an accepted read past 21 us) is covered
// there too.  But `l2c_mshr` must not depend on a module outside itself
// for a safety property its OWN regression suite asserts
// (tb_l2c.cpp test_reset_compose_fill_and_writeback exists precisely to
// say mid-fill reset is safe), and a fixed cycle count that happens to be
// longer than DRAM's worst case is not a proof of anything.  See
// tb_l2c.cpp test_reset_mid_fill_stale_burst_* for the fail-before/
// pass-after pair.
// fill_err suppresses
// the install on a fill SLVERR/DECERR (S_INSTALL/S_SWR skip inst_valid)
// and rsp_resp reports the error to every requester instead of silently
// installing garbage (Minor-16).
//
// Verilog-2005, sync active-high rst.
`default_nettype none
module l2c_mshr #(
    parameter SET_BITS   = 12, parameter TAG_BITS = 14, parameter WAY_BITS = 3,
    parameter ADDR_WIDTH = 32, parameter LINE_BITS = 512, parameter ID_WIDTH = 6,
    parameter N = 8, parameter IDX_BITS = 3, parameter REPLAY_N = 4,
    // RK_BITS = ceil(log2(REPLAY_N)); must be >= 1 (REPLAY_N >= 2).
    parameter RK_BITS = 2
) (
    input  wire                  clk, rst,
    // Lookup (combinational).
    input  wire [SET_BITS-1:0]   lu_set,       input wire [TAG_BITS-1:0] lu_tag,
    output wire                  lu_hit,       output wire [IDX_BITS-1:0] lu_idx,
    output wire [7:0]            lu_onehot, // selected lookup entry, same priority as lu_idx
    // Busy-way query (combinational): ways in `bw_set` already claimed by
    // an in-flight MSHR entry -- l2c.v excludes these from victim-way
    // selection so two misses can never double-claim the same way.
    input  wire [SET_BITS-1:0]   bw_set,       output wire [7:0] bw_mask,
    // SECOND busy-way query port (2026-09-16, FMax: victim precompute).
    // l2c_ctrl now resolves the victim ONE STAGE EARLY, from stage Q, so it
    // needs the busy-way answer for TWO sets in the same cycle: `bw_set` is
    // stage 2's own set (the refresh copy, which keeps the registered victim
    // from going stale while stage 2 stalls) and `bwq_set` is stage Q's set
    // (the copy that is registered into stage 2 on the advancing edge).
    // Exported as a second port for exactly the reason `idq2_*` is: the
    // question is asked at two points in one cycle and a mux in front of a
    // single port would put a LATE select (`pipe_adv_c`) ahead of the
    // deepest cone in the module, which is the defect being removed.
    input  wire [SET_BITS-1:0]   bwq_set,      output wire [7:0] bwq_mask,
    // ID-busy query (combinational, Critical-3).
    input  wire [ID_WIDTH-1:0]   idq_id,       output wire idq_busy,
    // Second id-CAM port (2026-08-20, l2c_ctrl lookup pipelining).  The
    // pipelined lookup needs the same-id-ordering question answered at TWO
    // points in the SAME cycle -- at the front door for a beat being
    // dispatched to the bypass engine (idq_id/idq_busy, unchanged), and at
    // the resolve stage for the beat being resolved.  This port exports the
    // per-ENTRY match vector, not a reduced busy bit, because the resolve
    // stage needs "any entry other than the one I am merging into" as well
    // as "any entry at all", and reducing here would force a second CAM.
    input  wire [ID_WIDTH-1:0]   idq2_id,      output wire [7:0] idq2_vec,
    // Allocate (primary miss, no existing entry).
    input  wire                  alloc_valid,  output wire alloc_ready,
    input  wire [SET_BITS-1:0]   alloc_set,    input wire [TAG_BITS-1:0] alloc_tag,
    input  wire [WAY_BITS-1:0]   alloc_way,    input wire alloc_is_write,
    input  wire [ID_WIDTH-1:0]   alloc_id,     input wire [127:0] alloc_wdata,
    input  wire [15:0]           alloc_wstrb,  input wire [1:0] alloc_qoff,
    input  wire                  alloc_last,   input wire alloc_need_resp,
    // Quadrant valid/dirty masks of the victim way AS IT IS NOW (see header).
    input  wire [3:0]            alloc_vsec_pre, input wire [3:0] alloc_dsec_pre,
    // Secondary merge (into an already-allocated entry, lu_idx).
    input  wire                  merge_valid,  output wire merge_ready,
    input  wire [IDX_BITS-1:0]   merge_idx,    input wire merge_is_write,
    input  wire [ID_WIDTH-1:0]   merge_id,     input wire [127:0] merge_wdata,
    input  wire [15:0]           merge_wstrb,  input wire [1:0] merge_qoff,
    input  wire                  merge_last,   input wire merge_need_resp,
    // Install pulse -> l2c.v forwards to l2c_tags/l2c_data write ports.
    output reg                   inst_valid,
    output reg  [SET_BITS-1:0]   inst_set,      output reg [WAY_BITS-1:0] inst_way,
    output reg  [TAG_BITS-1:0]   inst_tag,
    output reg  [3:0]            inst_vsec,     output reg [3:0] inst_dsec,
    output wire [LINE_BITS-1:0]  inst_data,     output reg [63:0] inst_strb,
    // Replay response -> l2c.v's arbitrated R/B channel mux.
    output reg                   rsp_valid,     input wire rsp_ready,
    output reg                   rsp_is_write,  output reg [ID_WIDTH-1:0] rsp_id,
    output reg  [127:0]          rsp_rdata,     output reg rsp_last,
    output reg  [1:0]            rsp_resp,
    // Exact live table occupancy (popcount of m_v), for VIO/debug.
    output reg  [3:0]            occupancy,
    // AXI-master read sub-port (fill).
    output wire [ID_WIDTH-1:0]   m_arid,        output wire [ADDR_WIDTH-1:0] m_araddr,
    output wire [7:0]            m_arlen,       output wire [2:0] m_arsize,
    output wire [1:0]            m_arburst,     output wire m_arvalid,
    input  wire                  m_arready,
    input  wire [ID_WIDTH-1:0]   m_rid,         input wire [127:0] m_rdata,
    input  wire [1:0]            m_rresp,       input wire m_rlast,
    input  wire                  m_rvalid,      output wire m_rready
);
    /* verilator lint_off UNUSEDSIGNAL */
    wire [ID_WIDTH-1:0] unused_rid = m_rid;
    /* verilator lint_on UNUSEDSIGNAL */
    integer i, j;
    reg                m_v     [0:N-1]; reg [SET_BITS-1:0] m_set [0:N-1];
    reg [TAG_BITS-1:0] m_tag   [0:N-1]; reg [WAY_BITS-1:0] m_way [0:N-1];
    reg                m_issued[0:N-1]; reg m_fill_done [0:N-1];
    reg [1:0]          m_beat  [0:N-1]; reg m_fill_err [0:N-1];
    wire [LINE_BITS-1:0] scan_line;
    // Quadrant masks: m_vsec is the way's valid mask AT ALLOCATION (so
    // ~m_vsec is exactly what this fill must install); m_dsec accumulates
    // the dirty mask this entry will leave behind and is updated in step
    // with each write it applies.
    reg [3:0]          m_vsec  [0:N-1]; reg [3:0] m_dsec [0:N-1];
    reg                p_wr    [0:N-1]; reg [ID_WIDTH-1:0] p_id  [0:N-1];
    reg [127:0]        p_wdata [0:N-1]; reg [15:0]         p_wstrb [0:N-1];
    reg [1:0]          p_qoff  [0:N-1]; reg p_last [0:N-1]; reg p_need [0:N-1];
    reg [RK_BITS:0]    r_cnt   [0:N-1];
    reg                r_wr    [0:N-1][0:REPLAY_N-1]; reg [ID_WIDTH-1:0] r_id [0:N-1][0:REPLAY_N-1];
    reg [1:0]          r_qoff  [0:N-1][0:REPLAY_N-1]; reg r_last [0:N-1][0:REPLAY_N-1];
    reg                r_need  [0:N-1][0:REPLAY_N-1];

    // -- Replay PAYLOAD memory (r_wdata/r_wstrb), see header ---------------
    // {wstrb, wdata} for every replay slot, flattened to ONE 1-D array
    // addressed {entry, slot} so it infers as a real memory.  Written once
    // at merge-enqueue, read once at S_SWR: RAM behaviour, not register
    // behaviour.  Everything else in the r_* family stays in flops --
    // r_id because idq_busy CAMs it combinationally (a RAM cannot be
    // searched associatively), the rest because they steer combinational
    // control and are only 352 FF in total.
    //
    // Depth is N * 2**RK_BITS, not N * REPLAY_N: the address is a plain
    // concatenation, so a non-power-of-two REPLAY_N simply leaves the top
    // slots of each entry unaddressed rather than aliasing into the next
    // entry's slot 0.
    localparam RPAY_AW    = IDX_BITS + RK_BITS;
    localparam RPAY_DEPTH = N * (1 << RK_BITS);
    (* ram_style = "distributed" *)
    reg [143:0] r_pay [0:RPAY_DEPTH-1];
    wire [RPAY_AW-1:0] rpay_wa_c = {merge_idx, r_cnt[merge_idx][RK_BITS-1:0]};

    // -- Lookup / alloc / free / scan / idq via l2c_pri8 --------------------
    reg [7:0] lu_match_c, free_vec_c, issue_vec_c, scan_vec_c, bw_mask_c, bwq_mask_c;
    reg       idq_busy_c;
    reg [7:0] idq2_vec_c;
    // BUSY-WAY MASK, FLATTENED (2026-09-12, FMax).  It used to read
    //
    //     for (i...) if (m_v[i] && (m_set[i]==bw_set)) bw_mask_c[m_way[i]] = 1'b1;
    //
    // inside the big accumulate loop below.  That is a VARIABLE-INDEXED
    // blocking write to a vector, repeated eight times in sequence: each
    // iteration's result is an input to the next, so synthesis has no
    // choice but to build eight decode+merge stages back to back.  Measured
    // on the routed 200 MHz stub build it cost EIGHT LUT levels / 1.88 ns
    // between `req_set` (which drives bw_set) and `bw_mask`, and that
    // 1.88 ns sits at the head of the trunk shared by BOTH of the design's
    // failing cones -- the one ending at the URAM/BRAM array addresses
    // (req_set -> bw_mask -> victim_sel_way -> way_ok_c -> s_lookup_miss_go
    // -> p2_done_c -> q_adv_c -> do_accept_c -> tags_raddr) and the one
    // that leaves the module on s_axi_wready.
    //
    // The same function written as an OR of per-entry one-hot decodes is
    // bit-identical (the original only ever SET bits, never cleared one)
    // but is a balanced tree: the 12-bit set compare and the 3->8 way
    // decode are computed in parallel for all eight entries and then
    // OR-reduced, ~4 levels instead of 8.  Split out of the loop below so
    // it cannot be re-serialised by a later edit to that loop.
    integer bi;
    reg [7:0] bw_acc_c;
    always @(*) begin
        bw_acc_c = 8'b0;
        for (bi = 0; bi < N; bi = bi + 1)
            bw_acc_c = bw_acc_c |
                       ({8{m_v[bi] && (m_set[bi] == bw_set)}} & (8'd1 << m_way[bi]));
        bw_mask_c = bw_acc_c;
    end
    // Identical CAM on the second query set.  Written out rather than
    // generated so that a later edit to one cannot silently re-serialise
    // the other (the same reason the first one was split out of the big
    // accumulate loop below).
    integer bqi;
    reg [7:0] bwq_acc_c;
    always @(*) begin
        bwq_acc_c = 8'b0;
        for (bqi = 0; bqi < N; bqi = bqi + 1)
            bwq_acc_c = bwq_acc_c |
                        ({8{m_v[bqi] && (m_set[bqi] == bwq_set)}} & (8'd1 << m_way[bqi]));
        bwq_mask_c = bwq_acc_c;
    end
    always @(*) begin
        lu_match_c = 8'b0; free_vec_c = 8'b0; issue_vec_c = 8'b0;
        scan_vec_c = 8'b0; occupancy = 4'd0;
        idq_busy_c = 1'b0; idq2_vec_c = 8'b0;
        for (i = 0; i < N; i = i + 1) begin
            lu_match_c[i] = m_v[i] && (m_set[i] == lu_set) && (m_tag[i] == lu_tag);
            free_vec_c[i] = !m_v[i];
            issue_vec_c[i] = m_v[i] && !m_issued[i];
            scan_vec_c[i] = m_v[i] && m_fill_done[i];
            if (m_v[i]) occupancy = occupancy + 4'd1;
            if (m_v[i]) begin
                if (p_id[i] == idq_id) idq_busy_c = 1'b1;
                if (p_id[i] == idq2_id) idq2_vec_c[i] = 1'b1;
                for (j = 0; j < REPLAY_N; j = j + 1) begin
                    if (j < r_cnt[i] && r_id[i][j] == idq_id) idq_busy_c = 1'b1;
                    if (j < r_cnt[i] && r_id[i][j] == idq2_id) idq2_vec_c[i] = 1'b1;
                end
            end
        end
    end
    assign bw_mask  = bw_mask_c;
    assign bwq_mask = bwq_mask_c;
    assign idq_busy = idq_busy_c;
    assign idq2_vec = idq2_vec_c;

    // synthesis translate_off
    // INVARIANT (2026-08-20, l2c_ctrl lookup pipelining): at most ONE valid
    // entry may claim a given (set, way).  Two entries on one way means two
    // fills will install different tags and different data into the same
    // physical line, and the loser's `alloc_vsec_pre` quadrants are marked
    // valid over the winner's data -- silent corruption that survives into
    // a later read.
    //
    // It was true by construction before the lookup was pipelined: the
    // allocator consults `bw_mask` (live per-set busy ways), and the only
    // path that bypasses `bw_mask` is `way_ok_c`'s `any_tm_c` shortcut,
    // which could only fire on a tag array read AFTER the previous
    // request's tag write.  A pipelined lookup reads the array two cycles
    // before it resolves, so `any_tm_c` can now fire on a STALE tag -- and
    // l2c_ctrl's accept-time set/line interlock is the only thing that
    // stops it.  Checked here rather than argued, because the corruption
    // it produces is far away from its cause.
    integer ca, cb;
    always @(posedge clk) begin
        if (!rst) begin
            for (ca = 0; ca < N; ca = ca + 1)
                for (cb = ca + 1; cb < N; cb = cb + 1)
                    if (m_v[ca] && m_v[cb] && (m_set[ca] == m_set[cb]) &&
                        (m_way[ca] == m_way[cb])) begin
                        $display("L2C_MSHR ASSERT: entries %0d and %0d both claim set %0d way %0d (tags %0h / %0h)",
                                 ca, cb, m_set[ca], m_way[ca], m_tag[ca], m_tag[cb]);
                        $fatal(1);
                    end
        end
    end
    // synthesis translate_on
    wire                lu_hit_c, free_any_c, issue_any_c, scan_any_c;
    wire [IDX_BITS-1:0] lu_idx_c, free_idx_c, issue_idx_rot_c, scan_idx_rot_c;
    l2c_pri8 u_pri_lu   (.req(lu_match_c), .hit(lu_hit_c),   .idx(lu_idx_c), .onehot(lu_onehot));
    l2c_pri8 u_pri_free (.req(free_vec_c), .hit(free_any_c), .idx(free_idx_c), .onehot());
    // Fill issue and completion service are independent round-robin walks.
    reg [IDX_BITS-1:0] issue_rr_ptr;
    wire [7:0] issue_vec_rot_c = (issue_vec_c >> issue_rr_ptr) |
                                  (issue_vec_c << (4'd8 - issue_rr_ptr));
    wire [IDX_BITS-1:0] issue_idx_c = issue_idx_rot_c + issue_rr_ptr;
    l2c_pri8 u_pri_issue (.req(issue_vec_rot_c), .hit(issue_any_c), .idx(issue_idx_rot_c), .onehot());
    // Completion SCAN is round-robin, not fixed lowest-index: a fixed-priority scan
    // would let a continuously-refreshed low index starve higher indices
    // forever.  Rotate the valid mask by rr_ptr, encode, un-rotate;
    // rr_ptr advances past whichever entry SCAN just picked.
    reg [IDX_BITS-1:0] rr_ptr;
    wire [7:0] scan_vec_rot_c = (scan_vec_c >> rr_ptr) | (scan_vec_c << (4'd8 - rr_ptr));
    wire [IDX_BITS-1:0] scan_idx_c = scan_idx_rot_c + rr_ptr;
    l2c_pri8 u_pri_scan (.req(scan_vec_rot_c), .hit(scan_any_c), .idx(scan_idx_rot_c), .onehot());
    assign lu_hit      = lu_hit_c;
    assign lu_idx      = lu_idx_c;
    assign alloc_ready = free_any_c;
    // S_FREE-vs-merge window.  The S_SNEXT guard covers the cycle the FSM
    // DECIDES to free, but not the S_FREE cycle itself: during S_FREE m_v[act]
    // is still 1, so lu_hit still asserts and l2c_ctrl can enqueue a merge into
    // the entry on the very cycle it is invalidated.  That merge is orphaned --
    // the write never lands and NO B RESPONSE IS EVER EMITTED, hanging the
    // requester until the xbar watchdog SLVERRs it.  Reachable only when a fill
    // returns SLVERR/DECERR (measured 0 occurrences in 33,977 otherwise-normal
    // S_FREE cycles), which is exactly why it had never been seen.
    assign merge_ready = (r_cnt[merge_idx] < REPLAY_N) &&
                         !((st == S_FREE) && (merge_idx == act));

    // -- Pipelined fill issue ------------------------------------------------
    // The live combinational candidate gives one-AR-per-cycle throughput.
    // If downstream backpressures, ar_hold locks that candidate so AXI
    // VALID/payload stability does not depend on the table changing.
    reg ar_hold_valid;
    reg [IDX_BITS-1:0] ar_hold_idx;

    // -- Completed-entry install/replay scheduler ---------------------------
    localparam S_SCAN=4'd0, S_INSTALL=4'd1, S_PRSP=4'd2,
               S_SNEXT=4'd3, S_SWR=4'd4, S_SRSP=4'd5,
               S_FREE=4'd6, S_DRAIN=4'd7;
    reg [3:0]           st;
    reg [IDX_BITS-1:0]  act;
    reg [RK_BITS:0]     rk;
    reg [6:0]           drain_ctr;
    // Outstanding fill-burst debt -- see the S_DRAIN note in the header.
    // Width: one AR per MSHR entry can be outstanding at a time (an entry
    // issues exactly once, on m_issued 0->1, and is freed only after
    // m_fill_done), so the true maximum is N.  IDX_BITS+1 bits holds N.
    //
    // DELIBERATELY NOT RESET.  That is the entire point: it is the record
    // of what DDR still owes us, and `rst` is exactly when every other
    // copy of that record is thrown away.  Power-up value is the flop's
    // INIT (0) -- Vivado's default for a reg with no initialiser, and the
    // same under --x-initial fast, which is how every Verilator build in
    // this repo is configured.  At power-up nothing has been issued, so 0 is
    // physically correct value.
    reg [IDX_BITS:0]    ar_debt;
    wire ar_fire_c    = m_arvalid && m_arready;
    wire rlast_fire_c = m_rvalid && m_rready && m_rlast;
    function [127:0] merge_quad;
        input [127:0] old_q, wdata; input [15:0] wstrb; integer bb;
        begin
            for (bb = 0; bb < 16; bb = bb + 1)
                merge_quad[bb*8 +: 8] = wstrb[bb] ? wdata[bb*8 +: 8] : old_q[bb*8 +: 8];
        end
    endfunction
    // rk truncated to a safe 2b slot index; rk==REPLAY_N only occurs in
    // S_SNEXT's "loop done" check, never used to index r_*[act][..] below.
    wire [RK_BITS-1:0] rk_i = rk[RK_BITS-1:0];
    // ── Active-entry read muxes, hoisted (area) ─────────────────────────
    // `act` only moves in S_SCAN and `rk` only in the replay walk, so every
    // consumer below reads the SAME entry/slot in the same cycle.  Written
    // inline (m_line[act][p_qoff[act]*128 +: 128], r_wdata[act][rk_i], ...)
    // each use site elaborated its own FLAT dynamic mux -- five separate
    // 32:1-over-128b selects on m_line alone.  Hoisting turns that into one
    // 8:1-over-512b stage plus cheap 4:1 quadrant picks, which is where the
    // bulk of l2c_mshr's LUT count lives (docs/l2c_perf.md).  Pure refactor:
    // identical function, no state, no new timing arc (the 8:1 mux was
    // already in every one of those paths).
    // ── The ACTIVE-LINE register (2026-08-19, docs/l2c_perf.md) ─────────
    // `m_line` used to be written from THREE sources: fill beats, the
    // primary write's merge at S_INSTALL, and each replay write's merge at
    // S_SWR.  A 4096-bit register array with a 3-source write mux costs a
    // ~3-LUT cone PER BIT -- post-route evidence: the path into
    // m_line_reg[1][127]/D runs through m_line[1][127]_i_3 -> _i_4 -> _i_1
    // (build/vivado/reports/timing_synth.rpt), i.e. ~12.3 K LUT for the
    // array's write network alone, roughly 59% of this module's 20,953 LUT.
    //
    // Only ONE entry is ever installed/replayed at a time (`act`, and the
    // walk cannot start until that entry's fill is complete, so no beat can
    // land in it afterwards).  So the merges do not need random access into
    // the array at all: snapshot the chosen line into a single 512-bit
    // register when SCAN picks it, do every merge there, and leave `m_line`
    // as a pure fill-assembly buffer written ONLY by returning R beats --
    // whose D input is then m_rdata directly, with no data mux at all.
    //
    // Same behaviour, one write network instead of three.  It also turns
    // m_line into a write-once/read-once structure, which is what makes the
    // follow-up (m_line -> BRAM) straightforward.
    reg [LINE_BITS-1:0] act_line;
    wire [LINE_BITS-1:0] line_act_c   = act_line;
    // The install payload is already registered here. S_INSTALL/S_SWR update
    // act_line with the same merge on the edge that raises inst_valid; neither
    // successor state changes act_line while that install pulse is consumed.
    // S_SCAN can replace it only after inst_valid has gone low. A separate
    // 512-bit output register therefore duplicates both data and write control.
    // Payload when inst_valid=0 is unspecified; the valid/strobe/tag timing is
    // unchanged. tb_l2c retains the old registered payload as an independent
    // simulation reference and compares every valid install against it.
    assign inst_data = act_line;
    wire [1:0]           p_qoff_act_c = p_qoff[act];
    wire [1:0]           r_qoff_act_c = r_qoff[act][rk_i];
    // Asynchronous read out of the replay-payload RAM.  Distributed RAM
    // (LUTRAM), not block RAM, and for two independent reasons:
    //
    //   * AREA.  32 x 144 b is one LUT6 per bit as RAM32M/RAM32X1D, and
    //     the read mux IS the LUTRAM address decode -- the 32:1-over-144b
    //     select the register array needed DISAPPEARS rather than moving
    //     somewhere else.  A RAMB18 holds 18 Kb; 4.6 Kb of payload would
    //     burn whole block RAMs at a few percent fill and still need a
    //     wide output mux or a pipeline stage in front of S_SWR.
    //
    //   * RISK.  A combinational read leaves the replay walk cycle-for-
    //     cycle identical to what the register array did: no pipeline
    //     stage, no prefetch, no extra state, nothing to get wrong.  A
    //     replay that merged the wrong cycle's payload would corrupt a
    //     write the requester has ALREADY been told succeeded -- silent
    //     data corruption, not a performance bug -- so "no timing change
    //     at all" is worth more here than the block RAM would be.
    //
    // For the record, ONE registered read stage would also be functionally
    // safe: S_SWR is only ever entered from S_SNEXT, and both `act` and
    // `rk` are stable for that whole S_SNEXT cycle, so the address is
    // always presented at least a cycle early.  TWO stages are not (tb
    // tb_l2c.cpp's replay_* scenarios were checked RED against exactly
    // that).  Nothing else in the FSM advertises that one-cycle margin, so
    // do not spend it without re-running those scenarios.
    wire [RPAY_AW-1:0]   rpay_ra_c     = {act, rk_i};
    wire [143:0]         rpay_q_c      = r_pay[rpay_ra_c];
    wire [127:0]         r_wdata_act_c = rpay_q_c[127:0];
    wire [15:0]          r_wstrb_act_c = rpay_q_c[143:128];
    wire [127:0] p_wdata_act_c = p_wdata[act];
    wire [15:0]  p_wstrb_act_c = p_wstrb[act];
    // Merge at a fixed destination quadrant below. Selecting an old quadrant,
    // merging, then dynamically writing it back sends qoff through both a
    // 512-to-128 read mux and a 128-to-512 write network. Each fixed lane needs
    // only its local old bytes, shared new bytes, and one quadrant enable.
    // There is no extra register, state, or install/replay cycle.
    // Primary and replay merges are mutually exclusive. Select their narrow
    // input once, before distributing it to the four fixed destination lanes,
    // rather than giving every lane its own primary/replay update network.
    wire merge_replay_c = (st == S_SWR);
    wire merge_line_c = !m_fill_err[act] &&
                        (merge_replay_c || ((st == S_INSTALL) && p_wr[act]));
    wire [127:0] merge_wdata_c = merge_replay_c ? r_wdata_act_c : p_wdata_act_c;
    wire [15:0] merge_wstrb_c = merge_replay_c ? r_wstrb_act_c : p_wstrb_act_c;
    wire [1:0] merge_qoff_c = merge_replay_c ? r_qoff_act_c : p_qoff_act_c;
    integer merge_q;
    // Quadrant-only strobe for S_SWR (Critical-6): same shift pattern as
    // l2c_ctrl.v's own hit-path dw_strb.
    wire [63:0] swr_strb_c = {48'b0, r_wstrb_act_c} << (r_qoff_act_c * 16);
    // S_INSTALL writes only the quadrants this fill actually had to fetch
    // -- the ones that were INVALID when the entry was allocated.  A
    // resident (and possibly dirty) quadrant of the same line keeps the
    // data already in the array.
    wire [3:0]  fetch_mask_c = ~m_vsec[act];
    wire [63:0] inst_strb_c  = {{16{fetch_mask_c[3]}}, {16{fetch_mask_c[2]}},
                                {16{fetch_mask_c[1]}}, {16{fetch_mask_c[0]}}};
    wire [3:0]  p_dsec_c     = m_dsec[act] | (p_wr[act] ? (4'b0001 << p_qoff_act_c) : 4'b0000);
    wire [3:0]  r_dsec_c     = m_dsec[act] | (4'b0001 << r_qoff_act_c);
    wire [IDX_BITS-1:0] ar_idx_c = ar_hold_valid ? ar_hold_idx : issue_idx_c;
    assign m_arid    = {{(ID_WIDTH-IDX_BITS){1'b0}}, ar_idx_c};
    assign m_araddr  = {m_tag[ar_idx_c], m_set[ar_idx_c], {(ADDR_WIDTH-TAG_BITS-SET_BITS){1'b0}}};
    assign m_arlen   = 8'd3;
    assign m_arsize  = 3'd4;
    assign m_arburst = 2'd1;
    assign m_arvalid = (st != S_DRAIN) && (ar_hold_valid || issue_any_c);
    wire [IDX_BITS-1:0] r_idx_c = m_rid[IDX_BITS-1:0];
    assign m_rready  = (st == S_DRAIN) ||
                       (m_v[r_idx_c] && m_issued[r_idx_c] && !m_fill_done[r_idx_c]);
    // Four independently enabled fill banks.  A dynamic part-select write
    // on a 512-bit array can infer a read/modify/write mux for every bit.
    // Each bank has just one write address and one asynchronous read address;
    // the scan/install timing, eight entries and reset-drain rules are unchanged.
    genvar fill_quad;
    generate for (fill_quad = 0; fill_quad < LINE_BITS/128; fill_quad = fill_quad + 1) begin : g_fill_bank
        (* ram_style = "distributed" *) reg [127:0] data [0:N-1];
        always @(posedge clk) begin
            if (!rst && m_rvalid && m_rready && st != S_DRAIN &&
                m_beat[r_idx_c] == fill_quad)
                data[r_idx_c] <= m_rdata;
        end
        assign scan_line[fill_quad*128 +: 128] = data[scan_idx_c];
    end endgenerate
    // Fill-burst debt.  Its own always block, with NO `rst` arm, so the
    // "survives reset" property is visible rather than buried in an else.
    // The AR arm is unconditional on purpose: on the cycle `rst` first
    // asserts, `st` still holds its pre-reset value, so `m_arvalid` may
    // still be high and an AR may still be ACCEPTED by DDR in that very
    // cycle -- the reset arm below would then clear `m_issued` and lose
    // it.  Counting the handshake itself cannot miss that burst.  Nor can
    // it double-count it: `issue_vec_c` requires `!m_issued`, so the entry
    // handshaking here is never one of the already-issued ones.
    always @(posedge clk) begin
        if (ar_fire_c && !rlast_fire_c)      ar_debt <= ar_debt + 1'b1;
        else if (!ar_fire_c && rlast_fire_c) ar_debt <= ar_debt - 1'b1;
    end

    // synthesis translate_off
    always @(posedge clk) begin
        if (rlast_fire_c && !ar_fire_c && (ar_debt == {(IDX_BITS+1){1'b0}})) begin
            $display("[%0t] l2c_mshr: FATAL ar_debt underflow -- RLAST consumed with no outstanding fill burst", $time);
            $finish;
        end
        if (ar_fire_c && !rlast_fire_c && (ar_debt == N[IDX_BITS:0])) begin
            $display("[%0t] l2c_mshr: FATAL ar_debt overflow -- more than N=%0d fill bursts outstanding", $time, N);
            $finish;
        end
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < N; i = i + 1) begin
                m_v[i] <= 1'b0; m_issued[i] <= 1'b0;
                m_fill_done[i] <= 1'b0; m_beat[i] <= 2'd0;
                m_fill_err[i] <= 1'b0;
                r_cnt[i] <= {(RK_BITS+1){1'b0}};
            end
            st <= S_DRAIN; act <= {IDX_BITS{1'b0}};
            rk <= {(RK_BITS+1){1'b0}}; inst_valid <= 1'b0; rsp_valid <= 1'b0;
            rr_ptr <= {IDX_BITS{1'b0}}; issue_rr_ptr <= {IDX_BITS{1'b0}};
            ar_hold_valid <= 1'b0; ar_hold_idx <= {IDX_BITS{1'b0}};
            drain_ctr <= 7'd0;
        end else begin
            inst_valid <= 1'b0;
            if (alloc_valid && alloc_ready) begin
                m_v[free_idx_c]     <= 1'b1;
                m_issued[free_idx_c] <= 1'b0;
                m_fill_done[free_idx_c] <= 1'b0;
                m_beat[free_idx_c]  <= 2'd0;
                m_fill_err[free_idx_c] <= 1'b0;
                m_set[free_idx_c]   <= alloc_set;
                m_tag[free_idx_c]   <= alloc_tag;
                m_way[free_idx_c]   <= alloc_way;
                p_wr[free_idx_c]    <= alloc_is_write;
                p_id[free_idx_c]    <= alloc_id;
                p_wdata[free_idx_c] <= alloc_wdata;
                p_wstrb[free_idx_c] <= alloc_wstrb;
                p_qoff[free_idx_c]  <= alloc_qoff;
                p_last[free_idx_c]  <= alloc_last;
                p_need[free_idx_c]  <= alloc_need_resp;
                m_vsec[free_idx_c]  <= alloc_vsec_pre;
                m_dsec[free_idx_c]  <= alloc_dsec_pre;
                r_cnt[free_idx_c]   <= {(RK_BITS+1){1'b0}};
            end
            if (merge_valid && merge_ready) begin
                r_wr[merge_idx][r_cnt[merge_idx][RK_BITS-1:0]]    <= merge_is_write;
                r_id[merge_idx][r_cnt[merge_idx][RK_BITS-1:0]]    <= merge_id;
                r_qoff[merge_idx][r_cnt[merge_idx][RK_BITS-1:0]]  <= merge_qoff;
                r_last[merge_idx][r_cnt[merge_idx][RK_BITS-1:0]]  <= merge_last;
                r_need[merge_idx][r_cnt[merge_idx][RK_BITS-1:0]]  <= merge_need_resp;
                // Single write port of the replay-payload RAM.  The array
                // body is never reset -- r_cnt alone decides which slots
                // hold live data, and a reset sweep over 32 x 144 b would
                // force the whole thing back into flops.
                r_pay[rpay_wa_c] <= {merge_wstrb, merge_wdata};
                r_cnt[merge_idx] <= r_cnt[merge_idx] + 1'b1;
            end

            // One new fill AR may launch every cycle. A candidate first
            // presented into backpressure is held until its handshake.
            if (m_arvalid && !m_arready && !ar_hold_valid) begin
                ar_hold_valid <= 1'b1;
                ar_hold_idx   <= issue_idx_c;
            end else if (m_arvalid && m_arready) begin
                m_issued[ar_idx_c] <= 1'b1;
                m_beat[ar_idx_c] <= 2'd0;
                m_fill_err[ar_idx_c] <= 1'b0;
                issue_rr_ptr <= ar_idx_c + 1'b1;
                ar_hold_valid <= 1'b0;
            end

            // AXI read IDs are the MSHR indices, so fills may return while
            // another completed entry is installing/replaying.
            if (m_rvalid && m_rready && st != S_DRAIN) begin
                if (m_rresp != 2'b00) m_fill_err[r_idx_c] <= 1'b1;
                if (m_rlast) begin
                    m_fill_done[r_idx_c] <= 1'b1;
                    if (m_beat[r_idx_c] != 2'd3) m_fill_err[r_idx_c] <= 1'b1;
                end else begin
                    m_beat[r_idx_c] <= m_beat[r_idx_c] + 2'd1;
                end
            end
            case (st)
                S_DRAIN: begin
                    // 128-cycle settle floor, kept exactly as it was so
                    // nothing downstream moves in time -- then the real
                    // gate: leave only once DDR owes us nothing.  With no
                    // debt this is bit-for-bit the old behaviour; with
                    // debt outstanding we keep m_rready high (see
                    // m_arvalid/m_rready above) and sink until it clears,
                    // however long DDR takes.  No timeout: a fill that
                    // never returns stops the machine where it broke
                    // instead of silently consuming the next fill's data.
                    if (drain_ctr == 7'd127) begin
                        if (ar_debt == {(IDX_BITS+1){1'b0}}) st <= S_SCAN;
                    end else begin
                        drain_ctr <= drain_ctr + 1'b1;
                    end
                end
                S_SCAN: if (scan_any_c) begin
                    act <= scan_idx_c; st <= S_INSTALL;
                    // The one and only read of m_line: snapshot the picked
                    // entry's assembled line into act_line for the whole
                    // install/replay walk.  Safe because scan_vec_c requires
                    // m_fill_done, so no further beat can land in this entry.
                    act_line <= scan_line;
                    rr_ptr <= scan_idx_c + 1'b1; // next SCAN starts past this entry
                end
                S_INSTALL: begin
                    // Whole freshly-fetched line, full strobe (line was
                    // invalid before -- no clobber risk, unlike S_SWR);
                    // primary write's quadrant is overridden with the
                    // merged value.  Skipped on fill_err (Minor-16).
                    if (!m_fill_err[act]) begin
                        inst_valid     <= 1'b1;
                        inst_set       <= m_set[act];
                        inst_way       <= m_way[act];
                        inst_tag       <= m_tag[act];
                        // The fill covered the whole line, so every
                        // quadrant is valid afterwards; dirty is whatever
                        // was already dirty plus this write's quadrant.
                        inst_vsec      <= 4'b1111;
                        inst_dsec      <= p_dsec_c;
                        m_dsec[act]    <= p_dsec_c;
                        inst_strb      <= inst_strb_c;
                    end
                    rk <= {(RK_BITS+1){1'b0}};
                    st <= S_PRSP;
                end
                S_PRSP: if (p_need[act]) begin
                    rsp_valid    <= 1'b1;
                    rsp_is_write <= p_wr[act];
                    rsp_id       <= p_id[act];
                    rsp_rdata    <= line_act_c[p_qoff_act_c*128 +: 128];
                    rsp_last     <= p_last[act];
                    rsp_resp     <= m_fill_err[act] ? 2'b10 : 2'b00;
                    if (rsp_valid && rsp_ready) begin rsp_valid <= 1'b0; st <= S_SNEXT; end
                end else st <= S_SNEXT;
                S_SNEXT: begin
                    // Free-vs-merge race guard: a secondary merge landing
                    // on THIS entry the exact cycle its replay loop
                    // finishes must not be orphaned by S_FREE clearing
                    // m_v[act] first; merge_valid/ready/idx are live
                    // (combinational) this cycle so the check is timed
                    // correctly (r_cnt itself wouldn't yet reflect it).
                    if (rk == r_cnt[act]) begin
                        if (!(merge_valid && merge_ready && (merge_idx == act)))
                            st <= S_FREE;
                        // else stay: next cycle sees rk < r_cnt[act].
                    end else if (r_wr[act][rk_i]) st <= S_SWR;
                    else st <= S_SRSP;
                end
                S_SWR: begin
                    // Quadrant-only strobe (Critical-6, see header).
                    // Skipped entirely on fill_err.
                    if (!m_fill_err[act]) begin
                        inst_valid     <= 1'b1;
                        inst_set       <= m_set[act];
                        inst_way       <= m_way[act];
                        inst_tag       <= m_tag[act];
                        inst_vsec      <= 4'b1111;
                        inst_dsec      <= r_dsec_c;
                        m_dsec[act]    <= r_dsec_c;
                        inst_strb      <= swr_strb_c;
                    end
                    st <= r_need[act][rk_i] ? S_SRSP : S_SNEXT;
                    if (!r_need[act][rk_i]) rk <= rk + 1'b1;
                end
                S_SRSP: begin
                    rsp_valid    <= 1'b1;
                    rsp_is_write <= r_wr[act][rk_i];
                    rsp_id       <= r_id[act][rk_i];
                    rsp_rdata    <= line_act_c[r_qoff_act_c*128 +: 128];
                    rsp_last     <= r_last[act][rk_i];
                    rsp_resp     <= m_fill_err[act] ? 2'b10 : 2'b00;
                    if (rsp_valid && rsp_ready) begin
                        rsp_valid <= 1'b0; rk <= rk + 1'b1; st <= S_SNEXT;
                    end
                end
                S_FREE: begin
                    m_v[act] <= 1'b0;
                    m_issued[act] <= 1'b0;
                    m_fill_done[act] <= 1'b0;
                    st <= S_SCAN;
                end
                default: st <= S_SCAN;
            endcase
            // No new stage: these are exactly the former S_INSTALL/S_SWR
            // writes. S_SCAN's snapshot is mutually exclusive, and reset,
            // fill-error suppression and untouched-byte retention are intact.
            if (merge_line_c) begin
                for (merge_q = 0; merge_q < 4; merge_q = merge_q + 1) begin
                    if (merge_qoff_c == merge_q[1:0])
                        act_line[merge_q*128 +: 128] <= merge_quad(
                            line_act_c[merge_q*128 +: 128], merge_wdata_c, merge_wstrb_c);
                end
            end
        end
    end

    // synthesis translate_off
    // INVARIANT (sim-only): the payload S_SWR merges into act_line must be
    // exactly the {wstrb, wdata} enqueued by the merge that created slot
    // `rk` of entry `act`.
    //
    // r_pay replaced a pair of [entry][slot] REGISTER arrays with one 1-D
    // memory addressed {act, rk_i}.  Two failure modes that change would
    // introduce are both silent at the AXI boundary and both corrupt a
    // write the requester has already been told succeeded: an address
    // flattening slip (wrong concat order, wrong slot width, aliasing
    // across entries), and -- should anyone later move this array to block
    // RAM -- a read-latency slip that lets S_SWR observe a neighbouring
    // cycle's contents.  The shadow below keeps the ORIGINAL 2-D indexing,
    // written straight off the merge_* inputs, and re-derives the expected
    // payload independently, so either failure stops the run loudly.
    //
    // Sim-only: it is a full duplicate of the array and must never reach
    // synthesis.
    reg [143:0] r_pay_shadow [0:N-1][0:REPLAY_N-1];
    always @(posedge clk) begin
        if (!rst) begin
            if (merge_valid && merge_ready)
                r_pay_shadow[merge_idx][r_cnt[merge_idx][RK_BITS-1:0]]
                    <= {merge_wstrb, merge_wdata};
            if ((st == S_SWR) &&
                ({r_wstrb_act_c, r_wdata_act_c} !== r_pay_shadow[act][rk_i])) begin
                $display("L2C_MSHR ASSERT: replay payload mismatch entry=%0d slot=%0d ram=%h shadow=%h",
                         act, rk_i, {r_wstrb_act_c, r_wdata_act_c},
                         r_pay_shadow[act][rk_i]);
                $fatal(1);
            end
        end
    end
    // synthesis translate_on
endmodule
`default_nettype wire
