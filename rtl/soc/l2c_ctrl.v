// l2c_ctrl.v -- L2 cache-proper controller: front door, hit datapath,
// miss dispatch, PLRU/busy-way victim selection, reset-walk gating.
//
// Instantiates l2c_reset + l2c_tags + l2c_data + l2c_mshr + l2c_victim_sel
// (victim writeback buffer + bypass engine live in l2c.v, their AXI-
// master sub-ports arbitrate against this module's mshr-fill sub-port
// there).
//
// ============================ THE LOOKUP PIPELINE ======================
// 2026-08-20.  The lookup is a THREE-STAGE PIPELINE holding three
// different requests, retiring ONE BEAT PER CYCLE.  It was a three-cycle
// LOOP (`st`: S_IDLE -> S_WAIT -> S_LOOKUP -> S_IDLE) holding one request
// at a time, i.e. one beat per THREE cycles -- 533 MB/s on a 128 b x
// 100 MHz fabric whose ceiling is 1.6 GB/s.
//
//   S0  accept   front door selects a beat, loads stage A (a_*) and the
//                array address register (ra_q)
//   A   address  a_*'s set stands on the array read address pins
//   S1  read     l2c_tags output lands (1 cyc); it is registered again
//                here (tq_*) so it arrives with l2c_data's 2-cycle output
//   S2  resolve  hit/miss classify, tag+data+PLRU write, response
//
// 2026-09-15: stage A is new, and the reason it exists is TIMING, not
// function -- see "ARRAY ADDRESS REGISTER" at tags_raddr below for the
// measured path it removes.  A hit is now 4 cycles of LATENCY, still ONE
// cycle of occupancy: the pipeline retires one beat per cycle exactly as
// before, and every measured cycles/op row below is unchanged.  The
// pipeline also became a RIGID shift (every stage moves together or none
// does) rather than an elastic one; accept_slot_c argues why that is what
// keeps the array clock enable a single net and the accept cone's
// comparator count at two rather than three.
//
// The resolve body (`req_*`) is byte-for-byte the old S_LOOKUP body.
// What is new is everything that keeps three in-flight requests from
// lying to each other:
//
//  * ARRAY READ-AFTER-WRITE.  S2 writes the arrays on the edge it
//    resolves, and that write is invisible to whatever occupied stages 1
//    and 2 at the time.  Two stages, two blind cycles: `set_haz_c` checks
//    a new request against BOTH occupied stages at accept, which is exact
//    rather than conservative.  (⚠ This used to say "both arrays are
//    READ-FIRST"; that is a claim about silicon that cannot be supported
//    and is not what makes this safe -- see the READ-FIRST note at
//    set_haz_c below, and l2c_data.v's header.)  It is
//    NOT a plain same-set interlock -- a 64 B burst is four beats of one
//    line and one set, so that would have left burst reads at 3
//    cycles/beat.  Same-LINE READ PAIRS overlap; anything else sharing a
//    set does not.  See set_haz_c for why each excluded case corrupts.
//  * SAME-ID RESPONSE ORDERING.  Critical-3's front-door hold-off could
//    not survive: it is exactly beats 2..N of a missing burst, which want
//    to MERGE into their leader's MSHR entry (ordered by construction,
//    that entry's replay list is a FIFO).  The check moved to S2, where
//    the outcome is known, using l2c_mshr's new per-entry id-match vector
//    (`idq2_vec`) -- see ord_now_block_c / ord_merge_block_c.  Worth
//    3.6x on stream_read_64B at DDR-40 and 7.2x at DDR-200 on its own.
//  * CRITICAL-7 SKEW.  Unchanged in window (still the 2-deep inst_hist:
//    the read-to-resolve distance is still two cycles -- and STILL two
//    after 2026-09-15's extra stage, because that stage sits BEFORE the
//    array, so the cycle was added ahead of the read and not between the
//    read and the resolve; see the rr block), changed in RESPONSE.  The old S_LOOKUP -> S_WAIT bounce has no meaning once
//    S_WAIT holds a different request, so S2 stays put and RE-READS its
//    own set (`rr_start_c`).  Flush was rejected -- the front door's beat
//    counters have already advanced past this beat.  Forwarding was
//    rejected -- it would put a {set,way,tag,vsec,dsec} compare and a
//    512 b strobe-merge mux into the resolve cone, which is the design's
//    tightest.
//  * STALLS hold the pipeline's array view via `ce_c`, a clock enable on
//    tq_*, on l2c_tags's own output register, and on BOTH of l2c_data's.
//    Until 2026-09-15 only the SECOND register was enabled: the first
//    free-ran, and was held instead by re-pointing a COMBINATIONAL read
//    address back at the stalled stage's own set, which cost nothing.
//    With the address in a register that re-issue would cost a cycle, so
//    the whole read pipeline is clock-enabled instead -- and because the
//    stages now shift rigidly, one enable is correct for all of them, so
//    l2c_data still sees exactly one control net reaching its 64 URAMs.
//
// The FRONT DOOR had to be deepened in the same change (docs/
// soc_v2_interconnect_shape.md S9a #4).  7d49e1f measured the
// single-entry door as free, correctly: `ar_have` cleared on the edge
// that took a burst's last beat, giving the next header a two-cycle
// window against a three-cycle loop.  At one beat per cycle that slack is
// zero and a 1-beat-burst stream ran at 2.01 cyc/op.  One-entry pre-latch
// per header channel, `s_arready = !ar_pend_v` -- still a plain register
// bit, deliberately not `!ar_have || <ends this cycle>`, which would have
// pulled the accept cone onto the AXI ready path.
//
// Measured, cycles/op (DDR-40; DDR-200 identical on every hit row).
//
// RE-MEASURED 2026-09-15, A/B against the same tb, after the array-address
// register stage (L2C_PERF=1, all 58 rows x both DDR models).  Every row
// where this module is the limiter is UNCHANGED to the printed precision:
//
//   hit_read 8 out              1.01 -> 1.01   (1587.6 -> 1584.5 MB/s)
//   hit_read_single_id 8 out    1.01 -> 1.01
//   hit_write 8 out             2.01 -> 2.01
//   hit_read_64B_burst          4.02 -> 4.02
//   stream_read_64B (DDR-40)   17.53 -> 17.53
//   door_read_1beat_hit         1.01 -> 1.01
//   door_rw_mixed_hit           1.01 -> 1.01
//   every bypass_* row, cacheable_read_512B, miss_read 16 out: identical
//
// The 28 rows that DID move all moved by the ONE extra cycle of latency,
// divided by how many ops are in flight -- which is the signature of a
// latency change and not a throughput change:
//
//   hit_read  1 outstanding     5.00 -> 6.00   (+1.00 = +1 cy / 1 op)
//   hit_read  2 outstanding     2.50 -> 3.00   (+0.50 = +1 cy / 2 ops)
//   miss_read 1 / 2 / 4 / 8 out  +0.99 / +0.98 / +0.48 / +0.23
//   miss_read 16 out            +0.00           (fully amortised)
//   stream_write_64B            5.06 -> 5.08
//
// and the reported hit latency moves by exactly one cycle: the tb's
// `hdr->data` for a read hit 4.0 -> 5.0 cycles, `perf smoke: avg hit
// round-trip latency` 5.16 -> 6.20.  A HIT IS NOW 4 CYCLES OF LATENCY AND
// STILL ONE CYCLE OF OCCUPANCY.
//
// The fetch door was re-measured too, and needed a tb fix first: its
// warm-up drain could open the measurement window on top of ~250 cycles of
// warm-up burst still being dispatched (tb_l2c.cpp fetch_door_run).  With
// that fixed, the fetch door reads 1.992 / 3.977 / 1.992 cyc/burst -- the
// 2-and-4-quadrant-per-burst floors -- and is BYTE-IDENTICAL on the RTL
// before and after this change.
//   hit_read 8 out         3.00 -> 1.01   533 -> 1588 MB/s
//   hit_read_64B_burst    12.01 -> 4.02   533 -> 1594 MB/s
//   stream_read_64B       63.84 -> 17.53  (223.84 -> 31.08 at DDR-200)
//   stream_write_64B       7.03 -> 5.06   910 -> 1264 MB/s
//   hit_write 8 out        3.00 -> 2.01   -- the residual is the TB
//                          master's AW/W coupling, not this module: with
//                          an AW-runahead master the same stream reaches
//                          1.016 (tb_l2c.cpp front_door_throughput_floor
//                          case 3).  l2c cannot take a W beat until
//                          aw_have is registered.
// =======================================================================
//
// The hit response leaves through a 1-deep skid register (hit_rsp_*), so
// a hit costs 3 cycles of LATENCY and 1 cycle of occupancy (there is no
// separate response-hold state).  AXI bursts decomposed 1 INCR beat/cyc.
//
// tw_en/dw_en/pw_en are pure COMBINATIONAL functions of the live S2
// decision vs. mshr_inst_valid the SAME cycle the physical write happens
// -- a hit-write only "goes" (s_lookup_hit_wr_go) when MSHR doesn't want
// the port this cycle; else S2 stalls/retries.
//
// Round-2 review fixes (see the signal itself for detail): Critical-1
// (req_need gate) -- one hit_rsp_valid per WRITE BURST, not per beat.
// Critical-3 (id_busy_c) -- hold off a new accept while cur_id has a
// live op in MSHR/bypass (s_wready tracks do_accept_c's readiness
// exactly).  SINCE THE PIPELINE this applies at the door only to a beat
// bound for the BYPASS ENGINE; a cache-path beat is ordered at the
// resolve stage instead (ord_now_block_c / ord_merge_block_c).
// Critical-7 (skew_hazard_c) -- tags is 1cyc latency, data is 2cyc; an
// MSHR install inside a request's own array-read shadow writes too late
// for its resolve -- a 2-deep {valid,set} install history catches a
// same-set hit and forces a re-read (rr_start_c).  Important-10 (req_illegal) -- non-INCR bursts rejected SLVERR;
// narrower-than-16B beats are legal AXI4 (narrow-transfer-on-wide-bus,
// standard for a 32-bit master on a 128-bit port) and are handled
// correctly by cur_qoff/req_wstrb's byte-level merge (l2c_data.v) and
// aw_cur_addr/ar_cur_addr's size-generic per-beat stride below -- an
// earlier, over-strict awsize==16B-only gate here rejected every such
// write with SLVERR, which was the actual root cause of "boot_fsm's
// first RAM-zero-fill write hangs/aborts through the L2C-fronted DDR
// path" (2026-07-24 investigation): axi_narrow_to_wide.v (shared by the
// CPU dcache-bypass path, boot_fsm, and JTAG) legitimately emits
// awsize<4 single-beat transfers, and ALL of them were being SLVERR'd
// at this front door before ever reaching the byte-merge machinery
// below.  Important-11 (rw_favor) -- read/write beat selection
// alternates (s_wready derives from write_sel, not a stale proxy).
// Round-4 (byp_req_last) -- multi-beat bypass reads need the real
// per-beat last flag threaded through, see the signal below.
//
// FULL-LINE WRITE, NO FILL (2026-08-19, docs/l2c_perf.md S11).  A write
// burst that covers whole 64 B lines with every byte strobe set does not
// need the line fetched first -- the fill is read 4 x 128 b from DDR only
// to be overwritten in its entirety.  Measured cost of not doing this:
// ~5.0 cycles per 32-bit word on the boot RAM-zero path vs ~1.25 with
// l2c removed from the chain entirely, i.e. l2c was a 4x tax on
// streaming writes and burst length could not amortise it (the fill is
// per LINE).
//
// The path taken here is ALLOCATE-WITHOUT-FETCH, not write-through /
// no-allocate.  l2c is the SoC-wide point of coherency and does not
// snoop: whatever it holds is the newest copy, and every other master's
// read reaches DDR only through it.  Allocating keeps that true -- the
// line lands valid+dirty with the written data and any reader hits it.
// Write-through would create a window in which the newest data is in
// flight to DDR and present in NEITHER l2c nor (yet) DRAM, so a
// concurrent fill for the same line could return pre-write data and
// install it -- a stale-data state that is not observable today.  It
// would also need a new "in-flight write-through address" hazard query
// on the fill path (the analogue of victim_query_hit).  Allocating
// avoids all of it and reuses the victim/PLRU machinery unchanged.
//
// Detection is deliberately CONSERVATIVE -- burst-shape only, decided at
// AW time: INCR, awsize == 16 B, 64 B-aligned base, and (awlen+1) a
// multiple of 4, i.e. a whole number of naturally-aligned lines.  Beats
// are then gathered 4-at-a-time into flw_data/flw_strb at the front door
// (they do NOT enter the tag pipeline individually) and the assembled
// line is dispatched as ONE request.  If every strobe was set the line
// installs with no fill at all; if any beat was partial the gather
// replays the four quadrants as ordinary per-beat requests, which is
// bit-for-bit the pre-existing behaviour (fetch, allocate, merge) at a
// cost of 4 extra cycles per line.  Anything that is not the easy shape
// (unaligned, narrow awsize, partial first-beat strobe, a partial line,
// a bypass-window address) never arms the gather and is untouched.
//
// SECTORED VALID + DIRTY (2026-08-19, docs/l2c_perf.md S12).  Valid and
// dirty are four bits each per way -- one per 128-bit quadrant, which is
// exactly one 68040 L1D line.  This generalises the full-line-write path
// above rather than sitting beside it:
//
//   * A write that FULLY COVERS a quadrant supplies every byte of it, so
//     that quadrant needs no fill whatever the rest of the line looks
//     like.  It installs valid+dirty on the spot.  00f079c's whole-line
//     case is now just "all four quadrants covered at once"; its gather
//     survives as a THROUGHPUT shortcut (one tag-pipeline request per
//     line instead of four), not as the thing that removes the fetch.
//   * Only DIRTY quadrants are written back on eviction, so a line
//     touched in one quadrant costs one 16 B writeback beat, not four.
//
// Measured on tb-l2c-sctr (write 16 B at the head of every 64 B line,
// steady state, every line missing and evicting): 4 fill beats + 4
// writeback beats per line before, 0 + 1 after.
//
// The three rules that keep it correct, all of them load-bearing:
//
//   1. ALLOCATION KEYS ON TAG MATCH, NOT ON HIT.  A line whose tag
//      matches but whose requested quadrant is invalid must fill INTO
//      THAT WAY (`any_tm_c`/`tm_way_c`), never allocate a second way for
//      the same tag -- two ways with one tag in a set breaks the
//      uniqueness the whole design assumes.  `way_hit_c` (tag match AND
//      quadrant valid) still drives the hit response; `tag_match_c` (tag
//      match AND line valid) drives way selection and eviction.
//   2. A WRITE TO AN ALREADY-VALID QUADRANT OF A LINE WITH A LIVE MSHR
//      ENTRY RETRIES; it does not merge.  l2c_mshr merges into `act_line`,
//      which holds FETCHED data -- correct for a quadrant that was
//      invalid, stale for one the cache already owns.  Retrying costs the
//      fill's remaining latency and cannot livelock (the fill completes
//      without needing the front door).
//   3. THE VALID/DIRTY MERGE IS A READ-MODIFY-WRITE, and it is only sound
//      because the tag array is read at accept time and `skew_hazard_c`
//      bounces any request whose set an MSHR install touched in the two
//      cycles since.  Do not remove that bounce, and do not narrow it to
//      one cycle -- the pipeline resolves two cycles after its read, so
//      both history entries are load-bearing.  Since 2026-08-20 the
//      OTHER writer of that set, an older request still inside the
//      pipeline, is excluded at accept time by `set_haz_c` instead.
//
// Timing: none of this is in the front-door accept cone (`s_wready` /
// `do_accept_c`), which reads no tag-array state at all and is the
// design's binding path.  Quadrant-valid select and line-valid reduce
// are 4-bit functions computed in parallel with the tag compare's first
// LUT level and consumed by its second, so `way_hit_c` keeps its depth.
// What grows is the tag-array WRITE-DATA path (the RMW), which had
// slack.  See docs/l2c_perf.md S12.3.
//
// Verilog-2005, sync active-high rst.
`default_nettype none
module l2c_ctrl #(
    parameter SET_BITS = 12, parameter TAG_BITS = 14, parameter WAY_BITS = 3,
    parameter ADDR_WIDTH = 32, parameter ID_WIDTH = 6, parameter LINE_BITS = 512,
    // Per-MSHR-entry secondary-merge FIFO depth, and its index width
    // (ceil(log2(MSHR_REPLAY_N)), minimum 1).  See l2c.v's own
    // MSHR_REPLAY_N for the sizing argument and the measured area cost.
    parameter MSHR_REPLAY_N = 4, parameter MSHR_REPLAY_K = 2
) (
    input  wire clk, rst,
    // s_axi accept side (AW/W/AR -- the R/B response side is arbitrated
    // in l2c.v, fed by hit_rsp_*/mshr_rsp_* below).
    input  wire [ID_WIDTH-1:0]   s_awid,   input wire [ADDR_WIDTH-1:0] s_awaddr, input wire [7:0] s_awlen,
    input  wire [2:0] s_awsize,  input wire [1:0] s_awburst, input wire s_awvalid, output wire s_awready,
    input  wire [127:0] s_wdata, input wire [15:0] s_wstrb,  input wire s_wlast, input wire s_wvalid, output wire s_wready,
    input  wire [ID_WIDTH-1:0]   s_arid,   input wire [ADDR_WIDTH-1:0] s_araddr, input wire [7:0] s_arlen,
    input  wire [2:0] s_arsize,  input wire [1:0] s_arburst, input wire s_arvalid, output wire s_arready,
    // Level A (task #269) fetch-side AR sub-port.  A THIRD accept
    // candidate into the SAME front door, mirroring s_ar*/ar_have's own
    // shape exactly.  The caller (l2c.v) presents an already-decomposed
    // 128-bit-quadrant-granular burst descriptor here -- f_arsize/f_arlen
    // are NOT the caller's native 256b values, see l2c.v's decompose
    // comment.  Read-only: there is no f_aw*/f_w* -- fetch never writes.
    input  wire [ID_WIDTH-1:0]   f_arid,   input wire [ADDR_WIDTH-1:0] f_araddr, input wire [7:0] f_arlen,
    input  wire [2:0] f_arsize,  input wire [1:0] f_arburst, input wire f_arvalid, output wire f_arready,
    // Hit-path response (to l2c.v's R/B arbiter).  hit_rsp_is_fetch tags
    // which of the two front-door sources (LSU s_axi vs. the fetch f_ar*
    // port above) this particular skid-register occupant came from, so
    // l2c.v's response arbiter can route it to the correct physical bus
    // -- purely a passthrough tag alongside hit_rsp_is_write, set at the
    // exact same 3 S_LOOKUP resolve sites; no hit-detection logic changes.
    output reg  hit_rsp_valid, output reg hit_rsp_is_write, output reg hit_rsp_is_fetch, output reg [ID_WIDTH-1:0] hit_rsp_id,
    output reg  [127:0] hit_rsp_rdata, output reg hit_rsp_last, output reg [1:0] hit_rsp_resp, input wire hit_rsp_ready,
    // MSHR replay response (pass-through, to l2c.v's R/B arbiter).
    // mshr_rsp_is_fetch: same source tag as hit_rsp_is_fetch above,
    // derived from the extra source bit folded into the MSHR's internal
    // ID compare (Critical-3 fix, see the l2c_mshr instantiation below).
    output wire mshr_rsp_valid, output wire mshr_rsp_is_write, output wire mshr_rsp_is_fetch, output wire [ID_WIDTH-1:0] mshr_rsp_id,
    output wire [127:0] mshr_rsp_rdata, output wire mshr_rsp_last, output wire [1:0] mshr_rsp_resp, input wire mshr_rsp_ready,
    // Victim-buffer push + hazard query (l2c_victim lives in l2c.v).
    output reg  victim_push_valid, output reg [ADDR_WIDTH-1:0] victim_push_addr, output reg [LINE_BITS-1:0] victim_push_data,
    output reg  [3:0] victim_push_dsec,
    input  wire victim_push_ready, output wire [ADDR_WIDTH-1:0] victim_query_addr, input wire victim_query_hit,
    // Bypass classification + handoff (l2c_bypass lives in l2c.v).
    output wire [ADDR_WIDTH-1:0] byp_match_addr, input wire byp_match_hit,
    // Second bypass classification port, on the WRITE-side beat address --
    // byp_match_addr carries the read address in any cycle a read wins the
    // front door, and the full-line gather must classify its own line in
    // exactly those cycles too.  See l2c_bypass.v's match_addr2 comment.
    output wire [ADDR_WIDTH-1:0] byp_match2_addr, input wire byp_match2_hit,
    output reg  byp_req_valid, output reg byp_req_is_write, output reg [ADDR_WIDTH-1:0] byp_req_addr,
    output reg  [ID_WIDTH-1:0] byp_req_id, output reg [127:0] byp_req_wdata, output reg [15:0] byp_req_wstrb,
    output reg  byp_req_need_resp, output reg byp_req_last, input wire byp_req_ready,
    // Bypass active-op query (Critical-3, l2c_bypass lives in l2c.v).
    input  wire byp_active_valid, input wire [ID_WIDTH-1:0] byp_active_id,
    // MSHR fill AR/R sub-port (pass-through, arbitrated vs. bypass in l2c.v).
    output wire [ID_WIDTH-1:0] mf_arid, output wire [ADDR_WIDTH-1:0] mf_araddr, output wire [7:0] mf_arlen,
    output wire [2:0] mf_arsize, output wire [1:0] mf_arburst, output wire mf_arvalid, input wire mf_arready,
    input  wire [ID_WIDTH-1:0] mf_rid, input wire [127:0] mf_rdata, input wire [1:0] mf_rresp,
    input  wire mf_rlast, input wire mf_rvalid, output wire mf_rready,
    // 2026-07-24 L2C/MIG-cal-race debug tap — see dbg_l2c_write_snap below.
    output wire [10:0] dbg_write_snap,
    // 2026-07-25 hit/miss/occupancy counters -- so a live boot can show
    // whether the cache is doing anything useful at all, not just that
    // it isn't SLVERR'ing.  dbg_hit_count includes both a genuine
    // tag-array hit (any_hit_c && s_lookup_hit_go) and an MSHR merge
    // (mshr_lu_hit && merge_valid_c) -- a merge still avoids issuing a
    // SECOND DRAM fill for an already-in-flight miss, so it counts
    // toward "avoided a DRAM round trip" same as a tag hit.
    // dbg_miss_count increments once per genuine new MSHR allocation
    // (s_lookup_miss_go). dbg_mshr_occupancy is a live up/down count of
    // outstanding MSHR entries (alloc on s_lookup_miss_go, free on
    // mshr_rsp_valid && mshr_rsp_ready), bounded [0, l2c_mshr's N=8].
    // All three are free-running (wrap on overflow) -- a live JTAG/VIO
    // read gets a point-in-time snapshot, not a saturating stat.
    output reg  [31:0] dbg_hit_count,
    output reg  [31:0] dbg_miss_count,
    output wire [3:0]  dbg_mshr_occupancy
);
    `include "l2c_plru_funcs.vh"
    // WLAST isn't relied on -- beat-count tracking (aw_cur_last) drives it.
    /* verilator lint_off UNUSEDSIGNAL */
    wire unused_wlast = s_wlast;
    /* verilator lint_on UNUSEDSIGNAL */
    // -- Reset walk -------------------------------------------------------
    wire rst_busy, rst_clr_en; wire [SET_BITS-1:0] rst_clr_set;
    /* verilator lint_off PINCONNECTEMPTY */
    l2c_reset #(.SET_BITS(SET_BITS)) u_rst (
        .clk(clk), .rst(rst), .busy(rst_busy), .done(), .clr_en(rst_clr_en), .clr_set(rst_clr_set)
    );
    /* verilator lint_on PINCONNECTEMPTY */
    // -- Front-door beat trackers (AW/W burst, AR burst) -------------------
    // 2026-08-20 (lookup pipelining): the per-beat address is a RUNNING
    // REGISTER, not `base + (beat << size)` recomputed combinationally.  It
    // is bit-identical for INCR -- the only burst type this module accepts
    // -- and it takes a 32-bit adder AND an 8->32 barrel shifter out of the
    // accept cone, which is where the pipeline's new set/line hazard
    // comparators had to go.  The shift amount is now applied once, at
    // header accept, to produce a registered per-beat stride.
    reg aw_have; reg [ID_WIDTH-1:0] aw_id;
    reg [7:0] aw_len; reg [7:0] aw_beat; reg [1:0] aw_burst;
    reg [ADDR_WIDTH-1:0] aw_cur, aw_step;
    reg ar_have; reg [ID_WIDTH-1:0] ar_id;
    reg [7:0] ar_len; reg [7:0] ar_beat; reg [1:0] ar_burst;
    reg [ADDR_WIDTH-1:0] ar_cur, ar_step;
    // 2026-09-17 (READ-channel twin of the AW-channel fix, see "AR BEAT
    // ADDRESS" below).  ar_beat's only consumer, `ar_beat == ar_len`, is a
    // pure function of registers and is therefore settled a full cycle
    // before the front door asks for it.  Carried as a register refreshed
    // on exactly the edges that move ar_beat or ar_len, which keeps an
    // 8-bit comparator -- and ar_beat itself -- out of the door's cone.
    //
    // Cannot go stale in the way a registered VICTIM could: a victim is a
    // snapshot of state other agents keep changing, whereas this is a
    // function of registers this block itself owns, and every write to
    // ar_beat/ar_len in this file updates it in the same statement.
    // ar_last_q is unreadable while ar_have is 0 (every consumer is
    // qualified by ar_have), which is what makes the reset value harmless:
    // ar_beat/ar_len have no reset either.
    reg ar_last_q;
    //
    // TWO-DEEP HEADER DOOR (2026-08-20, and it is the OTHER half of
    // pipelining the lookup -- shipping the pipeline alone gets ~2/3 of
    // the win and no more).
    //
    // 7d49e1f measured the single-entry door (`s_arready = !ar_have`) as
    // costing nothing, REFUSED-EARLIER == 0 on every cacheable row.  That
    // result was TRUE and is now VOID, and the reason it was true is
    // exactly the reason it stopped being true: with a 3-cycle lookup
    // loop, `ar_have` cleared on the edge that took a burst's last beat
    // and the next header then had a TWO-CYCLE window (S_WAIT + S_LOOKUP)
    // in which to arrive before an accept slot was wasted.  Two cycles of
    // slack against a three-cycle loop.  At one beat per cycle the slack
    // is zero: the header for burst N+1 can only be latched the cycle
    // AFTER burst N's last beat, and its own first beat only the cycle
    // after that.  One-beat bursts therefore run at 2.00 cyc/op --
    // measured, with 255 REFUSED-EARLIER bubbles over 256 ops, on the
    // pipeline before this slot existed.
    //
    // The fix is a one-entry PRE-LATCH per channel, not a wider
    // `s_*ready` expression.  `s_arready = !ar_pend_v` stays a plain
    // register bit; making it `!ar_have || <burst ends this cycle>` would
    // have pulled do_accept_c -- and with it the whole hazard/selection
    // cone -- into the AXI ready path, which is the one output the
    // upstream master's own timing closure depends on.  The pre-latch
    // stores awsize rather than the decoded per-beat stride so the
    // 1<<size shifter stays on the promotion path (registered inputs)
    // instead of multiplying on the accept path.  Cost ~53 FF/channel.
    reg ar_pend_v; reg [ID_WIDTH-1:0] ar_pend_id; reg [ADDR_WIDTH-1:0] ar_pend_addr;
    reg [7:0] ar_pend_len; reg [2:0] ar_pend_size; reg [1:0] ar_pend_burst;
    reg aw_pend_v; reg [ID_WIDTH-1:0] aw_pend_id; reg [ADDR_WIDTH-1:0] aw_pend_addr;
    reg [7:0] aw_pend_len; reg [2:0] aw_pend_size; reg [1:0] aw_pend_burst;
    reg aw_pend_flw;
    // Level A (task #269): fetch-side AR tracker.  Originally single-
    // burst-in-flight (`f_arready = !fetch_ar_have`), which deasserted
    // f_arready for the whole life of a captured burst's beat walk and
    // so re-serialised the CPU's 5-MSHR instruction prefetcher down to
    // one fetch in flight -- the "REFUSED-EARLIER pathology on
    // back-to-back one-beat fetch bursts" the original note predicted.
    //
    // 2026-09-02: given the SAME one-entry pre-latch the LSU header
    // channels already carry, for the same reason and in the same shape.
    // This is the fabric's only 1-deep gate on the I-fetch path (cpu040's
    // axi_i binds straight to this port, bypassing the crossbar -- see
    // fpga_top_ddr.vh's f_axi_* bind), so it was the single limiter
    // between a 5-deep prefetcher above and 8 MSHRs / an 8-outstanding
    // MIG bridge below.
    //
    // `f_arready` stays a plain register bit (`!fetch_ar_pend_v`), NOT
    // `!fetch_ar_have || <burst ends this cycle>` -- see the s_arready
    // comment above for why pulling do_accept_c into the AXI ready path
    // is the thing to avoid.
    reg fetch_ar_have; reg [ID_WIDTH-1:0] fetch_ar_id; reg [ADDR_WIDTH-1:0] fetch_ar_base;
    reg [7:0] fetch_ar_len; reg [2:0] fetch_ar_size; reg [7:0] fetch_ar_beat; reg [1:0] fetch_ar_burst;
    reg fetch_ar_pend_v; reg [ID_WIDTH-1:0] fetch_ar_pend_id; reg [ADDR_WIDTH-1:0] fetch_ar_pend_addr;
    reg [7:0] fetch_ar_pend_len; reg [2:0] fetch_ar_pend_size; reg [1:0] fetch_ar_pend_burst;
    assign s_awready = !aw_pend_v && !rst_busy; assign s_arready = !ar_pend_v && !rst_busy;
    assign f_arready = !fetch_ar_pend_v && !rst_busy;
    wire [ADDR_WIDTH-1:0] aw_cur_addr = aw_cur;
    wire [ADDR_WIDTH-1:0] ar_cur_addr = ar_cur;
    wire [ADDR_WIDTH-1:0] fetch_ar_cur_addr = fetch_ar_base + ({{(ADDR_WIDTH-8){1'b0}}, fetch_ar_beat} << fetch_ar_size);
    wire aw_cur_last = (aw_beat == aw_len);
    wire ar_cur_last = ar_last_q;
    wire fetch_ar_cur_last = (fetch_ar_beat == fetch_ar_len);
    // ── AR BEAT ADDRESS: the adder is NOT in the select cone ─────────────
    //
    // 2026-09-17.  `ar_cur <= ar_cur + ar_step` written inside
    // `else if (ar_take_c)` MEANS `ar_cur + (ar_take_c ? ar_step : 0)`,
    // because the alternative to advancing is holding and holding is adding
    // zero.  That is how the tool builds it: the front-door accept decision
    // -- among the latest signals in this module -- enters the bit-0..7
    // CARRY8 on DI[1] and must then RIPPLE the whole ADDR_WIDTH chain
    // before it reaches a flop.  The signature is a per-bit slack gradient
    // (one step per CARRY8), which is what the measured aw_cur family had:
    //
    //   aw_cur_reg[12] -0.103 (2 CARRY8) ... aw_cur_reg[27] -0.189 (4 CARRY8)
    //
    // This is the SAME STRUCTURE on the read channel.  It was left alone
    // when aw_cur was fixed only because no ar_cur endpoint had yet been
    // measured failing -- which is a statement about which traffic the
    // timing run happened to stress, not about the logic.  A defect is
    // patched whether or not today's build happens to reach it.
    //
    // THE ASYMMETRY.  Both adder operands are plain registers, so the sum
    // is settled from the first picosecond of the cycle and then sits idle
    // for the whole time the front door is making up its mind.  Computing
    // it unconditionally and letting the late signal do nothing but PICK
    // between two already-ready values costs ~ADDR_WIDTH LUTs, zero
    // cycles, and no new state.
    //
    // WHY THE UPDATE IS WRITTEN OUTSIDE THE HEADER-DOOR if/else BELOW.  The
    // fold is only legal because "hold" and "add zero" are the same thing,
    // so the defence is to give the register a real CE and leave NO
    // ar_cur-hold term in its D.  Re-folding then costs the tool TWO
    // ADDR_WIDTH-bit operand muxes to save one, and stops being worth
    // doing.  `keep` is a second line of defence, not the primary one --
    // do not delete the CE shape and rely on the attribute alone.
    (* keep = "true" *) wire [ADDR_WIDTH-1:0] ar_cur_next = ar_cur + ar_step;
    wire [ADDR_WIDTH-1:0] ar_cur_ld = ar_pend_v ? ar_pend_addr : s_araddr;
    // Beat consumption + burst completion, forward-declared so the header
    // promotion logic below can be written once for both channels.  The
    // WRITE side has two consumers -- the tag pipeline and the full-line
    // gather -- and they are mutually exclusive by construction: while
    // flw_gath_rdy_c is high the raw-beat term of write_avail is low, and
    // while a gathered line is being dispatched (flw_avail_c) no raw beat
    // is taken.
    //
    // Level A (task #269): `ar_take_c` below is scoped to LSU-sourced read
    // beats specifically (lsu_read_sel, defined further down), NOT the
    // broader `read_sel` -- a fetch-sourced beat must NEVER advance the
    // LSU's own ar_beat/ar_cur/ar_have bookkeeping, or the LSU's burst
    // tracking silently corrupts on any cycle fetch traffic wins the door
    // instead. See fetch_take_c below for the fetch tracker's own advance.
    wire aw_take_c, ar_take_c;
    wire aw_ends_c = aw_take_c && aw_cur_last;
    wire ar_ends_c = ar_take_c && ar_cur_last;
    wire aw_act_free_c = !aw_have || aw_ends_c;   // active slot free at this edge
    wire ar_act_free_c = !ar_have || ar_ends_c;
    // ar_cur's two write enables, transcribed one-for-one from the header
    // door's if/else below so the equivalence is checkable by eye:
    //   load = the door promotes a header into the active slot this edge
    //   inc  = the door does NOT promote and an LSU read beat is taken
    // They are mutually exclusive by construction (`!ar_act_free_c`).
    wire ar_cur_load_c = ar_act_free_c && (ar_pend_v || (s_arvalid && s_arready));
    wire ar_cur_inc_c  = !ar_act_free_c && ar_take_c;
    // Fetch-side twin of ar_take_c / ar_ends_c / ar_act_free_c.  Scoped to
    // fetch_read_sel for the same reason ar_take_c is scoped to
    // lsu_read_sel: an LSU-sourced accept must never advance the fetch
    // burst's own beat bookkeeping, or vice versa.
    wire fetch_ar_take_c = door_take_c && fetch_read_sel;
    wire fetch_ar_ends_c = fetch_ar_take_c && fetch_ar_cur_last;
    wire fetch_ar_act_free_c = !fetch_ar_have || fetch_ar_ends_c;
    // -- Full-line-write gather (see the header) ---------------------------
    // aw_flw: THIS burst's shape permits gathering.  Decided once, at AW
    // accept, from the header alone -- INCR, 16 B beats, 64 B-aligned base,
    // (awlen+1) a multiple of 4.  awlen[1:0]==2'b11 is exactly that
    // divisibility test.
    reg aw_flw;
    localparam G_IDLE = 2'd0, G_GATH = 2'd1, G_FULL = 2'd2, G_RPLY = 2'd3;
    reg [1:0]            gst;
    reg [LINE_BITS-1:0]  flw_data;   // assembled line
    reg [63:0]           flw_strb;   // per-byte strobes, replay path only
    reg [1:0]            flw_cnt;    // quadrants gathered so far (wraps at 4)
    reg [1:0]            flw_rq;     // replay quadrant index
    reg                  flw_allstrb;// AND of every gathered beat's &wstrb
    reg                  flw_last;   // gathered beat was the burst's last
    reg [ADDR_WIDTH-1:0] flw_addr;   // 64 B-aligned line base
    reg [ID_WIDTH-1:0]   flw_id;     // burst's AXI ID, latched: aw_id may
                                     // already belong to the NEXT burst by
                                     // the time the assembled line dispatches
    assign byp_match2_addr = aw_cur_addr;
    // Arm only on a line boundary, and only when the first beat's strobes
    // are all set -- a first beat that is already partial can never produce
    // a full line, so the common "byte-enabled store stream" shape never
    // pays the gather's 4-cycle replay penalty.  The bypass classification
    // is taken here, on the line base, and NOT re-asked for quadrants 1..3;
    // l2c_bypass asserts every enabled window is at least 64 B granular,
    // which makes that sound.
    wire flw_arm_c      = aw_have && aw_flw && (aw_beat[1:0] == 2'b00) &&
                          !byp_match2_hit && (&s_wstrb);
    wire flw_gath_rdy_c = !rst_busy && aw_have &&
                          ((gst == G_GATH) || ((gst == G_IDLE) && flw_arm_c));
    wire flw_gath_c     = flw_gath_rdy_c && s_wvalid;
    // A whole line is assembled and wants the tag pipeline.
    wire flw_avail_c    = (gst == G_FULL) || (gst == G_RPLY);
    // 2026-08-19 (docs/l2c_perf.md): S_HITRESP is gone.  It existed only
    // to hold the op for the cycle its R/B response drained, which cost a
    // 4th cycle on EVERY hit even when the response was accepted
    // immediately.  hit_rsp_* is now a plain 1-deep skid register --
    // S_LOOKUP loads it and returns straight to S_IDLE, and only stalls
    // (retries S_LOOKUP) when the register is still full.  Measured
    // 4.00 -> 3.00 cycles per sustained hit, no added state.
    //
    // 2026-08-20 (lookup pipelining): the shared `st` register is gone
    // too.  The three cycles are now three STAGES that hold three
    // different requests, so the loop retires one beat per cycle instead
    // of one per three:
    //
    //   S0  accept        drive the array read address, load stage 1
    //   S1  array read    tags out at the end of this cycle
    //   S2  resolve       tags (registered by tq_*) + data (l2c_data's own
    //                     2nd output register) both land here
    //
    // 2026-09-15 (array-address pipelining): a FOURTH stage.  See the
    // "ARRAY ADDRESS REGISTER" section of the header.  Stage A's registers
    // are `a_*`, stage Q's are `q_*`, and the resolve stage keeps the old
    // `req_*` names so the resolve body below reads exactly as it did.
    reg a_v, q_v, req_v;
    // Important-11: alternate read/write preference each accept (was fixed
    // read-absolute priority, could starve writes under a continuous AR stream).
    reg rw_favor;
    // Level A (task #269): a second read source.  read_sel's own
    // read-vs-write arbitration is UNCHANGED (still just "is there ANY
    // read pending"); fetch_favor is a second, independent alternator
    // that decides WHICH read source wins when both s_ar* (LSU) and f_ar*
    // (fetch) are pending in the same cycle -- same alternation shape as
    // rw_favor, so neither read source can starve the other.
    reg fetch_favor;
    wire read_avail  = ar_have || fetch_ar_have;
    // A beat destined for the gather is NOT offered to the tag pipeline;
    // an assembled line (flw_avail_c) is offered INSTEAD of a raw beat.
    wire write_avail = flw_avail_c ||
                       (aw_have && s_wvalid && (gst == G_IDLE) && !flw_gath_rdy_c);
    wire read_sel  = read_avail && (!write_avail || !rw_favor);
    wire write_sel = !read_sel && write_avail;
    wire any_sel    = read_sel || write_sel;
    // Sub-pick within read_sel: fetch wins only if it's actually pending
    // AND (LSU isn't pending OR it's fetch's turn); LSU wins the rest of
    // read_sel's cycles.  When only one read source is pending, the other
    // term is structurally forced regardless of fetch_favor.
    wire fetch_read_sel = read_sel && fetch_ar_have && (!ar_have || fetch_favor);
    wire lsu_read_sel   = read_sel && !fetch_read_sel;
    // G_FULL dispatches the line base (qoff 0, full strobe); G_RPLY walks
    // the four quadrants exactly as the raw beats would have arrived.
    wire [ADDR_WIDTH-1:0] flw_disp_addr = (gst == G_FULL)
                                        ? flw_addr
                                        : {flw_addr[ADDR_WIDTH-1:6], flw_rq, 4'b0000};
    wire flw_disp_last_c = flw_last && ((gst == G_FULL) || (flw_rq == 2'd3));
    wire [ADDR_WIDTH-1:0] cur_addr = fetch_read_sel ? fetch_ar_cur_addr
                                   : (lsu_read_sel ? ar_cur_addr
                                   : (flw_avail_c ? flw_disp_addr : aw_cur_addr));
    wire [ID_WIDTH-1:0]   cur_id   = fetch_read_sel ? fetch_ar_id
                                   : (lsu_read_sel ? ar_id
                                   : (flw_avail_c ? flw_id : aw_id));
    wire                  cur_last = fetch_read_sel ? fetch_ar_cur_last
                                   : (lsu_read_sel ? ar_cur_last
                                   : (flw_avail_c ? flw_disp_last_c : aw_cur_last));
    wire                  cur_need = read_sel ? 1'b1 : cur_last;
    // Level A (task #269): tags this accept as fetch-sourced for the
    // downstream MSHR id-source-qualification (Q5) and the hit/mshr
    // response routing tag (hit_rsp_is_fetch/mshr_rsp_is_fetch).
    wire                  cur_is_fetch = fetch_read_sel;
    wire [127:0]          cur_wdata = flw_avail_c ? flw_data[flw_rq*128 +: 128] : s_wdata;
    wire [15:0]           cur_wstrb = flw_avail_c ? flw_strb[flw_rq*16  +: 16]  : s_wstrb;
    wire [1:0]              cur_qoff = cur_addr[5:4];
    wire [SET_BITS-1:0]     cur_set  = cur_addr[SET_BITS+5:6];
    wire [TAG_BITS-1:0]     cur_tag  = cur_addr[31:SET_BITS+6];
    assign byp_match_addr = cur_addr;
    // A gathered line was classified non-bypass on its base address before
    // the first beat was ever taken (flw_arm_c), so the write_sel term is
    // structurally redundant -- it is written anyway so that a gathered
    // line can NEVER be routed into the bypass engine, which would drop
    // three of its four quadrants.
    //
    // It MUST be qualified by write_sel, not by flw_avail_c alone: a READ
    // wins the front door in plenty of the cycles where an assembled line
    // is waiting, and `byp_match_hit && !flw_avail_c` sent every one of
    // those bypass-window reads down the CACHE path instead of to the
    // bypass engine (found by tb-l2c L2C_SEED=1: a bypass read returned
    // quadrant-shifted data plus an extra R beat).  write_sel does not
    // feed write_avail/read_sel, so this is not a combinational loop.
    wire is_bypass_c = byp_match_hit && !(flw_avail_c && write_sel);
    // Important-10 (relaxed 2026-07-24, see header comment): only INCR
    // bursts are required -- awsize is not restricted to 16B.  A gathered
    // line's legality was settled at AW accept (aw_flw requires INCR), and
    // aw_burst may by then describe the NEXT burst, so it is not re-read.
    wire cur_burst_ill_c = fetch_read_sel ? (fetch_ar_burst != 2'b01)
                         : (lsu_read_sel ? (ar_burst != 2'b01)
                         : (flw_avail_c ? 1'b0 : (aw_burst != 2'b01)));
    wire illegal_c = cur_burst_ill_c;
    // Critical-3: hold off accept while cur_id has a live op in MSHR/bypass.
    //
    // The bypass term is qualified by !is_bypass_c (2026-08-20).  What
    // Critical-3 protects is AXI4's same-id response ordering, and the
    // hazard is a request that would take a DIFFERENT path from an
    // already-live op with the same id -- the cache path can answer a hit
    // in three cycles while a bypass op is still in DRAM, which would put
    // two responses for one id out of order.  A request that is ITSELF
    // headed for the bypass engine has no such hazard: l2c_bypass queues
    // it behind the live op and delivers responses in acceptance order
    // (see its "ONE ID AT A TIME" header note, which is also what makes
    // its outbound transactions share one AXI id).
    //
    // Applying the hold-off to bypass-to-bypass anyway was the SECOND
    // half of the 512 B-bypass-read problem, and a separable one: a burst
    // is decomposed into per-beat requests that all carry the burst's own
    // id, so every beat after the first was refused here.  That kept the
    // burst sitting in ar_have for the whole transfer, and CPU traffic
    // behind it inherited the wait -- 5040 of 5430 front-door idle cycles
    // were this term (7d49e1f).  Deepening l2c_bypass alone would not
    // have fixed it; this line is what lets the beats actually reach the
    // deeper engine.
    //
    // 2026-08-20 (lookup pipelining): Critical-3 now applies at the door
    // only to a beat headed for the BYPASS ENGINE, which is dispatched
    // there and then and so must be ordered against everything already in
    // flight.  A CACHE-path beat is no longer refused here for its id --
    // the check moved to the resolve stage (`ord_*_block_c` below), where
    // the outcome is actually known.  That is what lets beats 2..N of a
    // MISSING burst reach the MSHR and merge into their leader's entry
    // instead of being held at the door for a whole DRAM round trip per
    // beat; the `idbusy` bubble counter was 3316 of 3574 idle cycles on
    // `stream_read_64B` before this.
    //
    // Level A (task #269), Q5: mshr_idq_busy is already source-qualified
    // -- it is driven by the u_mshr instance below, whose idq_id/alloc_id/
    // merge_id all carry an extra {is_fetch, id} bit so axi_i and axi_d
    // (independently-numbered AXI masters) can never alias each other's
    // IDs in the MSHR table.  The byp_active_id compare below is NOT
    // qualified -- l2c_bypass.v carries no source tag (out of scope for
    // Level A, see the report) -- so a live bypass op and an unrelated
    // fetch op sharing a numeric ID can still spuriously stall each other;
    // narrower and lower-severity than the pre-fix mshr-side bug (a
    // throughput stall, not a correctness issue), and bypass windows are
    // never legitimate fetch targets in practice.  The NEW idq2_id/idq2_vec
    // ordering check the 2026-08-20 pipelining added (see u_mshr below)
    // is likewise widened with the {is_fetch, id} tag, for the same
    // reason -- an unwidened idq2_id would silently miss a same-numeric-ID
    // match between a live fetch entry and an LSU query (or vice versa),
    // reopening exactly the response-ordering hazard idq2 exists to close.
    wire mshr_idq_busy;
    wire pipe_id_haz_c;   // same id already inside the lookup pipeline
    wire id_busy_c = mshr_idq_busy ||
                     (byp_active_valid && (byp_active_id == cur_id) && !is_bypass_c);
    // -- Lookup-pipeline occupancy + array hazard at the door -------------
    //
    // Stage-A/stage-Q/stage-2 request state.  `a_*` is stage A (its set is
    // the one standing on the array address pins THIS cycle), `q_*` is
    // stage Q (its array output has landed in the arrays' first output
    // register THIS cycle), `req_*` is stage 2 (resolve).
    reg [ADDR_WIDTH-1:0] a_addr; reg [SET_BITS-1:0] a_set; reg [TAG_BITS-1:0] a_tag;
    reg a_is_write; reg [ID_WIDTH-1:0] a_id; reg [127:0] a_wdata; reg [15:0] a_wstrb;
    reg [1:0] a_qoff; reg a_last; reg a_need; reg a_illegal; reg a_full;
    reg [3:0] a_cov, a_dty, a_want; reg [LINE_BITS-1:0] a_line;
    reg a_is_fetch;
    reg [ADDR_WIDTH-1:0] q_addr; reg [SET_BITS-1:0] q_set; reg [TAG_BITS-1:0] q_tag;
    reg q_is_write; reg [ID_WIDTH-1:0] q_id; reg [127:0] q_wdata; reg [15:0] q_wstrb;
    reg [1:0] q_qoff; reg q_last; reg q_need; reg q_illegal; reg q_full;
    reg [3:0] q_cov, q_dty, q_want; reg [LINE_BITS-1:0] q_line;
    // Level A (task #269): stage-1 copy of the fetch source tag, threaded
    // to req_is_fetch (stage 2) the same way every other q_*->req_* field
    // is -- see the S0->S1 / S1->S2 blocks below.
    reg q_is_fetch;
    // Forward declaration for pipe_id_haz_c below, which is written before
    // req_id is declared.  2026-09-15: req_set_w/req_tag_w/req_is_write_w
    // went with it -- set_haz_c no longer reads the resolve stage at all
    // (see its own comment), so only the id survives.
    wire [ID_WIDTH-1:0] req_id_w;
    wire pipe_adv_c, p2_free_c, rr_start_c, rr_busy_c;
    // A cycle in which the pipeline shifts, and therefore the only kind of
    // cycle in which stage A can take a new request.
    //
    // 2026-09-15.  This USED to be `(!q_v || q_adv_c) && ...`, i.e. an
    // ELASTIC pipeline: a new request could be accepted into an empty
    // stage 1 even while stage 2 was stalled.  The array-address register
    // makes a rigid shift strictly better, and not only because it is
    // simpler:
    //
    //   * ONE array clock enable.  Stages A and Q now advance together by
    //     construction, so `ce_c` is correct for BOTH of l2c_data's output
    //     registers.  An elastic pipeline would have needed two separate
    //     enables, i.e. a SECOND combinational-cone-to-64-URAM-CE-pin net
    //     beside the one that already exists -- the exact net shape this
    //     whole change exists to remove.
    //   * TWO set/tag comparators in the accept cone, not three.  A rigid
    //     shift makes "an accept happened this cycle" imply "stage 2
    //     retired this cycle" (p2_free_c is a term of do_accept_c), so the
    //     resolve-stage occupant's array write always lands on the edge
    //     BEFORE the accepted request's array read and is therefore
    //     visible to it.  Only stages A and Q have to be checked by
    //     `set_haz_c`.  With an elastic door a stalled stage-2 occupant
    //     could still be sitting there when the new read issues, so all
    //     three stages would have needed a comparator -- growth in the
    //     binding accept cone, which is the one thing that must not grow.
    //
    // What it costs: a request can no longer be accepted into an empty
    // pipeline while stage 2 is stalled.  Measured cost on the tb's
    // throughput rows: none (see the cycles/op table in the header).
    wire accept_slot_c = pipe_adv_c && !rr_busy_c && !rr_start_c;
    // ...but the BYPASS door keeps the ELASTIC condition, which is the one
    // it had before the rigid shift.
    //
    // Every argument for the rigid form above is about the tag/data ARRAYS:
    // one clock enable, two set comparators, an array write ordered ahead of
    // an array read.  A bypass beat touches none of them -- it never enters
    // the lookup pipeline, never reads the arrays and never writes them.  It
    // was nevertheless sharing `accept_slot_c`, so making that signal rigid
    // silently coupled MMIO dispatch to the cache's resolve: a beat for a
    // non-cacheable window would sit at the door for as long as stage 2 was
    // stalled on an MSHR allocation or a full victim buffer, which on this
    // SoC means VIA/SCSI/DAFB register traffic queueing behind a DRAM miss
    // that has nothing to do with it.
    //
    // `(!a_v || pipe_adv_c)` is exactly what `(!q_v || q_adv_c)` meant
    // before the extra stage: "the stage a new request would enter is free
    // or is emptying this cycle".  The rr terms stay, as they did before.
    // Ordering against the pipeline is unaffected -- that is `pipe_id_haz_c`
    // (all three stages) and `id_busy_c`, neither of which is touched.
    //
    // Costs no timing: do_bypass_c does not feed `ra_q` or the a_* register
    // enables, and `a_v` is a register.
    //
    // ⚠ TWO THINGS THE ABOVE DOES NOT SAY, added 2026-09-18 (race audit),
    // because between them they decide whether anyone should ever touch
    // this line again:
    //
    // 1. THIS RELIEF IS PARTIAL, and the paragraph above reads as though it
    //    were total.  The pipeline is a RIGID shift, so while stage 2 is
    //    stalled `pipe_adv_c` is 0 and `a_v` cannot change.  Whatever stage
    //    A held when the stall began, it holds for the whole stall.  So
    //    `(!a_v || pipe_adv_c)` frees the bypass door only in the sub-case
    //    where the pipe happened to be DRAINED behind the stalled request
    //    -- and a long stall means a full MSHR or a full victim buffer,
    //    i.e. exactly the loaded conditions under which stage A is most
    //    likely to be occupied.  The motivating case, "VIA/SCSI/DAFB
    //    register traffic queueing behind a DRAM miss", is still blocked
    //    whenever anything is queued behind that miss.
    //
    //    If this ever needs to be complete, the answer is to drop the term
    //    entirely -- `byp_slot_c = !rr_busy_c && !rr_start_c` -- because a
    //    bypass beat NEVER ENTERS STAGE A, so "the stage a new request
    //    would enter is free" is not a statement about it at all.  Nothing
    //    is lost: capacity is `byp_req_ready`, and ordering is
    //    `pipe_id_haz_c` (all three stages, {is_fetch,id}-tagged) and
    //    `id_busy_c`, none of which this term participates in.  It also
    //    SHRINKS the accept cone rather than growing it.  Not done here
    //    because of (2).
    //
    // 2. NONE OF IT IS LIVE ON HARDWARE.  fpga_top_ddr.vh leaves the
    //    windows at l2c.v's defaults -- NUM_BYPASS_WINDOWS=1, BYP_WIN_EN=0
    //    -- so `is_bypass_c` is constant false, `do_bypass_c` is constant
    //    false, and this whole door is dead logic the synthesiser removes.
    //    The motivation recorded above cannot occur in THIS SoC at all:
    //    VIA/SCSI/DAFB traffic is decoded to the crossbar's S1 (peripheral
    //    bus) and never reaches l2c's slave port; l2c only ever sees the
    //    cacheable DDR span.  The window's one historical consumer (the
    //    DDR-backed RAM-disk volume at 0x7000_0000) was deleted on
    //    2026-09-10.  tb_l2c.v sets BYP_WIN_EN=1, so the bypass path and
    //    every guard in l2c_bypass.v are exercised in SIMULATION ONLY.
    //
    //    Which is also why no test distinguishes the two forms: reverting
    //    this to `accept_slot_c` outright leaves the directed suite and the
    //    randomized scoreboard green.  It is a latency property, and the tb
    //    drives a single serialized AR stream, so a bypass beat always
    //    queues behind the cache reads in the MASTER and never reaches the
    //    door while the pipe is frozen.  Demonstrating either form needs a
    //    second independent requester.
    //
    //    So: if a future raw-DDR client re-enables a window (fpga_top_ddr.vh
    //    says what else that takes), fix (1) at the same time, and build the
    //    second requester first so the change is testable.  Until then this
    //    is correct-but-dead and not worth touching the FMax-critical accept
    //    cone for.
    wire byp_slot_c    = (!a_v || pipe_adv_c) && !rr_busy_c && !rr_start_c;
    //
    // ARRAY READ-AFTER-WRITE HAZARD (the reason a pipelined lookup is not
    // just "delete the FSM").  Stage 2 writes the tag and data arrays on
    // the edge it resolves, and that write is invisible to whatever is in
    // stages 1 and 2 at the time.  Two stages, two blind cycles: checking a
    // new request against both occupied stages at accept time is therefore
    // EXACT, not conservative.
    //
    // ⚠ WHY IT IS INVISIBLE -- CORRECTED 2026-09-18 (race audit).  This
    // used to read "both arrays are READ-FIRST, and the read that feeds a
    // resolve was issued two cycles earlier".  The second clause is true;
    // the first is a claim about SILICON that cannot be supported, and it
    // was doing load-bearing work in the argument.  READ_FIRST is a BRAM
    // WRITE_MODE.  The data array is URAM, which has no write-mode
    // attribute at all (UG573 ch.2: "there are no user definable
    // read-first, write-first, no-change modes with UltraRAM"), and for the
    // tag array's shape -- simple dual port, independent read and write
    // addresses -- UG901's RW_ADDR_COLLISION entry says Vivado defaults to
    // WRITE_FIRST "for best timing" and calls the collision output
    // "unpredictable".  Verilog gives read-first for free, so no test we
    // can write would ever have caught the design depending on it.
    //
    // The real argument, per stage, is:
    //
    //   stage Q's occupant -- its array access edge was one cycle EARLIER
    //       than this write.  Already happened; nothing to collide with.
    //       (Its `dout <= dout_r1` lands on the same edge, but that is a
    //       register copy, not a memory access.)
    //   stage A's occupant -- its array access edge IS this write's edge.
    //       A GENUINE same-cycle collision, and the only thing that makes
    //       it safe is `set_haz_c` below: that occupant was accepted one
    //       cycle ago and checked against this very request, so the sets
    //       cannot match -- except through the same-line-read-pair
    //       exemption, and a read leader writes NEITHER tag NOR data.  It
    //       does write PLRU, and that collision IS consumed: harmlessly,
    //       because PLRU is a replacement hint and way-busy is separately
    //       enforced by `busy_mask` in l2c_victim_sel.
    //   the MSHR's install/replay writes -- asynchronous to this pipeline,
    //       so they DO land on a reader's access edge.  Covered by
    //       `skew_hazard_c`, which RE-READS rather than resolving; the
    //       2-deep `inst_hist` window provably spans the access edge (an
    //       install writing at the edge a request accepted at T takes its
    //       data read sets inst_hist_v0 on that same edge, which shifts to
    //       inst_hist_v1 and is visible at that request's resolve).
    //
    // So the protection is the accept-time interlock plus the re-read, not
    // the array semantics -- and `make tb-l2c-collision` is the standing
    // proof, rebuilding both arrays so a collided read returns poison (or
    // the new data) and requiring the whole suite to pass anyway.
    //
    // The rule is not "same set stalls".  A 64 B burst is four beats of
    // ONE line and one set, so a blanket same-set interlock would leave
    // `hit_read_64B_burst` at 3 cycles/beat and the whole exercise would
    // be pointless.  What is actually safe to overlap is a SAME-LINE READ
    // PAIR, and nothing else in the same set:
    //
    //   same line, both reads   -- OK.  A read hit writes no tag and no
    //       data (only PLRU), so the follower's view of its own line is
    //       current.  If the leader MISSED, it wrote a placeholder tag
    //       (tag=leader's, vsec=0) that the follower cannot see -- but the
    //       follower is the same line, so `mshr_lu_hit` (live MSHR state,
    //       not an array read) is true and it takes the merge path.
    //   same set, different line -- BLOCKED.  This is the one that bites:
    //       a leader miss that EVICTS way W leaves the follower's stale
    //       view showing W still holding the evicted tag.  A follower
    //       write would then reinstate that tag over the placeholder and
    //       land its data in the wrong line; a follower read with the
    //       evicted tag would allocate a SECOND MSHR entry onto a way that
    //       is already claimed (`way_ok_c` takes the `any_tm_c` shortcut
    //       and never consults `mshr_bw_mask`).  Both are silent
    //       corruption.
    //   either side a write     -- BLOCKED, even same-line: the follower
    //       would read the data array before the leader's line install
    //       landed and answer with pre-write data.
    //
    // Cost: two 12-bit set compares and two 14-bit tag compares in the
    // accept cone.  Paid for by making the per-beat address a register
    // (see aw_cur/ar_cur above), which removed a 32-bit adder and a
    // barrel shifter from the same cone.
    //
    // 2026-09-15 (array-address pipelining): the two stages checked here
    // are now A and Q, NOT Q and stage 2.  The rule is unchanged -- "check
    // every older request whose array WRITE can still land at or after my
    // array READ" -- but the array read moved one cycle later (it is now
    // issued from `ra_q`, the cycle AFTER accept), so the window slid by
    // one stage:
    //
    //   stage 2's occupant  -- NOT checked.  do_accept_c requires
    //       accept_slot_c, which requires p2_free_c, which for an occupied
    //       stage 2 means p2_done_c: it RESOLVES on this very edge.  Its
    //       write therefore lands at edge T->T+1 and the accepted
    //       request's read is at edge T+1->T+2, one full edge later, so it
    //       sees the write.  This is exactly the invariant the rigid shift
    //       buys, and it is checked in simulation (see the
    //       "accept implies stage 2 retires" assertion below).
    //   stage Q's occupant  -- CHECKED.  It reaches stage 2 at T+1 and
    //       resolves at T+1 at the earliest, writing on edge T+1->T+2 --
    //       the SAME edge as the new request's read.  Both arrays are
    //       one cycle EARLIER than that write, so it is invisible -- it
    //       already happened, which is a property of the pipeline, not of
    //       the array.  Later resolves are worse.
    //   stage A's occupant  -- CHECKED.  It cannot resolve before T+2, so
    //       its write is strictly after the new request's read.
    //
    // Count and shape are identical to before (two 12-bit set compares and
    // two 14-bit tag compares); only which two registers feed them changed.
    // The same-line-read-pair exception carries over verbatim: a read hit
    // writes no tag and no data, and a leader that MISSED is caught by
    // `mshr_lu_hit` (live MSHR state, not an array read) at the follower's
    // own resolve.
    wire a_same_set_c   = a_v && (a_set == cur_set);
    wire q_same_set_c   = q_v && (q_set == cur_set);
    wire a_same_line_c  = a_same_set_c && (a_tag == cur_tag);
    wire q_same_line_c  = q_same_set_c && (q_tag == cur_tag);
    wire set_haz_c = (a_same_set_c && !(a_same_line_c && !a_is_write && !write_sel)) ||
                     (q_same_set_c && !(q_same_line_c && !q_is_write && !write_sel));
    // Response-ordering hazard for a BYPASS dispatch: the pipeline holds
    // ops whose ids are not in the MSHR yet, so `mshr_idq_busy` alone no
    // longer sees them. Widened with the {is_fetch, id} tag for the same
    // reason idq_id/idq2_id/alloc_id/merge_id all are (see the Level A
    // task #269 comment above `id_busy_c`): axi_i and axi_d are
    // independently-numbered AXI masters, so an unwidened q_id/req_id_w
    // compare treats a same-numeric-ID fetch op and LSU op as the same
    // occupant. A continuous fetch stream sharing a bypass op's numeric ID
    // then keeps `pipe_id_haz_c` permanently true for that bypass op --
    // real, reproducible indefinite starvation, not a bounded stall,
    // because the pipeline is never actually vacated by unrelated fetch
    // traffic. This is exactly the Q700 ROM's inhibited/open-bus probe
    // shape while instruction fetch remains active (tb_l2c.cpp
    // fetch_id0_does_not_starve_lsu_bypass_id0).
    // 2026-09-15: three stages now, and ALL THREE are checked -- unlike
    // set_haz_c above, stage 2 is NOT excused here.  set_haz_c is about an
    // array WRITE landing before an array READ, which the rigid shift
    // orders for us; this is about a RESPONSE ordering against an op that
    // has not answered yet.  A stage-2 occupant that resolves on this very
    // edge into a MISS has its response come from l2c_mshr much later, and
    // its MSHR entry does not exist until that edge -- so `mshr_idq_busy`
    // cannot see it this cycle either.  Dropping the stage-2 term would
    // open exactly the window Critical-3 exists to close.
    assign pipe_id_haz_c = (a_v   && (a_id     == cur_id) && (a_is_fetch   == cur_is_fetch)) ||
                           (q_v   && (q_id     == cur_id) && (q_is_fetch   == cur_is_fetch)) ||
                           (req_v && (req_id_w == cur_id) && (req_is_fetch == cur_is_fetch));
    // s_wready must track do_accept_c's WRITE-side readiness EXACTLY
    // (every accept term included) -- a blanket "aw_have && !ar_have"
    // proxy was only valid back when reads had fixed absolute priority; it
    // broke silently under Important-11's rw_favor fix, which lets
    // write_sel win a tie while ar_have is still 1.  Any mismatch here
    // means a W-beat handshake the front door never actually captured.
    assign s_wready = flw_gath_rdy_c ? 1'b1
                    : (write_sel && !flw_avail_c && !rst_busy &&
                       (is_bypass_c ? (byp_slot_c && byp_req_ready && !id_busy_c && !pipe_id_haz_c)
                                    : (accept_slot_c && !set_haz_c)));
    wire do_accept_c = !rst_busy && any_sel && !is_bypass_c && accept_slot_c && !set_haz_c;

    // 2026-07-24: real-HW investigation — boot_fsm's first write into the
    // L2C-fronted DDR path appears to hang (SD-loader LED flashes briefly
    // then stops; s0_wready reads permanently 0 via VIO). Bundle the
    // front-door accept state so a debug_ila (ENABLE_ILA=1) capture can
    // show exactly where the very first AW/W handshake stalls. Leading
    // theory: l2c.v/l2c_ctrl.v have NO dependency on ddr_cal_done
    // anywhere (grepped, zero hits) — L2C's reset is plain
    // core_rst_bank[4], same as everything else, and its own reset-walk
    // finishes in ~4096 cycles, long before real MIG calibration
    // completes. It likely opens its front door and accepts boot_fsm's
    // first write before DDR is actually ready, then hangs waiting for
    // a B-response from a MIG path that isn't calibrated yet.
    assign dbg_write_snap = {
        s_awvalid,     // [10]
        s_awready,     // [9]
        s_wvalid,      // [8]
        s_wready,      // [7]
        aw_have,       // [6]
        rst_busy,      // [5]
        id_busy_c,     // [4]
        write_avail,   // [3]
        write_sel,     // [2]
        req_v, q_v     // [1:0] lookup-pipeline occupancy (was `st`)
    };
    wire do_bypass_c = !rst_busy && any_sel && is_bypass_c && byp_req_ready &&
                       byp_slot_c && !id_busy_c && !pipe_id_haz_c;
    wire door_take_c = do_accept_c || do_bypass_c;
    // Level A (task #269): scoped to lsu_read_sel, NOT the broader
    // read_sel -- ar_take_c drives the LSU-only ar_beat/ar_cur/ar_have
    // advance in the two-deep-door promotion block above (ar_act_free_c /
    // "else if (ar_take_c)").  A blanket read_sel here would spuriously
    // advance the LSU's own burst bookkeeping on a cycle where a
    // fetch-sourced beat, not an LSU one, actually won the door.
    assign ar_take_c = door_take_c && lsu_read_sel;
    assign aw_take_c = flw_gath_c || (door_take_c && !read_sel && !flw_avail_c);
    reg [ADDR_WIDTH-1:0] req_addr; reg [SET_BITS-1:0] req_set; reg [TAG_BITS-1:0] req_tag;
    reg req_is_write; reg [ID_WIDTH-1:0] req_id; reg [127:0] req_wdata; reg [15:0] req_wstrb;
    reg [1:0] req_qoff; reg req_last; reg req_need; reg req_illegal;
    // Level A (task #269): source tag for THIS accepted request, latched
    // alongside req_is_write at accept time.  Consumed only for response
    // routing (hit_rsp_is_fetch/mshr_rsp_is_fetch below) and the MSHR
    // id-source qualification -- never read by hit-detection/eviction.
    reg req_is_fetch;
    assign req_id_w = req_id;
    // This request is a whole 64 B line with every strobe set: it needs no
    // fill and its data comes from `req_line`, not req_wdata.
    //
    // 2026-08-20 (lookup pipelining): the line USED to stay in flw_data,
    // with gst held at G_FULL for the request's whole life so the next
    // line's gather could not overwrite it.  That serialised the gather
    // against the lookup and pinned `stream_write_64B` at 4 gather cycles
    // + 3 lookup cycles per line.  The assembled line is now COPIED into
    // the pipeline at accept (q_line -> req_line) and gst is released
    // immediately, so the next line gathers while this one resolves.  Cost
    // 2 x 512 FF; the 512 b dw_data mux it feeds already existed.
    reg req_full; reg [LINE_BITS-1:0] req_line;
    // Per-quadrant masks for THIS request, all decided at accept time so
    // none of them is in the S_LOOKUP resolve cone:
    //   req_cov  -- quadrants whose every byte this request supplies, so
    //               they can be installed valid with no fill at all.
    //   req_dty  -- quadrants this request modifies at all (a partial
    //               strobe dirties its quadrant without covering it).
    //   req_want -- quadrants this request needs to READ and cannot
    //               supply, i.e. what a fill would actually be for.
    reg [3:0] req_cov, req_dty, req_want;
    wire       cur_qfull_c = &cur_wstrb;
    wire       cur_qany_c  = |cur_wstrb;
    wire [3:0] cur_cov_c   = (gst == G_FULL) ? 4'b1111
                           : (cur_qfull_c ? (4'b0001 << cur_qoff) : 4'b0000);
    wire [3:0] cur_dty_c   = (gst == G_FULL) ? 4'b1111
                           : (cur_qany_c  ? (4'b0001 << cur_qoff) : 4'b0000);
    wire [3:0] cur_want_c  = (!read_sel && ((gst == G_FULL) || cur_qfull_c))
                           ? 4'b0000 : (4'b0001 << cur_qoff);
    // ======================= ARRAY ADDRESS REGISTER ======================
    //
    // 2026-09-15 (FMax).  This is the whole point of stage A.
    //
    // WHAT WAS MEASURED.  On the routed 200 MHz eth build (post-route
    // timing_20260914_222410.rpt) u_l2c owned 56 of 250 violated setup
    // paths, and 41 of those 56 ended on a URAM288 `ADDR_A[n]` pin of
    // u_data -- the array read address.  The worst L2C-internal path, at
    // -0.684 ns, was
    //
    //   u_mshr/m_set_reg[1][5]/C  ->  mshr_bw_mask -> victim_sel_way ->
    //   need_evict_c -> way_ok_c -> s_lookup_miss_go -> p2_done_c ->
    //   do_accept_c  ->  <the mux LUT below>  ->  u_data/g_way[4].
    //   mem_reg_uram_5/ADDR_A[9]
    //
    // 5.389 ns of data path, of which 4.119 ns (76%) is ROUTE and 1.270 ns
    // is logic, over 16 levels.  The last two hops alone are 0.897 ns (the
    // fo=913 accept-select net out to the replicated mux LUTs) plus
    // 0.682 ns (that LUT to the URAM pin) = 1.579 ns of pure distance
    // sitting BEHIND the entire resolve cone.
    //
    // WHY max_fanout DID NOT FIX IT, AND IS NOW REMOVED.  The old
    // `(* max_fanout = 10 *)` did what it was asked: the placer put a copy
    // of the address mux in each way's URAM column.  But replicating a
    // COMBINATIONAL driver moves the long wire from after the LUT to
    // before it -- the copies' INPUTS (do_accept_c and the sets) still come
    // from the middle of the die.  That is the 0.897 ns fo=913 hop above.
    // A wire cannot be shortened by copying the gate on the far end of it.
    //
    // WHAT A REGISTER DOES INSTEAD.  ra_q is ONE register, placed by the
    // placer at the END of the resolve cone, where its D input is a short
    // local route.  Its Q then has a WHOLE CLOCK PERIOD, with zero logic in
    // it, to cross the die to 64 URAM288s and 9 BRAMs: ~0.08 ns clk-to-Q
    // plus the ~1.3 ns of distance the arrays' span costs, against 5.0 ns.
    // The path that used to fail becomes two paths, each with slack.
    //
    // NO max_fanout HERE, DELIBERATELY.  Replicating ra_q would re-create
    // the original defect one gate earlier: each copy's D would then be the
    // long route, and it would be in series with the resolve cone again.
    // One central register, one long register-to-array net, is the shape
    // that works.  (`arr_wset_c` below keeps its max_fanout -- it is fed
    // from registers, not from the resolve cone, so replication there is
    // still the right answer.)
    //
    // SELECTION.  Steady state ra_q takes the set being accepted.  When
    // nothing is accepted it HOLDS, which keeps stage A's set standing on
    // the pins -- the arrays are clock-enabled (ce_c) rather than
    // re-addressed, because with a registered address "re-issue the set I
    // still need" would cost a cycle instead of being free.  The skew re-
    // read sequence (rr_start_c/rr_p1/rr_p2) outranks everything; see the
    // rr block below for its three-cycle address schedule.
    reg [SET_BITS-1:0] ra_q;
    wire [SET_BITS-1:0] tags_raddr = ra_q;
    // -- Tags/data/PLRU arrays ---------------------------------------------
    wire [7:0] rvalid; wire [31:0] rvsec, rdsec; wire [8*TAG_BITS-1:0] rtag; wire [6:0] rplru;
    wire mshr_inst_valid; wire [3:0] mshr_inst_vsec, mshr_inst_dsec;
    wire [SET_BITS-1:0] mshr_inst_set; wire [WAY_BITS-1:0] mshr_inst_way;
    wire [TAG_BITS-1:0] mshr_inst_tag; wire [LINE_BITS-1:0] mshr_inst_data; wire [63:0] mshr_inst_strb;
    wire tw_en, dw_en, pw_en;
    wire [SET_BITS-1:0] tw_set, dw_set, pw_set; wire [WAY_BITS-1:0] tw_way, dw_way;
    wire [3:0] tw_vsec, tw_dsec; wire [TAG_BITS-1:0] tw_tag;
    wire [LINE_BITS-1:0] dw_data; wire [63:0] dw_strb; wire [6:0] pw_data;
    // Critical-7: 2-deep {valid,set} history of the MSHR install pulse (see header).
    reg inst_hist_v0, inst_hist_v1; reg [SET_BITS-1:0] inst_hist_set0, inst_hist_set1;
    always @(posedge clk) begin
        if (rst) begin inst_hist_v0 <= 1'b0; inst_hist_v1 <= 1'b0; end
        else begin
            inst_hist_v1 <= inst_hist_v0; inst_hist_set1 <= inst_hist_set0;
            inst_hist_v0 <= mshr_inst_valid; inst_hist_set0 <= mshr_inst_set;
        end
    end
    wire inst_hist_hit_c = (inst_hist_v0 && (inst_hist_set0 == req_set)) ||
                           (inst_hist_v1 && (inst_hist_set1 == req_set));
    //
    // STALE-SNAPSHOT STICKY (2026-09-15).  inst_hist alone is NOT sufficient
    // any more, and the reason is the single most subtle consequence of
    // putting the read address in a register.  It cost a full randomized-
    // scoreboard run to find, so it is written down at length.
    //
    // inst_hist is a fixed 2-deep window, and it is exact only if every
    // request's array snapshot is at most 2 cycles old when it is resolved
    // -- or, where it is older, if the hazard is re-evaluated on every
    // intervening cycle so the rolling window never skips one.  Stage 2
    // satisfies the second form: it IS re-evaluated every cycle it is live,
    // so however long it stalls, every install cycle is seen by at least
    // one evaluation.
    //
    // Stage Q satisfies NEITHER, and it used to satisfy the first.  Before
    // this change the arrays' first output register free-ran and the read
    // address was combinational, so a stalled pipeline re-issued stage 1's
    // own set every single cycle: its snapshot was continuously refreshed
    // and was never more than one cycle old when it was finally clocked
    // into the resolve stage.  With a registered address that re-issue
    // would cost a cycle instead of being free, so the read pipeline is
    // clock-enabled instead -- and a clock-enabled first register HOLDS.
    // A request that sits in stage Q for N stalled cycles therefore carries
    // a snapshot that is N cycles old into the resolve stage, where a
    // 2-deep window cannot see the installs it missed.  Nothing evaluates a
    // hazard for stage Q in the meantime, because stage Q has no resolve.
    //
    // The fix is to make the hazard STICKY across exactly the stages that
    // cannot re-evaluate it, instead of widening a window that can never be
    // wide enough (the stall is unbounded).  `q_stale`/`req_stale` record
    // "an MSHR install has landed on my set since my snapshot was read":
    //
    //   * SET, not accumulated, on the A->Q edge -- that edge IS the array
    //     read, and both arrays are read-first, so an install in the same
    //     cycle writes too late for it and is blind.  Installs while the
    //     request merely WAITED in stage A are not blind: its read had not
    //     happened yet, so it sees them.
    //   * ACCUMULATED for every cycle the request sits in stage Q, which is
    //     the window inst_hist cannot bound.
    //   * carried into `req_stale` on the Q->S2 edge and accumulated there
    //     too, which makes stage 2's own coverage independent of the
    //     rolling-window argument as well.
    //   * RESET by the skew re-read, which is the only thing that gives the
    //     request a fresh snapshot: stage 2's is re-read on edge C+1->C+2
    //     and stage Q's is restored on edge C+2->C+3, so each resets on its
    //     own edge and captures the install that is blind to it.
    //
    // inst_hist is KEPT -- but ⚠ NOT for the reason recorded here until
    // 2026-09-18, which said it "is not redundant belt-and-braces: it is
    // what covers the resolve stage across the re-read, where req_stale has
    // just been cleared".  That does not survive inspection: `rr_p1`
    // reloads `req_stale <= inst_on_r_c` on the re-read cycle and the
    // accumulate below re-arms it on every later install, so the re-read's
    // own window is already covered by req_stale alone.  Measured to match:
    // with `skew_hazard_c = req_stale` (inst_hist removed entirely) the
    // directed suite and 320k randomized ops across four seeds all pass.
    //
    // What inst_hist actually covers is the OTHER end -- an install that
    // landed before this request reached stage 2 at all, i.e. against the
    // A and Q cycles, which reach req_stale only through the q_stale chain
    // on the Q->S2 edge.  It is a second, overlapping path to the same
    // conclusion.  Keeping it is cheap and conservative; what is NOT
    // acceptable is the previous situation, where the only written
    // justification for a load-bearing-looking guard was wrong and no test
    // distinguished it either way.  If it is ever removed, the thing to
    // re-derive is the A/Q coverage, not the re-read.  In the no-stall case the two are the same condition
    // cycle for cycle, so this adds no re-reads to the steady state
    // (measured: 450 -> 404 over the tb's 1.55 M cycles).
    reg q_stale, req_stale;
    wire inst_on_a_c = mshr_inst_valid && (mshr_inst_set == a_set);
    wire inst_on_q_c = mshr_inst_valid && (mshr_inst_set == q_set);
    wire inst_on_r_c = mshr_inst_valid && (mshr_inst_set == req_set);
    wire skew_hazard_c = inst_hist_hit_c || req_stale;
    // -- Array output pipeline (the thing that makes the stages line up) ---
    //
    // l2c_tags is 1-cycle, l2c_data is 2-cycle.  The old FSM papered over
    // the difference by holding raddr stable for both of its non-accept
    // cycles.  A pipeline cannot: raddr moves every cycle.  So the tag
    // output gets ONE more register here (tq_*), which puts it on the same
    // cycle as the data array's own second output register.
    //
    // 2026-09-15 (array-address pipelining).  `ce_c` is now a clock enable
    // on EVERY array output register -- tq_* here, l2c_tags's own internal
    // dout/plru_dout, and BOTH of l2c_data's.  It used to gate only the
    // second stage, with the first left free-running and held instead by
    // re-pointing a combinational read address at the stalled stage's own
    // set.  That trick dies with a registered address: re-issuing a set
    // now costs a cycle rather than being free.  Clock-enabling the whole
    // array read pipeline is the direct replacement, and it is exactly as
    // cheap in NET COUNT because the rigid shift (see accept_slot_c) makes
    // one enable correct for all of them -- l2c_data still sees one
    // control net reaching its 64 URAMs, not two.
    //
    // The Critical-7 window is UNCHANGED at 2 deep, and that is not an
    // accident of this change but a property of where the new register
    // went.  inst_hist has to cover the cycles in which an MSHR install is
    // BLIND to a request -- the cycles from that request's array read edge
    // up to its resolve.  A request accepted at T now presents its address
    // at T+1 (ra_q), reads on edge T+1->T+2, and resolves at T+3: blind
    // cycles T+1 and T+2, which is two, exactly as when it was accepted at
    // T, read on edge T->T+1 and resolved at T+2.  The extra cycle was
    // spent BEFORE the array, not after it.  Registering the array output
    // instead would have grown this window; registering the input does not.
    //
    // `rr` is the skew-hazard re-read: stage 2 stays put while the arrays
    // are re-read for its own set, and is not allowed to resolve until the
    // fresh view has arrived.  It replaces the old S_LOOKUP -> S_WAIT
    // bounce, which has no meaning once S_WAIT holds a DIFFERENT request.
    // A flush was rejected: the front door's beat counters have already
    // advanced past this beat, so re-injecting it would mean unwinding
    // aw_beat/ar_beat/aw_have/ar_have; and forwarding was rejected because
    // it would put a {set,way,tag,vsec,dsec} compare AND a 512 b
    // strobe-merge mux into the resolve cone, which is the design's
    // tightest.
    //
    // It is now a THREE-cycle sequence (was two), because the address it
    // hijacks is a register and because the re-read has to give stage Q's
    // array output back afterwards -- the first output register is the
    // only place that output lives, and the re-read borrows it.  Cycle by
    // cycle, with rr_start_c at C:
    //
    //   C     ra_q <= req_set.  Pipeline frozen (p2_free_c is 0 while
    //         stage 2 cannot resolve), ce_c low, so stage A's read result
    //         is simply discarded -- its ADDRESS is still in a_set and is
    //         handed back at C+2, which is why discarding it is free.
    //   C+1   pins = req_set.  ce_c high: the first register takes req's
    //         fresh line/tags; the second takes whatever the first held
    //         (stage Q's), which is junk for stage 2 and is overwritten
    //         next cycle -- harmless, and it is what lets ONE enable serve
    //         both registers.  ra_q <= q_set.
    //   C+2   pins = q_set.  ce_c high: the second register takes req's
    //         fresh view (this is the one that counts), the first takes
    //         stage Q's output back.  ra_q <= a_set.
    //   C+3   stage 2 resolves against a snapshot read on edge C+1->C+2;
    //         stage Q's output is restored; stage A's set is back on the
    //         pins.  Normal operation, no bubble.
    //
    // Blind-install window for that fresh snapshot: C+1 and C+2, so the
    // same 2-deep inst_hist covers the re-read too, and the install that
    // triggered it (at C-1 or C-2) has aged out, so it cannot re-fire on
    // its own cause.  Cost 3 cycles on an event measured at 450 in a
    // 1.55 M-cycle tb run.
    reg rr_p1, rr_p2;
    assign rr_busy_c = rr_p1 || rr_p2;
    wire ce_c = pipe_adv_c || rr_busy_c;
    reg [7:0] tq_rvalid; reg [31:0] tq_rvsec, tq_rdsec;
    reg [8*TAG_BITS-1:0] tq_rtag; reg [6:0] tq_rplru;
    always @(posedge clk) begin
        if (ce_c) begin
            tq_rvalid <= rvalid; tq_rvsec <= rvsec; tq_rdsec <= rdsec;
            tq_rtag <= rtag; tq_rplru <= rplru;
        end
    end
    // Shared array WRITE address.  tw_set, dw_set and pw_set are all
    // `req_set` and the MSHR-install override is the same `mshr_inst_set`
    // for all three, so this was already one net electrically -- naming it
    // makes that explicit and gives the MAX_FANOUT above (same 64-URAM
    // reach, same reason) something to hang on.
    (* max_fanout = 10 *)
    wire [SET_BITS-1:0] arr_wset_c = mshr_inst_valid ? mshr_inst_set : tw_set;
    // ---------------- ONE-HOT ARRAY WRITE ENABLES (2026-09-15, FMax) -------
    //
    // The arrays used to take (wr_en, wr_way) and decode `wr_en && (wr_way
    // == w)` themselves, which put THREE logic levels between the resolve
    // decision and a WEA/write-enable pin: tw_en itself, the
    // install-vs-resolve wr_en mux, and the way compare.  Measured on the
    // routed netlist as the last three hops of the worst L2C-internal path
    // (-0.656 ns): 0.241 + 0.605 + 0.380 = 1.226 ns, over 6 of the 8 tag
    // BRAMs' physical span.
    //
    // Decoded here instead, it is ONE level.  The decode itself is NOT on
    // the critical path, and that is the point rather than a coincidence:
    //   * `mshr_inst_way_oh_c` comes straight off l2c_mshr's install
    //     registers;
    //   * `sel_way_c` is one level SHORTER than tw_en is (tw_en is
    //     `line_install_c || (s_lookup_miss_go && !any_tm_c)`, i.e. one
    //     level past the same cone sel_way_c ends), so its decode lands in
    //     the same cycle slot the old way-compare occupied and the final
    //     LUT has tw_en as its LATEST input -- exactly what we want it to
    //     be, since that is the signal we are shortening.
    // The broadcast clear folds in here too (5 inputs, one LUT6).
    //
    // This also gives the placer EIGHT independent enable drivers, one per
    // way, each with a single array as its load -- the thing `max_fanout`
    // on a combinational driver could not deliver, because here the copies
    // differ (they are a decode, not a replication) and their shared input
    // is a single-bit control, not a 12-bit bus.
    // (tw_way and dw_way are both `sel_way_c`; they are used here rather
    // than sel_way_c itself only because they are forward-declared above
    // and sel_way_c is not -- the arrays are instantiated ahead of the
    // resolve body that computes it.)
    wire [7:0] tw_way_oh_c         = 8'b1 << tw_way;
    wire [7:0] dw_way_oh_c         = 8'b1 << dw_way;
    wire [7:0] mshr_inst_way_oh_c  = 8'b1 << mshr_inst_way;
    wire [7:0] tags_wen_oh_c, data_wen_oh_c;
    genvar gwoh;
    generate
        for (gwoh = 0; gwoh < 8; gwoh = gwoh + 1) begin : g_wenoh
            assign tags_wen_oh_c[gwoh] = rst_clr_en ||
                   (mshr_inst_valid ? mshr_inst_way_oh_c[gwoh]
                                    : (tw_en && tw_way_oh_c[gwoh]));
            assign data_wen_oh_c[gwoh] =
                   (mshr_inst_valid ? mshr_inst_way_oh_c[gwoh]
                                    : (dw_en && dw_way_oh_c[gwoh]));
        end
    endgenerate
    l2c_tags #(.SETS(4096), .SET_BITS(SET_BITS), .WAYS(8), .WAY_BITS(WAY_BITS), .TAG_BITS(TAG_BITS)) u_tags (
        .clk(clk), .raddr(tags_raddr), .rd_en(ce_c),
        .rvalid(rvalid), .rvsec(rvsec), .rdsec(rdsec), .rtag(rtag),
        .wr_wen_oh(tags_wen_oh_c), .wr_set(arr_wset_c),
        .wr_vsec(mshr_inst_valid ? mshr_inst_vsec : tw_vsec),
        .wr_dsec(mshr_inst_valid ? mshr_inst_dsec : tw_dsec), .wr_tag(mshr_inst_valid ? mshr_inst_tag : tw_tag),
        .clr_en(rst_clr_en), .clr_set(rst_clr_set),
        .rplru(rplru), .plru_wr_en(pw_en), .plru_wr_set(pw_set), .plru_wr_data(pw_data)
    );
    wire [8*LINE_BITS-1:0] rdata;
    l2c_data #(.SETS(4096), .SET_BITS(SET_BITS), .WAYS(8), .WAY_BITS(WAY_BITS),
               .LINE_BITS(LINE_BITS), .LINE_BYTES(64)) u_data (
        .clk(clk), .raddr(tags_raddr), .rd_en(ce_c), .rdata(rdata), .wr_wen_oh(data_wen_oh_c),
        .wr_set(arr_wset_c),
        .wr_data(mshr_inst_valid ? mshr_inst_data : dw_data), .wr_strb(mshr_inst_valid ? mshr_inst_strb : dw_strb)
    );
    // -- S_LOOKUP resolve: hit/miss classify --------------------------------
    // Rule 1 (see header).  way_hit_c = tag match AND the requested
    // quadrant is valid -> a real hit, drives the response.  tag_match_c =
    // tag match AND the line has ANY valid quadrant -> this way OWNS this
    // address, drives way selection so a partially-valid line is never
    // duplicated into a second way.  Both are one LUT level past the tag
    // compare's partial products: comparing TAG_BITS=14 bits needs five
    // LUT6 partials, leaving a free input on the level-2 LUT for the
    // quadrant-valid select (4 data + 2 select = one LUT6, computed in
    // parallel with level 1) and for the line-valid reduce.
    wire [7:0] way_hit_c, tag_match_c; genvar gwv;
    generate
        for (gwv = 0; gwv < 8; gwv = gwv + 1) begin : g_hitchk
            wire [3:0] vsec_w = tq_rvsec[gwv*4 +: 4];
            wire       tagm_w = (tq_rtag[gwv*TAG_BITS +: TAG_BITS] == req_tag);
            assign tag_match_c[gwv] = tq_rvalid[gwv]   && tagm_w;
            assign way_hit_c[gwv]   = vsec_w[req_qoff] && tagm_w;
        end
    endgenerate
    wire any_hit_c; wire [2:0] hit_way_c;
    l2c_pri8 u_hitpri (.req(way_hit_c), .hit(any_hit_c), .idx(hit_way_c));
    wire any_tm_c; wire [2:0] tm_way_c;
    l2c_pri8 u_tmpri (.req(tag_match_c), .hit(any_tm_c), .idx(tm_way_c));
    wire mshr_lu_hit; wire [2:0] mshr_lu_idx;
    wire mshr_alloc_ready, mshr_merge_ready; wire [7:0] mshr_bw_mask, mshr_bwq_mask;
    // ====================================================================
    // VICTIM PRECOMPUTE -- THE HEAD OF THE TRUNK (2026-09-16, FMax)
    // ====================================================================
    //
    // WHAT WAS WRONG.  On the routed 200 MHz SoC netlist the design's worst
    // path, and 884 of its 5429 failing endpoints, was
    //
    //   req_set_reg[5]/C -> u_mshr <6 levels> -> mshr_bw_mask
    //     -> <4 levels> -> victim_sel_way -> <4 levels> -> pipe_adv_c
    //     -> u_data/rd_en -> g_way[3].mem_reg_uram_3/EN_A
    //
    //   -0.315 ns, 15 levels, 4.747 ns of data path of which 76% is ROUTE.
    //
    // Segment by segment, measured:
    //     0.412 ns  req_set out to the MSHR CAM
    //     1.332 ns  the 8-entry set CAM -> bw_mask          (6 levels)
    //     0.785 ns  bw_mask -> victim_sel_way               (4 levels)
    //     0.782 ns  -> need_evict_c -> way_ok_c
    //               -> s_lookup_miss_go -> p2_done_c -> pipe_adv_c
    //     1.436 ns  pipe_adv_c -> the fo=194 array-enable net -> EN_A
    //
    // The 2026-09-15 array-address pipelining took the array ADDRESS out of
    // this cone (`ra_q`) and the AXI slices took the crossbar's write FSM
    // out of it, which is why the same path measured -0.684 ns before them
    // and -0.315 ns after.  But the SHAPE did not change: the
    // `bw_mask -> victim_sel_way -> need_evict_c -> way_ok_c ->
    // s_lookup_miss_go -> p2_done_c` trunk appears character for character
    // in the -0.684 ns path and in the -0.315 ns one.  Only the endpoint
    // moved, from a URAM ADDR_A pin to a URAM EN_A pin.  The head of the
    // trunk -- 2.5 ns of the 4.7 ns, from `req_set` to `victim_sel_way` --
    // had never been touched.
    //
    // WHY THE TAIL CANNOT BE FIXED AND THE HEAD CAN.  The 1.436 ns tail is
    // one LUT and a 194-load net to 64 URAM288s and 9 BRAMs.  Replicating
    // that LUT does not help, for exactly the reason the ARRAY ADDRESS
    // REGISTER block above records: replicating a COMBINATIONAL driver
    // moves the long wire from after the gate to before it, and the copies'
    // shared input (`pipe_adv_c`) is the LATEST signal in the cone.  A wire
    // cannot be shortened by copying the gate on the far end of it.  Nor
    // can the enable be registered the way the address was: a clock enable
    // is, by definition, a decision made inside the cycle it acts on.
    //
    // The head, by contrast, is precomputable, because the victim choice
    // does not depend on anything that is only known at stage 2:
    //
    //   * `rvalid` / `rplru` / `rdsec` -- l2c_tags is a ONE-cycle array and
    //     l2c_ctrl re-registers it into `tq_*` to line it up with l2c_data's
    //     two.  So the tag array's output for a request is live on
    //     `rvalid`/`rplru`/`rdsec` during the cycle that request sits in
    //     stage Q, one full cycle before it resolves.
    //   * the busy-way mask -- live MSHR state, queryable for any set.
    //
    // So the victim is chosen at stage Q and REGISTERED into stage 2.  This
    // is the same move the array address made, and the prior block's own
    // statement of why it works applies verbatim: one register, placed by
    // the placer at the end of its cone, whose Q then has a whole clock
    // period with zero logic in it.  What is left in the cycle is
    //
    //   victim_sel_way_q/C -> need_evict_c -> way_ok_c
    //     -> s_lookup_miss_go -> p2_done_c -> pipe_adv_c -> rd_en -> EN_A
    //
    // i.e. the 0.782 ns + 1.436 ns tail alone, ~2.4 ns against 5.0 ns.
    //
    // ZERO LATENCY COST.  Nothing moved between pipeline stages: stage Q
    // was already spending that cycle waiting for l2c_data's second output
    // register, with the tag array's answer sitting idle beside it.  The
    // precompute fills dead time.  Hit latency, miss latency, the accept
    // rate and the re-read schedule are all unchanged.
    //
    // ---- TWO CANDIDATES, NOT A MUX IN FRONT OF ONE -------------------
    //
    // Which set the next cycle's stage-2 occupant will have depends on
    // `pipe_adv_c`, which is the late signal this whole change exists to
    // get away from -- putting it in front of the CAM would move the defect
    // rather than remove it.  Both candidates are therefore computed in
    // full, in parallel, and `pipe_adv_c` selects between them as the last
    // gate before the register: ONE LUT past a signal that already has
    // ~1.6 ns of slack at that point.
    //
    //   Candidate Q  -- stage Q's request advances in (pipe_adv_c = 1).
    //   Candidate R  -- stage 2 keeps its occupant  (pipe_adv_c = 0).
    //
    // Candidate R is not an optimisation, it is a DEADLOCK FIX.  A victim
    // registered once on entry and never refreshed goes stale against the
    // MSHR: if every way of the set was claimed at that instant,
    // `victim_ok` is 0, the miss cannot resolve, the pipeline cannot
    // advance, and the register can never reload -- while the entries that
    // made it 0 complete their fills and free (an install does not need the
    // front door).  The cache wedges permanently.  Candidate R reloads the
    // register every cycle stage 2 stalls, so a freed way is visible one
    // cycle later and the stall is bounded.
    //
    // ---- WHY A ONE-CYCLE-OLD MASK IS SAFE ----------------------------
    //
    // The registered mask is always one cycle behind the MSHR, so soundness
    // needs: no way may become BUSY between the cycle the victim is chosen
    // and the cycle it is used.  (The other direction -- a way that has
    // FREED still showing busy -- only costs a candidate, and is what
    // candidate R bounds.)
    //
    // Ways become busy only through `alloc_valid_c`, and stage 2 is the
    // only allocator in the design (l2c_bypass never touches the MSHR, and
    // a MERGE joins an existing entry without claiming a new way).  Once a
    // request is IN stage 2 it is therefore the only thing that can
    // allocate, and allocating is what ends its stay -- so no allocation
    // can occur underneath it while it stalls.  That leaves exactly one
    // edge to account for: the edge the request enters stage 2, on which
    // the OUTGOING occupant may allocate.
    //
    // That case is closed by `set_haz_c`, which already exists and is
    // already checked at accept.  Stages shift rigidly, so the request
    // accepted one cycle before R is permanently one stage ahead of it: it
    // is in stage 2 exactly while R is in stage Q, and it was in stage A
    // when R was accepted -- where `a_same_set_c` compared their sets.  A
    // differing set means its allocation cannot appear in R's mask at all.
    // The one exemption `set_haz_c` grants is a same-LINE pair of READS,
    // and there the leader's allocation is found by R's `mshr_lu_hit`,
    // which is LIVE MSHR state, not this snapshot: R merges into that entry
    // and never consults a victim.  (A write is never exempt, so an
    // `s_lookup_ins_go` that installs over a way can never be the follower
    // of a same-set allocation.)
    //
    // Checked rather than only argued -- see the ALLOC-vs-LIVE-MASK
    // assertion below, which fires on the exact edge a stale mask would
    // introduce the corruption.
    //
    // The array view the next cycle's stage-2 occupant will resolve
    // against.  `tq_*` takes the live array output whenever `ce_c` is high,
    // so "what tq_* will hold next cycle" is `ce_c ? live : tq_*`.  The two
    // halves of `ce_c` are split because only one of them is late:
    // `rr_busy_c` is a pair of plain registers, available at the start of
    // the cycle.  This is what keeps the skew re-read correct -- on rr_p2
    // the fresh snapshot is on `rvalid`/`rplru` and has not reached `tq_*`
    // yet, and it is that cycle's registered value that stage 2 resolves
    // against at C+3.
    wire [7:0]  vs_r_valid = rr_busy_c ? rvalid : tq_rvalid;
    wire [31:0] vs_r_dsec  = rr_busy_c ? rdsec  : tq_rdsec;
    wire [6:0]  vs_r_plru  = rr_busy_c ? rplru  : tq_rplru;
    // "This way holds a dirty line" reduced for all eight ways in parallel
    // with the CAM -> victim chain, then selected by one bit -- the 2026-09-12
    // transform, kept, because it keeps this (now register-to-register)
    // cone shallow too.
    wire [7:0] vs_r_wdirty, vs_q_wdirty; genvar gvd;
    generate
        for (gvd = 0; gvd < 8; gvd = gvd + 1) begin : g_vsdirty
            assign vs_r_wdirty[gvd] = vs_r_valid[gvd] && (|vs_r_dsec[gvd*4 +: 4]);
            assign vs_q_wdirty[gvd] = rvalid[gvd]     && (|rdsec[gvd*4 +: 4]);
        end
    endgenerate
    wire vs_r_ok, vs_q_ok; wire [2:0] vs_r_way, vs_q_way;
    l2c_victim_sel u_vsel_r (.rvalid(vs_r_valid), .busy_mask(mshr_bw_mask), .plru_t(vs_r_plru),
                             .victim_ok(vs_r_ok), .victim_way(vs_r_way));
    l2c_victim_sel u_vsel_q (.rvalid(rvalid), .busy_mask(mshr_bwq_mask), .plru_t(rplru),
                             .victim_ok(vs_q_ok), .victim_way(vs_q_way));
    reg [2:0] victim_sel_way_q; reg victim_sel_ok_q, victim_dirty_q;
    always @(posedge clk) begin
        if (rst) begin
            victim_sel_way_q <= 3'd0; victim_sel_ok_q <= 1'b0; victim_dirty_q <= 1'b0;
        end else begin
            victim_sel_way_q <= pipe_adv_c ? vs_q_way : vs_r_way;
            victim_sel_ok_q  <= pipe_adv_c ? vs_q_ok  : vs_r_ok;
            victim_dirty_q   <= pipe_adv_c ? vs_q_wdirty[vs_q_way]
                                           : vs_r_wdirty[vs_r_way];
        end
    end
    wire [2:0] victim_sel_way = victim_sel_way_q;
    wire       victim_sel_ok  = victim_sel_ok_q;
    wire [3:0] victim_dsec_c  = tq_rdsec[victim_sel_way*4 +: 4];
    // Rule 1: a tag match keeps its own way, so nothing is evicted for it.
    //
    // 2026-09-12 (FMax): `need_evict_c` is on the design's binding trunk --
    // req_set -> mshr_bw_mask -> victim_sel_way -> need_evict_c ->
    // way_ok_c -> s_lookup_miss_go -> p2_done_c, which then fans into BOTH
    // the array read address (tags_raddr, ~1500 URAM/BRAM address pins) and
    // s_axi_wready.  It used to be computed from `victim_dsec_c`, i.e. an
    // 8:1-over-4b select FOLLOWED by a 4-input OR and an AND -- three LUT
    // levels PAST victim_sel_way, measured at 0.94 ns on the routed 200 MHz
    // stub build.
    //
    // "This way holds a dirty line" is a function of the ARRAY OUTPUT
    // ONLY (tq_rvalid/tq_rdsec, registered beside stage 2), not of the
    // victim choice, so it can be reduced for all eight ways IN PARALLEL
    // with the bw_mask -> victim_sel_way chain and then selected.  That
    // leaves victim_sel_way driving a plain 8:1 select of ONE bit -- one
    // LUT level, or an F7 mux -- instead of three levels.
    // victim_dsec_c is kept for victim_push_dsec (a register input with
    // slack), which is the only other consumer.
    // 2026-09-16: the eight-way dirty reduce AND the select by the victim way
    // both moved into the precompute above, so this is now one AND of two
    // registers' worth of information instead of a select past a CAM.
    wire need_evict_c = !any_tm_c && victim_dirty_q;
    // The way this request will end up owning, and that way's current
    // quadrant masks -- 4'b0 for a fresh victim, whose contents are being
    // discarded.  ONE 8:1-over-4b select each, shared by the RMW below,
    // by the eviction decision and by the MSHR allocation snapshot.
    wire [2:0] sel_way_c   = any_tm_c ? tm_way_c : victim_sel_way;
    wire [3:0] base_vsec_c = any_tm_c ? tq_rvsec[tm_way_c*4 +: 4] : 4'b0000;
    wire [3:0] base_dsec_c = any_tm_c ? tq_rdsec[tm_way_c*4 +: 4] : 4'b0000;
    // What this request still has to fetch: quadrants it needs to read and
    // neither already owns nor supplies itself.
    //
    // 2026-09-15 (FMax).  Same transform as `way_dirty_c` above, applied to
    // the other reduction hanging off the tag-match cone, and for the same
    // reason -- this one was measured on the routed 200 MHz eth netlist as
    // the worst path with BOTH ends inside u_l2c:
    //
    //   u_tags/g_way[4].mem_reg_bram_1/CLKBWRCLK (the tag array's output
    //   register, which Vivado has absorbed tq_rtag into: 0.218 ns clk-to-Q
    //   is a DO_REG=1 delay) -> tag_match_c -> <l2c_pri8> -> <8:1-over-4b
    //   select> -> base_vsec_c -> need_fill_c -> s_lookup_miss_go -> tw_en
    //   -> u_tags/g_way[0].mem_reg_bram_2/WEA[0]
    //
    //   -0.656 ns, 10 logic levels, 4.915 ns of data path of which 3.627 ns
    //   (73.8%) is ROUTE.  FOUR of those ten levels are spent between
    //   tag_match_c and need_fill_c, and three of them need not be.
    //
    // `req_want` is either 4'b0000 or ONE-HOT AT req_qoff by construction
    // (cur_want_c above), so `|(req_want & ~vsec)` is exactly "this request
    // wants a quadrant AND that quadrant is not valid in this way" -- a 4:1
    // select of the way's vsec by req_qoff, which depends on the ARRAY
    // OUTPUT and on stage 2's own registers and NOT on the tag compare.  It
    // is therefore computed for all eight ways in parallel with the tag
    // compare, leaving a single 2:1 select past tag_match_c and then the
    // same 8:1-of-one-bit the victim path already uses.
    //
    // The no-match case folds into the same select instead of needing its
    // own mux afterwards: l2c_pri8 returns idx 7 when nothing matches, and
    // tag_match_c[7] is then 0, so way 7's "not matched" leg supplies
    // `req_want_any_c` -- which is what `|(req_want & ~4'b0000)` is.
    //
    // base_vsec_c/base_dsec_c stay: they feed tw_vsec and the MSHR
    // allocation snapshot, both register inputs with slack.
    wire req_want_any_c = |req_want;
    wire [7:0] way_need_fill_c; genvar gwf;
    generate
        for (gwf = 0; gwf < 8; gwf = gwf + 1) begin : g_wayneed
            // Level 1, in parallel with the tag compare: is the wanted
            // quadrant absent from this way?
            wire wq_absent_w = !tq_rvsec[gwf*4 + req_qoff];
            // Level 1 past tag_match_c: a way that does NOT hold this
            // address contributes "everything this request wants is
            // missing", which is what the old any_tm_c mux said.
            assign way_need_fill_c[gwf] =
                req_want_any_c && (tag_match_c[gwf] ? wq_absent_w : 1'b1);
        end
    endgenerate
    wire       need_fill_c = way_need_fill_c[tm_way_c];
    // ⚠ TWO MORE STRESS-ONLY GUARDS (race audit 2026-09-18), same shape as
    // hit_wr_blocked_c above: both can be deleted with `make tb-l2c` still
    // reporting 65 PASS / 0 FAIL, and both then corrupt data.
    //
    //   `victim_push_ready` -- without it a SECOND eviction is allowed while
    //       the first push is still unaccepted, overwriting victim_push_*
    //       and losing a dirty line.  Measured: 5/5 stress seeds fail with a
    //       scoreboard mismatch.  The worst of the three, and the default
    //       gate is completely blind to it.
    //   `!mshr_inst_valid` -- array write-port arbitration.  Without it a
    //       miss allocates believing it wrote a placeholder tag that the
    //       install actually suppressed.  Measured: >=3/5 stress seeds fail.
    //
    // Edit either of these with `make tb-l2c-stress`, not `make tb-l2c`.
    wire way_ok_c  = any_tm_c || (victim_sel_ok && (!need_evict_c || victim_push_ready));
    wire miss_ok_c = mshr_alloc_ready && way_ok_c && !mshr_inst_valid;
    assign victim_query_addr = req_addr;
    // Hit-response skid register occupancy.  A hit (or an illegal-burst
    // SLVERR) may only resolve when the register is free or drains this
    // cycle; otherwise S_LOOKUP simply retries.  Misses are NOT gated on
    // this -- their response comes from l2c_mshr's own rsp_* channel much
    // later, so blocking a miss dispatch here would be pure loss.
    wire hit_rsp_free_c  = !hit_rsp_valid || hit_rsp_ready;
    wire hit_rsp_block_c = req_need && !hit_rsp_free_c;
    // Stage 2 may resolve when it holds a request AND the array outputs
    // registered beside it belong to that request (no re-read in flight).
    wire p2_data_ok_c    = !rr_busy_c;
    wire s2_live_c       = req_v && p2_data_ok_c;
    // -- AXI4 same-id response ordering, moved from the door to here -----
    //
    // Critical-3 used to refuse the ACCEPT of any beat whose id was live in
    // the MSHR.  A pipelined lookup cannot afford that: it is exactly the
    // beats 2..N of a missing burst, and they are the ones that want to
    // MERGE into their leader's MSHR entry (whose replay list responds in
    // insertion order, so merging is ordered BY CONSTRUCTION).  The check
    // therefore moved to the point where the outcome is known:
    //
    //   * a path that answers NOW (hit / no-fetch install / SLVERR) must
    //     not overtake ANY live entry carrying this id  -> idq2_any_c;
    //   * a MERGE is ordered by the entry it joins, so only OTHER entries
    //     carrying this id matter                      -> idq2_other_c;
    //   * a fresh ALLOCATION gets its own entry, so any other entry with
    //     this id is a hazard                          -> idq2_any_c.
    //
    // l2c_mshr exports the per-entry id-match VECTOR (idq2_vec) rather than
    // a reduced busy bit precisely so the "all but the entry I am merging
    // into" form can be taken here without a second CAM.
    wire [7:0] idq2_vec;
    wire [7:0] lu_onehot_c  = mshr_lu_hit ? (8'b1 << mshr_lu_idx) : 8'b0;
    wire       idq2_any_c   = |idq2_vec;
    wire       idq2_other_c = |(idq2_vec & ~lu_onehot_c);
    wire       byp_ord_c    = byp_active_valid && (byp_active_id == req_id);
    wire ord_now_block_c    = idq2_any_c   || byp_ord_c;
    wire ord_merge_block_c  = idq2_other_c || byp_ord_c;
    wire s_lookup_active = s2_live_c && !req_illegal && !skew_hazard_c && !victim_query_hit;
    // Rule 2: a WRITE onto a quadrant the cache already owns must not
    // proceed while that line has a live fill -- the install would revert
    // it, and it cannot merge either (l2c_mshr merges into fetched data,
    // which is stale for a quadrant the cache owns).  It retries; the fill
    // completes without needing the front door, so this cannot livelock.
    // ⚠ STRESS-ONLY GUARD (race audit 2026-09-18).  This is "Rule 2" -- a
    // write onto an already-valid quadrant of a line with a live MSHR entry
    // must RETRY, never merge; l2c_mshr.v's header says "Do not relax that
    // gate here".  Deleting it (`= 1'b0`) leaves `make tb-l2c` reporting
    // 65 PASS / 0 FAIL: the DIRECTED SUITE CANNOT SEE IT.  Only the
    // randomized scoreboard catches it, and at the OLD stress size
    // (3 seeds x 50k ops) it did not either -- 0/3.  Measured at the
    // current size (5 seeds x 100k): 4/5 seeds fail with a final-verify
    // data mismatch.  That is why tb-l2c-stress is now in ALL_TBS and why
    // its sizing is pinned in the Makefile.  If you edit this line, run
    // `make tb-l2c-stress`, not `make tb-l2c`.
    wire hit_wr_blocked_c = req_is_write && mshr_lu_hit;
    wire s_lookup_hit     = s_lookup_active && any_hit_c && !hit_wr_blocked_c;
    wire s_lookup_hit_go  = s_lookup_hit && (!req_is_write || !mshr_inst_valid) &&
                            !hit_rsp_block_c && !ord_now_block_c;
    wire s_lookup_hit_wr_go = s_lookup_hit_go && req_is_write;
    wire miss_no_hit_c   = s_lookup_active && !any_hit_c;
    // A full-line request never merges into an MSHR entry (the entry holds
    // one 128-bit quadrant) and never allocates one (it needs no fill); on
    // mshr_lu_hit it simply retries until that entry installs, after which
    // it resolves as an ordinary write HIT over the now-resident line.
    wire merge_valid_c   = miss_no_hit_c && !req_full && mshr_lu_hit && mshr_merge_ready &&
                           !ord_merge_block_c;
    wire alloc_valid_c   = miss_no_hit_c && !mshr_lu_hit && !req_full && need_fill_c &&
                           miss_ok_c && !ord_now_block_c;
    wire s_lookup_miss_go = miss_no_hit_c && !mshr_lu_hit && alloc_valid_c;
    // NO-FETCH INSTALL -- the generalisation of 00f079c's full-line case.
    // A write that supplies every byte of the quadrants it touches needs
    // nothing from DRAM: pick the way (the tag-matching one if the line is
    // already resident, else a victim, evicted if it has dirty quadrants),
    // then write data + quadrant valid + quadrant dirty in one shot.  Tag
    // and data land on the SAME clock edge, so no cycle advertises a valid
    // quadrant whose data has not arrived.  The MSHR is not involved at
    // all, hence no mshr_alloc_ready term.
    wire ins_ok_c        = way_ok_c && !mshr_inst_valid && !hit_rsp_block_c && !ord_now_block_c;
    wire s_lookup_ins_go = miss_no_hit_c && !mshr_lu_hit && req_is_write &&
                           !need_fill_c && ins_ok_c;
    // Both paths that leave quadrants VALID and DIRTY in one shot.
    wire line_install_c  = s_lookup_hit_wr_go || s_lookup_ins_go;
    // -- Pipeline advance -------------------------------------------------
    // Stage 2 retires this cycle.  Every outcome that used to write
    // `st <= S_IDLE` is a term here, and every stall that used to fall
    // through the case (leaving st at S_LOOKUP) is its absence.
    wire p2_done_c = s2_live_c &&
                     (req_illegal ? (!hit_rsp_block_c && !ord_now_block_c)
                                  : (s_lookup_hit_go || merge_valid_c ||
                                     s_lookup_ins_go || s_lookup_miss_go));
    assign rr_start_c = s2_live_c && !req_illegal && skew_hazard_c;
    assign p2_free_c  = !req_v || p2_done_c;
    // The rigid shift: every stage moves, or none does.  `!rr_busy_c` and
    // `!rr_start_c` are redundant (during either, stage 2 cannot resolve,
    // so p2_free_c is already 0) and are written anyway -- they were terms
    // of the old accept_slot_c, so they cost nothing that was not already
    // being paid, and they make the freeze independent of that reasoning.
    assign pipe_adv_c = p2_free_c && !rr_busy_c && !rr_start_c;
    // Rule 1: a miss onto a tag-matching way must NOT rewrite that way's
    // tag entry -- doing so would drop the quadrants already resident
    // there.  The tag is already correct, and those quadrants stay valid
    // and hittable for the whole fill.
    assign tw_en = line_install_c || (s_lookup_miss_go && !any_tm_c); assign tw_set = req_set;
    assign tw_way = sel_way_c;
    // Rule 3: read-modify-write, sound only because `skew_hazard_c`
    // bounces any request whose set an install touched since it was read.
    assign tw_vsec = line_install_c ? (base_vsec_c | req_cov) : 4'b0000;
    assign tw_dsec = line_install_c ? (base_dsec_c | req_dty) : 4'b0000;
    assign tw_tag = req_tag;
    assign dw_en = line_install_c; assign dw_set = req_set;
    assign dw_way = sel_way_c;
    assign dw_data = req_full ? req_line : {4{req_wdata}};
    assign dw_strb = req_full ? {64{1'b1}} : ({48'b0, req_wstrb} << (req_qoff * 16));
    // Way-select read mux, SHARED between the hit-data path and the
    // dirty-victim path.  The two are mutually exclusive by construction
    // (a victim is only pushed on s_lookup_miss_go, which requires
    // !any_hit_c), so ONE 512b 8:1 mux serves both.  The earlier code
    // spent a separate 128b 32:1 mux (8 ways x 4 quadrants) on the hit
    // path on top of the 512b 8:1 victim mux; estimated saving ~600 LUT
    // (docs/l2c_perf.md).
    wire [2:0]           rd_way_c  = sel_way_c;
    wire [LINE_BITS-1:0] rd_line_c = rdata[rd_way_c*LINE_BITS +: LINE_BITS];
    assign pw_en = s_lookup_hit_go || s_lookup_miss_go || s_lookup_ins_go; assign pw_set = req_set;
    assign pw_data = l2c_plru_next(tq_rplru, sel_way_c);
    // Keep the stale-response sink active for the full tag-clear walk.  The
    // front door cannot use the cache during rst_busy anyway, and releasing
    // the MSHR drain earlier allows a delayed pre-reset fill to sit at
    // RVALID until a post-reset allocation reuses its RID.
    // Level A (task #269), Q5 fix: axi_i and axi_d are independently-
    // numbered AXI masters (each starts its own IDs from 0), so a raw
    // ID_WIDTH-wide compare at idq_id/alloc_id/merge_id would let an LSU
    // op and an unrelated fetch op sharing the same numeric ID spuriously
    // block each other (Critical-3, id_busy_c).  Fix: fold cur_is_fetch/
    // req_is_fetch in as an EXTRA top bit ONLY for this instance's
    // internal ID bookkeeping (idq_id compare, p_id/r_id storage, rsp_id)
    // -- widen just this instantiation's ID_WIDTH by 1.  m_arid/m_rid are
    // NOT part of that bookkeeping (m_arid is the MSHR's own entry-index
    // ID, m_rid's only consumed field is its low IDX_BITS, see
    // l2c_mshr.v's own m_arid/r_idx_c), but they DO inherit the wider
    // port from the same parameter, so route them through local
    // ID_WIDTH+1-wide wires and explicitly slice/pad at the mf_arid/
    // mf_rid module-port boundary below -- keeps every OTHER port
    // (mf_arid, mf_rid, and hence l2c.v's m_axi_arid toward DRAM) at
    // EXACTLY ID_WIDTH bits, unchanged.  No change to l2c_mshr.v itself:
    // its ID matching is a plain equality/storage compare over whatever
    // width its parameter gives it.
    wire [ID_WIDTH:0] mshr_m_arid_x; wire [ID_WIDTH:0] mshr_rsp_id_x;
    assign mf_arid = mshr_m_arid_x[ID_WIDTH-1:0];
    // synthesis translate_off
    // mshr_m_arid_x's extra top bit is always 0 (it's zero-padded from
    // ar_idx_c, which is only IDX_BITS wide -- see l2c_mshr.v's m_arid
    // assign), so dropping it above is lossless; assert that invariant
    // rather than trust the comment.
    always @(posedge clk) if (!rst && mshr_m_arid_x[ID_WIDTH]) begin
        $display("L2C_CTRL ASSERT: mshr fill-AR id top bit unexpectedly set: 0x%0x", mshr_m_arid_x);
        $fatal(1);
    end
    // synthesis translate_on
    assign mshr_rsp_id = mshr_rsp_id_x[ID_WIDTH-1:0];
    assign mshr_rsp_is_fetch = mshr_rsp_id_x[ID_WIDTH];
    l2c_mshr #(.SET_BITS(SET_BITS), .TAG_BITS(TAG_BITS), .WAY_BITS(WAY_BITS),
               .ADDR_WIDTH(ADDR_WIDTH), .LINE_BITS(LINE_BITS), .ID_WIDTH(ID_WIDTH+1),
               .REPLAY_N(MSHR_REPLAY_N), .RK_BITS(MSHR_REPLAY_K)) u_mshr (
        .clk(clk), .rst(rst || rst_busy),
        .lu_set(req_set), .lu_tag(req_tag), .lu_hit(mshr_lu_hit), .lu_idx(mshr_lu_idx),
        .bw_set(req_set), .bw_mask(mshr_bw_mask),
        .bwq_set(q_set),  .bwq_mask(mshr_bwq_mask),
        .idq_id({cur_is_fetch, cur_id}), .idq_busy(mshr_idq_busy),
        .idq2_id({req_is_fetch, req_id}), .idq2_vec(idq2_vec),
        .alloc_valid(alloc_valid_c), .alloc_ready(mshr_alloc_ready), .alloc_set(req_set), .alloc_tag(req_tag),
        .alloc_way(sel_way_c), .alloc_is_write(req_is_write), .alloc_id({req_is_fetch, req_id}), .alloc_wdata(req_wdata),
        .alloc_wstrb(req_wstrb), .alloc_qoff(req_qoff), .alloc_last(req_last), .alloc_need_resp(req_need),
        .alloc_vsec_pre(base_vsec_c), .alloc_dsec_pre(base_dsec_c),
        .merge_valid(merge_valid_c), .merge_ready(mshr_merge_ready), .merge_idx(mshr_lu_idx),
        .merge_is_write(req_is_write), .merge_id({req_is_fetch, req_id}), .merge_wdata(req_wdata), .merge_wstrb(req_wstrb),
        .merge_qoff(req_qoff), .merge_last(req_last), .merge_need_resp(req_need),
        .inst_valid(mshr_inst_valid), .inst_set(mshr_inst_set),
        .inst_way(mshr_inst_way), .inst_tag(mshr_inst_tag),
        .inst_vsec(mshr_inst_vsec), .inst_dsec(mshr_inst_dsec), .inst_data(mshr_inst_data),
        .inst_strb(mshr_inst_strb),
        .rsp_valid(mshr_rsp_valid), .rsp_ready(mshr_rsp_ready), .rsp_is_write(mshr_rsp_is_write),
        .rsp_id(mshr_rsp_id_x), .rsp_rdata(mshr_rsp_rdata), .rsp_last(mshr_rsp_last), .rsp_resp(mshr_rsp_resp),
        .occupancy(dbg_mshr_occupancy),
        .m_arid(mshr_m_arid_x), .m_araddr(mf_araddr), .m_arlen(mf_arlen), .m_arsize(mf_arsize),
        .m_arburst(mf_arburst), .m_arvalid(mf_arvalid), .m_arready(mf_arready),
        .m_rid({1'b0, mf_rid}), .m_rdata(mf_rdata), .m_rresp(mf_rresp), .m_rlast(mf_rlast), .m_rvalid(mf_rvalid), .m_rready(mf_rready)
    );
    // synthesis translate_off
    // ALLOC-vs-LIVE-MASK (2026-09-16, victim precompute).  The victim way is
    // chosen one cycle early now, against a busy-way mask that is one cycle
    // old by the time it is used.  The argument that this is sound is in the
    // VICTIM PRECOMPUTE block above; this is the check that it IS sound,
    // placed on the exact edge a stale mask would introduce the corruption
    // rather than where the corruption later surfaces.  l2c_mshr's own
    // duplicate-(set,way) assertion is the backstop; this one names the
    // cause.
    //
    // `any_tm_c` is excluded deliberately, for the same reason l2c_mshr's
    // header excludes it: a tag match keeps its OWN way, which is allowed to
    // be mask-busy (that is a merge-eligible fill into a line this request
    // already owns), and it is the one path that legitimately bypasses the
    // busy-way mask.
    always @(posedge clk) begin
        if (!rst && !rst_busy && alloc_valid_c && mshr_alloc_ready &&
            !any_tm_c && mshr_bw_mask[sel_way_c]) begin
            $display("L2C_CTRL ASSERT: allocation for set %0d claims way %0d, which the LIVE MSHR busy mask (%b) already holds -- the one-cycle-old victim precompute went stale",
                     req_set, sel_way_c, mshr_bw_mask);
            $fatal(1);
        end
    end
    // synthesis translate_on

    // synthesis translate_off
    // INVARIANT: a bypass-window address must NEVER enter the tag/data
    // arrays.  docs/l2c_spec.md S1 invariant 2 -- a cached dirty line's
    // eviction writeback can land on and clobber a later bypass write to
    // the same physical DRAM address, which is silent corruption rather
    // than a classification error.
    //
    // Before the full-line-write gather, is_bypass_c was literally
    // byp_match_hit, so this was true by construction and needed no
    // assertion.  The gather made it CONDITIONAL for the first time --
    // and the first version of that condition was wrong (it dropped
    // every bypass read that won the front door while an assembled line
    // was waiting; see is_bypass_c's own comment).  A gathered line's
    // base is classified non-bypass before its first beat is taken and
    // l2c_bypass asserts >= 64 B window granularity, so byp_match_hit is
    // 0 for every flw dispatch too -- the invariant is exact, not
    // approximate, and it is now checked on every accept in every tb run
    // rather than trusted.
    always @(posedge clk) begin
        if (!rst && do_accept_c && byp_match_hit) begin
            $display("L2C_CTRL ASSERT: bypass-window address 0x%08x accepted onto the CACHE path (read_sel=%b write_sel=%b flw_avail=%b gst=%0d)",
                     cur_addr, read_sel, write_sel, flw_avail_c, gst);
            $fatal(1);
        end
    end
    // INVARIANT (2026-09-15): AN ACCEPT IMPLIES STAGE 2 RETIRES ON THE SAME
    // EDGE.  This is the whole reason `set_haz_c` gets away with checking
    // only stages A and Q.  If a request could be accepted while an
    // occupied stage 2 did NOT resolve, that occupant's array write could
    // land AFTER the accepted request's array read (which is now one edge
    // later than the accept), and a same-set follower would resolve against
    // a snapshot that pre-dates an eviction -- the duplicate-way install
    // l2c_tags.v's own assertion exists to catch, reached from a different
    // direction.  It holds because do_accept_c requires accept_slot_c
    // requires pipe_adv_c requires p2_free_c, but that is four levels of
    // indirection through signals other people will edit, so it is checked
    // rather than trusted.
    always @(posedge clk) begin
        if (!rst && do_accept_c && req_v && !p2_done_c) begin
            $display("L2C_CTRL ASSERT: accepted addr 0x%08x (set %0d) while stage 2 (set %0d) did NOT retire -- set_haz_c only checks stages A and Q and is now unsound",
                     cur_addr, cur_set, req_set);
            $fatal(1);
        end
    end
    // INVARIANT: the pipeline is a rigid shift.  A stage may only become
    // occupied on a cycle the whole pipeline advanced; anything else means
    // an occupancy bit was written outside the pipe_adv_c block and the
    // array output registers (clock-enabled by that same signal) are no
    // longer aligned with the request they belong to.
    reg pv_a, pv_q, pv_r, pv_adv, pv_rst;
    always @(posedge clk) begin
        pv_a <= a_v; pv_q <= q_v; pv_r <= req_v;
        pv_adv <= pipe_adv_c; pv_rst <= rst;
        if (!rst && !pv_rst && !pv_adv &&
            ((pv_a !== a_v) || (pv_q !== q_v) || (pv_r !== req_v))) begin
            $display("L2C_CTRL ASSERT: pipeline occupancy {a,q,req} %b%b%b -> %b%b%b on a cycle pipe_adv_c was low",
                     pv_a, pv_q, pv_r, a_v, q_v, req_v);
            $fatal(1);
        end
    end
    // INVARIANT (the complement of the one above): a FETCH-sourced request
    // must NEVER be routed to the bypass engine.  l2c_bypass carries no
    // source tag, so its response returns on byp_rsp_* and is granted onto
    // the LSU-facing s_axi R/B mux in l2c.v (see that file's fetch response
    // pick, and its own note about there being no RSP_BYP leg).  That is two
    // failures at once:
    //   (a) the fetch port never gets its beats -- l2c.v's fq_*/f_rvalid_q
    //       reassembly fills ONLY from mshr_rsp/hit_rsp, so axi_i hangs
    //       forever; and
    //   (b) a spurious R burst appears on the LSU port carrying the FETCH's
    //       own ARID, which the crossbar delivers to an unrelated master.
    // Unreachable today ONLY because BYP_WIN_EN is 0 in every shipping
    // config -- fpga_top_ddr.vh leaves l2c's bypass-window parameters at
    // their disabled defaults, and since the DDR-backed RAM disk was
    // deleted (2026-09-10) there is no consumer left to turn them on.  It
    // was previously recorded as a "known, documented residual gap" in a
    // comment -- a comment cannot fail a test run, so it is an assertion
    // now.
    //
    // Note for whoever enables a bypass window: f_axi_araddr arrives
    // as a folded CPU address while BYP_WIN_BASE is a FLATTENED DDR address,
    // so the two are not in the same coordinate space.  Expect this to fire
    // on a legitimate ROM-probe RAM alias before it ever fires on the
    // carve-out aperture it was meant to describe.  That is the point: find
    // out at the first sim, not on hardware.  See docs/fabric_concurrency_contract.md
    // clause C4/C5 for the door and same-ID rules this sits alongside.
    always @(posedge clk) begin
        if (!rst && do_bypass_c && cur_is_fetch) begin
            $display("L2C_CTRL ASSERT: FETCH-sourced address 0x%08x (id=%0d) routed to the BYPASS engine -- l2c_bypass has no fetch tag; its response would misroute onto s_axi R/B and the fetch port would hang",
                     cur_addr, cur_id);
            $fatal(1);
        end
    end
    // ar_last_q is a REDUNDANT ENCODING -- checked, not argued.
    //
    // 2026-09-17.  It exists only to lift `ar_beat == ar_len` out of the
    // front door's cone (see "AR BEAT ADDRESS" above).  A register that
    // shadows other state is the exact shape that went wrong when the
    // victim way was registered once and never refreshed, so the shadowing
    // is CHECKED here, on every cycle of every test, rather than defended
    // in a comment that cannot fail a run.  If a future edit adds another
    // writer of ar_beat/ar_len and forgets this one, it fires on the first
    // cycle that writer runs.  Guarded by ar_have because ar_beat/ar_len
    // have no reset and ar_last_q is unreadable while the slot is empty.
    always @(posedge clk) begin
        if (!rst && ar_have && (ar_last_q !== (ar_beat == ar_len))) begin
            $display("L2C_CTRL ASSERT: ar_last_q=%b desynced from (ar_beat=%0d == ar_len=%0d) -- a writer of ar_beat/ar_len did not refresh it",
                     ar_last_q, ar_beat, ar_len);
            $fatal(1);
        end
    end
    // synthesis translate_on
    always @(posedge clk) begin
        if (rst) begin
            aw_have <= 1'b0; ar_have <= 1'b0; aw_pend_v <= 1'b0; ar_pend_v <= 1'b0;
            fetch_ar_have <= 1'b0; fetch_ar_pend_v <= 1'b0;
            rw_favor <= 1'b0; fetch_favor <= 1'b0; hit_rsp_valid <= 1'b0;
            a_v <= 1'b0; q_v <= 1'b0; req_v <= 1'b0; rr_p1 <= 1'b0; rr_p2 <= 1'b0;
            q_stale <= 1'b0; req_stale <= 1'b0;
            victim_push_valid <= 1'b0; byp_req_valid <= 1'b0;
            dbg_hit_count <= 32'd0; dbg_miss_count <= 32'd0;
            aw_flw <= 1'b0; gst <= G_IDLE; flw_cnt <= 2'd0; flw_rq <= 2'd0;
            // ar_last_q's invariant cannot hold at reset -- ar_beat/ar_len
            // are not reset either -- but it is unreadable until a
            // promotion writes it, because every consumer is qualified by
            // ar_have, which IS reset.  Given a defined value anyway so the
            // shadow-consistency assertion below has something to compare.
            ar_last_q <= 1'b0;
            flw_allstrb <= 1'b0; flw_last <= 1'b0;
            a_full <= 1'b0; q_full <= 1'b0; req_full <= 1'b0;
        end else begin
            rr_p1 <= rr_start_c; rr_p2 <= rr_p1;
            // Stale-snapshot sticky -- see skew_hazard_c.  Order matters:
            // the re-read's reset must win over the accumulate, and the
            // pipeline shift (further down) must win over both, because on
            // a shift the bit belongs to a DIFFERENT request.
            if (inst_on_q_c) q_stale   <= 1'b1;
            if (inst_on_r_c) req_stale <= 1'b1;
            // Re-read resets.  Stage 2's fresh snapshot is read on edge
            // rr_p1 -> rr_p2, stage Q's restored one on edge rr_p2 -> C+3;
            // each captures the install that is blind to its own edge.
            if (rr_p1) req_stale <= inst_on_r_c;
            if (rr_p2) q_stale   <= inst_on_q_c;
            // Array read address register -- see the ARRAY ADDRESS REGISTER
            // block above.  The re-read schedule outranks the accept, and
            // an unclaimed cycle HOLDS (stage A's set stays on the pins).
            if (rr_start_c)       ra_q <= req_set;
            else if (rr_p1)       ra_q <= q_set;
            else if (rr_p2)       ra_q <= a_set;
            else if (do_accept_c) ra_q <= cur_set;
            byp_req_valid <= byp_req_valid && !byp_req_ready; // clears once accepted
            // 1-deep hit-response skid: self-clearing on its handshake.
            // Placed BEFORE the case so a fresh S_LOOKUP load in the same
            // cycle overrides it (back-to-back hits, response accepted the
            // cycle it is presented).
            hit_rsp_valid <= hit_rsp_valid && !hit_rsp_ready;
            if (victim_push_valid && victim_push_ready) victim_push_valid <= 1'b0;
            // -- Header door: active slot + one-deep pre-latch ---------
            // Promotion first: whenever the active slot frees, it takes
            // the pre-latch if one is waiting, else the header arriving
            // this very cycle, else it goes empty.
            if (ar_act_free_c) begin
                if (ar_pend_v) begin
                    ar_have <= 1'b1; ar_id <= ar_pend_id;
                    ar_len <= ar_pend_len; ar_burst <= ar_pend_burst; ar_beat <= 8'd0;
                    ar_step <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1} << ar_pend_size;
                    // ar_beat <= 0, so the length test is against beat 0.
                    // See the ar_last_q note at its declaration.
                    ar_last_q <= (ar_pend_len == 8'd0);
                end else if (s_arvalid && s_arready) begin
                    ar_have <= 1'b1; ar_id <= s_arid;
                    ar_len <= s_arlen; ar_burst <= s_arburst; ar_beat <= 8'd0;
                    ar_step <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1} << s_arsize;
                    ar_last_q <= (s_arlen == 8'd0);
                end else begin
                    // Slot goes empty.  ar_beat/ar_len both HOLD here, so
                    // ar_last_q must hold with them.
                    ar_have <= 1'b0;
                end
            end else if (ar_take_c) begin
                ar_beat <= ar_beat + 8'd1;
                // ar_len does not move on a beat, so the invariant follows
                // the incremented beat alone.
                ar_last_q <= ((ar_beat + 8'd1) == ar_len);
            end
            // ar_cur is written OUTSIDE the door's if/else, with a real CE
            // and no hold-feedback in its D -- see "AR BEAT ADDRESS" above
            // for why that shape is the fix and not a stylistic choice.
            // ar_cur_load_c / ar_cur_inc_c are transcribed from the branch
            // conditions directly above them.
            if (ar_cur_load_c)     ar_cur <= ar_cur_ld;
            else if (ar_cur_inc_c) ar_cur <= ar_cur_next;
            // Pre-latch: holds a header only when it could NOT go straight
            // into the active slot on this same edge.
            if (s_arvalid && s_arready && !(ar_act_free_c && !ar_pend_v)) begin
                ar_pend_v <= 1'b1; ar_pend_id <= s_arid; ar_pend_addr <= s_araddr;
                ar_pend_len <= s_arlen; ar_pend_size <= s_arsize; ar_pend_burst <= s_arburst;
            end else if (ar_act_free_c && ar_pend_v) begin
                ar_pend_v <= 1'b0;
            end
            if (aw_act_free_c) begin
                if (aw_pend_v) begin
                    aw_have <= 1'b1; aw_id <= aw_pend_id; aw_cur <= aw_pend_addr;
                    aw_len <= aw_pend_len; aw_burst <= aw_pend_burst; aw_beat <= 8'd0;
                    aw_step <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1} << aw_pend_size;
                    aw_flw <= aw_pend_flw;
                end else if (s_awvalid && s_awready) begin
                    aw_have <= 1'b1; aw_id <= s_awid; aw_cur <= s_awaddr;
                    aw_len <= s_awlen; aw_burst <= s_awburst; aw_beat <= 8'd0;
                    aw_step <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1} << s_awsize;
                    // Conservative full-line-write shape test, header only:
                    // INCR, 16 B beats, 64 B-aligned base, whole lines.
                    aw_flw <= (s_awburst == 2'b01) && (s_awsize == 3'd4) &&
                              (s_awaddr[5:0] == 6'd0) && (s_awlen[1:0] == 2'b11);
                end else begin
                    aw_have <= 1'b0;
                end
            end else if (aw_take_c) begin
                aw_beat <= aw_beat + 8'd1; aw_cur <= aw_cur + aw_step;
            end
            if (s_awvalid && s_awready && !(aw_act_free_c && !aw_pend_v)) begin
                aw_pend_v <= 1'b1; aw_pend_id <= s_awid; aw_pend_addr <= s_awaddr;
                aw_pend_len <= s_awlen; aw_pend_size <= s_awsize; aw_pend_burst <= s_awburst;
                aw_pend_flw <= (s_awburst == 2'b01) && (s_awsize == 3'd4) &&
                               (s_awaddr[5:0] == 6'd0) && (s_awlen[1:0] == 2'b11);
            end else if (aw_act_free_c && aw_pend_v) begin
                aw_pend_v <= 1'b0;
            end
            // Level A (task #269): fetch header door -- active slot + one-deep
            // pre-latch, structurally identical to the s_ar*/ar_have door
            // above.  Promotion first: whenever the active slot frees it
            // takes the pre-latch if one is waiting, else the header
            // arriving this very cycle, else it goes empty.
            if (fetch_ar_act_free_c) begin
                if (fetch_ar_pend_v) begin
                    fetch_ar_have <= 1'b1; fetch_ar_id <= fetch_ar_pend_id;
                    fetch_ar_base <= fetch_ar_pend_addr; fetch_ar_len <= fetch_ar_pend_len;
                    fetch_ar_size <= fetch_ar_pend_size; fetch_ar_burst <= fetch_ar_pend_burst;
                    fetch_ar_beat <= 8'd0;
                end else if (f_arvalid && f_arready) begin
                    fetch_ar_have <= 1'b1; fetch_ar_id <= f_arid;
                    fetch_ar_base <= f_araddr; fetch_ar_len <= f_arlen;
                    fetch_ar_size <= f_arsize; fetch_ar_burst <= f_arburst;
                    fetch_ar_beat <= 8'd0;
                end else begin
                    fetch_ar_have <= 1'b0;
                end
            end else if (fetch_ar_take_c) begin
                fetch_ar_beat <= fetch_ar_beat + 8'd1;
            end
            // Pre-latch: holds a header only when it could NOT go straight
            // into the active slot on this same edge.
            if (f_arvalid && f_arready && !(fetch_ar_act_free_c && !fetch_ar_pend_v)) begin
                fetch_ar_pend_v <= 1'b1; fetch_ar_pend_id <= f_arid;
                fetch_ar_pend_addr <= f_araddr; fetch_ar_pend_len <= f_arlen;
                fetch_ar_pend_size <= f_arsize; fetch_ar_pend_burst <= f_arburst;
            end else if (fetch_ar_act_free_c && fetch_ar_pend_v) begin
                fetch_ar_pend_v <= 1'b0;
            end
            // Full-line-write gather.  A gathered beat is consumed HERE and
            // never reaches the tag pipeline, so it owns the aw_beat/aw_have
            // accounting for itself; the S_IDLE arm below skips that update
            // whenever it is dispatching an already-assembled line
            // (flw_avail_c), and the two are mutually exclusive by gst.
            if (flw_gath_c) begin
                flw_data[flw_cnt*128 +: 128] <= s_wdata;
                flw_strb[flw_cnt*16  +: 16]  <= s_wstrb;
                flw_last <= aw_cur_last;   // beat/burst accounting is in the header door above
                if (gst == G_IDLE) begin
                    // First quadrant: flw_arm_c already required &s_wstrb.
                    gst <= G_GATH; flw_addr <= aw_cur_addr; flw_id <= aw_id;
                    flw_cnt <= 2'd1; flw_allstrb <= 1'b1;
                end else begin
                    flw_cnt <= flw_cnt + 2'd1;
                    flw_allstrb <= flw_allstrb && (&s_wstrb);
                    if (flw_cnt == 2'd3) begin
                        // Fourth quadrant closes the line.  flw_allstrb is
                        // still the AND over quadrants 0..2 at this point,
                        // so this beat's own strobes are folded in here.
                        gst    <= (flw_allstrb && (&s_wstrb)) ? G_FULL : G_RPLY;
                        flw_rq <= 2'd0;
                    end
                end
            end
            // Beat accounting below is common to both accept paths
            // (do_bypass_c/do_accept_c are mutually exclusive).
            //
            // ---- S0: front-door accept -------------------------------
            if (door_take_c) rw_favor <= !rw_favor;
            // Level A (task #269): fetch/LSU read-source alternation +
            // fetch's own single-entry tracker advance.  Scoped to
            // `door_take_c && read_sel` (any read accept, fetch or LSU) --
            // the LSU-only per-beat advance for ar_beat/ar_cur/ar_have
            // lives in the two-deep-door promotion block below, gated by
            // `ar_take_c`, which is itself scoped to lsu_read_sel so a
            // fetch-sourced accept can never spuriously advance the LSU's
            // own burst bookkeeping.
            // The fetch burst's own beat/have advance moved into the
            // header-door promotion block above (fetch_ar_act_free_c /
            // "else if (fetch_ar_take_c)"), so that a burst ending on the
            // same edge a new header arrives promotes straight through
            // with no dead cycle.  Only the read-source alternator is
            // still driven from here.
            if (door_take_c && read_sel) begin
                fetch_favor <= !fetch_favor;
            end
            // Gather release.  BOTH shapes now release at ACCEPT: G_RPLY
            // walks its four quadrants one per accept as before, and
            // G_FULL -- which used to be pinned until its request RESOLVED
            // so flw_data could not be overwritten -- is released here
            // because the assembled line is copied into the pipeline
            // (q_line) on this same edge.  That is what lets the next
            // line's gather overlap this line's lookup.
            if (do_accept_c && write_sel && flw_avail_c) begin
                if (gst == G_RPLY) begin
                    flw_rq <= flw_rq + 2'd1;
                    if (flw_rq == 2'd3) gst <= G_IDLE;
                end else begin
                    gst <= G_IDLE;
                end
            end
            if (do_bypass_c) begin
                byp_req_valid <= 1'b1; byp_req_is_write <= write_sel; byp_req_addr <= cur_addr;
                byp_req_id <= cur_id; byp_req_wdata <= s_wdata; byp_req_wstrb <= s_wstrb;
                byp_req_need_resp <= cur_need;
                // Round-4: thread cur_last to l2c_bypass so a
                // multi-beat read's RLAST lands correctly (rsp_last).
                byp_req_last <= cur_last;
            end
            // ---- The rigid shift: S0 -> A -> Q -> S2 -----------------
            //
            // 2026-09-15.  These used to be two independently-enabled
            // transfers (`do_accept_c` loading stage 1, `q_adv_c` loading
            // stage 2).  They are one shift now: on `pipe_adv_c` everything
            // moves and the accept decides only whether a REQUEST or a
            // BUBBLE enters stage A.  See accept_slot_c for why the rigid
            // form is what keeps the array clock enable a single net and
            // the accept cone's comparator count at two.
            //
            // A bubble's payload registers are left to hold stale values --
            // nothing reads a stage's payload without its valid bit -- so
            // the wide copies (a_line/q_line especially) stay gated by the
            // same `*_full` bits they always were.
            if (pipe_adv_c) begin
                a_v   <= do_accept_c;
                q_v   <= a_v;
                req_v <= q_v;
                // ---- S0 -> A ----------------------------------------
                if (do_accept_c) begin
                    a_addr <= cur_addr; a_set <= cur_set; a_tag <= cur_tag;
                    a_is_write <= write_sel; a_id <= cur_id; a_wdata <= cur_wdata;
                    a_wstrb <= cur_wstrb; a_qoff <= cur_qoff; a_last <= cur_last;
                    a_need <= cur_need; a_illegal <= illegal_c;
                    a_full <= write_sel && (gst == G_FULL);
                    if (write_sel && (gst == G_FULL)) a_line <= flw_data;
                    a_cov  <= write_sel ? cur_cov_c : 4'b0000;
                    a_dty  <= write_sel ? cur_dty_c : 4'b0000;
                    a_want <= cur_want_c;
                    // Level A (task #269): thread the fetch source tag
                    // through stage A alongside every other cur_* field.
                    a_is_fetch <= cur_is_fetch;
                end
                // ---- A -> Q -----------------------------------------
                // This edge IS stage A's array read, so the sticky starts
                // here rather than carrying anything forward.
                q_stale <= inst_on_a_c;
                if (a_v) begin
                    q_addr <= a_addr; q_set <= a_set; q_tag <= a_tag;
                    q_is_write <= a_is_write; q_id <= a_id; q_wdata <= a_wdata;
                    q_wstrb <= a_wstrb; q_qoff <= a_qoff; q_last <= a_last;
                    q_need <= a_need; q_illegal <= a_illegal;
                    q_full <= a_full; if (a_full) q_line <= a_line;
                    q_cov <= a_cov; q_dty <= a_dty; q_want <= a_want;
                    q_is_fetch <= a_is_fetch;
                end
                // ---- Q -> S2 ----------------------------------------
                req_stale <= q_stale || inst_on_q_c;
                if (q_v) begin
                    req_addr <= q_addr; req_set <= q_set; req_tag <= q_tag;
                    req_is_write <= q_is_write; req_id <= q_id; req_wdata <= q_wdata;
                    req_wstrb <= q_wstrb; req_qoff <= q_qoff; req_last <= q_last;
                    req_need <= q_need; req_illegal <= q_illegal;
                    req_full <= q_full; if (q_full) req_line <= q_line;
                    req_cov <= q_cov; req_dty <= q_dty; req_want <= q_want;
                    req_is_fetch <= q_is_fetch;
                end
            end
            // ---- S2: resolve -----------------------------------------
            if (s2_live_c) begin
                // Important-10: illegal burst -> 1 SLVERR/burst.
                if (req_illegal) begin
                    if (!hit_rsp_block_c && !ord_now_block_c) begin
                        // synthesis translate_off
                        $display("L2C_CTRL: rejecting illegal burst id=%0d addr=0x%08x with SLVERR", req_id, req_addr);
                        // synthesis translate_on
                        if (req_need) begin
                            hit_rsp_valid <= 1'b1; hit_rsp_is_write <= req_is_write; hit_rsp_is_fetch <= req_is_fetch; hit_rsp_id <= req_id;
                            hit_rsp_rdata <= {4{32'hDEADBEEF}}; hit_rsp_last <= req_last; hit_rsp_resp <= 2'b10;
                        end
                    end // else: skid full or id-ordered behind a live op, retry
                end else if (skew_hazard_c) begin
                    // Critical-7: rr_start_c has already re-issued this
                    // request's own set on the array read port; stage 2
                    // simply cannot resolve for two cycles.
                end else if (victim_query_hit) begin // stall, retry
                end else if (any_hit_c) begin
                    if (s_lookup_hit_go) begin
                        dbg_hit_count <= dbg_hit_count + 32'd1;
                        if (req_need) begin // Critical-1: non-last write beat gets no B
                            hit_rsp_valid <= 1'b1; hit_rsp_is_write <= req_is_write; hit_rsp_is_fetch <= req_is_fetch; hit_rsp_id <= req_id;
                            hit_rsp_rdata <= rd_line_c[req_qoff*128 +: 128];
                            hit_rsp_last <= req_last; hit_rsp_resp <= 2'b00;
                        end
                    end
                    // else: MSHR owns the write port, the response skid is
                    // still full, or an older op with this id is live -- retry
                end else if (mshr_lu_hit) begin
                    if (merge_valid_c) begin
                        dbg_hit_count <= dbg_hit_count + 32'd1; // merge avoids a 2nd DRAM fill
                    end
                end else if (s_lookup_ins_go) begin
                    // Quadrant miss that costs NO fill: this write
                    // supplied every byte of every quadrant it touches,
                    // so DRAM has nothing to contribute.  Counted as a
                    // miss because that is what the tag array saw; the
                    // DDR read it would have caused is never issued.
                    dbg_miss_count <= dbg_miss_count + 32'd1;
                    if (req_need) begin
                        // Level A (task #269): this hit_rsp_is_fetch <=
                        // was silently absent here even though it exists
                        // at the two S2 sites above -- this branch's text
                        // is byte-identical to what upstream's own 3-way
                        // merge algorithm treated as unconflicted, which
                        // means it took upstream's version verbatim.
                        hit_rsp_valid <= 1'b1; hit_rsp_is_write <= 1'b1; hit_rsp_is_fetch <= req_is_fetch; hit_rsp_id <= req_id;
                        hit_rsp_rdata <= rd_line_c[req_qoff*128 +: 128];
                        hit_rsp_last <= req_last; hit_rsp_resp <= 2'b00;
                    end
                    if (need_evict_c) begin
                        victim_push_valid <= 1'b1;
                        victim_push_addr <= {tq_rtag[victim_sel_way*TAG_BITS +: TAG_BITS], req_set,
                                              {(ADDR_WIDTH-TAG_BITS-SET_BITS){1'b0}}};
                        victim_push_data <= rd_line_c;
                        victim_push_dsec <= victim_dsec_c;
                    end
                end else if (s_lookup_miss_go) begin
                    dbg_miss_count <= dbg_miss_count + 32'd1;
                    if (need_evict_c) begin
                        victim_push_valid <= 1'b1;
                        victim_push_addr <= {tq_rtag[victim_sel_way*TAG_BITS +: TAG_BITS], req_set,
                                              {(ADDR_WIDTH-TAG_BITS-SET_BITS){1'b0}}};
                        victim_push_data <= rd_line_c;
                        victim_push_dsec <= victim_dsec_c;
                    end
                end
            end
        end
    end
endmodule
`default_nettype wire
