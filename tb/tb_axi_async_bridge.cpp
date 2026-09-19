// tb_axi_async_bridge.cpp — Verilator unit testbench for
//                            rtl/sys/axi_async_bridge.v
//
// 6+ scenarios — AXI4 bridge across two clocks backed by 5 async_fifos:
//
//   1. single_read_1beat              — AR + R across the bridge, 1 beat.
//   2. single_write_1beat             — AW + W + B across the bridge.
//   3. burst_read_8beat               — ARLEN=7, 8 R beats with RLAST.
//   4. burst_write_8beat              — AWLEN=7, 8 W beats + one B.
//   5. read_write_interleave          — writes and reads intermixed; both
//                                       directions keep order.
//   6. far_side_backpressure          — slave delays AW/AR ready; bridge
//                                       FIFOs absorb without losing data.
//   7. slow_to_fast                   — s_clk 1×, m_clk 3× ratio.
//   8. fast_to_slow                   — s_clk 3×, m_clk 1× ratio.
//   9. mid_transaction_mside_reset    — T6 CDC/reset-hardening: assert
//                                       ONLY m_rst mid-transaction
//                                       (simulating a MIG UI recal
//                                       pulse; s_rst stays low the
//                                       whole time) after a warm-up
//                                       round trip has already advanced
//                                       the per-channel async_fifo
//                                       pointers to a nonzero baseline.
//                                       Bridge must recover: no stuck
//                                       s_bvalid/s_rvalid, and
//                                       subsequent AXI transactions
//                                       complete with no corruption.
//
// Build via: make tb-axi-async-bridge

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <vector>
#include <verilated.h>
#include "Vaxi_async_bridge.h"

static Vaxi_async_bridge* dut    = nullptr;
static int                n_pass = 0, n_fail = 0;

static constexpr int DATA_W  = 64;
static constexpr int ADDR_W  = 32;
static constexpr int ID_W    = 4;

// ─── Ticking ─────────────────────────────────────────────────────────────
static void eval() { dut->eval(); }

// Forward declarations — referenced from reset() below.
static void slave_drive_idle();
static void slave_drive_combinational();
static void slave_latch_edge();

// ─── Software AXI slave on the m-side that responds to AW/W/AR beats ─
struct SlaveModel {
    std::map<uint64_t, uint64_t> mem;
    struct RQ { uint32_t id; uint64_t addr; uint8_t len; uint8_t burst_beat_idx; };
    std::deque<RQ> r_pending;
    struct AWQ { uint32_t id; uint64_t addr; uint8_t len; };
    std::deque<AWQ> aw_pending;
    struct WQ { uint64_t data; uint8_t strb; bool last; };
    std::deque<WQ> w_pending;
    int  readiness_cycle = 0;
    bool backpressure_enabled = false;
    bool data_ready_this_cycle() {
        if (!backpressure_enabled) return true;
        return (readiness_cycle++ & 1) == 0;
    }
};
static SlaveModel slv;

// Clock phase counters (used in ratio-mode helpers + debug).
static int   s_phase = 0;
static int   m_phase = 0;

// Pre-edge snapshot of m-side handshakes + payloads — see do_m_edge().
struct PreEdge {
    bool aw, w, ar, b, r;
    uint32_t awid, arid;
    uint64_t awaddr, araddr, wdata;
    uint8_t  awlen, arlen, wstrb;
    bool     wlast;
};
static PreEdge pre;

// Clear all inputs to safe defaults.
static void clear_inputs() {
    dut->s_awid = 0; dut->s_awaddr = 0; dut->s_awlen = 0;
    dut->s_awsize = 0; dut->s_awburst = 0; dut->s_awlock = 0;
    dut->s_awcache = 0; dut->s_awprot = 0; dut->s_awqos = 0;
    dut->s_awuser = 0; dut->s_awvalid = 0;
    dut->s_wdata = 0; dut->s_wstrb = 0; dut->s_wlast = 0;
    dut->s_wuser = 0; dut->s_wvalid = 0;
    dut->s_bready = 0;
    dut->s_arid = 0; dut->s_araddr = 0; dut->s_arlen = 0;
    dut->s_arsize = 0; dut->s_arburst = 0; dut->s_arlock = 0;
    dut->s_arcache = 0; dut->s_arprot = 0; dut->s_arqos = 0;
    dut->s_aruser = 0; dut->s_arvalid = 0;
    dut->s_rready = 0;

    dut->m_awready = 0;
    dut->m_wready  = 0;
    dut->m_bid = 0; dut->m_bresp = 0; dut->m_buser = 0; dut->m_bvalid = 0;
    dut->m_arready = 0;
    dut->m_rid = 0; dut->m_rdata = 0; dut->m_rresp = 0; dut->m_rlast = 0;
    dut->m_ruser = 0; dut->m_rvalid = 0;
}

// Post-deassert settle must clear each underlying async_fifo's
// REQ_MIN_HOLD (rtl/board/async_fifo.v) saturating counter: req_w/req_r
// stay asserted for AT LEAST REQ_MIN_HOLD=8 local cycles after latching,
// regardless of ack, once the raw s_rst/m_rst pulse itself has ended.
// 8 cycles (the pre-T6-review-round-3 value) is exactly the minimum
// with no margin — bumped to 20 for headroom over the 8-cycle floor
// plus the ack round trip.
static void reset() {
    dut->s_clk = 0; dut->m_clk = 0;
    dut->s_rst = 1; dut->m_rst = 1;
    clear_inputs();
    for (int i = 0; i < 8; i++) {
        dut->s_clk = 1; dut->m_clk = 1; eval();
        dut->s_clk = 0; dut->m_clk = 0; eval();
    }
    dut->s_rst = 0; dut->m_rst = 0;
    // Clear software slave model so tests are independent.
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();
    slv.readiness_cycle = 0;
    slave_drive_idle();
    for (int i = 0; i < 20; i++) {
        dut->s_clk = 1; dut->m_clk = 1; eval();
        dut->s_clk = 0; dut->m_clk = 0; eval();
    }
}

#define CHECK(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s: `%s`\n", name, #cond); \
        return false; \
    } \
} while (0)
#define CHECK_EQ(name, got, exp) do { \
    uint64_t _g = (uint64_t)(got), _e = (uint64_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%llx expected 0x%llx\n", \
               name, (unsigned long long)_g, (unsigned long long)_e); \
        return false; \
    } \
} while (0)

// Slave-side model — split into two phases:
//   slave_pre_edge()  : call BEFORE an m_clk posedge.  Updates ready
//                       flags and sets m_bvalid/m_rvalid/m_rdata
//                       combinationally based on current slave state
//                       and FIFO outputs already visible at the bridge.
//   slave_post_edge() : call AFTER an m_clk posedge.  Latches any
//                       accepted AW/W/AR/B/R into the software state.
// Outside of an m_clk edge we only need to keep ready-flags deasserted
// and reuse the last-set *valid* outputs so the bridge sees stable
// signals between edges.
static void slave_drive_combinational();
static void slave_latch_edge();

// Idle drive: keep ready flags low so no transfer is accepted.
static void slave_drive_idle() {
    dut->m_awready = 0;
    dut->m_wready  = 0;
    dut->m_arready = 0;
    // bvalid / rvalid persist from previous state (registered).
}

static void slave_drive_combinational() {
    bool ok = slv.data_ready_this_cycle();
    dut->m_awready = ok ? 1 : 0;
    dut->m_wready  = ok ? 1 : 0;
    dut->m_arready = ok ? 1 : 0;

    // B drive: if a B is pending (aw got all its w beats), set bvalid.
    // This is already handled in slave_latch_edge() which pre-arms
    // bvalid after latching the final W of a burst.

    // R drive: if there's an R in progress, present the current beat.
    if (!slv.r_pending.empty()) {
        auto& rq = slv.r_pending.front();
        uint64_t addr = rq.addr + rq.burst_beat_idx * 8;
        uint64_t v = slv.mem.count(addr) ? slv.mem[addr] : 0;
        dut->m_rid    = rq.id;
        dut->m_rdata  = v;
        dut->m_rresp  = 0;
        dut->m_rlast  = (rq.burst_beat_idx == rq.len);
        dut->m_ruser  = 0;
        dut->m_rvalid = 1;
    }
}

static void slave_latch_edge() {
    // We use pre-edge snapshots because the posedge itself changes many
    // of these signals (e.g. m_wvalid drops to 0 if this was the last
    // beat in the fifo).  The snapshots capture the intent at the edge.

    // 1. Handle previously-driven B: if it was popped by the bridge B
    //    fifo, clear.
    if (pre.b) {
        dut->m_bvalid = 0;
    }
    // 2. Handle previously-driven R accept.
    if (pre.r) {
        if (!slv.r_pending.empty()) {
            auto& rq = slv.r_pending.front();
            if (rq.burst_beat_idx >= rq.len) {
                slv.r_pending.pop_front();
                dut->m_rvalid = 0;
            } else {
                rq.burst_beat_idx++;
            }
        } else {
            dut->m_rvalid = 0;
        }
    }

    // 3-5. AW/W/AR capture using the pre-edge snapshots — post-posedge
    //      payload signals may have shifted (bridge-side pointer
    //      advanced), so we use the snapshots.
    if (pre.aw) {
        SlaveModel::AWQ aw;
        aw.id   = pre.awid;
        aw.addr = pre.awaddr;
        aw.len  = pre.awlen;
        slv.aw_pending.push_back(aw);
    }
    if (pre.w) {
        SlaveModel::WQ wq;
        wq.data = pre.wdata;
        wq.strb = pre.wstrb;
        wq.last = pre.wlast;
        slv.w_pending.push_back(wq);
    }
    if (pre.ar) {
        SlaveModel::RQ rq;
        rq.id   = pre.arid;
        rq.addr = pre.araddr;
        rq.len  = pre.arlen;
        rq.burst_beat_idx = 0;
        slv.r_pending.push_back(rq);
    }

    // 6. Match any aw_pending with enough w_pending → commit to memory
    //    and set m_bvalid.  We do this AFTER the B-accept check above,
    //    so a bvalid set here persists through the next posedge.
    while (!slv.aw_pending.empty() && !dut->m_bvalid) {
        auto& aw = slv.aw_pending.front();
        if (slv.w_pending.size() < (size_t)(aw.len + 1)) break;
        for (int i = 0; i <= aw.len; i++) {
            auto& wq = slv.w_pending.front();
            slv.mem[aw.addr + i * 8] = wq.data;
            slv.w_pending.pop_front();
        }
        dut->m_bid = aw.id;
        dut->m_bresp = 0;
        dut->m_buser = 0;
        dut->m_bvalid = 1;
        slv.aw_pending.pop_front();
        break;
    }
}

// Proper pre-edge / post-edge pattern.  Before each m_clk posedge we
// drive the slave combinational outputs; after each m_clk posedge we
// latch the handshake state.  s_clk edges don't touch the slave model
// state directly — only the bridge's s-side FIFOs.
//
// CRITICAL: the m-side and s-side must NOT coalesce multiple posedges
// into a single "step" on either side.  The master driver (do_master_*)
// samples (valid, ready, data) after EACH step and makes a handshake
// decision — so coalescing multiple s_clk edges into one step_sN_m1()
// call causes multi-beat acceptance to collapse into one beat with the
// wrong data.  The fix is to advance EXACTLY ONE edge on the edge the
// master cares about, interleaving the other side's edges in between
// via a phase counter.

static void do_m_edge() {
    slave_drive_combinational();
    // Snapshot handshake outcomes that WILL happen at the posedge.
    pre.aw = dut->m_awvalid && dut->m_awready;
    pre.w  = dut->m_wvalid  && dut->m_wready;
    pre.ar = dut->m_arvalid && dut->m_arready;
    pre.b  = dut->m_bvalid  && dut->m_bready;
    pre.r  = dut->m_rvalid  && dut->m_rready;
    if (pre.aw) {
        pre.awid   = dut->m_awid;
        pre.awaddr = dut->m_awaddr;
        pre.awlen  = dut->m_awlen;
    }
    if (pre.w) {
        pre.wdata = dut->m_wdata;
        pre.wstrb = dut->m_wstrb;
        pre.wlast = dut->m_wlast;
    }
    if (pre.ar) {
        pre.arid   = dut->m_arid;
        pre.araddr = dut->m_araddr;
        pre.arlen  = dut->m_arlen;
    }
    dut->m_clk = 1; eval();
    slave_latch_edge();
    dut->m_clk = 0; eval();
    slave_drive_idle();
    eval();
    m_phase++;
}
static void do_s_edge() {
    dut->s_clk = 1; eval();
    dut->s_clk = 0; eval();
    s_phase++;
}

// "Step" = advance ONE clock on the domain the master is watching,
// while keeping the other domain running at the requested ratio in the
// interleaved background.  ratio_mode=0: equal-rate, step both by 1.
// ratio_mode=1: s is slow (1×), m is fast (r×).  Step s by 1 every r
// m-side edges.  ratio_mode=2: s is fast (r×), m is slow (1×).  Step m
// by 1 every r s-side edges.  We always advance the SIDE THE MASTER
// IS ABOUT TO SAMPLE by exactly one edge, and drain the other side as
// needed to maintain the ratio.

static void step_both() {
    // 1:1 — advance both one edge, m first then s (or together).
    do_m_edge();
    do_s_edge();
}
// 1 s-edge + enough m-edges to maintain ratio.  Master is SLOW — it
// wants to sample after this one s_edge.
static void step_s_slow(int r) {
    for (int i = 0; i < r; i++) do_m_edge();
    do_s_edge();
}
// 1 s-edge + one m_edge every `r` s_edges.  Master is FAST — it wants
// to sample after EACH s_edge, so we must not coalesce multiple s_edges
// into one "step".  The m-side runs at 1/r the rate.
static void step_s_fast(int r) {
    do_s_edge();
    if ((s_phase % r) == 0) do_m_edge();
}

// ─── Master-side helpers: blocking AW/W/AR/R/B ────────────────────────
static bool do_master_write(uint64_t addr, const std::vector<uint64_t>& data,
                            uint8_t id, int ratio_mode = 0) {
    uint8_t len = (uint8_t)(data.size() - 1);
    // Phase 1: present AW.
    dut->s_awid    = id;
    dut->s_awaddr  = addr;
    dut->s_awlen   = len;
    dut->s_awsize  = 3;  // 8 bytes
    dut->s_awburst = 1;  // INCR
    dut->s_awvalid = 1;

    int guard = 5000;
    while (guard--) {
        // Clock one edge; then inspect the handshake latch.  A posedge
        // with valid+ready high accepts the beat — we must drop valid
        // BEFORE the next posedge to avoid double-enqueueing.
        if (ratio_mode == 0) step_both();
        else if (ratio_mode == 1) step_s_slow(3);
        else if (ratio_mode == 2) step_s_fast(3);
        if (dut->s_awready) {
            // Accept: drop awvalid.
            dut->s_awvalid = 0;
            break;
        }
    }
    dut->s_awvalid = 0;

    // Phase 2: W beats — one beat per (valid+ready) cycle.
    // AXI handshake rule: a beat is accepted on the SAME posedge where
    // valid+ready are both high.  The sequence here: present valid+wdata,
    // tick a single clock, check ready.  If ready was high at that
    // posedge, the beat was accepted; drop valid and advance to next
    // word.  Critically we must only tick ONCE per iteration so we don't
    // accidentally push the same word twice.
    size_t wbeats = data.size();
    size_t w_i = 0;
    dut->s_wvalid = 0;
    guard = 50000;
    while (guard-- && w_i < wbeats) {
        dut->s_wdata  = data[w_i];
        dut->s_wstrb  = 0xFF;
        dut->s_wlast  = (w_i == wbeats - 1);
        dut->s_wvalid = 1;
        // Sample wready BEFORE the tick — that tells us whether THIS
        // posedge will accept the beat.
        bool ready_now = dut->s_wready;
        if (ratio_mode == 0) step_both();
        else if (ratio_mode == 1) step_s_slow(3);
        else if (ratio_mode == 2) step_s_fast(3);
        if (ready_now) {
            w_i++;
            dut->s_wvalid = 0;
        }
    }
    dut->s_wvalid = 0; dut->s_wlast = 0;

    // Phase 3: wait for B.  We hold bready=1 until we see bvalid, which
    // means the posedge that asserted bvalid also accepted it (b_fifo
    // rd_en = bvalid_i & bready).
    dut->s_bready = 1;
    guard = 10000;
    bool got_b = false;
    while (guard--) {
        if (ratio_mode == 0) step_both();
        else if (ratio_mode == 1) step_s_slow(3);
        else if (ratio_mode == 2) step_s_fast(3);
        if (dut->s_bvalid) {
            got_b = true;
            dut->s_bready = 0;
            break;
        }
    }
    if (!got_b) {
        printf("    DEBUG: B lost. aw_pend=%zu w_pend=%zu mem=%zu wbeats=%zu\n",
               slv.aw_pending.size(), slv.w_pending.size(), slv.mem.size(),
               data.size());
    }
    dut->s_bready = 0;
    // Advance one cycle so m-side sees bvalid dropped.
    if (ratio_mode == 0) step_both();
    else if (ratio_mode == 1) step_s_slow(3);
    else if (ratio_mode == 2) step_s_fast(3);
    return got_b;
}

static bool do_master_read(uint64_t addr, int beats, std::vector<uint64_t>& out,
                           uint8_t id, int ratio_mode = 0) {
    uint8_t len = (uint8_t)(beats - 1);
    dut->s_arid    = id;
    dut->s_araddr  = addr;
    dut->s_arlen   = len;
    dut->s_arsize  = 3;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    int guard = 5000;
    while (guard--) {
        if (ratio_mode == 0) step_both();
        else if (ratio_mode == 1) step_s_slow(3);
        else if (ratio_mode == 2) step_s_fast(3);
        if (dut->s_arready) {
            dut->s_arvalid = 0;
            break;
        }
    }
    dut->s_arvalid = 0;

    // Accept R beats.  Each cycle with rvalid+rready yields one beat.
    dut->s_rready = 1;
    out.clear();
    guard = 20000;
    while (guard--) {
        if (ratio_mode == 0) step_both();
        else if (ratio_mode == 1) step_s_slow(3);
        else if (ratio_mode == 2) step_s_fast(3);
        if (dut->s_rvalid) {
            out.push_back(dut->s_rdata);
            bool is_last = dut->s_rlast;
            (void)is_last;
            if (is_last) {
                dut->s_rready = 0;
                break;
            }
            (void)is_last;
        }
    }
    dut->s_rready = 0;
    // Advance a cycle so m-side settles.
    if (ratio_mode == 0) step_both();
    else if (ratio_mode == 1) step_s_slow(3);
    else if (ratio_mode == 2) step_s_fast(3);
    return (int)out.size() == beats;
}

// ═══════════════════════════════════════════════════════════════════════
// 1. single_read_1beat
// ═══════════════════════════════════════════════════════════════════════
static bool test_single_read_1beat() {
    reset();
    slv.mem.clear();
    slv.mem[0x1000] = 0xDEADBEEFCAFEBABEULL;
    std::vector<uint64_t> got;
    if (!do_master_read(0x1000, 1, got, 5)) {
        printf("  FAIL: read did not produce expected beats\n");
        return false;
    }
    CHECK_EQ("data", got[0], 0xDEADBEEFCAFEBABEULL);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 2. single_write_1beat
// ═══════════════════════════════════════════════════════════════════════
static bool test_single_write_1beat() {
    reset();
    slv.mem.clear();
    std::vector<uint64_t> d = {0xFEEDFACEDEADBEEFULL};
    if (!do_master_write(0x2000, d, 7)) {
        printf("  FAIL: write did not produce B\n");
        return false;
    }
    CHECK("memory updated", slv.mem.count(0x2000) == 1);
    CHECK_EQ("memory value", slv.mem[0x2000], 0xFEEDFACEDEADBEEFULL);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 3. burst_read_8beat
// ═══════════════════════════════════════════════════════════════════════
static bool test_burst_read_8beat() {
    reset();
    slv.mem.clear();
    for (int i = 0; i < 8; i++) {
        slv.mem[0x3000 + i*8] = 0x1000ULL * i + 0xA0ULL;
    }
    std::vector<uint64_t> got;
    if (!do_master_read(0x3000, 8, got, 2)) {
        printf("  FAIL: burst read size mismatch (%zu)\n", got.size());
        return false;
    }
    for (int i = 0; i < 8; i++) {
        CHECK_EQ("burst beat", got[i], 0x1000ULL * i + 0xA0ULL);
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 4. burst_write_8beat
// ═══════════════════════════════════════════════════════════════════════
static bool test_burst_write_8beat() {
    reset();
    slv.mem.clear();
    std::vector<uint64_t> d;
    for (int i = 0; i < 8; i++) d.push_back(0xB00BULL * (i+1));
    if (!do_master_write(0x4000, d, 3)) {
        printf("  FAIL: burst write no B\n");
        return false;
    }
    for (int i = 0; i < 8; i++) {
        CHECK_EQ("beat in mem", slv.mem[0x4000 + i*8], 0xB00BULL * (i+1));
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 5. read_write_interleave — alternate reads and writes, check order.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read_write_interleave() {
    reset();
    slv.mem.clear();
    for (int iter = 0; iter < 4; iter++) {
        std::vector<uint64_t> d = {0xC0DEULL + iter};
        if (!do_master_write(0x5000 + iter * 8, d, iter)) return false;
        std::vector<uint64_t> got;
        if (!do_master_read(0x5000 + iter * 8, 1, got, iter)) return false;
        CHECK_EQ("interleave rdback", got[0], 0xC0DEULL + iter);
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 6. far_side_backpressure — slave-side ready toggles every other cycle.
// ═══════════════════════════════════════════════════════════════════════
static bool test_far_side_backpressure() {
    reset();
    slv.mem.clear();
    slv.backpressure_enabled = true;
    slv.readiness_cycle = 0;
    // A shorter burst reduces guard iterations needed at 50% accept rate.
    std::vector<uint64_t> d;
    for (int i = 0; i < 4; i++) d.push_back(0xBEEFULL * (i+1));
    if (!do_master_write(0x6000, d, 1)) {
        slv.backpressure_enabled = false;
        printf("  FAIL: write under backpressure\n");
        return false;
    }
    std::vector<uint64_t> got;
    if (!do_master_read(0x6000, 4, got, 1)) {
        slv.backpressure_enabled = false;
        printf("  FAIL: read under backpressure\n");
        return false;
    }
    for (int i = 0; i < 4; i++) {
        CHECK_EQ("bp beat", got[i], 0xBEEFULL * (i+1));
    }
    slv.backpressure_enabled = false;
    return true;
}

// Start a transaction, reset both domains before the m-side can drain it,
// then prove the bridge comes back clean and does not replay stale FIFO
// contents.
static bool test_reset_clears_inflight() {
    reset();
    slv.mem.clear();
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();

    dut->s_awid    = 6;
    dut->s_awaddr  = 0x9000;
    dut->s_awlen   = 0;
    dut->s_awsize  = 3;
    dut->s_awburst = 1;
    dut->s_awlock  = 0;
    dut->s_awcache = 0;
    dut->s_awprot  = 0;
    dut->s_awqos   = 0;
    dut->s_awuser  = 0;
    dut->s_awvalid = 1;
    dut->s_wdata   = 0x123456789ABCDEF0ULL;
    dut->s_wstrb   = 0xFF;
    dut->s_wlast   = 1;
    dut->s_wuser   = 0;
    dut->s_wvalid  = 1;
    dut->s_bready  = 1;
    dut->s_arvalid = 0;
    dut->s_rready  = 1;

    // One s-clock edge is enough to load the bridge FIFOs; the downstream
    // m-side has not been clocked yet, so nothing should have committed.
    do_s_edge();
    dut->s_awvalid = 0;
    dut->s_wvalid  = 0;

    dut->s_rst = 1;
    dut->m_rst = 1;
    for (int i = 0; i < 4; i++) {
        do_m_edge();
        do_s_edge();
    }
    bool during_reset = (dut->s_awready == 0) && (dut->s_wready == 0) &&
                        (dut->s_arready == 0) && (dut->m_awvalid == 0) &&
                        (dut->m_wvalid == 0) && (dut->m_arvalid == 0);
    dut->s_rst = 0;
    dut->m_rst = 0;
    clear_inputs();
    slave_drive_idle();
    eval();
    // Post-deassert settle must clear each underlying async_fifo's
    // REQ_MIN_HOLD=8-cycle minimum request hold (see reset() above for
    // the full rationale) before the "prove fresh traffic works" phase
    // below starts a new transaction.
    for (int i = 0; i < 20; i++) {
        do_m_edge();
        do_s_edge();
    }
    bool drained = slv.mem.empty() && slv.aw_pending.empty() &&
                   slv.w_pending.empty() && slv.r_pending.empty();

    // Prove the bridge can accept fresh traffic again.
    std::vector<uint64_t> d = {0xAA55AA55AA55AA55ULL};
    bool recovered = do_master_write(0xA000, d, 4) &&
                     slv.mem.count(0xA000) == 1 &&
                     slv.mem[0xA000] == 0xAA55AA55AA55AA55ULL;
    return during_reset && drained && recovered;
}

// ═══════════════════════════════════════════════════════════════════════
// 7. slow_to_fast — s_clk 1×, m_clk 3×
// ═══════════════════════════════════════════════════════════════════════
static bool test_slow_to_fast() {
    reset();
    slv.mem.clear();
    std::vector<uint64_t> d = {0x11ULL, 0x22ULL, 0x33ULL, 0x44ULL};
    if (!do_master_write(0x7000, d, 2, /*ratio_mode=*/1)) {
        printf("  FAIL: slow-fast write\n"); return false;
    }
    std::vector<uint64_t> got;
    if (!do_master_read(0x7000, 4, got, 2, /*ratio_mode=*/1)) {
        printf("  FAIL: slow-fast read got %zu beats\n", got.size());
        for (size_t i = 0; i < got.size(); i++)
            printf("    beat[%zu] = 0x%llx\n", i,
                   (unsigned long long)got[i]);
        return false;
    }
    for (int i = 0; i < 4; i++) CHECK_EQ("sf beat", got[i], d[i]);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 8. fast_to_slow — s_clk 3×, m_clk 1×
// ═══════════════════════════════════════════════════════════════════════
static bool test_fast_to_slow() {
    reset();
    slv.mem.clear();
    std::vector<uint64_t> d = {0xAAULL, 0xBBULL, 0xCCULL, 0xDDULL};
    if (!do_master_write(0x8000, d, 3, /*ratio_mode=*/2)) {
        printf("  FAIL: fast-slow write\n"); return false;
    }
    std::vector<uint64_t> got;
    if (!do_master_read(0x8000, 4, got, 3, /*ratio_mode=*/2)) {
        printf("  FAIL: fast-slow read\n"); return false;
    }
    for (int i = 0; i < 4; i++) CHECK_EQ("fs beat", got[i], d[i]);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 9. mid_transaction_mside_reset — T6 CDC/reset-hardening regression.
// ═══════════════════════════════════════════════════════════════════════
static bool test_mid_transaction_mside_reset() {
    reset();
    slv.mem.clear();
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();

    // Warm up: complete an ordinary write+read round trip first so the
    // per-channel async_fifo pointers (s_clk-write/m_clk-read on
    // AW/W/AR, m_clk-write/s_clk-read on B/R) have already advanced
    // together to a nonzero-but-equal baseline before the one-sided
    // reset hits.  A reset from an all-zero baseline never distinguishes
    // coupled vs. uncoupled RTL; a nonzero baseline does.
    std::vector<uint64_t> warm = {0x1010101010101010ULL};
    CHECK("warm-up write completes", do_master_write(0xB000, warm, 5));
    std::vector<uint64_t> warm_rd;
    CHECK("warm-up read completes", do_master_read(0xB000, 1, warm_rd, 5));
    CHECK_EQ("warm-up data", warm_rd[0], warm[0]);

    // Start a fresh write burst from the s side and get a couple of
    // beats queued into the bridge before the m-side reset hits.
    // s_rst stays low throughout — the upstream master never sees
    // anything happen; this models a MIG UI recalibration pulse that
    // only touches the downstream (m) side.
    dut->s_awid    = 7;
    dut->s_awaddr  = 0xB100;
    dut->s_awlen   = 3;      // 4 beats
    dut->s_awsize  = 3;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    dut->s_wdata   = 0x2222222222222222ULL;
    dut->s_wstrb   = 0xFF;
    dut->s_wlast   = 0;
    dut->s_wvalid  = 1;
    dut->s_bready  = 1;
    for (int i = 0; i < 3; i++) do_s_edge();
    dut->s_awvalid = 0;

    // Assert ONLY m_rst.
    dut->m_rst = 1;
    for (int i = 0; i < 6; i++) { do_m_edge(); do_s_edge(); }
    dut->m_rst = 0;
    // Let both sides settle out of the coupled reset.
    for (int i = 0; i < 6; i++) { do_m_edge(); do_s_edge(); }

    dut->s_wvalid = 0; dut->s_wlast = 0; dut->s_bready = 0;
    clear_inputs();
    slave_drive_idle();
    eval();
    for (int i = 0; i < 4; i++) { do_m_edge(); do_s_edge(); }

    // No protocol corruption left over on the s side from the aborted
    // in-flight burst — no stuck valid.
    CHECK("s_bvalid not stuck high after m-reset settle", dut->s_bvalid == 0);
    CHECK("s_rvalid not stuck high after m-reset settle", dut->s_rvalid == 0);

    // Prove the bridge fully recovers: a fresh write, then a fresh
    // burst read, both complete correctly with no corruption.
    slv.mem.clear();
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();
    std::vector<uint64_t> wd = {0xCAFEBABE00000001ULL, 0xCAFEBABE00000002ULL,
                                 0xCAFEBABE00000003ULL, 0xCAFEBABE00000004ULL};
    CHECK("post-reset burst write completes", do_master_write(0xC000, wd, 9));
    for (size_t i = 0; i < wd.size(); i++)
        CHECK_EQ("post-reset mem beat", slv.mem[0xC000 + i * 8], wd[i]);

    std::vector<uint64_t> rd;
    CHECK("post-reset burst read completes", do_master_read(0xC000, 4, rd, 9));
    for (int i = 0; i < 4; i++) CHECK_EQ("post-reset read beat", rd[i], wd[i]);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 10. sside_only_reset_stale_read_sink — T14 regression: an outstanding
//     AR burst (already accepted by the m-side slave) at the moment of
//     an s-side-ONLY reset (m_rst stays low, matching a JTAG/core-only
//     reset with the DDR MIG continuing to run -- see docs t14-report.md
//     ROUND 2/3).  async_fifo's own T6 coupled-reset handshake correctly
//     relays the reset and zeroes BOTH the R-fifo's wptr and rptr
//     together (no pointer desync) -- but it has no AXI burst semantics.
//     The downstream slave is never told to abandon the burst and keeps
//     trying to deliver its remaining R beats; once the handshake
//     settles and m_rready returns high, those pre-reset beats get
//     accepted into the freshly-zeroed fifo and forwarded to the s-side
//     as if they were brand new.  This proves the bridge must not let
//     ANY of that stale burst's beats leak into a later, unrelated read.
// ═══════════════════════════════════════════════════════════════════════
static bool test_sside_only_reset_stale_read_sink() {
    reset();
    slv.mem.clear();
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();

    // Warm-up round trip so the per-channel fifo pointers start from a
    // nonzero baseline (see mid_transaction_mside_reset rationale above).
    std::vector<uint64_t> warm = {0x5A5A5A5A5A5A5A5AULL};
    CHECK("warm-up write completes", do_master_write(0xD800, warm, 5));
    std::vector<uint64_t> warm_rd;
    CHECK("warm-up read completes", do_master_read(0xD800, 1, warm_rd, 5));

    // Stage the OLD (soon-to-be-abandoned) burst: 8 beats, distinctive
    // 0xDEAD.... pattern so any leak is unmistakable.
    for (int i = 0; i < 8; i++) slv.mem[0xD000 + i * 8] = 0xDEAD000000000000ULL + i;

    dut->s_arid    = 8;
    dut->s_araddr  = 0xD000;
    dut->s_arlen   = 7;   // 8 beats
    dut->s_arsize  = 3;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 0;   // do NOT drain -- reset must hit before any beat
                           // is consumed by the s side.
    int guard = 2000;
    bool ar_accepted = false;
    while (guard--) {
        step_both();
        if (dut->s_arready) { dut->s_arvalid = 0; ar_accepted = true; break; }
    }
    CHECK("old AR accepted by bridge", ar_accepted);
    dut->s_arvalid = 0;

    // Let a few m-edges run so the slave forwards the AR and starts
    // pushing beats into the bridge's R fifo (write side) -- unread,
    // since s_rready is still 0 -- BEFORE the reset hits.  This creates
    // the "buffered-but-undrained, about-to-be-pointer-erased-then-
    // re-arrive" window that the T6 handshake alone does not protect
    // against (see round-2 root-cause finding in t14-report.md).
    for (int i = 0; i < 3; i++) do_m_edge();

    // Assert s_rst ONLY. m_rst stays low the whole time -- the downstream
    // slave is never told to abandon the burst and will keep trying to
    // deliver its 8 beats via m_rvalid throughout this window.
    dut->s_rst = 1;
    for (int i = 0; i < 40; i++) { do_m_edge(); do_s_edge(); }
    dut->s_rst = 0;
    // Settle past each async_fifo's REQ_MIN_HOLD=8 floor plus the ack
    // round trip -- keep pumping m-edges so the sink logic has time to
    // absorb the full owed burst, all while s_rready stays 0.
    for (int i = 0; i < 60; i++) { do_m_edge(); do_s_edge(); }

    dut->s_wvalid = 0; dut->s_wlast = 0; dut->s_bready = 0;
    clear_inputs();
    slave_drive_idle();
    eval();
    for (int i = 0; i < 10; i++) { do_m_edge(); do_s_edge(); }

    // Fresh, distinguishable burst at an unrelated address.
    for (int i = 0; i < 4; i++) slv.mem[0xE000 + i * 8] = 0xF00D000000000000ULL + i;
    std::vector<uint64_t> got;
    bool fresh_ok = do_master_read(0xE000, 4, got, 9);
    CHECK("fresh post-reset read completes", fresh_ok);
    for (int i = 0; i < 4; i++) {
        CHECK_EQ("fresh beat (no stale leak)", got[i], 0xF00D000000000000ULL + i);
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 11. sside_only_reset_stale_write_sink — B-channel analog: an AW+W
//     burst fully forwarded and committed by the m-side slave, whose
//     BRESP is still outstanding at the moment of an s-side-ONLY reset.
//     Same stale-replay exposure as the read side, mirrored on B (the
//     CPU write path goes through this same bridge -- see t14-report.md
//     ROUND 3 authorization).
// ═══════════════════════════════════════════════════════════════════════
static bool test_sside_only_reset_stale_write_sink() {
    reset();
    slv.mem.clear();
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();

    std::vector<uint64_t> warm = {0x6B6B6B6B6B6B6B6BULL};
    CHECK("warm-up write completes", do_master_write(0xD900, warm, 5));

    // Fill the B fifo just short of its usable capacity with filler
    // writes that each fully complete (bridge-accepted into the B
    // fifo, confirmed via m_bvalid clearing) before the next is fired,
    // so the fifo state is unambiguous.  Empirically 3 fillers exactly
    // saturate this B_DEPTH_LOG2=2 fifo's usable slots (the registered
    // wr_full_r in async_fifo.v lags the raw full condition by one
    // cycle) -- verified below via an explicit per-filler drain CHECK,
    // so if that margin ever changes this test fails loudly instead of
    // silently mis-measuring.  With the fifo saturated, the "real" old
    // write's BRESP (queued next) is forced to genuinely stall at the
    // slave boundary (m_bvalid=1, m_bready=0 from real backpressure,
    // not from any sink logic) -- the same "presented-but-not-yet-
    // accepted, mid-transit" shape as the confirmed read-side root
    // cause, deterministically, instead of racing a single beat against
    // multi-cycle CDC-relay latency.
    dut->s_bready = 0;
    for (int i = 0; i < 3; i++) {
        // Fire-and-forget AW+W (do_master_write would drain its own B
        // via s_bready=1, defeating the fill-up here).
        dut->s_awid = 1; dut->s_awaddr = 0xD100 + i * 8; dut->s_awlen = 0;
        dut->s_awsize = 3; dut->s_awburst = 1; dut->s_awvalid = 1;
        int fguard = 2000; bool faw_ok = false;
        while (fguard--) { step_both(); if (dut->s_awready) { dut->s_awvalid = 0; faw_ok = true; break; } }
        CHECK("filler AW accepted", faw_ok);
        dut->s_awvalid = 0;
        dut->s_wdata = 0x1111000000000000ULL + i; dut->s_wstrb = 0xFF; dut->s_wlast = 1;
        dut->s_wvalid = 1;
        fguard = 2000; bool fw_ok = false;
        while (fguard--) {
            bool ready_now = dut->s_wready;
            step_both();
            if (ready_now) { dut->s_wvalid = 0; fw_ok = true; break; }
        }
        CHECK("filler W accepted", fw_ok);
        dut->s_wvalid = 0; dut->s_wlast = 0;
        // Let THIS filler's BRESP arm, then confirm it gets accepted
        // into the B fifo (m_bvalid clears) before firing the next one
        // -- proceed one at a time, deterministically, instead of
        // racing multiple in-flight CDC forwards.
        int settle_guard = 200;
        while (settle_guard-- && !dut->m_bvalid) do_m_edge();
        CHECK("filler BRESP armed", dut->m_bvalid == 1);
        settle_guard = 200;
        while (settle_guard-- && dut->m_bvalid) do_m_edge();
        CHECK("filler BRESP drained into B fifo (fifo not yet saturated)",
              dut->m_bvalid == 0);
    }

    // Old (soon-to-be-abandoned) single-beat write, fully forwarded
    // (AW+W both accepted by the m side) so the slave commits it to
    // memory and arms m_bvalid -- but we never drain s_bready, so the
    // BRESP is still outstanding when the s-only reset hits.
    dut->s_awid = 8; dut->s_awaddr = 0xD400; dut->s_awlen = 0;
    dut->s_awsize = 3; dut->s_awburst = 1; dut->s_awvalid = 1;
    dut->s_bready = 0;
    int guard = 2000; bool aw_ok = false;
    while (guard--) { step_both(); if (dut->s_awready) { dut->s_awvalid = 0; aw_ok = true; break; } }
    CHECK("old AW accepted", aw_ok);
    dut->s_awvalid = 0;

    dut->s_wdata = 0xDEADDEADDEADDEADULL; dut->s_wstrb = 0xFF; dut->s_wlast = 1;
    dut->s_wvalid = 1;
    guard = 2000; bool w_ok = false;
    while (guard--) {
        bool ready_now = dut->s_wready;
        step_both();
        if (ready_now) { dut->s_wvalid = 0; w_ok = true; break; }
    }
    CHECK("old W accepted", w_ok);
    dut->s_wvalid = 0; dut->s_wlast = 0;

    // Let m-edges run, POLLING for m_bvalid, so we assert reset the
    // instant the slave has forwarded+matched+committed the AW+W and
    // armed its BRESP.  With the B fifo already full from the filler
    // writes above, this beat is now GENUINELY stalled at the slave
    // boundary (m_bready=0 from real backpressure) -- reset lands on a
    // response that is provably still mid-transit, matching the
    // coordinator's described shape and the confirmed read-side root
    // cause, rather than a still-buffered-and-unforwarded AW/W (that
    // narrower case is already covered by reset_clears_inflight's
    // full-reset scenario and is a distinct, AW/AR-request-side
    // exposure noted separately in t14-report.md).
    int arm_guard = 200;
    while (arm_guard-- && !dut->m_bvalid) do_m_edge();
    CHECK("old BRESP armed pre-reset (mid-transit, not yet fifo-accepted)",
          dut->m_bvalid == 1);
    // Confirm it's genuinely THE OLD WRITE's own BRESP (id=8), not a
    // leftover filler -- the fillers all drained (checked above) before
    // this AW+W was even issued, so this also catches any regression in
    // that draining discipline.
    CHECK_EQ("armed BRESP belongs to the old write (id=8)", dut->m_bid, 8);
    CHECK("old write committed to memory pre-reset", slv.mem.count(0xD400) == 1);

    // Assert s_rst ONLY, before draining B.  m_rst stays low -- the
    // slave is unaware and will complete + present the stale BRESP
    // regardless.
    dut->s_rst = 1;
    for (int i = 0; i < 40; i++) { do_m_edge(); do_s_edge(); }
    dut->s_rst = 0;
    for (int i = 0; i < 60; i++) { do_m_edge(); do_s_edge(); }

    dut->s_bready = 0;
    clear_inputs();
    slave_drive_idle();
    eval();
    for (int i = 0; i < 10; i++) { do_m_edge(); do_s_edge(); }

    // Direct check: no stale BRESP should be sitting in the bridge's B
    // fifo before we've issued any post-reset transaction at all.  This
    // is the load-bearing assertion -- do_master_write()'s own success
    // check below only proves "a B arrived", which would be a false
    // pass if a stale, wrong-ID BRESP got silently consumed instead of
    // the fresh write's own.
    CHECK("no stale BRESP present before any post-reset write is issued",
          dut->s_bvalid == 0);

    // Fresh, unrelated write must complete cleanly -- its OWN BRESP must
    // arrive (not be swallowed by a stale one), and memory must reflect
    // only the fresh value.
    slv.mem.erase(0xD400);
    std::vector<uint64_t> freshd = {0xCAFEF00DCAFEF00DULL};
    bool fresh_ok = do_master_write(0xDA00, freshd, 9);
    CHECK("fresh post-reset write completes (B not swallowed by stale)", fresh_ok);
    CHECK_EQ("received BRESP belongs to the fresh write (id=9, not stale id=8)",
             dut->s_bid, 9);
    CHECK("fresh mem committed", slv.mem.count(0xDA00) == 1);
    CHECK_EQ("fresh mem value", slv.mem[0xDA00], freshd[0]);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 12. double_reset_no_debt_overcount -- full-branch-review I1.
// axi_bridge_stale_sink.v's `debt` used to ACCUMULATE `outstanding` on
// every reset_event instead of snapshotting it: with one burst still
// genuinely undrained (debt=1, outstanding=1) from a FIRST s-only
// reset, a SECOND s-only reset landing inside that ~100-cycle drain
// window (JTAG double-pulse territory) computed debt=1+1=2 instead of
// debt=1 -- double-counting the SAME undrained burst. Once that one
// stale burst genuinely completes, sinking stayed active for one beat
// too many, silently eating the NEXT, completely unrelated fresh
// response. This proves a genuinely fresh, distinguishable read issued
// after a double reset arrives intact.
// ═══════════════════════════════════════════════════════════════════════
static bool test_double_reset_no_debt_overcount() {
    reset();
    slv.mem.clear();
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();

    std::vector<uint64_t> warm = {0x5A5A5A5A5A5A5A5AULL};
    CHECK("warm-up write completes", do_master_write(0xD800, warm, 5));
    std::vector<uint64_t> warm_rd;
    CHECK("warm-up read completes", do_master_read(0xD800, 1, warm_rd, 5));

    // Stage a 256-beat burst (ARLEN's max) -- once `sinking` engages
    // after the FIRST reset, the sink mechanism accepts (and discards)
    // m-side beats unconditionally regardless of r_fifo fullness (that
    // is the whole point of the fix: it bypasses ordinary backpressure
    // so the abandoned burst drains fast) -- so a SHORT burst reaches
    // RLAST and re-zeroes debt within just a few cycles of the first
    // reset, well before a second reset could ever land while debt is
    // still nonzero (confirmed by direct SINKDBG instrumentation during
    // triage: an 8-beat burst fully drained to debt=0 within 3 cycles
    // of the FIRST reset alone). A 256-beat burst, draining at roughly
    // 1 beat/cycle once sinking engages, cannot possibly finish within
    // the short window between the two resets below, guaranteeing debt
    // is still genuinely nonzero when the second reset lands.
    dut->s_arid    = 8;
    dut->s_araddr  = 0xD000;
    dut->s_arlen   = 255; // 256 beats
    dut->s_arsize  = 3;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 0;   // never drain from the s side -- both resets must
                           // catch this burst outstanding purely via the
                           // sink mechanism, not an s-side read
    int guard = 2000;
    bool ar_accepted = false;
    while (guard--) {
        step_both();
        if (dut->s_arready) { dut->s_arvalid = 0; ar_accepted = true; break; }
    }
    CHECK("old AR accepted by bridge", ar_accepted);
    dut->s_arvalid = 0;

    // Let the AR forward to the slave; a handful of beats buffer into
    // the R fifo pre-reset (irrelevant to the count below -- the T6
    // handshake's own pointer-zero already wipes whatever's buffered
    // there on every reset; only what the slave still owes AFTER that
    // wipe is what outstanding/debt track).
    for (int i = 0; i < 5; i++) do_m_edge();

    // FIRST s-only reset.
    dut->s_rst = 1;
    for (int i = 0; i < 20; i++) { do_m_edge(); do_s_edge(); }
    dut->s_rst = 0;
    // Brief settle -- short enough that a 256-beat burst, even
    // draining at ~1 beat/cycle once sinking kicks in, cannot possibly
    // reach RLAST yet.
    for (int i = 0; i < 15; i++) { do_m_edge(); do_s_edge(); }

    // SECOND s-only reset, landing squarely inside the first reset's
    // drain window -- debt is still genuinely nonzero here.
    dut->s_rst = 1;
    for (int i = 0; i < 20; i++) { do_m_edge(); do_s_edge(); }
    dut->s_rst = 0;

    // Full settle -- long enough for the entire 256-beat stale burst
    // to drain via the sink mechanism regardless of whether the I1 bug
    // is present; the difference under test is whether sinking
    // OVER-drains afterward (stuck asserted for one extra completion
    // it doesn't own), eating one extra, unrelated fresh beat.
    for (int i = 0; i < 400; i++) { do_m_edge(); do_s_edge(); }

    dut->s_wvalid = 0; dut->s_wlast = 0; dut->s_bready = 0;
    clear_inputs();
    slave_drive_idle();
    eval();
    for (int i = 0; i < 10; i++) { do_m_edge(); do_s_edge(); }

    // Fresh, distinguishable read at an unrelated address must arrive
    // intact -- if debt over-counted (the I1 bug), sinking would still
    // be (incorrectly) active and would eat this fresh burst instead.
    for (int i = 0; i < 4; i++) slv.mem[0xE100 + i * 8] = 0xF00D000000000000ULL + i;
    std::vector<uint64_t> got;
    bool fresh_ok = do_master_read(0xE100, 4, got, 9);
    CHECK("fresh post-double-reset read completes", fresh_ok);
    for (int i = 0; i < 4; i++) {
        CHECK_EQ("fresh beat (not eaten by debt over-count)", got[i], 0xF00D000000000000ULL + i);
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// 13. sside_reset_mid_w_burst_pad — task #162.  The case
// axi_bridge_stale_sink.v's header explicitly documents as NOT handled:
// an s-side-only reset landing MID-W-BURST.
//
// The slave has accepted an AW and k of that burst's N beats when the
// reset lands.  The remaining N-k beats are gone forever (the s-side W
// fifo's pointers were zeroed by the T6 coupled-reset handshake, and
// nothing re-sends them), so the slave stays parked mid-burst.  AW and W
// ride independent async_fifos, so a fresh, unrelated write issued
// afterwards gets its AW accepted while its W beats flow into a slave
// still counting down the OLD burst.  The W channel carries no address
// and no ID -- only FIFO-ordered data -- so the slave commits the fresh
// write's data to the OLD address, and the AW/W pairing is permanently
// shifted from then on: EVERY subsequent write silently corrupts, with
// no error and no BRESP anywhere to signal it.
//
// The fix (rtl/soc/axi_bridge_w_pad.v) completes the abandoned burst on
// the m-side with wstrb=0 filler beats and correct WLAST, so the slave's
// write state machine genuinely finishes and returns to idle before any
// fresh W data is admitted.  The BRESP that completion produces is
// already owed to axi_bridge_stale_sink's `debt` and gets sunk.
//
// Swept across every mid-burst landing point k = 0 .. N-1 (k=0 is the
// "AW accepted, no W beats yet" corner, which is the same hazard).
// ═══════════════════════════════════════════════════════════════════════
static bool midw_pad_one_case(int k, int nbeats) {
    reset();
    slv.mem.clear();
    slv.aw_pending.clear();
    slv.w_pending.clear();
    slv.r_pending.clear();

    // Warm-up round trip so the per-channel fifo pointers start from a
    // nonzero baseline (same rationale as the tests above).
    std::vector<uint64_t> warm = {0x7C7C7C7C7C7C7C7CULL};
    CHECK("warm-up write completes", do_master_write(0xD700, warm, 5));

    // OLD burst: AW only so far -- N beats at 0xB000, id=8.
    dut->s_awid    = 8;
    dut->s_awaddr  = 0xB000;
    dut->s_awlen   = (uint8_t)(nbeats - 1);
    dut->s_awsize  = 3;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    int guard = 2000;
    bool aw_ok = false;
    while (guard--) {
        step_both();
        if (dut->s_awready) { dut->s_awvalid = 0; aw_ok = true; break; }
    }
    CHECK("old AW accepted by bridge", aw_ok);
    dut->s_awvalid = 0;
    for (int i = 0; i < 40; i++) { do_m_edge(); do_s_edge(); }
    CHECK("old AW actually forwarded to the slave", slv.aw_pending.size() == 1);

    // Feed exactly k of the burst's N beats, and confirm the SLAVE has
    // them -- the reset must land with the slave genuinely mid-burst,
    // not merely with data buffered inside the bridge.
    for (int i = 0; i < k; i++) {
        dut->s_wdata = 0xDEAD000000000000ULL + (uint64_t)i;
        dut->s_wstrb = 0xFF;
        dut->s_wlast = (i == nbeats - 1);
        dut->s_wvalid = 1;
        guard = 2000;
        bool beat_ok = false;
        while (guard--) {
            bool ready_now = dut->s_wready;
            step_both();
            if (ready_now) { dut->s_wvalid = 0; beat_ok = true; break; }
        }
        CHECK("old W beat accepted by bridge", beat_ok);
        dut->s_wvalid = 0; dut->s_wlast = 0;
    }
    for (int i = 0; i < 40; i++) { do_m_edge(); do_s_edge(); }
    CHECK("slave is parked exactly k beats into the old burst",
          (int)slv.w_pending.size() == k);
    CHECK("old burst has NOT completed at the slave (still mid-burst)",
          slv.aw_pending.size() == 1);

    // s_rst ONLY.  m_rst stays low -- the slave is never told to abandon
    // anything and keeps waiting for the burst's remaining beats.
    dut->s_rst = 1;
    for (int i = 0; i < 40; i++) { do_m_edge(); do_s_edge(); }
    dut->s_rst = 0;
    for (int i = 0; i < 150; i++) { do_m_edge(); do_s_edge(); }

    clear_inputs();
    slave_drive_idle();
    eval();
    for (int i = 0; i < 10; i++) { do_m_edge(); do_s_edge(); }

    // Load-bearing, and the part that is impossible without filler
    // completion: the slave must be back at IDLE -- the abandoned burst
    // fully consumed -- before any fresh write is issued.
    CHECK("abandoned burst was completed on the m-side (no AW left parked)",
          slv.aw_pending.empty());
    CHECK("abandoned burst consumed all its W beats (slave back to idle)",
          slv.w_pending.empty());
    // ...and its BRESP must have been sunk, not replayed upstream.
    CHECK("no stale BRESP leaked to the s side", dut->s_bvalid == 0);

    // Fresh, unrelated 2-beat write.  Must land completely, at its OWN
    // address, and collect its OWN BRESP.
    std::vector<uint64_t> fresh = {0xCAFE000000000001ULL, 0xCAFE000000000002ULL};
    CHECK("fresh post-reset write completes", do_master_write(0xC000, fresh, 9));
    CHECK_EQ("BRESP belongs to the fresh write (id=9)", dut->s_bid, 9);
    CHECK("fresh beat 0 committed at its own address", slv.mem.count(0xC000) == 1);
    CHECK_EQ("fresh beat 0 value", slv.mem[0xC000], fresh[0]);
    CHECK("fresh beat 1 committed at its own address", slv.mem.count(0xC008) == 1);
    CHECK_EQ("fresh beat 1 value", slv.mem[0xC008], fresh[1]);

    // Nothing from the fresh write may have been misattributed into the
    // abandoned burst's address range -- that misattribution IS the bug.
    for (int i = 0; i < nbeats; i++) {
        uint64_t a = 0xB000 + (uint64_t)i * 8;
        uint64_t v = slv.mem.count(a) ? slv.mem[a] : 0ULL;
        CHECK("fresh write data must not land at the abandoned burst's address",
              v != fresh[0] && v != fresh[1]);
    }
    return true;
}

static bool test_sside_reset_mid_w_burst_pad() {
    const int NBEATS = 8;
    bool all_ok = true;
    for (int k = 0; k < NBEATS; k++) {
        if (!midw_pad_one_case(k, NBEATS)) {
            printf("    ^ above failure was for reset landing after beat %d of %d\n",
                   k, NBEATS);
            all_ok = false;
        }
    }
    return all_ok;
}

// ─── Runner ────────────────────────────────────────────────────────────
static void run(const char* name, bool (*fn)()) {
    bool r = fn();
    printf("[%s] %s\n", r ? "PASS" : "FAIL", name);
    if (r) n_pass++; else n_fail++;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaxi_async_bridge();

    run("single_read_1beat",       test_single_read_1beat);
    run("single_write_1beat",      test_single_write_1beat);
    run("burst_read_8beat",        test_burst_read_8beat);
    run("burst_write_8beat",       test_burst_write_8beat);
    run("read_write_interleave",   test_read_write_interleave);
    run("far_side_backpressure",   test_far_side_backpressure);
    run("reset_clears_inflight",   test_reset_clears_inflight);
    run("slow_to_fast",            test_slow_to_fast);
    run("fast_to_slow",            test_fast_to_slow);
    run("mid_transaction_mside_reset", test_mid_transaction_mside_reset);
    run("sside_only_reset_stale_read_sink",  test_sside_only_reset_stale_read_sink);
    run("sside_only_reset_stale_write_sink", test_sside_only_reset_stale_write_sink);
    run("double_reset_no_debt_overcount", test_double_reset_no_debt_overcount);
    run("sside_reset_mid_w_burst_pad", test_sside_reset_mid_w_burst_pad);

    printf("\naxi_async_bridge: %d PASS / %d FAIL\n", n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
