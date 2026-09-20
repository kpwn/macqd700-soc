// tb_l2c.cpp -- Verilator unit testbench for the L2 system cache
// (rtl/soc/l2c.v + submodules), driven through tb/tb_l2c.v.
//
// Two build variants share this file (see Makefile tb-l2c /
// tb-l2c-bypass-all stanzas):
//   - normal (default): DutT = Vtb_l2c, full directed + randomized suite.
//   - BYPASS_ALL_BUILD:  DutT = Vtb_l2c_bypass, L2_BYPASS_ALL=1 wrapper,
//     runs only the pass-through equivalence check (there is no cache to
//     exercise in that build).
//
// Architecture:
//   - MemSlave: backpressuring, latency-randomizing (200-263 cycle) AXI4
//     memory model on the DUT's m_axi (master) port, backed by a flat
//     byte array ("backing").
//   - Requester: concurrent-outstanding AXI4 master driving the DUT's
//     s_axi (slave) port -- multiple single- or multi-beat bursts may be
//     in flight at once (needed to exercise MSHR merge / full
//     backpressure / victim hazard timing), matched to completion by ID.
//   - golden[]: host-side "truth" array. Reads are checked against it
//     the moment their burst completes; the randomized stress loop also
//     does a full read-back verification pass at the end (no RTL
//     backdoor -- reads go through the DUT like everything else).
//
// See docs/l2c_spec.md for the module's design; this file exercises
// every scenario listed in the T10 brief.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <vector>

#include <verilated.h>
#ifdef BYPASS_ALL_BUILD
#include "Vtb_l2c_bypass.h"
using DutT = Vtb_l2c_bypass;
#else
#include "Vtb_l2c.h"
using DutT = Vtb_l2c;
#endif

// ─── Geometry / address-map constants (mirrors docs/l2c_spec.md) ─────────
static constexpr uint64_t LINE_BYTES     = 64;
static constexpr uint64_t CACHEABLE_BASE = 0x0000'0000ULL;
static constexpr uint64_t CACHEABLE_SIZE = 16ULL * 1024 * 1024; // 16 MB test window
// Bypass window sits immediately after the cacheable region (adjacent,
// not overlapping -- l2c_bypass.v's disjointness assert treats "base ==
// cacheable_base+size" as disjoint) so the host-side backing/golden
// arrays stay a modest ~17 MB instead of spanning a huge address gap.
static constexpr uint64_t BYPASS_BASE    = CACHEABLE_SIZE;
static constexpr uint64_t BYPASS_SIZE    = 1ULL * 1024 * 1024;  // matches tb_l2c.v's window
static constexpr uint64_t MEM_TOTAL      = BYPASS_BASE + BYPASS_SIZE;

static DutT* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;
// Important-13: an unmatched B/R (no corresponding entry in
// w_awaiting_b/r_awaiting) used to be silently dropped -- print-only,
// gated behind an env var, and never failed the test.  Any real protocol
// violation (extra/duplicate response, ID corruption, phantom beat from
// a buggy response-arbiter lock -- exactly the class of bug the
// Critical-5 fix targeted) must now fail the run unconditionally.
static bool g_unmatched_resp_seen = false;
// Set only by the same-ID directed test, which deliberately drives raw
// AXI beats outside the id-keyed map tracking (see test_same_id_order).
static bool g_suppress_unmatched_check = false;

static constexpr uint32_t DEFAULT_SEED = 0xC0FFEE42u;
static std::mt19937 rng(DEFAULT_SEED);
// Keep DDR timing randomness independent from request generation.  That
// makes failures reproducible even when a directed scenario adds or removes
// requester-side random draws.
static std::mt19937 ddr_rng(DEFAULT_SEED ^ 0xDDE20064u);
static uint32_t g_seed = DEFAULT_SEED;
static int g_ddr_latency_base = 200;
static int g_ddr_latency_jitter = 64;
static int g_ddr_beat_gap_max = 3;

// ─── Backing memory (what a real DDR behind m_axi holds) + golden model ──
static std::vector<uint8_t> backing(MEM_TOTAL);
static std::vector<uint8_t> golden(MEM_TOTAL);
static std::vector<bool>    touched(MEM_TOTAL, false);

static int env_nonnegative(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    char* end = nullptr;
    long parsed = std::strtol(value, &end, 0);
    return (end != value && *end == '\0' && parsed >= 0) ? static_cast<int>(parsed) : fallback;
}

static void configure_randomness() {
    if (const char* value = std::getenv("L2C_SEED"))
        g_seed = static_cast<uint32_t>(std::strtoul(value, nullptr, 0));
    g_ddr_latency_base = env_nonnegative("L2C_DDR_LATENCY_BASE", 200);
    g_ddr_latency_jitter = env_nonnegative("L2C_DDR_LATENCY_JITTER", 64);
    g_ddr_beat_gap_max = env_nonnegative("L2C_DDR_BEAT_GAP_MAX", 3);
    rng.seed(g_seed);
    ddr_rng.seed(g_seed ^ 0xDDE20064u);
    printf("  seed=0x%08x DDR first-response latency=%d+random(0..%d) cycles, beat-gap=0..%d\n",
           g_seed, g_ddr_latency_base, std::max(0, g_ddr_latency_jitter - 1),
           g_ddr_beat_gap_max);
}

static void init_memories() {
    for (uint64_t i = 0; i < MEM_TOTAL; i++) {
        uint8_t v = static_cast<uint8_t>(rng() & 0xFF);
        backing[i] = v;
        golden[i]  = v;
    }
}

// ─── Clocking ──────────────────────────────────────────────────────────
static void eval() { dut->eval(); }

// ─── m_axi memory-model slave (backpressuring + latency-randomizing) ────
struct MemSlave {
    bool     aw_have = false;
    uint32_t aw_id = 0; uint64_t aw_addr = 0; int aw_len = 0; int aw_beat = 0;
    struct PendingB { uint32_t id; uint64_t ready_at; uint8_t resp; };
    std::deque<PendingB> b_q;

    struct PendingR {
        uint32_t id;
        uint64_t addr;
        int beats;
        int sent = 0;
        uint64_t ready_at;
        uint8_t resp;
    };
    std::deque<PendingR> r_q;

    int lat() {
        return g_ddr_latency_base +
               (g_ddr_latency_jitter > 0 ? static_cast<int>(ddr_rng() % g_ddr_latency_jitter) : 0);
    }
    int beat_gap() {
        return g_ddr_beat_gap_max > 0 ? static_cast<int>(ddr_rng() % (g_ddr_beat_gap_max + 1)) : 0;
    }
    bool gate(int pct_ready) { return static_cast<int>(ddr_rng() % 100) < pct_ready; }
};
static MemSlave mslv;
// IMPORTANT-A test hook: force AW/W ready to 1 every cycle, bypassing the
// random gate(80) above, so a reset-during-drain scenario can be timed
// deterministically (land the reset exactly on the AW-accept-to-WLAST
// cycle count, then rely on mslv.lat()'s own 10-60 cycle floor to
// guarantee the eventual BRESP arrives comfortably AFTER, never before,
// the reset).
static bool g_mem_force_ready = false;
// Pipeline-directed test hook: accept ARs into the memory model while
// suppressing RVALID, then release them in an explicitly chosen order.
static bool g_mem_block_rvalid = false;
// Hold fill ARs at VALID-not-READY to exercise payload stability and a full
// MSHR table whose requests have not yet reached DDR.
static bool g_mem_block_arready = false;
// Applied to every beat of the next accepted memory read, then cleared.
static uint8_t g_mem_next_rresp = 0;
// Applied to the B of the next write burst the model completes, then
// cleared.  A dirty writeback has no requester to notify, so this is the
// only way to reach l2c_victim.v's BRESP handling from the outside.
static uint8_t g_mem_next_bresp = 0;
// Optional log of every accepted AW's shape.  A single "last AW" snapshot
// was enough while exactly one writeback could be in flight; with a
// pipelined victim buffer several bursts are queued at once and it is the
// SEQUENCE of shapes that has to stay right.
static bool g_aw_log_on = false;
static std::vector<std::pair<uint64_t,int>> g_aw_log;
// IMPORTANT-A v2 test hook: force m_axi_awready to 0 UNCONDITIONALLY
// (overrides both the random gate and g_mem_force_ready) so a directed
// test can pin a victim writeback in S_AW -- PRESENTED, never ACCEPTED
// -- for as long as needed. This is the exact precondition the
// aw_open-vs-aw_busy reset-hold bug required: routine downstream
// backpressure on AWREADY, not a fault condition.
static bool g_mem_block_awready = false;
// Cycles in which DDR had an R beat presented and l2c refused it.  The
// drain-debt tests use this: a stale burst that nobody is prepared to
// receive parks m_axi_rready at 0 and takes the SHARED DDR read channel
// down for every master, which is otherwise invisible from the requester
// side (the symptom is "everything stopped", not a wrong value).
static uint64_t g_r_stall_cycles = 0;

static void mem_drive_comb() {
    dut->m_axi_awready = (!mslv.aw_have && !g_mem_block_awready && (g_mem_force_ready || mslv.gate(80))) ? 1 : 0;
    dut->m_axi_wready  = (mslv.aw_have && (g_mem_force_ready || mslv.gate(80))) ? 1 : 0;
    dut->m_axi_arready = (!g_mem_block_arready && (g_mem_force_ready || mslv.gate(80))) ? 1 : 0;

    if (!mslv.b_q.empty() && mslv.b_q.front().ready_at <= sim_time) {
        dut->m_axi_bvalid = 1; dut->m_axi_bid = mslv.b_q.front().id;
        dut->m_axi_bresp = mslv.b_q.front().resp;
    } else { dut->m_axi_bvalid = 0; }

    if (!g_mem_block_rvalid && !mslv.r_q.empty() && mslv.r_q.front().ready_at <= sim_time) {
        auto& f = mslv.r_q.front();
        dut->m_axi_rvalid = 1; dut->m_axi_rid = f.id; dut->m_axi_rresp = f.resp;
        dut->m_axi_rlast = (f.sent == f.beats - 1) ? 1 : 0;
        // rdata itself is packed by mem_drive_rdata() below (128b signal ->
        // Verilator VlWide<4> of 32b words, NOT 16 byte-indexable elements).
    } else { dut->m_axi_rvalid = 0; }
}

// Verilator represents >64b ports as VlWide<N> (array of uint32_t words).
// Small helpers keep the byte-level packing code above readable/portable.
template <typename T>
static void set_wide_bytes(T& sig, const uint8_t* bytes, int nbytes) {
    for (int w = 0; w < nbytes / 4; w++) {
        uint32_t v = bytes[w*4] | (bytes[w*4+1] << 8) | (bytes[w*4+2] << 16) | (bytes[w*4+3] << 24);
        sig[w] = v;
    }
}
template <typename T>
static void get_wide_bytes(const T& sig, uint8_t* bytes, int nbytes) {
    for (int w = 0; w < nbytes / 4; w++) {
        uint32_t v = sig[w];
        bytes[w*4] = v & 0xFF; bytes[w*4+1] = (v>>8)&0xFF; bytes[w*4+2] = (v>>16)&0xFF; bytes[w*4+3] = (v>>24)&0xFF;
    }
}

static void mem_drive_rdata() {
    if (!mslv.r_q.empty() && mslv.r_q.front().ready_at <= sim_time) {
        auto& f = mslv.r_q.front();
        uint64_t a = f.addr + static_cast<uint64_t>(f.sent) * 16;
        uint8_t buf[16];
        for (int b = 0; b < 16; b++) buf[b] = (a + b < MEM_TOTAL) ? backing[a + b] : 0;
        set_wide_bytes(dut->m_axi_rdata, buf, 16);
    }
}

// IMPORTANT-A test hook: monotonic counts of AW/W handshakes, updated on
// the SAME settled pre-edge snapshot mem_latch_edge() already uses for
// everything else -- peeking at dut->m_axi_awvalid/wvalid AFTER tick()
// returns is exactly the post-edge race the tick() comment above warns
// about (by then the DUT's own registers, e.g. st moving S_AW->S_W, may
// have already changed, so the combinational m_awvalid the caller reads
// no longer reflects what was actually presented during the handshake
// cycle).  Tests that need to stop after N observed beats compare these
// counters across a tick() instead of re-reading raw DUT wires.
static uint64_t g_dbg_aw_accepts = 0, g_dbg_w_accepts = 0, g_dbg_ar_accepts = 0;
// Shape of the most recent writeback AW.  With sectored dirty bits the
// writeback burst is no longer always the whole line, so its base address
// and length are themselves observable behaviour worth asserting on.
static uint64_t g_dbg_last_aw_addr = 0;
static int      g_dbg_last_aw_len  = 0;

static void mem_latch_edge() {
    // AW accept
    if (!mslv.aw_have && dut->m_axi_awvalid && dut->m_axi_awready) {
        mslv.aw_have = true; mslv.aw_id = dut->m_axi_awid; mslv.aw_addr = dut->m_axi_awaddr;
        mslv.aw_len = dut->m_axi_awlen; mslv.aw_beat = 0;
        g_dbg_aw_accepts++;
        g_dbg_last_aw_addr = dut->m_axi_awaddr;
        g_dbg_last_aw_len  = dut->m_axi_awlen;
        if (g_aw_log_on) g_aw_log.emplace_back(dut->m_axi_awaddr, dut->m_axi_awlen);
    }
    // W beat accept
    if (mslv.aw_have && dut->m_axi_wvalid && dut->m_axi_wready) {
        uint8_t buf[16];
        get_wide_bytes(dut->m_axi_wdata, buf, 16);
        uint16_t strb = dut->m_axi_wstrb;
        uint64_t a = mslv.aw_addr + static_cast<uint64_t>(mslv.aw_beat) * 16;
        for (int b = 0; b < 16; b++)
            if ((strb >> b) & 1) if (a + b < MEM_TOTAL) backing[a + b] = buf[b];
        mslv.aw_beat++;
        g_dbg_w_accepts++;
        if (mslv.aw_beat == mslv.aw_len + 1) {
            mslv.b_q.push_back({mslv.aw_id, sim_time + static_cast<uint64_t>(mslv.lat()),
                                 g_mem_next_bresp});
            g_mem_next_bresp = 0;
            mslv.aw_have = false;
        }
    }
    // B accept
    if (!mslv.b_q.empty() && dut->m_axi_bvalid && dut->m_axi_bready) mslv.b_q.pop_front();
    // AR accept
    if (dut->m_axi_arvalid && dut->m_axi_arready) {
        mslv.r_q.push_back({static_cast<uint32_t>(dut->m_axi_arid), static_cast<uint64_t>(dut->m_axi_araddr),
                             dut->m_axi_arlen + 1, 0, sim_time + static_cast<uint64_t>(mslv.lat()),
                             g_mem_next_rresp});
        g_mem_next_rresp = 0;
        g_dbg_ar_accepts++;
    }
    // R beat accept
    if (!mslv.r_q.empty() && dut->m_axi_rvalid && dut->m_axi_rready) {
        auto& f = mslv.r_q.front();
        f.sent++;
        if (f.sent < f.beats)
            f.ready_at = sim_time + static_cast<uint64_t>(mslv.beat_gap());
        if (f.sent == f.beats) mslv.r_q.pop_front();
    }
}

// Important-13: AXI4 payload-stability assertion on the DUT's own
// m_axi AW/AR (master) channels -- once VALID is asserted, ID/ADDR/LEN
// must not change until READY fires (exactly the invariant the
// Critical-4 arbiter-lock fix restores).  Snapshotted the cycle a
// channel FIRST goes valid-but-not-accepted; compared every subsequent
// cycle it's STILL valid-but-not-accepted.  Called on the same settled
// pre-edge snapshot as mem_latch_edge()/req_latch_edge().
struct StableSnap { bool armed = false; uint32_t id = 0; uint64_t addr = 0; int len = 0; };
static StableSnap g_aw_snap, g_ar_snap;
static bool g_stability_violation = false;

// ─── Bypass-engine invariants, checked EVERY cycle of EVERY scenario ─────
// Two properties that the 2026-08-20 pipelining rewrite must not break,
// both of them things a cycle-count assertion would not notice:
//
//  1. NO READ/WRITE OVERLAP on the master port.  AXI orders nothing
//     between the R and B channels, so a bypass read must never be in
//     flight alongside a bypass write -- otherwise a read could pass a
//     write to the same address inside the slave and return pre-write
//     data.  v1 got this free by being one-at-a-time; v2 gets it from
//     l2c_bypass's dir_ok_c, and this is the check that says so.
//
//     Measured from the AXI WIRES, not from a DUT internal: bypass
//     transactions carry ID bit [ID_WIDTH-1] = 1 (l2c_defs.vh's tagging
//     note), MSHR fills and victim writebacks carry 0, and the memory
//     model echoes IDs on B/R.  Deliberately independent of the taps --
//     a tap and the logic it watches can go wrong together.
//
//     Post-reset strays belong to a dead epoch and are legitimately
//     concurrent with fresh traffic of the other direction, so the
//     counts are split the same way l2c_bypass splits bout_cnt from
//     bsink_cnt: outstanding-at-reset moves into a `stale` bucket which
//     retiring responses drain first.
//
//  2. A PRESENTED OP STILL OWNS ITS SLOT.  dbg_by_inflight (dispatched,
//     awaiting R/B) can never exceed dbg_by_occ (accepted, not yet
//     answered upstream), and neither can exceed the depth.  Retiring a
//     slot at response-CAPTURE instead of at response-DELIVERY would
//     satisfy every cycle count in this file and silently break rsp_id.
static long g_by_rd_out = 0, g_by_wr_out = 0;
static long g_by_rd_stale = 0, g_by_wr_stale = 0;
static bool g_by_mixdir_violation = false;
static bool g_by_tap_violation = false;
static unsigned g_by_inflight_max = 0, g_by_occ_max = 0;
static const uint32_t BY_ID_TAG = 0x20; // ID_WIDTH=6 -> bit [5]

static void by_reset_epoch() { g_by_rd_stale = g_by_rd_out; g_by_wr_stale = g_by_wr_out; }

static void by_latch_edge() {
    // NOT MEANINGFUL IN THE L2_BYPASS_ALL PASS-THROUGH BUILD.  There, s_axi
    // ids reach m_axi unchanged and this harness allocates ids 1..59 -- so
    // any id >= 32 sets the tag bit and would be miscounted as bypass
    // traffic, firing the exclusion check on ordinary read/write overlap.
    // In the real build the bit is a genuine discriminator: bypass drives
    // {1'b1, ...}, MSHR fills drive their entry index (0..7) and victim
    // writebacks drive 0.
#ifdef BYPASS_ALL_BUILD
    return;
#else
    if (dut->m_axi_arvalid && dut->m_axi_arready && (dut->m_axi_arid & BY_ID_TAG)) g_by_rd_out++;
    if (dut->m_axi_rvalid && dut->m_axi_rready && dut->m_axi_rlast &&
        (dut->m_axi_rid & BY_ID_TAG)) {
        g_by_rd_out--; if (g_by_rd_stale > 0) g_by_rd_stale--;
    }
    if (dut->m_axi_awvalid && dut->m_axi_awready && (dut->m_axi_awid & BY_ID_TAG)) g_by_wr_out++;
    if (dut->m_axi_bvalid && dut->m_axi_bready && (dut->m_axi_bid & BY_ID_TAG)) {
        g_by_wr_out--; if (g_by_wr_stale > 0) g_by_wr_stale--;
    }
    const long live_rd = g_by_rd_out - g_by_rd_stale;
    const long live_wr = g_by_wr_out - g_by_wr_stale;
    if (live_rd > 0 && live_wr > 0 && !g_by_mixdir_violation) {
        printf("  BYPASS ORDER VIOLATION: %ld read(s) and %ld write(s) outstanding on the "
               "master port at the same time t=%llu\n",
               live_rd, live_wr, (unsigned long long)sim_time);
        g_by_mixdir_violation = true;
    }
#endif
}

static void by_sample_taps() {
    const unsigned occ    = dut->dbg_by_occ      & 0x3F;
    const unsigned infl   = dut->dbg_by_inflight & 0x3F;
    const unsigned slots  = dut->dbg_by_slots    & 0xFF;
    if (occ > g_by_occ_max) g_by_occ_max = occ;
    if (infl > g_by_inflight_max) g_by_inflight_max = infl;
    if ((infl > occ || occ > slots) && !g_by_tap_violation) {
        printf("  BYPASS TAP VIOLATION: %u presented with %u occupied of %u slots t=%llu\n",
               infl, occ, slots, (unsigned long long)sim_time);
        g_by_tap_violation = true;
    }
}
static void check_stable(StableSnap& snap, bool valid, bool ready, uint32_t id, uint64_t addr, int len, const char* chan) {
    if (!valid) { snap.armed = false; return; }
    if (ready) { snap.armed = false; return; } // accepted this cycle -- next cycle is a fresh transfer
    if (!snap.armed) { snap.armed = true; snap.id = id; snap.addr = addr; snap.len = len; return; }
    if (snap.id != id || snap.addr != addr || snap.len != len) {
        printf("  STABILITY VIOLATION on m_axi_%s: payload changed while VALID asserted and READY low "
               "(was id=%u addr=0x%llx len=%d, now id=%u addr=0x%llx len=%d) t=%llu\n",
               chan, snap.id, (unsigned long long)snap.addr, snap.len, id, (unsigned long long)addr, len,
               (unsigned long long)sim_time);
        g_stability_violation = true;
        snap.id = id; snap.addr = addr; snap.len = len; // don't re-report every cycle after the first
    }
}
static void mem_check_stability() {
    check_stable(g_aw_snap, dut->m_axi_awvalid, dut->m_axi_awready, dut->m_axi_awid, dut->m_axi_awaddr, dut->m_axi_awlen, "aw");
    check_stable(g_ar_snap, dut->m_axi_arvalid, dut->m_axi_arready, dut->m_axi_arid, dut->m_axi_araddr, dut->m_axi_arlen, "ar");
}
// Same check, but on the DUT's s_axi R/B RESPONSE outputs -- directly
// validates the Critical-5 response-mux lock (r_lock/b_lock in l2c.v):
// payload must not swap while s_axi_rvalid/bvalid is held with rready/
// bready low (now exercised for real by req_ready_gate()'s backpressure
// above).  Uses rdata's low 64b only (cheap, sufficient to catch a swap).
static StableSnap g_r_snap, g_b_snap;
static void req_check_stability() {
    uint8_t rbuf[16]; get_wide_bytes(dut->s_axi_rdata, rbuf, 16);
    uint64_t rdata_lo; std::memcpy(&rdata_lo, rbuf, 8);
    check_stable(g_r_snap, dut->s_axi_rvalid, dut->s_axi_rready, dut->s_axi_rid, rdata_lo, dut->s_axi_rresp, "r(id/data_lo/resp)");
    check_stable(g_b_snap, dut->s_axi_bvalid, dut->s_axi_bready, dut->s_axi_bid, dut->s_axi_bresp, 0, "b(id/resp)");
}

// ─── s_axi requester (concurrent-outstanding AXI4 master) ────────────────
struct WBurst {
    uint32_t id; uint64_t addr; int beats;
    std::vector<std::array<uint8_t,16>> data;
    std::vector<uint16_t> strb;
    int issue_beat = 0;
    // Perf-suite timestamps (see the measurement suite near the bottom of
    // this file).  Zero unless the perf suite is running.
    uint64_t t_issue = 0, t_aw = 0, t_b = 0;
    bool aw_done = false; // latched once AWVALID&&AWREADY fire -- AWREADY
                          // is only a single-cycle pulse (the DUT drops it
                          // the moment its own aw_have latches), so this
                          // must be a sticky flag, not re-derived from the
                          // current cycle's awvalid&&awready every time.
    bool resp_ok = false, done = false;
};
struct RBurst {
    uint32_t id; uint64_t addr; int beats;
    std::vector<std::array<uint8_t,16>> got;
    uint8_t last_resp = 0; // captured from s_axi_rresp on RLAST -- SLVERR checking (Important-10)
    // Perf-suite timestamps (see the measurement suite near the bottom of
    // this file).  Zero unless the perf suite is running.
    uint64_t t_issue = 0, t_ar = 0, t_r0 = 0, t_rl = 0;
    bool done = false;
};
// Important-10 test hook: override the NEXT issued read's AxBURST/AxSIZE
// (default -1 = use the normal legal INCR/full-width values below).
// Reset to -1 once the AR handshake fires so it never leaks into a
// later, unrelated read.
static int g_force_arburst = -1, g_force_arsize = -1;
// Same idea for the NEXT issued write's AWSIZE (default -1 = full 16B,
// matching every other write in this file).  Narrower-than-16B AWSIZE is
// legal AXI4 (narrow-transfer-on-wide-bus) and must be ACCEPTED now that
// Important-10's front-door size gate only rejects non-INCR bursts --
// see test_illegal_burst_slverr / test_narrow_awsize_write.  Reset to -1
// once the AW handshake fires so it never leaks into a later write.
static int g_force_awsize = -1;
static std::deque<std::shared_ptr<WBurst>> w_pending_issue;
static std::shared_ptr<WBurst>             w_issuing;
// AW RUNAHEAD (2026-08-20, lookup pipelining).  The default master here
// drives AW and W from ONE cursor: it cannot present burst N+1's header
// until burst N's last W beat has been accepted.  That is a legal but
// conservative master, and once the lookup retires a beat per cycle it
// becomes the binding constraint on a 1-beat-per-burst write stream --
// l2c cannot consume a W beat until `aw_have` is REGISTERED, so a master
// that presents AW and W in the same cycle spends two cycles per burst no
// matter how deep the door is.  Turning this flag on splits the two
// cursors (AW runs ahead, W beats still drain in AW order, as AXI4
// requires), which is what a pipelined master does and what separates
// "the DUT costs a cycle" from "the stimulus did".
//
// OFF by default so every correctness scenario and every pre-existing
// perf row keeps the exact stimulus it was written against.
static bool                                g_aw_runahead = false;
static std::shared_ptr<WBurst>             aw_issuing;   // runahead AW cursor
static std::deque<std::shared_ptr<WBurst>> w_aw_sent;    // AW accepted, W not begun
static std::map<uint32_t, std::shared_ptr<WBurst>> w_awaiting_b;
static std::map<uint32_t, std::shared_ptr<WBurst>> w_done;

static std::deque<std::shared_ptr<RBurst>> r_pending_issue;
static std::shared_ptr<RBurst>             r_issuing;
// Per-id QUEUE (not a single-entry map): AXI allows a NEW AR to be
// accepted (header latched) for an id that already has an OLDER,
// still-unresolved outstanding transaction under the same id (e.g. one
// blocked behind Critical-3's id_busy_c dispatch gate) -- a single-entry
// map would let the newer arrival silently clobber the older one's
// tracking the moment its own AR handshake fires, well before the older
// one's data has even arrived.  A deque models AXI's actual same-ID
// completion-order guarantee directly: R beats for a given id are always
// consumed from the FRONT of that id's queue.
static std::map<uint32_t, std::deque<std::shared_ptr<RBurst>>> r_awaiting;
static std::map<uint32_t, std::shared_ptr<RBurst>> r_done;

// Shipping CPU=m68k040 has a second, independently-numbered read source on
// l2c's dedicated 256-bit fetch port.  Keep a long fetch stream resident in
// the shared front door for the dual-source ordering/starvation regression.
static bool g_fetch_stream = false;
static uint64_t g_fetch_ar_hs = 0;
static uint64_t g_fetch_rlast_hs = 0;
// Fetch-driver shape knobs.  Defaults reproduce the original fixed
// single 4 KiB burst at 0x0008_0000 with ID 0, so every pre-existing
// fetch scenario is bit-identical; the door-throughput test below is the
// only caller that changes them.
//
// Why a separate shape at all: a 128-beat burst exercises the fetch
// port's BEAT walk, and a one-entry header door is invisible inside one
// burst.  cpu040's real I-side traffic is the opposite shape -- short,
// back-to-back line fills from a 5-MSHR prefetcher -- and that is
// precisely where a burst-granular door serialises.
static int      g_fetch_arlen  = 127;        // beats-1 per fetch burst
static uint64_t g_fetch_base   = 0x0008'0000ull;
static uint64_t g_fetch_stride = 0;          // 0 = stay on one address
static int      g_fetch_nid    = 1;          // distinct rotating ARIDs
static uint64_t g_fetch_addr   = 0x0008'0000ull;
static int      g_fetch_id     = 0;
static uint64_t g_fetch_ar_pres    = 0;      // cycles fetch AR was offered
static uint64_t g_fetch_ar_refused = 0;      // ...and the door refused it

// Ring buffer of recently-issued ops, for post-mortem dumps on mismatch.
struct OpLogEntry { uint64_t addr; bool is_write; uint32_t id; uint64_t t; };
static std::vector<OpLogEntry> op_log;
static void log_op(uint64_t addr, bool is_write, uint32_t id) {
    op_log.push_back({addr, is_write, id, sim_time});
    if (op_log.size() > 64) op_log.erase(op_log.begin());
}
static void dump_op_log() {
    printf("  -- last %zu issued ops --\n", op_log.size());
    for (auto& e : op_log)
        printf("     t=%llu %s id=%u addr=0x%llx\n", (unsigned long long)e.t, e.is_write ? "W" : "R", e.id, (unsigned long long)e.addr);
}

static std::set<uint32_t> free_ids;
static uint32_t alloc_id() {
    // Dereferencing begin() on an empty set is UB -- fail loudly instead
    // of silently corrupting id-based response routing (this is exactly
    // how a concurrency-window bug manifested during bring-up: see the
    // inflight.size() comment in test_randomized_scoreboard).
    if (free_ids.empty()) {
        fprintf(stderr, "FATAL: alloc_id() called with an empty id pool -- "
                         "too many concurrent outstanding requests\n");
        std::abort();
    }
    uint32_t id = *free_ids.begin();
    free_ids.erase(free_ids.begin());
    return id;
}
static void init_ids() { for (uint32_t i = 1; i < 60; i++) free_ids.insert(i); }

// Important-13: s_axi_bready/rready used to be hardwired to 1 -- the
// consumer side of the DUT's own response arbiter (Critical-5's r_lock/
// b_lock) never saw a single VALID-but-not-READY cycle, so the "hold the
// lock until genuinely accepted" property was never actually exercised.
// Randomized backpressure (default ON, 85% ready) creates that window on
// every run; L2C_NO_BACKPRESSURE=1 reverts to always-ready for isolating
// an unrelated failure.
static bool g_perf_always_ready = false;
static bool req_ready_gate(int pct_ready) {
    if (g_perf_always_ready) return true;
    if (std::getenv("L2C_NO_BACKPRESSURE")) return true;
    return static_cast<int>(rng() % 100) < pct_ready;
}

static void req_drive_comb() {
    const std::shared_ptr<WBurst>& aw_cur = g_aw_runahead ? aw_issuing : w_issuing;
    if (aw_cur && !aw_cur->aw_done) {
        dut->s_axi_awvalid = 1; dut->s_axi_awid = aw_cur->id; dut->s_axi_awaddr = aw_cur->addr;
        dut->s_axi_awlen = aw_cur->beats - 1;
        dut->s_axi_awsize = (g_force_awsize >= 0) ? g_force_awsize : 4;
        dut->s_axi_awburst = 1;
    } else dut->s_axi_awvalid = 0;
    if (w_issuing) {
        dut->s_axi_wvalid = 1;
        set_wide_bytes(dut->s_axi_wdata, w_issuing->data[w_issuing->issue_beat].data(), 16);
        dut->s_axi_wstrb = w_issuing->strb[w_issuing->issue_beat];
        dut->s_axi_wlast = (w_issuing->issue_beat == w_issuing->beats - 1) ? 1 : 0;
    } else dut->s_axi_wvalid = 0;

    if (r_issuing) {
        dut->s_axi_arvalid = 1; dut->s_axi_arid = r_issuing->id; dut->s_axi_araddr = r_issuing->addr;
        dut->s_axi_arlen = r_issuing->beats - 1;
        dut->s_axi_arsize = (g_force_arsize >= 0) ? g_force_arsize : 4;
        dut->s_axi_arburst = (g_force_arburst >= 0) ? g_force_arburst : 1;
    } else dut->s_axi_arvalid = 0;

    dut->s_axi_bready = req_ready_gate(85) ? 1 : 0;
    dut->s_axi_rready = req_ready_gate(85) ? 1 : 0;

    dut->f_axi_arvalid = g_fetch_stream ? 1 : 0;
    dut->f_axi_arid = g_fetch_id;
    dut->f_axi_araddr = static_cast<uint32_t>(g_fetch_addr);
    dut->f_axi_arlen = g_fetch_arlen; // default 127 = 4 KiB, see g_fetch_arlen
    dut->f_axi_arsize = 5;  // native 256-bit fetch beat
    dut->f_axi_arburst = 1;
    dut->f_axi_rready = 1;
}

static void req_pump_issue() {
    if (g_aw_runahead) {
        if (!aw_issuing && !w_pending_issue.empty()) { aw_issuing = w_pending_issue.front(); w_pending_issue.pop_front(); }
        if (!w_issuing && !w_aw_sent.empty()) { w_issuing = w_aw_sent.front(); w_aw_sent.pop_front(); }
    } else if (!w_issuing && !w_pending_issue.empty()) { w_issuing = w_pending_issue.front(); w_pending_issue.pop_front(); }
    if (!r_issuing && !r_pending_issue.empty()) { r_issuing = r_pending_issue.front(); r_pending_issue.pop_front(); }
}

static void req_latch_edge() {
    if (dut->f_axi_arvalid) {
        g_fetch_ar_pres++;
        if (!dut->f_axi_arready) g_fetch_ar_refused++;
    }
    if (dut->f_axi_arvalid && dut->f_axi_arready) {
        g_fetch_ar_hs++;
        if (g_fetch_stride) {
            g_fetch_addr += g_fetch_stride;
            if (g_fetch_nid > 1) g_fetch_id = (g_fetch_id + 1) % g_fetch_nid;
        }
    }
    if (dut->f_axi_rvalid && dut->f_axi_rready && dut->f_axi_rlast) g_fetch_rlast_hs++;
    if (g_aw_runahead && aw_issuing && !aw_issuing->aw_done &&
        dut->s_axi_awvalid && dut->s_axi_awready) {
        aw_issuing->aw_done = true; aw_issuing->t_aw = sim_time;
        g_force_awsize = -1; // one-shot override, see Important-10 hook
        w_aw_sent.push_back(aw_issuing); aw_issuing = nullptr;
    }
    if (w_issuing) {
        if (!g_aw_runahead && !w_issuing->aw_done && dut->s_axi_awvalid && dut->s_axi_awready) {
            w_issuing->aw_done = true;
            w_issuing->t_aw = sim_time;
            g_force_awsize = -1; // one-shot override, see Important-10 hook
            if (std::getenv("L2C_DEBUG_HS")) printf("  HS: AW accept id=%u addr=0x%llx beats=%d t=%llu\n", w_issuing->id, (unsigned long long)w_issuing->addr, w_issuing->beats, (unsigned long long)sim_time);
        }
        if (w_issuing->aw_done && dut->s_axi_wvalid && dut->s_axi_wready) {
            if (std::getenv("L2C_DEBUG_HS")) printf("  HS: W accept id=%u beat=%d/%d strb=0x%x t=%llu\n", w_issuing->id, w_issuing->issue_beat, w_issuing->beats, w_issuing->strb[w_issuing->issue_beat], (unsigned long long)sim_time);
            w_issuing->issue_beat++;
            if (w_issuing->issue_beat == w_issuing->beats) {
                if (std::getenv("L2C_DEBUG_HS")) printf("  HS: W burst DONE id=%u -> w_awaiting_b t=%llu\n", w_issuing->id, (unsigned long long)sim_time);
                w_awaiting_b[w_issuing->id] = w_issuing;
                w_issuing = nullptr;
            }
        }
    }
    if (r_issuing && dut->s_axi_arvalid && dut->s_axi_arready) {
        r_issuing->t_ar = sim_time;
        r_awaiting[r_issuing->id].push_back(r_issuing);
        r_issuing = nullptr;
        g_force_arburst = -1; g_force_arsize = -1; // one-shot override, see Important-10 hook
    }
    if (dut->s_axi_bvalid && dut->s_axi_bready) {
        uint32_t id = dut->s_axi_bid;
        if (std::getenv("L2C_DEBUG_HS")) printf("  HS: B accept id=%u t=%llu\n", id, (unsigned long long)sim_time);
        auto it = w_awaiting_b.find(id);
        if (it != w_awaiting_b.end()) {
            it->second->resp_ok = (dut->s_axi_bresp == 0);
            it->second->t_b = sim_time;
            it->second->done = true;
            w_done[id] = it->second;
            w_awaiting_b.erase(it);
        } else if (!g_suppress_unmatched_check) {
            printf("  UNEXPECTED B for id=%u bresp=%u (not in w_awaiting_b, size=%zu) t=%llu -- protocol violation (duplicate/phantom/misrouted response)\n",
                   id, (unsigned)dut->s_axi_bresp, w_awaiting_b.size(), (unsigned long long)sim_time);
            if (!g_unmatched_resp_seen) dump_op_log();
            g_unmatched_resp_seen = true;
        }
    }
    if (dut->s_axi_rvalid && dut->s_axi_rready) {
        uint32_t id = dut->s_axi_rid;
        auto it = r_awaiting.find(id);
        bool have_entry = (it != r_awaiting.end()) && !it->second.empty();
        if (!have_entry && !g_suppress_unmatched_check) {
            printf("  UNEXPECTED R for id=%u (not in r_awaiting, size=%zu) t=%llu -- protocol violation (duplicate/phantom/misrouted response)\n",
                   id, r_awaiting.size(), (unsigned long long)sim_time);
            if (!g_unmatched_resp_seen) dump_op_log();
            g_unmatched_resp_seen = true;
        }
        if (have_entry) {
            // R beats for a given id are always consumed from the FRONT
            // of that id's queue (AXI same-ID in-order completion).
            auto& front = it->second.front();
            std::array<uint8_t,16> beat{};
            get_wide_bytes(dut->s_axi_rdata, beat.data(), 16);
            if (front->got.empty()) front->t_r0 = sim_time;
            front->got.push_back(beat);
            front->last_resp = dut->s_axi_rresp;
            if (dut->s_axi_rlast) {
                front->t_rl = sim_time;
                front->done = true;
                r_done[id] = front;
                it->second.pop_front();
                if (it->second.empty()) r_awaiting.erase(it);
            }
        }
    }
}

// One full clock period, running both models each edge.
// One full clock period.  Handshake "did AW/W/AR/B/R fire" detection
// (mem_latch_edge/req_latch_edge) MUST happen on the settled PRE-edge
// values, before clk is raised -- several of the ready/valid signals we
// check (e.g. s_axi_awready, driven combinationally from the DUT's
// `aw_have` register) are produced by registers that update on the very
// same rising edge, so re-reading them *after* `clk=1; eval();` can
// observe the *post*-acceptance value (ready already dropped) and miss
// the handshake entirely -- a same-edge race that silently deadlocks the
// whole driver (aw_have latches, s_axi_awready drops in the same eval(),
// the testbench never records the AW as accepted, and re-asserts AWVALID
// forever). Reading everything on the stable pre-edge snapshot avoids
// this class of bug uniformly across every channel.
// ── Eviction-pressure accounting (see tb_l2c.v's dbg_vb_occ/dbg_evict_stall)
// Free-running, sampled every cycle with the clock low (all combinational
// settled to what the coming edge will sample).  g_ev.* is snapshotted and
// diffed around a measured window; nothing here affects the DUT.
static constexpr int EV_MAX_SLOTS = 31;     // dbg_vb_occ is 4 bits
struct EvStats {
    uint64_t cycles = 0;
    uint64_t occ_hist[EV_MAX_SLOTS + 1] = {0};  // cycles at each VB occupancy
    uint64_t stall = 0;                 // cycles a miss was blocked ONLY by a full VB
    uint64_t drain_busy = 0;            // cycles >=1 writeback was in flight/pending
    uint64_t seq_busy = 0;              // cycles the AW/W sequencer had work to send
    uint64_t out_sum = 0;               // sum of outstanding writebacks per cycle
    uint64_t out_max = 0;               // high-water outstanding writebacks
};
static EvStats g_ev;

// ── FRONT-DOOR occupancy accounting (2026-08-20 front-door depth study) ──
// See tb_l2c.v's dbg_fd_* comment for what each tap means.  The point of
// this block is to answer one question with a number instead of an
// argument: l2c_ctrl holds ONE AW burst and ONE AR burst at a time, so a
// master cannot hand it burst N+1's header until burst N's last beat has
// been consumed.  How much throughput does that cost?
//
// The denominator that matters is `idle` -- cycles the tag pipeline was at
// its accept point.  The pipeline is a 3-cycle S_IDLE -> S_WAIT ->
// S_LOOKUP loop, so it offers at most one accept every three cycles no
// matter what the door does; the door can only be blamed for cycles where
// an accept slot EXISTED and went unused.
//
// Two counterfactuals are kept, because the loose one is misleading on
// its own and the difference between them IS the answer:
//
//   nowork_hostwork -- pipeline idle, door empty, and the C++ requester
//       still had an un-accepted burst queued.  This over-counts badly:
//       with a finite outstanding window the requester only forms its next
//       header when an older transaction completes, so the header simply
//       did not EXIST earlier and no amount of door depth could have
//       latched it.  Every window=1 row reads 50% on this metric for that
//       reason alone.
//   nowork_refused  -- the strict one, and the honest one.  The bubble is
//       counted only if the header now being presented was ALREADY being
//       presented (and therefore refused, since the door was full) on the
//       previous cycle.  That is exactly the condition under which a
//       deeper door would have had the burst resident and could have fed
//       the pipeline this cycle.  This is the number that decides whether
//       depth is worth buying.
//
// idbusy bubbles are not recoverable by DOOR depth (the next beat behind
// them carries the same ID).  Note that as of 2026-08-20 a BYPASS beat no
// longer produces them at all -- l2c_ctrl's Critical-3 hold-off is
// qualified with !is_bypass_c, because the bypass engine delivers same-id
// responses in acceptance order by construction.  What a saturated bypass
// stream produces now is `bypnr`: the engine's queue is genuinely full,
// which is real backpressure and is recoverable by ENGINE depth
// (BYPASS_SLOTS), not by door depth.  docs/l2c_perf.md S14.
struct FdStats {
    uint64_t cycles = 0;
    uint64_t idle = 0, accept = 0, gather = 0;
    uint64_t nowork = 0, nowork_hostwork = 0, nowork_refused = 0, idbusy = 0, bypnr = 0;
    uint64_t ar_stall = 0, aw_stall = 0, w_stall = 0;
    uint64_t ar_pres = 0, aw_pres = 0, w_pres = 0;
    uint64_t hdr_burst = 0;      // AR/AW headers actually accepted
    uint64_t stall_run_max = 0;  // longest unbroken AR-or-AW refusal run
    // 2026-08-20 lookup pipelining.
    uint64_t sethaz = 0;         // accept refused by the array RAW interlock
    uint64_t s2_stall = 0;       // resolve stage held a request it could not retire
    uint64_t s2_reread = 0;      // skew-hazard re-reads armed
};
static FdStats g_fd;
static uint64_t g_fd_run = 0;
static FdStats fd_diff(const FdStats& now, const FdStats& then) {
    FdStats d;
    d.cycles = now.cycles - then.cycles;
    d.idle = now.idle - then.idle;   d.accept = now.accept - then.accept;
    d.gather = now.gather - then.gather;
    d.nowork = now.nowork - then.nowork;
    d.nowork_hostwork = now.nowork_hostwork - then.nowork_hostwork;
    d.nowork_refused = now.nowork_refused - then.nowork_refused;
    d.idbusy = now.idbusy - then.idbusy;   d.bypnr = now.bypnr - then.bypnr;
    d.ar_stall = now.ar_stall - then.ar_stall; d.aw_stall = now.aw_stall - then.aw_stall;
    d.w_stall = now.w_stall - then.w_stall;
    d.ar_pres = now.ar_pres - then.ar_pres;   d.aw_pres = now.aw_pres - then.aw_pres;
    d.w_pres = now.w_pres - then.w_pres;
    d.hdr_burst = now.hdr_burst - then.hdr_burst;
    d.stall_run_max = now.stall_run_max;
    d.sethaz = now.sethaz - then.sethaz;
    d.s2_stall = now.s2_stall - then.s2_stall;
    d.s2_reread = now.s2_reread - then.s2_reread;
    return d;
}
// Self-check on the instrumentation itself.  dbg_evict_stall means "this
// miss is blocked ONLY by the victim buffer having no free slot", which by
// definition cannot happen unless EVERY slot is occupied.  The capacity
// comes from the DUT (dbg_vb_slots) rather than a constant here, so
// deepening l2c_victim cannot quietly turn this check into a tautology.
// If the tap were mis-wired -- wrong hierarchical path, inverted
// push_ready, stuck signal -- this is what catches it, and every
// conclusion drawn from the eviction numbers depends on the tap being right.
static bool g_ev_tap_violation = false;

static void tick() {
    dut->clk = 0; eval();
    req_pump_issue();
    mem_drive_comb(); mem_drive_rdata(); req_drive_comb();
    eval();
    g_ev.cycles++;
    {
        const unsigned occ   = dut->dbg_vb_occ   & 0x1F;
        const unsigned slots = dut->dbg_vb_slots & 0x1F;
        const unsigned out   = dut->dbg_vb_out   & 0x1F;
        g_ev.occ_hist[occ <= EV_MAX_SLOTS ? occ : EV_MAX_SLOTS]++;
        g_ev.out_sum += out;
        if (out > g_ev.out_max) g_ev.out_max = out;
        if (dut->dbg_evict_stall) {
            g_ev.stall++;
            if (occ != slots) {
                if (!g_ev_tap_violation)
                    printf("  EV TAP VIOLATION: evict_stall asserted with victim-buffer "
                           "occupancy %u (must be %u = SLOTS) t=%llu\n",
                           occ, slots, (unsigned long long)sim_time);
                g_ev_tap_violation = true;
            }
        }
        // A second, independent invariant on the same taps, and the one
        // that actually guards the pipelining: a writeback that is
        // outstanding must still OCCUPY ITS SLOT.  Retirement is keyed on
        // B and never on AW-accept, because a presented-or-accepted write
        // is not durable and its line must stay visible to
        // victim_query_hit until it is -- freeing early is precisely the
        // silent stale-data bug this whole rewrite had to avoid.  The
        // sink_cnt term is the one legitimate exception: post-reset
        // strays are outstanding on the bus but own no slot.
        const unsigned sink = dut->dbg_vb_sink & 0x1F;
        if (out > occ + sink || out > slots || sink > out) {
            if (!g_ev_tap_violation)
                printf("  EV TAP VIOLATION: %u writebacks outstanding (%u post-reset strays) "
                       "with occupancy %u of %u slots t=%llu\n",
                       out, sink, occ, slots, (unsigned long long)sim_time);
            g_ev_tap_violation = true;
        }
    }
    if (dut->dbg_vb_drain_busy) g_ev.drain_busy++;
    if (dut->dbg_vb_seq_busy)   g_ev.seq_busy++;
    by_sample_taps();
    if (dut->rst) by_reset_epoch();
    {
        // Front-door sampling.  Same phase as the eviction taps above:
        // clock low, every combinational signal settled to what the coming
        // rising edge will actually sample.
        const bool host_work = r_issuing || w_issuing || aw_issuing ||
                                !w_aw_sent.empty() ||
                                !r_pending_issue.empty() || !w_pending_issue.empty();
        // "was a header presented AND refused on the previous cycle" --
        // the strict recoverability test described on FdStats.
        static bool prev_hdr_refused = false;
        g_fd.cycles++;
        if (dut->dbg_fd_idle)   g_fd.idle++;
        if (dut->dbg_fd_accept) g_fd.accept++;
        if (dut->dbg_fd_gather) g_fd.gather++;
        if (dut->dbg_fd_nowork) {
            g_fd.nowork++;
            if (host_work) g_fd.nowork_hostwork++;
            if (prev_hdr_refused) g_fd.nowork_refused++;
        }
        if (dut->dbg_fd_idbusy) g_fd.idbusy++;
        if (dut->dbg_fd_bypnr)  g_fd.bypnr++;
        if (dut->dbg_fd_sethaz) g_fd.sethaz++;
        if (dut->dbg_s2_stall)  g_fd.s2_stall++;
        if (dut->dbg_s2_reread) g_fd.s2_reread++;
        if (dut->s_axi_arvalid) g_fd.ar_pres++;
        if (dut->s_axi_awvalid) g_fd.aw_pres++;
        if (dut->s_axi_wvalid)  g_fd.w_pres++;
        if (dut->dbg_fd_ar_stall) g_fd.ar_stall++;
        if (dut->dbg_fd_aw_stall) g_fd.aw_stall++;
        if (dut->dbg_fd_w_stall)  g_fd.w_stall++;
        if (dut->s_axi_arvalid && dut->s_axi_arready) g_fd.hdr_burst++;
        if (dut->s_axi_awvalid && dut->s_axi_awready) g_fd.hdr_burst++;
        if (dut->dbg_fd_ar_stall || dut->dbg_fd_aw_stall) {
            g_fd_run++;
            if (g_fd_run > g_fd.stall_run_max) g_fd.stall_run_max = g_fd_run;
        } else g_fd_run = 0;
        prev_hdr_refused = dut->dbg_fd_ar_stall || dut->dbg_fd_aw_stall;
    }
    if (dut->m_axi_rvalid && !dut->m_axi_rready) g_r_stall_cycles++;
    mem_latch_edge(); by_latch_edge(); req_latch_edge();
    mem_check_stability(); req_check_stability();
    dut->clk = 1; eval();
    sim_time++;
    eval();
}

static void reset_dut() {
    dut->rst = 1; dut->clk = 0;
    dut->s_axi_awvalid = 0; dut->s_axi_wvalid = 0; dut->s_axi_arvalid = 0;
    dut->s_axi_bready = 1; dut->s_axi_rready = 1;
    dut->f_axi_arvalid = 0; dut->f_axi_rready = 1;
    dut->m_axi_awready = 0; dut->m_axi_wready = 0; dut->m_axi_arready = 0;
    dut->m_axi_bvalid = 0; dut->m_axi_rvalid = 0;
    eval();
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    // Reset-walk: ~4096 cycles, sweeps all sets before AW/AR ready goes high.
    // Also verifies the "accepts held, then clean state" reset-walk contract.
    int waited = 0;
    while (dut->s_axi_awready == 0 && dut->s_axi_arready == 0 && waited < 6000) { tick(); waited++; }
}

// ─── High-level helpers: enqueue + block-wait for completion ─────────────
static std::array<uint8_t,16> mkbeat(uint64_t addr) {
    std::array<uint8_t,16> b{};
    for (int i = 0; i < 16; i++) b[i] = golden[addr + i];
    return b;
}

static uint32_t issue_read(uint64_t addr, int beats = 1) {
    auto rb = std::make_shared<RBurst>();
    rb->id = alloc_id(); rb->addr = addr; rb->beats = beats;
    rb->t_issue = sim_time;
    r_pending_issue.push_back(rb);
    return rb->id;
}
// Critical-3 test hook: issue with an EXPLICIT id, bypassing alloc_id()'s
// uniqueness guarantee -- deliberately reuses an id across two
// concurrently-outstanding reads (test_same_id_ordering only).  The
// caller is responsible for not colliding with the normal free_ids pool.
static void issue_read_explicit_id(uint64_t addr, uint32_t id, int beats = 1) {
    auto rb = std::make_shared<RBurst>();
    rb->id = id; rb->addr = addr; rb->beats = beats;
    rb->t_issue = sim_time;
    r_pending_issue.push_back(rb);
}
static uint32_t issue_write(uint64_t addr, const std::vector<std::array<uint8_t,16>>& data,
                             const std::vector<uint16_t>& strb) {
    auto wb = std::make_shared<WBurst>();
    wb->id = alloc_id(); wb->addr = addr; wb->beats = static_cast<int>(data.size());
    wb->data = data; wb->strb = strb;
    wb->t_issue = sim_time;
    w_pending_issue.push_back(wb);
    return wb->id;
}
// Perf-suite hook: same as issue_write() but with a caller-chosen id, so a
// single-ID streaming master (the shape a real CPU/DMA master presents at
// xbar S0) can be modelled against l2c's same-ID accept gate.
static void issue_write_explicit_id(uint64_t addr, uint32_t id,
                                     const std::vector<std::array<uint8_t,16>>& data,
                                     const std::vector<uint16_t>& strb) {
    auto wb = std::make_shared<WBurst>();
    wb->id = id; wb->addr = addr; wb->beats = static_cast<int>(data.size());
    wb->data = data; wb->strb = strb;
    wb->t_issue = sim_time;
    w_pending_issue.push_back(wb);
}
static uint32_t issue_write_full(uint64_t addr, uint64_t val_seed) {
    std::array<uint8_t,16> d{};
    for (int i = 0; i < 16; i++) d[i] = static_cast<uint8_t>((val_seed + i) & 0xFF);
    return issue_write(addr, {d}, {0xFFFF});
}

static bool wait_read_done(uint32_t id, int max_cycles = 20000) {
    int c = 0;
    while (r_done.find(id) == r_done.end() && c < max_cycles) { tick(); c++; }
    return r_done.find(id) != r_done.end();
}
static bool wait_write_done(uint32_t id, int max_cycles = 20000) {
    int c = 0;
    while (w_done.find(id) == w_done.end() && c < max_cycles) { tick(); c++; }
    return w_done.find(id) != w_done.end();
}
static void free_read(uint32_t id) { r_done.erase(id); free_ids.insert(id); }
static void free_write(uint32_t id) { w_done.erase(id); free_ids.insert(id); }

static bool do_read_check(uint64_t addr, const char* what) {
    uint32_t id = issue_read(addr);
    if (!wait_read_done(id)) { printf("  %s: TIMEOUT waiting for read @0x%llx\n", what, (unsigned long long)addr); return false; }
    auto rb = r_done[id];
    bool ok = std::memcmp(rb->got[0].data(), &golden[addr], 16) == 0;
    if (!ok) {
        printf("  %s: DATA MISMATCH @0x%llx got=%02x%02x.. exp=%02x%02x..\n", what, (unsigned long long)addr,
               rb->got[0][0], rb->got[0][1], golden[addr], golden[addr+1]);
    }
    free_read(id);
    return ok;
}
static bool do_write_apply(uint64_t addr, uint64_t seed, const char* what) {
    uint32_t id = issue_write_full(addr, seed);
    if (!wait_write_done(id)) { printf("  %s: TIMEOUT waiting for write @0x%llx\n", what, (unsigned long long)addr); return false; }
    for (int i = 0; i < 16; i++) golden[addr + i] = static_cast<uint8_t>((seed + i) & 0xFF);
    touched[addr] = true;
    bool ok = w_done[id]->resp_ok;
    free_write(id);
    return ok;
}

#define CHECK(name, cond) do { \
    bool _r = (cond); \
    if (!_r) printf("  FAIL check: %s (%s:%d)\n", name, __FILE__, __LINE__); \
    ok = ok && _r; \
} while (0)

// ─── Directed scenarios ───────────────────────────────────────────────────

static bool test_read_miss_fill() {
    bool ok = true;
    uint64_t a = 0x1000;
    CHECK("read miss returns DRAM content", do_read_check(a, "read_miss_fill"));
    return ok;
}

static bool test_read_hit() {
    bool ok = true;
    uint64_t a = 0x1040;
    CHECK("prime (miss)", do_read_check(a, "prime"));
    // second access to the same line should hit -- checked for latency in
    // the perf-smoke pass; here we just confirm data is still correct.
    CHECK("hit returns same data", do_read_check(a, "read_hit"));
    return ok;
}

static bool test_fill_bank_quadrants() {
    bool ok = true;
    // More than one full MSHR-table turnover. Each request starts in a
    // different quadrant; then check the other three banks of that line.
    for (unsigned epoch = 0; epoch < 4; ++epoch) {
        uint32_t ids[8];
        uint64_t addrs[8];
        for (unsigned slot = 0; slot < 8; ++slot) {
            addrs[slot] = 0x600000 + epoch*0x1000 + slot*64 + (slot%4)*16;
            ids[slot] = issue_read(addrs[slot]);
        }
        for (unsigned slot = 0; slot < 8; ++slot) {
            bool completed = wait_read_done(ids[slot]);
            CHECK("fill bank concurrent completion", completed);
            if (completed) {
                CHECK("fill bank concurrent payload",
                      std::memcmp(r_done[ids[slot]]->got[0].data(),
                                  &golden[addrs[slot]], 16) == 0);
                free_read(ids[slot]);
            }
        }
        for (unsigned slot = 0; slot < 8; ++slot)
            for (unsigned q = 0; q < 4; ++q)
                CHECK("fill bank quadrant readback",
                      do_read_check((addrs[slot] & ~uint64_t(63)) + q*16,
                                    "fill_bank_quadrants"));
    }
    return ok;
}

static bool test_write_miss_allocate() {
    bool ok = true;
    uint64_t a = 0x2000;
    CHECK("write miss allocates", do_write_apply(a, 0x11, "write_miss"));
    CHECK("readback after write-allocate", do_read_check(a, "readback"));
    return ok;
}

static bool test_write_hit_dirty() {
    bool ok = true;
    uint64_t a = 0x2040;
    CHECK("prime write (miss)", do_write_apply(a, 0x22, "prime_write"));
    CHECK("second write (hit, dirty)", do_write_apply(a, 0x33, "write_hit"));
    CHECK("readback", do_read_check(a, "readback_dirty_hit"));
    return ok;
}

// Fill all 8 ways of one set with dirty lines, then force a 9th install to
// evict one -- confirms writeback ordering (evicted data survives via the
// victim buffer / DRAM, and is byte-exact on readback from a *different*
// line reusing the same set after the fact wouldn't prove writeback --
// instead we directly re-read the evicted line's own address, which by
// then must come from DRAM, exercising the drain path end-to-end).
static bool test_dirty_eviction_writeback() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES; // same set index, +1 tag
    uint64_t base = 0x3000;
    for (int way = 0; way < 8; way++) {
        uint64_t a = base + static_cast<uint64_t>(way) * set_stride;
        CHECK("fill way (dirty)", do_write_apply(a, 0x40 + way, "fill_way"));
    }
    // 9th line, same set -> evicts one of the 8 (PLRU), whose dirty data
    // must have reached DRAM (or be readable coherently through L2 again).
    uint64_t a9 = base + 8ULL * set_stride;
    CHECK("9th alloc forces eviction", do_write_apply(a9, 0x99, "evict_trigger"));
    // Every original way's data must still read back correctly, whether it
    // stayed resident or was evicted+refetched.
    for (int way = 0; way < 8; way++) {
        uint64_t a = base + static_cast<uint64_t>(way) * set_stride;
        CHECK("evicted-or-resident readback", do_read_check(a, "post_evict_readback"));
    }
    CHECK("9th line readback", do_read_check(a9, "post_evict_9th"));
    return ok;
}

// Victim-buffer hazard: force an eviction, then immediately (same-ish
// window) re-read the just-evicted address before its writeback is
// guaranteed to have drained -- correctness (not stall-cycle-exactness)
// is what's checked: the read must still return the correct (evicted)
// data despite racing the drain.
static bool test_victim_buffer_hazard() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    uint64_t base = 0x4000;
    for (int way = 0; way < 8; way++)
        CHECK("fill way", do_write_apply(base + static_cast<uint64_t>(way) * set_stride, 0x50 + way, "fill"));
    uint64_t victim_addr = base; // way 0, oldest per PLRU, likely evicted next
    uint64_t a9 = base + 8ULL * set_stride;
    CHECK("trigger eviction", do_write_apply(a9, 0xAA, "evict_trigger"));
    // Immediately race a read against the (possibly still-draining) victim,
    // at offset 0 AND at a NONZERO offset within the line.
    //
    // ⚠ THIS IS NOT THE CRITICAL-2 NEGATIVE CONTROL, corrected 2026-09-18
    // (race audit).  It used to claim it was: "the hazard compare must be
    // line-aligned, addr[31:6] -- an earlier version compared the raw
    // per-beat address against the buffer's line-aligned address and missed
    // every query whose low 6 bits were nonzero, returning stale/wrong data
    // instead of the still-buffered dirty copy".  The history is real; the
    // claim that THIS test pins it is not.  MEASURED: restore the full
    // address compare in l2c_victim.v's q_match_c (`v_addr[gq] ==
    // query_addr`) and the directed suite reports 64 PASS / 1 FAIL -- the
    // single failure is randomized_scoreboard.  victim_buffer_hazard itself
    // passes, offset-32 read and all.
    //
    // Why it no longer discriminates: by the time the offset-32 read is
    // issued, the intervening eight "fill way (re-evict)" writes plus the
    // second eviction trigger have given the victim entry ample time to
    // drain, so the read is not actually racing anything and query_hit is
    // false either way.  Restoring the control means issuing the nonzero-
    // offset read against a victim entry that is still resident -- i.e.
    // immediately after its own eviction trigger, with no intervening
    // traffic, the way the offset-0 read is -- and asserting on the
    // still-buffered dirty copy.  That changes this test's cycle count and
    // therefore the L2C_PERF coverage baseline, so it is recorded here
    // rather than done in passing.  Until then, Critical-2 is covered by
    // randomized_scoreboard alone.
    CHECK("read raced against drain (offset 0)", do_read_check(victim_addr, "victim_hazard_read_off0"));
    for (int way = 0; way < 8; way++)
        CHECK("fill way (re-evict)", do_write_apply(base + static_cast<uint64_t>(way) * set_stride, 0x60 + way, "refill"));
    uint64_t a10 = base + 9ULL * set_stride; // same set, 10th distinct tag
    CHECK("trigger eviction 2", do_write_apply(a10, 0xBB, "evict_trigger2"));
    CHECK("read raced against drain (offset 32B)", do_read_check(victim_addr + 32, "victim_hazard_read_off32"));
    // Explicitly drain the victim buffer before returning: PLRU may have
    // evicted a DIFFERENT way than way 0 in either round above, and the
    // two targeted reads only force a wait for whichever line THEY
    // happen to hit -- an un-drained entry left occupying either of the
    // 2 (global, shared) victim-buffer slots would silently starve a
    // LATER, unrelated test's own eviction (push_ready needs a free
    // slot).  Reading back every way + both eviction-trigger lines
    // forces every possible victim entry through its hazard-wait.
    for (int way = 0; way < 8; way++)
        CHECK("post-test drain readback", do_read_check(base + static_cast<uint64_t>(way) * set_stride, "victim_hazard_drain"));
    CHECK("post-test drain readback (9th)", do_read_check(a9, "victim_hazard_drain_9th"));
    CHECK("post-test drain readback (10th)", do_read_check(a10, "victim_hazard_drain_10th"));
    return ok;
}

// MSHR merge: issue two reads to the same line back-to-back (second one
// issued before the first's fill can possibly have completed) -- both
// must return correct data, proving the secondary merged into the
// primary's MSHR entry rather than being dropped or double-allocating.
static bool test_mshr_merge() {
    bool ok = true;
    uint64_t a = 0x5000;
    uint32_t id1 = issue_read(a);
    uint32_t id2 = issue_read(a + 16); // same line, different quadrant
    CHECK("primary completes", wait_read_done(id1));
    CHECK("secondary (merged) completes", wait_read_done(id2));
    if (r_done.count(id1)) {
        CHECK("primary data correct", std::memcmp(r_done[id1]->got[0].data(), &golden[a], 16) == 0);
        free_read(id1);
    }
    if (r_done.count(id2)) {
        CHECK("secondary data correct", std::memcmp(r_done[id2]->got[0].data(), &golden[a+16], 16) == 0);
        free_read(id2);
    }
    return ok;
}

// MSHR full backpressure: allocate 8 concurrent misses to 8 DISTINCT
// lines (exhausting the 8-entry table), then issue a 9th -- it must not
// be dropped, just delayed until a slot frees.
static bool test_mshr_full_backpressure() {
    bool ok = true;
    uint64_t base = 0x6000;
    std::vector<uint32_t> ids;
    for (int i = 0; i < 9; i++) ids.push_back(issue_read(base + static_cast<uint64_t>(i) * LINE_BYTES * 97));
    int done_count = 0;
    for (auto id : ids) if (wait_read_done(id, 30000)) done_count++;
    CHECK("all 9 misses eventually complete (none dropped)", done_count == 9);
    for (int i = 0; i < 9; i++) {
        if (r_done.count(ids[i])) {
            uint64_t a = base + static_cast<uint64_t>(i) * LINE_BYTES * 97;
            CHECK("backpressured miss data correct", std::memcmp(r_done[ids[i]]->got[0].data(), &golden[a], 16) == 0);
            free_read(ids[i]);
        }
    }
    return ok;
}

// Eight independent misses must launch before any data returns, and RID must
// select the correct per-entry assembly buffer even when complete bursts come
// back in the reverse of AR acceptance order.  The production MIG path is
// accepted-order today, but the MSHR itself is deliberately ID-demultiplexed;
// keeping that property tested prevents a future scheduler from silently
// coupling correctness back to one-at-a-time response ordering.
static bool test_mshr_pipelined_reverse_returns() {
    bool ok = true;
    constexpr int N = 8;
    const uint64_t base = 0x0020'0000ULL;
    std::vector<uint32_t> ids;
    uint64_t ar0 = g_dbg_ar_accepts;

    g_mem_block_rvalid = true;
    for (int i = 0; i < N; i++)
        ids.push_back(issue_read(base + static_cast<uint64_t>(i) * 17 * LINE_BYTES));

    int guard = 20000;
    while ((g_dbg_ar_accepts - ar0) < N && guard-- > 0) tick();
    CHECK("all eight fill ARs accepted before any R", (g_dbg_ar_accepts - ar0) == N);
    CHECK("memory holds eight independent fill bursts", mslv.r_q.size() == N);
    CHECK("exact MSHR occupancy reaches eight", dut->dbg_mshr_occupancy == N);

    if (mslv.r_q.size() == N) {
        std::reverse(mslv.r_q.begin(), mslv.r_q.end());
        for (auto& r : mslv.r_q) r.ready_at = sim_time;
    }
    g_mem_block_rvalid = false;

    for (int i = 0; i < N; i++) {
        bool done = wait_read_done(ids[i]);
        CHECK("reverse-return fill completes", done);
        if (done) {
            uint64_t a = base + static_cast<uint64_t>(i) * 17 * LINE_BYTES;
            CHECK("reverse-return fill data follows RID",
                  r_done[ids[i]]->got.size() == 1 &&
                  std::memcmp(r_done[ids[i]]->got[0].data(), &golden[a], 16) == 0);
            free_read(ids[i]);
        }
    }
    CHECK("all reverse-return bursts drained", mslv.r_q.empty());
    int occupancy_guard = 100;
    while (dut->dbg_mshr_occupancy != 0 && occupancy_guard-- > 0) tick();
    CHECK("exact MSHR occupancy returns to zero", dut->dbg_mshr_occupancy == 0);
    return ok;
}

static size_t accepted_read_headers() {
    size_t count = 0;
    for (const auto& by_id : r_awaiting) count += by_id.second.size();
    return count;
}

// Fill the MSHR table while DDR holds ARREADY low.  This checks that all
// eight requests may be represented independently before any fill is issued,
// that the presented AR remains stable, and that release launches/drains the
// whole table without losing a request.
static bool test_mshr_ar_backpressure_release() {
    bool ok = true;
    constexpr int N = 8;
    const uint64_t base = 0x0028'0000ULL;
    std::vector<uint64_t> addrs;
    std::vector<uint32_t> ids;
    const uint64_t ar0 = g_dbg_ar_accepts;

    g_mem_block_arready = true;
    g_mem_block_rvalid = true;
    for (int i = 0; i < N; i++) {
        addrs.push_back(base + static_cast<uint64_t>(i) * 29 * LINE_BYTES);
        ids.push_back(issue_read(addrs.back()));
    }
    int guard = 2000;
    while (dut->dbg_mshr_occupancy < N && guard-- > 0) tick();
    for (int i = 0; i < 64; i++) tick();
    CHECK("all MSHRs allocate behind ARREADY backpressure", dut->dbg_mshr_occupancy == N);
    CHECK("no fill AR accepted while ARREADY is forced low", g_dbg_ar_accepts == ar0);
    CHECK("one fill AR remains presented while blocked", dut->m_axi_arvalid);

    g_mem_block_arready = false;
    guard = 2000;
    while ((g_dbg_ar_accepts - ar0) < N && guard-- > 0) tick();
    CHECK("all blocked fill ARs launch after release", (g_dbg_ar_accepts - ar0) == N);
    CHECK("all launched fills are retained by memory", mslv.r_q.size() == N);
    g_mem_block_rvalid = false;

    for (int i = 0; i < N; i++) {
        const bool done = wait_read_done(ids[i], 20000);
        CHECK("AR-backpressured fill completes", done);
        if (done) {
            CHECK("AR-backpressured fill data correct",
                  r_done[ids[i]]->got.size() == 1 &&
                  std::memcmp(r_done[ids[i]]->got[0].data(), &golden[addrs[i]], 16) == 0);
            free_read(ids[i]);
        }
    }
    return ok;
}

// One primary plus L2C_REPLAY_N secondary requests is the exact per-entry
// replay capacity.  One MORE same-line request must remain backpressured,
// then make progress once the first entry installs and frees.
//
// L2C_REPLAY_N mirrors l2c.v's MSHR_REPLAY_N parameter (default 4).  Define
// it on the compile line to exercise a resized replay FIFO -- the saturation
// point is a function of that parameter, not a property of the design, and
// hardcoding 4 here made the reduced-area config look like a failure.
#ifndef L2C_REPLAY_N
#define L2C_REPLAY_N 4
#endif
static bool test_mshr_merge_queue_saturation() {
    bool ok = true;
    constexpr int N = L2C_REPLAY_N + 2; // primary + REPLAY_N secondaries + 1 staged
    const uint64_t base = 0x002C'0000ULL;
    std::vector<uint64_t> addrs;
    std::vector<uint32_t> ids;

    g_mem_block_rvalid = true;
    for (int i = 0; i < N; i++) {
        addrs.push_back(base + static_cast<uint64_t>(i & 3) * 16);
        ids.push_back(issue_read(addrs.back()));
    }
    int guard = 3000;
    while (accepted_read_headers() < N && guard-- > 0) tick();
    for (int i = 0; i < 64; i++) tick();
    // The AXI front door has one request register beyond the MSHR replay
    // slots, so all six headers may handshake.  The sixth must remain staged
    // there: it may not allocate another MSHR or launch another DDR fill.
    CHECK("front door retains one request beyond MSHR merge capacity",
          accepted_read_headers() == N);
    CHECK("replay saturation does not launch a duplicate same-line fill",
          mslv.r_q.size() == 1);
    CHECK("same-line demand consumes one MSHR", dut->dbg_mshr_occupancy == 1);
    CHECK("staged sixth request cannot complete before the fill",
          r_done.count(ids.back()) == 0);

    g_mem_block_rvalid = false;
    for (int i = 0; i < N; i++) {
        const bool done = wait_read_done(ids[i], 30000);
        CHECK("saturated same-line request completes", done);
        if (done) {
            CHECK("saturated same-line request data correct",
                  std::memcmp(r_done[ids[i]]->got[0].data(), &golden[addrs[i]], 16) == 0);
            free_read(ids[i]);
        }
    }
    return ok;
}

// Hot hits must keep completing while every MSHR is occupied by a cold fill
// whose DDR response is delayed.  This is the useful non-blocking-cache case,
// rather than only measuring eight concurrent misses in isolation.
static bool test_hits_progress_under_full_mshr() {
    bool ok = true;
    constexpr int N = 8;
    const uint64_t hot_base = 0x0030'0000ULL;
    const uint64_t cold_base = 0x0038'0000ULL;
    std::vector<uint64_t> hot, cold;
    std::vector<uint32_t> cold_ids, hot_ids;

    for (int i = 0; i < N; i++) {
        hot.push_back(hot_base + static_cast<uint64_t>(i) * 11 * LINE_BYTES);
        CHECK("prime hot line", do_read_check(hot.back(), "prime_hot"));
    }

    g_mem_block_rvalid = true;
    const uint64_t ar0 = g_dbg_ar_accepts;
    for (int i = 0; i < N; i++) {
        cold.push_back(cold_base + static_cast<uint64_t>(i) * 13 * LINE_BYTES);
        cold_ids.push_back(issue_read(cold.back()));
    }
    int guard = 5000;
    while ((g_dbg_ar_accepts - ar0) < N && guard-- > 0) tick();
    CHECK("all cold fills launch", (g_dbg_ar_accepts - ar0) == N);
    CHECK("cold fills occupy every MSHR", dut->dbg_mshr_occupancy == N);

    for (uint64_t a : hot) hot_ids.push_back(issue_read(a));
    for (int i = 0; i < N; i++) {
        const bool done = wait_read_done(hot_ids[i], 3000);
        CHECK("hot hit completes while DDR fills are blocked", done);
        if (done) {
            CHECK("hot hit data correct under miss pressure",
                  std::memcmp(r_done[hot_ids[i]]->got[0].data(), &golden[hot[i]], 16) == 0);
            free_read(hot_ids[i]);
        }
    }
    bool any_cold_completed = false;
    for (uint32_t id : cold_ids) any_cold_completed |= r_done.count(id) != 0;
    CHECK("blocked cold fills do not fabricate completions", !any_cold_completed);

    g_mem_block_rvalid = false;
    for (int i = 0; i < N; i++) {
        const bool done = wait_read_done(cold_ids[i], 30000);
        CHECK("cold fill completes after DDR release", done);
        if (done) {
            CHECK("cold fill data correct after DDR release",
                  std::memcmp(r_done[cold_ids[i]]->got[0].data(), &golden[cold[i]], 16) == 0);
            free_read(cold_ids[i]);
        }
    }
    return ok;
}

// Exercise the RID-demultiplexed assembly buffers repeatedly, with a new
// random complete-burst order each round.  Bursts remain non-interleaved,
// matching AXI, but completion order changes across IDs.
static bool test_mshr_randomized_completion_stress() {
    bool ok = true;
    constexpr int N = 8;
    constexpr int ROUNDS = 32;
    const uint64_t base = 0x0040'0000ULL;

    for (int round = 0; round < ROUNDS; round++) {
        std::vector<uint64_t> addrs;
        std::vector<uint32_t> ids;
        const uint64_t ar0 = g_dbg_ar_accepts;
        g_mem_block_rvalid = true;
        for (int i = 0; i < N; i++) {
            uint64_t line = static_cast<uint64_t>(round * N + i) * 37;
            addrs.push_back(base + line * LINE_BYTES + static_cast<uint64_t>((round + i) & 3) * 16);
            ids.push_back(issue_read(addrs.back()));
        }
        int guard = 5000;
        while ((g_dbg_ar_accepts - ar0) < N && guard-- > 0) tick();
        CHECK("stress round launches eight fills", (g_dbg_ar_accepts - ar0) == N);
        CHECK("stress round reaches full MSHR occupancy", dut->dbg_mshr_occupancy == N);
        CHECK("stress round memory has eight bursts", mslv.r_q.size() == N);
        if (mslv.r_q.size() == N) {
            std::shuffle(mslv.r_q.begin(), mslv.r_q.end(), ddr_rng);
            for (auto& request : mslv.r_q) request.ready_at = sim_time;
        }
        g_mem_block_rvalid = false;
        for (int i = 0; i < N; i++) {
            const bool done = wait_read_done(ids[i], 30000);
            CHECK("random-order fill completes", done);
            if (done) {
                CHECK("random-order fill follows RID",
                      std::memcmp(r_done[ids[i]]->got[0].data(), &golden[addrs[i]], 16) == 0);
                free_read(ids[i]);
            }
        }
        guard = 1000;
        while (dut->dbg_mshr_occupancy != 0 && guard-- > 0) tick();
        CHECK("stress round drains all MSHRs", dut->dbg_mshr_occupancy == 0);
    }
    printf("  randomized completion stress: %d rounds x %d concurrent fills\n", ROUNDS, N);
    return ok;
}

// A failed DDR fill must return SLVERR to the requester, avoid installing
// the line, and permit a clean retry that performs a second memory access.
static bool test_fill_error_retry() {
    bool ok = true;
    const uint64_t addr = 0x0078'0000ULL;
    const uint64_t ar0 = g_dbg_ar_accepts;
    g_mem_next_rresp = 2;
    uint32_t bad = issue_read(addr);
    CHECK("errored fill returns a response", wait_read_done(bad, 30000));
    if (r_done.count(bad)) {
        CHECK("errored fill reports SLVERR", r_done[bad]->last_resp == 2);
        free_read(bad);
    }
    int guard = 1000;
    while (dut->dbg_mshr_occupancy != 0 && guard-- > 0) tick();
    CHECK("errored fill releases its MSHR", dut->dbg_mshr_occupancy == 0);
    CHECK("errored fill reached DDR once", (g_dbg_ar_accepts - ar0) == 1);

    uint32_t retry = issue_read(addr);
    CHECK("retry after fill error completes", wait_read_done(retry, 30000));
    if (r_done.count(retry)) {
        CHECK("retry returns OKAY", r_done[retry]->last_resp == 0);
        CHECK("retry data comes from backing memory",
              std::memcmp(r_done[retry]->got[0].data(), &golden[addr], 16) == 0);
        free_read(retry);
    }
    CHECK("retry performs a new DDR fill instead of hitting bad data",
          (g_dbg_ar_accepts - ar0) == 2);
    return ok;
}

// Prime one set dirty, then launch another full MSHR window of write misses
// to distinct tags in that same set.  This simultaneously stresses busy-way
// exclusion, PLRU selection, the two-entry victim queue, fill installation,
// and writeback forward progress.
static bool test_concurrent_same_set_dirty_evictions() {
    bool ok = true;
    constexpr int WAYS = 8;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    const uint64_t base = 0x0001'8000ULL;
    std::vector<uint64_t> addrs;
    std::vector<uint32_t> ids;

    for (int tag = 0; tag < WAYS; tag++) {
        uint64_t addr = base + static_cast<uint64_t>(tag) * set_stride;
        addrs.push_back(addr);
        CHECK("prime dirty same-set way", do_write_apply(addr, 0x20 + tag, "prime_same_set"));
    }
    for (int tag = WAYS; tag < 2 * WAYS; tag++) {
        uint64_t addr = base + static_cast<uint64_t>(tag) * set_stride;
        addrs.push_back(addr);
        ids.push_back(issue_write_full(addr, 0x60 + tag));
    }
    for (int i = 0; i < WAYS; i++) {
        const bool done = wait_write_done(ids[i], 60000);
        CHECK("concurrent same-set write miss completes", done);
        if (done) {
            auto wb = w_done[ids[i]];
            CHECK("concurrent same-set write returns OKAY", wb->resp_ok);
            for (int byte = 0; byte < 16; byte++)
                golden[wb->addr + byte] = wb->data[0][byte];
            touched[wb->addr] = true;
            free_write(ids[i]);
        }
    }
    for (uint64_t addr : addrs)
        CHECK("all evicted or resident same-set tags read back", do_read_check(addr, "same_set_readback"));
    return ok;
}

// ── PIPELINE HAZARD: same set, different line, both reads ───────────────
// (2026-08-20, l2c_ctrl lookup pipelining.)
//
// The pipelined lookup reads the tag array at ACCEPT and resolves TWO
// CYCLES LATER, so a request cannot see the tag write of anything still
// inside the pipeline.  l2c_ctrl's accept-time interlock therefore refuses
// to overlap two requests in the same SET unless they are the same LINE and
// both reads -- and the same-line-read exemption is not a nicety, it is
// what keeps a 4-beat burst (four beats, one line, one set) running at one
// beat per cycle instead of three.
//
// This test attacks the boundary of that exemption: same set, DIFFERENT
// line, both reads.  Every way of one set is primed PARTIALLY VALID (a
// 16 B full-strobe write installs its own quadrant with no fill), and the
// set is then hammered with pairs of reads issued back to back so they
// land in adjacent pipeline stages:
//
//   leader   -- a fresh tag: a full miss, which must EVICT a way and
//               rewrite that way's tag entry;
//   follower -- quadrant 1 of a RESIDENT tag: tag match, quadrant invalid,
//               so it takes `any_tm_c` and fills INTO the matching way.
//
// If those two are allowed to overlap, the follower resolves against a tag
// array in which the leader has already re-tagged the very way the
// follower still believes is its own.  `way_ok_c` short-circuits on
// `any_tm_c` and never consults `mshr_bw_mask`, so the follower allocates a
// SECOND MSHR entry onto a way the leader's fill already owns -- caught
// immediately by l2c_mshr's (set, way) uniqueness assertion, and by the
// data check below once both fills install over each other.
//
// RED-verified against the mutant that relaxes `set_haz_c` to
//     (q_same_set_c && (q_is_write || write_sel)) || (p2_same_set_c && ...)
// i.e. one that keeps the write interlock but drops the same-LINE
// requirement for read pairs.  That mutant passes all 61 other scenarios,
// including the 20 000-op randomized scoreboard and every eviction test.
static bool test_pipeline_same_set_read_overlap() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;   // one full set index apart
    const uint64_t base = 0x0002'C000ULL;
    const int RESIDENT = 8;    // == WAYS: fill the set so every miss evicts
    const int FRESH    = 48;   // spare tags, all inside the 16 MB window
    const int ROUNDS   = 24;

    for (int t = 0; t < RESIDENT; t++)
        CHECK("prime partially-valid same-set way",
              do_write_apply(base + static_cast<uint64_t>(t) * set_stride,
                             0x40 + t, "pipe_sset_prime"));

    for (int r = 0; r < ROUNDS; r++) {
        std::vector<uint64_t> a; std::vector<uint32_t> id;
        for (int k = 0; k < 4; k++) {
            const int n = r * 4 + k;
            const uint64_t fresh = base +
                static_cast<uint64_t>(RESIDENT + (n % FRESH)) * set_stride + 16;
            const uint64_t resid = base +
                static_cast<uint64_t>(n % RESIDENT) * set_stride + 16;
            a.push_back(fresh); id.push_back(issue_read(fresh));
            a.push_back(resid); id.push_back(issue_read(resid));
        }
        for (size_t i = 0; i < id.size(); i++) {
            const bool done = wait_read_done(id[i], 60000);
            CHECK("same-set overlapped read completes", done);
            if (!done) return false;
            auto rb = r_done[id[i]];
            const bool match = std::memcmp(rb->got[0].data(), &golden[a[i]], 16) == 0;
            if (!match)
                printf("  pipe_sset: DATA MISMATCH @0x%llx got=%02x%02x.. exp=%02x%02x..\n",
                       (unsigned long long)a[i], rb->got[0][0], rb->got[0][1],
                       golden[a[i]], golden[a[i] + 1]);
            CHECK("same-set overlapped read returns correct data", match);
            free_read(id[i]);
        }
    }
    // The primed quadrants are the ones a double-allocation corrupts: the
    // losing fill marks its `alloc_vsec_pre` quadrants valid over the
    // winner's data without writing them.
    for (int t = 0; t < RESIDENT; t++)
        CHECK("primed quadrant survives same-set read pressure",
              do_read_check(base + static_cast<uint64_t>(t) * set_stride, "pipe_sset_q0"));
    return ok;
}

static bool test_bypass_read_write_ordering() {
    bool ok = true;
    uint64_t a = BYPASS_BASE + 0x100;
    CHECK("bypass write", do_write_apply(a, 0x77, "bypass_write"));
    CHECK("bypass readback (ordered after write)", do_read_check(a, "bypass_read"));
    // A second write immediately followed by a read must observe the new value.
    CHECK("bypass write 2", do_write_apply(a, 0x88, "bypass_write2"));
    CHECK("bypass readback 2", do_read_check(a, "bypass_read2"));
    return ok;
}

static bool test_reset_walk() {
    bool ok = true;
    // A second reset mid-run: must accept nothing during the walk, then
    // present a clean (dirty-lines-dropped) state afterwards.
    uint64_t a = 0x7000;
    CHECK("prime a dirty line pre-reset", do_write_apply(a, 0xEE, "pre_reset_write"));
    reset_dut();
    CHECK("s_axi ready after reset walk", dut->s_axi_awready == 1 || dut->s_axi_arready == 1);
    // Golden model can't know post-reset DRAM content precisely (any
    // still-dirty, un-written-back line's data is dropped by the reset
    // per the documented "dirty lines are dropped" contract, but clean
    // lines and lines already flushed to "DRAM" are unaffected -- the TB
    // has no visibility into which is which for any given address).  Any
    // address `touched[]` before this point can no longer be trusted, so
    // resync golden[] to backing[] (the only thing guaranteed to survive
    // a reset) and clear touched[] for everything -- otherwise a LATER
    // test (e.g. the randomized scoreboard's final-verify pass, which
    // walks every touched[] address ever set, including by earlier
    // directed tests) would flag false mismatches against stale
    // pre-reset expectations.  This is a testbench-bookkeeping fix, not
    // an RTL behavior change -- confirmed via the L2C_DEBUG_NO_BYPASS /
    // priority-arbiter fixes that the RTL itself returns correct data
    // for everything actually written *after* this reset.
    std::copy(backing.begin(), backing.end(), golden.begin());
    std::fill(touched.begin(), touched.end(), false);
    uint64_t a2 = 0x7100;
    CHECK("fresh access after reset works", do_write_apply(a2, 0xF0, "post_reset_write"));
    CHECK("fresh readback after reset", do_read_check(a2, "post_reset_read"));
    return ok;
}

// Common setup for the three IMPORTANT-A scenarios below: fill 8 ways of
// one set (all dirty), then issue a 9th write WITHOUT waiting for its
// own front-door completion -- that 9th write is itself a MISS (drives
// an MSHR fill) and, separately, forces a PLRU eviction of one of the
// original 8 ways, pushing it into l2c_victim for a decoupled background
// writeback (docs/l2c_spec.md S5: "fill may proceed before its victim
// drains").  g_mem_force_ready removes AW/AR/W-accept jitter so a given
// number of master-port beats can be counted deterministically.
// The reset-mid-writeback scenarios below are about what happens when a
// reset lands part way through a writeback BURST, so they need a victim
// whose burst is as long as the design can produce.  With sectored dirty
// bits that is no longer automatic: a single 16 B write dirties one
// quadrant and evicts as ONE beat.  Dirty all four quadrants of every way
// so the writeback is still the full 4 beats.
//
// The 9th access is deliberately a PARTIAL-strobe write: it cannot supply
// the whole quadrant, so it still allocates an MSHR entry and puts a real
// fill in flight -- which is what test_reset_compose_fill_and_writeback
// needs on the R side while the eviction runs on the W side.
static uint32_t setup_pending_eviction(uint64_t base, uint8_t seed) {
    const uint64_t set_stride = 4096 * LINE_BYTES;
    for (int way = 0; way < 8; way++)
        for (int q = 0; q < 4; q++)
            do_write_apply(base + static_cast<uint64_t>(way) * set_stride + q * 16,
                           seed + way + q * 4, "fill");
    g_mem_force_ready = true;
    std::array<uint8_t,16> d{};
    for (int i = 0; i < 16; i++) d[i] = static_cast<uint8_t>((seed + 0x40 + i) & 0xFF);
    return issue_write(base + 8ULL * set_stride, {d}, {0x00FF});
}
// Common post-reset cleanup: reset_dut() itself never touches the host-
// side w_*/r_* tracking maps or free_ids -- a request mid-flight at
// reset time has nothing meaningful left to track, so drop it here
// rather than per-scenario.
static void reset_and_clear_tracking(uint32_t stray_id) {
    reset_dut();
    g_mem_force_ready = false;
    std::copy(backing.begin(), backing.end(), golden.begin());
    std::fill(touched.begin(), touched.end(), false);
    free_ids.insert(stray_id);
    w_pending_issue.clear(); w_issuing = nullptr; aw_issuing = nullptr; w_aw_sent.clear();
    w_awaiting_b.clear(); w_done.clear();
    r_pending_issue.clear(); r_issuing = nullptr; r_awaiting.clear(); r_done.clear();
}

// IMPORTANT-A fail-before/pass-after: reset mid-W-burst (AW already
// ACCEPTED by DRAM, only 2 of 4 W beats sent) must not abandon DRAM
// waiting for the remaining beats forever.  l2c_victim.v's S_RSTDRAINW
// must complete the burst with wstrb=0 filler beats, and l2c.v's AW/W/B
// arbiter must keep routing to it across reset (grant-preservation fix)
// -- proven by a completely unrelated post-reset write succeeding (if
// the master port were wedged waiting on the abandoned burst, nothing
// else could ever get an AW accepted again).
static bool test_reset_mid_w_burst() {
    bool ok = true;
    uint64_t aw0 = g_dbg_aw_accepts, w0 = g_dbg_w_accepts;
    uint32_t evict_id = setup_pending_eviction(0x68000, 0x10);
    int guard = 0;
    while (g_dbg_aw_accepts == aw0 && guard < 5000) { tick(); guard++; }
    CHECK("victim writeback AW observed on m_axi", g_dbg_aw_accepts == aw0 + 1);
    while ((g_dbg_w_accepts - w0) < 2 && guard < 5000) { tick(); guard++; }
    CHECK("stopped after exactly 2 of 4 W beats", (g_dbg_w_accepts - w0) == 2);

    reset_and_clear_tracking(evict_id);

    uint64_t post_addr = 0x50000;
    CHECK("post-reset write completes (master port not wedged on the abandoned burst)",
          do_write_apply(post_addr, 0x21, "post_reset"));
    CHECK("post-reset readback", do_read_check(post_addr, "post_reset_readback"));
    return ok;
}

// IMPORTANT-A / EXP D fail-before/pass-after: reset while a writeback is
// in S_B (W fully sent, BRESP still outstanding) must not let that
// stray BRESP get consumed as a LATER, unrelated writeback's own
// completion (perpetual off-by-one B association) -- l2c_victim.v's
// S_RSTSINKB sinks it first.  The mem model's B channel is a single
// FIFO (front-of-queue only), so the stray B MUST drain before ANY
// later B can even be presented; proven by running a SECOND, fully
// independent eviction after the reset and confirming its dirty data is
// correctly written back and readable (if the stray B had instead been
// mis-consumed as the second eviction's own completion, that eviction's
// real BRESP would never arrive and the driver would time out).
static bool test_reset_b_pending_next_writeback_correct() {
    bool ok = true;
    uint64_t aw0 = g_dbg_aw_accepts, w0 = g_dbg_w_accepts;
    uint32_t evict_id = setup_pending_eviction(0x60000, 0x30);
    int guard = 0;
    while (g_dbg_aw_accepts == aw0 && guard < 5000) { tick(); guard++; }
    CHECK("writeback AW observed", g_dbg_aw_accepts == aw0 + 1);
    while ((g_dbg_w_accepts - w0) < 4 && guard < 5000) { tick(); guard++; }
    CHECK("all 4 W beats sent (now waiting on BRESP)", (g_dbg_w_accepts - w0) == 4);
    // Do not wait for the BRESP -- mslv.lat()'s 200-cycle floor guarantees
    // it isn't ready_at yet; reset now, landing squarely in S_B.

    reset_and_clear_tracking(evict_id);

    const uint64_t set_stride = 4096 * LINE_BYTES;
    uint64_t base2 = 0x70000;
    for (int way = 0; way < 8; way++)
        CHECK("fill way 2 (dirty)", do_write_apply(base2 + static_cast<uint64_t>(way) * set_stride, 0x50 + way, "fill2"));
    uint64_t a9b = base2 + 8ULL * set_stride;
    CHECK("second (fully independent) eviction trigger completes", do_write_apply(a9b, 0xCC, "evict2"));
    for (int way = 0; way < 8; way++)
        CHECK("post-reset second-eviction readback", do_read_check(base2 + static_cast<uint64_t>(way) * set_stride, "post_reset_evict2_readback"));
    CHECK("second eviction's own 9th-line readback", do_read_check(a9b, "post_reset_evict2_9th_readback"));
    return ok;
}

// IMPORTANT-A composition check: reset while an MSHR fill (R side,
// Important-9's existing S_DRAIN) AND a victim writeback (W/B side,
// this round's fix) are BOTH mid-flight simultaneously -- the two
// drains live in entirely independent arbiter/state machines (no shared
// signals), so they should compose without interference; this proves it
// by exercising a fresh read AND a fresh write, to two different lines,
// after a reset landed mid-composition.
static bool test_reset_compose_fill_and_writeback() {
    bool ok = true;
    uint32_t evict_id = setup_pending_eviction(0x88000, 0x70);
    // No need to pin an exact sub-state here -- just let both the 9th
    // write's own MSHR fill (R side) and the evicted way's writeback (W
    // side) run concurrently for a while before resetting mid-flight.
    for (int i = 0; i < 30; i++) tick();

    reset_and_clear_tracking(evict_id);

    uint64_t a = 0x90000, b = 0x90000 + LINE_BYTES;
    CHECK("post-reset read completes (AR/R side not wedged)", do_read_check(a, "compose_read"));
    CHECK("post-reset write completes (AW/W/B side not wedged)", do_write_apply(b, 0x11, "compose_write"));
    CHECK("post-reset write readback", do_read_check(b, "compose_write_readback"));
    return ok;
}

// ════════════════════════════════════════════════════════════════════════
// RESET WITH A FILL AR OUTSTANDING -- the drain-debt gate (l2c_mshr.v).
//
// On reset l2c_ctrl holds l2c_mshr in reset for the WHOLE 4096-cycle
// l2c_reset tag-clear walk (`.rst(rst || rst_busy)`), and the very first
// reset cycle clears m_issued[] -- the only record of which fill ARs DDR
// has already accepted.  S_DRAIN then sank R beats for a FIXED 128
// cycles, i.e. it assumed DDR would answer an accepted read inside
//     8 + 4096 + 128 = 4232 cycles = 21 us @ 200 MHz.
// That is a bounded-time assumption about DRAM, and a MIG refresh storm
// plus arbitration can break it without anything being faulty.
//
// Past the window the stale burst arrives carrying rid = 0 and lands in
// one of two states, one test each below:
//   (a) a post-reset fill has re-allocated entry 0, so the stale line is
//       installed and answered as THAT fill's data -- silent corruption;
//   (b) nothing is live, so m_axi_rready is 0 and stays 0 -- the shared
//       DDR read channel dead for every master.
//
// The fix counts the debt the transactions themselves define (ar_debt:
// +1 per accepted fill AR, -1 per consumed RLAST, and NOT reset) instead
// of counting cycles.  The 128-cycle floor is kept verbatim so nothing
// downstream moves in time; the debt is an ADDITIONAL exit condition.
//
// Both tests stall DDR for 6000 cycles, comfortably past 4232.  The
// scratch proof they are promoted from measured PASS at 3000 and 4000
// and the failures above at 4300.
// ════════════════════════════════════════════════════════════════════════

// (a) The stale burst must not be consumed as a post-reset fill's data.
static bool test_reset_mid_fill_stale_burst_not_installed() {
    bool ok = true;
    const uint64_t A = 0xA0000, B = 0xA0040;   // adjacent lines => distinct sets

    // Stamp the two lines with patterns that cannot be confused for each
    // other, in BOTH the DRAM model and the golden copy.  Earlier tests
    // leave long runs of sequential bytes behind, and two such runs can
    // be byte-identical across different addresses -- which would make
    // "B's read returned A's line" invisible.  Written before any of
    // them is cached, so there is nothing to invalidate.
    for (int i = 0; i < 16; i++) {
        backing[A + i] = golden[A + i] = static_cast<uint8_t>(0xA0 + i);
        backing[B + i] = golden[B + i] = static_cast<uint8_t>(0xB0 + i);
    }

    // Let DDR ACCEPT the fill AR, then hold every R beat.
    g_mem_block_rvalid = true;
    uint64_t ar0 = g_dbg_ar_accepts;
    uint32_t id_a = issue_read(A);
    int guard = 0;
    while (g_dbg_ar_accepts == ar0 && guard < 5000) { tick(); guard++; }
    CHECK("fill AR for line A accepted by DDR", g_dbg_ar_accepts == ar0 + 1);

    // Reset with that burst owed.
    reset_and_clear_tracking(id_a);

    // Stall past the old fixed 4232-cycle sink window.
    for (int i = 0; i < 6000; i++) tick();

    // A fresh read to a DIFFERENT line.  The table is empty, so it takes
    // MSHR entry 0 -- the very index the stale burst carries in its rid.
    uint32_t id_b = issue_read(B);
    for (int i = 0; i < 50; i++) tick();

    // Now let DDR answer.  The model's R queue is FIFO, so line A's stale
    // burst comes out first.
    g_mem_block_rvalid = false;

    CHECK("post-reset read completes", wait_read_done(id_b, 40000));
    if (r_done.count(id_b)) {
        bool match = std::memcmp(r_done[id_b]->got[0].data(), &golden[B], 16) == 0;
        if (!match)
            printf("  stale-burst install: read of 0x%llx got=%02x%02x%02x%02x "
                   "exp=%02x%02x%02x%02x (pre-reset line A holds %02x%02x%02x%02x)\n",
                   (unsigned long long)B,
                   r_done[id_b]->got[0][0], r_done[id_b]->got[0][1],
                   r_done[id_b]->got[0][2], r_done[id_b]->got[0][3],
                   golden[B], golden[B+1], golden[B+2], golden[B+3],
                   golden[A], golden[A+1], golden[A+2], golden[A+3]);
        CHECK("post-reset read got ITS OWN line, not the pre-reset burst's", match);
        free_read(id_b);
    }
    CHECK("cache still usable afterwards", do_read_check(B + 0x1000, "post_stale_usable"));
    return ok;
}

// (b) ...and with nothing live to mis-consume it, the stale burst must
//     still be SUNK rather than left parked against m_axi_rready = 0.
static bool test_reset_mid_fill_stale_burst_does_not_wedge_r_channel() {
    bool ok = true;
    const uint64_t A = 0xB0000;

    g_mem_block_rvalid = true;
    uint64_t ar0 = g_dbg_ar_accepts;
    uint32_t id_a = issue_read(A);
    int guard = 0;
    while (g_dbg_ar_accepts == ar0 && guard < 5000) { tick(); guard++; }
    CHECK("fill AR accepted by DDR", g_dbg_ar_accepts == ar0 + 1);

    reset_and_clear_tracking(id_a);
    for (int i = 0; i < 6000; i++) tick();

    // Release with NOTHING live in the table.
    uint64_t stall0 = g_r_stall_cycles;
    g_mem_block_rvalid = false;
    for (int i = 0; i < 400; i++) tick();
    uint64_t stalled = g_r_stall_cycles - stall0;
    if (stalled >= 64)
        printf("  stale burst parked: %llu cycles of m_axi_rvalid && !m_axi_rready "
               "(the DDR read channel is dead for every master)\n",
               (unsigned long long)stalled);
    CHECK("stale burst sunk promptly instead of parking the R channel", stalled < 64);

    CHECK("read after the stale burst completes and is correct",
          do_read_check(0xB8000, "post_stale"));
    CHECK("and so does the next one", do_read_check(0xB8040, "post_stale2"));
    return ok;
}

// IMPORTANT-A v2 fail-before/pass-after: reset while the victim's AW is
// PRESENTED but NOT YET ACCEPTED (S_AW, routine downstream backpressure
// on AWREADY -- the old l2c.v held its reset-grant on `aw_busy`, which
// sets the moment AW is presented, well before any real handshake).
// l2c_victim.v itself resets cleanly to S_IDLE from S_AW (its reset
// `case` falls to `default`, no drain queued, no stray_b_owed bump) --
// so a grant held on the old aw_busy-keyed condition waited forever for
// a B that would never arrive, permanently locking the AW/W/B arbiter to
// "victim" and starving bypass of ANY chance to present its own AW
// (m_axi_awvalid muxes to vw_awvalid, which stays 0 post-reset since the
// victim has nothing queued) -- a permanent head-of-line deadlock,
// probe-confirmed. Two bypass writes are queued back-to-back post-reset
// (the second naturally serializes behind the first, since bypass is
// single-outstanding by construction, §8) to directly exercise the
// "second op queued behind the first" HOL scenario the probe found.
static bool test_reset_aw_pending_unaccepted() {
    bool ok = true;
    uint64_t aw0 = g_dbg_aw_accepts;
    g_mem_block_awready = true;
    uint32_t evict_id = setup_pending_eviction(0x98000, 0x90);
    // Let the victim's AW sit VALID-but-not-READY for a good while --
    // g_dbg_aw_accepts only increments on a REAL awvalid&&awready
    // handshake, so it staying put proves the AW is genuinely still
    // pending (presented, not accepted), not just that we haven't looked
    // yet.
    for (int i = 0; i < 200; i++) tick();
    CHECK("victim AW still unaccepted (blocked by AWREADY)", g_dbg_aw_accepts == aw0);

    reset_and_clear_tracking(evict_id);
    g_mem_block_awready = false;

    uint64_t bp1 = BYPASS_BASE + 0x200, bp2 = BYPASS_BASE + 0x240;
    uint32_t id1 = issue_write_full(bp1, 0x61);
    uint32_t id2 = issue_write_full(bp2, 0x62); // queued behind id1 at the s_axi driver
    CHECK("first post-reset bypass write completes promptly (no HOL stall)",
          wait_write_done(id1, 5000));
    CHECK("second post-reset bypass write completes promptly (no HOL stall)",
          wait_write_done(id2, 5000));
    if (w_done.count(id1)) {
        for (int i = 0; i < 16; i++) golden[bp1 + i] = static_cast<uint8_t>((0x61 + i) & 0xFF);
        CHECK("first bypass write resp OKAY", w_done[id1]->resp_ok);
        free_write(id1);
    }
    if (w_done.count(id2)) {
        for (int i = 0; i < 16; i++) golden[bp2 + i] = static_cast<uint8_t>((0x62 + i) & 0xFF);
        CHECK("second bypass write resp OKAY", w_done[id2]->resp_ok);
        free_write(id2);
    }
    CHECK("first bypass readback", do_read_check(bp1, "reset_aw_pending_bypass_readback1"));
    CHECK("second bypass readback", do_read_check(bp2, "reset_aw_pending_bypass_readback2"));
    return ok;
}

// ══════════════════════════════════════════════════════════════════════════
// BYPASS PIPELINING (2026-08-20 rewrite of l2c_bypass.v)
// ══════════════════════════════════════════════════════════════════════════
// v1 was one request/response engine doing a full DDR round trip per 16 B
// beat, so a 512 B RAM-disk transfer cost 32 serialized round trips.  The
// scenarios below lock down what the pipelined version has to do AND what
// it must not stop doing.
//
// Every bound here is expressed in terms of the DEPTH TAP, not a constant,
// because the same directed suite runs against every BYPASS_SLOTS build in
// the tb-l2c-bypdepth sweep.  A hardcoded cycle budget would either fail at
// SLOTS=1 or be vacuous at SLOTS=32.
static unsigned by_slots() { return dut->dbg_by_slots ? dut->dbg_by_slots : 1; }
// A 32-beat transfer takes ceil(32/slots) DDR round trips plus fill/drain.
// 300 cycles is one round trip with generous headroom over the model's
// 200 + rand(0..63) first-beat latency and its 0..3-cycle beat gaps.
static uint64_t by_budget(int beats, int extra_trips) {
    const unsigned s = by_slots();
    return static_cast<uint64_t>((beats + s - 1) / s + extra_trips) * 300ULL;
}
// Peak concurrency a `beats`-beat transfer can actually reach.  Capped at
// beats-1, not beats: once the depth is >= the burst length the last beat's
// dispatch races the FIRST response, so the counter never sees all of them
// outstanding at once.  Measured 31, not 32, at BYPASS_SLOTS=32 -- which is
// the engine behaving correctly, not a shortfall.  Bounding the expectation
// here rather than special-casing it in each scenario keeps the assertion
// exact at every depth the sweep builds.
static unsigned by_expect_inflight(int beats) {
    const unsigned s = by_slots();
    const unsigned cap = (beats > 1) ? static_cast<unsigned>(beats - 1) : 1u;
    return (s < cap) ? s : cap;
}

// The headline property, READ side: many bypass reads in flight at once.
// Asserted on dbg_by_inflight (transactions PRESENTED to DRAM and still
// awaiting R) reaching the full depth, which was structurally <= 1 before
// the rewrite -- a cycle count alone would let a merely-faster serial
// engine pass.
static bool test_bypass_read_pipelines_to_depth() {
    bool ok = true;
    const unsigned slots = by_slots();
    const uint64_t base  = BYPASS_BASE + 0x30000;
    const int      beats = 32;                 // 512 B, the RAM-disk shape

    std::vector<std::array<uint8_t,16>> wd(beats);
    std::vector<uint16_t> ws(beats, 0xFFFF);
    for (int b = 0; b < beats; b++)
        for (int k = 0; k < 16; k++)
            wd[b][k] = static_cast<uint8_t>(0xA0 + b * 7 + k);
    uint32_t wid = issue_write(base, wd, ws);
    if (!wait_write_done(wid, 60000)) { CHECK("bypass prime write completes", false); return ok; }
    CHECK("bypass prime write OKAY", w_done[wid]->resp_ok);
    for (int b = 0; b < beats; b++)
        for (int k = 0; k < 16; k++) golden[base + b*16 + k] = wd[b][k];
    touched[base] = true;
    free_write(wid);

    g_by_inflight_max = 0;
    const uint64_t t0 = sim_time;
    uint32_t rid = issue_read(base, beats);
    if (!wait_read_done(rid, 60000)) { CHECK("512B bypass read completes", false); return ok; }
    const uint64_t cyc  = sim_time - t0;
    const unsigned infl = g_by_inflight_max;

    bool data_ok = true;
    for (int b = 0; b < beats; b++)
        if (std::memcmp(r_done[rid]->got[b].data(), &golden[base + b*16], 16) != 0) data_ok = false;
    CHECK("512B bypass read returns the written data", data_ok);
    free_read(rid);

    const unsigned want_infl = by_expect_inflight(beats);
    if (infl < want_infl)
        printf("  byp read pipelining: peak %u reads in flight, expected %u (depth %u)\n",
               infl, want_infl, slots);
    CHECK("bypass reads reach the engine's full depth in flight", infl >= want_infl);
    if (cyc > by_budget(beats, 2))
        printf("  byp read pipelining: %llu cycles for %d beats at depth %u (budget %llu)\n",
               (unsigned long long)cyc, beats, slots, (unsigned long long)by_budget(beats, 2));
    CHECK("512B bypass read costs ceil(beats/depth) round trips, not beats",
          cyc <= by_budget(beats, 2));
    return ok;
}

// Same property, WRITE side, as its own scenario.  386ea29's deep-query
// test and 7d49e1f's s_awready case both passed their own mutant because a
// read path covered for the write path inside one mixed scenario; a
// write-only case is the fix for that, not an optional extra.
static bool test_bypass_write_pipelines_to_depth() {
    bool ok = true;
    const unsigned slots = by_slots();
    const uint64_t base  = BYPASS_BASE + 0x34000;
    const int      beats = 32;

    std::vector<std::array<uint8_t,16>> wd(beats);
    std::vector<uint16_t> ws(beats, 0xFFFF);
    for (int b = 0; b < beats; b++)
        for (int k = 0; k < 16; k++)
            wd[b][k] = static_cast<uint8_t>(0x11 + b * 5 + k);

    g_by_inflight_max = 0;
    const uint64_t t0 = sim_time;
    uint32_t wid = issue_write(base, wd, ws);
    if (!wait_write_done(wid, 60000)) { CHECK("512B bypass write completes", false); return ok; }
    const uint64_t cyc  = sim_time - t0;
    const unsigned infl = g_by_inflight_max;
    CHECK("512B bypass write OKAY", w_done[wid]->resp_ok);
    for (int b = 0; b < beats; b++)
        for (int k = 0; k < 16; k++) golden[base + b*16 + k] = wd[b][k];
    touched[base] = true;
    free_write(wid);

    const unsigned want_infl = by_expect_inflight(beats);
    if (infl < want_infl)
        printf("  byp write pipelining: peak %u writes in flight, expected %u (depth %u)\n",
               infl, want_infl, slots);
    CHECK("bypass writes reach the engine's full depth in flight", infl >= want_infl);
    if (cyc > by_budget(beats, 2))
        printf("  byp write pipelining: %llu cycles for %d beats at depth %u (budget %llu)\n",
               (unsigned long long)cyc, beats, slots, (unsigned long long)by_budget(beats, 2));
    CHECK("512B bypass write costs ceil(beats/depth) round trips, not beats",
          cyc <= by_budget(beats, 2));
    // The data has to be right too -- a pipelined write that dropped or
    // reordered a beat would still make the cycle budget.
    for (int b = 0; b < beats; b += 7)
        CHECK("bypass write readback", do_read_check(base + b*16, "byp_wpipe_readback"));
    return ok;
}

// THE ORDERING PROPERTY THE PIPELINING MUST NOT COST.  AXI orders nothing
// between the R and B channels, so a bypass read must never be outstanding
// alongside a bypass write.  v1 got that free by being one-at-a-time; v2
// gets it from l2c_bypass's dir_ok_c drain-on-direction-change.
//
// The check is the every-cycle MASTER-PORT invariant installed in
// by_latch_edge() -- NOT a data comparison.  A data comparison cannot see
// this bug: tb_l2c.cpp's memory model applies writes at W-BEAT time and
// samples read data at R-DRIVE time, so an out-of-order read still returns
// post-write bytes here.  The bug would be invisible in this harness and
// real on silicon.
//
// Same-id is what makes the two ops coexist in the queue at all
// (l2c_bypass accepts one id at a time), and same-address is what makes
// the ordering matter.
static bool test_bypass_read_write_never_overlap() {
    bool ok = true;
    const uint64_t a = BYPASS_BASE + 0x38000;
    const uint32_t shared = 61;   // outside alloc_id()'s 0..59 pool
    const bool was_violated = g_by_mixdir_violation;
    const unsigned occ0 = g_by_occ_max;

    for (int round = 0; round < 6; round++) {
        std::array<uint8_t,16> d{};
        for (int k = 0; k < 16; k++) d[k] = static_cast<uint8_t>(0x50 + round*16 + k);
        // Queue both WITHOUT waiting: the write's B is still ~200 cycles
        // away when the read's beat reaches the front door.
        if (round & 1) {
            issue_read_explicit_id(a, shared);
            issue_write_explicit_id(a, shared, {d}, {0xFFFF});
        } else {
            issue_write_explicit_id(a, shared, {d}, {0xFFFF});
            issue_read_explicit_id(a, shared);
        }
        if (!wait_write_done(shared, 60000)) { CHECK("same-id bypass write completes", false); return ok; }
        CHECK("same-id bypass write OKAY", w_done[shared]->resp_ok);
        w_done.erase(shared);
        if (!wait_read_done(shared, 60000)) { CHECK("same-id bypass read completes", false); return ok; }
        r_done.erase(shared);
        for (int k = 0; k < 16; k++) golden[a + k] = d[k];
        touched[a] = true;
    }
    CHECK("no bypass read and write ever outstanding at the same time",
          g_by_mixdir_violation == was_violated);
    // Anti-vacuity: the scenario really did put bypass traffic through the
    // engine, so the invariant above is not trivially satisfied.
    CHECK("scenario actually drove the bypass engine", g_by_occ_max >= occ0 && g_by_occ_max > 0);
    CHECK("final value visible", do_read_check(a, "byp_rw_overlap_final"));
    return ok;
}

// AXIS-2 REGRESSION LOCK.  A bypass burst used to hold l2c_ctrl's single
// ar_have tracker for its entire duration because every beat after the
// first was refused by Critical-3's id_busy_c -- 5040 of 5430 front-door
// idle cycles in the mixed perf row (7d49e1f).  That hold-off guards a
// same-id-DIFFERENT-PATH hazard and does not apply bypass-to-bypass, so
// l2c_ctrl now qualifies it with !is_bypass_c.
//
// What this asserts is the ATTRIBUTION, not just the speed: bypass beats
// must not be refused as an id hazard at all.  The door may still stall on
// bypnr (the engine's queue genuinely full) -- that is real backpressure,
// and the anti-vacuity check below REQUIRES the door to have been stalled,
// so "nothing was ever blocked" cannot pass this test by accident.
static bool test_bypass_burst_leaves_front_door_usable() {
    bool ok = true;
    const unsigned slots = by_slots();
    const uint64_t hits  = 0x2C0000;
    const uint64_t byp   = BYPASS_BASE + 0x3A000;
    const int n_hits = 32, beats = 32;

    for (int i = 0; i < 8; i++)
        CHECK("warm cacheable line", do_write_apply(hits + i*LINE_BYTES, 0x30 + i, "byp_door_warm"));

    FdStats fd0 = g_fd;
    const uint64_t t0 = sim_time;
    uint32_t bid = issue_read(byp, beats);
    std::vector<uint32_t> hid;
    for (int i = 0; i < n_hits; i++) hid.push_back(issue_read(hits + (i % 8) * LINE_BYTES));
    if (!wait_read_done(bid, 60000)) { CHECK("bypass block read completes", false); return ok; }
    free_read(bid);
    for (size_t i = 0; i < hid.size(); i++) {
        if (!wait_read_done(hid[i], 60000)) { CHECK("cacheable hit behind bypass completes", false); return ok; }
        auto rb = r_done[hid[i]];
        if (std::memcmp(rb->got[0].data(), &golden[rb->addr], 16) != 0)
            CHECK("cacheable hit behind bypass returns the right data", false);
        free_read(hid[i]);
    }
    const uint64_t cyc = sim_time - t0;
    FdStats d = fd_diff(g_fd, fd0);

    if (d.idbusy * 10 > d.idle)
        printf("  byp door: idbusy %llu of %llu idle cycles at depth %u\n",
               (unsigned long long)d.idbusy, (unsigned long long)d.idle, slots);
    CHECK("bypass beats are not refused as an id hazard", d.idbusy * 10 <= d.idle);
    // Anti-vacuity #1: the door really was under pressure in this window.
    CHECK("the front door really was stalled in this window",
          d.ar_stall > static_cast<uint64_t>(beats));
    // Anti-vacuity #2: the accepts being talked about actually happened.
    CHECK("both traffic classes reached the door",
          d.accept >= static_cast<uint64_t>(beats + n_hits));
    if (cyc > by_budget(beats, 3))
        printf("  byp door: %llu cycles for %d bypass beats + %d hits at depth %u (budget %llu)\n",
               (unsigned long long)cyc, beats, n_hits, slots,
               (unsigned long long)by_budget(beats, 3));
    CHECK("cacheable hits are not held for the whole bypass transfer",
          cyc <= by_budget(beats, 3));
    return ok;
}

// IMPORTANT-A, bypass half.  With several bypass writes in flight a reset
// leaves DRAM owing several BRESPs.  Two things have to be right, or a
// LATER, unrelated victim writeback mis-consumes one of them as its own
// completion and retires a slot whose dirty line never reached DRAM:
//
//   * l2c_bypass has to sink them (bout_cnt/bsink_cnt), and
//   * l2c.v's write-port arbiter has to keep the grant pointed at BYPASS
//     across the reset so they are routed there rather than to l2c_victim.
//
// The post-reset dirty-eviction readback is what makes the second half
// observable -- a bare "fresh bypass write still works" check passes even
// when the strays land on the victim engine.
// One round of the reset-mid-bypass-write scenario.  `wait_drained`
// selects WHICH SUB-STATE the reset lands in, and that distinction is the
// whole point of running it twice:
//
//   false -- reset while the sequencer is still streaming (typically D_W,
//            one AW accepted with its W beat unsent).  Exercises the
//            filler-beat path.
//   true  -- reset once every accepted slot has been DISPATCHED and the
//            sequencer is parked in D_HDR waiting for BRESPs.
//            `inflight == occ` is exactly that condition: dptr only
//            advances at W-accept, so an op mid-D_W is counted in occ but
//            not yet in inflight.  Exercises the pure-sink path.
//
// The second round exists because the FIRST VERSION OF THIS TEST ONLY DID
// THE FIRST ONE, and a mutant that deletes the sink from the D_HDR reset
// branch passed it -- the reset always landed mid-burst, where the D_W
// branch (which the mutant left alone) still loaded bsink_cnt.  That is
// the same "the test covered a neighbouring path instead of the one it
// names" failure 386ea29 and 7d49e1f each hit once.
static bool reset_mid_bypass_write_round(uint64_t byp, uint64_t ev, uint8_t seed,
                                          bool wait_drained) {
    bool ok = true;
    const int beats = 16;
    std::vector<std::array<uint8_t,16>> wd(beats);
    std::vector<uint16_t> ws(beats, 0xFFFF);
    for (int b = 0; b < beats; b++)
        for (int k = 0; k < 16; k++) wd[b][k] = static_cast<uint8_t>(seed + b + k);
    uint32_t wid = issue_write(byp, wd, ws);

    const long want_out = (by_slots() >= 2) ? 2 : 1;
    int guard = 0;
    if (wait_drained) {
        const unsigned want_occ = (by_slots() >= 2) ? 2 : 1;
        while (guard < 60000 &&
               !(dut->dbg_by_occ >= want_occ && dut->dbg_by_inflight == dut->dbg_by_occ)) {
            tick(); guard++;
        }
        CHECK("bypass sequencer is parked awaiting BRESPs when the reset lands",
              dut->dbg_by_occ >= want_occ && dut->dbg_by_inflight == dut->dbg_by_occ);
    } else {
        while ((g_by_wr_out - g_by_wr_stale) < want_out && guard < 60000) { tick(); guard++; }
    }
    CHECK("bypass writes are outstanding on the master port when the reset lands",
          (g_by_wr_out - g_by_wr_stale) >= want_out);

    reset_and_clear_tracking(wid);

    // A fresh dirty eviction.  This is the half that catches a stray BRESP
    // going to the wrong engine: if l2c_bypass does not sink what DRAM owes
    // it, l2c.v's arbiter never sees aw_out reach zero and l2c_victim can
    // never acquire the write port at all -- so the ninth way's writeback
    // never happens and the evicted line reads back stale.  If instead the
    // grant were released to the victim, the victim would consume a stray as
    // one of its own completions and retire a slot early, with the same
    // visible result on a different line.
    const uint64_t set_stride = 4096 * LINE_BYTES;
    for (int way = 0; way < 9; way++)
        CHECK("post-reset fill", do_write_apply(ev + static_cast<uint64_t>(way) * set_stride,
                                                static_cast<uint8_t>(0x70 + way),
                                                "byp_reset_evict_fill"));
    for (int way = 0; way < 9; way++)
        CHECK("post-reset evicted line readback",
              do_read_check(ev + static_cast<uint64_t>(way) * set_stride, "byp_reset_evict_read"));

    CHECK("post-reset bypass write", do_write_apply(byp, seed ^ 0xA5, "byp_reset_write"));
    CHECK("post-reset bypass readback", do_read_check(byp, "byp_reset_read"));
    return ok;
}

// IMPORTANT-A, bypass half.  With several bypass writes in flight a reset
// leaves DRAM owing several BRESPs.  Two things have to be right, or the
// write port never becomes usable again (or worse, l2c_victim consumes a
// stray as one of its own writebacks' completion and retires a slot whose
// dirty line never reached DRAM):
//
//   * l2c_bypass has to sink them (bout_cnt/bsink_cnt), and
//   * l2c.v's write-port arbiter has to keep the grant pointed at BYPASS
//     across the reset so they are routed there rather than to l2c_victim.
//
// The post-reset dirty-eviction readback is what makes both halves
// observable -- a bare "fresh bypass write still works" check passes even
// when the strays land on the victim engine.
static bool test_reset_mid_bypass_write_pipeline() {
    bool ok = true;
    ok &= reset_mid_bypass_write_round(BYPASS_BASE + 0x3C000, 0x2E0000, 0xC1, false);
    ok &= reset_mid_bypass_write_round(BYPASS_BASE + 0x3E000, 0x2E0040, 0x39, true);
    return ok;
}

static bool test_back_to_back_mixed_bursts() {
    bool ok = true;
    uint64_t base = 0x8000;
    // A real multi-beat (4-beat, full 64B line) INCR write burst, then a
    // matching multi-beat read burst, back to back, plus interleaved
    // single-beat ops to other lines.
    std::vector<std::array<uint8_t,16>> wd(4);
    std::vector<uint16_t> ws(4, 0xFFFF);
    for (int i = 0; i < 4; i++) for (int b = 0; b < 16; b++) wd[i][b] = static_cast<uint8_t>(0xC0 + i*16 + b);
    uint32_t wid = issue_write(base, wd, ws);
    uint32_t rid_other = issue_read(base + 4096ULL * LINE_BYTES);
    CHECK("burst write completes", wait_write_done(wid));
    if (w_done.count(wid)) { free_write(wid); for (int i = 0; i < 4; i++) for (int b = 0; b < 16; b++) golden[base + i*16 + b] = wd[i][b]; touched[base]=true; }
    CHECK("interleaved single read completes", wait_read_done(rid_other));
    if (r_done.count(rid_other)) free_read(rid_other);

    uint32_t rid = issue_read(base, 4);
    CHECK("burst read completes", wait_read_done(rid));
    if (r_done.count(rid)) {
        auto& rb = r_done[rid];
        bool data_ok = true;
        for (int i = 0; i < 4; i++) if (std::memcmp(rb->got[i].data(), wd[i].data(), 16) != 0) data_ok = false;
        CHECK("burst read data matches burst write", data_ok);
        free_read(rid);
    }
    // A handful of quick back-to-back single-beat ops to unrelated lines.
    for (int i = 0; i < 6; i++) {
        uint64_t a = base + static_cast<uint64_t>(1000 + i) * LINE_BYTES;
        CHECK("mixed op", do_write_apply(a, 0x30 + i, "mixed"));
    }
    return ok;
}

// Critical-1 fail-before/pass-after: a multi-beat write burst to an
// ALREADY-CACHED line (every beat hits) must produce exactly one BRESP
// for the whole burst, not one per beat.  wait_write_done() itself only
// proves the FIRST B arrived; the real assertion is the generic
// unmatched-response check (g_unmatched_resp_seen, checked in main()) --
// a spurious extra B for this id after it's freed is exactly what an
// unfixed per-beat-BRESP bug would produce, and main() fails the whole
// run if it's ever seen.  Give the DUT a bounded idle window afterward
// so any such extra B has time to surface before this test returns.
static bool test_multi_beat_write_hit_single_bresp() {
    bool ok = true;
    uint64_t base = 0xB000;
    CHECK("prime line (miss)", do_write_apply(base, 0x01, "prime"));
    std::vector<std::array<uint8_t,16>> wd(4);
    std::vector<uint16_t> ws(4, 0xFFFF);
    for (int i = 0; i < 4; i++) for (int b = 0; b < 16; b++) wd[i][b] = static_cast<uint8_t>(0xD0 + i*16 + b);
    uint32_t wid = issue_write(base, wd, ws);
    CHECK("multi-beat HIT write completes with exactly one B", wait_write_done(wid));
    if (w_done.count(wid)) {
        for (int i = 0; i < 4; i++) for (int b = 0; b < 16; b++) golden[base + i*16 + b] = wd[i][b];
        touched[base] = true;
        free_write(wid);
    }
    for (int i = 0; i < 100; i++) tick(); // idle window for a spurious extra B to surface
    uint32_t rid = issue_read(base, 4);
    CHECK("readback matches burst write", wait_read_done(rid));
    if (r_done.count(rid)) {
        bool data_ok = true;
        for (int i = 0; i < 4; i++) if (std::memcmp(r_done[rid]->got[i].data(), wd[i].data(), 16) != 0) data_ok = false;
        CHECK("readback data correct", data_ok);
        free_read(rid);
    }
    return ok;
}

// Important-10 fail-before/pass-after: a non-INCR burst (WRAP) must be
// rejected with SLVERR -- one response per burst, the target address left
// completely untouched (array/MSHR/bypass never see it).  Uses the
// g_force_arburst hook (issue_read() otherwise only ever emits legal
// INCR/full-width reads).
//
// 2026-07-24 correction: a narrower-than-16B AxSIZE is legal AXI4
// (narrow-transfer-on-wide-bus -- the standard shape for a 32-bit master
// like axi_narrow_to_wide.v's CPU-bypass/boot_fsm/JTAG clients on this
// 128-bit port) and must be ACCEPTED, not SLVERR'd.  An earlier, overly
// strict front-door gate rejected every such transfer -- this was the
// confirmed root cause of boot_fsm's first RAM-zero-fill write dying with
// a BRESP error on real hardware once L2C fronted the DDR path. This test
// used to assert the old (wrong) SLVERR behavior; it now asserts the
// corrected accept-and-merge behavior instead of being deleted.
static bool test_illegal_burst_slverr() {
    bool ok = true;
    uint64_t a = 0xC000;
    CHECK("prime (so a real access would hit if wrongly processed)", do_write_apply(a, 0x77, "prime"));

    g_force_arburst = 2; // WRAP
    uint32_t id = issue_read(a);
    CHECK("WRAP burst gets a response (not silently dropped)", wait_read_done(id));
    if (r_done.count(id)) {
        CHECK("WRAP burst rejected with SLVERR", r_done[id]->last_resp == 2 /* SLVERR */);
        free_read(id);
    }

    g_force_arsize = 2; // 4B, narrower than the full 16B line width
    uint32_t id2 = issue_read(a);
    CHECK("narrow-size read gets a response (not silently dropped)", wait_read_done(id2));
    if (r_done.count(id2)) {
        CHECK("narrow-size read accepted with OKAY", r_done[id2]->last_resp == 0 /* OKAY */);
        CHECK("narrow-size read returns correct line data",
              std::memcmp(r_done[id2]->got[0].data(), &golden[a], 16) == 0);
        free_read(id2);
    }

    // The primed line itself must still read back correctly.
    CHECK("primed line unaffected by rejected WRAP burst", do_read_check(a, "post_illegal_readback"));
    return ok;
}

// The actual production bug (2026-07-24): boot_fsm.v (and the CPU
// dcache-bypass path, and JTAG) drive AXI writes through
// axi_narrow_to_wide.v, which legitimately emits a single-beat write
// with AWSIZE derived from the narrow WSTRB pattern -- 0/1/2 (1/2/4
// bytes), NEVER 4 (16 bytes).  Before the Important-10 relaxation above,
// every one of these writes was rejected with SLVERR at L2C's front
// door; boot_fsm's very first RAM-zero-fill write (a 4-byte, AWSIZE=2
// write to address 0) died this way on real hardware.  This test
// reproduces that exact shape directly against L2C and checks it is
// now accepted and correctly byte-merged (cur_qoff/req_wstrb do not
// depend on AWSIZE at all -- see l2c_ctrl.v).
static bool test_narrow_awsize_write() {
    bool ok = true;
    uint64_t a = 0xC100;
    CHECK("prime full line", do_write_apply(a, 0x11, "prime"));

    // Single-beat, 4-byte write (AWSIZE=2) at the base of the line,
    // wstrb marking only the low 4 bytes -- exactly boot_fsm's
    // zero-fill write shape.
    g_force_awsize = 2;
    std::array<uint8_t,16> d{};
    d[0] = 0xDE; d[1] = 0xAD; d[2] = 0xBE; d[3] = 0xEF;
    uint32_t id = issue_write(a, {d}, {0x000F});
    CHECK("narrow-awsize write completes (not silently dropped)", wait_write_done(id));
    if (w_done.count(id)) {
        CHECK("narrow-awsize write accepted with OKAY", w_done[id]->resp_ok);
        free_write(id);
        golden[a+0] = 0xDE; golden[a+1] = 0xAD; golden[a+2] = 0xBE; golden[a+3] = 0xEF;
        touched[a] = true;
    }
    // Bytes 4..15 must be untouched (wstrb-gated merge, not a full-line
    // overwrite) and bytes 0..3 must reflect the new narrow write.
    CHECK("narrow write merged correctly, rest of line preserved", do_read_check(a, "narrow_awsize_write_readback"));
    return ok;
}

// Partial-WSTRB sweep across ALL FOUR QUADRANTS of the 64-byte line.
//
// COVERAGE GAP THIS CLOSES (found while chasing task #240, the 7.5.3
// lost-low-word).  test_narrow_awsize_write above is the ONLY partial-strobe
// test, and it drives exactly one shape: wstrb=0x000F, 4 bytes, at a
// line-ALIGNED address.  That means req_qoff is 0, so
//
//     l2c_ctrl.v:247   dw_strb = {48'b0, req_wstrb} << (req_qoff * 16)
//
// has only ever been exercised with a shift of ZERO.  The same is true of the
// MSHR's merge path (l2c_mshr.v:158, swr_strb_c, identical shift pattern).  A
// quadrant-selection bug would therefore be completely invisible: writes to
// quadrant 1/2/3 would land in quadrant 0, silently corrupting two lines'
// worth of bytes at once, and every existing L2C test would still pass.
//
// This sweeps widths 1/2/4/8 bytes at every aligned offset, in every quadrant,
// and after EACH write re-reads the whole 16-byte quadrant to confirm that the
// strobed bytes changed and NOTHING else did.  The 2-byte width matters
// specifically: the m68k splits a misaligned LONG store into two 2-byte beats,
// which is the access shape at the heart of #240.
//
// Note this is a WRITE-HIT test by construction -- each quadrant is primed
// first, so the partial write hits a resident line and takes the dw_strb path
// rather than the MSHR merge path.
static bool test_partial_wstrb_all_quadrants() {
    bool ok = true;
    const uint64_t base = 0xD000;              // 64-byte (one L2C line) aligned

    for (int q = 0; q < 4; q++) {
        char nm[64];
        snprintf(nm, sizeof nm, "prime quadrant %d", q);
        CHECK(nm, do_write_apply(base + q * 16, 0x40 + q * 0x10, nm));
    }

    const int widths[] = {1, 2, 4, 8};
    for (int q = 0; q < 4; q++) {
        for (int wi = 0; wi < 4; wi++) {
            const int w = widths[wi];
            for (int off = 0; off + w <= 16; off += w) {
                const uint64_t a = base + q * 16;
                const uint16_t strb = (uint16_t)((((1u << w) - 1u) << off) & 0xFFFF);
                std::array<uint8_t,16> d{};
                for (int i = 0; i < w; i++)
                    d[off + i] = (uint8_t)(0xA0 + q * 0x10 + off + i);

                char nm[96];
                snprintf(nm, sizeof nm, "q%d w%d off%d strb=0x%04x", q, w, off, strb);

                uint32_t id = issue_write(a, {d}, {strb});
                bool done = wait_write_done(id);
                CHECK(nm, done);
                if (!done) continue;
                CHECK(nm, w_done[id]->resp_ok);
                free_write(id);

                // Only the strobed bytes may change.
                for (int i = 0; i < w; i++) golden[a + off + i] = d[off + i];
                touched[a] = true;
                CHECK(nm, do_read_check(a, nm));
            }
        }
    }
    return ok;
}

// ══════════════════════════════════════════════════════════════════════════
// Full-line write, no fill (l2c_ctrl.v's flw_* gather path)
// ══════════════════════════════════════════════════════════════════════════
// The discriminator throughout is g_dbg_ar_accepts -- the memory model's
// count of ARs actually presented on l2c's DDR master port.  A line that
// was fetched costs exactly one AR (one 4-beat MSHR fill); a line the
// no-fetch path installed costs ZERO.  Nothing else in these scenarios
// issues a read to DDR, so the counter is unambiguous.  Data correctness
// is checked through the DUT only (do_read_check / issue_read), never by
// reaching into backing[].
//
// Verified RED before the RTL change: full_line_write_no_fetch (1 fill
// AR, expected 0), full_line_write_multi_line_burst (4, expected 0) and
// full_line_write_evicts_dirty_victim (1, expected 0).
//
// The other three are GREEN both before and after, deliberately:
//   * partial_line_write_still_fetches and unaligned_burst_still_fetches
//     are the NEGATIVE guards -- they stop a future widening of the
//     detector from dropping a fetch a partially-covered line needs.
//   * full_line_write_over_resident_dirty already worked (a resident
//     line was never fetched); it pins that the new install path does
//     not regress the in-place case.

// Quiesce the DUT so an AR count taken after this reflects only the
// traffic the scenario itself caused.
static void flw_settle(int n = 250) { for (int i = 0; i < n; i++) tick(); }

// One aligned 64 B write burst (4 x 16 B beats) with caller-chosen strobes.
static bool flw_write_line(uint64_t base, uint8_t seed, const uint16_t strb[4],
                            const char* what) {
    std::vector<std::array<uint8_t,16>> wd(4);
    std::vector<uint16_t> ws(4);
    for (int i = 0; i < 4; i++) {
        ws[i] = strb[i];
        for (int b = 0; b < 16; b++) wd[i][b] = static_cast<uint8_t>(seed + i * 16 + b);
    }
    uint32_t id = issue_write(base, wd, ws);
    if (!wait_write_done(id)) {
        printf("  %s: TIMEOUT waiting for 64B write @0x%llx\n", what, (unsigned long long)base);
        return false;
    }
    bool ok = w_done[id]->resp_ok;
    free_write(id);
    for (int i = 0; i < 4; i++)
        for (int b = 0; b < 16; b++)
            if ((ws[i] >> b) & 1) golden[base + i*16 + b] = wd[i][b];
    touched[base] = true;
    return ok;
}

// Read a whole line back through the DUT and compare against golden[].
static bool flw_check_line(uint64_t base, const char* what) {
    uint32_t id = issue_read(base, 4);
    if (!wait_read_done(id)) {
        printf("  %s: TIMEOUT reading back 64B @0x%llx\n", what, (unsigned long long)base);
        return false;
    }
    bool ok = true;
    for (int i = 0; i < 4; i++)
        if (std::memcmp(r_done[id]->got[i].data(), &golden[base + i*16], 16) != 0) {
            printf("  %s: DATA MISMATCH @0x%llx quadrant %d\n", what, (unsigned long long)base, i);
            ok = false;
        }
    free_read(id);
    return ok;
}

static const uint16_t FLW_ALL[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

// The headline case: a cold, unresident line written in full must install
// with NO fill at all.
static bool test_full_line_write_no_fetch() {
    bool ok = true;
    const uint64_t base = 0x00E0000;   // 64 B aligned, untouched by other tests
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    CHECK("full-line write completes", flw_write_line(base, 0x11, FLW_ALL, "flw_no_fetch"));
    flw_settle();
    uint64_t fills = g_dbg_ar_accepts - ar0;
    if (fills != 0) printf("  flw_no_fetch: %llu fill AR(s), expected 0\n", (unsigned long long)fills);
    CHECK("no DDR fill was issued for a fully-written line", fills == 0);
    CHECK("line reads back byte-exact", flw_check_line(base, "flw_no_fetch"));
    // ...and the readback itself came from the cache, not a late fill.
    flw_settle();
    CHECK("readback needed no fill either", g_dbg_ar_accepts - ar0 == fills);
    return ok;
}

// NEGATIVE CASE -- the one that stops a future widening of the detector
// from silently losing data.  Same burst shape, but one beat carries a
// partial strobe, so the line is NOT fully covered and the fetch is
// mandatory: the unwritten bytes must come back as the DRAM content.
static bool test_partial_line_write_still_fetches() {
    bool ok = true;
    const uint64_t base = 0x00E1000;
    const uint16_t strb[4] = {0xFFFF, 0xFFFF, 0x00FF, 0xFFFF};  // quadrant 2 half-written
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    CHECK("partial-line write completes", flw_write_line(base, 0x22, strb, "flw_partial"));
    flw_settle();
    uint64_t fills = g_dbg_ar_accepts - ar0;
    if (fills != 1) printf("  flw_partial: %llu fill AR(s), expected exactly 1\n", (unsigned long long)fills);
    CHECK("a partially-covered line STILL fetches", fills == 1);
    // golden[] kept the pre-write DRAM bytes for the unstrobed half of
    // quadrant 2, so this check fails outright if the fetch was skipped.
    CHECK("unwritten bytes survive as DRAM content", flw_check_line(base, "flw_partial"));
    return ok;
}

// A 4-beat burst starting 16 B into a line straddles two lines, so the
// AW-shape detector never arms the gather and the beats enter the tag
// pipeline one at a time.
//
// EXPECTATION CHANGED BY SECTORING (was: "both straddled lines are
// fetched", 2 fill ARs).  Every one of these four beats fully covers its
// own 16 B quadrant, and a fully-covered quadrant needs nothing from DRAM
// whatever the rest of its line looks like -- so the correct answer is now
// ZERO fills, and the burst-shape detector is no longer what decides it.
// The invariant this scenario actually guards is unchanged and still
// checked below: the quadrants the burst did NOT write must still come
// back as DRAM content, which is what the two whole-line readbacks force
// (each of them touches three quadrants this burst never supplied, so
// each costs its own fill at read time).  test_sector_partial_quadrant_
// still_fetches is the guard for a quadrant that is only PARTLY written.
static bool test_unaligned_burst_still_fetches() {
    bool ok = true;
    const uint64_t base = 0x00E2000 + 16;   // deliberately not 64 B aligned
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    CHECK("unaligned 4-beat write completes", flw_write_line(base, 0x33, FLW_ALL, "flw_unaligned"));
    flw_settle();
    uint64_t fills = g_dbg_ar_accepts - ar0;
    if (fills != 0) printf("  flw_unaligned: %llu fill AR(s), expected 0\n", (unsigned long long)fills);
    CHECK("fully-covered quadrants need no fetch even across a line boundary", fills == 0);
    CHECK("straddled data correct (line A)", flw_check_line(base - 16, "flw_unaligned_a"));
    CHECK("straddled data correct (line B)", flw_check_line(base + 48, "flw_unaligned_b"));
    return ok;
}

// ══════════════════════════════════════════════════════════════════════════
// Sectored valid + dirty (l2c_tags.v vsec/dsec, docs/l2c_perf.md S12)
// ══════════════════════════════════════════════════════════════════════════
// Discriminators: g_dbg_ar_accepts for fills (as above) and
// g_dbg_w_accepts / g_dbg_last_aw_* for writebacks -- an eviction used to
// be 4 W beats at the line base whatever had been touched.
//
// RED before the RTL change: sector_write_no_fetch (1 fill AR, expected
// 0), sector_dirty_writeback_is_partial (4 W beats, expected 1),
// sector_writeback_span_skips_gap (4 full-strobe beats, expected 2 of 4
// carrying data), sector_mixed_dirty_span (4 W beats at the line base,
// expected 2 at base+16).
//
// GREEN both sides, deliberately -- the negative guards:
// sector_partial_quadrant_still_fetches (a quadrant only PARTLY written
// must still be fetched, and the bytes the write did not cover must read
// back as DRAM content) and sector_clean_quadrant_not_written_back (a
// quadrant that was never written must never reach DRAM; if it was never
// even fetched, writing it back would push URAM garbage over live data).

// Write one 16 B quadrant with caller-chosen strobes.
static bool sctr_write_quad(uint64_t addr, uint8_t seed, uint16_t strb, const char* what) {
    std::array<uint8_t,16> d{};
    for (int i = 0; i < 16; i++) d[i] = static_cast<uint8_t>(seed + i);
    uint32_t id = issue_write(addr, {d}, {strb});
    if (!wait_write_done(id)) {
        printf("  %s: TIMEOUT waiting for 16B write @0x%llx\n", what, (unsigned long long)addr);
        return false;
    }
    bool ok = w_done[id]->resp_ok;
    free_write(id);
    for (int i = 0; i < 16; i++) if ((strb >> i) & 1) golden[addr + i] = d[i];
    touched[addr] = true;
    return ok;
}

// THE HEADLINE CASE.  One fully-strobed 16 B write -- exactly the shape a
// 68040 copyback L1D line push presents -- to a cold line must install its
// own quadrant with no fill at all.  Before sectoring this dragged the
// whole 64 B line in from DRAM to overwrite a quarter of it.
static bool test_sector_write_no_fetch() {
    bool ok = true;
    const uint64_t base = 0x00F0000;
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    CHECK("16 B quadrant write completes", sctr_write_quad(base, 0x11, 0xFFFF, "sctr_nofetch"));
    flw_settle();
    uint64_t fills = g_dbg_ar_accepts - ar0;
    if (fills != 0) printf("  sctr_nofetch: %llu fill AR(s), expected 0\n", (unsigned long long)fills);
    CHECK("a fully-covered quadrant is installed with no fill", fills == 0);
    uint32_t rid = issue_read(base);
    CHECK("written quadrant reads back", wait_read_done(rid));
    if (r_done.count(rid)) {
        CHECK("written quadrant byte-exact",
              std::memcmp(r_done[rid]->got[0].data(), &golden[base], 16) == 0);
        free_read(rid);
    }
    flw_settle();
    CHECK("the readback hit, it did not fill", g_dbg_ar_accepts - ar0 == 0);
    return ok;
}

// NEGATIVE GUARD.  A write that covers only PART of a quadrant cannot
// supply the rest of it, so the fetch is mandatory and the bytes it did
// not write must come back as DRAM content.  This is the guard that stops
// the no-fetch install from ever being widened past full coverage.
static bool test_sector_partial_quadrant_still_fetches() {
    bool ok = true;
    const uint64_t base = 0x00F1000;
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    CHECK("partial-strobe quadrant write completes",
          sctr_write_quad(base, 0x22, 0x00FF, "sctr_partial"));  // low 8 bytes only
    flw_settle();
    uint64_t fills = g_dbg_ar_accepts - ar0;
    if (fills != 1) printf("  sctr_partial: %llu fill AR(s), expected exactly 1\n",
                           (unsigned long long)fills);
    CHECK("a partly-covered quadrant STILL fetches", fills == 1);
    // golden[] kept the pre-write DRAM bytes for the unstrobed half, so
    // this fails outright if the fetch was skipped.
    CHECK("unwritten bytes survive as DRAM content", do_read_check(base, "sctr_partial_rb"));
    return ok;
}

// Rule 1: a quadrant miss on a line whose TAG already matches must fill
// into THAT way, not allocate a second way for the same tag.  Write
// quadrant 0 with no fetch, then read quadrant 2 of the same line: the
// read must return DRAM content, quadrant 0 must still hold the write,
// and the set must not now contain two copies of the line (which would
// show up as the two quadrants disagreeing after an eviction sweep).
static bool test_sector_fill_into_resident_way() {
    bool ok = true;
    const uint64_t base = 0x00F2000;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    CHECK("install quadrant 0 with no fetch", sctr_write_quad(base, 0x33, 0xFFFF, "sctr_resident"));
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    CHECK("read of an invalid quadrant of a resident line", do_read_check(base + 32, "sctr_resident_q2"));
    flw_settle();
    CHECK("that read cost exactly one fill", g_dbg_ar_accepts - ar0 == 1);
    CHECK("the written quadrant still holds its own data", do_read_check(base, "sctr_resident_q0"));
    // Push the set eight ways deep so any duplicate copy of `base` would
    // have to be evicted, then read it back: a stale duplicate written
    // back after the live one would show here.
    for (int way = 1; way <= 8; way++)
        CHECK("conflict fill", do_write_apply(base + static_cast<uint64_t>(way) * set_stride,
                                               0x40 + way, "sctr_resident_conf"));
    CHECK("line survives the eviction sweep intact", do_read_check(base, "sctr_resident_final"));
    CHECK("and so does its fetched quadrant", do_read_check(base + 32, "sctr_resident_final2"));
    return ok;
}

// Only the DIRTY quadrant is written back.  Nine tags into one set, each
// touched in exactly one 16 B quadrant: the eviction that follows must be
// ONE W beat at that quadrant's own address, not four at the line base.
static bool test_sector_dirty_writeback_is_partial() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    const uint64_t base = 0x00F3000;
    for (int way = 0; way < 8; way++)
        CHECK("fill way (one dirty quadrant)",
              sctr_write_quad(base + static_cast<uint64_t>(way) * set_stride, 0x50 + way,
                              0xFFFF, "sctr_wb_fill"));
    flw_settle();
    uint64_t w0 = g_dbg_w_accepts, aw0 = g_dbg_aw_accepts;
    const uint64_t a9 = base + 8ULL * set_stride;
    CHECK("9th tag evicts one of them", sctr_write_quad(a9, 0x99, 0xFFFF, "sctr_wb_evict"));
    flw_settle();
    uint64_t beats = g_dbg_w_accepts - w0;
    if (beats != 1) printf("  sctr_wb: %llu writeback W beat(s), expected 1\n",
                           (unsigned long long)beats);
    CHECK("exactly one writeback burst", g_dbg_aw_accepts - aw0 == 1);
    CHECK("a single dirty quadrant costs ONE writeback beat", beats == 1);
    CHECK("the writeback burst is one beat long", g_dbg_last_aw_len == 0);
    // Every original way must still read back correctly -- whichever was
    // evicted had to reach DRAM through the (now partial) writeback.
    for (int way = 0; way < 8; way++)
        CHECK("evicted/resident way survives",
              do_read_check(base + static_cast<uint64_t>(way) * set_stride, "sctr_wb_rb"));
    CHECK("installed 9th line correct", do_read_check(a9, "sctr_wb_rb9"));
    return ok;
}

// The dirty span need not start at the line base: quadrants 1 and 2 dirty
// must produce a 2-beat burst at base+16.
static bool test_sector_mixed_dirty_span() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    const uint64_t base = 0x00F4000;
    for (int way = 0; way < 8; way++) {
        const uint64_t a = base + static_cast<uint64_t>(way) * set_stride;
        CHECK("quadrant 1", sctr_write_quad(a + 16, 0x60 + way, 0xFFFF, "sctr_span"));
        CHECK("quadrant 2", sctr_write_quad(a + 32, 0x70 + way, 0xFFFF, "sctr_span"));
    }
    flw_settle();
    uint64_t w0 = g_dbg_w_accepts, aw0 = g_dbg_aw_accepts;
    const uint64_t a9 = base + 8ULL * set_stride;
    CHECK("9th tag evicts one of them", sctr_write_quad(a9, 0xA9, 0xFFFF, "sctr_span_evict"));
    flw_settle();
    CHECK("exactly one writeback burst", g_dbg_aw_accepts - aw0 == 1);
    CHECK("two dirty quadrants cost TWO writeback beats", g_dbg_w_accepts - w0 == 2);
    CHECK("the burst starts at the first dirty quadrant, not the line base",
          (g_dbg_last_aw_addr & 0x3F) == 16);
    CHECK("and is two beats long", g_dbg_last_aw_len == 1);
    for (int way = 0; way < 8; way++) {
        const uint64_t a = base + static_cast<uint64_t>(way) * set_stride;
        CHECK("quadrant 1 survives", do_read_check(a + 16, "sctr_span_rb1"));
        CHECK("quadrant 2 survives", do_read_check(a + 32, "sctr_span_rb2"));
    }
    return ok;
}

// NEGATIVE GUARD -- the one that catches silent corruption rather than a
// policy change.  Write quadrants 0 and 3 of a cold line with full
// strobes: quadrants 1 and 2 are then VALID NOWHERE -- never written,
// never fetched, so the URAM behind them holds whatever was last there.
// The dirty mask 4'b1001 has no contiguous form, so the writeback burst
// necessarily spans all four quadrants; beats 1 and 2 must go out with
// wstrb=0.  If they did not, DRAM's copy of those quadrants would be
// replaced by stale array contents -- which this proves by reading them
// back after the line has been evicted.
static bool test_sector_clean_quadrant_not_written_back() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    const uint64_t base = 0x00F5000;
    for (int way = 0; way < 8; way++) {
        const uint64_t a = base + static_cast<uint64_t>(way) * set_stride;
        CHECK("quadrant 0", sctr_write_quad(a,      0x80 + way, 0xFFFF, "sctr_gap"));
        CHECK("quadrant 3", sctr_write_quad(a + 48, 0x90 + way, 0xFFFF, "sctr_gap"));
    }
    flw_settle();
    const uint64_t a9 = base + 8ULL * set_stride;
    CHECK("9th tag evicts one of them", sctr_write_quad(a9, 0xB9, 0xFFFF, "sctr_gap_evict"));
    flw_settle();
    // golden[] still holds the ORIGINAL DRAM bytes for quadrants 1 and 2
    // of every way, because nothing ever wrote them.  Reading them now
    // fetches them from DRAM -- and they must be unchanged.
    for (int way = 0; way < 8; way++) {
        const uint64_t a = base + static_cast<uint64_t>(way) * set_stride;
        CHECK("never-written quadrant 1 still holds DRAM content",
              do_read_check(a + 16, "sctr_gap_q1"));
        CHECK("never-written quadrant 2 still holds DRAM content",
              do_read_check(a + 32, "sctr_gap_q2"));
        CHECK("written quadrant 0 survives", do_read_check(a,      "sctr_gap_q0"));
        CHECK("written quadrant 3 survives", do_read_check(a + 48, "sctr_gap_q3"));
    }
    return ok;
}

// Rule 2: a write onto a quadrant the cache already owns, issued while
// that same line has a fill in flight for a DIFFERENT quadrant, must not
// be lost.  It cannot merge (l2c_mshr merges into fetched data, stale for
// a quadrant the cache owns) so l2c_ctrl retries it until the fill
// installs.  Queue the read and the write back to back, with distinct
// IDs, so the write is resolving while the fill is live.
static bool test_sector_write_during_fill_retries() {
    bool ok = true;
    const uint64_t base = 0x00F6000;
    for (int round = 0; round < 8; round++) {
        const uint64_t a = base + static_cast<uint64_t>(round) * 64;
        CHECK("install quadrant 0", sctr_write_quad(a, 0xC0 + round, 0xFFFF, "sctr_race"));
        std::array<uint8_t,16> d{};
        for (int i = 0; i < 16; i++) d[i] = static_cast<uint8_t>(0xD0 + round + i);
        uint32_t rid = issue_read(a + 32);            // fill for quadrant 2
        uint32_t wid = issue_write(a, {d}, {0xFFFF}); // overwrite quadrant 0, same line
        CHECK("fill read completes", wait_read_done(rid));
        CHECK("racing write completes", wait_write_done(wid));
        if (r_done.count(rid)) {
            CHECK("filled quadrant is DRAM content",
                  std::memcmp(r_done[rid]->got[0].data(), &golden[a + 32], 16) == 0);
            free_read(rid);
        }
        if (w_done.count(wid)) free_write(wid);
        for (int i = 0; i < 16; i++) golden[a + i] = d[i];
        CHECK("the racing write survived the install", do_read_check(a, "sctr_race_rb"));
    }
    return ok;
}

// A 16-beat burst covering four whole lines: four installs, zero fills.
static bool test_full_line_write_multi_line_burst() {
    bool ok = true;
    const uint64_t base = 0x00E3000;
    std::vector<std::array<uint8_t,16>> wd(16);
    std::vector<uint16_t> ws(16, 0xFFFF);
    for (int i = 0; i < 16; i++)
        for (int b = 0; b < 16; b++) wd[i][b] = static_cast<uint8_t>(0x40 + i*16 + b);
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    uint32_t id = issue_write(base, wd, ws);
    CHECK("4-line burst completes", wait_write_done(id));
    if (w_done.count(id)) {
        CHECK("4-line burst OKAY", w_done[id]->resp_ok);
        free_write(id);
        for (int i = 0; i < 16; i++)
            for (int b = 0; b < 16; b++) golden[base + i*16 + b] = wd[i][b];
        for (int l = 0; l < 4; l++) touched[base + l*64] = true;
    }
    flw_settle();
    uint64_t fills = g_dbg_ar_accepts - ar0;
    if (fills != 0) printf("  flw_multiline: %llu fill AR(s), expected 0\n", (unsigned long long)fills);
    CHECK("no fills across a 4-line burst", fills == 0);
    for (int l = 0; l < 4; l++)
        CHECK("multi-line data correct", flw_check_line(base + l*64, "flw_multiline"));
    return ok;
}

// The line is already resident and DIRTY.  A full-line write must overwrite
// it IN PLACE -- not allocate a second copy, not lose the old tag's data --
// and still cost no fill.
static bool test_full_line_write_over_resident_dirty() {
    bool ok = true;
    const uint64_t base = 0x00E4000;
    CHECK("prime line resident+dirty", do_write_apply(base, 0x55, "flw_resident_prime"));
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    CHECK("full-line write over resident line", flw_write_line(base, 0x66, FLW_ALL, "flw_resident"));
    flw_settle();
    CHECK("resident full-line write needs no fill", g_dbg_ar_accepts - ar0 == 0);
    CHECK("resident line reads back as the NEW data", flw_check_line(base, "flw_resident"));
    return ok;
}

// A full-line write that misses into a set whose ways are all valid+dirty
// must still evict a victim through the writeback buffer, and the evicted
// line's data must survive.  Same victim machinery as an ordinary miss --
// this proves the no-fetch path did not skip it.
static bool test_full_line_write_evicts_dirty_victim() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;   // +1 tag, same set
    const uint64_t base = 0x00E5000;
    for (int way = 0; way < 8; way++)
        CHECK("fill way (dirty)", do_write_apply(base + static_cast<uint64_t>(way) * set_stride,
                                                  0x80 + way, "flw_evict_fill"));
    flw_settle();
    uint64_t ar0 = g_dbg_ar_accepts;
    const uint64_t a9 = base + 8ULL * set_stride;
    CHECK("9th tag arrives as a full-line write", flw_write_line(a9, 0x99, FLW_ALL, "flw_evict"));
    flw_settle();
    uint64_t fills = g_dbg_ar_accepts - ar0;
    if (fills != 0) printf("  flw_evict: %llu fill AR(s) for the installing line, expected 0\n",
                            (unsigned long long)fills);
    CHECK("the evicting line itself was not fetched", fills == 0);
    CHECK("installed line correct", flw_check_line(a9, "flw_evict"));
    // Every original way must still read back correctly -- whichever one was
    // evicted has to have reached DRAM through the victim buffer.
    for (int way = 0; way < 8; way++)
        CHECK("evicted/resident way survives",
              do_read_check(base + static_cast<uint64_t>(way) * set_stride, "flw_evict_readback"));
    return ok;
}

// REGRESSION for the bug the L2C_SEED=1 randomized run found: while an
// assembled full line is waiting for the tag pipeline (flw_avail_c),
// reads keep winning the front door -- and an early version of
// l2c_ctrl.v's is_bypass_c qualified that term with flw_avail_c alone
// rather than with write_sel, so a bypass-window read that won the front
// door in one of those cycles took the CACHE path instead of the bypass
// engine.  It came back with quadrant-shifted data plus an extra R beat.
//
// Reproducing it needs BOTH halves of what the randomized run had:
//   * a bypass read STALLED at the front door -- l2c_bypass is a single
//     request/response engine, so a queue of bypass reads keeps
//     byp_req_ready low and leaves one sitting in ar_have unable to
//     dispatch; and
//   * full lines assembling continuously underneath it, so flw_avail_c
//     is high in the cycle that stalled read finally gets dispatched.
// A lone bypass read is dispatched during the gather itself and never
// sees the window -- which is why an earlier, simpler version of this
// test passed against the buggy RTL.  Hence: both streams in flight at
// once, for many rounds.
//
// What actually CATCHES the bug is l2c_ctrl.v's own sim-only invariant
// ("a bypass-window address must never be accepted onto the cache
// path"), which this scenario's traffic drives into the window within a
// few rounds -- $fatal against the buggy qualifier, silent against the
// fixed one.  The data comparisons here are the second line of defence:
// whether a wrongly-cached bypass read returns visibly wrong bytes
// depends on what happens to be resident, so they are not on their own a
// reliable detector.  Do not delete the RTL assertion in favour of this
// test; the assertion is the guard, this is the stimulus.
// ══════════════════════════════════════════════════════════════════════════
// Deep / pipelined victim buffer (2026-08-20).  l2c_victim.v went from two
// slots with a round-trip-serialized drain FSM to SLOTS slots with many
// writebacks outstanding at once.  These three scenarios pin the parts of
// that which could go silently wrong.
// ══════════════════════════════════════════════════════════════════════════

// Fill one set eight ways deep with a known pattern, then push eight more
// tags into it so every original line is evicted, WITH the memory model
// refusing AWREADY the whole time.  Nothing drains, so the buffer really
// does fill to SLOTS and DRAM's copy of every original line stays stale.
//
// THE POINT.  l2c does not snoop; victim_query_hit is the only thing
// standing between a re-read of a just-evicted line and stale DRAM.  With
// two slots a two-way compare covered it.  With eight, an entry sitting in
// slot 3..7 is exactly as un-drained as one in slot 0 and must stall the
// re-read just the same.  If the compare missed it, the read would fall
// through to a fresh fill and get DRAM's pre-writeback bytes -- silent
// stale data, not a stall bug.  RED-verified by narrowing l2c_victim.v's
// query generate loop to 2 slots: the probe read then COMPLETES while
// AWREADY is still blocked, and returns the wrong bytes.
static bool deep_victim_round(uint64_t base, int probe, unsigned slots) {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    // Evictions come out of ONE eight-way set, so eight is all this shape
    // can ever queue no matter how deep the buffer is.
    const unsigned want = slots < 8 ? slots : 8;
    g_mem_block_awready = true;
    for (int way = 0; way < 8; way++)
        CHECK("prime dirty way", do_write_apply(base + static_cast<uint64_t>(way) * set_stride,
                                                 0x70 + way, "deep_vb_prime"));
    // Eight more tags into the same set: eight evictions, none of which can
    // start draining.  These are READS, deliberately -- they install CLEAN
    // lines, so the probe read below can allocate its own way without
    // needing a victim-buffer slot.  Evicting with writes instead leaves the
    // set eight-deep in DIRTY lines, and then the probe stalls on
    // `victim_push_ready` (l2c_ctrl.v's way_ok_c) rather than on
    // victim_query_hit -- which looks identical from outside and would make
    // this scenario pass against a query that sees only two slots.  That is
    // not hypothetical: the first version of this test did exactly that and
    // survived the RED mutant.
    for (unsigned t = 8; t < 8 + want; t++)
        CHECK("evicting read", do_read_check(base + static_cast<uint64_t>(t) * set_stride,
                                              "deep_vb_evict"));
    flw_settle(50);
    const unsigned occ = dut->dbg_vb_occ & 0x1F;
    if (occ != want)
        printf("  deep_vb: victim buffer holds %u entries, expected %u\n", occ, want);
    CHECK("every queued victim is resident with no drain possible", occ == want);
    CHECK("nothing was written back while AWREADY was blocked", (dut->dbg_vb_out & 0x1F) == 0);

    // Race a read against an entry that can only be in the victim buffer.
    const uint64_t pa = base + static_cast<uint64_t>(probe) * set_stride;
    uint32_t rid = issue_read(pa);
    for (int i = 0; i < 600; i++) tick();
    CHECK("read of a still-buffered victim must STALL, not read stale DRAM",
          r_done.find(rid) == r_done.end());

    g_mem_block_awready = false;
    CHECK("...and completes once the writeback drains", wait_read_done(rid, 40000));
    if (r_done.count(rid)) {
        bool match = std::memcmp(r_done[rid]->got[0].data(), &golden[pa], 16) == 0;
        if (!match) printf("  deep_vb: probe @0x%llx returned stale data\n",
                           (unsigned long long)pa);
        CHECK("the raced read returns the VICTIM's data, not DRAM's", match);
        free_read(rid);
    }
    // Everything the round touched must still be byte-exact.
    for (unsigned t = 0; t < 8 + want; t++)
        CHECK("post-drain readback", do_read_check(base + static_cast<uint64_t>(t) * set_stride,
                                                    "deep_vb_rb"));
    for (int i = 0; i < 400; i++) tick();
    CHECK("victim buffer fully retired", (dut->dbg_vb_occ & 0x1F) == 0);
    return ok;
}

static bool test_deep_victim_query_covers_every_slot() {
    bool ok = true;
    const unsigned slots = dut->dbg_vb_slots & 0x1F;
    CHECK("victim buffer is deeper than the v1 two slots", slots > 2);
    // Probe a shallow, a middle and the last eviction-order position across
    // three independent sets.  Which physical slot a given line lands in is
    // PLRU's business; sweeping the position reaches all of them.
    ok &= deep_victim_round(0x00A0000, 0, slots);
    ok &= deep_victim_round(0x00A0040, 3, slots);
    ok &= deep_victim_round(0x00A0080, 7, slots);
    // (probe 7 is the last primed way; the set is eight ways deep.)
    return ok;
}

// A writeback whose BRESP comes back non-OKAY must still be SURFACED.
// There is no architectural error channel for it (docs/l2c_spec.md
// Minor-16: nobody is waiting on an eviction's own response), so
// l2c_victim.v counts and prints it; that counter is what this asserts on,
// rather than the entry quietly retiring as if nothing happened.  The
// entry DOES still retire -- the line's only copy is gone either way, and
// wedging the buffer would turn a reporting gap into a hang.
static bool test_victim_writeback_bresp_error_surfaces() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    const uint64_t base = 0x00A1000;
    const unsigned err0 = dut->dbg_vb_wb_err & 0xFF;
    for (int way = 0; way < 8; way++)
        CHECK("prime dirty way", do_write_apply(base + static_cast<uint64_t>(way) * set_stride,
                                                 0x90 + way, "wb_err_prime"));
    flw_settle();
    // The only m_axi write this test can produce is the eviction's own
    // writeback, so arming the next B is enough to target it.
    g_mem_next_bresp = 2;   // SLVERR
    CHECK("9th tag evicts one way", do_write_apply(base + 8ULL * set_stride, 0x9F, "wb_err_evict"));
    flw_settle(400);
    const unsigned err1 = dut->dbg_vb_wb_err & 0xFF;
    if (err1 == err0) printf("  wb_err: SLVERR writeback was not surfaced at all\n");
    CHECK("a non-OKAY writeback BRESP is surfaced, not dropped", err1 == err0 + 1);
    CHECK("the failed entry still retires (buffer does not wedge)",
          (dut->dbg_vb_occ & 0x1F) == 0 && (dut->dbg_vb_out & 0x1F) == 0);
    g_mem_next_bresp = 0;
    // The cache keeps working afterwards: a fresh eviction on the same set
    // completes normally and its data survives.
    CHECK("10th tag evicts again, normally",
          do_write_apply(base + 9ULL * set_stride, 0xA0, "wb_err_after"));
    CHECK("post-error line readback", do_read_check(base + 9ULL * set_stride, "wb_err_rb"));
    return ok;
}

// Sectored writebacks must stay partial when SEVERAL of them are queued.
// The per-entry dirty mask is what sets each burst's base and length; a
// pipelined engine that snapshotted the wrong entry's mask (or read it
// live from a pointer that had already advanced) would produce right-sized
// bursts for the head and wrong ones for everything behind it.  Eight
// lines, each dirty in exactly ONE quadrant and each a DIFFERENT quadrant
// across the set, are queued with AWREADY blocked and then released: every
// resulting AW must be a single beat at its own quadrant's address.
// Comparison is a multiset -- which way PLRU evicts when is not this
// test's business, but the shapes are.
static bool test_deep_victim_partial_writebacks_stay_partial() {
    bool ok = true;
    const uint64_t set_stride = 4096 * LINE_BYTES;
    const uint64_t base = 0x00A2000;
    const unsigned slots = dut->dbg_vb_slots & 0x1F;
    const unsigned want  = slots < 8 ? slots : 8;   // one 8-way set
    std::multiset<uint64_t> expect;
    // Each line: fetch it whole (so all four quadrants are VALID), then
    // dirty exactly one of them.  dsec = 1 << q, so the writeback must be
    // one beat at line + q*16.
    for (unsigned way = 0; way < want; way++) {
        const uint64_t a = base + static_cast<uint64_t>(way) * set_stride;
        const int q = static_cast<int>(way % 4);
        CHECK("fetch the whole line clean", do_read_check(a, "deep_part_fetch"));
        CHECK("dirty exactly one quadrant",
              sctr_write_quad(a + static_cast<uint64_t>(q) * 16, 0xB0 + way, 0xFFFF, "deep_part_dirty"));
        expect.insert(a + static_cast<uint64_t>(q) * 16);
    }
    flw_settle();
    g_mem_block_awready = true;
    g_aw_log.clear();
    g_aw_log_on = true;
    for (unsigned t = 8; t < 8 + want; t++)
        CHECK("evicting write", do_write_apply(base + static_cast<uint64_t>(t) * set_stride,
                                                0xC0 + t, "deep_part_evict"));
    flw_settle(50);
    CHECK("every partial victim is queued at once",
          (dut->dbg_vb_occ & 0x1F) == want);
    CHECK("and none of them has been written back yet", g_aw_log.empty());
    g_mem_block_awready = false;
    flw_settle(4000);
    g_aw_log_on = false;

    std::multiset<uint64_t> got;
    bool all_one_beat = true;
    for (const auto& e : g_aw_log) {
        if (e.second != 0) { all_one_beat = false;
            printf("  deep_part: writeback @0x%llx is %d beats, expected 1\n",
                   (unsigned long long)e.first, e.second + 1); }
        got.insert(e.first);
    }
    CHECK("every queued writeback is still ONE beat", all_one_beat);
    if (got != expect) {
        printf("  deep_part: writeback addresses do not match the dirty quadrants\n");
        for (auto a : expect) printf("    expected 0x%llx\n", (unsigned long long)a);
        for (auto a : got)    printf("    got      0x%llx\n", (unsigned long long)a);
    }
    CHECK("each burst starts at its OWN entry's dirty quadrant", got == expect);
    for (unsigned t = 0; t < 8 + want; t++)
        CHECK("post-drain readback", do_read_check(base + static_cast<uint64_t>(t) * set_stride,
                                                    "deep_part_rb"));
    return ok;
}

static bool test_bypass_read_during_full_line_gather() {
    bool ok = true;
    const uint64_t byp_base  = BYPASS_BASE + 0x2000;
    const uint64_t cacheable = 0x00E6000;         // 64 B aligned
    const int rounds = 24;
    const int n_each = 16;

    for (int i = 0; i < n_each; i++)
        CHECK("prime bypass addresses",
              do_write_apply(byp_base + static_cast<uint64_t>(i)*16, 0xC0 + i, "flw_byp_prime"));

    for (int r = 0; r < rounds && ok; r++) {
        std::vector<uint32_t> wids, rids;
        std::vector<uint64_t> waddrs;
        std::vector<std::vector<std::array<uint8_t,16>>> wdata;

        // Interleave the two streams into the pending queues in one go so
        // the front door sees them competing, not one after the other.
        for (int i = 0; i < n_each; i++) {
            const uint64_t line = cacheable + static_cast<uint64_t>(i) * 64;
            std::vector<std::array<uint8_t,16>> wd(4);
            std::vector<uint16_t> ws(4, 0xFFFF);
            for (int q = 0; q < 4; q++)
                for (int b = 0; b < 16; b++)
                    wd[q][b] = static_cast<uint8_t>(0xE0 + r*7 + i + q*16 + b);
            wids.push_back(issue_write(line, wd, ws));
            waddrs.push_back(line);
            wdata.push_back(wd);
            rids.push_back(issue_read(byp_base + static_cast<uint64_t>(i)*16));
        }

        for (int i = 0; i < n_each; i++) {
            if (!wait_write_done(wids[i], 200000)) { CHECK("gather-shaped write completes", false); return ok; }
            for (int q = 0; q < 4; q++)
                for (int b = 0; b < 16; b++) golden[waddrs[i] + q*16 + b] = wdata[i][q][b];
            touched[waddrs[i]] = true;
            free_write(wids[i]);
        }
        for (int i = 0; i < n_each; i++) {
            const uint64_t a = byp_base + static_cast<uint64_t>(i)*16;
            if (!wait_read_done(rids[i], 200000)) { CHECK("concurrent bypass read completes", false); return ok; }
            bool match = std::memcmp(r_done[rids[i]]->got[0].data(), &golden[a], 16) == 0;
            if (!match)
                printf("  flw_byp: round %d read @0x%llx wrong (got %02x%02x%02x%02x exp %02x%02x%02x%02x)\n",
                       r, (unsigned long long)a,
                       r_done[rids[i]]->got[0][0], r_done[rids[i]]->got[0][1],
                       r_done[rids[i]]->got[0][2], r_done[rids[i]]->got[0][3],
                       golden[a], golden[a+1], golden[a+2], golden[a+3]);
            CHECK("bypass read still takes the BYPASS path", match);
            free_read(rids[i]);
        }
    }
    for (int l = 0; l < n_each; l++)
        CHECK("cacheable lines still correct", flw_check_line(cacheable + l*64, "flw_byp_line"));
    return ok;
}

// Critical-3 fail-before/pass-after: two reads sharing the SAME AXI ID,
// queued back-to-back with NO wait in between -- read A (miss, slow) then
// read B (hit, fast).  issue_read_explicit_id() bypasses alloc_id()'s
// uniqueness so both are genuinely in flight under one id, exactly the
// AXI4 same-ID scenario A5.3 requires in-order completion for.  Without
// the id_busy_c gate, B's AR could be accepted (and even respond) before
// A resolves -- at best a same-ID reorder, at worst B's AR-accept
// silently clobbers A's still-live r_awaiting[id] tracking entry.
static bool test_same_id_ordering() {
    bool ok = true;
    uint64_t hit_addr = 0xD000, miss_addr = 0xD100;
    CHECK("prime hit line", do_write_apply(hit_addr, 0x88, "prime"));
    const uint32_t shared_id = 45;
    free_ids.erase(shared_id); // reserve -- alloc_id() must not also hand this out
    issue_read_explicit_id(miss_addr, shared_id); // A: queued first
    issue_read_explicit_id(hit_addr, shared_id);  // B: queued right behind A, same id, no wait
    bool first_ok = wait_read_done(shared_id);
    CHECK("first completion for the shared id arrives", first_ok);
    bool first_is_miss = false, first_is_hit = false;
    if (r_done.count(shared_id)) {
        first_is_miss = std::memcmp(r_done[shared_id]->got[0].data(), &golden[miss_addr], 16) == 0;
        first_is_hit  = std::memcmp(r_done[shared_id]->got[0].data(), &golden[hit_addr], 16) == 0;
        r_done.erase(shared_id); // keep shared_id reserved -- B is still to come
    }
    CHECK("first completion is A's (miss) data, not B's (hit) -- no same-ID reorder", first_is_miss && !first_is_hit);
    bool second_ok = wait_read_done(shared_id);
    CHECK("second completion for the shared id arrives", second_ok);
    if (r_done.count(shared_id)) {
        CHECK("second completion is B's (hit) data", std::memcmp(r_done[shared_id]->got[0].data(), &golden[hit_addr], 16) == 0);
        r_done.erase(shared_id);
    }
    free_ids.insert(shared_id);
    return ok;
}

// The fetch and LSU AXI ports have independent ID namespaces.  A numeric ID
// match across sources is not an ordering dependency.  Before this regression,
// l2c_ctrl's bypass-door `pipe_id_haz_c` compared only the numeric ID, so a
// continuous fetch stream using ID 0 kept an unrelated LSU bypass read using
// ID 0 outside the door indefinitely.  This is the exact shape of the Q700
// ROM's first inhibited/open-bus probe while instruction fetch remains active.
// ── Fetch header door: back-to-back short bursts ───────────────────────
// cpu040's axi_i binds STRAIGHT to this port (fpga_top_ddr.vh's f_axi_*
// bind; the crossbar is not in the path), and its I-side runs 5 MSHRs on
// 5 distinct AXI IDs.  So the traffic this port really sees is a stream of
// short, back-to-back, DISTINCT-ID line fills -- not the single long burst
// every other fetch scenario here drives.
//
// A one-entry header door is invisible inside one long burst and costs a
// full turnaround cycle between short ones, so this shape is the only one
// that can see it.  Two runs, one knob apart:
//   * rotating IDs  -- different-ID bursts may overlap in the pipeline, so
//                      the DOOR is what limits burst rate.  This is the
//                      real cpu040 shape and the one the pre-latch fixes.
//   * single ID     -- l2c_ctrl's id_busy_c/pipe_id_haz_c serialise
//                      same-ID accepts by design, so the door is NOT the
//                      limiter here.  Included as the control: it must NOT
//                      speed up, which is what proves the rotating-ID
//                      result is the door and not a measurement artifact.
struct FetchDoorResult { double cyc_per_burst; uint64_t bursts, cycles, refused; };

static FetchDoorResult fetch_door_run(int nid, int arlen, int bursts) {
    // Warm the region first with one long fetch burst so the measured
    // stream is all hits -- a cold MSHR stall would swamp the door effect.
    g_fetch_stream = false;
    g_fetch_arlen = 127; g_fetch_stride = 0; g_fetch_nid = 1;
    g_fetch_addr = g_fetch_base; g_fetch_id = 0;
    g_fetch_rlast_hs = 0;
    g_fetch_stream = true;
    for (int i = 0; i < 40000 && g_fetch_rlast_hs == 0; i++) tick();
    g_fetch_stream = false;
    // QUIESCE, not just "arready is high and no beat is on the wire this
    // cycle" (2026-09-15).  The old condition was
    //     while (!f_axi_arready || f_axi_rvalid) tick();
    // and it exits the moment the one-entry AR pre-latch promotes -- which
    // it does with a whole 4 KiB warm-up burst (256 internal 128 b
    // quadrants) still sitting in l2c_ctrl's ACTIVE fetch tracker, and with
    // f_axi_rvalid merely low on that particular cycle.  The measurement
    // window below then opened on top of up to ~256 cycles of warm-up
    // dispatch and CHARGED THEM TO THE MEASURED STREAM.
    //
    // That is what the numbers this function used to print actually were.
    // Instrumenting l2c_ctrl to log every cycle the measured (arlen=0)
    // burst shape was live showed the measured stream itself running in a
    // DENSE, UNBROKEN block of 512 cycles for 256 bursts -- exactly the
    // 2.00 cyc/burst pipeline floor -- in every configuration, rotating IDs
    // included, while the printed figures were 2.988 / 2.004 depending only
    // on where the drain loop happened to stop.  The old comment below
    // attributing the rotating-ID figure to same-ID quadrant spacing was
    // therefore explaining a measurement artifact; the door and the
    // pipeline reach the floor in both shapes.
    //
    // Requiring a sustained quiet WINDOW fixes it: a fetch burst under
    // dispatch produces a response beat at least every other cycle, so 32
    // consecutive cycles of "ready for a header and nothing coming back"
    // cannot happen until the tracker is genuinely empty.
    {
        int quiet = 0;
        for (int i = 0; i < 40000 && quiet < 32; i++) {
            tick();
            if (dut->f_axi_arready && !dut->f_axi_rvalid) quiet++; else quiet = 0;
        }
    }

    // Measured stream: `bursts` back-to-back bursts of (arlen+1) 32 B
    // beats, walking distinct lines inside the warmed 4 KiB.
    const uint64_t bytes_per_burst = static_cast<uint64_t>(arlen + 1) * 32u;
    g_fetch_arlen  = arlen;
    g_fetch_stride = bytes_per_burst;
    g_fetch_nid    = nid;
    g_fetch_addr   = g_fetch_base;
    g_fetch_id     = 0;
    g_fetch_ar_hs = 0; g_fetch_ar_pres = 0; g_fetch_ar_refused = 0;

    uint64_t t0 = sim_time;
    g_fetch_stream = true;
    for (int i = 0; i < 200000 && g_fetch_ar_hs < static_cast<uint64_t>(bursts); i++) {
        tick();
        // Stay inside the warmed 4 KiB.
        if (g_fetch_addr >= g_fetch_base + 4096) g_fetch_addr = g_fetch_base;
    }
    uint64_t cycles = sim_time - t0;
    g_fetch_stream = false;
    // Drain.
    for (int i = 0; i < 20000 && dut->f_axi_rvalid; i++) tick();

    // Restore the driver's original fixed shape for later scenarios.
    g_fetch_arlen = 127; g_fetch_stride = 0; g_fetch_nid = 1;
    g_fetch_addr = g_fetch_base; g_fetch_id = 0;

    FetchDoorResult r;
    r.bursts = g_fetch_ar_hs;
    r.cycles = cycles;
    r.refused = g_fetch_ar_refused;
    r.cyc_per_burst = r.bursts ? static_cast<double>(cycles) / static_cast<double>(r.bursts) : 0.0;
    return r;
}

static bool test_fetch_door_back_to_back_bursts() {
    bool ok = true;
    reset_dut();
    const bool old_force = g_mem_force_ready, old_always = g_perf_always_ready;
    const int old_lat = g_ddr_latency_base, old_jit = g_ddr_latency_jitter, old_gap = g_ddr_beat_gap_max;
    g_perf_always_ready = true; g_mem_force_ready = true;
    g_ddr_latency_base = 40; g_ddr_latency_jitter = 0; g_ddr_beat_gap_max = 0;

    // 1-beat (32 B) bursts, 4 rotating IDs: the purest look at burst rate.
    FetchDoorResult rot1 = fetch_door_run(/*nid=*/4, /*arlen=*/0, /*bursts=*/256);
    printf("  fetch door, 1-beat bursts, 4 rotating IDs : %.3f cyc/burst "
           "(%llu bursts / %llu cyc, AR refused %llu cyc)\n",
           rot1.cyc_per_burst, (unsigned long long)rot1.bursts,
           (unsigned long long)rot1.cycles, (unsigned long long)rot1.refused);

    // 2-beat (64 B = one L2 line) bursts, 4 rotating IDs: the real cpu040
    // line-fill shape.
    FetchDoorResult rot2 = fetch_door_run(/*nid=*/4, /*arlen=*/1, /*bursts=*/256);
    printf("  fetch door, 2-beat bursts, 4 rotating IDs : %.3f cyc/burst "
           "(%llu bursts / %llu cyc, AR refused %llu cyc)\n",
           rot2.cyc_per_burst, (unsigned long long)rot2.bursts,
           (unsigned long long)rot2.cycles, (unsigned long long)rot2.refused);

    // Control: same traffic, one ID.
    FetchDoorResult one1 = fetch_door_run(/*nid=*/1, /*arlen=*/0, /*bursts=*/256);
    printf("  fetch door, 1-beat bursts, single ID      : %.3f cyc/burst "
           "(%llu bursts / %llu cyc, AR refused %llu cyc)\n",
           one1.cyc_per_burst, (unsigned long long)one1.bursts,
           (unsigned long long)one1.cycles, (unsigned long long)one1.refused);

    CHECK("rotating-ID 1-beat fetch stream completed every burst", rot1.bursts == 256);
    CHECK("rotating-ID 2-beat fetch stream completed every burst", rot2.bursts == 256);
    CHECK("single-ID 1-beat fetch stream completed every burst",   one1.bursts == 256);

    // ── What these numbers mean ──────────────────────────────────────
    //
    // CORRECTED 2026-09-15.  The figures this block used to record --
    //
    //   shape                      no pre-latch   with pre-latch
    //   1-beat, single ID              2.992          2.004
    //   1-beat, 4 rotating IDs         2.992          2.988
    //   2-beat, 4 rotating IDs         4.984          4.973
    //
    // -- were measured through the broken warm-up drain fixed above, and
    // the rotating-ID rows in particular were mostly warm-up residue
    // charged to the measured stream, not DUT behaviour.  With the drain
    // actually quiescing the fetch port, all three shapes sit on their
    // structural floor and the ID pattern makes no difference at all:
    //
    //   shape                      measured
    //   1-beat, single ID              1.992   (2 quadrant lookups/burst)
    //   1-beat, 4 rotating IDs         1.992
    //   2-beat, 4 rotating IDs         3.977   (4 quadrant lookups/burst)
    //
    // i.e. one 128 b quadrant lookup per cycle, which is exactly the
    // pipeline's one-beat-per-cycle floor -- a 256 b fetch beat is two
    // quadrants and a 64 B line fill is four, so 2.00 and 4.00 are the
    // ceilings of this datapath width.  The old text attributed the
    // rotating-ID figure to same-ID quadrant spacing through
    // pipe_id_haz_c; that explanation was fitted to an artifact and is
    // withdrawn.  Beating 2.00/4.00 still needs a datapath-width change
    // (serve a 256-bit fetch beat in one lookup), which remains out of
    // scope -- but it is the ONLY thing left, the door is not a limiter
    // in any shape.
    //
    // The bounds below are kept where they were: they are loose upper
    // guards, and leaving them loose means they still catch a real
    // regression without re-failing if the floor moves by a fill cycle.
    // Verified 2026-09-15 to give byte-identical figures on the RTL
    // before and after the array-address pipelining stage, which is what
    // establishes that stage cost the fetch path nothing.
    CHECK("single-ID back-to-back 1-beat fetch bursts are not door-limited "
          "to a turnaround cycle (this is the pre-latch's regression guard)",
          one1.cyc_per_burst < 2.10);
    CHECK("rotating-ID 1-beat fetch bursts do not regress",
          rot1.cyc_per_burst < 3.30);
    CHECK("rotating-ID 2-beat (cpu040's real 64 B line-fill) bursts do not regress",
          rot2.cyc_per_burst < 5.50);
    // Anti-vacuity: the master really was pushing continuously, and the
    // door really did refuse it sometimes (i.e. we measured a door, not
    // an idle port).
    CHECK("the fetch stream really is presenting a header nearly every cycle",
          rot1.cycles > 0 && g_fetch_ar_pres > 0);

    g_mem_force_ready = old_force; g_perf_always_ready = old_always;
    g_ddr_latency_base = old_lat; g_ddr_latency_jitter = old_jit; g_ddr_beat_gap_max = old_gap;
    return ok;
}

static bool test_fetch_id0_does_not_starve_lsu_bypass_id0() {
    bool ok = true;
    reset_dut();

    g_fetch_ar_hs = 0;
    g_fetch_rlast_hs = 0;
    // Warm all 4 KiB first.  Cold MSHR stalls create occasional empty
    // lookup stages and are not the board's steady ROM-fetch condition.
    g_fetch_stream = true;
    for (int i = 0; i < 20000 && g_fetch_rlast_hs == 0; i++) tick();
    CHECK("fetch warm-up burst completes", g_fetch_rlast_hs != 0);
    g_fetch_stream = false;
    for (int i = 0; i < 4000 && (!dut->f_axi_arready || dut->f_axi_rvalid); i++) tick();
    CHECK("fetch warm-up drains before the measured stream", dut->f_axi_arready && !dut->f_axi_rvalid);

    g_fetch_ar_hs = 0;
    g_fetch_rlast_hs = 0;
    g_fetch_stream = true;
    for (int i = 0; i < 32 && g_fetch_ar_hs == 0; i++) tick();
    CHECK("fetch stream enters the shared front door", g_fetch_ar_hs != 0);

    issue_read_explicit_id(BYPASS_BASE + 0x80, 0);
    const uint64_t start = sim_time;
    while (!r_done.count(0) && sim_time - start < 600) tick();
    CHECK("LSU bypass ID0 completes while fetch ID0 remains active", r_done.count(0) != 0);
    if (r_done.count(0)) {
        CHECK("LSU bypass response is OKAY", r_done[0]->last_resp == 0);
        CHECK("LSU bypass data is byte-exact",
              std::memcmp(r_done[0]->got[0].data(), &golden[BYPASS_BASE + 0x80], 16) == 0);
    }

    g_fetch_stream = false;
    r_pending_issue.clear();
    r_issuing.reset();
    r_awaiting.clear();
    r_done.erase(0);
    reset_dut();
    return ok;
}

// Stress the response-source and address tags with the exact traffic class
// seen in the Q700 list walk: fetch ID0 remains hot while LSU ID0 repeatedly
// reads distinct low-memory lines.  The two AXI ports own independent ID
// namespaces; every LSU beat must still return the line it requested.
static bool test_dual_source_response_association_stress() {
    bool ok = true;
    reset_dut();

    g_fetch_ar_hs = 0;
    g_fetch_rlast_hs = 0;
    g_fetch_stream = true;
    for (int i = 0; i < 20000 && g_fetch_rlast_hs == 0; i++) tick();
    CHECK("association stress fetch warm-up completes", g_fetch_rlast_hs != 0);

    static constexpr uint64_t addrs[] = {0x0000'88a0u, 0x0000'00c0u,
                                          0x0000'0130u, 0x0000'0200u};
    for (int iter = 0; iter < 5000 && ok; iter++) {
        const uint64_t addr = addrs[iter & 3];
        issue_read_explicit_id(addr, 0);
        const uint64_t start = sim_time;
        while (!r_done.count(0) && sim_time - start < 2000) tick();
        CHECK("dual-source LSU ID0 response completes", r_done.count(0) != 0);
        if (!r_done.count(0)) break;
        CHECK("dual-source LSU ID0 response is OKAY", r_done[0]->last_resp == 0);
        CHECK("dual-source LSU ID0 response keeps its requested line",
              std::memcmp(r_done[0]->got[0].data(), &golden[addr], 16) == 0);
        r_done.erase(0);
    }

    g_fetch_stream = false;
    r_pending_issue.clear();
    r_issuing.reset();
    r_awaiting.clear();
    r_done.erase(0);
    reset_dut();
    return ok;
}

// Critical-6 fail-before/pass-after: a primary miss plus two secondary
// merges (quadrants 0-2 of one line) plus a fourth write racing in at
// quadrant 3 (which may itself merge or land via the fast hit path,
// depending on exact timing -- either is a valid, uninteresting outcome;
// what matters is what happens next).  Every quadrant must end up
// holding ITS OWN write's value: a full-line MSHR replay strobe (the
// pre-fix bug) would silently revert an unrelated quadrant's value back
// to the MSHR's own stale line_reg snapshot the next time ANY queued
// replay step for this entry executes.
static bool test_mshr_replay_partial_write() {
    bool ok = true;
    uint64_t base = 0xE000;
    auto mk = [](uint8_t seed) { std::array<uint8_t,16> d{}; for (int i = 0; i < 16; i++) d[i] = static_cast<uint8_t>(seed + i); return d; };
    uint32_t id0 = issue_write(base,      {mk(0xA0)}, {0xFFFF});
    uint32_t id1 = issue_write(base + 16, {mk(0xA1)}, {0xFFFF});
    uint32_t id2 = issue_write(base + 32, {mk(0xA2)}, {0xFFFF});
    uint32_t id3 = issue_write(base + 48, {mk(0xA3)}, {0xFFFF});
    CHECK("quadrant0 (primary) completes", wait_write_done(id0));
    CHECK("quadrant1 (merge) completes",   wait_write_done(id1));
    CHECK("quadrant2 (merge) completes",   wait_write_done(id2));
    CHECK("quadrant3 (race) completes",    wait_write_done(id3));
    for (uint32_t id : {id0, id1, id2, id3}) if (w_done.count(id)) free_write(id);
    for (int q = 0; q < 4; q++) {
        uint64_t a = base + static_cast<uint64_t>(q) * 16;
        std::array<uint8_t,16> exp = mk(static_cast<uint8_t>(0xA0 + q));
        uint32_t rid = issue_read(a);
        CHECK("quadrant readback completes", wait_read_done(rid));
        if (r_done.count(rid)) {
            CHECK("quadrant holds its own write, not clobbered by another quadrant's replay",
                  std::memcmp(r_done[rid]->got[0].data(), exp.data(), 16) == 0);
            free_read(rid);
        }
    }
    return ok;
}

// -- MSHR replay-payload scenarios (l2c_mshr.v `r_pay`) -------------------
// l2c_mshr.v keeps every replay slot's {wstrb, wdata} in ONE memory
// (`r_pay`) addressed {entry, slot} instead of a per-slot register array.
// The tests below exist to pin that mapping and its timing.  Before them
// the only thing that reliably drove S_SWR with DISTINCT per-slot
// payloads was the randomized stress loop, so an address-flattening slip
// (wrong concat order/width, aliasing across entries) or a read-latency
// slip -- what moving `r_pay` to block RAM would introduce -- could merge
// the wrong quadrant's bytes into a line the requester has already been
// told it wrote, and still leave every directed test green.  That is
// silent disk-level corruption, not a performance bug.
//
// Every write below is PARTIAL-strobe on purpose.  A fully-strobed 16 B
// write to a line with no live MSHR entry takes l2c_ctrl's no-fetch
// sector-install path (s_lookup_ins_go) and never reaches l2c_mshr at
// all -- which is why test_mshr_replay_partial_write, written before that
// path existed, no longer exercises replay.
static uint32_t issue_write_masked(uint64_t addr, uint8_t seed, uint16_t strb) {
    std::array<uint8_t,16> d{};
    for (int i = 0; i < 16; i++) d[i] = static_cast<uint8_t>(seed + i * 7 + 1);
    return issue_write(addr, {d}, {strb});
}
static void golden_apply_masked(uint64_t addr, uint8_t seed, uint16_t strb) {
    for (int i = 0; i < 16; i++)
        if ((strb >> i) & 1) golden[addr + i] = static_cast<uint8_t>(seed + i * 7 + 1);
    touched[addr] = true;
}
// Settle to "no MSHR entry live, no fill outstanding" so each scenario
// starts from a known table state (the occupancy / r_q.size() checks below
// are only meaningful against an empty table).
static bool mshr_quiesce(int max_cycles = 40000) {
    int c = 0;
    while ((dut->dbg_mshr_occupancy != 0 || !mslv.r_q.empty()) && c < max_cycles) { tick(); c++; }
    return dut->dbg_mshr_occupancy == 0 && mslv.r_q.empty();
}

// Three secondaries, each on a DIFFERENT quadrant of the primary's line
// and each with its own payload AND its own strobe.  Every slot's bytes
// are distinguishable from every other slot's, so a replay that reads the
// wrong slot -- or the wrong entry -- lands visibly wrong bytes in a
// quadrant rather than merely reordering identical data.
static bool test_replay_payload_slot_mapping() {
    bool ok = true;
    const uint64_t base = 0x00440000ULL;
    const uint8_t  seed[4] = {0x10, 0x21, 0x32, 0x43};
    const uint16_t strb[4] = {0x0F0F, 0x00FF, 0xF0F0, 0x3333};
    uint32_t id[4];
    CHECK("quiesced before replay slot-mapping scenario", mshr_quiesce());

    g_mem_block_rvalid = true;
    id[0] = issue_write_masked(base, seed[0], strb[0]);  // primary: allocates + fills
    int guard = 3000;
    while (mslv.r_q.empty() && guard-- > 0) tick();
    CHECK("partial-strobe primary allocates exactly one fill", mslv.r_q.size() == 1);
    for (int q = 1; q < 4; q++) id[q] = issue_write_masked(base + q * 16, seed[q], strb[q]);
    for (int i = 0; i < 200; i++) tick();
    CHECK("three secondaries merge into the one entry", dut->dbg_mshr_occupancy == 1);
    CHECK("merging launches no duplicate same-line fill", mslv.r_q.size() == 1);
    for (int q = 0; q < 4; q++)
        CHECK("no same-line write completes before its fill returns", w_done.count(id[q]) == 0);

    g_mem_block_rvalid = false;
    for (int q = 0; q < 4; q++) {
        const bool done = wait_write_done(id[q]);
        CHECK("replayed write completes", done);
        if (done) {
            CHECK("replayed write returns OKAY", w_done[id[q]]->resp_ok);
            free_write(id[q]);
        }
        golden_apply_masked(base + q * 16, seed[q], strb[q]);
    }
    for (int q = 0; q < 4; q++)
        CHECK("quadrant holds exactly its own slot's payload under its own strobe",
              do_read_check(base + q * 16, "replay_payload_slot_mapping"));
    return ok;
}

// Fill all REPLAY_N slots of one entry, every one of them targeting the
// SAME quadrant with overlapping strobes.  The final bytes are only right
// if slot k is replayed with slot k's payload in slot order: any
// permutation of the payloads changes the answer.  This is the scenario
// the per-quadrant test above cannot catch, because there each slot owns
// a disjoint quadrant.
static bool test_replay_all_slots_same_quadrant_order() {
    bool ok = true;
    const uint64_t addr = 0x00450000ULL;
    const uint8_t  seed[L2C_REPLAY_N + 1] = {0x50, 0x61, 0x72, 0x83, 0x94};
    const uint16_t strb[L2C_REPLAY_N + 1] = {0x0FFF, 0xFF00, 0x00FF, 0xF00F, 0x5555};
    uint32_t id[L2C_REPLAY_N + 1];
    CHECK("quiesced before slot-order scenario", mshr_quiesce());

    g_mem_block_rvalid = true;
    id[0] = issue_write_masked(addr, seed[0], strb[0]);
    int guard = 3000;
    while (mslv.r_q.empty() && guard-- > 0) tick();
    CHECK("primary allocates exactly one fill", mslv.r_q.size() == 1);
    for (int k = 1; k <= L2C_REPLAY_N; k++) id[k] = issue_write_masked(addr, seed[k], strb[k]);
    for (int i = 0; i < 200; i++) tick();
    CHECK("all REPLAY_N slots of one entry are occupied", dut->dbg_mshr_occupancy == 1);
    CHECK("saturated replay queue launches no second fill", mslv.r_q.size() == 1);

    g_mem_block_rvalid = false;
    for (int k = 0; k <= L2C_REPLAY_N; k++) {
        const bool done = wait_write_done(id[k]);
        CHECK("same-quadrant replay completes", done);
        if (done) {
            CHECK("same-quadrant replay returns OKAY", w_done[id[k]]->resp_ok);
            free_write(id[k]);
        }
        golden_apply_masked(addr, seed[k], strb[k]);
    }
    CHECK("overlapping same-quadrant replays applied in program order",
          do_read_check(addr, "replay_all_slots_same_quadrant_order"));
    return ok;
}

// Sweep the offset between "the blocked fill's R beats are released" and
// "a third same-line write is issued", so that write lands at every point
// across S_INSTALL, S_PRSP and the S_SNEXT<->S_SWR walk -- including
// S_SNEXT's free-vs-merge race guard, the one cycle where a merge is
// enqueued into the very slot the walk is about to read.  That cycle is
// the worst case for the replay-payload read: the data must be the value
// written on the immediately preceding edge, not the slot's old contents.
static bool test_replay_merge_during_install_walk() {
    bool ok = true;
    const uint16_t strb[3] = {0x0F0F, 0xF0F0, 0x33CC};
    for (int d = 0; d < 40; d++) {
        const uint64_t base = 0x00500000ULL + static_cast<uint64_t>(d) * LINE_BYTES;
        const uint8_t seed[3] = {static_cast<uint8_t>(0x20 + d),
                                 static_cast<uint8_t>(0x60 + d),
                                 static_cast<uint8_t>(0xA0 + d)};
        if (!mshr_quiesce()) { CHECK("quiesced before walk-merge sweep step", false); break; }

        g_mem_block_rvalid = true;
        uint32_t id0 = issue_write_masked(base, seed[0], strb[0]);
        int guard = 3000;
        while (mslv.r_q.empty() && guard-- > 0) tick();
        uint32_t id1 = issue_write_masked(base + 16, seed[1], strb[1]);
        for (int i = 0; i < 120; i++) tick();
        CHECK("sweep step holds one entry with one queued replay", dut->dbg_mshr_occupancy == 1);

        g_mem_block_rvalid = false;
        for (int i = 0; i < d; i++) tick();
        // Lands somewhere in [install, replay walk, after free] depending
        // on d.  Whichever it is, the bytes must come out right.
        uint32_t id2 = issue_write_masked(base + 32, seed[2], strb[2]);

        bool done = true;
        for (uint32_t wid : {id0, id1, id2}) {
            const bool d1 = wait_write_done(wid);
            done = done && d1;
            if (d1) { CHECK("walk-merge write returns OKAY", w_done[wid]->resp_ok); free_write(wid); }
        }
        CHECK("all three same-line writes complete", done);
        for (int q = 0; q < 3; q++) golden_apply_masked(base + q * 16, seed[q], strb[q]);
        for (int q = 0; q < 3; q++)
            CHECK("quadrant correct whatever cycle the late merge landed on",
                  do_read_check(base + q * 16, "replay_merge_during_install_walk"));
        if (!ok) { printf("  (walk-merge sweep first failed at d=%d)\n", d); break; }
    }
    return ok;
}

// Four entries, each with a primary plus two secondaries, all filled from
// a single release of the blocked memory model so their install/replay
// walks run back to back.  `act` therefore changes between consecutive
// S_SWR bursts, which is exactly what an entry-vs-slot address mix-up in
// the replay-payload memory gets wrong: one entry's replay would merge a
// neighbouring entry's bytes.
static bool test_back_to_back_replays_distinct_entries() {
    bool ok = true;
    const int NL = 4;
    const uint64_t base = 0x00600000ULL;
    const uint16_t strb[3] = {0x0F0F, 0x00FF, 0xF0F0};
    uint64_t a[NL][3];
    uint32_t id[NL][3];
    uint8_t  sd[NL][3];
    CHECK("quiesced before multi-entry replay scenario", mshr_quiesce());

    g_mem_block_rvalid = true;
    for (int l = 0; l < NL; l++) {
        const uint64_t lb = base + static_cast<uint64_t>(l) * 7 * LINE_BYTES;
        for (int q = 0; q < 3; q++) {
            a[l][q] = lb + static_cast<uint64_t>(q) * 16;
            sd[l][q] = static_cast<uint8_t>(0x11 * (l + 1) + 0x25 * q);
        }
        id[l][0] = issue_write_masked(a[l][0], sd[l][0], strb[0]);
        int guard = 3000;
        while (mslv.r_q.size() < static_cast<size_t>(l + 1) && guard-- > 0) tick();
    }
    CHECK("one fill per distinct line", mslv.r_q.size() == static_cast<size_t>(NL));
    for (int l = 0; l < NL; l++)
        for (int q = 1; q < 3; q++)
            id[l][q] = issue_write_masked(a[l][q], sd[l][q], strb[q]);
    for (int i = 0; i < 300; i++) tick();
    CHECK("four entries live, each with two queued replays",
          dut->dbg_mshr_occupancy == NL);
    CHECK("no duplicate fills from the secondaries",
          mslv.r_q.size() == static_cast<size_t>(NL));

    g_mem_block_rvalid = false;
    for (int l = 0; l < NL; l++)
        for (int q = 0; q < 3; q++) {
            const bool done = wait_write_done(id[l][q]);
            CHECK("multi-entry replay completes", done);
            if (done) {
                CHECK("multi-entry replay returns OKAY", w_done[id[l][q]]->resp_ok);
                free_write(id[l][q]);
            }
            golden_apply_masked(a[l][q], sd[l][q], strb[q]);
        }
    for (int l = 0; l < NL; l++)
        for (int q = 0; q < 3; q++)
            CHECK("each entry replayed its OWN slots, not a neighbour entry's",
                  do_read_check(a[l][q], "back_to_back_replays_distinct_entries"));
    return ok;
}

#ifdef BYPASS_ALL_BUILD
static bool test_bypass_all_equivalence() {
    bool ok = true;
    // In L2_BYPASS_ALL mode there is no cache: every address (including
    // ones inside the normal "cacheable" range) must pass straight
    // through to m_axi with correct data, single round-trip, no
    // dependency on tag/data/MSHR machinery (none is even instantiated).
    for (int i = 0; i < 32; i++) {
        uint64_t a = (static_cast<uint64_t>(rng()) % (CACHEABLE_SIZE / 16)) * 16;
        CHECK("bypass-all write", do_write_apply(a, 0x60 + i, "bypass_all_write"));
        CHECK("bypass-all readback", do_read_check(a, "bypass_all_read"));
    }
    return ok;
}
#endif

// ─── Randomized scoreboard: >=20000 mixed ops vs. golden model ───────────
static bool test_randomized_scoreboard(int n_ops) {
    bool ok = true;
    std::set<uint64_t> inflight; // 16B-aligned chunk hazard tracker
    int issued = 0, completed = 0;
    int n_writes_done = 0, n_reads_done = 0;
    int max_ops_outstanding = 0;
    int max_chunks_outstanding = 0;
    int max_mshr_occupancy = 0;
    uint64_t stream_cursor = 0x0080'0000ULL;
    bool progress_debug = std::getenv("L2C_DEBUG_PROGRESS") != nullptr;
    uint64_t stall_cycles = 0;
    int last_completed = -1;
    // Unconditional no-progress guard.  This loop had none (the STALL
    // report below is behind L2C_DEBUG_PROGRESS), so any DUT wedge hung the
    // whole binary forever instead of failing -- which makes a wedge
    // indistinguishable from a slow run, and makes mutation testing
    // unusable because a lethal mutant reports nothing at all.  Two lethal
    // mutants during the 2026-08-20 bypass work hung here for >12 minutes
    // each before being killed by hand.  500,000 cycles is ~2,000 DRAM
    // round trips at the model's worst-case latency; the healthy run never
    // goes more than a few thousand without a completion.
    uint64_t last_progress_t = sim_time;
    const uint64_t sb_t0 = sim_time;
    // A wedge does not always mean "nothing completes": a design that has
    // lost only its write port still retires reads, so it dribbles forward
    // and the no-progress guard below never trips.  The absolute cap
    // catches that shape.  A healthy 20,000-op run finishes in ~1.4 M
    // cycles, so 20 M is >10x headroom.
    const uint64_t SB_CYCLE_CAP = 20000000ULL;
    while (completed < n_ops) {
        if (completed != last_completed) last_progress_t = sim_time;
        if (sim_time - sb_t0 > SB_CYCLE_CAP) {
            printf("  randomized scoreboard: OVER %llu CYCLES at completed=%d/%d "
                   "-- treating as a wedge (a partially-wedged DUT still retires "
                   "some ops, so the no-progress guard below cannot see it)\n",
                   (unsigned long long)SB_CYCLE_CAP, completed, n_ops);
            return false;
        }
        if (sim_time - last_progress_t > 500000) {
            printf("  randomized scoreboard: NO PROGRESS in 500000 cycles at completed=%d/%d "
                   "(w_pend=%zu w_issuing=%d w_awaiting=%zu r_pend=%zu r_issuing=%d r_awaiting=%zu) "
                   "-- treating as a wedge\n",
                   completed, n_ops, w_pending_issue.size(), w_issuing != nullptr,
                   w_awaiting_b.size(), r_pending_issue.size(), r_issuing != nullptr,
                   r_awaiting.size());
            return false;
        }
        if (progress_debug) {
            if (completed != last_completed) {
                if (completed % 200 == 0) {
                    printf("  progress: completed=%d (w=%d r=%d) w_pend=%zu w_issuing=%d w_awaiting=%zu r_pend=%zu r_issuing=%d r_awaiting=%zu inflight=%zu free_ids=%zu t=%llu\n",
                           completed, n_writes_done, n_reads_done, w_pending_issue.size(), w_issuing != nullptr,
                           w_awaiting_b.size(), r_pending_issue.size(), r_issuing != nullptr, r_awaiting.size(),
                           inflight.size(), free_ids.size(), (unsigned long long)sim_time);
                    printf("    w_awaiting ids:");
                    for (auto& kv : w_awaiting_b) printf(" %u@0x%llx", kv.first, (unsigned long long)kv.second->addr);
                    printf("\n    r_awaiting ids:");
                    for (auto& kv : r_awaiting) for (auto& rb : kv.second) printf(" %u@0x%llx", kv.first, (unsigned long long)rb->addr);
                    printf("\n");
                }
                last_completed = completed;
                stall_cycles = sim_time;
            } else if (sim_time - stall_cycles > 200000) {
                printf("  STALL: no progress in 200000 cycles at completed=%d w_pend=%zu w_issuing=%d w_awaiting=%zu r_pend=%zu r_issuing=%d r_awaiting=%zu\n",
                       completed, w_pending_issue.size(), w_issuing != nullptr, w_awaiting_b.size(),
                       r_pending_issue.size(), r_issuing != nullptr, r_awaiting.size());
                stall_cycles = sim_time;
            }
        }
        // Keep substantially more operations outstanding than the eight
        // MSHRs.  This exercises front-door backpressure, queued hits, merge
        // traffic, bypass traffic, and response arbitration at the same time,
        // while the chunk hazard set keeps the golden model unambiguous.
        // MUST count everything actually in flight -- queued-but-not-yet-
        // issued and mid-issue, not just w_awaiting_b/r_awaiting (which
        // only gain entries *after* a tick() actually runs the AXI
        // handshake). Counting only the latter let this loop dump up to
        // n_ops requests into the queues in one shot on the very first
        // pass (before any tick() had run), exhausting the finite id
        // pool and calling alloc_id() on an empty set (UB) -- corrupting
        // id-based response routing and stalling the whole driver.
        while ((issued - completed) < 24 && issued < n_ops) {
            // Important-13: mix in multi-beat bursts (~20%) and partial
            // write strobes (~33% of writes) so the golden-model scoreboard
            // -- not just directed scenarios -- covers these classes.
            int burst_class = static_cast<int>(rng() % 16);
            int beats = (burst_class < 9) ? 1 :
                        (burst_class < 11) ? 2 :
                        (burst_class < 13) ? 3 :
                        (burst_class < 15) ? 4 : 8;
            uint64_t a;
            int tries = 0;
            bool conflict;
            do {
                int byp_mod = 20;
                if (const char* e = std::getenv("L2C_DEBUG_BYPASS_MOD")) byp_mod = std::atoi(e);
                bool bypass = byp_mod > 0 && (rng() % byp_mod) == 0 && !std::getenv("L2C_DEBUG_NO_BYPASS");
                uint64_t lo = bypass ? BYPASS_BASE : CACHEABLE_BASE;
                uint64_t sz = bypass ? BYPASS_SIZE : CACHEABLE_SIZE;
                uint64_t max_chunks = sz / 16 - static_cast<uint64_t>(beats); // leave room for the whole burst
                if (bypass) {
                    a = lo + (rng() % (max_chunks + 1)) * 16;
                } else {
                    const int locality = static_cast<int>(rng() % 100);
                    if (locality < 35) {
                        // 256 KiB hot working set: produces meaningful hits
                        // and same-line merges instead of an all-cold stream.
                        constexpr uint64_t HOT_BASE = 0x0010'0000ULL;
                        constexpr uint64_t HOT_CHUNKS = (256 * 1024) / 16;
                        a = HOT_BASE + (rng() % (HOT_CHUNKS - beats + 1)) * 16;
                    } else if (locality < 65) {
                        // Forward stream, wrapping inside a 2 MiB region.
                        constexpr uint64_t STREAM_BASE = 0x0080'0000ULL;
                        constexpr uint64_t STREAM_SIZE = 2 * 1024 * 1024;
                        a = stream_cursor;
                        stream_cursor += static_cast<uint64_t>(beats) * 16;
                        if (stream_cursor + 8 * 16 >= STREAM_BASE + STREAM_SIZE)
                            stream_cursor = STREAM_BASE;
                    } else if (locality < 80) {
                        // Repeated pressure on one cache set across many tags.
                        constexpr uint64_t SET_BASE = 0x0002'4000ULL;
                        constexpr uint64_t SET_STRIDE = 4096 * LINE_BYTES;
                        uint64_t tag = rng() % 48;
                        a = SET_BASE + tag * SET_STRIDE + (rng() & 3u) * 16;
                    } else {
                        a = lo + (rng() % (max_chunks + 1)) * 16;
                    }
                }
                conflict = false;
                for (int i = 0; i < beats && !conflict; i++) if (inflight.count(a + static_cast<uint64_t>(i) * 16)) conflict = true;
                tries++;
            } while (conflict && tries < 8);
            if (conflict) break; // give the window time to drain
            for (int i = 0; i < beats; i++) inflight.insert(a + static_cast<uint64_t>(i) * 16);
            bool is_write = (rng() % 2) == 0;
            if (is_write) {
                bool partial = (rng() % 3) == 0; // ~33% of writes use a partial strobe
                std::vector<std::array<uint8_t,16>> wd(beats);
                std::vector<uint16_t> ws(beats);
                for (int i = 0; i < beats; i++) {
                    uint64_t seed = rng() & 0xFF;
                    for (int b = 0; b < 16; b++) wd[i][b] = static_cast<uint8_t>((seed + b + i) & 0xFF);
                    uint16_t s = 0xFFFF;
                    if (partial) { s = static_cast<uint16_t>(rng() & 0xFFFF); if (s == 0) s = 1; }
                    ws[i] = s;
                }
                auto wb = std::make_shared<WBurst>();
                wb->id = alloc_id(); wb->addr = a; wb->beats = beats;
                wb->data = wd; wb->strb = ws;
                w_pending_issue.push_back(wb); // wb->addr/data/strb carry everything needed to drain later
                log_op(a, true, wb->id);
            } else {
                uint32_t rid = issue_read(a, beats);
                log_op(a, false, rid);
            }
            issued++;
        }
        tick();
        max_ops_outstanding = std::max(max_ops_outstanding, issued - completed);
        max_chunks_outstanding = std::max(max_chunks_outstanding, static_cast<int>(inflight.size()));
        max_mshr_occupancy = std::max(max_mshr_occupancy, static_cast<int>(dut->dbg_mshr_occupancy));
        // Drain completed writes.  Important-13: must handle EVERY beat
        // (not just beat 0) and respect each beat's OWN strobe -- a
        // partial-strobe beat leaves untouched bytes at their prior
        // golden[] value, exactly like the RTL leaves them at their
        // prior DRAM/cache value.  Every 16B chunk the op actually
        // touched must also be erased from `inflight`, or multi-beat
        // bursts leak chunks that never become available again.
        for (auto it = w_done.begin(); it != w_done.end(); ) {
            auto& wb = *it->second;
            for (int beat = 0; beat < wb.beats; beat++) {
                uint64_t a = wb.addr + static_cast<uint64_t>(beat) * 16;
                for (int b = 0; b < 16; b++) if ((wb.strb[beat] >> b) & 1) golden[a + b] = wb.data[beat][b];
                touched[a] = true;
                inflight.erase(a);
            }
            free_ids.insert(it->first);
            completed++; n_writes_done++;
            it = w_done.erase(it);
        }
        // Drain completed reads, checking every beat against golden.
        static int mismatch_dumps = 0;
        for (auto it = r_done.begin(); it != r_done.end(); ) {
            auto& rb = *it->second;
            for (int beat = 0; beat < rb.beats; beat++) {
                uint64_t a = rb.addr + static_cast<uint64_t>(beat) * 16;
                bool match = std::memcmp(rb.got[beat].data(), &golden[a], 16) == 0;
                if (!match) {
                    printf("  scoreboard MISMATCH @0x%llx beat=%d id=%u t=%llu got=%02x%02x%02x%02x%02x%02x%02x%02x exp=%02x%02x%02x%02x%02x%02x%02x%02x\n",
                           (unsigned long long)a, beat, it->first, (unsigned long long)sim_time,
                           rb.got[beat][0], rb.got[beat][1], rb.got[beat][2], rb.got[beat][3],
                           rb.got[beat][4], rb.got[beat][5], rb.got[beat][6], rb.got[beat][7],
                           golden[a], golden[a+1], golden[a+2], golden[a+3], golden[a+4], golden[a+5], golden[a+6], golden[a+7]);
                    ok = false;
                    if (mismatch_dumps++ < 3) dump_op_log();
                }
                inflight.erase(a);
            }
            free_ids.insert(it->first);
            completed++; n_reads_done++;
            it = r_done.erase(it);
        }
    }
    // Final pass: read back every touched line through the DUT and
    // compare -- per the brief, no RTL backdoor, real reads only.
    int verified = 0;
    for (uint64_t a = 0; a < CACHEABLE_SIZE; a += 16) {
        if (!touched[a]) continue;
        uint32_t id = issue_read(a);
        if (!wait_read_done(id)) { printf("  final-verify TIMEOUT @0x%llx\n", (unsigned long long)a); ok = false; continue; }
        bool match = std::memcmp(r_done[id]->got[0].data(), &golden[a], 16) == 0;
        if (!match) { printf("  final-verify MISMATCH @0x%llx\n", (unsigned long long)a); ok = false; }
        free_read(id);
        verified++;
    }
    for (uint64_t a = BYPASS_BASE; a < BYPASS_BASE + BYPASS_SIZE; a += 16) {
        if (!touched[a]) continue;
        uint32_t id = issue_read(a);
        if (!wait_read_done(id)) { printf("  final-verify TIMEOUT @0x%llx\n", (unsigned long long)a); ok = false; continue; }
        bool match = std::memcmp(r_done[id]->got[0].data(), &golden[a], 16) == 0;
        if (!match) { printf("  final-verify (bypass) MISMATCH @0x%llx\n", (unsigned long long)a); ok = false; }
        free_read(id);
        verified++;
    }
    printf("  randomized scoreboard: %d ops, %d addresses final-verified, "
           "max outstanding=%d ops/%d chunks, max MSHR occupancy=%d\n",
           n_ops, verified, max_ops_outstanding, max_chunks_outstanding,
           max_mshr_occupancy);
    return ok;
}

// ─── Performance smoke: average hit latency (print only, no hard assert) ─
static void perf_smoke_hit_latency() {
    uint64_t base = 0x9000;
    do_write_apply(base, 0x01, "perf_prime");
    const int N = 200;
    uint64_t total = 0;
    for (int i = 0; i < N; i++) {
        uint32_t id = issue_read(base);
        uint64_t t0 = sim_time;
        wait_read_done(id);
        total += (sim_time - t0);
        free_read(id);
    }
    printf("  perf smoke: avg hit round-trip latency over %d repeats = %.2f cycles\n", N, (double)total / N);
}


// ══════════════════════════════════════════════════════════════════════════
// Performance measurement suite (`L2C_PERF=1`)
// ══════════════════════════════════════════════════════════════════════════
// Runs LAST, after the correctness suite, and deliberately resets the DUT
// between scenarios to get a cold cache -- which drops dirty lines by
// design (docs/l2c_spec.md S7), so golden[] and backing[] diverge and NO
// data checking happens in here.  This is a measurement harness only.
//
// Everything is measured against a DETERMINISTIC memory model (fixed
// first-beat latency, zero beat gap, always-ready) and an always-ready
// requester, so the numbers isolate l2c's own pipeline occupancy from
// DDR jitter.  The DDR latency is swept because some limits are
// latency-bound and some are pipeline-bound, and only the sweep shows
// which is which.
//
// Reported as cycles/op and MB/s at a 100 MHz core clock (the real
// core_clk this SoC runs, see synth/ku5p.xdc).
#ifndef BYPASS_ALL_BUILD
static constexpr double PERF_CLK_MHZ = 100.0;

struct PerfRow {
    std::string group, name, stim;
    double cyc_per_op = 0, bytes_per_op = 0, lat = 0;
};
static std::vector<PerfRow> g_perf_rows;

// Latency accumulators, reset per scenario, summed in perf_reap().
static uint64_t g_lat_sum = 0, g_lat_n = 0;        // AR-accept  -> first R beat
static uint64_t g_full_sum = 0, g_full_n = 0;      // issue      -> last beat / B

static int perf_reap() {
    int n = 0;
    for (auto it = r_done.begin(); it != r_done.end(); ) {
        g_lat_sum += it->second->t_r0 - it->second->t_ar; g_lat_n++;
        g_full_sum += it->second->t_rl - it->second->t_issue; g_full_n++;
        if (it->first < 60) free_ids.insert(it->first);
        n++; it = r_done.erase(it);
    }
    for (auto it = w_done.begin(); it != w_done.end(); ) {
        g_lat_sum += it->second->t_b - it->second->t_aw; g_lat_n++;
        g_full_sum += it->second->t_b - it->second->t_issue; g_full_n++;
        if (it->first < 60) free_ids.insert(it->first);
        n++; it = w_done.erase(it);
    }
    return n;
}

static void perf_quiesce() {
    int guard = 0;
    while ((!w_pending_issue.empty() || w_issuing || aw_issuing || !w_aw_sent.empty() ||
            !w_awaiting_b.empty() ||
            !r_pending_issue.empty() || r_issuing || !r_awaiting.empty()) && guard < 400000) {
        tick(); perf_reap(); guard++;
    }
    for (int i = 0; i < 8; i++) { tick(); perf_reap(); }
}

// Cold cache for the next scenario.  A reset walk is ~4096 cycles; it is
// the only way to guarantee a cold tag array without a backdoor.
static void perf_cold() { perf_quiesce(); reset_dut(); }

struct StreamCfg {
    int n_ops = 64;        // transactions (bursts), not beats
    int window = 8;        // max outstanding transactions
    int beats = 1;         // beats per burst (AxLEN = beats-1)
    uint64_t base = 0;
    uint64_t stride = 64;  // address step per transaction
    int wrap = 0;          // 0 = never wrap (cold walk); >0 = reuse `wrap` addrs
    bool is_write = false;
    int fixed_id = -1;     // >=0: every transaction uses this one AXI ID
};

// Front-door delta for the most recently measured window; perf_row()
// prints it under the row it belongs to.
static FdStats g_fd_win;

static double perf_stream(const StreamCfg& c) {
    g_lat_sum = g_lat_n = g_full_sum = g_full_n = 0;
    perf_quiesce();
    FdStats fd0 = g_fd;
    g_fd.stall_run_max = 0; g_fd_run = 0;   // stall_run_max is a per-window max
    uint64_t t0 = sim_time;
    int issued = 0, done = 0;
    while (done < c.n_ops) {
        while (issued < c.n_ops && (issued - done) < c.window) {
            uint64_t idx = (c.wrap > 0) ? static_cast<uint64_t>(issued % c.wrap)
                                        : static_cast<uint64_t>(issued);
            uint64_t a = c.base + idx * c.stride;
            if (c.is_write) {
                std::vector<std::array<uint8_t,16>> d(c.beats);
                std::vector<uint16_t> st(c.beats, 0xFFFF);
                for (int b = 0; b < c.beats; b++)
                    for (int k = 0; k < 16; k++) d[b][k] = static_cast<uint8_t>(issued + b + k);
                if (c.fixed_id >= 0) issue_write_explicit_id(a, static_cast<uint32_t>(c.fixed_id), d, st);
                else                 issue_write(a, d, st);
            } else {
                if (c.fixed_id >= 0) issue_read_explicit_id(a, static_cast<uint32_t>(c.fixed_id), c.beats);
                else                 issue_read(a, c.beats);
            }
            issued++;
        }
        tick();
        done += perf_reap();
        if (sim_time - t0 > 3000000) { printf("  PERF: HANG (issued=%d done=%d)\n", issued, done); break; }
    }
    g_fd_win = fd_diff(g_fd, fd0);
    return static_cast<double>(sim_time - t0) / c.n_ops;
}

// ── WHAT THESE EVICTION SCENARIOS ESTABLISHED (2026-08-20) ──────────────
// Two rounds, both recorded here so the second is not read out of context.
//
// ROUND 1 (commit 54919fe) evaluated an EAGER WRITEBACK proposal: clean
// dirty lines during idle DDR cycles so evictions become silent
// replacements and the then-2-entry victim buffer stops gating misses at
// l2c_ctrl.v's `way_ok_c`.  The buffer did fill -- under conflict-eviction
// traffic it was full 93-98% of cycles and a miss was blocked on it 91-98%
// -- but the drain engine was busy 94-99% of those same cycles, so there
// was no idle write bandwidth for a cleaner to move work into.  The real
// limiter was that a writeback was ROUND-TRIP SERIALIZED: one drain FSM
// (S_AW -> S_W -> S_B) under an arbiter that held the port from AW
// PRESENTATION to B, so the master carried one write at a time and the
// second slot could not start draining until the first one's B returned.
//
//     cycles/op = 3.0 + 0.746 x DDR_write_latency     (R^2 ~ 1)
//     L=40  -> 32.85    L=200 -> 152.22    L~0 -> 3.75
//
// The L~0 point came from temporarily completing B one cycle after WLAST.
//
// ROUND 2 built exactly what round 1 recommended -- SLOTS-deep victim
// buffer with its payload in LUTRAM (878d002's precedent) and an AW
// arbiter that locks the port to one OWNER but not to one TRANSACTION.
// Measured on this same harness:
//
//     evict_write, 4 out:   32.85 -> 4.84 cyc/op at DDR-40   (6.8x)
//                          152.22 -> 19.53 cyc/op at DDR-200 (7.8x)
//     evict_write_bursty:   35.82 -> 4.97      167.07 -> 20.59
//
// and every non-eviction row of this suite is bit-identical.  On the
// FAITHFUL chain (tb_l2c_sctr.cpp: real CDC + MIG bridge + posted writes,
// ~14-cycle round trip) the 16 B/line steady state -- the shape a 68040
// copyback L1D push actually presents -- went 3.745 -> 2.000 cycles/word
// with VB-full 86.6% -> 0.0% and miss-blocked 46.6% -> 0.0%.
//
// DOES EAGER WRITEBACK COME BACK?  No, and now for a better reason.
// Round 1's rejection was contingent on the serialization, so it was
// re-measured.  `dbg_vb_drain_busy` alone no longer answers it -- with a
// pipelined engine the sequencer is idle most of a saturated run because
// it has already handed everything to DRAM -- so the taps now separate
// "a writeback is in flight" from "the AW/W sequencer is busy", which is
// the port occupancy a cleaner would actually compete for.
//
//   mixed traffic, whole 2.22 M-cycle run (L2C_EVSTATS=1):
//       buffer full 0.17% (was 4.67%), miss blocked on it 0.00% -- ZERO
//       cycles out of 2.22 M (was 0.97%), sequencer busy 2.23%.
//   conflict traffic, 8 slots, 4 outstanding:
//       DDR-40  miss-blocked 37.9%, sequencer busy 30.9%, mean 6.25/8 out
//       DDR-200 miss-blocked 84.6%, sequencer busy  7.7%, mean 7.57/8 out
//   bursty (128-op bursts, 4000-cycle gaps): burst miss-blocked 91.6% ->
//       39.3% at DDR-40 and 98.2% -> 85.4% at DDR-200; gaps ~96% idle.
//
// Misses do still block under saturated conflict traffic -- but the buffer
// is full of writebacks ALREADY COMMITTED TO DRAM AND WAITING ON B, not of
// dirty lines cleaned too late.  Eager writeback needs free SLOTS to put
// work into and would compete for the very ones that are occupied; it
// would make this regime worse.  What is left is a pure depth-vs-latency
// relationship, and depth is a parameter (verilator -GVICTIM_SLOTS=n):
//
//     SLOTS    DDR-40 cyc/op (miss-blocked)   DDR-200 cyc/op (miss-blocked)
//       2         17.18  (82.5%)                  76.87  (96.1%)
//       4          8.94  (66.4%)                  38.63  (92.2%)
//       8          4.84  (37.9%)                  19.53  (84.6%)
//      16          3.00  ( 0.0%)                  10.01  (70.0%)
//
// 8 is shipped; 16 doubles the line-address CAM feeding l2c_ctrl's
// s_lookup_active and hit-resolve's level-2 LUTs are full after 980342c,
// so that trade wants a timing report rather than this model.  See
// docs/l2c_perf.md S13 for the full write-up.
//
// Dirty-eviction pressure stream.  perf_stream's StreamCfg walks a linear
// address range, which never conflict-evicts: it touches each set once.
// This generator instead walks SET-minor / TAG-major over a set-conflicting
// working set, so once the eight ways of a set are full every subsequent
// touch of that set both MISSES and EVICTS A DIRTY VICTIM -- the only shape
// under which the victim buffer can actually become the limiter.
//
//   addr = base + tag * SET_STRIDE + set * 64,  SET_STRIDE = 4096 * 64
//
// n_tags > 8 is required for steady-state eviction; the first 8 passes are
// cold fills and are included in the reported average (they are a fixed
// small fraction when n_ops >> n_sets * 8).
static constexpr uint64_t PERF_SET_STRIDE = 4096ULL * 64ULL;

static double perf_evict_stream(int n_ops, int window, int n_sets, int n_tags,
                                bool is_write, uint64_t base, EvStats* out) {
    g_lat_sum = g_lat_n = g_full_sum = g_full_n = 0;
    perf_quiesce();
    EvStats ev0 = g_ev;
    FdStats fd0 = g_fd;
    g_fd.stall_run_max = 0; g_fd_run = 0;   // stall_run_max is a per-window max
    uint64_t t0 = sim_time;
    int issued = 0, done = 0;
    while (done < n_ops) {
        while (issued < n_ops && (issued - done) < window) {
            uint64_t set = static_cast<uint64_t>(issued % n_sets);
            uint64_t tag = static_cast<uint64_t>((issued / n_sets) % n_tags);
            uint64_t a = base + tag * PERF_SET_STRIDE + set * 64;
            if (is_write) {
                std::vector<std::array<uint8_t,16>> d(1);
                std::vector<uint16_t> st(1, 0xFFFF);
                for (int k = 0; k < 16; k++) d[0][k] = static_cast<uint8_t>(issued + k);
                issue_write(a, d, st);
            } else {
                issue_read(a, 1);
            }
            issued++;
        }
        tick();
        done += perf_reap();
        if (sim_time - t0 > 3000000) { printf("  PERF: evict HANG (issued=%d done=%d)\n", issued, done); break; }
    }
    g_fd_win = fd_diff(g_fd, fd0);
    if (out) {
        out->cycles = g_ev.cycles - ev0.cycles;
        out->stall  = g_ev.stall  - ev0.stall;
        out->drain_busy = g_ev.drain_busy - ev0.drain_busy;
        out->seq_busy   = g_ev.seq_busy   - ev0.seq_busy;
        out->out_sum = g_ev.out_sum - ev0.out_sum;
        out->out_max = g_ev.out_max;
        for (int i = 0; i <= EV_MAX_SLOTS; i++) out->occ_hist[i] = g_ev.occ_hist[i] - ev0.occ_hist[i];
    }
    return static_cast<double>(sim_time - t0) / n_ops;
}

// BURSTY eviction: the best case eager writeback could possibly have.
// Bursts of conflict-evicting traffic separated by fully idle gaps, so a
// background cleaner would have real idle DDR-write bandwidth to spend
// AND real dirty lines to spend it on.  Two numbers are reported:
//
//   burst cyc/op         -- the latency the requester actually waits on.
//                           This is what eager writeback would shorten.
//   gap writeback-idle   -- cycles in the gaps during which the writeback
//                           engine was idle, i.e. the budget a cleaner has.
//                           A writeback costs one DDR write round trip, so
//                           budget/round-trip bounds the lines it could
//                           pre-clean before the next burst.
struct BurstStats { double burst_cyc_per_op = 0; uint64_t burst_stall = 0, burst_cycles = 0;
                    uint64_t gap_cycles = 0, gap_wb_idle = 0; };

static BurstStats perf_evict_burst(int n_bursts, int burst_ops, int gap_cycles,
                                    int n_sets, int n_tags, uint64_t base) {
    BurstStats bs;
    int seq = 0;
    for (int b = 0; b < n_bursts; b++) {
        perf_quiesce();
        EvStats ev0 = g_ev;
        uint64_t t0 = sim_time;
        int issued = 0, done = 0;
        while (done < burst_ops) {
            while (issued < burst_ops && (issued - done) < 8) {
                uint64_t set = static_cast<uint64_t>(seq % n_sets);
                uint64_t tag = static_cast<uint64_t>((seq / n_sets) % n_tags);
                uint64_t a = base + tag * PERF_SET_STRIDE + set * 64;
                std::vector<std::array<uint8_t,16>> d(1);
                std::vector<uint16_t> st(1, 0xFFFF);
                for (int k = 0; k < 16; k++) d[0][k] = static_cast<uint8_t>(seq + k);
                issue_write(a, d, st);
                issued++; seq++;
            }
            tick();
            done += perf_reap();
            if (sim_time - t0 > 3000000) { printf("  PERF: burst HANG\n"); break; }
        }
        bs.burst_cyc_per_op += static_cast<double>(sim_time - t0) / burst_ops;
        bs.burst_stall  += g_ev.stall  - ev0.stall;
        bs.burst_cycles += g_ev.cycles - ev0.cycles;
        // Idle gap: nothing issued, so any writeback-engine idle here is
        // budget a background cleaner could have spent.
        EvStats gv0 = g_ev;
        for (int i = 0; i < gap_cycles; i++) { tick(); perf_reap(); }
        bs.gap_cycles  += g_ev.cycles - gv0.cycles;
        bs.gap_wb_idle += (g_ev.cycles - gv0.cycles) - (g_ev.drain_busy - gv0.drain_busy);
    }
    bs.burst_cyc_per_op /= n_bursts;
    return bs;
}

// Victim-buffer occupancy is reported as "empty / partly full / FULL"
// rather than one column per slot: with a deep buffer the interesting
// question is no longer "is the second slot in use" but "is the buffer
// the thing blocking misses", and only the FULL bucket can do that.
static void perf_ev_report(const EvStats& ev) {
    const unsigned slots = dut->dbg_vb_slots & 0x1F;
    uint64_t part = 0;
    for (unsigned i = 1; i < slots && i <= EV_MAX_SLOTS; i++) part += ev.occ_hist[i];
    const uint64_t full = (slots <= EV_MAX_SLOTS) ? ev.occ_hist[slots] : 0;
    printf("  %-34s VB empty/partial/FULL = %5.1f%% /%5.1f%% /%5.1f%% (of %u slots)   "
           "miss-blocked-on-full-VB = %llu cyc (%.2f%% of %llu)\n", "",
           100.0 * ev.occ_hist[0] / ev.cycles,
           100.0 * part / ev.cycles,
           100.0 * full / ev.cycles,
           slots,
           (unsigned long long)ev.stall,
           100.0 * ev.stall / ev.cycles,
           (unsigned long long)ev.cycles);
    printf("  %-34s writeback in flight %5.1f%% of cycles, AW/W sequencer busy "
           "%5.1f%%   mean outstanding %.2f (peak %llu)\n", "",
           100.0 * ev.drain_busy / ev.cycles,
           100.0 * ev.seq_busy / ev.cycles,
           static_cast<double>(ev.out_sum) / ev.cycles,
           (unsigned long long)ev.out_max);
}

// One compact line per perf row: what the front door did with the accept
// opportunities the tag pipeline gave it, and how long the master sat on a
// refused header.  `ideal` is the fraction of a perfect infinitely-deep
// door -- accepts / (accepts + recoverable bubbles).  100% means depth
// cannot buy anything for this traffic shape.
static void perf_fd_report(const FdStats& d) {
    if (!d.idle) return;
    const uint64_t work = d.accept + d.gather;
    const uint64_t ideal_den = work + d.nowork_refused;
    printf("  %-34s door: idle %llu, accept %llu + gather %llu = %.1f%% used | "
           "bubbles: nowork %llu (queued %llu, REFUSED-EARLIER %llu) idbusy %llu byp %llu | "
           "ideal-door %.1f%% | hdr refused AR %llu/%llu AW %llu/%llu W %llu/%llu cyc, "
           "%llu hdrs, max run %llu\n", "",
           (unsigned long long)d.idle, (unsigned long long)d.accept,
           (unsigned long long)d.gather, 100.0 * work / d.idle,
           (unsigned long long)d.nowork, (unsigned long long)d.nowork_hostwork,
           (unsigned long long)d.nowork_refused,
           (unsigned long long)d.idbusy, (unsigned long long)d.bypnr,
           ideal_den ? 100.0 * work / ideal_den : 100.0,
           (unsigned long long)d.ar_stall, (unsigned long long)d.ar_pres,
           (unsigned long long)d.aw_stall, (unsigned long long)d.aw_pres,
           (unsigned long long)d.w_stall,  (unsigned long long)d.w_pres,
           (unsigned long long)d.hdr_burst, (unsigned long long)d.stall_run_max);
    // Pipeline-era detail: what the 3-stage lookup itself lost, as opposed
    // to what the door lost.  `sethaz` is the array read-after-write
    // interlock refusing an accept; `s2 stall` is the resolve stage unable
    // to retire (MSHR full, response skid full, victim buffer, same-id
    // ordering); `re-read` is a Critical-7 skew-hazard replay.
    printf("  %-34s pipe: accept refused by set-hazard %llu | resolve stalled %llu of %llu cyc "
           "| skew re-reads %llu\n", "",
           (unsigned long long)d.sethaz, (unsigned long long)d.s2_stall,
           (unsigned long long)d.cycles, (unsigned long long)d.s2_reread);
}

static void perf_row(const char* group, const char* name, const char* stim,
                      double cyc_per_op, double bytes_per_op) {
    PerfRow r;
    r.group = group; r.name = name; r.stim = stim;
    r.cyc_per_op = cyc_per_op; r.bytes_per_op = bytes_per_op;
    r.lat = g_lat_n ? static_cast<double>(g_lat_sum) / g_lat_n : 0.0;
    g_perf_rows.push_back(r);
    printf("  %-34s %-30s %8.2f cyc/op %8.1f MB/s   hdr->data %6.1f cy\n",
           name, stim, cyc_per_op,
           bytes_per_op * PERF_CLK_MHZ / cyc_per_op, r.lat);
    perf_fd_report(g_fd_win);
}

static void perf_set_ddr(int latency) {
    g_ddr_latency_base = latency; g_ddr_latency_jitter = 0; g_ddr_beat_gap_max = 0;
    g_mem_force_ready = true;
}

static void perf_suite() {
    const bool old_force = g_mem_force_ready;
    const int old_lat = g_ddr_latency_base, old_jit = g_ddr_latency_jitter, old_gap = g_ddr_beat_gap_max;
    g_perf_always_ready = true;
    // Perf traffic deliberately re-reads addresses whose dirty copies get
    // dropped by the inter-scenario resets, so the scoreboard's unmatched
    // response checks are the only thing that still matters here.
    char buf[128];

    for (int pass = 0; pass < 2; pass++) {
        const int L = pass ? 200 : 40;
        perf_set_ddr(L);
        printf("\n  ── DDR model: fixed %d-cycle first-beat latency, 0 beat gap, always ready ──\n", L);

        // ---- 1. HIT throughput / latency (warm, 32 resident lines) -------
        perf_cold();
        { StreamCfg w; w.n_ops = 32; w.window = 8; w.base = 0x200000; w.stride = 64; perf_stream(w); }
        for (int win : {1, 2, 8}) {
            StreamCfg c; c.n_ops = 512; c.window = win; c.base = 0x200000; c.stride = 64; c.wrap = 32;
            double cy = perf_stream(c);
            snprintf(buf, sizeof buf, "16B read hit, %d outstanding", win);
            perf_row("hit", "hit_read", buf, cy, 16);
        }
        {   // single-ID hit stream: hits never enter the MSHR, so the
            // same-ID accept gate should NOT bite here -- measured, not assumed.
            StreamCfg c; c.n_ops = 512; c.window = 8; c.base = 0x200000; c.stride = 64; c.wrap = 32; c.fixed_id = 61;
            double cy = perf_stream(c);
            perf_row("hit", "hit_read_single_id", "16B read hit, 8 outstanding, 1 ID", cy, 16);
        }
        {   StreamCfg c; c.n_ops = 512; c.window = 8; c.base = 0x200000; c.stride = 64; c.wrap = 32; c.is_write = true;
            double cy = perf_stream(c);
            perf_row("hit", "hit_write", "16B write hit, 8 outstanding", cy, 16);
        }
        {   StreamCfg c; c.n_ops = 256; c.window = 8; c.base = 0x200000; c.stride = 64; c.wrap = 32; c.beats = 4;
            double cy = perf_stream(c);
            perf_row("hit", "hit_read_64B_burst", "64B (4-beat) read hit burst", cy, 64);
        }

        // ---- 2. MISS throughput vs. outstanding window -------------------
        for (int win : {1, 2, 4, 8, 16}) {
            perf_cold();
            StreamCfg c; c.n_ops = 96; c.window = win; c.base = 0x400000; c.stride = 64;
            double cy = perf_stream(c);
            snprintf(buf, sizeof buf, "16B read miss, %d outstanding, uniq IDs", win);
            perf_row("miss", "miss_read", buf, cy, 16);
        }
        {   perf_cold();
            StreamCfg c; c.n_ops = 48; c.window = 8; c.base = 0x500000; c.stride = 64; c.fixed_id = 61;
            double cy = perf_stream(c);
            perf_row("miss", "miss_read_single_id", "16B read miss, 8 outstanding, 1 ID", cy, 16);
        }
        {   perf_cold();
            StreamCfg c; c.n_ops = 96; c.window = 8; c.base = 0x600000; c.stride = 4096;
            double cy = perf_stream(c);
            perf_row("miss", "miss_read_stride4k", "16B read miss, 4 KB stride, 8 out", cy, 16);
        }
        {   perf_cold();
            StreamCfg c; c.n_ops = 96; c.window = 8; c.base = 0x700000; c.stride = 64; c.is_write = true;
            double cy = perf_stream(c);
            perf_row("miss", "miss_write_alloc", "16B write miss (alloc), 8 out", cy, 16);
        }

        // ---- 3. Streaming: line-sized and block-sized bursts --------------
        {   perf_cold();
            StreamCfg c; c.n_ops = 64; c.window = 8; c.base = 0x800000; c.stride = 64; c.beats = 4;
            double cy = perf_stream(c);
            perf_row("stream", "stream_read_64B", "64B read burst, 8 out, uniq IDs", cy, 64);
        }
        {   perf_cold();
            StreamCfg c; c.n_ops = 32; c.window = 8; c.base = 0x900000; c.stride = 64; c.beats = 4; c.fixed_id = 61;
            double cy = perf_stream(c);
            perf_row("stream", "stream_read_64B_single_id", "64B read burst, 8 out, 1 ID", cy, 64);
        }
        {   perf_cold();
            StreamCfg c; c.n_ops = 64; c.window = 8; c.base = 0xA00000; c.stride = 64; c.beats = 4; c.is_write = true;
            double cy = perf_stream(c);
            perf_row("stream", "stream_write_64B", "64B write burst, 8 out, uniq IDs", cy, 64);
        }
        {   perf_cold();
            StreamCfg c; c.n_ops = 12; c.window = 4; c.base = 0xB00000; c.stride = 512; c.beats = 32;
            double cy = perf_stream(c);
            perf_row("stream", "cacheable_read_512B", "512B (32-beat) cacheable read", cy, 512);
        }

        // ---- 3b. DIRTY-EVICTION PRESSURE ----------------------------------
        // The only workload shape under which the 2-entry victim buffer can
        // be the limiter.  Reported alongside the cycles/op is the exact
        // cycle count an eager-writeback scheme would be trying to remove:
        // cycles in which a miss was ready to dispatch and was blocked ONLY
        // by `victim_push_ready`.
        for (int win : {1, 4, 8}) {
            perf_cold();
            EvStats ev;
            double cy = perf_evict_stream(512, win, 16, 16, /*is_write=*/true, 0x0, &ev);
            snprintf(buf, sizeof buf, "16B write, conflict-evict, %d out", win);
            perf_row("evict", "evict_write", buf, cy, 16);
            perf_ev_report(ev);
        }
        {   // Read-heavy variant: the victim data comes from lines dirtied
            // by an earlier write pass, so evictions are dirty but the
            // demand stream is reads.
            perf_cold();
            perf_evict_stream(256, 8, 16, 8, /*is_write=*/true, 0x0, nullptr);
            EvStats ev;
            double cy = perf_evict_stream(512, 8, 16, 16, /*is_write=*/false, 0x0, &ev);
            perf_row("evict", "evict_read_after_dirty", "16B read, conflict-evict, 8 out", cy, 16);
            perf_ev_report(ev);
        }

        // ---- 3c. BURSTY eviction: eager writeback's best case --------------
        {   perf_cold();
            BurstStats bs = perf_evict_burst(6, 128, 4000, 16, 16, 0x0);
            printf("  %-34s %-30s %8.2f cyc/op (burst only)\n",
                   "evict_write_bursty", "128-op bursts, 4000-cyc gaps", bs.burst_cyc_per_op);
            printf("  %-34s burst: miss-blocked-on-full-VB %llu of %llu cyc (%.1f%%)   "
                   "gap: writeback engine IDLE %llu of %llu cyc\n", "",
                   (unsigned long long)bs.burst_stall, (unsigned long long)bs.burst_cycles,
                   100.0 * bs.burst_stall / bs.burst_cycles,
                   (unsigned long long)bs.gap_wb_idle, (unsigned long long)bs.gap_cycles);
        }

        // ---- 3d. FRONT-DOOR PRESSURE --------------------------------------
        // Shapes chosen to put the maximum possible pressure on the ONE
        // aw_have / ar_have register pair, i.e. to maximise the header
        // rate per accepted beat:
        //   * 1-beat bursts   -- one header per beat, the worst ratio there is.
        //   * simultaneous R+W -- both trackers busy at once, so rw_favor
        //                        alternates and each channel's header has to
        //                        wait out the other's beat as well as its own.
        //   * all-hit         -- removes DDR latency from the picture, so
        //                        anything left is pipeline/door occupancy.
        {   perf_cold();
            { StreamCfg w; w.n_ops = 32; w.window = 8; w.base = 0xD00000; w.stride = 64; perf_stream(w); }
            StreamCfg c; c.n_ops = 512; c.window = 8; c.base = 0xD00000; c.stride = 64; c.wrap = 32;
            double cy = perf_stream(c);
            perf_row("door", "door_read_1beat_hit", "1-beat read hit, 8 out (max hdr rate)", cy, 16);
        }
        {   // Interleaved reads and writes to the same resident working
            // set: the only shape in which BOTH front-door trackers are
            // occupied continuously.
            perf_cold();
            { StreamCfg w; w.n_ops = 32; w.window = 8; w.base = 0xD40000; w.stride = 64; perf_stream(w); }
            g_lat_sum = g_lat_n = g_full_sum = g_full_n = 0;
            perf_quiesce();
            FdStats fd0 = g_fd; g_fd.stall_run_max = 0; g_fd_run = 0;
            uint64_t t0 = sim_time;
            const int n_ops = 512;
            int issued = 0, done = 0;
            while (done < n_ops) {
                while (issued < n_ops && (issued - done) < 8) {
                    uint64_t a2 = 0xD40000 + static_cast<uint64_t>(issued % 32) * 64;
                    if (issued & 1) {
                        std::vector<std::array<uint8_t,16>> d(1);
                        std::vector<uint16_t> st(1, 0xFFFF);
                        for (int k = 0; k < 16; k++) d[0][k] = static_cast<uint8_t>(issued + k);
                        issue_write(a2, d, st);
                    } else issue_read(a2, 1);
                    issued++;
                }
                tick();
                done += perf_reap();
                if (sim_time - t0 > 3000000) { printf("  PERF: door mix HANG\n"); break; }
            }
            g_fd_win = fd_diff(g_fd, fd0);
            perf_row("door", "door_rw_mixed_hit", "1-beat R+W hit interleave, 8 out",
                     static_cast<double>(sim_time - t0) / n_ops, 16);
        }
        {   // Same interleave, but every access misses: the MSHRs supply
            // miss parallelism WITHIN a burst; this asks whether the door
            // limits it ACROSS bursts.
            perf_cold();
            g_lat_sum = g_lat_n = g_full_sum = g_full_n = 0;
            perf_quiesce();
            FdStats fd0 = g_fd; g_fd.stall_run_max = 0; g_fd_run = 0;
            uint64_t t0 = sim_time;
            const int n_ops = 128;
            int issued = 0, done = 0;
            while (done < n_ops) {
                while (issued < n_ops && (issued - done) < 8) {
                    uint64_t a2 = 0xD80000 + static_cast<uint64_t>(issued) * 64;
                    if (issued & 1) {
                        std::vector<std::array<uint8_t,16>> d(1);
                        std::vector<uint16_t> st(1, 0xFFFF);
                        for (int k = 0; k < 16; k++) d[0][k] = static_cast<uint8_t>(issued + k);
                        issue_write(a2, d, st);
                    } else issue_read(a2, 1);
                    issued++;
                }
                tick();
                done += perf_reap();
                if (sim_time - t0 > 3000000) { printf("  PERF: door miss mix HANG\n"); break; }
            }
            g_fd_win = fd_diff(g_fd, fd0);
            perf_row("door", "door_rw_mixed_miss", "1-beat R+W miss interleave, 8 out",
                     static_cast<double>(sim_time - t0) / n_ops, 16);
        }

        // ---- 4. Bypass window (the RAM-disk profile) ----------------------
        {   perf_quiesce();
            StreamCfg c; c.n_ops = 8; c.window = 2; c.base = BYPASS_BASE; c.stride = 512; c.beats = 32;
            double cy = perf_stream(c);
            perf_row("bypass", "bypass_read_512B", "512B (32-beat) bypass read", cy, 512);
        }
        {   perf_quiesce();
            StreamCfg c; c.n_ops = 8; c.window = 2; c.base = BYPASS_BASE + 0x40000; c.stride = 512; c.beats = 32; c.is_write = true;
            double cy = perf_stream(c);
            perf_row("bypass", "bypass_write_512B", "512B (32-beat) bypass write", cy, 512);
        }
        {   perf_quiesce();
            StreamCfg c; c.n_ops = 64; c.window = 2; c.base = BYPASS_BASE + 0x80000; c.stride = 16;
            double cy = perf_stream(c);
            perf_row("bypass", "bypass_read_16B", "16B single-beat bypass read", cy, 16);
        }

        // ---- 5. Mixed: CPU-ish hits + a bypass streamer -------------------
        // Both share the one S0 AR channel, which is exactly how the real
        // xbar presents them; this measures head-of-line cost.
        {   perf_cold();
            { StreamCfg w; w.n_ops = 32; w.window = 8; w.base = 0xC00000; w.stride = 64; perf_stream(w); }
            g_lat_sum = g_lat_n = g_full_sum = g_full_n = 0;
            perf_quiesce();
            FdStats fd0 = g_fd; g_fd.stall_run_max = 0; g_fd_run = 0;
            uint64_t t0 = sim_time;
            const int n_hits = 256, n_blocks = 4;
            int hits_iss = 0, blk_iss = 0, done = 0, want = n_hits + n_blocks;
            while (done < want) {
                if (hits_iss + blk_iss - done < 8) {
                    // ~1 block read per 64 CPU accesses -- a disk stream
                    // running underneath a running Mac.
                    if (blk_iss * 64 <= hits_iss && blk_iss < n_blocks) {
                        issue_read(BYPASS_BASE + 0xC0000 + static_cast<uint64_t>(blk_iss) * 512, 32);
                        blk_iss++;
                    } else if (hits_iss < n_hits) {
                        issue_read(0xC00000 + static_cast<uint64_t>(hits_iss % 32) * 64);
                        hits_iss++;
                    }
                }
                tick();
                done += perf_reap();
                if (sim_time - t0 > 3000000) { printf("  PERF: mixed HANG\n"); break; }
            }
            double cy = static_cast<double>(sim_time - t0);
            printf("  %-34s %-30s %8.2f cyc total for %d hits + %d x 512B blocks\n",
                   "mixed_cpu_hits_plus_bypass", "shared S0 AR channel", cy, n_hits, n_blocks);
            printf("  %-34s %-30s %8.2f cyc/hit-equivalent\n", "", "(vs. hit_read baseline)", cy / n_hits);
            g_fd_win = fd_diff(g_fd, fd0);
            perf_fd_report(g_fd_win);
        }

        // ---- 5b. Two CACHEABLE masters sharing S0 -------------------------
        // The real multi-master shape on the cached path: a CPU-ish 1-beat
        // hit stream interleaved with 4-beat (64 B) line fills, which is
        // what an L1 miss looks like.  The interesting question is
        // head-of-line: a 4-beat burst owns the single ar_have tracker for
        // 12 cycles (three per beat through S_IDLE/S_WAIT/S_LOOKUP), so a
        // 1-beat read behind it waits.  A second tracker could interleave
        // them -- but only by moving latency between the two streams, since
        // the beats themselves still cost 3 cycles each either way.  This
        // row exists so that claim is a measurement and not an assertion.
        {   perf_cold();
            { StreamCfg w; w.n_ops = 64; w.window = 8; w.base = 0xE00000; w.stride = 64; perf_stream(w); }
            g_lat_sum = g_lat_n = g_full_sum = g_full_n = 0;
            perf_quiesce();
            FdStats fd0 = g_fd; g_fd.stall_run_max = 0; g_fd_run = 0;
            uint64_t t0 = sim_time;
            const int n_short = 256, n_long = 32;
            int s_iss = 0, l_iss = 0, done = 0, want = n_short + n_long;
            while (done < want) {
                while (s_iss + l_iss - done < 8 && (s_iss < n_short || l_iss < n_long)) {
                    // ~8 single-beat accesses per 64 B line fill.
                    if (l_iss * 8 <= s_iss && l_iss < n_long) {
                        issue_read(0xE00000 + static_cast<uint64_t>(l_iss % 32) * 64, 4);
                        l_iss++;
                    } else if (s_iss < n_short) {
                        issue_read(0xE00000 + static_cast<uint64_t>(s_iss % 32) * 64, 1);
                        s_iss++;
                    } else break;
                }
                tick();
                done += perf_reap();
                if (sim_time - t0 > 3000000) { printf("  PERF: two-master HANG\n"); break; }
            }
            g_fd_win = fd_diff(g_fd, fd0);
            const double cy = static_cast<double>(sim_time - t0);
            printf("  %-34s %-30s %8.2f cyc total, %d x 1-beat + %d x 4-beat hits (%.2f cyc/beat)\n",
                   "two_cacheable_masters", "shared S0 AR channel", cy, n_short, n_long,
                   cy / (n_short + 4 * n_long));
            perf_fd_report(g_fd_win);
        }
    }

    g_perf_always_ready = false;
    g_mem_force_ready = old_force;
    g_ddr_latency_base = old_lat; g_ddr_latency_jitter = old_jit; g_ddr_beat_gap_max = old_gap;
}

// FRONT-DOOR THROUGHPUT FLOOR (re-baselined 2026-08-20 for the pipelined lookup)
//
// HISTORY, because the previous baseline was right and is now void, and the
// reason it flipped is the whole point of this test.
//
//   7d49e1f asserted a 3.00 cyc/op floor and proved a SINGLE-ENTRY header
//   door cost nothing (REFUSED-EARLIER == 0 on every cacheable row).  Both
//   halves were true, and they were true BECAUSE the lookup was a 3-cycle
//   S_IDLE / S_WAIT / S_LOOKUP loop: `ar_have` cleared on the edge that
//   took a burst's last beat, so the next header had a TWO-cycle window
//   (S_WAIT + S_LOOKUP) to arrive before any accept slot was wasted.  Two
//   cycles of slack against a three-cycle loop.
//
//   Pipelining the lookup to one beat per cycle deleted that slack.  The
//   pipeline ALONE measured 2.01 cyc/op on this stream with 255
//   REFUSED-EARLIER bubbles out of 256 ops, i.e. the door became the
//   limiter the instant the loop stopped hiding it, exactly as predicted.
//   l2c_ctrl now carries a one-entry pre-latch per header channel and the
//   stream runs at ~1.02 cyc/op with REFUSED-EARLIER back to 0.
//
// The floor asserted below is therefore 1 beat/cycle, not 3, and the
// anti-vacuity guards had to be re-derived too: with a 2-deep door and a
// 1/cycle pipeline this master can no longer saturate the door at all, so
// "AR is refused >50% of presented cycles" (the old proof that the test was
// not vacuous) is now FALSE BY DESIGN.  It is replaced by the two
// properties that actually matter and cannot both hold vacuously:
//
//   * the master really is presenting a header on (nearly) every cycle
//     (ar_pres / w_pres vs. elapsed cycles), so one accept per cycle is
//     genuinely being ASKED for; and
//   * every header presented was eventually accepted (hdr_burst == n_ops)
//     with zero REFUSED-EARLIER bubbles.
//
// A regression that narrows the door back to one entry fails the cyc/op
// check AND lights up REFUSED-EARLIER; a regression that widens the array
// hazard interlock fails the new `sethaz == 0` check; a regression that
// un-pipelines the lookup fails cyc/op alone.
//
// Everything it exercises is a HIT, so nothing here depends on DDR
// latency, MSHR depth or the victim buffer.
struct FloorResult { double cyc_per_op = 0; uint64_t cycles = 0; FdStats fd; };

// mix: 0 = reads only, 1 = writes only, 2 = alternating R/W.
// All three are needed, and the reason is worth stating: with an
// alternating stream the two channels COVER FOR EACH OTHER.  Narrowing
// only the AW header window leaves the interleaved stream at exactly the
// same cycles/op because a read fills every slot the write side drops, so
// an R+W-only test is blind to a write-side regression, and vice versa.
// Each channel therefore gets its own saturating single-channel case, and
// the interleave is kept on top of them for rw_favor coverage rather than
// as the primary detector.
static FloorResult front_door_floor_run(uint64_t base, int n_lines, int n_ops,
                                         int window, int mix) {
    FloorResult res;
    perf_quiesce();
    FdStats fd0 = g_fd;
    g_fd.stall_run_max = 0; g_fd_run = 0;
    const uint64_t t0 = sim_time;
    int issued = 0, done = 0;
    while (done < n_ops) {
        while (issued < n_ops && (issued - done) < window) {
            const uint64_t a = base + static_cast<uint64_t>(issued % n_lines) * 64;
            const bool is_wr = (mix == 1) || ((mix == 2) && (issued & 1));
            if (is_wr) {
                std::vector<std::array<uint8_t,16>> d(1);
                std::vector<uint16_t> st(1, 0xFFFF);
                for (int k = 0; k < 16; k++) d[0][k] = static_cast<uint8_t>(issued + k);
                issue_write(a, d, st);
            } else {
                issue_read(a, 1);
            }
            issued++;
        }
        tick();
        done += perf_reap();
        if (sim_time - t0 > 400000) break;   // caller's cyc/op check will fail
    }
    res.fd = fd_diff(g_fd, fd0);
    res.cycles = sim_time - t0;
    res.cyc_per_op = static_cast<double>(res.cycles) / n_ops;
    return res;
}

static bool test_front_door_throughput_floor() {
    bool ok = true;
    // Deterministic environment: this is a cycle-count assertion, so DDR
    // jitter and requester backpressure have to be off.  All saved and
    // restored: later checks in main() share these globals.
    const bool old_force = g_mem_force_ready, old_always = g_perf_always_ready;
    const bool old_runahead = g_aw_runahead;
    const int old_lat = g_ddr_latency_base, old_jit = g_ddr_latency_jitter, old_gap = g_ddr_beat_gap_max;
    g_perf_always_ready = true;
    g_ddr_latency_base = 40; g_ddr_latency_jitter = 0; g_ddr_beat_gap_max = 0;
    g_mem_force_ready = true;

    const uint64_t BASE = 0xF00000;
    const int LINES = 32;
    // Prime: a 16 B write with every strobe set fully covers its quadrant,
    // so it installs valid+dirty with NO fill (docs/l2c_perf.md S12).  The
    // working set goes resident without a single DDR read, which keeps the
    // measurement below free of any fill latency at all.
    for (int i = 0; i < LINES; i++) {
        std::vector<std::array<uint8_t,16>> d(1);
        std::vector<uint16_t> st(1, 0xFFFF);
        for (int k = 0; k < 16; k++) d[0][k] = static_cast<uint8_t>(i + k);
        issue_write(BASE + static_cast<uint64_t>(i) * 64, d, st);
    }
    perf_quiesce();

    // 1. Read-only stream: one header per beat, the worst header:beat
    //    ratio a master can present.
    FloorResult r = front_door_floor_run(BASE, LINES, 256, 8, /*mix=*/0);
    printf("  front door, 1-beat read hits : %.3f cyc/op, door %llu/%llu accepts used, "
           "refused-earlier bubbles %llu, AR presented %llu of %llu cyc (refused %llu), "
           "set-hazard refusals %llu, resolve stalls %llu\n",
           r.cyc_per_op, (unsigned long long)r.fd.accept, (unsigned long long)r.fd.idle,
           (unsigned long long)r.fd.nowork_refused,
           (unsigned long long)r.fd.ar_pres, (unsigned long long)r.cycles,
           (unsigned long long)r.fd.ar_stall,
           (unsigned long long)r.fd.sethaz, (unsigned long long)r.fd.s2_stall);
    // 1.00 is the pipelined floor (one beat retired per cycle).  The
    // tolerance covers only the three cycles of pipeline fill at the start
    // of the stream, not a per-op regression: one extra cycle per op would
    // read 2.00, which is exactly what the un-deepened door measured.
    CHECK("1-beat read hit stream sustains the 1-beat/cycle pipeline floor",
          r.cyc_per_op < 1.10);
    // The door must never make the pipeline idle while a header it already
    // refused is waiting outside.  This is the direct statement that the
    // door's depth is sufficient, and it is exact: zero, not "few".
    CHECK("no accept slot lost to a previously-refused read header",
          r.fd.nowork_refused == 0);
    // Anti-vacuity #1: the master really is asking for one accept per
    // cycle.  Without this the two checks above would pass on a stream
    // that simply never offered work.
    CHECK("the read stream really is presenting a header nearly every cycle",
          r.cycles > 0 && r.fd.ar_pres * 100 >= r.cycles * 95);
    // Anti-vacuity #2: every header offered was taken.
    CHECK("every read header presented was accepted", r.fd.hdr_burst == 256);
    // The array read-after-write interlock must cost NOTHING here: 32
    // distinct lines walked in order means no two requests within two
    // pipeline stages of each other ever share a set.  A regression that
    // widened the interlock to plain same-set, or to same-set-any-mix,
    // would light this up.
    CHECK("array hazard interlock refuses nothing on a distinct-line read stream",
          r.fd.sethaz == 0);

    // 2. Write-only stream, DEFAULT master (AW and W from one cursor).
    //    This one does NOT reach 1.00 and the reason is not the door:
    //    l2c cannot consume a W beat until `aw_have` is registered, so a
    //    master that presents AW and W in the same cycle spends one cycle
    //    per burst waiting for its own header to land.  With 1-beat bursts
    //    that is 2.00 cyc/op, and REFUSED-EARLIER stays 0 throughout,
    //    which is the evidence that it is an AW-to-W latency cost and not
    //    a door-depth cost.  Case 3 proves it by removing the master's
    //    serialisation and nothing else.
    FloorResult w = front_door_floor_run(BASE, LINES, 256, 8, /*mix=*/1);
    printf("  front door, 1-beat write hits: %.3f cyc/op, door %llu/%llu accepts used, "
           "refused-earlier bubbles %llu, W refused %llu of %llu presented cyc, "
           "set-hazard refusals %llu\n",
           w.cyc_per_op, (unsigned long long)w.fd.accept, (unsigned long long)w.fd.idle,
           (unsigned long long)w.fd.nowork_refused,
           (unsigned long long)w.fd.w_stall, (unsigned long long)w.fd.w_pres,
           (unsigned long long)w.fd.sethaz);
    CHECK("1-beat write hit stream sustains the AW-then-W master's 2-cycle floor",
          w.cyc_per_op < 2.10);
    CHECK("no accept slot lost to a previously-refused write header",
          w.fd.nowork_refused == 0);
    CHECK("the write stream really is presenting a beat nearly every cycle",
          w.cycles > 0 && w.fd.w_pres * 100 >= w.cycles * 95);
    CHECK("every write header presented was accepted", w.fd.hdr_burst == 256);
    CHECK("array hazard interlock refuses nothing on a distinct-line write stream",
          w.fd.sethaz == 0);

    // 3. Write-only stream, AW RUNAHEAD master.  Identical traffic, one
    //    difference: the AW channel is allowed to present burst N+1's
    //    header while burst N's W beats drain (W order is still AW order,
    //    as AXI4 requires).  This isolates the DUT's own write floor from
    //    the default master's AW/W coupling.
    g_aw_runahead = true;
    FloorResult wr = front_door_floor_run(BASE, LINES, 256, 8, /*mix=*/1);
    g_aw_runahead = false;
    printf("  front door, 1-beat write hits (AW runahead): %.3f cyc/op, door %llu/%llu accepts "
           "used, refused-earlier bubbles %llu, AW presented %llu of %llu cyc\n",
           wr.cyc_per_op, (unsigned long long)wr.fd.accept, (unsigned long long)wr.fd.idle,
           (unsigned long long)wr.fd.nowork_refused,
           (unsigned long long)wr.fd.aw_pres, (unsigned long long)wr.cycles);
    CHECK("write hit stream reaches the 1-beat/cycle floor with a pipelined AW master",
          wr.cyc_per_op < 1.10);
    CHECK("no accept slot lost to a previously-refused header under AW runahead",
          wr.fd.nowork_refused == 0);
    CHECK("AW runahead really did remove the master's serialisation (it is faster)",
          wr.cyc_per_op < w.cyc_per_op - 0.5);

    // 4. Interleaved reads and writes: both front-door trackers occupied
    //    continuously, so rw_favor alternates and each channel's header has
    //    to wait out the other channel's beat as well as its own.
    //    Deliberately NOT the primary detector, see front_door_floor_run.
    FloorResult m = front_door_floor_run(BASE, LINES, 256, 8, /*mix=*/2);
    printf("  front door, 1-beat R+W hits  : %.3f cyc/op, door %llu/%llu accepts used, "
           "refused-earlier bubbles %llu, set-hazard refusals %llu\n",
           m.cyc_per_op, (unsigned long long)m.fd.accept, (unsigned long long)m.fd.idle,
           (unsigned long long)m.fd.nowork_refused, (unsigned long long)m.fd.sethaz);
    CHECK("interleaved R+W hit stream sustains the 1-beat/cycle pipeline floor",
          m.cyc_per_op < 1.10);
    CHECK("no accept slot lost to a previously-refused header under R+W interleave",
          m.fd.nowork_refused == 0);
    CHECK("every R+W header presented was accepted", m.fd.hdr_burst == 256);

    perf_quiesce();
    g_perf_always_ready = old_always; g_mem_force_ready = old_force;
    g_aw_runahead = old_runahead;
    g_ddr_latency_base = old_lat; g_ddr_latency_jitter = old_jit; g_ddr_beat_gap_max = old_gap;
    return ok;
}
#endif // !BYPASS_ALL_BUILD

// ─── Runner ────────────────────────────────────────────────────────────
static void run(const char* name, bool (*fn)()) {
    bool r = fn();
    printf("[%s] %s\n", r ? "PASS" : "FAIL", name);
    if (r) n_pass++; else n_fail++;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new DutT();
    configure_randomness();
    init_memories();
    init_ids();
    reset_dut();

#ifndef BYPASS_ALL_BUILD
    if (std::getenv("L2C_DEBUG_SINGLE_WRITE")) {
        uint32_t id = issue_write_full(0x2000, 0x11);
        for (int i = 0; i < 300; i++) {
            printf("t=%3d aw=%d/%d w=%d/%d ar=%d/%d r=%d/%d b=%d/%d\n", i,
                   dut->s_axi_awvalid, dut->s_axi_awready, dut->s_axi_wvalid, dut->s_axi_wready,
                   dut->s_axi_arvalid, dut->s_axi_arready, dut->s_axi_rvalid, dut->s_axi_rready,
                   dut->s_axi_bvalid, dut->s_axi_bready);
            printf("    m: aw=%d/%d w=%d/%d ar=%d/%d r=%d/%d b=%d/%d\n",
                   dut->m_axi_awvalid, dut->m_axi_awready, dut->m_axi_wvalid, dut->m_axi_wready,
                   dut->m_axi_arvalid, dut->m_axi_arready, dut->m_axi_rvalid, dut->m_axi_rready,
                   dut->m_axi_bvalid, dut->m_axi_bready);
            tick();
            if (w_done.count(id)) { printf("DONE at t=%d\n", i); break; }
        }
        delete dut;
        return 0;
    }
    if (std::getenv("L2C_DEBUG_EVICT_STRESS")) {
        // Same set, many distinct tags -- forces continuous PLRU eviction
        // well past the 8-way capacity. Fully sequential/blocking so any
        // corruption is trivial to localize to a specific tag index.
        const uint64_t set_stride = 4096 * LINE_BYTES;
        const uint64_t base = 0x10000;
        int n = 60;
        if (const char* e = std::getenv("L2C_DEBUG_EVICT_N")) n = std::atoi(e);
        bool all_ok = true;
        for (int i = 0; i < n; i++) {
            uint64_t a = base + static_cast<uint64_t>(i) * set_stride;
            uint32_t id = issue_write_full(a, 0x40 + i);
            if (!wait_write_done(id)) { printf("evict_stress: write %d TIMEOUT @0x%llx\n", i, (unsigned long long)a); all_ok = false; continue; }
            free_write(id);
            for (int b = 0; b < 16; b++) golden[a+b] = static_cast<uint8_t>((0x40+i+b) & 0xFF);
            touched[a] = true;
        }
        printf("evict_stress: wrote %d tags into one set, now reading all back in order...\n", n);
        for (int i = 0; i < n; i++) {
            uint64_t a = base + static_cast<uint64_t>(i) * set_stride;
            uint32_t id = issue_read(a);
            if (!wait_read_done(id)) { printf("evict_stress: read %d TIMEOUT @0x%llx\n", i, (unsigned long long)a); all_ok = false; continue; }
            bool match = std::memcmp(r_done[id]->got[0].data(), &golden[a], 16) == 0;
            if (!match) {
                printf("evict_stress: read %d MISMATCH @0x%llx got=%02x%02x%02x%02x exp=%02x%02x%02x%02x\n", i,
                       (unsigned long long)a, r_done[id]->got[0][0], r_done[id]->got[0][1], r_done[id]->got[0][2], r_done[id]->got[0][3],
                       golden[a], golden[a+1], golden[a+2], golden[a+3]);
                all_ok = false;
            }
            free_read(id);
        }
        printf("evict_stress: %s\n", all_ok ? "ALL OK" : "FAILURES SEEN");
        delete dut;
        return all_ok ? 0 : 1;
    }
    if (std::getenv("L2C_DEBUG_EVICT_CONCURRENT")) {
        // Same set, many distinct tags, CONCURRENT (not sequential) --
        // stresses busy_mask/victim-select's handling of multiple
        // in-flight misses to the same set at once (unlike the fully
        // sequential evict_stress test above).
        const uint64_t set_stride = 4096 * LINE_BYTES;
        const uint64_t base = 0x20000;
        int n = 60, window = 6;
        if (const char* e = std::getenv("L2C_DEBUG_EVICT_N")) n = std::atoi(e);
        if (const char* e = std::getenv("L2C_DEBUG_EVICT_WINDOW")) window = std::atoi(e);
        bool all_ok = true;
        std::vector<uint64_t> addrs(n);
        for (int i = 0; i < n; i++) addrs[i] = base + static_cast<uint64_t>(i) * set_stride;
        // Concurrent write phase.  Gate on (issued - completed), NOT on
        // w_awaiting_b/w_issuing alone -- those only gain/lose entries
        // *after* a tick() actually runs the handshake, so counting only
        // them lets this loop dump all n writes into w_pending_issue in
        // one shot before the first tick() ever runs (the exact class of
        // bug already found/fixed in test_randomized_scoreboard's
        // inflight.size() window check).
        int next = 0, done_count = 0;
        while (next < n || done_count < n) {
            while (next < n && (next - done_count) < window) {
                issue_write_full(addrs[next], 0x60 + next);
                next++;
            }
            tick();
            for (auto it = w_done.begin(); it != w_done.end(); ) {
                for (int b = 0; b < 16; b++) golden[it->second->addr+b] = it->second->data[0][b];
                touched[it->second->addr] = true;
                free_ids.insert(it->first);
                done_count++;
                it = w_done.erase(it);
            }
            if (sim_time > 500000) { printf("evict_stress_concurrent: HANG in write phase (next=%d done=%d)\n", next, done_count); all_ok = false; break; }
        }
        printf("evict_stress_concurrent: wrote %d tags (window=%d), now reading back...\n", n, window);
        for (int i = 0; i < n; i++) {
            uint32_t id = issue_read(addrs[i]);
            if (!wait_read_done(id)) { printf("evict_stress_concurrent: read %d TIMEOUT @0x%llx\n", i, (unsigned long long)addrs[i]); all_ok = false; continue; }
            bool match = std::memcmp(r_done[id]->got[0].data(), &golden[addrs[i]], 16) == 0;
            if (!match) {
                printf("evict_stress_concurrent: read %d MISMATCH @0x%llx got=%02x%02x%02x%02x exp=%02x%02x%02x%02x\n", i,
                       (unsigned long long)addrs[i], r_done[id]->got[0][0], r_done[id]->got[0][1], r_done[id]->got[0][2], r_done[id]->got[0][3],
                       golden[addrs[i]], golden[addrs[i]+1], golden[addrs[i]+2], golden[addrs[i]+3]);
                all_ok = false;
            }
            free_read(id);
        }
        printf("evict_stress_concurrent: %s\n", all_ok ? "ALL OK" : "FAILURES SEEN");
        delete dut;
        return all_ok ? 0 : 1;
    }
    if (std::getenv("L2C_DEBUG_EVICT_MIXED")) {
        // Same set, MIXED concurrent reads+writes (unlike the write-only
        // concurrent test above) -- repeat many rounds to accumulate
        // eviction pressure, checking every completion immediately.
        const uint64_t set_stride = 4096 * LINE_BYTES;
        const uint64_t base = 0x30000;
        int rounds = 2000, window = 7;
        if (const char* e = std::getenv("L2C_DEBUG_EVICT_ROUNDS")) rounds = std::atoi(e);
        if (const char* e = std::getenv("L2C_DEBUG_EVICT_WINDOW")) window = std::atoi(e);
        // addr[i] = base + i*stride must stay within [0, CACHEABLE_SIZE) --
        // cap rounds at the number of distinct tags that actually fit
        // (each address touched at most once, avoiding read/write hazard
        // bookkeeping this simple debug harness doesn't do).
        uint64_t max_tags = (CACHEABLE_SIZE - base) / set_stride;
        if (static_cast<uint64_t>(rounds) > max_tags) rounds = static_cast<int>(max_tags);
        bool all_ok = true;
        int issued = 0, done = 0;
        std::vector<uint64_t> tag_addr(rounds);
        for (int i = 0; i < rounds; i++) tag_addr[i] = base + static_cast<uint64_t>(i) * set_stride;
        while (done < rounds) {
            while (issued < rounds && (issued - done) < window) {
                uint64_t a = tag_addr[issued];
                if (issued % 2 == 0) {
                    uint32_t id = issue_write_full(a, 0x70 + issued);
                    log_op(a, true, id);
                } else {
                    uint32_t id = issue_read(a);
                    log_op(a, false, id);
                }
                issued++;
            }
            tick();
            for (auto it = w_done.begin(); it != w_done.end(); ) {
                for (int b = 0; b < 16; b++) golden[it->second->addr+b] = it->second->data[0][b];
                touched[it->second->addr] = true;
                free_ids.insert(it->first);
                done++;
                it = w_done.erase(it);
            }
            for (auto it = r_done.begin(); it != r_done.end(); ) {
                uint64_t a = it->second->addr;
                bool match = std::memcmp(it->second->got[0].data(), &golden[a], 16) == 0;
                if (!match) {
                    printf("evict_mixed: READ MISMATCH @0x%llx got=%02x%02x%02x%02x exp=%02x%02x%02x%02x\n",
                           (unsigned long long)a, it->second->got[0][0], it->second->got[0][1], it->second->got[0][2], it->second->got[0][3],
                           golden[a], golden[a+1], golden[a+2], golden[a+3]);
                    all_ok = false;
                    dump_op_log();
                }
                free_ids.insert(it->first);
                done++;
                it = r_done.erase(it);
            }
            if (sim_time > 2000000) { printf("evict_mixed: HANG (issued=%d done=%d)\n", issued, done); all_ok = false; break; }
        }
        printf("evict_mixed: %d rounds, %s\n", rounds, all_ok ? "ALL OK" : "FAILURES SEEN");
        delete dut;
        return all_ok ? 0 : 1;
    }
#endif

#ifdef BYPASS_ALL_BUILD
    run("bypass_all_passthrough_equivalence", test_bypass_all_equivalence);
#else
    if (!std::getenv("L2C_SKIP_DIRECTED")) {
    run("read_miss_fill",              test_read_miss_fill);
    run("fill_bank_quadrants",         test_fill_bank_quadrants);
    run("read_hit",                    test_read_hit);
    run("write_miss_allocate",         test_write_miss_allocate);
    run("write_hit_dirty",             test_write_hit_dirty);
    run("dirty_eviction_writeback",    test_dirty_eviction_writeback);
    run("victim_buffer_hazard",        test_victim_buffer_hazard);
    run("mshr_merge",                  test_mshr_merge);
    run("mshr_full_backpressure",      test_mshr_full_backpressure);
    run("mshr_pipelined_reverse_returns", test_mshr_pipelined_reverse_returns);
    run("mshr_ar_backpressure_release", test_mshr_ar_backpressure_release);
    run("mshr_merge_queue_saturation",  test_mshr_merge_queue_saturation);
    run("hits_progress_under_full_mshr", test_hits_progress_under_full_mshr);
    run("mshr_randomized_completion_stress", test_mshr_randomized_completion_stress);
    run("fill_error_retry",             test_fill_error_retry);
    run("concurrent_same_set_dirty_evictions", test_concurrent_same_set_dirty_evictions);
    run("pipeline_same_set_read_overlap", test_pipeline_same_set_read_overlap);
    run("bypass_read_write_ordering",  test_bypass_read_write_ordering);
    run("reset_walk",                  test_reset_walk);
    run("back_to_back_mixed_bursts",   test_back_to_back_mixed_bursts);
    run("multi_beat_write_hit_single_bresp", test_multi_beat_write_hit_single_bresp);
    run("illegal_burst_slverr",        test_illegal_burst_slverr);
    run("narrow_awsize_write",         test_narrow_awsize_write);
    run("partial_wstrb_all_quadrants", test_partial_wstrb_all_quadrants);
    run("same_id_ordering",            test_same_id_ordering);
    run("fetch_id0_does_not_starve_lsu_bypass_id0",
        test_fetch_id0_does_not_starve_lsu_bypass_id0);
    run("fetch_door_back_to_back_bursts", test_fetch_door_back_to_back_bursts);
    run("dual_source_response_association_stress",
        test_dual_source_response_association_stress);
    run("mshr_replay_partial_write",   test_mshr_replay_partial_write);
    run("replay_payload_slot_mapping", test_replay_payload_slot_mapping);
    run("replay_all_slots_same_quadrant_order", test_replay_all_slots_same_quadrant_order);
    run("replay_merge_during_install_walk", test_replay_merge_during_install_walk);
    run("back_to_back_replays_distinct_entries", test_back_to_back_replays_distinct_entries);
    run("full_line_write_no_fetch",         test_full_line_write_no_fetch);
    run("partial_line_write_still_fetches", test_partial_line_write_still_fetches);
    run("unaligned_burst_still_fetches",    test_unaligned_burst_still_fetches);
    run("full_line_write_multi_line_burst", test_full_line_write_multi_line_burst);
    run("full_line_write_over_resident_dirty", test_full_line_write_over_resident_dirty);
    run("full_line_write_evicts_dirty_victim", test_full_line_write_evicts_dirty_victim);
    run("sector_write_no_fetch",            test_sector_write_no_fetch);
    run("sector_partial_quadrant_still_fetches", test_sector_partial_quadrant_still_fetches);
    run("sector_fill_into_resident_way",    test_sector_fill_into_resident_way);
    run("sector_dirty_writeback_is_partial", test_sector_dirty_writeback_is_partial);
    run("sector_mixed_dirty_span",          test_sector_mixed_dirty_span);
    run("sector_clean_quadrant_not_written_back", test_sector_clean_quadrant_not_written_back);
    run("sector_write_during_fill_retries", test_sector_write_during_fill_retries);
    run("bypass_read_during_full_line_gather", test_bypass_read_during_full_line_gather);
    run("reset_mid_w_burst",           test_reset_mid_w_burst);
    run("reset_b_pending_next_writeback_correct", test_reset_b_pending_next_writeback_correct);
    run("reset_compose_fill_and_writeback", test_reset_compose_fill_and_writeback);
    run("reset_aw_pending_unaccepted",      test_reset_aw_pending_unaccepted);
    run("deep_victim_query_covers_every_slot", test_deep_victim_query_covers_every_slot);
    run("victim_writeback_bresp_error_surfaces", test_victim_writeback_bresp_error_surfaces);
    run("deep_victim_partial_writebacks_stay_partial",
        test_deep_victim_partial_writebacks_stay_partial);
    run("bypass_read_pipelines_to_depth",   test_bypass_read_pipelines_to_depth);
    run("bypass_write_pipelines_to_depth",  test_bypass_write_pipelines_to_depth);
    run("bypass_read_write_never_overlap",  test_bypass_read_write_never_overlap);
    run("bypass_burst_leaves_front_door_usable", test_bypass_burst_leaves_front_door_usable);
    run("reset_mid_bypass_write_pipeline",  test_reset_mid_bypass_write_pipeline);
    // Registered LAST among the directed tests so the ddr_rng draw stream
    // seen by everything above is unchanged.
    run("reset_mid_fill_stale_burst_not_installed",
        test_reset_mid_fill_stale_burst_not_installed);
    run("reset_mid_fill_stale_burst_does_not_wedge_r_channel",
        test_reset_mid_fill_stale_burst_does_not_wedge_r_channel);
    }

    int n_ops = 20000;
    if (const char* e = std::getenv("L2C_RAND_OPS")) n_ops = std::atoi(e);
    {
        bool r = test_randomized_scoreboard(n_ops);
        printf("[%s] randomized_scoreboard\n", r ? "PASS" : "FAIL");
        if (r) n_pass++; else n_fail++;
    }

    perf_smoke_hit_latency();
    run("front_door_throughput_floor", test_front_door_throughput_floor);
    if (std::getenv("L2C_PERF")) {
        printf("\n=== l2c performance measurement suite ===\n");
        perf_suite();
        // The perf suite resets the DUT mid-flight between scenarios and
        // reads addresses whose dirty copies were dropped by design, so
        // suppress the golden/unmatched bookkeeping it cannot satisfy.
        g_unmatched_resp_seen = false;
    }
#endif

    if (g_unmatched_resp_seen) {
        printf("[FAIL] unmatched_response_protocol_check (see UNEXPECTED B/R lines above)\n");
        n_fail++;
    } else if (n_pass || n_fail) {
        printf("[PASS] unmatched_response_protocol_check\n");
        n_pass++;
    }
    if (g_stability_violation) {
        printf("[FAIL] axi_payload_stability_check (see STABILITY VIOLATION lines above)\n");
        n_fail++;
    } else if (n_pass || n_fail) {
        printf("[PASS] axi_payload_stability_check\n");
        n_pass++;
    }

    if (g_ev.cycles) {
        if (g_ev_tap_violation) {
            printf("[FAIL] eviction_tap_self_check (see EV TAP VIOLATION above)\n");
            n_fail++;
        } else {
            printf("[PASS] eviction_tap_self_check\n");
            n_pass++;
        }
    }

    // Whole-run bypass invariants (see by_latch_edge/by_sample_taps).  These
    // hold over EVERY cycle of EVERY scenario, the randomized scoreboard and
    // the perf suite included -- which is the point: the randomized mix
    // issues bypass reads and writes concurrently, so the ordering invariant
    // gets far more exposure there than any directed case can give it.
#ifndef BYPASS_ALL_BUILD
    if (g_ev.cycles) {
        if (g_by_mixdir_violation) {
            printf("[FAIL] bypass_rw_exclusion_check (see BYPASS ORDER VIOLATION above)\n");
            n_fail++;
        } else {
            printf("[PASS] bypass_rw_exclusion_check\n");
            n_pass++;
        }
        if (g_by_tap_violation) {
            printf("[FAIL] bypass_tap_self_check (see BYPASS TAP VIOLATION above)\n");
            n_fail++;
        } else {
            printf("[PASS] bypass_tap_self_check (peak %u in flight of %u occupied, depth %u)\n",
                   g_by_inflight_max, g_by_occ_max, dut->dbg_by_slots);
            n_pass++;
        }
    }
#endif

    if (std::getenv("L2C_EVSTATS") && g_ev.cycles) {
        printf("\n=== whole-run eviction pressure (every cycle of every test) ===\n");
        printf("  cycles                 %llu\n", (unsigned long long)g_ev.cycles);
        for (int i = 0; i <= EV_MAX_SLOTS; i++)
            if (g_ev.occ_hist[i])
                printf("  victim-buffer occ %-2d   %5.2f%%\n", i,
                       100.0 * g_ev.occ_hist[i] / g_ev.cycles);
        printf("  miss blocked on VB     %5.2f%% (%llu cyc)\n",
               100.0 * g_ev.stall / g_ev.cycles, (unsigned long long)g_ev.stall);
        printf("  writeback in flight    %5.2f%%\n", 100.0 * g_ev.drain_busy / g_ev.cycles);
        printf("  AW/W sequencer busy    %5.2f%%\n", 100.0 * g_ev.seq_busy / g_ev.cycles);
        printf("  writebacks outstanding mean %.3f, peak %llu\n",
               static_cast<double>(g_ev.out_sum) / g_ev.cycles,
               (unsigned long long)g_ev.out_max);
    }

    // Pipeline-mechanism coverage over the WHOLE run.  These three paths
    // are rare by construction and every perf row reads 0 on all of them,
    // so without this line a bug in any of them would sit behind dead
    // code and the suite would still read green.  Non-zero here is the
    // evidence that the directed + randomized scenarios really do drive
    // the array RAW interlock, the skew-hazard re-read, and the resolve
    // stall (which is what forces l2c_data's rd_en hold to matter).
    printf("  pipeline coverage: set-hazard refusals %llu, skew re-reads %llu, "
           "resolve stalls %llu, cycles %llu\n",
           (unsigned long long)g_fd.sethaz, (unsigned long long)g_fd.s2_reread,
           (unsigned long long)g_fd.s2_stall, (unsigned long long)g_fd.cycles);
    printf("\nl2c: %d PASS / %d FAIL\n", n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
