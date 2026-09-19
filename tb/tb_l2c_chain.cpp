// tb_l2c_chain.cpp -- T13 integration testbench for the REAL chain:
//   l2c -> axi_async_bridge -> axi_ddr4_mig_bridge -> sim_mig_backend
// across two clock domains (core_clk / mig_clk), via tb/tb_l2c_chain.v.
//
// This does NOT re-derive l2c's own cache-correctness suite (that's
// tb-l2c / tb-l2c-bypass-all, docs/l2c_spec.md S10) -- it proves the
// INTEGRATION seam: that l2c's traffic survives the real CDC + MIG
// contract shim + behavioural memory unmodified, with concurrency and
// resets exercised end to end.  Scenarios (T13 brief):
//
//   1. randomized_chain_traffic  -- >=5000 mixed R/W ops with s_axi
//      backpressure + STALL_ENABLE mig-side command stalls, verified
//      against a host-side golden model, no RTL backdoor (read-back
//      through the DUT only, exactly like tb_l2c.cpp's own scoreboard).
//   2. concurrent_fill_overlap   -- 8 back-to-back miss reads (issued
//      without waiting) vs. 8 sequential (issue+wait) reads to fresh
//      lines; prints the cycle counts and asserts the concurrent form
//      is faster, proving MSHR fills actually overlap across the CDC.
//   3. reset_mid_traffic         -- core_rst asserted with fills/
//      writebacks in flight (mig_clk keeps running); chain must recover
//      and fresh post-reset traffic must be correct (pre-reset dirty
//      data is expected lost per docs/l2c_spec.md S1 invariant 3).
//   4. basic_rw_smoke            -- small directed R/W set; run under
//      BOTH build variants (see Makefile tb-l2c-chain /
//      tb-l2c-chain-off) to prove the harness/wiring itself is correct
//      with l2c in vs. entirely absent from the chain
//      (CHAIN_L2C_ENABLE Verilog parameter, tb_l2c_chain.v).
//
// Two build variants share this file (Makefile tb-l2c-chain /
// tb-l2c-chain-off), mirroring tb_l2c.cpp's BYPASS_ALL_BUILD pattern:
//   default:        CHAIN_L2C_ENABLE=1 (l2c really in the chain).
//   CHAIN_OFF_BUILD: CHAIN_L2C_ENABLE=0 (l2c elaborated out entirely --
//                    mirrors fpga_top_ddr.vh's L2C_ENABLE-undefined
//                    path).  Runs only the l2c-agnostic scenarios
//                    (basic_rw_smoke + a smaller randomized pass) --
//                    there is no cache to exercise fill-overlap/reset-
//                    drain semantics against.

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
#include "Vtb_l2c_chain.h"
using DutT = Vtb_l2c_chain;

static DutT* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;
static bool g_unmatched_resp_seen = false;
static bool g_track_mig_reads = false;
static bool g_mig_return_seen = false;
static unsigned g_mig_ar_before_first_r = 0;
static unsigned g_mig_peak_read_q = 0;

static std::mt19937 rng(0xC0FFEE13u);

// ─── Address window / golden model ────────────────────────────────────────
// Kept well inside sim_mig_backend's directly-mapped 32 MiB span
// (BEATS_LOG2=20 in tb_l2c_chain.v) and clear of its ROM-aliasing special
// case (0x4000_0000+, rtl/board/sim_mig_backend.v addr_to_idx) -- an
// ordinary RAM-shaped working set, matching what l2c's default
// CACHEABLE_BASE/SIZE already covers with no bypass window enabled.
static constexpr uint64_t WORK_BASE = 0x0000'0000ULL;
static constexpr uint64_t WORK_SIZE = 8ULL * 1024 * 1024;  // 8 MB
static constexpr uint64_t LINE_BYTES = 64;

// Verilator's --x-initial fast zeroes uninitialized regs (project
// convention, see docs/agent_policy.md), matching sim_mig_backend's
// mem_lane[] arrays before any write -- golden[] starts at 0 to match,
// and "touched" tracks which bytes we've actually verified via a write
// so read checks of never-written bytes still correctly expect 0.
static std::vector<uint8_t> golden(WORK_SIZE, 0);

// Every 16 B beat this testbench has ever WRITTEN, so golden[] can be
// re-established after a reset.  See resync_golden_after_reset().
static std::set<uint64_t> g_written;

// ─── Clocking: two independent nets, matched-rate (1:1) tick ────────────
// Frequency-ratio CDC diversity (slow-to-fast / fast-to-slow) is already
// covered by tb_axi_async_bridge.cpp's own unit tb (per the T13 brief:
// "use the existing tb clocking idioms") -- this integration tb reuses
// that bridge as a black box and focuses on the L2C-specific concurrency/
// reset scenarios.  Matched-rate still drives two distinct clock nets
// through two distinct reset domains and the real async_fifo double-flop
// synchronizers -- a legitimate CDC exercise, not a single-clock shortcut.
static void eval() { dut->eval(); }

// Forward declarations -- defined below, after the BFM state structs
// they operate on.
void req_pump_issue();
void req_drive_comb();
void req_latch_edge();

static void tick() {
    dut->core_clk = 0; dut->mig_clk = 0; eval();
    req_pump_issue();
    req_drive_comb();
    eval();
    if (g_track_mig_reads) {
        if (dut->dbg_mig_read_q_count > g_mig_peak_read_q)
            g_mig_peak_read_q = dut->dbg_mig_read_q_count;
        if (dut->dbg_mig_ar_fire && !g_mig_return_seen)
            g_mig_ar_before_first_r++;
        if (dut->dbg_mig_r_fire)
            g_mig_return_seen = true;
    }
    req_latch_edge();
    dut->core_clk = 1; dut->mig_clk = 1; eval();
    sim_time++;
    eval();
}

// ─── s_axi requester (concurrent-outstanding AXI4 master) ────────────────
// Adapted verbatim from tb_l2c.cpp's req_* BFM -- tb_l2c_chain.v exposes
// the identical s_axi_* port shape (128b data / 6b ID / 32b addr) so the
// driver logic is unchanged; only the memory-model half (m_axi) is gone,
// replaced by the real chain inside the DUT.
struct WBurst {
    uint32_t id; uint64_t addr; int beats;
    std::vector<std::array<uint8_t,16>> data;
    std::vector<uint16_t> strb;
    int issue_beat = 0;
    bool aw_done = false;
    bool resp_ok = false, done = false;
};
struct RBurst {
    uint32_t id; uint64_t addr; int beats;
    std::vector<std::array<uint8_t,16>> got;
    uint8_t last_resp = 0;
    bool done = false;
};
static std::deque<std::shared_ptr<WBurst>> w_pending_issue;
static std::shared_ptr<WBurst>             w_issuing;
static std::map<uint32_t, std::shared_ptr<WBurst>> w_awaiting_b;
static std::map<uint32_t, std::shared_ptr<WBurst>> w_done;

static std::deque<std::shared_ptr<RBurst>> r_pending_issue;
static std::shared_ptr<RBurst>             r_issuing;
static std::map<uint32_t, std::deque<std::shared_ptr<RBurst>>> r_awaiting;
static std::map<uint32_t, std::shared_ptr<RBurst>> r_done;

static std::set<uint32_t> free_ids;
static uint32_t alloc_id() {
    if (free_ids.empty()) {
        fprintf(stderr, "FATAL: alloc_id() empty pool\n");
        std::abort();
    }
    uint32_t id = *free_ids.begin();
    free_ids.erase(free_ids.begin());
    return id;
}
static void init_ids() { for (uint32_t i = 1; i < 60; i++) free_ids.insert(i); }

static bool req_ready_gate(int pct_ready) {
    if (std::getenv("L2C_CHAIN_NO_BACKPRESSURE")) return true;
    return static_cast<int>(rng() % 100) < pct_ready;
}

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

void req_drive_comb() {
    if (w_issuing) {
        if (!w_issuing->aw_done) {
            dut->s_axi_awvalid = 1; dut->s_axi_awid = w_issuing->id; dut->s_axi_awaddr = w_issuing->addr;
            dut->s_axi_awlen = w_issuing->beats - 1; dut->s_axi_awsize = 4; dut->s_axi_awburst = 1;
        } else dut->s_axi_awvalid = 0;
        dut->s_axi_wvalid = 1;
        set_wide_bytes(dut->s_axi_wdata, w_issuing->data[w_issuing->issue_beat].data(), 16);
        dut->s_axi_wstrb = w_issuing->strb[w_issuing->issue_beat];
        dut->s_axi_wlast = (w_issuing->issue_beat == w_issuing->beats - 1) ? 1 : 0;
    } else { dut->s_axi_awvalid = 0; dut->s_axi_wvalid = 0; }

    if (r_issuing) {
        dut->s_axi_arvalid = 1; dut->s_axi_arid = r_issuing->id; dut->s_axi_araddr = r_issuing->addr;
        dut->s_axi_arlen = r_issuing->beats - 1; dut->s_axi_arsize = 4; dut->s_axi_arburst = 1;
    } else dut->s_axi_arvalid = 0;

    dut->s_axi_bready = req_ready_gate(85) ? 1 : 0;
    dut->s_axi_rready = req_ready_gate(85) ? 1 : 0;
}

void req_pump_issue() {
    if (!w_issuing && !w_pending_issue.empty()) { w_issuing = w_pending_issue.front(); w_pending_issue.pop_front(); }
    if (!r_issuing && !r_pending_issue.empty()) { r_issuing = r_pending_issue.front(); r_pending_issue.pop_front(); }
}

void req_latch_edge() {
    if (w_issuing) {
        if (!w_issuing->aw_done && dut->s_axi_awvalid && dut->s_axi_awready) {
            w_issuing->aw_done = true;
        }
        if (w_issuing->aw_done && dut->s_axi_wvalid && dut->s_axi_wready) {
            w_issuing->issue_beat++;
            if (w_issuing->issue_beat == w_issuing->beats) {
                w_awaiting_b[w_issuing->id] = w_issuing;
                w_issuing = nullptr;
            }
        }
    }
    if (r_issuing && dut->s_axi_arvalid && dut->s_axi_arready) {
        r_awaiting[r_issuing->id].push_back(r_issuing);
        r_issuing = nullptr;
    }
    if (dut->s_axi_bvalid && dut->s_axi_bready) {
        uint32_t id = dut->s_axi_bid;
        auto it = w_awaiting_b.find(id);
        if (it != w_awaiting_b.end()) {
            it->second->resp_ok = (dut->s_axi_bresp == 0);
            it->second->done = true;
            w_done[id] = it->second;
            w_awaiting_b.erase(it);
        } else {
            printf("  UNEXPECTED B for id=%u bresp=%u t=%llu -- protocol violation\n",
                   id, (unsigned)dut->s_axi_bresp, (unsigned long long)sim_time);
            g_unmatched_resp_seen = true;
        }
    }
    if (dut->s_axi_rvalid && dut->s_axi_rready) {
        uint32_t id = dut->s_axi_rid;
        auto it = r_awaiting.find(id);
        bool have_entry = (it != r_awaiting.end()) && !it->second.empty();
        if (!have_entry) {
            printf("  UNEXPECTED R for id=%u t=%llu -- protocol violation\n",
                   id, (unsigned long long)sim_time);
            g_unmatched_resp_seen = true;
        } else {
            auto& front = it->second.front();
            std::array<uint8_t,16> beat{};
            get_wide_bytes(dut->s_axi_rdata, beat.data(), 16);
            front->got.push_back(beat);
            front->last_resp = dut->s_axi_rresp;
            if (dut->s_axi_rlast) {
                front->done = true;
                r_done[id] = front;
                it->second.pop_front();
                if (it->second.empty()) r_awaiting.erase(it);
            }
        }
    }
}

// ─── Reset: mig side calibrates first (independent counter), then core
//     side releases and l2c's own reset-walk (if in the chain) runs. ────
static void reset_dut(bool core_only = false) {
    if (!core_only) {
        dut->mig_rst = 1; dut->mig_clk = 0; dut->core_clk = 0;
    }
    dut->core_rst = 1;
    dut->s_axi_awvalid = 0; dut->s_axi_wvalid = 0; dut->s_axi_arvalid = 0;
    dut->s_axi_bready = 1; dut->s_axi_rready = 1;
    eval();
    for (int i = 0; i < 8; i++) tick();
    if (!core_only) {
        dut->mig_rst = 0;
        int waited = 0;
        while (dut->cal_done == 0 && waited < 200) { tick(); waited++; }
    }
    dut->core_rst = 0;
    int waited = 0;
    while (dut->s_axi_awready == 0 && dut->s_axi_arready == 0 && waited < 8000) { tick(); waited++; }
}

// ─── High-level helpers ────────────────────────────────────────────────
static std::array<uint8_t,16> mkbeat(uint64_t addr) {
    std::array<uint8_t,16> b{};
    for (int i = 0; i < 16; i++) b[i] = golden[addr + i - WORK_BASE];
    return b;
}
static uint32_t issue_read(uint64_t addr, int beats = 1) {
    auto rb = std::make_shared<RBurst>();
    rb->id = alloc_id(); rb->addr = addr; rb->beats = beats;
    r_pending_issue.push_back(rb);
    return rb->id;
}
static uint32_t issue_write(uint64_t addr, const std::vector<std::array<uint8_t,16>>& data) {
    auto wb = std::make_shared<WBurst>();
    wb->id = alloc_id(); wb->addr = addr; wb->beats = (int)data.size();
    wb->data = data;
    wb->strb.assign(data.size(), 0xFFFF);
    w_pending_issue.push_back(wb);
    return wb->id;
}
static bool wait_read_done(uint32_t id, int max_cycles = 40000) {
    int c = 0;
    while (r_done.find(id) == r_done.end() && c < max_cycles) { tick(); c++; }
    return r_done.find(id) != r_done.end();
}
static bool wait_write_done(uint32_t id, int max_cycles = 40000) {
    int c = 0;
    while (w_done.find(id) == w_done.end() && c < max_cycles) { tick(); c++; }
    return w_done.find(id) != w_done.end();
}
static void free_read(uint32_t id) { r_done.erase(id); free_ids.insert(id); }
static void free_write(uint32_t id) { w_done.erase(id); free_ids.insert(id); }

static bool do_write_apply(uint64_t addr, uint8_t seed) {
    std::array<uint8_t,16> beat{};
    for (int i = 0; i < 16; i++) beat[i] = (uint8_t)(seed + i);
    uint32_t id = issue_write(addr, {beat});
    if (!wait_write_done(id)) { printf("  write timeout addr=0x%llx\n", (unsigned long long)addr); return false; }
    bool ok = w_done[id]->resp_ok;
    free_write(id);
    if (ok) { for (int i = 0; i < 16; i++) golden[addr - WORK_BASE + i] = beat[i];
              g_written.insert(addr); }
    return ok;
}
static bool do_read_check(uint64_t addr) {
    uint32_t id = issue_read(addr);
    if (!wait_read_done(id)) { printf("  read timeout addr=0x%llx\n", (unsigned long long)addr); return false; }
    auto rb = r_done[id];
    bool match = (rb->got.size() == 1) && (rb->got[0] == mkbeat(addr));
    if (!match) {
        printf("  MISMATCH addr=0x%llx got=", (unsigned long long)addr);
        for (auto b : rb->got[0]) printf("%02x", b);
        printf(" want=");
        for (auto b : mkbeat(addr)) printf("%02x", b);
        printf("\n");
    }
    free_read(id);
    return match;
}

#define CHECK(name, cond) do { \
    if (!(cond)) { printf("  FAIL %s\n", name); return false; } \
} while (0)

// ═══════════════════════════════════════════════════════════════════════
// Scenario 4: basic_rw_smoke -- runs under BOTH build variants.
// ═══════════════════════════════════════════════════════════════════════
static bool test_basic_rw_smoke() {
    reset_dut();
    bool ok = true;
    for (int i = 0; i < 32; i++) {
        uint64_t addr = WORK_BASE + (uint64_t)i * LINE_BYTES;
        ok &= do_write_apply(addr, (uint8_t)(i * 7 + 1));
    }
    for (int i = 0; i < 32; i++) {
        uint64_t addr = WORK_BASE + (uint64_t)i * LINE_BYTES;
        ok &= do_read_check(addr);
    }
    // Read-modify-write / re-read to exercise a hit path (when l2c is in
    // the chain) or a plain pass-through re-read (when it isn't).
    for (int i = 0; i < 32; i++) {
        uint64_t addr = WORK_BASE + (uint64_t)i * LINE_BYTES;
        ok &= do_read_check(addr);
    }
    return ok && !g_unmatched_resp_seen;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 1: randomized_chain_traffic -- >=5000 mixed ops, concurrent
// outstanding, backpressure (default 85% ready on both sides), STALL_
// ENABLE mig-side command stalls (compiled into the DUT, see Makefile).
// ═══════════════════════════════════════════════════════════════════════
// ── golden[] MUST be re-established across a reset ──────────────────────
// (2026-08-20.)  l2c DROPS DIRTY LINES ON RESET by design
// (docs/l2c_spec.md S7), so anything this testbench wrote that was still
// dirty in the cache when reset_dut() fired is gone and DRAM holds the
// pre-write value.  golden[] cannot be carried across that boundary; the
// only thing that survives is what the DUT itself now returns.  tb_l2c.cpp
// does the same thing after its reset-walk scenario (see its "resync
// golden[] to backing[]" comment) -- this tb was missing the equivalent.
//
// This was a LATENT bug, not a new one: basic_rw_smoke leaves 32 lines
// dirty and unflushed, test_randomized_chain_traffic then resets, and the
// suite only failed if the randomized address stream happened to READ one
// of those 32 lines before writing it -- roughly a 1-in-6 chance, and the
// stream depends on completion timing because the backpressure gate draws
// from the same RNG.  Demonstrated directly on unmodified RTL: a forced
// post-reset read of WORK_BASE+0x80 returns 00 while golden[] claims 0f.
static void resync_golden_after_reset() {
    for (uint64_t a : g_written) {
        uint32_t id = issue_read(a);
        if (!wait_read_done(id)) { printf("  resync timeout addr=0x%llx\n", (unsigned long long)a); continue; }
        auto rb = r_done[id];
        if (rb->got.size() == 1)
            for (int b = 0; b < 16; b++) golden[a - WORK_BASE + b] = rb->got[0][b];
        free_read(id);
    }
}

static bool test_randomized_chain_traffic(int n_ops) {
    reset_dut();
    resync_golden_after_reset();
    bool ok = true;
    int in_flight = 0;
    const int MAX_IN_FLIGHT = 16; // id pool is 59-wide; keep headroom
    int issued = 0, completed = 0;
    // Track outstanding ops so we can drain+verify as they finish rather
    // than blocking fully serially (keeps concurrency real).
    struct Outstanding { bool is_write; uint32_t id; uint64_t addr; uint8_t seed; };
    std::vector<Outstanding> pend;

    auto drain_one_if_ready = [&]() -> bool {
        for (size_t i = 0; i < pend.size(); i++) {
            auto& o = pend[i];
            if (o.is_write) {
                auto it = w_done.find(o.id);
                if (it != w_done.end()) {
                    bool good = it->second->resp_ok;
                    if (good) { for (int b = 0; b < 16; b++) golden[o.addr - WORK_BASE + b] = (uint8_t)(o.seed + b);
                                g_written.insert(o.addr); }
                    free_write(o.id);
                    pend.erase(pend.begin() + i);
                    completed++;
                    return good;
                }
            } else {
                auto it = r_done.find(o.id);
                if (it != r_done.end()) {
                    bool good = it->second->got.size() == 1 && it->second->got[0] == mkbeat(o.addr);
                    if (!good) {
                        printf("  MISMATCH (randomized) addr=0x%llx\n", (unsigned long long)o.addr);
                    }
                    free_read(o.id);
                    pend.erase(pend.begin() + i);
                    completed++;
                    return good;
                }
            }
        }
        return true;
    };

    int guard = n_ops * 400 + 200000;
    while ((issued < n_ops || !pend.empty()) && guard-- > 0) {
        while (issued < n_ops && (int)pend.size() < MAX_IN_FLIGHT && free_ids.size() > 4) {
            uint64_t line = (uint64_t)(rng() % (WORK_SIZE / LINE_BYTES));
            uint64_t addr = WORK_BASE + line * LINE_BYTES + (rng() % 4) * 16; // random 16B beat within the line
            bool is_write = (rng() % 2) == 0;
            if (is_write) {
                uint8_t seed = (uint8_t)(rng() & 0xFF);
                std::array<uint8_t,16> beat{};
                for (int b = 0; b < 16; b++) beat[b] = (uint8_t)(seed + b);
                uint32_t id = issue_write(addr, {beat});
                pend.push_back({true, id, addr, seed});
            } else {
                uint32_t id = issue_read(addr);
                pend.push_back({false, id, addr, 0});
            }
            issued++;
        }
        tick();
        ok &= drain_one_if_ready();
        if (!ok) break;
    }
    while (!pend.empty() && guard-- > 0) { tick(); ok &= drain_one_if_ready(); }
    CHECK("all ops completed", completed == n_ops || !ok);
    printf("  randomized_chain_traffic: issued=%d completed=%d\n", issued, completed);
    return ok && !g_unmatched_resp_seen && completed == n_ops;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2: concurrent_fill_overlap -- 8 back-to-back miss reads
// (issued without waiting for completion) vs. 8 sequential (issue+wait)
// reads to FRESH lines, proving MSHR fills overlap across the CDC (the
// whole point of T11's multi-outstanding bridge rework, now exercised
// through l2c's own 8-entry MSHR table on top of it).
// ═══════════════════════════════════════════════════════════════════════
static bool test_concurrent_fill_overlap() {
    reset_dut();
    // Warm up (unrelated lines) so the very first measurement isn't
    // paying for the reset-walk's residual settle.
    do_write_apply(WORK_BASE, 0xAA);
    do_read_check(WORK_BASE);

    // -- Concurrent: 8 distinct fresh lines, issued back-to-back. --
    const int N = 8;
    uint64_t base_concurrent = WORK_BASE + 4096 * LINE_BYTES;
    uint64_t t0 = sim_time;
    g_mig_return_seen = false;
    g_mig_ar_before_first_r = 0;
    g_mig_peak_read_q = 0;
    g_track_mig_reads = true;
    std::vector<uint32_t> ids;
    for (int i = 0; i < N; i++) ids.push_back(issue_read(base_concurrent + (uint64_t)i * LINE_BYTES));
    bool ok = true;
    for (auto id : ids) { ok &= wait_read_done(id); free_read(id); }
    g_track_mig_reads = false;
    uint64_t concurrent_cycles = sim_time - t0;

    // -- Sequential: 8 more distinct fresh lines, issue+wait each. --
    uint64_t base_seq = WORK_BASE + 8192 * LINE_BYTES;
    uint64_t t1 = sim_time;
    for (int i = 0; i < N; i++) {
        uint32_t id = issue_read(base_seq + (uint64_t)i * LINE_BYTES);
        ok &= wait_read_done(id);
        free_read(id);
    }
    uint64_t sequential_cycles = sim_time - t1;

    printf("  concurrent_fill_overlap: 8 concurrent=%llu cycles, 8 sequential=%llu cycles "
           "(speedup %.2fx), MIG queue peak=%u, ARs before first R=%u\n",
           (unsigned long long)concurrent_cycles, (unsigned long long)sequential_cycles,
           sequential_cycles > 0 ? (double)sequential_cycles / (double)concurrent_cycles : 0.0,
           g_mig_peak_read_q, g_mig_ar_before_first_r);

    CHECK("no protocol violations", !g_unmatched_resp_seen);
    CHECK("all 8 MIG reads accepted before the first reply", g_mig_ar_before_first_r == N);
    CHECK("MIG read queue reached all 8 occupied entries", g_mig_peak_read_q == N);
    CHECK("concurrent issue is faster than 8x sequential", concurrent_cycles < sequential_cycles);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 3: reset_mid_traffic -- core_rst asserted with fills/
// writebacks in flight; mig_clk/mig_rst keep running throughout (models
// a core-domain-only reset, e.g. JTAG debug reset, while DDR/MIG stay
// live).  l2c's reset drain + the async bridge's own coupled-reset
// handshake (T6 hardening) compose; chain must recover, and FRESH
// post-reset writes/reads must be correct (pre-reset dirty data is
// expected lost per docs/l2c_spec.md S1 invariant 3 -- not checked here).
// ═══════════════════════════════════════════════════════════════════════
static bool test_reset_mid_traffic() {
    reset_dut();
    // Seed a dirty line, then evict it via other traffic while immediately
    // starting a fresh fill -- fills and (potentially) a victim writeback
    // both mid-flight when reset hits.
    do_write_apply(WORK_BASE, 0x11);
    for (int i = 1; i < 10; i++) do_write_apply(WORK_BASE + (uint64_t)i * LINE_BYTES, (uint8_t)(0x20 + i));

    // Kick off several fresh-line reads (fills) and a write (potential
    // future writeback) without waiting for any of them.
    std::vector<uint32_t> rids;
    for (int i = 0; i < 4; i++) rids.push_back(issue_read(WORK_BASE + (uint64_t)(20 + i) * LINE_BYTES));
    uint32_t wid = issue_write(WORK_BASE + 30 * LINE_BYTES, {mkbeat(0)});
    (void)wid;
    for (int i = 0; i < 5; i++) tick();  // let some of this get in-flight inside l2c/bridge

    // Core-only reset: core_rst asserts/deasserts; mig_clk/mig_rst are
    // left running (reset_dut(core_only=true) skips touching mig_rst).
    reset_dut(/*core_only=*/true);

    // Drop tracking for whatever was in flight pre-reset -- their
    // responses (if any ever arrive) are not meaningful post-reset.
    w_pending_issue.clear(); r_pending_issue.clear();
    w_issuing = nullptr; r_issuing = nullptr;
    w_awaiting_b.clear(); r_awaiting.clear();
    w_done.clear(); r_done.clear();
    free_ids.clear(); init_ids();
    g_unmatched_resp_seen = false;  // pre-reset in-flight responses, if any trickle in during the reset window, are expected and not a protocol violation
    for (int i = 0; i < 50; i++) tick();
    g_unmatched_resp_seen = false;  // re-clear after the settle window too

    // Fresh writes+reads must now be correct.
    bool ok = true;
    for (int i = 0; i < 16; i++) {
        uint64_t addr = WORK_BASE + (uint64_t)(50 + i) * LINE_BYTES;
        ok &= do_write_apply(addr, (uint8_t)(0x50 + i));
    }
    for (int i = 0; i < 16; i++) {
        uint64_t addr = WORK_BASE + (uint64_t)(50 + i) * LINE_BYTES;
        ok &= do_read_check(addr);
    }
    CHECK("post-reset traffic clean", !g_unmatched_resp_seen);
    return ok;
}

// ─── Runner ────────────────────────────────────────────────────────────
static void run(const char* name, bool (*fn)()) {
    g_unmatched_resp_seen = false;
    bool r = fn();
    printf("[%s] %s\n", r ? "PASS" : "FAIL", name);
    if (r) n_pass++; else n_fail++;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new DutT();
    init_ids();

    run("basic_rw_smoke", test_basic_rw_smoke);
#ifdef CHAIN_OFF_BUILD
    // No l2c in the chain -- no fills/MSHR/reset-drain to exercise;
    // still worth proving the harness pushes a decent volume of traffic
    // cleanly straight through the CDC + MIG bridge + sim backend.
    run("randomized_chain_traffic_small", []() { return test_randomized_chain_traffic(500); });
#else
    run("randomized_chain_traffic", []() { return test_randomized_chain_traffic(5000); });
    run("concurrent_fill_overlap", test_concurrent_fill_overlap);
    run("reset_mid_traffic", test_reset_mid_traffic);
#endif

    printf("\ntb_l2c_chain: %d PASS / %d FAIL\n", n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
