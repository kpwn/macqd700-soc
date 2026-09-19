// tb_vram_ddr_chain.cpp -- VRAM-in-DDR integration testbench (T14,
// reshaped by T16 "decode-level VRAM lane") for the REAL chain:
//   axi_xbar (VRAM_IN_DDR; S3 genuine slave again) -> S3 ->
//     [address-translate] -> axi_vram_priority_mux3 (3-way: l2c-path
//     [when CHAIN_L2C_ENABLE] <-> S3/VRAM-aperture lane <-> scanout)
//     -> axi_async_bridge -> axi_ddr4_mig_bridge -> sim_mig_backend
// via tb/tb_vram_ddr_chain.v.
//
// This does NOT re-derive l2c's own cache-correctness suite (tb-l2c) or
// axi_xbar's own general arbitration suite (tb-axi-xbar) -- it proves the
// integration seam specifically:
//
//   1. cpu_write_scanout_read_roundtrip -- CPU writes a known byte
//      pattern to the VRAM aperture through the REAL axi_xbar (proving
//      the S3 zero-base + this tb's own address-translate to the
//      carveout), reads it back via the CPU AXI path (must match
//      original bytes -- the write-side and read-side byte swaps are
//      inverses of each other, an involution) AND via the scanout
//      streaming port (must match the VRAM-native swapped byte order --
//      same convention tb_framebuffer_pixel proves for the
//      VRAM_IN_DDR=off path).
//   2. concurrent_streaming -- CPU read/write traffic (both the
//      VRAM-aperture/S3-lane path AND, separately, the l2c/RAM path) and
//      scanout streaming reads run CONCURRENTLY through the shared
//      3-way arbiter; verifies all traffic classes complete correctly,
//      measures/prints the starvation bound under sustained scanout load
//      SEPARATELY for l2c (RAM) traffic and for CPU FB (S3-lane) writes
//      -- proving the "scanout always wins, but l2c/S3 traffic is never
//      starved outright" priority contract for both victim classes --
//      and measures/prints the scanout underrun margin at a computed
//      1080p8-equivalent fetch rate (with sim_mig_backend STALL_ENABLE=1
//      jitter).
//   3. reset_mid_stream -- core_rst asserted with CPU traffic and a
//      scanout line-fetch both in flight; the mux/chain must not wedge,
//      and fresh post-reset traffic on both paths must be correct.
//   4. raw_write_then_scan_read_same_line (T16, new) -- a CPU write to a
//      line immediately followed by a scanout request for that SAME
//      line must observe the NEW data, never stale pre-write bytes, even
//      though the write and the read take DIFFERENT physical paths
//      through the 3-way arbiter (S3 lane vs. scanout) that could in
//      principle race. Ordering is NOT enforced by any new machinery in
//      the arbiter/lane -- it is guaranteed further downstream, in the
//      ONE shared `axi_ddr4_mig_bridge.v` both paths funnel through: its
//      same-64B-line hazard check (`rd_hazard`/`wr_hazard_hit`, see that
//      file's "same-64B-line hazard check (review C2/I3)" section) holds
//      a queued read's AR back from the MIG for as long as any
//      outstanding write's 64B line overlaps it, GLOBALLY across every
//      upstream master's traffic -- it has no notion of "which master"
//      issued which op, so it applies identically whether the write and
//      read arrived via the same arbiter grant or different ones. This
//      test exists to VERIFY that global property actually reaches this
//      seam's traffic shape, not to add a second ordering mechanism.
//   5. s3_flush_abort_no_wedge (T16, new) -- axi_xbar's existing
//      slv_flush machinery (S3 has always been a `is_flush_domain_slv()`
//      member, unaffected by T16's revert of the S3->S0 decode fold)
//      aborts an S3 op mid-flight; the late backend response must be
//      dropped by the xbar's existing poison/unroutability machinery
//      with no wedge, and the lane must be usable again afterward.
//
// Build variants (Makefile tb-vram-ddr-chain / tb-vram-ddr-chain-nol2c):
//   default:  CHAIN_L2C_ENABLE=1 (l2c genuinely in the RAM/ROM/FB path;
//             matches the intended production L2C_ENABLE+VRAM_IN_DDR
//             combined build -- VRAM-aperture traffic never reaches l2c
//             either way, see axi_vram_priority_mux3.v's header).
//   NOL2C:    CHAIN_L2C_ENABLE=0 (xbar S0 -> mux's l2c_* port directly,
//             matching VRAM_IN_DDR alone without L2C_ENABLE).

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
#include "Vtb_vram_ddr_chain.h"
#include "Vtb_vram_ddr_chain___024root.h"
using DutT = Vtb_vram_ddr_chain;

static DutT* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;
static bool g_unmatched_resp_seen = false;

static std::mt19937 rng(0x71E14DDA);

// ─── Address window / carveout geometry ────────────────────────────────
static constexpr uint32_t VRAM_APERTURE_BASE = 0xF9000000u;
static constexpr uint32_t TEST_SPAN = 8192; // bytes/pixels exercised (well inside the 2 MiB aperture / 32 MiB carveout)
// Scanout ring LINE size -- must track scanout_line_fetch.v's LINE_BYTES
// (= 1 << scanout_ddr_reader.v's LINE_OFF_W).  Raised 64 -> 128 when the
// reader went to 8-beat bursts.  Scenario 2c below derives its
// NUM_LINE_BUF+1 deadlock boundary from this, so a stale value here would
// silently stop probing the boundary it names.  Unrelated to the "same-64B
// line" hazard granularity in axi_ddr4_mig_bridge.v (see this file's header)
// -- that is a different structure and stays 64 B.
static constexpr uint32_t LINE_BYTES = 128;
// A RAM address, well away from anything else this tb touches -- used to
// drive genuine l2c-path (xbar S0) traffic alongside VRAM-aperture (S3
// lane) and scanout traffic in the concurrent_streaming / RAW-ordering
// scenarios. l2c-path traffic never touches `stored[]`/the VRAM byte
// swap -- it's plain RAM, checked by simple readback-equals-written.
static constexpr uint32_t RAM_TRAFFIC_BASE = 0x00010000u;

// `stored[]` mirrors exactly what's physically in DRAM after a CPU write
// (i.e. AFTER the xbar's write-side byte swap) -- this is the golden
// model BOTH the scanout read-back (compared directly) and the CPU
// read-back (compared after re-applying the same swap, since the
// transform is its own inverse) are checked against.
static std::vector<uint8_t> stored(TEST_SPAN, 0);

static void swap32_lane(uint8_t* b) {
    uint8_t t0 = b[0], t1 = b[1], t2 = b[2], t3 = b[3];
    b[0] = t3; b[1] = t2; b[2] = t1; b[3] = t0;
}
static void vram_swap16(uint8_t* beat /* 16 bytes, in place */) {
    for (int lane = 0; lane < 4; lane++) swap32_lane(beat + lane * 4);
}

static void eval() { dut->eval(); }

void req_pump_issue();
void req_drive_comb();
void req_latch_edge();
void scan_drive_and_latch_pre();
void scan_drive_and_latch_post();

static bool arb_monitor_enable = false;
static bool hold_mig_clock = false;
static uint32_t arb_max_bulk = 0;
static int arb_bulk_at_scan_ar = -1;
static uint64_t arb_scan_ar_time = 0;
static unsigned mig_ratio_phase = 0;

static void mig_cycle() {
    if (hold_mig_clock) return;
    dut->mig_clk = 1; eval();
    dut->mig_clk = 0; eval();
}

static void tick() {
    dut->core_clk = 0; dut->mig_clk = 0; eval();
    // Production crosses 100 MHz core traffic into the 333.25 MHz MIG UI
    // domain. A repeating 3/3/4 cadence exercises the async FIFOs with
    // distinct clock edges instead of making CDC logic behave synchronously.
    const unsigned mig_cycles = (++mig_ratio_phase == 3) ? 4 : 3;
    if (mig_ratio_phase == 3) mig_ratio_phase = 0;
    for (unsigned i = 0; i < mig_cycles; i++) mig_cycle();
    req_pump_issue();
    req_drive_comb();
    scan_drive_and_latch_pre();
    eval();
    if (arb_monitor_enable) {
        arb_max_bulk = std::max(arb_max_bulk, (uint32_t)dut->dbg_mux_bulk_count);
        if (dut->dbg_scan_ar_fire && arb_bulk_at_scan_ar < 0) {
            arb_bulk_at_scan_ar = dut->dbg_mux_bulk_count;
            arb_scan_ar_time = sim_time;
        }
    }
    req_latch_edge();
    scan_drive_and_latch_post();
    dut->core_clk = 1; dut->mig_clk = 0; eval();
    sim_time++;
    eval();
}

// ─── CPU AXI requester (concurrent-outstanding, adapted from tb_l2c_chain) ─
struct WBurst {
    uint32_t id; uint64_t addr; int beats;
    uint8_t awsize = 4;
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
    if (free_ids.empty()) { fprintf(stderr, "FATAL: alloc_id() empty pool\n"); std::abort(); }
    uint32_t id = *free_ids.begin();
    free_ids.erase(free_ids.begin());
    return id;
}
static void init_ids() { for (uint32_t i = 1; i < 14; i++) free_ids.insert(i); } // ID_WIDTH=4 on the cpu_* BFM port

static bool req_ready_gate(int pct_ready) {
    if (std::getenv("VRAM_CHAIN_NO_BACKPRESSURE")) return true;
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
            dut->cpu_awvalid = 1; dut->cpu_awid = w_issuing->id; dut->cpu_awaddr = w_issuing->addr;
            dut->cpu_awlen = w_issuing->beats - 1; dut->cpu_awsize = w_issuing->awsize; dut->cpu_awburst = 1;
        } else dut->cpu_awvalid = 0;
        dut->cpu_wvalid = 1;
        set_wide_bytes(dut->cpu_wdata, w_issuing->data[w_issuing->issue_beat].data(), 16);
        dut->cpu_wstrb = w_issuing->strb[w_issuing->issue_beat];
        dut->cpu_wlast = (w_issuing->issue_beat == w_issuing->beats - 1) ? 1 : 0;
    } else { dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; }

    if (r_issuing) {
        dut->cpu_arvalid = 1; dut->cpu_arid = r_issuing->id; dut->cpu_araddr = r_issuing->addr;
        dut->cpu_arlen = r_issuing->beats - 1; dut->cpu_arsize = 4; dut->cpu_arburst = 1;
    } else dut->cpu_arvalid = 0;

    dut->cpu_bready = req_ready_gate(85) ? 1 : 0;
    dut->cpu_rready = req_ready_gate(85) ? 1 : 0;
}

void req_pump_issue() {
    if (!w_issuing && !w_pending_issue.empty()) { w_issuing = w_pending_issue.front(); w_pending_issue.pop_front(); }
    if (!r_issuing && !r_pending_issue.empty()) { r_issuing = r_pending_issue.front(); r_pending_issue.pop_front(); }
}

void req_latch_edge() {
    if (w_issuing) {
        if (!w_issuing->aw_done && dut->cpu_awvalid && dut->cpu_awready) w_issuing->aw_done = true;
        if (w_issuing->aw_done && dut->cpu_wvalid && dut->cpu_wready) {
            w_issuing->issue_beat++;
            if (w_issuing->issue_beat == w_issuing->beats) {
                w_awaiting_b[w_issuing->id] = w_issuing;
                w_issuing = nullptr;
            }
        }
    }
    if (r_issuing && dut->cpu_arvalid && dut->cpu_arready) {
        r_awaiting[r_issuing->id].push_back(r_issuing);
        r_issuing = nullptr;
    }
    if (dut->cpu_bvalid && dut->cpu_bready) {
        uint32_t id = dut->cpu_bid;
        auto it = w_awaiting_b.find(id);
        if (it != w_awaiting_b.end()) {
            it->second->resp_ok = (dut->cpu_bresp == 0);
            it->second->done = true;
            w_done[id] = it->second;
            w_awaiting_b.erase(it);
        } else {
            printf("  UNEXPECTED B for id=%u t=%llu\n", id, (unsigned long long)sim_time);
            g_unmatched_resp_seen = true;
        }
    }
    if (dut->cpu_rvalid && dut->cpu_rready) {
        uint32_t id = dut->cpu_rid;
        auto it = r_awaiting.find(id);
        bool have_entry = (it != r_awaiting.end()) && !it->second.empty();
        if (!have_entry) {
            printf("  UNEXPECTED R for id=%u t=%llu\n", id, (unsigned long long)sim_time);
            g_unmatched_resp_seen = true;
        } else {
            auto& front = it->second.front();
            std::array<uint8_t,16> beat{};
            get_wide_bytes(dut->cpu_rdata, beat.data(), 16);
            front->got.push_back(beat);
            front->last_resp = dut->cpu_rresp;
            if (dut->cpu_rlast) {
                front->done = true;
                r_done[id] = front;
                it->second.pop_front();
                if (it->second.empty()) r_awaiting.erase(it);
            }
        }
    }
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
static uint32_t issue_write_shape(uint64_t addr, const std::array<uint8_t,16>& data,
                                  uint16_t strb, uint8_t awsize) {
    auto wb = std::make_shared<WBurst>();
    wb->id = alloc_id(); wb->addr = addr; wb->beats = 1; wb->awsize = awsize;
    wb->data = {data}; wb->strb = {strb};
    w_pending_issue.push_back(wb);
    return wb->id;
}
static bool wait_read_done(uint32_t id, int max_cycles = 40000) {
    int c = 0; while (r_done.find(id) == r_done.end() && c < max_cycles) { tick(); c++; }
    return r_done.find(id) != r_done.end();
}
static bool wait_write_done(uint32_t id, int max_cycles = 40000) {
    int c = 0; while (w_done.find(id) == w_done.end() && c < max_cycles) { tick(); c++; }
    return w_done.find(id) != w_done.end();
}
static void free_read(uint32_t id) { r_done.erase(id); free_ids.insert(id); }
static void free_write(uint32_t id) { w_done.erase(id); free_ids.insert(id); }

static bool do_write_apply(uint32_t ap_off, uint8_t seed) {
    std::array<uint8_t,16> beat{};
    for (int i = 0; i < 16; i++) beat[i] = (uint8_t)(seed + i);
    uint32_t id = issue_write(VRAM_APERTURE_BASE + ap_off, {beat});
    if (!wait_write_done(id)) { printf("  write timeout off=0x%x\n", ap_off); return false; }
    bool ok = w_done[id]->resp_ok;
    free_write(id);
    if (ok) {
        std::array<uint8_t,16> swapped = beat;
        vram_swap16(swapped.data());
        for (int i = 0; i < 16; i++) stored[ap_off + i] = swapped[i];
    }
    return ok;
}
static bool do_read_check(uint32_t ap_off) {
    uint32_t id = issue_read(VRAM_APERTURE_BASE + ap_off);
    if (!wait_read_done(id)) { printf("  read timeout off=0x%x\n", ap_off); return false; }
    auto rb = r_done[id];
    std::array<uint8_t,16> expect{};
    for (int i = 0; i < 16; i++) expect[i] = stored[ap_off + i];
    vram_swap16(expect.data()); // read-side re-applies the same swap -- involution -> original bytes
    bool match = (rb->got.size() == 1) && (rb->got[0] == expect);
    if (!match) {
        printf("  CPU READBACK MISMATCH off=0x%x got=", ap_off);
        for (auto b : rb->got[0]) printf("%02x", b);
        printf(" want=");
        for (auto b : expect) printf("%02x", b);
        printf("\n");
    }
    free_read(id);
    return match;
}

// ─── RAM (l2c-path) traffic helpers ─────────────────────────────────────
// Plain RAM: no byte swap, no `stored[]` involvement -- these exercise
// the l2c_* side of axi_vram_priority_mux3 (xbar S0 -> l2c, when
// CHAIN_L2C_ENABLE -- or S0 straight through otherwise), never the S3
// lane.
static bool do_ram_write_apply(uint32_t ram_off, uint8_t seed, uint64_t* lat_out = nullptr) {
    std::array<uint8_t,16> beat{};
    for (int i = 0; i < 16; i++) beat[i] = (uint8_t)(seed + i);
    uint64_t t0 = sim_time;
    uint32_t id = issue_write((uint64_t)ram_off, {beat});
    if (!wait_write_done(id)) { printf("  ram write timeout off=0x%x\n", ram_off); return false; }
    if (lat_out) *lat_out = sim_time - t0;
    bool ok = w_done[id]->resp_ok;
    free_write(id);
    return ok;
}
static bool do_ram_read_check(uint32_t ram_off, const std::array<uint8_t,16>& expect, uint64_t* lat_out = nullptr) {
    uint64_t t0 = sim_time;
    uint32_t id = issue_read((uint64_t)ram_off);
    if (!wait_read_done(id)) { printf("  ram read timeout off=0x%x\n", ram_off); return false; }
    if (lat_out) *lat_out = sim_time - t0;
    auto rb = r_done[id];
    bool match = (rb->got.size() == 1) && (rb->got[0] == expect);
    if (!match) {
        printf("  RAM READBACK MISMATCH off=0x%x\n", ram_off);
    }
    free_read(id);
    return match;
}

// ─── Scanout streaming BFM ──────────────────────────────────────────────
// scanout_ddr_reader has no backpressure signal on its streaming port
// (matches vram.v) -- every asserted scan_rd_en is accepted; responses
// arrive in order with variable latency via scan_rd_valid.
static std::deque<uint32_t> scan_pending;      // offsets awaiting a response, in order
static std::deque<uint8_t>  scan_got;           // received bytes, in order
static uint64_t scan_issued = 0, scan_completed = 0;
static uint64_t scan_max_gap_cycles = 0;
static uint64_t scan_last_valid_time = 0;
static bool scan_have_last_valid_time = false;
static bool scan_driver_active = false;
static uint32_t scan_next_off = 0;
static uint32_t scan_stop_off = 0;
static bool scan_req_this_cycle = false;

// Matches fb_reader.v's real in_flight_cnt credit discipline (default
// INFLIGHT_WIDTH=4 => max 15 outstanding, see fb_reader.v's header) --
// the real consumer NEVER pushes unboundedly; this driver shouldn't
// either, or it can legitimately overrun scanout_ddr_reader's request
// queue on a latency spike (e.g. right after a reset) that a real,
// credit-throttled consumer would simply never produce.
static constexpr size_t SCAN_MAX_OUTSTANDING = 15;

void scan_drive_and_latch_pre() {
    scan_req_this_cycle = scan_driver_active && (scan_next_off < scan_stop_off) &&
                          (scan_pending.size() < SCAN_MAX_OUTSTANDING);
    if (scan_req_this_cycle) {
        dut->scan_rd_en = 1;
        dut->scan_rd_addr = scan_next_off; // BPP=8 -> pixel index == byte offset
    } else {
        dut->scan_rd_en = 0;
    }
}
void scan_drive_and_latch_post() {
    if (scan_req_this_cycle) {
        scan_pending.push_back(scan_next_off);
        scan_next_off++;
        scan_issued++;
    }
    if (dut->scan_rd_valid) {
        if (scan_pending.empty()) {
            static int nunexp = 0;
            if (nunexp++ < 8) {
                printf("  UNEXPECTED scan_rd_valid with no pending request t=%llu data=%08x [dbg q_head=%u q_tail=%u q_count=%u f_state=%u issued=%llu completed=%llu]\n",
                       (unsigned long long)sim_time, (unsigned)dut->scan_rd_data,
                       dut->dbg_q_head, dut->dbg_q_tail, dut->dbg_q_count, dut->dbg_f_state,
                       (unsigned long long)scan_issued, (unsigned long long)scan_completed);
            }
            g_unmatched_resp_seen = true;
        } else {
            uint32_t off = scan_pending.front(); scan_pending.pop_front();
            // The port returns a 4-BYTE GROUP: [31:24] is the byte at `off`
            // (valid at any alignment), [23:0] the bytes at +1/+2/+3 (valid
            // only when off is 4-byte aligned).  This driver is a CONSUMER, so
            // the pixel byte is the top lane.
            const uint32_t grp = (uint32_t)dut->scan_rd_data;
            uint8_t got = (uint8_t)((grp >> 24) & 0xFFu);
            uint8_t want = stored[off];
            // Hold the widened port to its whole contract, not just the lane
            // an 8bpp consumer looks at: for a 4-byte-ALIGNED request all four
            // returned bytes must be stored[off..off+3].  These are exactly
            // the lanes 24bpp direct colour reads as R/G/B, and nothing else
            // in this tb would notice them being wrong.
            if ((off & 3u) == 0u && (off + 3u) < TEST_SPAN) {
                const uint32_t want_grp = ((uint32_t)stored[off + 0] << 24)
                                        | ((uint32_t)stored[off + 1] << 16)
                                        | ((uint32_t)stored[off + 2] <<  8)
                                        |  (uint32_t)stored[off + 3];
                if (grp != want_grp) {
                    static int nwide = 0;
                    if (nwide++ < 8) {
                        printf("  SCANOUT WIDE-GROUP MISMATCH off=0x%x got=%08x want=%08x\n",
                               off, grp, want_grp);
                    }
                    g_unmatched_resp_seen = true;
                }
            }
            if (got != want) {
                static int nmismatch = 0;
                if (nmismatch++ < 8) {
                    printf("  SCANOUT MISMATCH off=0x%x got=%02x want=%02x  [dbg q_head=%u q_tail=%u q_count=%u f_state=%u pending.size=%zu pending.front=0x%x issued=%llu completed=%llu]\n",
                           off, got, want, dut->dbg_q_head, dut->dbg_q_tail, dut->dbg_q_count, dut->dbg_f_state,
                           scan_pending.size(), scan_pending.empty() ? 0 : scan_pending.front(),
                           (unsigned long long)scan_issued, (unsigned long long)scan_completed);
                } else if (nmismatch % 500 == 0) {
                    printf("  SCANOUT MISMATCH off=0x%x got=%02x want=%02x (suppressed, count=%d)\n", off, got, want, nmismatch);
                }
                g_unmatched_resp_seen = true;
            }
            scan_got.push_back(got);
            scan_completed++;
            if (scan_have_last_valid_time) {
                uint64_t gap = sim_time - scan_last_valid_time;
                if (gap > scan_max_gap_cycles) scan_max_gap_cycles = gap;
            }
            scan_last_valid_time = sim_time;
            scan_have_last_valid_time = true;
        }
    }
}
static void scan_start(uint32_t from_off, uint32_t to_off_excl) {
    scan_next_off = from_off; scan_stop_off = to_off_excl;
    scan_driver_active = true;
    scan_pending.clear(); scan_got.clear();
    scan_issued = 0; scan_completed = 0; scan_max_gap_cycles = 0;
    scan_have_last_valid_time = false;
}
static void scan_stop() { scan_driver_active = false; }
static bool scan_drain(int max_cycles) {
    int c = 0;
    while ((scan_completed < scan_issued || scan_driver_active) && c < max_cycles) {
        if (scan_next_off >= scan_stop_off) scan_driver_active = false;
        tick(); c++;
    }
    return scan_pending.empty();
}

// ─── Reset ──────────────────────────────────────────────────────────────
static void reset_dut(bool core_only = false) {
    hold_mig_clock = false;
    if (!core_only) { dut->mig_rst = 1; dut->mig_clk = 0; dut->core_clk = 0; }
    dut->core_rst = 1;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->cpu_arvalid = 0;
    dut->cpu_bready = 1; dut->cpu_rready = 1;
    dut->scan_rd_en = 0;
    dut->test_bulk_enable = 0;
    dut->test_bulk_arvalid = 0;
    dut->test_bulk_araddr = 0;
    dut->test_bulk_rready = 1;
    dut->slv_flush = 0;
    eval();
    for (int i = 0; i < 8; i++) tick();
    if (!core_only) {
        dut->mig_rst = 0;
        int waited = 0;
        while (dut->cal_done == 0 && waited < 200) { tick(); waited++; }
    }
    dut->core_rst = 0;
    int waited = 0;
    while (dut->cpu_awready == 0 && dut->cpu_arready == 0 && waited < 8000) { tick(); waited++; }
}

#define CHECK(name, cond) do { if (!(cond)) { printf("  FAIL %s\n", name); return false; } } while (0)

// ═══════════════════════════════════════════════════════════════════════
// Scenario 1: cpu_write_scanout_read_roundtrip
// ═══════════════════════════════════════════════════════════════════════
static bool test_roundtrip() {
    reset_dut();
    bool ok = true;
    const int N = TEST_SPAN / 16;
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 5 + 3));
    for (int i = 0; i < N; i++) ok &= do_read_check((uint32_t)i * 16);
    CHECK("cpu writes/reads clean", ok && !g_unmatched_resp_seen);

    // Scanout streaming read-back over the same span.
    scan_start(0, TEST_SPAN);
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    CHECK("scanout drained", scan_completed == TEST_SPAN);
    CHECK("VRAM/scanout traffic does not allocate L2 misses", dut->dbg_l2_miss_count == 0);
    CHECK("VRAM/scanout traffic leaves L2 MSHRs empty", dut->dbg_l2_mshr_occupancy == 0);
    CHECK("scanout data matches stored (byte-swapped) golden", !g_unmatched_resp_seen);
    scan_stop();
    printf("  roundtrip: %u bytes written+read via CPU, %llu pixels streamed via scanout, max inter-response gap=%llu cycles\n",
           TEST_SPAN, (unsigned long long)scan_completed, (unsigned long long)scan_max_gap_cycles);
    return ok;
}

// Reproduce the exact wide-side shape emitted by axi_narrow_to_wide for
// sixteen byte stores.  The main chain regression historically used only
// AWSIZE=4/full-WSTRB beats, which cannot expose address/strobe handling
// failures in the QuickDraw hot path.
static bool test_narrow_byte_store_shapes() {
    reset_dut();
    const uint32_t base = 0x400;

    for (uint32_t off = 0; off < 16; off++) {
        std::array<uint8_t,16> beat{};
        const uint8_t value = static_cast<uint8_t>(0x41 + off);
        const uint32_t word_lane = off & ~3u;
        const uint32_t be_byte_lane = 3u - (off & 3u);
        beat[word_lane + be_byte_lane] = value;
        const uint16_t strb = static_cast<uint16_t>(1u << (word_lane + be_byte_lane));
        const uint32_t id = issue_write_shape(VRAM_APERTURE_BASE + base + off,
                                              beat, strb, 0);
        CHECK("narrow byte store completes", wait_write_done(id));
        CHECK("narrow byte store returns OKAY", w_done[id]->resp_ok);
        free_write(id);
        stored[base + off] = value;
    }

    scan_start(base, base + 16);
    CHECK("narrow byte scanout drains", scan_drain(20000));
    CHECK("narrow byte scanout returns every byte", scan_completed == 16);
    CHECK("narrow byte path has no unmatched responses", !g_unmatched_resp_seen);
    scan_stop();
    return true;
}

// Saturate the cache-miss side before introducing a cold scanout line.  The
// shared mux may overlap ordinary fills for throughput, but accepted-order
// DDR responses mean every accepted fill is non-preemptible queueing ahead
// of scanout.  MAX_BULK_AHEAD is therefore a latency contract, not merely a
// priority preference -- which is exactly why the bound is read from the
// DUT (dbg_mux_max_bulk_ahead) rather than written here as a literal: a
// retuned parameter must re-run this scenario against the NEW bound, not
// quietly stop testing anything.
static bool test_scanout_bounded_under_l2_saturation() {
    reset_dut();
    bool ok = true;
    const uint32_t base = 0x0030'0000u;

    arb_monitor_enable = true;
    arb_max_bulk = 0;
    arb_bulk_at_scan_ar = -1;
    arb_scan_ar_time = 0;
    hold_mig_clock = true;
    dut->test_bulk_enable = 1;
    dut->test_bulk_rready = 1;

    // Inject distinct-ID requests at the L2 slave port. Holding the MIG
    // clock leaves their genuine fills accepted at the core-side mux/CDC
    // boundary. The old injector entered after L2 and could report
    // "saturation" with zero misses/occupancy.
    for (int i = 0; i < 8; i++) {
        dut->test_bulk_araddr = base + (uint32_t)i * 64u;
        dut->test_bulk_arvalid = 1;
        eval();
        int issue_guard = 1000;
        while (!dut->test_bulk_arready && issue_guard-- > 0) tick();
        CHECK("direct L2 request becomes ready", dut->test_bulk_arready);
        tick();
        dut->test_bulk_arvalid = 0;
        eval();
    }
    const int mba = (int)dut->dbg_mux_max_bulk_ahead;   // scanout-active bound
    const int mbq = (int)dut->dbg_mux_max_bulk_quiet;   // scanout-quiet bound
    // Nothing is asking for scanout yet, so the arbiter is in its QUIET
    // state and the CPU is entitled to the whole route FIFO.  That is the
    // point of the adaptive cap: scanout's reservation is not charged to
    // the CPU during the (long) gaps between scanout's refill excursions.
    // Saturate until the bulk window stops GROWING, rather than until it
    // reaches a particular number.  With `hold_mig_clock` the downstream
    // async bridge's own AR CDC FIFO (axi_async_bridge's AR_DEPTH_LOG2=2,
    // i.e. 4 entries) can be the binding term instead of the mux's cap, and
    // which of the two binds depends on parameters this scenario does not
    // own.  What the scenario actually needs is "the worst convoy this
    // composition can build", whatever produced it.
    int guard = 2000;
    int stable = 0, last_bulk = -1;
    while (guard-- > 0 && (stable < 40 || dut->dbg_l2_mshr_occupancy < 8)) {
        tick();
        if ((int)dut->dbg_mux_bulk_count != last_bulk) {
            last_bulk = (int)dut->dbg_mux_bulk_count; stable = 0;
        } else stable++;
    }
    const int sat_bulk = (int)dut->dbg_mux_bulk_count;
    printf("  saturation setup: bulk=%d (active cap %d, quiet cap %d) rdq=%u l2_mshr=%u l2_misses=%u guard=%d\n",
           sat_bulk, mba, mbq, (unsigned)dut->dbg_mux_rdq_count,
           (unsigned)dut->dbg_l2_mshr_occupancy, (unsigned)dut->dbg_l2_miss_count, guard);
    CHECK("saturation builds a convoy at least as deep as the scanout-active cap",
          sat_bulk >= mba);
    CHECK("saturation never exceeds the quiet-state bulk admission window",
          sat_bulk <= mbq);
    CHECK("saturation traffic allocated all eight real L2 misses",
          dut->dbg_l2_mshr_occupancy == 8 && dut->dbg_l2_miss_count >= 8);

    // The scan request is introduced INTO the frozen convoy, and the MIG
    // clock is then released so the convoy drains ahead of it.  The release
    // has to happen before waiting for the scan AR handshake: the convoy
    // fills the downstream async bridge's own AR CDC FIFO, so with the MIG
    // clock held NOTHING can be accepted, scanout included.  (That is not a
    // regression, it is what "the CPU is allowed to use the whole read
    // pipeline while the display is quiet" means -- and the frozen-clock rig
    // is the only place it looks like a stall, because only here does the
    // pipeline never drain.)
    const uint32_t scan_off = TEST_SPAN - LINE_BYTES;
    uint64_t request_time = sim_time;
    scan_start(scan_off, scan_off + 1);
    hold_mig_clock = false;
    guard = 200 + 200 * mbq;
    while (arb_bulk_at_scan_ar < 0 && guard-- > 0) tick();
    printf("  scan AR fired with bulk=%d queued ahead (frozen convoy was %d)\n",
           arb_bulk_at_scan_ar, sat_bulk);
    CHECK("scan AR accepted while a saturated bulk convoy is outstanding",
          arb_bulk_at_scan_ar > 0);
    guard = 800 + 200 * mbq;
    while (scan_completed == 0 && guard-- > 0) tick();
    uint64_t response_latency = sim_time - request_time;
    scan_stop();

    // The bound scales with the admission window because that is precisely
    // what the window buys the CPU and costs scanout: each additional bulk
    // burst admitted ahead of a scan request is 4 more beats of in-order R
    // data (a 64 B L2 line at 128 b/beat) that must drain first.
    const uint64_t latency_bound = 400 + 40 * (uint64_t)mbq;
    CHECK("scan AR accepted under saturated L2 demand", arb_bulk_at_scan_ar >= 0);
    CHECK("no more bulk bursts accepted ahead of scanout than the configured bound",
          arb_bulk_at_scan_ar >= 0 && arb_bulk_at_scan_ar <= mbq && arb_max_bulk <= (uint32_t)mbq);
    // The SUSTAINED bound (bulk_count may not GROW past MAX_BULK_AHEAD once
    // scanout is active) is asserted inside axi_vram_priority_mux3.v itself,
    // where it can watch every admission rather than every sampled cycle.
    CHECK("cold scanout request receives a bounded reply",
          scan_completed == 1 && response_latency < latency_bound);

    guard = 800;
    while (dut->dbg_mux_bulk_count != 0 && guard-- > 0) tick();
    CHECK("queued bulk reads drain after scanout service", dut->dbg_mux_bulk_count == 0);
    dut->test_bulk_enable = 0;
    arb_monitor_enable = false;
    printf("  bounded scanout: bulk_at_AR=%d max_bulk=%u AR-to-reply=%llu cycles request-to-reply=%llu cycles\n",
           arb_bulk_at_scan_ar, arb_max_bulk,
           (unsigned long long)(sim_time >= arb_scan_ar_time ?
                                (scan_last_valid_time - arb_scan_ar_time) : 0),
           (unsigned long long)response_latency);
    return ok && !g_unmatched_resp_seen;
}

// Seed backing DDR through the uncached VRAM lane, then read those bytes
// through distinct cold low-RAM lines while scanout continuously consumes the
// same physical region. The simulation backend's finite storage folds the
// VRAM carveout onto low RAM, giving both paths a known nonzero backing line
// without changing the xbar's production-strict RAM visibility policy. This
// catches a fill delivered to the wrong MSHR/tag even when all bursts retain
// legal AXI lengths and response codes.
static bool test_cold_fill_scoreboard_under_scanout() {
    reset_dut();
    bool ok = true;
    const int chunks = TEST_SPAN / 16;
    for (int i = 0; i < chunks; i++)
        ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 29 + 0x35));
    CHECK("cold-fill backing data seeded through VRAM bypass", ok);

    scan_start(0, TEST_SPAN);
    constexpr int COLD_LINES = TEST_SPAN / 64;
    constexpr int WINDOW = 8;
    int max_mshr = 0;
    int checked = 0;

    for (int first = 0; first < COLD_LINES; first += WINDOW) {
        if (scan_next_off >= scan_stop_off && scan_pending.empty())
            scan_start(0, TEST_SPAN);

        std::array<uint32_t, WINDOW> ids{};
        std::array<uint32_t, WINDOW> offsets{};
        for (int lane = 0; lane < WINDOW; lane++) {
            const int n = first + lane;
            const uint32_t off = (uint32_t)(n % COLD_LINES) * 64u;
            offsets[lane] = off;
            ids[lane] = issue_read(off);
        }

        int guard = 5000;
        while (guard-- > 0) {
            bool all_done = true;
            for (uint32_t id : ids) all_done &= r_done.count(id) != 0;
            if (all_done) break;
            tick();
            max_mshr = std::max(max_mshr, (int)dut->dbg_l2_mshr_occupancy);
        }
        for (int lane = 0; lane < WINDOW; lane++) {
            CHECK("cold fill completes under scanout", r_done.count(ids[lane]) != 0);
            auto rb = r_done[ids[lane]];
            std::array<uint8_t,16> expect{};
            for (int b = 0; b < 16; b++) expect[b] = stored[offsets[lane] + (uint32_t)b];
            CHECK("cold fill data remains associated with its requested line",
                  rb->last_resp == 0 && rb->got.size() == 1 && rb->got[0] == expect);
            free_read(ids[lane]);
            checked++;
        }
    }

    scan_driver_active = false;
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    scan_stop();
    CHECK("CPU/xbar cold-fill path reaches a live L2 miss", max_mshr >= 1);
    CHECK("cold-fill/scanout stress has no unmatched or corrupt response", !g_unmatched_resp_seen);
    printf("  cold-fill scoreboard: %d distinct L2 lines checked, max MSHR occupancy=%d\n",
           checked, max_mshr);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2: concurrent_streaming -- CPU R/W + scanout streaming at the
// same time. Measures CPU starvation bound under sustained scanout and
// the scanout underrun margin at a computed fetch rate.
// ═══════════════════════════════════════════════════════════════════════
static bool test_concurrent_streaming() {
    reset_dut();
    bool ok = true;
    // Seed the golden span first (single-threaded, no scanout yet).
    const int N = TEST_SPAN / 16;
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 11 + 7));
    CHECK("seed writes clean", ok);

    // Start scanout streaming continuously over the whole span, wrapping,
    // while CPU issues concurrent read traffic (re-reading the same
    // stable data -- no writes during streaming, so the golden compare
    // stays valid for both traffic classes simultaneously).
    scan_start(0, TEST_SPAN);

    // Measure CPU-op latency under sustained scanout load.
    uint64_t max_cpu_latency = 0, total_cpu_latency = 0;
    int cpu_ops = 64;
    for (int i = 0; i < cpu_ops; i++) {
        // Keep the scanout driver's window open by re-arming when it
        // reaches the end (this scenario cares about SUSTAINED scanout
        // load, not a one-shot pass).
        if (scan_next_off >= scan_stop_off && scan_pending.empty()) scan_start(0, TEST_SPAN);
        uint32_t off = (uint32_t)((i % N) * 16);
        uint64_t t0 = sim_time;
        uint32_t id = issue_read(VRAM_APERTURE_BASE + off);
        ok &= wait_read_done(id);
        uint64_t lat = sim_time - t0;
        if (lat > max_cpu_latency) max_cpu_latency = lat;
        total_cpu_latency += lat;
        std::array<uint8_t,16> expect{};
        for (int b = 0; b < 16; b++) expect[b] = stored[off + b];
        vram_swap16(expect.data());
        if (!(r_done[id]->got.size() == 1 && r_done[id]->got[0] == expect)) {
            printf("  concurrent CPU read mismatch off=0x%x\n", off);
            ok = false;
        }
        free_read(id);
    }

    // -- l2c-path (RAM) traffic under the SAME sustained scanout load --
    // Proves scanout's strict-top-priority contract holds against l2c
    // traffic specifically (not just the S3 lane), and that l2c traffic
    // itself isn't starved outright by the RR it shares with the S3 lane
    // (scanout never contends with l2c on this axis -- it only ever
    // contends with whichever of {l2c, s3} the top-level arbiter is
    // currently favoring for AR/R).
    uint64_t max_ram_latency = 0, total_ram_latency = 0;
    int ram_ops = 32;
    std::array<uint8_t,16> ram_expect{};
    for (int i = 0; i < 16; i++) ram_expect[i] = (uint8_t)(0xC0 + i);
    ok &= do_ram_write_apply(RAM_TRAFFIC_BASE, 0xC0);
    for (int i = 0; i < ram_ops; i++) {
        if (scan_next_off >= scan_stop_off && scan_pending.empty()) scan_start(0, TEST_SPAN);
        uint64_t lat = 0;
        bool rok = do_ram_read_check(RAM_TRAFFIC_BASE, ram_expect, &lat);
        ok &= rok;
        if (lat > max_ram_latency) max_ram_latency = lat;
        total_ram_latency += lat;
    }
    double avg_ram_latency = (double)total_ram_latency / ram_ops;
    printf("  concurrent_streaming: l2c (RAM) read latency under sustained scanout: max=%llu cycles, avg=%.1f cycles (%d ops)\n",
           (unsigned long long)max_ram_latency, avg_ram_latency, ram_ops);
    CHECK("l2c-path starvation bound", max_ram_latency < 20000);

    // -- CPU FB (S3 lane) WRITE latency under the SAME sustained scanout
    //    load -- writes land just past the actively-scanned window
    //    (TEST_SPAN..) so they can never alias what the concurrent
    //    scan_start(0, TEST_SPAN) pass above is reading; this measures
    //    pure latency-under-load, correctness of S3 writes themselves is
    //    already proven by cpu_write_scanout_read_roundtrip. --
    uint64_t max_fbw_latency = 0, total_fbw_latency = 0;
    int fbw_ops = 32;
    for (int i = 0; i < fbw_ops; i++) {
        if (scan_next_off >= scan_stop_off && scan_pending.empty()) scan_start(0, TEST_SPAN);
        uint64_t lat = 0;
        std::array<uint8_t,16> beat{};
        for (int b = 0; b < 16; b++) beat[b] = (uint8_t)(0x40 + i + b);
        uint64_t t0 = sim_time;
        uint32_t id = issue_write(VRAM_APERTURE_BASE + TEST_SPAN + (uint32_t)i * 16, {beat});
        bool wok = wait_write_done(id);
        lat = sim_time - t0;
        ok &= wok && w_done[id]->resp_ok;
        free_write(id);
        if (lat > max_fbw_latency) max_fbw_latency = lat;
        total_fbw_latency += lat;
    }
    double avg_fbw_latency = (double)total_fbw_latency / fbw_ops;
    printf("  concurrent_streaming: CPU FB (S3 lane) write latency under sustained scanout: max=%llu cycles, avg=%.1f cycles (%d ops)\n",
           (unsigned long long)max_fbw_latency, avg_fbw_latency, fbw_ops);
    CHECK("CPU FB write starvation bound", max_fbw_latency < 20000);

    // Fully drain the scanout stream (not just stop issuing new requests
    // -- scan_stop() alone would abandon whatever the RTL's internal
    // queue still has outstanding, desyncing this driver's in-order
    // tracking against the next scan_start() below) before moving on.
    scan_driver_active = false;
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    scan_stop();
    CHECK("no protocol violations under concurrent load", !g_unmatched_resp_seen);

    double avg_cpu_latency = (double)total_cpu_latency / cpu_ops;
    printf("  concurrent_streaming: CPU read latency under sustained scanout: max=%llu cycles, avg=%.1f cycles (%d ops)\n",
           (unsigned long long)max_cpu_latency, avg_cpu_latency, cpu_ops);
    // Bound: a single-outstanding-per-channel arbiter (axi_vram_priority_
    // mux3.v) caps CPU-path added latency at roughly one scanout burst's
    // AR..RLAST duration; generous guard band for CDC + sim_mig_backend
    // latency.
    CHECK("CPU starvation bound", max_cpu_latency < 20000);

    // -- Underrun margin at a 1080p8-equivalent fetch rate --
    // 1024x768 8bpp @ 60 Hz needs 1024*768*60 = 47,185,920 B/s. At the
    // canonical 100 MHz core_clk signoff target that's 0.472 B/core_clk
    // cycle = one pixel request roughly every 1/0.472 = 2.12 core_clk
    // cycles. We drive scanout at the FASTEST possible rate this BFM
    // supports (one request every cycle, i.e. worst case ~2.1x the real
    // required rate) with sim_mig_backend jitter enabled and report the
    // measured max response gap as the underrun margin -- as long as the
    // measured gap stays well under what a real consumer's line-buffer
    // slack could absorb, there is margin.
    scan_start(TEST_SPAN / 2, TEST_SPAN); // fresh window, avoid the ring's already-warm state from before
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    CHECK("scanout drained (rate scenario)", scan_completed == (TEST_SPAN - TEST_SPAN/2));
    CHECK("no scanout data corruption", !g_unmatched_resp_seen);
    double required_pixels_per_cycle = 47185920.0 / 100000000.0; // 1024x768@60 8bpp @ 100 MHz
    double achieved_pixels_per_cycle = (double)scan_completed / (double)(sim_time - 0); // conservative: whole elapsed sim time, not just this window
    printf("  concurrent_streaming: scanout max inter-response gap=%llu cycles; required=%.3f px/cycle, worst-case-instantaneous-gap-equivalent rate=%.3f px/cycle\n",
           (unsigned long long)scan_max_gap_cycles, required_pixels_per_cycle,
           scan_max_gap_cycles > 0 ? 1.0 / (double)scan_max_gap_cycles : 0.0);
    scan_stop();
    // A real consumer never sees single-request granularity underrun --
    // it has a line-buffered slack window (fb_reader.v's rsp FIFO +
    // video_top's line buffer). The bound below is the max gap this
    // module itself can ever produce (one full miss-fetch's worth,
    // documented in scanout_ddr_reader.v's header ~44 cycles at 100 MHz)
    // plus jitter headroom.
    CHECK("scanout worst-case gap within derived bound", scan_max_gap_cycles < 400);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2b: frame_wrap_repaint -- full-branch-review C1. scanout_
// line_fetch.v's ring is a cache with NO frame-boundary flush: `hit`
// used to serve a resident lb_line == req_line match FOREVER, so a CPU
// repaint between frames (the normal double-buffering-free case on this
// platform -- Mac OS just writes the SAME framebuffer address range
// again every frame) could leave scanout serving PREVIOUS-frame bytes
// indefinitely. Reproduced by the reviewer's standalone probe
// (scratchpad/t14_review/tb_framewrap.cpp): frame 2 issued no AR at all
// for lines that happened to still be ring-resident from frame 1.
// `stored[]` IS the golden model scan_drive_and_latch_post() already
// compares every response against, so simply repainting it with a
// DIFFERENT pattern between two scan_start(0, TEST_SPAN) passes is
// sufficient to catch any stale-residue leak automatically -- no new
// tracking needed.
// ═══════════════════════════════════════════════════════════════════════
static bool test_frame_wrap_repaint() {
    reset_dut();
    bool ok = true;
    const int N = TEST_SPAN / 16;

    // Frame 1: paint pattern A, stream it back.
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 5 + 3));
    CHECK("frame1 paint completes", ok);
    scan_start(0, TEST_SPAN);
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    CHECK("frame1 scanout drained", scan_completed == TEST_SPAN);
    CHECK("frame1 scanout matches pattern A", !g_unmatched_resp_seen);
    scan_stop();

    // CPU repaints the ENTIRE span with a DIFFERENT pattern -- the
    // "vsync between frames" moment -- before scanout restarts at
    // offset 0 again for frame 2.
    g_unmatched_resp_seen = false; // isolate frame2's own check from frame1's
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 7 + 101));
    CHECK("frame2 repaint completes", ok);

    // Frame 2: scanout restarts at offset 0. Every byte must reflect
    // the repainted pattern B, not frame 1's residue.
    scan_start(0, TEST_SPAN);
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    CHECK("frame2 scanout drained", scan_completed == TEST_SPAN);
    CHECK("frame2 scanout matches repainted pattern B (no stale frame-1 residue)",
          !g_unmatched_resp_seen);
    scan_stop();

    // Third pass for good measure -- proves the ring keeps flushing
    // cleanly across REPEATED wraps, not just a single one-shot fix.
    g_unmatched_resp_seen = false;
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 11 + 17));
    CHECK("frame3 repaint completes", ok);
    scan_start(0, TEST_SPAN);
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    CHECK("frame3 scanout drained", scan_completed == TEST_SPAN);
    CHECK("frame3 scanout matches repainted pattern C (no stale frame-2 residue)",
          !g_unmatched_resp_seen);
    scan_stop();
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2c: frame_wrap_small_no_deadlock -- full-branch-review C1's
// second symptom. NUM_LINE_BUF=4; for a frame of <= NUM_LINE_BUF+1 = 5
// lines, EVERY resident ring slot at the wrap boundary holds a line
// index numerically LARGER than the new frame's low req_line values,
// so the eviction rule `lb_line[hs] < req_line` finds no victim and the
// fetch engine idles forever -- permanent stall, not just stale data.
// Exercises exactly the reviewer's boundary (5 lines) across several
// consecutive wraps with a bounded scan_drain() so a real deadlock
// fails this test instead of hanging the whole suite.
// ═══════════════════════════════════════════════════════════════════════
static bool test_frame_wrap_small_no_deadlock() {
    reset_dut();
    bool ok = true;
    const uint32_t SMALL_SPAN = 5 * LINE_BYTES; // NUM_LINE_BUF+1 lines, the exact deadlock boundary
    const int N = SMALL_SPAN / 16;
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 3 + 1));
    CHECK("small-frame paint completes", ok);

    for (int frame = 0; frame < 4; frame++) {
        g_unmatched_resp_seen = false;
        scan_start(0, SMALL_SPAN);
        ok &= scan_drain((int)SMALL_SPAN * 200 + 5000);
        char msg[96];
        snprintf(msg, sizeof(msg), "small frame %d scanout drained (no deadlock at wrap)", frame);
        CHECK(msg, scan_completed == SMALL_SPAN);
        snprintf(msg, sizeof(msg), "small frame %d content correct", frame);
        CHECK(msg, !g_unmatched_resp_seen);
        scan_stop();
    }
    printf("  frame_wrap_small_no_deadlock: %u-byte (%u-line) frame streamed cleanly across 4 wraps\n",
           SMALL_SPAN, SMALL_SPAN / LINE_BYTES);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 3: reset_mid_stream(pre_reset_ticks) -- a BOUNDED handful of
// ops genuinely in flight at the moment reset asserts (this seam's own
// reset-drain: scanout_ddr_reader.v's F_RSTDRAIN and axi_ro_priority_
// mux2.v's ar_open-gated hold), not a sustained deep-backlog stream.
//
// This phase sweep keeps the injected request set deliberately bounded;
// the separate victim-writeback reset test exercises the bridge's deeper
// write recovery while real L2 state is active.
// ═══════════════════════════════════════════════════════════════════════
static bool test_reset_mid_stream(int pre_reset_ticks) {
    reset_dut();
    bool ok = true;
    const int N = TEST_SPAN / 16;
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 3 + 1));
    CHECK("seed writes clean", ok);

    // A single scanout request (scan_start bounded to exactly one offset)
    // and a single CPU read -- genuinely a "handful of ops in flight,"
    // not a sustained stream.
    scan_start(0, 1);
    uint32_t rid = issue_read(VRAM_APERTURE_BASE);
    for (int i = 0; i < pre_reset_ticks; i++) tick(); // let it get in-flight inside the mux/bridge/backend

    // Stop GENERATING new scanout requests before calling reset_dut() --
    // reset_dut() internally spins for as many cycles as the reset
    // sequence needs (e.g. l2c's own ~4096-cycle set-walk reset FSM,
    // docs/l2c_spec.md S7, when CHAIN_L2C_ENABLE=1), and this driver has
    // no backpressure-aware notion of "the DUT is mid-reset, stop
    // issuing" -- leaving it running would keep pushing/tracking new
    // requests for thousands of cycles against a DUT transitioning
    // through reset, which is a fundamentally different scenario from
    // the bounded request set this phase sweep intends to test. Whatever
    // is already queued legitimately stays in flight into the reset.
    // that's the scenario under test -- it's just abandoned by the
    // tracking clear below rather than compared against post-reset
    // data.
    scan_driver_active = false;
    reset_dut(/*core_only=*/true);

    // Drop tracking for whatever was in flight pre-reset.
    w_pending_issue.clear(); r_pending_issue.clear();
    w_issuing = nullptr; r_issuing = nullptr;
    w_awaiting_b.clear(); r_awaiting.clear();
    w_done.clear(); r_done.clear();
    free_ids.clear(); init_ids();
    scan_stop(); scan_pending.clear(); scan_got.clear();
    (void)rid;
    g_unmatched_resp_seen = false; // pre-reset trickle-in is expected, not a violation
    for (int i = 0; i < 50; i++) tick();
    g_unmatched_resp_seen = false;
    // Fresh CPU writes/reads and a fresh scanout pass must both be correct
    // post-reset (the carveout's DRAM contents survive reset -- only the
    // cache/reader controller state resets, matching l2c's own "dirty
    // lines dropped, DRAM contents are whatever reached it" story --
    // here nothing was dropped since these are FRESH writes after reset).
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(0x80 + i));
    for (int i = 0; i < N; i++) ok &= do_read_check((uint32_t)i * 16);
    scan_start(0, TEST_SPAN);
    ok &= scan_drain(TEST_SPAN * 200 + 5000);
    CHECK("scanout drained post-reset", scan_completed == TEST_SPAN);
    scan_stop();
    CHECK("post-reset traffic clean on both paths", !g_unmatched_resp_seen);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 4 (T16, new): raw_write_then_scan_read_same_line -- a CPU
// write to a line, immediately followed (before the write even
// completes) by a scanout request for the SAME line, must observe the
// NEW data. See this file's header for the full citation of the
// mechanism (axi_ddr4_mig_bridge.v's global same-64B-line hazard check,
// downstream of the S3-lane/scanout arbitration point) -- this test
// exists to verify that mechanism actually covers this seam's traffic
// shape, not to add a second ordering mechanism at the arbiter/lane.
// ═══════════════════════════════════════════════════════════════════════
static bool test_raw_write_then_scan_read_same_line() {
    reset_dut();
    bool ok = true;
    const uint32_t LINE_OFF = 3 * LINE_BYTES; // an arbitrary line, distinct from other tests' offsets

    // Baseline: settle the WHOLE line (LINE_BYTES/16 beats) to a known
    // pattern A --
    // fully complete, not part of the race -- via the CPU AXI readback
    // path only (NOT scanout). Deliberately does NOT touch scanout here:
    // scanout_line_fetch.v's ring caches a resident line for the rest of
    // the current "frame" (only invalidated at a frame-wrap boundary,
    // see scanout_line_fetch.v's frame_wrap_c) -- if the baseline were
    // observed via scanout first, the race read below would be served
    // from that residency (a cache HIT, no new AR issued at all) instead
    // of triggering a fresh fetch, which would defeat the point of this
    // test entirely (it would pass regardless of whether the hazard
    // check works, since no new DRAM read would ever happen). The race
    // read below MUST be this line's first-ever scanout touch in this
    // test, guaranteeing a cold fetch (miss) that actually issues a new
    // AR for the hazard check to gate.
    for (uint32_t b = 0; b < LINE_BYTES; b += 16) ok &= do_write_apply(LINE_OFF + b, (uint8_t)(0xA0 + b));
    for (uint32_t b = 0; b < LINE_BYTES; b += 16) ok &= do_read_check(LINE_OFF + b);
    CHECK("baseline line settles (CPU readback, no scanout touch yet)", ok);

    // The race: issue a NEW write (pattern B) to the line's FIRST beat
    // only, update the golden model immediately (mirroring what real
    // hardware guarantees the eventual result will be), then -- WITHOUT
    // waiting for the write to complete -- issue a scanout request for
    // the whole line, genuinely overlapping the write's own in-flight
    // window (the write takes many cycles to traverse xbar -> S3 ->
    // arbiter -> async bridge -> mig bridge -> sim_mig_backend; a single
    // `tick()` gap is nowhere near enough time for it to have committed).
    g_unmatched_resp_seen = false;
    std::array<uint8_t,16> beatB{};
    for (int i = 0; i < 16; i++) beatB[i] = (uint8_t)(0xB0 + i);
    uint32_t wid = issue_write(VRAM_APERTURE_BASE + LINE_OFF, {beatB});
    std::array<uint8_t,16> swappedB = beatB;
    vram_swap16(swappedB.data());
    for (int i = 0; i < 16; i++) stored[LINE_OFF + i] = swappedB[i]; // bytes 16..LINE_BYTES-1 keep pattern A, already correct in stored[]

    tick(); // the write is now genuinely presenting/in-flight, nowhere near its B response
    scan_start(LINE_OFF, LINE_OFF + LINE_BYTES);

    ok &= wait_write_done(wid);
    ok &= w_done[wid] && w_done[wid]->resp_ok;
    free_write(wid);
    ok &= scan_drain(5000);
    CHECK("post-race scanout drained", scan_completed == LINE_BYTES);
    CHECK("post-race scanout observes pattern B on the raced beat, pattern A elsewhere in the line -- never stale",
          !g_unmatched_resp_seen);
    scan_stop();
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 5 (T16, new): s3_flush_abort_no_wedge -- axi_xbar's existing
// slv_flush machinery aborts an S3 op mid-flight (S3 has always been an
// `is_flush_domain_slv()` member -- unaffected by T16's revert of the
// S3->S0 decode fold, which only touched decode_slv()/ddr_flatten(), not
// the flush-domain membership function). The late backend response must
// be dropped cleanly by the xbar's existing poison/unroutability
// machinery with no wedge, and the lane must be usable again afterward.
// ═══════════════════════════════════════════════════════════════════════
static bool test_s3_flush_abort_no_wedge() {
    reset_dut();
    bool ok = true;

    // Issue an S3 (VRAM-aperture) write, then assert slv_flush for one
    // cycle while it's genuinely in flight (mirroring axi_xbar's own
    // documented flush-domain contract -- a pulse, not a held level).
    std::vector<std::array<uint8_t,16>> beats(4);
    for (size_t b = 0; b < beats.size(); b++)
        for (int i = 0; i < 16; i++)
            beats[b][i] = (uint8_t)(0x90 + b * 16 + i);
    // Freeze MIG, accept AW and exactly part of a four-beat W burst, then
    // flush before WLAST.  This forces the xbar to drain the remaining
    // upstream beats while independently padding the accepted downstream
    // transaction; waiting for w_awaiting_b would exercise only late-B.
    hold_mig_clock = true;
    uint32_t wid = issue_write(VRAM_APERTURE_BASE + 4 * LINE_BYTES, beats);
    int setup_guard = 200;
    bool partial_w = false;
    while (setup_guard-- > 0) {
        tick();
        if (w_issuing && w_issuing->id == wid && w_issuing->aw_done &&
            w_issuing->issue_beat > 0 && w_issuing->issue_beat < w_issuing->beats) {
            partial_w = true;
            break;
        }
    }
    CHECK("flush setup stopped a four-beat S3 write before WLAST", partial_w);

    dut->slv_flush = 1;
    tick();
    dut->slv_flush = 0;
    hold_mig_clock = false;

    // The flushed op's own tracking is abandoned here (the xbar's own
    // flush contract is "exactly one clean SLVERR or a locally-absorbed
    // response for flushed transactions" -- not this tb's concern to
    // decode precisely; what matters is the lane is NOT wedged
    // afterward). Drain whatever comes back (SLVERR/OKAY, either is
    // acceptable) without hanging the suite.
    auto write_still_active = [&]() {
        if (w_issuing && w_issuing->id == wid) return true;
        if (w_awaiting_b.find(wid) != w_awaiting_b.end()) return true;
        for (const auto& pending : w_pending_issue)
            if (pending->id == wid) return true;
        return false;
    };
    int drained = 0;
    while (drained < 5000 && write_still_active()) { tick(); drained++; }
    CHECK("flush-aborted partial S3 write retired", !write_still_active());
    w_awaiting_b.erase(wid);
    w_done.erase(wid);
    free_ids.insert(wid);
    g_unmatched_resp_seen = false; // a flush-aborted op's late/absent response is expected, not a protocol violation

    // Give the fabric a settle window past the flush pulse (matches this
    // tb's other post-flush-class patterns, e.g. reset_mid_stream's
    // post-reset settle) before proving the lane is usable again.
    for (int i = 0; i < 50; i++) tick();
    g_unmatched_resp_seen = false;

    // Prove the lane is USABLE AFTER: a completely fresh S3 write/read
    // roundtrip AND a fresh scanout pass must both work cleanly -- no
    // wedge left behind by the aborted op.
    const uint32_t POST_OFF = 5 * LINE_BYTES;
    ok &= do_write_apply(POST_OFF, 0xE0);
    ok &= do_read_check(POST_OFF);
    CHECK("S3 lane usable after flush-abort (CPU r/w)", ok);
    scan_start(POST_OFF, POST_OFF + LINE_BYTES);
    ok &= scan_drain(5000);
    CHECK("scanout usable after flush-abort", scan_completed == LINE_BYTES && !g_unmatched_resp_seen);
    scan_stop();
    return ok;
}

// A core-only reset is observed on the source side of axi_async_bridge while
// its MIG side keeps running. The bridge owns completion of any downstream
// write already opened at that boundary. An upstream L2 victim must not also
// replay filler W beats after reset, or those beats have no AW and can be
// consumed by a later unrelated write.
static bool test_reset_mid_l2_victim_writeback() {
    reset_dut();
    bool ok = true;
    constexpr uint32_t SET_BASE = 0x0010'0000u;
    constexpr uint32_t SET_STRIDE = 0x0004'0000u;

    // Dirty every way of one set, then miss on a ninth tag to launch a real
    // four-beat victim writeback through mux3 and the async bridge.
    //
    // All FOUR 16 B quadrants of each way are written on purpose.  L2C's
    // dirty bits are per quadrant (docs/l2c_perf.md S12), so a line touched
    // in one quadrant evicts as a ONE-beat burst -- and this scenario needs
    // a burst with beats still owed at the moment reset lands, which is the
    // whole point of the S_RSTDRAINW filler path it exercises.
    for (int way = 0; way < 8; way++)
        for (int q = 0; q < 4; q++)
            ok &= do_ram_write_apply(SET_BASE + (uint32_t)way * SET_STRIDE + (uint32_t)q * 16,
                                     (uint8_t)(0x20 + way * 13 + q * 3));
    CHECK("dirty set primed for victim-reset test", ok);

    std::array<uint8_t,16> ninth{};
    for (int b = 0; b < 16; b++) ninth[b] = (uint8_t)(0xD0 + b);
    uint32_t abandoned_id = issue_write(SET_BASE + 8u * SET_STRIDE, {ninth});
    int guard = 10000;
    while (guard-- > 0 &&
           !(dut->dbg_l2_victim_wburst && dut->dbg_l2_victim_beat >= 1))
        tick();
    CHECK("reset lands during an accepted L2 victim W burst",
          dut->dbg_l2_victim_wburst && dut->dbg_l2_victim_beat >= 1);

    reset_dut(/*core_only=*/true);

    // The pre-reset requester and its dirty cache state are architecturally
    // abandoned. Remove only host-side tracking, then test fresh traffic.
    w_pending_issue.clear(); r_pending_issue.clear();
    w_issuing = nullptr; r_issuing = nullptr;
    w_awaiting_b.clear(); r_awaiting.clear();
    w_done.clear(); r_done.clear();
    free_ids.clear(); init_ids();
    (void)abandoned_id;
    g_unmatched_resp_seen = false;
    for (int i = 0; i < 200; i++) tick();
    g_unmatched_resp_seen = false;

    ok &= do_write_apply(0x1000, 0x70);
    ok &= do_read_check(0x1000);
    std::array<uint8_t,16> ram_expect{};
    for (int b = 0; b < 16; b++) ram_expect[b] = (uint8_t)(0xA0 + b);
    ok &= do_ram_write_apply(0x0080'0000u, 0xA0);
    ok &= do_ram_read_check(0x0080'0000u, ram_expect);
    CHECK("unrelated RAM/VRAM writes remain clean after victim reset", ok);
    CHECK("victim reset leaves no orphan response", !g_unmatched_resp_seen);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 7: cpu_multibeat_incr_burst -- the CPU data master's NEW shape
// ═══════════════════════════════════════════════════════════════════════
//
// Until 2026-07-30 the CPU data port could not emit a multi-beat burst at
// all: its 32->128-bit socket widener (`axi_narrow_to_wide.v`) had no
// `n_arlen`/`n_awlen` ports and hardcoded the wide-side `arlen`/`awlen` to
// `8'd0`, so the D-cache's burst refill path was dead on hardware and
// every CPU access through this chain was a single 128-bit beat.  With
// that transport fixed on the CPU side, the data master now issues ONE
// wide INCR burst per line: `ar/awsize=3'd4` (16 B), `arburst/awburst=INCR`,
// `arlen/awlen = beats-1`, WLAST on the final beat only, one B per burst.
//
// Every other CPU-lane scenario in this file issues `beats=1`, so this
// chain -- axi_xbar M0 -> S0 -> l2c (when CHAIN_L2C_ENABLE) -> mux3 ->
// axi_async_bridge -> axi_ddr4_mig_bridge -> sim_mig_backend -- has never
// been exercised with the shape the CPU is about to start producing.  The
// two failure modes this is looking for are both silent at elaboration:
// a hop that ignores AxLEN and returns ONE beat hangs the master waiting
// for RLAST; a hop that ignores WLAST corrupts the FOLLOWING transaction.
//
// What is deliberately covered:
//   (a) len=2 -- the D-cache's 32 B line, the shape the CPU emits today.
//   (b) len=4 -- exactly one 64 B L2C line (`L2C_LINE_BYTES`, l2c_defs.vh).
//   (c) len=8 and len=16 -- LONGER than one L2C line, so the front door
//       must re-derive set/tag per beat and cross a line mid-burst.  Also
//       longer than the D-cache writeback's awlen=7.
//   (d) a 16 B-aligned but NOT 64 B-aligned start with len=8, so the
//       crossing lands mid-burst at an odd beat index rather than on a
//       burst boundary.  This is also the `addr[4]=1` case in
//       axi_ddr4_mig_bridge's 128->256-bit repacker, where the first MIG
//       beat is upper-half-only.
//   (e) per-beat address correctness: after each burst write, every beat
//       is re-read with a SEPARATE single-beat read.  A hop that wrote
//       all beats to the burst's base address would still pass a
//       burst-read-back (it would read the same wrong place), so the
//       single-beat cross-check is the part that actually pins the
//       address stride.
//   (f) beat COUNT and RLAST placement: `got.size()` must be exactly
//       `beats` (the harness only stops collecting on RLAST), so both a
//       short burst and an over-long one fail.
//   (g) exactly one B per write burst -- an extra B lands in the
//       harness's UNEXPECTED-B path and sets g_unmatched_resp_seen.
//   (h) a burst immediately followed by a SINGLE-beat access to a
//       different line, checking no burst state leaked into it.
//   (i) back-to-back bursts with no wait between them.
//
// Note the harness randomly deasserts RREADY/BREADY (`req_ready_gate(85)`),
// so mid-burst master backpressure is exercised throughout rather than
// needing its own case.
static bool test_cpu_multibeat_incr_burst() {
    reset_dut();
    bool ok = true;

    // Well clear of RAM_TRAFFIC_BASE's other users, and 4 KB-aligned so no
    // burst here can cross the AXI 4 KB boundary (max span below is 256 B).
    const uint32_t BURST_BASE = 0x00020000u;
    const int      L2C_LINE   = 64;   // l2c_defs.vh L2C_LINE_BYTES

    struct Case { uint32_t off; int beats; const char* what; };
    static const Case kCases[] = {
        {0x0000,  2, "len=2 (D-cache 32 B line, aligned)"},
        {0x0100,  4, "len=4 (exactly one 64 B L2C line)"},
        {0x0200,  8, "len=8 (2 L2C lines -- crosses a line)"},
        {0x0400, 16, "len=16 (4 L2C lines -- crosses 3 lines)"},
        {0x0630,  8, "len=8 from +0x30 (crossing lands mid-burst, addr[4]=1)"},
    };

    for (const auto& c : kCases) {
        const uint32_t addr = BURST_BASE + c.off;

        // Distinct, position-derived data in every beat AND every byte, so
        // a swapped pair of beats or a stuck address is visibly wrong.
        std::vector<std::array<uint8_t,16>> beats((size_t)c.beats);
        for (int b = 0; b < c.beats; b++)
            for (int i = 0; i < 16; i++)
                beats[b][i] = (uint8_t)(0xA0 ^ (c.off + b * 16 + i));

        // ── multi-beat write burst ──────────────────────────────────────
        uint32_t wid = issue_write((uint64_t)addr, beats);
        if (!wait_write_done(wid)) {
            printf("  BURST WRITE TIMEOUT %s addr=0x%08x -- a hop is not "
                   "consuming all W beats or never returns B\n", c.what, addr);
            return false;
        }
        bool wok = w_done[wid]->resp_ok;
        free_write(wid);
        if (!wok) {
            printf("  BURST WRITE got non-OKAY BRESP: %s addr=0x%08x\n", c.what, addr);
            return false;
        }
        CHECK("exactly one B per write burst", !g_unmatched_resp_seen);

        // ── (e) per-beat address stride, checked with SINGLE-beat reads ──
        for (int b = 0; b < c.beats; b++) {
            uint32_t sid = issue_read((uint64_t)(addr + b * 16), 1);
            if (!wait_read_done(sid)) {
                printf("  single-beat verify read TIMEOUT %s beat=%d\n", c.what, b);
                return false;
            }
            auto rb = r_done[sid];
            bool m = (rb->got.size() == 1) && (rb->got[0] == beats[b]) &&
                     (rb->last_resp == 0);
            if (!m) {
                printf("  BURST WRITE LANDED WRONG: %s beat=%d addr=0x%08x\n"
                       "    got =", c.what, b, addr + b * 16);
                if (!rb->got.empty()) for (auto x : rb->got[0]) printf("%02x", x);
                printf("\n    want=");
                for (auto x : beats[b]) printf("%02x", x);
                printf("\n");
                free_read(sid);
                return false;
            }
            free_read(sid);
        }

        // ── multi-beat read burst over the same span ────────────────────
        uint32_t rid = issue_read((uint64_t)addr, c.beats);
        if (!wait_read_done(rid)) {
            printf("  BURST READ TIMEOUT %s addr=0x%08x -- a hop returned "
                   "fewer beats than ARLEN+1 (RLAST never seen)\n", c.what, addr);
            return false;
        }
        auto rburst = r_done[rid];
        // (f) exact beat count: the harness stops collecting on RLAST, so a
        // size mismatch means RLAST landed on the wrong beat.
        if ((int)rburst->got.size() != c.beats) {
            printf("  BURST READ WRONG BEAT COUNT: %s got %zu beats, want %d "
                   "(RLAST misplaced)\n", c.what, rburst->got.size(), c.beats);
            free_read(rid);
            return false;
        }
        for (int b = 0; b < c.beats; b++) {
            if (rburst->got[b] != beats[b]) {
                printf("  BURST READ BEAT %d MISMATCH: %s addr=0x%08x\n"
                       "    got =", b, c.what, addr + b * 16);
                for (auto x : rburst->got[b]) printf("%02x", x);
                printf("\n    want=");
                for (auto x : beats[b]) printf("%02x", x);
                printf("\n");
                free_read(rid);
                return false;
            }
        }
        bool rresp_ok = (rburst->last_resp == 0);
        free_read(rid);
        CHECK("burst read RRESP OKAY on every beat", rresp_ok);
        CHECK("no unmatched R/B during burst", !g_unmatched_resp_seen);

        // ── (h) a single-beat access straight after the burst ───────────
        // Different 64 B line, so this cannot be served out of any burst
        // state that should have retired.  Catches leaked beat counters.
        const uint32_t after = addr + (uint32_t)c.beats * 16 + (uint32_t)L2C_LINE;
        std::array<uint8_t,16> one{};
        for (int i = 0; i < 16; i++) one[i] = (uint8_t)(0x5C ^ i);
        uint32_t aw = issue_write((uint64_t)after, {one});
        if (!wait_write_done(aw)) {
            printf("  post-burst SINGLE-beat write TIMEOUT after %s\n", c.what);
            return false;
        }
        bool awok = w_done[aw]->resp_ok;
        free_write(aw);
        uint32_t ar = issue_read((uint64_t)after, 1);
        if (!wait_read_done(ar)) {
            printf("  post-burst SINGLE-beat read TIMEOUT after %s\n", c.what);
            return false;
        }
        bool arok = (r_done[ar]->got.size() == 1) && (r_done[ar]->got[0] == one);
        free_read(ar);
        if (!(awok && arok)) {
            printf("  BURST STATE LEAKED INTO NEXT SINGLE-BEAT ACCESS after %s\n", c.what);
            return false;
        }

        printf("  %-52s OK (%d beats, %u B)\n", c.what, c.beats, (unsigned)c.beats * 16);
    }

    // ── (i) back-to-back bursts, both in flight before either completes ──
    // The harness serialises AW acceptance per burst but does not wait for
    // B, so this genuinely queues a second burst behind the first and
    // checks the write path cannot lose ownership mid-burst and interleave.
    {
        const uint32_t a0 = BURST_BASE + 0x0800u;
        const uint32_t a1 = BURST_BASE + 0x0900u;
        std::vector<std::array<uint8_t,16>> b0(4), b1(4);
        for (int b = 0; b < 4; b++)
            for (int i = 0; i < 16; i++) {
                b0[b][i] = (uint8_t)(0x11 + b * 16 + i);
                b1[b][i] = (uint8_t)(0x88 - b * 16 - i);
            }
        uint32_t w0 = issue_write((uint64_t)a0, b0);
        uint32_t w1 = issue_write((uint64_t)a1, b1);
        if (!wait_write_done(w0) || !wait_write_done(w1)) {
            printf("  BACK-TO-BACK BURST WRITE TIMEOUT\n");
            return false;
        }
        ok &= w_done[w0]->resp_ok && w_done[w1]->resp_ok;
        free_write(w0); free_write(w1);
        CHECK("back-to-back burst writes both OKAY", ok);

        uint32_t r0 = issue_read((uint64_t)a0, 4);
        uint32_t r1 = issue_read((uint64_t)a1, 4);
        if (!wait_read_done(r0) || !wait_read_done(r1)) {
            printf("  BACK-TO-BACK BURST READ TIMEOUT\n");
            return false;
        }
        bool m0 = (r_done[r0]->got.size() == 4);
        bool m1 = (r_done[r1]->got.size() == 4);
        for (int b = 0; b < 4 && m0 && m1; b++) {
            m0 = m0 && (r_done[r0]->got[b] == b0[b]);
            m1 = m1 && (r_done[r1]->got[b] == b1[b]);
        }
        free_read(r0); free_read(r1);
        if (!(m0 && m1)) {
            printf("  BACK-TO-BACK BURSTS INTERLEAVED OR MIS-ROUTED "
                   "(m0=%d m1=%d)\n", (int)m0, (int)m1);
            return false;
        }
        printf("  %-52s OK\n", "back-to-back 4-beat bursts, no interleave");
    }

    CHECK("no unmatched R/B anywhere in the burst scenario", !g_unmatched_resp_seen);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 8: cpu_burst_concurrent_with_scanout
// ═══════════════════════════════════════════════════════════════════════
// The scanout reader has always driven 8-beat `arlen=7` INCR bursts
// (scanout_line_fetch.v ARLEN, scanout_ddr_reader.v LINE_OFF_W=7), so the
// READ direction of this chain has carried multi-beat traffic all along --
// but only from a master that owns its own lane of axi_vram_priority_mux3.
// What is new is a multi-beat CPU burst contending for the SAME downstream
// port at the same time.  mux3 locks its AR grant from AR to RLAST and its
// AW grant from AW to B, so neither burst may have its beats interleaved
// with the other's; this checks that property with real traffic rather
// than by reading the arbiter.
static bool test_cpu_burst_concurrent_with_scanout() {
    reset_dut();
    bool ok = true;

    // Seed the scanout span so its reads have defined data (and so the
    // golden `stored[]` model stays consistent for the scan checker).
    const int N = (int)(4 * LINE_BYTES / 16);
    for (int i = 0; i < N; i++) ok &= do_write_apply((uint32_t)i * 16, (uint8_t)(i * 7 + 1));
    CHECK("seed writes clean", ok);

    const uint32_t BURST_BASE = 0x00030000u;

    // Sustained scanout streaming for the whole window.
    scan_start(0, 4 * LINE_BYTES);

    // While that runs, push CPU bursts on the RAM (l2c) lane.
    for (int round = 0; round < 6 && ok; round++) {
        const uint32_t addr = BURST_BASE + (uint32_t)round * 0x100u;
        const int beats = (round & 1) ? 8 : 4;   // 8 beats crosses an L2C line
        std::vector<std::array<uint8_t,16>> data((size_t)beats);
        for (int b = 0; b < beats; b++)
            for (int i = 0; i < 16; i++)
                data[b][i] = (uint8_t)(0x3B ^ (round * 31 + b * 16 + i));

        uint32_t wid = issue_write((uint64_t)addr, data);
        if (!wait_write_done(wid)) {
            printf("  CPU burst write TIMEOUT under scanout load (round=%d)\n", round);
            scan_stop();
            return false;
        }
        ok &= w_done[wid]->resp_ok;
        free_write(wid);

        uint32_t rid = issue_read((uint64_t)addr, beats);
        if (!wait_read_done(rid)) {
            printf("  CPU burst read TIMEOUT under scanout load (round=%d)\n", round);
            scan_stop();
            return false;
        }
        auto rb = r_done[rid];
        bool m = ((int)rb->got.size() == beats);
        for (int b = 0; b < beats && m; b++) m = m && (rb->got[b] == data[b]);
        free_read(rid);
        if (!m) {
            printf("  CPU BURST CORRUPTED BY CONCURRENT SCANOUT BURST (round=%d, "
                   "beats=%d)\n", round, beats);
            scan_stop();
            return false;
        }
    }

    ok &= scan_drain(4 * LINE_BYTES * 400 + 20000);
    CHECK("scanout still drains alongside CPU bursts", scan_completed == 4 * LINE_BYTES);
    scan_stop();
    CHECK("no unmatched R/B under concurrent bursts", !g_unmatched_resp_seen);
    printf("  6 CPU bursts (4 and 8 beats) completed against sustained "
           "8-beat scanout bursts; max scanout gap=%llu cycles\n",
           (unsigned long long)scan_max_gap_cycles);
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

    run("cpu_write_scanout_read_roundtrip", test_roundtrip);
    run("narrow_byte_store_shapes", test_narrow_byte_store_shapes);
    run("scanout_bounded_under_l2_saturation", test_scanout_bounded_under_l2_saturation);
    run("cold_fill_scoreboard_under_scanout", test_cold_fill_scoreboard_under_scanout);
    run("concurrent_streaming", test_concurrent_streaming);
    run("frame_wrap_repaint", test_frame_wrap_repaint);
    run("frame_wrap_small_no_deadlock", test_frame_wrap_small_no_deadlock);
    run("raw_write_then_scan_read_same_line", test_raw_write_then_scan_read_same_line);
    run("s3_flush_abort_no_wedge", test_s3_flush_abort_no_wedge);
    run("reset_mid_l2_victim_writeback", test_reset_mid_l2_victim_writeback);
    run("cpu_multibeat_incr_burst", test_cpu_multibeat_incr_burst);
    run("cpu_burst_concurrent_with_scanout", test_cpu_burst_concurrent_with_scanout);

    // reset_mid_stream, varying the exact cycle reset asserts relative to
    // the in-flight ops (re-gate requirement: deterministic across
    // several distinct reset timings, not just one lucky/unlucky point).
    static const int kResetTickVariants[] = {1, 2, 3, 4, 6};
    for (int t : kResetTickVariants) {
        char name[64];
        snprintf(name, sizeof(name), "reset_mid_stream(pre_reset_ticks=%d)", t);
        g_unmatched_resp_seen = false;
        bool r = test_reset_mid_stream(t);
        printf("[%s] %s\n", r ? "PASS" : "FAIL", name);
        if (r) n_pass++; else n_fail++;
    }

    printf("\ntb_vram_ddr_chain: %d PASS / %d FAIL\n", n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
