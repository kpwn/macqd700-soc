// tb_axi_ddr4_mig_bridge.cpp -- pcie_test MIG AXI contract shim checks.
//
// Built via: make tb-axi-ddr4-mig-bridge

#include <array>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <verilated.h>
#include "Vaxi_ddr4_mig_bridge.h"

static Vaxi_ddr4_mig_bridge* dut = nullptr;
static int n_pass = 0;
static int n_fail = 0;

using Word128 = std::array<uint32_t, 4>;
using Word256 = std::array<uint32_t, 8>;

struct MigWriteBeat {
    Word256 data;
    uint32_t strb;
    bool last;
};

template <typename Port>
static void set_w128(Port& p, const Word128& v) {
    for (int i = 0; i < 4; i++) p[i] = v[i];
}

template <typename Port>
static Word128 get_w128(Port& p) {
    Word128 v{};
    for (int i = 0; i < 4; i++) v[i] = p[i];
    return v;
}

template <typename Port>
static void set_w256(Port& p, const Word256& v) {
    for (int i = 0; i < 8; i++) p[i] = v[i];
}

template <typename Port>
static Word256 get_w256(Port& p) {
    Word256 v{};
    for (int i = 0; i < 8; i++) v[i] = p[i];
    return v;
}

static std::string hex128(const Word128& v) {
    char buf[80];
    std::snprintf(buf, sizeof buf, "%08x_%08x_%08x_%08x",
                  v[3], v[2], v[1], v[0]);
    return std::string(buf);
}

static std::string hex256(const Word256& v) {
    char buf[160];
    std::snprintf(buf, sizeof buf,
                  "%08x_%08x_%08x_%08x_%08x_%08x_%08x_%08x",
                  v[7], v[6], v[5], v[4], v[3], v[2], v[1], v[0]);
    return std::string(buf);
}

#define CHECK(name, cond) do { \
    if (cond) { \
        std::printf("[PASS] %s\n", name); \
        n_pass++; \
    } else { \
        std::printf("[FAIL] %s\n", name); \
        n_fail++; \
    } \
} while (0)

static void eval() {
    dut->eval();
}

static void tick() {
    dut->clk = 0;
    eval();
    dut->clk = 1;
    eval();
}

static void idle_inputs() {
    dut->s_awid = 0;
    dut->s_awaddr = 0;
    dut->s_awlen = 0;
    dut->s_awsize = 0;
    dut->s_awburst = 0;
    dut->s_awvalid = 0;
    set_w128(dut->s_wdata, Word128{0, 0, 0, 0});
    dut->s_wstrb = 0;
    dut->s_wlast = 0;
    dut->s_wvalid = 0;
    dut->s_bready = 1;
    dut->s_arid = 0;
    dut->s_araddr = 0;
    dut->s_arlen = 0;
    dut->s_arsize = 0;
    dut->s_arburst = 0;
    dut->s_arvalid = 0;
    dut->s_rready = 1;

    dut->m_awready = 0;
    dut->m_wready = 0;
    dut->m_bid = 0;
    dut->m_bresp = 0;
    dut->m_bvalid = 0;
    dut->m_arready = 0;
    dut->m_rid = 0;
    set_w256(dut->m_rdata, Word256{0, 0, 0, 0, 0, 0, 0, 0});
    dut->m_rresp = 0;
    dut->m_rlast = 1;
    dut->m_rvalid = 0;
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    eval();
}

static bool accept_write(uint32_t addr, uint8_t id, const Word128& data,
                         uint16_t strb, bool last = true,
                         uint8_t size = 4, uint8_t burst = 1,
                         uint8_t len = 0) {
    dut->s_awid = id;
    dut->s_awaddr = addr;
    dut->s_awlen = len;
    dut->s_awsize = size;
    dut->s_awburst = burst;
    dut->s_awvalid = 1;
    set_w128(dut->s_wdata, data);
    dut->s_wstrb = strb;
    dut->s_wlast = last ? 1 : 0;
    dut->s_wvalid = 1;
    eval();
    bool ready = dut->s_awready && dut->s_wready;
    tick();
    dut->s_awvalid = 0;
    dut->s_wvalid = 0;
    dut->s_wlast = 0;
    eval();
    return ready;
}

static bool accept_write_address(uint32_t addr, uint8_t id, uint8_t len,
                                 uint8_t size = 4, uint8_t burst = 1) {
    dut->s_awid = id;
    dut->s_awaddr = addr;
    dut->s_awlen = len;
    dut->s_awsize = size;
    dut->s_awburst = burst;
    dut->s_awvalid = 1;
    eval();
    bool ok = dut->s_awready;
    tick();
    dut->s_awvalid = 0;
    eval();
    return ok;
}

static bool accept_write_data(const Word128& data, uint16_t strb, bool last) {
    set_w128(dut->s_wdata, data);
    dut->s_wstrb = strb;
    dut->s_wlast = last ? 1 : 0;
    dut->s_wvalid = 1;
    eval();
    bool ok = dut->s_wready;
    tick();
    dut->s_wvalid = 0;
    dut->s_wlast = 0;
    eval();
    return ok;
}

static bool accept_read(uint32_t addr, uint8_t id, uint8_t size = 4,
                        uint8_t burst = 1, uint8_t len = 0) {
    dut->s_arid = id;
    dut->s_araddr = addr;
    dut->s_arlen = len;
    dut->s_arsize = size;
    dut->s_arburst = burst;
    dut->s_arvalid = 1;
    eval();
    bool ready = dut->s_arready;
    tick();
    dut->s_arvalid = 0;
    eval();
    return ready;
}

// A LOCALLY REJECTED read must still answer with the full ARLEN+1 beats
// the master asked for, every beat RRESP=SLVERR / RDATA=0, RLAST on the
// last one only, and nothing forwarded to the MIG.  Returning a single
// truncated beat leaves a master that asked for N beats half-filling its
// cache line and waiting for an RLAST that never comes (task:
// xbar-burst-gaps, GAP 3).
static bool consume_rejected_read_burst(uint8_t id, int beats) {
    bool ok = true;
    for (int b = 0; b < beats; b++) {
        eval();
        const bool want_last = (b == beats - 1);
        Word128 got = get_w128(dut->s_rdata);
        bool beat_ok = dut->s_rvalid && dut->s_rid == id &&
                       dut->s_rresp == 2 &&
                       ((bool)dut->s_rlast == want_last) &&
                       got == Word128{0, 0, 0, 0} &&
                       !dut->m_arvalid;
        if (!beat_ok) {
            std::printf("  rejected-read beat %d/%d: rvalid=%u rid=0x%02x rresp=%u "
                        "rlast=%u (want %u) m_arvalid=%u\n",
                        b, beats, (uint32_t)dut->s_rvalid, (uint32_t)dut->s_rid,
                        (uint32_t)dut->s_rresp, (uint32_t)dut->s_rlast,
                        (uint32_t)want_last, (uint32_t)dut->m_arvalid);
        }
        ok = ok && beat_ok;
        tick();
    }
    // The burst is over: no stray extra beat may follow.
    eval();
    if (dut->s_rvalid) {
        std::printf("  rejected-read: a %d-beat burst was followed by an extra beat\n",
                    beats);
        ok = false;
    }
    return ok;
}

static bool forwarded_write_matches(uint32_t exp_addr, uint32_t exp_strb,
                                    const Word256& exp_data) {
    dut->m_awready = 1;
    dut->m_wready = 1;
    eval();
    for (int i = 0; i < 4 && !(dut->m_awvalid || dut->m_wvalid); i++) {
        tick();
        eval();
    }
    bool valid = dut->m_awvalid && dut->m_wvalid;
    bool meta = dut->m_awid == 0 && dut->m_awaddr == exp_addr &&
                dut->m_awlen == 0 && dut->m_awsize == 5 &&
                dut->m_awburst == 1 && dut->m_wlast == 1 &&
                dut->m_wstrb == exp_strb;
    Word256 got = get_w256(dut->m_wdata);
    bool data_ok = got == exp_data;
    if (!(valid && meta && data_ok)) {
        std::printf("  forwarded write got addr=0x%08x len=%u strb=0x%08x last=%u data=%s\n",
                    (uint32_t)dut->m_awaddr, (uint32_t)dut->m_awlen,
                    (uint32_t)dut->m_wstrb, (uint32_t)dut->m_wlast,
                    hex256(got).c_str());
    }
    tick();
    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    return valid && meta && data_ok;
}

static bool forwarded_write_burst_matches(uint32_t exp_addr, uint8_t exp_len,
                                          const std::vector<MigWriteBeat>& beats,
                                          bool backpressure) {
    bool ok = true;
    if (backpressure) {
        dut->m_awready = 0;
        dut->m_wready = 0;
        eval();
        for (int i = 0; i < 4 && !dut->m_awvalid; i++) {
            tick();
            eval();
        }
        // Pipelined cut-through contract: AW is pending (and stays pending
        // under backpressure) as soon as data has been accepted from the
        // repo side, but W is never presented to the MIG strictly BEFORE
        // its own AW ("data no later than command" -- see the bridge's
        // header note).  Confirm that ordering explicitly: m_wvalid must
        // stay low here even though the repo-side data is already fully
        // queued internally.
        ok = ok && dut->m_awvalid && !dut->m_wvalid &&
             dut->m_awaddr == exp_addr && dut->m_awlen == exp_len;
        tick();

        // Granting m_awready issues AW -- and, per the SAME "no later
        // than" rule, simultaneous is fine: the first buffered beat may
        // (and here, does) become valid on the exact same cycle AW is
        // issued, not one cycle later.  This is what lets an isolated
        // burst avoid an artificial bubble between AW and its data.
        dut->m_awready = 1;
        dut->m_wready = 0;
        eval();
        ok = ok && dut->m_awvalid && dut->m_wvalid &&
             dut->m_wstrb == beats[0].strb && get_w256(dut->m_wdata) == beats[0].data;
        tick();

        dut->m_awready = 0;
        dut->m_wready = 0;
        eval();
        ok = ok && !dut->m_awvalid && dut->m_wvalid &&
             dut->m_wstrb == beats[0].strb && get_w256(dut->m_wdata) == beats[0].data;
        tick();
    }

    if (!backpressure) {
        dut->m_awready = 1;
        dut->m_wready = 1;
        eval();
        for (int i = 0; i < 4 && !(dut->m_awvalid || dut->m_wvalid); i++) {
            tick();
            eval();
        }
    }

    for (size_t i = 0; i < beats.size(); i++) {
        dut->m_awready = backpressure ? 0 : 1;
        dut->m_wready = 1;
        eval();
        Word256 got = get_w256(dut->m_wdata);
        bool aw_ok = backpressure || (i != 0) ||
                     (dut->m_awvalid && dut->m_awid == 0 &&
                      dut->m_awaddr == exp_addr && dut->m_awlen == exp_len &&
                      dut->m_awsize == 5 && dut->m_awburst == 1);
        bool w_ok = dut->m_wvalid &&
                    dut->m_wstrb == beats[i].strb &&
                    dut->m_wlast == (beats[i].last ? 1 : 0) &&
                    got == beats[i].data;
        if (!(aw_ok && w_ok)) {
            std::printf("  burst write beat %zu got awv=%u addr=0x%08x len=%u wv=%u strb=0x%08x last=%u data=%s\n",
                        i, (uint32_t)dut->m_awvalid, (uint32_t)dut->m_awaddr,
                        (uint32_t)dut->m_awlen, (uint32_t)dut->m_wvalid,
                        (uint32_t)dut->m_wstrb, (uint32_t)dut->m_wlast,
                        hex256(got).c_str());
        }
        ok = ok && aw_ok && w_ok;
        tick();
    }
    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    return ok;
}

static bool finish_write_response(uint8_t exp_id, uint8_t resp = 0) {
    dut->m_bresp = resp;
    dut->m_bvalid = 1;
    eval();
    bool ok = dut->s_bvalid && dut->s_bid == exp_id &&
              dut->s_bresp == resp && dut->m_bready;
    tick();
    dut->m_bvalid = 0;
    eval();
    return ok;
}

// Check that the AR forwarded to the MIG carries the expected translated
// address / length / size / burst, then accept it.
//
// EXPECTATION CORRECTED (ddr-bridge-timing): this used to demand that
// m_arvalid be high in the very first cycle after the repo-side AR was
// accepted -- i.e. it silently required the AR payload to be a purely
// COMBINATIONAL function of the descriptor queue.  That is a cycle-
// accuracy assumption, not an AXI requirement: AXI4 places no bound on
// how long a master may take to present AR, only on how it behaves once
// VALID is asserted.  Encoding it here made the check structurally
// incapable of passing for ANY registered AR output stage, which is
// exactly what the 333 MHz timing fix needed (the combinational form put
// an 8:1 array mux plus the 26-bit hazard comparator bank directly on the
// MIG's AR inputs).
//
// The real intent -- "the AR that reaches the MIG is the right one" -- is
// preserved and unweakened.  The wait is bounded, and if m_arvalid never
// asserts within the window `saw_ar` stays false and the check FAILS, so
// this cannot pass vacuously (verified by pointing a scenario at a wrong
// expected address and at a never-issued AR).  The write-side twin,
// forwarded_write_matches(), has always had exactly this bounded spin.
static bool forwarded_read_matches(uint32_t exp_addr, uint8_t exp_len = 0) {
    dut->m_arready = 1;
    eval();
    bool saw_ar = false;
    for (int i = 0; i < 8 && !saw_ar; i++) {
        if (dut->m_arvalid) {
            saw_ar = true;
            break;
        }
        tick();
        eval();
    }
    bool ok = saw_ar && dut->m_arid == 0 &&
              dut->m_araddr == exp_addr && dut->m_arlen == exp_len &&
              dut->m_arsize == 5 && dut->m_arburst == 1;
    if (!ok) {
        std::printf("  forwarded read: saw_ar=%d addr=0x%08x (want 0x%08x) "
                    "len=%u (want %u) size=%u burst=%u\n",
                    (int)saw_ar, (uint32_t)dut->m_araddr, exp_addr,
                    (uint32_t)dut->m_arlen, (uint32_t)exp_len,
                    (uint32_t)dut->m_arsize, (uint32_t)dut->m_arburst);
    }
    tick();
    dut->m_arready = 0;
    eval();
    return ok;
}

static bool expect_repo_read(uint8_t exp_id, const Word128& exp_data,
                             bool exp_last, uint8_t exp_resp = 0) {
    eval();
    Word128 got = get_w128(dut->s_rdata);
    bool ok = dut->s_rvalid && dut->s_rid == exp_id &&
              dut->s_rresp == exp_resp &&
              dut->s_rlast == (exp_last ? 1 : 0) &&
              got == exp_data;
    if (!ok) {
        std::printf("  repo read got valid=%u id=%u resp=%u last=%u data=%s\n",
                    (uint32_t)dut->s_rvalid, (uint32_t)dut->s_rid,
                    (uint32_t)dut->s_rresp, (uint32_t)dut->s_rlast,
                    hex128(got).c_str());
    }
    return ok;
}

static bool finish_read_response(uint8_t exp_id, const Word256& mig_data,
                                 const Word128& exp_repo_data,
                                 uint8_t resp = 0) {
    set_w256(dut->m_rdata, mig_data);
    dut->m_rresp = resp;
    dut->m_rlast = 1;
    dut->m_rvalid = 1;
    eval();
    bool ok = expect_repo_read(exp_id, exp_repo_data, true, resp) && dut->m_rready;
    tick();
    dut->m_rvalid = 0;
    eval();
    return ok;
}

static bool feed_mig_beat_check(const Word256& mig_data, bool mig_last,
                                uint8_t exp_id, const Word128& exp_data,
                                bool exp_last) {
    set_w256(dut->m_rdata, mig_data);
    dut->m_rresp = 0;
    dut->m_rlast = mig_last ? 1 : 0;
    dut->m_rvalid = 1;
    eval();
    bool ok = expect_repo_read(exp_id, exp_data, exp_last) && dut->m_rready;
    tick();
    dut->m_rvalid = 0;
    eval();
    return ok;
}

// Feeds one MIG beat while the repo-side consumer is stalled (s_rready=0),
// confirming the intake skid still accepts it (m_rready=1) even though the
// bypassed first half can't fire yet -- the exact "drops a cycle" bug the
// T11 rework removes (the old single-hold-register design forced m_rready
// low whenever a held half was draining, even with MIG data ready).  The
// MIG-side handshake completes in exactly one cycle here (m_rvalid drops
// right after, like a well-behaved AXI source -- holding stale valid data
// across further cycles, as the pre-T11 helper did, is not legal AXI
// master behaviour and would double-accept into the skid under this
// design).  Then releases s_rready and confirms the now-registered beat
// fires correctly.
static bool feed_mig_beat_check_no_rstall(const Word256& mig_data, bool mig_last,
                                          uint8_t exp_id, const Word128& exp_first_half,
                                          bool exp_first_last) {
    dut->s_rready = 0;
    set_w256(dut->m_rdata, mig_data);
    dut->m_rresp = 0;
    dut->m_rlast = mig_last ? 1 : 0;
    dut->m_rvalid = 1;
    eval();
    bool accepted = dut->m_rready && dut->s_rvalid &&
                    expect_repo_read(exp_id, exp_first_half, exp_first_last);
    tick();
    dut->m_rvalid = 0;
    eval();

    dut->s_rready = 1;
    eval();
    bool fired = expect_repo_read(exp_id, exp_first_half, exp_first_last);
    tick();
    eval();
    return accepted && fired;
}

static bool accept_held_read(uint8_t exp_id, const Word128& exp_repo_data,
                             bool exp_repo_last) {
    eval();
    // No new MIG beat is being driven this cycle (m_rvalid already
    // dropped by the caller), so m_rready's value is a don't-care here --
    // only the correct unpack/sequencing of the already-registered second
    // half matters.
    bool ok = expect_repo_read(exp_id, exp_repo_data, exp_repo_last);
    tick();
    eval();
    return ok;
}

static void scenario_lower_half_write_maps_to_256_beat() {
    reset();
    Word128 data{0x00112233u, 0x44556677u, 0x8899AABBu, 0xCCDDEEFFu};
    bool ok = accept_write(0x00000100u, 0x2A, data, 0x00FFu);
    ok = ok && forwarded_write_matches(
        0x00000100u, 0x000000FFu,
        Word256{0x00112233u, 0x44556677u, 0x8899AABBu, 0xCCDDEEFFu,
                0, 0, 0, 0});
    ok = ok && finish_write_response(0x2A);
    CHECK("lower_half_write_maps_to_256_beat", ok);
}

static void scenario_upper_half_write_maps_to_256_beat() {
    reset();
    Word128 data{0x11111111u, 0x22222222u, 0x33333333u, 0x44444444u};
    bool ok = accept_write(0x00000110u, 0x15, data, 0xF00Fu);
    ok = ok && forwarded_write_matches(
        0x00000100u, 0xF00F0000u,
        Word256{0, 0, 0, 0,
                0x11111111u, 0x22222222u, 0x33333333u, 0x44444444u});
    ok = ok && finish_write_response(0x15);
    CHECK("upper_half_write_maps_to_256_beat", ok);
}

static void scenario_lower_half_write_burst_packs_pairs_with_backpressure() {
    reset();
    Word128 a{0xA0000000u, 0xA0000001u, 0xA0000002u, 0xA0000003u};
    Word128 b{0xB0000000u, 0xB0000001u, 0xB0000002u, 0xB0000003u};
    Word128 c{0xC0000000u, 0xC0000001u, 0xC0000002u, 0xC0000003u};
    Word128 d{0xD0000000u, 0xD0000001u, 0xD0000002u, 0xD0000003u};
    bool ok = accept_write_address(0x00001000u, 0x31, 3);
    ok = ok && accept_write_data(a, 0x00FFu, false);
    ok = ok && accept_write_data(b, 0xFF00u, false);
    ok = ok && accept_write_data(c, 0x0F0Fu, false);
    ok = ok && accept_write_data(d, 0xF0F0u, true);
    ok = ok && forwarded_write_burst_matches(
        0x00001000u, 1,
        {
            MigWriteBeat{Word256{a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]},
                         0xFF0000FFu, false},
            MigWriteBeat{Word256{c[0], c[1], c[2], c[3], d[0], d[1], d[2], d[3]},
                         0xF0F00F0Fu, true},
        },
        true);
    ok = ok && finish_write_response(0x31);
    CHECK("lower_half_write_burst_packs_pairs_with_backpressure", ok);
}

static void scenario_upper_half_write_burst_edge() {
    reset();
    Word128 a{0xA1000000u, 0xA1000001u, 0xA1000002u, 0xA1000003u};
    Word128 b{0xB1000000u, 0xB1000001u, 0xB1000002u, 0xB1000003u};
    Word128 c{0xC1000000u, 0xC1000001u, 0xC1000002u, 0xC1000003u};
    bool ok = accept_write_address(0x00002010u, 0x32, 2);
    ok = ok && accept_write_data(a, 0xAAAAu, false);
    ok = ok && accept_write_data(b, 0x5555u, false);
    ok = ok && accept_write_data(c, 0xF00Fu, true);
    ok = ok && forwarded_write_burst_matches(
        0x00002000u, 1,
        {
            MigWriteBeat{Word256{0, 0, 0, 0, a[0], a[1], a[2], a[3]},
                         0xAAAA0000u, false},
            MigWriteBeat{Word256{b[0], b[1], b[2], b[3], c[0], c[1], c[2], c[3]},
                         0xF00F5555u, true},
        },
        false);
    ok = ok && finish_write_response(0x32);
    CHECK("upper_half_write_burst_edge", ok);
}

static void scenario_long_write_burst_reaches_high_store_index() {
    reset();
    constexpr int repo_beats = 66;
    std::vector<Word128> data;
    std::vector<uint16_t> strb;
    std::vector<MigWriteBeat> mig_beats;
    data.reserve(repo_beats);
    strb.reserve(repo_beats);
    mig_beats.reserve(repo_beats / 2);

    for (int i = 0; i < repo_beats; i++) {
        uint32_t base = 0xE0000000u + static_cast<uint32_t>(i) * 0x10u;
        data.push_back(Word128{base + 0u, base + 1u, base + 2u, base + 3u});
        strb.push_back((i & 1) ? 0xF00Fu : 0x0FF0u);
    }
    for (int i = 0; i < repo_beats; i += 2) {
        const Word128& lo = data[i];
        const Word128& hi = data[i + 1];
        mig_beats.push_back(MigWriteBeat{
            Word256{lo[0], lo[1], lo[2], lo[3], hi[0], hi[1], hi[2], hi[3]},
            (static_cast<uint32_t>(strb[i + 1]) << 16) | strb[i],
            (i + 2) == repo_beats});
    }

    // Cut-through: this burst produces 33 MIG beats, far more than the
    // 2-deep output skid can hold, so the MIG side must be serviced
    // CONCURRENTLY with data injection (unlike the pre-T11 store-and-
    // forward design, which buffered the entire burst locally before
    // touching the MIG at all -- exactly the behaviour this rework
    // removes).  m_awready/m_wready are held high throughout and every
    // drained beat is captured, in order, for comparison against the
    // expected sequence at the end.
    dut->m_awready = 1;
    dut->m_wready = 1;

    dut->s_awid = 0x35;
    dut->s_awaddr = 0x00008000u;
    dut->s_awlen = repo_beats - 1;
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    eval();
    bool ok = dut->s_awready;

    bool aw_captured = false;
    uint32_t cap_awaddr = 0;
    uint8_t cap_awlen = 0;
    std::vector<MigWriteBeat> captured;

    for (int i = 0; i < repo_beats; i++) {
        set_w128(dut->s_wdata, data[i]);
        dut->s_wstrb = strb[i];
        dut->s_wlast = (i == repo_beats - 1) ? 1 : 0;
        dut->s_wvalid = 1;
        eval();
        // Retry like a real AXI master would: hold s_wvalid + the same
        // data steady until s_wready, rather than assuming single-cycle
        // acceptance (the skid can transiently backpressure for a few
        // cycles right at burst start, before AW has registered/issued).
        for (int spins = 0; spins < 8 && !dut->s_wready; spins++) {
            if (dut->m_awvalid && dut->m_awready && !aw_captured) {
                aw_captured = true;
                cap_awaddr = dut->m_awaddr;
                cap_awlen = dut->m_awlen;
            }
            if (dut->m_wvalid && dut->m_wready) {
                captured.push_back(MigWriteBeat{get_w256(dut->m_wdata),
                                                (uint32_t)dut->m_wstrb,
                                                dut->m_wlast != 0});
            }
            tick();
            dut->s_awvalid = 0;
            eval();
        }
        ok = ok && dut->s_wready;
        if (dut->m_awvalid && dut->m_awready && !aw_captured) {
            aw_captured = true;
            cap_awaddr = dut->m_awaddr;
            cap_awlen = dut->m_awlen;
        }
        if (dut->m_wvalid && dut->m_wready) {
            captured.push_back(MigWriteBeat{get_w256(dut->m_wdata),
                                            (uint32_t)dut->m_wstrb,
                                            dut->m_wlast != 0});
        }
        tick();
        dut->s_awvalid = 0;
        eval();
    }
    dut->s_wvalid = 0;
    dut->s_wlast = 0;
    eval();

    // Drain whatever is still in flight in the skid after the last data
    // cycle (up to its 2-beat depth).
    for (int i = 0; i < 4 && captured.size() < mig_beats.size(); i++) {
        if (dut->m_wvalid && dut->m_wready) {
            captured.push_back(MigWriteBeat{get_w256(dut->m_wdata),
                                            (uint32_t)dut->m_wstrb,
                                            dut->m_wlast != 0});
        }
        tick();
        eval();
    }

    ok = ok && aw_captured && cap_awaddr == 0x00008000u &&
         cap_awlen == static_cast<uint8_t>((repo_beats / 2) - 1);
    ok = ok && captured.size() == mig_beats.size();
    for (size_t i = 0; ok && i < mig_beats.size(); i++) {
        if (!(captured[i].data == mig_beats[i].data &&
              captured[i].strb == mig_beats[i].strb &&
              captured[i].last == mig_beats[i].last)) {
            std::printf("  long burst beat %zu mismatch: strb=0x%08x last=%d data=%s\n",
                        i, captured[i].strb, (int)captured[i].last,
                        hex256(captured[i].data).c_str());
            ok = false;
        }
    }

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();

    ok = ok && finish_write_response(0x35);
    CHECK("long_write_burst_reaches_high_store_index", ok);
}

static void scenario_word_write_maps_to_256_lane() {
    reset();
    Word128 data{0xA5A50000u, 0xA5A50001u, 0xA5A50002u, 0xA5A50003u};
    bool ok = accept_write(0x0000010Cu, 0x22, data, 0xF000u, true, 2);
    ok = ok && forwarded_write_matches(
        0x00000100u, 0x0000F000u,
        Word256{0xA5A50000u, 0xA5A50001u, 0xA5A50002u, 0xA5A50003u,
                0, 0, 0, 0});
    ok = ok && finish_write_response(0x22);
    CHECK("word_write_maps_to_256_lane", ok);
}

static void scenario_word_write_with_byte_offset_is_forwarded() {
    reset();
    Word128 data{0x5A5A0000u, 0x5A5A0001u, 0x5A5A0002u, 0x5A5A0003u};
    bool ok = accept_write(0x0000010Eu, 0x23, data, 0x00F0u, true, 2);
    ok = ok && forwarded_write_matches(
        0x00000100u, 0x000000F0u,
        Word256{0x5A5A0000u, 0x5A5A0001u, 0x5A5A0002u, 0x5A5A0003u,
                0, 0, 0, 0});
    ok = ok && finish_write_response(0x23);
    CHECK("word_write_with_byte_offset_is_forwarded", ok);
}

// HW-confirmed P1 from bitstream 9a2c14f: the Q700 ROM RAM-sizing routine at
// offset 0x46aa issues `move.b Dn,(addr)` to low RAM.  After task #153 the
// axi_narrow_to_wide adapter correctly derives `awsize=0` from a one-bit
// wstrb and presents a byte-granular awaddr to the MIG bridge.  The bridge
// `wr_aw_word_ok` check only accepted `awsize==2` and rejected awsize=0
// outright with a local SLVERR -- the exact failure flagged on real HW at
// AW addr 0x000025c4 with WSTRB=0x0010 (lane 4, post-narrow_to_wide).
static void scenario_narrow_byte_write_is_forwarded() {
    reset();
    // Mirror the exact HW-observed access: addr 0x25c4 with WSTRB lane 4
    // set.  Data lane[1] holds the byte we care about (lane = addr[3:2] = 1).
    Word128 data{0xDEADBEEFu, 0x000000A5u, 0xDEADBEEFu, 0xDEADBEEFu};
    bool ok = accept_write(0x000025C4u, 0x37, data, 0x0010u, true, /*size=*/0);
    ok = ok && forwarded_write_matches(
        0x000025C0u, 0x00000010u,
        Word256{0xDEADBEEFu, 0x000000A5u, 0xDEADBEEFu, 0xDEADBEEFu,
                0, 0, 0, 0});
    ok = ok && finish_write_response(0x37);
    CHECK("narrow_byte_write_is_forwarded", ok);
}

// Companion: byte write to the upper half of the 32B MIG beat with awsize=0.
// Confirms the WSTRB shift logic is honoured for the upper-128b half too.
static void scenario_narrow_byte_write_upper_half_is_forwarded() {
    reset();
    Word128 data{0xDEADBEEFu, 0xDEADBEEFu, 0xDEADBEEFu, 0xDE000000u};
    bool ok = accept_write(0x0000011Fu, 0x38, data, 0x8000u, true, /*size=*/0);
    ok = ok && forwarded_write_matches(
        0x00000100u, 0x80000000u,
        Word256{0, 0, 0, 0,
                0xDEADBEEFu, 0xDEADBEEFu, 0xDEADBEEFu, 0xDE000000u});
    ok = ok && finish_write_response(0x38);
    CHECK("narrow_byte_write_upper_half_is_forwarded", ok);
}

// Halfword (16-bit) write -- narrow_to_wide derives awsize=1 from a two-bit
// contiguous wstrb (e.g. MOVE.W to a halfword-aligned address).  Same
// rejection bug under the old `s_awsize == 3'd2` gate.
static void scenario_narrow_halfword_write_is_forwarded() {
    reset();
    Word128 data{0xDEADBEEFu, 0x0000BEEFu, 0xDEADBEEFu, 0xDEADBEEFu};
    bool ok = accept_write(0x00002600u, 0x39, data, 0x0030u, true, /*size=*/1);
    ok = ok && forwarded_write_matches(
        0x00002600u, 0x00000030u,
        Word256{0xDEADBEEFu, 0x0000BEEFu, 0xDEADBEEFu, 0xDEADBEEFu,
                0, 0, 0, 0});
    ok = ok && finish_write_response(0x39);
    CHECK("narrow_halfword_write_is_forwarded", ok);
}

static void scenario_read_slices_upper_half_and_preserves_id() {
    reset();
    bool ok = accept_read(0x00000210u, 0x3D);
    ok = ok && forwarded_read_matches(0x00000200u);
    Word256 mig{0xAAA00000u, 0xAAA00001u, 0xAAA00002u, 0xAAA00003u,
                0xBBB00000u, 0xBBB00001u, 0xBBB00002u, 0xBBB00003u};
    Word128 exp{0xBBB00000u, 0xBBB00001u, 0xBBB00002u, 0xBBB00003u};
    ok = ok && finish_read_response(0x3D, mig, exp);
    CHECK("read_slices_upper_half_and_preserves_id", ok);
}

static void scenario_lower_half_read_burst_unpacks_pairs_with_backpressure() {
    reset();
    Word128 a{0xA2000000u, 0xA2000001u, 0xA2000002u, 0xA2000003u};
    Word128 b{0xB2000000u, 0xB2000001u, 0xB2000002u, 0xB2000003u};
    Word128 c{0xC2000000u, 0xC2000001u, 0xC2000002u, 0xC2000003u};
    Word128 d{0xD2000000u, 0xD2000001u, 0xD2000002u, 0xD2000003u};
    bool ok = accept_read(0x00003000u, 0x33, 4, 1, 3);
    ok = ok && forwarded_read_matches(0x00003000u, 1);
    ok = ok && feed_mig_beat_check_no_rstall(
        Word256{a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]},
        false, 0x33, a, false);
    ok = ok && accept_held_read(0x33, b, false);
    ok = ok && feed_mig_beat_check(
        Word256{c[0], c[1], c[2], c[3], d[0], d[1], d[2], d[3]},
        true, 0x33, c, false);
    ok = ok && accept_held_read(0x33, d, true);
    CHECK("lower_half_read_burst_unpacks_pairs_with_backpressure", ok);
}

static void scenario_upper_half_read_burst_edge() {
    reset();
    Word128 a{0xA3000000u, 0xA3000001u, 0xA3000002u, 0xA3000003u};
    Word128 b{0xB3000000u, 0xB3000001u, 0xB3000002u, 0xB3000003u};
    Word128 c{0xC3000000u, 0xC3000001u, 0xC3000002u, 0xC3000003u};
    bool ok = accept_read(0x00004010u, 0x34, 4, 1, 2);
    ok = ok && forwarded_read_matches(0x00004000u, 1);
    ok = ok && feed_mig_beat_check(
        Word256{0, 0, 0, 0, a[0], a[1], a[2], a[3]},
        false, 0x34, a, false);
    ok = ok && feed_mig_beat_check(
        Word256{b[0], b[1], b[2], b[3], c[0], c[1], c[2], c[3]},
        true, 0x34, b, false);
    ok = ok && accept_held_read(0x34, c, true);
    CHECK("upper_half_read_burst_edge", ok);
}

static void scenario_word_read_returns_repo_half_for_narrow_slice() {
    reset();
    bool ok = accept_read(0x00000214u, 0x24, 2);
    ok = ok && forwarded_read_matches(0x00000200u);
    Word256 mig{0xAAA00000u, 0xAAA00001u, 0xAAA00002u, 0xAAA00003u,
                0xBBB00000u, 0xBBB00001u, 0xBBB00002u, 0xBBB00003u};
    Word128 exp{0xBBB00000u, 0xBBB00001u, 0xBBB00002u, 0xBBB00003u};
    ok = ok && finish_read_response(0x24, mig, exp);
    CHECK("word_read_returns_repo_half_for_narrow_slice", ok);
}

static void scenario_word_read_with_byte_offset_is_forwarded() {
    reset();
    bool ok = accept_read(0x00000216u, 0x25, 2);
    ok = ok && forwarded_read_matches(0x00000200u);
    Word256 mig{0xC0C00000u, 0xC0C00001u, 0xC0C00002u, 0xC0C00003u,
                0xD0D00000u, 0xD0D00001u, 0xD0D00002u, 0xD0D00003u};
    Word128 exp{0xD0D00000u, 0xD0D00001u, 0xD0D00002u, 0xD0D00003u};
    ok = ok && finish_read_response(0x25, mig, exp);
    CHECK("word_read_with_byte_offset_is_forwarded", ok);
}

static void scenario_high_address_is_local_slverr() {
    reset();
    Word128 data{0xDEADBEEFu, 0, 0, 0};
    bool ok = accept_write(0x80000000u, 0x09, data, 0x000Fu);
    eval();
    ok = ok && dut->s_bvalid && dut->s_bid == 0x09 &&
         dut->s_bresp == 2 && !dut->m_awvalid && !dut->m_wvalid;
    tick();

    ok = ok && accept_read(0x80000000u, 0x0A);
    eval();
    Word128 got = get_w128(dut->s_rdata);
    ok = ok && dut->s_rvalid && dut->s_rid == 0x0A &&
         dut->s_rresp == 2 && dut->s_rlast && got == Word128{0, 0, 0, 0} &&
         !dut->m_arvalid;
    tick();
    CHECK("high_address_is_local_slverr", ok);
}

static void scenario_narrow_multibeat_burst_is_local_slverr() {
    reset();
    bool ok = accept_write_address(0x00000300u, 0x05, 1, 2);
    ok = ok && accept_write_data(Word128{1, 2, 3, 4}, 0x000Fu, false);
    ok = ok && accept_write_data(Word128{5, 6, 7, 8}, 0x00F0u, true);
    eval();
    ok = ok && dut->s_bvalid && dut->s_bid == 0x05 &&
         dut->s_bresp == 2 && !dut->m_awvalid && !dut->m_wvalid;
    tick();

    // arlen=1 -> the master asked for TWO beats, so it must get two
    // SLVERR beats with RLAST on the second.  This expectation was
    // previously written as a single truncated beat, i.e. it asserted the
    // GAP-3 bug as intended behaviour; both the RTL and this expectation
    // are corrected together.
    ok = ok && accept_read(0x00000300u, 0x06, 2, 1, 1);
    ok = ok && consume_rejected_read_burst(0x06, 2);
    CHECK("narrow_multibeat_burst_is_local_slverr", ok);
}

// GAP 3: sweep every rejection reason that a MULTI-BEAT read can trip, and
// a range of lengths, confirming the bridge returns arlen+1 SLVERR beats
// each time rather than one truncated beat.  Rejection reasons per
// ar_word_ok/ar_beat_ok: arsize != 4, araddr[3:0] != 0, non-INCR burst,
// and araddr[31] set.
static void scenario_rejected_multibeat_read_returns_full_burst() {
    struct Case { const char* why; uint32_t addr; uint8_t size; uint8_t burst; };
    const Case cases[] = {
        {"narrow arsize",      0x00000300u, 2, 1},
        {"misaligned araddr",  0x00000304u, 4, 1},
        {"FIXED burst",        0x00000320u, 4, 0},
        {"WRAP burst",         0x00000320u, 4, 2},
        {"high address",       0x80000000u, 4, 1},
    };
    bool ok = true;
    for (const Case& c : cases) {
        for (uint8_t len : {uint8_t(1), uint8_t(3), uint8_t(7)}) {
            reset();
            const uint8_t id = 0x2A;
            bool this_ok = accept_read(c.addr, id, c.size, c.burst, len);
            this_ok = this_ok && consume_rejected_read_burst(id, (int)len + 1);
            if (!this_ok)
                std::printf("  FAILED case: %s, arlen=%u\n", c.why, (uint32_t)len);
            ok = ok && this_ok;
        }
    }
    CHECK("rejected_multibeat_read_returns_full_burst", ok);
}

static void scenario_misaligned_address_is_local_slverr() {
    reset();
    Word128 data{0xABCDEF01u, 0xABCDEF02u, 0xABCDEF03u, 0xABCDEF04u};
    bool ok = accept_write(0x00000304u, 0x06, data, 0x0FF0u);
    ok = ok && dut->s_bvalid && dut->s_bid == 0x06 &&
         dut->s_bresp == 2 && !dut->m_awvalid && !dut->m_wvalid;
    tick();

    ok = ok && accept_read(0x00000308u, 0x07);
    Word128 got = get_w128(dut->s_rdata);
    ok = ok && dut->s_rvalid && dut->s_rid == 0x07 &&
         dut->s_rresp == 2 && dut->s_rlast && got == Word128{0, 0, 0, 0} &&
         !dut->m_arvalid;
    tick();
    CHECK("misaligned_address_is_local_slverr", ok);
}

static void scenario_fixed_burst_is_local_slverr() {
    reset();
    Word128 data{0xFACE0001u, 0xFACE0002u, 0xFACE0003u, 0xFACE0004u};
    bool ok = accept_write(0x00000320u, 0x08, data, 0xFFFFu, true, 4, 0);
    ok = ok && dut->s_bvalid && dut->s_bid == 0x08 &&
         dut->s_bresp == 2 && !dut->m_awvalid && !dut->m_wvalid;
    tick();

    ok = ok && accept_read(0x00000320u, 0x09, 4, 0);
    Word128 got = get_w128(dut->s_rdata);
    ok = ok && dut->s_rvalid && dut->s_rid == 0x09 &&
         dut->s_rresp == 2 && dut->s_rlast && got == Word128{0, 0, 0, 0} &&
         !dut->m_arvalid;
    tick();
    CHECK("fixed_burst_is_local_slverr", ok);
}

static void scenario_wrong_size_is_local_slverr() {
    reset();
    // size=3 (8 byte) is not supported; expect local SLVERR.  Sizes 0/1/2
    // are all valid post-narrow-byte-store.
    Word128 data{0xCAFE0001u, 0xCAFE0002u, 0xCAFE0003u, 0xCAFE0004u};
    bool ok = accept_write(0x00000400u, 0x11, data, 0xFFFFu, true, 3);
    ok = ok && dut->s_bvalid && dut->s_bid == 0x11 &&
         dut->s_bresp == 2 && !dut->m_awvalid && !dut->m_wvalid;
    tick();

    ok = ok && accept_read(0x00000400u, 0x12, 3);
    Word128 got = get_w128(dut->s_rdata);
    ok = ok && dut->s_rvalid && dut->s_rid == 0x12 &&
         dut->s_rresp == 2 && dut->s_rlast && got == Word128{0, 0, 0, 0} &&
         !dut->m_arvalid;
    tick();
    CHECK("wrong_size_is_local_slverr", ok);
}

static void scenario_missing_wlast_is_local_slverr() {
    reset();
    Word128 data{0x11112222u, 0x33334444u, 0x55556666u, 0x77778888u};
    bool ok = accept_write(0x00000500u, 0x2C, data, 0x00FFu, false);
    eval();
    ok = ok && dut->s_bvalid && dut->s_bid == 0x2C &&
         dut->s_bresp == 2 && !dut->m_awvalid && !dut->m_wvalid;
    tick();
    CHECK("missing_wlast_is_local_slverr", ok);
}

// ════════════════════════════════════════════════════════════════════
// T11 multi-outstanding / cut-through coverage: pipelined MIG models +
// randomized scoreboard scenarios.
// ════════════════════════════════════════════════════════════════════

// Behavioural single-outstanding-per-channel MIG model, matching the
// documented contract of rtl/board/sim_mig_backend.v ("single outstanding
// write + single outstanding read; AXI allows interleaving the two
// directions").  Supports full AW/AR burst lengths (32 B/beat, matching
// what the bridge always emits) and optional per-cycle random ready/valid
// stall injection for backpressure-sweep coverage.  Byte-addressed sparse
// storage; unwritten bytes read as zero.
struct MigModel {
    std::unordered_map<uint32_t, uint8_t> mem;
    std::mt19937 rng;
    int stall_pct;

    bool     aw_have = false;
    uint32_t aw_addr = 0;
    uint8_t  aw_len = 0;
    bool     w_active = false;
    uint32_t w_addr = 0;
    uint8_t  w_beats_left = 0;
    bool     b_pending = false;

    bool     ar_have = false;
    uint32_t ar_addr = 0;
    uint8_t  ar_len = 0;
    bool     r_active = false;
    uint32_t r_addr = 0;
    uint8_t  r_beats_left = 0;

    MigModel(uint32_t seed, int stall_pct_) : rng(seed), stall_pct(stall_pct_) {}

    bool roll_stall() {
        return stall_pct > 0 && (int)(rng() % 100) < stall_pct;
    }

    Word256 read_beat(uint32_t addr) const {
        Word256 v{0, 0, 0, 0, 0, 0, 0, 0};
        for (int lane = 0; lane < 32; lane++) {
            auto it = mem.find(addr + (uint32_t)lane);
            uint8_t b = (it != mem.end()) ? it->second : 0;
            v[lane / 4] |= (uint32_t)b << ((lane % 4) * 8);
        }
        return v;
    }

    void write_beat(uint32_t addr, const Word256& data, uint32_t strb) {
        for (int lane = 0; lane < 32; lane++) {
            if (!((strb >> lane) & 1u)) continue;
            uint8_t b = (uint8_t)((data[lane / 4] >> ((lane % 4) * 8)) & 0xFFu);
            mem[addr + (uint32_t)lane] = b;
        }
    }

    // Sets m_*ready / m_rvalid / m_rdata for THIS cycle from state as of
    // the last observe_and_advance().  Call, then eval(), then inspect
    // dut->s_*, THEN call observe_and_advance().
    void drive_outputs() {
        dut->m_awready = (!aw_have && !w_active && !roll_stall()) ? 1 : 0;
        dut->m_wready  = (w_active && !roll_stall()) ? 1 : 0;
        dut->m_bvalid  = b_pending ? 1 : 0;
        dut->m_bid = 0;
        dut->m_bresp = 0;
        dut->m_arready = (!ar_have && !r_active && !roll_stall()) ? 1 : 0;
        dut->m_rvalid  = r_active ? 1 : 0;
        if (r_active) {
            set_w256(dut->m_rdata, read_beat(r_addr));
            dut->m_rresp = 0;
            dut->m_rlast = (r_beats_left == 0) ? 1 : 0;
        } else {
            dut->m_rlast = 0;
        }
        dut->m_rid = 0;
    }

    // Observes this cycle's settled handshakes and advances model state.
    // Call after eval(), before tick().
    void observe_and_advance() {
        if (dut->m_awvalid && dut->m_awready) {
            aw_have = true;
            aw_addr = dut->m_awaddr;
            aw_len  = dut->m_awlen;
        }
        if (aw_have && !w_active) {
            w_active     = true;
            w_addr       = aw_addr;
            w_beats_left = aw_len;
            aw_have      = false;
        }
        if (w_active && dut->m_wvalid && dut->m_wready) {
            write_beat(w_addr, get_w256(dut->m_wdata), dut->m_wstrb);
            if (w_beats_left == 0) {
                w_active  = false;
                b_pending = true;
            } else {
                w_addr += 32;
                w_beats_left--;
            }
        }
        if (b_pending && dut->m_bvalid && dut->m_bready) {
            b_pending = false;
        }

        if (dut->m_arvalid && dut->m_arready) {
            ar_have = true;
            ar_addr = dut->m_araddr;
            ar_len  = dut->m_arlen;
        }
        if (ar_have && !r_active) {
            r_active     = true;
            r_addr       = ar_addr;
            r_beats_left = ar_len;
            ar_have      = false;
        }
        if (r_active && dut->m_rvalid && dut->m_rready) {
            if (r_beats_left == 0) {
                r_active = false;
            } else {
                r_addr += 32;
                r_beats_left--;
            }
        }
    }
};

static void mig_settle(MigModel& mig) {
    mig.drive_outputs();
    eval();
}
static void mig_advance(MigModel& mig) {
    mig.observe_and_advance();
    tick();
}

// A read-only MIG model with FIXED PIPELINE LATENCY: accepts a new AR
// every cycle it is offered (unbounded internal queue) and returns the
// corresponding R beat exactly LATENCY cycles later, regardless of how
// many other ARs are already in flight.  This is what a real pipelined
// DDR4 controller looks like (command accepted, data returns after a
// fixed CAS-like delay) -- unlike MigModel/sim_mig_backend.v's simpler
// single-outstanding stub, it can actually show the benefit of RMAX_
// OUTSTANDING-deep issuance.  Data content is a fixed pattern (irrelevant
// to the timing measurement this drives).
struct PipelinedReadMig {
    static constexpr int LATENCY = 4;
    std::deque<int> inflight_countdown;
    bool cur_valid = false;

    void settle() {
        dut->m_arready = 1;
        dut->m_rvalid = cur_valid ? 1 : 0;
        if (cur_valid) {
            set_w256(dut->m_rdata, Word256{0xD00DD00Du, 0xD00DD00Du, 0xD00DD00Du,
                                           0xD00DD00Du, 0, 0, 0, 0});
            dut->m_rresp = 0;
            dut->m_rlast = 1;
        } else {
            dut->m_rlast = 0;
        }
        dut->m_rid = 0;
    }

    void advance() {
        if (dut->m_arvalid && dut->m_arready) {
            inflight_countdown.push_back(LATENCY);
        }
        for (auto& c : inflight_countdown) {
            if (c > 0) c--;
        }
        if (!cur_valid && !inflight_countdown.empty() && inflight_countdown.front() == 0) {
            cur_valid = true;
            inflight_countdown.pop_front();
        }
        if (cur_valid && dut->m_rvalid && dut->m_rready) {
            cur_valid = false;
        }
    }
};

// Issues 8 back-to-back reads without waiting for any to complete before
// issuing the next (relying on RMAX_OUTSTANDING=8), against a fixed-
// latency pipelined MIG model, and measures wall-cycle count against a
// naive-serial baseline (8x an isolated single-read round trip through
// the SAME model).  Directly exercises the T11 requirement: "8 back-to-
// back reads pipelined ... measure and print the speedup factor."
static void scenario_eight_pipelined_reads_speedup() {
    const int N = 8;
    const uint32_t base = 0x00020000u;

    reset();
    dut->s_rready = 1;
    PipelinedReadMig solo_mig;
    int solo_cycles = 0;
    dut->s_arid = 0x10;
    dut->s_araddr = base;
    dut->s_arlen = 0;
    dut->s_arsize = 4;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    bool ar_accepted = false;
    for (; solo_cycles < 100 && !ar_accepted; solo_cycles++) {
        solo_mig.settle();
        eval();
        ar_accepted = dut->s_arvalid && dut->s_arready;
        solo_mig.advance();
        tick();
        if (ar_accepted) dut->s_arvalid = 0;
    }
    bool r_seen = false;
    for (; solo_cycles < 100 && !r_seen; solo_cycles++) {
        solo_mig.settle();
        eval();
        r_seen = dut->s_rvalid != 0;
        solo_mig.advance();
        tick();
    }

    reset();
    dut->s_rready = 1;
    PipelinedReadMig mig;
    int issued = 0, completed = 0, cycles = 0;
    std::vector<uint8_t> expect_ids;
    bool id_ok = true;
    while (completed < N && cycles < 2000) {
        if (issued < N) {
            dut->s_arid = (uint8_t)(0x10 + issued);
            dut->s_araddr = base + (uint32_t)issued * 16u;
            dut->s_arlen = 0;
            dut->s_arsize = 4;
            dut->s_arburst = 1;
            dut->s_arvalid = 1;
        } else {
            dut->s_arvalid = 0;
        }
        mig.settle();
        eval();
        if (dut->s_arvalid && dut->s_arready) {
            expect_ids.push_back((uint8_t)(0x10 + issued));
            issued++;
        }
        if (dut->s_rvalid && dut->s_rready) {
            if (completed >= (int)expect_ids.size() || dut->s_rid != expect_ids[completed]) {
                id_ok = false;
            }
            completed++;
        }
        mig.advance();
        tick();
        cycles++;
    }
    dut->s_arvalid = 0;
    eval();

    double naive_serial = (double)solo_cycles * N;
    double speedup = naive_serial / (double)cycles;
    std::printf("  eight_pipelined_reads: solo_rt=%d cyc, naive_serial=%.0f cyc, "
                "pipelined=%d cyc, speedup=%.2fx\n",
                solo_cycles, naive_serial, cycles, speedup);
    bool ok = id_ok && completed == N && (double)cycles < naive_serial * 0.7;
    CHECK("eight_pipelined_reads_speedup", ok);
}

// Interleaves single-beat writes and reads (including reads that race
// immediately behind a write to the SAME address) against the single-
// outstanding-per-channel MigModel, scoreboarding every value against a
// byte-addressed golden model.
static void scenario_interleaved_read_write_streams() {
    reset();
    dut->s_bready = 1;
    dut->s_rready = 1;
    MigModel mig(/*seed=*/42, /*stall_pct=*/0);
    std::unordered_map<uint32_t, Word128> golden;

    const uint32_t base = 0x00030000u;
    const int ops = 64;
    bool ok = true;

    // Single write / single read in flight from the repo side at a time,
    // to keep this scenario's bookkeeping simple and its focus purely on
    // interleaving + same-address correctness.  Multi-outstanding
    // pipelining depth (multiple writes/reads truly concurrent, not one
    // spun to completion before the next is issued) is exercised by
    // scenario_pipelined_writes_multi_outstanding and by
    // scenario_random_scoreboard_10k_mixed_ops (both fixed in the T11
    // review response -- this comment previously claimed the scoreboard
    // already covered it, which was false: it spun each op to completion
    // one at a time).
    dut->s_awvalid = 0;
    dut->s_wvalid = 0;
    dut->s_arvalid = 0;

    for (int i = 0; i < ops && ok; i++) {
        bool do_write = (i % 3) != 2;  // 2/3 writes, 1/3 reads
        uint32_t addr = base + (uint32_t)(i % 8) * 16u;  // 8-line pool: guarantees re-visits

        if (do_write) {
            Word128 data{0xE0000000u + (uint32_t)i, 0xE1000000u + (uint32_t)i,
                        0xE2000000u + (uint32_t)i, 0xE3000000u + (uint32_t)i};
            dut->s_awid = (uint8_t)i;
            dut->s_awaddr = addr;
            dut->s_awlen = 0;
            dut->s_awsize = 4;
            dut->s_awburst = 1;
            dut->s_awvalid = 1;
            set_w128(dut->s_wdata, data);
            dut->s_wstrb = 0xFFFF;
            dut->s_wlast = 1;
            dut->s_wvalid = 1;
            bool committed = false;
            for (int spins = 0; spins < 32 && !committed; spins++) {
                mig_settle(mig);
                bool aw_fire_now = dut->s_awvalid && dut->s_awready;
                bool w_fire_now  = dut->s_wvalid && dut->s_wready;
                bool b_fire_now  = dut->s_bvalid != 0;
                mig_advance(mig);
                if (aw_fire_now) dut->s_awvalid = 0;
                if (w_fire_now) dut->s_wvalid = 0;
                if (b_fire_now) {
                    golden[addr] = data;
                    committed = true;
                }
            }
            ok = ok && committed;
        } else {
            dut->s_arid = (uint8_t)(0x40 + i);
            dut->s_araddr = addr;
            dut->s_arlen = 0;
            dut->s_arsize = 4;
            dut->s_arburst = 1;
            dut->s_arvalid = 1;
            Word128 got{};
            bool got_data = false;
            for (int spins = 0; spins < 16 && !got_data; spins++) {
                mig_settle(mig);
                bool ar_fire_now = dut->s_arvalid && dut->s_arready;
                bool r_fire_now  = dut->s_rvalid && dut->s_rready;
                Word128 beat = get_w128(dut->s_rdata);
                mig_advance(mig);
                if (ar_fire_now) dut->s_arvalid = 0;
                if (r_fire_now) {
                    got = beat;
                    got_data = true;
                }
            }
            ok = ok && !dut->s_arvalid && got_data;
            auto it = golden.find(addr);
            Word128 expect = (it != golden.end()) ? it->second : Word128{0, 0, 0, 0};
            if (!(ok && got == expect)) {
                std::printf("  interleaved op %d: read addr=0x%08x got=%s want=%s\n",
                            i, addr, hex128(got).c_str(), hex128(expect).c_str());
                ok = false;
            }
        }
    }
    dut->s_awvalid = 0;
    dut->s_wvalid = 0;
    dut->s_arvalid = 0;
    eval();
    CHECK("interleaved_read_write_streams", ok);
}

// Explicit same-address write-then-read: without the T11 hazard check, a
// pipelined read issued right behind a still-in-flight write to the same
// 64B line could race ahead of it in the MIG and observe stale data.
// Confirms the read is held back until the write has actually landed and
// then returns the fresh value.
static void scenario_write_read_hazard_returns_new_data() {
    reset();
    dut->s_bready = 1;
    dut->s_rready = 1;
    MigModel mig(/*seed=*/7, /*stall_pct=*/0);
    const uint32_t addr = 0x00040000u;
    const uint8_t wid = 0x61, rid = 0x62;
    Word128 data{0xFEEDFACEu, 0xCAFEBABEu, 0xDEADC0DEu, 0x8BADF00Du};

    // Issue AW+W back-to-back, then IMMEDIATELY (same call sequence, no
    // extra settle delay beyond the AW/W accept cycles) issue the
    // overlapping AR, exercising the hazard stall.
    dut->s_awid = wid;
    dut->s_awaddr = addr;
    dut->s_awlen = 0;
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    set_w128(dut->s_wdata, data);
    dut->s_wstrb = 0xFFFF;
    dut->s_wlast = 1;
    dut->s_wvalid = 1;
    // NOTE: s_awvalid/s_wvalid must stay asserted THROUGH the tick() that
    // actually samples them (mig_advance() below) -- checking readiness,
    // then clearing the valid signal, then only afterward calling
    // mig_advance()/tick() would drop the signal before the clock edge
    // that was supposed to capture it.  Compute the fire flags from THIS
    // cycle's settled state, advance the clock, and only clear after.
    bool aw_ok = false, w_ok = false;
    for (int spins = 0; spins < 16 && (dut->s_awvalid || dut->s_wvalid); spins++) {
        mig_settle(mig);
        bool aw_fire_now = dut->s_awvalid && dut->s_awready;
        bool w_fire_now  = dut->s_wvalid && dut->s_wready;
        mig_advance(mig);
        if (aw_fire_now) { aw_ok = true; dut->s_awvalid = 0; }
        if (w_fire_now)  { w_ok = true;  dut->s_wvalid = 0; }
    }

    dut->s_arid = rid;
    dut->s_araddr = addr;
    dut->s_arlen = 0;
    dut->s_arsize = 4;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;

    Word128 got{};
    bool got_data = false;
    for (int spins = 0; spins < 32 && !got_data; spins++) {
        mig_settle(mig);
        bool ar_fire_now = dut->s_arvalid && dut->s_arready;
        bool r_fire_now  = dut->s_rvalid && dut->s_rready;
        Word128 beat = get_w128(dut->s_rdata);
        mig_advance(mig);
        if (ar_fire_now) dut->s_arvalid = 0;
        if (r_fire_now) {
            got = beat;
            got_data = true;
        }
    }
    dut->s_arvalid = 0;
    eval();

    bool ok = aw_ok && w_ok && got_data && got == data;
    if (!ok) {
        std::printf("  hazard: got=%s want=%s\n", hex128(got).c_str(), hex128(data).c_str());
    }
    CHECK("write_read_hazard_returns_new_data", ok);
}

// RLAST/beat-count integrity for aligned repo bursts at len=0 (1 beat),
// len=3 (4 beats), and len=255 (256 beats, the max AXI4 burst length) --
// confirms RLAST asserts on exactly the last beat and no more/fewer beats
// than promised are ever returned, for the pipelined drain path.
static void scenario_rlast_beat_count_integrity() {
    for (uint8_t len : {(uint8_t)0, (uint8_t)3, (uint8_t)255}) {
        reset();
        dut->s_rready = 1;
        MigModel mig(/*seed=*/99, /*stall_pct=*/0);
        const uint32_t addr = 0x00050000u;
        const uint8_t id = 0x70;

        // Pre-fill golden content so the returned data is checkable too.
        std::vector<Word128> expect;
        int repo_beats = (int)len + 1;
        for (int i = 0; i < repo_beats; i++) {
            uint32_t base = 0xB0000000u + (uint32_t)i * 0x10u;
            expect.push_back(Word128{base, base + 1, base + 2, base + 3});
        }
        // Seed the model's memory directly (byte-addressed, MIG address
        // space == repo address here since addr[4]=0 / no remap needed
        // for this 4 B-aligned test address).
        for (int i = 0; i < repo_beats; i++) {
            uint32_t a = addr + (uint32_t)i * 16u;
            Word256 beat = mig.read_beat(a & ~0x1Fu);
            int half = (a >> 4) & 1;
            for (int w = 0; w < 4; w++) {
                beat[half * 4 + w] = expect[i][w];
            }
            mig.write_beat(a & ~0x1Fu, beat, 0xFFFFFFFFu);
        }

        dut->s_arid = id;
        dut->s_araddr = addr;
        dut->s_arlen = len;
        dut->s_arsize = 4;
        dut->s_arburst = 1;
        dut->s_arvalid = 1;

        std::vector<Word128> got;
        std::vector<bool> got_last;
        bool ok = true;
        int guard = 0;
        while ((int)got.size() < repo_beats && guard < repo_beats * 8 + 64) {
            mig_settle(mig);
            bool ar_fire_now = dut->s_arvalid && dut->s_arready;
            bool r_fire_now  = dut->s_rvalid && dut->s_rready;
            uint8_t r_id_now = dut->s_rid;
            Word128 r_data_now = get_w128(dut->s_rdata);
            bool r_last_now = dut->s_rlast != 0;
            mig_advance(mig);
            if (ar_fire_now) dut->s_arvalid = 0;
            if (r_fire_now) {
                if (r_id_now != id) ok = false;
                got.push_back(r_data_now);
                got_last.push_back(r_last_now);
            }
            guard++;
        }
        dut->s_arvalid = 0;
        eval();

        ok = ok && (int)got.size() == repo_beats;
        for (int i = 0; ok && i < repo_beats; i++) {
            bool exp_last = (i == repo_beats - 1);
            if (got_last[i] != exp_last || !(got[i] == expect[i])) {
                std::printf("  rlast-integrity len=%u beat=%d: last=%d(want %d) data=%s want=%s\n",
                            (unsigned)len, i, (int)got_last[i], (int)exp_last,
                            hex128(got[i]).c_str(), hex128(expect[i]).c_str());
                ok = false;
            }
        }
        char name[64];
        std::snprintf(name, sizeof name, "rlast_beat_count_integrity_len%u", (unsigned)len);
        CHECK(name, ok);
    }
}

// Explicit multi-outstanding WRITE pipelining (review I4): queues
// WMAX_OUTSTANDING (4) bare AW commands (NO W data at all yet) while
// m_awready is held low, confirming the command queue is genuinely
// WMAX_OUTSTANDING deep and entirely decoupled from data streaming --
// AW1..AW3 accept while AW0 hasn't even been issued to the MIG, let
// alone drained.  (AXI4 requires W data to be delivered strictly in the
// same order as its AW and fully before the next burst's data starts --
// unlike AW/AR command acceptance, W data is NOT something this bridge,
// or any AXI4-compliant master, may legally interleave across IDs -- so
// "W beats behind an unissued AW" is tested by streaming each burst's
// full data only after its AW is already queued, not by racing multiple
// bursts' data concurrently.)  A 5th AW is then confirmed refused (queue
// genuinely full).  Releasing the MIG side and streaming each burst's
// full 2-beat data in turn then confirms all 4 drain correctly, in
// order, with the right id/data -- including bursts whose AW had to wait
// behind an earlier one still queued/unissued.
static void scenario_pipelined_writes_multi_outstanding() {
    reset();
    dut->s_bready = 1;
    MigModel mig(/*seed=*/55, /*stall_pct=*/0);

    const int N = 4;  // WMAX_OUTSTANDING
    const uint32_t base = 0x00080000u;
    struct WOp { uint32_t addr; uint8_t id; Word128 data0, data1; };
    std::vector<WOp> ops;
    for (int i = 0; i < N; i++) {
        ops.push_back(WOp{base + (uint32_t)i * 32u, (uint8_t)(0x90 + i),
                          Word128{0xF0000000u + (uint32_t)i, 0xF1000000u + (uint32_t)i,
                                  0xF2000000u + (uint32_t)i, 0xF3000000u + (uint32_t)i},
                          Word128{0xE0000000u + (uint32_t)i, 0xE1000000u + (uint32_t)i,
                                  0xE2000000u + (uint32_t)i, 0xE3000000u + (uint32_t)i}});
    }

    dut->m_awready = 0;
    dut->m_wready = 0;
    bool ok = true;
    for (int i = 0; i < N && ok; i++) {
        dut->s_awid = ops[i].id;
        dut->s_awaddr = ops[i].addr;
        dut->s_awlen = 1;  // 2 repo beats
        dut->s_awsize = 4;
        dut->s_awburst = 1;
        dut->s_awvalid = 1;
        eval();
        bool aw_fire_now = dut->s_awvalid && dut->s_awready;
        tick();
        if (aw_fire_now) {
            dut->s_awvalid = 0;
        } else {
            std::printf("  pipelined-writes: AW %d not accepted while MIG held off\n", i);
            ok = false;
        }
    }

    // The queue should now be exactly full: a 5th AW must be refused.
    dut->s_awid = 0x99;
    dut->s_awaddr = base + (uint32_t)N * 32u;
    dut->s_awlen = 1;
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    eval();
    bool fifth_blocked = !dut->s_awready;
    tick();
    dut->s_awvalid = 0;
    if (!fifth_blocked) {
        std::printf("  pipelined-writes: a 5th AW was accepted while the queue "
                    "should already be full at WMAX_OUTSTANDING=%d\n", N);
    }
    ok = ok && fifth_blocked;

    // Release the MIG side, then stream each burst's FULL data in turn
    // (AXI4 write-data ordering: one burst's data completely before the
    // next's); all N should drain, in order, with the right id/resp,
    // including the later bursts whose AW sat queued behind earlier
    // still-unissued ones.
    dut->m_awready = 1;
    dut->m_wready = 1;
    for (int i = 0; i < N && ok; i++) {
        const Word128* beats[2] = {&ops[i].data0, &ops[i].data1};
        for (int b = 0; b < 2 && ok; b++) {
            set_w128(dut->s_wdata, *beats[b]);
            dut->s_wstrb = 0xFFFF;
            dut->s_wlast = (b == 1) ? 1 : 0;
            dut->s_wvalid = 1;
            bool w_ok = false;
            for (int spins = 0; spins < 16 && !w_ok; spins++) {
                mig_settle(mig);
                bool w_fire_now = dut->s_wvalid && dut->s_wready;
                mig_advance(mig);
                if (w_fire_now) { w_ok = true; dut->s_wvalid = 0; }
            }
            if (!w_ok) {
                std::printf("  pipelined-writes: op %d beat%d never accepted after "
                            "MIG release\n", i, b);
                ok = false;
            }
        }
        bool committed = false;
        for (int spins = 0; spins < 64 && !committed; spins++) {
            mig_settle(mig);
            bool b_fire_now = dut->s_bvalid && dut->s_bready;
            uint8_t b_id_now = dut->s_bid;
            uint8_t b_resp_now = dut->s_bresp;
            mig_advance(mig);
            if (b_fire_now) {
                committed = true;
                if (b_id_now != ops[i].id || b_resp_now != 0) {
                    std::printf("  pipelined-writes: B #%d id=%u resp=%u want id=%u\n",
                                i, (unsigned)b_id_now, (unsigned)b_resp_now,
                                (unsigned)ops[i].id);
                    ok = false;
                }
            }
        }
        if (!committed) {
            std::printf("  pipelined-writes: B #%d never arrived\n", i);
            ok = false;
        }
    }

    // Confirm the data actually landed in the MIG model's memory for
    // each address (both beats), not just that B fired.
    for (int i = 0; i < N && ok; i++) {
        Word256 beat = mig.read_beat(ops[i].addr);
        Word128 got0{beat[0], beat[1], beat[2], beat[3]};
        Word128 got1{beat[4], beat[5], beat[6], beat[7]};
        if (!(got0 == ops[i].data0 && got1 == ops[i].data1)) {
            std::printf("  pipelined-writes: mig memory op %d got0=%s got1=%s "
                        "want0=%s want1=%s\n",
                        i, hex128(got0).c_str(), hex128(got1).c_str(),
                        hex128(ops[i].data0).c_str(), hex128(ops[i].data1).c_str());
            ok = false;
        }
    }

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    CHECK("pipelined_writes_multi_outstanding", ok);
}

// MINOR (review round 3): stream burst-0's W data to completion while
// its own AW (and three MORE AWs queued behind it) are still blocked at
// the MIG (m_awready held low throughout). Exercises the skid-vs-
// wq_aw_issued interaction under contention: burst-0's fully-packed
// 256-bit beats reach the local output skid entirely BEFORE its AW has
// ever been issued downstream (wq_aw_issued still 0 for that slot), with
// three more descriptors queuing up behind it, none of them issued
// either. Confirms all four drain correctly, in order, with the right
// id/data, once the MIG finally releases.
static void scenario_pipelined_write_data_streams_before_aw_issued() {
    reset();
    dut->s_bready = 1;
    MigModel mig(/*seed=*/77, /*stall_pct=*/0);

    const int N = 4;
    const uint32_t base = 0x00082000u;
    struct WOp { uint32_t addr; uint8_t id; Word128 data0, data1; };
    std::vector<WOp> ops;
    for (int i = 0; i < N; i++) {
        ops.push_back(WOp{base + (uint32_t)i * 32u, (uint8_t)(0xA0 + i),
                          Word128{0x10000000u + (uint32_t)i, 0x11000000u + (uint32_t)i,
                                  0x12000000u + (uint32_t)i, 0x13000000u + (uint32_t)i},
                          Word128{0x20000000u + (uint32_t)i, 0x21000000u + (uint32_t)i,
                                  0x22000000u + (uint32_t)i, 0x23000000u + (uint32_t)i}});
    }

    // B responses (s_bvalid) can fire autonomously as soon as a
    // descriptor's local skid has drained, which -- as this test
    // deliberately exercises -- can happen well before later descriptors
    // even finish streaming their data.  s_bready is tied high for the
    // whole test, so a B beat completes its handshake the instant it
    // appears regardless of whether the polling loop below happens to be
    // "watching" -- capture every B event continuously via one shared
    // step() used for every eval/tick in this test, rather than only
    // during a dedicated post-hoc collection loop (which would silently
    // miss any B that fired earlier and mis-attribute order).
    std::vector<std::pair<uint8_t, uint8_t>> b_events;  // (id, resp) in arrival order
    auto step = [&]() {
        mig.drive_outputs();
        eval();
        if (dut->s_bvalid && dut->s_bready) {
            b_events.push_back({dut->s_bid, dut->s_bresp});
        }
        mig.observe_and_advance();
        tick();
    };

    dut->m_awready = 0;
    dut->m_wready = 0;
    bool ok = true;

    // Accept AW0 upstream, then stream ALL of burst-0's data upstream too
    // -- entirely before any AW (including its own) has ever reached the
    // MIG. Burst-0's data must be free to drain into the local skid
    // (wr_data_active does not require wq_aw_issued) while its own AW
    // sits unissued.
    dut->s_awid = ops[0].id;
    dut->s_awaddr = ops[0].addr;
    dut->s_awlen = 1;  // 2 repo beats -> 1 MIG beat
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    mig.drive_outputs();
    eval();
    bool aw0_fire = dut->s_awvalid && dut->s_awready;
    if (dut->s_bvalid && dut->s_bready) {
        b_events.push_back({dut->s_bid, dut->s_bresp});
    }
    mig.observe_and_advance();
    tick();
    if (aw0_fire) dut->s_awvalid = 0;
    if (!aw0_fire) { std::printf("  data-before-aw: AW0 not accepted\n"); ok = false; }

    const Word128* beats0[2] = {&ops[0].data0, &ops[0].data1};
    for (int b = 0; b < 2 && ok; b++) {
        set_w128(dut->s_wdata, *beats0[b]);
        dut->s_wstrb = 0xFFFF;
        dut->s_wlast = (b == 1) ? 1 : 0;
        dut->s_wvalid = 1;
        bool w_ok = false;
        for (int spins = 0; spins < 16 && !w_ok; spins++) {
            mig.drive_outputs();
            eval();
            bool w_fire_now = dut->s_wvalid && dut->s_wready;
            if (dut->s_bvalid && dut->s_bready) {
                b_events.push_back({dut->s_bid, dut->s_bresp});
            }
            mig.observe_and_advance();
            tick();
            if (w_fire_now) { w_ok = true; dut->s_wvalid = 0; }
        }
        if (!w_ok) {
            std::printf("  data-before-aw: burst0 beat%d never accepted while its AW is "
                        "still unissued\n", b);
            ok = false;
        }
    }

    // Queue AWs 1-3 behind it -- still no MIG release. op0 remains at the
    // head of the AW-issue ring, unissued, with its data already sitting
    // fully drained into the skid.
    for (int i = 1; i < N && ok; i++) {
        dut->s_awid = ops[i].id;
        dut->s_awaddr = ops[i].addr;
        dut->s_awlen = 1;
        dut->s_awsize = 4;
        dut->s_awburst = 1;
        dut->s_awvalid = 1;
        mig.drive_outputs();
        eval();
        bool aw_fire_now = dut->s_awvalid && dut->s_awready;
        if (dut->s_bvalid && dut->s_bready) {
            b_events.push_back({dut->s_bid, dut->s_bresp});
        }
        mig.observe_and_advance();
        tick();
        if (aw_fire_now) {
            dut->s_awvalid = 0;
        } else {
            std::printf("  data-before-aw: AW %d not accepted while MIG held off\n", i);
            ok = false;
        }
    }

    // Release the MIG: op0's already-streamed data must drain correctly
    // once its long-unissued AW finally goes out, then ops 1-3 stream
    // their data normally.
    dut->m_awready = 1;
    dut->m_wready = 1;
    for (int i = 1; i < N && ok; i++) {
        const Word128* beats[2] = {&ops[i].data0, &ops[i].data1};
        for (int b = 0; b < 2 && ok; b++) {
            set_w128(dut->s_wdata, *beats[b]);
            dut->s_wstrb = 0xFFFF;
            dut->s_wlast = (b == 1) ? 1 : 0;
            dut->s_wvalid = 1;
            bool w_ok = false;
            for (int spins = 0; spins < 16 && !w_ok; spins++) {
                mig.drive_outputs();
                eval();
                bool w_fire_now = dut->s_wvalid && dut->s_wready;
                if (dut->s_bvalid && dut->s_bready) {
                    b_events.push_back({dut->s_bid, dut->s_bresp});
                }
                mig.observe_and_advance();
                tick();
                if (w_fire_now) { w_ok = true; dut->s_wvalid = 0; }
            }
            if (!w_ok) {
                std::printf("  data-before-aw: op %d beat%d never accepted after MIG "
                            "release\n", i, b);
                ok = false;
            }
        }
    }

    // Drain any remaining B events (all four should have arrived by now,
    // but give it a healthy margin).
    for (int spins = 0; spins < 64 && ok && b_events.size() < (size_t)N; spins++) {
        step();
    }

    if (b_events.size() != (size_t)N) {
        std::printf("  data-before-aw: saw %zu B events, want %d\n", b_events.size(), N);
        ok = false;
    } else {
        for (int i = 0; i < N; i++) {
            if (b_events[i].first != ops[i].id || b_events[i].second != 0) {
                std::printf("  data-before-aw: B #%d id=%u resp=%u want id=%u\n",
                            i, (unsigned)b_events[i].first, (unsigned)b_events[i].second,
                            (unsigned)ops[i].id);
                ok = false;
            }
        }
    }

    for (int i = 0; i < N && ok; i++) {
        Word256 beat = mig.read_beat(ops[i].addr);
        Word128 got0{beat[0], beat[1], beat[2], beat[3]};
        Word128 got1{beat[4], beat[5], beat[6], beat[7]};
        if (!(got0 == ops[i].data0 && got1 == ops[i].data1)) {
            std::printf("  data-before-aw: mig memory op %d got0=%s got1=%s want0=%s "
                        "want1=%s\n",
                        i, hex128(got0).c_str(), hex128(got1).c_str(),
                        hex128(ops[i].data0).c_str(), hex128(ops[i].data1).c_str());
            ok = false;
        }
    }

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    CHECK("pipelined_write_data_streams_before_aw_issued", ok);
}

// Malformed-mid-burst containment (review I1): beat0+beat1 of a 4-beat
// promised burst form a legitimate MIG pair and get pushed for real;
// beat2 arrives with WLAST asserted early (a placement mismatch -- the
// AW promised 4 beats).  Confirms: (a) the repo-side response is still
// SLVERR with the right id; (b) the MIG nonetheless receives EXACTLY the
// originally-promised MIG beat count (2) -- the real pair0 beat, then a
// wstrb=0 padding beat completing the burst (a real MIG needs exact
// AWLEN framing and can't be short-changed without wedging its address
// counter for every later transfer); (c) a completely unrelated write
// issued right after still completes normally (no wedge).
static void scenario_malformed_burst_completes_with_padding() {
    reset();
    dut->s_bready = 1;
    dut->m_awready = 1;
    dut->m_wready = 1;

    const uint32_t addr = 0x00090000u;  // addr[4]=0, 32B-aligned
    const uint8_t id = 0xA1;
    Word128 a{0xAAAA0000u, 0xAAAA0001u, 0xAAAA0002u, 0xAAAA0003u};  // beat0
    Word128 b{0xBBBB0000u, 0xBBBB0001u, 0xBBBB0002u, 0xBBBB0003u};  // beat1 (completes pair0)
    Word128 c{0xCCCC0000u, 0xCCCC0001u, 0xCCCC0002u, 0xCCCC0003u};  // beat2 (malformed: early WLAST)

    dut->s_awid = id;
    dut->s_awaddr = addr;
    dut->s_awlen = 3;   // 4 repo beats promised -> 2 MIG beats
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    eval();
    bool ok = dut->s_awready;

    std::vector<MigWriteBeat> captured;
    bool aw_captured = false;
    uint32_t cap_awaddr = 0;
    bool sb_captured = false;
    uint8_t cap_bid = 0, cap_bresp = 0;

    auto capture_this_cycle = [&]() {
        if (dut->m_awvalid && dut->m_awready && !aw_captured) {
            aw_captured = true;
            cap_awaddr = dut->m_awaddr;
        }
        if (dut->m_wvalid && dut->m_wready) {
            captured.push_back(MigWriteBeat{get_w256(dut->m_wdata),
                                            (uint32_t)dut->m_wstrb, dut->m_wlast != 0});
        }
        if (dut->s_bvalid && !sb_captured) {
            sb_captured = true;
            cap_bid = dut->s_bid;
            cap_bresp = dut->s_bresp;
        }
    };

    auto drive_beat = [&](const Word128& data, bool wlast) -> bool {
        set_w128(dut->s_wdata, data);
        dut->s_wstrb = 0xFFFF;
        dut->s_wlast = wlast ? 1 : 0;
        dut->s_wvalid = 1;
        bool beat_ok = false;
        for (int spins = 0; spins < 16 && !beat_ok; spins++) {
            eval();
            bool aw_fire_now = dut->s_awvalid && dut->s_awready;
            bool w_fire_now  = dut->s_wvalid && dut->s_wready;
            capture_this_cycle();
            tick();
            if (aw_fire_now) dut->s_awvalid = 0;
            if (w_fire_now) { beat_ok = true; dut->s_wvalid = 0; }
        }
        return beat_ok;
    };

    ok = ok && drive_beat(a, false);
    ok = ok && drive_beat(b, false);
    ok = ok && drive_beat(c, true);  // malformed: WLAST two beats early

    // Local B and both expected MIG beats may lag a few cycles behind the
    // last W beat's acceptance -- keep polling (and capturing) until both
    // have shown up or the guard expires.
    for (int spins = 0; spins < 32 && (!sb_captured || captured.size() < 2); spins++) {
        eval();
        capture_this_cycle();
        tick();
    }

    ok = ok && sb_captured && cap_bid == id && cap_bresp == 2;
    if (!(sb_captured && cap_bid == id && cap_bresp == 2)) {
        std::printf("  malformed-padding: local resp captured=%d bid=%u bresp=%u\n",
                    (int)sb_captured, (unsigned)cap_bid, (unsigned)cap_bresp);
    }

    ok = ok && aw_captured && cap_awaddr == addr && captured.size() == 2;
    if (aw_captured && cap_awaddr == addr && captured.size() == 2) {
        Word256 exp_pair0{a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]};
        if (!(captured[0].data == exp_pair0 && captured[0].strb == 0xFFFFFFFFu &&
              !captured[0].last)) {
            std::printf("  malformed-padding: pair0 mismatch strb=0x%08x last=%d data=%s\n",
                        captured[0].strb, (int)captured[0].last,
                        hex256(captured[0].data).c_str());
            ok = false;
        }
        if (!(captured[1].strb == 0 && captured[1].last)) {
            std::printf("  malformed-padding: padding beat strb=0x%08x last=%d data=%s\n",
                        captured[1].strb, (int)captured[1].last,
                        hex256(captured[1].data).c_str());
            ok = false;
        }
    } else {
        std::printf("  malformed-padding: captured %zu mig beats (want 2), aw_captured=%d "
                    "addr=0x%08x want=0x%08x\n",
                    captured.size(), (int)aw_captured, cap_awaddr, addr);
        ok = false;
    }

    // No wedge: a completely unrelated write right after should still
    // complete cleanly.
    Word128 d{0xD0D0D0D0u, 0xD1D1D1D1u, 0xD2D2D2D2u, 0xD3D3D3D3u};
    bool wr_ok = accept_write(0x000A0000u, 0xB2, d, 0xFFFFu);
    Word256 exp_d{d[0], d[1], d[2], d[3], 0, 0, 0, 0};
    wr_ok = wr_ok && forwarded_write_matches(0x000A0000u, 0xFFFFu, exp_d);
    wr_ok = wr_ok && finish_write_response(0xB2);
    if (!wr_ok) {
        std::printf("  malformed-padding: post-malformed write failed to complete cleanly\n");
    }
    ok = ok && wr_ok;

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    CHECK("malformed_burst_completes_with_padding", ok);
}

// Review round-3 Important-1 regression test: a malformed burst whose
// WLAST-placement mismatch surfaces on the VERY FIRST beat -- i.e. ZERO
// real MIG beats ever pushed before the mismatch is seen -- while the AW
// has already been issued (or is irrevocably committed to issuing) to the
// MIG, because AW issuance is eager and independent of W-data streaming.
// Before the round-3 fix, wr_pad_trigger only fired when a prior
// well-formed prefix had been pushed (wr_real_need_push ||
// wr_mig_pushed_count != 0), so this exact case left the MIG-side burst
// permanently short-changed: the promised beats never arrived, the
// hazard-tracking bq_ entry never popped (any overlapping read would be
// blocked forever), and if the MIG happened to be stalling m_awready,
// m_awvalid would even retract mid-presentation without a handshake
// (AXI4 A3.2.1 violation). The round-3 fix (sticky aw_presenting +
// wq_aw_committed) keys the pad obligation on "AW issued or issuing"
// instead of "prior prefix pushed", so a zero-prefix-but-committed
// descriptor pads its ENTIRE promised beat count with wstrb=0.
static void scenario_malformed_burst_zero_prefix_padding() {
    reset();
    dut->s_bready = 1;
    dut->m_awready = 1;
    dut->m_wready = 1;

    const uint32_t addr = 0x00090000u;  // addr[4]=0, 32B-aligned
    const uint8_t id = 0xA5;
    Word128 a{0xAAAA0000u, 0xAAAA0001u, 0xAAAA0002u, 0xAAAA0003u};  // beat0

    dut->s_awid = id;
    dut->s_awaddr = addr;
    dut->s_awlen = 3;   // 4 repo beats promised -> 2 MIG beats
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    eval();
    bool ok = dut->s_awready;

    std::vector<MigWriteBeat> captured;
    bool aw_captured = false;
    uint32_t cap_awaddr = 0;
    bool sb_captured = false;
    uint8_t cap_bid = 0, cap_bresp = 0;

    auto capture_this_cycle = [&]() {
        if (dut->m_awvalid && dut->m_awready && !aw_captured) {
            aw_captured = true;
            cap_awaddr = dut->m_awaddr;
        }
        if (dut->m_wvalid && dut->m_wready) {
            captured.push_back(MigWriteBeat{get_w256(dut->m_wdata),
                                            (uint32_t)dut->m_wstrb, dut->m_wlast != 0});
        }
        if (dut->s_bvalid && !sb_captured) {
            sb_captured = true;
            cap_bid = dut->s_bid;
            cap_bresp = dut->s_bresp;
        }
    };

    auto drive_beat = [&](const Word128& data, bool wlast) -> bool {
        set_w128(dut->s_wdata, data);
        dut->s_wstrb = 0xFFFF;
        dut->s_wlast = wlast ? 1 : 0;
        dut->s_wvalid = 1;
        bool beat_ok = false;
        for (int spins = 0; spins < 16 && !beat_ok; spins++) {
            eval();
            bool aw_fire_now = dut->s_awvalid && dut->s_awready;
            bool w_fire_now  = dut->s_wvalid && dut->s_wready;
            capture_this_cycle();
            tick();
            if (aw_fire_now) dut->s_awvalid = 0;
            if (w_fire_now) { beat_ok = true; dut->s_wvalid = 0; }
        }
        return beat_ok;
    };

    // Give the AW a few free cycles to reach the MIG BEFORE any W beat
    // arrives -- reproduces the eager-AW-issuance race from the review.
    for (int spins = 0; spins < 3; spins++) {
        eval();
        bool aw_fire_now = dut->s_awvalid && dut->s_awready;
        capture_this_cycle();
        tick();
        if (aw_fire_now) dut->s_awvalid = 0;  // single accept only -- do
                                               // not let AW re-fire on
                                               // idle cycles
    }
    ok = ok && aw_captured;  // AW must already be at the MIG by construction

    ok = ok && drive_beat(a, true);  // malformed: WLAST on the very FIRST
                                      // beat -- zero prior real beats pushed

    for (int spins = 0; spins < 32 && (!sb_captured || captured.size() < 2); spins++) {
        eval();
        capture_this_cycle();
        tick();
    }

    ok = ok && sb_captured && cap_bid == id && cap_bresp == 2;
    if (!(sb_captured && cap_bid == id && cap_bresp == 2)) {
        std::printf("  malformed-zero-prefix: local resp captured=%d bid=%u bresp=%u\n",
                    (int)sb_captured, (unsigned)cap_bid, (unsigned)cap_bresp);
    }

    // The ENTIRE promised burst (2 MIG beats) must be delivered as pure
    // padding -- the AW was already committed before any real data
    // arrived, so it owes its full promised beat count, not merely the
    // beats beyond a (nonexistent) prefix.
    ok = ok && aw_captured && cap_awaddr == addr && captured.size() == 2;
    if (aw_captured && cap_awaddr == addr && captured.size() == 2) {
        if (!(captured[0].strb == 0 && !captured[0].last)) {
            std::printf("  malformed-zero-prefix: pad beat0 strb=0x%08x last=%d\n",
                        captured[0].strb, (int)captured[0].last);
            ok = false;
        }
        if (!(captured[1].strb == 0 && captured[1].last)) {
            std::printf("  malformed-zero-prefix: pad beat1 strb=0x%08x last=%d\n",
                        captured[1].strb, (int)captured[1].last);
            ok = false;
        }
    } else {
        std::printf("  malformed-zero-prefix: captured %zu mig beats (want 2), aw_captured=%d "
                    "addr=0x%08x want=0x%08x\n",
                    captured.size(), (int)aw_captured, cap_awaddr, addr);
        ok = false;
    }

    // No wedge: a completely unrelated write right after should still
    // complete cleanly (the MIG's address counter was not left mid-burst).
    Word128 d{0xD0D0D0D0u, 0xD1D1D1D1u, 0xD2D2D2D2u, 0xD3D3D3D3u};
    bool wr_ok = accept_write(0x000A0000u, 0xB3, d, 0xFFFFu);
    Word256 exp_d{d[0], d[1], d[2], d[3], 0, 0, 0, 0};
    wr_ok = wr_ok && forwarded_write_matches(0x000A0000u, 0xFFFFu, exp_d);
    wr_ok = wr_ok && finish_write_response(0xB3);
    if (!wr_ok) {
        std::printf("  malformed-zero-prefix: post-malformed write failed to complete cleanly\n");
    }
    ok = ok && wr_ok;

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    CHECK("malformed_burst_zero_prefix_padding", ok);
}

// Review round-3 re-review Fix 1(a) regression test: an accept-time-
// REJECTED write (misaligned address -> aw_ok=0, so its AW must NEVER
// reach the MIG) whose data stream ALSO carries an early-WLAST placement
// mismatch. Before the fix, the AW-issue SKIP path (a rejected-and-
// untouched descriptor's AW is never sent to the MIG) incorrectly marked
// wq_aw_issued=1 for the skipped descriptor, so wq_aw_committed read that
// flag as "AW committed" and armed wr_pad_trigger -- pushing wstrb=0
// padding W beats (including a synthetic WLAST) to the MIG for a burst
// that never had a command in front of it. Expected-correct behavior:
// pure local SLVERR, ZERO m_aw handshakes, ZERO m_w beats for the
// malformed op, and a subsequent legal write still completes cleanly.
static void scenario_rejected_aw_early_wlast_no_orphan() {
    reset();
    dut->s_bready = 1;
    dut->m_awready = 1;
    dut->m_wready = 1;

    int m_aw = 0, m_w = 0;
    bool sb = false;
    uint8_t sb_resp = 0;

    // Misaligned (addr[3:0] != 0, size=4) len=3 write: locally rejected.
    dut->s_awid = 0x44;
    dut->s_awaddr = 0x000B0004u;
    dut->s_awlen = 3;
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    for (int t = 0; t < 6; t++) {
        eval();
        bool aw_f = dut->s_awvalid && dut->s_awready;
        if (dut->m_awvalid && dut->m_awready) m_aw++;
        tick();
        if (aw_f) dut->s_awvalid = 0;
    }

    // beat0 with EARLY WLAST (burst promised 4 repo beats).
    Word128 bad{0x0BAD0000u, 0x0BAD0001u, 0x0BAD0002u, 0x0BAD0003u};
    set_w128(dut->s_wdata, bad);
    dut->s_wstrb = 0xFFFF;
    dut->s_wlast = 1;
    dut->s_wvalid = 1;
    for (int t = 0; t < 40; t++) {
        eval();
        bool w_f = dut->s_wvalid && dut->s_wready;
        if (dut->m_awvalid && dut->m_awready) m_aw++;
        if (dut->m_wvalid && dut->m_wready) m_w++;
        if (dut->s_bvalid && dut->s_bready && !sb) { sb = true; sb_resp = dut->s_bresp; }
        tick();
        if (w_f) dut->s_wvalid = 0;
    }

    bool ok = (m_aw == 0) && (m_w == 0) && sb && (sb_resp == 2);
    if (!ok) {
        std::printf("  rejected-aw-early-wlast: mig_aw=%d mig_w=%d sb=%d resp=%u "
                    "(want 0/0/1/2)\n", m_aw, m_w, (int)sb, (unsigned)sb_resp);
    }

    // Follow-up legal write must still complete cleanly (no wedge left
    // behind by the rejected/malformed op).
    Word128 d{0x600D0000u, 0x600D0001u, 0x600D0002u, 0x600D0003u};
    bool wr_ok = accept_write(0x000C0000u, 0x55, d, 0xFFFFu);
    Word256 exp_d{d[0], d[1], d[2], d[3], 0, 0, 0, 0};
    wr_ok = wr_ok && forwarded_write_matches(0x000C0000u, 0xFFFFu, exp_d);
    wr_ok = wr_ok && finish_write_response(0x55);
    if (!wr_ok) {
        std::printf("  rejected-aw-early-wlast: follow-up legal write failed to complete\n");
    }
    ok = ok && wr_ok;

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    CHECK("rejected_aw_early_wlast_no_orphan", ok);
}

// Review round-3 re-review Fix 1(b) regression test: warms all
// WMAX_OUTSTANDING ring slots with legal, fully-retired single-beat
// writes (each leaves wq_aw_issued=1 behind in its physical slot), then
// sends a LEGAL single-beat AW with SAME-CYCLE W data whose WLAST is
// wrong (wlast=0 on an AWLEN=0 burst -- a "missing WLAST" placement
// mismatch). This exercises the wr_bypass coincidence on a physical slot
// whose PREVIOUS occupant really did have its AW issued: before the fix,
// wq_aw_committed's first disjunct read wq_aw_issued[wq_data_ptr] during
// the SAME cycle the new descriptor is being accepted -- the array read
// that cycle is combinationally STALE (it returns the previous
// occupant's value, since the new descriptor's own register write
// hasn't landed yet) -- so wr_pad_trigger incorrectly armed for a
// descriptor whose OWN AW was never issued, pushing 1 orphan wstrb=0 W
// beat to the MIG with zero AW handshake for it. Expected-correct
// behavior: pure local SLVERR, ZERO m_aw/m_w activity for the malformed
// op, and a subsequent legal write still completes cleanly.
static void scenario_warm_ring_stale_bypass_no_orphan() {
    reset();
    dut->s_bready = 1;
    MigModel mig(/*seed=*/91, /*stall_pct=*/0);

    const int N = 4;  // WMAX_OUTSTANDING
    dut->m_awready = 1;
    dut->m_wready = 1;
    bool ok = true;

    // Warm all N slots: legal single-beat writes, driven through the
    // real MigModel so each one is genuinely accepted, issued, and
    // retired (leaves wq_aw_issued=1 behind in every physical ring slot).
    for (int i = 0; i < N; i++) {
        Word128 d{0x600D0000u + (uint32_t)i, 0x600D0001u + (uint32_t)i,
                  0x600D0002u + (uint32_t)i, 0x600D0003u + (uint32_t)i};
        dut->s_awid = (uint8_t)(0x10 + i);
        dut->s_awaddr = 0x000D0000u + (uint32_t)i * 64u;
        dut->s_awlen = 0;
        dut->s_awsize = 4;
        dut->s_awburst = 1;
        dut->s_awvalid = 1;
        set_w128(dut->s_wdata, d);
        dut->s_wstrb = 0xFFFF;
        dut->s_wlast = 1;
        dut->s_wvalid = 1;
        for (int t = 0; t < 20; t++) {
            mig_settle(mig);
            bool aw_f = dut->s_awvalid && dut->s_awready;
            bool w_f  = dut->s_wvalid && dut->s_wready;
            mig_advance(mig);
            if (aw_f) dut->s_awvalid = 0;
            if (w_f)  dut->s_wvalid = 0;
        }
    }

    // Slot 0 (physically) is now stale: wq_aw_issued=1 left over from
    // write #0. Legal AW (len=0), same-cycle W with WRONG WLAST.
    int aw5 = 0, w5 = 0;
    bool sb = false;
    uint8_t sb_resp = 0, sb_id = 0;
    dut->s_awid = 0x77;
    dut->s_awaddr = 0x000E0000u;
    dut->s_awlen = 0;
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    Word128 bad{0x0BAD0BADu, 0x0BAD0BADu, 0x0BAD0BADu, 0x0BAD0BADu};
    set_w128(dut->s_wdata, bad);
    dut->s_wstrb = 0xFFFF;
    dut->s_wlast = 0;  // WLAST violation: should be 1 on a len=0 burst
    dut->s_wvalid = 1;
    for (int t = 0; t < 40; t++) {
        mig_settle(mig);
        bool aw_f = dut->s_awvalid && dut->s_awready;
        bool w_f  = dut->s_wvalid && dut->s_wready;
        if (dut->m_awvalid && dut->m_awready) aw5++;
        if (dut->m_wvalid && dut->m_wready) w5++;
        if (dut->s_bvalid && dut->s_bready && !sb) {
            sb = true; sb_resp = dut->s_bresp; sb_id = dut->s_bid;
        }
        mig_advance(mig);
        if (aw_f) dut->s_awvalid = 0;
        if (w_f)  dut->s_wvalid = 0;
    }

    if (!(aw5 == 0 && w5 == 0 && sb && sb_id == 0x77 && sb_resp == 2)) {
        std::printf("  warm-ring-stale-bypass: mig_aw=%d mig_w=%d sb=%d id=0x%02x "
                    "resp=%u (want 0/0/1/0x77/2)\n",
                    aw5, w5, (int)sb, (unsigned)sb_id, (unsigned)sb_resp);
        ok = false;
    }

    // Follow-up legal write must still complete cleanly.
    Word128 e{0xE0000000u, 0xE0000001u, 0xE0000002u, 0xE0000003u};
    dut->s_awid = 0x88;
    dut->s_awaddr = 0x000F0000u;
    dut->s_awlen = 0;
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    set_w128(dut->s_wdata, e);
    dut->s_wstrb = 0xFFFF;
    dut->s_wlast = 1;
    dut->s_wvalid = 1;
    bool sb2 = false;
    for (int t = 0; t < 40 && !sb2; t++) {
        mig_settle(mig);
        bool aw_f = dut->s_awvalid && dut->s_awready;
        bool w_f  = dut->s_wvalid && dut->s_wready;
        if (dut->s_bvalid && dut->s_bready && dut->s_bid == 0x88) sb2 = true;
        mig_advance(mig);
        if (aw_f) dut->s_awvalid = 0;
        if (w_f)  dut->s_wvalid = 0;
    }
    if (!sb2) {
        std::printf("  warm-ring-stale-bypass: follow-up legal write never completed\n");
    }
    ok = ok && sb2;

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    CHECK("warm_ring_stale_bypass_no_orphan", ok);
}

// T15 investigation regression: back-to-back, SEPARATE single-beat
// (AWLEN=0) AXI writes whose addresses alternate the lower and upper
// half of the SAME 32B-aligned MIG beat (offset+0 then offset+16, each
// its own independent AW+W+B transaction, waiting for B before the
// next -- exactly the wr_bypass condition on every single one, since the
// write ring is empty again after each transaction fully retires).
// This is the exact traffic shape a T14 VRAM-in-DDR integration probe
// (tb_fb_reader_ddr_chain.cpp on feat/vram-ddr-migration) used to seed
// its golden framebuffer data before streaming it back through the real
// DDR read chain -- that probe reported a "read-data corruption" signature
// (beat 0 correct, beats 1-3 replay beat 0's own bytes) that looked
// exactly like a half-select bug in this bridge's read-side splitter.
// Root-caused via direct instrumentation (mem_lane-level tracing in
// sim_mig_backend.v + wr_bypass-level tracing in this bridge): the WRITE
// pairing was actually WRONG at its source -- the probe's own C++ driver
// never deasserted cpu_wvalid after a successful W handshake (a bug in
// tb_fb_reader_ddr_chain.cpp's cpu_w_drive_and_latch(), not in any RTL
// this repo owns), letting axi_async_bridge's W-FIFO see spurious repeat
// pushes of stale W data that later transactions' AWs incorrectly paired
// with. Fixing ONLY that C++ driver (zero RTL changes anywhere) made the
// same integration probe pass 4096/4096 pixels clean. This scenario
// captures the SHAPE of traffic that surfaced the confusion as a
// permanent regression against this bridge specifically, with a
// correctly-behaved (AXI4-compliant) driver: proves the bridge's own
// wr_bypass/half-packing logic was never at fault for this pattern.
static void scenario_back_to_back_single_beat_half_packing() {
    reset();
    dut->s_bready = 1;
    MigModel mig(/*seed=*/151, /*stall_pct=*/0);

    const uint32_t base = 0x00090000u;
    const int N_BEATS = 6;  // 6 separate 32B MIG beats, each written as
                             // two independent 16B halves
    bool ok = true;
    uint8_t next_id = 0x60;

    auto write_one = [&](uint32_t addr, uint32_t strb, uint32_t w0) -> bool {
        Word128 d{w0, w0 + 1u, w0 + 2u, w0 + 3u};
        dut->s_awid = next_id++;
        dut->s_awaddr = addr;
        dut->s_awlen = 0;
        dut->s_awsize = 4;
        dut->s_awburst = 1;
        dut->s_awvalid = 1;
        set_w128(dut->s_wdata, d);
        dut->s_wstrb = (uint16_t)strb;
        dut->s_wlast = 1;
        dut->s_wvalid = 1;
        bool committed = false;
        for (int spins = 0; spins < 32 && !committed; spins++) {
            mig_settle(mig);
            bool aw_fire_now = dut->s_awvalid && dut->s_awready;
            bool w_fire_now  = dut->s_wvalid && dut->s_wready;
            bool b_fire_now  = dut->s_bvalid != 0;
            mig_advance(mig);
            if (aw_fire_now) dut->s_awvalid = 0;
            if (w_fire_now) dut->s_wvalid = 0;
            if (b_fire_now) committed = true;
        }
        return committed;
    };

    for (int i = 0; i < N_BEATS && ok; i++) {
        uint32_t beat_addr = base + (uint32_t)i * 32u;
        uint32_t lo0 = 0xA0000000u + (uint32_t)i * 0x100u;
        uint32_t hi0 = 0xB0000000u + (uint32_t)i * 0x100u;
        bool wl = write_one(beat_addr,      0xFFFFu, lo0);
        bool wh = write_one(beat_addr + 16, 0xFFFFu, hi0);
        if (!(wl && wh)) {
            std::printf("  back-to-back-half-packing: beat %d write(s) failed to "
                        "complete (lo=%d hi=%d)\n", i, (int)wl, (int)wh);
            ok = false;
            continue;
        }
        Word256 got = mig.read_beat(beat_addr);
        Word128 got_lo{got[0], got[1], got[2], got[3]};
        Word128 got_hi{got[4], got[5], got[6], got[7]};
        Word128 want_lo{lo0, lo0 + 1u, lo0 + 2u, lo0 + 3u};
        Word128 want_hi{hi0, hi0 + 1u, hi0 + 2u, hi0 + 3u};
        if (!(got_lo == want_lo && got_hi == want_hi)) {
            std::printf("  back-to-back-half-packing: beat %d addr=0x%08x got_lo=%s "
                        "want_lo=%s got_hi=%s want_hi=%s\n",
                        i, beat_addr, hex128(got_lo).c_str(), hex128(want_lo).c_str(),
                        hex128(got_hi).c_str(), hex128(want_hi).c_str());
            ok = false;
        }
    }

    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();
    CHECK("back_to_back_single_beat_half_packing", ok);
}

// T15 investigation regression: back-to-back 4-beat/64B reads (AWLEN=3,
// 16B/repo-beat -- the exact shape rtl/soc/scanout_line_fetch.v issues
// against this bridge in the real VRAM-in-DDR integration), sweeping
// start-address alignment (addr[4]=0, the ONLY value the real caller
// ever produces since it's always 64B-line-aligned, plus addr[4]=1 for
// defensive coverage of the odd-alignment repo/mig-beat crossing case),
// each subsequent AR presented the cycle immediately after the previous
// one is accepted (RMAX_OUTSTANDING allows several genuinely in flight
// at the bridge's own ring level) -- the "aggressive back-to-back, >=2
// outstanding" cadence a T14 integration probe's failure was originally
// attributed to. Every beat is checked byte-exact against a per-address
// golden pattern. (See scenario_back_to_back_single_beat_half_packing's
// header: T15 root-caused that probe's failure to its OWN write-phase C++
// driver, not to this bridge's read path -- this scenario is the
// permanent regression proving the read path specifically, independent
// of that driver bug.)
static void scenario_back_to_back_4beat_reads_alignment_sweep() {
    for (int align = 0; align < 2; align++) {
        reset();
        dut->s_rready = 1;
        MigModel mig(/*seed=*/(uint32_t)(271 + align), /*stall_pct=*/0);

        const uint32_t region = 0x000A0000u + (uint32_t)align * 0x4000u;
        const int LINES = 6;
        auto golden_byte = [](uint32_t addr) -> uint8_t { return (uint8_t)(addr & 0xFFu); };

        // Seed memory directly (bypassing AXI -- isolates the read path).
        for (uint32_t off = 0; off < (uint32_t)LINES * 64u + 32u; off++) {
            uint32_t addr = region + off;
            uint32_t beat_addr = addr & ~0x1Fu;
            uint32_t lane = addr & 0x1Fu;
            Word256 cur = mig.read_beat(beat_addr);
            uint32_t word = cur[lane / 4];
            uint32_t shift = (lane % 4) * 8;
            word = (word & ~(0xFFu << shift)) | ((uint32_t)golden_byte(addr) << shift);
            cur[lane / 4] = word;
            mig.write_beat(beat_addr, cur, 0xFFFFFFFFu);
        }

        std::vector<uint32_t> pending_addr;
        int lines_issued = 0;
        bool armed = false;
        uint8_t next_id = 0x70;
        int checked = 0, mismatches = 0;
        int guard = 0;
        const int GUARD_MAX = 20000;
        std::vector<int> beats_left;

        while ((lines_issued < LINES || !pending_addr.empty()) && guard++ < GUARD_MAX) {
            if (lines_issued < LINES && !armed) {
                uint32_t addr = region + (uint32_t)lines_issued * 64u + (align ? 16u : 0u);
                dut->s_arid = next_id++;
                dut->s_araddr = addr;
                dut->s_arlen = 3;
                dut->s_arsize = 4;
                dut->s_arburst = 1;
                dut->s_arvalid = 1;
                armed = true;
            }
            mig_settle(mig);
            bool ar_fire = dut->s_arvalid && dut->s_arready;
            bool r_fire  = dut->s_rvalid && dut->s_rready;
            uint32_t r_word0 = dut->s_rdata[0];
            uint8_t r_resp = dut->s_rresp;
            mig_advance(mig);

            if (ar_fire) {
                uint32_t addr = region + (uint32_t)lines_issued * 64u + (align ? 16u : 0u);
                pending_addr.push_back(addr);
                beats_left.push_back(4);
                lines_issued++;
                dut->s_arvalid = 0;
                armed = false;
            }
            if (r_fire) {
                if (pending_addr.empty()) {
                    std::printf("  back-to-back-4beat-reads[align=%d]: R beat with no "
                                "pending request\n", align);
                    mismatches++;
                } else {
                    uint32_t beat_addr = pending_addr.front() +
                                          (uint32_t)(4 - beats_left.front()) * 16u;
                    uint32_t want = golden_byte(beat_addr) | ((uint32_t)golden_byte(beat_addr + 1) << 8) |
                                    ((uint32_t)golden_byte(beat_addr + 2) << 16) |
                                    ((uint32_t)golden_byte(beat_addr + 3) << 24);
                    if (r_word0 != want || r_resp != 0) {
                        mismatches++;
                        std::printf("  back-to-back-4beat-reads[align=%d]: beat_addr=0x%08x "
                                    "got=%08x want=%08x resp=%d\n",
                                    align, beat_addr, r_word0, want, (int)r_resp);
                    }
                    checked++;
                    beats_left.front()--;
                    if (beats_left.front() == 0) {
                        pending_addr.erase(pending_addr.begin());
                        beats_left.erase(beats_left.begin());
                    }
                }
            }
        }
        bool ok = (mismatches == 0) && (checked == LINES * 4) && (guard < GUARD_MAX);
        if (!ok && guard >= GUARD_MAX) {
            std::printf("  back-to-back-4beat-reads[align=%d]: GUARD EXPIRED\n", align);
        }
        char name[80];
        std::snprintf(name, sizeof(name), "back_to_back_4beat_reads_alignment_sweep_align%d", align);
        CHECK(name, ok);
    }
}

// app_rdy/app_wdf_rdy-equivalent (m_awready/m_wready/m_arready + m_rvalid
// pacing) random backpressure sweep: repeats a modest mixed read/write
// sequence at several stall probabilities, scoreboarding every value.
static void scenario_backpressure_sweep() {
    for (int stall_pct : {0, 15, 40, 70}) {
        reset();
        dut->s_bready = 1;
        dut->s_rready = 1;
        MigModel mig(/*seed=*/(uint32_t)(1000 + stall_pct), stall_pct);
        std::unordered_map<uint32_t, Word128> golden;
        const uint32_t base = 0x00060000u;
        const int ops = 40;
        bool ok = true;

        for (int i = 0; i < ops && ok; i++) {
            bool do_write = (i % 2) == 0;
            uint32_t addr = base + (uint32_t)(i % 6) * 16u;
            if (do_write) {
                Word128 data{0xC0000000u + (uint32_t)i, 0xC1000000u + (uint32_t)i,
                            0xC2000000u + (uint32_t)i, 0xC3000000u + (uint32_t)i};
                dut->s_awid = (uint8_t)i;
                dut->s_awaddr = addr;
                dut->s_awlen = 0;
                dut->s_awsize = 4;
                dut->s_awburst = 1;
                dut->s_awvalid = 1;
                set_w128(dut->s_wdata, data);
                dut->s_wstrb = 0xFFFF;
                dut->s_wlast = 1;
                dut->s_wvalid = 1;
                bool committed = false;
                for (int spins = 0; spins < 64 && !committed; spins++) {
                    mig_settle(mig);
                    bool aw_fire_now = dut->s_awvalid && dut->s_awready;
                    bool w_fire_now  = dut->s_wvalid && dut->s_wready;
                    bool b_fire_now  = dut->s_bvalid && dut->s_bready;
                    mig_advance(mig);
                    if (aw_fire_now) dut->s_awvalid = 0;
                    if (w_fire_now) dut->s_wvalid = 0;
                    if (b_fire_now) {
                        golden[addr] = data;
                        committed = true;
                    }
                }
                ok = ok && committed;
            } else {
                dut->s_arid = (uint8_t)(0x80 + i);
                dut->s_araddr = addr;
                dut->s_arlen = 0;
                dut->s_arsize = 4;
                dut->s_arburst = 1;
                dut->s_arvalid = 1;
                Word128 got{};
                bool got_data = false;
                for (int spins = 0; spins < 64 && !got_data; spins++) {
                    mig_settle(mig);
                    bool ar_fire_now = dut->s_arvalid && dut->s_arready;
                    bool r_fire_now  = dut->s_rvalid && dut->s_rready;
                    Word128 beat = get_w128(dut->s_rdata);
                    mig_advance(mig);
                    if (ar_fire_now) dut->s_arvalid = 0;
                    if (r_fire_now) {
                        got = beat;
                        got_data = true;
                    }
                }
                ok = ok && got_data;
                auto it = golden.find(addr);
                Word128 expect = (it != golden.end()) ? it->second : Word128{0, 0, 0, 0};
                if (!(ok && got == expect)) {
                    std::printf("  backpressure-sweep stall=%d%% op %d: got=%s want=%s\n",
                                stall_pct, i, hex128(got).c_str(), hex128(expect).c_str());
                    ok = false;
                }
            }
        }
        dut->s_awvalid = 0;
        dut->s_wvalid = 0;
        dut->s_arvalid = 0;
        eval();
        char name[64];
        std::snprintf(name, sizeof name, "backpressure_sweep_stall%d", stall_pct);
        CHECK(name, ok);
    }
}

// ══════════════════════════════════════════════════════════════════════
// Coverage added with the 333 MHz AR/W-side registration fix
// ══════════════════════════════════════════════════════════════════════
//
// (1) AR issued back-to-back at FULL RATE, every AR carrying a DIFFERENT
//     translated address AND a different arlen.
//
// This is the scenario that has teeth for a registered AR output stage.
// The old combinational AR could not get the payload wrong -- it was a
// live mux off rq_issue_ptr.  A registered stage can, in three specific
// ways, and this checks all three:
//   * capture the WRONG descriptor's payload  -> addr/len mismatch;
//   * issue the SAME descriptor twice (the real bug you get if
//     rq_issue_ptr keeps advancing on m_arready instead of on capture,
//     because ar_slot_free is high on the handshake cycle while the
//     pointer still points at the AR already in the register)
//                                              -> duplicate/extra AR;
//   * drop one                                 -> short count.
// It also asserts the ARs come out on CONSECUTIVE cycles, i.e. the slice
// reloads on the handshake cycle.  A slice written the naive way
// (ar_want gated on !ar_presenting alone) still passes every functional
// check above but halves AR throughput to one every two cycles -- that
// regression is invisible without this timing assertion.
//
// Alternating araddr[4] is deliberate: it makes mig_arlen alternate 1/2
// as well, so a payload captured from the neighbouring descriptor is
// caught by BOTH fields rather than only by the address.
static void scenario_ar_back_to_back_full_rate() {
    reset();
    dut->s_rready = 1;
    dut->m_arready = 1;   // never backpressure: we want full rate
    eval();

    const int N = 8;  // == RMAX_OUTSTANDING: the ring holds all of them
    struct ExpAr { uint32_t addr; uint8_t len; };
    std::vector<ExpAr> expect;
    std::vector<ExpAr> got;
    std::vector<int> got_cycle;

    for (int i = 0; i < N; i++) {
        // 4-beat (len=3) size-4 INCR read; every other one starts in the
        // upper half of its 32B MIG beat, which changes the MIG beat
        // count from 2 to 3 and therefore mig_arlen from 1 to 2.
        const uint32_t repo_addr = 0x00010000u + (uint32_t)i * 0x100u +
                                   ((i & 1) ? 0x10u : 0x00u);
        expect.push_back(ExpAr{repo_addr & ~0x1Fu,
                               (uint8_t)((i & 1) ? 2 : 1)});
    }

    int issued = 0;
    for (int cyc = 0; cyc < 64 && (int)got.size() < N; cyc++) {
        if (issued < N) {
            dut->s_arid = (uint8_t)(0x40 + issued);
            dut->s_araddr = 0x00010000u + (uint32_t)issued * 0x100u +
                            ((issued & 1) ? 0x10u : 0x00u);
            dut->s_arlen = 3;
            dut->s_arsize = 4;
            dut->s_arburst = 1;
            dut->s_arvalid = 1;
        } else {
            dut->s_arvalid = 0;
        }
        eval();
        const bool ar_accepted = (issued < N) && dut->s_arready;
        if (dut->m_arvalid && dut->m_arready) {
            got.push_back(ExpAr{(uint32_t)dut->m_araddr,
                                (uint8_t)dut->m_arlen});
            got_cycle.push_back(cyc);
        }
        tick();
        if (ar_accepted) issued++;
        eval();
    }
    dut->s_arvalid = 0;
    eval();

    bool ok = (issued == N);
    if (!ok) {
        std::printf("  ar_back_to_back: only %d/%d repo ARs accepted\n", issued, N);
    }
    if (got.size() != (size_t)N) {
        std::printf("  ar_back_to_back: MIG saw %zu ARs, expected %d "
                    "(duplicate or dropped AR)\n", got.size(), N);
        ok = false;
    }
    for (size_t i = 0; i < got.size() && i < expect.size(); i++) {
        if (got[i].addr != expect[i].addr || got[i].len != expect[i].len) {
            std::printf("  ar_back_to_back: AR %zu got addr=0x%08x len=%u, "
                        "want addr=0x%08x len=%u\n",
                        i, got[i].addr, (unsigned)got[i].len,
                        expect[i].addr, (unsigned)expect[i].len);
            ok = false;
        }
    }
    // Full rate: consecutive cycles once the first AR is out.
    for (size_t i = 1; i < got_cycle.size(); i++) {
        if (got_cycle[i] != got_cycle[i - 1] + 1) {
            std::printf("  ar_back_to_back: AR %zu issued at cycle %d, "
                        "previous at %d -- not full rate (register slice "
                        "is not reloading on the handshake cycle)\n",
                        i, got_cycle[i], got_cycle[i - 1]);
            ok = false;
        }
    }
    // Nothing extra may trail out after the last expected AR.
    for (int t = 0; t < 8; t++) {
        eval();
        if (dut->m_arvalid) {
            std::printf("  ar_back_to_back: stray extra AR after the "
                        "expected %d (addr=0x%08x)\n", N,
                        (uint32_t)dut->m_araddr);
            ok = false;
            break;
        }
        tick();
    }
    dut->m_arready = 0;
    eval();
    CHECK("ar_back_to_back_full_rate", ok);
}

// (2) AXI4 A3.2.1/A3.2.2 protocol monitor on the registered AR channel:
//     while ARVALID is high and ARREADY is low, ARVALID must stay high
//     and ARADDR/ARLEN/ARSIZE/ARBURST must not change.
//
// The whole point of the AR register slice is that its load enable looks
// at m_arready.  Getting that wrong -- e.g. reloading the payload from
// the next descriptor while the current AR is still unaccepted -- would
// silently corrupt a command on a real MIG and is exactly the failure a
// functional read test cannot see (the tb's other read scenarios accept
// AR immediately). Positive control: this loop only records a violation
// if it observes a stalled ARVALID cycle at all, so `stalled_cycles` is
// asserted non-zero -- otherwise a bridge that never asserted ARVALID
// would "pass" vacuously.
static void scenario_ar_payload_stable_under_backpressure() {
    reset();
    dut->s_rready = 1;
    eval();

    const int N = 6;
    int issued = 0;
    int stalled_cycles = 0;
    int accepted = 0;
    bool ok = true;

    bool prev_valid = false;
    uint32_t prev_addr = 0;
    uint32_t prev_len = 0, prev_size = 0, prev_burst = 0;
    bool prev_ready = false;

    for (int cyc = 0; cyc < 200 && accepted < N; cyc++) {
        // ARREADY high only 1 cycle in 4 -> long stalls with ARVALID up.
        dut->m_arready = ((cyc % 4) == 3) ? 1 : 0;
        if (issued < N) {
            dut->s_arid = (uint8_t)(0x50 + issued);
            dut->s_araddr = 0x00020000u + (uint32_t)issued * 0x100u +
                            ((issued & 1) ? 0x10u : 0x00u);
            dut->s_arlen = (uint8_t)(issued & 3);
            dut->s_arsize = 4;
            dut->s_arburst = 1;
            dut->s_arvalid = 1;
        } else {
            dut->s_arvalid = 0;
        }
        eval();

        // Check the PREVIOUS cycle's stall obligation against this cycle.
        if (prev_valid && !prev_ready) {
            stalled_cycles++;
            if (!dut->m_arvalid) {
                std::printf("  ar_stability: ARVALID dropped without a "
                            "handshake at cycle %d\n", cyc);
                ok = false;
            }
            if ((uint32_t)dut->m_araddr != prev_addr ||
                (uint32_t)dut->m_arlen != prev_len ||
                (uint32_t)dut->m_arsize != prev_size ||
                (uint32_t)dut->m_arburst != prev_burst) {
                std::printf("  ar_stability: AR payload changed while "
                            "stalled at cycle %d: addr 0x%08x->0x%08x "
                            "len %u->%u\n", cyc, prev_addr,
                            (uint32_t)dut->m_araddr, prev_len,
                            (uint32_t)dut->m_arlen);
                ok = false;
            }
        }

        const bool ar_accepted_repo = (issued < N) && dut->s_arready;
        if (dut->m_arvalid && dut->m_arready) accepted++;
        prev_valid = dut->m_arvalid != 0;
        prev_ready = dut->m_arready != 0;
        prev_addr = dut->m_araddr;
        prev_len = dut->m_arlen;
        prev_size = dut->m_arsize;
        prev_burst = dut->m_arburst;

        tick();
        if (ar_accepted_repo) issued++;
        eval();
    }
    dut->s_arvalid = 0;
    dut->m_arready = 0;
    eval();

    if (accepted != N) {
        std::printf("  ar_stability: only %d/%d ARs accepted by the MIG\n",
                    accepted, N);
        ok = false;
    }
    // Positive control -- the stability rule must actually have been
    // exercised, not merely never contradicted.
    if (stalled_cycles == 0) {
        std::printf("  ar_stability: never observed a stalled ARVALID cycle "
                    "-- the check was vacuous\n");
        ok = false;
    }
    CHECK("ar_payload_stable_under_backpressure", ok);
}

// (3) wr_bypass: AW and the FIRST W beat of a MULTI-beat burst accepted on
//     the SAME cycle, on a WARM ring.
//
// This is the coincidence the module header calls out (wq_data_ptr ==
// wq_wptr, the descriptor's array entries not yet written) and the exact
// case the W-side timing fix had to leave alone: beat 0 must be served by
// the COMBINATIONAL bypass off s_aw*, because neither the array nor the
// new cwd_* shadow holds anything valid for it yet.  Beats 1..n-1 are
// then served by cwd_*, so this scenario is a direct test of the
// bypass -> shadow handoff introduced by that change.
//
// Warming the ring first is what gives it teeth: every physical slot is
// left holding a RETIRED single-beat descriptor (repo_beats=1, opposite
// araddr[4]).  If cwd_* ever latched the stale array entry instead of the
// bypassed values, cur_wdata_repobeat would be 1 and the burst would
// terminate after one beat -- a wrong MIG beat count and a wrong WLAST
// position, both checked below.  addr[4]=1 additionally makes
// cur_wdata_addr4 load-bearing: the first MIG beat is upper-half-only
// with a zeroed lower half, which a wrong shadowed addr bit inverts.
static bool bypass_burst_case(uint32_t addr, int repo_beats,
                              const std::vector<MigWriteBeat>& exp_mig,
                              uint8_t id, const char* tag) {
    std::vector<Word128> data;
    std::vector<uint16_t> strb;
    for (int i = 0; i < repo_beats; i++) {
        data.push_back(Word128{0x7B000000u + (uint32_t)i * 4 + 0,
                               0x7B000000u + (uint32_t)i * 4 + 1,
                               0x7B000000u + (uint32_t)i * 4 + 2,
                               0x7B000000u + (uint32_t)i * 4 + 3});
        strb.push_back(0xFFFF);
    }

    dut->m_awready = 1;
    dut->m_wready = 1;
    // Park the early B (see the warm-up note) so the streaming loop below
    // does not consume it before it is checked.
    dut->s_bready = 0;

    // THE COINCIDENCE: AW and W beat 0 presented on the same cycle.
    dut->s_awid = id;
    dut->s_awaddr = addr;
    dut->s_awlen = (uint8_t)(repo_beats - 1);
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    set_w128(dut->s_wdata, data[0]);
    dut->s_wstrb = strb[0];
    dut->s_wlast = (repo_beats == 1) ? 1 : 0;
    dut->s_wvalid = 1;
    eval();
    bool ok = dut->s_awready && dut->s_wready;
    if (!ok) {
        std::printf("  %s: same-cycle AW+W not accepted (awready=%u wready=%u)\n",
                    tag, (unsigned)dut->s_awready, (unsigned)dut->s_wready);
    }

    std::vector<MigWriteBeat> captured;
    bool aw_seen = false;
    uint32_t cap_awaddr = 0;
    uint32_t cap_awlen = 0;
    int beat = 0;

    for (int cyc = 0; cyc < 64; cyc++) {
        eval();
        const bool w_fire = dut->s_wvalid && dut->s_wready;
        if (dut->m_awvalid && dut->m_awready && !aw_seen) {
            aw_seen = true;
            cap_awaddr = dut->m_awaddr;
            cap_awlen = dut->m_awlen;
        }
        if (dut->m_wvalid && dut->m_wready) {
            captured.push_back(MigWriteBeat{get_w256(dut->m_wdata),
                                            (uint32_t)dut->m_wstrb,
                                            dut->m_wlast != 0});
        }
        tick();
        dut->s_awvalid = 0;
        if (w_fire) {
            beat++;
            if (beat < repo_beats) {
                set_w128(dut->s_wdata, data[beat]);
                dut->s_wstrb = strb[beat];
                dut->s_wlast = (beat == repo_beats - 1) ? 1 : 0;
                dut->s_wvalid = 1;
            } else {
                dut->s_wvalid = 0;
                dut->s_wlast = 0;
            }
        }
        eval();
        if (beat >= repo_beats && captured.size() >= exp_mig.size() && aw_seen)
            break;
    }

    if (!aw_seen || cap_awaddr != (addr & ~0x1Fu) ||
        cap_awlen != (uint32_t)(exp_mig.size() - 1)) {
        std::printf("  %s: AW seen=%d addr=0x%08x (want 0x%08x) len=%u (want %zu)\n",
                    tag, (int)aw_seen, cap_awaddr, addr & ~0x1Fu,
                    cap_awlen, exp_mig.size() - 1);
        ok = false;
    }
    if (captured.size() != exp_mig.size()) {
        std::printf("  %s: %zu MIG beats, want %zu (a stale shadow would "
                    "truncate the burst here)\n",
                    tag, captured.size(), exp_mig.size());
        ok = false;
    }
    for (size_t i = 0; i < captured.size() && i < exp_mig.size(); i++) {
        if (!(captured[i].data == exp_mig[i].data &&
              captured[i].strb == exp_mig[i].strb &&
              captured[i].last == exp_mig[i].last)) {
            std::printf("  %s: MIG beat %zu mismatch: strb=0x%08x last=%d\n"
                        "     got  %s\n     want %s\n",
                        tag, i, captured[i].strb, (int)captured[i].last,
                        hex256(captured[i].data).c_str(),
                        hex256(exp_mig[i].data).c_str());
            ok = false;
        }
    }

    dut->m_awready = 0;
    dut->m_wready = 0;
    dut->s_bready = 1;
    eval();
    ok = ok && finish_write_response(id);
    return ok;
}

static void scenario_wr_bypass_same_cycle_aw_w_multibeat() {
    reset();
    dut->s_bready = 1;
    dut->m_awready = 1;
    dut->m_wready = 1;

    bool ok = true;

    // -- Warm every physical ring slot with a RETIRED single-beat write.
    //    Alternate araddr[4] so the stale entries differ from what the
    //    burst under test needs in both repo_beats AND addr[4].
    for (int i = 0; i < 4; i++) {
        Word128 d{0x5AA50000u + (uint32_t)i, 0x5AA50001u + (uint32_t)i,
                  0x5AA50002u + (uint32_t)i, 0x5AA50003u + (uint32_t)i};
        const uint32_t a = 0x00030000u + (uint32_t)i * 64u +
                           ((i & 1) ? 0x10u : 0x00u);
        // s_bready is held LOW across the drain so the bridge's early B
        // is parked rather than silently consumed by the drain ticks --
        // otherwise finish_write_response() below would find s_bvalid
        // already gone and the warm-up would "fail" for the wrong reason.
        dut->s_bready = 0;
        bool w_ok = accept_write(a, (uint8_t)(0x60 + i), d, 0xFFFF, true, 4);
        // Drain the single MIG beat so the slot genuinely retires.
        for (int t = 0; t < 8; t++) { eval(); tick(); }
        dut->s_bready = 1;
        eval();
        w_ok = w_ok && finish_write_response((uint8_t)(0x60 + i));
        if (!w_ok) {
            std::printf("  wr_bypass_multibeat: ring warm-up write %d failed\n", i);
            ok = false;
        }
    }

    const Word128 d0{0x7B000000u, 0x7B000001u, 0x7B000002u, 0x7B000003u};
    const Word128 d1{0x7B000004u, 0x7B000005u, 0x7B000006u, 0x7B000007u};
    const Word128 d2{0x7B000008u, 0x7B000009u, 0x7B00000Au, 0x7B00000Bu};
    const Word128 d3{0x7B00000Cu, 0x7B00000Du, 0x7B00000Eu, 0x7B00000Fu};
    const Word128 z{0, 0, 0, 0};

    // Case A: addr[4]=1 -> upper-half start.  3 MIG beats:
    //   beat0 = {d0, 0}   (lone upper, lower zero-filled)
    //   beat1 = {d2, d1}
    //   beat2 = {0, d3}   (lone trailing lower, upper zero-filled) + WLAST
    std::vector<MigWriteBeat> expA{
        MigWriteBeat{Word256{z[0], z[1], z[2], z[3], d0[0], d0[1], d0[2], d0[3]},
                     0xFFFF0000u, false},
        MigWriteBeat{Word256{d1[0], d1[1], d1[2], d1[3], d2[0], d2[1], d2[2], d2[3]},
                     0xFFFFFFFFu, false},
        MigWriteBeat{Word256{d3[0], d3[1], d3[2], d3[3], z[0], z[1], z[2], z[3]},
                     0x0000FFFFu, true},
    };
    ok = bypass_burst_case(0x00040010u, 4, expA, 0x71,
                           "wr_bypass_multibeat[addr4=1]") && ok;

    // Case B: addr[4]=0 -> paired from the first beat.  2 MIG beats.
    std::vector<MigWriteBeat> expB{
        MigWriteBeat{Word256{d0[0], d0[1], d0[2], d0[3], d1[0], d1[1], d1[2], d1[3]},
                     0xFFFFFFFFu, false},
        MigWriteBeat{Word256{d2[0], d2[1], d2[2], d2[3], d3[0], d3[1], d3[2], d3[3]},
                     0xFFFFFFFFu, true},
    };
    ok = bypass_burst_case(0x00050000u, 4, expB, 0x72,
                           "wr_bypass_multibeat[addr4=0]") && ok;

    CHECK("wr_bypass_same_cycle_aw_w_multibeat", ok);
}

// (4) The W-side context must follow wq_data_ptr, NOT wq_wptr.
//
// The wr_bypass select is only correct when the AW being accepted this
// cycle targets the very slot the W stream is consuming.  This scenario
// builds the case where an AW is accepted on the same cycle as a W beat
// that belongs to a DIFFERENT, EARLIER descriptor:
//
//   cycle 0: AW for burst A (2 repo beats) accepted; no W offered.
//   cycle 1: AW for burst B (1 repo beat) accepted AND A's first W beat
//            accepted -- wq_data_ptr is still A's slot while wq_wptr has
//            already moved to B's, so wr_bypass MUST be 0 and the packing
//            context must describe A (2 beats), not B (1 beat).
//
// Getting this wrong is silent and destructive rather than obviously
// broken: the bridge would compare A's first beat against B's length,
// decide WLAST is misplaced, downgrade A to SLVERR and start pushing
// padding beats at the MIG.  So the observable is A's B-channel response
// code, checked below.
//
// This scenario exists specifically because wr_bypass now consults a
// REGISTERED copy of (wq_data_ptr == wq_wptr).  A stale copy differs from
// the live comparison only in the cycle immediately after a pointer moves
// -- which is exactly cycle 1 here, and which nothing else in this tb was
// reaching (verified: registering the equality from the CURRENT instead
// of the NEXT pointer values leaves every other scenario green).
static void scenario_w_context_follows_data_ptr_not_wptr() {
    reset();
    dut->s_bready = 1;
    dut->m_awready = 1;
    dut->m_wready = 1;
    eval();

    struct AwOffer { uint32_t addr; uint8_t id; uint8_t len; };
    const AwOffer aws[2] = {
        {0x00060000u, 0x7A, 1},   // A: 2 repo beats, addr[4]=0 -> 1 MIG beat
        {0x00061000u, 0x7B, 0},   // B: 1 repo beat,  addr[4]=0 -> 1 MIG beat
    };
    struct WOffer { Word128 data; bool last; };
    const WOffer ws[3] = {
        {Word128{0xAC000000u, 0xAC000001u, 0xAC000002u, 0xAC000003u}, false},
        {Word128{0xAC000004u, 0xAC000005u, 0xAC000006u, 0xAC000007u}, true},
        {Word128{0xBC000000u, 0xBC000001u, 0xBC000002u, 0xBC000003u}, true},
    };

    int awi = 0, wi = 0, mig_w = 0;
    bool coincidence_seen = false;
    std::vector<std::pair<uint8_t, uint8_t>> bs;  // (id, resp)

    for (int cyc = 0; cyc < 64 && bs.size() < 2; cyc++) {
        if (awi < 2) {
            dut->s_awid = aws[awi].id;
            dut->s_awaddr = aws[awi].addr;
            dut->s_awlen = aws[awi].len;
            dut->s_awsize = 4;
            dut->s_awburst = 1;
            dut->s_awvalid = 1;
        } else {
            dut->s_awvalid = 0;
        }
        // W is deliberately withheld on cycle 0 so that A's AW retires
        // into the ring alone and wq_wptr moves ahead of wq_data_ptr.
        if (cyc >= 1 && wi < 3) {
            set_w128(dut->s_wdata, ws[wi].data);
            dut->s_wstrb = 0xFFFF;
            dut->s_wlast = ws[wi].last ? 1 : 0;
            dut->s_wvalid = 1;
        } else {
            dut->s_wvalid = 0;
            dut->s_wlast = 0;
        }
        eval();
        const bool awf = dut->s_awvalid && dut->s_awready;
        const bool wf = dut->s_wvalid && dut->s_wready;
        // The case under test: AW #1 (B) accepted on the same cycle as
        // W beat #0, which belongs to A.
        if (awf && wf && awi == 1 && wi == 0) coincidence_seen = true;
        if (dut->m_wvalid && dut->m_wready) mig_w++;
        if (dut->s_bvalid && dut->s_bready) {
            bs.push_back({(uint8_t)dut->s_bid, (uint8_t)dut->s_bresp});
        }
        tick();
        if (awf) awi++;
        if (wf) wi++;
        eval();
    }
    dut->s_awvalid = 0;
    dut->s_wvalid = 0;
    dut->s_wlast = 0;
    dut->m_awready = 0;
    dut->m_wready = 0;
    eval();

    bool ok = true;
    // Positive control: if the AW+W coincidence never happened, the
    // scenario proved nothing and must not report success.
    if (!coincidence_seen) {
        std::printf("  w_context_follows_data_ptr: the AW(B)+W(A beat0) "
                    "coincidence never occurred -- check was vacuous\n");
        ok = false;
    }
    if (awi != 2 || wi != 3) {
        std::printf("  w_context_follows_data_ptr: accepted %d/2 AW, %d/3 W\n",
                    awi, wi);
        ok = false;
    }
    if (bs.size() != 2 || bs[0].first != 0x7A || bs[0].second != 0 ||
        bs[1].first != 0x7B || bs[1].second != 0) {
        std::printf("  w_context_follows_data_ptr: B responses %zu:", bs.size());
        for (auto& b : bs) std::printf(" (id=0x%02x resp=%u)", b.first, b.second);
        std::printf("  -- want (0x7A,0) (0x7B,0); SLVERR on 0x7A means the "
                    "packing context came from B's descriptor\n");
        ok = false;
    }
    // A = 2 repo beats packed into 1 MIG beat; B = 1 repo beat -> 1 MIG
    // beat.  Any padding beats from a spurious downgrade show up here.
    if (mig_w != 2) {
        std::printf("  w_context_follows_data_ptr: %d MIG W beats, want 2 "
                    "(extra beats = padding from a spurious downgrade)\n",
                    mig_w);
        ok = false;
    }
    CHECK("w_context_follows_data_ptr_not_wptr", ok);
}

// Randomized scoreboard: >=10k mixed single-beat read/write ops against a
// byte-addressed golden model, under random MIG-side backpressure.
// Genuinely concurrent (review I4): up to MAX_W_INFLIGHT writes and
// MAX_R_INFLIGHT reads may be outstanding (accepted, awaiting B/R) at
// once, driven by independently-advancing AW+W / AR offer state machines
// on a single shared per-cycle loop -- NOT one op spun to completion
// before the next is even issued, which exercised none of the bridge's
// actual multi-outstanding pipelining.  B/R completions are matched
// strictly FIFO against their respective in-flight queues, matching the
// bridge's own same-ID-ordered-per-direction guarantee.  Avoids issuing
// a write to an address with a currently-outstanding read (the T11
// hazard check only protects the RAW direction -- see the bridge's
// header note / contract doc -- so a WAR race there is a known,
// documented non-goal, not something this scoreboard should flag).
static void scenario_random_scoreboard_10k() {
    reset();
    dut->s_bready = 1;
    dut->s_rready = 1;
    MigModel mig(/*seed=*/0xC0FFEEu, /*stall_pct=*/20);
    std::mt19937 rng(0xA5A5A5A5u);
    std::unordered_map<uint32_t, Word128> golden_full;
    std::unordered_set<uint32_t> outstanding_reads;

    const uint32_t base = 0x00070000u;
    const int POOL = 96;
    const int TOTAL_OPS = 10000;
    const int MAX_W_INFLIGHT = 4;  // matches WMAX_OUTSTANDING
    const int MAX_R_INFLIGHT = 6;  // < RMAX_OUTSTANDING, leaves headroom
    bool ok = true;
    int writes_done = 0, reads_done = 0, total_issued = 0;
    uint32_t id_ctr = 0;

    struct WOp { uint32_t addr; uint8_t id; Word128 data; };
    struct ROp { uint32_t addr; uint8_t id; };
    std::deque<WOp> w_inflight;  // AW+W accepted, awaiting B (FIFO)
    std::deque<ROp> r_inflight;  // AR accepted, awaiting R (FIFO)

    bool w_have_offer = false, aw_offer_done = false, w_offer_done = false;
    WOp  w_offer{};
    bool r_have_offer = false;
    ROp  r_offer{};

    dut->s_awvalid = 0;
    dut->s_wvalid = 0;
    dut->s_arvalid = 0;

    int guard = 0;
    while ((writes_done + reads_done) < TOTAL_OPS && ok && guard < TOTAL_OPS * 40) {
        guard++;

        // -- start a new write offer if there's room and the channel is
        //    idle; skip (retry next cycle) if every sampled address
        //    currently has an outstanding read (WAR avoidance). --
        if (!w_have_offer && (int)w_inflight.size() < MAX_W_INFLIGHT &&
            total_issued < TOTAL_OPS) {
            for (int tries = 0; tries < 8; tries++) {
                uint32_t addr = base + (rng() % POOL) * 16u;
                if (outstanding_reads.count(addr)) continue;
                w_offer.addr = addr;
                w_offer.id   = (uint8_t)(id_ctr++ & 0x3F);
                w_offer.data = Word128{(uint32_t)rng(), (uint32_t)rng(),
                                       (uint32_t)rng(), (uint32_t)rng()};
                w_have_offer  = true;
                aw_offer_done = false;
                w_offer_done  = false;
                total_issued++;
                dut->s_awid = w_offer.id;
                dut->s_awaddr = w_offer.addr;
                dut->s_awlen = 0;
                dut->s_awsize = 4;
                dut->s_awburst = 1;
                dut->s_awvalid = 1;
                set_w128(dut->s_wdata, w_offer.data);
                dut->s_wstrb = 0xFFFF;
                dut->s_wlast = 1;
                dut->s_wvalid = 1;
                break;
            }
        }

        // -- start a new read offer if there's room --
        if (!r_have_offer && (int)r_inflight.size() < MAX_R_INFLIGHT &&
            total_issued < TOTAL_OPS) {
            uint32_t addr = base + (rng() % POOL) * 16u;
            r_offer.addr = addr;
            r_offer.id   = (uint8_t)(id_ctr++ & 0x3F);
            outstanding_reads.insert(addr);
            r_have_offer = true;
            total_issued++;
            dut->s_arid = r_offer.id;
            dut->s_araddr = addr;
            dut->s_arlen = 0;
            dut->s_arsize = 4;
            dut->s_arburst = 1;
            dut->s_arvalid = 1;
        }

        mig_settle(mig);
        bool aw_fire_now = dut->s_awvalid && dut->s_awready;
        bool w_fire_now  = dut->s_wvalid && dut->s_wready;
        bool ar_fire_now = dut->s_arvalid && dut->s_arready;
        bool b_fire_now  = dut->s_bvalid && dut->s_bready;
        bool r_fire_now  = dut->s_rvalid && dut->s_rready;
        uint8_t b_id_now = dut->s_bid, b_resp_now = dut->s_bresp;
        uint8_t r_id_now = dut->s_rid, r_resp_now = dut->s_rresp;
        Word128 r_data_now = get_w128(dut->s_rdata);
        mig_advance(mig);

        if (aw_fire_now) { aw_offer_done = true; dut->s_awvalid = 0; }
        if (w_fire_now)  { w_offer_done  = true; dut->s_wvalid  = 0; }
        if (w_have_offer && aw_offer_done && w_offer_done) {
            w_inflight.push_back(w_offer);
            w_have_offer = false;
        }
        if (ar_fire_now) {
            r_inflight.push_back(r_offer);
            r_have_offer = false;
            dut->s_arvalid = 0;
        }
        if (b_fire_now) {
            if (w_inflight.empty()) {
                ok = false;
            } else {
                WOp done = w_inflight.front();
                w_inflight.pop_front();
                if (b_id_now != done.id || b_resp_now != 0) ok = false;
                golden_full[done.addr] = done.data;
                writes_done++;
            }
        }
        if (r_fire_now) {
            if (r_inflight.empty()) {
                ok = false;
            } else {
                ROp done = r_inflight.front();
                r_inflight.pop_front();
                if (r_id_now != done.id || r_resp_now != 0) ok = false;
                outstanding_reads.erase(done.addr);
                auto it = golden_full.find(done.addr);
                Word128 expect = (it != golden_full.end()) ? it->second : Word128{0, 0, 0, 0};
                if (!(r_data_now == expect)) {
                    std::printf("  random-scoreboard read #%d addr=0x%08x: got=%s want=%s\n",
                                reads_done, done.addr, hex128(r_data_now).c_str(),
                                hex128(expect).c_str());
                    ok = false;
                }
                reads_done++;
            }
        }
    }
    ok = ok && (guard < TOTAL_OPS * 40);
    dut->s_awvalid = 0;
    dut->s_wvalid = 0;
    dut->s_arvalid = 0;
    eval();
    std::printf("  random-scoreboard: %d writes, %d reads, %d total ops "
                "(concurrent, up to %d W / %d R in flight)\n",
                writes_done, reads_done, writes_done + reads_done,
                MAX_W_INFLIGHT, MAX_R_INFLIGHT);
    CHECK("random_scoreboard_10k_mixed_ops", ok);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaxi_ddr4_mig_bridge;

    scenario_lower_half_write_maps_to_256_beat();
    scenario_upper_half_write_maps_to_256_beat();
    scenario_lower_half_write_burst_packs_pairs_with_backpressure();
    scenario_upper_half_write_burst_edge();
    scenario_long_write_burst_reaches_high_store_index();
    scenario_word_write_maps_to_256_lane();
    scenario_word_write_with_byte_offset_is_forwarded();
    scenario_narrow_byte_write_is_forwarded();
    scenario_narrow_byte_write_upper_half_is_forwarded();
    scenario_narrow_halfword_write_is_forwarded();
    scenario_read_slices_upper_half_and_preserves_id();
    scenario_lower_half_read_burst_unpacks_pairs_with_backpressure();
    scenario_upper_half_read_burst_edge();
    scenario_word_read_returns_repo_half_for_narrow_slice();
    scenario_word_read_with_byte_offset_is_forwarded();
    scenario_high_address_is_local_slverr();
    scenario_narrow_multibeat_burst_is_local_slverr();
    scenario_rejected_multibeat_read_returns_full_burst();
    scenario_misaligned_address_is_local_slverr();
    scenario_fixed_burst_is_local_slverr();
    scenario_wrong_size_is_local_slverr();
    scenario_missing_wlast_is_local_slverr();

    scenario_eight_pipelined_reads_speedup();
    scenario_interleaved_read_write_streams();
    scenario_write_read_hazard_returns_new_data();
    scenario_rlast_beat_count_integrity();
    scenario_pipelined_writes_multi_outstanding();
    scenario_pipelined_write_data_streams_before_aw_issued();
    scenario_malformed_burst_completes_with_padding();
    scenario_malformed_burst_zero_prefix_padding();
    scenario_rejected_aw_early_wlast_no_orphan();
    scenario_warm_ring_stale_bypass_no_orphan();
    scenario_back_to_back_single_beat_half_packing();
    scenario_back_to_back_4beat_reads_alignment_sweep();
    scenario_backpressure_sweep();
    scenario_ar_back_to_back_full_rate();
    scenario_ar_payload_stable_under_backpressure();
    scenario_wr_bypass_same_cycle_aw_w_multibeat();
    scenario_w_context_follows_data_ptr_not_wptr();
    scenario_random_scoreboard_10k();

    if (n_fail == 0) {
        std::printf("All %d scenarios PASSED.\n", n_pass);
    } else {
        std::printf("%d scenarios FAILED (%d passed).\n", n_fail, n_pass);
    }

    delete dut;
    return n_fail == 0 ? 0 : 1;
}
