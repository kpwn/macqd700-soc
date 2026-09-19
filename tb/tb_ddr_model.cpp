// tb_ddr_model.cpp — Verilator unit testbench for ddr_ctrl.v in SIM_MODEL.
//
// Exercises the behavioural DDR slave against the following scenarios:
//   1. cal_done rises N cycles after reset release.
//   2. Single-beat write followed by single-beat read returns what was
//      written (full-strobe write).
//   3. Byte-strobe write merges into existing beat.
//   4. 32-bit narrow writes/reads in each lane of a 128-bit DDR beat.
//   5. Multi-beat INCR burst write followed by matching burst read — all
//      beats match in order.
//   6. Narrow INCR bursts preserve byte lanes inside one DDR beat.
//   7. ID round-trip: AWID echoed on BID, ARID echoed on RID.
//   8. Write-then-read-same-addr coherence across burst boundary.
//   9. Cannot start a transaction before cal_done.
//  10. Reset drops cal_done and re-arms traffic only after retraining.
//  11. Unsupported width/alignment requests return SLVERR in sim.
//  12. Unsupported AXI burst types return SLVERR and do not mutate data.
//
// Build: make tb-ddr-model
// Pass:  "All N scenarios PASSED."

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <array>
#include <vector>
#include <verilated.h>
#include "Vtb_ddr_model.h"

static Vtb_ddr_model* dut = nullptr;
static uint64_t sim_time  = 0;
static int n_pass = 0, n_fail = 0;

static constexpr int DATA_WIDTH = 128;
static constexpr int STRB_WIDTH = DATA_WIDTH / 8;
static constexpr int XID_WIDTH  = 6;

// 128-bit word helper (Verilator VlWide<4>).
using Word128 = std::array<uint32_t, 4>;

static Word128 w128_from_u64(uint64_t lo, uint64_t hi = 0) {
    return {(uint32_t)(lo & 0xFFFFFFFFu),
            (uint32_t)(lo >> 32),
            (uint32_t)(hi & 0xFFFFFFFFu),
            (uint32_t)(hi >> 32)};
}
static bool w128_eq(const Word128& a, const Word128& b) { return a == b; }
static std::string w128_hex(const Word128& a) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%08x_%08x_%08x_%08x",
                  a[3], a[2], a[1], a[0]);
    return buf;
}

static void set_byte(Word128& w, int idx, uint8_t val) {
    int word = idx / 4;
    int lane = idx % 4;
    w[word] &= ~(0xFFu << (lane * 8));
    w[word] |= (uint32_t)val << (lane * 8);
}

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

// ── Clock helpers ─────────────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void idle_slave_inputs() {
    dut->awid = 0; dut->awaddr = 0; dut->awlen = 0;
    dut->awsize = 0; dut->awburst = 0; dut->awvalid = 0;
    set_w128(dut->wdata, Word128{0,0,0,0});
    dut->wstrb = 0; dut->wlast = 0; dut->wvalid = 0;
    dut->bready = 1;
    dut->arid = 0; dut->araddr = 0; dut->arlen = 0;
    dut->arsize = 0; dut->arburst = 0; dut->arvalid = 0;
    dut->rready = 1;
}

static void reset() {
    idle_slave_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
}

// ── Wait helpers ──────────────────────────────────────────────────────────
static void wait_cal_done(int max_cycles = 64) {
    for (int i = 0; i < max_cycles; i++) {
        if (dut->ddr_cal_done) return;
        tick();
    }
}

static bool do_read_raw(uint32_t addr, Word128* out_data, uint8_t* out_id,
                        uint8_t arid_val, uint8_t arsize_val,
                        uint8_t arburst_val, uint32_t expect_rresp,
                        int max_cycles);

static bool do_write_raw(uint32_t addr, const Word128& data, uint16_t strb,
                         uint8_t awid_val, uint8_t awsize_val,
                         uint8_t awburst_val, uint32_t expect_bresp,
                         int max_cycles = 200) {
    dut->awid    = awid_val;
    dut->awaddr  = addr;
    dut->awlen   = 0;
    dut->awsize  = awsize_val;
    dut->awburst = awburst_val;
    dut->awvalid = 1;
    int cyc = 0;
    while (!dut->awready && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->awready) return false;
    tick();
    dut->awvalid = 0;

    set_w128(dut->wdata, data);
    dut->wstrb  = strb;
    dut->wlast  = 1;
    dut->wvalid = 1;
    cyc = 0;
    while (!dut->wready && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->wready) return false;
    tick();
    dut->wvalid = 0;
    dut->wlast  = 0;
    dut->wstrb  = 0;

    cyc = 0;
    while (!dut->bvalid && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->bvalid) return false;
    bool ok = (dut->bresp == expect_bresp) &&
              ((dut->bid & ((1u << XID_WIDTH) - 1)) == awid_val);
    tick();
    return ok;
}

// Perform a single-beat write of 128 bits with arbitrary strobe.
// Returns true iff BRESP=OKAY and BID echoes awid_val.
static bool do_write(uint32_t addr, const Word128& data, uint16_t strb,
                     uint8_t awid_val, int max_cycles = 200) {
    return do_write_raw(addr, data, strb, awid_val, 4, 1, 0, max_cycles);
}

// Read a single beat — returns result via *out_data / *out_id, true on success.
static bool do_read(uint32_t addr, Word128* out_data, uint8_t* out_id,
                    uint8_t arid_val, int max_cycles = 200) {
    return do_read_raw(addr, out_data, out_id, arid_val, 4, 1, 0, max_cycles);
}

static bool do_read_raw(uint32_t addr, Word128* out_data, uint8_t* out_id,
                        uint8_t arid_val, uint8_t arsize_val,
                        uint8_t arburst_val, uint32_t expect_rresp,
                        int max_cycles = 200) {
    dut->arid    = arid_val;
    dut->araddr  = addr;
    dut->arlen   = 0;
    dut->arsize  = arsize_val;
    dut->arburst = arburst_val;
    dut->arvalid = 1;
    int cyc = 0;
    while (!dut->arready && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->arready) return false;
    tick();
    dut->arvalid = 0;

    cyc = 0;
    while (!dut->rvalid && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->rvalid) return false;
    *out_data = get_w128(dut->rdata);
    *out_id   = (uint8_t)(dut->rid & ((1u << XID_WIDTH) - 1));
    bool ok   = (dut->rresp == expect_rresp) && dut->rlast;
    tick();
    return ok;
}

// Burst write of `beats` beats, full-strobe.
static bool do_burst_write(uint32_t addr, const std::vector<Word128>& beats,
                           uint8_t awid_val, int max_cycles = 400) {
    dut->awid    = awid_val;
    dut->awaddr  = addr;
    dut->awlen   = (uint8_t)(beats.size() - 1);
    dut->awsize  = 4;
    dut->awburst = 1;
    dut->awvalid = 1;
    int cyc = 0;
    while (!dut->awready && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->awready) return false;
    tick();
    dut->awvalid = 0;

    dut->wstrb = 0xFFFF;
    for (size_t i = 0; i < beats.size(); i++) {
        set_w128(dut->wdata, beats[i]);
        dut->wlast  = (i == beats.size() - 1) ? 1 : 0;
        dut->wvalid = 1;
        cyc = 0;
        while (!dut->wready && cyc < max_cycles) { tick(); cyc++; }
        if (!dut->wready) return false;
        tick();
    }
    dut->wvalid = 0;
    dut->wlast  = 0;
    dut->wstrb  = 0;

    cyc = 0;
    while (!dut->bvalid && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->bvalid) return false;
    bool ok = (dut->bresp == 0) && ((dut->bid & ((1u << XID_WIDTH) - 1)) == awid_val);
    tick();
    return ok;
}

static bool do_burst_write_raw(uint32_t addr,
                               const std::vector<Word128>& beats,
                               const std::vector<uint16_t>& strbs,
                               uint8_t awid_val,
                               uint8_t awsize_val,
                               uint8_t awburst_val,
                               uint32_t expect_bresp,
                               int max_cycles = 400) {
    if (beats.empty() || beats.size() != strbs.size()) return false;

    dut->awid    = awid_val;
    dut->awaddr  = addr;
    dut->awlen   = (uint8_t)(beats.size() - 1);
    dut->awsize  = awsize_val;
    dut->awburst = awburst_val;
    dut->awvalid = 1;
    int cyc = 0;
    while (!dut->awready && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->awready) return false;
    tick();
    dut->awvalid = 0;

    for (size_t i = 0; i < beats.size(); i++) {
        set_w128(dut->wdata, beats[i]);
        dut->wstrb  = strbs[i];
        dut->wlast  = (i == beats.size() - 1) ? 1 : 0;
        dut->wvalid = 1;
        cyc = 0;
        while (!dut->wready && cyc < max_cycles) { tick(); cyc++; }
        if (!dut->wready) return false;
        tick();
    }
    dut->wvalid = 0;
    dut->wlast  = 0;
    dut->wstrb  = 0;

    cyc = 0;
    while (!dut->bvalid && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->bvalid) return false;
    bool ok = (dut->bresp == expect_bresp) &&
              ((dut->bid & ((1u << XID_WIDTH) - 1)) == awid_val);
    tick();
    return ok;
}

static bool do_burst_read(uint32_t addr, size_t len,
                          std::vector<Word128>* out,
                          uint8_t arid_val, int max_cycles = 400) {
    dut->arid    = arid_val;
    dut->araddr  = addr;
    dut->arlen   = (uint8_t)(len - 1);
    dut->arsize  = 4;
    dut->arburst = 1;
    dut->arvalid = 1;
    int cyc = 0;
    while (!dut->arready && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->arready) return false;
    tick();
    dut->arvalid = 0;

    out->clear();
    for (size_t i = 0; i < len; i++) {
        cyc = 0;
        while (!dut->rvalid && cyc < max_cycles) { tick(); cyc++; }
        if (!dut->rvalid) return false;
        out->push_back(get_w128(dut->rdata));
        bool expect_last = (i == len - 1);
        if (expect_last && !dut->rlast) return false;
        if (!expect_last && dut->rlast)  return false;
        uint8_t rid_val = (uint8_t)(dut->rid & ((1u << XID_WIDTH) - 1));
        if (rid_val != arid_val) return false;
        if (dut->rresp != 0) return false;
        tick();
    }
    return true;
}

// ── Scenario runners ──────────────────────────────────────────────────────

#define CHECK(name, cond) do { \
    if (cond) { printf("[PASS] %s\n", name); n_pass++; } \
    else      { printf("[FAIL] %s\n", name); n_fail++; } \
} while (0)

static void scenario_cal_done() {
    reset();
    // Before cal_done, awready must be 0.
    dut->eval();
    bool before_ok = (dut->ddr_cal_done == 0) &&
                     (dut->awready == 0) &&
                     (dut->arready == 0);
    wait_cal_done();
    bool after_ok = (dut->ddr_cal_done == 1) &&
                    (dut->awready == 1) &&
                    (dut->arready == 1);
    CHECK("cal_done_rises_after_reset", before_ok && after_ok);
}

static void scenario_single_beat_wr_rd() {
    reset();
    wait_cal_done();
    Word128 pattern = w128_from_u64(0xDEADBEEFCAFEBABEULL, 0x0123456789ABCDEFULL);
    bool wok = do_write(0x100, pattern, 0xFFFF, /*awid=*/5);
    Word128 got{};
    uint8_t rid_val = 0;
    bool rok = do_read(0x100, &got, &rid_val, /*arid=*/7);
    bool data_ok = w128_eq(got, pattern);
    bool id_ok   = (rid_val == 7);
    if (!data_ok) {
        printf("   expected %s\n", w128_hex(pattern).c_str());
        printf("   got      %s\n", w128_hex(got).c_str());
    }
    CHECK("single_beat_wr_rd_roundtrip", wok && rok && data_ok && id_ok);
}

static void scenario_byte_strobe() {
    reset();
    wait_cal_done();
    // Prime the beat with a known pattern.
    Word128 base = w128_from_u64(0x1111111111111111ULL, 0x2222222222222222ULL);
    (void)do_write(0x200, base, 0xFFFF, 1);
    // Merge a byte in the low lane and one in the top lane.
    Word128 upd = base;
    set_byte(upd, 0,  0xAA);
    set_byte(upd, 15, 0x55);
    Word128 raw = base;
    set_byte(raw, 0,  0xAA);
    set_byte(raw, 15, 0x55);
    (void)do_write(0x200, raw, /*strb=*/0x8001, /*awid=*/2);
    Word128 got{};
    uint8_t id_tmp;
    (void)do_read(0x200, &got, &id_tmp, /*arid=*/2);
    bool ok = w128_eq(got, upd);
    if (!ok) {
        printf("   expected %s\n", w128_hex(upd).c_str());
        printf("   got      %s\n", w128_hex(got).c_str());
    }
    CHECK("byte_strobe_merges_into_beat", ok);
}

static void scenario_narrow_lane_accesses() {
    reset();
    wait_cal_done();

    Word128 lane1{};
    lane1[1] = 0x11223344u;
    bool w1 = do_write_raw(0x104, lane1, /*strb=*/0x00F0,
                           /*awid=*/6, /*awsize=*/2, /*awburst=*/1,
                           /*expect_bresp=*/0);

    Word128 lane3{};
    lane3[3] = 0xA1B2C3D4u;
    bool w3 = do_write_raw(0x10C, lane3, /*strb=*/0xF000,
                           /*awid=*/7, /*awsize=*/2, /*awburst=*/1,
                           /*expect_bresp=*/0);

    Word128 got1{};
    uint8_t rid_tmp = 0;
    bool r1 = do_read_raw(0x104, &got1, &rid_tmp, /*arid=*/6,
                          /*arsize=*/2, /*arburst=*/1,
                          /*expect_rresp=*/0);
    bool lane1_ok = (got1[1] == 0x11223344u) && (got1[3] == 0xA1B2C3D4u);

    Word128 got3{};
    bool r3 = do_read_raw(0x10C, &got3, &rid_tmp, /*arid=*/7,
                          /*arsize=*/2, /*arburst=*/1,
                          /*expect_rresp=*/0);
    bool lane3_ok = (got3[1] == 0x11223344u) && (got3[3] == 0xA1B2C3D4u);

    CHECK("narrow_lane_accesses_roundtrip", w1 && w3 && r1 && r3 &&
          lane1_ok && lane3_ok);
}

static void scenario_narrow_burst_byte_lanes() {
    reset();
    wait_cal_done();

    Word128 base = w128_from_u64(0x1111111122222222ULL,
                                 0x3333333344444444ULL);
    bool prime_ok = do_write(0x600, base, 0xFFFF, /*awid=*/10);

    std::vector<Word128> beats(4);
    beats[0][0] = 0xAAAA5555u;
    beats[1][1] = 0xBBBB6666u;
    beats[2][2] = 0xCCCC7777u;
    beats[3][3] = 0xDDDD8888u;
    std::vector<uint16_t> strbs = {
        0x0003,  // low two bytes of lane 0
        0x00C0,  // high two bytes of lane 1
        0x0F00,  // all bytes of lane 2
        0x1000   // low byte of lane 3
    };

    bool burst_ok = do_burst_write_raw(0x600, beats, strbs,
                                       /*awid=*/11, /*awsize=*/2,
                                       /*awburst=*/1, /*expect_bresp=*/0);

    Word128 expect = base;
    set_byte(expect, 0, 0x55);
    set_byte(expect, 1, 0x55);
    set_byte(expect, 6, 0xBB);
    set_byte(expect, 7, 0xBB);
    expect[2] = 0xCCCC7777u;
    set_byte(expect, 12, 0x88);

    Word128 got{};
    uint8_t rid_tmp = 0;
    bool read_ok = do_read(0x600, &got, &rid_tmp, /*arid=*/12);
    bool data_ok = w128_eq(got, expect);
    if (!data_ok) {
        printf("   expected %s\n", w128_hex(expect).c_str());
        printf("   got      %s\n", w128_hex(got).c_str());
    }
    CHECK("narrow_burst_byte_lanes_preserve_neighbors",
          prime_ok && burst_ok && read_ok && data_ok);
}

static void scenario_reset_retrains() {
    reset();
    wait_cal_done();
    Word128 payload = w128_from_u64(0x0123456789ABCDEFULL, 0xF0E1D2C3B4A59687ULL);
    (void)do_write(0x300, payload, 0xFFFF, 3);
    dut->rst = 1;
    tick();
    bool dropped = (dut->ddr_cal_done == 0) &&
                   (dut->awready == 0) &&
                   (dut->arready == 0);
    for (int i = 0; i < 4; i++) tick();
    bool still_low = (dut->ddr_cal_done == 0);
    dut->rst = 0;
    wait_cal_done();
    bool restored = (dut->ddr_cal_done == 1) &&
                    (dut->awready == 1) &&
                    (dut->arready == 1);
    CHECK("reset_retrains_ddr_and_reopens_bus", dropped && still_low && restored);
}

static void scenario_burst_wr_rd() {
    reset();
    wait_cal_done();
    const size_t N = 8;
    std::vector<Word128> beats;
    for (size_t i = 0; i < N; i++) {
        beats.push_back(w128_from_u64(0xA000000000000000ULL | i,
                                      0xB000000000000000ULL | i));
    }
    bool wok = do_burst_write(0x400, beats, /*awid=*/3);
    std::vector<Word128> got;
    bool rok = do_burst_read(0x400, N, &got, /*arid=*/3);
    bool all_match = got.size() == N;
    if (all_match) {
        for (size_t i = 0; i < N; i++) {
            if (!w128_eq(got[i], beats[i])) { all_match = false; break; }
        }
    }
    CHECK("multi_beat_incr_burst_roundtrip", wok && rok && all_match);
}

static void scenario_id_echo() {
    reset();
    wait_cal_done();
    // BID must echo exact AWID across multiple writes.
    Word128 p = w128_from_u64(0, 0);
    bool all_ok = true;
    for (uint8_t id = 0; id < 64; id++) {
        p[0] = id;
        bool w = do_write(0x1000 + (uint32_t)id * 16u, p, 0xFFFF, id);
        if (!w) { all_ok = false; break; }
    }
    for (uint8_t id = 0; id < 64; id++) {
        Word128 got;
        uint8_t rid_val;
        if (!do_read(0x1000 + (uint32_t)id * 16u, &got, &rid_val, id)) {
            all_ok = false; break;
        }
        if (rid_val != id)              { all_ok = false; break; }
        if (((uint32_t)got[0] & 0xFF) != id) { all_ok = false; break; }
    }
    CHECK("awid_arid_echo_in_bid_rid", all_ok);
}

static void scenario_write_then_read_same_addr_across_burst() {
    reset();
    wait_cal_done();
    // Burst write 4 beats starting at 0x800
    std::vector<Word128> beats = {
        w128_from_u64(0x0001, 0),
        w128_from_u64(0x0002, 0),
        w128_from_u64(0x0003, 0),
        w128_from_u64(0x0004, 0)
    };
    bool wok = do_burst_write(0x800, beats, 1);
    // Read back beat 2 (addr 0x820) as a single-beat read
    Word128 got;
    uint8_t id_tmp;
    bool rok = do_read(0x820, &got, &id_tmp, 1);
    bool match = w128_eq(got, beats[2]);
    CHECK("write_then_read_same_addr_coherent", wok && rok && match);
}

static void scenario_no_txn_before_cal() {
    // Keep in reset, verify awready/arready low.  Then release reset but
    // don't wait — awready should still be low until cal_done rises.
    reset();
    // Immediately after dereset.
    dut->eval();
    bool ok_pre = (dut->awready == 0) && (dut->arready == 0) &&
                  (dut->ddr_cal_done == 0);
    // Try to drive AWVALID; expect no AWREADY until cal_done.
    dut->awid = 0; dut->awaddr = 0; dut->awlen = 0;
    dut->awsize = 4; dut->awburst = 1; dut->awvalid = 1;
    bool seen_aw_during_cal = false;
    while (!dut->ddr_cal_done) {
        if (dut->awready) { seen_aw_during_cal = true; break; }
        tick();
    }
    dut->awvalid = 0;
    CHECK("no_txn_accepted_before_cal_done",
          ok_pre && !seen_aw_during_cal);
}

static void scenario_illegal_width_or_alignment_slverr() {
    reset();
    wait_cal_done();
    Word128 data = w128_from_u64(0xAA55AA55AA55AA55ULL, 0x1122334455667788ULL);
    Word128 got{};
    uint8_t rid_tmp = 0;

    bool bad_write = do_write_raw(0x104, data, 0xFFFF, /*awid=*/4,
                                  /*awsize=*/4, /*awburst=*/1,
                                  /*expect_bresp=*/2);
    bool bad_read = do_read_raw(0x104, &got, &rid_tmp, /*arid=*/5,
                                /*arsize=*/3, /*arburst=*/1,
                                /*expect_rresp=*/2);
    bool no_leak = !w128_eq(got, data);
    CHECK("illegal_width_or_alignment_return_slverr",
          bad_write && bad_read && no_leak);
}

static void scenario_illegal_burst_type_slverr() {
    reset();
    wait_cal_done();
    const uint32_t addr = 0x900;
    Word128 base = w128_from_u64(0x1122334455667788ULL,
                                 0x99AABBCCDDEEFF00ULL);
    Word128 bad  = w128_from_u64(0x8877665544332211ULL,
                                 0x00FFEEDDCCBBAA99ULL);

    bool prime_ok = do_write(addr, base, 0xFFFF, /*awid=*/6);
    bool bad_wr_ok = do_write_raw(addr, bad, 0xFFFF,
                                  /*awid=*/7, /*awsize=*/4,
                                  /*awburst=*/0, /*expect_bresp=*/2);

    Word128 got{};
    uint8_t rid_tmp = 0;
    bool bad_rd_ok = do_read_raw(addr, &got, &rid_tmp,
                                 /*arid=*/8, /*arsize=*/4,
                                 /*arburst=*/2, /*expect_rresp=*/2);

    Word128 after{};
    uint8_t id_tmp = 0;
    bool still_ok = do_read(addr, &after, &id_tmp, /*arid=*/9);

    bool no_mutation = w128_eq(after, base);
    bool no_leak = !w128_eq(got, base);
    CHECK("illegal_burst_type_return_slverr",
          prime_ok && bad_wr_ok && bad_rd_ok && still_ok &&
          no_mutation && no_leak);
}

static void scenario_perf_counters() {
    reset();
    wait_cal_done();
    uint32_t aw0 = dut->dbg_aw_cnt;
    uint32_t b0  = dut->dbg_b_cnt;
    Word128 p = w128_from_u64(0, 0);
    (void)do_write(0x2000, p, 0xFFFF, 1);
    (void)do_write(0x2010, p, 0xFFFF, 2);
    Word128 got; uint8_t id_tmp;
    (void)do_read(0x2000, &got, &id_tmp, 1);
    // AW counter should advance by exactly 2, B by exactly 2.
    uint32_t aw1 = dut->dbg_aw_cnt;
    uint32_t b1  = dut->dbg_b_cnt;
    CHECK("perf_counters_advance",
          (aw1 - aw0 == 2) && (b1 - b0 == 2) && (dut->dbg_r_cnt >= 1));
}

// ── Main ──────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_ddr_model;
    dut->clk = 0;
    dut->eval();

    scenario_cal_done();
    scenario_no_txn_before_cal();
    scenario_single_beat_wr_rd();
    scenario_byte_strobe();
    scenario_narrow_lane_accesses();
    scenario_burst_wr_rd();
    scenario_narrow_burst_byte_lanes();
    scenario_id_echo();
    scenario_write_then_read_same_addr_across_burst();
    scenario_perf_counters();
    scenario_reset_retrains();
    scenario_illegal_width_or_alignment_slverr();
    scenario_illegal_burst_type_slverr();

    printf("\n");
    if (n_fail == 0) {
        printf("All %d scenarios PASSED.\n", n_pass);
    } else {
        printf("%d scenarios PASSED, %d FAILED.\n", n_pass, n_fail);
    }
    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
