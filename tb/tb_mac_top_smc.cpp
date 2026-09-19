// tb_mac_top_smc.cpp — SMC snoop export verification (l1i-smc-snoop
// placeholder).  The SMC consumer (I-cache snoop receiver) does not yet
// exist at the time this tb was written — task #73 (l1i-smc-snoop) will
// add it.  Until then, the dcache.v module exposes snoop_valid /
// snoop_addr on its port list but m68k_core.v ties them off empty.
//
// Rather than build a full-top harness just to probe a wire that isn't
// consumed yet, we instantiate dcache.v directly and verify the snoop
// line pulses on writes that overlap a representative I-cache-visible
// address range (the ROM 0x40800000+ region where instructions are
// typically fetched from, as well as RAM where self-modifying code
// would land).
//
// The pre-condition for the future SMC receiver is:
//   - snoop_valid pulses for exactly one cycle per committed D-side
//     write (hit or write-allocate miss).
//   - snoop_addr is line-aligned (bottom 5 bits = 0, matching the
//     32 B cache line size).
//   - snoop is NOT generated on non-cacheable writes (I/O range).
//   - snoop is NOT generated on loads.
//   - snoop is NOT generated during flush_all writeback beats (those
//     are internal bookkeeping, not fresh CPU stores).
//
// When task #73 lands and wires snoop into the I-cache, this tb should
// be extended to instantiate both halves (dcache + icache) and verify
// the full snoop path.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>
#include <verilated.h>
#include "Vdcache.h"

static Vdcache* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;

// ─── Minimal AXI slave BFM (same shape as tb_dcache) ────────────────────
struct AxiMem {
    std::map<uint32_t, uint8_t> store;
    uint8_t byte_at(uint32_t a) {
        auto it = store.find(a);
        return (it == store.end()) ? 0 : it->second;
    }
    uint32_t read_line_word(uint32_t a) {
        uint32_t v = 0;
        for (int i = 0; i < 4; i++)
            v |= ((uint32_t)byte_at(a + i)) << ((3 - i) * 8);
        return v;
    }
    void write_line_word(uint32_t a, uint32_t w, uint8_t strb) {
        for (int i = 0; i < 4; i++)
            if (strb & (1u << (3 - i)))
                store[a + i] = (uint8_t)((w >> ((3 - i) * 8)) & 0xFF);
    }
};
static AxiMem mem;

struct AxiSlave {
    bool ar_cap = false;
    uint32_t ar_addr = 0;
    int r_delay = 0;
    bool r_pending = false;
    bool aw_cap = false;
    uint32_t aw_addr = 0;
    bool w_cap = false;
    uint32_t w_data = 0;
    uint8_t w_strb = 0;
    int b_delay = 0;
    bool b_pending = false;
};
static AxiSlave slv;

static void drive_slave() {
    dut->ar_ready = !slv.ar_cap && !slv.r_pending;
    dut->aw_ready = !slv.aw_cap && !slv.b_pending;
    dut->w_ready  = !slv.w_cap  && !slv.b_pending;
    dut->r_valid  = slv.r_pending;
    dut->r_data   = slv.r_pending ? mem.read_line_word(slv.ar_addr) : 0u;
    dut->r_resp   = 0;
    dut->b_valid  = slv.b_pending;
    dut->b_resp   = 0;
    dut->eval();
}

static void tick() {
    drive_slave();
    uint8_t  pre_ar_v = dut->ar_valid, pre_ar_r = dut->ar_ready;
    uint32_t pre_ar_a = dut->ar_addr;
    uint8_t  pre_aw_v = dut->aw_valid, pre_aw_r = dut->aw_ready;
    uint32_t pre_aw_a = dut->aw_addr;
    uint8_t  pre_w_v = dut->w_valid, pre_w_r = dut->w_ready;
    uint32_t pre_w_d = dut->w_data;
    uint8_t  pre_w_s = dut->w_strb;
    uint8_t  pre_r_v = dut->r_valid, pre_r_r = dut->r_ready;
    uint8_t  pre_b_v = dut->b_valid, pre_b_r = dut->b_ready;

    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();

    if (pre_ar_v && pre_ar_r && !slv.ar_cap) {
        slv.ar_cap = true; slv.ar_addr = pre_ar_a; slv.r_delay = 1;
    }
    if (slv.ar_cap && !slv.r_pending) {
        if (slv.r_delay > 0) slv.r_delay--;
        else slv.r_pending = true;
    }
    if (pre_r_v && pre_r_r) { slv.r_pending = false; slv.ar_cap = false; }

    if (pre_aw_v && pre_aw_r && !slv.aw_cap) { slv.aw_cap = true; slv.aw_addr = pre_aw_a; }
    if (pre_w_v && pre_w_r && !slv.w_cap)    { slv.w_cap = true; slv.w_data = pre_w_d; slv.w_strb = pre_w_s; }
    if (slv.aw_cap && slv.w_cap && !slv.b_pending) {
        mem.write_line_word(slv.aw_addr, slv.w_data, slv.w_strb);
        slv.aw_cap = slv.w_cap = false;
        slv.b_pending = true;
    }
    if (pre_b_v && pre_b_r) slv.b_pending = false;
    sim_time++;
}

static void reset() {
    dut->rst = 1;
    dut->req = 0; dut->is_write = 0; dut->addr = 0;
    dut->wdata = 0; dut->wstrb = 0;
    dut->cache_enable = 1;
    dut->cache_inh = 0;
    // Task #92 rewrite: obsolete flush_req/inval_req were collapsed into
    // the maint_req / maint_scope / maint_is_inv / maint_addr bundle.
    // This tb only cares about the SMC snoop export path, which is
    // unchanged; zero the maint bundle and the flush walker here.
    dut->maint_req = 0; dut->maint_is_inv = 0;
    dut->maint_scope = 0; dut->maint_addr = 0;
    dut->flush_all_req = 0; dut->flush_all_inval = 0;
    slv = AxiSlave{};
    mem.store.clear();
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

// Record snoop pulses observed during a given operation.
struct SnoopLog { std::vector<uint32_t> addrs; };

static void do_load(uint32_t addr, SnoopLog* log = nullptr, bool cache_inh = false) {
    dut->req = 1; dut->is_write = 0; dut->addr = addr;
    dut->cache_inh = cache_inh ? 1 : 0;
    for (int i = 0; i < 1000; i++) {
        tick();
        if (log && dut->snoop_valid) log->addrs.push_back(dut->snoop_addr);
        if (dut->rvalid) break;
    }
    dut->req = 0;
    dut->cache_inh = 0;
    for (int i = 0; i < 3; i++) {
        tick();
        if (log && dut->snoop_valid) log->addrs.push_back(dut->snoop_addr);
    }
}

static void do_store(uint32_t addr, uint32_t data, uint8_t strb = 0xF,
                     SnoopLog* log = nullptr, bool cache_inh = false) {
    dut->req = 1; dut->is_write = 1; dut->addr = addr;
    dut->wdata = data; dut->wstrb = strb;
    dut->cache_inh = cache_inh ? 1 : 0;
    for (int i = 0; i < 1000; i++) {
        tick();
        if (log && dut->snoop_valid) log->addrs.push_back(dut->snoop_addr);
        if (dut->bvalid) break;
    }
    dut->req = 0;
    dut->cache_inh = 0;
    for (int i = 0; i < 3; i++) {
        tick();
        if (log && dut->snoop_valid) log->addrs.push_back(dut->snoop_addr);
    }
}

static void mem_write32_be(uint32_t a, uint32_t v) {
    for (int i = 0; i < 4; i++) mem.store[a + i] = (v >> ((3-i)*8)) & 0xFF;
}

#define CHECK_EQ(msg, got, exp) do { \
    if ((uint32_t)(got) != (uint32_t)(exp)) { \
        std::printf("    FAIL %s: got 0x%x, expected 0x%x\n", msg, \
                    (uint32_t)(got), (uint32_t)(exp)); \
        return false; \
    } \
} while (0)
#define CHECK_TRUE(msg, cond) do { \
    if (!(cond)) { std::printf("    FAIL %s: condition false\n", msg); return false; } \
} while (0)

// ─── Scenarios ──────────────────────────────────────────────────────────
// 1. Store to RAM at an address an I-fetch would touch → snoop pulses
//    with line-aligned address.
static bool test_smc_ram_store() {
    reset();
    // RAM at 0x00001000 (line 0x1000..0x101F).  I-fetch at 0x1008 would
    // hit this line; a store at 0x1004 should snoop-broadcast 0x1000.
    uint32_t line = 0x00001000u;
    // Prime memory so fill-on-miss doesn't stall.
    for (int i = 0; i < 8; i++) mem_write32_be(line + i*4, 0);
    SnoopLog log;
    do_store(line + 4, 0xDEADBEEF, 0xF, &log);
    CHECK_TRUE("snoop pulsed on RAM store", !log.addrs.empty());
    CHECK_EQ("snoop addr line-aligned", log.addrs[0], line);
    return true;
}

// 2. Store to ROM range (0x40800000+) → cacheable, should also snoop.
//    ROM is read-only in real HW, but dcache treats it as cacheable
//    and a store would land in the cache (dirty line eviction would
//    fault via SLVERR when written back to ROM, but that's a separate
//    path).  The SMC snoop itself must still fire — self-modifying
//    code in RAM-overlaid ROM is exactly the case the snoop guards.
static bool test_smc_rom_range_store() {
    reset();
    uint32_t line = 0x40800000u;
    for (int i = 0; i < 8; i++) mem_write32_be(line + i*4, 0);
    SnoopLog log;
    do_store(line + 8, 0xCAFEBABEu, 0xF, &log);
    CHECK_TRUE("snoop pulsed on ROM-range store", !log.addrs.empty());
    CHECK_EQ("snoop addr line-aligned", log.addrs[0], line);
    return true;
}

// 3. Non-cacheable store (I/O range) must NOT generate a snoop — those
//    addresses are never fetched as instructions.
static bool test_smc_no_snoop_on_bypass() {
    reset();
    SnoopLog log;
    do_store(0x50001000u, 0xABCDEF00u, 0xF, &log, true);
    CHECK_TRUE("no snoop on I/O bypass store", log.addrs.empty());
    // Sentinel address too
    SnoopLog log2;
    do_store(0xFFFF0000u, 0xC0FFEE00u, 0xF, &log2, true);
    CHECK_TRUE("no snoop on sentinel store", log2.addrs.empty());
    return true;
}

// 4. Loads must NOT generate snoops.
static bool test_smc_no_snoop_on_load() {
    reset();
    for (int i = 0; i < 8; i++) mem_write32_be(0x2000 + i*4, 0xA5A5A5A5u);
    SnoopLog log;
    do_load(0x00002000u, &log);
    CHECK_TRUE("no snoop on cache-miss load", log.addrs.empty());
    // Reload (hit) — also no snoop
    SnoopLog log2;
    do_load(0x00002004u, &log2);
    CHECK_TRUE("no snoop on cache-hit load", log2.addrs.empty());
    return true;
}

// 5. Write-hit snoops — second store to a line already in the cache
//    also snoops (so the I-cache receiver sees every CPU update).
static bool test_smc_write_hit_snoops() {
    reset();
    uint32_t line = 0x00003000u;
    for (int i = 0; i < 8; i++) mem_write32_be(line + i*4, 0);

    // First store brings the line into cache (write-allocate) + snoops.
    SnoopLog log1;
    do_store(line, 0x11111111u, 0xF, &log1);
    CHECK_TRUE("miss store snoops", !log1.addrs.empty());

    // Second store to the SAME line hits.
    SnoopLog log2;
    do_store(line + 4, 0x22222222u, 0xF, &log2);
    CHECK_TRUE("hit store also snoops", !log2.addrs.empty());
    CHECK_EQ("hit store snoop addr line-aligned",
             log2.addrs[0], line);
    return true;
}

// 6. flush_all writes: during walking writebacks, snoop must NOT pulse
//    (the CPU did not issue a fresh store — this is internal
//    housekeeping).
static bool test_smc_no_snoop_during_flush_all() {
    reset();
    uint32_t line = 0x00004000u;
    for (int i = 0; i < 8; i++) mem_write32_be(line + i*4, 0);

    // Store (dirty the line); this legitimately snoops.
    SnoopLog pre;
    do_store(line, 0xDEADBEEFu, 0xF, &pre);
    int pre_count = pre.addrs.size();

    // Fire flush_all.  Watch for additional snoops.
    SnoopLog during;
    dut->flush_all_req = 1;
    for (int i = 0; i < 5000; i++) {
        tick();
        if (dut->snoop_valid) during.addrs.push_back(dut->snoop_addr);
        if (dut->flush_all_done) break;
    }
    dut->flush_all_req = 0;
    for (int i = 0; i < 3; i++) {
        tick();
        if (dut->snoop_valid) during.addrs.push_back(dut->snoop_addr);
    }

    CHECK_TRUE("snoop WAS fired on pre-flush CPU store", pre_count > 0);
    CHECK_TRUE("NO snoop during flush_all beats",        during.addrs.empty());
    return true;
}

// ─── main ───────────────────────────────────────────────────────────────
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { std::printf("[PASS] %s\n", #fn); n_pass++; } \
    else    { std::printf("[FAIL] %s\n", #fn); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vdcache;

    RUN(test_smc_ram_store);
    RUN(test_smc_rom_range_store);
    RUN(test_smc_no_snoop_on_bypass);
    RUN(test_smc_no_snoop_on_load);
    RUN(test_smc_write_hit_snoops);
    RUN(test_smc_no_snoop_during_flush_all);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
