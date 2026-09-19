// tb_mmu_walker.cpp — Verilator unit tb for the phase-3 MMU walker + ATC.
//
// Exercises rtl/core/mem/mmu.v (plus its mmu_walker.v + mmu_atc.v
// children) standalone.  The tb plays the role of memory via a tiny
// std::map-backed AXI4 slave BFM bound to the walker-side ports
// (w_ar_*, w_r_*, w_aw_*, w_w_*, w_b_*).  All page-table entries are
// constructed in tb-memory; the DUT walks them, installs an ATC entry,
// and returns the PA.
//
// Build:   make tb-mmu-walker
// Scope:   15+ scenarios covering 4 KB/8 KB pages, 2-/3-level walks,
//          ATC hit / miss, WP faults, supervisor faults, invalid PTEs,
//          PFLUSH variants, PTEST, modified/used bit set, cross-page.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>
#include <verilated.h>
#include "Vmmu.h"

static Vmmu* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

// ── std::map-backed tb memory (page tables + pages) ─────────────────
static std::map<uint32_t, uint32_t> memw;
static std::map<uint32_t, uint8_t> mem_rresp;
static std::map<uint32_t, uint8_t> mem_bresp;

static uint32_t mem_rd(uint32_t addr) {
    // Word-aligned reads only; return 0 for untouched locations.
    auto it = memw.find(addr & ~0x3u);
    return (it == memw.end()) ? 0u : it->second;
}
static void mem_wr(uint32_t addr, uint32_t data, uint8_t strb = 0xF) {
    uint32_t a = addr & ~0x3u;
    uint32_t cur = mem_rd(a);
    uint32_t mask = 0;
    for (int b = 0; b < 4; b++)
        if (strb & (1u << b))
            mask |= 0xFFu << (b * 8);
    memw[a] = (cur & ~mask) | (data & mask);
}
static uint8_t mem_resp(uint32_t addr) {
    auto it = mem_rresp.find(addr & ~0x3u);
    return (it == mem_rresp.end()) ? 0u : it->second;
}
static void mem_set_rresp(uint32_t addr, uint8_t resp) {
    mem_rresp[addr & ~0x3u] = resp & 0x3u;
}
static uint8_t mem_write_resp(uint32_t addr) {
    auto it = mem_bresp.find(addr & ~0x3u);
    return (it == mem_bresp.end()) ? 0u : it->second;
}
static void mem_set_bresp(uint32_t addr, uint8_t resp) {
    mem_bresp[addr & ~0x3u] = resp & 0x3u;
}

// ── AXI4 slave BFM ──────────────────────────────────────────────────
// Follows the same "sample pre-edge" pattern as tb_dcache.cpp: drive
// slave outputs combinationally, tick the clock, then capture what
// both sides agreed on BEFORE the posedge.
struct AxiSlave {
    // Read side
    bool ar_cap = false;
    uint32_t ar_addr = 0;
    int  r_delay = 0;
    bool r_pending = false;
    // Write side
    bool aw_cap = false;
    uint32_t aw_addr = 0;
    bool w_cap = false;
    uint32_t w_data = 0;
    uint8_t  w_strb = 0;
    bool b_pending = false;
    uint8_t b_resp = 0;

    // Back-pressure / slow-slave knobs (set by stress scenarios).
    int  extra_r_delay = 0;   // extra cycles to stall the R response
    int  ar_stall = 0;        // cycles to hold ar_ready low each AR
    int  ar_stall_ctr = 0;
    int  b_stall = 0;         // cycles to hold b_valid=0 per B
    int  b_stall_ctr = 0;
    int  aw_stall_ctr = 0;    // one-shot cycles to hold aw_ready=0
    int  w_stall_ctr = 0;     // one-shot cycles to hold w_ready=0

    // Read-count / write-count observability (reset per scenario).
    uint64_t n_ar = 0;
    uint64_t n_aw = 0;
};

static AxiSlave slv;

static void drive_slave() {
    // ar_ready gated by capture state AND optional stall counter.
    bool ar_ok = !slv.ar_cap && !slv.r_pending && (slv.ar_stall_ctr == 0);
    dut->w_ar_ready = ar_ok;
    dut->w_aw_ready = !slv.aw_cap && !slv.b_pending && (slv.aw_stall_ctr == 0);
    dut->w_w_ready  = !slv.w_cap  && !slv.b_pending && (slv.w_stall_ctr == 0);
    dut->w_r_valid  = slv.r_pending;
    dut->w_r_data   = slv.r_pending ? mem_rd(slv.ar_addr) : 0u;
    dut->w_r_resp   = slv.r_pending ? mem_resp(slv.ar_addr) : 0u;
    // b_valid gated by an optional b_stall countdown so we can stress backpressure.
    bool b_ok = slv.b_pending && (slv.b_stall_ctr == 0);
    dut->w_b_valid  = b_ok;
    dut->w_b_resp   = b_ok ? slv.b_resp : 0;
    dut->eval();
}

static bool debug_axi = false;

// ── Clock helpers ───────────────────────────────────────────────────
static void tick() {
    drive_slave();

    // Sample pre-edge signals.
    uint8_t  pre_ar_v = dut->w_ar_valid;
    uint8_t  pre_ar_r = dut->w_ar_ready;
    uint32_t pre_ar_a = dut->w_ar_addr;
    uint8_t  pre_aw_v = dut->w_aw_valid;
    uint8_t  pre_aw_r = dut->w_aw_ready;
    uint32_t pre_aw_a = dut->w_aw_addr;
    uint8_t  pre_w_v  = dut->w_w_valid;
    uint8_t  pre_w_r  = dut->w_w_ready;
    uint32_t pre_w_d  = dut->w_w_data;
    uint8_t  pre_w_s  = dut->w_w_strb;
    uint8_t  pre_r_v  = dut->w_r_valid;
    uint8_t  pre_r_r  = dut->w_r_ready;
    uint8_t  pre_b_v  = dut->w_b_valid;
    uint8_t  pre_b_r  = dut->w_b_ready;

    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;

    // AR handshake
    if (pre_ar_v && pre_ar_r && !slv.ar_cap) {
        slv.ar_cap  = true;
        slv.ar_addr = pre_ar_a;
        // Respect configurable extra R-delay and re-arm AR stall counter.
        slv.r_delay = 1 + slv.extra_r_delay;
        slv.ar_stall_ctr = slv.ar_stall;
        slv.n_ar++;
    } else if (slv.ar_stall_ctr > 0 && !slv.ar_cap) {
        slv.ar_stall_ctr--;
    }
    if (slv.ar_cap && !slv.r_pending) {
        if (slv.r_delay > 0) slv.r_delay--;
        else slv.r_pending = true;
    }
    if (pre_r_v && pre_r_r) {
        slv.r_pending = false;
        slv.ar_cap    = false;
    }

    // AW + W handshake
    if (pre_aw_v && pre_aw_r && !slv.aw_cap) {
        slv.aw_cap  = true;
        slv.aw_addr = pre_aw_a;
        slv.n_aw++;
    } else if (pre_aw_v && !pre_aw_r && slv.aw_stall_ctr > 0 && !slv.aw_cap) {
        slv.aw_stall_ctr--;
    }
    if (pre_w_v && pre_w_r && !slv.w_cap) {
        slv.w_cap  = true;
        slv.w_data = pre_w_d;
        slv.w_strb = pre_w_s;
    } else if (pre_w_v && !pre_w_r && slv.w_stall_ctr > 0 && !slv.w_cap) {
        slv.w_stall_ctr--;
    }
    if (slv.aw_cap && slv.w_cap && !slv.b_pending) {
        slv.b_resp = mem_write_resp(slv.aw_addr);
        if (slv.b_resp == 0)
            mem_wr(slv.aw_addr, slv.w_data, slv.w_strb);
        slv.aw_cap    = false;
        slv.w_cap     = false;
        slv.b_pending = true;
        slv.b_stall_ctr = slv.b_stall;
    } else if (slv.b_pending && slv.b_stall_ctr > 0) {
        slv.b_stall_ctr--;
    }
    if (pre_b_v && pre_b_r) slv.b_pending = false;

    if (debug_axi) {
        fprintf(stderr, "t=%llu arv=%d arr=%d ara=0x%08x rv=%d rd=0x%08x rr=%d | "
                         "awv=%d awr=%d awa=0x%08x wv=%d wd=0x%08x wr=%d bv=%d | "
                         "respR=%d paO=0x%08x flt=%d fc=%d fa=0x%08x\n",
                         (unsigned long long)sim_time,
                         pre_ar_v, pre_ar_r, pre_ar_a,
                         pre_r_v, dut->w_r_data, pre_r_r,
                         pre_aw_v, pre_aw_r, pre_aw_a,
                         pre_w_v, pre_w_d, pre_w_r,
                         pre_b_v,
                         dut->resp_ready, dut->pa_out, dut->fault, dut->fault_code_out,
                         dut->fault_addr_out);
    }
}

static void reset() {
    memw.clear();
    mem_rresp.clear();
    mem_bresp.clear();
    slv = AxiSlave();
    dut->rst = 1;
    dut->va_in = 0;
    dut->is_instruction = 0;
    dut->is_write = 0;
    dut->supervisor = 0;
    dut->wr_en = 0;
    dut->wr_cr = 0;
    dut->wr_val = 0;
    dut->req_valid = 0;
    dut->pflush_all_req = 0;
    dut->pflush_va_req = 0;
    dut->pflush_asid_req = 0;
    dut->pflush_addr = 0;
    dut->pflush_sup = 0;
    dut->ptest_req = 0;
    dut->ptest_va = 0;
    dut->ptest_is_write = 0;
    dut->ptest_sup = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
}

static void write_cr(int cr, uint32_t val) {
    dut->wr_en = 1;
    dut->wr_cr = cr;
    dut->wr_val = val;
    tick();
    dut->wr_en = 0;
    dut->wr_val = 0;
    tick();
}

// ── TC bit assembly ─────────────────────────────────────────────────
// Live 68040 TC bits used by rtl/core/mem/mmu_walker.v:
//   [15]    E — enable
//   [14]    P — 0 = 4 KB, 1 = 8 KB
// The legacy IS/TIA/TIB fields are still filled by some tests for API
// compatibility, but the 68040 walker uses fixed 7/7/root-pointer-page
// table slices matching MAME's 040 PMMU.
static uint32_t mk_tc(bool enable, bool psz8k, int is_field,
                      int tia_bits, int tib_bits) {
    uint32_t v = 0;
    if (enable) v |= (1u << 15);
    if (psz8k)  v |= (1u << 14);
    v |= (uint32_t)(is_field & 0xF) << 16;
    v |= (uint32_t)(tia_bits & 0xF) << 8;
    v |= (uint32_t)(tib_bits & 0xF) << 4;
    return v;
}

// ── Page-table helpers ──────────────────────────────────────────────
static constexpr uint32_t DESC_DT_INVALID  = 0x0u;
static constexpr uint32_t DESC_DT_RESIDENT = 0x1u;
static constexpr uint32_t DESC_DT_TABLE4   = 0x2u;
static constexpr uint32_t DESC_WP          = 1u << 2;
static constexpr uint32_t DESC_U           = 1u << 3;
static constexpr uint32_t DESC_M           = 1u << 4;
static constexpr uint32_t DESC_CI          = 1u << 6;
static constexpr uint32_t DESC_S           = 1u << 7;

// Pointer descriptor: [31:4]=addr, [1:0]=10 table4, [2]=WP, [3]=U.
static uint32_t mk_ptr(uint32_t next_addr, bool wp, bool used) {
    uint32_t v = (next_addr & 0xFFFFFFF0u) | DESC_DT_TABLE4;
    if (wp)   v |= DESC_WP;
    if (used) v |= DESC_U;
    return v;
}
// Page descriptor: [31:pg_bits]=PFN, [7]=S, [6]=CI, [4]=M,
// [3]=U, [2]=WP, [1:0]=01 resident page.
static uint32_t mk_page(uint32_t pfn_phys, int pg_bits, bool wp,
                        bool sup_only, bool modified, bool used,
                        bool cache_inh) {
    uint32_t mask = ~((1u << pg_bits) - 1);
    uint32_t v = (pfn_phys & mask) | DESC_DT_RESIDENT;
    if (wp)        v |= DESC_WP;
    if (used)      v |= DESC_U;
    if (modified)  v |= DESC_M;
    if (cache_inh) v |= DESC_CI;
    if (sup_only)  v |= DESC_S;
    return v;
}
static uint32_t mk_page_invalid() {
    return DESC_DT_INVALID;
}
static uint32_t mk_ptr_invalid() {
    return 0u;
}

// ── Drive a translation request and run until the walker completes
//    (or timeout) ─────────────────────────────────────────────────
struct XlateResult {
    uint32_t pa;
    bool     fault;
    uint32_t fault_addr;
    uint32_t fault_code;
};

static XlateResult xlate(uint32_t va, bool is_write, bool sup,
                         int timeout = 400) {
    dut->va_in          = va;
    dut->is_instruction = 0;
    dut->is_write       = is_write ? 1 : 0;
    dut->supervisor     = sup ? 1 : 0;
    dut->req_valid      = 1;
    XlateResult r = {0, false, 0, 0};

    // Wait one cycle so the DUT can detect "need_walk" and start the
    // walker (latching req_pending / w_start).
    tick();

    // If the request could be served combinationally (ITT hit, ATC hit,
    // MMU disabled), resp_ready stays high the whole time and pa_out /
    // fault are already valid.  Capture them now.
    if (dut->resp_ready) {
        r.pa         = dut->pa_out;
        r.fault      = dut->fault != 0;
        r.fault_addr = dut->fault_addr_out;
        r.fault_code = dut->fault_code_out;
        dut->req_valid = 0;
        tick();
        return r;
    }

    // Otherwise the walker is now engaged.  Wait for it to complete.
    // We consider "complete" when resp_ready returns high — that's the
    // cycle after w_done_{ok|fault} pulsed and last_* latched.
    for (int i = 0; i < timeout; i++) {
        tick();
        if (dut->resp_ready) {
            // Give one more cycle for the ATC fill to settle so a
            // subsequent probe sees the installed entry.
            r.pa         = dut->pa_out;
            r.fault      = dut->fault != 0;
            r.fault_addr = dut->fault_addr_out;
            r.fault_code = dut->fault_code_out;
            break;
        }
    }
    dut->req_valid = 0;
    tick();
    return r;
}

// ── Check macros ────────────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    if ((uint32_t)(got) != (uint32_t)(exp)) { \
        printf("  FAIL %s: got 0x%08x, expected 0x%08x\n", \
               name, (uint32_t)(got), (uint32_t)(exp)); \
        return false; \
    } \
} while(0)
#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { printf("  FAIL %s: expected true\n", name); return false; } \
} while(0)
#define CHECK_FALSE(name, cond) do { \
    if (cond)    { printf("  FAIL %s: expected false\n", name); return false; } \
} while(0)

static uint32_t atc_cnt();

// ── Page-table builder for a given VA → PA mapping (3-level, 4 KB) ─
// Root (URP/SRP) points to L1.  Choose fixed addresses for L1/L2 tables
// and install one entry each along the path.
//
//   L1 at 0x00010000 (128 × 4 B = 512 B; we use only one slot)
//   L2 at 0x00020000 (ditto)
static void build_3lvl_4k(uint32_t va, uint32_t pa,
                          bool wp = false, bool sup_only = false,
                          bool cache_inh = false,
                          bool used = true, bool modified = false) {
    uint32_t l1 = 0x10000;
    uint32_t l2 = 0x20000;
    int tia = 7, tib = 7;
    int pg_bits = 12;

    uint32_t l1_idx  = (va >> (32 - tia)) & ((1u << tia) - 1);
    uint32_t l2_idx  = ((va << tia) >> (32 - tib)) & ((1u << tib) - 1);
    uint32_t leaf_bits = 32 - tia - tib - pg_bits;  // 6
    uint32_t leaf_idx = (va >> pg_bits) & ((1u << leaf_bits) - 1);
    // L3 (leaf) table: pick a unique address per (l1_idx, l2_idx).
    uint32_t l3 = 0x100000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);

    mem_wr(l1 + l1_idx * 4, mk_ptr(l2, /*wp=*/false, /*used=*/true));
    mem_wr(l2 + l2_idx * 4, mk_ptr(l3, /*wp=*/false, /*used=*/true));
    mem_wr(l3 + leaf_idx * 4,
           mk_page(pa, pg_bits, wp, sup_only, modified, used, cache_inh));
}

// 8 KB + 68040 fixed 3-level: root=VA[31:25], ptr=VA[24:18],
// page=VA[17:13].
static void build_3lvl_8k(uint32_t va, uint32_t pa, bool wp = false) {
    uint32_t l1 = 0x10000;
    uint32_t l2 = 0x20000;
    int pg_bits = 13;
    uint32_t l1_idx  = (va >> 25) & 0x7f;
    uint32_t l2_idx  = (va >> 18) & 0x7f;
    uint32_t leaf_idx = (va >> pg_bits) & 0x1f;
    uint32_t l3 = 0x100000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);
    mem_wr(l1 + l1_idx * 4, mk_ptr(l2, false, true));
    mem_wr(l2 + l2_idx * 4, mk_ptr(l3, false, true));
    mem_wr(l3 + leaf_idx * 4,
           mk_page(pa, pg_bits, wp, false, false, true, false));
}

// Kept for older test names.  The live 68040 walker is fixed 3-level,
// so build a normal 040 table even when TC legacy fields request the
// old Phase-A 2-level mode.
static void build_2lvl_4k(uint32_t va, uint32_t pa) {
    build_3lvl_4k(va, pa);
}

struct Walk4kPath {
    uint32_t l1_idx;
    uint32_t l2_idx;
    uint32_t leaf_idx;
    uint32_t l1_addr;
    uint32_t l2_addr;
    uint32_t l3_base;
    uint32_t leaf_addr;
};

static Walk4kPath path_4k(uint32_t root_pointer, uint32_t l2_base,
                           uint32_t l3_seed, uint32_t va) {
    uint32_t l1_base = root_pointer & ~0x3u;
    uint32_t l2_aligned = l2_base & 0xFFFFFFF0u;
    uint32_t l3_aligned = l3_seed & 0xFFFFFFF0u;
    Walk4kPath p = {};
    p.l1_idx = (va >> 25) & 0x7Fu;
    p.l2_idx = (va >> 18) & 0x7Fu;
    p.leaf_idx = (va >> 12) & 0x3Fu;
    p.l3_base = l3_aligned + (p.l1_idx * 0x10000u) + (p.l2_idx * 0x1000u);
    p.l1_addr = l1_base + p.l1_idx * 4u;
    p.l2_addr = l2_aligned + p.l2_idx * 4u;
    p.leaf_addr = p.l3_base + p.leaf_idx * 4u;
    return p;
}

static void build_3lvl_4k_at(uint32_t root_pointer, uint32_t l2_base,
                             uint32_t l3_seed, uint32_t va, uint32_t pa,
                             bool page_wp = false,
                             bool sup_only = false,
                             bool cache_inh = false,
                             bool leaf_used = true,
                             bool modified = false,
                             bool l1_wp = false,
                             bool l2_wp = false,
                             bool l1_used = true,
                             bool l2_used = true) {
    Walk4kPath p = path_4k(root_pointer, l2_base, l3_seed, va);
    mem_wr(p.l1_addr, mk_ptr(l2_base, l1_wp, l1_used));
    mem_wr(p.l2_addr, mk_ptr(p.l3_base, l2_wp, l2_used));
    mem_wr(p.leaf_addr,
           mk_page(pa, 12, page_wp, sup_only, modified, leaf_used, cache_inh));
}

// Enable MMU with 3-level 4 KB configuration
static void cfg_3lvl_4k() {
    write_cr(5, 0x10000);   // URP
    write_cr(6, 0x10000);   // SRP
    write_cr(4, mk_tc(true, false, 0, 7, 7));
}
static void cfg_3lvl_8k() {
    write_cr(5, 0x10000); write_cr(6, 0x10000);
    write_cr(4, mk_tc(true, true, 0, 6, 6));
}
static void cfg_2lvl_4k() {
    write_cr(5, 0x10000); write_cr(6, 0x10000);
    write_cr(4, mk_tc(true, false, 1, 0, 7));
}

static uint32_t mk_ttr(uint8_t base, uint8_t mask, uint8_t s_fld, bool wp) {
    uint32_t v = ((uint32_t)base << 24) | ((uint32_t)mask << 16);
    v |= 1u << 15;                 // E
    v |= (uint32_t)(s_fld & 3u) << 13;
    if (wp) v |= DESC_WP;
    return v;
}

// ════════════════════════════════════════════════════════════════════
// Scenarios
// ════════════════════════════════════════════════════════════════════

static bool test_4k_3lvl_miss_then_hit() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000);
    auto r = xlate(0x80001ABC, /*wr=*/0, /*sup=*/1);
    CHECK_FALSE("no fault", r.fault);
    CHECK_EQ("PA 4K", r.pa, 0xABCDEABCu);
    // Second request: should be an ATC hit (no walker traffic).
    auto r2 = xlate(0x80001123, 0, 1);
    CHECK_FALSE("hit no fault", r2.fault);
    CHECK_EQ("ATC hit PA", r2.pa, 0xABCDE123u);
    return true;
}

static bool test_8k_3lvl_walk() {
    reset();
    cfg_3lvl_8k();
    build_3lvl_8k(0x80002000, 0xABCDE000);
    auto r = xlate(0x80002F00, 0, 1);
    CHECK_FALSE("no fault 8K", r.fault);
    // 8 KB: low 13 bits come from VA.
    CHECK_EQ("PA 8K", r.pa, 0xABCDEF00u);
    return true;
}

static bool test_2lvl_4k_walk() {
    reset();
    cfg_2lvl_4k();
    build_2lvl_4k(0x80001000, 0x12340000);
    auto r = xlate(0x80001555, 0, 1);
    CHECK_FALSE("2lvl 4K no fault", r.fault);
    CHECK_EQ("2lvl PA", r.pa, 0x12340555u);
    return true;
}

static bool test_atc_hit_after_fill() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x40001000, 0x55555000);
    auto r1 = xlate(0x40001100, 0, 1);
    CHECK_FALSE("first no fault", r1.fault);
    CHECK_EQ("first PA", r1.pa, 0x55555100u);
    // Count AXI reads: scramble the L1 entry to invalid.  If the
    // second lookup went back to the walker it would fault; if it hit
    // ATC, success.
    int l1_tia = 7;
    uint32_t l1_idx = (0x40001100u >> (32 - l1_tia)) & ((1u << l1_tia) - 1);
    mem_wr(0x10000 + l1_idx * 4, mk_ptr_invalid());
    auto r2 = xlate(0x40001200, 0, 1);
    CHECK_FALSE("ATC hit avoids walker", r2.fault);
    CHECK_EQ("ATC hit PA", r2.pa, 0x55555200u);
    return true;
}

static bool test_held_req_valid_no_rewalk() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0x00300000);

    dut->va_in          = 0x80001000;
    dut->is_instruction = 0;
    dut->is_write       = 0;
    dut->supervisor     = 0;
    dut->req_valid      = 1;

    tick();  // start walker
    CHECK_FALSE("walker engaged", dut->resp_ready);

    bool done = false;
    uint32_t pa = 0;
    bool fault = false;
    for (int i = 0; i < 400; i++) {
        tick();
        if (dut->resp_ready) {
            done = true;
            pa = dut->pa_out;
            fault = dut->fault != 0;
            break;
        }
    }
    CHECK_TRUE("translation completed", done);
    CHECK_FALSE("translation no fault", fault);
    CHECK_EQ("translated PA", pa, 0x00300000u);

    uint64_t ar_at_ready = slv.n_ar;
    for (int i = 0; i < 16; i++) tick();
    CHECK_EQ("held req_valid did not start a second walk",
             slv.n_ar, ar_at_ready);

    dut->req_valid = 0;
    tick();
    return true;
}

static bool test_invalid_pointer() {
    reset();
    cfg_3lvl_4k();
    // Build the valid page, then clobber L1.
    build_3lvl_4k(0x80001000, 0xABCDE000);
    uint32_t l1_idx = (0x80001000u >> 25) & 0x7F;
    mem_wr(0x10000 + l1_idx * 4, mk_ptr_invalid());
    auto r = xlate(0x80001000, 0, 1);
    CHECK_TRUE("invalid ptr fault", r.fault);
    CHECK_EQ("fc invalid ptr", r.fault_code, 1);
    return true;
}

static bool test_invalid_page() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000);
    int tia = 7, tib = 7, pg_bits = 12;
    uint32_t l1_idx = (0x80001000u >> (32 - tia)) & 0x7F;
    uint32_t l2_idx = ((0x80001000u << tia) >> (32 - tib)) & 0x7F;
    uint32_t leaf_bits = 32 - tia - tib - pg_bits; // 6
    uint32_t leaf_idx = (0x80001000u >> pg_bits) & ((1u << leaf_bits) - 1);
    uint32_t l3 = 0x100000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);
    mem_wr(l3 + leaf_idx * 4, mk_page_invalid());
    auto r = xlate(0x80001000, 0, 1);
    CHECK_TRUE("invalid page fault", r.fault);
    CHECK_EQ("fc invalid page", r.fault_code, 2);
    return true;
}

static bool test_wp_violation() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000, /*wp=*/true);
    // Read is OK (WP only affects writes).
    auto r1 = xlate(0x80001000, /*wr=*/0, 1);
    CHECK_FALSE("WP read no fault", r1.fault);
    CHECK_EQ("WP read PA", r1.pa, 0xABCDE000u);
    // Write faults.
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000, /*wp=*/true);
    auto r2 = xlate(0x80001000, /*wr=*/1, 1);
    CHECK_TRUE("WP write fault", r2.fault);
    CHECK_EQ("fc WP", r2.fault_code, 3);
    return true;
}

static bool test_supervisor_only_user_fault() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000, /*wp=*/false, /*sup_only=*/true);
    auto r = xlate(0x80001000, 0, /*sup=*/0);
    CHECK_TRUE("user on sup-only page faults", r.fault);
    CHECK_EQ("fc SUP", r.fault_code, 4);
    // Sup can access just fine.
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000, /*wp=*/false, /*sup_only=*/true);
    auto r2 = xlate(0x80001000, 0, 1);
    CHECK_FALSE("sup OK on sup-only", r2.fault);
    CHECK_EQ("sup PA", r2.pa, 0xABCDE000u);
    return true;
}

static bool test_pflush_all_clears_atc() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x40001000, 0x55555000);
    auto r1 = xlate(0x40001000, 0, 1);
    CHECK_FALSE("first walk OK", r1.fault);
    // Corrupt L1; ATC should still have it.
    uint32_t l1_idx = (0x40001000u >> 25) & 0x7F;
    mem_wr(0x10000 + l1_idx * 4, mk_ptr_invalid());
    // Pflush all, then try again — should fault via walker.
    dut->pflush_all_req = 1;
    tick();
    dut->pflush_all_req = 0;
    tick();
    auto r2 = xlate(0x40001000, 0, 1);
    CHECK_TRUE("post-pflush re-walk faults", r2.fault);
    return true;
}

static bool test_pflush_va_only_one() {
    reset();
    cfg_3lvl_4k();
    // Install two mappings.
    build_3lvl_4k(0x40001000, 0x55555000);
    build_3lvl_4k(0x60001000, 0x66666000);
    (void)xlate(0x40001000, 0, 1);
    (void)xlate(0x60001000, 0, 1);
    // Poison both backings so any re-walk would fail.
    uint32_t i_a = (0x40001000u >> 25) & 0x7F;
    uint32_t i_b = (0x60001000u >> 25) & 0x7F;
    mem_wr(0x10000 + i_a * 4, mk_ptr_invalid());
    mem_wr(0x10000 + i_b * 4, mk_ptr_invalid());
    // Pflush only A.
    dut->pflush_va_req = 1;
    dut->pflush_addr   = 0x40001000;
    dut->pflush_sup    = 1;
    tick();
    dut->pflush_va_req = 0;
    tick();
    auto ra = xlate(0x40001000, 0, 1);
    auto rb = xlate(0x60001000, 0, 1);
    CHECK_TRUE("A faults (flushed)", ra.fault);
    CHECK_FALSE("B still cached", rb.fault);
    CHECK_EQ("B PA", rb.pa, 0x66666000u);
    return true;
}

static bool test_pflush_va_clears_user_and_supervisor_tags() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u, false, /*sup_only=*/false);
    auto ru = xlate(0x80001000u, 0, /*sup=*/0);
    auto rs = xlate(0x80001000u, 0, /*sup=*/1);
    CHECK_FALSE("user fill OK", ru.fault);
    CHECK_FALSE("supervisor fill OK", rs.fault);
    CHECK_EQ("user PA", ru.pa, 0xAA000000u);
    CHECK_EQ("supervisor PA", rs.pa, 0xAA000000u);
    CHECK_EQ("user+supervisor ATC entries", atc_cnt(), 2);

    dut->pflush_va_req = 1;
    dut->pflush_addr   = 0x80001000u;
    dut->pflush_sup    = 1;
    tick();
    dut->pflush_va_req = 0;
    tick();

    CHECK_EQ("VA PFLUSH drops both sup tags", atc_cnt(), 0);
    return true;
}

static bool test_pflush_asid_stub() {
    // Our ASID scaffolding is a no-op; verify that pflush_asid_req
    // clears the ATC (treated same as pflush_all for Phase-A).
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x40001000, 0x55555000);
    (void)xlate(0x40001000, 0, 1);
    uint32_t i_a = (0x40001000u >> 25) & 0x7F;
    mem_wr(0x10000 + i_a * 4, mk_ptr_invalid());
    dut->pflush_asid_req = 1;
    tick();
    dut->pflush_asid_req = 0;
    tick();
    auto r = xlate(0x40001000, 0, 1);
    CHECK_TRUE("after pflush asid stub, cached entry gone", r.fault);
    return true;
}

static bool test_ptest_probe_hits() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000);
    // Drive PTEST.  Walker runs and ptest_done pulses.
    dut->ptest_req = 1;
    dut->ptest_va  = 0x80001ABC;
    dut->ptest_is_write = 0;
    dut->ptest_sup = 1;
    // Wait for ptest_done.
    bool saw_done = false;
    uint32_t got_pa = 0;
    bool got_hit = false;
    bool got_fault = false;
    for (int i = 0; i < 200; i++) {
        tick();
        if (dut->ptest_done) {
            saw_done = true;
            got_pa = dut->ptest_pa;
            got_hit = dut->ptest_hit != 0;
            got_fault = dut->ptest_fault != 0;
            break;
        }
    }
    dut->ptest_req = 0;
    tick();
    CHECK_TRUE("ptest_done", saw_done);
    CHECK_TRUE("ptest hit", got_hit);
    CHECK_FALSE("ptest no fault", got_fault);
    CHECK_EQ("ptest PA", got_pa, 0xABCDEABCu);
    return true;
}

static bool test_ptest_ttr_fast_path() {
    reset();
    write_cr(4, mk_tc(true, false, 0, 7, 7));
    write_cr(2, 0x4000C000u); // DTT0: base 0x40, all S modes, enabled.

    dut->ptest_req = 1;
    dut->ptest_va = 0x40001234u;
    dut->ptest_is_write = 0;
    dut->ptest_sup = 1;

    bool saw_done = false;
    uint32_t got_pa = 0;
    bool got_ttr = false;
    for (int i = 0; i < 8 && !saw_done; i++) {
        tick();
        if (dut->ptest_done) {
            saw_done = true;
            got_pa = dut->ptest_pa;
            got_ttr = dut->ptest_ttr != 0;
        }
    }
    dut->ptest_req = 0;
    tick();

    CHECK_TRUE("ptest ttr done", saw_done);
    CHECK_TRUE("ptest ttr flag", got_ttr);
    CHECK_EQ("ptest ttr PA passthrough", got_pa, 0x40001234u);
    CHECK_EQ("ptest ttr no walker AR", (uint32_t)slv.n_ar, 0u);
    return true;
}

static bool test_ptest_uses_ptest_sup_root() {
    reset();
    write_cr(5, 0x10000u); // URP
    write_cr(6, 0x40000u); // SRP
    write_cr(4, mk_tc(true, false, 0, 7, 7));

    const uint32_t va = 0x81234000u;
    build_3lvl_4k_at(0x10000u, 0x20000u, 0x100000u,
                     va, 0x0AA00000u);
    build_3lvl_4k_at(0x40000u, 0x50000u, 0x180000u,
                     va, 0x0BB00000u);

    // Make the live translation input supervisor=1.  PTEST must still
    // use ptest_sup=0 (DFC user space) to choose URP.
    dut->supervisor = 1;
    dut->ptest_req = 1;
    dut->ptest_va = va;
    dut->ptest_is_write = 0;
    dut->ptest_sup = 0;

    bool saw_done = false;
    uint32_t got_pa = 0;
    for (int i = 0; i < 200 && !saw_done; i++) {
        tick();
        if (dut->ptest_done) {
            saw_done = true;
            got_pa = dut->ptest_pa;
        }
    }
    dut->ptest_req = 0;
    tick();

    CHECK_TRUE("ptest user-root done", saw_done);
    CHECK_EQ("ptest uses URP", got_pa, 0x0AA00000u);
    return true;
}

static bool test_modified_bit_set_on_write() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000, /*wp=*/false,
                  /*sup_only=*/false, /*cache_inh=*/false,
                  /*used=*/true, /*modified=*/false);
    int tia = 7, tib = 7, pg_bits = 12;
    uint32_t l1_idx = (0x80001000u >> (32 - tia)) & 0x7F;
    uint32_t l2_idx = ((0x80001000u << tia) >> (32 - tib)) & 0x7F;
    uint32_t leaf_bits = 32 - tia - tib - pg_bits;
    uint32_t leaf_idx = (0x80001000u >> pg_bits) & ((1u << leaf_bits) - 1);
    uint32_t l3 = 0x100000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);
    uint32_t before = mem_rd(l3 + leaf_idx * 4);
    CHECK_FALSE("M pre=0", (before & DESC_M) != 0);
    auto r = xlate(0x80001000, /*wr=*/1, 1);
    CHECK_FALSE("no fault", r.fault);
    uint32_t after = mem_rd(l3 + leaf_idx * 4);
    CHECK_TRUE("M set", (after & DESC_M) != 0);
    return true;
}

static bool test_used_bit_set() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000, 0xABCDE000, /*wp=*/false, /*sup_only=*/false,
                  /*cache_inh=*/false, /*used=*/false, /*modified=*/false);
    int tia = 7, tib = 7, pg_bits = 12;
    uint32_t l1_idx = (0x80001000u >> (32 - tia)) & 0x7F;
    uint32_t l2_idx = ((0x80001000u << tia) >> (32 - tib)) & 0x7F;
    uint32_t leaf_bits = 32 - tia - tib - pg_bits;
    uint32_t leaf_idx = (0x80001000u >> pg_bits) & ((1u << leaf_bits) - 1);
    uint32_t l3 = 0x100000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);
    uint32_t before = mem_rd(l3 + leaf_idx * 4);
    CHECK_FALSE("U pre=0", (before & DESC_U) != 0);
    auto r = xlate(0x80001000, /*wr=*/0, 1);
    CHECK_FALSE("no fault", r.fault);
    uint32_t after = mem_rd(l3 + leaf_idx * 4);
    CHECK_TRUE("U set", (after & DESC_U) != 0);
    return true;
}

static bool test_asm_descriptor_constants() {
    reset();
    write_cr(5, 0x00200000);  // URP
    write_cr(6, 0x00200000);  // SRP

    // Constants mirrored from mmu_pagefault_rte_basic.s:
    // L1[0x40] -> L2, L2[0x00] -> L3, L3[0x01] -> PFN 0x00300000.
    // Table descriptors use DT[1:0]=10 and page descriptors use
    // DT[1:0]=01, with U in bit 3.
    mem_wr(0x00200100, 0x0021000a);
    mem_wr(0x00210000, 0x0022000a);
    mem_wr(0x00220004, 0x00300009);

    write_cr(4, mk_tc(true, false, 0, 7, 7));
    auto r = xlate(0x80001000, /*wr=*/0, /*sup=*/false);
    CHECK_FALSE("asm constants no fault", r.fault);
    CHECK_EQ("asm constants PA", r.pa, 0x00300000u);
    CHECK_EQ("leaf U already set", mem_rd(0x00220004), 0x00300009u);
    return true;
}

static bool test_rom_descriptor_constants() {
    reset();
    write_cr(5, 0x00200000);  // URP
    write_cr(6, 0x00200000);  // SRP

    // ROM checkpoint shape: table descriptors like 0x...0a and leaf
    // descriptors like 0x...39 must translate rather than faulting on
    // the old [3:2] decode.
    mem_wr(0x00200100, 0x0021000a);
    mem_wr(0x00210000, 0x0022000a);
    mem_wr(0x00220004, 0x00300039);

    write_cr(4, mk_tc(true, false, 0, 7, 7));
    auto r = xlate(0x80001000, /*wr=*/0, /*sup=*/false);
    CHECK_FALSE("rom descriptor no fault", r.fault);
    CHECK_EQ("rom descriptor PA", r.pa, 0x00300000u);
    auto rw = xlate(0x80001000, /*wr=*/1, /*sup=*/false);
    CHECK_FALSE("rom descriptor not WP", rw.fault);
    CHECK_EQ("rom descriptor write PA", rw.pa, 0x00300000u);
    return true;
}

static bool test_cross_page_access() {
    reset();
    cfg_3lvl_4k();
    // Install two adjacent pages with different PAs.  Access first the
    // tail of page A then the head of page B.
    build_3lvl_4k(0x80001000, 0xA0000000);
    build_3lvl_4k(0x80002000, 0xB0000000);
    auto rA = xlate(0x80001FFC, 0, 1);
    CHECK_FALSE("A no fault", rA.fault);
    CHECK_EQ("A PA", rA.pa, 0xA0000FFCu);
    auto rB = xlate(0x80002000, 0, 1);
    CHECK_FALSE("B no fault", rB.fault);
    CHECK_EQ("B PA", rB.pa, 0xB0000000u);
    return true;
}

static bool test_itt_passthrough_still_works() {
    // Sanity: existing ITT/DTT path must continue to function when the
    // walker is present.
    reset();
    write_cr(4, mk_tc(true, false, 0, 7, 7));
    // ITT0: base 0x40, mask 0x00, E=1, S=1x, W=0.
    uint32_t itt = 0;
    itt |= 0x40u << 24;
    itt |= 0u   << 16;
    itt |= 1u   << 15;
    itt |= 3u   << 13;
    write_cr(0, itt);
    dut->va_in = 0x40800000;
    dut->is_instruction = 1;
    dut->is_write = 0;
    dut->supervisor = 1;
    dut->req_valid = 1;
    tick();
    CHECK_FALSE("ITT no fault", dut->fault);
    CHECK_EQ("ITT PA = VA", dut->pa_out, 0x40800000u);
    dut->req_valid = 0;
    tick();
    return true;
}

static bool test_mmu_disabled_passthrough() {
    reset();
    // TC.E = 0 — walker should never run; PA = VA.
    dut->va_in = 0xDEADBEEF;
    dut->req_valid = 1;
    dut->supervisor = 1;
    tick();
    CHECK_EQ("disabled PA=VA", dut->pa_out, 0xDEADBEEFu);
    CHECK_FALSE("disabled no fault", dut->fault);
    dut->req_valid = 0;
    tick();
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Boot-critical walker widening scenarios
// ════════════════════════════════════════════════════════════════════

static bool test_boot_fixed_040_slices_ignore_legacy_tc_fields() {
    reset();
    write_cr(5, 0x10000);
    write_cr(6, 0x10000);
    // Deliberately odd legacy IS/TIA/TIB values.  The live 040 walker
    // must still use fixed VA[31:25]/[24:18]/[17:12] table slices.
    write_cr(4, mk_tc(true, false, 9, 1, 15));
    build_3lvl_4k(0x8A345000u, 0x0F123000u);
    auto r = xlate(0x8A345678u, 0, 1);
    CHECK_FALSE("fixed-slice walk no fault", r.fault);
    CHECK_EQ("fixed-slice PA", r.pa, 0x0F123678u);
    return true;
}

static bool test_boot_urp_srp_select_distinct_roots() {
    reset();
    write_cr(5, 0x10000);  // URP
    write_cr(6, 0x40000);  // SRP
    write_cr(4, mk_tc(true, false, 0, 7, 7));

    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     0x80001000u, 0x00A00000u);
    build_3lvl_4k_at(0x40000, 0x50000, 0x180000,
                     0x80001000u, 0x00B00000u);

    auto ru = xlate(0x80001044u, 0, /*sup=*/0);
    CHECK_FALSE("URP user no fault", ru.fault);
    CHECK_EQ("URP PA", ru.pa, 0x00A00044u);

    auto rs = xlate(0x80001044u, 0, /*sup=*/1);
    CHECK_FALSE("SRP supervisor no fault", rs.fault);
    CHECK_EQ("SRP PA", rs.pa, 0x00B00044u);
    CHECK_EQ("separate user/sup ATC entries", atc_cnt(), 2);
    return true;
}

static bool test_boot_unaligned_root_pointer_masking() {
    reset();
    write_cr(5, 0x10003);
    write_cr(6, 0x10003);
    write_cr(4, mk_tc(true, false, 0, 7, 7));
    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     0x80001000u, 0x00C00000u);

    auto r = xlate(0x80001AAAu, 0, 1);
    CHECK_FALSE("unaligned root no fault", r.fault);
    CHECK_EQ("unaligned root PA", r.pa, 0x00C00AAAu);
    return true;
}

static bool test_pointer_wp_accumulates_to_atc() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     0x80001000u, 0x00D00000u,
                     /*page_wp=*/false, /*sup_only=*/false,
                     /*cache_inh=*/false, /*leaf_used=*/true,
                     /*modified=*/false, /*l1_wp=*/true);

    auto rr = xlate(0x80001000u, /*wr=*/0, 1);
    CHECK_FALSE("pointer-WP read OK", rr.fault);
    CHECK_EQ("pointer-WP read PA", rr.pa, 0x00D00000u);

    uint64_t ar_before = slv.n_ar;
    auto rw = xlate(0x80001000u, /*wr=*/1, 1);
    uint64_t ar_after = slv.n_ar;
    CHECK_TRUE("pointer-WP write faults", rw.fault);
    CHECK_EQ("pointer-WP fault code", rw.fault_code, 3u);
    CHECK_TRUE("pointer-WP was cached into ATC", ar_after == ar_before);
    return true;
}

static bool test_l2_pointer_wp_cold_write_fault_no_fill() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     0x80001000u, 0x00E00000u,
                     /*page_wp=*/false, /*sup_only=*/false,
                     /*cache_inh=*/false, /*leaf_used=*/true,
                     /*modified=*/false, /*l1_wp=*/false,
                     /*l2_wp=*/true);

    auto r = xlate(0x80001000u, /*wr=*/1, 1);
    CHECK_TRUE("cold write through L2-WP faults", r.fault);
    CHECK_EQ("L2-WP fault code", r.fault_code, 3u);
    CHECK_EQ("faulting walk did not fill ATC", atc_cnt(), 0);
    CHECK_EQ("faulting L2-WP walk did no writes", slv.n_aw, 0);
    return true;
}

static bool test_clean_read_does_not_set_modified() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k(va, 0x00F00000u, /*wp=*/false, /*sup_only=*/false,
                  /*cache_inh=*/false, /*used=*/true, /*modified=*/false);
    auto r = xlate(va, /*wr=*/0, 1);
    CHECK_FALSE("clean read no fault", r.fault);
    CHECK_FALSE("read leaves M clear", (mem_rd(p.leaf_addr) & DESC_M) != 0);
    return true;
}

static bool test_pointer_used_bits_set_after_success() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     va, 0x01000000u,
                     /*page_wp=*/false, /*sup_only=*/false,
                     /*cache_inh=*/false, /*leaf_used=*/true,
                     /*modified=*/false, /*l1_wp=*/false,
                     /*l2_wp=*/false, /*l1_used=*/false,
                     /*l2_used=*/false);

    CHECK_FALSE("L1 U starts clear", (mem_rd(p.l1_addr) & DESC_U) != 0);
    CHECK_FALSE("L2 U starts clear", (mem_rd(p.l2_addr) & DESC_U) != 0);
    auto r = xlate(va, 0, 1);
    CHECK_FALSE("U writeback walk OK", r.fault);
    CHECK_TRUE("L1 U set", (mem_rd(p.l1_addr) & DESC_U) != 0);
    CHECK_TRUE("L2 U set", (mem_rd(p.l2_addr) & DESC_U) != 0);
    CHECK_EQ("two pointer U writebacks", slv.n_aw, 2);
    return true;
}

static bool test_fault_does_not_write_pending_pointer_used() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     va, 0x01100000u,
                     /*page_wp=*/false, /*sup_only=*/false,
                     /*cache_inh=*/false, /*leaf_used=*/true,
                     /*modified=*/false, /*l1_wp=*/false,
                     /*l2_wp=*/false, /*l1_used=*/false,
                     /*l2_used=*/true);
    mem_wr(p.l2_addr, mk_ptr_invalid());

    auto r = xlate(va, 0, 1);
    CHECK_TRUE("L2 invalid faults", r.fault);
    CHECK_EQ("L2 invalid fault code", r.fault_code, 1u);
    CHECK_FALSE("pending L1 U was not written", (mem_rd(p.l1_addr) & DESC_U) != 0);
    CHECK_EQ("faulting walk had no writebacks", slv.n_aw, 0);
    return true;
}

static bool test_unsupported_pointer_descriptor_faults() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k(va, 0x01200000u);
    mem_wr(p.l1_addr, (0x20000u & 0xFFFFFFF0u) | 0x3u);

    auto r = xlate(va, 0, 1);
    CHECK_TRUE("DT=11 pointer faults", r.fault);
    CHECK_EQ("DT=11 pointer fault code", r.fault_code, 1u);
    CHECK_EQ("DT=11 pointer fault addr", r.fault_addr, va);
    return true;
}

static bool test_indirect_leaf_descriptor_faults() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k(va, 0x01300000u);
    mem_wr(p.leaf_addr, mk_ptr(0x30000u, false, true));

    auto r_table4 = xlate(va, 0, 1);
    CHECK_TRUE("leaf DT=10 faults", r_table4.fault);
    CHECK_EQ("leaf DT=10 fault code", r_table4.fault_code, 5u);

    reset();
    cfg_3lvl_4k();
    p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k(va, 0x01300000u);
    mem_wr(p.leaf_addr, (0x30000u & 0xFFFFFFF0u) | 0x3u);
    auto r_table8 = xlate(va, 0, 1);
    CHECK_TRUE("leaf DT=11 faults", r_table8.fault);
    CHECK_EQ("leaf DT=11 fault code", r_table8.fault_code, 5u);
    return true;
}

static bool test_l1_axi_error_faults_invalid_pointer() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k(va, 0x01400000u);
    mem_set_rresp(p.l1_addr, 2);

    auto r = xlate(va, 0, 1);
    CHECK_TRUE("L1 AXI error faults", r.fault);
    CHECK_EQ("L1 AXI error fault code", r.fault_code, 1u);
    CHECK_EQ("L1 AXI error fault addr", r.fault_addr, va);
    return true;
}

static bool test_leaf_axi_error_faults_invalid_page() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k(va, 0x01500000u);
    mem_set_rresp(p.leaf_addr, 2);

    auto r = xlate(va, 0, 1);
    CHECK_TRUE("leaf AXI error faults", r.fault);
    CHECK_EQ("leaf AXI error fault code", r.fault_code, 2u);
    CHECK_EQ("leaf AXI error fault addr", r.fault_addr, va);
    CHECK_EQ("leaf AXI error had no writebacks", slv.n_aw, 0);
    return true;
}

static bool test_pointer_writeback_axi_error_faults() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     va, 0x01600000u,
                     /*page_wp=*/false, /*sup_only=*/false,
                     /*cache_inh=*/false, /*leaf_used=*/true,
                     /*modified=*/true, /*l1_wp=*/false,
                     /*l2_wp=*/false, /*l1_used=*/false,
                     /*l2_used=*/true);
    mem_set_bresp(p.l1_addr, 2);

    auto r = xlate(va, 0, 1);
    CHECK_TRUE("pointer writeback error faults", r.fault);
    CHECK_EQ("pointer writeback fault code", r.fault_code, 1u);
    CHECK_EQ("pointer writeback fault addr", r.fault_addr, va);
    CHECK_EQ("faulting pointer writeback did not fill ATC", atc_cnt(), 0);
    CHECK_FALSE("failed pointer U write did not update memory",
                (mem_rd(p.l1_addr) & DESC_U) != 0);
    return true;
}

static bool test_leaf_writeback_axi_error_faults() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    build_3lvl_4k_at(0x10000, 0x20000, 0x100000,
                     va, 0x01700000u,
                     /*page_wp=*/false, /*sup_only=*/false,
                     /*cache_inh=*/false, /*leaf_used=*/true,
                     /*modified=*/false, /*l1_wp=*/false,
                     /*l2_wp=*/false, /*l1_used=*/true,
                     /*l2_used=*/true);
    mem_set_bresp(p.leaf_addr, 2);

    auto r = xlate(va, 1, 1);
    CHECK_TRUE("leaf writeback error faults", r.fault);
    CHECK_EQ("leaf writeback fault code", r.fault_code, 2u);
    CHECK_EQ("leaf writeback fault addr", r.fault_addr, va);
    CHECK_EQ("faulting leaf writeback did not fill ATC", atc_cnt(), 0);
    CHECK_FALSE("failed leaf M write did not update memory",
                (mem_rd(p.leaf_addr) & DESC_M) != 0);
    return true;
}

static bool test_sup_fill_does_not_authorize_user() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    build_3lvl_4k(va, 0x01600000u, /*wp=*/false, /*sup_only=*/true);

    auto rs = xlate(va, 0, /*sup=*/1);
    CHECK_FALSE("supervisor fills sup-only page", rs.fault);
    uint64_t ar_before = slv.n_ar;
    auto ru = xlate(va, 0, /*sup=*/0);
    uint64_t ar_after = slv.n_ar;
    CHECK_TRUE("user still faults", ru.fault);
    CHECK_EQ("user sup-only fault code", ru.fault_code, 4u);
    CHECK_TRUE("user did not hit supervisor ATC entry", ar_after > ar_before);
    return true;
}

static bool test_dtt_bypass_invalid_tables_no_walk() {
    reset();
    write_cr(4, mk_tc(true, false, 0, 7, 7));
    write_cr(2, mk_ttr(0x50, 0x00, 0x3, /*wp=*/false));

    auto r = xlate(0x50001234u, /*wr=*/0, /*sup=*/1);
    CHECK_FALSE("DTT bypass no fault", r.fault);
    CHECK_EQ("DTT bypass PA=VA", r.pa, 0x50001234u);
    CHECK_EQ("DTT bypass no walker AR", slv.n_ar, 0);
    CHECK_EQ("DTT bypass no ATC fill", atc_cnt(), 0);
    return true;
}

static bool test_dtt_write_protect_fault_no_walk() {
    reset();
    write_cr(4, mk_tc(true, false, 0, 7, 7));
    write_cr(2, mk_ttr(0x50, 0x00, 0x3, /*wp=*/true));

    auto r = xlate(0x50001234u, /*wr=*/1, /*sup=*/1);
    CHECK_TRUE("DTT WP faults", r.fault);
    CHECK_EQ("DTT WP fault code", r.fault_code, 7u);
    CHECK_EQ("DTT WP fault addr", r.fault_addr, 0x50001234u);
    CHECK_EQ("DTT WP no walker AR", slv.n_ar, 0);
    return true;
}

static bool test_req_va_write_latched_while_busy() {
    reset();
    cfg_3lvl_4k();
    slv.extra_r_delay = 4;
    build_3lvl_4k(0x80001000u, 0x01700000u);
    build_3lvl_4k(0x80002000u, 0x01800000u);

    dut->va_in = 0x80001000u;
    dut->is_write = 0;
    dut->supervisor = 1;
    dut->req_valid = 1;
    tick();
    CHECK_FALSE("first walk is busy", dut->resp_ready);

    dut->va_in = 0x80002000u;
    dut->is_write = 1;
    bool done = false;
    for (int i = 0; i < 200; i++) {
        tick();
        if (dut->resp_ready) {
            done = true;
            break;
        }
    }
    CHECK_TRUE("latched request completed", done);
    CHECK_FALSE("latched request no fault", dut->fault);
    CHECK_EQ("latched request PA is first VA", dut->pa_out, 0x01700000u);

    uint64_t ar_at_done = slv.n_ar;
    for (int i = 0; i < 8; i++) tick();
    CHECK_TRUE("held mutated VA/write did not start second walk",
               slv.n_ar == ar_at_done);

    dut->req_valid = 0;
    dut->is_write = 0;
    dut->supervisor = 1;
    tick();
    slv.extra_r_delay = 0;
    auto rb = xlate(0x80002000u, 0, 1);
    CHECK_FALSE("second request after drop OK", rb.fault);
    CHECK_EQ("second request PA", rb.pa, 0x01800000u);
    return true;
}

static bool test_axi_write_address_before_data() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    slv.w_stall_ctr = 6;
    build_3lvl_4k(va, 0x01900000u, /*wp=*/false, /*sup_only=*/false,
                  /*cache_inh=*/false, /*used=*/false, /*modified=*/false);

    auto r = xlate(va, 0, 1, /*timeout=*/800);
    CHECK_FALSE("AW-before-W writeback OK", r.fault);
    CHECK_TRUE("leaf U set with delayed W", (mem_rd(p.leaf_addr) & DESC_U) != 0);
    return true;
}

static bool test_axi_write_data_before_address() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    Walk4kPath p = path_4k(0x10000, 0x20000, 0x100000, va);
    slv.aw_stall_ctr = 6;
    build_3lvl_4k(va, 0x01A00000u, /*wp=*/false, /*sup_only=*/false,
                  /*cache_inh=*/false, /*used=*/false, /*modified=*/false);

    auto r = xlate(va, 0, 1, /*timeout=*/800);
    CHECK_FALSE("W-before-AW writeback OK", r.fault);
    CHECK_TRUE("leaf U set with delayed AW", (mem_rd(p.leaf_addr) & DESC_U) != 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Stress / adversarial scenarios (task #100)
// ════════════════════════════════════════════════════════════════════

// ── Category 1: ATC thrashing ───────────────────────────────────────

// Helper: build a fresh 3-lvl 4K mapping and translate it; return pa.
static bool xlate_ok(uint32_t va, uint32_t exp_pa, bool sup = true) {
    auto r = xlate(va, 0, sup);
    if (r.fault) { printf("    unexpected fault va=0x%08x fc=%u\n",
                          va, r.fault_code); return false; }
    if (r.pa != exp_pa) { printf("    PA mismatch va=0x%08x got 0x%08x exp 0x%08x\n",
                                 va, r.pa, exp_pa); return false; }
    return true;
}

// Force a PFLUSHA from the tb (1-cycle pulse).
static void pulse_pflusha() {
    dut->pflush_all_req = 1; tick();
    dut->pflush_all_req = 0; tick();
}

// Read atc_valid_count combinationally.
static uint32_t atc_cnt() { dut->eval(); return dut->atc_valid_count; }

static bool test_atc_thrash_128_pages() {
    reset();
    cfg_3lvl_4k();
    // Install 128 distinct mappings at 4KB stride.
    const int N = 128;
    const uint32_t BASE = 0x80000000u;
    for (int i = 0; i < N; i++) {
        build_3lvl_4k(BASE + i * 0x1000u,
                      0xC0000000u + i * 0x1000u);
    }
    // Round 1: all misses.
    for (int i = 0; i < N; i++) {
        if (!xlate_ok(BASE + i * 0x1000u + 0x100u,
                       0xC0000000u + i * 0x1000u + 0x100u)) return false;
    }
    uint32_t vcnt1 = atc_cnt();
    CHECK_TRUE("ATC saturates at 64", vcnt1 == 64);
    // Round 2: re-access every VA.  Only ones still resident hit.
    uint64_t ar_before = slv.n_ar;
    for (int i = 0; i < N; i++) {
        if (!xlate_ok(BASE + i * 0x1000u,
                       0xC0000000u + i * 0x1000u)) return false;
    }
    uint64_t ar_after = slv.n_ar;
    uint64_t ar_in_round2 = ar_after - ar_before;
    CHECK_TRUE("round-2 walker still did substantial work", ar_in_round2 >= 64);
    return true;
}

static bool test_atc_lru_pressure() {
    reset();
    cfg_3lvl_4k();
    // 26 entries across different sets.  'A' maps to set 0, 'B' set 1...
    // Use VA[15:12] as the set index so each letter picks a unique set.
    for (int i = 0; i < 26; i++) {
        uint32_t va = 0x80000000u | (uint32_t)(i << 12);
        build_3lvl_4k(va, 0xD0000000u | (uint32_t)(i << 12));
    }
    // Walk A..Z.
    for (int i = 0; i < 26; i++) {
        uint32_t va = 0x80000000u | (uint32_t)(i << 12);
        if (!xlate_ok(va, 0xD0000000u | (uint32_t)(i << 12))) return false;
    }
    // Poison backing memory for A.
    build_3lvl_4k(0x80000000u, 0x0); // doesn't really matter — we just
    // clobber the leaf; but the ATC should still hit for A.
    uint64_t ar_before = slv.n_ar;
    auto r = xlate(0x80000000u, 0, 1);
    uint64_t ar_after = slv.n_ar;
    CHECK_FALSE("A still hits", r.fault);
    CHECK_TRUE("A was an ATC hit (no walker AR)", ar_after == ar_before);
    return true;
}

static bool test_atc_every_set_touched() {
    reset();
    cfg_3lvl_4k();
    // 16 distinct VAs each landing in a unique set.
    for (int s = 0; s < 16; s++) {
        uint32_t va = 0x80000000u | (uint32_t)(s << 12);
        build_3lvl_4k(va, 0xE0000000u | (uint32_t)(s << 12));
        if (!xlate_ok(va, 0xE0000000u | (uint32_t)(s << 12))) return false;
    }
    CHECK_EQ("16 entries filled", atc_cnt(), 16);
    return true;
}

static bool test_atc_4way_conflict_miss() {
    reset();
    cfg_3lvl_4k();
    // Pick 5 VAs all mapping to set 0 (VA[15:12] = 0) but with differing
    // upper bits so the tag differs per way.
    uint32_t va[5] = {
        0x00000100u,   // va[15:12] = 0 (set 0)
        0x00010100u,
        0x00020100u,
        0x00030100u,
        0x00040100u,
    };
    uint32_t pa[5] = {
        0x10000000u, 0x20000000u, 0x30000000u, 0x40000000u, 0x50000000u,
    };
    for (int i = 0; i < 5; i++) build_3lvl_4k(va[i] & ~0xFFFu, pa[i]);
    // Fill first 4 — every way of set 0 ends up valid.
    // VA low 12 bits are the page offset; PA low 12 bits come from the
    // same offset.  We translate VA + 0x100, so expect PA + 0x100.
    for (int i = 0; i < 4; i++) {
        if (!xlate_ok(va[i], pa[i] | 0x100u)) return false;
    }
    // The 5th must evict SOMETHING (tree-PLRU victim).
    if (!xlate_ok(va[4], pa[4] | 0x100u)) return false;
    CHECK_EQ("still 4 in set 0", atc_cnt(), 4);
    // Now poison ALL five backing pages so any re-walk would fail.
    for (int i = 0; i < 5; i++) {
        int tia = 7, tib = 7, pg_bits = 12;
        uint32_t v = va[i] & ~0xFFFu;
        uint32_t l1_idx = (v >> (32 - tia)) & 0x7F;
        uint32_t l2_idx = ((v << tia) >> (32 - tib)) & 0x7F;
        uint32_t leaf_bits = 32 - tia - tib - pg_bits;
        uint32_t leaf_idx = (v >> pg_bits) & ((1u << leaf_bits) - 1);
        uint32_t l3 = 0x100000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);
        mem_wr(l3 + leaf_idx * 4, mk_page_invalid());
    }
    // Re-walk all five: 4 should hit (still-resident) and 1 should fault.
    int n_fault = 0;
    for (int i = 0; i < 5; i++) {
        auto r = xlate(va[i], 0, 1);
        if (r.fault) n_fault++;
    }
    CHECK_EQ("exactly one was evicted ⇒ one faults", n_fault, 1);
    return true;
}

// ── Category 2: walker throughput ───────────────────────────────────

static bool test_back_to_back_8_walks() {
    reset();
    cfg_3lvl_4k();
    const int N = 8;
    for (int i = 0; i < N; i++)
        build_3lvl_4k(0x80000000u + (uint32_t)(i << 12),
                      0xB0000000u + (uint32_t)(i << 12));
    uint64_t t0 = sim_time;
    for (int i = 0; i < N; i++) {
        auto r = xlate(0x80000000u + (uint32_t)(i << 12), 0, 1);
        CHECK_FALSE("no fault", r.fault);
    }
    uint64_t dt = sim_time - t0;
    // Each 3-lvl 4K walk with single-cycle slave ≈ 9-14 cycles plus
    // 1-2 cycles dispatch overhead; N=8 walks ≲ 200 cyc is plenty.
    CHECK_TRUE("throughput sane", dt < (uint64_t)(N * 30));
    return true;
}

static bool test_walker_single_in_flight() {
    reset();
    cfg_3lvl_4k();
    // Slow the slave so the walker takes longer, giving us a big window.
    slv.extra_r_delay = 3;
    build_3lvl_4k(0x80001000u, 0xA0000000u);
    build_3lvl_4k(0x80002000u, 0xB0000000u);
    // Kick a walk; sample resp_ready while it's running.
    dut->va_in = 0x80001000u;
    dut->is_write = 0;
    dut->supervisor = 1;
    dut->req_valid = 1;
    // Wait one cycle for the request to latch.
    tick();
    // Now try to queue a second request with a different VA — even if
    // we change va_in while req_valid stays high, the walker must finish
    // the first one (w_va was latched) and only then consider the new va.
    bool saw_busy = false;
    for (int i = 0; i < 40; i++) {
        if (!dut->resp_ready) saw_busy = true;
        if (dut->resp_ready) {
            // First walk done; req is satisfied.
            break;
        }
        tick();
    }
    CHECK_TRUE("walker was busy at some point", saw_busy);
    CHECK_EQ("first PA correct", dut->pa_out, 0xA0000000u);
    dut->req_valid = 0;
    tick();
    slv.extra_r_delay = 0;
    return true;
}

static bool test_mixed_2lvl_3lvl_walks() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    if (!xlate_ok(0x80001000u, 0xAA000000u)) return false;
    // Flip to 2-level and do another walk.
    pulse_pflusha();
    cfg_2lvl_4k();
    build_2lvl_4k(0x80002000u, 0xBB000000u);
    if (!xlate_ok(0x80002000u, 0xBB000000u)) return false;
    // Back to 3-level.
    pulse_pflusha();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80003000u, 0xCC000000u);
    if (!xlate_ok(0x80003000u, 0xCC000000u)) return false;
    // And 2-level again.
    pulse_pflusha();
    cfg_2lvl_4k();
    build_2lvl_4k(0x80004000u, 0xDD000000u);
    if (!xlate_ok(0x80004000u, 0xDD000000u)) return false;
    return true;
}

// ── Category 3: PFLUSH under load ───────────────────────────────────

static bool test_pflush_va_during_walk() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xA0000000u);
    build_3lvl_4k(0x80002000u, 0xB0000000u);
    // Slow the slave so the walk is multi-cycle and we have a window.
    slv.extra_r_delay = 2;
    // Kick walk for A.
    dut->va_in = 0x80001000u;
    dut->is_write = 0;
    dut->supervisor = 1;
    dut->req_valid = 1;
    tick(); tick(); tick();   // a few cycles into the walk
    // Fire PFLUSH-by-VA on B — must NOT disturb A's walk.
    dut->pflush_va_req = 1;
    dut->pflush_addr   = 0x80002000u;
    dut->pflush_sup    = 1;
    tick();
    dut->pflush_va_req = 0;
    // Drain.
    for (int i = 0; i < 80 && !dut->resp_ready; i++) tick();
    CHECK_TRUE("A walk done", dut->resp_ready);
    CHECK_FALSE("A no fault", dut->fault);
    CHECK_EQ("A PA", dut->pa_out, 0xA0000000u);
    dut->req_valid = 0;
    tick();
    slv.extra_r_delay = 0;
    return true;
}

static bool test_pflush_va_matches_inflight() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xA0000000u);
    slv.extra_r_delay = 2;
    // Kick walk for A.
    dut->va_in = 0x80001000u;
    dut->is_write = 0;
    dut->supervisor = 1;
    dut->req_valid = 1;
    tick(); tick(); tick();
    // Fire PFLUSH-by-VA on A itself.
    dut->pflush_va_req = 1;
    dut->pflush_addr   = 0x80001000u;
    dut->pflush_sup    = 1;
    tick();
    dut->pflush_va_req = 0;
    // Drain walker.
    for (int i = 0; i < 80 && !dut->resp_ready; i++) tick();
    CHECK_TRUE("walk finished", dut->resp_ready);
    dut->req_valid = 0;
    tick();
    slv.extra_r_delay = 0;
    // Follow-up translation: the entry MAY or may not be resident
    // depending on relative timing of PFLUSH-cycle vs fill-cycle.
    // Either way, translation must still produce the correct PA.
    auto r = xlate(0x80001000u, 0, 1);
    CHECK_FALSE("follow-up no fault", r.fault);
    CHECK_EQ("follow-up PA", r.pa, 0xA0000000u);
    return true;
}

static bool test_pflush_va_matches_inflight_any_sup_tag() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xA0000000u);
    slv.extra_r_delay = 2;

    dut->va_in = 0x80001000u;
    dut->is_write = 0;
    dut->supervisor = 1;
    dut->req_valid = 1;
    tick(); tick(); tick();

    dut->pflush_va_req = 1;
    dut->pflush_addr   = 0x80001000u;
    dut->pflush_sup    = 0;
    tick();
    dut->pflush_va_req = 0;
    dut->req_valid = 0;

    for (int i = 0; i < 80 && !dut->resp_ready; i++) tick();
    CHECK_TRUE("walk finished", dut->resp_ready);
    tick();
    slv.extra_r_delay = 0;
    CHECK_EQ("mismatched-sup PFLUSH discarded inflight fill", atc_cnt(), 0);

    uint64_t ar_before = slv.n_ar;
    auto r = xlate(0x80001000u, 0, 1);
    uint64_t ar_after = slv.n_ar;
    CHECK_FALSE("follow-up no fault", r.fault);
    CHECK_EQ("follow-up PA", r.pa, 0xA0000000u);
    CHECK_TRUE("follow-up re-walked after mismatched-sup PFLUSH",
               ar_after > ar_before);
    return true;
}

static bool test_pflush_asid_mixed_entries() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xA0000000u);
    build_3lvl_4k(0x80002000u, 0xB0000000u);
    // User VAs (sup=0) — install user-mode entries.
    (void)xlate(0x80001000u, 0, 1);   // sup entry
    (void)xlate(0x80002000u, 0, 1);   // sup entry
    (void)xlate(0x80001000u, 0, 0);   // user entry
    (void)xlate(0x80002000u, 0, 0);   // user entry
    uint32_t pre = atc_cnt();
    CHECK_TRUE("at least 4 entries resident", pre >= 4);
    // PFLUSH-by-ASID (stub = PFLUSHA).
    dut->pflush_asid_req = 1; tick();
    dut->pflush_asid_req = 0; tick();
    CHECK_EQ("post-pflush-asid: empty", atc_cnt(), 0);
    return true;
}

static bool test_pflusha_midwalk() {
    reset();
    cfg_3lvl_4k();
    uint32_t va = 0x80001000u;
    uint32_t old_l1 = 0x10000u;
    uint32_t new_l1 = 0x40000u;
    build_3lvl_4k_at(old_l1, 0x20000u, 0x100000u,
                     va, 0xA0000000u);
    build_3lvl_4k_at(new_l1, 0x50000u, 0x60000u,
                     va, 0xB0000000u);

    slv.extra_r_delay = 6;
    dut->va_in = va;
    dut->supervisor = 1;
    dut->req_valid = 1;
    tick(); tick();

    // Switch SRP and fire PFLUSHA while the old-root walker is
    // mid-flight.  The stale old result must not fill the ATC or drive
    // last_pa; the level-held request should restart under the new root.
    write_cr(6, new_l1);
    pulse_pflusha();

    for (int i = 0; i < 400 && !dut->resp_ready; i++) tick();
    CHECK_TRUE("held request rewalks post-pflusha", dut->resp_ready);
    CHECK_FALSE("no fault", dut->fault);
    CHECK_EQ("new-root PA", dut->pa_out, 0xB0000000u);
    dut->req_valid = 0;
    tick();
    CHECK_EQ("one current entry post-pflusha+rewalk", atc_cnt(), 1);

    uint64_t ar_before = slv.n_ar;
    auto r = xlate(va + 0x40u, 0, 1);
    CHECK_FALSE("follow-up no fault", r.fault);
    CHECK_EQ("follow-up uses new-root ATC", r.pa, 0xB0000040u);
    CHECK_TRUE("follow-up hits ATC", slv.n_ar == ar_before);
    slv.extra_r_delay = 0;
    return true;
}

// ── Category 4: page-boundary straddling ───────────────────────────

static bool test_cross_page_long_split() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    build_3lvl_4k(0x80002000u, 0xBB000000u);
    // Last 2 bytes of page A.
    if (!xlate_ok(0x80001FFEu, 0xAA000FFEu)) return false;
    // First 2 bytes of page B.
    if (!xlate_ok(0x80002000u, 0xBB000000u)) return false;
    return true;
}

static bool test_movem_like_burst_across_page() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    build_3lvl_4k(0x80002000u, 0xBB000000u);
    // 10 translations: 5 in page A + 5 in page B.
    uint64_t ar_before = slv.n_ar;
    for (uint32_t off = 0x0; off < 0x14u; off += 0x4u) {
        uint32_t va = 0x80001FF0u + off;
        uint32_t pa_base = (va < 0x80002000u) ? 0xAA000000u : 0xBB000000u;
        uint32_t pa = pa_base | (va & 0xFFFu);
        if (!xlate_ok(va, pa)) return false;
    }
    uint64_t ar_after = slv.n_ar;
    // Two walks total (one per page); each 3-lvl walk does 3 ARs.
    // Budget: 6-9 ARs.  Assert <= 12 to leave headroom.
    CHECK_TRUE("at most 2 walks", (ar_after - ar_before) <= 12);
    return true;
}

static bool test_ifetch_cross_page_simulated() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    build_3lvl_4k(0x80002000u, 0xBB000000u);
    // Explicit is_instruction=1 to simulate fetch-side.
    dut->is_instruction = 1;
    auto r1 = xlate(0x80001FFCu, 0, 1);
    dut->is_instruction = 1;
    auto r2 = xlate(0x80002000u, 0, 1);
    dut->is_instruction = 0;
    CHECK_FALSE("ifetch A OK", r1.fault);
    CHECK_FALSE("ifetch B OK", r2.fault);
    CHECK_EQ("ifetch PA A", r1.pa, 0xAA000FFCu);
    CHECK_EQ("ifetch PA B", r2.pa, 0xBB000000u);
    return true;
}

// ── Category 5: page-size switching ────────────────────────────────

static bool test_pagesize_switch_4k_to_8k() {
    reset();
    cfg_3lvl_4k();
    // Choose a VA where switching vpn_lsb 12->13 lands in a different set.
    // VA[15:12]=0x5 ⇒ set 5 under 4K.  Under 8K, set index = VA[16:13]
    // = 0x2 ⇒ set 2.  Different set ⇒ cannot alias.
    uint32_t va = 0x80005000u;
    build_3lvl_4k(va, 0xAA000000u);
    if (!xlate_ok(va, 0xAA000000u)) return false;
    uint32_t cnt4k = atc_cnt();
    CHECK_EQ("1 resident 4K", cnt4k, 1);
    // Switch page size to 8K (don't pflush).  The probe path uses the
    // LIVE page_size_8k so the old entry's geometry is re-interpreted.
    // Rebuild an 8K mapping for a DIFFERENT VA (8K tables live at the
    // same L1 root but with different layout).
    write_cr(4, mk_tc(true, true, 0, 6, 6));
    // Rebuild 8K table tree.
    build_3lvl_8k(0x80006000u, 0xBB000000u);
    auto r = xlate(0x80006000u, 0, 1);
    CHECK_FALSE("8K walk after 4K fill OK", r.fault);
    CHECK_EQ("8K PA", r.pa, 0xBB000000u);
    return true;
}

static bool test_pagesize_consecutive() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    if (!xlate_ok(0x80001F00u, 0xAA000F00u)) return false;
    pulse_pflusha();
    cfg_3lvl_8k();
    build_3lvl_8k(0x80002000u, 0xCC000000u);
    auto r = xlate(0x80002F00u, 0, 1);
    CHECK_FALSE("8K walk", r.fault);
    // For 8K: PA low 13 bits come from VA.  VA=0x80002F00, low 13 = 0x0F00,
    // plus PFN 0xCC000000 >> 13 << 13 = 0xCC000000 (13 LSBs are 0).
    CHECK_EQ("8K PA", r.pa, 0xCC000F00u);
    return true;
}

static bool test_pagesize_switch_pflushall() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    if (!xlate_ok(0x80001000u, 0xAA000000u)) return false;
    // PFLUSHA, switch to 8K, walk a new VA.
    pulse_pflusha();
    CHECK_EQ("ATC empty after pflusha", atc_cnt(), 0);
    cfg_3lvl_8k();
    build_3lvl_8k(0x80002000u, 0xCC000000u);
    auto r = xlate(0x80002000u, 0, 1);
    CHECK_FALSE("8K walk post-switch OK", r.fault);
    CHECK_EQ("one entry resident", atc_cnt(), 1);
    return true;
}

// ── Category 6: root-pointer swap ──────────────────────────────────

static bool test_srp_swap_reflects() {
    reset();
    cfg_3lvl_4k();
    // First SRP (at 0x10000) → mapping A.
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    if (!xlate_ok(0x80001000u, 0xAA000000u, /*sup=*/1)) return false;
    // Rewrite SRP to point at a DIFFERENT L1 table we build by hand.
    // Put a new L1 table at 0x40000 with identical VA→different-PA mapping.
    int tia = 7, tib = 7, pg_bits = 12;
    uint32_t va = 0x80001000u;
    uint32_t l1_idx = (va >> (32 - tia)) & 0x7F;
    uint32_t l2_idx = ((va << tia) >> (32 - tib)) & 0x7F;
    uint32_t leaf_bits = 32 - tia - tib - pg_bits;
    uint32_t leaf_idx = (va >> pg_bits) & ((1u << leaf_bits) - 1);
    uint32_t new_l1 = 0x40000, new_l2 = 0x50000;
    uint32_t new_l3 = 0x60000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);
    mem_wr(new_l1 + l1_idx * 4, mk_ptr(new_l2, false, true));
    mem_wr(new_l2 + l2_idx * 4, mk_ptr(new_l3, false, true));
    mem_wr(new_l3 + leaf_idx * 4,
           mk_page(0xDD000000u, pg_bits, false, false, false, true, false));
    write_cr(6, new_l1);   // SRP rewrite.
    // Note: 68040 does not auto-invalidate on SRP write.  Caller must PFLUSH.
    pulse_pflusha();
    auto r = xlate(va, 0, 1);
    CHECK_FALSE("new SRP walk OK", r.fault);
    CHECK_EQ("new SRP PA", r.pa, 0xDD000000u);
    return true;
}

static bool test_urp_change_no_inval() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    // Populate a user entry.  (URP = SRP for our tests; install entry as user.)
    if (!xlate_ok(0x80001000u, 0xAA000000u, /*sup=*/0)) return false;
    // Change URP WITHOUT PFLUSHA.  The existing ATC entry survives.
    write_cr(5, 0x70000);   // new URP, points at unpopulated memory
    // User re-request — ATC hit on the stale entry.  This documents
    // the "PFLUSH-after-URP" software contract.
    uint64_t ar_before = slv.n_ar;
    auto r = xlate(0x80001000u, 0, /*sup=*/0);
    uint64_t ar_after = slv.n_ar;
    CHECK_FALSE("ATC hit after URP swap (no walker run)", r.fault);
    CHECK_TRUE("hit → no AR", ar_after == ar_before);
    CHECK_EQ("stale PA", r.pa, 0xAA000000u);
    return true;
}

static bool test_srp_no_affect_user() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    // Populate both user + sup entries.
    (void)xlate(0x80001000u, 0, /*sup=*/0);
    (void)xlate(0x80001000u, 0, /*sup=*/1);
    // Rewrite SRP (user unaffected by SRP).
    write_cr(6, 0x70000);
    // User re-request: still hits the user ATC entry.
    uint64_t ar_before = slv.n_ar;
    auto r = xlate(0x80001000u, 0, /*sup=*/0);
    uint64_t ar_after = slv.n_ar;
    CHECK_FALSE("user unaffected", r.fault);
    CHECK_TRUE("user was hit", ar_after == ar_before);
    return true;
}

// ── Category 7: FC crossings ───────────────────────────────────────

static bool test_sup_only_user_access() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u, false, /*sup_only=*/true);
    auto r = xlate(0x80001000u, 0, /*sup=*/0);
    CHECK_TRUE("user fault on sup-only", r.fault);
    CHECK_EQ("fc SUP", r.fault_code, 4u);
    return true;
}

static bool test_sup_only_sup_access() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u, false, /*sup_only=*/true);
    auto r = xlate(0x80001000u, 0, /*sup=*/1);
    CHECK_FALSE("sup OK", r.fault);
    CHECK_EQ("PA", r.pa, 0xAA000000u);
    return true;
}

static bool test_sup_access_user_page() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u, false, /*sup_only=*/false);
    auto r = xlate(0x80001000u, 0, /*sup=*/1);
    CHECK_FALSE("sup can access user page", r.fault);
    return true;
}

static bool test_toggle_fc_same_va() {
    reset();
    cfg_3lvl_4k();
    // Build ONE mapping shared by both modes.
    build_3lvl_4k(0x80001000u, 0xAA000000u, false, /*sup_only=*/false);
    // First access as user — installs a user ATC entry.
    if (!xlate_ok(0x80001000u, 0xAA000000u, /*sup=*/0)) return false;
    // Access as supervisor — separate ATC entry even though VA matches.
    uint64_t ar_before = slv.n_ar;
    auto rs = xlate(0x80001000u, 0, /*sup=*/1);
    uint64_t ar_after = slv.n_ar;
    CHECK_FALSE("sup walk OK", rs.fault);
    CHECK_TRUE("sup caused a walker run (sup_tag differs)", ar_after > ar_before);
    CHECK_EQ("2 distinct ATC entries", atc_cnt(), 2);
    // Re-access as user — should hit the user entry.
    uint64_t ar2 = slv.n_ar;
    auto ru = xlate(0x80001000u, 0, /*sup=*/0);
    uint64_t ar3 = slv.n_ar;
    CHECK_FALSE("user re-access OK", ru.fault);
    CHECK_TRUE("user is a hit (no AR)", ar3 == ar2);
    return true;
}

// ── Category 8: exception-during-walk ──────────────────────────────

static bool test_fault_then_recover() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    // Clobber L1 to force invalid-pointer fault.
    uint32_t l1_idx = (0x80001000u >> 25) & 0x7F;
    mem_wr(0x10000 + l1_idx * 4, mk_ptr_invalid());
    auto r1 = xlate(0x80001000u, 0, 1);
    CHECK_TRUE("1st: fault", r1.fault);
    CHECK_EQ("fc invalid-ptr", r1.fault_code, 1u);
    // Fix L1 and re-request.
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    auto r2 = xlate(0x80001000u, 0, 1);
    CHECK_FALSE("recover", r2.fault);
    CHECK_EQ("recovered PA", r2.pa, 0xAA000000u);
    return true;
}

static bool test_fault_then_different_va() {
    reset();
    cfg_3lvl_4k();
    // Pick A and B in DIFFERENT L1 slots (VA[31:25] differs).
    const uint32_t A = 0x80001000u;   // L1 slot 0x40
    const uint32_t B = 0xC0001000u;   // L1 slot 0x60
    build_3lvl_4k(A, 0xAA000000u);
    build_3lvl_4k(B, 0xBB000000u);
    // Clobber A's L1 entry only.
    uint32_t l1_idx = (A >> 25) & 0x7F;
    mem_wr(0x10000 + l1_idx * 4, mk_ptr_invalid());
    auto ra = xlate(A, 0, 1);
    CHECK_TRUE("A: fault", ra.fault);
    // B must walk cleanly — no stale state.
    auto rb = xlate(B, 0, 1);
    CHECK_FALSE("B: no fault after A's fault", rb.fault);
    CHECK_EQ("B: PA correct", rb.pa, 0xBB000000u);
    return true;
}

static bool test_wp_fault_no_write_leak() {
    reset();
    cfg_3lvl_4k();
    build_3lvl_4k(0x80001000u, 0xAA000000u, /*wp=*/true,
                  /*sup_only=*/false, /*cache_inh=*/false,
                  /*used=*/true, /*modified=*/false);
    int tia = 7, tib = 7, pg_bits = 12;
    uint32_t l1_idx = (0x80001000u >> (32 - tia)) & 0x7F;
    uint32_t l2_idx = ((0x80001000u << tia) >> (32 - tib)) & 0x7F;
    uint32_t leaf_bits = 32 - tia - tib - pg_bits;
    uint32_t leaf_idx = (0x80001000u >> pg_bits) & ((1u << leaf_bits) - 1);
    uint32_t l3 = 0x100000 + (l1_idx * 0x10000) + (l2_idx * 0x1000);
    uint32_t before = mem_rd(l3 + leaf_idx * 4);
    uint64_t aw_before = slv.n_aw;
    auto r = xlate(0x80001000u, /*wr=*/1, 1);
    CHECK_TRUE("WP fault", r.fault);
    CHECK_EQ("fc WP", r.fault_code, 3u);
    uint32_t after = mem_rd(l3 + leaf_idx * 4);
    CHECK_EQ("leaf descriptor unchanged", after, before);
    // The walker may still have written back a U=1 update, but M must NOT flip.
    CHECK_FALSE("M never set on WP write", (after & DESC_M) != 0);
    // (AW count may or may not have grown due to U-bit writeback
    // happening on earlier read; don't assert on it.)
    (void)aw_before;
    return true;
}

// ── Category 10: contention / slow-slave ───────────────────────────

static bool test_slow_axi_slave() {
    reset();
    cfg_3lvl_4k();
    slv.extra_r_delay = 10;
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    uint64_t t0 = sim_time;
    auto r = xlate(0x80001000u, 0, 1, /*timeout=*/800);
    uint64_t dt = sim_time - t0;
    CHECK_FALSE("slow walk OK", r.fault);
    CHECK_EQ("PA", r.pa, 0xAA000000u);
    // 3 R-beats * (1 base + 10 extra) + overhead ~ 40-60 cycles.
    CHECK_TRUE("slow latency observed", dt > 20);
    slv.extra_r_delay = 0;
    return true;
}

static bool test_axi_backpressure_ar() {
    reset();
    cfg_3lvl_4k();
    slv.ar_stall = 5;
    build_3lvl_4k(0x80001000u, 0xAA000000u);
    auto r = xlate(0x80001000u, 0, 1, /*timeout=*/800);
    CHECK_FALSE("backpressured AR: walk OK", r.fault);
    CHECK_EQ("PA correct", r.pa, 0xAA000000u);
    slv.ar_stall = 0;
    return true;
}

static bool test_axi_backpressure_b() {
    reset();
    cfg_3lvl_4k();
    // Need U=0 so walker does a writeback; stress B back-pressure.
    slv.b_stall = 15;
    build_3lvl_4k(0x80001000u, 0xAA000000u, /*wp=*/false,
                  /*sup_only=*/false, /*cache_inh=*/false,
                  /*used=*/false, /*modified=*/false);
    auto r = xlate(0x80001000u, 0, 1, /*timeout=*/1200);
    CHECK_FALSE("backpressured B: walk OK", r.fault);
    CHECK_EQ("PA correct", r.pa, 0xAA000000u);
    slv.b_stall = 0;
    return true;
}

// ════════════════════════════════════════════════════════════════════

#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

static bool has_arg(int argc, char** argv, const char* arg) {
    for (int i = 1; i < argc; i++)
        if (std::strcmp(argv[i], arg) == 0) return true;
    return false;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vmmu;
    bool boot_suite_only = has_arg(argc, argv, "+suite=boot");

    if (boot_suite_only) {
        RUN(test_boot_fixed_040_slices_ignore_legacy_tc_fields);
        RUN(test_boot_urp_srp_select_distinct_roots);
        RUN(test_boot_unaligned_root_pointer_masking);
        RUN(test_pointer_wp_accumulates_to_atc);
        RUN(test_l2_pointer_wp_cold_write_fault_no_fill);
        RUN(test_clean_read_does_not_set_modified);
        RUN(test_pointer_used_bits_set_after_success);
        RUN(test_fault_does_not_write_pending_pointer_used);
        RUN(test_unsupported_pointer_descriptor_faults);
        RUN(test_indirect_leaf_descriptor_faults);
        RUN(test_l1_axi_error_faults_invalid_pointer);
        RUN(test_leaf_axi_error_faults_invalid_page);
        RUN(test_sup_fill_does_not_authorize_user);
        RUN(test_dtt_bypass_invalid_tables_no_walk);
        RUN(test_dtt_write_protect_fault_no_walk);
        RUN(test_req_va_write_latched_while_busy);
        RUN(test_axi_write_address_before_data);
        RUN(test_axi_write_data_before_address);

        printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
        dut->final();
        delete dut;
        return (n_fail == 0) ? 0 : 1;
    }

    RUN(test_4k_3lvl_miss_then_hit);
    RUN(test_8k_3lvl_walk);
    RUN(test_2lvl_4k_walk);
    RUN(test_atc_hit_after_fill);
    RUN(test_held_req_valid_no_rewalk);
    RUN(test_invalid_pointer);
    RUN(test_invalid_page);
    RUN(test_wp_violation);
    RUN(test_supervisor_only_user_fault);
    RUN(test_pflush_all_clears_atc);
    RUN(test_pflush_va_only_one);
    RUN(test_pflush_va_clears_user_and_supervisor_tags);
    RUN(test_pflush_asid_stub);
    RUN(test_ptest_probe_hits);
    RUN(test_ptest_ttr_fast_path);
    RUN(test_ptest_uses_ptest_sup_root);
    RUN(test_modified_bit_set_on_write);
    RUN(test_used_bit_set);
    RUN(test_asm_descriptor_constants);
    RUN(test_rom_descriptor_constants);
    RUN(test_cross_page_access);
    RUN(test_itt_passthrough_still_works);
    RUN(test_mmu_disabled_passthrough);

    // ── Boot-critical widening scenarios ─────────────────────────────
    RUN(test_boot_fixed_040_slices_ignore_legacy_tc_fields);
    RUN(test_boot_urp_srp_select_distinct_roots);
    RUN(test_boot_unaligned_root_pointer_masking);
    RUN(test_pointer_wp_accumulates_to_atc);
    RUN(test_l2_pointer_wp_cold_write_fault_no_fill);
    RUN(test_clean_read_does_not_set_modified);
    RUN(test_pointer_used_bits_set_after_success);
    RUN(test_fault_does_not_write_pending_pointer_used);
    RUN(test_unsupported_pointer_descriptor_faults);
    RUN(test_indirect_leaf_descriptor_faults);
    RUN(test_l1_axi_error_faults_invalid_pointer);
    RUN(test_leaf_axi_error_faults_invalid_page);
    RUN(test_pointer_writeback_axi_error_faults);
    RUN(test_leaf_writeback_axi_error_faults);
    RUN(test_sup_fill_does_not_authorize_user);
    RUN(test_dtt_bypass_invalid_tables_no_walk);
    RUN(test_dtt_write_protect_fault_no_walk);
    RUN(test_req_va_write_latched_while_busy);
    RUN(test_axi_write_address_before_data);
    RUN(test_axi_write_data_before_address);

    // ── Stress / adversarial scenarios (task #100) ────────────────────
    // Category 1 — ATC thrashing
    RUN(test_atc_thrash_128_pages);
    RUN(test_atc_lru_pressure);
    RUN(test_atc_every_set_touched);
    RUN(test_atc_4way_conflict_miss);
    // Category 2 — walker throughput
    RUN(test_back_to_back_8_walks);
    RUN(test_walker_single_in_flight);
    RUN(test_mixed_2lvl_3lvl_walks);
    // Category 3 — PFLUSH under load
    RUN(test_pflush_va_during_walk);
    RUN(test_pflush_va_matches_inflight);
    RUN(test_pflush_va_matches_inflight_any_sup_tag);
    RUN(test_pflush_asid_mixed_entries);
    RUN(test_pflusha_midwalk);
    // Category 4 — page-boundary straddling
    RUN(test_cross_page_long_split);
    RUN(test_movem_like_burst_across_page);
    RUN(test_ifetch_cross_page_simulated);
    // Category 5 — page-size switching
    RUN(test_pagesize_switch_4k_to_8k);
    RUN(test_pagesize_consecutive);
    RUN(test_pagesize_switch_pflushall);
    // Category 6 — root-pointer swap
    RUN(test_srp_swap_reflects);
    RUN(test_urp_change_no_inval);
    RUN(test_srp_no_affect_user);
    // Category 7 — FC crossings
    RUN(test_sup_only_user_access);
    RUN(test_sup_only_sup_access);
    RUN(test_sup_access_user_page);
    RUN(test_toggle_fc_same_va);
    // Category 8 — exception-during-walk
    RUN(test_fault_then_recover);
    RUN(test_fault_then_different_va);
    RUN(test_wp_fault_no_write_leak);
    // Category 10 — contention / slow-slave
    RUN(test_slow_axi_slave);
    RUN(test_axi_backpressure_ar);
    RUN(test_axi_backpressure_b);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
