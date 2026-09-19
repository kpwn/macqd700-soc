// tb_video_smoke_ddr.cpp -- CPU-less video-smoke rig proof, VRAM_IN_DDR.
//
// WHAT THIS PROVES
//   The `VIDEO_SMOKE=1` + `VRAM_IN_DDR` combination -- until now blocked by
//   an elaboration-time `$fatal` -- actually paints the framebuffer, with
//   the RIGHT bytes at the RIGHT addresses, all the way through the real
//   production chain:
//
//     vram_smoke -> axi_vram_smoke_mux (select + `+CARVEOUT_BASE`)
//                -> axi_vram_priority_mux3 (S3 lane)
//                -> axi_async_bridge -> axi_ddr4_mig_bridge -> DRAM
//                -> scanout_ddr_reader (what the HDMI scanner reads)
//
//   Every module in that list is the production one, instantiated by
//   tb_vram_ddr_chain.v exactly the way fpga_top_ddr.vh instantiates it.
//
// THE BYTE-ORDER QUESTION (the one that would silently produce a
// plausible-but-wrong picture)
//   axi_xbar.v applies `vram_swap_words` on its S3 MASTER ports, so CPU
//   traffic arriving at the VRAM lane is already in plain AXI byte-lane
//   order.  vram_smoke authors its words in that same order (lane 0 =
//   lowest-address pixel).  Muxed in at the S3 lane, smoke therefore needs
//   NO swap.  Three scenarios below nail that down:
//
//     smoke_paints_the_carveout        -- smoke bytes read back UNSWAPPED
//     smoke_golden_is_swap_sensitive   -- a swapped golden would NOT match,
//                                         so scenario 1 is not vacuous
//     smoke_matches_cpu_path_byte_order-- a CPU write through the REAL xbar
//                                         S3 swap lands under the same
//                                         model, at the same addresses
//
//   NEGATIVE CONTROL: build the same tb with -GCHAIN_SMOKE_BROKEN_SWAP=1
//   (target `tb-video-smoke-ddr-negctl`).  That byte-reverses smoke's wdata
//   within each 32-bit group before the mux -- i.e. lands smoke on the
//   WRONG side of the S3 swap -- and scenario 1 MUST fail.  The negctl
//   target inverts the exit code, so a silently-insensitive checker breaks
//   the build too.
//
// GEOMETRY
//   64 rows x 16 px, 8bpp, ROW_BAND_LOG2=3.  Chosen so that (a) each SMPTE
//   bar is 2 bytes wide, which puts a bar BOUNDARY inside every 4-byte
//   group -- a within-group byte reversal is therefore visible, which it
//   would NOT be at the production 128-px bar width; and (b) the row-band
//   term is exercised (the phase advances every 8 rows).

#include "Vtb_vram_ddr_chain.h"
#include "verilated.h"

#include <array>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <vector>

// Must match tb_vram_ddr_chain.v's CHAIN_SMOKE_* parameters and the
// Makefile's -G overrides.
static constexpr int SMOKE_W = 16;
static constexpr int SMOKE_H = 64;
static constexpr int SMOKE_ROW_BAND_LOG2 = 3;
static constexpr int SMOKE_BPP = 8;
static constexpr int SMOKE_PX = SMOKE_W * SMOKE_H;   // == byte count at 8bpp

static constexpr uint32_t VRAM_APERTURE_BASE = 0xF9000000u;

static Vtb_vram_ddr_chain* dut = nullptr;
static uint64_t sim_time = 0;
static int g_pass = 0, g_fail = 0;

static void CHECK(const char* name, bool ok) {
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) g_pass++; else g_fail++;
}

// ─── vram_smoke.v's pattern, re-derived independently in C++ ────────────
// This is a POSITIVE CONTROL, not a copy of the RTL: it is written from
// the module's documented contract (8 vertical bars of W/8 px, CLUT-index
// table, bar phase advanced by y >> ROW_BAND_LOG2).  If the RTL and this
// disagree, one of them is wrong -- which is the point.
static uint8_t bar_clut_index(int idx) {
    switch (idx & 7) {
        case 0: return 0x0F;  // WHITE
        case 1: return 0x0B;  // YELLOW
        case 2: return 0x0E;  // CYAN
        case 3: return 0x0A;  // GREEN
        case 4: return 0x0D;  // MAGENTA
        case 5: return 0x09;  // RED
        case 6: return 0x0C;  // BLUE
        default: return 0x00; // BLACK
    }
}
static uint8_t golden_byte(int off) {
    const int bar_w = SMOKE_W / 8;
    int x = off % SMOKE_W;
    int y = off / SMOKE_W;
    int bar = x / bar_w;
    if (bar > 7) bar = 7;
    if (SMOKE_ROW_BAND_LOG2 != 0)
        bar = (bar + (y >> SMOKE_ROW_BAND_LOG2)) % 8;
    return bar_clut_index(bar);
}

// axi_xbar.v's vram_swap_word32 / vram_swap_words, for the CPU-path model.
static void swap32_lane(uint8_t* b) {
    uint8_t t0 = b[0], t1 = b[1], t2 = b[2], t3 = b[3];
    b[0] = t3; b[1] = t2; b[2] = t1; b[3] = t0;
}
static void vram_swap16(uint8_t* beat) {
    for (int lane = 0; lane < 4; lane++) swap32_lane(beat + lane * 4);
}

// `physical[]` is the golden model of what is actually in DRAM at each
// aperture byte offset.  Smoke fills it with the unswapped pattern; a CPU
// write overwrites its span with vram_swap16(beat).
static std::vector<uint8_t> physical(SMOKE_PX, 0);

// ─── clocking + BFMs ────────────────────────────────────────────────────
static void eval() { dut->eval(); }
static void cpu_drive_comb();
static void cpu_latch_edge();
static void scan_drive_pre();
static void scan_latch_post();

static void tick() {
    dut->core_clk = 0; dut->mig_clk = 0; eval();
    cpu_drive_comb();
    scan_drive_pre();
    eval();
    cpu_latch_edge();
    scan_latch_post();
    dut->core_clk = 1; dut->mig_clk = 1; eval();
    sim_time++;
    eval();
}

// ── CPU write BFM (axi_xbar M0, single 16-byte beat) ───────────────────
struct CpuWrite {
    uint32_t id = 1;
    uint64_t addr = 0;
    std::array<uint8_t,16> data{};
    bool aw_done = false, w_done_f = false, b_done = false, resp_ok = false;
    uint64_t b_time = 0;
};
static std::shared_ptr<CpuWrite> cpu_w;

static void set_wide_bytes(VlWide<4>& sig, const uint8_t* bytes) {
    for (int w = 0; w < 4; w++)
        sig[w] = (uint32_t)bytes[w*4] | ((uint32_t)bytes[w*4+1] << 8) |
                 ((uint32_t)bytes[w*4+2] << 16) | ((uint32_t)bytes[w*4+3] << 24);
}

static void cpu_drive_comb() {
    if (cpu_w && !cpu_w->b_done) {
        dut->cpu_awvalid = cpu_w->aw_done ? 0 : 1;
        dut->cpu_awid = cpu_w->id;
        dut->cpu_awaddr = (uint32_t)cpu_w->addr;
        dut->cpu_awlen = 0; dut->cpu_awsize = 4; dut->cpu_awburst = 1;
        dut->cpu_wvalid = cpu_w->w_done_f ? 0 : 1;
        set_wide_bytes(dut->cpu_wdata, cpu_w->data.data());
        dut->cpu_wstrb = 0xFFFF;
        dut->cpu_wlast = 1;
    } else {
        dut->cpu_awvalid = 0; dut->cpu_wvalid = 0;
    }
    dut->cpu_arvalid = 0;
    dut->cpu_bready = 1;
    dut->cpu_rready = 1;
}
static void cpu_latch_edge() {
    if (cpu_w && !cpu_w->b_done) {
        if (!cpu_w->aw_done && dut->cpu_awvalid && dut->cpu_awready) cpu_w->aw_done = true;
        if (!cpu_w->w_done_f && dut->cpu_wvalid && dut->cpu_wready) cpu_w->w_done_f = true;
        if (dut->cpu_bvalid && dut->cpu_bready) {
            cpu_w->b_done = true;
            cpu_w->resp_ok = (dut->cpu_bresp == 0);
            cpu_w->b_time = sim_time;
        }
    }
}

// ── scanout streaming BFM (byte at rd_addr is rd_data[31:24]) ──────────
static std::deque<uint32_t> scan_pending;
static std::vector<uint8_t> scan_got;
static bool scan_active = false;
static uint32_t scan_next = 0, scan_stop = 0;
static bool scan_req_this_cycle = false;
static constexpr size_t SCAN_MAX_OUTSTANDING = 15;

static void scan_drive_pre() {
    scan_req_this_cycle = scan_active && (scan_next < scan_stop) &&
                          (scan_pending.size() < SCAN_MAX_OUTSTANDING);
    dut->scan_rd_en = scan_req_this_cycle ? 1 : 0;
    if (scan_req_this_cycle) dut->scan_rd_addr = scan_next;
}
static void scan_latch_post() {
    if (scan_req_this_cycle) { scan_pending.push_back(scan_next); scan_next++; }
    if (dut->scan_rd_valid && !scan_pending.empty()) {
        uint32_t off = scan_pending.front(); scan_pending.pop_front();
        (void)off;
        scan_got.push_back((uint8_t)((dut->scan_rd_data >> 24) & 0xFFu));
    }
}
// Read [from, to) through the scanout port; returns the bytes in order.
static bool scan_range(uint32_t from, uint32_t to, std::vector<uint8_t>& out,
                       int max_cycles = 400000) {
    scan_pending.clear(); scan_got.clear();
    scan_next = from; scan_stop = to; scan_active = true;
    int c = 0;
    while (scan_got.size() < (size_t)(to - from) && c < max_cycles) { tick(); c++; }
    scan_active = false;
    for (int i = 0; i < 40; i++) tick();
    out = scan_got;
    return out.size() == (size_t)(to - from);
}

static int count_mismatches(const std::vector<uint8_t>& got,
                            const std::vector<uint8_t>& want,
                            const char* tag, int report_n = 6) {
    int n = 0;
    for (size_t i = 0; i < got.size() && i < want.size(); i++) {
        if (got[i] != want[i]) {
            if (n < report_n)
                printf("    %s mismatch @off=0x%03zx got=%02x want=%02x (x=%zu y=%zu)\n",
                       tag, i, got[i], want[i], i % SMOKE_W, i / SMOKE_W);
            n++;
        }
    }
    return n;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_vram_ddr_chain;

    const bool expect_fail = (std::getenv("SMOKE_DDR_EXPECT_FAIL") != nullptr);

    printf("── tb_video_smoke_ddr: VIDEO_SMOKE=1 through the VRAM_IN_DDR chain ──\n");
    printf("   geometry %dx%d @%dbpp, ROW_BAND_LOG2=%d, %d bytes\n",
           SMOKE_W, SMOKE_H, SMOKE_BPP, SMOKE_ROW_BAND_LOG2, SMOKE_PX);

    dut->core_clk = 0; dut->mig_clk = 0;
    dut->core_rst = 1; dut->mig_rst = 1; dut->slv_flush = 0;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->cpu_arvalid = 0;
    dut->cpu_bready = 1; dut->cpu_rready = 1;
    dut->scan_rd_en = 0; dut->scan_rd_addr = 0;
    for (int i = 0; i < 20; i++) tick();
    dut->core_rst = 0; dut->mig_rst = 0;
    for (int i = 0; i < 20; i++) tick();

    // Wait for the behavioural MIG to report calibration.
    { int c = 0; while (!dut->cal_done && c < 10000) { tick(); c++; } }
    CHECK("sim MIG backend reports cal_done", dut->cal_done != 0);

    // ── Scenario 1: smoke paints the carveout ──────────────────────────
    // A CPU write is parked in the queue from the very start so we can
    // prove the mux stalls S3 while smoke owns the lane.  It targets the
    // first 16 bytes ABOVE the smoke frame, so it can neither corrupt the
    // pattern nor be corrupted by it -- the two spans are disjoint and the
    // ordering between them is therefore irrelevant to the comparison.
    static constexpr uint32_t CPU_OFF = SMOKE_PX;
    cpu_w = std::make_shared<CpuWrite>();
    cpu_w->addr = VRAM_APERTURE_BASE + CPU_OFF;
    for (int i = 0; i < 16; i++) cpu_w->data[i] = (uint8_t)(0xA0 + i);

    uint64_t smoke_done_time = 0;
    { int c = 0;
      while (!dut->smoke_done && c < 400000) { tick(); c++; }
      smoke_done_time = sim_time; }
    CHECK("smoke_done asserts within 400k cycles", dut->smoke_done != 0);
    printf("    smoke_done at t=%llu\n", (unsigned long long)smoke_done_time);

    bool cpu_b_before_smoke = cpu_w->b_done && (cpu_w->b_time < smoke_done_time);
    CHECK("S3 CPU write did NOT complete while smoke owned the lane",
          !cpu_b_before_smoke);

    for (int i = 0; i < 8; i++) physical.assign(SMOKE_PX, 0);
    for (int i = 0; i < SMOKE_PX; i++) physical[i] = golden_byte(i);

    std::vector<uint8_t> got;
    bool complete = scan_range(0, SMOKE_PX, got);
    CHECK("scanout returned one response per request", complete);

    int mism = complete ? count_mismatches(got, physical, "smoke") : -1;
    if (complete && mism)
        printf("    %d/%d smoke bytes wrong\n", mism, SMOKE_PX);
    CHECK("smoke pattern reads back byte-exact through scanout_ddr_reader",
          complete && mism == 0);

    // ── Scenario 2: the golden is swap-SENSITIVE ───────────────────────
    // If a within-32-bit-group byte reversal of the golden ALSO matched,
    // scenario 1 would be blind to exactly the bug it exists to catch.
    if (complete) {
        std::vector<uint8_t> swapped = physical;
        for (size_t i = 0; i + 16 <= swapped.size(); i += 16)
            vram_swap16(&swapped[i]);
        int swap_mism = 0;
        for (size_t i = 0; i < swapped.size(); i++)
            if (swapped[i] != physical[i]) swap_mism++;
        printf("    a byte-reversed golden differs in %d/%d bytes\n",
               swap_mism, SMOKE_PX);
        CHECK("golden pattern is swap-sensitive (negative control is meaningful)",
              swap_mism > SMOKE_PX / 4);
    } else {
        CHECK("golden pattern is swap-sensitive (negative control is meaningful)", false);
    }

    // ── Scenario 3: CPU-path byte order agrees with smoke's ────────────
    // The parked CPU write must have landed by now.  Its physical DRAM
    // bytes are vram_swap16(beat) -- axi_xbar.v's S3 swap.  Reading them
    // back through the SAME scanout port that just read smoke's bytes
    // proves both sources are measured on one scale: smoke is on the
    // post-swap side, exactly where CPU traffic arrives.
    { int c = 0; while (!(cpu_w && cpu_w->b_done) && c < 200000) { tick(); c++; } }
    CHECK("parked S3 CPU write completes after smoke_done",
          cpu_w->b_done && cpu_w->resp_ok);

    {
        std::array<uint8_t,16> expect = cpu_w->data;
        vram_swap16(expect.data());
        std::vector<uint8_t> cpu_got;
        bool ok = scan_range(CPU_OFF, CPU_OFF + 16, cpu_got);
        bool match = ok;
        for (int i = 0; i < 16 && match; i++) match = (cpu_got[i] == expect[i]);
        if (!match && ok) {
            printf("    cpu-path got=");
            for (int i = 0; i < 16; i++) printf("%02x", cpu_got[i]);
            printf(" want=");
            for (int i = 0; i < 16; i++) printf("%02x", expect[i]);
            printf("\n");
        }
        CHECK("CPU write through the real xbar S3 swap lands under the same model",
              match);
    }

    // ── Scenario 4: the CPU write did not spray the frame ──────────────
    // Re-read a slice of the smoke frame after the CPU write landed; it
    // must still hold smoke's pattern.  A mistranslated CPU address (the
    // classic `+CARVEOUT_BASE` applied twice, or not at all) would land
    // somewhere inside the frame instead of just above it.
    {
        const uint32_t from = 0x180, to = 0x200;
        std::vector<uint8_t> tail;
        bool ok = scan_range(from, to, tail);
        std::vector<uint8_t> want(physical.begin() + from, physical.begin() + to);
        int n = ok ? count_mismatches(tail, want, "tail") : -1;
        CHECK("frame outside the CPU-written beat still holds the smoke pattern",
              ok && n == 0);
    }

    printf("──────────────────────────────────────────\n");
    printf("tb_video_smoke_ddr: %d PASS / %d FAIL\n", g_pass, g_fail);
    delete dut;

    if (expect_fail) {
        // Negative-control build: the checker MUST have detected the
        // deliberately-wrong byte order.
        if (g_fail == 0) {
            printf("NEGATIVE CONTROL FAILED: broken-swap build passed the checker.\n");
            return 1;
        }
        printf("NEGATIVE CONTROL OK: broken-swap build was detected (%d failures).\n", g_fail);
        return 0;
    }
    return g_fail == 0 ? 0 : 1;
}
