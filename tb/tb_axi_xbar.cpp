// tb_axi_xbar.cpp — Verilator unit testbench for rtl/soc/axi_xbar.v
//
// Exercises the AXI4 crossbar against multiple scenarios including the
// VRAM pixel aperture routing (task #147) and, as of 2026-07-16, the
// master-count reduction (5M -> 3M: CPU LSU merged with boot FSM behind
// a cpu_held_in_reset mux on M0, CPU IF renumbered M2, DMA's M4 master
// port stubbed out of the crossbar entirely — see axi_xbar.v's header).
//
//   1. CPU (M0) reads from RAM return via DDR path, BRESP/RRESP=OKAY
//   2. CPU (M0) writes to ROM region → BRESP=OKAY, silently dropped
//   3. Boot FSM (merged onto M0 via m0b_*, cpu_held_in_reset=1) writes to
//      ROM → BRESP=OKAY, forwarded to DDR at flattened offset
//      0x4000_0000 + (addr - 0x4000_0000)
//   3b. Host debug (M1) writes to ROM → BRESP=OKAY, forwarded to DDR at
//      flattened offset 0x4000_0000 + (addr - 0x4000_0000)
//   3c-3e. M0/boot merge correctness (added for the master-count
//      reduction): CPU LSU is structurally blocked from getting an AW
//      grant while cpu_held_in_reset=1 (not merely "doesn't try" — the
//      mux never samples its valid); after release, CPU LSU writes/reads
//      the shared physical port normally; and an adversarial scenario
//      that drives BOTH the CPU-LSU and boot-FSM AW channels valid
//      simultaneously proves the xbar's plain 2:1 mux is exclusive at
//      the wire level regardless of what upstream does — the two sides
//      can never blend into a single forwarded transaction.
//   4. CPU IF (M2) reads from framebuffer → forwarded to DDR at flattened
//      offset 0x4040_0000 + (addr - 0x6000_0000)
//   5. XDMA/JTAG host (M1) reads from I/O region → forwarded to S1
//      (peripheral bus)
//   6-11. Contention, decerr/open-bus, stress, cross-slave-parallel.
//   12-15. (DISABLED, kept for history) M4/DMA-master scenarios — the
//      M4 port no longer exists on the DUT since the 2026-07-16
//      master-count reduction; dma_ctrl's AXI4-Lite config port (S2) is
//      still covered by scenario 13.
//   16-27. Boundary/edge-case + VRAM aperture scenarios, updated for the
//      M2 (was M3) CPU-IF renumbering; M4 legs dropped where the
//      scenario mixed masters (M0/M1 legs retained unchanged).
//
// Built via: make tb-axi-xbar
// Pass output: "All N scenarios PASSED."

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <deque>
#include <map>
#include <string>
#include <verilated.h>
#include "Vaxi_xbar.h"

static Vaxi_xbar* dut = nullptr;
static uint64_t   sim_time = 0;
static int        n_pass = 0, n_fail = 0;

static constexpr int DATA_WIDTH = 128;
static constexpr int STRB_WIDTH = DATA_WIDTH / 8;
static constexpr uint32_t DDR_ROM_OFF = 0x40000000u;
static constexpr uint32_t DDR_FB_OFF = 0x40400000u;
static constexpr uint32_t RAM_DECODE_SIZE = 0x40000000u;
static constexpr uint32_t RAM_ALIAS_BASE = 0x58000000u;
static constexpr uint32_t ROM_WINDOW_SIZE = 0x00400000u;
static constexpr uint32_t RAM_WINDOW_MASK_DFLT = 0x03ffffffu; // 64 MiB - 1
static constexpr uint32_t RAM_WINDOW_MASK_4M = 0x003fffffu; // 4 MiB - 1

// Task review round 2, IMPORTANT: the RTL's WD_LOG2 default is 18
// (2^18 = 262144 cycles), but this Makefile rule overrides it with
// -GWD_LOG2=12 (2^12 = 4095 = WD_MAX_CNT) so the watchdog-timeout
// scenarios stay fast.  Every "wait out the watchdog" loop in this file
// uses WD_WAIT_CYCLES instead of a scenario-local magic number, so a
// future WD_LOG2 override change is a one-line edit.  Margin covers:
// whatever AW/W/AR setup a scenario already burned before starting its
// wait (issue_write/issue_read's own ~400-cycle tail), PLUS the
// len==255 edge case (scenario 35), which after the fire still needs
// 256 more cycles to stream out the fully-synthesized burst one beat
// per cycle via RS_SEND_RLOCAL.
static constexpr int WD_WAIT_CYCLES = 4600;

// ─── 128-bit data helpers (Verilator exposes as VlWide<4>) ──────────────
using Word128 = std::array<uint32_t, 4>;

static Word128 w128_from_u64(uint64_t lo, uint64_t hi = 0) {
    return {(uint32_t)(lo & 0xFFFFFFFFu),
            (uint32_t)(lo >> 32),
            (uint32_t)(hi & 0xFFFFFFFFu),
            (uint32_t)(hi >> 32)};
}
static bool w128_eq(const Word128& a, const Word128& b) {
    return a == b;
}
static std::string w128_hex(const Word128& a) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%08x_%08x_%08x_%08x", a[3], a[2], a[1], a[0]);
    return buf;
}

static uint32_t bswap32(uint32_t x) {
    return ((x & 0x000000FFu) << 24) |
           ((x & 0x0000FF00u) << 8)  |
           ((x & 0x00FF0000u) >> 8)  |
           ((x & 0xFF000000u) >> 24);
}

static Word128 vram_native_word_order(const Word128& a) {
    return {bswap32(a[0]), bswap32(a[1]), bswap32(a[2]), bswap32(a[3])};
}

static void set_byte(Word128& w, int idx, uint8_t val) {
    int word = idx / 4;
    int lane = idx % 4;
    w[word] &= ~(0xFFu << (lane * 8));
    w[word] |= (uint32_t)val << (lane * 8);
}

// Read/write master data ports via Verilator's WData view (4 x 32-bit lanes).
template <typename Port>
static void set_wdata(Port& p, const Word128& v) {
    for (int i = 0; i < 4; i++) p[i] = v[i];
}
template <typename Port>
static Word128 get_wdata(Port& p) {
    Word128 v{};
    for (int i = 0; i < 4; i++) v[i] = p[i];
    return v;
}

// ─── Slave memory model ─────────────────────────────────────────────────
// A simple flat sparse store keyed by 128-bit-aligned address.
struct SlaveMem {
    std::map<uint64_t, Word128> store;
    Word128 read(uint32_t a) {
        auto it = store.find(a & ~0xFu);
        return (it == store.end()) ? Word128{0,0,0,0} : it->second;
    }
    void write(uint32_t a, const Word128& d) { store[a & ~0xFu] = d; }
    void write_masked(uint32_t a, const Word128& d, uint16_t strb) {
        Word128 cur = read(a);
        for (int i = 0; i < 16; i++) {
            if (strb & (1u << i)) {
                int word = i / 4;
                int lane = i % 4;
                cur[word] &= ~(0xFFu << (lane * 8));
                cur[word] |= ((d[word] >> (lane * 8)) & 0xFFu) << (lane * 8);
            }
        }
        store[a & ~0xFu] = cur;
    }
};
static SlaveMem s0_mem;  // DDR (flattened)
static SlaveMem s1_mem;  // I/O (peripheral-bus backing store)
static SlaveMem s2_mem;  // DMA config (task #116 — 5M×3S retune)
static SlaveMem s3_mem;  // VRAM pixel aperture.
                         // Keys into this store are ZERO-BASED byte
                         // offsets into the framebuffer; the xbar has
                         // already peeled off VRAM_BASE (0xF900_0000).
static SlaveMem s4_mem;  // DAFB register shim.  Keys are zero-based byte
                         // offsets into 0xF980_0000..0xF980_0FFF.

// ─── Slave BFMs ─────────────────────────────────────────────────────────
// A slave FSM per port.  Captures AW+W bursts, applies them to the local
// store, then returns B.  For AR, returns an RLAST-terminated burst from
// the store.  No stalling beyond one-cycle ready delay per beat.
struct SlaveBfm {
    // AW state
    bool     aw_busy   = false;
    uint32_t aw_id     = 0;
    uint32_t aw_addr   = 0;
    uint8_t  aw_len    = 0;
    // W state (beats remaining)
    int      w_beats_left = 0;
    uint32_t w_cur_addr   = 0;
    // WLAST-placement + filler-beat audit (task: xbar-burst-gaps, GAP 1).
    // The BFM counts beats off AWLEN and deliberately ignores WLAST, so
    // WLAST correctness has to be asserted separately: every accepted beat
    // must carry WLAST exactly when it is the burst's last per AWLEN.  This
    // is what makes an xbar-side W pad provable rather than merely
    // plausible -- a pad that emitted the right NUMBER of beats with WLAST
    // in the wrong place would leave the slave's own burst counter
    // misaligned on real hardware.
    int      w_beats_seen    = 0;
    int      w_wlast_errors  = 0;
    int      w_pad_beats     = 0;  // beats accepted with WSTRB == 0
    // B
    bool     b_pending = false;
    uint32_t b_id      = 0;

    // AR
    bool     ar_busy   = false;
    uint32_t ar_id     = 0;
    uint32_t ar_addr   = 0;
    uint8_t  ar_len    = 0;
    int      r_beats_left = 0;
    Word128  r_cur_data;
    int      r_beats_delivered = 0; // task-review mid-burst read-timeout scenarios

    std::map<uint32_t, int> seen_aw_count;  // per-addr AW hits (debug)
    std::map<uint32_t, int> seen_ar_count;
};
static SlaveBfm s0_bfm, s1_bfm, s2_bfm, s3_bfm, s4_bfm;

// Backpressure injection for S0 (scenario 15, and task-review mid-burst
// WS_FWD_W-timeout scenario: set to a value far larger than any single
// test's cycle budget to model "never advances again", i.e. a
// permanently wedged slave rather than a bounded stall).
static int s0_stall_aw = 0;
static int s0_stall_w  = 0;
// Task-review mid-burst RS_WAIT_R-timeout scenarios: S0 stops driving
// RVALID once it has delivered this many beats of the CURRENT AR
// (-1 = disabled, deliver the whole burst normally).  Lets a burst
// genuinely enter RS_WAIT_R, deliver SOME real (OKAY) data, then wedge
// — distinct from RS_WAIT_SLV_AR (AR never even accepted).
static int s0_r_swallow_after = -1;

// Swallow hooks for S1 (scenario 30 — xbar-burst-guard watchdog).  When
// set, S1's BFM still accepts AW+W normally (so the xbar's write slot
// really does reach WS_WAIT_B, matching "a slave that never returns B")
// but never drives s1_bvalid — the transaction is accepted-but-wedged,
// exactly the case the Fix 2 watchdog exists for.
static bool s1_swallow_b  = false;
// Never assert s1_arready -- parks a read in RS_WAIT_SLV_AR so a slot-0
// abort can be observed while the transaction is genuinely in flight
// (scenario 44).  Mirrors s2_swallow_ar.
static bool s1_swallow_ar = false;
// Backlog item 3: "the bridge behind S1 is still in reset".  Distinct from
// s1_swallow_b, which models a LIVE slave that owes a B it never sends: a
// bridge in reset accepts nothing, remembers nothing and will NEVER answer,
// so a transaction handed to it in that window is simply gone.  Set for the
// pb-side reset-deassert skew window in scenario 47.
static bool s1_in_reset = false;
// Handshakes the resetting bridge swallowed and forgot.  Must stay 0.
static int  s1_reset_swallowed_aw = 0;
static int  s1_reset_swallowed_ar = 0;

// Task-review scenarios (slv_flush + read-side/mid-burst watchdog +
// poison-clear) deliberately target S2 (DMA config) instead of S1, so
// they don't disturb S1's already-poisoned end state from scenario 30.
// Backpressure/swallow hooks mirroring S0's and S1's above.
static int  s2_stall_aw   = 0;
static bool s2_swallow_ar = false; // never assert s2_arready — RS_WAIT_SLV_AR timeout
static bool s2_swallow_b  = false; // accept AW+W, never assert s2_bvalid — WS_WAIT_B timeout / flush-abort

// Task review round 2, item 4b: S3 (VRAM) IS in the flush domain (see
// is_flush_domain_slv() in the RTL) even though it's burst-capable, so
// it needs its own swallow hook to exercise the flush-abort path.
static bool s3_swallow_b = false;
static int s3_r_swallow_after = -1;

// ─── Multi-outstanding S3 read mode (backlog item 2) ──────────────────
// The default S3 BFM is single-outstanding on reads (ar_busy gates
// ARREADY), which cannot model the topology the S3 read quarantine
// actually has to survive: the read path is ID-routed with no per-slave
// owner lock, so several slots can have S3 reads in flight at once and
// their bursts can come back in ANY order.  This mode queues ARs, lets a
// scenario freeze R delivery, and lets it choose which queued burst is
// served first.
// NOTE ON IDs: ID_WIDTH is 4 in this build and Verilator does NOT mask an
// oversized value written to an input port, so an AWID/ARID wider than 4 bits
// overflows the xbar's internal {slot_tag, id} concatenation and corrupts the
// SLOT TAG -- the R burst then routes to the wrong slot and the transaction
// silently never completes.  Keep every id in these scenarios <= 0xF.
struct S3ArEntry { uint32_t id; uint32_t addr; uint8_t len; };
static bool  s3_r_multi = false;   // enable the queued BFM
static bool  s3_r_hold  = false;   // accept ARs, deliver no R beats
static bool  s3_r_lifo  = false;   // serve the most recently accepted AR first
static std::deque<S3ArEntry> s3_ar_q;
static bool     s3_r_cur_busy = false;
static S3ArEntry s3_r_cur{};
static int      s3_r_cur_left = 0;
static uint32_t s3_r_cur_addr = 0;
static int      s3_r_bursts_done = 0;
static void s3_r_multi_reset() {
    s3_r_multi = false; s3_r_hold = false; s3_r_lifo = false;
    s3_ar_q.clear();
    s3_r_cur_busy = false; s3_r_cur_left = 0; s3_r_cur_addr = 0;
    s3_r_bursts_done = 0;
}
// "Nothing is left owed on S3's read channel."
static bool s3_r_multi_drained() {
    return s3_ar_q.empty() && !s3_r_cur_busy;
}

// ─── Early-W acceptance mode + handshake-cycle probes (scenarios 28-30,
//     2026-07-21 write/read dead-cycle removal) ─────────────────────────
// s0_early_w_accept models an axi_async_bridge-style slave that FIFOs a
// W beat before its AW has been accepted (single-beat only).  The cycle
// probes record the BFM's own commit points so scenarios can assert
// exact handshake timing.
static bool     s0_early_w_accept = false;
static bool     s0_early_w_have   = false;
static Word128  s0_early_w_data{};
static uint16_t s0_early_w_strb   = 0;
static long s0_aw_capture_cycle = -1;  // sim_time at last AW capture
static long s0_first_w_cycle    = -1;  // sim_time at first W-beat capture
static long s0_ar_capture_cycle = -1;  // sim_time at last AR capture
static long s0_first_r_hs_cycle = -1;  // sim_time at first R-beat handshake

// ─── Clock helpers ──────────────────────────────────────────────────────
// MINOR 7 (task review): the RTL's sim-only region-straddle assertion
// (axi_xbar.v gen_aw_straddle_chk/gen_ar_straddle_chk) fires $finish,
// which by itself only sets Verilated::gotFinish() and prints a
// message — it does NOT stop this harness's control flow.  Without this
// check the assertion would silently fail to fail the run.  Checked
// once per tick() so it fires as soon as possible after either clock
// edge's $finish.
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
    if (Verilated::gotFinish()) {
        std::printf("[FATAL] Verilog $finish fired (sim-only assertion tripped) at sim_time=%llu — aborting test run.\n",
                    (unsigned long long)sim_time);
        std::fflush(stdout);
        std::exit(1);
    }
}

// ─── Master BFM state (per master) ──────────────────────────────────────
struct MasterBfm {
    // Outstanding write expectations in order:
    //   each entry: expected BID, expected BRESP
    std::deque<std::pair<uint32_t,uint32_t>> w_expect;
    // Completed writes (popped as BVALIDs arrive)
    int w_completed_ok = 0;
    int w_completed_errs = 0;

    // Outstanding reads in order: expected RID + RRESP for each beat group
    std::deque<std::pair<uint32_t,uint32_t>> r_expect;
    int r_completed = 0;
    int r_errors    = 0;

    // Captured R-beat data per completed transaction (for verification).
    std::vector<std::vector<Word128>> r_received;
    std::vector<Word128>              r_cur;   // accumulating current burst
    // Task review: per-BEAT RRESP alongside r_received/r_cur above — the
    // pre-existing r_expect/observe_r machinery only checks RRESP on the
    // beat carrying RLAST.  A burst-reject/watchdog-synthesized SLVERR
    // burst must show SLVERR on EVERY beat, not just the last one;
    // scenario 29 checks this explicitly.
    std::vector<std::vector<uint32_t>> r_received_resp;
    std::vector<uint32_t>              r_cur_resp;
    // Bus-side RLAST handshakes seen. This is EXACTLY the event the core's
    // AxiReadResetAbsorber decrements on, so counting it here models the
    // invariant the core actually depends on (AR issued == RLAST returned)
    // rather than a proxy for it.
    int r_lasts_seen = 0;
};
// Logical master indices (NOT raw xbar port numbers — M0/boot share one
// physical port; see axi_xbar.v header "M0/boot merge"):
//   mb[0] = CPU LSU        (m0_*,  physical M0, active when !cpu_held_in_reset)
//   mb[1] = host debug     (m1_*,  physical M1)
//   mb[2] = CPU IF         (m2_*,  physical M2, read-only)
//   mb[3] = boot FSM       (m0b_*, physical M0, active when cpu_held_in_reset)
// mb[4] is unused (was DMA/M4, stubbed out of the crossbar 2026-07-16).
static MasterBfm mb[5];

// ─── Idle inputs (per master — all defaulted off) ───────────────────────
static void idle_m0() {
    dut->m0_awid = 0; dut->m0_awaddr = 0; dut->m0_awlen = 0;
    dut->m0_awsize = 0; dut->m0_awburst = 0; dut->m0_awvalid = 0;
    set_wdata(dut->m0_wdata, Word128{0,0,0,0});
    dut->m0_wstrb = 0; dut->m0_wlast = 0; dut->m0_wvalid = 0;
    dut->m0_bready = 1;
    dut->m0_arid = 0; dut->m0_araddr = 0; dut->m0_arlen = 0;
    dut->m0_arsize = 0; dut->m0_arburst = 0; dut->m0_arvalid = 0;
    dut->m0_rready = 1;
}
static void idle_m0b() {
    dut->m0b_awid = 0; dut->m0b_awaddr = 0; dut->m0b_awlen = 0;
    dut->m0b_awsize = 0; dut->m0b_awburst = 0; dut->m0b_awvalid = 0;
    set_wdata(dut->m0b_wdata, Word128{0,0,0,0});
    dut->m0b_wstrb = 0; dut->m0b_wlast = 0; dut->m0b_wvalid = 0;
    dut->m0b_bready = 1;
}
static void idle_m1() {
    dut->m1_awid = 0; dut->m1_awaddr = 0; dut->m1_awlen = 0;
    dut->m1_awsize = 0; dut->m1_awburst = 0; dut->m1_awvalid = 0;
    set_wdata(dut->m1_wdata, Word128{0,0,0,0});
    dut->m1_wstrb = 0; dut->m1_wlast = 0; dut->m1_wvalid = 0;
    dut->m1_bready = 1;
    dut->m1_arid = 0; dut->m1_araddr = 0; dut->m1_arlen = 0;
    dut->m1_arsize = 0; dut->m1_arburst = 0; dut->m1_arvalid = 0;
    dut->m1_rready = 1;
}
static void idle_m2() {
    dut->m2_arid = 0; dut->m2_araddr = 0; dut->m2_arlen = 0;
    dut->m2_arsize = 0; dut->m2_arburst = 0; dut->m2_arvalid = 0;
    dut->m2_rready = 1;
}
static void idle_all_masters() {
    idle_m0(); idle_m0b(); idle_m1(); idle_m2();
    dut->cpu_held_in_reset = 0;
}

// ─── Drive one cycle of the slave BFMs + observe master side ────────────
// All per-cycle handshaking happens between rising edges.  This routine
// runs once per tick and drives slave-side handshakes as well as sampling
// master-side B/R to update the per-master BFMs.
static void step_slave_s0();
static void step_slave_s1();
static void step_slave_s2();
static void step_slave_s3();
static void step_slave_s4();
static void step_masters_observe();

static void cycle() {
    step_slave_s0();
    step_slave_s1();
    step_slave_s2();
    step_slave_s3();
    step_slave_s4();
    // Evaluate so slave drives propagate to master-side wiring combinationally.
    dut->eval();
    step_masters_observe();
    tick();
}

// Reset
static void reset() {
    idle_all_masters();
    dut->cpu_overlay_active = 0;
    dut->cpu_overlay_reset = 0;
    // slv_flush (task review CRITICAL fix): tied 0 by default — the
    // direct-instantiation harness here has no separate reset-domain
    // bridges to model, so only the dedicated slv_flush scenarios pulse
    // this input; everywhere else it must stay deasserted.
    dut->slv_flush = 0;
    // s1_far_reset: the pb island's third reset source (warm_peripheral_reset,
    // the 68040 RESET instruction).  Tied 0 except in scenario 54.
    dut->s1_far_reset = 0;
    // m0b_master_reset: boot_fsm's own reset (boot_fsm_rst).  Tied 0 except
    // in scenario 55.
    dut->m0b_master_reset = 0;
    dut->dbg_ram_window_lg2 = 26; // 64 MiB default window
    // Slave inputs initially "no response"
    dut->s0_awready = 0; dut->s0_wready = 0; dut->s0_bid = 0; dut->s0_bresp = 0; dut->s0_bvalid = 0;
    dut->s0_arready = 0; dut->s0_rid = 0; dut->s0_rresp = 0; dut->s0_rlast = 0; dut->s0_rvalid = 0;
    set_wdata(dut->s0_rdata, Word128{0,0,0,0});
    dut->s1_awready = 0; dut->s1_wready = 0; dut->s1_bid = 0; dut->s1_bresp = 0; dut->s1_bvalid = 0;
    dut->s1_arready = 0; dut->s1_rid = 0; dut->s1_rresp = 0; dut->s1_rlast = 0; dut->s1_rvalid = 0;
    set_wdata(dut->s1_rdata, Word128{0,0,0,0});
    dut->s2_awready = 0; dut->s2_wready = 0; dut->s2_bid = 0; dut->s2_bresp = 0; dut->s2_bvalid = 0;
    dut->s2_arready = 0; dut->s2_rid = 0; dut->s2_rresp = 0; dut->s2_rlast = 0; dut->s2_rvalid = 0;
    set_wdata(dut->s2_rdata, Word128{0,0,0,0});
    dut->s3_awready = 0; dut->s3_wready = 0; dut->s3_bid = 0; dut->s3_bresp = 0; dut->s3_bvalid = 0;
    dut->s3_arready = 0; dut->s3_rid = 0; dut->s3_rresp = 0; dut->s3_rlast = 0; dut->s3_rvalid = 0;
    set_wdata(dut->s3_rdata, Word128{0,0,0,0});
    dut->s4_awready = 0; dut->s4_wready = 0; dut->s4_bid = 0; dut->s4_bresp = 0; dut->s4_bvalid = 0;
    dut->s4_arready = 0; dut->s4_rid = 0; dut->s4_rresp = 0; dut->s4_rlast = 0; dut->s4_rvalid = 0;
    set_wdata(dut->s4_rdata, Word128{0,0,0,0});
    dut->rst = 1;
    for (int i = 0; i < 4; i++) { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); }
    dut->rst = 0;
    dut->cpu_overlay_reset = 0;
    tick();
}

// ─── Slave BFM per-cycle ────────────────────────────────────────────────
// S0 (DDR).
static void step_slave_s0() {
    // Default: ready, no response valid (state updates below).
    // AW accept — honour optional stall counter (task #116 backpressure test).
    if (!s0_bfm.aw_busy && s0_stall_aw <= 0) {
        dut->s0_awready = 1;
        if (dut->s0_awvalid && dut->s0_awready) {
            s0_bfm.aw_id   = dut->s0_awid;
            s0_bfm.aw_addr = dut->s0_awaddr;
            s0_bfm.aw_len  = dut->s0_awlen;
            s0_bfm.aw_busy = true;
            s0_bfm.w_beats_left = (int)dut->s0_awlen + 1;
            s0_bfm.w_cur_addr   = dut->s0_awaddr;
            s0_bfm.seen_aw_count[dut->s0_awaddr]++;
            s0_aw_capture_cycle = (long)sim_time;
            // Early-W mode: a single beat that arrived before this AW
            // (async-bridge style) applies now and completes the burst.
            if (s0_early_w_have) {
                s0_mem.write_masked(s0_bfm.aw_addr, s0_early_w_data,
                                    s0_early_w_strb);
                s0_early_w_have = false;
                s0_bfm.w_beats_left = 0;
                s0_bfm.b_pending = true;
                s0_bfm.b_id      = s0_bfm.aw_id;
                s0_bfm.aw_busy   = false;
            }
        }
    } else {
        dut->s0_awready = 0;
        if (s0_stall_aw > 0) s0_stall_aw--;
    }

    // W accept — if we have an open AW and no B pending
    if (s0_bfm.aw_busy && s0_bfm.w_beats_left > 0 && !s0_bfm.b_pending && s0_stall_w <= 0) {
        dut->s0_wready = 1;
        if (dut->s0_wvalid && dut->s0_wready) {
            Word128 d = get_wdata(dut->s0_wdata);
            // WLAST/filler audit (GAP 1) — see SlaveBfm's field comments.
            if (((bool)dut->s0_wlast) != (s0_bfm.w_beats_left == 1))
                s0_bfm.w_wlast_errors++;
            if (dut->s0_wstrb == 0) s0_bfm.w_pad_beats++;
            s0_bfm.w_beats_seen++;
            s0_mem.write_masked(s0_bfm.w_cur_addr, d, dut->s0_wstrb);
            s0_bfm.w_cur_addr += 16;
            s0_bfm.w_beats_left--;
            if (s0_first_w_cycle < 0) s0_first_w_cycle = (long)sim_time;
            if (s0_bfm.w_beats_left == 0) {
                s0_bfm.b_pending = true;
                s0_bfm.b_id      = s0_bfm.aw_id;
                s0_bfm.aw_busy   = false;
            }
        }
    } else if (s0_early_w_accept && !s0_bfm.aw_busy && !s0_early_w_have &&
               !s0_bfm.b_pending && s0_stall_w <= 0) {
        // Async-bridge-style early W FIFO (single-beat): accept the W
        // beat even though the AW has not been accepted yet.
        dut->s0_wready = 1;
        if (dut->s0_wvalid && dut->s0_wready) {
            s0_early_w_data = get_wdata(dut->s0_wdata);
            s0_early_w_strb = dut->s0_wstrb;
            s0_early_w_have = true;
            if (s0_first_w_cycle < 0) s0_first_w_cycle = (long)sim_time;
        }
    } else {
        dut->s0_wready = 0;
        if (s0_stall_w > 0) s0_stall_w--;
    }

    // B response
    if (s0_bfm.b_pending) {
        dut->s0_bvalid = 1;
        dut->s0_bid    = s0_bfm.b_id;
        dut->s0_bresp  = 0; // OKAY
        if (dut->s0_bready && dut->s0_bvalid) {
            s0_bfm.b_pending = false;
        }
    } else {
        dut->s0_bvalid = 0;
    }

    // AR accept
    if (!s0_bfm.ar_busy) {
        dut->s0_arready = 1;
        if (dut->s0_arvalid && dut->s0_arready) {
            s0_bfm.ar_id   = dut->s0_arid;
            s0_bfm.ar_addr = dut->s0_araddr;
            s0_bfm.ar_len  = dut->s0_arlen;
            s0_bfm.ar_busy = true;
            s0_bfm.r_beats_left = (int)dut->s0_arlen + 1;
            s0_bfm.r_beats_delivered = 0;
            s0_bfm.seen_ar_count[dut->s0_araddr]++;
            s0_ar_capture_cycle = (long)sim_time;
        }
    } else {
        dut->s0_arready = 0;
    }

    // R response.  Task-review mid-burst-timeout scenarios: once
    // s0_r_swallow_after beats of the CURRENT AR have been delivered,
    // stop driving RVALID for the rest of that burst (the AR stays
    // "accepted" — ar_busy true — so the xbar genuinely sits in
    // RS_WAIT_R with some real data already through, not RS_WAIT_SLV_AR).
    bool s0_r_wedged = (s0_r_swallow_after >= 0) &&
                       (s0_bfm.r_beats_delivered >= s0_r_swallow_after);
    if (s0_bfm.ar_busy && s0_bfm.r_beats_left > 0 && !s0_r_wedged) {
        dut->s0_rvalid = 1;
        dut->s0_rid    = s0_bfm.ar_id;
        dut->s0_rresp  = 0;
        dut->s0_rlast  = (s0_bfm.r_beats_left == 1);
        Word128 d = s0_mem.read(s0_bfm.ar_addr);
        set_wdata(dut->s0_rdata, d);
        if (dut->s0_rready && dut->s0_rvalid) {
            if (s0_first_r_hs_cycle < 0) s0_first_r_hs_cycle = (long)sim_time;
            s0_bfm.ar_addr += 16;
            s0_bfm.r_beats_left--;
            s0_bfm.r_beats_delivered++;
            if (s0_bfm.r_beats_left == 0) s0_bfm.ar_busy = false;
        }
    } else {
        dut->s0_rvalid = 0;
        dut->s0_rlast  = 0;
    }
}

static void step_slave_s1() {
    if (s1_in_reset) {
        // A bridge in reset does not hold its AXI slave port off -- its
        // ready lines come out of reset at their reset value and it
        // swallows whatever arrives without recording it.  That is the
        // hazard: the crossbar sees a clean handshake and the transaction
        // ceases to exist.
        dut->s1_awready = 1; dut->s1_wready = 1; dut->s1_arready = 1;
        dut->s1_bvalid = 0;  dut->s1_rvalid = 0; dut->s1_rlast = 0;
        if (dut->s1_awvalid) s1_reset_swallowed_aw++;
        if (dut->s1_arvalid) s1_reset_swallowed_ar++;
        return;
    }
    if (!s1_bfm.aw_busy) {
        dut->s1_awready = 1;
        if (dut->s1_awvalid && dut->s1_awready) {
            s1_bfm.aw_id   = dut->s1_awid;
            s1_bfm.aw_addr = dut->s1_awaddr;
            s1_bfm.aw_len  = dut->s1_awlen;
            s1_bfm.aw_busy = true;
            s1_bfm.w_beats_left = (int)dut->s1_awlen + 1;
            s1_bfm.w_cur_addr   = dut->s1_awaddr;
            s1_bfm.seen_aw_count[dut->s1_awaddr]++;
        }
    } else {
        dut->s1_awready = 0;
    }
    if (s1_bfm.aw_busy && s1_bfm.w_beats_left > 0 && !s1_bfm.b_pending) {
        dut->s1_wready = 1;
        if (dut->s1_wvalid && dut->s1_wready) {
            Word128 d = get_wdata(dut->s1_wdata);
            s1_mem.write_masked(s1_bfm.w_cur_addr, d, dut->s1_wstrb);
            s1_bfm.w_cur_addr += 16;
            s1_bfm.w_beats_left--;
            if (s1_bfm.w_beats_left == 0) {
                s1_bfm.b_pending = true;
                s1_bfm.b_id      = s1_bfm.aw_id;
                s1_bfm.aw_busy   = false;
            }
        }
    } else {
        dut->s1_wready = 0;
    }
    if (s1_bfm.b_pending && !s1_swallow_b) {
        dut->s1_bvalid = 1;
        dut->s1_bid    = s1_bfm.b_id;
        dut->s1_bresp  = 0;
        if (dut->s1_bready && dut->s1_bvalid) s1_bfm.b_pending = false;
    } else {
        // Swallowed: leave b_pending latched forever (the real slave
        // "thinks" it still owes a B) — this is exactly the late/stale
        // response the xbar's poison flag must make harmless once the
        // watchdog gives up on this transaction.
        dut->s1_bvalid = 0;
    }
    if (!s1_bfm.ar_busy && !s1_swallow_ar) {
        dut->s1_arready = 1;
        if (dut->s1_arvalid && dut->s1_arready) {
            s1_bfm.ar_id   = dut->s1_arid;
            s1_bfm.ar_addr = dut->s1_araddr;
            s1_bfm.ar_len  = dut->s1_arlen;
            s1_bfm.ar_busy = true;
            s1_bfm.r_beats_left = (int)dut->s1_arlen + 1;
            s1_bfm.seen_ar_count[dut->s1_araddr]++;
        }
    } else {
        dut->s1_arready = 0;
    }
    if (s1_bfm.ar_busy && s1_bfm.r_beats_left > 0) {
        dut->s1_rvalid = 1;
        dut->s1_rid    = s1_bfm.ar_id;
        dut->s1_rresp  = 0;
        dut->s1_rlast  = (s1_bfm.r_beats_left == 1);
        Word128 d = s1_mem.read(s1_bfm.ar_addr);
        set_wdata(dut->s1_rdata, d);
        if (dut->s1_rready && dut->s1_rvalid) {
            s1_bfm.ar_addr += 16;
            s1_bfm.r_beats_left--;
            if (s1_bfm.r_beats_left == 0) s1_bfm.ar_busy = false;
        }
    } else {
        dut->s1_rvalid = 0;
        dut->s1_rlast  = 0;
    }
}

// Slave 2 (DMA config) — single-beat BFM, mirrors S1 structure.  Task
// review: also carries s2_stall_aw (permanent-stall style, mirrors
// s0_stall_aw) and s2_swallow_ar (never accept AR) hooks so the
// slv_flush / read-side-watchdog / poison-clear scenarios have a Lite
// slave to target that scenario 30's S1 write-watchdog test never
// touches (keeping the two test groups' poison state independent).
static void step_slave_s2() {
    if (!s2_bfm.aw_busy && s2_stall_aw <= 0) {
        dut->s2_awready = 1;
        if (dut->s2_awvalid && dut->s2_awready) {
            s2_bfm.aw_id   = dut->s2_awid;
            s2_bfm.aw_addr = dut->s2_awaddr;
            s2_bfm.aw_len  = dut->s2_awlen;
            s2_bfm.aw_busy = true;
            s2_bfm.w_beats_left = (int)dut->s2_awlen + 1;
            s2_bfm.w_cur_addr   = dut->s2_awaddr;
            s2_bfm.seen_aw_count[dut->s2_awaddr]++;
        }
    } else {
        dut->s2_awready = 0;
        if (s2_stall_aw > 0) s2_stall_aw--;
    }
    if (s2_bfm.aw_busy && s2_bfm.w_beats_left > 0 && !s2_bfm.b_pending) {
        dut->s2_wready = 1;
        if (dut->s2_wvalid && dut->s2_wready) {
            Word128 d = get_wdata(dut->s2_wdata);
            s2_mem.write_masked(s2_bfm.w_cur_addr, d, dut->s2_wstrb);
            s2_bfm.w_cur_addr += 16;
            s2_bfm.w_beats_left--;
            if (s2_bfm.w_beats_left == 0) {
                s2_bfm.b_pending = true;
                s2_bfm.b_id      = s2_bfm.aw_id;
                s2_bfm.aw_busy   = false;
            }
        }
    } else {
        dut->s2_wready = 0;
    }
    if (s2_bfm.b_pending && !s2_swallow_b) {
        dut->s2_bvalid = 1;
        dut->s2_bid    = s2_bfm.b_id;
        dut->s2_bresp  = 0;
        if (dut->s2_bready && dut->s2_bvalid) s2_bfm.b_pending = false;
    } else {
        dut->s2_bvalid = 0;
    }
    if (!s2_bfm.ar_busy && !s2_swallow_ar) {
        dut->s2_arready = 1;
        if (dut->s2_arvalid && dut->s2_arready) {
            s2_bfm.ar_id   = dut->s2_arid;
            s2_bfm.ar_addr = dut->s2_araddr;
            s2_bfm.ar_len  = dut->s2_arlen;
            s2_bfm.ar_busy = true;
            s2_bfm.r_beats_left = (int)dut->s2_arlen + 1;
            s2_bfm.seen_ar_count[dut->s2_araddr]++;
        }
    } else {
        dut->s2_arready = 0;
    }
    if (s2_bfm.ar_busy && s2_bfm.r_beats_left > 0) {
        dut->s2_rvalid = 1;
        dut->s2_rid    = s2_bfm.ar_id;
        dut->s2_rresp  = 0;
        dut->s2_rlast  = (s2_bfm.r_beats_left == 1);
        Word128 d = s2_mem.read(s2_bfm.ar_addr);
        set_wdata(dut->s2_rdata, d);
        if (dut->s2_rready && dut->s2_rvalid) {
            s2_bfm.ar_addr += 16;
            s2_bfm.r_beats_left--;
            if (s2_bfm.r_beats_left == 0) s2_bfm.ar_busy = false;
        }
    } else {
        dut->s2_rvalid = 0;
        dut->s2_rlast  = 0;
    }
}

// Slave 3 (VRAM) — single-port BFM.  The xbar already strips VRAM_BASE
// so we store writes under the zero-based byte offset — matches what
// rtl/sys/vram.v sees in real integration.  Added for task #147.
static void step_slave_s3() {
    if (!s3_bfm.aw_busy) {
        dut->s3_awready = 1;
        if (dut->s3_awvalid && dut->s3_awready) {
            s3_bfm.aw_id   = dut->s3_awid;
            s3_bfm.aw_addr = dut->s3_awaddr;
            s3_bfm.aw_len  = dut->s3_awlen;
            s3_bfm.aw_busy = true;
            s3_bfm.w_beats_left = (int)dut->s3_awlen + 1;
            s3_bfm.w_cur_addr   = dut->s3_awaddr;
            s3_bfm.seen_aw_count[dut->s3_awaddr]++;
        }
    } else {
        dut->s3_awready = 0;
    }
    if (s3_bfm.aw_busy && s3_bfm.w_beats_left > 0 && !s3_bfm.b_pending) {
        dut->s3_wready = 1;
        if (dut->s3_wvalid && dut->s3_wready) {
            Word128 d = get_wdata(dut->s3_wdata);
            s3_mem.write_masked(s3_bfm.w_cur_addr, d, dut->s3_wstrb);
            s3_bfm.w_cur_addr += 16;
            s3_bfm.w_beats_left--;
            if (s3_bfm.w_beats_left == 0) {
                s3_bfm.b_pending = true;
                s3_bfm.b_id      = s3_bfm.aw_id;
                s3_bfm.aw_busy   = false;
            }
        }
    } else {
        dut->s3_wready = 0;
    }
    if (s3_bfm.b_pending && !s3_swallow_b) {
        dut->s3_bvalid = 1;
        dut->s3_bid    = s3_bfm.b_id;
        dut->s3_bresp  = 0;
        if (dut->s3_bready && dut->s3_bvalid) s3_bfm.b_pending = false;
    } else {
        dut->s3_bvalid = 0;
    }
    if (s3_r_multi) {
        // Queued, multi-outstanding read channel (backlog item 2).
        const size_t depth = 4;
        bool room = (s3_ar_q.size() + (s3_r_cur_busy ? 1u : 0u)) < depth;
        dut->s3_arready = room ? 1 : 0;
        if (!s3_r_cur_busy && !s3_r_hold && !s3_ar_q.empty()) {
            if (s3_r_lifo) { s3_r_cur = s3_ar_q.back();  s3_ar_q.pop_back(); }
            else           { s3_r_cur = s3_ar_q.front(); s3_ar_q.pop_front(); }
            s3_r_cur_busy = true;
            s3_r_cur_left = (int)s3_r_cur.len + 1;
            s3_r_cur_addr = s3_r_cur.addr;
        }
        const bool driving_r = s3_r_cur_busy && !s3_r_hold;
        if (driving_r) {
            dut->s3_rvalid = 1;
            dut->s3_rid    = s3_r_cur.id;
            dut->s3_rresp  = 0;
            dut->s3_rlast  = (s3_r_cur_left == 1);
            set_wdata(dut->s3_rdata, s3_mem.read(s3_r_cur_addr));
        } else {
            dut->s3_rvalid = 0;
            dut->s3_rlast  = 0;
        }
        // The quarantine's RREADY is combinational in s3_rvalid/s3_rid, so it
        // has to be SETTLED against the beat just driven before the handshake
        // can be sampled.  The other slave BFMs get away with reading the
        // pre-eval value only because their RREADY stays high for a whole
        // burst; this one's is asserted per-beat by tag, so a one-cycle-stale
        // read would silently miss every handshake.
        dut->eval();
        if (dut->s3_arvalid && room) {
            s3_ar_q.push_back(S3ArEntry{(uint32_t)dut->s3_arid,
                                        (uint32_t)dut->s3_araddr,
                                        (uint8_t)dut->s3_arlen});
            s3_bfm.seen_ar_count[dut->s3_araddr]++;
        }
        if (driving_r && dut->s3_rready) {
            s3_r_cur_addr += 16;
            s3_r_cur_left--;
            if (s3_r_cur_left == 0) { s3_r_cur_busy = false; s3_r_bursts_done++; }
        }
        return;
    }
    if (!s3_bfm.ar_busy) {
        dut->s3_arready = 1;
        if (dut->s3_arvalid && dut->s3_arready) {
            s3_bfm.ar_id   = dut->s3_arid;
            s3_bfm.ar_addr = dut->s3_araddr;
            s3_bfm.ar_len  = dut->s3_arlen;
            s3_bfm.ar_busy = true;
            s3_bfm.r_beats_left = (int)dut->s3_arlen + 1;
            s3_bfm.r_beats_delivered = 0;
            s3_bfm.seen_ar_count[dut->s3_araddr]++;
        }
    } else {
        dut->s3_arready = 0;
    }
    bool s3_r_wedged = (s3_r_swallow_after >= 0) &&
                       (s3_bfm.r_beats_delivered >= s3_r_swallow_after);
    if (s3_bfm.ar_busy && s3_bfm.r_beats_left > 0 && !s3_r_wedged) {
        dut->s3_rvalid = 1;
        dut->s3_rid    = s3_bfm.ar_id;
        dut->s3_rresp  = 0;
        dut->s3_rlast  = (s3_bfm.r_beats_left == 1);
        Word128 d = s3_mem.read(s3_bfm.ar_addr);
        set_wdata(dut->s3_rdata, d);
        if (dut->s3_rready && dut->s3_rvalid) {
            s3_bfm.ar_addr += 16;
            s3_bfm.r_beats_left--;
            s3_bfm.r_beats_delivered++;
            if (s3_bfm.r_beats_left == 0) s3_bfm.ar_busy = false;
        }
    } else {
        dut->s3_rvalid = 0;
        dut->s3_rlast  = 0;
    }
}

// Slave 4 (DAFB registers).  The xbar strips DAFB_BASE before forwarding,
// so this BFM stores zero-based register-window offsets.
static void step_slave_s4() {
    if (!s4_bfm.aw_busy) {
        dut->s4_awready = 1;
        if (dut->s4_awvalid && dut->s4_awready) {
            s4_bfm.aw_id   = dut->s4_awid;
            s4_bfm.aw_addr = dut->s4_awaddr;
            s4_bfm.aw_len  = dut->s4_awlen;
            s4_bfm.aw_busy = true;
            s4_bfm.w_beats_left = (int)dut->s4_awlen + 1;
            s4_bfm.w_cur_addr   = dut->s4_awaddr;
            s4_bfm.seen_aw_count[dut->s4_awaddr]++;
        }
    } else {
        dut->s4_awready = 0;
    }
    if (s4_bfm.aw_busy && s4_bfm.w_beats_left > 0 && !s4_bfm.b_pending) {
        dut->s4_wready = 1;
        if (dut->s4_wvalid && dut->s4_wready) {
            Word128 d = get_wdata(dut->s4_wdata);
            s4_mem.write_masked(s4_bfm.w_cur_addr, d, dut->s4_wstrb);
            s4_bfm.w_cur_addr += 16;
            s4_bfm.w_beats_left--;
            if (s4_bfm.w_beats_left == 0) {
                s4_bfm.b_pending = true;
                s4_bfm.b_id      = s4_bfm.aw_id;
                s4_bfm.aw_busy   = false;
            }
        }
    } else {
        dut->s4_wready = 0;
    }
    if (s4_bfm.b_pending) {
        dut->s4_bvalid = 1;
        dut->s4_bid    = s4_bfm.b_id;
        dut->s4_bresp  = 0;
        if (dut->s4_bready && dut->s4_bvalid) s4_bfm.b_pending = false;
    } else {
        dut->s4_bvalid = 0;
    }
    if (!s4_bfm.ar_busy) {
        dut->s4_arready = 1;
        if (dut->s4_arvalid && dut->s4_arready) {
            s4_bfm.ar_id   = dut->s4_arid;
            s4_bfm.ar_addr = dut->s4_araddr;
            s4_bfm.ar_len  = dut->s4_arlen;
            s4_bfm.ar_busy = true;
            s4_bfm.r_beats_left = (int)dut->s4_arlen + 1;
            s4_bfm.seen_ar_count[dut->s4_araddr]++;
        }
    } else {
        dut->s4_arready = 0;
    }
    if (s4_bfm.ar_busy && s4_bfm.r_beats_left > 0) {
        dut->s4_rvalid = 1;
        dut->s4_rid    = s4_bfm.ar_id;
        dut->s4_rresp  = 0;
        dut->s4_rlast  = (s4_bfm.r_beats_left == 1);
        Word128 d = s4_mem.read(s4_bfm.ar_addr);
        set_wdata(dut->s4_rdata, d);
        if (dut->s4_rready && dut->s4_rvalid) {
            s4_bfm.ar_addr += 16;
            s4_bfm.r_beats_left--;
            if (s4_bfm.r_beats_left == 0) s4_bfm.ar_busy = false;
        }
    } else {
        dut->s4_rvalid = 0;
        dut->s4_rlast  = 0;
    }
}

// ─── Master-side observation ────────────────────────────────────────────
// Sample B and R handshakes, update master expectation queues.

static void observe_b(int mi, uint8_t bvalid, uint8_t bready, uint32_t bid, uint32_t bresp) {
    if (bvalid && bready) {
        if (mb[mi].w_expect.empty()) {
            std::printf("[ERROR] M%d got unexpected BVALID (id=%u resp=%u)\n", mi, bid, bresp);
            n_fail++;
        } else {
            auto exp = mb[mi].w_expect.front(); mb[mi].w_expect.pop_front();
            if (bid != exp.first || bresp != exp.second) {
                std::printf("[ERROR] M%d B mismatch: got id=%u resp=%u expected id=%u resp=%u\n",
                            mi, bid, bresp, exp.first, exp.second);
                n_fail++;
            } else {
                if (bresp == 0) mb[mi].w_completed_ok++;
                else            mb[mi].w_completed_errs++;
            }
        }
    }
}
static void observe_r(int mi, uint8_t rvalid, uint8_t rready, uint32_t rid, uint32_t rresp, uint8_t rlast, Word128 rdata) {
    if (rvalid && rready) {
        mb[mi].r_cur.push_back(rdata);
        mb[mi].r_cur_resp.push_back(rresp);
        if (rlast) {
            mb[mi].r_lasts_seen++;   // the absorber's decrement event
            if (mb[mi].r_expect.empty()) {
                std::printf("[ERROR] M%d unexpected RLAST (id=%u resp=%u)\n", mi, rid, rresp);
                n_fail++;
            } else {
                auto exp = mb[mi].r_expect.front(); mb[mi].r_expect.pop_front();
                if (rid != exp.first || rresp != exp.second) {
                    std::printf("[ERROR] M%d R mismatch: got id=%u resp=%u expected id=%u resp=%u\n",
                                mi, rid, rresp, exp.first, exp.second);
                    n_fail++;
                } else {
                    if (rresp == 0) mb[mi].r_completed++;
                    else            mb[mi].r_errors++;
                }
            }
            mb[mi].r_received.push_back(mb[mi].r_cur);
            mb[mi].r_cur.clear();
            mb[mi].r_received_resp.push_back(mb[mi].r_cur_resp);
            mb[mi].r_cur_resp.clear();
        }
    }
}

static void step_masters_observe() {
    observe_b(0, dut->m0_bvalid, dut->m0_bready, dut->m0_bid, dut->m0_bresp);
    observe_b(1, dut->m1_bvalid, dut->m1_bready, dut->m1_bid, dut->m1_bresp);
    observe_b(3, dut->m0b_bvalid, dut->m0b_bready, dut->m0b_bid, dut->m0b_bresp);
    observe_r(0, dut->m0_rvalid, dut->m0_rready, dut->m0_rid, dut->m0_rresp, dut->m0_rlast, get_wdata(dut->m0_rdata));
    observe_r(1, dut->m1_rvalid, dut->m1_rready, dut->m1_rid, dut->m1_rresp, dut->m1_rlast, get_wdata(dut->m1_rdata));
    observe_r(2, dut->m2_rvalid, dut->m2_rready, dut->m2_rid, dut->m2_rresp, dut->m2_rlast, get_wdata(dut->m2_rdata));
}

// ─── M0/boot merge select ─────────────────────────────────────────────
// Drives the xbar's cpu_held_in_reset select input, settling a few
// cycles so the mux + downstream arbiter state see a stable value
// before any transaction is issued against it.
static void set_cpu_held_in_reset(bool held) {
    dut->cpu_held_in_reset = held ? 1 : 0;
    for (int i = 0; i < 4; i++) cycle();
}

// ─── Issue helpers (just set the AW/AR signals; caller drives the bus) ──
// Simple blocking issue: present AW, wait for AWREADY; present W beats,
// wait for WREADY per beat; then drop valids.
//
// issue_write<M>/issue_read<M> only cover the masters that keep a
// dedicated top-level port after the 2026-07-16 master-count reduction:
// M=0 (CPU LSU), M=1 (host debug) for writes; M=0/1/2 (CPU LSU / host
// debug / CPU IF) for reads.  Boot FSM writes go through the separate
// issue_write_boot() below, since it shares M0's physical port via the
// cpu_held_in_reset mux rather than having its own top-level index.

template <int M>
static void issue_write(uint32_t id, uint32_t addr, uint8_t len, uint8_t size,
                        const std::vector<Word128>& beats, uint32_t expect_bresp,
                        uint16_t wstrb = 0xFFFF) {
    static_assert(M == 0 || M == 1, "issue_write<M> only supports M0 (CPU LSU) and M1 (host debug)");
    // Enqueue expectation first (so observe can match it any cycle).
    mb[M].w_expect.push_back({id, expect_bresp});

    // Set AW
    if constexpr (M == 0) {
        dut->m0_awid = id; dut->m0_awaddr = addr; dut->m0_awlen = len;
        dut->m0_awsize = size; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    } else {  // M == 1
        dut->m1_awid = id; dut->m1_awaddr = addr; dut->m1_awlen = len;
        dut->m1_awsize = size; dut->m1_awburst = 1; dut->m1_awvalid = 1;
    }

    for (int i = 0; i < 500; i++) {
        dut->eval();
        uint8_t ready = (M==0) ? dut->m0_awready : dut->m1_awready;
        if (ready) { cycle(); break; }
        cycle();
    }
    if constexpr (M == 0) dut->m0_awvalid = 0;
    else                  dut->m1_awvalid = 0;

    for (size_t b = 0; b < beats.size(); b++) {
        bool is_last = (b == beats.size() - 1);
        if constexpr (M == 0) {
            set_wdata(dut->m0_wdata, beats[b]);
            dut->m0_wstrb = wstrb;
            dut->m0_wlast = is_last ? 1 : 0;
            dut->m0_wvalid = 1;
        } else {
            set_wdata(dut->m1_wdata, beats[b]);
            dut->m1_wstrb = wstrb;
            dut->m1_wlast = is_last ? 1 : 0;
            dut->m1_wvalid = 1;
        }
        for (int i = 0; i < 500; i++) {
            dut->eval();
            uint8_t ready = (M==0) ? dut->m0_wready : dut->m1_wready;
            if (ready) { cycle(); break; }
            cycle();
        }
    }
    if constexpr (M == 0) { dut->m0_wvalid = 0; dut->m0_wlast = 0; }
    else                  { dut->m1_wvalid = 0; dut->m1_wlast = 0; }

    for (int i = 0; i < 100; i++) cycle();
}

// Boot FSM write, via the m0b_* sub-port merged onto M0's physical port.
// Caller is responsible for having called set_cpu_held_in_reset(true)
// first — this helper does not touch the select itself, since some
// scenarios deliberately want to observe what happens when it's NOT
// selected (see the adversarial-exclusivity scenario).
static void issue_write_boot(uint32_t id, uint32_t addr, uint8_t len, uint8_t size,
                             const std::vector<Word128>& beats, uint32_t expect_bresp,
                             uint16_t wstrb = 0xFFFF) {
    mb[3].w_expect.push_back({id, expect_bresp});
    dut->m0b_awid = id; dut->m0b_awaddr = addr; dut->m0b_awlen = len;
    dut->m0b_awsize = size; dut->m0b_awburst = 1; dut->m0b_awvalid = 1;
    for (int i = 0; i < 500; i++) {
        dut->eval();
        if (dut->m0b_awready) { cycle(); break; }
        cycle();
    }
    dut->m0b_awvalid = 0;

    for (size_t b = 0; b < beats.size(); b++) {
        bool is_last = (b == beats.size() - 1);
        set_wdata(dut->m0b_wdata, beats[b]);
        dut->m0b_wstrb = wstrb;
        dut->m0b_wlast = is_last ? 1 : 0;
        dut->m0b_wvalid = 1;
        for (int i = 0; i < 500; i++) {
            dut->eval();
            if (dut->m0b_wready) { cycle(); break; }
            cycle();
        }
    }
    dut->m0b_wvalid = 0; dut->m0b_wlast = 0;

    for (int i = 0; i < 100; i++) cycle();
}

template <int M>
static void issue_read(uint32_t id, uint32_t addr, uint8_t len, uint8_t size,
                       uint32_t expect_rresp) {
    static_assert(M == 0 || M == 1 || M == 2,
                  "issue_read<M> only supports M0 (CPU LSU), M1 (host debug), M2 (CPU IF)");
    mb[M].r_expect.push_back({id, expect_rresp});
    if constexpr (M == 0) {
        dut->m0_arid = id; dut->m0_araddr = addr; dut->m0_arlen = len;
        dut->m0_arsize = size; dut->m0_arburst = 1; dut->m0_arvalid = 1;
    } else if constexpr (M == 1) {
        dut->m1_arid = id; dut->m1_araddr = addr; dut->m1_arlen = len;
        dut->m1_arsize = size; dut->m1_arburst = 1; dut->m1_arvalid = 1;
    } else {  // M == 2
        dut->m2_arid = id; dut->m2_araddr = addr; dut->m2_arlen = len;
        dut->m2_arsize = size; dut->m2_arburst = 1; dut->m2_arvalid = 1;
    }
    for (int i = 0; i < 500; i++) {
        dut->eval();
        uint8_t ready =
            (M==0) ? dut->m0_arready :
            (M==1) ? dut->m1_arready : dut->m2_arready;
        if (ready) { cycle(); break; }
        cycle();
    }
    if constexpr (M == 0) dut->m0_arvalid = 0;
    else if constexpr (M == 1) dut->m1_arvalid = 0;
    else                       dut->m2_arvalid = 0;

    for (int i = 0; i < 400; i++) cycle();
}

// ─── Scenarios ──────────────────────────────────────────────────────────
static void check(const char* name, bool ok) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) n_pass++; else n_fail++;
}

static void scenario_1_cpu_read_ram() {
    std::printf("\n=== Scenario 1: M0 CPU reads from RAM ===\n");
    // Pre-populate DDR memory at flattened RAM offset 0x00000040.
    Word128 v = w128_from_u64(0xCAFEBABE12345678ULL, 0xDEADBEEF87654321ULL);
    s0_mem.write(0x00000040, v);
    issue_read<0>(/*id*/ 0x3, /*addr*/ 0x00000040, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    bool got_ok = !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
                  w128_eq(mb[0].r_received.back()[0], v);
    check("M0 RAM read returns expected data via DDR path", got_ok);
    check("RESP=OKAY on RAM read", mb[0].r_errors == 0 && mb[0].r_completed >= 1);
}

static void scenario_2_cpu_write_rom_dropped() {
    std::printf("\n=== Scenario 2: M0 writes to ROM → OKAY drop ===\n");
    int prev_ok      = mb[0].w_completed_ok;
    // Write something at 0x40001000 (ROM region).  Should never reach DDR.
    // Clear the slot in DDR first.
    s0_mem.write(DDR_ROM_OFF + 0x00001000u, Word128{0,0,0,0});  // flattened ROM offset
    Word128 poison = w128_from_u64(0xBAADF00DBAADF00DULL, 0xBAADF00DBAADF00DULL);
    issue_write<0>(/*id*/ 0x7, /*addr*/ 0x40001000, /*len*/ 0, /*size*/ 4, {poison},
                   /*expect_bresp*/ 0 /*OKAY*/);
    check("M0 ROM write returns OKAY",
          mb[0].w_completed_ok == prev_ok + 1);
    Word128 still = s0_mem.read(DDR_ROM_OFF + 0x00001000u);
    check("ROM region in DDR untouched by M0 write",
          w128_eq(still, Word128{0,0,0,0}));
}

static void scenario_3_boot_write_rom_ok() {
    std::printf("\n=== Scenario 3: boot FSM (merged onto M0) writes to ROM → OKAY ===\n");
    set_cpu_held_in_reset(true);
    int prev_ok = mb[3].w_completed_ok;
    Word128 payload = w128_from_u64(0xF00DCAFEDEADBEEFULL, 0x0123456789ABCDEFULL);
    issue_write_boot(/*id*/ 0xA, /*addr*/ 0x40010000, /*len*/ 0, /*size*/ 4, {payload}, /*expect_bresp*/ 0);
    bool ok = (mb[3].w_completed_ok == prev_ok + 1);
    check("boot FSM ROM write returns OKAY", ok);
    // flattened ROM offset = DDR_ROM_OFF + (0x40010000 - 0x40000000)
    Word128 got = s0_mem.read(DDR_ROM_OFF + 0x00010000u);
    check("Boot FSM write reached DDR at flattened ROM offset",
          w128_eq(got, payload));
    set_cpu_held_in_reset(false);
}

// Scenario 3c (added for the 2026-07-16 master-count reduction): while
// cpu_held_in_reset=1, CPU LSU (M0) must be structurally blocked from
// getting an AW grant — not "happens to not try" (real boot_fsm/CPU
// reset sequencing already guarantees CPU LSU is quiescent during this
// window, see axi_xbar.v's "M0/boot merge" header note), but genuinely
// unreachable at the wire level: the mux never even samples m0_awvalid
// while selected to boot.  We drive m0_awvalid high directly (something
// the real integration should never do, but the xbar must still be safe
// against) and confirm m0_awready never asserts and DDR never sees the
// address.
static void scenario_3c_boot_merge_cpu_blocked_during_reset() {
    std::printf("\n=== Scenario 3c: CPU LSU AW blocked while cpu_held_in_reset=1 ===\n");
    set_cpu_held_in_reset(true);
    uint32_t addr = 0x00030000u;
    s0_mem.write(addr, Word128{0,0,0,0});
    dut->m0_awid = 0x9; dut->m0_awaddr = addr; dut->m0_awlen = 0;
    dut->m0_awsize = 4; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    bool ever_ready = false;
    for (int i = 0; i < 50; i++) {
        dut->eval();
        if (dut->m0_awready) ever_ready = true;
        cycle();
    }
    dut->m0_awvalid = 0;
    for (int i = 0; i < 20; i++) cycle();
    check("m0_awready never asserted while cpu_held_in_reset=1", !ever_ready);
    check("DDR S0 never saw the blocked CPU-LSU AW address",
          s0_bfm.seen_aw_count[addr] == 0);
    set_cpu_held_in_reset(false);
}

// Scenario 3d: after release (cpu_held_in_reset -> 0), CPU LSU reads and
// writes must reach the same physical M0 port normally — the merge must
// not leave any residue from the boot-FSM window.
static void scenario_3d_boot_merge_cpu_after_release() {
    std::printf("\n=== Scenario 3d: CPU LSU read/write works normally after release ===\n");
    set_cpu_held_in_reset(true);
    set_cpu_held_in_reset(false);  // simulate the boot->CPU handoff
    int prev_wok = mb[0].w_completed_ok;
    uint32_t addr = 0x00031000u;
    Word128 payload = w128_from_u64(0xC0FFEE00C0FFEE00ULL, 0x1234ABCD1234ABCDULL);
    issue_write<0>(/*id*/ 0x1, addr, 0, 4, {payload}, /*expect_bresp*/ 0);
    check("post-release CPU LSU write completes OKAY",
          mb[0].w_completed_ok == prev_wok + 1);
    check("post-release CPU LSU write landed at DDR", w128_eq(s0_mem.read(addr), payload));

    int prev_rok = mb[0].r_completed;
    issue_read<0>(/*id*/ 0x2, addr, 0, 4, /*rresp*/ 0);
    bool got = !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
               w128_eq(mb[0].r_received.back()[0], payload);
    check("post-release CPU LSU read returns what it wrote",
          mb[0].r_completed == prev_rok + 1 && got);
}

// Scenario 3e: adversarial exclusivity proof.  Drive BOTH m0_awvalid
// (CPU LSU) and m0b_awvalid (boot FSM) simultaneously, with DIFFERENT
// addresses/data, for several cycles under each select state.  The xbar
// is a plain 2:1 mux (not an arbiter) — only the selected side's request
// can ever reach DDR, regardless of what the deselected side is doing.
// This is a structural guarantee independent of the reset-sequencing
// argument documented in axi_xbar.v: even if two misbehaving upstream
// blocks both asserted valid at once, the two transactions could never
// blend into one forwarded request.
static void scenario_3e_boot_merge_adversarial_exclusivity() {
    std::printf("\n=== Scenario 3e: adversarial M0/boot simultaneous-valid exclusivity ===\n");
    uint32_t cpu_addr  = 0x00032000u;
    uint32_t boot_addr = 0x40030000u;  // ROM region — only boot may write it
    s0_mem.write(cpu_addr, Word128{0,0,0,0});
    s0_mem.write(DDR_ROM_OFF + 0x00030000u, Word128{0,0,0,0});
    Word128 cpu_payload  = w128_from_u64(0x1111111122222222ULL);
    Word128 boot_payload = w128_from_u64(0x3333333344444444ULL);

    // -- select=boot: assert BOTH AW valids simultaneously for a few
    // cycles (proving the deselected side's valid can dangle without
    // being sampled), then run each side's handshake to completion as a
    // single-shot transaction (deassert valid the cycle after its own
    // grant, exactly like a compliant AXI master) so neither side can
    // spuriously re-fire while the other's multi-cycle W burst drains.
    set_cpu_held_in_reset(true);
    dut->m0_awid = 0x1; dut->m0_awaddr = cpu_addr; dut->m0_awlen = 0;
    dut->m0_awsize = 4; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    dut->m0b_awid = 0x2; dut->m0b_awaddr = boot_addr; dut->m0b_awlen = 0;
    dut->m0b_awsize = 4; dut->m0b_awburst = 1; dut->m0b_awvalid = 1;
    mb[3].w_expect.push_back({0x2, 0});
    bool cpu_ever_awready = false;
    for (int i = 0; i < 20; i++) {
        dut->eval();
        if (dut->m0_awready) cpu_ever_awready = true;
        if (dut->m0b_awready) { cycle(); break; }
        cycle();
    }
    dut->m0_awvalid = 0;   // CPU side never got a grant; drop it regardless.
    dut->m0b_awvalid = 0;  // boot side just got its (single) grant.
    set_wdata(dut->m0b_wdata, boot_payload); dut->m0b_wstrb = 0xFFFF;
    dut->m0b_wlast = 1; dut->m0b_wvalid = 1;
    for (int i = 0; i < 20; i++) {
        dut->eval();
        if (dut->m0b_wready) { cycle(); break; }
        cycle();
    }
    dut->m0b_wvalid = 0; dut->m0b_wlast = 0;
    for (int i = 0; i < 30; i++) cycle();
    check("select=boot: CPU-side awready never asserted", !cpu_ever_awready);
    check("select=boot: boot's write landed at DDR",
          w128_eq(s0_mem.read(DDR_ROM_OFF + 0x00030000u), boot_payload));
    check("select=boot: CPU-side write never reached DDR",
          w128_eq(s0_mem.read(cpu_addr), Word128{0,0,0,0}));
    check("select=boot: DDR never saw an AW at the CPU-side address",
          s0_bfm.seen_aw_count[cpu_addr] == 0);

    // -- select=cpu: same idea, roles reversed.  Boot-side valid is left
    // asserted here to model a misbehaving/stale driver (real boot_fsm
    // is already idle by the time cpu_held_in_reset drops in the real
    // integration — see header note — this is deliberately adversarial).
    set_cpu_held_in_reset(false);
    dut->m0_awid = 0x3; dut->m0_awaddr = cpu_addr; dut->m0_awlen = 0;
    dut->m0_awsize = 4; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    dut->m0b_awid = 0x4; dut->m0b_awaddr = boot_addr + 0x10; dut->m0b_awlen = 0;
    dut->m0b_awsize = 4; dut->m0b_awburst = 1; dut->m0b_awvalid = 1;
    mb[0].w_expect.push_back({0x3, 0});
    bool boot_ever_awready = false;
    for (int i = 0; i < 20; i++) {
        dut->eval();
        if (dut->m0b_awready) boot_ever_awready = true;
        if (dut->m0_awready) { cycle(); break; }
        cycle();
    }
    dut->m0b_awvalid = 0;  // stale boot side never got a grant; drop it.
    dut->m0_awvalid = 0;   // CPU side just got its (single) grant.
    set_wdata(dut->m0_wdata, cpu_payload); dut->m0_wstrb = 0xFFFF;
    dut->m0_wlast = 1; dut->m0_wvalid = 1;
    for (int i = 0; i < 20; i++) {
        dut->eval();
        if (dut->m0_wready) { cycle(); break; }
        cycle();
    }
    dut->m0_wvalid = 0; dut->m0_wlast = 0;
    for (int i = 0; i < 30; i++) cycle();
    check("select=cpu: stale boot-side awready never asserted", !boot_ever_awready);
    check("select=cpu: CPU's write landed at DDR",
          w128_eq(s0_mem.read(cpu_addr), cpu_payload));
    check("select=cpu: stale boot-side write never reached DDR",
          s0_bfm.seen_aw_count[DDR_ROM_OFF + 0x00030010u] == 0);
}

static void scenario_3b_host_debug_write_rom_ok() {
    std::printf("\n=== Scenario 3b: M1 host debug writes to ROM → OKAY ===\n");
    int prev_ok = mb[1].w_completed_ok;
    Word128 payload = w128_from_u64(0x1020304050607080ULL, 0x90A0B0C0D0E0F001ULL);
    issue_write<1>(/*id*/ 0x4, /*addr*/ 0x40020000, /*len*/ 0, /*size*/ 4, {payload}, /*expect_bresp*/ 0);
    bool ok = (mb[1].w_completed_ok == prev_ok + 1);
    check("M1 ROM write returns OKAY for JTAG/XDMA loader", ok);
    // flattened ROM offset = DDR_ROM_OFF + (0x40020000 - 0x40000000)
    Word128 got = s0_mem.read(DDR_ROM_OFF + 0x00020000u);
    check("Host debug write reached DDR at flattened ROM offset",
          w128_eq(got, payload));
}

static void scenario_4_hdmi_read_fb() {
    std::printf("\n=== Scenario 4: M2 CPU IF reads framebuffer → DDR ===\n");
    // framebuffer at 0x60000100 → flattened DDR offset DDR_FB_OFF + 0x100
    Word128 pixels = w128_from_u64(0xAABBCCDDEEFF0011ULL, 0x2233445566778899ULL);
    s0_mem.write(DDR_FB_OFF + 0x00000100u, pixels);
    issue_read<2>(/*id*/ 0x1, /*addr*/ 0x60000100, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    bool got_ok = !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
                  w128_eq(mb[2].r_received.back()[0], pixels);
    check("M2 FB read returns expected pixels via DDR path", got_ok);
    // Confirm the DDR slave actually saw an AR at the flattened address.
    check("DDR S0 saw AR at flattened FB offset",
          s0_bfm.seen_ar_count[DDR_FB_OFF + 0x00000100u] >= 1);
}

// ── KNOWN-RED FINDING (address-decode audit, 2026-08) ────────────────
//   DELIBERATELY NOT CALLED FROM main() — see the note at its call site.
//
// FINDING: the ROM mirror DECODES over the right span but FLATTENS with
// the wrong granularity, so 3 of every 4 MiB of the mirror reads
// uninitialised DDR where MAME (and Q700 silicon) return the ROM.
//
//   Decode  (axi_xbar.v:1096-1099, correct):
//     is_rom_addr = (a & ~(ROM_MIRROR_SIZE-1)) == ROM_MIRROR_BASE
//       ROM_MIRROR_SIZE = 0x1000_0000  =>  ~(SIZE-1) = 0xF000_0000
//       (a & 0xF000_0000) == 0x4000_0000  =>  [0x4000_0000, 0x4FFF_FFFF]
//     — same span as MAME's map(0x40000000,0x400fffff).mirror(0x0ff00000).
//
//   Flatten (axi_xbar.v:1264-1266, the defect):
//     ddr_flatten = ((a - ROM_MIRROR_BASE) & (ROM_SIZE - 1)) + DDR_ROM_OFF
//       ROM_SIZE = 0x0040_0000  =>  mask 0x003F_FFFF   (4 MiB period)
//     MAME's offset is  a & 0x000F_FFFF                (1 MiB period)
//
//   And only 1 MiB is ever loaded: BOOT_ROM_SECTORS = 2048
//   (fpga_top.v:376) x 512 B = 0x0010_0000, written at
//   ROM_BASE_ADDR + (sector<<9) (boot_fsm.v:1392) => DDR
//   [0x4000_0000, 0x400F_FFFF].  DDR [0x4010_0000, 0x403F_FFFF] is
//   written by nothing.
//
//   Net: a ROM read returns real ROM iff (a & 0x0030_0000) == 0, i.e.
//   addr[21:20] == 00 — 64 of the 256 one-MiB blocks in the mirror.
//     0x4010_0000 -> DDR 0x4010_0000 (garbage);  MAME -> ROM byte 0
//     0x4020_0000 -> DDR 0x4020_0000 (garbage);  MAME -> ROM byte 0
//     0x4030_0000 -> DDR 0x4030_0000 (garbage);  MAME -> ROM byte 0
//     0x4040_0000 -> DDR 0x4000_0000 (ROM byte 0) == MAME  ✓
//
// Consequence for software: the ROM appears to wrap with a 4 MiB period
// instead of 1 MiB, and any read into a 0x40{1,2,3}x_xxxx MiB returns
// DRAM contents.  Whether the Q700 ROM / Mac OS actually reads there is
// NOT established — the ROM-size / wrap probe is the obvious candidate.
// Fix would be `& 32'h000F_FFFF` (or driving the mask off the real
// image size) — deliberately NOT applied here; the audit lands the
// evidence first.
//
// SIDE EFFECT, read before enabling: this scenario writes DDR at
// DDR_ROM_OFF + 0, which is the same word the later overlay scenarios
// ("overlay-active M0 low read returns ROM image data", "initial CPUI
// low read returns ROM image data") pre-seed and check.  Enabling the
// call as-is therefore produces 6 failures, not 3 — the 3 real ones
// below plus 3 collateral overlay failures.  When the flatten mask is
// fixed, either move this scenario after the overlay scenarios or
// restore DDR_ROM_OFF+0 to the overlay payload before returning.
static void scenario_rom_mirror_granularity() {
    std::printf("\n=== ROM mirror period: must be 1 MiB like MAME (task #245) ===\n");
    Word128 rom0 = w128_from_u64(0x4A4F4B4552303030ULL, 0x5A5A5A5A5A5A5A5AULL);
    // Byte 0 of the (1 MiB) ROM image, as boot_fsm would have left it.
    s0_mem.write(DDR_ROM_OFF + 0x00000000u, rom0);

    // Sanity: the canonical base and the 4 MiB-aligned copy already work.
    issue_read<2>(/*id*/ 0x2, /*addr*/ 0x40000000, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    check("ROM base 0x4000_0000 returns ROM byte 0",
          !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
          w128_eq(mb[2].r_received.back()[0], rom0));
    issue_read<2>(0x2, 0x40400000, 0, 4, 0);
    check("ROM +4 MiB returns ROM byte 0 (the 4 MiB period we DO have)",
          !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
          w128_eq(mb[2].r_received.back()[0], rom0));

    // MAME-faithful expectation: +1/+2/+3 MiB are ROM mirrors too.
    // These FAIL today — they flatten to DDR 0x4010/20/30_0000, which
    // boot_fsm never writes.
    issue_read<2>(0x2, 0x40100000, 0, 4, 0);
    check("ROM +1 MiB should mirror ROM byte 0 (MAME mirror(0x0ff00000))",
          !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
          w128_eq(mb[2].r_received.back()[0], rom0));
    issue_read<2>(0x2, 0x40200000, 0, 4, 0);
    check("ROM +2 MiB should mirror ROM byte 0",
          !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
          w128_eq(mb[2].r_received.back()[0], rom0));
    issue_read<2>(0x2, 0x40300000, 0, 4, 0);
    check("ROM +3 MiB should mirror ROM byte 0",
          !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
          w128_eq(mb[2].r_received.back()[0], rom0));

    // The flattened address the DDR slave actually saw is the direct
    // evidence.  Fixed behaviour: every +N MiB read folds onto
    // DDR_ROM_OFF + 0, and the unloaded DDR at DDR_ROM_OFF+0x10/20/30_0000
    // is never addressed at all.  (Before task #245 this scenario asserted
    // the OPPOSITE as a bug witness; it is inverted now that the mask is
    // ROM_IMAGE_SIZE.)
    check("all +N MiB mirrors folded onto DDR_ROM_OFF+0",
          s0_bfm.seen_ar_count[DDR_ROM_OFF + 0x00000000u] >= 4);
    check("DDR_ROM_OFF+0x100000 (unloaded DDR) never addressed",
          s0_bfm.seen_ar_count[DDR_ROM_OFF + 0x00100000u] == 0);
    check("DDR_ROM_OFF+0x200000 (unloaded DDR) never addressed",
          s0_bfm.seen_ar_count[DDR_ROM_OFF + 0x00200000u] == 0);
    check("DDR_ROM_OFF+0x300000 (unloaded DDR) never addressed",
          s0_bfm.seen_ar_count[DDR_ROM_OFF + 0x00300000u] == 0);
}

// ── RAM-disk aperture: REMOVED 2026-09-10 ────────────────────────────
// scenario_ramdisk_aperture() proved that 0x7000_0000..0x7FFF_FFFF
// decoded onto S0 (DDR) and flattened to 0x5000_0000 + offset, which is
// how vhdd_ddr's master (M3) and the JTAG host (M1) both reached the
// DDR-backed second SCSI volume.
//
// That volume was deleted on owner directive, and the decode and flatten
// arms went with it, so 0x7xxx_xxxx is unmapped again.  The coverage did
// not disappear with the scenario: scenario_7_decerr and
// scenario_10_decerr_all_unmapped BOTH already read/write 0x7000_0000 and
// require OKAY + 0x00000000 with no slave seeing the transaction, which is
// exactly the pre-aperture behaviour this removal restores.  Those two
// scenarios were passing vacuously while the aperture existed (they check
// the UNFLATTENED address against the S0 monitor); they are load-bearing
// again now.

static void scenario_5_xdma_read_io() {
    std::printf("\n=== Scenario 5: M1 XDMA reads I/O region → S1 ===\n");
    Word128 ioval = w128_from_u64(0xFEEDFACE00000000ULL);
    s1_mem.write(0x50000000, ioval);  // I/O addr passes through to S1 unchanged
    issue_read<1>(/*id*/ 0x5, /*addr*/ 0x50000000, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    bool got_ok = !mb[1].r_received.empty() && mb[1].r_received.back().size() == 1 &&
                  w128_eq(mb[1].r_received.back()[0], ioval);
    check("M1 I/O read returns value via S1 (peripheral bus)", got_ok);
    check("S1 saw AR at 0x50000000", s1_bfm.seen_ar_count[0x50000000] >= 1);
    check("S0 did NOT see an AR at 0x50000000", s0_bfm.seen_ar_count[0x50000000] == 0);
}

static void scenario_6_contention() {
    std::printf("\n=== Scenario 6: Contention — M0 + M1 8× writes to same slave ===\n");
    // Issue 8 back-to-back single-beat writes from M0 and M1 to the DDR slave.
    // We don't want to block both at the same time (our issue helper is blocking),
    // so interleave them per-txn.  Expected: 16 BRESP=OKAY totals, all correct.
    int prev_ok_0 = mb[0].w_completed_ok;
    int prev_ok_1 = mb[1].w_completed_ok;

    for (int k = 0; k < 8; k++) {
        uint32_t addr0 = 0x00000200 + (k * 0x40);  // RAM
        uint32_t addr1 = 0x00010000 + (k * 0x40);
        Word128  d0    = w128_from_u64(0x10 + k, 0x100 + k);
        Word128  d1    = w128_from_u64(0x20 + k, 0x200 + k);
        // Interleave: present M0's AW first; while it's working, present M1's
        // via the simple blocking issue (since xbar is muxed, both complete).
        issue_write<0>(/*id*/ 0x10 + k, addr0, 0, 4, {d0}, 0);
        issue_write<1>(/*id*/ 0x20 + k, addr1, 0, 4, {d1}, 0);
    }
    // Final check: 8 new OKs per master, and data readable back via slave store.
    check("M0 completed 8 writes with OKAY",
          mb[0].w_completed_ok == prev_ok_0 + 8);
    check("M1 completed 8 writes with OKAY",
          mb[1].w_completed_ok == prev_ok_1 + 8);
    bool all_data_ok = true;
    for (int k = 0; k < 8; k++) {
        Word128 d0 = w128_from_u64(0x10 + k, 0x100 + k);
        Word128 d1 = w128_from_u64(0x20 + k, 0x200 + k);
        if (!w128_eq(s0_mem.read(0x00000200 + k*0x40), d0)) {
            std::printf("  [contention] M0 slot %d mismatch\n", k); all_data_ok = false;
        }
        if (!w128_eq(s0_mem.read(0x00010000 + k*0x40), d1)) {
            std::printf("  [contention] M1 slot %d mismatch\n", k); all_data_ok = false;
        }
    }
    check("All contention-test write data landed correctly at DDR", all_data_ok);
}

static void scenario_7_decerr() {
    // MAME-canonical Q700: unmapped reads return OKAY + 0x00000000
    // (open bus, MAME default `set_unmap_value`), writes silently
    // dropped with OKAY.  No DECERR.
    std::printf("\n=== Scenario 7: Unmapped addresses → OKAY + open-bus ===\n");
    // M0 read at 0x7000_0000 (between FB end and 0x8000_0000) → unmapped
    int prev_ok = mb[0].r_completed;
    issue_read<0>(/*id*/ 0xE, /*addr*/ 0x70000000, /*len*/ 0, /*size*/ 4,
                  /*rresp*/ 0 /*OKAY*/);
    check("M0 read of unmapped 0x70000000 returns OKAY",
          mb[0].r_completed == prev_ok + 1);
    bool got_open_bus_r = !mb[0].r_received.empty() &&
                          mb[0].r_received.back().size() == 1 &&
                          w128_eq(mb[0].r_received.back()[0],
                                  Word128{0u, 0u, 0u, 0u});
    check("M0 read of unmapped 0x70000000 returns 0x00000000 open-bus",
          got_open_bus_r);
    // M1 write to 0x8000_0000 (silently dropped, OKAY)
    int prev_wok = mb[1].w_completed_ok;
    Word128 junk = w128_from_u64(0xDEADBEEF);
    issue_write<1>(/*id*/ 0xF, /*addr*/ 0x80000000, /*len*/ 0, /*size*/ 4, {junk},
                   /*expect_bresp*/ 0);
    check("M1 write of unmapped 0x80000000 returns OKAY (silently dropped)",
          mb[1].w_completed_ok == prev_wok + 1);
    // Confirm no slave saw these addresses.
    check("DDR S0 did not see unmapped AR 0x70000000",
          s0_bfm.seen_ar_count[0x70000000] == 0);
    check("I/O S1 did not see unmapped AR 0x70000000",
          s1_bfm.seen_ar_count[0x70000000] == 0);
    check("DDR S0 did not see unmapped AW 0x80000000",
          s0_bfm.seen_aw_count[0x80000000] == 0);
    check("I/O S1 did not see unmapped AW 0x80000000",
          s1_bfm.seen_aw_count[0x80000000] == 0);
}

// ─── Stress-test extensions (tb-axi-xbar-stress) ────────────────────────

// Scenario 8: 16× interleaved back-to-back writes from M0 and M1 to the
// same slave (DDR).  Verifies arbitration fairness and that all 32 txns
// complete with no lost transfers.
static void scenario_8_back_to_back_16x() {
    std::printf("\n=== Scenario 8: 16x back-to-back M0+M1 writes to DDR ===\n");
    int prev_ok_0 = mb[0].w_completed_ok;
    int prev_ok_1 = mb[1].w_completed_ok;

    for (int k = 0; k < 16; k++) {
        uint32_t addr0 = 0x00020000 + (k * 0x40);
        uint32_t addr1 = 0x00030000 + (k * 0x40);
        Word128 d0 = w128_from_u64(0x1000 + k, 0x1100 + k);
        Word128 d1 = w128_from_u64(0x2000 + k, 0x2200 + k);
        // IDs are 4-bit wide; xbar truncates above 0xF.
        issue_write<0>((k & 0xF), addr0, 0, 4, {d0}, 0);
        issue_write<1>((k & 0xF), addr1, 0, 4, {d1}, 0);
    }
    check("M0 completed 16 writes with OKAY",
          mb[0].w_completed_ok == prev_ok_0 + 16);
    check("M1 completed 16 writes with OKAY",
          mb[1].w_completed_ok == prev_ok_1 + 16);

    // Verify every single slot landed correctly.
    bool all_ok = true;
    for (int k = 0; k < 16; k++) {
        Word128 d0 = w128_from_u64(0x1000 + k, 0x1100 + k);
        Word128 d1 = w128_from_u64(0x2000 + k, 0x2200 + k);
        if (!w128_eq(s0_mem.read(0x00020000 + k*0x40), d0)) {
            std::printf("  M0 slot %d mismatch\n", k); all_ok = false;
        }
        if (!w128_eq(s0_mem.read(0x00030000 + k*0x40), d1)) {
            std::printf("  M1 slot %d mismatch\n", k); all_ok = false;
        }
    }
    check("All 32 txns landed correctly at DDR", all_ok);
}

// Scenario 9: mid-transaction "master abort" — we simulate a master
// that presents AWVALID but then drops it mid-handshake (before
// AWREADY).  AXI-compliance says a master MUST NOT deassert valid
// until ready, but the xbar should tolerate it by simply not
// forwarding.  Verify no spurious AW reaches the slave.
static void scenario_9_abort_before_awready() {
    std::printf("\n=== Scenario 9: master drops AWVALID before AWREADY ===\n");
    // Hold AWVALID high briefly, then drop it without waiting.  The
    // xbar might have latched the request once AWREADY was high; if it
    // did, we should see the transaction actually complete.  If the
    // drop happened before AWREADY, no transaction should fire.
    //
    // Here we use M1 which has no AW outstanding; keep AWREADY
    // blockage by pre-filling the slave with a slow B response isn't
    // easy — just test the simpler "issue-and-observe" case: we never
    // actually drop mid-handshake in practice (the blocking helper
    // waits for AWREADY), but we can simulate the scenario by peeking
    // at state.
    //
    // For this TB, we use a degenerate form: we count outstanding AWs
    // and confirm no leftover AW after a complete issue_write.
    int prev_out_s0_aw = s0_bfm.aw_busy ? 1 : 0;
    Word128 d = w128_from_u64(0xDEADu);
    issue_write<1>(0x6, 0x00040000, 0, 4, {d}, 0);
    int post_out_s0_aw = s0_bfm.aw_busy ? 1 : 0;
    check("No dangling AW after completed write", post_out_s0_aw == prev_out_s0_aw);
}

// Scenario 10: open-bus paths on multiple unmapped regions.  An unmapped
// read should return OKAY + 0x00000000 (MAME-canonical Q700 default
// unmap_value), NOT forward to any slave.
//   - 0x7000_0000: between FB and 0x8000_0000
//   - 0x9000_0000: above everything
//   - 0xFFFF_0000: sim magic window (unmapped per xbar)
static void scenario_10_decerr_all_unmapped() {
    std::printf("\n=== Scenario 10: open-bus for multiple unmapped regions ===\n");
    struct { uint32_t addr; const char* name; } regions[] = {
        {0x70000000, "gap-between-FB-and-0x80M"},
        {0x90000000, "above-IO"},
        {0xFFFF0000, "sim-magic"},
    };
    for (auto& r : regions) {
        int prev_rok = mb[0].r_completed;
        int prev_wok = mb[0].w_completed_ok;
        issue_read<0>(0x5, r.addr, 0, 4, /*rresp*/ 0 /*OKAY*/);
        Word128 d = w128_from_u64(0xBADu);
        issue_write<0>(0x6, r.addr, 0, 4, {d}, /*bresp*/ 0 /*OKAY*/);
        char msg[128];
        std::snprintf(msg, sizeof msg, "read %s (0x%08x) → OKAY (open-bus)",
                      r.name, r.addr);
        check(msg, mb[0].r_completed == prev_rok + 1);
        // Each unmapped read must return all-zeros.
        bool open_bus = !mb[0].r_received.empty() &&
                        mb[0].r_received.back().size() == 1 &&
                        w128_eq(mb[0].r_received.back()[0],
                                Word128{0u, 0u, 0u, 0u});
        std::snprintf(msg, sizeof msg, "read %s rdata == 0x00000000", r.name);
        check(msg, open_bus);
        std::snprintf(msg, sizeof msg, "write %s → OKAY (silently dropped)",
                      r.name);
        check(msg, mb[0].w_completed_ok == prev_wok + 1);
    }
}

// Scenario 11: XDMA read + CPU write simultaneously to different slaves
// — XDMA reads from IO (S1), CPU writes to RAM (S0).  Both should
// complete without interference.
static void scenario_11_cross_slave_parallel() {
    std::printf("\n=== Scenario 11: M0→S0 write + M1→S1 read (parallel slaves) ===\n");
    int prev_ok = mb[0].w_completed_ok;
    int prev_r  = mb[1].r_completed;

    // Pre-populate both backends.
    Word128 cpu_data  = w128_from_u64(0xC0FFEE);
    s1_mem.write(0x50000040, w128_from_u64(0xFEEDFEEDu));

    // IDs fit in ID_WIDTH=4 bits — any larger value gets truncated in
    // the xbar's rid output, causing an expect-vs-got mismatch.
    issue_write<0>(0x7, 0x00005000, 0, 4, {cpu_data}, 0);
    issue_read<1>(0x8, 0x50000040, 0, 4, 0);
    check("M0→S0 write OKAY",          mb[0].w_completed_ok == prev_ok + 1);
    check("M1→S1 read OKAY",           mb[1].r_completed == prev_r + 1);
    check("M0 write reached DDR",
          w128_eq(s0_mem.read(0x00005000), cpu_data));
    check("M1 read saw S1 value",
          !mb[1].r_received.empty() && mb[1].r_received.back().size() == 1 &&
          w128_eq(mb[1].r_received.back()[0], w128_from_u64(0xFEEDFEEDu)));
}

// ─── Task #116: M4 (DMA) + S2 (DMA_CFG) scenarios ───────────────────────
//
// scenario_12/14/15 below exercised the M4 (DMA) AXI4 *master* port,
// which no longer exists on the DUT as of the 2026-07-16 master-count
// reduction (dma_ctrl's data-movement master port is stubbed out of the
// crossbar — see axi_xbar.v's header note and fpga_top_dma.vh).  Per
// this repo's "never delete tests" policy, the scenario bodies are kept
// verbatim below (guarded out with #if 0, NOT deleted) as a historical
// record and for easy revival the day a real DMA consumer re-wires M4 —
// at that point, restore the M4 port on axi_xbar.v, re-add the mb[4]
// bookkeeping (idle_m4/observe_b(4,...)/observe_r(4,...)), and un-guard
// these.  scenario_13 (DMA *config*, via S2 — a slave port, untouched by
// this reduction) is unaffected and still runs normally.
#if 0
static void scenario_12_dma_write_ram() {
    std::printf("\n=== Scenario 12: M4 DMA writes to RAM (S0) ===\n");
    int prev_ok = mb[4].w_completed_ok;
    Word128 payload = w128_from_u64(0x1111222233334444ULL, 0x5555666677778888ULL);
    issue_write<4>(0x3, 0x00008000, 0, 4, {payload}, 0);
    check("M4 RAM write returns OKAY", mb[4].w_completed_ok == prev_ok + 1);
    check("M4 write reached DDR at flattened RAM offset 0x00008000",
          w128_eq(s0_mem.read(0x00008000), payload));
}
#endif

static void scenario_13_cpu_write_dma_cfg() {
    std::printf("\n=== Scenario 13: M0 writes DMA config (0x5010_0000) → S2 ===\n");
    int prev_ok = mb[0].w_completed_ok;
    Word128 cfg = w128_from_u64(0x00000001ULL);
    issue_write<0>(0x4, 0x50100000, 0, 4, {cfg}, 0);
    check("M0 DMA_CTRL write returns OKAY", mb[0].w_completed_ok == prev_ok + 1);
    check("S2 (DMA_CFG) saw AW at 0x50100000", s2_bfm.seen_aw_count[0x50100000] >= 1);
    check("S1 (IO) did NOT see AW at 0x50100000", s1_bfm.seen_aw_count[0x50100000] == 0);
    check("DMA config value landed on S2", w128_eq(s2_mem.read(0x50100000), cfg));
}

#if 0
static void scenario_14_dma_read_ram_vs_cpu_write() {
    std::printf("\n=== Scenario 14: M4 read RAM + M0 write RAM ===\n");
    Word128 pattern = w128_from_u64(0xAAAAAAAA55555555ULL, 0xDEADBEEFFEEDFACEULL);
    s0_mem.write(0x00009000, pattern);
    int prev_r = mb[4].r_completed;
    int prev_w = mb[0].w_completed_ok;
    Word128 cpu_data = w128_from_u64(0xC0DE00000001ULL);
    issue_write<0>(0x2, 0x0000A000, 0, 4, {cpu_data}, 0);
    issue_read <4>(0x6, 0x00009000, 0, 4, 0);
    check("M0 RAM write completed OK", mb[0].w_completed_ok == prev_w + 1);
    check("M4 RAM read completed OK",  mb[4].r_completed    == prev_r + 1);
    check("M4 received expected pattern",
          !mb[4].r_received.empty() && mb[4].r_received.back().size() == 1 &&
          w128_eq(mb[4].r_received.back()[0], pattern));
    check("M0 write data landed at target", w128_eq(s0_mem.read(0x0000A000), cpu_data));
}

static void scenario_15_backpressure_dma_write() {
    std::printf("\n=== Scenario 15: DMA write with S0 back-pressure ===\n");
    int prev_ok = mb[4].w_completed_ok;
    Word128 payload = w128_from_u64(0xFEEDFACEDEADBEEFULL, 0x0123456789ABCDEFULL);
    s0_stall_aw = 20;
    s0_stall_w  = 20;
    issue_write<4>(0x7, 0x0000B000, 0, 4, {payload}, 0);
    check("DMA write with backpressure still completes OK",
          mb[4].w_completed_ok == prev_ok + 1);
    check("DMA payload landed correctly under backpressure",
          w128_eq(s0_mem.read(0x0000B000), payload));
}
#endif

static void scenario_16_three_slave_parallel() {
    // Formerly M0→S0 + M1→S1 + M4→S2 (3-slave parallel).  The M4 leg is
    // dropped — M4 no longer exists (see the task #116 comment above);
    // the M0/M1 legs, which are unaffected by the master-count
    // reduction, are kept unchanged so this remains real 2-slave
    // parallel-arbitration coverage rather than being deleted outright.
    std::printf("\n=== Scenario 16: M0→S0 + M1→S1 (parallel slaves) ===\n");
    int prev_w0 = mb[0].w_completed_ok;
    int prev_w1 = mb[1].w_completed_ok;
    Word128 d0 = w128_from_u64(0x1111111111111111ULL);
    Word128 d1 = w128_from_u64(0x2222222222222222ULL);
    issue_write<0>(0xA, 0x0000C000, 0, 4, {d0}, 0);
    issue_write<1>(0xB, 0x50000080, 0, 4, {d1}, 0);
    check("M0→S0 completed OK", mb[0].w_completed_ok == prev_w0 + 1);
    check("M1→S1 completed OK", mb[1].w_completed_ok == prev_w1 + 1);
    check("M0 write landed at DDR", w128_eq(s0_mem.read(0x0000C000), d0));
    check("M1 write landed at IO",  w128_eq(s1_mem.read(0x50000080), d1));
}

// ─── tb-axi-xbar-stress extensions (task #82) ───────────────────────────
// Address-boundary edge cases. The xbar memory map has several hazardous
// edges that the task specifically calls out: top of DDR/RAM window,
// bottom of peripheral window, overlap at the DMA-hole-inside-IO.  These
// scenarios exercise each boundary.

// Scenario 17: RAM decode + default RAM-window aliasing.
// 1 GiB of low address space decodes as RAM.  With the default aliasing
// policy, the lg2=26 window selects a 64 MiB backing size and higher RAM
// probe addresses mirror by dropping bits above that window.
static void scenario_17_ram_window_edges() {
    std::printf("\n=== Scenario 17: RAM decode + default window aliasing ===\n");
    Word128 lo = w128_from_u64(0x0000000000000001ULL, 0x0BADC0DE0BADC0DEULL);
    Word128 hi = w128_from_u64(0xFFFFFFFFFFFFFFFFULL, 0x1111222233334444ULL);
    // addr 0 -> flattened DDR offset 0
    s0_mem.write(0x00000000, Word128{0,0,0,0});
    issue_write<0>(0x1, 0x00000000, 0, 4, {lo}, 0);

    // Last 16B-aligned address in the visible 64 MiB window.
    uint32_t vis_top = RAM_WINDOW_MASK_DFLT & ~0xFu; // 0x03FF_FFF0
    s0_mem.write(vis_top, Word128{0,0,0,0});
    issue_write<0>(0x2, vis_top, 0, 4, {hi}, 0);
    check("RAM addr=0 write landed at DDR offset 0",
          w128_eq(s0_mem.read(0x00000000), lo));
    check("RAM visible-top write landed at DDR",
          w128_eq(s0_mem.read(vis_top), hi));

    // First byte past visible RAM (0x04000000) is RAM-OOR open-bus
    // (MAME-canonical: OKAY + writes silently dropped).  RAM_ALIAS_MODE
    // is 0 by default — past-window probes do NOT wrap into low DDR.
    int prev_open_ok = mb[0].w_completed_ok;
    int prev_aw_low_off = s0_bfm.seen_aw_count[0x00000000u];
    Word128 junk = w128_from_u64(0xDEAD);
    issue_write<0>(0x3, 0x04000000u, 0, 4, {junk}, /*bresp*/0);
    check("addr=0x04000000 OOR write OKAY (silently dropped)",
          mb[0].w_completed_ok == prev_open_ok + 1);
    check("addr=0x04000000 did NOT alias to DDR offset 0 (mode=0)",
          s0_bfm.seen_aw_count[0x00000000u] == prev_aw_low_off);

    // First byte past 1 GiB RAM decode is ROM base. M0 writes are accepted
    // and silently dropped, matching Q700 ROM mirror behavior.
    int prev_rom_ok = mb[0].w_completed_ok;
    issue_write<0>(0x4, RAM_DECODE_SIZE, 0, 4, {junk}, /*bresp*/0);
    check("addr=0x40000000 is ROM (M0 write -> OKAY drop)",
          mb[0].w_completed_ok == prev_rom_ok + 1);
}

// Scenario 18: ROM-window top edge. ROM_SIZE=4 MiB is the backing size, but
// the CPU-visible 0x4xxx_xxxx range is a Q700 ROM mirror.  CPU writes anywhere
// in that mirror return OKAY and do not modify DDR.
static constexpr uint32_t ROM_IMAGE_PERIOD = 0x00100000u; // task #245
static void scenario_18_rom_window_top_edge() {
    std::printf("\n=== Scenario 18: ROM window top edge / past-end ===\n");
    set_cpu_held_in_reset(true);
    int prev_ok_boot = mb[3].w_completed_ok;
    Word128 d = w128_from_u64(0xAABBCCDDEEFF0011ULL);
    // Task #245: the ROM mirror period is now 1 MiB (MAME-faithful), so the
    // top of the CPU-visible mirror is +1 MiB, not +4 MiB.  This matters
    // because boot_fsm loads the image THROUGH this window
    // (ROM_BASE_ADDR + (sector<<9), boot_fsm.v:1392) -- i.e. the loader
    // shares the wrapping path.  It is safe only because BOOT_ROM_SECTORS
    // (2048 x 512 B = exactly 1 MiB) fills the period exactly, with ZERO
    // margin: raise BOOT_ROM_SECTORS above 2048 and loader writes silently
    // alias onto already-written bytes instead of extending the image.
    uint32_t rom_top = 0x40000000u + ROM_IMAGE_PERIOD - 16u; // 0x400F_FFF0
    issue_write_boot(0x4, rom_top, 0, 4, {d}, 0);
    check("boot FSM write at ROM top OK",
          mb[3].w_completed_ok == prev_ok_boot + 1);
    // flattened = DDR_ROM_OFF + ((rom_top - 0x40000000) & (period-1))
    check("Boot write landed at flattened ROM-top offset",
          w128_eq(s0_mem.read(DDR_ROM_OFF + ROM_IMAGE_PERIOD - 16u), d));
    set_cpu_held_in_reset(false);
    // One past the 4 MiB backing window is still inside the 0x4xxx ROM mirror.
    int prev_wok = mb[0].w_completed_ok;
    issue_write<0>(0x5, 0x40400000, 0, 4, {d}, /*bresp*/0);
    check("addr=0x40400000 remains ROM mirror (M0 write -> OKAY drop)",
          mb[0].w_completed_ok == prev_wok + 1);
    check("DDR S0 did not see forbidden M0 ROM-mirror write",
          s0_bfm.seen_aw_count[0x40400000u] == 0);
}

// Scenario 19: DMA hole (S2) vs IO (S1) boundary.  IO_BASE = 0x5000_0000,
// DMA_BASE = 0x5010_0000, DMA_SIZE = 0x0010_0000, IO_SIZE = 0x0100_0000.
//
// Expected routing:
//   0x500F_FFF0 → S1 IO  (just below DMA hole)
//   0x5010_0000 → S2 DMA (first DMA slot)
//   0x501F_FFF0 → S2 DMA (last DMA slot)
//   0x5020_0000 → S1 IO  (first IO slot above hole)
static void scenario_19_dma_hole_boundary() {
    std::printf("\n=== Scenario 19: DMA hole (S2) boundary inside IO (S1) ===\n");
    Word128 a = w128_from_u64(0x1111000011110000ULL);
    Word128 b = w128_from_u64(0x2222000022220000ULL);
    Word128 c = w128_from_u64(0x3333000033330000ULL);
    Word128 d = w128_from_u64(0x4444000044440000ULL);
    issue_write<0>(0x1, 0x500FFFF0, 0, 4, {a}, 0);
    issue_write<0>(0x2, 0x50100000, 0, 4, {b}, 0);
    issue_write<0>(0x3, 0x501FFFF0, 0, 4, {c}, 0);
    issue_write<0>(0x4, 0x50200000, 0, 4, {d}, 0);
    // Below-hole and above-hole must land on S1 (IO), not S2 (DMA).
    check("addr=0x500FFFF0 lands on S1 (IO)", w128_eq(s1_mem.read(0x500FFFF0), a));
    check("addr=0x500FFFF0 did NOT hit S2 (DMA)",
          s2_bfm.seen_aw_count[0x500FFFF0] == 0);
    check("addr=0x50200000 lands on S1 (IO)", w128_eq(s1_mem.read(0x50200000), d));
    check("addr=0x50200000 did NOT hit S2 (DMA)",
          s2_bfm.seen_aw_count[0x50200000] == 0);
    // DMA-hole addresses must land on S2, not S1.
    check("addr=0x50100000 lands on S2 (DMA)", w128_eq(s2_mem.read(0x50100000), b));
    check("addr=0x50100000 did NOT hit S1 (IO)",
          s1_bfm.seen_aw_count[0x50100000] == 0);
    check("addr=0x501FFFF0 lands on S2 (DMA)", w128_eq(s2_mem.read(0x501FFFF0), c));
    check("addr=0x501FFFF0 did NOT hit S1 (IO)",
          s1_bfm.seen_aw_count[0x501FFFF0] == 0);
}

// Scenario 20: FB window top + just past. FB_BASE = 0x6000_0000,
// FB_SIZE = 8 MB -> last slot 0x607FFFF0; 0x60800000 is unmapped.
static void scenario_20_fb_window_top_edge() {
    std::printf("\n=== Scenario 20: Framebuffer top-edge + past-end ===\n");
    Word128 p = w128_from_u64(0xCAFE0000CAFE0000ULL, 0xBEEF0000BEEF0000ULL);
    // flattened offset for 0x607FFFF0 = DDR_FB_OFF + 0x007FFFF0
    s0_mem.write(DDR_FB_OFF + 0x007ffff0u, p);
    issue_read<2>(0x6, 0x607FFFF0, 0, 4, /*rresp*/0);
    bool got = !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
               w128_eq(mb[2].r_received.back()[0], p);
    check("M2 FB top-edge read returns pixels", got);
    check("DDR S0 saw AR at flattened FB-top offset",
          s0_bfm.seen_ar_count[DDR_FB_OFF + 0x007ffff0u] >= 1);
    // One past FB end.  CPU IF master (M2) does not have a write port, so
    // test via M0 read (map check is master-agnostic on the AR side).
    // MAME-canonical: unmapped → OKAY + 0x00000000, no DECERR.
    int prev_rok = mb[0].r_completed;
    issue_read<0>(0x7, 0x60800000, 0, 4, /*rresp*/0);
    check("addr=0x60800000 (one past FB) → OKAY (open-bus)",
          mb[0].r_completed == prev_rok + 1);
}

// Scenario 21: Three-master concurrent hit on a SINGLE slave (DDR).
// M0 + M1 + M4 all issue 4 back-to-back single-beat writes to
// non-overlapping RAM slots.  Verifies arbitration fairness + no
// dropped transactions under 3-way pressure.
static void scenario_21_three_master_single_slave() {
    // Formerly M0+M1+M4 concurrent → DDR.  The M4 leg is dropped — M4 no
    // longer exists (see the task #116 comment above scenario 12/14/15).
    // A genuine 3-way-concurrent replacement isn't representable post-
    // merge either: the only remaining "3rd master" candidate is boot
    // FSM (via m0b_*), and it is by construction NEVER concurrent with
    // CPU LSU (M0) — that mutual exclusion is the entire point of the
    // merge (see axi_xbar.v header "M0/boot merge"), and is separately
    // proven by scenario_3e.  So this keeps genuine 2-way concurrent
    // M0+M1 pressure rather than fabricating a 3rd leg that couldn't
    // exist on real hardware.
    std::printf("\n=== Scenario 21: 2-master concurrent → DDR (single slave) ===\n");
    int p0 = mb[0].w_completed_ok, p1 = mb[1].w_completed_ok;
    for (int k = 0; k < 4; k++) {
        Word128 d0 = w128_from_u64(0xA000ULL + k, 0xAA000ULL + k);
        Word128 d1 = w128_from_u64(0xB000ULL + k, 0xBB000ULL + k);
        issue_write<0>(k & 0xF, 0x00100000 + (k*0x40), 0, 4, {d0}, 0);
        issue_write<1>(k & 0xF, 0x00110000 + (k*0x40), 0, 4, {d1}, 0);
    }
    check("M0 completed 4 writes OK", mb[0].w_completed_ok == p0 + 4);
    check("M1 completed 4 writes OK", mb[1].w_completed_ok == p1 + 4);
    bool all_ok = true;
    for (int k = 0; k < 4; k++) {
        Word128 d0 = w128_from_u64(0xA000ULL + k, 0xAA000ULL + k);
        Word128 d1 = w128_from_u64(0xB000ULL + k, 0xBB000ULL + k);
        if (!w128_eq(s0_mem.read(0x00100000 + k*0x40), d0)) all_ok = false;
        if (!w128_eq(s0_mem.read(0x00110000 + k*0x40), d1)) all_ok = false;
    }
    check("All 2×4 writes landed correctly at DDR", all_ok);
}

// Scenario 22: Same-address write-after-write from two different masters.
// Must serialise: whichever lands second wins.  Check that at least one
// of the two plausible values ended up in DDR and no transaction was
// silently dropped.
static void scenario_22_waw_same_addr_two_masters() {
    std::printf("\n=== Scenario 22: WAW same-addr (M0 then M1) → DDR ===\n");
    int p0 = mb[0].w_completed_ok, p1 = mb[1].w_completed_ok;
    Word128 d0 = w128_from_u64(0x0000DEAD0000DEADULL, 0x1111000011110000ULL);
    Word128 d1 = w128_from_u64(0x0000BEEF0000BEEFULL, 0x2222000022220000ULL);
    s0_mem.write(0x00200000, Word128{0,0,0,0});
    issue_write<0>(0x8, 0x00200000, 0, 4, {d0}, 0);
    issue_write<1>(0x9, 0x00200000, 0, 4, {d1}, 0);
    check("M0 write OK", mb[0].w_completed_ok == p0 + 1);
    check("M1 write OK", mb[1].w_completed_ok == p1 + 1);
    // Expected ordering — our blocking helper serialises issue_write<0>
    // first then issue_write<1>, so d1 must win.  DDR must not hold the
    // zero we initialised; otherwise the xbar lost one txn.
    Word128 final_val = s0_mem.read(0x00200000);
    check("final slot holds M1's value (last-writer-wins serial)",
          w128_eq(final_val, d1));
    check("DDR S0 saw two distinct AWs at 0x00200000",
          s0_bfm.seen_aw_count[0x00200000] >= 2);
}

// Scenario 23 — CPU (M0) write + read against the VRAM aperture (S3).
// Task #147: CPU writes to 0xF900_XXXX travel through xbar S3 and land
// in the vram slave with VRAM_BASE stripped.  Aperture is 2 MiB
// (0xF9000000..0xF91FFFFF) per Q700 silicon.  Verifies:
//   (a) a write at 0xF9000100 lands at S3 offset 0x00000100
//   (b) a read at the same CPU-side address returns the stored pixel word
//   (c) a write at aperture top 0xF91FFFF0 lands at S3 offset 0x001FFFF0
//   (d) 0xF9200000 (one past the 2 MB aperture) is UNMAPPED → open-bus
//       (OKAY + 0x00000000, MAME-canonical)
//   (e) the DAFB register window (0xF9800000) is not the pixel aperture;
//       it routes to S4 with the DAFB base stripped.
static void scenario_23_vram_aperture() {
    std::printf("\n=== Scenario 23: VRAM pixel aperture (task #147) ===\n");

    // (a) write at 0xF9000100 → S3 offset 0x00000100
    int prev_ok = mb[0].w_completed_ok;
    Word128 pixA = w128_from_u64(0x1122334455667788ULL, 0x99AABBCCDDEEFF00ULL);
    issue_write<0>(/*id*/ 0x2, /*addr*/ 0xF9000100, /*len*/ 0, /*size*/ 4,
                   {pixA}, /*bresp*/ 0);
    check("M0 CPU→VRAM write returns OKAY",
          mb[0].w_completed_ok == prev_ok + 1);
    check("VRAM slave (S3) received AW at stripped offset 0x100",
          s3_bfm.seen_aw_count[0x00000100] >= 1);
    check("VRAM slave did NOT see AW at raw CPU address 0xF9000100",
          s3_bfm.seen_aw_count[0xF9000100] == 0);
    check("S3 store holds the byte-swapped native VRAM word at offset 0x100",
          w128_eq(s3_mem.read(0x00000100), vram_native_word_order(pixA)));
    // Not routed to DDR
    check("DDR S0 did NOT see a write at 0xF9000100",
          s0_bfm.seen_aw_count[0xF9000100] == 0);

    // (b) read-back via CPU: should return the same word we wrote (from S3,
    // not DDR).  The xbar strips VRAM_BASE for reads too.
    issue_read<0>(/*id*/ 0x3, /*addr*/ 0xF9000100, /*len*/ 0, /*size*/ 4,
                  /*rresp*/ 0);
    bool got = !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
               w128_eq(mb[0].r_received.back()[0], pixA);
    check("M0 CPU→VRAM read returns the written pixel word", got);
    check("VRAM slave (S3) received AR at stripped offset 0x100",
          s3_bfm.seen_ar_count[0x00000100] >= 1);

    // (c) aperture top edge — last full 128-bit word inside 2 MB.
    int prev_ok2 = mb[0].w_completed_ok;
    Word128 pixT = w128_from_u64(0xCAFE0001CAFE0001ULL, 0xF00D0001F00D0001ULL);
    issue_write<0>(/*id*/ 0x4, /*addr*/ 0xF91FFFF0, /*len*/ 0, /*size*/ 4,
                   {pixT}, /*bresp*/ 0);
    check("VRAM top-edge write (0xF91FFFF0) returns OKAY",
          mb[0].w_completed_ok == prev_ok2 + 1);
    check("VRAM top-edge landed at stripped offset 0x001FFFF0",
          w128_eq(s3_mem.read(0x001FFFF0), vram_native_word_order(pixT)));

    // (d) one past the 2 MB aperture → open-bus (OKAY + 0x00000000).
    int prev_wok = mb[0].w_completed_ok;
    Word128 junk = w128_from_u64(0xDEADBEEF);
    issue_write<0>(/*id*/ 0x5, /*addr*/ 0xF9200000, /*len*/ 0, /*size*/ 4,
                   {junk}, /*bresp*/ 0 /*OKAY*/);
    check("0xF9200000 (one past VRAM) → OKAY (silently dropped)",
          mb[0].w_completed_ok == prev_wok + 1);
    check("VRAM slave did not see 0xF9200000",
          s3_bfm.seen_aw_count[0xF9200000] == 0);
    check("VRAM slave did not see stripped offset 0x00200000",
          s3_bfm.seen_aw_count[0x00200000] == 0);

    // (e) DAFB register window (0xF9800000) — S4 register shim, not
    // the S1 peripheral bus or S3 pixel aperture.
    int prev_ok3 = mb[0].w_completed_ok;
    issue_write<0>(/*id*/ 0x6, /*addr*/ 0xF9800000, /*len*/ 0, /*size*/ 4,
                   {junk}, /*bresp*/ 0 /*OKAY*/);
    check("0xF9800000 (DAFB register window) → S4 OKAY",
          mb[0].w_completed_ok == prev_ok3 + 1);
    check("S4 DAFB saw stripped AW offset 0x00000000",
          s4_bfm.seen_aw_count[0x00000000] >= 1);
    check("S1 peripheral bus did NOT see DAFB AW",
          s1_bfm.seen_aw_count[0xF9800000] == 0);
    check("VRAM slave did not see 0xF9800000",
          s3_bfm.seen_aw_count[0xF9800000] == 0);
}

// Scenario 24: Partial-byte write on the DDR path must preserve untouched
// bytes and route through S0 without lane scrambling.
static void scenario_24_ddr_partial_write_mask() {
    std::printf("\n=== Scenario 24: DDR partial-byte write preserves lane mask ===\n");
    uint32_t addr = 0x00012000;
    Word128 base  = w128_from_u64(0x1111111111111111ULL, 0x2222222222222222ULL);
    Word128 patch = base;
    set_byte(patch, 1,  0xAA);
    set_byte(patch, 8,  0xBB);
    set_byte(patch, 15, 0xCC);
    Word128 expect = base;
    set_byte(expect, 1,  0xAA);
    set_byte(expect, 8,  0xBB);
    set_byte(expect, 15, 0xCC);

    s0_mem.write(addr, base);
    issue_write<0>(/*id*/ 0xC, addr, /*len*/ 0, /*size*/ 4, {patch},
                   /*expect_bresp*/ 0, /*wstrb*/ 0x8102);
    check("S0 saw the partial-write address",
          s0_bfm.seen_aw_count[addr] >= 1);
    check("DDR byte-lane merge preserved untouched bytes",
          w128_eq(s0_mem.read(addr), expect));

    issue_read<0>(/*id*/ 0xD, addr, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    bool rd_ok = !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
                 w128_eq(mb[0].r_received.back()[0], expect);
    check("CPU read sees the merged DDR beat", rd_ok);
}

// Scenario 25 — Reprogram RAM-visible window through debug selector.
// lg2=22 (4 MiB) keeps 0x003F_FFF0 direct, but past-window probes at
// 0x0040_0000 take the open-bus / SLOT_VOID path under the MAME-canonical
// RAM_ALIAS_MODE=0 default (writes silently dropped, OKAY).
static void scenario_25_ram_window_reprogram() {
    std::printf("\n=== Scenario 25: RAM window reprogram (lg2=22, mode=0) ===\n");
    dut->dbg_ram_window_lg2 = 22;
    for (int i = 0; i < 4; i++) cycle();

    Word128 in = w128_from_u64(0x1234567890ABCDEFULL, 0x0BADF00D0BADF00DULL);
    Word128 oob = w128_from_u64(0xDEADBEEFDEADBEEFULL);
    int prev_ok = mb[0].w_completed_ok;
    issue_write<0>(0xE, 0x003FFFF0u, 0, 4, {in}, 0);
    check("4MiB visible-top write OK", mb[0].w_completed_ok == prev_ok + 1);
    check("4MiB visible-top landed at DDR", w128_eq(s0_mem.read(0x003FFFF0u), in));

    int prev_oob_ok = mb[0].w_completed_ok;
    int prev_aw_zero = s0_bfm.seen_aw_count[0x00000000u];
    issue_write<0>(0xF, 0x00400000u, 0, 4, {oob}, 0);
    check("4MiB+0 OOR write OKAY (silently dropped)",
          mb[0].w_completed_ok == prev_oob_ok + 1);
    check("4MiB+0 did NOT alias into DDR offset 0 (mode=0)",
          s0_bfm.seen_aw_count[0x00000000u] == prev_aw_zero);
    check("S0 did not see raw AW at 0x00400000", s0_bfm.seen_aw_count[0x00400000u] == 0);

    // Restore default for any later additions.
    dut->dbg_ram_window_lg2 = 26;
    for (int i = 0; i < 4; i++) cycle();
}

// Scenario 26 — CPU reset overlay lives at xbar decode, not in the core.
// With cpu_overlay_active=1, only M0 low-memory reads are remapped onto
// ROM_BASE and then flattened to the DDR ROM image offset.  With it off,
// the same low address is ordinary RAM.
static void scenario_26_cpu_overlay_decode() {
    std::printf("\n=== Scenario 26: CPU reset overlay decode ===\n");
    uint32_t low_addr = 0x00000100u;
    Word128 rom_word = w128_from_u64(0x0A0B0C0D0E0F1011ULL, 0x1213141516171819ULL);
    Word128 ram_word = w128_from_u64(0x2021222324252627ULL, 0x28292A2B2C2D2E2FULL);
    s0_mem.write(DDR_ROM_OFF + low_addr, rom_word);
    s0_mem.write(low_addr, ram_word);

    dut->cpu_overlay_active = 1;
    for (int i = 0; i < 4; i++) cycle();
    issue_read<0>(0x1, low_addr, 0, 4, 0);
    bool overlay_got_rom = !mb[0].r_received.empty() &&
                           mb[0].r_received.back().size() == 1 &&
                           w128_eq(mb[0].r_received.back()[0], rom_word);
    check("overlay-active M0 low read returns ROM image data", overlay_got_rom);
    check("overlay-active low read hit flattened ROM offset",
          s0_bfm.seen_ar_count[DDR_ROM_OFF + low_addr] >= 1);

    dut->cpu_overlay_active = 0;
    for (int i = 0; i < 4; i++) cycle();
    issue_read<0>(0x2, low_addr, 0, 4, 0);
    bool overlay_off_got_ram = !mb[0].r_received.empty() &&
                               mb[0].r_received.back().size() == 1 &&
                               w128_eq(mb[0].r_received.back()[0], ram_word);
    check("overlay-inactive M0 low read returns RAM data", overlay_off_got_ram);
    check("overlay-inactive low read hit RAM offset",
          s0_bfm.seen_ar_count[low_addr] >= 1);
}

// Scenario 26b — unified-reset overlay re-arm without resetting xbar fabric.
// A CPU high-ROM read disables the low-ROM overlay.  JTAG unified reset keeps
// the xbar service path alive, so the fabric `rst` stays low; the separate
// cpu_overlay_reset input must still re-arm the CPU vector-fetch alias.
static void scenario_26b_cpu_overlay_rearm_without_xbar_reset() {
    std::printf("\n=== Scenario 26b: CPU overlay re-arm without xbar reset ===\n");
    uint32_t low_addr = 0x00000000u;
    uint32_t high_rom_addr = 0x40000000u;
    Word128 rom_word = w128_from_u64(0x420DBFF30000002AULL, 0x4EFA006000000000ULL);
    Word128 ram_word = w128_from_u64(0x0000000000000000ULL, 0x0000000000000000ULL);
    s0_mem.write(DDR_ROM_OFF + low_addr, rom_word);
    s0_mem.write(low_addr, ram_word);

    dut->cpu_overlay_active = 1;
    dut->cpu_overlay_reset = 0;
    for (int i = 0; i < 4; i++) cycle();

    issue_read<2>(0x1, low_addr, 0, 4, 0);
    bool initial_overlay = !mb[2].r_received.empty() &&
                           mb[2].r_received.back().size() == 1 &&
                           w128_eq(mb[2].r_received.back()[0], rom_word);
    check("initial CPUI low read returns ROM image data", initial_overlay);

    issue_read<2>(0x2, high_rom_addr, 0, 4, 0);
    check("high-ROM CPU fetch disables overlay",
          s0_bfm.seen_ar_count[DDR_ROM_OFF] >= 2);

    issue_read<2>(0x3, low_addr, 0, 4, 0);
    bool disabled_got_ram = !mb[2].r_received.empty() &&
                            mb[2].r_received.back().size() == 1 &&
                            w128_eq(mb[2].r_received.back()[0], ram_word);
    check("post-disable CPUI low read returns RAM data", disabled_got_ram);

    dut->cpu_overlay_reset = 1;
    for (int i = 0; i < 2; i++) cycle();
    dut->cpu_overlay_reset = 0;
    for (int i = 0; i < 2; i++) cycle();

    issue_read<2>(0x4, low_addr, 0, 4, 0);
    bool rearmed_got_rom = !mb[2].r_received.empty() &&
                           mb[2].r_received.back().size() == 1 &&
                           w128_eq(mb[2].r_received.back()[0], rom_word);
    check("overlay-reset re-arms CPUI low read to ROM", rearmed_got_rom);
}

// Scenario 27 — Q700 ROM RAM-probe alias at 0x5800_0000.
// The Universal ROM writes a signature through this window before checking
// it during the late descriptor/monitor path.  It must behave like low RAM,
// not like an unmapped probe.
static void scenario_27_q700_ram_alias() {
    std::printf("\n=== Scenario 27: Q700 0x5800_0000 RAM alias ===\n");
    uint32_t alias_addr = RAM_ALIAS_BASE + 0x00002000u;
    uint32_t low_addr = 0x00002000u;
    Word128 via_alias = w128_from_u64(0xAAAA5555AAAA5555ULL,
                                      0x1122334455667788ULL);
    Word128 via_low = w128_from_u64(0x0102030405060708ULL,
                                    0xA0A1A2A3A4A5A6A7ULL);

    int prev_w = mb[0].w_completed_ok;
    issue_write<0>(0x1, alias_addr, 0, 4, {via_alias}, 0);
    check("0x5800 alias write completed OK", mb[0].w_completed_ok == prev_w + 1);
    check("0x5800 alias write landed at low DDR offset",
          w128_eq(s0_mem.read(low_addr), via_alias));
    check("0x5800 alias AW was flattened before S0",
          s0_bfm.seen_aw_count[low_addr] >= 1);

    s0_mem.write(low_addr, via_low);
    issue_read<0>(0x2, alias_addr, 0, 4, 0);
    bool alias_read_low = !mb[0].r_received.empty() &&
                          mb[0].r_received.back().size() == 1 &&
                          w128_eq(mb[0].r_received.back()[0], via_low);
    check("0x5800 alias read returns low RAM contents", alias_read_low);
    check("0x5800 alias AR was flattened before S0",
          s0_bfm.seen_ar_count[low_addr] >= 1);

    dut->dbg_ram_window_lg2 = 22;
    for (int i = 0; i < 4; i++) cycle();
    // With RAM_ALIAS_MODE=0 (MAME-canonical), past-window probes do NOT
    // wrap — they hit the open-bus / SLOT_VOID path and return OKAY
    // with writes silently dropped.  Reads return 0x00000000.
    int prev_alias_ok = mb[0].w_completed_ok;
    int prev_aw_count = s0_bfm.seen_aw_count[(0x00400000u & RAM_WINDOW_MASK_4M)];
    issue_write<0>(0x3, RAM_ALIAS_BASE + 0x00400000u, 0, 4,
                   {via_alias}, /*bresp*/0);
    check("0x5800 alias above 4MiB write OKAY (silently dropped)",
          mb[0].w_completed_ok == prev_alias_ok + 1);
    check("0x5800 alias above 4MiB did NOT wrap into DDR (mode=0)",
          s0_bfm.seen_aw_count[(0x00400000u & RAM_WINDOW_MASK_4M)] == prev_aw_count);

    dut->dbg_ram_window_lg2 = 26;
    for (int i = 0; i < 4; i++) cycle();
}

// ─── 2026-07-21 write/read dead-cycle removal (scenarios 28-30) ─────────
// The write path used to hold W beats until a full WS_WAIT_SLV_AW →
// WS_FWD_W state hop after the slave accepted AW; the read path entered
// RS_WAIT_R one cycle after the slave AR handshake (it observed the
// registered s_arvalid_r clear instead of the handshake).  These
// scenarios pin the removed dead cycles exactly, using the S0 BFM's own
// commit-point cycle probes.

// 28. With an always-ready slave, a single-beat write's W beat must be
//     accepted by the slave in the SAME cycle as its AW (early-W
//     forwarding during WS_WAIT_SLV_AW).  Pre-change the W handshake
//     trailed AW by one cycle minimum.
static void scenario_28_early_w_same_cycle_as_aw() {
    std::printf("\n=== Scenario 28: early-W — W beat lands same cycle as AW ===\n");
    s0_aw_capture_cycle = -1;
    s0_first_w_cycle    = -1;
    Word128 v = w128_from_u64(0x28282828AABBCCDDULL, 0x1122334455667788ULL);
    mb[0].w_expect.push_back({0x9, 0});
    dut->m0_awid = 0x9; dut->m0_awaddr = 0x00002040; dut->m0_awlen = 0;
    dut->m0_awsize = 4; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    set_wdata(dut->m0_wdata, v);
    dut->m0_wstrb = 0xFFFF; dut->m0_wlast = 1; dut->m0_wvalid = 1;
    for (int i = 0; i < 60; i++) {
        dut->eval();
        bool aw_hs = dut->m0_awvalid && dut->m0_awready;
        bool w_hs  = dut->m0_wvalid  && dut->m0_wready;
        cycle();
        if (aw_hs) dut->m0_awvalid = 0;
        if (w_hs)  { dut->m0_wvalid = 0; dut->m0_wlast = 0; }
    }
    check("s28: write completed (B consumed)", mb[0].w_expect.empty());
    check("s28: write data landed in DDR", w128_eq(s0_mem.read(0x00002040), v));
    check("s28: slave saw AW", s0_aw_capture_cycle >= 0);
    check("s28: W beat accepted same cycle as AW (dead cycle removed)",
          s0_first_w_cycle == s0_aw_capture_cycle);
}

// 29. W beat fully accepted BEFORE the slave accepts AW (slave AW
//     stalled; slave FIFOs W early, like axi_async_bridge).  Exercises
//     the ws_wdone path: on AW-accept the xbar must skip WS_FWD_W, go
//     straight to WS_WAIT_B, and complete exactly one B with the right
//     ID — no lost or duplicated beats.
static void scenario_29_w_complete_before_aw_accept() {
    std::printf("\n=== Scenario 29: W burst completes before AW is accepted ===\n");
    s0_aw_capture_cycle = -1;
    s0_first_w_cycle    = -1;
    s0_early_w_accept   = true;
    s0_stall_aw         = 4;
    Word128 v = w128_from_u64(0x2929292911223344ULL, 0x99AABBCCDDEEFF00ULL);
    mb[0].w_expect.push_back({0xA, 0});
    dut->m0_awid = 0xA; dut->m0_awaddr = 0x00002080; dut->m0_awlen = 0;
    dut->m0_awsize = 4; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    set_wdata(dut->m0_wdata, v);
    dut->m0_wstrb = 0xFFFF; dut->m0_wlast = 1; dut->m0_wvalid = 1;
    for (int i = 0; i < 80; i++) {
        dut->eval();
        bool aw_hs = dut->m0_awvalid && dut->m0_awready;
        bool w_hs  = dut->m0_wvalid  && dut->m0_wready;
        cycle();
        if (aw_hs) dut->m0_awvalid = 0;
        if (w_hs)  { dut->m0_wvalid = 0; dut->m0_wlast = 0; }
    }
    s0_early_w_accept = false;
    check("s29: write completed (B consumed)", mb[0].w_expect.empty());
    check("s29: write data landed in DDR", w128_eq(s0_mem.read(0x00002080), v));
    check("s29: W beat accepted strictly before AW (ws_wdone path)",
          s0_first_w_cycle >= 0 && s0_aw_capture_cycle > s0_first_w_cycle);
}

// 30. Read from an immediately-responding slave: the first R-beat
//     handshake must land exactly ONE cycle after the slave's AR
//     capture (RS_WAIT_SLV_AR now advances on the AR handshake itself).
//     Pre-change the earliest acceptance trailed AR by two cycles.
static void scenario_30_read_no_dead_cycle() {
    std::printf("\n=== Scenario 30: read R-beat acceptance without dead cycle ===\n");
    // scenario_26b leaves the boot overlay re-armed — a low-memory read
    // would remap to the ROM image.  Disable it; this scenario is about
    // plain RAM-read handshake timing.
    dut->cpu_overlay_active = 0;
    for (int i = 0; i < 4; i++) cycle();
    s0_ar_capture_cycle = -1;
    s0_first_r_hs_cycle = -1;
    Word128 v = w128_from_u64(0x3030303055AA55AAULL, 0x0123456789ABCDEFULL);
    s0_mem.write(0x00002100, v);
    issue_read<0>(/*id*/0x5, /*addr*/0x00002100, /*len*/0, /*size*/4, /*rresp*/0);
    check("s30: read data correct",
          !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
          w128_eq(mb[0].r_received.back()[0], v));
    check("s30: slave saw AR", s0_ar_capture_cycle >= 0);
    check("s30: first R handshake exactly 1 cycle after AR capture",
          s0_first_r_hs_cycle == s0_ar_capture_cycle + 1);
}

// ─── xbar-burst-guard (task T5) scenarios ────────────────────────────────

// Scenario 28: a burst (awlen>0) write decoded onto S1 (I/O — a
// single-beat-only "Lite" slave, backed here by axi_wide_to_axilite in
// real integration) must NOT be forwarded.  All W beats are drained
// through WLAST, BRESP=SLVERR, and the real S1 slave never sees the AW.
// The fabric must still work normally afterward.
static void scenario_28_burst_write_lite_slave_legalized() {
    std::printf("\n=== Scenario 28: burst write (len>0) to Lite slave (S1) -> drained + SLVERR ===\n");
    uint32_t addr = 0x50002000u;
    std::vector<Word128> beats = {
        w128_from_u64(0x1111111111111111ULL),
        w128_from_u64(0x2222222222222222ULL),
        w128_from_u64(0x3333333333333333ULL),
        w128_from_u64(0x4444444444444444ULL),
    };
    int prev_aw  = s1_bfm.seen_aw_count[addr];
    int prev_err = mb[1].w_completed_errs;
    issue_write<1>(/*id*/ 0x8, addr, /*len*/ 3, /*size*/ 4, beats,
                   /*expect_bresp*/ 2 /*SLVERR*/);
    check("burst write to S1 completes with BRESP=SLVERR",
          mb[1].w_completed_errs == prev_err + 1);
    check("S1 real slave never saw the burst AW (not forwarded)",
          s1_bfm.seen_aw_count[addr] == prev_aw);
    check("S1 store untouched by the rejected burst",
          w128_eq(s1_mem.read(addr), Word128{0,0,0,0}));

    // Fabric alive after: a normal single-beat write to a DIFFERENT S1
    // address must still succeed.
    uint32_t addr2 = 0x50002100u;
    Word128 payload = w128_from_u64(0xCAFEF00DCAFEF00DULL);
    int prev_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0x9, addr2, 0, 4, {payload}, /*expect_bresp*/ 0);
    check("post-reject single-beat write to S1 still succeeds",
          mb[1].w_completed_ok == prev_ok + 1);
    check("post-reject write landed at S1", w128_eq(s1_mem.read(addr2), payload));
    check("S1 real slave saw the post-reject AW",
          s1_bfm.seen_aw_count[addr2] >= 1);
}

// Scenario 29: a burst (arlen>0) read decoded onto S1 must synthesize
// exactly len+1 beats, all SLVERR, RLAST on the final beat only, and
// never reach the real slave.  Fabric alive after.
static void scenario_29_burst_read_lite_slave_legalized() {
    std::printf("\n=== Scenario 29: burst read (len>0) from Lite slave (S1) -> len+1 SLVERR beats, RLAST on final ===\n");
    uint32_t addr = 0x50003000u;
    s1_mem.write(addr, w128_from_u64(0xDEADDEADDEADDEADULL)); // must never be observed
    int prev_ar  = s1_bfm.seen_ar_count[addr];
    int prev_err = mb[0].r_errors;
    issue_read<0>(/*id*/ 0xA, addr, /*len*/ 3, /*size*/ 4, /*expect_rresp*/ 2 /*SLVERR*/);
    check("burst read from S1 completes with RRESP=SLVERR",
          mb[0].r_errors == prev_err + 1);
    check("burst read from S1 synthesized exactly len+1=4 beats",
          !mb[0].r_received.empty() && mb[0].r_received.back().size() == 4);
    // Task review: the pre-existing RRESP check above only samples the
    // LAST beat (the one observe_r matches against r_expect on RLAST).
    // Verify explicitly that EVERY one of the 4 synthesized beats is
    // SLVERR, not just the terminal one.
    bool all_beats_slverr = !mb[0].r_received_resp.empty();
    if (all_beats_slverr) {
        const auto& resps = mb[0].r_received_resp.back();
        all_beats_slverr = (resps.size() == 4);
        for (uint32_t r : resps) all_beats_slverr = all_beats_slverr && (r == 2 /*SLVERR*/);
    }
    check("ALL 4 synthesized beats carry RRESP=SLVERR (not just the last)",
          all_beats_slverr);
    check("S1 real slave never saw the burst AR (not forwarded)",
          s1_bfm.seen_ar_count[addr] == prev_ar);

    // Fabric alive after: a normal single-beat read from a DIFFERENT S1
    // address must still succeed and return real data.
    uint32_t addr2 = 0x50003100u;
    Word128 payload = w128_from_u64(0x1234567890ABCDEFULL);
    s1_mem.write(addr2, payload);
    int prev_ok = mb[0].r_completed;
    issue_read<0>(/*id*/ 0xB, addr2, 0, 4, /*expect_rresp*/ 0);
    bool got = !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
               w128_eq(mb[0].r_received.back()[0], payload);
    check("post-reject single-beat read from S1 still succeeds",
          mb[0].r_completed == prev_ok + 1 && got);
    check("S1 real slave saw the post-reject AR", s1_bfm.seen_ar_count[addr2] >= 1);
}

// ─── Task-review follow-up scenarios (CRITICAL slv_flush fix, IMPORTANT
// 2/3 mid-burst + read-side watchdog timing fixes) ───────────────────────
// These deliberately target S2 (DMA config) rather than S1 for anything
// watchdog/poison-related, so this group's poison state never interacts
// with scenario 30's S1-based test (which runs after these and expects
// S1 to still be pristine at its start).

// Scenario 31 (CRITICAL fix): slv_flush aborts an in-flight S2
// transaction (accepted AW+W, B swallowed — i.e. genuinely sitting in
// WS_WAIT_B) WITHOUT setting the sticky poison latch, and S2 is fully
// usable again — for REAL, reaching the actual slave — once flush
// deasserts and the swallow is released.
static void scenario_31_slv_flush_abort_mid_transaction() {
    std::printf("\n=== Scenario 31: slv_flush aborts in-flight S2 write (WS_WAIT_B), no poison, usable after ===\n");
    uint32_t addr = 0x50104000u; // inside the DMA cfg window (S2)
    Word128 payload = w128_from_u64(0xF1F1F1F1F1F1F1F1ULL);

    s2_swallow_b = true;
    int prev_err = mb[1].w_completed_errs;
    issue_write<1>(/*id*/ 0xC, addr, 0, 4, {payload}, /*expect_bresp*/ 2 /*SLVERR — predicted flush-abort response*/);
    check("write not yet completed right after issue (S2 B swallowed)",
          mb[1].w_completed_errs == prev_err && !mb[1].w_expect.empty());

    dut->slv_flush = 1;
    for (int i = 0; i < 20; i++) cycle();
    check("slv_flush abort completes quickly (no need for the full watchdog window)",
          mb[1].w_completed_errs == prev_err + 1);
    check("write B-expect queue drained after flush-abort", mb[1].w_expect.empty());

    dut->slv_flush = 0;
    s2_swallow_b   = false;
    // The real S2 BFM's b_pending for the ABORTED transaction is still
    // internally latched (only the DRIVE of s2_bvalid was suppressed by
    // the swallow, matching the same "late response must be harmless"
    // model used elsewhere) — and this simple single-slot BFM model
    // gates accepting a NEW write's W beats on "!b_pending" (see
    // step_slave_s2), so a stale un-consumed b_pending would otherwise
    // wedge the BFM itself for any FOLLOW-UP transaction.  The xbar
    // already ignores this stale B for real (s2_bready stays low
    // forever since sw_owned[2] was released and never re-armed against
    // S2 while poisoned/flushed) — clearing the BFM's own bookkeeping
    // here just keeps the TEST MODEL from self-blocking so we can prove
    // "S2 usable again" below; it changes nothing about what the RTL
    // does.
    s2_bfm.b_pending = false;
    for (int i = 0; i < 10; i++) cycle();

    // Not poisoned: a fresh write to S2 must succeed for REAL (OKAY,
    // reaches the real slave) once flush is gone and the swallow is
    // released.
    uint32_t addr2 = 0x50104100u;
    Word128 payload2 = w128_from_u64(0xC2C2C2C2C2C2C2C2ULL);
    int prev_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0xD, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("S2 not poisoned by the flush-abort: post-flush write succeeds OKAY",
          mb[1].w_completed_ok == prev_ok + 1);
    check("S2 not poisoned by the flush-abort: post-flush write reached the real slave",
          s2_bfm.seen_aw_count[addr2] >= 1 && w128_eq(s2_mem.read(addr2), payload2));
}

// Scenario 32 (IMPORTANT/CRITICAL): a genuine read-side watchdog timeout
// (AR never accepted by the real slave — RS_WAIT_SLV_AR, distinct from
// scenario 30's write-side WS_WAIT_B case) on S2, followed by asserting
// slv_flush to prove the sticky poison latch clears on the RISING EDGE
// (CRITICAL fix part iv), not merely while flush stays asserted.
static void scenario_32_read_watchdog_and_flush_poison_clear() {
    std::printf("\n=== Scenario 32: read-side watchdog (AR never accepted) -> poison -> slv_flush clears poison ===\n");
    uint32_t addr = 0x50104200u;
    s2_swallow_ar = true;
    int prev_err = mb[0].r_errors;
    issue_read<0>(/*id*/ 0xE, addr, 0, 4, /*expect_rresp*/ 2 /*SLVERR*/);
    check("read not yet completed right after issue (S2 AR swallowed)",
          mb[0].r_errors == prev_err && !mb[0].r_expect.empty());

    for (int i = 0; i < WD_WAIT_CYCLES; i++) cycle();
    check("read-side watchdog fired: read completes with SLVERR",
          mb[0].r_errors == prev_err + 1);
    check("read R-expect queue drained (no dangling expectation)", mb[0].r_expect.empty());
    check("S2 real slave never accepted the swallowed AR",
          s2_bfm.seen_ar_count[addr] == 0);

    // Poisoned now: a fresh probe (even with AR still swallowed) gets
    // immediate local SLVERR rather than waiting another full window.
    uint32_t addr2 = 0x50104300u;
    int prev_err2 = mb[0].r_errors;
    issue_read<0>(/*id*/ 0xF, addr2, 0, 4, /*expect_rresp*/ 2 /*SLVERR*/);
    check("S2 poisoned: fresh read gets immediate SLVERR",
          mb[0].r_errors == prev_err2 + 1);

    // CRITICAL fix part iv: the RISING EDGE of slv_flush clears the
    // poison latch.  Release the AR swallow first so the slave is
    // actually able to serve a real transaction once un-poisoned.
    s2_swallow_ar = false;
    dut->slv_flush = 1;
    for (int i = 0; i < 5; i++) cycle();
    dut->slv_flush = 0;
    for (int i = 0; i < 10; i++) cycle();

    uint32_t addr3 = 0x50104400u;
    Word128 payload3 = w128_from_u64(0xABCDEF0123456789ULL);
    s2_mem.write(addr3, payload3);
    int prev_ok = mb[0].r_completed;
    issue_read<0>(/*id*/ 0x1, addr3, 0, 4, /*expect_rresp*/ 0);
    bool got3 = !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
                w128_eq(mb[0].r_received.back()[0], payload3);
    check("poison cleared on slv_flush rising edge: post-flush read succeeds OKAY with real data",
          mb[0].r_completed == prev_ok + 1 && got3);
    check("poison cleared: post-flush read reached the real S2 slave",
          s2_bfm.seen_ar_count[addr3] >= 1);
}

// Scenario 33 (IMPORTANT 2a): mid-burst WS_FWD_W timeout.  S0 (DDR,
// burst-capable — S2 can never reach WS_FWD_W since Fix 1 rejects any
// burst to it before forwarding) accepts AW then wedges partway through
// the W beats (permanent stall, not the bounded backpressure of
// scenario 15).  Confirms the len+1 (not fixed 8'hff) drain-count fix:
// with the old 8'hff bug the drain would consume only 255 of a smaller
// burst's beats and desync the NEXT write on this master; here we just
// confirm the abort completes cleanly and the fabric survives, since
// this master's own beat count is far below 255 either way — the exact
// 8'hff-vs-len+1 boundary is covered analytically in the RTL comment,
// and by scenario 35's len=255 read-side regression below.
static void scenario_33_mid_burst_write_timeout() {
    std::printf("\n=== Scenario 33: mid-burst WS_FWD_W timeout (S0/DDR wedges after 3 of 8 W beats) ===\n");
    // Addresses for scenarios 33-35 are deliberately >= ROM_SIZE
    // (0x400000): cpu_overlay_active is left asserted by scenario 26b
    // (by design — it's the boot-time default and no scenario needs to
    // clear it) and redirects M0/M2 READS below that threshold into ROM
    // space (see apply_cpu_overlay in axi_xbar.v); writes are unaffected
    // (gen_aw_overlay is a pass-through) but staying above the
    // threshold for all three keeps every address in this group
    // uniformly overlay-immune regardless of read/write direction.
    uint32_t addr = 0x00434000u;
    std::vector<Word128> beats;
    for (int i = 0; i < 8; i++) beats.push_back(w128_from_u64(0x9000000000000000ULL + i));

    s0_stall_w = 0; // ensure clean starting state
    int prev_err = mb[0].w_completed_errs;
    // Drive AW + the first 3 W beats normally (real slave accepts them).
    // For the REMAINDER, a compliant AXI master is NOT allowed to
    // retract WVALID before the transfer happens — so beat 3 onward we
    // HOLD wvalid/wlast asserted for the entire watchdog window rather
    // than giving up after a bounded poll (which would leave WS_DRAIN_W,
    // the state the watchdog switches to, with no beat to ever consume
    // and the write slot permanently wedged — a testbench bug, not an
    // RTL one, caught while writing this scenario).
    dut->m0_awid = 0xA; dut->m0_awaddr = addr; dut->m0_awlen = 7;
    dut->m0_awsize = 4; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    mb[0].w_expect.push_back({0xA, 2 /*SLVERR — predicted watchdog response*/});
    for (int i = 0; i < 500; i++) {
        dut->eval();
        if (dut->m0_awready) { cycle(); break; }
        cycle();
    }
    dut->m0_awvalid = 0;
    for (int b = 0; b < 3; b++) {
        set_wdata(dut->m0_wdata, beats[b]);
        dut->m0_wstrb = 0xFFFF; dut->m0_wlast = 0; dut->m0_wvalid = 1;
        for (int i = 0; i < 500; i++) {
            dut->eval();
            if (dut->m0_wready) { cycle(); break; }
            cycle();
        }
    }
    s0_stall_w = 1000000; // wedge the real slave's W-channel permanently
    set_wdata(dut->m0_wdata, beats[7]);
    dut->m0_wstrb = 0xFFFF; dut->m0_wlast = 1; dut->m0_wvalid = 1;
    for (int i = 0; i < WD_WAIT_CYCLES; i++) cycle();
    dut->m0_wvalid = 0; dut->m0_wlast = 0;
    check("mid-burst W-wedge: watchdog fires, write completes with SLVERR",
          mb[0].w_completed_errs == prev_err + 1);
    check("mid-burst W-wedge: write B-expect queue drained", mb[0].w_expect.empty());

    // Fabric alive after (DDR is now poisoned by this test's own
    // watchdog — same "any slave, not just Lite ones, can be poisoned"
    // policy scenario 30 already exercises on S1 — clear it via
    // slv_flush's rising-edge poison-clear so later scenarios that rely
    // on a healthy S0 aren't affected).
    s0_stall_w = 0;
    // The REAL s0_bfm never saw the aborted transaction's later beats
    // (once the xbar's watchdog switched to WS_DRAIN_W it self-drives
    // WREADY and stops presenting anything on S0's real W channel at
    // all — ws0_drive[] only asserts in WS_FWD_W), so the model is left
    // with aw_busy stuck true / w_beats_left>0 forever, exactly the
    // "the far side forgot mid-transaction" condition slv_flush exists
    // to handle in real hardware (where the bridge's OWN reset would
    // clear this).  Model that reset here so the BFM doesn't wedge
    // itself for the follow-up write below — mirrors the s2_bfm.b_pending
    // fix in scenario 31 for the same underlying reason.
    s0_bfm.aw_busy      = false;
    s0_bfm.w_beats_left = 0;
    dut->slv_flush = 1;
    for (int i = 0; i < 5; i++) cycle();
    dut->slv_flush = 0;
    for (int i = 0; i < 10; i++) cycle();

    uint32_t addr2 = 0x00434100u;
    Word128 payload2 = w128_from_u64(0xD00DD00DD00DD00DULL);
    int prev_ok = mb[0].w_completed_ok;
    issue_write<0>(/*id*/ 0xB, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("DDR usable again after flush clears the self-inflicted poison",
          mb[0].w_completed_ok == prev_ok + 1 && w128_eq(s0_mem.read(addr2), payload2));
}

// Scenario 34 (IMPORTANT 2b/3): mid-burst RS_WAIT_R timeout.  S0 (DDR)
// accepts AR and delivers 3 REAL (OKAY) beats of an 8-beat burst, then
// wedges.  Confirms rs_beats_done tracking + the remaining-beats
// computation deliver exactly the missing 5 beats (all SLVERR), RLAST
// on the 8th overall — not len+1 fresh beats (which would give the
// master 11 total) and not just 1 (the old before-review clamp bug).
static void scenario_34_mid_burst_read_timeout() {
    std::printf("\n=== Scenario 34: mid-burst RS_WAIT_R timeout (S0/DDR wedges after 3 of 8 R beats) ===\n");
    uint32_t addr = 0x00434200u; // see scenario 33's comment on overlay-immune addressing
    for (int i = 0; i < 8; i++)
        s0_mem.write(addr + i * 16, w128_from_u64(0xA000000000000000ULL + i));

    s0_r_swallow_after = 3;
    int prev_err = mb[0].r_errors;
    issue_read<0>(/*id*/ 0xC, addr, /*len*/ 7, /*size*/ 4, /*expect_rresp*/ 2 /*SLVERR — the terminal RLAST beat*/);

    for (int i = 0; i < WD_WAIT_CYCLES; i++) cycle();
    check("mid-burst R-wedge: watchdog fires, read completes (terminal beat SLVERR)",
          mb[0].r_errors == prev_err + 1);
    check("mid-burst R-wedge: read R-expect queue drained", mb[0].r_expect.empty());
    check("mid-burst R-wedge: master received exactly 8 total beats (3 real + 5 synthesized)",
          !mb[0].r_received.empty() && mb[0].r_received.back().size() == 8);
    if (!mb[0].r_received_resp.empty() && mb[0].r_received_resp.back().size() == 8) {
        const auto& resps = mb[0].r_received_resp.back();
        bool first3_ok  = (resps[0] == 0) && (resps[1] == 0) && (resps[2] == 0);
        bool last5_err  = (resps[3] == 2) && (resps[4] == 2) && (resps[5] == 2) &&
                           (resps[6] == 2) && (resps[7] == 2);
        check("mid-burst R-wedge: first 3 beats OKAY (real), last 5 SLVERR (synthesized)",
              first3_ok && last5_err);
        const auto& data = mb[0].r_received.back();
        bool real_data_ok = w128_eq(data[0], w128_from_u64(0xA000000000000000ULL)) &&
                             w128_eq(data[1], w128_from_u64(0xA000000000000001ULL)) &&
                             w128_eq(data[2], w128_from_u64(0xA000000000000002ULL));
        check("mid-burst R-wedge: the 3 real beats carry the actual DDR data", real_data_ok);
    } else {
        check("mid-burst R-wedge: first 3 beats OKAY (real), last 5 SLVERR (synthesized)", false);
        check("mid-burst R-wedge: the 3 real beats carry the actual DDR data", false);
    }

    // Clean up: this test's own watchdog also poisons DDR (generic
    // per-slave policy, not Lite-only-specific) — clear it via flush so
    // later scenarios needing a healthy S0 aren't affected.
    s0_r_swallow_after = -1;
    dut->slv_flush = 1;
    for (int i = 0; i < 5; i++) cycle();
    dut->slv_flush = 0;
    for (int i = 0; i < 10; i++) cycle();
}

// Scenario 35 (IMPORTANT 2b regression): the exact len==255 edge case
// the 8-bit-vs-9-bit remaining-beats bug hits.  S0 accepts the AR but
// delivers ZERO real beats before wedging (rs_beats_done stays 0), so
// the "remaining" computation is (255+1)-0 = 256 — the one value an
// 8-bit computation cannot represent distinctly from a genuine
// zero-remaining, which is exactly what the "clamp to 1 if computed
// zero" safety net would misfire on without the 9-bit widening.  Must
// deliver all 256 beats, all SLVERR, RLAST on the 256th — NOT 1 beat.
static void scenario_35_read_timeout_len255_edge() {
    std::printf("\n=== Scenario 35: len=255 read-side watchdog edge case (IMPORTANT 2b regression) ===\n");
    uint32_t addr = 0x00435000u; // see scenario 33's comment on overlay-immune addressing
    s0_r_swallow_after = 0; // deliver nothing — hits the exact 8-bit-wrap ambiguity
    int prev_err = mb[0].r_errors;
    issue_read<0>(/*id*/ 0xD, addr, /*len*/ 255, /*size*/ 4, /*expect_rresp*/ 2 /*SLVERR*/);

    for (int i = 0; i < WD_WAIT_CYCLES; i++) cycle();
    check("len=255 edge case: watchdog fires, read completes (terminal beat SLVERR)",
          mb[0].r_errors == prev_err + 1);
    check("len=255 edge case: read R-expect queue drained", mb[0].r_expect.empty());
    check("len=255 edge case: master received exactly 256 total beats (NOT clamped to 1)",
          !mb[0].r_received.empty() && mb[0].r_received.back().size() == 256);
    bool all_slverr = !mb[0].r_received_resp.empty() &&
                       mb[0].r_received_resp.back().size() == 256;
    if (all_slverr) {
        for (uint32_t r : mb[0].r_received_resp.back()) all_slverr = all_slverr && (r == 2);
    }
    check("len=255 edge case: all 256 beats are SLVERR", all_slverr);

    // Clean up (see scenario 34's comment).
    s0_r_swallow_after = -1;
    dut->slv_flush = 1;
    for (int i = 0; i < 5; i++) cycle();
    dut->slv_flush = 0;
    for (int i = 0; i < 10; i++) cycle();
}

// ─── Task-review round 2 scenarios (CRITICAL flush-domain-membership
// fix + IMPORTANT-3 slot-0 discard) ───────────────────────────────────

// Scenario 36 (2026-09-12, work item 3 -- INVERTED, deliberately): S1 IS
// in the flush domain now.  An in-flight S1 write (B swallowed, genuinely
// sitting in WS_WAIT_B) is flush-aborted with a quick local SLVERR, is NOT
// poisoned, and S1 reaches the real slave again once the window closes.
// Mirrors scenario 31 (S2) and 37 (S3).
//
// This test used to assert the exact OPPOSITE -- "S1 must NEVER be
// flush-aborted" -- and that was correct for the topology it was written
// against: S1's CDC bridge and peripheral_bus were deliberately kept ALIVE
// through soc_full_rst so the host could still reach debug_ctrl, which lived
// behind S1 at 0x5090_0000.  Flush-aborting a live bridge would have orphaned
// a real B and returned a spurious SLVERR from a slave about to answer.
//
// What changed: the debug window no longer goes through S1 (axi_dbg_bus serves
// it locally), so fpga_top_peripherals.vh now resets the S1 CDC and
// peripheral_bus on the soc_full_rst event -- the same event slv_flush carries.
// With the bridge genuinely in reset there is no live bridge to orphan, and
// SLVERR is the truthful answer.  The old contract had no release path at all
// once ENABLE_WD went to 0, which is the measured hardware defect: after a JTAG
// `reset`, dbg_s1_slot_busy = 1 with dbg_slv_poisoned = 0x00 -- stranded, not
// poisoned.
static void scenario_36_s1_flush_abort() {
    std::printf("\n=== Scenario 36: S1 IS flush-aborted -- quick SLVERR, no poison, usable after ===\n");
    uint32_t addr = 0x50005000u;
    Word128 payload = w128_from_u64(0x1111222233334444ULL);

    s1_swallow_b = true;
    int prev_ok  = mb[1].w_completed_ok;
    int prev_err = mb[1].w_completed_errs;
    issue_write<1>(/*id*/ 0xA, addr, 0, 4, {payload}, /*expect_bresp*/ 2 /*SLVERR -- predicted flush-abort response*/);
    check("S1 write not yet completed (B swallowed)",
          mb[1].w_completed_ok == prev_ok && mb[1].w_completed_errs == prev_err &&
          !mb[1].w_expect.empty());

    dut->slv_flush = 1;
    for (int i = 0; i < 20; i++) cycle();
    check("S1 flush-abort completes quickly with SLVERR",
          mb[1].w_completed_errs == prev_err + 1 && mb[1].w_completed_ok == prev_ok);
    check("S1 write B-expect queue drained after flush-abort", mb[1].w_expect.empty());

    dut->slv_flush = 0;
    s1_swallow_b   = false;
    // Same test-model caveat as scenario 31 (S2): the BFM still has
    // b_pending latched for the ABORTED transaction, and this single-slot
    // model gates a NEW write's W beats on !b_pending, so leaving it set
    // would wedge the TEST, not the RTL.  In the real design there is no
    // stale B at all -- the S1 CDC and peripheral_bus are both in reset
    // for the whole slv_flush window, exactly like S2/S4/S5's
    // axi_wide_to_axilite bridges, so nothing downstream is left holding
    // a response.  (That is precisely why S1 can now be flush-aborted at
    // all; when the bridge was kept ALIVE across soc_full_rst, an orphaned
    // real B would have aliased onto S1's next write.)  The xbar ignores
    // it either way: s1_bready stays low because sw_owned[IO] was released
    // and is never re-armed against the aborted transaction.
    s1_bfm.b_pending = false;
    for (int i = 0; i < 10; i++) cycle();

    // Not poisoned: a fresh write must reach the REAL slave.
    uint32_t addr2 = 0x50005100u;
    Word128 payload2 = w128_from_u64(0x5555666677778888ULL);
    prev_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0xE, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("S1 not poisoned by the flush-abort: post-flush write succeeds OKAY",
          mb[1].w_completed_ok == prev_ok + 1);
    check("S1 not poisoned by the flush-abort: post-flush write reached the real slave",
          s1_bfm.seen_aw_count[addr2] >= 1 && w128_eq(s1_mem.read(addr2), payload2));
}

// Scenario 37 (CRITICAL fix, item 4b): S3 (VRAM) IS in the flush
// domain (is_flush_domain_slv(), even though it's burst-capable and so
// excluded from is_lite_only_slv()) — an in-flight S3 write gets a
// quick flush-abort SLVERR, no poison, and S3 is fully usable (reaches
// the real slave) once flush deasserts.  Mirrors scenario 31's S2 test.
static void scenario_37_s3_flush_abort() {
    std::printf("\n=== Scenario 37: S3 (VRAM) IS flush-aborted -- quick SLVERR, no poison, usable after ===\n");
    uint32_t addr = 0xF9001000u; // VRAM aperture, distinct from scenario 23's addresses
    Word128 payload = w128_from_u64(0xCAFEBABECAFEBABEULL);

    s3_swallow_b = true;
    int prev_err = mb[1].w_completed_errs;
    issue_write<1>(/*id*/ 0xB, addr, 0, 4, {payload}, /*expect_bresp*/ 2 /*SLVERR -- predicted flush-abort response*/);
    check("S3 write not yet completed (B swallowed)",
          mb[1].w_completed_errs == prev_err && !mb[1].w_expect.empty());

    dut->slv_flush = 1;
    for (int i = 0; i < 20; i++) cycle();
    check("S3 flush-abort completes quickly with SLVERR",
          mb[1].w_completed_errs == prev_err + 1);
    check("S3 write B-expect queue drained after flush-abort", mb[1].w_expect.empty());

    dut->slv_flush = 0;
    s3_swallow_b = false;
    int completions_after_local_b = mb[1].w_completed_ok + mb[1].w_completed_errs;
    for (int i = 0; i < 20; i++) cycle();
    check("S3 stale backend B consumed after the local flush response",
          !s3_bfm.b_pending);
    check("S3 stale backend B did not leak a second master completion",
          mb[1].w_completed_ok + mb[1].w_completed_errs == completions_after_local_b);

    // Not poisoned: a fresh write to S3 must succeed for REAL (reaches
    // the real slave).  Not checking data equality here -- the xbar
    // byte-swaps S3 writes (vram_swap_words), so a raw s3_mem compare
    // against the un-swapped payload would need to account for that;
    // reaching-the-real-slave is sufficient proof of "not poisoned".
    uint32_t addr2 = 0xF9001100u;
    Word128 payload2 = w128_from_u64(0xD0D0D0D0D0D0D0D0ULL);
    int prev_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0xC, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("S3 not poisoned by the flush-abort: post-flush write succeeds OKAY",
          mb[1].w_completed_ok == prev_ok + 1);
    // The xbar strips VRAM_BASE before forwarding to S3 (see
    // vram_flatten() / scenario 23's identical pattern) -- the real
    // BFM's seen_aw_count key is the STRIPPED offset, not the raw
    // system address.
    check("S3 not poisoned by the flush-abort: post-flush write reached the real slave",
          s3_bfm.seen_aw_count[addr2 - 0xF9000000u] >= 1);

    // Read-side twin: deliver exactly one beat of an accepted four-beat DDR
    // response, flush on the resulting mid-burst stall, then expose the
    // stale tail.  This checks both same-cycle remaining-beat accounting and
    // that cleanup consumes the backend through RLAST before a fresh S3 AR.
    uint32_t raddr = 0xF9001200u;
    s3_r_swallow_after = 1;
    int prev_rerr = mb[1].r_errors;
    issue_read<1>(/*id*/ 0xD, raddr, /*len*/ 3, /*size*/ 4,
                  /*expect_rresp*/ 2);
    check("S3 read remains pending while backend R is withheld",
          mb[1].r_errors == prev_rerr && !mb[1].r_expect.empty() && s3_bfm.ar_busy);

    dut->slv_flush = 1;
    for (int i = 0; i < 20; i++) cycle();
    check("S3 read flush-abort completes locally with SLVERR",
          mb[1].r_errors == prev_rerr + 1 && mb[1].r_expect.empty());
    bool exact_read_tail = !mb[1].r_received_resp.empty() &&
                           mb[1].r_received_resp.back().size() == 4 &&
                           mb[1].r_received_resp.back()[0] == 0;
    if (exact_read_tail) {
        for (size_t i = 1; i < 4; i++)
            exact_read_tail = exact_read_tail &&
                              (mb[1].r_received_resp.back()[i] == 2);
    }
    check("S3 read flush preserves one real beat and synthesizes exactly the remaining three",
          exact_read_tail);
    dut->slv_flush = 0;
    s3_r_swallow_after = -1;
    int read_completions_after_local = mb[1].r_completed + mb[1].r_errors;
    for (int i = 0; i < 20; i++) cycle();
    check("S3 stale backend R burst consumed through RLAST", !s3_bfm.ar_busy);
    check("S3 stale backend R did not leak a second master completion",
          mb[1].r_completed + mb[1].r_errors == read_completions_after_local);

    int prev_rok = mb[1].r_completed;
    issue_read<1>(/*id*/ 0xE, raddr + 0x100, /*len*/ 0, /*size*/ 4,
                  /*expect_rresp*/ 0);
    check("S3 remains usable after stale read quarantine",
          mb[1].r_completed == prev_rok + 1 &&
          s3_bfm.seen_ar_count[(raddr + 0x100) - 0xF9000000u] >= 1);
}

// Scenario 38 (IMPORTANT 3, item 4c): a flush-abort completing on
// master slot 0 (the CPU-LSU/boot-FSM SHARED physical port) while
// cpu_held_in_reset is asserted must be discarded silently -- no B
// ever presented on EITHER m0 (blocked structurally by the mux itself
// once cpu_held_in_reset flips, so this is a weaker check) or,
// critically, m0b/boot_fsm's view (which the mux WOULD otherwise route
// a locally-synthesized B to, since cpu_held_in_reset selects boot's
// side -- this is the actual hazard IMPORTANT 3 fixes: boot_fsm's
// reactive BREADY accounting mis-consuming a B that was never really
// meant for it).
static void scenario_38_slot0_discard_under_cpu_held() {
    std::printf("\n=== Scenario 38: slot-0 response discarded while cpu_held_in_reset is asserted ===\n");
    uint32_t addr = 0x50104500u; // S2 (flush-domain), issued via M0 this time
    Word128 payload = w128_from_u64(0x5A5A5A5A5A5A5A5AULL);

    s2_swallow_b = true;
    int prev_m0_ok  = mb[0].w_completed_ok;
    int prev_m0_err = mb[0].w_completed_errs;
    int prev_m3_ok  = mb[3].w_completed_ok;
    int prev_m3_err = mb[3].w_completed_errs;
    issue_write<0>(/*id*/ 0xD, addr, 0, 4, {payload}, /*expect_bresp*/ 0 /*irrelevant -- must never be delivered to ANYONE*/);
    check("slot-0 write not yet completed (S2 B swallowed)",
          mb[0].w_completed_ok == prev_m0_ok && mb[0].w_completed_errs == prev_m0_err &&
          !mb[0].w_expect.empty());

    // The CPU goes into reset WHILE this write is still outstanding --
    // exactly the race IMPORTANT 3 exists for.  cpu_held_in_reset flips
    // the M0/boot mux, so from here the shared physical port's B/R
    // channels present to boot_fsm's (m0b) view, not m0's.
    set_cpu_held_in_reset(true);

    dut->slv_flush = 1;
    for (int i = 0; i < 20; i++) cycle();
    dut->slv_flush = 0;
    for (int i = 0; i < 20; i++) cycle();

    check("slot-0 (m0) observed no completion at all -- expectation still queued, discarded not delivered",
          !mb[0].w_expect.empty() &&
          mb[0].w_completed_ok == prev_m0_ok && mb[0].w_completed_errs == prev_m0_err);
    check("the discard did NOT leak onto boot_fsm's (m0b) accounting either -- the actual hazard being fixed",
          mb[3].w_completed_ok == prev_m3_ok && mb[3].w_completed_errs == prev_m3_err);

    // Clean up: release cpu_held_in_reset and drop the now-permanently
    // (and correctly) discarded expectation so it doesn't confuse any
    // later scenario's observe_b() FIFO-order bookkeeping on mb[0].
    set_cpu_held_in_reset(false);
    mb[0].w_expect.clear();
    s2_swallow_b     = false;
    s2_bfm.b_pending = false; // real BFM's own bookkeeping, see scenario 31's comment
}

// Scenario 30: per-slave outstanding-transaction watchdog + poison.  S1's
// slave model accepts AW+W normally but never drives BVALID (swallowed —
// "a slave that never returns B", the exact case the watchdog exists
// for).  After WD_MAX_CNT+1 (2^WD_LOG2 — 4096 cycles as this tb build
// overrides WD_LOG2) cycles the xbar must abort: synthesize BRESP=SLVERR
// locally, release the slot, and latch S1 poisoned so every later
// access (write OR read, from any master) gets immediate local SLVERR
// without ever reaching the real slave — while other slaves stay
// completely unaffected.
// ─── Scenario 29b (task #219): master walks away in WS_WAIT_B ──────────
//
// A THIRD abandonment shape, distinct from scenario 30 (slave never
// answers -> poison) and from scenario 42c (slot-0 m0_abandon, which has
// its own dedicated escape): the SLAVE answers perfectly normally, but
// the owning MASTER never asserts BREADY again.
//
// That is exactly what axi_narrow_to_wide.v produces whenever its own
// abandonment watchdog force-clears aw_valid_q — its w_bready is
// `(aw_valid_q && n_bready) || aw_drain_q`, so it goes low and stays low
// — and the JTAG-AXI host sits behind such an instance (u_jtag_n2w in
// fpga_top_debug_host.vh).  M1 is that host port here, and unlike M0 it
// has NO m0_abandon-style escape hatch.
//
// Before the fix, ws_wd_fire was suppressed by `!mw_bvalid_slv[mi]` —
// BVALID alone, unbounded, independent of BREADY.  So the watchdog whose
// entire job is to rescue this slot was held off by the very B nobody
// would ever take: sw_owned[1] was pinned at 1 permanently and S1
// (VIA/SCC/SCSI on real hardware) went dead to EVERY master, including
// the CPU, until the next rst.
//
// Sensitivity: restore the old `!mw_bvalid_slv[mi]` suppression term in
// axi_xbar.v's gen_ws_wd and the last four checks below fail (verified
// by tampering).  The "still stuck before the watchdog window" check is
// the positive control — it proves the scenario really does construct
// the parked state rather than sailing past it.
static void scenario_29b_wait_b_master_walks_away() {
    std::printf("\n=== Scenario 29b: master walks away in WS_WAIT_B -> slave lock released, no poison ===\n");
    const uint32_t addr = 0x50006000u;
    const Word128 payload = w128_from_u64(0x1234'5678'9ABC'DEF0ULL);

    const int prev_ok  = mb[1].w_completed_ok;
    const int prev_err = mb[1].w_completed_errs;

    // Take M1's BREADY away for the whole transaction.  issue_write<1>
    // only drives AW/W and returns; it never waits on B.
    dut->m1_bready = 0;
    issue_write<1>(/*id*/ 0x44, addr, 0, 4, {payload}, /*expect_bresp*/ 0);

    check("29b: the write reached the real S1 slave and its data landed",
          w128_eq(s1_mem.read(addr), payload));
    check("29b: S1 has a real B outstanding that the xbar cannot deliver",
          s1_bfm.b_pending);

    // POSITIVE CONTROL for the whole scenario: well past any BREADY a
    // live master would ever need (B_HOLD_MAX is 63 in this build), but
    // still inside the watchdog window, the B must still be stuck.  If
    // this ever passes trivially the scenario is not testing anything.
    for (int i = 0; i < 200; i++) cycle();
    check("29b: [positive control] still parked before the watchdog window elapses",
          s1_bfm.b_pending);

    for (int i = 0; i < WD_WAIT_CYCLES; i++) cycle();

    check("29b: the dangling B was retired at the slave port",
          !s1_bfm.b_pending);
    check("29b: nothing was delivered to the walked-away master",
          mb[1].w_completed_ok == prev_ok &&
          mb[1].w_completed_errs == prev_err);

    // The master's expectation can never be satisfied — by design, this
    // path deliberately synthesizes no B for a master that is gone.
    mb[1].w_expect.clear();

    // NOTE: m1_bready deliberately stays LOW for the rest of the
    // scenario.  A master that has walked away does not come back, and
    // restoring BREADY here would itself release the stuck B — which
    // would make the recovery checks below pass for the wrong reason.

    // THE PROPERTY THAT MATTERS: S1 is usable again, by a DIFFERENT
    // master, and was not poisoned in the process.
    const uint32_t addr2 = 0x50006100u;
    const Word128 payload2 = w128_from_u64(0x0F0F'F0F0'0F0F'F0F0ULL);
    const int prev_aw2 = s1_bfm.seen_aw_count[addr2];
    const int prev_m0_ok = mb[0].w_completed_ok;
    issue_write<0>(/*id*/ 0x45, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0 /*OKAY*/);
    check("29b: another master can write S1 again (sw_owned[1] was released)",
          mb[0].w_completed_ok == prev_m0_ok + 1 &&
          w128_eq(s1_mem.read(addr2), payload2));
    check("29b: that write reached the REAL slave (S1 was not poisoned)",
          s1_bfm.seen_aw_count[addr2] == prev_aw2 + 1);

    // And the read channel is equally healthy (poison is cross-channel,
    // so an accidental poison here would show up on reads too).
    const uint32_t addr3 = 0x50006200u;
    const int prev_ar3 = s1_bfm.seen_ar_count[addr3];
    const int prev_m0_r = mb[0].r_completed;
    issue_read<0>(/*id*/ 0x46, addr3, 0, 4, /*expect_rresp*/ 0 /*OKAY*/);
    check("29b: S1 reads still answered OKAY by the real slave",
          mb[0].r_completed == prev_m0_r + 1 &&
          s1_bfm.seen_ar_count[addr3] == prev_ar3 + 1);

    // Restore the port for the scenarios that follow.  Nothing is
    // outstanding on it, so this delivers nothing.
    dut->m1_bready = 1;
    for (int i = 0; i < 8; i++) cycle();
}

static void scenario_30_slave_watchdog_poison() {
    std::printf("\n=== Scenario 30: per-slave watchdog timeout -> SLVERR + poison ===\n");
    uint32_t addr = 0x50004000u;
    Word128 payload = w128_from_u64(0x5555AAAA5555AAAAULL);

    s1_swallow_b = true;
    int prev_err = mb[1].w_completed_errs;
    // issue_write<> only drives the AW/W handshake and returns — it does
    // NOT block waiting for B — so this returns quickly even though the
    // real S1 BFM will never produce a BVALID (swallowed).  expect_bresp
    // is what we predict the watchdog will eventually synthesize.
    issue_write<1>(/*id*/ 0x4, addr, 0, 4, {payload}, /*expect_bresp*/ 2 /*SLVERR*/);
    check("write not yet completed right after issue (slave swallowed B)",
          mb[1].w_completed_errs == prev_err && !mb[1].w_expect.empty());

    // Run past the watchdog window (WD_WAIT_CYCLES, comfortably past
    // this tb build's WD_MAX_CNT+1 = 4096), with margin for the cycles
    // issue_write<>'s own AW/W handshake and tail already consumed
    // while ws_state sat in {WAIT_SLV_AW, FWD_W, WAIT_B}.
    for (int i = 0; i < WD_WAIT_CYCLES; i++) cycle();

    check("watchdog fired: write B eventually arrives as SLVERR",
          mb[1].w_completed_errs == prev_err + 1);
    check("write B queue drained (no dangling expectation)", mb[1].w_expect.empty());

    // Other slaves unaffected: DDR (S0) still works normally.
    uint32_t ddr_addr = 0x00033000u;
    Word128 ddr_payload = w128_from_u64(0x0BADF00D0BADF00DULL);
    int prev_ddr_ok = mb[0].w_completed_ok;
    issue_write<0>(/*id*/ 0x5, ddr_addr, 0, 4, {ddr_payload}, /*expect_bresp*/ 0);
    check("DDR (S0) unaffected by S1 watchdog/poison",
          mb[0].w_completed_ok == prev_ddr_ok + 1 &&
          w128_eq(s0_mem.read(ddr_addr), ddr_payload));

    // Poison: any subsequent write to S1 gets immediate local SLVERR,
    // never reaching the real slave.
    uint32_t addr2 = 0x50004100u;
    int prev_aw2  = s1_bfm.seen_aw_count[addr2];
    int prev_err2 = mb[1].w_completed_errs;
    issue_write<1>(/*id*/ 0x6, addr2, 0, 4, {payload}, /*expect_bresp*/ 2 /*SLVERR*/);
    check("post-poison write to S1 answered SLVERR immediately",
          mb[1].w_completed_errs == prev_err2 + 1);
    check("post-poison write never reached the real S1 slave",
          s1_bfm.seen_aw_count[addr2] == prev_aw2);

    // Poison is per-slave, not per-channel: a read to S1 must also now
    // get immediate SLVERR (validates the write-side and read-side
    // poison latches are OR'd together at the use sites).
    uint32_t addr3 = 0x50004200u;
    int prev_ar3  = s1_bfm.seen_ar_count[addr3];
    int prev_rerr = mb[0].r_errors;
    issue_read<0>(/*id*/ 0x7, addr3, 0, 4, /*expect_rresp*/ 2 /*SLVERR*/);
    check("post-poison read from S1 also answered SLVERR (cross-channel poison)",
          mb[0].r_errors == prev_rerr + 1);
    check("post-poison read never reached the real S1 slave",
          s1_bfm.seen_ar_count[addr3] == prev_ar3);

    // Task review MINOR 8: "late-B released after poison, verified
    // unrouted".  s1_bfm.b_pending has stayed latched internally this
    // whole time (only the DRIVE of s1_bvalid was suppressed by
    // s1_swallow_b, never the BFM's own completion state) — un-swallow
    // it now and confirm the xbar drops the stale response safely: no
    // new completion appears on any master, and S1 stays exactly as
    // poisoned/SLVERR-only as before.  This is the concrete proof for
    // the "sw_owned is now 0 and never re-armed against the real slave"
    // argument in ws_release_slot's caller comments.
    s1_swallow_b = false;
    int prev_wok_late  = mb[1].w_completed_ok;
    int prev_werr_late = mb[1].w_completed_errs;
    for (int i = 0; i < 50; i++) cycle();
    check("late (post-poison) B from the originally-wedged slave produces no new completion",
          mb[1].w_completed_ok == prev_wok_late && mb[1].w_completed_errs == prev_werr_late);

    uint32_t addr5 = 0x50004300u;
    int prev_err5 = mb[1].w_completed_errs;
    int prev_aw5  = s1_bfm.seen_aw_count[addr5];
    issue_write<1>(/*id*/ 0x9, addr5, 0, 4, {payload}, /*expect_bresp*/ 2 /*SLVERR*/);
    check("S1 still poisoned after the late B (a fresh write still gets immediate SLVERR)",
          mb[1].w_completed_errs == prev_err5 + 1);
    check("S1 still poisoned after the late B (fresh write still never reaches the real slave)",
          s1_bfm.seen_aw_count[addr5] == prev_aw5);
}

// ─── main ───────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────
// Scenarios 39-41: multi-beat 128-bit INCR bursts on the CPU data path,
// and the M0/boot fan-in mux flipping in the middle of one.
//
// WHY THESE EXIST.  Until 2026-07-30 no master on this fabric could emit a
// multi-beat transaction: the CPU's 32->128-bit socket widener
// (cpu/rtl/sys/axi_narrow_to_wide.v) had no n_arlen/n_awlen ports and
// hardcoded the wide-side arlen/awlen to 8'd0, boot_fsm.v drives awlen=0 /
// wlast=1, and if_to_axi.v drives arlen=0.  With the CPU's D-cache burst
// refill now working, M0 becomes the first multi-beat master this crossbar
// has ever carried: ar/awsize=3'd4, INCR, arlen/awlen = beats-1, WLAST on
// the final beat only, one B per burst.
//
// The existing burst coverage in this file only ever exercises the REJECT
// and TIMEOUT paths (scenarios 28/29 burst-into-a-lite-only-slave, 33/34/35
// mid-burst watchdog).  Nothing here ever completed a clean multi-beat
// burst against a burst-capable slave, which is the shape that now matters.
// ─────────────────────────────────────────────────────────────────────────

// Scenarios 33-35 deliberately leave S0 in a "the far side forgot"
// state (aw_busy stuck, stall counters set) and scenario 26b leaves
// cpu_overlay_active asserted, which redirects M0/M2 READS below
// ROM_SIZE (0x40_0000) into ROM space.  The burst scenarios below both
// read back their own writes and assert on the S0 BFM's beat accounting,
// so they need a pristine S0 and overlay-immune addresses.
static constexpr uint32_t BURST_ADDR_BASE = 0x00450000u; // >= ROM_SIZE
static void s0_bfm_fresh() {
    s0_bfm             = SlaveBfm{};
    s0_stall_aw        = 0;
    s0_stall_w         = 0;
    s0_r_swallow_after = -1;
    s0_early_w_accept  = false;
    s0_early_w_have    = false;
}

// Non-blocking primitives.  issue_write<M>() is blocking and drops the
// valids at the end, so it cannot express "stop half way through the W
// burst and change an input".  These let a scenario own the burst cycle by
// cycle.
static bool m0_aw_handshake(uint32_t id, uint32_t addr, uint8_t len,
                            uint8_t size, int max_cycles = 200) {
    dut->m0_awid = id; dut->m0_awaddr = addr; dut->m0_awlen = len;
    dut->m0_awsize = size; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    for (int i = 0; i < max_cycles; i++) {
        dut->eval();
        if (dut->m0_awready) { cycle(); dut->m0_awvalid = 0; return true; }
        cycle();
    }
    dut->m0_awvalid = 0;
    return false;
}

static bool m0_w_beat(const Word128& d, bool last, uint16_t strb = 0xFFFF,
                      int max_cycles = 200) {
    set_wdata(dut->m0_wdata, d);
    dut->m0_wstrb = strb;
    dut->m0_wlast = last ? 1 : 0;
    dut->m0_wvalid = 1;
    for (int i = 0; i < max_cycles; i++) {
        dut->eval();
        if (dut->m0_wready) { cycle(); dut->m0_wvalid = 0; dut->m0_wlast = 0; return true; }
        cycle();
    }
    dut->m0_wvalid = 0; dut->m0_wlast = 0;
    return false;
}

// Scenario 39: a clean multi-beat INCR write + read against S0 (DDR), the
// baseline shape the CPU now emits.  Proves arlen/awlen are forwarded
// verbatim, every beat lands 16 B apart, RLAST arrives on the last beat
// only, and exactly one B comes back per burst.
static void scenario_39_cpu_multibeat_burst_ddr() {
    std::printf("\n=== Scenario 39: M0 multi-beat 128-bit INCR burst to S0 (DDR) ===\n");
    s0_bfm_fresh();
    const uint32_t base = BURST_ADDR_BASE;
    const int      kLens[] = {2, 4, 8};

    for (int beats : kLens) {
        const uint32_t addr = base + (uint32_t)beats * 0x100u;
        std::vector<Word128> payload;
        for (int b = 0; b < beats; b++)
            payload.push_back(w128_from_u64(0xB0B0'0000'0000'0000ULL +
                                            (uint64_t)beats * 0x10000 + b));

        int prev_ok = mb[0].w_completed_ok, prev_err = mb[0].w_completed_errs;
        issue_write<0>(/*id*/ 0x1, addr, (uint8_t)(beats - 1), /*size*/ 4,
                       payload, /*expect_bresp*/ 0);

        char nm[128];
        std::snprintf(nm, sizeof(nm),
                      "len=%d write: exactly one BRESP=OKAY (no B per beat)", beats);
        check(nm, mb[0].w_completed_ok == prev_ok + 1 &&
                  mb[0].w_completed_errs == prev_err);

        // Every beat must have landed at its own 16 B-strided address.  A
        // hop that ignored the address stride and wrote every beat to the
        // burst base would pass a burst read-back but fails here.
        bool strided = true;
        for (int b = 0; b < beats; b++)
            strided = strided && w128_eq(s0_mem.read(addr + (uint32_t)b * 16),
                                         payload[b]);
        std::snprintf(nm, sizeof(nm),
                      "len=%d write: every beat landed at base+16*b", beats);
        check(nm, strided);

        size_t prev_r = mb[0].r_received.size();
        issue_read<0>(/*id*/ 0x2, addr, (uint8_t)(beats - 1), /*size*/ 4,
                      /*rresp*/ 0);
        bool got = (mb[0].r_received.size() == prev_r + 1) &&
                   ((int)mb[0].r_received.back().size() == beats);
        // r_received is only appended on RLAST, so a beat-count match is
        // also an RLAST-placement check.
        for (int b = 0; b < beats && got; b++)
            got = got && w128_eq(mb[0].r_received.back()[b], payload[b]);
        std::snprintf(nm, sizeof(nm),
                      "len=%d read: exactly %d beats, RLAST on the last, data in order",
                      beats, beats);
        check(nm, got);
    }
}

// Scenario 40: cpu_held_in_reset rises BETWEEN beats of an in-flight M0
// write burst, while boot_fsm has a W beat presented.
//
// THE HAZARD.  The M0 fan-in is a 2:1 mux on cpu_held_in_reset
// (axi_xbar.v ~:658-685).  Before this change it was purely
// combinational, so it re-selected mid-transaction: the slot stays in
// WS_FWD_W and keeps driving the owned slave's W channel, but wdata/wstrb/
// wlast/wvalid now source m0b_*, and m0b_wready goes high.  Boot's beat is
// then consumed as the CPU burst's continuation -- boot's data lands at
// the CPU's address, and because m0b_bvalid is also selected, boot books a
// completion for a write whose AW the crossbar never accepted.  Same
// failure class as #162 (a426f7b), one hop further in.
//
// This is reachable on real hardware: cpu_held_in_reset is driven by
// `cpu_rst` (fpga_top_xbar.vh:258), which ORs in `dbg_cold_reset_hold` --
// DBG_CONTROL bit 4, the JTAG `reset hold` path (fpga_top_clocks.vh:779)
// -- and that bit raises cpu_rst with the crossbar's own rst
// (core_rst_bank[4]) LOW and slv_flush deasserted.  With single-beat
// writes the exposure was the slave's WREADY latency, a cycle or two.
// With a multi-beat burst it is the whole burst.
static void scenario_40_cpu_held_flips_mid_w_burst() {
    std::printf("\n=== Scenario 40: cpu_held_in_reset flips mid-W-burst; boot's beat must not be stolen ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t addr = BURST_ADDR_BASE + 0x2000u;
    const Word128 beat0  = w128_from_u64(0xC0DE'0000'0000'0001ULL);
    const Word128 poison = w128_from_u64(0xDEAD'BEEF'DEAD'BEEFULL);

    // Prime both target words with a known sentinel so "was it written"
    // is unambiguous.
    const Word128 sentinel = w128_from_u64(0x1111'2222'3333'4444ULL);
    s0_mem.write(addr,      sentinel);
    s0_mem.write(addr + 16, sentinel);

    // A 2-beat CPU burst -- the D-cache's 32 B line.
    check("mid-burst: M0 AW (len=1) accepted", m0_aw_handshake(0x7, addr, 1, 4));
    check("mid-burst: M0 W beat 0 accepted",   m0_w_beat(beat0, /*last*/ false));

    // S0 has the burst open with one beat still owed.
    check("mid-burst: S0 burst open with 1 beat still owed",
          s0_bfm.aw_busy && s0_bfm.w_beats_left == 1);

    int prev_boot_ok  = mb[3].w_completed_ok;
    int prev_boot_err = mb[3].w_completed_errs;

    // The CPU is yanked into reset. Its own W channel goes quiet (that is
    // what reset does) and boot_fsm -- which is now the selected side --
    // has a beat of its own ready to go, with NO AW accepted for it.
    dut->m0_wvalid = 0; dut->m0_wlast = 0;
    dut->cpu_held_in_reset = 1;
    set_wdata(dut->m0b_wdata, poison);
    dut->m0b_wstrb  = 0xFFFF;
    dut->m0b_wlast  = 1;
    dut->m0b_wvalid = 1;
    dut->m0b_awvalid = 0;      // deliberately no AW from boot
    // cycle() steps the slave BFMs BEFORE its own eval(), so it samples
    // whatever the previous eval() produced.  Evaluate here so the BFM
    // sees the new mux select and boot's beat rather than a stale copy of
    // the CPU's beat 0 -- without this the harness itself double-captures
    // beat 0 and the scenario would "fail" for a testbench reason.
    dut->eval();
    for (int i = 0; i < 40; i++) cycle();

    // The crossbar now completes the abandoned burst itself (GAP 1), so
    // the burst DOES close — but beat 1 must be the crossbar's own WSTRB=0
    // filler, never boot's payload beat.  (This check used to read
    // `w_beats_left == 1`, i.e. "the burst is left open forever" — which
    // was true only because nothing padded it; asserting that today would
    // be asserting the absence of the GAP-1 fix.)
    check("boot's W beat was NOT consumed as the CPU burst's beat 1",
          s0_bfm.w_beats_left == 0 && s0_bfm.w_beats_seen == 2 &&
          s0_bfm.w_pad_beats == 1 && s0_bfm.w_wlast_errors == 0);
    if (w128_eq(s0_mem.read(addr + 16), poison)) {
        Word128 got = s0_mem.read(addr + 16);
        std::printf("    S0[0x%08x] = %08x_%08x_%08x_%08x  <-- boot's payload, "
                    "written at the CPU's address\n",
                    addr + 16, got[3], got[2], got[1], got[0]);
    }
    check("boot's payload did NOT land at the CPU burst's beat-1 address",
          !w128_eq(s0_mem.read(addr + 16), poison));
    // NOTE: an unexpected BVALID on the wrong master is ALSO reported by
    // the harness's own observe_b() error path (which prints
    // "M3 got unexpected BVALID"), so the counters below staying put is a
    // necessary but not sufficient check -- watch for that line too.
    check("boot did NOT receive a B for a write whose AW was never accepted",
          mb[3].w_completed_ok == prev_boot_ok &&
          mb[3].w_completed_errs == prev_boot_err &&
          !dut->m0b_bvalid);

    // Cleanup.  (Historical note: this scenario used to leave the abandoned
    // burst for the watchdog, because the crossbar had no W padding and the
    // slot inevitably timed out and poisoned S0.  Since GAP 1 landed the
    // burst is padded closed within a handful of cycles and S0 stays
    // healthy -- scenario 42 asserts that directly.)  Reset the DUT and the
    // BFMs so later scenarios start clean, and drop the CPU's
    // now-unanswerable expectation.
    dut->m0b_wvalid = 0; dut->m0b_wlast = 0;
    dut->cpu_held_in_reset = 0;
    mb[0].w_expect.clear();
    s0_bfm_fresh();
    reset();
}

// Scenario 41: the mirror image -- a BOOT burst in flight when
// cpu_held_in_reset FALLS.  Symmetric hazard: the CPU's W channel must not
// be able to feed boot's open burst, and the CPU must not receive boot's B.
// Boot FSM is single-beat today (boot_fsm.v:1488 awlen=0), so this leg is
// forward-looking rather than a live exposure -- but it is the same mux and
// costs nothing to pin down.
static void scenario_41_boot_burst_survives_cpu_release() {
    std::printf("\n=== Scenario 41: cpu_held_in_reset falls mid-W-burst on a boot burst ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(true);

    const uint32_t addr = BURST_ADDR_BASE + 0x3000u;
    const Word128 bbeat0 = w128_from_u64(0xB007'0000'0000'0001ULL);
    const Word128 cpoison = w128_from_u64(0xFEED'FACE'FEED'FACEULL);
    const Word128 sentinel = w128_from_u64(0x5555'6666'7777'8888ULL);
    s0_mem.write(addr, sentinel);
    s0_mem.write(addr + 16, sentinel);

    // Boot AW, 2 beats, then only beat 0.
    dut->m0b_awid = 0x9; dut->m0b_awaddr = addr; dut->m0b_awlen = 1;
    dut->m0b_awsize = 4; dut->m0b_awburst = 1; dut->m0b_awvalid = 1;
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0b_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m0b_awvalid = 0;
    check("boot AW (len=1) accepted", aw_ok);

    set_wdata(dut->m0b_wdata, bbeat0);
    dut->m0b_wstrb = 0xFFFF; dut->m0b_wlast = 0; dut->m0b_wvalid = 1;
    bool w_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0b_wready) { cycle(); w_ok = true; break; }
        cycle();
    }
    dut->m0b_wvalid = 0;
    check("boot W beat 0 accepted", w_ok);
    check("S0 burst open with 1 beat still owed", s0_bfm.aw_busy && s0_bfm.w_beats_left == 1);

    int prev_cpu_ok  = mb[0].w_completed_ok;
    int prev_cpu_err = mb[0].w_completed_errs;

    // cpu_held_in_reset falls with the burst still open, and the CPU LSU
    // presents a beat of its own with no AW accepted.
    dut->cpu_held_in_reset = 0;
    set_wdata(dut->m0_wdata, cpoison);
    dut->m0_wstrb = 0xFFFF; dut->m0_wlast = 1; dut->m0_wvalid = 1;
    dut->m0_awvalid = 0;
    dut->eval();   // see the cycle()-ordering note in scenario 40
    for (int i = 0; i < 40; i++) cycle();

    // See the same check in scenario 40: the burst is now padded closed by
    // the crossbar, so what must be proven is that beat 1 was FILLER and
    // not the CPU's payload.
    check("CPU's W beat was NOT consumed as boot's burst beat 1",
          s0_bfm.w_beats_left == 0 && s0_bfm.w_beats_seen == 2 &&
          s0_bfm.w_pad_beats == 1 && s0_bfm.w_wlast_errors == 0);
    check("CPU's payload did NOT land at boot's beat-1 address",
          !w128_eq(s0_mem.read(addr + 16), cpoison));
    check("CPU did NOT receive a B for a write it never issued",
          mb[0].w_completed_ok == prev_cpu_ok &&
          mb[0].w_completed_errs == prev_cpu_err &&
          !dut->m0_bvalid);

    dut->m0_wvalid = 0; dut->m0_wlast = 0;
    mb[3].w_expect.clear();
    s0_bfm_fresh();
    reset();
}

// ── Scenario 42 (task: xbar-burst-gaps, GAP 1) ─────────────────────────
// A slot-0 write burst whose OWNER is reset away mid-burst must be
// completed on the SLAVE side by the crossbar, not abandoned for the
// watchdog.
//
// Before this fix the slot simply sat in WS_FWD_W with the slave parked
// mid-burst.  2^WD_LOG2 cycles later the watchdog fired with poison=1 and
// S0 was SLVERR'd for EVERY master until the next rst — the whole DDR path
// died because one JTAG `reset hold` landed mid-writeback.  Exposure grew
// with bursts: it used to be one WREADY latency for a single beat, it is
// now the whole burst, on every D-cache writeback.
//
// Suppressing the poison is NOT the fix.  The slave genuinely IS left
// mid-burst, so a fresh write's beats would be consumed as the stale
// burst's continuation and physically commit to the OLD address — silent
// corruption, strictly worse than a loud death, and exactly what
// axi_bridge_w_pad.v exists to prevent one hop further out.  The crossbar
// therefore completes the burst itself with WSTRB=0 filler beats carrying
// a correct WLAST (WS_PAD_W), then swallows the slave's B (the owner is
// gone and must not be handed a response), releasing the slave cleanly.
//
// `stop_after` sweeps the beat index at which the abandonment lands.
static void scenario_42_abandoned_burst_is_padded(int beats, int stop_after) {
    std::printf("\n=== Scenario 42: slot-0 burst abandoned after %d of %d beats "
                "-> xbar pads the slave's burst ===\n", stop_after, beats);
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t addr = BURST_ADDR_BASE + 0x4000u +
                          (uint32_t)(beats * 16 + stop_after) * 0x100u;
    const Word128 sentinel = w128_from_u64(0x7E57'0000'DEAD'0000ULL);
    for (int b = 0; b < beats; b++) s0_mem.write(addr + (uint32_t)b * 16, sentinel);

    char nm[192];
    std::snprintf(nm, sizeof(nm), "abandon@%d/%d: M0 AW accepted", stop_after, beats);
    check(nm, m0_aw_handshake(0x21, addr, (uint8_t)(beats - 1), 4));

    std::vector<Word128> sent;
    bool w_ok = true;
    for (int b = 0; b < stop_after; b++) {
        Word128 d = w128_from_u64(0xABCD'0000'0000'0000ULL + (uint64_t)b);
        sent.push_back(d);
        w_ok = w_ok && m0_w_beat(d, /*last*/ false);
    }
    std::snprintf(nm, sizeof(nm), "abandon@%d/%d: first %d W beat(s) accepted",
                  stop_after, beats, stop_after);
    check(nm, w_ok);
    // For stop_after == 0 the slave has not necessarily captured the AW by
    // the cycle the xbar's own AW handshake completes, so poll briefly.
    for (int i = 0; i < 10 && !s0_bfm.aw_busy; i++) cycle();
    std::snprintf(nm, sizeof(nm), "abandon@%d/%d: S0 burst open with %d beat(s) owed",
                  stop_after, beats, beats - stop_after);
    check(nm, s0_bfm.aw_busy && s0_bfm.w_beats_left == beats - stop_after);

    const int prev_m0_ok   = mb[0].w_completed_ok;
    const int prev_m0_err  = mb[0].w_completed_errs;
    const int prev_m3_ok   = mb[3].w_completed_ok;
    const int prev_m3_err  = mb[3].w_completed_errs;

    // The CPU is yanked into reset mid-burst: its W channel goes quiet
    // (that is what reset does) and it will never consume a B either.
    dut->m0_wvalid = 0; dut->m0_wlast = 0;
    dut->cpu_held_in_reset = 1;
    dut->m0b_awvalid = 0; dut->m0b_wvalid = 0; dut->m0b_wlast = 0;
    dut->eval();   // see the cycle()-ordering note in scenario 40

    // (b) The completion must come from the pad, NOT the watchdog: bound
    // the wait far below WD_MAX_CNT+1 (4096 in this build).  A run that
    // needs the watchdog fails here rather than silently passing later.
    const int kPadBudget = 64;
    int cycles_to_done = -1;
    for (int i = 0; i < kPadBudget; i++) {
        cycle();
        if (!s0_bfm.aw_busy && s0_bfm.w_beats_left == 0 && !s0_bfm.b_pending) {
            cycles_to_done = i + 1;
            break;
        }
    }
    std::snprintf(nm, sizeof(nm),
                  "abandon@%d/%d: (a) slave's burst COMPLETED (not left open) "
                  "and its B consumed, in %d cycles (<< watchdog's 4096)",
                  stop_after, beats, cycles_to_done);
    check(nm, cycles_to_done > 0);

    std::snprintf(nm, sizeof(nm),
                  "abandon@%d/%d: (b) watchdog did NOT fire — exactly %d beats "
                  "reached the slave, %d of them WSTRB=0 filler",
                  stop_after, beats, beats, beats - stop_after);
    check(nm, s0_bfm.w_beats_seen == beats &&
              s0_bfm.w_pad_beats == beats - stop_after);

    std::snprintf(nm, sizeof(nm), "abandon@%d/%d: WLAST landed on the final beat only",
                  stop_after, beats);
    check(nm, s0_bfm.w_wlast_errors == 0);

    // The filler beats must be no-ops: the words the CPU never wrote keep
    // their sentinel, and the ones it did write keep the CPU's payload.
    bool mem_ok = true;
    for (int b = 0; b < beats; b++) {
        Word128 got = s0_mem.read(addr + (uint32_t)b * 16);
        mem_ok = mem_ok && w128_eq(got, (b < stop_after) ? sent[b] : sentinel);
    }
    std::snprintf(nm, sizeof(nm),
                  "abandon@%d/%d: filler beats wrote nothing (real beats intact)",
                  stop_after, beats);
    check(nm, mem_ok);

    std::snprintf(nm, sizeof(nm),
                  "abandon@%d/%d: no B leaked to EITHER side of the shared port",
                  stop_after, beats);
    check(nm, mb[0].w_completed_ok == prev_m0_ok &&
              mb[0].w_completed_errs == prev_m0_err &&
              mb[3].w_completed_ok == prev_m3_ok &&
              mb[3].w_completed_errs == prev_m3_err &&
              !dut->m0_bvalid && !dut->m0b_bvalid);

    // (c)+(d) S0 is NOT poisoned and the fabric still works: an unrelated
    // master's write must reach the real slave and land completely.  M1
    // (host debug) is used deliberately — it is independent of the M0/boot
    // mux, so this also proves the SLAVE, not merely the slot, recovered.
    const uint32_t addr2 = addr + 0x80u;
    const Word128 payload2 = w128_from_u64(0x600D'600D'600D'600DULL);
    int prev_m1_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0x22, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    std::snprintf(nm, sizeof(nm),
                  "abandon@%d/%d: (c) S0 not poisoned — later M1 write gets OKAY "
                  "and (d) lands correctly",
                  stop_after, beats);
    check(nm, mb[1].w_completed_ok == prev_m1_ok + 1 &&
              w128_eq(s0_mem.read(addr2), payload2));

    // ...and a full multi-beat burst still works end to end afterwards.
    dut->cpu_held_in_reset = 0;
    for (int i = 0; i < 4; i++) cycle();
    const uint32_t addr3 = addr + 0x100u;
    std::vector<Word128> payload3;
    for (int b = 0; b < 4; b++)
        payload3.push_back(w128_from_u64(0xFEED'0000'0000'0000ULL + (uint64_t)b));
    int prev_ok3 = mb[0].w_completed_ok;
    issue_write<0>(/*id*/ 0x23, addr3, 3, 4, payload3, /*expect_bresp*/ 0);
    bool strided3 = (mb[0].w_completed_ok == prev_ok3 + 1);
    for (int b = 0; b < 4; b++)
        strided3 = strided3 && w128_eq(s0_mem.read(addr3 + (uint32_t)b * 16), payload3[b]);
    std::snprintf(nm, sizeof(nm),
                  "abandon@%d/%d: (d) a fresh 4-beat burst lands completely afterwards",
                  stop_after, beats);
    check(nm, strided3);

    // The abandoned write's B is correctly never delivered — drop the
    // expectation so later scenarios' FIFO-order bookkeeping stays sane.
    mb[0].w_expect.clear();
    s0_bfm_fresh();
    reset();
}

// Scenario 42b: the abandonment lands while the slot is still in
// WS_WAIT_SLV_AW -- the slave has not accepted the AW yet.  The crossbar
// must NOT try to retract the AW (AXI4 A3.2.1 forbids dropping AWVALID
// before its handshake); it waits for the AW to be taken and pads from
// there.  Same end state: burst closed, no poison, no leaked B.
static void scenario_42b_abandon_with_aw_still_pending() {
    std::printf("\n=== Scenario 42b: abandonment while the slave's AW is still pending ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t addr = BURST_ADDR_BASE + 0x5000u;
    const Word128 sentinel = w128_from_u64(0x2222'3333'4444'5555ULL);
    for (int b = 0; b < 4; b++) s0_mem.write(addr + (uint32_t)b * 16, sentinel);

    // Hold S0's AW channel off for a bounded window, long enough that the
    // abandonment lands first but far short of the 4096-cycle watchdog.
    s0_stall_aw = 12;
    check("aw-pending: xbar accepted M0's AW (len=3)", m0_aw_handshake(0x41, addr, 3, 4));
    check("aw-pending: slave has NOT yet accepted the AW", !s0_bfm.aw_busy);

    const int prev_m0_ok  = mb[0].w_completed_ok;
    const int prev_m0_err = mb[0].w_completed_errs;
    const int prev_m3_ok  = mb[3].w_completed_ok;
    const int prev_m3_err = mb[3].w_completed_errs;

    dut->m0_wvalid = 0; dut->m0_wlast = 0; dut->m0_bready = 0;
    dut->cpu_held_in_reset = 1;
    dut->eval();

    int cycles_to_done = -1;
    for (int i = 0; i < 128; i++) {
        cycle();
        if (s0_bfm.w_beats_seen == 4 && !s0_bfm.aw_busy && !s0_bfm.b_pending) {
            cycles_to_done = i + 1;
            break;
        }
    }
    check("aw-pending: AW was taken, then all 4 beats padded out and the B "
          "swallowed (no watchdog)",
          cycles_to_done > 0 && s0_bfm.w_pad_beats == 4 &&
          s0_bfm.w_wlast_errors == 0);
    bool mem_ok = true;
    for (int b = 0; b < 4; b++)
        mem_ok = mem_ok && w128_eq(s0_mem.read(addr + (uint32_t)b * 16), sentinel);
    check("aw-pending: filler beats wrote nothing", mem_ok);
    check("aw-pending: no B leaked to either side",
          mb[0].w_completed_ok == prev_m0_ok &&
          mb[0].w_completed_errs == prev_m0_err &&
          mb[3].w_completed_ok == prev_m3_ok &&
          mb[3].w_completed_errs == prev_m3_err);

    dut->m0_bready = 1;
    const uint32_t addr2 = addr + 0x80u;
    const Word128 payload2 = w128_from_u64(0x1234'5678'9ABC'DEF0ULL);
    int prev_m1_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0x42, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("aw-pending: S0 not poisoned, later write lands",
          mb[1].w_completed_ok == prev_m1_ok + 1 &&
          w128_eq(s0_mem.read(addr2), payload2));

    dut->cpu_held_in_reset = 0;
    mb[0].w_expect.clear();
    s0_bfm_fresh();
    reset();
}

// Scenario 42c: the abandonment lands in WS_WAIT_B -- the W burst is fully
// through and only the response is outstanding.  This is the SINGLE-BEAT
// case, i.e. the shape that existed long before bursts.
//
// Before the fix there was nothing to pad, but the slot still died: the
// slave's B was offered to a master in reset that never asserts BREADY, and
// ws_wd_fire is deliberately suppressed while mw_bvalid_slv is high (MINOR
// 4, AXI4 BRESP stability), so not even the watchdog freed it.  sw_owned[0]
// stayed set forever and S0 was dead to every master.  m0_abandon now forces
// BREADY and masks both masters' BVALID so the response is swallowed.
static void scenario_42c_abandon_in_wait_b() {
    std::printf("\n=== Scenario 42c: abandonment in WS_WAIT_B (single beat, response outstanding) ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t addr = BURST_ADDR_BASE + 0x7000u;
    const Word128 payload = w128_from_u64(0x9999'8888'7777'6666ULL);

    // Take BREADY away FIRST: a CPU in reset does not acknowledge, and this
    // is what keeps the slot in WS_WAIT_B long enough to abandon it.
    dut->m0_bready = 0;
    check("wait-b: M0 AW (len=0) accepted", m0_aw_handshake(0x51, addr, 0, 4));
    check("wait-b: M0's single W beat accepted", m0_w_beat(payload, /*last*/ true));
    for (int i = 0; i < 6; i++) cycle();
    check("wait-b: burst closed on the slave, B outstanding to the xbar",
          s0_bfm.w_beats_left == 0 && s0_bfm.b_pending);

    const int prev_m0_ok  = mb[0].w_completed_ok;
    const int prev_m0_err = mb[0].w_completed_errs;
    const int prev_m3_ok  = mb[3].w_completed_ok;
    const int prev_m3_err = mb[3].w_completed_errs;

    dut->cpu_held_in_reset = 1;
    dut->eval();
    int cycles_to_done = -1;
    for (int i = 0; i < 64; i++) {
        cycle();
        if (!s0_bfm.b_pending) { cycles_to_done = i + 1; break; }
    }
    check("wait-b: the slave's B was swallowed by the crossbar (no watchdog, "
          "no permanent sw_owned lock)", cycles_to_done > 0);
    check("wait-b: no B leaked to either side of the shared port",
          mb[0].w_completed_ok == prev_m0_ok &&
          mb[0].w_completed_errs == prev_m0_err &&
          mb[3].w_completed_ok == prev_m3_ok &&
          mb[3].w_completed_errs == prev_m3_err);
    check("wait-b: the real beat still landed (it was accepted before the reset)",
          w128_eq(s0_mem.read(addr), payload));

    dut->m0_bready = 1;
    const uint32_t addr2 = addr + 0x80u;
    const Word128 payload2 = w128_from_u64(0xAAAA'BBBB'CCCC'DDDDULL);
    int prev_m1_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0x52, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("wait-b: S0 still usable by other masters afterwards",
          mb[1].w_completed_ok == prev_m1_ok + 1 &&
          w128_eq(s0_mem.read(addr2), payload2));

    dut->cpu_held_in_reset = 0;
    mb[0].w_expect.clear();
    s0_bfm_fresh();
    reset();
}

// ── Scenario 42d (backlog item 1) ──────────────────────────────────────
// THE MISMATCH TEST IS BLIND WHEN BOTH SIDES ARE HELD.
//
// Scenarios 42/42b/42c all abandon by MOVING the select: the CPU owned the
// slot and cpu_held_in_reset went 0 -> 1.  That is the only shape
// `m0_wsel_q != cpu_held_in_reset` can ever see.  The shape it cannot see
// is the one that actually happens on hardware during boot:
//
//   boot_fsm owns the slot (cpu_held_in_reset already 1) with a write in
//   flight to S0/DDR.  A JTAG debug-full-reset raises soc_full_rst.
//   boot_fsm_rst folds in soc_full_rst, so boot_fsm restarts and forgets
//   the write -- but cpu_held_in_reset STAYS 1, because soc_full_rst also
//   re-clears boot_rom_loaded and the CPU is still held.  1 != 1 is false.
//
// and no slave-side path rescues it either: S0/DDR is deliberately NOT in
// the flush domain (the MIG/l2c stay on core_rst to skip re-calibration),
// so ws_flush_abort[0] cannot fire.  The slot parks with the slave's burst
// left open; the watchdog then poisons S0, or -- in the WS_WAIT_B variant
// -- is suppressed outright and the slot never comes back at all.  The
// xbar's own `rst` is core_rst, which a debug-full-reset does not assert,
// so nothing short of reconfiguring the FPGA clears it.
//
// slv_flush IS soc_full_rst_bank[4], i.e. exactly the event that resets
// both slot-0 masters, so it is ORed into the m0_abandon set term.
//
// NEGATIVE CONTROL: with only the mismatch term, the pad never starts,
// s0_bfm.aw_busy stays 1 with beats owed, and the follow-up M1 write gets
// SLVERR from the poisoned slave.
static void scenario_42d_boot_side_abandon_via_flush(bool wait_b_variant) {
    std::printf("\n=== Scenario 42d%s: slot-0 abandonment with the select UNCHANGED "
                "(boot_fsm reset by soc_full_rst, cpu_held_in_reset stays 1) ===\n",
                wait_b_variant ? " (WS_WAIT_B)" : " (WS_FWD_W)");
    reset();
    s0_bfm_fresh();
    // The boot FSM owns slot 0 for the whole scenario -- the select never
    // moves, which is the entire point.
    set_cpu_held_in_reset(true);

    const uint32_t addr = BURST_ADDR_BASE + 0x9000u + (wait_b_variant ? 0x400u : 0u);
    const int beats      = wait_b_variant ? 1 : 4;
    const int stop_after = wait_b_variant ? 1 : 2;
    const Word128 sentinel = w128_from_u64(0x5E47'1EED'5E47'1EEDULL);
    for (int b = 0; b < beats; b++) s0_mem.write(addr + (uint32_t)b * 16, sentinel);

    // In the WS_WAIT_B variant the full burst goes through and only the
    // response is outstanding, so BREADY has to be taken away first: a
    // master in reset does not acknowledge.
    if (wait_b_variant) dut->m0b_bready = 0;

    // --- AW on the BOOT sub-port -------------------------------------
    dut->m0b_awid = 0x9; dut->m0b_awaddr = addr;
    dut->m0b_awlen = (uint8_t)(beats - 1); dut->m0b_awsize = 4;
    dut->m0b_awburst = 1; dut->m0b_awvalid = 1;
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0b_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m0b_awvalid = 0;
    check("42d: boot-side AW accepted while cpu_held_in_reset=1", aw_ok);

    std::vector<Word128> sent;
    bool w_ok = true;
    for (int b = 0; b < stop_after; b++) {
        Word128 d = w128_from_u64(0xB007'0000'0000'0000ULL + (uint64_t)b);
        sent.push_back(d);
        set_wdata(dut->m0b_wdata, d);
        dut->m0b_wstrb = 0xFFFF;
        dut->m0b_wlast = (b == beats - 1) ? 1 : 0;
        dut->m0b_wvalid = 1;
        bool beat_ok = false;
        for (int i = 0; i < 200; i++) {
            dut->eval();
            if (dut->m0b_wready) { cycle(); beat_ok = true; break; }
            cycle();
        }
        dut->m0b_wvalid = 0; dut->m0b_wlast = 0;
        w_ok = w_ok && beat_ok;
    }
    check("42d: boot-side W beats accepted", w_ok);

    for (int i = 0; i < 10 && !s0_bfm.aw_busy; i++) cycle();
    if (wait_b_variant) {
        for (int i = 0; i < 6; i++) cycle();
        check("42d: burst closed on the slave, B outstanding to the xbar",
              s0_bfm.w_beats_left == 0 && s0_bfm.b_pending);
    } else {
        check("42d: S0 burst left open with beats owed",
              s0_bfm.aw_busy && s0_bfm.w_beats_left == beats - stop_after);
    }

    const int prev_m0_ok  = mb[0].w_completed_ok;
    const int prev_m0_err = mb[0].w_completed_errs;
    const int prev_m3_ok  = mb[3].w_completed_ok;
    const int prev_m3_err = mb[3].w_completed_errs;

    // --- THE EVENT: soc_full_rst.  boot_fsm forgets; the select does not
    //     move, because the CPU is still held too. ----------------------
    dut->m0b_wvalid = 0; dut->m0b_wlast = 0; dut->m0b_awvalid = 0;
    check("42d: select really is unchanged (cpu_held_in_reset still 1)",
          dut->cpu_held_in_reset == 1);
    dut->slv_flush = 1;
    dut->eval();
    cycle();
    cycle();
    dut->slv_flush = 0;
    dut->eval();

    // The recovery must come from the abandon path, not the 4096-cycle
    // watchdog -- a run that needs the watchdog fails HERE.
    int cycles_to_done = -1;
    for (int i = 0; i < 64; i++) {
        cycle();
        if (!s0_bfm.aw_busy && s0_bfm.w_beats_left == 0 && !s0_bfm.b_pending) {
            cycles_to_done = i + 1;
            break;
        }
    }
    check("42d: (a) slave's burst completed and its B swallowed, well inside "
          "the watchdog window", cycles_to_done > 0);

    if (!wait_b_variant) {
        check("42d: (b) exactly AWLEN+1 beats reached the slave, the tail as "
              "WSTRB=0 filler",
              s0_bfm.w_beats_seen == beats &&
              s0_bfm.w_pad_beats == beats - stop_after);
        check("42d: WLAST landed on the final beat only", s0_bfm.w_wlast_errors == 0);
        bool mem_ok = true;
        for (int b = 0; b < beats; b++) {
            Word128 got = s0_mem.read(addr + (uint32_t)b * 16);
            mem_ok = mem_ok && w128_eq(got, (b < stop_after) ? sent[b] : sentinel);
        }
        check("42d: filler beats wrote nothing (real beats intact)", mem_ok);
    } else {
        check("42d: the real beat still landed (accepted before the reset)",
              w128_eq(s0_mem.read(addr), sent[0]));
    }

    check("42d: no B leaked to EITHER side of the shared slot-0 port",
          mb[0].w_completed_ok == prev_m0_ok &&
          mb[0].w_completed_errs == prev_m0_err &&
          mb[3].w_completed_ok == prev_m3_ok &&
          mb[3].w_completed_errs == prev_m3_err &&
          !dut->m0_bvalid && !dut->m0b_bvalid);

    // S0 must NOT be poisoned: an unrelated master still gets OKAY and its
    // data still lands.  This is the check that fails loudly on the
    // unfixed RTL once the watchdog has fired.
    dut->m0b_bready = 1;
    const uint32_t addr2 = addr + 0x80u;
    const Word128 payload2 = w128_from_u64(0x600D'F1A6'600D'F1A6ULL);
    const int prev_m1_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0xA, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("42d: (c) S0 not poisoned -- a later M1 write gets OKAY and lands",
          mb[1].w_completed_ok == prev_m1_ok + 1 &&
          w128_eq(s0_mem.read(addr2), payload2));

    // ...and the handoff to the CPU still works afterwards.
    set_cpu_held_in_reset(false);
    const uint32_t addr3 = addr + 0x100u;
    const Word128 payload3 = w128_from_u64(0xFEED'FACE'FEED'FACEULL);
    const int prev_ok3 = mb[0].w_completed_ok;
    issue_write<0>(/*id*/ 0xB, addr3, 0, 4, {payload3}, /*expect_bresp*/ 0);
    check("42d: (d) slot 0 usable by the CPU after the handoff",
          mb[0].w_completed_ok == prev_ok3 + 1 &&
          w128_eq(s0_mem.read(addr3), payload3));

    // The abandoned write's B is correctly never delivered.
    mb[3].w_expect.clear();
    mb[0].w_expect.clear();
    s0_bfm_fresh();
    reset();
}

// ── Scenario 45 (backlog item 2) ───────────────────────────────────────
// THE S3 READ QUARANTINE WAS SINGLE-ENTRY WITH NO PER-SLOT OWNER.
//
// The read path is ID-routed (s3_rtgt) with NO equivalent of the write
// side's sw_owned lock, so several slots can have S3 reads outstanding at
// once and their bursts can come back in any order.  A single slv_flush
// aborts EVERY slot that is on S3 -- and every one of them used to set the
// same single `s3_flush_read_active` bit.  Two failures followed:
//
//   (a) UNDER-DRAIN.  The quarantine cleared on the FIRST RLAST it saw, so
//       every further stale burst arrived with nobody consuming it.  Under
//       VRAM_IN_DDR that burst sits on the DDR read port S3 shares with S0,
//       so it blocks EVERY DDR read -- and the state is not on core_rst, so
//       it survives every reset the design has.  Reconfigure only.
//
//   (b) DATA LOSS.  The bit forced s3_rready unconditionally, so a beat
//       belonging to a slot that had NOT been aborted (still RS_WAIT_R,
//       master possibly not ready) was handshaken away and never delivered.
//
// Vehicle: two masters (M0 slot 0, M2 slot 2) with S3 reads genuinely in
// flight, R delivery frozen, one flush, then the bursts released in
// REVERSE order so the stale burst that arrives first is NOT the one a
// single-entry quarantine would happen to be holding.
static void scenario_45_s3_read_quarantine_is_per_slot() {
    std::printf("\n=== Scenario 45: two concurrent S3 reads flush-aborted -- BOTH stale "
                "bursts must be quarantined ===\n");
    reset();
    s3_bfm = SlaveBfm{};
    s3_r_multi_reset();
    set_cpu_held_in_reset(false);

    const uint32_t vaddr0 = 0xF9010000u;   // M0's read
    const uint32_t vaddr2 = 0xF9020000u;   // M2's read
    for (int b = 0; b < 2; b++) {
        s3_mem.write(0x00010000u + (uint32_t)b * 16,
                     w128_from_u64(0xA5A5'0000'0000'0000ULL + (uint64_t)b));
        s3_mem.write(0x00020000u + (uint32_t)b * 16,
                     w128_from_u64(0x5A5A'0000'0000'0000ULL + (uint64_t)b));
    }

    s3_r_multi = true;
    s3_r_hold  = true;    // ARs are accepted; not one R beat comes back

    // Both reads are aborted by the flush, so both masters get the local
    // SLVERR tail (see rs_release_slot).
    mb[0].r_expect.push_back({0x1, 2});
    dut->m0_arid = 0x1; dut->m0_araddr = vaddr0; dut->m0_arlen = 1;
    dut->m0_arsize = 4; dut->m0_arburst = 1; dut->m0_arvalid = 1;
    for (int i = 0; i < 200; i++) { dut->eval(); if (dut->m0_arready) { cycle(); break; } cycle(); }
    dut->m0_arvalid = 0;

    mb[2].r_expect.push_back({0x2, 2});
    dut->m2_arid = 0x2; dut->m2_araddr = vaddr2; dut->m2_arlen = 1;
    dut->m2_arsize = 4; dut->m2_arburst = 1; dut->m2_arvalid = 1;
    for (int i = 0; i < 200; i++) { dut->eval(); if (dut->m2_arready) { cycle(); break; } cycle(); }
    dut->m2_arvalid = 0;

    for (int i = 0; i < 40 && s3_ar_q.size() < 2; i++) cycle();
    check("45: BOTH S3 reads are genuinely in flight at the slave", s3_ar_q.size() == 2);

    // ── the flush ───────────────────────────────────────────────────────
    dut->slv_flush = 1;
    dut->eval();
    cycle(); cycle();
    dut->slv_flush = 0;
    dut->eval();
    for (int i = 0; i < 40; i++) cycle();

    check("45: M0's aborted read was answered with the local SLVERR tail",
          mb[0].r_expect.empty());
    check("45: M2's aborted read was answered with the local SLVERR tail",
          mb[2].r_expect.empty());

    // ── release the stale bursts, LAST ONE FIRST ────────────────────────
    // A single-entry quarantine retires on whichever RLAST it sees first and
    // then has nothing left to consume the other burst with.
    s3_r_lifo = true;
    s3_r_hold = false;
    int drain_cycles = -1;
    for (int i = 0; i < 400; i++) {
        cycle();
        if (s3_r_multi_drained()) { drain_cycles = i + 1; break; }
    }
    check("45: (a) BOTH stale bursts were consumed by the crossbar "
          "(a single-entry quarantine drains only one)",
          drain_cycles > 0 && s3_r_bursts_done == 2);

    // ── and S3/DDR is usable again ──────────────────────────────────────
    const uint32_t vaddr3 = 0xF9030000u;
    const Word128 fresh = w128_from_u64(0xC0FF'EE00'C0FF'EE00ULL);
    s3_mem.write(0x00030000u, vram_native_word_order(fresh));
    s3_r_lifo = false;
    const int prev_rok = mb[0].r_completed;
    issue_read<0>(/*id*/ 0x3, vaddr3, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    bool fresh_ok = (mb[0].r_completed == prev_rok + 1) &&
                    !mb[0].r_received.empty() &&
                    mb[0].r_received.back().size() == 1 &&
                    w128_eq(mb[0].r_received.back()[0], fresh);
    check("45: (b) a fresh S3 read completes with correct data afterwards "
          "-- the shared DDR read port is not blocked", fresh_ok);

    s3_r_multi_reset();
    s3_bfm = SlaveBfm{};
    mb[0].r_expect.clear();
    mb[2].r_expect.clear();
    reset();
}

// ── Scenario 45b (backlog item 2, failure mode (b)) ────────────────────
// A beat belonging to a slot that was NOT aborted must never be consumed
// by the quarantine.  One master reads S1 (which the flush DOES abort) and
// another reads S3 -- no: both must be on S3, one aborted and one not.
// The xbar aborts every S3 slot on a flush, so the only way to get a live
// S3 read alongside a quarantined one is to start the second read AFTER
// the flush has gone.  With a stale burst still owed, the AR-admission
// gate holds the new read off entirely, which is exactly the property that
// keeps failure mode (b) unreachable ONCE the quarantine is per-slot: this
// scenario pins that gate so a future relaxation cannot silently reopen it.
static void scenario_45b_no_ar_admitted_while_a_stale_burst_is_owed() {
    std::printf("\n=== Scenario 45b: no new S3 AR is admitted while a stale burst is "
                "still owed ===\n");
    reset();
    s3_bfm = SlaveBfm{};
    s3_r_multi_reset();
    set_cpu_held_in_reset(false);

    const uint32_t vaddr0 = 0xF9040000u;
    s3_mem.write(0x00040000u, w128_from_u64(0x1234'5678'9ABC'DEF0ULL));
    s3_r_multi = true;
    s3_r_hold  = true;

    mb[0].r_expect.push_back({0x4, 2});
    dut->m0_arid = 0x4; dut->m0_araddr = vaddr0; dut->m0_arlen = 0;
    dut->m0_arsize = 4; dut->m0_arburst = 1; dut->m0_arvalid = 1;
    for (int i = 0; i < 200; i++) { dut->eval(); if (dut->m0_arready) { cycle(); break; } cycle(); }
    dut->m0_arvalid = 0;
    for (int i = 0; i < 40 && s3_ar_q.empty(); i++) cycle();
    check("45b: the S3 read is in flight", s3_ar_q.size() == 1);

    dut->slv_flush = 1; dut->eval(); cycle(); cycle();
    dut->slv_flush = 0; dut->eval();
    for (int i = 0; i < 40; i++) cycle();
    check("45b: the aborted read was answered", mb[0].r_expect.empty());

    // A second read, offered while the stale burst is still owed, must not
    // reach the slave: its AR would be indistinguishable from the stale
    // one's on the R channel.
    const size_t q_before = s3_ar_q.size();
    dut->m2_arid = 0x5; dut->m2_araddr = 0xF9050000u; dut->m2_arlen = 0;
    dut->m2_arsize = 4; dut->m2_arburst = 1; dut->m2_arvalid = 1;
    bool admitted = false;
    for (int i = 0; i < 60; i++) { dut->eval(); if (dut->m2_arready) { admitted = true; } cycle(); }
    dut->m2_arvalid = 0;
    check("45b: no second S3 AR reached the slave while the stale burst is owed",
          s3_ar_q.size() == q_before && !admitted);

    // Release the stale burst; S3 must then be usable again.
    s3_r_hold = false;
    int drained = -1;
    for (int i = 0; i < 400; i++) {
        cycle();
        if (s3_r_multi_drained()) { drained = i + 1; break; }
    }
    check("45b: the stale burst retires once R delivery resumes", drained > 0);

    const Word128 val = w128_from_u64(0xDEC0'DE00'DEC0'DE00ULL);
    s3_mem.write(0x00050000u, vram_native_word_order(val));
    const int prev_rok = mb[2].r_completed;
    issue_read<2>(/*id*/ 0x5, 0xF9050000u, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    check("45b: the previously held-off read completes once the quarantine retires",
          mb[2].r_completed == prev_rok + 1 &&
          !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
          w128_eq(mb[2].r_received.back()[0], val));

    s3_r_multi_reset();
    s3_bfm = SlaveBfm{};
    mb[0].r_expect.clear();
    mb[2].r_expect.clear();
    reset();
}

// ── Scenario 46 (backlog item 5) ───────────────────────────────────────
// A MASTER THAT WALKS AWAY FROM A LOCAL R TAIL USED TO PIN ITS READ SLOT
// FOREVER.
//
// The module header's abandonment audit says "the read side never had this
// shape: rs_wd_fire suppresses on an actual RVALID&&RREADY handshake".
// That is true of RS_WAIT_R, where the SLAVE produces the data.  It is not
// true of RS_SEND_RLOCAL, where the CROSSBAR produces it:
//
//   * RS_SEND_RLOCAL is deliberately excluded from rs_active, so rs_wd_cnt
//     is held at zero there and rs_wd_fire can never fire;
//   * mr_rvalid_loc[] is asserted unconditionally from the state;
//   * nothing else ever leaves the state.
//
// So the slot sat at RS_SEND_RLOCAL permanently and that master could never
// read again -- the exact failure task #219 fixed on the B channel
// (ws_b_abandon), produced by the same master: axi_narrow_to_wide's own
// abandonment watchdog force-clears ar_valid_q, which takes its w_rready
// low and keeps it low, and the JTAG-AXI / XDMA host on M1 sits behind one.
//
// Run for M1 (host debug) and M2 (CPU instruction fetch), both of which are
// behind masters that can be reset out from under an in-flight read.
template <int M>
static void scenario_46_local_r_tail_abandoned(const char* who, uint32_t addr) {
    std::printf("\n=== Scenario 46: M%d (%s) walks away from its local R tail -- "
                "the read slot must not be pinned ===\n", M, who);
    static_assert(M == 1 || M == 2, "scenario 46 runs on M1 / M2");
    reset();
    set_cpu_held_in_reset(false);

    // A master in reset stops acknowledging.  Take RREADY away BEFORE the
    // AR so the tail is never accepted at all.
    if constexpr (M == 1) dut->m1_rready = 0; else dut->m2_rready = 0;

    // `addr` decodes to no slave, so the crossbar answers it entirely by
    // itself out of RS_SEND_RLOCAL -- no slave is ever involved, which is
    // what makes "no poison" the right release policy.
    if constexpr (M == 1) {
        dut->m1_arid = 0x6; dut->m1_araddr = addr; dut->m1_arlen = 3;
        dut->m1_arsize = 4; dut->m1_arburst = 1; dut->m1_arvalid = 1;
    } else {
        dut->m2_arid = 0x6; dut->m2_araddr = addr; dut->m2_arlen = 3;
        dut->m2_arsize = 4; dut->m2_arburst = 1; dut->m2_arvalid = 1;
    }
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        uint8_t rdy = (M == 1) ? dut->m1_arready : dut->m2_arready;
        if (rdy) { cycle(); aw_ok = true; break; }
        cycle();
    }
    if constexpr (M == 1) dut->m1_arvalid = 0; else dut->m2_arvalid = 0;
    check("46: the local-response read was accepted", aw_ok);

    bool offered = false;
    for (int i = 0; i < 20; i++) {
        cycle();
        if ((M == 1) ? dut->m1_rvalid : dut->m2_rvalid) { offered = true; break; }
    }
    check("46: the crossbar is offering the local R tail", offered);

    // The tb builds with -GB_HOLD_LOG2=6, so the grace window is 64 cycles.
    // Give it a generous multiple, still far below any watchdog.
    int released = -1;
    for (int i = 0; i < 600; i++) {
        cycle();
        if (!((M == 1) ? dut->m1_rvalid : dut->m2_rvalid)) { released = i + 1; break; }
    }
    check("46: (a) the unaccepted tail is withdrawn and the slot released",
          released > 0);

    // ...and the master can read again.  This is the check that matters:
    // on the unfixed RTL the slot is pinned and NO later read from this
    // master ever completes.
    if constexpr (M == 1) dut->m1_rready = 1; else dut->m2_rready = 1;
    const uint32_t raddr = 0x00460000u;
    const Word128 val = w128_from_u64(0x1357'9BDF'1357'9BDFULL);
    s0_mem.write(raddr, val);
    const int prev_rok = mb[M].r_completed;
    issue_read<M>(/*id*/ 0x7, raddr, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    check("46: (b) the master can read again afterwards",
          mb[M].r_completed == prev_rok + 1 &&
          !mb[M].r_received.empty() && mb[M].r_received.back().size() == 1 &&
          w128_eq(mb[M].r_received.back()[0], val));

    mb[M].r_expect.clear();
    reset();
}

// ── Scenario 47 (backlog item 3) ───────────────────────────────────────
// THE S1 RESET-DEASSERT RACE.
//
// slv_flush is soc_full_rst_bank[4], a core_clk net.  S1's far side -- the
// axi_pb_s1_cdc bridge and peripheral_bus behind it -- resets on
// pb_soc_full_rst_bank[3]: the SAME source event, released through a
// SEPARATE xpm_cdc_async_rst in the pb_clk domain.  Both use
// DEST_SYNC_FF = 4, but four pb_clk at 50 MHz is up to SIXTEEN core_clk at
// 200 MHz, and the CDC's FIFO pointers still have to resync after that.
//
// So slv_flush falls FIRST, and for a window afterwards the crossbar
// believes S1 is available while the bridge is still in reset.  A
// transaction admitted in that window is accepted by the crossbar, handed
// to a bridge that forgets it, and then nothing completes it: slv_flush is
// already low so the flush-abort paths cannot fire; ENABLE_WD defaults to
// 0 so the watchdogs are constant-false; ws_b_abandon needs a B the slave
// never produced; and `rst` is core_rst, which a debug-full-reset does not
// assert.  S1 is write-dead for every master until core_rst -- the exact
// end state MEASURED on hardware after a JTAG `reset`.
//
// Moving S1 into the flush domain fixed the transaction in flight WHEN the
// flush lands.  It did not fix the one admitted just after it lifts.
//
// The bridge-still-in-reset behaviour is modelled by s1_swallow_b: the
// slave accepts AW+W and never answers, which is exactly "the far side
// forgot it".  This build sets -GS1_RST_TAIL_LOG2=5, a 32-cycle tail.
static void scenario_47_s1_reset_deassert_tail() {
    std::printf("\n=== Scenario 47: S1 stays closed across the pb-side reset-deassert "
                "skew ===\n");
    reset();
    s1_bfm = SlaveBfm{};
    set_cpu_held_in_reset(false);
    // `rst` arms the tail too; let that one expire so the measurement
    // below is attributable to the FLUSH.
    for (int i = 0; i < 64; i++) cycle();

    const uint32_t addr = 0x50006000u;
    const Word128 payload = w128_from_u64(0x0BAD'0BAD'600D'600DULL);

    // The far side is still in reset: it accepts and forgets.
    s1_in_reset = true;
    s1_reset_swallowed_aw = 0;
    s1_reset_swallowed_ar = 0;

    dut->slv_flush = 1;
    dut->eval();
    cycle(); cycle();
    dut->slv_flush = 0;
    dut->eval();

    // Offer the write the very next cycle.
    const int prev_ok  = mb[1].w_completed_ok;
    const int prev_err = mb[1].w_completed_errs;
    mb[1].w_expect.push_back({0xC, 0});
    dut->m1_awid = 0xC; dut->m1_awaddr = addr; dut->m1_awlen = 0;
    dut->m1_awsize = 4; dut->m1_awburst = 1; dut->m1_awvalid = 1;

    // (a) For the first 8 cycles -- comfortably inside the 32-cycle tail,
    // comfortably outside the 1-2 cycles an ungated admission takes -- the
    // AW must NOT reach the slave.
    for (int i = 0; i < 8; i++) cycle();
    char nm[160];
    std::snprintf(nm, sizeof(nm),
                  "47: (a) no S1 AW reached the still-resetting bridge during the tail "
                  "(swallowed=%d, want 0)", s1_reset_swallowed_aw);
    check(nm, s1_reset_swallowed_aw == 0);

    // The bridge finishes leaving reset.  The held request must now go
    // through normally -- the tail is a bounded STALL, not a drop and not
    // a manufactured bus error.
    s1_in_reset = false;
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m1_awvalid = 0;
    check("47: the held AW is accepted once the tail expires", aw_ok);

    set_wdata(dut->m1_wdata, payload);
    dut->m1_wstrb = 0xFFFF; dut->m1_wlast = 1; dut->m1_wvalid = 1;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_wready) { cycle(); break; }
        cycle();
    }
    dut->m1_wvalid = 0; dut->m1_wlast = 0;
    for (int i = 0; i < 200; i++) cycle();

    check("47: (b) the write completes OKAY -- not SLVERR, not stranded",
          mb[1].w_completed_ok == prev_ok + 1 &&
          mb[1].w_completed_errs == prev_err &&
          mb[1].w_expect.empty());
    check("47: (b) and it reached the REAL slave with the right data",
          s1_bfm.seen_aw_count[addr] >= 1 && w128_eq(s1_mem.read(addr), payload));
    check("47: S1 was not poisoned by any of this", !dut->dbg_slv_poisoned);

    mb[1].w_expect.clear();
    s1_bfm = SlaveBfm{};
    reset();
}

// ── Scenario 48 (backlog item 4) ───────────────────────────────────────
// A MASTER THAT WALKS AWAY MID-BURST IN RS_WAIT_R BLOCKS THE SLAVE'S WHOLE
// READ CHANNEL, FOR EVERY MASTER.
//
// Item 4 as filed was "slot-0 reads to S0/DDR have no abort path at all".
// That is a true statement about the code, but the failure it implies --
// a slot-0 DDR read stranded by a CPU reset -- cannot happen here, for two
// independently engineered reasons (both pinned by scenario 48b below):
// S0's backend genuinely survives and answers, and cpu_d_rready_xbar is
// forced high for the whole of cpu_rst.
//
// The reachable hole in that area is a different one, and it is not
// slot-0-specific.  While a slot sits in RS_WAIT_R, the ONLY thing that
// drives the slave's RREADY for it is that slot's own master
// (s?_rready_per[] = ... && mr_rready[mi]).  If the master stops -- which
// is exactly what axi_narrow_to_wide does when its abandonment watchdog
// force-clears ar_valid_q, and the JTAG-AXI / XDMA host on M1 sits behind
// one -- then:
//
//   * the slave's RVALID is never acknowledged, so its read channel is
//     blocked for EVERY master, not just this one;
//   * with ENABLE_WD = 0 (the shipping default) rs_wd_fire is
//     constant-false and nothing releases the slot at all;
//   * and even with ENABLE_WD = 1, the watchdog's release POISONS the
//     slave and still never drains the stale burst, so the read channel
//     stays blocked and DDR is SLVERR'd for everyone on top.
//
// This is the read-side twin of ws_b_abandon, one channel over: the B
// channel got this treatment in task #219, the R channel did not.
//
// The fix drains the abandoned burst at the slave through RLAST from a new
// RS_DRAIN_R state -- the exact read-side mirror of WS_DRAIN_W -- and then
// idles the slot, with no poison: the slave is healthy, it is the master
// that left.
static void scenario_48_r_master_walks_away_midburst() {
    std::printf("\n=== Scenario 48: M1 walks away mid-burst in RS_WAIT_R -- S0's read "
                "channel must not stay blocked ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t addr = BURST_ADDR_BASE + 0xA000u;
    for (int b = 0; b < 4; b++)
        s0_mem.write(addr + (uint32_t)b * 16,
                     w128_from_u64(0xB105'0000'0000'0000ULL + (uint64_t)b));

    // M1 issues a 4-beat read, takes one beat, then stops acknowledging --
    // a master whose own abandonment watchdog has taken its RREADY low.
    int m1_ar_issued = 0;
    // mb[].r_lasts_seen is cumulative for the whole run, so snapshot it.
    const int prev_rlasts = mb[1].r_lasts_seen;
    // The read is abandoned mid-burst, so its terminating beat is the local
    // SLVERR tail, not OKAY data.
    mb[1].r_expect.push_back({0x8, 2});
    dut->m1_arid = 0x8; dut->m1_araddr = addr; dut->m1_arlen = 3;
    dut->m1_arsize = 4; dut->m1_arburst = 1; dut->m1_arvalid = 1;
    bool ar_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_arready) { cycle(); ar_ok = true; break; }
        cycle();
    }
    dut->m1_arvalid = 0;
    if (ar_ok) m1_ar_issued++;
    check("48: M1's burst read was accepted", ar_ok);

    int beats = 0;
    for (int i = 0; i < 40 && beats < 1; i++) {
        cycle();
        if (dut->m1_rvalid && dut->m1_rready) beats++;
    }
    check("48: at least one beat was delivered before the master left", beats >= 1);

    dut->m1_rready = 0;          // the master is gone
    dut->eval();

    // The burst is now stuck at the slave with beats owed.
    for (int i = 0; i < 10; i++) cycle();
    check("48: the slave still owes beats on this burst", s0_bfm.r_beats_left > 0);

    // Give it a long window -- far longer than B_HOLD_MAX (64 here) and
    // than the watchdog (4096 here).
    int drained = -1;
    for (int i = 0; i < 8192; i++) {
        cycle();
        if (!s0_bfm.ar_busy && s0_bfm.r_beats_left == 0) { drained = i + 1; break; }
    }
    check("48: (a) the abandoned burst is drained at the slave, freeing its "
          "read channel", drained > 0);

    // The master's RREADY returns (a reset master releases it; a slow one
    // never really lost it) -- the terminating tail is then delivered and
    // check (d) above can count it.
    dut->m1_rready = 1;
    for (int i = 0; i < 200 && !mb[1].r_expect.empty(); i++) cycle();

    // The payoff: an unrelated master must still be able to read DDR.
    const uint32_t addr2 = addr + 0x200u;
    const Word128 val = w128_from_u64(0x600D'0DD0'600D'0DD0ULL);
    s0_mem.write(addr2, val);
    const int prev_rok = mb[0].r_completed;
    issue_read<0>(/*id*/ 0x9, addr2, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    check("48: (b) an unrelated master can still read DDR afterwards",
          mb[0].r_completed == prev_rok + 1 &&
          !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
          w128_eq(mb[0].r_received.back()[0], val));
    check("48: (c) S0 was NOT poisoned -- the slave was healthy all along",
          !(dut->dbg_slv_poisoned & 0x01));

    // (d) THE MASTER MUST STILL BE ANSWERED.  Draining the slave frees its
    // read channel; it does NOT discharge the debt the master is tracking.
    // M0 and M2 have no axi_narrow_to_wide to self-answer them -- what waits
    // on them is the core's AxiReadResetAbsorber, which decrements ONLY on a
    // bus-side RLAST. Drop the response and its count never reaches zero,
    // `absorbing` stays latched, no AR can ever issue, and D20 halts with
    // ARBITER_WEDGE that no reset clears (the absorber is in a BOOT domain
    // on purpose). So: exactly one terminating RLAST, and the expectation
    // queue must drain.
    //
    // Modelled the way the absorber does it -- count ARs issued against
    // RLASTs returned -- rather than by watching a state bit, because that
    // is the actual invariant the core depends on.
    char nm[200];
    std::snprintf(nm, sizeof(nm),
                  "48: (d) the abandoned read is ANSWERED (%d AR issued, %d RLAST "
                  "returned) -- the absorber's count must return to zero",
                  m1_ar_issued, mb[1].r_lasts_seen - prev_rlasts);
    check(nm, (mb[1].r_lasts_seen - prev_rlasts) == m1_ar_issued &&
              mb[1].r_expect.empty());

    // The abandoned burst left M1's master-side BFM mid-burst -- that is
    // the whole point of the scenario, the master stopped acknowledging
    // part way through -- so clear the partial accumulation, or the NEXT
    // M1 read's beats append to it and every later assertion on
    // r_received.back() reads this scenario's payload.
    mb[0].r_expect.clear();
    mb[1].r_expect.clear();
    mb[1].r_cur.clear();
    mb[1].r_cur_resp.clear();
    s0_bfm_fresh();
    reset();
}

// ── Scenario 48b (backlog item 4, the part that is NOT a defect) ───────
// Item 4 as filed was "slot-0 reads to S0/DDR have no abort path at all --
// S0 is not in the flush domain and the slot-0 owner-gone term is S1-only".
// Both halves of that are true of the code, and the conclusion still does
// not follow, because two other things hold the case up:
//
//   (i)  S0's backend genuinely survives a soc_full_rst and ANSWERS.  The
//        MIG and l2c deliberately stay on core_rst to skip the ~100 ms
//        re-calibration (see is_flush_domain_slv()), so unlike S1's bridge
//        there is nothing that forgets the transaction.
//   (ii) the CPU port's RREADY is forced high for the whole of cpu_rst --
//        `cpu_d_rready_xbar = cpu_rst ? 1'b1 : cpu_d_rready`
//        (fpga_top_cpu.vh:175) -- and across the release edge the core's
//        AxiReadResetAbsorber, which lives in a resetKind=BOOT domain
//        precisely so cpu_rst cannot clear it, keeps it high for any read
//        the SoC had already accepted.
//
// So the burst drains, the slot releases, and no abort is needed.  Adding
// an owner-gone abort for S0 would be a REGRESSION: it would abandon a
// transaction whose slave is healthy and about to answer, and would then
// need a stale-burst quarantine on the shared DDR read port to clean up
// after itself.
//
// That argument is entirely a property of the INTEGRATION, not of this
// module, so it is pinned here as a regression guard: if the RREADY
// forcing in fpga_top_cpu.vh is ever removed, this is the test that says
// what it was load-bearing for.
static void scenario_48b_slot0_ddr_read_survives_cpu_reset() {
    std::printf("\n=== Scenario 48b: a slot-0 DDR read in flight when the CPU is reset "
                "completes on its own (why item 4 needs no abort) ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t addr = BURST_ADDR_BASE + 0xB000u;
    for (int b = 0; b < 4; b++)
        s0_mem.write(addr + (uint32_t)b * 16,
                     w128_from_u64(0xC0DE'0000'0000'0000ULL + (uint64_t)b));

    mb[0].r_expect.push_back({0xA, 0});
    dut->m0_arid = 0xA; dut->m0_araddr = addr; dut->m0_arlen = 3;
    dut->m0_arsize = 4; dut->m0_arburst = 1; dut->m0_arvalid = 1;
    bool ar_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0_arready) { cycle(); ar_ok = true; break; }
        cycle();
    }
    dut->m0_arvalid = 0;
    check("48b: the CPU's burst read was accepted", ar_ok);

    // The CPU is yanked into reset mid-burst.  m0_rready stays HIGH: that
    // is the contract cpu_d_rready_xbar implements.
    dut->cpu_held_in_reset = 1;
    dut->m0_rready = 1;
    dut->eval();

    int done = -1;
    for (int i = 0; i < 200; i++) {
        cycle();
        if (!s0_bfm.ar_busy && s0_bfm.r_beats_left == 0) { done = i + 1; break; }
    }
    check("48b: the burst completes at the slave with no abort needed", done > 0);
    check("48b: the read was fully answered to the (reset) master",
          mb[0].r_expect.empty());
    check("48b: S0 was not poisoned", !(dut->dbg_slv_poisoned & 0x01));

    // DDR is still usable by an unrelated master.
    dut->cpu_held_in_reset = 0;
    for (int i = 0; i < 4; i++) cycle();
    const uint32_t addr2 = addr + 0x200u;
    const Word128 val = w128_from_u64(0xFEE1'600D'FEE1'600DULL);
    s0_mem.write(addr2, val);
    const int prev_rok = mb[1].r_completed;
    issue_read<1>(/*id*/ 0xB, addr2, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    
    check("48b: DDR still usable afterwards",
          mb[1].r_completed == prev_rok + 1 &&
          !mb[1].r_received.empty() && mb[1].r_received.back().size() == 1 &&
          w128_eq(mb[1].r_received.back()[0], val));

    mb[0].r_expect.clear();
    mb[1].r_expect.clear();
    s0_bfm_fresh();
    reset();
}

// ── Scenario 49 (race audit, 2026-09-18) ───────────────────────────────
// A FLUSH THAT LANDS INSIDE RS_DRAIN_R DROPS THE MASTER'S RESPONSE *AND*
// SKIPS THE S3 QUARANTINE.
//
// RS_DRAIN_R (backlog item 4) is entered when the SLAVE is producing beats
// and the owning master has stopped taking them.  Its normal exit -- the
// drained burst's own RLAST -- deliberately hands off to RS_SEND_RLOCAL so
// that the master is still ANSWERED: the core's AxiReadResetAbsorber
// decrements only on a bus-side RLAST, lives in a resetKind=BOOT domain no
// reset can clear, and a dropped response therefore wedges it forever
// (that is what commit b8ac66b2 fixed, and what scenario 48 check (d)
// pins).
//
// Its OTHER exit did neither.  `slv_flush` inside RS_DRAIN_R went straight
// to RS_IDLE on the premise "the slave is being reset under us; it has
// forgotten the burst".  That premise is exactly the one
// S3_BACKEND_SURVIVES_FLUSH exists to deny: under VRAM_IN_DDR -- the
// topology the bitstream actually ships (fpga_top_xbar.vh) -- S3 is a
// route into the always-live DDR mux/MIG path, and that path does NOT
// forget.  Three consequences, all reachable together:
//
//   (a) the surviving backend's remaining beats arrive with nobody
//       consuming them.  s3_rready is driven only from RS_WAIT_R /
//       RS_DRAIN_R / s3_rq_v, and the slot is now in none of those, so the
//       S3 -- i.e. the shared DDR -- read channel is blocked for EVERY
//       master.  This is failure mode (a) of backlog item 2, reintroduced
//       one state over, and it survives every reset the design has
//       (s3_rq_v is on the xbar's `rst`, the DDR side is not), so only a
//       reconfigure clears it.
//   (b) the master is never answered -- the b8ac66b2 wedge, still live on
//       this exit.
//   (c) because s3_rq_v is what gates S3 AR admission, a fresh S3 read is
//       admitted on top of the stale burst.
//
// The RS_WAIT_R flush-abort branch immediately above gets all of this right
// (it sets s3_rq_v and releases through rs_release_slot, which always goes
// to RS_SEND_RLOCAL); RS_DRAIN_R was simply never brought into line.
//
// Reachability is not hypothetical: `rst` here is core_rst_bank[4] but
// `slv_flush` is soc_full_rst_bank[4], and soc_full_rst = core_rst ||
// jtag_debug_full_reset (fpga_top_clocks.vh:914).  A JTAG `reset` asserts
// the flush WITHOUT asserting the crossbar's rst -- which is the whole
// reason slv_flush exists.  The master that walks away from an S3 burst is
// the host debug peer on M1, behind an axi_narrow_to_wide whose own
// abandonment watchdog takes its RREADY low: read the framebuffer over
// JTAG, let the adapter give up, then type `reset`.
static void scenario_49_drain_r_flush_must_answer_and_quarantine() {
    std::printf("\n=== Scenario 49: slv_flush inside RS_DRAIN_R -- the master must still "
                "be answered and a surviving S3 backend must still be quarantined ===\n");
    reset();
    s3_bfm = SlaveBfm{};
    s3_r_multi_reset();
    set_cpu_held_in_reset(false);

    const uint32_t vaddr = 0xF9060000u;
    for (int b = 0; b < 16; b++)
        s3_mem.write(0x00060000u + (uint32_t)b * 16,
                     w128_from_u64(0xD8A1'0000'0000'0000ULL + (uint64_t)b));

    s3_r_multi = true;
    s3_r_hold  = false;

    const int prev_rlasts = mb[1].r_lasts_seen;
    int m1_ar_issued = 0;
    // The burst is abandoned mid-flight, so its terminating beat is the
    // crossbar's local SLVERR tail, not OKAY data.
    mb[1].r_expect.push_back({0xC, 2});
    dut->m1_rready = 1;
    dut->m1_arid = 0xC; dut->m1_araddr = vaddr; dut->m1_arlen = 15;
    dut->m1_arsize = 4; dut->m1_arburst = 1; dut->m1_arvalid = 1;
    bool ar_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_arready) { cycle(); ar_ok = true; break; }
        cycle();
    }
    dut->m1_arvalid = 0;
    if (ar_ok) m1_ar_issued++;
    check("49: M1's 16-beat S3 read was accepted", ar_ok);

    int beats = 0;
    for (int i = 0; i < 80 && beats < 2; i++) {
        cycle();
        if (dut->m1_rvalid && dut->m1_rready) beats++;
    }
    check("49: beats were flowing before the master left", beats >= 2);

    // The master walks away (its adapter's abandonment watchdog has taken
    // RREADY low and keeps it low).
    dut->m1_rready = 0;
    dut->eval();
    const int left_at_stop = s3_r_cur_left;

    // ... and after B_HOLD_MAX the crossbar enters RS_DRAIN_R and starts
    // consuming beats the master is not taking.
    int drain_begun = -1;
    for (int i = 0; i < 600; i++) {
        cycle();
        if (s3_r_cur_busy && s3_r_cur_left < left_at_stop) { drain_begun = i + 1; break; }
    }
    check("49: the crossbar entered RS_DRAIN_R (it is consuming beats the "
          "master is not taking)", drain_begun > 0);

    // Freeze the backend so the flush lands strictly INSIDE the drain.
    s3_r_hold = true;
    dut->eval();
    check("49: the drain is genuinely mid-burst -- beats are still owed",
          s3_r_cur_busy && s3_r_cur_left > 0);

    // ── the flush ───────────────────────────────────────────────────────
    dut->slv_flush = 1;
    dut->eval();
    cycle(); cycle();
    dut->slv_flush = 0;
    dut->eval();
    for (int i = 0; i < 40; i++) cycle();

    // The S3 backend SURVIVES it (S3_BACKEND_SURVIVES_FLUSH=1 here, the
    // shipping VRAM_IN_DDR topology) and resumes exactly where it stopped.
    s3_r_hold = false;
    int drained = -1;
    for (int i = 0; i < 4000; i++) {
        cycle();
        if (s3_r_multi_drained()) { drained = i + 1; break; }
    }
    check("49: (a) the surviving backend's stale beats are consumed -- the "
          "shared DDR read channel is not left blocked", drained > 0);

    // The master's RREADY returns (a reset master releases it; a slow one
    // never really lost it) and the terminating tail must be delivered.
    dut->m1_rready = 1;
    for (int i = 0; i < 600 && !mb[1].r_expect.empty(); i++) cycle();
    char nm[220];
    std::snprintf(nm, sizeof(nm),
                  "49: (b) the read is ANSWERED (%d AR issued, %d RLAST returned) -- "
                  "the absorber's count must return to zero",
                  m1_ar_issued, mb[1].r_lasts_seen - prev_rlasts);
    check(nm, (mb[1].r_lasts_seen - prev_rlasts) == m1_ar_issued &&
              mb[1].r_expect.empty());

    // (c) S3 is usable again, with correct data.
    const uint32_t vaddr2 = 0xF9070000u;
    const Word128 fresh = w128_from_u64(0x5EED'0FF0'5EED'0FF0ULL);
    s3_mem.write(0x00070000u, vram_native_word_order(fresh));
    const int prev_rok = mb[0].r_completed;
    issue_read<0>(/*id*/ 0xD, vaddr2, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    check("49: (c) a fresh S3 read completes with correct data afterwards",
          mb[0].r_completed == prev_rok + 1 &&
          !mb[0].r_received.empty() && mb[0].r_received.back().size() == 1 &&
          w128_eq(mb[0].r_received.back()[0], fresh));
    check("49: (d) S3 was NOT poisoned -- the slave was healthy throughout",
          !(dut->dbg_slv_poisoned & 0x08));

    // The abandoned burst left M1's master-side BFM mid-burst; clear the
    // partial accumulation so later scenarios do not append to it.
    s3_r_multi_reset();
    s3_bfm = SlaveBfm{};
    mb[0].r_expect.clear();
    mb[1].r_expect.clear();
    mb[1].r_cur.clear();
    mb[1].r_cur_resp.clear();
    reset();
}

// ── Scenarios 50 / 51 (race audit, 2026-09-18) ─────────────────────────
// THE WRITE SIDE'S TWO LOCAL-COMPLETION STATES HAD NO ABANDONMENT PATH AT
// ALL -- the exact hole backlog item 5 closed on the read side, never
// closed on its twin.
//
// Backlog item 5's own note says RS_SEND_RLOCAL's wedge "is exactly the
// failure task #219 fixed on the B channel (ws_b_abandon)".  It is not.
// ws_b_abandon covers WS_WAIT_B -- a B the SLAVE produced.  The two states
// where the CROSSBAR produces the completion were left exactly as
// RS_SEND_RLOCAL was before item 5:
//
//   WS_DRAIN_W     -- exits ONLY on `mw_wvalid && mw_wready_raw`, and its
//                     WREADY is a hard 1'b1 (gen_wready).  A master that
//                     stops sending W beats pins the slot forever.
//   WS_SEND_BLOCAL -- exits ONLY on `mw_bvalid_loc && mw_bready`.  A master
//                     that never takes the local B pins the slot forever.
//
// Neither is in `ws_active`, so ws_wd_cnt is held at zero for both and
// ws_wd_fire can never fire there; ws_flush_abort requires ws_active too.
// The file already KNOWS this -- ws_release_slot's GAP 1 comment says a
// misrouted local B would "wedge the slot in the unwatchdogged
// WS_SEND_BLOCAL" -- and worked around one specific ENTRY into the state
// instead of protecting the state.
//
// Both are reachable by the same master that motivated items 4 and 5: the
// JTAG-AXI / XDMA host on M1, behind an axi_narrow_to_wide whose own
// abandonment watchdog force-clears its valids and takes its ready low.
// M3 (dma_engine) is equally exposed, and slot 0 is exposed through the
// reset-ORDERING gap: ws_flush_abort fires on slv_flush
// (soc_full_rst_bank[4]) while the slot-0 silent-discard predicate needs
// `cpu_held_in_reset` to have ALREADY moved, and cpu_rst reaches the CPU
// through a settle/stretch chain -- so an abort in that skew takes
// need_drain=1 into WS_DRAIN_W and the CPU then stops sending.
//
// Worse than a pinned slot: WS_DRAIN_W holds WREADY HIGH forever, so when
// the master restarts, its NEXT write's beats are consumed as this dead
// burst's continuation -- the same W-channel misalignment IMPORTANT 2a
// names, arrived at from the other direction.
//
// Vehicle for both: a write to an UNMAPPED address, which takes the
// req_local route straight into WS_DRAIN_W (ws_slv = NONE), so no slave is
// ever involved and "no poison" is unambiguously the right policy.
static void scenario_50_local_b_tail_abandoned() {
    std::printf("\n=== Scenario 50: M1 walks away from its local B tail (WS_SEND_BLOCAL) "
                "-- the write slot must not be pinned ===\n");
    reset();
    set_cpu_held_in_reset(false);

    const uint32_t unmapped = 0x7000'0000u;
    const Word128  payload  = w128_from_u64(0xBAD0'B10C'BAD0'B10CULL);

    // The master is gone: BREADY low before the transaction even starts.
    dut->m1_bready = 0;
    dut->eval();

    dut->m1_awid = 0xE; dut->m1_awaddr = unmapped; dut->m1_awlen = 0;
    dut->m1_awsize = 4; dut->m1_awburst = 1; dut->m1_awvalid = 1;
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m1_awvalid = 0;
    check("50: the unmapped write's AW was accepted locally", aw_ok);

    set_wdata(dut->m1_wdata, payload);
    dut->m1_wstrb = 0xFFFF; dut->m1_wlast = 1; dut->m1_wvalid = 1;
    bool w_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_wready) { cycle(); w_ok = true; break; }
        cycle();
    }
    dut->m1_wvalid = 0; dut->m1_wlast = 0;
    check("50: its W beat was drained by the crossbar", w_ok);

    // The slot is now in WS_SEND_BLOCAL offering a B nobody will take.
    for (int i = 0; i < 4; i++) cycle();
    dut->eval();
    check("50: [positive control] the local B really is being offered",
          dut->m1_bvalid == 1);

    // Well past B_HOLD_MAX (64 in this build) the crossbar must give up.
    for (int i = 0; i < 400; i++) cycle();
    dut->eval();
    check("50: (a) the unaccepted local B tail is withdrawn",
          dut->m1_bvalid == 0);

    // BREADY deliberately stayed LOW until here for the same reason
    // scenario 29b keeps it low -- restoring it earlier would release the
    // stuck B and make everything below pass for the wrong reason.  And
    // now that it returns, NOTHING may be delivered: a tail that was
    // merely parked (not withdrawn) shows up here as a stale B that will
    // be mis-attributed to whatever this master does next.
    const int fails_before_restore = n_fail;
    dut->m1_bready = 1;
    for (int i = 0; i < 16; i++) cycle();
    // observe_b reports an unexpected BVALID as an [ERROR], so a stale tail
    // shows up as a bump in n_fail, not in w_completed_*.
    check("50: (b) no stale B arrives when BREADY returns -- the tail was really "
          "withdrawn, not just parked",
          n_fail == fails_before_restore);

    // THE PROPERTY THAT MATTERS: the write slot is usable again.
    const uint32_t addr2 = 0x5000'7000u;
    const Word128 payload2 = w128_from_u64(0x600D'B10C'600D'B10CULL);
    const int prev_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0xF, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("50: (c) M1 can write again -- the slot was not pinned",
          mb[1].w_completed_ok == prev_ok + 1 &&
          mb[1].w_expect.empty() &&
          w128_eq(s1_mem.read(addr2), payload2));
    check("50: (d) nothing was poisoned -- no slave was ever involved",
          !dut->dbg_slv_poisoned);

    mb[1].w_expect.clear();
    s1_bfm = SlaveBfm{};
    reset();
}

static void scenario_51_w_drain_master_stops_sending() {
    std::printf("\n=== Scenario 51: M1 stops sending W beats in WS_DRAIN_W -- the write "
                "slot must not be pinned ===\n");
    reset();
    set_cpu_held_in_reset(false);

    const uint32_t unmapped = 0x7000'1000u;

    dut->m1_bready = 1;
    // Unlike scenario 50's master, this one is still taking B -- it stopped
    // on the W channel only.  The crossbar must therefore ANSWER it: the
    // NONE-decode local policy is OKAY (MAME's unmap value), so that is
    // what the abandoned drain has to produce.
    mb[1].w_expect.push_back({0xA, 0});
    dut->m1_awid = 0xA; dut->m1_awaddr = unmapped; dut->m1_awlen = 3;
    dut->m1_awsize = 4; dut->m1_awburst = 1; dut->m1_awvalid = 1;
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m1_awvalid = 0;
    check("51: the unmapped 4-beat write's AW was accepted locally", aw_ok);

    // Send 2 of the 4 beats, then the master is reset away mid-burst.
    int sent = 0;
    for (int b = 0; b < 2; b++) {
        set_wdata(dut->m1_wdata, w128_from_u64(0xDEAD'0000'0000'0000ULL + (uint64_t)b));
        dut->m1_wstrb = 0xFFFF; dut->m1_wlast = 0; dut->m1_wvalid = 1;
        for (int i = 0; i < 200; i++) {
            dut->eval();
            if (dut->m1_wready) { cycle(); sent++; break; }
            cycle();
        }
    }
    dut->m1_wvalid = 0;
    dut->eval();
    check("51: two of the four beats were drained before the master left", sent == 2);

    // [positive control] the crossbar is still holding WREADY high for the
    // beats that will never come -- if this ever reads 0 the scenario has
    // stopped testing anything.
    for (int i = 0; i < 4; i++) cycle();
    dut->eval();
    check("51: [positive control] the crossbar is still waiting in WS_DRAIN_W",
          dut->m1_wready == 1 && dut->m1_bvalid == 0);

    // Well past B_HOLD_MAX the slot must be released -- and, per this
    // module's own "abandonment always produces a response" rule, the
    // master is offered its local B first (nothing has been offered to it
    // yet, unlike scenario 50's case).  If it does not take that either,
    // scenario 50's path retires it; the two compose.
    for (int i = 0; i < 600; i++) cycle();
    dut->eval();
    check("51: (a) the crossbar stopped waiting for beats that will never come",
          dut->m1_wready == 0);
    check("51: (a2) and the abandoned write was ANSWERED, not dropped",
          mb[1].w_expect.empty());

    // THE PROPERTY THAT MATTERS: the slot is usable again, and the new
    // write's beats are NOT eaten as the dead burst's continuation.
    const uint32_t addr2 = 0x5000'8000u;
    const Word128 payload2 = w128_from_u64(0xC1EA'2000'C1EA'2000ULL);
    const int prev_ok = mb[1].w_completed_ok;
    issue_write<1>(/*id*/ 0xB, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("51: (b) M1 can write again, and the data lands at the RIGHT address",
          mb[1].w_completed_ok == prev_ok + 1 &&
          mb[1].w_expect.empty() &&
          w128_eq(s1_mem.read(addr2), payload2));
    check("51: (c) nothing was poisoned -- no slave was ever involved",
          !dut->dbg_slv_poisoned);

    mb[1].w_expect.clear();
    s1_bfm = SlaveBfm{};
    reset();
}

// ── Scenario 52 (race audit, 2026-09-18) ───────────────────────────────
// AN ABANDONED CPU READ LEAKS A COUNT OUT OF `cpu_rd_outstanding`, SO THE
// ROM-OVERLAY DISARM CAN NEVER COMMIT AGAIN.
//
// The 200 MHz reset wedge was fixed by DEFERRING the overlay disarm until
// no CPU read is outstanding: `cpu_ar_fire` increments a counter,
// `cpu_r_done` (an RLAST handshake at M0/M2) decrements it, and the disarm
// commits only at zero.  The counter is therefore a ledger of "CPU reads
// the crossbar has accepted and not yet answered".
//
// Every path that retires a read WITHOUT an RLAST handshake breaks that
// ledger, and there is one: `rs_r_abandon` (backlog item 5) WITHDRAWS an
// unaccepted local R tail and idles the slot -- deliberately, because the
// master is gone.  Nothing decrements the counter for it, so it stays one
// high forever.  M2 is the CPU instruction fetch, which is exactly a
// master that can be reset out from under an in-flight read.
//
// Consequence is a silent degradation rather than a wedge: the hardware
// belt-and-braces disarm never fires again and the overlay falls back to
// VIA1 software control alone.  That is the pre-fix posture the 200 MHz
// wedge investigation concluded was not safe to rely on, reached by a
// different route and with nothing to report it.
//
// Vehicle: leak one count through an abandoned M2 read, then do a NORMAL
// high-ROM CPU fetch -- the very event whose job is to disarm the overlay
// -- and show the overlay is still aliasing low memory to ROM afterwards.
static void scenario_52_abandoned_cpu_read_leaks_overlay_ledger() {
    std::printf("\n=== Scenario 52: an abandoned CPU read must not strand the ROM-overlay "
                "disarm ledger ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t low_addr      = 0x00000000u;
    const uint32_t high_rom_addr = 0x40000000u;
    const Word128 rom_val = w128_from_u64(0x5201'0000'0000'0000ULL);
    const Word128 ram_val = w128_from_u64(0x5202'0000'0000'0000ULL);
    s0_mem.write(DDR_ROM_OFF + low_addr, rom_val);
    s0_mem.write(low_addr, ram_val);

    dut->cpu_overlay_active = 1;
    dut->cpu_overlay_reset  = 1;      // start from a known re-armed state
    for (int i = 0; i < 4; i++) cycle();
    dut->cpu_overlay_reset  = 0;
    for (int i = 0; i < 4; i++) cycle();
    check("52: [setup] the overlay starts armed", dut->cpu_overlay_effective == 1);

    // ── leak one count: M2 walks away from a local R tail ───────────────
    // 0x7000_0000 decodes to no slave, so the crossbar answers it entirely
    // out of RS_SEND_RLOCAL -- no slave is involved, and rs_r_abandon is
    // the only thing that can retire it.
    dut->m2_rready = 0;
    dut->eval();
    dut->m2_arid = 0x6; dut->m2_araddr = 0x7000'0000u; dut->m2_arlen = 0;
    dut->m2_arsize = 4; dut->m2_arburst = 1; dut->m2_arvalid = 1;
    bool ar_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m2_arready) { cycle(); ar_ok = true; break; }
        cycle();
    }
    dut->m2_arvalid = 0;
    check("52: the CPU-IF read was accepted (the ledger has counted it)", ar_ok);

    // Past B_HOLD_MAX the tail is withdrawn and the slot idles.
    for (int i = 0; i < 400; i++) cycle();
    dut->eval();
    check("52: [positive control] the abandoned tail really was withdrawn",
          dut->m2_rvalid == 0);
    mb[2].r_expect.clear();
    mb[2].r_cur.clear();
    mb[2].r_cur_resp.clear();
    dut->m2_rready = 1;
    for (int i = 0; i < 8; i++) cycle();

    // ── now the event that is SUPPOSED to disarm the overlay ────────────
    issue_read<2>(/*id*/ 0x7, high_rom_addr, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    for (int i = 0; i < 32; i++) cycle();
    dut->eval();
    check("52: (a) a high-ROM CPU fetch still disarms the overlay after an "
          "abandoned read",
          dut->cpu_overlay_disabled == 1);

    // ...and the observable consequence: low memory must now read RAM.
    issue_read<2>(/*id*/ 0x8, low_addr, /*len*/ 0, /*size*/ 4, /*rresp*/ 0);
    check("52: (b) and low memory reads RAM, not the ROM image",
          !mb[2].r_received.empty() && mb[2].r_received.back().size() == 1 &&
          w128_eq(mb[2].r_received.back()[0], ram_val));

    dut->cpu_overlay_active = 0;
    dut->cpu_overlay_reset  = 1;
    for (int i = 0; i < 4; i++) cycle();
    dut->cpu_overlay_reset  = 0;
    mb[2].r_expect.clear();
    s0_bfm_fresh();
    reset();
}

// ── Scenario 53 (race audit, 2026-09-18) ───────────────────────────────
// THE OVERLAY DISARM MUST STILL BE DEFERRED WHILE A CPU READ IS IN FLIGHT.
//
// This is the POSITIVE CONTROL for the 200 MHz reset-wedge fix, which had
// none.  ResetVectorFsm issues the 68040 reset-vector read at PHYSICAL 0,
// which only has a responder while the overlay aliases it into the ROM
// mirror.  `cpu_rom_read_seen` fires on an I-fetch REQUEST -- including a
// SPECULATIVE one -- so a wrong-path fetch could disarm the overlay WHILE
// that address-0 read was still in flight: the read then had no target, no
// R was ever produced, AxiDMerge held the RESETVEC grant, and the core
// halted with ARBITER_WEDGE ~60% of cold boots at 200 MHz.
//
// Without this scenario, scenario 52 could be "fixed" by deleting the
// deferral outright and reintroducing that wedge.  Together the two pin
// both directions: 53 says the disarm must WAIT, 52 says it must not wait
// FOREVER.
static void scenario_53_overlay_disarm_is_deferred() {
    std::printf("\n=== Scenario 53: the ROM-overlay disarm must be deferred while a CPU "
                "read is still in flight ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t low_addr      = 0x00000000u;
    const uint32_t high_rom_addr = 0x40000000u;
    const Word128 rom_val = w128_from_u64(0x5301'0000'0000'0000ULL);
    s0_mem.write(DDR_ROM_OFF + low_addr, rom_val);
    s0_mem.write(DDR_ROM_OFF,            rom_val);

    dut->cpu_overlay_active = 1;
    dut->cpu_overlay_reset  = 1;
    for (int i = 0; i < 4; i++) cycle();
    dut->cpu_overlay_reset  = 0;
    for (int i = 0; i < 4; i++) cycle();
    check("53: [setup] the overlay starts armed", dut->cpu_overlay_effective == 1);

    // ONE transaction is enough, and it is the real bug's own shape: the
    // high-ROM I-fetch that ARMS the disarm (cpu_rom_read_seen fires on
    // mr_arvalid, i.e. the REQUEST -- including a speculative one) is
    // itself the read that is still outstanding.  M2 does not take its beat
    // yet, standing in for the address-0 reset-vector read that had not
    // been answered when a wrong-path fetch disarmed the overlay.
    mb[2].r_expect.clear();
    mb[2].r_expect.push_back({0x9, 0});
    dut->m2_rready = 0;
    dut->eval();
    dut->m2_arid = 0x9; dut->m2_araddr = high_rom_addr; dut->m2_arlen = 0;
    dut->m2_arsize = 4; dut->m2_arburst = 1; dut->m2_arvalid = 1;
    bool ar_ok = false;
    for (int i = 0; i < 20; i++) {
        dut->eval();
        if (dut->m2_arready) { cycle(); ar_ok = true; break; }
        cycle();
    }
    dut->m2_arvalid = 0;
    check("53: the high-ROM CPU-IF fetch was accepted and is in flight", ar_ok);

    // Stay well inside B_HOLD_MAX (63 in this build) so rs_r_slv_abandon
    // does not retire the read for us -- the point is that it is LIVE.
    for (int i = 0; i < 12; i++) cycle();
    dut->eval();
    check("53: [positive control] it really is still unanswered",
          !mb[2].r_expect.empty());
    check("53: (a) the disarm is DEFERRED -- the overlay is still effective "
          "while that read is outstanding",
          dut->cpu_overlay_disabled == 0 && dut->cpu_overlay_effective == 1);

    // Let it complete; the disarm must then commit.
    dut->m2_rready = 1;
    for (int i = 0; i < 64; i++) cycle();
    dut->eval();
    check("53: (b) it commits once that read has been answered",
          dut->cpu_overlay_disabled == 1);
    check("53: (c) and the deferred read was ANSWERED", mb[2].r_expect.empty());

    dut->cpu_overlay_active = 0;
    dut->cpu_overlay_reset  = 1;
    for (int i = 0; i < 4; i++) cycle();
    dut->cpu_overlay_reset  = 0;
    mb[0].r_expect.clear();
    mb[2].r_expect.clear();
    s0_bfm_fresh();
    reset();
}

// ── Scenario 54 (race audit, 2026-09-18) ───────────────────────────────
// S1 MUST ALSO BE HELD CLOSED ACROSS THE **68040 RESET INSTRUCTION**.
//
// Backlog item 3 (scenario 47) covers the soc_full_rst / slv_flush case.
// The pb island has a THIRD reset source that reaches neither slv_flush nor
// this crossbar's rst:
//
//   pb_full_rst = pb_rst || debug_full_reset_pb_sync || warm_peripheral_reset
//
// `warm_peripheral_reset` is the 68040 RESET instruction -- an ordinary
// guest boot -- and it resets every Mac peripheral for ~128 core_clk plus
// the pb-side release sync while `peripheral_bus` itself stays live (it is
// deliberately on pb_soc_full_rst_bank[3] so the still-running CPU keeps a
// front door).  A transaction admitted in that window is handed straight to
// a peripheral that is in reset.
//
// Most slots survive that: their strobes are LEVELS held until ack, and
// every peripheral re-acks on release.  FOUR are one-cycle PULSES latched
// behind a "kicked" flag that only an ack can clear -- the SCSI DMA-shim
// read and write, and the ASC and ORWELL multi-byte write serialisers.  The
// pulse lands on a reset chip, the flag latches, it can never re-fire, and
// rd_busy/wr_busy stick -- which takes peripheral_bus's s_arready/s_awready
// to 0, i.e. S1 is dead TO EVERY MASTER until core_rst.  Nothing recovers
// it: ENABLE_ACK_WATCHDOG defaults to 0 and the production instantiation
// never overrides it, so that watchdog is compiled out.
//
// Same remedy as item 3, and deliberately the same one for all four
// strobes: HOLD, do not reject.  ASC and ORWELL have no dma_*_ready
// equivalent to gate on, so a front-door hold is the only mechanism that
// covers them all.
//
// ⚠ SCOPE.  This closes the transactions admitted DURING and just after the
// window.  A transaction already in flight when the far reset ASSERTS is a
// separate hole that has to be fixed inside peripheral_bus by re-arming its
// one-shot latches; it is NOT closed here and is NOT what this scenario
// tests.
static void scenario_54_s1_far_reset_tail() {
    std::printf("\n=== Scenario 54: S1 stays closed across a 68040 RESET instruction "
                "(warm_peripheral_reset), which slv_flush never sees ===\n");
    reset();
    s1_bfm = SlaveBfm{};
    set_cpu_held_in_reset(false);
    // `rst` arms the tail too; let that one expire so what follows is
    // attributable to s1_far_reset alone.
    for (int i = 0; i < 64; i++) cycle();

    const uint32_t waddr = 0x5000'9000u;
    const uint32_t raddr = 0x5000'9100u;
    const Word128 payload = w128_from_u64(0x5401'0000'5401'0000ULL);

    s1_in_reset = true;
    s1_reset_swallowed_aw = 0;
    s1_reset_swallowed_ar = 0;

    // POSITIVE CONTROL for the whole scenario: slv_flush is NOT what is
    // happening here.  If this were driven instead, scenario 47 would
    // already cover it and 54 would prove nothing.
    check("54: [positive control] slv_flush is deasserted throughout",
          dut->slv_flush == 0);

    dut->s1_far_reset = 1;
    dut->eval();
    cycle(); cycle();
    dut->s1_far_reset = 0;
    dut->eval();

    // Offer a write AND a read the very next cycle.
    mb[1].w_expect.push_back({0x4, 0});
    dut->m1_awid = 0x4; dut->m1_awaddr = waddr; dut->m1_awlen = 0;
    dut->m1_awsize = 4; dut->m1_awburst = 1; dut->m1_awvalid = 1;
    mb[1].r_expect.push_back({0x5, 0});
    dut->m1_arid = 0x5; dut->m1_araddr = raddr; dut->m1_arlen = 0;
    dut->m1_arsize = 4; dut->m1_arburst = 1; dut->m1_arvalid = 1;

    // Comfortably inside the 32-cycle tail this build is parameterised to,
    // and comfortably outside the 1-2 cycles an ungated admission takes.
    for (int i = 0; i < 8; i++) cycle();
    char nm[200];
    std::snprintf(nm, sizeof(nm),
                  "54: (a) no S1 AW reached the resetting pb island during the tail "
                  "(swallowed=%d, want 0)", s1_reset_swallowed_aw);
    check(nm, s1_reset_swallowed_aw == 0);
    // The READ side carries the identical term in the RTL and scenario 47
    // never asserted on it -- s1_reset_swallowed_ar was tracked and unused.
    std::snprintf(nm, sizeof(nm),
                  "54: (b) and no S1 AR did either (swallowed=%d, want 0) -- the read "
                  "side of the same gate", s1_reset_swallowed_ar);
    check(nm, s1_reset_swallowed_ar == 0);

    // The island finishes leaving reset; both held requests must then go
    // through normally.  A bounded STALL, not a drop and not a manufactured
    // bus error.
    s1_in_reset = false;
    s1_mem.write(raddr, payload);
    bool aw_ok = false;
    for (int i = 0; i < 400; i++) {
        dut->eval();
        if (dut->m1_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m1_awvalid = 0;
    check("54: (c) the held AW is accepted once the tail expires", aw_ok);

    set_wdata(dut->m1_wdata, payload);
    dut->m1_wstrb = 0xFFFF; dut->m1_wlast = 1; dut->m1_wvalid = 1;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m1_wready) { cycle(); break; }
        cycle();
    }
    dut->m1_wvalid = 0; dut->m1_wlast = 0;
    for (int i = 0; i < 400 && (!mb[1].w_expect.empty() || !mb[1].r_expect.empty()); i++) {
        dut->eval();
        if (dut->m1_arready) dut->m1_arvalid = 0;
        cycle();
    }
    dut->m1_arvalid = 0;
    check("54: (d) the write completes OKAY and reached the real slave",
          mb[1].w_expect.empty() && w128_eq(s1_mem.read(waddr), payload));
    check("54: (e) the held READ completes too", mb[1].r_expect.empty());
    check("54: (f) S1 was not poisoned by any of this", !dut->dbg_slv_poisoned);

    mb[1].w_expect.clear();
    mb[1].r_expect.clear();
    s1_bfm = SlaveBfm{};
    reset();
}

// ── Scenario 55 (race audit, 2026-09-18) ───────────────────────────────
// THE SLOT-0 ABANDON IS ALSO BLIND TO boot_fsm's OWN RESET.
//
// Scenario 42d pins the soc_full_rst / slv_flush arming of m0_abandon.  Its
// predicate has a THIRD arming event it still missed:
//
//     boot_fsm_rst = soc_full_rst || jtag_boot_bypass || dbg_cold_reset_hold
//
// `dbg_cold_reset_hold` is DBG_CONTROL[4], a sticky level a JTAG host raises
// at an arbitrary cycle.  It resets boot_fsm -- which forgets an in-flight
// ROM-copy burst -- while `cpu_held_in_reset` STAYS 1, because the hold is
// itself a cpu_rst term, and `soc_full_rst` never asserts.  So the mismatch
// term (m0_wsel_q != cpu_held_in_reset) is blind, exactly as it was before
// d85deb57, and slv_flush is low.
//
// Consequence: slot 0 sits in WS_FWD_W with S0/DDR parked mid-burst (or in
// WS_WAIT_B holding a B nobody will take); sw_owned[0] stays set and DDR is
// dead to EVERY master.  `rst` here is core_rst_bank[4], which a debug reset
// does not assert, so only a reconfigure clears it.
//
// Nothing downstream of the predicate needed changing: the WS_FWD_W
// m0_abandon branch already completes the slave's burst with wstrb=0 filler
// and a correct WLAST, and the WS_WAIT_B branch already swallows the
// orphaned B.  The machinery was right; it was never told about this event.
//
// Run both variants, exactly as 42d does, and carry 42d's own key positive
// control: the select must be genuinely UNCHANGED, and slv_flush must stay
// LOW throughout -- otherwise this scenario would merely be 42d again.
static void scenario_55_boot_master_reset_abandon(bool wait_b_variant) {
    std::printf("\n=== Scenario 55%s: boot_fsm reset by its OWN rst (dbg_cold_reset_hold) "
                "-- slv_flush low, select unchanged ===\n",
                wait_b_variant ? " (WS_WAIT_B)" : " (WS_FWD_W)");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(true);

    const uint32_t addr = BURST_ADDR_BASE + 0xC000u + (wait_b_variant ? 0x400u : 0u);
    const int beats      = wait_b_variant ? 1 : 4;
    const int stop_after = wait_b_variant ? 1 : 2;
    const Word128 sentinel = w128_from_u64(0x5501'AAAA'5501'AAAAULL);
    for (int b = 0; b < beats; b++) s0_mem.write(addr + (uint32_t)b * 16, sentinel);

    if (wait_b_variant) dut->m0b_bready = 0;

    dut->m0b_awid = 0x9; dut->m0b_awaddr = addr;
    dut->m0b_awlen = (uint8_t)(beats - 1); dut->m0b_awsize = 4;
    dut->m0b_awburst = 1; dut->m0b_awvalid = 1;
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0b_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m0b_awvalid = 0;
    check("55: boot-side AW accepted while cpu_held_in_reset=1", aw_ok);

    std::vector<Word128> sent;
    bool w_ok = true;
    for (int b = 0; b < stop_after; b++) {
        Word128 d = w128_from_u64(0xB007'5500'0000'0000ULL + (uint64_t)b);
        sent.push_back(d);
        set_wdata(dut->m0b_wdata, d);
        dut->m0b_wstrb = 0xFFFF;
        dut->m0b_wlast = (b == beats - 1) ? 1 : 0;
        dut->m0b_wvalid = 1;
        bool beat_ok = false;
        for (int i = 0; i < 200; i++) {
            dut->eval();
            if (dut->m0b_wready) { cycle(); beat_ok = true; break; }
            cycle();
        }
        dut->m0b_wvalid = 0; dut->m0b_wlast = 0;
        w_ok = w_ok && beat_ok;
    }
    check("55: boot-side W beats accepted", w_ok);

    for (int i = 0; i < 10 && !s0_bfm.aw_busy; i++) cycle();
    if (wait_b_variant) {
        for (int i = 0; i < 6; i++) cycle();
        check("55: burst closed on the slave, B outstanding to the xbar",
              s0_bfm.w_beats_left == 0 && s0_bfm.b_pending);
    } else {
        check("55: S0 burst left open with beats owed",
              s0_bfm.aw_busy && s0_bfm.w_beats_left == beats - stop_after);
    }

    // ── THE EVENT: boot_fsm's own reset.  NOT soc_full_rst. ─────────────
    dut->m0b_wvalid = 0; dut->m0b_wlast = 0; dut->m0b_awvalid = 0;
    check("55: [positive control] the select really is unchanged "
          "(cpu_held_in_reset still 1)", dut->cpu_held_in_reset == 1);
    check("55: [positive control] slv_flush is LOW -- this is not 42d's event",
          dut->slv_flush == 0);
    dut->m0b_master_reset = 1;
    dut->eval();
    cycle(); cycle();
    dut->m0b_master_reset = 0;
    dut->eval();

    // Recovery must come from the abandon path, not the 4096-cycle watchdog.
    int cycles_to_done = -1;
    for (int i = 0; i < 64; i++) {
        cycle();
        if (!s0_bfm.aw_busy && s0_bfm.w_beats_left == 0 && !s0_bfm.b_pending) {
            cycles_to_done = i + 1;
            break;
        }
    }
    check("55: (a) the slave's burst completed and its B was swallowed, well "
          "inside the watchdog window", cycles_to_done > 0);

    if (!wait_b_variant) {
        check("55: (b) exactly AWLEN+1 beats reached the slave, the tail as "
              "WSTRB=0 filler",
              s0_bfm.w_beats_seen == beats &&
              s0_bfm.w_pad_beats == beats - stop_after);
        check("55: WLAST landed on the final beat only", s0_bfm.w_wlast_errors == 0);
        bool mem_ok = true;
        for (int b = 0; b < beats; b++) {
            Word128 got = s0_mem.read(addr + (uint32_t)b * 16);
            mem_ok = mem_ok && w128_eq(got, (b < stop_after) ? sent[b] : sentinel);
        }
        check("55: (c) filler beats wrote nothing (the real beats are intact)", mem_ok);
    }

    // THE PAYOFF: DDR is usable again -- by the restarted boot FSM itself.
    dut->m0b_bready = 1;
    for (int i = 0; i < 8; i++) cycle();
    const uint32_t addr2 = addr + 0x200u;
    const Word128 payload2 = w128_from_u64(0x5502'BBBB'5502'BBBBULL);
    const int prev_ok = mb[3].w_completed_ok;
    mb[3].w_expect.push_back({0xA, 0});
    dut->m0b_awid = 0xA; dut->m0b_awaddr = addr2; dut->m0b_awlen = 0;
    dut->m0b_awsize = 4; dut->m0b_awburst = 1; dut->m0b_awvalid = 1;
    bool aw2 = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0b_awready) { cycle(); aw2 = true; break; }
        cycle();
    }
    dut->m0b_awvalid = 0;
    set_wdata(dut->m0b_wdata, payload2);
    dut->m0b_wstrb = 0xFFFF; dut->m0b_wlast = 1; dut->m0b_wvalid = 1;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0b_wready) { cycle(); break; }
        cycle();
    }
    dut->m0b_wvalid = 0; dut->m0b_wlast = 0;
    for (int i = 0; i < 300 && !mb[3].w_expect.empty(); i++) cycle();
    check("55: (d) slot 0 is usable again -- the restarted boot FSM's next "
          "write completes and lands",
          aw2 && mb[3].w_completed_ok == prev_ok + 1 &&
          mb[3].w_expect.empty() && w128_eq(s0_mem.read(addr2), payload2));
    check("55: (e) S0 was NOT poisoned -- it was healthy all along",
          !(dut->dbg_slv_poisoned & 0x01));

    mb[0].w_expect.clear();
    mb[3].w_expect.clear();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);
    reset();
}

// ── Scenario 43 (task: xbar-burst-gaps, GAP 2) ─────────────────────────
// A slot aborting out of WS_WAIT_SLV_AW whose W burst is ALREADY fully
// accepted by the slave must go straight to the local B, not via
// WS_DRAIN_W.
//
// `need_drain` used to be an unconditional 1'b1 at all six WS_WAIT_SLV_AW
// abort sites.  A slave that FIFOs W ahead of AW — axi_async_bridge, which
// sits behind S1, does exactly that — leaves ws_wdone=1 with no beats left
// for the master to send, so WS_DRAIN_W waits forever for beats that never
// arrive.  WS_DRAIN_W is excluded from ws_active, so NO watchdog rescues
// it: the master hangs rather than getting its SLVERR.  Pre-existing, not
// caused by bursts.
//
// Vehicle: S0 in early-W mode (async-bridge-style W FIFO) with its AW
// channel wedged, so the write really does sit in WS_WAIT_SLV_AW with
// ws_wdone=1 until the watchdog fires.
static void scenario_43_wdone_abort_skips_drain() {
    std::printf("\n=== Scenario 43: WS_WAIT_SLV_AW abort with W already done must not hang in WS_DRAIN_W ===\n");
    reset();
    s0_bfm_fresh();
    set_cpu_held_in_reset(false);

    const uint32_t addr = BURST_ADDR_BASE + 0x6000u;
    const Word128 payload = w128_from_u64(0x0DD0'0DD0'0DD0'0DD0ULL);

    s0_early_w_accept = true;      // slave FIFOs W ahead of AW
    s0_stall_aw       = 1000000;   // ...and never accepts the AW

    const int prev_err = mb[0].w_completed_errs;
    const int prev_ok  = mb[0].w_completed_ok;
    mb[0].w_expect.push_back({0x31, 2 /*SLVERR — the watchdog's response*/});

    dut->m0_awid = 0x31; dut->m0_awaddr = addr; dut->m0_awlen = 0;
    dut->m0_awsize = 4; dut->m0_awburst = 1; dut->m0_awvalid = 1;
    bool aw_ok = false;
    for (int i = 0; i < 200; i++) {
        dut->eval();
        if (dut->m0_awready) { cycle(); aw_ok = true; break; }
        cycle();
    }
    dut->m0_awvalid = 0;
    check("wdone-abort: xbar accepted M0's AW", aw_ok);

    // The single W beat is accepted by the slave's early-W FIFO, so the
    // xbar latches ws_wdone=1 while still in WS_WAIT_SLV_AW.
    bool w_ok = m0_w_beat(payload, /*last*/ true);
    check("wdone-abort: W beat accepted BEFORE the AW (early-W FIFO)", w_ok);
    check("wdone-abort: slave really did FIFO the beat ahead of its AW",
          s0_early_w_have && !s0_bfm.aw_busy);

    // The master is now finished — it has nothing left to send and just
    // waits for B.  Let the watchdog fire.
    for (int i = 0; i < WD_WAIT_CYCLES; i++) cycle();

    check("wdone-abort: master received its terminal BRESP=SLVERR "
          "(did NOT hang in an unwatchdogged WS_DRAIN_W)",
          mb[0].w_completed_errs == prev_err + 1 &&
          mb[0].w_completed_ok == prev_ok);
    check("wdone-abort: B-expect queue drained", mb[0].w_expect.empty());

    // Clean up: this scenario's own watchdog poisoned S0 (correctly — the
    // slave really was wedged).  Clear it the same way scenario 33 does.
    s0_stall_aw       = 0;
    s0_early_w_accept = false;
    s0_early_w_have   = false;
    s0_bfm.aw_busy      = false;
    s0_bfm.w_beats_left = 0;
    dut->slv_flush = 1;
    for (int i = 0; i < 5; i++) cycle();
    dut->slv_flush = 0;
    for (int i = 0; i < 10; i++) cycle();

    const uint32_t addr2 = addr + 0x40u;
    const Word128 payload2 = w128_from_u64(0xC0FF'EE00'C0FF'EE00ULL);
    int prev_ok2 = mb[0].w_completed_ok;
    issue_write<0>(/*id*/ 0x32, addr2, 0, 4, {payload2}, /*expect_bresp*/ 0);
    check("wdone-abort: DDR usable again after the flush clears the poison",
          mb[0].w_completed_ok == prev_ok2 + 1 &&
          w128_eq(s0_mem.read(addr2), payload2));

    s0_bfm_fresh();
    reset();
}

// Scenario 44 (2026-09-17): a slot-0 READ aborted while cpu_held_in_reset MUST be
// terminated with a well-formed SLVERR tail -- it must NOT be discarded silently.
//
// This is the READ twin of scenario 38, and it deliberately asserts the OPPOSITE
// contract, because the two sides are not symmetric.  `rs_release_slot` used to carry
// the same slot-0 silent-discard branch `ws_release_slot` has:
//
//     if ((slot == 0) && cpu_held_in_reset) rs_state[slot] <= RS_IDLE;   // no R at all
//
// justified as "that requester no longer exists".  It does exist.  The CPU socket
// carries `AxiReadResetAbsorber` (cpu040 src/main/scala/m68k040/socket/), added to
// bridge the reset-domain asymmetry between the core (`cpu_rst`) and this crossbar
// (`core_rst_bank[4]`).  It counts reads THIS module has accepted, it lives in a
// `resetKind = BOOT` domain precisely so `cpu_rst` cannot clear it, and it refuses to
// let the core issue any new AR until the count returns to zero.
//
// So a discarded read leaves that counter stuck forever: the CPU never issues another
// address -- not even the reset-vector fetch -- its merge arbiter sits on a grant with
// no progress, and D20's bounded-grant watchdog halts the core with ARBITER_WEDGE.
// Nothing recovers it, because no reset in this system clears a BOOT-domain register.
// Measured on the board for months as "a JTAG `reset` kills it and only reprogramming
// brings it back".
//
// The write side keeps its discard and scenario 38 still pins it: nothing on the core
// side counts writes, and there the discard also protects boot_fsm's B accounting.
//
// Note this fires WITHOUT slv_flush: rs_flush_abort's own slot-0 term is
// `(mi == 0) && (rs_slv == XBAR_SLV_IO) && cpu_held_in_reset`.
static void scenario_44_slot0_read_must_be_terminated() {
    std::printf("\n=== Scenario 44: slot-0 read aborted under cpu_held_in_reset is TERMINATED, not discarded ===\n");
    uint32_t addr = 0x50006000u;  // S1 (peripheral / XBAR_SLV_IO)

    s1_swallow_ar = true;
    int prev_ok  = mb[0].r_completed;
    int prev_err = mb[0].r_errors;
    size_t prev_bursts = mb[0].r_received.size();

    // In flight and unanswerable: the slave never takes the AR, so the slot parks in
    // RS_WAIT_SLV_AR -- exactly where a reset lands on a running machine.
    issue_read<0>(/*id*/ 0x7, addr, /*len*/ 0, /*size*/ 4, /*expect_rresp*/ 2 /*SLVERR*/);
    check("slot-0 read is genuinely in flight (S1 never accepted the AR)",
          !mb[0].r_expect.empty() &&
          mb[0].r_completed == prev_ok && mb[0].r_errors == prev_err);

    // The CPU goes into reset while that read is outstanding.
    set_cpu_held_in_reset(true);
    for (int i = 0; i < 60; i++) cycle();

    // THE CONTRACT.  Before the 2026-09-17 change this read vanished and every check
    // below failed -- which is the whole point of the scenario.
    check("the aborted slot-0 read is ANSWERED (terminal SLVERR), not silently discarded",
          mb[0].r_errors == prev_err + 1);
    check("the master's read expectation queue drained -- nothing left waiting forever",
          mb[0].r_expect.empty());
    check("exactly one burst was delivered for it",
          mb[0].r_received.size() == prev_bursts + 1);
    if (mb[0].r_received.size() == prev_bursts + 1 &&
        mb[0].r_received_resp.size() == prev_bursts + 1) {
        const auto& resps = mb[0].r_received_resp.back();
        check("len=0 abort delivers exactly one beat",   resps.size() == 1);
        check("and that beat carries RRESP=SLVERR",      resps.size() == 1 && resps[0] == 2);
    } else {
        check("len=0 abort delivers exactly one beat", false);
        check("and that beat carries RRESP=SLVERR",    false);
    }

    // Cleanup: release the hold, un-swallow, and let S1 settle.
    set_cpu_held_in_reset(false);
    s1_swallow_ar  = false;
    s1_bfm.ar_busy = false;
    for (int i = 0; i < 20; i++) cycle();

    // Not poisoned by the abort: S1 must still serve a real read.
    s1_mem.write(addr, w128_from_u64(0xC0FFEE0000000044ULL));
    int prev_ok2 = mb[0].r_completed;
    issue_read<0>(/*id*/ 0x8, addr, /*len*/ 0, /*size*/ 4, /*expect_rresp*/ 0);
    check("S1 still usable after the abort -- a following CPU read succeeds",
          mb[0].r_completed == prev_ok2 + 1);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaxi_xbar;
    reset();

    scenario_1_cpu_read_ram();
    scenario_2_cpu_write_rom_dropped();
    scenario_3_boot_write_rom_ok();
    // M0/boot merge correctness (2026-07-16 master-count reduction).
    scenario_3c_boot_merge_cpu_blocked_during_reset();
    scenario_3d_boot_merge_cpu_after_release();
    scenario_3e_boot_merge_adversarial_exclusivity();
    scenario_3b_host_debug_write_rom_ok();
    // DELIBERATELY UNREGISTERED known-red finding (address-decode audit,
    // 2026-08): the ROM mirror flattens with a 4 MiB period where MAME
    // uses 1 MiB, so 3 of every 4 MiB read uninitialised DDR.  See the
    // comment block above the scenario.  This tb has no XFAIL mechanism
    // (check() bumps a global fail count), so leaving the call commented
    // out is how the finding is parked without breaking `make tb-all`.
    // NOW REGISTERED — the flatten mask was fixed (ROM_IMAGE_SIZE, task
    // #245).  Moved to the END of the run order because it writes
    // DDR_ROM_OFF + 0, which the overlay scenarios above pre-seed; running
    // it here produced 3 collateral failures on top of the 3 real ones.
    scenario_4_hdmi_read_fb();
    scenario_5_xdma_read_io();
    scenario_6_contention();
    scenario_7_decerr();
    scenario_8_back_to_back_16x();
    scenario_9_abort_before_awready();
    scenario_10_decerr_all_unmapped();
    scenario_11_cross_slave_parallel();
    // scenario_12/14/15 (M4/DMA-master) are #if 0'd out — see the
    // task #116 comment above scenario_13.  scenario_13 (DMA config via
    // S2) is unaffected and still runs.
    scenario_13_cpu_write_dma_cfg();
    scenario_16_three_slave_parallel();
    // tb-axi-xbar-stress extensions (task #82)
    scenario_17_ram_window_edges();
    scenario_18_rom_window_top_edge();
    scenario_19_dma_hole_boundary();
    scenario_20_fb_window_top_edge();
    scenario_21_three_master_single_slave();
    scenario_22_waw_same_addr_two_masters();
    // VRAM aperture routing (task #147 — xbar-vram-wire).
    scenario_23_vram_aperture();
    scenario_24_ddr_partial_write_mask();
    scenario_25_ram_window_reprogram();
    scenario_26_cpu_overlay_decode();
    scenario_26b_cpu_overlay_rearm_without_xbar_reset();
    scenario_27_q700_ram_alias();
    // 2026-07-21 write/read dead-cycle removal
    scenario_28_early_w_same_cycle_as_aw();
    scenario_29_w_complete_before_aw_accept();
    scenario_30_read_no_dead_cycle();
    // xbar-burst-guard (task T5): burst legalization into single-beat
    // slaves + per-slave watchdog/poison.
    scenario_28_burst_write_lite_slave_legalized();
    scenario_29_burst_read_lite_slave_legalized();
    // Task-review follow-up (CRITICAL slv_flush fix, IMPORTANT 2/3 timing
    // fixes) — all target S2/S0, never S1, so scenario 30 below still
    // starts from a pristine S1.
    scenario_31_slv_flush_abort_mid_transaction();
    scenario_32_read_watchdog_and_flush_poison_clear();
    scenario_33_mid_burst_write_timeout();
    scenario_34_mid_burst_read_timeout();
    scenario_35_read_timeout_len255_edge();
    // Task-review round 2: flush-domain-membership fix (S1 excluded, S3
    // included) + IMPORTANT-3 slot-0 discard under cpu_held_in_reset.
    scenario_36_s1_flush_abort();
    scenario_37_s3_flush_abort();
    scenario_38_slot0_discard_under_cpu_held();
    // Task #219: master-walk-away in WS_WAIT_B.  MUST run before
    // scenario 30, which poisons S1 for the rest of the run.
    scenario_29b_wait_b_master_walks_away();
    scenario_30_slave_watchdog_poison();
    // Multi-beat 128-bit INCR bursts on the CPU data path (the CPU's
    // D-cache burst refill becomes this crossbar's first multi-beat
    // master), and the M0/boot fan-in mux flipping mid-burst.
    scenario_39_cpu_multibeat_burst_ddr();
    scenario_40_cpu_held_flips_mid_w_burst();
    scenario_41_boot_burst_survives_cpu_release();
    // xbar-burst-gaps: GAP 1 (abandoned burst is padded, not poisoned) and
    // GAP 2 (WS_WAIT_SLV_AW abort with W already done skips WS_DRAIN_W).
    // Run last: scenario 43 deliberately provokes a real watchdog timeout.
    for (int beats : {2, 4, 8})
        for (int stop_after = 0; stop_after < beats; stop_after++)
            scenario_42_abandoned_burst_is_padded(beats, stop_after);
    scenario_42b_abandon_with_aw_still_pending();
    scenario_42c_abandon_in_wait_b();
    scenario_42d_boot_side_abandon_via_flush(/*wait_b_variant*/ false);
    scenario_42d_boot_side_abandon_via_flush(/*wait_b_variant*/ true);
    scenario_55_boot_master_reset_abandon(/*wait_b_variant*/ false);
    scenario_55_boot_master_reset_abandon(/*wait_b_variant*/ true);
    scenario_45_s3_read_quarantine_is_per_slot();
    scenario_45b_no_ar_admitted_while_a_stale_burst_is_owed();
    scenario_46_local_r_tail_abandoned<1>("host debug", 0x7000'0000u);
    scenario_46_local_r_tail_abandoned<2>("CPU instruction fetch", 0x7000'0000u);
    scenario_47_s1_reset_deassert_tail();
    scenario_54_s1_far_reset_tail();
    scenario_48_r_master_walks_away_midburst();
    scenario_48b_slot0_ddr_read_survives_cpu_reset();
    scenario_49_drain_r_flush_must_answer_and_quarantine();
    scenario_50_local_b_tail_abandoned();
    scenario_51_w_drain_master_stops_sending();
    scenario_52_abandoned_cpu_read_leaks_overlay_ledger();
    scenario_53_overlay_disarm_is_deferred();
    scenario_43_wdone_abort_skips_drain();
    scenario_44_slot0_read_must_be_terminated();
    // Task #245 regression: ROM mirror must repeat every 1 MiB (MAME), not
    // 4 MiB.  Last, because it dirties DDR_ROM_OFF + 0.
    scenario_rom_mirror_granularity();

    std::printf("\n%s — %d passed, %d failed (%lu cycles)\n",
                (n_fail == 0 ? "All checks PASSED" : "Some checks FAILED"),
                n_pass, n_fail, (unsigned long)sim_time);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
