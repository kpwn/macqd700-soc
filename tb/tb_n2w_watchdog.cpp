// tb_n2w_watchdog.cpp — directed unit tb for the axi_narrow_to_wide
// abandoned-transaction watchdog.
//
// WHY THIS EXISTS
// ───────────────
// axi_narrow_to_wide sits between every narrow (32-bit) master in this SoC
// and the 128-bit fabric.  It holds ONE outstanding transaction per
// direction and gates n_awready/n_arready directly on that latch, so a wide
// side that never answers wedges the master permanently.  The abandonment
// watchdog is the only mechanism that turns that permanent SoC hang into a
// diagnosable SLVERR.
//
// It had no test of its own in this repo.  The only TIMEOUT_CYCLES override
// anywhere (tb/tb_l2c_wstream.v) exists to stop the watchdog firing during a
// long run, not to exercise it, and tb_n2w_vram_byte.v drives a full
// VRAM/xbar chain that cannot be stalled.  A liveness mechanism whose
// failure mode is a permanent hang had therefore never been observed
// working — and it has since been switched OFF by default
// (ENABLE_ABANDON_TIMEOUT=0) to keep a 20-second timer off the live WREADY
// handshake.  This tb is what makes turning it back on safe.
//
// TWO BUILDS, ONE SOURCE
// ──────────────────────
//   N2W_WATCHDOG_ENABLED=1 (built with -GENABLE_ABANDON_TIMEOUT=1)
//       Scenarios 1-5 and 7: the watchdog's actual behaviour.
//   N2W_WATCHDOG_ENABLED=0 (built with -GENABLE_ABANDON_TIMEOUT=0)
//       Scenario 6: with the watchdog compiled out, a stalled transaction
//       must hang CLEANLY AND FOREVER — no spurious SLVERR, no counter
//       side-effects, no state corruption, and a very late wide-side answer
//       must still complete normally.  This pins the disabled behaviour so
//       nobody later assumes the timeout is still there.
//
// SCENARIOS (enabled build)
//   1  AW expiry: BRESP=SLVERR + full write-side state teardown.
//   2  AR expiry: RRESP=SLVERR with correct beat accounting, including a
//      multi-beat read abandoned partway through (the case the RTL's own
//      comment near ar_err_left_q warns about: a single SLVERR beat on a
//      multi-beat read would leave the master waiting for the rest).
//   3  No false trip while progress drips in just under the timeout.
//   4  The exact boundary, expressed in the only frame that is physically
//      meaningful — CYCLES SINCE THE LAST PROGRESS EVENT, not the value of
//      an internal counter.  Fires iff the gap exceeds TIMEOUT_CYCLES+1.
//   5  Clean rearm after a fire, on both the "late stale beat arrives" and
//      the "wide side stays dead" recovery paths.
//   7  ADVERSARIAL: progress landing on the exact expiry cycle.  Asserts
//      END-TO-END AXI invariants (total beat counts, bounded response
//      latency) rather than internal state, so it is valid against any
//      implementation of the progress signal.
//
// Built via: make tb-n2w-watchdog
// Pass output: "All N checks PASSED."

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <verilated.h>
#include "Vtb_n2w_watchdog.h"

#ifndef N2W_WATCHDOG_ENABLED
#define N2W_WATCHDOG_ENABLED 1
#endif

// MUST match TIMEOUT_CYCLES in tb/tb_n2w_watchdog.v and the -G override in
// the Makefile rule.
static const int kTimeout = 64;

static const int kRespOkay   = 0;
static const int kRespSlverr = 2;

using Word128 = std::array<uint32_t, 4>;

static Vtb_n2w_watchdog* dut = nullptr;
static long g_cycle  = 0;
static int  n_pass   = 0;
static int  n_fail   = 0;

// ── Narrow-side response log ──────────────────────────────────────────
// Every narrow B/R handshake is recorded.  Beat COUNTS are the whole point
// of the read-side checks — "a response arrived" cannot distinguish a
// correct 8-beat answer from a 9-beat one that desyncs the master forever.
struct RBeat { uint32_t data; int resp; int last; long cyc; };
struct BBeat { int resp; long cyc; };
static std::vector<RBeat> r_log;
static std::vector<BBeat> b_log;

// ── Sticky observations, sampled every cycle ──────────────────────────
static uint32_t g_aw_cnt_max     = 0;
static uint32_t g_ar_cnt_max     = 0;
static bool     g_aw_fire_seen   = false;
static bool     g_ar_fire_seen   = false;
static long     g_aw_fire_cycle  = -1;
static long     g_ar_fire_cycle  = -1;

static void obs_clear() {
    g_aw_cnt_max = g_ar_cnt_max = 0;
    g_aw_fire_seen = g_ar_fire_seen = false;
    g_aw_fire_cycle = g_ar_fire_cycle = -1;
    r_log.clear();
    b_log.clear();
}

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    g_cycle++;
}

// One clock: settle combinationally with the inputs the caller has set,
// record any narrow-side handshake happening THIS cycle, then advance.
static void cyc() {
    dut->eval();
    if (dut->n_rvalid && dut->n_rready)
        r_log.push_back({(uint32_t)dut->n_rdata, (int)dut->n_rresp,
                         (int)dut->n_rlast, g_cycle});
    if (dut->n_bvalid && dut->n_bready)
        b_log.push_back({(int)dut->n_bresp, g_cycle});
    tick();
    if (dut->dbg_aw_timeout_cnt > g_aw_cnt_max) g_aw_cnt_max = dut->dbg_aw_timeout_cnt;
    if (dut->dbg_ar_timeout_cnt > g_ar_cnt_max) g_ar_cnt_max = dut->dbg_ar_timeout_cnt;
    // A write expiry is the only thing that ever sets aw_err_q or aw_drain_q;
    // a read expiry is the only thing that sets ar_drain_q or ar_err_left_q.
    if (!g_aw_fire_seen && (dut->dbg_aw_err_q || dut->dbg_aw_drain_q)) {
        g_aw_fire_seen  = true;
        g_aw_fire_cycle = g_cycle;
    }
    if (!g_ar_fire_seen && (dut->dbg_ar_drain_q || dut->dbg_ar_err_left_q)) {
        g_ar_fire_seen  = true;
        g_ar_fire_cycle = g_cycle;
    }
}

static void run(int n) { for (int i = 0; i < n; i++) cyc(); }

static void reset() {
    dut->rst       = 1;
    dut->n_awaddr  = 0; dut->n_awprot = 0; dut->n_awlen = 0; dut->n_awvalid = 0;
    dut->n_wdata   = 0; dut->n_wstrb  = 0; dut->n_wlast = 1; dut->n_wvalid  = 0;
    dut->n_bready  = 1;
    dut->n_araddr  = 0; dut->n_arprot = 0; dut->n_arsize = 2; dut->n_arlen = 0;
    dut->n_arvalid = 0; dut->n_rready = 1;
    dut->w_awready = 0; dut->w_wready = 0;
    dut->w_bid     = 0; dut->w_bresp  = 0; dut->w_bvalid = 0;
    dut->w_arready = 0; dut->w_rid    = 0; dut->w_rresp  = 0;
    dut->w_rlast   = 0; dut->w_rvalid = 0;
    for (int i = 0; i < 4; i++) dut->w_rdata[i] = 0;
    for (int i = 0; i < 3; i++) cyc();
    dut->rst = 0;
    cyc();
    obs_clear();
}

static void check(const char* name, bool ok, const std::string& detail = "") {
    if (ok) { n_pass++; printf("  PASS: %s\n", name); }
    else    { n_fail++; printf("  FAIL: %s%s%s\n", name,
                               detail.empty() ? "" : " — ", detail.c_str()); }
}

static std::string dec(long v) { return std::to_string(v); }
static std::string hex32(uint32_t v) {
    char b[16]; snprintf(b, sizeof b, "0x%08x", v); return std::string(b);
}

// ── Narrow-master primitives ──────────────────────────────────────────

// Present AW (and optionally the first W beat) and cycle until AW is
// accepted.  Returns the absolute cycle of the accepting handshake, or -1.
// The caller sets n_wdata/n_wstrb/n_wlast beforehand when with_w is true.
static long drive_aw(uint32_t addr, int len, bool with_w) {
    dut->n_awaddr  = addr;
    dut->n_awlen   = (uint8_t)len;
    dut->n_awvalid = 1;
    if (with_w) dut->n_wvalid = 1;
    for (int i = 0; i < 4 * kTimeout; i++) {
        dut->eval();
        bool aw_acc = dut->n_awready && dut->n_awvalid;
        bool w_acc  = dut->n_wready  && dut->n_wvalid;
        long at     = g_cycle;
        cyc();
        if (w_acc)  dut->n_wvalid  = 0;
        if (aw_acc) { dut->n_awvalid = 0; return at; }
    }
    dut->n_awvalid = 0;
    return -1;
}

// Hand over one narrow W beat, cycling until it is accepted.  Returns the
// absolute cycle of the accepting handshake, or -1.
static long drive_w(uint32_t data, int strb, bool last, int limit) {
    dut->n_wdata  = data;
    dut->n_wstrb  = (uint8_t)strb;
    dut->n_wlast  = last ? 1 : 0;
    dut->n_wvalid = 1;
    for (int i = 0; i < limit; i++) {
        dut->eval();
        bool acc = dut->n_wready && dut->n_wvalid;
        long at  = g_cycle;
        cyc();
        if (acc) { dut->n_wvalid = 0; return at; }
    }
    dut->n_wvalid = 0;
    return -1;
}

static long drive_ar(uint32_t addr, int len, int size) {
    dut->n_araddr  = addr;
    dut->n_arlen   = (uint8_t)len;
    dut->n_arsize  = (uint8_t)size;
    dut->n_arvalid = 1;
    for (int i = 0; i < 4 * kTimeout; i++) {
        dut->eval();
        bool acc = dut->n_arready && dut->n_arvalid;
        long at  = g_cycle;
        cyc();
        if (acc) { dut->n_arvalid = 0; return at; }
    }
    dut->n_arvalid = 0;
    return -1;
}

// ── Wide-slave primitives ─────────────────────────────────────────────

static void wide_stall_all() {
    dut->w_awready = 0; dut->w_wready = 0; dut->w_bvalid = 0;
    dut->w_arready = 0; dut->w_rvalid = 0;
}

// Accept the wide AW the DUT is presenting.  Returns the accepting cycle.
static long wide_accept_aw(int limit) {
    dut->w_awready = 1;
    for (int i = 0; i < limit; i++) {
        dut->eval();
        bool acc = dut->w_awvalid && dut->w_awready;
        long at  = g_cycle;
        cyc();
        if (acc) { dut->w_awready = 0; return at; }
    }
    dut->w_awready = 0;
    return -1;
}

static long wide_accept_ar(int limit) {
    dut->w_arready = 1;
    for (int i = 0; i < limit; i++) {
        dut->eval();
        bool acc = dut->w_arvalid && dut->w_arready;
        long at  = g_cycle;
        cyc();
        if (acc) { dut->w_arready = 0; return at; }
    }
    dut->w_arready = 0;
    return -1;
}

// Accept one wide W beat, capturing wdata/wstrb.  Returns the cycle.
static long wide_accept_w(int limit, Word128* data_out, uint32_t* strb_out,
                          int* last_out) {
    dut->w_wready = 1;
    for (int i = 0; i < limit; i++) {
        dut->eval();
        bool acc = dut->w_wvalid && dut->w_wready;
        long at  = g_cycle;
        if (acc) {
            if (data_out) for (int k = 0; k < 4; k++) (*data_out)[k] = dut->w_wdata[k];
            if (strb_out) *strb_out = dut->w_wstrb;
            if (last_out) *last_out = dut->w_wlast;
        }
        cyc();
        if (acc) { dut->w_wready = 0; return at; }
    }
    dut->w_wready = 0;
    return -1;
}

// Present a wide B beat until the DUT takes it.
static long wide_send_b(int resp, int limit) {
    dut->w_bvalid = 1; dut->w_bresp = (uint8_t)resp;
    for (int i = 0; i < limit; i++) {
        dut->eval();
        bool acc = dut->w_bvalid && dut->w_bready;
        long at  = g_cycle;
        cyc();
        if (acc) { dut->w_bvalid = 0; return at; }
    }
    dut->w_bvalid = 0;
    return -1;
}

// Present a wide R beat until the DUT takes it.
static long wide_send_r(const Word128& d, int resp, bool last, int limit) {
    for (int k = 0; k < 4; k++) dut->w_rdata[k] = d[k];
    dut->w_rresp  = (uint8_t)resp;
    dut->w_rlast  = last ? 1 : 0;
    dut->w_rvalid = 1;
    for (int i = 0; i < limit; i++) {
        dut->eval();
        bool acc = dut->w_rvalid && dut->w_rready;
        long at  = g_cycle;
        cyc();
        if (acc) { dut->w_rvalid = 0; return at; }
    }
    dut->w_rvalid = 0;
    return -1;
}

// ── Write-side teardown snapshot ──────────────────────────────────────
struct AwState {
    int valid, accepted, beats_left, wrx_left, wswal, wg_valid, wraw_valid,
        drain, err;
    uint32_t cnt;
};
static AwState aw_state() {
    AwState s;
    s.valid      = dut->dbg_aw_valid_q;
    s.accepted   = dut->dbg_aw_accepted_q;
    s.beats_left = dut->dbg_aw_beats_left_q;
    s.wrx_left   = dut->dbg_aw_wrx_left_q;
    s.wswal      = dut->dbg_aw_wswal_q;
    s.wg_valid   = dut->dbg_wg_valid_q;
    s.wraw_valid = dut->dbg_wraw_valid_q;
    s.drain      = dut->dbg_aw_drain_q;
    s.err        = dut->dbg_aw_err_q;
    s.cnt        = dut->dbg_aw_timeout_cnt;
    return s;
}

// Cycle until the write watchdog fires (aw_err_q/aw_drain_q set), capturing
// the DUT state on the very cycle it becomes visible.  n_bready must be low
// on entry if the caller wants to inspect before the B is consumed.
static bool wait_aw_fire(int limit, AwState* snap, long* at) {
    for (int i = 0; i < limit; i++) {
        cyc();
        if (dut->dbg_aw_err_q || dut->dbg_aw_drain_q) {
            if (snap) *snap = aw_state();
            if (at)   *at   = g_cycle;
            return true;
        }
    }
    return false;
}

static bool wait_ar_fire(int limit, long* at) {
    for (int i = 0; i < limit; i++) {
        cyc();
        if (dut->dbg_ar_drain_q || dut->dbg_ar_err_left_q) {
            if (at) *at = g_cycle;
            return true;
        }
    }
    return false;
}

// Drive a healthy single-beat write to completion against a cooperative
// wide slave.  Returns the BRESP seen by the narrow master, or -1.
static int healthy_write(uint32_t addr, uint32_t data, int strb,
                         Word128* wide_data, uint32_t* wide_strb) {
    size_t b0 = b_log.size();
    dut->n_wdata = data; dut->n_wstrb = (uint8_t)strb; dut->n_wlast = 1;
    if (drive_aw(addr, 0, true) < 0) return -1;
    if (wide_accept_aw(16) < 0) return -1;
    int last = 0;
    if (wide_accept_w(16, wide_data, wide_strb, &last) < 0) return -1;
    if (wide_send_b(kRespOkay, 16) < 0) return -1;
    for (int i = 0; i < 16 && b_log.size() == b0; i++) cyc();
    if (b_log.size() == b0) return -1;
    return b_log.back().resp;
}

// Drive a healthy single-beat read to completion.  Returns the data, and
// reports the RRESP through *resp.
static bool healthy_read(uint32_t addr, const Word128& beat, uint32_t* data,
                         int* resp) {
    size_t r0 = r_log.size();
    if (drive_ar(addr, 0, 2) < 0) return false;
    if (wide_accept_ar(16) < 0) return false;
    if (wide_send_r(beat, kRespOkay, true, 16) < 0) return false;
    for (int i = 0; i < 16 && r_log.size() == r0; i++) cyc();
    if (r_log.size() == r0) return false;
    if (data) *data = r_log.back().data;
    if (resp) *resp = r_log.back().resp;
    return true;
}

#if N2W_WATCHDOG_ENABLED

// ══════════════════════════════════════════════════════════════════════
// Scenario 1 — AW expiry: SLVERR + state teardown
// ══════════════════════════════════════════════════════════════════════
static void scenario_1_aw_fires() {
    printf("\n── Scenario 1: a stalled write is abandoned with BRESP=SLVERR ──\n");
    reset();
    wide_stall_all();

    dut->n_wdata = 0xDEADBEEFu; dut->n_wstrb = 0xF; dut->n_wlast = 1;
    long aw_cyc = drive_aw(0xA000'0000u, 0, true);
    check("aw-fire: narrow AW accepted", aw_cyc >= 0, "AW never accepted");
    check("aw-fire: the first W beat went with it",
          dut->dbg_aw_wrx_left_q == 0,
          "aw_wrx_left_q=" + dec(dut->dbg_aw_wrx_left_q));
    check("aw-fire: the transaction is outstanding",
          dut->dbg_aw_valid_q == 1, "aw_valid_q low");
    check("aw-fire: it is presented to the (dead) wide side",
          dut->w_awvalid == 1, "w_awvalid low — nothing was even offered");

    // Nothing may complete: BREADY stays high so a spurious B would be
    // caught, but the wide side is wedged, so the ONLY thing that can move
    // is the watchdog.
    uint32_t cnt_before = dut->dbg_aw_timeout_cnt;
    run(4);
    check("aw-fire: the timeout counter is running",
          dut->dbg_aw_timeout_cnt > cnt_before,
          "counter stuck at " + dec(dut->dbg_aw_timeout_cnt));

    dut->n_bready = 0;                     // hold the B so state is inspectable
    AwState s; long fire_at = -1;
    bool fired = wait_aw_fire(3 * kTimeout, &s, &fire_at);
    check("aw-fire: the watchdog fires", fired,
          "no expiry within 3x TIMEOUT_CYCLES — the SoC would hang forever");

    if (fired) {
        printf("       expiry at cycle %ld, %ld cycles after the last progress "
               "(TIMEOUT_CYCLES=%d)\n", fire_at, fire_at - aw_cyc, kTimeout);
        check("aw-fire: counter reached TIMEOUT_CYCLES",
              g_aw_cnt_max == (uint32_t)kTimeout,
              "max counter " + dec(g_aw_cnt_max) + " != " + dec(kTimeout));
        // Full teardown — every piece of write state tied to the abandoned
        // transaction.  A stale wg_valid_q/wraw_valid_q here permanently
        // blocks n_wready, which is the original hang wearing a new hat.
        check("aw-fire: aw_valid_q cleared",      s.valid == 0);
        check("aw-fire: aw_accepted_q cleared",   s.accepted == 0);
        check("aw-fire: aw_beats_left_q cleared", s.beats_left == 0,
              "beats_left=" + dec(s.beats_left));
        check("aw-fire: aw_wrx_left_q cleared",   s.wrx_left == 0,
              "wrx_left=" + dec(s.wrx_left));
        check("aw-fire: wg_valid_q cleared",      s.wg_valid == 0);
        check("aw-fire: wraw_valid_q cleared",    s.wraw_valid == 0);
        check("aw-fire: aw_drain_q armed",        s.drain == 1);
        check("aw-fire: aw_err_q armed (a B is owed)", s.err == 1);
        check("aw-fire: nothing left owed on the W channel", s.wswal == 0,
              "wswal=" + dec(s.wswal));
        check("aw-fire: the wide side is no longer being driven",
              dut->w_awvalid == 0 && dut->w_wvalid == 0,
              "w_awvalid=" + dec(dut->w_awvalid) +
              " w_wvalid=" + dec(dut->w_wvalid));

        // BVALID must be up and HOLD until BREADY (AXI4).
        dut->eval();
        bool bvalid_up = dut->n_bvalid;
        run(5);
        dut->eval();
        check("aw-fire: BVALID rises and holds while BREADY is low",
              bvalid_up && dut->n_bvalid,
              "n_bvalid glitched low with BREADY deasserted");
        check("aw-fire: BRESP is SLVERR", dut->n_bresp == kRespSlverr,
              "n_bresp=" + dec(dut->n_bresp));

        size_t b0 = b_log.size();
        dut->n_bready = 1;
        run(4);
        check("aw-fire: exactly one B beat is delivered",
              b_log.size() == b0 + 1,
              dec((long)(b_log.size() - b0)) + " B beats");
        check("aw-fire: the master is answered, not left hanging",
              b_log.size() > b0 && b_log.back().resp == kRespSlverr,
              "resp=" + dec(b_log.empty() ? -1 : b_log.back().resp));
    }
    dut->n_bready = 1;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 2 — AR expiry: SLVERR with correct beat accounting
// ══════════════════════════════════════════════════════════════════════
static void scenario_2_ar_fires() {
    printf("\n── Scenario 2: a stalled read is abandoned with RRESP=SLVERR ──\n");

    // 2a — single beat, wide AR never even accepted.
    reset();
    wide_stall_all();
    long ar_cyc = drive_ar(0xB000'0000u, 0, 2);
    check("ar-fire(1beat): narrow AR accepted", ar_cyc >= 0);
    check("ar-fire(1beat): the transaction is outstanding",
          dut->dbg_ar_valid_q == 1);

    long fire_at = -1;
    bool fired = wait_ar_fire(3 * kTimeout, &fire_at);
    check("ar-fire(1beat): the watchdog fires", fired,
          "no expiry — the master would wait forever");
    if (fired) {
        printf("       expiry at cycle %ld, %ld cycles after the last progress\n",
               fire_at, fire_at - ar_cyc);
        check("ar-fire(1beat): ar_valid_q cleared", dut->dbg_ar_valid_q == 0);
        check("ar-fire(1beat): rg_valid_q cleared", dut->dbg_rg_valid_q == 0);
        check("ar-fire(1beat): ar_drain_q armed",   dut->dbg_ar_drain_q == 1);
        check("ar-fire(1beat): exactly one beat is owed",
              dut->dbg_ar_err_left_q == 1,
              "ar_err_left_q=" + dec(dut->dbg_ar_err_left_q));
        check("ar-fire(1beat): the wide side is no longer being driven",
              dut->w_arvalid == 0);
    }
    run(8);
    check("ar-fire(1beat): exactly one R beat is delivered",
          r_log.size() == 1, dec((long)r_log.size()) + " R beats");
    if (r_log.size() == 1) {
        check("ar-fire(1beat): RRESP=SLVERR", r_log[0].resp == kRespSlverr,
              "resp=" + dec(r_log[0].resp));
        check("ar-fire(1beat): RLAST set", r_log[0].last == 1);
        check("ar-fire(1beat): RDATA zeroed, not stale buffer contents",
              r_log[0].data == 0, hex32(r_log[0].data));
    }
    check("ar-fire(1beat): no phantom B beats", b_log.empty(),
          dec((long)b_log.size()) + " B beats");

    // 2b — the interesting one.  An 8-narrow-beat burst gets its first wide
    // beat (4 good narrow beats) and the wide side then dies.  The RTL's own
    // comment near ar_err_left_q warns that answering with a SINGLE SLVERR
    // beat would leave the master waiting for beats 6/7/8 — one hang traded
    // for another.  The master must see EXACTLY arlen+1 beats, RLAST on the
    // last and on no other.
    reset();
    wide_stall_all();
    const int kLen  = 7;                       // 8 narrow beats
    const int kGood = 4;                       // one wide beat's worth
    long ar2 = drive_ar(0xC000'0000u, kLen, 2);
    check("ar-fire(burst): narrow AR accepted", ar2 >= 0);
    check("ar-fire(burst): issued as a 2-beat 128-bit burst",
          dut->w_arlen == 1, "w_arlen=" + dec(dut->w_arlen));
    check("ar-fire(burst): wide AR accepted", wide_accept_ar(16) >= 0);

    const Word128 good = {0x1111'1111u, 0x2222'2222u, 0x3333'3333u, 0x4444'4444u};
    check("ar-fire(burst): first wide beat delivered",
          wide_send_r(good, kRespOkay, false, 16) >= 0);
    for (int i = 0; i < 16 && r_log.size() < (size_t)kGood; i++) cyc();
    check("ar-fire(burst): the first 4 narrow beats arrive normally",
          r_log.size() == (size_t)kGood,
          dec((long)r_log.size()) + " good beats");

    long fire2 = -1;
    bool fired2 = wait_ar_fire(3 * kTimeout, &fire2);
    check("ar-fire(burst): the watchdog fires on the stalled remainder", fired2);
    if (fired2)
        check("ar-fire(burst): the 4 undelivered beats are owed, not 1",
              dut->dbg_ar_err_left_q == kLen + 1 - kGood,
              "ar_err_left_q=" + dec(dut->dbg_ar_err_left_q) +
              " (a single beat here would hang the master)");
    run(2 * kTimeout + 16);

    check("ar-fire(burst): master receives exactly arlen+1 beats",
          r_log.size() == (size_t)(kLen + 1),
          dec((long)r_log.size()) + " of " + dec(kLen + 1));
    bool pre_ok = r_log.size() >= (size_t)kGood;
    for (int i = 0; i < kGood && pre_ok; i++)
        pre_ok = (r_log[i].resp == kRespOkay) && (r_log[i].last == 0) &&
                 (r_log[i].data == good[i]);
    check("ar-fire(burst): beats delivered before the expiry are untouched",
          pre_ok, "a pre-expiry beat changed resp/last/data");
    bool err_ok = r_log.size() == (size_t)(kLen + 1);
    for (int i = kGood; i <= kLen && err_ok; i++)
        err_ok = (r_log[i].resp == kRespSlverr) && (r_log[i].data == 0);
    check("ar-fire(burst): every remaining beat is SLVERR with zeroed data",
          err_ok);
    int n_last = 0, last_idx = -1;
    for (size_t i = 0; i < r_log.size(); i++)
        if (r_log[i].last) { n_last++; last_idx = (int)i; }
    check("ar-fire(burst): exactly one RLAST, on the final beat",
          n_last == 1 && last_idx == kLen,
          "n_last=" + dec(n_last) + " at idx " + dec(last_idx));
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 3 — no false trip while progress drips in
// ══════════════════════════════════════════════════════════════════════
static void scenario_3_no_false_trip() {
    printf("\n── Scenario 3: progress just inside the window must NOT trip it ──\n");
    const int kGap = kTimeout - 8;      // just under the limit, every time

    // 3a — write burst, one narrow W beat every kGap cycles.
    reset();
    wide_stall_all();
    dut->n_wdata = 0xE000'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 0;
    long aw = drive_aw(0xD000'0000u, 7, true);   // 8 narrow beats, lane 0
    check("no-trip(write): AW accepted", aw >= 0);
    check("no-trip(write): wide AW accepted", wide_accept_aw(16) >= 0);

    bool ok = true;
    for (int beat = 1; beat < 8 && ok; beat++) {
        run(kGap);
        // The gather buffer fills every 4 narrow beats; hand the assembled
        // wide beat over when it appears so the narrow side can keep going.
        dut->eval();
        if (dut->w_wvalid) ok = ok && (wide_accept_w(4, nullptr, nullptr, nullptr) >= 0);
        ok = ok && (drive_w(0xE000'0000u + beat, 0xF, beat == 7, 4 * kTimeout) >= 0);
    }
    check("no-trip(write): all 8 narrow beats handed over", ok);
    dut->eval();
    if (dut->w_wvalid) wide_accept_w(8, nullptr, nullptr, nullptr);
    run(2);
    bool b_taken = wide_send_b(kRespOkay, 16) >= 0;
    run(4);
    check("no-trip(write): B relayed to the master",
          b_taken && b_log.size() == 1,
          dec((long)b_log.size()) + " B beats");
    check("no-trip(write): BRESP=OKAY, not a synthesized SLVERR",
          !b_log.empty() && b_log.back().resp == kRespOkay,
          "resp=" + dec(b_log.empty() ? -1 : b_log.back().resp));
    check("no-trip(write): the watchdog never fired", !g_aw_fire_seen,
          "fired at cycle " + dec(g_aw_fire_cycle));
    check("no-trip(write): the counter never reached the limit",
          g_aw_cnt_max < (uint32_t)kTimeout,
          "max counter " + dec(g_aw_cnt_max) + " >= " + dec(kTimeout));
    printf("       peak write counter %u of %d (gap %d)\n",
           g_aw_cnt_max, kTimeout, kGap);

    // 3b — read burst, one narrow R beat taken every kGap cycles.
    reset();
    wide_stall_all();
    check("no-trip(read): AR accepted", drive_ar(0xD100'0000u, 7, 2) >= 0);
    check("no-trip(read): wide AR accepted", wide_accept_ar(16) >= 0);
    const Word128 w0 = {0xA1A1'0000u, 0xA1A1'0001u, 0xA1A1'0002u, 0xA1A1'0003u};
    const Word128 w1 = {0xA1A1'0004u, 0xA1A1'0005u, 0xA1A1'0006u, 0xA1A1'0007u};
    dut->n_rready = 0;
    bool rok = wide_send_r(w0, kRespOkay, false, 16) >= 0;
    for (int beat = 0; beat < 8 && rok; beat++) {
        run(kGap);
        if (beat == 4) rok = rok && (wide_send_r(w1, kRespOkay, true, 16) >= 0);
        // Take exactly one narrow beat.
        dut->n_rready = 1;
        for (int i = 0; i < 4 * kTimeout; i++) {
            dut->eval();
            bool acc = dut->n_rvalid && dut->n_rready;
            cyc();
            if (acc) break;
        }
        dut->n_rready = 0;
    }
    dut->n_rready = 1;
    run(4);
    check("no-trip(read): master receives all 8 beats",
          r_log.size() == 8, dec((long)r_log.size()) + " beats");
    bool all_okay = r_log.size() == 8;
    for (size_t i = 0; i < r_log.size() && all_okay; i++)
        all_okay = (r_log[i].resp == kRespOkay);
    check("no-trip(read): every beat is OKAY, none synthesized", all_okay);
    check("no-trip(read): the watchdog never fired", !g_ar_fire_seen,
          "fired at cycle " + dec(g_ar_fire_cycle));
    check("no-trip(read): the counter never reached the limit",
          g_ar_cnt_max < (uint32_t)kTimeout,
          "max counter " + dec(g_ar_cnt_max) + " >= " + dec(kTimeout));
    printf("       peak read counter %u of %d (gap %d)\n",
           g_ar_cnt_max, kTimeout, kGap);
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 4 — the exact boundary
// ══════════════════════════════════════════════════════════════════════
//
// The boundary has to be expressed in the frame that is physically
// meaningful to a bus master: CYCLES SINCE THE LAST PROGRESS EVENT.  (The
// alternative — keying off the observed value of aw_timeout_cnt — measures
// the harness against an internal counter whose phase is an implementation
// detail, and would "differ" between two functionally identical designs.)
//
// A write burst hands over W beat 0, goes quiet for `gap` cycles against a
// wedged wide side, then hands over W beat 1.  Does the watchdog fire in
// between?
//
// Observed behaviour, identical in both the registered and the
// pre-registration RTL: the transaction survives a gap of TIMEOUT_CYCLES+1
// and is abandoned at TIMEOUT_CYCLES+2.  Registering aw_progress shifts the
// counter and the observation by the same one cycle, so the fire/no-fire
// decision is unchanged; only the absolute expiry cycle moves by one.
static bool boundary_write_fires(int gap) {
    reset();
    wide_stall_all();
    dut->n_wdata = 0xF000'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 0;
    long p0 = drive_aw(0xE000'0000u, 1, true);   // 2 narrow beats, lane 0
    if (p0 < 0) return false;
    // No wide-side channel can move: AW is not accepted, the gather buffer
    // is half full so w_wvalid is low.  The only progress possible is the
    // master handing over its second W beat.
    while (g_cycle < p0 + gap) cyc();
    drive_w(0xF000'0001u, 0xF, true, 4 * kTimeout);
    run(8);
    return g_aw_fire_seen;
}

static bool boundary_read_fires(int gap) {
    reset();
    wide_stall_all();
    if (drive_ar(0xE100'0000u, 3, 2) < 0) return false;   // 4 narrow beats
    if (wide_accept_ar(16) < 0) return false;
    dut->n_rready = 0;
    const Word128 d = {0xB0B0'0000u, 0xB0B0'0001u, 0xB0B0'0002u, 0xB0B0'0003u};
    long p0 = wide_send_r(d, kRespOkay, true, 16);        // last progress
    if (p0 < 0) return false;
    // rg_valid_q is now set, so w_rready is low: no wide progress is
    // possible either.  Only the master raising RREADY can make progress.
    while (g_cycle < p0 + gap) cyc();
    dut->n_rready = 1;
    run(8);
    return g_ar_fire_seen;
}

static void scenario_4_boundary() {
    printf("\n── Scenario 4: the exact boundary cycle ──\n");
    struct { int gap; bool expect; } cases[] = {
        { kTimeout - 1, false },
        { kTimeout,     false },
        { kTimeout + 1, false },
        { kTimeout + 2, true  },
    };
    for (auto& c : cases) {
        bool got = boundary_write_fires(c.gap);
        check(("boundary(write): gap=TIMEOUT" +
               std::string(c.gap - kTimeout >= 0 ? "+" : "") +
               dec(c.gap - kTimeout) + " -> " +
               (c.expect ? "abandoned" : "survives")).c_str(),
              got == c.expect,
              got ? "fired unexpectedly" : "did not fire");
    }
    for (auto& c : cases) {
        bool got = boundary_read_fires(c.gap);
        check(("boundary(read): gap=TIMEOUT" +
               std::string(c.gap - kTimeout >= 0 ? "+" : "") +
               dec(c.gap - kTimeout) + " -> " +
               (c.expect ? "abandoned" : "survives")).c_str(),
              got == c.expect,
              got ? "fired unexpectedly" : "did not fire");
    }
    // Determinism: the same gap must give the same answer every time.
    bool a = boundary_write_fires(kTimeout + 1);
    bool b = boundary_write_fires(kTimeout + 1);
    bool c = boundary_write_fires(kTimeout + 2);
    bool d = boundary_write_fires(kTimeout + 2);
    check("boundary: the decision is deterministic across repeats",
          (a == b) && (c == d), "same gap gave different answers");
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 5 — clean rearm after a fire
// ══════════════════════════════════════════════════════════════════════
static void scenario_5_rearm() {
    printf("\n── Scenario 5: the mechanism rearms for the next transaction ──\n");

    // 5a — write, with the late stale B eventually arriving.  Recovery ends
    // as soon as the orphaned B is absorbed.
    reset();
    wide_stall_all();
    dut->n_wdata = 0x1234'5678u; dut->n_wstrb = 0xF; dut->n_wlast = 1;
    drive_aw(0x1000'0000u, 0, true);
    long f = -1; AwState s;
    check("rearm(write): first write abandoned",
          wait_aw_fire(3 * kTimeout, &s, &f));
    run(4);
    check("rearm(write): SLVERR delivered",
          b_log.size() == 1 && b_log[0].resp == kRespSlverr);
    check("rearm(write): AWREADY stays low while the stale B is outstanding",
          (dut->eval(), dut->n_awready == 0),
          "a new write could be accepted and answered with the OLD B");

    // The wedged slave finally answers the abandoned write.  That beat is
    // for a transaction the master has already been told failed; it must be
    // absorbed, never surfaced.
    size_t b_before = b_log.size();
    check("rearm(write): the orphaned wide B is absorbed",
          wide_send_b(kRespOkay, 16) >= 0);
    run(4);
    check("rearm(write): the stale B is NOT relayed to the master",
          b_log.size() == b_before,
          dec((long)(b_log.size() - b_before)) + " extra B beats");
    dut->eval();
    check("rearm(write): AWREADY returns once recovery completes",
          dut->n_awready == 1, "still low");

    Word128 wd = {0, 0, 0, 0}; uint32_t ws = 0;
    // 0x...04 -> addr[3:2] == 1, so the word must land in 128-bit lane 1.
    int resp = healthy_write(0x1000'0004u, 0xCAFE'BABEu, 0xF, &wd, &ws);
    check("rearm(write): the next write completes with OKAY",
          resp == kRespOkay, "resp=" + dec(resp));
    check("rearm(write): its data reaches the wide side in the right lane",
          wd[1] == 0xCAFE'BABEu && ws == 0x00F0u,
          "wdata[1]=" + hex32(wd[1]) + " wstrb=" + hex32(ws));

    // 5b — write, with the wide side staying dead.  Recovery is bounded by
    // one further TIMEOUT_CYCLES, so the adapter rearms without any help.
    reset();
    wide_stall_all();
    dut->n_wdata = 0xAAAA'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 1;
    drive_aw(0x2000'0000u, 0, true);
    check("rearm(write,dead): abandoned", wait_aw_fire(3 * kTimeout, &s, &f));
    run(4);
    dut->eval();
    check("rearm(write,dead): AWREADY held low through the drain window",
          dut->n_awready == 0);
    run(2 * kTimeout);
    dut->eval();
    check("rearm(write,dead): recovery times out on its own and AWREADY returns",
          dut->n_awready == 1,
          "still low 2x TIMEOUT_CYCLES after the expiry — permanently wedged");
    resp = healthy_write(0x2000'0010u, 0x0BAD'F00Du, 0xF, &wd, &ws);
    check("rearm(write,dead): the next write completes with OKAY",
          resp == kRespOkay, "resp=" + dec(resp));

    // 5c — read: abandon, absorb the late stale burst, then read again.
    reset();
    wide_stall_all();
    drive_ar(0x3000'0000u, 0, 2);
    check("rearm(read): first read abandoned", wait_ar_fire(3 * kTimeout, &f));
    run(4);
    check("rearm(read): one SLVERR beat delivered",
          r_log.size() == 1 && r_log[0].resp == kRespSlverr,
          dec((long)r_log.size()) + " beats");
    dut->eval();
    check("rearm(read): ARREADY stays low while the stale R is outstanding",
          dut->n_arready == 0);
    size_t r_before = r_log.size();
    const Word128 stale = {0xDEAD'DEADu, 0, 0, 0};
    check("rearm(read): the orphaned wide R is absorbed",
          wide_send_r(stale, kRespOkay, true, 16) >= 0);
    run(4);
    check("rearm(read): the stale beat is NOT relayed to the master",
          r_log.size() == r_before,
          dec((long)(r_log.size() - r_before)) + " extra R beats");
    dut->eval();
    check("rearm(read): ARREADY returns once recovery completes",
          dut->n_arready == 1, "still low");
    uint32_t got = 0; int rresp = -1;
    const Word128 fresh = {0xFEED'FACEu, 0, 0, 0};
    bool ok = healthy_read(0x3000'0010u, fresh, &got, &rresp);
    check("rearm(read): the next read completes", ok);
    check("rearm(read): with correct data and OKAY, not the stale beat",
          got == 0xFEED'FACEu && rresp == kRespOkay,
          "data=" + hex32(got) + " resp=" + dec(rresp));

    // 5d — back-to-back expiries.  The watchdog itself must rearm, not just
    // the datapath: a second dead transaction must also be abandoned.
    reset();
    wide_stall_all();
    dut->n_wdata = 0x5555'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 1;
    drive_aw(0x4000'0000u, 0, true);
    check("rearm(b2b): first write abandoned", wait_aw_fire(3 * kTimeout, &s, &f));
    run(4);
    wide_send_b(kRespOkay, 16);            // clear the drain quickly
    run(4);
    obs_clear();
    dut->n_wdata = 0x6666'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 1;
    long aw2 = drive_aw(0x4000'0020u, 0, true);
    check("rearm(b2b): a second write is accepted", aw2 >= 0,
          "n_awready never came back");
    long f2 = -1;
    check("rearm(b2b): and is also abandoned — the watchdog rearmed",
          wait_aw_fire(3 * kTimeout, &s, &f2));
    run(4);
    check("rearm(b2b): the second write also gets exactly one SLVERR",
          b_log.size() == 1 && b_log[0].resp == kRespSlverr,
          dec((long)b_log.size()) + " B beats");
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 7 — ADVERSARIAL: progress landing on the expiry cycle
// ══════════════════════════════════════════════════════════════════════
//
// The expiry cycle and a live handshake can coincide.  Whether they CAN is
// itself implementation-dependent (a combinational progress term makes the
// coincidence impossible; a registered one makes it reachable), so the
// checks here are end-to-end AXI invariants that must hold either way:
//
//   * a read must deliver exactly ARLEN+1 beats, one RLAST, ever;
//   * a write whose master has handed over every beat it owes must get its
//     B without waiting another full timeout window.
//
// Both are driven at gap = TIMEOUT_CYCLES+2, the first gap at which the
// transaction is abandoned (scenario 4), i.e. the cycle on which the master
// resumes is exactly the cycle the watchdog gives up.
static void scenario_7_coincident_progress() {
    printf("\n── Scenario 7: a live handshake on the exact expiry cycle ──\n");
    const int kGap = kTimeout + 2;

    // 7a — READ.  A 4-beat burst has one wide beat buffered and the master
    // has taken none of it.  RREADY comes back exactly as the watchdog
    // expires.  If the beat handed over on that cycle is not accounted for
    // when ar_err_left_q is seeded, the master gets ARLEN+2 beats: a
    // protocol violation that desyncs its read channel permanently — the
    // exact class of hang this watchdog exists to prevent.
    reset();
    wide_stall_all();
    const int kLen = 3;                       // 4 narrow beats
    check("coincident(read): AR accepted", drive_ar(0xE200'0000u, kLen, 2) >= 0);
    check("coincident(read): wide AR accepted", wide_accept_ar(16) >= 0);
    dut->n_rready = 0;
    const Word128 d = {0xC0C0'0000u, 0xC0C0'0001u, 0xC0C0'0002u, 0xC0C0'0003u};
    long p0 = wide_send_r(d, kRespOkay, true, 16);
    check("coincident(read): wide beat buffered", p0 >= 0);
    while (g_cycle < p0 + kGap) cyc();
    dut->n_rready = 1;
    // Snapshot how many SLVERR beats the expiry decides it still owes.  If a
    // beat handed over on the expiry cycle itself is not netted off, this is
    // one too many and the master ends up with ARLEN+2 beats.
    long fire = -1;
    bool fired = wait_ar_fire(3 * kTimeout, &fire);
    int owed_at_fire = dut->dbg_ar_err_left_q;
    size_t delivered_at_fire = r_log.size();
    run(3 * kTimeout + 16);

    check("coincident(read): the read is abandoned as expected", fired,
          "the setup did not reach the expiry — check the gap");
    check("coincident(read): the beats owed at the expiry net off the beat "
          "handed over on that same cycle",
          owed_at_fire + (int)delivered_at_fire == kLen + 1,
          "ar_err_left_q=" + dec(owed_at_fire) + " with " +
          dec((long)delivered_at_fire) + " beat(s) already delivered = " +
          dec(owed_at_fire + (long)delivered_at_fire) + ", not " + dec(kLen + 1));
    check("coincident(read): master receives EXACTLY arlen+1 beats",
          r_log.size() == (size_t)(kLen + 1),
          dec((long)r_log.size()) + " beats for a " + dec(kLen + 1) +
          "-beat read — an over-count desyncs the master's R channel forever");
    int n_last = 0, last_idx = -1;
    for (size_t i = 0; i < r_log.size(); i++)
        if (r_log[i].last) { n_last++; last_idx = (int)i; }
    check("coincident(read): exactly one RLAST, on the final beat",
          n_last == 1 && last_idx == (int)r_log.size() - 1,
          "n_last=" + dec(n_last) + " at idx " + dec(last_idx));

    // 7b — WRITE.  A 2-beat burst; the master hands its FINAL beat (n_wlast)
    // on the expiry cycle.  At that point it owes nothing, so its SLVERR B
    // must follow promptly.  If the swallow counter is seeded from the
    // pre-handshake value it waits for a beat that will never come, and the
    // B is held back for another entire timeout window (20 s in the shipping
    // configuration).
    reset();
    wide_stall_all();
    dut->n_wdata = 0xF100'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 0;
    long w0 = drive_aw(0xE300'0000u, 1, true);
    check("coincident(write): AW + first W beat accepted", w0 >= 0);
    while (g_cycle < w0 + kGap) cyc();
    long wlast_at = drive_w(0xF100'0001u, 0xF, true, 4 * kTimeout);
    check("coincident(write): the final W beat is accepted", wlast_at >= 0);
    run(2);
    check("coincident(write): the write is abandoned as expected",
          g_aw_fire_seen, "the setup did not reach the expiry");
    // The master has now handed over every beat of its burst, so the
    // accept-and-discard counter must be seeded with nothing left to swallow.
    // Seeding it from the pre-handshake value leaves it waiting for a beat
    // that will never arrive, and n_bvalid is gated on it draining.
    check("coincident(write): nothing is left to swallow — the master already "
          "sent everything",
          dut->dbg_aw_wswal_q == 0,
          "aw_wswal_q=" + dec(dut->dbg_aw_wswal_q) + " after the master's final "
          "W beat; n_bvalid is gated on this reaching 0");

    const int kBBudget = 8;
    for (int i = 0; i < kBBudget && b_log.empty(); i++) cyc();
    bool prompt = !b_log.empty();
    if (!prompt) { run(3 * kTimeout); }
    check("coincident(write): the SLVERR B follows the last W beat promptly",
          prompt,
          b_log.empty()
              ? "no B at all"
              : "B arrived " + dec(b_log[0].cyc - wlast_at) + " cycles after the "
                "master's final W beat (budget " + dec(kBBudget) + ") — the "
                "master has handed over everything it owes, so nothing should "
                "be waiting");
    check("coincident(write): exactly one B, SLVERR",
          b_log.size() == 1 && b_log[0].resp == kRespSlverr,
          dec((long)b_log.size()) + " B beats");
}

#else  // !N2W_WATCHDOG_ENABLED

// ══════════════════════════════════════════════════════════════════════
// Scenario 6 — ENABLE_ABANDON_TIMEOUT = 0: hang cleanly, forever
// ══════════════════════════════════════════════════════════════════════
//
// With the watchdog compiled out, a wedged wide side hangs the narrow
// master.  That is the accepted trade.  What must NOT happen is a spurious
// SLVERR, a counter quietly running, corrupted state, or a transaction that
// cannot complete once the slave finally answers.  This pins that contract
// so nobody later assumes the timeout is still there.
static void scenario_6_disabled() {
    printf("\n── Scenario 6: with the watchdog disabled, hang cleanly ──\n");
    const int kLong = 4 * kTimeout;

    // 6a — write.
    reset();
    wide_stall_all();
    dut->n_wdata = 0x900D'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 1;
    check("disabled(write): AW accepted", drive_aw(0x7000'0000u, 0, true) >= 0);
    run(kLong);
    check("disabled(write): the timeout counter never moves",
          g_aw_cnt_max == 0, "aw_timeout_cnt reached " + dec(g_aw_cnt_max));
    check("disabled(write): the watchdog never fires", !g_aw_fire_seen,
          "aw_err_q/aw_drain_q asserted at cycle " + dec(g_aw_fire_cycle));
    check("disabled(write): no B beat is synthesized", b_log.empty(),
          dec((long)b_log.size()) + " spurious B beats");
    check("disabled(write): nothing is owed on the W channel",
          dut->dbg_aw_wswal_q == 0,
          "aw_wswal_q=" + dec(dut->dbg_aw_wswal_q));
    check("disabled(write): the transaction is still outstanding",
          dut->dbg_aw_valid_q == 1 && dut->dbg_wg_valid_q == 1,
          "state was torn down without a watchdog to do it");
    dut->eval();
    check("disabled(write): still presented to the fabric — a stuck "
          "transaction, visible over JTAG",
          dut->w_awvalid == 1 && dut->w_wvalid == 1,
          "w_awvalid=" + dec(dut->w_awvalid) + " w_wvalid=" + dec(dut->w_wvalid));
    check("disabled(write): AWREADY stays low (the master is blocked, as "
          "designed)", dut->n_awready == 0);

    // The slave finally wakes up, far past where the watchdog would have
    // given up.  No state was corrupted, so the write must simply complete.
    Word128 wd = {0, 0, 0, 0}; uint32_t ws = 0; int wlast = 0;
    check("disabled(write): the very late wide AW is still accepted",
          wide_accept_aw(16) >= 0);
    check("disabled(write): the very late wide W is still accepted",
          wide_accept_w(16, &wd, &ws, &wlast) >= 0);
    check("disabled(write): the buffered data survived the stall intact",
          wd[0] == 0x900D'0000u && ws == 0x000Fu && wlast == 1,
          "wdata[0]=" + hex32(wd[0]) + " wstrb=" + hex32(ws));
    check("disabled(write): B relayed", wide_send_b(kRespOkay, 16) >= 0);
    run(4);
    check("disabled(write): exactly one B, OKAY — not SLVERR",
          b_log.size() == 1 && b_log[0].resp == kRespOkay,
          dec((long)b_log.size()) + " B beats, resp=" +
          dec(b_log.empty() ? -1 : b_log[0].resp));
    dut->eval();
    check("disabled(write): AWREADY returns after the late completion",
          dut->n_awready == 1);

    // 6b — read.
    reset();
    wide_stall_all();
    check("disabled(read): AR accepted", drive_ar(0x7100'0000u, 0, 2) >= 0);
    run(kLong);
    check("disabled(read): the timeout counter never moves",
          g_ar_cnt_max == 0, "ar_timeout_cnt reached " + dec(g_ar_cnt_max));
    check("disabled(read): the watchdog never fires", !g_ar_fire_seen,
          "ar_drain_q/ar_err_left_q asserted at cycle " + dec(g_ar_fire_cycle));
    check("disabled(read): no R beat is synthesized", r_log.empty(),
          dec((long)r_log.size()) + " spurious R beats");
    check("disabled(read): the transaction is still outstanding",
          dut->dbg_ar_valid_q == 1 && dut->dbg_ar_err_left_q == 0);
    dut->eval();
    check("disabled(read): still presented to the fabric",
          dut->w_arvalid == 1);
    // MULTI-OUTSTANDING (2026-08-20).  This used to read "ARREADY stays
    // low", which was a restatement of the adapter holding ONE read at a
    // time — the property the multi-outstanding rework removed on
    // purpose.  With RD_OUTSTANDING credits the port survives one stalled
    // read, and what is worth pinning here is that the DISABLED watchdog
    // changes nothing about that: the stalled read is still outstanding,
    // still presented to the fabric, still unanswered, and the port is
    // still usable for the reads behind it.  The credit limit itself is
    // pinned separately below.
    check("disabled(read): ARREADY survives one stalled read", dut->n_arready == 1);

    // The credit limit is the real backpressure now.  Fill it and prove
    // ARREADY goes low — and that doing so still synthesises nothing and
    // never trips a compiled-out watchdog.
    {
        int extra = 0;
        while (dut->n_arready && extra < 64) {
            if (drive_ar(0x7180'0000u + 0x100u * (unsigned)extra, 0, 2) < 0) break;
            extra++;
            dut->eval();
        }
        check("disabled(read): the port backpressures once the credits run out",
              dut->n_arready == 0 && extra >= 1,
              "accepted " + dec(extra) + " further reads, n_arready=" +
              dec(dut->n_arready));
        run(kLong);
        check("disabled(read): a full queue still synthesises no response",
              r_log.empty() && !g_ar_fire_seen && g_ar_cnt_max == 0,
              dec((long)r_log.size()) + " spurious R beats, fire=" +
              dec(g_ar_fire_seen) + " cnt=" + dec(g_ar_cnt_max));
        // Put the DUT back to exactly one outstanding read so the late
        // completion checks below still describe what they say they do.
        reset();
        wide_stall_all();
        check("disabled(read): AR re-accepted after requeue",
              drive_ar(0x7100'0000u, 0, 2) >= 0);
        run(kLong);
    }

    check("disabled(read): the very late wide AR is still accepted",
          wide_accept_ar(16) >= 0);
    const Word128 late = {0x600D'0BADu, 0, 0, 0};
    check("disabled(read): the very late wide R is still accepted",
          wide_send_r(late, kRespOkay, true, 16) >= 0);
    run(8);
    check("disabled(read): exactly one R beat, OKAY with the real data",
          r_log.size() == 1 && r_log[0].resp == kRespOkay &&
          r_log[0].data == 0x600D'0BADu && r_log[0].last == 1,
          dec((long)r_log.size()) + " beats, data=" +
          hex32(r_log.empty() ? 0 : r_log[0].data));
    dut->eval();
    check("disabled(read): ARREADY returns after the late completion",
          dut->n_arready == 1);

    // 6c — a burst, to prove no partial-teardown path exists either.
    reset();
    wide_stall_all();
    dut->n_wdata = 0xB0B0'0000u; dut->n_wstrb = 0xF; dut->n_wlast = 0;
    check("disabled(burst): AW accepted", drive_aw(0x7200'0000u, 3, true) >= 0);
    run(kLong);
    check("disabled(burst): no counter, no fire, no synthesized response",
          g_aw_cnt_max == 0 && !g_aw_fire_seen && b_log.empty() &&
          dut->dbg_aw_err_q == 0,
          "cnt=" + dec(g_aw_cnt_max) + " fire=" + dec(g_aw_fire_seen) +
          " b=" + dec((long)b_log.size()));
    check("disabled(burst): the beat accounting is untouched",
          dut->dbg_aw_beats_left_q == 3 && dut->dbg_aw_wrx_left_q == 3,
          "beats_left=" + dec(dut->dbg_aw_beats_left_q) +
          " wrx_left=" + dec(dut->dbg_aw_wrx_left_q));
}

#endif  // N2W_WATCHDOG_ENABLED

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_n2w_watchdog;

#if N2W_WATCHDOG_ENABLED
    printf("tb_n2w_watchdog: axi_narrow_to_wide abandonment watchdog "
           "(ENABLE_ABANDON_TIMEOUT=1, TIMEOUT_CYCLES=%d)\n", kTimeout);
    scenario_1_aw_fires();
    scenario_2_ar_fires();
    scenario_3_no_false_trip();
    scenario_4_boundary();
    scenario_5_rearm();
    scenario_7_coincident_progress();
#else
    printf("tb_n2w_watchdog: axi_narrow_to_wide with the abandonment watchdog "
           "DISABLED (ENABLE_ABANDON_TIMEOUT=0)\n");
    scenario_6_disabled();
#endif

    dut->final();
    delete dut;

    printf("\n%d checks: %d passed, %d failed\n", n_pass + n_fail, n_pass, n_fail);
    if (n_fail) { printf("FAILED\n"); return 1; }
    printf("All %d checks PASSED.\n", n_pass);
    return 0;
}
