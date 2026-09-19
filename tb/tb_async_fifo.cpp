// tb_async_fifo.cpp — Verilator unit testbench for rtl/sys/async_fifo.v
//
// 21 scenarios — Gray-coded async FIFO across two independent clocks:
//
//   1.  reset_clears_state              — after reset both sides are idle,
//                                         empty asserted, full deasserted.
//   2.  single_write_read               — push one word, wait for sync,
//                                         pop it back identically.
//   3.  fill_to_full                    — push DEPTH words; wr_full asserts.
//   4.  drain_to_empty                  — pop all; rd_empty asserts again.
//   5.  simultaneous_rw_steady          — overlapping push + pop at 1:1
//                                         ratio keeps data ordered.
//   6.  wraparound                      — push-pop-push cycles past the
//                                         physical pointer wrap.
//   7.  watermarks                      — wr_almost_full / rd_almost_empty
//                                         toggle correctly near thresholds.
//   8.  slow_wclk_fast_rclk             — writer at 1:5, reader at 1:1;
//                                         reader stalls on empty, still
//                                         gets exact data order.
//   9.  fast_wclk_slow_rclk             — reverse ratio; writer fills up
//                                         to full, reader drains slowly.
//   10. full_then_drain_then_push       — full, then partial drain, then
//                                         push again; no data corruption.
//   11. empty_then_push_then_pop        — from idle empty, write one,
//                                         read one, idle again.
//   12. one_sided_mside_reset_mid_stream — assert ONLY rrst (simulating
//                                         a MIG UI recalibration) while
//                                         traffic is in flight, release,
//                                         verify the FIFO settles to a
//                                         consistent empty state and
//                                         subsequent traffic is FIFO-
//                                         correct.  Regression test for
//                                         the cross-coupled reset fix
//                                         (T6): without coupling, the
//                                         write pointer keeps counting
//                                         against a zeroed read pointer
//                                         and the occupancy/full/empty
//                                         flags desync.
//   13-14. one_sided_mside_reset_ratio_wr_slow/wr_fast — #12 re-run at
//                                         a 1:5 clock ratio, both
//                                         orientations (deployment shape
//                                         is ~50 MHz vs ~333 MHz),
//                                         exercising the request/ack
//                                         reset handshake's convergence
//                                         under asymmetric clock rates.
//   15. one_sided_mside_reset_ratio_wr_slow_sustained — #13 extended to
//                                         keep the 1:5 ratio through the
//                                         POST-RELEASE data phase too
//                                         (not just reset-assert/settle),
//                                         for resumed-sampling-mid-stream
//                                         coverage.
//   16. one_sided_sside_reset_mid_stream — same as #12 but asserting
//                                         ONLY wrst (simulating a PCIe
//                                         link retrain poisoning the
//                                         debug master's upstream side).
//   17-18. one_sided_sside_reset_ratio_wr_slow/wr_fast — same as
//                                         #13-14, for #16.
//   19. one_sided_mside_reset_back_to_back — T6 re-review Finding 3:
//                                         two back-to-back rrst-only
//                                         pulses separated by 1, 2, and
//                                         3 idle cycles (swept within
//                                         the scenario), verifying the
//                                         second reset settles cleanly
//                                         and isn't corrupted by a stale
//                                         ack left over from the first.
//   20. one_sided_sside_reset_back_to_back — mirror of #19 for wrst-only.
//   21. phase_swept_opposite_side_back_to_back_ratio — T6 final review
//                                         Finding (e): event A = rrst
//                                         (R-side) run to completion at
//                                         a 1:5 (wclk fast / rclk slow)
//                                         ratio, then event B = a SHORT
//                                         (single wclk cycle) wrst-only
//                                         pulse whose start offset,
//                                         relative to an empirically-
//                                         probed wr_full 1->0 transition
//                                         marking event A's dying tail,
//                                         is swept across 8 deltas
//                                         straddling that boundary.  A
//                                         push lands in the inter-reset
//                                         gap.  Verifies engagement for
//                                         event B always occurs
//                                         (indirectly: a missed
//                                         engagement would leave req_w,
//                                         and therefore wr_full, stuck
//                                         forever) and that post-
//                                         recovery traffic is FIFO-
//                                         correct.  See the REQ_MIN_HOLD
//                                         comment in async_fifo.v.
//
// Build via: make tb-async-fifo

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <vector>
#include <verilated.h>
#include "Vasync_fifo.h"

static Vasync_fifo* dut    = nullptr;
static int          n_pass = 0, n_fail = 0;

static constexpr int DEPTH_LOG2 = 4;
static constexpr int DEPTH      = 1 << DEPTH_LOG2;

// ─── Clock drivers — two independent virtual clocks ──────────────────────
// We model two clocks by time-step counting.  In each sim step we may
// rise clk_w and/or clk_r.  Between steps eval() is called so that both
// combinational logic + FF updates resolve consistently.
//
// For same-rate tests, posedges of wclk and rclk happen in the SAME sim
// step (with wclk's eval occurring first, then rclk's).  For different
// rates we interleave according to the ratio requested.

static void eval_once() { dut->eval(); }

// Rise, then fall, a single clock, evaluating after each.
static void pulse_wclk() {
    dut->wclk = 1; eval_once();
    dut->wclk = 0; eval_once();
}
static void pulse_rclk() {
    dut->rclk = 1; eval_once();
    dut->rclk = 0; eval_once();
}

// One "unit tick" where BOTH clocks rise then fall.  Use for same-rate.
static void tick_both() {
    // Rise both, eval, then fall both, eval — sequentially so that
    // each posedge is treated as its own event.
    dut->wclk = 1; dut->rclk = 1; eval_once();
    dut->wclk = 0; dut->rclk = 0; eval_once();
}

// Ticks where wclk runs 1×, rclk runs n× (fast reader).
static void tick_wr_slow(int rratio) {
    dut->wclk = 1; eval_once();
    dut->wclk = 0; eval_once();
    for (int i = 0; i < rratio; i++) {
        dut->rclk = 1; eval_once();
        dut->rclk = 0; eval_once();
    }
}
// Ticks where wclk runs n×, rclk runs 1× (fast writer).
static void tick_rr_slow(int wratio) {
    for (int i = 0; i < wratio; i++) {
        dut->wclk = 1; eval_once();
        dut->wclk = 0; eval_once();
    }
    dut->rclk = 1; eval_once();
    dut->rclk = 0; eval_once();
}

// Full reset — assert both sides, tick a handful, deassert.
// Post-deassert settle must clear the RTL's REQ_MIN_HOLD (async_fifo.v)
// saturating counter: req_w/req_r stay asserted for AT LEAST
// REQ_MIN_HOLD=8 local cycles after latching, regardless of ack, once
// the raw wrst/rrst pulse itself has ended.  6 cycles (the pre-T6-
// review-round-3 value) is no longer enough — it left req_w/req_r (and
// therefore w_eng/r_eng, and therefore wr_full/rd_empty) still forced
// busy when reset_both() returned, silently dropping every test's
// priming pushes.  16 gives 2x margin over the 8-cycle minimum plus
// room for the ack round trip.
static void reset_both() {
    dut->wclk = 0; dut->rclk = 0;
    dut->wrst = 1; dut->rrst = 1;
    dut->wr_en = 0; dut->rd_en = 0; dut->wr_data = 0;
    for (int i = 0; i < 6; i++) tick_both();
    dut->wrst = 0; dut->rrst = 0;
    for (int i = 0; i < 16; i++) tick_both();
}

#define CHECK(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s: condition `%s` failed\n", name, #cond); \
        return false; \
    } \
} while (0)

#define CHECK_EQ(name, got, exp) do { \
    if ((uint32_t)(got) != (uint32_t)(exp)) { \
        printf("  FAIL %s: got 0x%x expected 0x%x\n", \
               name, (uint32_t)(got), (uint32_t)(exp)); \
        return false; \
    } \
} while (0)

// ══════════════════════════════════════════════════════════════════════
// 1. reset_clears_state
// ══════════════════════════════════════════════════════════════════════
static bool test_reset_clears_state() {
    reset_both();
    CHECK("rd_empty after reset", dut->rd_empty == 1);
    CHECK("wr_full low after reset", dut->wr_full == 0);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 2. single_write_read
// ══════════════════════════════════════════════════════════════════════
static bool test_single_write_read() {
    reset_both();
    // push one
    dut->wr_data = 0xCAFEBABE;
    dut->wr_en   = 1;
    tick_both();
    dut->wr_en   = 0;
    // let the rclk side see it
    for (int i = 0; i < 8; i++) tick_both();
    CHECK("not empty after push+sync", dut->rd_empty == 0);
    uint32_t got = dut->rd_data;
    CHECK_EQ("rd_data matches", got, 0xCAFEBABE);
    dut->rd_en = 1;
    tick_both();
    dut->rd_en = 0;
    for (int i = 0; i < 4; i++) tick_both();
    CHECK("empty after pop", dut->rd_empty == 1);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 3. fill_to_full
// ══════════════════════════════════════════════════════════════════════
static bool test_fill_to_full() {
    reset_both();
    int pushed = 0;
    dut->wr_en = 1;
    for (int i = 0; i < DEPTH + 4; i++) {
        dut->wr_data = 0x1000 + i;
        // Sample wr_full BEFORE this tick — if full at posedge it's a no-op.
        if (!dut->wr_full) pushed++;
        tick_both();
    }
    dut->wr_en = 0;
    // wr_full should have asserted by now.
    CHECK("wr_full asserted near end of fill", dut->wr_full == 1);
    CHECK("pushed >= DEPTH-1 and <= DEPTH",
          pushed >= DEPTH - 1 && pushed <= DEPTH);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 4. drain_to_empty
// ══════════════════════════════════════════════════════════════════════
static bool test_drain_to_empty() {
    reset_both();
    // fill
    dut->wr_en = 1;
    for (int i = 0; i < DEPTH; i++) {
        dut->wr_data = 0xA000 + i;
        tick_both();
        if (dut->wr_full) break;
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();
    // drain
    int popped = 0;
    dut->rd_en = 1;
    for (int i = 0; i < DEPTH + 10; i++) {
        if (!dut->rd_empty) popped++;
        tick_both();
    }
    dut->rd_en = 0;
    for (int i = 0; i < 6; i++) tick_both();
    CHECK("rd_empty after drain", dut->rd_empty == 1);
    CHECK("popped at least DEPTH-1", popped >= DEPTH - 1);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 5. simultaneous_rw_steady
// ══════════════════════════════════════════════════════════════════════
static bool test_simultaneous_rw_steady() {
    reset_both();
    // Prime the FIFO with 4 words so reads can start immediately.
    dut->wr_en = 1;
    for (int i = 0; i < 4; i++) {
        dut->wr_data = 0xB000 + i;
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 4; i++) tick_both();

    std::deque<uint32_t> expected;
    for (int i = 0; i < 4; i++) expected.push_back(0xB000 + i);
    // Simultaneous push + pop at 1:1 ratio for 40 cycles.
    int next_wr = 4;
    for (int cyc = 0; cyc < 40; cyc++) {
        dut->wr_en = !dut->wr_full ? 1 : 0;
        if (dut->wr_en) {
            dut->wr_data = 0xB000 + next_wr;
            expected.push_back(0xB000 + next_wr);
            next_wr++;
        }
        dut->rd_en = !dut->rd_empty ? 1 : 0;
        uint32_t want = 0;
        bool checking = dut->rd_en == 1;
        if (checking) {
            want = dut->rd_data;
        }
        tick_both();
        if (checking) {
            CHECK("expected non-empty on pop", !expected.empty());
            uint32_t exp = expected.front(); expected.pop_front();
            CHECK_EQ("fifo order", want, exp);
        }
    }
    dut->wr_en = 0; dut->rd_en = 0;
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 6. wraparound — push DEPTH, pop DEPTH/2, push DEPTH/2, drain all.
// ══════════════════════════════════════════════════════════════════════
static bool test_wraparound() {
    reset_both();
    std::deque<uint32_t> expected;

    dut->wr_en = 1;
    for (int i = 0; i < DEPTH - 1; i++) {
        if (!dut->wr_full) {
            dut->wr_data = 0xC000 + i;
            expected.push_back(0xC000 + i);
        }
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < DEPTH/2; i++) {
        if (!dut->rd_empty) {
            CHECK_EQ("wrap-part1 pop", dut->rd_data, expected.front());
            expected.pop_front();
        }
        tick_both();
    }
    dut->rd_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    dut->wr_en = 1;
    for (int i = 0; i < DEPTH/2; i++) {
        if (!dut->wr_full) {
            dut->wr_data = 0xD000 + i;
            expected.push_back(0xD000 + i);
        }
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < DEPTH + 10; i++) {
        if (!dut->rd_empty) {
            CHECK_EQ("wrap-drain pop", dut->rd_data, expected.front());
            expected.pop_front();
        }
        tick_both();
    }
    dut->rd_en = 0;
    CHECK("all consumed", expected.empty());
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 7. watermarks
// ══════════════════════════════════════════════════════════════════════
static bool test_watermarks() {
    reset_both();
    // Initially empty → almost_empty asserted.
    for (int i = 0; i < 6; i++) tick_both();
    CHECK("almost_empty high at idle", dut->rd_almost_empty == 1);
    CHECK("almost_full low at idle", dut->wr_almost_full == 0);

    // Fill until almost_full.
    dut->wr_en = 1;
    bool saw_almost_full = false;
    for (int i = 0; i < DEPTH + 4; i++) {
        if (!dut->wr_full) {
            dut->wr_data = 0x2000 + i;
        }
        tick_both();
        if (dut->wr_almost_full) saw_almost_full = true;
    }
    dut->wr_en = 0;
    CHECK("wr_almost_full asserted during fill", saw_almost_full);

    // Drain; after enough rclk ticks, rd_almost_empty should come back.
    for (int i = 0; i < 6; i++) tick_both();
    dut->rd_en = 1;
    for (int i = 0; i < DEPTH + 10; i++) tick_both();
    dut->rd_en = 0;
    for (int i = 0; i < 6; i++) tick_both();
    CHECK("rd_almost_empty asserted after drain", dut->rd_almost_empty == 1);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 8. slow_wclk_fast_rclk — writer 1×, reader 5×.
// ══════════════════════════════════════════════════════════════════════
static bool test_slow_wclk_fast_rclk() {
    reset_both();
    std::deque<uint32_t> expected;
    dut->wr_en = 0; dut->rd_en = 0;

    // Each "outer" iteration: 1 wclk pulse + 5 rclk pulses.
    for (int i = 0; i < 30; i++) {
        dut->wr_en = !dut->wr_full ? 1 : 0;
        if (dut->wr_en) {
            dut->wr_data = 0x3000 + i;
            expected.push_back(0x3000 + i);
        }
        // The reader side runs between wclk edges.  Sample rd_data
        // just before the rclk pulse.
        dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once();

        // Now do the slow wclk pulse.
        dut->wclk = 1; eval_once(); dut->wclk = 0; eval_once();

        // 4 more fast rclks; pop when possible.
        for (int j = 0; j < 4; j++) {
            dut->rd_en = !dut->rd_empty ? 1 : 0;
            uint32_t data = dut->rd_data;
            bool checking = dut->rd_en == 1;
            dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once();
            if (checking) {
                CHECK("expected not empty", !expected.empty());
                CHECK_EQ("slow-w fast-r pop", data, expected.front());
                expected.pop_front();
            }
        }
        dut->wr_en = 0; dut->rd_en = 0;
    }
    // Drain remainder.
    dut->rd_en = 1;
    for (int j = 0; j < 50; j++) {
        uint32_t data = dut->rd_data;
        bool checking = !dut->rd_empty && expected.size() > 0;
        dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once();
        if (checking) {
            CHECK_EQ("slow-w drain pop", data, expected.front());
            expected.pop_front();
        }
        if (expected.empty()) break;
    }
    dut->rd_en = 0;
    CHECK("drained all", expected.empty());
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 9. fast_wclk_slow_rclk — writer 5×, reader 1×.
// ══════════════════════════════════════════════════════════════════════
static bool test_fast_wclk_slow_rclk() {
    reset_both();
    std::deque<uint32_t> expected;
    dut->wr_en = 0; dut->rd_en = 0;
    int wr_index = 0;

    for (int i = 0; i < 50; i++) {
        // 5 wclks: try to push each cycle.
        for (int j = 0; j < 5; j++) {
            dut->wr_en = !dut->wr_full ? 1 : 0;
            if (dut->wr_en) {
                dut->wr_data = 0x4000 + wr_index;
                expected.push_back(0x4000 + wr_index);
                wr_index++;
            }
            dut->wclk = 1; eval_once(); dut->wclk = 0; eval_once();
        }
        dut->wr_en = 0;

        // One rclk: pop if possible.
        dut->rd_en = !dut->rd_empty ? 1 : 0;
        uint32_t data = dut->rd_data;
        bool checking = dut->rd_en == 1;
        dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once();
        if (checking) {
            CHECK("expected not empty", !expected.empty());
            CHECK_EQ("fast-w slow-r pop", data, expected.front());
            expected.pop_front();
        }
        dut->rd_en = 0;
    }
    // Drain remainder on reader side.
    dut->rd_en = 1;
    for (int j = 0; j < 200; j++) {
        uint32_t data = dut->rd_data;
        bool checking = !dut->rd_empty && expected.size() > 0;
        dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once();
        if (checking) {
            CHECK_EQ("fast-w drain pop", data, expected.front());
            expected.pop_front();
        }
        if (expected.empty()) break;
    }
    CHECK("fast-w drained all", expected.empty());
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 10. full_then_drain_then_push
// ══════════════════════════════════════════════════════════════════════
static bool test_full_drain_push() {
    reset_both();
    std::deque<uint32_t> expected;
    // fill
    dut->wr_en = 1;
    for (int i = 0; i < DEPTH + 4; i++) {
        if (!dut->wr_full) {
            dut->wr_data = 0x5000 + i;
            expected.push_back(0x5000 + i);
        }
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();
    CHECK("full before drain", dut->wr_full == 1);

    // Drain half.
    dut->rd_en = 1;
    int popped = 0;
    while (popped < DEPTH/2) {
        if (!dut->rd_empty) {
            CHECK_EQ("drain-half pop", dut->rd_data, expected.front());
            expected.pop_front();
            popped++;
        }
        tick_both();
    }
    dut->rd_en = 0;
    for (int i = 0; i < 8; i++) tick_both();

    // Push more.  wr_full must have deasserted.
    CHECK("wr_full deasserted after drain", dut->wr_full == 0);
    dut->wr_en = 1;
    for (int i = 0; i < DEPTH/2; i++) {
        if (!dut->wr_full) {
            dut->wr_data = 0x6000 + i;
            expected.push_back(0x6000 + i);
        }
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    // Drain all; ordering preserved.
    dut->rd_en = 1;
    for (int i = 0; i < DEPTH*2 + 10; i++) {
        if (!dut->rd_empty) {
            CHECK("expected still non-empty", !expected.empty());
            CHECK_EQ("final drain pop", dut->rd_data, expected.front());
            expected.pop_front();
        }
        tick_both();
        if (expected.empty()) break;
    }
    dut->rd_en = 0;
    CHECK("everything drained", expected.empty());
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 11. empty_then_push_then_pop
// ══════════════════════════════════════════════════════════════════════
static bool test_empty_push_pop() {
    reset_both();
    CHECK("empty at start", dut->rd_empty == 1);
    dut->wr_en = 1;
    dut->wr_data = 0xFEEDFACE;
    tick_both();
    dut->wr_en = 0;
    for (int i = 0; i < 8; i++) tick_both();
    CHECK("not empty after push+sync", dut->rd_empty == 0);
    CHECK_EQ("data visible", dut->rd_data, 0xFEEDFACE);
    dut->rd_en = 1;
    tick_both();
    dut->rd_en = 0;
    for (int i = 0; i < 6; i++) tick_both();
    CHECK("empty after pop again", dut->rd_empty == 1);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 12-14. one_sided_mside_reset_mid_stream (+ ratioed variants; see #15
//     below for the ratio-sustained-through-post-release extension) —
//     T6 CDC/reset-hardening regression: assert ONLY rrst (the
//     "m-side"/reader reset — e.g. a MIG UI recalibration pulse) while
//     the writer keeps running.  The coupled reset/ack handshake in
//     async_fifo.v must synchronise rrst into the wclk domain and force
//     wptr to reset too, so the FIFO settles to a clean, consistent
//     empty state rather than a Gray-pointer desync (writer counting up
//     against a pointer the reader thinks is zero).
//
//     Parameterised by a `tick` function so the same scenario runs at
//     1:1 and at the two 1:5 ratio orientations (deployment shape is
//     ~50 MHz vs ~333 MHz) — this directly exercises the request/ack
//     handshake's convergence under asymmetric clock rates, not just
//     the same-rate case.
// ══════════════════════════════════════════════════════════════════════
// `tick()` is applied ONLY to the reset-assert and settle windows below
// — the phase that actually exercises the request/ack handshake's
// cross-domain convergence.  Priming and pre/post data-correctness
// checks use the already-proven 1:1 `tick_both()` so this test isn't
// also re-deriving the general ratioed-FIFO data-ordering coverage
// `slow_wclk_fast_rclk`/`fast_wclk_slow_rclk` already provide — mixing
// a burst-capable ratioed tick with a "one push/pop per loop iteration"
// checking pattern (as those tests correctly avoid via manual per-edge
// interleaving) would require re-deriving that same interleaving here
// for no additional handshake coverage.
static bool run_one_sided_mside_reset_mid_stream(void (*tick)()) {
    reset_both();
    std::deque<uint32_t> expected;

    // Prime with a few words and drain a couple, so both pointers are
    // non-zero and the FIFO is non-empty at the moment reset hits.
    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0x7000 + i;
        expected.push_back(0x7000 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 2; i++) {
        CHECK("mside: mid-stream pop before reset", !dut->rd_empty);
        CHECK_EQ("mside: mid-stream pop data", dut->rd_data, expected.front());
        expected.pop_front();
        tick_both();
    }
    dut->rd_en = 0;

    // Assert ONLY rrst for several ticks AT THE REQUESTED RATIO.  The
    // writer keeps trying to push the whole time — on correct (coupled)
    // RTL this must be blocked once the request/ack handshake engages;
    // on old (uncoupled) RTL the writer never sees any reset here at
    // all.
    dut->rrst  = 1;
    dut->wr_en = 1;
    dut->wr_data = 0xDEAD0000;
    for (int i = 0; i < 20; i++) {
        tick();
    }
    dut->wr_en = 0;
    dut->rrst  = 0;

    // Let both domains settle out of the coupled reset (still at the
    // requested ratio) — the request/ack round trip adds latency beyond
    // the naive level-OR scheme, so give this more margin than before.
    for (int i = 0; i < 20; i++) tick();

    CHECK("mside: empty after settle", dut->rd_empty == 1);
    CHECK("mside: full deasserted after settle", dut->wr_full == 0);

    // Subsequent traffic must be FIFO-correct.  Back to 1:1 — the
    // handshake convergence has already been exercised above; this is
    // just confirming recovery.
    std::deque<uint32_t> post;
    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0x7100 + i;
        post.push_back(0x7100 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 8; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 80 && !post.empty(); i++) {
        if (!dut->rd_empty) {
            CHECK_EQ("mside: post-reset pop", dut->rd_data, post.front());
            post.pop_front();
        }
        tick_both();
    }
    dut->rd_en = 0;
    CHECK("mside: post-reset traffic fully drained", post.empty());
    return true;
}
static bool test_one_sided_mside_reset_mid_stream() {
    return run_one_sided_mside_reset_mid_stream(tick_both);
}
static void tick_ratio_wr_slow_5() { tick_wr_slow(5); }
static void tick_ratio_rr_slow_5() { tick_rr_slow(5); }
static bool test_one_sided_mside_reset_ratio_wr_slow() {
    return run_one_sided_mside_reset_mid_stream(tick_ratio_wr_slow_5);
}
static bool test_one_sided_mside_reset_ratio_wr_fast() {
    return run_one_sided_mside_reset_mid_stream(tick_ratio_rr_slow_5);
}

// ══════════════════════════════════════════════════════════════════════
// 15. one_sided_mside_reset_ratio_wr_slow_sustained — MINOR (requested
//     in T6 re-review round 2): extends the ratioed one-sided m-side
//     reset scenario to keep the SAME 1:5 ratio through the POST-
//     RELEASE data phase too, not just the reset-assert/settle window —
//     i.e. "resumed sampling mid-stream" coverage: does normal FIFO
//     traffic recover correctly while STILL running at the deployment
//     clock ratio, rather than falling back to 1:1 the instant the
//     handshake settles (as the base ratioed variants above do, by
//     design — see the comment on run_one_sided_mside_reset_mid_stream).
//
//     Uses tick_wr_slow(5) throughout (writer slow/1x, reader fast/5x
//     per tick() call) because this direction is safe for a "check
//     available, pop, tick(), repeat" polling pattern on BOTH sides:
//     writes are exactly 0-or-1 per tick() call (wclk pulses once per
//     call, so updating wr_data once per loop iteration is always
//     correct), and reads use the SAME robust poll-for-available
//     pattern already used for post-reset draining in the base
//     scenarios above (tolerant of the up-to-5 pops that can land
//     inside one tick() call on the reader's fast rclk — it doesn't
//     assume a fixed event count per tick(), just checks state at each
//     loop-iteration boundary).
// ══════════════════════════════════════════════════════════════════════
// Manual per-edge interleave: 1 wclk pulse, then 5 rclk pulses, checking
// rd_data/rd_empty individually on EACH rclk edge — correctly handles
// up to 5 pops landing inside one "outer" iteration.  This is the same
// interleaving discipline test_slow_wclk_fast_rclk() uses; a black-box
// tick_wr_slow(5) call only exposes DUT state once per call, so a
// "check once, assume <=1 event, then call tick()" loop silently
// misattributes events whenever more than one pop lands inside a single
// call — exactly the bug this helper avoids.  Pops up to `max_pops`
// items out of `q` (FIFO order), across at most `max_outer_iters` outer
// iterations.  Returns the number of items actually popped+verified, or
// -1 on a data mismatch (also prints the failure).
static int drain_ratio_wr_slow_checked(std::deque<uint32_t>& q, int max_pops,
                                        int max_outer_iters) {
    int popped = 0;
    for (int i = 0; i < max_outer_iters && popped < max_pops && !q.empty(); i++) {
        dut->wclk = 1; eval_once(); dut->wclk = 0; eval_once();
        for (int j = 0; j < 5 && popped < max_pops && !q.empty(); j++) {
            dut->rd_en = !dut->rd_empty ? 1 : 0;
            uint32_t data = dut->rd_data;
            bool checking = dut->rd_en == 1;
            dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once();
            if (checking) {
                if (data != q.front()) {
                    printf("  FAIL mside-sustained: drain pop: got 0x%x expected 0x%x\n",
                           data, q.front());
                    return -1;
                }
                q.pop_front();
                popped++;
            }
        }
    }
    return popped;
}

static bool test_one_sided_mside_reset_ratio_wr_slow_sustained() {
    reset_both();
    std::deque<uint32_t> expected;

    // Prime at the ratio — push side is safe (exactly one wclk edge per
    // tick_ratio_wr_slow_5() call).
    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0x9000 + i;
        expected.push_back(0x9000 + i);
        tick_ratio_wr_slow_5();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_ratio_wr_slow_5();

    // Drain 2 words before the reset, still at the ratio, using the
    // per-edge-checked helper (NOT the naive "check once per tick()"
    // pattern, which the base ratioed variants avoid entirely by
    // scoping the ratio to reset-assert/settle only — see that
    // scenario's comment for the bug this caused during development).
    int got = drain_ratio_wr_slow_checked(expected, 2, 60);
    dut->rd_en = 0;
    CHECK("mside-sustained: pre-reset drain reached target count", got == 2);

    // Assert ONLY rrst, still at the ratio.
    dut->rrst    = 1;
    dut->wr_en   = 1;
    dut->wr_data = 0xDEAD1000;
    for (int i = 0; i < 20; i++) tick_ratio_wr_slow_5();
    dut->wr_en   = 0;
    dut->rrst    = 0;

    for (int i = 0; i < 20; i++) tick_ratio_wr_slow_5();

    CHECK("mside-sustained: empty after settle", dut->rd_empty == 1);
    CHECK("mside-sustained: full deasserted after settle", dut->wr_full == 0);

    // Post-release traffic STAYS at the ratio — this is the coverage
    // this scenario adds over the base ratioed variants, which fall
    // back to 1:1 tick_both() here for simplicity.
    std::deque<uint32_t> post;
    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0x9100 + i;
        post.push_back(0x9100 + i);
        tick_ratio_wr_slow_5();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 8; i++) tick_ratio_wr_slow_5();

    int post_total = (int)post.size();
    int got_post = drain_ratio_wr_slow_checked(post, post_total, 120);
    dut->rd_en = 0;
    CHECK("mside-sustained: post-reset traffic fully drained",
          got_post == post_total && post.empty());
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 16-18. one_sided_sside_reset_mid_stream (+ ratioed variants) — mirror
//     of the above: assert ONLY wrst (the "s-side"/writer reset — e.g.
//     a PCIe link retrain poisoning the debug master's upstream side)
//     while the reader keeps trying to pop.  Symmetric desync risk:
//     reader keeps advancing against a writer pointer the writer thinks
//     is zero.  Same 1:1 / 1:5-both-orientations parameterisation.
// ══════════════════════════════════════════════════════════════════════
// Same "tick() only during reset-assert/settle" strategy as
// run_one_sided_mside_reset_mid_stream() above — see its comment.
static bool run_one_sided_sside_reset_mid_stream(void (*tick)()) {
    reset_both();
    std::deque<uint32_t> expected;

    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0x8000 + i;
        expected.push_back(0x8000 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 2; i++) {
        CHECK("sside: mid-stream pop before reset", !dut->rd_empty);
        CHECK_EQ("sside: mid-stream pop data", dut->rd_data, expected.front());
        expected.pop_front();
        tick_both();
    }

    // Assert ONLY wrst for several ticks AT THE REQUESTED RATIO.  The
    // reader keeps trying to pop the whole time.
    dut->wrst = 1;
    for (int i = 0; i < 20; i++) {
        tick();
    }
    dut->rd_en = 0;
    dut->wrst  = 0;

    for (int i = 0; i < 20; i++) tick();

    CHECK("sside: empty after settle", dut->rd_empty == 1);
    CHECK("sside: full deasserted after settle", dut->wr_full == 0);

    std::deque<uint32_t> post;
    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0x8100 + i;
        post.push_back(0x8100 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 8; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 80 && !post.empty(); i++) {
        if (!dut->rd_empty) {
            CHECK_EQ("sside: post-reset pop", dut->rd_data, post.front());
            post.pop_front();
        }
        tick_both();
    }
    dut->rd_en = 0;
    CHECK("sside: post-reset traffic fully drained", post.empty());
    return true;
}
static bool test_one_sided_sside_reset_mid_stream() {
    return run_one_sided_sside_reset_mid_stream(tick_both);
}
static bool test_one_sided_sside_reset_ratio_wr_slow() {
    return run_one_sided_sside_reset_mid_stream(tick_ratio_wr_slow_5);
}
static bool test_one_sided_sside_reset_ratio_wr_fast() {
    return run_one_sided_sside_reset_mid_stream(tick_ratio_rr_slow_5);
}

// ══════════════════════════════════════════════════════════════════════
// one_sided_mside_reset_back_to_back / one_sided_sside_reset_back_to_back
//     — T6 re-review Finding 3 regression: two back-to-back one-sided
//     reset pulses (rrst-only, then wrst-only mirror) separated by only
//     1, 2, and 3 idle cycles (swept within one scenario, since a
//     narrow-window bug could hide behind a single lucky separation
//     value).  Without the stale-ack force-clear ("!w_eng"/"!r_eng") +
//     read-gate ("& w_eng"/"& r_eng") fix, the SECOND reset's
//     w_zero/r_zero could fire using a leftover ack from the FIRST
//     (already fully drained) handshake before the remote side has
//     actually engaged for the SECOND request — zeroing a pointer
//     against a genuinely live remote and reopening the phantom-pop
//     hazard specifically for back-to-back resets.
// ══════════════════════════════════════════════════════════════════════
static bool test_one_sided_mside_reset_back_to_back() {
    reset_both();
    std::deque<uint32_t> expected;

    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0xA000 + i;
        expected.push_back(0xA000 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 2; i++) {
        CHECK("b2b-mside: pre-reset pop", !dut->rd_empty);
        CHECK_EQ("b2b-mside: pre-reset pop data", dut->rd_data, expected.front());
        expected.pop_front();
        tick_both();
    }
    dut->rd_en = 0;

    // First reset pulse (m-side / rrst only).
    dut->rrst    = 1;
    dut->wr_en   = 1;
    dut->wr_data = 0xB0000001;
    for (int i = 0; i < 12; i++) tick_both();
    dut->rrst  = 0;
    dut->wr_en = 0;

    // Sweep a 1, 2, and 3 idle-cycle gap before a SECOND, independent
    // reset pulse fires — the first handshake's ack pipe may still be
    // mid-drain at that point.
    for (int gap = 1; gap <= 3; gap++) {
        for (int i = 0; i < gap; i++) tick_both();

        dut->rrst    = 1;
        dut->wr_en   = 1;
        dut->wr_data = 0xB0000002 + gap;
        for (int i = 0; i < 12; i++) tick_both();
        dut->rrst  = 0;
        dut->wr_en = 0;

        for (int i = 0; i < 20; i++) tick_both();

        char label[64];
        snprintf(label, sizeof(label), "b2b-mside gap=%d: empty after settle", gap);
        CHECK(label, dut->rd_empty == 1);
        snprintf(label, sizeof(label), "b2b-mside gap=%d: full deasserted after settle", gap);
        CHECK(label, dut->wr_full == 0);
    }

    // Fresh traffic after all the back-to-back resets must still be
    // FIFO-correct.
    std::deque<uint32_t> post;
    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0xA100 + i;
        post.push_back(0xA100 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 8; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 40 && !post.empty(); i++) {
        if (!dut->rd_empty) {
            CHECK_EQ("b2b-mside: post-reset pop", dut->rd_data, post.front());
            post.pop_front();
        }
        tick_both();
    }
    dut->rd_en = 0;
    CHECK("b2b-mside: post-reset traffic fully drained", post.empty());
    return true;
}

static bool test_one_sided_sside_reset_back_to_back() {
    reset_both();
    std::deque<uint32_t> expected;

    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0xC000 + i;
        expected.push_back(0xC000 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 6; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 2; i++) {
        CHECK("b2b-sside: pre-reset pop", !dut->rd_empty);
        CHECK_EQ("b2b-sside: pre-reset pop data", dut->rd_data, expected.front());
        expected.pop_front();
        tick_both();
    }

    // First reset pulse (s-side / wrst only).  Reader keeps trying to
    // pop the whole time (dut->rd_en stays 1 from above).
    dut->wrst = 1;
    for (int i = 0; i < 12; i++) tick_both();
    dut->wrst = 0;

    for (int gap = 1; gap <= 3; gap++) {
        for (int i = 0; i < gap; i++) tick_both();

        dut->wrst = 1;
        for (int i = 0; i < 12; i++) tick_both();
        dut->wrst = 0;

        for (int i = 0; i < 20; i++) tick_both();

        char label[64];
        snprintf(label, sizeof(label), "b2b-sside gap=%d: empty after settle", gap);
        CHECK(label, dut->rd_empty == 1);
        snprintf(label, sizeof(label), "b2b-sside gap=%d: full deasserted after settle", gap);
        CHECK(label, dut->wr_full == 0);
    }
    dut->rd_en = 0;

    std::deque<uint32_t> post;
    dut->wr_en = 1;
    for (int i = 0; i < 6; i++) {
        dut->wr_data = 0xC100 + i;
        post.push_back(0xC100 + i);
        tick_both();
    }
    dut->wr_en = 0;
    for (int i = 0; i < 8; i++) tick_both();

    dut->rd_en = 1;
    for (int i = 0; i < 40 && !post.empty(); i++) {
        if (!dut->rd_empty) {
            CHECK_EQ("b2b-sside: post-reset pop", dut->rd_data, post.front());
            post.pop_front();
        }
        tick_both();
    }
    dut->rd_en = 0;
    CHECK("b2b-sside: post-reset traffic fully drained", post.empty());
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// phase_swept_opposite_side_back_to_back_ratio — T6 final review round:
//     Finding (e) regression (RATIO-DEFEATED REQUEST-COLLAPSE HAZARD,
//     see the async_fifo.v header comment).  Unlike the same-side
//     back-to-back scenarios above (which stress the stale-ack pipe but
//     ARE NOT discrete-time-observable in Verilator — a real-silicon
//     torn-Gray-bit issue), this hazard is about a request pulse
//     collapsing to ~2 local cycles and falling entirely BETWEEN the
//     remote domain's slow synchroniser samples — a genuine MISS, not a
//     torn-bit issue, and therefore fully visible to a cycle-accurate
//     simulator.
//
//     Shape: event A = rrst-only (R-side) reset, run to full
//     completion at a 1:5 ratio with wclk FAST / rclk SLOW
//     (tick_ratio_rr_slow_5) -- R is "local" for event A and syncs INTO
//     the fast wclk domain, which is the EASY (never-missed) direction,
//     so event A is uninteresting for this hazard and exists only to
//     put the FIFO into the "just came out of a coupled reset" state
//     the stale-ack machinery cares about.  Event B = a SHORT wrst-only
//     pulse (single wclk cycle) -- W is "local" for event B and syncs
//     INTO the slow rclk domain, the HARD direction this hazard
//     targets.  Event B's start phase relative to the ratio's internal
//     5:1 sub-edge cadence is swept across 8 offsets (0..7 extra wclk
//     sub-edges inserted before it fires) so a narrow miss window can't
//     hide behind a single lucky phase.  A push is attempted right in
//     the inter-reset gap (between event A settling and event B firing)
//     to match the reviewer's exact scenario shape.
//
//     "Engagement for event B always occurs" is verified INDIRECTLY but
//     unambiguously: if R never engages for event B (the request pulse
//     was missed), req_w can never observe a genuine ack and therefore
//     never clears -- w_eng (and therefore wr_full) stays stuck high
//     FOREVER.  That is exactly what "full deasserted after event B
//     settle" below checks, and what would make the post-recovery
//     push+drain phase fail outright (nothing would ever be accepted).
// ══════════════════════════════════════════════════════════════════════
// Run event A (rrst-only, 1:5 ratio wclk-fast/rclk-slow) to genuine
// engagement, deassert, then advance one wclk sub-edge at a time
// (background rclk edges interleaved at the correct 5:1 cadence)
// counting edges until dut->wr_full is observed falling 1->0 -- the
// moment event A's RELAYED engagement (w_eng, via req_r_s2) actually
// drops on the W side.  This is an empirical probe rather than a fixed
// guess: the exact cycle count depends on req_r's own
// REQ_MIN_HOLD-plus-ack release timing in the (slow) rclk domain, which
// isn't worth hand-deriving precisely when the DUT can just be asked.
// Returns the wclk sub-edge count at the transition (>0), or -1 if not
// observed within the budget.
static int probe_event_a_wr_full_drop_edges() {
    dut->rrst = 1;
    for (int i = 0; i < 20; i++) tick_ratio_rr_slow_5();
    dut->rrst = 0;

    bool prev_full = dut->wr_full;
    int wclk_edges = 0;
    for (int outer = 0; outer < 60; outer++) {
        for (int j = 0; j < 5; j++) {
            dut->wclk = 1; eval_once(); dut->wclk = 0; eval_once();
            wclk_edges++;
            bool now_full = dut->wr_full;
            if (prev_full && !now_full) {
                return wclk_edges;
            }
            prev_full = now_full;
        }
        dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once();
    }
    return -1;
}

static bool test_phase_swept_opposite_side_back_to_back_ratio() {
    // Locate event A's actual wr_full 1->0 transition (its "dying
    // tail" boundary on the W side) via a probe pass, then sweep 8
    // offsets STRADDLING that boundary (both sides, not just after it)
    // — a fixed guess or a sweep anchored only at "some cycles after
    // deassertion" reliably missed the hazard during development: this
    // hazard requires event B's wrst to land WHILE w_eng is still
    // continuously 1 (straight through, no intervening 0), because the
    // Finding-3 force-clear (`!w_eng`) wipes the ack pipe the moment
    // w_eng reads 0 for even a single cycle — so only offsets at or
    // just before the transition can possibly reproduce it; offsets
    // comfortably after it cannot (force-clear has already run) and
    // offsets during the fully-engaged region haven't reached the tail
    // yet either.
    reset_both();
    dut->wr_en = 0;
    for (int i = 0; i < 4; i++) tick_both();
    int drop_edge = probe_event_a_wr_full_drop_edges();
    if (drop_edge < 0) {
        printf("  FAIL phase-swept: could not observe event A's wr_full drop within budget\n");
        return false;
    }

    static const int kOffsetDeltas[8] = {-2, -1, 0, 1, 2, 3, 5, 8};
    for (int pidx = 0; pidx < 8; pidx++) {
        int target_edge = drop_edge + kOffsetDeltas[pidx];
        if (target_edge < 0) target_edge = 0;

        reset_both();

        // Prime + drain a touch at 1:1 so pointers are non-zero before
        // event A even starts (parity with the other one-sided
        // regression scenarios; not load-bearing for this specific
        // hazard, which is about request-pulse visibility, not pointer
        // state).
        dut->wr_en = 1;
        for (int i = 0; i < 4; i++) {
            dut->wr_data = 0xD000 + i;
            tick_both();
        }
        dut->wr_en = 0;
        for (int i = 0; i < 6; i++) tick_both();
        dut->rd_en = 1;
        for (int i = 0; i < 6; i++) tick_both();
        dut->rd_en = 0;

        // Event A: rrst-only, held long enough to genuinely engage, at
        // the 1:5 ratio -- the easy/uninteresting direction for event A
        // itself.  Deliberately no settle/CHECK checkpoint between
        // deasserting rrst and starting event B below (see the probe
        // comment above for why).
        dut->rrst = 1;
        for (int i = 0; i < 20; i++) tick_ratio_rr_slow_5();
        dut->rrst = 0;

        // Advance exactly `target_edge` wclk sub-edges (background rclk
        // interleaved at the 5:1 cadence, matching the probe pass
        // above) so event B lands at the swept offset relative to the
        // empirically-located transition.
        for (int i = 0; i < target_edge; i++) {
            dut->wclk = 1; eval_once(); dut->wclk = 0; eval_once();
            if ((i % 5) == 4) { dut->rclk = 1; eval_once(); dut->rclk = 0; eval_once(); }
        }

        // A push landing in the inter-reset gap, right as event B is
        // about to fire.
        dut->wr_en   = 1;
        dut->wr_data = 0xE000 + pidx;
        dut->wclk = 1; eval_once(); dut->wclk = 0; eval_once();
        dut->wr_en = 0;

        // Event B: wrst-only, a SINGLE wclk-cycle pulse -- this is
        // exactly the caller contract REQ_MIN_HOLD exists to make safe
        // (the header now claims single-local-cycle-pulse safety is
        // ENFORCED, not just documented).
        dut->wrst = 1;
        dut->wclk = 1; eval_once(); dut->wclk = 0; eval_once();
        dut->wrst = 0;

        // Generous settle window, covering BOTH events, still at the
        // ratio.
        char label[96];
        for (int i = 0; i < 60; i++) tick_ratio_rr_slow_5();

        snprintf(label, sizeof(label), "delta=%d: empty after settle", kOffsetDeltas[pidx]);
        CHECK(label, dut->rd_empty == 1);
        snprintf(label, sizeof(label), "delta=%d: full deasserted after settle", kOffsetDeltas[pidx]);
        CHECK(label, dut->wr_full == 0);

        // Post-recovery: fresh traffic must be FIFO-correct (back to
        // 1:1 for simplicity of the data-ordering check itself; the
        // ratio's job was to stress event B above).
        std::deque<uint32_t> post;
        dut->wr_en = 1;
        for (int i = 0; i < 4; i++) {
            dut->wr_data = 0xF000 + i;
            post.push_back(0xF000 + i);
            tick_both();
        }
        dut->wr_en = 0;
        for (int i = 0; i < 8; i++) tick_both();

        dut->rd_en = 1;
        for (int i = 0; i < 40 && !post.empty(); i++) {
            if (!dut->rd_empty) {
                snprintf(label, sizeof(label), "delta=%d: post-recovery pop", kOffsetDeltas[pidx]);
                CHECK_EQ(label, dut->rd_data, post.front());
                post.pop_front();
            }
            tick_both();
        }
        dut->rd_en = 0;
        snprintf(label, sizeof(label), "delta=%d: post-recovery traffic fully drained", kOffsetDeltas[pidx]);
        CHECK(label, post.empty());
    }
    return true;
}

// ─── Runner ──────────────────────────────────────────────────────────────
static void run(const char* name, bool (*fn)()) {
    bool r = fn();
    printf("[%s] %s\n", r ? "PASS" : "FAIL", name);
    if (r) n_pass++; else n_fail++;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vasync_fifo();

    run("reset_clears_state",        test_reset_clears_state);
    run("single_write_read",         test_single_write_read);
    run("fill_to_full",              test_fill_to_full);
    run("drain_to_empty",            test_drain_to_empty);
    run("simultaneous_rw_steady",    test_simultaneous_rw_steady);
    run("wraparound",                test_wraparound);
    run("watermarks",                test_watermarks);
    run("slow_wclk_fast_rclk",       test_slow_wclk_fast_rclk);
    run("fast_wclk_slow_rclk",       test_fast_wclk_slow_rclk);
    run("full_then_drain_then_push", test_full_drain_push);
    run("empty_then_push_then_pop",  test_empty_push_pop);
    run("one_sided_mside_reset_mid_stream", test_one_sided_mside_reset_mid_stream);
    run("one_sided_mside_reset_ratio_wr_slow", test_one_sided_mside_reset_ratio_wr_slow);
    run("one_sided_mside_reset_ratio_wr_fast", test_one_sided_mside_reset_ratio_wr_fast);
    run("one_sided_mside_reset_ratio_wr_slow_sustained", test_one_sided_mside_reset_ratio_wr_slow_sustained);
    run("one_sided_sside_reset_mid_stream", test_one_sided_sside_reset_mid_stream);
    run("one_sided_sside_reset_ratio_wr_slow", test_one_sided_sside_reset_ratio_wr_slow);
    run("one_sided_sside_reset_ratio_wr_fast", test_one_sided_sside_reset_ratio_wr_fast);
    run("one_sided_mside_reset_back_to_back", test_one_sided_mside_reset_back_to_back);
    run("one_sided_sside_reset_back_to_back", test_one_sided_sside_reset_back_to_back);
    run("phase_swept_opposite_side_back_to_back_ratio", test_phase_swept_opposite_side_back_to_back_ratio);

    printf("\nasync_fifo: %d PASS / %d FAIL\n", n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
