// tb_if_stage.cpp — Verilator unit testbench for if_stage.v
//
// Exercises the instruction-fetch front-end's prefetch + line-buffer
// logic in isolation, with a simple 128-bit memory BFM serving line
// fetches.  Validates the task-#107 changes: the 1-entry victim slot
// and the adjacency-hit promotion.
//
// Scenarios:
//   1. cold_sequential_fetch       — no prefetch miss after the first
//                                    line lands; pd_valid stays high
//                                    as pc marches through multiple
//                                    lines sequentially.
//   2. mid_seq_branch_mispredict   — br_redirect flushes l0/l1; later
//                                    decode resumes at the new PC and
//                                    never sees stale bytes.
//   3. backward_branch_soft_hit    — a pred_redirect whose target lies
//                                    in the VICTIM slot is served with
//                                    zero refetch (the first cycle
//                                    post-redirect already has
//                                    pd_valid=1 and the correct window
//                                    contents).  This is the main
//                                    task-#107 win.
//   4. pred_redirect_without_consume
//                                  — a predicted redirect must take effect
//                                    even when the visible fall-through
//                                    decoder reports pd_consumed=0.
//   5. fetch_fault_metadata        — a faulted line still produces a
//                                    decode window, tagged as a fetch fault
//                                    for commit-time realization.
//
// Build:  make tb-if-stage
// Each scenario is a standalone function that drives if_stage with a
// controlled sequence of pd_consumed / br_redirect / pred_redirect
// pulses and inspects pd_valid, pd_buf, and the if_req / if_addr
// backward-facing bus.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>
#include <verilated.h>
#include "Vif_stage.h"

static Vif_stage* dut = nullptr;
static uint64_t   sim_time = 0;
static int        n_pass = 0, n_fail = 0;

// ─── Simple line-fetch BFM ───────────────────────────────────────────
// Serves 16-byte lines with a CONFIGURABLE delay (default 2 cycles to
// mimic the real BRAM-backed icache's 2-cycle read latency).  Sparse
// byte store (big-endian: byte 0 at MSB of the 128-bit line).
struct LineBus {
    std::map<uint32_t, uint8_t> store;
    std::map<uint32_t, bool> fault_lines;
    int      n_reqs   = 0;
    bool     captured = false;
    uint32_t cap_addr = 0;
    int      delay    = 0;
    int      delay_cycles = 2;  // adjustable: cold-miss latency

    uint8_t byte_at(uint32_t a) {
        auto it = store.find(a);
        return (it == store.end()) ? 0 : it->second;
    }
    bool fault_at(uint32_t a) {
        auto it = fault_lines.find(a & ~0xFu);
        return it != fault_lines.end() && it->second;
    }
    void write_word(uint32_t a, uint32_t w) {
        // BE: a+0 = MSB.
        store[a + 0] = (w >> 24) & 0xFF;
        store[a + 1] = (w >> 16) & 0xFF;
        store[a + 2] = (w >>  8) & 0xFF;
        store[a + 3] = (w >>  0) & 0xFF;
    }
    void clear_stats() { n_reqs = 0; }
};
static LineBus lbus;

static void drive_bus() {
    dut->if_rvalid = 0;
    dut->if_fault = 0;
    for (int i = 0; i < 4; i++) dut->if_rdata.at(i) = 0;

    // Capture request (if not already servicing and one is asserted).
    if (dut->if_req && !lbus.captured) {
        lbus.captured = true;
        lbus.cap_addr = dut->if_addr & ~0xFu;
        lbus.delay    = lbus.delay_cycles;
        lbus.n_reqs++;
    }

    if (lbus.captured) {
        if (lbus.delay > 0) {
            lbus.delay--;
        } else {
            // Deliver line.  Pack big-endian into LE word array
            // (word 3 = bits [127:96] = BE bytes 0..3).
            uint8_t line[16];
            for (int i = 0; i < 16; i++) line[i] = lbus.byte_at(lbus.cap_addr + i);
            uint32_t w3 = ((uint32_t)line[0]  << 24) | ((uint32_t)line[1]  << 16)
                        | ((uint32_t)line[2]  <<  8) |  line[3];
            uint32_t w2 = ((uint32_t)line[4]  << 24) | ((uint32_t)line[5]  << 16)
                        | ((uint32_t)line[6]  <<  8) |  line[7];
            uint32_t w1 = ((uint32_t)line[8]  << 24) | ((uint32_t)line[9]  << 16)
                        | ((uint32_t)line[10] <<  8) |  line[11];
            uint32_t w0 = ((uint32_t)line[12] << 24) | ((uint32_t)line[13] << 16)
                        | ((uint32_t)line[14] <<  8) |  line[15];
            dut->if_rdata.at(0) = w0;
            dut->if_rdata.at(1) = w1;
            dut->if_rdata.at(2) = w2;
            dut->if_rdata.at(3) = w3;
            dut->if_rvalid      = 1;
            dut->if_fault       = lbus.fault_at(lbus.cap_addr) ? 1 : 0;
        }
    }
    dut->eval();
}

// Clock one posedge; handle request/response transactions around the edge.
static void tick() {
    drive_bus();
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();

    // After the posedge the DUT latches the response and drops if_req.
    if (lbus.captured && lbus.delay == 0 && dut->if_rvalid) {
        lbus.captured = false;
    }
    sim_time++;
}

static void reset() {
    dut->rst           = 1;
    dut->pd_consumed   = 0;
    dut->br_redirect   = 0;
    dut->br_target     = 0;
    dut->pred_redirect = 0;
    dut->pred_target   = 0;
    dut->if_rvalid     = 0;
    dut->if_fault      = 0;
    for (int i = 0; i < 4; i++) dut->if_rdata.at(i) = 0;
    lbus.captured = false;
    lbus.delay    = 0;
    lbus.n_reqs   = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

// Return true when pd_valid stays high for `cycles` consecutive cycles
// while advancing pd_consumed=2 each step.  Used to validate that a
// prefetch hides the latency.
static bool consume_sequential(int cycles) {
    for (int i = 0; i < cycles; i++) {
        // Wait up to N cycles for pd_valid to assert.
        int timeout = 100;
        while (!dut->pd_valid && timeout-- > 0) {
            dut->pd_consumed = 0;
            tick();
        }
        if (!dut->pd_valid) return false;
        dut->pd_consumed = 2;
        tick();
    }
    dut->pd_consumed = 0;
    return true;
}

// Wait up to N cycles for pd_valid to go high; return the cycle count.
static int wait_for_pd_valid(int max_cycles = 100) {
    for (int i = 0; i < max_cycles; i++) {
        if (dut->pd_valid) return i;
        dut->pd_consumed = 0;
        tick();
    }
    return -1;
}

// Count how many line-fetch transactions occur over `cycles` ticks while
// advancing pd_consumed steadily.  Used to verify prefetch coverage.
static int measure_reqs_over(int cycles, int consumed_per_step) {
    int start = lbus.n_reqs;
    for (int i = 0; i < cycles; i++) {
        if (dut->pd_valid) dut->pd_consumed = consumed_per_step;
        else               dut->pd_consumed = 0;
        tick();
    }
    dut->pd_consumed = 0;
    return lbus.n_reqs - start;
}

#define CHECK_EQ(msg, got, exp) do { \
    if ((uint64_t)(got) != (uint64_t)(exp)) { \
        std::printf("    FAIL %s: got 0x%lx, expected 0x%lx\n", msg, \
                    (uint64_t)(got), (uint64_t)(exp)); \
        return false; \
    } \
} while (0)

#define CHECK_TRUE(msg, cond) do { \
    if (!(cond)) { std::printf("    FAIL %s\n", msg); return false; } \
} while (0)

// Seed one cache line (16 B) from four 32-bit words.
static void seed_line(uint32_t line_addr, uint32_t w0, uint32_t w1,
                      uint32_t w2, uint32_t w3) {
    lbus.write_word(line_addr + 0,  w0);
    lbus.write_word(line_addr + 4,  w1);
    lbus.write_word(line_addr + 8,  w2);
    lbus.write_word(line_addr + 12, w3);
}

// ─── Scenarios ─────────────────────────────────────────────────────────

// Scenario 1: Sequential fetch — the prefetcher should fill l1 in
// parallel with l0 consumption so pd_valid stays high as we march pc
// across multiple lines.  Measure the cycle cost of consuming 32 bytes
// (2 lines) with pd_consumed=2 each step — it should be close to 16
// cycles, not 16 + 2× icache-latency.
static bool test_cold_sequential_fetch() {
    reset();
    lbus.store.clear();
    lbus.fault_lines.clear();
    // Seed 3 lines at RESET_PC = 0x40800000.
    uint32_t base = 0x40800000;
    for (uint32_t line = 0; line < 3; line++)
        for (uint32_t off = 0; off < 16; off += 4)
            lbus.write_word(base + line * 16 + off,
                            0xAA00 | (line << 4) | (off / 4));

    // Let the front-end cold-start: wait for pd_valid to come up.
    int cold = wait_for_pd_valid(50);
    CHECK_TRUE("pd_valid after cold start", cold >= 0);
    CHECK_EQ("first cold fetch costs 1 downstream req", lbus.n_reqs, 1);

    // Now consume 32 bytes at 2 bytes/step.  Measure total cycles.
    int reqs_before = lbus.n_reqs;
    uint64_t t_before = sim_time;
    CHECK_TRUE("can consume 16 steps of 2 bytes each", consume_sequential(16));
    uint64_t cyc = sim_time - t_before;
    int reqs_added = lbus.n_reqs - reqs_before;

    // Expected: 2 extra lines fetched (for lines +1 and +2).  Cycles
    // should be close to 16 (the number of consumption steps) — any
    // icache stall would push it higher.
    CHECK_TRUE("extra reqs for 2 lines", reqs_added >= 1 && reqs_added <= 3);
    CHECK_TRUE("no excessive stall cycles (<=30)", cyc <= 30);
    return true;
}

// Scenario 2: A mid-sequence branch redirect flushes l0/l1 and resumes
// at the new target.  Verify pd_valid returns high with the correct
// data and no stale bytes.
static bool test_mid_seq_branch_mispredict() {
    reset();
    lbus.store.clear();
    lbus.fault_lines.clear();
    // Line A at 0x40800000, line B at 0x40801000 (different page).
    seed_line(0x40800000, 0xAAAA0000, 0xAAAA0001, 0xAAAA0002, 0xAAAA0003);
    seed_line(0x40801000, 0xBBBB0000, 0xBBBB0001, 0xBBBB0002, 0xBBBB0003);

    // Cold start: line A.
    CHECK_TRUE("pd_valid for line A", wait_for_pd_valid(50) >= 0);
    // Word 3 of pd_buf (bits [127:96]) = first 4 bytes of pc_line = AAAA0000.
    CHECK_EQ("line A bytes", dut->pd_buf.at(3), 0xAAAA0000);

    // Consume 4 bytes to advance pc into the line.
    dut->pd_consumed = 2;
    tick();
    dut->pd_consumed = 2;
    tick();
    dut->pd_consumed = 0;

    // Fire br_redirect to line B.
    dut->br_redirect = 1;
    dut->br_target   = 0x40801000;
    tick();
    dut->br_redirect = 0;
    dut->br_target   = 0;

    // Wait for pd_valid to come back with the new line.
    int post = wait_for_pd_valid(50);
    CHECK_TRUE("pd_valid after br_redirect", post >= 0);
    CHECK_EQ("pc reset to target", dut->pd_pc, 0x40801000);
    CHECK_EQ("line B bytes", dut->pd_buf.at(3), 0xBBBB0000);
    return true;
}

// Scenario 3: Backward-branch soft-hit.  Drive the fetch through line
// A → line B, then a pred_redirect back to line A.  Line A should
// still be resident in the victim slot, so the redirect is served
// without a fresh line-fetch.  This is the task-#107 primary win.
static bool test_backward_branch_soft_hit() {
    reset();
    lbus.store.clear();
    lbus.fault_lines.clear();
    // Line A at 0x40800000 (pc_line=0), line B at 0x40800010 (pc_line=1).
    seed_line(0x40800000, 0x11111111, 0x22222222, 0x33333333, 0x44444444);
    seed_line(0x40800010, 0x55555555, 0x66666666, 0x77777777, 0x88888888);

    CHECK_TRUE("pd_valid cold", wait_for_pd_valid(50) >= 0);
    CHECK_EQ("line A top word", dut->pd_buf.at(3), 0x11111111);

    // Consume all of line A + a few bytes of line B: 18 bytes total
    // (9 steps × 2 bytes).  This forces a line crossing that demotes
    // line A into the victim slot.
    for (int i = 0; i < 9; i++) {
        // Make sure pd_valid before consuming.
        int w = wait_for_pd_valid(50);
        CHECK_TRUE("pd_valid mid-stream", w >= 0);
        dut->pd_consumed = 2;
        tick();
    }
    dut->pd_consumed = 0;
    tick();

    // We should now be executing at pc=0x40800012 (line 1, offset 2).
    CHECK_EQ("pc mid line B", dut->pd_pc, 0x40800012);

    // Fire pred_redirect back to line A (pc=0x40800000) and make sure
    // consuming is true this cycle (pred_redirect is gated by
    // `consuming` inside if_stage).
    CHECK_TRUE("pd_valid for redirect", wait_for_pd_valid(50) >= 0);
    int reqs_before = lbus.n_reqs;
    dut->pd_consumed = 2;  // keep consuming to gate pred_redirect
    dut->pred_redirect = 1;
    dut->pred_target   = 0x40800000;
    tick();
    dut->pred_redirect = 0;
    dut->pred_target   = 0;
    dut->pd_consumed   = 0;

    // After pred_redirect, pc should be 0x40800000.  pd_valid should
    // come up QUICKLY (1 cycle) without a line-fetch because line A
    // is in the victim slot.
    int latency = 0;
    while (!dut->pd_valid && latency < 50) { tick(); latency++; }
    CHECK_TRUE("soft-hit pd_valid quickly", latency <= 3);
    CHECK_EQ("pc back to target", dut->pd_pc, 0x40800000);
    CHECK_EQ("line A top word restored", dut->pd_buf.at(3), 0x11111111);

    // Ideally ZERO new line-fetches for line A itself (it's in the
    // victim slot).  There may be a follow-up prefetch for the new
    // tail (line B), but that's on the prefetch path and does not
    // gate the decode resumption.  Allow at most 1 extra fetch.
    int reqs_added = lbus.n_reqs - reqs_before;
    CHECK_TRUE("soft-hit avoids target refetch", reqs_added <= 1);
    return true;
}

// Scenario 4: m68k_core generates pred_redirect when a branch/RTS leaves the
// dispatch register.  The visible fall-through decoder may be on phase 0 of a
// multi-uop instruction and therefore report pd_consumed=0.  if_stage still
// has to redirect immediately; otherwise the fall-through phase can leak.
static bool test_pred_redirect_without_consume() {
    reset();
    lbus.store.clear();
    lbus.fault_lines.clear();

    seed_line(0x40800000, 0x11111111, 0x22222222, 0x33333333, 0x44444444);
    seed_line(0x40800010, 0x55555555, 0x66666666, 0x77777777, 0x88888888);

    CHECK_TRUE("pd_valid cold", wait_for_pd_valid(50) >= 0);

    // Move to 0x40800004 so the target remains in the resident l0 line.
    dut->pd_consumed = 4;
    tick();
    dut->pd_consumed = 0;
    CHECK_EQ("pc before no-consume redirect", dut->pd_pc, 0x40800004);

    int reqs_before = lbus.n_reqs;
    dut->pred_redirect = 1;
    dut->pred_target   = 0x40800000;
    dut->pd_consumed   = 0;
    tick();
    dut->pred_redirect = 0;
    dut->pred_target   = 0;

    CHECK_EQ("pc redirected despite pd_consumed=0", dut->pd_pc, 0x40800000);
    CHECK_TRUE("target still valid after same-line redirect", dut->pd_valid);
    CHECK_TRUE("same-line redirect avoids target refetch",
               (lbus.n_reqs - reqs_before) <= 1);
    return true;
}

static bool test_fetch_fault_metadata() {
    reset();
    lbus.store.clear();
    lbus.fault_lines.clear();

    uint32_t base = 0x40800000;
    seed_line(base, 0xDEAD0000, 0xDEAD0001, 0xDEAD0002, 0xDEAD0003);
    lbus.fault_lines[base] = true;

    CHECK_TRUE("pd_valid for faulted line", wait_for_pd_valid(50) >= 0);
    CHECK_EQ("faulted line pc", dut->pd_pc, base);
    CHECK_EQ("faulted line pd_fault", dut->pd_fault, 1);
    CHECK_EQ("faulted line next fault clear", dut->pd_next_fault, 0);
    CHECK_EQ("faulted line bytes still forwarded", dut->pd_buf.at(3), 0xDEAD0000);
    return true;
}

// ─── main ──────────────────────────────────────────────────────────────
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { std::printf("[PASS] %s\n", #fn); n_pass++; } \
    else    { std::printf("[FAIL] %s\n", #fn); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vif_stage;

    RUN(test_cold_sequential_fetch);
    RUN(test_mid_seq_branch_mispredict);
    RUN(test_backward_branch_soft_hit);
    RUN(test_pred_redirect_without_consume);
    RUN(test_fetch_fault_metadata);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
