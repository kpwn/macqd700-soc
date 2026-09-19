// tb_irq_agg.cpp — Verilator unit testbench for irq_agg.v
//
// Exercises the priority encoder and NMI edge-detect latch in isolation.
// Build via: make tb-irq-agg
//
// Scenarios:
//   1. test_reset_no_ipl               — after reset, ipl = 0
//   2. test_level1_via1                — via1_irq high → ipl = 1
//   3. test_priority_highest_wins      — multiple lines → highest level
//   4. test_nmi_edge_latch             — rising edge of nmi_edge latches
//   5. test_nmi_ack_clears             — ipl_ack clears the NMI latch
//   6. test_nmi_level_held             — nmi_edge held high doesn't retake
//   7. test_level_preempts_below_mask  — level 3 overrides level 1
//   8. test_active_high_source_map      — every represented source level
//   9. test_ack_does_not_clear_levels   — peripheral latches own clearing
//  10. test_sr_mask_expectations        — commit-side SR.I take contract
//  11. test_priority_fallback_cascade   — stable fallback as sources clear
//  12. test_nmi_returns_to_lower_level  — NMI ack exposes lower source
//  13. test_nmi_rise_wins_over_ack      — simultaneous new NMI is retained
//
// Expected output:
//   [PASS] test_reset_no_ipl
//   [PASS] test_level1_via1
//   [PASS] test_priority_highest_wins
//   [PASS] test_nmi_edge_latch
//   [PASS] test_nmi_ack_clears
//   [PASS] test_nmi_level_held
//   [PASS] test_level_preempts_below_mask
//   [PASS] test_active_high_source_map
//   [PASS] test_ack_does_not_clear_levels
//   [PASS] test_sr_mask_expectations
//   [PASS] test_priority_fallback_cascade
//   [PASS] test_nmi_returns_to_lower_level
//   [PASS] test_nmi_rise_wins_over_ack
//   All 13 scenarios PASSED.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Virq_agg.h"

static Virq_agg* dut       = nullptr;
static uint64_t  sim_time  = 0;
static int       n_pass    = 0;
static int       n_fail    = 0;

// ── Clock helpers ──────────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

// Clear all peripheral + handshake inputs to a clean state.
static void clear_inputs() {
    dut->via1_irq   = 0;
    dut->via2_irq   = 0;
    dut->scsi_irq   = 0;
    dut->scc_irq    = 0;
    dut->snd_irq    = 0;
    dut->rsvd_irq6  = 0;
    dut->nmi_edge   = 0;
    dut->ipl_ack    = 0;
}

static void reset() {
    dut->rst = 1;
    clear_inputs();
    tick(); tick();
    dut->rst = 0;
    tick();
}

// ── Assertion helpers ──────────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    if ((uint32_t)(got) != (uint32_t)(exp)) { \
        printf("  FAIL %s: got %u, expected %u\n", \
               name, (uint32_t)(got), (uint32_t)(exp)); \
        return false; \
    } \
} while(0)

static void set_level(unsigned level, bool active) {
    switch (level) {
    case 1: dut->via1_irq  = active ? 1 : 0; break;
    case 2: dut->via2_irq  = active ? 1 : 0; break;
    case 3: dut->scsi_irq  = active ? 1 : 0; break;
    case 4: dut->scc_irq   = active ? 1 : 0; break;
    case 5: dut->snd_irq   = active ? 1 : 0; break;
    case 6: dut->rsvd_irq6 = active ? 1 : 0; break;
    default: break;
    }
}

static bool cpu_would_take_irq(unsigned ipl, unsigned sr_i_mask) {
    return (ipl != 0) && ((ipl > sr_i_mask) || (ipl == 7));
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 1 — after reset with all lines low, ipl = 0
// ══════════════════════════════════════════════════════════════════════
static bool test_reset_no_ipl() {
    reset();
    CHECK_EQ("ipl after reset", dut->ipl, 0);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 2 — via1_irq only → ipl = 1 (level-triggered, combinational)
// ══════════════════════════════════════════════════════════════════════
static bool test_level1_via1() {
    reset();
    dut->via1_irq = 1;
    dut->eval();
    CHECK_EQ("via1 only", dut->ipl, 1);
    // De-assert → back to 0
    dut->via1_irq = 0;
    dut->eval();
    CHECK_EQ("via1 released", dut->ipl, 0);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 3 — multiple lines active, highest level wins
// ══════════════════════════════════════════════════════════════════════
static bool test_priority_highest_wins() {
    reset();

    // VIA1 (1) + VIA2 (2) + SCSI (3) → expect 3
    dut->via1_irq = 1;
    dut->via2_irq = 1;
    dut->scsi_irq = 1;
    dut->eval();
    CHECK_EQ("levels 1+2+3", dut->ipl, 3);

    // Add SCC (4) → expect 4
    dut->scc_irq = 1;
    dut->eval();
    CHECK_EQ("levels 1+2+3+4", dut->ipl, 4);

    // Add sound (5) → expect 5
    dut->snd_irq = 1;
    dut->eval();
    CHECK_EQ("levels 1..5", dut->ipl, 5);

    // Add rsvd (6) → expect 6
    dut->rsvd_irq6 = 1;
    dut->eval();
    CHECK_EQ("levels 1..6", dut->ipl, 6);

    // Release all
    clear_inputs();
    dut->eval();
    CHECK_EQ("all released", dut->ipl, 0);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 4 — NMI edge latch: rising edge of nmi_edge → ipl = 7
// pulsing nmi_edge (high for 1 cycle then low) still leaves ipl = 7
// because the internal latch holds.
// ══════════════════════════════════════════════════════════════════════
static bool test_nmi_edge_latch() {
    reset();

    // First — with nmi_edge low for one reset cycle, ipl should stay 0.
    CHECK_EQ("nmi low pre-edge", dut->ipl, 0);

    // Raise nmi_edge; the rising edge sets the latch on the next posedge.
    dut->nmi_edge = 1;
    tick();               // latch sets here
    dut->eval();
    CHECK_EQ("nmi_pending after rise", dut->ipl, 7);

    // De-assert nmi_edge — latch should still hold (level-7 persists).
    dut->nmi_edge = 0;
    tick();
    dut->eval();
    CHECK_EQ("nmi held by latch", dut->ipl, 7);

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 5 — ipl_ack clears the NMI latch, so without a fresh edge
// ipl drops back to whatever lower levels are asserted (0 here).
// ══════════════════════════════════════════════════════════════════════
static bool test_nmi_ack_clears() {
    reset();

    // Arm the NMI latch
    dut->nmi_edge = 1;
    tick();
    dut->nmi_edge = 0;
    dut->eval();
    CHECK_EQ("nmi latched", dut->ipl, 7);

    // CPU acks at ipl=7 → latch clears next cycle
    dut->ipl_ack = 1;
    tick();
    dut->ipl_ack = 0;
    dut->eval();
    CHECK_EQ("post-ack ipl", dut->ipl, 0);

    // Subsequent held-high nmi_edge (no new rising edge) must NOT
    // re-raise the latch — NMI requires a fresh edge.
    dut->nmi_edge = 0;   // ensure it's low first
    tick();
    dut->nmi_edge = 1;   // this IS a rising edge vs previous 0
    tick();
    dut->eval();
    CHECK_EQ("fresh edge re-arms", dut->ipl, 7);

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 6 — if nmi_edge has been continuously high (never went low)
// the latch is armed ONCE on the first rising edge.  After ack, a
// continuously-high nmi_edge does NOT retrigger — only a 0→1 transition.
// ══════════════════════════════════════════════════════════════════════
static bool test_nmi_level_held() {
    reset();

    // Drive nmi_edge high at reset release.
    dut->nmi_edge = 1;
    tick();              // first rising edge, sets latch
    dut->eval();
    CHECK_EQ("first edge latches", dut->ipl, 7);

    // Ack the NMI.
    dut->ipl_ack = 1;
    tick();
    dut->ipl_ack = 0;
    dut->eval();
    CHECK_EQ("ack clears", dut->ipl, 0);

    // Hold nmi_edge high for several cycles — must NOT retrigger.
    for (int i = 0; i < 5; i++) {
        tick();
    }
    dut->eval();
    CHECK_EQ("held-high no retrigger", dut->ipl, 0);

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 7 — dynamic preemption: a level-1 IRQ asserted, then a
// level-3 IRQ arrives — ipl output immediately tracks the higher.
// When level 3 releases, ipl drops back to 1 (not 0).
// ══════════════════════════════════════════════════════════════════════
static bool test_level_preempts_below_mask() {
    reset();

    dut->via1_irq = 1;
    dut->eval();
    CHECK_EQ("level 1 asserted", dut->ipl, 1);

    // SCSI arrives at level 3
    dut->scsi_irq = 1;
    dut->eval();
    CHECK_EQ("level 3 preempts", dut->ipl, 3);

    // SCSI done
    dut->scsi_irq = 0;
    dut->eval();
    CHECK_EQ("back to level 1", dut->ipl, 1);

    dut->via1_irq = 0;
    dut->eval();
    CHECK_EQ("all cleared", dut->ipl, 0);

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 8 — active-high source map.  Each currently represented
// peripheral-style source drives exactly its assigned IPL while high:
// VIA1=L1, VIA2=L2, SCSI=L3, SCC=L4, ASC/sound=L5, DAFB/DMA-class
// reserved source=L6.  There is no DAFB video IRQ port today; L6 is the
// only current non-ASC high peripheral-style slot.
// ══════════════════════════════════════════════════════════════════════
static bool test_active_high_source_map() {
    reset();

    for (unsigned level = 1; level <= 6; level++) {
        clear_inputs();
        set_level(level, true);
        dut->eval();
        CHECK_EQ("source level asserted", dut->ipl, level);

        set_level(level, false);
        dut->eval();
        CHECK_EQ("source level released", dut->ipl, 0);
    }

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 9 — ipl_ack is not a peripheral clear.  It is a CPU-side take
// pulse; level IRQs remain visible until the owning peripheral deasserts.
// ══════════════════════════════════════════════════════════════════════
static bool test_ack_does_not_clear_levels() {
    reset();

    for (unsigned level = 1; level <= 6; level++) {
        clear_inputs();
        set_level(level, true);
        dut->eval();
        CHECK_EQ("level before ack", dut->ipl, level);

        dut->ipl_ack = 1;
        tick();
        dut->ipl_ack = 0;
        dut->eval();
        CHECK_EQ("level persists through ack", dut->ipl, level);

        set_level(level, false);
        dut->eval();
        CHECK_EQ("level clears only when source drops", dut->ipl, 0);
    }

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 10 — masking expectations.  irq_agg emits raw highest IPL;
// commit owns SR.I masking.  A same-or-lower non-NMI level is masked,
// a higher level is taken, and NMI bypasses SR.I.
// ══════════════════════════════════════════════════════════════════════
static bool test_sr_mask_expectations() {
    reset();

    dut->scsi_irq = 1;
    dut->eval();
    CHECK_EQ("raw level 3 with SR.I mask elsewhere", dut->ipl, 3);
    CHECK_EQ("SR.I=3 masks level 3", cpu_would_take_irq(dut->ipl, 3), 0);
    CHECK_EQ("SR.I=2 admits level 3", cpu_would_take_irq(dut->ipl, 2), 1);

    dut->snd_irq = 1;
    dut->eval();
    CHECK_EQ("raw level 5 overrides level 3", dut->ipl, 5);
    CHECK_EQ("SR.I=5 masks level 5", cpu_would_take_irq(dut->ipl, 5), 0);
    CHECK_EQ("SR.I=4 admits level 5", cpu_would_take_irq(dut->ipl, 4), 1);

    dut->nmi_edge = 1;
    tick();
    dut->eval();
    CHECK_EQ("raw NMI", dut->ipl, 7);
    CHECK_EQ("SR.I=7 still admits NMI", cpu_would_take_irq(dut->ipl, 7), 1);

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 11 — stable multiple-source fallback.  With all level sources
// asserted, the highest wins.  Clearing each highest source exposes the
// next one without requiring a clock edge or an ack.
// ══════════════════════════════════════════════════════════════════════
static bool test_priority_fallback_cascade() {
    reset();

    for (unsigned level = 1; level <= 6; level++)
        set_level(level, true);
    dut->eval();
    CHECK_EQ("all represented sources", dut->ipl, 6);

    for (unsigned level = 6; level >= 2; level--) {
        set_level(level, false);
        dut->eval();
        CHECK_EQ("fallback to next lower source", dut->ipl, level - 1);
    }

    dut->via1_irq = 0;
    dut->eval();
    CHECK_EQ("fallback cascade fully clear", dut->ipl, 0);

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 12 — NMI priority and clearing.  A level-7 latch overrides
// all level sources; acknowledging it exposes the highest still-active
// level IRQ instead of dropping to idle.
// ══════════════════════════════════════════════════════════════════════
static bool test_nmi_returns_to_lower_level() {
    reset();

    dut->via2_irq = 1;
    dut->snd_irq = 1;
    dut->eval();
    CHECK_EQ("lower active before NMI", dut->ipl, 5);

    dut->nmi_edge = 1;
    tick();
    dut->nmi_edge = 0;
    dut->eval();
    CHECK_EQ("NMI overrides lower levels", dut->ipl, 7);

    dut->ipl_ack = 1;
    tick();
    dut->ipl_ack = 0;
    dut->eval();
    CHECK_EQ("post-NMI ack returns to highest lower", dut->ipl, 5);

    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Scenario 13 — if a fresh NMI edge arrives on the same cycle as an ack,
// the set wins so the new NMI is not lost.
// ══════════════════════════════════════════════════════════════════════
static bool test_nmi_rise_wins_over_ack() {
    reset();

    dut->nmi_edge = 1;
    tick();
    dut->nmi_edge = 0;
    dut->eval();
    CHECK_EQ("initial NMI latched", dut->ipl, 7);

    tick();              // sample low so the next high is a fresh edge
    dut->ipl_ack = 1;
    dut->nmi_edge = 1;
    tick();
    dut->ipl_ack = 0;
    dut->nmi_edge = 0;
    dut->eval();
    CHECK_EQ("new NMI retained across ack", dut->ipl, 7);

    return true;
}

// ── Runner ─────────────────────────────────────────────────────────────
static void run(const char* name, bool (*fn)()) {
    bool ok = fn();
    if (ok) { printf("  [PASS] %s\n", name); n_pass++; }
    else    { printf("  [FAIL] %s\n", name); n_fail++; }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Virq_agg;

    printf("tb_irq_agg: running scenarios\n");
    run("test_reset_no_ipl",              test_reset_no_ipl);
    run("test_level1_via1",               test_level1_via1);
    run("test_priority_highest_wins",     test_priority_highest_wins);
    run("test_nmi_edge_latch",            test_nmi_edge_latch);
    run("test_nmi_ack_clears",            test_nmi_ack_clears);
    run("test_nmi_level_held",            test_nmi_level_held);
    run("test_level_preempts_below_mask", test_level_preempts_below_mask);
    run("test_active_high_source_map",    test_active_high_source_map);
    run("test_ack_does_not_clear_levels", test_ack_does_not_clear_levels);
    run("test_sr_mask_expectations",      test_sr_mask_expectations);
    run("test_priority_fallback_cascade", test_priority_fallback_cascade);
    run("test_nmi_returns_to_lower_level", test_nmi_returns_to_lower_level);
    run("test_nmi_rise_wins_over_ack",    test_nmi_rise_wins_over_ack);

    printf("\n");
    if (n_fail == 0) {
        printf("All %d scenarios PASSED.\n", n_pass);
    } else {
        printf("%d scenarios FAILED (of %d).\n", n_fail, n_pass + n_fail);
    }

    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
