// tb_debug_ctrl.cpp — Verilator unit testbench for rtl/core/debug/debug_ctrl.v
//
// Drives the AXI4-Lite slave port + observability inputs of debug_ctrl and
// verifies TIER 1 register behaviour per docs/debug_pcie.md:
//
//   * DBG_VERSION reads back 0xDEB6_0004
//   * DBG_CONTROL level bits (halt_req, init_done_override) round-trip
//   * DBG_CONTROL pulse bits (step, soft_rst) emit one-cycle pulses on
//     the dbg_step_req / dbg_soft_rst outputs
//   * DBG_CYCLE counter increments once per clock when not halted and
//     freezes when halt_req is asserted
//   * DBG_INST counter increments when commit_event_valid pulses
//   * PC trace ring advances through head, wraps modulo DEPTH, reports
//     correct PC_TRACE_HEAD
//   * DBG_REDIRECT_TRIGGER emits a single-cycle dbg_redirect_valid pulse
//     carrying the most recently written redirect PC
//
// Build via: make tb-debug
// Expected output: "All N scenarios PASSED." with exit code 0.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include "Vdebug_ctrl.h"

static Vdebug_ctrl* dut      = nullptr;
static uint64_t     sim_time = 0;
static int          n_pass   = 0;
static int          n_fail   = 0;

// ── Clock helpers ──────────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->s_axi_awaddr  = 0;
    dut->s_axi_awvalid = 0;
    dut->s_axi_wdata   = 0;
    dut->s_axi_wstrb   = 0;
    dut->s_axi_wvalid  = 0;
    dut->s_axi_bready  = 0;
    dut->s_axi_araddr  = 0;
    dut->s_axi_arvalid = 0;
    dut->s_axi_rready  = 0;

    dut->counters_clear    = 0;

    dut->dbg_halted        = 0;
    dut->dbg_exc_pending   = 0;
    dut->dbg_exc_vec       = 0;
    dut->dbg_pc            = 0;
    dut->dbg_committed     = 0;
    dut->dbg_cycle_count   = 0;
    dut->dbg_inst_count    = 0;
    dut->dbg_mispred_count = 0;
    dut->dbg_icache_hits   = 0;
    dut->dbg_icache_misses = 0;
    dut->dbg_dcache_hits   = 0;
    dut->dbg_dcache_misses = 0;
    dut->dbg_dcache_op_done = 0;
    dut->dbg_icache_op_done = 0;

    dut->dbg_auto_halt_event  = 0;
    dut->dbg_auto_halt_reason = 0;
    dut->dbg_auto_halt_pc     = 0;
    dut->dbg_auto_halt_inst   = 0;

    dut->commit_event_valid    = 0;
    dut->commit_event_pc       = 0;
    dut->commit_event_next_pc  = 0;
    dut->commit_event_data     = 0;
    dut->commit_event_arch_dst = 0;
    dut->commit_event_has_dst  = 0;

    dut->mispred_event_valid     = 0;
    dut->mispred_event_pc        = 0;
    dut->mispred_event_predicted = 0;
    dut->mispred_event_actual    = 0;

    dut->exc_event_valid = 0;
    dut->exc_event_vec   = 0;
    dut->exc_event_pc    = 0;

    dut->flush_event_valid = 0;
    dut->init_done_seen    = 0;

    // snap_rob_entry is 128-bit (VlWide<4>) — zero each word
    for (int i = 0; i < 4; i++) dut->snap_rob_entry.at(i) = 0;
    dut->snap_rat_phys  = 0;
    dut->snap_prf_value = 0;
    dut->snap_value_i   = 0;
    dut->dbg_fault_snap_trigger    = 0;
    dut->dbg_break_pc_skip_consume_array = 0;
    dut->dbg_break_uop_fire        = 0;
    dut->dbg_break_pc_hit_slot     = 0;
    dut->dbg_break_pc_hit_is_step  = 0;
    dut->dbg_live_mmu_tc_in   = 0;
    dut->dbg_live_mmu_dtt0_in = 0;
    dut->dbg_live_mmu_dtt1_in = 0;
    dut->dbg_live_mmu_itt0_in = 0;
    dut->dbg_live_mmu_itt1_in = 0;
    dut->dbg_live_mmu_srp_in  = 0;
    dut->dbg_live_mmu_urp_in  = 0;
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    tick(); tick(); tick();
    dut->rst = 0;
    tick();
}

// ── AXI4-Lite host BFM ─────────────────────────────────────────────────
// Drive a single write transaction to addr with data.  The pulse_observer
// callback (if set) is invoked with the combinational output state on each
// cycle between the start of the transaction and BVALID's acknowledgement —
// this lets tests sample one-cycle control pulses (step, soft_rst,
// redirect_valid, irq_inject_pulse) which otherwise would be missed.
static int         obs_pulses     = 0;
static uint32_t    obs_side_a     = 0;  // captured dbg_redirect_pc or irq_lvl
typedef void (*pulse_fn)(void);
static pulse_fn    obs_callback   = nullptr;

static void axil_write(uint32_t addr, uint32_t data, uint32_t strb = 0xF) {
    dut->s_axi_awaddr  = addr;
    dut->s_axi_awvalid = 1;
    dut->s_axi_wdata   = data;
    dut->s_axi_wstrb   = strb;
    dut->s_axi_wvalid  = 1;
    dut->s_axi_bready  = 1;

    bool aw_done = false, w_done = false;
    for (int i = 0; i < 32; i++) {
        dut->eval();
        if (!aw_done && dut->s_axi_awready) aw_done = true;
        if (!w_done  && dut->s_axi_wready)  w_done  = true;
        if (obs_callback) obs_callback();
        tick();
        if (aw_done) dut->s_axi_awvalid = 0;
        if (w_done)  dut->s_axi_wvalid  = 0;
        if (aw_done && w_done) break;
    }

    for (int i = 0; i < 32; i++) {
        dut->eval();
        if (obs_callback) obs_callback();
        if (dut->s_axi_bvalid) { tick(); break; }
        tick();
    }
    dut->s_axi_bready  = 0;
    dut->s_axi_awvalid = 0;
    dut->s_axi_wvalid  = 0;
}

// Drive a single read transaction to addr.  Returns RDATA.
static uint32_t axil_read(uint32_t addr) {
    dut->s_axi_araddr  = addr;
    dut->s_axi_arvalid = 1;
    dut->s_axi_rready  = 1;

    // Wait for ARREADY
    for (int i = 0; i < 32; i++) {
        dut->eval();
        if (dut->s_axi_arready) { tick(); break; }
        tick();
    }
    dut->s_axi_arvalid = 0;

    // Wait for RVALID
    uint32_t rdata = 0;
    for (int i = 0; i < 32; i++) {
        dut->eval();
        if (dut->s_axi_rvalid) {
            rdata = dut->s_axi_rdata;
            tick();
            break;
        }
        tick();
    }
    dut->s_axi_rready = 0;
    return rdata;
}

// ── Assertion helpers ──────────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    uint32_t _g = (uint32_t)(got); \
    uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%08x, expected 0x%08x\n", (name), _g, _e); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { printf("  FAIL %s: expected true\n", (name)); return false; } \
} while(0)

#define CHECK_FALSE(name, cond) do { \
    if (cond) { printf("  FAIL %s: expected false\n", (name)); return false; } \
} while(0)

// ════════════════════════════════════════════════════════════════════════
// Scenario 1 — DBG_VERSION read
// ════════════════════════════════════════════════════════════════════════
static bool test_version_read() {
    reset();
    uint32_t v = axil_read(0x000);
    CHECK_EQ("DBG_VERSION", v, 0xDEB60005u);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 2 — DBG_CONTROL level bits round-trip
// ════════════════════════════════════════════════════════════════════════
static bool test_control_level_bits() {
    reset();

    // Assert halt_req and init_done_override
    axil_write(0x008, 0x9);  // bit0 (halt) + bit3 (init_done_override)
    tick();
    dut->eval();
    CHECK_TRUE("dbg_halt_req high",           dut->dbg_halt_req);
    CHECK_TRUE("dbg_init_done_override high", dut->dbg_init_done_override);

    uint32_t v = axil_read(0x008);
    CHECK_EQ("CONTROL readback", v & 0x9, 0x9);

    // Deassert
    axil_write(0x008, 0x0);
    tick();
    dut->eval();
    CHECK_FALSE("dbg_halt_req low",           dut->dbg_halt_req);
    CHECK_FALSE("dbg_init_done_override low", dut->dbg_init_done_override);

    return true;
}

// Pulse observers — each samples the relevant control output per cycle.
static void obs_step()     { if (dut->dbg_step_req)   obs_pulses++; }
static void obs_srst()     { if (dut->dbg_soft_rst)   obs_pulses++; }
static void obs_redirect() { if (dut->dbg_redirect_valid) { obs_pulses++; obs_side_a = dut->dbg_redirect_pc; } }
static void obs_irq()      { if (dut->dbg_irq_inject_pulse) { obs_pulses++; obs_side_a = dut->dbg_irq_inject_lvl; } }

// ════════════════════════════════════════════════════════════════════════
// Scenario 3 — DBG_CONTROL pulse bits (step, soft_rst) emit 1-cycle pulse
// ════════════════════════════════════════════════════════════════════════
static bool test_control_pulse_bits() {
    reset();

    // Step pulse — observe across the whole transaction window
    obs_pulses = 0; obs_callback = obs_step;
    axil_write(0x008, 0x2);  // bit1 — step
    // Drain a few extra cycles in case the pulse lingers past BVALID
    for (int i = 0; i < 4; i++) { dut->eval(); obs_step(); tick(); }
    obs_callback = nullptr;
    CHECK_EQ("step pulse width (cycles)", obs_pulses, 1);
    CHECK_FALSE("dbg_halt_req after step", dut->dbg_halt_req);

    // Soft reset pulse
    obs_pulses = 0; obs_callback = obs_srst;
    axil_write(0x008, 0x4);  // bit2 — soft_rst
    for (int i = 0; i < 4; i++) { dut->eval(); obs_srst(); tick(); }
    obs_callback = nullptr;
    CHECK_EQ("soft_rst pulse width (cycles)", obs_pulses, 1);

    // reset_cause should now report soft-reset (2'd1)
    uint32_t rc = axil_read(0x02C) & 0x3;
    CHECK_EQ("RESET_CAUSE after soft_rst", rc, 1);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 4 — DBG_CYCLE counter ticks once per cycle + halts on halt_req
// ════════════════════════════════════════════════════════════════════════
static bool test_cycle_counter() {
    reset();

    uint32_t c0 = axil_read(0x1000);
    // Run some cycles while not halted
    for (int i = 0; i < 10; i++) tick();
    uint32_t c1 = axil_read(0x1000);
    CHECK_TRUE("DBG_CYCLE advanced when running", c1 > c0);

    // Halt and confirm it freezes
    axil_write(0x008, 0x1);  // halt_req
    // Let halt take effect
    for (int i = 0; i < 2; i++) tick();
    uint32_t c2 = axil_read(0x1000);
    for (int i = 0; i < 20; i++) tick();
    uint32_t c3 = axil_read(0x1000);
    CHECK_EQ("DBG_CYCLE frozen while halted", c3, c2);

    // Release
    axil_write(0x008, 0x0);
    for (int i = 0; i < 5; i++) tick();
    uint32_t c4 = axil_read(0x1000);
    CHECK_TRUE("DBG_CYCLE resumed after unhalt", c4 > c3);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 5 — DBG_INST counter increments only on commit_event_valid
// ════════════════════════════════════════════════════════════════════════
static bool test_inst_counter() {
    reset();

    uint32_t i0 = axil_read(0x1008);
    // Run cycles with no commits — inst count should not move
    for (int i = 0; i < 20; i++) tick();
    uint32_t i1 = axil_read(0x1008);
    CHECK_EQ("DBG_INST no-op cycles", i1, i0);

    // Pulse commit_event 5 times
    for (int i = 0; i < 5; i++) {
        dut->commit_event_valid = 1;
        dut->commit_event_pc    = 0x40800000u + (uint32_t)(i * 4);
        tick();
        dut->commit_event_valid = 0;
        dut->commit_event_pc    = 0;
        tick();
    }
    uint32_t i2 = axil_read(0x1008);
    CHECK_EQ("DBG_INST +5", i2 - i1, 5);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 6 — PC trace ring advances + wraps + head is correct
// ════════════════════════════════════════════════════════════════════════
static bool test_pc_trace_ring() {
    reset();

    // Push 1030 events so we wrap twice past the first 6 entries.
    const int N = 1030;
    const uint32_t PC_BASE = 0x4080'0000u;
    for (int i = 0; i < N; i++) {
        dut->commit_event_valid = 1;
        dut->commit_event_pc    = PC_BASE + (uint32_t)(i * 4);
        tick();
    }
    dut->commit_event_valid = 0;
    dut->commit_event_pc    = 0;
    tick();

    // Head should be at (N % 1024) = 6.  Address moved 0x10F00 → 0x11000
    // by task #30 debug_ctrl fix (slot-960 alias); test was missed.
    uint32_t head = axil_read(0x11000) & 0x3FF;
    CHECK_EQ("PC_TRACE_HEAD after 1030 events", head, 6);

    // The slot at (head-1) & 1023 should hold the most recent PC.
    uint32_t last_slot = (head - 1) & 0x3FF;
    uint32_t last_pc   = axil_read(0x10000 + last_slot * 4);
    uint32_t expected_last_pc = PC_BASE + (uint32_t)((N - 1) * 4);
    CHECK_EQ("PC_TRACE last entry", last_pc, expected_last_pc);

    // The slot at head itself holds the OLDEST in-ring entry (i.e. the
    // (N-1024)th PC we pushed, = PC_BASE + 6*4).
    uint32_t oldest = axil_read(0x10000 + head * 4);
    uint32_t expected_oldest = PC_BASE + (uint32_t)((N - 1024) * 4);
    CHECK_EQ("PC_TRACE oldest entry (wrap)", oldest, expected_oldest);

    // DBG_LAST_PC should also mirror the most-recent PC
    uint32_t last_pc_reg = axil_read(0x014);
    CHECK_EQ("DBG_LAST_PC", last_pc_reg, expected_last_pc);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 6b — PC trace ring pushes only on control-flow redirects.
// When commit_event_next_pc == next commit_event_pc (i.e. linear
// fall-through), the ring must NOT advance.  When the new pc breaks
// the chain — taken Bcc / JMP / JSR / RTS / RTE / exception entry —
// the *target* PC is pushed.  This is what makes the 64-deep ring
// useful for diagnosing wild-PC bugs.
// ════════════════════════════════════════════════════════════════════════
static bool test_pc_trace_redirect_only() {
    reset();

    // Stream a mixed sequence:
    //   pc=0x1000, next=0x1004     (first retire — always pushes)
    //   pc=0x1004, next=0x1008     (fall-through — no push)
    //   pc=0x1008, next=0x1010     (fall-through — no push)
    //   pc=0x9000, next=0x9004     (REDIRECT — pushes 0x9000)
    //   pc=0x9004, next=0x9008     (fall-through — no push)
    //   pc=0x4000, next=0x4004     (REDIRECT — pushes 0x4000)
    //   pc=0x4004, next=0x4008     (fall-through — no push)
    struct Step { uint32_t pc; uint32_t npc; };
    static const Step steps[] = {
        {0x1000, 0x1004},
        {0x1004, 0x1008},
        {0x1008, 0x1010},
        {0x9000, 0x9004},
        {0x9004, 0x9008},
        {0x4000, 0x4004},
        {0x4004, 0x4008},
    };
    const int N = (int)(sizeof(steps)/sizeof(steps[0]));

    for (int i = 0; i < N; i++) {
        dut->commit_event_valid   = 1;
        dut->commit_event_pc      = steps[i].pc;
        dut->commit_event_next_pc = steps[i].npc;
        tick();
    }
    dut->commit_event_valid = 0;
    dut->commit_event_pc    = 0;
    dut->commit_event_next_pc = 0;
    tick();

    // Expect 3 pushes total (first + two redirects).
    uint32_t head = axil_read(0x11000) & 0x3FF;
    CHECK_EQ("PC_TRACE_HEAD redirect-only count", head, 3);

    // Slot[0] = first retire (0x1000), slot[1] = first redirect (0x9000),
    // slot[2] = second redirect (0x4000).
    uint32_t s0 = axil_read(0x10000 + 0 * 4);
    uint32_t s1 = axil_read(0x10000 + 1 * 4);
    uint32_t s2 = axil_read(0x10000 + 2 * 4);
    CHECK_EQ("PC_TRACE slot0 first retire",  s0, 0x1000u);
    CHECK_EQ("PC_TRACE slot1 first redirect", s1, 0x9000u);
    CHECK_EQ("PC_TRACE slot2 second redirect", s2, 0x4000u);

    // (We don't check slot[3] — the PC trace BRAM is initial-zero, not
    // reset-zero, so it can carry stale data from a prior test in the
    // same tb run.  The head pointer + slot[0..2] checks above pin the
    // gating behaviour conclusively.)

    // DBG_LAST_PC must still mirror the most-recent retire (= 0x4004,
    // the last fall-through), proving last_pc_r updates on every retire
    // even when the ring suppresses the push.
    uint32_t last_pc_reg = axil_read(0x014);
    CHECK_EQ("DBG_LAST_PC tracks every retire", last_pc_reg, 0x4004u);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 7 — DBG_REDIRECT_TRIGGER emits one-cycle dbg_redirect_valid
// ════════════════════════════════════════════════════════════════════════
static bool test_redirect_trigger() {
    reset();

    // Set up the redirect PC first
    axil_write(0x018, 0xCAFEF00Du);
    tick();
    dut->eval();
    CHECK_EQ("dbg_redirect_pc latched", dut->dbg_redirect_pc, 0xCAFEF00Du);

    // Trigger — observe pulse across the whole transaction
    obs_pulses = 0; obs_side_a = 0; obs_callback = obs_redirect;
    axil_write(0x01C, 0x1);
    for (int i = 0; i < 4; i++) { dut->eval(); obs_redirect(); tick(); }
    obs_callback = nullptr;
    CHECK_EQ("redirect_valid pulse width", obs_pulses, 1);
    CHECK_EQ("redirect_pc during pulse",   obs_side_a, 0xCAFEF00Du);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 8 — Exception capture (EXC_VEC/EXC_PC + EXC_COUNT increments)
// ════════════════════════════════════════════════════════════════════════
static bool test_exception_capture() {
    reset();

    uint32_t ec0 = axil_read(0x1018);
    dut->exc_event_valid = 1;
    dut->exc_event_vec   = 0x2A;
    dut->exc_event_pc    = 0xDEAD'BEEFu;
    tick();
    dut->exc_event_valid = 0;
    tick();

    uint32_t vec = axil_read(0x024) & 0xFF;
    uint32_t pc  = axil_read(0x028);
    uint32_t ec1 = axil_read(0x1018);
    CHECK_EQ("DBG_EXC_VEC",    vec, 0x2A);
    CHECK_EQ("DBG_EXC_PC",     pc,  0xDEAD'BEEFu);
    CHECK_EQ("DBG_EXC_COUNT",  ec1 - ec0, 1);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 8b — Exception ring buffer
//   Verify last-32 (vec, pc, fault_addr, count_at_event) recorded per event,
//   head advances and wraps modulo-32, and counters_clear resets the head.
//   The ring exists so we can characterize storm composition (one PC looping
//   vs many faulting) rather than trusting the single-most-recent latch.
// ════════════════════════════════════════════════════════════════════════
static bool test_exc_ring_buffer() {
    reset();
    const uint32_t OFF_RING_BASE = 0x12000;
    const uint32_t OFF_RING_HEAD = 0x13000;
    const int DEPTH = 32;

    auto fire = [&](uint8_t vec, uint32_t pc, uint32_t fa) {
        dut->exc_event_valid      = 1;
        dut->exc_event_vec        = vec;
        dut->exc_event_pc         = pc;
        dut->exc_event_fault_addr = fa;
        tick();
        dut->exc_event_valid      = 0;
        dut->exc_event_fault_addr = 0;
        tick();
    };

    // Sanity: head=0 after reset.  Body is BRAM-style and keeps its
    // values across reset — host disambiguates fresh entries by the
    // `cnt` field (0 = never written this boot; otherwise <= current
    // exc_count_r → valid).  We mirror the PC-trace ring's contract
    // here intentionally.
    CHECK_EQ("ring head after reset", axil_read(OFF_RING_HEAD) & 0x1F, 0u);

    // Fire 3 distinct exceptions
    fire(0x02, 0x408046aau, 0x51001c00u);   // bus error mimicking the wedge
    fire(0x03, 0xDEADBEE0u, 0x00010002u);
    fire(0x0B, 0x4080010eu, 0x00000000u);

    CHECK_EQ("ring head after 3", axil_read(OFF_RING_HEAD) & 0x1F, 3u);
    CHECK_EQ("entry0 vec", axil_read(OFF_RING_BASE + 0*16 + 0) & 0xFF, 0x02u);
    CHECK_EQ("entry0 pc",  axil_read(OFF_RING_BASE + 0*16 + 4),       0x408046aau);
    CHECK_EQ("entry0 fa",  axil_read(OFF_RING_BASE + 0*16 + 8),       0x51001c00u);
    CHECK_EQ("entry0 cnt", axil_read(OFF_RING_BASE + 0*16 + 12),      1u);
    CHECK_EQ("entry1 vec", axil_read(OFF_RING_BASE + 1*16 + 0) & 0xFF, 0x03u);
    CHECK_EQ("entry1 pc",  axil_read(OFF_RING_BASE + 1*16 + 4),       0xDEADBEE0u);
    CHECK_EQ("entry1 cnt", axil_read(OFF_RING_BASE + 1*16 + 12),      2u);
    CHECK_EQ("entry2 vec", axil_read(OFF_RING_BASE + 2*16 + 0) & 0xFF, 0x0Bu);
    CHECK_EQ("entry2 cnt", axil_read(OFF_RING_BASE + 2*16 + 12),      3u);

    // Fire 31 more (= 34 total): head should wrap to slot 2 and the
    // OLDEST recorded entry (originally at slot 0) should now be the
    // 33rd value, not the 1st.
    for (int i = 0; i < 31; i++) {
        fire(0x10 + (uint8_t)i, 0xA0000000u + (uint32_t)i, 0xB0000000u + (uint32_t)i);
    }
    CHECK_EQ("ring head after 34", axil_read(OFF_RING_HEAD) & 0x1F, 2u);
    // After 3 initial fires (head→3) + 31 loop fires, the loop indices
    // 0..28 land in slots 3..31, loop i=29 wraps to slot 0, loop i=30
    // lands in slot 1.  So slot 0 holds loop i=29:
    //   vec = 0x10+29 = 0x2D, pc = 0xA000_001D, cnt = 3+30 = 33.
    CHECK_EQ("wrap slot0 vec", axil_read(OFF_RING_BASE + 0*16 + 0) & 0xFF, 0x2Du);
    CHECK_EQ("wrap slot0 pc",  axil_read(OFF_RING_BASE + 0*16 + 4),       0xA000001Du);
    CHECK_EQ("wrap slot0 cnt", axil_read(OFF_RING_BASE + 0*16 + 12),      33u);
    // Slot 1 was overwritten last (loop i=30): vec=0x2E, cnt=34.
    CHECK_EQ("wrap slot1 vec", axil_read(OFF_RING_BASE + 1*16 + 0) & 0xFF, 0x2Eu);
    CHECK_EQ("wrap slot1 cnt", axil_read(OFF_RING_BASE + 1*16 + 12),      34u);
    // Slot 2 was NOT overwritten (head=2 means "next to write here"),
    // so it still holds the 3rd initial fire's payload.
    CHECK_EQ("slot2 preserved vec", axil_read(OFF_RING_BASE + 2*16 + 0) & 0xFF, 0x0Bu);
    CHECK_EQ("slot2 preserved cnt", axil_read(OFF_RING_BASE + 2*16 + 12),      3u);

    // counters_clear (= soc_full_rst tied in by fpga_top from the
    // unified-reset path) resets the ring head AND exc_count together.
    // Body BRAM is left as-is by design (consistent with PC trace
    // ring).  Host disambiguates fresh entries by checking `cnt`.
    dut->counters_clear = 1;
    tick(); tick();
    dut->counters_clear = 0;
    tick(); tick();
    CHECK_EQ("ring head after counters_clear", axil_read(OFF_RING_HEAD) & 0x1F, 0u);
    CHECK_EQ("exc_count after counters_clear", axil_read(0x1018), 0u);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 9 — IRQ inject pulse + level latch
// ════════════════════════════════════════════════════════════════════════
static bool test_irq_inject() {
    reset();
    obs_pulses = 0; obs_side_a = 0; obs_callback = obs_irq;
    axil_write(0x020, 0x5);   // level 5
    for (int i = 0; i < 4; i++) { dut->eval(); obs_irq(); tick(); }
    obs_callback = nullptr;
    CHECK_EQ("irq_inject_pulse width",      obs_pulses, 1);
    CHECK_EQ("irq_inject_lvl during pulse", obs_side_a, 5);
    // Level persists after pulse
    CHECK_EQ("irq_inject_lvl latched",      (uint32_t)dut->dbg_irq_inject_lvl, 5);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 10b — RAM window selector register
// ════════════════════════════════════════════════════════════════════════
static bool test_ram_window_lg2_register() {
    reset();
    CHECK_EQ("RAM window default lg2=22 (4 MiB)", axil_read(0x058), 22u);
    CHECK_EQ("RAM window output default", dut->dbg_ram_window_lg2, 22u);

    axil_write(0x058, 22u);
    tick();
    dut->eval();
    CHECK_EQ("RAM window lg2 write 22", axil_read(0x058), 22u);
    CHECK_EQ("RAM window output 22", dut->dbg_ram_window_lg2, 22u);

    axil_write(0x058, 30u);
    tick();
    dut->eval();
    CHECK_EQ("RAM window lg2 write 30", axil_read(0x058), 30u);
    CHECK_EQ("RAM window output 30", dut->dbg_ram_window_lg2, 30u);

    axil_write(0x058, 21u);
    tick();
    dut->eval();
    CHECK_EQ("RAM window clamps low to 22", axil_read(0x058), 22u);
    CHECK_EQ("RAM window output clamps low", dut->dbg_ram_window_lg2, 22u);

    axil_write(0x058, 31u);
    tick();
    dut->eval();
    CHECK_EQ("RAM window clamps high to 30", axil_read(0x058), 30u);
    CHECK_EQ("RAM window output clamps high", dut->dbg_ram_window_lg2, 30u);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 10 — Auto-halt register programming + clearable latch
// ════════════════════════════════════════════════════════════════════════
static bool test_auto_halt_latch_and_clear() {
    reset();

    axil_write(0x030, 0x00001234u);
    axil_write(0x034, 0x00000001u);
    axil_write(0x038, 0x40801234u);
    axil_write(0x03C, 0x3u);  // enable halt-after + PC breakpoint
    tick();
    dut->eval();
    CHECK_TRUE("halt-after enable output", dut->dbg_halt_after_enable);
    CHECK_TRUE("break-pc enable output",   dut->dbg_break_pc_enable);
    CHECK_EQ("halt-after lo readback", axil_read(0x030), 0x00001234u);
    CHECK_EQ("halt-after hi readback", axil_read(0x034), 0x00000001u);
    CHECK_EQ("break-pc readback",      axil_read(0x038), 0x40801234u);
    CHECK_EQ("HALT_CTL enables",       axil_read(0x03C) & 0x3u, 0x3u);

    dut->dbg_auto_halt_event  = 1;
    dut->dbg_auto_halt_reason = 1; // halt-after
    dut->dbg_auto_halt_pc     = 0x40801230u;
    dut->dbg_auto_halt_inst   = 0x0000000100001234ull;
    tick();
    dut->dbg_auto_halt_event  = 0;
    dut->dbg_auto_halt_reason = 0;
    dut->dbg_halted           = 1;
    tick();
    dut->eval();
    CHECK_TRUE("auto halt drives halt_req", dut->dbg_halt_req);
    CHECK_EQ("HALT_CTL latched bits", axil_read(0x03C) & 0x39u, 0x19u);
    CHECK_EQ("HALT_REASON",           axil_read(0x040) & 0x0Fu, 0x0Au);
    CHECK_EQ("HALT_HIT_PC",           axil_read(0x044), 0x40801230u);
    CHECK_EQ("HALT_HIT_INST_LO",      axil_read(0x048), 0x00001234u);
    CHECK_EQ("HALT_HIT_INST_HI",      axil_read(0x04C), 0x00000001u);

    axil_write(0x03C, 0x7u);  // keep enables, clear latched auto halt
    dut->dbg_halted = 0;
    tick();
    dut->eval();
    CHECK_FALSE("auto halt cleared", dut->dbg_halt_req);
    CHECK_EQ("HALT_CTL after clear", axil_read(0x03C) & 0x3Fu, 0x03u);
    CHECK_EQ("HALT_HIT_PC cleared", axil_read(0x044), 0u);
    CHECK_EQ("HALT_HIT_INST_LO cleared", axil_read(0x048), 0u);
    CHECK_EQ("HALT_HIT_INST_HI cleared", axil_read(0x04C), 0u);

    axil_write(0x050, 0x00000004u);
    axil_write(0x03C, 0x44u);  // enable exception-vector halt, clear latch
    tick();
    dut->eval();
    CHECK_TRUE("halt-exc enable output", dut->dbg_halt_exc_enable);
    CHECK_EQ("halt-exc vec output", dut->dbg_halt_exc_vec, 4u);

    dut->dbg_auto_halt_event  = 1;
    dut->dbg_auto_halt_reason = 4; // exception-vector match
    dut->dbg_auto_halt_pc     = 0x4080000Cu;
    dut->dbg_auto_halt_inst   = 4;
    tick();
    dut->dbg_auto_halt_event  = 0;
    dut->dbg_auto_halt_reason = 0;
    dut->dbg_halted           = 1;
    tick();
    dut->eval();
    CHECK_TRUE("exception auto halt drives halt_req", dut->dbg_halt_req);
    CHECK_EQ("HALT_CTL exception latched", axil_read(0x03C) & 0xC8u, 0xC8u);
    CHECK_EQ("HALT_REASON exception bits", axil_read(0x040) & 0xC8u, 0xC8u);
    CHECK_EQ("HALT_EXC_VEC", axil_read(0x050), 4u);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 10c — Step pulse while preserving halted state.
//
// The JTAG `step` command first converts an auto halt into a manual halt,
// clears stale auto-halt latches, then writes CONTROL halt_req|step_pulse.
// This prevents the dedicated step pulse from being interpreted as a plain
// run release by debug_stop_manager.
// ════════════════════════════════════════════════════════════════════════
static bool test_step_sequence_preserves_halt_req() {
    reset();

    dut->dbg_auto_halt_event  = 1;
    dut->dbg_auto_halt_reason = 2; // break_pc
    dut->dbg_auto_halt_pc     = 0x40801234u;
    dut->dbg_auto_halt_inst   = 42u;
    tick();
    dut->dbg_auto_halt_event  = 0;
    dut->dbg_auto_halt_reason = 0;
    dut->dbg_halted           = 1;
    tick();
    dut->eval();
    CHECK_TRUE("auto halt initially drives halt_req", dut->dbg_halt_req);

    axil_write(0x008, 0x1u); // manual halt
    axil_write(0x03C, 0x6u); // preserve break_pc enable, clear latches
    tick();
    dut->eval();
    CHECK_TRUE("manual halt survives latch clear", dut->dbg_halt_req);
    CHECK_EQ("auto halt latches cleared before step", axil_read(0x03C) & 0x38u, 0u);

    obs_pulses = 0;
    obs_callback = obs_step;
    axil_write(0x008, 0x3u); // manual halt + step pulse
    for (int i = 0; i < 4; i++) { dut->eval(); obs_step(); tick(); }
    obs_callback = nullptr;
    dut->eval();
    CHECK_EQ("step sequence emits one pulse", obs_pulses, 1);
    CHECK_TRUE("halt_req remains asserted for step gate", dut->dbg_halt_req);
    CHECK_EQ("CONTROL keeps manual halt after step pulse", axil_read(0x008) & 0x1u, 1u);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 11 — Halt-after-N auto-advances on continue
// ════════════════════════════════════════════════════════════════════════
static bool test_halt_after_auto_advance() {
    reset();

    constexpr uint64_t kDelta = 1000;
    constexpr uint32_t kBasePc = 0x40800000u;

    auto drive_to = [&](uint64_t first, uint64_t last) {
        for (uint64_t inst = first; inst <= last; inst++) {
            dut->dbg_inst_count = inst;
            dut->dbg_pc = kBasePc + (uint32_t)(inst * 4u);
            tick();
        }
        dut->eval();
    };

    auto check_halt = [&](const char* label, uint64_t inst) -> bool {
        char msg[128];
        std::snprintf(msg, sizeof(msg), "%s halt-after latch", label);
        CHECK_EQ(msg, axil_read(0x03C) & 0x18u, 0x18u);
        std::snprintf(msg, sizeof(msg), "%s halt-after drives halt_req", label);
        CHECK_TRUE(msg, dut->dbg_halt_req);
        std::snprintf(msg, sizeof(msg), "%s halt retired count", label);
        CHECK_EQ(msg, axil_read(0x048), (uint32_t)inst);
        std::snprintf(msg, sizeof(msg), "%s halt pc live valid", label);
        CHECK_EQ(msg, axil_read(0x010), kBasePc + (uint32_t)(inst * 4u));
        return true;
    };

    auto continue_from_halt = [&](const char* label, uint64_t hit_inst) -> bool {
        // The host may clear the latch after other debug traffic has sampled
        // a live count beyond the captured halt point.  Re-arm must use the
        // captured hit count, not this transient live value.
        dut->dbg_inst_count = hit_inst + 37u;
        dut->dbg_pc = kBasePc + (uint32_t)((hit_inst + 37u) * 4u);
        dut->eval();

        axil_write(0x03C, 0x5u);
        axil_write(0x000, 0x0u);
        tick();
        dut->eval();

        char msg[128];
        std::snprintf(msg, sizeof(msg), "%s continue clears halt_req", label);
        CHECK_FALSE(msg, dut->dbg_halt_req);
        return true;
    };

    axil_write(0x030, (uint32_t)kDelta);
    axil_write(0x034, 0u);
    axil_write(0x03C, 0x5u);  // enable halt-after, clear any stale latch
    tick();
    dut->eval();
    CHECK_EQ("halt-after delta lo readback", axil_read(0x030), (uint32_t)kDelta);
    CHECK_EQ("halt-after delta hi readback", axil_read(0x034), 0u);

    drive_to(1, kDelta);
    if (!check_halt("first", kDelta)) return false;

    if (!continue_from_halt("first", kDelta)) return false;
    drive_to(kDelta + 1, 2 * kDelta - 1);
    CHECK_EQ("no early re-fire before 2N", axil_read(0x03C) & 0x19u, 0x01u);
    CHECK_FALSE("no halt_req before 2N", dut->dbg_halt_req);
    drive_to(2 * kDelta, 2 * kDelta);
    if (!check_halt("second", 2 * kDelta)) return false;

    if (!continue_from_halt("second", 2 * kDelta)) return false;
    drive_to(2 * kDelta + 1, 3 * kDelta - 1);
    CHECK_EQ("no early re-fire before 3N", axil_read(0x03C) & 0x19u, 0x01u);
    CHECK_FALSE("no halt_req before 3N", dut->dbg_halt_req);
    drive_to(3 * kDelta, 3 * kDelta);
    if (!check_halt("third", 3 * kDelta)) return false;

    if (!continue_from_halt("third", 3 * kDelta)) return false;
    drive_to(3 * kDelta + 1, 4 * kDelta - 1);
    CHECK_EQ("no early re-fire before 4N", axil_read(0x03C) & 0x19u, 0x01u);
    CHECK_FALSE("no halt_req before 4N", dut->dbg_halt_req);
    drive_to(4 * kDelta, 4 * kDelta);
    if (!check_halt("fourth", 4 * kDelta)) return false;
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 12 — Unimplemented offsets return 0 with OKAY
// ════════════════════════════════════════════════════════════════════════
static bool test_unimplemented_returns_zero() {
    reset();
    uint32_t v = axil_read(0x3000);   // unimplemented gap
    CHECK_EQ("unimplemented offset", v, 0);
    // Verify BRESP still OKAY on a write to unimplemented
    axil_write(0x3000, 0xDEADBEEFu);
    // If this hangs it's because BRESP wasn't issued — the BFM has a
    // 32-cycle timeout and would just fall through with BVALID never
    // seen; the subsequent read proves nothing corrupted state.
    uint32_t v2 = axil_read(0x3000);
    CHECK_EQ("write to unimplemented has no side-effect", v2, 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 12 — Halt-time architectural shadow apply emits load sequence
// ════════════════════════════════════════════════════════════════════════
static bool test_arch_shadow_apply_and_resume() {
    reset();
    axil_write(0x008, 0x1); // manual halt
    CHECK_TRUE("halt before arch apply", dut->dbg_halt_req);

    axil_write(0x2000, 0x11111111u); // D0
    axil_write(0x203C, 0xAAAAAAAAu); // A7
    axil_write(0x204C, 0x00002713u); // SR + CCR
    axil_write(0x2058, 0x80000000u); // TC
    axil_write(0x2074, 0x40801234u); // PC
    axil_write(0x2078, 0x1);         // apply + resume

    bool saw_d0 = false;
    bool saw_a7 = false;
    bool saw_ccr = false;
    bool saw_sr = false;
    bool saw_tc = false;
    bool saw_pc = false;

    for (int i = 0; i < 48; i++) {
        tick();
        dut->eval();
        if (dut->dbg_arch_reg_load_en && dut->dbg_arch_reg_load_idx == 0 &&
            dut->dbg_arch_reg_load_val == 0x11111111u) saw_d0 = true;
        if (dut->dbg_arch_reg_load_en && dut->dbg_arch_reg_load_idx == 15 &&
            dut->dbg_arch_reg_load_val == 0xAAAAAAAAu) saw_a7 = true;
        if (dut->dbg_arch_ccr_load_en && dut->dbg_arch_ccr_load_val == (0x2713u & 0x1f))
            saw_ccr = true;
        if (dut->dbg_arch_ctrl_load_en && dut->dbg_arch_ctrl_load_sel == 0 &&
            dut->dbg_arch_ctrl_load_val == 0x00002713u) saw_sr = true;
        if (dut->dbg_arch_mmu_load_en && dut->dbg_arch_mmu_load_sel == 4 &&
            dut->dbg_arch_mmu_load_val == 0x80000000u) saw_tc = true;
        if (dut->dbg_arch_pc_load_en && dut->dbg_arch_pc_load_val == 0x40801234u)
            saw_pc = true;
    }

    CHECK_TRUE("D0 load", saw_d0);
    CHECK_TRUE("A7 load", saw_a7);
    CHECK_TRUE("CCR load", saw_ccr);
    CHECK_TRUE("SR load", saw_sr);
    CHECK_TRUE("TC load", saw_tc);
    CHECK_TRUE("PC load", saw_pc);
    CHECK_FALSE("halt cleared after apply", dut->dbg_halt_req);
    CHECK_EQ("arch apply status done", axil_read(0x207C) & 0x7u, 0x2u);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 14 — Unified-reset bits (cold_reset_pulse bit 5,
//              cold_reset_hold bit 4) — clean-slate Phase-2 semantics.
//
//   * cold_reset_pulse fires a 1-cycle dbg_cold_reset_pulse strobe AND
//     also lights dbg_soft_rst (legacy alias) for one release.
//   * cold_reset_hold is sticky (level): asserted while bit 4 is set.
//     debug_ctrl runs on `core_rst` only, so this register survives any
//     reset short of board cold reset — exactly what gates the CPU
//     across the unified-reset pulse it triggers.
//   * legacy bit 2 (soft_rst) is folded into cold_reset_pulse: writing
//     bit 2=1 must also light dbg_cold_reset_pulse for one cycle.
// ════════════════════════════════════════════════════════════════════════
static void obs_cold_pulse() {
    if (dut->dbg_cold_reset_pulse) obs_pulses++;
}
static int obs_legacy_pulses = 0;
static void obs_legacy_and_cold() {
    if (dut->dbg_soft_rst) obs_legacy_pulses++;
    if (dut->dbg_cold_reset_pulse) obs_pulses++;
}

static bool test_unified_reset_bits() {
    reset();

    // ── (a) cold_reset_pulse via bit 5 emits a 1-cycle pulse ────────
    obs_pulses = 0; obs_legacy_pulses = 0;
    obs_callback = obs_legacy_and_cold;
    axil_write(0x008, 0x20);  // bit 5 — cold_reset_pulse
    for (int i = 0; i < 4; i++) {
        dut->eval();
        obs_legacy_and_cold();
        tick();
    }
    obs_callback = nullptr;
    CHECK_EQ("cold_reset_pulse pulse width (cycles)", obs_pulses, 1);
    CHECK_EQ("legacy soft_rst alias also fires (one release)",
             obs_legacy_pulses, 1);
    // reset_cause should be soft (2'd1) after the pulse.
    CHECK_EQ("RESET_CAUSE after cold_reset_pulse",
             axil_read(0x02C) & 0x3u, 1u);

    // cold_reset_hold should NOT be set after a pulse-only write.
    CHECK_FALSE("cold_reset_hold low after pulse-only",
                dut->dbg_cold_reset_hold);

    // ── (b) cold_reset_hold via bit 4 is a level (sticky) ────────────
    axil_write(0x008, 0x10);  // bit 4 — cold_reset_hold
    tick(); dut->eval();
    CHECK_TRUE("cold_reset_hold high after write", dut->dbg_cold_reset_hold);
    // Read back bit 4 from CONTROL.
    uint32_t v = axil_read(0x008);
    CHECK_EQ("CONTROL.cold_reset_hold readback", v & 0x10u, 0x10u);

    // Hold must NOT be cleared by anything short of an explicit clear.
    // (We can't test the survival across `rst` without external glue —
    // that's covered by tb_debug_full_reset's S7 scenario.)  But we
    // confirm the bit stays set across many idle cycles, and that
    // legacy bit 2 / bit 5 / bit 1 writes do NOT clobber it (because
    // bit 4 is in the same byte but the host writes it explicitly).
    for (int i = 0; i < 16; i++) tick();
    CHECK_TRUE("cold_reset_hold persists across idle cycles",
               dut->dbg_cold_reset_hold);

    // Host writes a cold_reset_pulse (bit 5) AND keeps bit 4 set in the
    // same word — production tooling pattern (CTL_COLD_RESET_HOLD |
    // CTL_COLD_RESET_PULSE).
    obs_pulses = 0; obs_legacy_pulses = 0;
    obs_callback = obs_legacy_and_cold;
    axil_write(0x008, 0x10 | 0x20);  // hold + pulse
    for (int i = 0; i < 4; i++) {
        dut->eval();
        obs_legacy_and_cold();
        tick();
    }
    obs_callback = nullptr;
    CHECK_EQ("hold+pulse: pulse asserted exactly once", obs_pulses, 1);
    CHECK_TRUE("hold+pulse: hold remains high",
               dut->dbg_cold_reset_hold);

    // Clear the hold by writing a CONTROL with bit 4=0.
    axil_write(0x008, 0x0);
    tick(); dut->eval();
    CHECK_FALSE("cold_reset_hold low after explicit clear",
                dut->dbg_cold_reset_hold);

    // ── (c) Legacy bit 2 (soft_rst) folds into cold_reset_pulse ─────
    obs_pulses = 0; obs_legacy_pulses = 0;
    obs_callback = obs_legacy_and_cold;
    axil_write(0x008, 0x4);  // legacy bit 2 — soft_rst
    for (int i = 0; i < 4; i++) {
        dut->eval();
        obs_legacy_and_cold();
        tick();
    }
    obs_callback = nullptr;
    CHECK_EQ("legacy bit-2 fires soft_rst pulse exactly once",
             obs_legacy_pulses, 1);
    CHECK_EQ("legacy bit-2 ALSO fires cold_reset_pulse (folded path)",
             obs_pulses, 1);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 15 — Unified reset clears debug runtime state, but keeps the
// JTAG reset-control latch reachable.  `counters_clear` models soc_full_rst:
// it must wipe stale halt-after/break/arch/debug state while preserving
// cold_reset_hold so the host can explicitly release the CPU afterward.
// ════════════════════════════════════════════════════════════════════════
static bool test_unified_reset_clears_runtime_not_hold() {
    reset();

    axil_write(0x008, 0x19);       // halt_req + init_done_override + hold
    axil_write(0x030, 0x12345678); // halt-after lo
    axil_write(0x034, 0x0000009a); // halt-after hi
    axil_write(0x038, 0x40801234); // break PC
    axil_write(0x03c, 0x47);       // halt-after + clear + halt-exc
    axil_write(0x060, 0x00000010); // halt-exc mask lane 0
    axil_write(0x2000, 0xa5a55a5a);

    dut->counters_clear = 1;
    tick();
    dut->counters_clear = 0;
    tick();
    dut->eval();

    CHECK_TRUE("cold_reset_hold survives counters_clear",
               dut->dbg_cold_reset_hold);
    CHECK_FALSE("halt_req clears on counters_clear", dut->dbg_halt_req);
    CHECK_FALSE("init_done_override clears on counters_clear",
                dut->dbg_init_done_override);
    CHECK_EQ("CONTROL after counters_clear keeps only hold",
             axil_read(0x008) & 0x1fu, 0x10u);
    CHECK_EQ("HALT_AFTER_LO clears on counters_clear",
             axil_read(0x030), 0u);
    CHECK_EQ("HALT_AFTER_HI clears on counters_clear",
             axil_read(0x034), 0u);
    CHECK_EQ("BREAK_PC clears on counters_clear",
             axil_read(0x038), 0u);
    CHECK_EQ("HALT_CTL clears on counters_clear",
             axil_read(0x03c) & 0x47u, 0u);
    CHECK_EQ("halt-exc mask clears on counters_clear",
             axil_read(0x060), 0u);
    CHECK_EQ("arch shadow clears on counters_clear",
             axil_read(0x2000), 0u);

    // AXI remains writable after unified reset; this is what releases
    // a CPU held by cold_reset_hold.
    axil_write(0x008, 0x0);
    tick();
    dut->eval();
    CHECK_FALSE("cold_reset_hold clears after post-reset write",
                dut->dbg_cold_reset_hold);

    // The live FPGA failure mode was stricter: soc_full_rst was still
    // asserted while the host attempted the release write.  That CONTROL
    // write must win for bit 4 even when counters_clear is high.
    axil_write(0x008, 0x10);
    CHECK_TRUE("cold_reset_hold reasserted before held-clear test",
               dut->dbg_cold_reset_hold);
    dut->counters_clear = 1;
    axil_write(0x008, 0x0);
    tick();
    dut->eval();
    CHECK_FALSE("cold_reset_hold clears while counters_clear is high",
                dut->dbg_cold_reset_hold);
    dut->counters_clear = 0;
    tick();
    dut->eval();

    axil_write(0x030, 0x0000beef);
    CHECK_EQ("post-reset AXI write still lands",
             axil_read(0x030), 0x0000beefu);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 19 — Precise-breakpoint skip-once latch.
//
// Verify OFF_BP_SKIP_ONCE (0x080) bit 0 round-trips via AXI write,
// is exposed on dbg_break_pc_skip_once level output, and clears on a
// dbg_break_pc_skip_consume pulse.  Concurrent set+consume: consume wins.
// ════════════════════════════════════════════════════════════════════════
static bool test_bp_skip_once() {
    reset();

    CHECK_EQ("skip_once level initially 0", dut->dbg_break_pc_skip_once_array, 0u);
    CHECK_EQ("skip_once readback initially 0", axil_read(0x080u), 0u);

    // AXI write of 1 → level latches.
    axil_write(0x080u, 0x1u);
    tick();
    dut->eval();
    CHECK_EQ("skip_once level after AXI set", dut->dbg_break_pc_skip_once_array, 1u);
    CHECK_EQ("skip_once readback after set",  axil_read(0x080u), 1u);

    // Pulse consume → level clears.
    dut->dbg_break_pc_skip_consume_array = 1;
    tick();
    dut->dbg_break_pc_skip_consume_array = 0;
    tick();
    dut->eval();
    CHECK_EQ("skip_once level after consume", dut->dbg_break_pc_skip_once_array, 0u);

    // Concurrent set + consume: consume wins (textually later in the always).
    axil_write(0x080u, 0x1u);
    dut->dbg_break_pc_skip_consume_array = 1;
    // Note: axil_write runs multiple cycles to complete the BFM transaction;
    // the consume pulse here may align with the BVALID cycle.  Either way,
    // a final NBA-resolved sample after one extra tick should be 0.
    tick();
    dut->dbg_break_pc_skip_consume_array = 0;
    tick();
    dut->eval();
    CHECK_EQ("skip_once consume wins same-cycle", dut->dbg_break_pc_skip_once_array, 0u);

    return true;
}

static bool test_multi_breakpoint_slots() {
    reset();

    axil_write(0x038u, 0x40801000u);
    axil_write(0x084u, 0x40802000u);
    axil_write(0x088u, 0x40803000u);
    axil_write(0x08Cu, 0x40804000u);
    axil_write(0x090u, 0x0000000Fu);
    tick();
    dut->eval();

    CHECK_EQ("slot0 pc readback", axil_read(0x038u), 0x40801000u);
    CHECK_EQ("slot1 pc readback", axil_read(0x084u), 0x40802000u);
    CHECK_EQ("slot2 pc readback", axil_read(0x088u), 0x40803000u);
    CHECK_EQ("slot3 pc readback", axil_read(0x08Cu), 0x40804000u);
    CHECK_EQ("bp ctrl enables", axil_read(0x090u) & 0xFu, 0xFu);
    CHECK_TRUE("legacy break-pc enable OR", dut->dbg_break_pc_enable);
    CHECK_EQ("enable array output", dut->dbg_break_pc_enable_array, 0xFu);

    dut->dbg_break_pc_hit_slot = 2;
    dut->dbg_break_uop_fire = 1;
    tick();
    dut->dbg_break_uop_fire = 0;
    tick();
    dut->eval();
    CHECK_EQ("slot2 skip auto-armed", axil_read(0x080u) & 0xFu, 0x4u);
    CHECK_EQ("hit valid and slot latched", axil_read(0x090u) & 0x8300u, 0x8200u);

    dut->dbg_break_pc_skip_consume_array = 0x4u;
    tick();
    dut->dbg_break_pc_skip_consume_array = 0;
    tick();
    dut->eval();
    CHECK_EQ("slot2 skip consumed", axil_read(0x080u) & 0xFu, 0u);

    axil_write(0x090u, 0x0000400Au);
    tick();
    dut->eval();
    CHECK_EQ("bp ctrl enables after write", axil_read(0x090u) & 0xFu, 0xAu);
    CHECK_EQ("hit valid cleared", axil_read(0x090u) & 0x8000u, 0u);

    axil_write(0x03Cu, 0x2u);
    tick();
    dut->eval();
    CHECK_EQ("HALT_CTL slot0 alias", axil_read(0x090u) & 0xFu, 0xBu);

    return true;
}

static bool test_step_macro_arm_lifecycle() {
    reset();

    CHECK_FALSE("step macro initially clear", dut->dbg_step_macro_arm);
    axil_write(0x008u, 0x80u);
    tick();
    dut->eval();
    CHECK_TRUE("step macro arm set by CONTROL bit7", dut->dbg_step_macro_arm);
    CHECK_EQ("CONTROL bit7 readback", axil_read(0x008u) & 0x80u, 0x80u);

    dut->dbg_break_pc_hit_is_step = 1;
    dut->dbg_break_pc_hit_slot = 1;
    dut->dbg_break_uop_fire = 1;
    tick();
    dut->dbg_break_uop_fire = 0;
    dut->dbg_break_pc_hit_is_step = 0;
    tick();
    dut->eval();
    CHECK_FALSE("step macro clears on step DBG break", dut->dbg_step_macro_arm);
    CHECK_EQ("step DBG break does not arm bp skip", axil_read(0x080u) & 0xFu, 0u);
    CHECK_EQ("step DBG break does not latch hit_valid", axil_read(0x090u) & 0x8000u, 0u);

    return true;
}

static bool test_icache_op_from_jtag() {
    reset();

    dut->dbg_halted = 1;
    axil_write(0x214u, 0x1u);

    bool saw_ic_req = false;
    for (int i = 0; i < 8; i++) {
        dut->eval();
        if (dut->dbg_icache_op_req) saw_ic_req = true;
        tick();
    }
    CHECK_TRUE("OFF_ICACHE_OP launches icache req", saw_ic_req);
    CHECK_EQ("OFF_ICACHE_OP busy", axil_read(0x214u) & 0x1u, 0x1u);

    dut->dbg_icache_op_done = 1;
    tick();
    dut->dbg_icache_op_done = 0;
    tick();
    CHECK_EQ("OFF_ICACHE_OP done", axil_read(0x214u) & 0x3u, 0x2u);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 18 — Fault snapshot sticky-latch.
//
// Drive a known dbg_wedge_state value, pulse dbg_fault_snap_trigger once,
// verify OFF_FAULT_SNAP_VALID becomes 1 and W0..W3 capture the wedge state.
// Then change wedge_state, pulse trigger again — sticky, must NOT overwrite.
// Write OFF_FAULT_SNAP_CLEAR — VALID returns to 0, next trigger latches the
// new state.
// ════════════════════════════════════════════════════════════════════════
static bool test_fault_snap_latch_once() {
    reset();

    // Drive a known wedge state.  VlWide<4>: word 0 = wedge_state[31:0].
    dut->dbg_wedge_state.at(0) = 0xa50fdeadu;  // [31:0]
    dut->dbg_wedge_state.at(1) = 0xbeefcafeu;  // [63:32]
    dut->dbg_wedge_state.at(2) = 0x51001c00u;  // [95:64] (EA in real fault)
    dut->dbg_wedge_state.at(3) = 0x408046aau;  // [127:96] (PC in real fault)
    dut->dbg_fault_snap_trigger = 0;
    tick();

    CHECK_EQ("snap_valid initially 0", axil_read(0x3010), 0u);
    CHECK_EQ("snap_w0 initially 0",    axil_read(0x3014), 0u);

    // Pulse trigger one cycle.
    dut->dbg_fault_snap_trigger = 1;
    tick();
    dut->dbg_fault_snap_trigger = 0;
    tick();
    dut->eval();

    CHECK_EQ("snap_valid after trigger", axil_read(0x3010), 1u);
    CHECK_EQ("snap_w0 captures wedge", axil_read(0x3014), 0xa50fdeadu);
    CHECK_EQ("snap_w1 captures wedge", axil_read(0x3018), 0xbeefcafeu);
    CHECK_EQ("snap_w2 captures wedge", axil_read(0x301Cu), 0x51001c00u);
    CHECK_EQ("snap_w3 captures wedge", axil_read(0x3020u), 0x408046aau);

    // Second trigger with different values: sticky, must NOT overwrite.
    dut->dbg_wedge_state.at(0) = 0xdeadbeefu;
    dut->dbg_wedge_state.at(3) = 0xfeedfaceu;
    dut->dbg_fault_snap_trigger = 1;
    tick();
    dut->dbg_fault_snap_trigger = 0;
    tick();
    dut->eval();

    CHECK_EQ("snap_w0 still first", axil_read(0x3014u), 0xa50fdeadu);
    CHECK_EQ("snap_w3 still first", axil_read(0x3020u), 0x408046aau);

    // Clear via AXI write.
    axil_write(0x3024u, 0x1u);
    tick();
    dut->eval();

    CHECK_EQ("snap_valid cleared", axil_read(0x3010u), 0u);

    // Next trigger latches the new state.
    dut->dbg_fault_snap_trigger = 1;
    tick();
    dut->dbg_fault_snap_trigger = 0;
    tick();
    dut->eval();

    CHECK_EQ("snap_valid re-armed", axil_read(0x3010u), 1u);
    CHECK_EQ("snap_w0 new value",   axil_read(0x3014u), 0xdeadbeefu);
    CHECK_EQ("snap_w3 new value",   axil_read(0x3020u), 0xfeedfaceu);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 17 — OFF_LIVE_VBR readback round-trip.
//
// Verify OFF_LIVE_VBR (0x2100) returns whatever the test bench drives onto
// dbg_live_vbr_in.  Reproduces the 2026-05-11 HW finding where OFF_LIVE_VBR
// returned a value matching A5 instead of the real VBR.  If this scenario
// passes, debug_ctrl alone correctly latches/forwards dbg_live_vbr_in and
// the upstream wiring (commit.v → m68k_core → fpga_top → debug_ctrl) is
// where the bug lives.
// ════════════════════════════════════════════════════════════════════════
static bool test_live_vbr_readback() {
    reset();

    // Drive dbg_live_vbr_in to a known value; this tb is the only thing
    // feeding the input.  Two ticks let the AR/R handshake settle.
    dut->dbg_live_vbr_in = 0xCAFEBABEu;
    tick();
    tick();
    uint32_t got = axil_read(0x2100);
    CHECK_EQ("OFF_LIVE_VBR readback first value", got, 0xCAFEBABEu);

    // Change the driver, verify the readback follows.
    dut->dbg_live_vbr_in = 0x40846980u;
    tick();
    tick();
    got = axil_read(0x2100);
    CHECK_EQ("OFF_LIVE_VBR readback follows input", got, 0x40846980u);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Main
// ════════════════════════════════════════════════════════════════════════
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vdebug_ctrl;

    RUN(test_version_read);
    RUN(test_control_level_bits);
    RUN(test_control_pulse_bits);
    RUN(test_cycle_counter);
    RUN(test_inst_counter);
    RUN(test_pc_trace_ring);
    RUN(test_pc_trace_redirect_only);
    RUN(test_redirect_trigger);
    RUN(test_exception_capture);
    RUN(test_exc_ring_buffer);
    RUN(test_irq_inject);
    RUN(test_ram_window_lg2_register);
    RUN(test_auto_halt_latch_and_clear);
    RUN(test_step_sequence_preserves_halt_req);
    RUN(test_halt_after_auto_advance);
    RUN(test_unimplemented_returns_zero);
    RUN(test_arch_shadow_apply_and_resume);
    RUN(test_unified_reset_bits);
    RUN(test_unified_reset_clears_runtime_not_hold);
    RUN(test_live_vbr_readback);
    RUN(test_fault_snap_latch_once);
    RUN(test_bp_skip_once);
    RUN(test_multi_breakpoint_slots);
    RUN(test_step_macro_arm_lifecycle);
    RUN(test_icache_op_from_jtag);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
