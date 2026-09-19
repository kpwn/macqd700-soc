// tb_mmu.cpp — Verilator unit testbench for the phase-2 MMU stub.
//
// Exercises rtl/core/mem/mmu.v standalone.  Each scenario is a self-
// contained function that calls CHECK() macros.
//
// Build:   make tb-mmu
// Expect:
//   [PASS] reset_passthrough
//   [PASS] cr_write_readback
//   [PASS] mmu_disabled_passthrough
//   [PASS] itt0_rom_region_match
//   [PASS] dtt0_io_region_match
//   [PASS] ttr_priority_0_over_1
//   [PASS] s_field_user_only
//   [PASS] s_field_sup_only
//   [PASS] s_field_both
//   [PASS] no_match_still_passthrough
//   [PASS] write_protect_fault
//   [PASS] write_protect_only_on_writes
//   [PASS] i_side_uses_itt_not_dtt
//
// All 13 scenarios PASSED.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <verilated.h>
#include "Vmmu.h"

static Vmmu*   dut       = nullptr;
static uint64_t sim_time = 0;
static int      n_pass   = 0;
static int      n_fail   = 0;

// ── Clock helpers ──────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void reset() {
    dut->rst            = 1;
    dut->va_in          = 0;
    dut->is_instruction = 0;
    dut->is_write       = 0;
    dut->supervisor     = 0;
    dut->wr_en          = 0;
    dut->wr_cr          = 0;
    dut->wr_val         = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
}

static void write_cr(uint32_t cr, uint32_t val) {
    dut->wr_en  = 1;
    dut->wr_cr  = cr;
    dut->wr_val = val;
    tick();
    dut->wr_en  = 0;
    dut->wr_val = 0;
    dut->eval();
}

// ── Assertion helpers ─────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    if ((uint32_t)(got) != (uint32_t)(exp)) { \
        printf("  FAIL %s: got 0x%08x, expected 0x%08x\n", \
               name, (uint32_t)(got), (uint32_t)(exp)); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s: condition false\n", name); \
        return false; \
    } \
} while(0)

#define CHECK_FALSE(name, cond) do { \
    if (cond) { \
        printf("  FAIL %s: condition unexpectedly true\n", name); \
        return false; \
    } \
} while(0)

// ── Helpers to build TTR register values ──────────────────────────
//   base  : top-byte of logical address that matches
//   mask  : bits in mask=1 are "don't care" in address compare
//   e     : enable bit
//   s     : 2'b00 = user only, 2'b01 = sup only, 2'b1x = both
//   w     : write-protect bit (bit 2)
static uint32_t mk_ttr(uint8_t base, uint8_t mask, int e, int s, int w) {
    uint32_t v = 0;
    v |= ((uint32_t)base) << 24;
    v |= ((uint32_t)mask) << 16;
    v |= (e ? 1u : 0u) << 15;
    v |= ((uint32_t)(s & 3)) << 13;
    v |= (w ? 1u : 0u) << 2;
    return v;
}

// ── Drive a translation request and sample outputs combinationally ─
static void xlate(uint32_t va, int inst, int wr, int sup) {
    dut->va_in          = va;
    dut->is_instruction = inst;
    dut->is_write       = wr;
    dut->supervisor     = sup;
    dut->eval();
}

// ════════════════════════════════════════════════════════════════════
// Scenario 1 — reset leaves MMU disabled; every VA passes through.
// ════════════════════════════════════════════════════════════════════
static bool test_reset_passthrough() {
    reset();
    CHECK_EQ("tc zero", dut->tc, 0);
    CHECK_EQ("itt0 zero", dut->itt0, 0);

    xlate(0x40800000, /*inst*/1, 0, /*sup*/1);
    CHECK_EQ("pa_out = va", dut->pa_out, 0x40800000);
    CHECK_FALSE("no fault", dut->fault);

    xlate(0xFFFF0000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_EQ("pa_out = va (write)", dut->pa_out, 0xFFFF0000);
    CHECK_FALSE("no fault on write", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 2 — CR writes round-trip through the readback ports.
// ════════════════════════════════════════════════════════════════════
static bool test_cr_write_readback() {
    reset();
    write_cr(0, 0xDEADBEEF);
    CHECK_EQ("itt0 readback", dut->itt0, 0xDEADBEEF);
    write_cr(1, 0x11223344);
    CHECK_EQ("itt1 readback", dut->itt1, 0x11223344);
    write_cr(2, 0x55667788);
    CHECK_EQ("dtt0 readback", dut->dtt0, 0x55667788);
    write_cr(3, 0x99AABBCC);
    CHECK_EQ("dtt1 readback", dut->dtt1, 0x99AABBCC);
    write_cr(4, 0x0000C0DE);
    CHECK_EQ("tc   readback", dut->tc,   0x0000C0DE);
    write_cr(5, 0xAAAA5555);
    CHECK_EQ("urp  readback", dut->urp,  0xAAAA5555);
    write_cr(6, 0x5555AAAA);
    CHECK_EQ("srp  readback", dut->srp,  0x5555AAAA);
    // Unknown cr index is a no-op
    write_cr(7, 0xFFFFFFFF);
    CHECK_EQ("unknown cr no-op (itt0 preserved)", dut->itt0, 0xDEADBEEF);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 3 — With TC.E = 0 every address passes through, even when
// an ITT would otherwise match + fault.
// ════════════════════════════════════════════════════════════════════
static bool test_mmu_disabled_passthrough() {
    reset();
    // Configure DTT0 as a write-protected region that WOULD fault if
    // the MMU were enabled.
    write_cr(2, mk_ttr(0x40, 0x00, /*e*/1, /*s*/3, /*w*/1));
    // TC.E = 0 (stays at reset value 0)
    xlate(0x40800000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_EQ("pa still = va", dut->pa_out, 0x40800000);
    CHECK_FALSE("no fault when MMU disabled", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 4 — Quadra ROM cold-start case: ITT0 covers the ROM region.
// Instruction fetches in [0x40000000..0x40FFFFFF] match via ITT0.
// ════════════════════════════════════════════════════════════════════
static bool test_itt0_rom_region_match() {
    reset();
    // TC.E = 1 (bit 15)
    write_cr(4, 1u << 15);
    // ITT0: base = 0x40, mask = 0x00 (top-byte must equal 0x40), E=1,
    // S=1x (both), W=0.  Matches all of 0x40000000..0x40FFFFFF.
    write_cr(0, mk_ttr(0x40, 0x00, 1, 3, 0));

    xlate(0x40800000, /*inst*/1, 0, /*sup*/1);
    CHECK_EQ("ROM fetch PA = VA", dut->pa_out, 0x40800000);
    CHECK_FALSE("no fault on ROM fetch", dut->fault);

    // Fetch outside the range: 0x00000000 — no ITT match, but stub
    // still passes through (no walker).
    xlate(0x00000000, /*inst*/1, 0, /*sup*/1);
    CHECK_EQ("non-ROM fetch PA = VA (stub fallthrough)", dut->pa_out, 0x0);
    CHECK_FALSE("no fault on non-ROM fetch", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 5 — DTT0 covers the I/O region (e.g. 0xF0000000..0xFFFFFFFF).
// ════════════════════════════════════════════════════════════════════
static bool test_dtt0_io_region_match() {
    reset();
    write_cr(4, 1u << 15);
    // DTT0: base = 0xF0, mask = 0x0F → top nibble 0xF (i.e.
    // 0xF0000000..0xFFFFFFFF), E=1, S=1x, W=0.
    write_cr(2, mk_ttr(0xF0, 0x0F, 1, 3, 0));

    xlate(0xFFFF0000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_EQ("I/O write PA = VA", dut->pa_out, 0xFFFF0000);
    CHECK_FALSE("I/O write no fault", dut->fault);

    xlate(0xF1234567, /*inst*/0, /*wr*/0, /*sup*/1);
    CHECK_EQ("I/O read PA = VA", dut->pa_out, 0xF1234567);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 6 — When both TTR0 and TTR1 match, TTR0's attributes win.
// Configure DTT0 as non-WP and DTT1 as WP; write must NOT fault.
// ════════════════════════════════════════════════════════════════════
static bool test_ttr_priority_0_over_1() {
    reset();
    write_cr(4, 1u << 15);
    // DTT0: base 0x40, mask 0x00, E=1, S=1x, W=0
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 0));
    // DTT1: same range, E=1, S=1x, W=1 (write-protect)
    write_cr(3, mk_ttr(0x40, 0x00, 1, 3, 1));

    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_FALSE("DTT0 (non-WP) wins over DTT1 (WP)", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 7 — S-field = 00 matches only in user mode.
// ════════════════════════════════════════════════════════════════════
static bool test_s_field_user_only() {
    reset();
    write_cr(4, 1u << 15);
    // DTT0: user-only (S=00), W=1 so a matched write would fault.
    write_cr(2, mk_ttr(0x40, 0x00, 1, 0, 1));

    // User + write → match → W-bit fault
    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/0);
    CHECK_TRUE("user-only TTR matches in user mode (WP fault)", dut->fault);

    // Supervisor + write → S-field mismatch, no match → pass-through
    // (no fault in phase-2 stub because no walker).
    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_FALSE("user-only TTR does not match in sup mode", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 8 — S-field = 01 matches only in supervisor mode.
// ════════════════════════════════════════════════════════════════════
static bool test_s_field_sup_only() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 1, 1));

    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_TRUE("sup-only TTR matches in sup mode (WP fault)", dut->fault);

    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/0);
    CHECK_FALSE("sup-only TTR does not match in user mode", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 9 — S-field = 1x matches both modes.
// ════════════════════════════════════════════════════════════════════
static bool test_s_field_both() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 2, 1));  // S=10
    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_TRUE("S=10 matches sup", dut->fault);
    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/0);
    CHECK_TRUE("S=10 matches user", dut->fault);

    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));  // S=11
    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_TRUE("S=11 matches sup", dut->fault);
    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/0);
    CHECK_TRUE("S=11 matches user", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 10 — MMU enabled but VA outside any TTR range:
// stub pretends walker said OK (pass-through, no fault).
// ════════════════════════════════════════════════════════════════════
static bool test_no_match_still_passthrough() {
    reset();
    write_cr(4, 1u << 15);
    // DTT0 covers 0x40 only
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 0));

    xlate(0x80000000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_EQ("PA = VA (no match)", dut->pa_out, 0x80000000);
    CHECK_FALSE("no fault (no walker, stub passes through)", dut->fault);
    CHECK_TRUE("no-match passthrough is cache-inhibited", dut->cache_inh_out);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 11 — Write-protect fault fires only when W=1 + is_write=1.
// ════════════════════════════════════════════════════════════════════
static bool test_write_protect_fault() {
    reset();
    write_cr(4, 1u << 15);
    // DTT0 W=1, S=1x
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));

    xlate(0x40100000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_TRUE("WP fault on write", dut->fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 12 — Write-protect does NOT fault on reads.
// ════════════════════════════════════════════════════════════════════
static bool test_write_protect_only_on_writes() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));

    xlate(0x40100000, /*inst*/0, /*wr*/0, /*sup*/1);
    CHECK_FALSE("no WP fault on read", dut->fault);
    CHECK_EQ("read PA = VA", dut->pa_out, 0x40100000);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Scenario 13 — I-side uses ITT, D-side uses DTT; they don't cross-talk.
// DTT0 WP; ITT0 non-WP.  An I-fetch on the overlapping address must
// NOT consult DTT0 and must NOT fault.
// ════════════════════════════════════════════════════════════════════
static bool test_i_side_uses_itt_not_dtt() {
    reset();
    write_cr(4, 1u << 15);
    // ITT0: matches 0x40 range, W=0
    write_cr(0, mk_ttr(0x40, 0x00, 1, 3, 0));
    // DTT0: matches 0x40 range, W=1 (would fault on write)
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));

    // I-fetch: must look at ITT0 (non-WP), is_write is irrelevant for
    // fetch anyway, but is_instruction=1 selects the I-side scan.
    xlate(0x40800000, /*inst*/1, /*wr*/0, /*sup*/1);
    CHECK_FALSE("I-fetch does not trip DTT WP", dut->fault);

    // D-side write on same address: DTT0 matches, W=1, WP fault.
    xlate(0x40800000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_TRUE("D-write trips DTT WP", dut->fault);

    return true;
}

// ════════════════════════════════════════════════════════════════════
// Fault-path extensions (tb-mmu-faults)
// ════════════════════════════════════════════════════════════════════
// The phase-2 MMU stub implements the happy-path ITT/DTT translation
// (covered by scenarios 1–13 above).  These additional scenarios focus
// on fault paths, ITT/DTT miss edge cases, URP vs SRP selection by
// SR.S, and TC enable/disable transitions.

// Scenario 14 — ITT miss + no DTT match: instruction fetch falls
// through to pass-through when no ITT matches (stub behaviour).
static bool test_itt_miss_instr_passthrough() {
    reset();
    write_cr(4, 1u << 15);  // TC.E = 1
    // ITT0 covers 0x40 only
    write_cr(0, mk_ttr(0x40, 0x00, 1, 3, 0));

    // I-fetch outside the 0x40 range: ITT0 doesn't match, ITT1 is zero.
    // Stub must still produce PA = VA, no fault.
    xlate(0x80000000, /*inst*/1, 0, /*sup*/1);
    CHECK_EQ("ITT-miss I-fetch PA=VA", dut->pa_out, 0x80000000);
    CHECK_FALSE("ITT-miss no fault",   dut->fault);
    return true;
}

// Scenario 15 — DTT miss: data access falls through too.
static bool test_dtt_miss_data_passthrough() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));   // DTT0 matches 0x40, W=1

    // Access outside range: no DTT match → pass-through, no fault.
    xlate(0x80000000, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_EQ("DTT-miss D-write PA=VA", dut->pa_out, 0x80000000);
    CHECK_FALSE("DTT-miss no fault",   dut->fault);
    return true;
}

// Scenario 16 — Invalid PTE synthesis: the phase-2 stub does NOT yet
// synthesise "access fault" on bad PTEs (no walker exists).  Document
// this by asserting that every VA passes through cleanly when TC is
// enabled but no TTR matches.  Fault only fires on W-bit WP.
static bool test_no_access_fault_on_walker_miss() {
    reset();
    write_cr(4, 1u << 15);
    // No TTRs configured — every access should pass through.
    for (uint32_t a : {0x00000000u, 0x12345678u, 0xCAFEBABEu}) {
        xlate(a, /*inst*/1, 0, /*sup*/1);
        CHECK_EQ("walker-miss I-fetch PA=VA",  dut->pa_out, a);
        CHECK_FALSE("walker-miss I-fault",     dut->fault);
        xlate(a, /*inst*/0, /*wr*/1, /*sup*/1);
        CHECK_EQ("walker-miss D-write PA=VA",  dut->pa_out, a);
        CHECK_FALSE("walker-miss D-fault",     dut->fault);
    }
    return true;
}

// Scenario 17 — URP vs SRP selection: phase-2 stub stores URP/SRP as
// readback-only registers and doesn't use them for translation (no
// walker).  Verify both are independently writable and don't alias.
// This matches the hardware design intent: supervisor-mode walks use
// SRP, user-mode walks use URP, selected by SR.S.
static bool test_urp_srp_independent() {
    reset();
    write_cr(5, 0xDEADBEEFu);  // URP
    write_cr(6, 0x12345678u);  // SRP
    CHECK_EQ("URP stored", dut->urp, 0xDEADBEEFu);
    CHECK_EQ("SRP stored", dut->srp, 0x12345678u);
    // Overwrite URP, SRP untouched
    write_cr(5, 0xAAAAAAAAu);
    CHECK_EQ("URP updated",        dut->urp, 0xAAAAAAAAu);
    CHECK_EQ("SRP preserved",      dut->srp, 0x12345678u);
    return true;
}

// Scenario 18 — TC enable/disable transition: toggling TC.E mid-flight
// should flip fault behaviour immediately (combinational path).
static bool test_tc_enable_disable_transition() {
    reset();
    // DTT0 configured as write-protected
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));

    // TC disabled (E=0): write to 0x40 should NOT fault.
    write_cr(4, 0);
    xlate(0x40100000, 0, 1, 1);
    CHECK_FALSE("TC disabled — no fault on WP match", dut->fault);

    // Enable TC → now same write SHOULD fault.
    write_cr(4, 1u << 15);
    xlate(0x40100000, 0, 1, 1);
    CHECK_TRUE("TC enabled — WP fault",               dut->fault);

    // Disable again → no fault.
    write_cr(4, 0);
    xlate(0x40100000, 0, 1, 1);
    CHECK_FALSE("TC re-disabled — no fault",          dut->fault);
    return true;
}

// Scenario 19 — ITT1 fallback: ITT1 is checked only when ITT0 doesn't
// match.  Verify ITT1 works as a secondary.
static bool test_itt1_secondary_match() {
    reset();
    write_cr(4, 1u << 15);
    // ITT0: covers 0x40 range, but with S=01 (sup only)
    write_cr(0, mk_ttr(0x40, 0x00, 1, 1, 0));
    // ITT1: covers same 0x40 range, S=1x (both modes)
    write_cr(1, mk_ttr(0x40, 0x00, 1, 3, 0));

    // User-mode I-fetch: ITT0 doesn't match (sup only).  ITT1 should.
    xlate(0x40100000, 1, 0, /*sup*/0);
    CHECK_EQ("ITT1 matches in user mode", dut->pa_out, 0x40100000);
    CHECK_FALSE("ITT1 match no fault",    dut->fault);
    return true;
}

// Scenario 20 — DTT1 WP: DTT1 configured W=1.  DTT0 doesn't match (wrong
// base).  A matching D-write should fault via DTT1.
static bool test_dtt1_wp_fault() {
    reset();
    write_cr(4, 1u << 15);
    // DTT0: covers 0x50 only
    write_cr(2, mk_ttr(0x50, 0x00, 1, 3, 0));
    // DTT1: covers 0x40 only, W=1
    write_cr(3, mk_ttr(0x40, 0x00, 1, 3, 1));

    xlate(0x40100000, 0, /*wr*/1, 1);
    CHECK_TRUE("DTT1 WP fault on matching write", dut->fault);
    return true;
}

// Scenario 21 — Fault vector: fault_vec is 8 (phase-2 stub placeholder).
static bool test_fault_vec_value() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));

    xlate(0x40100000, 0, 1, 1);
    CHECK_TRUE("fault on WP write", dut->fault);
    CHECK_EQ("fault_vec = 8",       dut->fault_vec, 8);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Additional tb-mmu-faults widening (task #82)
// These scenarios exercise the extended fault-reporting surface
// (fault_addr_out, fault_code_out), multi-TTR stress, and back-to-back
// fault-then-success sequences to confirm no latched fault state bleeds
// between accesses.
// ════════════════════════════════════════════════════════════════════

// Scenario 22 — TTR-WP fault surfaces fault_addr_out = faulting VA and
// fault_code_out = 3'd7 (ITT/DTT-WP per mmu.v comment).
static bool test_ttr_wp_fault_addr_and_code() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));   // DTT0 WP
    xlate(0x40ABCDEF, /*inst*/0, /*wr*/1, /*sup*/1);
    CHECK_TRUE("TTR WP fault fires", dut->fault);
    CHECK_EQ("fault_addr_out = faulting VA", dut->fault_addr_out, 0x40ABCDEF);
    CHECK_EQ("fault_code_out = 7 (TTR-WP)",  dut->fault_code_out, 7);
    return true;
}

// Scenario 23 — back-to-back WP fault then non-fault access: fault signal
// must be combinationally gated by the NEW request, not latched from the
// previous one.
static bool test_fault_not_latched_across_accesses() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));   // DTT0 WP

    xlate(0x40100000, 0, 1, 1);  // WP fault
    CHECK_TRUE("step1: WP fault set", dut->fault);

    xlate(0x40100000, 0, 0, 1);  // same addr, now a READ → no fault
    CHECK_FALSE("step2: read deasserts fault same cycle", dut->fault);

    xlate(0x80000000, 0, 1, 1);  // unmapped write, no TTR hit
    CHECK_FALSE("step3: unmapped access → no fault (stub passthrough)",
                dut->fault);
    return true;
}

// Scenario 24 — all four S-field combinations exercised in one pass.
// S=00 user-only, 01 sup-only, 10 both, 11 both.  Verify fault discrimination.
static bool test_all_s_field_combinations() {
    reset();
    write_cr(4, 1u << 15);
    // Install four DTT0s sequentially, each with a WP matching 0x40,
    // checking fault behaviour in both user + sup mode.
    struct Row { int s; int sup_should_fault; int user_should_fault; };
    Row rows[] = {
        {0b00, 0, 1},  // user only
        {0b01, 1, 0},  // sup only
        {0b10, 1, 1},  // both
        {0b11, 1, 1},  // both
    };
    for (auto& r : rows) {
        write_cr(2, mk_ttr(0x40, 0x00, 1, r.s, 1));
        xlate(0x40100000, 0, 1, /*sup*/1);
        if ((int)dut->fault != r.sup_should_fault) {
            printf("  FAIL s=%d sup: got %d exp %d\n",
                   r.s, (int)dut->fault, r.sup_should_fault);
            return false;
        }
        xlate(0x40100000, 0, 1, /*sup*/0);
        if ((int)dut->fault != r.user_should_fault) {
            printf("  FAIL s=%d usr: got %d exp %d\n",
                   r.s, (int)dut->fault, r.user_should_fault);
            return false;
        }
    }
    return true;
}

// Scenario 25 — all 4 TTRs enabled simultaneously, distinct ranges.
// Exercises the match-prioritiser when multiple candidates exist.
static bool test_all_four_ttrs_distinct_ranges() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(0, mk_ttr(0x40, 0x00, 1, 3, 0));  // ITT0 0x40xx, W=0
    write_cr(1, mk_ttr(0x50, 0x00, 1, 3, 0));  // ITT1 0x50xx, W=0 (unused — fetch bus)
    write_cr(2, mk_ttr(0x60, 0x00, 1, 3, 0));  // DTT0 0x60xx, W=0
    write_cr(3, mk_ttr(0x70, 0x00, 1, 3, 1));  // DTT1 0x70xx, W=1

    // I-fetch to 0x40 → ITT0 hit, no fault
    xlate(0x40000000, 1, 0, 1);
    CHECK_FALSE("I@0x40 (ITT0) no fault", dut->fault);
    CHECK_EQ("I@0x40 PA=VA", dut->pa_out, 0x40000000);

    // I-fetch to 0x50 → ITT1 hit, no fault
    xlate(0x50000000, 1, 0, 1);
    CHECK_FALSE("I@0x50 (ITT1) no fault", dut->fault);

    // D-write to 0x60 → DTT0, W=0 → no fault
    xlate(0x60000000, 0, 1, 1);
    CHECK_FALSE("D@0x60 (DTT0 W=0) no fault", dut->fault);

    // D-write to 0x70 → DTT1, W=1 → fault
    xlate(0x70000000, 0, 1, 1);
    CHECK_TRUE("D@0x70 (DTT1 W=1) fault", dut->fault);
    CHECK_EQ("D@0x70 fault_addr_out=VA", dut->fault_addr_out, 0x70000000);

    // D-fetch to 0x40 (ITT range but D-side) — no DTT match → no fault
    xlate(0x40000000, 0, 1, 1);
    CHECK_FALSE("D@0x40 no DTT match → no fault (passthrough)", dut->fault);
    return true;
}

// Scenario 26 — CR-write mid-sequence: updating TC.E live must take
// effect on the NEXT combinational probe without needing a reset.
static bool test_live_tc_toggle_under_wp() {
    reset();
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));  // DTT0 W=1

    // MMU disabled — no fault.
    xlate(0x40000000, 0, 1, 1);
    CHECK_FALSE("TC.E=0 → no fault", dut->fault);

    // Enable → fault.
    write_cr(4, 1u << 15);
    xlate(0x40000000, 0, 1, 1);
    CHECK_TRUE("TC.E=1 → WP fault fires", dut->fault);
    CHECK_EQ("fault_code=7", dut->fault_code_out, 7);

    // Redisable live — fault must clear on next probe.
    write_cr(4, 0);
    xlate(0x40000000, 0, 1, 1);
    CHECK_FALSE("TC.E=0 re-disable → fault clears", dut->fault);
    return true;
}

// Scenario 27 — URP/SRP update mid-sequence must not disturb pa_out for
// a TTR-covered translation (phase-2 stub ignores URP/SRP in this path).
static bool test_urp_srp_write_does_not_affect_ttr_path() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(0, mk_ttr(0x40, 0x00, 1, 3, 0));  // ITT0 match

    xlate(0x40000100, 1, 0, 1);
    CHECK_EQ("pre: PA=VA", dut->pa_out, 0x40000100);

    write_cr(5, 0xDEADBEEF);   // URP
    write_cr(6, 0xBEEFCAFE);   // SRP

    xlate(0x40000100, 1, 0, 1);
    CHECK_EQ("post CR writes: PA=VA still", dut->pa_out, 0x40000100);
    CHECK_FALSE("no spurious fault", dut->fault);
    return true;
}

// Scenario 28 — TTR mask-range match (not just zero-mask).  mask=0x0F
// in the TTR means the bottom nibble of the top byte is "don't care";
// a correct implementation must route 0x40xx AND 0x4Fxx through it.
static bool test_ttr_mask_don_t_care_nibble() {
    reset();
    write_cr(4, 1u << 15);
    // DTT0: base=0x40, mask=0x0F → matches 0x40..0x4F
    write_cr(2, mk_ttr(0x40, 0x0F, 1, 3, 1));

    xlate(0x40000000, 0, 1, 1);
    CHECK_TRUE("0x40 within TTR mask — WP fault",  dut->fault);
    xlate(0x4F123456, 0, 1, 1);
    CHECK_TRUE("0x4F within TTR mask — WP fault",  dut->fault);
    xlate(0x50000000, 0, 1, 1);
    CHECK_FALSE("0x50 outside TTR mask — no fault (no walker)", dut->fault);
    return true;
}

// Scenario 29 — fault-code priority: current stub reports TTR-WP > ATC >
// walker.  With only a TTR hit in play, confirm fault_code_out = 7 and
// pa_out = va (not zero).  This future-proofs the tb against the
// Phase-B wiring that will drive fault_addr_out from ATC/walker too.
static bool test_fault_code_priority_ttr_wins() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x40, 0x00, 1, 3, 1));

    xlate(0x40ABC000, 0, 1, 1);
    CHECK_TRUE("fault asserted", dut->fault);
    CHECK_EQ("fault_code = 7 (TTR-WP)", dut->fault_code_out, 7);
    CHECK_EQ("fault_addr = VA",         dut->fault_addr_out, 0x40ABC000);
    // pa_out is allowed to be VA (passthrough) in the faulting path per
    // the stub; just confirm it's not clobbered to something nonsensical.
    CHECK_EQ("pa_out mirrors VA",       dut->pa_out, 0x40ABC000);
    return true;
}

// Scenario 30 — data accesses must remain cache-inhibited until they resolve
// through an explicit TTR/ATC/walker result.  This catches the direct fallback
// case where PA=VA is only a pre-walker/default path, not a cacheable mapping.
static bool test_untranslated_fallback_cache_inhibited() {
    reset();
    write_cr(4, 1u << 15);
    write_cr(2, mk_ttr(0x50, 0x00, 1, 3, 0));

    xlate(0x40803598, 0, 0, 1);
    CHECK_EQ("fallback PA=VA", dut->pa_out, 0x40803598);
    CHECK_FALSE("fallback no fault", dut->fault);
    CHECK_TRUE("fallback cache-inhibited", dut->cache_inh_out);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Main
// ════════════════════════════════════════════════════════════════════
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vmmu;

    RUN(test_reset_passthrough);
    RUN(test_cr_write_readback);
    RUN(test_mmu_disabled_passthrough);
    RUN(test_itt0_rom_region_match);
    RUN(test_dtt0_io_region_match);
    RUN(test_ttr_priority_0_over_1);
    RUN(test_s_field_user_only);
    RUN(test_s_field_sup_only);
    RUN(test_s_field_both);
    RUN(test_no_match_still_passthrough);
    RUN(test_write_protect_fault);
    RUN(test_write_protect_only_on_writes);
    RUN(test_i_side_uses_itt_not_dtt);
    // tb-mmu-faults extensions
    RUN(test_itt_miss_instr_passthrough);
    RUN(test_dtt_miss_data_passthrough);
    RUN(test_no_access_fault_on_walker_miss);
    RUN(test_urp_srp_independent);
    RUN(test_tc_enable_disable_transition);
    RUN(test_itt1_secondary_match);
    RUN(test_dtt1_wp_fault);
    RUN(test_fault_vec_value);
    // tb-mmu-faults widening (task #82)
    RUN(test_ttr_wp_fault_addr_and_code);
    RUN(test_fault_not_latched_across_accesses);
    RUN(test_all_s_field_combinations);
    RUN(test_all_four_ttrs_distinct_ranges);
    RUN(test_live_tc_toggle_under_wp);
    RUN(test_urp_srp_write_does_not_affect_ttr_path);
    RUN(test_ttr_mask_don_t_care_nibble);
    RUN(test_fault_code_priority_ttr_wins);
    RUN(test_untranslated_fallback_cache_inhibited);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
