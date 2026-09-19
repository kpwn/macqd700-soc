// tb_via2.cpp — Verilator unit testbench for via2.v (full Q700 variant)
//
// Build:   make tb-via2
// Covers the full-6522 feature set ported from via1.v plus the Q700-
// specific NuBus-slot / PA slot-IRQ path.
//
// Scenarios (tagged by name so the agent-policy grep can find them):
//   1.  test_reset_reads                 — reset values of writeable regs
//   2.  test_reg_roundtrip               — write/read round-trip
//   3.  test_t1_oneshot                  — T1 fires exactly once
//   4.  test_t1_freerun_pb7              — T1 continuous + PB7 square wave
//   5.  test_t1_oneshot_pb7              — PB7 one-shot pulse on underflow
//   6.  test_t2_timed                    — T2 timed-interrupt one-shot
//   7.  test_sr_shift_out                — SR shift-out fires IFR[SR] after 8 bits
//   8.  test_sr_shift_in                 — SR shift-in fires IFR[SR] after 8 bits
//   9.  test_ifr_ier_priority            — IER masking of IFR into irq line
//  10.  test_ifr_write_1_clear           — IFR bit clearing semantics
//  11.  test_slot_irq_ca1                — PA slot-line fall latches IFR[CA1]
//  12.  test_ca1_edge_polarity           — PCR[0] selects pos/neg edge
//  13.  test_cb1_edge_polarity           — PCR[4] selects pos/neg edge
//  14.  test_ora_hs_vs_nh                — ORA-read clears IFR; ORA-NH does not
//  15.  test_port_a_input_reads          — PA DDRA=0 returns pa_in (slot sense)
//  16.  test_reset_asserted_slot_latches_ca1 — asserted slot across reset latches CA1
//  17.  test_pa6_video_slot_ca1          — PA6 video slot latches CA1
//  18.  test_pcr_independent_irq_clears  — ORA/ORB leave CA2/CB2 alone in IRQ mode
//  19.  test_t2_pulsecount               — PB6 falling edges decrement pulse-count T2
//  20.  test_ca2_cb2_independent_edge_polarity — PCR[2]/[6] select IRQ edge
//  21.  test_orb_write_clears_cb_handshake — ORB write acknowledges PB side
//  24.  test_scsi_irq_edge_never_strands_across_phi2_window
//                                       — a 53C96 IRQ edge must reach IFR
//                                         BEFORE the ISR's IFR clear, and
//                                         must never outlive the source
//
// Scenario 16 is the "corner I almost got wrong" — reg 1 vs reg 15 read
// semantics for IFR[CA1]/[CA2] handshake-clear.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vvia2.h"

static Vvia2*   dut       = nullptr;
static uint64_t sim_time  = 0;
static int      n_pass    = 0;
static int      n_fail    = 0;

// ── Clock helpers ──────────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void reset() {
    dut->rst       = 1;
    dut->phi2_tick = 0;
    dut->pb_addr   = 0;
    dut->pb_wdata  = 0;
    dut->pb_wr     = 0;
    dut->pb_rd     = 0;
    dut->pa_in     = 0xFF;         // slot-IRQ idle high
    dut->pb_in     = 0xFF;
    dut->ca1_in    = 1;
    dut->ca2_in    = 1;
    dut->cb1_in    = 1;
    dut->cb2_in    = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
}

// ── Assertion helpers ──────────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    uint32_t _g = (uint32_t)(got); \
    uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%02x, expected 0x%02x\n", name, _g, _e); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s: condition false\n", name); \
        return false; \
    } \
} while(0)

// ── Bus helpers ────────────────────────────────────────────────────────
static void bus_write(uint8_t addr, uint8_t data) {
    dut->pb_addr   = addr & 0xF;
    dut->pb_wdata  = data;
    dut->pb_wr     = 1;
    dut->pb_rd     = 0;
    dut->phi2_tick = 0;
    tick();
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
}

static uint8_t bus_read(uint8_t addr) {
    dut->pb_addr   = addr & 0xF;
    dut->pb_rd     = 1;
    dut->pb_wr     = 0;
    dut->phi2_tick = 0;
    tick();
    dut->pb_rd     = 0;
    dut->eval();
    return dut->pb_rdata & 0xFF;
}

static void phi2_ticks(uint32_t n) {
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    for (uint32_t i = 0; i < n; i++) {
        dut->phi2_tick = 1;
        tick();
        dut->phi2_tick = 0;
    }
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 1 — reset values of the writeable regs
// ════════════════════════════════════════════════════════════════════════
static bool test_reset_reads() {
    reset();
    CHECK_EQ("ORB reset",  bus_read(0),  0xFF);
    CHECK_EQ("ORA reset",  bus_read(1),  0xFF);
    CHECK_EQ("DDRB reset", bus_read(2),  0x00);
    CHECK_EQ("DDRA reset", bus_read(3),  0x00);
    CHECK_EQ("ACR reset",  bus_read(11), 0x00);
    CHECK_EQ("PCR reset",  bus_read(12), 0x00);
    // IER MSB reads as 1 (per 6522), low 7 bits cleared
    CHECK_EQ("IER reset",  bus_read(14), 0x80);
    // IFR clean at reset
    CHECK_EQ("IFR reset",  bus_read(13), 0x00);
    CHECK_TRUE("irq low at reset", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 2 — write/read round-trip across writeable regs
// ════════════════════════════════════════════════════════════════════════
static bool test_reg_roundtrip() {
    reset();
    // DDRs first so we can set ORB/ORA bits that read back through the
    // output path (not the input side).
    bus_write(2, 0xFF);
    bus_write(3, 0xFF);
    bus_write(0, 0x5A); CHECK_EQ("ORB r/t",  bus_read(0),  0x5A);
    bus_write(1, 0xA5); CHECK_EQ("ORA r/t",  bus_read(1),  0xA5);
    bus_write(10, 0x3C); CHECK_EQ("SR  r/t", bus_read(10), 0x3C);
    bus_write(11, 0x55); CHECK_EQ("ACR r/t", bus_read(11), 0x55);
    bus_write(12, 0xAA); CHECK_EQ("PCR r/t", bus_read(12), 0xAA);
    bus_write(6, 0x78); CHECK_EQ("T1LL r/t", bus_read(6), 0x78);
    bus_write(7, 0x56); CHECK_EQ("T1LH r/t", bus_read(7), 0x56);
    bus_write(15, 0x33); CHECK_EQ("ORA-NH r/t", bus_read(15), 0x33);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 3 — T1 one-shot fires exactly once
// ════════════════════════════════════════════════════════════════════════
static bool test_t1_oneshot() {
    reset();
    bus_write(11, 0x00);          // ACR[7:6]=00 → one-shot, no PB7
    bus_write(6,  0x04);          // T1LL=4
    bus_write(5,  0x00);          // T1CH=0 → counter=0x0004, arm T1
    CHECK_EQ("IFR pre-tick", bus_read(13) & 0x40, 0x00);
    phi2_ticks(6);
    CHECK_EQ("IFR T1 after wrap", bus_read(13) & 0x40, 0x40);
    bus_write(13, 0x40);
    CHECK_EQ("IFR T1 cleared", bus_read(13) & 0x40, 0x00);
    phi2_ticks(40);
    CHECK_EQ("IFR T1 does NOT re-fire (one-shot)",
             bus_read(13) & 0x40, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 4 — T1 free-run + PB7 square wave (ACR[7:6]=11)
// ════════════════════════════════════════════════════════════════════════
static bool test_t1_freerun_pb7() {
    reset();
    bus_write(2, 0x00);           // DDRB=0 — PB7 is timer-driven anyway
    bus_write(11, 0xC0);          // ACR[7:6]=11 → free-run + PB7 toggle
    bus_write(6,  0x02);
    bus_write(5,  0x00);          // T1=0x0002, arm
    phi2_ticks(4);
    CHECK_EQ("IFR T1 1st wrap", bus_read(13) & 0x40, 0x40);
    // Sample PB7 via ORB read (reg 0).  DDRB[7]=0 so orb[7] is ignored;
    // our VIA2 overlays the timer output when ACR[7]=1.
    uint8_t pb7_a = (bus_read(0) >> 7) & 1;
    // Clear IFR_T1 and let it wrap again
    bus_write(13, 0x40);
    phi2_ticks(4);
    CHECK_EQ("IFR T1 2nd wrap (continuous)", bus_read(13) & 0x40, 0x40);
    uint8_t pb7_b = (bus_read(0) >> 7) & 1;
    CHECK_TRUE("PB7 square wave toggles", pb7_a != pb7_b);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 5 — T1 one-shot PB7 pulse (ACR[7:6]=10)
// Per 6522 datasheet: PB7 goes low at T1CH write (start of countdown)
// and returns high on underflow.  This is the corner that bit me —
// easy to invert the polarity if you read "pulse on wrap" too literally.
// ════════════════════════════════════════════════════════════════════════
static bool test_t1_oneshot_pb7() {
    reset();
    bus_write(11, 0x80);          // ACR[7:6]=10 → one-shot + PB7 pulse
    bus_write(6,  0x04);
    bus_write(5,  0x00);          // arm T1 — PB7 drops low now
    // Sample PB7 while counting: should be low.
    phi2_ticks(1);
    uint8_t pb7_cnt = (bus_read(0) >> 7) & 1;
    CHECK_EQ("PB7 low while counting", pb7_cnt, 0);
    // Let the counter underflow
    phi2_ticks(6);
    CHECK_EQ("IFR T1 fires once", bus_read(13) & 0x40, 0x40);
    uint8_t pb7_after = (bus_read(0) >> 7) & 1;
    CHECK_EQ("PB7 high after underflow", pb7_after, 1);
    // Ticks further — PB7 must stay high (no re-pulse).
    phi2_ticks(30);
    pb7_after = (bus_read(0) >> 7) & 1;
    CHECK_EQ("PB7 stays high (one-shot)", pb7_after, 1);
    // Clear IFR and confirm no re-fire.
    bus_write(13, 0x40);
    phi2_ticks(50);
    CHECK_EQ("IFR T1 no re-fire (one-shot)", bus_read(13) & 0x40, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 6 — T2 timed one-shot
// ════════════════════════════════════════════════════════════════════════
static bool test_t2_timed() {
    reset();
    bus_write(11, 0x00);          // ACR[5]=0 → T2 timed-interrupt
    bus_write(8,  0x03);          // T2 latch low
    bus_write(9,  0x00);          // T2CH=0, arm (T2=0x0003)
    CHECK_EQ("IFR T2 pre-tick", bus_read(13) & 0x20, 0x00);
    phi2_ticks(6);
    CHECK_EQ("IFR T2 after underflow", bus_read(13) & 0x20, 0x20);
    bus_write(13, 0x20);
    phi2_ticks(40);
    CHECK_EQ("IFR T2 stays cleared (one-shot)",
             bus_read(13) & 0x20, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 7 — Shift register shift-OUT: after 8 bits, IFR[SR] sets
// ════════════════════════════════════════════════════════════════════════
static bool test_sr_shift_out() {
    reset();
    // ACR[4:2] = 3'b110 → shift out under phi2
    bus_write(11, 0x18);
    CHECK_EQ("ACR set shift-out", bus_read(11), 0x18);
    bus_write(10, 0xA5);          // write SR → kick shift
    CHECK_EQ("IFR SR pre-shift", bus_read(13) & 0x04, 0x00);
    phi2_ticks(10);               // 8 bits + slack
    CHECK_EQ("IFR SR after 8-bit shift", bus_read(13) & 0x04, 0x04);
    // Reading SR clears IFR[SR]
    (void)bus_read(10);
    CHECK_EQ("IFR SR cleared by SR read", bus_read(13) & 0x04, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 8 — Shift register shift-IN under CB2 external
// ════════════════════════════════════════════════════════════════════════
static bool test_sr_shift_in() {
    reset();
    // ACR[4:2] = 3'b011 → shift-in under external clock
    bus_write(11, 0x0C);
    CHECK_EQ("ACR set shift-in", bus_read(11), 0x0C);
    dut->cb2_in = 1;              // hold external data line high
    bus_write(10, 0x00);          // writing SR kicks the shift counter
    phi2_ticks(10);
    CHECK_EQ("IFR SR after shift-in 8 bits",
             bus_read(13) & 0x04, 0x04);
    // After shift-in of all-1s, SR holds 0xFF.
    CHECK_EQ("SR holds shifted-in pattern", bus_read(10), 0xFF);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 9 — IFR/IER priority: IER masks IFR into the irq wire
// ════════════════════════════════════════════════════════════════════════
static bool test_ifr_ier_priority() {
    reset();
    bus_write(11, 0x00);
    bus_write(6,  0x01);
    bus_write(5,  0x00);          // arm T1
    phi2_ticks(3);
    CHECK_EQ("IFR T1 set", bus_read(13) & 0x40, 0x40);
    dut->eval();
    CHECK_TRUE("irq low (IER masked)", dut->irq == 0);
    // Enable T1 IRQ (IER bit 6 set, MSB=1)
    bus_write(14, 0xC0);
    CHECK_EQ("IER shows bit 6 set", bus_read(14), 0xC0);
    dut->eval();
    CHECK_TRUE("irq high (T1 enabled)", dut->irq == 1);
    // Clear IER bit 6 (MSB=0 → clear)
    bus_write(14, 0x40);
    dut->eval();
    CHECK_TRUE("irq low (T1 disabled)", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 10 — IFR write-1-clear semantics
// ════════════════════════════════════════════════════════════════════════
static bool test_ifr_write_1_clear() {
    reset();
    // Arm T1 + T2 together
    bus_write(11, 0x00);
    bus_write(6,  0x01); bus_write(5, 0x00);   // T1
    bus_write(8,  0x02); bus_write(9, 0x00);   // T2
    phi2_ticks(10);
    CHECK_EQ("both IFR set", bus_read(13) & 0x60, 0x60);
    // Write 1 to T1 bit only — T2 must survive
    bus_write(13, 0x40);
    CHECK_EQ("only T1 cleared", bus_read(13) & 0x60, 0x20);
    bus_write(13, 0x20);
    CHECK_EQ("T2 cleared too", bus_read(13) & 0x60, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 11 — PCR-independent IRQ clear semantics
// ════════════════════════════════════════════════════════════════════════
static bool test_pcr_independent_irq_clears() {
    reset();
    bus_write(12, 0x02);              // CA2 independent IRQ mode
    dut->ca2_in = 0;
    phi2_ticks(1);
    dut->ca2_in = 1;                  // CA2 edge
    phi2_ticks(1);
    CHECK_EQ("CA2 edge latched", bus_read(13) & 0x01, 0x01);
    (void)bus_read(1);
    CHECK_EQ("ORA preserves CA2 in independent IRQ mode",
             bus_read(13) & 0x03, 0x01);

    bus_write(12, 0x60);              // CB2 independent IRQ mode, positive edge
    dut->cb2_in = 0;
    phi2_ticks(1);
    dut->cb2_in = 1;                  // CB2 edge
    phi2_ticks(1);
    CHECK_EQ("CB2 edge latched", bus_read(13) & 0x08, 0x08);
    (void)bus_read(0);
    CHECK_EQ("ORB preserves CB2 in independent IRQ mode",
             bus_read(13) & 0x18, 0x08);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 12 — T2 pulse-count mode
// ════════════════════════════════════════════════════════════════════════
static bool test_t2_pulsecount() {
    reset();
    bus_write(11, 0x20);              // ACR[5]=1 → pulse-count mode
    bus_write(8,  0x01);
    bus_write(9,  0x00);              // arm with count=1
    CHECK_EQ("IFR T2 pre-pulse", bus_read(13) & 0x20, 0x00);

    dut->pb_in = 0xFF;
    tick();
    dut->pb_in = 0xBF;                // PB6 fall #1
    tick();
    CHECK_EQ("IFR T2 after first fall", bus_read(13) & 0x20, 0x00);

    dut->pb_in = 0xFF;
    tick();
    dut->pb_in = 0xBF;                // PB6 fall #2 underflows
    tick();
    CHECK_EQ("IFR T2 after second fall", bus_read(13) & 0x20, 0x20);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 12b/12c — T1CH/T2CH write races phi2_tick countdown wrap (T3)
//
// Regression test for the write-vs-countdown race: the CPU-write case and
// the phi2 countdown live in one always block with the countdown ordered
// LAST, so a T1CH/T2CH write landing on the EXACT same cycle as phi2_tick
// — with the OLD counter value at 0 — let the countdown's wrap branch
// override the CPU's freshly-loaded counter (non-blocking assignment
// ordering), spuriously set IFR, and disarm a one-shot the CPU just
// rearmed.  Must fail before the gate in via2.v is applied.
// ════════════════════════════════════════════════════════════════════════
static bool test_t1_write_vs_phi2_tick_race() {
    reset();

    bus_write(11, 0x00);   // ACR[7:6]=00 -> T1 one-shot
    bus_write(6,  0x02);   // T1LL
    bus_write(5,  0x00);   // T1CH write -> counter=0x0002, armed=1

    phi2_ticks(2);
    CHECK_EQ("T1C low = 0 before race", bus_read(4), 0x00);
    CHECK_EQ("T1C high = 0 before race", bus_read(5), 0x00);
    CHECK_EQ("IFR T1 still clear pre-race", bus_read(13) & 0x40, 0x00);

    bus_write(6, 0x37);    // T1LL = 0x37 for the racing write

    dut->pb_addr   = 5;
    dut->pb_wdata  = 0x00;
    dut->pb_wr     = 1;
    dut->pb_rd     = 0;
    dut->phi2_tick = 1;
    tick();
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
    dut->phi2_tick = 0;

    CHECK_EQ("T1C low holds new write post-race", bus_read(4), 0x37);
    CHECK_EQ("T1C high holds new write post-race", bus_read(5), 0x00);
    CHECK_EQ("IFR T1 not spuriously set by race", bus_read(13) & 0x40, 0x00);

    phi2_ticks(0x37 + 1);
    CHECK_EQ("IFR T1 fires once after rearmed count elapses",
             bus_read(13) & 0x40, 0x40);
    return true;
}

static bool test_t2_write_vs_phi2_tick_race() {
    reset();

    bus_write(11, 0x00);   // ACR[5]=0 -> T2 timed interrupt (one-shot)
    bus_write(8,  0x02);   // T2CL
    bus_write(9,  0x00);   // T2CH write -> counter=0x0002, armed=1

    phi2_ticks(2);
    CHECK_EQ("T2C low = 0 before race", bus_read(8), 0x00);
    CHECK_EQ("T2C high = 0 before race", bus_read(9), 0x00);
    CHECK_EQ("IFR T2 still clear pre-race", bus_read(13) & 0x20, 0x00);

    bus_write(8, 0x29);    // T2CL = 0x29 for the racing write

    dut->pb_addr   = 9;
    dut->pb_wdata  = 0x00;
    dut->pb_wr     = 1;
    dut->pb_rd     = 0;
    dut->phi2_tick = 1;
    tick();
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
    dut->phi2_tick = 0;

    CHECK_EQ("T2C low holds new write post-race", bus_read(8), 0x29);
    CHECK_EQ("T2C high holds new write post-race", bus_read(9), 0x00);
    CHECK_EQ("IFR T2 not spuriously set by race", bus_read(13) & 0x20, 0x00);

    phi2_ticks(0x29 + 1);
    CHECK_EQ("IFR T2 fires once after rearmed count elapses",
             bus_read(13) & 0x20, 0x20);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 13 — CA2/CB2 independent IRQ modes honor their PCR edge bits.
// PCR[2] selects CA2 positive edge when set; PCR[6] does the same for CB2.
// ════════════════════════════════════════════════════════════════════════
static bool test_ca2_cb2_independent_edge_polarity() {
    reset();

    bus_write(12, 0x02);              // CA2 independent IRQ, negative edge
    dut->ca2_in = 1;
    phi2_ticks(1);
    dut->ca2_in = 0;                  // selected falling edge
    phi2_ticks(1);
    CHECK_EQ("CA2 neg edge latched", bus_read(13) & 0x01, 0x01);
    bus_write(13, 0x01);
    dut->ca2_in = 1;                  // wrong edge for negative mode
    phi2_ticks(1);
    CHECK_EQ("CA2 pos edge ignored in neg mode", bus_read(13) & 0x01, 0x00);

    bus_write(12, 0x06);              // CA2 independent IRQ, positive edge
    dut->ca2_in = 0;
    phi2_ticks(1);
    dut->ca2_in = 1;                  // selected rising edge
    phi2_ticks(1);
    CHECK_EQ("CA2 pos edge latched", bus_read(13) & 0x01, 0x01);
    bus_write(13, 0x01);
    dut->ca2_in = 0;                  // wrong edge for positive mode
    phi2_ticks(1);
    CHECK_EQ("CA2 neg edge ignored in pos mode", bus_read(13) & 0x01, 0x00);

    bus_write(12, 0x20);              // CB2 independent IRQ, negative edge
    dut->cb2_in = 1;
    phi2_ticks(1);
    dut->cb2_in = 0;                  // selected falling edge
    phi2_ticks(1);
    CHECK_EQ("CB2 neg edge latched", bus_read(13) & 0x08, 0x08);
    bus_write(13, 0x08);
    dut->cb2_in = 1;                  // wrong edge for negative mode
    phi2_ticks(1);
    CHECK_EQ("CB2 pos edge ignored in neg mode", bus_read(13) & 0x08, 0x00);

    bus_write(12, 0x60);              // CB2 independent IRQ, positive edge
    dut->cb2_in = 0;
    phi2_ticks(1);
    dut->cb2_in = 1;                  // selected rising edge
    phi2_ticks(1);
    CHECK_EQ("CB2 pos edge latched", bus_read(13) & 0x08, 0x08);
    bus_write(13, 0x08);
    dut->cb2_in = 0;                  // wrong edge for positive mode
    phi2_ticks(1);
    CHECK_EQ("CB2 neg edge ignored in pos mode", bus_read(13) & 0x08, 0x00);

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 14 — NuBus slot-IRQ propagation: falling edge on PA[6:0]
// latches IFR[CA1] (negative-edge mode per PCR[0]=0, the Mac ROM default).
// ════════════════════════════════════════════════════════════════════════
static bool test_slot_irq_ca1() {
    reset();
    bus_write(12, 0x00);              // PCR[0]=0 → neg edge
    // Idle — no slot asserting
    dut->pa_in = 0xFF;
    phi2_ticks(2);
    CHECK_EQ("IFR CA1 idle clean", bus_read(13) & 0x02, 0x00);
    // Slot $B asserts (pulls PA[2] low)
    dut->pa_in = 0xFB;
    phi2_ticks(2);
    CHECK_EQ("IFR CA1 on slot-IRQ", bus_read(13) & 0x02, 0x02);
    // Port A read value should show the slot bit low
    // (DDRA=0 so PA[2] is input)
    CHECK_EQ("ORA reflects pa_in", bus_read(1) & 0x04, 0x00);
    // Reading ORA clears IFR[CA1] — handshake behaviour
    CHECK_EQ("IFR CA1 cleared by ORA read",
             bus_read(13) & 0x02, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 15 — CA1 edge polarity: PCR[0]=1 selects positive edge
// ════════════════════════════════════════════════════════════════════════
static bool test_ca1_edge_polarity() {
    reset();
    bus_write(12, 0x01);              // PCR[0]=1 → positive edge
    dut->pa_in = 0x00;                // all slots asserted (low)
    phi2_ticks(2);
    // Still neg edge so far — shouldn't latch CA1
    CHECK_EQ("IFR CA1 no pos edge yet", bus_read(13) & 0x02, 0x00);
    dut->pa_in = 0x01;                // PA[0] rose 0 → 1 (de-assert slot)
    phi2_ticks(2);
    CHECK_EQ("IFR CA1 latches on pos edge",
             bus_read(13) & 0x02, 0x02);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 16 — CB1 edge polarity: PCR[4]=1 selects positive edge
// ════════════════════════════════════════════════════════════════════════
static bool test_cb1_edge_polarity() {
    reset();
    bus_write(12, 0x00);              // PCR[4]=0 → negative edge
    dut->cb1_in = 1;                  // idle high
    phi2_ticks(2);
    dut->cb1_in = 0;                  // 1 → 0 transition
    phi2_ticks(2);
    CHECK_EQ("IFR CB1 latches on neg edge",
             bus_read(13) & 0x10, 0x10);
    bus_write(13, 0x10);
    CHECK_EQ("IFR CB1 cleared", bus_read(13) & 0x10, 0x00);

    bus_write(12, 0x10);              // PCR[4]=1 → positive edge
    dut->cb1_in = 0;
    phi2_ticks(2);
    CHECK_EQ("IFR CB1 no pos edge yet", bus_read(13) & 0x10, 0x00);
    dut->cb1_in = 1;                  // 0 → 1 transition
    phi2_ticks(2);
    CHECK_EQ("IFR CB1 latches on pos edge",
             bus_read(13) & 0x10, 0x10);
    (void)bus_read(0);                // ORB read clears CB1/CB2
    CHECK_EQ("IFR CB1 cleared by ORB read", bus_read(13) & 0x10, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 17 — ORA-handshake vs ORA-no-handshake (reg 1 vs reg 15)
//
// THIS IS THE CORNER I ALMOST GOT WRONG: reading reg 1 (ORA) clears
// IFR[CA1] + IFR[CA2] per the 6522 handshake protocol; reading reg 15
// (ORA-NH) returns the same data but does NOT touch IFR.  Getting this
// wrong would confuse any slot-interrupt handler that peeks PA while
// holding the CA1 edge pending for re-processing later.
// ════════════════════════════════════════════════════════════════════════
static bool test_ora_hs_vs_nh() {
    reset();
    bus_write(12, 0x00);              // neg-edge CA1
    dut->pa_in = 0xFF;
    phi2_ticks(2);
    dut->pa_in = 0xDF;                // slot $E (PA[5]) falls
    phi2_ticks(2);
    CHECK_EQ("IFR CA1 set before read", bus_read(13) & 0x02, 0x02);
    // Read ORA-NH first — must NOT clear IFR[CA1]
    uint8_t pa_nh = bus_read(15);
    CHECK_EQ("ORA-NH returns live PA", pa_nh, 0xDF);
    CHECK_EQ("IFR CA1 survives ORA-NH read",
             bus_read(13) & 0x02, 0x02);
    // Now read ORA (reg 1) — must clear IFR[CA1]
    uint8_t pa_hs = bus_read(1);
    CHECK_EQ("ORA returns live PA", pa_hs, 0xDF);
    CHECK_EQ("IFR CA1 cleared by ORA read",
             bus_read(13) & 0x02, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 18 — PA DDRA=0 returns pa_in (slot-sense read path)
// ════════════════════════════════════════════════════════════════════════
static bool test_port_a_input_reads() {
    reset();
    bus_write(3, 0x00);               // DDRA=0 — all inputs
    // Drive a distinctive slot-sense pattern.
    dut->pa_in = 0xC3;
    dut->eval();
    // Use ORA-NH (reg 15) to avoid clobbering IFR.
    CHECK_EQ("ORA-NH reflects pa_in full", bus_read(15), 0xC3);
    // With some DDRA bits asserted, output value wins on those bits.
    bus_write(3, 0xF0);               // top nibble = outputs
    bus_write(15, 0x5A);              // write ORA (no handshake)
    CHECK_EQ("ORA muxed",
             bus_read(15),
             (0x5A & 0xF0) | (0xC3 & 0x0F));
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 19 — An already-asserted slot line across reset still latches CA1
// ════════════════════════════════════════════════════════════════════════
static bool test_reset_asserted_slot_latches_ca1() {
    dut->rst       = 1;
    dut->phi2_tick = 0;
    dut->pb_addr   = 0;
    dut->pb_wdata  = 0;
    dut->pb_wr     = 0;
    dut->pb_rd     = 0;
    dut->pa_in     = 0xBF;         // PA6/video slot already asserted low
    dut->pb_in     = 0xFF;
    dut->ca1_in    = 0;
    dut->ca2_in    = 1;
    dut->cb1_in    = 1;
    dut->cb2_in    = 0;
    tick(); tick();
    dut->rst = 0;
    tick();

    // The external-pin edge latches sample every pb_clk, not on phi2_tick
    // (see the load-bearing comment in via2.v and scenario 24 below), so
    // an already-asserted slot line latches CA1 on the FIRST post-reset
    // clock — it no longer waits up to a phi2 period.  That ordering is
    // the whole point: an IFR flag latched later than the CPU access
    // meant to clear it strands forever in PCR independent mode.
    CHECK_EQ("IFR CA1 latches asserted slot on the first post-reset clock",
             bus_read(13) & 0x02, 0x02);
    phi2_ticks(1);
    CHECK_EQ("IFR CA1 stays latched across the phi2 boundary",
             bus_read(13) & 0x02, 0x02);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 20 — PA6 is the active-low built-in video slot IRQ source.
// Driving PA6 low must be visible through ORA/ORA-NH and latch CA1.
// ════════════════════════════════════════════════════════════════════════
static bool test_pa6_video_slot_ca1() {
    reset();
    bus_write(3, 0x00);               // DDRA=0 — all PA pins are inputs
    bus_write(12, 0x00);              // PCR[0]=0 — slot CA1 neg edge
    dut->pa_in = 0xFF;
    phi2_ticks(2);
    CHECK_EQ("PA6 idle high", bus_read(15) & 0x40, 0x40);
    CHECK_EQ("IFR CA1 idle clean", bus_read(13) & 0x02, 0x00);

    dut->pa_in = 0xBF;                // video slot $F asserted: PA6 low
    phi2_ticks(2);
    CHECK_EQ("PA6 reports video slot low", bus_read(15) & 0x40, 0x00);
    CHECK_EQ("PA6 latches slot CA1", bus_read(13) & 0x02, 0x02);

    dut->pa_in = 0xFF;                // video slot deasserted
    phi2_ticks(2);
    CHECK_EQ("PA6 returns high", bus_read(15) & 0x40, 0x40);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 21 — ORB write acknowledges PB-side handshakes.
//
// A plain ORB access is a PB-side handshake acknowledgement: it clears
// CB1 and CB2 when CB2 is not configured as an independent IRQ input.
// In independent IRQ mode, CB2 is a real interrupt source and must only
// clear through IFR write-1-clear.
// ════════════════════════════════════════════════════════════════════════
static bool test_orb_write_clears_cb_handshake() {
    reset();

    bus_write(12, 0x00);              // CB1/CB2 negative-edge handshake mode
    dut->cb1_in = 1;
    phi2_ticks(1);
    dut->cb1_in = 0;
    phi2_ticks(1);
    CHECK_EQ("CB1 edge latched before ORB write", bus_read(13) & 0x10, 0x10);

    bus_write(0, 0x5A);
    CHECK_EQ("ORB write clears CB1 handshake", bus_read(13) & 0x10, 0x00);

    bus_write(12, 0x20);              // CB2 independent IRQ, negative edge
    dut->cb2_in = 1;
    phi2_ticks(1);
    dut->cb2_in = 0;
    phi2_ticks(1);
    CHECK_EQ("CB2 independent edge latched", bus_read(13) & 0x08, 0x08);

    bus_write(0, 0xA5);
    CHECK_EQ("ORB write preserves independent CB2 IRQ",
             bus_read(13) & 0x08, 0x08);

    bus_write(13, 0x08);
    CHECK_EQ("CB2 independent clears by IFR write", bus_read(13) & 0x08, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 24 — a device IRQ edge must be latched into IFR BEFORE the CPU
//               access that services it (the 2026-09-15 53C96 IRQ strand)
// ════════════════════════════════════════════════════════════════════════
//
// WHAT THIS GUARDS
// ────────────────
// VIA2 CB2 is the 53C96 IRQ on the Q700 (fpga_top_peripherals.vh:1763,
// `.cb2_in(~scsi_irq_pb)`; MAME wires it identically,
// macquadra700.cpp:773).  The Mac programs PCR = 0x22 — CB2 INDEPENDENT,
// NEGATIVE edge — in which mode an ORB access does NOT clear IFR.CB2;
// only a write to IFR does.  So the 6522 converts a device LEVEL into an
// EDGE-latched flag that only software can retire.
//
// The driver's ISR/poll protocol, measured over a healthy 50-second MAME
// boot of System 7.5.3 on macqd700 (tap on VIA2 regs 0/C/D/E and the
// 53C96 register window), is ALWAYS:
//
//     C96  R reg4 -> 0x80        ; 53C96 status, S_INTR set: "is it mine?"
//     VIA2 W IFR  <- 0x88        ; clear IFR.CB2  -- and ONLY if S_INTR
//     C96  R reg6                ; seq step
//     C96  R reg5 -> 0x20        ; istatus read: the 53C96 IRQ line FALLS
//
// 9 of 9 CB2-driven interrupt entries in that boot took exactly that
// shape, and in every one the 53C96 reported S_INTR.  The driver NEVER
// clears IFR.CB2 on an interrupt the 53C96 does not claim.  An IFR.CB2
// flag that outlives its source is therefore PERMANENT: the ISR reads
// 53C96 status, sees 0x00, claims nothing, returns without writing IFR —
// and via2_irq (= |(ifr & ier), a LEVEL to the 68k) re-fires forever.
// Measured on silicon 2026-09-15: every one of the 32 exception-ring
// slots vec=0x1a at ~277k/s, VIA2 IFR=0xC8 IER=0x9A, 53C96 idle with
// irq=0 / istatus=0x00 / phase=BUS_FREE, SD completions frozen.
//
// The defect this test pins: the CA1/CA2/CB1/CB2 edge latches used to be
// sampled only on phi2_tick (783 360 Hz = 1.28 us = ~64 pb_clk cycles).
// A real 6522 is CLOCKED by phi2, so on a real Q700 every CPU access to
// the VIA costs a full phi2 cycle and the latch is necessarily ordered
// before the CPU access that follows the interrupt.  Here the CPU reaches
// VIA2 over pb_clk at 50 MHz, so the whole ISR above fits INSIDE one phi2
// period — and the deferred latch could land in the window between the
// driver's IFR clear and its istatus read.  The istatus read then removed
// the only evidence the flag was real.
//
// The invariant asserted below is the one that matters and is sampling-
// rate-agnostic:
//   (a) DELIVERY  — by the time the CPU can observe the interrupt through
//                   the device, VIA2 already holds the CB2 flag; and
//   (b) NO STRAND — after the source deasserts, no CB2 flag survives the
//                   clear that preceded it.
// The phi2 boundary is swept across every pb_clk offset of the ISR window
// so the test cannot pass by phase luck.
static uint32_t strand_phi2_cnt = 0;

static void strand_tick() {
    // Free-running phi2 NCO: one tick every 64 pb_clk cycles, the real
    // 50 MHz / 783 360 Hz ratio rounded (fpga_top_clocks.vh:1079).
    dut->phi2_tick = (strand_phi2_cnt == 0) ? 1 : 0;
    tick();
    dut->phi2_tick = 0;
    strand_phi2_cnt = (strand_phi2_cnt + 1) & 63;
}

static void strand_bus_write(uint8_t addr, uint8_t data) {
    dut->pb_addr = addr & 0xF; dut->pb_wdata = data;
    dut->pb_wr = 1; dut->pb_rd = 0;
    strand_tick();
    dut->pb_wr = 0; dut->pb_wdata = 0;
}

static uint8_t strand_bus_read(uint8_t addr) {
    dut->pb_addr = addr & 0xF; dut->pb_rd = 1; dut->pb_wr = 0;
    strand_tick();
    dut->pb_rd = 0; dut->eval();
    return dut->pb_rdata & 0xFF;
}

static bool test_scsi_irq_edge_never_strands_across_phi2_window() {
    for (int phase = 0; phase < 64; ++phase) {
        reset();
        strand_phi2_cnt = (uint32_t)phase;
        // Board posture: 53C96 IRQ idle (scsi_irq_pb=0 -> cb2_in=1),
        // PCR = 0x22, CB2 interrupt enabled in IER.
        dut->cb2_in = 1;
        strand_bus_write(12, 0x22);          // PCR
        strand_bus_write(14, 0x88);          // IER: enable CB2
        for (int i = 0; i < 160; i++) strand_tick();
        strand_bus_write(13, 0x7F);          // clear every IFR flag
        if (strand_bus_read(13) & 0x08) {
            printf("  FAIL phase=%d: IFR.CB2 set before the test even began\n",
                   phase);
            return false;
        }

        // ── the 53C96 raises its IRQ ────────────────────────────────────
        dut->cb2_in = 0;
        for (int i = 0; i < 3; i++) strand_tick();   // pb-side propagation

        // ── driver step 1: read the 53C96's own status (S_INTR).  By the
        //    time that access can complete, VIA2 must ALREADY hold the
        //    flag, or the clear that follows it clears nothing.
        for (int i = 0; i < 4; i++) strand_tick();   // the C96 reg-4 access
        uint8_t ifr_at_service = strand_bus_read(13);
        if (!(ifr_at_service & 0x08)) {
            printf("  FAIL phase=%d: CB2 flag not latched yet when the driver "
                   "reached the device (ifr=0x%02x) -- the clear below will "
                   "miss it\n", phase, ifr_at_service);
            return false;
        }

        // ── driver step 2: clear IFR.CB2 ───────────────────────────────
        strand_bus_write(13, 0x88);

        // ── driver steps 3+4: C96 reg-6 then reg-5.  The istatus read is
        //    what drops the 53C96's IRQ line.
        for (int i = 0; i < 10; i++) strand_tick();
        dut->cb2_in = 1;                              // istatus read
        for (int i = 0; i < 256; i++) strand_tick();  // several phi2 periods

        uint8_t ifr = strand_bus_read(13);
        if (ifr & 0x08) {
            printf("  FAIL phase=%d: IFR.CB2 STRANDED after the 53C96 "
                   "deasserted (ifr=0x%02x, via2_irq=%d) -- this is the "
                   "livelock\n", phase, ifr, (int)dut->irq);
            return false;
        }
        if (dut->irq) {
            printf("  FAIL phase=%d: via2_irq stuck high with the source "
                   "idle (ifr=0x%02x)\n", phase, ifr);
            return false;
        }
    }
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
    dut = new Vvia2;

    RUN(test_reset_reads);
    RUN(test_reg_roundtrip);
    RUN(test_t1_oneshot);
    RUN(test_t1_freerun_pb7);
    RUN(test_t1_oneshot_pb7);
    RUN(test_t2_timed);
    RUN(test_sr_shift_out);
    RUN(test_sr_shift_in);
    RUN(test_ifr_ier_priority);
    RUN(test_ifr_write_1_clear);
    RUN(test_pcr_independent_irq_clears);
    RUN(test_t2_pulsecount);
    RUN(test_t1_write_vs_phi2_tick_race);
    RUN(test_t2_write_vs_phi2_tick_race);
    RUN(test_ca2_cb2_independent_edge_polarity);
    RUN(test_slot_irq_ca1);
    RUN(test_ca1_edge_polarity);
    RUN(test_cb1_edge_polarity);
    RUN(test_ora_hs_vs_nh);
    RUN(test_port_a_input_reads);
    RUN(test_reset_asserted_slot_latches_ca1);
    RUN(test_pa6_video_slot_ca1);
    RUN(test_orb_write_clears_cb_handshake);
    RUN(test_scsi_irq_edge_never_strands_across_phi2_window);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
