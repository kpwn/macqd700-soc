// tb_via1.cpp — Verilator unit testbench for via1.v (full-boot variant)
//
// Build:   make tb-via1
// Scenarios:
//   1. Reset reads — ORB=0x08 (overlay live, external PB otherwise low), DDRB=0, etc.
//   2. Write/read round-trip on all 16 registers.
//   3. Overlay bit (output) is 1 at reset; stays 1 until DDRB[3]=1;
//      then follows ORB[3].  Decoupled from the ORB[3] CPU-read path.
//   3b. ORB[3] CPU-read follows the standard 6522 DDR mux: pb_in[3]
//      when DDRB[3]=0 (Q700 ADB-IRQ pin) and orb[3] when DDRB[3]=1
//      (classic-Mac overlay-clear readback).
//   4. DDRB-gated port B: outputs reflect ORB, inputs reflect pb_in.
//   5. RTC side-channel pins are DDR-gated and idle deselected at reset.
//   6. ORB[7:5] follow the normal 6522 DDR mux.
//   7. Timer 1 one-shot: counts down, fires IFR[6] exactly once.
//   8. Timer 1 continuous: auto-reload, re-fires.
//   9. IER masks IFR into irq output.
//  10. IFR write-1-clear; IER set/clear via MSB.
//  11. SR shift-in — ADB transceiver delivers a byte; IFR[2] sets,
//      SR register holds the byte, CPU can read it.
//  12. SR shift-out — CPU writes SR under TX ACR mode; adb_tx_valid
//      pulses with the written byte.
//  13. ADB empty-bus probe — SR receive mode completes with idle-high
//      0xFF and IFR[2] instead of spinning forever.
//  14. Timer 2 one-shot — underflow sets IFR[5], disarms.
//  15. 60 Hz VBL tick — IFR[CA1] sets after VBL_PHI2_DIV phi2 ticks;
//      ORA (reg 1) acknowledges it, ORB/reg 15 do not.
//  16. Timer 2 pulse-count — PB6 falling edges decrement and underflow.
//  17. RTC Read-Seconds — full 4-byte shift-out matches the counter.
//  18. IRQ aggregation — multiple IFR bits gated by IER -> irq line.
//  19. ADB SR write re-arms empty-bus receive while ACR remains RX.
//  20. External DAFB vblank IRQ edge latches CA1 and drives irq.
//  21. RTC CKO feeds VIA1 CA2 and obeys PCR edge/ack semantics.
//  22. ADB modem idle CB2 edge latches VIA1 CB2 and honors independent IRQ
//      clear semantics.
//
// phi2_tick is driven manually; each `phi2_ticks(N)` call issues N
// pulses, one per clock edge.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vvia1.h"

static Vvia1*   dut       = nullptr;
static uint64_t sim_time  = 0;
static int      n_pass    = 0;
static int      n_fail    = 0;

static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void reset() {
    dut->rst          = 1;
    dut->phi2_tick    = 0;
    dut->pb_addr      = 0;
    dut->pb_wdata     = 0;
    dut->pb_wr        = 0;
    dut->pb_rd        = 0;
    dut->pa_in        = 0;
    dut->pb_in        = 0;
    dut->adb_rx_byte  = 0;
    dut->adb_rx_valid = 0;
    dut->rtc_data_i   = 0;
    dut->rtc_cko      = 1;
    dut->vblank_irq_in = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
}

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

// Write: one-cycle pulse.  No phi2 tick inside bus access.
static void bus_write(uint8_t addr, uint8_t data) {
    dut->pb_addr  = addr & 0xF;
    dut->pb_wdata = data;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    dut->phi2_tick = 0;
    tick();
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
}

static uint8_t bus_read(uint8_t addr) {
    dut->pb_addr   = addr & 0xF;
    dut->pb_rd     = 1;
    dut->pb_wr     = 0;
    dut->phi2_tick = 0;
    tick();
    dut->pb_rd     = 0;
    dut->eval();           // propagate registered pb_rdata
    return dut->pb_rdata & 0xFF;
}

// Issue `n` phi2_tick pulses (one per cycle).
static void phi2_ticks(uint32_t n) {
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    for (uint32_t i = 0; i < n; i++) {
        dut->phi2_tick = 1;
        tick();
        dut->phi2_tick = 0;
    }
}

static void fire_t1(uint8_t count) {
    bus_write(11, bus_read(11) & 0xBF);  // T1 one-shot
    bus_write(6, count);
    bus_write(5, 0x00);
    phi2_ticks((uint32_t)count + 2);
}

static void fire_t2(uint8_t count) {
    bus_write(11, bus_read(11) & 0xDF);  // T2 one-shot
    bus_write(8, count);
    bus_write(9, 0x00);
    phi2_ticks((uint32_t)count + 2);
}

static void fire_ca1(void) {
    bus_write(12, (bus_read(12) | 0x01) & 0xF1);  // CA1 positive edge
    dut->vblank_irq_in = 1;
    tick();
}

static void fire_ca2(void) {
    bus_write(12, bus_read(12) & 0xF0);  // CA2 input, negative edge, handshake
    dut->rtc_cko = 0;
    tick();
}

// ─── Scenario 1: reset reads ────────────────────────────────────────────
static bool test_reset_reads() {
    reset();
    // pb_in[3] = ~adb_irq_pending in the Q700 platform wiring (see
    // rtl/fpga_top_peripherals.vh and macquadra700.cpp::via_in_b).
    // Reset baseline: pb_in=0 in this unit tb, so ORB[3] reads 0.  The
    // overlay_bit output is decoupled and still asserts 1 (live latch).
    CHECK_EQ("ORB reset",   bus_read(0),  0x00);
    CHECK_TRUE("overlay latch asserted at reset", dut->overlay_bit == 1);
    CHECK_EQ("ORA reset",   bus_read(1),  0x00);
    CHECK_EQ("DDRB reset",  bus_read(2),  0x00);
    CHECK_EQ("DDRA reset",  bus_read(3),  0x00);
    CHECK_EQ("ACR reset",   bus_read(11), 0x00);
    CHECK_EQ("PCR reset",   bus_read(12), 0x00);
    CHECK_EQ("IFR reset",   bus_read(13), 0x00);
    CHECK_EQ("IER reset",   bus_read(14), 0x80);
    CHECK_TRUE("irq low at reset", dut->irq == 0);
    return true;
}

// ─── Scenario 2: round-trip on writeable regs ──────────────────────────
static bool test_reg_roundtrip() {
    reset();
    bus_write(3, 0xFF);
    bus_write(2, 0x1F);
    bus_write(0, 0x1A);
    CHECK_EQ("ORB r/t", bus_read(0), 0x1A);
    bus_write(1, 0xA5);
    CHECK_EQ("ORA r/t", bus_read(1), 0xA5);
    CHECK_EQ("DDRA r/t", bus_read(3), 0xFF);
    CHECK_EQ("DDRB r/t", bus_read(2), 0x1F);

    bus_write(10, 0x5A); CHECK_EQ("SR r/t",  bus_read(10), 0x5A);
    bus_write(11, 0x3C); CHECK_EQ("ACR r/t", bus_read(11), 0x3C);
    bus_write(12, 0xC3); CHECK_EQ("PCR r/t", bus_read(12), 0xC3);

    bus_write(6, 0x78);
    bus_write(7, 0x56);
    CHECK_EQ("T1LL r/t", bus_read(6), 0x78);
    CHECK_EQ("T1LH r/t", bus_read(7), 0x56);

    bus_write(15, 0x33);
    CHECK_EQ("ORA-NH r/t", bus_read(15), 0x33);
    return true;
}

// ─── Scenario 3: overlay bit (overlay_bit OUTPUT path, decoder side) ────
//
// The overlay_bit output is what glue.v / xbar consume to decide whether
// low memory aliases to ROM.  It MUST stay asserted (=1) at reset and
// while DDRB[3]=0, so the CPU sees ROM at vector 0 even before it has
// driven the overlay latch.  Once DDRB[3]=1, overlay_bit follows orb[3].
//
// This is now decoupled from the ORB[3] read path — see test_orb_pb3_*
// scenarios for the read-side (Q700 ADB-IRQ-pin) contract.
static bool test_overlay_bit() {
    reset();
    CHECK_TRUE("overlay=1 @reset", dut->overlay_bit == 1);
    bus_write(0, 0x00);
    dut->eval();
    CHECK_TRUE("overlay=1 before DDR output", dut->overlay_bit == 1);
    bus_write(2, 0x08);
    dut->eval();
    CHECK_TRUE("overlay=0 after ORB[3]=0 & DDR[3]=1", dut->overlay_bit == 0);
    bus_write(0, 0x08);
    dut->eval();
    CHECK_TRUE("overlay=1 after ORB[3]=1 & DDR[3]=1", dut->overlay_bit == 1);
    return true;
}

// ─── Scenario 3b: ORB[3] read path follows DDR mux (Q700 ADB-IRQ pin) ───
//
// Q700 wires VIA1 PB3 to the ADB modem's irq_pending output (active-low):
// pb_in[3] = ~adb_irq_pending.  Per macquadra700.cpp::via_in_b:
//   if (!m_adb_irq_pending) val |= 0x08;
// So while DDRB[3]=0 (the post-reset Q700 direction once the ROM clears
// the overlay latch via DDRB[3]=1; orb[3]=0; DDRB[3]=0 again, OR equivalently
// the platform never drives PB3 because Q700 has no overlay flop on this
// pin) the CPU read of ORB[3] must mirror pb_in[3], not the internal
// overlay latch.  Verifies the bit is decoupled from overlay_bit.
static bool test_orb_pb3_adb_irq_input() {
    reset();
    // DDRB[3]=0, pb_in[3]=0 (ADB IRQ pending — modem active-low asserted)
    dut->pb_in = 0x00;
    CHECK_EQ("ORB[3] reads pb_in[3]=0 in input mode", bus_read(0) & 0x08, 0x00);
    // pb_in[3]=1 (ADB IRQ idle — modem deasserted; bit reads HIGH per MAME)
    dut->pb_in = 0x08;
    CHECK_EQ("ORB[3] reads pb_in[3]=1 in input mode", bus_read(0) & 0x08, 0x08);
    // Overlay bit is independent of the read path: still asserted (=1)
    // because DDRB[3]=0 means overlay_live = 1.
    CHECK_TRUE("overlay_bit unaffected by pb_in[3] toggling",
               dut->overlay_bit == 1);

    // Drive DDRB[3]=1, ORB[3]=0 (classic-Mac overlay-clear sequence).
    bus_write(2, 0x08);
    bus_write(0, 0x00);
    dut->eval();
    CHECK_TRUE("overlay=0 after ddrb[3]=1, orb[3]=0", dut->overlay_bit == 0);
    // pb_in[3] toggling no longer affects the read because DDRB[3]=1.
    dut->pb_in = 0x08;
    CHECK_EQ("ORB[3] reads orb[3]=0 in output mode (overlay clear)",
             bus_read(0) & 0x08, 0x00);
    dut->pb_in = 0x00;
    CHECK_EQ("ORB[3] still reads orb[3]=0 in output mode (pb_in irrelevant)",
             bus_read(0) & 0x08, 0x00);

    // Drive DDRB[3]=1, ORB[3]=1 (re-asserting overlay).
    bus_write(0, 0x08);
    dut->eval();
    CHECK_TRUE("overlay=1 after orb[3]=1 in output mode",
               dut->overlay_bit == 1);
    CHECK_EQ("ORB[3] reads orb[3]=1 in output mode",
             bus_read(0) & 0x08, 0x08);

    // Tristate again (DDRB[3]=0): bit should follow pb_in[3] again.
    bus_write(2, 0x00);
    dut->pb_in = 0x08;
    CHECK_EQ("ORB[3] back to pb_in[3]=1 after retristate",
             bus_read(0) & 0x08, 0x08);
    dut->pb_in = 0x00;
    CHECK_EQ("ORB[3] back to pb_in[3]=0 after retristate",
             bus_read(0) & 0x08, 0x00);
    return true;
}

// ─── Scenario 4: DDR-gated port B mux ──────────────────────────────────
static bool test_port_b_mux() {
    reset();
    bus_write(2, 0x0F);
    bus_write(0, 0x05);
    dut->pb_in = 0xA0;
    CHECK_EQ("ORB muxed", bus_read(0), 0xA5);
    return true;
}

// ─── Scenario 5: RTC side-channel visibility ─────────────────────────
static bool test_rtc_sidechannel_visibility() {
    reset();
    CHECK_EQ("RTC selected at reset", dut->rtc_enb, 0);
    CHECK_EQ("RTC clk idle at reset", dut->rtc_clk, 0);
    CHECK_EQ("RTC data latch active at reset", dut->rtc_data_oe, 1);

    bus_write(0, 0x00);
    CHECK_EQ("RTC enabled low from ORB[2]", dut->rtc_enb, 0);
    CHECK_EQ("RTC clk low", dut->rtc_clk, 0);
    CHECK_EQ("RTC data out low", dut->rtc_data_o, 0);
    CHECK_EQ("RTC data latch active while selected", dut->rtc_data_oe, 1);

    bus_write(2, 0x01);  // PB0 output; PB1/PB2 still use ORB for RTC side-channel.
    CHECK_EQ("RTC data output enabled by DDRB[0]", dut->rtc_data_oe, 1);

    bus_write(0, 0x07);
    CHECK_EQ("RTC disabled high from ORB[2]", dut->rtc_enb, 1);
    CHECK_EQ("RTC clk high from ORB[1]", dut->rtc_clk, 1);
    CHECK_EQ("RTC data out high", dut->rtc_data_o, 1);

    bus_write(2, 0x00);  // PB0 input
    dut->pb_in = 0x01;
    CHECK_EQ("RTC data input visible through ORB", bus_read(0) & 0x01, 0x01);
    dut->pb_in = 0x00;
    CHECK_EQ("RTC data input low visible through ORB", bus_read(0) & 0x01, 0x00);

    std::printf("  via1 firstlight summary: rtc_enb=%u rtc_clk=%u rtc_data_oe=%u\n",
                (unsigned)dut->rtc_enb,
                (unsigned)dut->rtc_clk,
                (unsigned)dut->rtc_data_oe);
    return true;
}

// ─── Scenario 5: high PB bits follow DDR mux ───────────────────────────
//
// pb_in=0xA0 with DDRB=0: every bit follows the plain 6522 DDR mux —
// inputs read the pin, outputs read the orb latch (matches MAME's
// via6522_device).  bit 0 follows pb_in; pb_in[3]=0 → ORB[3]=0 (Q700
// ADB-IRQ-pin contract); pb_in[7:5]=101 → bits 7,5 visible.  See
// test_orb_rtc_bits_ddr_mux for the bits-1,2 (RTC clk/enb) corner.
static bool test_orb_high_bits_follow_ddr_mux() {
    reset();
    dut->pb_in = 0xA0;
    CHECK_EQ("ORB high input bits visible (pb_in[3]=0 reads 0)",
             bus_read(0), 0xA0);

    // Force pb_in[3]=1 (ADB IRQ idle) and re-check.
    dut->pb_in = 0xA8;
    CHECK_EQ("ORB[3] reads pb_in[3]=1 (ADB modem idle)",
             bus_read(0), 0xA8);

    bus_write(2, 0xE0);
    bus_write(0, 0x40);
    dut->pb_in = 0xA0;
    // DDRB[3]=0 still, so bit 3 follows pb_in[3]=0.  Bits 7,5 are now
    // outputs from ORB write 0x40 → bits 7=0, 5=0.  pb_in[7:5] tristated.
    // Bits 1,2 are DDRB inputs (0xE0[1:2]=0) → read pin pb_in[2:1]=0.
    // bit 0 follows pb_in (0).
    CHECK_EQ("ORB high output bits visible (pb_in[3]=0)",
             bus_read(0), 0x40);
    return true;
}

// ─── Scenario 5b: ORB bits 1,2 (RTC clk/enb) follow the plain DDR mux ──
//
// Regression guard for the orb_rd fix — matches MAME's via6522_device
// golden model (`make tb-via1-lockstep` first divergence was event #55,
// an ORB read with DDRB[1:2]=input).  The previous RTL overrode ORB[2:1]
// to always return the orb latch; MAME returns the pin.  The RTC
// clk/enb side-channel still reads the orb latch directly and is
// unaffected by DDR state — checked alongside.
static bool test_orb_rtc_bits_ddr_mux() {
    reset();
    // Write the orb latch bits 1,2 high (orb is the latch — DDRB does
    // not gate the write).  rtc_clk/rtc_enb track the latch directly.
    bus_write(0, 0x06);
    CHECK_EQ("RTC clk side-channel = orb[1]", dut->rtc_clk, 1);
    CHECK_EQ("RTC enb side-channel = orb[2]", dut->rtc_enb, 1);

    // DDRB[1:2]=input, pin low → ORB read of bits 1,2 = pin = 0, even
    // though the orb latch holds them high.
    bus_write(2, 0x00);
    dut->pb_in = 0x00;
    CHECK_EQ("ORB[2:1] read = pin(0) when DDRB[1:2]=input",
             bus_read(0) & 0x06, 0x00);
    CHECK_EQ("RTC clk still = orb latch (DDR-independent)", dut->rtc_clk, 1);
    CHECK_EQ("RTC enb still = orb latch (DDR-independent)", dut->rtc_enb, 1);

    // Pin high → ORB read of bits 1,2 = pin = 1.
    dut->pb_in = 0x06;
    CHECK_EQ("ORB[2:1] read = pin(1) when DDRB[1:2]=input",
             bus_read(0) & 0x06, 0x06);

    // DDRB[1:2]=output → ORB read of bits 1,2 = orb latch, pin ignored.
    bus_write(2, 0x06);
    dut->pb_in = 0x00;
    CHECK_EQ("ORB[2:1] read = orb latch when DDRB[1:2]=output",
             bus_read(0) & 0x06, 0x06);
    return true;
}

// ─── Scenario 6: T1 one-shot ───────────────────────────────────────────
static bool test_t1_oneshot() {
    reset();
    bus_write(11, 0x00);
    bus_write(6, 0x05);
    bus_write(5, 0x00);
    CHECK_EQ("IFR pre-tick",  bus_read(13) & 0x40, 0x00);
    phi2_ticks(6);
    CHECK_EQ("IFR T1 after wrap", bus_read(13) & 0x40, 0x40);
    bus_write(13, 0x40);
    CHECK_EQ("IFR T1 cleared", bus_read(13) & 0x40, 0x00);
    phi2_ticks(50);
    CHECK_EQ("IFR T1 stays cleared (one-shot)", bus_read(13) & 0x40, 0x00);
    return true;
}

// ─── Scenario 7: T1 continuous ─────────────────────────────────────────
static bool test_t1_continuous() {
    reset();
    bus_write(11, 0x40);
    bus_write(6, 0x03);
    bus_write(5, 0x00);
    phi2_ticks(5);
    CHECK_EQ("IFR T1 after first wrap", bus_read(13) & 0x40, 0x40);
    bus_write(13, 0x40);
    CHECK_EQ("IFR T1 cleared mid-run", bus_read(13) & 0x40, 0x00);
    phi2_ticks(6);
    CHECK_EQ("IFR T1 re-fires (continuous)", bus_read(13) & 0x40, 0x40);
    return true;
}

// ─── Scenario 8: IER masks IRQ ─────────────────────────────────────────
static bool test_ier_masks_irq() {
    reset();
    bus_write(11, 0x00);
    bus_write(6, 0x01);
    bus_write(5, 0x00);
    phi2_ticks(3);
    CHECK_EQ("IFR T1 set", bus_read(13) & 0x40, 0x40);
    dut->eval();
    CHECK_TRUE("irq low (IER masked)", dut->irq == 0);
    bus_write(14, 0xC0);
    CHECK_EQ("IER shows bit 6 set", bus_read(14), 0xC0);
    dut->eval();
    CHECK_TRUE("irq high (T1 enabled)", dut->irq == 1);
    bus_write(14, 0x40);
    dut->eval();
    CHECK_TRUE("irq low (T1 disabled)", dut->irq == 0);
    return true;
}

// ─── Scenario 9: IFR write-1-clear ─────────────────────────────────────
static bool test_ifr_write_clear() {
    reset();
    bus_write(11, 0x00);
    bus_write(6, 0x01);
    bus_write(5, 0x00);
    phi2_ticks(3);
    CHECK_EQ("IFR T1 set", bus_read(13) & 0x40, 0x40);
    bus_write(13, 0x20);
    CHECK_EQ("IFR T1 still set", bus_read(13) & 0x40, 0x40);
    bus_write(13, 0x40);
    CHECK_EQ("IFR T1 cleared", bus_read(13) & 0x40, 0x00);
    return true;
}

static bool test_ifr_write_ff_clears_all_and_deasserts_irq() {
    reset();

    fire_t1(0x01);
    fire_t2(0x01);
    fire_ca1();
    fire_ca2();

    bus_write(14, 0xE2);  // enable T1, T2, CA1 (Q700 Level-1 mask)
    CHECK_EQ("Q700 IRQ storm seed IFR", bus_read(13), 0xE3);
    dut->eval();
    CHECK_TRUE("irq high before IFR #$ff ack", dut->irq == 1);

    bus_write(13, 0xFF);
    CHECK_EQ("IFR #$ff clears all pending flags", bus_read(13), 0x00);
    dut->eval();
    CHECK_TRUE("irq low after all enabled IFR bits clear", dut->irq == 0);
    return true;
}

static bool test_timer_counter_access_clears_ifr() {
    reset();

    fire_t1(0x02);
    CHECK_EQ("IFR T1 set before T1CL read", bus_read(13) & 0x40, 0x40);
    (void)bus_read(4);
    CHECK_EQ("T1CL read clears IFR T1", bus_read(13) & 0x40, 0x00);

    fire_t1(0x02);
    CHECK_EQ("IFR T1 set before T1CH write", bus_read(13) & 0x40, 0x40);
    bus_write(5, 0x00);
    CHECK_EQ("T1CH write clears IFR T1", bus_read(13) & 0x40, 0x00);

    fire_t2(0x02);
    CHECK_EQ("IFR T2 set before T2CL read", bus_read(13) & 0x20, 0x20);
    (void)bus_read(8);
    CHECK_EQ("T2CL read clears IFR T2", bus_read(13) & 0x20, 0x00);

    fire_t2(0x02);
    CHECK_EQ("IFR T2 set before T2CH write", bus_read(13) & 0x20, 0x20);
    bus_write(9, 0x00);
    CHECK_EQ("T2CH write clears IFR T2", bus_read(13) & 0x20, 0x00);
    return true;
}

// ─── Scenario 10: ORA/ORB handshake read clears ────────────────────────
static bool test_port_handshake_clears() {
    reset();

    fire_ca1();
    fire_ca2();
    CHECK_EQ("CA1/CA2 set before ORA-NH write", bus_read(13) & 0x03, 0x03);
    bus_write(15, 0x55);
    CHECK_EQ("ORA-NH write does not clear CA1/CA2", bus_read(13) & 0x03, 0x03);
    bus_write(1, 0xAA);
    CHECK_EQ("ORA write clears CA1/CA2", bus_read(13) & 0x03, 0x00);

    dut->vblank_irq_in = 0;
    dut->rtc_cko = 1;
    tick();
    fire_ca1();
    fire_ca2();
    CHECK_EQ("CA1/CA2 set before ORA-NH read", bus_read(13) & 0x03, 0x03);
    (void)bus_read(15);
    CHECK_EQ("ORA-NH read does not clear CA1/CA2", bus_read(13) & 0x03, 0x03);
    (void)bus_read(1);
    CHECK_EQ("ORA read clears CA1/CA2", bus_read(13) & 0x03, 0x00);
    return true;
}

// ─── Scenario 11: T2 pulse-count mode ────────────────────────────────
static bool test_t2_pulsecount() {
    reset();
    bus_write(11, 0x20);          // ACR[5]=1 → pulse-count mode
    bus_write(8,  0x01);
    bus_write(9,  0x00);          // arm with count=1
    CHECK_EQ("IFR T2 pre-pulse", bus_read(13) & 0x20, 0x00);

    dut->pb_in = 0x40;            // PB6 high
    tick();
    dut->pb_in = 0x00;            // first falling edge
    tick();
    CHECK_EQ("IFR T2 after first fall", bus_read(13) & 0x20, 0x00);

    dut->pb_in = 0x40;
    tick();
    dut->pb_in = 0x00;            // second falling edge underflows
    tick();
    CHECK_EQ("IFR T2 after second fall", bus_read(13) & 0x20, 0x20);
    return true;
}

// ─── Scenario 11b/11c: T1CH/T2CH write races phi2_tick countdown wrap ──
//
// Regression test for the write-vs-countdown race (T3 fix): the CPU-write
// case and the phi2 countdown live in one always block with the countdown
// ordered LAST, so a T1CH/T2CH write landing on the EXACT same cycle as
// phi2_tick — with the OLD counter value at 0 — let the countdown's wrap
// branch override the CPU's freshly-loaded counter (non-blocking
// assignment ordering), spuriously set IFR, and disarm a one-shot the CPU
// just rearmed.  Must fail before the gate in via1.v is applied.
static bool test_t1_write_vs_phi2_tick_race() {
    reset();

    // Arm T1 one-shot, latch low = 0x02: counter starts at 2.
    bus_write(11, 0x00);   // ACR: T1 one-shot
    bus_write(6,  0x02);   // T1LL
    bus_write(5,  0x00);   // T1CH write -> counter=0x0002, armed=1

    // Drain to counter == 0 with plain phi2 ticks (no write in flight).
    phi2_ticks(2);
    CHECK_EQ("T1C low = 0 before race", bus_read(4), 0x00);
    CHECK_EQ("T1C high = 0 before race", bus_read(5), 0x00);
    CHECK_EQ("IFR T1 still clear pre-race", bus_read(13) & 0x40, 0x00);

    // Pre-load a distinctive new low latch byte for the racing write.
    bus_write(6, 0x37);    // T1LL = 0x37

    // Race: CPU writes T1CH (arms counter = {0x00, 0x37}) on the EXACT
    // same cycle phi2_tick fires while the old counter (0x0000) is about
    // to wrap.
    dut->pb_addr   = 5;
    dut->pb_wdata  = 0x00;
    dut->pb_wr     = 1;
    dut->pb_rd     = 0;
    dut->phi2_tick = 1;
    tick();
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
    dut->phi2_tick = 0;

    // (a) counter holds the newly written value, not the wrap's 0xFFFF.
    CHECK_EQ("T1C low holds new write post-race", bus_read(4), 0x37);
    CHECK_EQ("T1C high holds new write post-race", bus_read(5), 0x00);
    // (b) no spurious IFR_T1 from the countdown's wrap branch.
    CHECK_EQ("IFR T1 not spuriously set by race", bus_read(13) & 0x40, 0x00);

    // (c) one-shot stays armed: the freshly-loaded count (0x0037) must
    // still reach underflow and fire IFR_T1 exactly once at the right
    // time (V+1 ticks, matching test_t1_oneshot's timing convention).
    phi2_ticks(0x37 + 1);
    CHECK_EQ("IFR T1 fires once after rearmed count elapses",
             bus_read(13) & 0x40, 0x40);
    return true;
}

static bool test_t2_write_vs_phi2_tick_race() {
    reset();

    // Arm T2 one-shot (ACR[5]=0 by default), latch low = 0x02.
    bus_write(8, 0x02);    // T2CL
    bus_write(9, 0x00);    // T2CH write -> counter=0x0002, armed=1

    phi2_ticks(2);
    CHECK_EQ("T2C low = 0 before race", bus_read(8), 0x00);
    CHECK_EQ("T2C high = 0 before race", bus_read(9), 0x00);
    CHECK_EQ("IFR T2 still clear pre-race", bus_read(13) & 0x20, 0x00);

    // Pre-load a distinctive new low latch byte for the racing write.
    bus_write(8, 0x29);    // T2CL = 0x29

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

// ─── T1 IFR semantics bundle (per Synertek 6522 datasheet §3.3) ─────────
//
// Three scenarios grouped here:
//   (a) T1 one-shot underflow sets IFR[6] exactly once and disarms; any
//       further phi2 ticks must NOT re-latch IFR[6] even if the counter
//       wraps back through 0 while free-running.
//   (b) Explicit "write 1 to bit 6" of IFR clears IFR[6]; writing 0 does
//       NOT clear it (only the 1-bits clear — 6522 write-1-clear).
//   (c) Read of T1CL (register 4) clears IFR[6] — this is the standard
//       ACK-by-read path the Mac ROM uses.
//
// Also validates that after a post-underflow clear, T1 is stable — no
// phantom re-fires even if the counter wraps to 0 again in one-shot mode.
static bool test_t1_ifr_oneshot_underflow() {
    reset();

    // ACR=0x00 → T1 one-shot.  Latch = 0x0003 → counter wraps on 4th tick.
    bus_write(11, 0x00);
    bus_write(6,  0x03);   // T1LL
    bus_write(5,  0x00);   // T1CH write arms T1, counter = 0x0003

    CHECK_EQ("IFR T1 pre-tick", bus_read(13) & 0x40, 0x00);

    // Pump: 3→2, 2→1, 1→0, 0→wrap (set IFR, reload 0xFFFF, disarm).
    phi2_ticks(4);
    CHECK_EQ("IFR T1 latched on underflow", bus_read(13) & 0x40, 0x40);

    return true;
}

static bool test_t1_ifr_write_one_clear() {
    reset();

    // Fire T1 to set IFR[6].
    bus_write(11, 0x00);
    bus_write(6,  0x01);
    bus_write(5,  0x00);
    phi2_ticks(3);
    CHECK_EQ("IFR T1 set", bus_read(13) & 0x40, 0x40);

    // Write 0 to bit 6: should NOT clear it (write-1-clear).
    bus_write(13, 0x00);
    CHECK_EQ("IFR T1 still set after write-0", bus_read(13) & 0x40, 0x40);

    // Write bit 5 only (T2): IFR[6] unchanged.
    bus_write(13, 0x20);
    CHECK_EQ("IFR T1 unchanged by T2-bit-only write", bus_read(13) & 0x40, 0x40);

    // Write 1 to bit 6: clears.
    bus_write(13, 0x40);
    CHECK_EQ("IFR T1 cleared by write-1-to-clear", bus_read(13) & 0x40, 0x00);

    // One-shot already disarmed; 100 more ticks must not re-fire.
    phi2_ticks(100);
    CHECK_EQ("IFR T1 stays cleared (one-shot, disarmed)",
             bus_read(13) & 0x40, 0x00);

    return true;
}

static bool test_t1_ifr_read_t1cl_clears() {
    reset();

    // Fire T1 once.
    bus_write(11, 0x00);
    bus_write(6,  0x02);
    bus_write(5,  0x00);
    phi2_ticks(4);
    CHECK_EQ("IFR T1 latched before T1CL read",
             bus_read(13) & 0x40, 0x40);

    // Reading T1CL (reg 4) should clear IFR[6] (6522 ACK semantics).
    (void)bus_read(4);
    CHECK_EQ("IFR T1 cleared after T1CL read",
             bus_read(13) & 0x40, 0x00);

    // Reading T1CH (reg 5) is NOT an ACK — should not re-set or clear.
    (void)bus_read(5);
    CHECK_EQ("IFR T1 still cleared after T1CH read",
             bus_read(13) & 0x40, 0x00);

    return true;
}

// ─── Scenario 12: SR shift-in from ADB transceiver ─────────────────────
static bool test_sr_shift_in() {
    reset();

    // ACR[4:2] = 3'b011  → SR-in under external clock.
    bus_write(11, 0x0C);   // ACR bit 3,2 = 11 (shift-in under ext)
    CHECK_EQ("ACR set shift-in", bus_read(11), 0x0C);
    bus_write(14, 0x84);    // enable SR interrupt, as ROM ADB polls do

    // Drive transceiver interface: byte 0x3C, valid high.
    dut->adb_rx_byte  = 0x3C;
    dut->adb_rx_valid = 1;

    // One phi2_tick latches it.
    phi2_ticks(1);
    dut->adb_rx_valid = 0;

    // IFR bit 2 (SR) set BEFORE reading SR (because reading SR clears it
    // per 6522).  Then SR should hold 0x3C.
    CHECK_EQ("IFR SR set after rx byte",  bus_read(13) & 0x04, 0x04);
    CHECK_EQ("IFR summary set for enabled SR", bus_read(13) & 0x84, 0x84);
    dut->eval();
    CHECK_TRUE("irq high for enabled SR", dut->irq == 1);
    CHECK_EQ("SR holds adb byte",         bus_read(10), 0x3C);
    // Reading SR should have cleared IFR[SR].
    CHECK_EQ("IFR SR cleared by SR read", bus_read(13) & 0x04, 0x00);
    CHECK_EQ("IFR summary cleared by SR read", bus_read(13) & 0x84, 0x00);
    return true;
}

// ─── Scenario 13: SR shift-out to ADB transceiver ──────────────────────
static bool test_sr_shift_out() {
    reset();

    // ACR[4:2] = 3'b110  → SR-out under T2 rate (we only look at TX pulse).
    bus_write(11, 0x18);          // bits 4,3 = 11
    CHECK_EQ("ACR set shift-out", bus_read(11), 0x18);

    // Write SR — that queues the TX byte.
    bus_write(10, 0xA9);

    // Before phi2_tick, adb_tx_valid is low.
    dut->eval();
    CHECK_TRUE("adb_tx_valid low pre-tick", dut->adb_tx_valid == 0);

    // One phi2_tick fires the TX pulse.
    phi2_ticks(1);

    // After the tick phase ended (pulse was during the tick), check via
    // re-driving phi2=0 and eval.
    // adb_tx_valid is a one-cycle pulse at the rising edge when phi2_tick
    // is high.  Back up one extra tick to see it expire, but grab the
    // byte live.
    dut->eval();
    CHECK_EQ("adb_tx_byte", dut->adb_tx_byte, 0xA9);

    // IFR[2] should also be set (TX complete indication).
    CHECK_EQ("IFR SR set after tx",  bus_read(13) & 0x04, 0x04);
    return true;
}

// ─── Scenario 14: ADB empty bus completes SR receive ───────────────────
static bool test_adb_empty_bus_idle_completion() {
    reset();

    bus_write(14, 0x84);    // enable SR interrupt
    bus_write(11, 0x0C);    // ACR[4:2] = 3'b011, SR in under external clock

    phi2_ticks(15);
    CHECK_EQ("IFR SR clear before idle timeout", bus_read(13) & 0x04, 0x00);
    dut->eval();
    CHECK_TRUE("irq low before idle timeout", dut->irq == 0);

    phi2_ticks(1);
    CHECK_EQ("IFR SR set on empty ADB bus", bus_read(13) & 0x04, 0x04);
    CHECK_EQ("IFR summary set on empty ADB bus", bus_read(13) & 0x84, 0x84);
    dut->eval();
    CHECK_TRUE("irq high for empty ADB bus completion", dut->irq == 1);
    CHECK_EQ("SR idle-high empty bus byte", bus_read(10), 0xFF);
    CHECK_EQ("IFR SR cleared by empty-bus SR read", bus_read(13) & 0x04, 0x00);

    phi2_ticks(32);
    CHECK_EQ("empty-bus completion is one-shot per ACR arm",
             bus_read(13) & 0x04, 0x00);
    return true;
}

static bool test_adb_external_byte_beats_idle_completion() {
    reset();

    bus_write(11, 0x0C);    // arm SR receive and its empty-bus fallback
    phi2_ticks(4);

    dut->adb_rx_byte  = 0x42;
    dut->adb_rx_valid = 1;
    phi2_ticks(1);
    dut->adb_rx_valid = 0;

    CHECK_EQ("IFR SR set by external ADB byte", bus_read(13) & 0x04, 0x04);
    CHECK_EQ("SR holds external byte, not idle", bus_read(10), 0x42);

    phi2_ticks(32);
    CHECK_EQ("idle fallback canceled by external byte", bus_read(13) & 0x04, 0x00);
    return true;
}

static bool test_adb_sr_write_rearms_idle_receive() {
    reset();

    bus_write(11, 0x0C);    // ACR[4:2] = 3'b011, SR in under external clock
    phi2_ticks(16);
    CHECK_EQ("first empty-bus IFR SR", bus_read(13) & 0x04, 0x04);
    CHECK_EQ("first empty-bus SR byte", bus_read(10), 0xFF);
    CHECK_EQ("first empty-bus SR read clears IFR", bus_read(13) & 0x04, 0x00);

    // Keep ACR in receive mode and write SR again, as ROM probes can do
    // between ADB command phases.  The write should start a fresh empty-bus
    // completion instead of requiring ACR to be rewritten.
    bus_write(10, 0x00);
    phi2_ticks(15);
    CHECK_EQ("rearmed empty-bus pending", bus_read(13) & 0x04, 0x00);
    phi2_ticks(1);
    CHECK_EQ("rearmed empty-bus IFR SR", bus_read(13) & 0x04, 0x04);
    CHECK_EQ("rearmed empty-bus SR byte", bus_read(10), 0xFF);
    CHECK_EQ("rearmed empty-bus read clears IFR", bus_read(13) & 0x04, 0x00);
    return true;
}

// ─── Scenario 15: Timer 2 one-shot ─────────────────────────────────────
static bool test_t2_oneshot() {
    reset();

    // T2 low then high — starts the timer.  Load count = 4.
    bus_write(8, 0x04);    // T2CL (latch)
    bus_write(9, 0x00);    // T2CH → counter = 0x0004, arms T2

    CHECK_EQ("IFR T2 pre-tick", bus_read(13) & 0x20, 0x00);

    // 5 phi2 ticks: 4,3,2,1,0, then underflow on next tick = 5th edge.
    phi2_ticks(5);
    CHECK_EQ("IFR T2 after underflow", bus_read(13) & 0x20, 0x20);

    // Clear IFR and verify no further fires (one-shot).
    bus_write(13, 0x20);
    CHECK_EQ("IFR T2 cleared", bus_read(13) & 0x20, 0x00);
    phi2_ticks(100);
    CHECK_EQ("IFR T2 stays cleared", bus_read(13) & 0x20, 0x00);
    return true;
}

// ─── Scenario 16: 60 Hz VBL tick (CA1) ─────────────────────────────────
static bool test_vbl_ca1_tick() {
    reset();

    // Default VBL_PHI2_DIV = 16666.  Pre-tick: no CA1 IFR.
    CHECK_EQ("IFR CA1 pre-tick", bus_read(13) & 0x02, 0x00);

    // Drive 16666 phi2 pulses — one full VBL period.
    phi2_ticks(16666);
    CHECK_EQ("IFR CA1 after VBL period", bus_read(13) & 0x02, 0x02);

    // MAME's 6522 VIA model maps CA1 to the port-A side: reading ORB
    // clears only PB-side handshake IRQs, and ORA-NH returns PA without
    // the handshake clear.  The VBL CA1 flag is acknowledged by ORA.
    (void)bus_read(0);
    CHECK_EQ("IFR CA1 survives ORB read", bus_read(13) & 0x02, 0x02);
    bus_write(0, 0x11);
    CHECK_EQ("IFR CA1 survives ORB write", bus_read(13) & 0x02, 0x02);
    (void)bus_read(15);
    CHECK_EQ("IFR CA1 survives ORA-NH read", bus_read(13) & 0x02, 0x02);
    (void)bus_read(1);
    CHECK_EQ("IFR CA1 cleared by ORA read", bus_read(13) & 0x02, 0x00);

    // Another full period fires it again.
    phi2_ticks(16666);
    CHECK_EQ("IFR CA1 refires next period", bus_read(13) & 0x02, 0x02);

    bus_write(15, 0x55);
    CHECK_EQ("IFR CA1 survives ORA-NH write", bus_read(13) & 0x02, 0x02);
    bus_write(1, 0xAA);
    CHECK_EQ("IFR CA1 cleared by ORA write", bus_read(13) & 0x02, 0x00);
    return true;
}

// ─── Scenario 17: IRQ aggregation ──────────────────────────────────────
static bool test_irq_aggregation() {
    reset();

    // Fire T1 one-shot + T2 one-shot + SR in (via adb rx).
    // 1) Load T1 small: counter=1 so it wraps very quickly.
    bus_write(11, 0x0C);       // ACR[4:2]=011 (shift-in) + T1 one-shot
    bus_write(6, 0x00);
    bus_write(5, 0x00);        // T1 counter = 0x0000 → wraps on first tick
    // 2) T2 small: counter=2
    bus_write(8, 0x02);
    bus_write(9, 0x00);
    // 3) Deliver an ADB byte
    dut->adb_rx_byte  = 0x7E;
    dut->adb_rx_valid = 1;

    phi2_ticks(3);
    dut->adb_rx_valid = 0;

    // All three IFR bits should now be set.
    uint8_t ifr = bus_read(13);
    CHECK_EQ("IFR T1 set",  ifr & 0x40, 0x40);
    CHECK_EQ("IFR T2 set",  ifr & 0x20, 0x20);
    CHECK_EQ("IFR SR set",  ifr & 0x04, 0x04);

    // With IER=0, irq should be low.
    dut->eval();
    CHECK_TRUE("irq low before mask", dut->irq == 0);

    // Enable only SR.
    bus_write(14, 0x84);      // set bit 2
    dut->eval();
    CHECK_TRUE("irq high — SR enabled", dut->irq == 1);

    // Disable SR, enable only T1.
    bus_write(14, 0x04);      // clear bit 2
    bus_write(14, 0xC0);      // set bit 6
    dut->eval();
    CHECK_TRUE("irq high — T1 enabled", dut->irq == 1);

    // Clear IFR[T1], T2 still pending but not enabled.
    bus_write(13, 0x40);
    dut->eval();
    CHECK_TRUE("irq low — only T2 pending, not enabled", dut->irq == 0);

    // Enable T2; fires.
    bus_write(14, 0xA0);      // set bit 5
    dut->eval();
    CHECK_TRUE("irq high — T2 enabled", dut->irq == 1);

    return true;
}

// ─── Scenario 20: external DAFB vblank IRQ edge feeds CA1 ─────────────
static bool test_external_dafb_vblank_irq() {
    reset();

    bus_write(12, 0x01);          // CA1 positive edge for this direct level test
    bus_write(14, 0x82);          // enable CA1/VBL interrupt
    CHECK_TRUE("irq low before DAFB VBL", dut->irq == 0);

    dut->vblank_irq_in = 1;
    tick();
    dut->eval();
    CHECK_EQ("DAFB VBL latches IFR CA1", bus_read(13) & 0x82, 0x82);
    CHECK_TRUE("irq high from DAFB VBL", dut->irq == 1);

    (void)bus_read(1);
    CHECK_EQ("ORA read clears VIA CA1", bus_read(13) & 0x02, 0x00);
    CHECK_TRUE("irq low while DAFB level is still high", dut->irq == 0);
    tick();
    CHECK_EQ("held DAFB level does not re-latch CA1", bus_read(13) & 0x02, 0x00);

    dut->vblank_irq_in = 0;
    tick();
    dut->vblank_irq_in = 1;
    tick();
    CHECK_EQ("fresh DAFB VBL edge re-latches CA1", bus_read(13) & 0x02, 0x02);
    return true;
}

// ─── Scenario 20b: 60 Hz periodic DAFB VBL drives CA1 IRQ (task #145) ─
//
// Drives `vblank_irq_in` as a square-wave-ish train of edges at the
// rate the platform top routes from the HDMI pipeline (after pulse
// extension), and proves that the IFR.CA1 latch fires once per
// rising edge, the IRQ line stays asserted until the CPU acks via
// ORA read, and a held high level does NOT spuriously re-latch the
// IFR (matches the via1.v vblank_irq_in != vblank_irq_prev edge
// detector contract).
static bool test_periodic_dafb_vbl_irq() {
    reset();

    // PCR: CA1 input, low-to-high edge select (Q700 ROM default).
    bus_write(12, 0x01);
    // IER: enable CA1.
    bus_write(14, 0x82);

    const int vbl_count = 8;     // > 2 frames worth — a regression
                                 // window we can observe in unit time.
    const int level_high_clks = 4;
    const int level_low_clks  = 12;

    int seen_rising_edge_irqs = 0;
    for (int i = 0; i < vbl_count; i++) {
        // Rising edge — should latch IFR.CA1 and assert irq.
        dut->vblank_irq_in = 1;
        for (int t = 0; t < level_high_clks; t++) tick();
        if (dut->irq) seen_rising_edge_irqs++;
        // ACK via ORA read; IFR.CA1 clears, irq drops.
        (void)bus_read(1);
        CHECK_TRUE("irq drops after ORA ack", dut->irq == 0);
        // Falling edge: should NOT re-latch CA1 (PCR is low-to-high).
        dut->vblank_irq_in = 0;
        for (int t = 0; t < level_low_clks; t++) tick();
        CHECK_EQ("falling edge does not latch CA1 in low-to-high mode",
                 bus_read(13) & 0x02, 0x00);
    }

    // Each rising edge produced one IRQ.  Allow ±0 because the test
    // is fully deterministic — there's no real CDC here, just the
    // vblank_irq_in/prev edge detector inside via1.v.
    CHECK_EQ("seen rising-edge IRQs matches expected count",
             seen_rising_edge_irqs, vbl_count);
    return true;
}

// ─── Scenario 21: RTC CKO input feeds CA2 ─────────────────────────────
static bool test_rtc_cko_ca2_irq() {
    reset();

    // PCR reset selects CA2 input, high-to-low edge.
    dut->rtc_cko = 0;
    tick();
    CHECK_EQ("RTC CKO falling edge latches CA2", bus_read(13) & 0x01, 0x01);

    // Default CA2 mode is not independent IRQ, so ORA read acknowledges it.
    (void)bus_read(1);
    CHECK_EQ("ORA read clears CA2 in handshake mode", bus_read(13) & 0x01, 0x00);

    // Positive edge is ignored until PCR selects low-to-high.
    dut->rtc_cko = 1;
    tick();
    CHECK_EQ("CA2 positive edge ignored in default mode", bus_read(13) & 0x01, 0x00);

    bus_write(12, 0x04);  // CA2 input, positive edge.
    dut->rtc_cko = 0;
    tick();
    CHECK_EQ("CA2 negative edge ignored in positive mode", bus_read(13) & 0x01, 0x00);
    dut->rtc_cko = 1;
    tick();
    CHECK_EQ("CA2 positive edge latches when PCR selects it", bus_read(13) & 0x01, 0x01);

    // Q700 independent IRQ input mode preserves CA2 across ORA handshakes.
    bus_write(13, 0x01);
    bus_write(12, 0x22);  // CA2 input, positive edge, independent IRQ.
    dut->rtc_cko = 0;
    tick();
    CHECK_EQ("CA2 independent IRQ ignores negative edge", bus_read(13) & 0x01, 0x00);
    dut->rtc_cko = 1;
    tick();
    CHECK_EQ("CA2 independent IRQ latches", bus_read(13) & 0x01, 0x01);
    (void)bus_read(1);
    CHECK_EQ("ORA preserves CA2 in independent mode", bus_read(13) & 0x01, 0x01);
    bus_write(13, 0x01);
    CHECK_EQ("IFR write clears independent CA2", bus_read(13) & 0x01, 0x00);

    return true;
}

static bool test_ca1_held_level_no_retrigger() {
    reset();

    bus_write(12, 0x01);          // CA1 positive edge
    dut->vblank_irq_in = 1;
    tick();
    CHECK_EQ("held CA1 rising edge latches once", bus_read(13) & 0x02, 0x02);

    bus_write(13, 0x02);
    CHECK_EQ("held CA1 clear succeeds", bus_read(13) & 0x02, 0x00);
    for (int i = 0; i < 16; i++) tick();
    CHECK_EQ("held CA1 high does not re-latch", bus_read(13) & 0x02, 0x00);
    return true;
}

static bool test_ca1_one_cycle_pulse_no_retrigger() {
    reset();

    bus_write(12, 0x01);          // CA1 positive edge
    dut->vblank_irq_in = 1;
    tick();
    dut->vblank_irq_in = 0;
    tick();
    CHECK_EQ("CA1 one-cycle pulse latches once", bus_read(13) & 0x02, 0x02);

    bus_write(13, 0x02);
    CHECK_EQ("CA1 pulse clear succeeds", bus_read(13) & 0x02, 0x00);
    for (int i = 0; i < 16; i++) tick();
    CHECK_EQ("CA1 pulse stays clear after idle", bus_read(13) & 0x02, 0x00);
    return true;
}

static bool test_ca2_held_level_no_retrigger_pcr22() {
    reset();

    bus_write(12, 0x22);          // Q700: CA2 independent positive-edge IRQ
    dut->rtc_cko = 0;
    tick();
    CHECK_EQ("CA2 low idle does not latch in PCR 0x22", bus_read(13) & 0x01, 0x00);
    dut->rtc_cko = 1;
    tick();
    CHECK_EQ("held CA2 rising edge latches once", bus_read(13) & 0x01, 0x01);

    bus_write(13, 0x01);
    CHECK_EQ("held CA2 clear succeeds", bus_read(13) & 0x01, 0x00);
    for (int i = 0; i < 16; i++) tick();
    CHECK_EQ("held CA2 high does not re-latch", bus_read(13) & 0x01, 0x00);
    return true;
}

static bool test_ca2_one_cycle_pulse_no_retrigger_pcr22() {
    reset();

    bus_write(12, 0x22);          // Q700: CA2 independent positive-edge IRQ
    dut->rtc_cko = 0;
    tick();
    dut->rtc_cko = 1;
    tick();
    dut->rtc_cko = 0;
    tick();
    CHECK_EQ("CA2 one-cycle pulse latches once", bus_read(13) & 0x01, 0x01);

    bus_write(13, 0x01);
    CHECK_EQ("CA2 pulse clear succeeds", bus_read(13) & 0x01, 0x00);
    for (int i = 0; i < 16; i++) tick();
    CHECK_EQ("CA2 pulse stays clear after idle", bus_read(13) & 0x01, 0x00);
    return true;
}

// ─── Scenario 22: ADB CB1/CB2 idle pulser is GUTTED — no spurious IRQs ─
//
// The previous implementation synthesised idle CB1/CB2 edges every
// phi2_tick (~1.3 µs) so ROM VIA probes would see sticky ADB IFR bits.
// In practice this produced edges faster than any handler could clear
// them, causing an infinite IRQ storm on FPGA hardware.  The pulser was
// removed (rtl/mac/via1.v) until real ADB lands.  This test now asserts
// the new behavior: with no pulser, IFR[CB1]/IFR[CB2] stay clear under
// the same Q700 PCR setup.
static bool test_adb_modem_idle_cb2_irq() {
    reset();

    // Q700 PCR setup: CB2 input, high-to-low, independent.
    bus_write(12, 0x22);
    phi2_ticks(128);   // way more than the old idle period (64 ticks)
    CHECK_EQ("ADB idle pulser GUTTED: no CB2 IRQ", bus_read(13) & 0x08, 0x00);
    CHECK_EQ("ADB idle pulser GUTTED: no CB1 IRQ", bus_read(13) & 0x10, 0x00);

    bus_write(12, 0x00);  // handshake CB2 mode
    phi2_ticks(128);
    CHECK_EQ("ADB idle pulser GUTTED: no CB2 IRQ in handshake mode",
             bus_read(13) & 0x08, 0x00);

    return true;
}

// ─── Main ──────────────────────────────────────────────────────────────
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vvia1;

    RUN(test_reset_reads);
    RUN(test_reg_roundtrip);
    RUN(test_overlay_bit);
    RUN(test_orb_pb3_adb_irq_input);
    RUN(test_port_b_mux);
    RUN(test_rtc_sidechannel_visibility);
    RUN(test_orb_high_bits_follow_ddr_mux);
    RUN(test_orb_rtc_bits_ddr_mux);
    RUN(test_t1_oneshot);
    RUN(test_t1_continuous);
    RUN(test_ier_masks_irq);
    RUN(test_ifr_write_clear);
    RUN(test_ifr_write_ff_clears_all_and_deasserts_irq);
    RUN(test_timer_counter_access_clears_ifr);
    RUN(test_port_handshake_clears);
    RUN(test_t2_pulsecount);
    RUN(test_t1_write_vs_phi2_tick_race);
    RUN(test_t2_write_vs_phi2_tick_race);
    RUN(test_t1_ifr_oneshot_underflow);
    RUN(test_t1_ifr_write_one_clear);
    RUN(test_t1_ifr_read_t1cl_clears);
    RUN(test_sr_shift_in);
    RUN(test_sr_shift_out);
    RUN(test_adb_empty_bus_idle_completion);
    RUN(test_adb_external_byte_beats_idle_completion);
    RUN(test_adb_sr_write_rearms_idle_receive);
    RUN(test_t2_oneshot);
    RUN(test_vbl_ca1_tick);
    RUN(test_irq_aggregation);
    RUN(test_external_dafb_vblank_irq);
    RUN(test_periodic_dafb_vbl_irq);
    RUN(test_rtc_cko_ca2_irq);
    RUN(test_ca1_held_level_no_retrigger);
    RUN(test_ca1_one_cycle_pulse_no_retrigger);
    RUN(test_ca2_held_level_no_retrigger_pcr22);
    RUN(test_ca2_one_cycle_pulse_no_retrigger_pcr22);
    RUN(test_adb_modem_idle_cb2_irq);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
