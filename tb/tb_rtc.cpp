// tb_rtc.cpp — Verilator unit testbench for rtc.v
//
// Build:   make tb-rtc
// Scenarios:
//   1. Reset defaults are deterministic: seconds=0, idle data=0.
//   2. Read-seconds via the 4-byte command sequence (0x81, 0x85, 0x89, 0x8D)
//      matches MAME/macrtc byte order: low byte first.
//   3. PRAM byte addressing round-trips and only the targeted byte changes.
//   4. Read/write sequencing is transaction-scoped and clears idle data low.
//   5. Control bits are deterministic: test-register and write-protect
//      readback work, the test bit does not reset the clock, and
//      protected writes are ignored.
//   6. seconds_advance_over_time — +rtc_fast plusarg collapses SEC_DIV by
//      SEC_DIV_FAST_RATIO (default 1000×); tick N*(SEC_DIV/RATIO) phi2
//      pulses and confirm the seconds register reports N.
//   7. MAME seconds byte order is visible directly at the four commands.
//   8. Write-seconds commands update the same low-byte-first register order.
//   9. Command/data input is sampled on rtcClk falling edges, matching MAME.
//  10. 343-0042 register aliases and extended PRAM commands are decoded.
//  11. Seconds reads stay latched across intervening phi2 ticks, matching
//      ROM-style polling.
//  12. +rtc_trace can be enabled without changing bus-visible behaviour.
//  13. ROM-style pull-up one bits are accepted for command/address/write data.
//  14. +rtc_mame_state resets PRAM to MAME's clean-NVRAM all-zero state,
//      and +rtc_init_seconds seeds the seconds counter for lockstep.
//  15. CKO resets high, toggles at half-second cadence, and seconds advance
//      on its rising edge.
//
// The rtc module expects SEC_DIV phi2 pulses per second; for this test
// we rely on the parameter default (1_000_000) but only care about the
// logical shift protocol — we force the seconds value indirectly by
// advancing phi2 ticks before reading.

#include <cstdio>
#include <cstdint>
#include <initializer_list>
#include <vector>
#include <verilated.h>
#include "Vrtc.h"

static Vrtc*     dut      = nullptr;
static uint64_t  sim_time = 0;
static int       n_pass   = 0;
static int       n_fail   = 0;

static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void wait_pram_clear() {
    for (unsigned i = 0; i < 260 && dut->pram_busy; ++i) tick();
    if (dut->pram_busy) VL_FATAL_MT(__FILE__, __LINE__, "", "PRAM clear did not finish");
}

// Warm reset — pulses `rst` ONLY.
//
// As of the battery-backed-PRAM change, `rst` deliberately does not clear
// the PRAM array (it models the lithium-cell-backed parameter RAM of a
// real Macintosh RTC IC).  Everything else in the module — shift FSM,
// seconds counter, write_protect, test_mode — still rewinds.
static void warm_reset() {
    dut->rst         = 1;
    dut->pram_clear  = 0;
    dut->phi2_tick   = 0;
    dut->rtc_enb     = 1;   // idle high
    dut->rtc_clk     = 0;
    dut->rtc_data_o  = 0;
    dut->rtc_data_oe = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
}

// Pulse the explicit PRAM zap (Cmd-Opt-P-R) without touching `rst`.
// Rewrites all 256 bytes from the module's power-on image.
static void pram_zap() {
    dut->pram_clear = 1;
    tick(); tick();
    dut->pram_clear = 0;
    tick();
    wait_pram_clear();
}

// Full test-isolation reset — warm reset PLUS an explicit PRAM zap.
//
// This is what the per-scenario `reset()` used to mean implicitly, back
// when `rst` wiped PRAM.  Now that PRAM persists, scenarios that want a
// pristine array must say so; driving the new `pram_clear` port here
// keeps every pre-existing scenario isolated *and* exercises the new
// port on every single test.  Scenarios that specifically test
// persistence across reset must call warm_reset() instead.
static void reset() {
    dut->rst         = 1;
    dut->pram_clear  = 1;
    dut->phi2_tick   = 0;
    dut->rtc_enb     = 1;   // idle high
    dut->rtc_clk     = 0;
    dut->rtc_data_o  = 0;
    dut->rtc_data_oe = 0;
    tick(); tick();
    dut->rst        = 0;
    dut->pram_clear = 0;
    tick();
    wait_pram_clear();
}

#define CHECK_EQ(name, got, exp) do { \
    if ((uint32_t)(got) != (uint32_t)(exp)) { \
        printf("  FAIL %s: got 0x%02x, expected 0x%02x\n", \
               name, (uint32_t)(got), (uint32_t)(exp)); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { printf("  FAIL %s\n", name); return false; } \
} while(0)

// Spin up a fresh Vrtc with a specific set of plusargs in force, and make
// it the active `dut` for the lifetime of the object.  rtc.v evaluates its
// $test$plusargs-gated initial block on the new instance's first eval(),
// so the plusargs must be installed BEFORE construction.  RAII so an early
// `return false` out of CHECK_EQ still restores the shared dut instead of
// leaking it and corrupting every later scenario.
struct ScopedDut {
    Vrtc* fresh;
    Vrtc* saved;
    explicit ScopedDut(std::initializer_list<const char*> plusargs) {
        std::vector<const char*> argv;
        argv.push_back("tb_rtc");
        for (const char* a : plusargs) argv.push_back(a);
        argv.push_back(nullptr);
        Verilated::commandArgs((int)argv.size() - 1,
                               const_cast<char**>(argv.data()));
        fresh = new Vrtc;
        saved = dut;
        dut   = fresh;
    }
    ~ScopedDut() { fresh->final(); delete fresh; dut = saved; }
    ScopedDut(const ScopedDut&)            = delete;
    ScopedDut& operator=(const ScopedDut&) = delete;
};

// Advance n phi2 pulses (one per clock).
static void phi2_ticks(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        dut->phi2_tick = 1;
        tick();
        dut->phi2_tick = 0;
    }
}

// Shift one bit into the rtc (CPU -> RTC): MAME/macrtc samples host data
// on rtcClk high-to-low, so hold data stable before the falling edge.
static void shift_bit_in(int b) {
    dut->rtc_data_o  = b & 1;
    dut->rtc_data_oe = 1;
    dut->rtc_clk     = 1; tick();
    dut->rtc_clk     = 0; tick();
}

static void shift_bit_in_pullup_one(int b) {
    dut->rtc_data_o  = 0;
    dut->rtc_data_oe = b ? 0 : 1;
    dut->rtc_clk     = 1; tick();
    dut->rtc_clk     = 0; tick();
}

// Shift one bit out (RTC -> CPU): read data becomes valid after the falling
// edge, matching MAME/macrtc and the 343-0042 timing.
static int shift_bit_out() {
    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 1; tick();
    dut->rtc_clk     = 0; tick();  // falling edge updates rtc_data_i
    dut->eval();
    return dut->rtc_data_i & 1;
}

// Full transaction: assert rtcEnb, shift cmd, then 8 data bits, deassert.
// cmd bit 7 = R/W; if write, we push data_byte; if read we shift out and
// return the sampled byte.
static uint8_t rtc_transaction(uint8_t cmd, uint8_t data_byte) {
    uint8_t out = 0;

    // Start: rtcEnb low
    dut->rtc_enb = 0; tick();

    // Shift 8 bits of command MSB-first
    for (int i = 7; i >= 0; i--)
        shift_bit_in((cmd >> i) & 1);

    // Shift 8 bits of data
    if (cmd & 0x80) {
        // Read
        for (int i = 7; i >= 0; i--) {
            int bit = shift_bit_out();
            out = (uint8_t)((out << 1) | bit);
        }
    } else {
        for (int i = 7; i >= 0; i--)
            shift_bit_in((data_byte >> i) & 1);
    }

    // Deassert: drop CPU drive, rtcEnb high
    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 0; tick();
    dut->rtc_enb     = 1; tick();

    return out;
}

static uint8_t rtc_xpram_transaction(uint8_t cmd, uint8_t addr_byte, uint8_t data_byte) {
    uint8_t out = 0;

    dut->rtc_enb = 0; tick();

    for (int i = 7; i >= 0; i--)
        shift_bit_in((cmd >> i) & 1);
    for (int i = 7; i >= 0; i--)
        shift_bit_in((addr_byte >> i) & 1);

    if (cmd & 0x80) {
        for (int i = 7; i >= 0; i--) {
            int bit = shift_bit_out();
            out = (uint8_t)((out << 1) | bit);
        }
    } else {
        for (int i = 7; i >= 0; i--)
            shift_bit_in((data_byte >> i) & 1);
    }

    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 0; tick();
    dut->rtc_enb     = 1; tick();

    return out;
}

static uint8_t rtc_transaction_pullup_ones(uint8_t cmd, uint8_t data_byte) {
    uint8_t out = 0;

    dut->rtc_enb = 0; tick();

    for (int i = 7; i >= 0; i--)
        shift_bit_in_pullup_one((cmd >> i) & 1);

    if (cmd & 0x80) {
        for (int i = 7; i >= 0; i--) {
            int bit = shift_bit_out();
            out = (uint8_t)((out << 1) | bit);
        }
    } else {
        for (int i = 7; i >= 0; i--)
            shift_bit_in_pullup_one((data_byte >> i) & 1);
    }

    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 0; tick();
    dut->rtc_enb     = 1; tick();

    return out;
}

static uint8_t rtc_xpram_transaction_pullup_ones(uint8_t cmd,
                                                 uint8_t addr_byte,
                                                 uint8_t data_byte) {
    uint8_t out = 0;

    dut->rtc_enb = 0; tick();

    for (int i = 7; i >= 0; i--)
        shift_bit_in_pullup_one((cmd >> i) & 1);
    for (int i = 7; i >= 0; i--)
        shift_bit_in_pullup_one((addr_byte >> i) & 1);

    if (cmd & 0x80) {
        for (int i = 7; i >= 0; i--) {
            int bit = shift_bit_out();
            out = (uint8_t)((out << 1) | bit);
        }
    } else {
        for (int i = 7; i >= 0; i--)
            shift_bit_in_pullup_one((data_byte >> i) & 1);
    }

    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 0; tick();
    dut->rtc_enb     = 1; tick();

    return out;
}

static uint8_t xpram_cmd_for_addr(uint8_t addr, bool read) {
    return (uint8_t)((read ? 0x80 : 0x00) | 0x38 | ((addr >> 5) & 0x07));
}

static uint8_t xpram_addr_byte(uint8_t addr) {
    return (uint8_t)(((addr & 0x1f) << 2) | 0x01);
}

static uint8_t xpram_read(uint8_t addr) {
    return rtc_xpram_transaction(xpram_cmd_for_addr(addr, true),
                                 xpram_addr_byte(addr), 0);
}

static void xpram_write(uint8_t addr, uint8_t value) {
    (void)rtc_xpram_transaction(xpram_cmd_for_addr(addr, false),
                                xpram_addr_byte(addr), value);
}

static void shift_bit_in_falling_sampled(int bit) {
    dut->rtc_data_oe = 1;
    dut->rtc_data_o  = bit ? 0 : 1;
    dut->rtc_clk     = 1; tick();
    dut->rtc_data_o  = bit & 1;
    tick();
    dut->rtc_clk     = 0; tick();
}

static void rtc_write_transaction_falling_sampled(uint8_t cmd, uint8_t data_byte) {
    dut->rtc_enb = 0; tick();
    for (int i = 7; i >= 0; i--)
        shift_bit_in_falling_sampled((cmd >> i) & 1);
    for (int i = 7; i >= 0; i--)
        shift_bit_in_falling_sampled((data_byte >> i) & 1);
    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 0; tick();
    dut->rtc_enb     = 1; tick();
}

static uint32_t read_seconds32() {
    uint8_t b0 = rtc_transaction(0x81, 0);
    uint8_t b1 = rtc_transaction(0x85, 0);
    uint8_t b2 = rtc_transaction(0x89, 0);
    uint8_t b3 = rtc_transaction(0x8D, 0);
    return (uint32_t(b3) << 24) | (uint32_t(b2) << 16)
         | (uint32_t(b1) <<  8) |  uint32_t(b0);
}

static uint8_t rtc_read_byte_with_delay(uint8_t cmd, uint32_t phi2_delay) {
    uint8_t out = 0;

    dut->rtc_enb = 0; tick();
    for (int i = 7; i >= 0; i--)
        shift_bit_in((cmd >> i) & 1);

    dut->rtc_data_oe = 0;
    phi2_ticks(phi2_delay);

    for (int i = 7; i >= 0; i--) {
        int bit = shift_bit_out();
        out = (uint8_t)((out << 1) | bit);
    }

    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 0; tick();
    dut->rtc_enb     = 1; tick();
    return out;
}

static void rtc_partial_write_transaction(uint8_t cmd, uint8_t data_byte, int data_bits) {
    dut->rtc_enb = 0; tick();
    for (int i = 7; i >= 0; i--)
        shift_bit_in((cmd >> i) & 1);
    for (int i = 7; i >= 8 - data_bits; i--)
        shift_bit_in((data_byte >> i) & 1);
    dut->rtc_data_oe = 0;
    dut->rtc_clk     = 0; tick();
    dut->rtc_enb     = 1; tick();
}

// ─── Scenario 1: reset defaults + power-on PRAM image ──────────────────
//
// Two different contracts are checked here, and the label wording keeps
// them apart on purpose:
//   * rtc_data_i / cko / seconds are RESET defaults — `rst` rewinds them.
//   * the pram[]/xpram[] bytes are POWER-ON defaults — they come from the
//     configuration-time `initial` in rtc.v, NOT from `rst`.  Since the
//     battery-backed-PRAM change, `rst` never touches the array; calling
//     a check here "pram[8] reset" would be actively misleading.
//
// Opts into the populated SCBI-magic image via +rtc_populated_pram (the
// sim default is MAME's all-zero NVRAM), so the assertions below are a
// real test of the image rather than "everything is zero anyway".
static bool test_reset_defaults() {
    ScopedDut sd{"+rtc_populated_pram"};
    reset();
    CHECK_EQ("idle data low", dut->rtc_data_i, 0);
    CHECK_EQ("CKO reset high", dut->cko, 1);
    CHECK_EQ("seconds reset", read_seconds32(), 0);
    CHECK_EQ("pram[8] power-on default", rtc_transaction(0xA0, 0), 0x13);
    CHECK_EQ("pram[9] power-on default", rtc_transaction(0xA4, 0), 0x88);
    CHECK_EQ("xpram[2] power-on default", xpram_read(0x02), 0x4F);
    CHECK_EQ("xpram[3] power-on default", xpram_read(0x03), 0x48);
    CHECK_EQ("xpram[0x10] power-on default", xpram_read(0x10), 0xA8);
    CHECK_EQ("xpram[0x47] power-on default", xpram_read(0x47), 0x33);
    CHECK_EQ("xpram[0x77] power-on default", xpram_read(0x77), 0x01);
    CHECK_EQ("xpram[0x78] power-on default", xpram_read(0x78), 0xFF);
    CHECK_EQ("xpram[0x7b] power-on default", xpram_read(0x7B), 0xDF);
    CHECK_EQ("xpram[0xf8] power-on default", xpram_read(0xF8), 0x53);
    CHECK_EQ("xpram[0xf9] power-on default", xpram_read(0xF9), 0x43);
    CHECK_EQ("xpram[0xfa] power-on default", xpram_read(0xFA), 0x42);
    CHECK_EQ("xpram[0xfb] power-on default", xpram_read(0xFB), 0x49);
    return true;
}

// ─── Scenario 2: read-seconds matches counter ───────────────────────────
static bool test_read_seconds() {
    reset();
    // Advance 3 seconds worth of phi2 ticks.
    phi2_ticks(3000000);
    CHECK_EQ("seconds readback", read_seconds32(), 3);
    return true;
}

// ─── Scenario 3: PRAM byte addressing ──────────────────────────────────
static bool test_pram_byte_addressing() {
    ScopedDut sd{"+rtc_populated_pram"};
    reset();
    rtc_transaction(0x20, 0xA5);   // write reg=8
    rtc_transaction(0x30, 0x5A);   // write reg=12
    rtc_transaction(0x4C, 0xC3);   // write reg=19

    CHECK_EQ("pram[8]",  rtc_transaction(0xA0, 0), 0xA5);
    CHECK_EQ("test register is not PRAM[12]", rtc_transaction(0xB0, 0), 0x00);
    CHECK_EQ("pram[19]", rtc_transaction(0xCC, 0), 0xC3);
    CHECK_EQ("pram[9] keeps power-on default", rtc_transaction(0xA4, 0), 0x88);

    rtc_transaction(0x50, 0x6D);   // write reg=20, a 343-0042 PRAM window
    CHECK_EQ("pram[20]", rtc_transaction(0xD0, 0), 0x6D);
    return true;
}

// ─── Scenario 4: sequencing / idle behaviour ───────────────────────────
static bool test_sequence_and_idle() {
    reset();
    rtc_transaction(0x20, 0x5A);
    CHECK_EQ("idle data low after write", dut->rtc_data_i, 0);
    CHECK_EQ("pram[8] committed at end", rtc_transaction(0xA0, 0), 0x5A);

    // A read transaction leaves the data_out latch cleared low, matching MAME.
    (void)rtc_transaction(0xA0, 0);
    CHECK_EQ("idle data low after read", dut->rtc_data_i, 0);
    return true;
}

// ─── Scenario 4b: incomplete writes are ignored ────────────────────────
static bool test_incomplete_write_is_ignored() {
    ScopedDut sd{"+rtc_populated_pram"};
    reset();
    rtc_partial_write_transaction(0x20, 0x5A, 4);
    CHECK_EQ("partial PRAM write ignored", rtc_transaction(0xA0, 0), 0x13);
    rtc_partial_write_transaction(0x38, 0xA5, 3);
    CHECK_EQ("partial extended write ignored", xpram_read(0x42), 0x00);
    return true;
}

// ─── Scenario 5: control bits / write-protect latch ────────────────────
static bool test_control_bits_and_write_protect() {
    ScopedDut sd{"+rtc_populated_pram"};
    reset();

    CHECK_EQ("test register reset", rtc_transaction(0xB1, 0), 0x00);
    CHECK_EQ("write-protect reset", rtc_transaction(0xB5, 0), 0x00);

    phi2_ticks(1000000);
    CHECK_EQ("seconds before test register", read_seconds32(), 1);

    rtc_transaction(0x31, 0x80);
    CHECK_EQ("test register set", rtc_transaction(0xB1, 0), 0x80);
    CHECK_EQ("test register leaves seconds intact", read_seconds32(), 1);

    phi2_ticks(1000000);
    CHECK_EQ("test register leaves clock running", read_seconds32(), 2);

    rtc_transaction(0x31, 0x00);
    CHECK_EQ("test register clear", rtc_transaction(0xB1, 0), 0x00);

    rtc_transaction(0x34, 0x80);
    CHECK_EQ("write-protect set", rtc_transaction(0xB5, 0), 0x80);

    rtc_transaction(0x31, 0x80);
    CHECK_EQ("test register blocked while write-protected", rtc_transaction(0xB1, 0), 0x00);

    uint32_t seconds_before_write_protect = read_seconds32();
    rtc_transaction(0x20, 0xA5);
    rtc_transaction(0x01, 0x12);
    rtc_transaction(0x05, 0x34);
    rtc_transaction(0x09, 0x56);
    rtc_transaction(0x0D, 0x78);
    CHECK_EQ("PRAM blocked while write-protected", rtc_transaction(0xA0, 0), 0x13);
    CHECK_EQ("seconds blocked while write-protected", read_seconds32(), seconds_before_write_protect);

    rtc_transaction(0x37, 0x00);
    CHECK_EQ("write-protect cleared", rtc_transaction(0xB5, 0), 0x00);

    rtc_transaction(0x20, 0xA5);
    CHECK_EQ("PRAM writable after clear", rtc_transaction(0xA0, 0), 0xA5);
    return true;
}

// ─── Scenario 6: seconds_advance_over_time — uses +rtc_fast ─────────────
//
// With +rtc_fast on the Verilator command line, SEC_DIV is divided by
// SEC_DIV_FAST_RATIO (default 1000×), so one "second" becomes 1000 phi2
// pulses.  We tick N*1000 phi2 pulses and verify the seconds register
// has incremented by N.  The check reads the 32-bit seconds value via
// the standard 4-byte transaction.
//
// The tb instantiates a SECOND Vrtc dut here with +rtc_fast set before
// construction, so the module's $test$plusargs-gated initial block
// selects fast mode for this scenario without disturbing the 1 MHz-real
// mode used by scenarios 1–3.
static bool test_seconds_advance_over_time() {
    // Swap to a fresh DUT with the +rtc_fast plusarg in force.  The
    // plusarg is evaluated at the initial-block, which runs on the first
    // eval() of the new instance.
    const char* fast_argv[] = { "tb_rtc", "+rtc_fast", nullptr };
    Verilated::commandArgs(2, const_cast<char**>(fast_argv));
    Vrtc* fresh = new Vrtc;
    Vrtc* saved = dut;
    dut = fresh;

    reset();

    // Initial snapshot (reset guarantees 0, and the shift-sequence costs
    // only clk edges, no phi2 ticks).
    uint32_t s0 = read_seconds32();
    if (s0 != 0) {
        printf("  FAIL initial seconds = 0: got %u\n", s0);
        fresh->final(); delete fresh; dut = saved; return false;
    }

    // With +rtc_fast, SEC_DIV_EFF = 1_000_000 / 1000 = 1000 phi2 ticks.
    const uint32_t TICKS_PER_SEC_FAST = 1000;
    const uint32_t N = 5;

    // Tick N "seconds" worth.
    phi2_ticks(N * TICKS_PER_SEC_FAST);
    uint32_t sN = read_seconds32();
    if (sN != N) {
        printf("  FAIL seconds_advance_over_time: got %u, expected %u\n",
               sN, N);
        fresh->final(); delete fresh; dut = saved; return false;
    }

    // Second pass — another N seconds worth, cumulative.
    phi2_ticks(N * TICKS_PER_SEC_FAST);
    uint32_t s2N = read_seconds32();
    uint32_t delta = s2N - sN;
    if (delta != N) {
        printf("  FAIL seconds_advance_over_time(2): delta=%u, expected %u\n",
               delta, N);
        fresh->final(); delete fresh; dut = saved; return false;
    }

    fresh->final();
    delete fresh;
    dut = saved;
    return true;
}

// ─── Scenario 7: MAME seconds byte order ────────────────────────────────
static bool test_mame_seconds_byte_order() {
    const char* fast_argv[] = { "tb_rtc", "+rtc_fast", nullptr };
    Verilated::commandArgs(2, const_cast<char**>(fast_argv));
    Vrtc* fresh = new Vrtc;
    Vrtc* saved = dut;
    dut = fresh;

    reset();

    const uint32_t TICKS_PER_SEC_FAST = 1000;
    phi2_ticks(258 * TICKS_PER_SEC_FAST);

    uint8_t b0 = rtc_transaction(0x81, 0);
    uint8_t b1 = rtc_transaction(0x85, 0);
    uint8_t b2 = rtc_transaction(0x89, 0);
    uint8_t b3 = rtc_transaction(0x8D, 0);
    bool ok = true;

    if (b0 != 0x02 || b1 != 0x01 || b2 != 0x00 || b3 != 0x00) {
        printf("  FAIL MAME seconds order: got %02x %02x %02x %02x\n",
               b0, b1, b2, b3);
        ok = false;
    } else {
        printf("  rtc firstlight summary: seconds=258 bytes[81,85,89,8d]=%02x,%02x,%02x,%02x\n",
               b0, b1, b2, b3);
    }

    fresh->final();
    delete fresh;
    dut = saved;
    return ok;
}

// ─── Scenario 8: write-seconds byte order ───────────────────────────────
static bool test_write_seconds_byte_order() {
    reset();

    rtc_transaction(0x01, 0x04);
    rtc_transaction(0x05, 0x03);
    rtc_transaction(0x09, 0x02);
    rtc_transaction(0x0D, 0x01);

    CHECK_EQ("seconds write/readback", read_seconds32(), 0x01020304);
    CHECK_EQ("seconds cmd 0x81 after write", rtc_transaction(0x81, 0), 0x04);
    CHECK_EQ("seconds cmd 0x85 after write", rtc_transaction(0x85, 0), 0x03);
    CHECK_EQ("seconds cmd 0x89 after write", rtc_transaction(0x89, 0), 0x02);
    CHECK_EQ("seconds cmd 0x8D after write", rtc_transaction(0x8D, 0), 0x01);

    rtc_transaction(0x11, 0x08);
    rtc_transaction(0x15, 0x07);
    rtc_transaction(0x19, 0x06);
    rtc_transaction(0x1D, 0x05);

    CHECK_EQ("seconds write aliases", read_seconds32(), 0x05060708);
    CHECK_EQ("seconds read alias 0x91", rtc_transaction(0x91, 0), 0x08);
    CHECK_EQ("seconds read alias 0x95", rtc_transaction(0x95, 0), 0x07);
    CHECK_EQ("seconds read alias 0x99", rtc_transaction(0x99, 0), 0x06);
    CHECK_EQ("seconds read alias 0x9D", rtc_transaction(0x9D, 0), 0x05);
    return true;
}

// ─── Scenario 9: falling-edge sampling ──────────────────────────────────
static bool test_mame_falling_edge_sampling() {
    reset();
    rtc_write_transaction_falling_sampled(0x20, 0x5A);
    CHECK_EQ("falling-edge sampled PRAM write", rtc_transaction(0xA0, 0), 0x5A);
    return true;
}

// ─── Scenario 10: extended PRAM and register decode aliases ─────────────
static bool test_extended_pram_commands() {
    reset();

    CHECK_EQ("xpram power-on default", xpram_read(0x42), 0x00);
    xpram_write(0x42, 0xA6);
    CHECK_EQ("xpram write/readback", xpram_read(0x42), 0xA6);

    xpram_write(0x1F, 0x5B);
    CHECK_EQ("xpram low sector", xpram_read(0x1F), 0x5B);

    xpram_write(0xE3, 0xC7);
    CHECK_EQ("xpram high sector", xpram_read(0xE3), 0xC7);

    // MAME macrtc.cpp:268-273 (RTC_STATE_XPWRITE branch) commits the
    // byte unconditionally — extended-PRAM writes are NOT gated by the
    // write-protect bit.  The WP check at macrtc.cpp:280 lives inside
    // the RTC_STATE_WRITE branch only.  Verify our RTL matches.
    rtc_transaction(0x36, 0x80);
    xpram_write(0x42, 0x19);
    CHECK_EQ("xpram passes through while write-protected (MAME-faithful)",
             xpram_read(0x42), 0x19);

    // Clear WP and overwrite to confirm the path is exercised.
    rtc_transaction(0x34, 0x00);
    xpram_write(0x42, 0x6E);
    CHECK_EQ("xpram still writable after clear", xpram_read(0x42), 0x6E);
    return true;
}

// ─── Scenario 11: ROM-style polling latch ──────────────────────────────
static bool test_seconds_read_latch_during_polling() {
    const char* fast_argv[] = { "tb_rtc", "+rtc_fast", nullptr };
    Verilated::commandArgs(2, const_cast<char**>(fast_argv));
    Vrtc* fresh = new Vrtc;
    Vrtc* saved = dut;
    dut = fresh;

    reset();

    rtc_transaction(0x01, 0x01);
    CHECK_EQ("seed seconds for polling", read_seconds32(), 1);

    const uint8_t latched = rtc_read_byte_with_delay(0x81, 1000);
    CHECK_EQ("seconds byte latched across phi2 ticks", latched, 0x01);
    CHECK_EQ("seconds advanced while polling", read_seconds32(), 2);

    fresh->final();
    delete fresh;
    dut = saved;
    return true;
}

// ─── Scenario 12: trace plusarg smoke ───────────────────────────────────
static bool test_trace_mode_smoke() {
    ScopedDut sd{"+rtc_trace", "+rtc_populated_pram"};
    reset();
    CHECK_EQ("trace mode read power-on default", rtc_transaction(0xA0, 0), 0x13);
    rtc_transaction(0x20, 0x22);
    CHECK_EQ("trace mode write/readback", rtc_transaction(0xA0, 0), 0x22);
    return true;
}

// ─── Scenario 13: ROM-style pull-up one bits ───────────────────────────────
static bool test_rom_style_pullup_one_bits() {
    reset();

    rtc_transaction_pullup_ones(0x40, 0xA5);
    CHECK_EQ("pull-up one PRAM command/write",
             rtc_transaction_pullup_ones(0xC0, 0), 0xA5);

    const uint8_t addr = 0x42;
    rtc_xpram_transaction_pullup_ones(xpram_cmd_for_addr(addr, false),
                                      xpram_addr_byte(addr), 0x96);
    CHECK_EQ("pull-up one XPRAM command/write",
             rtc_xpram_transaction_pullup_ones(xpram_cmd_for_addr(addr, true),
                                               xpram_addr_byte(addr), 0),
             0x96);
    return true;
}

// ─── Scenario 14: MAME-compatible reset state ──────────────────────────
static bool test_mame_state_reset_defaults() {
    const char* mame_argv[] = {
        "tb_rtc", "+rtc_mame_state", "+rtc_init_seconds=3860006400", nullptr
    };
    Verilated::commandArgs(3, const_cast<char**>(mame_argv));
    Vrtc* fresh = new Vrtc;
    Vrtc* saved = dut;
    dut = fresh;

    reset();
    CHECK_EQ("MAME state classic PRAM byte", rtc_transaction(0xA0, 0), 0x00);
    CHECK_EQ("MAME state xpram 0x47", xpram_read(0x47), 0x00);
    CHECK_EQ("MAME state xpram 0x78", xpram_read(0x78), 0x00);
    CHECK_EQ("MAME state xpram 0xf9", xpram_read(0xF9), 0x00);
    CHECK_EQ("MAME state seconds", read_seconds32(), 0xE6130600);

    fresh->final();
    delete fresh;
    dut = saved;
    return true;
}

// ─── Scenario 16: Mac-style READ SECONDS round-trip + PRAM round-trip ─
// Mac OS reads the date by issuing the four READ_SECONDS commands
// (0x81/0x85/0x89/0x8D) and assembling a 32-bit Apple-epoch counter.
// This test verifies (a) the write/readback path round-trips a known
// timestamp, (b) the seconds counter advances naturally over phi2 ticks,
// (c) PRAM bytes round-trip through the small-PRAM register window
// (cmd 0x21..0x2D = write, 0xA1..0xAD = read), and (d) the MAME-faithful
// xpram WP behaviour holds (writes pass through regardless of WP).
static bool test_mac_style_date_and_pram_roundtrip() {
    const char* fast_argv[] = { "tb_rtc", "+rtc_fast", nullptr };
    Verilated::commandArgs(2, const_cast<char**>(fast_argv));
    Vrtc* fresh = new Vrtc;
    Vrtc* saved = dut;
    dut = fresh;

    reset();

    // Mac OS Apple-epoch seed: 2024-01-01 00:00:00 GMT ≈ 0xE2380180.
    // Drive the RTC's seconds register via the four WRITE_SECONDS
    // commands and confirm the corresponding READ_SECONDS sequence
    // returns the same long.
    const uint32_t apple_epoch_2024 = 0xE2380180u;
    rtc_transaction(0x01, (uint8_t)(apple_epoch_2024 >>  0));
    rtc_transaction(0x05, (uint8_t)(apple_epoch_2024 >>  8));
    rtc_transaction(0x09, (uint8_t)(apple_epoch_2024 >> 16));
    rtc_transaction(0x0D, (uint8_t)(apple_epoch_2024 >> 24));
    CHECK_EQ("Mac-style WRITE_SECONDS round-trip via READ_SECONDS",
             read_seconds32(), apple_epoch_2024);

    // Tick three "seconds" worth of phi2 pulses (+rtc_fast collapses
    // SEC_DIV by 1000×).  Mac OS expects the counter to advance.
    phi2_ticks(3 * 1000);
    CHECK_EQ("Mac-style date advances naturally",
             read_seconds32(), apple_epoch_2024 + 3);

    // PRAM round-trip on small-PRAM window (registers 16..31 → cmd
    // 0x40..0x7C / 0xC0..0xFC).
    rtc_transaction(0x40, 0xDE);   // write reg=16
    rtc_transaction(0x44, 0xAD);   // write reg=17
    CHECK_EQ("PRAM small-window roundtrip [16]",
             rtc_transaction(0xC0, 0), 0xDE);
    CHECK_EQ("PRAM small-window roundtrip [17]",
             rtc_transaction(0xC4, 0), 0xAD);

    // Extended PRAM round-trip — the OS uses this for AppleTalk node
    // address, network preferences, etc.
    xpram_write(0x55, 0xC9);
    xpram_write(0x99, 0x73);
    CHECK_EQ("xpram round-trip @0x55", xpram_read(0x55), 0xC9);
    CHECK_EQ("xpram round-trip @0x99", xpram_read(0x99), 0x73);

    // MAME-faithful: setting WP does NOT block xpram writes (only the
    // small-PRAM and seconds writes are gated).  Verify the xpram path
    // still passes through.
    rtc_transaction(0x34, 0x80);
    CHECK_EQ("write-protect set", rtc_transaction(0xB5, 0), 0x80);
    xpram_write(0x55, 0xA5);
    CHECK_EQ("xpram still writes under WP (MAME macrtc.cpp:268-273)",
             xpram_read(0x55), 0xA5);

    // small-PRAM and seconds writes ARE gated by WP (MAME line 280).
    rtc_transaction(0x40, 0xFF);   // attempt write reg=16
    CHECK_EQ("small-PRAM blocked by WP (MAME macrtc.cpp:280)",
             rtc_transaction(0xC0, 0), 0xDE);
    rtc_transaction(0x01, 0x00);   // attempt seconds[7:0]
    CHECK_EQ("seconds blocked by WP (MAME macrtc.cpp:280)",
             read_seconds32() & 0xFF,
             (apple_epoch_2024 + 3) & 0xFF);

    fresh->final();
    delete fresh;
    dut = saved;
    return true;
}

// ─── Scenario 15: RTC CKO timing ───────────────────────────────────────
static bool test_cko_half_second_timing() {
    const char* fast_argv[] = { "tb_rtc", "+rtc_fast", nullptr };
    Verilated::commandArgs(2, const_cast<char**>(fast_argv));
    Vrtc* fresh = new Vrtc;
    Vrtc* saved = dut;
    dut = fresh;

    reset();
    CHECK_EQ("CKO starts high", dut->cko, 1);
    CHECK_EQ("seconds start before CKO edges", read_seconds32(), 0);

    phi2_ticks(500);
    CHECK_EQ("CKO low at first half-second", dut->cko, 0);
    CHECK_EQ("seconds unchanged on CKO falling edge", read_seconds32(), 0);

    phi2_ticks(500);
    CHECK_EQ("CKO high at second half-second", dut->cko, 1);
    CHECK_EQ("seconds advance on CKO rising edge", read_seconds32(), 1);

    fresh->final();
    delete fresh;
    dut = saved;
    return true;
}

// ─── Scenario 17: PRAM survives a warm reset (battery-backed) ──────────
//
// THE feature test.  On a real Macintosh the PRAM lives behind a lithium
// cell inside the RTC IC: user settings (display depth, boot device,
// sound volume, AppleTalk node) survive both power cycles and warm
// resets.  rtc.v models that by deliberately NOT resetting the array.
//
// Positive control: this test would pass trivially if `rst` were dead
// (unconnected, stuck low, whatever), so it also proves the SAME reset
// pulse really did rewind the module's other state — write_protect and
// the seconds counter.  Without that, "PRAM survived" would be
// indistinguishable from "nothing happened".
static bool test_pram_survives_warm_reset() {
    ScopedDut sd{"+rtc_populated_pram"};
    reset();   // full isolation: rst + explicit zap → known power-on image

    // Distinctive values, both different from the power-on image
    // (pram[8] powers on at 0x13, xpram[0x55] at 0x00).
    rtc_transaction(0x20, 0x5A);        // classic PRAM reg 8  <- 0x5A
    xpram_write(0x55, 0xA6);            // extended PRAM 0x55  <- 0xA6
    CHECK_EQ("pram[8] written before reset",  rtc_transaction(0xA0, 0), 0x5A);
    CHECK_EQ("xpram[0x55] written before reset", xpram_read(0x55), 0xA6);

    // Dirty two pieces of state that MUST rewind, so the reset pulse is
    // provably live.
    rtc_transaction(0x34, 0x80);        // write_protect <- 1
    CHECK_EQ("write-protect set before reset", rtc_transaction(0xB5, 0), 0x80);
    rtc_transaction(0x37, 0x00);        // clear WP again so the PRAM
                                        // readback path below is open
    rtc_transaction(0x01, 0x77);        // seconds[7:0] <- 0x77
    CHECK_EQ("seconds seeded before reset", read_seconds32() & 0xFF, 0x77);
    rtc_transaction(0x34, 0x80);        // re-arm write_protect
    CHECK_EQ("write-protect re-armed", rtc_transaction(0xB5, 0), 0x80);

    // Warm reset: `rst` ONLY — no pram_clear.
    warm_reset();

    // Positive control — these MUST have rewound.
    CHECK_EQ("control: write-protect cleared by rst", rtc_transaction(0xB5, 0), 0x00);
    CHECK_EQ("control: seconds cleared by rst", read_seconds32(), 0x00000000);

    // The actual feature — these MUST have survived.
    CHECK_EQ("pram[8] survives warm reset",     rtc_transaction(0xA0, 0), 0x5A);
    CHECK_EQ("xpram[0x55] survives warm reset", xpram_read(0x55), 0xA6);

    // Untouched bytes keep their power-on image across the reset too —
    // i.e. the array is genuinely undisturbed, not partially rewritten.
    CHECK_EQ("pram[9] undisturbed by warm reset",     rtc_transaction(0xA4, 0), 0x88);
    CHECK_EQ("xpram[0xf8] undisturbed by warm reset", xpram_read(0xF8), 0x53);

    // And it survives a SECOND reset — persistence is not a one-shot.
    warm_reset();
    CHECK_EQ("pram[8] survives a second warm reset",     rtc_transaction(0xA0, 0), 0x5A);
    CHECK_EQ("xpram[0x55] survives a second warm reset", xpram_read(0x55), 0xA6);
    return true;
}

// ─── Scenario 18: pram_clear is the escape hatch (Cmd-Opt-P-R) ─────────
//
// Since `rst` no longer wipes PRAM, `pram_clear` is the ONLY way back to
// a known-good image — the recovery path if a bad PRAM image ever wedges
// the ROM boot.  Runs on the populated image on purpose: clearing to a
// non-zero image proves the port restores pram_reset_value() rather than
// merely zeroing (which an all-zero default would not distinguish).
static bool test_pram_clear_restores_power_on_image() {
    ScopedDut sd{"+rtc_populated_pram"};
    reset();

    rtc_transaction(0x20, 0x5A);        // pram[8]:      0x13 -> 0x5A
    xpram_write(0x02, 0x99);            // xpram[0x02]:  0x4F -> 0x99
    xpram_write(0x55, 0xA6);            // xpram[0x55]:  0x00 -> 0xA6
    CHECK_EQ("pram[8] dirtied",    rtc_transaction(0xA0, 0), 0x5A);
    CHECK_EQ("xpram[2] dirtied",   xpram_read(0x02), 0x99);
    CHECK_EQ("xpram[0x55] dirtied", xpram_read(0x55), 0xA6);

    // Seed some non-PRAM state so we can prove the zap is PRAM-scoped
    // and does not double as a general reset.
    rtc_transaction(0x01, 0x77);        // seconds[7:0] <- 0x77
    CHECK_EQ("seconds seeded before zap", read_seconds32() & 0xFF, 0x77);

    // Zap — no rst involved.
    pram_zap();

    CHECK_EQ("pram[8] back to power-on image",     rtc_transaction(0xA0, 0), 0x13);
    CHECK_EQ("xpram[2] back to power-on image",    xpram_read(0x02), 0x4F);
    CHECK_EQ("xpram[0x55] back to power-on image", xpram_read(0x55), 0x00);
    CHECK_EQ("xpram[0xf8] still 'S' after zap",    xpram_read(0xF8), 0x53);

    // PRAM-scoped: the seconds counter is NOT collateral damage.
    CHECK_EQ("zap leaves seconds alone", read_seconds32() & 0xFF, 0x77);

    // The array stays writable afterwards.
    rtc_transaction(0x20, 0x3C);
    CHECK_EQ("pram writable after zap", rtc_transaction(0xA0, 0), 0x3C);
    return true;
}

static void ext_write(uint8_t addr, uint8_t value) {
    dut->pram_ext_addr = addr;
    dut->pram_ext_wdata = value;
    dut->pram_ext_we = 1;
    tick();
    dut->pram_ext_we = 0;
}

static uint8_t ext_read(uint8_t addr) {
    dut->pram_ext_addr = addr;
    tick();
    return dut->pram_ext_rdata;
}

static bool test_pram_bram_cross_ports_and_sweep() {
    ScopedDut sd{};
    reset();
    uint32_t rng = 0x68c040;
    uint8_t pattern[256];
    for (unsigned i = 0; i < 256; ++i) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        pattern[i] = rng;
        ext_write(i, pattern[i]);
    }
    for (unsigned i = 0; i < 256; ++i)
        CHECK_EQ("external write / serial read", xpram_read(i), pattern[i]);
    for (unsigned i = 0; i < 256; ++i) xpram_write(i, pattern[i] ^ 0xff);
    warm_reset();
    for (unsigned i = 0; i < 256; ++i)
        CHECK_EQ("serial write / external read after warm reset", ext_read(i), pattern[i] ^ 0xff);

    dut->pram_clear = 1; tick();
    dut->pram_clear = 0;
    // Ordinary reset halfway through the sweep must not cancel/restart it.
    for (unsigned i = 0; i < 255; ++i) {
        dut->rst = (i >= 100 && i < 110);
        tick();
        CHECK_TRUE("busy until last byte", dut->pram_busy);
    }
    tick();
    CHECK_EQ("sweep ends after exactly 256 writes", dut->pram_busy, 0);
    for (unsigned i = 0; i < 256; ++i) CHECK_EQ("all bytes cleared", ext_read(i), 0);
    // Restores while the Mac is held in reset remain legal.
    dut->rst = 1;
    ext_write(0xff, 0xa7);
    CHECK_EQ("restore during reset", ext_read(0xff), 0xa7);
    dut->rst = 0; tick();
    CHECK_EQ("restore survives reset release", xpram_read(0xff), 0xa7);

    dut->pram_clear = 1;
    for (unsigned i = 0; i < 300; ++i) tick();
    CHECK_TRUE("held clear keeps port unavailable", dut->pram_busy);
    dut->pram_clear = 0; tick();
    CHECK_EQ("held clear does not repeatedly restart sweep", dut->pram_busy, 0);
    ext_write(0xff, 0x42);
    dut->pram_clear = 1; tick();
    dut->pram_clear = 0;
    wait_pram_clear();
    CHECK_EQ("second clear removes later write", ext_read(0xff), 0);
    return true;
}

static bool test_pram_bram_collisions() {
    ScopedDut sd{};
    reset();
    for (unsigned same = 0; same < 2; ++same) {
        // Finish a serial write on exactly the same edge as an external write.
        dut->rtc_enb = 0; tick();
        for (int bit = 7; bit >= 0; --bit) shift_bit_in((0x20 >> bit) & 1);
        for (int bit = 7; bit >= 0; --bit) shift_bit_in((0x5a >> bit) & 1);
        dut->rtc_enb = 1;
        ext_write(same ? 8 : 9, 0xa5);
        CHECK_EQ("same-byte external write wins", xpram_read(8), same ? 0xa5 : 0x5a);
        CHECK_EQ("different-byte external write also survives", xpram_read(9), 0xa5);
    }
    // External write at serial read-command completion: serial must latch OLD
    // data, not the replacement arriving on the other READ_FIRST RAM port.
    ext_write(8, 0xc3);
    dut->rtc_enb = 0; tick();
    for (int bit = 7; bit >= 1; --bit) shift_bit_in((0xa0 >> bit) & 1);
    dut->rtc_data_o = 0; dut->rtc_clk = 1; tick();
    dut->rtc_clk = 0;
    ext_write(8, 0x69);
    uint8_t out = 0;
    for (int bit = 7; bit >= 0; --bit) out = (out << 1) | shift_bit_out();
    dut->rtc_enb = 1; tick();
    CHECK_EQ("serial read-before-write collision", out, 0xc3);
    CHECK_EQ("external collision write stored", ext_read(8), 0x69);

    dut->pram_clear = 1;
    ext_write(8, 0xff);
    dut->pram_clear = 0;
    wait_pram_clear();
    CHECK_EQ("clear takes priority over raw external strobe", ext_read(8), 0);
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
    dut = new Vrtc;

    // The PRAM-default flip (rtc.v: default PRAM image is now MAME's
    // all-zero NVRAM) had disabled the five populated-image tests below.
    // They are re-enabled here: each now constructs its own DUT with
    // +rtc_populated_pram via ScopedDut, so they assert the populated
    // image explicitly instead of depending on a global default.
    RUN(test_reset_defaults);
    RUN(test_read_seconds);
    RUN(test_pram_byte_addressing);
    RUN(test_sequence_and_idle);
    RUN(test_incomplete_write_is_ignored);
    RUN(test_control_bits_and_write_protect);
    RUN(test_seconds_advance_over_time);
    RUN(test_mame_seconds_byte_order);
    RUN(test_write_seconds_byte_order);
    RUN(test_mame_falling_edge_sampling);
    RUN(test_extended_pram_commands);
    RUN(test_seconds_read_latch_during_polling);
    RUN(test_trace_mode_smoke);
    RUN(test_rom_style_pullup_one_bits);
    RUN(test_mame_state_reset_defaults);
    RUN(test_mac_style_date_and_pram_roundtrip);
    RUN(test_cko_half_second_timing);
    // Battery-backed PRAM: persistence across rst, and the explicit zap.
    RUN(test_pram_survives_warm_reset);
    RUN(test_pram_clear_restores_power_on_image);
    RUN(test_pram_bram_cross_ports_and_sweep);
    RUN(test_pram_bram_collisions);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
