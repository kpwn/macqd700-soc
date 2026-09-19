// tb/tb_sonic_trace_ring.cpp — unit tb for rtl/soc/sonic_trace_ring.v
//
// The ring is the instrument used to answer "what register sequence does the
// Mac's Ethernet driver actually issue to the SONIC before receptions stop
// being serviced".  An instrument that silently records the wrong thing is
// worse than no instrument: it produces a confident wrong answer.  So the
// properties this tb cares most about are the ones the whole measurement
// rests on.
//
//   1. A CONSTANT POLL LOOP MUST PRODUCE EXACTLY ONE RING ENTRY.
//      That is what makes the ring self-freezing on a wedge and therefore
//      what preserves the pre-wedge history.  Wrong in the "records
//      everything" direction and a spin on ISR overwrites all 4096 entries,
//      so every dump shows nothing but the wedge itself; wrong in the
//      "records nothing" direction and an empty dump is indistinguishable
//      from a broken tap.  test_poll_filter_collapses covers the first,
//      test_value_change_is_recorded the second, and
//      test_poll_filter_preserves_context proves the surrounding commands
//      survive a spin between them.
//
//   2. BYTE STROBES MUST BE CAPTURED.  This driver does half-register
//      writes, and q700_eth_sonic decodes CR's two halves as completely
//      different commands (low byte -> sonic_command_low(), high byte ->
//      RRRA/LCAM).  A trace that lost the strobes could not tell "wrote
//      0x0002 to CR (TXP)" from "wrote 0x00 to CR's high half (nothing)".
//      test_byte_strobes_recorded covers it.
//
// POSITIVE CONTROL (--positive-control): runs the same assertions with
// pb_ack held low, so the DUT can never commit an entry.  Every capture
// assertion must then FAIL.  The Makefile target runs this FIRST and
// requires it to report failures; a suite that cannot fail is not evidence.

#include "Vsonic_trace_ring.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

static Vsonic_trace_ring *dut;
static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

static int failures = 0;
static bool positive_control = false;

#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            failures++;                                                        \
            printf("  FAIL: ");                                                \
            printf(__VA_ARGS__);                                               \
            printf("   [%s:%d]\n", __FILE__, __LINE__);                        \
        }                                                                      \
    } while (0)

// SONIC register indices used below (rtl/mac/q700_eth_sonic.v localparams).
enum { REG_CR = 0x00, REG_DCR = 0x01, REG_RCR = 0x02, REG_IMR = 0x04,
       REG_ISR = 0x05, REG_CDP = 0x26, REG_CDC = 0x27 };

static void tick() {
    dut->pb_clk = 0;
    dut->rd_clk = 0;
    dut->eval();
    main_time++;
    dut->pb_clk = 1;
    dut->rd_clk = 1;
    dut->eval();
    main_time++;
}

static void idle(int n) {
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->pb_ack = 0;
    for (int i = 0; i < n; i++) tick();
}

static void reset_dut() {
    dut->pb_rst = 1;
    dut->rd_rst = 1;
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->pb_ack = 0;
    dut->pb_addr = 0;
    dut->pb_wdata = 0;
    dut->pb_wstrb = 0;
    dut->pb_rdata = 0;
    dut->rd_freeze = 0;
    dut->rd_clear = 0;
    dut->rd_addr = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->pb_rst = 0;
    dut->rd_rst = 0;
    for (int i = 0; i < 8; i++) tick();
}

// Drive one beat exactly the way q700_eth_sonic answers it: request and ack
// in the SAME cycle (`sonic_ack = sonic_cs && (sonic_rd||sonic_wr)` is
// combinational), with sonic_rdata valid on that cycle.
//
// pb_wstrb is deliberately driven to a NON-3 junk value during reads,
// mirroring the real port where sonic_wstrb comes from the write channel's
// latched state and is meaningless while reading.  A DUT that sampled it on
// reads would record that junk.
static void beat(bool is_write, int reg, uint16_t data, int strb = 3) {
    dut->pb_addr = reg & 0x3f;
    dut->pb_wr = is_write ? 1 : 0;
    dut->pb_rd = is_write ? 0 : 1;
    dut->pb_wdata = is_write ? data : 0xDEAD;
    dut->pb_wstrb = is_write ? strb : 0x1;   // junk on reads, see above
    dut->pb_rdata = is_write ? 0xFFFF : data;
    dut->pb_ack = positive_control ? 0 : 1;
    tick();
    idle(1);
}

// Same beat, but the device withholds ack for `delay` cycles while
// peripheral_bus holds the request.  Today's SONIC never does this; the DUT
// keeps the latch path so a future registered-ack device still records.
static void beat_delayed(bool is_write, int reg, uint16_t data, int strb,
                         int delay) {
    dut->pb_addr = reg & 0x3f;
    dut->pb_wr = is_write ? 1 : 0;
    dut->pb_rd = is_write ? 0 : 1;
    dut->pb_wdata = is_write ? data : 0xDEAD;
    dut->pb_wstrb = is_write ? strb : 0x1;
    dut->pb_rdata = 0xFFFF;
    dut->pb_ack = 0;
    for (int i = 0; i < delay; i++) tick();
    // Request still held; ack + read data land now.
    dut->pb_rdata = is_write ? 0xFFFF : data;
    dut->pb_ack = positive_control ? 0 : 1;
    tick();
    idle(1);
}

static uint32_t ring_read(int idx) {
    dut->rd_addr = idx;
    tick();
    tick();
    return dut->rd_data;
}

static uint32_t wrptr() {
    for (int i = 0; i < 4; i++) tick();
    return dut->rd_wrptr;
}

static uint32_t filtered() {
    for (int i = 0; i < 4; i++) tick();
    return dut->rd_filtered;
}

static void rearm() {
    dut->rd_freeze = 0;
    dut->rd_clear = !dut->rd_clear;   // TOGGLE
    for (int i = 0; i < 12; i++) tick();
}

struct Entry {
    bool wr;
    int strb;
    int reg;
    uint16_t data;
    int skipped;
};
static Entry decode(uint32_t e) {
    Entry x;
    x.wr      = (e >> 31) & 1;
    x.strb    = (e >> 29) & 0x3;
    x.reg     = (e >> 23) & 0x3f;
    x.data    = (e >> 7) & 0xffff;
    x.skipped = e & 0x7f;
    return x;
}

// ── Tests ────────────────────────────────────────────────────────────────

static void test_basic_capture() {
    printf("test_basic_capture\n");
    reset_dut();
    rearm();
    beat(true, REG_IMR, 0x0f00);          // W IMR = enable interrupts
    beat(false, REG_ISR, 0x0200);         // R ISR = TXDN pending
    beat(true, REG_CDP, 0x1234);          // W CAM descriptor pointer
    CHECK(wrptr() == 3, "expected 3 entries, wr_ptr=%u", wrptr());

    Entry a = decode(ring_read(0));
    CHECK(a.wr && a.reg == REG_IMR && a.data == 0x0f00 && a.strb == 3,
          "entry0 wrong: wr=%d reg=%02x d=%04x strb=%d",
          a.wr, a.reg, a.data, a.strb);
    Entry b = decode(ring_read(1));
    CHECK(!b.wr && b.reg == REG_ISR && b.data == 0x0200,
          "entry1 wrong: wr=%d reg=%02x d=%04x", b.wr, b.reg, b.data);
    Entry c = decode(ring_read(2));
    CHECK(c.wr && c.reg == REG_CDP && c.data == 0x1234,
          "entry2 wrong: wr=%d reg=%02x d=%04x", c.wr, c.reg, c.data);
}

// Register index must be the full 6 bits — the CAM registers the driver
// programs live at 0x26/0x27, above a 4-bit field.
static void test_full_register_index() {
    printf("test_full_register_index\n");
    reset_dut();
    rearm();
    beat(true, REG_CDC, 0x000f);          // CAM descriptor count = 15 entries
    beat(true, 0x3f, 0xbeef);             // DCR2, the highest index
    CHECK(wrptr() == 2, "expected 2 entries, wr_ptr=%u", wrptr());
    CHECK(decode(ring_read(0)).reg == REG_CDC, "reg 0x27 truncated to 0x%02x",
          decode(ring_read(0)).reg);
    CHECK(decode(ring_read(1)).reg == 0x3f, "reg 0x3f truncated to 0x%02x",
          decode(ring_read(1)).reg);
}

// Half-register writes are how this driver works, and CR's two halves are
// different commands.  The strobes must survive.
static void test_byte_strobes_recorded() {
    printf("test_byte_strobes_recorded\n");
    reset_dut();
    rearm();
    beat(true, REG_CR, 0x0002, 0x1);      // low half only: TXP command
    beat(true, REG_CR, 0x0200, 0x2);      // high half only: LCAM command
    beat(true, REG_CR, 0x0002, 0x3);      // full 16-bit write
    beat(false, REG_ISR, 0x0000);         // a read: strobes must read 2'b11
    CHECK(wrptr() == 4, "expected 4 entries, wr_ptr=%u", wrptr());

    Entry a = decode(ring_read(0));
    CHECK(a.wr && a.strb == 0x1 && a.data == 0x0002,
          "low-half CR write lost its strobes: strb=%d d=%04x", a.strb, a.data);
    Entry b = decode(ring_read(1));
    CHECK(b.wr && b.strb == 0x2 && b.data == 0x0200,
          "high-half CR write lost its strobes: strb=%d d=%04x", b.strb, b.data);
    Entry c = decode(ring_read(2));
    CHECK(c.wr && c.strb == 0x3, "full CR write strb=%d", c.strb);
    Entry d = decode(ring_read(3));
    CHECK(!d.wr && d.strb == 0x3,
          "a read must record strb=3 (the SONIC returns the whole register); "
          "got strb=%d — the DUT is sampling the write channel's junk",
          d.strb);
}

// THE load-bearing property: a driver spinning on ISR (the register a wedged
// SONIC driver polls, because with IMR=0 no interrupt can ever assert) must
// not be able to scroll the pre-wedge history out of the ring.
static void test_poll_filter_collapses() {
    printf("test_poll_filter_collapses\n");
    reset_dut();
    rearm();
    for (int i = 0; i < 2000; i++) beat(false, REG_ISR, 0x0000);
    CHECK(wrptr() == 1, "2000 identical ISR polls should collapse to 1 entry, got %u",
          wrptr());
    Entry a = decode(ring_read(0));
    CHECK(!a.wr && a.reg == REG_ISR && a.data == 0x0000, "collapsed entry wrong");
}

// The same, in the shape the dump is actually read in: the commands
// SURROUNDING a spin must still be there, in order, after the spin.  This is
// what "the ring stops advancing on its own" is for.
static void test_poll_filter_preserves_context() {
    printf("test_poll_filter_preserves_context\n");
    reset_dut();
    rearm();
    beat(true, REG_IMR, 0x0f00);
    beat(true, REG_CR, 0x0002, 0x1);       // issue transmit
    for (int i = 0; i < 6000; i++) beat(false, REG_CR, 0x0002);  // spin on TXP
    beat(true, REG_ISR, 0x0200);           // finally: clear TXDN
    CHECK(wrptr() == 4,
          "spin must not evict surrounding commands: expected 4 entries, got %u",
          wrptr());
    Entry a = decode(ring_read(0));
    Entry b = decode(ring_read(1));
    Entry c = decode(ring_read(2));
    Entry d = decode(ring_read(3));
    CHECK(a.wr && a.reg == REG_IMR && a.data == 0x0f00, "e0: IMR write lost");
    CHECK(b.wr && b.reg == REG_CR && b.strb == 0x1, "e1: CR TXP write lost");
    CHECK(!c.wr && c.reg == REG_CR && c.data == 0x0002, "e2: first poll read lost");
    CHECK(d.wr && d.reg == REG_ISR && d.data == 0x0200, "e3: ISR clear lost");
    // The spin is still VISIBLE, as the suppressed count on the entry that
    // follows it (saturating at 127) and in the live filtered total.
    CHECK(d.skipped == 127, "entry after a 6000-deep spin should carry the "
                            "saturated skipped count 127, got %d", d.skipped);
    CHECK(filtered() == 5999, "live filtered total should be 5999, got %u",
          filtered());
}

// The other direction: real value changes must all survive, or the trace
// would be missing the transitions that carry all the information.
static void test_value_change_is_recorded() {
    printf("test_value_change_is_recorded\n");
    reset_dut();
    rearm();
    for (int i = 0; i < 5; i++) beat(false, REG_ISR, 0x0000);
    for (int i = 0; i < 2; i++) beat(false, REG_ISR, 0x0200);
    beat(false, REG_ISR, 0x0210);
    CHECK(wrptr() == 3, "expected 3 distinct values recorded, got %u", wrptr());
    CHECK(decode(ring_read(0)).data == 0x0000, "v0");
    CHECK(decode(ring_read(1)).data == 0x0200, "v1");
    CHECK(decode(ring_read(2)).data == 0x0210, "v2");
}

// Writes are commands.  "The driver re-issued CR_TXP eight times and the
// eighth never completed" is exactly the signal being hunted, so identical
// consecutive writes must NEVER be filtered.
static void test_repeated_writes_all_recorded() {
    printf("test_repeated_writes_all_recorded\n");
    reset_dut();
    rearm();
    for (int i = 0; i < 32; i++) beat(true, REG_CR, 0x0002, 0x1);
    CHECK(wrptr() == 32, "32 identical CR=TXP writes must all be recorded, got %u",
          wrptr());
    CHECK(filtered() == 0, "writes must never be counted as filtered, got %u",
          filtered());
}

// A write to a register must invalidate that register's read shadow.  Almost
// every SONIC register is write-masked and CR is command-decoded, so the
// read-back after a write is the only place the trace shows what the write
// ACTUALLY did — it must never be dropped for matching a pre-write value.
static void test_write_invalidates_read_shadow() {
    printf("test_write_invalidates_read_shadow\n");
    reset_dut();
    rearm();
    beat(false, REG_CR, 0x0084);  // recorded, shadow = 0x0084
    beat(false, REG_CR, 0x0084);  // dropped
    CHECK(wrptr() == 1, "setup: expected 1, got %u", wrptr());
    beat(true, REG_CR, 0x0000, 0x1);
    beat(false, REG_CR, 0x0084);  // must be recorded again
    CHECK(wrptr() == 3, "read after write must not be filtered, wr_ptr=%u", wrptr());
    Entry e = decode(ring_read(2));
    CHECK(!e.wr && e.data == 0x0084, "entry2 wrong: wr=%d d=%04x", e.wr, e.data);
}

// Per-register shadows: a spin on ISR must not evict CR's shadow (or vice
// versa), or two alternating polls would each keep re-admitting the other.
static void test_per_register_shadow() {
    printf("test_per_register_shadow\n");
    reset_dut();
    rearm();
    for (int i = 0; i < 500; i++) {
        beat(false, REG_ISR, 0x0000);
        beat(false, REG_CR, 0x0002);
    }
    CHECK(wrptr() == 2,
          "alternating ISR/CR polls at constant values must collapse to 2 "
          "entries (one shadow each), got %u", wrptr());
    CHECK(decode(ring_read(0)).reg == REG_ISR, "e0 should be ISR");
    CHECK(decode(ring_read(1)).reg == REG_CR, "e1 should be CR");
}

// "Driver is hammering ISR and never sees a bit set" vs "driver stopped
// touching the SONIC completely" are different diagnoses.  The live filtered
// counter is what separates them WITHOUT freezing the ring.
static void test_live_filtered_counter() {
    printf("test_live_filtered_counter\n");
    reset_dut();
    rearm();
    CHECK(filtered() == 0, "filtered should start at 0, got %u", filtered());
    for (int i = 0; i < 10; i++) beat(false, REG_ISR, 0x0000);
    uint32_t f1 = filtered();
    CHECK(f1 == 9, "10 identical polls -> 1 recorded + 9 filtered, got %u", f1);
    uint32_t p1 = wrptr();
    for (int i = 0; i < 40; i++) beat(false, REG_ISR, 0x0000);
    CHECK(wrptr() == p1, "a spin must not advance wr_ptr, %u -> %u", p1, wrptr());
    CHECK(filtered() == 49, "filtered must keep climbing during the spin, got %u",
          filtered());
    rearm();
    CHECK(filtered() == 0, "re-arm must clear the filtered total, got %u",
          filtered());
}

// The per-entry skipped count must reset after each recorded entry, so a
// nonzero value always means "the driver spun immediately before THIS event".
static void test_skipped_count_resets() {
    printf("test_skipped_count_resets\n");
    reset_dut();
    rearm();
    beat(false, REG_ISR, 0x0000);              // e0, skipped = 0
    for (int i = 0; i < 5; i++) beat(false, REG_ISR, 0x0000);  // 5 dropped
    beat(false, REG_ISR, 0x0200);              // e1, skipped = 5
    beat(false, REG_ISR, 0x0000);              // e2, skipped = 0 again
    CHECK(wrptr() == 3, "expected 3 entries, got %u", wrptr());
    CHECK(decode(ring_read(0)).skipped == 0, "e0 skipped=%d",
          decode(ring_read(0)).skipped);
    CHECK(decode(ring_read(1)).skipped == 5, "e1 skipped=%d (expected 5)",
          decode(ring_read(1)).skipped);
    CHECK(decode(ring_read(2)).skipped == 0, "e2 skipped=%d (expected 0 — the "
          "count must reset after every recorded entry)",
          decode(ring_read(2)).skipped);
}

static void test_freeze_and_rearm() {
    printf("test_freeze_and_rearm\n");
    reset_dut();
    rearm();
    beat(true, REG_IMR, 0x0f00);
    CHECK(wrptr() == 1, "pre-freeze wr_ptr=%u", wrptr());
    dut->rd_freeze = 1;
    for (int i = 0; i < 12; i++) tick();
    beat(true, REG_RCR, 0x4000);
    beat(true, REG_DCR, 0x0020);
    CHECK(wrptr() == 1, "frozen ring must not advance, wr_ptr=%u", wrptr());
    tick();
    CHECK(dut->rd_frozen == 1, "rd_frozen should read 1 while frozen");
    // The captured window must be intact, not overwritten by the two
    // post-freeze writes.
    Entry a = decode(ring_read(0));
    CHECK(a.wr && a.reg == REG_IMR && a.data == 0x0f00,
          "frozen entry was overwritten: reg=%02x d=%04x", a.reg, a.data);
    rearm();
    CHECK(wrptr() == 0, "re-arm must rewind wr_ptr, got %u", wrptr());
    tick();
    CHECK(dut->rd_frozen == 0, "re-arm must clear frozen");
    beat(true, REG_RCR, 0x4000);
    CHECK(wrptr() == 1, "capture must resume after re-arm, wr_ptr=%u", wrptr());
}

// Re-arm must wipe the filter shadow, so every capture window is
// SELF-CONTAINED: the first read of each register after a re-arm is always
// recorded, even if it happens to match a value read in the previous window.
// Without this, entry 0 of a dump could be missing because of history the
// dump does not contain — a silent hole in the trace.
static void test_rearm_resets_filter_baseline() {
    printf("test_rearm_resets_filter_baseline\n");
    reset_dut();
    rearm();
    beat(false, REG_ISR, 0x0200);
    beat(false, REG_ISR, 0x0200);   // filtered within this window
    CHECK(wrptr() == 1, "setup: expected 1 entry, got %u", wrptr());
    rearm();
    beat(false, REG_ISR, 0x0200);   // same value, NEW window -> must record
    CHECK(wrptr() == 1,
          "re-arm must wipe the read shadow so the new window opens with a "
          "baseline entry; got %u entries", wrptr());
    Entry a = decode(ring_read(0));
    CHECK(!a.wr && a.reg == REG_ISR && a.data == 0x0200,
          "baseline entry wrong: wr=%d reg=%02x d=%04x", a.wr, a.reg, a.data);
    beat(false, REG_ISR, 0x0200);   // and the filter is live again
    CHECK(wrptr() == 1, "filter must be live again after the baseline, got %u",
          wrptr());
}

static void test_wrap_sets_flag() {
    printf("test_wrap_sets_flag\n");
    reset_dut();
    rearm();
    // 4096 writes -> exactly one full lap.  Writes, so nothing is filtered.
    for (int i = 0; i < 4096; i++)
        beat(true, REG_CDP, (uint16_t)(i & 0xffff));
    tick();
    CHECK(dut->rd_wrapped == 1, "wrapped flag should be set after a full lap");
    CHECK(wrptr() == 0, "wr_ptr should be back at 0, got %u", wrptr());
    // Two more: they must land at 0 and 1, overwriting the oldest entries.
    beat(true, REG_CDC, 0xAAAA);
    beat(true, REG_CDC, 0xBBBB);
    CHECK(wrptr() == 2, "wr_ptr should be 2 after wrap+2, got %u", wrptr());
    CHECK(decode(ring_read(0)).data == 0xAAAA, "wrapped entry0 d=%04x",
          decode(ring_read(0)).data);
    CHECK(decode(ring_read(1)).data == 0xBBBB, "wrapped entry1 d=%04x",
          decode(ring_read(1)).data);
}

// A device that withholds ack must still be recorded exactly once, with the
// data present on the ACK cycle.
static void test_delayed_ack_beat() {
    printf("test_delayed_ack_beat\n");
    reset_dut();
    rearm();
    beat_delayed(true, REG_RCR, 0x4000, 0x3, 3);
    beat_delayed(false, REG_RCR, 0x4000, 0x3, 2);
    CHECK(wrptr() == 2, "ack-withheld beats: expected 2 entries, got %u", wrptr());
    Entry a = decode(ring_read(0));
    CHECK(a.wr && a.reg == REG_RCR && a.data == 0x4000 && a.strb == 3,
          "delayed write entry wrong: wr=%d reg=%02x d=%04x strb=%d",
          a.wr, a.reg, a.data, a.strb);
    Entry b = decode(ring_read(1));
    CHECK(!b.wr && b.reg == REG_RCR && b.data == 0x4000,
          "delayed read entry wrong: wr=%d reg=%02x d=%04x", b.wr, b.reg, b.data);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--positive-control")) positive_control = true;

    dut = new Vsonic_trace_ring;

    if (positive_control)
        printf("=== POSITIVE CONTROL: pb_ack held low, capture assertions MUST fail ===\n");

    test_basic_capture();
    test_full_register_index();
    test_byte_strobes_recorded();
    test_poll_filter_collapses();
    test_poll_filter_preserves_context();
    test_value_change_is_recorded();
    test_repeated_writes_all_recorded();
    test_write_invalidates_read_shadow();
    test_per_register_shadow();
    test_live_filtered_counter();
    test_skipped_count_resets();
    test_freeze_and_rearm();
    test_rearm_resets_filter_baseline();
    test_wrap_sets_flag();
    test_delayed_ack_beat();

    delete dut;

    // Exit-code convention matches tb-scsi-trace-ring: make aborts on
    // non-zero, so the positive-control run must exit 0 when the assertions
    // correctly FAILED, and non-zero when they did not fail (i.e. the suite
    // is inert).
    if (positive_control) {
        if (failures == 0) {
            printf("POSITIVE CONTROL DID NOT FAIL - the assertions are inert.\n");
            return 1;
        }
        printf("positive control failed as required (%d checks) - assertions are live\n",
               failures);
        return 0;
    }

    if (failures) {
        printf("FAILED: %d check(s)\n", failures);
        return 1;
    }
    printf("all sonic_trace_ring tests passed\n");
    return 0;
}
