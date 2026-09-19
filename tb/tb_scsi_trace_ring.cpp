// tb/tb_scsi_trace_ring.cpp — unit tb for rtl/soc/scsi_trace_ring.v
//
// The ring is the instrument used to answer "what register sequence does
// the Mac OS SCSI Manager actually issue before the 53C96 wedges".  An
// instrument that silently records the wrong thing is worse than no
// instrument -- it produces a confident wrong answer.  So the property
// this tb cares most about is the one the whole measurement rests on:
//
//     A CONSTANT POLL LOOP MUST PRODUCE EXACTLY ONE RING ENTRY.
//
// That is what makes the ring self-freezing on a wedge and therefore what
// preserves the pre-wedge history.  If it were wrong in the "records
// everything" direction, the poll loop would overwrite the 4096-entry ring
// and every dump would show nothing but the wedge itself.  If it were
// wrong in the "records nothing" direction, an empty dump would be
// indistinguishable from a broken tap.  test_poll_filter_collapses covers
// the first; test_value_change_is_recorded covers the second.
//
// POSITIVE CONTROL (--positive-control): runs the same assertions with
// pb_ack held low, so the DUT can never commit an entry.  Every capture
// assertion must then FAIL.  The Makefile target runs this FIRST and
// requires a non-zero exit; a suite that cannot fail is not evidence.

#include "Vscsi_trace_ring.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static Vscsi_trace_ring *dut;
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

static void reset_dut() {
    dut->pb_rst = 1;
    dut->rd_rst = 1;
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->pb_ack = 0;
    dut->pb_addr = 0;
    dut->pb_wdata = 0;
    dut->pb_rdata = 0;
    dut->rd_freeze = 0;
    dut->rd_clear = 0;
    dut->rd_addr = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->pb_rst = 0;
    dut->rd_rst = 0;
    for (int i = 0; i < 8; i++) tick();
}

// Drive one peripheral-bus beat the way scsi.v answers it: the request is
// held for one cycle and pb_ack comes back the NEXT cycle, with pb_rdata
// valid on the ack cycle.
static void beat(bool is_write, int reg, uint8_t data, bool dma_shim = false) {
    dut->pb_addr = dma_shim ? 0x100 : (reg & 0xf);
    dut->pb_wr = is_write ? 1 : 0;
    dut->pb_rd = is_write ? 0 : 1;
    dut->pb_wdata = is_write ? data : 0;
    tick();
    // ack cycle
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->pb_ack = positive_control ? 0 : 1;
    dut->pb_rdata = is_write ? 0 : data;
    tick();
    dut->pb_ack = 0;
    tick();
}

static uint32_t ring_read(int idx) {
    dut->rd_addr = idx;
    tick();
    tick();
    return dut->rd_data;
}

static uint32_t wrptr() {
    tick();
    return dut->rd_wrptr;
}

static void clear_ring() {
    dut->rd_freeze = 0;
    dut->rd_clear = !dut->rd_clear; // TOGGLE
    for (int i = 0; i < 8; i++) tick();
}

struct Entry {
    bool wr, dma;
    int reg;
    uint8_t data;
};
static Entry decode(uint32_t e) {
    Entry x;
    x.wr = (e >> 31) & 1;
    x.dma = (e >> 30) & 1;
    x.reg = (e >> 26) & 0xf;
    x.data = (e >> 18) & 0xff;
    return x;
}

// ── Tests ────────────────────────────────────────────────────────────────

static void test_basic_capture() {
    printf("test_basic_capture\n");
    reset_dut();
    clear_ring();
    beat(true, 3, 0xc1);  // W3 = SELECT
    beat(false, 4, 0x02); // R4 = COMMAND phase
    beat(true, 0, 0x10);  // W0 = TC lsb
    CHECK(wrptr() == 3, "expected 3 entries, wr_ptr=%u", wrptr());

    Entry a = decode(ring_read(0));
    CHECK(a.wr && a.reg == 3 && a.data == 0xc1, "entry0 wrong: wr=%d reg=%d d=%02x",
          a.wr, a.reg, a.data);
    Entry b = decode(ring_read(1));
    CHECK(!b.wr && b.reg == 4 && b.data == 0x02, "entry1 wrong: wr=%d reg=%d d=%02x",
          b.wr, b.reg, b.data);
    Entry c = decode(ring_read(2));
    CHECK(c.wr && c.reg == 0 && c.data == 0x10, "entry2 wrong: wr=%d reg=%d d=%02x",
          c.wr, c.reg, c.data);
}

// THE load-bearing property: a wedged driver spinning on one register must
// not be able to scroll the pre-wedge history out of the ring.
static void test_poll_filter_collapses() {
    printf("test_poll_filter_collapses\n");
    reset_dut();
    clear_ring();
    for (int i = 0; i < 2000; i++) beat(false, 4, 0x01);
    CHECK(wrptr() == 1, "2000 identical reg-4 polls should collapse to 1 entry, got %u",
          wrptr());
    Entry a = decode(ring_read(0));
    CHECK(!a.wr && a.reg == 4 && a.data == 0x01, "collapsed entry wrong");
}

// The other direction: real value changes must all survive, or the trace
// would be missing the transitions that carry all the information.
static void test_value_change_is_recorded() {
    printf("test_value_change_is_recorded\n");
    reset_dut();
    clear_ring();
    // The healthy DATA-IN progression MAME shows: 0x01 -> 0x11 -> 0x91.
    for (int i = 0; i < 5; i++) beat(false, 4, 0x01);
    for (int i = 0; i < 2; i++) beat(false, 4, 0x11);
    beat(false, 4, 0x91);
    CHECK(wrptr() == 3, "expected 3 distinct values recorded, got %u", wrptr());
    CHECK(decode(ring_read(0)).data == 0x01, "v0");
    CHECK(decode(ring_read(1)).data == 0x11, "v1");
    CHECK(decode(ring_read(2)).data == 0x91, "v2");
}

// Writes are commands.  A driver re-issuing the SAME Transfer Info command
// to pull the next 16-byte chunk is the exact signal we are hunting, so
// identical consecutive writes must NEVER be filtered.
static void test_repeated_writes_all_recorded() {
    printf("test_repeated_writes_all_recorded\n");
    reset_dut();
    clear_ring();
    for (int i = 0; i < 32; i++) beat(true, 3, 0x90);
    CHECK(wrptr() == 32, "32 identical W3=0x90 must all be recorded, got %u", wrptr());
}

// A write to a register must invalidate that register's read shadow, or a
// read that happens to match a pre-write value would be wrongly dropped.
static void test_write_invalidates_read_shadow() {
    printf("test_write_invalidates_read_shadow\n");
    reset_dut();
    clear_ring();
    beat(false, 4, 0x55); // recorded, shadow=0x55
    beat(false, 4, 0x55); // dropped
    CHECK(wrptr() == 1, "setup: expected 1, got %u", wrptr());
    beat(true, 4, 0x00);  // write to reg 4 -> invalidate shadow
    beat(false, 4, 0x55); // must be recorded again
    CHECK(wrptr() == 3, "read after write must not be filtered, wr_ptr=%u", wrptr());
    CHECK(!decode(ring_read(2)).wr && decode(ring_read(2)).data == 0x55, "entry2");
}

// The pseudo-DMA port must not alias a register's shadow: DMA payload
// bytes repeat constantly and would poison reg 0's filter if they shared
// an index.
static void test_dma_shim_has_its_own_shadow() {
    printf("test_dma_shim_has_its_own_shadow\n");
    reset_dut();
    clear_ring();
    beat(false, 0, 0xAA);           // reg 0
    beat(false, 0, 0xAA, true);     // DMA shim, same value -> separate shadow
    CHECK(wrptr() == 2, "DMA shim must not alias reg 0, wr_ptr=%u", wrptr());
    CHECK(decode(ring_read(1)).dma, "entry1 should be flagged dma");
}

static void test_freeze_and_clear() {
    printf("test_freeze_and_clear\n");
    reset_dut();
    clear_ring();
    beat(true, 3, 0x41);
    CHECK(wrptr() == 1, "pre-freeze");
    dut->rd_freeze = 1;
    for (int i = 0; i < 8; i++) tick();
    beat(true, 3, 0x42);
    beat(true, 3, 0x43);
    CHECK(wrptr() == 1, "frozen ring must not advance, wr_ptr=%u", wrptr());
    tick();
    CHECK(dut->rd_frozen == 1, "rd_frozen should read 1 while frozen");
    clear_ring();
    CHECK(wrptr() == 0, "clear must rewind wr_ptr, got %u", wrptr());
    tick();
    CHECK(dut->rd_frozen == 0, "clear must re-arm");
    beat(true, 3, 0x44);
    CHECK(wrptr() == 1, "capture must resume after clear, wr_ptr=%u", wrptr());
}

static void test_wrap_sets_flag() {
    printf("test_wrap_sets_flag\n");
    reset_dut();
    clear_ring();
    // 4096 distinct writes -> exactly one full lap.
    for (int i = 0; i < 4096; i++) beat(true, 3, (uint8_t)(i & 0xff));
    tick();
    CHECK(dut->rd_wrapped == 1, "wrapped flag should be set after a full lap");
    CHECK(wrptr() == 0, "wr_ptr should be back at 0, got %u", wrptr());
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--positive-control")) positive_control = true;

    dut = new Vscsi_trace_ring;

    if (positive_control)
        printf("=== POSITIVE CONTROL: pb_ack held low, capture assertions MUST fail ===\n");

    test_basic_capture();
    test_poll_filter_collapses();
    test_value_change_is_recorded();
    test_repeated_writes_all_recorded();
    test_write_invalidates_read_shadow();
    test_dma_shim_has_its_own_shadow();
    test_freeze_and_clear();
    test_wrap_sets_flag();

    delete dut;

    // Exit-code convention matches tb-vhdd-ctrl: make aborts on non-zero, so
    // the positive-control run must exit 0 when the assertions correctly
    // FAILED, and non-zero when they did not fail (i.e. the suite is inert).
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
    printf("all scsi_trace_ring tests passed\n");
    return 0;
}
