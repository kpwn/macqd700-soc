// tb_scsi_c96_register.cpp — register-level unit tb for the C96 (53C96)
// front-end of `rtl/mac/scsi.v` (TURBOSCSI_C96=1).
//
// Exercises each MAME-faithful CSR's read/write semantics against the
// NCR 53C90/53C90A/53C94/53C96 register definitions in MAME's
// `src/devices/machine/ncr53c90.{cpp,h}` (mame0287).
//
// Build via:   make tb-scsi-c96-register
//
// Register layout (offset within the chip — DAFB strides 16 bytes apart
// in CPU space; peripheral_bus.v collapses to pb_addr[3:0] = addr[7:4]):
//   off  read                        write
//    0   tcounter[7:0]               tcount[7:0]
//    1   tcounter[15:8]              tcount[15:8]
//    2   FIFO pop (side-effect)      FIFO push (side-effect)
//    3   command (echo command[0])   command (queued, runs)
//    4   status                      bus_id (target id [2:0])
//    5   istatus (clears IRQ)        select_timeout
//    6   seq_step                    sync_period
//    7   fifo_flags (= fifo_pos)     sync_offset
//    8   config1                     config1
//    9   —                            clock_conv (write-only)
//   0a   —                            test (write-only)
//   0b   config2                     config2
//   0c   config3                     config3
//   0f   —                            fifo_align (write-only)
//
// Every test groups around exactly one of:
//   • register read returns the expected value after a known write
//   • write side-effect on a register fires the expected behavior
//   • read side-effect (FIFO pop, istatus clear) lands as expected
//
// The DUT carries TWO front-ends (NCR 5380 and NCR 53C96).  This tb only
// exercises the C96 path (Verilator -GTURBOSCSI_C96=1).  The 5380 path is
// covered by `tb_scsi.cpp` (35 scenarios, kept green).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"

static Vtb_scsi_vhdd_sd*   dut    = nullptr;
static int      n_pass = 0;
static int      n_fail = 0;

static void tick() {
    dut->clk = 1;
    dut->eval();
    dut->clk = 0;
    dut->eval();
}

static void reset() {
    dut->rst         = 1;
    // Runtime disk size (from the SD CSD at boot); without it the target
    // reports a zero-sector disk and every read fails.
    dut->disk_num_lbas = 1048576u;
    dut->pb_addr     = 0;
    dut->pb_wdata    = 0;
    dut->pb_wr       = 0;
    dut->pb_rd       = 0;
    dut->sd_busy     = 0;
    dut->sd_done     = 0;
    dut->sd_error    = 0;
    dut->sd_rd_valid = 0;
    dut->sd_rd_data  = 0;
    dut->sd_wr_ready = 0;
    for (int i = 0; i < 4; ++i) tick();
    dut->rst = 0;
    tick();
}

static uint8_t reg_r(uint8_t off) {
    dut->pb_addr = off & 0xf;          // C96 registers occupy [0xf:0]
    dut->pb_rd   = 1;
    dut->pb_wr   = 0;
    tick();
    dut->pb_rd   = 0;
    dut->eval();
    return dut->pb_rdata & 0xff;
}

static void reg_w(uint8_t off, uint8_t v) {
    dut->pb_addr  = off & 0xf;
    dut->pb_wdata = v;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    tick();
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
}

#define CHECK_EQ(name, got, exp) do {                                     \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp);         \
    if (_g != _e) {                                                       \
        std::printf("  FAIL %s: got 0x%02x, expected 0x%02x\n",           \
                    name, _g, _e);                                        \
        return false;                                                     \
    }                                                                     \
} while (0)

#define CHECK_TRUE(name, cond) do {                                       \
    if (!(cond)) { std::printf("  FAIL %s\n", name); return false; }      \
} while (0)

#define RUN(test_fn) do {                                                 \
    std::printf("[RUN ] %s\n", #test_fn);                                 \
    bool _ok = test_fn();                                                 \
    if (_ok) { ++n_pass; std::printf("[PASS] %s\n", #test_fn); }          \
    else     { ++n_fail; std::printf("[FAIL] %s\n", #test_fn); }          \
} while (0)

// 53C9x command opcodes (subset; full list in ncr53c90.h)
static constexpr uint8_t CM_NOP             = 0x00;
static constexpr uint8_t CM_FLUSH_FIFO      = 0x01;
static constexpr uint8_t CM_RESET           = 0x02;
static constexpr uint8_t CM_RESET_BUS       = 0x03;
static constexpr uint8_t CD_SELECT          = 0x41;
static constexpr uint8_t CD_SELECT_ATN      = 0x42;
static constexpr uint8_t CD_SELECT_ATN_STOP = 0x43;
static constexpr uint8_t CI_COMPLETE        = 0x11;
static constexpr uint8_t CI_MSG_ACCEPT      = 0x12;

// status (off 4) bits
static constexpr uint8_t S_GROSS  = 0x40;
static constexpr uint8_t S_PARITY = 0x20;
static constexpr uint8_t S_TC0    = 0x10;
static constexpr uint8_t S_TCC    = 0x08;
static constexpr uint8_t S_INTR   = 0x80;
// istatus (off 5) bits
static constexpr uint8_t I_SCSI_RESET = 0x80;
static constexpr uint8_t I_ILLEGAL    = 0x40;
static constexpr uint8_t I_DISCONNECT = 0x20;
static constexpr uint8_t I_BUS        = 0x10;
static constexpr uint8_t I_FUNCTION   = 0x08;

// ─────────────────────────────────────────────────────────────────────
// 1. Reset clears all read-side registers to zero.
// ─────────────────────────────────────────────────────────────────────
static bool test_reset_zeroes_read_regs() {
    reset();
    CHECK_EQ("tcounter_lo at reset",   reg_r(0x0), 0x00);
    CHECK_EQ("tcounter_hi at reset",   reg_r(0x1), 0x00);
    CHECK_EQ("fifo_top at reset",      reg_r(0x2), 0x00);
    CHECK_EQ("command at reset",       reg_r(0x3), 0x00);
    CHECK_EQ("status at reset",        reg_r(0x4), 0x00);
    CHECK_EQ("istatus at reset",       reg_r(0x5), 0x00);
    CHECK_EQ("seq_step at reset",      reg_r(0x6), 0x00);
    CHECK_EQ("fifo_flags at reset",    reg_r(0x7), 0x00);
    CHECK_EQ("config1 at reset",       reg_r(0x8), 0x00);
    CHECK_EQ("config2 at reset",       reg_r(0xb), 0x00);
    CHECK_EQ("config3 at reset",       reg_r(0xc), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 2. tcount lo+hi write loads, but tcounter (read) does not change
//    until a DMA-bit command write reloads it.
// ─────────────────────────────────────────────────────────────────────
static bool test_tcount_loads_on_dma_command() {
    reset();
    reg_w(0x0, 0x34);   // tcount lo
    reg_w(0x1, 0x12);   // tcount hi
    CHECK_EQ("tcounter unchanged before DMA cmd lo", reg_r(0x0), 0x00);
    CHECK_EQ("tcounter unchanged before DMA cmd hi", reg_r(0x1), 0x00);
    reg_w(0x3, 0x80 | CM_NOP);  // DMA bit set: latches tcounter
    CHECK_EQ("tcounter lo after DMA cmd", reg_r(0x0), 0x34);
    CHECK_EQ("tcounter hi after DMA cmd", reg_r(0x1), 0x12);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 3. Non-DMA commands clear tcounter to 0 (per MAME).
// ─────────────────────────────────────────────────────────────────────
static bool test_non_dma_command_clears_tcounter() {
    reset();
    reg_w(0x0, 0x55);
    reg_w(0x1, 0xAA);
    reg_w(0x3, 0x80 | CM_NOP);  // DMA NOP: load tcounter
    CHECK_EQ("tcounter loaded by DMA NOP lo", reg_r(0x0), 0x55);
    CHECK_EQ("tcounter loaded by DMA NOP hi", reg_r(0x1), 0xAA);
    reg_w(0x3, CM_NOP);         // non-DMA NOP: clears tcounter
    CHECK_EQ("tcounter cleared by non-DMA NOP lo", reg_r(0x0), 0x00);
    CHECK_EQ("tcounter cleared by non-DMA NOP hi", reg_r(0x1), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 4. FIFO push via reg-2 write and pop via reg-2 read (FIFO behavior).
// ─────────────────────────────────────────────────────────────────────
static bool test_fifo_push_pop_order() {
    reset();
    reg_w(0x2, 0xDE);
    reg_w(0x2, 0xAD);
    reg_w(0x2, 0xBE);
    reg_w(0x2, 0xEF);
    CHECK_EQ("fifo_flags = 4 after 4 pushes", reg_r(0x7), 0x04);
    CHECK_EQ("fifo pop 1 = 0xDE", reg_r(0x2), 0xDE);
    CHECK_EQ("fifo pop 2 = 0xAD", reg_r(0x2), 0xAD);
    CHECK_EQ("fifo_flags = 2 after 2 pops", reg_r(0x7), 0x02);
    CHECK_EQ("fifo pop 3 = 0xBE", reg_r(0x2), 0xBE);
    CHECK_EQ("fifo pop 4 = 0xEF", reg_r(0x2), 0xEF);
    CHECK_EQ("fifo_flags = 0 after drain", reg_r(0x7), 0x00);
    CHECK_EQ("fifo pop empty returns 0", reg_r(0x2), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 5. FIFO saturates at 16 bytes — extra pushes are discarded.
// ─────────────────────────────────────────────────────────────────────
static bool test_fifo_saturates_at_16() {
    reset();
    for (int i = 0; i < 20; ++i) reg_w(0x2, (uint8_t)i);
    CHECK_EQ("fifo_flags = 16 (saturated)", reg_r(0x7), 0x10);
    // Drain: should see bytes 0..15 in order, since pushes 16..19 dropped.
    for (int i = 0; i < 16; ++i) {
        char name[40];
        std::snprintf(name, sizeof(name), "fifo pop[%d]=%d", i, i);
        CHECK_EQ(name, reg_r(0x2), (uint8_t)i);
    }
    CHECK_EQ("fifo_flags = 0 after full drain", reg_r(0x7), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 6. CM_FLUSH_FIFO drops fifo_pos to 0 without reading bytes.
// ─────────────────────────────────────────────────────────────────────
static bool test_cm_flush_fifo() {
    reset();
    reg_w(0x2, 0xAA);
    reg_w(0x2, 0xBB);
    reg_w(0x2, 0xCC);
    CHECK_EQ("fifo has 3 bytes", reg_r(0x7), 0x03);
    reg_w(0x3, CM_FLUSH_FIFO);
    CHECK_EQ("fifo flushed", reg_r(0x7), 0x00);
    CHECK_EQ("fifo pop after flush returns 0", reg_r(0x2), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 7. CM_RESET clears tcount/tcounter/istatus/IRQ/seq_step/fifo, masks
//    config1 to bottom 3 bits, zeroes config2/config3.
// ─────────────────────────────────────────────────────────────────────
static bool test_cm_reset_chip() {
    reset();
    // Pre-load some state.
    reg_w(0x0, 0xFF);
    reg_w(0x1, 0xFF);
    reg_w(0x3, 0x80 | CM_NOP);   // load tcounter
    reg_w(0x2, 0x42);             // 1 byte in fifo
    // config1: keep [6] (interrupt-disable on bus-reset) clear so the
    // CM_RESET_BUS below actually fires the IRQ we then check.  Use
    // 0xB7 = b'1011_0111' instead.
    reg_w(0x8, 0xB7);
    reg_w(0xb, 0x55);             // config2
    reg_w(0xc, 0xAA);             // config3
    // Cause IRQ via bus reset.
    reg_w(0x3, CM_RESET_BUS);
    CHECK_TRUE("INTR set before chip reset", (reg_r(0x4) & S_INTR) != 0);
    // Now CM_RESET.
    reg_w(0x3, CM_RESET);
    CHECK_EQ("status cleared by CM_RESET", reg_r(0x4), 0x00);
    CHECK_EQ("istatus cleared by CM_RESET", reg_r(0x5), 0x00);
    CHECK_EQ("seq_step cleared by CM_RESET", reg_r(0x6), 0x00);
    CHECK_EQ("fifo flushed by CM_RESET", reg_r(0x7), 0x00);
    CHECK_EQ("tcounter lo cleared", reg_r(0x0), 0x00);
    CHECK_EQ("tcounter hi cleared", reg_r(0x1), 0x00);
    CHECK_EQ("config1 masked to b'0000_0111'", reg_r(0x8), 0x07);
    CHECK_EQ("config2 zeroed", reg_r(0xb), 0x00);
    CHECK_EQ("config3 zeroed", reg_r(0xc), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 8. CM_RESET_BUS asserts I_SCSI_RESET / IRQ unless config1[6] set.
// ─────────────────────────────────────────────────────────────────────
static bool test_cm_reset_bus_irq_gated_by_config1_bit6() {
    reset();
    // Default config1[6]=0 → IRQ asserted.
    reg_w(0x3, CM_RESET_BUS);
    CHECK_EQ("istatus = SCSI_RESET", reg_r(0x5), I_SCSI_RESET);
    CHECK_EQ("status INTR cleared after istatus read", reg_r(0x4) & S_INTR, 0x00);
    // Now set config1[6] (interrupt-disable on bus-reset) and retry.
    reg_w(0x8, 0x40);
    reg_w(0x3, CM_RESET_BUS);
    CHECK_EQ("istatus stays 0 with config1[6] set", reg_r(0x5), 0x00);
    CHECK_EQ("status INTR stays 0 with config1[6] set", reg_r(0x4) & S_INTR, 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 9. Command echo: reg-3 read returns the most-recently-written cmd byte.
// ─────────────────────────────────────────────────────────────────────
static bool test_command_echo() {
    reset();
    reg_w(0x3, CM_NOP);
    CHECK_EQ("command echo CM_NOP", reg_r(0x3), CM_NOP);
    reg_w(0x3, CM_FLUSH_FIFO);
    CHECK_EQ("command echo CM_FLUSH_FIFO", reg_r(0x3), CM_FLUSH_FIFO);
    // DMA-bit set is preserved in the echo.
    reg_w(0x3, 0x80 | CM_NOP);
    CHECK_EQ("command echo DMA NOP keeps bit7", reg_r(0x3), 0x80 | CM_NOP);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 10. bus_id write keeps only [2:0]; over-range bits are masked.
// ─────────────────────────────────────────────────────────────────────
static bool test_bus_id_masks_to_3_bits() {
    reset();
    reg_w(0x4, 0xF6);             // 0xF6 & 7 = 6 (Q700 HDD ID)
    // bus_id is observed indirectly: a CD_SELECT(bus_id=3) shortcut path
    // takes a different branch than other IDs.  Assert by selecting
    // bus_id=6 with timeout: we expect timeout-driven I_DISCONNECT.
    reg_w(0x5, 0x20);             // small select_timeout
    reg_w(0x3, CD_SELECT);
    // Poll seq_step to drive the timeout shim (each off-6 read advances
    // the poll counter).
    for (int i = 0; i < 32; ++i) {
        if (reg_r(0x4) & S_INTR) break;
        (void)reg_r(0x6);
    }
    CHECK_EQ("absent-target select fires DISCONNECT istatus",
             reg_r(0x5), I_DISCONNECT);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 11. select_timeout (off 5 write) is preserved across non-DMA cmds.
//     We test this indirectly by varying timeouts and counting polls.
// ─────────────────────────────────────────────────────────────────────
static bool test_select_timeout_governs_disconnect_polls() {
    reset();
    reg_w(0x4, 0x05);       // bus_id = 5 (no-such-target)
    reg_w(0x5, 0x1B);       // timeout = 27 → limit = (27-13)>>1 = 7
    reg_w(0x3, CD_SELECT);
    int polls_before_irq = 0;
    for (int i = 0; i < 32; ++i) {
        if (reg_r(0x4) & S_INTR) break;
        (void)reg_r(0x6);
        ++polls_before_irq;
    }
    CHECK_EQ("disconnect after expected polls", polls_before_irq, 7);
    CHECK_EQ("istatus = DISCONNECT", reg_r(0x5), I_DISCONNECT);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 12. istatus self-clear on read: reading istatus while IRQ pending
//     drops istatus to 0, status[7] (INTR) to 0, and clears sticky
//     status bits (GROSS / PARITY / TCC) per MAME.
// ─────────────────────────────────────────────────────────────────────
static bool test_istatus_clears_irq_on_read() {
    reset();
    reg_w(0x3, CM_RESET_BUS);
    CHECK_EQ("INTR set", reg_r(0x4) & S_INTR, S_INTR);
    CHECK_EQ("istatus = SCSI_RESET", reg_r(0x5), I_SCSI_RESET);
    // Second istatus read returns 0 (already cleared).
    CHECK_EQ("istatus 0 after first read", reg_r(0x5), 0x00);
    CHECK_EQ("INTR cleared after read", reg_r(0x4) & S_INTR, 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 13. CD_SELECT with bus_id=3 is an ABSENT TARGET, exactly like every
//     other non-live ID.
//
//     INVERTED 2026-08-07.  This test used to assert the opposite: that
//     ID 3 took a "probe-test compat" shortcut in which the select
//     timeout was explicitly DISARMED and a status read walked seq_step
//     0→1, mimicking a CD-ROM answering at the traditional Apple CD ID.
//     That shim is gone.  It was not merely dead weight — with the
//     timeout disarmed and `phase` never leaving S_BUS_FREE, ID 3
//     answered neither "present" (status[2:0] stayed 000, so the ROM's
//     boot-scan gate at 0x40898e12 could never be satisfied) nor
//     "absent" (no I_DISCONNECT could ever fire).  SCSI Manager 4.3
//     issues CD_SELECT to every ID and waits for the disconnect IRQ, so
//     the 7.5.3 bus scan parked on ID 3 forever.  This machine has
//     exactly two SCSI devices, the SD-backed HDD and the DDR RAM disk;
//     there is no CD-ROM, so ID 3 must time out.
// ─────────────────────────────────────────────────────────────────────
static bool test_select_bus_id_3_is_an_absent_target() {
    reset();
    reg_w(0x4, 0x03);
    reg_w(0x5, 0xA7);
    reg_w(0x3, CD_SELECT);
    CHECK_EQ("seq_step starts at 0", reg_r(0x6), 0x00);
    // No phase change: status[2:0] derives from `phase`, which stays
    // S_BUS_FREE because nothing answered.
    uint8_t s = reg_r(0x4);
    CHECK_EQ("status phase bits stay BUS_FREE", s & 0x07, 0x00);
    // A status (offset 4) read must NOT walk seq_step for ID 3 any more.
    // Note reg_r(0x6) above already consumed one poll of the poll-counted
    // shim, and with select_timeout=0xA7 the limit is (0xA7-13)>>1 = 77,
    // so the shim is nowhere near firing yet.
    CHECK_EQ("status read does not advance seq_step", reg_r(0x6), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Selection-timeout chip timer (2026-08-07).
//
// The 53C96 raises I_DISCONNECT after a time-based interval when a
// select finds no responder.  SCSI Manager 4.3 (System 7.5.3) never
// polls seq_step while a select is outstanding — it parks the request
// and waits for the IRQ — so a poll-counted shim alone hangs its bus
// scan forever.  MAME's arbitration chain (ncr53c90.cpp) is:
//     arbitrate()       delay(11)                 :1076
//     ARB_COMPLETE      delay(6)                  :349
//     ARB_ASSERT_SEL    delay_cycles(4)           :359
//     ARB_SET_DEST      delay(2)                  :368
//     ARB_RELEASE_BUSY  delay(8192*select_timeout):385
//     ARB_TIMEOUT_BUSY  delay(1000)               :413
//     ARB_TIMEOUT_ABORT → istatus |= I_DISCONNECT :422-431
// with delay(n) == n * (clock_conv ? clock_conv : 8) chip clocks and
// delay_cycles(n) == n chip clocks, so
//     T_chip = clock_conv * (1019 + 8192*select_timeout) + 4
// The Q700 clocks its 53C96 at 50 MHz/2 = 25 MHz (MAME
// macquadra700.cpp:770) and our peripheral bus runs at 50 MHz, hence
// C96_CLK_DIV = 2 pb_clk cycles per chip clock.
//
// At the SM4.3 programming (clock_conv=5 for a 25 MHz part,
// select_timeout=0): T_chip = 5*1019+4 = 5099 → 5099 * 40 ns = 204.0 us,
// which is the ~203 us measured on MAME's macqd700 bus scan.
// ─────────────────────────────────────────────────────────────────────

// Chip clocks MAME would take for the given programming.
static uint32_t mame_timeout_chip_clocks(uint8_t clock_conv,
                                         uint8_t select_timeout) {
    uint32_t cc = clock_conv ? clock_conv : 8u;
    return cc * (1019u + 8192u * (uint32_t)select_timeout) + 4u;
}

// Arms a select of `bus_id` and free-runs the clock — with ZERO register
// accesses of any kind — until `irq` rises.  Returns the pb_clk cycle
// count from the command write to the IRQ, or 0 if `limit` expired.
// Observing the raw irq line (not a status poll) is deliberate: any
// register read would let the poll-counted shim, or a read side-effect,
// take the credit for the disconnect.
static uint32_t run_select_timeout(uint8_t bus_id, uint8_t clock_conv,
                                   uint8_t select_timeout, uint32_t limit) {
    reset();
    reg_w(0x9, clock_conv);       // clock_w
    reg_w(0x4, bus_id);
    reg_w(0x5, select_timeout);
    reg_w(0x3, CD_SELECT);        // arms the timer on this posedge
    for (uint32_t c = 1; c <= limit; ++c) {
        tick();
        if (dut->irq) return c;
    }
    return 0;
}

// ─────────────────────────────────────────────────────────────────────
// 21. A select of an absent ID raises I_DISCONNECT + IRQ after the chip
//     timer, with ZERO seq_step reads.  Runs for ID 3 (the ex-CD-ROM
//     shim) and ID 5 (an ID that never had a shim — the positive
//     control that the mechanism is generic, not an ID-3 patch).
// ─────────────────────────────────────────────────────────────────────
static bool test_absent_select_timer_fires_without_polls() {
    const uint8_t ids[] = { 3, 5 };
    for (uint8_t id : ids) {
        // clock_conv=5, select_timeout=0 → the SM4.3 programming.
        const uint32_t exp_pb = 2u * mame_timeout_chip_clocks(5, 0);
        uint32_t got = run_select_timeout(id, 5, 0, 4u * exp_pb);
        std::printf("    id=%u irq at pb cycle %u (expected ~%u)\n",
                    id, got, exp_pb);
        CHECK_TRUE("timer fired at all", got != 0);
        // +-4 pb cycles covers the prescaler phase and the one-cycle
        // arm/observe latency.  A loose bound here would not distinguish
        // "a real timer" from "some other watchdog".
        CHECK_TRUE("timer fired at the MAME-derived time",
                   got >= exp_pb - 4 && got <= exp_pb + 4);
        // The IRQ carries I_DISCONNECT, and reading istatus clears it.
        CHECK_EQ("istatus = DISCONNECT", reg_r(0x5), I_DISCONNECT);
        CHECK_EQ("INTR cleared after istatus read", reg_r(0x4) & S_INTR, 0);
        // seq_step was never read, and stays 0.
        CHECK_EQ("seq_step untouched", reg_r(0x6), 0x00);
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 22. The interval really is derived from select_timeout and clock_conv
//     — it is not a fixed bail-out.  Three programmings, three distinct
//     MAME-predicted intervals.
// ─────────────────────────────────────────────────────────────────────
static bool test_select_timer_tracks_registers() {
    struct { uint8_t cc, st; } cases[] = {
        { 5, 0 },   // SM4.3 programming     →  5099 chip clocks
        { 2, 0 },   // post-reset clock_conv →  2042 chip clocks
        { 5, 1 },   // one 8192-clock unit   → 46059 chip clocks
    };
    for (auto& c : cases) {
        const uint32_t exp_pb = 2u * mame_timeout_chip_clocks(c.cc, c.st);
        uint32_t got = run_select_timeout(5, c.cc, c.st, 4u * exp_pb);
        std::printf("    clock_conv=%u select_timeout=%u -> %u pb cycles"
                    " (expected ~%u)\n", c.cc, c.st, got, exp_pb);
        CHECK_TRUE("timer fired", got != 0);
        CHECK_TRUE("interval matches MAME's formula",
                   got >= exp_pb - 4 && got <= exp_pb + 4);
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 23. NEGATIVE CONTROL for 21/22: a select of the LIVE target (ID 0,
//     dev_en[0]=1 in this tb build) must never raise I_DISCONNECT, no
//     matter how long we wait.  Without this, tests 21/22 would still
//     pass if the timer armed unconditionally and the "two live
//     devices" contract were broken.
// ─────────────────────────────────────────────────────────────────────
static bool test_live_target_select_never_times_out() {
    reset();
    reg_w(0x9, 0x05);
    reg_w(0x4, 0x00);             // TARGET_ID = 0, live in this build
    reg_w(0x5, 0x00);
    reg_w(0x3, CD_SELECT);
    const uint32_t window = 4u * 2u * mame_timeout_chip_clocks(5, 0);
    for (uint32_t c = 0; c < window; ++c) tick();
    // A live selection may legitimately raise its own completion IRQ
    // (I_FUNCTION|I_BUS via the sel_pending hook); what it must NEVER
    // do is report DISCONNECT.
    uint8_t ist = reg_r(0x5);
    std::printf("    live-target istatus after %u cycles = 0x%02x\n",
                window, ist);
    CHECK_TRUE("live target did not disconnect",
               (ist & I_DISCONNECT) == 0);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 24. The poll-counted shim still works and still wins when the driver
//     DOES poll — the ROM and 7.0.1 SCSI Manager depend on it.  With
//     clock_conv=5/select_timeout=0x1B the chip timer would need
//     2*(5*(1019+8192*27)+4) = 2,222,050 pb cycles; the shim gets there
//     in 7 polls.  This is the regression guard on "additive, not a
//     replacement".
// ─────────────────────────────────────────────────────────────────────
static bool test_poll_shim_still_beats_the_timer() {
    reset();
    reg_w(0x9, 0x05);
    reg_w(0x4, 0x05);
    reg_w(0x5, 0x1B);             // limit = (27-13)>>1 = 7 polls
    reg_w(0x3, CD_SELECT);
    int polls = 0;
    for (int i = 0; i < 32; ++i) {
        if (reg_r(0x4) & S_INTR) break;
        (void)reg_r(0x6);
        ++polls;
    }
    CHECK_EQ("disconnect after expected polls", polls, 7);
    CHECK_EQ("istatus = DISCONNECT", reg_r(0x5), I_DISCONNECT);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 25. CM_RESET and CM_RESET_BUS disarm the chip timer.
// ─────────────────────────────────────────────────────────────────────
static bool test_chip_and_bus_reset_disarm_the_timer() {
    const uint8_t cancels[] = { CM_RESET, CM_RESET_BUS };
    for (uint8_t cancel : cancels) {
        reset();
        reg_w(0x9, 0x05);
        reg_w(0x4, 0x03);
        reg_w(0x5, 0x00);
        reg_w(0x3, CD_SELECT);
        for (int i = 0; i < 64; ++i) tick();   // partway into the interval
        reg_w(0x3, cancel);
        (void)reg_r(0x5);                      // drain any reset istatus
        const uint32_t window = 4u * 2u * mame_timeout_chip_clocks(5, 0);
        for (uint32_t c = 0; c < window; ++c) {
            tick();
            CHECK_TRUE("cancelled timer stayed quiet", !dut->irq);
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 14. CI_MSG_ACCEPT fires I_DISCONNECT.
// ─────────────────────────────────────────────────────────────────────
static bool test_ci_msg_accept_disconnect() {
    // Initiator-group commands are only valid while connected as an
    // initiator (MAME ncr53c90a check_valid_command, ncr53c90.cpp:1300;
    // scsi_fuzz finding F3): at bus-free a CI_MSG_ACCEPT is ILLEGAL.
    reset();
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("istatus = ILLEGAL at bus-free", reg_r(0x5), 0x40);
    CHECK_EQ("INTR cleared after istatus read", reg_r(0x4) & S_INTR, 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 15. CI_COMPLETE fires I_FUNCTION + stuffs status/message into FIFO.
// ─────────────────────────────────────────────────────────────────────
static bool test_ci_complete_fills_fifo() {
    // Same validity contract as above: CI_COMPLETE at bus-free is
    // ILLEGAL and moves nothing into the FIFO.  The connected-mode
    // CI_COMPLETE behavior (status+msg pushed, I_FUNCTION) is covered
    // end-to-end by the tb_scsi_c96_read6 scenarios.
    reset();
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("fifo untouched at bus-free", reg_r(0x7), 0x00);
    CHECK_EQ("istatus = ILLEGAL at bus-free", reg_r(0x5), 0x40);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 16. config1/config2/config3 round-trip read/write (no masks beyond reset).
// ─────────────────────────────────────────────────────────────────────
static bool test_config_round_trip() {
    reset();
    reg_w(0x8, 0xC4);
    CHECK_EQ("config1 round-trip", reg_r(0x8), 0xC4);
    reg_w(0xb, 0x12);
    CHECK_EQ("config2 round-trip", reg_r(0xb), 0x12);
    reg_w(0xc, 0x07);
    CHECK_EQ("config3 round-trip", reg_r(0xc), 0x07);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 17. sync_period (off 6 write) and sync_offset (off 7 write): writable
//     side does not interfere with the read side (which is seq_step /
//     fifo_flags respectively).
// ─────────────────────────────────────────────────────────────────────
static bool test_sync_writes_do_not_clobber_read_view() {
    reset();
    reg_w(0x6, 0xFF);   // sync_period[4:0] = 0x1f
    reg_w(0x7, 0xFF);   // sync_offset[3:0] = 0x0f
    CHECK_EQ("seq_step still 0 after sync_period write", reg_r(0x6), 0x00);
    CHECK_EQ("fifo_flags still 0 after sync_offset write", reg_r(0x7), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 18. clock_w / test_w / fifo_align_w (off 9, 0xa, 0xf): writes are
//     accepted-and-dropped without observable read-side effect.
// ─────────────────────────────────────────────────────────────────────
static bool test_write_only_offsets_no_state_corruption() {
    reset();
    reg_w(0x9, 0xFF);
    reg_w(0xa, 0xFF);
    reg_w(0xf, 0xFF);
    // Sanity: reading those returns the array's `default 0xff` placeholder.
    // We DO NOT depend on that; only that other registers stayed 0.
    CHECK_EQ("status untouched after w/o offset writes", reg_r(0x4), 0x00);
    CHECK_EQ("fifo untouched", reg_r(0x7), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 19. Unrecognised command fires I_ILLEGAL istatus + IRQ.
// ─────────────────────────────────────────────────────────────────────
static bool test_unknown_command_fires_illegal() {
    reset();
    reg_w(0x3, 0x55);   // not in the supported set
    CHECK_EQ("status INTR set", reg_r(0x4) & S_INTR, S_INTR);
    CHECK_EQ("istatus = ILLEGAL", reg_r(0x5), I_ILLEGAL);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 20. After CM_RESET_BUS (which sets I_SCSI_RESET), a subsequent
//     CM_RESET zeroes EVERYTHING including the still-pending istatus.
// ─────────────────────────────────────────────────────────────────────
static bool test_cm_reset_clears_pending_istatus() {
    reset();
    reg_w(0x3, CM_RESET_BUS);
    CHECK_EQ("istatus pending = SCSI_RESET", reg_r(0x4) & S_INTR, S_INTR);
    reg_w(0x3, CM_RESET);
    CHECK_EQ("status all-zero after CM_RESET", reg_r(0x4), 0x00);
    CHECK_EQ("istatus zero after CM_RESET", reg_r(0x5), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 21. Command-register queue depth vs the RESET special-case ORDER.
//
//     MAME command_w (ncr53c90.cpp:887-903) tests the queue-full
//     condition FIRST and returns:
//
//         if(command_pos == 2) { status |= S_GROSS_ERROR; ...; return; }
//         if((data & 0x7f) == CM_RESET || == CM_RESET_BUS) command_pos = 0;
//         command[command_pos++] = data;
//         if(command_pos == 1) start_command();
//
//     so a RESET / RESET_BUS written into a FULL command register is
//     dropped exactly like any other opcode — it does NOT reset the
//     chip.  With one slot free it still bypasses the queue and runs
//     immediately.  Getting this order wrong let a wedged chip be reset
//     out of a full queue (scsi_fuzz seeds 13/14/22/23 diverged from
//     MAME here: our chip came back pristine while MAME stayed wedged
//     with cmd=..:2).
// ─────────────────────────────────────────────────────────────────────
static bool test_full_command_queue_drops_reset_forms() {
    // 0x55 is an unrecognised opcode: it OCCUPIES a queue slot (raising
    // I_ILLEGAL) without touching the bus, so the queue can be filled
    // without perturbing the back-end.

    // ── positive control: one slot free -> the bypass still works ────
    reset();
    reg_w(0x8, 0x07);                 // config1: own ID 7 (survives reset)
    reg_w(0x3, 0x55);                 // slot 0, dispatched (command_pos 1)
    CHECK_EQ("echo after first command", reg_r(0x3), 0x55);
    reg_w(0x3, CM_RESET);             // command_pos == 1 -> bypass, runs
    CHECK_EQ("chip reset from a half-full queue clears the echo",
             reg_r(0x3), 0x00);
    CHECK_EQ("chip reset from a half-full queue clears status",
             reg_r(0x4), 0x00);
    CHECK_EQ("chip reset from a half-full queue clears istatus",
             reg_r(0x5), 0x00);

    // ── the regression: queue FULL -> reset forms are dropped ────────
    reset();
    reg_w(0x8, 0x07);
    reg_w(0x3, 0x55);                 // slot 0, dispatched
    reg_w(0x3, 0x55);                 // slot 1, queued  (command_pos == 2)
    CHECK_EQ("echo still slot 0 with a full queue", reg_r(0x3), 0x55);

    reg_w(0x3, CM_RESET);             // dropped: S_GROSS_ERROR, no reset
    CHECK_TRUE("CM_RESET into a full queue sets S_GROSS_ERROR",
               (reg_r(0x4) & S_GROSS) != 0);
    CHECK_EQ("CM_RESET into a full queue did not reset the chip",
             reg_r(0x3), 0x55);
    CHECK_EQ("CM_RESET into a full queue did not clear config1",
             reg_r(0x8), 0x07);

    reg_w(0x3, CM_RESET_BUS);         // dropped the same way
    CHECK_TRUE("CM_RESET_BUS into a full queue sets S_GROSS_ERROR",
               (reg_r(0x4) & S_GROSS) != 0);
    CHECK_EQ("CM_RESET_BUS into a full queue did not reset the chip",
             reg_r(0x3), 0x55);
    // A dropped CM_RESET_BUS raises no SCSI-reset interrupt either —
    // the pending istatus is still the slot-0 I_ILLEGAL.
    CHECK_EQ("dropped CM_RESET_BUS leaves istatus at I_ILLEGAL",
             reg_r(0x5), I_ILLEGAL);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Driver
// ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    RUN(test_reset_zeroes_read_regs);
    RUN(test_tcount_loads_on_dma_command);
    RUN(test_non_dma_command_clears_tcounter);
    RUN(test_fifo_push_pop_order);
    RUN(test_fifo_saturates_at_16);
    RUN(test_cm_flush_fifo);
    RUN(test_cm_reset_chip);
    RUN(test_cm_reset_bus_irq_gated_by_config1_bit6);
    RUN(test_command_echo);
    RUN(test_bus_id_masks_to_3_bits);
    RUN(test_select_timeout_governs_disconnect_polls);
    RUN(test_istatus_clears_irq_on_read);
    RUN(test_select_bus_id_3_is_an_absent_target);
    RUN(test_ci_msg_accept_disconnect);
    RUN(test_ci_complete_fills_fifo);
    RUN(test_config_round_trip);
    RUN(test_sync_writes_do_not_clobber_read_view);
    RUN(test_write_only_offsets_no_state_corruption);
    RUN(test_unknown_command_fires_illegal);
    RUN(test_cm_reset_clears_pending_istatus);
    RUN(test_absent_select_timer_fires_without_polls);
    RUN(test_select_timer_tracks_registers);
    RUN(test_live_target_select_never_times_out);
    RUN(test_poll_shim_still_beats_the_timer);
    RUN(test_chip_and_bus_reset_disarm_the_timer);
    RUN(test_full_command_queue_drops_reset_forms);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
