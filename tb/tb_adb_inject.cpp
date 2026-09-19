// tb_adb_inject.cpp — directed unit testbench for the host-side ADB
// event-injection path (adb_inject.v + adb_keyboard.v + adb_mouse.v).
//
// Covers the MMIO producer → device-model consumer contract:
//   - pb_* byte writes (the shape peripheral_bus SLOT_ADBINJ delivers,
//     whether the byte came from the m68k or from a JTAG-AXI word
//     write) land in the keyboard FIFO / mouse accumulator;
//   - a subsequent TALK R0 device-bus poll (the shape adb_phy's
//     dispatcher emits for the PIC firmware) reports the injected
//     event with correct ADB register-0 encoding;
//   - with nothing injected, TALK R0 still terminates with
//     dev_resp_empty (the "always answer polls" behaviour that
//     unblocks PIC firmware boot-time polling) — regression guard.
//
// Both the legacy byte-granular register offsets (0x00..0x05) and the
// word-aligned alias block (0x10..0x1C, reachable from the 32-bit
// JTAG-AXI master) are exercised.
//
// Build: make tb-adb-inject

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vtb_adb_inject.h"

static Vtb_adb_inject* dut = nullptr;
static int n_pass = 0;
static int n_fail = 0;

// Register offsets — mirror rtl/mac/adb_inject.v.
static const uint8_t OFF_KBD_ENQ      = 0x00;
static const uint8_t OFF_KBD_STAT     = 0x01;
static const uint8_t OFF_MOUSE_BTN    = 0x02;
static const uint8_t OFF_MOUSE_DX     = 0x03;
static const uint8_t OFF_MOUSE_DY     = 0x04;
static const uint8_t OFF_MOUSE_STAT   = 0x05;
static const uint8_t OFF_KBD_ENQ_W    = 0x10;
static const uint8_t OFF_MOUSE_BTN_W  = 0x14;
static const uint8_t OFF_MOUSE_DX_W   = 0x18;
static const uint8_t OFF_MOUSE_DY_W   = 0x1C;

// Device-bus ops — mirror adb_keyboard.v / adb_mouse.v.
static const uint8_t OP_TALK_R0 = 4;
static const uint8_t OP_TALK_R3 = 5;
static const uint8_t OP_FLUSH   = 1;

static const uint8_t ADDR_KBD   = 2;
static const uint8_t ADDR_MOUSE = 3;

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
}

static void reset() {
    dut->rst = 1;
    dut->pb_addr = 0; dut->pb_wdata = 0; dut->pb_wr = 0; dut->pb_rd = 0;
    dut->dev_cmd_valid = 0; dut->dev_cmd_addr = 0; dut->dev_cmd_op = 0;
    dut->dev_listen_valid = 0; dut->dev_listen_b0 = 0; dut->dev_listen_b1 = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick(); tick();
}

#define CHECK_EQ(name, got, exp) do { \
    uint64_t _g = (uint64_t)(got); \
    uint64_t _e = (uint64_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%02lx, expected 0x%02lx (line %d)\n", \
               name, (unsigned long)_g, (unsigned long)_e, __LINE__); \
        return false; \
    } \
} while (0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s (line %d)\n", name, __LINE__); \
        return false; \
    } \
} while (0)

// One MMIO byte write, exactly as peripheral_bus delivers it: pb_wr
// pulses for one cycle; adb_inject registers its ack + inj pulse on the
// following edge; the device model latches the event one edge later.
static void pb_write(uint8_t off, uint8_t byte) {
    dut->pb_addr  = off;
    dut->pb_wdata = byte;
    dut->pb_wr    = 1;
    tick();
    dut->pb_wr = 0;
    // inj pulse cycle + device latch cycle + settle.
    tick(); tick(); tick();
}

// One MMIO byte read; returns pb_rdata sampled on the ack cycle.
static uint8_t pb_read(uint8_t off) {
    dut->pb_addr = off;
    dut->pb_rd   = 1;
    tick();
    dut->pb_rd = 0;
    // Registered ack/rdata arrive after this edge.
    dut->eval();
    uint8_t v = dut->pb_rdata;
    if (!dut->pb_ack) { tick(); v = dut->pb_rdata; }
    tick();
    return v;
}

// One device-bus command pulse (as adb_phy's dispatcher emits).
// Captures the addressed device's resp_valid/resp_empty over the next
// few cycles.  Fills *valid/*empty/*b0/*b1 for the kbd or mouse face.
struct TalkResult {
    bool valid = false;
    bool empty = false;
    uint8_t b0 = 0, b1 = 0;
};

static TalkResult talk(uint8_t addr, uint8_t op) {
    dut->dev_cmd_valid = 1;
    dut->dev_cmd_addr  = addr;
    dut->dev_cmd_op    = op;
    tick();
    dut->dev_cmd_valid = 0;
    TalkResult r;
    for (int i = 0; i < 4; i++) {
        dut->eval();
        bool v = (addr == ADDR_KBD) ? dut->kbd_resp_valid : dut->ms_resp_valid;
        bool e = (addr == ADDR_KBD) ? dut->kbd_resp_empty : dut->ms_resp_empty;
        if (v) {
            r.valid = true;
            r.b0 = (addr == ADDR_KBD) ? dut->kbd_resp_b0 : dut->ms_resp_b0;
            r.b1 = (addr == ADDR_KBD) ? dut->kbd_resp_b1 : dut->ms_resp_b1;
        }
        if (e) r.empty = true;
        tick();
    }
    return r;
}

// ── Scenario 1: idle keyboard TALK R0 answers "empty", not silence ──
// Regression guard for the PIC-firmware-unblocking behaviour: with no
// injected events the device must still terminate the poll (via
// dev_resp_empty, which adb_phy turns into a valid "no data" frame).
static bool test_kbd_idle_talk_empty() {
    reset();
    TalkResult r = talk(ADDR_KBD, OP_TALK_R0);
    CHECK_TRUE("kbd idle TALK R0 reports empty", r.empty);
    CHECK_TRUE("kbd idle TALK R0 has no data", !r.valid);
    CHECK_TRUE("kbd idle: no SRQ", !dut->kbd_srq);
    return true;
}

// ── Scenario 2: idle mouse TALK R0 answers "empty" ──────────────────
static bool test_mouse_idle_talk_empty() {
    reset();
    TalkResult r = talk(ADDR_MOUSE, OP_TALK_R0);
    CHECK_TRUE("mouse idle TALK R0 reports empty", r.empty);
    CHECK_TRUE("mouse idle TALK R0 has no data", !r.valid);
    CHECK_TRUE("mouse idle: no SRQ", !dut->ms_srq);
    return true;
}

// ── Scenario 3: key event via the word-aligned alias register ───────
static bool test_kbd_inject_alias() {
    reset();
    pb_write(OFF_KBD_ENQ_W, 0x1C);          // key-down, code 0x1C
    // Status via the alias read: bit0 = not-empty, bits[6:1] = count.
    uint8_t st = pb_read(OFF_KBD_ENQ_W);
    CHECK_EQ("kbd status after 1 enqueue", st, (1 << 1) | 1);
    CHECK_TRUE("kbd SRQ asserted while event pending", dut->kbd_srq);

    TalkResult r = talk(ADDR_KBD, OP_TALK_R0);
    CHECK_TRUE("kbd TALK R0 valid", r.valid);
    CHECK_EQ("kbd TALK R0 byte0 = keycode", r.b0, 0x1C);
    CHECK_EQ("kbd TALK R0 byte1 = 0xFF pad", r.b1, 0xFF);

    // Drained: next poll is empty again, status reads 0.
    st = pb_read(OFF_KBD_ENQ_W);
    CHECK_EQ("kbd status drained", st, 0x00);
    r = talk(ADDR_KBD, OP_TALK_R0);
    CHECK_TRUE("kbd TALK R0 empty after drain", r.empty);
    return true;
}

// ── Scenario 4: down+up pair reports both events in one TALK ────────
static bool test_kbd_inject_pair_one_report() {
    reset();
    pb_write(OFF_KBD_ENQ_W, 0x1C);          // down
    pb_write(OFF_KBD_ENQ_W, 0x9C);          // up (bit7 = release)
    TalkResult r = talk(ADDR_KBD, OP_TALK_R0);
    CHECK_TRUE("kbd TALK R0 valid", r.valid);
    CHECK_EQ("kbd TALK R0 byte0 = down event", r.b0, 0x1C);
    CHECK_EQ("kbd TALK R0 byte1 = up event", r.b1, 0x9C);
    r = talk(ADDR_KBD, OP_TALK_R0);
    CHECK_TRUE("kbd TALK R0 empty after pair drained", r.empty);
    return true;
}

// ── Scenario 5: legacy byte-granular keyboard offset still works ────
static bool test_kbd_inject_legacy_offset() {
    reset();
    pb_write(OFF_KBD_ENQ, 0x2A);
    uint8_t st = pb_read(OFF_KBD_STAT);
    CHECK_EQ("kbd legacy status after enqueue", st, (1 << 1) | 1);
    TalkResult r = talk(ADDR_KBD, OP_TALK_R0);
    CHECK_TRUE("kbd TALK R0 valid", r.valid);
    CHECK_EQ("kbd TALK R0 byte0", r.b0, 0x2A);
    CHECK_EQ("kbd TALK R0 byte1", r.b1, 0xFF);
    return true;
}

// ── Scenario 6: mouse event via the word-aligned alias block ────────
static bool test_mouse_inject_alias() {
    reset();
    pb_write(OFF_MOUSE_BTN_W, 0x01);         // button down
    pb_write(OFF_MOUSE_DX_W,  0x05);         // dx = +5
    pb_write(OFF_MOUSE_DY_W,  0xFD);         // dy = -3
    uint8_t st = pb_read(OFF_MOUSE_BTN_W);
    CHECK_EQ("mouse status pending", st, 0x01);
    CHECK_TRUE("mouse SRQ asserted while pending", dut->ms_srq);

    TalkResult r = talk(ADDR_MOUSE, OP_TALK_R0);
    CHECK_TRUE("mouse TALK R0 valid", r.valid);
    // byte0: bit7 = ~button (0 = pressed), bits6..0 = dy (7-bit 2's C).
    CHECK_EQ("mouse TALK R0 byte0 = pressed | dy=-3", r.b0, 0x00 | (0x7D));
    // byte1: bit7 = 1 (single-button), bits6..0 = dx.
    CHECK_EQ("mouse TALK R0 byte1 = 0x80 | dx=+5", r.b1, 0x85);

    // Deltas consumed: next poll empty, status clear.
    st = pb_read(OFF_MOUSE_BTN_W);
    CHECK_EQ("mouse status consumed", st, 0x00);
    TalkResult r2 = talk(ADDR_MOUSE, OP_TALK_R0);
    CHECK_TRUE("mouse TALK R0 empty after consume", r2.empty);
    return true;
}

// ── Scenario 7: legacy byte-granular mouse offsets still work ───────
static bool test_mouse_inject_legacy_offsets() {
    reset();
    pb_write(OFF_MOUSE_BTN, 0x01);
    pb_write(OFF_MOUSE_DX,  0x7F);           // +127, saturates to +63
    pb_write(OFF_MOUSE_DY,  0x02);
    uint8_t st = pb_read(OFF_MOUSE_STAT);
    CHECK_EQ("mouse legacy status pending", st, 0x01);
    TalkResult r = talk(ADDR_MOUSE, OP_TALK_R0);
    CHECK_TRUE("mouse TALK R0 valid", r.valid);
    CHECK_EQ("mouse byte0 = pressed | dy=+2", r.b0, 0x02);
    CHECK_EQ("mouse byte1 = 0x80 | dx sat +63", r.b1, 0x80 | 0x3F);
    return true;
}

// ── Scenario 8: button release encodes bit7 = 1 ─────────────────────
static bool test_mouse_button_up() {
    reset();
    pb_write(OFF_MOUSE_BTN_W, 0x00);         // button up event
    TalkResult r = talk(ADDR_MOUSE, OP_TALK_R0);
    CHECK_TRUE("mouse TALK R0 valid", r.valid);
    CHECK_EQ("mouse byte0 = released | dy=0", r.b0, 0x80);
    CHECK_EQ("mouse byte1 = 0x80 | dx=0", r.b1, 0x80);
    return true;
}

// ── Scenario 9: delta accumulation across writes before one poll ────
static bool test_mouse_delta_accumulate() {
    reset();
    pb_write(OFF_MOUSE_DX_W, 0x0A);          // +10
    pb_write(OFF_MOUSE_DX_W, 0x0A);          // +10 more
    pb_write(OFF_MOUSE_DY_W, 0xF6);          // -10
    TalkResult r = talk(ADDR_MOUSE, OP_TALK_R0);
    CHECK_TRUE("mouse TALK R0 valid", r.valid);
    CHECK_EQ("mouse byte0 = released | dy=-10", r.b0, 0x80 | 0x76);
    CHECK_EQ("mouse byte1 = 0x80 | dx=+20", r.b1, 0x80 | 0x14);
    return true;
}

// ── Scenario 10: TALK R3 unaffected by injection (addr/handler) ─────
static bool test_talk_r3_identity() {
    reset();
    pb_write(OFF_KBD_ENQ_W, 0x1C);
    TalkResult rk = talk(ADDR_KBD, OP_TALK_R3);
    CHECK_TRUE("kbd TALK R3 valid", rk.valid);
    CHECK_EQ("kbd TALK R3 byte0 = SRQ|addr=2", rk.b0, 0x80 | 0x02);
    CHECK_EQ("kbd TALK R3 byte1 = handler 1", rk.b1, 0x01);
    TalkResult rm = talk(ADDR_MOUSE, OP_TALK_R3);
    CHECK_TRUE("mouse TALK R3 valid", rm.valid);
    CHECK_EQ("mouse TALK R3 byte0 = SRQ|addr=3", rm.b0, 0x80 | 0x03);
    CHECK_EQ("mouse TALK R3 byte1 = handler 1", rm.b1, 0x01);
    return true;
}

// ── Scenario 11: FLUSH clears injected-but-unread events ────────────
static bool test_flush_clears_pending() {
    reset();
    pb_write(OFF_KBD_ENQ_W, 0x1C);
    pb_write(OFF_MOUSE_DX_W, 0x05);
    (void)talk(ADDR_KBD, OP_FLUSH);
    (void)talk(ADDR_MOUSE, OP_FLUSH);
    TalkResult rk = talk(ADDR_KBD, OP_TALK_R0);
    CHECK_TRUE("kbd empty after FLUSH", rk.empty);
    TalkResult rm = talk(ADDR_MOUSE, OP_TALK_R0);
    CHECK_TRUE("mouse empty after FLUSH", rm.empty);
    return true;
}

#define RUN(t) do { \
    printf("%-40s ", #t); fflush(stdout); \
    if (t()) { printf("PASS\n"); n_pass++; } \
    else     { n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_adb_inject;

    RUN(test_kbd_idle_talk_empty);
    RUN(test_mouse_idle_talk_empty);
    RUN(test_kbd_inject_alias);
    RUN(test_kbd_inject_pair_one_report);
    RUN(test_kbd_inject_legacy_offset);
    RUN(test_mouse_inject_alias);
    RUN(test_mouse_inject_legacy_offsets);
    RUN(test_mouse_button_up);
    RUN(test_mouse_delta_accumulate);
    RUN(test_talk_r3_identity);
    RUN(test_flush_clears_pending);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
