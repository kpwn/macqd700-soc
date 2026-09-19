// tb_adb.cpp — directed unit testbench for adb_modem.v + adb_keyboard.v +
// adb_mouse.v.
//
// Build:   make tb-adb
//
// Scenarios (each returns bool — true on PASS):
//   1. reset_state            — at reset, no IRQ, no SRQ, modem idle.
//   2. talk_kbd_empty         — TALK kbd reg 0 with empty FIFO returns
//                                a single 0xFF.
//   3. talk_kbd_event         — inject 1 keycode, TALK reg 0 returns
//                                {keycode, 0xFF}.
//   4. talk_kbd_two_events    — inject 2 keycodes, TALK reg 0 returns
//                                both in one transaction.
//   5. talk_kbd_register3     — TALK reg 3 returns {SRQ|addr, handler}.
//   6. listen_kbd_set_addr    — LISTEN reg 3 to change addr; subsequent
//                                TALK at the old addr fails, new addr
//                                succeeds.
//   7. flush_kbd              — FLUSH drains the FIFO.
//   8. mouse_event_motion     — inject dx/dy; TALK mouse reg 0 returns
//                                deltas.
//   9. mouse_button           — inject button; TALK reports button bit
//                                cleared (pressed).
//  10. srq_pending            — kbd FIFO non-empty raises adb_irq_pending
//                                while modem is idle.
//  11. unaddressed_talk       — TALK addr 5 yields a single 0xFF, no
//                                hang.
//  12. listen_then_talk       — LISTEN R3 to change handler ID; TALK R3
//                                reads back the new handler.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <verilated.h>
#include "Vtb_adb.h"

static Vtb_adb*  dut       = nullptr;
static uint64_t  sim_time  = 0;
static int       n_pass    = 0;
static int       n_fail    = 0;

static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void reset() {
    dut->rst             = 1;
    dut->phi2_tick       = 0;
    dut->via_rx_ready    = 0;
    dut->via_tx_byte     = 0;
    dut->via_tx_valid    = 0;
    dut->inj_kc_valid    = 0;
    dut->inj_kc_byte     = 0;
    dut->inj_btn_valid   = 0;
    dut->inj_btn_state   = 0;
    dut->inj_dx_valid    = 0;
    dut->inj_dx          = 0;
    dut->inj_dy_valid    = 0;
    dut->inj_dy          = 0;
    tick(); tick(); tick();
    dut->rst = 0;
    tick(); tick();
}

#define CHECK_EQ(name, got, exp) do { \
    uint64_t _g = (uint64_t)(got); \
    uint64_t _e = (uint64_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%08lx, expected 0x%08lx (line %d)\n", \
               name, (unsigned long)_g, (unsigned long)_e, __LINE__); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s (line %d)\n", name, __LINE__); \
        return false; \
    } \
} while(0)

// Send one byte to the modem (host → modem TX path).  Pulses
// via_tx_valid for one cycle; the modem's S_IDLE samples it.  Wait for
// dbg_state == S_IDLE first so we don't race with S_DELAY pacing.
static void send_tx_byte(uint8_t b) {
    int waits = 0;
    while (dut->dbg_state != 0 /* S_IDLE */ && waits++ < 1024) {
        dut->phi2_tick = 1; tick();
        dut->phi2_tick = 0; tick();
    }
    dut->via_tx_byte  = b;
    dut->via_tx_valid = 1;
    tick();
    dut->via_tx_valid = 0;
    dut->via_tx_byte  = 0;
    tick();
}

// Wait up to N (clk + phi2) cycles for via_rx_valid; once seen, capture
// the byte, pulse via_rx_ready, and return the byte.  Returns -1 on
// timeout.
static int wait_rx_byte(int max_phi2 = 64) {
    for (int i = 0; i < max_phi2; i++) {
        // Check before each phi2 burst.
        for (int s = 0; s < 8; s++) {
            if (dut->via_rx_valid) {
                int b = dut->via_rx_byte;
                // Acknowledge: pulse via_rx_ready for one tick under
                // phi2 (the modem samples the ready signal directly,
                // not phi2-gated, since the FSM ack is in the always
                // block on every clk).  Hold ready high for one full
                // clk to make sure the modem's edge-sample sees it.
                dut->via_rx_ready = 1;
                tick();
                dut->via_rx_ready = 0;
                tick();
                return b;
            }
            tick();
        }
        // Burst one phi2 pulse to allow modem state machine to advance
        // (DELAY + S_IDLE entry pacing).
        dut->phi2_tick = 1; tick();
        dut->phi2_tick = 0; tick();
    }
    return -1;
}

// ─── Test scenarios ───────────────────────────────────────────────────

static bool test_reset_state() {
    printf("[RUN] test_reset_state\n");
    reset();
    CHECK_EQ("via_rx_valid",    dut->via_rx_valid, 0);
    CHECK_EQ("adb_irq_pending", dut->adb_irq_pending, 0);
    CHECK_EQ("dbg_state",       dut->dbg_state, 0);  // S_IDLE
    return true;
}

static bool test_talk_kbd_empty() {
    printf("[RUN] test_talk_kbd_empty\n");
    reset();
    // TALK address 2 reg 0 = 0x2C.
    send_tx_byte(0x2C);
    int b0 = wait_rx_byte(32);
    CHECK_TRUE("got rx byte", b0 >= 0);
    CHECK_EQ("empty TALK b0 == 0xFF", b0, 0xFF);
    // No second byte expected for empty.
    // Wait a few more phi2 ticks to confirm we're back to idle without
    // emitting anything else.
    for (int i = 0; i < 16; i++) {
        if (dut->via_rx_valid) {
            printf("  FAIL unexpected second rx byte 0x%02x\n", dut->via_rx_byte);
            return false;
        }
        dut->phi2_tick = 1; tick();
        dut->phi2_tick = 0; tick();
    }
    return true;
}

static bool test_talk_kbd_event() {
    printf("[RUN] test_talk_kbd_event\n");
    reset();
    // Inject one keycode.
    dut->inj_kc_byte  = 0x35;       // ADB code for some key
    dut->inj_kc_valid = 1;
    tick();
    dut->inj_kc_valid = 0;
    tick();
    // Confirm injection bumped the FIFO.
    CHECK_EQ("kbd_status post-inject", dut->inj_kc_status & 0x01, 0x01);
    // Now TALK reg 0 should return {0x35, 0xFF}.
    send_tx_byte(0x2C);
    int b0 = wait_rx_byte(32);
    int b1 = wait_rx_byte(32);
    CHECK_EQ("kbd b0", b0, 0x35);
    CHECK_EQ("kbd b1", b1, 0xFF);
    return true;
}

static bool test_talk_kbd_two_events() {
    printf("[RUN] test_talk_kbd_two_events\n");
    reset();
    // Inject two keycodes back to back.
    dut->inj_kc_byte  = 0x10;
    dut->inj_kc_valid = 1; tick(); dut->inj_kc_valid = 0; tick();
    dut->inj_kc_byte  = 0x90;       // 0x90 = release of code 0x10
    dut->inj_kc_valid = 1; tick(); dut->inj_kc_valid = 0; tick();
    send_tx_byte(0x2C);
    int b0 = wait_rx_byte(32);
    int b1 = wait_rx_byte(32);
    CHECK_EQ("kbd b0", b0, 0x10);
    CHECK_EQ("kbd b1", b1, 0x90);
    return true;
}

static bool test_talk_kbd_register3() {
    printf("[RUN] test_talk_kbd_register3\n");
    reset();
    // TALK address 2 reg 3 = 0x2F.
    send_tx_byte(0x2F);
    int b0 = wait_rx_byte(32);
    int b1 = wait_rx_byte(32);
    // byte0 = {srq_enable, 3'b000, addr}: srq=1 default, addr=2 → 0x82.
    CHECK_EQ("kbd reg3 b0", b0, 0x82);
    // byte1 = handler ID (default 1).
    CHECK_EQ("kbd reg3 b1", b1, 0x01);
    return true;
}

static bool test_listen_kbd_set_addr() {
    printf("[RUN] test_listen_kbd_set_addr\n");
    reset();
    // LISTEN address 2 reg 3 = 0x2B.  Payload byte0=0xE7 (=> 0xE in
    // upper nibble triggers "set address only" with new addr 7),
    // byte1=0xFE (leave handler).
    send_tx_byte(0x2B);
    send_tx_byte(0xE7);
    send_tx_byte(0xFE);
    // Expect a single ack byte (0x00).
    int ack = wait_rx_byte(32);
    CHECK_EQ("listen ack", ack, 0x00);
    // TALK reg3 at addr 2 should now NOT match (returns 0xFF).
    send_tx_byte(0x2F);
    int empty = wait_rx_byte(32);
    CHECK_EQ("old addr no resp", empty, 0xFF);
    // TALK reg3 at addr 7 should match and return {0x87, 0x01}.
    send_tx_byte(0x7F);
    int b0 = wait_rx_byte(32);
    int b1 = wait_rx_byte(32);
    CHECK_EQ("new reg3 b0", b0, 0x87);
    CHECK_EQ("new reg3 b1", b1, 0x01);
    return true;
}

static bool test_flush_kbd() {
    printf("[RUN] test_flush_kbd\n");
    reset();
    // Inject a keycode, then FLUSH addr 2 = 0x21.
    dut->inj_kc_byte  = 0x42;
    dut->inj_kc_valid = 1; tick(); dut->inj_kc_valid = 0; tick();
    send_tx_byte(0x21);
    int ack = wait_rx_byte(32);
    CHECK_EQ("flush ack", ack, 0x00);
    // Subsequent TALK reg 0 should be empty.
    send_tx_byte(0x2C);
    int empty = wait_rx_byte(32);
    CHECK_EQ("post-flush empty", empty, 0xFF);
    return true;
}

static bool test_mouse_event_motion() {
    printf("[RUN] test_mouse_event_motion\n");
    reset();
    // Inject dx=+5, dy=-3.
    dut->inj_dx       = 5;
    dut->inj_dx_valid = 1; tick(); dut->inj_dx_valid = 0; tick();
    dut->inj_dy       = -3;
    dut->inj_dy_valid = 1; tick(); dut->inj_dy_valid = 0; tick();
    // TALK addr 3 reg 0 = 0x3C.
    send_tx_byte(0x3C);
    int b0 = wait_rx_byte(32);
    int b1 = wait_rx_byte(32);
    // byte0 = {~button, sat7(dy)}. button=0 → bit7=1.  dy=-3 → 0x7D
    // (7-bit signed -3).  → 0xFD.
    CHECK_EQ("mouse b0 (button|dy)", b0, 0xFD);
    // byte1 = {1, sat7(dx)}.  dx=+5 → 0x05 → 0x85.
    CHECK_EQ("mouse b1 (1|dx)", b1, 0x85);
    return true;
}

static bool test_mouse_button() {
    printf("[RUN] test_mouse_button\n");
    reset();
    dut->inj_btn_state = 1;
    dut->inj_btn_valid = 1; tick(); dut->inj_btn_valid = 0; tick();
    send_tx_byte(0x3C);
    int b0 = wait_rx_byte(32);
    int b1 = wait_rx_byte(32);
    // button pressed (1) → ~button = 0; dy=0 → b0 = 0x00.
    CHECK_EQ("mouse b0 button down", b0, 0x00);
    // dx=0 → b1 = 0x80.
    CHECK_EQ("mouse b1 (1|0)", b1, 0x80);
    return true;
}

static bool test_srq_pending() {
    printf("[RUN] test_srq_pending\n");
    reset();
    // Inject without TALKing.  After the modem settles, adb_irq_pending
    // should rise.
    dut->inj_kc_byte  = 0x55;
    dut->inj_kc_valid = 1; tick(); dut->inj_kc_valid = 0; tick();
    // Run a bunch of clk to let modem settle into S_IDLE.
    for (int i = 0; i < 20; i++) tick();
    CHECK_EQ("srq raised", dut->adb_irq_pending, 1);
    // Drain via TALK; SRQ should fall.
    send_tx_byte(0x2C);
    (void)wait_rx_byte(32);
    (void)wait_rx_byte(32);
    // Settle.
    for (int i = 0; i < 16; i++) {
        dut->phi2_tick = 1; tick();
        dut->phi2_tick = 0; tick();
    }
    CHECK_EQ("srq cleared", dut->adb_irq_pending, 0);
    return true;
}

static bool test_unaddressed_talk() {
    printf("[RUN] test_unaddressed_talk\n");
    reset();
    // TALK addr 5 reg 0 = 0x5C.  No device matches.
    send_tx_byte(0x5C);
    int b0 = wait_rx_byte(32);
    CHECK_EQ("no resp == 0xFF", b0, 0xFF);
    // Confirm no second byte.
    for (int i = 0; i < 16; i++) {
        if (dut->via_rx_valid) {
            printf("  FAIL unexpected 2nd byte 0x%02x\n", dut->via_rx_byte);
            return false;
        }
        dut->phi2_tick = 1; tick();
        dut->phi2_tick = 0; tick();
    }
    return true;
}

static bool test_listen_then_talk_handler() {
    printf("[RUN] test_listen_then_talk_handler\n");
    reset();
    // LISTEN addr 2 reg 3 with byte0=0x00 (no addr change), byte1=0x42
    // (set handler to 0x42).
    send_tx_byte(0x2B);
    send_tx_byte(0x00);
    send_tx_byte(0x42);
    int ack = wait_rx_byte(32);
    CHECK_EQ("listen ack", ack, 0x00);
    // TALK reg 3 should return {0x82, 0x42}.
    send_tx_byte(0x2F);
    int b0 = wait_rx_byte(32);
    int b1 = wait_rx_byte(32);
    CHECK_EQ("kbd reg3 b0", b0, 0x82);
    CHECK_EQ("new handler b1", b1, 0x42);
    return true;
}

// ─── Driver ───────────────────────────────────────────────────────────

typedef bool (*test_fn)();
struct test_entry { const char* name; test_fn fn; };

static const test_entry TESTS[] = {
    {"test_reset_state",            test_reset_state},
    {"test_talk_kbd_empty",         test_talk_kbd_empty},
    {"test_talk_kbd_event",         test_talk_kbd_event},
    {"test_talk_kbd_two_events",    test_talk_kbd_two_events},
    {"test_talk_kbd_register3",     test_talk_kbd_register3},
    {"test_listen_kbd_set_addr",    test_listen_kbd_set_addr},
    {"test_flush_kbd",              test_flush_kbd},
    {"test_mouse_event_motion",     test_mouse_event_motion},
    {"test_mouse_button",           test_mouse_button},
    {"test_srq_pending",            test_srq_pending},
    {"test_unaddressed_talk",       test_unaddressed_talk},
    {"test_listen_then_talk_handler", test_listen_then_talk_handler},
};

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_adb;

    int total = sizeof(TESTS)/sizeof(TESTS[0]);
    for (int i = 0; i < total; i++) {
        bool ok = TESTS[i].fn();
        if (ok) {
            printf("[PASS] %s\n", TESTS[i].name);
            n_pass++;
        } else {
            printf("[FAIL] %s\n", TESTS[i].name);
            n_fail++;
        }
    }

    printf("\n%d/%d scenarios passed.\n", n_pass, total);

    delete dut;
    return n_fail ? 1 : 0;
}
