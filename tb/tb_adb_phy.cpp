// tb_adb_phy.cpp — directed unit testbench for the bit-level ADB PHY
// (rtl/mac/adb_phy.v), driving it exactly as a real ADB host (the Q700
// PIC1654S modem) would over the open-drain bus, and decoding whatever
// bit-level response the addressed device (adb_keyboard.v /
// adb_mouse.v) drives back.
//
// This is deliberately independent of tb_adb.cpp (which covers the
// byte-level adb_modem.v abstraction — dead code on real HW, see
// rtl/soc/fpga_top_peripherals.vh's "ADB modem + bus devices" comment).
// adb_phy.v is what actually answers the real PIC firmware's ADB-bus
// traffic during boot, so this tb exercises the real fix: a TALK to an
// address a device answers must produce a genuine open-drain response
// frame (start bit + 16 data bits + stop bit) that a bit-sampling host
// can decode back into the device's register bytes.
//
// Build: make tb-adb-phy
//
// Scenarios (each returns bool — true on PASS):
//   1. reset_idle          — at reset, bus idle-high, no dev_cmd fires.
//   2. talk_kbd_r0_event   — inject a keycode; TALK addr=2 reg=0 decodes
//                            back to {keycode, 0xFF} over the bus.
//   3. talk_kbd_r3         — TALK addr=2 reg=3 decodes to {SRQ|addr=2,
//                            handler=1}.
//   4. talk_mouse_r0       — inject dx/dy/button; TALK addr=3 reg=0
//                            decodes back the motion report.
//   5. talk_unaddressed    — TALK to an address neither device answers
//                            produces pure bus silence (no low pulse at
//                            all during the response window).
//   6. srq_hold_after_empty_talk — a device with a pending event (SRQ)
//                            that ISN'T the one just TALKed (which
//                            answers empty) still gets to stretch the
//                            command stop bit's low SRQ_HOLD_TICKS past
//                            the host's release — the real ADB Service
//                            Request signal, in the ONLY window the
//                            real PIC firmware ever samples it (its
//                            bounded post-stop release-wait loop; see
//                            rtl/mac/adb_phy.v's header).
//   7. srq_hold_after_valid_response — a DIFFERENT device with a
//                            still-pending event gets its stop-bit SRQ
//                            stretch even when the TALKed device
//                            answers with data (the response frame
//                            follows after the hold ends).
//   8. no_srq_hold_once_serviced — once the only device with a pending
//                            event has been TALKed (consuming it), no
//                            SRQ hold follows.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <verilated.h>
#include "Vtb_adb_phy.h"

static Vtb_adb_phy* dut      = nullptr;
static uint64_t      sim_time = 0;
static int            n_pass  = 0;
static int            n_fail  = 0;

// ADB host frame timing, mirroring what the REAL 342S0440-B firmware
// measurably produces on the bus (one phi2_step() tick == one PIC
// instruction cycle == one adb_phy.v counter step; see
// tb_adb_pic_phy.cpp, which measured these with the genuine firmware in
// the loop): attention low ~365, sync HIGH gap ~30, self-clocked bit
// cells ('1' = low ~16, '0' = low ~30), stop low ~30 right after bit 8
// (no Tlt before it).  adb_phy.v's receiver is self-clocking, so these
// only need to be on the right side of its width thresholds.
static const int ATTN_TICKS  = 365;
static const int SYNC_HIGH   = 30;
static const int LOW0        = 30;   // '0' bit low width (> BIT1_MAX_TICKS)
static const int LOW1        = 16;   // '1' bit low width (<= BIT1_MAX_TICKS)
static const int HIGH0       = 16;   // '0' bit high tail
static const int HIGH1       = 30;   // '1' bit high tail
static const int STOP_LOW    = 30;
// Response TX timing (adb_phy.v's transmit constants).
static const int RESP_CELL   = 50;   // TX_CELL_TICKS
static const int RESP_MARGIN = 300;  // extra slack while scanning for the response
// SRQ: adb_phy drives low from the stop bit's falling edge through
// SRQ_HOLD_TICKS(90) past the host's release.  send_command() returns
// at that release, so the observable remaining low run is ~90 ticks.
static const int SRQ_RUN_MIN = 75;
static const int SRQ_RUN_MAX = 100;

static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

// One phi2_tick pulse (1 simulated ADB microsecond) plus a few plain
// clk edges so the un-gated device-bus dispatch/response-latch logic
// (adb_phy.v's dispatch_req edge detector + resp_latched_* capture)
// settles well before the next phi2 pulse — see adb_phy.v's comment on
// why dev_cmd_valid must be a clean single-fast-clk pulse.
static void phi2_step() {
    dut->phi2_tick = 1; tick();
    dut->phi2_tick = 0;
    tick(); tick(); tick(); tick();
}

static void reset() {
    dut->rst           = 1;
    dut->phi2_tick      = 0;
    dut->pic_adb_out    = 1;   // idle-high
    dut->inj_kc_valid   = 0;
    dut->inj_kc_byte    = 0;
    dut->inj_btn_valid  = 0;
    dut->inj_btn_state  = 0;
    dut->inj_dx_valid   = 0;
    dut->inj_dx         = 0;
    dut->inj_dy_valid   = 0;
    dut->inj_dy         = 0;
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

// Drive one ADB data bit (MSB-first caller convention), self-clocked
// firmware shape: each cell is a low pulse whose width encodes the bit
// ('1' short ~16, '0' long ~30) followed by a high tail.
static void drive_bit(bool one) {
    int low_t  = one ? LOW1 : LOW0;
    int high_t = one ? HIGH1 : HIGH0;
    dut->pic_adb_out = 0;
    for (int i = 0; i < low_t; i++) phi2_step();
    dut->pic_adb_out = 1;
    for (int i = 0; i < high_t; i++) phi2_step();
}

// Drive a full Attention + sync gap + 8-bit command byte + host stop
// bit, shaped exactly like the real firmware's frame (bits are
// followed IMMEDIATELY by the stop pulse — no Tlt in between; Tlt
// comes after the stop, where the response window opens).  Leaves
// pic_adb_out = 1 (released) on return; the phy resolves the frame
// (SRQ / response / idle) at the stop bit's rising edge.
static void send_command(uint8_t cmd) {
    // Attention: low >= ATTN_MIN_TICKS, then release.
    dut->pic_adb_out = 0;
    for (int i = 0; i < ATTN_TICKS; i++) phi2_step();
    dut->pic_adb_out = 1;

    // Sync: a short HIGH gap; the first bit's falling edge starts the
    // byte.
    for (int i = 0; i < SYNC_HIGH; i++) phi2_step();

    // 8 command bits, MSB first.
    for (int b = 7; b >= 0; b--) drive_bit((cmd >> b) & 1);

    // Host's own stop bit, right after bit 8's high tail.
    dut->pic_adb_out = 0;
    for (int i = 0; i < STOP_LOW; i++) phi2_step();
    dut->pic_adb_out = 1;
    phi2_step();
}

// Host stop-to-start turnaround (Tlt) before a LISTEN payload frame.
// Real ADB Tlt is 140-260us; at one tick per PIC instruction cycle that
// lands well inside adb_phy's LSN_START_MAX (400) bound.
static const int TLT_TICKS = 100;

// Drive the host's LISTEN payload frame, which follows the LISTEN
// command's stop bit: Tlt turnaround, then start bit + 16 data bits
// (b0 then b1, MSB first) + stop bit, in the SAME self-clocked encoding
// as the command byte.  adb_phy uses the start/stop cells for framing
// only and ignores their values, so they just have to be valid bit
// cells.  Leaves the line released (high) on return.
static void send_listen_payload(uint8_t b0, uint8_t b1) {
    for (int i = 0; i < TLT_TICKS; i++) phi2_step();
    drive_bit(false);                                  // start bit
    for (int b = 7; b >= 0; b--) drive_bit((b0 >> b) & 1);
    for (int b = 7; b >= 0; b--) drive_bit((b1 >> b) & 1);
    drive_bit(true);                                   // stop bit
    phi2_step();
}

// After send_command(), scan the bus for a device response frame:
// start bit (low-dominant) + 16 data bits + stop bit.  Returns true
// and fills b0/b1 if a response was found; returns false (silence) if
// the bus stays high (adb_in==1) for the whole scan window.
static bool decode_response(uint8_t& b0, uint8_t& b1) {
    std::vector<int> low(RESP_MARGIN + 200 + 18 * RESP_CELL, 0);
    int n = (int)low.size();
    int start_idx = -1;
    for (int i = 0; i < n; i++) {
        phi2_step();
        bool is_low = (dut->adb_in == 0);
        low[i] = is_low ? 1 : 0;
        if (start_idx < 0 && is_low) start_idx = i;
    }
    if (start_idx < 0) return false;   // pure silence — no device answered

    auto sample_cell = [&](int cell_idx) -> bool {
        int idx = start_idx + cell_idx * RESP_CELL + RESP_CELL / 2;
        if (idx < 0 || idx >= n) return true;  // out of range -> treat as released/high
        return low[idx] != 0;   // true == bus was low at the mid-cell point
    };

    // Cell 0 = start bit (should be low-dominant, i.e. "0"-shaped).
    CHECK_TRUE("start bit is 0-shaped", sample_cell(0) == true);

    uint16_t word = 0;
    for (int k = 1; k <= 16; k++) {
        bool bit_is_one = !sample_cell(k);  // high at midpoint == '1'
        word = (uint16_t)((word << 1) | (bit_is_one ? 1 : 0));
    }
    b0 = (uint8_t)(word >> 8);
    b1 = (uint8_t)(word & 0xFF);

    // Cell 17 = stop bit (should be high-dominant, i.e. "1"-shaped).
    CHECK_TRUE("stop bit is 1-shaped", sample_cell(17) == false);
    return true;
}

// Scan `window` ticks starting from wherever the caller left off, and
// return the length of the LONGEST consecutive run of adb_in==0 (bus
// low) observed.  A normal framed response's individual bit cells are
// low for at most ~32 ticks (a "0" bit); the SRQ hold
// (rtl/mac/adb_phy.v's ST_SRQ_HOLD) keeps the bus low for
// SRQ_HOLD_TICKS(90) past the host's stop-bit release (send_command
// returns right at that release, so the observable remaining run is
// ~90), which is unmistakably longer — the max run length alone
// robustly distinguishes "an SRQ hold happened somewhere in this
// window" from ordinary bit framing, without needing to know its
// exact start offset.
static int max_low_run(int window) {
    int best = 0, cur = 0;
    for (int i = 0; i < window; i++) {
        phi2_step();
        if (dut->adb_in == 0) {
            cur++;
            if (cur > best) best = cur;
        } else {
            cur = 0;
        }
    }
    return best;
}

static bool test_reset_idle() {
    reset();
    for (int i = 0; i < 20; i++) phi2_step();
    CHECK_EQ("adb_in idle-high", dut->adb_in, 1);
    return true;
}

static bool test_talk_kbd_r0_event() {
    reset();
    // Inject one keycode (bit7=0 => key-down code 0x2A) into the
    // keyboard's FIFO.
    dut->inj_kc_valid = 1;
    dut->inj_kc_byte  = 0x2A;
    tick();
    dut->inj_kc_valid = 0;
    for (int i = 0; i < 10; i++) tick();

    // TALK addr=2 (keyboard default), reg 0: addr=0010, op=11, reg=00
    // -> 0b0010_1100 = 0x2C.
    send_command(0x2C);

    uint8_t b0 = 0, b1 = 0;
    CHECK_TRUE("kbd TALK R0 produced a response", decode_response(b0, b1));
    CHECK_EQ("kbd TALK R0 byte0 = injected keycode", b0, 0x2A);
    CHECK_EQ("kbd TALK R0 byte1 = 0xFF (no second key)", b1, 0xFF);
    return true;
}

static bool test_talk_kbd_r3() {
    reset();
    // TALK addr=2, reg 3: 0b0010_1111 = 0x2F.
    send_command(0x2F);

    uint8_t b0 = 0, b1 = 0;
    CHECK_TRUE("kbd TALK R3 produced a response", decode_response(b0, b1));
    // byte0 = {srq_enable=1, 3'b000, addr=2} = 0x82; byte1 = handler = 1.
    CHECK_EQ("kbd TALK R3 byte0 = 0x82 (SRQ|addr2)", b0, 0x82);
    CHECK_EQ("kbd TALK R3 byte1 = handler 1", b1, 0x01);
    return true;
}

static bool test_talk_mouse_r0() {
    reset();
    dut->inj_btn_valid = 1;
    dut->inj_btn_state = 1;   // pressed
    tick();
    dut->inj_btn_valid = 0;
    dut->inj_dx_valid  = 1;
    dut->inj_dx        = 5;
    tick();
    dut->inj_dx_valid = 0;
    dut->inj_dy_valid = 1;
    dut->inj_dy       = (int8_t)-3;
    tick();
    dut->inj_dy_valid = 0;
    for (int i = 0; i < 10; i++) tick();

    // TALK addr=3 (mouse default), reg 0: 0b0011_1100 = 0x3C.
    send_command(0x3C);

    uint8_t b0 = 0, b1 = 0;
    CHECK_TRUE("mouse TALK R0 produced a response", decode_response(b0, b1));
    // byte0: bit7 = ~button(pressed=1 -> 0), bits6..0 = dy(-3 -> 0x7D
    //   two's complement 7-bit = 125 = 0x7D).  byte1: bit7=1 (single
    //   button), bits6..0 = dx(5).
    CHECK_EQ("mouse TALK R0 byte0 (button+dy)", b0, 0x7D);
    CHECK_EQ("mouse TALK R0 byte1 (dx)",        b1, 0x85);
    return true;
}

static bool test_talk_unaddressed() {
    reset();
    // TALK addr=5 (neither keyboard(2) nor mouse(3)), reg 0: 0b0101_1100 = 0x5C.
    send_command(0x5C);

    uint8_t b0 = 0, b1 = 0;
    CHECK_TRUE("unaddressed TALK is pure silence", !decode_response(b0, b1));
    return true;
}

static bool test_srq_hold_after_empty_talk() {
    reset();
    // Give the mouse a pending event (SRQ) but TALK the keyboard
    // instead — the keyboard has nothing injected, so it answers empty
    // (real ADB silence in the normal response window).
    dut->inj_dx_valid = 1;
    dut->inj_dx        = 5;
    tick();
    dut->inj_dx_valid  = 0;
    for (int i = 0; i < 10; i++) tick();

    send_command(0x2C);  // TALK addr=2 (keyboard), reg 0 -> empty

    int run = max_low_run(400);
    CHECK_TRUE("SRQ hold present after empty TALK to an unrelated device",
               run >= SRQ_RUN_MIN && run <= SRQ_RUN_MAX);
    return true;
}

static bool test_srq_hold_after_valid_response() {
    reset();
    // Mouse has a real event to report...
    dut->inj_dx_valid = 1;
    dut->inj_dx        = 5;
    tick();
    dut->inj_dx_valid  = 0;
    // ...and the keyboard ALSO has a pending event we will NOT service
    // this transaction.
    dut->inj_kc_valid  = 1;
    dut->inj_kc_byte   = 0x2A;
    tick();
    dut->inj_kc_valid  = 0;
    for (int i = 0; i < 10; i++) tick();

    // TALK addr=3 (mouse), reg 0 -> normal data response, consumes the
    // mouse's own event.  The keyboard's still-pending event should
    // chain an SRQ hold immediately after the response's stop bit.
    send_command(0x3C);

    int run = max_low_run(1400);
    CHECK_TRUE("SRQ hold present alongside a valid TALK response",
               run >= SRQ_RUN_MIN && run <= SRQ_RUN_MAX);
    return true;
}

static bool test_no_srq_hold_once_serviced() {
    reset();
    // Mouse is the ONLY device with a pending event.
    dut->inj_dx_valid = 1;
    dut->inj_dx        = 5;
    tick();
    dut->inj_dx_valid  = 0;
    for (int i = 0; i < 10; i++) tick();

    // TALK addr=3 (mouse), reg 0 -> consumes the mouse's only event.
    send_command(0x3C);

    // No device has anything left to report -> no SRQ hold anywhere in
    // the window (longest low run stays within ordinary bit-cell
    // bounds, well under the SRQ hold length).
    int run = max_low_run(1400);
    CHECK_TRUE("no SRQ hold once the only pending event was serviced",
               run < 60);
    return true;
}

// ── LISTEN payload capture (2026-07-25) ───────────────────────────────
// These are the regression gate for the bug that kept ADB dead on real
// hardware: adb_phy never received the payload frame after a LISTEN
// command, so dev_listen_valid was permanently 0 and devices could never
// be relocated.  Mac OS's ADBReInit RELOCATES devices via LISTEN R3, so
// with the payload dropped its device table named addresses our devices
// never moved to and every later poll hit a dead address.
//
// NOTE these deliberately exercise the BIT-LEVEL path through adb_phy.
// tb_adb.cpp's own LISTEN coverage drives adb_modem.v — the byte-level
// cousin that fpga_top_peripherals.vh documents as inert and NOT part of
// the real boot path — so it could (and did) stay green throughout.
static bool test_listen_r3_relocates_mouse() {
    reset();
    uint8_t b0 = 0, b1 = 0;

    // Mouse answers at its default address 3 first.
    send_command(0x3F);                       // addr 3, TALK, reg 3
    CHECK_TRUE("mouse answers at default addr 3", decode_response(b0, b1));
    CHECK_EQ("...reporting addr 3", b0 & 0x0F, 0x03);

    // LISTEN addr 3 reg 3 (0x3B) + payload: byte0 = 0xE9 (Apple "change
    // address only" code 0xE | new address 9), byte1 = 0xFE (keep the
    // existing handler ID).
    send_command(0x3B);
    send_listen_payload(0xE9, 0xFE);

    // The old address must go silent...
    send_command(0x3F);
    CHECK_TRUE("old addr 3 is silent after relocate",
               !decode_response(b0, b1));

    // ...and the device must answer at its new one, handler preserved.
    send_command(0x9F);                       // addr 9, TALK, reg 3
    CHECK_TRUE("mouse answers at relocated addr 9",
               decode_response(b0, b1));
    CHECK_EQ("...reporting addr 9 (SRQ still enabled)", b0, 0x89);
    CHECK_EQ("...handler preserved through 0xFE", b1, 0x01);
    return true;
}

// The op gate: adb_phy collapses LISTEN R1/R2 into OP_LISTEN_R0, whose
// payload is device data rather than an address/handler pair.  Feeding
// an R0 payload that merely LOOKS like a relocate must not move the
// device, or ordinary register writes would scramble the bus.
static bool test_listen_r0_payload_does_not_relocate() {
    reset();
    uint8_t b0 = 0, b1 = 0;

    send_command(0x38);                       // addr 3, LISTEN, reg 0
    send_listen_payload(0xE9, 0xFE);          // relocate-shaped, but R0

    send_command(0x3F);
    CHECK_TRUE("mouse still answers at addr 3", decode_response(b0, b1));
    CHECK_EQ("...still reporting addr 3", b0 & 0x0F, 0x03);

    send_command(0x9F);
    CHECK_TRUE("nothing appeared at addr 9", !decode_response(b0, b1));
    return true;
}

// A LISTEN aimed at another device must not disturb ours, even though
// every device on the bus sees the same payload frame.
static bool test_listen_r3_other_addr_ignored() {
    reset();
    uint8_t b0 = 0, b1 = 0;

    send_command(0x2B);                       // addr 2 (keyboard), LISTEN R3
    send_listen_payload(0xEA, 0xFE);          // move keyboard to addr 10

    send_command(0x3F);                       // mouse untouched at 3?
    CHECK_TRUE("mouse unaffected by keyboard's relocate",
               decode_response(b0, b1));
    CHECK_EQ("...still addr 3", b0 & 0x0F, 0x03);

    send_command(0xAF);                       // addr 10, TALK R3
    CHECK_TRUE("keyboard did relocate to addr 10",
               decode_response(b0, b1));
    CHECK_EQ("...reporting addr 10", b0 & 0x0F, 0x0A);
    return true;
}

#define RUN(fn) do { \
    printf("[RUN] %s\n", #fn); \
    if (fn()) { printf("[PASS] %s\n", #fn); n_pass++; } \
    else      { printf("[FAIL] %s\n", #fn); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_adb_phy;

    RUN(test_reset_idle);
    RUN(test_talk_kbd_r0_event);
    RUN(test_talk_kbd_r3);
    RUN(test_talk_mouse_r0);
    RUN(test_talk_unaddressed);
    RUN(test_srq_hold_after_empty_talk);
    RUN(test_srq_hold_after_valid_response);
    RUN(test_no_srq_hold_once_serviced);
    RUN(test_listen_r3_relocates_mouse);
    RUN(test_listen_r0_payload_does_not_relocate);
    RUN(test_listen_r3_other_addr_ignored);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
