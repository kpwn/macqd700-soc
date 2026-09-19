// tb_adb_pic_phy.cpp — REAL-FIRMWARE ADB chain integration testbench.
//
// Drives tb_adb_pic_phy.v: the genuine 342S0440-B ADB-modem PIC firmware
// (rtl/mac/adb_pic_fw.hex on pic16c5x.v, via adb_pic_modem.v) is the ADB
// HOST, adb_phy.v + adb_keyboard.v + adb_mouse.v answer it, and this C++
// plays the VIA1/68k role at the CB1/CB2/state-pin level with the same
// edge phasing as via1.v's MAME-matched external-clock shift register
// (shift-out presents the next bit on EVEN CB1 edges starting with the
// first falling edge; shift-in samples CB2 on ODD (rising) edges).
//
// Why this tb exists: tb_adb_phy.cpp bit-bangs a synthetic host whose
// timing mirrors adb_phy.v's own constants, so it can never catch a
// mismatch between adb_phy.v and the REAL host firmware.  The 2026-07-21
// hardware bug (boot stalls at "Welcome to Macintosh" spinning on VIA1
// SR reads; an injected ADB mouse event's event_pending never clears)
// was exactly such a mismatch: the firmware's idle-state autopoll frames
// were never decoded by adb_phy.v, so no device was ever TALKed again
// after the initial 68k-driven transaction and no SRQ could ever be
// signalled.
//
// 68k<->PIC state protocol (from the 342S0440-B disassembly, see
// github.com/lampmerchant/macseadb88 and adb_phy.v's header):
//   S0 (state=0): 68k clocks a command byte INTO the PIC (VIA SR
//       shift-out, PIC's ViaReceive).  If an autopoll result is pending
//       (F2_UNK1), the byte is a throwaway and the PIC asserts INT.
//   S1 (state=1): for a freshly-loaded command, the PIC runs the ADB bus
//       transaction (attention + command byte + stop [+ response]) and
//       then clocks buffer byte 0 back to the VIA (ViaSend), asserting
//       INT first if the transaction errored.
//   S2 (state=2): PIC clocks the next buffer byte back, asserting INT
//       first if SRQ was detected on the bus.
//   S3 (state=3): idle.  The PIC autonomously re-issues the last TALK
//       command (stashed in GPR14) on a countdown (~every other CDOWNA
//       expiry), and if it gets data or sees SRQ it notifies the 68k by
//       clocking the command byte into the VIA SR unsolicited.
//
// Scenarios:
//   1. sync_talk_mouse   — full 68k-driven TALK R0 addr 3 with an event
//                          injected first: adb_phy must decode the REAL
//                          firmware's frame, the mouse must answer, and
//                          the PIC must hand both data bytes back.
//   2. autopoll_drain    — event injected while idle in S3: the PIC's
//                          autonomous autopoll must reach the mouse
//                          (event_pending clears) and the PIC must
//                          notify the 68k via an unsolicited SR byte;
//                          the 68k-model then fetches the buffered data.
//                          This is the exact hardware-observed bug.
//   3. kbd_srq_via_mouse_poll — keyboard event injected while autopoll
//                          targets the mouse: the phy's SRQ bus hold
//                          must fire during the autopoll TALK, the PIC
//                          must notify, and a follow-up TALK to the
//                          keyboard must drain the keycode.
//
// Build: make tb-adb-pic-phy

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <verilated.h>
#include "Vtb_adb_pic_phy.h"

static Vtb_adb_pic_phy* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;

// ── time base ──────────────────────────────────────────────────────────
// One phi2 "tick" == one PIC instruction cycle == one adb_phy counter
// step (they share phi2_tick in rtl/soc/fpga_top_peripherals.vh).
static uint64_t phi2_ticks = 0;

// ── VIA-role model state ───────────────────────────────────────────────
static bool     out_active = false;   // SR shift-out (68k -> PIC) armed
static uint8_t  out_byte = 0;
static int      out_edge_cnt = 0;     // CB1 edges seen since arm (16 = done)

static uint8_t  in_shift = 0;         // SR shift-in (PIC -> 68k)
static int      in_bits = 0;
static uint64_t in_last_edge = 0;
static std::vector<uint8_t> rx_bytes; // completed PIC->VIA bytes

static int      prev_cb1 = 1;

// ── ADB bus waveform logger (for diagnostics) ─────────────────────────
struct Pulse { int level; uint64_t start; uint64_t len; };
static std::vector<Pulse> bus_log;
static int      prev_bus_lvl = 1;
static uint64_t bus_lvl_start = 0;
static bool     bus_log_en = false;

// dev_cmd_valid pulse capture (fast-clk one-shot from adb_phy) — must
// be sampled every fast clk edge, not once per phi2 step.
struct DevCmd { uint8_t addr; uint8_t op; uint64_t t; };
static std::vector<DevCmd> dev_cmds;

static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
    if (dut->dev_cmd_valid_o)
        dev_cmds.push_back({(uint8_t)dut->dev_cmd_addr_o,
                            (uint8_t)dut->dev_cmd_op_o, phi2_ticks});
}

static void via_monitors() {
    // CB1 edge handling (VIA external-clock shift register model).
    int cb1 = dut->cb1;
    if (cb1 != prev_cb1) {
        if (out_active) {
            // Shift-out: present the next bit on EVEN edges (first
            // falling edge presents b7) — matches via1.v / MAME
            // via6522 shift_out phasing.
            if ((out_edge_cnt & 1) == 0) {
                dut->via_cb2_out = (out_byte >> 7) & 1;
                out_byte = (uint8_t)(out_byte << 1);
            }
            out_edge_cnt++;
        } else if (cb1) {
            // Shift-in: sample CB2 on rising edges.
            if (in_bits != 0 && (phi2_ticks - in_last_edge) > 1000) {
                in_bits = 0;  // stale partial byte — resync
                in_shift = 0;
            }
            in_shift = (uint8_t)((in_shift << 1) | (dut->cb2 & 1));
            in_bits++;
            in_last_edge = phi2_ticks;
            if (in_bits == 8) {
                rx_bytes.push_back(in_shift);
                in_bits = 0;
                in_shift = 0;
            }
        }
        prev_cb1 = cb1;
    }

    // Host-drive waveform logger.
    if (bus_log_en) {
        int lvl = dut->pic_adb_out_o & 1;
        if (lvl != prev_bus_lvl) {
            bus_log.push_back({prev_bus_lvl, bus_lvl_start,
                               phi2_ticks - bus_lvl_start});
            prev_bus_lvl = lvl;
            bus_lvl_start = phi2_ticks;
        }
    }
}

static void phi2_step() {
    dut->phi2_tick = 1; tick();
    dut->phi2_tick = 0; tick(); tick();
    phi2_ticks++;
    via_monitors();
}

static void run_ticks(uint64_t n) { for (uint64_t i = 0; i < n; i++) phi2_step(); }

static void reset() {
    dut->rst = 1;
    dut->phi2_tick = 0;
    dut->adb_state = 3;      // S3 idle — matches firmware's LASTA init
    dut->via_cb2_oe = 0;
    dut->via_cb2_out = 1;
    dut->inj_kc_valid = 0;  dut->inj_kc_byte = 0;
    dut->inj_btn_valid = 0; dut->inj_btn_state = 0;
    dut->inj_dx_valid = 0;  dut->inj_dx = 0;
    dut->inj_dy_valid = 0;  dut->inj_dy = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    for (int i = 0; i < 8; i++) tick();

    out_active = false; out_edge_cnt = 0;
    in_shift = 0; in_bits = 0; in_last_edge = 0;
    rx_bytes.clear(); dev_cmds.clear(); bus_log.clear();
    prev_cb1 = dut->cb1;
    prev_bus_lvl = 1; bus_lvl_start = 0; bus_log_en = false;
    phi2_ticks = 0;

    // Let the firmware run its init and settle in the main loop.
    run_ticks(600);
}

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s (line %d)\n", name, __LINE__); \
        return false; \
    } \
} while (0)

#define CHECK_EQ(name, got, exp) do { \
    uint64_t _g = (uint64_t)(got), _e = (uint64_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%02lx, expected 0x%02lx (line %d)\n", \
               name, (unsigned long)_g, (unsigned long)_e, __LINE__); \
        return false; \
    } \
} while (0)

static void dump_bus_log(const char* tag) {
    printf("  [%s] host-drive waveform (%zu pulses):\n", tag, bus_log.size());
    size_t n = bus_log.size();
    size_t first = (n > 48) ? n - 48 : 0;
    for (size_t i = first; i < n; i++)
        printf("    t=%6llu  %s for %llu ticks\n",
               (unsigned long long)bus_log[i].start,
               bus_log[i].level ? "HIGH" : "LOW ",
               (unsigned long long)bus_log[i].len);
}

// ── 68k-driver primitives ─────────────────────────────────────────────

// Clock one byte into the PIC: arm SR shift-out, set the requested
// state, wait for the PIC's 16 CB1 edges.  Returns false on timeout.
static bool via_send_byte(uint8_t byte, int state, uint64_t timeout = 30000) {
    out_active = true;
    out_byte = byte;
    out_edge_cnt = 0;
    dut->via_cb2_oe = 1;
    dut->via_cb2_out = 1;
    dut->adb_state = state & 3;
    uint64_t start = phi2_ticks;
    while (out_edge_cnt < 16) {
        phi2_step();
        if (phi2_ticks - start > timeout) {
            out_active = false;
            printf("  [via_send_byte] timeout: only %d CB1 edges\n", out_edge_cnt);
            return false;
        }
    }
    out_active = false;
    // Hold the final bit a little past the 16th CB1 edge: the PIC
    // samples DIO one-two instructions AFTER raising SCLK, and the real
    // VIA keeps driving CB2 until the 68k reprograms ACR (much later).
    run_ticks(10);
    dut->via_cb2_oe = 0;   // back to shift-in (release CB2)
    return true;
}

// Set state and wait for one PIC->VIA byte (ViaSend).  Returns false on
// timeout.  The byte lands in *out.
static bool via_wait_rx_byte(int state, uint8_t* out, uint64_t timeout = 30000) {
    size_t have = rx_bytes.size();
    in_bits = 0; in_shift = 0;
    dut->adb_state = state & 3;
    uint64_t start = phi2_ticks;
    while (rx_bytes.size() == have) {
        phi2_step();
        if (phi2_ticks - start > timeout) {
            printf("  [via_wait_rx_byte] timeout in state %d (in_bits=%d)\n",
                   state, in_bits);
            return false;
        }
    }
    *out = rx_bytes.back();
    return true;
}

// Full 68k-driven command transaction: S0 (command byte in), S1 (byte 0
// back), S2 (byte 1 back), S3 (idle).  Mirrors the ROM ADB driver's use
// of the state pins.
static bool via_transaction(uint8_t cmd, uint8_t* b0, uint8_t* b1) {
    if (!via_send_byte(cmd, 0)) return false;
    run_ticks(50);
    // S1 runs the whole ADB bus transaction before the reply byte comes
    // back — allow generously for attention + frame + response.
    if (!via_wait_rx_byte(1, b0, 60000)) return false;
    run_ticks(50);
    if (!via_wait_rx_byte(2, b1, 30000)) return false;
    run_ticks(50);
    dut->adb_state = 3;
    run_ticks(200);
    return true;
}

// ── scenarios ─────────────────────────────────────────────────────────

// ADB command bytes: {addr[3:0], op[1:0]=11 (talk), reg[1:0]}
static const uint8_t CMD_TALK_MOUSE_R0 = 0x3C;  // addr 3, talk, reg 0
static const uint8_t CMD_TALK_KBD_R0   = 0x2C;  // addr 2, talk, reg 0

static void inject_mouse_dx(int8_t dx) {
    dut->inj_dx_valid = 1;
    dut->inj_dx = (uint8_t)dx;
    tick();
    dut->inj_dx_valid = 0;
    tick();
}

static bool test_sync_talk_mouse() {
    reset();

    // Inject button press + dx=5 + dy=-3 (same vector tb_adb_phy.cpp's
    // talk_mouse_r0 uses).
    dut->inj_btn_valid = 1; dut->inj_btn_state = 1; tick();
    dut->inj_btn_valid = 0;
    dut->inj_dx_valid = 1; dut->inj_dx = 5; tick();
    dut->inj_dx_valid = 0;
    dut->inj_dy_valid = 1; dut->inj_dy = (uint8_t)(int8_t)-3; tick();
    dut->inj_dy_valid = 0;
    run_ticks(5);
    CHECK_EQ("mouse event_pending set after injection",
             dut->inj_mouse_status & 1, 1);

    bus_log_en = true;
    uint8_t b0 = 0, b1 = 0;
    bool ok = via_transaction(CMD_TALK_MOUSE_R0, &b0, &b1);
    if (!ok || dev_cmds.empty()) dump_bus_log("sync_talk");
    CHECK_TRUE("transaction completed", ok);

    // The phy must have decoded the REAL firmware's command frame and
    // dispatched TALK R0 to addr 3.
    bool saw_talk = false;
    for (auto& c : dev_cmds)
        if (c.addr == 3 && c.op == 4 /* OP_TALK_R0 */) saw_talk = true;
    CHECK_TRUE("adb_phy decoded the firmware's TALK frame (dev_cmd addr=3 op=TALK_R0)",
               saw_talk);

    // Mouse data made the round trip back through the PIC's buffer:
    // byte0 = {~button=0, dy=-3 (7-bit) = 0x7D} = 0x7D
    // byte1 = {1, dx=5} = 0x85
    CHECK_EQ("mouse byte0 (button+dy)", b0, 0x7D);
    CHECK_EQ("mouse byte1 (dx)",        b1, 0x85);
    CHECK_EQ("mouse event consumed by TALK", dut->inj_mouse_status & 1, 0);
    return true;
}

static bool test_autopoll_drain() {
    reset();

    // Prime the PIC's autopoll: one normal 68k-driven TALK to the mouse
    // (empty — no event injected yet).  This stashes the TALK command in
    // the firmware's GPR14 and sets its autopoll-armed flag (F2_UNK0).
    uint8_t b0 = 0, b1 = 0;
    CHECK_TRUE("priming TALK completed",
               via_transaction(CMD_TALK_MOUSE_R0, &b0, &b1));

    // Idle in S3.  Now inject a mouse event — exactly what the JTAG
    // adb-mouse command did on real hardware.
    dut->adb_state = 3;
    run_ticks(500);
    dev_cmds.clear();
    rx_bytes.clear();
    inject_mouse_dx(5);
    CHECK_EQ("event_pending set", dut->inj_mouse_status & 1, 1);

    // The firmware's autonomous autopoll must re-TALK the mouse and
    // consume the event.  Autopoll fires roughly every other CDOWNA
    // countdown expiry (a few thousand PIC cycles); give it plenty.
    uint64_t start = phi2_ticks;
    bool drained = false;
    while (phi2_ticks - start < 200000) {
        phi2_step();
        if ((dut->inj_mouse_status & 1) == 0) { drained = true; break; }
    }
    if (!drained) {
        printf("  [autopoll] event_pending still set after %llu ticks; "
               "dev_cmds seen: %zu\n",
               (unsigned long long)(phi2_ticks - start), dev_cmds.size());
    }
    CHECK_TRUE("injected mouse event drained by firmware autopoll", drained);

    // The PIC must notify the 68k: an unsolicited SR byte (the stashed
    // TALK command echo) clocked into the VIA while we sit in S3.
    start = phi2_ticks;
    while (rx_bytes.empty() && phi2_ticks - start < 100000) phi2_step();
    CHECK_TRUE("PIC clocked an unsolicited notification byte into the VIA SR",
               !rx_bytes.empty());
    CHECK_EQ("notification byte is the stashed TALK command",
             rx_bytes.back(), CMD_TALK_MOUSE_R0);

    // 68k-model fetch of the buffered autopoll data: S0 with a throwaway
    // byte (the PIC discards it and asserts INT), then S1/S2 read the
    // buffered report.
    rx_bytes.clear();
    CHECK_TRUE("throwaway S0 byte accepted", via_send_byte(0x00, 0));
    run_ticks(50);
    CHECK_TRUE("buffered byte0 read", via_wait_rx_byte(1, &b0, 60000));
    run_ticks(50);
    CHECK_TRUE("buffered byte1 read", via_wait_rx_byte(2, &b1, 30000));
    dut->adb_state = 3;
    run_ticks(200);

    // dx=5, no button: byte0 = {~0=1, dy=0} = 0x80, byte1 = {1, 5} = 0x85.
    CHECK_EQ("autopolled byte0", b0, 0x80);
    CHECK_EQ("autopolled byte1", b1, 0x85);
    return true;
}

static bool test_kbd_srq_via_mouse_poll() {
    reset();

    // Prime autopoll on the MOUSE.
    uint8_t b0 = 0, b1 = 0;
    CHECK_TRUE("priming TALK completed",
               via_transaction(CMD_TALK_MOUSE_R0, &b0, &b1));
    dut->adb_state = 3;
    run_ticks(500);
    dev_cmds.clear();
    rx_bytes.clear();

    // Inject a KEYBOARD event: the autopoll targets the mouse (which
    // answers empty), so the keyboard must raise SRQ during the poll
    // (adb_phy's bus-low hold) for the PIC to learn about it.
    dut->inj_kc_valid = 1; dut->inj_kc_byte = 0x2A; tick();
    dut->inj_kc_valid = 0; tick();

    // Wait for the PIC's notification (F_SRQ -> F2_UNK1 -> unsolicited
    // ViaSend while idle).
    uint64_t start = phi2_ticks;
    while (rx_bytes.empty() && phi2_ticks - start < 300000) phi2_step();
    CHECK_TRUE("PIC notified the 68k after keyboard SRQ", !rx_bytes.empty());

    // 68k-model: drain the pending autopoll buffer first (throwaway S0 +
    // S1/S2 reads — contents are don't-care for an SRQ'd empty poll),
    // then TALK the keyboard directly and drain the keycode.
    rx_bytes.clear();
    CHECK_TRUE("throwaway S0 byte accepted", via_send_byte(0x00, 0));
    run_ticks(50);
    CHECK_TRUE("pending byte0 read", via_wait_rx_byte(1, &b0, 60000));
    run_ticks(50);
    CHECK_TRUE("pending byte1 read", via_wait_rx_byte(2, &b1, 30000));
    dut->adb_state = 3;
    run_ticks(300);

    CHECK_TRUE("keyboard TALK completed",
               via_transaction(CMD_TALK_KBD_R0, &b0, &b1));
    CHECK_EQ("keyboard byte0 = injected keycode", b0, 0x2A);
    CHECK_EQ("keyboard byte1 = 0xFF (no second key)", b1, 0xFF);
    CHECK_EQ("keyboard FIFO drained", dut->inj_kc_status & 1, 0);
    return true;
}

#define RUN(fn) do { \
    printf("[RUN] %s\n", #fn); \
    if (fn()) { printf("[PASS] %s\n", #fn); n_pass++; } \
    else      { printf("[FAIL] %s\n", #fn); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_adb_pic_phy;

    RUN(test_sync_talk_mouse);
    RUN(test_autopoll_drain);
    RUN(test_kbd_srq_via_mouse_poll);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
