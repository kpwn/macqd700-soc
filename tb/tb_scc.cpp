// tb_scc.cpp — Verilator unit testbench for scc.v (Zilog Z85C30 real)
//
// Build:   make tb-scc
//
// Scenarios (tagged by name so the agent-policy grep can find them):
//   1.  test_reset_clean              — reset values of the whole bank
//   2.  test_wr0_pointer_semantics    — WR0 pointer + reset to 0 after WRn
//   3.  test_wrn_roundtrip            — write WR1..WR15, read back via RRn
//   4.  test_rr0_idle_status          — RR0 on idle chan reports TX empty
//   5.  test_tx_no_external_device    — TX drains without loopback and leaves RX empty
//   6.  test_tx_loopback_single       — WR8 + loopback + BRG → RR8 echoes
//   7.  test_tx_loopback_3char_fifo   — pump 3 chars, verify FIFO order
//   8.  test_rx_irq_raises            — char arrival raises irq via WR9 MIE
//   9.  test_tx_empty_irq             — TX-empty IRQ fires + clears on WR8
//  10.  test_brg_baud_rate            — byte-time scales with time-const
//  11.  test_master_ie_gate           — WR9[3]=0 forces irq low
//  12.  test_chan_a_vs_b_isolation    — chan B activity doesn't leak to A
//  13.  test_wr9_chanA_reset          — chan A reset clears A, leaves B
//  14.  test_wr9_hardware_reset       — WR9[7:6]=11 clears both channels
//  15.  test_rr3_ip_summary           — RR3 on chan A reflects both channels
//  16.  test_chan_b_decode_and_rr2    — B decode isolation + RR2 vector handling
//  17.  test_rr1_overrun_and_reset    — RX overrun sticks until WR0 error reset
//  18.  test_wr0_reset_highest_ius    — WR0 highest-IUS reset clears one source
//  19.  test_reset_irq_safe_defaults  — enabling IRQs while idle raises no IRQ
//  20.  test_wr9_chanB_reset          — chan B reset clears B, leaves A
//  21.  test_wr0_command_decode       — reset-ext and reset-TX commands don't alias
//  22.  test_data_port_preserves_ptr  — idle data reads/writes don't disturb WR0 ptr
//  23.  test_wr0_pointer_is_shared    — Universal Bus WR0 pointer is global
//  24.  test_external_rx_path         — external serial byte source fills RX FIFO
//  25.  test_external_line_status     — CTS/DCD/SYNC pins are reflected in RR0
//  26.  test_external_status_irq      — WR15-gated line changes raise ext IRQ
//  27.  test_tx_output_pins           — TX valid/data pulses after byte time
//  28.  test_modem_control_outputs    — WR5 RTS/DTR drive active-low pins
//  29.  test_rom_probe_init_rr0       — Q700 ROM init sequence leaves TX empty
//  30.  test_rom_probe_rr1_residue    — Q700 ROM RR1 reads see reset residue code
//  31.  test_rom_probe_rr15_reset      — Q700 ROM sees MAME/Z8530 WR15 reset state
//  32.  test_dead_slot_writes_inert    — dead WR slots don't perturb live state
//  33.  test_tx_multibyte_poll         — ROM-style 4-byte TX with poll-before-write
//                                         (Z85C30 1-byte hold, NOT a 4-byte FIFO —
//                                         see scc.v header / MAME z80scc.cpp:1051)
//  34.  test_brg_reload_boundary       — BRG reload formula is TC+2 per
//                                         MAME z80scc.cpp:2787 (was TC+1, off-by-one)
//  35a. test_rx_fifo_push_pop_same_cycle_a — chan A: CPU pop + RX push on
//                                         the same clock edge net correctly
//                                         (T4 bug 1: level/slot collision)
//  35b. test_rx_fifo_push_pop_same_cycle_b — same collision, chan B
//  36.  test_wr0_reset_highest_ius_via_chanB — RESET_HIGHEST_IUS issued via
//                                         chan B still clears chan-A source
//                                         first (T4 bug 2: daisy-chain order)
//  37.  test_sdlc_lapenq_localtalk_polled — replays the exact register
//                                         sequence the ROM .MPP driver runs
//                                         for LocalTalk lapENQ node-address
//                                         acquisition (captured from a MAME
//                                         macqd700 golden boot via Lua bus
//                                         tap, 2026-07-22).  Locks down the
//                                         SYNC/HUNT latch (RR0 bit4 = 0x54
//                                         idle value) that the driver's
//                                         carrier-sense loop branches on
//                                         (btst #4), polled TBE progression
//                                         through a 3-byte SDLC frame with
//                                         WR0 CRC-command no-ops, and that
//                                         no IRQ fires anywhere (MAME
//                                         completes acquisition with zero
//                                         SCC interrupts).
//
// "Corner I almost got wrong": WR0 pointer management.  The pointer
// resets to 0 after the TARGETED WRn write completes — not after the
// WR0 pointer-set write.  scenario 2 locks that down, and scenario 3
// stress-tests it across the whole bank.
//
// Address encoding (matches scc.v):
//   pb_addr[0] = 0 → control port, 1 → data port
//   pb_addr[1] = 0 → channel B,     1 → channel A
// So: 0 = B/ctrl, 1 = B/data, 2 = A/ctrl, 3 = A/data.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vscc.h"

static Vscc*    dut       = nullptr;
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
    dut->pb_addr   = 0;
    dut->pb_wdata  = 0;
    dut->pb_wr     = 0;
    dut->pb_rd     = 0;
    dut->rx_a_valid = 0;
    dut->rx_a_data  = 0;
    dut->rx_b_valid = 0;
    dut->rx_b_data  = 0;
    dut->cts_a_n    = 0;
    dut->dcd_a_n    = 0;
    dut->sync_a_n   = 1;
    dut->cts_b_n    = 0;
    dut->dcd_b_n    = 0;
    dut->sync_b_n   = 1;
    tick(); tick(); tick();
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
    if (!(cond)) { printf("  FAIL %s\n", name); return false; } \
} while(0)

// ── Bus helpers ────────────────────────────────────────────────────────
// Writes pulse pb_wr for one cycle; ack arrives next cycle.  We tick
// twice after the pulse to let the pipeline settle — that mirrors how
// peripheral_bus drives the scc (single-cycle strobe, then idle).
static void bus_write(uint8_t addr, uint8_t data) {
    dut->pb_addr  = addr & 0xF;
    dut->pb_wdata = data;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    tick();
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
    tick();
}

static uint8_t bus_read(uint8_t addr) {
    dut->pb_addr = addr & 0xF;
    dut->pb_rd   = 1;
    dut->pb_wr   = 0;
    tick();
    dut->pb_rd   = 0;
    dut->eval();
    uint8_t d = dut->pb_rdata & 0xFF;
    tick();     // let side-effects (ptr reset, FIFO pop) settle
    return d;
}

// Idle for N core clocks.
static void idle(uint32_t n) {
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->rx_a_valid = 0;
    dut->rx_b_valid = 0;
    for (uint32_t i = 0; i < n; i++) tick();
}

static void set_no_cable_lines() {
    dut->cts_a_n  = 1;
    dut->dcd_a_n  = 1;
    dut->sync_a_n = 1;
    dut->cts_b_n  = 1;
    dut->dcd_b_n  = 1;
    dut->sync_b_n = 1;
    idle(2);
}

static bool wait_tx_a(uint8_t& data, uint32_t timeout) {
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->rx_a_valid = 0;
    dut->rx_b_valid = 0;
    for (uint32_t i = 0; i < timeout; i++) {
        tick();
        if (dut->tx_a_valid) {
            data = dut->tx_a_data & 0xFF;
            return true;
        }
    }
    return false;
}

static bool wait_tx_b(uint8_t& data, uint32_t timeout) {
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->rx_a_valid = 0;
    dut->rx_b_valid = 0;
    for (uint32_t i = 0; i < timeout; i++) {
        tick();
        if (dut->tx_b_valid) {
            data = dut->tx_b_data & 0xFF;
            return true;
        }
    }
    return false;
}

static void inject_rx_a(uint8_t data) {
    dut->rx_a_data = data;
    dut->rx_a_valid = 1;
    tick();
    dut->rx_a_valid = 0;
}

static void inject_rx_b(uint8_t data) {
    dut->rx_b_data = data;
    dut->rx_b_valid = 1;
    tick();
    dut->rx_b_valid = 0;
}

static void set_a_lines(uint8_t cts_n, uint8_t dcd_n, uint8_t sync_n) {
    dut->cts_a_n = cts_n;
    dut->dcd_a_n = dcd_n;
    dut->sync_a_n = sync_n;
    tick();
}

// ── Address shortcuts ──────────────────────────────────────────────────
static const uint8_t B_CTRL = 0;
static const uint8_t B_DATA = 1;
static const uint8_t A_CTRL = 2;
static const uint8_t A_DATA = 3;

// WR0 pointer-set shortcut: writes WR0 with low 3 bits = n for n < 8,
// OR with command 001 (point high) combined with bits [2:0] = n-8 for
// n >= 8.  This uses the single-write form of the datasheet: a WR0
// write with bits [5:3] = 001 and bits [2:0] = Dn latches the pointer
// to 8 + Dn for the next access.
static void set_ptr_a(uint8_t n) {
    if (n < 8) {
        bus_write(A_CTRL, n & 0x7);
    } else {
        bus_write(A_CTRL, (0x1 << 3) | (n & 0x7));
    }
}
static void set_ptr_b(uint8_t n) {
    if (n < 8) {
        bus_write(B_CTRL, n & 0x7);
    } else {
        bus_write(B_CTRL, (0x1 << 3) | (n & 0x7));
    }
}

// Write WRn on channel A.
static void wra(uint8_t n, uint8_t v) {
    if (n == 0) {
        bus_write(A_CTRL, v);
    } else {
        set_ptr_a(n);
        bus_write(A_CTRL, v);
    }
}
static void wrb(uint8_t n, uint8_t v) {
    if (n == 0) {
        bus_write(B_CTRL, v);
    } else {
        set_ptr_b(n);
        bus_write(B_CTRL, v);
    }
}

// Read RRn on channel A.
static uint8_t rra(uint8_t n) {
    if (n == 0) return bus_read(A_CTRL);
    set_ptr_a(n);
    return bus_read(A_CTRL);
}
static uint8_t rrb(uint8_t n) {
    if (n == 0) return bus_read(B_CTRL);
    set_ptr_b(n);
    return bus_read(B_CTRL);
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 1 — reset values
// ════════════════════════════════════════════════════════════════════════
static bool test_reset_clean() {
    reset();
    // RR0 after reset: TX buffer empty + DCD + CTS stubs + TX under-run.
    // Expected = 0x6C:  bit 2 TX_EMPTY = 1, bit 3 DCD = 1, bit 5 CTS = 1,
    // bit 6 TX_UR = 1.
    uint8_t s = rra(0);
    CHECK_EQ("RR0 reset A", s, 0x6C);
    s = rrb(0);
    CHECK_EQ("RR0 reset B", s, 0x6C);
    CHECK_EQ("WR2 reset A", rra(2), 0x00);
    CHECK_EQ("WR2 reset B", rrb(2), 0x00);
    CHECK_EQ("WR9 reset A", rra(9), 0x00);
    CHECK_EQ("WR9 reset B", rrb(9), 0x00);
    CHECK_EQ("WR12 reset A", rra(12), 0x00);
    CHECK_EQ("WR12 reset B", rrb(12), 0x00);
    CHECK_EQ("WR15 reset A", rra(15), 0xF8);
    CHECK_EQ("WR15 reset B", rrb(15), 0xF8);
    CHECK_TRUE("irq low at reset", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 2 — WR0 pointer + pointer-resets-after-WRn
//
// WR12 has a readback at RR12 per the datasheet; we use it to confirm
// the pointer did advance to a targeted register and back to 0.  WR3 is
// write-only (RR3 is the IP summary) — don't test its readback path.
// ════════════════════════════════════════════════════════════════════════
static bool test_wr0_pointer_semantics() {
    reset();
    // Point at WR12 then write 0x3C → WR12 = 0x3C.
    set_ptr_a(12);
    bus_write(A_CTRL, 0x3C);
    // Pointer should now be 0.  Read RR0 — must reflect idle status.
    uint8_t s = bus_read(A_CTRL);
    CHECK_EQ("RR0 after WR12 write", s, 0x6C);
    // Confirm WR12 really holds 0x3C (via RR12 readback path).
    set_ptr_a(12);
    s = bus_read(A_CTRL);
    CHECK_EQ("WR12 roundtrip", s, 0x3C);
    // Point-high path: set ptr 8, write 0xAB → WR8 = 0xAB.  WR8 write
    // via pointer also loads tx_hold → tx_hold_full=1 → RR0 TX_EMPTY
    // bit must now be 0.
    set_ptr_a(8);
    bus_write(A_CTRL, 0xAB);
    s = bus_read(A_CTRL);
    CHECK_EQ("RR0 after WR8 write (tx_hold_full)", s & 0x04, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 3 — WRn roundtrip (only registers that have a readback path)
//
// Per Zilog datasheet WR↔RR are SEPARATE register files — only WR2,
// WR9, WR12, WR13, WR15 (and some WR4/5 mirrors in our implementation)
// have RR readback routes.  Other registers are exercised via their
// side-effects in the loopback scenarios below.
// ════════════════════════════════════════════════════════════════════════
static bool test_wrn_roundtrip() {
    reset();
    struct Case { uint8_t n; uint8_t v; };
    // WR9 skipped — writes with the top bits set trigger reset commands.
    Case cases[] = {
        {2, 0x44}, {4, 0xC7}, {5, 0x6A},
        {11, 0x22}, {12, 0x55}, {13, 0xCC}, {15, 0xDD},
    };
    for (auto &c : cases) {
        wra(c.n, c.v);
        uint8_t got = rra(c.n);
        uint8_t exp = (c.n == 15) ? (c.v & 0xFA) : c.v;
        if (got != exp) {
            printf("  FAIL WR%u r/t A: got 0x%02x, expected 0x%02x\n",
                    c.n, got, exp);
            return false;
        }
    }
    // WR2 is shared — writing on B must be visible on A's RR2.
    wrb(2, 0x91);
    uint8_t v = rra(2);
    CHECK_EQ("WR2 shared A", v, 0x91);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 4 — RR0 on idle channel reports TX buffer empty
// ════════════════════════════════════════════════════════════════════════
static bool test_rr0_idle_status() {
    reset();
    // No WR8 write yet → TX buffer empty bit must be set (bit 2 = 1).
    uint8_t s = rra(0);
    CHECK_EQ("RR0 bit2 (TX_EMPTY)", s & 0x04, 0x04);
    CHECK_EQ("RR0 bit0 (RX_AVAIL)", s & 0x01, 0x00);
    return true;
}

// ── Shared loopback setup ──────────────────────────────────────────────
// Program chan A for local-loopback with BRG enabled + RX enabled +
// TX enabled.  Time constant kept tiny so byte-time elapses fast.
static void program_chan_a_loopback(uint16_t tc) {
    // WR3 = RX enable
    wra(3, 0x01);
    // WR5 = TX enable (bit 3)
    wra(5, 0x08);
    // WR12/WR13 = time constant
    wra(12, tc & 0xFF);
    wra(13, (tc >> 8) & 0xFF);
    // WR14 = BRG enable (bit 0) + local loopback (bit 4)
    wra(14, 0x11);
    // WR9[3] = MIE on, just in case the test inspects irq
    wra(9, 0x08);
}

// Program chan B identically.
static void program_chan_b_loopback(uint16_t tc) {
    wrb(3, 0x01);
    wrb(5, 0x08);
    wrb(12, tc & 0xFF);
    wrb(13, (tc >> 8) & 0xFF);
    wrb(14, 0x11);
    wra(9, 0x08);   // shared
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 5 — TX drains without loopback and RX stays empty
// ════════════════════════════════════════════════════════════════════════
static bool test_tx_no_external_device() {
    reset();
    // Pure transmit path: BRG + TX enable, but no local loopback and
    // no receive enable.  This models ROM bring-up probing a live SCC
    // before any external serial wiring is attached.
    wra(5, 0x08);      // TX enable
    wra(12, 0x02);
    wra(13, 0x00);
    wra(14, 0x01);     // BRG enable only
    wra(9, 0x08);      // MIE on, though no IRQ should be generated

    bus_write(A_DATA, 0xC3);
    uint8_t s = rra(0);
    CHECK_EQ("RR0 after WR8 accepted by idle TX", s & 0x04, 0x04);
    CHECK_EQ("RR1 after WR8 (ALL_SENT=0)", rra(1) & 0x01, 0x00);

    idle(1200);
    s = rra(0);
    CHECK_EQ("RR0 after transmit completes (TX_EMPTY=1)", s & 0x04, 0x04);
    CHECK_EQ("RR0 after transmit completes (RX_AVAIL=0)", s & 0x01, 0x00);
    CHECK_EQ("RR1 after transmit completes (ALL_SENT=1)", rra(1) & 0x01, 0x01);

    uint8_t d = bus_read(A_DATA);
    CHECK_EQ("RX data absent without loopback", d, 0x00);
    CHECK_TRUE("irq low without TX IE", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 6 — TX → loopback → RX for a single byte on chan A
// ════════════════════════════════════════════════════════════════════════
static bool test_tx_loopback_single() {
    reset();
    program_chan_a_loopback(2);  // tiny time constant
    // Send one byte.
    bus_write(A_DATA, 0x5A);
    // With TX enabled and idle, WR8 immediately loads the shifter, leaving
    // transmit-buffer-empty visible like the SCC8530/MAME model.
    uint8_t s = rra(0);
    CHECK_EQ("RR0 after WR8 accepted by idle TX", s & 0x04, 0x04);
    // Idle long enough for BRG to fire twice (latch + byte complete).
    // PCLK_DIV=54 in default param; tests override to 2 via Verilator
    // tb helper.  We don't override here; but each brg_ref = PCLK_DIV*2
    // cycles, and loading a byte needs 2 brg_ref pulses plus BRG
    // count-down (tc+2 each, per MAME z80scc.cpp:2787).  Give ourselves
    // lots of slack.
    idle(1200);
    // RX FIFO should now have one char.
    s = rra(0);
    CHECK_EQ("RR0 after byte time (RX_AVAIL=1)", s & 0x01, 0x01);
    // Pop it.
    uint8_t d = bus_read(A_DATA);
    CHECK_EQ("RX data byte", d, 0x5A);
    // After pop, RX_AVAIL clears.
    s = rra(0);
    CHECK_EQ("RR0 after pop (RX_AVAIL=0)", s & 0x01, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 7 — pump 3 chars through chan A loopback (FIFO stress)
// ════════════════════════════════════════════════════════════════════════
static bool test_tx_loopback_3char_fifo() {
    reset();
    program_chan_a_loopback(1);
    uint8_t payload[3] = { 0x11, 0x22, 0x33 };
    // Blast three writes, let BRG process each.  We can't push three
    // at once because TX hold is single-buffered — but sending, waiting,
    // sending works.
    for (int i = 0; i < 3; i++) {
        bus_write(A_DATA, payload[i]);
        idle(800);
    }
    // Expect 3 bytes in FIFO; read them and verify ordering.
    for (int i = 0; i < 3; i++) {
        uint8_t s = rra(0);
        CHECK_TRUE("RX_AVAIL during drain", (s & 0x01) != 0);
        uint8_t d = bus_read(A_DATA);
        if (d != payload[i]) {
            printf("  FAIL FIFO byte %d: got 0x%02x, expected 0x%02x\n",
                    i, d, payload[i]);
            return false;
        }
    }
    uint8_t s = rra(0);
    CHECK_EQ("RX empty after drain", s & 0x01, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 8 — RX IRQ raises when byte arrives + WR1/WR9 enabled
// ════════════════════════════════════════════════════════════════════════
static bool test_rx_irq_raises() {
    reset();
    program_chan_a_loopback(1);
    // Enable RX IRQ — WR1[4:3] = 01 (first char)
    wra(1, 0x08);
    CHECK_TRUE("irq low before TX", dut->irq == 0);
    bus_write(A_DATA, 0x77);
    idle(1000);
    // irq should now be high (RX IP + RX IE + MIE).
    dut->eval();
    CHECK_TRUE("irq high on RX arrival", dut->irq == 1);
    // Popping drains the FIFO and clears rx_ip.
    uint8_t d = bus_read(A_DATA);
    CHECK_EQ("RX byte read", d, 0x77);
    dut->eval();
    CHECK_TRUE("irq low after drain", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 9 — TX-empty IRQ fires, clears on next WR8
// ════════════════════════════════════════════════════════════════════════
static bool test_tx_empty_irq() {
    reset();
    program_chan_a_loopback(1);
    // Enable TX IRQ — WR1[1]
    wra(1, 0x02);
    bus_write(A_DATA, 0x88);
    // BRG firing should set tx_ip after the byte leaves tx_hold.
    idle(800);
    dut->eval();
    CHECK_TRUE("irq high (TX empty)", dut->irq == 1);
    // Reset TX IP via WR0 cmd 101 (= 0x28)
    bus_write(A_CTRL, 0x28);
    dut->eval();
    // May still have RX side raising irq — disable RX IE to isolate.
    wra(1, 0x00);   // disable all
    dut->eval();
    CHECK_TRUE("irq low after reset TX IP + IE off", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 10 — BRG divides faster with smaller time constant
// ════════════════════════════════════════════════════════════════════════
static bool test_brg_baud_rate() {
    reset();
    // Drive same byte through chan A with two time constants and count
    // core cycles until RX_AVAIL rises.  Smaller TC → fewer cycles.
    uint32_t t_fast = 0, t_slow = 0;

    // Fast: tc = 1.
    program_chan_a_loopback(1);
    bus_write(A_DATA, 0xA5);
    for (uint32_t c = 0; c < 5000; c++) {
        uint8_t s = rra(0);
        if (s & 0x01) { t_fast = c; break; }
    }
    CHECK_TRUE("fast byte arrived", t_fast != 0);
    // Drain.
    (void)bus_read(A_DATA);
    idle(40);

    // Reset and try slow: tc = 20.  Need a full reset so BRG state is
    // clean.
    reset();
    program_chan_a_loopback(20);
    bus_write(A_DATA, 0xA5);
    for (uint32_t c = 0; c < 30000; c++) {
        uint8_t s = rra(0);
        if (s & 0x01) { t_slow = c; break; }
    }
    CHECK_TRUE("slow byte arrived", t_slow != 0);
    // The slow one should need strictly more cycles than the fast one.
    if (t_slow <= t_fast) {
        printf("  FAIL BRG scaling: fast=%u slow=%u (slow should be bigger)\n",
                t_fast, t_slow);
        return false;
    }
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 11 — MIE gates the irq output regardless of IPs
// ════════════════════════════════════════════════════════════════════════
static bool test_master_ie_gate() {
    reset();
    program_chan_a_loopback(1);
    wra(1, 0x08);               // RX IE on
    // First prove the RX IRQ path fires normally (MIE already on from
    // program_chan_a_loopback).
    bus_write(A_DATA, 0x10);
    idle(800);
    dut->eval();
    CHECK_TRUE("irq high baseline", dut->irq == 1);
    // Disable MIE — irq must drop.
    wra(9, 0x00);
    dut->eval();
    CHECK_TRUE("irq low when MIE=0", dut->irq == 0);
    // Re-enable MIE — irq returns (rx_ip still latched).
    wra(9, 0x08);
    dut->eval();
    CHECK_TRUE("irq re-asserts when MIE=1", dut->irq == 1);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 12 — channel A and channel B are independent
// ════════════════════════════════════════════════════════════════════════
static bool test_chan_a_vs_b_isolation() {
    reset();
    program_chan_b_loopback(1);
    wrb(1, 0x08);
    // Send a byte on B — it must land in B's FIFO, not A's.
    bus_write(B_DATA, 0xBB);
    idle(800);
    uint8_t sb = rrb(0);
    uint8_t sa = rra(0);
    CHECK_EQ("B RX_AVAIL=1", sb & 0x01, 0x01);
    CHECK_EQ("A RX_AVAIL=0", sa & 0x01, 0x00);
    // Drain B and verify isolation from the other direction too.
    uint8_t d = bus_read(B_DATA);
    CHECK_EQ("B data", d, 0xBB);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 13 — WR9[7:6]=10 clears chan A only
// ════════════════════════════════════════════════════════════════════════
static bool test_wr9_chanA_reset() {
    reset();
    // Load some state into both channels.
    wra(4, 0xAB);
    wrb(4, 0xCD);
    // Chan A reset (WR9[7:6]=10, low bits = MIE=1 to keep enables sane)
    wra(9, 0x88);
    // WR4 on A should be 0 now; WR4 on B should survive.
    uint8_t va = rra(4);
    uint8_t vb = rrb(4);
    CHECK_EQ("A WR4 cleared", va, 0x00);
    CHECK_EQ("B WR4 survives", vb, 0xCD);
    CHECK_EQ("WR9 low bits survive A reset", rra(9) & 0x3F, 0x08);
    CHECK_EQ("WR9 low bits mirror on B", rrb(9) & 0x3F, 0x08);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 14 — WR9[7:6]=11 hardware reset clears both channels
// ════════════════════════════════════════════════════════════════════════
static bool test_wr9_hardware_reset() {
    reset();
    wra(4, 0x11);
    wrb(4, 0x22);
    wra(12, 0xFF);
    wrb(12, 0xEE);
    // Hardware reset
    wra(9, 0xCA);
    // Everything cleared.
    CHECK_EQ("A WR4 cleared", rra(4),  0x00);
    CHECK_EQ("B WR4 cleared", rrb(4),  0x00);
    CHECK_EQ("A WR12 cleared", rra(12), 0x00);
    CHECK_EQ("B WR12 cleared", rrb(12), 0x00);
    // WR9[7:6] self-cleared, while the programmed control bits survive.
    CHECK_EQ("A WR9[7:6] self-clear", rra(9) & 0xC0, 0x00);
    CHECK_EQ("A WR9 low bits survive hw reset", rra(9) & 0x3F, 0x0A);
    CHECK_EQ("B WR9 low bits survive hw reset", rrb(9) & 0x3F, 0x0A);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 15 — RR3 on channel A reflects both channels' IPs
// ════════════════════════════════════════════════════════════════════════
static bool test_rr3_ip_summary() {
    reset();
    // Enable RX-IE + MIE on BOTH channels, and arm both loopbacks.
    program_chan_a_loopback(1);
    program_chan_b_loopback(1);
    wra(1, 0x08);
    wrb(1, 0x08);
    // Drive a byte on B first — RR3 bit 2 (ChB rx_ip) should set.
    bus_write(B_DATA, 0xEE);
    idle(800);
    uint8_t r3 = rra(3);
    CHECK_TRUE("RR3 ChB rx_ip set", (r3 & 0x04) != 0);
    // Drive A too — RR3 bit 5 (ChA rx_ip) should also set.
    bus_write(A_DATA, 0xFF);
    idle(800);
    r3 = rra(3);
    CHECK_TRUE("RR3 ChA rx_ip set", (r3 & 0x20) != 0);
    CHECK_TRUE("RR3 both chan IPs set", (r3 & 0x24) == 0x24);
    // Drain B, re-check: B should clear, A should stay.
    (void)bus_read(B_DATA);
    r3 = rra(3);
    CHECK_EQ("RR3 ChB rx_ip cleared", r3 & 0x04, 0x00);
    CHECK_TRUE("RR3 ChA rx_ip survives", (r3 & 0x20) != 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 16 — channel B decode isolation + RR2 stable-vector handling
// ════════════════════════════════════════════════════════════════════════
static bool test_chan_b_decode_and_rr2() {
    reset();
    // A/B control registers must not alias.
    wra(12, 0x11);
    wrb(12, 0x22);
    CHECK_EQ("A WR12 isolated", rra(12), 0x11);
    CHECK_EQ("B WR12 isolated", rrb(12), 0x22);
    CHECK_EQ("A WR12 survives B write", rra(12), 0x11);

    // RR2 should preserve the shared vector even while an interrupt is
    // pending.  RR3 carries the pending-source summary.
    wra(2, 0xA0);
    CHECK_EQ("RR2 A baseline", rra(2), 0xA0);
    CHECK_EQ("RR2 B baseline", rrb(2), 0xA0);

    program_chan_b_loopback(1);
    wrb(1, 0x08);   // RX IE
    bus_write(B_DATA, 0x5A);
    idle(800);

    uint8_t vec_a = rra(2);
    uint8_t vec_b = rrb(2);
    CHECK_EQ("RR2 A unchanged", vec_a, 0xA0);
    CHECK_EQ("RR2 B unchanged", vec_b, 0xA0);

    // Drain B to leave the channel quiet for any follow-on reads.
    uint8_t d = bus_read(B_DATA);
    CHECK_EQ("B RX byte", d, 0x5A);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 17 — RR1 overrun sticks until WR0 error reset
// ════════════════════════════════════════════════════════════════════════
static bool test_rr1_overrun_and_reset() {
    reset();
    program_chan_a_loopback(1);
    wra(1, 0x08);   // RX IE so the path is exercised with IRQs enabled

    // Push four bytes without draining the FIFO; the fourth should set
    // the overrun latch.  Poll RR0 so the test tracks visible state
    // rather than a guessed byte-time.
    uint8_t payloads[4] = { 0x10, 0x20, 0x30, 0x40 };
    for (int i = 0; i < 4; i++) {
        bus_write(A_DATA, payloads[i]);
        if (i < 3) {
            bool settled = false;
            for (uint32_t c = 0; c < 20000; c++) {
                uint8_t s = rra(0);
                if ((s & 0x05) == 0x05) {
                    settled = true;
                    break;
                }
            }
            CHECK_TRUE("RR0 reports RX data after loopback", settled);
        } else {
            bool ov = false;
            for (uint32_t c = 0; c < 50000; c++) {
                if (rra(1) & 0x20) {
                    ov = true;
                    break;
                }
            }
            CHECK_TRUE("RR1 overrun set", ov);
        }
    }

    // Error reset clears RR1[5] without disturbing the FIFO contents.
    bus_write(A_CTRL, 0x30);   // WR0 command 110 = error reset
    CHECK_EQ("RR1 overrun cleared", rra(1) & 0x20, 0x00);
    CHECK_EQ("RR0 still reports RX data", rra(0) & 0x01, 0x01);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 18 — WR0 reset highest IUS clears one source at a time
// ════════════════════════════════════════════════════════════════════════
static bool test_wr0_reset_highest_ius() {
    reset();
    program_chan_a_loopback(1);
    wra(1, 0x0A);   // RX IE + TX IE

    // Generate both TX and RX pending conditions.
    bus_write(A_DATA, 0x55);
    idle(800);
    uint8_t r3 = rra(3);
    CHECK_TRUE("RR3 both A sources pending", (r3 & 0x30) == 0x30);
    CHECK_TRUE("irq high before reset highest IUS", dut->irq == 1);

    // First reset should clear the highest-priority A source (RX).
    bus_write(A_CTRL, 0x38);   // WR0 command 111 = reset highest IUS
    r3 = rra(3);
    CHECK_EQ("RR3 after first highest-IUS reset", r3 & 0x30, 0x10);
    CHECK_TRUE("irq remains high with TX pending", dut->irq == 1);

    // Second reset clears the remaining TX source.
    bus_write(A_CTRL, 0x38);
    r3 = rra(3);
    CHECK_EQ("RR3 after second highest-IUS reset", r3 & 0x30, 0x00);
    CHECK_TRUE("irq low after all sources cleared", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 19 — reset/idle state stays interrupt-safe when software
// enables the SCC before any serial device is attached.
// ════════════════════════════════════════════════════════════════════════
static bool test_reset_irq_safe_defaults() {
    reset();

    // Enable every modeled per-channel interrupt source, then turn on
    // master interrupt enable.  No RX byte, TX-empty transition, or
    // external-status transition has occurred, so the IRQ line must stay
    // low even though RR0 reports both transmitters idle/empty.
    wra(1, 0x1B);
    wrb(1, 0x1B);
    wra(9, 0x08);
    dut->eval();
    CHECK_TRUE("irq low after idle enables", dut->irq == 0);
    CHECK_EQ("RR3 no pending sources", rra(3), 0x00);
    CHECK_EQ("A idle TX/RX status", rra(0), 0x6C);
    CHECK_EQ("B idle TX/RX status", rrb(0), 0x6C);
    CHECK_EQ("A empty RX data read", bus_read(A_DATA), 0x00);
    CHECK_EQ("B empty RX data read", bus_read(B_DATA), 0x00);
    dut->eval();
    CHECK_TRUE("irq remains low after empty RX reads", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 20 — WR9[7:6]=01 clears chan B only
// ════════════════════════════════════════════════════════════════════════
static bool test_wr9_chanB_reset() {
    reset();
    // Load visible and queued state into both channels.
    wra(4, 0x55);
    wrb(4, 0xAA);
    wra(12, 0x11);
    wrb(12, 0x22);
    bus_write(A_DATA, 0xA5);
    bus_write(B_DATA, 0x5A);

    // Chan B reset (WR9[7:6]=01) with MIE preserved in the low bits.
    wra(9, 0x48);

    CHECK_EQ("A WR4 survives B reset", rra(4), 0x55);
    CHECK_EQ("A WR12 survives B reset", rra(12), 0x11);
    CHECK_EQ("A TX hold survives B reset", rra(0) & 0x04, 0x00);
    CHECK_EQ("B WR4 cleared", rrb(4), 0x00);
    CHECK_EQ("B WR12 cleared", rrb(12), 0x00);
    CHECK_EQ("B TX hold cleared", rrb(0) & 0x04, 0x04);
    CHECK_EQ("WR9 low bits survive B reset A", rra(9) & 0x3F, 0x08);
    CHECK_EQ("WR9 low bits survive B reset B", rrb(9) & 0x3F, 0x08);
    CHECK_EQ("WR9 reset command self-clear A", rra(9) & 0xC0, 0x00);
    CHECK_EQ("WR9 reset command self-clear B", rrb(9) & 0xC0, 0x00);
    CHECK_TRUE("irq low after B reset", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 21 — WR0 command decode keeps reset-ext/status and reset-TX
// interrupt-pending commands distinct.
// ════════════════════════════════════════════════════════════════════════
static bool test_wr0_command_decode() {
    reset();
    // Generate a TX-empty interrupt without RX loopback noise.
    wra(5, 0x08);      // TX enable
    wra(12, 0x01);
    wra(13, 0x00);
    wra(14, 0x01);     // BRG enable only
    wra(1, 0x02);      // TX IE
    wra(9, 0x08);      // MIE

    bus_write(A_DATA, 0x4A);
    idle(800);
    dut->eval();
    CHECK_TRUE("irq high before WR0 command", dut->irq == 1);
    CHECK_TRUE("RR3 A TX pending before command", (rra(3) & 0x10) != 0);

    // 0x10 is reset external/status, not reset TX IP.  With no external
    // status source modeled, it must leave TX pending untouched.
    bus_write(A_CTRL, 0x10);
    dut->eval();
    CHECK_TRUE("irq stays high after reset-ext", dut->irq == 1);
    CHECK_TRUE("RR3 A TX still pending after reset-ext", (rra(3) & 0x10) != 0);

    // 0x28 is the actual reset-TX-IP command.
    bus_write(A_CTRL, 0x28);
    dut->eval();
    CHECK_TRUE("irq low after reset-TX-IP", dut->irq == 0);
    CHECK_EQ("RR3 A TX cleared", rra(3) & 0x10, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 22 — data-port access is direct WR8/RR8 access and does not
// disturb the pending control-port register pointer.
// ════════════════════════════════════════════════════════════════════════
static bool test_data_port_preserves_ptr() {
    reset();

    set_ptr_a(12);
    CHECK_EQ("empty A data read deterministic", bus_read(A_DATA), 0x00);
    bus_write(A_CTRL, 0x34);
    CHECK_EQ("A WR12 after empty data read", rra(12), 0x34);

    set_ptr_b(13);
    bus_write(B_DATA, 0xA6);
    bus_write(B_CTRL, 0x56);
    CHECK_EQ("B WR13 after data write", rrb(13), 0x56);

    set_ptr_b(12);
    CHECK_EQ("empty B data read deterministic", bus_read(B_DATA), 0x00);
    bus_write(B_CTRL, 0x78);
    CHECK_EQ("B WR12 after empty data read", rrb(12), 0x78);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 23 — the WR0 pointer is shared across both channels
//
// MAME's Universal Bus SCC model stores the selected register pointer in
// z80scc_device::m_wr0_ptrbits, not in each channel.  A pointer selected
// through one channel therefore targets the next control access on either
// channel.
// ════════════════════════════════════════════════════════════════════════
static bool test_wr0_pointer_is_shared() {
    reset();

    set_ptr_a(12);
    bus_write(B_CTRL, 0x9A);
    CHECK_EQ("A-selected ptr writes B WR12", rrb(12), 0x9A);

    set_ptr_b(13);
    bus_write(A_CTRL, 0xBC);
    CHECK_EQ("B-selected ptr writes A WR13", rra(13), 0xBC);

    set_ptr_a(12);
    uint8_t s = bus_read(B_CTRL);
    CHECK_EQ("A-selected ptr reads B RR12", s, 0x9A);
    CHECK_EQ("shared ptr clears after cross-channel read", bus_read(A_CTRL), 0x6C);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 24 — external RX byte source fills the receive FIFO
// ════════════════════════════════════════════════════════════════════════
static bool test_external_rx_path() {
    reset();

    inject_rx_a(0x11);
    CHECK_EQ("external RX ignored while disabled", rra(0) & 0x01, 0x00);

    wra(3, 0x01);                    // RX enable
    inject_rx_a(0x42);
    CHECK_EQ("A RR0 RX available", rra(0) & 0x01, 0x01);
    CHECK_EQ("A external RX byte", bus_read(A_DATA), 0x42);
    CHECK_EQ("A RR0 RX clears after read", rra(0) & 0x01, 0x00);

    wrb(3, 0x01);
    wrb(1, 0x08);                    // RX interrupt enable
    wra(9, 0x08);                    // master interrupt enable
    inject_rx_b(0x73);
    dut->eval();
    CHECK_TRUE("B external RX raises IRQ", dut->irq == 1);
    CHECK_EQ("B external RX byte", bus_read(B_DATA), 0x73);
    dut->eval();
    CHECK_TRUE("B external RX IRQ clears after drain", dut->irq == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 25 — external line status pins are reflected in RR0
// ════════════════════════════════════════════════════════════════════════
static bool test_external_line_status() {
    reset();

    CHECK_EQ("A idle DCD/CTS high, sync low", rra(0) & 0x38, 0x28);

    dut->cts_a_n = 1;
    dut->dcd_a_n = 1;
    dut->sync_a_n = 0;
    tick();
    CHECK_EQ("A inactive DCD/CTS and active sync", rra(0) & 0x38, 0x10);

    dut->cts_b_n = 1;
    dut->dcd_b_n = 0;
    dut->sync_b_n = 0;
    tick();
    CHECK_EQ("B line status", rrb(0) & 0x38, 0x18);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 26 — WR15-gated external/status interrupts
// ════════════════════════════════════════════════════════════════════════
static bool test_external_status_irq() {
    reset();

    wra(1, 0x01);    // external/status interrupt enable
    wra(15, 0x20);   // CTS interrupt enable only
    wra(9, 0x08);    // master interrupt enable
    dut->eval();
    CHECK_TRUE("ext irq low before line change", dut->irq == 0);

    set_a_lines(1, 0, 1); // CTS deasserts
    dut->eval();
    CHECK_TRUE("ext irq high after CTS transition", dut->irq == 1);
    CHECK_EQ("RR3 A ext pending", rra(3) & 0x08, 0x08);
    bus_write(A_CTRL, 0x10); // WR0 reset external/status interrupts
    dut->eval();
    CHECK_TRUE("ext irq clears on reset-ext command", dut->irq == 0);
    CHECK_EQ("RR3 A ext clears", rra(3) & 0x08, 0x00);

    reset();
    wra(1, 0x01);
    wra(15, 0x00);   // line changes not armed in WR15
    wra(9, 0x08);
    set_a_lines(1, 0, 1);
    dut->eval();
    CHECK_TRUE("WR15 masks external line changes", dut->irq == 0);
    CHECK_EQ("RR3 no masked ext pending", rra(3) & 0x08, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 27 — TX completion pins expose completed bytes
// ════════════════════════════════════════════════════════════════════════
static bool test_tx_output_pins() {
    reset();

    wra(5, 0x08);      // TX enable, no local loopback
    wra(12, 0x01);
    wra(13, 0x00);
    wra(14, 0x01);     // BRG enable
    bus_write(A_DATA, 0x5A);

    uint8_t data = 0;
    CHECK_TRUE("channel A TX valid pulses", wait_tx_a(data, 1200));
    CHECK_EQ("channel A TX byte", data, 0x5A);
    CHECK_TRUE("channel B TX stays idle", dut->tx_b_valid == 0);

    reset();
    wrb(5, 0x08);
    wrb(12, 0x01);
    wrb(13, 0x00);
    wrb(14, 0x01);
    bus_write(B_DATA, 0xA6);

    data = 0;
    CHECK_TRUE("channel B TX valid pulses", wait_tx_b(data, 1200));
    CHECK_EQ("channel B TX byte", data, 0xA6);
    CHECK_TRUE("channel A TX stays idle", dut->tx_a_valid == 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 28 — WR5 drives active-low RTS/DTR modem outputs
// ════════════════════════════════════════════════════════════════════════
static bool test_modem_control_outputs() {
    reset();
    CHECK_EQ("A RTS default deasserted", dut->rts_a_n, 1);
    CHECK_EQ("A DTR default deasserted", dut->dtr_a_n, 1);
    CHECK_EQ("B RTS default deasserted", dut->rts_b_n, 1);
    CHECK_EQ("B DTR default deasserted", dut->dtr_b_n, 1);

    wra(5, 0x82); // DTR + RTS, TX disabled
    CHECK_EQ("A RTS asserted low", dut->rts_a_n, 0);
    CHECK_EQ("A DTR asserted low", dut->dtr_a_n, 0);
    CHECK_EQ("B RTS unaffected", dut->rts_b_n, 1);
    CHECK_EQ("B DTR unaffected", dut->dtr_b_n, 1);

    wrb(5, 0x82);
    CHECK_EQ("B RTS asserted low", dut->rts_b_n, 0);
    CHECK_EQ("B DTR asserted low", dut->dtr_b_n, 0);

    wra(9, 0x80); // channel A reset
    tick();
    CHECK_EQ("A RTS deasserts after channel reset", dut->rts_a_n, 1);
    CHECK_EQ("A DTR deasserts after channel reset", dut->dtr_a_n, 1);
    CHECK_EQ("B RTS survives A reset", dut->rts_b_n, 0);
    CHECK_EQ("B DTR survives A reset", dut->dtr_b_n, 0);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 29 — Q700 ROM SCC init leaves RR0 TX-empty visible
//
// This replays the first Channel A initialization block seen in the
// MAME-driven Q700 ROM trace.  The ROM polls RR0[2] before writing the first
// data byte, so no WR8/data side effect should have made TX buffer empty low.
// ════════════════════════════════════════════════════════════════════════
static bool test_rom_probe_init_rr0() {
    reset();
    set_no_cable_lines();

    CHECK_EQ("ROM initial RR0", bus_read(A_CTRL), 0x44);
    wra(9, 0xC0);
    CHECK_EQ("ROM RR0 after WR9 reset", bus_read(A_CTRL), 0x44);

    wra(4, 0x4C);
    wra(11, 0x50);
    wra(15, 0x00);
    wra(12, 0x00);
    wra(13, 0x00);
    wra(14, 0x01);
    wra(3, 0xC1);
    wra(5, 0x6A);
    CHECK_EQ("ROM RR0 after WR5 0x6a", bus_read(A_CTRL), 0x44);

    wra(14, 0x11);
    wra(5, 0x68);
    CHECK_EQ("ROM empty data read", bus_read(A_DATA), 0x00);
    bus_write(A_CTRL, 0x30);
    CHECK_EQ("ROM poll RR0 TX empty", bus_read(A_CTRL), 0x44);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 30 — Q700 ROM SCC RR1 residue-code reset value
//
// The Q700 ROM writes WR0=1 and then reads RR1 while checking serial state.
// The Z85C30 reset value keeps RR1[2:1] high (0x06), and MAME's z80scc
// returns 0x07 when ALL_SENT is also true.  Returning only 0x01 diverges from
// native MAME and changes the ROM's SCC timing path.
// ════════════════════════════════════════════════════════════════════════
static bool test_rom_probe_rr1_residue() {
    reset();
    set_no_cable_lines();

    CHECK_EQ("ROM initial RR1", rra(1), 0x07);

    wra(5, 0x68);
    wra(12, 0x02);
    wra(13, 0x00);
    wra(14, 0x01);
    bus_write(A_DATA, 0x00);
    CHECK_EQ("ROM RR1 while TX active", rra(1), 0x06);

    idle(1200);
    bus_write(A_CTRL, 0x01);
    CHECK_EQ("ROM indexed RR1 after TX completes", bus_read(A_CTRL), 0x07);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 31 — Q700 ROM SCC RR15 reset/readback path
//
// Native MAME initializes the 8530 external/status enable register to 0xf8.
// The ROM selects RR15 with WR0=0x0f while probing serial state; matching that
// reset image avoids a platform-lockstep divergence at PC 0x40847958.
// ════════════════════════════════════════════════════════════════════════
static bool test_rom_probe_rr15_reset() {
    reset();
    set_no_cable_lines();

    bus_write(A_CTRL, 0x0F);
    CHECK_EQ("ROM indexed RR15 reset", bus_read(A_CTRL), 0xF8);

    wra(15, 0xFF);
    CHECK_EQ("RR15 masks unused bits", rra(15), 0xFA);

    wra(9, 0x80);
    tick();
    CHECK_EQ("RR15 after channel reset", rra(15), 0xF8);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 32 — Z85C30 dead-slot writes (WR0/WR6/WR7/WR10) preserve
// observable state when no command/side-effect bit is set.  This pins
// the dead-storage gate (live_wr_slot) to the documented spec subset:
// only command effects fire on those slots; nothing latches a value.
// ════════════════════════════════════════════════════════════════════════
static bool test_dead_slot_writes_inert() {
    reset();
    set_no_cable_lines();

    // Snapshot RR0 / RR1 before touching dead slots.
    uint8_t rr0_before = rra(0);
    uint8_t rr1_before = rra(1);

    // WR6 / WR7 / WR10 are dead-by-design.  Use the pointer mechanism
    // (WR0=ptr) then write the target.  The write must NOT perturb RR0
    // or RR1 — those reflect TX/RX live state.
    bus_write(A_CTRL, 0x06);            // ptr <- 6
    bus_write(A_CTRL, 0xAA);            // WR6 <- 0xAA (no-op)
    CHECK_EQ("RR0 unchanged after WR6", rra(0), rr0_before);

    bus_write(A_CTRL, 0x07);            // ptr <- 7
    bus_write(A_CTRL, 0x55);            // WR7 <- 0x55 (no-op)
    CHECK_EQ("RR0 unchanged after WR7", rra(0), rr0_before);

    bus_write(A_CTRL, 0x0A);            // ptr <- 10
    bus_write(A_CTRL, 0xFF);            // WR10 <- 0xFF (no-op)
    CHECK_EQ("RR0 unchanged after WR10", rra(0), rr0_before);
    CHECK_EQ("RR1 unchanged after WR10", rra(1), rr1_before);

    // WR0 with command bits = 0 should also be a no-op (only sets ptr=0).
    bus_write(A_CTRL, 0x00);
    CHECK_EQ("RR0 unchanged after WR0 cmd=0", rra(0), rr0_before);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 33 — multi-byte TX with poll-before-write protocol
//
// The Z85C30 has a 1-byte TX hold (NOT a 4-byte FIFO; that is an ESCC
// variant feature — see MAME z80scc.cpp:1051 and the scc.v header).
// ROM software follows the documented protocol: poll RR0[2]
// (TX_BUF_EMPTY) before each WR8 to avoid clobbering an in-flight byte.
// This scenario walks four bytes through that path and verifies every
// byte appears on the tx_a_data wire in order.  If we ever silently
// drop a byte (e.g. a regression that turns the hold into level-driven
// instead of edge-loaded), this fails.
// ════════════════════════════════════════════════════════════════════════
// Holder for tx_a_valid samples observed across this scenario.  Filled
// by the helpers below so we never miss a 1-cycle pulse, even when the
// pulse falls inside a bus_write/bus_read cycle.
static uint8_t s_obs_tx[8];
static int     s_obs_tx_n;

static void capture_tx_a() {
    if (dut->tx_a_valid && s_obs_tx_n < 8) {
        s_obs_tx[s_obs_tx_n++] = dut->tx_a_data & 0xFF;
    }
}

// Wrappers around tick / bus ops that sample tx_a_valid every cycle.
static void tick_capture() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
    capture_tx_a();
}

static void bus_write_capture(uint8_t addr, uint8_t data) {
    dut->pb_addr  = addr & 0xF;
    dut->pb_wdata = data;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    tick_capture();
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
    tick_capture();
}

static uint8_t bus_read_capture(uint8_t addr) {
    dut->pb_addr = addr & 0xF;
    dut->pb_rd   = 1;
    dut->pb_wr   = 0;
    tick_capture();
    dut->pb_rd   = 0;
    dut->eval();
    uint8_t d = dut->pb_rdata & 0xFF;
    tick_capture();
    return d;
}

static bool test_tx_multibyte_poll() {
    reset();
    set_no_cable_lines();
    s_obs_tx_n = 0;

    wra(5, 0x08);      // TX enable, no loopback
    wra(12, 0x02);     // small but non-trivial TC so the poll loop runs
    wra(13, 0x00);
    wra(14, 0x01);     // BRG enable

    const uint8_t payload[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

    for (int i = 0; i < 4; i++) {
        // Poll RR0[2] until TX buffer reports empty (matches ROM
        // protocol: write only when the chip has accepted the prior byte).
        bool ready = false;
        for (uint32_t c = 0; c < 20000; c++) {
            if ((bus_read_capture(2 /* A_CTRL */) & 0x04) != 0) {
                ready = true; break;
            }
        }
        CHECK_TRUE("TX_BUF_EMPTY raises before next WR8", ready);
        bus_write_capture(A_DATA, payload[i]);
    }
    // Drain trailing in-flight byte.  Plain ticks here; sim_time grows
    // freely.  Ceiling sized for TC=2, frame_clocks=10, PCLK_DIV=2.
    for (uint32_t c = 0; c < 40000 && s_obs_tx_n < 4; c++) {
        tick_capture();
    }
    CHECK_EQ("all 4 TX bytes observed", s_obs_tx_n, 4);
    for (int i = 0; i < 4; i++) {
        if (s_obs_tx[i] != payload[i]) {
            printf("  FAIL TX byte %d: got 0x%02x, expected 0x%02x\n",
                    i, s_obs_tx[i], payload[i]);
            return false;
        }
    }
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 34 — BRG reload formula is TC+2 (per MAME z80scc.cpp:2787)
//
// This pins the off-by-one fix.  We measure how many core cycles a
// loopback byte takes for two adjacent time-constants (TC=0 and TC=2)
// and check that the increment matches the +2 formula.
//
// At TC=N the BRG reloads to N+2 brg_ref ticks per output bit.  The
// frame is 10 bit-times in x1 mode (default, WR4=0), so byte time
// scales as (N+2)*10 brg_refs * (PCLK_DIV*2) core cycles.  With
// PCLK_DIV=2 (tb override): brg_ref every 4 core cycles.
//
// Going from TC=0 → TC=2 should add ((4-2)/2) ≈ 2× brg_refs per bit, or
// roughly 80 extra core cycles total.  We don't pin the exact count
// (loopback push and pointer accesses add jitter), but we DO pin the
// monotonicity AND the rough proportionality: a TC=0 run must be
// distinctly faster than a TC=2 run, which in turn must be distinctly
// faster than a TC=10 run.  If someone accidentally reverts to TC+1,
// TC=0 collapses to a bit-period of 1, which is observably faster than
// the +2 path — but the relative ordering still holds.  The boundary
// that ONLY the +2 form satisfies is the absolute floor: with TC=0 and
// PCLK_DIV=2, byte-time must be at least 2*10*4 = 80 core cycles
// (cannot be 40, which is what TC+1 would give for TC=0).
// ════════════════════════════════════════════════════════════════════════
static bool test_brg_reload_boundary() {
    auto measure_byte_time = [](uint16_t tc) -> uint32_t {
        reset();
        program_chan_a_loopback(tc);
        uint32_t cycles_at_tx_start = (uint32_t)sim_time;
        bus_write(A_DATA, 0xA5);
        for (uint32_t c = 0; c < 100000; c++) {
            uint8_t s = rra(0);
            if (s & 0x01) {
                return ((uint32_t)sim_time) - cycles_at_tx_start;
            }
        }
        return 0;
    };

    uint32_t t0  = measure_byte_time(0);
    uint32_t t2  = measure_byte_time(2);
    uint32_t t10 = measure_byte_time(10);

    CHECK_TRUE("TC=0 byte arrived",  t0  != 0);
    CHECK_TRUE("TC=2 byte arrived",  t2  != 0);
    CHECK_TRUE("TC=10 byte arrived", t10 != 0);

    // Monotonic: bigger TC → strictly more cycles.
    if (!(t0 < t2 && t2 < t10)) {
        printf("  FAIL BRG monotonic: t0=%u t2=%u t10=%u\n", t0, t2, t10);
        return false;
    }

    // Absolute floor: TC=0 with TC+2 formula must be at least
    // 2 brg_ref ticks per bit * 10 bits * 4 core cycles per brg_ref =
    // 80 cycles.  TC+1 would give 40 cycles, which is below this floor.
    // sim_time advances by 2 per tick() (rising + falling edge), so the
    // raw delta is in half-cycles; floor in raw delta is 160.
    if (t0 < 160) {
        printf("  FAIL BRG TC=0 too fast: t0=%u (TC+1 regression?)\n", t0);
        return false;
    }
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 35 — same-cycle RX FIFO push + CPU pop nets correctly (T4 bug 1)
// ════════════════════════════════════════════════════════════════════════
// "Corner almost gotten wrong": a CPU data-port pop (rd_a/rd_b) and an RX
// push (loopback or external) both wrote rxlvl_x/rxf_x* in the SAME
// always block with no netting — the push (scheduled later in program
// order) simply clobbered the pop's non-blocking assignment, so level
// over-counted by +1 and the pushed byte landed at the PRE-pop slot
// instead of the correct post-pop tail.  This drives a CPU read of
// A_DATA and an external rx_a_valid push on the IDENTICAL clock edge
// and checks the netted result on both channels.
static bool test_rx_fifo_push_pop_same_cycle_a() {
    reset();
    wra(3, 0x01);          // RX enable — gates the external push path
    inject_rx_a(0x11);     // level 0 -> 1  (rxf_a0 = 0x11)
    inject_rx_a(0x22);     // level 1 -> 2  (rxf_a0=0x11, rxf_a1=0x22)
    uint8_t s = rra(0);
    CHECK_TRUE("RX_AVAIL before same-cycle push+pop", (s & 0x01) != 0);

    // Drive a CPU data-port read (pop) and an external RX byte (push)
    // on the SAME rising edge.
    dut->pb_addr    = A_DATA;
    dut->pb_rd      = 1;
    dut->pb_wr      = 0;
    dut->rx_a_data  = 0x33;
    dut->rx_a_valid = 1;
    tick();
    dut->pb_rd      = 0;
    dut->rx_a_valid = 0;
    tick();     // let side effects settle, mirrors bus_read()

    // Level must net to unchanged (still 2): one popped, one pushed.
    s = rra(0);
    CHECK_TRUE("RX_AVAIL after same-cycle push+pop", (s & 0x01) != 0);

    // 0x11 (the byte being popped) must be gone; 0x22 must survive at
    // the head; 0x33 (the colliding push) must land at the correct
    // post-pop tail slot — draining now must yield exactly 0x22, 0x33,
    // then empty (NOT 0x11 still present, and NOT a stuck level=3).
    uint8_t d0 = bus_read(A_DATA);
    CHECK_EQ("collision: 0x22 survives at head, 0x11 popped", d0, 0x22);
    uint8_t d1 = bus_read(A_DATA);
    CHECK_EQ("collision: pushed 0x33 lands at post-pop tail", d1, 0x33);
    s = rra(0);
    CHECK_EQ("RX empty after draining exactly 2 (level netted, not +1)",
             s & 0x01, 0x00);
    return true;
}

// Same collision, channel B, to confirm the fix (per-channel netting)
// isn't accidentally A-only.
static bool test_rx_fifo_push_pop_same_cycle_b() {
    reset();
    wrb(3, 0x01);
    inject_rx_b(0xA1);
    inject_rx_b(0xA2);
    uint8_t s = rrb(0);
    CHECK_TRUE("RX_AVAIL before same-cycle push+pop (B)", (s & 0x01) != 0);

    dut->pb_addr    = B_DATA;
    dut->pb_rd      = 1;
    dut->pb_wr      = 0;
    dut->rx_b_data  = 0xA3;
    dut->rx_b_valid = 1;
    tick();
    dut->pb_rd      = 0;
    dut->rx_b_valid = 0;
    tick();

    s = rrb(0);
    CHECK_TRUE("RX_AVAIL after same-cycle push+pop (B)", (s & 0x01) != 0);
    uint8_t d0 = bus_read(B_DATA);
    CHECK_EQ("collision B: 0xA2 survives at head, 0xA1 popped", d0, 0xA2);
    uint8_t d1 = bus_read(B_DATA);
    CHECK_EQ("collision B: pushed 0xA3 lands at post-pop tail", d1, 0xA3);
    s = rrb(0);
    CHECK_EQ("RX empty after draining exactly 2 (B, level netted)",
             s & 0x01, 0x00);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 36 — RESET_HIGHEST_IUS issued via channel B honours the fixed
// Z85C30 daisy-chain order RxA>TxA>ExtA>RxB>TxB>ExtB (T4 bug 2)
// ════════════════════════════════════════════════════════════════════════
// "Corner almost gotten wrong": the channel-B WR0 command arm cleared
// its OWN three sources (RxB/TxB/ExtB) before falling back to chan A —
// i.e. B-first order when the command happened to arrive on channel B's
// control port.  The real chip's IUS daisy chain is fixed regardless of
// which port carried the command.  This pends both channels and issues
// the command through B, checking that the chan-A source clears first.
static bool test_wr0_reset_highest_ius_via_chanB() {
    reset();
    program_chan_a_loopback(1);
    program_chan_b_loopback(1);
    wra(1, 0x0A);   // RX IE + TX IE, chan A
    wrb(1, 0x0A);   // RX IE + TX IE, chan B

    // Pend RX + TX interrupts on BOTH channels.
    bus_write(A_DATA, 0x55);
    idle(800);
    bus_write(B_DATA, 0x66);
    idle(800);

    uint8_t r3 = rra(3);
    CHECK_EQ("RR3: rx_ip_a, tx_ip_a, rx_ip_b, tx_ip_b all pending",
             r3 & 0x36, 0x36);

    // Issue RESET_HIGHEST_IUS (WR0 cmd 111 = 0x38) via channel B's
    // control port.  Fixed daisy-chain order must clear rx_ip_a FIRST,
    // even though the command arrived on B.
    bus_write(B_CTRL, 0x38);
    r3 = rra(3);
    CHECK_EQ("A-side rx_ip clears first even via chan-B command",
             r3 & 0x36, 0x16);   // rx_ip_a (bit5) cleared; rest survive
    CHECK_TRUE("irq remains high, other sources still pending", dut->irq == 1);

    // Next reset (still via B) clears tx_ip_a next, per the fixed order
    // — NOT rx_ip_b (which the pre-fix B-first order would have hit).
    bus_write(B_CTRL, 0x38);
    r3 = rra(3);
    CHECK_EQ("tx_ip_a clears next (not rx_ip_b) via chan-B command",
             r3 & 0x36, 0x06);   // tx_ip_a (bit4) cleared; rx_ip_b/tx_ip_b survive
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

// ════════════════════════════════════════════════════════════════════════
// Scenario 37 — LocalTalk lapENQ node-address acquisition, MAME-golden
//
// Register-level replay of the ROM .MPP driver's SDLC transmit sequence
// on channel B, captured from a working MAME macqd700 System 7.0.1 boot
// (Lua write/read tap on 0x50f0c020-26, sim-time 7.492s window).  The
// golden trace shows the whole acquisition is POLLED — RR0 idle value
// 0x54 (TX-underrun/EOM + SYNC/HUNT + TX-empty), TBE-driven byte pacing,
// and ZERO SCC interrupts across the entire window.  The carrier-sense
// loop in the driver does `btst #4,(RR0)` and defers forever if
// SYNC/HUNT reads 0 — the pre-fix behavior with the board's /SYNC pin
// tied high (welcome-screen AppleTalk hang).
// ════════════════════════════════════════════════════════════════════════
static bool poll_rrb0_until(uint8_t want_set_mask, uint8_t& last,
                            uint32_t max_polls) {
    for (uint32_t i = 0; i < max_polls; i++) {
        last = rrb(0);
        if (last & want_set_mask) return true;
    }
    return false;
}

static bool test_sdlc_lapenq_localtalk_polled() {
    reset();
    set_no_cable_lines();   // board reality: CTS/DCD/SYNC all tied high

    // ── .MPP init block (MAME trace 7.492005s, pc=0x72c54..0x72c58) ──
    wrb(9, 0x40);           // channel B reset
    wrb(4, 0x20);           // SDLC mode, x1 clock, sync-mode enable
    wrb(10, 0xe0);          // CRC preset / NRZI / mark idle (dead slot)
    wrb(6, 0x00);           // SDLC address (dead slot)
    wrb(7, 0x7e);           // SDLC flag (dead slot)
    wrb(12, 0x06);          // TC low = 6 → 230.4 kbit LocalTalk rate
    wrb(13, 0x00);          // TC high
    wrb(14, 0xc0);          // DPLL cmd, BRG still off
    wrb(3, 0xdd);           // RX enable + ENTER HUNT (bit4) → hunt := 1
    wrb(2, 0x00);           // vector
    wrb(15, 0x08);          // ext-status IE mask: DCD only
    wrb(1, 0x09);           // ext IE + RX-int-first-char; TX IE OFF
    wrb(9, 0x0a);           // MIE + NV
    wrb(11, 0x70);          // RX clock = DPLL, TX clock = BRG
    wrb(14, 0x21);          // DPLL search mode + BRG enable
    wrb(5, 0x60);           // TX 8-bit, TX still disabled
    wrb(6, 0x01);           // node-id candidate 1

    // Golden idle value: 0x54 = underrun/EOM(6) + SYNC/HUNT(4) + TBE(2).
    CHECK_EQ("RR0 idle after MPP init (hunt latched)", rrb(0), 0x54);
    CHECK_TRUE("no irq after init", dut->irq == 0);

    for (int frame = 0; frame < 2; frame++) {
        // ── carrier-sense loop (trace pc=0x72d16/0x72d1c ×10) ──
        // Driver does btst #4 on each read and bails to a defer path
        // (which this board can never resume from) if hunt reads 0.
        for (int i = 0; i < 10; i++) {
            uint8_t s = rrb(0);
            if (s != 0x54) {
                printf("  FAIL carrier-sense RR0 iter %d frame %d: got 0x%02x, expected 0x54\n",
                       i, frame, s);
                return false;
            }
            wrb(14, 0x41);  // reset missing clock + BRG enable
        }
        CHECK_EQ("RR10 (loop/clock status stub)", rrb(10), 0x00);
        wrb(5, 0x62);       // RTS jiggle (trace pc=0x72d38..0x72d50)
        wrb(5, 0x60);
        wrb(5, 0x6b);       // TX ENABLE + RTS
        wrb(3, 0xd0);       // RX off during TX — hunt must STAY set
        bus_write(B_CTRL, 0x80);   // WR0: Reset TX CRC — no-op, no state damage
        CHECK_EQ("RR0 after TX arm + CRC reset", rrb(0), 0x54);

        // ── 3-byte lapENQ frame: dst=0x01 src=0x01 type=0x81 ──
        bus_write(B_DATA, 0x01);
        CHECK_EQ("RR0 after byte1 (write-through, TBE=1)", rrb(0), 0x54);
        bus_write(B_DATA, 0x01);
        CHECK_EQ("RR0 after byte2 (hold full, TBE=0)", rrb(0), 0x50);
        uint8_t s = 0;
        CHECK_TRUE("TBE returns within a byte-time (byte1 done)",
                   poll_rrb0_until(0x04, s, 200));
        bus_write(B_DATA, 0x81);
        bus_write(B_CTRL, 0xc0);   // WR0: Reset TX underrun/EOM — no-op
        CHECK_TRUE("TBE returns within a byte-time (byte2 done)",
                   poll_rrb0_until(0x04, s, 200));

        // ── teardown (trace pc=0x72fb8..0x72fd0) ──
        wrb(5, 0x62);
        wrb(5, 0x60);       // TX disabled while byte3 still shifting
        wrb(14, 0x41);
        wrb(3, 0xdd);       // RX re-enable + ENTER HUNT again

        // Byte 3 must still drain out of the shifter (tx pulse = 0x81).
        uint8_t txd = 0;
        CHECK_TRUE("byte3 tx pulse observed", wait_tx_b(txd, 2000));
        CHECK_EQ("byte3 tx data", txd, 0x81);

        CHECK_EQ("RR0 back to idle after frame", rrb(0), 0x54);
        // Channel-B IP bits (RR3[2:0] = rx/tx/ext) must stay clear — the
        // whole acquisition is polled.  Chan A's ext-IP bit is excluded:
        // this tb's own set_no_cable_lines() pin flip latches it (WR15
        // reset 0xF8 enables DCD/CTS ext sources; harness noise, masked
        // by WR1 ext-IE=0, unrelated to the lapENQ contract on chan B).
        CHECK_EQ("RR3 chan-B IPs stay clear (polled protocol)", rra(3) & 0x07, 0x00);
        CHECK_TRUE("no irq at any point (MAME: zero SCC ints)", dut->irq == 0);
        CHECK_EQ("RX FIFO stays empty (frame is sunk)", rrb(0) & 0x01, 0x00);
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vscc;

    RUN(test_reset_clean);
    RUN(test_wr0_pointer_semantics);
    RUN(test_wrn_roundtrip);
    RUN(test_rr0_idle_status);
    RUN(test_tx_no_external_device);
    RUN(test_tx_loopback_single);
    RUN(test_tx_loopback_3char_fifo);
    RUN(test_rx_irq_raises);
    RUN(test_tx_empty_irq);
    RUN(test_brg_baud_rate);
    RUN(test_master_ie_gate);
    RUN(test_chan_a_vs_b_isolation);
    RUN(test_wr9_chanA_reset);
    RUN(test_wr9_hardware_reset);
    RUN(test_rr3_ip_summary);
    RUN(test_chan_b_decode_and_rr2);
    RUN(test_rr1_overrun_and_reset);
    RUN(test_wr0_reset_highest_ius);
    RUN(test_reset_irq_safe_defaults);
    RUN(test_wr9_chanB_reset);
    RUN(test_wr0_command_decode);
    RUN(test_data_port_preserves_ptr);
    RUN(test_wr0_pointer_is_shared);
    RUN(test_external_rx_path);
    RUN(test_external_line_status);
    RUN(test_external_status_irq);
    RUN(test_tx_output_pins);
    RUN(test_modem_control_outputs);
    RUN(test_rom_probe_init_rr0);
    RUN(test_rom_probe_rr1_residue);
    RUN(test_rom_probe_rr15_reset);
    RUN(test_dead_slot_writes_inert);
    RUN(test_tx_multibyte_poll);
    RUN(test_brg_reload_boundary);
    RUN(test_rx_fifo_push_pop_same_cycle_a);
    RUN(test_rx_fifo_push_pop_same_cycle_b);
    RUN(test_wr0_reset_highest_ius_via_chanB);
    RUN(test_sdlc_lapenq_localtalk_polled);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
