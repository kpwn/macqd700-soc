#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Vasc.h"

static Vasc dut;

static void tick() {
    dut.clk = 0; dut.eval();
    dut.clk = 1; dut.eval();
}

static void reset() {
    dut.rst = 1;
    dut.phi2_tick = 0;
    dut.pb_wr = 0;
    dut.pb_rd = 0;
    dut.pb_addr = 0;
    dut.pb_wdata = 0;
    tick(); tick();
    dut.rst = 0;
    tick();
}

static void write8(uint16_t addr, uint8_t data) {
    dut.pb_addr = addr;
    dut.pb_wdata = data;
    dut.pb_wr = 1;
    tick();
    dut.pb_wr = 0;
}

static uint8_t read8(uint16_t addr) {
    dut.pb_addr = addr;
    dut.pb_rd = 1;
    tick();
    dut.pb_rd = 0;
    dut.eval();
    return dut.pb_rdata;
}

static uint16_t read_a_rp() {
    uint16_t hi = read8(0xF02);
    return ((hi << 8) | read8(0xF03)) & 0x3ff;
}

static int run(uint32_t clocks) {
    int valid = 0;
    dut.pb_rd = dut.pb_wr = 0;
    dut.phi2_tick = 0;
    for (uint32_t i = 0; i < clocks; ++i) {
        tick();
        valid += dut.audio_sample_valid != 0;
    }
    return valid;
}

static bool check(const char *what, int actual, int lo, int hi) {
    if (actual >= lo && actual <= hi) return true;
    std::printf("FAIL %s: %d (expected %d..%d)\n", what, actual, lo, hi);
    return false;
}

int main() {
    bool ok = true;

    reset();
    write8(0x80A, 1);                 // record mode: suppress Sonora A level IRQ
    for (int i = 0; i < 800; ++i) write8(0x000, uint8_t(i));
    const uint16_t normal_before = read_a_rp();
    const int normal_valid = run(10000);
    const uint16_t normal_after = read_a_rp();
    // Exactly 441 output ticks in 10,000 clocks at 1 MHz.  The readback
    // edges can contribute one additional pop at an interval boundary.
    ok &= check("44.1 kHz FIFO output", normal_valid, 441, 441);
    ok &= check("FIFO bytes consumed", (normal_after - normal_before) & 0x3ff,
                441, 442);

    reset();
    write8(0x80A, 1);
    for (int i = 0; i < 800; ++i) write8(0x000, uint8_t(i));
    write8(0xF04, 0x7f);
    write8(0xF05, 0xff);              // step 0x8000 = one byte per two ticks
    write8(0xF08, 0x80);              // enable SRC on A only
    const uint16_t half_before = read_a_rp();
    const int half_valid = run(10000);
    const uint16_t half_after = read_a_rp();
    ok &= check("SRC keeps output at 44.1 kHz", half_valid, 441, 441);
    ok &= check("SRC consumes half the bytes", (half_after - half_before) & 0x3ff,
                220, 222);

    reset();
    write8(0x80A, 1);
    for (int i = 0; i < 650; ++i) write8(0x000, uint8_t(i));
    write8(0xF04, 0x7f);
    write8(0xF05, 0xff);
    write8(0xF08, 0x80);
    run(10000);
    ok &= check("half-empty completion flag after SRC playback",
                read8(0x810) & 0x80, 0x80, 0x80);
    ok &= check("half-empty completion flag acknowledges",
                read8(0x810) & 0x80, 0, 0);

    reset();
    write8(0xF04, 0x00);
    write8(0xF05, 0x00);
    write8(0xF08, 0x80);              // one byte per 65536 output ticks
    run(10000);
    ok &= check("SRC hold is not FIFO underflow", read8(0x810) & 0x10, 0, 0);

    reset();
    write8(0x80A, 1);
    run(100);
    ok &= check("half-empty FIFO asserts service IRQ", dut.irq, 1, 1);
    ok &= check("FIFO status reports service condition", read8(0x804) & 0x04,
                0x04, 0x04);
    // MAME EASC/Sonora retains IRQ while HALF_FULL_B is set.  Software
    // disables that channel's IRQ before acknowledging the next batch.
    ok &= check("804 read retains asserted half-empty IRQ", dut.irq, 1, 1);
    write8(0xF29, 1);
    ok &= check("IRQ control masks and clears service IRQ", dut.irq, 0, 0);

    reset();
    write8(0x801, 2);                // ROM wavetable mode
    ok &= check("wave mode ignores FIFO clock", run(10000), 0, 0);
    int wave_valid = 0;
    for (int i = 0; i < 35; ++i) {
        dut.phi2_tick = 1;
        tick();
        wave_valid += dut.audio_sample_valid != 0;
        dut.phi2_tick = 0;
        tick();
        wave_valid += dut.audio_sample_valid != 0;
    }
    ok &= check("wave mode uses phi2 divider", wave_valid, 1, 1);

    std::printf("ASC FIFO rate/SRC: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
