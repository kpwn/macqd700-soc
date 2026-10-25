// EASC register and CD-XA behavior exercised by the Quadra 700 ROM.
// The reference decoder below follows MAME asc_easc_device::decode_cdxa.
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vasc.h"

static Vasc dut;
static void tick() { dut.clk = 0; dut.eval(); dut.clk = 1; dut.eval(); }
static void write(uint16_t addr, uint8_t value) {
    dut.pb_addr = addr; dut.pb_wdata = value; dut.pb_wr = 1; tick();
    dut.pb_wr = 0; dut.pb_wdata = 0;
}
static uint8_t read(uint16_t addr) {
    dut.pb_addr = addr; dut.pb_rd = 1; tick(); dut.pb_rd = 0; dut.eval();
    return dut.pb_rdata;
}
static bool eq(const char *name, int got, int expected) {
    if (got == expected) return true;
    std::printf("FAIL %s: got %d expected %d\n", name, got, expected);
    return false;
}
static int16_t signed16(uint16_t x) { return static_cast<int16_t>(x); }

struct Decoder {
    int s0 = 0, s1 = 0;
    unsigned pos = 0, subpos = 0, at = 0;
    uint8_t param = 0, byte = 0;
    std::array<int8_t, 8> coeff = {};
    int16_t next(const std::vector<uint8_t> &stream) {
        if (pos == 0) { param = stream.at(at++); subpos = 0; }
        const unsigned filter = (param >> 4) & 3;
        const unsigned shift = std::min<unsigned>(param & 15, 12);
        int raw;
        if (subpos == 0) {
            byte = stream.at(at++);
            raw = static_cast<int16_t>((byte & 15) << 12);
            subpos = 1;
        } else {
            raw = static_cast<int16_t>(((byte >> 4) & 15) << 12);
            subpos = 0;
        }
        const int prediction = (coeff[filter * 2 + 1] * s0
                              + coeff[filter * 2] * s1 + 32) >> 6;
        const int sample = std::clamp((raw >> shift) + prediction,
                                      -32768, 32767);
        s1 = s0; s0 = sample;
        pos = (pos + 1) % 28;
        return static_cast<int16_t>(sample);
    }
};

int main() {
    dut.rst = 1; tick(); tick(); dut.rst = 0; tick();
    if (!eq("version", read(0x800), 0xb0)) return 1;
    if (!eq("reset mode", read(0x801), 0)) return 1;
    if (!eq("clock", read(0x807), 3)) return 1;
    dut.phi2_tick = 1; tick(); dut.phi2_tick = 0; tick();
    if (!eq("idle half-full B", read(0x804) & 4, 4)) return 1;
    write(0x801, 1);
    if (!eq("ROM FIFO service prelude", read(0x804) & 0x0c, 0x0c)) return 1;
    write(0x808, 1); // one sample per phi2 pulse
    // Two ADPCM blocks per channel.  Filter 1 exercises both predictor
    // multipliers; the ROM programs its coefficients through F11/F13.
    std::vector<uint8_t> stream;
    for (int block = 0; block < 2; ++block) {
        stream.push_back(static_cast<uint8_t>(0x14 + block));
        for (int i = 0; i < 14; ++i)
            stream.push_back(static_cast<uint8_t>(0x31 + i * 17 + block));
    }
    for (uint8_t v : stream) { write(0x000, v); write(0x400, v); }
    write(0xf12, 0xe0); write(0xf13, 0x3c);
    write(0xf32, 0xe0); write(0xf33, 0x3c);
    if (!eq("coefficient readback", read(0xf13), 0x3c)) return 1;
    write(0xf08, 0x02); write(0xf28, 0x02);
    Decoder model; model.coeff[2] = -32; model.coeff[3] = 60;
    for (int i = 0; i < 56; ++i) {
        dut.phi2_tick = 1; tick(); dut.phi2_tick = 0; tick();
        const int16_t want = model.next(stream);
        if (!dut.audio_sample_valid || signed16(dut.audio_pcm_l) != want
                || signed16(dut.audio_pcm_r) != want) {
            std::printf("FAIL sample %d: valid=%u left=%d right=%d expected=%d\n",
                i, dut.audio_sample_valid, signed16(dut.audio_pcm_l),
                signed16(dut.audio_pcm_r), want);
            return 1;
        }
    }
    if (!eq("FIFO A read pointer", read(0xf03), 30)) return 1;
    if (!eq("FIFO B read pointer", read(0xf23), 30)) return 1;

    // The Sound Manager also uses ordinary unsigned 8-bit FIFO PCM.
    // EASC's two output channels must remain independent even though
    // R_CONTROL is read-only and its stereo bit is never set.
    dut.rst = 1; tick(); tick(); dut.rst = 0; tick();
    write(0x801, 1); write(0x808, 1);
    write(0xf09, 0); write(0xf29, 0);
    const std::array<uint8_t, 4> left = {0x80, 0x90, 0x10, 0xff};
    const std::array<uint8_t, 4> right = {0x80, 0x70, 0xf0, 0x00};
    for (unsigned i = 0; i < left.size(); ++i) {
        write(0x000, left[i]); write(0x400, right[i]);
    }
    for (unsigned i = 0; i < left.size(); ++i) {
        dut.phi2_tick = 1; tick(); dut.phi2_tick = 0; tick();
        const int16_t want_left = (static_cast<int>(left[i]) - 128) * 256;
        const int16_t want_right = (static_cast<int>(right[i]) - 128) * 256;
        if (!dut.audio_sample_valid || signed16(dut.audio_pcm_l) != want_left
                || signed16(dut.audio_pcm_r) != want_right) {
            std::printf("FAIL PCM %u: valid=%u left=%d right=%d expected=%d,%d\n",
                i, dut.audio_sample_valid, signed16(dut.audio_pcm_l),
                signed16(dut.audio_pcm_r), want_left, want_right);
            return 1;
        }
    }
    if (!eq("PCM IRQ", dut.irq, 1)) return 1;
    if (!eq("PCM half-full status", read(0x804) & 5, 5)) return 1;

    // SRC 0x7fff means a 0x8000 step: one FIFO byte every two output
    // samples.  Check both sample hold and byte consumption.
    dut.rst = 1; tick(); tick(); dut.rst = 0; tick();
    write(0x801, 1); write(0x808, 1);
    write(0x000, 0x90); write(0x000, 0xa0);
    write(0x400, 0x70); write(0x400, 0x60);
    write(0xf04, 0x7f); write(0xf05, 0xff);
    write(0xf24, 0x7f); write(0xf25, 0xff);
    write(0xf08, 0x80); write(0xf28, 0x80);
    const std::array<int, 4> src_left = {0, 4096, 4096, 8192};
    const std::array<int, 4> src_right = {0, -4096, -4096, -8192};
    for (unsigned i = 0; i < src_left.size(); ++i) {
        dut.phi2_tick = 1; tick(); dut.phi2_tick = 0; tick();
        if (signed16(dut.audio_pcm_l) != src_left[i]
                || signed16(dut.audio_pcm_r) != src_right[i]) {
            std::printf("FAIL SRC %u: left=%d right=%d expected=%d,%d\n",
                i, signed16(dut.audio_pcm_l), signed16(dut.audio_pcm_r),
                src_left[i], src_right[i]);
            return 1;
        }
    }
    if (!eq("SRC FIFO A pointer", read(0xf03), 2)) return 1;
    if (!eq("SRC FIFO B pointer", read(0xf23), 2)) return 1;

    // A two-byte block-start pop crosses the half-full boundary on its
    // second byte: MAME raises the A/B half bits immediately at cap 511.
    dut.rst = 1; tick(); tick(); dut.rst = 0; tick();
    write(0x801, 1); write(0x808, 1);
    for (int i = 0; i < 512; ++i) {
        write(0x000, i == 0 ? 0x00 : 0x11);
        write(0x400, i == 0 ? 0x00 : 0x11);
    }
    write(0xf09, 0); write(0xf29, 0);
    write(0xf08, 0x02); write(0xf28, 0x02);
    dut.phi2_tick = 1; tick(); dut.phi2_tick = 0; tick();
    if (!eq("CD-XA half-full on second pop", read(0x804) & 5, 5)) return 1;
    if (!eq("CD-XA half-full IRQ", dut.irq, 1)) return 1;
    std::puts("PASS EASC CD-XA mode 2, stereo PCM/IRQ, and SRC pacing");
    return 0;
}
