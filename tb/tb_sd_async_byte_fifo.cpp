#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <verilated.h>
#include "Vsd_async_byte_fifo.h"

static Vsd_async_byte_fifo dut;
static uint64_t cycles = 0;
static std::deque<uint8_t> expected;
static int sent = 0;
static int received = 0;

static bool step(bool offer, uint8_t byte, bool consume) {
    dut.wr_clk = 0;
    dut.rd_clk = 0;
    dut.wr_en = 0;
    dut.rd_en = 0;
    dut.eval();
    const bool pb_edge = !(cycles & 1);
    const bool write = offer && !dut.wr_full;
    const bool read = pb_edge && consume && !dut.rd_empty;
    const uint8_t got = dut.rd_data;
    dut.wr_en = write;
    dut.wr_data = byte;
    dut.rd_en = read;
    dut.eval();
    if (read) {
        if (expected.empty() || got != expected.front()) {
            std::fprintf(stderr, "FIFO order error at cycle %llu: got %02x expected %02x\n",
                         (unsigned long long)cycles, got,
                         expected.empty() ? 0xff : expected.front());
            return false;
        }
        expected.pop_front();
        ++received;
    }
    if (write) {
        expected.push_back(byte);
        ++sent;
    }
    dut.wr_clk = 1;
    if (pb_edge) dut.rd_clk = 1;
    dut.eval();
    ++cycles;
    return true;
}

static void reset() {
    dut.wr_clk = 0;
    dut.rd_clk = 0;
    dut.arst = 1;
    dut.eval();
    expected.clear();
    for (int i = 0; i < 8; ++i) {
        if (!step(false, 0, false)) std::abort();
    }
    dut.arst = 0;
    dut.eval();
    for (int i = 0; i < 8; ++i) {
        if (!step(false, 0, false)) std::abort();
    }
}

static bool stream(int count, int spacing, bool stall_reader) {
    const int start_sent = sent;
    const int start_recv = received;
    for (int i = 0; i < 200000 && received - start_recv < count; ++i) {
        const int offered = sent - start_sent;
        const bool offer = offered < count && (spacing == 1 || (i % spacing) == 0);
        const bool consume = !stall_reader || (i % 17) < 11;
        if (!step(offer, uint8_t(offered ^ (offered >> 8)), consume)) return false;
    }
    if (sent - start_sent != count || received - start_recv != count || !expected.empty()) {
        std::fprintf(stderr, "FIFO count mismatch: sent=%d received=%d expected_queue=%zu\n",
                     sent - start_sent, received - start_recv, expected.size());
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    reset();
    // 100 MHz source, 50 MHz sink: a byte every four source clocks is the
    // four-bit 50 MHz wire rate. Also exercise back-to-back offers and
    // reader stalls, where the FIFO must apply real backpressure.
    if (!stream(4096, 4, false)) return 1;
    if (!stream(4096, 1, true)) return 1;
    // A shared asynchronous reset flushes an interrupted transaction.
    for (int i = 0; i < 20; ++i)
        if (!step(true, uint8_t(i), false)) return 1;
    reset();
    if (!stream(4096, 4, true)) return 1;
    std::printf("PASS native SD async FIFO: 12288 ordered bytes, wire-rate, stalls, reset\n");
    return 0;
}
