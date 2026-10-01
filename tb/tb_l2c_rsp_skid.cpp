#include "Vl2c_rsp_skid.h"
#include "verilated.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>

#ifndef FALL_THROUGH_BUILD
#define FALL_THROUGH_BUILD 0
#endif

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vl2c_rsp_skid d;
    std::deque<uint32_t> expected;
    bool stalled = false;
    uint32_t held = 0;
    unsigned cycles = 0, accepted = 0, delivered = 0, full = 0, bypassed = 0;
    auto require = [&](bool ok, const char *why) {
        if (!ok) { std::fprintf(stderr, "FAIL cycle %u: %s\n", cycles, why); std::exit(1); }
    };
    auto step = [&](bool valid, uint32_t data, bool ready, bool reset = false) {
        d.clk = 0; d.rst = reset; d.s_valid = valid; d.s_data = data; d.m_ready = ready; d.eval();
        const bool sr = d.s_ready;
        // The upstream acceptance cone must be independent of downstream ready.
        d.m_ready = !ready; d.eval();
        require(bool(d.s_ready) == sr, "downstream ready reached upstream ready");
        d.m_ready = ready; d.eval();
        if (!reset) {
            const bool want_valid = !expected.empty() || (FALL_THROUGH_BUILD && valid);
            require(bool(d.m_valid) == want_valid, "output valid differs from FIFO/bypass contract");
            require(sr == (expected.size() < 2), "upstream ready differs from two-slot capacity");
            if (!sr) full++;
            if (stalled) require(d.m_valid && d.m_data == held, "refused response changed payload");
            const bool was_empty = expected.empty();
            if (valid && sr) { expected.push_back(data); accepted++; }
            if (d.m_valid) {
                require(!expected.empty(), "unexpected response");
                require(d.m_data == expected.front(), "lost/reordered/corrupted response");
                if (ready) { expected.pop_front(); delivered++; if (was_empty) bypassed++; }
            }
            stalled = d.m_valid && !ready;
            held = d.m_data;
        } else { expected.clear(); stalled = false; }
        d.clk = 1; d.eval(); d.clk = 0; d.eval(); cycles++;
        return valid && sr && !reset;
    };
    step(false, 0, false, true);
    // Empty pass, empty refusal, fill, full refusal, drain, simultaneous refill.
    step(true, 0x12345678, true);
    step(false, 0, true);
    step(true, 0xabcdef01, false);
    step(true, 0x76543210, false);
    step(false, 0, false);
    step(false, 0, true);
    step(true, 0xdeadbeef, true);
    step(false, 0, true);
    step(false, 0, true);
    for (unsigned i = 0; i < 100; i++) require(step(true, i, true), "unstalled input bubble");
    step(false, 0, true);
    // Reset discards buffered beats without leaking them into the next stream.
    step(true, 0x11111111, false); step(true, 0x22222222, false);
    step(false, 0, false, true); step(false, 0, true);
    std::mt19937 rng(0x68040);
    bool pending = false; uint32_t payload = 0;
    for (unsigned i = 0; i < 20000; i++) {
        if (!pending && rng() % 4) { pending = true; payload = rng(); }
        if (step(pending, payload, rng() % 3 != 0)) pending = false;
    }
    while (pending) if (step(true, payload, true)) pending = false;
    while (!expected.empty()) step(false, 0, true);
    require(accepted == delivered + 2, "reset loss count differs from two buffered beats");
    require(full > 0, "full-buffer path not exercised");
    require(!FALL_THROUGH_BUILD || bypassed > 100, "empty forwarding not exercised");
    std::printf("PASS ft=%d cycles=%u accepted=%u delivered=%u full=%u bypassed=%u (two beats discarded by reset)\n",
                FALL_THROUGH_BUILD, cycles, accepted, delivered, full, bypassed);
    d.final();
}
