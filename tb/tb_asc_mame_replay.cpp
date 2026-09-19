// tb_asc_mame_replay.cpp — drive Verilator-compiled rtl/mac/asc.v with a
// MAME-captured Q700 ASC bus trace and emit a WAV.
//
// This is the "Option 3" bridge: instead of swapping our RTL into a
// custom MAME build (which would require a full MAME rebuild) we capture
// the bit-exact byte sequence MAME's CPU emits to the ASC during cold
// boot via tools/mame_asc_capture.lua, then drive the same writes /
// reads into our Vasc and capture the audio_pcm_l/r stream.  The
// resulting WAV represents what our RTL would emit if it sat in the
// EASC's place inside a Q700 — modulo the audio mixer downstream.
//
// Replay model:
//   * Trace events have ms-coarse timestamps (one per MAME frame at
//     60 Hz; events inside the same frame share a timestamp and are
//     replayed back-to-back in order).
//   * Each event maps to one pb_addr/pb_wdata cycle.  Between events
//     within the same frame, we tick a few clk cycles to give the slave
//     time to ack.  Between frame boundaries, we idle clk + phi2_tick
//     for the remainder of the frame interval (16.6 ms / 60 Hz).
//   * After the final captured event, we keep ticking phi2 (idle) for
//     a configurable trailer duration (default 1.5 s) so the wavetable
//     engine has time to produce the full sustained chime.
//
// The output WAV is written at the exact ASC sample-tick rate (each
// audio_sample_valid pulse becomes one frame; left/right come from
// audio_pcm_l / audio_pcm_r).  At default RATE_DEFAULT=35 with a
// 1 MHz phi2 the rate is ~22381 Hz.
//
// Build:  make tb-asc-mame-replay
// Usage:  build/asc_mame_replay/Vasc_replay <trace.tsv> <out.wav> \
//             [--trailer-ms 1500] [--phi2-mhz 1] [--max-events N]

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <verilated.h>
#include "Vasc.h"

namespace {

struct TraceEvent {
    uint64_t time_ns;
    char     rw;
    uint32_t reg_off;
    uint8_t  byte;
};

static Vasc*    dut       = nullptr;
static uint64_t sim_time  = 0;
// One core-clock period: tb_asc.cpp uses sim_time++ per half-cycle, so
// one full clock = 2 sim_time units.  We keep the same convention.
static int      n_phi2    = 0;
static uint64_t phi2_period_clk = 100;  // # of clk cycles per phi2 pulse (1 MHz @ 100 MHz clk)

static inline void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

// One core-clock cycle WITH a phi2_tick pulse (single-cycle high) —
// ASC samples phi2_tick at posedge clk and counts down its rate divider.
static inline void tick_with_phi2() {
    dut->phi2_tick = 1;
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
    dut->phi2_tick = 0;
}

// Run N core clocks, injecting one phi2_tick every phi2_period_clk
// cycles.  The audio engine ticks at sample_rate = phi2 / RATE_DEFAULT,
// so this is the tick generator that drives the chime engine.
static void run_clocks(uint64_t n_clk) {
    for (uint64_t i = 0; i < n_clk; ++i) {
        if ((i % phi2_period_clk) == (phi2_period_clk - 1)) {
            tick_with_phi2();
        } else {
            tick();
        }
    }
}

// Audio capture: one stereo frame per audio_sample_valid pulse.
struct AudioFrame { int16_t l, r; };
static std::vector<AudioFrame> audio_frames;
static int prev_valid = 0;

static void poll_audio() {
    int v = (int)dut->audio_sample_valid;
    if (v && !prev_valid) {
        AudioFrame f;
        f.l = (int16_t)dut->audio_pcm_l;
        f.r = (int16_t)dut->audio_pcm_r;
        audio_frames.push_back(f);
    }
    prev_valid = v;
}

// Thin wrapper that polls audio every clock so we don't miss a
// sample_valid edge that drops between ticks.
static void run_clocks_audio(uint64_t n_clk) {
    for (uint64_t i = 0; i < n_clk; ++i) {
        if ((i % phi2_period_clk) == (phi2_period_clk - 1)) {
            tick_with_phi2();
        } else {
            tick();
        }
        poll_audio();
    }
}

static void reset_dut() {
    dut->rst       = 1;
    dut->phi2_tick = 0;
    dut->pb_addr   = 0;
    dut->pb_wdata  = 0;
    dut->pb_wr     = 0;
    dut->pb_rd     = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
    // No post-reset register pokes — we want the chip to start in its
    // hardware reset state and let the captured trace drive every
    // configuration write the ROM emits.
}

static void bus_write(uint32_t reg_off, uint8_t byte) {
    dut->pb_addr  = (reg_off & 0xfff);
    dut->pb_wdata = byte;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    tick();
    dut->pb_wr    = 0;
    dut->pb_addr  = 0;
    dut->pb_wdata = 0;
    poll_audio();
}

static void bus_read(uint32_t reg_off) {
    dut->pb_addr = (reg_off & 0xfff);
    dut->pb_rd   = 1;
    dut->pb_wr   = 0;
    tick();
    dut->pb_rd   = 0;
    dut->pb_addr = 0;
    poll_audio();
}

static bool parse_trace(const std::string& path, std::vector<TraceEvent>& out) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) {
        std::fprintf(stderr, "cannot open %s\n", path.c_str());
        return false;
    }
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\0') continue;
        // Format: seq\ttime_ns\trw\treg_off_hex\tbyte_hex\n
        unsigned long long seq;
        unsigned long long time_ns;
        char rw_buf[4];
        unsigned int reg_off;
        unsigned int byte;
        // Use %s for rw and %x for hex fields.
        int parsed = std::sscanf(line, "%llu\t%llu\t%3s\t%x\t%x",
                                 &seq, &time_ns, rw_buf, &reg_off, &byte);
        if (parsed != 5) continue;
        TraceEvent ev;
        ev.time_ns = time_ns;
        ev.rw      = rw_buf[0];
        ev.reg_off = reg_off & 0xfff;
        ev.byte    = (uint8_t)(byte & 0xff);
        out.push_back(ev);
    }
    std::fclose(f);
    return true;
}

static void write_wav(const std::string& path, uint32_t sample_rate_hz) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::fprintf(stderr, "cannot write %s\n", path.c_str());
        std::exit(2);
    }
    const uint32_t n_frames = (uint32_t)audio_frames.size();
    const uint32_t data_sz  = n_frames * 4;  // 2ch * 2 bytes
    const uint32_t riff_sz  = 36 + data_sz;
    auto w_u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto w_u16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f);
    w_u32(riff_sz);
    std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f);
    w_u32(16);  // fmt chunk size
    w_u16(1);   // PCM
    w_u16(2);   // 2 channels
    w_u32(sample_rate_hz);
    w_u32(sample_rate_hz * 4);  // byte rate
    w_u16(4);                   // block align
    w_u16(16);                  // bits per sample
    std::fwrite("data", 1, 4, f);
    w_u32(data_sz);
    for (const auto& fr : audio_frames) {
        std::fwrite(&fr.l, 2, 1, f);
        std::fwrite(&fr.r, 2, 1, f);
    }
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    if (argc < 3) {
        std::fprintf(stderr,
            "usage: %s <trace.tsv> <out.wav> [--trailer-ms N] [--phi2-mhz N] [--max-events N]\n",
            argv[0]);
        return 2;
    }
    std::string trace_path = argv[1];
    std::string wav_path   = argv[2];
    uint64_t    trailer_ms = 1500;
    // The Q700 phi2 is 783_360 Hz (VIA1's reference NCO).  asc.v's
    // RATE_DEFAULT=35 over that produces 22381 Hz — matches the
    // first-gen Apple Sound Chip's audited rate (asc.v lines 311-323).
    double      phi2_mhz   = 0.78336;
    uint64_t    max_events = (uint64_t)-1;

    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--trailer-ms" && i + 1 < argc) {
            trailer_ms = std::strtoull(argv[++i], nullptr, 0);
        } else if (a == "--phi2-mhz" && i + 1 < argc) {
            phi2_mhz = std::strtod(argv[++i], nullptr);
        } else if (a == "--max-events" && i + 1 < argc) {
            max_events = std::strtoull(argv[++i], nullptr, 0);
        }
    }

    // Convert phi2_mhz into clk cycles per phi2 pulse.  We arbitrarily
    // pick a 100 MHz core clock (tb_asc.cpp does the same — sim_time is
    // in arbitrary units, only clk-cycle counts matter for ASC's logic
    // since the only timebase the chip cares about is phi2_tick).
    const uint64_t core_mhz = 100;
    phi2_period_clk = (uint64_t)((double)core_mhz / phi2_mhz);
    if (phi2_period_clk == 0) phi2_period_clk = 1;

    std::vector<TraceEvent> events;
    if (!parse_trace(trace_path, events)) return 2;
    if (events.empty()) {
        std::fprintf(stderr, "no events parsed from %s\n", trace_path.c_str());
        return 1;
    }
    if (events.size() > max_events) events.resize(max_events);

    std::printf("replay: %zu events from %s\n", events.size(), trace_path.c_str());
    std::printf("        trace span %llu ns .. %llu ns\n",
        (unsigned long long)events.front().time_ns,
        (unsigned long long)events.back().time_ns);
    std::printf("        core_clk = %lu MHz, phi2 = %.2f MHz (%lu clks/phi2)\n",
        (unsigned long)core_mhz, phi2_mhz, (unsigned long)phi2_period_clk);
    std::printf("        trailer = %lu ms\n", (unsigned long)trailer_ms);

    dut = new Vasc;
    reset_dut();

    // Replay loop.  We map each event timestamp delta into core-clock
    // ticks, then drive the bus access.
    uint64_t prev_time_ns = events.front().time_ns;

    // Lead-in: emit phi2 ticks for whatever was emulated before the
    // first event, capped at 200 ms (don't over-quiesce).
    uint64_t lead_in_ns = std::min<uint64_t>(prev_time_ns, 200000000ULL);
    uint64_t lead_in_clks = (uint64_t)(lead_in_ns * 1e-9 * core_mhz * 1e6);
    run_clocks_audio(lead_in_clks);

    for (size_t i = 0; i < events.size(); ++i) {
        const auto& ev = events[i];
        uint64_t dt_ns = ev.time_ns - prev_time_ns;
        if (dt_ns > 0) {
            uint64_t dt_clks = (uint64_t)(dt_ns * 1e-9 * core_mhz * 1e6);
            // Cap pathological gaps (frame boundary) at 100 ms — we
            // really only need the ASC engine to sample-tick at its
            // rate; longer idle just makes replay slow.
            if (dt_clks > core_mhz * 1000 * 100ULL) {  // > 100 ms
                dt_clks = core_mhz * 1000 * 100ULL;
            }
            run_clocks_audio(dt_clks);
        }
        prev_time_ns = ev.time_ns;

        if (ev.rw == 'W') {
            bus_write(ev.reg_off, ev.byte);
        } else {
            bus_read(ev.reg_off);
        }
        // Tiny inter-event idle so the slave can latch / ack.
        run_clocks_audio(2);
    }

    // Trailer — let the wavetable engine play out the chime.
    uint64_t trailer_clks = trailer_ms * core_mhz * 1000;
    std::printf("        captured %zu audio frames so far; running trailer (%lu clks)...\n",
                audio_frames.size(), (unsigned long)trailer_clks);
    run_clocks_audio(trailer_clks);

    // Derive the actual sample rate from how many frames we got over
    // (lead_in + replay_span + trailer).  Or use the chip's known
    // RATE_DEFAULT-derived rate (22381 Hz at 1 MHz phi2 with rate=35).
    // We compute it empirically from the simulated timeline.
    uint64_t total_clks = sim_time / 2;  // tick increments by 2
    double total_s = (double)total_clks / (core_mhz * 1e6);
    uint32_t derived_rate = (audio_frames.empty() || total_s <= 0)
        ? 22381
        : (uint32_t)(audio_frames.size() / total_s);

    std::printf("        captured %zu audio frames over %.3f sim-s -> derived rate %u Hz\n",
                audio_frames.size(), total_s, derived_rate);

    // The asc.v audio engine ticks at phi2 / 35 = 22381 Hz under the
    // default RATE_DEFAULT; the derived rate should match that closely.
    // Use the chip's intrinsic rate for the WAV header to keep pitch
    // intact regardless of any rounding in our run_clocks math.
    uint32_t intrinsic_rate = (uint32_t)((double)phi2_mhz * 1e6 / 35.0 + 0.5);
    if (intrinsic_rate == 0) intrinsic_rate = derived_rate;

    write_wav(wav_path, intrinsic_rate);
    std::printf("        wrote %s (%zu frames @ %u Hz, %.3f s)\n",
                wav_path.c_str(),
                audio_frames.size(),
                intrinsic_rate,
                audio_frames.empty() ? 0.0 : (double)audio_frames.size() / intrinsic_rate);

    delete dut;
    return 0;
}
