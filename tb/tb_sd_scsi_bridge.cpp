// tb_sd_scsi_bridge.cpp — Unit testbench for rtl/sys/sd_scsi_bridge.v
//
// Verifies the CDC primitives in isolation:
//   1. go pulse pb→core: a single pb_go produces exactly one core_go
//   2. cmd_type/lba/block_count cross stably and arrive at core after go
//   3. busy level sync core→pb: pb_busy follows core_busy after the
//      sync ladder closes
//   4. done pulse core→pb: a single core_done produces exactly one pb_done
//   5. error level latched at done and observable on pb side
//   6. read byte stream: each core_rd_valid pulse + data arrives once on pb
//   7. write byte stream: each core_wr_ready pulse arrives once on pb,
//      and pb_wr_data is visible to core_wr_data after the sync ladder
//
// The two clocks tick at different rates to exercise the CDC.
//
// Build: make tb-sd-scsi-bridge

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <verilated.h>
#include "Vsd_scsi_bridge.h"

static Vsd_scsi_bridge* dut      = nullptr;
static uint64_t         sim_time = 0;
static int              n_pass   = 0;
static int              n_fail   = 0;

// pb_clk = 50 MHz  → period 20 ns → half 10 ns
// core_clk = 200 MHz → period  5 ns → half 2.5 ns (we use 3 ns + 2 ns for
//   integer ticks under the 1 ns sim_time unit)
static constexpr int PB_HALF_NS   = 10;
static constexpr int CORE_PERIOD_NS = 5;

static int pb_phase = 0;     // 0 = pb_clk low for 10 ns, 1 = high for 10 ns
static int pb_phase_ns = 0;
static int core_phase = 0;   // toggles every 2-3 ns to approximate 200 MHz
static int core_phase_ns = 0;

static void tick_1ns() {
    sim_time += 1;
    pb_phase_ns += 1;
    if (pb_phase_ns >= PB_HALF_NS) {
        pb_phase_ns = 0;
        pb_phase ^= 1;
        dut->pb_clk = pb_phase;
    }
    core_phase_ns += 1;
    // Toggle every 2-3 ns alternately for an average 5 ns period.
    int target = (core_phase == 0) ? 3 : 2;  // 3 ns low, 2 ns high
    if (core_phase_ns >= target) {
        core_phase_ns = 0;
        core_phase ^= 1;
        dut->core_clk = core_phase;
    }
    dut->eval();
}

// Run for `ns` ns, but if `cb` returns true at any point return early.
template <typename Cb>
static bool run_until(int max_ns, Cb cb) {
    for (int i = 0; i < max_ns; i++) {
        tick_1ns();
        if (cb()) return true;
    }
    return false;
}

static void run_for(int ns) {
    for (int i = 0; i < ns; i++) tick_1ns();
}

static bool pb_posedge() {
    static int prev = 0;
    int cur = dut->pb_clk;
    bool edge = (prev == 0) && (cur == 1);
    prev = cur;
    return edge;
}

static bool core_posedge() {
    static int prev = 0;
    int cur = dut->core_clk;
    bool edge = (prev == 0) && (cur == 1);
    prev = cur;
    return edge;
}

#define CHECK_EQ(name, got, exp) do {                                     \
    if ((got) != (exp)) {                                                 \
        std::fprintf(stderr, "FAIL %s: got=%llu exp=%llu @ t=%llu\n",     \
                     name, (unsigned long long)(got),                     \
                     (unsigned long long)(exp),                           \
                     (unsigned long long)sim_time);                       \
        n_fail++;                                                          \
    } else { n_pass++; }                                                   \
} while (0)

#define CHECK_TRUE(name, cond) do {                                       \
    if (!(cond)) {                                                        \
        std::fprintf(stderr, "FAIL %s @ t=%llu\n", name,                  \
                     (unsigned long long)sim_time);                       \
        n_fail++;                                                          \
    } else { n_pass++; }                                                   \
} while (0)

static void apply_reset() {
    dut->pb_rst   = 1;
    dut->core_rst = 1;
    dut->pb_clk   = 0;
    dut->core_clk = 0;
    dut->pb_cmd_type    = 0;
    dut->pb_lba         = 0;
    dut->pb_block_count = 0;
    dut->pb_go          = 0;
    dut->pb_rd_ready    = 0;
    dut->pb_wr_data     = 0;
    // Producer-availability back-pressure level (pb -> core).  Held
    // high here: this tb measures the CDC's transport, not the pacing.
    dut->pb_wr_avail    = 1;
    dut->core_busy      = 0;
    dut->core_done      = 0;
    dut->core_error     = 0;
    dut->core_rd_valid  = 0;
    dut->core_rd_data   = 0;
    dut->core_wr_ready  = 0;
    run_for(60);
    dut->pb_rst   = 0;
    dut->core_rst = 0;
    run_for(40);
}

// ── Scenario 1 ─────────────────────────────────────────────────────
// pb_go pulse → exactly one core_go pulse, with cmd/lba/block_count
// observable on the core side.
static void s_go_pulse_and_request_fields() {
    apply_reset();

    dut->pb_cmd_type    = 1;            // CT_CMD17
    dut->pb_lba         = 0xCAFEBABE;
    dut->pb_block_count = 0x0007;
    // wait for a clean pb posedge to assert pb_go
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 1;
    tick_1ns();
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 0;
    // Watch the core side for exactly one core_go pulse.
    int seen = 0;
    int max_ns = 200;
    int last_core_go = 0;
    for (int i = 0; i < max_ns; i++) {
        tick_1ns();
        if (core_posedge()) {
            if (dut->core_go && !last_core_go) seen++;
            last_core_go = dut->core_go;
        }
    }
    CHECK_EQ("scenario1: core_go pulse count", seen, 1);
    CHECK_EQ("scenario1: core_cmd_type",    dut->core_cmd_type,    1);
    CHECK_EQ("scenario1: core_lba",         dut->core_lba,         0xCAFEBABEu);
    CHECK_EQ("scenario1: core_block_count", dut->core_block_count, 0x0007);
}

// ── Scenario 2 ─────────────────────────────────────────────────────
// busy level: assert core_busy, expect pb_busy to rise within ~3 pb cycles.
static void s_busy_level() {
    apply_reset();
    dut->core_busy = 1;
    int waited_ns = 0;
    while (dut->pb_busy != 1 && waited_ns < 200) { tick_1ns(); waited_ns++; }
    CHECK_TRUE("scenario2: pb_busy rose after core_busy", dut->pb_busy == 1);
    dut->core_busy = 0;
    waited_ns = 0;
    while (dut->pb_busy != 0 && waited_ns < 200) { tick_1ns(); waited_ns++; }
    CHECK_TRUE("scenario2: pb_busy fell after core_busy clear", dut->pb_busy == 0);
}

// ── Scenario 3 ─────────────────────────────────────────────────────
// done pulse + error latch.  Pulse core_done with core_error=1, expect
// a single pb_done with pb_error=1.
static void s_done_and_error_latch() {
    apply_reset();
    // First go pb side so pb_busy_internal is set; otherwise pb_done is
    // benign but may also appear as level-mismatched.
    dut->pb_cmd_type = 1; dut->pb_lba = 0; dut->pb_block_count = 1;
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 1;
    tick_1ns();
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 0;
    run_for(60);  // let core observe go

    // Pulse core_done with core_error=1 on a core posedge.
    while (!core_posedge()) tick_1ns();
    dut->core_error = 1;
    dut->core_done  = 1;
    tick_1ns();
    while (!core_posedge()) tick_1ns();
    dut->core_done  = 0;

    // Expect pb_done within ~6 pb cycles.
    int seen = 0;
    int last_pb_done = 0;
    for (int i = 0; i < 300; i++) {
        tick_1ns();
        if (pb_posedge()) {
            if (dut->pb_done && !last_pb_done) seen++;
            last_pb_done = dut->pb_done;
        }
        if (seen) break;
    }
    CHECK_EQ("scenario3: pb_done pulse seen", seen, 1);
    CHECK_TRUE("scenario3: pb_error latched", dut->pb_error == 1);
}

// Helper: pulse a 1-bit core-side input for exactly one core_clk cycle.
// Asserts the signal on a core posedge, then deasserts on the next.
static void core_pulse_rd_valid(uint8_t data) {
    while (!core_posedge()) tick_1ns();
    dut->core_rd_data  = data;
    dut->core_rd_valid = 1;
    tick_1ns();
    while (!core_posedge()) tick_1ns();
    dut->core_rd_valid = 0;
}

static void core_pulse_wr_ready() {
    while (!core_posedge()) tick_1ns();
    dut->core_wr_ready = 1;
    tick_1ns();
    while (!core_posedge()) tick_1ns();
    dut->core_wr_ready = 0;
}

// ── Scenario 4 ─────────────────────────────────────────────────────
// Read byte stream: pulse core_rd_valid 4 times with distinct data,
// observe 4 pb_rd_valid pulses with the same data.
static void s_read_byte_stream() {
    apply_reset();

    const uint8_t pattern[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    std::vector<uint8_t> received;

    // Drain pulses on the pb side concurrently with each producer step.
    auto drain_pb = [&](int ns) {
        int last_pb_rd_valid = 0;
        for (int i = 0; i < ns; i++) {
            tick_1ns();
            if (pb_posedge()) {
                if (dut->pb_rd_valid && !last_pb_rd_valid) {
                    received.push_back(dut->pb_rd_data);
                }
                last_pb_rd_valid = dut->pb_rd_valid;
            }
        }
    };

    // Produce one byte, then drain ≥ 100 ns so the pb side observes it
    // (sync ladder + edge detect + pb_clk visibility = ~3 pb cycles =
    // 60 ns; 200 ns gives lots of margin).
    for (int i = 0; i < 4; i++) {
        core_pulse_rd_valid(pattern[i]);
        drain_pb(200);
    }

    CHECK_EQ("scenario4: read byte count", received.size(), (size_t)4);
    if (received.size() == 4) {
        for (int i = 0; i < 4; i++) {
            char name[64];
            std::snprintf(name, sizeof(name), "scenario4: byte[%d]", i);
            CHECK_EQ(name, received[i], pattern[i]);
        }
    }
}

// ── Scenario 5 ─────────────────────────────────────────────────────
// Write byte stream: pb side presents wr_data; core pulses wr_ready;
// scsi-side bridge advances and presents next byte.  Verify each
// core_wr_ready pulse generates exactly one pb_wr_ready, and that
// pb_wr_data is visible at core_wr_data after the sync ladder.
static void s_write_byte_stream() {
    apply_reset();

    const uint8_t pattern[3] = {0x12, 0x34, 0x56};
    int idx = 0;
    int seen_pulses = 0;
    int last_pb_wr_ready = 0;

    // pb side: drive pattern[0] first; advance on each pb_wr_ready.
    dut->pb_wr_data = pattern[0];

    // Run helper that advances the simulation while watching pb_wr_ready
    // and updating idx + pb_wr_data.
    auto run_and_watch = [&](int ns) {
        for (int i = 0; i < ns; i++) {
            tick_1ns();
            if (pb_posedge()) {
                if (dut->pb_wr_ready && !last_pb_wr_ready) {
                    seen_pulses++;
                    idx++;
                    if (idx < 3) dut->pb_wr_data = pattern[idx];
                }
                last_pb_wr_ready = dut->pb_wr_ready;
            }
        }
    };

    // Wait for core_wr_data to settle to pattern[0] before the first pulse.
    int settle_ns = 0;
    while (dut->core_wr_data != pattern[0] && settle_ns < 200) {
        tick_1ns();
        settle_ns++;
    }
    CHECK_TRUE("scenario5: pattern[0] visible on core side", dut->core_wr_data == pattern[0]);

    // Pulse core_wr_ready; pb advances idx.  After each pulse drain >100 ns
    // so the toggle round-trip closes and core sees the next byte.
    core_pulse_wr_ready();
    run_and_watch(200);
    CHECK_EQ("scenario5: pulse 1 idx",     idx, 1);
    CHECK_EQ("scenario5: core sees byte 1", dut->core_wr_data, pattern[1]);

    core_pulse_wr_ready();
    run_and_watch(200);
    CHECK_EQ("scenario5: pulse 2 idx",     idx, 2);
    CHECK_EQ("scenario5: core sees byte 2", dut->core_wr_data, pattern[2]);

    core_pulse_wr_ready();
    run_and_watch(200);
    CHECK_EQ("scenario5: pulse 3 idx",     idx, 3);
    CHECK_EQ("scenario5: total pulses",    seen_pulses, 3);
}

// ── Scenario 6 ─────────────────────────────────────────────────────
// ASYMMETRIC RESET.  The SoC drives the two sides from different reset
// trees (core: soc_full_rst_bank[5] | warm_peripheral_reset, direct;
// pb: pb_full_rst_bank[2] through an xpm_cdc_async_rst + BUFG), so the
// core side can reset several pb cycles before the pb side.
//
// The failure this guards against is NOT "the transfer aborts" — that
// is expected and fine.  It is "the transfer is reported as having
// COMPLETED SUCCESSFULLY".  A one-sided reset must never fabricate a
// clean pb_done: either nothing happens, or the completion carries
// pb_error.
static void s_core_reset_midtransfer_no_fake_success() {
    apply_reset();

    // (a) Complete one clean transfer so the core-side done toggle sits
    //     at 1.  This is the state that turns the next core reset into a
    //     1→0 edge on the pb side.
    dut->pb_cmd_type = 1; dut->pb_lba = 0x1000; dut->pb_block_count = 1;
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 1;
    tick_1ns();
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 0;
    run_for(60);
    while (!core_posedge()) tick_1ns();
    dut->core_busy = 1;
    dut->core_error = 0;
    dut->core_done  = 1;
    tick_1ns();
    while (!core_posedge()) tick_1ns();
    dut->core_done = 0;
    dut->core_busy = 0;
    run_for(300);
    CHECK_TRUE("scenario6: first transfer completed cleanly",
               dut->pb_error == 0);

    // (b) Start a second transfer and let the core side pick it up.
    while (!pb_posedge()) tick_1ns();
    dut->pb_lba = 0x2000;
    dut->pb_go  = 1;
    tick_1ns();
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 0;
    dut->core_busy = 1;
    run_for(120);
    CHECK_TRUE("scenario6: second transfer in flight", dut->pb_busy == 1);

    // (c) Reset ONLY the core side, mid-transfer.  pb_rst stays low.
    //     The observation window opens the moment core_rst asserts —
    //     the spurious edge propagates through the pb sync ladder in
    //     ~3 pb cycles (60 ns), i.e. while core_rst is still high.
    int fake_done    = 0;
    int errored_done = 0;
    int last_pb_done = 0;
    auto watch_pb = [&](int ns) {
        for (int i = 0; i < ns; i++) {
            tick_1ns();
            if (pb_posedge()) {
                if (dut->pb_done && !last_pb_done) {
                    if (dut->pb_error) errored_done++; else fake_done++;
                }
                last_pb_done = dut->pb_done;
            }
        }
    };

    while (!core_posedge()) tick_1ns();
    dut->core_rst  = 1;
    dut->core_busy = 0;
    dut->core_done = 0;
    watch_pb(60);
    dut->core_rst = 0;

    // (d) Watch the pb side.  A pb_done pulse with pb_error==0 is the
    //     fabricated completion: scsi.v would report GOOD status for a
    //     transfer that never moved a byte.
    watch_pb(600);
    CHECK_EQ("scenario6: no fabricated clean completion on core reset",
             fake_done, 0);
    CHECK_TRUE("scenario6: bridge reports the aborted transfer as an error",
               errored_done >= 1 && dut->pb_error == 1);
}

// ── Scenario 7 ─────────────────────────────────────────────────────
// Same asymmetry, read-stream flavour: a one-sided core reset must not
// inject a phantom byte into scsi.v's 512-byte sector ring (which would
// desync the ring pointer for the rest of the transfer).
static void s_core_reset_no_phantom_read_byte() {
    apply_reset();

    // Move one byte so core_rd_tog sits at 1.
    core_pulse_rd_valid(0x5A);
    run_for(200);

    int bytes_after_reset = 0;
    int last_rd_valid = 0;

    auto watch_rd = [&](int ns) {
        for (int i = 0; i < ns; i++) {
            tick_1ns();
            if (pb_posedge()) {
                if (dut->pb_rd_valid && !last_rd_valid) bytes_after_reset++;
                last_rd_valid = dut->pb_rd_valid;
            }
        }
    };
    while (!core_posedge()) tick_1ns();
    dut->core_rst = 1;
    watch_rd(60);
    dut->core_rst = 0;
    watch_rd(600);
    CHECK_EQ("scenario7: no phantom read byte on one-sided core reset",
             bytes_after_reset, 0);
}

// ── Scenario 8 ─────────────────────────────────────────────────────
// The mirror case: reset ONLY the pb side while a request toggle is
// pending.  The core side must not see a phantom core_go (which would
// launch an SD command nobody asked for, against stale lba/count).
static void s_pb_reset_no_phantom_go() {
    apply_reset();

    // One request so pb_go_tog sits at 1.
    dut->pb_cmd_type = 1; dut->pb_lba = 0x3000; dut->pb_block_count = 1;
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 1;
    tick_1ns();
    while (!pb_posedge()) tick_1ns();
    dut->pb_go = 0;
    run_for(200);

    int gos_after_reset = 0;
    int last_go = 0;
    auto watch_go = [&](int ns) {
        for (int i = 0; i < ns; i++) {
            tick_1ns();
            if (core_posedge()) {
                if (dut->core_go && !last_go) gos_after_reset++;
                last_go = dut->core_go;
            }
        }
    };
    while (!pb_posedge()) tick_1ns();
    dut->pb_rst = 1;
    watch_go(60);
    dut->pb_rst = 0;
    watch_go(600);
    CHECK_EQ("scenario8: no phantom core_go on one-sided pb reset",
             gos_after_reset, 0);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vsd_scsi_bridge;

    s_go_pulse_and_request_fields();
    s_busy_level();
    s_done_and_error_latch();
    s_read_byte_stream();
    s_write_byte_stream();
    s_core_reset_midtransfer_no_fake_success();
    s_core_reset_no_phantom_read_byte();
    s_pb_reset_no_phantom_go();

    std::printf("=== sd_scsi_bridge ===\n");
    std::printf("PASS=%d  FAIL=%d\n", n_pass, n_fail);

    delete dut;
    return n_fail ? 1 : 0;
}
