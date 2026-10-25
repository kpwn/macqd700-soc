#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vsd_scsi_native_bridge.h"

static Vsd_scsi_native_bridge dut;
static uint64_t cyc = 0;
static std::vector<uint8_t> pb_read;
static std::vector<uint8_t> core_write;
static int pb_write_pos = 0;
static int done_count = 0;
static bool done_early = false;
static bool saw_core_go = false;
static int requested_bytes = 512;

static void tick() {
    dut.core_clk = 0;
    dut.pb_clk = 0;
    dut.eval();
    bool pb_edge = !(cyc & 1);
    bool pb_write_take = pb_edge && dut.pb_wr_ready;
    bool core_write_take = dut.core_wr_ready && dut.core_wr_avail;
    uint8_t write_byte = dut.core_wr_data;
    dut.core_clk = 1;
    if (pb_edge) dut.pb_clk = 1;
    dut.eval();
    if (dut.core_go) saw_core_go = true;
    if (core_write_take) core_write.push_back(write_byte);
    if (pb_edge) {
        if (pb_write_take) ++pb_write_pos;
        if (dut.pb_rd_valid) pb_read.push_back(dut.pb_rd_data);
        if (dut.pb_done) {
            ++done_count;
            if (dut.pb_cmd_type == 2 && pb_read.size() != size_t(requested_bytes))
                done_early = true;
        }
    }
    ++cyc;
}

static void reset() {
    dut.pb_rst = 1;
    dut.core_rst = 1;
    dut.pb_go = 0;
    dut.pb_rd_ready = 1;
    dut.pb_wr_avail = 0;
    dut.pb_wr_data = 0;
    dut.core_busy = 0;
    dut.core_done = 0;
    dut.core_error = 0;
    dut.core_rd_valid = 0;
    dut.core_rd_data = 0;
    dut.core_wr_ready = 0;
    for (int i = 0; i < 10; ++i) tick();
    dut.pb_rst = 0;
    dut.core_rst = 0;
    for (int i = 0; i < 10; ++i) tick();
    pb_read.clear();
    core_write.clear();
    pb_write_pos = 0;
    done_count = 0;
    done_early = false;
    saw_core_go = false;
}

static void launch(int type, int blocks) {
    dut.pb_cmd_type = type;
    dut.pb_lba = 8192;
    dut.pb_block_count = blocks;
    requested_bytes = blocks * 512;
    dut.pb_go = 1;
    // Start at a pb edge.
    while (cyc & 1) tick();
    tick();
    dut.pb_go = 0;
}

static bool test_read(int blocks) {
    reset();
    launch(2, blocks);
    int offered = 0;
    bool core_active = false;
    bool done_sent = false;
    for (int i = 0; i < 300000 && done_count == 0; ++i) {
        if (saw_core_go) core_active = true;
        dut.core_busy = core_active && !done_sent;
        dut.pb_rd_ready = (i % 59) < 41;
        dut.core_clk = 0;
        dut.pb_clk = 0;
        dut.eval();
        bool send = core_active && offered < requested_bytes &&
                    (i % 4 == 0) && dut.core_rd_ready;
        dut.core_rd_valid = send;
        dut.core_rd_data = uint8_t(offered);
        dut.core_done = core_active && offered == requested_bytes && !done_sent;
        tick();
        if (send) ++offered;
        if (dut.core_done) done_sent = true;
        dut.core_done = 0;
    }
    if (done_count != 1 || done_early || pb_read.size() != size_t(requested_bytes) || dut.pb_error) {
        std::fprintf(stderr, "read completion failed: offered=%d delivered=%zu done=%d early=%d error=%d\n",
                     offered, pb_read.size(), done_count, done_early, int(dut.pb_error));
        return false;
    }
    for (int i = 0; i < requested_bytes; ++i)
        if (pb_read[i] != uint8_t(i)) {
            std::fprintf(stderr, "read byte mismatch at %d: %02x\n", i, pb_read[i]);
            return false;
        }
    return true;
}

static bool test_write(int blocks) {
    reset();
    launch(4, blocks);
    bool core_active = false;
    bool done_sent = false;
    for (int i = 0; i < 300000 && done_count == 0; ++i) {
        if (saw_core_go) core_active = true;
        dut.core_busy = core_active && !done_sent;
        dut.pb_wr_avail = pb_write_pos < requested_bytes;
        dut.pb_wr_data = uint8_t(pb_write_pos);
        dut.core_clk = 0;
        dut.pb_clk = 0;
        dut.eval();
        dut.core_wr_ready = core_active && core_write.size() < size_t(requested_bytes) &&
                            (i % 4 == 0) && dut.core_wr_avail;
        dut.core_done = core_active && core_write.size() == size_t(requested_bytes) && !done_sent;
        tick();
        if (dut.core_done) done_sent = true;
        dut.core_done = 0;
    }
    if (done_count != 1 || core_write.size() != size_t(requested_bytes) ||
        pb_write_pos != requested_bytes || dut.pb_error) {
        std::fprintf(stderr, "write completion failed: staged=%d consumed=%zu done=%d error=%d\n",
                     pb_write_pos, core_write.size(), done_count, int(dut.pb_error));
        return false;
    }
    for (int i = 0; i < requested_bytes; ++i)
        if (core_write[i] != uint8_t(i)) {
            std::fprintf(stderr, "write byte mismatch at %d: %02x\n", i, core_write[i]);
            return false;
        }
    return true;
}

static bool test_partial_read_error() {
    reset();
    launch(2, 1);
    bool core_active = false;
    bool done_sent = false;
    int offered = 0;
    for (int i = 0; i < 2000 && done_count == 0; ++i) {
        if (saw_core_go) core_active = true;
        dut.core_busy = core_active && !done_sent;
        dut.core_clk = 0;
        dut.pb_clk = 0;
        dut.eval();
        bool send = core_active && offered < 32 && (i % 4 == 0) && dut.core_rd_ready;
        dut.core_rd_valid = send;
        dut.core_rd_data = uint8_t(offered);
        dut.core_done = core_active && offered == 32 && !done_sent;
        dut.core_error = dut.core_done;
        tick();
        if (send) ++offered;
        if (dut.core_done) done_sent = true;
        dut.core_done = 0;
        dut.core_error = 0;
    }
    if (done_count != 1 || !dut.pb_error || pb_read.size() > 32) {
        std::fprintf(stderr, "partial read error not propagated: bytes=%zu done=%d error=%d\n",
                     pb_read.size(), done_count, int(dut.pb_error));
        return false;
    }
    return true;
}

static bool test_core_reset_abort() {
    reset();
    launch(2, 1);
    for (int i = 0; i < 20; ++i) tick();
    if (!saw_core_go) {
        std::fputs("core request did not arrive before reset test\n", stderr);
        return false;
    }
    dut.core_busy = 1;
    for (int i = 0; i < 20; ++i) tick();
    dut.core_rst = 1;
    dut.core_busy = 0;
    for (int i = 0; i < 12; ++i) tick();
    dut.core_rst = 0;
    for (int i = 0; i < 100 && done_count == 0; ++i) tick();
    if (done_count != 1 || !dut.pb_error) {
        std::fprintf(stderr, "core reset did not abort: done=%d error=%d\n",
                     done_count, int(dut.pb_error));
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (!test_read(1) || !test_write(1) ||
        !test_read(32) || !test_write(32) ||
        !test_partial_read_error() || !test_core_reset_abort()) return 1;
    std::puts("PASS native SD SCSI bridge: 1/32-sector read/write, 25 MB/s source, backpressure, ordered completion, error/reset abort");
    return 0;
}
