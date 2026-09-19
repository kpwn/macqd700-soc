#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vperipheral_reset_sequencer.h"

static Vperipheral_reset_sequencer *dut;
static int failures;

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
}

#define CHECK(msg, cond) do { \
    if (!(cond)) { std::printf("[FAIL] %s\n", msg); failures++; } \
    else std::printf("[PASS] %s\n", msg); \
} while (0)

static void reset_dut() {
    dut->rst = 1;
    dut->reset_req = 0;
    dut->storage_busy = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vperipheral_reset_sequencer;
    reset_dut();

    dut->reset_req = 1;
    tick();
    CHECK("idle request starts peripheral reset", dut->peripheral_reset);
    CHECK("active reset also resets storage", dut->storage_reset_req);

    int active = dut->peripheral_reset ? 1 : 0;
    for (int i = 0; i < 12; i++) {
        tick();
        if (dut->peripheral_reset) active++;
    }
    CHECK("pulse is exactly eight cycles", active == 8);
    CHECK("held request does not retrigger", !dut->peripheral_reset);

    dut->reset_req = 0;
    tick();
    dut->storage_busy = 1;
    dut->reset_req = 1;
    tick();
    CHECK("busy storage defers peripheral reset", !dut->peripheral_reset);
    dut->reset_req = 0;
    tick();
    CHECK("pending request keeps storage reset asserted", dut->storage_reset_req);
    dut->storage_busy = 0;
    tick();
    CHECK("peripheral reset starts once storage closes", dut->peripheral_reset);

    active = dut->peripheral_reset ? 1 : 0;
    while (dut->peripheral_reset) {
        tick();
        if (dut->peripheral_reset) active++;
    }
    CHECK("deferred pulse is exactly eight cycles", active == 8);
    CHECK("storage reset releases after warm reset", !dut->storage_reset_req);

    dut->final();
    delete dut;
    std::printf("\n%s\n", failures ? "FAILED" : "all scenarios passed");
    return failures ? 1 : 0;
}
