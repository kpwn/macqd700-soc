// tb_clk_rst.cpp — Verilator unit testbench for rtl/sys/clk_rst.v.
//
// Scenarios:
//   1. 50 MHz bring-up path: with CLK_DIVIDE=4, verify the divided clock
//      rises every four input cycles on a 200 MHz input clock.
//   2. Reset release: rst_out stays asserted until four divided-clock
//      rising edges occur after init_done and rst_in both allow release.
//   3. Re-arm: dropping init_done or asserting rst_in again reasserts
//      rst_out and requires another four divided-clock edges before the
//      core can leave reset.
//
// Build: make tb-clk-rst
// Pass:  "All 3 scenarios PASSED."

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <verilated.h>
#include "Vtb_clk_rst.h"

#ifndef TB_CLK_DIVIDE
#define TB_CLK_DIVIDE 4
#endif

static Vtb_clk_rst* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, fmt, ...) do { \
    if (cond) { \
        n_pass++; \
        std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); \
    } else { \
        n_fail++; \
        std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); \
    } \
} while (0)

static void settle() {
    dut->eval();
}

static void set_inputs(uint8_t rst_in, uint8_t init_done) {
    dut->rst_in = rst_in;
    dut->init_done = init_done;
    settle();
}

static bool step_cycle() {
    bool prev_clk = dut->clk_out;
    dut->clk_in = 0;
    dut->eval();
    dut->clk_in = 1;
    dut->eval();
    sim_time++;
    return (!prev_clk && dut->clk_out);
}

static int wait_for_rises(int target_rises, int* rise_intervals, int max_cycles) {
    int rises = 0;
    int since_last_rise = 0;
    while (rises < target_rises && since_last_rise < max_cycles) {
        since_last_rise++;
        if (step_cycle()) {
            rise_intervals[rises] = since_last_rise;
            rises++;
            since_last_rise = 0;
        }
    }
    return rises;
}

static bool test_divide_and_reset_release() {
    int fail_before = n_fail;

    set_inputs(1, 0);
    CHECK(dut->rst_out == 1, "rst_out asserts while init_done is low");
    CHECK(dut->clk_out == 0, "clk_out is held low while the combined reset request is asserted");

    set_inputs(0, 1);
    CHECK(dut->rst_out == 1, "rst_out stays high until divided clock edges arrive");
    CHECK(dut->clk_out == 0, "clk_out remains low until the divider restarts");

    int rise_intervals[4] = {0, 0, 0, 0};
    int rises = wait_for_rises(4, rise_intervals, 64);
    CHECK(rises == 4, "saw four divided-clock rising edges before reset release");
    const int first_rise = TB_CLK_DIVIDE / 2;
    CHECK(rise_intervals[0] == first_rise && rise_intervals[1] == TB_CLK_DIVIDE &&
          rise_intervals[2] == TB_CLK_DIVIDE && rise_intervals[3] == TB_CLK_DIVIDE,
          "CLK_DIVIDE=%d starts low, rises after %d input cycles, then repeats every %d cycles",
          TB_CLK_DIVIDE, first_rise, TB_CLK_DIVIDE);
    CHECK(dut->rst_out == 0, "rst_out deasserts after the fourth divided-clock rise");
    return (n_fail == fail_before) &&
           rises == 4 && dut->rst_out == 0 &&
           rise_intervals[0] == first_rise && rise_intervals[1] == TB_CLK_DIVIDE &&
           rise_intervals[2] == TB_CLK_DIVIDE && rise_intervals[3] == TB_CLK_DIVIDE;
}

static bool test_rearm_on_reset_and_init_drop() {
    int fail_before = n_fail;

    set_inputs(1, 1);
    CHECK(dut->rst_out == 1, "asserting rst_in immediately reasserts rst_out");
    CHECK(dut->clk_out == 0, "clk_out clears again when rst_in is reasserted");

    set_inputs(0, 1);
    CHECK(dut->rst_out == 1, "release restarts the four-edge reset pipeline");

    int rise_intervals[4] = {0, 0, 0, 0};
    int rises = wait_for_rises(4, rise_intervals, 64);
    CHECK(rises == 4, "reset pipeline completes again after re-arm");
    CHECK(dut->rst_out == 0, "rst_out drops again after four more divided-clock rises");

    set_inputs(0, 0);
    CHECK(dut->rst_out == 1, "dropping init_done reasserts rst_out");

    set_inputs(0, 1);
    CHECK(dut->rst_out == 1, "init_done high alone still requires four edges before release");

    rises = wait_for_rises(4, rise_intervals, 64);
    CHECK(rises == 4, "reset pipeline re-clocks after init_done returns");
    CHECK(dut->rst_out == 0, "rst_out deasserts after the second clean release");

    return n_fail == fail_before;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_clk_rst;

    std::printf("── tb_clk_rst: clk_rst divide/reset unit tb ──\n");
    dut->clk_in = 0;
    dut->rst_in = 0;
    dut->init_done = 0;
    settle();

    CHECK(test_divide_and_reset_release(),
          "configured divide path and first reset-release sequence");
    CHECK(test_rearm_on_reset_and_init_drop(),
          "reset pipeline re-arms after rst_in/init_done changes");

    std::printf("All %d scenarios %s\n", n_pass + n_fail,
                (n_fail == 0) ? "PASSED." : "FAILED.");
    return (n_fail == 0) ? 0 : 1;
}
