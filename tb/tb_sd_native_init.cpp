#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vsd_native_init.h"

static Vsd_native_init dut;
static int mode = 0;
static int acmd41_count = 0;
static bool csr_pending = false;
static bool cmd_pending = false;
static bool csr_error_pending = false;
static uint32_t response_words[4] = {};
static std::vector<int> commands;
static std::vector<int> command_cycles;
static std::vector<uint16_t> csr_addrs;
static std::vector<int> bus_order;
static int cycle_number = 0;

static void tick() {
    dut.clk = 0;
    dut.csr_rsp_valid = csr_pending;
    dut.csr_rsp_error = csr_error_pending;
    dut.cmd_done = cmd_pending;
    dut.cmd_error = 0;
    for (int i = 0; i < 4; ++i) dut.cmd_response[i] = response_words[i];
    dut.eval();
    const bool take_csr = !dut.rst && dut.csr_req_valid && dut.csr_req_ready;
    const bool take_cmd = !dut.rst && dut.cmd_start && !dut.cmd_busy;
    const uint16_t csr_addr = dut.csr_req_addr;
    const int cmd = dut.cmd_index;
    uint32_t next_response[4] = {};
    if (take_csr) {
        csr_addrs.push_back(csr_addr);
        if (csr_addr == 0x2804) bus_order.push_back(0x100);
    }
    if (take_cmd) {
        commands.push_back(cmd);
        command_cycles.push_back(cycle_number);
        bus_order.push_back(cmd);
        if (cmd == 8) next_response[0] = mode == 1 ? 0x155 : 0x1aa;
        if (cmd == 41) {
            ++acmd41_count;
            next_response[0] = mode == 2 || acmd41_count < 2 ? 0 : 0xc0ff8000;
        }
        if (cmd == 3) next_response[0] = 0x12340000;
        if (cmd == 9) {
            next_response[3] = mode == 3 ? 0 : 0x40000000;
            next_response[1] = 0x1fff0000;
        }
    }
    dut.clk = 1;
    dut.eval();
    csr_pending = take_csr;
    csr_error_pending = take_csr && mode == 4 && csr_addr == 0x2804;
    cmd_pending = take_cmd;
    for (int i = 0; i < 4; ++i) response_words[i] = next_response[i];
    ++cycle_number;
}

static bool run(int test_mode) {
    mode = test_mode;
    acmd41_count = 0;
    csr_pending = false;
    cmd_pending = false;
    csr_error_pending = false;
    commands.clear();
    command_cycles.clear();
    csr_addrs.clear();
    bus_order.clear();
    cycle_number = 0;
    for (auto &word : response_words) word = 0;
    dut.rst = 1;
    dut.csr_req_ready = 1;
    dut.cmd_busy = 0;
    tick();
    dut.rst = 0;
    for (int i = 0; i < 1000 && !dut.ready && !dut.error; ++i) tick();
    if (test_mode == 0) {
        const std::vector<int> expected = {0, 8, 55, 41, 55, 41, 2, 3,
                                           9, 7, 55, 6, 16};
        const std::vector<uint16_t> csr_expected = {0x2804, 0x2808, 0x2804};
        const std::vector<int> expected_order = {0x100, 0, 8, 55, 41, 55,
                                                  41, 0x100, 2, 3, 9, 7,
                                                  55, 6, 16};
        if (!dut.ready || dut.error || dut.rca != 0x12340000 ||
            dut.card_num_lbas != 8388608 || commands != expected ||
            csr_addrs != csr_expected || bus_order != expected_order ||
            command_cycles[2] - command_cycles[1] < 20 ||
            command_cycles[3] - command_cycles[2] < 20) {
            std::fprintf(stderr, "init sequence failed: ready=%d error=%d phase=%d rca=%08x lbas=%u cmds=%zu\n",
                         int(dut.ready), int(dut.error), int(dut.debug_phase),
                         dut.rca, dut.card_num_lbas, commands.size());
            for (int cmd : commands) std::fprintf(stderr, " cmd=%d", cmd);
            std::fputc('\n', stderr);
            for (int addr : csr_addrs) std::fprintf(stderr, " csr=%04x", addr);
            std::fputc('\n', stderr);
            for (int op : bus_order) std::fprintf(stderr, " op=%x", op);
            std::fputc('\n', stderr);
            return false;
        }
    } else {
        const int expected_phase = test_mode == 1 ? 3 :
                                   test_mode == 2 ? 6 :
                                   test_mode == 3 ? 9 : 0;
        if (!dut.error || dut.ready || dut.debug_phase != expected_phase) {
            std::fprintf(stderr, "init error case %d failed: ready=%d error=%d phase=%d\n",
                         test_mode, int(dut.ready), int(dut.error),
                         int(dut.debug_phase));
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int test_mode = 0; test_mode < 5; ++test_mode)
        if (!run(test_mode)) return 1;
    std::puts("PASS native SD init: sequence, OCR retry, capacity/RCA, voltage/CSD/CSR failures");
    return 0;
}
