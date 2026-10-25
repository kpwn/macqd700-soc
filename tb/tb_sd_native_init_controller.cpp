#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vsd_native_init_controller.h"

static Vsd_native_init_controller dut;
static bool pending_ack = false;
static bool pending_err = false;
static uint32_t pending_data = 0;
static uint32_t cmd_arg = 0;
static uint32_t cmd_word = 0;
static uint32_t response_words[4] = {};
static int ocr_count = 0;
static int mode = 0;
static int writes = 0;
static std::vector<int> commands;

static void card_command() {
    const int cmd = (cmd_word >> 8) & 63;
    commands.push_back(cmd);
    for (auto &word : response_words) word = 0;
    if (cmd == 8) response_words[0] = 0x1aa;
    if (cmd == 41) {
        ++ocr_count;
        response_words[0] = ocr_count == 1 ? 0 : 0xc0ff8000;
    }
    if (cmd == 3) response_words[0] = 0x12340000;
    if (cmd == 9) {
        response_words[3] = 0x40000000;
        response_words[1] = 0x1fff0000;
    }
    (void)cmd_arg;
}

static void tick() {
    dut.clk = 0;
    dut.wb_ack = pending_ack;
    dut.wb_err = pending_err;
    dut.wb_dat_r = pending_data;
    dut.eval();
    bool take = !dut.rst && dut.wb_cyc && dut.wb_stb && !dut.wb_ack && !dut.wb_err;
    uint16_t addr = uint16_t(dut.wb_adr << 2);
    bool next_err = false;
    uint32_t next_data = 0;
    if (take) {
        if (mode == 1 && writes == 0) next_err = true;
        if (dut.wb_we) {
            ++writes;
            if (addr == 0x1000) cmd_arg = dut.wb_dat_w;
            if (addr == 0x1004) cmd_word = dut.wb_dat_w;
            if (addr == 0x1008) card_command();
        } else if (addr == 0x101c) {
            next_data = mode == 2 && commands.back() == 8 ? 9 : 1;
        } else if (addr >= 0x100c && addr <= 0x1018) {
            next_data = response_words[3 - (addr - 0x100c) / 4];
        }
    }
    dut.clk = 1;
    dut.eval();
    pending_ack = take && !next_err;
    pending_err = take && next_err;
    pending_data = next_data;
}

static bool run(int test_mode) {
    mode = test_mode;
    pending_ack = false;
    pending_err = false;
    pending_data = 0;
    cmd_arg = 0;
    cmd_word = 0;
    ocr_count = 0;
    writes = 0;
    commands.clear();
    for (auto &word : response_words) word = 0;
    dut.rst = 1;
    tick();
    dut.rst = 0;
    for (int i = 0; i < 2000 && !dut.ready && !dut.error; ++i) tick();
    if (test_mode == 0) {
        const std::vector<int> expected = {0, 8, 55, 41, 55, 41, 2, 3,
                                           9, 7, 55, 6, 16};
        if (!dut.ready || dut.error || dut.card_num_lbas != 8388608 ||
            dut.rca != 0x12340000 || commands != expected) {
            std::fprintf(stderr, "controller init failure: ready=%d error=%d phase=%d lbas=%u commands=%zu\n",
                         int(dut.ready), int(dut.error), int(dut.debug_phase),
                         dut.card_num_lbas, commands.size());
            return false;
        }
    } else if (!dut.error || dut.ready ||
               dut.debug_phase != (test_mode == 1 ? 0 : 3)) {
        std::fprintf(stderr, "controller error handling failure mode=%d ready=%d error=%d phase=%d\n",
                     test_mode, int(dut.ready), int(dut.error), int(dut.debug_phase));
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int test_mode = 0; test_mode < 3; ++test_mode)
        if (!run(test_mode)) return 1;
    std::puts("PASS native SD init controller: Wishbone CSR + command + init, OCR retry, bus/card errors");
    return 0;
}
