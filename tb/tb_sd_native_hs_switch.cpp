#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vsd_native_hs_switch.h"

static Vsd_native_hs_switch dut;
static bool ack_pending = false;
static bool err_pending = false;
static uint32_t data_pending = 0;
static uint32_t command_arg = 0, command_word = 0;
static bool command_sent = false;
static int dma_step = 0, mode = 0;
static int divider_writes = 0;
static std::vector<uint16_t> writes;

static void tick() {
    dut.clk = 0;
    dut.wb_ctrl_ack = ack_pending;
    dut.wb_ctrl_err = err_pending;
    dut.wb_ctrl_dat_r = data_pending;
    dut.wb_dma_cyc = command_sent && dma_step < (mode == 2 ? 1 : 2) &&
                     (dma_step == 0 || !dut.wb_dma_ack);
    dut.wb_dma_stb = dut.wb_dma_cyc;
    dut.wb_dma_we = 1;
    dut.wb_dma_sel = 15;
    dut.wb_dma_adr = dma_step == 0 ? 3 : 4;
    dut.wb_dma_dat_w = dma_step == 0 ?
        (mode == 1 ? 0u : 0x00000380u) :
        (mode == 1 ? 0u : 0x00000001u);
    dut.eval();
    const bool take = !dut.rst && dut.wb_ctrl_cyc && dut.wb_ctrl_stb &&
                      !dut.wb_ctrl_ack && !dut.wb_ctrl_err;
    const uint16_t addr = uint16_t(dut.wb_ctrl_adr << 2);
    const bool dma_take = dut.wb_dma_cyc && dut.wb_dma_stb && !dut.wb_dma_ack;
    bool next_err = false;
    uint32_t next_data = 0;
    if (take) {
        if (dut.wb_ctrl_we) {
            writes.push_back(addr);
            if (addr == 0x1000) command_arg = dut.wb_ctrl_dat_w;
            if (addr == 0x1004) command_word = dut.wb_ctrl_dat_w;
            if (addr == 0x1008) command_sent = true;
            if (addr == 0x2804) {
                ++divider_writes;
                if (mode == 4) next_err = true;
                if (dut.wb_ctrl_dat_w != 2) next_err = true;
            }
        } else if (addr == 0x101c) {
            next_data = mode == 3 ? 0x5 : 1;
        } else if (addr == 0x1020) {
            next_data = mode == 5 ? 0 : 1;
        } else if (addr == 0x0810) {
            next_data = dma_step >= (mode == 2 ? 1 : 2) ? 1 : 0;
        }
    }
    dut.clk = 1;
    dut.eval();
    ack_pending = take && !next_err;
    err_pending = take && next_err;
    data_pending = next_data;
    if (dma_take) ++dma_step;
}

static bool run(int test_mode) {
    mode = test_mode;
    ack_pending = err_pending = command_sent = false;
    data_pending = command_arg = command_word = 0;
    dma_step = divider_writes = 0;
    writes.clear();
    dut.rst = 1;
    dut.wb_dma_cyc = 0;
    dut.wb_dma_stb = 0;
    tick();
    dut.rst = 0;
    for (int i = 0; i < 3000 && !dut.done && !dut.error; ++i) tick();
    const bool should_error = test_mode >= 2;
    const bool should_hs = test_mode == 0;
    if (bool(dut.error) != should_error ||
        bool(dut.done) != !should_error ||
        bool(dut.high_speed_active) != should_hs ||
        command_arg != 0x80fffff1u ||
        command_word != ((6u << 8) | (1u << 5) | 1u) ||
        divider_writes != (test_mode == 0 || test_mode == 4 ? 1 : 0)) {
        std::fprintf(stderr,
            "HS mode %d failed: done=%d error=%d active=%d dma=%d divider=%d arg=%08x cmd=%08x\n",
            test_mode, int(dut.done), int(dut.error),
            int(dut.high_speed_active), dma_step, divider_writes,
            command_arg, command_word);
        return false;
    }
    const std::vector<uint16_t> setup =
        {0x080c, 0x0800, 0x0804, 0x0808, 0x080c, 0x1024, 0x1028};
    if (writes.size() < setup.size() ||
        !std::equal(setup.begin(), setup.end(), writes.begin())) {
        std::fprintf(stderr, "HS mode %d DMA setup sequence failed\n", mode);
        return false;
    }
    return true;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    for (int test_mode = 0; test_mode < 6; ++test_mode)
        if (!run(test_mode)) return 1;
    std::puts("PASS native SD CMD6: confirmed 50 MHz, unsupported fallback, missing DMA, card/CSR errors, timeout");
    return 0;
}
