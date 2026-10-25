#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vsd_native_block_engine.h"

static Vsd_native_block_engine dut;
static bool csr_ack_pending = false;
static uint32_t csr_data_pending = 0;
static uint32_t command_word = 0;
static uint32_t command_arg = 0;
static int block_count = 0;
static int transfer_words = 0;
static int dma_index = 0;
static bool dma_active = false;
static bool write_test = false;
static int source_pos = 0;
static std::vector<uint8_t> read_bytes;
static std::vector<int> commands;
static bool failed = false;
static int cycles = 0;

static uint32_t pattern_word(int index) {
    uint32_t b = uint32_t(index * 4);
    return ((b & 255) << 24) | (((b + 1) & 255) << 16) |
           (((b + 2) & 255) << 8) | ((b + 3) & 255);
}

static void tick() {
    dut.clk = 0;
    dut.wb_ctrl_ack = csr_ack_pending;
    dut.wb_ctrl_err = 0;
    dut.wb_ctrl_dat_r = csr_data_pending;
    dut.wb_dma_cyc = dma_active && dma_index < transfer_words;
    dut.wb_dma_stb = dut.wb_dma_cyc;
    dut.wb_dma_we = !write_test;
    dut.wb_dma_adr = dma_index;
    dut.wb_dma_dat_w = pattern_word(dma_index);
    dut.wb_dma_sel = 15;
    dut.wr_avail = write_test && source_pos < block_count * 512;
    dut.wr_data = uint8_t(source_pos);
    dut.rd_ready = !write_test && (cycles % 19) < 13;
    dut.eval();
    bool take_csr = !dut.rst && dut.wb_ctrl_cyc && dut.wb_ctrl_stb && !dut.wb_ctrl_ack;
    bool take_write_byte = dut.wr_ready;
    bool take_read_byte = dut.rd_valid;
    uint8_t read_byte = dut.rd_data;
    bool take_dma = dut.wb_dma_ack;
    uint16_t csr_addr = uint16_t(dut.wb_ctrl_adr << 2);
    uint32_t next_csr_data = 0;
    if (take_csr) {
        if (dut.wb_ctrl_we) {
            if (csr_addr == 0x1000) command_arg = dut.wb_ctrl_dat_w;
            if (csr_addr == 0x1004) command_word = dut.wb_ctrl_dat_w;
            if (csr_addr == 0x1028) block_count = dut.wb_ctrl_dat_w;
            if ((csr_addr == 0x0808 || csr_addr == 0x2008) &&
                dut.wb_ctrl_dat_w != uint32_t(block_count * 512)) {
                std::fputs("DMA length mismatch\n", stderr);
                failed = true;
            }
            if (csr_addr == 0x1024 && dut.wb_ctrl_dat_w != 512) {
                std::fputs("SD block length mismatch\n", stderr);
                failed = true;
            }
            if (csr_addr == 0x1008) {
                int cmd = (command_word >> 8) & 63;
                commands.push_back(cmd);
                if (cmd == 17 || cmd == 18 || cmd == 24 || cmd == 25) {
                    int expected_data_type = write_test ? 2 : 1;
                    if (((command_word >> 5) & 3) != expected_data_type ||
                        (command_word & 3) != 1 || command_arg != 8192) {
                        std::fputs("card transfer command encoding mismatch\n", stderr);
                        failed = true;
                    }
                    transfer_words = block_count * 128;
                    dma_index = 0;
                    dma_active = true;
                } else if (cmd == 12 && (((command_word >> 5) & 3) != 0 ||
                                         (command_word & 3) != 3)) {
                    std::fputs("CMD12 encoding mismatch\n", stderr);
                    failed = true;
                }
            }
        } else if (csr_addr == 0x101c) {
            next_csr_data = 1;
        } else if (csr_addr == 0x1020 || csr_addr == 0x0810 || csr_addr == 0x2010) {
            next_csr_data = dma_active && dma_index >= transfer_words ? 1 : 0;
        }
    }
    dut.clk = 1;
    dut.eval();
    if (take_write_byte) ++source_pos;
    if (take_read_byte) read_bytes.push_back(read_byte);
    if (take_dma) {
        if (dut.wb_dma_err) {
            std::fputs("DMA address error\n", stderr);
            failed = true;
        }
        if (write_test && dut.wb_dma_dat_r != pattern_word(dma_index)) {
            std::fprintf(stderr, "DMA word mismatch index=%d got=%08x expected=%08x\n",
                         dma_index, dut.wb_dma_dat_r, pattern_word(dma_index));
            failed = true;
        }
        ++dma_index;
    }
    csr_ack_pending = take_csr;
    csr_data_pending = next_csr_data;
    ++cycles;
}

static bool run(bool write, int blocks) {
    csr_ack_pending = false;
    csr_data_pending = 0;
    command_word = 0;
    command_arg = 0;
    block_count = blocks;
    transfer_words = 0;
    dma_index = 0;
    dma_active = false;
    write_test = write;
    source_pos = 0;
    read_bytes.clear();
    commands.clear();
    failed = false;
    cycles = 0;
    dut.rst = 1;
    dut.go = 0;
    dut.cmd_type = write ? (blocks > 1 ? 4 : 3) : (blocks > 1 ? 2 : 1);
    dut.block_count = blocks;
    dut.lba = 8192;
    for (int i = 0; i < 6; ++i) tick();
    dut.rst = 0;
    tick();
    dut.go = 1;
    tick();
    dut.go = 0;
    for (int i = 0; i < 200000 && !dut.done && !failed; ++i) tick();
    if (!dut.done || dut.error || failed || dut.debug_dma_words != blocks * 128 ||
        dma_index != blocks * 128 || block_count != blocks) {
        std::fprintf(stderr, "block transfer failed: write=%d blocks=%d done=%d error=%d dma=%d/%d debug=%d cycles=%d\n",
                     write, blocks, int(dut.done), int(dut.error), dma_index,
                     blocks * 128, int(dut.debug_dma_words), cycles);
        return false;
    }
    const std::vector<int> expected_commands = blocks > 1 ?
        std::vector<int>{write ? 25 : 18, 12} :
        std::vector<int>{write ? 24 : 17};
    if (commands != expected_commands || command_arg != (blocks > 1 ? 0u : 8192u)) {
        std::fputs("card command sequence mismatch\n", stderr);
        return false;
    }
    if (write) {
        if (source_pos != blocks * 512) {
            std::fprintf(stderr, "write source count=%d\n", source_pos);
            return false;
        }
    } else {
        if (read_bytes.size() != size_t(blocks * 512)) {
            std::fprintf(stderr, "read byte count=%zu expected=%d\n",
                         read_bytes.size(), blocks * 512);
            return false;
        }
        for (int i = 0; i < blocks * 512; ++i)
            if (read_bytes[i] != uint8_t(i)) {
                std::fprintf(stderr, "read byte mismatch at %d got=%02x\n",
                             i, read_bytes[i]);
                return false;
            }
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (!run(false, 1) || !run(true, 1) ||
        !run(false, 32) || !run(true, 32)) return 1;
    std::puts("PASS native SD block engine: 1/32-sector read/write, DMA RAM, ordered bytes, CMD12");
    return 0;
}
