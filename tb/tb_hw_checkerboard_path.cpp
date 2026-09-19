// tb_hw_checkerboard_path.cpp -- run checkerboard program through xbar->VRAM.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>
#include <verilated.h>

#include "Vtb_hw_checkerboard_path.h"

static Vtb_hw_checkerboard_path* dut = nullptr;
static uint64_t sim_time = 0;
static uint64_t timeout = 2000000;
static std::string bin_path = "build/hw_smoke/checkerboard_sim.bin";

static constexpr uint32_t ROM_BASE = 0x40800000u;
static constexpr int FB_W = 128;
static constexpr int FB_H = 48;
static constexpr int TILE = 32;
static constexpr int EXPECTED_STORES = (FB_W * FB_H) / 4;

static std::vector<uint8_t> rom(64 * 1024, 0xFF);

static int if_pending = 0;
static uint32_t if_addr_q = 0;

static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { \
        n_pass++; \
        std::printf("  [PASS] %s\n", msg); \
    } else { \
        n_fail++; \
        std::printf("  [FAIL] %s\n", msg); \
    } \
} while (0)

static void parse_args(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; i++) {
        std::string a(argv[i]);
        if (a.rfind("+bin=", 0) == 0) {
            bin_path = a.substr(5);
        } else if (a.rfind("+timeout=", 0) == 0) {
            timeout = std::strtoull(a.substr(9).c_str(), nullptr, 0);
        }
    }
}

static bool load_bin() {
    std::ifstream f(bin_path, std::ios::binary);
    if (!f) return false;
    f.read(reinterpret_cast<char*>(rom.data()), rom.size());
    return f.good() || f.eof();
}

static uint8_t read_rom8(uint32_t addr) {
    uint32_t off = addr - ROM_BASE;
    if (off >= rom.size()) return 0xFF;
    return rom[off];
}

static void set_fetch_line(uint32_t addr) {
    uint32_t base = addr & ~0xFu;
    uint8_t line[16];
    for (int i = 0; i < 16; i++) line[i] = read_rom8(base + i);

    uint32_t w0 = (uint32_t(line[12]) << 24) | (uint32_t(line[13]) << 16)
                | (uint32_t(line[14]) << 8)  |  uint32_t(line[15]);
    uint32_t w1 = (uint32_t(line[8])  << 24) | (uint32_t(line[9])  << 16)
                | (uint32_t(line[10]) << 8)  |  uint32_t(line[11]);
    uint32_t w2 = (uint32_t(line[4])  << 24) | (uint32_t(line[5])  << 16)
                | (uint32_t(line[6])  << 8)  |  uint32_t(line[7]);
    uint32_t w3 = (uint32_t(line[0])  << 24) | (uint32_t(line[1])  << 16)
                | (uint32_t(line[2])  << 8)  |  uint32_t(line[3]);

    dut->if_rdata.at(0) = w0;
    dut->if_rdata.at(1) = w1;
    dut->if_rdata.at(2) = w2;
    dut->if_rdata.at(3) = w3;
}

static void drive_ifetch() {
    dut->if_rvalid = 0;
    dut->if_fault = 0;
    if (dut->if_req && if_pending == 0) {
        if_addr_q = dut->if_addr;
        if_pending = 2;
    }
    if (if_pending > 0) {
        if (--if_pending == 0) {
            set_fetch_line(if_addr_q);
            dut->if_rvalid = 1;
            dut->if_fault = 0;
        }
    }
}

static void tick() {
    dut->clk = 1;
    dut->eval();
    sim_time++;
    dut->clk = 0;
    dut->eval();
    if (sim_time > timeout) {
        std::fprintf(stderr, "[TIMEOUT] exceeded %llu cycles\n",
                     static_cast<unsigned long long>(timeout));
        std::exit(2);
    }
}

static void reset() {
    dut->clk = 0;
    dut->rst = 1;
    dut->if_rvalid = 0;
    dut->if_fault = 0;
    dut->rd_addr = 0;
    dut->rd_en = 0;
    dut->eval();
    for (int i = 0; i < 16; i++) tick();
    dut->rst = 0;
    for (int i = 0; i < 4; i++) {
        drive_ifetch();
        tick();
    }
}

static uint8_t read_pixel(int x, int y) {
    dut->rd_addr = y * FB_W + x;
    dut->rd_en = 1;
    tick();
    dut->rd_en = 0;
    tick();
    // CONSUMER: the byte at rd_addr is the always-valid top lane of the
    // 4-byte group the streaming port now returns.
    uint8_t value = (uint8_t)((dut->rd_data >> 24) & 0xFFu);
    tick();
    return value;
}

static uint8_t expected_pixel(int x, int y) {
    return (((x / TILE) ^ (y / TILE)) & 1) ? 0xFF : 0x00;
}

int main(int argc, char** argv) {
    parse_args(argc, argv);
    if (!load_bin()) {
        std::fprintf(stderr, "[ERROR] failed to load %s\n", bin_path.c_str());
        return 1;
    }

    dut = new Vtb_hw_checkerboard_path;
    reset();

    while (dut->vram_store_count < EXPECTED_STORES && !Verilated::gotFinish()) {
        drive_ifetch();
        tick();
    }

    for (int i = 0; i < 64; i++) {
        drive_ifetch();
        tick();
    }

    std::printf("---- checkerboard path summary ------------\n");
    std::printf("  cycles            = %llu\n", static_cast<unsigned long long>(sim_time));
    std::printf("  committed         = %u\n", dut->dbg_committed);
    std::printf("  last_pc           = 0x%08x\n", dut->dbg_last_pc);
    std::printf("  vram_store_count  = %u\n", dut->vram_store_count);

    CHECK(dut->vram_store_count == EXPECTED_STORES,
          "program completed the expected VRAM store count");

    const int xs[] = {0, 31, 32, 63, 64, 95, 96, 127};
    const int ys[] = {0, 31, 32, 47};
    bool pixels_ok = true;
    for (int y : ys) {
        for (int x : xs) {
            uint8_t got = read_pixel(x, y);
            uint8_t exp = expected_pixel(x, y);
            if (got != exp) {
                std::printf("    pixel(%d,%d): got 0x%02x expected 0x%02x\n",
                            x, y, got, exp);
                pixels_ok = false;
            }
        }
    }
    CHECK(pixels_ok, "scanner reads back the expected 32x32 checkerboard");

    std::printf("---- result -------------------------------\n");
    std::printf("  pass=%d fail=%d\n", n_pass, n_fail);

    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
