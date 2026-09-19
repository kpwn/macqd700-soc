#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <verilated.h>
#include "Vfpga_top.h"
#include "Vfpga_top___024root.h"

static Vfpga_top* dut = nullptr;
static uint64_t sim_time = 0;

static constexpr uint32_t ROM_BYTES = 0x00400000u;
static constexpr uint32_t RAM_BYTES = 0x04000000u;
static constexpr uint32_t ROM_BEAT_BASE = RAM_BYTES / 16u;

static void tick() {
    dut->sys_clk_p = 0;
    dut->sys_clk_n = 1;
    dut->eval();
    sim_time++;
    dut->sys_clk_p = 1;
    dut->sys_clk_n = 0;
    dut->eval();
    sim_time++;
}

static bool load_file(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot open ROM: %s\n", path.c_str());
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(in),
               std::istreambuf_iterator<char>());
    return true;
}

static void ddr_write_byte(uint32_t byte_off, uint8_t v) {
    const uint32_t beat = ROM_BEAT_BASE + (byte_off >> 4);
    const uint32_t byte_in_beat = byte_off & 0xfu;
    const uint32_t lane = (byte_in_beat & ~3u) | (3u - (byte_in_beat & 3u));
    switch (lane) {
    case 0:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b0 [beat] = v; break;
    case 1:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b1 [beat] = v; break;
    case 2:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b2 [beat] = v; break;
    case 3:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b3 [beat] = v; break;
    case 4:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b4 [beat] = v; break;
    case 5:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b5 [beat] = v; break;
    case 6:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b6 [beat] = v; break;
    case 7:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b7 [beat] = v; break;
    case 8:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b8 [beat] = v; break;
    case 9:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b9 [beat] = v; break;
    case 10: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b10[beat] = v; break;
    case 11: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b11[beat] = v; break;
    case 12: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b12[beat] = v; break;
    case 13: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b13[beat] = v; break;
    case 14: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b14[beat] = v; break;
    default: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b15[beat] = v; break;
    }
}

static bool preload_rom(const std::string& path) {
    std::vector<uint8_t> rom;
    if (!load_file(path, rom)) return false;
    if (rom.size() > ROM_BYTES) return false;
    for (uint32_t i = 0; i < ROM_BYTES; i++)
        ddr_write_byte(i, i < rom.size() ? rom[i] : 0);
    std::fprintf(stderr, "[probe] preloaded %zu bytes\n", rom.size());
    return true;
}

static uint32_t arch_reg(int idx) {
    const uint8_t phys = dut->rootp->fpga_top__DOT__u_cpu__DOT__u_rat__DOT__crat[idx];
    return dut->rootp->fpga_top__DOT__u_cpu__DOT__prf[phys];
}

static void print_regs(const char* tag) {
    std::fprintf(stderr,
        "%s t=%llu ret=%u pc=%08x bpc=%08x npc=%08x kind=%u "
        "D0=%08x D2=%08x D3=%08x A0=%08x A1=%08x A2=%08x A4=%08x A6=%08x A7=%08x "
        "SR=%04x CCR=%02x VBR=%08x\n",
        tag,
        (unsigned long long)sim_time,
        (unsigned)dut->rootp->fpga_top__DOT__dbg_committed,
        (unsigned)dut->rootp->fpga_top__DOT__dbg_pc,
        (unsigned)dut->rootp->fpga_top__DOT__dbg_boundary_pc,
        (unsigned)dut->rootp->fpga_top__DOT__dbg_boundary_next_pc,
        (unsigned)dut->rootp->fpga_top__DOT__dbg_boundary_kind,
        arch_reg(0), arch_reg(2), arch_reg(3),
        arch_reg(8), arch_reg(9), arch_reg(10), arch_reg(12),
        arch_reg(14), arch_reg(15),
        (unsigned)dut->rootp->fpga_top__DOT__u_cpu__DOT__u_commit__DOT__arch_sr,
        (unsigned)dut->rootp->fpga_top__DOT__dbg_ccr,
        (unsigned)dut->rootp->fpga_top__DOT__u_cpu__DOT__u_commit__DOT__arch_vbr);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    uint64_t max_retired = 50000;
    if (argc > 1) max_retired = std::strtoull(argv[1], nullptr, 0);

    dut = new Vfpga_top;
    dut->cpu_resetn = 0;
    dut->btn = 0;
    dut->uart_rtl_0_rxd = 1;
    dut->sd_miso = 1;
    dut->al9134_int = 0;
    for (int i = 0; i < 64; i++) tick();
    if (!preload_rom("files/420dbff3.rom")) return 1;
    dut->cpu_resetn = 1;

    uint32_t last_retired = 0;
    uint32_t last_ar = 0;
    uint32_t prints = 0;
    while (!Verilated::gotFinish() && sim_time < 200000000ULL) {
        tick();

        if (dut->rootp->fpga_top__DOT__u_cpu__DOT__daxi_arvalid &&
            dut->rootp->fpga_top__DOT__u_cpu__DOT__daxi_arready) {
            last_ar = dut->rootp->fpga_top__DOT__u_cpu__DOT__daxi_araddr;
            if ((last_ar < 0x1000) || ((last_ar & 0x0ffff000u) == 0x00002000u) ||
                ((last_ar & 0xffff0000u) == 0x40000000u) ||
                ((last_ar & 0xffff0000u) == 0x40800000u)) {
                std::fprintf(stderr, "AR t=%llu ret=%u addr=%08x\n",
                    (unsigned long long)sim_time,
                    (unsigned)dut->rootp->fpga_top__DOT__dbg_committed,
                    last_ar);
            }
        }
        if (dut->rootp->fpga_top__DOT__u_cpu__DOT__daxi_rvalid &&
            dut->rootp->fpga_top__DOT__u_cpu__DOT__daxi_rready) {
            if ((last_ar < 0x1000) || ((last_ar & 0x0ffff000u) == 0x00002000u) ||
                ((last_ar & 0xffff0000u) == 0x40000000u) ||
                ((last_ar & 0xffff0000u) == 0x40800000u)) {
                std::fprintf(stderr, " R t=%llu ret=%u addr=%08x data=%08x\n",
                    (unsigned long long)sim_time,
                    (unsigned)dut->rootp->fpga_top__DOT__dbg_committed,
                    last_ar,
                    (unsigned)dut->rootp->fpga_top__DOT__u_cpu__DOT__daxi_rdata);
            }
        }

        const uint32_t retired = dut->rootp->fpga_top__DOT__dbg_committed;
        if (retired != last_retired) {
            const uint32_t bpc = dut->rootp->fpga_top__DOT__dbg_boundary_pc & 0x00ffffffu;
            if (retired < 80 || (bpc >= 0x2e20 && bpc < 0x2e70) ||
                (bpc >= 0x3da0 && bpc < 0x3dc0)) {
                print_regs("RET");
                if (++prints > 500) break;
            }
            last_retired = retired;
            if (retired >= max_retired) break;
        }
    }
    print_regs("STOP");
    dut->final();
    delete dut;
    return 0;
}
