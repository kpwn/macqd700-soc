// tb_decode_probe.cpp -- batch-probe rtl/core/decode/decode.v.
//
// This is intentionally not a pass/fail unit test.  It feeds ROM bytes at a
// caller-provided address list into the current RTL decoder and prints one CSV
// row per address so host-side tools can identify vec-4 fallback gaps.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <verilated.h>
#include "Vdecode.h"

static std::string plus_arg(int argc, char** argv, const std::string& prefix,
                            const std::string& fallback = "") {
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg.rfind(prefix, 0) == 0)
            return arg.substr(prefix.size());
    }
    return fallback;
}

static uint32_t parse_u32(const std::string& s) {
    size_t idx = 0;
    return static_cast<uint32_t>(std::stoul(s, &idx, 0));
}

static std::vector<uint8_t> read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "tb_decode_probe: cannot open %s\n", path.c_str());
        std::exit(2);
    }
    f.seekg(0, std::ios::end);
    std::streamoff n = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<uint8_t> data(static_cast<size_t>(n));
    if (n > 0)
        f.read(reinterpret_cast<char*>(data.data()), n);
    return data;
}

static std::vector<uint32_t> read_addrs(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "tb_decode_probe: cannot open %s\n", path.c_str());
        std::exit(2);
    }

    std::vector<uint32_t> addrs;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream iss(line);
        std::string tok;
        if (!(iss >> tok))
            continue;
        addrs.push_back(parse_u32(tok));
    }
    return addrs;
}

static void tick(Vdecode* dut) {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
}

static void reset(Vdecode* dut) {
    dut->rst = 1;
    dut->pd_valid = 0;
    dut->pd_fault = 0;
    dut->pd_next_fault = 0;
    dut->rn_ready = 1;
    dut->flush_en = 0;
    dut->pd_pc = 0;
    for (int i = 0; i < 4; i++)
        dut->pd_buf[i] = 0;
    tick(dut);
    tick(dut);
    dut->rst = 0;
    dut->eval();
}

static bool rom_offset(uint32_t addr, uint32_t base, size_t rom_size, size_t& off) {
    if (addr >= base && static_cast<uint64_t>(addr - base) < rom_size) {
        off = static_cast<size_t>(addr - base);
        return true;
    }

    // Q700 boot traces may still show the low 0x4000_0000 mirror.  If the
    // caller uses the high 0x4080_0000 ROM base, accept mirror addresses too.
    if (base == 0x40800000u && addr >= 0x40000000u &&
        static_cast<uint64_t>(addr - 0x40000000u) < rom_size) {
        off = static_cast<size_t>(addr - 0x40000000u);
        return true;
    }

    return false;
}

static uint16_t read_be16(const std::vector<uint8_t>& rom, size_t off) {
    if (off + 1 >= rom.size())
        return 0;
    return (static_cast<uint16_t>(rom[off]) << 8) | rom[off + 1];
}

static void set_pd_buf(Vdecode* dut, const std::vector<uint8_t>& rom, size_t off) {
    uint8_t bytes[16] = {0};
    for (int i = 0; i < 16; i++) {
        size_t idx = off + static_cast<size_t>(i);
        if (idx < rom.size())
            bytes[i] = rom[idx];
    }

    for (int w = 0; w < 4; w++) {
        int base = (3 - w) * 4;
        dut->pd_buf[w] =
            (static_cast<uint32_t>(bytes[base + 0]) << 24) |
            (static_cast<uint32_t>(bytes[base + 1]) << 16) |
            (static_cast<uint32_t>(bytes[base + 2]) << 8) |
            (static_cast<uint32_t>(bytes[base + 3]) << 0);
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    std::string rom_path = plus_arg(argc, argv, "+rom=");
    std::string addr_path = plus_arg(argc, argv, "+addr_file=");
    uint32_t base = parse_u32(plus_arg(argc, argv, "+base=", "0x40800000"));

    if (rom_path.empty() || addr_path.empty()) {
        std::fprintf(stderr,
                     "usage: Vdecode +rom=<rom.bin> +addr_file=<addr.txt> "
                     "[+base=0x40800000]\n");
        return 2;
    }

    std::vector<uint8_t> rom = read_file(rom_path);
    std::vector<uint32_t> addrs = read_addrs(addr_path);

    Vdecode* dut = new Vdecode;
    reset(dut);

    std::puts("addr,opword,uop_valid,uop_type,uop_op,exc_valid,exc_vec,exc_fault_addr,requires_supervisor,len_bytes,pd_consumed");

    for (uint32_t addr : addrs) {
        size_t off = 0;
        if (!rom_offset(addr, base, rom.size(), off))
            continue;

        dut->flush_en = 1;
        tick(dut);
        dut->flush_en = 0;
        dut->pd_valid = 1;
        dut->pd_fault = 0;
        dut->pd_next_fault = 0;
        dut->rn_ready = 1;
        dut->pd_pc = addr;
        set_pd_buf(dut, rom, off);
        dut->eval();

        uint16_t opword = read_be16(rom, off);
        uint32_t len_bytes = dut->uop_npc - dut->uop_pc;
        std::printf("0x%08x,0x%04x,%u,%u,%u,%u,%u,0x%08x,%u,%u,%u\n",
                    addr, opword,
                    static_cast<unsigned>(dut->uop_valid),
                    static_cast<unsigned>(dut->uop_type),
                    static_cast<unsigned>(dut->uop_op),
                    static_cast<unsigned>(dut->exc_valid),
                    static_cast<unsigned>(dut->exc_vec),
                    static_cast<unsigned>(dut->exc_fault_addr),
                    static_cast<unsigned>(dut->requires_supervisor),
                    static_cast<unsigned>(len_bytes),
                    static_cast<unsigned>(dut->pd_consumed));
    }

    delete dut;
    return 0;
}
