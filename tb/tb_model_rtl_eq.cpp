// tb_model_rtl_eq.cpp — model-vs-RTL equality checker for Q700 map contracts.
//
// This checker compares the host ROM-bus model (`RomBootBus`) against the
// RTL-observable glue decode contract (`rtl/mac/glue.v`) on high-impact
// surfaces:
//   - memory-map/decode categories
//   - overlay/ROM-window behavior transitions
//   - peripheral register baseline behavior in the host model
//
// Built via: make tb-model-rtl-eq

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <verilated.h>

#include "Vglue.h"
#include "models/rom_boot_bus.h"

namespace {

constexpr uint8_t CLASS_UNMAP = 0;
constexpr uint8_t CLASS_RAM = 1;
constexpr uint8_t CLASS_ROM = 2;
constexpr uint8_t CLASS_IO = 3;
constexpr uint8_t CLASS_VIDEO = 4;

constexpr size_t kCategoryCount =
    static_cast<size_t>(RomBootBus::Category::Count);

enum class CsExpect {
    None,
    Ram,
    Rom,
    Via1,
    Via2,
    Enet,
    Sonic,
    Scc,
    Orwell,
    Scsi,
    Asc,
    Iwm,
    Video
};

struct GlueObs {
    uint32_t out_addr = 0;
    uint8_t out_class = 0;
    bool cs_ram = false;
    bool cs_rom = false;
    bool cs_via1 = false;
    bool cs_via2 = false;
    bool cs_enet = false;
    bool cs_sonic = false;
    bool cs_scc = false;
    bool cs_orwell = false;
    bool cs_scsi = false;
    bool cs_asc = false;
    bool cs_iwm = false;
    bool cs_video = false;
    bool fault = false;
};

struct DecodeCase {
    const char* name;
    uint32_t addr;
    bool overlay;
    RomBootBus::Category model_cat;
    uint8_t glue_class;
    CsExpect glue_cs;
    bool glue_fault;
};

static int g_failures = 0;

static void fail_msg(const char* scope, const char* msg) {
    std::printf("[FAIL] %s: %s\n", scope, msg);
    g_failures++;
}

template <typename T, typename U>
static void check_eq(const char* scope, const char* what, T got, U exp) {
    if (got != static_cast<T>(exp)) {
        std::printf("[FAIL] %s: %s got=0x%llx exp=0x%llx\n",
                    scope,
                    what,
                    static_cast<unsigned long long>(got),
                    static_cast<unsigned long long>(exp));
        g_failures++;
    }
}

static void check_true(const char* scope, const char* what, bool cond) {
    if (!cond) {
        std::printf("[FAIL] %s: %s\n", scope, what);
        g_failures++;
    }
}

static std::array<uint64_t, kCategoryCount> snapshot_reads(
    const RomBootBus& bus) {
    std::array<uint64_t, kCategoryCount> out{};
    for (size_t i = 0; i < kCategoryCount; i++) {
        auto cat = static_cast<RomBootBus::Category>(i);
        out[i] = bus.stats(cat).reads;
    }
    return out;
}

static bool read_category(RomBootBus& bus, uint32_t addr,
                          RomBootBus::Category* out_cat) {
    if (!out_cat) return false;
    const auto before = snapshot_reads(bus);
    (void)bus.read8(addr);
    const auto after = snapshot_reads(bus);

    int hit = -1;
    for (size_t i = 0; i < kCategoryCount; i++) {
        if (after[i] == before[i] + 1u) {
            if (hit >= 0) return false;
            hit = static_cast<int>(i);
        } else if (after[i] != before[i]) {
            return false;
        }
    }
    if (hit < 0) return false;
    *out_cat = static_cast<RomBootBus::Category>(hit);
    return true;
}

static std::vector<uint8_t> synth_rom() {
    std::vector<uint8_t> rom(RomBootBus::Q700_ROM_SIZE, 0xffu);
    rom[0] = 0x42;
    rom[1] = 0x0d;
    rom[2] = 0xbf;
    rom[3] = 0xf3;
    rom[4] = 0x00;
    rom[5] = 0x00;
    rom[6] = 0x00;
    rom[7] = 0x2a;
    rom[0x2a] = 0x4e;
    rom[0x2b] = 0x71;
    return rom;
}

static bool init_bus(RomBootBus& bus, bool overlay) {
    if (!bus.set_rom_image(synth_rom(), "tb-model-rtl-eq")) return false;
    bus.set_overlay_active(overlay);
    bus.set_stop_on_unshared_io(false);
    return true;
}

static GlueObs decode_glue(Vglue& dut, uint32_t addr, bool overlay,
                           bool write = false) {
    dut.req_addr = addr;
    dut.req_valid = 1;
    dut.req_rw = write ? 1 : 0;
    dut.req_size = 2;
    dut.req_fc = 0b101;
    dut.overlay_in = overlay ? 1 : 0;
    dut.eval();

    GlueObs d{};
    d.out_addr = dut.out_addr;
    d.out_class = dut.out_class;
    d.cs_ram = dut.cs_ram;
    d.cs_rom = dut.cs_rom;
    d.cs_via1 = dut.cs_via1;
    d.cs_via2 = dut.cs_via2;
    d.cs_enet = dut.cs_enet;
    d.cs_sonic = dut.cs_sonic;
    d.cs_scc = dut.cs_scc;
    d.cs_orwell = dut.cs_orwell;
    d.cs_scsi = dut.cs_scsi;
    d.cs_asc = dut.cs_asc;
    d.cs_iwm = dut.cs_iwm;
    d.cs_video = dut.cs_video;
    d.fault = dut.fault;
    return d;
}

static int active_cs_count(const GlueObs& d) {
    return (d.cs_ram ? 1 : 0) + (d.cs_rom ? 1 : 0) + (d.cs_via1 ? 1 : 0) +
           (d.cs_via2 ? 1 : 0) + (d.cs_enet ? 1 : 0) +
           (d.cs_sonic ? 1 : 0) + (d.cs_scc ? 1 : 0) +
           (d.cs_orwell ? 1 : 0) + (d.cs_scsi ? 1 : 0) +
           (d.cs_asc ? 1 : 0) + (d.cs_iwm ? 1 : 0) +
           (d.cs_video ? 1 : 0);
}

static bool cs_line(const GlueObs& d, CsExpect cs) {
    switch (cs) {
        case CsExpect::None: return active_cs_count(d) == 0;
        case CsExpect::Ram: return d.cs_ram && active_cs_count(d) == 1;
        case CsExpect::Rom: return d.cs_rom && active_cs_count(d) == 1;
        case CsExpect::Via1: return d.cs_via1 && active_cs_count(d) == 1;
        case CsExpect::Via2: return d.cs_via2 && active_cs_count(d) == 1;
        case CsExpect::Enet: return d.cs_enet && active_cs_count(d) == 1;
        case CsExpect::Sonic: return d.cs_sonic && active_cs_count(d) == 1;
        case CsExpect::Scc: return d.cs_scc && active_cs_count(d) == 1;
        case CsExpect::Orwell: return d.cs_orwell && active_cs_count(d) == 1;
        case CsExpect::Scsi: return d.cs_scsi && active_cs_count(d) == 1;
        case CsExpect::Asc: return d.cs_asc && active_cs_count(d) == 1;
        case CsExpect::Iwm: return d.cs_iwm && active_cs_count(d) == 1;
        case CsExpect::Video: return d.cs_video && active_cs_count(d) == 1;
    }
    return false;
}

static bool run_decode_suite(Vglue& dut) {
    const DecodeCase cases[] = {
        { "overlay-low-rom", 0x00000000u, true, RomBootBus::Category::Rom,
          CLASS_ROM, CsExpect::Rom, false },
        { "low-ram-visible", 0x00000000u, false, RomBootBus::Category::Ram,
          CLASS_RAM, CsExpect::Ram, false },
        { "rom-window-base", 0x40000020u, false, RomBootBus::Category::Rom,
          CLASS_ROM, CsExpect::Rom, false },
        { "rom-window-mid", 0x4008fff0u, false, RomBootBus::Category::Rom,
          CLASS_ROM, CsExpect::Rom, false },
        { "via1-canonical", RomBootBus::VIA1_BASE, false,
          RomBootBus::Category::Via1, CLASS_IO, CsExpect::Via1, false },
        { "via1-mirror", 0x50f01c00u, false, RomBootBus::Category::Via1,
          CLASS_IO, CsExpect::Via1, false },
        { "via2-canonical", RomBootBus::VIA2_BASE, false,
          RomBootBus::Category::Via2, CLASS_IO, CsExpect::Via2, false },
        { "enet-canonical", 0x50008000u, false,
          RomBootBus::Category::Enet, CLASS_IO, CsExpect::Enet, false },
        { "sonic-canonical", 0x5000a000u, false,
          RomBootBus::Category::Sonic, CLASS_IO, CsExpect::Sonic, false },
        { "scc-canonical", RomBootBus::SCC_BASE, false,
          RomBootBus::Category::Scc, CLASS_IO, CsExpect::Scc, false },
        { "orwell-canonical", 0x5000e000u, false,
          RomBootBus::Category::Orwell, CLASS_IO, CsExpect::Orwell, false },
        { "scsi-canonical", RomBootBus::TURBOSCSI_BASE, false,
          RomBootBus::Category::Scsi, CLASS_IO, CsExpect::Scsi, false },
        { "asc-canonical", RomBootBus::ASC_BASE, false,
          RomBootBus::Category::Asc, CLASS_IO, CsExpect::Asc, false },
        { "swim-canonical", RomBootBus::SWIM_BASE, false,
          RomBootBus::Category::Swim, CLASS_IO, CsExpect::Iwm, false },
        { "dafb-reg-base", RomBootBus::DAFB_REG_BASE, false,
          RomBootBus::Category::DafbReg, CLASS_VIDEO, CsExpect::Video, false },
        { "vram-aperture", MemModel::VRAM_BASE + 0x100u, false,
          RomBootBus::Category::Vram, CLASS_VIDEO, CsExpect::None, false },
        { "io-gap-fault", 0x50004000u, false, RomBootBus::Category::Unmapped,
          CLASS_IO, CsExpect::None, true },
        { "unmapped-top-fault", 0x80000000u, false,
          RomBootBus::Category::Unmapped, CLASS_UNMAP, CsExpect::None, true },
    };

    const int fail_start = g_failures;
    for (const auto& tc : cases) {
        const int case_fail_before = g_failures;
        RomBootBus bus;
        if (!init_bus(bus, tc.overlay)) {
            fail_msg(tc.name, "failed to init RomBootBus");
            continue;
        }

        RomBootBus::Category cat = RomBootBus::Category::Count;
        if (!read_category(bus, tc.addr, &cat)) {
            fail_msg(tc.name, "could not infer model category");
            continue;
        }
        if (cat != tc.model_cat) {
            std::printf("[FAIL] %s: model category got=%s exp=%s\n",
                        tc.name,
                        RomBootBus::category_name(cat),
                        RomBootBus::category_name(tc.model_cat));
            g_failures++;
        }

        const GlueObs d = decode_glue(dut, tc.addr, tc.overlay, false);
        check_eq(tc.name, "glue class", d.out_class, tc.glue_class);
        check_eq(tc.name, "glue fault", d.fault ? 1u : 0u,
                 tc.glue_fault ? 1u : 0u);
        if (!cs_line(d, tc.glue_cs)) {
            std::printf(
                "[FAIL] %s: glue cs mismatch (ram=%d rom=%d via1=%d via2=%d "
                "enet=%d sonic=%d scc=%d orwell=%d scsi=%d asc=%d "
                "iwm=%d video=%d)\n",
                tc.name,
                d.cs_ram ? 1 : 0,
                d.cs_rom ? 1 : 0,
                d.cs_via1 ? 1 : 0,
                d.cs_via2 ? 1 : 0,
                d.cs_enet ? 1 : 0,
                d.cs_sonic ? 1 : 0,
                d.cs_scc ? 1 : 0,
                d.cs_orwell ? 1 : 0,
                d.cs_scsi ? 1 : 0,
                d.cs_asc ? 1 : 0,
                d.cs_iwm ? 1 : 0,
                d.cs_video ? 1 : 0);
            g_failures++;
        }

        if (g_failures == case_fail_before) {
            std::printf("[PASS] %s\n", tc.name);
        }
    }

    return g_failures == fail_start;
}

static bool run_overlay_suite(Vglue& dut) {
    const int fail_start = g_failures;
    const char* scope = "overlay-rom-window";

    RomBootBus bus;
    if (!init_bus(bus, true)) {
        fail_msg(scope, "failed to init RomBootBus");
        return false;
    }

    bus.write32(0x00000000u, 0x11223344u);
    check_eq(scope, "overlay read low word", bus.read32(0x00000000u),
             0x420dbff3u);

    const GlueObs g_ovl_on = decode_glue(dut, 0x00000000u, true, false);
    check_eq(scope, "glue class while overlay=1", g_ovl_on.out_class,
             CLASS_ROM);
    check_eq(scope, "glue aliased addr while overlay=1", g_ovl_on.out_addr,
             0x40000000u);

    bus.note_instruction_fetch(RomBootBus::Q700_RESET_PC);
    check_true(scope, "model overlay cleared on high-ROM fetch",
               !bus.overlay_active());

    const GlueObs g_ovl_off = decode_glue(dut, 0x00000000u, false, false);
    check_eq(scope, "glue class while overlay=0", g_ovl_off.out_class,
             CLASS_RAM);
    check_eq(scope, "glue pass-through addr while overlay=0", g_ovl_off.out_addr,
             0x00000000u);
    check_eq(scope, "ram visible after overlay clear", bus.read32(0x00000000u),
             0x11223344u);

    RomBootBus via_bus;
    if (!init_bus(via_bus, true)) {
        fail_msg(scope, "failed to init VIA overlay bus");
        return false;
    }
    via_bus.write8(RomBootBus::VIA1_BASE + 0x400u, 0x08u);
    check_true(scope, "VIA1 DDRB[3]=1 clears overlay in model",
               !via_bus.overlay_active());
    const GlueObs g_via = decode_glue(dut, RomBootBus::VIA1_BASE + 0x400u,
                                      true, true);
    check_eq(scope, "VIA1 decode class", g_via.out_class, CLASS_IO);
    check_true(scope, "VIA1 decode chip-select", g_via.cs_via1);

    if (g_failures == fail_start)
        std::printf("[PASS] %s\n", scope);
    return g_failures == fail_start;
}

static bool run_peripheral_baseline_suite(Vglue& dut) {
    const int fail_start = g_failures;
    const char* scope = "peripheral-baseline";

    RomBootBus bus;
    if (!init_bus(bus, false)) {
        fail_msg(scope, "failed to init RomBootBus");
        return false;
    }

    struct Probe {
        const char* name;
        uint32_t addr;
        uint32_t expected;
        RomBootBus::Category cat;
        uint8_t glue_class;
        CsExpect cs;
    };
    const Probe probes[] = {
        { "via1-orb-idle", RomBootBus::VIA1_BASE, 0x09u,
          RomBootBus::Category::Via1, CLASS_IO, CsExpect::Via1 },
        { "via2-orb-idle", RomBootBus::VIA2_BASE, 0xc7u,
          RomBootBus::Category::Via2, CLASS_IO, CsExpect::Via2 },
        { "enet-idle", 0x50008000u, 0x00u,
          RomBootBus::Category::Enet, CLASS_IO, CsExpect::Enet },
        { "sonic-idle", 0x5000a000u, 0x00u,
          RomBootBus::Category::Sonic, CLASS_IO, CsExpect::Sonic },
        { "scc-rr0-idle", RomBootBus::SCC_BASE, 0x6cu,
          RomBootBus::Category::Scc, CLASS_IO, CsExpect::Scc },
        { "orwell-reset", 0x5000e000u, 0x00u,
          RomBootBus::Category::Orwell, CLASS_IO, CsExpect::Orwell },
        { "scsi-reg0-reset", RomBootBus::TURBOSCSI_BASE, 0x00u,
          RomBootBus::Category::Scsi, CLASS_IO, CsExpect::Scsi },
        { "asc-version", RomBootBus::ASC_BASE + 0x800u, 0xbcu,
          RomBootBus::Category::Asc, CLASS_IO, CsExpect::Asc },
        { "swim-iwm-idle", RomBootBus::SWIM_BASE, 0xffu,
          RomBootBus::Category::Swim, CLASS_IO, CsExpect::Iwm },
        { "dafb-sense-lsb", RomBootBus::DAFB_REG_BASE + 0x203u, 0x07u,
          RomBootBus::Category::DafbReg, CLASS_VIDEO, CsExpect::Video },
    };

    for (const auto& p : probes) {
        const int probe_fail_before = g_failures;
        RomBootBus::Category cat = RomBootBus::Category::Count;
        if (!read_category(bus, p.addr, &cat)) {
            std::printf("[FAIL] %s: could not infer category\n", p.name);
            g_failures++;
            continue;
        }
        if (cat != p.cat) {
            std::printf("[FAIL] %s: category got=%s exp=%s\n",
                        p.name,
                        RomBootBus::category_name(cat),
                        RomBootBus::category_name(p.cat));
            g_failures++;
        }

        const uint32_t got = bus.read8(p.addr);
        check_eq(p.name, "model read8", got, p.expected);

        const GlueObs d = decode_glue(dut, p.addr, false, false);
        check_eq(p.name, "glue class", d.out_class, p.glue_class);
        check_eq(p.name, "glue fault", d.fault ? 1u : 0u, 0u);
        if (!cs_line(d, p.cs)) {
            std::printf("[FAIL] %s: glue peripheral cs mismatch\n", p.name);
            g_failures++;
        } else if (g_failures == probe_fail_before) {
            std::printf("[PASS] %s\n", p.name);
        }
    }

    bus.write8(RomBootBus::VIA2_BASE + 0x0200u, 0x5au);
    check_eq(scope, "via2 reg write/readback", bus.read8(RomBootBus::VIA2_BASE + 0x0200u),
             0x5au);

    bus.write8(RomBootBus::TURBOSCSI_BASE + 0x10u, 0xaau);
    check_eq(scope, "scsi reg write/readback", bus.read8(RomBootBus::TURBOSCSI_BASE + 0x10u),
             0xaau);

    bus.write8(RomBootBus::ASC_BASE + 0x801u, 0x12u);
    check_eq(scope, "asc mode write/readback", bus.read8(RomBootBus::ASC_BASE + 0x801u),
             0x12u);

    if (g_failures == fail_start)
        std::printf("[PASS] %s\n", scope);
    return g_failures == fail_start;
}

} // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vglue dut;
    dut.clk = 0;
    dut.rst = 0;
    dut.req_valid = 0;
    dut.eval();

    run_decode_suite(dut);
    run_overlay_suite(dut);
    run_peripheral_baseline_suite(dut);

    if (g_failures == 0) {
        std::printf("tb_model_rtl_eq: PASS\n");
        return 0;
    }
    std::printf("tb_model_rtl_eq: FAIL failures=%d\n", g_failures);
    return 1;
}
