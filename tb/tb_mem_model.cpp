// tb_mem_model.cpp -- host checks for the shared ROM harness memory model.

#include <cstdint>
#include <cstdio>
#include "models/mem_model.h"

static int failures = 0;

#define CHECK_EQ(name, got, exp) do { \
    uint32_t g_ = static_cast<uint32_t>(got); \
    uint32_t e_ = static_cast<uint32_t>(exp); \
    if (g_ != e_) { \
        std::printf("[FAIL] %s: got 0x%08x expected 0x%08x\n", name, g_, e_); \
        failures++; \
    } else { \
        std::printf("[PASS] %s\n", name); \
    } \
} while (0)

int main() {
    MemModel mem;

    CHECK_EQ("default RAM window size", mem.ram_window_size(),
             MemModel::RAM_WINDOW_DEFAULT);
    CHECK_EQ("legacy ROM alias disabled by default",
             mem.legacy_rom_alias_enabled(), 0u);
    mem.write32(0x00000020u, 0x55667788u);
    CHECK_EQ("RAM out-of-window read is open-bus at +64MiB",
             mem.read32(0x04000020u), 0xffffffffu);
    CHECK_EQ("RAM out-of-window read is open-bus at +128MiB",
             mem.read32(0x08000020u), 0xffffffffu);

    CHECK_EQ("set 4MiB RAM window", mem.set_ram_window_size(0x00400000u), 1u);
    mem.write32(0x00000024u, 0x11223344u);
    CHECK_EQ("RAM out-of-window read is open-bus at +4MiB",
             mem.read32(0x00400024u), 0xffffffffu);
    CHECK_EQ("RAM out-of-window read is open-bus at +12MiB",
             mem.read32(0x00C00024u), 0xffffffffu);
    CHECK_EQ("reject invalid non-power2 RAM window",
             mem.set_ram_window_size(0x00600000u), 0u);

    mem.write32(MemModel::ROM_BASE + 0x10u, 0x12345678u);
    CHECK_EQ("ROM base mapping moved to 0x4000_0000",
             mem.read32(MemModel::ROM_BASE + 0x10u), 0x12345678u);
    CHECK_EQ("ROM legacy alias open-bus when disabled",
             mem.read32(MemModel::ROM_LEGACY_BASE + 0x10u), 0xffffffffu);
    mem.enable_legacy_rom_alias(true);
    CHECK_EQ("ROM legacy alias reads same backing when enabled",
             mem.read32(MemModel::ROM_LEGACY_BASE + 0x10u), 0x12345678u);

    mem.write32(MemModel::VRAM_BASE, 0x11223344u);
    CHECK_EQ("vram base persists", mem.read32(MemModel::VRAM_BASE), 0x11223344u);

    const uint32_t tail = MemModel::VRAM_BASE + MemModel::VRAM_SIZE - 4u;
    mem.write32(tail, 0xA5C35A3Cu);
    CHECK_EQ("vram top word persists", mem.read32(tail), 0xA5C35A3Cu);

    CHECK_EQ("vram below unmapped", mem.mapped(MemModel::VRAM_BASE - 4u), 0u);
    CHECK_EQ("vram past end unmapped", mem.mapped(MemModel::VRAM_BASE + MemModel::VRAM_SIZE), 0u);

    mem.write32(MemModel::MAGIC_BASE, 0xC0FFEE00u);
    CHECK_EQ("magic region unchanged", mem.read32(MemModel::MAGIC_BASE), 0xC0FFEE00u);

    if (failures == 0) {
        std::printf("All mem_model checks PASSED.\n");
        return 0;
    }
    std::printf("%d mem_model checks FAILED.\n", failures);
    return 1;
}
