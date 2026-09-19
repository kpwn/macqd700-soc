// tb_sd_scsi_lba_mapper.cpp — unit tests for raw SCSI-to-SD LBA mapping.

#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Vsd_scsi_lba_mapper.h"

static Vsd_scsi_lba_mapper* dut = nullptr;
static int n_pass = 0;
static int n_fail = 0;

static constexpr uint32_t SD_RAW_BASE_LBA = 8192u;
static constexpr uint32_t ROM_WINDOW_LAST_LBA = SD_RAW_BASE_LBA - 1u;
static constexpr uint32_t SCSI_NUM_LBAS = 1048576u;

#define CHECK_EQ(name, got, exp) do { \
    uint32_t _g = (uint32_t)(got); \
    uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        std::printf("  FAIL %s: got 0x%08x, expected 0x%08x\n", \
                    name, _g, _e); \
        return false; \
    } \
} while (0)

static void drive(uint32_t lba, uint32_t blocks) {
    // Disk size is now a runtime input (derived from the SD card's CSD)
    // rather than a compile-time parameter -- drive it explicitly.
    dut->scsi_num_lbas = SCSI_NUM_LBAS;
    dut->scsi_lba = lba;
    dut->scsi_blocks = blocks & 0x00ffffffu;
    dut->eval();
}

// Same, with an explicit disk size: proves the bound actually tracks the
// input instead of a baked-in constant.  This is the regression that would
// have caught the 2026-07-28 capacity overrun, where a 700 MB image was
// provisioned onto a target hardcoded to report 512 MB and 188 MB of the
// HFS volume became unreachable.
static void drive_sized(uint32_t num_lbas, uint32_t lba, uint32_t blocks) {
    dut->scsi_num_lbas = num_lbas;
    dut->scsi_lba = lba;
    dut->scsi_blocks = blocks & 0x00ffffffu;
    dut->eval();
}

static bool test_frontier_maps_after_reserved_window() {
    drive(0, 1);
    CHECK_EQ("LBA0 valid", dut->valid, 1);
    CHECK_EQ("LBA0 maps after 4MiB", dut->sd_lba, SD_RAW_BASE_LBA);

    drive(0x1234u, 7);
    CHECK_EQ("middle valid", dut->valid, 1);
    CHECK_EQ("middle maps with bias", dut->sd_lba, SD_RAW_BASE_LBA + 0x1234u);
    return true;
}

static bool test_raw_base_is_first_sector_after_rom_window() {
    drive(0, 1);
    CHECK_EQ("ROM window last LBA", ROM_WINDOW_LAST_LBA, 8191u);
    CHECK_EQ("raw base equals ROM last + 1",
             dut->sd_lba, ROM_WINDOW_LAST_LBA + 1u);
    CHECK_EQ("raw base does not overlap ROM window",
             dut->sd_lba > ROM_WINDOW_LAST_LBA, 1u);
    return true;
}

static bool test_last_exposed_lba_is_valid() {
    drive(SCSI_NUM_LBAS - 1, 1);
    CHECK_EQ("last valid", dut->valid, 1);
    CHECK_EQ("last sd lba", dut->sd_lba,
             SD_RAW_BASE_LBA + SCSI_NUM_LBAS - 1);
    return true;
}

static bool test_capacity_overflow_fails_closed() {
    drive(SCSI_NUM_LBAS - 1, 2);
    CHECK_EQ("two block overrun invalid", dut->valid, 0);
    CHECK_EQ("overrun still maps deterministically", dut->sd_lba,
             SD_RAW_BASE_LBA + SCSI_NUM_LBAS - 1);

    drive(SCSI_NUM_LBAS, 1);
    CHECK_EQ("first hidden lba invalid", dut->valid, 0);
    return true;
}

static bool test_zero_block_request_fails_closed() {
    drive(0, 0);
    CHECK_EQ("zero block invalid", dut->valid, 0);
    CHECK_EQ("zero block map deterministic", dut->sd_lba, SD_RAW_BASE_LBA);
    return true;
}

// The capacity bound must follow the runtime input, not a baked-in constant.
// Regression for 2026-07-28: a 700 MB image was provisioned onto a target
// whose reported capacity was hardcoded to 512 MB, so 188 MB of the HFS
// volume -- including the Alternate MDB in the volume's second-to-last
// block -- was unreachable and those reads never completed.
static bool test_bound_tracks_runtime_capacity() {
    // A small card: the last sector inside it is valid...
    const uint32_t small = 100000u;
    drive_sized(small, small - 1u, 1);
    CHECK_EQ("small: last lba valid", dut->valid, 1);
    // ...and one past the end fails closed.
    drive_sized(small, small, 1);
    CHECK_EQ("small: one past end invalid", dut->valid, 0);

    // A larger card makes the SAME lba legal -- proving the bound moved.
    const uint32_t big = 2000000u;
    drive_sized(big, small, 1);
    CHECK_EQ("big: same lba now valid", dut->valid, 1);
    CHECK_EQ("big: maps past reserved window", dut->sd_lba,
             SD_RAW_BASE_LBA + small);

    // And the big card's own frontier still fails closed.
    drive_sized(big, big, 1);
    CHECK_EQ("big: one past end invalid", dut->valid, 0);
    return true;
}

#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { std::printf("[PASS] " #fn "\n"); n_pass++; } \
    else { std::printf("[FAIL] " #fn "\n"); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vsd_scsi_lba_mapper;

    RUN(test_frontier_maps_after_reserved_window);
    RUN(test_raw_base_is_first_sector_after_rom_window);
    RUN(test_last_exposed_lba_is_valid);
    RUN(test_capacity_overflow_fails_closed);
    RUN(test_zero_block_request_fails_closed);
    RUN(test_bound_tracks_runtime_capacity);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
