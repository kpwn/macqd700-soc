// tb_fp_rat.cpp - standalone Verilator unit test for rtl/core/rename/fp_rat.v

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <verilated.h>
#include "Vfp_rat.h"

static Vfp_rat* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

static constexpr uint8_t ARCH_FP_REGS = 8;
static constexpr uint8_t PHYS_FP_REGS = 24;
static constexpr uint8_t INITIAL_FREE_COUNT = PHYS_FP_REGS - ARCH_FP_REGS;
static constexpr uint8_t PHYS_MASK = 0x1f;

static void tick() {
    dut->clk = 0;
    dut->eval();
    sim_time++;
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

static void zero_inputs() {
    dut->src_a = 0;
    dut->src_b = 0;
    dut->alloc_arch_dst = 0;
    dut->alloc_en = 0;
    dut->commit_en = 0;
    dut->commit_arch = 0;
    dut->commit_phys = 0;
    dut->commit_old_phys = 0;
    dut->flush_en = 0;
    dut->cdb_fp_phys = 0;
    dut->cdb_fp_en = 0;
    dut->cdb_fp_has_dst = 0;
    dut->dbg_load_en = 0;
}

static void reset() {
    dut->rst = 1;
    zero_inputs();
    tick();
    tick();
    dut->rst = 0;
    dut->eval();
}

static uint8_t rd_phys(uint8_t arch) {
    dut->src_a = arch & 0x7;
    dut->eval();
    return dut->phys_src_a & PHYS_MASK;
}

static bool rd_ready(uint8_t arch) {
    dut->src_a = arch & 0x7;
    dut->eval();
    return dut->src_a_rdy != 0;
}

static uint8_t free_cnt() {
    dut->eval();
    return dut->dbg_free_cnt & 0x3f;
}

static uint8_t alloc_dst(uint8_t arch_dst, uint8_t* old_out = nullptr) {
    dut->alloc_arch_dst = arch_dst & 0x7;
    dut->alloc_en = 1;
    dut->eval();
    uint8_t phys = dut->alloc_phys_dst & PHYS_MASK;
    if (old_out) *old_out = dut->alloc_phys_old & PHYS_MASK;
    tick();
    dut->alloc_en = 0;
    dut->alloc_arch_dst = 0;
    dut->eval();
    return phys;
}

static void cdb_pulse(uint8_t phys, bool has_dst = true) {
    dut->cdb_fp_phys = phys & PHYS_MASK;
    dut->cdb_fp_en = 1;
    dut->cdb_fp_has_dst = has_dst ? 1 : 0;
    tick();
    dut->cdb_fp_phys = 0;
    dut->cdb_fp_en = 0;
    dut->cdb_fp_has_dst = 0;
    dut->eval();
}

static void commit_pulse(uint8_t arch, uint8_t phys, uint8_t old_phys) {
    dut->commit_arch = arch & 0x7;
    dut->commit_phys = phys & PHYS_MASK;
    dut->commit_old_phys = old_phys & PHYS_MASK;
    dut->commit_en = 1;
    tick();
    dut->commit_en = 0;
    dut->commit_arch = 0;
    dut->commit_phys = 0;
    dut->commit_old_phys = 0;
    dut->eval();
}

#define CHECK_EQ(name, got, exp) do { \
    uint32_t g = static_cast<uint32_t>(got); \
    uint32_t e = static_cast<uint32_t>(exp); \
    if (g != e) { \
        std::printf("  FAIL %s: got %u (0x%x), expected %u (0x%x)\n", \
                    name, g, g, e, e); \
        return false; \
    } \
} while (0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        std::printf("  FAIL %s: condition false\n", name); \
        return false; \
    } \
} while (0)

static bool test_reset_initial_map() {
    reset();
    for (uint8_t i = 0; i < ARCH_FP_REGS; i++) {
        char lbl[32];
        std::snprintf(lbl, sizeof(lbl), "ratmap[%u]", i);
        CHECK_EQ(lbl, rd_phys(i), i);
        CHECK_TRUE("reset ready", rd_ready(i));
    }
    CHECK_TRUE("alloc_ok reset", dut->alloc_ok != 0);
    CHECK_EQ("first alloc phys", dut->alloc_phys_dst & PHYS_MASK, 8);
    CHECK_EQ("free count reset", free_cnt(), INITIAL_FREE_COUNT);
    return true;
}

static bool test_speculative_alloc_and_ready() {
    reset();
    uint8_t old = 0xff;
    uint8_t p0 = alloc_dst(0, &old);
    CHECK_EQ("alloc FP0 phys", p0, 8);
    CHECK_EQ("alloc FP0 old", old, 0);
    CHECK_EQ("FP0 map after alloc", rd_phys(0), 8);
    CHECK_TRUE("FP0 not ready after alloc", !rd_ready(0));
    CHECK_EQ("free after alloc", free_cnt(), INITIAL_FREE_COUNT - 1);

    cdb_pulse(p0, false);
    CHECK_TRUE("has_dst gate blocks wake", !rd_ready(0));
    cdb_pulse(p0, true);
    CHECK_TRUE("CDB wakes allocated phys", rd_ready(0));
    return true;
}

static bool test_commit_free_and_flush_preserve() {
    reset();
    uint8_t old = 0;
    uint8_t p0 = alloc_dst(0, &old);
    uint8_t p1 = alloc_dst(1);
    CHECK_EQ("FP0 speculative phys", p0, 8);
    CHECK_EQ("FP1 speculative phys", p1, 9);
    CHECK_EQ("free after two allocs", free_cnt(), INITIAL_FREE_COUNT - 2);

    commit_pulse(0, p0, old);
    CHECK_EQ("commit frees old phys", free_cnt(), INITIAL_FREE_COUNT - 1);

    dut->flush_en = 1;
    tick();
    dut->flush_en = 0;
    dut->eval();

    CHECK_EQ("flush preserves committed FP0", rd_phys(0), p0);
    CHECK_EQ("flush rolls back FP1", rd_phys(1), 1);
    CHECK_EQ("free after flush", free_cnt(), INITIAL_FREE_COUNT);
    return true;
}

static bool test_same_cycle_commit_flush() {
    reset();
    uint8_t old = 0;
    uint8_t p0 = alloc_dst(0, &old);

    dut->commit_en = 1;
    dut->commit_arch = 0;
    dut->commit_phys = p0;
    dut->commit_old_phys = old;
    dut->flush_en = 1;
    tick();
    dut->commit_en = 0;
    dut->flush_en = 0;
    dut->commit_arch = 0;
    dut->commit_phys = 0;
    dut->commit_old_phys = 0;
    dut->eval();

    CHECK_EQ("same-cycle commit+flush FP0", rd_phys(0), p0);
    CHECK_EQ("same-cycle free count", free_cnt(), INITIAL_FREE_COUNT);
    CHECK_TRUE("committed phys remains ready state unchanged", !rd_ready(0));
    cdb_pulse(p0);
    CHECK_TRUE("committed phys can wake", rd_ready(0));
    return true;
}

static bool test_exhaustion_and_reuse() {
    reset();
    for (uint8_t i = 0; i < INITIAL_FREE_COUNT; i++) {
        CHECK_TRUE("alloc_ok before exhaustion", dut->alloc_ok != 0);
        uint8_t got = alloc_dst(i & 0x7);
        CHECK_EQ("sequential free-list pop", got, static_cast<uint8_t>(8 + i));
    }

    dut->eval();
    CHECK_TRUE("alloc_ok exhausted low", dut->alloc_ok == 0);
    CHECK_EQ("free count exhausted", free_cnt(), 0);

    commit_pulse(0, 23, 8);
    CHECK_EQ("free after commit old", free_cnt(), 1);
    CHECK_TRUE("alloc_ok after commit free", dut->alloc_ok != 0);
    CHECK_EQ("reused freed phys", dut->alloc_phys_dst & PHYS_MASK, 8);
    return true;
}

static bool test_dbg_load_reset() {
    reset();
    alloc_dst(0);
    alloc_dst(1);
    CHECK_EQ("free before dbg load", free_cnt(), INITIAL_FREE_COUNT - 2);

    dut->dbg_load_en = 1;
    tick();
    dut->dbg_load_en = 0;
    dut->eval();

    for (uint8_t i = 0; i < ARCH_FP_REGS; i++)
        CHECK_EQ("dbg-load identity map", rd_phys(i), i);
    CHECK_EQ("dbg-load free count", free_cnt(), INITIAL_FREE_COUNT);
    CHECK_EQ("dbg-load first free", dut->alloc_phys_dst & PHYS_MASK, 8);
    return true;
}

static void run_test(const char* name, bool (*fn)()) {
    std::printf("[ RUN      ] %s\n", name);
    bool ok = fn();
    if (ok) {
        n_pass++;
        std::printf("[       OK ] %s\n", name);
    } else {
        n_fail++;
        std::printf("[  FAILED  ] %s\n", name);
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vfp_rat;
    reset();

    run_test("reset_initial_map", test_reset_initial_map);
    run_test("speculative_alloc_and_ready", test_speculative_alloc_and_ready);
    run_test("commit_free_and_flush_preserve", test_commit_free_and_flush_preserve);
    run_test("same_cycle_commit_flush", test_same_cycle_commit_flush);
    run_test("exhaustion_and_reuse", test_exhaustion_and_reuse);
    run_test("dbg_load_reset", test_dbg_load_reset);

    std::printf("fp_rat tests: %d passed, %d failed\n", n_pass, n_fail);
    delete dut;
    return n_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
