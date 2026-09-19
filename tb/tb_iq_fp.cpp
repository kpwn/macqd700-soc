// tb_iq_fp.cpp - standalone Verilator unit test for rtl/core/issue/iq_fp.v

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <verilated.h>
#include "Viq_fp.h"

static Viq_fp* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

static constexpr uint8_t FPU_FADD = 0;
static constexpr uint8_t FPU_FMUL = 2;

static void tick() {
    dut->clk = 0;
    dut->eval();
    sim_time++;
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

static void zero_inputs() {
    dut->flush_en = 0;
    dut->disp_en = 0;
    dut->disp_uop_op = 0;
    dut->disp_phys_src_a = 0;
    dut->disp_phys_src_b = 0;
    dut->disp_src_a_rdy = 0;
    dut->disp_src_b_rdy = 0;
    dut->disp_phys_dst = 0;
    dut->disp_has_dst = 0;
    dut->disp_rob_tag = 0;
    dut->cdb_fp_en = 0;
    dut->cdb_fp_phys = 0;
    dut->cdb_fp_has_dst = 0;
    dut->iss_ready = 1;
}

static void reset() {
    dut->rst = 1;
    zero_inputs();
    tick();
    tick();
    dut->rst = 0;
    dut->eval();
}

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        std::printf("  FAIL %s: condition false\n", name); \
        return false; \
    } \
} while (0)

#define CHECK_EQ(name, got, exp) do { \
    uint32_t g = static_cast<uint32_t>(got); \
    uint32_t e = static_cast<uint32_t>(exp); \
    if (g != e) { \
        std::printf("  FAIL %s: got %u (0x%x), expected %u (0x%x)\n", \
                    name, g, g, e, e); \
        return false; \
    } \
} while (0)

static void dispatch_ready(uint8_t op, uint8_t psa, uint8_t psb,
                           uint8_t pdst, uint8_t rob_tag) {
    dut->disp_en = 1;
    dut->disp_uop_op = op;
    dut->disp_phys_src_a = psa;
    dut->disp_phys_src_b = psb;
    dut->disp_src_a_rdy = 1;
    dut->disp_src_b_rdy = 1;
    dut->disp_phys_dst = pdst;
    dut->disp_has_dst = 1;
    dut->disp_rob_tag = rob_tag;
    tick();
    dut->disp_en = 0;
    dut->eval();
}

static bool test_issue_holds_until_ready() {
    reset();

    dut->iss_ready = 0;
    dispatch_ready(FPU_FADD, 1, 2, 8, 0x11);
    CHECK_TRUE("issue suppressed by ready", dut->iss_valid == 0);

    dut->iss_ready = 1;
    dut->eval();
    CHECK_TRUE("held entry becomes valid", dut->iss_valid != 0);
    CHECK_EQ("held op", dut->iss_uop_op, FPU_FADD);
    CHECK_EQ("held src a", dut->iss_phys_src_a, 1);
    CHECK_EQ("held src b", dut->iss_phys_src_b, 2);
    CHECK_EQ("held dst", dut->iss_phys_dst, 8);
    CHECK_EQ("held rob tag", dut->iss_rob_tag, 0x11);

    tick();
    dut->eval();
    CHECK_TRUE("entry clears after ready issue", dut->iss_valid == 0);
    return true;
}

static bool test_cdb_has_dst_gate_and_wake() {
    reset();

    dut->disp_en = 1;
    dut->disp_uop_op = FPU_FMUL;
    dut->disp_phys_src_a = 5;
    dut->disp_phys_src_b = 6;
    dut->disp_src_a_rdy = 0;
    dut->disp_src_b_rdy = 1;
    dut->disp_phys_dst = 9;
    dut->disp_has_dst = 1;
    dut->disp_rob_tag = 0x12;
    tick();
    dut->disp_en = 0;
    dut->eval();
    CHECK_TRUE("waiting on src a", dut->iss_valid == 0);

    dut->cdb_fp_en = 1;
    dut->cdb_fp_phys = 5;
    dut->cdb_fp_has_dst = 0;
    tick();
    dut->cdb_fp_en = 0;
    dut->eval();
    CHECK_TRUE("has_dst gate blocks wake", dut->iss_valid == 0);

    dut->cdb_fp_en = 1;
    dut->cdb_fp_phys = 5;
    dut->cdb_fp_has_dst = 1;
    tick();
    dut->cdb_fp_en = 0;
    dut->eval();
    CHECK_TRUE("cdb wakes source", dut->iss_valid != 0);
    CHECK_EQ("woken op", dut->iss_uop_op, FPU_FMUL);
    CHECK_EQ("woken dst", dut->iss_phys_dst, 9);
    CHECK_EQ("woken rob tag", dut->iss_rob_tag, 0x12);
    return true;
}

static bool test_flush_clears_waiting_entry() {
    reset();

    dut->iss_ready = 0;
    dispatch_ready(FPU_FADD, 1, 2, 8, 0x13);
    CHECK_TRUE("entry held before flush", dut->disp_ready != 0);

    dut->flush_en = 1;
    tick();
    dut->flush_en = 0;
    dut->iss_ready = 1;
    dut->eval();
    CHECK_TRUE("flush cleared entry", dut->iss_valid == 0);
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
    dut = new Viq_fp;
    reset();

    run_test("issue_holds_until_ready", test_issue_holds_until_ready);
    run_test("cdb_has_dst_gate_and_wake", test_cdb_has_dst_gate_and_wake);
    run_test("flush_clears_waiting_entry", test_flush_clears_waiting_entry);

    std::printf("iq_fp tests: %d passed, %d failed\n", n_pass, n_fail);
    delete dut;
    return n_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
