// tb_decode_fpu.cpp - standalone decode.v tests for first FPU decode slice.
//
// Build: make tb-decode-fpu

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <verilated.h>
#include "Vdecode.h"

namespace {

static constexpr uint8_t UOP_FP  = 1;
static constexpr uint8_t UOP_SYS = 5;
static constexpr uint8_t SZ_LONG = 2;

static constexpr uint8_t FPU_FADD = 0;
static constexpr uint8_t FPU_FSUB = 1;
static constexpr uint8_t FPU_FMUL = 2;
static constexpr uint8_t FPU_FDIV = 3;
static constexpr uint8_t FPU_FABS = 5;
static constexpr uint8_t FPU_FNEG = 6;
static constexpr uint8_t FPU_FMOV = 7;

int n_pass = 0;
int n_fail = 0;

void check_eq(const char* scope, const char* field, uint64_t got, uint64_t exp) {
    if (got != exp) {
        std::fprintf(stderr, "[%s] %s: got 0x%llx expected 0x%llx\n",
                     scope, field,
                     static_cast<unsigned long long>(got),
                     static_cast<unsigned long long>(exp));
        n_fail++;
    } else {
        n_pass++;
    }
}

void tick(Vdecode* dut) {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
}

void reset(Vdecode* dut) {
    dut->rst = 1;
    dut->pd_valid = 0;
    dut->pd_fault = 0;
    dut->pd_next_fault = 0;
    dut->pd_inst_len_hint = 0;
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

void set_pd_buf_words(Vdecode* dut, const std::vector<uint16_t>& opwords) {
    uint8_t bytes[16] = {0};
    for (size_t i = 0; i < opwords.size() && i < 8; i++) {
        bytes[2 * i + 0] = static_cast<uint8_t>(opwords[i] >> 8);
        bytes[2 * i + 1] = static_cast<uint8_t>(opwords[i] & 0xff);
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

uint16_t fpu_rr_ext(uint8_t src, uint8_t dst, uint8_t opmode) {
    return static_cast<uint16_t>(((src & 7) << 10) |
                                 ((dst & 7) << 7) |
                                 (opmode & 0x7f));
}

void drive(Vdecode* dut, uint32_t pc, uint16_t opword, uint16_t extword,
           bool pd_fault = false, bool pd_next_fault = false) {
    dut->flush_en = 1;
    tick(dut);
    dut->flush_en = 0;

    dut->pd_valid = 1;
    dut->pd_fault = pd_fault ? 1 : 0;
    dut->pd_next_fault = pd_next_fault ? 1 : 0;
    dut->pd_inst_len_hint = 0;
    dut->rn_ready = 1;
    dut->pd_pc = pc;
    set_pd_buf_words(dut, {opword, extword, 0x4e71, 0x4e71, 0x4e71, 0x4e71});

    // decode.v has a 1-cycle F3 capture stage between pd_buf and outputs.
    tick(dut);
    dut->eval();
}

void expect_fp_rr(Vdecode* dut, const char* scope, uint8_t opmode,
                  uint8_t src, uint8_t dst, uint8_t fpu_op,
                  uint8_t src_a, bool has_src_b, uint8_t src_b) {
    drive(dut, 0x40800000u, 0xf200, fpu_rr_ext(src, dst, opmode));

    check_eq(scope, "uop_valid", dut->uop_valid, 1);
    check_eq(scope, "uop_type", dut->uop_type, UOP_FP);
    check_eq(scope, "uop_op", dut->uop_op, fpu_op);
    check_eq(scope, "uop_size", dut->uop_size, SZ_LONG);
    check_eq(scope, "has_src_a", dut->has_src_a, 1);
    check_eq(scope, "arch_src_a", dut->arch_src_a, src_a);
    check_eq(scope, "has_src_b", dut->has_src_b, has_src_b ? 1 : 0);
    check_eq(scope, "arch_src_b", dut->arch_src_b, src_b);
    check_eq(scope, "has_dst", dut->has_dst, 1);
    check_eq(scope, "arch_dst", dut->arch_dst, dst);
    check_eq(scope, "flags_wr", dut->flags_wr, 0);
    check_eq(scope, "flags_rd", dut->flags_rd, 0);
    check_eq(scope, "exc_valid", dut->exc_valid, 0);
    check_eq(scope, "len_bytes", dut->uop_npc - dut->uop_pc, 4);
}

void expect_fline_trap(Vdecode* dut, const char* scope,
                       uint16_t opword, uint16_t extword) {
    drive(dut, 0x40800000u, opword, extword);

    check_eq(scope, "uop_valid", dut->uop_valid, 1);
    check_eq(scope, "uop_type", dut->uop_type, UOP_SYS);
    check_eq(scope, "exc_valid", dut->exc_valid, 1);
    check_eq(scope, "exc_vec", dut->exc_vec, 11);
    check_eq(scope, "has_src_a", dut->has_src_a, 0);
    check_eq(scope, "has_src_b", dut->has_src_b, 0);
    check_eq(scope, "has_dst", dut->has_dst, 0);
    check_eq(scope, "len_bytes", dut->uop_npc - dut->uop_pc, 2);
}

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    Vdecode* dut = new Vdecode;
    reset(dut);

    expect_fp_rr(dut, "fmove_fp2_fp5", 0x00, 2, 5, FPU_FMOV, 2, false, 2);
    expect_fp_rr(dut, "fabs_fp2_fp5",  0x18, 2, 5, FPU_FABS, 2, false, 2);
    expect_fp_rr(dut, "fneg_fp4_fp6",  0x1a, 4, 6, FPU_FNEG, 4, false, 4);
    expect_fp_rr(dut, "fdiv_fp2_fp5",  0x20, 2, 5, FPU_FDIV, 5, true, 2);
    expect_fp_rr(dut, "fadd_fp1_fp3",  0x22, 1, 3, FPU_FADD, 3, true, 1);
    expect_fp_rr(dut, "fmul_fp7_fp0",  0x23, 7, 0, FPU_FMUL, 0, true, 7);
    expect_fp_rr(dut, "fsub_fp4_fp6",  0x28, 4, 6, FPU_FSUB, 6, true, 4);

    // Unsupported arithmetic and EA-source forms must not enter iq_fp.
    expect_fline_trap(dut, "unsupported_fsqrt", 0xf200, fpu_rr_ext(1, 2, 0x04));
    expect_fline_trap(dut, "ea_source_fadd", 0xf200,
                      static_cast<uint16_t>(0x4000 | fpu_rr_ext(1, 2, 0x22)));
    expect_fline_trap(dut, "f300_other_fpu_class", 0xf300,
                      fpu_rr_ext(1, 2, 0x22));

    // A supported FP op consumes its mandatory extension word.  At PC+14,
    // len=4 crosses the 16-byte fetch line; pd_next_fault must therefore
    // become an instruction bus fault.  If decode accidentally reports
    // len=2 here, the fault override will not fire.
    drive(dut, 0x4080000eu, 0xf200, fpu_rr_ext(1, 3, 0x22), false, true);
    check_eq("fpu_ext_next_line_fault", "uop_type", dut->uop_type, UOP_SYS);
    check_eq("fpu_ext_next_line_fault", "exc_valid", dut->exc_valid, 1);
    check_eq("fpu_ext_next_line_fault", "exc_vec", dut->exc_vec, 2);
    check_eq("fpu_ext_next_line_fault", "exc_fault_addr", dut->exc_fault_addr,
             0x40800010u);

    std::printf("tb_decode_fpu: %d passed, %d failed\n", n_pass, n_fail);
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
