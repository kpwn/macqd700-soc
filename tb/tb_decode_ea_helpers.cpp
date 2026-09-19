#include <cstdint>
#include <cstdio>

#include <verilated.h>
#include "Vtb_decode_ea_helpers.h"

static int errors = 0;

static void check(bool ok, const char* name) {
    if (!ok) {
        std::fprintf(stderr, "FAIL: %s\n", name);
        errors++;
    }
}

static void drive(Vtb_decode_ea_helpers& dut, uint8_t mode, uint8_t reg,
                  uint8_t size, uint16_t ext) {
    dut.ea_mode = mode;
    dut.ea_reg = reg;
    dut.ea_size = size;
    dut.ea_ext = ext;
    dut.eval();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_decode_ea_helpers dut;

    drive(dut, 0, 0, 0, 0);
    check(dut.stride == 1, "byte stride D0");
    drive(dut, 0, 7, 0, 0);
    check(dut.stride == 2, "byte stride A7");
    drive(dut, 0, 7, 1, 0);
    check(dut.stride == 2, "word stride");
    drive(dut, 0, 7, 2, 0);
    check(dut.stride == 4, "long stride");
    drive(dut, 0, 7, 3, 0);
    check(dut.stride == 0, "invalid stride");

    drive(dut, 0, 0, 2, 0);
    check(dut.ext_words == 0, "Dn ext words");
    drive(dut, 5, 0, 2, 0);
    check(dut.ext_words == 1, "d16 An ext words");
    drive(dut, 7, 1, 2, 0);
    check(dut.ext_words == 2, "abs long ext words");
    drive(dut, 7, 3, 2, 0);
    check(dut.ext_words == 1, "PC indexed brief ext words");
    drive(dut, 6, 0, 2, 0x0123);
    check(dut.ext_words == 4, "full ext bd word od long words");
    drive(dut, 7, 4, 0, 0);
    check(dut.ext_words == 1, "immediate byte ext words");
    drive(dut, 7, 4, 2, 0);
    check(dut.ext_words == 2, "immediate long ext words");

    drive(dut, 0, 0, 2, 0);
    check(dut.data_alterable, "Dn data alterable");
    drive(dut, 1, 0, 2, 0);
    check(!dut.data_alterable, "An not data alterable");
    drive(dut, 3, 0, 2, 0);
    check(dut.data_alterable, "postinc data alterable");
    drive(dut, 7, 0, 2, 0);
    check(dut.data_alterable, "abs word data alterable");
    drive(dut, 7, 2, 2, 0);
    check(!dut.data_alterable, "PC rel not data alterable");
    drive(dut, 7, 4, 2, 0);
    check(!dut.data_alterable, "imm not data alterable");

    drive(dut, 0, 0, 2, 0);
    check(!dut.memory_alterable, "Dn not memory alterable");
    drive(dut, 2, 0, 2, 0);
    check(dut.memory_alterable, "indirect memory alterable");
    drive(dut, 4, 0, 2, 0);
    check(dut.memory_alterable, "predec memory alterable");
    drive(dut, 7, 3, 2, 0);
    check(!dut.memory_alterable, "PC indexed not memory alterable");

    drive(dut, 2, 0, 2, 0);
    check(dut.control, "indirect control");
    drive(dut, 3, 0, 2, 0);
    check(!dut.control, "postinc not control");
    drive(dut, 4, 0, 2, 0);
    check(!dut.control, "predec not control");
    drive(dut, 7, 2, 2, 0);
    check(dut.control, "PC disp control");
    drive(dut, 7, 4, 2, 0);
    check(!dut.control, "imm not control");

    drive(dut, 2, 0, 2, 0);
    check(dut.control_alterable, "indirect control alterable");
    drive(dut, 6, 0, 2, 0);
    check(dut.control_alterable, "indexed control alterable");
    drive(dut, 7, 2, 2, 0);
    check(!dut.control_alterable, "PC disp not control alterable");

    drive(dut, 2, 3, 2, 0);
    check(dut.an_ind_or_d16, "An indirect simple RMW EA");
    drive(dut, 5, 4, 2, 0);
    check(dut.an_ind_or_d16, "d16 An simple RMW EA");
    drive(dut, 3, 0, 2, 0);
    check(!dut.an_ind_or_d16, "postinc not simple RMW EA");
    drive(dut, 7, 0, 2, 0);
    check(!dut.an_ind_or_d16, "abs word not simple RMW EA");

    drive(dut, 7, 3, 2, 0);
    check(!dut.immediate, "PC indexed not immediate");
    drive(dut, 7, 4, 2, 0);
    check(dut.immediate, "immediate");
    drive(dut, 7, 1, 2, 0);
    check(!dut.pc_relative, "abs long not PC relative");
    drive(dut, 7, 2, 2, 0);
    check(dut.pc_relative, "PC disp relative");
    drive(dut, 7, 3, 2, 0);
    check(dut.pc_relative, "PC indexed relative");

    if (errors != 0) {
        std::fprintf(stderr, "decode EA helper checks failed: %d\n", errors);
        return 1;
    }

    std::puts("decode EA helper checks passed");
    return 0;
}
