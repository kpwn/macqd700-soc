#include "Vl2c_pri8.h"
#include "verilated.h"
#include <cstdio>

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vl2c_pri8 dut;
    unsigned ordering_checks = 0;
    for (unsigned mask = 0; mask < 256; ++mask) {
        unsigned expected_index = 7;
        for (unsigned i = 0; i < 8; ++i) {
            if (mask & (1u << i)) { expected_index = i; break; }
        }
        const unsigned expected_onehot = mask ? (1u << expected_index) : 0;
        dut.req = mask;
        dut.eval();
        if (dut.hit != (mask != 0) || dut.idx != expected_index ||
            dut.onehot != expected_onehot) {
            std::fprintf(stderr, "FAIL mask=%02x hit=%u idx=%u onehot=%02x expected=%02x\n",
                         mask, unsigned(dut.hit), unsigned(dut.idx),
                         unsigned(dut.onehot), expected_onehot);
            return 1;
        }
        for (unsigned ids = 0; ids < 256; ++ids) {
            const unsigned old_mask = dut.hit ? (1u << dut.idx) : 0;
            const bool before = (ids & ~old_mask) != 0;
            const bool after = (ids & ~unsigned(dut.onehot)) != 0;
            if (before != after) {
                std::fprintf(stderr, "FAIL ordering mask=%02x ids=%02x\n", mask, ids);
                return 1;
            }
            ++ordering_checks;
        }
    }
    dut.final();
    std::printf("PASS l2c_pri8: 256 masks, %u ordering-vector combinations\n", ordering_checks);
    return 0;
}
