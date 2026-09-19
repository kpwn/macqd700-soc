// tb_pic16c5x.cpp - Verilator unit testbench for pic16c5x.v
//
// Build: make tb-pic16c5x

#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Vpic16c5x.h"

static Vpic16c5x* dut = nullptr;
static uint64_t sim_time = 0;
static bool cycle_trace = false;

static void tick() {
    dut->clk = 0;
    dut->eval();
    sim_time++;
    dut->clk = 1;
    dut->eval();
    sim_time++;
    if (cycle_trace) {
        std::printf("EDGE %llu rst=%d en=%d done=%d pc=%03x w=%02x pa=%02x pb=%02x ta=%02x tb=%02x\n",
                    (unsigned long long)sim_time, dut->rst, dut->cyc_en,
                    dut->cyc_done, dut->dbg_pc, dut->dbg_w, dut->porta_out,
                    dut->portb_out, dut->porta_dir, dut->portb_dir);
    }
}

static void reset() {
    dut->rst = 1;
    dut->cyc_en = 1;
    dut->porta_in = 0;
    dut->portb_in = 0x3c;
    tick();
    tick();
    dut->rst = 0;
}

static bool check_eq(const char* name, uint32_t got, uint32_t exp) {
    if (got != exp) {
        std::printf("  FAIL %s: got 0x%02x, expected 0x%02x\n",
                    name, got, exp);
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    cycle_trace = Verilated::commandArgsPlusMatch("pic_cycle_trace")[0] != '\0';
    dut = new Vpic16c5x;
    reset();

    const char* names[] = {
        "MOVLW+MOVWF: W->RAM",
        "ADDWF carry+Z flags",
        "SUBWF borrow semantics",
        "BSF/BCF bit set/clear",
        "BTFSS skip taken",
        "GOTO jump",
        "CALL+RETLW: 2-level stack",
        "INCFSZ skip",
        "RLF rotate through carry",
        "Indirect via FSR/INDF",
        "TRIS sets porta_dir",
        "Open-drain port read (TRIS-independent)",
        "ADDWF PCL computed redirect"
    };

    int passed = 0;
    bool failed = false;

    // Run full-speed, sparse enables and deterministic irregular enables.
    // Each reset must fetch 0x1ff before the very first enabled rising edge.
    for (int mode = 0; mode < 3 && !failed; mode++) {
      reset();
      if (!check_eq("reset PC", dut->dbg_pc, 0x1ff)) failed = true;
      tick();
      if (!check_eq("reset-vector GOTO target", dut->dbg_pc, 0) ||
          !check_eq("reset-vector retirement", dut->cyc_done, 1)) failed = true;
      tick();
      if (!check_eq("GOTO bubble PC", dut->dbg_pc, 0) ||
          !check_eq("GOTO bubble retirement", dut->cyc_done, 0)) failed = true;
      int next = 1;
      for (int cycle = 0; cycle < 2000 && next <= 13 && !failed; cycle++) {
        const int idle = mode == 0 ? 0 : mode == 1 ? 3 : ((cycle * 17 + 3) % 8);
        dut->cyc_en = 0;
        for (int gap = 0; gap < idle; gap++) tick();
        dut->cyc_en = 1;
        tick();
        uint8_t out = dut->portb_out & 0xff;
        if (out == 0xee) {
            std::printf("  FAIL firmware reported scenario failure at pc=0x%03x\n",
                        dut->dbg_pc & 0x1ff);
            failed = true;
        } else if (out == (0x80 | next)) {
            bool ok = true;
            if (next == 11) {
                ok = check_eq("porta_dir after TRIS", dut->porta_dir & 0xff, 0x0f);
            }
            std::printf("%s %s\n", ok ? "PASS" : "FAIL", names[next - 1]);
            if (!ok) {
                failed = true;
            } else {
                passed++;
                next++;
            }
        }
      }

      if (next <= 13 && !failed) {
        std::printf("  FAIL timeout waiting for scenario %d, pc=0x%03x out=0x%02x\n",
                    next, dut->dbg_pc & 0x1ff, dut->portb_out & 0xff);
        failed = true;
      }
    }

    std::printf("%d/39 scenarios passed\n", passed);
    delete dut;
    return failed || passed != 39 ? 1 : 0;
}
