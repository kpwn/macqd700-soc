// tb_sd_bridge_pulse.cpp — standalone pulse-conservation measurement of
// sd_scsi_bridge.v's core→pb read-byte CDC (single-byte latch + toggle
// handshake, rtl/soc/sd_scsi_bridge.v:189-253).  MEASUREMENT ONLY.
//
// Question under test: does the toggle CDC conserve core_rd_valid
// pulses into pb_rd_valid pulses?  The mechanism under suspicion: two
// core-side pulses inside the pb-side 2-FF sync + edge-detect window
// flip the toggle twice — the pb side then sees at most one edge (one
// or both bytes lost, latched data overwritten).
//
// Clocks: core_clk stepped every cycle, pb_clk at half rate (the real
// SoC runs pb=50 MHz, core=100/200 MHz — this models the 2:1 shape).
// For each pulse spacing SP (core cycles between rd_valid pulses) we
// send 64 incrementing bytes and count pb-side pulses + check the
// received sequence for skips/duplicates.
//
// Build via:   make tb-sd-bridge-pulse

#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vsd_scsi_bridge.h"

static Vsd_scsi_bridge* dut = nullptr;
static long cyc = 0;
static std::vector<uint8_t> rx;
static int rx_pulses = 0;

// One full core_clk cycle; pb_clk runs at half rate (posedge on even
// cycles).  pb-side registered outputs are sampled right after the pb
// posedge eval.
static void step_core() {
    dut->core_clk = 1;
    bool pb_edge = (cyc % 2 == 0);
    if (pb_edge) dut->pb_clk = 1;
    dut->eval();
    if (pb_edge && dut->pb_rd_valid) {
        ++rx_pulses;
        rx.push_back(dut->pb_rd_data);
    }
    dut->core_clk = 0;
    if (!pb_edge) dut->pb_clk = 0;
    dut->eval();
    ++cyc;
}

static void reset_all() {
    dut->pb_rst = 1;
    dut->core_rst = 1;
    dut->pb_go = 0;
    dut->pb_rd_ready = 1;
    dut->pb_wr_avail = 1;
    dut->core_rd_valid = 0;
    dut->core_rd_data = 0;
    dut->core_busy = 0;
    dut->core_done = 0;
    dut->core_error = 0;
    dut->core_wr_ready = 0;
    for (int i = 0; i < 8; ++i) step_core();
    dut->pb_rst = 0;
    dut->core_rst = 0;
    for (int i = 0; i < 8; ++i) step_core();
    rx.clear();
    rx_pulses = 0;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vsd_scsi_bridge;
    int n_fail = 0;

    const int spacings[] = {1, 2, 3, 4, 6, 8, 16};
    const int NPULSE = 64;
    std::printf("sd_scsi_bridge core->pb read CDC pulse conservation "
                "(pb_clk = core_clk/2)\n");
    for (int sp : spacings) {
        reset_all();
        for (int p = 0; p < NPULSE; ++p) {
            dut->core_rd_valid = 1;
            dut->core_rd_data  = (uint8_t)p;
            step_core();
            dut->core_rd_valid = 0;
            for (int g = 1; g < sp; ++g) step_core();
        }
        for (int i = 0; i < 32; ++i) step_core();   // drain
        int skips = 0, dups = 0;
        for (size_t i = 1; i < rx.size(); ++i) {
            int d = (int)rx[i] - (int)rx[i - 1];
            if (d == 0) ++dups;
            else if (d != 1) skips += d - 1;
        }
        bool ok = (rx_pulses == NPULSE) && (skips == 0) && (dups == 0);
        std::printf("  SP=%-2d : pulses in=%d out=%d lost=%d "
                    "data-skips=%d dups=%d %s\n",
                    sp, NPULSE, rx_pulses, NPULSE - rx_pulses,
                    skips, dups, ok ? "OK" : "**LOSS**");
        if (!ok) ++n_fail;
    }

    // Adjacent-cycle pulse PAIRS with wide spacing between pairs — the
    // exact toggle-swallow shape (two flips before the pb edge-detector
    // samples).
    {
        reset_all();
        const int NPAIR = 32;
        for (int p = 0; p < NPAIR; ++p) {
            dut->core_rd_valid = 1;
            dut->core_rd_data  = (uint8_t)(2 * p);
            step_core();
            dut->core_rd_data  = (uint8_t)(2 * p + 1);
            step_core();
            dut->core_rd_valid = 0;
            for (int g = 0; g < 20; ++g) step_core();
        }
        for (int i = 0; i < 32; ++i) step_core();
        int expect = 2 * NPAIR;
        std::printf("  PAIRS (2 adjacent, 20-cycle gap): in=%d out=%d "
                    "lost=%d %s\n", expect, rx_pulses,
                    expect - rx_pulses,
                    (rx_pulses == expect) ? "OK" : "**LOSS**");
        if (rx_pulses != expect) ++n_fail;
    }

    std::printf("%s\n", n_fail ? "BRIDGE CDC LOSES PULSES in the shapes "
                                 "marked **LOSS** above."
                               : "All pulse shapes conserved.");
    delete dut;
    // Measurement tb: always exit 0 — the printed table is the product.
    return 0;
}
