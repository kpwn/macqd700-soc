// tb_l2c_bypass_window.cpp — the RAM-disk L2 bypass window's MASK POLARITY
//
// l2c_bypass.v's mask convention is INVERTED relative to the "size mask"
// people reach for by reflex:
//
//   1 bits = FIXED base bits, which must equal BASE at those positions
//   0 bits = in-window offset, don't-cares
//   hit  <=>  (addr & mask) == (base & mask)
//
// So a 256 MB window at 0x5000_0000 is mask 0xF000_0000, NOT 0x0FFF_FFFF.
// Get it backwards and l2c silently CACHES the RAM disk instead of
// bypassing it — 256 MB of streamed disk blocks evicting the CPU's working
// set out of a 2 MB L2, with no error anywhere and nothing that fails.
// That is not a hypothetical failure mode: rtl/soc/axi_defs.vh's own
// comment block calls the polarity out precisely because it has caught
// people before.
//
// This file pins the polarity down as an executable fact, against the
// exact constants rtl/soc/fpga_top_ddr.vh instantiates.  The Makefile
// builds it TWICE — once with the real mask, once with the mask inverted —
// and requires the inverted build to FAIL.  Otherwise the test would be
// asserting nothing about polarity at all.
//
// Build-time parameters come from -G overrides; WIN_MASK_INVERTED tells
// the C++ which pass it is.

#include <verilated.h>
#include "Vl2c_bypass.h"
#include <cstdio>

#ifndef EXPECT_POLARITY_OK
#define EXPECT_POLARITY_OK 1
#endif

static Vl2c_bypass* d;
double sc_time_stamp() { return 0; }
static int fails = 0;

static void t(uint32_t a, bool want, const char* why) {
    d->match_addr = a;
    d->eval();
    bool got = d->match_hit;
    printf("  %s 0x%08X hit=%d want=%d   %s\n",
           got == want ? "ok:  " : "FAIL:", a, got, want, why);
    if (got != want) fails++;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    d = new Vl2c_bypass;
    d->clk = 0; d->rst = 1; d->req_valid = 0; d->rsp_ready = 1;
    d->eval();

    printf("=== l2c bypass window: RAM disk at flattened DDR 0x5000_0000 ===\n");
    printf("    (1 bits in MASK are FIXED base bits — see the header)\n");

    // Inside the window.
    t(0x50000000u, true,  "RAM-disk carveout base");
    t(0x50000010u, true,  "just inside the base");
    t(0x58000000u, true,  "middle of the 256 MB window");
    t(0x5FFFFFF0u, true,  "top of the 256 MB window");

    // Immediately outside, both edges.
    t(0x4FFFFFF0u, false, "one 16-byte beat BELOW the window");
    t(0x60000000u, false, "one beat ABOVE the window");

    // Everything the window must stay disjoint from — l2c_bypass.v
    // $fatal-asserts this at elaboration, but only for the cacheable
    // span; the VRAM carveout is checked here explicitly.
    t(0x00000000u, false, "RAM base — must stay CACHEABLE");
    t(0x3FFFFFF0u, false, "top of the 1 GiB RAM decode window — cacheable");
    t(0x40000000u, false, "ROM at flattened DDR — cacheable");
    t(0x40400000u, false, "framebuffer at flattened DDR — cacheable");
    t(0x40BFFFF0u, false, "top of the FB span — cacheable");
    t(0x46000000u, false, "VRAM-in-DDR carveout base — separate carveout");
    t(0x47FFFFF0u, false, "VRAM-in-DDR carveout top");

#if EXPECT_POLARITY_OK
    printf("\n%d checks failed\n", fails);
    if (fails) printf("POLARITY IS WRONG — the RAM disk would be CACHED.\n");
    return fails ? 1 : 0;
#else
    // Positive-control pass: the mask is deliberately inverted.  Success
    // here means the checks NOTICED.
    printf("\nPOSITIVE CONTROL (mask inverted): %d checks failed\n", fails);
    if (fails == 0)
        printf("POSITIVE CONTROL DID NOT FIRE — this test cannot detect a "
               "polarity error, so its green run proves nothing.\n");
    else
        printf("POSITIVE CONTROL OK: an inverted mask is detected.\n");
    return fails ? 0 : 1;
#endif
}
