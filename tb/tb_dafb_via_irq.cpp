// tb_dafb_via_irq.cpp -- DAFB vblank to VIA1 to irq_agg route test
//
// Build via: make tb-dafb-via-irq

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vtb_dafb_via_irq.h"

static Vtb_dafb_via_irq* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, fmt, ...) do { \
    if (cond) { n_pass++; std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); } \
    else { n_fail++; std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); } \
} while (0)

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->phi2_tick = 0;
    dut->frame_tick = 0;
    dut->dafb_awaddr = 0;
    dut->dafb_awvalid = 0;
    dut->dafb_wdata = 0;
    dut->dafb_wstrb = 0;
    dut->dafb_wvalid = 0;
    dut->dafb_bready = 0;
    dut->dafb_araddr = 0;
    dut->dafb_arvalid = 0;
    dut->dafb_rready = 0;
    dut->via_addr = 0;
    dut->via_wdata = 0;
    dut->via_wr = 0;
    dut->via_rd = 0;
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
}

static void run_cycles(int cycles) {
    idle_inputs();
    for (int i = 0; i < cycles; i++) tick();
}

// video.v's vblank source is now the externally-supplied frame_tick input
// (task T8 fix 2) rather than a free-running 1024-cycle counter.  Drive
// one rising edge, mirroring tb_dafb.cpp's pulse_frame_tick().
static void pulse_frame_tick() {
    idle_inputs();
    dut->frame_tick = 0;
    tick();
    dut->frame_tick = 1;
    tick();
    dut->frame_tick = 0;
    tick();
}

static int dafb_write(uint32_t addr, uint32_t data, uint32_t strb = 0xF) {
    dut->dafb_awaddr = addr;
    dut->dafb_awvalid = 1;
    dut->dafb_wdata = data;
    dut->dafb_wstrb = strb;
    dut->dafb_wvalid = 1;
    dut->dafb_bready = 1;

    bool aw_done = false;
    bool w_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (!aw_done && dut->dafb_awready) aw_done = true;
        if (!w_done && dut->dafb_wready) w_done = true;
        tick();
        if (aw_done) dut->dafb_awvalid = 0;
        if (w_done) dut->dafb_wvalid = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->dafb_bvalid) {
            int resp = dut->dafb_bresp;
            tick();
            dut->dafb_bready = 0;
            return resp == 0 ? 0 : 2;
        }
        tick();
    }
    return 3;
}

static int dafb_read(uint32_t addr, uint32_t* out) {
    dut->dafb_araddr = addr;
    dut->dafb_arvalid = 1;
    dut->dafb_rready = 1;

    bool ar_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->dafb_arready) {
            ar_done = true;
            tick();
            break;
        }
        tick();
    }
    dut->dafb_arvalid = 0;
    if (!ar_done) return 1;

    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->dafb_rvalid) {
            int resp = dut->dafb_rresp;
            *out = dut->dafb_rdata;
            tick();
            dut->dafb_rready = 0;
            return resp == 0 ? 0 : 2;
        }
        tick();
    }
    return 3;
}

static void via_write(uint8_t addr, uint8_t data) {
    dut->via_addr = addr & 0xF;
    dut->via_wdata = data;
    dut->via_wr = 1;
    dut->via_rd = 0;
    tick();
    dut->via_wr = 0;
    dut->via_wdata = 0;
}

static uint8_t via_read(uint8_t addr) {
    dut->via_addr = addr & 0xF;
    dut->via_rd = 1;
    dut->via_wr = 0;
    tick();
    dut->via_rd = 0;
    dut->eval();
    return dut->via_rdata & 0xFF;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_dafb_via_irq;

    std::printf("tb_dafb_via_irq: DAFB -> VIA1 -> irq_agg route\n");
    reset();

    CHECK(dut->dafb_irq == 0, "DAFB IRQ starts low");
    CHECK(dut->via1_irq == 0, "VIA1 IRQ starts low");
    CHECK(dut->ipl == 0, "aggregated IPL starts at 0");

    // MAME-canonical DAFB register layout (post-8752fbd7):
    //   +0x00 BASE_HI, +0x04 BASE_LO, +0x08 STRIDE, +0x10 CONFIG,
    //   +0x220 RAMDAC PCBR (drives fb_bpp_reg via r_pcbr_set).
    // fb_ready requires fb_base_px && fb_stride_px && fb_bpp_reg all
    // non-zero (see video.v fb_ready definition).
    CHECK(dafb_write(0x00, 0x00000001) == 0, "program DAFB base_hi");
    CHECK(dafb_write(0x08, 0x00000100) == 0, "program DAFB stride");
    CHECK(dafb_write(0x10, 0x00000000) == 0, "program DAFB config (8bpp default)");
    CHECK(dafb_write(0x220, 0x00000018) == 0, "program DAFB PCBR=8bpp"); // r_pcbr_set=1
    CHECK(dafb_write(0x1C, 0x00000001) == 0, "enable DAFB vblank IRQ");
    via_write(12, 0x01); // direct DAFB IRQ test uses CA1 positive edge
    via_write(14, 0x82); // VIA1 CA1/VBL enable

    pulse_frame_tick();
    dut->eval();
    uint32_t status = 0;
    CHECK(dafb_read(0x20, &status) == 0, "read DAFB IRQ status");
    CHECK(status == 0x00000003, "DAFB status is pending+enabled (0x%08x)", status);
    CHECK(dut->dafb_irq == 1, "DAFB IRQ level is high");

    uint8_t ifr = via_read(13);
    CHECK((ifr & 0x82) == 0x82, "VIA1 CA1 IFR summary set (0x%02x)", ifr);
    CHECK(dut->via1_irq == 1, "VIA1 IRQ propagates enabled CA1");
    CHECK(dut->ipl == 1, "irq_agg emits IPL 1 for DAFB via VIA1");

    (void)via_read(1);
    ifr = via_read(13);
    CHECK((ifr & 0x02) == 0x00, "VIA1 ORA read acknowledges CA1 (0x%02x)", ifr);
    CHECK(dut->via1_irq == 0, "VIA1 IRQ drops after VIA ack");
    CHECK(dut->ipl == 0, "aggregated IPL drops after VIA ack");
    CHECK(dut->dafb_irq == 1, "DAFB IRQ remains pending until DAFB ack");

    CHECK(dafb_write(0x20, 0x00000001) == 0, "ack DAFB vblank pending");
    CHECK(dafb_read(0x20, &status) == 0, "read DAFB status after ack");
    CHECK(status == 0x00000000, "DAFB status clears after ack (0x%08x)", status);
    CHECK(dut->dafb_irq == 0, "DAFB IRQ clears after DAFB ack");

    std::printf("result: pass=%d fail=%d\n", n_pass, n_fail);
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
