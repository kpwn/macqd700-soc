// tb_axil_split2.cpp — unit tb for rtl/soc/axil_split2.v
//
// The demux that lets pram_sd share the AXI_SD_JTAG_BASE slave window with
// sd_jtag_writer.  A routing bug here is the kind that shows up as
// "pram-save silently wrote the SD-JTAG writer's LBA register", so the
// test models BOTH downstream slaves as distinct register files and
// asserts every access landed on exactly one of them.
//
// Build: make tb-axil-split2

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vaxil_split2.h"

static Vaxil_split2* dut = nullptr;
static int n_pass = 0, n_fail = 0;

// Downstream slave models: 16 words each, plus per-slave access counters
// so "the write went to the right place" is checkable both ways.
struct Slave {
    uint32_t regs[16] = {0};
    int writes = 0;
    int reads  = 0;
};
static Slave s0, s1;

// Each slave: 1-deep, accepts AW+W together, answers B next cycle;
// accepts AR, answers R next cycle.
struct SlavePort {
    Slave* m;
    bool bvalid = false, rvalid = false;
    uint32_t rdata = 0;
    bool aw_seen = false, w_seen = false;
    uint32_t addr = 0, data = 0;
};
static SlavePort p0{&s0}, p1{&s1};

static void slave_pre(SlavePort& p,
                      uint8_t& awready, uint8_t& wready, uint8_t& arready,
                      uint8_t& bvalid, uint8_t& rvalid, uint32_t& rdata) {
    awready = p.bvalid ? 0 : 1;
    wready  = p.bvalid ? 0 : 1;
    arready = p.rvalid ? 0 : 1;
    bvalid  = p.bvalid ? 1 : 0;
    rvalid  = p.rvalid ? 1 : 0;
    rdata   = p.rdata;
}

static void slave_post(SlavePort& p,
                       uint8_t awvalid, uint32_t awaddr,
                       uint8_t wvalid, uint32_t wdata,
                       uint8_t bready,
                       uint8_t arvalid, uint32_t araddr,
                       uint8_t rready) {
    if (!p.bvalid) {
        if (awvalid) { p.aw_seen = true; p.addr = awaddr; }
        if (wvalid)  { p.w_seen  = true; p.data = wdata; }
        if (p.aw_seen && p.w_seen) {
            p.m->regs[(p.addr >> 2) & 0xF] = p.data;
            p.m->writes++;
            p.aw_seen = p.w_seen = false;
            p.bvalid = true;
        }
    } else if (bready) {
        p.bvalid = false;
    }

    if (!p.rvalid) {
        if (arvalid) {
            p.rdata = p.m->regs[(araddr >> 2) & 0xF];
            p.m->reads++;
            p.rvalid = true;
        }
    } else if (rready) {
        p.rvalid = false;
    }
}

static void tick() {
    // Drive downstream ready/valid from the models before the edge.
    uint8_t aw0, w0, ar0, b0, r0v; uint32_t r0d;
    uint8_t aw1, w1, ar1, b1, r1v; uint32_t r1d;
    slave_pre(p0, aw0, w0, ar0, b0, r0v, r0d);
    slave_pre(p1, aw1, w1, ar1, b1, r1v, r1d);
    dut->d0_awready = aw0; dut->d0_wready = w0; dut->d0_arready = ar0;
    dut->d0_bvalid  = b0;  dut->d0_bresp  = 0;
    dut->d0_rvalid  = r0v; dut->d0_rdata  = r0d; dut->d0_rresp = 0;
    dut->d1_awready = aw1; dut->d1_wready = w1; dut->d1_arready = ar1;
    dut->d1_bvalid  = b1;  dut->d1_bresp  = 0;
    dut->d1_rvalid  = r1v; dut->d1_rdata  = r1d; dut->d1_rresp = 0;

    dut->clk = 0; dut->eval();

    // Sample what the DUT is presenting downstream, gated by the models'
    // ready/valid, then advance the models.
    uint8_t d0aw = dut->d0_awvalid && aw0, d0w = dut->d0_wvalid && w0;
    uint8_t d0ar = dut->d0_arvalid && ar0;
    uint8_t d1aw = dut->d1_awvalid && aw1, d1w = dut->d1_wvalid && w1;
    uint8_t d1ar = dut->d1_arvalid && ar1;
    uint32_t d0awa = dut->d0_awaddr, d0wd = dut->d0_wdata;
    uint32_t d1awa = dut->d1_awaddr, d1wd = dut->d1_wdata;
    uint32_t d0ara = dut->d0_araddr, d1ara = dut->d1_araddr;
    uint8_t d0br = dut->d0_bready, d1br = dut->d1_bready;
    uint8_t d0rr = dut->d0_rready, d1rr = dut->d1_rready;

    dut->clk = 1; dut->eval();

    slave_post(p0, d0aw, d0awa, d0w, d0wd, d0br, d0ar, d0ara, d0rr);
    slave_post(p1, d1aw, d1awa, d1w, d1wd, d1br, d1ar, d1ara, d1rr);
}

static void reset() {
    s0 = Slave(); s1 = Slave();
    p0 = SlavePort{&s0}; p1 = SlavePort{&s1};
    dut->u_awvalid = 0; dut->u_wvalid = 0; dut->u_bready = 0;
    dut->u_arvalid = 0; dut->u_rready = 0;
    dut->u_awaddr = 0; dut->u_wdata = 0; dut->u_wstrb = 0xF; dut->u_araddr = 0;
    dut->rst = 1;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

static void wr(uint32_t addr, uint32_t data) {
    dut->u_awaddr = addr; dut->u_awvalid = 1;
    dut->u_wdata = data;  dut->u_wstrb = 0xF; dut->u_wvalid = 1;
    dut->u_bready = 0;
    int g = 0;
    while (!(dut->u_awready && dut->u_wready)) { tick(); if (++g > 500) break; }
    tick();
    dut->u_awvalid = 0; dut->u_wvalid = 0;
    g = 0;
    while (!dut->u_bvalid) { tick(); if (++g > 500) break; }
    dut->u_bready = 1; tick(); dut->u_bready = 0;
}

static uint32_t rd(uint32_t addr) {
    dut->u_araddr = addr; dut->u_arvalid = 1; dut->u_rready = 0;
    int g = 0;
    while (!dut->u_arready) { tick(); if (++g > 500) break; }
    tick();
    dut->u_arvalid = 0;
    g = 0;
    while (!dut->u_rvalid) { tick(); if (++g > 500) break; }
    uint32_t d = dut->u_rdata;
    dut->u_rready = 1; tick(); dut->u_rready = 0;
    return d;
}

#define CHECK_EQ(label, got, exp) do { \
    uint32_t g_ = (uint32_t)(got), e_ = (uint32_t)(exp); \
    if (g_ != e_) { printf("  FAIL %s: got 0x%x expected 0x%x\n", label, g_, e_); return false; } \
} while (0)

// Bit 8 low -> d0, high -> d1, and neither slave sees the other's traffic.
static bool test_routing() {
    reset();
    wr(0x004, 0xAAAA0001);           // d0
    wr(0x104, 0xBBBB0002);           // d1
    CHECK_EQ("d0 write count", s0.writes, 1);
    CHECK_EQ("d1 write count", s1.writes, 1);
    CHECK_EQ("d0 reg1", s0.regs[1], 0xAAAA0001u);
    CHECK_EQ("d1 reg1", s1.regs[1], 0xBBBB0002u);

    CHECK_EQ("read d0", rd(0x004), 0xAAAA0001u);
    CHECK_EQ("read d1", rd(0x104), 0xBBBB0002u);
    CHECK_EQ("d0 read count", s0.reads, 1);
    CHECK_EQ("d1 read count", s1.reads, 1);
    return true;
}

// A stream of alternating accesses must never leak across.
static bool test_interleaved() {
    reset();
    for (int i = 0; i < 16; i++) {
        wr(0x000 + i * 4, 0x1000 + i);
        wr(0x100 + i * 4, 0x2000 + i);
    }
    for (int i = 0; i < 16; i++) {
        CHECK_EQ("d0 readback", rd(0x000 + i * 4), (uint32_t)(0x1000 + i));
        CHECK_EQ("d1 readback", rd(0x100 + i * 4), (uint32_t)(0x2000 + i));
    }
    CHECK_EQ("d0 writes", s0.writes, 16);
    CHECK_EQ("d1 writes", s1.writes, 16);
    return true;
}

// W arriving before AW is legal in AXI-Lite and must still route by the
// address that turns up later.
static bool test_w_before_aw() {
    reset();
    dut->u_wdata = 0xC0DE0001; dut->u_wstrb = 0xF; dut->u_wvalid = 1;
    int g = 0;
    while (!dut->u_wready) { tick(); if (++g > 500) break; }
    tick();
    dut->u_wvalid = 0;
    for (int i = 0; i < 4; i++) tick();

    dut->u_awaddr = 0x108; dut->u_awvalid = 1;
    g = 0;
    while (!dut->u_awready) { tick(); if (++g > 500) break; }
    tick();
    dut->u_awvalid = 0;
    g = 0;
    while (!dut->u_bvalid) { tick(); if (++g > 500) break; }
    dut->u_bready = 1; tick(); dut->u_bready = 0;

    CHECK_EQ("d1 got it", s1.regs[2], 0xC0DE0001u);
    CHECK_EQ("d0 untouched", s0.writes, 0);
    return true;
}

// A downstream slave whose B response is purely COMBINATIONAL
// (bvalid = awvalid & wvalid) cannot work behind this store-and-forward
// split, and this scenario pins that down.
//
// This is not hypothetical.  fpga_top_sd.vh's `DISABLE_SD_JTAG_WRITER`
// arm — the one every PRODUCTION bitstream builds — terminated the
// SD-JTAG window with exactly that idiom.  It was correct while the only
// upstream was axi_wide_to_axilite (which holds AWVALID up until BVALID),
// and it silently becomes a write-channel deadlock the moment this split
// is inserted, because the split drops AWVALID/WVALID as soon as they are
// accepted and only THEN looks for BVALID.  The fix was to put a real
// axil_null_slave there.  If someone ever puts the combinational form
// back, this test says why they must not.
static bool test_combinational_tieoff_deadlocks() {
    reset();
    // Drive d0 by hand with the combinational idiom instead of using the
    // registered slave model.
    dut->u_awaddr = 0x004; dut->u_awvalid = 1;
    dut->u_wdata = 0xDEAD; dut->u_wstrb = 0xF; dut->u_wvalid = 1;
    dut->u_bready = 1;
    bool completed = false;
    for (int i = 0; i < 500; i++) {
        // Manual pre-edge drive: combinational tie-off on d0.
        dut->d0_awready = 1;
        dut->d0_wready  = 1;
        dut->d0_arready = 1;
        dut->d0_bvalid  = dut->d0_awvalid && dut->d0_wvalid;
        dut->d0_bresp   = 0;
        dut->d0_rvalid  = dut->d0_arvalid;
        dut->d0_rdata   = 0;
        dut->d0_rresp   = 0;
        dut->d1_awready = 1; dut->d1_wready = 1; dut->d1_arready = 1;
        dut->d1_bvalid  = 0; dut->d1_rvalid = 0;
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
        if (dut->u_bvalid) { completed = true; break; }
    }
    if (completed) {
        printf("  FAIL combinational tie-off unexpectedly completed — the\n"
               "       fpga_top_sd.vh hazard this scenario documents may have\n"
               "       changed shape; re-derive it before deleting the test.\n");
        return false;
    }
    return true;
}

static void run(const char* name, bool (*fn)()) {
    printf("Running %s...\n", name);
    if (fn()) { printf("  PASS %s\n", name); n_pass++; }
    else n_fail++;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaxil_split2;
    run("routing",     test_routing);
    run("interleaved", test_interleaved);
    run("w_before_aw", test_w_before_aw);
    run("combinational_tieoff_deadlocks", test_combinational_tieoff_deadlocks);
    printf("axil_split2: %d passed, %d failed\n", n_pass, n_fail);
    dut->final();
    delete dut;
    return n_fail ? 1 : 0;
}
