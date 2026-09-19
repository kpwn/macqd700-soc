// tb_axil_null_slave.cpp — Verilator unit tb for rtl/soc/axil_null_slave.v
//
// axil_null_slave terminates a reserved address window whose peripheral is
// gone — currently the xbar S2 "DMA config" hole at 0x5010_0000 after
// dma_ctrl was removed (rtl/soc/fpga_top_dma.vh).  Its whole reason to
// exist is that a stray access COMPLETES instead of parking a burst on
// the interconnect forever, so what has to be proven is protocol
// liveness, not data.
//
// The upstream driver is rtl/soc/axi_wide_to_axilite.v, which tracks
// `wr_aw_done` and `wr_w_done` independently and can therefore present AW
// and W in either order, or together.  Scenarios 2-4 exist specifically
// because a terminator that required both handshakes in the same cycle
// would deadlock that bridge.
//
// What we test:
//   1. reset_idle              — out of reset: no parked response, both
//                                write halves ready, read channel ready.
//   2. write_aw_then_w         — AW first, W later → exactly one BVALID,
//                                BRESP=OKAY.
//   3. write_w_then_aw         — W first, AW later → same.
//   4. write_aw_and_w_together — both in one cycle → same.
//   5. read_returns_zero_okay  — AR → RVALID, RDATA=0, RRESP=OKAY.
//   6. no_second_response      — B/R are single-beat: after the response
//                                is taken, nothing else appears while the
//                                bus is idle.
//   7. backpressure_holds      — with BREADY/RREADY low the response is
//                                held stable, and the slave refuses new
//                                requests (awready/wready/arready low)
//                                rather than overwriting it.
//   8. back_to_back_writes     — 4 writes in a row all complete; proves
//                                aw_seen/w_seen actually clear.
//   9. back_to_back_reads      — same for the read channel.
//  10. reset_clears_parked     — assert reset with a response parked and
//                                confirm it is dropped, then the slave
//                                still works.
//  11. no_valid_to_ready_comb  — *ready must not be a function of *valid
//                                in the same cycle (that would be a
//                                handshake loop with the bridge).
//
// Build: `make tb-axil-null-slave`.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vaxil_null_slave.h"

static Vaxil_null_slave* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;

#define CHECK(name, cond)                                                   \
    do {                                                                    \
        if (cond) { n_pass++; }                                             \
        else {                                                              \
            n_fail++;                                                       \
            std::printf("  FAIL %s (t=%llu)\n", (name),                     \
                        (unsigned long long)sim_time);                      \
        }                                                                   \
    } while (0)

#define CHECK_EQ(name, got, exp)                                            \
    do {                                                                    \
        uint32_t g_ = (uint32_t)(got), e_ = (uint32_t)(exp);                \
        if (g_ == e_) { n_pass++; }                                         \
        else {                                                              \
            n_fail++;                                                       \
            std::printf("  FAIL %s: got 0x%08x expected 0x%08x (t=%llu)\n", \
                        (name), g_, e_, (unsigned long long)sim_time);      \
        }                                                                   \
    } while (0)

static void eval() { dut->eval(); }

// Posedge with the current input values, then settle.
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->awaddr  = 0; dut->awvalid = 0;
    dut->wdata   = 0; dut->wstrb   = 0; dut->wvalid = 0;
    dut->bready  = 0;
    dut->araddr  = 0; dut->arvalid = 0;
    dut->rready  = 0;
}

static void do_reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
    eval();
}

// Drive one write with a configurable AW/W skew, consuming B.
// skew < 0 : W leads AW by |skew| cycles.  skew > 0 : AW leads W.
// Returns cycles spent, or -1 on timeout (a deadlock is exactly the
// failure mode this module exists to prevent).
static int do_write(uint32_t addr, uint32_t data, int skew, int timeout = 64) {
    bool aw_done = false, w_done = false, b_done = false;
    int aw_at = (skew > 0) ? 0 : -skew;
    int w_at  = (skew > 0) ? skew : 0;
    // NB: the AW/W issue cycle counts from the start of this call.
    for (int c = 0; c < timeout; c++) {
        dut->awvalid = (!aw_done && c >= aw_at) ? 1 : 0;
        dut->awaddr  = addr;
        dut->wvalid  = (!w_done && c >= w_at) ? 1 : 0;
        dut->wdata   = data;
        dut->wstrb   = 0xF;
        dut->bready  = 1;
        eval();
        bool aw_fire = dut->awvalid && dut->awready;
        bool w_fire  = dut->wvalid  && dut->wready;
        bool b_fire  = dut->bvalid  && dut->bready;
        if (b_fire) {
            CHECK_EQ("write BRESP OKAY", dut->bresp, 0);
            b_done = true;
        }
        tick();
        if (aw_fire) aw_done = true;
        if (w_fire)  w_done  = true;
        if (b_done) {
            idle_inputs(); eval();
            return c + 1;
        }
    }
    idle_inputs(); eval();
    return -1;
}

static int do_read(uint32_t addr, uint32_t& data_out, int timeout = 64) {
    bool ar_done = false;
    for (int c = 0; c < timeout; c++) {
        dut->arvalid = ar_done ? 0 : 1;
        dut->araddr  = addr;
        dut->rready  = 1;
        eval();
        bool ar_fire = dut->arvalid && dut->arready;
        bool r_fire  = dut->rvalid  && dut->rready;
        if (r_fire) {
            data_out = dut->rdata;
            CHECK_EQ("read RRESP OKAY", dut->rresp, 0);
            tick();
            idle_inputs(); eval();
            return c + 1;
        }
        tick();
        if (ar_fire) ar_done = true;
    }
    idle_inputs(); eval();
    return -1;
}

// ══════════════════════════════════════════════════════════════════════
static bool test_reset_idle() {
    do_reset();
    CHECK("reset: no parked BVALID", dut->bvalid == 0);
    CHECK("reset: no parked RVALID", dut->rvalid == 0);
    CHECK("reset: AWREADY high",     dut->awready == 1);
    CHECK("reset: WREADY high",      dut->wready  == 1);
    CHECK("reset: ARREADY high",     dut->arready == 1);
    return true;
}

static bool test_write_aw_then_w() {
    do_reset();
    int cyc = do_write(0x00100, 0xDEADBEEF, +3);
    CHECK("AW-then-W write completes (no deadlock)", cyc > 0);
    return true;
}

static bool test_write_w_then_aw() {
    do_reset();
    int cyc = do_write(0x00104, 0xCAFEF00D, -3);
    CHECK("W-then-AW write completes (no deadlock)", cyc > 0);
    return true;
}

static bool test_write_aw_and_w_together() {
    do_reset();
    int cyc = do_write(0x00108, 0x12345678, 0);
    CHECK("same-cycle AW+W write completes", cyc > 0);
    return true;
}

static bool test_read_returns_zero_okay() {
    do_reset();
    uint32_t d = 0xFFFFFFFFu;
    int cyc = do_read(0x0010C, d);
    CHECK("read completes (no deadlock)", cyc > 0);
    CHECK_EQ("read data is zero", d, 0);
    return true;
}

static bool test_no_second_response() {
    do_reset();
    CHECK("write completes", do_write(0x00110, 0xA5A5A5A5, 0) > 0);
    // Bus idle, BREADY high: nothing more may appear.
    dut->bready = 1; dut->rready = 1; eval();
    bool spurious = false;
    for (int i = 0; i < 16; i++) {
        if (dut->bvalid || dut->rvalid) spurious = true;
        tick();
    }
    CHECK("no spurious second B/R response", !spurious);
    idle_inputs(); eval();
    return true;
}

static bool test_backpressure_holds() {
    do_reset();
    // Land AW+W together, then hold BREADY low.
    dut->awvalid = 1; dut->awaddr = 0x00120;
    dut->wvalid  = 1; dut->wdata  = 0xBEEF; dut->wstrb = 0xF;
    dut->bready  = 0;
    eval();
    CHECK("AW accepted", dut->awready == 1);
    CHECK("W accepted",  dut->wready  == 1);
    tick();
    dut->awvalid = 0; dut->wvalid = 0;
    eval();
    CHECK("BVALID parked", dut->bvalid == 1);
    // Held stable, and no new request accepted while parked.
    bool held = true, took_new = false;
    for (int i = 0; i < 16; i++) {
        if (!dut->bvalid || dut->bresp != 0) held = false;
        if (dut->awready || dut->wready)     took_new = true;
        tick(); eval();
    }
    CHECK("BVALID held stable under backpressure", held);
    CHECK("no new write accepted while B parked", !took_new);
    // Release.
    dut->bready = 1; eval();
    tick();
    dut->bready = 0; eval();
    CHECK("BVALID cleared after BREADY", dut->bvalid == 0);
    CHECK("AWREADY returns", dut->awready == 1);

    // Same for reads.
    dut->arvalid = 1; dut->araddr = 0x00124; dut->rready = 0;
    eval();
    CHECK("AR accepted", dut->arready == 1);
    tick();
    dut->arvalid = 0; eval();
    CHECK("RVALID parked", dut->rvalid == 1);
    bool r_held = true, r_took_new = false;
    for (int i = 0; i < 16; i++) {
        if (!dut->rvalid || dut->rdata != 0 || dut->rresp != 0) r_held = false;
        if (dut->arready) r_took_new = true;
        tick(); eval();
    }
    CHECK("RVALID held stable under backpressure", r_held);
    CHECK("no new read accepted while R parked", !r_took_new);
    dut->rready = 1; eval();
    tick();
    dut->rready = 0; eval();
    CHECK("RVALID cleared after RREADY", dut->rvalid == 0);
    CHECK("ARREADY returns", dut->arready == 1);
    idle_inputs(); eval();
    return true;
}

static bool test_back_to_back_writes() {
    do_reset();
    for (int i = 0; i < 4; i++) {
        char lbl[48];
        std::snprintf(lbl, sizeof(lbl), "back-to-back write %d completes", i);
        CHECK(lbl, do_write(0x00200 + 4 * i, 0x1000 + i, 0) > 0);
    }
    return true;
}

static bool test_back_to_back_reads() {
    do_reset();
    for (int i = 0; i < 4; i++) {
        uint32_t d = 0xFFFFFFFFu;
        char lbl[48];
        std::snprintf(lbl, sizeof(lbl), "back-to-back read %d completes", i);
        CHECK(lbl, do_read(0x00300 + 4 * i, d) > 0);
        CHECK_EQ("back-to-back read data zero", d, 0);
    }
    return true;
}

static bool test_reset_clears_parked() {
    do_reset();
    // Park a B and an R, then reset.
    dut->awvalid = 1; dut->awaddr = 0x00400;
    dut->wvalid  = 1; dut->wdata  = 0x55; dut->wstrb = 0xF;
    dut->arvalid = 1; dut->araddr = 0x00404;
    dut->bready = 0; dut->rready = 0;
    eval(); tick();
    dut->awvalid = 0; dut->wvalid = 0; dut->arvalid = 0;
    eval();
    CHECK("parked B before reset", dut->bvalid == 1);
    CHECK("parked R before reset", dut->rvalid == 1);
    dut->rst = 1; eval(); tick();
    dut->rst = 0; eval();
    CHECK("reset dropped parked B", dut->bvalid == 0);
    CHECK("reset dropped parked R", dut->rvalid == 0);
    idle_inputs(); eval();
    // Still functional afterwards.
    CHECK("write works after reset", do_write(0x00408, 0x99, 0) > 0);
    uint32_t d = 0xFFFFFFFFu;
    CHECK("read works after reset", do_read(0x0040C, d) > 0);
    return true;
}

// A *ready that moves when only the matching *valid moves would be a
// combinational valid->ready path, i.e. a handshake loop with the
// upstream bridge.  Sample each ready with valid low and high in the
// same settled state and require no change.
static bool test_no_valid_to_ready_comb() {
    do_reset();
    dut->awvalid = 0; dut->wvalid = 0; dut->arvalid = 0; eval();
    int aw0 = dut->awready, w0 = dut->wready, ar0 = dut->arready;
    dut->awvalid = 1; dut->wvalid = 1; dut->arvalid = 1; eval();
    CHECK("AWREADY not combinational on AWVALID", dut->awready == aw0);
    CHECK("WREADY not combinational on WVALID",   dut->wready  == w0);
    CHECK("ARREADY not combinational on ARVALID", dut->arready == ar0);
    idle_inputs(); eval();
    return true;
}

#define RUN(fn)                                                             \
    do {                                                                    \
        int before = n_fail;                                                \
        fn();                                                               \
        std::printf("[%s] %s\n", (n_fail == before) ? "PASS" : "FAIL", #fn); \
    } while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaxil_null_slave;

    RUN(test_reset_idle);
    RUN(test_write_aw_then_w);
    RUN(test_write_w_then_aw);
    RUN(test_write_aw_and_w_together);
    RUN(test_read_returns_zero_okay);
    RUN(test_no_second_response);
    RUN(test_backpressure_holds);
    RUN(test_back_to_back_writes);
    RUN(test_back_to_back_reads);
    RUN(test_reset_clears_parked);
    RUN(test_no_valid_to_ready_comb);

    std::printf("\n=== tb_axil_null_slave: %d checks passed, %d failed ===\n",
                n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
