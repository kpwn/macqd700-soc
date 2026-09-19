// tb_axi_dbg_split.cpp — prove the private debug path survives a wedged
// system path.
//
// The property under test is the reason the module exists: on p150
// (0xE934ECC7) a stalled peripheral wedged the shared fabric and EVERY
// JTAG-AXI read failed, including reads to DRAM that had nothing to do
// with the stall, because the single-outstanding narrow-to-wide adapter
// head-of-line blocked them.  Here we hold the system port permanently
// not-ready and require debug reads and writes to still complete.
//
// A positive control is included: with the split bypassed (i.e. a request
// aimed at the SYSTEM window while that window is wedged) the transaction
// must NOT complete.  Without it, a testbench that never wedges anything
// would pass trivially.

#include <cstdio>
#include <cstdlib>
#include <verilated.h>
#include "Vaxi_dbg_bus.h"

static Vaxi_dbg_bus *dut;
static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

static int n_pass = 0, n_fail = 0;
static void ok(const char *what, bool cond) {
    if (cond) { std::printf("  [PASS] %s\n", what); ++n_pass; }
    else      { std::printf("  [FAIL] %s\n", what); ++n_fail; }
}

static void tick() {
    dut->clk = 0; dut->eval(); ++main_time;
    dut->clk = 1; dut->eval(); ++main_time;
}

// The system path is WEDGED for the whole run: never ready, never responds.
static void wedge_system_port() {
    dut->m_awready = 0;
    dut->m_wready  = 0;
    dut->m_bvalid  = 0;
    dut->m_arready = 0;
    dut->m_rvalid  = 0;
}

// The debug path is a trivially-responsive register file.
static void debug_port_defaults() {
    dut->d_awready = 1;
    dut->d_wready  = 1;
    dut->d_bvalid  = 0;
    dut->d_arready = 1;
    dut->d_rvalid  = 0;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaxi_dbg_bus;

    dut->rst = 1; dut->clk = 0;
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_bready = 0;
    dut->s_arvalid = 0; dut->s_rready = 0;
    wedge_system_port();
    debug_port_defaults();
    for (int i = 0; i < 5; ++i) tick();
    dut->rst = 0;
    for (int i = 0; i < 2; ++i) tick();

    std::printf("[RUN ] debug READ completes while the system path is WEDGED\n");
    {
        dut->s_araddr  = 0x50900044;   // inside the debug window
        dut->s_arlen   = 0;
        dut->s_arvalid = 1;
        dut->s_rready  = 1;

        bool ar_taken = false, routed_to_dbg = false;
        for (int i = 0; i < 50 && !ar_taken; ++i) {
            dut->eval();
            if (dut->d_arvalid) routed_to_dbg = true;
            if (dut->s_arready && dut->s_arvalid) ar_taken = true;
            tick();
        }
        dut->s_arvalid = 0;
        ok("AR routed to the DEBUG port", routed_to_dbg);
        ok("AR accepted despite the wedged system port", ar_taken);

        // Debug slave returns data.
        dut->d_rvalid = 1; dut->d_rdata = 0xDEADBEEF;
        dut->d_rresp = 0;  dut->d_rlast = 1;
        bool got = false;
        for (int i = 0; i < 50 && !got; ++i) {
            dut->eval();
            if (dut->s_rvalid && dut->s_rready) {
                got = (dut->s_rdata == 0xDEADBEEF);
            }
            tick();
        }
        dut->d_rvalid = 0;
        ok("debug READ DATA returned to the master", got);
    }

    std::printf("[RUN ] debug WRITE completes while the system path is WEDGED\n");
    {
        dut->s_awaddr  = 0x50900010;
        dut->s_awlen   = 0;
        dut->s_awvalid = 1;
        bool aw_taken = false, routed = false;
        for (int i = 0; i < 50 && !aw_taken; ++i) {
            dut->eval();
            if (dut->d_awvalid) routed = true;
            if (dut->s_awready && dut->s_awvalid) aw_taken = true;
            tick();
        }
        dut->s_awvalid = 0;
        ok("AW routed to the DEBUG port", routed);
        ok("AW accepted despite the wedged system port", aw_taken);

        dut->s_wdata = 0x12345678; dut->s_wstrb = 0xF;
        dut->s_wlast = 1; dut->s_wvalid = 1;
        bool w_taken = false;
        for (int i = 0; i < 50 && !w_taken; ++i) {
            dut->eval();
            if (dut->s_wready && dut->s_wvalid) w_taken = true;
            tick();
        }
        dut->s_wvalid = 0;
        ok("W accepted on the debug path", w_taken);

        dut->d_bvalid = 1; dut->d_bresp = 0; dut->s_bready = 1;
        bool b_got = false;
        for (int i = 0; i < 50 && !b_got; ++i) {
            dut->eval();
            if (dut->s_bvalid && dut->s_bready) b_got = true;
            tick();
        }
        dut->d_bvalid = 0; dut->s_bready = 0;
        ok("write RESPONSE returned to the master", b_got);
    }

    std::printf("[RUN ] POSITIVE CONTROL: a SYSTEM-window read must NOT complete\n");
    {
        // Same wedge, but now target an address outside the debug window.
        // If this "passes" too, the test proves nothing.
        dut->s_araddr  = 0x00000000;   // low RAM -> system path
        dut->s_arvalid = 1;
        dut->s_rready  = 1;
        bool completed = false, routed_dbg = false;
        for (int i = 0; i < 200; ++i) {
            dut->eval();
            if (dut->d_arvalid) routed_dbg = true;
            if (dut->s_rvalid && dut->s_rready) completed = true;
            tick();
        }
        dut->s_arvalid = 0; dut->s_rready = 0;
        ok("system read did NOT leak onto the debug port", !routed_dbg);
        ok("system read correctly STALLS on the wedged path", !completed);
    }

    // ─────────────────────────────────────────────────────────────
    // THE REGRESSION TEST: a STUCK SYSTEM READ must not block DEBUG.
    //
    // The first version of this module shared one busy latch per
    // direction.  A system read to a wedged peripheral set it and never
    // cleared it, so every later debug read was refused BY THIS MODULE.
    // Hardware caught it (p152) after the unit tb passed, because the tb
    // only ever tested debug-while-wedged, never system-first-then-debug.
    // ─────────────────────────────────────────────────────────────
    std::printf("[RUN ] a STUCK system read must NOT block later debug reads\n");
    {
        // 1. Issue a system read that will never be answered.
        dut->m_arready = 1;              // system accepts the request...
        dut->m_rvalid  = 0;              // ...and then never responds.
        dut->s_araddr  = 0x00000000;
        dut->s_arvalid = 1;
        dut->s_rready  = 1;
        bool sys_taken = false;
        for (int i = 0; i < 50 && !sys_taken; ++i) {
            dut->eval();
            if (dut->s_arready && dut->s_arvalid) sys_taken = true;
            tick();
        }
        dut->s_arvalid = 0;
        dut->m_arready = 0;              // wedged from here on
        ok("system read was accepted (now permanently outstanding)", sys_taken);

        // 2. With that read still outstanding, a DEBUG read must work.
        dut->s_araddr  = 0x50900044;
        dut->s_arvalid = 1;
        bool dbg_taken = false;
        for (int i = 0; i < 50 && !dbg_taken; ++i) {
            dut->eval();
            if (dut->s_arready && dut->s_arvalid) dbg_taken = true;
            tick();
        }
        dut->s_arvalid = 0;
        ok("DEBUG read accepted while a system read is stuck", dbg_taken);

        dut->d_rvalid = 1; dut->d_rdata = 0xCAFEBABE;
        dut->d_rresp = 0;  dut->d_rlast = 1;
        bool dbg_got = false;
        for (int i = 0; i < 50 && !dbg_got; ++i) {
            dut->eval();
            if (dut->s_rvalid && dut->s_rready && dut->s_rdata == 0xCAFEBABE)
                dbg_got = true;
            tick();
        }
        dut->d_rvalid = 0;
        ok("DEBUG data returned while a system read is stuck", dbg_got);
    }

    std::printf("[RUN ] same property for WRITES\n");
    {
        dut->m_awready = 1; dut->m_bvalid = 0;   // system accepts, never responds
        dut->s_awaddr  = 0x00000000;
        dut->s_awvalid = 1;
        bool sys_aw = false;
        for (int i = 0; i < 50 && !sys_aw; ++i) {
            dut->eval();
            if (dut->s_awready && dut->s_awvalid) sys_aw = true;
            tick();
        }
        dut->s_awvalid = 0; dut->m_awready = 0;
        ok("system write accepted (now permanently outstanding)", sys_aw);

        dut->s_awaddr  = 0x50900010;
        dut->s_awvalid = 1;
        bool dbg_aw = false;
        for (int i = 0; i < 50 && !dbg_aw; ++i) {
            dut->eval();
            if (dut->s_awready && dut->s_awvalid) dbg_aw = true;
            tick();
        }
        dut->s_awvalid = 0;
        ok("DEBUG write accepted while a system write is stuck", dbg_aw);
    }

    // ── BURST into the debug window must take the SYSTEM branch ──────
    // The local slaves are AXI-Lite: one AW, one W, one B.  A burst has to
    // go out the system port so the crossbar's burst legalization
    // (is_lite_only_slv includes XBAR_SLV_IO) answers it SLVERR, exactly as
    // it did when this window was a crossbar slave.  If it were routed to
    // d_* the AXI-Lite slave would take the AW and then see W beats it has
    // no AW for.
    std::printf("[RUN ] a BURST into the debug window is routed to the SYSTEM port\n");
    {
        dut->rst = 1; for (int i = 0; i < 3; ++i) tick(); dut->rst = 0;
        wedge_system_port();
        debug_port_defaults();
        for (int i = 0; i < 2; ++i) tick();

        // Read burst: len = 3 (four beats) at a debug-window address.
        dut->s_araddr  = 0x50900044;
        dut->s_arlen   = 3;
        dut->s_arvalid = 1;
        dut->s_rready  = 1;
        bool saw_dbg_ar = false, saw_sys_ar = false;
        for (int i = 0; i < 20; ++i) {
            dut->eval();
            if (dut->d_arvalid) saw_dbg_ar = true;
            if (dut->m_arvalid) saw_sys_ar = true;
            tick();
        }
        dut->s_arvalid = 0;
        dut->s_arlen   = 0;
        ok("burst AR was NOT offered to the AXI-Lite debug port", !saw_dbg_ar);
        ok("burst AR went out the system port instead", saw_sys_ar);

        // Write burst: same property.
        dut->s_awaddr  = 0x50900010;
        dut->s_awlen   = 3;
        dut->s_awvalid = 1;
        bool saw_dbg_aw = false, saw_sys_aw = false;
        for (int i = 0; i < 20; ++i) {
            dut->eval();
            if (dut->d_awvalid) saw_dbg_aw = true;
            if (dut->m_awvalid) saw_sys_aw = true;
            tick();
        }
        dut->s_awvalid = 0;
        dut->s_awlen   = 0;
        ok("burst AW was NOT offered to the AXI-Lite debug port", !saw_dbg_aw);
        ok("burst AW went out the system port instead", saw_sys_aw);
    }

    std::printf("\n%d passed, %d failed.\n", n_pass, n_fail);
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
