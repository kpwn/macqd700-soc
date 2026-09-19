// tb_axi_w_skid.cpp — Verilator testbench for rtl/soc/axi_w_skid.v.
//
// The slice sits between the crossbar's S0 write port and the DDR-side
// slave (u_l2c, or u_vram_lane_mux/u_ddr with L2C_ENABLE off).  Nothing
// else in the tree covers it: `tb-axi-xbar` instantiates axi_xbar.v with
// BFM slaves and the slice lives one level up in fpga_top_xbar.vh, and
// `tb-l2c` drives l2c directly.  A W-channel slice that loses, duplicates,
// reorders or TEARS a beat corrupts DRAM writes silently, so it gets its
// own test.
//
// What is checked, on every cycle of randomised valid/ready pressure:
//
//   1. STREAM EQUALITY.  The exact sequence of {wdata, wstrb, wlast}
//      accepted upstream must come out downstream, in order, with nothing
//      added or dropped.  A model FIFO holds the expected sequence.
//   2. PAYLOAD STABILITY.  While m_wvalid is high and m_wready is low, the
//      downstream payload may not change.  This is the failure a beat-count
//      check cannot see: the right NUMBER of beats arrives, but a slave
//      that samples mid-stall gets a torn one.
//   3. READY IS REGISTERED.  s_wready must be a function of state only --
//      it must NOT move within a cycle when m_wready moves.  That is the
//      entire reason the module exists (it cuts the crossbar write FSM's
//      combinational path to the L2C front door), so it is asserted
//      directly: with the DUT settled, toggling m_wready and re-evaluating
//      must leave s_wready unchanged.
//   4. NO STALL-FREE THROUGHPUT LOSS.  With m_wready tied high and
//      s_wvalid tied high, the slice must accept one beat per cycle.
//   5. RESET leaves it empty and quiet.
//
// Build: make tb-axi-w-skid

#include "Vaxi_w_skid.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <cstdint>

static Vaxi_w_skid *dut;
static vluint64_t   main_time = 0;
double sc_time_stamp() { return main_time; }

static int failures = 0;

static void fail(const char *what, long cyc) {
    fprintf(stderr, "[FAIL] %s (cycle %ld)\n", what, cyc);
    if (++failures > 20) {
        fprintf(stderr, "too many failures, aborting\n");
        exit(1);
    }
}

struct Beat {
    uint32_t data[4];   // 128 bits, Verilator splits into 4 words
    uint32_t strb;
    uint8_t  last;
    bool operator!=(const Beat &o) const {
        for (int i = 0; i < 4; i++) if (data[i] != o.data[i]) return true;
        return strb != o.strb || last != o.last;
    }
};

static Beat capture_out() {
    Beat b;
    for (int i = 0; i < 4; i++) b.data[i] = dut->m_wdata[i];
    b.strb = dut->m_wstrb;
    b.last = dut->m_wlast;
    return b;
}

static void tick() {
    dut->clk = 0; dut->eval(); main_time++;
    dut->clk = 1; dut->eval(); main_time++;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaxi_w_skid;

    // ---- reset ----------------------------------------------------------
    dut->rst = 1; dut->s_wvalid = 0; dut->m_wready = 0; dut->clk = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->eval();
    if (dut->m_wvalid) fail("m_wvalid asserted out of reset", 0);
    if (!dut->s_wready) fail("s_wready low out of reset (slice should be empty)", 0);
    dut->rst = 0;

    // ---- 3: ready must be REGISTERED ------------------------------------
    // Drive the slice into the stalled-with-one-beat state, then wiggle
    // m_wready with no clock edge and confirm s_wready does not follow.
    dut->m_wready = 0;
    dut->s_wvalid = 1;
    dut->s_wdata[0] = 0xA5A5A5A5u; dut->s_wdata[1] = 0; dut->s_wdata[2] = 0; dut->s_wdata[3] = 0;
    dut->s_wstrb = 0xFFFFu; dut->s_wlast = 0;
    tick();                       // beat lands in the output register
    tick();                       // next beat lands in the skid
    dut->eval();
    {
        uint8_t before = dut->s_wready;
        dut->m_wready = 1; dut->eval();
        uint8_t after_high = dut->s_wready;
        dut->m_wready = 0; dut->eval();
        uint8_t after_low = dut->s_wready;
        if (before != after_high || before != after_low)
            fail("s_wready moved with m_wready inside one cycle -- the slice is "
                 "NOT cutting the combinational ready path, which is its whole purpose", 0);
    }

    // re-reset before the randomised phase
    dut->rst = 1; dut->s_wvalid = 0; dut->m_wready = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;

    // ---- 1 + 2: randomised stream equality and payload stability --------
    std::deque<Beat> model;
    long accepted = 0, delivered = 0;
    uint32_t seed = 12345;
    auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return (seed >> 16) & 0x7FFF; };

    bool  prev_stalled = false;
    Beat  prev_out{};

    for (long cyc = 0; cyc < 200000; cyc++) {
        // Drive a fresh beat offer and a random downstream ready.
        uint8_t offer = (rnd() % 100) < 70;
        Beat in;
        in.data[0] = rnd() | (rnd() << 15);
        in.data[1] = (uint32_t)accepted;          // sequence stamp
        in.data[2] = rnd();
        in.data[3] = rnd();
        in.strb    = rnd() & 0xFFFFu;
        in.last    = (rnd() % 8) == 0;

        dut->s_wvalid = offer;
        for (int i = 0; i < 4; i++) dut->s_wdata[i] = in.data[i];
        dut->s_wstrb = in.strb;
        dut->s_wlast = in.last;
        dut->m_wready = (rnd() % 100) < 60;
        dut->eval();

        // 2: payload stability across a stall.
        if (prev_stalled) {
            if (!dut->m_wvalid)
                fail("m_wvalid dropped while stalled (a beat was withdrawn)", cyc);
            Beat now = capture_out();
            if (now != prev_out)
                fail("W payload CHANGED while stalled at m_wvalid", cyc);
        }

        // Record the handshakes that this edge will perform.
        bool up   = dut->s_wvalid && dut->s_wready;
        bool down = dut->m_wvalid && dut->m_wready;
        if (down) {
            Beat got = capture_out();
            if (model.empty()) {
                fail("downstream produced a beat the upstream never sent", cyc);
            } else {
                if (got != model.front())
                    fail("downstream beat does not match the upstream beat (out of "
                         "order, or corrupted)", cyc);
                model.pop_front();
            }
            delivered++;
        }
        if (up) { model.push_back(in); accepted++; }

        prev_stalled = dut->m_wvalid && !dut->m_wready;
        prev_out     = capture_out();

        tick();
    }

    // Drain.
    dut->s_wvalid = 0; dut->m_wready = 1;
    for (int i = 0; i < 16; i++) {
        dut->eval();
        if (dut->m_wvalid && dut->m_wready) {
            Beat got = capture_out();
            if (model.empty()) fail("extra beat during drain", -1);
            else { if (got != model.front()) fail("drain beat mismatch", -1); model.pop_front(); }
            delivered++;
        }
        tick();
    }
    if (!model.empty())
        fail("beats accepted upstream never came out downstream (LOST DATA)", -1);

    // ---- 4: full throughput when nothing stalls -------------------------
    dut->rst = 1; dut->s_wvalid = 0; dut->m_wready = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    dut->m_wready = 1;
    dut->s_wvalid = 1;
    dut->s_wstrb = 0xFFFFu; dut->s_wlast = 0;
    long ups = 0, downs = 0;
    for (int i = 0; i < 1000; i++) {
        dut->s_wdata[0] = i;
        dut->eval();
        if (dut->s_wvalid && dut->s_wready) ups++;
        if (dut->m_wvalid && dut->m_wready) downs++;
        tick();
    }
    // One cycle of fill latency, otherwise one beat per cycle each way.
    if (ups < 999)
        fail("upstream accept rate below 1 beat/cycle with no downstream stall", -1);
    if (downs < 998)
        fail("downstream delivery rate below 1 beat/cycle with no stall", -1);

    printf("axi_w_skid: %ld beats accepted, %ld delivered under random pressure; "
           "steady-state %ld/%ld beats per 1000 cycles\n",
           accepted, delivered, ups, downs);
    printf("axi_w_skid: %s\n", failures ? "FAIL" : "PASS");
    delete dut;
    return failures ? 1 : 0;
}
