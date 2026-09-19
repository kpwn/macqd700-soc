// tb_ifetch_window_guard.cpp -- the fetch-side window guard must answer
// out-of-window fetches with a local DECERR burst and pass in-window
// fetches through untouched.  Includes a POSITIVE CONTROL (an in-window
// burst must NOT be faulted) so a guard that faults everything cannot
// pass.  Motivating hardware failure: p164 executed DRAM filler fetched
// from NuBus slot-$B physical space (0xFB0493AA).
#include <cstdio>
#include <cstdint>
#include <array>
#include <vector>
#include <verilated.h>
#include "Vifetch_window_guard.h"

static Vifetch_window_guard *dut;
static vluint64_t t = 0;
double sc_time_stamp() { return t; }
static int npass = 0, nfail = 0;
static void ok(const char *w, bool c) { std::printf("  [%s] %s\n", c ? "PASS" : "FAIL", w); (c ? npass : nfail)++; }
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }

// A tiny downstream model: accepts AR when told, then returns len+1 OKAY beats
// with rdata[31:0] = addr + beat, id echoed.
// `latency` is the AR-accept -> first-R-beat delay.  It defaults to 0, which
// is what this model used to be -- and that zero is why the m_burst_open
// defect (scenario "no beats yet") was invisible here for so long: with a
// downstream that answers in the same cycle, the window between "AR accepted
// downstream" and "first beat observed" does not exist.  Real downstream is
// l2c + DDR, documented at 40-200 core clocks.
struct Down { bool have=false; uint8_t id=0, len=0, beat=0; uint32_t addr=0; bool accept=true;
              int latency=0; int lat_left=0; } dn;
static void down_step() {
    dut->m_arready = (!dn.have && dn.accept) ? 1 : 0;
    if (dut->m_arvalid && dut->m_arready) { dn.have = true; dn.id = dut->m_arid; dn.len = dut->m_arlen; dn.beat = 0; dn.addr = dut->m_araddr; dn.lat_left = dn.latency; }
    bool r_ready_to_drive = dn.have && dn.lat_left == 0;
    dut->m_rvalid = r_ready_to_drive ? 1 : 0;
    dut->m_rid = dn.id; dut->m_rresp = 0; dut->m_rlast = (r_ready_to_drive && dn.beat == dn.len) ? 1 : 0;
    dut->m_rdata[0] = dn.addr + dn.beat;
}
static void down_advance() {
    if (dn.have && dn.lat_left > 0) { dn.lat_left--; return; }
    if (dut->m_rvalid && dut->m_rready) { if (dn.beat == dn.len) dn.have = false; else dn.beat++; }
}

// Issue one AR and collect the response beats (with optional rready stalls).
struct Rsp { int beats=0; int decerr=0; int okay=0; bool last_ok=false; uint8_t id=0; bool forwarded=false; };
static Rsp run(uint32_t addr, uint8_t len, uint8_t id, bool is_rom, bool stall) {
    Rsp r; dut->s_araddr = addr; dut->s_arlen = len; dut->s_arid = id; dut->s_ar_is_rom = is_rom; dut->s_arvalid = 1;
    bool taken = false; int cyc = 0;
    while (!taken && cyc < 200) { down_step(); dut->eval(); if (dut->m_arvalid) r.forwarded = true; if (dut->s_arready) taken = true; tick(); down_advance(); cyc++; }
    dut->s_arvalid = 0;
    if (!taken) { std::printf("  AR never accepted for 0x%08x\n", addr); return r; }
    for (cyc = 0; cyc < 400 && !r.last_ok; cyc++) {
        dut->s_rready = stall ? ((cyc % 3) == 0) : 1;
        down_step(); dut->eval();
        if (dut->s_rvalid && dut->s_rready) {
            r.beats++; r.id = dut->s_rid;
            if (dut->s_rresp == 3) r.decerr++; else if (dut->s_rresp == 0) r.okay++;
            if (dut->s_rlast) r.last_ok = true;
        }
        tick(); down_advance();
    }
    dut->s_rready = 0; return r;
}

// Boundary matrix independent of the older single-request downstream model.
// All stimulus/response accounting is sampled BEFORE the active clock edge.
static void response_boundary(int mode) {
    std::printf("[RUN ] response arbitration boundary mode %d\n", mode);
    dut->rst = 1; dut->s_arvalid = 0; dut->s_rready = 0;
    dut->m_rvalid = 0; dut->m_arready = 1;
    tick(); dut->rst = 0; tick();
    struct Beat { unsigned id, resp, last, data; };
    std::vector<Beat> got;
    bool held = false, stable = true;
    Beat previous{};
    std::array<uint32_t, 8> previous_data{};
    auto cycle = [&]() {
        dut->eval();
        Beat b{dut->s_rid, dut->s_rresp, dut->s_rlast, dut->s_rdata[0]};
        if (held) {
            stable &= dut->s_rvalid && b.id == previous.id &&
                      b.resp == previous.resp && b.last == previous.last;
            for (int i = 0; i < 8; i++) stable &= dut->s_rdata[i] == previous_data[i];
        }
        held = dut->s_rvalid && !dut->s_rready;
        previous = b;
        for (int i = 0; i < 8; i++) previous_data[i] = dut->s_rdata[i];
        if (dut->s_rvalid && dut->s_rready) got.push_back(b);
        tick();
    };
    auto ar = [&](unsigned id, unsigned addr) {
        dut->s_arid = id; dut->s_araddr = addr; dut->s_arlen = 1;
        dut->s_ar_is_rom = 0; dut->s_arvalid = 1; dut->eval();
        ok("boundary AR accepted", dut->s_arready);
        cycle(); dut->s_arvalid = 0;
    };
    ar(3, 0x1000);
    if (mode == 2) ar(4, 0x2000);
    dut->m_rvalid = 1; dut->m_rid = 3; dut->m_rresp = 0;
    dut->m_rlast = 0; dut->m_rdata[0] = 0x12345678;
    for (int i = 1; i < 8; i++) dut->m_rdata[i] = 0x10203040u + i;
    if (mode == 2) {
        dut->s_rready = 1; cycle();
        dut->m_rlast = 1; dut->m_rdata[0] = 0x87654321;
    } else {
        dut->s_rready = mode == 1;
    }
    // mode 0: first beat stalled; mode 1: first beat handshakes;
    // mode 2: previous burst's LAST handshakes while fault AR is accepted.
    dut->s_arid = 12; dut->s_araddr = 0xFB000000;
    dut->s_arlen = 0; dut->s_arvalid = 1; dut->eval();
    ok("boundary fault AR accepted", dut->s_arready);
    cycle(); dut->s_arvalid = 0;
    if (mode == 1) { dut->m_rlast = 1; dut->m_rdata[0] = 0x87654321; }
    if (mode == 2) {
        dut->m_rid = 4; dut->m_rlast = 0; dut->m_rdata[0] = 0x12345678;
    }
    dut->s_rready = 0; cycle(); cycle();
    dut->s_rready = 1;
    // Downstream advances ONLY on its own handshake, even on broken RTL.
    for (int guard = 0; guard < 30 && dut->m_rvalid; guard++) {
        dut->eval(); const bool accepted = dut->m_rready;
        const bool last = dut->m_rlast;
        cycle();
        if (accepted) {
            if (last) dut->m_rvalid = 0;
            else { dut->m_rlast = 1; dut->m_rdata[0] = 0x87654321; }
        }
    }
    for (int i = 0; i < 10; i++) cycle();
    ok("boundary stalled payload is stable", stable);
    const unsigned n = mode == 2 ? 4 : 2;
    bool sequence = got.size() == n + 1;
    for (unsigned i = 0; i < got.size() && i < n; i++)
        sequence &= got[i].id == (i < 2 ? 3u : 4u) && got[i].resp == 0 &&
                    got[i].last == (i & 1) &&
                    got[i].data == ((i & 1) ? 0x87654321u : 0x12345678u);
    if (got.size() == n + 1)
        sequence &= got[n].id == 12 && got[n].resp == 3 && got[n].last && got[n].data == 0;
    ok("boundary drains older bursts before exactly one fault response", sequence);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv); dut = new Vifetch_window_guard;
    dut->rst = 1; dut->clk = 0; dut->ram_window_lg2 = 26; /* 64 MiB visible */ dut->s_arvalid = 0; dut->s_rready = 0;
    for (int i = 0; i < 4; i++) tick(); dut->rst = 0; tick();

    std::printf("[RUN ] POSITIVE CONTROL: in-window RAM fetch passes through with OKAY data\n");
    { Rsp r = run(0x03FFFFC0, 1, 5, false, false);
      ok("forwarded to l2c", r.forwarded); ok("2 beats", r.beats == 2); ok("all OKAY", r.okay == 2 && r.decerr == 0); ok("id echoed", r.id == 5); ok("rlast seen", r.last_ok); }

    std::printf("[RUN ] ROM mirror fetch (upstream decode says ROM) passes through\n");
    { Rsp r = run(0x40800000, 1, 6, true, false); ok("forwarded", r.forwarded); ok("OKAY beats", r.okay == 2 && r.decerr == 0); }

    std::printf("[RUN ] slot-$B physical fetch 0xFB0493AA (the p164 crash) -> local DECERR burst\n");
    { Rsp r = run(0xFB0493A0, 1, 9, false, false);
      ok("NOT forwarded to l2c", !r.forwarded); ok("2 beats", r.beats == 2); ok("all DECERR", r.decerr == 2 && r.okay == 0); ok("id echoed", r.id == 9); ok("rlast on last beat", r.last_ok);
      ok("fault sticky + addr captured", dut->fault_sticky && dut->fault_addr == 0xFB0493A0); ok("fault_count == 1", dut->fault_count == 1); }

    std::printf("[RUN ] just past the visible RAM window (0x04000000) faults; just inside (0x03FFFFC0) does not\n");
    { Rsp a = run(0x04000000, 0, 2, false, false); ok("0x04000000 -> DECERR", !a.forwarded && a.decerr == 1);
      Rsp b = run(0x03FFFFC0, 0, 3, false, false); ok("0x03FFFFC0 -> OKAY", b.forwarded && b.okay == 1); }

    std::printf("[RUN ] RAM-probe alias 0x58000000 and IO 0x50000000 fault (no code lives there)\n");
    { Rsp a = run(0x58000000, 0, 4, false, false); ok("alias -> DECERR", !a.forwarded && a.decerr == 1);
      Rsp b = run(0x50000000, 0, 4, false, false); ok("io -> DECERR", !b.forwarded && b.decerr == 1); }

    std::printf("[RUN ] back-pressure: DECERR burst with stalled rready keeps exact beat count\n");
    { Rsp r = run(0xFE000000, 3, 7, false, true); ok("4 beats", r.beats == 4); ok("all DECERR", r.decerr == 4); ok("rlast on last", r.last_ok); }

    std::printf("[RUN ] a faulting AR while a downstream burst is OPEN waits for its rlast (no interleave)\n");
    { // start an in-window 4-beat burst downstream but consume only 1 beat, then issue a faulting AR
      dut->s_araddr = 0x00001000; dut->s_arlen = 3; dut->s_arid = 1; dut->s_ar_is_rom = 0; dut->s_arvalid = 1;
      bool taken=false; for (int c=0;c<50&&!taken;c++){ down_step(); dut->eval(); if (dut->s_arready) taken=true; tick(); down_advance(); }
      dut->s_arvalid = 0; ok("in-window AR accepted", taken);
      dut->s_rready = 1; int got=0; for (int c=0;c<20&&got<1;c++){ down_step(); dut->eval(); if (dut->s_rvalid&&dut->s_rready) got++; tick(); down_advance(); }
      dut->s_rready = 0; ok("consumed 1 of 4 downstream beats", got==1);
      dut->s_araddr = 0xFB000000; dut->s_arlen = 0; dut->s_arid = 12; dut->s_arvalid = 1;
      taken=false; for (int c=0;c<50&&!taken;c++){ down_step(); dut->eval(); if (dut->s_arready) taken=true; tick(); down_advance(); }
      dut->s_arvalid = 0; ok("faulting AR accepted by the guard", taken);
      // now drain: the next 3 beats must be the downstream OKAY beats (ids 1), THEN the DECERR (id 12)
      int seq_ok=1, n=0; uint8_t ids[8]={0}; uint8_t resps[8]={0}; dut->s_rready = 1;
      for (int c=0;c<80&&n<4;c++){ down_step(); dut->eval(); if (dut->s_rvalid&&dut->s_rready){ ids[n]=dut->s_rid; resps[n]=dut->s_rresp; n++; } tick(); down_advance(); }
      dut->s_rready = 0;
      seq_ok = (n==4) && ids[0]==1 && ids[1]==1 && ids[2]==1 && resps[0]==0 && resps[1]==0 && resps[2]==0 && ids[3]==12 && resps[3]==3;
      ok("downstream burst finished first, then the DECERR (no interleave)", seq_ok); }

    std::printf("[RUN ] fault acceptance must not replace a stalled first downstream beat\n");
    {
      // Drive both channels explicitly, sampling handshakes before the edge.
      // An older valid fetch has been accepted, but its first R beat has
      // not handshaken yet: m_burst_open is still false.
      dut->rst = 1; dut->s_arvalid = 0; dut->m_rvalid = 0;
      dut->s_rready = 0; tick(); dut->rst = 0; tick();
      dut->s_araddr = 0x1000; dut->s_arlen = 1; dut->s_arid = 3;
      dut->s_ar_is_rom = 0; dut->s_arvalid = 1; dut->m_arready = 1;
      dut->eval(); ok("older valid AR accepted", dut->s_arready && dut->m_arvalid);
      tick(); dut->s_arvalid = 0;
      dut->m_rvalid = 1; dut->m_rid = 3; dut->m_rresp = 0;
      dut->m_rlast = 0; dut->m_rdata[0] = 0x12345678;
      dut->eval();
      ok("first downstream beat presented while stalled",
         dut->s_rvalid && !dut->s_rready && dut->s_rid == 3);
      tick();
      dut->s_araddr = 0xFB000000; dut->s_arlen = 0;
      dut->s_arid = 12; dut->s_arvalid = 1;
      dut->eval(); ok("concurrent fault AR accepted", dut->s_arready);
      tick(); dut->s_arvalid = 0; dut->eval();
      ok("stalled RVALID/RID/RRESP/RLAST/data remain unchanged after fault AR",
         dut->s_rvalid && dut->s_rid == 3 && dut->s_rresp == 0 &&
         !dut->s_rlast && dut->s_rdata[0] == 0x12345678);
      // Drain the older burst, then expect exactly the local fault response.
      dut->s_rready = 1; dut->eval();
      ok("first older beat is accepted downstream", dut->m_rready);
      tick(); dut->m_rlast = 1; dut->m_rdata[0] = 0x87654321; dut->eval();
      ok("second older beat remains associated with RID 3",
         dut->s_rvalid && dut->m_rready && dut->s_rid == 3 &&
         dut->s_rresp == 0 && dut->s_rlast && dut->s_rdata[0] == 0x87654321);
      tick(); dut->m_rvalid = 0; dut->eval();
      bool fault_seen = false;
      for (int i = 0; i < 8; i++) {
        dut->eval();
        if (dut->s_rvalid && dut->s_rready)
          fault_seen |= dut->s_rid == 12 && dut->s_rresp == 3 && dut->s_rlast;
        tick();
      }
      ok("fault response follows the older burst", fault_seen);
    }

    for (int mode = 0; mode < 3; mode++) response_boundary(mode);
    // ── m_burst_open is a BEAT-OBSERVATION flag, not a record of outstanding-ness ──
    // Between AR acceptance downstream and the FIRST returned beat -- the whole
    // l2c miss + DDR round trip -- m_burst_open reads 0, so the guard believes
    // the R channel is free and goes straight to G_EMIT.  Its local DECERR burst
    // then PRECEDES the in-flight fetch's data (and holds m_rready low while it
    // does).  The header's claim that a DECERR burst "never interleaves inside an
    // open downstream burst" is true for interleave and false for precede.
    //
    // Reachability is asserted by this repo's own RTL: l2c_mshr.v and l2c.v both
    // record that the core this SoC hosts is multi-outstanding on its I side, and
    // axi_i binds straight through this guard in the shipping CPU_M68K040 build.
    std::printf("[RUN ] a faulting AR while a downstream fetch is OUTSTANDING BUT HAS NO BEATS YET must still wait\n");
    { dn = Down{}; dn.latency = 12;
      dut->s_araddr = 0x00002000; dut->s_arlen = 3; dut->s_arid = 1; dut->s_ar_is_rom = 0; dut->s_arvalid = 1;
      bool taken=false; for (int c=0;c<50&&!taken;c++){ down_step(); dut->eval(); if (dut->s_arready) taken=true; tick(); down_advance(); }
      dut->s_arvalid = 0; ok("in-window AR accepted downstream", taken);
      // Positive control: it really has produced nothing yet.
      down_step(); dut->eval();
      ok("[positive control] no beat has come back yet", dut->m_rvalid == 0);

      dut->s_araddr = 0xFB000000; dut->s_arlen = 0; dut->s_arid = 13; dut->s_arvalid = 1;
      taken=false; for (int c=0;c<50&&!taken;c++){ down_step(); dut->eval(); if (dut->s_arready) taken=true; tick(); down_advance(); }
      dut->s_arvalid = 0; ok("faulting AR accepted by the guard", taken);

      int n=0; uint8_t ids[8]={0}; uint8_t resps[8]={0}; dut->s_rready = 1;
      for (int c=0;c<200&&n<5;c++){ down_step(); dut->eval(); if (dut->s_rvalid&&dut->s_rready){ ids[n]=dut->s_rid; resps[n]=dut->s_rresp; n++; } tick(); down_advance(); }
      dut->s_rready = 0;
      bool seq_ok = (n==5) && ids[0]==1 && ids[1]==1 && ids[2]==1 && ids[3]==1 &&
                    resps[0]==0 && resps[1]==0 && resps[2]==0 && resps[3]==0 &&
                    ids[4]==13 && resps[4]==3;
      if (!seq_ok) { std::printf("    got %d beats:", n); for (int i=0;i<n;i++) std::printf(" id=%u/resp=%u", ids[i], resps[i]); std::printf("\n"); }
      ok("the outstanding fetch's 4 OKAY beats come FIRST, then the DECERR", seq_ok);
      dn = Down{}; }

    // ── s_arready's out-of-window arm has no rst term ──────────────────────────
    // `s_arready = g_busy ? 0 : (in_window ? m_arready : 1'b1)` -- the 1'b1 arm is
    // unconditional, while the g_accept capture lives inside the `else` of
    // `if (rst)`.  The upstream is the CPU axi_i master on cpu_rst, a DIFFERENT
    // net from this guard's core_rst, so an out-of-window AR can genuinely be
    // presented while the guard is held: it handshakes, nothing is recorded, no
    // DECERR is ever emitted, and the I-cache's refill slot for that id never
    // clears.
    std::printf("[RUN ] an out-of-window AR presented while the guard is in RESET must not be swallowed\n");
    { dn = Down{};
      dut->rst = 1; dut->s_rready = 0;
      dut->s_araddr = 0xFB001000; dut->s_arlen = 0; dut->s_arid = 14; dut->s_ar_is_rom = 0; dut->s_arvalid = 1;
      bool handshaked = false;
      for (int c=0;c<8;c++){ down_step(); dut->eval(); if (dut->s_arready) handshaked = true; tick(); down_advance(); }
      ok("the guard refuses the AR while it is reset", !handshaked);
      dut->rst = 0;
      // and once out of reset the very same AR is answered properly
      bool taken=false; for (int c=0;c<50&&!taken;c++){ down_step(); dut->eval(); if (dut->s_arready) taken=true; tick(); down_advance(); }
      dut->s_arvalid = 0;
      ok("[positive control] and it is accepted once reset lifts", taken);
      int n=0, dec=0; dut->s_rready = 1;
      for (int c=0;c<80&&n<1;c++){ down_step(); dut->eval(); if (dut->s_rvalid&&dut->s_rready){ n++; if (dut->s_rresp==3) dec++; } tick(); down_advance(); }
      dut->s_rready = 0;
      ok("and answered with a DECERR", n==1 && dec==1);
      dn = Down{}; }

    std::printf("\n%d passed, %d failed.\n", npass, nfail);
    delete dut; return nfail ? 1 : 0;
}
