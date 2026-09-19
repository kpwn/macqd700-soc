// tb_reset_vectors.cpp — Verilator unit testbench for if_stage.v
//                         FETCH_RESET_VECTORS reset-vector-fetch FSM
//
// Three scenarios, each run twice — once for FETCH_RESET_VECTORS=1 (mode A)
// and once for FETCH_RESET_VECTORS=0 (mode B).  The per-mode build of
// Vif_stage is selected via the -GFETCH_RESET_VECTORS=<n> Verilator flag
// on two separate binaries: Vreset_vec_on and Vreset_vec_off.  A thin
// CLI arg ("on" / "off") is parsed from argv[1] so the Makefile can
// dispatch.
//
// Scenarios
//   mode-A.1: cold vector-0 fetch
//     Pre-populate a synthetic 128-bit line at addr 0x0 with
//     SSP=0xDEADBEEF, PC=0x12345678.  Confirm:
//       - if_req goes high at addr 0x0 after reset deassert
//       - vec_ssp_valid_o pulses one cycle when rvalid lands
//       - vec_ssp_o == 0xDEADBEEF on that cycle
//       - PC advances to 0x12345678 and normal fetch resumes
//
//   mode-A.2: vector-0 bus error
//     Mark the 0x0 line as faulted.  Confirm:
//       - vec_ssp_valid_o stays 0 (no spurious SSP write)
//       - pd_fault asserts so commit can dispatch bus-error vec 2
//
//   mode-B.1: legacy reset (FETCH_RESET_VECTORS=0)
//     No vector fetch; PC should start at RESET_PC (default
//     0x4080_0000) and the first fetch targets that PC's line.
//     Confirm:
//       - if_req goes high at addr 0x40800000 (PC's line)
//       - vec_ssp_valid_o stays 0 forever
//
// Build via make tb-reset-vectors.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <verilated.h>
#include "Vif_stage.h"

static Vif_stage* dut = nullptr;
static uint64_t   sim_time = 0;
static int        n_pass = 0, n_fail = 0;

// ─── Simple 128-bit line-fetch BFM ────────────────────────────────────
// Mirrors tb_if_stage.cpp's LineBus: serves big-endian 16-byte lines
// with a configurable response delay.  Sparse byte store; addresses
// not written read as 0x00.  Per-line fault flag for bus-error tests.
struct LineBus {
    std::map<uint32_t, uint8_t> store;
    std::map<uint32_t, bool>    fault_lines;
    int      n_reqs   = 0;
    bool     captured = false;
    uint32_t cap_addr = 0;
    int      delay    = 0;
    int      delay_cycles = 2;

    uint8_t byte_at(uint32_t a) {
        auto it = store.find(a);
        return (it == store.end()) ? 0 : it->second;
    }
    bool fault_at(uint32_t a) {
        auto it = fault_lines.find(a & ~0xFu);
        return it != fault_lines.end() && it->second;
    }
    void write_word(uint32_t a, uint32_t w) {
        // big-endian: a+0 = MSB
        store[a + 0] = (w >> 24) & 0xFF;
        store[a + 1] = (w >> 16) & 0xFF;
        store[a + 2] = (w >>  8) & 0xFF;
        store[a + 3] = (w >>  0) & 0xFF;
    }
    void clear() {
        store.clear();
        fault_lines.clear();
        n_reqs = 0;
        captured = false;
        delay = 0;
    }
};
static LineBus lbus;

// Latches: grab combinational outputs at the boundary where they pulse
// (before the posedge consumes them).  `vec_fired` is set whenever
// vec_ssp_valid_o is observed high during the pre-posedge eval — the
// natural pulse window — so the scenario code doesn't miss the 1-cycle
// pulse by checking at the wrong phase.
static bool     vec_fired  = false;
static uint32_t vec_fired_ssp = 0;
static int      vec_fired_count = 0;
static bool     pd_fault_seen = false;

static void drive_bus() {
    dut->if_rvalid = 0;
    dut->if_fault  = 0;
    for (int i = 0; i < 4; i++) dut->if_rdata.at(i) = 0;

    if (dut->if_req && !lbus.captured) {
        lbus.captured = true;
        lbus.cap_addr = (uint32_t)dut->if_addr & ~0xFu;
        lbus.delay    = lbus.delay_cycles;
        lbus.n_reqs++;
    }

    if (lbus.captured) {
        if (lbus.delay > 0) {
            lbus.delay--;
        } else {
            uint8_t line[16];
            for (int i = 0; i < 16; i++)
                line[i] = lbus.byte_at(lbus.cap_addr + (uint32_t)i);
            uint32_t w3 = ((uint32_t)line[0]  << 24) | ((uint32_t)line[1]  << 16)
                        | ((uint32_t)line[2]  <<  8) |  line[3];
            uint32_t w2 = ((uint32_t)line[4]  << 24) | ((uint32_t)line[5]  << 16)
                        | ((uint32_t)line[6]  <<  8) |  line[7];
            uint32_t w1 = ((uint32_t)line[8]  << 24) | ((uint32_t)line[9]  << 16)
                        | ((uint32_t)line[10] <<  8) |  line[11];
            uint32_t w0 = ((uint32_t)line[12] << 24) | ((uint32_t)line[13] << 16)
                        | ((uint32_t)line[14] <<  8) |  line[15];
            dut->if_rdata.at(0) = w0;
            dut->if_rdata.at(1) = w1;
            dut->if_rdata.at(2) = w2;
            dut->if_rdata.at(3) = w3;
            dut->if_rvalid      = 1;
            dut->if_fault       = lbus.fault_at(lbus.cap_addr) ? 1 : 0;
        }
    }
    dut->eval();
    // Snapshot the combinational outputs now: vec_ssp_valid_o is a
    // single-cycle pulse qualified on (vec_state==VEC_WAIT && if_rvalid
    // && if_req && !if_fault) and that predicate becomes false after
    // the next posedge advances vec_state to IDLE.  Sampling here is
    // the natural pre-posedge observation window.
    if (dut->vec_ssp_valid_o) {
        vec_fired     = true;
        vec_fired_ssp = (uint32_t)dut->vec_ssp_o;
        vec_fired_count++;
    }
    if (dut->pd_fault) pd_fault_seen = true;
}

static void tick() {
    drive_bus();
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    if (lbus.captured && lbus.delay == 0 && dut->if_rvalid) {
        lbus.captured = false;
    }
    sim_time++;
}

static void reset() {
    dut->rst           = 1;
    dut->pd_consumed   = 0;
    dut->br_redirect   = 0;
    dut->br_target     = 0;
    dut->pred_redirect = 0;
    dut->pred_target   = 0;
    dut->dbg_halt      = 0;
    dut->dbg_load_pc_en= 0;
    dut->dbg_load_pc_val = 0;
    dut->if_rvalid     = 0;
    dut->if_fault      = 0;
    for (int i = 0; i < 4; i++) dut->if_rdata.at(i) = 0;
    lbus.clear();
    vec_fired       = false;
    vec_fired_ssp   = 0;
    vec_fired_count = 0;
    pd_fault_seen   = false;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    // Do NOT tick yet — the caller wants to observe cycle 0 after
    // reset deassertion to watch if_req / vec_ssp_valid_o transitions.
}

#define CHECK_EQ(msg, got, exp) do { \
    if ((uint64_t)(got) != (uint64_t)(exp)) { \
        std::printf("    FAIL %s: got 0x%lx, expected 0x%lx\n", msg, \
                    (uint64_t)(got), (uint64_t)(exp)); \
        return false; \
    } \
} while (0)

#define CHECK_TRUE(msg, cond) do { \
    if (!(cond)) { std::printf("    FAIL %s\n", msg); return false; } \
} while (0)

// ─── Mode-A scenarios (FETCH_RESET_VECTORS=1) ─────────────────────────

// After reset, the CPU should issue a fetch at addr 0x0, latch SSP and
// PC from the returned line, and resume normal fetch at the loaded PC.
static bool test_cold_vector_fetch() {
    reset();
    // Seed the vector-0 line: 16 bytes at addr 0x0.
    //   bytes [0..3] = SSP = 0xDEADBEEF (big-endian)
    //   bytes [4..7] = PC  = 0x12345678 (big-endian)
    //   bytes [8..15] = don't care
    lbus.write_word(0x0, 0xDEADBEEFu);
    lbus.write_word(0x4, 0x12345678u);
    // Seed the first code line so normal fetch at PC=0x12345678 returns
    // data.  Line address = 0x12345670.  Contents don't need to decode;
    // tb doesn't consume.
    lbus.write_word(0x12345670, 0xA5A5A5A5u);
    lbus.write_word(0x12345674, 0xA5A5A5A6u);
    lbus.write_word(0x12345678, 0xA5A5A5A7u);
    lbus.write_word(0x1234567C, 0xA5A5A5A8u);

    // Cycle 1 post-reset: VEC_REQ → drive if_addr=0 / if_req=1.
    // The reset seq ticked for 8 cycles already.  After reset deassert
    // the DUT transitions to VEC_REQ on the very next posedge.
    tick();

    // Either this cycle or the next, if_req must be high with addr=0.
    int req_seen = 0;
    for (int i = 0; i < 4 && !req_seen; i++) {
        if (dut->if_req && (dut->if_addr == 0x0u)) req_seen = 1;
        else                                       tick();
    }
    CHECK_TRUE("if_req goes high at addr 0x0", req_seen);

    // Tick enough cycles for the line response to land and the DUT to
    // transition out of VEC_WAIT.  drive_bus() snapshots
    // vec_ssp_valid_o during the pre-posedge eval window.
    for (int i = 0; i < 30 && !vec_fired; i++) tick();
    CHECK_TRUE("vec_ssp_valid_o pulsed", vec_fired);
    CHECK_EQ("vec_ssp_o at pulse", vec_fired_ssp, 0xDEADBEEFu);

    // The pulse must be single-cycle: snapshot counter exactly 1.
    CHECK_EQ("vec_ssp_valid_o pulse is single-cycle",
             vec_fired_count, 1);

    // PC now drives normal fetch.  We asserted a new code line at
    // 0x12345670, so the DUT should shortly issue if_req at that line
    // address.
    int pc_fetch = 0;
    for (int i = 0; i < 20 && !pc_fetch; i++) {
        if (dut->if_req && ((uint32_t)dut->if_addr == 0x12345670u)) pc_fetch = 1;
        tick();
    }
    CHECK_TRUE("normal fetch resumes at loaded PC's line", pc_fetch);

    // Also confirm dbg_pc now equals the loaded PC.
    CHECK_EQ("dbg_pc == loaded PC", dut->dbg_pc, 0x12345678u);
    return true;
}

// Bus error on the vector-0 line.  vec_ssp_valid_o must NOT fire; pd_fault
// should assert at pc=0 so the core's commit path dispatches the bus-
// error exception.
static bool test_vector_bus_error() {
    reset();
    lbus.fault_lines[0x0] = true;

    // Tick past the VEC_REQ → VEC_WAIT transition and let the faulted
    // response land.
    tick();
    for (int i = 0; i < 30 && !pd_fault_seen; i++) tick();
    CHECK_TRUE("pd_fault asserts on faulted vector fetch", pd_fault_seen);
    CHECK_EQ("pd_pc remains at 0 during fault", dut->dbg_pc, 0x0u);
    // vec_ssp_valid_o must never have pulsed — we don't publish an
    // SSP on a faulted vector read.
    CHECK_TRUE("vec_ssp_valid_o never pulsed on fault", !vec_fired);
    return true;
}

// ─── Mode-B scenarios (FETCH_RESET_VECTORS=0) ─────────────────────────

// Legacy boot: no vector fetch.  PC starts at the RESET_PC parameter
// (default 0x4080_0000); the first if_req hits that line.  vec_ssp_valid_o
// stays tied to 0 forever.
static bool test_legacy_reset() {
    reset();
    // Seed something at line 0x40800000 so the prefetcher gets a clean
    // response instead of fault-by-unmapped.
    lbus.write_word(0x40800000u, 0x11223344u);
    lbus.write_word(0x40800004u, 0x55667788u);
    lbus.write_word(0x40800008u, 0x99AABBCCu);
    lbus.write_word(0x4080000Cu, 0xDDEEFF00u);

    // Expect if_req at 0x40800000 — NOT at 0x0.
    int req_seen = 0;
    int req_at_zero = 0;
    for (int i = 0; i < 10; i++) {
        if (dut->if_req) {
            if ((uint32_t)dut->if_addr == 0x40800000u) req_seen = 1;
            if ((uint32_t)dut->if_addr == 0x0u)        req_at_zero = 1;
        }
        tick();
    }
    CHECK_TRUE("if_req lands at RESET_PC line 0x40800000", req_seen);
    CHECK_TRUE("no vector-0 fetch at addr 0x0", !req_at_zero);

    // Let the response come back.
    for (int i = 0; i < 10; i++) tick();

    // vec_ssp_valid_o must be wired to constant-0 (FETCH_RESET_VECTORS=0
    // means the VEC_* state never leaves IDLE, so the predicate driving
    // vec_ssp_valid_o is always false).  vec_fired is the cumulative
    // snapshot across every drive_bus() eval — it should never have
    // flipped.
    CHECK_TRUE("vec_ssp_valid_o never pulsed in legacy mode",
               !vec_fired);
    return true;
}

#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { std::printf("[PASS] %s\n", #fn); n_pass++; } \
    else    { std::printf("[FAIL] %s\n", #fn); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vif_stage;

    std::string mode = (argc > 1) ? argv[1] : "on";

    if (mode == "on") {
        std::printf("=== FETCH_RESET_VECTORS=1 (mode A) ===\n");
        RUN(test_cold_vector_fetch);
        RUN(test_vector_bus_error);
    } else {
        std::printf("=== FETCH_RESET_VECTORS=0 (mode B) ===\n");
        RUN(test_legacy_reset);
    }

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
