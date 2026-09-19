// tb_scsi_c96_nondma_trailing.cpp — the non-DMA DATA IN trailing-transfer
// completion contract (2026-09-06 boot-wedge fix, owner-approved REAL-CHIP
// semantics over MAME fidelity — see the divergence box at the completion
// hook in rtl/mac/scsi.v and docs/scsi_fuzz.md "Deliberate real-chip
// divergences").
//
// Contract under test — a non-DMA CI_XFER (0x10) in DATA IN:
//   1. moves exactly ONE byte and completes (I_BUS + disarm), REGARDLESS
//      of prior FIFO residue.  MAME instead free-runs a residue-armed
//      receive until the FIFO fills, with no interrupt — the behaviour
//      that turned a stolen istatus read into the permanent 7.5.3 boot
//      wedge at ROM 0x40899704 (silicon, p143 trace ring).
//   2. with NOTHING to move (backing supply permanently drained, provider
//      idle) it must STILL complete with I_BUS — the trailing transfer.
//      The real chip interrupts on the trailing transfer; without this
//      the command parks with no interrupt and only a JTAG bus reset
//      recovers the machine (silicon, JTAG round 2: xfr_armed=1,
//      xfr_dma=0, phase=DATA_IN, fifo_pos=0, nondma_supply=0).
//
// Three tests, each with its checks REQUIRED to execute (a stage that
// aborts early fails the run — no silently-dead assertions):
//   T1 healthy per-byte loop        (existing contract, regression guard)
//   T2 residue-armed 0x10           (park shape 1: one byte + I_BUS, no
//                                    free-run, byte order preserved)
//   T3 supply-exhaustion trailing   (park shape 2: mid-stream bus reset +
//                                    re-select desyncs the provider kick;
//                                    every trailing 0x10 completes)
//
// Build via:   make tb-scsi-c96-nondma-trailing

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"

static Vtb_scsi_vhdd_sd* dut = nullptr;
static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, ...) do {                                   \
        if (cond) { ++n_pass; }                                 \
        else { ++n_fail; printf("FAIL %s:%d: ", __func__, __LINE__); \
               printf(__VA_ARGS__); printf("\n"); }             \
    } while (0)

// ─── SD backing-store mock (read path only; mirrors tb_scsi_c96_read6) ──
struct SdMock {
    std::vector<uint8_t> read_sector;   // byte pool (repeats mod size)
    int gap = 0;                        // cycles between rd_valid pulses
    int skid_max = 2;
    int skid = 2;
    enum class State { Idle, Reading, Done } st = State::Idle;
    int cnt = 0, total = 512, delay = 0, gap_ctr = 0;
} sd_mock;

static void sd_mock_tick() {
    dut->sd_busy     = (sd_mock.st != SdMock::State::Idle) ? 1 : 0;
    dut->sd_done     = 0;
    dut->sd_error    = 0;
    dut->sd_rd_valid = 0;
    dut->sd_rd_data  = 0;
    dut->sd_wr_ready = 0;

    if (sd_mock.st == SdMock::State::Idle && dut->sd_go) {
        sd_mock.cnt     = 0;
        sd_mock.delay   = 2;
        sd_mock.gap_ctr = 0;
        if (dut->sd_cmd_type == 1) {                    // CMD17
            sd_mock.st = SdMock::State::Reading;
            sd_mock.total = 512;
        } else if (dut->sd_cmd_type == 2) {             // CMD18
            sd_mock.st = SdMock::State::Reading;
            sd_mock.total = 512 * (int)dut->sd_block_count;
        } else {
            sd_mock.st = SdMock::State::Done;
        }
        dut->sd_busy = 1;
        return;
    }
    if (sd_mock.st == SdMock::State::Reading) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0)   { --sd_mock.delay;   return; }
        if (sd_mock.gap_ctr > 0) { --sd_mock.gap_ctr; return; }
        if (dut->sd_rd_ready)       sd_mock.skid = sd_mock.skid_max;
        else if (sd_mock.skid > 0)  --sd_mock.skid;
        else                        return;
        if (sd_mock.cnt < sd_mock.total) {
            dut->sd_rd_valid = 1;
            dut->sd_rd_data  = sd_mock.read_sector.empty()
                ? 0
                : sd_mock.read_sector[sd_mock.cnt % sd_mock.read_sector.size()];
            ++sd_mock.cnt;
            sd_mock.gap_ctr = sd_mock.gap;
            if (sd_mock.cnt == sd_mock.total) sd_mock.st = SdMock::State::Done;
        }
    } else if (sd_mock.st == SdMock::State::Done) {
        dut->sd_busy = 0;
        dut->sd_done = 1;
        sd_mock.st = SdMock::State::Idle;
    }
}

static void tick() {
    sd_mock_tick();
    dut->clk = 1; dut->eval();
    dut->clk = 0; dut->eval();
}
static void tick_n(int n) { for (int i = 0; i < n; ++i) tick(); }

static void reset() {
    dut->rst = 1;
    dut->disk_num_lbas = 1048576u;
    dut->pb_addr = 0; dut->pb_wdata = 0; dut->pb_wr = 0; dut->pb_rd = 0;
    dut->pb_dma16_lo_beat = 0;
    dut->scsi_ctrl_in = 0;
    sd_mock = SdMock();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)((i * 13 + 7) & 0xff);
    for (int i = 0; i < 4; ++i) tick();
    dut->rst = 0;
    tick();
}

static uint8_t reg_r(uint8_t off) {
    dut->pb_addr = off & 0xf;
    dut->pb_rd = 1; dut->pb_wr = 0;
    tick();
    dut->pb_rd = 0;
    dut->eval();
    return dut->pb_rdata & 0xff;
}
static void reg_w(uint8_t off, uint8_t v) {
    dut->pb_addr = off & 0xf;
    dut->pb_wdata = v;
    dut->pb_wr = 1; dut->pb_rd = 0;
    tick();
    dut->pb_wr = 0; dut->pb_wdata = 0;
    tick();                    // one idle cycle: command dispatch settles
}

// Wait for INT; returns cycles waited, or -1 on timeout.
static int wait_irq(int budget) {
    for (int i = 0; i < budget; ++i) {
        if (dut->irq) return i;
        tick();
    }
    return -1;
}

// Chip + bus normalization (the scsi_fuzz preamble, minus the fuzz CTRL op).
static void normalize() {
    reg_w(3, 0x02);            // CM_RESET (chip)
    reg_w(8, 0x07);
    reg_w(3, 0x03);            // CM_RESET_BUS
    tick_n(200);
    (void)reg_r(5);            // drain I_SCSI_RESET
    tick_n(50);
    (void)reg_r(5);
    reg_w(3, 0x01);            // CM_FLUSH_FIFO
    reg_w(4, 0x06);            // dest bus ID = TARGET_ID (6)
    reg_w(5, 0x01);            // select timeout
    reg_w(6, 0x05);
    reg_w(7, 0x00);
    reg_w(9, 0x02);
    reg_w(0, 0x00);
    reg_w(1, 0x00);
    tick_n(50);
}

// Bare-CDB select (CD_SELECT 0x41).  Returns true on select-complete.
static bool select_cdb(const uint8_t* cdb, int len) {
    for (int i = 0; i < len; ++i) reg_w(2, cdb[i]);
    reg_w(3, 0x41);
    if (wait_irq(200000) < 0) return false;
    uint8_t ist = reg_r(5);            // I_FUNCTION | I_BUS, retires 0x41
    tick_n(10);
    return ist == 0x18;
}

// ─── T1: healthy per-byte loop ──────────────────────────────────────────
static bool t1_ran = false;
static void test_healthy_perbyte() {
    t1_ran = true;
    normalize();
    static const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    CHECK(select_cdb(cdb, 6), "READ(6) select did not complete");
    for (int i = 0; i < 8; ++i) {
        reg_w(3, 0x10);                       // non-DMA CI_XFER
        int w = wait_irq(200000);
        CHECK(w >= 0, "iter %d: no INT for per-byte 0x10", i);
        if (w < 0) return;
        CHECK(reg_r(7) == 1, "iter %d: fifo flags != 1", i);
        uint8_t b   = reg_r(2);
        uint8_t exp = sd_mock.read_sector[i % sd_mock.read_sector.size()];
        CHECK(b == exp, "iter %d: byte %02x != expected %02x", i, b, exp);
        uint8_t ist = reg_r(5);
        CHECK(ist == 0x10, "iter %d: istat %02x != I_BUS", i, ist);
        tick_n(10);
    }
}

// ─── T2: residue-armed 0x10 — one byte + I_BUS, no free-run ─────────────
static bool t2_ran = false;
static void test_residue_one_byte() {
    t2_ran = true;
    normalize();
    static const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    CHECK(select_cdb(cdb, 6), "READ(6) select did not complete");

    // Two healthy iterations (bytes 0 and 1 popped).
    for (int i = 0; i < 2; ++i) {
        reg_w(3, 0x10);
        CHECK(wait_irq(200000) >= 0, "warmup iter %d: no INT", i);
        (void)reg_r(2);
        (void)reg_r(5);
        tick_n(10);
    }

    // The stolen completion: arm, take the INT, read istatus — but skip
    // the FIFO pop (the ROM's error path at 0x40899004).  One byte of
    // residue is left staged.
    reg_w(3, 0x10);
    CHECK(wait_irq(200000) >= 0, "steal iter: no INT");
    (void)reg_r(5);
    tick_n(10);
    CHECK(reg_r(7) == 1, "steal iter: expected 1 byte of residue");

    // The retry the wedge hinged on: 0x10 armed WITH residue.
    // Real chip: one byte in behind the residue, I_BUS, disarm.
    // (MAME free-runs to fifo=16 with no interrupt — the boot wedge.)
    reg_w(3, 0x10);
    int w = wait_irq(200000);
    CHECK(w >= 0, "residue-armed 0x10 never completed (the Round-1 park)");
    if (w < 0) return;
    uint8_t flags = reg_r(7);
    CHECK(flags == 2, "residue arm moved %d bytes, expected exactly 1 "
                      "(fifo=residue+1=2); 16 = the MAME free-run", flags);
    uint8_t b2 = reg_r(2);
    uint8_t b3 = reg_r(2);
    CHECK(b2 == sd_mock.read_sector[2], "head pop %02x != skipped byte %02x",
          b2, sd_mock.read_sector[2]);
    CHECK(b3 == sd_mock.read_sector[3], "next pop %02x != next byte %02x",
          b3, sd_mock.read_sector[3]);
    uint8_t ist = reg_r(5);
    CHECK(ist == 0x10, "residue arm istat %02x != I_BUS", ist);
}

// ─── T3: supply-exhaustion trailing completion ──────────────────────────
// Mid-stream CM_RESET_BUS + re-select leaves the new transfer without a
// provider kick (vh_kicked=0): the stale ring drains, then NOTHING can
// arrive — the silicon Round-2 park.  Every trailing 0x10 must still
// complete with I_BUS, retiring through the istatus read.
static bool t3_ran = false;
static void test_supply_exhaustion_trailing() {
    t3_ran = true;
    normalize();
    sd_mock.gap = 60;                    // slow provider: reset lands mid-stream
    static const uint8_t cdb10[10] =
        {0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00};
    CHECK(select_cdb(cdb10, 10), "first READ(10) select did not complete");

    // Two healthy per-byte iterations to get into DATA IN.
    for (int i = 0; i < 2; ++i) {
        reg_w(3, 0x10);
        CHECK(wait_irq(400000) >= 0, "pre-reset iter %d: no INT", i);
        (void)reg_r(2);
        (void)reg_r(5);
        tick_n(10);
    }

    // Mid-stream SCSI bus reset (the JTAG intervention / an OS retry).
    reg_w(3, 0x03);
    tick_n(200);
    uint8_t ist = reg_r(5);
    CHECK(ist == 0x80, "bus reset istat %02x != I_SCSI_RESET", ist);
    tick_n(50);

    // Re-select and re-issue the READ(10) while the old stream is live.
    reg_w(3, 0x01);                      // flush FIFO
    CHECK(select_cdb(cdb10, 10), "second READ(10) select did not complete");

    // Per-byte drain.  The stale stream serves ~a block's worth, then the
    // supply is gone for good.  On the pre-fix RTL iteration ~525 parks
    // forever (no INT, queue jams at 2, S_GROSS_ERROR on the next write).
    int trailing = 0;                    // completions with an empty FIFO
    bool parked = false;
    for (int i = 0; i < 1100; ++i) {
        reg_w(3, 0x10);
        int w = wait_irq(400000);
        if (w < 0) {
            parked = true;
            CHECK(false, "iter %d: 0x10 never completed — the Round-2 "
                         "supply-exhaustion park (istat=%02x flags=%d "
                         "stat=%02x)", i, reg_r(5), reg_r(7), reg_r(4));
            break;
        }
        uint8_t flags = reg_r(7);
        if (flags == 0) ++trailing;      // completed with nothing to move
        else            (void)reg_r(2);
        uint8_t is2 = reg_r(5);
        CHECK(is2 == 0x10, "iter %d: istat %02x != I_BUS", i, is2);
        if (is2 != 0x10) { parked = true; break; }
        tick_n(4);
    }
    // The desync must actually have produced trailing transfers, or this
    // test proved nothing (guards the recipe itself).
    CHECK(trailing > 0, "recipe never reached the trailing-transfer state "
                        "(supply never exhausted) — test is vacuous");
    CHECK(!parked, "per-byte loop parked instead of completing");
    // The queue must have retired every command: no gross error latched.
    uint8_t stat = reg_r(4);
    CHECK((stat & 0x40) == 0, "S_GROSS_ERROR latched (stat=%02x): queue "
                              "jammed behind an unretired 0x10", stat);
    sd_mock.gap = 0;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    // Deterministic pattern pool (matches the fuzz harness flavour).
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)((i * 13 + 7) & 0xff);

    reset();
    test_healthy_perbyte();
    test_residue_one_byte();
    test_supply_exhaustion_trailing();

    bool all_ran = t1_ran && t2_ran && t3_ran;
    printf("tb_scsi_c96_nondma_trailing: %d passed, %d failed%s\n",
           n_pass, n_fail, all_ran ? "" : " (A STAGE DID NOT RUN)");
    delete dut;
    return (n_fail == 0 && all_ran) ? 0 : 1;
}
