// tb_scsi_c96_cmdout_drain.cpp — System 7.0.1 SCSI Manager command-out
// flow against the C96 path: deferred select + progressive CDB FIFO
// drain + (ATN form) MSG_OUT-before-COMMAND.
//
// Repro of the 2026-07-21 real-HW post-"Welcome to Macintosh" boot
// wedge: the RAM-resident SCSI Manager the System file installs issues
//   1. tcount = 1, command 0xC1 (DMA | CD_SELECT), FIFO EMPTY
//   2. writes CDB bytes 0..n-2 into the FIFO (reg 2)
//   3. busy-waits:  while ((fifo_flags & 0x1F) != 0 &&
//                          (status & 7) == COMMAND(2))  spin;
//   4. writes the LAST CDB byte through the DAFB pseudo-DMA port
// (driver code observed in low RAM ~0x29AE0 on HW / 0x2992e in MAME —
// disassembly in the 2026-07-21 investigation).  On a real 53C96 (and
// MAME's ncr53c90) the select sequence REQ/ACKs each FIFO byte to the
// target as it arrives, so step 3 terminates with count==0 and the bus
// still in COMMAND phase.  The pre-fix RTL accumulated the bytes in the
// visible FIFO until the whole CDB had arrived — deadlocking step 3
// forever (fifo_pos==5, phase==COMMAND).
//
// Scenario 2 covers the ATN form the same driver uses when it wants
// disconnect privileges: 0xC2 (DMA | CD_SELECT_ATN) with an empty FIFO
// must report MSG_OUT (110) first, consume one IDENTIFY byte from a
// FIFO write, and only then report COMMAND (010).
//
// Build via:   make tb-scsi-c96-cmdout-drain

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"

static Vtb_scsi_vhdd_sd* dut    = nullptr;
static int    n_pass = 0;
static int    n_fail = 0;

// ─── Mocked SD backing store (mirrors tb_scsi_c96_read6.cpp) ────────
struct SdMock {
    std::vector<uint8_t> read_sector;
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    bool     present = true;

    enum class State { Idle, Reading, Writing, Done } st = State::Idle;
    int cnt = 0;
    int total = 512;
    int delay = 0;
} sd_mock;

static void sd_mock_tick() {
    dut->sd_busy      = (sd_mock.st != SdMock::State::Idle) ? 1 : 0;
    dut->sd_done      = 0;
    dut->sd_error     = 0;
    dut->sd_rd_valid  = 0;
    dut->sd_rd_data   = 0;
    dut->sd_wr_ready  = 0;

    if (sd_mock.present && sd_mock.st == SdMock::State::Idle && dut->sd_go) {
        sd_mock.last_lba      = dut->sd_lba;
        sd_mock.last_cmd_type = dut->sd_cmd_type;
        sd_mock.cnt           = 0;
        sd_mock.delay         = 2;
        if (dut->sd_cmd_type == 1) {
            sd_mock.st    = SdMock::State::Reading;   // CMD17
            sd_mock.total = 512;
        } else if (dut->sd_cmd_type == 2) {
            sd_mock.st    = SdMock::State::Reading;   // CMD18
            sd_mock.total = 512 * (int)dut->sd_block_count;
        } else if (dut->sd_cmd_type == 3) {
            sd_mock.st = SdMock::State::Writing;
        } else {
            sd_mock.st = SdMock::State::Done;
        }
        dut->sd_busy = 1;
        return;
    }
    if (sd_mock.st == SdMock::State::Reading) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) { --sd_mock.delay; return; }
        if (sd_mock.cnt < sd_mock.total) {
            dut->sd_rd_valid = 1;
            uint8_t byte = 0;
            if (!sd_mock.read_sector.empty()) {
                byte = sd_mock.read_sector[sd_mock.cnt %
                                            sd_mock.read_sector.size()];
            }
            dut->sd_rd_data = byte;
            ++sd_mock.cnt;
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
    dut->clk = 1;
    dut->eval();
    dut->clk = 0;
    dut->eval();
}

static void tick_n(int n) { for (int i = 0; i < n; ++i) tick(); }

static void reset() {
    dut->rst         = 1;
    // Runtime disk size (from the SD CSD at boot); without it the target
    // reports a zero-sector disk and every read fails.
    dut->disk_num_lbas = 1048576u;
    dut->pb_addr     = 0;
    dut->pb_wdata    = 0;
    dut->pb_wr       = 0;
    dut->pb_rd       = 0;
    sd_mock = SdMock();
    for (int i = 0; i < 4; ++i) tick();
    dut->rst = 0;
    tick();
}

static uint8_t reg_r(uint8_t off) {
    dut->pb_addr = off & 0xf;
    dut->pb_rd   = 1;
    dut->pb_wr   = 0;
    tick();
    dut->pb_rd   = 0;
    dut->eval();
    return dut->pb_rdata & 0xff;
}

static void reg_w(uint8_t off, uint8_t v) {
    dut->pb_addr  = off & 0xf;
    dut->pb_wdata = v;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    tick();
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
}

static uint8_t shim_r() {
    dut->pb_addr  = 0x100;
    dut->pb_rd    = 1;
    dut->pb_wr    = 0;
    tick();
    dut->pb_rd    = 0;
    dut->eval();
    return dut->pb_rdata & 0xff;
}

static void shim_w(uint8_t v) {
    dut->pb_addr  = 0x100;
    dut->pb_wdata = v;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    tick();
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
}

#define CHECK_EQ(name, got, exp) do {                                     \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp);         \
    if (_g != _e) {                                                       \
        std::printf("  FAIL %s: got 0x%02x, expected 0x%02x\n",           \
                    name, _g, _e);                                        \
        return false;                                                     \
    }                                                                     \
} while (0)

#define CHECK_TRUE(name, cond) do {                                       \
    if (!(cond)) { std::printf("  FAIL %s\n", name); return false; }      \
} while (0)

#define RUN(test_fn) do {                                                 \
    std::printf("[RUN ] %s\n", #test_fn);                                 \
    bool _ok = test_fn();                                                 \
    if (_ok) { ++n_pass; std::printf("[PASS] %s\n", #test_fn); }          \
    else     { ++n_fail; std::printf("[FAIL] %s\n", #test_fn); }          \
} while (0)

// 53C9x command opcodes
static constexpr uint8_t CM_RESET       = 0x02;
static constexpr uint8_t CD_SELECT      = 0x41;
static constexpr uint8_t CD_SELECT_ATN  = 0x42;
static constexpr uint8_t CI_XFER        = 0x10;
static constexpr uint8_t CI_COMPLETE    = 0x11;
static constexpr uint8_t CI_MSG_ACCEPT  = 0x12;

// status / istatus bits
static constexpr uint8_t S_INTR       = 0x80;
static constexpr uint8_t I_FUNCTION   = 0x08;
static constexpr uint8_t I_BUS        = 0x10;
static constexpr uint8_t I_DISCONNECT = 0x20;

static bool wait_intr(int spins) {
    for (int i = 0; i < spins; ++i) {
        if (reg_r(0x4) & S_INTR) return true;
        tick_n(1);
    }
    return false;
}

// Wait until (status & 7) == want; returns true if reached.
static bool wait_phase(uint8_t want, int spins) {
    for (int i = 0; i < spins; ++i) {
        if ((reg_r(0x4) & 0x07) == want) return true;
        tick_n(1);
    }
    return false;
}

static constexpr uint32_t SD_RAW_BASE_LBA = 8192u;
static constexpr uint8_t  TARGET_ID = 6;

// Shared tail: CDB stuffing with the System-7 drain-poll, tail byte via
// the pseudo-DMA port, then a full 512-byte DATA IN drain + completion.
static bool sys7_cdb_and_data_phase() {
    // System 7 driver: CDB bytes 0..n-2 via FIFO writes.
    // READ(6) LBA 0, 1 block: 08 00 00 00 01 00 — first 5 via FIFO.
    const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 5; ++i) reg_w(0x2, cdb[i]);

    // The drain-poll (driver low-RAM loop ~0x2992e):
    //   d1 = fifo_flags & 0x1F; if (d1 == 0) break;
    //   d1 = status & 7;        if (d1 != 2) break;
    // Bounded here; on the pre-fix RTL it never exits with count==0.
    uint8_t count = 0xff, ph = 0xff;
    for (int spin = 0; spin < 2000; ++spin) {
        count = reg_r(0x7) & 0x1f;
        if (count == 0) { ph = reg_r(0x4) & 0x07; break; }
        ph = reg_r(0x4) & 0x07;
        if (ph != 0x02) break;
    }
    CHECK_EQ("drain-poll exits with FIFO empty", count, 0x00);
    CHECK_EQ("still COMMAND phase after drain (tail byte owed)", ph, 0x02);

    // Tail CDB byte via the DAFB pseudo-DMA port (tcount=1 beat).
    shim_w(cdb[5]);

    // Select-complete: I_FUNCTION|I_BUS once the back-end settles in
    // its first real bus phase (DATA IN once the SD mock filled it).
    CHECK_TRUE("select-complete interrupt fires", wait_intr(8000));
    CHECK_EQ("select-complete phase = DATA IN", reg_r(0x4) & 0x07, 0x01);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    // Arm the data transfer (DMA transfer-information) and drain.
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    reg_w(0x3, 0x80 | CI_XFER);

    std::vector<uint8_t> data(512, 0);
    int got = 0;
    for (int spin = 0; spin < 8000 && got < 512; ++spin) {
        uint8_t s = reg_r(0x4);
        if ((s & 0x07) != 0x01) { tick_n(1); continue; }
        data[got++] = shim_r();
    }
    CHECK_EQ("got 512 READ bytes", got, 512);

    bool match = true;
    for (int i = 0; i < 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, data[i], sd_mock.read_sector[i]);
            match = false;
            break;
        }
    }
    CHECK_TRUE("READ data byte-identical", match);
    CHECK_EQ("sd_cmd_type was CMD17 (1)", sd_mock.last_cmd_type, 1);
    CHECK_EQ("sd_lba was 8192 (HDD partition base)",
             sd_mock.last_lba, SD_RAW_BASE_LBA);

    CHECK_TRUE("after 512 bytes I_BUS fires", wait_intr(200));
    CHECK_EQ("phase = STATUS after data", reg_r(0x4) & 0x07, 0x03);
    // Retire the 0x90 first: commands queue until a nonzero istatus
    // read pops them (MAME command_pop_and_chain; scsi_fuzz finding F8).
    CHECK_EQ("data-phase istatus = I_BUS", reg_r(0x5), I_BUS);

    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);

    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    tick_n(4);
    CHECK_EQ("bus free after MSG_ACCEPT", reg_r(0x4) & 0x07, 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 1 — the HW wedge repro: DMA|CD_SELECT (0xC1), empty FIFO,
// tcount=1, CDB 0..4 via FIFO, drain-poll, tail via pseudo-DMA port.
// ─────────────────────────────────────────────────────────────────────
static bool test_sys7_deferred_select_cmdout_drain() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i) {
        sd_mock.read_sector[i] = (uint8_t)(0xA5 ^ (i & 0xFF) ^ (i >> 4));
    }

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);

    // Driver programs tcount=1 (the tail CDB byte is the one DMA beat).
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0x80 | CD_SELECT);          // 0xC1, FIFO EMPTY

    // Driver gate: seq_step != 0 and phase == COMMAND before stuffing.
    CHECK_TRUE("phase == COMMAND after empty-FIFO select",
               wait_phase(0x02, 100));
    CHECK_TRUE("seq_step != 0 after select", reg_r(0x6) != 0x00);

    return sys7_cdb_and_data_phase();
}

// ─────────────────────────────────────────────────────────────────────
// Test 2 — ATN form: DMA|CD_SELECT_ATN (0xC2), empty FIFO.  Real chip
// (MAME ncr53c90) enters MSG_OUT first; the driver polls for phase 6,
// writes IDENTIFY (0x80) to the FIFO, then polls for COMMAND and runs
// the same CDB flow as test 1.
// ─────────────────────────────────────────────────────────────────────
static bool test_sys7_select_atn_msgout_first() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i) {
        sd_mock.read_sector[i] = (uint8_t)(0x3C ^ (i & 0xFF));
    }

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);

    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0x80 | CD_SELECT_ATN);      // 0xC2, FIFO EMPTY

    // Driver waits for MSG_OUT (110 = 6) before sending IDENTIFY.
    CHECK_TRUE("phase == MSG_OUT after empty-FIFO ATN select",
               wait_phase(0x06, 100));

    // IDENTIFY (no disconnect privilege): 0x80.  Must be consumed as
    // the message — NOT counted as a CDB byte, FIFO stays empty.
    reg_w(0x2, 0x80);
    CHECK_EQ("FIFO empty after IDENTIFY consumed", reg_r(0x7) & 0x1f, 0);

    // Then the target switches to COMMAND.
    CHECK_TRUE("phase == COMMAND after IDENTIFY", wait_phase(0x02, 100));

    return sys7_cdb_and_data_phase();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    RUN(test_sys7_deferred_select_cmdout_drain);
    RUN(test_sys7_select_atn_msgout_first);

    std::printf("\n%d passed, %d failed\n", n_pass, n_fail);
    const int rc = (n_fail == 0) ? 0 : 1;
    dut->final();
    delete dut;
    return rc;
}
