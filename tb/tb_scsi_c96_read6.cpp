// tb_scsi_c96_read6.cpp — end-to-end READ(6) of a 512-byte sector via
// the C96 path, with the SD backing store mocked.
//
// Drives the chip exactly like the Q700 ROM's SCSI Manager driver
// (READ_6 of LBA 0 / 1 block; MAME ncr53c90.cpp is the reference):
//   1. CM_RESET (chip)
//   2. bus_id = TARGET_ID (6)
//   3. Push 6-byte READ(6) CDB into FIFO: 08 00 00 00 01 00
//   4. tcount = 512
//   5. command = (DMA bit) | CD_SELECT_ATN_STOP
//   6. Mock SD CMD17 fires; mock returns 512 known bytes
//   7. Wait select-complete: istatus = I_FUNCTION|I_BUS, phase DATA IN
//   8. Arm the transfer: tcount = 512, command 0x90 (DMA | CI_XFER)
//   9. Drain 512 bytes via DMA shim
//  10. Validate byte-identical with the injected pattern
//  11. I_BUS fires on the DATA_IN→STATUS phase change; CI_COMPLETE
//      pulls status/msg into the FIFO; CI_MSG_ACCEPT frees the bus
//
// This is the M4 deliverable: proves that the C96 → back-end → sd_ctrl
// data path works on a SCSI LBA that maps (via sd_scsi_lba_mapper) to
// SD sector 8192 + 0 = 8192 (the start of the HDD partition on the
// SD card — boot blocks live here).
//
// Build via:   make tb-scsi-c96-read6

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"

static Vtb_scsi_vhdd_sd* dut    = nullptr;
static int    n_pass = 0;
static int    n_fail = 0;

// ─── Mocked SD backing store (mirrors tb_scsi.cpp's SdMock) ─────────
struct SdMock {
    std::vector<uint8_t> read_sector;     // byte pool used as read source
    std::vector<uint8_t> written_sector;
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    bool     present = true;
    bool     inject_error = false;
    // Real-SD pacing: cycles between rd_valid pulses.  The physical
    // sd_ctrl SPI path emits one byte every ~320 ns (~16 sys-clks at
    // 50 MHz); gap > 0 forces the CMD18 ring to underrun mid-chunk so
    // TC0 lags the chunk arm — the timing shape the unit tbs never
    // exercised before (and exactly the sim/HW gap that hid the
    // tcounter-semantics bug).
    int      gap = 0;
    // Consumer back-pressure: the fixed sd_ctrl checks rd_ready before
    // issuing the next SPI byte-read; the bridge CDC + one in-flight
    // SPI byte gives a small skid.  Model: when sd_rd_ready is low we
    // may emit up to `skid_max` more bytes, then pause until it rises
    // again.  Pre-fix RTL ties sd_rd_ready high, so this model is
    // transparent there (that's what makes the ring-overrun test a
    // faithful repro of the HW wedge).
    int      skid_max = 2;
    int      skid = 2;

    enum class State { Idle, Reading, Writing, Done } st = State::Idle;
    int cnt = 0;
    int total = 512;
    int delay = 0;
    int gap_ctr = 0;
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
        sd_mock.gap_ctr       = 0;
        if (dut->sd_cmd_type == 1) {
            sd_mock.st    = SdMock::State::Reading;   // CMD17
            sd_mock.total = 512;
        } else if (dut->sd_cmd_type == 2) {
            sd_mock.st    = SdMock::State::Reading;   // CMD18 multi-block
            sd_mock.total = 512 * (int)dut->sd_block_count;
        } else if (dut->sd_cmd_type == 3 || dut->sd_cmd_type == 4) {
            sd_mock.st    = SdMock::State::Writing;   // CMD24 / CMD25
            sd_mock.total = 512 * (dut->sd_block_count
                                   ? (int)dut->sd_block_count : 1);
        } else {
            sd_mock.st = SdMock::State::Done;
        }
        dut->sd_busy = 1;
        return;
    }
    if (sd_mock.st == SdMock::State::Reading) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) { --sd_mock.delay; return; }
        if (sd_mock.gap_ctr > 0) { --sd_mock.gap_ctr; return; }
        // Honor the DUT's read back-pressure (see skid comment above).
        if (dut->sd_rd_ready) {
            sd_mock.skid = sd_mock.skid_max;
        } else if (sd_mock.skid > 0) {
            --sd_mock.skid;
        } else {
            return;                      // paused: ring full downstream
        }
        if (sd_mock.cnt < sd_mock.total) {
            if (sd_mock.inject_error && sd_mock.cnt >= 128) {
                sd_mock.st    = SdMock::State::Done;
                dut->sd_done  = 1;
                dut->sd_error = 1;
                return;
            }
            dut->sd_rd_valid = 1;
            uint8_t byte = 0;
            if (!sd_mock.read_sector.empty()) {
                byte = sd_mock.read_sector[sd_mock.cnt %
                                            sd_mock.read_sector.size()];
            }
            dut->sd_rd_data = byte;
            ++sd_mock.cnt;
            sd_mock.gap_ctr = sd_mock.gap;
            if (sd_mock.cnt == sd_mock.total) sd_mock.st = SdMock::State::Done;
        }
    } else if (sd_mock.st == SdMock::State::Writing) {
        // Record what the DUT actually hands over, so a data-OUT test can
        // verify the payload that reached the backing store rather than
        // only that the chip stopped complaining.
        //
        // NOTE the handshake: a byte transfers only when BOTH sd_wr_ready
        // (ours) and sd_wr_valid (the DUT's) are high, so ready is raised
        // and the DUT re-evaluated before sampling.  tb_scsi.cpp's older
        // write mock samples on ready alone; that is harmless there
        // because its only write test is single-block (the whole block is
        // already buffered before S_VH_WAIT_WR starts draining), but on
        // the multi-block ring — where fill and drain overlap — sampling
        // without valid records bytes the producer has not written yet.
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) { --sd_mock.delay; return; }
        if (sd_mock.cnt < sd_mock.total) {
            dut->sd_wr_ready = 1;
            dut->eval();
            if (dut->sd_wr_valid) {
                sd_mock.written_sector.push_back(dut->sd_wr_data & 0xFF);
                ++sd_mock.cnt;
                if (sd_mock.cnt == sd_mock.total)
                    sd_mock.st = SdMock::State::Done;
            }
        }
    } else if (sd_mock.st == SdMock::State::Done) {
        dut->sd_busy = 0;
        dut->sd_done = 1;
        if (sd_mock.inject_error) dut->sd_error = 1;
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

// Register-aperture helpers
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
static constexpr uint8_t CM_RESET           = 0x02;
static constexpr uint8_t CD_SELECT          = 0x41;
static constexpr uint8_t CD_SELECT_ATN_STOP = 0x43;
static constexpr uint8_t CD_SELECT_ATN      = 0x42;
static constexpr uint8_t CI_XFER            = 0x10;
static constexpr uint8_t CI_COMPLETE        = 0x11;
static constexpr uint8_t CI_MSG_ACCEPT      = 0x12;

// status / istatus bits
static constexpr uint8_t S_INTR       = 0x80;
static constexpr uint8_t I_FUNCTION   = 0x08;
static constexpr uint8_t I_BUS        = 0x10;
static constexpr uint8_t I_DISCONNECT = 0x20;

// Wait for the INTR mirror in the status register, ticking between
// polls; returns true if it fired within `spins` polls.
static bool wait_intr(int spins) {
    for (int i = 0; i < spins; ++i) {
        if (reg_r(0x4) & S_INTR) return true;
        tick_n(1);
    }
    return false;
}

// SD bias: SCSI LBA 0 → SD sector 8192 (raw partition base).
static constexpr uint32_t SD_RAW_BASE_LBA = 8192u;

static constexpr uint8_t TARGET_ID = 6;

// ─────────────────────────────────────────────────────────────────────
// Test 1 — READ(6) LBA 0, 1 block, byte-identical against pattern.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_read6_lba0_one_block() {
    reset();

    // Inject a known sector pattern into the mock backing store.
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i) {
        sd_mock.read_sector[i] = (uint8_t)(0x80 ^ (i & 0xFF) ^ (i >> 8));
    }

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);

    // READ(6): opcode 0x08, LBA[20:16]=0 LBA[15:8]=0 LBA[7:0]=0,
    // alloc=1 (1 block), control=0.
    const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    CHECK_EQ("FIFO has 6 bytes (CDB)", reg_r(0x7), 0x06);

    reg_w(0x0, 0x00);   // tcount lo
    reg_w(0x1, 0x02);   // tcount hi (= 512)
    // ATN_STOP with a preloaded CDB now halts after one message byte
    // (MAME contract, scsi_fuzz finding F2) — use the bare DMA select,
    // which consumes the whole CDB, for these end-to-end data flows.
    reg_w(0x3, 0x80 | CD_SELECT);

    // Select-complete: fires once the SD mock has filled the buffer and
    // the back-end settles in DATA_IN (I_FUNCTION|I_BUS, seq 4).
    CHECK_TRUE("select-complete interrupt fires", wait_intr(8000));
    CHECK_EQ("select-complete phase = DATA IN", reg_r(0x4) & 0x07, 0x01);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    // Arm the data transfer (DMA transfer-information).
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    reg_w(0x3, 0x80 | CI_XFER);

    // Drain 512 bytes via DMA shim.
    std::vector<uint8_t> data(512, 0);
    int got = 0;
    for (int spin = 0; spin < 8000 && got < 512; ++spin) {
        uint8_t s = reg_r(0x4);
        if ((s & 0x07) != 0x01) { tick_n(1); continue; }
        data[got++] = shim_r();
    }
    CHECK_EQ("got 512 READ bytes", got, 512);

    // Verify byte-identical with the mock backing store pattern.
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

    // SD command type was CMD17 (read).
    CHECK_EQ("sd_cmd_type was CMD17 (1)", sd_mock.last_cmd_type, 1);
    // SD LBA was bias + 0 = 8192.
    CHECK_EQ("sd_lba was 8192 (HDD partition base)",
             sd_mock.last_lba, SD_RAW_BASE_LBA);

    // After 512 bytes: DATA_IN→STATUS phase change, I_BUS fires.
    CHECK_TRUE("after 512 bytes I_BUS fires", wait_intr(200));
    CHECK_EQ("phase = STATUS after data", reg_r(0x4) & 0x07, 0x03);
    CHECK_EQ("phase-change istatus = I_BUS", reg_r(0x5), I_BUS);

    // CI_COMPLETE pulls status (00 GOOD) + msg (00 COMPLETE) into FIFO.
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("FIFO has status+msg", reg_r(0x7), 0x02);
    CHECK_EQ("status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);

    // CI_MSG_ACCEPT frees the bus.
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    tick_n(4);
    CHECK_EQ("bus free after MSG_ACCEPT", reg_r(0x4) & 0x07, 0x00);
    return true;
}


// ─────────────────────────────────────────────────────────────────────
// Test — READ(10) LBA 2314, 1 block, via the C96 path.
//
// Why: this exact command is what FAILS on hardware.  The .ASYC00 disk
// driver issues READ(10) LBA 0x090A x1, gets CHECK CONDITION, retries 16x
// and returns ioErr(-36) -- while the volume reports ENABLED / 4194304
// blocks and sd_ctrl logs ZERO errors, i.e. the command is rejected before
// any back-end request.  Before this test there was NO READ(10) coverage on
// the C96 path at all (every tb_scsi_c96_*.cpp used READ(6)), even though
// READ(10) via C96 is what Mac OS actually issues -- MAME's trace of the
// same boot is 3597 x READ(10) and zero READ(6).
//
// The suspected arm is scsi.v:2905 `!vh_chk_ok`: sd_scsi_lba_mapper requires
// scsi_blocks != 0, and scsi.v:833 wires vh_chk_blocks to the LIVE
// xfer_blocks, which is cleared at selection (scsi.v:2462).  If the C96 fast
// path reaches S_VH_WAIT_RD before the CDB loads the block count, a valid
// read is rejected with ILLEGAL REQUEST / ASC 0x21.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_read10_lba2314_one_block() {
    reset();

    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x5A ^ (i & 0xFF) ^ (i >> 8));

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);

    // READ(10): opcode 0x28, LBA 0x0000090A, transfer length 0x0001.
    const uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x09, 0x0A,
                             0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; ++i) reg_w(0x2, cdb[i]);
    CHECK_EQ("FIFO has 10 bytes (CDB)", reg_r(0x7), 0x0A);

    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    // ATN_STOP with a preloaded CDB now halts after one message byte
    // (MAME contract, scsi_fuzz finding F2) — use the bare DMA select,
    // which consumes the whole CDB, for these end-to-end data flows.
    reg_w(0x3, 0x80 | CD_SELECT);

    CHECK_TRUE("select-complete interrupt fires", wait_intr(8000));
    // A rejected read lands in STATUS (011) instead of DATA IN (001).
    uint8_t ph = reg_r(0x4) & 0x07;
    if (ph != 0x01) {
        std::printf("  FAIL READ(10) went to phase %u (STATUS=3) instead of "
                    "DATA IN -- the command was REJECTED before any data\n", ph);
        return false;
    }
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

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
    CHECK_EQ("got 512 READ(10) bytes", got, 512);

    bool match = true;
    for (int i = 0; i < 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, data[i], sd_mock.read_sector[i]);
            match = false; break;
        }
    }
    CHECK_TRUE("READ(10) data byte-identical", match);
    CHECK_EQ("sd_cmd_type was CMD17 (1)", sd_mock.last_cmd_type, 1);
    CHECK_EQ("sd_lba was 8192 + 2314", sd_mock.last_lba,
             SD_RAW_BASE_LBA + 2314u);

    CHECK_TRUE("after 512 bytes I_BUS fires", wait_intr(200));
    CHECK_EQ("phase = STATUS after data", reg_r(0x4) & 0x07, 0x03);
    // Retire the 0x90: commands queue until an istatus read pops them
    // (MAME command_pop_and_chain; scsi_fuzz finding F8).
    CHECK_EQ("data-phase istatus = I_BUS", reg_r(0x5), I_BUS);

    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("FIFO has status+msg", reg_r(0x7), 0x02);
    // THE assertion: a valid READ(10) must not come back CHECK CONDITION.
    uint8_t st = reg_r(0x2);
    if (st != 0x00) {
        std::printf("  FAIL READ(10) status 0x%02x (0x02 = CHECK CONDITION) "
                    "on a valid LBA -- this is the hardware bug\n", st);
        return false;
    }
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 2 — READ(6) LBA 1 maps to SD sector 8193 (bias offset).
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_read6_lba1_offset() {
    reset();
    sd_mock.read_sector.assign(512, 0xA5);   // distinct pattern

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x01, 0x01, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    // ATN_STOP with a preloaded CDB now halts after one message byte
    // (MAME contract, scsi_fuzz finding F2) — use the bare DMA select,
    // which consumes the whole CDB, for these end-to-end data flows.
    reg_w(0x3, 0x80 | CD_SELECT);

    // Select-complete, then arm the transfer.
    CHECK_TRUE("select-complete interrupt fires", wait_intr(8000));
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    reg_w(0x3, 0x80 | CI_XFER);

    int got = 0;
    for (int spin = 0; spin < 8000 && got < 512; ++spin) {
        uint8_t s = reg_r(0x4);
        if ((s & 0x07) != 0x01) { tick_n(1); continue; }
        (void)shim_r();
        ++got;
    }
    CHECK_EQ("got 512 bytes", got, 512);

    // sd_lba should be SD_RAW_BASE_LBA + 1 = 8193.
    CHECK_EQ("sd_lba was 8193 (LBA 1 → 8192+1)",
             sd_mock.last_lba, SD_RAW_BASE_LBA + 1);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 3 — READ(6) with SD-side error → CHECK CONDITION on the wire.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_read6_sd_error() {
    reset();
    sd_mock.read_sector.assign(512, 0x42);
    sd_mock.inject_error = true;

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    // ATN_STOP with a preloaded CDB now halts after one message byte
    // (MAME contract, scsi_fuzz finding F2) — use the bare DMA select,
    // which consumes the whole CDB, for these end-to-end data flows.
    reg_w(0x3, 0x80 | CD_SELECT);

    // The SD mock errors before a full block arrives, so the back-end
    // goes straight to STATUS with CHECK CONDITION — the select-complete
    // interrupt (I_FUNCTION|I_BUS) fires with phase = STATUS (011); the
    // initiator sees no data phase and runs CI_COMPLETE directly.
    CHECK_TRUE("select-complete interrupt fires", wait_intr(4000));
    CHECK_EQ("error path phase = STATUS", reg_r(0x4) & 0x07, 0x03);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    // CI_COMPLETE returns the REAL status byte — CHECK CONDITION.
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("FIFO has status+msg", reg_r(0x7), 0x02);
    CHECK_EQ("status = CHECK CONDITION", reg_r(0x2), 0x02);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Golden Q700 ROM boot-scan flow — byte-for-byte replay of the MAME
// 0.285 macqd700 register trace (captured 2026-07-15 with a Lua memory
// tap on 0x5000F000..0x5000F1FF while booting System 7.0.1 from an
// nscsi harddisk at ID 6).  Every boot-scan transaction looks like:
//
//   1. bus_id=6; FLUSH_FIFO; tcount=0x0001; command 0xC1 (DMA|SELECT)
//      with the FIFO STILL EMPTY.
//   2. Poll seq_step until != 0 (MAME's empty-fifo arbitration hack
//      returns 1), then poll status until phase bits != 0 (COMMAND,
//      010) — ROM 0x40898e12 loop.
//   3. Push CDB bytes 0..n-2 through the FIFO (ROM 0x40898e90), the
//      LAST CDB byte through the DAFB pseudo-DMA port (ROM 0x40898eb4;
//      the select's tcount=1 covers it → TC0).
//   4. Poll status for 0x91 = INTR|TC0|DATA_IN; fifo_flags == 0;
//      istatus == 0x18 (I_FUNCTION|I_BUS select-complete).
//   5. Per 16-byte chunk (ROM 0x408992b8): FLUSH_FIFO; tcount=0x0010;
//      command 0x90 (DMA|XFER); poll status until TC0 (bit 4) — the
//      counter must exhaust BEFORE the CPU drains a single byte (MAME
//      decrements tcounter on chip-side ACKO, not on CPU DACK); then
//      16 DMA-port reads; then poll status for INTR; istatus == 0x10
//      (I_BUS, MAME INIT_XFR_BUS_COMPLETE at TC0+drq-low, with the
//      target still in DATA_IN).
//   6. After the final chunk: phase == STATUS (011); CI_COMPLETE →
//      I_FUNCTION + status/msg in FIFO; CI_MSG_ACCEPT → I_DISCONNECT.
//
// This is the sequence the previous unit tests did NOT model (they
// armed one big CI_XFER with tcount == transfer size and polled
// istatus, not status-TC0) — which is why they stayed green while the
// real ROM wedged in S_DATA_IN on hardware.
// ─────────────────────────────────────────────────────────────────────
static bool rom_flow_read6(uint32_t lba, int blocks,
                           std::vector<uint8_t>& out,
                           int stall_after_bytes = -1,
                           int stall_cycles = 0,
                           bool stall_before_drain = false,
                           // Real bus-access latency per blind DMA-port
                           // read: on real hardware each CPU movew to
                           // the DAFB pseudo-DMA window takes several
                           // core_clk cycles (AXI xbar + peripheral_bus
                           // handshake + DAFB CDC round trip), NOT the
                           // single tick() shim_r() costs here.  This
                           // matters: with zero inter-read latency the
                           // ROM's 8-word blind burst (0x40899300-
                           // 0x4089931c) completes in ~16 core cycles,
                           // far faster than a gap-paced CMD18
                           // background refill can interleave — hiding
                           // any bug in beat-counting/data-select logic
                           // that's sensitive to mid-burst ring state
                           // (t_req toggling from unrelated refill
                           // bookkeeping).  Set > 0 to make the burst
                           // realistically slow enough for that
                           // interleaving to actually occur, the way it
                           // does on real silicon.
                           int bus_latency_cycles = 0) {
    const int total = blocks * 512;

    // 1. Select with EMPTY FIFO (DMA form, tcount = 1).
    reg_w(0x4, TARGET_ID);
    reg_w(0x3, 0x01);               // FLUSH_FIFO
    reg_w(0x1, 0x00);               // tcount hi
    reg_w(0x0, 0x01);               // tcount lo = 1
    reg_w(0x3, 0xC1);               // DMA | CD_SELECT — FIFO EMPTY

    // 2. ROM gate: seq_step != 0, then status phase != 0.
    uint8_t v = 0;
    for (int i = 0; i < 64; ++i) { v = reg_r(0x6); if (v & 7) break; }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", (v & 7) != 0);
    for (int i = 0; i < 64; ++i) { v = reg_r(0x4); if (v & 7) break; }
    CHECK_EQ("status phase = COMMAND during CDB stuffing", v & 7, 0x2);

    // 3. CDB: first 5 bytes via FIFO, control byte via the DMA port.
    reg_w(0x2, 0x08);
    reg_w(0x2, (lba >> 16) & 0x1f);
    reg_w(0x2, (lba >> 8) & 0xff);
    reg_w(0x2, lba & 0xff);
    reg_w(0x2, blocks & 0xff);
    shim_w(0x00);

    // 4. Select-complete: 0x91 (SD latency allowed in between — a
    //    gap-paced CMD18 first block takes ~512*(gap+1) cycles).
    for (int i = 0; i < 64000; ++i) { v = reg_r(0x4); if (v & 0x80) break; }
    CHECK_EQ("post-select status = INTR|TC0|DATA_IN", v, 0x91);
    CHECK_EQ("fifo drained after CDB send", reg_r(0x7), 0x00);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    // 5. 16-byte chunk loop.
    out.clear();
    bool stalled = false;
    while ((int)out.size() < total) {
        // Optional mid-transfer drain stall: models a VBL/Timer ISR
        // pre-empting the ROM's poll loop for `stall_cycles` while the
        // SD stream keeps delivering (the HW failure was captured with
        // VBL interrupts actively firing — exc_count climbing — and
        // wedged only minutes into boot, i.e. only when an ISR landed
        // inside a long multi-block CMD18).
        bool stall_this_chunk = false;
        if (!stalled && stall_after_bytes >= 0 &&
            (int)out.size() >= stall_after_bytes) {
            stalled = true;
            if (stall_before_drain) stall_this_chunk = true;
            else                    tick_n(stall_cycles);
        }
        reg_w(0x3, 0x01);           // FLUSH_FIFO
        reg_w(0x1, 0x00);
        reg_w(0x0, 0x10);           // tcount = 16
        reg_w(0x3, 0x90);           // DMA | CI_XFER
        // ROM poll1 (0x408992d2): wait for TC0 *before* any DMA read.
        // INTR here would be the ROM's error path.
        bool tc0 = false;
        for (int i = 0; i < 200000; ++i) {
            v = reg_r(0x4);
            if (v & 0x10) { tc0 = true; break; }
            if (v & 0x80) break;                  // unexpected interrupt
        }
        CHECK_TRUE("TC0 sets before the CPU drains the chunk", tc0);
        // ROM poll2 (0x408992f6-0x408992fc, golden-trace-confirmed):
        // one software DRQ (DAFB +0x24 bit 9) check before the blind
        // burst — NOT rechecked per-word inside the burst itself.
        bool drq_pre = false;
        for (int i = 0; i < 4000; ++i) {
            if (dut->drq) { drq_pre = true; break; }
            tick_n(1);
        }
        CHECK_TRUE("software DRQ pre-check before blind burst", drq_pre);
        // Drain-window stall: the ISR lands between the ROM's TC0 poll
        // (0x408992d2) and its 16 blind DMA-port reads (0x40899300) —
        // the window where the HW wedge's lost beats occur.
        if (stall_this_chunk) tick_n(stall_cycles);
        for (int i = 0; i < 16; ++i) {
            if (bus_latency_cycles > 0) tick_n(bus_latency_cycles);
            out.push_back(shim_r());
        }
        // ROM 0x40899322: wait for INTR after the drain.
        bool intr = false;
        for (int i = 0; i < 4000; ++i) {
            v = reg_r(0x4);
            if (v & 0x80) { intr = true; break; }
        }
        CHECK_TRUE("I_BUS interrupt after chunk drain", intr);
        CHECK_EQ("chunk istatus = I_BUS", reg_r(0x5), I_BUS);
    }

    // 6. Status/message per ROM 0x40898cae..0x40898cde.
    CHECK_EQ("phase = STATUS after final chunk", reg_r(0x4) & 7, 0x3);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("FIFO has status+msg", reg_r(0x7), 0x02);
    CHECK_EQ("status = GOOD", reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    tick_n(4);
    CHECK_EQ("bus free after MSG_ACCEPT", reg_r(0x4) & 7, 0x00);
    return true;
}

// Test 4 — golden ROM flow, single-block READ(6) (CMD17 path).
static bool test_c96_rom_flow_read6_one_block() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x5A ^ (i & 0xFF) ^ (i >> 6));

    std::vector<uint8_t> data;
    if (!rom_flow_read6(0, 1, data)) return false;

    CHECK_EQ("got 512 bytes", (uint32_t)data.size(), 512);
    for (int i = 0; i < 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, data[i], sd_mock.read_sector[i]);
            return false;
        }
    }
    CHECK_EQ("sd_cmd_type was CMD17 (1)", sd_mock.last_cmd_type, 1);
    CHECK_EQ("sd_lba was 8192", sd_mock.last_lba, SD_RAW_BASE_LBA);
    return true;
}

// Test 5 — golden ROM flow, 3-block READ(6) through the CMD18 ring,
// with the SD mock paced at one byte per 20 cycles (slower than the
// initiator's chunk drain, like the real ~320 ns/byte SPI stream) so
// mid-chunk ring underruns and lagging TC0 are actually exercised.
static bool test_c96_rom_flow_cmd18_slow_sd() {
    reset();
    sd_mock.read_sector.resize(3 * 512);
    for (int i = 0; i < 3 * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0xC3 ^ (i & 0xFF) ^ (i >> 8));
    sd_mock.gap = 20;

    std::vector<uint8_t> data;
    if (!rom_flow_read6(2, 3, data)) return false;

    CHECK_EQ("got 1536 bytes", (uint32_t)data.size(), 3 * 512);
    for (int i = 0; i < 3 * 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, data[i], sd_mock.read_sector[i]);
            return false;
        }
    }
    CHECK_EQ("sd_cmd_type was CMD18 (2)", sd_mock.last_cmd_type, 2);
    CHECK_EQ("sd_lba was 8194 (LBA 2 → 8192+2)",
             sd_mock.last_lba, SD_RAW_BASE_LBA + 2);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 6 — golden ROM flow, long CMD18 read with a VBL-length drain
// stall landing INSIDE a chunk's TC0→drain window (the 2026-07-15 HW
// wedge at ROM PC 0x40899322).
//
// HW evidence: boot reached the desktop (hundreds of clean transfers),
// then froze at the post-drain INTR poll (pc_live 0x40899322/28) with
// exc_count climbing (VBL ISRs still firing) and tcounter LSB == 0 over
// JTAG.  Mechanism: sd_ctrl cannot be paused, so during an ISR-stalled
// drain the SPI stream keeps filling the 512-byte CMD18 ring —
// overwriting undrained bytes past 512 of backlog (corruption) and
// wrapping the 10-bit sd_buf_count at 1024 (0x400 backlog reads as 0).
// A wrapped-to-~0 count during the ROM's 16 blind DMA-port reads drops
// t_req mid-drain, the remaining reads stop counting as beats,
// c96_xfr_left never reaches 0, and the chunk-completion I_BUS never
// fires — the ROM polls status bit 7 forever.
//
// The stall lands after TC0 is seen for a chunk, with the fill sized so
// sd_buf_count sits just under 1024 when the 16 blind reads start.
// Pre-fix this wedges exactly like the HW; post-fix (real rd_ready
// back-pressure bounding the ring) it completes byte-identical.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_rom_flow_cmd18_vbl_stall_ring_overrun() {
    reset();
    const int blocks = 8;                     // 4 KiB — a System-file read
    sd_mock.read_sector.resize(blocks * 512);
    for (int i = 0; i < blocks * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x37 ^ (i & 0xFF) ^ (i >> 8));
    sd_mock.gap = 20;                         // ~real SPI byte pacing

    // Stall sized so the fill runs ~1020 bytes ahead: with gap=20 the
    // mock delivers one byte per 21 cycles.  1020*21 = 21420 cycles.
    std::vector<uint8_t> data;
    if (!rom_flow_read6(16, blocks, data,
                        /*stall_after_bytes=*/512,
                        /*stall_cycles=*/1020 * 21,
                        /*stall_before_drain=*/true)) return false;

    CHECK_EQ("got 4096 bytes", (uint32_t)data.size(), blocks * 512);
    for (int i = 0; i < blocks * 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x"
                        " (ring overwrite)\n",
                        i, data[i], sd_mock.read_sector[i]);
            return false;
        }
    }
    CHECK_EQ("sd_cmd_type was CMD18 (2)", sd_mock.last_cmd_type, 2);
    return true;
}

// Same overrun class, stall landing BETWEEN chunks (count wraps during
// the stall itself): pre-fix this starves acceptance ~1024 bytes early
// and wedges at the ROM's TC0 poll (0x408992d2) instead.
static bool test_c96_rom_flow_cmd18_vbl_stall_between_chunks() {
    reset();
    const int blocks = 8;
    sd_mock.read_sector.resize(blocks * 512);
    for (int i = 0; i < blocks * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x91 ^ (i & 0xFF) ^ (i >> 7));
    sd_mock.gap = 20;

    std::vector<uint8_t> data;
    if (!rom_flow_read6(24, blocks, data,
                        /*stall_after_bytes=*/512,
                        /*stall_cycles=*/1100 * 21,
                        /*stall_before_drain=*/false)) return false;

    CHECK_EQ("got 4096 bytes", (uint32_t)data.size(), blocks * 512);
    for (int i = 0; i < blocks * 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x"
                        " (ring overwrite)\n",
                        i, data[i], sd_mock.read_sector[i]);
            return false;
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test — realistic bus-access latency during the blind 16-byte drain
// burst (the actual 2026-07-15 HW wedge mechanism, confirmed by a
// fresh golden MAME capture of DAFB +0x24: the ROM programs ctrl=
// 0x1ec — bit 7 (read DRQ-check) IS set — and does exactly ONE
// software DRQ poll (0x408992f6-0x408992fc, btst #9) before the
// blind 8-word burst, never rechecking per-word).
//
// The first VBL-stall repro (test_c96_rom_flow_cmd18_vbl_stall_*)
// passed sim but did NOT reproduce on hardware — the coordinator's
// HW re-test showed the identical hang after that fix landed.
// Root cause of the sim/HW gap: shim_r() costs exactly one tick() in
// this harness, so the 16-byte blind burst here completes in ~16
// core cycles — far faster than a gap-paced CMD18 background SD
// refill can interleave with it.  On real hardware each blind
// DMA-port read costs several core_clk cycles (AXI xbar +
// peripheral_bus handshake + DAFB CDC round trip), stretching the
// burst enough for the background ring refill to run concurrently.
//
// During that stretched window, pre-fix RTL's pseudo_dma_in_beat /
// drq_c96 / pb_rd_mux for the DMA shim all required literal t_req —
// the bare-5380 REQ line, whose CMD18 assignment is recomputed after
// EVERY drained beat from the ring's CURRENT refill state, not from
// whether THIS chunk's already-chip-accepted bytes (accept_pend) are
// staged.  A t_req glitch mid-burst (from ordinary background ring
// bookkeeping, no VBL/ISR/overrun needed at all) silently dropped a
// beat: stale data selected, c96_xfr_left not decremented, but the
// bus cycle still completed (ack unconditional) — so the CPU sailed
// on to the INTR poll at 0x40899322 with c96_xfr_left != 0 forever,
// matching the exact HW symptom (frozen PC past the burst, TC0
// still set, INTR never fires).
//
// This test reproduces that: realistic per-read bus latency plus
// ordinary (non-stalled) slow SD pacing — no VBL stall needed.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_rom_flow_cmd18_realistic_bus_latency() {
    reset();
    const int blocks = 6;
    sd_mock.read_sector.resize(blocks * 512);
    for (int i = 0; i < blocks * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x5D ^ (i & 0xFF) ^ (i >> 6));
    sd_mock.gap = 20;                 // real SPI byte pacing

    std::vector<uint8_t> data;
    if (!rom_flow_read6(48, blocks, data,
                        /*stall_after_bytes=*/-1,
                        /*stall_cycles=*/0,
                        /*stall_before_drain=*/false,
                        /*bus_latency_cycles=*/40)) return false;

    CHECK_EQ("got 3072 bytes", (uint32_t)data.size(), blocks * 512);
    for (int i = 0; i < blocks * 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x"
                        " (dropped/stale beat under realistic bus"
                        " latency)\n",
                        i, data[i], sd_mock.read_sector[i]);
            return false;
        }
    }
    CHECK_EQ("sd_cmd_type was CMD18 (2)", sd_mock.last_cmd_type, 2);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 7 — the SECOND golden-trace drain flow (driver-era, ROM
// 0x408995e2..0x40899664): tcount=512, ONE DMA|CI_XFER per 512-byte
// block, then 512 blind DMA-port reads with NO TC0 poll and NO
// per-chunk DAFB DRQ re-check — pacing relies on the DAFB DTACK
// holdoff, which the tb models by spinning on the scsi drq output
// before each read.  Per-block completion contract is the same I_BUS
// (golden trace seq 20782-20784: status 0x91 → istatus 0x10 after the
// 256th word read).
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_driver_flow_block_burst_cmd18() {
    reset();
    const int blocks = 3;
    sd_mock.read_sector.resize(blocks * 512);
    for (int i = 0; i < blocks * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x6B ^ (i & 0xFF) ^ (i >> 8));
    sd_mock.gap = 20;

    // Select with empty FIFO + deferred CDB, as the ROM does.
    reg_w(0x4, TARGET_ID);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    uint8_t v = 0;
    for (int i = 0; i < 64; ++i) { v = reg_r(0x6); if (v & 7) break; }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", (v & 7) != 0);
    const uint8_t lba = 40;
    reg_w(0x2, 0x08);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, lba);
    reg_w(0x2, (uint8_t)blocks);
    shim_w(0x00);
    for (int i = 0; i < 64000; ++i) { v = reg_r(0x4); if (v & 0x80) break; }
    CHECK_EQ("post-select status = INTR|TC0|DATA_IN", v, 0x91);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    std::vector<uint8_t> data;
    for (int blk = 0; blk < blocks; ++blk) {
        reg_w(0x0, 0x00);           // tcount = 512
        reg_w(0x1, 0x02);
        reg_w(0x3, 0x90);           // DMA | CI_XFER — one per block
        // 512 blind reads, each paced by the DAFB DTACK holdoff (drq).
        for (int i = 0; i < 512; ++i) {
            bool drq_up = false;
            for (int spin = 0; spin < 20000; ++spin) {
                if (dut->drq) { drq_up = true; break; }
                tick_n(1);
            }
            CHECK_TRUE("DRQ rises for each blind driver-flow read",
                       drq_up);
            data.push_back(shim_r());
        }
        // ROM 0x40899706: status bit7 poll, then istatus == I_BUS.  The
        // ROM's poll is UNTIMED; this budget must cover the 2026-09-07
        // pre-staging behaviour, where the per-chunk I_BUS additionally
        // waits for the ring to re-stage (~RING_HIGH_WATER bytes at the
        // provider's pace — here gap=20, ~10k ticks) so the next blind
        // burst starts with full headroom instead of stalling mid-burst.
        bool intr = false;
        for (int i = 0; i < 50000; ++i) {
            v = reg_r(0x4);
            if (v & 0x80) { intr = true; break; }
        }
        CHECK_TRUE("I_BUS interrupt after 512-byte block burst", intr);
        CHECK_EQ("block istatus = I_BUS", reg_r(0x5), I_BUS);
    }

    CHECK_EQ("phase = STATUS after final block", reg_r(0x4) & 7, 0x3);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("status = GOOD", reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);

    CHECK_EQ("got 1536 bytes", (uint32_t)data.size(), blocks * 512);
    for (int i = 0; i < blocks * 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, data[i], sd_mock.read_sector[i]);
            return false;
        }
    }
    CHECK_EQ("sd_cmd_type was CMD18 (2)", sd_mock.last_cmd_type, 2);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test — System 7.5.3 driver flow: the 16-byte chunked data-in drain is
// gated on FIFO Flags (offset 7) bit 4, i.e. "FIFO holds >= 16 bytes".
//
// HW repro, bitstream 0xE74B330C (2026-08-01).  7.5.3's SCSI driver
// runs this loop at 0x40592..0x405C8 with A3 = 0x50F0F000:
//
//   0x40592  BTST #4,$40(A3)      ; STAT bit 4 = TC
//   0x40598  BNE.S 0x405B8
//   ...
//   0x405B8  MOVE.L (A0),D5       ; DAFB +0x24
//   0x405BE  BTST D0,D5           ; bit 9 = DRQ
//   0x405C0  BEQ.S 0x40592
//   0x405C2  BTST #4,$70(A3)      ; FIFO Flags bit 4 == count >= 16
//   0x405C8  BEQ.S 0x40592        ; <-- spun here forever
//   0x405CC  MOVE.W $100(A1),(A2)+   x8   ; 16-byte pseudo-DMA drain
//
// Our C96 DMA data-in path bypasses c96_fifo (bytes are counted by
// c96_accept_pend and served from sec_buf), so offset 7 read 0 for the
// whole transfer and that gate never opened.  MAME's chip pushes every
// received DATA IN byte into the FIFO that offset 7 reports
// (ncr53c90.cpp:471 recv_byte → fifo_push, :1141 fifo_flags_r), so it
// reaches 16 and the drain proceeds.
//
// The Q700 ROM / System 7.0.1 driver polls only STAT + the DAFB DRQ
// bit and never reads offset 7 — which is why this stayed latent.
//
// Pre-fix this test hangs the gate and fails at "FIFO Flags bit 4
// sets"; post-fix it drains byte-identical data.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_fifo_flags_gate_753_chunked_drain() {
    reset();

    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i) {
        sd_mock.read_sector[i] = (uint8_t)(0x5A ^ (i & 0xFF) ^ (i >> 3));
    }

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);

    const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    // Regression guard for the non-data-in uses of offset 7: the CDB
    // FIFO count must still be reported from c96_fifo_pos.
    CHECK_EQ("FIFO Flags reports CDB byte count", reg_r(0x7), 0x06);

    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);           // tcount = 512
    // ATN_STOP with a preloaded CDB now halts after one message byte
    // (MAME contract, scsi_fuzz finding F2) — use the bare DMA select,
    // which consumes the whole CDB, for these end-to-end data flows.
    reg_w(0x3, 0x80 | CD_SELECT);

    CHECK_TRUE("select-complete interrupt fires", wait_intr(8000));
    CHECK_EQ("select-complete phase = DATA IN", reg_r(0x4) & 0x07, 0x01);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    // 7.5.3 moves the 512-byte block as 32 chunks of 16 bytes.  Mirrors
    // the MAME macqd700 register trace of the same driver (7.5.3 booting
    // to the Finder), which writes TCLO once and then re-issues 0x90 per
    // chunk WITHOUT rewriting the transfer count — every command with
    // bit 7 set reloads tcounter from the reg0/reg1 latch and clears TC0
    // (MAME load_tcounter(), ncr53c90.cpp:918):
    //
    //   800c47c0  W r0 = 10        TC = 16, written ONCE
    //   800c47d2  W r3 = 90        DMA | TRANSFER INFORMATION
    //   800c47da  R r4 = 0x01      DATA IN, TC0 clear, no INT
    //   800c78d2  R r4 = 0x11      TC0 set, DATA IN, still NO INT
    //   800c47fe  R r7 = 0x10      FIFO FLAGS = 16
    //   800c4808  R DMA window x8  8 x 16-bit = 16 bytes
    //   800c7aac  R r4 = 0x91      INT asserts HERE and only here
    //   800c7ac0  R r5 = 0x10      INTR = BUS SERVICE
    //   800c47d2  W r3 = 90        next chunk
    reg_w(0x0, 0x10);           // tcount = 16 — written once, per MAME
    reg_w(0x1, 0x00);

    std::vector<uint8_t> data;
    for (int chunk = 0; chunk < 32; ++chunk) {
        reg_w(0x3, 0x90);       // DMA | CI_XFER; reloads TC, clears TC0

        // Spin for TC0 (STAT bit 4).  MAME sees exactly 0x11 here for
        // chunks 0..30: TC0 + DATA IN phase and — critically — INT
        // still CLEAR.  Completion is deferred until the host drains
        // the FIFO, so a chip that raised INT on terminal count would
        // be wrong.
        //
        // The LAST chunk is different: its 16th byte is the block's
        // 512th, and the target releases DATA IN and drives STATUS the
        // moment that byte is ACKed BY THE CHIP — before the host has
        // drained the FIFO.  Measured 2026-08-19 against MAME 0.285
        // with a byte-identical differential script (32 chunks of
        // W3=90 / RC4 / RC7 / 16 beats / RC4 / RC5): MAME reads 0x11
        // pre-drain on chunks 0..30 and 0x13 (TC0 | STATUS) on chunk
        // 31, INT still deferred, then 0x93 + I_BUS after the drain.
        // The previous blanket 0x11 expectation encoded the old
        // accept_pend visibility shim (which held the back-end in
        // DATA IN until the CPU drained) — golden-divergent on the
        // final chunk.
        bool tc0 = false;
        for (int spin = 0; spin < 20000; ++spin) {
            uint8_t s = reg_r(0x4);
            if (s & 0x10) { tc0 = true; break; }
            tick_n(1);
        }
        CHECK_TRUE("TC0 sets on counter exhaustion", tc0);
        if (chunk < 31) {
            CHECK_EQ("STAT = 0x11 (TC0 | DATA IN, INT deferred)",
                     reg_r(0x4), 0x11);
        } else {
            CHECK_EQ("STAT = 0x13 (TC0 | STATUS, INT deferred; last "
                     "chunk — target released DATA IN on final ACK)",
                     reg_r(0x4), 0x13);
        }

        // 0x405C2: BTST #4,$70(A3) — the gate that hung on hardware.
        // MAME answers 0x10; pre-fix we answered 0x00 forever.  Bare
        // count, no sequence step in bits 7:5 (that is reg 6 only).
        uint8_t flags = reg_r(0x7);
        if (flags != 0x10) {
            std::printf("  FAIL chunk %d: FIFO Flags = 0x%02x, expected "
                        "0x10 (STAT = 0x%02x, drq = %d) — 7.5.3's drain "
                        "gate never opens\n",
                        chunk, flags, reg_r(0x4), (int)dut->drq);
            return false;
        }
        CHECK_TRUE("DRQ asserted while the FIFO holds data", dut->drq);

        // 0x405CC: eight MOVE.W $100(A1),(A2)+ = 16 bytes off the
        // pseudo-DMA port.  The drain is what releases completion.
        for (int i = 0; i < 16; ++i) data.push_back(shim_r());

        // 0x405EE: BTST #7,$40(A3) — INT asserts only now.
        bool intr = false;
        for (int spin = 0; spin < 4000; ++spin) {
            if (reg_r(0x4) & S_INTR) { intr = true; break; }
            tick_n(1);
        }
        CHECK_TRUE("completion interrupt fires once the FIFO is drained",
                   intr);
        // 0x405F6: MOVE.B $50(A3),D5 — INTR = BUS SERVICE.
        CHECK_EQ("istatus = I_BUS (BUS SERVICE)", reg_r(0x5), I_BUS);
    }

    CHECK_EQ("drained the full 512-byte block", (uint32_t)data.size(), 512u);
    for (int i = 0; i < 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, data[i], sd_mock.read_sector[i]);
            return false;
        }
    }

    // Once the data phase is done the FIFO-flags override must release:
    // CI_COMPLETE stages status+msg in the real FIFO and offset 7 has
    // to report those two bytes, not a stale data-in count.
    // The last chunk's completion I_BUS and the DATA_IN→STATUS
    // phase-change I_BUS are the same interrupt (both OR bit 4 into
    // istatus), and the chunk loop above already acked it — so poll the
    // phase bits rather than waiting for a second interrupt.
    bool at_status = false;
    for (int spin = 0; spin < 4000; ++spin) {
        if ((reg_r(0x4) & 0x07) == 0x03) { at_status = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("phase leaves DATA IN for STATUS after the last chunk",
               at_status);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("FIFO Flags reports status+msg after data phase",
             reg_r(0x7), 0x02);
    CHECK_EQ("status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// INQUIRY driven the way the System 7.5.3 SCSI Manager actually drives
// it on hardware — replayed byte-for-byte from the live 53C96 trace ring
// (tb/vectors/hw_scsi_trace_753_wedge.csv, entries 4074..4095):
//
//     W4=00                       dest bus ID
//     W2 x6  12 00 00 00 24 00    INQUIRY CDB into the FIFO
//     W3=41                       CD_SELECT, no ATN, **non-DMA form**
//     R4=91  R6=04  R3=41  R7=00  R5=18   select complete, DATA IN
//     W1=00 W0=10                 TC latch = 16
//     W3=90                       DMA | CI_XFER, 16-byte chunk
//     ... 16 pseudo-DMA beats ...
//     (x2, then four per-byte non-DMA 0x10 Transfer Infos popped at R2
//      for the 4-byte tail — 36 = 16 + 16 + 4)
//
// No existing tb drove this shape: tb_scsi_c96_inquiry.cpp arms INQUIRY
// with tcount=36 (the WHOLE transfer) behind a SELECT_ATN_STOP, which
// never exercises the chunked TC=16 / plain-0x41 combination.
// ─────────────────────────────────────────────────────────────────────
static bool inquiry_flow_tc16(std::vector<uint8_t>& out) {
    uint8_t v = 0;

    reg_w(0x4, TARGET_ID);              // W4 — destination bus ID
    reg_w(0x2, 0x12);                   // INQUIRY
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x24);                   // allocation length = 36
    reg_w(0x2, 0x00);
    reg_w(0x3, 0x41);                   // CD_SELECT (no ATN), non-DMA

    for (int i = 0; i < 64000; ++i) { v = reg_r(0x4); if (v & 0x80) break; }
    // TC0 (bit 4) is deliberately masked out: the HW trace reads 0x91
    // only because S_TC0 was left sticky by the PREVIOUS transaction (a
    // tcount-carrying select), and this select carries no tcount of its
    // own.  On a freshly reset chip the same select reports 0x81.
    CHECK_EQ("post-select status = INTR|DATA_IN", v & 0x87, 0x81);
    CHECK_EQ("seq_step = 4",            reg_r(0x6) & 0x7, 0x4);
    CHECK_EQ("command readback = 0x41", reg_r(0x3), 0x41);
    CHECK_EQ("fifo drained after CDB send", reg_r(0x7), 0x00);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    out.clear();
    // TC latch written ONCE, then one 0x90 per chunk with nothing in
    // between but status polling — no FLUSH_FIFO, no TC rewrite.  That
    // is the load-bearing shape of the MAME golden trace; see the header
    // of tb/vectors/scsi96_mame_q700_753_tc16.csv.
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x10);                   // TC = 16
    // Two TC=16 DMA chunks (32 of the 36 bytes).
    for (int chunk = 0; chunk < 2; ++chunk) {
        reg_w(0x3, 0x90);               // DMA | CI_XFER
        bool tc0 = false;
        for (int i = 0; i < 200000; ++i) {
            v = reg_r(0x4);
            if (v & 0x10) { tc0 = true; break; }
            if (v & 0x80) break;        // an interrupt here is the error path
        }
        if (!tc0) {
            std::printf("  FAIL chunk %d: TC0 never set — status=0x%02x "
                        "cmd=0x%02x tc=0x%02x%02x fifo=0x%02x istat=0x%02x\n",
                        chunk, v, reg_r(0x3), reg_r(0x1), reg_r(0x0),
                        reg_r(0x7), reg_r(0x5));
            return false;
        }
        CHECK_EQ("fifo flags report the staged 16-byte chunk",
                 reg_r(0x7), 0x10);
        for (int i = 0; i < 16; ++i) out.push_back(shim_r());
        bool intr = false;
        for (int i = 0; i < 4000; ++i) {
            v = reg_r(0x4);
            if (v & 0x80) { intr = true; break; }
        }
        CHECK_TRUE("I_BUS interrupt after the chunk drain", intr);
        CHECK_EQ("chunk istatus = I_BUS", reg_r(0x5), I_BUS);
    }

    // 4-byte tail: four per-byte non-DMA Transfer Infos, popped at reg 2.
    // MAME ncr53c90.cpp:650 "non-dma in: every byte" → bus_complete().
    for (int i = 0; i < 4; ++i) {
        reg_w(0x3, 0x10);               // CI_XFER, non-DMA
        bool intr = false;
        for (int k = 0; k < 4000; ++k) {
            v = reg_r(0x4);
            if (v & 0x80) { intr = true; break; }
        }
        CHECK_TRUE("I_BUS after a per-byte non-DMA Transfer Info", intr);
        CHECK_EQ("non-DMA tail istatus = I_BUS", reg_r(0x5), I_BUS);
        CHECK_EQ("non-DMA tail staged exactly one FIFO byte",
                 reg_r(0x7), 0x01);
        out.push_back(reg_r(0x2));
    }

    bool at_status = false;
    for (int spin = 0; spin < 4000; ++spin) {
        if ((reg_r(0x4) & 0x07) == 0x03) { at_status = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("phase leaves DATA IN for STATUS after the 36th byte",
               at_status);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    return true;
}

// "APPLE   " / "HD SC ..." / "0001" — canned_byte(CANNED_INQUIRY, ...),
// rtl/mac/scsi.v:666-687.
static bool check_inquiry_payload(const std::vector<uint8_t>& d) {
    CHECK_EQ("36 INQUIRY bytes", (uint32_t)d.size(), 36u);
    CHECK_EQ("peripheral device type = DIRECT ACCESS", d[0], 0x00);
    CHECK_EQ("SCSI-2",             d[2], 0x02);
    CHECK_EQ("additional length",  d[4], 31);
    const char* vendor = "APPLE   ";
    for (int i = 0; i < 8; ++i) {
        if (d[8 + i] != (uint8_t)vendor[i]) {
            std::printf("  FAIL vendor[%d]: got 0x%02x ('%c'), expected '%c'\n",
                        i, d[8 + i], d[8 + i], vendor[i]);
            return false;
        }
    }
    CHECK_EQ("revision[0]", d[32], '0');
    CHECK_EQ("revision[3]", d[35], '1');
    return true;
}

// CONTROL — the same TC=16 / 0x41 INQUIRY on a freshly reset chip, with
// no preceding transaction.  This one passes both before and after the
// fix; it is here so a failure of the test below can only be read as
// "the PRECEDING multi-block read poisoned the chip", not as "chunked
// INQUIRY is broken in general".
static bool test_c96_inquiry_tc16_standalone() {
    reset();
    std::vector<uint8_t> data;
    if (!inquiry_flow_tc16(data)) return false;
    return check_inquiry_payload(data);
}

// ─────────────────────────────────────────────────────────────────────
// THE 7.5.3 "Starting up..." WEDGE (HW-measured, bitstream 0xDD5127F3).
//
// Exactly the hardware ordering: a multi-block READ(6) through the CMD18
// ring, drained to completion, and THEN the OS re-scans the bus with
// INQUIRY.  vh_req_multi (rtl/mac/scsi.v:151) is assigned only in the
// READ/WRITE dispatch arms (:2336-2428) and is never cleared by a
// canned-payload command, so the INQUIRY runs with vh_multi_read still 1
// — and c96_avail_bytes (:1109) then reads the drained multi-block ring
// occupancy (0) instead of xfer_bytes_left (36).  c96_accept_ev (:1112)
// can never fire, tcounter never leaves 16, S_TC0 never sets, the DMA
// chunk-completion hook never raises I_BUS, and the interrupt-driven
// SCSI Manager waits forever.
//
// RED before the fix, at the first chunk of the INQUIRY only — the
// READ(6) ahead of it completes normally, matching the trace ring.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_inquiry_tc16_after_multiblock_read() {
    reset();
    sd_mock.read_sector.resize(3 * 512);
    for (int i = 0; i < 3 * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x3C ^ (i & 0xFF) ^ (i >> 8));
    sd_mock.gap = 4;

    std::vector<uint8_t> blk;
    if (!rom_flow_read6(4, 3, blk)) return false;
    CHECK_EQ("multi-block read still delivers every byte",
             (uint32_t)blk.size(), 3u * 512u);
    for (int i = 0; i < 3 * 512; ++i) {
        if (blk[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL read byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, blk[i], sd_mock.read_sector[i]);
            return false;
        }
    }

    // …and now the bus re-scan that hardware dies on.
    std::vector<uint8_t> data;
    if (!inquiry_flow_tc16(data)) return false;
    return check_inquiry_payload(data);
}

// Same sticky-state poison, non-DMA form: after a multi-block read,
// c96_nondma_supply (rtl/mac/scsi.v:1061) reads the same drained ring
// count, so even the per-byte 0x10 Transfer Info path stalls.  Driven
// with 0x10 from the very first byte so the failure cannot be blamed on
// the DMA accept path.
static bool test_c96_inquiry_nondma_after_multiblock_read() {
    reset();
    // MUST be a multi-block read: the single-block arm (rtl/mac/scsi.v
    // :2380) already assigns vh_req_multi <= 0, so a 1-block read leaves
    // nothing stale behind and this scenario would pass for the wrong
    // reason.  2 blocks takes the CMD18 arm (:2386).
    sd_mock.read_sector.resize(2 * 512);
    for (int i = 0; i < 2 * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0xA5 ^ (i & 0xFF) ^ (i >> 8));

    std::vector<uint8_t> blk;
    if (!rom_flow_read6(0, 2, blk)) return false;

    uint8_t v = 0;
    reg_w(0x4, TARGET_ID);
    reg_w(0x2, 0x12); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x24); reg_w(0x2, 0x00);
    reg_w(0x3, 0x41);
    for (int i = 0; i < 64000; ++i) { v = reg_r(0x4); if (v & 0x80) break; }
    CHECK_EQ("post-select status = INTR|TC0|DATA_IN", v, 0x91);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    std::vector<uint8_t> data;
    for (int i = 0; i < 36; ++i) {
        reg_w(0x3, 0x10);               // non-DMA CI_XFER, one byte
        bool intr = false;
        for (int k = 0; k < 20000; ++k) {
            v = reg_r(0x4);
            if (v & 0x80) { intr = true; break; }
        }
        if (!intr) {
            std::printf("  FAIL byte %d: non-DMA Transfer Info never "
                        "completed — status=0x%02x cmd=0x%02x fifo=0x%02x\n",
                        i, v, reg_r(0x3), reg_r(0x7));
            return false;
        }
        CHECK_EQ("non-DMA istatus = I_BUS", reg_r(0x5), I_BUS);
        CHECK_EQ("non-DMA staged exactly one FIFO byte", reg_r(0x7), 0x01);
        data.push_back(reg_r(0x2));
    }
    return check_inquiry_payload(data);
}

// ═════════════════════════════════════════════════════════════════════
// NON-DMA TRANSFER INFORMATION IN A **DATA OUT** PHASE
//
// HW-MEASURED (2026-08-07, bitstream 0xC74E3489, 53C96 trace ring).  The
// 7.5.3 boot reaches ~85% / six extensions and then retries ONE
// transaction forever — 4096+ ring events in 70 s, every retry ending on
// exactly this register sequence:
//
//     R4=90  R3=90  R8=47  W8=c7  W2=ee  W3=10  R4=10
//
// i.e. status shows INTR|TC0|phase 000 (DATA OUT), the previous command
// echo is 0x90 (DMA|CI_XFER), the driver stages ONE byte in the FIFO
// (W2) and issues a NON-DMA Transfer Information (W3=0x10) — and the
// chip then sits at status 0x10 (TC0 | DATA OUT, INTR clear) forever.
//
// MECHANISM (read directly out of the RTL, not inferred): the FIFO-drain
// side of a non-DMA transfer only exists for data IN.
//   * rtl/mac/scsi.v c96_fifo_fill_beat is gated on `phase == S_DATA_IN`.
//   * the only consumers of c96_fifo[] are the reg-2 read pop, the
//     CDB-assembly select path and the CI_COMPLETE status/msg preload —
//     grep the file: nothing sends a FIFO byte to the target.
//   * so with c96_xfr_armed=1, c96_xfr_dma=0, phase==S_DATA_OUT, every
//     completion hook misses: the non-DMA hook needs c96_fifo_fill_beat
//     (DATA IN only), the phase-change hook needs phase==S_STATUS (the
//     phase can never advance because nothing consumes the byte), and
//     the DMA chunk hook needs c96_xfr_dma.  Dead end, no I_BUS ever.
//
// MAME's counterpart (thirdparty/mame/src/devices/machine/ncr53c90.cpp):
//   :601-616  INIT_XFR / S_PHASE_DATA_OUT → "can't send if the fifo is
//             empty" (fifo_pos==0 → break) else send_byte()
//   :749-763  send_byte() → m_scsi_bus->data_w(..., fifo_pop())
//   :649      INIT_XFR_WAIT_REQ:
//             `|| (!dma_command && (xfr_phase & S_INP) == 0 &&
//                  fifo_pos == 0)   // non-dma out: fifo empty`
//             → INIT_XFR_BUS_COMPLETE
//   :686-692  INIT_XFR_BUS_COMPLETE → bus_complete()
//   :792-799  bus_complete(): `state = IDLE; istatus |= I_BUS;`
//   :1234-1237 decrement_tcounter() returns immediately when
//             `!dma_command` — a non-DMA transfer does NOT touch
//             tcounter, in either direction.
// So: drain the WHOLE FIFO (one byte per REQ, looping) and complete with
// I_BUS the moment the FIFO is empty — including the degenerate case of
// a 0x10 issued with an already-empty FIFO.
//
// MODE SELECT(6) (rtl/mac/scsi.v cdb[0]==0x15) is used below because it
// is the shortest command in the target model that parks the bus in
// DATA OUT without involving the backing store, so these scenarios
// cannot be confounded by SD write timing.
// ═════════════════════════════════════════════════════════════════════

// Select the target with a MODE SELECT(6) carrying `param_len` bytes of
// parameter list, and leave the chip sitting in DATA OUT.
static bool modesel_select_data_out(int param_len) {
    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    const uint8_t cdb[6] = {0x15, 0x00, 0x00, 0x00,
                            (uint8_t)param_len, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    CHECK_EQ("FIFO has 6 bytes (MODE SELECT CDB)", reg_r(0x7), 0x06);
    reg_w(0x3, 0x41);                 // CD_SELECT_ATN, non-DMA select
    CHECK_TRUE("MODE SELECT select-complete fires", wait_intr(20000));
    // INTR | no TC0 (non-DMA command zeroes tcounter without setting the
    // sticky) | phase 000 = DATA OUT.
    CHECK_EQ("post-select status = INTR|DATA_OUT", reg_r(0x4), 0x80);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    return true;
}

// Close out a MODE SELECT that has reached STATUS.
static bool modesel_finish() {
    CHECK_EQ("phase = STATUS after parameter list", reg_r(0x4) & 0x07, 0x03);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    return true;
}

// ── STANDALONE CONTROL ───────────────────────────────────────────────
// The SAME MODE SELECT(6), parameter list moved with the DMA form of
// Transfer Information (0x90) through the pseudo-DMA shim.  That path
// (pseudo_dma_out_beat) already exists, so this scenario passes BOTH
// before and after the fix.  It pins down the select flow, the MODE
// SELECT dispatch, the DATA OUT phase encoding and the STATUS/MSG
// close-out, so a RED in the non-DMA scenarios below can only be read
// one way: the non-DMA data-out send path is missing.
static bool test_c96_modesel_dma_data_out_control() {
    reset();
    if (!modesel_select_data_out(4)) return false;

    reg_w(0x0, 0x04);                 // tcount = 4
    reg_w(0x1, 0x00);
    reg_w(0x3, 0x80 | CI_XFER);       // DMA | CI_XFER
    const uint8_t param[4] = {0xee, 0x00, 0x00, 0x08};
    for (int i = 0; i < 4; ++i) shim_w(param[i]);

    CHECK_TRUE("DMA data-out transfer completes", wait_intr(4000));
    CHECK_EQ("DMA data-out istatus = I_BUS", reg_r(0x5), I_BUS);
    return modesel_finish();
}

// ── RED before the fix ───────────────────────────────────────────────
// The measured signature, byte for byte: stage one byte at reg 2, issue
// 0x10, wait for the interrupt.  Pre-fix this never fires and the loop
// below reports the exact chip state the trace ring showed.
static bool test_c96_modesel_nondma_data_out_753_wedge() {
    reset();
    if (!modesel_select_data_out(4)) return false;

    const uint8_t param[4] = {0xee, 0x00, 0x00, 0x08};
    for (int i = 0; i < 4; ++i) {
        reg_w(0x2, param[i]);                       // W2 = <byte>
        CHECK_EQ("FIFO staged the parameter byte", reg_r(0x7), 0x01);
        reg_w(0x3, CI_XFER);                        // W3 = 0x10
        bool intr = false;
        uint8_t v = 0;
        for (int k = 0; k < 20000; ++k) {
            v = reg_r(0x4);
            if (v & 0x80) { intr = true; break; }
        }
        if (!intr) {
            std::printf("  FAIL param byte %d: non-DMA Transfer Info in "
                        "DATA OUT never completed — status=0x%02x "
                        "cmd=0x%02x fifo=0x%02x\n",
                        i, v, reg_r(0x3), reg_r(0x7));
            return false;
        }
        CHECK_EQ("non-DMA data-out istatus = I_BUS", reg_r(0x5), I_BUS);
        CHECK_EQ("non-DMA data-out drained the FIFO", reg_r(0x7), 0x00);
    }
    return modesel_finish();
}

// ── RED before the fix ───────────────────────────────────────────────
// MAME sends the WHOLE FIFO on one non-DMA Transfer Information in an
// out phase (INIT_XFR → send_byte → INIT_XFR_WAIT_REQ → fifo_pos != 0 →
// INIT_XFR again), completing only on "non-dma out: fifo empty"
// (:649).  This is deliberately NOT the one-byte-per-command shape of
// the data-IN path (:650, "non-dma in: every byte"), and a fix that
// simply mirrored data-in would fail here.
static bool test_c96_nondma_data_out_drains_whole_fifo() {
    reset();
    if (!modesel_select_data_out(4)) return false;

    reg_w(0x2, 0x11);
    reg_w(0x2, 0x22);
    reg_w(0x2, 0x33);
    CHECK_EQ("FIFO staged three bytes", reg_r(0x7), 0x03);
    reg_w(0x3, CI_XFER);
    CHECK_TRUE("one 0x10 drains the whole FIFO", wait_intr(20000));
    CHECK_EQ("istatus = I_BUS", reg_r(0x5), I_BUS);
    CHECK_EQ("FIFO empty after the drain", reg_r(0x7), 0x00);
    // 3 of 4 parameter bytes moved: still in DATA OUT, one byte owed.
    CHECK_EQ("still in DATA OUT with one byte owed", reg_r(0x4) & 0x07,
             0x00);
    // A non-DMA transfer must NOT touch tcounter (ncr53c90.cpp:1234-1237
    // returns early when !dma_command), so TC0 must still be clear.
    CHECK_EQ("non-DMA transfer left TC0 clear", reg_r(0x4) & 0x10, 0x00);

    reg_w(0x2, 0x44);
    reg_w(0x3, CI_XFER);
    CHECK_TRUE("final parameter byte completes", wait_intr(20000));
    CHECK_EQ("istatus = I_BUS", reg_r(0x5), I_BUS);
    return modesel_finish();
}

// ── RED before the 2026-08-19 correction, GREEN after ────────────────
// RECALIBRATED WITH DIFFERENTIAL EVIDENCE (scsi_fuzz seed 27 vs MAME
// 0.285).  This scenario used to assert that an empty-FIFO 0x10 in an
// out phase COMPLETES with I_BUS, citing ncr53c90.cpp:608-610 as "the
// send is skipped, the state machine still falls through to
// INIT_XFR_WAIT_REQ".  It does not fall through: that `break` leaves
// step() with state == INIT_XFR_SEND_BYTE and NO delay timer armed, so
// INIT_XFR_WAIT_REQ — and the ":649 non-dma out: fifo empty" completion
// — are never reached.  MAME PARKS with no interrupt.  Measured
// byte-identically on a minimal script (ATN_STOP select, DMA CI_XFER
// drain, then a bare `W 3 10` with an empty FIFO):
//     rtl (old) SYNC 3 stat=92 istat=10 irq=1
//     mame      SYNC 3 stat=12 istat=00 irq=0
// and inserting a single `W 2 aa` before the 0x10 made the two sides
// agree exactly — which is what the second half of this test now pins.
static bool test_c96_nondma_data_out_empty_fifo_parks() {
    reset();
    if (!modesel_select_data_out(4)) return false;

    CHECK_EQ("FIFO empty before the command", reg_r(0x7), 0x00);
    reg_w(0x3, CI_XFER);
    CHECK_TRUE("empty-FIFO 0x10 in DATA OUT raises NO interrupt",
               !wait_intr(20000));
    CHECK_EQ("istatus still clear", reg_r(0x5), 0x00);
    CHECK_EQ("no byte moved — still in DATA OUT", reg_r(0x4) & 0x07, 0x00);

    // ...and a host FIFO write unparks it: MAME's fifo_w() (:869-877)
    // ends in step(false), which walks INIT_XFR_SEND_BYTE ->
    // INIT_XFR_WAIT_REQ -> INIT_XFR -> send_byte().  The byte goes out
    // and the drained FIFO then completes on :649 with I_BUS.
    reg_w(0x2, 0xaa);
    CHECK_TRUE("a staged byte unparks the transfer", wait_intr(20000));
    CHECK_EQ("istatus = I_BUS", reg_r(0x5), I_BUS);
    CHECK_EQ("the staged byte was sent", reg_r(0x7), 0x00);
    return true;
}

// ── RED before the 2026-08-19 correction, GREEN after ────────────────
// Post-ATN_STOP message drain PACING.  MAME sends the first byte of an
// out-phase transfer synchronously inside start_command()
// (start_command -> INIT_XFR -> send_byte()), and every later byte only
// when send_byte()'s delay() timer fires, i.e. only as emulated time
// advances.  A host that immediately hammers the pseudo-DMA aperture
// therefore pops the bytes the chip has NOT yet sent.  Our drain ran
// free at one byte per cycle and emptied the FIFO first, so those pops
// returned garbage instead of the residue.
// Measured against MAME 0.285 (scsi_fuzz seed 39): a DMA CI_XFER armed
// in MSG_OUT over a 6-byte FIFO 08 00 7f fe 01 00, followed by blind
// aperture reads —
//     mame      00 7f fe 01 00 00 ...   (exactly ONE byte sent)
//     rtl (old) 00 00 00 00 00 00 ...   (all six sent)
// This scenario pins the "exactly one" against the same shape.
static bool test_c96_atn_stop_drain_sends_one_byte_per_quiesce() {
    reset();

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);

    // IDENTIFY + a 6-byte residue, exactly the seed-39 shape.
    const uint8_t staged[7] = {0xc0, 0x08, 0x00, 0x7f, 0xfe, 0x01, 0x00};
    for (int i = 0; i < 7; ++i) reg_w(0x2, staged[i]);
    CHECK_EQ("FIFO staged 7 bytes", reg_r(0x7) & 0x1f, 0x07);

    reg_w(0x3, CD_SELECT_ATN_STOP);
    CHECK_TRUE("ATN_STOP halts after one message byte", wait_intr(20000));
    CHECK_EQ("ATN_STOP istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    CHECK_EQ("the 6-byte residue is retained", reg_r(0x7) & 0x1f, 0x06);

    // DMA Transfer Information in MSG_OUT, then a back-to-back blind
    // aperture burst — no idle window, so only the dispatch-time send
    // may have happened.
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    reg_w(0x3, 0x80 | CI_XFER);
    // Two idle bus cycles stand in for MAME's synchronous dispatch-time
    // send (start_command() calls send_byte() inline).  The fabric can
    // never issue the next CPU access with a zero-cycle gap anyway; what
    // this scenario pins is that ONE byte leaves, not that it leaves in
    // the very same cycle.
    tick_n(2);
    uint8_t got[8];
    for (int i = 0; i < 8; ++i) got[i] = shim_r();

    // Head of the residue is consumed by the ONE dispatch-time send;
    // the rest pops out in order, then MAME's memmove-pop semantics
    // repeat the last byte off an empty FIFO.
    const uint8_t want[8] = {0x00, 0x7f, 0xfe, 0x01, 0x00,
                             0x00, 0x00, 0x00};
    for (int i = 0; i < 8; ++i) {
        if (got[i] != want[i]) {
            std::printf("  FAIL blind pop[%d]: got 0x%02x, want 0x%02x\n",
                        i, got[i], want[i]);
            return false;
        }
    }
    std::printf("  PASS exactly one message byte left before the pops\n");
    return true;
}

// ── RED before the fix ───────────────────────────────────────────────
// END-TO-END payload check.  The scenarios above prove the command
// COMPLETES; this one proves the bytes actually reach the target.  A
// WRITE(6) of one 512-byte block, every byte moved by the non-DMA
// Transfer Information path in 16-byte FIFO loads (the 53C96 FIFO is
// 16 deep — ncr53c90.cpp fifo_w() saturates there), compared against
// what the SD mock recorded.  A fix that popped the FIFO and raised
// I_BUS but wrote the wrong byte — or wrote pb_wdata instead of
// c96_fifo[0] — passes every other test here and fails this one.
static bool test_c96_write6_nondma_data_out_payload() {
    reset();

    std::vector<uint8_t> payload(512);
    for (int i = 0; i < 512; ++i)
        payload[i] = (uint8_t)(0x5A ^ (i & 0xFF) ^ (i >> 8));

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    // WRITE(6): opcode 0x0A, LBA 0, 1 block.
    const uint8_t cdb[6] = {0x0A, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    reg_w(0x3, 0x41);                 // CD_SELECT_ATN, non-DMA select
    CHECK_TRUE("WRITE(6) select-complete fires", wait_intr(20000));
    CHECK_EQ("post-select status = INTR|DATA_OUT", reg_r(0x4), 0x80);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    for (int off = 0; off < 512; off += 16) {
        for (int i = 0; i < 16; ++i) reg_w(0x2, payload[off + i]);
        CHECK_EQ("FIFO fully loaded", reg_r(0x7), 0x10);
        reg_w(0x3, CI_XFER);          // 0x10 non-DMA Transfer Information
        bool intr = false;
        uint8_t v = 0;
        for (int k = 0; k < 40000; ++k) {
            v = reg_r(0x4);
            if (v & 0x80) { intr = true; break; }
        }
        if (!intr) {
            std::printf("  FAIL offset %d: non-DMA WRITE data-out never "
                        "completed — status=0x%02x cmd=0x%02x fifo=0x%02x\n",
                        off, v, reg_r(0x3), reg_r(0x7));
            return false;
        }
        CHECK_EQ("non-DMA data-out istatus = I_BUS", reg_r(0x5), I_BUS);
        CHECK_EQ("FIFO drained", reg_r(0x7), 0x00);
    }

    // The back-end kicks the volume write once the block is complete.
    bool at_status = false;
    for (int i = 0; i < 40000; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x03) { at_status = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("phase reaches STATUS after the 512th byte", at_status);
    CHECK_EQ("sd_cmd_type was a write", sd_mock.last_cmd_type, 3);
    CHECK_EQ("sd_lba was 8192 (HDD partition base)", sd_mock.last_lba,
             SD_RAW_BASE_LBA);
    CHECK_EQ("512 bytes reached the backing store",
             (uint32_t)sd_mock.written_sector.size(), 512u);
    for (int i = 0; i < 512; ++i) {
        if (sd_mock.written_sector[i] != payload[i]) {
            std::printf("  FAIL written byte[%d]: got 0x%02x, expected "
                        "0x%02x\n", i, sd_mock.written_sector[i],
                        payload[i]);
            return false;
        }
    }
    return modesel_finish();
}

// ── NOT COVERED HERE: multi-block WRITE through the concurrent-drain ring
// A 2-block WRITE(6) driven by this path completes and hands over the
// right byte COUNT, but its payload cannot be validated with this tb's
// SdMock.  MEASURED: an otherwise-identical scenario driven through the
// pre-existing pseudo-DMA path (0x90 + shim writes) corrupts the payload
// in exactly the same way, from byte 0 — so whatever this is, it is not
// the non-DMA send path added here.  The mock is the prime suspect: in
// S_DATA_OUT the ring drain advances on vh_wr_ready ALONE (see the
// "Port B" block comment in rtl/mac/scsi.v, and note vh_wr_valid is
// asserted only in S_VH_WAIT_WR), so a mock that parks sd_wr_ready high
// from sd_go — which this one does, and which no real SPI sd_ctrl does —
// runs vh_drain_ptr away from the fill pointer.  Gating the mock on
// sd_wr_valid instead does not help, because that signal is low for the
// whole concurrent drain by design.
// Faithful multi-block write coverage lives in tb-scsi-sd-e2e
// (s10_write10_lba_200_4blocks_cmd25 — 4 blocks, byte-exact, down to the
// SPI pins, PASSING), but it drives the bare-5380 REQ/ACK path, so the
// C96 non-DMA multi-block combination has no faithful harness anywhere
// yet.  Deliberately left uncovered rather than asserted against a model
// that cannot distinguish a mock artefact from an RTL bug.

// ═════════════════════════════════════════════════════════════════════
// NON-DMA TRANSFER INFORMATION IN **STATUS** AND **MSG IN**
//
// Closing the rest of the class the four previous bugs belonged to,
// rather than meeting instance five on hardware.  MAME's INIT_XFR puts
// S_PHASE_STATUS and S_PHASE_MSG_IN on the same arm as S_PHASE_DATA_IN
// (ncr53c90.cpp:623-633): one byte into the FIFO, then complete.
//
// The two pre-fix failure modes are DIFFERENT, so they get separate
// scenarios:
//   * STATUS  — the phase-change hook already granted I_BUS, but no byte
//               was ever pushed, so the driver popped 0x00 instead of the
//               real status byte.  SILENT WRONG DATA, not a hang, which
//               is why a completion-only assertion would have missed it.
//               The RED assertion is therefore on the FIFO COUNT and the
//               BYTE VALUE, not on the interrupt.
//   * MSG IN  — nothing matched at all, so it hangs like DATA OUT did.
//
// And the completions differ: STATUS -> I_BUS (bus_complete, :792-799),
// MSG IN -> I_FUNCTION (function_complete, :782-790, reached via
// INIT_XFR_RECV_BYTE_NACK at :629-630).  A fix that returned I_BUS for
// both passes a hang test and still fails here.
// ═════════════════════════════════════════════════════════════════════

// Drive a READ(6) far enough to park the target in STATUS with a known
// status byte (GOOD) pending, without using CI_COMPLETE.
static bool read6_to_status_phase() {
    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    // ATN_STOP with a preloaded CDB now halts after one message byte
    // (MAME contract, scsi_fuzz finding F2) — use the bare DMA select,
    // which consumes the whole CDB, for these end-to-end data flows.
    reg_w(0x3, 0x80 | CD_SELECT);
    CHECK_TRUE("select-complete fires", wait_intr(20000));
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    // Move the whole 512-byte data phase with DMA so the target advances
    // to STATUS on its own.
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    reg_w(0x3, 0x80 | CI_XFER);
    int got = 0;
    for (int spin = 0; spin < 20000 && got < 512; ++spin) {
        if ((reg_r(0x4) & 0x07) != 0x01) { tick_n(1); continue; }
        shim_r();
        ++got;
    }
    CHECK_EQ("drained 512 data bytes", got, 512);
    bool at_status = false;
    for (int i = 0; i < 20000; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x03) { at_status = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("target reached STATUS phase", at_status);
    reg_r(0x5);                       // drain the data-phase interrupt
    CHECK_EQ("FIFO empty entering STATUS", reg_r(0x7), 0x00);
    return true;
}

// ── RED before the fix (silent wrong data, NOT a hang) ───────────────
static bool test_c96_nondma_status_phase_pushes_status_byte() {
    reset();
    sd_mock.read_sector.assign(512, 0x77);
    if (!read6_to_status_phase()) return false;

    reg_w(0x3, CI_XFER);              // 0x10 in STATUS phase
    CHECK_TRUE("non-DMA STATUS transfer completes", wait_intr(20000));
    // bus_complete() -> I_BUS (ncr53c90.cpp:792-799).
    CHECK_EQ("STATUS istatus = I_BUS", reg_r(0x5), I_BUS);
    // The assertion that actually catches the pre-fix behaviour: the
    // byte must be IN the FIFO.  Pre-fix this reads 0x00.
    CHECK_EQ("one status byte staged in the FIFO", reg_r(0x7), 0x01);
    CHECK_EQ("status byte = GOOD (0x00 from a clean READ)", reg_r(0x2),
             0x00);
    return true;
}

// ── RED before the fix (hang) ────────────────────────────────────────
// Reached via the DISCONNECT path, which parks the target in MSG IN with
// xfer_msg = 0x04 — a NON-zero message byte, so "did a byte actually get
// pushed" cannot be confused with a zeroed FIFO read.
static bool test_c96_nondma_msgin_phase_completes_with_i_function() {
    reset();
    sd_mock.read_sector.assign(512, 0x5A);
    if (!read6_to_status_phase()) return false;

    // CI_COMPLETE parks the chip in MSG_IN (status+msg pulled), then
    // flush so the FIFO is empty for the 0x10 under test.
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    reg_w(0x3, 0x01);                 // CM_FLUSH_FIFO
    CHECK_EQ("FIFO flushed", reg_r(0x7), 0x00);
    CHECK_EQ("parked in MSG IN", reg_r(0x4) & 0x07, 0x07);

    // 2026-08-19 RECALIBRATION (scsi_fuzz seed 13 + a byte-identical
    // directed differential vs MAME 0.285): the message byte was
    // already consumed by CI_COMPLETE and is still ACK-HELD (MAME
    // INIT_CPT_RECV_BYTE_NACK) -- the target cannot present another
    // REQ until CI_MSG_ACCEPT drops ACK, so a 0x10 issued HERE hangs
    // silently: recv_byte() waits on REQ forever.  Measured: reg4
    // stays 0x17 (no INT), fflags stays 0, istatus reads 0, and a
    // follow-up command queues behind the stuck 0x10 -- identically
    // on both sides.  The old expectation (a SECOND I_FUNCTION pull
    // of the same message byte) was pre-R1 RTL behavior; the
    // I_FUNCTION pull belongs to the FIRST consumption of the byte
    // only (status via 0x10, then ONE msg 0x10 -- the seed-13 flow).
    reg_w(0x3, CI_XFER);              // 0x10 in ACK-held MSG IN
    for (int k = 0; k < 20000; ++k) {
        uint8_t v = reg_r(0x4);
        if (v & 0x80) {
            std::printf("  FAIL 0x10 in ACK-held MSG IN must hang "
                        "silently (MAME contract) but interrupted, "
                        "reg4=0x%02x istatus=0x%02x\n", v, reg_r(0x5));
            return false;
        }
    }
    CHECK_EQ("no byte staged (msg already consumed + ACK held)",
             reg_r(0x7), 0x00);
    CHECK_EQ("istatus stays clear", reg_r(0x5), 0x00);
    CHECK_EQ("still parked in MSG IN", reg_r(0x4) & 0x07, 0x07);
    // Recover for the next scenario: chip reset clears the stuck
    // command queue (CI_MSG_ACCEPT would queue behind the 0x10 --
    // measured on both sides).
    reg_w(0x3, 0x02);
    (void)reg_r(0x5);
    return true;
}

// ── CONTROL — passes on BOTH sides of the fix ────────────────────────
// A non-DMA 0x10 issued in DATA IN must still complete with I_BUS and
// stage exactly one byte, i.e. latching c96_xfr_phase did not disturb the
// hardware-validated data-IN path.  This is the guard on the refactor
// itself rather than on the new phases.
static bool test_c96_nondma_data_in_unchanged_by_phase_latch() {
    reset();
    std::vector<uint8_t> data;
    if (!inquiry_flow_tc16(data)) return false;
    return check_inquiry_payload(data);
}

// ═════════════════════════════════════════════════════════════════════
// NON-DMA TRANSFER INFORMATION IN THE REMAINING TWO OUT-GROUP PHASES
// (COMMAND and MSG OUT) — ncr53c90.cpp:601-616
//
// MAME's INIT_XFR handles S_PHASE_DATA_OUT, S_PHASE_COMMAND and
// S_PHASE_MSG_OUT on one arm.  DATA OUT was fixed 2026-08-07; these are
// the other two.  Both are reachable from the CPU's point of view during
// a deferred-CDB select (scsi.v:2131-2160): the back-end parks in
// S_COMMAND, and the ATN form additionally REPORTS MSG_OUT (110) through
// c96_phase_bits_eff (scsi.v:1286-1292) until the IDENTIFY byte arrives.
//
// The chip FIFO is structurally EMPTY throughout that window — the
// progressive-drain byte sink (scsi.v:1421-1428) and its two write
// intercepts deliberately do not push c96_fifo — so the only MAME arm
// that can apply is the degenerate empty-FIFO one:
//   :606-610  "can't send if the fifo is empty" -> no send_byte()
//   :663-666  INIT_XFR_SEND_BYTE -> INIT_XFR_WAIT_REQ
//   :649      !dma_command && (xfr_phase & S_INP) == 0 && fifo_pos == 0
//             (COMMAND = 010 and MSG OUT = 110 both have S_INP clear)
//   :686-692  INIT_XFR_BUS_COMPLETE -> bus_complete()
//   :792-799  bus_complete(): state = IDLE; istatus |= I_BUS
// So: I_BUS, no byte moved, the in-flight select untouched.
//
// The DMA form must NOT complete here (:644 needs S_TC0, which a
// just-reloaded tcounter has cleared; :649 requires !dma_command) — that
// is the ROM's own deferred-select tail flow and completing it early
// would break boot.  test_c96_dma_out_group_xfer_does_not_complete_early
// pins that down.
// ═════════════════════════════════════════════════════════════════════

// Deferred-CDB select, non-DMA form: CD_SELECT (0x41) with an EMPTY
// FIFO.  Leaves the back-end in S_COMMAND with c96_sel_active set, which
// is the only state in which the CPU can see an out-group phase other
// than DATA OUT.  `cmd` selects the plain (0x41) or ATN (0x42) form.
static bool deferred_select_out_group(uint8_t cmd) {
    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    reg_w(0x3, 0x01);                 // CM_FLUSH_FIFO
    CHECK_EQ("FIFO empty before deferred select", reg_r(0x7), 0x00);
    reg_w(0x3, cmd);                  // select with an EMPTY FIFO
    uint8_t v = 0;
    for (int i = 0; i < 64; ++i) { v = reg_r(0x6); if (v & 7) break; }
    CHECK_TRUE("deferred select: seq_step != 0", (v & 7) != 0);
    return true;
}

// TEST UNIT READY(6) — the shortest CDB that takes the target straight
// to STATUS, so a deferred select can be finished without involving the
// backing store or a data phase.
static const uint8_t TUR6_CDB[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Push CDB bytes [from, 6) through reg 2 and close the transaction out.
// Every byte before the last must leave the CPU-visible FIFO count at 0
// (progressive drain, scsi.v:1421-1428) and the phase still COMMAND —
// that invariant is what makes the empty-FIFO MAME arm above the only
// applicable one.
static bool deferred_cdb_finish_tur(int from) {
    for (int i = from; i < 6; ++i) {
        reg_w(0x2, TUR6_CDB[i]);
        if (i < 5) {
            CHECK_EQ("progressive drain keeps the FIFO empty",
                     reg_r(0x7), 0x00);
            CHECK_EQ("still in COMMAND phase mid-CDB", reg_r(0x4) & 7, 0x2);
        }
    }
    CHECK_TRUE("deferred select completes on the last CDB byte",
               wait_intr(20000));
    // seq BEFORE istatus: reading offset 5 clears both (scsi.v:2239-2245).
    CHECK_EQ("select-complete seq = 4", reg_r(0x6), 0x04);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    CHECK_EQ("TEST UNIT READY parks in STATUS", reg_r(0x4) & 7, 0x3);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    return true;
}

// ── STANDALONE CONTROL — passes on BOTH sides of the fix ─────────────
// The same deferred select and the same TEST UNIT READY CDB, with NO
// Transfer Information issued in COMMAND phase.  It pins down the
// deferred-select entry, the COMMAND phase encoding, the
// progressive-drain FIFO invariant the fix's fifo_pos == 0 gate depends
// on, the select-complete interrupt and the STATUS/MSG close-out, so a
// RED in the two scenarios below can only be read one way: a non-DMA
// Transfer Information issued in an out-group phase never completes.
static bool test_c96_deferred_select_out_group_control() {
    reset();
    if (!deferred_select_out_group(0x41)) return false;
    CHECK_EQ("deferred select parks the bus in COMMAND",
             reg_r(0x4) & 7, 0x2);
    return deferred_cdb_finish_tur(0);
}

// ── RED before the fix (hang) ────────────────────────────────────────
// ncr53c90.cpp:603 S_PHASE_COMMAND on the out arm.
static bool test_c96_nondma_command_phase_xfer_completes() {
    reset();
    if (!deferred_select_out_group(0x41)) return false;
    CHECK_EQ("bus is in COMMAND phase", reg_r(0x4) & 7, 0x2);

    // MAME command queue (finding F8): the select still occupies its
    // command slot (it retires only on a nonzero istatus read), so a
    // Transfer Information issued now QUEUES — it must NOT execute and
    // must NOT interrupt while the select is still assembling its CDB.
    reg_w(0x3, CI_XFER);              // 0x10 queued behind the select
    for (int k = 0; k < 4000; ++k) {
        uint8_t v = reg_r(0x4);
        if (v & 0x80) {
            std::printf("  FAIL queued 0x10 completed mid-select — "
                        "status=0x%02x istatus=0x%02x\n", v, reg_r(0x5));
            return false;
        }
    }
    // Deliver the CDB; the select completes normally.
    for (int i = 0; i < 6; ++i) {
        reg_w(0x2, TUR6_CDB[i]);
        if (i < 5) {
            CHECK_EQ("progressive drain keeps the FIFO empty",
                     reg_r(0x7), 0x00);
            CHECK_EQ("still in COMMAND phase mid-CDB", reg_r(0x4) & 7, 0x2);
        }
    }
    CHECK_TRUE("deferred select completes on the last CDB byte",
               wait_intr(20000));
    CHECK_EQ("select-complete seq = 4", reg_r(0x6), 0x04);
    // This istatus read retires the select AND dispatches the queued
    // 0x10 (MAME command_pop_and_chain) — in the phase current NOW,
    // which for TEST UNIT READY is STATUS.
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    CHECK_TRUE("chained 0x10 completes", wait_intr(20000));
    // Non-DMA Transfer Information in STATUS receives exactly one byte
    // (the status byte) into the FIFO and raises I_BUS
    // (ncr53c90.cpp:623-633 + :650 "non-dma in: every byte").
    CHECK_EQ("chained 0x10 pulled one byte", reg_r(0x7), 0x01);
    CHECK_EQ("chained 0x10 istatus = I_BUS", reg_r(0x5), I_BUS);
    CHECK_EQ("the byte is the GOOD status", reg_r(0x2), 0x00);
    // The status byte was consumed out of band; clean up rather than
    // model the driver-nonsense CI_COMPLETE-after-eaten-status tail.
    reg_w(0x3, CM_RESET);
    reg_w(0x3, 0x03);   // CM_RESET_BUS
    (void)reg_r(0x5);
    return true;
}

// The ATN form of the same queue contract: MSG_OUT is reported while
// the deferred select owes its IDENTIFY, and a Transfer Information
// issued there queues exactly the same way.
static bool test_c96_nondma_msg_out_phase_xfer_completes() {
    reset();
    if (!deferred_select_out_group(0x42)) return false;   // CD_SELECT_ATN
    CHECK_EQ("ATN deferred select reports MSG OUT", reg_r(0x4) & 7, 0x6);

    reg_w(0x3, CI_XFER);              // queued behind the select (F8)
    for (int k = 0; k < 4000; ++k) {
        uint8_t v = reg_r(0x4);
        if (v & 0x80) {
            std::printf("  FAIL queued 0x10 completed mid-ATN-select — "
                        "status=0x%02x istatus=0x%02x\n", v, reg_r(0x5));
            return false;
        }
    }
    // IDENTIFY, then the CDB — the ATN select must still work end to end.
    reg_w(0x2, 0xC0);
    CHECK_EQ("IDENTIFY consumed, phase now COMMAND", reg_r(0x4) & 7, 0x2);
    CHECK_EQ("IDENTIFY advanced seq to 2", reg_r(0x6), 0x02);
    for (int i = 0; i < 6; ++i) reg_w(0x2, TUR6_CDB[i]);
    CHECK_TRUE("deferred ATN select completes", wait_intr(20000));
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    CHECK_TRUE("chained 0x10 completes", wait_intr(20000));
    CHECK_EQ("chained 0x10 pulled one byte", reg_r(0x7), 0x01);
    CHECK_EQ("chained 0x10 istatus = I_BUS", reg_r(0x5), I_BUS);
    reg_w(0x3, CM_RESET);
    reg_w(0x3, 0x03);   // CM_RESET_BUS
    (void)reg_r(0x5);
    return true;
}

// Queue-depth contract (finding F8): slot 0 holds the in-flight select,
// slot 1 holds one queued command, and a THIRD command is dropped with
// S_GROSS_ERROR (status bit 6) and no interrupt (MAME command_w).
static bool test_c96_nondma_out_group_xfer_is_repeatable() {
    reset();
    if (!deferred_select_out_group(0x41)) return false;
    reg_w(0x3, CI_XFER);              // slot 1
    CHECK_EQ("no gross error with two commands pending",
             reg_r(0x4) & 0x40, 0x00);
    reg_w(0x3, CI_XFER);              // dropped: queue full
    CHECK_EQ("third command sets S_GROSS_ERROR", reg_r(0x4) & 0x40, 0x40);
    CHECK_EQ("gross error does not interrupt", reg_r(0x4) & 0x80, 0x00);
    // The select must be undisturbed by the dropped command.
    for (int i = 0; i < 6; ++i) reg_w(0x2, TUR6_CDB[i]);
    CHECK_TRUE("deferred select still completes", wait_intr(20000));
    // The istatus read clears the sticky gross error (MAME istatus_r)
    // and dispatches the one command that DID queue.
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    CHECK_EQ("gross error cleared by the istatus read",
             reg_r(0x4) & 0x40, 0x00);
    CHECK_TRUE("queued 0x10 completes after the pop", wait_intr(20000));
    CHECK_EQ("queued 0x10 istatus = I_BUS", reg_r(0x5), I_BUS);
    reg_w(0x3, CM_RESET);
    reg_w(0x3, 0x03);   // CM_RESET_BUS
    (void)reg_r(0x5);
    return true;
}

// The DMA form (0x90) queued mid-select must not complete early either —
// this is the ROM's own deferred-select flow ordering guarantee.
static bool test_c96_dma_out_group_xfer_does_not_complete_early() {
    reset();
    if (!deferred_select_out_group(0x41)) return false;
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);                 // tcount = 1
    reg_w(0x3, 0x90);                 // DMA | CI_XFER: queued (F8)
    for (int k = 0; k < 4000; ++k) {
        uint8_t v = reg_r(0x4);
        if (v & 0x80) {
            std::printf("  FAIL DMA-form 0x90 completed early in COMMAND "
                        "phase — status=0x%02x istatus=0x%02x\n",
                        v, reg_r(0x5));
            return false;
        }
    }
    // The in-flight CDB delivery must be completely undisturbed.
    for (int i = 0; i < 6; ++i) reg_w(0x2, TUR6_CDB[i]);
    CHECK_TRUE("deferred select still completes", wait_intr(20000));
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);
    // Chained 0x90 dispatches in STATUS phase; behavior there is out of
    // scope here (covered by the differential fuzzer) — clean up.
    reg_w(0x3, CM_RESET);
    reg_w(0x3, 0x03);   // CM_RESET_BUS
    (void)reg_r(0x5);
    return true;
}

// ── SELECT_ATN carrying an IDENTIFY message byte ─────────────────────────
// Every other c96 scenario in this file pushes a BARE CDB into the FIFO and
// then issues a select.  That is not what a real Mac SCSI Manager 4.3 does
// when it wants disconnect privileges: per the 53C9x contract (and MAME's
// ncr53c90, which scsi.v cites as its reference) CD_SELECT_ATN selects,
// sends ONE message byte popped from the FIFO -- the IDENTIFY, 0x80/0xC0 --
// and only THEN treats the remaining FIFO bytes as the CDB.
//
// scsi.v knows this contract: c96_sel_msg_out (declared ~line 480) models it
// exactly.  But it is gated on `c96_fifo_pos == 0`, so it only covers the
// EMPTY-FIFO deferred select used by the ROM boot scan.  The synthetic
// short-circuit path (scsi.v:2166) handles CD_SELECT, CD_SELECT_ATN and
// CD_SELECT_ATN_STOP identically and does `cdb[0] <= c96_fifo[0]`, so the
// IDENTIFY becomes the opcode: 0xC0 has [7:5]=0b110, giving cdb_len 6 and an
// unknown opcode, which lands on the invalid-opcode arm (scsi.v:2872,
// key 5 / ASC 0x20) and returns CHECK CONDITION with NO backend dispatch.
//
// That signature -- CHECK CONDITION, zero SD activity, repeating forever --
// is exactly what the .ASYC00 driver hits on hardware after ~3.5k good
// transactions, at the point the Finder comes up and SCSI Manager 4.3 starts
// issuing ATN-form selects.
static bool test_c96_select_atn_identify_read10() {
    reset();

    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x3C ^ (i & 0xFF) ^ (i >> 8));

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);

    // The real driver's FIFO layout: IDENTIFY first, then the CDB.
    // 0xC0 = IDENTIFY | DiscPriv | LUN 0.
    reg_w(0x2, 0xC0);
    const uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x09, 0x0A,
                             0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; ++i) reg_w(0x2, cdb[i]);
    CHECK_EQ("FIFO has 11 bytes (IDENTIFY + CDB)", reg_r(0x7), 0x0B);

    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    reg_w(0x3, 0x80 | CD_SELECT_ATN);

    CHECK_TRUE("select-complete interrupt fires", wait_intr(8000));
    uint8_t ph = reg_r(0x4) & 0x07;
    if (ph != 0x01) {
        std::printf("  FAIL SELECT_ATN(IDENTIFY+READ10) went to phase %u "
                    "(STATUS=3) instead of DATA IN -- the IDENTIFY byte was "
                    "parsed as the CDB opcode and the command was REJECTED "
                    "before any backend dispatch\n", ph);
        return false;
    }
    // Retire the select (F8: the 0x90 below would otherwise queue).
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

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
    CHECK_EQ("got 512 bytes", got, 512);

    bool match = true;
    for (int i = 0; i < 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  FAIL byte[%d]: got 0x%02x, expected 0x%02x\n",
                        i, data[i], sd_mock.read_sector[i]);
            match = false; break;
        }
    }
    CHECK_TRUE("data byte-identical", match);
    CHECK_EQ("sd_cmd_type was CMD17 (1)", sd_mock.last_cmd_type, 1);
    CHECK_EQ("sd_lba was 8192 + 2314", sd_mock.last_lba,
             SD_RAW_BASE_LBA + 2314u);

    CHECK_TRUE("after 512 bytes I_BUS fires", wait_intr(200));
    CHECK_EQ("phase = STATUS after data", reg_r(0x4) & 0x07, 0x03);
    // Retire the 0x90 (F8) before the completion sequence.
    CHECK_EQ("data-phase istatus = I_BUS", reg_r(0x5), I_BUS);

    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("FIFO has status+msg", reg_r(0x7), 0x02);
    uint8_t st = reg_r(0x2);
    if (st != 0x00) {
        std::printf("  FAIL status 0x%02x (0x02 = CHECK CONDITION) on a valid "
                    "READ(10) carried by SELECT_ATN\n", st);
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    RUN(test_c96_read6_lba0_one_block);
    RUN(test_c96_read10_lba2314_one_block);
    RUN(test_c96_select_atn_identify_read10);
    RUN(test_c96_read6_lba1_offset);
    RUN(test_c96_read6_sd_error);
    RUN(test_c96_rom_flow_read6_one_block);
    RUN(test_c96_rom_flow_cmd18_slow_sd);
    RUN(test_c96_rom_flow_cmd18_vbl_stall_ring_overrun);
    RUN(test_c96_rom_flow_cmd18_vbl_stall_between_chunks);
    RUN(test_c96_rom_flow_cmd18_realistic_bus_latency);
    RUN(test_c96_driver_flow_block_burst_cmd18);
    RUN(test_c96_fifo_flags_gate_753_chunked_drain);
    RUN(test_c96_inquiry_tc16_standalone);
    RUN(test_c96_inquiry_tc16_after_multiblock_read);
    RUN(test_c96_inquiry_nondma_after_multiblock_read);
    RUN(test_c96_modesel_dma_data_out_control);
    RUN(test_c96_modesel_nondma_data_out_753_wedge);
    RUN(test_c96_nondma_data_out_drains_whole_fifo);
    RUN(test_c96_nondma_data_out_empty_fifo_parks);
    RUN(test_c96_atn_stop_drain_sends_one_byte_per_quiesce);
    RUN(test_c96_write6_nondma_data_out_payload);
    RUN(test_c96_nondma_data_in_unchanged_by_phase_latch);
    RUN(test_c96_nondma_status_phase_pushes_status_byte);
    RUN(test_c96_nondma_msgin_phase_completes_with_i_function);
    RUN(test_c96_deferred_select_out_group_control);
    RUN(test_c96_nondma_command_phase_xfer_completes);
    RUN(test_c96_nondma_msg_out_phase_xfer_completes);
    RUN(test_c96_nondma_out_group_xfer_is_repeatable);
    RUN(test_c96_dma_out_group_xfer_does_not_complete_early);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
