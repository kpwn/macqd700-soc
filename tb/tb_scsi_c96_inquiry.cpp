// tb_scsi_c96_inquiry.cpp — end-to-end INQUIRY through the C96 path.
//
// Drives the chip exactly like the Q700 ROM's SCSI Manager driver
// (MAME ncr53c90.cpp is the behavioral reference):
//   1. CM_RESET (chip)
//   2. bus_id = TARGET_ID (=6, set by -GTARGET_ID=6 at compile time)
//   3. select_timeout = 0xa7 (Mac default)
//   4. Push 6-byte INQUIRY CDB into the FIFO
//   5. tcount = 36 (allocation length)
//   6. command = (DMA bit) | CD_SELECT_ATN_STOP
//   7. Wait for the select-complete interrupt: istatus =
//      I_FUNCTION|I_BUS (0x18), seq=4 (MAME function_bus_complete()),
//      status[2:0] = 001 (DATA IN)
//   8. Arm the data transfer: tcount = 36, command = 0x90
//      (DMA | CI_XFER "transfer information")
//   9. Read 36 bytes via the DMA shim at 0x100
//  10. Wait for the phase-change interrupt: istatus = I_BUS (0x10),
//      status[2:0] = 011 (STATUS), TC0 set (MAME bus_complete())
//  11. CI_COMPLETE (0x11) → istatus = I_FUNCTION, FIFO = status+msg
//  12. CI_MSG_ACCEPT (0x12) → istatus = I_DISCONNECT, bus free
//
// Build via:   make tb-scsi-c96-inquiry

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"

static Vtb_scsi_vhdd_sd*   dut    = nullptr;
static int      n_pass = 0;
static int      n_fail = 0;

static void tick() {
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
    dut->sd_busy     = 0;
    dut->sd_done     = 0;
    dut->sd_error    = 0;
    dut->sd_rd_valid = 0;
    dut->sd_rd_data  = 0;
    dut->sd_wr_ready = 0;
    for (int i = 0; i < 4; ++i) tick();
    dut->rst = 0;
    tick();
}

// Register-aperture helpers (collapsed to pb_addr[3:0]).
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

// DMA-shim helpers: pb_addr=0x100/0x101 with 9-bit address path.
static uint8_t shim_r() {
    dut->pb_addr  = 0x100;
    dut->pb_rd    = 1;
    dut->pb_wr    = 0;
    tick();
    dut->pb_rd    = 0;
    dut->eval();
    return dut->pb_rdata & 0xff;
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
static constexpr uint8_t CM_NOP             = 0x00;
static constexpr uint8_t CM_RESET           = 0x02;
static constexpr uint8_t CD_SELECT          = 0x41;
static constexpr uint8_t CD_SELECT_ATN      = 0x42;
static constexpr uint8_t CD_SELECT_ATN_STOP = 0x43;
static constexpr uint8_t CI_XFER            = 0x10;
static constexpr uint8_t CI_COMPLETE        = 0x11;
static constexpr uint8_t CI_MSG_ACCEPT      = 0x12;

// status (off 4) bits
static constexpr uint8_t S_INTR   = 0x80;
static constexpr uint8_t S_TC0    = 0x10;
// istatus (off 5) bits
static constexpr uint8_t I_FUNCTION   = 0x08;
static constexpr uint8_t I_BUS        = 0x10;
static constexpr uint8_t I_DISCONNECT = 0x20;

// TARGET_ID is what the back-end answers selection at.  Built-in default
// is 0; this tb is built with -GTARGET_ID=6 to match the Quadra 700 HDD.
static constexpr uint8_t TARGET_ID = 6;

// ─────────────────────────────────────────────────────────────────────
// Test 1 — INQUIRY via C96 short-circuit reaches DATA_IN with bus phase
// status[2:0] reporting I/O (=0b001).
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_inquiry_reaches_data_in() {
    reset();

    // 1. Soft-reset chip.
    reg_w(0x3, CM_RESET);
    CHECK_EQ("after CM_RESET status[INTR]", reg_r(0x4) & S_INTR, 0x00);

    // 2. Configure: bus_id, select_timeout.
    reg_w(0x4, TARGET_ID);     // bus_id = 6
    reg_w(0x5, 0xa7);          // select_timeout

    // 3. Push 6-byte INQUIRY CDB into FIFO.
    const uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    CHECK_EQ("FIFO has 6 bytes (CDB)", reg_r(0x7), 0x06);

    // 4. Set tcount = 36 (allocation length).
    reg_w(0x0, 36);
    reg_w(0x1, 0x00);

    // 5. Issue the bare DMA select.  (ATN_STOP with a preloaded CDB now
    //    halts after one message byte, per the MAME contract — scsi_fuzz
    //    finding F2.)
    reg_w(0x3, 0x80 | CD_SELECT);

    // 6. CDB should have been consumed.
    CHECK_EQ("FIFO drained by selection", reg_r(0x7), 0x00);

    // 7. Within a few cycles the back-end runs CMD_EXEC then enters
    //    DATA_IN; status[2:0] = 001 (I/O bit asserted).
    bool reached_data_in = false;
    for (int i = 0; i < 32; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x01) { reached_data_in = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("back-end reaches DATA_IN phase", reached_data_in);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 2 — INQUIRY data byte-stream end-to-end.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_inquiry_data_apple_hd_sc() {
    reset();

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    const uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    reg_w(0x0, 36);
    reg_w(0x1, 0x00);
    // (bare DMA select; see finding F2 note above)
    reg_w(0x3, 0x80 | CD_SELECT);

    // Select-complete interrupt: istatus = I_FUNCTION|I_BUS, seq = 4,
    // phase bits = 001 (DATA IN) — MAME function_bus_complete().
    bool got_sel_intr = false;
    for (int i = 0; i < 64 && !got_sel_intr; ++i) {
        if (reg_r(0x4) & S_INTR) got_sel_intr = true;
        else tick_n(1);
    }
    CHECK_TRUE("select-complete interrupt fires", got_sel_intr);
    CHECK_EQ("select-complete phase = DATA IN", reg_r(0x4) & 0x07, 0x01);
    // DMA-form select with an untouched transfer count: MAME reports
    // seq=2, not 4 — DISC_SEL_WAIT_REQ (ncr53c90.cpp:548-556) gives 4
    // only when (!dma_command || TC0) && fifo empty.
    CHECK_EQ("select-complete seq = 2 (DMA form, no TC0)",
             reg_r(0x6), 0x02);
    CHECK_EQ("select-complete istatus", reg_r(0x5), I_FUNCTION | I_BUS);

    // Arm the data transfer: tcount = 36, DMA transfer-information.
    reg_w(0x0, 36);
    reg_w(0x1, 0x00);
    reg_w(0x3, 0x80 | CI_XFER);

    // Drain 36 INQUIRY bytes via the DMA shim; each shim read consumes
    // one byte and advances buf_rd_ptr.
    uint8_t inq[36];
    int got = 0;
    for (int spin = 0; spin < 4000 && got < 36; ++spin) {
        // Wait for back-end to be ready (t_req==1 in DATA_IN).  We poll
        // status (off 4) for the DATA_IN phase encoding [2:0]=001 (I/O).
        uint8_t s = reg_r(0x4);
        if ((s & 0x07) != 0x01) { tick_n(1); continue; }
        // Take one byte from the DMA shim.
        inq[got++] = shim_r();
    }
    CHECK_EQ("got 36 INQUIRY bytes", got, 36);

    // Validate the canned reply (Apple HD SC).
    CHECK_EQ("INQUIRY device type",   inq[0], 0x00);  // DA device
    CHECK_EQ("INQUIRY removable",     inq[1], 0x00);
    CHECK_EQ("INQUIRY SCSI version",  inq[2], 0x02);  // SCSI-2
    CHECK_EQ("INQUIRY response fmt",  inq[3], 0x02);
    CHECK_EQ("INQUIRY add len",       inq[4], 31);
    // Vendor "APPLE   "
    CHECK_EQ("vendor[0]=A", inq[8],  'A');
    CHECK_EQ("vendor[1]=P", inq[9],  'P');
    CHECK_EQ("vendor[2]=P", inq[10], 'P');
    CHECK_EQ("vendor[3]=L", inq[11], 'L');
    CHECK_EQ("vendor[4]=E", inq[12], 'E');
    CHECK_EQ("vendor[5]=' '", inq[13], ' ');
    CHECK_EQ("vendor[6]=' '", inq[14], ' ');
    CHECK_EQ("vendor[7]=' '", inq[15], ' ');
    // Product "HD SC ..." (space-padded)
    CHECK_EQ("product[0]=H", inq[16], 'H');
    CHECK_EQ("product[1]=D", inq[17], 'D');
    CHECK_EQ("product[2]=' '", inq[18], ' ');
    CHECK_EQ("product[3]=S", inq[19], 'S');
    CHECK_EQ("product[4]=C", inq[20], 'C');
    // Revision "0001"
    CHECK_EQ("rev[0]=0", inq[32], '0');
    CHECK_EQ("rev[1]=0", inq[33], '0');
    CHECK_EQ("rev[2]=0", inq[34], '0');
    CHECK_EQ("rev[3]=1", inq[35], '1');

    // After 36 bytes the back-end transitions S_DATA_IN→S_STATUS; the
    // armed CI_XFER completes with I_BUS (MAME bus_complete()) and the
    // transfer counter has hit zero (TC0).
    bool got_bus = false;
    uint8_t status_reg = 0;
    for (int i = 0; i < 100; ++i) {
        status_reg = reg_r(0x4);
        if (status_reg & S_INTR) {
            uint8_t istatus = reg_r(0x5);
            if (istatus & I_BUS) { got_bus = true; break; }
        }
        tick_n(2);
    }
    CHECK_TRUE("after 36 bytes I_BUS fires (phase change)", got_bus);
    CHECK_EQ("phase = STATUS after data", status_reg & 0x07, 0x03);
    CHECK_TRUE("TC0 set after 36 DMA beats", (status_reg & S_TC0) != 0);

    // CI_COMPLETE pulls status + message into the FIFO and fires
    // I_FUNCTION (MAME function_complete()).
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("FIFO has status+msg", reg_r(0x7), 0x02);
    CHECK_EQ("status byte = GOOD", reg_r(0x2), 0x00);
    CHECK_EQ("msg byte = COMPLETE", reg_r(0x2), 0x00);
    CHECK_EQ("FIFO drained", reg_r(0x7), 0x00);

    // CI_MSG_ACCEPT releases the bus: I_DISCONNECT + bus free.
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("CI_MSG_ACCEPT istatus = I_DISCONNECT", reg_r(0x5),
             I_DISCONNECT);
    tick_n(4);
    CHECK_EQ("bus free after MSG_ACCEPT", reg_r(0x4) & 0x07, 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 3 — Selecting a non-existent target (TARGET_ID-1) still arms the
// timeout shim; no synthetic transfer launches.
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_inquiry_wrong_target_id_disconnects() {
    reset();
    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID - 1);   // 5 — no responder
    reg_w(0x5, 0x1B);             // tight timeout
    const uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    // (bare DMA select; see finding F2 note above)
    reg_w(0x3, 0x80 | CD_SELECT);
    bool got_disconnect = false;
    for (int i = 0; i < 32; ++i) {
        uint8_t s = reg_r(0x4);
        if (s & S_INTR) {
            uint8_t istatus = reg_r(0x5);
            if (istatus == 0x20 /* I_DISCONNECT */) {
                got_disconnect = true;
                break;
            }
        }
        (void)reg_r(0x6);  // drive the timeout poll counter
    }
    CHECK_TRUE("wrong target → I_DISCONNECT", got_disconnect);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 4 — CI_COMPLETE (0x11) issued while the target is still driving
// DATA IN pulls the next REAL payload byte, not a 0x00 placeholder.
//
// MAME's CI_COMPLETE (ncr53c90.cpp:1011-1015) runs recv_byte()
// unconditionally, and RECV_WAIT_SETTLE (:465-482) pushes whatever the
// target has on the data lines.  Only an OUT phase (nobody driving) can
// yield 0x00.  INIT_CPT_RECV_WAIT_REQ (:578-589) then sees the phase is
// still != MSG_IN, zeroes the command queue and bus_complete()s (I_BUS).
//
// RED before the 2026-08-19 fix: the RTL pushed a hardcoded 0x00 for
// every non-STATUS / non-MSG_IN phase (scsi_fuzz seeds 41 and 43, where
// MAME served the INQUIRY / READ(6) byte and the RTL served 0x00).
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_ci_complete_in_data_in_pulls_live_byte() {
    reset();

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    const uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    reg_w(0x3, CD_SELECT);          // non-DMA select, bare CDB

    bool in_data_in = false;
    for (int i = 0; i < 256 && !in_data_in; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x01) in_data_in = true;
        else tick_n(1);
    }
    CHECK_TRUE("select reaches DATA IN", in_data_in);
    (void)reg_r(0x5);               // drain the select interrupt

    // Two non-DMA Transfer Informations move INQUIRY bytes 0 and 1
    // (each 0x00 for a LUN-0 direct-access device), leaving byte 2
    // (0x02, "SCSI-2") as the next byte the target will hand over.
    for (int b = 0; b < 2; ++b) {
        reg_w(0x3, CI_XFER);
        bool done = false;
        for (int i = 0; i < 256 && !done; ++i) {
            if (reg_r(0x4) & S_INTR) done = true;
            else tick_n(1);
        }
        CHECK_TRUE("non-DMA CI_XFER completes", done);
        CHECK_EQ("non-DMA CI_XFER istatus = I_BUS", reg_r(0x5), I_BUS);
        CHECK_EQ("staged INQUIRY byte", reg_r(0x2), 0x00);
    }
    CHECK_EQ("FIFO empty before CI_COMPLETE", reg_r(0x7) & 0x1f, 0x00);
    CHECK_EQ("still in DATA IN", reg_r(0x4) & 0x07, 0x01);

    // CI_COMPLETE in DATA IN: I_BUS, and ONE live data byte in the FIFO.
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("CI_COMPLETE in DATA IN istatus = I_BUS", reg_r(0x5), I_BUS);
    bool staged = false;
    for (int i = 0; i < 256 && !staged; ++i) {
        if ((reg_r(0x7) & 0x1f) == 0x01) staged = true;
        else tick_n(1);
    }
    CHECK_TRUE("CI_COMPLETE staged exactly one byte", staged);
    // The live byte, NOT the out-phase 0x00 placeholder.
    CHECK_EQ("CI_COMPLETE pulled INQUIRY byte 2", reg_r(0x2), 0x02);
    CHECK_EQ("FIFO drained", reg_r(0x7) & 0x1f, 0x00);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Test 5 — INQUIRY addressed at a LUN this target does not have answers
// with byte 0 = 0x7F (PERIPHERAL QUALIFIER 011b + PERIPHERAL DEVICE TYPE
// 1Fh), not the 0x00 direct-access type.
//
// nscsi_hd.cpp:171-174 — "If the SCSI target device is not capable of
// supporting a peripheral device connected to this logical unit, the
// device server shall set these fields to 7Fh".  This is how a SCSI
// Manager bus scan learns nothing sits behind LUNs 1..7; the RTL used to
// report a healthy Apple HD SC at all eight (scsi_fuzz seed 41).
// ─────────────────────────────────────────────────────────────────────
static bool test_c96_inquiry_bad_lun_reports_7f() {
    reset();

    reg_w(0x3, CM_RESET);
    reg_w(0x4, TARGET_ID);
    reg_w(0x5, 0xa7);
    // IDENTIFY naming LUN 4, then the INQUIRY CDB.  SELECT_ATN sends the
    // first FIFO byte as the message and the rest as the CDB.
    const uint8_t msg_cdb[7] = {0x84, 0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    for (int i = 0; i < 7; ++i) reg_w(0x2, msg_cdb[i]);
    reg_w(0x3, CD_SELECT_ATN);

    bool in_data_in = false;
    for (int i = 0; i < 256 && !in_data_in; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x01) in_data_in = true;
        else tick_n(1);
    }
    CHECK_TRUE("bad-LUN INQUIRY still reaches DATA IN", in_data_in);
    (void)reg_r(0x5);

    reg_w(0x3, CI_XFER);
    bool done = false;
    for (int i = 0; i < 256 && !done; ++i) {
        if (reg_r(0x4) & S_INTR) done = true;
        else tick_n(1);
    }
    CHECK_TRUE("non-DMA CI_XFER completes", done);
    CHECK_EQ("non-DMA CI_XFER istatus = I_BUS", reg_r(0x5), I_BUS);
    CHECK_EQ("bad-LUN INQUIRY byte 0 = 0x7F", reg_r(0x2), 0x7f);
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    RUN(test_c96_inquiry_reaches_data_in);
    RUN(test_c96_inquiry_data_apple_hd_sc);
    RUN(test_c96_inquiry_wrong_target_id_disconnects);
    RUN(test_c96_ci_complete_in_data_in_pulls_live_byte);
    RUN(test_c96_inquiry_bad_lun_reports_7f);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
