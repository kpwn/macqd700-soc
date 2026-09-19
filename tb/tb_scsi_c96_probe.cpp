#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"

static Vtb_scsi_vhdd_sd* dut = nullptr;

static void tick() {
    dut->clk = 1;
    dut->eval();
    dut->clk = 0;
    dut->eval();
}

static void reset() {
    dut->rst = 1;
    // Runtime disk size (from the SD CSD at boot); without it the target
    // reports a zero-sector disk and every read fails.
    dut->disk_num_lbas = 1048576u;
    dut->pb_addr = 0;
    dut->pb_wdata = 0;
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->sd_busy = 0;
    dut->sd_done = 0;
    dut->sd_error = 0;
    dut->sd_rd_valid = 0;
    dut->sd_rd_data = 0;
    dut->sd_wr_ready = 0;
    for (int i = 0; i < 4; ++i) tick();
    dut->rst = 0;
    tick();
}

static uint8_t read_reg(uint16_t addr) {
    dut->pb_addr = addr & 0x1ff;
    dut->pb_rd = 1;
    dut->pb_wr = 0;
    tick();
    dut->pb_rd = 0;
    dut->eval();
    return dut->pb_rdata & 0xff;
}

static void write_reg(uint16_t addr, uint8_t data) {
    dut->pb_addr = addr & 0x1ff;
    dut->pb_wdata = data;
    dut->pb_wr = 1;
    dut->pb_rd = 0;
    tick();
    dut->pb_wr = 0;
}

static bool expect_eq(const char* name, uint8_t got, uint8_t exp) {
    if (got != exp) {
        std::printf("FAIL %s: got=0x%02x exp=0x%02x\n", name, got, exp);
        return false;
    }
    std::printf("PASS %s\n", name);
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    reset();
    bool ok = true;
    ok &= expect_eq("c96 idle status", read_reg(4), 0x00);

    write_reg(3, 0x02);
    ok &= expect_eq("c96 chip reset status", read_reg(4), 0x00);

    // Bus reset (CM_RESET_BUS = 0x03) raises I_SCSI_RESET in istatus and
    // sets the INTR (bit 7) of status until istatus is read.  MAME:
    // ncr53c90.cpp `step()` BUSRESET_WAIT_INT.  Probe-test smoke confirms
    // that a clear-istatus-on-read drops the interrupt indication.
    write_reg(3, 0x03);
    ok &= expect_eq("c96 bus reset asserts INTR", read_reg(4), 0x80);
    ok &= expect_eq("c96 bus reset istatus is SCSI_RESET", read_reg(5), 0x80);
    ok &= expect_eq("c96 istatus read clears INTR", read_reg(4), 0x00);
    ok &= expect_eq("c96 istatus self-clears after read", read_reg(5), 0x00);

    // bus_id=1 — an ABSENT id (this build's TARGET_ID is 0).  Selecting
    // the present target now mirrors MAME: arbitration + selection run
    // immediately and status[2:0] shows COMMAND phase even before the
    // CDB is in the FIFO (golden macqd700 boot-scan trace 2026-07-15),
    // so the idle-phase expectation only holds for a no-responder id.
    write_reg(4, 0x01);
    write_reg(5, 0xa7);
    write_reg(3, 0x41);
    ok &= expect_eq("c96 absent-select starts idle phase", read_reg(4), 0x00);

    delete dut;
    return ok ? 0 : 1;
}
