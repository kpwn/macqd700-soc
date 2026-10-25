#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vsd_native_command.h"

static Vsd_native_command dut;
static bool pending = false;
static uint32_t pending_data = 0;
static bool pending_error = false;
static int mode = 0;
static int event_reads = 0;
static std::vector<uint16_t> addresses;

static void tick() {
    dut.clk = 0;
    dut.csr_rsp_valid = pending;
    dut.csr_rsp_rdata = pending_data;
    dut.csr_rsp_error = pending_error;
    dut.csr_rsp_timeout = 0;
    dut.eval();
    bool accepted = dut.csr_req_valid && dut.csr_req_ready;
    uint16_t addr = dut.csr_req_addr;
    uint32_t data = 0;
    bool error = false;
    if (accepted) {
        addresses.push_back(addr);
        if (mode == 3 && addr == 0x1000) error = true;
        if (addr == 0x101c) {
            ++event_reads;
            data = mode == 1 ? 5 : mode == 2 || event_reads < 3 ? 0 : 1;
        } else if (addr == 0x100c) data = 0x11223344;
        else if (addr == 0x1010) data = 0x55667788;
        else if (addr == 0x1014) data = 0x99aabbcc;
        else if (addr == 0x1018) data = 0xddeeff00;
    }
    dut.clk = 1;
    dut.eval();
    pending = accepted;
    pending_data = data;
    pending_error = error;
}

static void reset(int test_mode) {
    mode = test_mode;
    pending = false;
    pending_data = 0;
    pending_error = false;
    event_reads = 0;
    addresses.clear();
    dut.rst = 1;
    dut.start = 0;
    dut.csr_req_ready = 1;
    tick();
    dut.rst = 0;
    tick();
}

static bool run(int test_mode) {
    reset(test_mode);
    dut.command = 9;
    dut.argument = 0x12340000;
    dut.response_type = 2;
    dut.data_type = 0;
    dut.start = 1;
    tick();
    dut.start = 0;
    for (int i = 0; i < 200 && !dut.done; ++i) tick();
    if (!dut.done || dut.busy) {
        std::fprintf(stderr, "command did not terminate in mode %d\n", test_mode);
        return false;
    }
    if (test_mode == 0) {
        const std::vector<uint16_t> expected = {
            0x1000, 0x1004, 0x1008, 0x101c, 0x101c, 0x101c,
            0x100c, 0x1010, 0x1014, 0x1018};
        if (dut.error || dut.timeout || dut.event_status != 1 ||
            dut.response[3] != 0x11223344 ||
            dut.response[2] != 0x55667788 ||
            dut.response[1] != 0x99aabbcc ||
            dut.response[0] != 0xddeeff00 || addresses != expected) {
            std::fputs("successful command or response ordering mismatch\n", stderr);
            return false;
        }
    } else if (!dut.error || bool(dut.timeout) != (test_mode == 1 || test_mode == 2)) {
        std::fprintf(stderr, "command error/timeout not reported in mode %d\n", test_mode);
        return false;
    }
    tick();
    if (dut.done) {
        std::fputs("done did not pulse\n", stderr);
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int test_mode = 0; test_mode < 4; ++test_mode)
        if (!run(test_mode)) return 1;
    std::puts("PASS native SD command: CSR sequence, 128-bit response, card error, event timeout, CSR error");
    return 0;
}
