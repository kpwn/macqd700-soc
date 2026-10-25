#include <cstdio>
#include <verilated.h>
#include "Vsd_native_csr_master.h"

static Vsd_native_csr_master dut;

static void tick() {
    dut.clk = 0;
    dut.eval();
    dut.clk = 1;
    dut.eval();
}

static bool check(bool condition, const char* message) {
    if (!condition) std::fprintf(stderr, "FAIL %s\n", message);
    return condition;
}

static void request(bool write, unsigned addr, unsigned data) {
    dut.req_valid = 1;
    dut.req_write = write;
    dut.req_addr = addr;
    dut.req_wdata = data;
    tick();
    dut.req_valid = 0;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut.rst = 1;
    dut.req_valid = 0;
    dut.wb_ack = 0;
    dut.wb_err = 0;
    dut.wb_dat_r = 0;
    tick();
    dut.rst = 0;
    tick();

    request(false, 0x1018, 0);
    if (!check(dut.wb_cyc && dut.wb_stb && !dut.wb_we &&
               dut.wb_adr == (0x1018u >> 2) && dut.wb_sel == 15,
               "read Wishbone address/control")) return 1;
    tick();
    tick();
    dut.wb_dat_r = 0x1234abcd;
    dut.wb_ack = 1;
    tick();
    dut.wb_ack = 0;
    if (!check(!dut.wb_cyc && dut.rsp_valid && !dut.rsp_error &&
               dut.rsp_rdata == 0x1234abcd, "read response")) return 1;
    tick();
    if (!check(!dut.rsp_valid && dut.req_ready, "response pulse/ready")) return 1;

    request(true, 0x2804, 2);
    if (!check(dut.wb_cyc && dut.wb_we && dut.wb_adr == (0x2804u >> 2) &&
               dut.wb_dat_w == 2, "write Wishbone address/data")) return 1;
    dut.wb_err = 1;
    tick();
    dut.wb_err = 0;
    if (!check(dut.rsp_valid && dut.rsp_error && !dut.rsp_timeout,
               "Wishbone error response")) return 1;
    tick();

    request(false, 0x080c, 0);
    for (int i = 0; i < 8 && !dut.rsp_valid; ++i) tick();
    if (!check(dut.rsp_valid && dut.rsp_error && dut.rsp_timeout &&
               !dut.wb_cyc, "bounded Wishbone timeout")) return 1;
    tick();

    request(false, 0x1000, 0);
    dut.rst = 1;
    tick();
    if (!check(!dut.wb_cyc && !dut.rsp_valid, "reset cancels transaction")) return 1;

    std::puts("PASS native SD CSR master: read, write, bus error, timeout, reset");
    return 0;
}
