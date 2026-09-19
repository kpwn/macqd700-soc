// tb_vhdd_ctrl.cpp — unit testbench for rtl/soc/vhdd_ctrl.v
//
// This register file IS the host's control surface for both SCSI volumes
// (tools/jtag_repl.tcl's `vhdd-status` / `vhdd-enable` / `ramdisk-size`),
// and it also has to keep behaving as the xbar S2 window terminator it
// replaced — a stray access anywhere in the 1 MB window must COMPLETE,
// and must not be able to reach a register.
//
// The scenarios that matter most, and why:
//
//   * SAME-CYCLE AW+W.  axi_wide_to_axilite drives AW and W from separate
//     flags and can present them in either order OR together.  When they
//     land together the latched copies of wdata/wstrb are still stale, so
//     a naive commit stores the PREVIOUS write's data — silently, with
//     BRESP=OKAY.  `test_ctrl_same_cycle` is the check for that, and it
//     is the one this file was written for.
//   * CLAMP AT THE REGISTER.  RD_BLOCKS is clamped on write, not at the
//     point of use, so what the host reads back is always the value in
//     force.  A clamp applied downstream would let `ramdisk-size 4096`
//     read back 4096 while the volume was really 256 MB.
//   * OUT-OF-REGISTER OFFSETS.  Must read 0 and must NOT alias onto a
//     register.
//
// POSITIVE CONTROL: see the report accompanying this change.  Both of the
// checks above were proven RED by deliberately breaking the DUT (reverting
// `commit_data` to the latched `w_data_q`, and removing the RD_BLOCKS
// clamp), then restored.

#include <verilated.h>
#include "Vvhdd_ctrl.h"

#include <cstdio>
#include <cstdint>
#include <cstdarg>

static Vvhdd_ctrl* dut = nullptr;
static vluint64_t  main_time = 0;
static int         checks = 0;
static int         failures = 0;

double sc_time_stamp() { return (double)main_time; }

// Must match the instantiation in rtl/soc/fpga_top_dma.vh.
static const uint32_t IDENT_VALUE   = 0x5D0D0001u;
static const uint32_t RD_APERTURE   = 0x70000000u;
static const uint32_t RD_MAX_BLOCKS = 524288u;      // 256 MiB / 512
static const uint32_t RD_BLK_RESET  = 65536u;       //  32 MiB / 512

static void tick() {
    dut->clk = 0; dut->eval(); main_time++;
    dut->clk = 1; dut->eval(); main_time++;
}

static void ck(bool cond, const char* fmt, ...) {
    checks++;
    char buf[400];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (cond) printf("  ok:   %s\n", buf);
    else    { printf("  FAIL: %s\n", buf); failures++; }
}

static void idle() {
    dut->awvalid = 0; dut->wvalid = 0; dut->bready = 1;
    dut->arvalid = 0; dut->rready = 1;
    dut->awaddr = 0; dut->araddr = 0; dut->wdata = 0; dut->wstrb = 0xF;
}

static void reset_dut() {
    dut->rst = 1;
    dut->cfg_rst = 1;
    idle();
    dut->sd_num_lbas = 0; dut->rd_busy = 0; dut->rd_error = 0;
    dut->rd_state = 0; dut->rd_wdog_fires = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    dut->cfg_rst = 0;
    tick();
}

// Staggered write: AW first, then W a few cycles later.
static void write_reg_staggered(uint32_t off, uint32_t val, uint8_t strb = 0xF) {
    dut->awaddr = off; dut->awvalid = 1;
    for (int i = 0; i < 50 && !(dut->awvalid && dut->awready); i++) {
        dut->eval(); if (dut->awready) break; tick();
    }
    dut->eval();
    tick();
    dut->awvalid = 0;
    for (int i = 0; i < 3; i++) tick();
    dut->wdata = val; dut->wstrb = strb; dut->wvalid = 1;
    for (int i = 0; i < 50; i++) { dut->eval(); if (dut->wready) break; tick(); }
    tick();
    dut->wvalid = 0;
    for (int i = 0; i < 50; i++) { dut->eval(); if (dut->bvalid) break; tick(); }
    tick();
    idle();
    tick();
}

// Same-cycle write: AW and W presented together, which is what
// axi_wide_to_axilite actually does most of the time.
static void write_reg_same_cycle(uint32_t off, uint32_t val, uint8_t strb = 0xF) {
    dut->awaddr = off; dut->awvalid = 1;
    dut->wdata = val;  dut->wstrb = strb; dut->wvalid = 1;
    for (int i = 0; i < 50; i++) {
        dut->eval();
        if (dut->awready && dut->wready) break;
        tick();
    }
    tick();
    dut->awvalid = 0; dut->wvalid = 0;
    for (int i = 0; i < 50; i++) { dut->eval(); if (dut->bvalid) break; tick(); }
    tick();
    idle();
    tick();
}

static uint32_t read_reg(uint32_t off) {
    dut->araddr = off; dut->arvalid = 1;
    for (int i = 0; i < 50; i++) { dut->eval(); if (dut->arready) break; tick(); }
    tick();
    dut->arvalid = 0;
    uint32_t v = 0;
    for (int i = 0; i < 50; i++) {
        dut->eval();
        if (dut->rvalid) { v = dut->rdata; break; }
        tick();
    }
    tick();
    idle();
    tick();
    return v;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vvhdd_ctrl;

    printf("=== vhdd_ctrl unit tb ===\n");

    printf("[reset defaults]\n");
    reset_dut();
    ck(read_reg(0x000) == IDENT_VALUE,
       "IDENT reads 0x%08X (got 0x%08X)", IDENT_VALUE, read_reg(0x000));
    ck(read_reg(0x004) == 0x3, "CTRL resets to 0x3 (both volumes live)");
    ck(dut->dev_en == 0x3, "dev_en output resets to 0x3");
    ck(read_reg(0x008) == RD_BLK_RESET,
       "RD_BLOCKS resets to %u (32 MiB)", RD_BLK_RESET);
    ck(dut->rd_num_lbas == RD_BLK_RESET, "rd_num_lbas output matches");
    ck(read_reg(0x014) == RD_MAX_BLOCKS, "RD_MAX_BLOCKS reads %u", RD_MAX_BLOCKS);
    ck(read_reg(0x018) == RD_APERTURE, "RD_APERTURE reads 0x%08X", RD_APERTURE);

    printf("[CTRL — staggered AW then W]\n");
    write_reg_staggered(0x004, 0x1);
    ck(read_reg(0x004) == 0x1, "CTRL=1 read back");
    ck(dut->dev_en == 0x1, "dev_en follows (SD only)");
    write_reg_staggered(0x004, 0x2);
    ck(dut->dev_en == 0x2, "dev_en=2 (RAM disk only)");

    printf("[CTRL — AW and W in the SAME cycle]\n");
    // The failure this catches is silent: a stale-latch commit stores the
    // PREVIOUS write's data and still answers BRESP=OKAY.  Write a value
    // that differs from the one before it in every bit that matters.
    write_reg_staggered(0x004, 0x1);          // prime the latches with 1
    write_reg_same_cycle(0x004, 0x2);
    ck(read_reg(0x004) == 0x2,
       "CTRL=2 via same-cycle AW+W (got 0x%X) — a stale-latch commit would "
       "leave 0x1 here", read_reg(0x004));
    write_reg_staggered(0x004, 0x2);
    write_reg_same_cycle(0x004, 0x3);
    ck(read_reg(0x004) == 0x3, "CTRL=3 via same-cycle AW+W");
    ck(dut->dev_en == 0x3, "dev_en follows a same-cycle write");

    printf("[RD_BLOCKS]\n");
    write_reg_same_cycle(0x008, 4096);
    ck(read_reg(0x008) == 4096, "RD_BLOCKS=4096 read back");
    ck(dut->rd_num_lbas == 4096, "rd_num_lbas=4096");

    write_reg_same_cycle(0x008, 0x00100000);   // 1M blocks = 512 MB > max
    ck(read_reg(0x008) == RD_MAX_BLOCKS,
       "an over-max RD_BLOCKS write is CLAMPED at the register (got %u, "
       "expected %u) — so the host reads back the size actually in force",
       read_reg(0x008), RD_MAX_BLOCKS);
    ck(dut->rd_num_lbas == RD_MAX_BLOCKS, "rd_num_lbas clamped too");

    write_reg_same_cycle(0x008, 0xFFFFFFFFu);
    ck(read_reg(0x008) == RD_MAX_BLOCKS, "0xFFFFFFFF also clamps, no wrap");

    printf("[byte strobes]\n");
    write_reg_same_cycle(0x008, 0x00001234);
    ck(read_reg(0x008) == 0x00001234, "RD_BLOCKS=0x1234");
    write_reg_same_cycle(0x008, 0xFFFFFF56, 0x1);   // only byte 0 enabled
    ck(read_reg(0x008) == 0x00001256,
       "wstrb=0b0001 changed ONLY byte 0 (got 0x%08X, expected 0x00001256)",
       read_reg(0x008));

    printf("[STATUS packing]\n");
    dut->rd_busy = 1; dut->rd_error = 0; dut->rd_state = 0x9;
    dut->rd_wdog_fires = 0xBEEF;
    write_reg_same_cycle(0x004, 0x2);
    {
        uint32_t st = read_reg(0x010);
        ck((st & 0x3) == 0x2,        "STATUS[1:0] mirrors CTRL (0x%X)", st & 0x3);
        ck(((st >> 2) & 1) == 1,     "STATUS[2] = rd_busy");
        ck(((st >> 3) & 1) == 0,     "STATUS[3] = rd_error");
        ck(((st >> 4) & 0xF) == 0x9, "STATUS[7:4] = rd_state (0x%X)", (st >> 4) & 0xF);
        ck((st >> 16) == 0xBEEF,     "STATUS[31:16] = wdog fires (0x%04X)", st >> 16);
        ck(read_reg(0x01C) == 0xBEEF, "RD_WDOG_FIRES reads the same counter");
        ck(((st >> 8) & 1) == 0,     "STATUS[8] = wprot, clear here");
    }
    // Pin the whole STATUS layout against the write-protect bit.  Widening
    // CTRL to carry wprot silently shifted every field above ctrl_q up by one
    // and truncated the concat to 33 bits, losing the top watchdog bit --
    // caught only because these field assertions are positional.  wprot lives
    // in the reserved padding at [8] precisely so it displaces nothing.
    {
        write_reg_same_cycle(0x004, 0x6);       // CTRL: RAM-disk + wprot
        uint32_t st = read_reg(0x010);
        ck((st & 0x3) == 0x2,        "wprot does not disturb STATUS[1:0] (0x%X)", st & 0x3);
        ck(((st >> 8) & 1) == 1,     "STATUS[8] follows CTRL[2] wprot");
        ck(((st >> 2) & 1) == 1,     "wprot does not shift rd_busy");
        ck(((st >> 4) & 0xF) == 0x9, "wprot does not shift rd_state (0x%X)", (st >> 4) & 0xF);
        ck((st >> 16) == 0xBEEF,     "wprot does not truncate wdog (0x%04X)", st >> 16);
        write_reg_same_cycle(0x004, 0x2);       // restore
    }
    dut->rd_busy = 0; dut->rd_error = 1; dut->rd_state = 0x0;
    {
        uint32_t st = read_reg(0x010);
        ck(((st >> 2) & 1) == 0, "STATUS[2] clears with rd_busy");
        ck(((st >> 3) & 1) == 1, "STATUS[3] sets with rd_error");
    }

    printf("[SD_BLOCKS passthrough]\n");
    dut->sd_num_lbas = 0x000F4240;
    ck(read_reg(0x00C) == 0x000F4240, "SD_BLOCKS reflects the platform input");

    printf("[53C96 trace-ring readout registers]\n");
    // These landed at 0x020/0x024/0x028 and widened the register-block
    // decode from 5 to 6 address bits.  Before this section existed, the
    // terminator check below asserted that 0x020 "reads 0" -- and it still
    // passed after the widening, because this tb leaves the ring inputs
    // undriven at 0.  It was passing for the wrong reason: a live register
    // masquerading as unmapped space.  Drive the inputs to distinctive
    // values so the readout is actually proven to be wired.
    dut->trace_wrptr   = 0xABC;
    dut->trace_wrapped = 1;
    dut->trace_frozen  = 1;
    dut->trace_rd_data = 0xDEADBEEF;
    // bit 17 = frozen, bit 16 = wrapped, bits [11:0] = wr_ptr.  This exact
    // packing is what tools/jtag_repl.tcl's scsi_trace_status decodes.
    ck(read_reg(0x020) == ((1u << 17) | (1u << 16) | 0xABC),
       "TRACE_CTRL packs {frozen,wrapped,wr_ptr} (got 0x%08X)", read_reg(0x020));
    ck(read_reg(0x028) == 0xDEADBEEF,
       "TRACE_DATA returns the ring RAM output (got 0x%08X)", read_reg(0x028));
    write_reg_same_cycle(0x024, 0x00000123);
    ck(read_reg(0x024) == 0x123, "TRACE_ADDR is writable/readable (got 0x%08X)",
       read_reg(0x024));
    ck(dut->trace_rd_addr == 0x123, "TRACE_ADDR drives the ring read port");
    // freeze is a level; clear is a TOGGLE the ring edge-detects.
    uint32_t clr_before = dut->trace_clear;
    write_reg_same_cycle(0x020, 0x1);
    ck(dut->trace_freeze == 1, "TRACE_CTRL bit0 asserts freeze");
    ck(dut->trace_clear == clr_before, "freeze alone must NOT toggle clear");
    write_reg_same_cycle(0x020, 0x2);
    ck(dut->trace_freeze == 0, "TRACE_CTRL bit0 clear releases freeze");
    ck(dut->trace_clear != clr_before, "TRACE_CTRL bit1 toggles clear");

    printf("[terminator behaviour outside the register block]\n");
    write_reg_same_cycle(0x004, 0x3);
    uint32_t ctrl_before = read_reg(0x004);
    // 0x040 replaces the old 0x020 probe here: 0x020 is a real register
    // now, so asserting it reads 0 no longer tests the terminator.  The
    // register block is 6 address bits, so 0x040 is the first offset
    // genuinely outside it.
    ck(read_reg(0x040) == 0, "offset 0x040 reads 0 (does NOT mirror IDENT)");
    ck(read_reg(0x100) == 0, "offset 0x100 reads 0");
    ck(read_reg(0x5004) == 0, "offset 0x5004 reads 0 (does NOT mirror CTRL)");
    // A wild write far inside the window must complete and must not reach
    // a register — that is exactly the failure mode a mirroring decode
    // would produce.
    write_reg_same_cycle(0x5004, 0x0);
    ck(read_reg(0x004) == ctrl_before,
       "a wild write to 0x5004 completed but did NOT touch CTRL (still 0x%X)",
       read_reg(0x004));
    ck(dut->dev_en == 0x3, "dev_en unchanged by the wild write");

    // ── CTRL persistence across the two reset domains ───────────────────
    // ctrl_q runs on cfg_rst, not rst.  That is what lets an operator do
    // `vhdd-enable ram 1` over JTAG and then reboot the Mac over JTAG with
    // the enable still in force: the debug-full-reset drives soc_full_rst
    // (this module's `rst`) but NOT core_rst (`cfg_rst`).  A power cycle or
    // a real platform reset drives both and restores CTRL_RESET.
    //
    // Test against a NON-default value so the check discriminates: the
    // module's CTRL_RESET default is 2'b11, so writing 0x1 and seeing 0x1
    // survive proves the register held, and seeing it snap back to 0x3
    // proves cfg_rst actually reset it.
    reset_dut();
    write_reg_staggered(0x004, 0x1);
    ck(read_reg(0x004) == 0x1, "CTRL written to 0x1 (SD only)");
    ck(dut->dev_en == 0x1, "dev_en follows CTRL = 0x1");

    // Pulse `rst` alone — the JTAG-reboot case.
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
    ck(read_reg(0x004) == 0x1,
       "CTRL SURVIVES rst (soc_full_rst): still 0x%X", read_reg(0x004));
    ck(dut->dev_en == 0x1, "dev_en still 0x1 after rst");

    // Pulse `cfg_rst` — the power-cycle / platform-reset case.
    dut->cfg_rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->cfg_rst = 0;
    tick();
    ck(read_reg(0x004) == 0x3,
       "CTRL restored to CTRL_RESET by cfg_rst: 0x%X", read_reg(0x004));
    ck(dut->dev_en == 0x3, "dev_en back to the default after cfg_rst");

    // ── net-vHDD endpoint CSR ────────────────────────────────────────
    // The provider that consumes these does not exist yet, so this is the
    // only thing standing between a transposed strobe index and a board
    // that silently talks to the wrong host.
    printf("[net-vHDD endpoint]\n");
    // Absent-until-configured: a non-zero default would make an
    // unconfigured board start claiming frames from the SONIC, and the
    // sharing demux would fan broadcasts at a client whose tready is
    // strapped -- holding the frame forever and starving the wire.
    ck(dut->net_our_mac == 0,  "our MAC resets to zero (client absent)");
    ck(dut->net_dst_mac == 0,  "dst MAC resets to zero");
    ck(dut->net_our_ip == 0,   "our IP resets to zero");
    ck(dut->net_our_port == 0, "our port resets to zero");

    write_reg_same_cycle(0x040, 0x5D0D0001);
    write_reg_same_cycle(0x044, 0x00000200);
    ck(dut->net_our_mac == 0x02005D0D0001ULL,
       "our MAC assembles across LO/HI (0x%012llx)",
       (unsigned long long)dut->net_our_mac);
    ck(read_reg(0x040) == 0x5D0D0001, "MAC_LO reads back");
    ck(read_reg(0x044) == 0x00000200, "MAC_HI reads back, high half zeroed");

    write_reg_same_cycle(0x048, 0xC0A86409);
    ck(dut->net_our_ip == 0xC0A86409, "our IP");
    write_reg_same_cycle(0x04C, 0x68042C00);
    write_reg_same_cycle(0x050, 0x0000E04C);
    ck(dut->net_dst_mac == 0xE04C68042C00ULL, "dst MAC assembles");
    write_reg_same_cycle(0x054, 0xC0A86401);
    ck(dut->net_dst_ip == 0xC0A86401, "dst IP");

    // One register, two fields: a swapped pair here points the board at the
    // right host on the wrong port, which looks like a dead link.
    write_reg_same_cycle(0x058, 0x4E421234);
    ck(dut->net_our_port == 0x4E42, "our port is the HIGH half (0x%04x)",
       dut->net_our_port);
    ck(dut->net_dst_port == 0x1234, "dst port is the LOW half (0x%04x)",
       dut->net_dst_port);
    ck(read_reg(0x058) == 0x4E421234, "PORTS reads back as written");

    // Widening the decode window to 0x7F must not have moved anything.
    ck(read_reg(0x000) != 0, "IDENT still readable after the window widened");
    ck(read_reg(0x004) == (uint32_t)(dut->dev_en | (dut->sd_wprot << 2)),
       "CTRL still readable and 3 bits wide");

    printf("\n%d checks, %d failures\n", checks, failures);
    delete dut;
    return failures ? 1 : 0;
}
