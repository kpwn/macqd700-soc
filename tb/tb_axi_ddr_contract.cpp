// tb_axi_ddr_contract.cpp -- first-light AXI-to-DDR memory contract.
//
// Exercises axi_narrow_to_wide feeding ddr_ctrl SIM_MODEL.  The scenarios
// focus on 32-bit boot/core-style accesses used by ROM load, SD load, and
// VRAM fill paths before the real MIG IP is integrated.

#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Vtb_axi_ddr_contract.h"

static Vtb_axi_ddr_contract* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

static void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->n_awaddr = 0;
    dut->n_awvalid = 0;
    dut->n_wdata = 0;
    dut->n_wstrb = 0;
    dut->n_wlast = 0;
    dut->n_wvalid = 0;
    dut->n_bready = 1;
    dut->n_araddr = 0;
    // 2 = 4 bytes.  Matches the shape this tb drove before the adapter
    // grew n_arsize; the sub-word scenarios below override it per-read.
    dut->n_arsize = 2;
    dut->n_arvalid = 0;
    dut->n_rready = 1;
    dut->wide_override = 0;
    dut->dx_awaddr = 0;
    dut->dx_awsize = 0;
    dut->dx_awvalid = 0;
    dut->dx_wdata[0] = 0;
    dut->dx_wdata[1] = 0;
    dut->dx_wdata[2] = 0;
    dut->dx_wdata[3] = 0;
    dut->dx_wstrb = 0;
    dut->dx_wlast = 0;
    dut->dx_wvalid = 0;
    dut->dx_bready = 1;
    dut->dx_araddr = 0;
    dut->dx_arsize = 0;
    dut->dx_arvalid = 0;
    dut->dx_rready = 1;
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
}

static bool wait_cal_done(int max_cycles = 64) {
    for (int i = 0; i < max_cycles; i++) {
        if (dut->ddr_cal_done) return true;
        tick();
    }
    return false;
}

static bool write32(uint32_t addr, uint32_t data, uint8_t strb,
                    uint32_t expect_bresp = 0, int max_cycles = 200) {
    dut->n_awaddr = addr;
    dut->n_awvalid = 1;
    int cyc = 0;
    while (!dut->n_awready && cyc < max_cycles) {
        tick();
        cyc++;
    }
    if (!dut->n_awready) return false;
    tick();
    dut->n_awvalid = 0;

    dut->n_wdata = data;
    dut->n_wstrb = strb;
    dut->n_wlast = 1;
    dut->n_wvalid = 1;
    cyc = 0;
    while (!dut->n_wready && cyc < max_cycles) {
        tick();
        cyc++;
    }
    if (!dut->n_wready) return false;
    tick();
    dut->n_wvalid = 0;
    dut->n_wlast = 0;
    dut->n_wstrb = 0;

    cyc = 0;
    while (!dut->n_bvalid && cyc < max_cycles) {
        tick();
        cyc++;
    }
    if (!dut->n_bvalid) return false;
    bool ok = dut->n_bresp == expect_bresp;
    tick();
    return ok;
}

static bool read32(uint32_t addr, uint32_t* data, uint32_t expect_rresp = 0,
                   int max_cycles = 200, uint8_t arsize = 2) {
    dut->n_araddr = addr;
    dut->n_arsize = arsize;
    dut->n_arvalid = 1;
    int cyc = 0;
    while (!dut->n_arready && cyc < max_cycles) {
        tick();
        cyc++;
    }
    if (!dut->n_arready) return false;
    tick();
    dut->n_arvalid = 0;

    cyc = 0;
    while (!dut->n_rvalid && cyc < max_cycles) {
        tick();
        cyc++;
    }
    if (!dut->n_rvalid) return false;
    *data = dut->n_rdata;
    bool ok = (dut->n_rresp == expect_rresp) && dut->n_rlast;
    tick();
    return ok;
}

#define CHECK(name, cond) do { \
    if (cond) { std::printf("[PASS] %s\n", name); n_pass++; } \
    else { std::printf("[FAIL] %s\n", name); n_fail++; } \
} while (0)

static void scenario_ddr_calibrates_before_contract_traffic() {
    reset();
    dut->eval();
    bool cal_low_after_reset = !dut->ddr_cal_done;
    bool cal_ok = wait_cal_done();
    bool write_ok = write32(0x00000000, 0x420DBFF3u, 0xF);
    uint32_t got = 0;
    bool read_ok = read32(0x00000000, &got);
    CHECK("ddr_calibrates_before_contract_traffic",
          cal_low_after_reset && cal_ok && write_ok && read_ok &&
          got == 0x420DBFF3u);
}

static void scenario_all_32bit_lanes_share_one_ddr_beat() {
    reset();
    bool cal_ok = wait_cal_done();
    bool w0 = write32(0x00000100, 0x00112233u, 0xF);
    bool w1 = write32(0x00000104, 0x44556677u, 0xF);
    bool w2 = write32(0x00000108, 0x8899AABBu, 0xF);
    bool w3 = write32(0x0000010C, 0xCCDDEEFFu, 0xF);

    uint32_t r0 = 0, r1 = 0, r2 = 0, r3 = 0;
    bool q0 = read32(0x00000100, &r0);
    bool q1 = read32(0x00000104, &r1);
    bool q2 = read32(0x00000108, &r2);
    bool q3 = read32(0x0000010C, &r3);

    bool data_ok = (r0 == 0x00112233u) && (r1 == 0x44556677u) &&
                   (r2 == 0x8899AABBu) && (r3 == 0xCCDDEEFFu);
    CHECK("all_32bit_lanes_share_one_ddr_beat",
          cal_ok && w0 && w1 && w2 && w3 && q0 && q1 && q2 && q3 &&
          data_ok);
}

static void scenario_byte_strobes_preserve_neighbor_lanes() {
    reset();
    bool cal_ok = wait_cal_done();
    bool prime0 = write32(0x00000200, 0x11111111u, 0xF);
    bool prime1 = write32(0x00000204, 0x22222222u, 0xF);
    bool prime2 = write32(0x00000208, 0x33333333u, 0xF);
    bool prime3 = write32(0x0000020C, 0x44444444u, 0xF);

    bool patch1 = write32(0x00000204, 0xAAAA5555u, 0x3);
    bool patch3 = write32(0x0000020C, 0xDEADBEEFu, 0xC);

    uint32_t r0 = 0, r1 = 0, r2 = 0, r3 = 0;
    bool q0 = read32(0x00000200, &r0);
    bool q1 = read32(0x00000204, &r1);
    bool q2 = read32(0x00000208, &r2);
    bool q3 = read32(0x0000020C, &r3);

    bool data_ok = (r0 == 0x11111111u) &&
                   (r1 == 0x22225555u) &&
                   (r2 == 0x33333333u) &&
                   (r3 == 0xDEAD4444u);
    CHECK("byte_strobes_preserve_neighbor_lanes",
          cal_ok && prime0 && prime1 && prime2 && prime3 &&
          patch1 && patch3 && q0 && q1 && q2 && q3 && data_ok);
}

static void scenario_boundary_words_match_sd_load_stride() {
    reset();
    bool cal_ok = wait_cal_done();
    const uint32_t base = 0x00000FF0u;

    bool ok = true;
    for (int i = 0; i < 8; i++) {
        ok = ok && write32(base + (uint32_t)i * 4u,
                           0xA5000000u | (uint32_t)i, 0xF);
    }
    for (int i = 0; i < 8; i++) {
        uint32_t got = 0;
        ok = ok && read32(base + (uint32_t)i * 4u, &got);
        ok = ok && (got == (0xA5000000u | (uint32_t)i));
    }

    CHECK("boundary_words_match_sd_load_stride", cal_ok && ok);
}

static void scenario_vram_base_alignment_window() {
    reset();
    bool cal_ok = wait_cal_done();
    const uint32_t fb = 0x60000000u;

    bool w_tail = write32(fb + 0x0000000Cu, 0x00ABCDEFu, 0xF);
    bool w_head = write32(fb + 0x00000010u, 0x00123456u, 0xF);

    uint32_t tail = 0, head = 0;
    bool r_tail = read32(fb + 0x0000000Cu, &tail);
    bool r_head = read32(fb + 0x00000010u, &head);

    CHECK("vram_base_alignment_window",
          cal_ok && w_tail && w_head && r_tail && r_head &&
          tail == 0x00ABCDEFu && head == 0x00123456u);
}

// Direct-drive a single wide-side AW+W beat into ddr_ctrl with the
// caller-supplied awsize / awaddr / wstrb / wdata.  Used to inject
// AXI protocol violations the narrow adapter wouldn't normally emit.
static bool wide_write(uint32_t awaddr, uint8_t awsize,
                       uint16_t wstrb, uint64_t wdata_lo, uint64_t wdata_hi,
                       uint32_t expect_bresp, int max_cycles = 200) {
    dut->wide_override = 1;
    dut->dx_wdata[0] = static_cast<uint32_t>(wdata_lo & 0xFFFFFFFFu);
    dut->dx_wdata[1] = static_cast<uint32_t>(wdata_lo >> 32);
    dut->dx_wdata[2] = static_cast<uint32_t>(wdata_hi & 0xFFFFFFFFu);
    dut->dx_wdata[3] = static_cast<uint32_t>(wdata_hi >> 32);
    dut->dx_wstrb = wstrb;
    dut->dx_wlast = 1;
    dut->dx_wvalid = 1;
    dut->dx_awaddr = awaddr;
    dut->dx_awsize = awsize;
    dut->dx_awvalid = 1;
    int cyc = 0;
    while (!dut->dx_awready && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->dx_awready) {
        dut->dx_awvalid = 0; dut->dx_wvalid = 0;
        dut->wide_override = 0;
        return false;
    }
    tick();
    dut->dx_awvalid = 0;

    cyc = 0;
    while (!dut->dx_wready && cyc < max_cycles) { tick(); cyc++; }
    bool w_ok = dut->dx_wready;
    tick();
    dut->dx_wvalid = 0;
    dut->dx_wlast = 0;
    dut->dx_wstrb = 0;

    cyc = 0;
    while (!dut->dx_bvalid && cyc < max_cycles) { tick(); cyc++; }
    if (!dut->dx_bvalid) {
        dut->wide_override = 0;
        return false;
    }
    bool ok = (dut->dx_bresp == expect_bresp) && w_ok;
    tick();
    dut->wide_override = 0;
    return ok;
}

static void scenario_misaligned_word_access_returns_slverr() {
    // Post-fix contract: ddr_ctrl's align_error is size-aware.  We now
    // exercise three distinct cases through the AXI stack:
    //   1. True protocol violation  awsize=2, wstrb=0xF, addr=0x1  -> SLVERR
    //   2. Legitimate byte store    awsize=0, wstrb=0x1,  addr=0x3  -> OKAY
    //   3. Legitimate word store    awsize=1, wstrb=0x3,  addr=0x2  -> OKAY
    //
    // Cases 2+3 arrive via the narrow adapter with a byte-granular
    // awaddr + sub-word wstrb — the shape the CPU LSU / dcache-bypass
    // paths actually emit on real HW.  Case 1 bypasses the adapter
    // via the dx_* wide-side override so we can present the illegal
    // awsize=2 + misaligned addr the adapter (by design) never emits.
    //
    // ddr_ctrl uses standard AXI little-endian byte-lane mapping:
    // wstrb[i] covers wdata[8*i+7:8*i] which lands at byte-offset `i`
    // within the 128-bit beat.  The adapter's narrow→wide lane
    // placement keeps this convention (see axi_narrow_to_wide.v).
    reset();
    bool cal_ok = wait_cal_done();

    // Prime a known word so the legitimate sub-word stores below can
    // verify neighbour-lane preservation on read-back.  With AXI
    // little-endian lane mapping, `write32(0x400, 0x13572468, 0xF)`
    // leaves memory bytes at 0x400..0x403 = 68 24 57 13.
    bool prime = write32(0x00000400, 0x13572468u, 0xF);

    // (1) True misalignment — injected directly on the wide side.
    // awaddr=0x411, awsize=2 violates the AXI "awaddr aligned to
    // awsize" rule.  Place the awaddr in a fresh 128-bit beat so the
    // SLVERR doesn't perturb the primed beat at 0x400.
    bool bad_wide = wide_write(/*awaddr=*/0x00000411u,
                               /*awsize=*/2, /*wstrb=*/0x000Fu,
                               /*wdata_lo=*/0xDEADBEEFull, /*wdata_hi=*/0ull,
                               /*expect_bresp=*/2);

    // (2) Legitimate byte store at byte-offset 3 of the primed word
    // (address 0x403).  Narrow wstrb=0x1 has a single contiguous bit,
    // so the adapter derives awsize=0.  With awsize=0 AXI permits any
    // awaddr.  wdata[7:0]=0xAA carries the byte; wstrb[0] places it
    // at beat-offset 0 of the selected lane.  In this lane-0 beat,
    // that's physical byte 0x400 — so the byte lands at 0x400, not
    // 0x403.  (Rewriting the lowest byte of the primed word: the
    // upper three bytes stay at 0x135724.)
    bool byte_wr = write32(0x00000400u, 0x000000AAu, 0x1);

    // (3) Legitimate half-word store with wstrb=0x3.  Adapter derives
    // awsize=1.  AXI requires awaddr aligned to 2 bytes.  We pass
    // awaddr=0x406 (2-byte aligned within the beat).  wstrb[1:0]
    // selects bytes 0 and 1 of the narrow lane at addr[3:2]=1, i.e.
    // beat-offsets 4 and 5.  wdata lane = 0x0000CAFE → bytes
    // wdata[7:0]=0xFE, wdata[15:8]=0xCA; writes beat[4]=0xFE,
    // beat[5]=0xCA.  Lane 1 reads back as {beat[7],[6],[5],[4]} =
    // {0, 0, 0xCA, 0xFE} = 0x0000CAFE.
    bool half_wr = write32(0x00000406u, 0x0000CAFEu, 0x3);

    // Read-back.
    uint32_t w0 = 0, w1 = 0, w_fresh = 0;
    bool rd0 = read32(0x00000400u, &w0);
    bool rd1 = read32(0x00000404u, &w1);
    bool rdf = read32(0x00000410u, &w_fresh);

    bool data_ok = (w0 == 0x135724AAu) &&
                   (w1 == 0x0000CAFEu) &&
                   (w_fresh == 0x00000000u);

    CHECK("misaligned_word_access_returns_slverr",
          cal_ok && prime && bad_wide && byte_wr && half_wr &&
          rd0 && rd1 && rdf && data_ok);
}

static void scenario_single_beat_write_blocks_extra_w_until_b() {
    reset();
    bool cal_ok = wait_cal_done();

    dut->n_bready = 0;
    dut->n_awaddr = 0x00000500;
    dut->n_awvalid = 1;
    int cyc = 0;
    while (!dut->n_awready && cyc < 200) {
        tick();
        cyc++;
    }
    bool aw_ok = dut->n_awready;
    tick();
    dut->n_awvalid = 0;

    dut->n_wdata = 0x01020304u;
    dut->n_wstrb = 0xF;
    dut->n_wlast = 1;
    dut->n_wvalid = 1;
    cyc = 0;
    while (!dut->n_wready && cyc < 200) {
        tick();
        cyc++;
    }
    bool w_ok = dut->n_wready;
    tick();
    dut->n_wvalid = 0;
    dut->n_wlast = 0;

    cyc = 0;
    while (!dut->n_bvalid && cyc < 200) {
        tick();
        cyc++;
    }
    bool b_held = dut->n_bvalid;

    // MULTI-OUTSTANDING (2026-08-20).  axi_narrow_to_wide's write front
    // end now releases at the transaction's last WIDE W beat instead of
    // at B, so n_wready legitimately returns while B is still crossing
    // the fabric — that is the serialization this rework removed, and
    // "n_wready is low here" is no longer the contract.
    //
    // What this scenario actually protects is unchanged: an AW-less W
    // beat must not be spliced onto the transaction that has already been
    // handed to the fabric, and must not reach DDR.  The adapter has a
    // ONE-DEEP raw skid for the (protocol-legal) W-before-AW case, so the
    // beat may be absorbed exactly once and must then backpressure; the
    // memory read below is the anchor that proves none of it landed.
    dut->n_wdata = 0xAABBCCDDu;
    dut->n_wstrb = 0xF;
    dut->n_wlast = 1;
    dut->n_wvalid = 1;
    dut->eval();
    tick();                       // at most this one beat can be absorbed
    dut->eval();
    bool extra_w_then_blocked = !dut->n_wready;
    dut->n_wvalid = 0;
    dut->n_wlast = 0;
    dut->n_wstrb = 0;
    dut->n_bready = 1;
    tick();

    uint32_t got = 0;
    bool read_ok = read32(0x00000500, &got);
    CHECK("single_beat_write_blocks_extra_w_until_b",
          cal_ok && aw_ok && w_ok && b_held && extra_w_then_blocked &&
          read_ok && got == 0x01020304u);
}

static void scenario_counters_observe_boot_style_writes() {
    reset();
    bool cal_ok = wait_cal_done();
    uint32_t aw0 = dut->dbg_aw_cnt;
    uint32_t w0 = dut->dbg_w_cnt;
    uint32_t b0 = dut->dbg_b_cnt;
    uint32_t ar0 = dut->dbg_ar_cnt;
    uint32_t r0 = dut->dbg_r_cnt;

    bool wr0 = write32(0x00000300, 0x12345678u, 0xF);
    bool wr1 = write32(0x00000304, 0x87654321u, 0xF);
    uint32_t rd = 0;
    bool rr = read32(0x00000304, &rd);

    CHECK("counters_observe_boot_style_writes",
          cal_ok && wr0 && wr1 && rr && rd == 0x87654321u &&
          dut->dbg_aw_cnt - aw0 == 2 &&
          dut->dbg_w_cnt - w0 == 2 &&
          dut->dbg_b_cnt - b0 == 2 &&
          dut->dbg_ar_cnt - ar0 == 1 &&
          dut->dbg_r_cnt - r0 == 1);
}

// n_arsize is the pin whose absence broke this tb's elaboration.  Prove it
// is genuinely carried through to the wide side rather than tied to a
// constant: arsize=0 must reach DDR as arsize=0 at the UNTOUCHED byte
// address, arsize=1 clears bit 0, arsize=2 clears bits[1:0].  On plain
// DRAM the returned data is identical for all three (the adapter always
// slices the 32-bit word at addr[3:2]), which is exactly why an
// observation point on the wide AR channel is needed — see dbg_wide_ar*.
static void scenario_narrow_arsize_reaches_ddr_unaligned() {
    reset();
    bool cal_ok = wait_cal_done();
    bool prime = write32(0x00000400, 0x11223344u, 0xF);

    uint32_t b = 0, h = 0, w = 0;
    bool rb = read32(0x00000403, &b, 0, 200, /*arsize=*/0);
    bool byte_shape_ok = (dut->dbg_wide_araddr == 0x00000403u) &&
                         (dut->dbg_wide_arsize == 0) &&
                         (dut->dbg_wide_arlen == 0);

    bool rh = read32(0x00000403, &h, 0, 200, /*arsize=*/1);
    bool half_shape_ok = (dut->dbg_wide_araddr == 0x00000402u) &&
                         (dut->dbg_wide_arsize == 1) &&
                         (dut->dbg_wide_arlen == 0);

    bool rw = read32(0x00000403, &w, 0, 200, /*arsize=*/2);
    bool word_shape_ok = (dut->dbg_wide_araddr == 0x00000400u) &&
                         (dut->dbg_wide_arsize == 2) &&
                         (dut->dbg_wide_arlen == 0);

    // All three slice lane addr[3:2]==0, i.e. the containing 32-bit word.
    bool data_ok = (b == 0x11223344u) && (h == 0x11223344u) &&
                   (w == 0x11223344u);

    CHECK("narrow_arsize_reaches_ddr_unaligned",
          cal_ok && prime && rb && rh && rw &&
          byte_shape_ok && half_shape_ok && word_shape_ok && data_ok);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_axi_ddr_contract;
    dut->clk = 0;
    dut->eval();

    scenario_ddr_calibrates_before_contract_traffic();
    scenario_all_32bit_lanes_share_one_ddr_beat();
    scenario_byte_strobes_preserve_neighbor_lanes();
    scenario_boundary_words_match_sd_load_stride();
    scenario_vram_base_alignment_window();
    scenario_misaligned_word_access_returns_slverr();
    scenario_single_beat_write_blocks_extra_w_until_b();
    scenario_counters_observe_boot_style_writes();
    scenario_narrow_arsize_reaches_ddr_unaligned();

    std::printf("\n");
    if (n_fail == 0) {
        std::printf("All %d scenarios PASSED.\n", n_pass);
    } else {
        std::printf("%d scenarios PASSED, %d FAILED.\n", n_pass, n_fail);
    }

    dut->final();
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
