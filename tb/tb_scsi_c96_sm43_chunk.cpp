// tb_scsi_c96_sm43_chunk.cpp — SM4.3 (System 7.5.3 SCSI Manager, old-API
// path) chunk-loop repro tb for the 53C96 DATA-IN wedge measured on HW
// (bitstream 0xE80161D3): command readback 0x90, status 0x01 (DATA IN,
// no INT, no TC0), tcounter 0x0010 undecremented, FIFO flags 0, SD
// backing store fully drained/idle.
//
// 2026-08-07: originally landed as INVESTIGATION + MEASUREMENT ONLY (the
// mixed DMA/non-DMA family reproduced the wedge, 10 of 273 scenarios
// RED).  It is now the regression gate for the fix — see the mixed /
// over-read / short-supply / gapless families at the bottom of main().
//
// Register-level golden sequence measured from MAME (booting 7.5.3):
//   1. (knob) absent-ID probe: dest=2, FIFO CDB preload, cmd 0x41,
//      selection-timeout I_DISCONNECT (reg5=0x20), flush, cmd 0x44.
//   2. live select, DMA form: dest=6, TC=2, FIFO preload of the first
//      4 CDB bytes, cmd 0xC1, then ONE 16-bit pdma write carrying the
//      last 2 CDB bytes.  Select-complete: INT, seq=4, reg5=0x18.
//   3. chunk loop, N*512/16 iterations: TC latch written ONCE (0x0010);
//      per chunk cmd 0x90 (reloads tcounter from the latch, clears TC0),
//      poll reg4 for TC0, reg7 expect 0x10, 8 x 16-bit pdma reads,
//      poll reg4 for INT, reg5 expect 0x10 (I_BUS).
//   4. status: phase 011, cmd 0x11 (ICCS) -> I_FUNCTION + 2 FIFO bytes,
//      pop status+msg, cmd 0x12 -> I_DISCONNECT.
//
// PLUS the mixed DMA / non-DMA Transfer Info family (the confirmed
// mechanism): MAME's golden contract for non-DMA 0x10 in DATA IN is
// ONE byte into the FIFO + I_BUS per command (fflags reads 0x01,
// ncr53c90.cpp:601-635 / :650 / :686-692 / :792-799).  scsi.v's
// c96_fifo_fill_beat used to free-run 1 byte/idle-cycle, so a single
// 0x10 pumped the FIFO to 16 while silently consuming target supply,
// starving a later 0x90 chunk into the exact HW wedge state.  These
// scenarios run FIRST after the benign control.
//
// Families and what each one is a control FOR:
//   mixed / tail       — per-byte non-DMA 0x10 (RED before the fix)
//   tail gapless       — the fill beat must not be blocked by a
//                        gap-free reg-4 poll (RED against the old
//                        `!pb_rd && !pb_wr` gate)
//   overread           — MAME's short-chunk contract: phase change ends
//                        the chunk with I_BUS and NO TC0
//   short supply       — POSITIVE CONTROL for the supply-exhaustion
//                        phase advance (RED with that advance removed)
//
// Build via:   make tb-scsi-c96-sm43-chunk

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"
#include "Vtb_scsi_vhdd_sd___024root.h"

static Vtb_scsi_vhdd_sd* dut = nullptr;
static int n_pass = 0;
static int n_fail = 0;
static std::string cur_scn = "?";

// ─── Mocked SD backing store (cloned from tb_scsi_c96_read6.cpp) ─────
// Adds first-byte latency L on top of the per-byte gap G.
struct SdMock {
    std::vector<uint8_t> read_sector;
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    bool     present = true;
    int      gap = 0;          // cycles between rd_valid pulses
    int      first_latency = 0;// extra cycles before the FIRST byte
    int      skid_max = 2;     // post-rd_ready-drop skid (CDC + in-flight)
    int      skid = 2;
    bool     burst_pair = false; // every 8th byte: deliver the NEXT byte
                                 // in the adjacent cycle (toggle-CDC
                                 // pulse-pair shape)
    int      truncate = 0;       // deliver (total - truncate) bytes then
                                 // report DONE with NO error — a backing
                                 // store that ends a read short.  This is
                                 // the POSITIVE CONTROL for the RTL's
                                 // supply-exhaustion phase advance.

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
        sd_mock.delay         = 2 + sd_mock.first_latency;
        sd_mock.gap_ctr       = 0;
        if (dut->sd_cmd_type == 1) {
            sd_mock.st    = SdMock::State::Reading;   // CMD17
            sd_mock.total = 512 - sd_mock.truncate;
        } else if (dut->sd_cmd_type == 2) {
            sd_mock.st    = SdMock::State::Reading;   // CMD18 multi-block
            sd_mock.total = 512 * (int)dut->sd_block_count - sd_mock.truncate;
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
        if (sd_mock.gap_ctr > 0) { --sd_mock.gap_ctr; return; }
        if (dut->sd_rd_ready) {
            sd_mock.skid = sd_mock.skid_max;
        } else if (sd_mock.skid > 0) {
            --sd_mock.skid;
        } else {
            return;                      // paused: ring full downstream
        }
        if (sd_mock.cnt < sd_mock.total) {
            dut->sd_rd_valid = 1;
            uint8_t byte = 0;
            if (!sd_mock.read_sector.empty()) {
                byte = sd_mock.read_sector[sd_mock.cnt %
                                            sd_mock.read_sector.size()];
            }
            dut->sd_rd_data = byte;
            ++sd_mock.cnt;
            sd_mock.gap_ctr = (sd_mock.burst_pair && (sd_mock.cnt % 8 == 7))
                              ? 0 : sd_mock.gap;
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
    dut->rst = 1;
    dut->disk_num_lbas = 1048576u;
    dut->pb_addr  = 0;
    dut->pb_wdata = 0;
    dut->pb_wr    = 0;
    dut->pb_rd    = 0;
    int gap = sd_mock.gap, lat = sd_mock.first_latency;
    int skid = sd_mock.skid_max;
    int trunc = sd_mock.truncate;
    bool bp = sd_mock.burst_pair;
    std::vector<uint8_t> pool = sd_mock.read_sector;
    sd_mock = SdMock();
    sd_mock.gap = gap;
    sd_mock.first_latency = lat;
    sd_mock.skid_max = skid;
    sd_mock.skid = skid;
    sd_mock.burst_pair = bp;
    sd_mock.truncate = trunc;
    sd_mock.read_sector = pool;
    for (int i = 0; i < 4; ++i) tick();
    dut->rst = 0;
    tick();
}

// Register-aperture helpers (identical contract to tb_scsi_c96_read6)
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

// Pseudo-DMA shim, byte lanes 0x100/0x101.  A driver-visible 16-bit
// access is two byte beats through the byte-serialising peripheral bus.
static uint8_t shim_r_at(uint16_t addr) {
    dut->pb_addr = addr;
    dut->pb_rd   = 1;
    dut->pb_wr   = 0;
    tick();
    dut->pb_rd   = 0;
    dut->eval();
    return dut->pb_rdata & 0xff;
}

static void shim_w_at(uint16_t addr, uint8_t v) {
    dut->pb_addr  = addr;
    dut->pb_wdata = v;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    tick();
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
}

static void shim_w16(uint8_t b0, uint8_t b1) {
    shim_w_at(0x100, b0);
    shim_w_at(0x101, b1);
}

static void shim_r16(std::vector<uint8_t>& out) {
    out.push_back(shim_r_at(0x100));
    out.push_back(shim_r_at(0x101));
}

// ─── Internal-state access (verilator public_flat_rd) ────────────────
#define U_SCSI(sig) (dut->rootp->tb_scsi_vhdd_sd__DOT__u_scsi__DOT__##sig)

static void dump_state(const char* why) {
    std::printf("  [%s] DUMP (%s): tcounter=%u accept_pend=%u xfr_left=%u "
                "vh_buf_count=%u phase=%u | cons supplied=%u counted=%u "
                "accepted=%u drained=%u violations=%u | sd cnt=%d st=%d\n",
                cur_scn.c_str(), why,
                (unsigned)U_SCSI(c96_tcounter),
                (unsigned)U_SCSI(c96_accept_pend),
                (unsigned)U_SCSI(c96_xfr_left),
                (unsigned)U_SCSI(vh_buf_count),
                (unsigned)U_SCSI(phase),
                (unsigned)U_SCSI(cons_supplied),
                (unsigned)U_SCSI(cons_counted),
                (unsigned)U_SCSI(cons_accepted),
                (unsigned)U_SCSI(cons_drained),
                (unsigned)U_SCSI(cons_violations),
                sd_mock.cnt, (int)sd_mock.st);
}

#define CHECK_EQ(name, got, exp) do {                                     \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp);         \
    if (_g != _e) {                                                       \
        std::printf("  [%s] FAIL %s: got 0x%02x, expected 0x%02x\n",      \
                    cur_scn.c_str(), name, _g, _e);                       \
        dump_state(name);                                                 \
        return false;                                                     \
    }                                                                     \
} while (0)

#define CHECK_TRUE(name, cond) do {                                       \
    if (!(cond)) {                                                        \
        std::printf("  [%s] FAIL %s\n", cur_scn.c_str(), name);           \
        dump_state(name);                                                 \
        return false;                                                     \
    }                                                                     \
} while (0)

static constexpr uint8_t TARGET_ID = 6;
static constexpr uint8_t I_FUNCTION   = 0x08;
static constexpr uint8_t I_BUS        = 0x10;
static constexpr uint8_t I_DISCONNECT = 0x20;

// SIM-harvest read burst: mimics the interrupt-driven poll routine —
// reads {reg4, reg6, reg3, reg7}, and reg5 ONLY when reg4 bit7 was set.
static void harvest_burst() {
    uint8_t s = reg_r(0x4);
    (void)reg_r(0x6);
    (void)reg_r(0x3);
    (void)reg_r(0x7);
    if (s & 0x80) (void)reg_r(0x5);
}

// Step 1 — absent-ID probe (SM4.3 recovery sequence).
static bool absent_id_probe() {
    reg_w(0x5, 0x01);              // short chip-timer select timeout
    reg_w(0x4, 0x02);              // dest ID 2 — absent
    const uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; ++i) reg_w(0x2, cdb[i]);
    reg_w(0x3, 0x41);              // select WITHOUT ATN, non-DMA
    // SM4.3 does NOT poll seq_step; it waits for the chip timer.
    bool intr = false;
    for (int i = 0; i < 200000; ++i) {
        if (reg_r(0x4) & 0x80) { intr = true; break; }
    }
    CHECK_TRUE("probe: selection-timeout interrupt fires", intr);
    CHECK_EQ("probe: istatus = I_DISCONNECT", reg_r(0x5), I_DISCONNECT);
    reg_w(0x3, 0x01);              // flush FIFO
    reg_w(0x3, 0x44);              // per MAME trace (0x44 here)
    return true;
}

// Step 2 — live-target select, DMA form.  TC=2 covers the 2-byte pdma
// CDB tail.
static bool live_select(uint32_t lba, int nblocks) {
    reg_w(0x4, TARGET_ID);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x02);              // TC = 2
    reg_w(0x2, 0x08);              // READ(6) opcode
    reg_w(0x2, (lba >> 16) & 0x1f);
    reg_w(0x2, (lba >> 8) & 0xff);
    reg_w(0x2, lba & 0xff);
    reg_w(0x3, 0xC1);              // DMA | select without ATN
    shim_w16((uint8_t)nblocks, 0x00);   // ONE 16-bit pdma write: CDB tail
    bool intr = false;
    uint8_t s = 0;
    for (int i = 0; i < 400000; ++i) {
        s = reg_r(0x4);
        if (s & 0x80) { intr = true; break; }
    }
    CHECK_TRUE("select-complete interrupt fires", intr);
    CHECK_EQ("select-complete seq = 4", reg_r(0x6), 0x04);
    CHECK_EQ("select-complete istatus = 0x18", reg_r(0x5),
             I_FUNCTION | I_BUS);
    reg_w(0x3, 0x01);              // flush FIFO
    return true;
}

// One DMA 16-byte chunk of the SM4.3 loop.  The TC latch is NOT
// rewritten here — the 0x90 command write reloads tcounter from the
// latch (the semantics under test).
static bool dma_chunk(std::vector<uint8_t>& data, int chunk_idx,
                      int poll_bound = 200000) {
    reg_w(0x3, 0x90);              // DMA | Transfer Information
    bool tc0 = false;
    uint8_t s = 0;
    for (int i = 0; i < poll_bound; ++i) {
        s = reg_r(0x4);
        if (s & 0x10) { tc0 = true; break; }
    }
    if (!tc0) {
        std::printf("  [%s] FAIL chunk %d: TC0 never set (reg4=0x%02x) — "
                    "WEDGE STATE\n", cur_scn.c_str(), chunk_idx, s);
        dump_state("TC0 poll timeout");
        return false;
    }
    CHECK_EQ("chunk fflags = 0x10", reg_r(0x7), 0x10);
    for (int i = 0; i < 8; ++i) shim_r16(data);
    bool intr = false;
    for (int i = 0; i < poll_bound; ++i) {
        s = reg_r(0x4);
        if (s & 0x80) { intr = true; break; }
    }
    if (!intr) {
        std::printf("  [%s] FAIL chunk %d: post-drain INT never fires "
                    "(reg4=0x%02x) — WEDGE STATE\n",
                    cur_scn.c_str(), chunk_idx, s);
        dump_state("post-drain INT timeout");
        return false;
    }
    CHECK_EQ("chunk istatus = I_BUS", reg_r(0x5), I_BUS);
    return true;
}

// A SHORT (< 16 byte) DMA chunk: the TC latch is rewritten to exactly
// `n`, then one 0x90 chunk drains n bytes byte-at-a-time (a driver's
// 16-bit pseudo-DMA access is two byte beats through the byte-
// serialising peripheral bus anyway, so a byte-granular drain is the
// same traffic).  Used to finish a transaction whose residue is not a
// multiple of 16 — exactly what SM4.3 does for a tail.
static bool dma_partial_chunk(std::vector<uint8_t>& data, int n,
                              int chunk_idx, int poll_bound = 200000) {
    reg_w(0x1, 0x00);
    reg_w(0x0, (uint8_t)n);
    reg_w(0x3, 0x90);
    bool tc0 = false;
    uint8_t s = 0;
    for (int i = 0; i < poll_bound; ++i) {
        s = reg_r(0x4);
        if (s & 0x10) { tc0 = true; break; }
    }
    if (!tc0) {
        std::printf("  [%s] FAIL partial chunk %d (n=%d): TC0 never set "
                    "(reg4=0x%02x) — WEDGE STATE\n",
                    cur_scn.c_str(), chunk_idx, n, s);
        dump_state("partial-chunk TC0 poll timeout");
        return false;
    }
    CHECK_EQ("partial chunk fflags = n", reg_r(0x7), (uint32_t)n);
    for (int i = 0; i < n; ++i)
        data.push_back(shim_r_at((i & 1) ? 0x101 : 0x100));
    bool intr = false;
    for (int i = 0; i < poll_bound; ++i) {
        s = reg_r(0x4);
        if (s & 0x80) { intr = true; break; }
    }
    if (!intr) {
        std::printf("  [%s] FAIL partial chunk %d (n=%d): post-drain INT "
                    "never fires (reg4=0x%02x) — WEDGE STATE\n",
                    cur_scn.c_str(), chunk_idx, n, s);
        dump_state("partial-chunk post-drain INT timeout");
        return false;
    }
    CHECK_EQ("partial chunk istatus = I_BUS", reg_r(0x5), I_BUS);
    return true;
}

// Step 4 — status/message epilogue.
static bool status_epilogue() {
    CHECK_EQ("phase = STATUS after final chunk", reg_r(0x4) & 0x07, 0x03);
    reg_w(0x3, 0x11);              // ICCS
    bool intr = false;
    for (int i = 0; i < 4000; ++i) {
        if (reg_r(0x4) & 0x80) { intr = true; break; }
    }
    CHECK_TRUE("ICCS interrupt fires", intr);
    CHECK_EQ("ICCS istatus = I_FUNCTION", reg_r(0x5), I_FUNCTION);
    CHECK_EQ("ICCS fflags = 2 (status+msg)", reg_r(0x7), 0x02);
    CHECK_EQ("status = GOOD", reg_r(0x2), 0x00);
    CHECK_EQ("msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, 0x12);              // message accepted
    intr = false;
    for (int i = 0; i < 4000; ++i) {
        if (reg_r(0x4) & 0x80) { intr = true; break; }
    }
    CHECK_TRUE("MSG-ACCEPT interrupt fires", intr);
    CHECK_EQ("MSG-ACCEPT istatus = I_DISCONNECT", reg_r(0x5), I_DISCONNECT);
    tick_n(8);
    return true;
}

// End-of-transaction conservation check (reads the RTL-side counters).
static bool conservation_check(int nblocks) {
    uint32_t sup = U_SCSI(cons_supplied), cnt = U_SCSI(cons_counted);
    uint32_t acc = U_SCSI(cons_accepted), drn = U_SCSI(cons_drained);
    uint32_t vio = U_SCSI(cons_violations);
    std::printf("  [%s] conservation: supplied=%u counted=%u accepted=%u "
                "drained=%u residue=%u violations=%u\n",
                cur_scn.c_str(), sup, cnt, acc, drn,
                (unsigned)U_SCSI(vh_buf_count), vio);
    CHECK_EQ("conservation supplied == counted", cnt, sup);
    CHECK_EQ("conservation supplied == nblocks*512",
             sup, (uint32_t)(nblocks * 512));
    CHECK_EQ("no conservation violations", vio, 0);
    return true;
}

// One full golden SM4.3 transaction.
static bool sm43_transaction(uint32_t lba, int nblocks, bool probe,
                             bool harvest, std::vector<uint8_t>& data) {
    if (probe && !absent_id_probe()) return false;
    if (!live_select(lba, nblocks)) return false;
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x10);              // TC latch = 16, written ONCE
    const int chunks = nblocks * 512 / 16;
    for (int c = 0; c < chunks; ++c) {
        if (!dma_chunk(data, c)) return false;
        if (harvest && c + 1 < chunks) harvest_burst();
    }
    if (!status_epilogue()) return false;
    return conservation_check(nblocks);
}

static bool verify_data(const std::vector<uint8_t>& data, int nbytes,
                        int repeat = 1) {
    CHECK_EQ("drained byte count", (uint32_t)data.size(),
             (uint32_t)(nbytes * repeat));
    for (int r = 0; r < repeat; ++r) {
        for (int i = 0; i < nbytes; ++i) {
            if (data[r * nbytes + i] != sd_mock.read_sector[i]) {
                std::printf("  [%s] FAIL byte[%d] (rep %d): got 0x%02x, "
                            "expected 0x%02x\n", cur_scn.c_str(), i, r,
                            data[r * nbytes + i], sd_mock.read_sector[i]);
                return false;
            }
        }
    }
    return true;
}

static void fill_pool(int nbytes, uint8_t seed) {
    sd_mock.read_sector.resize(nbytes);
    for (int i = 0; i < nbytes; ++i)
        sd_mock.read_sector[i] = (uint8_t)(seed ^ (i & 0xFF) ^ (i >> 8));
}

// ─────────────────────────────────────────────────────────────────────
// Scenario runners
// ─────────────────────────────────────────────────────────────────────
static bool scn_sm43(int nblocks, int gap, int lat, bool probe,
                     bool harvest, int ntrans) {
    sd_mock.gap = gap;
    sd_mock.first_latency = lat;
    sd_mock.skid_max = 2;
    sd_mock.burst_pair = false;
    fill_pool(nblocks * 512, (uint8_t)(0x5A + nblocks));
    reset();
    for (int t = 0; t < ntrans; ++t) {
        std::vector<uint8_t> data;
        if (!sm43_transaction((uint32_t)(16 + t * nblocks), nblocks,
                              probe, harvest, data)) {
            std::printf("  [%s] failed in transaction %d/%d\n",
                        cur_scn.c_str(), t + 1, ntrans);
            return false;
        }
        if (!verify_data(data, nblocks * 512)) return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// PRIME family — SM4.3-NATIVE TC=512 block flow, MAME-trace-exact
// (mame_run2.log, pc 0x408995e2..0x40899752 loop):
//   select: dest=6, flush, TC=1, cmd 0xC1, 5 CDB bytes via reg2, ONE
//   pdma tail byte (control 0x00); select-complete reg4=0x91,
//   reg7, reg5=0x18.
//   per block: reg0=0x00 reg1=0x02 (TC=512, rewritten per block),
//   cmd 0x90, BLIND drain of 256 pdma word reads paced only by the
//   DAFB DTACK holdoff (drq spin), then poll reg4 for INT, reg7,
//   reg5=0x10.  Final block: reg4 phase = STATUS, then ICCS epilogue.
//
// Fault injection: a drain STALL of S cycles after D drained bytes in
// block 0 (a VBL/Timer ISR pre-empting the pdma loop) — long enough
// for the provider stream to hit RING_HIGH_WATER so vh_rd_ready
// deasserts; plus the adversarial-provider knobs (skid K = bytes
// delivered AFTER rd_ready drops; burst pairs).  A TC=512 block
// starved 16 bytes short reads back EXACTLY the HW wedge state
// (tcounter=16, TC0 clear, accept_pend=0, fflags=0, DATA_IN, no INT).
// ─────────────────────────────────────────────────────────────────────
static bool native_select(uint32_t lba, int nblocks) {
    reg_w(0x4, TARGET_ID);
    reg_w(0x3, 0x01);              // flush
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);              // TC = 1 (covers the pdma tail byte)
    reg_w(0x3, 0xC1);              // DMA | select without ATN, empty FIFO
    // ROM gate: seq_step != 0, then phase = COMMAND.
    uint8_t v = 0;
    for (int i = 0; i < 64; ++i) { v = reg_r(0x6); if (v & 7) break; }
    CHECK_TRUE("native select: seq_step != 0", (v & 7) != 0);
    for (int i = 0; i < 64; ++i) { v = reg_r(0x4); if (v & 7) break; }
    CHECK_EQ("native select: phase = COMMAND", v & 7, 0x2);
    reg_w(0x2, 0x08);              // 5 CDB bytes via the FIFO
    reg_w(0x2, (lba >> 16) & 0x1f);
    reg_w(0x2, (lba >> 8) & 0xff);
    reg_w(0x2, lba & 0xff);
    reg_w(0x2, (uint8_t)nblocks);
    shim_w_at(0x100, 0x00);        // pdma tail: control byte
    bool intr = false;
    uint8_t s = 0;
    for (int i = 0; i < 400000; ++i) {
        s = reg_r(0x4);
        if (s & 0x80) { intr = true; break; }
    }
    CHECK_TRUE("native select-complete interrupt fires", intr);
    CHECK_EQ("native select-complete reg4 = 0x91", s, 0x91);
    (void)reg_r(0x7);
    CHECK_EQ("native select-complete istatus = 0x18", reg_r(0x5),
             I_FUNCTION | I_BUS);
    return true;
}

static bool scn_native(int nblocks, int stall_after, int stall_cycles,
                       int skid_k, bool burst) {
    sd_mock.gap = 3;
    sd_mock.first_latency = 0;
    sd_mock.skid_max = skid_k;
    sd_mock.burst_pair = burst;
    fill_pool(nblocks * 512, 0x37);
    reset();
    if (!native_select(64, nblocks)) return false;
    std::vector<uint8_t> data;
    bool stalled = false;
    for (int blk = 0; blk < nblocks; ++blk) {
        (void)reg_r(0x4);          // driver loop head (0x408995e2)
        reg_w(0x0, 0x00);          // TC = 512, rewritten per block
        reg_w(0x1, 0x02);
        reg_w(0x3, 0x90);          // DMA | Transfer Information
        for (int i = 0; i < 512; ++i) {
            if (!stalled && (int)data.size() >= stall_after &&
                stall_cycles > 0) {
                stalled = true;
                tick_n(stall_cycles);   // ISR pre-empts the pdma loop
            }
            bool drq_up = false;
            for (int spin = 0; spin < 400000; ++spin) {
                if (dut->drq) { drq_up = true; break; }
                tick_n(1);
            }
            if (!drq_up) {
                std::printf("  [%s] FAIL block %d byte %d: DRQ never "
                            "rises — WEDGE STATE\n",
                            cur_scn.c_str(), blk, i);
                dump_state("DRQ starvation mid-block");
                return false;
            }
            data.push_back(shim_r_at((i & 1) ? 0x101 : 0x100));
        }
        bool intr = false;
        uint8_t s = 0;
        for (int i = 0; i < 400000; ++i) {
            s = reg_r(0x4);
            if (s & 0x80) { intr = true; break; }
        }
        if (!intr) {
            std::printf("  [%s] FAIL block %d: post-drain INT never "
                        "fires (reg4=0x%02x) — WEDGE STATE\n",
                        cur_scn.c_str(), blk, s);
            dump_state("block INT timeout");
            return false;
        }
        (void)reg_r(0x7);
        CHECK_EQ("native block istatus = I_BUS", reg_r(0x5), I_BUS);
    }
    if (!status_epilogue()) return false;
    if (!verify_data(data, nblocks * 512)) return false;
    return conservation_check(nblocks);
}

// One non-DMA 0x10 single-byte read, MAME golden contract: EXACTLY ONE
// byte into the FIFO + I_BUS, fflags reads 0x01, pop via reg2.  Polls
// with idle-bus gaps (tick_n(3)) — real driver polls have inter-access
// gaps, and c96_fifo_fill_beat is gated on !pb_rd && !pb_wr, so a
// gap-free poll loop would artificially freeze the fill path.
// Records divergences from golden but does NOT abort, so the
// downstream (post-mixed-segment) chunk behaviour can be measured.
// Returns the number of divergences.
static int nondma_byte_read(int b, std::vector<uint8_t>& data,
                            const char* tag, int poll_gap = 3) {
    int div = 0;
    reg_w(0x3, 0x10);              // non-DMA Transfer Information
    bool intr = false;
    uint8_t s = 0;
    for (int i = 0; i < 20000; ++i) {
        s = reg_r(0x4);
        if (s & 0x80) { intr = true; break; }
        if (poll_gap) tick_n(poll_gap);
    }
    uint8_t fflags = reg_r(0x7);
    if (b == 0)
        std::printf("  [%s] MEASURED fflags after first %s non-DMA 0x10 "
                    "= 0x%02x (MAME golden: 0x01) intr=%d reg4=0x%02x\n",
                    cur_scn.c_str(), tag, fflags, (int)intr, s);
    if (!intr) {
        std::printf("  [%s] DIVERGENCE %s byte %d: I_BUS never fires "
                    "(reg4=0x%02x fflags=0x%02x)\n",
                    cur_scn.c_str(), tag, b, s, fflags);
        dump_state("non-DMA 0x10 INT timeout");
        ++div;
    } else {
        uint8_t is5 = reg_r(0x5);
        if (is5 != I_BUS) {
            std::printf("  [%s] DIVERGENCE %s byte %d: istatus=0x%02x "
                        "(golden I_BUS=0x10)\n",
                        cur_scn.c_str(), tag, b, is5);
            ++div;
        }
    }
    if (fflags != 0x01) ++div;
    data.push_back(reg_r(0x2));    // pop (0x00 if the FIFO is empty)
    return div;
}

// Mixed DMA / non-DMA Transfer Info scenario (prime suspect).
//   k DMA 16-byte chunks, then m non-DMA 0x10 single-byte reads, then
//   the REMAINING BYTES of the transaction as 16-byte 0x90 chunks plus
//   one short tail chunk.  A TC0/INT timeout on a post-mixed chunk is
//   the HW-wedge repro — its dump is the scenario's product.
//
// 2026-08-07 — SCENARIO ARITHMETIC CORRECTED.  As first written this
// drained (k + (total_chunks - k)) * 16 = nblocks*512 bytes of DMA
// *plus* the m non-DMA bytes, i.e. it asked the target for m bytes MORE
// than the CDB's supply.  That was consistent with the buggy RTL (where
// a 0x10 stole 16 bytes and delivered 1, so the over-read cancelled),
// but it is unsatisfiable against MAME: a non-DMA Transfer Information
// moves a REAL byte off the target (ncr53c90.cpp:601-635, :650), so
// after m of them the target has nblocks*512 - k*16 - m bytes left, and
// no faithful model can make a 16-byte chunk hit TC0 on a supply that
// short — MAME's target simply changes phase and the chunk ends with
// I_BUS and NO TC0 (INIT_XFR_WAIT_REQ phase-change branch, :653-657).
// The over-read behaviour is not dropped: it is now covered explicitly
// and asserted against that MAME contract by scn_overread() below.
static bool scn_mixed(int nblocks, int k, int m) {
    sd_mock.gap = 0;
    sd_mock.first_latency = 0;
    sd_mock.skid_max = 2;
    sd_mock.burst_pair = false;
    fill_pool(nblocks * 512, 0xA3);
    reset();
    if (!live_select(32, nblocks)) return false;
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x10);              // TC latch = 16, once
    std::vector<uint8_t> data;
    int div = 0;
    for (int c = 0; c < k; ++c)
        if (!dma_chunk(data, c)) return false;
    for (int b = 0; b < m; ++b)
        div += nondma_byte_read(b, data, "mixed");
    // Completion: the driver has taken k*16 + m bytes; drain exactly the
    // residue as full 16-byte chunks plus (if needed) one short chunk.
    const int total_bytes = nblocks * 512;
    int remaining = total_bytes - (k * 16 + m);
    int c = k;
    while (remaining >= 16) {
        if (!dma_chunk(data, c, 20000)) {
            std::printf("  [%s] WEDGE-REPRO at chunk %d after mixed "
                        "segment (k=%d m=%d, %d bytes still owed)\n",
                        cur_scn.c_str(), c, k, m, remaining);
            reg_w(0x3, 0x03);      // CM_RESET_BUS — recover for next scn
            (void)reg_r(0x5);
            return false;
        }
        remaining -= 16;
        ++c;
    }
    if (remaining > 0 && !dma_partial_chunk(data, remaining, c, 20000)) {
        std::printf("  [%s] WEDGE-REPRO at tail chunk %d after mixed "
                    "segment (k=%d m=%d, %d bytes owed)\n",
                    cur_scn.c_str(), c, k, m, remaining);
        reg_w(0x3, 0x03);
        (void)reg_r(0x5);
        return false;
    }
    dump_state("mixed transaction completed all chunks");
    if (div != 0) {
        std::printf("  [%s] %d golden-contract divergences in the "
                    "non-DMA segment\n", cur_scn.c_str(), div);
        reg_w(0x3, 0x03);
        (void)reg_r(0x5);
        return false;
    }
    if (!status_epilogue()) return false;
    // The non-DMA bytes must be the correct IN-SEQUENCE bytes of the
    // stream — a per-byte 0x10 that returns the wrong byte, or that
    // silently swallows extra bytes, breaks this.
    if (!verify_data(data, total_bytes)) return false;
    return conservation_check(nblocks);
}

// Over-read scenario — the initiator asks for MORE than the CDB's
// supply (the shape scn_mixed used to have implicitly).  MAME contract
// (ncr53c90.cpp INIT_XFR_WAIT_REQ :653-657 → INIT_XFR_BUS_COMPLETE
// :686-692 → bus_complete() :792-799): the target releases DATA IN when
// its supply runs out, the armed DMA|CI_XFER ends on the PHASE CHANGE,
// istatus gets I_BUS, and TC0 is NEVER set because those bytes were
// never transferred.  The chip must land in STATUS with an interrupt —
// a SHORT READ — not sit armed in DATA IN with no completion path.
//   over = 1..15, how many bytes short the final 16-byte chunk is (and
//   therefore how many bytes past the supply the initiator asks for).
static bool scn_overread(int nblocks, int over) {
    sd_mock.gap = 0;
    sd_mock.first_latency = 0;
    sd_mock.skid_max = 2;
    sd_mock.burst_pair = false;
    fill_pool(nblocks * 512, 0x6B);
    reset();
    if (!live_select(80, nblocks)) return false;
    std::vector<uint8_t> data;
    const int total_bytes = nblocks * 512;
    // Knock the stream off the 16-byte grid by `over` bytes first, so
    // the LAST full-size chunk is short by exactly `over`.
    if (!dma_partial_chunk(data, over, -1)) return false;
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x10);              // TC latch = 16 from here on
    const int full = (total_bytes - over) / 16;   // chunks that fit whole
    for (int cc = 0; cc < full; ++cc)
        if (!dma_chunk(data, cc)) return false;
    // The final chunk asks for 16 but only (16 - over) remain.
    //
    // 2026-08-19 RECALIBRATION (R1 rework): asserted against a
    // byte-identical differential vs MAME 0.285 (script: DMA select,
    // 1-byte partial + 31 full chunks, then the short chunk with a
    // DRQ-paced drain — both sides converged line-for-line).  The MAME
    // contract for the short chunk:
    //   * the target's residue stages through the REAL FIFO; when the
    //     supply runs out the target releases DATA IN — pre-drain the
    //     chip reads STAT 0x03 (STATUS, no TC0, no INT) with
    //     fflags == 16-over;
    //   * the drain is DRQ-paced.  BUSMD_1 DMA_IN needs fifo_pos >= 2
    //     until TC0 ("save last remaining byte for the processor"),
    //     and STATUS-phase pops decrement tcounter (MAME dma_r outside
    //     DATA IN) — so for over <= 7 TC0 sets mid-drain and the whole
    //     FIFO drains, while over >= 8 stalls with ONE byte left that
    //     the processor picks up through reg 2;
    //   * I_BUS fires only once DRQ is low (INIT_XFR_BUS_COMPLETE
    //     `if (dma_command && drq) break;`) — deferred, but never an
    //     interrupt-free wedge: the drain the driver performs anyway
    //     is what releases it.
    // The previous expectations (unpaced blind drain, I_BUS with the
    // residue still staged, never TC0, ICCS fflags exactly 2) encoded
    // the pre-R1 accept_pend shim, which held bytes outside the FIFO.
    const int residue = 16 - over;
    CHECK_EQ("over-read: residue before the short chunk",
             (uint32_t)(total_bytes - over - full * 16),
             (uint32_t)residue);
    reg_w(0x3, 0x90);
    bool staged = false;
    for (int i = 0; i < 200000; ++i) {
        if (reg_r(0x7) == residue) { staged = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("over-read: residue staged through the real FIFO", staged);
    if (residue >= 2) {
        CHECK_EQ("over-read: pre-drain STAT = 0x03 (STATUS, no TC0, no "
                 "INT — I_BUS held while DRQ is high)", reg_r(0x4), 0x03);
    } else {
        // residue == 1: fifo_pos never reaches the BUSMD_1 DRQ
        // threshold, so `dma_command && drq` is already false and
        // bus_complete() fires the I_BUS immediately.
        CHECK_EQ("over-read: STAT = 0x83 (I_BUS immediate, DRQ never "
                 "asserted for a single staged byte)", reg_r(0x4), 0x83);
    }
    int beats = 0;
    for (int i = 0; i < 16; ++i) {
        bool drq_up = false;
        for (int spin = 0; spin < 20000; ++spin) {
            if (dut->drq) { drq_up = true; break; }
            tick_n(1);
        }
        if (!drq_up) break;            // BUSMD_1 last-byte holdback
        data.push_back(shim_r_at((i & 1) ? 0x101 : 0x100));
        ++beats;
    }
    CHECK_EQ("over-read: DRQ-paced beat count", (uint32_t)beats,
             (uint32_t)((over <= 7) ? residue : residue - 1));
    bool intr = false;
    uint8_t s = 0;
    for (int i = 0; i < 200000; ++i) {
        s = reg_r(0x4);
        if (s & 0x80) { intr = true; break; }
        tick_n(1);
    }
    if (!intr) {
        std::printf("  [%s] FAIL over-read: short final chunk never "
                    "interrupted (reg4=0x%02x) — INTERRUPT-FREE WEDGE\n",
                    cur_scn.c_str(), s);
        dump_state("over-read short-chunk INT timeout");
        return false;
    }
    CHECK_EQ("over-read: short chunk istatus = I_BUS", reg_r(0x5), I_BUS);
    if (over <= 7) {
        CHECK_EQ("over-read: TC0 set by the STATUS-phase pops",
                 reg_r(0x4), 0x13);
        CHECK_EQ("over-read: FIFO fully drained", reg_r(0x7), 0x00);
    } else {
        CHECK_EQ("over-read: no TC0 (too few pops)", reg_r(0x4), 0x03);
        CHECK_EQ("over-read: one byte left for the processor (BUSMD_1)",
                 reg_r(0x7), 0x01);
        data.push_back(reg_r(0x2));    // processor pickup of the residue
        CHECK_EQ("over-read: FIFO empty after reg-2 pickup",
                 reg_r(0x7), 0x00);
    }
    if (!status_epilogue()) return false;
    if (!verify_data(data, total_bytes)) return false;
    return conservation_check(nblocks);
}

// Tail-byte variant: drain all but the last m bytes of an N=1 read via
// DMA chunks (rewriting the TC latch for one partial chunk when
// needed), then m non-DMA 0x10 byte reads at end-of-supply, then the
// full ICCS epilogue.  Hypothesis: "accidentally works" via
// instant-I_BUS-in-STATUS; measure the FIFO residue and that ICCS sees
// fflags == 0x02 exactly.
static bool scn_tail(int m, int poll_gap = 3) {
    sd_mock.gap = 0;
    sd_mock.first_latency = 0;
    sd_mock.skid_max = 2;
    sd_mock.burst_pair = false;
    fill_pool(512, 0xC7);
    reset();
    if (!live_select(48, 1)) return false;
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x10);
    std::vector<uint8_t> data;
    int full_chunks = (512 - m) / 16;
    int partial = (512 - m) % 16;
    for (int c = 0; c < full_chunks; ++c)
        if (!dma_chunk(data, c)) return false;
    if (partial) {
        reg_w(0x1, 0x00);
        reg_w(0x0, (uint8_t)partial);   // one partial chunk
        reg_w(0x3, 0x90);
        bool tc0 = false;
        for (int i = 0; i < 200000; ++i)
            if (reg_r(0x4) & 0x10) { tc0 = true; break; }
        CHECK_TRUE("partial chunk TC0 sets", tc0);
        for (int i = 0; i < partial / 2; ++i) shim_r16(data);
        bool intr = false;
        for (int i = 0; i < 200000; ++i)
            if (reg_r(0x4) & 0x80) { intr = true; break; }
        CHECK_TRUE("partial chunk INT fires", intr);
        CHECK_EQ("partial chunk istatus = I_BUS", reg_r(0x5), I_BUS);
    }
    int div = 0;
    for (int b = 0; b < m; ++b)
        div += nondma_byte_read(b, data, "tail", poll_gap);
    // Measure the ICCS-time FIFO residue explicitly (golden: exactly
    // 0x02 = status+msg; leftover data-in bytes are a divergence).
    uint8_t s4 = reg_r(0x4);
    std::printf("  [%s] MEASURED post-tail reg4=0x%02x fflags=0x%02x "
                "(golden: phase 011, fflags 0x00)\n",
                cur_scn.c_str(), s4, reg_r(0x7));
    if (!status_epilogue()) return false;
    if (div != 0) {
        std::printf("  [%s] %d golden-contract divergences in the tail "
                    "segment\n", cur_scn.c_str(), div);
        return false;
    }
    if (!verify_data(data, 512)) return false;
    return conservation_check(1);
}

// POSITIVE CONTROL for the RTL's supply-exhaustion phase advance.
// The backing store ends a multi-block read `trunc` bytes early and
// reports DONE with NO error, so the ring empties while the CDB still
// owes bytes.  There is no drain beat left to carry the phase exit
// (drain beats need accept_pend != 0, accepts need avail > pend), so
// without the structural advance the chip sits armed in DATA IN with
// tcounter != 0 and never interrupts again — the exact HW signature
// (cmd=0x90, status=0x01, tcounter=0x0010, FIFO 0, back-end idle).
// With it the transaction ends as a SHORT READ: STATUS phase, I_BUS,
// normal ICCS epilogue, and NOT ONE invented byte.
static bool scn_short_supply(int nblocks, int trunc_bytes) {
    sd_mock.gap = 0;
    sd_mock.first_latency = 0;
    sd_mock.skid_max = 2;
    sd_mock.burst_pair = false;
    sd_mock.truncate = trunc_bytes;
    fill_pool(nblocks * 512, 0x2D);
    reset();
    bool ok = false;
    const int supplied = nblocks * 512 - trunc_bytes;
    const int full     = supplied / 16;
    const int residue  = supplied - full * 16;
    std::vector<uint8_t> data;
    do {
        if (!live_select(96, nblocks)) break;
        reg_w(0x1, 0x00);
        reg_w(0x0, 0x10);              // TC latch = 16
        bool inner_fail = false;
        for (int c = 0; c < full; ++c)
            if (!dma_chunk(data, c)) { inner_fail = true; break; }
        if (inner_fail) break;
        uint8_t s = reg_r(0x4);
        if (residue != 0) {
            // The tail bytes are still in the ring: one more chunk takes
            // them, and the supply runs dry inside it.
            // 2026-08-19 RECALIBRATION (R1 rework): the staged bytes
            // live in the REAL FIFO now, so the drain follows the
            // BUSMD_1 contract (differential-adjudicated vs MAME
            // 0.285): DRQ needs fifo_pos >= 2 until TC0, and pops that
            // race the staging inside DATA IN do NOT decrement tcounter
            // — the DRQ-paced loop therefore legitimately stops one
            // byte short ("save last remaining byte for the
            // processor"), the phase-change I_BUS fires once DRQ is
            // low, and the processor collects the final byte through
            // reg 2.  The old expectation (DRQ carries every last byte)
            // encoded the pre-R1 accept_pend shim.
            reg_w(0x3, 0x90);
            for (int i = 0; i < residue; ++i) {
                bool drq_up = false;
                for (int spin = 0; spin < 200000; ++spin) {
                    if (dut->drq) { drq_up = true; break; }
                    tick_n(1);
                }
                if (!drq_up) break;    // BUSMD_1 last-byte holdback
                data.push_back(shim_r_at((i & 1) ? 0x101 : 0x100));
            }
            bool intr = false;
            for (int i = 0; i < 200000; ++i) {
                s = reg_r(0x4);
                if (s & 0x80) { intr = true; break; }
                tick_n(1);
            }
            if (!intr) {
                std::printf("  [%s] FAIL short supply: tail chunk never "
                            "interrupted (reg4=0x%02x) — INTERRUPT-FREE "
                            "WEDGE\n", cur_scn.c_str(), s);
                dump_state("short-supply tail INT timeout");
                break;
            }
            if (reg_r(0x5) != I_BUS) {
                std::printf("  [%s] FAIL short supply: tail istatus != "
                            "I_BUS\n", cur_scn.c_str());
                break;
            }
            {
                uint8_t fl = reg_r(0x7);
                if (fl == 1) {
                    data.push_back(reg_r(0x2));   // processor pickup
                } else if (fl != 0) {
                    std::printf("  [%s] FAIL short supply: %u bytes left "
                                "in the FIFO after the paced drain "
                                "(expected 0 or 1)\n",
                                cur_scn.c_str(), (unsigned)fl);
                    dump_state("short-supply tail residue");
                    break;
                }
            }
            s = reg_r(0x4);
        }
        if ((s & 0x07) != 0x03) {
            std::printf("  [%s] FAIL short supply: chip still in phase %d, "
                        "did not advance to STATUS (reg4=0x%02x)\n",
                        cur_scn.c_str(), s & 0x07, s);
            dump_state("short-supply phase did not advance");
            break;
        }
        if (!status_epilogue()) break;
        if (!verify_data(data, supplied)) break;
        std::printf("  [%s] short read completed: %d of %d bytes, no "
                    "invented data\n",
                    cur_scn.c_str(), (int)data.size(), nblocks * 512);
        ok = true;
    } while (false);
    sd_mock.truncate = 0;              // never leak the knob to later scns
    return ok;
}

#define RUN_SCN(label, call) do {                                         \
    cur_scn = label;                                                      \
    std::printf("[RUN ] %s\n", cur_scn.c_str());                          \
    bool _ok = (call);                                                    \
    if (_ok) { ++n_pass; std::printf("[PASS] %s\n", cur_scn.c_str()); }   \
    else     { ++n_fail; std::printf("[FAIL] %s\n", cur_scn.c_str()); }   \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    // ── Positive control: the benign configuration must pass before
    //    any other FAIL counts as signal. ─────────────────────────────
    RUN_SCN("benign N=1 G=0 L=0", scn_sm43(1, 0, 0, false, false, 1));
    if (n_fail != 0) {
        std::printf("\nBENIGN CONFIGURATION FAILED — fix the tb before "
                    "reading anything else as signal.\n");
        std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
        return 1;
    }

    char label[128];

    // ── PRIME family: native TC=512 flow + drain stall + adversarial
    //    provider (see scn_native block comment). ─────────────────────
    RUN_SCN("native benign N=1", scn_native(1, 0, 0, 2, false));
    for (int nb : {1, 3})
        for (int d : {64, 240, 480, 496})
            for (int s : {1000, 10000, 60000})
                for (int k : {0, 2, 16}) {
                    std::snprintf(label, sizeof label,
                                  "native N=%d D=%d S=%d K=%d", nb, d, s, k);
                    RUN_SCN(label, scn_native(nb, d, s, k, false));
                }
    for (int k : {0, 2, 16}) {
        std::snprintf(label, sizeof label,
                      "native burst N=1 D=240 S=10000 K=%d", k);
        RUN_SCN(label, scn_native(1, 240, 10000, k, true));
    }

    // ── Mixed DMA / non-DMA Transfer Info family ────────────────────
    // (each scenario attempts full completion after the mixed segment,
    // so a downstream chunk wedge is captured in the same run)
    for (int nb : {1, 2})
        for (int k : {1, 2})
            for (int m : {1, 4}) {
                std::snprintf(label, sizeof label,
                              "mixed N=%d k=%d m=%d", nb, k, m);
                RUN_SCN(label, scn_mixed(nb, k, m));
            }
    RUN_SCN("tail m=4",  scn_tail(4));
    RUN_SCN("tail m=16", scn_tail(16));
    // GAP-FREE poll variants: a driver that spins on reg4 with no idle
    // cycle between accesses.  The pre-fix fill beat was gated on
    // `!pb_rd && !pb_wr`, so this loop starved the FIFO push it was
    // waiting for and could never see its own interrupt.
    RUN_SCN("tail gapless m=4",  scn_tail(4, 0));
    RUN_SCN("tail gapless m=16", scn_tail(16, 0));

    // ── Over-read family: the initiator asks for more than the CDB
    //    supplies.  MAME ends the chunk on the PHASE CHANGE with I_BUS
    //    and no TC0 (a short read); an interrupt-free wedge here is the
    //    HW failure mode this landing removes. ───────────────────────
    for (int nb : {1, 2})
        for (int ov : {1, 2, 8, 15}) {
            std::snprintf(label, sizeof label,
                          "overread N=%d over=%d", nb, ov);
            RUN_SCN(label, scn_overread(nb, ov));
        }

    // ── Short-supply family: POSITIVE CONTROL for the supply-exhaustion
    //    phase advance.  The backing store ends the read early with no
    //    error; the transfer must finish as a short read instead of
    //    parking armed in DATA IN with no interrupt. ─────────────────
    for (int nb : {2, 3})
        for (int t : {16, 64, 5, 37}) {
            std::snprintf(label, sizeof label,
                          "short supply N=%d trunc=%d", nb, t);
            RUN_SCN(label, scn_short_supply(nb, t));
        }

    // ── Pacing sweep ────────────────────────────────────────────────
    for (int nb : {1, 2, 3})
        for (int g : {0, 1, 3, 17, 33})
            for (int lat : {0, 200, 2000})
                for (int probe : {0, 1})
                    for (int hv : {0, 1}) {
                        std::snprintf(label, sizeof label,
                                      "sm43 N=%d G=%d L=%d probe=%d hv=%d",
                                      nb, g, lat, probe, hv);
                        RUN_SCN(label,
                                scn_sm43(nb, g, lat, probe, hv, 1));
                    }

    // ── Back-to-back transactions (state pollution across
    //    transactions: accept_pend / xfr_armed / vh_buf_count) ───────
    struct { int nb, g, lat; } b2b[] = {
        {1, 0, 0}, {2, 3, 200}, {3, 17, 2000},
    };
    for (auto& c : b2b)
        for (int probe : {0, 1}) {
            std::snprintf(label, sizeof label,
                          "b2b x3 N=%d G=%d L=%d probe=%d",
                          c.nb, c.g, c.lat, probe);
            RUN_SCN(label, scn_sm43(c.nb, c.g, c.lat, probe != 0,
                                    false, 3));
        }

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
