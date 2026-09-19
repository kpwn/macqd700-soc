// tb_scsi_c96_mame_chunk.cpp — MEASURED-driver "bare repeat" chunk-loop
// replay for the 53C96 DATA-IN path.
//
// WHY THIS FILE EXISTS
// ────────────────────
// System 7.5.3 wedges on real hardware with the 53C96 left in:
//     cmd = 0x90 (Transfer Info | DMA), status = 0x01 (DATA IN, no INT,
//     no TC0), tcounter = 0x0010 (16, UNDECREMENTED), FIFO empty,
//     seq = 0, SD back-end idle with nothing owed.
//
// A full-boot capture of a HEALTHY MAME 7.5.3 boot (275,355 53C96
// register-access events over 45 s) establishes, by measurement:
//
//  1. The driver drains a DATA IN phase by loading the transfer counter
//     ONCE and then re-issuing Transfer Info (0x90) REPEATEDLY.  Each
//     0x90 moves exactly TC bytes.  Arithmetically verified: a READ(6)
//     of 19 blocks (9728 B) with TC = 16 issues exactly 608 x 0x90
//     (608 * 16 = 9728).
//
//  2. Two modes appear.  Common: data-phase TC = 0x0200 (512), one 0x90
//     per 512-byte block (>3400 transactions).  Rare — 20 transactions,
//     and the one that matches the HW wedge state — data-phase
//     TC = 0x0010 (16) with 32 x 0x90 per 512-byte transfer.
//
//  3. CRITICAL: between consecutive 0x90 chunks the driver writes
//     NOTHING.  The measured inter-chunk event sequence is pure polling:
//
//         W3=90{XFER_INFO/DMA}
//         R4=01{DATA_IN}          x5
//         R4=11{TC,DATA_IN}       x2
//         R4=91{INT,TC,DATA_IN}
//         R5=10{FUNC_CMPL}
//         W3=90{XFER_INFO/DMA}    <-- next chunk: a BARE repeat
//         R4=01 ...               (same again)
//
//     There is NO FLUSH_FIFO (W3=01) and NO transfer-counter rewrite
//     (W0/W1) between chunks.  The chip is expected to reload tcounter
//     from its TC latch on every command with bit 7 set, by itself.
//
// THE COVERAGE GAP THIS CLOSES
// ────────────────────────────
// tb_scsi_c96_read6.cpp's ROM 16-byte chunk loop (rom_flow_read6, the
// while() at ~line 496) re-arms with a FULL FLUSH_FIFO + TC rewrite
// before EVERY chunk:
//     reg_w(0x3, 0x01);  reg_w(0x1, 0x00);  reg_w(0x0, 0x10);
//     reg_w(0x3, 0x90);
// The real driver does none of that.  So the combination of
//   [ROM-form DMA select with tcount = 1 + deferred pdma CDB tail]
//   x [bare-repeat 16-byte chunk loop, TC latched once]
// was untested here: if scsi.v depended on the flush or the TC rewrite
// to re-arm its data-in supply bookkeeping, that test would stay green
// while hardware wedged — which is exactly the observed symptom.
//
// (tb_scsi_c96_sm43_chunk.cpp covers bare-repeat under the SM4.3 select
// form — TC = 2, four FIFO CDB bytes, one 16-bit pdma tail.  This file
// covers it under the ROM/7.0.1 select form — TC = 1, five FIFO CDB
// bytes, one 8-bit pdma tail — which is a different arming path into
// the same chunk loop.)
//
// This tb is MEASUREMENT.  A RED result here is a valid and valuable
// outcome; do not "fix" it by weakening an assertion.
//
// Build via:   make tb-scsi-c96-mame-chunk
//              (runs the POSITIVE CONTROL first, then the real pass)

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"
#include "Vtb_scsi_vhdd_sd___024root.h"

static Vtb_scsi_vhdd_sd* dut    = nullptr;
static int    n_pass = 0;
static int    n_fail = 0;
static bool   positive_control = false;
static std::string cur_scn = "?";
// Set by whichever assertion class fired first, so the positive-control
// pass can prove that the SPECIFIC check it targeted is the one that
// went red (not some unrelated collateral failure).
static std::string g_fail_kind = "";

static void note_fail(const char* kind) {
    if (g_fail_kind.empty()) g_fail_kind = kind;
}

// ─── Mocked SD backing store (cloned from tb_scsi_c96_read6.cpp) ─────
struct SdMock {
    std::vector<uint8_t> read_sector;     // byte pool used as read source
    std::vector<uint8_t> written_sector;
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    bool     present = true;
    bool     inject_error = false;
    // Real-SD pacing: cycles between rd_valid pulses.  The physical
    // sd_ctrl SPI path emits one byte every ~320 ns (~16 sys-clks at
    // 50 MHz).
    int      gap = 0;
    // Consumer back-pressure skid (see tb_scsi_c96_read6.cpp).
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
    dut->rst           = 1;
    dut->disk_num_lbas = 1048576u;
    dut->pb_addr       = 0;
    dut->pb_wdata      = 0;
    dut->pb_wr         = 0;
    dut->pb_rd         = 0;
    int keep_gap  = sd_mock.gap;
    int keep_skid = sd_mock.skid_max;
    sd_mock = SdMock();
    sd_mock.gap      = keep_gap;
    sd_mock.skid_max = keep_skid;
    sd_mock.skid     = keep_skid;
    for (int i = 0; i < 4; ++i) tick();
    dut->rst = 0;
    tick();
}

// ─── Register-aperture helpers ───────────────────────────────────────
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
    dut->pb_wr    = 0;
    dut->eval();
    // Mirror peripheral_bus.v's pulse-gating: it withholds the scsi_rd
    // PULSE until dma_rd_ready is high, rather than firing blind and
    // losing the beat.  Driving pb_rd unconditionally modelled a host
    // that ignores back-pressure -- one this SoC cannot actually build --
    // and so could not observe a supply-starved beat at all.
    for (int i = 0; i < 100000 && !dut->dma_rd_ready; ++i) {
        tick_n(1);
        dut->eval();
    }
    dut->pb_rd    = 1;
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

// ─── Internal-state access (verilator public_flat_rd), mirroring the
//     exact fields we read over JTAG on hardware ─────────────────────
#define U_SCSI(sig) (dut->rootp->tb_scsi_vhdd_sd__DOT__u_scsi__DOT__##sig)

// NOTE: reg_r() itself ticks the clock, so calling it from the dump
// perturbs the DUT by one cycle per read.  That is acceptable here —
// the dump only ever runs on a path that is already returning failure.
static void dump_state(const char* why) {
    uint8_t s  = reg_r(0x4);
    uint8_t is = reg_r(0x5);      // destructive: reading istatus clears it
    uint8_t sq = reg_r(0x6);
    uint8_t ff = reg_r(0x7);
    std::printf("  [%s] DUMP (%s): tcounter=0x%04x accept_pend=%u "
                "xfr_left=%u vh_buf_count=%u phase=%u drq=%d | "
                "status=0x%02x istatus=0x%02x seq=0x%02x fifo_flags=0x%02x "
                "| cons supplied=%u counted=%u accepted=%u drained=%u "
                "violations=%u | sd cnt=%d st=%d\n",
                cur_scn.c_str(), why,
                (unsigned)U_SCSI(c96_tcounter),
                (unsigned)U_SCSI(c96_accept_pend),
                (unsigned)U_SCSI(c96_xfr_left),
                (unsigned)U_SCSI(vh_buf_count),
                (unsigned)U_SCSI(phase),
                (int)dut->drq,
                s, is, sq, ff,
                (unsigned)U_SCSI(cons_supplied),
                (unsigned)U_SCSI(cons_counted),
                (unsigned)U_SCSI(cons_accepted),
                (unsigned)U_SCSI(cons_drained),
                (unsigned)U_SCSI(cons_violations),
                sd_mock.cnt, (int)sd_mock.st);
}

#define CHECK_EQ(kind, name, got, exp) do {                               \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp);         \
    if (_g != _e) {                                                       \
        std::printf("  [%s] FAIL %s: got 0x%02x, expected 0x%02x\n",      \
                    cur_scn.c_str(), name, _g, _e);                       \
        note_fail(kind);                                                  \
        dump_state(name);                                                 \
        return false;                                                     \
    }                                                                     \
} while (0)

#define CHECK_TRUE(kind, name, cond) do {                                 \
    if (!(cond)) {                                                        \
        std::printf("  [%s] FAIL %s\n", cur_scn.c_str(), name);           \
        note_fail(kind);                                                  \
        dump_state(name);                                                 \
        return false;                                                     \
    }                                                                     \
} while (0)

#define RUN(scn_name, expr) do {                                          \
    cur_scn = scn_name;                                                   \
    std::printf("[RUN ] %s\n", scn_name);                                 \
    bool _ok = (expr);                                                    \
    if (_ok) { ++n_pass; std::printf("[PASS] %s\n", scn_name); }          \
    else     { ++n_fail; std::printf("[FAIL] %s\n", scn_name); }          \
} while (0)

// 53C9x command opcodes
static constexpr uint8_t CI_COMPLETE   = 0x11;
static constexpr uint8_t CI_MSG_ACCEPT = 0x12;

// status / istatus bits
static constexpr uint8_t S_INTR       = 0x80;
static constexpr uint8_t S_TC0        = 0x10;
static constexpr uint8_t I_FUNCTION   = 0x08;
static constexpr uint8_t I_BUS        = 0x10;
static constexpr uint8_t I_DISCONNECT = 0x20;

static constexpr uint32_t SD_RAW_BASE_LBA = 8192u;
static constexpr uint8_t  TARGET_ID       = 6;

// ─────────────────────────────────────────────────────────────────────
// Step 1 — ROM-form DMA select (verbatim from tb_scsi_c96_read6.cpp's
// rom_flow_read6 steps 1-4): empty FIFO, tcount = 1, command 0xC1, five
// CDB bytes via the FIFO, the control byte via the pseudo-DMA port.
// ─────────────────────────────────────────────────────────────────────
static bool rom_select_read6(uint32_t lba, int blocks) {
    reg_w(0x4, TARGET_ID);
    reg_w(0x3, 0x01);               // FLUSH_FIFO
    reg_w(0x1, 0x00);               // tcount hi
    reg_w(0x0, 0x01);               // tcount lo = 1
    reg_w(0x3, 0xC1);               // DMA | CD_SELECT — FIFO EMPTY

    uint8_t v = 0;
    for (int i = 0; i < 64; ++i) { v = reg_r(0x6); if (v & 7) break; }
    CHECK_TRUE("select", "seq_step != 0 after empty-FIFO select",
               (v & 7) != 0);
    for (int i = 0; i < 64; ++i) { v = reg_r(0x4); if (v & 7) break; }
    CHECK_EQ("select", "status phase = COMMAND during CDB stuffing",
             v & 7, 0x2);

    reg_w(0x2, 0x08);
    reg_w(0x2, (lba >> 16) & 0x1f);
    reg_w(0x2, (lba >> 8) & 0xff);
    reg_w(0x2, lba & 0xff);
    reg_w(0x2, blocks & 0xff);
    shim_w(0x00);

    for (int i = 0; i < 64000; ++i) { v = reg_r(0x4); if (v & 0x80) break; }
    CHECK_EQ("select", "post-select status = INTR|TC0|DATA_IN", v, 0x91);
    CHECK_EQ("select", "fifo drained after CDB send", reg_r(0x7), 0x00);
    CHECK_EQ("select", "select-complete istatus", reg_r(0x5),
             I_FUNCTION | I_BUS);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Step 2 — THE MEASURED BARE-REPEAT CHUNK LOOP.
//
// The transfer counter latch is written EXACTLY ONCE, before the loop.
// Inside the loop the ONLY register WRITE is `reg_w(0x3, 0x90)`.
// There is deliberately no FLUSH_FIFO and no W0/W1 anywhere below —
// that is the whole point of this file.  Do not add one.
//
// `skip_drain_chunk >= 0` is the POSITIVE CONTROL knob: that chunk's
// 16 pseudo-DMA reads are skipped, which must make the per-chunk
// completion assertions go red.
// ─────────────────────────────────────────────────────────────────────
static bool bare_repeat_drain(std::vector<uint8_t>& out, int nchunks,
                              int skip_drain_chunk, int chunk_bytes = 16) {
    // ── ONCE, before the loop: TC latch = chunk_bytes ──
    reg_w(0x1, 0x00);
    reg_w(0x0, (uint8_t)chunk_bytes);
    // MAME's FIFO-flags register saturates at the real chip's 16-byte
    // FIFO depth (scsi.v:1158, ncr53c90.cpp:627).
    const uint32_t exp_ff = (chunk_bytes >= 16) ? 16u : (uint32_t)chunk_bytes;

    // ── MEASURED against live MAME 0.285 / macqd700, 2026-08-19 ───────
    // Directed differential script through tools/fuzz/scsi_fuzz.py's own
    // executors (tb_scsi_fuzz.cpp + tools/mame_scsi96_fuzz.lua), same
    // byte-identical op log to both sides, config3=0x04, sync_offset=0,
    // READ(6) 1 block, chunk armed with DMA|CI_XFER and NOT drained:
    //
    //   32-byte chunk : stat=01 (TC0 CLEAR) tclo=10 flags=10   BOTH SIDES
    //   16-byte chunk : stat=11 (TC0 SET  ) tclo=00 flags=10   BOTH SIDES
    //
    // tcounter decrements on FIFO PUSH (ncr53c90.cpp:471-477) and pushes
    // stop at fifo_pos == 16 (:626-627), so a chunk LARGER than the FIFO
    // physically cannot reach TC0 until the host drains.  Polling TC0
    // before the drain — which this loop used to do unconditionally — is
    // unsatisfiable on the GOLDEN MODEL too, not just on our RTL.
    const bool tc0_reachable_predrain = (chunk_bytes <= 16);

    for (int chunk = 0; chunk < nchunks; ++chunk) {
        const bool last_chunk = (chunk == nchunks - 1);
        // ── the ONLY write inside the loop ──
        reg_w(0x3, 0x90);           // DMA | Transfer Information

        // Poll for the chip to have staged everything this chunk can
        // stage.  For chunk_bytes <= 16 that coincides with TC0; for a
        // larger chunk the terminal pre-drain state is a FULL FIFO with
        // TC0 still clear (see the measurement above).
        bool staged = false;
        uint8_t s = 0;
        for (int i = 0; i < 200000; ++i) {
            s = reg_r(0x4);
            if (tc0_reachable_predrain) {
                if (s & S_TC0) { staged = true; break; }
            } else if ((reg_r(0x7) & 0x1f) == exp_ff) { staged = true; break; }
        }
        if (!staged) {
            std::printf("  [%s] FAIL chunk %d/%d: chip never staged the "
                        "chunk after a BARE repeat of 0x90 (status=0x%02x) "
                        "— WEDGE\n",
                        cur_scn.c_str(), chunk, nchunks, s);
            note_fail("chunk");
            dump_state("bare-repeat staging poll timeout");
            return false;
        }
        // MEASURED (32-byte chunk, both sides): TC0 CLEAR pre-drain and
        // tcounter parked at chunk_bytes - 16 (tclo=10 for 32).
        if (!tc0_reachable_predrain) {
            if (s & S_TC0) {
                std::printf("  [%s] FAIL chunk %d/%d: TC0 set pre-drain for "
                            "a %d-byte chunk (status=0x%02x); MAME parks at "
                            "TC0 CLEAR, tcounter=%d\n",
                            cur_scn.c_str(), chunk, nchunks, chunk_bytes, s,
                            chunk_bytes - 16);
                note_fail("chunk");
                dump_state("bare-repeat premature TC0");
                return false;
            }
            uint8_t tclo = reg_r(0x0);
            if (tclo != (uint8_t)(chunk_bytes - 16)) {
                std::printf("  [%s] FAIL chunk %d/%d: pre-drain tcounter lo "
                            "= 0x%02x, expected 0x%02x\n",
                            cur_scn.c_str(), chunk, nchunks, tclo,
                            (unsigned)(chunk_bytes - 16));
                note_fail("chunk");
                dump_state("bare-repeat pre-drain tcounter");
                return false;
            }
        }
        // Phase at the staging point.
        //
        // MEASURED (live MAME, probe on :scsi bus save item `ctrl` +
        // ncr53c96 `status`, composing status_r() = status | (ctrl & 7)
        // with ZERO perturbation — ncr53c90.cpp:status_r):
        //   mid-block chunk : R4 = 0x11  phase 1 DATA_IN   busctrl=0x0029
        //   LAST chunk      : R4 = 0x13  phase 3 STATUS    busctrl=0x002b
        // The last chunk of a 512-byte READ exhausts the block, so the
        // TARGET drops S_CTL->STATUS as soon as the 512th byte is ACKed —
        // which is the same event that sets TC0 — even though 16 bytes
        // are still sitting undrained in the chip FIFO.  status_r() ORs
        // the LIVE bus phase in, so the driver sees STATUS here.  The old
        // unconditional "must still be DATA IN" is only correct for
        // chunks that are not the last of the block.
        {
            uint8_t ph = s & 0x07;
            bool ok = last_chunk ? (ph == 0x01 || ph == 0x03) : (ph == 0x01);
            if (!ok) {
                std::printf("  [%s] FAIL chunk %d/%d: phase = %u "
                            "(status=0x%02x); expected DATA IN%s\n",
                            cur_scn.c_str(), chunk, nchunks, ph, s,
                            last_chunk ? " or STATUS (last chunk)" : "");
                note_fail("chunk");
                dump_state("bare-repeat phase at staging point");
                return false;
            }
        }

        // Wait for DRQ before the blind pseudo-DMA burst.
        bool drq_up = false;
        for (int i = 0; i < 4000; ++i) {
            if (dut->drq) { drq_up = true; break; }
            tick_n(1);
        }
        if (!drq_up) {
            std::printf("  [%s] FAIL chunk %d/%d: DRQ never rises before "
                        "the burst\n", cur_scn.c_str(), chunk, nchunks);
            note_fail("chunk");
            dump_state("bare-repeat DRQ timeout");
            return false;
        }

        // Golden trace also reads FIFO Flags = 0x10 here (>= 16 bytes
        // staged); 7.5.3's drain gate is BTST #4 on this register.
        uint8_t ff = reg_r(0x7);
        if (ff != exp_ff) {
            std::printf("  [%s] FAIL chunk %d/%d: FIFO Flags = 0x%02x, "
                        "expected 0x%02x\n",
                        cur_scn.c_str(), chunk, nchunks, ff, exp_ff);
            note_fail("chunk");
            dump_state("bare-repeat fifo flags");
            return false;
        }

        // chunk_bytes pseudo-DMA port reads (MOVE.W pairs on real HW).
        if (chunk == skip_drain_chunk) {
            std::printf("  [%s] POSITIVE CONTROL: skipping chunk %d's "
                        "%d-byte drain on purpose\n",
                        cur_scn.c_str(), chunk, chunk_bytes);
            for (int i = 0; i < chunk_bytes; ++i) out.push_back(0x00);
        } else {
            for (int i = 0; i < chunk_bytes; ++i) out.push_back(shim_r());
        }

        // Poll status for INT (bit 7) — asserts only after the drain.
        bool intr = false;
        for (int i = 0; i < 200000; ++i) {
            s = reg_r(0x4);
            if (s & S_INTR) { intr = true; break; }
        }
        if (!intr) {
            std::printf("  [%s] FAIL chunk %d/%d: post-drain INT never "
                        "fires (status=0x%02x) — WEDGE\n",
                        cur_scn.c_str(), chunk, nchunks, s);
            note_fail("chunk");
            dump_state("bare-repeat post-drain INT timeout");
            return false;
        }
        // For a chunk larger than the FIFO, TC0 is only reachable AFTER
        // the drain — assert it here so the "latch is load-bearing"
        // scenario still proves the full chunk_bytes moved per 0x90.
        if (!tc0_reachable_predrain && !(s & S_TC0)) {
            std::printf("  [%s] FAIL chunk %d/%d: TC0 still clear after "
                        "draining %d bytes (status=0x%02x)\n",
                        cur_scn.c_str(), chunk, nchunks, chunk_bytes, s);
            note_fail("chunk");
            dump_state("bare-repeat post-drain TC0");
            return false;
        }
        uint8_t is = reg_r(0x5);
        if (is != I_BUS) {
            std::printf("  [%s] FAIL chunk %d/%d: istatus = 0x%02x, "
                        "expected 0x10 (FUNC_CMPL / BUS SERVICE)\n",
                        cur_scn.c_str(), chunk, nchunks, is);
            note_fail("chunk");
            dump_state("bare-repeat chunk istatus");
            return false;
        }
    }
    return true;
}

// ─── Status/message epilogue ─────────────────────────────────────────
static bool status_epilogue() {
    bool at_status = false;
    for (int i = 0; i < 4000; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x03) { at_status = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("epilogue", "phase leaves DATA IN for STATUS", at_status);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("epilogue", "CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5),
             I_FUNCTION);
    CHECK_EQ("epilogue", "FIFO has status+msg", reg_r(0x7), 0x02);
    CHECK_EQ("epilogue", "status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("epilogue", "msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("epilogue", "CI_MSG_ACCEPT istatus = I_DISCONNECT",
             reg_r(0x5), I_DISCONNECT);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// The scenario: 512-byte READ(6) drained as 32 x 16-byte BARE-repeat
// chunks.
//
//   corrupt_expected  — POSITIVE CONTROL: perturb one byte of the
//                       expectation so the data comparison must fire.
//   skip_drain_chunk  — POSITIVE CONTROL: skip one chunk's drain so the
//                       per-chunk completion assertions must fire.
// ─────────────────────────────────────────────────────────────────────
static bool scn_mame_bare_repeat(uint32_t lba, int gap,
                                 bool corrupt_expected,
                                 int skip_drain_chunk,
                                 int chunk_bytes = 16) {
    sd_mock.gap = gap;
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x3C ^ (i & 0xFF) ^ (i >> 4));

    if (!rom_select_read6(lba, 1)) return false;

    std::vector<uint8_t> data;
    if (!bare_repeat_drain(data, 512 / chunk_bytes, skip_drain_chunk,
                           chunk_bytes)) {
        std::printf("  [%s] drained %u/512 bytes before the wedge\n",
                    cur_scn.c_str(), (unsigned)data.size());
        return false;
    }
    if (!status_epilogue()) return false;

    CHECK_EQ("data", "drained the full 512-byte block",
             (uint32_t)data.size(), 512u);

    // Byte-identical against the mocked backing store's sector.
    std::vector<uint8_t> expect = sd_mock.read_sector;
    if (corrupt_expected) {
        expect[137] ^= 0xFF;
        std::printf("  [%s] POSITIVE CONTROL: expectation byte[137] "
                    "corrupted on purpose\n", cur_scn.c_str());
    }
    for (int i = 0; i < 512; ++i) {
        if (data[i] != expect[i]) {
            std::printf("  [%s] FAIL byte[%d]: got 0x%02x, expected "
                        "0x%02x\n", cur_scn.c_str(), i, data[i], expect[i]);
            note_fail("data");
            dump_state("data mismatch");
            return false;
        }
    }

    CHECK_EQ("data", "sd_cmd_type was CMD17 (1)", sd_mock.last_cmd_type, 1);
    CHECK_EQ("data", "sd_lba was 8192 + lba",
             sd_mock.last_lba, SD_RAW_BASE_LBA + lba);
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; ++i)
        if (!std::strcmp(argv[i], "--positive-control"))
            positive_control = true;

    dut = new Vtb_scsi_vhdd_sd;

    if (positive_control) {
        std::printf("=== 53C96 MAME bare-repeat chunk loop — POSITIVE "
                    "CONTROL pass (failures below are the POINT) ===\n");

        // Control A — the data comparison must be able to fail.
        g_fail_kind.clear();
        RUN("pc_data_compare_can_fail",
            scn_mame_bare_repeat(/*lba=*/0, /*gap=*/0,
                                 /*corrupt_expected=*/true,
                                 /*skip_drain_chunk=*/-1));
        bool data_fired = (g_fail_kind == "data");

        // Control B — the per-chunk completion checks must be able to
        // fail (skip chunk 7's drain: INT can never fire for it).
        g_fail_kind.clear();
        RUN("pc_chunk_completion_can_fail",
            scn_mame_bare_repeat(/*lba=*/0, /*gap=*/0,
                                 /*corrupt_expected=*/false,
                                 /*skip_drain_chunk=*/7));
        bool chunk_fired = (g_fail_kind == "chunk");

        std::printf("\npositive control: data-compare fired=%d "
                    "chunk-completion fired=%d (%d/%d scenarios went "
                    "red as required)\n",
                    (int)data_fired, (int)chunk_fired, n_fail,
                    n_pass + n_fail);
        int rc = (data_fired && chunk_fired && n_fail == 2) ? 0 : 1;
        if (rc != 0)
            std::printf("POSITIVE CONTROL DID NOT FIRE — this tb measures "
                        "nothing.\n");
        else
            std::printf("POSITIVE CONTROL OK: both assertion classes can "
                        "fail.\n");
        delete dut;
        return rc;
    }

    std::printf("=== 53C96 MAME bare-repeat chunk loop "
                "(TC latched ONCE, 32 x bare 0x90) ===\n");

    // Instant SD supply — isolates pure chip-side re-arm semantics.
    RUN("bare_repeat_32x16_fast_sd",
        scn_mame_bare_repeat(/*lba=*/0, /*gap=*/0, false, -1));
    // Realistic ~320 ns/byte SPI pacing, so the chunk loop out-runs the
    // provider and the re-arm has to survive a genuinely empty ring.
    RUN("bare_repeat_32x16_slow_sd",
        scn_mame_bare_repeat(/*lba=*/3, /*gap=*/20, false, -1));
    // Discriminator: the SAME bare-repeat loop with the latch set to 32
    // must move 32 bytes per 0x90, not 16.  Proves the chip reloads
    // tcounter FROM THE LATCH on each command rather than happening to
    // work for a hardcoded/leftover 16.
    RUN("bare_repeat_16x32_latch_is_load_bearing",
        scn_mame_bare_repeat(/*lba=*/7, /*gap=*/20, false, -1,
                             /*chunk_bytes=*/32));

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
