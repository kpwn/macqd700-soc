// tb_scsi_c96_stall_shapes.cpp — directed reproductions of the three
// 53C96 stall shapes fingerprinted on hardware 2026-09-06/07:
//
//   (i)   FIFO-residue INT park: cmd=0x90, TC0 reached, residue bytes in
//         the FIFO, INT never asserts.  ROM spins untimed at 0x40899706.
//   (ii)  Select-retry cycle: cmd=0xC1, Ticks-deadlined, loops forever.
//   (iii) DMA-select CDB-tail DREQ spin at ROM 0x40898ea8: status=0x81,
//         FIFO=16, DRQ suppressed.
//
// ROOT-CAUSE MODEL THIS FILE ENCODES (see the scenario comments for the
// per-shape evidence):
//
//   The ROM's chunk drain is a BLIND burst of 16-bit MOVE.W reads from
//   the DAFB pseudo-DMA port (ROM 0x40899628: 16x movew %a1@(256),%a2+,
//   Duff-dispatched, no DRQ check, no TC0 poll first).  scsi.v replays a
//   host word as two byte beats and, per MAME dma16_r (ncr53c90.cpp:1329
//   `if (fifo_pos < 2) return dma_r() | 0xff00;`), DEGRADES the access
//   to a single-byte pop when fifo_pos < 2 at the high beat.  On MAME
//   the chip's supply is instantaneous — recv_byte() refills the FIFO
//   before the CPU's next instruction — so a blind burst never observes
//   fifo_pos < 2 while the transfer counter still owes bytes; the
//   degrade exists only for the true tail.  Our supply has real latency
//   (vhdd ring + SD pacing + 1 accept/cycle), so a word beat CAN land at
//   fifo_pos == 1 with tcounter != 0.  Every such beat pops ONE byte
//   while the host advances its pointer by TWO: one byte of transfer
//   leaks into FIFO residue, silently corrupting the block AND — once
//   TC0 sets — leaving residue > the BUSMD_1 drq threshold, so the
//   chunk-completion hook (`TC0 && !drq`, MAME INIT_XFR_BUS_COMPLETE
//   `if (dma_command && drq) break;`) can never fire.  INT never rises.
//
//   Arithmetic check against the hardware fingerprint (shape i): a
//   512-byte chunk drained as 256 blind words with 9 degraded beats
//   gives pops = 2*(256-9)+9 = 503, accepts = 512 (TC0 set), residue =
//   512-503 = 9 — exactly the measured status=0x11 / fifo=9 / INT-clear
//   park, twice, identically (deterministic pacing => deterministic
//   dip count).
//
//   Shapes (ii)/(iii) are DOWNSTREAM surfaces of the same park in
//   deadline-bounded driver contexts: the driver aborts the parked
//   transfer and retries.  The retry either cycles forever (ii) or, in
//   weaves where slot 0 of the MAME-faithful 2-deep command queue is
//   still occupied by the parked command, wedges at the ROM's UNTIMED
//   DAFB-DRQ spin (iii): the retried 0xC1 queues (or is GROSS-dropped)
//   and never dispatches, so the select-phase DRQ disjunct
//   (c96_sel_active && c96_sel_dma && fifo != 16) is dead via
//   sel_active=0 — and the ROM's CDB stuffs land in the plain FIFO,
//   accreting toward 16.  Scenarios s2/s3 measure exactly those
//   dynamics.
//
// The f8ad8c33 lesson is honoured throughout: every pseudo-DMA beat
// helper presents its address/lo-beat first and HONOURS dma_rd_ready
// before pulsing pb_rd, exactly as peripheral_bus.v's
// rd_scsi_dma_shim_active does (it gates the pulse on scsi_dma_rd_ready
// for BOTH halves of a split word).  A harness that ignores the
// handshake models a host this SoC cannot build and cannot observe
// supply-starvation bugs at all.
//
// Build via:   make tb-scsi-c96-stall-shapes
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
static std::string g_fail_kind = "";

static void note_fail(const char* kind) {
    if (g_fail_kind.empty()) g_fail_kind = kind;
}

// ─── Mocked SD backing store (cloned from tb_scsi_c96_mame_chunk.cpp,
//     plus a periodic-stall knob to model ring-refill dips) ───────────
struct SdMock {
    std::vector<uint8_t> read_sector;
    std::vector<uint8_t> written_sector;
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    bool     present = true;
    int      gap = 0;          // cycles between rd_valid pulses
    // Periodic supply stall: after every `stall_every` bytes delivered,
    // go silent for `stall_len` cycles.  Models the real provider's
    // occasional refill hiccups (SPI block boundaries, CDC ladder,
    // pre-empting ISRs) that let the blind word burst catch the FIFO at
    // occupancy 1.  0 = disabled.
    int      stall_every = 0;
    int      stall_len   = 0;
    int      skid_max = 2;
    int      skid = 2;

    enum class State { Idle, Reading, Writing, Done } st = State::Idle;
    int cnt = 0;
    int total = 512;
    int delay = 0;
    int gap_ctr = 0;
    int stall_ctr = 0;
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
        sd_mock.stall_ctr     = 0;
        if (dut->sd_cmd_type == 1) {
            sd_mock.st    = SdMock::State::Reading;
            sd_mock.total = 512;
        } else if (dut->sd_cmd_type == 2) {
            sd_mock.st    = SdMock::State::Reading;
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
        if (sd_mock.stall_ctr > 0) { --sd_mock.stall_ctr; return; }
        if (sd_mock.gap_ctr > 0) { --sd_mock.gap_ctr; return; }
        if (dut->sd_rd_ready) {
            sd_mock.skid = sd_mock.skid_max;
        } else if (sd_mock.skid > 0) {
            --sd_mock.skid;
        } else {
            return;
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
            sd_mock.gap_ctr = sd_mock.gap;
            if (sd_mock.stall_every > 0 &&
                (sd_mock.cnt % sd_mock.stall_every) == 0)
                sd_mock.stall_ctr = sd_mock.stall_len;
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
    dut->rst           = 1;
    dut->disk_num_lbas = 1048576u;
    dut->pb_addr       = 0;
    dut->pb_wdata      = 0;
    dut->pb_wr         = 0;
    dut->pb_rd         = 0;
    dut->pb_dma16_lo_beat = 0;
    int keep_gap  = sd_mock.gap;
    int keep_se   = sd_mock.stall_every;
    int keep_sl   = sd_mock.stall_len;
    sd_mock = SdMock();
    sd_mock.gap         = keep_gap;
    sd_mock.stall_every = keep_se;
    sd_mock.stall_len   = keep_sl;
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

// ─── Pseudo-DMA beats, peripheral_bus.v-faithful ─────────────────────
// Present the address (and lo-beat flag) FIRST, then wait for
// dma_rd_ready before pulsing pb_rd — rd_scsi_dma_shim_active gates the
// pulse on scsi_dma_rd_ready for every DMA-shim beat, split-word halves
// included.  `budget` bounds the wait; a timeout is reported to the
// caller (on hardware it would be a CPU stalled on the aperture).
static const int BEAT_BUDGET = 400000;

static bool shim_beat_r(uint16_t addr, bool lo_beat, uint8_t* out) {
    dut->pb_addr          = addr;
    dut->pb_dma16_lo_beat = lo_beat ? 1 : 0;
    dut->pb_wr            = 0;
    dut->eval();
    int i = 0;
    for (; i < BEAT_BUDGET && !dut->dma_rd_ready; ++i) tick();
    if (!dut->dma_rd_ready) {
        dut->pb_dma16_lo_beat = 0;
        return false;
    }
    dut->pb_rd = 1;
    tick();
    dut->pb_rd = 0;
    dut->pb_dma16_lo_beat = 0;
    dut->eval();
    *out = dut->pb_rdata & 0xff;
    return true;
}

static bool shim_r16(uint16_t* out) {
    uint8_t hi = 0, lo = 0;
    if (!shim_beat_r(0x100, false, &hi)) return false;
    if (!shim_beat_r(0x101, true,  &lo)) return false;
    *out = ((uint16_t)hi << 8) | lo;
    return true;
}

static bool shim_r8(uint8_t* out) {
    return shim_beat_r(0x100, false, out);
}

static bool shim_w8(uint8_t v) {
    dut->pb_addr          = 0x100;
    dut->pb_dma16_lo_beat = 0;
    dut->pb_wdata         = v;
    dut->eval();
    int i = 0;
    for (; i < BEAT_BUDGET && !dut->dma_wr_ready; ++i) tick();
    if (!dut->dma_wr_ready) return false;
    dut->pb_wr = 1;
    tick();
    dut->pb_wr = 0;
    dut->pb_wdata = 0;
    dut->eval();
    return true;
}

// ─── Internal-state peeks (same fields the JTAG fingerprints read) ───
#define U_SCSI(sig) (dut->rootp->tb_scsi_vhdd_sd__DOT__u_scsi__DOT__##sig)

static void dump_state(const char* why) {
    uint8_t s  = reg_r(0x4);
    uint8_t sq = reg_r(0x6);
    uint8_t ff = reg_r(0x7);
    std::printf("  [%s] DUMP (%s): tcounter=0x%04x phase=%u drq=%d "
                "sel_active=%u cmdpos=%u | status=0x%02x seq=0x%02x "
                "fifo_flags=0x%02x | sd cnt=%d st=%d\n",
                cur_scn.c_str(), why,
                (unsigned)U_SCSI(c96_tcounter),
                (unsigned)U_SCSI(phase),
                (int)dut->drq,
                (unsigned)U_SCSI(c96_sel_active),
                (unsigned)U_SCSI(c96_command_pos),
                s, sq, ff, sd_mock.cnt, (int)sd_mock.st);
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

static constexpr uint8_t CI_COMPLETE   = 0x11;
static constexpr uint8_t CI_MSG_ACCEPT = 0x12;
static constexpr uint8_t S_INTR        = 0x80;
static constexpr uint8_t S_TC0_BIT     = 0x10;
static constexpr uint8_t I_FUNCTION    = 0x08;
static constexpr uint8_t I_BUS         = 0x10;
static constexpr uint8_t I_DISCONNECT  = 0x20;

static constexpr uint32_t SD_RAW_BASE_LBA = 8192u;
static constexpr uint8_t  TARGET_ID       = 6;

// ─── ROM-form DMA select (ROM 0x40898dc8..0x40898eb4 replayed) ───────
// W4=ID, W3=FLUSH, W1/W0 tcount=1, W3=0xC1; gate on seq!=0 &&
// status[2:0]==COMMAND; five CDB bytes via reg 2; the control byte via
// the pseudo-DMA port once DAFB DRQ (our drq line) rises.
static bool rom_select_read6(uint32_t lba, int blocks) {
    reg_w(0x4, TARGET_ID);
    reg_w(0x3, 0x01);               // FLUSH_FIFO
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);               // tcount = 1
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
    // ROM 0x40898ea8: untimed DAFB DRQ spin before the pdma tail byte.
    bool drq_up = false;
    for (int i = 0; i < 20000; ++i) {
        if (dut->drq) { drq_up = true; break; }
        tick();
    }
    CHECK_TRUE("select", "DRQ rises for the CDB tail byte", drq_up);
    CHECK_TRUE("select", "pdma tail byte accepted", shim_w8(0x00));

    for (int i = 0; i < 64000; ++i) { v = reg_r(0x4); if (v & 0x80) break; }
    CHECK_EQ("select", "post-select status = INTR|TC0|DATA_IN", v, 0x91);
    CHECK_EQ("select", "fifo drained after CDB send", reg_r(0x7), 0x00);
    CHECK_EQ("select", "select-complete istatus", reg_r(0x5),
             I_FUNCTION | I_BUS);
    return true;
}

// ─── ROM chunk drain, word-beat form (ROM 0x40899606..0x4089966e) ────
// Per chunk: W0/W1 = tcount, W3 = 0x90, then chunk/2 BLIND word reads
// (paced only by dma_rd_ready, as peripheral_bus paces the CPU), then
// the UNTIMED INT wait (bounded here so a park is a reported failure,
// not a hung tb), then istatus must read I_BUS.
//
// skip_words > 0 withholds that many trailing word beats of chunk 0 —
// the POSITIVE CONTROL for park detection (a host that under-drains
// must park the completion; that is MAME-faithful).
static bool rom_word_chunk_drain(std::vector<uint8_t>& out, int total_bytes,
                                 int chunk_bytes, int skip_words,
                                 bool* parked, uint8_t* park_status,
                                 uint8_t* park_fifo) {
    *parked = false;
    const int nchunks = total_bytes / chunk_bytes;
    for (int chunk = 0; chunk < nchunks; ++chunk) {
        reg_w(0x0, (uint8_t)(chunk_bytes & 0xff));
        reg_w(0x1, (uint8_t)((chunk_bytes >> 8) & 0xff));
        reg_w(0x3, 0x90);           // DMA | Transfer Information

        int words = chunk_bytes / 2;
        if (chunk == 0 && skip_words > 0) words -= skip_words;
        for (int w = 0; w < words; ++w) {
            uint16_t v = 0;
            if (!shim_r16(&v)) {
                std::printf("  [%s] FAIL chunk %d word %d: beat withheld "
                            "past budget (dma_rd_ready never rose)\n",
                            cur_scn.c_str(), chunk, w);
                note_fail("beat");
                dump_state("word beat withheld");
                return false;
            }
            out.push_back((uint8_t)(v >> 8));
            out.push_back((uint8_t)(v & 0xff));
        }

        // ROM 0x4089966e -> 0x40899704: untimed INT wait.
        bool intr = false;
        uint8_t s = 0;
        for (int i = 0; i < 300000; ++i) {
            s = reg_r(0x4);
            if (s & S_INTR) { intr = true; break; }
        }
        if (!intr) {
            *parked      = true;
            *park_status = s;
            *park_fifo   = reg_r(0x7) & 0x1f;
            std::printf("  [%s] PARK at chunk %d/%d: INT never fires — "
                        "status=0x%02x fifo=%u tcounter=0x%04x (shape-i "
                        "signature)\n",
                        cur_scn.c_str(), chunk, nchunks, s,
                        (unsigned)*park_fifo,
                        (unsigned)U_SCSI(c96_tcounter));
            note_fail("park");
            dump_state("post-drain INT park");
            return false;
        }
        uint8_t is = reg_r(0x5);
        if ((is & 0x30) != I_BUS) {
            std::printf("  [%s] FAIL chunk %d/%d: istatus=0x%02x, ROM "
                        "expects I_BUS\n", cur_scn.c_str(), chunk, nchunks,
                        is);
            note_fail("istatus");
            dump_state("chunk istatus");
            return false;
        }
    }
    return true;
}

static bool status_epilogue() {
    bool at_status = false;
    for (int i = 0; i < 8000; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x03) { at_status = true; break; }
        tick_n(1);
    }
    CHECK_TRUE("epilogue", "phase leaves DATA IN for STATUS", at_status);
    reg_w(0x3, CI_COMPLETE);
    CHECK_EQ("epilogue", "CI_COMPLETE istatus = I_FUNCTION", reg_r(0x5),
             I_FUNCTION);
    CHECK_EQ("epilogue", "status = GOOD",  reg_r(0x2), 0x00);
    CHECK_EQ("epilogue", "msg = COMPLETE", reg_r(0x2), 0x00);
    reg_w(0x3, CI_MSG_ACCEPT);
    CHECK_EQ("epilogue", "CI_MSG_ACCEPT istatus = I_DISCONNECT",
             reg_r(0x5), I_DISCONNECT);
    return true;
}

static void fill_pattern() {
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0xA5 ^ (i & 0xFF) ^ (i >> 3));
}

// ─────────────────────────────────────────────────────────────────────
// Shape (i): 512-byte READ(6), ROM word-beat drain, supply with
// periodic dips.  Pre-fix: word beats landing at fifo_pos==1 with
// tcounter!=0 degrade to 1-byte pops, leak residue, and park the TC0
// completion behind drq — INT never fires (status=0x11, fifo=residue).
// Post-fix: the degrade is suppressed while the armed DMA-in transfer
// still owes bytes and the supply has them (the low beat waits out the
// supply via c96_shim_rd_starved instead), so every chunk completes and
// the block is byte-exact.
// ─────────────────────────────────────────────────────────────────────
static bool scn_shape1(uint32_t lba, int chunk_bytes, int gap,
                       int stall_every, int stall_len,
                       bool corrupt_expected, int skip_words) {
    sd_mock.gap         = gap;
    sd_mock.stall_every = stall_every;
    sd_mock.stall_len   = stall_len;
    reset();
    fill_pattern();

    if (!rom_select_read6(lba, 1)) return false;

    std::vector<uint8_t> data;
    bool parked = false;
    uint8_t ps = 0, pf = 0;
    if (!rom_word_chunk_drain(data, 512, chunk_bytes, skip_words,
                              &parked, &ps, &pf)) {
        std::printf("  [%s] drained %u/512 bytes before the failure\n",
                    cur_scn.c_str(), (unsigned)data.size());
        return false;
    }
    if (!status_epilogue()) return false;

    CHECK_EQ("data", "drained the full 512-byte block",
             (uint32_t)data.size(), 512u);
    std::vector<uint8_t> expect = sd_mock.read_sector;
    if (corrupt_expected) {
        expect[211] ^= 0xFF;
        std::printf("  [%s] POSITIVE CONTROL: expectation byte[211] "
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
    CHECK_EQ("data", "sd_lba was 8192 + lba",
             sd_mock.last_lba, SD_RAW_BASE_LBA + lba);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Shape (ii) exit path: park a transfer deterministically (under-drain,
// modelling a driver whose Ticks deadline expired mid-chunk), then
// replay the ROM's ABORT path (0x40899086/0x408990a2: drain words while
// DRQ, poll INT, read istatus, FLUSH) and prove a fresh select + READ
// completes.  This is the recovery the select-retry cycle depends on:
// if it works, shape (ii) can only loop forever when every retry parks
// AGAIN — i.e. (ii) is downstream of shape (i)'s park, not a separate
// select-engine defect.
// ─────────────────────────────────────────────────────────────────────
// Shape (i) AT THE HARDWARE'S TRANSFER SIZE.
//
// Coverage gap this closes (2026-09-08): every other scenario in this
// file drains a SINGLE 512-byte block.  The transfer that actually
// stalls on hardware is MULTI-BLOCK -- the ROM's SCSI driver reads 6
// blocks (3072 bytes) per command, and the measured board failures pin
// at 0x40898ea8 (DRQ wait) and 0x40899706 (completion wait) part-way
// through such a transfer.  A single-block scenario never crosses a
// block boundary, so it cannot exercise the ring re-staging that has to
// happen BETWEEN blocks while the supply is dipping.
//
// The two tbs that DO issue multi-block reads (tb_scsi.cpp,
// tb_scsi_sd_e2e.cpp) model an INSTANTANEOUS supply, so they never
// starve it.  This scenario is the missing intersection: multi-block
// transfer + dipping supply.
static bool scn_shape1_multiblock(uint32_t lba, int blocks, int chunk_bytes,
                                  int gap, int stall_every, int stall_len) {
    const int total_bytes = 512 * blocks;
    sd_mock.gap         = gap;
    sd_mock.stall_every = stall_every;
    sd_mock.stall_len   = stall_len;
    reset();
    fill_pattern();

    if (!rom_select_read6(lba, blocks)) return false;

    std::vector<uint8_t> data;
    bool parked = false;
    uint8_t ps = 0, pf = 0;
    if (!rom_word_chunk_drain(data, total_bytes, chunk_bytes, 0,
                              &parked, &ps, &pf)) {
        std::printf("  [%s] drained %u/%d bytes before the failure "
                    "(parked=%d status=0x%02x fifo=%u)\n",
                    cur_scn.c_str(), (unsigned)data.size(), total_bytes,
                    (int)parked, ps, (unsigned)pf);
        return false;
    }
    if (!status_epilogue()) return false;

    CHECK_EQ("data", "drained the full multi-block transfer",
             (uint32_t)data.size(), (uint32_t)total_bytes);

    // The mock serves read_sector[cnt % 512], so every block repeats the
    // pattern.  A byte-exact compare across ALL blocks is what catches the
    // silent single-byte-degrade corruption, not merely the stall.
    for (int i = 0; i < total_bytes; ++i) {
        uint8_t e = sd_mock.read_sector[i % 512];
        if (data[i] != e) {
            std::printf("  [%s] FAIL byte[%d] (block %d off %d): got "
                        "0x%02x, expected 0x%02x\n", cur_scn.c_str(), i,
                        i / 512, i % 512, data[i], e);
            note_fail("data");
            dump_state("multi-block data mismatch");
            return false;
        }
    }
    return true;
}

// MANY CONSECUTIVE COMMANDS -- the "fails on the 150th" signature.
//
// Every other scenario in this file issues exactly ONE command.  The
// hardware failure is deterministic on the ~150th disk command of a
// boot (measured: sd done_count=149, the 150th never completes), which
// is the signature of state ACCUMULATING ACROSS commands -- a residue,
// pointer, or credit that is not fully restored at command teardown --
// rather than anything wrong within a single transfer.  A single-command
// scenario cannot see that class of bug by construction.
//
// Each iteration is a full select + multi-block drain + status epilogue,
// byte-checked, so the first command that either stalls or corrupts
// fails the scenario and reports WHICH command index it was.
static bool scn_many_commands(int ncmds, int blocks, int chunk_bytes,
                              int gap, int stall_every, int stall_len) {
    const int total_bytes = 512 * blocks;
    sd_mock.gap         = gap;
    sd_mock.stall_every = stall_every;
    sd_mock.stall_len   = stall_len;
    reset();
    fill_pattern();

    for (int cmd = 0; cmd < ncmds; ++cmd) {
        uint32_t lba = 0x1304;          // the ROM re-reads the same LBA
        if (!rom_select_read6(lba, blocks)) {
            std::printf("  [%s] SELECT failed on command #%d\n",
                        cur_scn.c_str(), cmd + 1);
            return false;
        }
        std::vector<uint8_t> data;
        bool parked = false;
        uint8_t ps = 0, pf = 0;
        if (!rom_word_chunk_drain(data, total_bytes, chunk_bytes, 0,
                                  &parked, &ps, &pf)) {
            std::printf("  [%s] STALLED on command #%d after %u/%d bytes "
                        "(parked=%d status=0x%02x fifo=%u)\n",
                        cur_scn.c_str(), cmd + 1, (unsigned)data.size(),
                        total_bytes, (int)parked, ps, (unsigned)pf);
            return false;
        }
        if (!status_epilogue()) {
            std::printf("  [%s] epilogue failed on command #%d\n",
                        cur_scn.c_str(), cmd + 1);
            return false;
        }
        for (int i = 0; i < total_bytes; ++i) {
            uint8_t e = sd_mock.read_sector[i % 512];
            if (data[i] != e) {
                std::printf("  [%s] CORRUPT on command #%d byte[%d]: got "
                            "0x%02x expected 0x%02x\n", cur_scn.c_str(),
                            cmd + 1, i, data[i], e);
                note_fail("data");
                return false;
            }
        }
    }
    std::printf("  [%s] %d consecutive commands clean\n",
                cur_scn.c_str(), ncmds);
    return true;
}

static bool scn_shape2_abort_recovers() {
    sd_mock.gap = 0;
    sd_mock.stall_every = 0;
    sd_mock.stall_len = 0;
    reset();
    fill_pattern();

    if (!rom_select_read6(0, 1)) return false;

    // Arm one 16-byte chunk and drain only 3 of 8 words: residue 10.
    reg_w(0x0, 16); reg_w(0x1, 0);
    reg_w(0x3, 0x90);
    for (int w = 0; w < 3; ++w) {
        uint16_t v = 0;
        CHECK_TRUE("abort", "partial drain beat", shim_r16(&v));
    }
    // Driver deadline expires here: NO INT wait.  Confirm the park is
    // real first (INT must be low with residue above the threshold).
    tick_n(2000);
    uint8_t s = reg_r(0x4);
    CHECK_TRUE("abort", "park precondition: INT low with residue",
               (s & S_INTR) == 0);
    CHECK_TRUE("abort", "park precondition: residue staged",
               (reg_r(0x7) & 0x1f) >= 2);

    // ROM abort path 0x408990a2: while !INT { if DRQ pop a word }.
    bool intr = false;
    for (int i = 0; i < 200000; ++i) {
        s = reg_r(0x4);
        if (s & S_INTR) { intr = true; break; }
        if (dut->drq) {
            uint16_t v = 0;
            if (!shim_r16(&v)) break;
        } else {
            tick_n(1);
        }
    }
    CHECK_TRUE("abort", "abort drain reaches INT (0x40899704 returns)",
               intr);
    (void)reg_r(0x5);               // istatus service (pops slot 0)
    reg_w(0x3, 0x01);               // FLUSH (ROM 0x408996d2)
    tick_n(50);

    // The bus is still CONNECTED (the target holds BSY with 496 bytes of
    // the block owed) — a select here is ILLEGAL on the real chip and on
    // MAME (check_valid_command: CD_SELECT is a disconnected-state
    // command, invalid in MODE_I), which is why the ROM's dispatcher
    // (0x40898fae) runs the bus to completion phase by phase before any
    // retry.  Model that: finish the DATA IN phase with one big DMA
    // chunk, then the status/message epilogue to bus free.
    {
        reg_w(0x0, (uint8_t)(496 & 0xff));
        reg_w(0x1, (uint8_t)(496 >> 8));
        reg_w(0x3, 0x90);
        for (int w = 0; w < 248; ++w) {
            uint16_t v = 0;
            CHECK_TRUE("abort", "run-to-completion drain beat",
                       shim_r16(&v));
        }
        bool done = false;
        for (int i = 0; i < 300000; ++i) {
            if (reg_r(0x4) & S_INTR) { done = true; break; }
        }
        CHECK_TRUE("abort", "run-to-completion chunk completes", done);
        CHECK_EQ("abort", "run-to-completion istatus = I_BUS",
                 reg_r(0x5) & 0x30, I_BUS);
        if (!status_epilogue()) return false;
    }

    // The retry the select-cycle performs: full fresh transaction.
    if (!rom_select_read6(3, 1)) return false;
    std::vector<uint8_t> data;
    bool parked = false; uint8_t ps = 0, pf = 0;
    if (!rom_word_chunk_drain(data, 512, 16, 0, &parked, &ps, &pf))
        return false;
    if (!status_epilogue()) return false;
    CHECK_EQ("abort", "retry read is byte-complete",
             (uint32_t)data.size(), 512u);
    for (int i = 0; i < 512; ++i) {
        if (data[i] != sd_mock.read_sector[i]) {
            std::printf("  [%s] FAIL retry byte[%d]: got 0x%02x expected "
                        "0x%02x\n", cur_scn.c_str(), i, data[i],
                        sd_mock.read_sector[i]);
            note_fail("abort");
            return false;
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Shape (iii) mechanism: a transfer whose completion INT was MISSED
// (the driver's Ticks deadline expired, or an interrupt-context istatus
// read raced — the p143-proven steal) leaves the COMPLETED command in
// queue slot 0: MAME pops the queue only on an istatus read that
// returns nonzero (istatus_r, ncr53c90.cpp:1117), and the driver never
// performed one.  The retry then writes FLUSH — which QUEUES in slot 1
// and does NOT execute (MAME command_w starts a command only at
// command_pos==1) — and 0xC1, which is GROSS-dropped on the full queue.
// The select never dispatches, so the select-phase DRQ disjunct is dead
// via sel_active==0 — NOT via its fifo==16 term, which this scenario
// measures directly (the task asked to verify that reading).  The
// completion had already run dma_set(DMA_NONE), so the data-in DRQ
// formula is dead too: DRQ can never rise, and the ROM's UNTIMED DAFB
// DRQ spin at 0x40898ea8 never exits.  Each retry round's CDB stuffs
// land in the plain FIFO and accrete to 16 — the measured fingerprint
// (status = INT | phase, FIFO = 16, DRQ low).  Every term of this
// absorbing state matches MAME's model; the DEFECT is whatever parked
// or desynced the transfer upstream, which is shape (i)'s root cause.
// (The hardware sample read status=0x81 — TC0 clear — where this weave
// leaves 0x91: TC0 is sticky until the next DMA-form dispatch, so any
// dispatched DMA command between the wreck and the sample clears it.
// Not load-bearing for the mechanism.)
// ─────────────────────────────────────────────────────────────────────
static bool scn_shape3_queue_park_drq_dead() {
    sd_mock.gap = 0;
    sd_mock.stall_every = 0;
    sd_mock.stall_len = 0;
    reset();
    fill_pattern();

    if (!rom_select_read6(0, 1)) return false;

    // One healthy 16-byte chunk, fully drained: completion fires I_BUS.
    reg_w(0x0, 16); reg_w(0x1, 0);
    reg_w(0x3, 0x90);
    for (int w = 0; w < 8; ++w) {
        uint16_t v = 0;
        CHECK_TRUE("shape3", "chunk drain beat", shim_r16(&v));
    }
    bool intr = false;
    for (int i = 0; i < 300000; ++i) {
        if (reg_r(0x4) & S_INTR) { intr = true; break; }
    }
    CHECK_TRUE("shape3", "chunk completion INT fires", intr);
    // The driver MISSES it: no istatus read.  Slot 0 stays occupied by
    // the completed 0x90; INT stays pending.

    // Retry rounds, ROM-faithful writes, three times over.
    uint8_t fifo_prev = reg_r(0x7) & 0x1f;
    for (int round = 0; round < 4; ++round) {
        reg_w(0x3, 0x01);           // FLUSH — queues in slot 1, must NOT run
        reg_w(0x4, TARGET_ID);
        reg_w(0x1, 0x00);
        reg_w(0x0, 0x01);
        reg_w(0x3, 0xC1);           // queue full — GROSS-dropped
        tick_n(200);
        CHECK_EQ("shape3", "select never dispatches (sel_active stays 0)",
                 (unsigned)U_SCSI(c96_sel_active), 0u);
        uint8_t ff = reg_r(0x7) & 0x1f;
        CHECK_TRUE("shape3", "queued FLUSH did not clear the FIFO",
                   ff >= fifo_prev);
        // The ROM's five reg-2 CDB stuffs: with sel_active==0 they are
        // plain FIFO pushes and accrete.
        reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
        reg_w(0x2, 0x03); reg_w(0x2, 0x01);
        uint8_t ff2 = reg_r(0x7) & 0x1f;
        CHECK_TRUE("shape3", "CDB stuffs accrete in the FIFO (cap 16)",
                   ff2 == (uint8_t)((ff + 5 > 16) ? 16 : ff + 5));
        fifo_prev = ff2;
        // ROM 0x40898ea8: the UNTIMED DAFB DRQ spin.  dma_dir is NONE
        // (completion ran dma_set(DMA_NONE)) and sel_active is 0, so
        // DRQ must never rise — the wedge.
        bool drq_seen = false;
        for (int i = 0; i < 8000; ++i) {
            if (dut->drq) { drq_seen = true; break; }
            tick();
        }
        if (drq_seen) {
            std::printf("  [%s] FAIL round %d: DRQ rose while the select "
                        "is undispatched\n", cur_scn.c_str(), round);
            note_fail("shape3");
            dump_state("unexpected DRQ");
            return false;
        }
        uint8_t s = reg_r(0x4);
        CHECK_TRUE("shape3", "INT stays pending across the round",
                   (s & S_INTR) != 0);
        CHECK_EQ("shape3", "phase reads DATA_IN across the round",
                 s & 0x07, 0x01);
    }
    CHECK_EQ("shape3", "FIFO accreted to full (the hardware fingerprint)",
             reg_r(0x7) & 0x1f, 16);
    std::printf("  [%s] absorbing state verified: sel_active=0, DRQ dead, "
                "fifo=16, status=0x%02x — shape (iii)'s terminal state is "
                "reachable only DOWNSTREAM of a missed/parked completion\n",
                cur_scn.c_str(), reg_r(0x4));
    return true;
}

// Control: the select-phase DRQ disjunct itself is healthy — a clean
// deferred DMA select raises DRQ for the CDB tail (this is what
// rom_select_read6 exercises), and a full transaction completes.
static bool scn_select_drq_control() {
    sd_mock.gap = 0; sd_mock.stall_every = 0; sd_mock.stall_len = 0;
    reset();
    fill_pattern();
    if (!rom_select_read6(0, 1)) return false;
    std::vector<uint8_t> data;
    bool parked = false; uint8_t ps = 0, pf = 0;
    if (!rom_word_chunk_drain(data, 512, 16, 0, &parked, &ps, &pf))
        return false;
    if (!status_epilogue()) return false;
    CHECK_EQ("control", "512 bytes", (uint32_t)data.size(), 512u);
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; ++i)
        if (!std::strcmp(argv[i], "--positive-control"))
            positive_control = true;

    dut = new Vtb_scsi_vhdd_sd;

    if (positive_control) {
        std::printf("=== 53C96 stall shapes — POSITIVE CONTROL pass "
                    "(failures below are the POINT) ===\n");

        // Control A — data comparison can fail.
        g_fail_kind.clear();
        RUN("pc_data_compare_can_fail",
            scn_shape1(0, 16, 0, 0, 0, /*corrupt=*/true, /*skip=*/0));
        bool data_fired = (g_fail_kind == "data");

        // Control B — the park detector can fire: under-drain chunk 0 by
        // two words and the INT wait must park (residue 4 > threshold).
        g_fail_kind.clear();
        RUN("pc_park_detector_can_fire",
            scn_shape1(0, 16, 0, 0, 0, /*corrupt=*/false, /*skip=*/2));
        bool park_fired = (g_fail_kind == "park");

        std::printf("\npositive control: data-compare fired=%d "
                    "park-detector fired=%d (%d/%d scenarios went red as "
                    "required)\n",
                    (int)data_fired, (int)park_fired, n_fail,
                    n_pass + n_fail);
        int rc = (data_fired && park_fired && n_fail == 2) ? 0 : 1;
        if (rc != 0)
            std::printf("POSITIVE CONTROL DID NOT FIRE — this tb "
                        "measures nothing.\n");
        else
            std::printf("POSITIVE CONTROL OK: both assertion classes can "
                        "fail.\n");
        delete dut;
        return rc;
    }

    std::printf("=== 53C96 stall shapes — ROM-faithful word-beat "
                "replays ===\n");

    // Shape (i) controls + repro.
    RUN("s1_word_chunks_steady_tc16",
        scn_shape1(0, 16, 0, 0, 0, false, 0));
    RUN("s1_word_chunks_steady_tc512",
        scn_shape1(1, 512, 0, 0, 0, false, 0));
    // Supply dips: every 96 bytes the provider goes silent for 300
    // cycles, so the blind word burst catches the FIFO at occupancy <=1
    // a handful of times per block — the degrade-leak trigger.
    RUN("s1_word_chunks_dips_tc16",
        scn_shape1(2, 16, 0, 96, 300, false, 0));
    RUN("s1_word_chunks_dips_tc512",
        scn_shape1(3, 512, 0, 96, 300, false, 0));
    // Slow-but-steady supply (SPI-like pacing): every beat waits, none
    // may degrade.
    RUN("s1_word_chunks_slow_tc512",
        scn_shape1(4, 512, 6, 0, 0, false, 0));

    // Multi-block at the ROM's real transfer size (6 blocks = 3072 B):
    // steady supply as a control, then dips, then a uniformly slow drip.
    RUN("s1_multiblock_6x512_steady",
        scn_shape1_multiblock(0x1304, 6, 512, 0, 0, 0));
    RUN("s1_multiblock_6x512_dips",
        scn_shape1_multiblock(0x1304, 6, 512, 2, 64, 40));
    RUN("s1_multiblock_6x512_slow",
        scn_shape1_multiblock(0x1304, 6, 512, 20, 0, 0));

    // Cross-command state: the hardware dies on the ~150th command.
    RUN("s1_many_commands_160_dips",
        scn_many_commands(160, 6, 512, 2, 64, 40));

    // Shape (ii): the abort path recovers a parked transfer, so the
    // retry cycle's exit exists whenever the retry itself does not park.
    RUN("s2_abort_recovers", scn_shape2_abort_recovers());

    // Shape (iii): queue-park absorbing state + DRQ-disjunct verification.
    RUN("s3_queue_park_drq_dead", scn_shape3_queue_park_drq_dead());
    RUN("s3_select_drq_control", scn_select_drq_control());

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
