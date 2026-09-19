// tb_pram_sd.cpp — unit testbench for manual PRAM <-> SD persistence.
//
// Build / run:  make tb-pram-sd            (default post-reset PRAM image)
//               make tb-pram-sd-populated  (same tests, +rtc_populated_pram)
//
// DUT: tb/tb_pram_sd_top.v = the real pram_sd.v + pram_cdc.v + rtc.v in
// their two real clock domains, plus the SD card modelled here at the SPI
// byte level.
//
// ══════════════════════════════════════════════════════════════════════
// WHAT THIS TESTBENCH IS BUILT TO AVOID
// ══════════════════════════════════════════════════════════════════════
// The failure mode this project keeps rediscovering is a test that passes
// while measuring nothing (FMOVEM.X "passed" for months while moving zero
// bytes).  Three deliberate structural choices here:
//
//  1. CROSS-PATH ASSERTIONS.  Every content check writes PRAM through one
//     path and reads it back through the OTHER.  Contents go in through
//     rtc.v's own bit-banged serial command protocol — the same wires the
//     68k uses — and come back out through pram_sd's snapshot, or vice
//     versa.  A pram_sd that operated on a private shadow copy would pass
//     a same-path test and fails these.
//
//  2. THE CARD IS CHECKED BYTE-FOR-BYTE, HOST-SIDE.  The expected 512-byte
//     sector (header, checksum and payload) is built independently in C++
//     and compared against what the card model actually received.  A save
//     that wrote the right length of wrong bytes fails.
//
//  3. THE DEFAULTS ARE CAPTURED, NOT HARDCODED.  Tests that assert "PRAM
//     was reset to its post-reset defaults" compare against an image
//     captured at runtime by zapping rtc.v and reading all 256 bytes back
//     over the serial protocol.  That is why the same binary is also run
//     with +rtc_populated_pram: under that plusarg rtc.v's defaults are a
//     completely different (non-zero) image, so a pram_sd that hardcoded
//     "fall back to zeros" passes the default run and fails the populated
//     one.  There is exactly one definition of the defaults and this
//     proves pram_sd uses it.
//
// ══════════════════════════════════════════════════════════════════════
// WHAT IT CANNOT SEE — stated rather than implied
// ══════════════════════════════════════════════════════════════════════
//  * The Makefile target builds with `--x-initial unique --x-assign unique`
//    (NOT the project-wide `fast`) precisely because pram_sd's 512-byte
//    staging buffer is deliberately un-reset.  Under `fast` an
//    uninitialised read returns a tidy 0 and a read-before-write bug is
//    structurally invisible; under `unique` it returns garbage that these
//    byte-exact comparisons catch.  If you ever change this target back to
//    `fast`, you have silently deleted that coverage.
//  * PRAM reset-survival IS observable here (scenario 11): rtc.v's array
//    has a real `initial` image and genuinely is not touched by `rst`, so
//    writing a pattern, pulsing rst and reading it back is a true test.
//    What is NOT observable in any Verilator run is bitstream-load
//    behaviour — whether the synthesised array powers up the way the
//    `initial` says.  That is a hardware property; do not read scenario 11
//    as covering it.
//  * The wrapper reproduces fpga_top_sd.vh's PRAM grant/reset ownership
//    interlock.  It does not instantiate the complete boot/SCSI/JTAG mux;
//    contention among those masters remains an integration-level property.

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <array>
#include <vector>
#include <verilated.h>
#include "Vtb_pram_sd_top.h"

static Vtb_pram_sd_top* top = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

// ── Register map (must match rtl/soc/pram_sd.v) ───────────────────────
static const uint32_t REG_IDENT   = 0x00;
static const uint32_t REG_CTRL    = 0x04;
static const uint32_t REG_STATUS  = 0x08;
static const uint32_t REG_BUFPTR  = 0x0C;
static const uint32_t REG_BUFDATA = 0x10;
static const uint32_t REG_LBA     = 0x14;
static const uint32_t REG_CKREF   = 0x18;

static const uint32_t IDENT_VALUE = 0x50524D31u;   // 'PRM1'
static const uint32_t CMD_SAVE    = 0x50520001u;
static const uint32_t CMD_LOAD    = 0x50520002u;
static const uint32_t CMD_SNAP    = 0x50520003u;

static const uint32_t PRAM_LBA    = 8191;

enum {
    RES_OK = 0, RES_BAD_MAGIC = 1, RES_BAD_VERSION = 2, RES_BAD_LENGTH = 3,
    RES_BAD_CKSUM = 4, RES_BLANK = 5, RES_SD_ERR = 6, RES_ARB_TIMEOUT = 7,
    RES_NOT_BOOTED = 8, RES_CDC_TIMEOUT = 9, RES_BAD_CMD = 10
};

static const char* res_name(uint32_t r) {
    switch (r) {
        case RES_OK: return "OK";
        case RES_BAD_MAGIC: return "BAD_MAGIC";
        case RES_BAD_VERSION: return "BAD_VERSION";
        case RES_BAD_LENGTH: return "BAD_LENGTH";
        case RES_BAD_CKSUM: return "BAD_CKSUM";
        case RES_BLANK: return "BLANK";
        case RES_SD_ERR: return "SD_ERR";
        case RES_ARB_TIMEOUT: return "ARB_TIMEOUT";
        case RES_NOT_BOOTED: return "NOT_BOOTED";
        case RES_CDC_TIMEOUT: return "CDC_TIMEOUT";
        case RES_BAD_CMD: return "BAD_CMD";
        default: return "?";
    }
}

#define ST_BUSY(s)      ((s) & 1u)
#define ST_DONE(s)      (((s) >> 1) & 1u)
#define ST_ERROR(s)     (((s) >> 2) & 1u)
#define ST_DEFAULTS(s)  (((s) >> 3) & 1u)
#define ST_RESULT(s)    (((s) >> 4) & 0xFu)
#define ST_SDERR(s)     (((s) >> 8) & 0xFu)
#define ST_CKCALC(s)    (((s) >> 16) & 0xFFFFu)

// ══════════════════════════════════════════════════════════════════════
// SD card model, at the SPI byte level
// ══════════════════════════════════════════════════════════════════════
struct SdCard {
    enum State {
        S_CMD, S_R1,
        S_RD_GAP, S_RD_DATA, S_RD_CRC,
        S_WR_GAP, S_WR_TOKEN, S_WR_DATA, S_WR_CRC, S_WR_RESP, S_WR_BUSY,
        S_QUIET
    };

    State   state = S_CMD;
    std::vector<uint8_t> frame;
    uint8_t sector[512];            // persistent card contents
    uint8_t written[512];           // what the last CMD24 actually delivered
    uint32_t lba = 0;
    uint32_t last_cmd = 0;
    int     idx = 0;
    int     gap = 0;
    int     crcn = 0;
    int     busy = 0;
    int     r1_pad = 0;

    // Test knobs / observations.
    uint8_t force_r1 = 0x00;        // non-zero => card rejects the command
    int     cmd_count = 0;          // total commands seen since reset
    int     cmd17_count = 0;
    int     cmd24_count = 0;
    bool    protocol_error = false;
    bool    write_complete = false;

    SdCard() { memset(sector, 0, sizeof sector); memset(written, 0, sizeof written); }

    void deselect() { state = S_CMD; frame.clear(); }

    uint8_t accept(uint8_t tx) {
        switch (state) {
            case S_CMD:
                if (frame.empty() && tx == 0xFF) return 0xFF;   // idle padding
                frame.push_back(tx);
                if (frame.size() == 6) {
                    last_cmd = frame[0] & 0x3Fu;
                    lba = ((uint32_t)frame[1] << 24) | ((uint32_t)frame[2] << 16) |
                          ((uint32_t)frame[3] << 8)  | (uint32_t)frame[4];
                    frame.clear();
                    cmd_count++;
                    if (last_cmd == 17) cmd17_count++;
                    else if (last_cmd == 24) cmd24_count++;
                    else protocol_error = true;
                    r1_pad = 1;                                  // exercise R1 polling
                    state = S_R1;
                }
                return 0xFF;

            case S_R1:
                if (r1_pad > 0) { r1_pad--; return 0xFF; }
                if (force_r1) { state = S_QUIET; return force_r1; }
                if (last_cmd == 17) { state = S_RD_GAP; gap = 2; }
                else                { state = S_WR_GAP; }
                return 0x00;

            case S_RD_GAP:
                if (gap > 0) { gap--; return 0xFF; }
                state = S_RD_DATA; idx = 0;
                return 0xFE;                                     // data token
            case S_RD_DATA: {
                uint8_t b = sector[idx++];
                if (idx == 512) { state = S_RD_CRC; crcn = 0; }
                return b;
            }
            case S_RD_CRC:
                if (++crcn == 2) state = S_QUIET;
                return 0xFF;                                     // CRC ignored

            case S_WR_GAP:
                if (tx != 0xFF) protocol_error = true;
                state = S_WR_TOKEN;
                return 0xFF;
            case S_WR_TOKEN:
                if (tx != 0xFE) { protocol_error = true; return 0xFF; }
                state = S_WR_DATA; idx = 0;
                return 0xFF;
            case S_WR_DATA:
                written[idx] = tx;
                sector[idx] = tx;                                // card persists it
                if (++idx == 512) { state = S_WR_CRC; crcn = 0; }
                return 0xFF;
            case S_WR_CRC:
                if (++crcn == 2) state = S_WR_RESP;
                return 0xFF;
            case S_WR_RESP:
                state = S_WR_BUSY; busy = 4;
                return 0xE5;                                     // data accepted
            case S_WR_BUSY:
                if (busy > 0) { busy--; return 0x00; }
                write_complete = true;
                state = S_QUIET;
                return 0xFF;

            case S_QUIET:
            default:
                return 0xFF;
        }
    }
} card;

// ══════════════════════════════════════════════════════════════════════
// Clocking.  core_clk every tick; pb_clk toggles every tick => half rate.
// ══════════════════════════════════════════════════════════════════════
static bool     rsp_pending = false;
static uint8_t  rsp_byte    = 0xFF;
static uint8_t  pb_level    = 0;
static bool     pb_frozen   = false;
static bool     prev_cs_n   = true;
// Grant policy: models fpga_top_sd.vh's interlock (grant only rises when
// the SCSI engine is idle, and is held until the request drops).
static bool     grant_enabled = true;
static int      grant_delay   = 4;
static int      grant_countdown = 0;

static void tick() {
    top->core_clk = 0;
    top->pb_clk   = pb_level;
    top->spi_cmd_ready = 1;
    top->spi_rsp_valid = rsp_pending ? 1 : 0;
    top->spi_rsp_data  = rsp_byte;
    top->eval();

    bool    cmd_fire = top->spi_cmd_valid && top->spi_cmd_ready;
    uint8_t cmd_byte = (uint8_t)top->spi_cmd_data;
    bool    cs_n     = top->spi_cs_n != 0;

    top->core_clk = 1;
    top->eval();

    if (cs_n && !prev_cs_n) card.deselect();     // deselect resets the card FSM
    prev_cs_n = cs_n;

    if (rsp_pending) rsp_pending = false;
    if (cmd_fire) { rsp_byte = card.accept(cmd_byte); rsp_pending = true; }

    // Grant handshake, evaluated after the edge.
    if (!top->sd_req) {
        top->sd_arb_allow = 0;
        grant_countdown = grant_delay;
    } else if (grant_enabled && !top->sd_gnt) {
        if (grant_countdown > 0) grant_countdown--;
        else top->sd_arb_allow = 1;
    }

    if (!pb_frozen) pb_level ^= 1;
    sim_time++;
}

static void pb_tick() { tick(); tick(); }        // one full pb_clk period

// ══════════════════════════════════════════════════════════════════════
// AXI-Lite host
// ══════════════════════════════════════════════════════════════════════
static void axil_write(uint32_t addr, uint32_t data, uint8_t strb = 0xF) {
    top->s_awaddr  = addr;
    top->s_wdata   = data;
    top->s_wstrb   = strb;
    top->s_awvalid = 1;
    top->s_wvalid  = 1;
    top->s_bready  = 0;
    int guard = 0;
    while (!(top->s_awready && top->s_wready)) {
        tick();
        if (++guard > 5000) { printf("  AXI write ready timeout @0x%02x\n", addr); break; }
    }
    tick();
    top->s_awvalid = 0;
    top->s_wvalid  = 0;
    guard = 0;
    while (!top->s_bvalid) {
        tick();
        if (++guard > 5000) { printf("  AXI write resp timeout @0x%02x\n", addr); break; }
    }
    top->s_bready = 1;
    tick();
    top->s_bready = 0;
}

static uint32_t axil_read(uint32_t addr) {
    top->s_araddr  = addr;
    top->s_arvalid = 1;
    top->s_rready  = 0;
    int guard = 0;
    while (!top->s_arready) {
        tick();
        if (++guard > 5000) { printf("  AXI read AR timeout @0x%02x\n", addr); return 0xDEADBEEFu; }
    }
    tick();
    top->s_arvalid = 0;
    guard = 0;
    while (!top->s_rvalid) {
        tick();
        if (++guard > 5000) { printf("  AXI read R timeout @0x%02x\n", addr); return 0xDEADBEEFu; }
    }
    uint32_t d = top->s_rdata;
    top->s_rready = 1;
    tick();
    top->s_rready = 0;
    return d;
}

// ══════════════════════════════════════════════════════════════════════
// rtc.v serial command protocol — the INDEPENDENT witness path.
// Byte-for-byte the sequence tb_rtc.cpp uses (which is MAME's macrtc).
// ══════════════════════════════════════════════════════════════════════
static void shift_bit_in(int b) {
    top->rtc_data_o  = b & 1;
    top->rtc_data_oe = 1;
    top->rtc_clk = 1; pb_tick();
    top->rtc_clk = 0; pb_tick();
}

static int shift_bit_out() {
    top->rtc_data_oe = 0;
    top->rtc_clk = 1; pb_tick();
    top->rtc_clk = 0; pb_tick();
    return top->rtc_data_i & 1;
}

static uint8_t rtc_xpram_txn(uint8_t cmd, uint8_t addr_byte, uint8_t data_byte) {
    uint8_t out = 0;
    top->rtc_enb = 0; pb_tick();
    for (int i = 7; i >= 0; i--) shift_bit_in((cmd >> i) & 1);
    for (int i = 7; i >= 0; i--) shift_bit_in((addr_byte >> i) & 1);
    if (cmd & 0x80) {
        for (int i = 7; i >= 0; i--) out = (uint8_t)((out << 1) | shift_bit_out());
    } else {
        for (int i = 7; i >= 0; i--) shift_bit_in((data_byte >> i) & 1);
    }
    top->rtc_data_oe = 0;
    top->rtc_clk = 0; pb_tick();
    top->rtc_enb = 1; pb_tick();
    return out;
}

static uint8_t xpram_cmd_for_addr(uint8_t a, bool read) {
    return (uint8_t)((read ? 0x80 : 0x00) | 0x38 | ((a >> 5) & 0x07));
}
static uint8_t xpram_addr_byte(uint8_t a) { return (uint8_t)((a & 0x1F) << 2); }

static uint8_t xpram_read(uint8_t a) {
    return rtc_xpram_txn(xpram_cmd_for_addr(a, true), xpram_addr_byte(a), 0);
}
static void xpram_write(uint8_t a, uint8_t v) {
    (void)rtc_xpram_txn(xpram_cmd_for_addr(a, false), xpram_addr_byte(a), v);
}

static void xpram_read_all(std::array<uint8_t, 256>& out) {
    for (int i = 0; i < 256; i++) out[i] = xpram_read((uint8_t)i);
}
static void xpram_write_all(const std::array<uint8_t, 256>& in) {
    for (int i = 0; i < 256; i++) xpram_write((uint8_t)i, in[i]);
}

// ══════════════════════════════════════════════════════════════════════
// Host-side model of the on-card format (independent of the RTL)
// ══════════════════════════════════════════════════════════════════════
static uint16_t fletcher16(const uint8_t* buf) {
    uint8_t a = 0x0A, b = 0x5D;
    for (int i = 0; i < 8; i++)   { a = (uint8_t)(a + buf[i]); b = (uint8_t)(b + a); }
    for (int i = 16; i < 272; i++){ a = (uint8_t)(a + buf[i]); b = (uint8_t)(b + a); }
    return (uint16_t)((b << 8) | a);
}

static void build_expected_sector(const std::array<uint8_t, 256>& pram, uint8_t* out) {
    memset(out, 0, 512);
    out[0] = 'P'; out[1] = 'R'; out[2] = 'M'; out[3] = '1';
    out[4] = 0x00; out[5] = 0x01;               // version 1
    out[6] = 0x01; out[7] = 0x00;               // length 256
    for (int i = 0; i < 256; i++) out[16 + i] = pram[i];
    uint16_t ck = fletcher16(out);
    out[8] = (uint8_t)(ck >> 8);
    out[9] = (uint8_t)(ck & 0xFF);
}

// ══════════════════════════════════════════════════════════════════════
// Reset / helpers
// ══════════════════════════════════════════════════════════════════════
static void idle_inputs() {
    top->boot_done   = 1;
    top->s_awaddr = 0; top->s_awvalid = 0;
    top->s_wdata = 0; top->s_wstrb = 0xF; top->s_wvalid = 0; top->s_bready = 0;
    top->s_araddr = 0; top->s_arvalid = 0; top->s_rready = 0;
    top->spi_cmd_ready = 1; top->spi_rsp_valid = 0; top->spi_rsp_data = 0xFF;
    top->sd_arb_allow = 0;
    top->rtc_enb = 1; top->rtc_clk = 0; top->rtc_data_o = 0; top->rtc_data_oe = 0;
    top->phi2_tick = 0;
    top->rtc_pram_clear = 0;
}

// Full reset of the logic AND an explicit PRAM zap, so each scenario
// starts from rtc.v's post-reset image.
static void reset_all() {
    idle_inputs();
    card = SdCard();
    rsp_pending = false; rsp_byte = 0xFF; prev_cs_n = true;
    pb_frozen = false; grant_enabled = true; grant_countdown = grant_delay;
    top->core_rst = 1;
    top->pb_rst   = 1;
    top->rtc_pram_clear = 1;
    for (int i = 0; i < 40; i++) tick();
    top->core_rst = 0;
    top->pb_rst   = 0;
    for (int i = 0; i < 40; i++) tick();
    top->rtc_pram_clear = 0;
    for (int i = 0; i < 20; i++) tick();
}

// Pulse the operator zap (Cmd-Opt-P-R) without resetting anything else.
static void pram_zap() {
    top->rtc_pram_clear = 1;
    for (int i = 0; i < 40; i++) tick();
    top->rtc_pram_clear = 0;
    for (int i = 0; i < 20; i++) tick();
}

static uint32_t run_cmd(uint32_t cmd, uint64_t budget = 4000000) {
    axil_write(REG_CTRL, cmd);
    uint64_t start = sim_time;
    uint32_t st = 0;
    while ((sim_time - start) < budget) {
        st = axil_read(REG_STATUS);
        if (!ST_BUSY(st) && ST_DONE(st)) return st;
        for (int i = 0; i < 16; i++) tick();
    }
    printf("  !! run_cmd(0x%08x) NEVER COMPLETED (status=0x%08x) — unbounded\n",
           cmd, st);
    return 0xFFFFFFFFu;
}

static void read_stage_buffer(uint8_t* out512) {
    axil_write(REG_BUFPTR, 0);
    for (int i = 0; i < 512; i++) out512[i] = (uint8_t)(axil_read(REG_BUFDATA) & 0xFF);
}

#define CHECK_TRUE(label, cond) do { \
    if (!(cond)) { printf("  FAIL %s\n", (label)); return false; } \
} while (0)

#define CHECK_EQ(label, got, exp) do { \
    uint32_t g_ = (uint32_t)(got), e_ = (uint32_t)(exp); \
    if (g_ != e_) { printf("  FAIL %s: got 0x%x expected 0x%x\n", (label), g_, e_); return false; } \
} while (0)

static bool cmp_pram(const char* label,
                     const std::array<uint8_t, 256>& got,
                     const std::array<uint8_t, 256>& exp) {
    for (int i = 0; i < 256; i++) {
        if (got[i] != exp[i]) {
            printf("  FAIL %s: PRAM[0x%02x] got 0x%02x expected 0x%02x\n",
                   label, i, got[i], exp[i]);
            return false;
        }
    }
    return true;
}

// A pattern that is distinct at every index and shares no byte value with
// a plausible default image at the same index.
static std::array<uint8_t, 256> make_pattern(uint8_t salt) {
    std::array<uint8_t, 256> p{};
    for (int i = 0; i < 256; i++) p[i] = (uint8_t)((i * 7 + 0x5A + salt) ^ 0xA5);
    return p;
}

// The post-reset default image, captured from rtc.v at runtime rather
// than hardcoded here.  See the header note about +rtc_populated_pram.
static std::array<uint8_t, 256> capture_defaults() {
    pram_zap();
    std::array<uint8_t, 256> d{};
    xpram_read_all(d);
    return d;
}

// ══════════════════════════════════════════════════════════════════════
// Scenarios
// ══════════════════════════════════════════════════════════════════════

// 1 — the register file is actually there, and points at the LBA we think.
static bool test_ident_and_lba() {
    reset_all();
    CHECK_EQ("IDENT", axil_read(REG_IDENT), IDENT_VALUE);
    CHECK_EQ("PRAM LBA", axil_read(REG_LBA), PRAM_LBA);
    return true;
}

// 2 — SNAP stages the LIVE PRAM.  Written over the serial protocol, read
// back through pram_sd: cross-path.
static bool test_snap_reads_live_pram() {
    reset_all();
    auto pat = make_pattern(0);
    xpram_write_all(pat);

    uint32_t st = run_cmd(CMD_SNAP);
    CHECK_TRUE("snap completed", st != 0xFFFFFFFFu);
    CHECK_EQ("snap result", ST_RESULT(st), RES_OK);
    CHECK_EQ("snap touched no card command", card.cmd_count, 0);

    uint8_t got[512];
    read_stage_buffer(got);

    uint8_t exp[512];
    build_expected_sector(pat, exp);
    for (int i = 0; i < 512; i++) {
        if (got[i] != exp[i]) {
            printf("  FAIL staged sector[0x%03x]: got 0x%02x expected 0x%02x\n",
                   i, got[i], exp[i]);
            return false;
        }
    }
    CHECK_EQ("STATUS checksum matches host Fletcher-16",
             ST_CKCALC(st), fletcher16(exp));
    return true;
}

// 3 — SAVE puts exactly that sector on the card, at LBA 8191, via CMD24.
static bool test_save_writes_sector() {
    reset_all();
    auto pat = make_pattern(1);
    xpram_write_all(pat);

    uint32_t st = run_cmd(CMD_SAVE);
    CHECK_TRUE("save completed", st != 0xFFFFFFFFu);
    CHECK_EQ("save result", ST_RESULT(st), RES_OK);
    CHECK_TRUE("no card protocol error", !card.protocol_error);
    CHECK_TRUE("card write completed", card.write_complete);
    CHECK_EQ("exactly one CMD24", card.cmd24_count, 1);
    CHECK_EQ("no CMD17", card.cmd17_count, 0);
    CHECK_EQ("target LBA", card.lba, PRAM_LBA);

    uint8_t exp[512];
    build_expected_sector(pat, exp);
    for (int i = 0; i < 512; i++) {
        if (card.written[i] != exp[i]) {
            printf("  FAIL card sector[0x%03x]: got 0x%02x expected 0x%02x\n",
                   i, card.written[i], exp[i]);
            return false;
        }
    }
    return true;
}

// 4 — the headline round trip: save -> zap -> load restores byte-exactly.
//     Source and verification both go over the serial protocol.
static bool test_round_trip_save_clear_load() {
    reset_all();
    auto defaults = capture_defaults();
    auto pat = make_pattern(2);
    xpram_write_all(pat);

    // Prove the write landed before we trust the save.
    std::array<uint8_t, 256> before{};
    xpram_read_all(before);
    if (!cmp_pram("pattern actually written", before, pat)) return false;

    uint32_t st = run_cmd(CMD_SAVE);
    CHECK_EQ("save result", ST_RESULT(st), RES_OK);

    pram_zap();
    std::array<uint8_t, 256> zapped{};
    xpram_read_all(zapped);
    if (!cmp_pram("zap really reset PRAM", zapped, defaults)) return false;
    // Guard against a vacuous test: the pattern must differ from defaults,
    // or "restored" would be indistinguishable from "never changed".
    CHECK_TRUE("pattern differs from defaults", pat != defaults);

    st = run_cmd(CMD_LOAD);
    CHECK_TRUE("load completed", st != 0xFFFFFFFFu);
    CHECK_EQ("load result", ST_RESULT(st), RES_OK);
    CHECK_EQ("no error bit", ST_ERROR(st), 0u);
    CHECK_EQ("defaults NOT applied", ST_DEFAULTS(st), 0u);
    CHECK_EQ("exactly one CMD17", card.cmd17_count, 1);

    std::array<uint8_t, 256> after{};
    xpram_read_all(after);
    if (!cmp_pram("restored PRAM byte-exact", after, pat)) return false;
    return true;
}

// 5 — a corrupted payload byte must be REJECTED and the array put back on
//     rtc.v's own post-reset image (not left holding the garbage).
static bool test_bad_checksum_falls_back_to_defaults() {
    reset_all();
    auto defaults = capture_defaults();
    auto pat = make_pattern(3);
    xpram_write_all(pat);
    CHECK_EQ("save result", ST_RESULT(run_cmd(CMD_SAVE)), RES_OK);

    card.sector[16 + 100] ^= 0xFF;          // flip one payload byte on the card

    uint32_t st = run_cmd(CMD_LOAD);
    CHECK_TRUE("load completed", st != 0xFFFFFFFFu);
    CHECK_EQ("result is BAD_CKSUM", ST_RESULT(st), RES_BAD_CKSUM);
    CHECK_EQ("error bit set", ST_ERROR(st), 1u);
    CHECK_EQ("defaults applied bit set", ST_DEFAULTS(st), 1u);
    CHECK_TRUE("computed and stored checksums really differ",
               ST_CKCALC(st) != (axil_read(REG_CKREF) & 0xFFFFu));

    std::array<uint8_t, 256> after{};
    xpram_read_all(after);
    if (!cmp_pram("PRAM reset to post-reset defaults", after, defaults)) return false;
    return true;
}

// 6 — a blank (all-zero) sector, i.e. a fresh card, is rejected too.
static bool test_blank_sector_falls_back_to_defaults() {
    reset_all();
    auto defaults = capture_defaults();
    auto pat = make_pattern(4);
    xpram_write_all(pat);
    memset(card.sector, 0, sizeof card.sector);   // fresh card

    uint32_t st = run_cmd(CMD_LOAD);
    CHECK_TRUE("load completed", st != 0xFFFFFFFFu);
    CHECK_EQ("result is BLANK", ST_RESULT(st), RES_BLANK);
    CHECK_EQ("defaults applied bit set", ST_DEFAULTS(st), 1u);

    std::array<uint8_t, 256> after{};
    xpram_read_all(after);
    if (!cmp_pram("PRAM reset to post-reset defaults", after, defaults)) return false;
    // And specifically NOT the pattern that was live before the load.
    CHECK_TRUE("PRAM is not the pre-load contents", after != pat);
    return true;
}

// 7 — stale HDD-ish data with a plausible-looking but wrong magic.
static bool test_bad_magic_falls_back_to_defaults() {
    reset_all();
    auto defaults = capture_defaults();
    auto pat = make_pattern(5);
    xpram_write_all(pat);
    CHECK_EQ("save result", ST_RESULT(run_cmd(CMD_SAVE)), RES_OK);

    card.sector[0] = 'Q';                   // 'QRM1' — not our magic

    uint32_t st = run_cmd(CMD_LOAD);
    CHECK_EQ("result is BAD_MAGIC", ST_RESULT(st), RES_BAD_MAGIC);
    CHECK_EQ("defaults applied bit set", ST_DEFAULTS(st), 1u);

    std::array<uint8_t, 256> after{};
    xpram_read_all(after);
    return cmp_pram("PRAM reset to post-reset defaults", after, defaults);
}

// 8 — an SD-level failure (card rejects the command) is also bounded and
//     also lands on the defaults.
static bool test_sd_error_falls_back_to_defaults() {
    reset_all();
    auto defaults = capture_defaults();
    auto pat = make_pattern(6);
    xpram_write_all(pat);

    card.force_r1 = 0x04;                   // ILLEGAL_COMMAND

    uint32_t st = run_cmd(CMD_LOAD);
    CHECK_TRUE("load completed (bounded)", st != 0xFFFFFFFFu);
    CHECK_EQ("result is SD_ERR", ST_RESULT(st), RES_SD_ERR);
    CHECK_TRUE("sd err_cause is non-zero", ST_SDERR(st) != 0);
    CHECK_EQ("defaults applied bit set", ST_DEFAULTS(st), 1u);

    std::array<uint8_t, 256> after{};
    xpram_read_all(after);
    return cmp_pram("PRAM reset to post-reset defaults", after, defaults);
}

// 9 — bounded response: the SPI bus is never granted.
static bool test_arb_timeout_is_bounded() {
    reset_all();
    grant_enabled = false;                  // the grant never arrives

    uint32_t st = run_cmd(CMD_LOAD, 200000);
    CHECK_TRUE("load terminated despite no grant", st != 0xFFFFFFFFu);
    CHECK_EQ("result is ARB_TIMEOUT", ST_RESULT(st), RES_ARB_TIMEOUT);
    CHECK_EQ("busy dropped", ST_BUSY(st), 0u);
    CHECK_EQ("request released", top->sd_req, 0);
    CHECK_EQ("no card command was issued", card.cmd_count, 0);
    return true;
}

// 10 — bounded response: the destination clock domain is dead.
static bool test_cdc_timeout_is_bounded() {
    reset_all();
    pb_frozen = true;                       // pb_clk stops; rtc cannot answer

    uint32_t st = run_cmd(CMD_SNAP, 200000);
    CHECK_TRUE("snap terminated despite a dead pb_clk", st != 0xFFFFFFFFu);
    CHECK_EQ("result is CDC_TIMEOUT", ST_RESULT(st), RES_CDC_TIMEOUT);
    CHECK_EQ("busy dropped", ST_BUSY(st), 0u);
    pb_frozen = false;
    return true;
}

// A full SoC reset must not revoke the PRAM master's grant after CMD24 has
// opened the card's receive window.  Hold reset asserted through completion:
// the operation must retain ownership, finish byte-exactly, and only then
// permit the wrapper to reset the PRAM FSM.
static bool test_reset_during_active_save_retains_sd_ownership() {
    reset_all();
    auto pat = make_pattern(0x31);
    xpram_write_all(pat);

    axil_write(REG_CTRL, CMD_SAVE);
    uint64_t guard = 4000000;
    while (!(card.state == SdCard::S_WR_DATA && card.idx >= 137) &&
           guard-- > 0)
        tick();
    CHECK_TRUE("save reached a live partial CMD24 block", guard > 0);
    CHECK_TRUE("PRAM operation owns the SD bus", top->pram_sd_busy &&
                                                      top->sd_req &&
                                                      top->sd_gnt);

    top->core_rst = 1;
    top->pb_rst = 1;
    for (int i = 0; i < 32; i++) {
        tick();
        CHECK_TRUE("reset did not revoke an active PRAM grant",
                   !top->sd_req || top->sd_gnt);
    }
    CHECK_TRUE("reset did not erase the active PRAM FSM",
               top->pram_sd_busy);

    guard = 4000000;
    while (top->pram_sd_busy && guard-- > 0) {
        tick();
        CHECK_TRUE("grant held for the complete ownership request",
                   !top->sd_req || top->sd_gnt);
    }
    CHECK_TRUE("active save completed while reset was pending", guard > 0);
    CHECK_TRUE("card completed CMD24 before ownership released",
               card.write_complete && !card.protocol_error);
    CHECK_EQ("reset-time save targeted only PRAM LBA", card.lba, PRAM_LBA);

    uint8_t exp[512];
    build_expected_sector(pat, exp);
    for (int i = 0; i < 512; i++) {
        if (card.written[i] != exp[i]) {
            printf("  FAIL reset-time sector[0x%03x]: got 0x%02x expected 0x%02x\n",
                   i, card.written[i], exp[i]);
            return false;
        }
    }

    // Leave reset asserted long enough to prove the now-idle owner and grant
    // do reset, then release it for the next independent scenario.
    for (int i = 0; i < 8; i++) tick();
    CHECK_EQ("grant released after the card was closed", top->sd_gnt, 0);
    top->core_rst = 0;
    top->pb_rst = 0;
    for (int i = 0; i < 40; i++) tick();
    return true;
}

// 11 — the property the whole feature rests on: PRAM survives a warm
//      reset.  (Observable here; see the header for what is NOT.)
static bool test_pram_survives_warm_reset() {
    reset_all();
    auto pat = make_pattern(7);
    xpram_write_all(pat);

    top->pb_rst = 1;
    top->core_rst = 1;
    for (int i = 0; i < 40; i++) tick();
    top->pb_rst = 0;
    top->core_rst = 0;
    for (int i = 0; i < 40; i++) tick();

    std::array<uint8_t, 256> after{};
    xpram_read_all(after);
    return cmp_pram("PRAM survived warm reset", after, pat);
}

// 12 — the manual-only contract.  Mac-side PRAM traffic, a reset, and a
//      long idle stretch must never produce a single SD command.
static bool test_no_automatic_sd_access() {
    reset_all();
    // Prove the card model CAN see commands, so "0 commands" is meaningful.
    CHECK_EQ("control: SNAP is card-free", ST_RESULT(run_cmd(CMD_SNAP)), RES_OK);
    CHECK_EQ("save reaches the card", ST_RESULT(run_cmd(CMD_SAVE)), RES_OK);
    CHECK_EQ("control: card saw the save", card.cmd24_count, 1);

    int baseline = card.cmd_count;

    // Now behave like a running Mac: lots of PRAM writes, a warm reset,
    // and idle time.  None of it may touch the card.
    for (int i = 0; i < 24; i++) xpram_write((uint8_t)(i * 9), (uint8_t)(0xC0 + i));
    top->pb_rst = 1; top->core_rst = 1;
    for (int i = 0; i < 40; i++) tick();
    top->pb_rst = 0; top->core_rst = 0;
    for (int i = 0; i < 60000; i++) tick();
    for (int i = 0; i < 24; i++) (void)xpram_read((uint8_t)(i * 9));

    CHECK_EQ("no SD command without an explicit CTRL write",
             card.cmd_count, baseline);
    return true;
}

// 13 — an unrecognised CTRL word is refused loudly, not silently ignored.
static bool test_bad_command_is_reported() {
    reset_all();
    axil_write(REG_CTRL, 0x50520099u);
    uint32_t st = axil_read(REG_STATUS);
    CHECK_EQ("busy not set", ST_BUSY(st), 0u);
    CHECK_EQ("done set", ST_DONE(st), 1u);
    CHECK_EQ("result is BAD_CMD", ST_RESULT(st), RES_BAD_CMD);
    CHECK_EQ("no card command", card.cmd_count, 0);
    return true;
}

// ══════════════════════════════════════════════════════════════════════
static void run_test(const char* name, bool (*fn)()) {
    printf("Running %s...\n", name);
    if (fn()) { printf("  PASS %s\n", name); n_pass++; }
    else      { n_fail++; }
}

#ifdef AUTOLOAD_BUILD
// Boot-time autoload (AUTOLOAD_ON_BOOT=1).  PRAM is only "battery-backed" to
// the Mac if the saved sector is installed BEFORE the 68k can read it, and
// boot_rom_loaded -- the signal that releases the CPU -- is the very signal
// that tells this module the card is usable.  So fpga_top holds the CPU on
// autoload_pending, and everything here is about that handshake being honest:
// it must go high on reset, must fall exactly once the load resolves, and must
// never hang (or the board would not boot at all).
static bool test_autoload_installs_before_cpu_release() {
    // Bring the module up with the card NOT yet ready.
    idle_inputs();
    card = SdCard();
    rsp_pending = false; rsp_byte = 0xFF; prev_cs_n = true;
    pb_frozen = false; grant_enabled = true; grant_countdown = grant_delay;
    top->boot_done = 0;
    top->core_rst = 1; top->pb_rst = 1; top->rtc_pram_clear = 1;
    for (int i = 0; i < 40; i++) tick();
    top->core_rst = 0; top->pb_rst = 0;
    for (int i = 0; i < 40; i++) tick();
    top->rtc_pram_clear = 0;
    for (int i = 0; i < 20; i++) tick();

    CHECK_TRUE("CPU is held from reset", top->autoload_pending == 1);
    CHECK_EQ("nothing touches the card before boot_done", card.cmd_count, 0);

    // Stage a valid saved sector, then declare the card ready.
    auto defaults = capture_defaults();
    auto pat = make_pattern(5);
    CHECK_TRUE("pattern differs from defaults", pat != defaults);
    build_expected_sector(pat, card.sector);

    top->boot_done = 1;
    for (int i = 0; i < 400000 && top->autoload_pending; i++) tick();
    CHECK_TRUE("hold is released", top->autoload_pending == 0);
    CHECK_EQ("autoload issued exactly one read", card.cmd17_count, 1);
    CHECK_EQ("autoload wrote nothing", card.cmd24_count, 0);

    // The saved image must actually be live by the time the hold drops.
    std::array<uint8_t, 256> live{};
    xpram_read_all(live);
    if (!cmp_pram("saved PRAM installed before release", live, pat)) return false;

    // One shot only: no retry loop, no second transfer.
    for (int i = 0; i < 200000; i++) tick();
    CHECK_EQ("autoload does not repeat", card.cmd17_count, 1);
    CHECK_TRUE("hold stays released", top->autoload_pending == 0);
    return true;
}

// A card that cannot supply a good sector must NOT wedge the boot: the module
// falls back to rtc.v's defaults loudly and drops the hold anyway.  Without
// this the feature could brick a board whose PRAM sector was never written.
static bool test_autoload_failure_still_releases_cpu() {
    idle_inputs();
    card = SdCard();
    rsp_pending = false; rsp_byte = 0xFF; prev_cs_n = true;
    pb_frozen = false; grant_enabled = true; grant_countdown = grant_delay;
    top->boot_done = 0;
    top->core_rst = 1; top->pb_rst = 1; top->rtc_pram_clear = 1;
    for (int i = 0; i < 40; i++) tick();
    top->core_rst = 0; top->pb_rst = 0;
    for (int i = 0; i < 40; i++) tick();
    top->rtc_pram_clear = 0;
    for (int i = 0; i < 20; i++) tick();
    CHECK_TRUE("CPU is held from reset", top->autoload_pending == 1);

    memset(card.sector, 0, sizeof card.sector);   // blank card
    top->boot_done = 1;
    for (int i = 0; i < 400000 && top->autoload_pending; i++) tick();
    CHECK_TRUE("blank sector still releases the CPU", top->autoload_pending == 0);
    return true;
}
#endif

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vtb_pram_sd_top;

    bool populated = false;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "+rtc_populated_pram")) populated = true;
    printf("tb_pram_sd: rtc.v default image = %s\n",
           populated ? "POPULATED (+rtc_populated_pram)" : "post-reset default");

#ifndef AUTOLOAD_BUILD
    // The manual-only scenarios below assume nothing ever starts a transfer on
    // its own, and reset_all() leaves boot_done high -- which in an autoload
    // build legitimately fires the one-shot and perturbs their card counters.
    // That build runs its own scenarios instead.
    run_test("ident_and_lba",                     test_ident_and_lba);
    run_test("snap_reads_live_pram",              test_snap_reads_live_pram);
    run_test("save_writes_sector",                test_save_writes_sector);
    run_test("round_trip_save_clear_load",        test_round_trip_save_clear_load);
    run_test("bad_checksum_falls_back_to_defaults", test_bad_checksum_falls_back_to_defaults);
    run_test("blank_sector_falls_back_to_defaults", test_blank_sector_falls_back_to_defaults);
    run_test("bad_magic_falls_back_to_defaults",  test_bad_magic_falls_back_to_defaults);
    run_test("sd_error_falls_back_to_defaults",   test_sd_error_falls_back_to_defaults);
    run_test("arb_timeout_is_bounded",            test_arb_timeout_is_bounded);
    run_test("cdc_timeout_is_bounded",            test_cdc_timeout_is_bounded);
    run_test("reset_during_active_save_retains_sd_ownership",
             test_reset_during_active_save_retains_sd_ownership);
    run_test("pram_survives_warm_reset",          test_pram_survives_warm_reset);
    run_test("no_automatic_sd_access",            test_no_automatic_sd_access);
    run_test("bad_command_is_reported",           test_bad_command_is_reported);
#endif
#ifdef AUTOLOAD_BUILD
    run_test("autoload_installs_before_cpu_release",
             test_autoload_installs_before_cpu_release);
    run_test("autoload_failure_still_releases_cpu",
             test_autoload_failure_still_releases_cpu);
#endif

    printf("pram_sd: %d passed, %d failed\n", n_pass, n_fail);
    (void)res_name(0);
    top->final();
    delete top;
    return n_fail ? 1 : 0;
}
