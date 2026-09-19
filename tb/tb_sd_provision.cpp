// tb_sd_provision.cpp — Verilator testbench for sd_provision_core
// (rtl/soc/sd_provision_core.v via the tb_sd_provision_top.v wrapper).
//
// Validates the fast bulk SD-write path of the dedicated provisioning
// bitstream end to end:
//   * boot_fsm card init (CMD0/8/55+41/58/6 + 1 dummy sector) completes
//     against a bit-level SPI SD-card model (adapted from tb_sd_boot.cpp)
//     and card_ready asserts;
//   * 32-bit AXI4 INCR write BURSTS (256 beats = 1 KiB per transaction,
//     standing in for the burst-capable JTAG-to-AXI master) land in the
//     staging BRAM, readable back through the same window;
//   * a bulk CMD25 op writes dozens of sectors from staging to the card,
//     byte-exact at the right LBAs, with a zlib-compatible CRC32 of the
//     streamed bytes in the CRC32 register;
//   * a bulk CMD18 CRC-verify op reads the range back and reproduces the
//     same CRC — and detects a deliberately corrupted card byte;
//   * BLKCNT guard rails (0 / > STAGE_SECTORS) error out cleanly;
//   * GO_VERIFY is POSITION-INDEPENDENT: the CRC32 reported for a given
//     (LBA, BLKCNT) pair does not depend on how many GO ops preceded it,
//     on their LBAs/sizes, or on whether a short (partial) batch ran
//     first — the property `tools/jtag_repl.tcl sd_verify_file` relies on
//     when it batches a long range into 256-sector chunks with a short
//     residue at the end.  See test_verify_range_independence /
//     test_verify_partial_then_full / test_verify_corruption_positions /
//     test_write_verify_alternation.
//
// Build: make tb-sd-provision

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <verilated.h>
#include "Vtb_sd_provision_top.h"

static Vtb_sd_provision_top* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

// Must match tb_sd_provision_top.v (= the production sd_provision_top).
static const uint32_t STAGE_SECTORS = 256;

// sd_bulk_writer address map.
static const uint32_t REG_LBA    = 0x00000000;
static const uint32_t REG_CTRL   = 0x00000004;
static const uint32_t REG_STATUS = 0x00000008;
static const uint32_t REG_BLKCNT = 0x0000000C;
static const uint32_t REG_CRC32  = 0x00000010;
static const uint32_t REG_IDENT  = 0x00000014;
static const uint32_t REG_CAPS   = 0x00000018;
static const uint32_t STAGE_BASE = 0x00100000;

static const uint32_t CTRL_GO_WRITE  = 0x5DB00001;
static const uint32_t CTRL_GO_VERIFY = 0x5DB00002;
static const uint32_t IDENT_VALUE    = 0x5DB70001;

// ═════════════════════════════════════════════════════════════════════
// zlib-compatible CRC32 (reflected, poly 0xEDB88320) — reference for the
// hardware CRC32 register.
// ═════════════════════════════════════════════════════════════════════
static uint32_t crc32_update(uint32_t crc, const uint8_t* p, size_t n) {
    crc = ~crc;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
static uint32_t crc32_of(const std::vector<uint8_t>& v) {
    return crc32_update(0, v.data(), v.size());
}

// ═════════════════════════════════════════════════════════════════════
// SD-card software model — bit-level at the SPI pins.  Init + CMD17/18
// read support adapted from tb_sd_boot.cpp; CMD24/CMD25 write capture
// and backing storage added for the bulk-writer scenarios.
// ═════════════════════════════════════════════════════════════════════
struct SdCard {
    std::vector<uint8_t> mosi_frame;
    bool                 in_frame = false;
    std::vector<uint8_t> miso_fifo;

    int  acmd41_replies = 0;

    bool     multi_read_active = false;
    uint32_t multi_read_lba    = 0;
    int      cmd12_count       = 0;

    // Write capture.
    enum WrPhase { WP_NONE, WP_WAIT_TOKEN, WP_DATA, WP_CRC };
    WrPhase  wr_phase   = WP_NONE;
    bool     wr_multi   = false;
    uint32_t wr_lba     = 0;
    int      wr_crc_cnt = 0;
    std::vector<uint8_t> wr_buf;

    int  cmd24_count = 0;
    int  cmd25_count = 0;
    int  stop_token_count = 0;
    bool bad = false;

    // Sector backing store.
    std::map<uint32_t, std::vector<uint8_t>> storage;

    void push_response(const std::vector<uint8_t>& v) {
        for (auto b : v) miso_fifo.push_back(b);
    }

    void queue_read_block(uint32_t sec) {
        std::vector<uint8_t> resp;
        resp.push_back(0xFF);
        resp.push_back(0xFE);
        auto it = storage.find(sec);
        for (int k = 0; k < 512; k++) {
            uint8_t b = (it != storage.end())
                            ? it->second[k]
                            : (uint8_t)((sec * 37 + k) & 0xFF);
            resp.push_back(b);
        }
        resp.push_back(0x00);
        resp.push_back(0x00);
        push_response(resp);
    }

    uint8_t pop_miso() {
        if (miso_fifo.empty() && multi_read_active) {
            queue_read_block(multi_read_lba);
            multi_read_lba++;
        }
        if (miso_fifo.empty()) return 0xFF;
        uint8_t b = miso_fifo.front();
        miso_fifo.erase(miso_fifo.begin());
        return b;
    }

    void commit_written_sector() {
        storage[wr_lba] = wr_buf;
        wr_lba++;
        wr_buf.clear();
        // Data-response token (accepted) + short busy + release.
        push_response({0xE5, 0x00, 0x00, 0x00, 0xFF});
        wr_phase = wr_multi ? WP_WAIT_TOKEN : WP_NONE;
    }

    void handle_frame() {
        uint8_t cmd = mosi_frame[0] & 0x3F;
        uint32_t arg = ((uint32_t)mosi_frame[1] << 24) |
                       ((uint32_t)mosi_frame[2] << 16) |
                       ((uint32_t)mosi_frame[3] << 8) |
                       (uint32_t)mosi_frame[4];
        switch (cmd) {
            case 0:  push_response({0xFF, 0x01}); break;
            case 8:  push_response({0xFF, 0x01, 0x00, 0x00, 0x01, 0xAA}); break;
            case 55: push_response({0xFF, 0x01}); break;
            case 41:
                if (acmd41_replies >= 1) push_response({0xFF, 0x00});
                else                     push_response({0xFF, 0x01});
                acmd41_replies++;
                break;
            case 58: push_response({0xFF, 0x00, 0xC0, 0xFF, 0x80, 0x00}); break;
            case 6:  push_response({0xFF, 0x04}); break;   // reject CMD6
            case 17:
                push_response({0xFF, 0x00});
                queue_read_block(arg);
                break;
            case 18:
                push_response({0xFF, 0x00});
                queue_read_block(arg);
                multi_read_active = true;
                multi_read_lba    = arg + 1;
                break;
            case 12:
                cmd12_count++;
                multi_read_active = false;
                miso_fifo.clear();
                push_response({0xFF, 0x00});
                break;
            case 24:
                cmd24_count++;
                wr_multi = false;
                wr_lba   = arg;
                wr_phase = WP_WAIT_TOKEN;
                wr_buf.clear();
                push_response({0xFF, 0x00});
                break;
            case 25:
                cmd25_count++;
                wr_multi = true;
                wr_lba   = arg;
                wr_phase = WP_WAIT_TOKEN;
                wr_buf.clear();
                push_response({0xFF, 0x00});
                break;
            default:
                push_response({0xFF, 0x04});
                break;
        }
    }

    void observe_mosi(uint8_t b) {
        // Write-capture phases take priority over frame parsing — data
        // bytes may alias command-start patterns.
        switch (wr_phase) {
            case WP_WAIT_TOKEN:
                if (b == 0xFE || b == 0xFC) {
                    if (wr_multi && b == 0xFE) bad = true;
                    if (!wr_multi && b == 0xFC) bad = true;
                    wr_phase = WP_DATA;
                    wr_buf.clear();
                } else if (b == 0xFD) {
                    // CMD25 stop-tran token.
                    if (!wr_multi) bad = true;
                    stop_token_count++;
                    wr_phase = WP_NONE;
                    // Busy after stop-tran, then release.
                    push_response({0x00, 0x00, 0xFF});
                }
                // else: 0xFF gap/poll bytes — ignore.
                return;
            case WP_DATA:
                wr_buf.push_back(b);
                if (wr_buf.size() == 512) {
                    wr_phase   = WP_CRC;
                    wr_crc_cnt = 0;
                }
                return;
            case WP_CRC:
                if (++wr_crc_cnt == 2) commit_written_sector();
                return;
            case WP_NONE:
            default:
                break;
        }

        if (!in_frame) {
            if ((b & 0xC0) == 0x40) {
                mosi_frame.clear();
                mosi_frame.push_back(b);
                in_frame = true;
            }
        } else {
            mosi_frame.push_back(b);
            if (mosi_frame.size() == 6) {
                handle_frame();
                in_frame = false;
            }
        }
    }
} sd;

// Bit-level SPI observer (same scheme as tb_sd_boot.cpp).
struct SpiObserver {
    int     rise_count = 0;
    uint8_t mosi_byte  = 0;
    uint8_t miso_byte  = 0xFF;
    int     miso_bits_left = 0;
    bool    last_clk   = false;
    bool    last_cs_n  = true;
} sp;

static void spi_tick() {
    bool clk_now  = dut->spi_clk;
    bool cs_n_now = dut->spi_cs_n;

    if (sp.last_cs_n && !cs_n_now) {
        sp.rise_count = 0;
        sp.mosi_byte  = 0;
    }
    sp.last_cs_n = cs_n_now;

    if (cs_n_now) {
        dut->spi_miso = 1;
        sp.last_clk   = clk_now;
        return;
    }

    bool rising  = (!sp.last_clk &&  clk_now);
    bool falling = ( sp.last_clk && !clk_now);

    if (rising) {
        sp.mosi_byte = (sp.mosi_byte << 1) | (dut->spi_mosi & 1);
        sp.rise_count++;
        if (sp.rise_count == 8) {
            sd.observe_mosi(sp.mosi_byte);
            sp.rise_count = 0;
            sp.mosi_byte  = 0;
        }
    }

    if (falling || rising) {
        if (sp.miso_bits_left == 0) {
            sp.miso_byte      = sd.pop_miso();
            sp.miso_bits_left = 8;
        }
        dut->spi_miso = (sp.miso_byte >> 7) & 1;
        if (falling) {
            sp.miso_byte       = (sp.miso_byte << 1) | 1;
            sp.miso_bits_left--;
        }
    } else {
        dut->spi_miso = (sp.miso_byte >> 7) & 1;
    }

    sp.last_clk = clk_now;
}

static void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    spi_tick();
    dut->eval();
    sim_time++;
}

static void idle_axi() {
    dut->s_awaddr = 0; dut->s_awlen = 0; dut->s_awvalid = 0;
    dut->s_wdata = 0;  dut->s_wstrb = 0; dut->s_wlast = 0; dut->s_wvalid = 0;
    dut->s_bready = 0;
    dut->s_araddr = 0; dut->s_arlen = 0; dut->s_arvalid = 0;
    dut->s_rready = 0;
}

static void reset() {
    idle_axi();
    dut->spi_miso = 1;
    dut->rst = 1;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

// ═════════════════════════════════════════════════════════════════════
// 32-bit burst AXI master driver (stands in for the JTAG-to-AXI IP).
// ═════════════════════════════════════════════════════════════════════
static bool axi_write_burst(uint32_t addr, const uint32_t* words, int n) {
    dut->s_awaddr  = addr;
    dut->s_awlen   = (uint8_t)(n - 1);
    dut->s_awvalid = 1;
    int guard = 0;
    while (!dut->s_awready) {
        tick();
        if (++guard > 10000) { printf("  AW timeout @0x%08x\n", addr); return false; }
    }
    tick();
    dut->s_awvalid = 0;

    for (int i = 0; i < n; i++) {
        dut->s_wdata  = words[i];
        dut->s_wstrb  = 0xF;
        dut->s_wlast  = (i == n - 1) ? 1 : 0;
        dut->s_wvalid = 1;
        guard = 0;
        while (!dut->s_wready) {
            tick();
            if (++guard > 10000) { printf("  W timeout beat %d\n", i); return false; }
        }
        tick();
    }
    dut->s_wvalid = 0;
    dut->s_wlast  = 0;

    guard = 0;
    while (!dut->s_bvalid) {
        tick();
        if (++guard > 10000) { printf("  B timeout @0x%08x\n", addr); return false; }
    }
    dut->s_bready = 1;
    tick();
    dut->s_bready = 0;
    return true;
}

static bool axi_write32(uint32_t addr, uint32_t data) {
    return axi_write_burst(addr, &data, 1);
}

static bool axi_read_burst(uint32_t addr, uint32_t* words, int n) {
    dut->s_araddr  = addr;
    dut->s_arlen   = (uint8_t)(n - 1);
    dut->s_arvalid = 1;
    int guard = 0;
    while (!dut->s_arready) {
        tick();
        if (++guard > 10000) { printf("  AR timeout @0x%08x\n", addr); return false; }
    }
    tick();
    dut->s_arvalid = 0;

    for (int i = 0; i < n; i++) {
        dut->s_rready = 1;
        guard = 0;
        while (!dut->s_rvalid) {
            tick();
            if (++guard > 10000) { printf("  R timeout beat %d\n", i); return false; }
        }
        words[i] = dut->s_rdata;
        bool last = dut->s_rlast;
        tick();
        if ((i == n - 1) != last) {
            printf("  RLAST mismatch at beat %d (rlast=%d)\n", i, (int)last);
            return false;
        }
    }
    dut->s_rready = 0;
    return true;
}

static uint32_t axi_read32(uint32_t addr) {
    uint32_t v = 0xDEADBEEF;
    axi_read_burst(addr, &v, 1);
    return v;
}

// Pack a byte stream into little-endian 32-bit staging words (byte k of
// the stream lives in word k>>2, bits [8*(k&3) +: 8]) — the same packing
// sd-write-fast uses on the host side.
static std::vector<uint32_t> pack_words(const std::vector<uint8_t>& bytes) {
    std::vector<uint32_t> w((bytes.size() + 3) / 4, 0);
    for (size_t k = 0; k < bytes.size(); k++)
        w[k >> 2] |= ((uint32_t)bytes[k]) << (8 * (k & 3));
    return w;
}

// Fill staging via 256-beat (1 KiB) bursts — the real JTAG txn shape.
static bool stage_fill(const std::vector<uint8_t>& bytes) {
    std::vector<uint32_t> w = pack_words(bytes);
    for (size_t off = 0; off < w.size(); off += 256) {
        int n = (int)((w.size() - off) < 256 ? (w.size() - off) : 256);
        if (!axi_write_burst(STAGE_BASE + (uint32_t)(off * 4), &w[off], n))
            return false;
    }
    return true;
}

static bool poll_op_done(uint64_t max_cycles, uint32_t* status_out) {
    uint64_t start = sim_time;
    while (sim_time - start < max_cycles) {
        for (int i = 0; i < 500; i++) tick();
        uint32_t st = axi_read32(REG_STATUS);
        if (((st & 1) == 0) && ((st & 2) != 0)) {
            *status_out = st;
            return true;
        }
    }
    *status_out = axi_read32(REG_STATUS);
    return false;
}

#define CHECK_TRUE(label, cond) do { \
    if (!(cond)) { printf("  FAIL %s\n", (label)); return false; } \
} while (0)

#define CHECK_EQ(label, got, exp) do { \
    uint32_t g = (uint32_t)(got); \
    uint32_t e = (uint32_t)(exp); \
    if (g != e) { \
        printf("  FAIL %s: got 0x%08x expected 0x%08x\n", (label), g, e); \
        return false; \
    } \
} while (0)

// ═════════════════════════════════════════════════════════════════════
// Bulk-op helpers (shared by the verify-position scenarios)
// ═════════════════════════════════════════════════════════════════════

// Budget for a 256-sector CMD18 verify: 131072 bytes x ~64 core cycles
// per SPI byte ~= 8.4M cycles.  7x headroom.
static const uint64_t OP_BUDGET = 60000000;

static bool do_op(uint32_t ctrl, uint32_t lba, uint32_t nsec,
                  uint32_t* crc_out, uint32_t* st_out) {
    if (!axi_write32(REG_LBA, lba))      { printf("  LBA write failed\n");    return false; }
    if (!axi_write32(REG_BLKCNT, nsec))  { printf("  BLKCNT write failed\n"); return false; }
    if (!axi_write32(REG_CTRL, ctrl))    { printf("  CTRL write failed\n");   return false; }
    uint32_t st = 0;
    if (!poll_op_done(OP_BUDGET, &st)) {
        printf("  op timeout (ctrl=0x%08x lba=%u n=%u status=0x%08x)\n",
               ctrl, lba, nsec, st);
        return false;
    }
    if (st & 0x4) {
        printf("  op error (ctrl=0x%08x lba=%u n=%u status=0x%08x cause=%u)\n",
               ctrl, lba, nsec, st, (st >> 4) & 0xF);
        return false;
    }
    if (st_out)  *st_out  = st;
    if (crc_out) *crc_out = axi_read32(REG_CRC32);
    return true;
}

static bool do_verify(uint32_t lba, uint32_t nsec, uint32_t* crc_out) {
    return do_op(CTRL_GO_VERIFY, lba, nsec, crc_out, nullptr);
}
static bool do_write(uint32_t lba, uint32_t nsec, uint32_t* crc_out) {
    return do_op(CTRL_GO_WRITE, lba, nsec, crc_out, nullptr);
}

// Deterministic, LBA-dependent card content.  LBA-dependent on purpose:
// if the verify path ever read the WRONG range, the CRC would change.
static uint8_t card_byte(uint32_t lba, int k) {
    uint32_t v = lba * 131u + (uint32_t)k * 7u + (lba >> 4) * 29u + 0x5Au;
    return (uint8_t)(v & 0xFF);
}

static void card_preload(uint32_t lba0, uint32_t nsec) {
    for (uint32_t s = 0; s < nsec; s++) {
        std::vector<uint8_t> v(512);
        for (int k = 0; k < 512; k++) v[k] = card_byte(lba0 + s, k);
        sd.storage[lba0 + s] = v;
    }
}

// Reference CRC32 read straight out of the card model's backing store, so
// a deliberate corruption is reflected in the expectation too.
static uint32_t card_crc(uint32_t lba0, uint32_t nsec) {
    uint32_t c = 0;
    for (uint32_t s = 0; s < nsec; s++) {
        auto it = sd.storage.find(lba0 + s);
        if (it == sd.storage.end()) {
            printf("  card_crc: sector %u not preloaded\n", lba0 + s);
            return 0xDEADBEEF;
        }
        c = crc32_update(c, it->second.data(), 512);
    }
    return c;
}

// Reset only the card model's PROTOCOL state (so a fresh init sequence is
// answered correctly); the sector backing store and the traffic counters
// survive.
static void sd_model_protocol_reset() {
    sd.mosi_frame.clear();
    sd.in_frame          = false;
    sd.miso_fifo.clear();
    sd.acmd41_replies    = 0;
    sd.multi_read_active = false;
    sd.multi_read_lba    = 0;
    sd.wr_phase          = SdCard::WP_NONE;
    sd.wr_multi          = false;
    sd.wr_lba            = 0;
    sd.wr_crc_cnt        = 0;
    sd.wr_buf.clear();
    sp = SpiObserver();
}

// Full DUT reset + card re-init: gives a GO op a genuinely empty history
// (position 0), which is what "the first batch of a run" means on HW.
static bool hard_reset_and_init() {
    sd_model_protocol_reset();
    reset();
    const uint64_t MAX_CYC = 3000000;
    uint64_t start = sim_time;
    while (sim_time - start < MAX_CYC && !dut->card_ready && !dut->init_error)
        tick();
    if (dut->init_error || !dut->card_ready) {
        printf("  re-init failed (card_ready=%d init_error=%d)\n",
               (int)dut->card_ready, (int)dut->init_error);
        return false;
    }
    return true;
}

// Run `n` GO_VERIFY ops that are deliberately UNLIKE the range under
// test: different LBAs, different (and non-uniform) block counts, so any
// carried counter / CRC / prefetch state would be left in a state that
// differs per `n`.
static bool run_predecessor_verifies(uint32_t region_lba, uint32_t region_sec,
                                     int n) {
    static const uint32_t sz[8] = { 1, 3, 17, 5, 33, 2, 21, 9 };
    for (int i = 0; i < n; i++) {
        uint32_t nsec = sz[i % 8];
        uint32_t off  = ((uint32_t)i * 13u) % (region_sec - nsec);
        uint32_t junk = 0;
        if (!do_verify(region_lba + off, nsec, &junk)) {
            printf("  predecessor verify %d (lba=%u n=%u) failed\n",
                   i, region_lba + off, nsec);
            return false;
        }
    }
    return true;
}

// ═════════════════════════════════════════════════════════════════════
// Scenarios
// ═════════════════════════════════════════════════════════════════════

static bool test_card_init_ready() {
    // boot_fsm init + 1 dummy sector against the modelled card.
    const uint64_t MAX_CYC = 3000000;
    uint64_t start = sim_time;
    while (sim_time - start < MAX_CYC && !dut->card_ready && !dut->init_error)
        tick();
    CHECK_TRUE("no init error", !dut->init_error);
    CHECK_TRUE("card_ready asserted", dut->card_ready);

    CHECK_EQ("IDENT", axi_read32(REG_IDENT), IDENT_VALUE);
    CHECK_EQ("CAPS", axi_read32(REG_CAPS), (1u << 16) | STAGE_SECTORS);
    uint32_t st = axi_read32(REG_STATUS);
    CHECK_TRUE("STATUS.card_ready", (st & 0x100) != 0);
    CHECK_TRUE("STATUS.init_error clear", (st & 0x200) == 0);
    CHECK_TRUE("STATUS idle", (st & 1) == 0);
    printf("  init complete at cycle %llu\n",
           (unsigned long long)(sim_time - start));
    return true;
}

static bool test_staging_burst_roundtrip() {
    // Two full 256-beat bursts + readback (single-beat and burst).
    std::vector<uint8_t> bytes(2048);
    for (size_t i = 0; i < bytes.size(); i++)
        bytes[i] = (uint8_t)((i * 13 + 5) & 0xFF);
    CHECK_TRUE("stage fill", stage_fill(bytes));

    std::vector<uint32_t> w = pack_words(bytes);
    CHECK_EQ("word[0]", axi_read32(STAGE_BASE + 0), w[0]);
    CHECK_EQ("word[255]", axi_read32(STAGE_BASE + 255 * 4), w[255]);
    CHECK_EQ("word[256]", axi_read32(STAGE_BASE + 256 * 4), w[256]);
    CHECK_EQ("word[511]", axi_read32(STAGE_BASE + 511 * 4), w[511]);

    uint32_t rb[16];
    CHECK_TRUE("burst read", axi_read_burst(STAGE_BASE + 128 * 4, rb, 16));
    for (int i = 0; i < 16; i++)
        CHECK_EQ("burst read word", rb[i], w[128 + i]);
    return true;
}

static bool test_bulk_write_sectors() {
    // 48 sectors (a few dozen, > one BRAM row, multiple bursts) at the
    // real disk-image base LBA 8192.
    const int      NSEC = 48;
    const uint32_t LBA  = 8192;

    std::vector<uint8_t> bytes(NSEC * 512);
    for (int s = 0; s < NSEC; s++)
        for (int k = 0; k < 512; k++)
            bytes[s * 512 + k] = (uint8_t)((s * 7 + k * 3 + 1) & 0xFF);

    CHECK_TRUE("stage fill", stage_fill(bytes));
    CHECK_TRUE("set LBA", axi_write32(REG_LBA, LBA));
    CHECK_TRUE("set BLKCNT", axi_write32(REG_BLKCNT, NSEC));
    int cmd25_before = sd.cmd25_count;
    int stop_before  = sd.stop_token_count;
    CHECK_TRUE("GO write", axi_write32(REG_CTRL, CTRL_GO_WRITE));

    uint32_t st = 0;
    CHECK_TRUE("op completes", poll_op_done(30000000, &st));
    if (st & 0x4) {
        printf("  FAIL op error, status=0x%08x\n", st);
        return false;
    }
    CHECK_EQ("one CMD25", sd.cmd25_count - cmd25_before, 1);
    CHECK_EQ("one stop token", sd.stop_token_count - stop_before, 1);
    CHECK_TRUE("model clean", !sd.bad);

    for (int s = 0; s < NSEC; s++) {
        auto it = sd.storage.find(LBA + s);
        if (it == sd.storage.end()) {
            printf("  FAIL sector %u never written\n", LBA + s);
            return false;
        }
        for (int k = 0; k < 512; k++) {
            if (it->second[k] != bytes[s * 512 + k]) {
                printf("  FAIL LBA %u byte %d: got 0x%02x expected 0x%02x\n",
                       LBA + s, k, it->second[k], bytes[s * 512 + k]);
                return false;
            }
        }
    }
    // No stray sectors outside [LBA, LBA+NSEC) — except the guard test's
    // later writes; at this point storage must be exactly the range.
    for (auto& kv : sd.storage) {
        if (kv.first < LBA || kv.first >= LBA + NSEC) {
            printf("  FAIL stray sector %u written\n", kv.first);
            return false;
        }
    }

    CHECK_EQ("CRC32 (write stream)", axi_read32(REG_CRC32), crc32_of(bytes));
    return true;
}

static bool test_bulk_verify_crc() {
    const int      NSEC = 48;
    const uint32_t LBA  = 8192;

    std::vector<uint8_t> bytes(NSEC * 512);
    for (int s = 0; s < NSEC; s++)
        for (int k = 0; k < 512; k++)
            bytes[s * 512 + k] = (uint8_t)((s * 7 + k * 3 + 1) & 0xFF);
    uint32_t expect = crc32_of(bytes);

    CHECK_TRUE("set LBA", axi_write32(REG_LBA, LBA));
    CHECK_TRUE("set BLKCNT", axi_write32(REG_BLKCNT, NSEC));
    CHECK_TRUE("GO verify", axi_write32(REG_CTRL, CTRL_GO_VERIFY));
    uint32_t st = 0;
    CHECK_TRUE("verify completes", poll_op_done(30000000, &st));
    CHECK_TRUE("verify no error", (st & 0x4) == 0);
    CHECK_EQ("CRC32 (verify pass)", axi_read32(REG_CRC32), expect);

    // Corrupt one byte on the card; the CRC must now differ.
    sd.storage[LBA + 17][100] ^= 0xFF;
    CHECK_TRUE("GO verify 2", axi_write32(REG_CTRL, CTRL_GO_VERIFY));
    CHECK_TRUE("verify 2 completes", poll_op_done(30000000, &st));
    uint32_t crc2 = axi_read32(REG_CRC32);
    CHECK_TRUE("corruption detected", crc2 != expect);
    sd.storage[LBA + 17][100] ^= 0xFF;   // restore
    return true;
}

static bool test_blkcnt_guard_and_single_sector() {
    // BLKCNT = 0 → local error 0xE, no SD traffic.
    int cmd25_before = sd.cmd25_count;
    CHECK_TRUE("set BLKCNT 0", axi_write32(REG_BLKCNT, 0));
    CHECK_TRUE("GO", axi_write32(REG_CTRL, CTRL_GO_WRITE));
    for (int i = 0; i < 50; i++) tick();
    uint32_t st = axi_read32(REG_STATUS);
    CHECK_TRUE("blkcnt=0 done", (st & 2) != 0);
    CHECK_TRUE("blkcnt=0 error", (st & 4) != 0);
    CHECK_EQ("blkcnt=0 err_cause", (st >> 4) & 0xF, 0xE);
    CHECK_EQ("blkcnt=0 no CMD25", sd.cmd25_count - cmd25_before, 0);

    // BLKCNT > STAGE_SECTORS → same guard.
    CHECK_TRUE("set BLKCNT 65", axi_write32(REG_BLKCNT, STAGE_SECTORS + 1));
    CHECK_TRUE("GO", axi_write32(REG_CTRL, CTRL_GO_WRITE));
    for (int i = 0; i < 50; i++) tick();
    st = axi_read32(REG_STATUS);
    CHECK_TRUE("blkcnt=65 error", (st & 4) != 0);
    CHECK_EQ("blkcnt=65 err_cause", (st >> 4) & 0xF, 0xE);

    // Tail-round equivalent: single sector at a different LBA.
    std::vector<uint8_t> bytes(512);
    for (int k = 0; k < 512; k++) bytes[k] = (uint8_t)((k * 11 + 3) & 0xFF);
    CHECK_TRUE("stage fill", stage_fill(bytes));
    CHECK_TRUE("set LBA", axi_write32(REG_LBA, 9000));
    CHECK_TRUE("set BLKCNT 1", axi_write32(REG_BLKCNT, 1));
    CHECK_TRUE("GO", axi_write32(REG_CTRL, CTRL_GO_WRITE));
    CHECK_TRUE("1-sector op completes", poll_op_done(10000000, &st));
    CHECK_TRUE("1-sector no error", (st & 4) == 0);
    auto it = sd.storage.find(9000);
    CHECK_TRUE("sector 9000 written", it != sd.storage.end());
    for (int k = 0; k < 512; k++) {
        if (it->second[k] != bytes[k]) {
            printf("  FAIL LBA 9000 byte %d: got 0x%02x expected 0x%02x\n",
                   k, it->second[k], bytes[k]);
            return false;
        }
    }
    CHECK_EQ("CRC32 (1 sector)", axi_read32(REG_CRC32), crc32_of(bytes));
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Verify-position scenarios.
//
// Motivation (2026-08 HW observation): `sd_disk_diff.sh repair <img> 0
// 95411` reported CLEAN, and minutes later `... 0 1024000` reported four
// damaged batches (file-relative sectors 0, 83200, 87552, 87808 = batch
// indices 0, 325, 342, 343) — all inside the range that had just
// certified clean.  Both runs start at LBA 8192 and step 256, so the
// batch grid is bit-identical; only the FINAL partial batch differs
// (95411 = 372*256 + 179).  The hypothesis under test is that a
// GO_VERIFY's reported CRC32 depends on its position in the sequence of
// preceding ops (i.e. on carried state), which would make a narrow range
// miss damage a wide range finds.
// ─────────────────────────────────────────────────────────────────────

// A big preloaded region, comfortably larger than a 256-sector batch.
static const uint32_t VR_BASE = 8192;
static const uint32_t VR_SEC  = 1024;

static bool test_verify_range_independence() {
    card_preload(VR_BASE, VR_SEC);

    const uint32_t B      = 16;                 // batch size under test
    const uint32_t TARGET = VR_BASE + 5 * B;
    const uint32_t expect = card_crc(TARGET, B);

    // Position 0: the very first GO op after reset + card init.
    CHECK_TRUE("re-init (pos 0)", hard_reset_and_init());
    uint32_t crc0 = 0;
    CHECK_TRUE("verify at position 0", do_verify(TARGET, B, &crc0));
    CHECK_EQ("position-0 CRC vs card model", crc0, expect);

    // Positions 1, 2, 8: N preceding GO_VERIFYs of assorted LBAs/sizes.
    static const int NPRED[3] = { 1, 2, 8 };
    for (int t = 0; t < 3; t++) {
        int n = NPRED[t];
        CHECK_TRUE("re-init (pos N)", hard_reset_and_init());
        CHECK_TRUE("predecessor verifies",
                   run_predecessor_verifies(VR_BASE, VR_SEC, n));
        uint32_t crcN = 0;
        CHECK_TRUE("verify at position N", do_verify(TARGET, B, &crcN));
        if (crcN != crc0) {
            printf("  FAIL position dependence: CRC after %d predecessors "
                   "= 0x%08x, at position 0 = 0x%08x\n", n, crcN, crc0);
            return false;
        }
        CHECK_EQ("position-N CRC vs card model", crcN, expect);
    }

    // Same range twice back to back, no reset in between.
    uint32_t a = 0, b = 0;
    CHECK_TRUE("verify A", do_verify(TARGET, B, &a));
    CHECK_TRUE("verify B", do_verify(TARGET, B, &b));
    CHECK_EQ("back-to-back CRC stable", b, a);
    CHECK_EQ("back-to-back CRC vs position 0", a, crc0);
    printf("  CRC 0x%08x reproduced at positions 0,1,2,8 and back to back\n",
           crc0);
    return true;
}

static bool test_verify_partial_then_full() {
    card_preload(VR_BASE, VR_SEC);

    // The real shapes: 95411 sectors = 372 full 256-sector batches + a
    // 179-sector residue.  LBA_A/LBA_B are disjoint so a leaked LBA or
    // block count would change the CRC.
    const uint32_t LBA_A  = VR_BASE;             // partial batch site
    const uint32_t N_PART = 179;
    const uint32_t LBA_B  = VR_BASE + 256;       // full batch site
    const uint32_t N_FULL = 256;

    const uint32_t exp_part = card_crc(LBA_A, N_PART);
    const uint32_t exp_full = card_crc(LBA_B, N_FULL);

    // Standalone full 256-sector batch (position 0).
    CHECK_TRUE("re-init (full alone)", hard_reset_and_init());
    uint32_t full_alone = 0;
    CHECK_TRUE("full verify alone", do_verify(LBA_B, N_FULL, &full_alone));
    CHECK_EQ("full-alone CRC vs card model", full_alone, exp_full);

    // Partial batch first, then the same full batch.
    CHECK_TRUE("re-init (partial then full)", hard_reset_and_init());
    uint32_t part = 0;
    CHECK_TRUE("partial verify", do_verify(LBA_A, N_PART, &part));
    CHECK_EQ("partial CRC vs card model", part, exp_part);
    uint32_t full_after = 0;
    CHECK_TRUE("full verify after partial", do_verify(LBA_B, N_FULL, &full_after));
    if (full_after != full_alone) {
        printf("  FAIL partial batch poisoned the next full batch: "
               "0x%08x after a %u-sector op vs 0x%08x standalone\n",
               full_after, N_PART, full_alone);
        return false;
    }

    // ...and the reverse order: the full batch must not poison a partial.
    uint32_t part_after = 0;
    CHECK_TRUE("partial verify after full", do_verify(LBA_A, N_PART, &part_after));
    CHECK_EQ("partial CRC after a full batch", part_after, part);

    // A 1-sector op between them (the most extreme residue) too.
    uint32_t one = 0;
    CHECK_TRUE("1-sector verify", do_verify(LBA_B + 300, 1, &one));
    CHECK_EQ("1-sector CRC vs card model", one, card_crc(LBA_B + 300, 1));
    uint32_t full_after_one = 0;
    CHECK_TRUE("full verify after 1-sector",
               do_verify(LBA_B, N_FULL, &full_after_one));
    CHECK_EQ("full CRC after a 1-sector op", full_after_one, full_alone);
    printf("  256-sector CRC 0x%08x stable standalone / after 179 / after 1\n",
           full_alone);
    return true;
}

static bool test_verify_corruption_positions() {
    // Positive control: without this, "everything matched" would only
    // prove the harness is inert.
    card_preload(VR_BASE, VR_SEC);

    const uint32_t B      = 16;
    const uint32_t TARGET = VR_BASE + 7 * B;
    const uint32_t clean  = card_crc(TARGET, B);

    // Corrupt one byte inside the batch (not the first or last sector).
    sd.storage[TARGET + 9][300] ^= 0xFF;
    const uint32_t dirty = card_crc(TARGET, B);
    CHECK_TRUE("model CRC changed by corruption", dirty != clean);

    // Detected as the FIRST op after reset...
    CHECK_TRUE("re-init (dirty pos 0)", hard_reset_and_init());
    uint32_t c_first = 0;
    CHECK_TRUE("dirty verify at position 0", do_verify(TARGET, B, &c_first));
    if (c_first == clean) {
        printf("  FAIL corruption invisible at position 0 (CRC 0x%08x)\n", c_first);
        return false;
    }
    CHECK_EQ("dirty position-0 CRC vs card model", c_first, dirty);

    // ...and equally as the 6th op.
    CHECK_TRUE("re-init (dirty pos N)", hard_reset_and_init());
    CHECK_TRUE("predecessor verifies",
               run_predecessor_verifies(VR_BASE, VR_SEC, 5));
    uint32_t c_nth = 0;
    CHECK_TRUE("dirty verify at position 5", do_verify(TARGET, B, &c_nth));
    if (c_nth == clean) {
        printf("  FAIL corruption invisible after 5 predecessors "
               "(CRC 0x%08x)\n", c_nth);
        return false;
    }
    CHECK_EQ("dirty position-N CRC matches position 0", c_nth, c_first);

    // Corruption inside a 256-sector batch is caught after a 179-sector
    // partial batch too — the exact shape of the HW observation.
    const uint32_t WLBA = VR_BASE + 512;
    const uint32_t wclean = card_crc(WLBA, 256);
    sd.storage[WLBA + 200][11] ^= 0x5A;
    const uint32_t wdirty = card_crc(WLBA, 256);
    CHECK_TRUE("wide model CRC changed", wdirty != wclean);
    uint32_t junk = 0;
    CHECK_TRUE("179-sector predecessor", do_verify(VR_BASE, 179, &junk));
    uint32_t wgot = 0;
    CHECK_TRUE("wide dirty verify", do_verify(WLBA, 256, &wgot));
    CHECK_EQ("wide dirty CRC vs card model", wgot, wdirty);
    CHECK_TRUE("wide corruption detected", wgot != wclean);

    // Restore both bytes; the CRCs must come back clean.
    sd.storage[TARGET + 9][300] ^= 0xFF;
    sd.storage[WLBA + 200][11]  ^= 0x5A;
    uint32_t c_restored = 0;
    CHECK_TRUE("restored verify", do_verify(TARGET, B, &c_restored));
    CHECK_EQ("restored CRC clean", c_restored, clean);
    uint32_t w_restored = 0;
    CHECK_TRUE("wide restored verify", do_verify(WLBA, 256, &w_restored));
    CHECK_EQ("wide restored CRC clean", w_restored, wclean);
    return true;
}

static bool test_write_verify_alternation() {
    // A verify following a write must report the WRITTEN content, and the
    // write's staging buffer must not leak into a later verify's CRC.
    const uint32_t NSEC  = 24;
    const uint32_t LBA_W = 40000;
    const uint32_t LBA_V = 41000;

    card_preload(LBA_V, NSEC);
    const uint32_t exp_v = card_crc(LBA_V, NSEC);

    std::vector<uint8_t> p1(NSEC * 512), p2(NSEC * 512);
    for (size_t i = 0; i < p1.size(); i++) {
        p1[i] = (uint8_t)((i * 31 + 17) & 0xFF);
        p2[i] = (uint8_t)((i * 97 + 200) & 0xFF);
    }

    // Stage + write P1, then verify the same range.
    CHECK_TRUE("stage P1", stage_fill(p1));
    uint32_t wcrc = 0;
    CHECK_TRUE("write P1", do_write(LBA_W, NSEC, &wcrc));
    CHECK_EQ("write CRC == CRC(P1)", wcrc, crc32_of(p1));
    uint32_t vcrc = 0;
    CHECK_TRUE("verify written range", do_verify(LBA_W, NSEC, &vcrc));
    CHECK_EQ("verify reads back P1", vcrc, crc32_of(p1));

    // Now stage P2 WITHOUT writing it, and verify an unrelated range.
    // The staged bytes must not reach the verify CRC.
    CHECK_TRUE("stage P2", stage_fill(p2));
    uint32_t vcrc2 = 0;
    CHECK_TRUE("verify unrelated range", do_verify(LBA_V, NSEC, &vcrc2));
    CHECK_EQ("staged P2 does not leak into verify", vcrc2, exp_v);

    // Commit P2 elsewhere, verify it, then re-verify P1's range: the
    // second write must not have disturbed the first.
    uint32_t wcrc2 = 0;
    CHECK_TRUE("write P2", do_write(LBA_W + 100, NSEC, &wcrc2));
    CHECK_EQ("write CRC == CRC(P2)", wcrc2, crc32_of(p2));
    uint32_t vcrc3 = 0;
    CHECK_TRUE("verify P2 range", do_verify(LBA_W + 100, NSEC, &vcrc3));
    CHECK_EQ("verify reads back P2", vcrc3, crc32_of(p2));
    uint32_t vcrc4 = 0;
    CHECK_TRUE("re-verify P1 range", do_verify(LBA_W, NSEC, &vcrc4));
    CHECK_EQ("P1 range still P1", vcrc4, crc32_of(p1));

    // Card model backing store agrees byte for byte.
    for (uint32_t s = 0; s < NSEC; s++) {
        auto it = sd.storage.find(LBA_W + s);
        CHECK_TRUE("P1 sector present", it != sd.storage.end());
        if (memcmp(it->second.data(), &p1[s * 512], 512) != 0) {
            printf("  FAIL LBA %u does not hold P1\n", LBA_W + s);
            return false;
        }
    }
    CHECK_TRUE("model clean", !sd.bad);
    return true;
}

#define RUN(fn) do { \
    printf("Running " #fn "...\n"); \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    // Self-check the reference CRC32 against the canonical vector.
    {
        const char* v = "123456789";
        uint32_t c = crc32_update(0, (const uint8_t*)v, 9);
        if (c != 0xCBF43926) {
            printf("reference CRC32 self-check failed: 0x%08x\n", c);
            return 1;
        }
    }

    dut = new Vtb_sd_provision_top;
    reset();

    RUN(test_card_init_ready);
    RUN(test_staging_burst_roundtrip);
    RUN(test_bulk_write_sectors);
    RUN(test_bulk_verify_crc);
    RUN(test_blkcnt_guard_and_single_sector);
    RUN(test_verify_range_independence);
    RUN(test_verify_partial_then_full);
    RUN(test_verify_corruption_positions);
    RUN(test_write_verify_alternation);

    printf("\nsd_provision: %d passed, %d failed\n", n_pass, n_fail);
    dut->final();
    delete dut;
    return n_fail ? 1 : 0;
}
