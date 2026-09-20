// tb_sd_ctrl.cpp — Focused unit test for rtl/board/sd_ctrl.v
//
// The RUN() list in main() is the authority on what runs; the summary
// below predates the CRC16 and watchdog scenarios and is kept only as
// an orientation for the original eight.
//
// Scenarios:
//   1. CMD17 single-block read — canned pattern landed in rd stream
//   2. CMD18 multi-block read  — 4 blocks → verify each block + CMD12 stop
//   3. CMD24 single-block write — pattern echoed from wr stream lands at SD
//   4. CMD25 multi-block write — 4 blocks, all stored + stop-tran received
//   5. R1-bad error — card returns R1=0x04 → error + ERR_R1_BAD
//   6. R1 timeout  — silent card → error + ERR_R1_TO
//   7. Write reject (DR bad) — card returns data-response != 0x05 → ERR_DR_BAD
//   8. Unknown cmd_type — caller passes CT=5 → error + ERR_UNK_CMD
//
// Build: make tb-sd-ctrl

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <deque>
#include <map>
#include <initializer_list>
#include <verilated.h>
#include "Vtb_sd_ctrl.h"

static Vtb_sd_ctrl* dut      = nullptr;
static uint64_t     sim_time = 0;
static int          n_pass   = 0;
static int          n_fail   = 0;

// Command type encodings (must match sd_ctrl.v localparams)
static const uint8_t CT_IDLE  = 0;
static const uint8_t CT_CMD17 = 1;
static const uint8_t CT_CMD18 = 2;
static const uint8_t CT_CMD24 = 3;
static const uint8_t CT_CMD25 = 4;

// Error causes
static const uint8_t ERR_NONE    = 0;
static const uint8_t ERR_R1_BAD  = 1;
static const uint8_t ERR_R1_TO   = 2;
static const uint8_t ERR_TOK_TO  = 3;
static const uint8_t ERR_DR_BAD  = 4;
static const uint8_t ERR_BUSY_TO = 5;
static const uint8_t ERR_UNK_CMD = 6;
static const uint8_t ERR_CRC_BAD = 7;
static const uint8_t ERR_WDOG    = 8;   // global per-request watchdog

// Write-path state and byte-engine encodings used only to select reset
// landing points.  Keep these mirrors adjacent to the test that exercises
// every phase so an encoding change fails visibly instead of losing coverage.
static const uint8_t S_W_GAP_S   = 11;
static const uint8_t S_W_GAP_W   = 12;
static const uint8_t S_W_TOK_S   = 13;
static const uint8_t S_W_TOK_W   = 14;
static const uint8_t S_W_DATA_S  = 15;
static const uint8_t S_W_DATA_W  = 16;
static const uint8_t S_W_DCRC_S  = 17;
static const uint8_t S_W_DCRC_W  = 18;
static const uint8_t S_W_RESP_S  = 19;
static const uint8_t S_W_RESP_W  = 20;
static const uint8_t S_W_BUSY_S  = 21;
static const uint8_t S_W_BUSY_W  = 22;
static const uint8_t S_W_STOP_S  = 23;
static const uint8_t S_W_STOP_W  = 24;
static const uint8_t S_W_FBUSY_S = 25;
static const uint8_t S_W_FBUSY_W = 26;
static const uint8_t BS_IDLE      = 0;
static const uint8_t BS_SEND      = 1;
static const uint8_t BS_WAIT      = 2;

// CRC16-CCITT (poly 0x1021, init 0x0000, non-reflected) — matches
// sd_ctrl.v's crc16_step function bit-for-bit.  Standard SD/MMC
// data-block CRC.
static uint16_t crc16_ccitt(const std::vector<uint8_t>& data) {
    uint16_t crc = 0x0000;
    for (uint8_t b : data) {
        crc ^= (uint16_t)b << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// ─────────────────────────────────────────────────────────────────────
// SD-card software model — speaks CMD17/18/24/25/12.
// ─────────────────────────────────────────────────────────────────────
struct SdCard {
    // Control knobs for the per-scenario setup
    bool silent         = false;    // never push any bytes at all
    uint8_t cmd17_r1    = 0x00;     // R1 to send for CMD17 (0x00 = OK)
    uint8_t cmd18_r1    = 0x00;
    uint8_t cmd24_r1    = 0x00;
    uint8_t cmd25_r1    = 0x00;
    uint8_t cmd12_r1    = 0x00;
    // The byte sent immediately after CMD12's frame, before the real R1
    // (SD Physical Layer spec: CMD12-during-read gets one undefined
    // "stuff" byte the card is still finishing internal pipeline state
    // on). Real hardware does NOT guarantee this is 0xFF — default here
    // matches every pre-existing scenario's assumption (0xFF, i.e. "no
    // real stuff byte content to speak of"); the dedicated
    // test_cmd12_nonff_stuff_byte scenario below sets it to a non-0xFF,
    // MSB-clear value to catch a real-HW bug (2026-07-24) where sd_ctrl's
    // generic R1 poll misread this byte as a bogus real R1.
    uint8_t cmd12_stuff_byte = 0xFF;
    uint8_t write_dr    = 0xE5;     // data-response byte for accepted writes
    // Reject the data block at this serial position within the current
    // write command (0 = the first block).  -1 = never.  Models a card
    // that refuses ONE block mid-CMD25 — the trigger that leaves the
    // sequential-write session open if the host does not close it.
    int     reject_block_serial = -1;
    int     w_block_serial      = 0;
    // How many blocks this card refused because the data-block CRC16 the
    // host sent did not match the bytes it actually received.  A real
    // card validates; this model used to capture unconditionally, which
    // made a truncated/padded block indistinguishable from a good one.
    int     crc_rejected_blocks = 0;
    // Whether this card is validating the data-block CRC16 at all.  On
    // the real platform that is a whole-session setting the host chooses
    // with CMD59/CRC_ON_OFF (boot_fsm.v sends it), so BOTH values are
    // real configurations of the shipping hardware.  With validation off
    // the card commits whatever 512 bytes it counted, which is what turns
    // an abandoned write session from "the next command fails" into "the
    // next command's bytes are written into the disk image".
    bool    validates_write_crc = true;

    // CRC fault injection: the queued read block at serial position
    // `corrupt_block_serial` (0 = the very first block of the command,
    // 1 = the second, ...) gets its CRC16 field deliberately corrupted
    // (real CRC XORed with 0xFFFF).  -1 (default) = never corrupt.
    int corrupt_block_serial = -1;
    int block_serial_counter = 0;
    bool corrupt_all_blocks  = false;   // persistent-failure scenario
    // How many CMD17/18 command frames the card has actually seen —
    // lets a scenario confirm a whole-command retry really re-sent the
    // frame, not just re-derived success from stale state.
    int cmd17_frames_seen = 0;
    int cmd18_frames_seen = 0;

    // MOSI parsing
    std::vector<uint8_t> mosi_frame;
    bool                 in_frame       = false;

    // Write data phase tracking
    enum WState { W_IDLE, W_AWAIT_TOKEN, W_RECV_DATA, W_RECV_CRC,
                  W_RESPOND, W_BUSY };
    WState      wstate         = W_IDLE;
    int         w_data_count   = 0;
    int         w_crc_count    = 0;
    uint8_t     w_crc_hi       = 0;
    uint8_t     w_crc_lo       = 0;
    int         w_busy_ticks   = 0;
    int         write_busy_ticks = 4;
    uint32_t    w_lba          = 0;
    uint8_t     w_expected_cmd = 0;   // 24 or 25
    std::vector<uint8_t> w_block_bytes;

    // Set when the 0xFD stop-tran token arrives, so the busy window that
    // follows returns the card to command-wait instead of back to
    // "waiting for the next data token".  Without this the model could
    // never represent a CLOSED CMD25 session, i.e. it could not tell the
    // difference the whole PRAM/HDD bug turns on.
    bool     stop_pending = false;

    // Captures
    std::map<uint32_t, std::vector<uint8_t>> captured_writes;
    std::map<uint32_t, uint16_t> captured_write_crcs;
    bool stop_tran_seen = false;      // 0xFD received in MOSI stream
    int  cmd12_count    = 0;          // how many CMD12 frames we saw
    std::vector<uint8_t> last_cmd12_frame;  // raw 6 bytes of the last CMD12 seen

    // "The card is in a state only the host can leave."  True from the
    // moment a CMD24/CMD25 is accepted until its block completes
    // (CMD24) or its stop-tran token is honoured (CMD25).  While this
    // is true the card is NOT listening for commands — every byte the
    // next master puts on MOSI is write data.
    bool write_session_open() const { return wstate != W_IDLE; }

    // Canned read data
    std::map<uint32_t, std::vector<uint8_t>> canned_reads;

    // MISO byte source
    std::deque<uint8_t> miso_fifo;

    // Multi-block read tracking
    bool     multi_read_active = false;
    uint32_t multi_read_lba    = 0;
    uint16_t multi_read_remaining = 0;

    void push(uint8_t b) { if (!silent) miso_fifo.push_back(b); }
    void push(std::initializer_list<uint8_t> v) {
        if (silent) return;
        for (auto b : v) miso_fifo.push_back(b);
    }
    void push(const std::vector<uint8_t>& v) {
        if (silent) return;
        for (auto b : v) miso_fifo.push_back(b);
    }

    uint8_t pop_miso() {
        // Busy-phase: hold MISO low for w_busy_ticks bytes.
        if (wstate == W_BUSY) {
            if (miso_fifo.empty()) {
                if (w_busy_ticks > 0) {
                    w_busy_ticks--;
                    return 0x00;
                } else {
                    // Busy released.  For CMD25 multi-block we stay
                    // alive to receive the next 0xFC / 0xFD token; for
                    // CMD24 we return to idle.  A CMD25 whose stop-tran
                    // token has already arrived is CLOSED — that is the
                    // only way out of sequential-write mode.
                    if (w_expected_cmd == 25 && !stop_pending) {
                        wstate = W_AWAIT_TOKEN;
                    } else {
                        wstate       = W_IDLE;
                        stop_pending = false;
                    }
                }
            }
        }
        // If there's a pending multi-block read, queue up the next block
        // only once the FIFO has drained.
        if (miso_fifo.empty() && multi_read_active && multi_read_remaining > 0) {
            queue_read_block(multi_read_lba, true);
            multi_read_lba++;
            multi_read_remaining--;
            if (multi_read_remaining == 0) multi_read_active = false;
        }
        if (miso_fifo.empty()) return 0xFF;
        uint8_t b = miso_fifo.front();
        miso_fifo.pop_front();
        return b;
    }

    std::vector<uint8_t> pattern_for_lba(uint32_t lba) {
        auto it = canned_reads.find(lba);
        if (it != canned_reads.end()) {
            std::vector<uint8_t> d = it->second;
            d.resize(512, 0);
            return d;
        }
        std::vector<uint8_t> d(512);
        for (int k = 0; k < 512; k++) d[k] = (uint8_t)((lba * 37 + k) & 0xFF);
        return d;
    }

    // Queue a single block of read data: optional gap byte, 0xFE token,
    // 512 bytes, a real computed CRC16 (or a deliberately-wrong one, see
    // bad_crc_blocks).  `with_gap` true adds a 0xFF byte before the 0xFE
    // (helps exercise the token-polling loop).
    void queue_read_block(uint32_t lba, bool with_gap) {
        std::vector<uint8_t> resp;
        if (with_gap) resp.push_back(0xFF);
        resp.push_back(0xFE);
        auto data = pattern_for_lba(lba);
        resp.insert(resp.end(), data.begin(), data.end());
        uint16_t crc = crc16_ccitt(data);
        if (corrupt_all_blocks || block_serial_counter == corrupt_block_serial)
            crc ^= 0xFFFF;
        block_serial_counter++;
        resp.push_back((uint8_t)(crc >> 8));
        resp.push_back((uint8_t)(crc & 0xFF));
        push(resp);
    }

    void handle_cmd_frame() {
        uint8_t cmdbyte = mosi_frame[0];
        uint8_t cmd     = cmdbyte & 0x3F;
        uint32_t lba = ((uint32_t)mosi_frame[1] << 24) |
                       ((uint32_t)mosi_frame[2] << 16) |
                       ((uint32_t)mosi_frame[3] <<  8) |
                       ((uint32_t)mosi_frame[4]);
        switch (cmd) {
            case 17: {
                cmd17_frames_seen++;
                push({0xFF, cmd17_r1});
                if (cmd17_r1 == 0) queue_read_block(lba, true);
                break;
            }
            case 18: {
                cmd18_frames_seen++;
                push({0xFF, cmd18_r1});
                if (cmd18_r1 == 0) {
                    // Queue the first block immediately; subsequent blocks
                    // are queued on demand from pop_miso().
                    queue_read_block(lba, true);
                    multi_read_active = true;
                    multi_read_lba    = lba + 1;
                    multi_read_remaining = 0xFFFF;   // until CMD12 arrives
                }
                break;
            }
            case 12: {
                // STOP_TRANSMISSION — end multi-block read.
                cmd12_count++;
                last_cmd12_frame = mosi_frame;
                multi_read_active    = false;
                multi_read_remaining = 0;
                // Also flush any partial block we were streaming.
                miso_fifo.clear();
                // Upstream's CMD12 R1: may come after stuff byte. Give
                // one stuff byte + R1.
                push({cmd12_stuff_byte, cmd12_r1});
                break;
            }
            case 24: {
                push({0xFF, cmd24_r1});
                if (cmd24_r1 == 0) {
                    wstate         = W_AWAIT_TOKEN;
                    w_data_count   = 0;
                    w_crc_count    = 0;
                    w_lba          = lba;
                    w_expected_cmd = 24;
                    w_block_serial = 0;
                    stop_pending   = false;
                    w_block_bytes.clear();
                }
                break;
            }
            case 25: {
                push({0xFF, cmd25_r1});
                if (cmd25_r1 == 0) {
                    wstate         = W_AWAIT_TOKEN;
                    w_data_count   = 0;
                    w_crc_count    = 0;
                    w_lba          = lba;
                    w_expected_cmd = 25;
                    w_block_serial = 0;
                    stop_pending   = false;
                    w_block_bytes.clear();
                }
                break;
            }
            default:
                push({0xFF, (uint8_t)0x04});
                break;
        }
    }

    void observe_mosi(uint8_t b) {
        // In write data phase, consume bytes regardless of 0b01 framing.
        switch (wstate) {
            case W_AWAIT_TOKEN:
                if (w_expected_cmd == 25 && b == 0xFC) {
                    wstate = W_RECV_DATA;
                    w_data_count = 0;
                    w_block_bytes.clear();
                    return;
                }
                if (w_expected_cmd == 24 && b == 0xFE) {
                    wstate = W_RECV_DATA;
                    w_data_count = 0;
                    w_block_bytes.clear();
                    return;
                }
                // Stop-tran token (CMD25 tail)
                if (w_expected_cmd == 25 && b == 0xFD) {
                    stop_tran_seen = true;
                    stop_pending   = true;
                    wstate = W_BUSY;
                    w_busy_ticks = write_busy_ticks;
                    return;
                }
                // Gap bytes — ignore.
                return;
            case W_RECV_DATA:
                w_block_bytes.push_back(b);
                w_data_count++;
                if (w_data_count == 512) {
                    wstate = W_RECV_CRC;
                    w_crc_count = 0;
                }
                return;
            case W_RECV_CRC:
                if (w_crc_count == 0) w_crc_hi = b;
                else                  w_crc_lo = b;
                w_crc_count++;
                if (w_crc_count == 2) {
                    // Data-block acceptance, the way a real card decides
                    // it.  Three ways a block can be refused:
                    //   * the scenario forced a non-accept response byte
                    //     (write_dr), or
                    //   * the scenario nominated THIS block's serial
                    //     position for rejection, or
                    //   * the CRC16 the host sent does not match the
                    //     bytes actually received.
                    // The last one is what makes a truncated-and-padded
                    // block visible: it must NOT be programmed.  This
                    // model used to capture every block unconditionally,
                    // so a half-real sector looked identical to a good
                    // one and no test could ever see the difference.
                    uint16_t crc_recv =
                        (uint16_t)(((uint16_t)w_crc_hi << 8) | w_crc_lo);
                    uint16_t crc_want = crc16_ccitt(w_block_bytes);
                    uint8_t  dr       = write_dr;
                    bool     accept   = (write_dr == 0xE5);
                    if (w_block_serial == reject_block_serial) {
                        dr     = 0x0B;      // "write error"
                        accept = false;
                    }
                    if (validates_write_crc && crc_recv != crc_want) {
                        dr     = 0x0B;      // "CRC error"
                        accept = false;
                        crc_rejected_blocks++;
                    }
                    w_block_serial++;
                    push(dr);
                    if (accept) {
                        captured_writes[w_lba]     = w_block_bytes;
                        captured_write_crcs[w_lba] = crc_recv;
                        // Advance LBA so the next block of a multi-block
                        // write doesn't overwrite this one.  A REFUSED
                        // block was never programmed, so the card's write
                        // pointer does not move.
                        w_lba++;
                    }
                    w_busy_ticks = write_busy_ticks;
                    wstate = W_BUSY;
                }
                return;
            case W_BUSY:
                // Host is polling with 0xFF; we're holding MISO low.
                // pop_miso() decrements w_busy_ticks and transitions
                // out of W_BUSY on its own.  Nothing to do here.
                return;
            case W_IDLE:
            default:
                break;
        }

        // Normal command frame reassembly.
        if (!in_frame) {
            if ((b & 0xC0) == 0x40) {
                mosi_frame.clear();
                mosi_frame.push_back(b);
                in_frame = true;
            }
        } else {
            mosi_frame.push_back(b);
            if (mosi_frame.size() == 6) {
                handle_cmd_frame();
                in_frame = false;
            }
        }
    }
} sd;

// ─────────────────────────────────────────────────────────────────────
// SPI pin observer — same idiom as tb_sd_boot.cpp.
// ─────────────────────────────────────────────────────────────────────
struct SpiObs {
    uint8_t  mosi_byte       = 0;
    int      rise_count      = 0;
    uint8_t  miso_byte       = 0xFF;
    int      miso_bits_left  = 0;
    bool     last_clk        = false;
    bool     last_cs_n       = true;
    // Total complete bytes clocked out on MOSI since reset.  The
    // watchdog scenarios use this as a "is the transport still moving?"
    // witness: while sd_ctrl is parked in S_TOK_SEND or S_RD_FLUSH_S
    // waiting on rd_ready, NO SPI byte is issued at all, so this count
    // freezes.  Neither park state is directly observable from outside
    // the module, so this plus the rd-byte count is how the two are
    // told apart.
    uint64_t mosi_bytes      = 0;
} sp;

static void spi_tick() {
    bool clk_now  = dut->spi_clk;
    bool cs_n_now = dut->spi_cs_n;

    if (sp.last_cs_n && !cs_n_now) {
        sp.rise_count     = 0;
        sp.mosi_byte      = 0;
        sp.miso_bits_left = 0;
        sp.miso_byte      = 0xFF;
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
            sp.mosi_bytes++;
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
            sp.miso_byte      = (sp.miso_byte << 1) | 1;
            sp.miso_bits_left--;
        }
    } else {
        dut->spi_miso = (sp.miso_byte >> 7) & 1;
    }

    sp.last_clk = clk_now;
}

// ─────────────────────────────────────────────────────────────────────
// Clock + reset + idle drivers
// ─────────────────────────────────────────────────────────────────────
static std::vector<uint8_t> g_rd_bytes;
static std::vector<uint8_t> g_wr_bytes_src;
static size_t               g_wr_src_idx = 0;

// Recorded by test_r1_timeout, consumed by
// test_wdog_base_clears_the_r1_poll_timeout: the MEASURED cost of a full
// POLL_TIMEOUT R1 wait, which is the floor the watchdog base has to
// clear.  Measured rather than re-derived so the check cannot drift away
// from what sd_ctrl.v actually does.
static uint64_t g_r1_timeout_cycles = 0;

// ── Producer-availability (wr_avail) model ───────────────────────────
// Default: the producer always has a byte (g_wr_stall_cycles == 0), which
// is what every pre-existing scenario implicitly asserted.  A scenario can
// arm a stall: once the engine has consumed `g_wr_stall_at` bytes, hold
// wr_avail LOW for `g_wr_stall_cycles` clocks and drive a POISON byte on
// wr_data.  If sd_ctrl honours the credit the poison can never reach the
// card; if it ignores it, the poison lands in the block AND
// g_wr_ready_during_stall counts the illegal consumption.
static size_t g_wr_stall_at          = 0;
static int    g_wr_stall_cycles      = 0;
static int    g_wr_ready_during_stall = 0;
static bool   g_wr_avail_prev        = true;
static constexpr uint8_t WR_POISON   = 0xDE;

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    spi_tick();
    // Capture rd stream
    if (dut->rd_valid) g_rd_bytes.push_back(dut->rd_data);
    // Feed wr stream
    if (g_wr_src_idx < g_wr_bytes_src.size()) {
        dut->wr_data = g_wr_bytes_src[g_wr_src_idx];
    } else {
        dut->wr_data = 0xFF;
    }
    if (dut->wr_ready && g_wr_src_idx < g_wr_bytes_src.size()) {
        g_wr_src_idx++;
    }
    // Contract check: the engine must not have consumed a byte on a
    // posedge where we were holding wr_avail low.
    if (dut->wr_ready && !g_wr_avail_prev) g_wr_ready_during_stall++;
    // Producer-availability model (see the globals above).
    if (g_wr_stall_cycles > 0 && g_wr_src_idx >= g_wr_stall_at) {
        g_wr_stall_cycles--;
        dut->wr_avail = 0;
        dut->wr_data  = WR_POISON;
    } else {
        dut->wr_avail = 1;
    }
    g_wr_avail_prev = dut->wr_avail != 0;
    dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->stall_transport = 0;
    // Default every scenario to the SHIPPING-parameter sd_ctrl copy.
    // Only the watchdog-mechanism scenarios switch to a time-scaled one;
    // see tb_sd_ctrl.v's header for what each copy shortens and why.
    dut->wdog_dut_sel = 0;
    dut->cmd_type     = CT_IDLE;
    dut->lba          = 0;
    dut->block_count  = 0;
    dut->go           = 0;
    dut->rd_ready     = 1;
    dut->wr_data      = 0xFF;
    // Producer always has a byte unless a scenario deliberately says
    // otherwise (test_wr_avail_stalls_the_data_block does).
    dut->wr_avail     = 1;
    dut->cs_n_in      = 0;   // CS asserted throughout the tests
    dut->spi_miso     = 1;
    dut->crc_check_en = 0;   // legacy behaviour unless a scenario opts in
}

static void reset() {
    idle_inputs();
    sp = SpiObs();
    sd = SdCard();
    g_rd_bytes.clear();
    g_wr_bytes_src.clear();
    g_wr_src_idx = 0;
    g_wr_stall_at           = 0;
    g_wr_stall_cycles       = 0;
    g_wr_ready_during_stall = 0;
    g_wr_avail_prev         = true;
    dut->rst = 1;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

// Kick off a command and wait for done
static bool kick_and_wait(uint8_t ct, uint32_t lba, uint16_t bc,
                          uint64_t max_cycles = 3000000) {
    dut->cmd_type    = ct;
    dut->lba         = lba;
    dut->block_count = bc;
    dut->go          = 1;
    tick();
    dut->go          = 0;
    dut->cmd_type    = CT_IDLE;
    uint64_t start = sim_time;
    while (sim_time - start < max_cycles) {
        if (dut->done) { tick(); return true; }
        tick();
    }
    return false;
}

#define CHECK_TRUE(label, cond) do { \
    if (!(cond)) { printf("  FAIL %s\n", (label)); return false; } \
} while(0)

#define CHECK_EQ(label, got, exp) do { \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%x, expected 0x%x\n", (label), _g, _e); \
        return false; \
    } \
} while(0)

// ═════════════════════════════════════════════════════════════════════
// Scenarios
// ═════════════════════════════════════════════════════════════════════

static bool test_cmd17_read() {
    reset();
    const uint32_t LBA = 0x00000042;

    CHECK_TRUE("CMD17 completes", kick_and_wait(CT_CMD17, LBA, 1));
    CHECK_TRUE("no error", !dut->error);
    CHECK_EQ("err_cause=NONE", dut->err_cause, ERR_NONE);
    CHECK_EQ("rd_bytes size", g_rd_bytes.size(), 512u);
    for (int k = 0; k < 512; k++) {
        uint8_t want = (uint8_t)((LBA * 37 + k) & 0xFF);
        if (g_rd_bytes[k] != want) {
            printf("  FAIL byte %d: got 0x%02x, want 0x%02x\n",
                   k, g_rd_bytes[k], want);
            return false;
        }
    }
    return true;
}

static bool test_cmd18_multi_read() {
    reset();
    const uint32_t BASE = 0x00001000;
    const uint16_t N    = 4;

    CHECK_TRUE("CMD18 completes", kick_and_wait(CT_CMD18, BASE, N, 6000000));
    CHECK_TRUE("no error", !dut->error);
    CHECK_EQ("rd_bytes size", g_rd_bytes.size(), (unsigned)(512 * N));
    for (int b = 0; b < N; b++) {
        uint32_t lba = BASE + b;
        for (int k = 0; k < 512; k++) {
            uint8_t want = (uint8_t)((lba * 37 + k) & 0xFF);
            uint8_t got  = g_rd_bytes[b * 512 + k];
            if (got != want) {
                printf("  FAIL block %d byte %d: got 0x%02x want 0x%02x\n",
                       b, k, got, want);
                return false;
            }
        }
    }
    CHECK_TRUE("CMD12 was issued", sd.cmd12_count >= 1);
    // Real-HW bug (2026-07-24): CMD12's frame was left on the old dummy
    // placeholder (argument bytes + CRC7 all 8'h01) when every other
    // command was fixed to send a real CRC7 (this card enforces CRC7
    // unconditionally on every command, not just CMD0/CMD8). The sim SD
    // model doesn't validate CRC7 itself (a known, separately-tracked
    // gap), so this asserts on the actual bytes sent instead of relying
    // on the model to reject a bad frame.
    CHECK_EQ("CMD12 frame captured", sd.last_cmd12_frame.size(), 6u);
    if (sd.last_cmd12_frame.size() == 6) {
        CHECK_EQ("CMD12 cmd byte", sd.last_cmd12_frame[0], 0x4C);
        CHECK_EQ("CMD12 arg byte 0", sd.last_cmd12_frame[1], 0x00);
        CHECK_EQ("CMD12 arg byte 1", sd.last_cmd12_frame[2], 0x00);
        CHECK_EQ("CMD12 arg byte 2", sd.last_cmd12_frame[3], 0x00);
        CHECK_EQ("CMD12 arg byte 3", sd.last_cmd12_frame[4], 0x00);
        CHECK_EQ("CMD12 real CRC7+stop", sd.last_cmd12_frame[5], 0x61);
    }
    return true;
}

// Real-HW bug (2026-07-24, found via fpga_top_sdmin per-attempt
// telemetry): CMD12's mandatory stuff byte is NOT guaranteed to be 0xFF
// on real silicon (every pre-existing scenario's sim SD model hardcoded
// it to 0xFF, which is why this was never caught in sim). sd_ctrl.v's
// generic R1 poll used to treat any non-0xFF, MSB-clear byte as the real
// R1 — so a stuff byte that happened to look like a real (bad) R1 code
// aborted an otherwise fully successful multi-block read. Confirmed on
// real HW: a complete, CRC32-verified 2048-sector transfer still ended
// in err_cause=ERR_R1_BAD (R1=0x04) purely from CMD12's own response.
static bool test_cmd12_nonff_stuff_byte() {
    reset();
    const uint32_t BASE = 0x00001000;
    const uint16_t N    = 4;
    sd.cmd12_stuff_byte = 0x04;   // MSB clear, looks like ILLEGAL_COMMAND

    CHECK_TRUE("CMD18 completes", kick_and_wait(CT_CMD18, BASE, N, 6000000));
    CHECK_TRUE("no error despite non-0xFF CMD12 stuff byte", !dut->error);
    CHECK_EQ("rd_bytes size", g_rd_bytes.size(), (unsigned)(512 * N));
    CHECK_TRUE("CMD12 was issued", sd.cmd12_count >= 1);
    return true;
}

static bool test_cmd24_write() {
    reset();
    const uint32_t LBA = 0x00000099;
    g_wr_bytes_src.resize(512);
    for (int k = 0; k < 512; k++) g_wr_bytes_src[k] = (uint8_t)(0x5A ^ k);

    CHECK_TRUE("CMD24 completes", kick_and_wait(CT_CMD24, LBA, 1));
    CHECK_TRUE("no error", !dut->error);
    auto it = sd.captured_writes.find(LBA);
    CHECK_TRUE("SD captured write @ LBA", it != sd.captured_writes.end());
    CHECK_EQ("captured length", it->second.size(), 512u);
    for (int k = 0; k < 512; k++) {
        uint8_t want = (uint8_t)(0x5A ^ k);
        if (it->second[k] != want) {
            printf("  FAIL byte %d: got 0x%02x want 0x%02x\n",
                   k, it->second[k], want);
            return false;
        }
    }
    return true;
}

static bool test_cmd25_multi_write() {
    reset();
    const uint32_t BASE = 0x00002000;
    const uint16_t N    = 4;
    g_wr_bytes_src.resize(512 * N);
    for (int b = 0; b < N; b++) {
        for (int k = 0; k < 512; k++) {
            g_wr_bytes_src[b * 512 + k] = (uint8_t)((b * 0x11 + k) & 0xFF);
        }
    }

    CHECK_TRUE("CMD25 completes", kick_and_wait(CT_CMD25, BASE, N, 6000000));
    CHECK_TRUE("no error", !dut->error);
    for (int b = 0; b < N; b++) {
        uint32_t lba = BASE + b;
        auto it = sd.captured_writes.find(lba);
        if (it == sd.captured_writes.end()) {
            printf("  FAIL block %u not captured\n", b);
            return false;
        }
        for (int k = 0; k < 512; k++) {
            uint8_t want = (uint8_t)((b * 0x11 + k) & 0xFF);
            if (it->second[k] != want) {
                printf("  FAIL block %d byte %d: got 0x%02x want 0x%02x\n",
                       b, k, it->second[k], want);
                return false;
            }
        }
    }
    CHECK_TRUE("stop-tran 0xFD was sent", sd.stop_tran_seen);
    return true;
}

// A completed CMD25 must be fully closed before done, because the next owner
// may issue CMD24 immediately.  Exercise both commands without resetting the
// controller or card, across legal zero-to-long card busy intervals.
static bool test_cmd25_then_cmd24_without_reset() {
    static const int busy_intervals[] = {0, 1, 4, 63};

    for (int busy_ticks : busy_intervals) {
        reset();
        sd.write_busy_ticks = busy_ticks;

        const uint32_t multi_lba = 0x00002400 + (uint32_t)busy_ticks * 8;
        const uint32_t single_lba = multi_lba + 5;
        g_wr_bytes_src.resize(2 * 512);
        for (int k = 0; k < 2 * 512; k++)
            g_wr_bytes_src[k] = (uint8_t)(0x37 ^ k ^ busy_ticks);

        CHECK_TRUE("back-to-back CMD25 completes",
                   kick_and_wait(CT_CMD25, multi_lba, 2, 6000000));
        CHECK_TRUE("CMD25 reports no error", !dut->error);
        CHECK_TRUE("CMD25 closed the card before done",
                   !sd.write_session_open());

        g_wr_bytes_src.resize(512);
        for (int k = 0; k < 512; k++)
            g_wr_bytes_src[k] = (uint8_t)(0xC1 ^ (k * 7) ^ busy_ticks);
        g_wr_src_idx = 0;

        CHECK_TRUE("immediate CMD24 completes",
                   kick_and_wait(CT_CMD24, single_lba, 1, 3000000));
        CHECK_TRUE("immediate CMD24 reports no error", !dut->error);
        CHECK_TRUE("CMD24 closed the card before done",
                   !sd.write_session_open());
        CHECK_TRUE("immediate CMD24 reached its requested LBA",
                   sd.captured_writes.count(single_lba) == 1);
        for (int k = 0; k < 512; k++) {
            uint8_t want = (uint8_t)(0xC1 ^ (k * 7) ^ busy_ticks);
            if (sd.captured_writes[single_lba][k] != want) {
                printf("  FAIL busy=%d CMD24 byte %d: got 0x%02x want 0x%02x\n",
                       busy_ticks, k, sd.captured_writes[single_lba][k], want);
                return false;
            }
        }
    }
    return true;
}

// A SoC reset used to reset sd_ctrl and sd_spi immediately, even if the card
// was part-way through CMD25.  The card itself is not reset by an FPGA reset;
// it remains in receive mode and treats boot/provisioning traffic as the rest
// of the disk block.  Keep the software card alive across the reset pulse and
// prove the controller rejects the torn block, closes CMD25, then accepts a
// fresh command at exactly the requested LBA.
static bool test_reset_mid_cmd25_closes_card_before_restart() {
    reset();
    const uint32_t HDD_LBA  = 0x00002800;
    const uint32_t NEXT_LBA = 0x00002A00;
    const int STALL_AT = 137;

    for (uint32_t lba = 0; lba < 8; lba++) {
        std::vector<uint8_t> sentinel(512);
        for (int k = 0; k < 512; k++)
            sentinel[k] = (uint8_t)(0xA5 ^ lba ^ k);
        sd.captured_writes[lba] = sentinel;
    }
    const std::map<uint32_t, std::vector<uint8_t>> rom_before =
        sd.captured_writes;

    g_wr_bytes_src.resize(4 * 512);
    for (size_t k = 0; k < g_wr_bytes_src.size(); k++)
        g_wr_bytes_src[k] = (uint8_t)(0x19 ^ (k * 11));
    g_wr_stall_at     = STALL_AT;
    g_wr_stall_cycles = 100000000;
    dut->cmd_type    = CT_CMD25;
    dut->lba         = HDD_LBA;
    dut->block_count = 4;
    dut->go          = 1;
    tick();
    dut->go          = 0;
    dut->cmd_type    = CT_IDLE;

    uint64_t guard = 3000000;
    while ((g_wr_src_idx < (size_t)STALL_AT || !sd.write_session_open()) &&
           guard-- > 0)
        tick();
    CHECK_TRUE("CMD25 reached a live partial data block before reset",
               guard > 0 && sd.write_session_open());
    CHECK_TRUE("controller is busy at reset", dut->busy != 0);

    // This resets the FPGA logic only.  Deliberately do not call reset(),
    // because that would also reset the card model and make the test lie.
    dut->rst = 1;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;

    guard = 3000000;
    while (dut->busy && guard-- > 0) tick();
    CHECK_TRUE("deferred reset close terminates", guard > 0);
    CHECK_TRUE("card is closed before restart", !sd.write_session_open());
    CHECK_TRUE("torn CMD25 block was rejected",
               sd.captured_writes.count(HDD_LBA) == 0);

    // busy falls one cycle before reset_close_pending drives local_rst.
    // Let that deferred reset edge and its idle release complete before a
    // new owner presents work (the SoC's boot/CPU release has a much larger
    // settling interval than this).
    tick();
    tick();

    g_wr_stall_at     = 0;
    g_wr_stall_cycles = 0;
    dut->wr_avail     = 1;
    g_wr_bytes_src.resize(512);
    for (int k = 0; k < 512; k++)
        g_wr_bytes_src[k] = (uint8_t)(0xD3 ^ (k * 5));
    g_wr_src_idx = 0;

    CHECK_TRUE("post-reset CMD24 completes",
               kick_and_wait(CT_CMD24, NEXT_LBA, 1, 3000000));
    CHECK_TRUE("post-reset CMD24 reports no error", !dut->error);
    CHECK_TRUE("post-reset CMD24 reached only its requested LBA",
               sd.captured_writes.count(NEXT_LBA) == 1);
    for (const auto& [lba, expected] : rom_before) {
        CHECK_TRUE("reset/restart traffic did not modify reserved sectors",
                   sd.captured_writes[lba] == expected);
    }
    return true;
}

struct WriteResetPoint {
    const char* name;
    uint8_t state;
    int byte_state;
    int byte_idx;
    int byte_done;
    // -1: any response, 0: response byte is zero, 1: response is nonzero.
    int rsp_nonzero;
};

static bool at_write_reset_point(const WriteResetPoint& p) {
    if (dut->dbg_state != p.state) return false;
    if (p.byte_state >= 0 && dut->dbg_byte_state != p.byte_state) return false;
    if (p.byte_idx >= 0 && dut->dbg_byte_idx != p.byte_idx) return false;
    if (p.byte_done >= 0 && dut->dbg_byte_done != p.byte_done) return false;
    if (p.rsp_nonzero >= 0 &&
        ((dut->dbg_byte_rsp != 0) != (p.rsp_nonzero != 0)))
        return false;
    return true;
}

// Reset can arrive on every core clock, while the SPI byte engine exposes
// three materially different phases: request pending, response pending, and
// the one-cycle completion pulse.  Exercise every normal CMD25 write state,
// every byte phase, all data/CRC boundaries, and both sides of busy release.
// The card model remains alive across reset and validates CRC, so a duplicated
// or omitted byte either commits a detectably wrong sector or strands the card.
static bool test_reset_at_every_cmd25_write_phase() {
    std::vector<WriteResetPoint> points;
    auto add_s = [&](const char* name, uint8_t state, int idx = -1) {
        points.push_back({name, state, BS_IDLE, idx, -1, -1});
    };
    auto add_w = [&](const char* name, uint8_t state, int idx = -1) {
        points.push_back({name, state, BS_SEND, idx, 0, -1});
        points.push_back({name, state, BS_WAIT, idx, 0, -1});
        points.push_back({name, state, BS_IDLE, idx, 1, -1});
    };

#ifdef SDCTRL_WRITE_STAGE
    add_s("command-pre-s", 29);
    add_w("command-pre-w", 30);
    add_s("command-send", 1);
    add_w("command-wait", 2);
    add_s("r1-send", 3);
    add_w("r1-wait", 4);
#endif
    add_s("gap-s", S_W_GAP_S);
    add_w("gap-w", S_W_GAP_W);
    add_s("token-s", S_W_TOK_S);
    add_w("token-w", S_W_TOK_W);

    static const int data_indices[] = {0, 1, 255, 510, 511};
    for (int idx : data_indices) {
        add_s("data-s", S_W_DATA_S, idx);
        add_w("data-w", S_W_DATA_W, idx);
    }
    for (int idx = 0; idx < 2; idx++) {
        add_s("crc-s", S_W_DCRC_S, idx);
        add_w("crc-w", S_W_DCRC_W, idx);
    }

    add_s("response-s", S_W_RESP_S);
    add_w("response-w", S_W_RESP_W);
    add_s("busy-s", S_W_BUSY_S);
    points.push_back({"busy-w-send", S_W_BUSY_W, BS_SEND, -1, 0, -1});
    points.push_back({"busy-w-wait", S_W_BUSY_W, BS_WAIT, -1, 0, -1});
    points.push_back({"busy-w-low", S_W_BUSY_W, BS_IDLE, -1, 1, 0});
    points.push_back({"busy-w-release", S_W_BUSY_W, BS_IDLE, -1, 1, 1});
    add_s("stop-s", S_W_STOP_S);
    add_w("stop-w", S_W_STOP_W);
    add_s("final-busy-s", S_W_FBUSY_S);
    points.push_back({"final-busy-w-send", S_W_FBUSY_W, BS_SEND, -1, 0, -1});
    points.push_back({"final-busy-w-wait", S_W_FBUSY_W, BS_WAIT, -1, 0, -1});
    points.push_back({"final-busy-w-low", S_W_FBUSY_W, BS_IDLE, -1, 1, 0});
    points.push_back({"final-busy-w-release", S_W_FBUSY_W, BS_IDLE, -1, 1, 1});

    size_t covered = 0;
#ifdef SDCTRL_WRITE_STAGE
    const std::vector<uint8_t> commands = {CT_CMD24, CT_CMD25};
#else
    const std::vector<uint8_t> commands = {CT_CMD25};
#endif
    for (uint8_t command : commands) {
    for (size_t case_idx = 0; case_idx < points.size(); case_idx++) {
        const WriteResetPoint& p = points[case_idx];
        if (command == CT_CMD24 && p.state >= S_W_STOP_S && p.state <= S_W_FBUSY_W) continue;
        reset();
#ifdef SDCTRL_WRITE_STAGE
        // Staging must finish a genuine sector even when the card does not
        // enforce CRC. A padded/bad-CRC abort would corrupt this sentinel.
        sd.validates_write_crc = false;
#endif
        sd.write_busy_ticks = 4;
        const uint32_t hdd_lba = 0x00004000 + (uint32_t)case_idx * 2;
        const uint32_t pram_lba = 8191;

        std::map<uint32_t, std::vector<uint8_t>> canaries;
        for (uint32_t lba = 0; lba < 8; lba++) {
            std::vector<uint8_t> sector(512);
            for (int k = 0; k < 512; k++)
                sector[k] = (uint8_t)(0xA7 ^ lba ^ (uint32_t)(k * 13));
            sd.captured_writes[lba] = sector;
            canaries[lba] = sector;
        }

        std::vector<uint8_t> hdd_payload(512);
        for (int k = 0; k < 512; k++)
            hdd_payload[k] = (uint8_t)(0x39 ^ case_idx ^ (size_t)(k * 17));
        g_wr_bytes_src = hdd_payload;
        g_wr_src_idx = 0;
        dut->cmd_type = command;
        dut->lba = hdd_lba;
        dut->block_count = 1;
        dut->go = 1;
        tick();
        dut->go = 0;
        dut->cmd_type = CT_IDLE;

        uint64_t guard = 4000000;
        while (!at_write_reset_point(p) && guard-- > 0) tick();
        if (guard == 0) {
            printf("  FAIL reset point %zu (%s) was not reached\n",
                   case_idx, p.name);
            return false;
        }

        // Reset the FPGA controller only.  Eight clocks is intentionally
        // shorter than an SPI byte, so SEND/WAIT cases prove the pending byte
        // is retired after rst deasserts via the latched close request.
        dut->rst = 1;
#ifdef SDCTRL_WRITE_STAGE
        g_wr_bytes_src.assign(512, 0xDE); // reset destroys upstream data
        g_wr_src_idx = 0;
#endif
        for (int i = 0; i < 8; i++) tick();
        dut->rst = 0;

        guard = 4000000;
        while (dut->busy && guard-- > 0) tick();
        if (guard == 0 || sd.write_session_open()) {
            printf("  FAIL reset point %zu (%s) did not close CMD25\n",
                   case_idx, p.name);
            return false;
        }
        tick();
        tick();

        auto hdd = sd.captured_writes.find(hdd_lba);
        if (hdd != sd.captured_writes.end() && hdd->second != hdd_payload) {
            printf("  FAIL reset point %zu (%s) committed a torn HDD sector\n",
                   case_idx, p.name);
            return false;
        }

        std::vector<uint8_t> pram_payload(512);
        for (int k = 0; k < 512; k++)
            pram_payload[k] = (uint8_t)(0xD5 ^ case_idx ^ (size_t)(k * 5));
        g_wr_bytes_src = pram_payload;
        g_wr_src_idx = 0;
        if (!kick_and_wait(CT_CMD24, pram_lba, 1, 3000000) || dut->error ||
            sd.write_session_open() ||
            sd.captured_writes[pram_lba] != pram_payload) {
            printf("  FAIL reset point %zu (%s) broke following PRAM CMD24\n",
                   case_idx, p.name);
            return false;
        }

        for (const auto& [lba, data] : sd.captured_writes) {
            bool allowed = lba < 8 || lba == hdd_lba || lba == pram_lba;
            if (!allowed) {
                printf("  FAIL reset point %zu (%s) wrote unexpected LBA %u\n",
                       case_idx, p.name, lba);
                return false;
            }
        }
        for (const auto& [lba, data] : canaries) {
            if (sd.captured_writes[lba] != data) {
                printf("  FAIL reset point %zu (%s) changed canary LBA %u\n",
                       case_idx, p.name, lba);
                return false;
            }
        }
        ++covered;
    }
    }

    printf("       (%zu reset landing points covered)\n", covered);
    return true;
}

// ── wr_avail: REAL write-side back-pressure ──────────────────────────
// Before 2026-08-08 S_W_DATA_S latched wr_data and issued the SPI byte
// unconditionally whenever the byte engine was idle, so a producer that
// stalled mid-block had the drain run ahead of the fill and the card got
// whatever happened to be on wr_data.  This tb could not see that: its
// producer is a C++ array that is ALWAYS ready, i.e. the one shape in
// which the bug is invisible.  So the model above can now stall, and it
// drives a POISON byte while stalled — a value that is in the payload
// nowhere, so a single sampled byte during the stall is a hard failure.
static bool test_wr_avail_stalls_the_data_block() {
    reset();
    const uint32_t LBA = 0x000000C7;
    g_wr_bytes_src.resize(512);
    for (int k = 0; k < 512; k++) g_wr_bytes_src[k] = (uint8_t)(0x31 + k * 7);

    // Stall after 200 bytes for 20,000 clocks — many SPI byte-times
    // (FAST_HALF=4 → ~64 clocks/byte) and far inside the shipping
    // watchdog budget (~1e9 clocks), so a correct engine simply waits.
    g_wr_stall_at     = 200;
    g_wr_stall_cycles = 20000;

    CHECK_TRUE("CMD24 with a stalled producer completes",
               kick_and_wait(CT_CMD24, LBA, 1, 3000000));
    CHECK_TRUE("no error (the stall must not trip the watchdog)",
               !dut->error);
    CHECK_EQ("engine never consumed a byte while wr_avail was low",
             g_wr_ready_during_stall, 0);
    CHECK_TRUE("the stall really was exercised", g_wr_stall_cycles == 0);

    auto it = sd.captured_writes.find(LBA);
    CHECK_TRUE("SD captured write @ LBA", it != sd.captured_writes.end());
    CHECK_EQ("captured length", it->second.size(), 512u);
    for (int k = 0; k < 512; k++) {
        uint8_t want = (uint8_t)(0x31 + k * 7);
        if (it->second[k] != want) {
            printf("  FAIL byte %d: got 0x%02x want 0x%02x%s\n",
                   k, it->second[k], want,
                   it->second[k] == WR_POISON
                       ? "  <-- POISON: engine sampled wr_data while "
                         "wr_avail was low" : "");
            return false;
        }
    }
    return true;
}

// Same, on the MULTI-block path, with the stall placed exactly on a
// block boundary — the byte where scsi.v's ring changes blocks and where
// a fencepost in either back-pressure direction shows up first.
static bool test_wr_avail_stall_at_block_boundary() {
    reset();
    const uint32_t BASE = 0x00003000;
    const uint16_t N    = 3;
    g_wr_bytes_src.resize(512 * N);
    for (size_t k = 0; k < g_wr_bytes_src.size(); k++)
        g_wr_bytes_src[k] = (uint8_t)((k * 5) ^ 0x77);

    g_wr_stall_at     = 512;      // first byte of block 1
    g_wr_stall_cycles = 20000;

    CHECK_TRUE("CMD25 with a boundary stall completes",
               kick_and_wait(CT_CMD25, BASE, N, 6000000));
    CHECK_TRUE("no error", !dut->error);
    CHECK_EQ("engine never consumed a byte while wr_avail was low",
             g_wr_ready_during_stall, 0);
    CHECK_TRUE("the stall really was exercised", g_wr_stall_cycles == 0);
    for (int b = 0; b < N; b++) {
        auto it = sd.captured_writes.find(BASE + b);
        if (it == sd.captured_writes.end()) {
            printf("  FAIL block %d not captured\n", b);
            return false;
        }
        for (int k = 0; k < 512; k++) {
            uint8_t want = (uint8_t)(((b * 512 + k) * 5) ^ 0x77);
            if (it->second[k] != want) {
                printf("  FAIL block %d byte %d: got 0x%02x want 0x%02x%s\n",
                       b, k, it->second[k], want,
                       it->second[k] == WR_POISON ? "  <-- POISON" : "");
                return false;
            }
        }
    }
    CHECK_TRUE("stop-tran 0xFD was sent", sd.stop_tran_seen);
    return true;
}

// ── Per-block state really does reset between blocks of one CMD25 ────
// A multi-block write reuses one FSM for N blocks: byte_idx, wr_valid and
// wr_crc_calc all have to be re-initialised at each 0xFC data token
// (S_W_TOK_W in sd_ctrl.v), not just once at the start of the command.
// This is invisible at N=1 by construction — which matters, because on
// hardware 32 of the 7.5.3 driver's 41 writes ARE single-block and work
// fine, so single-block coverage proves nothing about the multi-block
// path.  Give every block a different payload and check the CRC16 the
// card actually received for EACH one: a leaked accumulator or a
// non-reset byte counter shows up as a wrong CRC on block 1 onwards even
// when every data byte is correct.
static bool test_cmd25_per_block_state_resets() {
    reset();
    const uint32_t BASE = 0x00004000;
    const uint16_t N    = 5;
    g_wr_bytes_src.resize(512 * N);
    std::vector<std::vector<uint8_t>> blocks(N, std::vector<uint8_t>(512));
    for (int b = 0; b < N; b++) {
        for (int k = 0; k < 512; k++) {
            // Deliberately distinct per block, and NOT a function of the
            // absolute offset, so a stale accumulator cannot coincide.
            uint8_t v = (uint8_t)((0xC3 * (b + 1)) ^ (k * 3) ^ (k >> 5));
            blocks[b][k] = v;
            g_wr_bytes_src[b * 512 + k] = v;
        }
    }

    CHECK_TRUE("CMD25 completes", kick_and_wait(CT_CMD25, BASE, N, 6000000));
    CHECK_TRUE("no error", !dut->error);
    CHECK_TRUE("stop-tran 0xFD was sent", sd.stop_tran_seen);
    for (int b = 0; b < N; b++) {
        uint32_t lba = BASE + b;
        auto it = sd.captured_writes.find(lba);
        if (it == sd.captured_writes.end()) {
            printf("  FAIL block %d not captured\n", b);
            return false;
        }
        if (it->second.size() != 512u) {
            printf("  FAIL block %d captured %zu bytes, want 512 "
                   "(byte counter did not reset)\n", b, it->second.size());
            return false;
        }
        for (int k = 0; k < 512; k++) {
            if (it->second[k] != blocks[b][k]) {
                printf("  FAIL block %d byte %d: got 0x%02x want 0x%02x\n",
                       b, k, it->second[k], blocks[b][k]);
                return false;
            }
        }
        auto ic = sd.captured_write_crcs.find(lba);
        if (ic == sd.captured_write_crcs.end()) {
            printf("  FAIL block %d CRC not captured\n", b);
            return false;
        }
        uint16_t want = crc16_ccitt(blocks[b]);
        if (ic->second != want) {
            printf("  FAIL block %d CRC16: got 0x%04x want 0x%04x "
                   "(wr_crc_calc did not reset at the 0xFC token)\n",
                   b, ic->second, want);
            return false;
        }
    }
    return true;
}

static bool test_r1_bad() {
    reset();
    sd.cmd17_r1 = 0x04;   // illegal cmd

    CHECK_TRUE("cmd completes", kick_and_wait(CT_CMD17, 0, 1));
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=R1_BAD", dut->err_cause, ERR_R1_BAD);
    return true;
}

static bool test_r1_timeout() {
    reset();
    sd.silent = true;   // no MISO response at all

    // Wait budget: POLL_TIMEOUT (sd_ctrl.v) bumped 8192->1000000 bytes
    // (2026-07-15, HW bring-up -- see that file's timeout comment).
    // Empirically needs comfortably under 300M sim cycles at this tb's
    // SPI timing to actually hit the timeout and unwind through
    // S_ERROR; budgeted generously above the observed cost.
    uint64_t t0 = sim_time;
    CHECK_TRUE("cmd completes", kick_and_wait(CT_CMD17, 0, 1, 300000000));
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=R1_TO", dut->err_cause, ERR_R1_TO);
    // Hand the measured cost to
    // test_wdog_base_clears_the_r1_poll_timeout — this scenario reaching
    // ERR_R1_TO at all is itself proof the watchdog did not preempt it.
    g_r1_timeout_cycles = sim_time - t0;
    return true;
}

static bool test_dr_bad() {
    reset();
    sd.write_dr = 0x0B;   // 0bxxxx1011 = CRC error, non-accepted response
    g_wr_bytes_src.resize(512, 0xAA);

    CHECK_TRUE("cmd completes", kick_and_wait(CT_CMD24, 0, 1));
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=DR_BAD", dut->err_cause, ERR_DR_BAD);
    CHECK_EQ("write response diagnostic preserves CRC reject",
             dut->dbg_last_write_resp, 0x0B);
    CHECK_EQ("single-block reject reports block zero", dut->dbg_block_idx, 0);
    return true;
}

static bool test_unknown_cmd() {
    reset();
    // 5 is outside CT_CMD17..CMD25 (max 4).
    CHECK_TRUE("cmd completes fast", kick_and_wait(5, 0, 1, 1000));
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=UNK_CMD", dut->err_cause, ERR_UNK_CMD);
    return true;
}

// ═════════════════════════════════════════════════════════════════════
// GLOBAL PER-REQUEST WATCHDOG scenarios
// ═════════════════════════════════════════════════════════════════════
// These exist because of a blind spot this file created: idle_inputs()
// sets rd_ready=1 and no scenario ever lowered it, so the ONLY unit tb
// covering sd_ctrl could not reach either of the module's two
// rd_ready-gated pause states (S_TOK_SEND, S_RD_FLUSH_S) — the exact
// states in which it used to park with `busy` high forever, deadlocking
// the SCSI target and through it the CPU.  A test that pins high the
// signal whose LOW value is the bug is not coverage.
//
// Two of the three scenarios below stall the consumer for real and
// prove the watchdog answers; the third proves it does NOT answer for a
// consumer that is merely slow, which is the failure mode that would
// turn a hang into spurious disk errors.

// Mirrors of sd_ctrl.v's REQ_WDOG_* parameters.  Kept as plain
// constants (not read out of the DUT) on purpose: if someone changes
// the RTL constant without thinking about the budget, these scenarios
// should go red and make them think about it.
//
// HOW EACH ONE IS PINNED TO THE RTL — all three are, two-sidedly, and
// none of it costs a billion cycles:
//
//   WDOG_BASE_TICKS / WDOG_BLK_TICKS
//       The two park scenarios run on DUT_FAST_TICK, which is the SAME
//       base and per-block shift as the shipping part with only the
//       prescaler shortened (12 -> 3).  So their two-sided fire-time
//       check measures (BASE + BLK) directly, in shipping units, at
//       1/512th the wall clock.  Change REQ_WDOG_BASE_TICKS in the RTL
//       without changing the mirror here and they go red.
//
//   WDOG_TICK_CYCLES
//       Pinned by test_wdog_prescaler_is_the_shipped_tick, which runs on
//       DUT_TINY_BASE: shipping prescaler, base collapsed to 1 tick, so
//       expiry lands at (1 + BLK) * TICK_CYCLES = 135,168 cycles.
//
// Only after that do the arithmetic-only scenarios (boot-sized budget,
// R1-poll headroom) get to trust these numbers.
static const uint64_t WDOG_TICK_CYCLES = 4096;    // 1 << REQ_WDOG_TICK_LOG2
static const uint64_t WDOG_BASE_TICKS  = 245760;  // REQ_WDOG_BASE_TICKS
static const uint64_t WDOG_BLK_TICKS   = 32;      // 1 << REQ_WDOG_BLK_SHIFT

// wdog_dut_sel encodings — must match tb_sd_ctrl.v's SEL_* localparams.
static const uint8_t DUT_SHIP      = 0;   // shipping parameters
static const uint8_t DUT_FAST_TICK = 1;   // REQ_WDOG_TICK_LOG2 = 3
static const uint8_t DUT_TINY_BASE = 2;   // REQ_WDOG_BASE_TICKS = 1
static const uint8_t DUT_WDOG_OFF  = 3;   // TINY_BASE + REQ_WDOG_ENABLE = 0
// ...and the overrides those two copies carry.
static const uint64_t FAST_TICK_CYCLES = 8;   // 1 << 3
static const uint64_t TINY_BASE_TICKS  = 1;

// Budget in TICKS — identical for u_ship and u_fast_tick, which is the
// whole point of scaling the prescaler rather than the base.
static uint64_t wdog_limit_ticks(uint16_t blocks) {
    return WDOG_BASE_TICKS + (uint64_t)blocks * WDOG_BLK_TICKS;
}

// Budget in CYCLES for the shipping instance.
static uint64_t wdog_limit_cycles(uint16_t blocks) {
    return wdog_limit_ticks(blocks) * WDOG_TICK_CYCLES;
}

// Budget in CYCLES for the short-prescaler instance.
static uint64_t wdog_limit_cycles_fast(uint16_t blocks) {
    return wdog_limit_ticks(blocks) * FAST_TICK_CYCLES;
}

// Pulse `go` without waiting — the watchdog scenarios need to drive
// rd_ready themselves while the request is in flight.
static void kick(uint8_t ct, uint32_t lba, uint16_t bc) {
    dut->cmd_type    = ct;
    dut->lba         = lba;
    dut->block_count = bc;
    dut->go          = 1;
    tick();
    dut->go          = 0;
    dut->cmd_type    = CT_IDLE;
}

// ═════════════════════════════════════════════════════════════════════
// ABANDONED WRITE SESSION  (the PRAM-clobbers-the-HDD bug, 2026-08-09)
// ═════════════════════════════════════════════════════════════════════
//
// Reported symptoms, both at once:
//   (a) saving PRAM (a single CMD24 at SD LBA 8191) clobbers the HDD
//       image, which starts at the very next sector, 8192;
//   (b) PRAM never persists — a later pram-load at 8191 finds nothing.
//
// One mechanism produces both.  A CMD24/CMD25 puts the CARD into a
// receive state that only the host can leave: it counts exactly 512
// data bytes + 2 CRC bytes per block, and a CMD25 stays in sequential-
// write mode until it sees the 0xFD stop-tran token.  Deasserting CS#
// ends neither.  Every error exit on sd_ctrl's write path used to jump
// straight to S_ERROR, which drops `busy` and pulses `done` with the
// card still open.
//
// fpga_top_sd.vh raises pram_gnt on any cycle where sd_ctrl_scsi is not
// busy — and that condition is now satisfied.  pram_sd's CMD24 frame,
// its R1 poll bytes and its 512-byte payload are then consumed by the
// card as WRITE DATA and programmed at the CARD's own running write
// pointer, which is inside the HDD image.  Sector 8191 is never touched.
//
// Two scenarios: the first pins the protocol obligation, the second
// measures the damage end-to-end.

// A — protocol: an aborted CMD25 must still close the card.
static bool test_cmd25_abort_closes_the_card() {
    reset();
    const uint32_t BASE = 0x00002200;
    const uint16_t N    = 4;
    g_wr_bytes_src.resize(512 * N);
    for (size_t k = 0; k < g_wr_bytes_src.size(); k++)
        g_wr_bytes_src[k] = (uint8_t)((k * 3) ^ 0x2B);

    // The card refuses block 1 (a "write error" data-response).  The
    // transport is demonstrably alive — the card just answered — so
    // there is no excuse for walking away with the session open.
    sd.reject_block_serial = 1;

    CHECK_TRUE("CMD25 completes", kick_and_wait(CT_CMD25, BASE, N, 6000000));
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=DR_BAD (the ORIGINAL cause, not the close's)",
             dut->err_cause, ERR_DR_BAD);
    CHECK_TRUE("block 0 was programmed",
               sd.captured_writes.count(BASE) == 1);
    CHECK_TRUE("the refused block was NOT programmed",
               sd.captured_writes.count(BASE + 1) == 0);
    // The two assertions this scenario exists for.
    CHECK_TRUE("stop-tran 0xFD was sent on the abort path",
               sd.stop_tran_seen);
    CHECK_TRUE("card is back in command-wait, not sequential-write",
               !sd.write_session_open());
    return true;
}

// B — damage: abandon a CMD25 MID-DATA-BLOCK (a producer stall that
// runs into the watchdog, exactly the shape sd_ctrl.v's own header
// describes as "an interrupt pre-empting the Mac's pseudo-DMA loop"),
// then do what the operator does next: save PRAM at LBA 8191.  The only
// sector allowed to change is 8191.
//
// DUT_TINY_BASE is used purely to make the watchdog fire in ~135k cycles
// instead of ~1e9; the abandonment being modelled is parameter-
// independent.
static bool test_cmd25_abort_then_pram_save_hits_only_8191() {
    reset();
    dut->wdog_dut_sel = DUT_TINY_BASE;

    // Match the shipping SD layout: HDD image at 8192+, PRAM at 8191.
    const uint32_t HDD_BASE = 8192;
    const uint32_t PRAM_LBA = 8191;
    const uint16_t N        = 4;

    g_wr_bytes_src.resize(512 * N);
    for (size_t k = 0; k < g_wr_bytes_src.size(); k++)
        g_wr_bytes_src[k] = (uint8_t)((k * 11) ^ 0x40);

    // Producer dies 100 bytes into block 0 and never comes back, so the
    // engine parks in S_W_DATA_S with the card mid-block.
    g_wr_stall_at     = 100;
    g_wr_stall_cycles = 100000000;

    CHECK_TRUE("the abandoned CMD25 terminates",
               kick_and_wait(CT_CMD25, HDD_BASE, N, 4000000));
    CHECK_TRUE("it terminated as an error", dut->error != 0);
    CHECK_EQ("err_cause=WDOG", dut->err_cause, ERR_WDOG);
    CHECK_EQ("engine never consumed a byte while wr_avail was low",
             g_wr_ready_during_stall, 0);
    // Deliberately recorded, not asserted, HERE: the damage assertion
    // below is the point of the scenario, and bailing out early on the
    // mechanism check would hide which sector actually got clobbered.
    const bool card_left_open = sd.write_session_open();

    // Snapshot the disk, then do the PRAM save.  crc_rejected_blocks is
    // snapshotted too: the close sequence deliberately sends an invalid
    // CRC for its padded block (so the card refuses a half-real sector),
    // and that legitimate rejection must not be mistaken for the card
    // eating the PRAM save's traffic.  Only the DELTA across the save
    // is evidence.
    std::map<uint32_t, std::vector<uint8_t> > before = sd.captured_writes;
    const int crc_rejects_before = sd.crc_rejected_blocks;

    g_wr_stall_at     = 0;
    g_wr_stall_cycles = 0;
    dut->wr_avail     = 1;
    g_wr_bytes_src.assign(512, 0);
    for (int k = 0; k < 512; k++)
        g_wr_bytes_src[k] = (uint8_t)(0x91 ^ (k * 13));
    g_wr_src_idx = 0;

    const bool pram_save_terminated =
        kick_and_wait(CT_CMD24, PRAM_LBA, 1, 4000000);

    // THE assertion.  Anything new at 8192+ is the HDD image being
    // overwritten with PRAM bytes at the card's own write pointer.
    for (std::map<uint32_t, std::vector<uint8_t> >::const_iterator it =
             sd.captured_writes.begin();
         it != sd.captured_writes.end(); ++it) {
        if (it->first == PRAM_LBA) continue;
        std::map<uint32_t, std::vector<uint8_t> >::const_iterator b =
            before.find(it->first);
        if (b == before.end() || b->second != it->second) {
            printf("  FAIL sector %u was modified by a PRAM save that "
                   "should only have touched %u  <-- the HDD image, "
                   "written at the CARD's own pointer because the "
                   "abandoned CMD25 was never closed (card_left_open=%d)\n",
                   it->first, PRAM_LBA, (int)card_left_open);
            return false;
        }
    }
    CHECK_TRUE("card was closed after the abandoned write", !card_left_open);
    CHECK_TRUE("the truncated block was NOT programmed",
               before.count(HDD_BASE) == 0);
    // The mechanism, measured: a non-zero count here means the card
    // consumed the PRAM save's command frame / poll bytes / payload as
    // WRITE DATA for the still-open block, then refused the resulting
    // garbage sector.  It is only refused because this card validates
    // CRC16 (boot_fsm enables CMD59); the bytes still went to the card
    // as a write, at an address the host never asked for.
    CHECK_EQ("the card ate no part of the PRAM save as write data",
             sd.crc_rejected_blocks - crc_rejects_before, 0);
    CHECK_TRUE("the PRAM save terminated", pram_save_terminated);
    CHECK_TRUE("the PRAM save did not error", !dut->error);
    CHECK_TRUE("PRAM really landed at 8191",
               sd.captured_writes.count(PRAM_LBA) == 1);
    for (int k = 0; k < 512; k++) {
        uint8_t want = (uint8_t)(0x91 ^ (k * 13));
        if (sd.captured_writes[PRAM_LBA][k] != want) {
            printf("  FAIL PRAM byte %d: got 0x%02x want 0x%02x\n",
                   k, sd.captured_writes[PRAM_LBA][k], want);
            return false;
        }
    }
    return true;
}

// C — symptom (a): the HDD image written with the NEXT master's traffic.
// Same abandonment as B, but on a card session where the data CRC16 is
// not being validated.  CMD59/CRC_ON_OFF is a whole-session setting the
// host chooses (boot_fsm.v sends it, and there is a documented real-HW
// history of this card being awkward about CRC modes), so BOTH states
// are real configurations of this platform.  With validation off the
// card COMMITS the 512 bytes it counted — so the PRAM save's command
// frame and R1 poll bytes are programmed into the disk image.
//
// The assertion is on CONTENT rather than address, because in this
// particular abandonment the card's pointer still names the sector the
// aborted write was addressed to.  A torn sector there is unavoidable
// once the producer dies; a sector whose tail is the NEXT master's
// command bytes is the bug.
static bool test_abandoned_cmd25_bleeds_next_master_into_the_image() {
    reset();
    dut->wdog_dut_sel = DUT_TINY_BASE;
    sd.validates_write_crc = false;

    const uint32_t HDD_BASE = 8192;
    const uint32_t PRAM_LBA = 8191;
    const int      STALL_AT = 100;

    g_wr_bytes_src.resize(512 * 4);
    for (size_t k = 0; k < g_wr_bytes_src.size(); k++)
        g_wr_bytes_src[k] = (uint8_t)((k * 7) ^ 0x1D);

    g_wr_stall_at     = STALL_AT;
    g_wr_stall_cycles = 100000000;

    CHECK_TRUE("the abandoned CMD25 terminates",
               kick_and_wait(CT_CMD25, HDD_BASE, 4, 4000000));
    CHECK_TRUE("it terminated as an error", dut->error != 0);

    // Now the operator saves PRAM, exactly as scenario B does.
    g_wr_stall_at     = 0;
    g_wr_stall_cycles = 0;
    dut->wr_avail     = 1;
    g_wr_bytes_src.assign(512, 0);
    for (int k = 0; k < 512; k++)
        g_wr_bytes_src[k] = (uint8_t)(0x91 ^ (k * 13));
    g_wr_src_idx = 0;
    kick_and_wait(CT_CMD24, PRAM_LBA, 1, 4000000);

    std::map<uint32_t, std::vector<uint8_t> >::const_iterator it =
        sd.captured_writes.find(HDD_BASE);
    if (it != sd.captured_writes.end()) {
        // Everything past the point the producer died must be the abort
        // sequence's own 0x00 padding.  Anything else got there off the
        // SPI bus AFTER sd_ctrl had already reported the request done.
        for (int k = STALL_AT; k < 512; k++) {
            if (it->second[k] != 0x00) {
                printf("  FAIL sector %u byte %d = 0x%02x: the disk image "
                       "was written with traffic from the NEXT master "
                       "(the PRAM save at LBA %u), because the abandoned "
                       "CMD25 left the card mid-block\n",
                       HDD_BASE, k, it->second[k], PRAM_LBA);
                return false;
            }
        }
        for (int k = 0; k < STALL_AT; k++) {
            uint8_t want = (uint8_t)((k * 7) ^ 0x1D);
            if (it->second[k] != want) {
                printf("  FAIL sector %u byte %d: got 0x%02x want 0x%02x\n",
                       HDD_BASE, k, it->second[k], want);
                return false;
            }
        }
    }
    // And the save itself must still be able to reach its own sector.
    CHECK_TRUE("PRAM landed at 8191",
               sd.captured_writes.count(PRAM_LBA) == 1);
    return true;
}

// 1. Park in S_TOK_SEND: rd_ready is low from the very start, so the
//    module walks the command frame and R1 (neither is gated) and then
//    stops dead at the data-token poll.  token_cnt cannot advance,
//    because it only advances on a COMPLETED poll and no poll is ever
//    issued — pre-watchdog this was an unbounded stall with busy high.
static bool test_wdog_fires_parked_in_tok_send() {
    reset();
    // Shipping tick budget, 8-cycle prescaler instead of 4096 — see the
    // pinning note above wdog_limit_ticks().  ~2.0M cycles instead of
    // ~1.0G, with the assertion still measuring REQ_WDOG_BASE_TICKS.
    dut->wdog_dut_sel = DUT_FAST_TICK;
    dut->rd_ready = 0;          // consumer never grants readiness

    const uint64_t LIMIT = wdog_limit_cycles_fast(1);
    uint64_t t0 = sim_time;
    kick(CT_CMD17, 0x00000010, 1);

    // Let the prologue and, with pipelining, the prefetched sector run.
    // The latter parks in RD_DRAIN rather than before the token. Both
    // must retain the watchdog until the consumer receives its bytes.
    for (int i = 0; i < 200000; i++) tick();
    CHECK_TRUE("transport actually started", sp.mosi_bytes >= 8);
    CHECK_TRUE("still busy in the token park", dut->busy != 0);
    CHECK_TRUE("no done yet", dut->done == 0);
    CHECK_TRUE("no error yet", dut->error == 0);

    // Now prove it is genuinely wedged, not merely slow: no further SPI
    // byte is clocked at all over a long window.  200k cycles is ~10% of
    // this instance's whole budget and ~3800 SPI byte-times.
    uint64_t frozen_at = sp.mosi_bytes;
    for (int i = 0; i < 200000; i++) tick();
    CHECK_EQ("SPI transport frozen while parked", sp.mosi_bytes, frozen_at);
    CHECK_TRUE("still busy after the freeze window", dut->busy != 0);

    // Run out the watchdog.
    bool saw_done = false;
    uint8_t busy_at_done = 1;
    while (sim_time - t0 < LIMIT + 200000) {
        if (dut->done) { saw_done = true; busy_at_done = dut->busy; break; }
        tick();
    }
    uint64_t elapsed = sim_time - t0;

    CHECK_TRUE("watchdog answered (done pulsed)", saw_done);
    CHECK_EQ("busy deasserted on the same edge as done", busy_at_done, 0);
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=WDOG", dut->err_cause, ERR_WDOG);
    CHECK_EQ("nothing was ever handed to the consumer",
             g_rd_bytes.size(), 0u);
    CHECK_EQ("SPI stayed frozen right through", sp.mosi_bytes, frozen_at);
    // Two-sided, on purpose.  Not early, because a watchdog that fires
    // well before its budget kills slow-but-healthy cards.  Not late,
    // because this is also what PINS the WDOG_* mirrors above to the
    // RTL's actual REQ_WDOG_* parameters — and
    // test_wdog_budget_covers_a_boot_sized_transfer's arithmetic is only
    // trustworthy while that pin holds.  (The upper bound is enforced by
    // the loop budget: overshoot means no `done` and the "watchdog
    // answered" check fails.)  Running on DUT_FAST_TICK does not weaken
    // this: that copy carries the SHIPPING REQ_WDOG_BASE_TICKS, so the
    // number being pinned here is the shipped one.
    CHECK_TRUE("fired at the budget, not before", elapsed > (LIMIT / 10) * 9);
    CHECK_TRUE("fired by the budget: WDOG_* mirrors match the RTL",
               elapsed <= LIMIT + 100000);
    printf("       (tok_send park: fired at %llu cycles, budget %llu)\n",
           (unsigned long long)elapsed, (unsigned long long)LIMIT);
    return true;
}

// 2. Park in S_RD_FLUSH_S: let the block be received and CRC-checked
//    normally, then drop rd_ready after the FIRST flushed byte.  The
//    flush pace counter only advances while rd_ready is high, so the
//    module stops with 511 bytes still in blk_buf.  This is the state
//    the real HW deadlock parks in (scsi.v holds vh_rd_ready low once
//    its ring hits RING_HIGH_WATER, and only the CPU can drain it).
static bool test_wdog_fires_parked_in_rd_flush() {
    reset();
    dut->wdog_dut_sel = DUT_FAST_TICK;   // see test 1
    dut->rd_ready = 1;

    const uint64_t LIMIT = wdog_limit_cycles_fast(1);
    uint64_t t0 = sim_time;
    kick(CT_CMD17, 0x00000021, 1);

    // Advance until the very first byte reaches the consumer, i.e. we
    // are demonstrably inside S_RD_FLUSH_S (the only state that drives
    // rd_valid), then stall.
    while (g_rd_bytes.empty() && sim_time - t0 < 400000) tick();
    CHECK_TRUE("reached the flush stage", !g_rd_bytes.empty());
    dut->rd_ready = 0;
    size_t frozen_rd = g_rd_bytes.size();
    CHECK_TRUE("stalled mid-block, not after it", frozen_rd < 512);

    uint64_t frozen_mosi = sp.mosi_bytes;
    for (int i = 0; i < 200000; i++) tick();
    CHECK_EQ("flush frozen: no further bytes to the consumer",
             g_rd_bytes.size(), frozen_rd);
    CHECK_EQ("SPI frozen too", sp.mosi_bytes, frozen_mosi);
    CHECK_TRUE("still busy while parked mid-flush", dut->busy != 0);
    CHECK_TRUE("no done yet", dut->done == 0);

    bool saw_done = false;
    uint8_t busy_at_done = 1;
    while (sim_time - t0 < LIMIT + 400000) {
        if (dut->done) { saw_done = true; busy_at_done = dut->busy; break; }
        tick();
    }
    uint64_t elapsed = sim_time - t0;

    CHECK_TRUE("watchdog answered (done pulsed)", saw_done);
    CHECK_EQ("busy deasserted on the same edge as done", busy_at_done, 0);
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=WDOG", dut->err_cause, ERR_WDOG);
    CHECK_EQ("consumer got exactly what it accepted, no more",
             g_rd_bytes.size(), frozen_rd);
    CHECK_TRUE("fired at the budget, not before", elapsed > (LIMIT / 10) * 9);
    printf("       (rd_flush park: fired at %llu cycles, budget %llu, "
           "%zu/512 bytes flushed)\n",
           (unsigned long long)elapsed, (unsigned long long)LIMIT, frozen_rd);
    return true;
}

// 3. The negative control, and the one that matters most: a consumer
//    that is very slow but always eventually accepts must NOT be
//    killed.  A watchdog that fires here converts a hang into spurious
//    disk errors, which is strictly worse.
//
//    Runs on the SHIPPING instance, deliberately: this is the one
//    scenario where the question is "does the configuration we actually
//    ship kill a healthy consumer", so a time-scaled copy would be
//    answering a different question.
//
//    rd_ready is granted in 16-cycle windows once every 16,384 cycles.
//    S_RD_FLUSH_S needs FLUSH_PACE_CYCLES=32 rd_ready-high cycles per
//    byte, so each byte costs two windows: ~32,768 cycles/byte, ~16.8M
//    cycles/block — roughly 317x the 52,822 cycles a healthy block
//    takes.  Two blocks come to ~33.7M cycles.  Against the 10 s base
//    that is ~3.3% of the 1.007G-cycle budget (it was ~33% of the old
//    1 s base) — the transfer is just as pathological, the budget got
//    bigger.  The tight-margin half of this property, "the watchdog does
//    not fire EARLY", is pinned separately and two-sidedly by the two
//    park scenarios' `elapsed > 0.9 * LIMIT` lower bound.
static bool test_wdog_does_not_kill_a_slow_but_progressing_read() {
    reset();

    const uint16_t BLOCKS = 2;
    const uint64_t LIMIT  = wdog_limit_cycles(BLOCKS);
    const uint32_t LBA    = 0x00000100;
    const uint64_t PERIOD = 16384;
    const uint64_t WINDOW = 16;

    dut->rd_ready = 1;
    uint64_t t0 = sim_time;
    kick(CT_CMD18, LBA, BLOCKS);

    bool saw_done = false;
    while (sim_time - t0 < LIMIT + 400000) {
        dut->rd_ready = (((sim_time - t0) % PERIOD) < WINDOW) ? 1 : 0;
        if (dut->done) { saw_done = true; break; }
        tick();
    }
    uint64_t elapsed = sim_time - t0;

    CHECK_TRUE("slow read completed", saw_done);
    CHECK_TRUE("NOT killed by the watchdog", dut->error == 0);
    CHECK_EQ("err_cause=NONE", dut->err_cause, ERR_NONE);
    CHECK_EQ("every byte of every block arrived",
             g_rd_bytes.size(), (size_t)(512 * BLOCKS));
    for (int b = 0; b < BLOCKS; b++) {
        for (int k = 0; k < 512; k++) {
            uint8_t want = (uint8_t)(((LBA + b) * 37 + k) & 0xFF);
            if (g_rd_bytes[b * 512 + k] != want) {
                printf("  FAIL block %d byte %d: got 0x%02x, want 0x%02x\n",
                       b, k, g_rd_bytes[b * 512 + k], want);
                return false;
            }
        }
    }
    CHECK_TRUE("stop-tran CMD12 still issued", sd.cmd12_count >= 1);
    // Positive control on the test itself: if this ever stops being a
    // genuinely slow transfer (e.g. someone "fixes" the pacing), it is
    // no longer testing what it claims to.
    CHECK_TRUE("this really was a pathologically slow transfer",
               elapsed > 100ull * 52822ull);
    CHECK_TRUE("and it still finished inside the budget", elapsed < LIMIT);
    printf("       (slow read: completed in %llu cycles, budget %llu, "
           "%.1f%% of budget)\n",
           (unsigned long long)elapsed, (unsigned long long)LIMIT,
           100.0 * (double)elapsed / (double)LIMIT);
    return true;
}

// 4. The other way to get this wrong: boot_fsm.v issues ONE CMD18 for
//    all NUM_SECTORS=2048 ROM sectors.  A flat "~1 second" watchdog
//    would kill the ROM load on real hardware and no tb in this repo
//    would notice — tb_sd_boot instantiates boot_fsm with
//    NUM_SECTORS=16, so it never gets near the budget.  Simulating the
//    full 2048 blocks costs ~108M cycles, so instead: measure the real
//    per-block cost on a 256-block run at full speed, then check the
//    boot-sized budget against that MEASURED number rather than against
//    an arithmetic guess.
static bool test_wdog_budget_covers_a_boot_sized_transfer() {
    reset();
    dut->rd_ready = 1;

    const uint16_t BLOCKS    = 256;
    const uint16_t BOOT_BLKS = 2048;      // boot_fsm.v NUM_SECTORS
    const uint32_t LBA       = 0x00000000;

    uint64_t t0 = sim_time;
    CHECK_TRUE("256-block CMD18 completes",
               kick_and_wait(CT_CMD18, LBA, BLOCKS, 40000000));
    uint64_t elapsed = sim_time - t0;

    CHECK_TRUE("no error on a healthy full-speed multi-block read",
               dut->error == 0);
    CHECK_EQ("all bytes arrived", g_rd_bytes.size(), (size_t)(512 * BLOCKS));

    uint64_t per_block  = elapsed / BLOCKS;
    uint64_t boot_cost  = per_block * BOOT_BLKS;
    uint64_t boot_budget = wdog_limit_cycles(BOOT_BLKS);
    printf("       (per-block %llu cycles; boot-sized %u-block read would "
           "cost ~%llu, budget %llu, %.2fx margin)\n",
           (unsigned long long)per_block, BOOT_BLKS,
           (unsigned long long)boot_cost,
           (unsigned long long)boot_budget,
           (double)boot_budget / (double)boot_cost);
    CHECK_TRUE("this run had budget headroom",
               elapsed < wdog_limit_cycles(BLOCKS));
    CHECK_TRUE("boot-sized ROM load fits the watchdog budget with >=2x margin",
               boot_budget > 2 * boot_cost);
    return true;
}

// 5. Pins WDOG_TICK_CYCLES — the one mirror the park scenarios cannot,
//    because they run on a copy whose prescaler is deliberately short.
//    DUT_TINY_BASE has the SHIPPING prescaler and per-block shift with
//    the base collapsed to 1 tick, so a 1-block request expires at
//    (1 + 32) * 4096 = 135,168 cycles.  Same park mechanism as scenario
//    1 (rd_ready low from the start), 15,000x cheaper, and it goes red
//    in both directions if REQ_WDOG_TICK_LOG2 moves.
static bool test_wdog_prescaler_is_the_shipped_tick() {
    reset();
    dut->wdog_dut_sel = DUT_TINY_BASE;
    dut->rd_ready = 0;

    const uint64_t LIMIT =
        (TINY_BASE_TICKS + WDOG_BLK_TICKS) * WDOG_TICK_CYCLES;
    uint64_t t0 = sim_time;
    kick(CT_CMD17, 0x00000010, 1);

    bool saw_done = false;
    while (sim_time - t0 < LIMIT + 20000) {
        if (dut->done) { saw_done = true; break; }
        tick();
    }
    uint64_t elapsed = sim_time - t0;

    CHECK_TRUE("watchdog answered (done pulsed)", saw_done);
    CHECK_EQ("err_cause=WDOG", dut->err_cause, ERR_WDOG);
    CHECK_EQ("nothing was handed to the consumer", g_rd_bytes.size(), 0u);
    // Two-sided against the shipping 4096-cycle tick.  A TICK_LOG2 of 11
    // would land at 67,584 and trip the lower bound; 13 would land at
    // 270,336, overrun the loop, and trip "watchdog answered".
    CHECK_TRUE("fired at the budget, not before", elapsed > (LIMIT / 10) * 9);
    CHECK_TRUE("fired by the budget: WDOG_TICK_CYCLES matches the RTL",
               elapsed <= LIMIT + 20000);
    printf("       (tiny-base park: fired at %llu cycles, budget %llu "
           "= (%llu+%llu) ticks x %llu cycles/tick)\n",
           (unsigned long long)elapsed, (unsigned long long)LIMIT,
           (unsigned long long)TINY_BASE_TICKS,
           (unsigned long long)WDOG_BLK_TICKS,
           (unsigned long long)WDOG_TICK_CYCLES);
    return true;
}

// 5b. REQ_WDOG_ENABLE=0 removes the escape — and ONLY the escape.
//
//     DUT_WDOG_OFF is DUT_TINY_BASE's parameter-for-parameter twin with
//     the enable cleared, and this runs scenario 5's EXACT park (rd_ready
//     low from the start, same CMD17, same block count).  Scenario 5
//     proves that park fires ERR_WDOG at 135,168 cycles; this proves the
//     same park does NOT fire with the knob off.
//
//     The pairing is what makes this non-vacuous.  A test that only
//     asserted "no ERR_WDOG here" would pass just as happily if the park
//     had silently stopped stalling, if `go` never reached the instance,
//     or if the mux were wired to the wrong copy — so it would keep
//     passing after the knob became a no-op.  Because its twin fires on
//     the identical stimulus, "no fire" can only mean the enable did it.
//
//     So we also assert the request is still PARKED (busy high, nothing
//     delivered) well past the budget: the point of the knob is that a
//     stalled request waits forever instead of erroring, and "no error
//     because it quietly completed" would be a different bug wearing the
//     same green tick.
static bool test_wdog_disable_removes_the_escape() {
    reset();
    dut->wdog_dut_sel = DUT_WDOG_OFF;
    dut->rd_ready = 0;

    const uint64_t LIMIT =
        (TINY_BASE_TICKS + WDOG_BLK_TICKS) * WDOG_TICK_CYCLES;
    uint64_t t0 = sim_time;
    kick(CT_CMD17, 0x00000010, 1);

    // Run WELL past the budget its twin fires at (4x + the same slack
    // scenario 5 allows), so this cannot pass by simply not waiting long
    // enough.
    bool saw_done = false;
    while (sim_time - t0 < LIMIT * 4 + 20000) {
        if (dut->done) { saw_done = true; break; }
        tick();
    }
    uint64_t waited = sim_time - t0;

    CHECK_TRUE("no watchdog escape: done never pulsed", !saw_done);
    CHECK_TRUE("request is still parked, not quietly finished",
               dut->busy != 0);
    CHECK_EQ("still nothing handed to the consumer", g_rd_bytes.size(), 0u);
    CHECK_TRUE("waited past the twin's fire point",
               waited > LIMIT * 2);
    printf("       (wdog-off park: still busy after %llu cycles; the "
           "enabled twin fires at %llu)\n",
           (unsigned long long)waited, (unsigned long long)LIMIT);
    return true;
}

// 6. The other arithmetic constraint on the base, and the reason it can
//    never be shrunk back to "however long feels snappy": every OTHER
//    error cause in this module is a MORE SPECIFIC diagnosis than
//    ERR_WDOG, and the global watchdog silently destroys any of them
//    whose own budget it undercuts.  The largest is POLL_TIMEOUT's R1
//    wait.  Rather than recompute it from SPI arithmetic, use the cost
//    test_r1_timeout actually MEASURED a few scenarios ago.
//
//    This costs zero extra simulated cycles: test_r1_timeout already
//    runs the full silent-card poll to completion.
static bool test_wdog_base_clears_the_r1_poll_timeout() {
    // No recorded cost means test_r1_timeout never reached ERR_R1_TO —
    // which, unless someone reordered the RUN list, means the watchdog
    // preempted it.  That IS the failure this scenario guards against,
    // so it is a hard fail here rather than a skip.
    CHECK_TRUE("test_r1_timeout reached ERR_R1_TO and recorded its cost "
               "(if not: the watchdog already preempted it)",
               g_r1_timeout_cycles > 0);

    uint64_t base_cycles = WDOG_BASE_TICKS * WDOG_TICK_CYCLES;
    // A command can legitimately burn a full R1 poll AND a full
    // write-busy poll (BUSY_TIMEOUT == POLL_TIMEOUT in sd_ctrl.v, and
    // both are paced by the same SPI byte clock, so the doubled case
    // costs ~2x the measured single one).  Covering that is what keeps
    // ERR_BUSY_TO reportable rather than degrading it to ERR_WDOG.
    uint64_t r1_plus_busy = 2 * g_r1_timeout_cycles;

    printf("       (base %llu cycles; measured R1 poll timeout %llu "
           "-> %.1fx; R1+busy %llu -> %.1fx)\n",
           (unsigned long long)base_cycles,
           (unsigned long long)g_r1_timeout_cycles,
           (double)base_cycles / (double)g_r1_timeout_cycles,
           (unsigned long long)r1_plus_busy,
           (double)base_cycles / (double)r1_plus_busy);

    CHECK_TRUE("watchdog base sits above the R1 poll timeout, so "
               "ERR_R1_TO survives as the reported cause",
               base_cycles > g_r1_timeout_cycles);
    CHECK_TRUE("...and above a full R1 poll plus a full write-busy poll, "
               "so ERR_BUSY_TO survives too",
               base_cycles > r1_plus_busy);
    return true;
}

// ── CRC16 validate/retry scenarios (crc_check_en=1) ────────────────────

static bool test_crc_check_good_passthrough() {
    reset();
    dut->crc_check_en = 1;
    const uint32_t LBA = 0x00000042;

    CHECK_TRUE("CMD17 completes", kick_and_wait(CT_CMD17, LBA, 1));
    CHECK_TRUE("no error", !dut->error);
    CHECK_EQ("err_cause=NONE", dut->err_cause, ERR_NONE);
    CHECK_EQ("rd_bytes size", g_rd_bytes.size(), 512u);
    CHECK_EQ("exactly one CMD17 frame", sd.cmd17_frames_seen, 1);
    for (int k = 0; k < 512; k++) {
        uint8_t want = (uint8_t)((LBA * 37 + k) & 0xFF);
        if (g_rd_bytes[k] != want) {
            printf("  FAIL byte %d: got 0x%02x, want 0x%02x\n",
                   k, g_rd_bytes[k], want);
            return false;
        }
    }
    return true;
}

static bool test_crc_check_cmd18_good_passthrough() {
    // Same as test_cmd18_multi_read but with crc_check_en=1 — confirms
    // the buffer-then-flush restructuring doesn't disturb a normal
    // multi-block run when every block's CRC is correct.
    reset();
    dut->crc_check_en = 1;
    const uint32_t BASE = 0x00001000;
    const uint16_t N    = 4;

    CHECK_TRUE("CMD18 completes", kick_and_wait(CT_CMD18, BASE, N, 8000000));
    CHECK_TRUE("no error", !dut->error);
    CHECK_EQ("rd_bytes size", g_rd_bytes.size(), (unsigned)(512 * N));
    for (int b = 0; b < N; b++) {
        uint32_t lba = BASE + b;
        for (int k = 0; k < 512; k++) {
            uint8_t want = (uint8_t)((lba * 37 + k) & 0xFF);
            uint8_t got  = g_rd_bytes[b * 512 + k];
            if (got != want) {
                printf("  FAIL block %d byte %d: got 0x%02x want 0x%02x\n",
                       b, k, got, want);
                return false;
            }
        }
    }
    CHECK_TRUE("CMD12 was issued", sd.cmd12_count >= 1);
    return true;
}

static bool test_crc_check_cmd17_retry_then_succeed() {
    reset();
    dut->crc_check_en = 1;
    sd.corrupt_block_serial = 0;   // only the FIRST attempt is bad
    const uint32_t LBA = 0x00000077;

    CHECK_TRUE("CMD17 completes", kick_and_wait(CT_CMD17, LBA, 1));
    CHECK_TRUE("no error (retry recovered)", !dut->error);
    CHECK_EQ("err_cause=NONE", dut->err_cause, ERR_NONE);
    CHECK_EQ("exactly two CMD17 frames sent (1 bad + 1 retry)",
             sd.cmd17_frames_seen, 2);
    CHECK_EQ("rd_bytes size — only the GOOD attempt reached the caller",
             g_rd_bytes.size(), 512u);
    for (int k = 0; k < 512; k++) {
        uint8_t want = (uint8_t)((LBA * 37 + k) & 0xFF);
        if (g_rd_bytes[k] != want) {
            printf("  FAIL byte %d: got 0x%02x, want 0x%02x\n",
                   k, g_rd_bytes[k], want);
            return false;
        }
    }
    return true;
}

static bool test_crc_check_cmd17_exhausts_retries() {
    reset();
    dut->crc_check_en = 1;
    sd.corrupt_all_blocks = true;   // every attempt is bad, forever

    CHECK_TRUE("cmd completes", kick_and_wait(CT_CMD17, 0, 1, 6000000));
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=CRC_BAD", dut->err_cause, ERR_CRC_BAD);
    // Initial attempt + CRC_RETRY_MAX(5) retries = 6 frames total.
    CHECK_EQ("6 CMD17 frames sent (1 + 5 retries)",
             sd.cmd17_frames_seen, 6);
    CHECK_EQ("NOTHING ever reached the caller — no partial/bad data leaked",
             g_rd_bytes.size(), 0u);
    return true;
}

static bool test_crc_check_cmd18_aborts_on_bad_block() {
    reset();
    dut->crc_check_en = 1;
    sd.corrupt_block_serial = 0;   // the very first block is bad
    const uint32_t BASE = 0x00003000;
    const uint16_t N    = 3;

    CHECK_TRUE("cmd completes", kick_and_wait(CT_CMD18, BASE, N, 6000000));
    CHECK_TRUE("error raised", dut->error != 0);
    CHECK_EQ("err_cause=CRC_BAD", dut->err_cause, ERR_CRC_BAD);
    CHECK_TRUE("CMD12 issued to cleanly stop the card",
               sd.cmd12_count >= 1);
    CHECK_EQ("no bytes reached the caller (first block was the bad one)",
             g_rd_bytes.size(), 0u);
    return true;
}

static bool test_read_banks_backpressure_and_random_stalls() {
    reset();
    dut->crc_check_en = 1;
    const uint32_t base = 0x423;
    kick(CT_CMD18, base, 5);
    const uint64_t first_start = sim_time;
    while (g_rd_bytes.size() < 17 && sim_time - first_start < 1000000) tick();
    CHECK_EQ("initial bytes arrived", g_rd_bytes.size(), 17u);
    dut->rd_ready = 0;
    // Allow the producer to fill all available banks, then verify it parks
    // instead of overwriting unread bytes or consuming the next sector.
    for (int i = 0; i < 200000; ++i) tick();
    const uint64_t wire_bytes = sp.mosi_bytes;
    for (int i = 0; i < 10000; ++i) tick();
    CHECK_EQ("full banks stop SPI", sp.mosi_bytes, wire_bytes);
    CHECK_EQ("stalled consumer sees no bytes", g_rd_bytes.size(), 17u);
    CHECK_TRUE("no early completion while bytes buffered", dut->busy && !dut->done);

    uint32_t rng = 0x53444344;
    const uint64_t start = sim_time;
    while (!dut->done && sim_time - start < 2000000) {
        rng = rng * 1664525u + 1013904223u;
        dut->rd_ready = (rng >> 29) == 0;
        tick();
    }
    CHECK_TRUE("stalled transfer completes", dut->done && !dut->error);
    CHECK_EQ("exactly five sectors before done", g_rd_bytes.size(), 5u * 512);
    for (size_t i = 0; i < g_rd_bytes.size(); ++i)
        CHECK_EQ("bank order and contents", g_rd_bytes[i],
                 uint8_t((base + i / 512) * 37 + i % 512));
    dut->rd_ready = 1;
    for (int i = 0; i < 1000; ++i) tick();
    CHECK_EQ("no bytes after done", g_rd_bytes.size(), 5u * 512);
    return true;
}

static bool test_crc_later_bank_never_published() {
    reset();
    dut->crc_check_en = 1;
    sd.corrupt_block_serial = 1;
    const uint32_t base = 0x724;
    kick(CT_CMD18, base, 4);
    const uint64_t start = sim_time;
    while (g_rd_bytes.size() < 17 && sim_time - start < 1000000) tick();
    CHECK_EQ("first bank begins delivery", g_rd_bytes.size(), 17u);
    dut->rd_ready = 0;
    for (int i = 0; i < 200000; ++i) tick();
    dut->rd_ready = 1;
    while (!dut->done && sim_time - start < 2000000) tick();
    CHECK_TRUE("bad second bank completes with error", dut->done && dut->error);
    CHECK_EQ("CRC failure recorded", dut->err_cause, ERR_CRC_BAD);
    CHECK_EQ("only validated first sector delivered", g_rd_bytes.size(), 512u);
    CHECK_TRUE("card read session closed", sd.cmd12_count != 0);
    for (size_t i = 0; i < g_rd_bytes.size(); ++i)
        CHECK_EQ("good first sector intact", g_rd_bytes[i], uint8_t(base * 37 + i));
    for (int i = 0; i < 1000; ++i) tick();
    CHECK_EQ("bad bytes never escape after completion", g_rd_bytes.size(), 512u);
    return true;
}

static bool test_reset_discards_queued_read_banks() {
    reset();
    dut->crc_check_en = 1;
    kick(CT_CMD18, 0x824, 4);
    const uint64_t start = sim_time;
    while (g_rd_bytes.size() < 17 && sim_time - start < 1000000) tick();
    CHECK_EQ("read was active", g_rd_bytes.size(), 17u);
    dut->rd_ready = 0;
    for (int i = 0; i < 200000; ++i) tick();
    dut->rst = 1;
    for (int i = 0; i < 8; ++i) tick();
    dut->rst = 0;
    dut->rd_ready = 1;
    for (int i = 0; i < 1000; ++i) {
        tick();
        CHECK_TRUE("reset leaves no queued byte or completion", !dut->rd_valid && !dut->done);
    }
    CHECK_TRUE("controller idle after reset", !dut->busy);
    // System reset reinitializes the card before another request. Model
    // that separately; this test does not claim reset sends CMD12.
    return test_cmd17_read();
}

static bool test_crc_write_sends_real_crc() {
    reset();
    // Write-side CRC generation is unconditional — exercise it with
    // crc_check_en left at its default (0) to confirm it doesn't depend
    // on the read-side gate.
    const uint32_t LBA = 0x00000055;
    g_wr_bytes_src.resize(512);
    for (int k = 0; k < 512; k++) g_wr_bytes_src[k] = (uint8_t)(0xA5 ^ k);

    CHECK_TRUE("CMD24 completes", kick_and_wait(CT_CMD24, LBA, 1));
    CHECK_TRUE("no error", !dut->error);
    auto it = sd.captured_write_crcs.find(LBA);
    CHECK_TRUE("CRC bytes were captured", it != sd.captured_write_crcs.end());
    uint16_t expect = crc16_ccitt(sd.captured_writes[LBA]);
    CHECK_EQ("data-block CRC16 matches an independent computation",
             it->second, expect);
    CHECK_TRUE("CRC is a REAL value, not the old dummy 0x0000",
               it->second != 0x0000);
    return true;
}

// The per-scenario cycle cost is printed because this module's timeouts
// are all wall-clock budgets and the global request watchdog
// (REQ_WDOG_LIMIT in sd_ctrl.v) has to sit above every legitimate
// scenario's cost.  Anyone re-sizing that constant should read these
// numbers rather than re-deriving them from SPI arithmetic.
#define RUN(fn) do { \
    uint64_t _t0 = sim_time; \
    bool ok = fn(); \
    uint64_t _dt = sim_time - _t0; \
    if (ok) { printf("[PASS] " #fn " (%llu cycles)\n", \
                     (unsigned long long)_dt); n_pass++; } \
    else    { printf("[FAIL] " #fn " (%llu cycles)\n", \
                     (unsigned long long)_dt); n_fail++; } \
} while(0)

#ifdef SDCTRL_WRITE_STAGE
static bool test_staged_reset_during_fill() {
    // First sector and partially staged successor: no token for an incomplete
    // bank, and buffered-but-unsent data must not escape after reset.
    for (int consumed : {1, 137, 511, 512 + 137, 512 + 138}) {
        reset();
        sd.validates_write_crc = false;
        const uint32_t lba = 0x7000;
        std::vector<uint8_t> original(3 * 512);
        for (size_t i = 0; i < original.size(); ++i) original[i] = uint8_t(i * 13 + 9);
        g_wr_bytes_src = original;
        if (consumed == 512 + 138) {
            g_wr_stall_at = consumed;
            g_wr_stall_cycles = 1000000;
        }
        dut->cmd_type = CT_CMD25; dut->lba = lba;
        dut->block_count = 3; dut->go = 1;
        tick(); dut->go = 0; dut->cmd_type = CT_IDLE;
        uint64_t guard = 1000000;
        while (g_wr_src_idx < size_t(consumed) && guard-- > 0) tick();
        CHECK_TRUE("staging reset landing reached", guard > 0);
        if (consumed == 512 + 138) {
            guard = 1000000;
            while (dut->dbg_state != 47 && guard-- > 0) tick();
            CHECK_TRUE("reset between sectors with partial next bank", guard > 0);
        }
        if (consumed < 512) CHECK_TRUE("no write command before full sector", !sd.write_session_open());
        g_wr_bytes_src.assign(3 * 512, 0xDE); g_wr_src_idx = 0;
        dut->rst = 1;
        for (int n = 0; n < 8; ++n) tick();
        dut->rst = 0;
        guard = 1000000;
        while (dut->busy && guard-- > 0) tick();
        CHECK_TRUE("staged reset closes", guard > 0 && !sd.write_session_open());
        for (const auto& [addr, data] : sd.captured_writes) {
            CHECK_TRUE("only complete first sector may commit", consumed >= 512 && addr == lba);
            CHECK_TRUE("reset cannot change active payload", data == std::vector<uint8_t>(original.begin(), original.begin() + 512));
        }
        tick(); tick();
        g_wr_stall_cycles = 0;
        g_wr_bytes_src.assign(512, 0xA7); g_wr_src_idx = 0;
        CHECK_TRUE("following PRAM write completes", kick_and_wait(CT_CMD24, 8191, 1));
        CHECK_TRUE("following write reaches correct sector", sd.captured_writes[8191] == g_wr_bytes_src);
    }
    return true;
}

static bool test_staged_busy_timeout_quarantines() {
    // A prior quarantine is intentionally not resettable: simulate a real
    // FPGA/card power cycle before this independent fault injection.
    dut->final(); delete dut; dut = new Vtb_sd_ctrl;
    reset();
    sd.write_busy_ticks = 10000; // staged unit instance has a 64-poll limit
    g_wr_bytes_src.assign(1024, 0x63);
    dut->cmd_type = CT_CMD25; dut->lba = 0x7500;
    dut->block_count = 2; dut->go = 1;
    tick(); dut->go = 0;
    uint64_t guard = 1000000;
    while (dut->dbg_state != 48 && guard-- > 0) tick();
    CHECK_TRUE("busy timeout enters quarantine", guard > 0);
    CHECK_TRUE("quarantine retains owner and error", dut->busy && dut->error);
    tick();
    CHECK_TRUE("quarantine reports failed completion", dut->done && dut->error);
    tick();
    CHECK_TRUE("quarantine completion is a pulse", !dut->done);
    const auto before = sd.captured_writes;
    dut->rst = 1;
    for (int n = 0; n < 8; ++n) tick();
    dut->rst = 0;
    dut->cmd_type = CT_CMD24; dut->lba = 8191; dut->block_count = 1;
    dut->go = 1;
    tick();
    CHECK_TRUE("new requests fail without accessing card", dut->done && dut->error);
    dut->go = 0;
    for (int n = 0; n < 10000; ++n) tick();
    CHECK_TRUE("reset/go cannot release quarantined owner", dut->busy && dut->dbg_state == 48);
    CHECK_TRUE("no following writes while quarantined", sd.captured_writes == before);
    return true;
}

static bool test_staged_reset_transport_deadline() {
    reset();
    g_wr_bytes_src.assign(1024, 0x47);
    dut->cmd_type = CT_CMD25; dut->lba = 0x7600;
    dut->block_count = 2; dut->go = 1;
    tick(); dut->go = 0;
    uint64_t guard = 1000000;
    while (!(dut->dbg_state == S_W_DATA_W && dut->dbg_byte_state == BS_WAIT)
           && guard-- > 0) tick();
    CHECK_TRUE("transport fault landing reached", guard > 0);
    dut->stall_transport = 1;
    dut->rst = 1;
    for (int n = 0; n < 8; ++n) tick();
    dut->rst = 0;
    guard = 100000;
    while (dut->dbg_state != 48 && guard-- > 0) tick();
    CHECK_TRUE("reset cleanup deadline works with normal watchdog disabled", guard > 0);
    CHECK_TRUE("lost byte response retains bus ownership", dut->busy && dut->error);
    return true;
}
#endif

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_sd_ctrl;

    RUN(test_cmd17_read);
    RUN(test_cmd18_multi_read);
    RUN(test_cmd12_nonff_stuff_byte);
    RUN(test_cmd24_write);
    RUN(test_cmd25_multi_write);
    RUN(test_cmd25_then_cmd24_without_reset);
#ifndef SDCTRL_WRITE_STAGE
    RUN(test_reset_mid_cmd25_closes_card_before_restart);
#endif
    RUN(test_reset_at_every_cmd25_write_phase);
    RUN(test_wr_avail_stalls_the_data_block);
    RUN(test_wr_avail_stall_at_block_boundary);
    RUN(test_cmd25_per_block_state_resets);
#ifndef SDCTRL_WRITE_STAGE
    RUN(test_cmd25_abort_closes_the_card);
    RUN(test_cmd25_abort_then_pram_save_hits_only_8191);
    RUN(test_abandoned_cmd25_bleeds_next_master_into_the_image);
#endif
    RUN(test_r1_bad);
    RUN(test_r1_timeout);
    RUN(test_dr_bad);
    RUN(test_unknown_cmd);
    RUN(test_wdog_fires_parked_in_tok_send);
    RUN(test_wdog_fires_parked_in_rd_flush);
    RUN(test_wdog_does_not_kill_a_slow_but_progressing_read);
    RUN(test_wdog_budget_covers_a_boot_sized_transfer);
    RUN(test_wdog_prescaler_is_the_shipped_tick);
    RUN(test_wdog_disable_removes_the_escape);
    RUN(test_wdog_base_clears_the_r1_poll_timeout);
    RUN(test_crc_check_good_passthrough);
    RUN(test_crc_check_cmd18_good_passthrough);
    RUN(test_crc_check_cmd17_retry_then_succeed);
    RUN(test_crc_check_cmd17_exhausts_retries);
    RUN(test_crc_check_cmd18_aborts_on_bad_block);
    RUN(test_read_banks_backpressure_and_random_stalls);
    RUN(test_crc_later_bank_never_published);
    RUN(test_reset_discards_queued_read_banks);
    RUN(test_crc_write_sends_real_crc);
#ifdef SDCTRL_WRITE_STAGE
    RUN(test_staged_reset_during_fill);
    RUN(test_staged_reset_transport_deadline);
    // Last: quarantine intentionally cannot be cleared by ordinary reset.
    RUN(test_staged_busy_timeout_quarantines);
#endif

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
