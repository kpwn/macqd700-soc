// tb_sd_boot.cpp — Verilator testbench for boot_fsm + sd_spi integration
//
// Wires up sd_spi (SPI master) and boot_fsm (SD init + AXI burst writer) to
// a software model of an SD card and a tiny AXI4-MM slave.  The SD-card
// model:
//   - Responds to the standard init sequence (CMD0 → CMD8 → CMD55/ACMD41 →
//     CMD58 → CMD6 → one CMD18 multi-block run).
//   - Rejects CMD6 (returns R1=0x04 "illegal command") so the fallback
//     "stay at fast mode" path in boot_fsm is exercised.
//   - Serves canned sector payloads for sectors 0..NUM_SECTORS-1.
//     Sector N is filled with the pattern: byte[k] = (N * 37 + k) & 0xFF.
//
// The AXI slave:
//   - Accepts AW bursts (INCR, AWLEN=127, AWSIZE=2), handshakes a single W
//     beat per cycle, captures the 32-bit word at the burst address + beat
//     offset, and replies B=OKAY once WLAST is observed.
//   - Backing store is a small sparse map<uint32_t, uint32_t>.
//
// Build: make tb-sd-boot
// Expected output: all scenarios PASSED with exit 0.
//
// The default NUM_SECTORS for the tb is 16 (8 KiB) — production value of
// 2048 would take far too long in sim at the 400 kHz init rate / 25 MHz
// fast rate.  We pass NUM_SECTORS=16 via a `boot_fsm #(.NUM_SECTORS(16))`
// instantiation in the wrapper module (tb_sd_boot_top.v) — see Makefile
// target.  To further accelerate, SLOW_HALF/FAST_HALF/HS_HALF are also
// overridden to {4, 4, 4} so each SPI byte takes ~32 core cycles.  16
// sectors (vs the previous 4) gives enough headroom to clearly see the
// CMD18 amortisation of the per-sector CMD17 frame overhead.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <verilated.h>
#include "Vtb_sd_boot_top.h"

static Vtb_sd_boot_top* dut      = nullptr;
static uint64_t         sim_time = 0;
static int              n_pass   = 0;
static int              n_fail   = 0;

// CRC16-CCITT, matching sd_ctrl.v's crc16_step bit-for-bit (poly 0x1021,
// MSB-first, init 0) — copied from tb_sd_ctrl.cpp so this tb's SD-card
// model can emit real, correctly-computed CRC16 once boot_fsm enables
// card-side checking via CMD59.
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

static const int NUM_SECTORS = 16;   // must match instantiation in wrapper

// boot_fsm.v ST_* encodings we assert on directly (localparam [5:0] block).
static const unsigned ST_DONE  = 20;
static const unsigned ST_ZERO_B = 24;
static const unsigned ST_ERROR = 21;

// boot_fsm.v ERR_CAUSE_BRESP.
static const unsigned ERR_CAUSE_BRESP = 7;

// RAM pre-zero pass size the wrapper was elaborated with.  The Makefile
// drives BOTH this define and the wrapper's -GZERO_BYTES from one variable,
// so they cannot drift.  0 = zero pass disabled (default tb-sd-boot build).
#ifndef TB_ZERO_BYTES
#define TB_ZERO_BYTES 0u
#endif
static const uint32_t ZERO_BYTES = (uint32_t)(TB_ZERO_BYTES);

// ROM-mirror + VRAM-zero build knobs.  Driven by the tb-sd-boot-mirror
// Makefile target from the SAME -G overrides given to the wrapper, so the
// Verilog elaboration and this file's expectations cannot drift apart —
// same pattern as TB_ZERO_BYTES/ZERO_BYTES above.  0/0 (the defaults)
// leave the +mirror scenario gate a no-op on every other build.
#ifndef TB_MIRROR_LOW_RAM
#define TB_MIRROR_LOW_RAM 0
#endif
#ifndef TB_MIRROR_IMAGE_BYTES
#define TB_MIRROR_IMAGE_BYTES 0u
#endif
#ifndef TB_VRAM_ZERO_BASE
#define TB_VRAM_ZERO_BASE 0u
#endif
#ifndef TB_VRAM_ZERO_BYTES
#define TB_VRAM_ZERO_BYTES 0u
#endif
static const bool     MIRROR_LOW_RAM     = (TB_MIRROR_LOW_RAM) != 0;
static const uint32_t MIRROR_IMAGE_BYTES = (uint32_t)(TB_MIRROR_IMAGE_BYTES);
static const uint32_t VRAM_ZERO_BASE     = (uint32_t)(TB_VRAM_ZERO_BASE);
static const uint32_t VRAM_ZERO_BYTES    = (uint32_t)(TB_VRAM_ZERO_BYTES);

// ═════════════════════════════════════════════════════════════════════
// AXI4-MM write slave model
// ═════════════════════════════════════════════════════════════════════
struct AxiSlave {
    // AW channel
    bool     aw_seen     = false;
    uint32_t aw_addr     = 0;
    uint8_t  aw_len      = 0;     // beats - 1
    uint8_t  aw_id       = 0;
    int      aw_delay    = 0;
    uint32_t aw_count    = 0;
    uint8_t  max_aw_len  = 0;

    // W channel
    int      beats_left  = 0;
    uint32_t next_addr   = 0;

    // B channel
    bool     b_pending   = false;
    uint8_t  b_id        = 0;

    // ── BRESP fault injection (RAM pre-zero pass) ──────────────────────
    // The zero pass writes to low RAM (address < ROM_BASE); the ROM copy
    // writes at/above it.  err_on_zero_write is the 1-based ordinal of the
    // zero-pass write whose B response should come back SLVERR; 0 = never.
    // zero_aw_count counts every zero-pass AW accepted, so a test can prove
    // the FSM actually STOPPED issuing them after the error.
    uint32_t err_on_zero_write = 0;
    uint32_t zero_aw_count     = 0;
    // W BEATS (not AWs) retired by the zero pass.  The pass now issues
    // multi-beat INCR bursts (boot_fsm ZERO_AWLEN), so the AW count is
    // ZERO_BYTES/(4*(AWLEN+1)) and varies with the burst shape, while the
    // beat count is always ZERO_BYTES/4.  Assert on the beats so the test
    // measures COVERAGE rather than the transfer shape.
    uint32_t zero_w_beats      = 0;
    bool     inject_err_b      = false;   // latched at AW, cleared at B
    // ── Zero-pass cycle accounting (perf measurement) ──────────────────
    // sim_time at the first zero-pass AW handshake and at the last zero-pass
    // W beat.  The delta is the cost of the pass itself, isolated from SD
    // init and the ROM copy, and is what gets extrapolated to wall clock.
    uint64_t zero_first_cyc    = 0;
    uint64_t zero_last_cyc     = 0;

    // ── Fabric-shape model ─────────────────────────────────────────────
    // The default (0 / 0 / false) is the historical always-ready,
    // same-cycle-B slave, so every pre-existing scenario is unchanged.
    //
    // b_latency models the AW->B round trip the real path actually pays
    // (adapter + xbar + L2C + async bridge + MIG).  It is the ONLY reason
    // burst length matters: it is paid once per burst, so cycles/word is
    // roughly 1 + (fixed overhead)/(beats per burst).
    //
    // stall_b_at_aw withholds the B response for one nominated zero-pass
    // burst forever — the fail-closed case.  boot_fsm must then park, never
    // issue another AW, and never raise rom_loaded (which is the CPU
    // release gate, fpga_top_clocks.vh).
    int      b_latency         = 0;
    int      b_delay           = 0;
    uint32_t stall_b_at_aw     = 0;   // 1-based zero AW ordinal; 0 = never
    bool     b_stalled         = false;

    // Longest run of W handshakes on CONSECUTIVE cycles inside the zero
    // region.  1 means the master drops WVALID between every beat, i.e. the
    // burst costs 2 cycles/beat no matter how ready the slave is.
    uint32_t max_b2b_w         = 0;
    uint32_t cur_b2b_w         = 0;
    uint64_t last_w_cyc        = 0;
    bool     had_w             = false;

    // Backing store (sparse)
    std::map<uint32_t, uint32_t> mem;
} axs;

// Expected ROM base per docs/peripheral_arch.md.
static const uint32_t ROM_BASE = 0x40000000u;

static void axi_tick() {
    // Default deasserts for inputs we pulse.
    // We want a simple 1-cycle handshake slave — keep ready high when idle.

    // ── AW channel ─────────────────────────────────────────────────────
    if (!axs.aw_seen) {
        dut->m_axi_awready = 1;
        if (dut->m_axi_awvalid) {
            axs.aw_addr    = dut->m_axi_awaddr;
            axs.aw_len     = dut->m_axi_awlen;
            axs.aw_id      = dut->m_axi_awid;
            axs.aw_count++;
            if (axs.aw_len > axs.max_aw_len) axs.max_aw_len = axs.aw_len;
            axs.aw_seen    = true;
            axs.beats_left = axs.aw_len + 1;
            axs.next_addr  = axs.aw_addr;
            if (axs.aw_addr < ROM_BASE) {
                if (axs.zero_aw_count == 0) axs.zero_first_cyc = sim_time;
                axs.zero_aw_count++;
                if (axs.stall_b_at_aw != 0 &&
                    axs.zero_aw_count == axs.stall_b_at_aw) {
                    axs.b_stalled = true;
                }
                if (axs.err_on_zero_write != 0 &&
                    axs.zero_aw_count == axs.err_on_zero_write) {
                    axs.inject_err_b = true;
                }
            }
        }
    } else {
        dut->m_axi_awready = 0;
    }

    // ── W channel ──────────────────────────────────────────────────────
    if (axs.aw_seen && axs.beats_left > 0) {
        dut->m_axi_wready = 1;
        if (dut->m_axi_wvalid) {
            uint32_t a = axs.next_addr & ~0x3u;
            uint32_t wd = dut->m_axi_wdata;
            uint32_t strb = dut->m_axi_wstrb;

            // Compose word with the given strobes.  Byte at addr+i lives
            // in lane (3 - i) for the project's big-endian convention.
            uint32_t existing = axs.mem.count(a) ? axs.mem[a] : 0;
            uint32_t merged = existing;
            for (int i = 0; i < 4; i++) {
                if (strb & (1u << (3 - i))) {
                    uint32_t b = (wd >> ((3 - i) * 8)) & 0xFFu;
                    uint32_t mask = 0xFFu << ((3 - i) * 8);
                    merged = (merged & ~mask) | (b << ((3 - i) * 8));
                }
            }
            axs.mem[a] = merged;
            if (a < ROM_BASE) {
                axs.zero_w_beats++;
                axs.zero_last_cyc = sim_time;
                if (axs.had_w && sim_time == axs.last_w_cyc + 1) axs.cur_b2b_w++;
                else                                             axs.cur_b2b_w = 1;
                if (axs.cur_b2b_w > axs.max_b2b_w) axs.max_b2b_w = axs.cur_b2b_w;
                axs.last_w_cyc = sim_time;
                axs.had_w      = true;
            }

            axs.next_addr += 4;
            axs.beats_left--;

            if (dut->m_axi_wlast && axs.beats_left == 0) {
                axs.b_pending = true;
                axs.b_delay   = axs.b_latency;
                axs.b_id      = axs.aw_id;
            } else if (dut->m_axi_wlast && axs.beats_left != 0) {
                fprintf(stderr, "[AXI] WLAST early at beat with %d left\n",
                        axs.beats_left);
                n_fail++;
            }
        }
    } else {
        dut->m_axi_wready = 0;
    }

    // ── B channel ──────────────────────────────────────────────────────
    if (axs.b_pending && axs.b_stalled) {
        // Response withheld forever for this burst.
        dut->m_axi_bvalid = 0;
    } else if (axs.b_pending && axs.b_delay > 0) {
        axs.b_delay--;
        dut->m_axi_bvalid = 0;
    } else if (axs.b_pending) {
        dut->m_axi_bvalid = 1;
        dut->m_axi_bresp  = axs.inject_err_b ? 2 : 0;   // 2 = SLVERR
        dut->m_axi_bid    = axs.b_id;
        if (dut->m_axi_bready) {
            axs.b_pending    = false;
            axs.aw_seen      = false;   // ready for next burst
            axs.inject_err_b = false;
        }
    } else {
        dut->m_axi_bvalid = 0;
    }
}

// ═════════════════════════════════════════════════════════════════════
// SD-card software model
// ═════════════════════════════════════════════════════════════════════
//
// Tracks the SPI MOSI byte stream.  On a recognised CMD frame (6 bytes
// starting with a byte of the form 0b01xxxxxx), queues the appropriate
// response bytes into a small FIFO that is drained out on MISO, in reply
// to each subsequent 0xFF the host sends.

struct SdCard {
    // MOSI byte sink
    std::vector<uint8_t> mosi_frame;    // accumulates bytes of current frame
    bool                 in_frame = false;

    // MISO byte source (FIFO of pending responses + data payload)
    std::vector<uint8_t> miso_fifo;

    // State
    bool   initialised  = false;
    int    acmd41_replies = 0;   // number of times ACMD41 has been seen
    bool   next_is_acmd = false; // CMD55 seen, next is ACMD41

    // Test knobs for cold-boot CMD0 flakiness coverage.
    //
    // cmd0_silent_count : N CMD0 frames after which the card stays mute (we
    //                     return 0xFF padding only — no R1 byte) for the
    //                     first `cmd0_silent_count` attempts.  Forces the
    //                     boot_fsm's R1-timeout retry path.
    // cmd0_bad_r1_count : N CMD0 frames return a wrong R1 (0x05 illegal-cmd)
    //                     for the first `cmd0_bad_r1_count` attempts.
    //                     Forces the boot_fsm's wrong-R1 retry path.
    // cmd0_seen         : counter incremented every CMD0 the model receives,
    //                     so the test can assert how many retries happened.
    int cmd0_silent_count = 0;
    int cmd0_bad_r1_count = 0;
    int cmd0_seen         = 0;

    // CMD18 multi-block read: once started, the card streams blocks
    // back-to-back until CMD12 arrives on MOSI.
    bool       multi_read_active = false;
    uint32_t   multi_read_lba    = 0;

    int cmd12_count = 0;

    // CRC fault injection for the CMD18 (multi-block, boot-ROM-load) path
    // — exercises boot_fsm.v's whole-run retry (CTRL_RETRY_MAX /
    // ST_CTRL_RETRY_DRAIN), added alongside it.
    //   corrupt_once    : the very next data block served has its CRC16
    //                     XORed with 0xFFFF, then this clears itself — a
    //                     single bad block on one particular pass.  A
    //                     whole-run retry's replay is naturally clean,
    //                     since it consumes a fresh CMD18 (fresh blocks).
    //   corrupt_forever : every data block ever served is corrupted —
    //                     models a persistently bad card/link, so the
    //                     retry budget should exhaust into ST_ERROR.
    bool corrupt_once    = false;
    bool corrupt_forever = false;

    // When true, CMD6 (SWITCH_FUNC / HS-mode request) is accepted
    // (R1=0x00) instead of the default "illegal command" rejection —
    // exercises the real-HW finding that boot_fsm.v must NOT engage
    // spi_hs_mode even when the card would allow it (see boot_fsm.v's
    // ST_CMD6_CRC_WAIT comment).
    bool csd_v1 = false;   // false = CSD v2.0 (SDHC), true = CSD v1.0 (SDSC)
    bool accept_cmd6 = false;

    void push_response(const std::vector<uint8_t>& v) {
        for (auto b : v) miso_fifo.push_back(b);
    }

    void queue_read_block(uint32_t sec) {
        std::vector<uint8_t> resp;
        resp.push_back(0xFF);
        resp.push_back(0xFE);
        std::vector<uint8_t> data;
        data.reserve(512);
        for (int k = 0; k < 512; k++) {
            uint8_t b = (uint8_t)((sec * 37 + k) & 0xFF);
            data.push_back(b);
            resp.push_back(b);
        }
        uint16_t crc = crc16_ccitt(data);
        if (corrupt_forever || corrupt_once) {
            crc ^= 0xFFFF;
            corrupt_once = false;
        }
        resp.push_back((uint8_t)(crc >> 8));
        resp.push_back((uint8_t)(crc & 0xFF));
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

    // Handle a fully-received 6-byte frame.
    void handle_frame() {
        uint8_t cmdbyte = mosi_frame[0];
        uint8_t cmd = cmdbyte & 0x3F;

        switch (cmd) {
            case 0: {   // CMD0 — GO_IDLE
                cmd0_seen++;
                if (cmd0_silent_count > 0) {
                    // Silent card: send only 0xFF padding (no R1).  The
                    // boot_fsm's R1-poll loop will eventually time out
                    // and retry per the CMD0_RETRY_MAX path.
                    cmd0_silent_count--;
                    // Push some 0xFFs so the host doesn't latch a stale
                    // partial response from a previous frame.
                    push_response({0xFF, 0xFF, 0xFF, 0xFF});
                } else if (cmd0_bad_r1_count > 0) {
                    // Wrong-R1 case: card returned R1 with illegal-cmd
                    // bit set (some power-on transients look like this).
                    cmd0_bad_r1_count--;
                    push_response({0xFF, 0x05});
                } else {
                    push_response({0xFF, 0x01});
                }
                break;
            }
            case 8: {   // CMD8 — SEND_IF_COND
                // R1 = 0x01 then 4-byte echo (we just echo 0x00 0x00 0x01 0xAA)
                push_response({0xFF, 0x01, 0x00, 0x00, 0x01, 0xAA});
                break;
            }
            case 55: {  // CMD55 — APP_CMD
                push_response({0xFF, 0x01});
                next_is_acmd = true;
                break;
            }
            case 41: {  // ACMD41 — SD_SEND_OP_COND
                // After ~2 attempts, return 0x00 (ready).
                if (acmd41_replies >= 1) {
                    push_response({0xFF, 0x00});
                    initialised = true;
                } else {
                    push_response({0xFF, 0x01});
                }
                acmd41_replies++;
                next_is_acmd = false;
                break;
            }
            case 9: {   // CMD9 — SEND_CSD
                // R1=0x00, then a data block: 0xFE + 16 CSD bytes + 2 CRC.
                // CSD v2.0 (CSD_STRUCTURE=1, csd[0][7:6]=0b01 -> 0x40) with
                // C_SIZE = 30719, i.e. (C_SIZE+1) * 1024 = 31,457,280
                // sectors = 16 GiB.  C_SIZE occupies bits 69:48, so it lands
                // in csd[7][5:0] : csd[8] : csd[9] = 0x00 : 0x77 : 0xFF.
                if (csd_v1) {
                    // CSD v1.0 (SDSC).  CSD_STRUCTURE=0, READ_BL_LEN=9,
                    // C_SIZE=2047, C_SIZE_MULT=7 ->
                    //   (C_SIZE+1) << (C_SIZE_MULT+2+READ_BL_LEN-9)
                    //   = 2048 << 9 = 1048576 sectors.
                    // The v1.0 decode path is otherwise UNTESTED — it only
                    // runs on older/smaller cards.
                    push_response({0xFF, 0x00, 0xFF, 0xFE,
                                   0x00, 0x00, 0x00, 0x00,   // csd[0..3]
                                   0x00, 0x09, 0x01, 0xFF,   // csd[4..7]
                                   0xC0, 0x03, 0x80, 0x00,   // csd[8..11]
                                   0x00, 0x00, 0x00, 0x00,   // csd[12..15]
                                   0x00, 0x00});             // CRC16 (unchecked)
                } else {
                push_response({0xFF, 0x00, 0xFF, 0xFE,
                               0x40, 0x0E, 0x00, 0x32,   // csd[0..3]
                               0x5B, 0x59, 0x00, 0x00,   // csd[4..7]
                               0x77, 0xFF, 0x7F, 0x80,   // csd[8..11]
                               0x0A, 0x40, 0x00, 0x00,   // csd[12..15]
                               0x00, 0x00});             // CRC16 (unchecked)
                }
                break;
            }
            case 58: {  // CMD58 — READ_OCR
                // R1 = 0x00, then 4-byte OCR with CCS=1 (SDHC) in bit 30.
                // OCR[31:24] = 0xC0 → bit 30 = 1 (CCS), bit 31 = 1 (pwr done).
                push_response({0xFF, 0x00, 0xC0, 0xFF, 0x80, 0x00});
                break;
            }
            case 59: {  // CMD59 — CRC_ON_OFF: card accepts, enabling
                        // boot_fsm's sd_crc_enabled / sd_ctrl's
                        // crc_check_en for the CMD18 ROM-load path below.
                push_response({0xFF, 0x00});
                break;
            }
            case 6: {   // CMD6 — SWITCH_FUNC
                if (accept_cmd6) {
                    // R1=0x00 (accepted) then the 64-byte status block
                    // (token + 64 dummy bytes + 2 dummy CRC — boot_fsm
                    // only counts these bytes, never validates their
                    // CRC). Real-HW finding 2026-07-23: even when a card
                    // DOES accept CMD6, boot_fsm.v now deliberately never
                    // engages spi_hs_mode (see its ST_CMD6_CRC_WAIT
                    // comment) — this scenario proves that in sim.
                    std::vector<uint8_t> resp = {0xFF, 0x00, 0xFF, 0xFE};
                    for (int k = 0; k < 64; k++) resp.push_back(0x00);
                    resp.push_back(0x00);
                    resp.push_back(0x00);
                    push_response(resp);
                } else {
                    // Returning R1 with bit 2 set (illegal cmd) — any
                    // non-zero but also-non-0x01 forces boot_fsm into the
                    // "fallback to 25 MHz, proceed with CMD17" path.
                    push_response({0xFF, 0x04});
                }
                break;
            }
            case 17: {  // CMD17 — READ_SINGLE_BLOCK
                // Sector number (SDHC: block number).
                uint32_t sec = ((uint32_t)mosi_frame[1] << 24) |
                               ((uint32_t)mosi_frame[2] << 16) |
                               ((uint32_t)mosi_frame[3] <<  8) |
                               ((uint32_t)mosi_frame[4]);
                // R1=0x00, 0xFF padding, 0xFE token, 512 data bytes, CRC16
                std::vector<uint8_t> resp;
                resp.push_back(0xFF);
                resp.push_back(0x00);
                resp.push_back(0xFF);
                resp.push_back(0xFE);
                for (int k = 0; k < 512; k++) {
                    resp.push_back((uint8_t)((sec * 37 + k) & 0xFF));
                }
                resp.push_back(0x00);  // CRC16[15:8]
                resp.push_back(0x00);  // CRC16[7:0]
                push_response(resp);
                break;
            }
            case 18: {  // CMD18 — READ_MULTIPLE_BLOCK
                uint32_t sec = ((uint32_t)mosi_frame[1] << 24) |
                               ((uint32_t)mosi_frame[2] << 16) |
                               ((uint32_t)mosi_frame[3] <<  8) |
                               ((uint32_t)mosi_frame[4]);
                // R1=0x00 then the first block immediately; subsequent
                // blocks are queued on demand from pop_miso() until CMD12.
                push_response({0xFF, 0x00});
                queue_read_block(sec);
                multi_read_active = true;
                multi_read_lba    = sec + 1;
                break;
            }
            case 12: {  // CMD12 — STOP_TRANSMISSION
                cmd12_count++;
                multi_read_active = false;
                // Drain any pending mid-block data and acknowledge.
                miso_fifo.clear();
                push_response({0xFF, 0x00});
                break;
            }
            default: {
                push_response({0xFF, 0x04});
                break;
            }
        }
    }

    // One SPI byte has been shifted out by the host (MOSI).
    void observe_mosi(uint8_t b) {
        if (!in_frame) {
            if ((b & 0xC0) == 0x40) {   // CMD frame start: 01xxxxxx
                mosi_frame.clear();
                mosi_frame.push_back(b);
                in_frame = true;
            }
            // else: dummy/polling byte, ignore.
        } else {
            mosi_frame.push_back(b);
            if (mosi_frame.size() == 6) {
                handle_frame();
                in_frame = false;
            }
        }
    }
} sd;

// Drive the SPI slave pins from the software model each cycle.  We watch
// spi_clk rising edges to shift MOSI into a byte accumulator, and each
// SPI clock we drive MISO from the FIFO's current byte's current bit.
//
// To avoid reimplementing the entire SPI clock/phase machine, we observe
// CS_N deassertion + spi_clk rising/falling edges and treat 8 rising
// edges as a completed byte (mode 0: MOSI valid at rise, MISO driven
// before rise so host samples stable).
struct SpiObserver {
    int      rise_count = 0;
    uint8_t  mosi_byte  = 0;
    uint8_t  miso_byte  = 0xFF;    // the byte currently being clocked out
    int      miso_bits_left = 0;
    bool     last_clk   = false;
    bool     last_cs_n  = true;
} sp;

static void spi_tick() {
    bool clk_now = dut->spi_clk;
    bool cs_n_now = dut->spi_cs_n;

    // On CS re-assertion (falling edge), reset byte alignment.
    if (sp.last_cs_n && !cs_n_now) {
        sp.rise_count = 0;
        sp.mosi_byte  = 0;
    }
    sp.last_cs_n = cs_n_now;

    // When idle or CS high, drive MISO high (idle pulled up).
    if (cs_n_now) {
        dut->spi_miso = 1;
        sp.last_clk   = clk_now;
        return;
    }

    // SPI mode 0: host drives MOSI on falling edge, samples MISO on
    // rising edge.  Device (us): latch MOSI on rising edge, drive MISO
    // on falling edge.
    bool rising  = (!sp.last_clk &&  clk_now);
    bool falling = ( sp.last_clk && !clk_now);

    if (rising) {
        // Shift MOSI bit in (MSB first).
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
        // Drive MSB first.
        dut->spi_miso = (sp.miso_byte >> 7) & 1;
        if (falling) {
            sp.miso_byte       = (sp.miso_byte << 1) | 1;   // shift + fill
            sp.miso_bits_left--;
        }
    } else {
        // Hold MISO between edges (keep current value).
        dut->spi_miso = (sp.miso_byte >> 7) & 1;
    }

    sp.last_clk = clk_now;
}

// ═════════════════════════════════════════════════════════════════════
// Clocking
// ═════════════════════════════════════════════════════════════════════
// One call = one rising edge of clk.  The posedge-generated outputs are
// observed immediately after the posedge; our drivers update combinational
// inputs for the next posedge.
static void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    // Now all posedge NBAs have settled — sample outputs, drive inputs.
    spi_tick();
    axi_tick();
    dut->eval();
    sim_time++;
}

static void reset() {
    dut->rst       = 1;
    // Existing scenarios assume the pre-zero pass runs whenever ZERO_BYTES
    // is non-zero, i.e. the historical always-on behaviour.  The skip path
    // is covered explicitly by test_zero_en_low_skips_pre_zero().
    dut->zero_en   = 1;
    dut->spi_miso  = 1;
    dut->m_axi_awready = 0;
    dut->m_axi_wready  = 0;
    dut->m_axi_bvalid  = 0;
    dut->m_axi_bresp   = 0;
    dut->m_axi_bid     = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

// ═════════════════════════════════════════════════════════════════════
// Scenarios
// ═════════════════════════════════════════════════════════════════════

// Big-endian packer used by boot_fsm: byte at offset +i is in lane (3-i).
static uint32_t pack_be(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) |
           ((uint32_t)b2 <<  8) | ((uint32_t)b3);
}

static bool test_boot_completes() {
    reset();

    // Run for a generous number of cycles.  At HS_HALF=2/FAST_HALF=2
    // overrides in the wrapper we get ~32 core cycles per byte, plus
    // command overhead: roughly 520 bytes per sector × 32 cycles = 16640
    // cycles/sector.  NUM_SECTORS=16 → ~266k cycles + init overhead.
    const uint64_t MAX_CYC = 1500000;
    uint64_t start = sim_time;
    for (uint64_t i = 0; i < MAX_CYC && !dut->rom_loaded && !dut->error; i++) {
        tick();
    }
    uint64_t elapsed = sim_time - start;
    if (dut->error) {
        printf("  FAIL boot error raised (err_cause=%u, last_r1=0x%02x, sector=%u)\n",
               (unsigned)dut->dbg_err_cause, (unsigned)dut->dbg_last_r1,
               (unsigned)dut->dbg_sector);
        return false;
    }
    if (!dut->rom_loaded) {
        printf("  FAIL boot did not finish within %llu cycles (st=%u, sector=%u)\n",
               (unsigned long long)MAX_CYC, (unsigned)dut->dbg_st,
               (unsigned)dut->dbg_sector);
        return false;
    }
    // rom_loading is broadcast through a dedicated FF (rom_loading_q) for
    // post-place fanout reasons; rom_loaded asserts one cycle BEFORE
    // rom_loading_q catches up.  Tick a couple of cycles to let the
    // broadcast FF settle before checking the released level.
    for (int i = 0; i < 4; i++) tick();
    if (dut->rom_loading) {
        printf("  FAIL rom_loading still high after completion\n");
        return false;
    }
    printf("  boot completed in %llu core-clk cycles (NUM_SECTORS=%d)\n",
           (unsigned long long)elapsed, NUM_SECTORS);
    return true;
}

static bool test_axi_contents() {
    uint32_t expected_aw = NUM_SECTORS * 128;
    if (axs.aw_count != expected_aw) {
        printf("  FAIL saw %u AW transactions, expected %u single-beat writes\n",
               axs.aw_count, expected_aw);
        return false;
    }
    if (axs.max_aw_len != 0) {
        printf("  FAIL boot_fsm emitted AWLEN=%u; first-light path requires single-beat writes\n",
               (unsigned)axs.max_aw_len);
        return false;
    }

    // Already-completed boot from the previous scenario; inspect the
    // AXI slave's memory map.  For each sector N, expect 128 × 32-bit
    // words starting at ROM_BASE + N*512 containing the canned pattern.
    for (int sec = 0; sec < NUM_SECTORS; sec++) {
        for (int beat = 0; beat < 128; beat++) {
            uint32_t a = ROM_BASE + sec * 512 + beat * 4;
            int k = beat * 4;
            uint8_t b0 = (uint8_t)((sec * 37 + (k + 0)) & 0xFF);
            uint8_t b1 = (uint8_t)((sec * 37 + (k + 1)) & 0xFF);
            uint8_t b2 = (uint8_t)((sec * 37 + (k + 2)) & 0xFF);
            uint8_t b3 = (uint8_t)((sec * 37 + (k + 3)) & 0xFF);
            uint32_t expected = pack_be(b0, b1, b2, b3);
            if (axs.mem.count(a) == 0) {
                printf("  FAIL sector %d beat %d (addr 0x%08x) not written\n",
                       sec, beat, a);
                return false;
            }
            uint32_t got = axs.mem[a];
            if (got != expected) {
                printf("  FAIL addr 0x%08x: got 0x%08x, expected 0x%08x\n",
                       a, got, expected);
                return false;
            }
        }
    }

    // Also check nothing spilled outside the expected simulated sector span.
    uint32_t end = ROM_BASE + NUM_SECTORS * 512;
    for (auto& kv : axs.mem) {
        if (kv.first < ROM_BASE || kv.first >= end) {
            printf("  FAIL stray write at 0x%08x\n", kv.first);
            return false;
        }
    }
    return true;
}

static bool test_spi_went_fast() {
    // After boot, spi_fast_mode should be high (we fell back off CMD6 HS
    // but still accepted the OCR-phase switch to fast).
    if (!dut->spi_fast_mode) {
        printf("  FAIL spi_fast_mode not latched after boot\n");
        return false;
    }
    // HS mode should NOT be high (our model rejected CMD6).
    if (dut->spi_hs_mode) {
        printf("  FAIL spi_hs_mode high but card rejected CMD6\n");
        return false;
    }
    return true;
}

// ─── Cold-boot flakiness coverage ────────────────────────────────────────
//
// These tests exercise the CMD0 retry path that mitigates the observed
// "1-in-2 cold boots fails" symptom on the live FPGA: cards occasionally
// either (a) stay silent on the first CMD0 (R1-poll times out) or (b)
// return a wrong R1 (e.g. 0x05 illegal-cmd) before settling.  The
// boot_fsm now retries CMD0 up to CMD0_RETRY_MAX (=8) times; a healthy
// card on the second or third attempt should still complete boot.
//
// Each scenario fully resets the SPI/AXI/SD-model state.  We can't share
// the dut across separate boot runs without a workaround (rom_loaded is
// a sticky FF), so the flakiness scenarios run a NEW Vtb instance.

static void reset_world_for_flaky() {
    // Drop dut state.  Re-create the testbench so rom_loaded latches
    // a new fresh-rst high.
    delete dut;
    dut = new Vtb_sd_boot_top;
    sd  = SdCard{};
    sp  = SpiObserver{};
    axs = AxiSlave{};
    sim_time = 0;
    reset();
}

static bool boot_to_completion(uint64_t max_cyc, const char* tag) {
    uint64_t start = sim_time;
    for (uint64_t i = 0; i < max_cyc && !dut->rom_loaded && !dut->error; i++) {
        tick();
    }
    if (dut->error) {
        printf("  FAIL [%s] boot error (err_cause=%u, last_r1=0x%02x, sector=%u)\n",
               tag, (unsigned)dut->dbg_err_cause, (unsigned)dut->dbg_last_r1,
               (unsigned)dut->dbg_sector);
        return false;
    }
    if (!dut->rom_loaded) {
        printf("  FAIL [%s] boot did not finish within %llu cycles (st=%u, sector=%u)\n",
               tag, (unsigned long long)max_cyc, (unsigned)dut->dbg_st,
               (unsigned)dut->dbg_sector);
        return false;
    }
    printf("  [%s] boot completed in %llu cycles, cmd0_seen=%d\n",
           tag, (unsigned long long)(sim_time - start), sd.cmd0_seen);
    return true;
}

static bool test_cmd0_silent_then_recover() {
    reset_world_for_flaky();
    sd.cmd0_silent_count = 2;   // first 2 CMD0 frames return 0xFFs only
    if (!boot_to_completion(2000000, "silent")) return false;
    if (sd.cmd0_seen < 3) {
        printf("  FAIL silent: expected >=3 CMD0 frames (2 silent + 1 ok), saw %d\n",
               sd.cmd0_seen);
        return false;
    }
    return true;
}

static bool test_cmd0_bad_r1_then_recover() {
    reset_world_for_flaky();
    sd.cmd0_bad_r1_count = 3;   // first 3 CMD0 frames return R1=0x05
    if (!boot_to_completion(2000000, "bad_r1")) return false;
    if (sd.cmd0_seen < 4) {
        printf("  FAIL bad_r1: expected >=4 CMD0 frames (3 bad + 1 ok), saw %d\n",
               sd.cmd0_seen);
        return false;
    }
    return true;
}

static bool test_cmd0_retry_exhaustion_errors() {
    reset_world_for_flaky();
    // 9 silent CMD0s — boot_fsm retries up to 8 times then errors out.
    sd.cmd0_silent_count = 9;
    // Run until error or rom_loaded.
    for (uint64_t i = 0; i < 5000000 && !dut->rom_loaded && !dut->error; i++) {
        tick();
    }
    if (dut->rom_loaded) {
        printf("  FAIL retry-exhaust: expected error, got rom_loaded\n");
        return false;
    }
    if (!dut->error) {
        printf("  FAIL retry-exhaust: no error within timeout (cmd0_seen=%d)\n",
               sd.cmd0_seen);
        return false;
    }
    // err_cause==5 is "R1 timeout" — the path we expect when CMD0 retries
    // exhaust on silent cards.
    if (dut->dbg_err_cause != 5) {
        printf("  FAIL retry-exhaust: err_cause=%u, expected 5 (R1 timeout)\n",
               (unsigned)dut->dbg_err_cause);
        return false;
    }
    printf("  [retry-exhaust] saw error err_cause=5 after %d CMD0 frames\n",
           sd.cmd0_seen);
    return true;
}

// ─── CMD18 whole-run CRC retry coverage (boot_fsm.v CTRL_RETRY_MAX) ─────
//
// The card now accepts CMD59, so sd_crc_enabled goes high and boot_fsm's
// own sd_ctrl instance validates every ROM-load block's CRC16.  These
// scenarios exercise the corner the SD-CRC-fix investigation actually
// found on real hardware: a bad block during the one-shot, non-resumable
// CMD18 ROM bulk-load.  Before boot_fsm.v's whole-run retry, ANY such
// failure cleanly halted boot forever (err_cause=3) — exactly the
// real-HW symptom this coverage guards against regressing.

static bool test_cmd18_crc_retry_recovers() {
    reset_world_for_flaky();
    sd.corrupt_once = true;   // exactly one bad block, on the first pass
    if (!boot_to_completion(3000000, "crc-retry")) return false;
    if (!dut->sd_crc_enabled) {
        printf("  FAIL crc-retry: sd_crc_enabled never asserted (model CMD59 path broken?)\n");
        return false;
    }
    if (sd.corrupt_once) {
        printf("  FAIL crc-retry: corrupt_once never consumed (block never served?)\n");
        return false;
    }
    return true;
}

static bool test_cmd18_crc_retry_exhausts() {
    reset_world_for_flaky();
    sd.corrupt_forever = true;   // every attempt's blocks are bad, forever
    for (uint64_t i = 0; i < 10000000 && !dut->rom_loaded && !dut->error; i++) {
        tick();
    }
    if (dut->rom_loaded) {
        printf("  FAIL crc-exhaust: expected error, got rom_loaded\n");
        return false;
    }
    if (!dut->error) {
        printf("  FAIL crc-exhaust: no error within timeout\n");
        return false;
    }
    if (dut->dbg_err_cause != 3) {
        printf("  FAIL crc-exhaust: err_cause=%u, expected 3 (ctrl_error/CRC)\n",
               (unsigned)dut->dbg_err_cause);
        return false;
    }
    if (dut->dbg_ctrl_retry != 5) {
        printf("  FAIL crc-exhaust: dbg_ctrl_retry=%u, expected 5 (CTRL_RETRY_MAX)\n",
               (unsigned)dut->dbg_ctrl_retry);
        return false;
    }
    // Confirm the new raw-CRC telemetry itself is meaningful: the model
    // corrupts the real CRC by XORing 0xFFFF, so calc/recv must differ,
    // and recv must NOT equal the correct value either (rules out the
    // tap wiring silently reading back the computed side twice).
    if (dut->dbg_rd_crc_calc == dut->dbg_rd_crc_recv) {
        printf("  FAIL crc-exhaust: dbg_rd_crc_calc == dbg_rd_crc_recv (0x%04x) — "
               "telemetry not distinguishing computed vs received\n",
               (unsigned)dut->dbg_rd_crc_calc);
        return false;
    }
    uint16_t expected_recv = dut->dbg_rd_crc_calc ^ 0xFFFF;
    if (dut->dbg_rd_crc_recv != expected_recv) {
        printf("  FAIL crc-exhaust: dbg_rd_crc_recv=0x%04x, expected 0x%04x "
               "(calc XOR 0xFFFF, matching the injected corruption)\n",
               (unsigned)dut->dbg_rd_crc_recv, (unsigned)expected_recv);
        return false;
    }
    printf("  [crc-exhaust] saw error err_cause=3, dbg_ctrl_retry=5, "
           "calc=0x%04x recv=0x%04x after retry budget exhausted\n",
           (unsigned)dut->dbg_rd_crc_calc, (unsigned)dut->dbg_rd_crc_recv);
    return true;
}

static bool test_cmd6_accepted_never_engages_hs() {
    reset_world_for_flaky();
    sd.accept_cmd6 = true;   // card WOULD allow the HS switch this time
    if (!boot_to_completion(3000000, "cmd6-accept")) return false;
    if (dut->spi_hs_mode) {
        printf("  FAIL cmd6-accept: spi_hs_mode engaged — real-HW CRC "
               "failures at HS_HALF (50 MHz) must never be reachable again\n");
        return false;
    }
    if (!dut->spi_fast_mode) {
        printf("  FAIL cmd6-accept: spi_fast_mode not set (should stay at 25 MHz)\n");
        return false;
    }
    return true;
}

// The SCSI target's reported disk capacity is derived from this, so a wrong
// or unread CSD silently mis-sizes the disk.  Regression for 2026-07-28,
// where a 700 MB image sat on a target hardcoded to report 512 MB and the
// 188 MB beyond that -- including the HFS Alternate MDB in the volume's
// second-to-last block -- could never be read, hanging the File Manager.

// The CSD v1.0 (SDSC) decode path only runs on older/smaller cards and was
// otherwise completely untested — test_csd_capacity_decoded only exercises
// v2.0 (SDHC).  Both branches are live in boot_fsm.v.
static bool test_csd_v1_capacity_decoded() {
    reset_world_for_flaky();
    sd.csd_v1 = true;
    bool ok = boot_to_completion(3000000, "csd-v1");
    sd.csd_v1 = false;
    if (!ok) return false;
    const uint32_t expected = 1048576u;   // 2048 << 9
    if (dut->card_num_lbas != expected) {
        printf("  FAIL csd-v1: card_num_lbas=%u, expected %u "
               "(CSD v1.0 C_SIZE=2047 MULT=7 RBL=9)\n",
               (unsigned)dut->card_num_lbas, (unsigned)expected);
        return false;
    }
    printf("  [csd-v1] decoded card_num_lbas=%u sectors\n",
           (unsigned)dut->card_num_lbas);
    return true;
}

static bool test_csd_capacity_decoded() {
    reset_world_for_flaky();
    if (!boot_to_completion(3000000, "csd")) return false;
    // Mock card is CSD v2.0 with C_SIZE=30719 -> (30719+1)*1024 sectors.
    const uint32_t expected = 31457280u;
    if (dut->card_num_lbas != expected) {
        printf("  FAIL csd: card_num_lbas=%u, expected %u "
               "(CSD v2.0 C_SIZE=30719 -> 16 GiB)\n",
               (unsigned)dut->card_num_lbas, (unsigned)expected);
        return false;
    }
    printf("  [csd] decoded card_num_lbas=%u sectors (16 GiB)\n",
           (unsigned)dut->card_num_lbas);
    return true;
}

// ─── RAM pre-zero pass (ST_ZERO_AW/W/B) ──────────────────────────────────
//
// Only meaningful in the tb-sd-boot-zero build, where the wrapper is
// elaborated with ZERO_BYTES != 0.  boot_fsm zeroes low RAM after the ROM
// copy and BEFORE rom_loaded (== boot_rom_ready in fpga_top_clocks.vh) lets
// the CPU out of reset, so a zeroing write that fails must be fatal.

static uint32_t expected_zero_writes() { return ZERO_BYTES / 4u; }

// zero_en LOW must skip the pre-zero pass entirely and still boot.
//
// This is the warm-reset path: fpga_top drives zero_en from a flag that is
// cleared by a platform reset (power-on / btn3 / vio-hard-reset) and set by
// the JTAG debug-full reset, so a warm reboot keeps the previous session's
// RAM and saves the ~4 s the pass costs on hardware.
//
// The assertion that matters is not just "it booted" but "it touched NO low
// RAM": a skip that still issued some of the writes would look fine on the
// screen while silently wrecking the RAM-preservation property the warm path
// exists to provide.
static bool test_zero_en_low_skips_pre_zero() {
    reset_world_for_flaky();
    dut->zero_en = 0;                       // survives reset_world_for_flaky
    if (!boot_to_completion(3000000, "zero-skip")) return false;
    if (dut->error) {
        printf("  FAIL zero-skip: error asserted on an all-OKAY run\n");
        return false;
    }
    if (dut->dbg_st != ST_DONE) {
        printf("  FAIL zero-skip: dbg_st=%u, expected ST_DONE (%u)\n",
               (unsigned)dut->dbg_st, ST_DONE);
        return false;
    }
    if (axs.zero_aw_count != 0 || axs.zero_w_beats != 0) {
        printf("  FAIL zero-skip: pass ran anyway (%u AWs, %u W beats) -- "
               "with zero_en low it must issue NONE\n",
               (unsigned)axs.zero_aw_count, (unsigned)axs.zero_w_beats);
        return false;
    }
    // And nothing below ROM_BASE may have been written at all.
    for (uint32_t a = 0; a < ZERO_BYTES; a += 4) {
        if (axs.mem.count(a) != 0) {
            printf("  FAIL zero-skip: RAM word 0x%08x was written despite "
                   "zero_en=0\n", a);
            return false;
        }
    }
    return true;
}

static bool test_zero_pass_completes_clean() {
    reset_world_for_flaky();
    if (!boot_to_completion(3000000, "zero-clean")) return false;
    if (dut->error) {
        printf("  FAIL zero-clean: error asserted on an all-OKAY run\n");
        return false;
    }
    if (dut->dbg_st != ST_DONE) {
        printf("  FAIL zero-clean: dbg_st=%u, expected ST_DONE (%u)\n",
               (unsigned)dut->dbg_st, ST_DONE);
        return false;
    }
    // Coverage is measured in W BEATS: burst-shape independent.  The AW
    // count is ZERO_BYTES/(4*(AWLEN+1)) and legitimately changes whenever
    // boot_fsm's ZERO_AWLEN changes, so asserting on it made this test fail
    // for a correct 16-beat-burst zero pass (2026-08-15).
    if (axs.zero_w_beats != expected_zero_writes()) {
        printf("  FAIL zero-clean: %u zero-pass W beats, expected %u "
               "(ZERO_BYTES=%u)\n",
               (unsigned)axs.zero_w_beats, (unsigned)expected_zero_writes(),
               (unsigned)ZERO_BYTES);
        return false;
    }
    if (axs.zero_aw_count == 0 ||
        axs.zero_w_beats % axs.zero_aw_count != 0) {
        printf("  FAIL zero-clean: %u W beats across %u AWs is not a whole "
               "number of beats per burst\n",
               (unsigned)axs.zero_w_beats, (unsigned)axs.zero_aw_count);
        return false;
    }
    for (uint32_t a = 0; a < ZERO_BYTES; a += 4) {
        if (axs.mem.count(a) == 0) {
            printf("  FAIL zero-clean: RAM word 0x%08x never written\n", a);
            return false;
        }
        if (axs.mem[a] != 0) {
            printf("  FAIL zero-clean: RAM word 0x%08x = 0x%08x, expected 0\n",
                   a, axs.mem[a]);
            return false;
        }
    }
    // ── Burst-shape / streaming assertions ────────────────────────────
    //
    // THE regression this guards (fixed 2026-08-19): ST_ZERO_W re-armed
    // WVALID from `if (!m_axi_wvalid)`, which is never true on a handshake
    // cycle because the deassert above the `case` is non-blocking.  WVALID
    // therefore dropped for one cycle after EVERY beat and the pass ran at
    // exactly 2 cycles/beat regardless of burst length or how ready the
    // slave was.  Nothing in the old scenario noticed: the data landed, the
    // beat count was right, only the time was wrong -- and the time IS the
    // cold boot (4.06 s of a 4.06 s boot on HW).
    //
    // The slave here holds WREADY high for the whole burst, so a healthy
    // master streams the entire burst on consecutive cycles.
    const uint32_t beats_per_burst = axs.zero_w_beats / axs.zero_aw_count;
    const uint32_t want_b2b = beats_per_burst < 4 ? beats_per_burst : 4;
    if (axs.max_b2b_w < want_b2b) {
        printf("  FAIL zero-clean: longest run of back-to-back W beats is %u, "
               "expected >= %u against an always-ready slave -- the master is "
               "dropping WVALID between beats (2 cycles/beat)\n",
               (unsigned)axs.max_b2b_w, (unsigned)want_b2b);
        return false;
    }

    // ── Perf report ───────────────────────────────────────────────────
    // Cycle cost of the pass alone, plus the extrapolation to the real
    // 256 MiB RAM window at the 100 MHz core clock.  This is the number the
    // 4.06 s cold-boot measurement is being optimised against; print it on
    // every run so a regression in the zero pass's throughput is visible
    // without re-deriving it by hand.
    uint64_t zcyc = axs.zero_last_cyc - axs.zero_first_cyc + 1;
    double   cpb  = (double)zcyc / (double)axs.zero_w_beats;
    double   full = cpb * (double)(0x10000000ull / 4ull) / 100e6;
    printf("  [zero-clean] %u words zeroed in %u AW bursts (AWLEN=%u), "
           "rom_loaded with no error\n",
           (unsigned)(axs.zero_w_beats), (unsigned)axs.zero_aw_count,
           (unsigned)axs.max_aw_len);
    printf("  [zero-perf ] %llu cycles, %.3f cycles/word, "
           "%.3f bytes/cycle -> 256 MiB = %.3f s @100 MHz\n",
           (unsigned long long)zcyc, cpb, 4.0 / cpb, full);

    // Budget: one cycle per beat plus a generous fixed per-burst overhead.
    // Fails on the pre-2026-08-19 2.19 cycles/word at any burst shape.
    const double budget = 1.0 + 12.0 / (double)beats_per_burst;
    if (cpb > budget) {
        printf("  FAIL zero-clean: %.3f cycles/word exceeds the %.3f budget "
               "for a %u-beat burst against an always-ready slave\n",
               cpb, budget, (unsigned)beats_per_burst);
        return false;
    }
    return true;
}

// Throughput under a realistic AW->B round trip.
//
// The always-ready slave above measures boot_fsm in isolation; the real
// path (axi_narrow_to_wide -> axi_xbar -> L2C -> async bridge -> MIG) pays
// a fixed latency per BURST, not per beat.  That latency is the entire
// argument for long bursts: it divides by the beat count.  Modelling it
// here keeps the extrapolation to hardware honest, and pins the shape so a
// future shortening of ZERO_AWLEN shows up as a number rather than as a
// slower boot nobody attributes to this file.
static bool test_zero_pass_under_fabric_latency() {
    reset_world_for_flaky();
    const int LAT = 60;              // cycles, last W beat -> BVALID
    axs.b_latency = LAT;
    if (!boot_to_completion(30000000, "zero-lat")) return false;
    if (dut->error) {
        printf("  FAIL zero-lat: error asserted on an all-OKAY run\n");
        return false;
    }
    if (axs.zero_w_beats != expected_zero_writes()) {
        printf("  FAIL zero-lat: %u zero-pass W beats, expected %u\n",
               (unsigned)axs.zero_w_beats, (unsigned)expected_zero_writes());
        return false;
    }
    const uint32_t bpb  = axs.zero_w_beats / axs.zero_aw_count;
    uint64_t       zcyc = axs.zero_last_cyc - axs.zero_first_cyc + 1;
    double         cpb  = (double)zcyc / (double)axs.zero_w_beats;
    double         full = cpb * (double)(0x10000000ull / 4ull) / 100e6;
    printf("  [zero-lat  ] B latency %d cyc, %u-beat bursts: %.3f cycles/word "
           "-> 256 MiB = %.3f s @100 MHz\n", LAT, (unsigned)bpb, cpb, full);
    // One cycle per beat, plus the round trip amortised over the burst,
    // plus slack for the AW/B state hops.
    const double budget = 1.0 + (double)(LAT + 12) / (double)bpb;
    if (cpb > budget) {
        printf("  FAIL zero-lat: %.3f cycles/word exceeds the %.3f budget -- "
               "the per-burst latency is not being amortised\n", cpb, budget);
        return false;
    }
    return true;
}

// Fail-closed on a write that never answers.
//
// Companion to the SLVERR case below.  A slave that simply never returns B
// is the other half of "a failed RAM-zeroing write must not release the
// CPU" (commit ab493b2): there is no error beat to latch, so the only
// correct behaviour is to park forever with rom_loaded low.  What must NOT
// happen is the FSM timing out, giving up on the response, and marching on
// to the next burst -- that would leave a hole in the zeroed region and
// still release the CPU.  With one burst outstanding at a time the check is
// simply "no further AW is ever issued".
static bool test_zero_stuck_bresp_never_releases_cpu() {
    reset_world_for_flaky();
    axs.stall_b_at_aw = 2;           // 2nd zero burst never gets its B

    // Run well past the point where the whole pass would have finished.
    const uint64_t BUDGET = 3000000;
    for (uint64_t i = 0; i < BUDGET; i++) {
        tick();
        if (dut->rom_loaded) {
            printf("  FAIL zero-stuck: rom_loaded asserted while a zeroing "
                   "write was still unanswered -- the CPU would be released "
                   "onto partially-zeroed RAM (st=%u, AWs=%u)\n",
                   (unsigned)dut->dbg_st, (unsigned)axs.zero_aw_count);
            return false;
        }
        if (axs.zero_aw_count > 2) {
            printf("  FAIL zero-stuck: %u zero AWs issued -- the FSM abandoned "
                   "an outstanding write and moved on\n",
                   (unsigned)axs.zero_aw_count);
            return false;
        }
    }
    if (axs.zero_aw_count != 2) {
        printf("  FAIL zero-stuck: %u zero AWs, expected the pass to reach "
               "exactly 2 and stop\n", (unsigned)axs.zero_aw_count);
        return false;
    }
    if (dut->dbg_st != ST_ZERO_B) {
        printf("  FAIL zero-stuck: dbg_st=%u, expected ST_ZERO_B (%u) -- the "
               "FSM should still be waiting on the response\n",
               (unsigned)dut->dbg_st, ST_ZERO_B);
        return false;
    }
    if (dut->error) {
        printf("  FAIL zero-stuck: error asserted with no error beat on the "
               "bus\n");
        return false;
    }
    printf("  [zero-stuck] B withheld on burst 2: parked in ST_ZERO_B for "
           "%llu cycles, no further AW, rom_loaded never asserted\n",
           (unsigned long long)BUDGET);
    return true;
}

// THE regression for task #167.
//
// One zeroing write returns SLVERR.  Before the fix, boot_fsm's non-OKAY
// BRESP handler sat ABOVE `case (st)`, so ST_ZERO_B's unconditional
// `st <= ST_ZERO_AW / ST_DONE` (guarded by the very same
// `m_axi_bready && m_axi_bvalid`) overwrote the ST_ERROR transition:
// last-assignment-wins.  The FSM finished the pass, reached ST_DONE, raised
// rom_loaded, and the CPU came out of reset onto RAM that was never fully
// zeroed — which presents exactly like the already-fixed "stale-RAM Sad
// Mac", i.e. a live bug wearing a closed bug's face.
static bool test_zero_bresp_error_is_fatal() {
    reset_world_for_flaky();
    // 2nd zero-pass BURST returns SLVERR.  Was 8, which only existed while
    // the pass issued one beat per AW; at the production burst shape a
    // 4 KiB ZERO_BYTES is only 4 bursts, so an ordinal near the end silently
    // stopped injecting anything and the test passed vacuously.
    const uint32_t FAIL_AT = 2;
    axs.err_on_zero_write = FAIL_AT;

    // Run until the FSM either errors out or (wrongly) declares the ROM
    // loaded.  Both outcomes assert on the same cycle in the buggy RTL, so
    // the real check is what happens AFTER.
    for (uint64_t i = 0; i < 3000000 && !dut->rom_loaded && !dut->error; i++) {
        tick();
    }
    if (!dut->error) {
        printf("  FAIL zero-bresp: SLVERR never latched error "
               "(rom_loaded=%u, st=%u, zero_writes=%u)\n",
               (unsigned)dut->rom_loaded, (unsigned)dut->dbg_st,
               (unsigned)axs.zero_aw_count);
        return false;
    }
    if (dut->dbg_err_cause != ERR_CAUSE_BRESP) {
        printf("  FAIL zero-bresp: err_cause=%u, expected %u (BRESP)\n",
               (unsigned)dut->dbg_err_cause, ERR_CAUSE_BRESP);
        return false;
    }
    uint32_t writes_at_error = axs.zero_aw_count;

    // (a1) it must have ENTERED the error state on that very beat.  The
    // original bug landed exactly here: `error` latched but `st` was
    // overwritten by ST_ZERO_B's own next-state assignment on the same beat,
    // so dbg_st reads back ST_ZERO_AW (22) and the pass just carries on.
    if (dut->dbg_st != ST_ERROR) {
        printf("  FAIL zero-bresp: error latched but dbg_st=%u, expected "
               "ST_ERROR (%u) — the ST_ERROR transition was overwritten by "
               "a normal-progress next-state assignment on the same B beat\n",
               (unsigned)dut->dbg_st, ST_ERROR);
        return false;
    }

    // (a2) the FSM must STAY IN the error state, and
    // (b) the CPU release gate (rom_loaded) must never fire.
    // Budget generously past the point where the remaining ~ZERO_BYTES/4
    // writes would have completed had the pass carried on.
    const uint64_t SETTLE = 200000;
    for (uint64_t i = 0; i < SETTLE; i++) {
        tick();
        if (dut->rom_loaded) {
            printf("  FAIL zero-bresp: rom_loaded asserted %llu cycles after a "
                   "failed zeroing write — CPU would be released onto "
                   "partially-zeroed RAM (st=%u, zero_writes=%u)\n",
                   (unsigned long long)i, (unsigned)dut->dbg_st,
                   (unsigned)axs.zero_aw_count);
            return false;
        }
        if (dut->dbg_st != ST_ERROR) {
            printf("  FAIL zero-bresp: left ST_ERROR after %llu cycles "
                   "(dbg_st=%u) — error state is not sticky\n",
                   (unsigned long long)i, (unsigned)dut->dbg_st);
            return false;
        }
    }
    if (!dut->error) {
        printf("  FAIL zero-bresp: error deasserted during settle window\n");
        return false;
    }
    // The zeroing pass must have stopped dead, not run to completion.
    if (axs.zero_aw_count != writes_at_error) {
        printf("  FAIL zero-bresp: %u more zero writes issued after the error "
               "(%u -> %u)\n",
               (unsigned)(axs.zero_aw_count - writes_at_error),
               (unsigned)writes_at_error, (unsigned)axs.zero_aw_count);
        return false;
    }
    if (axs.zero_w_beats >= expected_zero_writes()) {
        printf("  FAIL zero-bresp: pass completed all %u words despite the "
               "error\n", (unsigned)axs.zero_w_beats);
        return false;
    }
    printf("  [zero-bresp] SLVERR on zero write #%u -> ST_ERROR "
           "(err_cause=%u), held for %llu cycles, rom_loaded never asserted, "
           "pass stopped at %u/%u writes\n",
           (unsigned)FAIL_AT, ERR_CAUSE_BRESP, (unsigned long long)SETTLE,
           (unsigned)axs.zero_aw_count, (unsigned)expected_zero_writes());
    return true;
}

// ─── ROM-mirror + VRAM-zero (boot_fsm MIRROR_LOW_RAM / VRAM_ZERO_*) ──────
//
// Only meaningful in the tb-sd-boot-mirror build, where the wrapper is
// elaborated with MIRROR_LOW_RAM=1.  Two properties matter, each getting
// its own scenario so a failure names exactly which one broke:
//
//   1. Every word streamed from SD lands BOTH at its native ROM_BASE_ADDR
//      offset (the existing contract, unchanged) AND at the identical
//      relative offset from address 0 -- the dual write that lets the CPU
//      find correct ROM content in RAM at reset with no crossbar redirect.
//   2. The low-RAM zero pass, which runs immediately after, must start
//      AFTER the mirrored image (MIRROR_IMAGE_BYTES) and not before it --
//      otherwise the pass would immediately overwrite the mirror it just
//      wrote with zeros, silently reintroducing the exact bug this feature
//      exists to close.  Checked by requiring the mirror region still hold
//      ROM content post-boot while the region above it reads back zero.
//
// Word pattern reproduced from test_axi_contents(): sector N, byte offset
// k within the sector, byte value (N*37+k)&0xFF, packed big-endian.

static uint32_t expected_rom_word(uint32_t byte_off) {
    uint32_t sec = byte_off / 512u;
    uint32_t k   = byte_off % 512u;
    uint8_t b0 = (uint8_t)((sec * 37 + (k + 0)) & 0xFF);
    uint8_t b1 = (uint8_t)((sec * 37 + (k + 1)) & 0xFF);
    uint8_t b2 = (uint8_t)((sec * 37 + (k + 2)) & 0xFF);
    uint8_t b3 = (uint8_t)((sec * 37 + (k + 3)) & 0xFF);
    return pack_be(b0, b1, b2, b3);
}

static bool test_mirror_dual_write_matches_rom(int response_delay = 0) {
    if (!MIRROR_LOW_RAM) {
        printf("  FAIL mirror-dual: build not elaborated with MIRROR_LOW_RAM=1 "
               "-- see the tb-sd-boot-mirror target\n");
        return false;
    }
    reset_world_for_flaky();
    axs.b_latency = response_delay;
    if (!boot_to_completion(3000000, "mirror-dual")) return false;
    if (dut->error) {
        printf("  FAIL mirror-dual: error asserted on an all-OKAY run\n");
        return false;
    }
    if (dut->dbg_st != ST_DONE) {
        printf("  FAIL mirror-dual: dbg_st=%u, expected ST_DONE (%u)\n",
               (unsigned)dut->dbg_st, ST_DONE);
        return false;
    }
    // The whole streamed image (NUM_SECTORS*512 bytes) must be mirrored;
    // the tb build's MIRROR_IMAGE_BYTES is required to cover exactly that
    // span so this check has full coverage.
    const uint32_t image_bytes = (uint32_t)NUM_SECTORS * 512u;
    if (MIRROR_IMAGE_BYTES < image_bytes) {
        printf("  FAIL mirror-dual: MIRROR_IMAGE_BYTES=0x%08x smaller than the "
               "streamed image (0x%08x) -- test cannot cover the whole ROM\n",
               (unsigned)MIRROR_IMAGE_BYTES, (unsigned)image_bytes);
        return false;
    }
    for (uint32_t off = 0; off < image_bytes; off += 4) {
        uint32_t expected = expected_rom_word(off);
        uint32_t rom_addr = ROM_BASE + off;
        if (axs.mem.count(rom_addr) == 0 || axs.mem[rom_addr] != expected) {
            printf("  FAIL mirror-dual: native ROM word at 0x%08x got 0x%08x, "
                   "expected 0x%08x\n", rom_addr,
                   (unsigned)(axs.mem.count(rom_addr) ? axs.mem[rom_addr] : 0),
                   (unsigned)expected);
            return false;
        }
        if (axs.mem.count(off) == 0 || axs.mem[off] != expected) {
            printf("  FAIL mirror-dual: mirrored word at 0x%08x got 0x%08x, "
                   "expected 0x%08x (native ROM word was correct)\n", off,
                   (unsigned)(axs.mem.count(off) ? axs.mem[off] : 0),
                   (unsigned)expected);
            return false;
        }
    }
    printf("  [mirror-dual] %u bytes mirrored 1:1 against native ROM content "
           "(ROM_BASE=0x%08x, mirror base 0x00000000)\n",
           (unsigned)image_bytes, ROM_BASE);
    return true;
}

static bool test_mirror_fifo_stalled_wrap() {
    // Both halves of each pair must survive while B stalls the reader;
    // the multi-sector image wraps the compact FIFO many times.
    return test_mirror_dual_write_matches_rom(37);
}

static bool test_mirror_and_vram_zero_boundary() {
    if (!MIRROR_LOW_RAM) {
        printf("  FAIL mirror-zero: build not elaborated with MIRROR_LOW_RAM=1 "
               "-- see the tb-sd-boot-mirror target\n");
        return false;
    }
    reset_world_for_flaky();
    if (!boot_to_completion(3000000, "mirror-zero")) return false;
    if (dut->error) {
        printf("  FAIL mirror-zero: error asserted on an all-OKAY run\n");
        return false;
    }
    if (dut->dbg_st != ST_DONE) {
        printf("  FAIL mirror-zero: dbg_st=%u, expected ST_DONE (%u)\n",
               (unsigned)dut->dbg_st, ST_DONE);
        return false;
    }

    // (1) The mirror region must still hold ROM content, NOT zero -- proves
    // the zero pass started at MIRROR_IMAGE_BYTES and did not clobber the
    // mirror it just wrote.
    const uint32_t image_bytes = (uint32_t)NUM_SECTORS * 512u;
    for (uint32_t off = 0; off < image_bytes; off += 4) {
        uint32_t expected = expected_rom_word(off);
        if (axs.mem.count(off) == 0 || axs.mem[off] != expected) {
            printf("  FAIL mirror-zero: mirror word at 0x%08x got 0x%08x, "
                   "expected 0x%08x (ROM content) -- the zero pass appears "
                   "to have overwritten the mirror it was supposed to skip\n",
                   off, (unsigned)(axs.mem.count(off) ? axs.mem[off] : 0),
                   (unsigned)expected);
            return false;
        }
    }

    // (2) Everything from MIRROR_IMAGE_BYTES up to ZERO_BYTES must read
    // back zero -- the low-RAM pass ran, offset correctly past the mirror.
    for (uint32_t a = MIRROR_IMAGE_BYTES; a < ZERO_BYTES; a += 4) {
        if (axs.mem.count(a) == 0) {
            printf("  FAIL mirror-zero: RAM word 0x%08x never written\n", a);
            return false;
        }
        if (axs.mem[a] != 0) {
            printf("  FAIL mirror-zero: RAM word 0x%08x = 0x%08x, expected 0\n",
                   a, axs.mem[a]);
            return false;
        }
    }

    // (3) The VRAM pass, if configured, must also have completed: every
    // word in [VRAM_ZERO_BASE, VRAM_ZERO_BASE+VRAM_ZERO_BYTES) reads zero.
    if (VRAM_ZERO_BYTES != 0) {
        for (uint32_t a = VRAM_ZERO_BASE; a < VRAM_ZERO_BASE + VRAM_ZERO_BYTES;
             a += 4) {
            if (axs.mem.count(a) == 0) {
                printf("  FAIL mirror-zero: VRAM word 0x%08x never written\n", a);
                return false;
            }
            if (axs.mem[a] != 0) {
                printf("  FAIL mirror-zero: VRAM word 0x%08x = 0x%08x, "
                       "expected 0\n", a, axs.mem[a]);
                return false;
            }
        }
    }

    printf("  [mirror-zero] mirror region intact (0x0-0x%08x), low-RAM zeroed "
           "(0x%08x-0x%08x), VRAM zeroed (0x%08x-0x%08x)\n",
           image_bytes, MIRROR_IMAGE_BYTES, ZERO_BYTES, VRAM_ZERO_BASE,
           VRAM_ZERO_BASE + VRAM_ZERO_BYTES);
    return true;
}

#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_sd_boot_top;

    // +zero_pass selects the RAM pre-zero scenarios.  They need the wrapper
    // elaborated with ZERO_BYTES != 0, which perturbs the AW counts the
    // ROM-copy scenarios assert on — hence a separate build/target rather
    // than one binary running both sets.
    if (Verilated::commandArgsPlusMatch("zero_pass")[0] != '\0') {
        if (ZERO_BYTES == 0) {
            printf("+zero_pass requires a build with -GZERO_BYTES != 0 "
                   "(TB_ZERO_BYTES=%u) — see the tb-sd-boot-zero target\n",
                   (unsigned)ZERO_BYTES);
            dut->final();
            delete dut;
            return 1;
        }
        RUN(test_zero_pass_completes_clean);
        RUN(test_zero_pass_under_fabric_latency);
        RUN(test_zero_en_low_skips_pre_zero);
        RUN(test_zero_bresp_error_is_fatal);
        RUN(test_zero_stuck_bresp_never_releases_cpu);
        printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
        dut->final();
        delete dut;
        return (n_fail == 0) ? 0 : 1;
    }

    // +mirror selects the ROM-mirror + VRAM-zero scenarios.  They need the
    // wrapper elaborated with MIRROR_LOW_RAM=1 (which changes the AW/W
    // shape of the whole ROM-copy phase — every word doubles), hence a
    // separate build/target rather than folding into tb-sd-boot or
    // tb-sd-boot-zero.
    if (Verilated::commandArgsPlusMatch("mirror")[0] != '\0') {
        if (!MIRROR_LOW_RAM) {
            printf("+mirror requires a build with -GMIRROR_LOW_RAM=1 -- see "
                   "the tb-sd-boot-mirror target\n");
            dut->final();
            delete dut;
            return 1;
        }
        RUN(test_mirror_dual_write_matches_rom);
        RUN(test_mirror_fifo_stalled_wrap);
        RUN(test_mirror_and_vram_zero_boundary);
        printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
        dut->final();
        delete dut;
        return (n_fail == 0) ? 0 : 1;
    }

    RUN(test_boot_completes);
    RUN(test_axi_contents);
    RUN(test_spi_went_fast);
    RUN(test_cmd0_silent_then_recover);
    RUN(test_cmd0_bad_r1_then_recover);
    RUN(test_cmd0_retry_exhaustion_errors);
    RUN(test_cmd18_crc_retry_recovers);
    RUN(test_cmd18_crc_retry_exhausts);
    RUN(test_cmd6_accepted_never_engages_hs);
    RUN(test_csd_capacity_decoded);
    RUN(test_csd_v1_capacity_decoded);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
