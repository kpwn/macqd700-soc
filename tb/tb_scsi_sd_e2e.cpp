// tb_scsi_sd_e2e.cpp — End-to-end testbench for SCSI → bridge → sd_ctrl →
// sd_spi → SD-card chain (no SD-side mocking).
//
// What it proves:
//   - SCSI READ(6/10) and WRITE(6/10) actually move correct bytes through
//     the real SD path, with the LBA biased through SD_LBA_BIAS=8192.
//   - TEST UNIT READY / INQUIRY / READ CAPACITY work without SD traffic.
//   - The pb_clk ↔ core_clk bridge transports request fields, completion
//     status, and data byte streams faithfully.
//
// Architecture:
//   * Two clocks toggle independently at 1-ns granularity:
//       pb_clk = 50 MHz (period 20 ns, half 10 ns)
//       core_clk = 200 MHz by default, or the production 100 MHz with
//                  SCSI_E2E_CORE100 (both retain 25 MHz SPI SCK)
//   * The 5380 register interface is exercised on pb_clk-aligned writes /
//     reads, replicating the Mac's selection / CDB / DATA / STATUS /
//     MSG_IN sequence.
//   * The SPI pins are observed at the bit level by a host-side SD card
//     model (`SdCard`) that recognises CMD17 (READ_SINGLE_BLOCK) and
//     CMD24 (WRITE_SINGLE_BLOCK).
//
// Build: make tb-scsi-sd-e2e
//
// TWO FRONT ENDS
//   make tb-scsi-sd-e2e      — bare 5380 REQ/ACK (scenarios s1..s10)
//   make tb-scsi-sd-e2e-c96  — NCR 53C96 + DAFB pseudo-DMA, i.e. the
//                              path the Q700 ROM and the 7.5.3 SCSI
//                              Manager actually drive (scenarios c1..)
//
// The C96 build is selected by `SCSI_E2E_C96 (Verilog) / -DSCSI_E2E_C96
// (C++) from the same sources.  It exists because the combination that
// ships — a MULTI-BLOCK WRITE (CMD25 ring) through C96 pseudo-DMA — had
// no byte-exact coverage anywhere in the tree, and the bare-5380 path
// back-pressures its ring correctly while the C96 path did not.

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <deque>
#include <map>
#include <initializer_list>
#include <verilated.h>
#include "Vtb_scsi_sd_e2e.h"
// Scenarios in BOTH builds assert on scsi.v's sim-only write-ring
// observers (cons_wr_over / cons_wr_under) and read the live ring
// occupancy (vh_buf_count) — all `verilator public_flat_rd`.  A
// byte-compare alone says "the card got the wrong data"; these name the
// MECHANISM — producer overran the ring, or consumer drained it empty —
// and let a scenario position itself EXACTLY on the ring-full boundary
// instead of hoping it lands there.
#include "Vtb_scsi_sd_e2e___024root.h"


// ─────────────────────────────────────────────────────────────────────
// Globals
// ─────────────────────────────────────────────────────────────────────
static Vtb_scsi_sd_e2e* dut      = nullptr;
static uint64_t         sim_time = 0;
static int              n_pass   = 0;
static int              n_fail   = 0;

static constexpr uint32_t SD_LBA_BIAS = 8192u;
static constexpr uint32_t RING_FULL   = 512u;

// Live taps into scsi.v (all `verilator public_flat_rd`).
static uint32_t wr_over_count() {
    return dut->rootp->tb_scsi_sd_e2e__DOT__u_scsi__DOT__u_scsi__DOT__cons_wr_over;
}
static uint32_t wr_under_count() {
    return dut->rootp->tb_scsi_sd_e2e__DOT__u_scsi__DOT__u_scsi__DOT__cons_wr_under;
}
static uint32_t ring_count() {
    return dut->rootp->tb_scsi_sd_e2e__DOT__u_scsi__DOT__u_scsi__DOT__vh_buf_count;
}

// Clock state.
static constexpr int PB_HALF_NS = 10;     // 50 MHz pb_clk → half-period 10 ns
#ifdef SCSI_E2E_CORE100
static constexpr int CORE_LOW_NS  = 5;
static constexpr int CORE_HIGH_NS = 5;
#else
static constexpr int CORE_LOW_NS  = 3;
static constexpr int CORE_HIGH_NS = 2;
#endif
static int pb_phase = 0;
static int pb_phase_ns = 0;
static int core_phase = 0;
static int core_phase_ns = 0;
static int initial_pb_offset_ns = 0;

// ─────────────────────────────────────────────────────────────────────
// Host-side SD card model (driven on the SPI pin interface)
//
// Recognises:
//   CMD17 (READ_SINGLE_BLOCK)        — R1=0, 0xFE + 512 + 2 CRC
//   CMD18 (READ_MULTIPLE_BLOCK)      — R1=0, 0xFE + 512 + 2 CRC, N times,
//                                       then waits for CMD12 + R1=0
//   CMD24 (WRITE_SINGLE_BLOCK)       — R1=0, accept 0xFE + 512 + 2 CRC,
//                                       reply 0xE5 + busy bytes
//   CMD25 (WRITE_MULTIPLE_BLOCK)     — R1=0, accept N × (0xFC + 512 + 2 CRC)
//                                       each acked with 0xE5 + busy, end on
//                                       0xFD stop-tran token + busy
//   CMD12 (STOP_TRANSMISSION)        — R1=0 after a stuff byte, ends CMD18
// ─────────────────────────────────────────────────────────────────────
struct SdCard {
    // Backing storage indexed by SD LBA (already pre-biased by scsi.v).
    std::map<uint32_t, std::vector<uint8_t>> sectors;

    // MOSI parsing
    std::vector<uint8_t> mosi_frame;
    bool                 in_frame = false;

    // Read phase state — for CMD18 we keep streaming new blocks until
    // CMD12 arrives.  read_active=true means we have an outstanding
    // CMD18 stream (we keep enqueueing 0xFE+block as soon as the host
    // has clocked all queued bytes out).
    bool     read_active = false;
    // SLOW-CARD KNOB (starvation testing). This model used to top the MISO FIFO
    // up the INSTANT it drained -- an infinitely fast card -- so a blind drain
    // could never outrun the supply and the whole starvation path (withheld
    // beats / c96_shim_rd_starved) went untested end to end. With
    // slow_gap_bytes > 0 the card stalls that many idle (0xFF) byte times before
    // delivering each new block, which is what a real card does.
    int      slow_gap_bytes = 0;
    int      gap_left       = 0;
    long     stall_events   = 0;   // times the card actually withheld a byte
    uint32_t read_lba    = 0;

    // Write phase state
    enum WState { W_IDLE, W_AWAIT_TOKEN, W_RECV_DATA, W_RECV_CRC, W_BUSY };
    WState   wstate         = W_IDLE;
    int      w_data_count   = 0;
    int      w_crc_count    = 0;
    int      w_busy_ticks   = 0;
    uint32_t w_lba          = 0;
    uint8_t  w_expected_cmd = 0;     // 24, 25, or 0
    bool     w_multiblock   = false; // CMD25
    int      cmd24_frames_seen = 0;
    int      cmd25_frames_seen = 0;
    std::vector<uint8_t> w_block_bytes;

    // ── Provider-error injection ─────────────────────────────────────
    // Reject the data-response token for one block of a write, the way a
    // real card does when it will not accept the data.  sd_ctrl.v
    // S_W_RESP_W (:1318-1328) requires xxx0_010_1 to accept, so 0x0B
    // (xxx0_101_1) takes the ERR_DR_BAD -> S_ERROR path and the provider
    // raises done|error MID-STREAM, which is the case the SCSI target's
    // S_DATA_OUT had no handling for at all.
    // Defaults to -1: every pre-existing scenario is unaffected.
    int      w_fail_at_block = -1;   // block index within this SCSI transfer
    int      w_blocks_seen   = 0;

    // MISO byte source
    std::deque<uint8_t> miso_fifo;

    void push(uint8_t b) { miso_fifo.push_back(b); }
    void push(std::initializer_list<uint8_t> v) {
        for (auto b : v) miso_fifo.push_back(b);
    }
    void push(const std::vector<uint8_t>& v) {
        for (auto b : v) miso_fifo.push_back(b);
    }

    // Returns the next MISO byte the card would clock out (0xFF if idle).
    uint8_t pop_miso() {
        // While CMD18 is active and we've drained the queued block, top
        // up with the next block immediately so sd_ctrl never sees a
        // gap longer than the spec's 0xFF token-wait.  CMD12 will
        // clear read_active before this fires for the next block.
        if (read_active && miso_fifo.empty()) {
            if (gap_left > 0) {
                gap_left--;
                stall_events++;
                return 0xFF;          // card not ready yet -- supply stalls
            }
            queue_read_block(read_lba);
            read_lba++;
            gap_left = slow_gap_bytes;
        }
        // In busy phase, we hold MISO low for w_busy_ticks bytes.
        // When busy ends and we're mid-CMD25, return to AWAIT_TOKEN so
        // we recognise the next 0xFC data token (or the 0xFD stop-tran
        // token) instead of waiting for a command frame.
        if (wstate == W_BUSY) {
            if (miso_fifo.empty()) {
                if (w_busy_ticks > 0) {
                    w_busy_ticks--;
                    return 0x00;
                } else {
                    wstate = w_multiblock ? W_AWAIT_TOKEN : W_IDLE;
                }
            }
        }
        if (miso_fifo.empty()) return 0xFF;
        uint8_t b = miso_fifo.front();
        miso_fifo.pop_front();
        return b;
    }

    std::vector<uint8_t> sector_for(uint32_t lba) {
        auto it = sectors.find(lba);
        if (it != sectors.end()) {
            std::vector<uint8_t> d = it->second;
            d.resize(512, 0);
            return d;
        }
        // Default pattern when caller didn't pre-load — use a deterministic
        // value seeded by LBA so we can spot-check.
        std::vector<uint8_t> d(512);
        for (int k = 0; k < 512; k++) d[k] = (uint8_t)((lba * 37 + k) & 0xFF);
        return d;
    }

    void queue_read_block(uint32_t lba) {
        // Pad gap byte + token + 512 data bytes + 2 CRC
        push(0xFF);
        push(0xFE);
        auto data = sector_for(lba);
        uint16_t crc = 0;
        for (auto b : data) {
            crc ^= uint16_t(b) << 8;
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021)
                                     : uint16_t(crc << 1);
        }
        for (auto b : data) push(b);
        push(uint8_t(crc >> 8));
        push(uint8_t(crc));
    }

    void handle_cmd_frame() {
        uint8_t cmdbyte = mosi_frame[0];
        uint8_t cmd     = cmdbyte & 0x3F;
        uint32_t lba = ((uint32_t)mosi_frame[1] << 24) |
                       ((uint32_t)mosi_frame[2] << 16) |
                       ((uint32_t)mosi_frame[3] <<  8) |
                       ((uint32_t)mosi_frame[4]);
        switch (cmd) {
            case 17:    // READ_SINGLE_BLOCK
                push(0xFF);
                push(0x00);                  // R1=OK
                queue_read_block(lba);
                break;
            case 18:    // READ_MULTIPLE_BLOCK
                push(0xFF);
                push(0x00);                  // R1=OK
                queue_read_block(lba);
                read_active = true;
                read_lba    = lba + 1;       // next block to auto-queue
                break;
            case 24:    // WRITE_SINGLE_BLOCK
                cmd24_frames_seen++;
                push(0xFF);
                push(0x00);                  // R1=OK
                wstate         = W_AWAIT_TOKEN;
                w_data_count   = 0;
                w_crc_count    = 0;
                w_lba          = lba;
                w_expected_cmd = 24;
                w_multiblock   = false;
                w_block_bytes.clear();
                break;
            case 25:    // WRITE_MULTIPLE_BLOCK
                cmd25_frames_seen++;
                push(0xFF);
                push(0x00);                  // R1=OK
                wstate         = W_AWAIT_TOKEN;
                w_data_count   = 0;
                w_crc_count    = 0;
                w_lba          = lba;
                w_expected_cmd = 25;
                w_multiblock   = true;
                w_block_bytes.clear();
                break;
            case 12:    // STOP_TRANSMISSION (terminates CMD18 stream)
                read_active = false;
                miso_fifo.clear();
                push(0xFF);                  // stuff byte
                push(0x00);                  // R1=OK
                push(0xFF);                  // post-CMD trailing 0xFF
                break;
            default:
                push(0xFF);
                push(0x04);                  // illegal
                break;
        }
    }

    void observe_mosi(uint8_t b) {
        // Write data phase: consume bytes regardless of frame bit pattern.
        switch (wstate) {
            case W_AWAIT_TOKEN:
                if (w_expected_cmd == 24 && b == 0xFE) {
                    wstate = W_RECV_DATA;
                    w_data_count = 0;
                    w_block_bytes.clear();
                    return;
                }
                if (w_expected_cmd == 25 && b == 0xFC) {
                    wstate = W_RECV_DATA;
                    w_data_count = 0;
                    w_block_bytes.clear();
                    return;
                }
                if (w_expected_cmd == 25 && b == 0xFD) {
                    // Stop-tran token from sd_ctrl ends the CMD25 stream.
                    // The byte shifted in alongside 0xFD was selected before
                    // observe_mosi() saw the completed token.  The next byte
                    // is therefore the first busy poll and must come from
                    // w_busy_ticks, not from a queued idle byte.
                    w_busy_ticks = 4;
                    wstate = W_BUSY;
                    w_expected_cmd = 0;
                    w_multiblock = false;
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
                w_crc_count++;
                if (w_crc_count == 2) {
                    // Queue the data response (0xE5 = data accepted), then
                    // push the card into busy: a few 0x00 bytes followed
                    // by a non-zero "released" indicator.
                    // Injected failure: reject this block instead, and do
                    // NOT store it — the card refused the data.
                    if (w_fail_at_block >= 0 &&
                        w_blocks_seen == w_fail_at_block) {
                        push(0x0B);           // data response: rejected
                    } else {
                        push(0xE5);
                        sectors[w_lba] = w_block_bytes;
                    }
                    w_blocks_seen++;
                    w_busy_ticks = 4;
                    if (w_multiblock) {
                        // Roll into next block of the same CMD25.
                        w_lba++;
                        wstate = W_BUSY;
                    } else {
                        wstate = W_BUSY;
                    }
                }
                return;
            case W_BUSY:
                // Busy poll bytes — ignored.  pop_miso() advances
                // wstate to W_AWAIT_TOKEN (multiblock) or W_IDLE
                // (single-block) once busy_ticks expires.
                return;
            case W_IDLE:
            default:
                break;
        }

        // Normal command-frame reassembly.
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
// SPI pin observer — bit-level, sampled on rising spi_clk
// ─────────────────────────────────────────────────────────────────────
struct SpiObs {
    uint8_t  mosi_byte       = 0;
    int      rise_count      = 0;
    uint8_t  miso_byte       = 0xFF;
    int      miso_bits_left  = 0;
    bool     last_clk        = false;
    bool     last_cs_n       = true;
} spi;

static void spi_tick() {
    bool clk_now  = dut->spi_clk;
    bool cs_n_now = dut->spi_cs_n;

    // CS deassertion resets the per-byte counters.
    if (spi.last_cs_n && !cs_n_now) {
        spi.rise_count     = 0;
        spi.mosi_byte      = 0;
        // Mode 0 requires the first MISO bit to be stable before the
        // first rising SCK edge.  Start with one idle-high byte already
        // launched; later bytes are launched on the preceding falling
        // edge below.
        spi.miso_bits_left = 8;
        spi.miso_byte      = 0xFF;
    }
    spi.last_cs_n = cs_n_now;

    if (cs_n_now) {
        dut->spi_miso = 1;
        spi.last_clk  = clk_now;
        return;
    }

    bool rising  = (!spi.last_clk &&  clk_now);
    bool falling = ( spi.last_clk && !clk_now);

    if (rising) {
        spi.mosi_byte = (spi.mosi_byte << 1) | (dut->spi_mosi & 1);
        spi.rise_count++;
        if (spi.rise_count == 8) {
            sd.observe_mosi(spi.mosi_byte);
            spi.rise_count = 0;
            spi.mosi_byte  = 0;
        }
    }

    if (falling) {
        if (spi.miso_bits_left <= 1) {
            // The previous byte ended on this edge.  Launch the next
            // byte now so bit 7 has a full low half-cycle of setup.
            spi.miso_byte      = sd.pop_miso();
            spi.miso_bits_left = 8;
        } else {
            spi.miso_byte      = (spi.miso_byte << 1) | 1;
            spi.miso_bits_left--;
        }
    }
    dut->spi_miso = (spi.miso_byte >> 7) & 1;

    spi.last_clk = clk_now;
}

// ─────────────────────────────────────────────────────────────────────
// 1-ns dual-clock tick + spi observer
// ─────────────────────────────────────────────────────────────────────
static void tick_1ns() {
    sim_time += 1;

    pb_phase_ns += 1;
    if (pb_phase_ns >= PB_HALF_NS) {
        pb_phase_ns = 0;
        pb_phase ^= 1;
        dut->pb_clk = pb_phase;
    }

    core_phase_ns += 1;
    int target = (core_phase == 0) ? CORE_LOW_NS : CORE_HIGH_NS;
    if (core_phase_ns >= target) {
        core_phase_ns = 0;
        core_phase ^= 1;
        dut->core_clk = core_phase;
    }

    dut->eval();
    spi_tick();
    dut->eval();
}

static bool pb_posedge() {
    static int prev = 0;
    int cur = dut->pb_clk;
    bool edge = (prev == 0) && (cur == 1);
    prev = cur;
    return edge;
}

// Wait for the next pb_clk rising edge (to align register-bus drives).
static void wait_pb_posedge() {
    while (!pb_posedge()) tick_1ns();
}

// Run for `n` pb cycles (used between bus events).
static void run_pb_cycles(int n) {
    for (int i = 0; i < n; i++) {
        wait_pb_posedge();
    }
}

// Run for `ns` ns.
static void run_ns(int ns) {
    for (int i = 0; i < ns; i++) tick_1ns();
}

// ─────────────────────────────────────────────────────────────────────
// 5380 register-bus driver — pb-clock-aligned
// ─────────────────────────────────────────────────────────────────────
static void bus_write(uint16_t addr, uint8_t data) {
    wait_pb_posedge();
    dut->pb_addr  = addr & 0x1ff;
    dut->pb_wdata = data;
    dut->pb_wr    = 1;
    dut->pb_rd    = 0;
    // Hold for one full pb cycle.
    run_ns(PB_HALF_NS * 2);
    dut->pb_wr    = 0;
    dut->pb_wdata = 0;
}

static uint8_t bus_read(uint16_t addr) {
    wait_pb_posedge();
    dut->pb_addr = addr & 0x1ff;
    dut->pb_rd   = 1;
    dut->pb_wr   = 0;
    run_ns(PB_HALF_NS * 2);
    dut->pb_rd   = 0;
    // pb_rdata is registered one cycle after request; give it one extra
    // cycle to settle before sampling.
    run_ns(PB_HALF_NS * 2);
    return dut->pb_rdata & 0xFF;
}

// 5380 register bit constants
static constexpr uint8_t IC_RST = 0x80;
static constexpr uint8_t IC_ACK = 0x10;
static constexpr uint8_t IC_BSY = 0x08;
static constexpr uint8_t IC_SEL = 0x04;
static constexpr uint8_t IC_ATN = 0x02;
static constexpr uint8_t IC_DB  = 0x01;

static constexpr uint8_t SR_BSY = 0x40;
static constexpr uint8_t SR_REQ = 0x20;
static constexpr uint8_t SR_MSG = 0x10;
static constexpr uint8_t SR_CD  = 0x08;
static constexpr uint8_t SR_IO  = 0x04;

// ─────────────────────────────────────────────────────────────────────
// 5380 selection / CDB / DATA / STATUS / MSG_IN helpers
// ─────────────────────────────────────────────────────────────────────
static bool do_select(uint8_t target_id, int timeout = 64) {
    bus_write(0, (uint8_t)(1 << target_id));
    bus_write(1, IC_SEL | IC_DB);
    for (int i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_BSY) return true;
        run_pb_cycles(1);
    }
    return false;
}

static void drop_sel() {
    bus_write(1, IC_DB);
    run_pb_cycles(2);
}

static bool send_cdb_byte(uint8_t b, int timeout = 256) {
    int i;
    for (i = 0; i < timeout; i++) {
        if (bus_read(4) & SR_REQ) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    bus_write(0, b);
    bus_write(1, IC_ACK | IC_DB);
    for (i = 0; i < timeout; i++) {
        if (!(bus_read(4) & SR_REQ)) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    bus_write(1, IC_DB);
    run_pb_cycles(2);
    return true;
}

// Pull one DATA_IN byte. Generous timeout because the SD path requires the
// 6-byte CMD frame + R1 poll + 512 byte read + 2 CRC bytes to flow over
// SPI (each byte ≈ 8 SPI clocks ≈ 32 core cycles for FAST_HALF=4) before
// the first DATA_IN REQ is observable.
static bool recv_in_byte(uint8_t& out, int timeout = 200000) {
    int i;
    for (i = 0; i < timeout; i++) {
        if (bus_read(4) & SR_REQ) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    out = bus_read(0);
    bus_write(1, IC_ACK);
    for (i = 0; i < timeout; i++) {
        if (!(bus_read(4) & SR_REQ)) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    bus_write(1, 0);
    run_pb_cycles(2);
    return true;
}

// Push one DATA_OUT byte.
static bool push_out_byte(uint8_t b, int timeout = 200000) {
    int i;
    for (i = 0; i < timeout; i++) {
        if (bus_read(4) & SR_REQ) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    bus_write(0, b);
    bus_write(1, IC_ACK);
    for (i = 0; i < timeout; i++) {
        if (!(bus_read(4) & SR_REQ)) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    bus_write(1, 0);
    run_pb_cycles(2);
    return true;
}

// Receive STATUS byte + MSG_IN byte and let the bus return to BUS_FREE.
static bool recv_status_and_msg(uint8_t& status, uint8_t& msg,
                                int timeout = 200000) {
    if (!recv_in_byte(status, timeout)) return false;
    if (!recv_in_byte(msg,    timeout)) return false;
    // Wait for BSY to drop (DISCONNECT → BUS_FREE).
    for (int i = 0; i < 200; i++) {
        if (!(bus_read(4) & SR_BSY)) return true;
        run_pb_cycles(1);
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────
// Reset
// ─────────────────────────────────────────────────────────────────────
static void apply_reset() {
    dut->rst       = 1;
    dut->pb_clk    = 0;
    dut->core_clk  = 0;
    dut->pb_addr   = 0;
    dut->pb_wdata  = 0;
    dut->pb_wr     = 0;
    dut->pb_rd     = 0;
    dut->spi_miso  = 1;

    pb_phase = initial_pb_offset_ns / PB_HALF_NS;
    pb_phase_ns = initial_pb_offset_ns % PB_HALF_NS;
    dut->pb_clk = pb_phase;
    core_phase = 0; core_phase_ns = 0;

    // Reset internal state of host-side SD card and SPI observer
    sd = SdCard();
    spi = SpiObs();

    run_ns(200);   // ample for both clocks

    dut->rst = 0;
    run_ns(200);
}

// ─────────────────────────────────────────────────────────────────────
// Pre-loading SD-card backing
// ─────────────────────────────────────────────────────────────────────
static void preload_sd_lba(uint32_t sd_lba, const std::vector<uint8_t>& data) {
    auto& v = sd.sectors[sd_lba];
    v = data;
    v.resize(512, 0);
}

// ─────────────────────────────────────────────────────────────────────
// Test helpers
// ─────────────────────────────────────────────────────────────────────
#define CHECK_TRUE(label, cond) do {                                          \
    if (!(cond)) {                                                            \
        std::printf("  FAIL %s @ t=%llu\n", (label),                          \
                    (unsigned long long)sim_time);                            \
        return false;                                                         \
    }                                                                         \
} while (0)

#define CHECK_EQ(label, got, exp) do {                                        \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp);             \
    if (_g != _e) {                                                           \
        std::printf("  FAIL %s: got 0x%x expected 0x%x @ t=%llu\n", (label),  \
                    _g, _e, (unsigned long long)sim_time);                    \
        return false;                                                         \
    }                                                                         \
} while (0)

// ═════════════════════════════════════════════════════════════════════
// Scenarios
// ═════════════════════════════════════════════════════════════════════

// s1 — TEST UNIT READY (CDB 0x00). No SD traffic, expect GOOD status.
static bool s1_tur() {
    apply_reset();
    CHECK_TRUE("s1: select", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s1: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    uint8_t status, msg;
    CHECK_TRUE("s1: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s1: status GOOD", status, 0x00);
    CHECK_EQ("s1: msg COMPLETE", msg, 0x00);
    CHECK_EQ("s1: no SD lba sent", sd.sectors.size(), 0u);
    return true;
}

// s2 — INQUIRY. 36 bytes returned, no SD traffic.
static bool s2_inquiry() {
    apply_reset();
    CHECK_TRUE("s2: select", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    for (int i = 0; i < 6; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s2: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    uint8_t inq[36];
    for (int i = 0; i < 36; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s2: inq[%d]", i);
        CHECK_TRUE(l, recv_in_byte(inq[i]));
    }
    CHECK_EQ("s2: inq[0] device-type",   inq[0], 0x00);
    CHECK_EQ("s2: inq[2] SCSI version",  inq[2], 0x02);
    CHECK_EQ("s2: inq[4] additional len", inq[4], 31);
    CHECK_EQ("s2: inq[8] vendor", inq[8], 'A');
    CHECK_EQ("s2: inq[16] product", inq[16], 'H');
    uint8_t status, msg;
    CHECK_TRUE("s2: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s2: status GOOD", status, 0x00);
    CHECK_EQ("s2: msg COMPLETE", msg, 0x00);
    CHECK_EQ("s2: no SD lba sent", sd.sectors.size(), 0u);
    return true;
}

// s3 — READ CAPACITY (10). 8 bytes returned, GOOD status.
static bool s3_read_capacity() {
    apply_reset();
    CHECK_TRUE("s3: select", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x25, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s3: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    uint8_t cap[8];
    for (int i = 0; i < 8; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s3: cap[%d]", i);
        CHECK_TRUE(l, recv_in_byte(cap[i]));
    }
    // last LBA = DISK_NUM_LBAS - 1 = 0x000FFFFF (1048575)
    CHECK_EQ("s3: last LBA[31:24]", cap[0], 0x00);
    CHECK_EQ("s3: last LBA[23:16]", cap[1], 0x0F);
    CHECK_EQ("s3: last LBA[15:8]",  cap[2], 0xFF);
    CHECK_EQ("s3: last LBA[7:0]",   cap[3], 0xFF);
    CHECK_EQ("s3: block size hi",   cap[6], 0x02);
    CHECK_EQ("s3: block size lo",   cap[7], 0x00);
    uint8_t status, msg;
    CHECK_TRUE("s3: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s3: status GOOD", status, 0x00);
    return true;
}

// s4 — READ(6) LBA 0, 1 block.  Pre-loaded pattern at SD sector 8192.
static bool s4_read6_lba0_1block() {
    apply_reset();
    std::vector<uint8_t> pattern(512);
    for (int i = 0; i < 512; i++) pattern[i] = (uint8_t)(i ^ 0x5A);
    preload_sd_lba(SD_LBA_BIAS + 0, pattern);

    CHECK_TRUE("s4: select", do_select(0));
    drop_sel();
    // READ(6): op=0x08, lba=0, len=1
    uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s4: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<uint8_t> got(512);
    for (int i = 0; i < 512; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s4: byte[%d]", i);
        CHECK_TRUE(l, recv_in_byte(got[i]));
    }
    for (int i = 0; i < 512; i++) {
        if (got[i] != pattern[i]) {
            std::printf("  FAIL s4: byte %d got=0x%02x want=0x%02x\n",
                        i, got[i], pattern[i]);
            return false;
        }
    }
    uint8_t status, msg;
    CHECK_TRUE("s4: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s4: status GOOD", status, 0x00);
    return true;
}

// s5 — READ(10) LBA 5, 2 blocks.  Pre-loaded patterns at SD sectors
//       8197 and 8198.
static bool s5_read10_lba_5_2blocks() {
    apply_reset();
    std::vector<uint8_t> p1(512), p2(512);
    for (int i = 0; i < 512; i++) {
        p1[i] = (uint8_t)(0xC0 ^ (i * 7));
        p2[i] = (uint8_t)(0x33 + i);
    }
    preload_sd_lba(SD_LBA_BIAS + 5, p1);
    preload_sd_lba(SD_LBA_BIAS + 6, p2);

    CHECK_TRUE("s5: select", do_select(0));
    drop_sel();
    // READ(10): op=0x28, lba=0x00000005, len=0x0002
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x00, 0x05,
                       0x00, 0x00, 0x02, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s5: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<uint8_t> got(1024);
    for (int i = 0; i < 1024; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s5: byte[%d]", i);
        CHECK_TRUE(l, recv_in_byte(got[i]));
    }
    for (int i = 0; i < 512; i++) {
        if (got[i] != p1[i]) {
            std::printf("  FAIL s5: blk0 byte %d got=0x%02x want=0x%02x\n",
                        i, got[i], p1[i]);
            return false;
        }
        if (got[512 + i] != p2[i]) {
            std::printf("  FAIL s5: blk1 byte %d got=0x%02x want=0x%02x\n",
                        i, got[512 + i], p2[i]);
            return false;
        }
    }
    uint8_t status, msg;
    CHECK_TRUE("s5: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s5: status GOOD", status, 0x00);
    return true;
}

// s6 — WRITE(6) LBA 3, 1 block.  Verify SD sector 8195 holds the bytes.
static bool s6_write6_lba_3_1block() {
    apply_reset();
    CHECK_TRUE("s6: select", do_select(0));
    drop_sel();
    // WRITE(6): op=0x0A, lba=3, len=1
    uint8_t cdb[6] = {0x0A, 0x00, 0x00, 0x03, 0x01, 0x00};
    for (int i = 0; i < 6; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s6: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<uint8_t> data(512);
    for (int i = 0; i < 512; i++) data[i] = (uint8_t)(i + 0xA5);
    for (int i = 0; i < 512; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s6: push[%d]", i);
        CHECK_TRUE(l, push_out_byte(data[i]));
    }
    uint8_t status, msg;
    CHECK_TRUE("s6: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s6: status GOOD", status, 0x00);

    auto it = sd.sectors.find(SD_LBA_BIAS + 3);
    CHECK_TRUE("s6: SD captured LBA 8195", it != sd.sectors.end());
    CHECK_EQ("s6: captured length", it->second.size(), 512u);
    for (int i = 0; i < 512; i++) {
        if (it->second[i] != data[i]) {
            std::printf("  FAIL s6: byte %d got=0x%02x want=0x%02x\n",
                        i, it->second[i], data[i]);
            return false;
        }
    }
    return true;
}

// s7 — WRITE(10) LBA 10, 1 block.  Verify SD sector 8202.
static bool s7_write10_lba_10_1block() {
    apply_reset();
    CHECK_TRUE("s7: select", do_select(0));
    drop_sel();
    // WRITE(10): op=0x2A, lba=0x0000000A, len=0x0001
    uint8_t cdb[10] = {0x2A, 0x00, 0x00, 0x00, 0x00, 0x0A,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s7: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<uint8_t> data(512);
    for (int i = 0; i < 512; i++) data[i] = (uint8_t)(0x3D + (i * 5));
    for (int i = 0; i < 512; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s7: push[%d]", i);
        CHECK_TRUE(l, push_out_byte(data[i]));
    }
    uint8_t status, msg;
    CHECK_TRUE("s7: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s7: status GOOD", status, 0x00);

    auto it = sd.sectors.find(SD_LBA_BIAS + 10);
    CHECK_TRUE("s7: SD captured LBA 8202", it != sd.sectors.end());
    for (int i = 0; i < 512; i++) {
        if (it->second[i] != data[i]) {
            std::printf("  FAIL s7: byte %d got=0x%02x want=0x%02x\n",
                        i, it->second[i], data[i]);
            return false;
        }
    }
    return true;
}

// s9 — READ(10) LBA 100, 4 blocks via single CMD18.
//      Pre-loads SD sectors 8292..8295 with distinct patterns and
//      verifies all 4×512=2048 bytes round-trip end-to-end with
//      exactly one CMD18 frame on the wire (one R1, no per-block
//      command headers, single CMD12 tail).
static bool s9_read10_lba_100_4blocks_cmd18() {
    apply_reset();
    std::vector<std::vector<uint8_t>> blks(4, std::vector<uint8_t>(512));
    for (int b = 0; b < 4; b++) {
        for (int i = 0; i < 512; i++) {
            blks[b][i] = (uint8_t)((b << 4) ^ (i * 11) ^ 0x5A);
        }
        preload_sd_lba(SD_LBA_BIAS + 100 + b, blks[b]);
    }
    CHECK_TRUE("s9: select", do_select(0));
    drop_sel();
    // READ(10): op=0x28, lba=100, len=4
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x00, 100,
                       0x00, 0x00, 0x04, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s9: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<uint8_t> got(2048);
    for (int i = 0; i < 2048; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s9: byte[%d]", i);
        CHECK_TRUE(l, recv_in_byte(got[i]));
    }
    for (int b = 0; b < 4; b++) {
        for (int i = 0; i < 512; i++) {
            if (got[b * 512 + i] != blks[b][i]) {
                std::printf("  FAIL s9: blk%d byte%d got=0x%02x want=0x%02x\n",
                            b, i, got[b * 512 + i], blks[b][i]);
                return false;
            }
        }
    }
    uint8_t status, msg;
    CHECK_TRUE("s9: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s9: status GOOD", status, 0x00);
    return true;
}

// s10 — WRITE(10) LBA 200, 4 blocks via single CMD25.
//       Pushes 4×512=2048 bytes of distinct patterns through the
//       initiator, then reads back the SD-side captured sectors
//       8392..8395 to confirm the bytes landed in order.
static bool s10_write10_lba_200_4blocks_cmd25() {
    apply_reset();
    CHECK_TRUE("s10: select", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x2A, 0x00, 0x00, 0x00, 0x00, 200,
                       0x00, 0x00, 0x04, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s10: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<std::vector<uint8_t>> blks(4, std::vector<uint8_t>(512));
    for (int b = 0; b < 4; b++) {
        for (int i = 0; i < 512; i++) {
            blks[b][i] = (uint8_t)((b * 0x37) + (i * 13) + 0x91);
        }
    }
    for (int b = 0; b < 4; b++) {
        for (int i = 0; i < 512; i++) {
            char l[28]; std::snprintf(l, sizeof(l), "s10: push[%d,%d]", b, i);
            CHECK_TRUE(l, push_out_byte(blks[b][i]));
        }
    }
    uint8_t status, msg;
    CHECK_TRUE("s10: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s10: status GOOD", status, 0x00);
    for (int b = 0; b < 4; b++) {
        auto it = sd.sectors.find(SD_LBA_BIAS + 200 + b);
        char lbl[40]; std::snprintf(lbl, sizeof(lbl),
                                    "s10: SD captured LBA %u",
                                    SD_LBA_BIAS + 200 + b);
        CHECK_TRUE(lbl, it != sd.sectors.end());
        CHECK_EQ("s10: captured length", it->second.size(), 512u);
        for (int i = 0; i < 512; i++) {
            if (it->second[i] != blks[b][i]) {
                std::printf("  FAIL s10: blk%d byte%d got=0x%02x want=0x%02x\n",
                            b, i, it->second[i], blks[b][i]);
                return false;
            }
        }
    }
    return true;
}

// s11 — ATN DETOUR WITH THE RING EXACTLY FULL (bare-5380 only).
//
// The initiator may raise ATN mid-DATA_OUT to send a message; for any
// message that is not 0x04 DISCONNECT the target consumes it and goes
// straight back to the data phase (scsi.v S_MSG_OUT, the
// non-DISCONNECT arm).  That resume used to assert `t_req <= 1'b1`
// UNCONDITIONALLY.
//
// That was harmless while there was no producer back-pressure at all —
// the ring was being blown past full continuously anyway.  It stops
// being harmless the moment back-pressure works: parking AT RING_FULL
// is then the ORDINARY steady state whenever the initiator outruns SPI
// (which it does, by ~16x), so an ATN detour is MOST likely to land
// exactly there, and an unconditional re-arm invites a push that
// overwrites a byte the SD provider has not drained.
//
// This scenario therefore does not hope to land on the boundary — it
// WAITS for vh_buf_count to actually reach RING_FULL, then raises ATN.
// Note this is the REACHABLE resume path: ATN needs r_init_cmd[1],
// writable only through the bare-5380 register arm (scsi.v:2170-2174),
// which the C96 decode shadows entirely when TURBOSCSI_C96_EN=1.  Hence
// this lives in the default build, not the C96 one.
//
// HONEST SCOPE — READ THIS BEFORE TRUSTING IT AS PROOF.  This scenario
// does NOT discriminate the two thresholds, and it was sabotage-checked
// to confirm that: reverting scsi.v's gated resume back to an
// unconditional `t_req <= 1'b1` leaves this GREEN.  The measurement it
// prints says why — the ATN detour costs almost exactly ONE SPI
// byte-time, so the ring drains 512 -> 511 while the message byte is
// being handshaked, and the resume then arms onto a ring with exactly
// one slot free.  The overrun needs the ring to still be at 512 at the
// resume cycle, i.e. a provider that drains NOTHING for the whole
// detour (a card in a long busy phase).  So:
//   * what this scenario really buys is coverage of a path that had
//     NONE anywhere in the tree — tb_scsi.cpp's disconnect scenario is
//     DATA_IN only, and the write-side ATN detour was never exercised
//     end-to-end to the SPI pins at all;
//   * the RTL gate it accompanies is DEFENCE IN DEPTH, explicitly
//     unproven-by-test, kept because a one-byte margin is too thin to
//     leave to provider timing.
// The printed ring delta is deliberately left in so a future reader can
// see the margin move if the provider or the bus timing changes.
static bool push_msg_out_byte(uint8_t b, int timeout = 20000) {
    int i;
    for (i = 0; i < timeout; i++) {
        uint8_t st = bus_read(4);
        if ((st & (SR_REQ | SR_MSG | SR_CD)) == (SR_REQ | SR_MSG | SR_CD) &&
            !(st & SR_IO)) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    bus_write(0, b);
    bus_write(1, IC_ACK | IC_ATN);
    for (i = 0; i < timeout; i++) {
        if (!(bus_read(4) & SR_REQ)) break;
        run_pb_cycles(1);
    }
    if (i == timeout) return false;
    bus_write(1, 0);              // drop ACK and ATN together
    run_pb_cycles(2);
    return true;
}

static bool s11_write10_3blocks_atn_detour_at_ring_full() {
    apply_reset();
    CHECK_TRUE("s11: select", do_select(0));
    drop_sel();
    const uint32_t LBA = 320;
    uint8_t cdb[10] = {0x2A, 0x00, 0x00, 0x00, 0x01, 0x40,
                       0x00, 0x00, 0x03, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s11: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<uint8_t> data(3 * 512);
    for (size_t k = 0; k < data.size(); k++)
        data[k] = (uint8_t)((k * 11) ^ (k >> 7) ^ 0x6D);

    bool detoured = false;
    for (size_t k = 0; k < data.size(); k++) {
        // Once the provider is live and the ring has genuinely filled,
        // take the ATN detour exactly once.
        if (!detoured && k > 600) {
            for (int t = 0; t < 200000 && ring_count() < RING_FULL; t++)
                tick_1ns();
            if (ring_count() >= RING_FULL) {
                uint32_t before = ring_count();
                uint64_t t0 = sim_time;
                bus_write(1, IC_ATN);
                CHECK_TRUE("s11: MSG_OUT byte accepted",
                           push_msg_out_byte(0x80));   // IDENTIFY, not 0x04
                uint32_t after = ring_count();
                std::printf("       s11: ring %u -> %u across the ATN detour "
                            "(%llu ns, ~%llu SPI byte-times)\n",
                            before, after,
                            (unsigned long long)(sim_time - t0),
                            (unsigned long long)((sim_time - t0) / 320));
                detoured = true;
            }
        }
        char l[28]; std::snprintf(l, sizeof(l), "s11: push[%zu]", k);
        CHECK_TRUE(l, push_out_byte(data[k]));
    }
    CHECK_TRUE("s11: the ATN detour actually happened at a FULL ring",
               detoured);

    uint8_t status, msg;
    CHECK_TRUE("s11: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s11: status GOOD", status, 0x00);

    if (wr_over_count() != 0) {
        std::printf("  FAIL s11: RING OVERRUN — %u initiator bytes landed on "
                    "a full ring; the ATN-detour resume re-armed REQ onto a "
                    "full ring\n", wr_over_count());
        return false;
    }
    if (wr_under_count() != 0) {
        std::printf("  FAIL s11: RING UNDERFLOW — %u bytes\n",
                    wr_under_count());
        return false;
    }
    for (int b = 0; b < 3; b++) {
        auto it = sd.sectors.find(SD_LBA_BIAS + LBA + b);
        if (it == sd.sectors.end()) {
            std::printf("  FAIL s11: SD never captured LBA %u\n",
                        SD_LBA_BIAS + LBA + b);
            return false;
        }
        for (int i = 0; i < 512; i++) {
            if (it->second[i] != data[b * 512 + i]) {
                std::printf("  FAIL s11: blk%d byte%d got=0x%02x want=0x%02x\n",
                            b, i, it->second[i], data[b * 512 + i]);
                return false;
            }
        }
    }
    return true;
}

// s8 — READ CAPACITY again, after writes, to confirm geometry stable.
static bool s8_read_capacity_after_writes() {
    apply_reset();
    CHECK_TRUE("s8: select", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x25, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s8: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    uint8_t cap[8];
    for (int i = 0; i < 8; i++) {
        char l[24]; std::snprintf(l, sizeof(l), "s8: cap[%d]", i);
        CHECK_TRUE(l, recv_in_byte(cap[i]));
    }
    CHECK_EQ("s8: last LBA hi", cap[1], 0x0F);
    CHECK_EQ("s8: last LBA lo", cap[3], 0xFF);
    CHECK_EQ("s8: block hi",    cap[6], 0x02);
    uint8_t status, msg;
    CHECK_TRUE("s8: status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("s8: status GOOD", status, 0x00);
    return true;
}

#ifdef SCSI_E2E_C96
// ═════════════════════════════════════════════════════════════════════
// NCR 53C96 + DAFB pseudo-DMA front end (`SCSI_E2E_C96)
//
// Everything above drives the bare 5380 REQ/ACK protocol.  The Mac does
// not: the Q700 ROM and the 7.5.3 SCSI Manager talk to the 53C96
// register file and move payload through the DAFB pseudo-DMA shim at
// pb_addr 0x100 (scsi.v's `pb_dma_shim`).  The register sequences below
// are the ones tb/tb_scsi_c96_read6.cpp already established against
// MAME's ncr53c90.cpp; what is NEW here is running them all the way
// down to the SPI pins with a real SD-card model, and in particular
// running a MULTI-BLOCK WRITE — the CMD25 ring — through them.
//
// FLOW CONTROL IS THE POINT OF THIS HARNESS.  The DAFB shim spins on the
// chip's DRQ line before every pseudo-DMA beat (ROM 0x40898ea8; RTL
// mirror: scsi.v's dma_wr_ready / peripheral_bus.v's write pulse gate),
// so `c96_wait_drq()` below is not testbench politeness — it is the
// initiator-side half of the flow-control contract.  A producer that
// pushes without it drops bytes on the floor instead of exercising
// back-pressure, which would make these tests measure nothing.
// ═════════════════════════════════════════════════════════════════════

// 53C9x command opcodes / status bits (MAME ncr53c90.h).
static constexpr uint8_t CM_RESET      = 0x02;
static constexpr uint8_t CD_SELECT_ATN = 0x41;
static constexpr uint8_t CI_XFER       = 0x10;
static constexpr uint8_t CI_COMPLETE   = 0x11;
static constexpr uint8_t CI_MSG_ACCEPT = 0x12;
static constexpr uint8_t C96_INTR      = 0x80;
static constexpr uint8_t I_FUNCTION    = 0x08;
static constexpr uint8_t I_BUS         = 0x10;
static constexpr uint8_t I_DISCONNECT  = 0x20;

static uint8_t reg_r(uint8_t off)              { return bus_read(off & 0xF); }
static void    reg_w(uint8_t off, uint8_t v)   { bus_write(off & 0xF, v); }
static uint8_t shim_r()                        { return bus_read(0x100); }
static void    shim_w(uint8_t v)               { bus_write(0x100, v); }

// Poll the INTR mirror in the status register (offset 4).  Each reg_r is
// ~2 pb periods, so `spins` * 40 ns bounds the wait.
static bool c96_wait_intr(int spins = 200000) {
    for (int i = 0; i < spins; i++) {
        if (reg_r(0x4) & C96_INTR) return true;
    }
    return false;
}

// Wait for the chip to raise DRQ.  This is the DAFB shim's own gate.
static bool c96_wait_drq(int ns_budget = 4000000) {
    for (int i = 0; i < ns_budget; i++) {
        if (dut->drq) return true;
        tick_1ns();
    }
    return false;
}

// Wait for the back-end to park in a given SCSI phase (status[2:0]).
static bool c96_wait_phase(uint8_t want, int spins = 200000) {
    for (int i = 0; i < spins; i++) {
        if ((reg_r(0x4) & 0x07) == want) return true;
    }
    return false;
}

// Select the target with `n` CDB bytes staged in the FIFO (the non-DMA
// CD_SELECT_ATN form — MAME start_command()/select path).
static bool c96_select_with_cdb(const uint8_t* cdb, int n) {
    reg_w(0x3, CM_RESET);
    reg_w(0x4, 0x00);          // select bus id = TARGET_ID (0 here)
    reg_w(0x5, 0xa7);          // select timeout
    for (int i = 0; i < n; i++) reg_w(0x2, cdb[i]);
    reg_w(0x3, CD_SELECT_ATN);
    if (!c96_wait_intr(200000)) {
        std::printf("  FAIL c96: select never completed (status=0x%02x)\n",
                    reg_r(0x4));
        return false;
    }
    uint8_t ist = reg_r(0x5);
    if ((ist & (I_FUNCTION | I_BUS)) != (I_FUNCTION | I_BUS)) {
        std::printf("  FAIL c96: select istatus=0x%02x (want I_FUNCTION|I_BUS)\n",
                    ist);
        return false;
    }
    return true;
}

// Close a transaction that has reached STATUS: CI_COMPLETE pulls
// status+msg into the FIFO, CI_MSG_ACCEPT frees the bus.
static bool c96_finish(uint8_t& status, uint8_t& msg) {
    if (!c96_wait_phase(0x03)) {
        std::printf("  FAIL c96: never reached STATUS (status=0x%02x)\n",
                    reg_r(0x4));
        return false;
    }
    reg_w(0x3, CI_COMPLETE);
    (void)reg_r(0x5);
    status = reg_r(0x2);
    msg    = reg_r(0x2);
    reg_w(0x3, CI_MSG_ACCEPT);
    (void)reg_r(0x5);
    return true;
}

// Push one DMA|CI_XFER chunk of `n` bytes out through the pseudo-DMA
// shim, DRQ-gated.  `stall_at` (>=0) inserts a producer stall of
// `stall_ns` nanoseconds AFTER that many bytes of this chunk have been
// pushed — the "an interrupt pre-empted the Mac's pseudo-DMA loop"
// case.  `wait_done` false skips the chunk-completion wait, which the
// caller must do for the FINAL chunk of a multi-block write: tcounter
// reaches 0 on the same beat that moves the FSM out of S_DATA_OUT, so
// that chunk's I_BUS only arrives via the phase-change hook once the
// whole SD write has completed.
static bool c96_dma_out_chunk(const uint8_t* buf, int n,
                              bool wait_done,
                              int stall_at = -1, int stall_ns = 0) {
    reg_w(0x0, (uint8_t)(n & 0xFF));
    reg_w(0x1, (uint8_t)((n >> 8) & 0xFF));
    reg_w(0x3, 0x80 | CI_XFER);
    for (int i = 0; i < n; i++) {
        if (!c96_wait_drq()) {
            std::printf("  FAIL c96: DRQ never rose for out byte %d "
                        "(status=0x%02x)\n", i, reg_r(0x4));
            return false;
        }
        shim_w(buf[i]);
        if (i == stall_at && stall_ns > 0) run_ns(stall_ns);
    }
    if (!wait_done) return true;
    if (!c96_wait_intr()) {
        std::printf("  FAIL c96: DMA out chunk never completed "
                    "(status=0x%02x)\n", reg_r(0x4));
        return false;
    }
    (void)reg_r(0x5);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// Shared multi-block WRITE(10) driver.
//
//   blocks     — SCSI block count.  The production-safe provider keeps one
//                continuous caller stream but emits one closed CMD24 per
//                block; provisioning alone is allowed to use CMD25.
//   chunk      — bytes per DMA|CI_XFER, i.e. how the driver slices the
//                payload.  16 is what the Q700 ROM uses (0x40899322);
//                values that do NOT divide 512 deliberately straddle
//                block boundaries and the 512-byte ring wrap.
//   stall_byte — transfer-relative byte index after which to stall the
//                producer for `stall_ns`, or -1 for full tilt.
// ─────────────────────────────────────────────────────────────────────
static bool c96_write10_multi(uint32_t lba, int blocks, int chunk,
                              int stall_byte, int stall_ns,
                              const char* tag, bool reset_first = true) {
    if (reset_first) apply_reset();
    const int cmd24_before = sd.cmd24_frames_seen;
    const int cmd25_before = sd.cmd25_frames_seen;
    const int total = blocks * 512;
    std::vector<uint8_t> payload(total);
    for (int i = 0; i < total; i++) {
        payload[i] = (uint8_t)((i * 13) ^ (i >> 8) ^ (lba & 0xFF) ^ 0x91);
    }
    const uint8_t cdb[10] = {0x2A, 0x00,
                             (uint8_t)((lba >> 24) & 0xFF),
                             (uint8_t)((lba >> 16) & 0xFF),
                             (uint8_t)((lba >>  8) & 0xFF),
                             (uint8_t)( lba        & 0xFF),
                             0x00,
                             (uint8_t)((blocks >> 8) & 0xFF),
                             (uint8_t)( blocks       & 0xFF),
                             0x00};
    if (!c96_select_with_cdb(cdb, 10)) return false;
    CHECK_EQ("c96: post-select phase = DATA OUT", reg_r(0x4) & 0x07, 0x00);

    for (int off = 0; off < total; off += chunk) {
        int n = (total - off < chunk) ? (total - off) : chunk;
        bool last = (off + n >= total);
        int  s_at = -1;
        if (stall_byte >= off && stall_byte < off + n) s_at = stall_byte - off;
        if (!c96_dma_out_chunk(&payload[off], n, !last, s_at, stall_ns)) {
            std::printf("  FAIL %s: chunk at offset %d\n", tag, off);
            return false;
        }
    }

    uint8_t status = 0xFF, msg = 0xFF;
    if (!c96_finish(status, msg)) return false;

    bool ok = true;
    if (sd.cmd24_frames_seen - cmd24_before != blocks) {
        std::printf("  FAIL %s: saw %d CMD24 frames, want %d\n", tag,
                    sd.cmd24_frames_seen - cmd24_before, blocks);
        ok = false;
    }
    if (sd.cmd25_frames_seen != cmd25_before) {
        std::printf("  FAIL %s: SCSI-safe path emitted %d CMD25 frames\n", tag,
                    sd.cmd25_frames_seen - cmd25_before);
        ok = false;
    }
    // Reported, not returned on: a non-GOOD status here is informative
    // (it means scsi.v's ring-underflow escalation fired) but it must not
    // hide the byte-compare and mechanism evidence below.
    if (status != 0x00 || msg != 0x00) {
        std::printf("  FAIL %s: status=0x%02x msg=0x%02x (want GOOD/COMPLETE)"
                    "%s\n", tag, status, msg,
                    status == 0x02
                        ? "  <-- CHECK CONDITION: scsi.v escalated a ring"
                          " underflow instead of silently returning GOOD"
                        : "");
        ok = false;
    }

    // ── Byte-exactness at the SPI pins ────────────────────────────────
    // This is the thing that matters: what the CARD ended up holding.
    int bad_bytes = 0, first_bad_blk = -1, first_bad_off = -1;
    uint8_t first_bad_got = 0, first_bad_want = 0;
    for (int b = 0; b < blocks; b++) {
        auto it = sd.sectors.find(SD_LBA_BIAS + lba + b);
        if (it == sd.sectors.end()) {
            std::printf("  FAIL %s: SD never captured LBA %u\n",
                        tag, SD_LBA_BIAS + lba + b);
            ok = false;
            continue;
        }
        if (it->second.size() != 512u) {
            std::printf("  FAIL %s: blk%d captured %zu bytes, want 512\n",
                        tag, b, it->second.size());
            ok = false;
            continue;
        }
        for (int i = 0; i < 512; i++) {
            if (it->second[i] != payload[b * 512 + i]) {
                if (first_bad_blk < 0) {
                    first_bad_blk  = b;
                    first_bad_off  = i;
                    first_bad_got  = it->second[i];
                    first_bad_want = payload[b * 512 + i];
                }
                bad_bytes++;
            }
        }
    }
    if (bad_bytes) {
        std::printf("  FAIL %s: SILENT DATA CORRUPTION — %d of %d bytes wrong "
                    "at the card (first: blk%d byte%d got=0x%02x want=0x%02x), "
                    "SCSI status was GOOD\n",
                    tag, bad_bytes, total, first_bad_blk, first_bad_off,
                    first_bad_got, first_bad_want);
        ok = false;
    }

    // ── The mechanism assertions ──────────────────────────────────────
    // Worth more than the byte compare alone: they name WHICH half of
    // the ring's flow control failed.  Reported even when the bytes came
    // out right, because "correct by luck of timing" is not correct.
    if (wr_over_count() != 0) {
        std::printf("  FAIL %s: RING OVERRUN — %u initiator bytes landed on "
                    "a full ring (producer back-pressure dead: scsi.v "
                    "S_DATA_OUT guards cannot deassert t_req)\n",
                    tag, wr_over_count());
        ok = false;
    }
    if (wr_under_count() != 0) {
        std::printf("  FAIL %s: RING UNDERFLOW — %u provider bytes taken "
                    "from an empty ring (consumer back-pressure dead: "
                    "sd_ctrl S_W_DATA_S issues bytes unconditionally)\n",
                    tag, wr_under_count());
        ok = false;
    }
    return ok;
}

// ─────────────────────────────────────────────────────────────────────
// Provider fails MID-STREAM during a multi-block WRITE.
//
// S_DATA_IN has had a mid-stream provider-error escape since the
// multi-block read ring landed (scsi.v:3087, `(vh_multi_read) && vh_done
// && vh_error` -> CHECK CONDITION / MEDIUM ERROR / STATUS).  S_DATA_OUT
// had NOTHING: between its case arm and S_VH_WAIT_WR there was no
// vh_done/vh_error handling at all.
//
// That was survivable only by accident while producer back-pressure was
// broken: t_req stayed stuck high, so the initiator kept pushing, drained
// xfer_bytes_left, fell into S_VH_WAIT_WR and terminated on
// VH_WAIT_TIMEOUT.  Corrupt, but terminated.  Once REQ is correctly
// deasserted on a full ring, a provider that stops draining freezes
// vh_buf_count at RING_FULL, the S_DATA_OUT re-arm clause
// (vh_buf_count < RING_FULL) can never fire, and REQ parks low FOREVER —
// a wedged SCSI bus, which takes the machine down with no diagnosis.
// The SCSI sd_ctrl runs with REQ_WDOG_ENABLE(0) (fpga_top_sd.vh), so
// there is no watchdog to rescue it either.
//
// This scenario must therefore assert TERMINATION first and payload
// second, and it must BOUND its own wait: a test that spins forever on a
// wedge hangs the suite instead of failing it.
//
// STATUS 2026-08-19.  The escape described above EXISTS and works
// (rtl/mac/scsi.v, the `(vh_multi_write) && vh_done && vh_error` arm at
// the end of the S_DATA_OUT case).  Between 2026-08-08 and 2026-08-19
// c16/c17 nevertheless reported a WEDGE — but at BYTE 16, one 16-byte
// chunk in, long before the injected provider error at block 5.  That
// was this driver loop failing to read register 5 between chunks; see
// the comment at the istatus read below.  Sabotage-verified after the
// fix: forcing that S_DATA_OUT arm false reproduces the ORIGINAL red
// signature exactly (c16 at byte 3584 of 20480, c17 at byte 1024 of
// 1536), so these two scenarios really are the escape's only coverage
// and they are live.  Do not conclude from a byte-16 WEDGE that the
// escape is missing.
// ─────────────────────────────────────────────────────────────────────
static bool c96_write10_provider_fails(uint32_t lba, int blocks, int chunk,
                                       int fail_at_block, const char* tag) {
    apply_reset();
    sd.w_fail_at_block = fail_at_block;

    const int total = blocks * 512;
    std::vector<uint8_t> payload(total);
    for (int i = 0; i < total; i++)
        payload[i] = (uint8_t)((i * 13) ^ (i >> 8) ^ (lba & 0xFF) ^ 0x91);

    const uint8_t cdb[10] = {0x2A, 0x00,
                             (uint8_t)((lba >> 24) & 0xFF),
                             (uint8_t)((lba >> 16) & 0xFF),
                             (uint8_t)((lba >>  8) & 0xFF),
                             (uint8_t)( lba        & 0xFF),
                             0x00,
                             (uint8_t)((blocks >> 8) & 0xFF),
                             (uint8_t)( blocks       & 0xFF),
                             0x00};
    if (!c96_select_with_cdb(cdb, 10)) return false;

    // Push until either all bytes are gone or the target abandons the
    // data phase (which is the CORRECT response to a dead provider).
    // Every wait is bounded and reports a WEDGE rather than spinning.
    for (int off = 0; off < total; off += chunk) {
        int n = (total - off < chunk) ? (total - off) : chunk;
        reg_w(0x0, (uint8_t)(n & 0xFF));
        reg_w(0x1, (uint8_t)((n >> 8) & 0xFF));
        reg_w(0x3, 0x80 | CI_XFER);
        bool left_data_out = false;
        for (int i = 0; i < n; i++) {
            // Bounded DRQ wait.  Leaving DATA OUT is a legitimate exit.
            bool got = false;
            for (int t = 0; t < 4000000; t++) {
                if (dut->drq) { got = true; break; }
                tick_1ns();
            }
            if (!got) {
                uint8_t st4 = reg_r(0x4);
                if ((st4 & 0x07) != 0x00) { left_data_out = true; break; }
                std::printf("  FAIL %s: WEDGE — DRQ never rose for byte %d "
                            "of %d, target still parked in DATA OUT "
                            "(status=0x%02x fifo=0x%02x).  REQ is deasserted "
                            "with the ring full and nothing can re-arm it: "
                            "the provider stopped draining and S_DATA_OUT "
                            "has no vh_done/vh_error escape.\n",
                            tag, off + i, total, st4, reg_r(0x7));
                return false;
            }
            shim_w(payload[off + i]);
        }
        if (left_data_out) break;
        // ── Consume the chunk-completion interrupt ────────────────────
        // MANDATORY between chunks, and its absence is what used to make
        // this scenario "wedge" at byte 16 — one whole chunk in, long
        // before the injected provider error.  Per MAME 0.285
        // ncr53c90.cpp, function_complete() does NOT clear command_pos,
        // command_w() therefore parks the next Transfer Information in
        // command[1], and ONLY istatus_r() (register 5) runs
        // command_pop_and_chain() to start it (:1103-1121, :907-916).
        // scsi.v models that queue faithfully (c96_cmd_q1 + the
        // pop-and-chain arm at scsi.v:2321-2330), so a driver that never
        // reads register 5 stalls the chip by design — exactly as it
        // would on real silicon.  c96_dma_out_chunk() has always done
        // this; this hand-rolled loop did not, which is why the two
        // provider-error scenarios reported a WEDGE that had nothing to
        // do with the provider.
        if (!c96_wait_intr(400000)) {
            std::printf("  FAIL %s: chunk at offset %d never raised the "
                        "completion interrupt (status=0x%02x)\n",
                        tag, off, reg_r(0x4));
            return false;
        }
        (void)reg_r(0x5);
        if ((reg_r(0x4) & 0x07) != 0x00) break;    // target moved on
    }

    // The command MUST terminate, and it must terminate visibly.
    if (!c96_wait_phase(0x03, 400000)) {
        std::printf("  FAIL %s: WEDGE — target never reached STATUS after a "
                    "provider failure (status=0x%02x).  The transfer is "
                    "unrecoverable and the SCSI bus is hung.\n",
                    tag, reg_r(0x4));
        return false;
    }
    reg_w(0x3, CI_COMPLETE);
    (void)reg_r(0x5);
    uint8_t status = reg_r(0x2);
    uint8_t msg    = reg_r(0x2);
    reg_w(0x3, CI_MSG_ACCEPT);
    (void)reg_r(0x5);

    bool ok = true;
    if (status != 0x02) {
        std::printf("  FAIL %s: provider rejected block %d but the target "
                    "reported status=0x%02x — a failed write MUST surface as "
                    "CHECK CONDITION, not GOOD\n", tag, fail_at_block, status);
        ok = false;
    }
    if (msg != 0x00) {
        std::printf("  FAIL %s: msg=0x%02x (want COMPLETE)\n", tag, msg);
        ok = false;
    }
    // The ring must not have been overrun on the way out either.
    if (wr_over_count() != 0) {
        std::printf("  FAIL %s: RING OVERRUN — %u bytes landed on a full "
                    "ring while unwinding the error\n", tag, wr_over_count());
        ok = false;
    }
    return ok;
}

// c16 — the shape that matters: the 40-block WRITE(10) at LBA 87595 from
//       the driver census, with the card rejecting block 5 mid-stream.
static bool c16_c96_write10_40blocks_provider_fails_midstream() {
    return c96_write10_provider_fails(87595, 40, 16, 5, "c16");
}

// c17 — the small multi-block case, failing on the very first block the
//       provider is asked to take.
static bool c17_c96_write10_3blocks_provider_fails_first_block() {
    return c96_write10_provider_fails(300, 3, 16, 0, "c17");
}

// c1 — POSITIVE CONTROL.  READ(10) LBA 300, 2 blocks, driven through the
//      identical C96 + pseudo-DMA front end.  If this fails, the harness
//      cannot see the target at all and every write result below is
//      meaningless.
static bool c96_read10_measured(int blocks, bool reset = true) {
    if (reset) apply_reset();
    std::vector<std::vector<uint8_t>> blks(blocks, std::vector<uint8_t>(512));
    for (int b = 0; b < blocks; b++) {
        for (int i = 0; i < 512; i++)
            blks[b][i] = (uint8_t)((b << 5) ^ (i * 7) ^ 0x3C);
        preload_sd_lba(SD_LBA_BIAS + 300 + b, blks[b]);
    }
    const uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x01, 0x2C,
                             0x00, uint8_t(blocks >> 8), uint8_t(blocks), 0x00};
    const uint64_t start = sim_time;
    uint64_t first_byte = 0;
    CHECK_TRUE("c1: select", c96_select_with_cdb(cdb, 10));
    CHECK_EQ("c1: post-select phase = DATA IN", reg_r(0x4) & 0x07, 0x01);

    std::vector<uint8_t> got(blocks * 512, 0);
    for (int off = 0; off < blocks * 512; off += 16) {
        reg_w(0x0, 0x10);
        reg_w(0x1, 0x00);
        reg_w(0x3, 0x80 | CI_XFER);
        for (int i = 0; i < 16; i++) {
            CHECK_TRUE("c1: DRQ for in byte", c96_wait_drq());
            got[off + i] = shim_r();
            if (off == 0 && i == 0) first_byte = sim_time;
        }
        CHECK_TRUE("c1: in chunk completes", c96_wait_intr());
        (void)reg_r(0x5);
    }
    for (int b = 0; b < blocks; b++) {
        for (int i = 0; i < 512; i++) {
            if (got[b * 512 + i] != blks[b][i]) {
                std::printf("  FAIL c1: blk%d byte%d got=0x%02x want=0x%02x\n",
                            b, i, got[b * 512 + i], blks[b][i]);
                return false;
            }
        }
    }
    uint8_t status = 0xFF, msg = 0xFF;
    CHECK_TRUE("c1: finish", c96_finish(status, msg));
    CHECK_EQ("c1: status GOOD", status, 0x00);
    std::printf("[PERF] READ10 blocks=%d reset=%d first_us=%.3f total_us=%.3f MBps=%.3f\n",
                blocks, reset, (first_byte - start) / 1000.0,
                (sim_time - start) / 1000.0,
                blocks * 512.0 * 1000.0 / (sim_time - start));
    return true;
}

static bool c1_c96_read10_2blocks_control() {
    return c96_read10_measured(2);
}

static bool c19_c96_read_performance() {
    for (int blocks : {1, 2, 8, 32, 64}) {
        if (!c96_read10_measured(blocks)) return false;
        // Same extent again: covers cached data, fill-in-progress and bypass.
        if (!c96_read10_measured(blocks, false)) return false;
    }
    return true;
}

static bool c20_c96_read_clock_phase_sweep() {
    // Vary the 50 MHz sampling phase across an entire period. This checks
    // deterministic CDC delivery; it does not simulate metastability.
    for (int offset = 0; offset < 2 * PB_HALF_NS; ++offset) {
        initial_pb_offset_ns = offset;
        if (!c96_read10_measured(8)) {
            std::printf("  FAIL read CDC at pb offset %d ns\n", offset);
            initial_pb_offset_ns = 0;
            return false;
        }
    }
    initial_pb_offset_ns = 0;
    return true;
}

// c2 — RED-A (Bug 1, producer overruns the ring).
//      WRITE(10) LBA 200, 4 blocks (one CMD25), 2048 bytes pushed as
//      fast as the DRQ gate allows.  The initiator moves a byte every
//      ~2 pb cycles (40 ns); SPI moves one every ~320 ns.  With working
//      producer back-pressure the chip drops DRQ once the 512-byte ring
//      is full and the initiator waits.  Without it the initiator runs
//      ~1500 bytes ahead and buf_wr_ptr (mod 512) overwrites bytes the
//      provider has not drained — wrong payload, right LBA, GOOD status.
static bool c2_c96_write10_4blocks_full_tilt() {
    return c96_write10_multi(200, 4, 16, -1, 0, "c2");
}

// c3 — RED-B (Bug 3, provider drains an empty ring).
//      Same transfer, but the producer stops mid-block-2 for 250 us —
//      far longer than the 512 SPI byte-times (164 us) it takes the
//      provider to drain a full ring.  sd_ctrl's S_W_DATA_S issues the
//      next SPI byte unconditionally, so once the ring is empty it
//      re-sends whatever stale content sits at sec_buf[vh_drain_ptr]
//      from the previous block.  On hardware the stall is an interrupt
//      pre-empting the Mac's pseudo-DMA loop.
static bool c3_c96_write10_4blocks_stall_mid_block2() {
    return c96_write10_multi(220, 4, 16, 512 + 200, 250000, "c3");
}

// ── Widening for the producer-back-pressure fix ──────────────────────
// The corner that was almost got wrong is the ARM THRESHOLD, not the
// existence of the else: the guard runs ON a beat that has already
// pushed, so comparing the pre-push count against RING_FULL still lets
// one byte too many through.  These vary block count and chunk size so
// the ring wrap and the block boundary stop coinciding.

// c5 — smallest multi-block case: 2 blocks.  The kick fires at the end
//      of block 1, so block 2 is the ONLY concurrently-drained block —
//      the shortest window in which the guard can be wrong.
static bool c5_c96_write10_2blocks() {
    return c96_write10_multi(400, 2, 16, -1, 0, "c5");
}

// c6 — 8 blocks: the ring wraps 8 times and the producer spends most of
//      the transfer parked on a full ring, so any leak in the arm
//      threshold accumulates instead of hiding in the first block.
static bool c6_c96_write10_8blocks() {
    return c96_write10_multi(500, 8, 16, -1, 0, "c6");
}

// c7 — chunk size 48 does NOT divide 512, so DMA|CI_XFER chunks straddle
//      SCSI block boundaries and the 512-byte ring wrap lands at a
//      different offset in every chunk.  This is the case where t_req
//      has to drop and re-arm in the middle of an armed chunk, with the
//      DAFB DRQ gate following it.
static bool c7_c96_write10_3blocks_chunk48() {
    return c96_write10_multi(600, 3, 48, -1, 0, "c7");
}

// ── Widening for the consumer-back-pressure fix ──────────────────────
// The corners almost got wrong here are (a) placing the stall exactly on
// a block boundary rather than mid-block, and (b) assuming a stall short
// enough that the ring residue covers it.  Both are covered.

// c8 — stall placed exactly ON a block boundary (after the last byte of
//      block 1).  That is the byte where the ring's block bookkeeping
//      turns over, so a fencepost in either direction lands here first.
static bool c8_c96_write10_4blocks_stall_at_block_boundary() {
    return c96_write10_multi(240, 4, 16, 1023, 250000, "c8");
}

// c9 — a 1 ms stall, ~3x longer than the whole 4-block transfer takes at
//      SPI rate.  Byte-exactness must not depend on the stall being
//      short enough for the ring residue to cover it.
static bool c9_c96_write10_4blocks_long_stall() {
    return c96_write10_multi(260, 4, 16, 512 + 64, 1000000, "c9");
}

// ── Block counts the 7.5.3 driver ACTUALLY issues ────────────────────
// From a MAME Lua tap of the same 7.5.3 image (275,355 53C96 register
// events, reaching the same boot point): 41 WRITE commands, ALL
// WRITE(10) 0x2A, ZERO WRITE(6) 0x0A.  Block-count histogram —
//   1 block x32, 2 x2, 3 x1, 4 x1, 14 x1, 15 x1, 17 x1, 18 x1, 40 x1
// 9 of 41 are multi-block and they carry 115 of the 147 blocks written
// (78%).  Every one of them lands inside a disk batch that comes back
// dirty after a boot.  32 of 41 are single-block, and single-block
// writes demonstrably survive on hardware — which is exactly the shape
// in which both of these bugs are invisible.
//
// So: size the corpus on what the driver issues, not on round numbers.

// c10 — THE PRIME SUSPECT.  40 blocks at LBA 87595 is the largest real
//       WRITE on the boot path, and all 11 changed blocks of dirty disk
//       batch 87552 arrive in this single command.  20 KiB through a
//       512-byte ring: it wraps 40 times.
static bool c10_c96_write10_40blocks_lba_87595_prime_suspect() {
    return c96_write10_multi(87595, 40, 16, -1, 0, "c10");
}

// c11 / c12 — 18 and 17 blocks, both observed on the boot path (batch
//       87808).  Odd and even about the ring wrap, ~34-36 wraps each.
static bool c11_c96_write10_18blocks() {
    return c96_write10_multi(87987, 18, 16, -1, 0, "c11");
}
static bool c12_c96_write10_17blocks() {
    return c96_write10_multi(87907, 17, 16, -1, 0, "c12");
}

// c13 / c14 — 15 and 14 blocks, also observed (LBA 10164 and 87925).
static bool c13_c96_write10_15blocks() {
    return c96_write10_multi(10164, 15, 16, -1, 0, "c13");
}
static bool c14_c96_write10_14blocks() {
    return c96_write10_multi(87925, 14, 16, -1, 0, "c14");
}

// c15 — THE CONTROL.  32 of the driver's 41 writes are single-block, and
//       they work on hardware today.  A single-block write takes the
//       fully-buffered path (no ring, no concurrent drain), so neither
//       fix may touch it.  A regression HERE would be worse than the bug
//       being fixed, so it is an explicit scenario rather than an
//       assumption.
static bool c15_c96_write10_1block_control() {
    return c96_write10_multi(10147, 1, 16, -1, 0, "c15");
}

// c18 — transaction-cadence stress.  Every earlier write scenario resets the
// whole DUT and host-side card before it starts, which cannot expose stale
// bridge toggles, ring pointers, or an incompletely closed CMD25 carrying into
// the next command.  L2 makes the CPU-side command stream faster, so run the
// real System 7 block-count shapes back-to-back at the minimum cadence this
// front end permits, with no RTL reset, card reset, or inserted idle time.
static bool c18_c96_back_to_back_write10_stress() {
    apply_reset();

    std::map<uint32_t, std::vector<uint8_t>> rom_before;
    for (uint32_t lba = 0; lba < 16; lba++) {
        std::vector<uint8_t> data(512);
        for (int i = 0; i < 512; i++)
            data[i] = (uint8_t)(0xA5 ^ lba ^ i);
        preload_sd_lba(lba, data);
        rom_before[lba] = data;
    }

    static const int block_counts[] = {
        40, 1, 18, 1, 17, 2, 1, 15, 1, 3,
        14, 1, 4, 2, 1, 1, 1, 1, 1, 1
    };
    uint32_t lba = 20000;
    for (int pass = 0; pass < 2; pass++) {
        for (int blocks : block_counts) {
            char tag[40];
            std::snprintf(tag, sizeof(tag), "c18-p%d-lba%u-n%d",
                          pass, lba, blocks);
            if (!c96_write10_multi(lba, blocks, 16, -1, 0, tag,
                                   /*reset_first=*/false))
                return false;
            lba += (uint32_t)blocks + 3;
        }
    }

    if (sd.wstate != SdCard::W_IDLE) {
        std::printf("  FAIL c18: card write session still open after final command"
                    " (wstate=%d)\n", (int)sd.wstate);
        return false;
    }
    for (const auto& [rom_lba, expected] : rom_before) {
        if (sd.sectors[rom_lba] != expected) {
            std::printf("  FAIL c18: back-to-back disk writes modified ROM LBA %u\n",
                        rom_lba);
            return false;
        }
    }
    return true;
}

// c4 — the ATTRIBUTION test for commit 9545cea.
//
// That commit recorded: a 2-block WRITE(6) driven through the C96
// NON-DMA path "completes and hands over the right byte COUNT, but its
// payload comes out wrong ... from byte 0", and that an otherwise-
// identical run through the pre-existing pseudo-DMA path corrupted it
// the same way.  It blamed the SdMock in tb/tb_scsi_c96_read6.cpp
// (which parks sd_wr_ready high from sd_go) but could not prove that
// without a faithful harness.
//
// This IS the faithful harness: a real SD-card model on the SPI pins,
// with the card's own 0xE5/busy pacing, no mock anywhere.  It runs the
// exact scenario 9545cea described — WRITE(6), 2 blocks, every byte
// moved by a non-DMA Transfer Information (0x10) in 16-byte FIFO loads
// (MAME "non-dma out: fifo empty", ncr53c90.cpp:649 → :686-692 →
// :792-799).  A green result here after the flow-control fixes, with
// the mock nowhere in the picture, settles the attribution.
static bool c4_c96_write6_2blocks_nondma() {
    apply_reset();
    const uint32_t lba    = 250;
    const int      blocks = 2;
    const int      total  = blocks * 512;
    std::vector<uint8_t> payload(total);
    for (int i = 0; i < total; i++)
        payload[i] = (uint8_t)(0x5A ^ (i & 0xFF) ^ (i >> 8));

    // WRITE(6): op 0x0A, LBA 250, 2 blocks.
    const uint8_t cdb[6] = {0x0A, 0x00,
                            (uint8_t)((lba >> 8) & 0x1F),
                            (uint8_t)(lba & 0xFF),
                            (uint8_t)blocks, 0x00};
    CHECK_TRUE("c4: select", c96_select_with_cdb(cdb, 6));
    CHECK_EQ("c4: post-select phase = DATA OUT", reg_r(0x4) & 0x07, 0x00);

    for (int off = 0; off < total; off += 16) {
        for (int i = 0; i < 16; i++) reg_w(0x2, payload[off + i]);
        CHECK_EQ("c4: FIFO fully loaded", reg_r(0x7), 0x10);
        reg_w(0x3, CI_XFER);            // non-DMA Transfer Information
        if (!c96_wait_intr()) {
            std::printf("  FAIL c4: non-DMA out at offset %d never completed "
                        "(status=0x%02x cmd=0x%02x fifo=0x%02x)\n",
                        off, reg_r(0x4), reg_r(0x3), reg_r(0x7));
            return false;
        }
        (void)reg_r(0x5);
        CHECK_EQ("c4: FIFO drained", reg_r(0x7), 0x00);
    }

    uint8_t status = 0xFF, msg = 0xFF;
    CHECK_TRUE("c4: finish", c96_finish(status, msg));
    CHECK_EQ("c4: status GOOD", status, 0x00);

    bool ok = true;
    int bad = 0;
    int fb_blk = -1, fb_off = -1;
    uint8_t fb_got = 0, fb_want = 0;
    for (int b = 0; b < blocks; b++) {
        auto it = sd.sectors.find(SD_LBA_BIAS + lba + b);
        if (it == sd.sectors.end()) {
            std::printf("  FAIL c4: SD never captured LBA %u\n",
                        SD_LBA_BIAS + lba + b);
            ok = false;
            continue;
        }
        for (int i = 0; i < 512; i++) {
            if (it->second[i] != payload[b * 512 + i]) {
                if (fb_blk < 0) {
                    fb_blk = b; fb_off = i;
                    fb_got = it->second[i]; fb_want = payload[b * 512 + i];
                }
                bad++;
            }
        }
    }
    if (bad) {
        std::printf("  FAIL c4: SILENT DATA CORRUPTION — %d of %d bytes wrong "
                    "at the card (first: blk%d byte%d got=0x%02x want=0x%02x)"
                    " — NO MOCK INVOLVED\n",
                    bad, total, fb_blk, fb_off, fb_got, fb_want);
        ok = false;
    }
    if (wr_over_count() != 0) {
        std::printf("  FAIL c4: RING OVERRUN — %u initiator bytes landed on "
                    "a full ring\n", wr_over_count());
        ok = false;
    }
    if (wr_under_count() != 0) {
        std::printf("  FAIL c4: RING UNDERFLOW — %u provider bytes taken "
                    "from an empty ring\n", wr_under_count());
        ok = false;
    }
    return ok;
}

#endif  // SCSI_E2E_C96

// s12 — READ(10) LBA 100, 4 blocks via CMD18 with a STARVED SD supply.
//       Same payload check as s9, but the card stalls between blocks so the
//       blind drain genuinely outruns the supply. This is the exact condition
//       that once produced SILENT DATA CORRUPTION: reads hit an empty FIFO,
//       popped nothing, and returned the stale fifo[0] byte (fixed f8ad8c33).
//
//       Returning that stale byte is MAME-faithful and deliberately preserved,
//       so correctness here rests ENTIRELY on the pacing (withheld beats) never
//       letting the host pop an empty FIFO in the first place. Nothing else in
//       this tb could test that: the SD model topped its FIFO up instantly, so
//       the supply could never fall behind at all.
//
//       If the pacing regresses, this fails as WRONG DATA rather than as a hang.
static bool s12_read10_cmd18_starved_supply() {
    apply_reset();
    sd.slow_gap_bytes = 64;          // stall 64 byte-times before each block
    sd.gap_left       = 64;
    sd.stall_events   = 0;
    std::vector<std::vector<uint8_t>> blks(4, std::vector<uint8_t>(512));
    for (int b = 0; b < 4; b++) {
        for (int i = 0; i < 512; i++) {
            blks[b][i] = (uint8_t)((b * 0x33) ^ (i * 7) ^ 0xA5);
        }
        preload_sd_lba(SD_LBA_BIAS + 100 + b, blks[b]);
    }
    bool ok = true;
    CHECK_TRUE("s12: select", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x00, 100,
                       0x00, 0x00, 0x04, 0x00};
    for (int i = 0; i < 10; i++) {
        char l[26]; std::snprintf(l, sizeof(l), "s12: cdb[%d]", i);
        CHECK_TRUE(l, send_cdb_byte(cdb[i]));
    }
    std::vector<uint8_t> got(2048);
    for (int i = 0; i < 2048; i++) {
        char l[26]; std::snprintf(l, sizeof(l), "s12: byte[%d]", i);
        CHECK_TRUE(l, recv_in_byte(got[i]));
    }
    for (int b = 0; b < 4 && ok; b++) {
        for (int i = 0; i < 512; i++) {
            if (got[b * 512 + i] != blks[b][i]) {
                std::printf("  FAIL s12: STARVED SUPPLY CORRUPTED DATA -- "
                            "blk%d byte%d got=0x%02x want=0x%02x\n",
                            b, i, got[b * 512 + i], blks[b][i]);
                ok = false;
                break;
            }
        }
    }
    if (ok) {
        uint8_t status, msg;
        CHECK_TRUE("s12: status/msg", recv_status_and_msg(status, msg));
        CHECK_EQ("s12: status GOOD", status, 0x00);
    }
    // SELF-VERIFYING: if the card never actually withheld a byte, this scenario
    // is just s9 wearing a different name and proves nothing about starvation.
    if (sd.stall_events == 0) {
        std::printf("  FAIL s12: supply never stalled -- scenario is VACUOUS\n");
        ok = false;
    } else {
        std::printf("  s12: supply stalled %ld byte-times (starvation exercised)\n",
                    sd.stall_events);
    }
    sd.slow_gap_bytes = 0;
    sd.gap_left       = 0;
    return ok;
}

#define RUN(fn) do {                                                          \
    const uint64_t start_ns = sim_time;                                       \
    bool ok = fn();                                                           \
    std::printf("[TIME] " #fn " %.3f us\n", (sim_time - start_ns) / 1000.0); \
    if (ok) { std::printf("[PASS] " #fn "\n"); n_pass++; }                    \
    else    { std::printf("[FAIL] " #fn "\n"); n_fail++; }                    \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_sd_e2e;

#ifdef SCSI_E2E_C96
    RUN(c1_c96_read10_2blocks_control);
    RUN(c19_c96_read_performance);
    RUN(c20_c96_read_clock_phase_sweep);
    RUN(c2_c96_write10_4blocks_full_tilt);
    RUN(c3_c96_write10_4blocks_stall_mid_block2);
    RUN(c4_c96_write6_2blocks_nondma);
    RUN(c5_c96_write10_2blocks);
    RUN(c6_c96_write10_8blocks);
    RUN(c7_c96_write10_3blocks_chunk48);
    RUN(c8_c96_write10_4blocks_stall_at_block_boundary);
    RUN(c9_c96_write10_4blocks_long_stall);
    RUN(c10_c96_write10_40blocks_lba_87595_prime_suspect);
    RUN(c11_c96_write10_18blocks);
    RUN(c12_c96_write10_17blocks);
    RUN(c13_c96_write10_15blocks);
    RUN(c14_c96_write10_14blocks);
    RUN(c15_c96_write10_1block_control);
    RUN(c16_c96_write10_40blocks_provider_fails_midstream);
    RUN(c17_c96_write10_3blocks_provider_fails_first_block);
    RUN(c18_c96_back_to_back_write10_stress);
#else
    RUN(s1_tur);
    RUN(s2_inquiry);
    RUN(s3_read_capacity);
    RUN(s4_read6_lba0_1block);
    RUN(s5_read10_lba_5_2blocks);
    RUN(s6_write6_lba_3_1block);
    RUN(s7_write10_lba_10_1block);
    RUN(s8_read_capacity_after_writes);
    RUN(s9_read10_lba_100_4blocks_cmd18);
    RUN(s10_write10_lba_200_4blocks_cmd25);
    RUN(s11_write10_3blocks_atn_detour_at_ring_full);
    RUN(s12_read10_cmd18_starved_supply);
#endif

    std::printf("\n=== tb-scsi-sd-e2e ===\n");
    std::printf("PASS=%d  FAIL=%d  (total=%d)\n", n_pass, n_fail, n_pass + n_fail);

    delete dut;
    return n_fail ? 1 : 0;
}
