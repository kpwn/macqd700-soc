// tb_scsi.cpp — Verilator unit testbench for scsi.v (NCR 5380 phase FSM)
//
// Exercises the full target-side SCSI state machine end-to-end against a
// mocked SD backing store.  The Mac (initiator) is simulated by driving
// the NCR 5380 register bus through classic Selection / Command / Data /
// Status / Message-In sequences.
//
// Build via:   make tb-scsi
//
// Scenarios:
//   1. reset_defaults_and_decode        — reset values and register routing.
//   2. handshake_status_bits            — ATN/ACK show up in Bus-and-Status.
//   3. bus_free_to_selection_id0        — Mac selects ID 0 → we BSY.
//   4. selection_non_zero_id_ignored    — Mac selects ID 1..6 → no BSY.
//   5. rom_benign_unselected_probe      — ROM-style absent-ID probes stay idle.
//   6. inquiry_6_byte                   — INQUIRY CDB, 36 B response.
//   7. fixed_response_allocation_limits — short INQUIRY/SENSE stop at alloc.
//   8. read_capacity_raw_geometry       — READ CAPACITY reports raw disk size.
//   8b. mode_sense6_header              — MODE SENSE 6 4-byte header bytes.
//   9. read6_raw_frontier               — READ 6 maps onto the raw SD window.
//  10. read6_zero_length_is_256_blocks  — READ(6) length 0 is not a no-op.
//  11. read10_one_block                 — READ 10 non-zero LBA + data.
//  12. read6_no_device_timeout          — absent SD back-end returns CHECK.
//  13. read10_lba_out_of_range          — invalid read fails closed before SD.
//  14. write10_lba_out_of_range         — invalid write fails closed before SD.
//  15. write6_one_block                 — WRITE 6 LBA 1 × 1 → buffer match.
//  16. write10_one_block                — WRITE 10 non-zero LBA + data.
//  17. read10_sd_error                  — READ 10 with SD fault → CHECK.
//  17b. write10_sd_error               — WRITE 10 with SD fault → CHECK.
//  17c. read10_multiblock_midstream_sd_error — provider faults while the
//                                        target is already in DATA_IN.
//  18. reset_during_selection           — RST line bit → return BUS_FREE.
//  19. irq_on_status_msgin_boundaries   — IRQ asserted and cleared.
//  20. phase_match_tracks_target_command — BAS[3] follows TCR phase bits.
//  21. completion_latch_clears_on_reg7   — BAS[7] tracks completion and clears.
//  22. reset_clears_pending_irq          — RST drops stale IRQ/DRQ latches.
//  23. drq_wire_tracks_data_phase        — DRQ output matches BAS[6].
//  24. tur_reports_no_medium_after_timeout — TEST UNIT READY reflects media loss.
//  25. read10_zero_length_no_sd          — READ(10) zero length is GOOD no-op.
//  26. write10_zero_length_no_sd         — WRITE(10) zero length is GOOD no-op.
//  27. status_irq_visible_without_extra_poll — STATUS REQ and IRQ appear together.
//  28. check_condition_sense_persists    — sense survives unrelated GOOD command.
//  29. pseudo_dma_reads_reg6_and_completes — DMA-read path uses input-data reg.
//  30. zero_length_completion_visible     — zero-length status has END_DMA now.
//  31. read10_two_blocks_lba_sequence     — multi-block reads step raw LBAs.
//  32. write6_zero_length_is_256_blocks  — WRITE(6) length 0 is not a no-op.
//  33. write6_no_device_timeout           — missing write back-end CHECKs.
//  34. turboscsi_dma_shim_read_advances  — 0x100/0x101 reads drain DATA_IN.
//  35. turboscsi_dma_shim_write_advances — 0x100/0x101 writes drain DATA_OUT.
//
// NCR 5380 register map (pb_addr[2:0]) — read vs write:
//   0 R:Current SCSI Data   W:Output Data
//   1 R/W:Initiator Command
//   2 R/W:Mode
//   3 R/W:Target Command
//   4 R:Current SCSI Bus Status  W:Select Enable
//   5 R:Bus and Status           W:Start DMA Send
//   6 R:Input Data               W:Start DMA Target Receive
//   7 R:Reset Interrupt (clear)  W:Start DMA Initiator Receive
//
// Initiator Command bits (positive logic):
//   [7]=RST [4]=ACK [3]=BSY [2]=SEL [1]=ATN [0]=DATA_BUS
//
// Current SCSI Bus Status bits (positive logic):
//   [7]=RST [6]=BSY [5]=REQ [4]=MSG [3]=C_D [2]=I_O [1]=SEL [0]=DBP
//
// Bus and Status bits:
//   [4]=IRQ [3]=PHASE_MATCH [1]=ATN [0]=ACK

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"

static Vtb_scsi_vhdd_sd*   dut      = nullptr;
static uint64_t sim_time = 0;
static int      n_pass   = 0;
static int      n_fail   = 0;
static constexpr uint32_t SD_RAW_BASE_LBA = 8192u;
static constexpr uint32_t SCSI_NUM_LBAS = 1048576u;

// ── Mocked SD backing store ────────────────────────────────────────────
// Holds up to a few sectors worth of data indexed by SD-sector number
// (already biased by the raw SD base LBA).  The mock drives the sd_ctrl-shaped request
// interface back at the DUT: on sd_go it honours cmd_type (1=read,
// 3=write), replays 512 bytes over 512 rd_valid pulses for reads, or
// clocks 512 wr_ready pulses for writes.  An `inject_error` flag causes
// the mock to fire sd_done with sd_error=1 (simulates a medium fault).
struct SdMock {
    // sector_key = sd_lba as driven by the DUT (scsi_lba + 8192)
    std::vector<uint8_t> read_sector;     // 512 B pool used as read source
    std::vector<uint8_t> written_sector;  // last WRITE captured (512 B)
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    std::vector<uint32_t> lba_history;
    std::vector<uint8_t> cmd_history;
    bool     inject_error = false;
    bool     present = true;

    // Internal state for the mock FSM
    enum class State { Idle, Reading, Writing, Done } st = State::Idle;
    int cnt = 0;     // bytes transferred in current block
    int delay = 0;   // small artificial latency per SD request
    uint16_t blocks_left = 0;  // for CMD18/CMD25 multi-block tracking
    bool     multiblock = false;
    int      block_idx = 0;    // current block (0-based) within multi-block
} sd_mock;

// Drive SD mock outputs onto DUT inputs based on current DUT.sd_go/state.
static void sd_mock_tick() {
    // Defaults: all signals low this cycle.
    dut->sd_busy      = (sd_mock.st != SdMock::State::Idle) ? 1 : 0;
    dut->sd_done      = 0;
    dut->sd_error     = 0;
    dut->sd_rd_valid  = 0;
    dut->sd_rd_data   = 0;
    dut->sd_wr_ready  = 0;

    if (sd_mock.present &&
        sd_mock.st == SdMock::State::Idle &&
        dut->sd_go) {
        sd_mock.last_lba      = dut->sd_lba;
        sd_mock.last_cmd_type = dut->sd_cmd_type;
        sd_mock.lba_history.push_back(dut->sd_lba);
        sd_mock.cmd_history.push_back(dut->sd_cmd_type);
        sd_mock.cnt = 0;
        sd_mock.delay = 2;  // small "command turnaround" latency
        sd_mock.block_idx = 0;
        // Honour multi-block CMD18/CMD25 by streaming N back-to-back
        // blocks within a single mock transaction (one history entry,
        // one sd_done at the very end).  Unknown cmd_types fall through
        // to Done (silently).
        sd_mock.blocks_left = (dut->sd_block_count == 0) ? 1
                                                         : dut->sd_block_count;
        sd_mock.multiblock = (dut->sd_cmd_type == 2 || dut->sd_cmd_type == 4);
        if (dut->sd_cmd_type == 1 || dut->sd_cmd_type == 2) {
            sd_mock.st = SdMock::State::Reading;
        } else if (dut->sd_cmd_type == 3 || dut->sd_cmd_type == 4) {
            sd_mock.st = SdMock::State::Writing;
        } else {
            sd_mock.st = SdMock::State::Done;
        }
        dut->sd_busy = 1;
        return;
    }

    if (sd_mock.st == SdMock::State::Reading) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) { sd_mock.delay--; return; }
        if (sd_mock.cnt < 512) {
            if (sd_mock.inject_error && sd_mock.cnt >= 128) {
                // Fault partway through — jump to Done with error.
                sd_mock.st = SdMock::State::Done;
                dut->sd_done  = 1;
                dut->sd_error = 1;
                return;
            }
            dut->sd_rd_valid = 1;
            uint8_t byte = 0;
            if (!sd_mock.read_sector.empty()) {
                // Slight per-block variation so multi-block tests can
                // tell blocks apart: bake block_idx into the byte.
                size_t pool_idx = (sd_mock.cnt +
                                   sd_mock.block_idx * 512u) %
                                  sd_mock.read_sector.size();
                byte = sd_mock.read_sector[pool_idx];
            }
            dut->sd_rd_data = byte;
            sd_mock.cnt++;
            if (sd_mock.cnt == 512) {
                if (sd_mock.multiblock &&
                    sd_mock.block_idx + 1 < (int)sd_mock.blocks_left) {
                    // Roll over to the next block of the same CMD18.
                    sd_mock.block_idx++;
                    sd_mock.cnt = 0;
                    sd_mock.delay = 2;
                } else {
                    sd_mock.st = SdMock::State::Done;
                }
            }
        }
    } else if (sd_mock.st == SdMock::State::Writing) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) { sd_mock.delay--; return; }
        if (sd_mock.cnt < 512) {
            dut->sd_wr_ready = 1;
            sd_mock.written_sector.push_back(dut->sd_wr_data & 0xFF);
            sd_mock.cnt++;
            if (sd_mock.cnt == 512) {
                if (sd_mock.multiblock &&
                    sd_mock.block_idx + 1 < (int)sd_mock.blocks_left) {
                    sd_mock.block_idx++;
                    sd_mock.cnt = 0;
                    sd_mock.delay = 2;
                } else {
                    sd_mock.st = SdMock::State::Done;
                }
            }
        }
    } else if (sd_mock.st == SdMock::State::Done) {
        // Hold done for one cycle, then return to Idle.
        dut->sd_busy = 0;
        dut->sd_done = 1;
        if (sd_mock.inject_error) dut->sd_error = 1;
        sd_mock.st = SdMock::State::Idle;
    }
}

// ── Clock helpers ──────────────────────────────────────────────────────
static void tick_n(int n = 1) {
    for (int i = 0; i < n; i++) {
        // Rising edge
        sd_mock_tick();
        dut->clk = 1;
        dut->eval();
        // Falling edge
        dut->clk = 0;
        dut->eval();
        sim_time++;
    }
}

static void reset() {
    dut->rst = 1;
    // Disk size is a runtime input now (derived from the SD card's CSD at
    // boot) rather than a compile-time parameter -- drive it, or the target
    // reports a zero-sector disk and every read fails.
    dut->disk_num_lbas = SCSI_NUM_LBAS;
    dut->wprot = 0;            // volume writable unless a test locks it
    dut->pb_addr = 0; dut->pb_wdata = 0; dut->pb_wr = 0; dut->pb_rd = 0;
    dut->sd_busy = 0; dut->sd_done = 0; dut->sd_error = 0;
    dut->sd_rd_valid = 0; dut->sd_rd_data = 0;
    dut->sd_wr_ready = 0;
    sd_mock = SdMock();
    tick_n(4);
    dut->rst = 0;
    tick_n(2);
}

// ── Assertion helpers ──────────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%08x, expected 0x%08x (t=%lu)\n", \
               name, _g, _e, (unsigned long)sim_time); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s (t=%lu)\n", name, (unsigned long)sim_time); \
        return false; \
    } \
} while(0)

// ── Bus helpers ────────────────────────────────────────────────────────
static void bus_write(uint8_t addr, uint8_t data) {
    dut->pb_addr = addr & 0x7;
    dut->pb_wdata = data;
    dut->pb_wr = 1;
    dut->pb_rd = 0;
    tick_n(1);
    dut->pb_wr = 0;
    dut->pb_wdata = 0;
}

static void bus_write9(uint16_t addr, uint8_t data) {
    dut->pb_addr = addr & 0x1ff;
    dut->pb_wdata = data;
    dut->pb_wr = 1;
    dut->pb_rd = 0;
    tick_n(1);
    dut->pb_wr = 0;
    dut->pb_wdata = 0;
}

static uint8_t bus_read(uint8_t addr) {
    dut->pb_addr = addr & 0x7;
    dut->pb_rd = 1;
    dut->pb_wr = 0;
    tick_n(1);
    dut->pb_rd = 0;
    dut->eval();
    return dut->pb_rdata & 0xFF;
}

static uint8_t bus_read9(uint16_t addr) {
    dut->pb_addr = addr & 0x1ff;
    dut->pb_rd = 1;
    dut->pb_wr = 0;
    tick_n(1);
    dut->pb_rd = 0;
    dut->eval();
    return dut->pb_rdata & 0xFF;
}

// ── 5380 Initiator-Command bit constants ──────────────────────────────
static constexpr uint8_t IC_RST = 0x80;
static constexpr uint8_t IC_ACK = 0x10;
static constexpr uint8_t IC_BSY = 0x08;
static constexpr uint8_t IC_SEL = 0x04;
static constexpr uint8_t IC_ATN = 0x02;
static constexpr uint8_t IC_DB  = 0x01;

// Current SCSI Bus Status bits
static constexpr uint8_t SR_BSY = 0x40;
static constexpr uint8_t SR_REQ = 0x20;
static constexpr uint8_t SR_MSG = 0x10;
static constexpr uint8_t SR_CD  = 0x08;
static constexpr uint8_t SR_IO  = 0x04;

// Bus and Status bits
static constexpr uint8_t BS_IRQ = 0x10;
static constexpr uint8_t BS_END_DMA = 0x80;
static constexpr uint8_t BS_DRQ = 0x40;
static constexpr uint8_t BS_PHASE_MATCH = 0x08;
static constexpr uint8_t BS_BUSYERR = 0x04;
static constexpr uint8_t BS_ATN = 0x02;
static constexpr uint8_t BS_ACK = 0x01;

// ── Selection helpers ─────────────────────────────────────────────────
// Drive an NCR-5380 Selection of `target_id`.  Returns true if the DUT
// asserted BSY within `timeout` ticks.
static bool do_select(uint8_t target_id, int timeout = 64) {
    bus_write(0, (uint8_t)(1 << target_id));   // Output Data = ID bit
    bus_write(1, IC_SEL | IC_DB);              // Assert SEL + DATA_BUS
    for (int i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_BSY) return true;
        tick_n(1);
    }
    return false;
}

// Drop SEL to advance from S_SELECT → S_COMMAND.
static void drop_sel() {
    bus_write(1, IC_DB);      // SEL off
    tick_n(2);
}

// Send a single CDB byte via REQ/ACK handshake.  Returns true if REQ was
// observed and the handshake completed within the timeout.
static bool send_cdb_byte(uint8_t b, int timeout = 64) {
    // Wait for REQ asserted
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_REQ) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    // Place byte on Output Data
    bus_write(0, b);
    // Assert ACK (plus SEL-group-off)
    bus_write(1, IC_ACK | IC_DB);
    // Wait for REQ to go away (target accepts byte)
    for (i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_REQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    // Drop ACK
    bus_write(1, IC_DB);
    tick_n(2);
    return true;
}

// Pull one data-in byte via REQ/ACK.
static bool recv_in_byte(uint8_t& out, int timeout = 1024) {
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_REQ) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    out = bus_read(0);                // latch current data
    bus_write(1, IC_ACK);
    // Wait for REQ to drop
    for (i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_REQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(1, 0);
    tick_n(2);
    return true;
}

// Pull one data-in byte through the 5380 Input Data register.  This mirrors
// ROM pseudo-DMA polling: wait for DRQ in Bus-and-Status, read reg 6, ACK.
static bool recv_dma_in_byte(uint8_t& out, int timeout = 1024) {
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        uint8_t bas = bus_read(5);
        if ((s & SR_REQ) && (bas & BS_DRQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    out = bus_read(6);
    bus_write(1, IC_ACK);
    for (i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_REQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(1, 0);
    tick_n(2);
    return true;
}

// Push one data-out byte via REQ/ACK.
static bool push_out_byte(uint8_t b, int timeout = 1024) {
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_REQ) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(0, b);
    bus_write(1, IC_ACK);
    for (i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_REQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(1, 0);
    tick_n(2);
    return true;
}

static bool wait_for_reg_match(uint8_t addr, uint8_t mask, uint8_t value,
                               int timeout = 1024) {
    for (int i = 0; i < timeout; i++) {
        uint8_t v = bus_read(addr);
        if ((v & mask) == value) return true;
        tick_n(1);
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 1 — reset defaults and register decode smoke.
// ═══════════════════════════════════════════════════════════════════════
static bool test_reset_defaults_and_decode() {
    reset();
    CHECK_EQ("IRQ low after reset", dut->irq, 0);
    CHECK_EQ("DRQ low after reset", dut->drq, 0);
    CHECK_EQ("pb_ack low after reset", dut->pb_ack, 0);
    // sd_cmd_type / sd_lba idle values changed with the vhdd refactor and
    // the change is inert.  They used to be registers inside scsi.v that
    // reset to 0; they are now combinational functions of the vhdd request
    // (vhdd_sd.v), so at reset they read the encoding of "single-block
    // read of volume LBA 0" — cmd_type CMD17, lba = the reserved-window
    // base.  Nothing samples them while sd_go is low: sd_scsi_bridge
    // latches cmd_type/lba/block_count only on `pb_go && !pb_busy_internal`.
    // What actually has to hold at reset is that NO request is in flight,
    // which is sd_go — checked below and unchanged.
    CHECK_EQ("sd_cmd_type reset (single-block read encoding)",
             dut->sd_cmd_type, 1);
    CHECK_EQ("sd_lba reset (volume LBA 0 = reserved-window base)",
             dut->sd_lba, SD_RAW_BASE_LBA);
    CHECK_EQ("sd_block_count reset", dut->sd_block_count, 1u);
    CHECK_EQ("sd_go reset", dut->sd_go, 0);
    CHECK_EQ("sd_wr_data reset", dut->sd_wr_data, 0);

    CHECK_EQ("reg0 default", bus_read(0), 0);
    CHECK_EQ("reg1 default", bus_read(1), 0);
    CHECK_EQ("reg2 default", bus_read(2), 0);
    CHECK_EQ("reg3 default", bus_read(3), 0);
    CHECK_EQ("reg4 default", bus_read(4), 0);
    CHECK_EQ("reg5 default", bus_read(5), 0);
    CHECK_EQ("reg6 default", bus_read(6), 0);
    CHECK_EQ("reg7 default", bus_read(7), 0);

    bus_write(1, 0x91);
    CHECK_EQ("reg1 decode", bus_read(1), 0x91);
    CHECK_EQ("reg4 shows RST", bus_read(4) & 0x80, 0x80);
    bus_write(1, 0x00);
    CHECK_EQ("reg1 clear RST", bus_read(1), 0x00);
    CHECK_EQ("reg4 back idle", bus_read(4), 0x00);

    bus_write(2, 0x2C);
    CHECK_EQ("reg2 decode", bus_read(2), 0x2C);
    bus_write(3, 0x5A);
    CHECK_EQ("reg3 decode", bus_read(3), 0x5A);

    bus_write(0, 0xA5);
    CHECK_EQ("reg0 read stays bus-data echo", bus_read(0), 0x00);

    bus_write(4, 0x44);
    CHECK_EQ("reg4 stays idle", bus_read(4), 0x00);
    bus_write(5, 0x55);
    CHECK_EQ("reg5 stays idle", bus_read(5), 0x00);
    bus_write(6, 0x66);
    CHECK_EQ("reg6 stays idle", bus_read(6), 0x00);
    bus_write(7, 0x77);
    CHECK_EQ("reg7 read clears to zero", bus_read(7), 0x00);

    CHECK_EQ("reg1 cleared", bus_read(1), 0x00);
    CHECK_EQ("reg2 preserved", bus_read(2), 0x2C);
    CHECK_EQ("reg3 preserved", bus_read(3), 0x5A);
    return true;
}

// Consume Status byte + Message-In byte (both via REQ/ACK), returning
// both via out params.  This drives the tail of the command sequence to
// DISCONNECT.
static bool recv_status_and_msg(uint8_t& status, uint8_t& msg) {
    // STATUS phase is signalled by BSY|REQ|CD|IO.
    // MSG_IN phase adds MSG.
    if (!recv_in_byte(status)) return false;
    if (!recv_in_byte(msg))    return false;
    // DUT goes to DISCONNECT → BUS_FREE.  Wait for BSY to drop.
    for (int i = 0; i < 64; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_BSY)) return true;
        tick_n(1);
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2 — handshakes reflect ATN/ACK in Bus-and-Status.
// ═══════════════════════════════════════════════════════════════════════
static bool test_handshake_status_bits() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    CHECK_TRUE("entered command phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_CD));

    bus_write(1, IC_ATN | IC_DB);
    CHECK_EQ("ATN reflected in bus/status", bus_read(5) & BS_ATN, BS_ATN);
    bus_write(1, IC_DB);
    tick_n(2);

    CHECK_TRUE("command REQ visible",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_CD));
    bus_write(0, 0x00);
    bus_write(1, IC_ACK | IC_ATN | IC_DB);
    CHECK_EQ("ACK reflected in bus/status", bus_read(5) & BS_ACK, BS_ACK);
    CHECK_EQ("ATN held in bus/status", bus_read(5) & BS_ATN, BS_ATN);

    for (int i = 0; i < 32; i++) {
        if (!(bus_read(4) & SR_REQ)) break;
        tick_n(1);
    }
    bus_write(1, IC_DB);
    tick_n(2);

    for (int i = 0; i < 5; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "TUR byte[%d]", i + 1);
        CHECK_TRUE(lbl, send_cdb_byte(0x00));
    }

    uint8_t status, msg;
    CHECK_TRUE("status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("status GOOD", status, 0x00);
    CHECK_EQ("msg COMPLETE", msg, 0x00);

    (void)bus_read(7);
    tick_n(2);
    CHECK_EQ("IRQ cleared in bus/status", bus_read(5) & BS_IRQ, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 17 — MAME/NCR 5380 BAS[PHASE_MATCH] behaviour.
//
// MAME ncr5380.cpp computes BAS_PHASEMATCH from the current bus phase
// bits compared with the Target Command register phase field.  The ROM
// driver uses this as a request-visibility guard before handshaking.
// ═══════════════════════════════════════════════════════════════════════
static bool test_phase_match_tracks_target_command() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    CHECK_TRUE("entered command phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_CD));

    CHECK_EQ("command phase mismatch with TC=DATAOUT",
             bus_read(5) & BS_PHASE_MATCH, 0);
    bus_write(3, 0x02);  // Target Command phase = COMMAND (C/D)
    CHECK_EQ("command phase match with TC=COMMAND",
             bus_read(5) & BS_PHASE_MATCH, BS_PHASE_MATCH);
    bus_write(3, 0x01);  // DATA IN, deliberately wrong for command phase
    CHECK_EQ("command phase mismatch with TC=DATAIN",
             bus_read(5) & BS_PHASE_MATCH, 0);
    bus_write(3, 0x02);

    uint8_t cdb[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "PM TUR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    bus_write(3, 0x03);  // STATUS = C/D | I/O
    CHECK_TRUE("entered status phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO | SR_MSG,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO));
    CHECK_EQ("status phase match with TC=STATUS",
             bus_read(5) & BS_PHASE_MATCH, BS_PHASE_MATCH);

    uint8_t status = 0;
    CHECK_TRUE("recv status", recv_in_byte(status));
    CHECK_EQ("status GOOD", status, 0x00);

    bus_write(3, 0x07);  // MSG IN = MSG | C/D | I/O
    CHECK_TRUE("entered message-in phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO | SR_MSG,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO | SR_MSG));
    CHECK_EQ("message phase match with TC=MSG_IN",
             bus_read(5) & BS_PHASE_MATCH, BS_PHASE_MATCH);

    uint8_t msg = 0;
    CHECK_TRUE("recv msg", recv_in_byte(msg));
    CHECK_EQ("msg COMPLETE", msg, 0x00);
    std::printf("  scsi firstlight summary: phase-match command/status/msg-in visible, irq=%u\n",
                (unsigned)dut->irq);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 1 — BUS_FREE → Selection of ID=0 → our BSY asserts.
// ═══════════════════════════════════════════════════════════════════════
static bool test_bus_free_to_selection_id0() {
    reset();
    CHECK_EQ("idle bus_status", bus_read(4), 0);
    CHECK_TRUE("selection ID=0", do_select(0));
    // Now BSY asserted; also SEL echo shown while Mac holds SEL.
    uint8_t s = bus_read(4);
    CHECK_TRUE("BSY set after selection", (s & SR_BSY));
    // Drop SEL → expect Command phase (CD asserted, REQ raised eventually)
    drop_sel();
    bool saw_cmd = false;
    for (int i = 0; i < 64; i++) {
        uint8_t b = bus_read(4);
        if ((b & SR_BSY) && (b & SR_CD) && (b & SR_REQ)) {
            saw_cmd = true; break;
        }
        tick_n(1);
    }
    CHECK_TRUE("entered Command phase", saw_cmd);
    // Reset RST to abort
    bus_write(1, IC_RST);
    tick_n(4);
    bus_write(1, 0);
    tick_n(4);
    CHECK_EQ("bus idle after RST", bus_read(4), 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2 — Selection of ID 1..6 → we stay in Bus Free.
// ═══════════════════════════════════════════════════════════════════════
static bool test_selection_non_zero_id_ignored() {
    reset();
    for (uint8_t id = 1; id <= 6; id++) {
        // Drive non-matching select attempt
        bus_write(0, (uint8_t)(1 << id));
        bus_write(1, IC_SEL | IC_DB);
        // Poll for BSY — must NEVER appear.
        for (int i = 0; i < 32; i++) {
            uint8_t s = bus_read(4);
            char lbl[32]; snprintf(lbl, sizeof(lbl), "no BSY id=%u", id);
            CHECK_EQ(lbl, s & SR_BSY, 0);
            tick_n(1);
        }
        // Clear SEL to release
        bus_write(1, 0);
        tick_n(4);
    }
    // And a final ID=7 (initiator's own ID) also should not latch us.
    bus_write(0, 0x80);
    bus_write(1, IC_SEL | IC_DB);
    for (int i = 0; i < 32; i++) {
        CHECK_EQ("no BSY id=7", bus_read(4) & SR_BSY, 0);
        tick_n(1);
    }
    bus_write(1, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 5 — ROM-style benign probe of absent SCSI IDs.
//
// The Mac ROM may poke mode/select/DMA trigger registers while it scans
// the SCSI bus.  Non-target selections must not assert BSY, IRQ, DRQ, or
// busy-error; they should look like open bus-free probes until ID 0 is
// selected.
// ═══════════════════════════════════════════════════════════════════════
static bool test_rom_benign_unselected_probe() {
    reset();

    bus_write(2, 0x00);      // Mode: normal initiator mode
    bus_write(3, 0x00);      // Target Command: bus-free phase
    bus_write(4, 0xFF);      // Select-enable mask write is benign here
    bus_write(5, 0x01);      // DMA trigger writes are swallowed in BUS_FREE
    bus_write(6, 0x02);
    bus_write(7, 0x03);

    for (uint8_t id = 1; id <= 6; id++) {
        uint8_t select_bus = (uint8_t)((1u << 7) | (1u << id));
        bus_write(0, select_bus);       // initiator ID 7 + probed target
        bus_write(1, IC_SEL | IC_DB);
        for (int i = 0; i < 32; i++) {
            uint8_t s = bus_read(4);
            char lbl[48]; snprintf(lbl, sizeof(lbl), "ROM probe no BSY id=%u", id);
            CHECK_EQ(lbl, s & SR_BSY, 0);
            CHECK_EQ("ROM probe no REQ", s & SR_REQ, 0);
            CHECK_EQ("ROM probe no MSG/CD/IO", s & (SR_MSG | SR_CD | SR_IO), 0);
            CHECK_EQ("ROM probe no IRQ/BUSYERR", bus_read(5) & (BS_IRQ | BS_BUSYERR), 0);
            CHECK_TRUE("ROM probe irq wire low", dut->irq == 0);
            tick_n(1);
        }
        bus_write(1, 0x00);
        tick_n(4);
    }

    bus_write(1, IC_RST);
    CHECK_EQ("RST bit visible during benign probe", bus_read(4) & 0x80, 0x80);
    bus_write(1, 0x00);
    tick_n(4);
    CHECK_EQ("bus free after benign probe reset", bus_read(4), 0x00);
    CHECK_EQ("bus/status quiet after benign probe", bus_read(5) & (BS_IRQ | BS_DRQ | BS_BUSYERR), 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 3 — INQUIRY 6-byte CDB → 36-byte response.
// ═══════════════════════════════════════════════════════════════════════
static bool test_inquiry_6_byte() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    // Send CDB: INQUIRY (opcode 0x12), LUN 0, alloc 36, ctrl 0.
    uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "send cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    // Receive 36 bytes of INQUIRY data.
    uint8_t inq[36];
    for (int i = 0; i < 36; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "recv inq[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(inq[i]));
    }
    // Validate key fields.
    CHECK_EQ("INQUIRY device type",   inq[0], 0x00);
    CHECK_EQ("INQUIRY removable",     inq[1], 0x00);
    CHECK_EQ("INQUIRY SCSI version",  inq[2], 0x02);
    CHECK_EQ("INQUIRY additional len", inq[4], 31);
    // Vendor "APPLE   "; product "HD SC..."
    CHECK_EQ("INQUIRY vendor[0]",  inq[8],  'A');
    CHECK_EQ("INQUIRY vendor[1]",  inq[9],  'P');
    CHECK_EQ("INQUIRY vendor[2]",  inq[10], 'P');
    CHECK_EQ("INQUIRY vendor[3]",  inq[11], 'L');
    CHECK_EQ("INQUIRY vendor[4]",  inq[12], 'E');
    CHECK_EQ("INQUIRY product[0]", inq[16], 'H');
    CHECK_EQ("INQUIRY product[1]", inq[17], 'D');
    CHECK_EQ("INQUIRY product[3]", inq[19], 'S');
    CHECK_EQ("INQUIRY product[4]", inq[20], 'C');
    // Status + message-in
    uint8_t status, msg;
    CHECK_TRUE("status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("INQUIRY status GOOD", status, 0x00);
    CHECK_EQ("INQUIRY msg COMPLETE", msg,   0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 7 — MAME/nscsi fixed-response allocation lengths.
//
// MAME's nscsi hard disk model uses the CDB allocation byte as the data-in
// byte count for fixed responses such as INQUIRY and MODE SENSE, bounded by
// the generated response size.  The Mac ROM probes these commands before raw
// disk reads, so a short allocation must advance to STATUS instead of waiting
// for an unrequested long response.
// ═══════════════════════════════════════════════════════════════════════
static bool test_fixed_response_allocation_limits() {
    reset();
    CHECK_TRUE("selection short inquiry", do_select(0));
    drop_sel();
    uint8_t inq_cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x05, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "short INQ cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(inq_cdb[i]));
    }

    uint8_t inq[5];
    for (int i = 0; i < 5; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "short INQ byte[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(inq[i]));
    }
    CHECK_EQ("short INQ device type", inq[0], 0x00);
    CHECK_EQ("short INQ additional len", inq[4], 31);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("short INQ status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("short INQ status GOOD", status, 0x00);
    CHECK_EQ("short INQ msg COMPLETE", msg, 0x00);

    reset();
    CHECK_TRUE("selection zero inquiry", do_select(0));
    drop_sel();
    uint8_t zero_inq_cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "zero INQ cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(zero_inq_cdb[i]));
    }
    CHECK_TRUE("zero INQ status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("zero INQ status GOOD", status, 0x00);
    CHECK_EQ("zero INQ msg COMPLETE", msg, 0x00);

    reset();
    CHECK_TRUE("selection bad opcode", do_select(0));
    drop_sel();
    uint8_t bad_cdb[6] = {0x7F, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "bad opcode cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(bad_cdb[i]));
    }
    CHECK_TRUE("bad opcode status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("bad opcode CHECK", status, 0x02);
    CHECK_EQ("bad opcode completion latch", bus_read(5) & BS_END_DMA,
             BS_END_DMA);

    CHECK_TRUE("selection short sense", do_select(0));
    drop_sel();
    uint8_t sense_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x04, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "short SENSE cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(sense_cdb[i]));
    }

    uint8_t sense[4];
    for (int i = 0; i < 4; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "short SENSE byte[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("short SENSE response", sense[0], 0x70);
    CHECK_EQ("short SENSE key illegal request", sense[2] & 0x0F, 0x05);
    CHECK_TRUE("short SENSE status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("short SENSE status GOOD", status, 0x00);
    CHECK_EQ("short SENSE msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 8 — READ CAPACITY 10 reports the synthesized raw-disk geometry.
//
// This is the ROM-facing "what size disk is it?" probe. It should work
// without any active SD backing-store transaction because the controller
// synthesizes the geometry from DISK_NUM_LBAS.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read_capacity_raw_geometry() {
    reset();
    sd_mock.present = false;

    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x25, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RCAP cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    CHECK_TRUE("RCAP entered data-in",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_IO));
    CHECK_EQ("RCAP DRQ visible", bus_read(5) & BS_DRQ, BS_DRQ);
    CHECK_EQ("RCAP no SD command", sd_mock.last_cmd_type, 0);
    CHECK_EQ("RCAP no SD lba", sd_mock.last_lba, 0u);
    CHECK_EQ("RCAP no sd_go", dut->sd_go, 0);

    uint8_t cap[8];
    for (int i = 0; i < 8; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RCAP recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(cap[i]));
    }
    CHECK_EQ("RCAP last LBA[31:24]", cap[0], 0x00);
    CHECK_EQ("RCAP last LBA[23:16]", cap[1], 0x0F);
    CHECK_EQ("RCAP last LBA[15:8]", cap[2], 0xFF);
    CHECK_EQ("RCAP last LBA[7:0]", cap[3], 0xFF);
    CHECK_EQ("RCAP block size[31:24]", cap[4], 0x00);
    CHECK_EQ("RCAP block size[23:16]", cap[5], 0x00);
    CHECK_EQ("RCAP block size[15:8]", cap[6], 0x02);
    CHECK_EQ("RCAP block size[7:0]", cap[7], 0x00);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("RCAP status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("RCAP status GOOD", status, 0x00);
    CHECK_EQ("RCAP msg COMPLETE", msg, 0x00);
    CHECK_EQ("RCAP DRQ low after data", dut->drq, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 8b — MODE SENSE 6 → 4-byte mode parameter header.
//
// The existing allocation-limit scenario issues MODE SENSE only to check
// that a short/zero allocation terminates the phase; nothing ever
// validated the four header BYTES.  That gap was found by positive
// control while moving the canned data-in payloads out of sec_buf and
// into the `canned_byte` ROM (rtl/mac/scsi.v): corrupting the MODE SENSE
// entry changed no test result at all.  Per MAME's nscsi hard-disk model
// the header is {mode data len = 3, medium type = 0, device specific = 0,
// block descriptor len = 0} with no pages appended.
// ═══════════════════════════════════════════════════════════════════════
static bool test_mode_sense6_header() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    // MODE SENSE 6 (0x1A), PC/page 0x00, alloc 4, ctrl 0.
    uint8_t cdb[6] = {0x1A, 0x00, 0x00, 0x00, 0x04, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "MS6 cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    uint8_t hdr[4];
    for (int i = 0; i < 4; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "MS6 recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(hdr[i]));
    }
    CHECK_EQ("MS6 mode data length",       hdr[0], 0x03);
    CHECK_EQ("MS6 medium type",            hdr[1], 0x00);
    CHECK_EQ("MS6 device specific",        hdr[2], 0x00);
    CHECK_EQ("MS6 block descriptor length", hdr[3], 0x00);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("MS6 status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("MS6 status GOOD", status, 0x00);
    CHECK_EQ("MS6 msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 4 — READ 6 (LBA 0, 1 block) → raw frontier + data.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read6_raw_frontier() {
    reset();
    // Preload mock with an incrementing pattern.
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; i++) {
        sd_mock.read_sector[i] = (uint8_t)(i ^ 0x5A);
    }
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    CHECK_TRUE("entered command phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_CD));
    // READ 6: op=0x08, lba=0x00000, len=1.  This lands at the raw SD
    // frontier (LBA 8192) and proves the reserved boot window stays
    // separate from disk traffic.
    uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ6 cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    CHECK_TRUE("entered data-in phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_IO));
    CHECK_TRUE("data phase exposes DRQ",
               wait_for_reg_match(5, BS_DRQ | BS_IRQ, BS_DRQ));
    CHECK_EQ("READ6 sd_lba raw base", sd_mock.last_lba, SD_RAW_BASE_LBA);
    CHECK_EQ("READ6 sd cmd",      sd_mock.last_cmd_type, 1);

    std::vector<uint8_t> got(512);
    for (int i = 0; i < 512; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ6 recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(got[i]));
    }
    CHECK_TRUE("status phase after data",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO | SR_MSG,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO));
    CHECK_EQ("status phase IRQ bit", bus_read(5) & BS_IRQ, BS_IRQ);
    CHECK_EQ("status phase DRQ low", bus_read(5) & BS_DRQ, 0);
    // Spot-check pattern
    for (int i = 0; i < 512; i += 53) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ6 byte[%d]", i);
        CHECK_EQ(lbl, got[i], (uint8_t)(i ^ 0x5A));
    }
    // Status + msg
    uint8_t status, msg;
    CHECK_TRUE("READ6 status", recv_status_and_msg(status, msg));
    CHECK_EQ("READ6 status GOOD", status, 0x00);
    CHECK_EQ("READ6 msg COMPLETE", msg,  0x00);
    CHECK_TRUE("IRQ latched through disconnect", dut->irq == 1);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 10 — READ(6) transfer length 0 means 256 blocks.
//
// READ(10) length 0 is a no-op, but six-byte READ/WRITE commands use zero
// to request 256 blocks.  The ROM boot path can use either command family,
// so keep this legacy SCSI-1 convention explicit.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read6_zero_length_is_256_blocks() {
    reset();
    sd_mock.read_sector.assign(512, 0xE6);

    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x02, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "READ6 zero cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    std::vector<uint8_t> got(1024);
    for (int i = 0; i < 1024; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "READ6 zero recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(got[i]));
    }

    // After multi-block CMD18 cutover this is one CMD18 covering all
    // 256 blocks of the implicit READ(6) length-zero, not 256 CMD17s.
    CHECK_EQ("READ6 zero request count is one CMD18",
             (uint32_t)sd_mock.lba_history.size(), 1u);
    CHECK_EQ("READ6 zero base lba", sd_mock.lba_history[0],
             SD_RAW_BASE_LBA + 2u);
    CHECK_EQ("READ6 zero cmd is CMD18", sd_mock.cmd_history[0], 2);
    CHECK_EQ("READ6 zero first byte", got[0], 0xE6);
    CHECK_EQ("READ6 zero second block byte", got[512], 0xE6);
    CHECK_EQ("READ6 zero still busy after two of 256 blocks",
             bus_read(4) & SR_BSY, SR_BSY);

    bus_write(1, IC_RST);
    tick_n(4);
    bus_write(1, 0);
    tick_n(4);
    CHECK_EQ("READ6 zero reset leaves bus free", bus_read(4), 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 7 — READ 10 (non-zero LBA, 1 block) → raw offset + data.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read10_one_block() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; i++) {
        sd_mock.read_sector[i] = (uint8_t)(0xC0 ^ (i * 7));
    }

    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x12, 0x34,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ10 cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    CHECK_TRUE("READ10 entered data-in",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_IO));
    CHECK_EQ("READ10 sd_lba bias", sd_mock.last_lba,
             SD_RAW_BASE_LBA + 0x1234u);
    CHECK_EQ("READ10 sd cmd", sd_mock.last_cmd_type, 1);

    std::vector<uint8_t> got(512);
    for (int i = 0; i < 512; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ10 recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(got[i]));
    }
    for (int i = 0; i < 512; i += 41) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ10 byte[%d]", i);
        CHECK_EQ(lbl, got[i], (uint8_t)(0xC0 ^ (i * 7)));
    }

    uint8_t status, msg;
    CHECK_TRUE("READ10 status", recv_status_and_msg(status, msg));
    CHECK_EQ("READ10 status GOOD", status, 0x00);
    CHECK_EQ("READ10 msg COMPLETE", msg, 0x00);
    return true;
}


// ═══════════════════════════════════════════════════════════════════════
// Scenario: repeated READ(10) soak — many back-to-back reads must ALL
// return GOOD, with no CHECK CONDITION creeping in.
//
// Why (2026-08-18 hardware investigation): on the FPGA the .ASYC00 disk
// driver takes CHECK CONDITION on READ(10) LBA 2314 x1 and retries 16x
// before returning ioErr(-36) -- but only after ~3571 successful reads and
// ~49 s of boot, with the volume reporting ENABLED / 4194304 blocks and
// ZERO sd_ctrl errors. A single read passes (test_read10_one_block), so if
// the fault is accumulated state inside scsi.v rather than the command
// itself, only repetition exposes it.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read10_repeat_soak() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; i++)
        sd_mock.read_sector[i] = (uint8_t)(0xC0 ^ (i * 7));

    const int ITERS = 64;
    for (int it = 0; it < ITERS; it++) {
        uint32_t lba = 0x090A + (uint32_t)it;   // around the LBA that fails on HW
        char lbl[64];
        snprintf(lbl, sizeof(lbl), "soak[%d] selection", it);
        CHECK_TRUE(lbl, do_select(0));
        drop_sel();
        uint8_t cdb[10] = {0x28, 0x00,
                           (uint8_t)((lba >> 24) & 0xFF), (uint8_t)((lba >> 16) & 0xFF),
                           (uint8_t)((lba >> 8) & 0xFF),  (uint8_t)(lba & 0xFF),
                           0x00, 0x00, 0x01, 0x00};
        for (int i = 0; i < 10; i++) {
            snprintf(lbl, sizeof(lbl), "soak[%d] cdb[%d]", it, i);
            CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
        }
        snprintf(lbl, sizeof(lbl), "soak[%d] data-in", it);
        CHECK_TRUE(lbl, wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                           SR_BSY | SR_REQ | SR_IO));
        std::vector<uint8_t> got(512);
        for (int i = 0; i < 512; i++) {
            if (!recv_in_byte(got[i])) {
                printf("  FAIL soak[%d] recv[%d] (t=%lu)\n", it, i, (unsigned long)sim_time);
                return false;
            }
        }
        uint8_t status = 0xFF, msg = 0xFF;
        snprintf(lbl, sizeof(lbl), "soak[%d] status", it);
        CHECK_TRUE(lbl, recv_status_and_msg(status, msg));
        if (status != 0x00) {
            printf("  FAIL soak[%d]: status 0x%02x (CHECK CONDITION) at lba %u "
                   "after %d good reads (t=%lu)\n",
                   it, status, lba, it, (unsigned long)sim_time);
            return false;
        }
    }
    printf("  soak: %d consecutive READ(10) all GOOD\n", ITERS);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 8 — READ 6 with no backing store → timeout CHECK CONDITION.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read6_no_device_timeout() {
    reset();
    sd_mock.present = false;
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "NODEV cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    uint8_t status, msg;
    CHECK_TRUE("NODEV status", recv_status_and_msg(status, msg));
    CHECK_EQ("NODEV CHECK CONDITION", status, 0x02);
    CHECK_EQ("NODEV msg COMPLETE",    msg,    0x00);
    CHECK_EQ("NODEV busy error latched", bus_read(5) & BS_BUSYERR,
             BS_BUSYERR);

    CHECK_TRUE("re-selection", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "NODEV RS cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "NODEV RS recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("NODEV sense response", sense[0], 0x70);
    CHECK_EQ("NODEV sense key NOT READY", sense[2] & 0x0F, 0x02);
    CHECK_EQ("NODEV sense ASC medium absent", sense[12], 0x3A);
    CHECK_TRUE("NODEV RS status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("NODEV RS status GOOD", status, 0x00);
    CHECK_EQ("NODEV RS msg COMPLETE", msg, 0x00);

    (void)bus_read(7);
    tick_n(2);
    CHECK_EQ("NODEV busy error cleared", bus_read(5) & BS_BUSYERR, 0);
    CHECK_EQ("NODEV IRQ cleared", dut->irq, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 10 — READ 10 past the exposed capacity → CHECK CONDITION.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read10_lba_out_of_range() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();

    uint8_t cdb[10] = {0x28, 0x00,
                       (uint8_t)(SCSI_NUM_LBAS >> 16),
                       (uint8_t)(SCSI_NUM_LBAS >> 8),
                       (uint8_t)(SCSI_NUM_LBAS >> 0), 0x00,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ10 OOR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    uint8_t status, msg;
    CHECK_TRUE("READ10 OOR status", recv_status_and_msg(status, msg));
    CHECK_EQ("READ10 OOR CHECK CONDITION", status, 0x02);
    CHECK_EQ("READ10 OOR msg COMPLETE", msg, 0x00);
    CHECK_EQ("READ10 OOR completion latch", bus_read(5) & BS_END_DMA,
             BS_END_DMA);
    CHECK_EQ("READ10 OOR no SD cmd", sd_mock.last_cmd_type, 0);
    CHECK_EQ("READ10 OOR no SD lba", sd_mock.last_lba, 0u);

    CHECK_TRUE("re-selection", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ10 OOR RS cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ10 OOR RS recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("READ10 OOR sense response", sense[0], 0x70);
    CHECK_EQ("READ10 OOR sense key", sense[2] & 0x0F, 0x05);
    CHECK_EQ("READ10 OOR sense ASC", sense[12], 0x21);
    CHECK_TRUE("READ10 OOR RS status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("READ10 OOR RS status GOOD", status, 0x00);
    CHECK_EQ("READ10 OOR RS msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 11 — WRITE 10 past the exposed capacity → CHECK CONDITION.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write10_lba_out_of_range() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();

    uint8_t cdb[10] = {0x2A, 0x00,
                       (uint8_t)(SCSI_NUM_LBAS >> 16),
                       (uint8_t)(SCSI_NUM_LBAS >> 8),
                       (uint8_t)(SCSI_NUM_LBAS >> 0), 0x00,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE10 OOR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    for (int i = 0; i < 512; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE10 OOR push[%d]", i);
        CHECK_TRUE(lbl, push_out_byte((uint8_t)(0x80 ^ i)));
    }

    uint8_t status, msg;
    CHECK_TRUE("WRITE10 OOR status", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE10 OOR CHECK CONDITION", status, 0x02);
    CHECK_EQ("WRITE10 OOR msg COMPLETE", msg, 0x00);
    CHECK_EQ("WRITE10 OOR completion latch", bus_read(5) & BS_END_DMA,
             BS_END_DMA);
    CHECK_EQ("WRITE10 OOR no SD cmd", sd_mock.last_cmd_type, 0);
    CHECK_EQ("WRITE10 OOR no SD lba", sd_mock.last_lba, 0u);

    CHECK_TRUE("re-selection", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE10 OOR RS cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE10 OOR RS recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("WRITE10 OOR sense response", sense[0], 0x70);
    CHECK_EQ("WRITE10 OOR sense key", sense[2] & 0x0F, 0x05);
    CHECK_EQ("WRITE10 OOR sense ASC", sense[12], 0x21);
    CHECK_TRUE("WRITE10 OOR RS status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE10 OOR RS status GOOD", status, 0x00);
    CHECK_EQ("WRITE10 OOR RS msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 12 — WRITE 6 (LBA 0, 1 block) → SD sees 512 B, GOOD status.
// ═══════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════
// Write-protect — vhdd CTRL[2].  A locked volume must REFUSE writes the way
// a real target does: CHECK CONDITION with sense key 7 (DATA PROTECT) and
// ASC 0x27 (WRITE PROTECTED), and critically the card must not be touched.
// Reporting it properly matters: silently dropping the write would leave the
// guest believing it succeeded, so its in-memory filesystem state would
// diverge from the disk -- exactly the corruption this flag exists to avoid.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write_protect_refuses_write6() {
    reset();
    dut->wprot = 1;
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x0A, 0x00, 0x00, 0x01, 0x01, 0x00}; // WRITE6 lba=1
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WP WRITE6 cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    uint8_t status, msg;
    CHECK_TRUE("WP WRITE6 status", recv_status_and_msg(status, msg));
    CHECK_EQ("WP WRITE6 CHECK CONDITION", status, 0x02);
    CHECK_EQ("WP WRITE6 msg COMPLETE", msg, 0x00);
    // The card must be untouched: no write command reached the SD mock.
    CHECK_EQ("WP WRITE6 wrote nothing", (uint32_t)sd_mock.written_sector.size(), 0u);

    // And the sense data must say WRITE PROTECTED, not something generic.
    CHECK_TRUE("WP selection for sense", do_select(0));
    drop_sel();
    uint8_t sense_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x0E, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WP SENSE cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(sense_cdb[i]));
    }
    uint8_t sense[14];
    for (int i = 0; i < 14; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WP SENSE byte[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("WP sense key DATA PROTECT", sense[2] & 0x0F, 0x07);
    CHECK_EQ("WP sense ASC WRITE PROTECTED", sense[12], 0x27);
    CHECK_TRUE("WP sense status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("WP sense status GOOD", status, 0x00);

    // Unlocking restores normal service.
    dut->wprot = 0;
    return true;
}

static bool test_write6_one_block() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x0A, 0x00, 0x00, 0x01, 0x01, 0x00}; // lba=1
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE6 cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    // Push 512 bytes of pattern.
    for (int i = 0; i < 512; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE6 push[%d]", i);
        CHECK_TRUE(lbl, push_out_byte((uint8_t)(i + 0xA5)));
    }
    // The SD mock will now drain those 512 bytes once sd_go fires.
    // Status + msg
    uint8_t status, msg;
    CHECK_TRUE("WRITE6 status", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE6 status GOOD", status, 0x00);
    CHECK_EQ("WRITE6 msg COMPLETE", msg,  0x00);
    // Confirm SD mock saw the right 512 bytes at lba 1+8192.
    CHECK_EQ("WRITE6 sd_lba bias", sd_mock.last_lba, 8192 + 1);
    CHECK_EQ("WRITE6 sd cmd",      sd_mock.last_cmd_type, 3);
    CHECK_EQ("WRITE6 512 bytes",
             (uint32_t)sd_mock.written_sector.size(), 512);
    for (int i = 0; i < 512; i += 37) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE6 data[%d]", i);
        CHECK_EQ(lbl, sd_mock.written_sector[i], (uint8_t)(i + 0xA5));
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 13 — WRITE(6) transfer length 0 means 256 blocks.
//
// Six-byte WRITE follows the same legacy convention as READ(6): transfer
// length 0 means 256 blocks.  Push only two blocks and then reset; that is
// enough to prove the command is live multi-block traffic, not a no-op.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write6_zero_length_is_256_blocks() {
    reset();
    CHECK_TRUE("selection WRITE6 zero length", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x0A, 0x00, 0x00, 0x04, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "WRITE6 zero cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    // Push enough bytes for the SCSI FSM to kick CMD25 and start the
    // SD-side drain.  The test only validates that the dispatch is
    // correct (one CMD25 with block_count=256, base lba=4).  The full
    // 256-block transfer is impractical to validate here, and the
    // per-block CMD24 commit-as-you-go behavior is gone — so we don't
    // assert on written_sector beyond the first block's bytes.
    for (int i = 0; i < 512; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "WRITE6 zero push[%d]", i);
        CHECK_TRUE(lbl, push_out_byte((uint8_t)(0x20 + i)));
    }
    for (int i = 0;
         i < 4096 && (sd_mock.lba_history.empty() ||
                      sd_mock.written_sector.size() < 512);
         i++) {
        tick_n(1);
    }

    CHECK_EQ("WRITE6 zero request count is one CMD25",
             (uint32_t)sd_mock.lba_history.size(), 1u);
    CHECK_EQ("WRITE6 zero base lba", sd_mock.lba_history[0],
             SD_RAW_BASE_LBA + 4u);
    CHECK_EQ("WRITE6 zero cmd is CMD25", sd_mock.cmd_history[0], 4);
    CHECK_TRUE("WRITE6 zero at least one block committed",
               sd_mock.written_sector.size() >= 512u);
    CHECK_EQ("WRITE6 zero byte 0", sd_mock.written_sector[0],
             (uint8_t)0x20);

    bus_write(1, IC_RST);
    tick_n(4);
    bus_write(1, 0);
    tick_n(4);
    CHECK_EQ("WRITE6 zero reset leaves bus free", bus_read(4), 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 13 — WRITE 10 (non-zero LBA, 1 block) → SD offset + data.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write10_one_block() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x2A, 0x00, 0x00, 0x00, 0x23, 0x45,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE10 cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    for (int i = 0; i < 512; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE10 push[%d]", i);
        CHECK_TRUE(lbl, push_out_byte((uint8_t)(0x3D + (i * 5))));
    }

    uint8_t status, msg;
    CHECK_TRUE("WRITE10 status", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE10 status GOOD", status, 0x00);
    CHECK_EQ("WRITE10 msg COMPLETE", msg, 0x00);
    CHECK_EQ("WRITE10 sd_lba bias", sd_mock.last_lba,
             SD_RAW_BASE_LBA + 0x2345u);
    CHECK_EQ("WRITE10 sd cmd", sd_mock.last_cmd_type, 3);
    CHECK_EQ("WRITE10 512 bytes",
             (uint32_t)sd_mock.written_sector.size(), 512);
    for (int i = 0; i < 512; i += 29) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "WRITE10 data[%d]", i);
        CHECK_EQ(lbl, sd_mock.written_sector[i],
                 (uint8_t)(0x3D + (i * 5)));
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 14 — READ 10 with SD error → CHECK CONDITION + REQUEST SENSE.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read10_sd_error() {
    reset();
    sd_mock.inject_error = true;
    sd_mock.read_sector.assign(512, 0x55);
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    // READ 10: op=0x28, flags=0, lba[31..0]=0, group=0, len=0x0001, ctrl=0
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "READ10 cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    // Expect CHECK CONDITION (0x02) in status.
    uint8_t status, msg;
    CHECK_TRUE("READ10 err status", recv_status_and_msg(status, msg));
    CHECK_EQ("READ10 CHECK CONDITION", status, 0x02);
    CHECK_EQ("READ10 msg COMPLETE",    msg,    0x00);
    // Follow up with REQUEST SENSE — expect MEDIUM ERROR sense key.
    sd_mock.inject_error = false;
    CHECK_TRUE("re-selection", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RS cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RS recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("sense response code", sense[0], 0x70);
    CHECK_EQ("sense key MEDIUM ERROR", sense[2] & 0x0F, 0x03);
    CHECK_EQ("sense ASC UNRECOVERED", sense[12], 0x11);
    CHECK_TRUE("RS status/msg",
               recv_status_and_msg(status, msg));
    CHECK_EQ("RS status GOOD", status, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 14b — WRITE 10 with a backing-store error → CHECK CONDITION
// + REQUEST SENSE.  The mirror of scenario 14.
//
// Why this exists: S_VH_WAIT_WR's `vh_done && vh_error` arm (MEDIUM
// ERROR / ASC 0x0C WRITE ERROR) has been in scsi.v all along with NO
// test on it — only the READ arm was covered.  That asymmetry matters
// now that sd_ctrl.v has a global per-request watchdog: a wedged write
// is exactly as likely to surface here as a wedged read, and this is the
// path that turns it into a status the ROM can retry instead of a hang.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write10_sd_error() {
    reset();
    sd_mock.inject_error = true;
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    // WRITE 10: op=0x2A, lba=0x2345, len=0x0001
    uint8_t cdb[10] = {0x2A, 0x00, 0x00, 0x00, 0x23, 0x45,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "WRITE10 err cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    for (int i = 0; i < 512; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "WRITE10 err push[%d]", i);
        CHECK_TRUE(lbl, push_out_byte((uint8_t)(0x11 + (i * 3))));
    }

    uint8_t status, msg;
    CHECK_TRUE("WRITE10 err status", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE10 CHECK CONDITION", status, 0x02);
    CHECK_EQ("WRITE10 msg COMPLETE",    msg,    0x00);

    // REQUEST SENSE must report the WRITE-specific sense, not the READ
    // one — a copy-paste of the read arm would show ASC 0x11 here.
    sd_mock.inject_error = false;
    CHECK_TRUE("re-selection", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RS cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RS recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("sense response code", sense[0], 0x70);
    CHECK_EQ("sense key MEDIUM ERROR", sense[2] & 0x0F, 0x03);
    CHECK_EQ("sense ASC WRITE ERROR", sense[12], 0x0C);
    CHECK_TRUE("RS status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("RS status GOOD", status, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 14c — MULTI-BLOCK READ 10, provider errors MID-STREAM (i.e.
// while the target is already in S_DATA_IN, not sitting in
// S_VH_WAIT_RD) → CHECK CONDITION.
//
// This is the arm that actually matters for the sd_ctrl watchdog: the
// deadlock it exists to break parks S_RD_FLUSH_S while scsi.v holds
// vh_rd_ready low from S_DATA_IN, so the `done | error` the watchdog
// produces arrives with phase == S_DATA_IN.  scsi.v's S_VH_WAIT_RD arm
// (scenario 14) can never see it — a multi-block read enters S_DATA_IN
// on the first buffered byte and never returns to S_VH_WAIT_RD.  The
// separate mid-stream arm exists in the RTL and was untested.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read10_multiblock_midstream_sd_error() {
    reset();
    sd_mock.inject_error = true;   // mock faults 128 bytes into block 0
    sd_mock.read_sector.assign(512, 0x5A);
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    // READ 10, 2 blocks → vh_req_multi, so S_VH_WAIT_RD hands off to
    // S_DATA_IN as soon as one byte is buffered.
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x02, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[44]; snprintf(lbl, sizeof(lbl), "READ10 multi cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    // Do NOT drain: the initiator is deliberately passive, which is the
    // shape of the real hang.  Wait for the target to abort into STATUS
    // (BSY|REQ|CD|IO, MSG clear) on its own.
    CHECK_TRUE("target aborted the stream into STATUS by itself",
               wait_for_reg_match(4,
                    SR_BSY | SR_REQ | SR_MSG | SR_CD | SR_IO,
                    SR_BSY | SR_REQ |          SR_CD | SR_IO, 8192));

    uint8_t status, msg;
    CHECK_TRUE("multi-block err status", recv_status_and_msg(status, msg));
    CHECK_EQ("multi-block CHECK CONDITION", status, 0x02);
    CHECK_EQ("multi-block msg COMPLETE",    msg,    0x00);

    sd_mock.inject_error = false;
    CHECK_TRUE("re-selection", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RS cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RS recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("sense response code", sense[0], 0x70);
    CHECK_EQ("sense key MEDIUM ERROR", sense[2] & 0x0F, 0x03);
    CHECK_EQ("sense ASC UNRECOVERED", sense[12], 0x11);
    CHECK_TRUE("RS status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("RS status GOOD", status, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 15 — RST line asserted during selection → return Bus Free.
// ═══════════════════════════════════════════════════════════════════════
static bool test_reset_during_selection() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    // We're in SELECT / COMMAND phase; assert RST.
    bus_write(1, IC_RST);
    tick_n(8);
    // Clear RST
    bus_write(1, 0);
    tick_n(8);
    // Bus must be quiet: BSY=0, nothing asserted.
    CHECK_EQ("bus_status idle", bus_read(4) & SR_BSY, 0);
    // And another selection should still work.
    CHECK_TRUE("re-selection", do_select(0));
    bus_write(1, IC_RST);
    tick_n(4);
    bus_write(1, 0);
    tick_n(4);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 16 — IRQ asserts on STATUS/MSG_IN boundaries and clears after
// reading reg 7 (Reset Interrupt).
// ═══════════════════════════════════════════════════════════════════════
static bool test_irq_boundaries() {
    reset();
    CHECK_TRUE("IRQ low at reset", dut->irq == 0);
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    // Full TEST UNIT READY flow; TUR enters STATUS→MSG_IN without any
    // data phase.
    uint8_t cdb[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "TUR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    // STATUS: wait for REQ; confirm IRQ is asserted when we see REQ
    // with CD|IO|BSY (status phase bits).
    bool irq_saw = false;
    for (int i = 0; i < 64; i++) {
        uint8_t s = bus_read(4);
        if ((s & (SR_BSY|SR_CD|SR_IO|SR_REQ)) ==
            (SR_BSY|SR_CD|SR_IO|SR_REQ)) {
            if (dut->irq) { irq_saw = true; break; }
        }
        tick_n(1);
    }
    CHECK_TRUE("IRQ high in STATUS", irq_saw);
    // Pull status + msg
    uint8_t status, msg;
    CHECK_TRUE("status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("TUR status", status, 0x00);
    CHECK_EQ("TUR msg",    msg,    0x00);
    // IRQ should still be asserted after DISCONNECT (level-sensitive).
    CHECK_TRUE("IRQ still high", dut->irq == 1);
    // Read reg 7 (Reset Interrupt) → clears IRQ.
    (void)bus_read(7);
    tick_n(2);
    CHECK_TRUE("IRQ cleared after reg 7 read", dut->irq == 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 27 — STATUS-phase REQ and IRQ are visible together.
//
// The ROM commonly polls phase bits and Bus-and-Status back-to-back.  When
// STATUS becomes visible, reg5[IRQ] and the irq wire must already be high;
// otherwise software can see a ready status byte and a quiet interrupt bit
// for one poll cycle.
// ═══════════════════════════════════════════════════════════════════════
static bool test_status_irq_visible_without_extra_poll() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();

    uint8_t cdb[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[40]; snprintf(lbl, sizeof(lbl), "IRQNOW TUR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    uint8_t phase = bus_read(4);
    CHECK_EQ("STATUS phase ready",
             phase & (SR_BSY | SR_REQ | SR_CD | SR_IO),
             SR_BSY | SR_REQ | SR_CD | SR_IO);
    uint8_t bas = bus_read(5);
    CHECK_EQ("IRQ bit live with STATUS REQ", bas & BS_IRQ, BS_IRQ);
    CHECK_EQ("END_DMA bit live with STATUS REQ", bas & BS_END_DMA, BS_END_DMA);
    CHECK_EQ("DRQ low in STATUS", bas & BS_DRQ, 0);
    CHECK_EQ("irq wire live with STATUS REQ", dut->irq, 1);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("TUR status GOOD", status, 0x00);
    CHECK_EQ("TUR msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 19 — The Bus-and-Status completion latch shows command end and
// clears when the initiator reads reg 7.
// ═══════════════════════════════════════════════════════════════════════
static bool test_completion_latch_clears_on_reg7() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();

    uint8_t cdb[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "CMP TUR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("TUR status", status, 0x00);
    CHECK_EQ("TUR msg", msg, 0x00);
    CHECK_TRUE("completion latch visible in BAS",
               (bus_read(5) & BS_END_DMA) == BS_END_DMA);

    (void)bus_read(7);
    tick_n(2);
    CHECK_EQ("completion latch cleared by reg 7 read",
             bus_read(5) & BS_END_DMA, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 20 — RST clears a latched command-complete IRQ.
//
// The ROM can reset the SCSI bus after probing.  A stale level IRQ after
// RST would make the polling path see an interrupt for a command that no
// longer exists, so reset must leave both reg 5 and the irq wire quiet.
// ═══════════════════════════════════════════════════════════════════════
static bool test_reset_clears_pending_irq() {
    reset();
    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "RSTIRQ TUR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    CHECK_TRUE("status irq visible",
               wait_for_reg_match(5, BS_IRQ | BS_DRQ, BS_IRQ));
    CHECK_TRUE("irq wire high before reset", dut->irq == 1);

    bus_write(1, IC_RST);
    tick_n(4);
    CHECK_EQ("RST bit visible", bus_read(4) & 0x80, 0x80);
    CHECK_EQ("RST clears irq wire", dut->irq, 0);
    CHECK_EQ("RST clears BAS IRQ/DRQ/BUSYERR",
             bus_read(5) & (BS_IRQ | BS_DRQ | BS_BUSYERR), 0);

    bus_write(1, 0x00);
    tick_n(4);
    CHECK_EQ("bus free after reset release", bus_read(4), 0x00);
    CHECK_EQ("BAS quiet after reset release",
             bus_read(5) & (BS_IRQ | BS_DRQ | BS_BUSYERR), 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 21 — DRQ output follows the same live data-phase request as
// Bus-and-Status bit 6.  Q700 VIA2 PA6 consumes this after board-level
// inversion, so status/message IRQ phases must not leave it asserted.
// ═══════════════════════════════════════════════════════════════════════
static bool test_drq_wire_tracks_data_phase() {
    reset();
    CHECK_EQ("DRQ idle low", dut->drq, 0);

    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x04, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "DRQ INQ cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    CHECK_TRUE("entered data-in",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_IO));
    CHECK_EQ("BAS DRQ high in data phase", bus_read(5) & BS_DRQ, BS_DRQ);
    CHECK_EQ("wire DRQ high in data phase", dut->drq, 1);

    uint8_t byte = 0;
    for (int i = 0; i < 4; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "DRQ INQ byte[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(byte));
    }

    CHECK_TRUE("entered status phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO | SR_MSG,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO));
    CHECK_EQ("BAS DRQ low in status phase", bus_read(5) & BS_DRQ, 0);
    CHECK_EQ("wire DRQ low in status phase", dut->drq, 0);
    CHECK_EQ("IRQ high in status phase", dut->irq, 1);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("INQ status GOOD", status, 0x00);
    CHECK_EQ("INQ msg COMPLETE", msg, 0x00);
    CHECK_EQ("wire DRQ stays low after disconnect", dut->drq, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 23 — TEST UNIT READY reflects a known missing backing store.
//
// The first TUR after reset is optimistic because this target shim has no
// separate media-present input.  Once a raw READ times out, later readiness
// probes must fail closed with NOT READY / medium not present instead of
// telling the ROM the disk is ready.
// ═══════════════════════════════════════════════════════════════════════
static bool test_tur_reports_no_medium_after_timeout() {
    reset();
    sd_mock.present = false;

    CHECK_TRUE("selection initial TUR", do_select(0));
    drop_sel();
    uint8_t tur_cdb[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "initial TUR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(tur_cdb[i]));
    }
    uint8_t status = 0, msg = 0;
    CHECK_TRUE("initial TUR status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("initial TUR status GOOD", status, 0x00);
    CHECK_EQ("initial TUR msg COMPLETE", msg, 0x00);
    (void)bus_read(7);
    tick_n(2);

    CHECK_TRUE("selection missing-media READ", do_select(0));
    drop_sel();
    uint8_t read_cdb[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "missing READ cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(read_cdb[i]));
    }
    CHECK_TRUE("missing READ status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("missing READ CHECK", status, 0x02);
    CHECK_EQ("missing READ msg COMPLETE", msg, 0x00);
    (void)bus_read(7);
    tick_n(2);

    CHECK_TRUE("selection no-medium TUR", do_select(0));
    drop_sel();
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "no-medium TUR cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(tur_cdb[i]));
    }
    CHECK_TRUE("no-medium TUR status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("no-medium TUR CHECK", status, 0x02);
    CHECK_EQ("no-medium TUR msg COMPLETE", msg, 0x00);

    CHECK_TRUE("selection no-medium sense", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "no-medium SENSE cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "no-medium SENSE recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("no-medium sense response", sense[0], 0x70);
    CHECK_EQ("no-medium sense key", sense[2] & 0x0F, 0x02);
    CHECK_EQ("no-medium sense ASC", sense[12], 0x3A);
    CHECK_TRUE("no-medium SENSE status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("no-medium SENSE status GOOD", status, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 24 — READ(10) with transfer length zero is a successful no-op.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read10_zero_length_no_sd() {
    reset();
    sd_mock.present = false;

    CHECK_TRUE("selection READ10 zero", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x12, 0x34,
                       0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "READ10 zero cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("READ10 zero status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("READ10 zero status GOOD", status, 0x00);
    CHECK_EQ("READ10 zero msg COMPLETE", msg, 0x00);
    CHECK_EQ("READ10 zero no SD cmd", sd_mock.last_cmd_type, 0);
    CHECK_EQ("READ10 zero no SD lba", sd_mock.last_lba, 0u);
    CHECK_EQ("READ10 zero DRQ low", dut->drq, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 25 — WRITE(10) with transfer length zero is a successful no-op.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write10_zero_length_no_sd() {
    reset();
    sd_mock.present = false;

    CHECK_TRUE("selection WRITE10 zero", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x2A, 0x00, 0x00, 0x00, 0x23, 0x45,
                       0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "WRITE10 zero cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("WRITE10 zero status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE10 zero status GOOD", status, 0x00);
    CHECK_EQ("WRITE10 zero msg COMPLETE", msg, 0x00);
    CHECK_EQ("WRITE10 zero no SD cmd", sd_mock.last_cmd_type, 0);
    CHECK_EQ("WRITE10 zero no SD lba", sd_mock.last_lba, 0u);
    CHECK_EQ("WRITE10 zero no bytes", (uint32_t)sd_mock.written_sector.size(), 0u);
    CHECK_EQ("WRITE10 zero DRQ low", dut->drq, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 28 — CHECK CONDITION sense persists until REQUEST SENSE.
//
// Clearing reg7 or running a benign ROM poll command must not erase the
// sense details for the previous CHECK CONDITION.  Only REQUEST SENSE
// consumes the latched fixed-format sense data.
// ═══════════════════════════════════════════════════════════════════════
static bool test_check_condition_sense_persists() {
    reset();

    CHECK_TRUE("selection bad opcode", do_select(0));
    drop_sel();
    uint8_t bad_cdb[6] = {0x7F, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "persist bad cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(bad_cdb[i]));
    }
    uint8_t status = 0, msg = 0;
    CHECK_TRUE("bad opcode status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("bad opcode CHECK", status, 0x02);
    CHECK_EQ("bad opcode msg COMPLETE", msg, 0x00);

    (void)bus_read(7);
    tick_n(2);
    CHECK_EQ("reg7 clears IRQ only", dut->irq, 0);

    CHECK_TRUE("selection intervening INQUIRY", do_select(0));
    drop_sel();
    uint8_t inq_cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x05, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "persist INQ cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(inq_cdb[i]));
    }
    uint8_t inq[5];
    for (int i = 0; i < 5; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "persist INQ recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(inq[i]));
    }
    CHECK_EQ("intervening INQ GOOD device", inq[0], 0x00);
    CHECK_TRUE("intervening INQ status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("intervening INQ status GOOD", status, 0x00);

    CHECK_TRUE("selection persisted SENSE", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "persist SENSE cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "persist SENSE recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("persist sense response", sense[0], 0x70);
    CHECK_EQ("persist sense key illegal request", sense[2] & 0x0F, 0x05);
    CHECK_EQ("persist sense ASC invalid opcode", sense[12], 0x20);
    CHECK_TRUE("persist SENSE status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("persist SENSE status GOOD", status, 0x00);

    CHECK_TRUE("selection cleared SENSE", do_select(0));
    drop_sel();
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "clear SENSE cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    for (int i = 0; i < 18; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "clear SENSE recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("cleared sense key", sense[2] & 0x0F, 0x00);
    CHECK_EQ("cleared sense ASC", sense[12], 0x00);
    CHECK_TRUE("clear SENSE status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("clear SENSE status GOOD", status, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 29 — Pseudo-DMA data-in polling reads the same live byte from
// reg 6 as the normal Current Data register and completes with IRQ+END_DMA.
// ═══════════════════════════════════════════════════════════════════════
static bool test_pseudo_dma_reads_reg6_and_completes() {
    reset();

    CHECK_TRUE("selection pseudo DMA INQ", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x10, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "pseudo DMA cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    bus_write(7, 0x01);  // Start DMA Initiator Receive: observed trigger.
    CHECK_TRUE("pseudo DMA entered data-in",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_IO));
    CHECK_EQ("pseudo DMA DRQ visible", bus_read(5) & BS_DRQ, BS_DRQ);
    CHECK_EQ("pseudo DMA wire high", dut->drq, 1);

    uint8_t data[16];
    for (int i = 0; i < 16; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "pseudo DMA byte[%d]", i);
        CHECK_TRUE(lbl, recv_dma_in_byte(data[i]));
    }
    CHECK_EQ("pseudo DMA vendor A", data[8], 'A');
    CHECK_EQ("pseudo DMA vendor P", data[9], 'P');
    CHECK_EQ("pseudo DMA vendor P", data[10], 'P');
    CHECK_EQ("pseudo DMA vendor L", data[11], 'L');

    CHECK_TRUE("pseudo DMA status visible",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO | SR_MSG,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO));
    uint8_t bas = bus_read(5);
    CHECK_EQ("pseudo DMA status IRQ", bas & BS_IRQ, BS_IRQ);
    CHECK_EQ("pseudo DMA status END_DMA", bas & BS_END_DMA, BS_END_DMA);
    CHECK_EQ("pseudo DMA status DRQ low", bas & BS_DRQ, 0);
    CHECK_EQ("pseudo DMA wire low", dut->drq, 0);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("pseudo DMA status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("pseudo DMA status GOOD", status, 0x00);
    CHECK_EQ("pseudo DMA msg COMPLETE", msg, 0x00);

    (void)bus_read(7);
    tick_n(2);
    CHECK_EQ("pseudo DMA reset-irq clears IRQ", dut->irq, 0);
    CHECK_EQ("pseudo DMA reset-irq clears END_DMA", bus_read(5) & BS_END_DMA, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 30 — Zero-length READ(10) skips SD and exposes STATUS, IRQ, and
// END_DMA together on the first ROM-style poll.
// ═══════════════════════════════════════════════════════════════════════
static bool test_zero_length_completion_visible() {
    reset();
    sd_mock.present = false;

    CHECK_TRUE("selection zero completion READ10", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x56, 0x78,
                       0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "zero completion cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    uint8_t phase = bus_read(4);
    CHECK_EQ("zero completion status phase",
             phase & (SR_BSY | SR_REQ | SR_CD | SR_IO),
             SR_BSY | SR_REQ | SR_CD | SR_IO);
    uint8_t bas = bus_read(5);
    CHECK_EQ("zero completion IRQ", bas & BS_IRQ, BS_IRQ);
    CHECK_EQ("zero completion END_DMA", bas & BS_END_DMA, BS_END_DMA);
    CHECK_EQ("zero completion DRQ low", bas & BS_DRQ, 0);
    CHECK_EQ("zero completion irq wire", dut->irq, 1);
    CHECK_EQ("zero completion drq wire", dut->drq, 0);
    CHECK_EQ("zero completion no SD cmd", sd_mock.last_cmd_type, 0);
    CHECK_EQ("zero completion no SD lba", sd_mock.last_lba, 0u);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("zero completion status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("zero completion status GOOD", status, 0x00);
    CHECK_EQ("zero completion msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 31 — READ(10) with two blocks issues two biased raw SD reads.
//
// This catches the state-machine loop from DATA_IN back through SD_WAIT_RD:
// the second block must increment both the exposed SCSI LBA and the physical
// raw SD LBA, while keeping DRQ limited to byte-ready data phases.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read10_two_blocks_lba_sequence() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; i++) {
        sd_mock.read_sector[i] = (uint8_t)(0x31 ^ (i * 3));
    }

    CHECK_TRUE("selection READ10 two-block", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x04, 0x20,
                       0x00, 0x00, 0x02, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "READ10 two cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    std::vector<uint8_t> got(1024);
    for (int i = 0; i < 1024; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "READ10 two recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(got[i]));
    }

    // Multi-block READ(10) issues a single CMD18 spanning both blocks.
    CHECK_EQ("READ10 two request count is one CMD18",
             (uint32_t)sd_mock.lba_history.size(), 1u);
    CHECK_EQ("READ10 two base lba", sd_mock.lba_history[0],
             SD_RAW_BASE_LBA + 0x420u);
    CHECK_EQ("READ10 two cmd is CMD18", sd_mock.cmd_history[0], 2);
    CHECK_EQ("READ10 two byte 0", got[0], (uint8_t)0x31);
    CHECK_EQ("READ10 two byte 512", got[512], (uint8_t)0x31);

    CHECK_TRUE("READ10 two status phase",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO | SR_MSG,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO));
    CHECK_EQ("READ10 two DRQ low in status", bus_read(5) & BS_DRQ, 0);

    uint8_t status = 0, msg = 0;
    CHECK_TRUE("READ10 two status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("READ10 two status GOOD", status, 0x00);
    CHECK_EQ("READ10 two msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 32 — WRITE(6) with no backing store times out into CHECK.
//
// READ missing-media behavior was already covered.  The write side must fail
// closed too, latch BUSY_ERROR, preserve NOT READY sense, and never report a
// committed SD write request from the mock.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write6_no_device_timeout() {
    reset();
    sd_mock.present = false;

    CHECK_TRUE("selection WRITE6 no-device", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x0A, 0x00, 0x00, 0x03, 0x01, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "WRITE6 nodev cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    for (int i = 0; i < 512; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "WRITE6 nodev push[%d]", i);
        CHECK_TRUE(lbl, push_out_byte((uint8_t)(0x44 + i)));
    }

    CHECK_TRUE("WRITE6 nodev reached status",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  4096));
    uint8_t status = 0, msg = 0;
    CHECK_TRUE("WRITE6 nodev status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE6 nodev CHECK", status, 0x02);
    CHECK_EQ("WRITE6 nodev msg COMPLETE", msg, 0x00);
    CHECK_EQ("WRITE6 nodev busy error", bus_read(5) & BS_BUSYERR, BS_BUSYERR);
    CHECK_EQ("WRITE6 nodev mock saw no accepted request",
             (uint32_t)sd_mock.lba_history.size(), 0u);

    CHECK_TRUE("selection WRITE6 nodev sense", do_select(0));
    drop_sel();
    uint8_t rs_cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "WRITE6 nodev RS cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(rs_cdb[i]));
    }
    uint8_t sense[18];
    for (int i = 0; i < 18; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "WRITE6 nodev RS recv[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(sense[i]));
    }
    CHECK_EQ("WRITE6 nodev sense response", sense[0], 0x70);
    CHECK_EQ("WRITE6 nodev sense key NOT READY", sense[2] & 0x0F, 0x02);
    CHECK_EQ("WRITE6 nodev sense ASC", sense[12], 0x3A);
    CHECK_TRUE("WRITE6 nodev RS status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("WRITE6 nodev RS status GOOD", status, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 34 — TurboSCSI pseudo-DMA read shim advances DATA_IN
// ═══════════════════════════════════════════════════════════════════════
static bool test_turboscsi_dma_shim_read_advances() {
    reset();
    CHECK_TRUE("selection DMA shim INQ", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x12, 0x00, 0x00, 0x00, 0x04, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[48]; snprintf(lbl, sizeof(lbl), "DMA shim cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    CHECK_TRUE("DMA shim entered data-in",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_IO,
                                  SR_BSY | SR_REQ | SR_IO));
    CHECK_EQ("DMA shim DRQ visible", bus_read(5) & BS_DRQ, BS_DRQ);

    tick_n(2);
    CHECK_EQ("DMA shim inquiry byte0", bus_read9(0x100), 0x00);
    CHECK_EQ("DMA shim inquiry byte1", bus_read9(0x101), 0x00);
    CHECK_EQ("DMA shim inquiry byte2", bus_read9(0x100), 0x02);
    CHECK_EQ("DMA shim inquiry byte3", bus_read9(0x101), 0x02);

    CHECK_TRUE("DMA shim status visible",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO));
    uint8_t bas = bus_read(5);
    CHECK_EQ("DMA shim status IRQ", bas & BS_IRQ, BS_IRQ);
    CHECK_EQ("DMA shim status END_DMA", bas & BS_END_DMA, BS_END_DMA);
    CHECK_EQ("DMA shim status DRQ low", bas & BS_DRQ, 0);

    uint8_t status = 0xff, msg = 0xff;
    CHECK_TRUE("DMA shim status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("DMA shim status GOOD", status, 0x00);
    CHECK_EQ("DMA shim msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 35 — TurboSCSI pseudo-DMA write shim advances DATA_OUT
// ═══════════════════════════════════════════════════════════════════════
static bool test_turboscsi_dma_shim_write_advances() {
    reset();
    CHECK_TRUE("selection DMA shim mode-select", do_select(0));
    drop_sel();
    uint8_t cdb[6] = {0x15, 0x00, 0x00, 0x00, 0x03, 0x00};
    for (int i = 0; i < 6; i++) {
        char lbl[56]; snprintf(lbl, sizeof(lbl), "DMA shim mode cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }

    CHECK_TRUE("DMA shim entered data-out",
               wait_for_reg_match(4, SR_BSY | SR_REQ, SR_BSY | SR_REQ));
    CHECK_EQ("DMA shim write DRQ visible", bus_read(5) & BS_DRQ, BS_DRQ);

    bus_write9(0x100, 0xde);
    bus_write9(0x101, 0xad);
    bus_write9(0x100, 0xbe);

    CHECK_TRUE("DMA shim write status visible",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_CD | SR_IO));
    uint8_t bas = bus_read(5);
    CHECK_EQ("DMA shim write status IRQ", bas & BS_IRQ, BS_IRQ);
    CHECK_EQ("DMA shim write END_DMA", bas & BS_END_DMA, BS_END_DMA);
    CHECK_EQ("DMA shim write DRQ low", bas & BS_DRQ, 0);

    uint8_t status = 0xff, msg = 0xff;
    CHECK_TRUE("DMA shim write status/msg", recv_status_and_msg(status, msg));
    CHECK_EQ("DMA shim write status GOOD", status, 0x00);
    CHECK_EQ("DMA shim write msg COMPLETE", msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Disconnect / reselection helpers and scenarios
// ═══════════════════════════════════════════════════════════════════════
//
// The DISCONNECT message protocol per SCSI-2 (and MAME ncr53c90.cpp):
//   1. While target is in DATA phase (REQ asserted), initiator raises ATN.
//   2. Target snaps to MSG_OUT phase: REQ asserted with C/D=1, MSG=1, IO=0.
//   3. Initiator pushes message byte 0x04 (DISCONNECT) via REQ/ACK.
//   4. Target sends 0x04 back via MSG_IN, sets I_DISCONNECT, drops BSY.
//   5. Bus enters BUS_FREE; the suspended state is preserved internally.

// Pull a MSG_IN byte (BSY|MSG|C/D|I/O all asserted, plus REQ).
static bool recv_msg_in_byte(uint8_t& out, int timeout = 1024) {
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if ((s & (SR_REQ | SR_MSG | SR_CD | SR_IO)) ==
            (SR_REQ | SR_MSG | SR_CD | SR_IO)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    out = bus_read(0);
    bus_write(1, IC_ACK);
    for (i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_REQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(1, 0);
    tick_n(2);
    return true;
}

// Push a MSG_OUT byte: ATN already asserted by caller.  Wait for REQ in
// MSG_OUT phase (BSY|MSG|C/D, no I/O), drive byte, ACK, drop ACK + ATN.
static bool push_msg_out_byte(uint8_t b, int timeout = 1024) {
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        // MSG_OUT: BSY|MSG|C/D + REQ; I/O is LOW.
        if ((s & (SR_REQ | SR_MSG | SR_CD)) == (SR_REQ | SR_MSG | SR_CD) &&
            !(s & SR_IO)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(0, b);
    bus_write(1, IC_ACK | IC_ATN | IC_DB);  // keep ATN; ACK
    // Wait for REQ to drop
    for (i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_REQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(1, IC_DB);   // drop ATN + ACK; keep DB so SEL-group-zero quirks don't fire
    tick_n(2);
    bus_write(1, 0);
    tick_n(2);
    return true;
}

// Scenario 36 — DISCONNECT during DATA_IN suspends and drops BSY.
//
// MAME refs: ncr53c90.h:153 (I_DISCONNECT=0x20), ncr53c90.cpp:972-989
// (CD_RESELECT command resumes), SCSI-2 spec MSG_OUT 0x04 semantics.
//
// Mac OS issues this exact sequence during long-latency disk I/O.
static bool test_disconnect_during_data_in() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; i++) sd_mock.read_sector[i] = (uint8_t)(0xA5 ^ i);

    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x00, 0x10,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "DISC cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    CHECK_TRUE("entered DATA_IN",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_IO, SR_BSY | SR_REQ | SR_IO));

    // Pull a few bytes normally to confirm the data path works.
    uint8_t got;
    for (int i = 0; i < 4; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "pre-disconnect byte[%d]", i);
        CHECK_TRUE(lbl, recv_in_byte(got));
        CHECK_EQ(lbl, got, (uint8_t)(0xA5 ^ i));
    }

    // Now assert ATN to interrupt the data phase and force MSG_OUT.
    bus_write(1, IC_ATN);
    tick_n(2);
    // DUT should snap into MSG_OUT (BSY|MSG|C/D, no I/O) with REQ.
    CHECK_TRUE("snapped to MSG_OUT",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_MSG | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_MSG | SR_CD));
    // Push DISCONNECT (0x04) byte; drops ATN at end.
    CHECK_TRUE("pushed DISCONNECT byte", push_msg_out_byte(0x04));

    // DUT now sends 0x04 back via MSG_IN and asserts IRQ.
    uint8_t msg = 0;
    CHECK_TRUE("recv DISCONNECT echo via MSG_IN", recv_msg_in_byte(msg));
    CHECK_EQ("MSG_IN byte = 0x04 (DISCONNECT)", msg, 0x04);

    // BSY should drop (BUS_FREE) within a few ticks; IRQ remains until reg7
    // is read.  Bus-and-Status[4]=IRQ should be set.
    CHECK_TRUE("BSY drops to BUS_FREE",
               wait_for_reg_match(4, SR_BSY, 0));
    uint8_t bas = bus_read(5);
    CHECK_EQ("IRQ asserted post-disconnect", bas & BS_IRQ, BS_IRQ);
    CHECK_EQ("END_DMA latched post-disconnect", bas & BS_END_DMA, BS_END_DMA);
    return true;
}

// Scenario 37 — Non-DISCONNECT MSG_OUT byte (e.g. IDENTIFY 0x80) is
// consumed and the data phase resumes; no disconnect side effect.
static bool test_msg_out_non_disconnect_resumes() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; i++) sd_mock.read_sector[i] = (uint8_t)(i ^ 0x33);

    CHECK_TRUE("selection", do_select(0));
    drop_sel();
    uint8_t cdb[10] = {0x28, 0x00, 0x00, 0x00, 0x00, 0x20,
                       0x00, 0x00, 0x01, 0x00};
    for (int i = 0; i < 10; i++) {
        char lbl[32]; snprintf(lbl, sizeof(lbl), "MSGOUT cdb[%d]", i);
        CHECK_TRUE(lbl, send_cdb_byte(cdb[i]));
    }
    CHECK_TRUE("entered DATA_IN",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_IO, SR_BSY | SR_REQ | SR_IO));

    // Pull one byte, then assert ATN.
    uint8_t got;
    CHECK_TRUE("byte 0 pre-MSGOUT", recv_in_byte(got));
    CHECK_EQ("byte 0 value", got, (uint8_t)(0 ^ 0x33));

    bus_write(1, IC_ATN);
    tick_n(2);
    CHECK_TRUE("MSG_OUT phase visible",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_MSG | SR_CD | SR_IO,
                                  SR_BSY | SR_REQ | SR_MSG | SR_CD));
    // Send IDENTIFY (0x80) — non-DISCONNECT.  DUT should resume DATA_IN.
    CHECK_TRUE("pushed IDENTIFY 0x80", push_msg_out_byte(0x80));

    // We should be back in DATA_IN.
    CHECK_TRUE("resumed DATA_IN",
               wait_for_reg_match(4, SR_BSY | SR_REQ | SR_IO, SR_BSY | SR_REQ | SR_IO));
    // Pull the next byte — should be byte 1 of the sector.
    CHECK_TRUE("byte 1 post-resume", recv_in_byte(got));
    CHECK_EQ("byte 1 value", got, (uint8_t)(1 ^ 0x33));
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_vhdd_sd;

    RUN(test_reset_defaults_and_decode);
    RUN(test_handshake_status_bits);
    RUN(test_bus_free_to_selection_id0);
    RUN(test_selection_non_zero_id_ignored);
    RUN(test_rom_benign_unselected_probe);
    RUN(test_inquiry_6_byte);
    RUN(test_fixed_response_allocation_limits);
    RUN(test_read_capacity_raw_geometry);
    RUN(test_mode_sense6_header);
    RUN(test_read6_raw_frontier);
    RUN(test_read6_zero_length_is_256_blocks);
    RUN(test_read10_one_block);
    RUN(test_read10_repeat_soak);
    RUN(test_read6_no_device_timeout);
    RUN(test_read10_lba_out_of_range);
    RUN(test_write10_lba_out_of_range);
    RUN(test_write_protect_refuses_write6);
    RUN(test_write6_one_block);
    RUN(test_write6_zero_length_is_256_blocks);
    RUN(test_write10_one_block);
    RUN(test_read10_sd_error);
    RUN(test_write10_sd_error);
    RUN(test_read10_multiblock_midstream_sd_error);
    RUN(test_reset_during_selection);
    RUN(test_irq_boundaries);
    RUN(test_status_irq_visible_without_extra_poll);
    RUN(test_phase_match_tracks_target_command);
    RUN(test_completion_latch_clears_on_reg7);
    RUN(test_reset_clears_pending_irq);
    RUN(test_drq_wire_tracks_data_phase);
    RUN(test_tur_reports_no_medium_after_timeout);
    RUN(test_read10_zero_length_no_sd);
    RUN(test_write10_zero_length_no_sd);
    RUN(test_check_condition_sense_persists);
    RUN(test_pseudo_dma_reads_reg6_and_completes);
    RUN(test_zero_length_completion_visible);
    RUN(test_read10_two_blocks_lba_sequence);
    RUN(test_write6_no_device_timeout);
    RUN(test_turboscsi_dma_shim_read_advances);
    RUN(test_turboscsi_dma_shim_write_advances);
    RUN(test_disconnect_during_data_in);
    RUN(test_msg_out_non_disconnect_resumes);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
