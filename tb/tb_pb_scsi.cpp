// tb_pb_scsi.cpp — integration tb for T1b (scsi-pb-reconcile): drives the
// REAL peripheral_bus.v AXI4 slave port down into a REAL scsi.v
// (TURBOSCSI_C96=1), through the tb_pb_scsi.v wrapper.
//
// Every pre-existing SCSI tb (tb_turboscsi.cpp, tb_scsi_c96_read6.cpp,
// tb_scsi.cpp) drives scsi.v's pb_addr/pb_wdata/pb_wr/pb_rd ports
// directly, bypassing peripheral_bus.v.  None of them can exercise:
//   - The SCSI DMA-shim handshake through peripheral_bus's real FSM:
//     scsi.v's TurboSCSI DRQ-check can withhold pb_ack for many cycles
//     (DRQ low); peripheral_bus must not fire a scsi_rd/scsi_wr pulse on
//     a cycle where the ack would be withheld (or that beat's ack is
//     lost forever — a permanent AXI/fabric wedge, since this slave only
//     supports one outstanding R and one outstanding W).  This base
//     branch's mechanism (commits f22b26d / eba8aa2, 2026-07-21) gates
//     the pulse on scsi.v's exported dma_rd_ready/dma_wr_ready — but
//     only covered the WORD-size (movew) DMA-shim path; the T1b
//     reconciliation found (via this tb, against unmodified base RTL)
//     that a plain BYTE-size DMA-shim read still fell through to the
//     ungated generic one-shot pulse and could lose its ack.  Widened
//     rd_scsi_dma_shim_active (rtl/soc/peripheral_bus.v, was
//     rd_scsi_word_active) to cover both sizes.
//   - The bounded ack watchdog (peripheral_bus.v's PB_WATCHDOG_LOG2 /
//     PB_ACK_TIMEOUT, landed independently on this base as commit
//     26b7146): aborts with SLVERR if any beat's ack doesn't arrive in
//     time, so no peripheral can hang the fabric.  Production default
//     2^24; this wrapper overrides to 2^12 so the timeout scenarios
//     stay fast in sim — see tb_pb_scsi.v.
//   - The same-slot concurrent R+W interlock (landed independently on
//     this base as commit 63cde88): peripheral_bus's independent
//     write/read FSMs mux one shared per-peripheral address port
//     write-first; a concurrent R+W used to present the WRITE's address
//     during the READ's pb_rd pulse.
//
// Scenarios:
//   1. register_pass_through_via_axi — basic AXI -> peripheral_bus ->
//      scsi.v wiring sanity (proves the harness itself works before
//      trusting the handshake-specific scenarios).
//   2. drq_checked_read_rises_later — DRQ-checked DMA-shim READ (byte
//      size) while DRQ is low; DRQ rises later (a slow-paced CMD18
//      mid-chunk refill); the beat completes with correct data, OKAY
//      response, and the bus stays alive afterwards.  This is the
//      scenario that failed against unmodified base RTL (fail-before
//      evidence for the byte-access gap above) before the
//      rd_scsi_dma_shim_active widening.
//   3. drq_checked_write_rises_later — DRQ-checked DMA-shim WRITE while
//      DRQ is genuinely low (a real small tcount exhausted via real
//      AXI beats); the held wr_scsi_dma_shim_active beat (byte latched
//      into wr_scsi_byte_q) completes the instant drq_c96 goes true —
//      see the note further below for why the "rise" is driven via an
//      internal poke rather than a second concurrent AXI write.  This
//      scenario already passed against unmodified base RTL (the write
//      side's gate was never size-restricted).
//   4. drq_never_rises_times_out — DRQ-checked DMA-shim READ with DRQ
//      permanently low (BUS_FREE, no selection at all — every drq_c96
//      OR-term is structurally false); the beat aborts with SLVERR
//      after the watchdog, and a following unrelated VIA1 access still
//      succeeds.
//   5. drq_write_never_rises_times_out — same as (4) but for the WRITE
//      side (wr_scsi_dma_shim_active never sees scsi_ack).
//   6. concurrent_write_read_same_peripheral — AWVALID+ARVALID present
//      the same cycle, both targeting SCSI but DIFFERENT registers; the
//      write completes first, and the concurrent read returns its OWN
//      register's value, not the write's address/data.
//
// Build via: make tb-pb-scsi
//
// Note on scenario 3 (drq_checked_write_rises_later): unlike the READ
// side, the WRITE side's drq_c96 term
// (`(phase==S_DATA_OUT) && t_req && c96_xfr_armed && c96_xfr_dma &&
// (c96_tcounter != 0)`, rtl/mac/scsi.v) has NO input that an external
// SD-timing mock can move asynchronously: t_req is driven purely by CPU
// actions (pb_wr beats / the reg-3 command register) and, once
// asserted for a CMD25 ring transfer, is never cleared by SD-side drain
// progress (draining only touches sd_buf_count/sd_drain_ptr, confirmed
// by direct RTL trace) — and c96_tcounter only reloads on a reg-3
// write with the DMA bit set.  Reaching "DRQ low, then rises" for
// writes therefore genuinely requires a SECOND, concurrent AXI write
// (the tcount reload) while the first (gated) write is still pending —
// impossible through this single-outstanding-write AXI slave (and
// doubly so given the same-slot R+W acceptance interlock, which
// serializes acceptance).  The test instead gets DRQ low the honest way
// (exhausts a real, small tcount via real AXI pushes) and simulates the
// second actor's effect (a rearm write landing) by directly poking
// scsi.v's internal c96_tcounter (and c96_xfr_armed — see the poke
// site's comment for why both) via Verilator's --public-flat-rw while
// the gated write is genuinely in flight on the AXI bus (driven with a
// manual, non-blocking loop instead of the axi_write_full() helper so
// the poke can land mid-transaction).  This exercises exactly what
// matters here — that wr_scsi_dma_shim_active correctly holds pb_wr and
// completes the
// instant drq_c96 goes true — without needing scsi.v's write-side
// back-pressure model to support async progress it doesn't have.

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <verilated.h>
#include "Vtb_pb_scsi.h"
#include "Vtb_pb_scsi___024root.h"

static Vtb_pb_scsi* dut = nullptr;
static int n_pass = 0;
static int n_fail = 0;

using Word128 = std::array<uint32_t, 4>;

template <typename Port>
static void set128(Port& p, const Word128& v) {
    for (int i = 0; i < 4; i++) p[i] = v[i];
}
template <typename Port>
static Word128 get128(Port& p) {
    Word128 v{};
    for (int i = 0; i < 4; i++) v[i] = p[i];
    return v;
}

// ─── Mocked SD backing store (trimmed from tb_scsi_c96_read6.cpp) ───────
struct SdMock {
    std::vector<uint8_t> read_sector;
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    bool     present = true;
    // Cycles between rd_valid pulses once streaming (READ direction) —
    // the knob used to create a genuine "DRQ low now, rises later"
    // window mid-CMD18-chunk (see drq_checked_read_rises_later).
    int      gap = 0;
    // Cycles between wr_ready pulses once draining (WRITE direction) —
    // the equivalent knob for drq_checked_write_rises_later: paced
    // slowly enough that the CMD25 write ring is still full (and
    // t_req/DRQ still low) by the time the test attempts the next
    // gated DMA-shim write.
    int      wgap = 0;
    int      skid_max = 2;
    int      skid = 2;
    // Extra cycles before the FIRST byte of a kicked request appears —
    // i.e. how long scsi.v's FSM is parked in S_VH_WAIT_RD / S_VH_WAIT_WR
    // waiting on the provider.  DEFAULT 0 = the historical 2-cycle mock,
    // so no existing scenario changes.  A real SPI card at 25 MHz needs
    // ~320 ns/byte, so a 512-byte block is ~164 us == ~16,400 core cycles
    // at 100 MHz; the board's wedge window is that wide, sim's was ~2.
    int      go_latency = 0;

    enum class State { Idle, Reading, Writing, Done } st = State::Idle;
    int cnt = 0;
    int total = 512;
    int delay = 0;
    int gap_ctr = 0;
} sd_mock;

static void sd_mock_tick() {
    dut->sd_busy     = (sd_mock.st != SdMock::State::Idle) ? 1 : 0;
    dut->sd_done     = 0;
    dut->sd_error    = 0;
    dut->sd_rd_valid = 0;
    dut->sd_rd_data  = 0;
    dut->sd_wr_ready = 0;

    if (sd_mock.present && sd_mock.st == SdMock::State::Idle && dut->sd_go) {
        sd_mock.last_lba      = dut->sd_lba;
        sd_mock.last_cmd_type = dut->sd_cmd_type;
        sd_mock.cnt     = 0;
        sd_mock.delay   = 2 + sd_mock.go_latency;
        sd_mock.gap_ctr = 0;
        // cmd_type: 1=CMD17 (read, single), 2=CMD18 (read, ring),
        // 3=CMD24 (write, single), 4=CMD25 (write, ring).
        bool is_write = (dut->sd_cmd_type == 3 || dut->sd_cmd_type == 4);
        sd_mock.st    = is_write ? SdMock::State::Writing
                                  : SdMock::State::Reading;
        sd_mock.total = (dut->sd_cmd_type == 2)
                         ? 512 * (int)dut->sd_block_count
                         : 512;
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
            return;
        }
        if (sd_mock.cnt < sd_mock.total) {
            dut->sd_rd_valid = 1;
            uint8_t byte = 0;
            if (!sd_mock.read_sector.empty())
                byte = sd_mock.read_sector[sd_mock.cnt % sd_mock.read_sector.size()];
            dut->sd_rd_data = byte;
            ++sd_mock.cnt;
            sd_mock.gap_ctr = sd_mock.gap;
            if (sd_mock.cnt == sd_mock.total) sd_mock.st = SdMock::State::Done;
        }
    } else if (sd_mock.st == SdMock::State::Writing) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) { --sd_mock.delay; return; }
        if (sd_mock.gap_ctr > 0) { --sd_mock.gap_ctr; return; }
        dut->sd_wr_ready = 1;
        ++sd_mock.cnt;
        sd_mock.gap_ctr = sd_mock.wgap;
        if (sd_mock.cnt >= sd_mock.total) sd_mock.st = SdMock::State::Done;
    } else if (sd_mock.st == SdMock::State::Done) {
        dut->sd_busy = 0;
        dut->sd_done = 1;
        sd_mock.st = SdMock::State::Idle;
    }
}

static void tick() {
    sd_mock_tick();
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
}

static void reset() {
    dut->s_awid = 0; dut->s_awaddr = 0; dut->s_awlen = 0;
    dut->s_awsize = 0; dut->s_awburst = 0; dut->s_awvalid = 0;
    set128(dut->s_wdata, Word128{0, 0, 0, 0});
    dut->s_wstrb = 0; dut->s_wlast = 0; dut->s_wvalid = 0;
    dut->s_bready = 1;
    dut->s_arid = 0; dut->s_araddr = 0; dut->s_arlen = 0;
    dut->s_arsize = 0; dut->s_arburst = 0; dut->s_arvalid = 0;
    dut->s_rready = 1;
    dut->scsi_ctrl_in = 0;
    // tb_pb_scsi.v used to hardcode this; it is a port now so the SCSI
    // differential fuzzer can reuse the same top with its own disk size.
    dut->disk_num_lbas = 1048576;
    sd_mock = SdMock();
    dut->periph_rst = 0;   // peripheral-only reset idle (see tb_pb_scsi.v)
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
}

// ─── AXI helpers ─────────────────────────────────────────────────────
struct AxiReadResult { uint32_t data; uint32_t resp; bool completed; };

static AxiReadResult axi_read_full(uint32_t addr, uint8_t arsize, int max_cycles) {
    dut->s_arid    = 0x3;
    dut->s_araddr  = addr;
    dut->s_arlen   = 0;
    dut->s_arsize  = arsize;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    uint32_t got = 0, resp = 0;
    bool ar_done = false, r_done = false;
    for (int i = 0; i < max_cycles && !r_done; i++) {
        dut->eval();
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            got  = v[lane];
            resp = dut->s_rresp;
        }
        tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) r_done = true;
    }
    dut->s_arvalid = 0;
    for (int i = 0; i < 2; i++) tick();
    return {got, resp, r_done};
}

static uint32_t axi_read(uint32_t addr, uint32_t* rresp = nullptr,
                         uint8_t arsize = 4, int max_cycles = 400) {
    AxiReadResult r = axi_read_full(addr, arsize, max_cycles);
    if (rresp) *rresp = r.resp;
    return r.data;
}

struct AxiWriteResult { uint32_t resp; bool completed; };

static AxiWriteResult axi_write_full(uint32_t addr, uint8_t byte, int max_cycles) {
    dut->s_awid    = 0xC;
    dut->s_awaddr  = addr;
    dut->s_awlen   = 0;
    dut->s_awsize  = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    Word128 w{0, 0, 0, 0};
    w[lane] = byte;
    set128(dut->s_wdata, w);
    dut->s_wstrb  = (uint16_t)(1u << (lane * 4 + 0));
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    uint32_t resp = 0;
    for (int i = 0; i < max_cycles && !b_done; i++) {
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) resp = dut->s_bresp;
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    for (int i = 0; i < 2; i++) tick();
    return {resp, b_done};
}

static void axi_write(uint32_t addr, uint8_t byte, uint32_t* bresp = nullptr) {
    AxiWriteResult r = axi_write_full(addr, byte, 400);
    if (bresp) *bresp = r.resp;
}

// Drive AW+W and AR concurrently (same cycle), matching
// tb_peripheral_bus.cpp's axi_write_read_concurrent — proves Bug 3's
// accept-level serialization (write wins, read waits, both complete).
struct ConcurrentResult {
    bool write_ok, read_ok;
    uint32_t read_data, read_resp, write_resp;
};
static ConcurrentResult axi_write_read_concurrent(uint32_t waddr, uint8_t wbyte,
                                                  uint32_t raddr, int max_cycles) {
    ConcurrentResult r{false, false, 0, 0, 0};

    dut->s_awid    = 0xC;
    dut->s_awaddr  = waddr;
    dut->s_awlen   = 0;
    dut->s_awsize  = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    uint32_t wlane = (waddr >> 2) & 0x3;
    Word128 wv{0, 0, 0, 0};
    wv[wlane] = wbyte;
    set128(dut->s_wdata, wv);
    dut->s_wstrb  = (uint16_t)(1u << (wlane * 4 + 0));
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    dut->s_arid    = 0x3;
    dut->s_araddr  = raddr;
    dut->s_arlen   = 0;
    dut->s_arsize  = 0;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 1;
    uint32_t rlane = (raddr >> 2) & 0x3;

    bool aw_done = false, w_done = false, b_done = false;
    bool ar_done = false, rd_done = false;
    for (int i = 0; i < max_cycles && !(b_done && rd_done); i++) {
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (b_hs) r.write_resp = dut->s_bresp;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            r.read_data = v[rlane];
            r.read_resp = dut->s_rresp;
        }
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) rd_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0; dut->s_arvalid = 0;
    r.write_ok = b_done;
    r.read_ok  = rd_done;
    for (int i = 0; i < 2; i++) tick();
    return r;
}

// ─── SCSI register-file convenience wrappers ────────────────────────────
// Register N lives at raw offset 0xF000 + (N<<4) (DAFB 16-byte stride,
// peripheral_bus collapses to addr[7:4]).  The DMA shim is 0xF100.
static constexpr uint32_t SCSI_BASE     = 0x000F000u;
static constexpr uint32_t SCSI_DMA_SHIM = 0x000F100u;

static void reg_w(uint8_t off, uint8_t v) {
    axi_write(SCSI_BASE | ((uint32_t)off << 4), v);
}
static uint8_t reg_r(uint8_t off) {
    return (uint8_t)axi_read(SCSI_BASE | ((uint32_t)off << 4), nullptr, 4, 400);
}
// Returns the byte and (via out-params) both the AXI resp AND whether the
// beat actually completed within the cycle budget.  Callers MUST check
// `completed` before trusting `resp` — on a wedged (pre-fix) beat the
// read never fires an R handshake, resp stays at its uninitialized-read
// default (0 == OKAY), so a bare `resp == OKAY` check passes spuriously
// even though nothing actually completed (reviewer-caught gap).
static uint8_t shim_r(uint32_t* resp, bool* completed, int max_cycles = 400) {
    AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, /*arsize=byte*/0, max_cycles);
    if (resp) *resp = r.resp;
    if (completed) *completed = r.completed;
    return (uint8_t)r.data;
}

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { std::printf("  FAIL %s\n", name); return false; } \
} while (0)
#define CHECK_EQ(name, got, exp) do { \
    uint32_t _g = (uint32_t)(got); uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        std::printf("  FAIL %s: got 0x%x, expected 0x%x\n", name, _g, _e); \
        return false; \
    } \
} while (0)
#define RUN(fn) do { \
    std::printf("[RUN ] %s\n", #fn); \
    bool ok = fn(); \
    if (ok) { ++n_pass; std::printf("[PASS] %s\n", #fn); } \
    else    { ++n_fail; std::printf("[FAIL] %s\n", #fn); } \
} while (0)

// ─────────────────────────────────────────────────────────────────────
// 1. Basic AXI -> peripheral_bus -> scsi.v wiring sanity.
// ─────────────────────────────────────────────────────────────────────
static bool test_register_pass_through_via_axi() {
    reset();
    reg_w(0x3, 0x02);   // CM_RESET
    CHECK_EQ("post-reset status @ reg4", reg_r(0x4), 0x00);
    CHECK_EQ("post-reset interrupt @ reg5", reg_r(0x5), 0x00);

    reg_w(0x8, 0x11);
    reg_w(0xB, 0x22);
    reg_w(0xC, 0x33);
    CHECK_EQ("config1 round-trip", reg_r(0x8), 0x11);
    CHECK_EQ("config2 round-trip", reg_r(0xB), 0x22);
    CHECK_EQ("config3 round-trip", reg_r(0xC), 0x33);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 2. Bug 1: DRQ-checked DMA-shim READ while DRQ is low; DRQ rises
// later; the beat completes with correct data and the bus stays alive.
// ─────────────────────────────────────────────────────────────────────
static bool test_drq_checked_read_rises_later() {
    reset();
    // 2-block READ(6) -> CMD18 ring path: scsi.v enters DATA_IN (and
    // fires select-complete) as soon as the FIRST byte is buffered, not
    // the whole transfer (see scsi.v's phase<=S_DATA_IN CMD18 special-
    // case).  With a generous SD pacing gap, byte 0 is ready almost
    // immediately but byte 1 lags well behind — a genuine "DRQ low now,
    // rises later" window on the chunk's SECOND DMA-shim beat.
    sd_mock.read_sector.resize(2 * 512);
    for (int i = 0; i < 2 * 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x40 ^ (i & 0xFF) ^ (i >> 5));
    sd_mock.gap = 400;

    reg_w(0x4, 0x00);   // bus_id = TARGET_ID (0)
    reg_w(0x3, 0x01);   // FLUSH_FIFO
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);   // tcount = 1 (select's DMA counter covers the tail byte)
    reg_w(0x3, 0xC1);   // DMA | CD_SELECT, empty FIFO

    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", seq_ok);

    // CDB: READ(6) opcode 0x08, lba=0, blocks=2 -> CMD18.
    reg_w(0x2, 0x08);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x02);
    axi_write(SCSI_DMA_SHIM, 0x00);   // control byte via the DMA port tail

    uint8_t v = 0;
    bool select_ok = false;
    // 2026-09-07 pre-staging: select-complete for a multi-block read now
    // waits for the ring to STAGE (RING_HIGH_WATER bytes) before DATA_IN
    // entry — at this test's extreme gap=400 pacing that is ~200k ticks.
    // The ROM's own select wait is Ticks-scaled and covers this easily.
    for (int i = 0; i < 300000; ++i) {
        v = reg_r(0x4);
        if (v & 0x80) { select_ok = true; break; }
    }
    CHECK_TRUE("select-complete interrupt fires", select_ok);
    CHECK_EQ("select-complete phase = DATA_IN", v & 0x07, 0x01);
    CHECK_EQ("select-complete istatus = I_FUNCTION|I_BUS", reg_r(0x5), 0x18);

    // 2026-09-07 RECALIBRATION (supply pre-staging): DATA_IN entry now
    // waits until the ring is staged to RING_HIGH_WATER, so a small
    // chunk armed right after select-complete can no longer catch the
    // supply lagging — the original 2-byte-chunk window is gone BY
    // DESIGN.  The genuine "DRQ low now, rises later" window now lives
    // where pre-staging cannot reach: MID-chunk, once a blind drain has
    // consumed the staged headroom and the gap=400 supply crawl is all
    // that feeds the FIFO.  Recreate exactly that: arm ONE chunk
    // covering the whole 1024-byte payload, drain past the headroom
    // blind, then flip the DRQ-check on and prove the checked beat is
    // withheld (not lost) until the next byte lands.
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x04);   // tcount = 1024 — the whole 2-block payload
    reg_w(0x3, 0x90);   // DMA | CI_XFER

    // Blind-drain beyond the staged headroom (RING_HIGH_WATER = 496
    // plus whatever trickled in): 600 beats leaves the ring empty and
    // the transfer mid-chunk, supply crawling at 400 cycles/byte.
    for (int i = 0; i < 600; ++i) {
        uint32_t r = 99; bool c = false;
        uint8_t b = shim_r(&r, &c, /*max_cycles=*/20000);
        CHECK_TRUE("blind drain beat completes (paced by the supply)", c);
        CHECK_EQ("blind drain byte value", b, sd_mock.read_sector[i]);
    }

    // Enable the read DRQ-check on the DMA-shim window.
    dut->scsi_ctrl_in = 0x080;   // bit 7
    dut->eval();
    CHECK_TRUE("DRQ is low mid-chunk once the drain outran the supply",
               dut->scsi_drq == 0);

    // The checked beat: withheld until the crawling supply lands the
    // next byte, then completes with it — the property this test
    // exists to prove (a pulse issued only when dma_rd_ready allows,
    // never a lost beat).
    uint32_t resp0 = 99;
    bool completed0 = false;
    uint8_t b0 = shim_r(&resp0, &completed0, /*max_cycles=*/20000);
    CHECK_TRUE("checked beat completed after the supply caught up",
               completed0);
    CHECK_EQ("checked beat resp OKAY", resp0, 0);
    CHECK_EQ("checked beat value", b0, sd_mock.read_sector[600]);

    uint32_t resp1 = 99;
    bool completed1 = false;
    uint8_t b1 = shim_r(&resp1, &completed1, /*max_cycles=*/20000);
    // Reviewer-caught gap (2026-08-19, kept): `completed` is the
    // load-bearing check — a silently-defaulted resp of 0 must not pass.
    CHECK_TRUE("second checked beat actually completed (not silently "
               "defaulted)", completed1);
    CHECK_EQ("second checked beat resp OKAY (beat completed, not stuck)",
             resp1, 0);
    CHECK_EQ("second checked beat value", b1, sd_mock.read_sector[601]);

    // Bus alive afterwards: an unrelated register read still succeeds.
    dut->scsi_ctrl_in = 0;
    uint32_t seqresp = 99;
    (void)axi_read(SCSI_BASE | (0x6u << 4), &seqresp, 4, 400);
    CHECK_EQ("bus alive: seq_step read resp OKAY after the beat", seqresp, 0);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 3. Bug 1 (write side): DRQ-checked DMA-shim WRITE while DRQ is low;
// DRQ rises later; the held wr_scsi_dma_shim_active beat completes.
// ─────────────────────────────────────────────────────────────────────
static bool test_drq_checked_write_rises_later() {
    reset();
    // WRITE(6), 2 blocks -> S_DATA_OUT / CMD25.  See the file-header note
    // for why the "DRQ rises" half of this scenario pokes c96_tcounter
    // directly instead of a second concurrent AXI write.
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);   // FLUSH_FIFO
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);   // tcount = 1 (select's DMA counter covers the tail byte)
    reg_w(0x3, 0xC1);   // DMA | CD_SELECT, empty FIFO

    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", seq_ok);

    // CDB: WRITE(6) opcode 0x0A, lba=0, blocks=2, tail=control(0).
    reg_w(0x2, 0x0A);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x02);
    axi_write(SCSI_DMA_SHIM, 0x00);   // control byte via the DMA port tail

    uint8_t v = 0;
    bool select_ok = false;
    for (int i = 0; i < 20000; ++i) {
        v = reg_r(0x4);
        if (v & 0x80) { select_ok = true; break; }
    }
    CHECK_TRUE("select-complete interrupt fires", select_ok);
    CHECK_EQ("select-complete phase = DATA_OUT", v & 0x07, 0x00);
    CHECK_EQ("select-complete istatus = I_FUNCTION|I_BUS", reg_r(0x5), 0x18);

    // Arm a genuinely small chunk (tcount=2): drq_c96's write term
    // requires c96_tcounter != 0, and it only decrements on real
    // accepted beats — so pushing exactly 2 real bytes exhausts it
    // deterministically, no pacing/timing guesswork needed (unlike the
    // read side's SD-gap approach).
    reg_w(0x0, 0x02);
    reg_w(0x1, 0x00);
    reg_w(0x3, 0x90);   // DMA | CI_XFER

    // Push both tcount bytes for real (DRQ-check still disabled —
    // t_req is already 1 from the S_DATA_OUT dispatch and tcounter is
    // nonzero for both, so neither beat is gated).
    axi_write(SCSI_DMA_SHIM, 0xAA);
    axi_write(SCSI_DMA_SHIM, 0xBB);
    CHECK_EQ("c96_tcounter exhausted after 2 real beats",
             dut->rootp->tb_pb_scsi__DOT__u_scsi__DOT__c96_tcounter, 0);

    // Enable the write DRQ-check now that tcounter is genuinely 0.
    dut->scsi_ctrl_in = 0x100;   // bit 8
    CHECK_TRUE("DRQ is low once tcounter is exhausted",
              dut->scsi_drq == 0);

    // Third beat: manually drive AW/W (not the blocking helper) so we
    // can poke c96_tcounter mid-transaction, simulating the effect of a
    // concurrent reg-3 rearm landing while this beat is held off.
    dut->s_awid = 0xC; dut->s_awaddr = SCSI_DMA_SHIM; dut->s_awlen = 0;
    dut->s_awsize = 4; dut->s_awburst = 1; dut->s_awvalid = 1;
    uint32_t wlane = (SCSI_DMA_SHIM >> 2) & 0x3u;
    Word128 wv{0, 0, 0, 0}; wv[wlane] = 0xEE;
    set128(dut->s_wdata, wv);
    dut->s_wstrb = (uint16_t)(1u << (wlane * 4));
    dut->s_wlast = 1; dut->s_wvalid = 1; dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    uint32_t bresp = 99;
    bool poked = false;
    int poke_at_cycle = 30;   // comfortably inside the hold, well under
                              // any timeout — proves the beat is
                              // genuinely gated up to this point.
    for (int i = 0; i < 500 && !b_done; ++i) {
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) bresp = dut->s_bresp;
        if (!poked && i == poke_at_cycle) {
            CHECK_TRUE("still held right before the poke (no premature ack)",
                      !b_done && !b_hs);
            // c96_tcounter alone is not enough: scsi.v's "C96 DMA chunk
            // completion" hook (the c96_xfr_armed<=0 branch guarded by
            // `phase==S_DATA_OUT && c96_tcounter==0`) already fired one
            // cycle after our 2nd real push made tcounter visibly 0,
            // clearing c96_xfr_armed — exactly the real HW behavior a
            // genuine reg-3 rearm would need to undo (it reloads BOTH
            // c96_tcounter and c96_xfr_armed together).  Poke both.
            dut->rootp->tb_pb_scsi__DOT__u_scsi__DOT__c96_tcounter    = 1;
            dut->rootp->tb_pb_scsi__DOT__u_scsi__DOT__c96_xfr_armed   = 1;
            poked = true;
        }
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    for (int i = 0; i < 2; ++i) tick();

    CHECK_TRUE("poke landed before the beat completed", poked);
    CHECK_TRUE("gated write beat completed once tcounter went nonzero",
              b_done);
    CHECK_EQ("gated write resp OKAY", bresp, 0);

    // Bus alive afterwards: an unrelated register read still succeeds.
    dut->scsi_ctrl_in = 0;
    uint32_t seqresp = 99;
    (void)axi_read(SCSI_BASE | (0x6u << 4), &seqresp, 4, 400);
    CHECK_EQ("bus alive: seq_step read resp OKAY after the beat", seqresp, 0);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 4. Bug 2: DRQ-checked DMA-shim READ whose DRQ never rises (BUS_FREE,
// no selection) times out to SLVERR instead of hanging the fabric.
// ─────────────────────────────────────────────────────────────────────
static bool test_drq_never_rises_times_out() {
    reset();
    // BUS_FREE, no selection at all: drq_c96's three OR-terms all
    // require c96_sel_active or phase==S_DATA_IN/S_DATA_OUT, none of
    // which hold here, so DRQ is structurally 0 forever.
    dut->scsi_ctrl_in = 0x080;   // read DRQ-check enabled
    // tb_pb_scsi.v overrides peripheral_bus's PB_WATCHDOG_LOG2 to 12
    // (2^12 = 4096 cycles); budget comfortably past that.
    AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, /*arsize=byte*/0,
                                    /*max_cycles=*/6000);
    CHECK_TRUE("DMA-shim read eventually completes (aborts, doesn't hang)",
              r.completed);
    CHECK_EQ("aborted read resp = SLVERR", r.resp, 2);

    // The bus stays alive: a normal access to a DIFFERENT peripheral
    // (VIA1, via the wrapper's register-file stand-in) still succeeds.
    dut->scsi_ctrl_in = 0;
    uint32_t vresp = 99;
    uint32_t got = axi_read(0x0000000u, &vresp, 4, 400);
    CHECK_EQ("VIA1 read resp OKAY after SCSI timeout", vresp, 0);
    CHECK_EQ("VIA1 reg0 reset pattern (0xA0)", got & 0xFF, 0xA0);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 5. Bug 2 (write side): DRQ-checked DMA-shim WRITE whose DRQ never
// rises times out to SLVERR, bus alive after.
// ─────────────────────────────────────────────────────────────────────
static bool test_drq_write_never_rises_times_out() {
    reset();
    dut->scsi_ctrl_in = 0x100;   // write DRQ-check enabled, BUS_FREE
    AxiWriteResult w = axi_write_full(SCSI_DMA_SHIM, 0x55, /*max_cycles=*/6000);
    CHECK_TRUE("DMA-shim write eventually completes (aborts, doesn't hang)",
              w.completed);
    CHECK_EQ("aborted write resp = SLVERR", w.resp, 2);

    dut->scsi_ctrl_in = 0;
    uint32_t vresp = 99;
    uint32_t got = axi_read(0x0000000u, &vresp, 4, 400);
    CHECK_EQ("VIA1 read resp OKAY after SCSI write timeout", vresp, 0);
    CHECK_EQ("VIA1 reg0 reset pattern (0xA0)", got & 0xFF, 0xA0);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 6. Bug 3: concurrent AW+AR to the SAME peripheral (SCSI), DIFFERENT
// registers — write wins acceptance, read waits, and the read returns
// its OWN register's value once it completes (not the write's).
// ─────────────────────────────────────────────────────────────────────
static bool test_concurrent_write_read_same_peripheral() {
    reset();
    reg_w(0x8, 0xAA);   // config1 — distinguishable preload
    reg_w(0xB, 0xBB);   // config2 — distinguishable preload

    uint32_t waddr = SCSI_BASE | (0x8u << 4);   // config1
    uint32_t raddr = SCSI_BASE | (0xBu << 4);   // config2 (DIFFERENT reg)
    ConcurrentResult r = axi_write_read_concurrent(waddr, 0xCC, raddr, 600);
    CHECK_TRUE("concurrent write completed", r.write_ok);
    CHECK_TRUE("concurrent read completed", r.read_ok);
    CHECK_EQ("write resp OKAY", r.write_resp, 0);
    CHECK_EQ("read resp OKAY", r.read_resp, 0);

    CHECK_EQ("config1 got the write's data", reg_r(0x8), 0xCC);
    // The concurrent read must return config2's value (0xBB) — proof
    // the address mux never showed it the WRITE's address (0xF080)
    // during its pb_rd pulse.
    CHECK_EQ("concurrent read returned config2, not the write's address/data",
             r.read_data & 0xFF, 0xBB);
    return true;
}


// ─────────────────────────────────────────────────────────────────────
// 7. Q700 ROM's real 16-byte DMA-in chunk drain, with LBTM set.
//
// This is the boot-critical case, replayed from a live MAME trace of
// macqd700 running the same Q700 ROM (mame 0.285, save-state item probe
// on :scsi:7:ncr53c96 + a tap on the DAFB TurboSCSI aperture):
//
//   #6    W0c a=50f0f0c0            pc=40899120   -> config3 = 0x04 (LBTM)
//   ...
//   #1612 R04 ...  fifo=16 stat=10 tcnt=0 dir=1 cfg3=04 drq=1
//   #1613 DMAR a=50f4f100 pc=40899304 fifo=14   <- 16-bit pop, 2 bytes
//   #1614 DMAR             pc=40899308 fifo=12
//   #1615 DMAR             pc=4089930c fifo=10
//   #1616 DMAR             pc=40899310 fifo= 8
//   #1617 DMAR             pc=40899314 fifo= 6
//   #1618 DMAR             pc=40899318 fifo= 4
//   #1619 DMAR             pc=4089931c fifo= 2
//   #1620 DMAR             pc=40899320 fifo= 0  drq=0 irq=1 (I_BUS)
//
// Eight `move.w` beats, each popping TWO bytes ATOMICALLY through
// ncr53c94_device::dma16_r() (ncr53c90.cpp:1325-1350).  MAME's DAFB
// shim checks DRQ exactly ONCE per host access, BEFORE the pop
// (dafb.cpp:1000-1010) — never between the two bytes.  That matters
// because with LBTM (config3 bit 2, conf3_mask::LBTM) the BUSMD_1
// DMA_IN DRQ formula is `fifo_pos > 1` (ncr53c90.cpp:1387), so the
// odd intermediate occupancy a byte-granular drain would produce has
// DRQ LOW.  peripheral_bus.v splits a word DMA-shim read into two
// byte-granular pb beats (rd_scsi_phase_q); if the DRQ hold-off is
// applied to BOTH halves, the drain wedges partway down the chunk and
// the CPU takes a bus error at the aperture — the hardware-confirmed
// Sad Mac 0F02 at ROM PC 0x4089931c, fault address 0x50f4f100.
// ─────────────────────────────────────────────────────────────────────
static bool test_lbtm_word_chunk_drain_matches_mame() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.gap = 0;

    // The Q700 ROM writes config3 = 0x04 (LBTM) at PC 0x40899120.
    reg_w(0xC, 0x04);
    reg_w(0x4, 0x00);   // bus_id = TARGET_ID (0)
    reg_w(0x3, 0x01);   // FLUSH_FIFO
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);   // DMA | CD_SELECT, empty FIFO

    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", seq_ok);

    // CDB: READ(6) opcode 0x08, lba=0, blocks=1.
    reg_w(0x2, 0x08);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x01);
    axi_write(SCSI_DMA_SHIM, 0x00);

    uint8_t v = 0;
    bool select_ok = false;
    for (int i = 0; i < 20000; ++i) {
        v = reg_r(0x4);
        if (v & 0x80) { select_ok = true; break; }
    }
    CHECK_TRUE("select-complete interrupt fires", select_ok);
    CHECK_EQ("select-complete phase = DATA_IN", v & 0x07, 0x01);
    CHECK_EQ("select-complete istatus = I_FUNCTION|I_BUS", reg_r(0x5), 0x18);

    // Arm a 16-byte chunk, exactly like the ROM (tcount=16, DMA|CI_XFER).
    reg_w(0x0, 0x10);
    reg_w(0x1, 0x00);
    reg_w(0x3, 0x90);

    dut->scsi_ctrl_in = 0x080;   // DAFB read DRQ-check on

    // Let the chip stage the whole chunk, like MAME's trace record
    // #1612 (fifo=16, TC0 set) before the first DMAR.
    for (int i = 0; i < 4000 && (reg_r(0x7) & 0x1F) != 0x10; ++i) {}
    CHECK_EQ("chip staged the whole 16-byte chunk", reg_r(0x7) & 0x1F, 0x10);
    CHECK_TRUE("TC0 set once the chunk is fully received", (reg_r(0x4) & 0x10) != 0);
    CHECK_TRUE("DRQ high with a full FIFO", dut->scsi_drq == 1);

    // Eight word beats, MAME records #1613..#1620.
    for (int beat = 0; beat < 8; ++beat) {
        AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, /*arsize=word*/1,
                                        /*max_cycles=*/9000);
        char nm[96];
        std::snprintf(nm, sizeof nm,
                      "word beat %d completed (MAME record #%d, ROM PC 0x%08x)",
                      beat, 1613 + beat, 0x40899304u + 4u * (unsigned)beat);
        CHECK_TRUE(nm, r.completed);
        std::snprintf(nm, sizeof nm, "word beat %d resp OKAY", beat);
        CHECK_EQ(nm, r.resp, 0);
        uint16_t got = (uint16_t)(r.data & 0xFFFF);
        uint16_t exp = (uint16_t)((sd_mock.read_sector[2 * beat] << 8) |
                                   sd_mock.read_sector[2 * beat + 1]);
        std::snprintf(nm, sizeof nm, "word beat %d payload", beat);
        CHECK_EQ(nm, got, exp);
    }

    // MAME record #1620: the FIFO is empty and the chunk-complete
    // interrupt (I_BUS) has fired.
    CHECK_EQ("FIFO fully drained after 8 word beats", reg_r(0x7) & 0x1F, 0x00);
    CHECK_TRUE("DRQ low once the chunk is drained", dut->scsi_drq == 0);

    dut->scsi_ctrl_in = 0;
    uint32_t seqresp = 99;
    (void)axi_read(SCSI_BASE | (0x6u << 4), &seqresp, 4, 400);
    CHECK_EQ("bus alive after the chunk drain", seqresp, 0);
    return true;
}


// ─────────────────────────────────────────────────────────────────────
// 8. BOARD WEDGE REPRO (hardware-confirmed 2026-08-19, bitstream
// f63207b+8bd3ded).  After the Sad Mac 0F02 fix the machine runs far
// past 0x4089931c and then spins forever here:
//
//   ROM 0x40899270  moveb #16,%a3@(48)      ; reg3 = 0x10 non-DMA XFER
//   ROM 0x40899276  jsr    %pc@(0x40899704) ; wait-for-INT, no timeout
//   ROM 0x4089927a  rts                     ; <- (a7) read off the board
//   ROM 0x40899704  moveb %a3@(64),%d5 / btst #7,%d5 / beqs  <- PC lives here
//
// Halted board state: PC=0x4089970a D5=0x13 A3=0x50f0f000 SR=0x2000
//   reg3 cmd echo = 0x10   reg4 = 0x13 (STATUS phase, TC0, INT CLEAR)
//   reg7 flags    = 0x01   reg0/reg1 tcounter = 0x0000
// GROSS_ERROR (status 0x40) is CLEAR, so the 0x10 was ACCEPTED, not
// dropped by MAME's command_pos==2 rule (ncr53c90.cpp command_w).
//
// The isolated transition is NOT the bug: a directed differential
// (config3=0x04, one 512-byte DMA chunk, residual byte present, then
// 0x10) is byte-identical RTL vs MAME and DOES raise the interrupt:
//   SYNC fifo=1:fa irq=1 stat=93 flags=01 istat=10   (before 0x10)
//   SYNC fifo=2:fa00 irq=1 stat=97 flags=02 istat=10 (after  0x10)
// So the wedge needs the ROM's REAL flow: 32 bare repeats of 0x90, each
// drained by 8 blind 16-bit reads.  The scsi_fuzz harness cannot express
// that (byte-only aperture ops, DRQ-paced, no peripheral_bus — see
// docs/scsi_fuzz.md "Known blind spots"), which is exactly why this
// lives here instead.
// ─────────────────────────────────────────────────────────────────────
static bool test_rom_chunked_block_then_nondma_xfer_completes() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.gap = 0;

    reg_w(0xC, 0x04);   // config3 = LBTM, as the Q700 ROM writes at 0x40899120
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", seq_ok);
    reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x01);
    axi_write(SCSI_DMA_SHIM, 0x00);

    uint8_t v = 0; bool select_ok = false;
    for (int i = 0; i < 20000; ++i) { v = reg_r(0x4); if (v & 0x80) { select_ok = true; break; } }
    CHECK_TRUE("select-complete interrupt fires", select_ok);
    CHECK_EQ("select-complete phase = DATA_IN", v & 0x07, 0x01);
    CHECK_EQ("select-complete istatus", reg_r(0x5), 0x18);

    dut->scsi_ctrl_in = 0x080;          // DAFB read DRQ-check on

    // TC latch written ONCE, then 32 BARE repeats of 0x90 (the ROM's loop).
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x10);
    for (int chunk = 0; chunk < 32; ++chunk) {
        reg_w(0x3, 0x90);
        bool ready = false;
        for (int i = 0; i < 20000; ++i) {
            if ((reg_r(0x7) & 0x1f) == 0x10) { ready = true; break; }
        }
        char nm[96];
        std::snprintf(nm, sizeof nm, "chunk %d staged 16 bytes", chunk);
        CHECK_TRUE(nm, ready);
        // 8 blind 16-bit beats — the ROM's unrolled move.w loop.
        for (int beat = 0; beat < 8; ++beat) {
            AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, /*word*/1, 9000);
            std::snprintf(nm, sizeof nm, "chunk %d beat %d completed", chunk, beat);
            CHECK_TRUE(nm, r.completed);
            std::snprintf(nm, sizeof nm, "chunk %d beat %d resp OKAY", chunk, beat);
            CHECK_EQ(nm, r.resp, 0);
        }
        bool intr = false;
        for (int i = 0; i < 20000; ++i) { v = reg_r(0x4); if (v & 0x80) { intr = true; break; } }
        std::snprintf(nm, sizeof nm, "chunk %d post-drain INT", chunk);
        CHECK_TRUE(nm, intr);
        std::snprintf(nm, sizeof nm, "chunk %d istatus = I_BUS", chunk);
        CHECK_EQ(nm, reg_r(0x5), 0x10);
    }

    // ── the wedge point: ROM 0x40899270 writes 0x10 and waits ──
    std::printf("  [pre-0x10] reg4=0x%02x reg7=0x%02x reg0=0x%02x reg1=0x%02x\n",
                reg_r(0x4), reg_r(0x7) & 0x1f, reg_r(0x0), reg_r(0x1));
    reg_w(0x3, 0x10);
    bool intr = false;
    for (int i = 0; i < 50000; ++i) { v = reg_r(0x4); if (v & 0x80) { intr = true; break; } }
    if (!intr) {
        std::printf("  [WEDGE REPRODUCED] non-DMA 0x10 never raised INT: "
                    "reg4=0x%02x reg7=0x%02x reg0=0x%02x reg1=0x%02x reg3=0x%02x\n",
                    v, reg_r(0x7) & 0x1f, reg_r(0x0), reg_r(0x1), reg_r(0x3));
    }
    CHECK_TRUE("non-DMA Transfer Information (0x10) raises INT "
               "(board wedge: it never does)", intr);
    return true;
}


// ─────────────────────────────────────────────────────────────────────
// 9. SD-LATENCY INJECTION — provider-wait window (2026-08-19).
//
// The board wedge (bitstream f63207b+8bd3ded) is: ROM 0x40899010 pushes
// 0xEE into reg 2, ROM 0x40899270 writes reg3 = 0x10 (non-DMA Transfer
// Information) and jsr's the ROM's UNTIMED wait-for-INT at 0x40899704.
// Board halted: reg3=0x10, reg4=0x13 (STATUS+TC0, INT clear), reg7=0x01,
// GROSS_ERROR clear, sd_busy=0, static over 16 s.
//
// Sim never saw it because SdMock answered a kick in ~2 cycles, so
// S_VH_WAIT_RD was a blink, while the board runs a real SPI card
// (~164 us per 512-byte block == ~16,400 core cycles at 100 MHz).
// SdMock::go_latency (default 0, so no existing scenario changes) makes
// that window arbitrarily wide, and this scenario lands the ROM's exact
// two writes inside it.
//
// MEASURED RESULT: this does NOT reproduce the board.  What actually
// happens is that the select in command slot 0 has not retired, so
// command_pos == 1 and the 0x10 is QUEUED into slot 1 — scsi.v:2283-2285
// updates c96_cmd_q1 and leaves the reg-3 echo showing 0xC1.  Verified
// by cycle-accurate observation: the AXI write completes OKAY, reg3
// reads back 0xC1, and c96_xfr_armed is never asserted at all.  That is
// MAME-faithful (ncr53c90.cpp command_w starts a command only from slot
// 0) and it is NOT the board's state: the board's reg3 reads 0x10, which
// means its queue was EMPTY and start_command() did run.
//
// So the board's precondition is still unreproduced, and the committed
// dbg_c96_state probe (probe_in26) remains the way to settle it.  This
// scenario is kept because the property it pins is worth a regression
// test either way: a command written during the provider-wait must be
// QUEUED, not LOST, and an interrupt must still arrive afterwards.
// ─────────────────────────────────────────────────────────────────────
#define U_SCSI(sig) (dut->rootp->tb_pb_scsi__DOT__u_scsi__DOT__##sig)

static bool test_cmd_write_during_provider_wait_is_queued_not_lost() {
    reset();
    sd_mock.read_sector.resize(2 * 512);
    for (int i = 0; i < 2 * 512; ++i) sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.go_latency = 20000;         // ~ a real SPI block time (see SdMock)

    reg_w(0xC, 0x04);
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after select", seq_ok);
    reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x02);         // READ(6) 2 blocks
    axi_write(SCSI_DMA_SHIM, 0x00);

    // Park in S_VH_WAIT_RD(6) — the provider-wait window the go_latency
    // knob widens from ~2 cycles to a realistic SPI block time.
    bool in_wait = false;
    for (int i = 0; i < 600000; ++i) {
        if (U_SCSI(phase) == 6 && U_SCSI(c96_xfer_active)) { in_wait = true; break; }
        tick();
    }
    CHECK_TRUE("FSM parks in S_VH_WAIT_RD with a live connection", in_wait);

    // ROM 0x40899010 / 0x40899270: push a byte, then non-DMA 0x10.
    reg_w(0x2, 0xEE);
    AxiWriteResult wr = axi_write_full(SCSI_BASE | (0x3u << 4), 0x10, 4000);
    CHECK_TRUE("reg3 write completes", wr.completed);
    CHECK_EQ("reg3 write resp OKAY", wr.resp, 0);

    // MEASURED here, and MAME-faithful (ncr53c90.cpp command_w): the
    // select in slot 0 has not retired, so command_pos == 1 and the 0x10
    // is QUEUED into slot 1 — scsi.v:2283-2285 updates c96_cmd_q1 and
    // deliberately leaves the reg-3 echo alone.  start_command() does not
    // run, so nothing arms.  The queued command must not be LOST: the
    // istatus read that retires slot 0 dispatches it.
    CHECK_EQ("reg3 echo still shows the unretired select (queued, not started)",
             reg_r(0x3), 0xC1);

    // Let the provider land, then drain the interrupts the way a driver
    // does; the queued 0x10 must eventually be dispatched and complete.
    sd_mock.go_latency = 0;
    bool intr = false; uint8_t st = 0;
    for (int i = 0; i < 400000; ++i) {
        st = reg_r(0x4);
        if (st & 0x80) { intr = true; break; }
    }
    CHECK_TRUE("an interrupt still arrives after the provider-wait", intr);
    CHECK_TRUE("bus alive afterwards", (reg_r(0x6) & 0xF8) == 0);
    return true;
}


// ─────────────────────────────────────────────────────────────────────
// 10. PHASE-DRIFT COMPLETION COVERAGE — the 2026-08-19 board hang.
//
// A non-DMA CI_XFER (0x10) left armed with the bus settled in STATUS
// must ALWAYS reach a completion, whatever phase was latched into
// c96_xfr_phase at dispatch.  MAME's arm is general (ncr53c90.cpp:
// 653-657, `(ctrl & S_PHASE_MASK) != xfr_phase`); ours enumerated
// {S_DATA_IN, S_DATA_OUT}, so 9 of the 12 latched values had NO hook at
// all and the command stayed armed forever.
//
// RED before the scsi.v fix (measured): latched 0,1,2,3,6,7,9,10,11 all
// wedge, each landing on exactly the hardware signature — live phase
// STATUS, one staged byte, istatus 0x00, no IRQ — which is the board's
// reg4=0x13 / reg7=0x01 / INT-clear state, with the ROM parked in its
// untimed wait-for-INT at 0x40899704.  GREEN after: all 12 complete.
//
// c96_xfr_phase latches the INTERNAL FSM state (`c96_xfr_phase <=
// phase`), and that enum contains S_CMD_EXEC(3) / S_VH_WAIT_RD(6) /
// S_VH_WAIT_WR(7) — backing-store wait states MAME's xfr_phase can
// never hold because it is masked from the bus lines.  Those are
// exactly the values a real SD card's latency makes reachable and a
// 2-cycle mock does not, which is why eight RTL-vs-MAME differentials
// all came back byte-identical while the board hung.
//
// The latch is fault-injected rather than raced: the wedge depends on
// WHICH value is latched, not on how it got there, and injection makes
// the coverage total and deterministic instead of timing-dependent.
// ─────────────────────────────────────────────────────────────────────
static bool test_nondma_xfer_completes_from_any_latched_phase() {
    static const char* PH[12] = {"BUS_FREE","SELECT","COMMAND","CMD_EXEC",
        "DATA_IN","DATA_OUT","VH_WAIT_RD","VH_WAIT_WR","STATUS","MSG_IN",
        "DISCONNECT","RESET"};
    std::printf("  latched_phase  ->  completes?\n");
    for (unsigned L = 0; L < 12; ++L) {
        reset();
        sd_mock.read_sector.resize(512);
        for (int i = 0; i < 512; ++i) sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
        reg_w(0xC, 0x04); reg_w(0x4, 0x00); reg_w(0x3, 0x01);
        reg_w(0x1, 0x00); reg_w(0x0, 0x01); reg_w(0x3, 0xC1);
        for (int i = 0; i < 64; ++i) if (reg_r(0x6) & 7) break;
        reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
        reg_w(0x2, 0x00); reg_w(0x2, 0x01);
        axi_write(SCSI_DMA_SHIM, 0x00);
        for (int i = 0; i < 20000; ++i) if (reg_r(0x4) & 0x80) break;
        (void)reg_r(0x5);
        // Drain the whole block via DMA so the target moves to STATUS.
        dut->scsi_ctrl_in = 0x080;
        reg_w(0x1, 0x00); reg_w(0x0, 0x10);
        for (int c = 0; c < 32; ++c) {
            reg_w(0x3, 0x90);
            for (int i = 0; i < 20000; ++i) if ((reg_r(0x7) & 0x1f) == 0x10) break;
            for (int b = 0; b < 8; ++b) (void)axi_read_full(SCSI_DMA_SHIM, 1, 9000);
            for (int i = 0; i < 20000; ++i) if (reg_r(0x4) & 0x80) break;
            (void)reg_r(0x5);
        }
        dut->scsi_ctrl_in = 0;
        reg_w(0x2, 0xEE);                       // the ROM's staged byte
        // Fault-inject: a non-DMA CI_XFER armed with latched phase = L.
        U_SCSI(c96_xfr_armed)  = 1;
        U_SCSI(c96_xfr_dma)    = 0;
        U_SCSI(c96_xfr_phase)  = L;
        U_SCSI(c96_istatus)    = 0;
        U_SCSI(c96_irq_pending)= 0;
        bool done = false;
        for (int i = 0; i < 60000 && !done; ++i) {
            if (U_SCSI(c96_irq_pending) || !U_SCSI(c96_xfr_armed)) done = true;
            tick();
        }
        std::printf("    %2u %-11s -> %s   (live phase=%u fifo=%u istat=0x%02x)\n",
                    L, PH[L], done ? "completes" : "** WEDGES **",
                    (unsigned)U_SCSI(phase), (unsigned)U_SCSI(c96_fifo_pos),
                    (unsigned)U_SCSI(c96_istatus));
        if (!done) {
            char nm[96];
            std::snprintf(nm, sizeof nm,
                          "armed non-DMA 0x10 completes with latched phase %u (%s)",
                          L, PH[L]);
            CHECK_TRUE(nm, done);
        }
    }
    return true;
}

// ═════════════════════════════════════════════════════════════════════
// 16-BIT PSEUDO-DMA APERTURE — the ROM's actual access width.
//
// Coverage-audit additions (2026-08-19).  Before these, the ONLY word-
// width scenario in the whole repo was test_lbtm_word_chunk_drain_
// matches_mame (READ, full FIFO, LBTM).  Everything below it — the
// write half of the aperture, and both halves' UNDERFLOW/DEGRADE
// corners — was reachable by the ROM and covered by nothing.
//
// Golden reference, MAME 0.285:
//   dafb.cpp:993-1078  turboscsi_dma_r/w — the DRQ check happens ONCE
//        per host access (BIT(m_scsi_ctrl,7) for reads, bit 8 for
//        writes) and then the whole 16-bit access is handed to the chip
//        in one call.  A word access is NEVER split into two
//        independently DRQ-gated halves.
//   ncr53c90.h:312-313 dma16_swap_r/w = swapendian_int16 around
//        dma16_r/dma16_w (the 68k is big-endian; the chip FIFO is
//        byte-ordered lowest-address-first).
//   ncr53c90.cpp:1325-1350 dma16_r:
//        if (fifo_pos < 2) return dma_r() | 0xff00;   // <- UNDERFLOW
//        else pop two bytes.
//        After the swap the host therefore sees {residual, 0xFF} — one
//        real byte in the HIGH half and a literal 0xFF filler in the
//        low half.
//   ncr53c90.cpp:1352-1368 dma16_w:
//        if (fifo_pos > 14 || tcounter == 1) { dma_w(data & 0x00ff);
//                                              return; }             // <- DEGRADE
//        else push two bytes.
//        With the swap, `data & 0x00ff` is the FIRST memory byte of the
//        CPU's word; the second byte is DISCARDED.  This is precisely
//        the guard that makes the write side atomic: the chip never
//        needs a second DRQ grant part-way through a word.
// ═════════════════════════════════════════════════════════════════════

// Word (16-bit) AXI write to a big-endian byte-lane fabric.  `b0` is the
// byte at the lower address (the m68k `move.w`'s high byte), `b1` the
// byte at addr+1.  peripheral_bus's wr_addr_byte mapping (addr[1:0]==0
// -> wr_lane_data[31:24]) fixes the lane/strobe placement below; the
// serializer walks the strobe HIGH bit first, so b0 is sent first.
static AxiWriteResult axi_write_word_full(uint32_t addr, uint8_t b0, uint8_t b1,
                                          int max_cycles) {
    dut->s_awid    = 0xC;
    dut->s_awaddr  = addr;
    dut->s_awlen   = 0;
    dut->s_awsize  = 1;          // 2 bytes
    dut->s_awburst = 1;
    dut->s_awvalid = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    Word128 w{0, 0, 0, 0};
    w[lane] = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16);
    set128(dut->s_wdata, w);
    dut->s_wstrb  = (uint16_t)(0x3u << (lane * 4 + 2));   // bits [3:2] of the lane
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    uint32_t resp = 0;
    for (int i = 0; i < max_cycles && !b_done; i++) {
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) resp = dut->s_bresp;
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    for (int i = 0; i < 2; i++) tick();
    return {resp, b_done};
}

// Shared preamble: WRITE(6) select -> S_DATA_OUT, chip connected, ready
// for a DMA CI_XFER.  Returns false if the select never completed.
static bool enter_data_out_phase(int blocks) {
    reset();
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);   // FLUSH_FIFO
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);   // select's own DMA counter covers the CDB tail byte
    reg_w(0x3, 0xC1);   // DMA | CD_SELECT, empty FIFO
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    if (!seq_ok) return false;
    reg_w(0x2, 0x0A);   // WRITE(6)
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, 0x00);
    reg_w(0x2, (uint8_t)blocks);
    axi_write(SCSI_DMA_SHIM, 0x00);   // control byte through the DMA port
    for (int i = 0; i < 20000; ++i) {
        uint8_t v = reg_r(0x4);
        if (v & 0x80) return (v & 0x07) == 0x00;   // DATA_OUT
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────
// 11. WRITE-SIDE ATOMICITY OF THE 16-BIT APERTURE.
//
// The exact mirror of the Sad Mac 0F 02 that f63207b fixed on the READ
// side, and it is NOT fixed on the write side: peripheral_bus.v:1719-21
// says in as many words "Writes never take the split path (the write
// side has its own per-strobe-bit serializer), so this is read-only by
// construction" — scsi_dma16_lo_beat is driven only from rd_scsi_dma16_lo,
// and scsi.v's c96_shim_wr_withhold (scsi.v:1599) has no
// c96_dma16_lo_inherit term at all.  So a word write becomes two
// independently DRQ-gated byte pulses (wr_scsi_word_active,
// peripheral_bus.v:1206-1210, each gated on scsi_dma_wr_ready).
//
// The reachable divergence is tcounter == 1:
//   MAME  — one DRQ check (DRQ is high: DMA_OUT drq = !TC0 &&
//           fifo_pos < 15, and tcounter==1 means TC0 is still clear),
//           then dma16_w's `tcounter == 1` guard pushes exactly ONE
//           byte (the first memory byte) and returns.  The access
//           completes; the second byte is discarded.
//   OURS  — beat 0 is accepted and decrements c96_tcounter to 0, which
//           drops drq_c96's DATA_OUT term (scsi.v:1476-1477
//           `c96_tcounter != 0`); beat 1 then re-runs the DRQ check
//           with the check enabled and no inherit -> withheld.
//
// This is a genuine driver path: any pseudo-DMA write whose remaining
// count is odd ends on a tcounter==1 word beat, and the Q700 ROM's own
// DMA-form select feeds its CDB tail through this same aperture.
// ─────────────────────────────────────────────────────────────────────
static bool test_word_pdma_write_is_atomic_like_dma16_w() {
    // ── POSITIVE CONTROL ──────────────────────────────────────────────
    // Prove the word-write helper itself reaches scsi.v before asserting
    // anything about the DRQ interaction: with the check DISABLED the
    // same access must complete and move TWO bytes.  Without this, a
    // malformed strobe/lane would make the scenario below fail for the
    // wrong reason — the exact trap docs/scsi_fuzz.md's blind-spot #2
    // describes (scsi_ctrl_in == 0 makes every DRQ path dead code).
    CHECK_TRUE("WRITE(6) select reached DATA OUT (control)",
               enter_data_out_phase(1));
    (void)reg_r(0x5);
    reg_w(0x0, 0x04);
    reg_w(0x1, 0x00);
    reg_w(0x3, 0x90);
    dut->scsi_ctrl_in = 0;      // blind aperture — no DRQ gating at all
    {
        uint32_t tc_before = U_SCSI(c96_tcounter);
        AxiWriteResult c = axi_write_word_full(SCSI_DMA_SHIM, 0x12, 0x34, 4000);
        CHECK_TRUE("control: blind word write completes", c.completed);
        CHECK_EQ("control: blind word write resp OKAY", c.resp, 0);
        CHECK_EQ("control: blind word write moved TWO bytes "
                 "(the helper really reaches scsi.v)",
                 tc_before - (uint32_t)U_SCSI(c96_tcounter), 2u);
    }

    // ── the real scenario ─────────────────────────────────────────────
    CHECK_TRUE("WRITE(6) select reached DATA OUT", enter_data_out_phase(1));
    (void)reg_r(0x5);

    // Arm a one-byte DMA chunk: tcount = 1.
    reg_w(0x0, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x3, 0x90);           // DMA | CI_XFER
    for (int i = 0; i < 2000 && !dut->scsi_drq; ++i) tick();
    CHECK_TRUE("DRQ high with tcount=1 armed in DATA OUT (MAME: !TC0 && "
               "fifo_pos<15)", dut->scsi_drq == 1);

    dut->scsi_ctrl_in = 0x100;  // DAFB write DRQ-check on (scsi_ctrl bit 8)

    // ONE host access, exactly as `move.w d0,(a1)` produces.
    AxiWriteResult r = axi_write_word_full(SCSI_DMA_SHIM, 0xA5, 0x5A, 4000);
    CHECK_TRUE("word PDMA write completes (MAME: one DRQ check, then "
               "dma16_w's tcounter==1 guard pushes one byte and returns)",
               r.completed);
    CHECK_EQ("word PDMA write resp OKAY", r.resp, 0);

    // MAME transfers exactly ONE byte here — `dma_w(data & 0x00ff)` on
    // the byte-swapped word is the FIRST memory byte (0xA5).  tcounter
    // therefore lands on 0 (TC0), not on an underflowed 0x1FFFF.
    CHECK_EQ("tcounter is exactly exhausted, not underflowed",
             U_SCSI(c96_tcounter), 0);

    dut->scsi_ctrl_in = 0;
    uint32_t seqresp = 99;
    (void)axi_read(SCSI_BASE | (0x6u << 4), &seqresp, 4, 400);
    CHECK_EQ("bus alive after the word write", seqresp, 0);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 12. READ-SIDE UNDERFLOW: the RESIDUAL byte.
//
// ncr53c90.cpp:1325-1328 — dma16_r with fewer than two bytes staged
// returns `dma_r() | 0xff00`, i.e. ONE popped byte plus a literal 0xFF
// filler; after dma16_swap_r the host word is {residual, 0xFF}.
//
// This is reachable with the DRQ check ENABLED and no LBTM: the BUSMD_1
// DMA_IN threshold (ncr53c90.cpp:1385-1389) is
//     fifo_pos > ((config3.2 || !TC0) ? 1 : 0)
// so with config3 = 0 and TC0 SET the threshold is 0 — a single staged
// byte asserts DRQ ("save last remaining byte for the processor" only
// applies while LBTM is set or the count is unfinished).  A driver
// draining an odd-length chunk with `move.w` lands here on its last
// beat.
//
// Our split path issues two independent dma_r() pops.  Under the F15
// memmove-exact FIFO the second pop of an empty FIFO REPEATS the last
// popped byte, so the host word would be {residual, residual} — the
// low half carries a duplicate of real payload where MAME puts a
// recognisable 0xFF filler.  A driver that trusts the low half writes
// one byte of garbage into the transfer buffer.
//
// NEGATIVE CONTROL: the first sub-assertion pins fifo_pos == 1 with DRQ
// HIGH before the read, so this scenario cannot pass by never reaching
// the underflow path.
// ─────────────────────────────────────────────────────────────────────
static bool test_word_pdma_read_underflow_pads_low_half_with_ff() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.gap = 0;

    reg_w(0xC, 0x00);   // config3 = 0 — NO LBTM, so the DRQ threshold is 0
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", seq_ok);
    reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x01);
    axi_write(SCSI_DMA_SHIM, 0x00);
    uint8_t v = 0; bool select_ok = false;
    for (int i = 0; i < 20000; ++i) { v = reg_r(0x4); if (v & 0x80) { select_ok = true; break; } }
    CHECK_TRUE("select-complete interrupt fires", select_ok);
    CHECK_EQ("select-complete phase = DATA_IN", v & 0x07, 0x01);
    (void)reg_r(0x5);

    // Arm a ONE-byte chunk so the FIFO settles at exactly one staged
    // byte with TC0 set.
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0x90);
    bool staged = false;
    for (int i = 0; i < 20000; ++i) {
        if ((reg_r(0x7) & 0x1f) == 0x01 && (reg_r(0x4) & 0x10)) { staged = true; break; }
    }
    CHECK_TRUE("chip staged exactly one byte with TC0 set", staged);

    dut->scsi_ctrl_in = 0x080;  // DAFB read DRQ-check on
    for (int i = 0; i < 200; ++i) tick();
    // Negative control: without this the scenario could "pass" by never
    // reaching the underflow branch at all.
    CHECK_EQ("precondition: fifo_pos == 1", U_SCSI(c96_fifo_pos), 1);
    CHECK_TRUE("precondition: DRQ high at fifo_pos==1 with config3.2 clear "
               "and TC0 set (MAME threshold 0)", dut->scsi_drq == 1);

    AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, /*arsize=word*/1, 4000);
    CHECK_TRUE("underflowing word beat completes", r.completed);
    CHECK_EQ("underflowing word beat resp OKAY", r.resp, 0);
    uint16_t got = (uint16_t)(r.data & 0xFFFF);
    CHECK_EQ("residual byte lands in the HIGH half", (got >> 8) & 0xFF,
             sd_mock.read_sector[0]);
    CHECK_EQ("low half is MAME's 0xFF filler, not a duplicated payload byte "
             "(ncr53c90.cpp:1327 `dma_r() | 0xff00`)", got & 0xFF, 0xFF);
    CHECK_EQ("exactly one byte was popped", U_SCSI(c96_fifo_pos), 0);

    dut->scsi_ctrl_in = 0;
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 13. READ-SIDE UNDERFLOW ON A COMPLETELY EMPTY FIFO (blind aperture).
//
// Same MAME branch, fifo_pos == 0.  `dma_r()` on an empty FIFO is the
// F15 memmove-repeat (docs/scsi_fuzz.md) — MAME's fifo_pop leaves the
// stale slot contents in place, so the byte returned is the LAST byte
// popped.  The 0xff00 OR is unconditional, so the low half is still
// 0xFF.  Driven blind (scsi_ctrl_in == 0, the DAFB reset default) since
// DRQ is structurally low with an empty FIFO.
//
// This pins the pairing of the two rules — F15's repeat AND the 0xFF
// filler — which is what a driver polling past the end of a chunk sees.
// ─────────────────────────────────────────────────────────────────────
static bool test_word_pdma_read_empty_fifo_repeats_last_byte_with_ff() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.gap = 0;

    reg_w(0xC, 0x00);
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    for (int i = 0; i < 64; ++i) if (reg_r(0x6) & 7) break;
    reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x01);
    axi_write(SCSI_DMA_SHIM, 0x00);
    bool select_ok = false;
    for (int i = 0; i < 20000; ++i) if (reg_r(0x4) & 0x80) { select_ok = true; break; }
    CHECK_TRUE("select-complete interrupt fires", select_ok);
    (void)reg_r(0x5);

    // Two-byte chunk, drained by one word beat -> FIFO empty, and the
    // last byte popped is sd_mock.read_sector[1].
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x02);
    reg_w(0x3, 0x90);
    bool staged = false;
    for (int i = 0; i < 20000; ++i) {
        if ((reg_r(0x7) & 0x1f) == 0x02) { staged = true; break; }
    }
    CHECK_TRUE("chip staged the 2-byte chunk", staged);
    AxiReadResult first = axi_read_full(SCSI_DMA_SHIM, 1, 4000);
    CHECK_TRUE("first word beat completes", first.completed);
    CHECK_EQ("first word beat payload",
             (uint16_t)(first.data & 0xFFFF),
             (uint16_t)((sd_mock.read_sector[0] << 8) | sd_mock.read_sector[1]));
    CHECK_EQ("precondition: FIFO is now empty", U_SCSI(c96_fifo_pos), 0);

    // Blind word read of the drained aperture.
    AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, 1, 4000);
    CHECK_TRUE("blind word beat on an empty FIFO completes", r.completed);
    uint16_t got = (uint16_t)(r.data & 0xFFFF);
    CHECK_EQ("high half repeats the last popped byte (F15 memmove FIFO)",
             (got >> 8) & 0xFF, sd_mock.read_sector[1]);
    CHECK_EQ("low half is the 0xFF filler, not a second repeat "
             "(ncr53c90.cpp:1327)", got & 0xFF, 0xFF);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// 14. STANDALONE BYTE ACCESS TO 0x101 MUST RUN ITS OWN DRQ CHECK.
//
// Regression gate for the latent hole 8bd3ded closed.  f63207b keyed the
// split-word DRQ inherit on ADDRESS PARITY (`pb_addr[0] &&
// c96_dma16_hi_granted`), which cannot tell the low half of a word
// access from a genuine standalone byte access to the aperture's odd
// byte.  A byte-granular drain walking 0x100,0x101,0x100,... would then
// have skipped the DRQ check on every second byte.
//
// That access pattern is not hypothetical: tb_scsi_c96_sm43_chunk.cpp
// walks exactly it (`shim_r_at((i & 1) ? 0x101 : 0x100)`), and it stayed
// green through the parity-inherit era only because it never sets
// scsi_ctrl_in's DRQ-check bit — one config bit away from the real
// machine.  Nothing in the repo drove that pattern through the REAL
// peripheral_bus with the check ON, so nothing could see the hole.
//
// MAME reference: dafb.cpp:1000-1010 runs the `BIT(m_scsi_ctrl,7)` DRQ
// test on EVERY host access, and an 8-bit access (mem_mask != 0xffff)
// goes to plain dma_r() — there is no cross-access grant to inherit.
// Under LBTM (config3 bit 2, which the Q700 ROM sets) the DMA_IN
// threshold is `fifo_pos > 1`, so the byte sitting at fifo_pos == 1 is
// reserved for the processor and the aperture must HOLD OFF — which the
// fabric surfaces as the watchdog SLVERR, exactly as
// test_drq_never_rises_times_out pins for the structural case.
// ─────────────────────────────────────────────────────────────────────
static bool test_byte_drain_at_odd_aperture_byte_rechecks_drq() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.gap = 0;

    reg_w(0xC, 0x04);   // config3 = LBTM, as the Q700 ROM writes
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", seq_ok);
    reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x01);
    axi_write(SCSI_DMA_SHIM, 0x00);
    uint8_t v = 0; bool select_ok = false;
    for (int i = 0; i < 20000; ++i) { v = reg_r(0x4); if (v & 0x80) { select_ok = true; break; } }
    CHECK_TRUE("select-complete interrupt fires", select_ok);
    CHECK_EQ("select-complete phase = DATA_IN", v & 0x07, 0x01);
    (void)reg_r(0x5);

    reg_w(0x1, 0x00);
    reg_w(0x0, 0x10);   // tcount = 16
    reg_w(0x3, 0x90);
    for (int i = 0; i < 20000 && (reg_r(0x7) & 0x1F) != 0x10; ++i) {}
    CHECK_EQ("chip staged the whole 16-byte chunk", reg_r(0x7) & 0x1F, 0x10);

    dut->scsi_ctrl_in = 0x080;   // read DRQ-check ON — the real machine

    // Byte-granular drain, alternating even/odd aperture bytes.  Under
    // LBTM this must stop with exactly ONE byte left: DRQ is
    // `fifo_pos > 1`, so beats 0..14 are granted and beat 15 is held off.
    for (int i = 0; i < 15; ++i) {
        uint32_t addr = SCSI_DMA_SHIM | ((i & 1) ? 1u : 0u);
        AxiReadResult r = axi_read_full(addr, /*arsize=byte*/0, 9000);
        char nm[96];
        std::snprintf(nm, sizeof nm, "byte beat %d (0x%03x) completed", i,
                      (unsigned)(addr & 0x1FF));
        CHECK_TRUE(nm, r.completed);
        std::snprintf(nm, sizeof nm, "byte beat %d resp OKAY", i);
        CHECK_EQ(nm, r.resp, 0);
        std::snprintf(nm, sizeof nm, "byte beat %d payload", i);
        CHECK_EQ(nm, r.data & 0xFF, sd_mock.read_sector[i]);
    }
    CHECK_EQ("LBTM reserved exactly one byte for the processor",
             U_SCSI(c96_fifo_pos), 1);
    CHECK_TRUE("DRQ is low with one byte left under LBTM",
               dut->scsi_drq == 0);

    // Beat 15 lands on 0x101 (odd).  It is a STANDALONE byte access, so
    // peripheral_bus leaves scsi_dma16_lo_beat low and scsi.v must run
    // its own DRQ check: held off -> watchdog SLVERR.  With the
    // pre-8bd3ded parity inherit this beat is granted and returns OKAY.
    AxiReadResult held = axi_read_full(SCSI_DMA_SHIM | 1u, /*byte*/0, 9000);
    CHECK_TRUE("held-off beat aborts rather than hanging the fabric",
               held.completed);
    CHECK_EQ("standalone byte read of 0x101 re-checks DRQ and is held off "
             "(pre-8bd3ded parity inherit granted it: resp OKAY)",
             held.resp, 2);
    CHECK_EQ("the reserved byte is still in the FIFO",
             U_SCSI(c96_fifo_pos), 1);

    // ...and the processor collects it through reg 2, per the 53C94
    // "save last remaining byte for the processor" rule.
    dut->scsi_ctrl_in = 0;
    CHECK_EQ("reserved byte readable through reg 2", reg_r(0x2),
             sd_mock.read_sector[15]);
    return true;
}


// ─────────────────────────────────────────────────────────────────────
// 11. DMA-form DATA OUT must drain staged FIFO bytes to the target.
//
// HW hang (bitstream 57eff3a, i.e. after the phase-drift fix landed):
//   reg3=0x90 armed   reg4=0x10 (TC0, phase DATA OUT)   reg7=0x01
//   probe: xfr_armed=1 xfr_dma=1 dma_dir=OUT  latched==live==DATA_OUT(5)
//          xfr_left=511  fifo_pos=1  t_req=1  istatus=0  irq_pending=0
//   SD back end idle and healthy (2502 completions, 0 errors).
//
// The OUT-phase send beat used to carry `!c96_xfr_dma`, so with a
// DMA-form CI_XFER armed NOTHING handed staged FIFO bytes to the target.
// MAME's send is not dma-gated (ncr53c90.cpp:603-620), and its DMA-out
// completion test (:648, `dma_command && S_TC0 && fifo_pos == 0`) is
// only reachable BECAUSE those sends drain the FIFO.  So fifo_pos could
// never reach 0 and the transfer stayed armed forever with the target
// still asserting REQ — no interrupt, nothing to break the tie.
//
// MEASURED against live MAME (directed differential, identical op log):
//   before:  RTL fifo=3:aabbcc flags=03  |  MAME fifo=0: flags=00
//   after :  byte-identical on both sides
// tcounter matched at 512 throughout, so this is a missing SEND, not a
// counter desync.
//
// This is the WRITE path, so the assertions below check the bytes are
// ROUTED as payload (xfer_bytes_left advances by exactly the number of
// bytes staged), not merely popped — an unrouted pop would silently
// corrupt disk contents rather than hang.
// ─────────────────────────────────────────────────────────────────────
static bool test_dma_out_drains_staged_fifo_bytes_to_target() {
    reset();
    sd_mock.read_sector.resize(512);
    reg_w(0xC, 0x04);
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after select", seq_ok);
    // WRITE(6), 1 block -> DATA OUT.
    reg_w(0x2, 0x0A); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x01);
    axi_write(SCSI_DMA_SHIM, 0x00);

    uint8_t v = 0; bool sel_ok = false;
    for (int i = 0; i < 20000; ++i) { v = reg_r(0x4); if (v & 0x80) { sel_ok = true; break; } }
    CHECK_TRUE("select-complete interrupt fires", sel_ok);
    CHECK_EQ("phase = DATA OUT", v & 0x07, 0x00);
    (void)reg_r(0x5);

    // Arm a 512-byte DMA CI_XFER, exactly as the driver does.
    reg_w(0x0, 0x00); reg_w(0x1, 0x02);
    reg_w(0x3, 0x90);
    for (int i = 0; i < 200; ++i) tick();
    uint32_t left_before = U_SCSI(xfer_bytes_left);

    // Stage three bytes through register 2 (the FIFO port).
    reg_w(0x2, 0xAA); reg_w(0x2, 0xBB); reg_w(0x2, 0xCC);

    // MAME sends them to the target; the FIFO must drain to empty.
    bool drained = false;
    for (int i = 0; i < 20000; ++i) {
        if ((reg_r(0x7) & 0x1f) == 0) { drained = true; break; }
    }
    if (!drained)
        std::printf("  [HANG REPRODUCED] staged bytes stuck: reg7=0x%02x "
                    "armed=%u dma=%u t_req=%u\n",
                    reg_r(0x7) & 0x1f, (unsigned)U_SCSI(c96_xfr_armed),
                    (unsigned)U_SCSI(c96_xfr_dma), (unsigned)U_SCSI(t_req));
    CHECK_TRUE("DMA-form DATA OUT drains staged FIFO bytes to the target",
               drained);

    // DATA INTEGRITY: the three bytes must be accounted as write payload,
    // not discarded.  An unrouted pop would leave this unchanged.
    uint32_t left_after = U_SCSI(xfer_bytes_left);
    CHECK_EQ("staged bytes were routed as write payload (3 accounted)",
             left_before - left_after, 3u);
    return true;
}

// -----------------------------------------------------------------
// 15. REG-4 BUS PHASE DURING A PROVIDER WAIT.
//
// S_CMD_EXEC and S_VH_WAIT_RD / S_VH_WAIT_WR are OUR back-end's states,
// not the SCSI bus's: the target is still connected and still holding
// the phase it last drove, but we have stepped aside to decode the CDB
// or wait on the backing store.  MAME has no equivalent - status_r()
// (ncr53c90.cpp:1088-1095) reports the LIVE bus lines, so the phase bits
// simply do not move while a target thinks.
//
// Ours fell through c96_phase_bits' `default: 3'b000`, which is the
// single worst answer available: 000 is DATA OUT, i.e. "the target wants
// you to send it data" - reported on a READ command, with INTR low, for
// as long as the backing store takes.  On a real SD card that is
// milliseconds (go_latency models it; see SdMock).
//
// The correct answer is the phase the target is actually still driving,
// which for a CDB just accepted is COMMAND.
//
// NEGATIVE CONTROL: the scenario pins phase == S_VH_WAIT_RD and INTR
// low at the moment it samples, so it cannot pass by sampling after the
// FSM has already moved on to DATA IN (which would also read non-zero).
// -----------------------------------------------------------------
static bool test_reg4_phase_holds_bus_phase_across_provider_wait() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.go_latency = 20000;         // ~ a real SPI block time

    reg_w(0xC, 0x04);
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after select", seq_ok);
    reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x01);     // READ(6), 1 block
    axi_write(SCSI_DMA_SHIM, 0x00);

    bool in_wait = false;
    for (int i = 0; i < 600000; ++i) {
        if (U_SCSI(phase) == 6 /*S_VH_WAIT_RD*/ && U_SCSI(c96_xfer_active)) {
            in_wait = true; break;
        }
        tick();
    }
    CHECK_TRUE("FSM parks in S_VH_WAIT_RD with a live connection", in_wait);

    uint8_t st = reg_r(0x4);
    // Negative control: still parked, still silent - so the phase bits
    // below really are the ones a polling driver sees mid-wait.
    CHECK_EQ("still in S_VH_WAIT_RD when sampled", U_SCSI(phase), 6);
    CHECK_EQ("no interrupt to tell the driver otherwise", st & 0x80, 0x00);
    CHECK_EQ("reg4 reports the target's held phase (COMMAND), not the "
             "3'b000 default that reads as DATA OUT on a READ",
             st & 0x07, 0x02);

    // And it must still track the bus once the provider lands.
    sd_mock.go_latency = 0;
    bool got_in = false;
    for (int i = 0; i < 400000; ++i) {
        if ((reg_r(0x4) & 0x07) == 0x01) { got_in = true; break; }
    }
    CHECK_TRUE("phase bits follow the bus to DATA IN afterwards", got_in);
    return true;
}

// -----------------------------------------------------------------
// 16. SYNCHRONOUS-MODE DATA-IN TRANSFER COUNTER.
//
// MAME counts a DATA IN transfer down in exactly ONE of two places,
// selected by sync_offset:
//   async (sync_offset == 0) - at ACKO, in the receive path:
//       ncr53c90.cpp:472-477, `(mode == MODE_I) && (sync_offset == 0) &&
//       (phase == S_PHASE_DATA_IN)` -> decrement_tcounter()
//   sync  (sync_offset != 0) - at DACK, when the host pops:
//       ncr53c90.cpp:1198-1201, `(sync_offset != 0) || (phase !=
//       S_PHASE_DATA_IN)` -> decrement_tcounter()
//
// We implemented only the async half: the accept path decremented
// unconditionally and dma_r() carried only the `phase != DATA_IN` test.
// Register 7 is fully driver-writable (scsi.v's 4'h7 case) and drq_c96
// already consults c96_sync_offset, so the two were inconsistent with
// each other: any driver that negotiated a synchronous offset counted
// down as the CHIP accepted bytes instead of as the HOST popped them -
// up to a FIFO depth early.  TC0 then set with bytes still staged, and
// the sync DMA_IN DRQ formula (`!TC0 && fifo_pos > 1`,
// ncr53c90.cpp:1388-1389) went LOW, stranding them: a hard hang.
//
// RED before / GREEN after is the tcounter value at the moment the chip
// has staged more bytes than the chunk's count.
// -----------------------------------------------------------------
static bool test_sync_offset_counts_data_in_at_the_host_pop() {
    reset();
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.gap = 0;

    reg_w(0xC, 0x00);
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);
    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) { if (reg_r(0x6) & 7) { seq_ok = true; break; } }
    CHECK_TRUE("seq_step != 0 after select", seq_ok);
    reg_w(0x2, 0x08); reg_w(0x2, 0x00); reg_w(0x2, 0x00);
    reg_w(0x2, 0x00); reg_w(0x2, 0x01);     // READ(6), 1 block
    axi_write(SCSI_DMA_SHIM, 0x00);
    uint8_t v = 0; bool sel_ok = false;
    for (int i = 0; i < 20000; ++i) { v = reg_r(0x4); if (v & 0x80) { sel_ok = true; break; } }
    CHECK_TRUE("select-complete interrupt fires", sel_ok);
    CHECK_EQ("select-complete phase = DATA_IN", v & 0x07, 0x01);
    (void)reg_r(0x5);

    // Negotiate a synchronous offset (register 7 - sync_offset_w).
    reg_w(0x7, 0x08);
    CHECK_EQ("sync offset took", U_SCSI(c96_sync_offset), 8);

    // Arm a 4-byte DMA chunk and let the chip stage from the target.
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x04);
    reg_w(0x3, 0x90);
    bool overstaged = false;
    for (int i = 0; i < 40000; ++i) {
        if (U_SCSI(c96_fifo_pos) > 4) { overstaged = true; break; }
        tick();
    }
    // Negative control: in SYNC mode the receive path has no tcounter to
    // stop it, so the chip legitimately stages past the chunk's count
    // (MAME INIT_XFR's DATA IN arm stops only at fifo_pos == 16,
    // ncr53c90.cpp:625-627).  If this never happens the assertion below
    // proves nothing - and it is also the RED signature of the bug: an
    // accept-side decrement drives tcounter to 0, which is what stops
    // the staging AND sets TC0 AND drops DRQ on the staged bytes.
    if (!overstaged)
        std::printf("  [SYNC EARLY-TC0] staging stopped at fifo=%u with "
                    "tcounter=%u TC0=%u drq=%u - the staged bytes are "
                    "stranded\n",
                    (unsigned)U_SCSI(c96_fifo_pos),
                    (unsigned)U_SCSI(c96_tcounter),
                    (unsigned)((reg_r(0x4) & 0x10) ? 1 : 0),
                    (unsigned)dut->scsi_drq);
    CHECK_TRUE("chip stages past the chunk count in sync mode", overstaged);

    CHECK_EQ("tcounter is untouched by chip-side accepts in sync mode",
             U_SCSI(c96_tcounter), 4);
    CHECK_EQ("TC0 therefore still clear", reg_r(0x4) & 0x10, 0x00);

    // With TC0 clear and bytes staged the sync DMA_IN DRQ formula is
    // true, so the host CAN drain - the exact thing an early TC0 broke.
    dut->scsi_ctrl_in = 0x080;
    for (int i = 0; i < 200; ++i) tick();
    CHECK_TRUE("DRQ high so the host can drain (sync: !TC0 && fifo_pos>1)",
               dut->scsi_drq == 1);

    // Four host pops must be what exhausts the count.
    for (int b = 0; b < 3; ++b) {
        AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, 0, 9000);
        CHECK_TRUE("host pop completes", r.completed);
    }
    CHECK_EQ("three host pops took tcounter 4 -> 1", U_SCSI(c96_tcounter), 1);
    CHECK_EQ("TC0 still clear one byte out", reg_r(0x4) & 0x10, 0x00);
    {
        AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, 0, 9000);
        CHECK_TRUE("fourth host pop completes", r.completed);
    }
    CHECK_EQ("the fourth pop exhausts the count", U_SCSI(c96_tcounter), 0);
    CHECK_EQ("and sets TC0", reg_r(0x4) & 0x10, 0x10);
    dut->scsi_ctrl_in = 0;
    return true;
}

// -----------------------------------------------------------------
// 17. DATA-OUT DRQ MUST BACK OFF ON A NEARLY-FULL FIFO.
//
// MAME's check_drq() DMA_OUT arm (ncr53c90.cpp:1391-1393) is exactly
//     drq = !(status & S_TC0) && fifo_pos < 15
// - the `< 15` is what stops the host pushing a 16th and 17th byte into
// a FIFO that has nowhere to put them (fifo_push silently drops past
// 16).  Our DATA_OUT term carried the !TC0 half (`c96_tcounter != 0`)
// but no occupancy half at all, so reg 7 advertised "send me more" at
// any occupancy.
//
// Reg 7 is host-visible and this is the write path, so the difference is
// a driver pushing bytes the chip discards.  Asserted directly on the
// DRQ formula by injecting the occupancy, because reaching 15 staged
// bytes organically requires stalling the very send path that drains
// them - the formula is what is under test, not how the FIFO filled.
// -----------------------------------------------------------------
static bool test_data_out_drq_backs_off_near_full_fifo() {
    CHECK_TRUE("WRITE(6) select reached DATA OUT", enter_data_out_phase(1));
    (void)reg_r(0x5);

    // A large DMA chunk: !TC0 for the whole scenario, so the occupancy
    // term is the only thing that can move DRQ.
    reg_w(0x0, 0x00);
    reg_w(0x1, 0x02);
    reg_w(0x3, 0x90);
    for (int i = 0; i < 200; ++i) tick();
    CHECK_TRUE("DRQ high with an empty FIFO and count remaining",
               dut->scsi_drq == 1);
    CHECK_EQ("precondition: TC0 clear", reg_r(0x4) & 0x10, 0x00);
    CHECK_EQ("precondition: FIFO empty", U_SCSI(c96_fifo_pos), 0);

    // 14 staged bytes: still room for the two bytes of a word push.
    U_SCSI(c96_fifo_pos) = 14;
    dut->eval();
    CHECK_TRUE("DRQ still high at fifo_pos == 14 (MAME: < 15)",
               dut->scsi_drq == 1);

    // 15: MAME drops DRQ here.
    U_SCSI(c96_fifo_pos) = 15;
    dut->eval();
    CHECK_TRUE("DRQ drops at fifo_pos == 15", dut->scsi_drq == 0);

    U_SCSI(c96_fifo_pos) = 16;
    dut->eval();
    CHECK_TRUE("DRQ stays low at fifo_pos == 16", dut->scsi_drq == 0);

    U_SCSI(c96_fifo_pos) = 0;
    dut->eval();
    CHECK_TRUE("DRQ returns once the FIFO has room again",
               dut->scsi_drq == 1);
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_pb_scsi;

    RUN(test_register_pass_through_via_axi);
    RUN(test_drq_checked_read_rises_later);
    RUN(test_drq_checked_write_rises_later);
    RUN(test_drq_never_rises_times_out);
    RUN(test_drq_write_never_rises_times_out);
    RUN(test_concurrent_write_read_same_peripheral);
    RUN(test_lbtm_word_chunk_drain_matches_mame);
    RUN(test_rom_chunked_block_then_nondma_xfer_completes);
    RUN(test_cmd_write_during_provider_wait_is_queued_not_lost);
    RUN(test_dma_out_drains_staged_fifo_bytes_to_target);
    RUN(test_nondma_xfer_completes_from_any_latched_phase);
    RUN(test_word_pdma_write_is_atomic_like_dma16_w);
    RUN(test_word_pdma_read_underflow_pads_low_half_with_ff);
    RUN(test_word_pdma_read_empty_fifo_repeats_last_byte_with_ff);
    RUN(test_byte_drain_at_odd_aperture_byte_rechecks_drq);
    RUN(test_reg4_phase_holds_bus_phase_across_provider_wait);
    RUN(test_sync_offset_counts_data_in_at_the_host_pop);
    RUN(test_data_out_drq_backs_off_near_full_fifo);

    // The three 16-bit-aperture findings this file was written to name
    // (word write atomicity, and the two dma16_r underflow scenarios)
    // are FIXED as of 2026-08-19 and are now plain regression gates:
    //   - peripheral_bus.v drives scsi_dma16_lo_beat for writes too
    //     (wr_scsi_dma16_lo), so scsi.v's write-side DRQ withhold gains
    //     the same c96_dma16_lo_inherit exemption the read side has had
    //     since f63207b.
    //   - scsi.v's c96_dma16_degrade reproduces MAME's once-per-access
    //     single-byte tests: dma16_w's `fifo_pos > 14 || tcounter == 1`
    //     (ncr53c90.cpp:1354) and dma16_r's `fifo_pos < 2`
    //     (ncr53c90.cpp:1327), swallowing the split low beat and reading
    //     back the literal 0xFF filler.
    // If any of them goes red again, the RTL regressed - not the tb.
    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
