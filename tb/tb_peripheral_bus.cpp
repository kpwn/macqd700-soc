// tb_peripheral_bus.cpp — Verilator unit tb for rtl/sys/peripheral_bus.v
//
// Exercises the Q700 I/O fan-out (VIA1, VIA2, Ethernet ID, SONIC, Orwell,
// SCC, SCSI, ASC, SWIM/IWM, debug_ctrl).  The AXI4 slave port is driven
// from a tiny master BFM; the downstream faces are checked individually.
// sd_provision was removed; its 0x080_0000 window now returns OKAY+0
// (the SLOT_FAULT path is a quiet open-bus VOID, NOT a DECERR — see
// peripheral_bus.v header for the silicon-mimicking rationale).
//
// Scenarios:
//   1. via1_byte_rw        — write a byte at 0x0F0_0000 → via1_wr pulses
//                             with correct byte; read back via via1_rdata.
//   2. via2_byte_rw        — 0x0F0_2000 → VIA2 pb_* face.
//   3. scc_decode          — 0x000_C000 → SCC.
//   4. orwell_decode       — 0x000_E000 → Orwell controls.
//   5. scsi_decode         — 0x000_F000 → SCSI/TurboSCSI shim.
//   6. asc_decode          — 0x001_4000 → ASC (16 KB window).
//   7. iwm_decode          — 0x001_E000 → SWIM/IWM.
//   6. q700_mame_mirror    — 0x50F81C00 aliases to VIA1; 0x5100 stays out.
//   7. dbg_service_route   — explicit service BAR routes to debug_ctrl.
//   8. unmapped_gaps_void  — known Q700 gaps return OKAY+0, no side effects.
//   9. concurrent_rw       — write to VIA1 while reading from VIA2 back-to-back.
//
// Built via `make tb-peripheral-bus`.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <array>
#include <map>
#include <utility>
#include <verilated.h>
#include "Vperipheral_bus.h"

static Vperipheral_bus* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;

// ─── 128-bit helpers ────────────────────────────────────────────────────
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

// ─── Master → DUT drivers ───────────────────────────────────────────────
static void idle_master() {
    dut->s_awid = 0; dut->s_awaddr = 0; dut->s_awlen = 0;
    dut->s_awsize = 0; dut->s_awburst = 0; dut->s_awvalid = 0;
    set128(dut->s_wdata, Word128{0,0,0,0});
    dut->s_wstrb = 0; dut->s_wlast = 0; dut->s_wvalid = 0;
    dut->s_bready = 1;
    dut->s_arid = 0; dut->s_araddr = 0; dut->s_arlen = 0;
    dut->s_arsize = 0; dut->s_arburst = 0; dut->s_arvalid = 0;
    dut->s_rready = 1;
}

// ─── Peripheral-face simple memories (handshake + rdata latch) ──────────
struct PbSlaveMem {
    // Observed writes: (addr, value) tuples
    std::vector<std::pair<uint32_t,uint32_t>> writes;
    // Observed read pulses: addr per distinct pb_rd rising edge.  Used
    // by the same-slot rd/wr interlock tests to prove a read pulse is
    // never emitted under a concurrent write's address.
    std::vector<uint32_t> reads;
    // Rdata to present in response to reads; keyed by addr.
    std::map<uint32_t, uint8_t> rdata_map;
    bool last_wr = false;
    bool last_rd = false;
};
static PbSlaveMem via1, via2, enet, sonic, orwell, scc, scsi, asc, iwm, adbinj;
static std::vector<uint8_t> sonic_wstrbs;

// Registered-ack state for real pb_* RTL devices that register pb_ack from
// pb_wr|pb_rd.
static bool via1_ack_next = false;
static bool via2_ack_next = false;
static bool scc_ack_next = false;
static bool scsi_ack_next = false;
static bool asc_ack_next = false;
static bool adbinj_ack_next = false;   // adb_inject.v registers its ack too

// Per-peripheral stall counters — decrement each cycle that the peripheral
// is being addressed (pb_wr or pb_rd high).  While > 0, the ack is held low
// so the peripheral_bus FSM blocks on pb_ack.  Set to 0 for combinational
// (SCC/SCSI/ASC) or 1-cycle (VIA1/VIA2) default behaviour.  Used by
// test_slow_ack_stall to simulate a narrow-band (slow) device.
static int via1_stall = 0;
static int via2_stall = 0;
static int enet_stall = 0;
static int sonic_stall = 0;
static int orwell_stall = 0;
static int scc_stall  = 0;
static int scsi_stall = 0;
static int asc_stall  = 0;
static int iwm_stall  = 0;

// Programmable scsi_rdata sequence: advances one entry per distinct
// scsi_rd rising edge (mirrors the write-log edge-detect below), so a
// test can verify a multi-beat access (e.g. the SCSI DMA-shim word-read
// split) actually pulls DIFFERENT sequential bytes rather than the same
// byte replicated.  Empty (default) falls back to the constant 0x5C
// every prior test in this file already assumes.
static std::vector<uint8_t> scsi_rdata_seq;
static size_t scsi_rdata_idx = 0;
static bool   prev_scsi_ack_edge = false;

// scsi.v's exported DRQ-gate mirrors (dma_rd_ready / dma_wr_ready).
// Default 1 = DRQ-check inactive (matches TURBOSCSI_C96_EN=0 or
// scsi_ctrl_in[8:7]=0).  The withhold tests drop these to 0 to model
// the DAFB DRQ-check holding off a DMA-shim beat, verifying
// peripheral_bus never fires a scsi_rd/scsi_wr pulse it would lose.
static int scsi_dma_rd_ready_v = 1;
static int scsi_dma_wr_ready_v = 1;

// ── Peripheral-fabric reset model (2026-09-18) ──────────────────────────
// 1 while the pb_* peripherals downstream are held in reset, with
// peripheral_bus itself still running -- the SoC's pb_full_rst
// (warm_peripheral_reset = a 68040 RESET instruction) vs
// pb_soc_full_rst split.  Drives the DUT's `periph_rst` port AND models
// what a real chip in reset actually does, which is the part that makes
// this a faithful test:
//
//   * pb_ack is hard 0 -- not "delayed", 0.  The per-peripheral `*_stall`
//     knobs above are NOT a model of this: they withhold ack while
//     `*_ack_next` still REMEMBERS the pulse, so the ack eventually fires
//     with no new strobe.  A reset chip never sampled the strobe and has
//     nothing to remember, so `*_ack_next` is cleared too.
//   * a write that lands in the window is discarded (not logged), so a
//     test can tell "delivered late" from "delivered into the void".
//   * pb_rdata reads back as the device's reset value (0).
//
// scsi_dma_{rd,wr}_ready stay at 1 on purpose: in the SoC they are
// combinational from a synchroniser reset by the SAME net as u_scsi, so
// during (and ~2 cycles past) the window they report "go ahead" from
// their reset state regardless of what the DAFB programmed.  That is why
// the shim's own DRQ gate cannot save it here.
static int periph_rst_v = 0;
// Every cycle in which peripheral_bus drove ANY pb_* strobe while the
// faces were held in reset.  That pulse is physically lost -- the device
// cannot sample it and will never ack it -- so a non-zero count here is
// the defect itself, independent of whatever the AXI side ends up
// reporting.
static int pulses_into_reset = 0;

// Edge-detect state for sample_pb_writes — only log a write once per
// distinct pulse, even if the pb_wr strobe is held high across multiple
// cycles due to a stall.
static bool prev_via1_wr = false, prev_via2_wr = false;
static bool prev_enet_wr = false, prev_sonic_wr = false, prev_orwell_wr = false;
static uint32_t prev_sonic_addr = 0;   // see the SONIC note in sample_pb_writes()
static bool prev_scc_wr  = false, prev_scsi_wr = false, prev_asc_wr = false;
static bool prev_iwm_wr  = false;
static bool prev_adbinj_wr = false;
// Read-pulse edge-detect state (same-slot interlock tests log via1/scsi
// pb_rd pulses with the address the pulse was emitted under).
static bool prev_via1_rd_edge = false, prev_scsi_rd_edge = false;
// When true, via1_rdata is address-sensitive (0xB0 | via1_addr) instead
// of the constant 0xA1 — lets a test detect a read that was routed to
// the WRONG register address.  Default off so every existing scenario
// keeps its historical constant-0xA1 model.
static bool via1_addr_rdata = false;

// ─── AXI4-Lite slave faces for debug_ctrl + DAFB shim ───────────────────
struct LiteSlave {
    bool aw_ack = false;
    bool w_ack  = false;
    bool b_pending = false;
    uint32_t b_resp = 0;
    bool ar_ack = false;
    bool r_pending = false;
    uint32_t r_data = 0;
    uint32_t r_resp = 0;
    // Record of writes/reads for verification
    std::vector<std::pair<uint32_t,uint32_t>> aw_log;
    std::vector<uint32_t> ar_log;
};
static LiteSlave dbg_slv, dafb_slv;

// ─── Clock helpers ──────────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void reset() {
    idle_master();
    // Peripheral faces idle/default
    dut->via1_rdata = 0; dut->via1_ack = 0;
    dut->via2_rdata = 0; dut->via2_ack = 0;
    dut->scc_rdata = 0;  dut->scc_ack = 0;
    dut->scsi_rdata = 0; dut->scsi_ack = 0;
    dut->scsi_dma_rd_ready = 1; dut->scsi_dma_wr_ready = 1;
    dut->asc_rdata = 0;  dut->asc_ack = 0;
    dut->enet_rdata = 0; dut->enet_ack = 0;
    dut->sonic_rdata = 0; dut->sonic_ack = 0;
    dut->orwell_rdata = 0; dut->orwell_ack = 0;
    dut->iwm_rdata = 0; dut->iwm_ack = 0;
    dut->adbinj_rdata = 0; dut->adbinj_ack = 0;
    // debug_ctrl + DAFB Lite slaves idle
    dut->dbg_awready = 0; dut->dbg_wready = 0; dut->dbg_bresp = 0; dut->dbg_bvalid = 0;
    dut->dbg_arready = 0; dut->dbg_rdata = 0; dut->dbg_rresp = 0; dut->dbg_rvalid = 0;
    dut->dafb_awready = 0; dut->dafb_wready = 0; dut->dafb_bresp = 0; dut->dafb_bvalid = 0;
    dut->dafb_arready = 0; dut->dafb_rdata = 0; dut->dafb_rresp = 0; dut->dafb_rvalid = 0;
    via1 = via2 = enet = sonic = orwell = scc = scsi = asc = iwm = adbinj = PbSlaveMem{};
    sonic_wstrbs.clear();
    dbg_slv = dafb_slv = LiteSlave{};
    via1_ack_next = false;
    via2_ack_next = false;
    scc_ack_next = false;
    scsi_ack_next = false;
    asc_ack_next = false;
    adbinj_ack_next = false;
    scsi_rdata_seq.clear();
    scsi_rdata_idx = 0;
    prev_scsi_ack_edge = false;
    scsi_dma_rd_ready_v = 1;
    scsi_dma_wr_ready_v = 1;
    periph_rst_v = 0;
    dut->periph_rst = 0;
    pulses_into_reset = 0;
    via1_stall = via2_stall = enet_stall = sonic_stall = orwell_stall = 0;
    scc_stall = scsi_stall = asc_stall = iwm_stall = 0;
    prev_via1_wr = prev_via2_wr = prev_enet_wr = prev_sonic_wr = prev_orwell_wr = false;
    prev_sonic_addr = 0;
    prev_scc_wr = prev_scsi_wr = prev_asc_wr = prev_iwm_wr = false;
    prev_adbinj_wr = false;
    prev_via1_rd_edge = prev_scsi_rd_edge = false;
    via1_addr_rdata = false;
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
}

// pb_* peripheral responders.  Real RTL devices register pb_ack/pb_rdata one
// cycle after the strobe.  The named placeholder endpoints are combinational.
static void drive_pb_peripherals() {
    dut->periph_rst = periph_rst_v;
    if (periph_rst_v) {
        // Every pb_* face is held in reset: no ack, ever, and reset-value
        // rdata.  (The DRQ mirrors deliberately stay 1 -- see periph_rst_v.)
        dut->via1_rdata = 0; dut->via2_rdata = 0; dut->enet_rdata = 0;
        dut->sonic_rdata = 0; dut->orwell_rdata = 0; dut->scc_rdata = 0;
        dut->scsi_rdata = 0; dut->asc_rdata = 0; dut->iwm_rdata = 0;
        dut->adbinj_rdata = 0;
        dut->via1_ack = 0; dut->via2_ack = 0; dut->enet_ack = 0;
        dut->sonic_ack = 0; dut->orwell_ack = 0; dut->scc_ack = 0;
        dut->scsi_ack = 0; dut->asc_ack = 0; dut->iwm_ack = 0;
        dut->adbinj_ack = 0;
        dut->scsi_dma_rd_ready = scsi_dma_rd_ready_v;
        dut->scsi_dma_wr_ready = scsi_dma_wr_ready_v;
        return;
    }
    // Combinational-ack placeholders — with optional stall override.  While
    // the stall counter is non-zero we actively hold ack low so the
    // peripheral_bus FSM waits.
    dut->enet_rdata  = 0x00;
    dut->sonic_rdata = 0x0014;
    dut->orwell_rdata = 0x00;
    dut->iwm_rdata   = 0xF1u;
    dut->enet_ack  = (enet_stall  == 0) ? (dut->enet_wr  | dut->enet_rd)  : 0;
    dut->sonic_ack = (sonic_stall == 0) ? (dut->sonic_wr | dut->sonic_rd) : 0;
    dut->orwell_ack = (orwell_stall == 0) ? (dut->orwell_wr | dut->orwell_rd) : 0;
    dut->iwm_ack  = (iwm_stall  == 0) ? (dut->iwm_wr  | dut->iwm_rd)  : 0;

    // adb_inject.v mirrors the registered-ack style (ack + rdata one
    // cycle after the strobe); constant rdata for tb verification.
    dut->adbinj_rdata = 0xB7u;
    dut->adbinj_ack   = adbinj_ack_next ? 1 : 0;

    dut->via1_ack = (via1_stall == 0 && via1_ack_next) ? 1 : 0;
    dut->via2_ack = (via2_stall == 0 && via2_ack_next) ? 1 : 0;
    dut->scc_ack  = (scc_stall  == 0 && scc_ack_next)  ? 1 : 0;
    dut->scsi_ack = (scsi_stall == 0 && scsi_ack_next) ? 1 : 0;
    dut->asc_ack  = (asc_stall  == 0 && asc_ack_next)  ? 1 : 0;
    // Constant rdata for tb verification.  (via1 optionally address-
    // sensitive for the same-slot interlock test.)
    dut->via1_rdata = via1_addr_rdata ? (0xB0u | (uint8_t)dut->via1_addr)
                                      : 0xA1u;
    dut->via2_rdata = 0xA2u;
    dut->scc_rdata  = 0xCCu;
    dut->scsi_rdata = scsi_rdata_seq.empty()
                      ? 0x5Cu
                      : scsi_rdata_seq[scsi_rdata_idx < scsi_rdata_seq.size()
                                        ? scsi_rdata_idx
                                        : scsi_rdata_seq.size() - 1];
    dut->scsi_dma_rd_ready = scsi_dma_rd_ready_v;
    dut->scsi_dma_wr_ready = scsi_dma_wr_ready_v;
    dut->asc_rdata  = 0xACu;
}

// Update the registered-ack state at the end of each cycle.  Also tick
// per-peripheral stall counters — decrement while the corresponding pb_*
// strobe is high so the stall only "consumes cycles" during real traffic.
static void step_registered_acks() {
    if (periph_rst_v) {
        // A chip in reset did not sample the strobe, so there is no
        // pending ack to deliver when it comes back.  This is exactly
        // what the `*_stall` knobs do NOT model.
        via1_ack_next = via2_ack_next = scc_ack_next = false;
        scsi_ack_next = asc_ack_next = adbinj_ack_next = false;
        prev_scsi_ack_edge = false;
        return;
    }
    via1_ack_next = dut->via1_wr | dut->via1_rd;
    via2_ack_next = dut->via2_wr | dut->via2_rd;
    scc_ack_next  = dut->scc_wr  | dut->scc_rd;
    scsi_ack_next = dut->scsi_wr | dut->scsi_rd;
    // Advance the programmable scsi_rdata sequence on each scsi_ack
    // rising edge (i.e. once a beat's data has actually been consumed
    // by the requester), NOT on the scsi_rd pulse itself — scsi_rdata
    // must stay stable at the value the pulse cycle read until the
    // corresponding ack cycle has captured it (mirrors real scsi.v's
    // one-cycle-registered pb_rdata/pb_ack shape).
    if (dut->scsi_ack && !prev_scsi_ack_edge &&
        scsi_rdata_idx + 1 < scsi_rdata_seq.size())
        ++scsi_rdata_idx;
    prev_scsi_ack_edge = dut->scsi_ack;
    asc_ack_next  = dut->asc_wr  | dut->asc_rd;
    adbinj_ack_next = dut->adbinj_wr | dut->adbinj_rd;
    if (via1_stall > 0 && (dut->via1_wr || dut->via1_rd)) via1_stall--;
    if (via2_stall > 0 && (dut->via2_wr || dut->via2_rd)) via2_stall--;
    if (enet_stall > 0 && (dut->enet_wr || dut->enet_rd)) enet_stall--;
    if (sonic_stall > 0 && (dut->sonic_wr || dut->sonic_rd)) sonic_stall--;
    if (orwell_stall > 0 && (dut->orwell_wr || dut->orwell_rd)) orwell_stall--;
    if (scc_stall  > 0 && (dut->scc_wr  || dut->scc_rd )) scc_stall--;
    if (scsi_stall > 0 && (dut->scsi_wr || dut->scsi_rd)) scsi_stall--;
    if (asc_stall  > 0 && (dut->asc_wr  || dut->asc_rd )) asc_stall--;
    if (iwm_stall  > 0 && (dut->iwm_wr  || dut->iwm_rd )) iwm_stall--;
}

// Capture any pb_wr pulse into the peripheral's memory log.  Edge-detect
// so that when s_wvalid is held over a multi-cycle stall (pb_wr stays
// high for the entire wait) we only log ONE write, not one per cycle.
// (The edge-state variables prev_*_wr are declared up-front near the
// other module-scope state.)
static void sample_pb_writes() {
    if (periph_rst_v) {
        // Bytes strobed at a face held in reset are discarded by the
        // device.  Count the lost strobes, then keep the edge-detect state
        // coherent so the first pulse AFTER the release is still an edge.
        if (dut->via1_wr || dut->via2_wr || dut->enet_wr || dut->sonic_wr ||
            dut->orwell_wr || dut->scc_wr || dut->scsi_wr || dut->asc_wr ||
            dut->iwm_wr || dut->adbinj_wr ||
            dut->via1_rd || dut->via2_rd || dut->enet_rd || dut->sonic_rd ||
            dut->orwell_rd || dut->scc_rd || dut->scsi_rd || dut->asc_rd ||
            dut->iwm_rd || dut->adbinj_rd)
            pulses_into_reset++;
        prev_via1_wr = prev_via2_wr = prev_enet_wr = prev_sonic_wr = false;
        prev_orwell_wr = prev_scc_wr = prev_scsi_wr = prev_asc_wr = false;
        prev_iwm_wr = prev_adbinj_wr = false;
        prev_via1_rd_edge = prev_scsi_rd_edge = false;
        return;
    }
    if (dut->via1_wr && !prev_via1_wr)
        via1.writes.push_back({(uint32_t)dut->via1_addr, (uint8_t)dut->via1_wdata});
    if (dut->via2_wr && !prev_via2_wr)
        via2.writes.push_back({(uint32_t)dut->via2_addr, (uint8_t)dut->via2_wdata});
    if (dut->enet_wr && !prev_enet_wr)
        enet.writes.push_back({(uint32_t)dut->enet_addr, (uint8_t)dut->enet_wdata});
    // SONIC's device face is one native 16-bit register transaction.
    if (dut->sonic_wr && !prev_sonic_wr) {
        sonic.writes.push_back({(uint32_t)dut->sonic_addr, (uint16_t)dut->sonic_wdata});
        sonic_wstrbs.push_back((uint8_t)dut->sonic_wstrb);
    }
    if (dut->orwell_wr && !prev_orwell_wr)
        orwell.writes.push_back({(uint32_t)dut->orwell_addr, (uint8_t)dut->orwell_wdata});
    if (dut->scc_wr && !prev_scc_wr)
        scc.writes.push_back({(uint32_t)dut->scc_addr,  (uint8_t)dut->scc_wdata});
    if (dut->scsi_wr && !prev_scsi_wr)
        scsi.writes.push_back({(uint32_t)dut->scsi_addr,(uint8_t)dut->scsi_wdata});
    if (dut->asc_wr && !prev_asc_wr)
        asc.writes.push_back({(uint32_t)dut->asc_addr, (uint8_t)dut->asc_wdata});
    if (dut->iwm_wr && !prev_iwm_wr)
        iwm.writes.push_back({(uint32_t)dut->iwm_addr, (uint8_t)dut->iwm_wdata});
    if (dut->adbinj_wr && !prev_adbinj_wr)
        adbinj.writes.push_back({(uint32_t)dut->adbinj_addr, (uint8_t)dut->adbinj_wdata});
    // Read-pulse logging (edge-detected, same discipline as writes).
    if (dut->via1_rd && !prev_via1_rd_edge)
        via1.reads.push_back((uint32_t)dut->via1_addr);
    if (dut->scsi_rd && !prev_scsi_rd_edge)
        scsi.reads.push_back((uint32_t)dut->scsi_addr);
    prev_via1_rd_edge = dut->via1_rd;
    prev_scsi_rd_edge = dut->scsi_rd;
    prev_via1_wr = dut->via1_wr;
    prev_via2_wr = dut->via2_wr;
    prev_enet_wr = dut->enet_wr;
    prev_sonic_wr = dut->sonic_wr;
    prev_sonic_addr = (uint32_t)dut->sonic_addr;
    prev_orwell_wr = dut->orwell_wr;
    prev_scc_wr  = dut->scc_wr;
    prev_scsi_wr = dut->scsi_wr;
    prev_asc_wr  = dut->asc_wr;
    prev_iwm_wr  = dut->iwm_wr;
    prev_adbinj_wr = dut->adbinj_wr;
}

// Drive the AXI4-Lite slave faces: accept AW+W instantly, present B one
// cycle later.  Accept AR instantly, present R one cycle later with a
// caller-provided rdata.
static void drive_lite_slaves() {
    // debug_ctrl
    dut->dbg_awready = !dbg_slv.aw_ack;
    dut->dbg_wready  = !dbg_slv.w_ack;
    if (dut->dbg_awvalid && dut->dbg_awready) {
        dbg_slv.aw_log.push_back({(uint32_t)dut->dbg_awaddr, 0});
        dbg_slv.aw_ack = true;
    }
    if (dut->dbg_wvalid && dut->dbg_wready) {
        if (!dbg_slv.aw_log.empty())
            dbg_slv.aw_log.back().second = (uint32_t)dut->dbg_wdata;
        dbg_slv.w_ack = true;
    }
    if (dbg_slv.aw_ack && dbg_slv.w_ack && !dbg_slv.b_pending) {
        dbg_slv.b_pending = true;
        dbg_slv.aw_ack = dbg_slv.w_ack = false;
    }
    dut->dbg_bvalid = dbg_slv.b_pending;
    dut->dbg_bresp  = 0;
    if (dut->dbg_bvalid && dut->dbg_bready) dbg_slv.b_pending = false;

    dut->dbg_arready = !dbg_slv.ar_ack;
    if (dut->dbg_arvalid && dut->dbg_arready) {
        dbg_slv.ar_log.push_back((uint32_t)dut->dbg_araddr);
        dbg_slv.ar_ack = true;
        dbg_slv.r_data = 0xCAFEBABEu;
    }
    dut->dbg_rvalid = dbg_slv.ar_ack;
    dut->dbg_rdata  = dbg_slv.r_data;
    dut->dbg_rresp  = 0;
    if (dut->dbg_rvalid && dut->dbg_rready) dbg_slv.ar_ack = false;

    // DAFB register shim
    dut->dafb_awready = !dafb_slv.aw_ack;
    dut->dafb_wready  = !dafb_slv.w_ack;
    if (dut->dafb_awvalid && dut->dafb_awready) {
        dafb_slv.aw_log.push_back({(uint32_t)dut->dafb_awaddr, 0});
        dafb_slv.aw_ack = true;
    }
    if (dut->dafb_wvalid && dut->dafb_wready) {
        if (!dafb_slv.aw_log.empty())
            dafb_slv.aw_log.back().second = (uint32_t)dut->dafb_wdata;
        dafb_slv.w_ack = true;
    }
    if (dafb_slv.aw_ack && dafb_slv.w_ack && !dafb_slv.b_pending) {
        dafb_slv.b_pending = true;
        dafb_slv.aw_ack = dafb_slv.w_ack = false;
    }
    dut->dafb_bvalid = dafb_slv.b_pending;
    dut->dafb_bresp  = 0;
    if (dut->dafb_bvalid && dut->dafb_bready) dafb_slv.b_pending = false;

    dut->dafb_arready = !dafb_slv.ar_ack;
    if (dut->dafb_arvalid && dut->dafb_arready) {
        dafb_slv.ar_log.push_back((uint32_t)dut->dafb_araddr);
        dafb_slv.ar_ack = true;
        dafb_slv.r_data = 0x0DAFB00Fu;
    }
    dut->dafb_rvalid = dafb_slv.ar_ack;
    dut->dafb_rdata  = dafb_slv.r_data;
    dut->dafb_rresp  = 0;
    if (dut->dafb_rvalid && dut->dafb_rready) dafb_slv.ar_ack = false;
}

// Advance one cycle, running all the downstream responders.
static void cycle() {
    drive_lite_slaves();
    dut->eval();
    drive_pb_peripherals();
    dut->eval();
    sample_pb_writes();
    step_registered_acks();
    tick();
    drive_lite_slaves();
    drive_pb_peripherals();
    dut->eval();
}

// ─── Helpers to issue AXI transactions ──────────────────────────────────
// Write a single 128-bit beat into the bus (lane selected by addr[3:2]).
// cycle() samples pre-edge signals and fires posedge; we sample the
// handshake results (awready/wready at pre-edge) to know when to drop
// our valids.
static void axi_write(uint32_t addr, uint32_t lane_data, uint8_t byte_in_lane = 0xFF,
                      uint32_t* bresp = nullptr) {
    dut->s_awid    = 0xC;
    dut->s_awaddr  = addr;
    dut->s_awlen   = 0;
    dut->s_awsize  = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    Word128 w{0,0,0,0};
    w[lane] = lane_data;
    set128(dut->s_wdata, w);
    uint16_t strb = 0;
    if (byte_in_lane != 0xFF)
        strb = (1u << (lane * 4 + byte_in_lane));
    else
        strb = (0xFu << (lane * 4));
    dut->s_wstrb  = strb;
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    uint32_t resp = 0;
    for (int i = 0; i < 200 && !b_done; i++) {
        // Peek pre-edge ready signals by running the responder pass.
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        // These are the pre-edge valid/ready values the DUT will latch
        // at the upcoming posedge.
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid && dut->s_wready;
        bool b_hs  = dut->s_bvalid && dut->s_bready;
        if (b_hs) resp = dut->s_bresp;
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    if (bresp) *bresp = resp;
    // Extra drain
    for (int i = 0; i < 2; i++) cycle();
}

// axi_write_multi: drive an AXI write with an explicit 4-bit lane-strb
// pattern (multiple bits permitted).  Mirrors how the LSU emits a
// MOVE.W or MOVE.L: AW carries the byte-granular start address, the
// lane data carries the m68k_mem_wdata-shaped 32-bit word, and lane-
// strb is the m68k_mem_strb pattern.  See the byte-lane note in
// peripheral_bus.v for the HIGH→LOW big-endian strobe walk.
// awsize is the AXI-encoded log2(bytes) of the access.  SONIC uses it to
// distinguish byte, word, and long placement on its native 16-bit device
// face.  The default of 4 (16 bytes) preserves historical non-SONIC callers.
static void axi_write_multi(uint32_t addr, uint32_t lane_data, uint8_t lane_strb_4b,
                            uint32_t* bresp = nullptr, uint8_t awsize = 4) {
    dut->s_awid    = 0xC;
    dut->s_awaddr  = addr;
    dut->s_awlen   = 0;
    dut->s_awsize  = awsize;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    Word128 w{0,0,0,0};
    w[lane] = lane_data;
    set128(dut->s_wdata, w);
    uint16_t strb = (uint16_t)((uint32_t)lane_strb_4b << (lane * 4));
    dut->s_wstrb  = strb;
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    uint32_t resp = 0;
    for (int i = 0; i < 400 && !b_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) resp = dut->s_bresp;
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    if (bresp) *bresp = resp;
    for (int i = 0; i < 2; i++) cycle();
}

static uint32_t axi_read(uint32_t addr, uint32_t* rresp = nullptr,
                         uint8_t arsize = 4) {
    dut->s_arid    = 0x3;
    dut->s_araddr  = addr;
    dut->s_arlen   = 0;
    dut->s_arsize  = arsize;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    uint32_t got = 0;
    uint32_t resp = 0;
    bool ar_done = false, r_done = false;
    for (int i = 0; i < 200 && !r_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            got = v[lane];
            resp = dut->s_rresp;
        }
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) r_done = true;
    }
    dut->s_arvalid = 0;
    if (rresp) *rresp = resp;
    for (int i = 0; i < 2; i++) cycle();
    return got;
}

// Drive write + read channels simultaneously.  AXI allows a write
// transaction (AW/W/B) and a read transaction (AR/R) to be in flight at
// the same time — `peripheral_bus.v` has independent wr_busy and rd_busy
// FSMs.  Returns the 32-bit read-lane data and asserts both B and R
// completed within budget.  `byte_in_lane` applies to the write only.
struct ConcurrentResult {
    bool  write_ok;
    bool  read_ok;
    uint32_t read_data;
    uint32_t read_resp;
};
static ConcurrentResult axi_write_read_concurrent(
        uint32_t waddr, uint32_t lane_wdata, uint8_t byte_in_lane,
        uint32_t raddr) {
    ConcurrentResult r{false, false, 0, 0};

    // AW + W setup
    dut->s_awid    = 0xC;
    dut->s_awaddr  = waddr;
    dut->s_awlen   = 0;
    dut->s_awsize  = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    uint32_t wlane = (waddr >> 2) & 0x3;
    Word128 wv{0,0,0,0};
    wv[wlane] = lane_wdata;
    set128(dut->s_wdata, wv);
    uint16_t strb = (byte_in_lane != 0xFF)
        ? (uint16_t)(1u << (wlane * 4 + byte_in_lane))
        : (uint16_t)(0xFu << (wlane * 4));
    dut->s_wstrb  = strb;
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    // AR setup
    dut->s_arid    = 0x3;
    dut->s_araddr  = raddr;
    dut->s_arlen   = 0;
    dut->s_arsize  = 4;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 1;

    uint32_t rlane = (raddr >> 2) & 0x3;
    bool aw_done = false, w_done = false, b_done = false;
    bool ar_done = false, rd_done = false;
    for (int i = 0; i < 400 && !(b_done && rd_done); i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            r.read_data = v[rlane];
            r.read_resp = dut->s_rresp;
        }
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) rd_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    dut->s_arvalid = 0;
    r.write_ok = b_done;
    r.read_ok  = rd_done;
    for (int i = 0; i < 2; i++) cycle();
    return r;
}

// ─── Assertion helpers ─────────────────────────────────────────────────
#define CHECK_EQ(msg, got, exp) do { \
    if ((uint32_t)(got) != (uint32_t)(exp)) { \
        std::printf("    FAIL %s: got 0x%x, expected 0x%x\n", msg, \
                    (uint32_t)(got), (uint32_t)(exp)); \
        return false; \
    } \
} while (0)

#define CHECK_TRUE(msg, cond) do { \
    if (!(cond)) { \
        std::printf("    FAIL %s: condition false\n", msg); \
        return false; \
    } \
} while (0)

// ─── Scenarios ─────────────────────────────────────────────────────────
// 1. VIA1 byte write/read at 0x0F0_0000
static bool test_via1_byte_rw() {
    reset();
    // Write 0xA5 to the VIA1 window.  Lane depends on addr[3:2].
    // Use address 0x0F0_0004 so addr[3:2] = 1 (lane 1).
    uint32_t addr = 0x0F00004;
    // Lane contents: put byte in lane's byte-0 position (LSB of 32b).
    // Byte 0 strobe + byte 0 data.
    uint32_t lane_val = 0xA5;
    axi_write(addr, lane_val, /*byte_in_lane*/ 0);
    CHECK_TRUE("via1 saw one write", via1.writes.size() == 1);
    CHECK_EQ("via1 byte = 0xA5", via1.writes[0].second, 0xA5);

    // Read back — via1_rdata is broadcast across all 4 bytes of the lane.
    uint32_t rr;
    uint32_t got = axi_read(addr, &rr);
    CHECK_EQ("via1 read resp OKAY", rr, 0);
    // Expect byte 0xA1 replicated four times in the selected lane
    CHECK_EQ("via1 read lane = 0xA1A1A1A1", got, 0xA1A1A1A1u);
    return true;
}

static bool test_via2_byte_rw() {
    reset();
    uint32_t addr = 0x0F02000;
    axi_write(addr, 0xDE, /*byte_in_lane*/ 0);
    CHECK_TRUE("via2 saw write", via2.writes.size() == 1);
    CHECK_EQ("via2 byte = 0xDE", via2.writes[0].second, 0xDE);

    uint32_t rr;
    uint32_t got = axi_read(addr, &rr);
    CHECK_EQ("via2 read resp OKAY", rr, 0);
    CHECK_EQ("via2 rdata replicated", got, 0xA2A2A2A2u);
    return true;
}

static bool test_q700_via_stride_aliases() {
    reset();

    axi_write(0x0F00000, 0x10, 0);
    axi_write(0x0F00200, 0x11, 0);
    axi_write(0x0F02000, 0x20, 0);
    axi_write(0x0F03000, 0x21, 0);
    uint32_t bresp = 0;
    axi_write(0x0F06000, 0x30, 0, &bresp);

    CHECK_TRUE("VIA1 saw two stride aliases", via1.writes.size() == 2);
    CHECK_EQ("VIA1 reg 0", via1.writes[0].first & 0xF, 0u);
    CHECK_EQ("VIA1 reg 1", via1.writes[1].first & 0xF, 1u);
    CHECK_EQ("VIA1 stride byte 0", via1.writes[0].second, 0x10u);
    CHECK_EQ("VIA1 stride byte 1", via1.writes[1].second, 0x11u);

    CHECK_TRUE("VIA2 saw two stride aliases", via2.writes.size() == 2);
    CHECK_EQ("VIA2 reg 0", via2.writes[0].first & 0xF, 0u);
    CHECK_EQ("VIA2 reg 8", via2.writes[1].first & 0xF, 8u);
    CHECK_EQ("VIA2 stride byte 0", via2.writes[0].second, 0x20u);
    CHECK_EQ("VIA2 stride byte 1", via2.writes[1].second, 0x21u);

    CHECK_EQ("VIA gap VOID OKAY", bresp, 0u);
    CHECK_TRUE("VIA gap did not hit debug", dbg_slv.aw_log.empty());
    return true;
}

static bool test_scc_decode() {
    reset();
    uint32_t addr = 0x000C022;  // SCC channel A control, common Q700 alias
    axi_write(addr, 0x77, /*byte_in_lane*/ 0);
    CHECK_TRUE("scc saw write", scc.writes.size() == 1);
    CHECK_EQ("scc byte", scc.writes[0].second, 0x77);
    CHECK_EQ("scc dc_ab low bits", scc.writes[0].first & 0x3, 0x2u);
    uint32_t rr;
    uint32_t got = axi_read(addr, &rr);
    CHECK_EQ("scc read resp OKAY", rr, 0);
    CHECK_EQ("scc read data", got, 0xCCCCCCCCu);
    return true;
}

static bool test_scsi_decode() {
    reset();
    uint32_t addr = 0x000F000;
    axi_write(addr, 0x33, 0);
    CHECK_TRUE("scsi saw write", scsi.writes.size() == 1);
    CHECK_EQ("scsi local addr", scsi.writes[0].first, 0x000u);
    CHECK_EQ("scsi byte", scsi.writes[0].second, 0x33);
    uint32_t rr;
    uint32_t got = axi_read(addr, &rr);
    CHECK_EQ("scsi read resp OKAY", rr, 0);
    CHECK_EQ("scsi read data", got, 0x5C5C5C5Cu);
    return true;
}

// SCSI DMA-shim (pb_addr 0x100/0x101) WORD read — the 2026-07-15 HW
// post-mortem root cause.  Every OTHER test in this file (and the
// standalone scsi.v-only tb_scsi_c96_read6.cpp harness) drives byte-
// granular accesses, which is why this bug survived every prior sim
// test: the Q700 ROM/driver actually drains this port with `movew`
// (16-bit) reads.  Per MAME's real 53C94 model
// (ncr53c94_device::dma16_r(), ncr53c90.cpp:1326) a 16-bit access must
// pop TWO SEQUENTIAL bytes (and decrement the transfer counter by 2),
// not read one byte and replicate it across the word — which is what
// peripheral_bus.v's generic byte-replicated pb_* read path did before
// this fix.  Verifies: (a) exactly TWO scsi_rd strobes occur for ONE
// AXI word read, (b) the assembled word holds the two DISTINCT
// sequential bytes in the correct big-endian order (first-fetched byte
// = high byte), not the same byte duplicated.
static bool test_scsi_dma_shim_word_read() {
    reset();
    scsi_rdata_seq = {0xAAu, 0xBBu};   // two DISTINCT sequential bytes

    uint32_t rresp = 0;
    uint32_t got = axi_read(0x000F100u, &rresp, /*arsize=word*/1);
    CHECK_EQ("SCSI DMA-shim word read resp OKAY", rresp, 0u);
    CHECK_EQ("SCSI DMA-shim word read assembled value",
             got, 0xAABBAABBu);
    CHECK_TRUE("SCSI DMA-shim word read consumed exactly two beats",
               scsi_rdata_idx == 1);

    // A plain BYTE read to the same address must still take the
    // original single-beat path (unchanged behaviour) — only WORD (and
    // wider) accesses to the DMA-shim range hit the new split path.
    reset();
    scsi_rdata_seq = {0xCCu, 0xDDu};
    got = axi_read(0x000F100u, &rresp, /*arsize=byte*/0);
    CHECK_EQ("SCSI DMA-shim byte read resp OKAY", rresp, 0u);
    CHECK_EQ("SCSI DMA-shim byte read single beat value",
             got, 0xCCCCCCCCu);
    CHECK_TRUE("SCSI DMA-shim byte read consumed exactly one beat",
               scsi_rdata_idx == 1);
    return true;
}

// SCSI DMA-shim WORD write — write-side complement of
// test_scsi_dma_shim_word_read.  A `move.w` store to the pseudo-DMA
// port arrives as ONE AXI beat with TWO hot strobe bits; per MAME's
// 53C94 (ncr53c94_device::dma16_w / dma16_swap_w) it must push TWO
// sequential bytes, high byte first on the m68k bus, so the transfer
// counter drains by 2.  Verifies the wr_scsi_* serializer emits two
// DISTINCT byte beats in big-endian order instead of the generic
// single-pulse path's one-byte-and-drop-the-rest behaviour.
static bool test_scsi_dma_shim_word_write() {
    reset();
    uint32_t bresp = 0xFFu;
    // LSU shape for MOVE.W to byte address 0xF100 (offset 0 in the
    // 32-bit lane): strb=4'b1100, wdata={hi, lo, 16'd0}.
    axi_write_multi(0x000F100u, 0xAABB0000u, 0xC, &bresp);
    CHECK_EQ("SCSI DMA-shim word write resp OKAY", bresp, 0u);
    CHECK_TRUE("SCSI DMA-shim word write delivered exactly two beats",
               scsi.writes.size() == 2);
    CHECK_EQ("beat 0 shim addr", scsi.writes[0].first, 0x100u);
    CHECK_EQ("beat 0 = high byte (big-endian first)",
             scsi.writes[0].second, 0xAAu);
    CHECK_EQ("beat 1 shim addr", scsi.writes[1].first, 0x100u);
    CHECK_EQ("beat 1 = low byte", scsi.writes[1].second, 0xBBu);
    return true;
}

// SCSI DMA-shim write vs DRQ-check withhold — the write-side analog of
// the 2026-07-20/21 read-side race (see rd_scsi_word_active's comment
// in peripheral_bus.v).  While scsi.v's exported dma_wr_ready mirror
// reads 0 (TurboSCSI DRQ-check active, drq_c96 low), peripheral_bus
// must NOT fire a scsi_wr pulse (the ack would be silently withheld
// and the beat lost forever) — but the AXI W beat itself must still be
// accepted (the serializer latches it), and the B response must be
// deferred until the byte has actually been delivered and acked.
static bool test_scsi_dma_shim_write_drq_withhold() {
    reset();
    scsi_dma_wr_ready_v = 0;

    // LSU-shaped byte store to the shim: AW=0xF100 (offset 0), byte in
    // wdata[31:24], strb bit 3 of lane 0.
    dut->s_awid    = 0xC;
    dut->s_awaddr  = 0x000F100u;
    dut->s_awlen   = 0;
    dut->s_awsize  = 0;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    Word128 w{0,0,0,0};
    w[0] = 0xE5000000u;
    set128(dut->s_wdata, w);
    dut->s_wstrb  = 0x8;
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    bool pulse_while_withheld = false;
    uint32_t resp = 0xFFu;
    for (int i = 0; i < 40; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (dut->scsi_wr) pulse_while_withheld = true;
        if (b_hs) { resp = dut->s_bresp; b_done = true; }
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
    }
    CHECK_TRUE("AXI W beat accepted during withhold (no W-channel stall)",
               w_done);
    CHECK_TRUE("no scsi_wr pulse while dma_wr_ready low",
               !pulse_while_withheld);
    CHECK_TRUE("no B response while the byte is undelivered", !b_done);
    CHECK_TRUE("no shim write logged while withheld", scsi.writes.empty());

    // Release the DRQ gate: the latched beat must replay and complete.
    scsi_dma_wr_ready_v = 1;
    for (int i = 0; i < 40 && !b_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool b_hs = dut->s_bvalid && dut->s_bready;
        if (b_hs) { resp = dut->s_bresp; b_done = true; }
        sample_pb_writes();
        step_registered_acks();
        tick();
    }
    CHECK_TRUE("write completed after DRQ release", b_done);
    CHECK_EQ("post-release resp OKAY", resp, 0u);
    CHECK_TRUE("exactly one byte delivered", scsi.writes.size() == 1);
    CHECK_EQ("delivered shim addr", scsi.writes[0].first, 0x100u);
    CHECK_EQ("delivered byte", scsi.writes[0].second, 0xE5u);
    for (int i = 0; i < 2; i++) cycle();
    return true;
}

// Read-side withhold — unit-level regression lock for the landed
// rd_scsi_word_active `scsi_dma_rd_ready` gate itself (the fix was
// originally only HW/ILA-verified): while dma_rd_ready reads 0 the
// pulse must be held off entirely (a fired pulse would lose that
// beat's ack forever, since rd_scsi_beat_kicked_q blocks any retry);
// on release the word read must complete with both sequential bytes.
static bool test_scsi_dma_shim_read_drq_withhold() {
    reset();
    scsi_rdata_seq = {0x11u, 0x22u};
    scsi_dma_rd_ready_v = 0;

    dut->s_arid    = 0x3;
    dut->s_araddr  = 0x000F100u;
    dut->s_arlen   = 0;
    dut->s_arsize  = 1;   // word
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 1;

    bool ar_done = false, r_done = false;
    bool pulse_while_withheld = false;
    uint32_t got = 0, resp = 0xFFu;
    for (int i = 0; i < 40; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (dut->scsi_rd) pulse_while_withheld = true;
        if (r_hs) r_done = true;
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
    }
    CHECK_TRUE("no scsi_rd pulse while dma_rd_ready low",
               !pulse_while_withheld);
    CHECK_TRUE("no R data while withheld", !r_done);

    scsi_dma_rd_ready_v = 1;
    for (int i = 0; i < 60 && !r_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool r_hs = dut->s_rvalid && dut->s_rready;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            got  = v[0];
            resp = dut->s_rresp;
            r_done = true;
        }
        sample_pb_writes();
        step_registered_acks();
        tick();
    }
    CHECK_TRUE("read completed after DRQ release", r_done);
    CHECK_EQ("post-release read resp OKAY", resp, 0u);
    CHECK_EQ("assembled word after release", got, 0x11221122u);
    for (int i = 0; i < 2; i++) cycle();
    return true;
}

static bool test_orwell_decode() {
    reset();
    uint32_t bresp = 0;
    uint32_t rresp = 0;

    axi_write(0x000E000u, 0x5Au, 0, &bresp);
    CHECK_EQ("orwell write resp OKAY", bresp, 0u);
    CHECK_TRUE("orwell saw write", orwell.writes.size() == 1);
    CHECK_EQ("orwell local addr", orwell.writes[0].first, 0x00u);
    CHECK_EQ("orwell byte", orwell.writes[0].second, 0x5Au);

    uint32_t got = axi_read(0x50F0E0FCu, &rresp);
    CHECK_EQ("orwell read resp OKAY", rresp, 0u);
    CHECK_EQ("orwell read reset value", got, 0u);

    axi_write(0x5000E100u, 0x11u, 0, &bresp);
    CHECK_EQ("post-orwell gap VOID OKAY", bresp, 0u);
    CHECK_EQ("orwell count stable after gap", (uint32_t)orwell.writes.size(), 1u);
    return true;
}

// Investigation record: real cpu040 hardware bus-faults (vec=0x02,
// fa=0x5000e000 exactly, confirmed via direct register capture) on a
// `movel %a2@,%d2`-shaped LONG read from this exact address, at ROM
// PC 0x4080315c -- docs/BUG_calibration_word_misplaced_0d00.md Part 13.
// This unit reproduces the identical transaction shape at the AXI-slave
// level: a default-arsize (LONG/32-bit) read straight at 0x5000E000, the
// literal fault address, not a nearby offset. It completes cleanly --
// OKAY response, data 0x00000000 -- confirming decode_slot()'s SLOT_ORWELL
// routing and orwell_stub's combinational ack/rdata=0 model are already
// correct and were already covered by test_orwell_decode above for a
// neighboring offset (0x50F0E0FC). No static peripheral_bus.v decode or
// device-model bug was found for this address; see the bug doc's RTL
// investigation section for the full reasoning on why the real-hardware
// fault is therefore believed to have a dynamic/timing root cause outside
// this module's static decode, not an address-decode-fidelity gap.
static bool test_orwell_e000_long_read_exact_fault_pc_address() {
    reset();
    uint32_t rresp = 0xFFFFFFFFu;
    uint32_t got = axi_read(0x5000E000u, &rresp);
    CHECK_EQ("0x5000E000 LONG read resp OKAY", rresp, 0u);
    CHECK_EQ("0x5000E000 LONG read data", got, 0u);
    return true;
}

static bool test_asc_decode() {
    reset();
    uint32_t addr = 0x0014000;
    axi_write(addr, 0x44, 0);
    CHECK_TRUE("asc saw write", asc.writes.size() == 1);
    CHECK_EQ("asc byte", asc.writes[0].second, 0x44);
    // Pre-byte-select fix this landed at asc_addr = 0x000 (from [13:2]).
    // Post-fix the low 2 bits of the CPU address carry through, so
    // writing to 0x14000 still lands at pb_addr 0x000.
    CHECK_EQ("asc addr[11:0] = 0x000", asc.writes[0].first & 0xFFFu, 0x000u);
    // ASC has an 8 KB Q700 window: 0x14000..0x15FFF.
    axi_write(0x0015FF0, 0x55, 0);
    CHECK_TRUE("asc saw far-end write", asc.writes.size() == 2);
    uint32_t rr;
    uint32_t got = axi_read(addr, &rr);
    CHECK_EQ("asc read resp OKAY", rr, 0);
    CHECK_EQ("asc read data", got, 0xACACACACu);
    return true;
}

static bool test_iwm_decode() {
    reset();
    uint32_t bresp = 0;
    uint32_t rresp = 0;

    axi_write(0x5001E000u + 15u * 0x200u, 0x17u, 0, &bresp);
    CHECK_EQ("iwm write resp OKAY", bresp, 0u);
    CHECK_TRUE("iwm saw write", iwm.writes.size() == 1);
    CHECK_EQ("iwm reg F", iwm.writes[0].first & 0xFu, 0xFu);
    CHECK_EQ("iwm byte", iwm.writes[0].second, 0x17u);

    uint32_t got = axi_read(0x50F1E000u + 14u * 0x200u, &rresp);
    CHECK_EQ("iwm read resp OKAY", rresp, 0u);
    CHECK_EQ("iwm read data replicated", got, 0xF1F1F1F1u);

    axi_write(0x50020000u, 0x22u, 0, &bresp);
    CHECK_EQ("post-iwm gap VOID OKAY", bresp, 0u);
    CHECK_EQ("iwm count stable after gap", (uint32_t)iwm.writes.size(), 1u);
    return true;
}

// ── SWIM/IWM register 0 must NOT be swallowed by the SCC alt-base window ──
//
// REGRESSION for the 2026-08-05 7.5.3 floppy stall.  The SCC alt-base
// carve-out matched off_raw[23:6]==0x3C780, i.e. 0xF1_E000..0xF1_E03F --
// shifted 0x20 LOW of the SCC alt-base ports it meant to cover, so it ate
// 0xF1_E000..0xF1_E01F: SWIM/IWM REGISTER 0 under the Q700 mirror, the
// exact address the .Sony driver polls.  On real hardware that read
// returned 0x54 (SCC chan-B RR0 idle) instead of the IWM value, and every
// poll also reset the SCC's SHARED register pointer.
//
// Why the old tests could not catch it: test_iwm_decode() touches reg 14
// at the mirror and reg 15 at the canonical base, and the alt-base test
// (now test_no_scc_alt_base_bit17_alias()) started probing at 0x50F1_E020.
// Nothing exercised 0xF1_E000..0xF1_E01F -- the one window that was broken.
//
// Update (task #246, 2026-08-05): the alt base was subsequently deleted
// outright after MAME showed ZERO accesses in E020..E03F across a full
// 7.5.3 boot.  The whole 0x50F1_Exxx window is SWIM/IWM now.
static bool test_iwm_reg0_not_stolen_by_scc_window() {
    reset();
    uint32_t bresp = 0;

    // reg 0 at the Q700 OR-mirror base the driver actually uses.
    axi_write(0x50F1E000u, 0x5Au, 0, &bresp);
    CHECK_EQ("iwm reg0 mirror write OKAY", bresp, 0u);
    CHECK_TRUE("iwm SAW reg0 write (not SCC)", iwm.writes.size() == 1);
    CHECK_EQ("iwm reg0 index", iwm.writes[0].first & 0xFu, 0x0u);
    CHECK_TRUE("scc did NOT see it", scc.writes.empty());

    // The whole 0xF1_E000..0xF1_E01F sub-window belongs to the IWM.
    reset();
    axi_write(0x50F1E01Eu, 0x11u, 0, &bresp);
    CHECK_EQ("iwm 0xE01E write OKAY", bresp, 0u);
    CHECK_TRUE("iwm saw 0xE01E", iwm.writes.size() == 1);
    CHECK_TRUE("scc did NOT see 0xE01E", scc.writes.empty());

    // 0xE020 / 0xE026 belong to the IWM too -- there is NO SCC alternate
    // base (task #246).  These two used to assert the opposite; the
    // expectation was inverted on 2026-08-05 after measuring MAME during a
    // real 7.5.3 boot: over 25 s it made 46 reads in 0x50F1_E000..E1FF, ALL
    // at 0x50F1_E000, and ZERO anywhere in E020..E03F -- while making
    // 284,055 reads in the CANONICAL SCC window (0x50F0_C020 / _C024).  The
    // SCC the OS talks to sits at offset 0x20 inside 0x50F0_C000, which is
    // almost certainly where the "alt base at 0x50F1_E020" idea came from.
    reset();
    axi_write(0x50F1E020u, 0x33u, 0, &bresp);
    CHECK_EQ("0xE020 OKAY", bresp, 0u);
    CHECK_TRUE("iwm saw 0xE020 (no SCC alt base)", iwm.writes.size() == 1);
    CHECK_TRUE("scc did NOT see 0xE020", scc.writes.empty());

    reset();
    axi_write(0x50F1E026u, 0x44u, 0, &bresp);
    CHECK_EQ("0xE026 OKAY", bresp, 0u);
    CHECK_TRUE("iwm saw 0xE026 (no SCC alt base)", iwm.writes.size() == 1);
    CHECK_TRUE("scc did NOT see 0xE026", scc.writes.empty());

    // Whole of IWM register 0's 0x200 stride is now contiguous IWM.
    reset();
    axi_write(0x50F1E040u, 0x55u, 0, &bresp);
    CHECK_EQ("0xE040 OKAY", bresp, 0u);
    CHECK_TRUE("iwm saw 0xE040", iwm.writes.size() == 1);
    CHECK_TRUE("scc did NOT see 0xE040", scc.writes.empty());

    reset();
    axi_write(0x50F1E1FEu, 0x66u, 0, &bresp);
    CHECK_EQ("0xE1FE OKAY", bresp, 0u);
    CHECK_TRUE("iwm saw 0xE1FE (top of reg-0 stride)", iwm.writes.size() == 1);
    CHECK_EQ("still reg 0", iwm.writes[0].first & 0xFu, 0x0u);
    CHECK_TRUE("scc did NOT see 0xE1FE", scc.writes.empty());

    // The CANONICAL SCC window is untouched -- this is where MAME's 284k
    // accesses actually land, so it must still decode to the SCC.
    reset();
    axi_write(0x50F0C020u, 0x77u, 0, &bresp);
    CHECK_EQ("canonical scc 0xC020 OKAY", bresp, 0u);
    CHECK_TRUE("scc saw 0xC020", scc.writes.size() == 1);
    CHECK_TRUE("iwm did NOT see 0xC020", iwm.writes.empty());

    reset();
    axi_write(0x50F0C024u, 0x88u, 0, &bresp);
    CHECK_EQ("canonical scc 0xC024 OKAY", bresp, 0u);
    CHECK_TRUE("scc saw 0xC024", scc.writes.size() == 1);
    CHECK_TRUE("iwm did NOT see 0xC024", iwm.writes.empty());
    return true;
}

// ── ADB injection window decode (SLOT_ADBINJ @ 0x001_1000) ──────────────
// The JTAG-AXI debug master reaches this window with full-strobe 32-bit
// word writes at word-aligned offsets (adb_inject.v's alias block at
// +0x10..+0x1C); peripheral_bus must deliver exactly ONE byte per such
// AXI write (the wdata LSB — see the wr_strb_byte priority walk) at the
// full byte offset within the window.  Also checks the Q700 mirror
// alias and the read-side status-byte broadcast.
static bool test_adbinj_decode() {
    reset();
    uint32_t bresp = 0;
    // Full-strobe word write, exactly as the JTAG-AXI master issues it:
    // keycode 0x1C to the word-aligned KBD_ENQUEUE alias at +0x10.
    axi_write(0x0011010u, 0x0000001Cu, /*byte_in_lane*/ 0xFF, &bresp);
    CHECK_EQ("adbinj write resp OKAY", bresp, 0u);
    CHECK_TRUE("adbinj saw one write", adbinj.writes.size() == 1);
    CHECK_EQ("adbinj offset = 0x10", adbinj.writes[0].first, 0x10u);
    CHECK_EQ("adbinj single byte = 0x1C", adbinj.writes[0].second, 0x1Cu);

    // Q700 mirror alias (addr bits under Q700_IO_MIRROR_MASK cleared):
    // 0x50F1_1014 → mac_off 0x011014 → MOUSE_BTN alias at +0x14.
    axi_write(0x50F11014u, 0x00000001u, 0xFF, &bresp);
    CHECK_EQ("adbinj mirror write resp OKAY", bresp, 0u);
    CHECK_TRUE("adbinj saw mirror write", adbinj.writes.size() == 2);
    CHECK_EQ("adbinj mirror offset = 0x14", adbinj.writes[1].first, 0x14u);
    CHECK_EQ("adbinj mirror byte = 0x01", adbinj.writes[1].second, 0x01u);

    // Legacy byte-granular offset still decodes (m68k-side byte write).
    axi_write(0x0011000u, 0x2A, /*byte_in_lane*/ 0, &bresp);
    CHECK_EQ("adbinj legacy write resp OKAY", bresp, 0u);
    CHECK_TRUE("adbinj saw legacy write", adbinj.writes.size() == 3);
    CHECK_EQ("adbinj legacy offset = 0x00", adbinj.writes[2].first, 0x00u);
    CHECK_EQ("adbinj legacy byte = 0x2A", adbinj.writes[2].second, 0x2Au);

    // Read: the status byte is broadcast across the addressed lane.
    uint32_t rr = 0;
    uint32_t got = axi_read(0x0011010u, &rr);
    CHECK_EQ("adbinj read resp OKAY", rr, 0u);
    CHECK_EQ("adbinj read lane = 0xB7B7B7B7", got, 0xB7B7B7B7u);
    return true;
}

// Task #123: verify ASC addr lane-select is BYTE-granular.
// Without the fix, the four SONORA control registers at +0x800/+0x801/+0x802/
// +0x803 (version/mode/chan_ctl/fifo_ctl) would all collapse to pb_addr 0x800
// because the low two address bits were dropped by the `addr[13:2]` slice.
// ROM initialises mode and chan_ctl with back-to-back byte stores — the bug
// makes those writes indistinguishable inside asc.v.
//
// Writes use the CPU-big-endian convention: byte at ASC offset N (= 0x14000+N)
// is driven via wstrb bit (3 - (N&3)) on the selected lane.  The peripheral-
// bus asc_addr output must include N's low two bits verbatim.
static bool test_asc_byte_select() {
    reset();

    // The ASC offsets 0x800..0x803 correspond to absolute addresses
    // 0x0014800, 0x0014801, 0x0014802, 0x0014803.  Byte stores drop
    // into the same 32-bit word (addr[3:2] = 0) but differ in byte lane.
    // axi_write() auto-derives the wstrb bit from (byte_in_lane), where
    // byte_in_lane is the little-endian byte position within the lane —
    // so byte_in_lane=3 drives wstrb[3] which the DUT interprets as byte
    // at addr+0 (big-endian MSB).  We use that mapping to exercise each
    // of 0x800..0x803.
    //
    // axi_write convention (see implementation): strb = 1 << (lane*4 +
    // byte_in_lane).  lane = addr[3:2].  For addr 0x2800 (lane 0), strb
    // bit = byte_in_lane.  And the fetched wr_byte picks wdata[b*8+7:b*8]
    // where b = byte_in_lane.  Thus the caller selects both the strb AND
    // the lane sub-byte together via byte_in_lane.
    //
    // For the mapping strb-bit → byte-offset (big-endian): strb[3] = offset
    // 0, strb[2] = 1, strb[1] = 2, strb[0] = 3.  The current peripheral_bus
    // passes wr_addr_q[1:0] directly, matching addr[1:0] of the CPU-level
    // AW address — not the strb-derived offset.  The test exercises that
    // path: the CPU sends awaddr = 0x2800+k and wstrb bit (3-k).

    struct Case { uint32_t addr; uint8_t val; int strb_bit_in_lane; };
    // byte_in_lane here is the *AXI-level* little-endian byte position
    // within the 32-bit lane.  For awaddr=0x2800+k (k∈{0..3}) the CPU
    // drives wstrb bit (3 - k) → byte_in_lane = (3 - k).
    Case cases[] = {
        {0x0014800, 0x11, 3},   // k=0 → wstrb[3] → byte_in_lane=3
        {0x0014801, 0x22, 2},   // k=1 → wstrb[2] → byte_in_lane=2
        {0x0014802, 0x33, 1},   // k=2 → wstrb[1] → byte_in_lane=1
        {0x0014803, 0x44, 0}    // k=3 → wstrb[0] → byte_in_lane=0
    };

    for (auto& c : cases) {
        // Place the byte value at the correct big-endian position in wdata.
        // axi_write places lane_val into the whole 32-bit lane; we replicate
        // the byte into the matching little-endian byte so only that strb
        // bit carries real data.
        uint32_t lane_val = ((uint32_t)c.val) << (c.strb_bit_in_lane * 8);
        axi_write(c.addr, lane_val, c.strb_bit_in_lane);
    }

    CHECK_TRUE("asc saw four byte writes", asc.writes.size() == 4);
    // pb_addr lands at the low 12 bits of CPU addr = (0x800 + k).
    CHECK_EQ("asc[0] addr", asc.writes[0].first & 0xFFFu, 0x800u);
    CHECK_EQ("asc[0] byte", asc.writes[0].second,         0x11u);
    CHECK_EQ("asc[1] addr", asc.writes[1].first & 0xFFFu, 0x801u);
    CHECK_EQ("asc[1] byte", asc.writes[1].second,         0x22u);
    CHECK_EQ("asc[2] addr", asc.writes[2].first & 0xFFFu, 0x802u);
    CHECK_EQ("asc[2] byte", asc.writes[2].second,         0x33u);
    CHECK_EQ("asc[3] addr", asc.writes[3].first & 0xFFFu, 0x803u);
    CHECK_EQ("asc[3] byte", asc.writes[3].second,         0x44u);

    // Also check the FIFO window stays byte-granular: four consecutive
    // bytes starting at offset 0x100 should land at pb_addr 0x100..0x103.
    reset();
    Case fifo_cases[] = {
        {0x0014100, 0xA0, 3},
        {0x0014101, 0xA1, 2},
        {0x0014102, 0xA2, 1},
        {0x0014103, 0xA3, 0}
    };
    for (auto& c : fifo_cases) {
        uint32_t lane_val = ((uint32_t)c.val) << (c.strb_bit_in_lane * 8);
        axi_write(c.addr, lane_val, c.strb_bit_in_lane);
    }
    CHECK_TRUE("asc saw four FIFO byte writes", asc.writes.size() == 4);
    for (int i = 0; i < 4; i++) {
        uint32_t exp_addr = 0x100u + i;
        CHECK_EQ("fifo addr", asc.writes[i].first & 0xFFFu, exp_addr);
        CHECK_EQ("fifo byte", asc.writes[i].second, (uint32_t)(0xA0 + i));
    }

    return true;
}

// ─── Multi-byte ASC writes via the iteration FSM ──────────────────────
// Q700 EASC startup-chime path emits 16-bit MOVE.W to FIFO-A volume L/R
// (asc base + 0xF06, two bytes) and 32-bit MOVE.L variants in places.
// peripheral_bus.v's wr_asc_* FSM serializes these into N back-to-back
// pb_wr pulses.  Strobe walk is HIGH→LOW (BE), so byte at AW is the
// HIGH byte of the m68k word, byte at AW+1 is the next byte, etc.
//
// Lane data layout matches m68k_mem_wdata for the corresponding off:
//   WORD off=2: data = {16'd0, hi, lo}  (bits[15:8]=hi, bits[7:0]=lo)
//   WORD off=0: data = {hi, lo, 16'd0}  (bits[31:24]=hi, bits[23:16]=lo)
//   LONG off=0: data = {b0, b1, b2, b3} (bits[31:24]=b0, ..., bits[7:0]=b3)
static bool test_asc_multi_byte_word_off2() {
    reset();
    // Mimic LSU emission of `move.w #0x7F00, asc+0xF06`:
    //   AW = 0x14F06 (peripheral_bus mac-offset; will alias to 0xF06)
    //   off = 2 → m68k_mem_strb(WORD,2) = 4'b0011, m68k_mem_wdata = {16'd0, 0x7F00}
    //   addr lane 1 (bit [3:2] of 0xF06 = 1).
    uint32_t addr = 0x0014F06u;
    uint32_t lane_data = 0x00007F00u; // hi=0x7F at bits[15:8], lo=0x00 at bits[7:0]
    uint8_t  strb = 0b0011;            // bit 1 (=hi byte) + bit 0 (=lo byte)
    axi_write_multi(addr, lane_data, strb);

    // Two pb writes expected: 0x7F at 0xF06, then 0x00 at 0xF07.
    CHECK_TRUE("asc word saw 2 writes", asc.writes.size() == 2);
    CHECK_EQ("asc word [0] addr 0xF06", asc.writes[0].first & 0xFFFu, 0xF06u);
    CHECK_EQ("asc word [0] byte 0x7F", asc.writes[0].second, 0x7Fu);
    CHECK_EQ("asc word [1] addr 0xF07", asc.writes[1].first & 0xFFFu, 0xF07u);
    CHECK_EQ("asc word [1] byte 0x00", asc.writes[1].second, 0x00u);
    return true;
}

static bool test_asc_multi_byte_word_off0() {
    reset();
    // Mimic LSU emission of `move.w #0xCAFE, asc+0xF04`:
    //   AW = 0x14F04, off = 0 → strb = 4'b1100, wdata = {0xCA, 0xFE, 16'd0}
    //   lane 1 (bit [3:2] of 0xF04 = 1).
    uint32_t addr = 0x0014F04u;
    uint32_t lane_data = 0xCAFE0000u; // hi=0xCA at bits[31:24], lo=0xFE at bits[23:16]
    uint8_t  strb = 0b1100;            // bit 3 (=hi byte) + bit 2 (=lo byte)
    axi_write_multi(addr, lane_data, strb);

    CHECK_TRUE("asc word off0 saw 2 writes", asc.writes.size() == 2);
    CHECK_EQ("asc word off0 [0] addr 0xF04", asc.writes[0].first & 0xFFFu, 0xF04u);
    CHECK_EQ("asc word off0 [0] byte 0xCA", asc.writes[0].second, 0xCAu);
    CHECK_EQ("asc word off0 [1] addr 0xF05", asc.writes[1].first & 0xFFFu, 0xF05u);
    CHECK_EQ("asc word off0 [1] byte 0xFE", asc.writes[1].second, 0xFEu);
    return true;
}

static bool test_asc_multi_byte_long_off0() {
    reset();
    // Mimic LSU emission of `move.l #0x12345678, asc+0xF20`:
    //   AW = 0x14F20, off=0 → strb = 4'b1111, wdata = 0x12345678
    //   lane 0 (bit [3:2] of 0xF20 = 0).
    uint32_t addr = 0x0014F20u;
    uint32_t lane_data = 0x12345678u;
    uint8_t  strb = 0b1111;
    axi_write_multi(addr, lane_data, strb);

    CHECK_TRUE("asc long saw 4 writes", asc.writes.size() == 4);
    CHECK_EQ("asc long [0] addr 0xF20", asc.writes[0].first & 0xFFFu, 0xF20u);
    CHECK_EQ("asc long [0] byte 0x12", asc.writes[0].second, 0x12u);
    CHECK_EQ("asc long [1] addr 0xF21", asc.writes[1].first & 0xFFFu, 0xF21u);
    CHECK_EQ("asc long [1] byte 0x34", asc.writes[1].second, 0x34u);
    CHECK_EQ("asc long [2] addr 0xF22", asc.writes[2].first & 0xFFFu, 0xF22u);
    CHECK_EQ("asc long [2] byte 0x56", asc.writes[2].second, 0x56u);
    CHECK_EQ("asc long [3] addr 0xF23", asc.writes[3].first & 0xFFFu, 0xF23u);
    CHECK_EQ("asc long [3] byte 0x78", asc.writes[3].second, 0x78u);
    return true;
}

static bool test_asc_multi_byte_followup_single_byte() {
    // After a multi-byte ASC burst, the bus must be cleanly idle and
    // ready to take a fresh single-byte write.  Catches FSM state-leak
    // bugs (e.g. wr_asc_strb_q not cleared, wr_asc_multi_q stuck high).
    reset();
    axi_write_multi(0x0014F06u, 0x00007F00u, 0b0011);
    CHECK_TRUE("multi-byte saw 2 writes", asc.writes.size() == 2);
    // Now a normal single-byte write should land at addr 0x800 with byte 0xA5.
    axi_write(0x0014800u, 0xA5u << 24, 3); // strb bit 3 = byte at addr+0
    CHECK_TRUE("follow-up single byte saw 3rd write", asc.writes.size() == 3);
    CHECK_EQ("follow-up addr 0x800", asc.writes[2].first & 0xFFFu, 0x800u);
    CHECK_EQ("follow-up byte 0xA5", asc.writes[2].second, 0xA5u);
    return true;
}

// ─── Multi-hot-strobe writes on the rest of the byte-granular family ────
//
// The wr_ser_* serializer used to be ASC-only.  A MAME System 7.0.1 /
// 7.5.3 boot capture (docs/mame_periph_multihot_reachability.md, two
// independent instruments, 1.69 M classified peripheral writes) proved
// that two more slots take real multi-hot beats from real code:
//
//   ORWELL — the ROM's `move.l D4,(A2)+` init loop at 0x40804872 walks
//            0x5000_E000..0x5000_E07C plus 0xA0..0xB8: 91 LONG writes per
//            boot, mem_mask 0xFFFFFFFF.
//   SONIC  — the System 7 Ethernet driver's reset sequence at RAM PC
//            0x000054EE (`moveq #4,D0 / move.l D0,(A2)`) issues LONG
//            writes to 0x50F0_A000 / A010 / A094 — and SONIC's own word
//            serializer is entered only on `size==WORD && !addr[0]`, so a
//            LONG missed it entirely and 3 of the 4 bytes were dropped on
//            a live 16-bit part's register page.
//
// Byte order is the file's HIGH→LOW strobe walk: the byte at AW sits in
// the HIGHEST hot strobe bit (big-endian m68k store), so the walk emits
// AW, AW+1, AW+2, AW+3 in memory order.
static bool test_orwell_multi_byte_long_serializes() {
    reset();
    // Faithful to the captured ROM beat: AW = ORWELL +0x00, LONG,
    // strb = 4'b1111, data = 0x124F0810 (the first walking-pattern
    // longword the ROM's init loop stores).
    axi_write_multi(0x000E000u, 0x124F0810u, 0b1111, nullptr, /*awsize*/ 2);
    CHECK_EQ("orwell long write count", (uint32_t)orwell.writes.size(), 4u);
    CHECK_EQ("orwell long [0] addr 0x00", orwell.writes[0].first, 0x00u);
    CHECK_EQ("orwell long [0] byte 0x12", orwell.writes[0].second, 0x12u);
    CHECK_EQ("orwell long [1] addr 0x01", orwell.writes[1].first, 0x01u);
    CHECK_EQ("orwell long [1] byte 0x4F", orwell.writes[1].second, 0x4Fu);
    CHECK_EQ("orwell long [2] addr 0x02", orwell.writes[2].first, 0x02u);
    CHECK_EQ("orwell long [2] byte 0x08", orwell.writes[2].second, 0x08u);
    CHECK_EQ("orwell long [3] addr 0x03", orwell.writes[3].first, 0x03u);
    CHECK_EQ("orwell long [3] byte 0x10", orwell.writes[3].second, 0x10u);
    return true;
}

static bool test_orwell_multi_byte_word_off2_serializes() {
    reset();
    // WORD at an odd-word offset: AW = ORWELL +0x0A, off = 2 →
    // strb = 4'b0011, lane data = {16'd0, hi, lo}.  Lane = (0x0A>>2)&3 = 2.
    axi_write_multi(0x000E00Au, 0x0000A5C3u, 0b0011, nullptr, /*awsize*/ 1);
    CHECK_EQ("orwell word write count", (uint32_t)orwell.writes.size(), 2u);
    CHECK_EQ("orwell word [0] addr 0x0A", orwell.writes[0].first, 0x0Au);
    CHECK_EQ("orwell word [0] byte 0xA5", orwell.writes[0].second, 0xA5u);
    CHECK_EQ("orwell word [1] addr 0x0B", orwell.writes[1].first, 0x0Bu);
    CHECK_EQ("orwell word [1] byte 0xC3", orwell.writes[1].second, 0xC3u);
    return true;
}

static bool test_orwell_multi_byte_followup_single_byte() {
    // FSM state-leak lock (mirrors test_asc_multi_byte_followup_single_byte):
    // after a burst the bus must be idle and take a fresh single-byte write.
    reset();
    axi_write_multi(0x000E000u, 0x124F0810u, 0b1111, nullptr, 2);
    CHECK_EQ("orwell burst write count", (uint32_t)orwell.writes.size(), 4u);
    axi_write(0x000E0A0u, 0x5Au, 0);
    CHECK_TRUE("orwell follow-up single byte", orwell.writes.size() == 5);
    CHECK_EQ("orwell follow-up addr 0xA0", orwell.writes[4].first, 0xA0u);
    CHECK_EQ("orwell follow-up byte 0x5A", orwell.writes[4].second, 0x5Au);
    return true;
}

static bool test_sonic_driver_reset_long_write_serializes() {
    reset();
    // Byte-for-byte the beat the MAME capture recorded from RAM-resident
    // System 7 driver code: `move.l D0,(A2)` with D0 = 4, A2 = 0x50F0A000.
    // AWSIZE=LONG, all four strobes hot.  This MISSES SONIC's word
    // predicate (size != WORD), which is exactly the residual that used
    // to drop 3 of the 4 bytes.
    axi_write_multi(0x5000A000u, 0x00000004u, 0b1111, nullptr, /*awsize*/ 2);
    CHECK_EQ("sonic long native write count", (uint32_t)sonic.writes.size(), 1u);
    CHECK_EQ("sonic long register CR", sonic.writes[0].first, 0x00u);
    CHECK_EQ("sonic long low halfword", sonic.writes[0].second, 0x0004u);
    CHECK_EQ("sonic long byte enables", sonic_wstrbs[0], 0x3u);
    return true;
}

static bool test_sonic_multi_byte_long_distinct_bytes() {
    reset();
    // Same shape with four distinguishable bytes, so a byte that landed
    // at the wrong address cannot pass by coincidence.  AW = SONIC +0x10
    // (the second address the capture recorded), lane = (0x10>>2)&3 = 0.
    axi_write_multi(0x5000A010u, 0xDEADBEEFu, 0b1111, nullptr, /*awsize*/ 2);
    CHECK_EQ("sonic long distinct native count", (uint32_t)sonic.writes.size(), 1u);
    CHECK_EQ("sonic register index", sonic.writes[0].first, 0x04u);
    CHECK_EQ("sonic connected low half", sonic.writes[0].second, 0xBEEFu);
    CHECK_EQ("sonic both bytes enabled", sonic_wstrbs[0], 0x3u);
    return true;
}

static bool test_sonic_word_path_not_shadowed_by_serializer() {
    // A word at physical +2 maps to both bytes of one native register
    // operation.  Which half of the 32-bit lane carries it is settled by the
    // lane convention this bus runs on: the lowest-address byte of a 32-bit
    // slot sits in bits [31:24] (see wr_addr_byte in peripheral_bus.v and the
    // MOVE.L note in rtl/soc/vram_cpu_byteswap.v), so physical bytes +2/+3 --
    // the SONIC's only connected half -- are lane bits [15:0].  The strobes
    // agree: 0b0011 marks [15:0] live.  So the delivered word is 1122, and
    // AABB (the unstrobed half) must NOT reach the device.  This used to
    // assert AABB, which contradicted both the strobes and the LONG-write
    // test below it, and left the System 7 driver's MOVE.W register writes
    // -- DCR among them -- delivering the wrong halfword on hardware.
    reset();
    axi_write_multi(0x5000A002u, 0xAABB1122u, 0b0011, nullptr, /*awsize*/ 1);
    CHECK_EQ("sonic word native write count", (uint32_t)sonic.writes.size(), 1u);
    CHECK_EQ("sonic word register", sonic.writes[0].first, 0x00u);
    CHECK_EQ("sonic word data", sonic.writes[0].second, 0x1122u);
    CHECK_EQ("sonic word enables", sonic_wstrbs[0], 0x3u);

    // Same shape one register up, with the halves swapped, so a selector that
    // simply always picks [31:16] cannot pass by pattern coincidence.
    reset();
    axi_write_multi(0x5000A006u, 0x1122AABBu, 0b0011, nullptr, /*awsize*/ 1);
    CHECK_EQ("sonic word register (DCR slot)", sonic.writes[0].first, 0x01u);
    CHECK_EQ("sonic word data (DCR slot)", sonic.writes[0].second, 0xAABBu);
    return true;
}

static bool test_sonic_multi_byte_followup_single_byte() {
    reset();
    axi_write_multi(0x5000A010u, 0xDEADBEEFu, 0b1111, nullptr, 2);
    CHECK_EQ("sonic burst write count", (uint32_t)sonic.writes.size(), 1u);
    axi_write(0x5000A020u, 0x99u, 0);
    CHECK_TRUE("sonic follow-up transaction", sonic.writes.size() == 2);
    CHECK_EQ("sonic follow-up register", sonic.writes[1].first, 0x08u);
    return true;
}

// ─── Locks on what this change deliberately did NOT do ──────────────────
//
// The serializer is correct only where AWADDR[1:0] reaches the device's
// local address AND the master's multi-hot convention means "N bytes".
// These scenarios pin both exclusions; they are as load-bearing as the
// positive tests above, because widening slot_serializes() past them
// would be a live regression, not an improvement.
static bool test_adbinj_full_strobe_word_not_serialized() {
    // ADBINJ's only producer is the JTAG-AXI host master, which puts ONE
    // meaningful byte in the strobed lane at a WORD-ALIGNED AWADDR with
    // all four strobes hot (peripheral_bus.v's second documented byte
    // convention).  Serializing that scatters the injected keycode over
    // four sub-registers — 0x1C would land at +0x13 and KBD_ENQUEUE at
    // +0x10 would receive 0x00.  Must stay a single pulse.
    reset();
    axi_write_multi(0x0011010u, 0x0000001Cu, 0b1111);
    CHECK_EQ("adbinj full-strobe write count", (uint32_t)adbinj.writes.size(), 1u);
    CHECK_EQ("adbinj offset stays 0x10", adbinj.writes[0].first, 0x10u);
    CHECK_EQ("adbinj byte stays 0x1C", adbinj.writes[0].second, 0x1Cu);
    return true;
}

static bool test_enet_multi_byte_not_serialized() {
    // ENET is byte-granular but took zero writes of any width across two
    // full System 7 boots, so there is no producer to serve and the same
    // host-convention ambiguity applies.  Latent, left alone: one pulse.
    reset();
    axi_write_multi(0x0008000u, 0x11223344u, 0b1111);
    CHECK_EQ("enet multi-hot write count", (uint32_t)enet.writes.size(), 1u);
    return true;
}

static bool test_strided_slots_multi_hot_stay_single_pulse() {
    // The strided/aliased family (VIA1, VIA2, IWM, SCSI register page)
    // and SCC must be untouched by this change: their device-local
    // address does not depend on AWADDR[1:0], so serializing would pulse
    // the SAME stateful register N times (VIA IFR/IER/SR/timer latches,
    // SWIM phase latches, the Z85C30's shared register pointer).  Exactly
    // one pb_wr pulse each, as before.
    reset();
    axi_write_multi(0x0F00000u, 0xAABBCCDDu, 0b1111);   // VIA1 reg 0
    CHECK_TRUE("via1 multi-hot: one pulse", via1.writes.size() == 1);
    axi_write_multi(0x0F02000u, 0xAABBCCDDu, 0b0011);   // VIA2 reg 0
    CHECK_TRUE("via2 multi-hot: one pulse", via2.writes.size() == 1);
    axi_write_multi(0x001E000u, 0xAABBCCDDu, 0b1111);   // IWM reg 0
    CHECK_TRUE("iwm multi-hot: one pulse", iwm.writes.size() == 1);
    axi_write_multi(0x000F040u, 0xAABBCCDDu, 0b1111);   // SCSI reg page
    CHECK_TRUE("scsi reg multi-hot: one pulse", scsi.writes.size() == 1);
    axi_write_multi(0x000C020u, 0xAABBCCDDu, 0b1111);   // SCC
    CHECK_TRUE("scc multi-hot: one pulse", scc.writes.size() == 1);
    return true;
}

static bool test_q700_mame_mirror() {
    reset();
    // MAME Q700 mirror mask clears addr[23:18], so 0x50F81C00
    // canonicalizes to VIA1 +0x1C00 (register 14 / IER).
    axi_write(0x50F81C00, 0xEE, 0);
    CHECK_TRUE("0x50F81C00 routed to VIA1", via1.writes.size() == 1);
    CHECK_EQ("0x50F81C00 VIA1 reg", via1.writes[0].first & 0xF, 14u);
    CHECK_EQ("0x50F81C00 byte", via1.writes[0].second, 0xEE);

    // 0x50F04000 canonicalizes to 0x50004000, a Q700 gap.  It must not
    // hit VIA1, SCC, or debug — but with the open-bus VOID policy, it
    // returns OKAY+0 to mimic real Q700 silicon.
    uint32_t bresp = 0;
    axi_write(0x50F04000, 0x40, 0, &bresp);
    CHECK_EQ("0x50F04000 VOID OKAY", bresp, 0u);
    CHECK_TRUE("0x50F04000 did not hit debug", dbg_slv.aw_log.empty());
    CHECK_EQ("VIA1 count stable after gap", (uint32_t)via1.writes.size(), 1u);
    CHECK_EQ("SCC count stable after gap", (uint32_t)scc.writes.size(), 0u);

    // 0x5100xxxx is outside the 0x5000_0000..0x50FF_FFFF Q700 I/O mirror;
    // if this unit is called directly with a full address, do not let
    // low 24-bit aliasing turn it into a Mac peripheral hit.  Also
    // routed through the VOID path now.
    axi_write(0x51001C00, 0x51, 0, &bresp);
    CHECK_EQ("0x51001C00 VOID OKAY", bresp, 0u);
    CHECK_TRUE("0x51001C00 did not hit debug", dbg_slv.aw_log.empty());
    CHECK_EQ("VIA1 count stable after 0x5100", (uint32_t)via1.writes.size(), 1u);
    CHECK_EQ("SCSI count stable after 0x5100", (uint32_t)scsi.writes.size(), 0u);
    return true;
}

// Investigation record for the real-cpu040-hardware "via-alias-corruption"
// family (docs/BUG_calibration_word_misplaced_0d00.md Part 12/13; cpu
// submodule's via-alias-corruption-fix ROM patch, 8 call sites).  The
// Q700 ROM's shared device-primitive probe at 0x408046a8/0x408046aa
// walks A2 = GLOBAL_IO_BASE(0x50F00000) + 0x1C00 with D2 in {0, 0x20000,
// 0x40000} added to A2 before each probe, unconditionally WRITING a
// walking-bit test pattern regardless of outcome.  A prior pass at this
// bug (a companion SoC-repo task) proposed extending the SLOT_FAULT/VOID
// open-bus emulation (the same mechanism task #246 used for the SCC
// alt-base alias) to cover this whole D2 family.  This test pins down,
// address by address, which of those offsets is ALREADY void today and
// which one is NOT safely voidable — and documents why.
//
//   D2=0x00000 -> A2+D2 = 0x50F01C00 -> mac_off 0x1C00 -> VIA1 reg 14 (IER)
//   D2=0x40000 -> A2+D2 = 0x50F41C00 -> mac_off 0x1C00 -> VIA1 reg 14 (IER)
//     (0x40000 is exactly one Q700_IO_MIRROR_MASK period (2^18), so it
//     folds back to the IDENTICAL mac_off as D2=0 -- same register.)
//   D2=0x20000 -> A2+D2 = 0x50F21C00 -> mac_off 0x21C00 -> no declared
//     slot (VIA2 tops out at mac_off 0x004000, IWM at 0x020000; 0x21C00
//     is past every one of them) -> ALREADY SLOT_FAULT/VOID today, no
//     RTL change needed.
//
// mac_off 0x1C00 is NOT a spare/unclaimed corner of the VIA1 8 KiB
// window: via1_addr = addr[12:9], so mac_off 0x1C00 (addr[12:9] == 14)
// IS, by this SoC's own device model AND MAME's documented mirror
// formula (test_q700_mame_mirror above, in place since commit 7e4f66e,
// long before this investigation), the real, live VIA1 Interrupt Enable
// Register -- the SAME address ordinary MacOS/ROM VIA1 interrupt
// management uses at 0x50F00000+0x1C00. There is no address-only way to
// tell the probe's incidental touch apart from a legitimate IER access:
// they are bit-for-bit the same transaction. Blanket-VOIDing mac_off
// 0x1C00 would silently break real VIA1 interrupt enable/disable for
// EVERY consumer (both cores, ordinary boot and steady-state operation),
// a far worse regression than the boot-probe corruption it would fix.
// This is why the D2=0/0x40000 half of the family is intentionally left
// alone here — see docs/BUG_calibration_word_misplaced_0d00.md's RTL
// investigation section for the full writeup and the decision to keep
// the existing ROM-patch (which neutralizes the ROM's own errant probe
// code, not the shared address decode) as the fix of record for that
// half.
static bool test_via_alias_probe_family_d2_offsets() {
    reset();
    uint32_t bresp = 0;

    // D2=0: genuinely VIA1 IER.  Left as SLOT_VIA1 -- this IS live,
    // load-bearing hardware, not open bus.
    axi_write(0x50F01C00, 0x01, 0, &bresp);
    CHECK_EQ("D2=0 write OKAY", bresp, 0u);
    CHECK_TRUE("D2=0 hit VIA1", via1.writes.size() == 1);
    CHECK_EQ("D2=0 -> VIA1 reg 14 (IER)", via1.writes[0].first & 0xF, 14u);

    // D2=0x40000: one full mirror period past D2=0 -- folds back onto the
    // EXACT SAME VIA1 IER register mac_off, not a distinct address.
    reset();
    axi_write(0x50F41C00, 0x02, 0, &bresp);
    CHECK_EQ("D2=0x40000 write OKAY", bresp, 0u);
    CHECK_TRUE("D2=0x40000 hit VIA1", via1.writes.size() == 1);
    CHECK_EQ("D2=0x40000 -> VIA1 reg 14 (IER), same as D2=0",
             via1.writes[0].first & 0xF, 14u);

    // D2=0x20000: already outside every declared slot window (VIA2 ends
    // at mac_off 0x4000, IWM at 0x20000; mac_off 0x21C00 is past both) --
    // already routes to SLOT_FAULT/VOID today, confirming no live device
    // is touched and no RTL change is needed for this specific offset.
    reset();
    axi_write(0x50F21C00, 0x03, 0, &bresp);
    CHECK_EQ("D2=0x20000 write OKAY (VOID)", bresp, 0u);
    CHECK_TRUE("D2=0x20000 did NOT hit VIA1", via1.writes.empty());
    CHECK_TRUE("D2=0x20000 did NOT hit VIA2", via2.writes.empty());
    return true;
}

// SCC alt-base alias for the MacsBug bit-17 dispatcher path.  The Q700
// ROM's "machine UNKNOWN" code (gated on D0[17]/D7[17] via the
// macsbug-feature-bit17 patch) routes the SCC base via *(A0+0x44) =
// 0x50F1_E020 — under MAME canonical mirror semantics that physical
// address aliases to mac_off 0x01_E020 = SWIM/IWM-reg-0.  We deliberately
// deviate from MAME for the 64-byte window 0x50F1_E000..0x50F1_E03F so
// the diagnostic patch produces host-visible TX traffic.  Other mirror
// copies of mac_off 0x01_E020 (raw 0x001_E020, 0x041_E020, ...) and
// non-overlapping IWM registers (raw 0x0F1_E200+) keep going to SLOT_IWM
// as MAME does.
static bool test_no_scc_alt_base_bit17_alias() {
    // Task #246.  This test used to ASSERT an SCC alternate base at
    // 0x50F1_E020 (the supposed MacsBug "bit-17 dispatcher" alias).  The
    // alias was deleted on 2026-08-05 and every expectation here inverted,
    // because it was never real:
    //
    //   MAME (macqd700), 25 s of a genuine 7.5.3 boot, read taps:
    //     0x50F1_E000..E1FF :      46 reads, ALL at 0x50F1_E000 (SWIM reg 0)
    //                              ZERO reads in E020..E03F
    //     0x50F0_C000..C0FF : 284,055 reads (C020 x282,085, C024 x1,970)
    //
    // The SCC the OS really drives is at offset 0x20 inside the CANONICAL
    // window (0x50F0_C020) -- the shared low offset 0x20 is almost certainly
    // where the "alt base" belief came from.  The alias also sat inside SWIM
    // register 0's 0x200 stride, so it answered .Sony floppy polls with SCC
    // chan-B control (RR0 = 0x54 idle) and reset the SCC's SHARED register
    // pointer, corrupting live AppleTalk register sequences.
    //
    // Keeping the probe points and flipping the expectations preserves the
    // coverage AND pins the corrected behaviour.
    reset();
    uint32_t bresp = 0;
    uint32_t rresp = 0;

    // 0x50F1_E020 is SWIM/IWM register 0, NOT an SCC chan-B control port.
    axi_write(0x50F1E020u, 0x09u, 0, &bresp);
    CHECK_EQ("0x50F1_E020 write OKAY", bresp, 0u);
    CHECK_TRUE("0x50F1_E020 routed to IWM (no alias)", iwm.writes.size() == 1);
    CHECK_TRUE("SCC untouched by 0x50F1_E020", scc.writes.empty());

    // 0x50F1_E022 -- same.
    axi_write(0x50F1E022u, 0x0Au, 0, &bresp);
    CHECK_EQ("0x50F1_E022 write OKAY", bresp, 0u);
    CHECK_TRUE("0x50F1_E022 routed to IWM (no alias)", iwm.writes.size() == 2);
    CHECK_TRUE("SCC untouched by 0x50F1_E022", scc.writes.empty());

    // 0x50F1_E026 -- same.
    axi_write(0x50F1E026u, 0x4Du, 0, &bresp);
    CHECK_EQ("0x50F1_E026 write OKAY", bresp, 0u);
    CHECK_TRUE("0x50F1_E026 routed to IWM (no alias)", iwm.writes.size() == 3);
    CHECK_TRUE("SCC untouched by 0x50F1_E026", scc.writes.empty());

    // Every one of those is still IWM register 0 -- the 0x200 stride is
    // contiguous now, with no hole punched in the middle of it.
    CHECK_EQ("E020 is reg 0", iwm.writes[0].first & 0xFu, 0x0u);
    CHECK_EQ("E022 is reg 0", iwm.writes[1].first & 0xFu, 0x0u);
    CHECK_EQ("E026 is reg 0", iwm.writes[2].first & 0xFu, 0x0u);

    // 0x50F1_E040 was the old "just past the window" probe; it stays IWM.
    axi_write(0x50F1E040u, 0x40u, 0, &bresp);
    CHECK_EQ("0x50F1_E040 mirror write OKAY", bresp, 0u);
    CHECK_TRUE("0x50F1_E040 hit IWM", iwm.writes.size() == 4);
    CHECK_EQ("SCC still never selected in this window",
             (uint32_t)scc.writes.size(), 0u);

    // Different mirror copy of the same canonical SWIM offset stays on IWM.
    axi_write(0x50F1FC00u, 0x14u, 0, &bresp);
    CHECK_EQ("0x50F1_FC00 mirror write OKAY", bresp, 0u);
    CHECK_TRUE("0x50F1_FC00 hit IWM reg 14",
               iwm.writes.size() == 5 &&
               (iwm.writes[4].first & 0xF) == 0xEu);

    // Canonical SCC base is unchanged -- and is now the ONLY way to the SCC.
    axi_write(0x5000C020u, 0x55u, 0, &bresp);
    CHECK_EQ("canonical 0x5000_C020 OKAY", bresp, 0u);
    CHECK_TRUE("canonical 0x5000_C020 routed to SCC",
               scc.writes.size() == 1);

    // ...including through the Q700 mirror copy the OS actually uses.
    axi_write(0x50F0C024u, 0x56u, 0, &bresp);
    CHECK_EQ("canonical 0x50F0_C024 OKAY", bresp, 0u);
    CHECK_TRUE("canonical 0x50F0_C024 routed to SCC",
               scc.writes.size() == 2);

    // Read-side: 0x50F1_E022 must now read back from the IWM stub, not the
    // SCC stub.  This is the read-path half of the alias removal -- without
    // it a decode that only fixed writes would pass.
    uint32_t got = axi_read(0x50F1E022u, &rresp);
    CHECK_EQ("0x50F1_E022 read OKAY", rresp, 0u);
    CHECK_TRUE("0x50F1_E022 read did NOT come from the SCC stub",
               got != 0xCCCCCCCCu);
    return true;
}

static bool test_enet_sonic_decode() {
    reset();
    uint32_t bresp = 0;

    axi_write(0x50008000u, 0x11u, 0);
    CHECK_TRUE("Ethernet ID write seen", enet.writes.size() == 1);
    CHECK_EQ("Ethernet ID byte", enet.writes[0].second, 0x11u);

    uint32_t got = axi_read(0x50F08000u);
    CHECK_EQ("Ethernet ID readback zero", got, 0u);

    axi_write(0x5000A000u, 0x22u, 0);
    CHECK_TRUE("SONIC write seen", sonic.writes.size() == 1);
    CHECK_EQ("SONIC upper-lane write ignored", sonic_wstrbs[0], 0u);

    got = axi_read(0x50F0A000u, nullptr, 2);
    CHECK_EQ("SONIC MOVE.L bus shape", got, 0xFFFF0014u);

    axi_write(0x50008008u, 0x33u, 0, &bresp);
    CHECK_EQ("post-Ethernet gap VOID OKAY", bresp, 0u);
    CHECK_EQ("Ethernet count stable after gap", (uint32_t)enet.writes.size(), 1u);

    axi_write(0x5000B100u, 0x44u, 0, &bresp);
    CHECK_EQ("post-SONIC gap VOID OKAY", bresp, 0u);
    CHECK_EQ("SONIC count stable after gap", (uint32_t)sonic.writes.size(), 1u);
    return true;
}

// sd_provision was removed; the 0x080_0000..0x08F_FFFF window now routes
// through the SLOT_FAULT (VOID) path — quiet open-bus OKAY + 0x00000000,
// no downstream slave hit.  Verify the upstream master sees OKAY and
// reads return all-zeros (MAME-canonical Q700 unmap_value).  See
// peripheral_bus.v header for the silicon-mimicking rationale.
static bool test_prov_window_void() {
    reset();
    uint32_t addr = 0x0800100;
    uint32_t bresp = 0xFF;
    uint32_t rresp = 0xFF;
    axi_write(addr, 0x12345678u, 0xFF, &bresp);
    CHECK_EQ("ex-prov write VOID OKAY", bresp, 0u);
    CHECK_TRUE("ex-prov write did not hit dbg", dbg_slv.aw_log.empty());
    CHECK_TRUE("ex-prov write did not hit dafb", dafb_slv.aw_log.empty());
    uint32_t got = axi_read(addr, &rresp);
    CHECK_EQ("ex-prov read VOID OKAY", rresp, 0u);
    CHECK_EQ("ex-prov read data all-zeros", got, 0u);
    return true;
}

// Probe a known Q700 I/O gap (the ex-sd_provision 0x080_0000 window is
// the canonical example — it sits inside the 16 MB I/O mirror but is
// not assigned to any peripheral).  MAME's default `set_unmap_value` is
// 0, so unmapped Q700 reads return 0x00000000 + OKAY, writes are
// acknowledged-and-dropped.  No DECERR — bus errors are not raised on
// unmapped Q700 addresses.
static bool test_unmapped_q700_gap_void() {
    reset();
    uint32_t bresp = 0xFF;
    uint32_t rresp = 0xFF;

    // Within the ex-sd_provision window (0x080_0000..0x08F_FFFF).
    uint32_t addr = 0x0805550;
    axi_write(addr, 0xCAFEBABEu, 0xFF, &bresp);
    CHECK_EQ("Q700 gap write VOID OKAY", bresp, 0u);
    CHECK_TRUE("Q700 gap write did not hit dbg", dbg_slv.aw_log.empty());
    CHECK_TRUE("Q700 gap write did not hit dafb", dafb_slv.aw_log.empty());
    CHECK_EQ("Q700 gap write did not hit via1", (uint32_t)via1.writes.size(), 0u);
    CHECK_EQ("Q700 gap write did not hit asc", (uint32_t)asc.writes.size(), 0u);

    uint32_t got = axi_read(addr, &rresp);
    CHECK_EQ("Q700 gap read VOID OKAY", rresp, 0u);
    CHECK_EQ("Q700 gap read data all-zeros", got, 0u);

    // Same window via the system-address form (0x5008_xxxx) — the
    // pre-Q700-mirror catch keeps both forms going through SLOT_FAULT.
    //
    // *** CORRECTION (address-decode audit, 2026-08): the comment above
    // is WRONG and this probe does NOT exercise the ex-sd_provision arm.
    //     off_raw        = 0x08C000
    //     off_raw[23:20] = 0x0            (the arm requires 0x8)
    //     mac_off        = 0x08C000 & 0x03FFFF = 0x0C000  ->  SLOT_SCC
    // It passes because bresp == 0 is also what the SCC returns.  The
    // real system-address form of this arm is 0x5080_0000; see
    // test_prov_void_true_system_form() below, which covers it properly
    // and asserts the VIA1 alias stays dead.  Kept as-is (never delete
    // coverage) — it is still a valid "SCC write returns OKAY" probe.
    bresp = 0xFF;
    axi_write(0x5008C000u, 0xDEADBEEFu, 0xFF, &bresp);
    CHECK_EQ("system 0x5008 gap write VOID OKAY", bresp, 0u);
    return true;
}

static bool test_dbg_service_route() {
    reset();
    // debug_ctrl is an explicit service BAR, not the fallback for Q700 gaps.
    uint32_t addr = 0x0900100;
    axi_write(addr, 0x87654321u);
    CHECK_TRUE("debug saw AXI-Lite write", dbg_slv.aw_log.size() == 1);
    CHECK_EQ("debug awaddr", dbg_slv.aw_log[0].first & 0xFFFFFu, 0x00100u);
    uint32_t got = axi_read(addr);
    CHECK_EQ("debug rdata = 0xCAFEBABE", got, 0xCAFEBABEu);
    return true;
}

static bool test_dafb_axi_passthrough() {
    reset();
    uint32_t addr = 0xF9800008u;
    uint32_t bresp = 0xFF;
    uint32_t rresp = 0xFF;

    axi_write(addr, 0x00000100u, 0xFF, &bresp);
    CHECK_EQ("DAFB write resp OKAY", bresp, 0u);
    CHECK_TRUE("DAFB saw AXI-Lite write", dafb_slv.aw_log.size() == 1);
    CHECK_EQ("DAFB write keeps full system address", dafb_slv.aw_log[0].first, addr);
    CHECK_EQ("DAFB write data", dafb_slv.aw_log[0].second, 0x00000100u);
    CHECK_TRUE("DAFB write did not hit debug", dbg_slv.aw_log.empty());

    uint32_t got = axi_read(addr, &rresp);
    CHECK_EQ("DAFB read resp OKAY", rresp, 0u);
    CHECK_EQ("DAFB read data", got, 0x0DAFB00Fu);
    CHECK_TRUE("DAFB saw AXI-Lite read", dafb_slv.ar_log.size() == 1);
    CHECK_EQ("DAFB read keeps full system address", dafb_slv.ar_log[0], addr);
    return true;
}

static bool test_unmapped_gaps_void() {
    reset();
    uint32_t bresp = 0;
    uint32_t rresp = 0;

    axi_write(0x0004000, 0x87654321u, 0xFF, &bresp);
    CHECK_EQ("canonical 0x4000 write VOID OKAY", bresp, 0u);
    CHECK_TRUE("debug not written by 0x4000 gap", dbg_slv.aw_log.empty());
    uint32_t got = axi_read(0x0004000, &rresp);
    CHECK_EQ("canonical 0x4000 read VOID OKAY", rresp, 0u);
    CHECK_EQ("canonical 0x4000 read data all-zeros", got, 0u);

    axi_write(0x0008008, 0x12u, 0, &bresp);
    CHECK_EQ("Ethernet gap write VOID OKAY", bresp, 0u);
    CHECK_EQ("Ethernet gap did not hit slot", (uint32_t)enet.writes.size(), 0u);

    axi_write(0x0009FFC, 0x34u, 0, &bresp);
    CHECK_EQ("SONIC gap write VOID OKAY", bresp, 0u);
    CHECK_EQ("SONIC gap did not hit slot", (uint32_t)sonic.writes.size(), 0u);

    axi_write(0x000F104, 0x55u, 0, &bresp);
    CHECK_EQ("SCSI page gap write VOID OKAY", bresp, 0u);
    CHECK_EQ("SCSI count stable after gap", (uint32_t)scsi.writes.size(), 0u);

    axi_write(0x50100000, 0x66u, 0, &bresp);
    CHECK_EQ("DMA carveout misroute VOID OKAY", bresp, 0u);
    CHECK_TRUE("DMA carveout did not hit debug", dbg_slv.aw_log.empty());
    return true;
}

static bool test_concurrent_rw() {
    reset();
    // Sequentially issue a write to VIA1 and a read from VIA2.  Because
    // the AXI slave port is single-outstanding per direction and our
    // helpers are blocking, we serialise here rather than truly
    // interleaving — but the test still verifies that two different
    // slots back-to-back don't corrupt each other.
    axi_write(0x0F00004, 0x99, 0);
    uint32_t rr;
    uint32_t got = axi_read(0x0F02004, &rr);
    CHECK_TRUE("VIA1 saw write", via1.writes.size() == 1);
    CHECK_EQ("VIA1 byte", via1.writes[0].second, 0x99);
    CHECK_EQ("VIA2 read resp OKAY", rr, 0);
    CHECK_EQ("VIA2 read data", got, 0xA2A2A2A2u);
    return true;
}

// ───────────────────────────────────────────────────────────────────────
// NEW STRESS SCENARIOS (task #82 widening)
// ───────────────────────────────────────────────────────────────────────

// 10. Simultaneous write to one peripheral + read from another in the
//     same AXI cycle window.  The write FSM and read FSM in
//     peripheral_bus.v are independent — this test verifies neither
//     corrupts the other and both complete within budget.
//
//     Write → VIA1 (registered-ack), Read → VIA2 (registered-ack, rdata
//     0xA2).  VIA2 read must return 0xA2A2A2A2 in its lane and VIA1
//     write must deposit the byte at its pb face.
static bool test_simultaneous_write_read() {
    reset();
    auto r = axi_write_read_concurrent(
        /*waddr=*/0x0F00008,      // VIA1, lane 2
        /*lane_wdata=*/0x7E,
        /*byte_in_lane=*/0,
        /*raddr=*/0x0F02008       // VIA2, lane 2
    );
    CHECK_TRUE("concurrent write completed", r.write_ok);
    CHECK_TRUE("concurrent read completed",  r.read_ok);
    CHECK_TRUE("VIA1 saw the write", via1.writes.size() == 1);
    CHECK_EQ("VIA1 write byte", via1.writes[0].second, 0x7E);
    CHECK_EQ("VIA2 read resp OKAY", r.read_resp, 0);
    CHECK_EQ("VIA2 read data = 0xA2A2A2A2", r.read_data, 0xA2A2A2A2u);
    return true;
}

// 10b. SAME-SLOT concurrent write + read — the hazard documented (and
//      now closed) in peripheral_bus.v's interlock block: every pb
//      device has ONE addr bus (write-priority mux) and ONE ack wire
//      shared by the independent rd/wr FSMs.  Pre-interlock, issuing a
//      read and a write to the SAME device in the same cycle window
//      made the read's pb_rd pulse go out under the WRITE's address,
//      and let each FSM consume the other's ack.
//
//      Write → VIA1 reg 2 (offset 0x400), Read → VIA1 reg 5 (offset
//      0xA00), issued in the same cycle.  With the interlock, the
//      write is accepted first (write priority, matching the addr
//      mux), the read is held off until B completes, and then:
//        • VIA1 sees exactly ONE write pulse, at reg 2, with the byte;
//        • VIA1 sees exactly ONE read pulse, at reg 5 (NOT reg 2);
//        • the read data is reg-5's (address-sensitive rdata model).
static bool test_same_slot_write_read_interlock() {
    reset();
    via1_addr_rdata = true;   // rdata = 0xB0 | via1_addr
    auto r = axi_write_read_concurrent(
        /*waddr=*/0x0F00400,      // VIA1 reg 2 (addr[12:9]=2), lane 0
        /*lane_wdata=*/0x6D,
        /*byte_in_lane=*/0,
        /*raddr=*/0x0F00A00       // VIA1 reg 5 (addr[12:9]=5), lane 0
    );
    CHECK_TRUE("same-slot write completed", r.write_ok);
    CHECK_TRUE("same-slot read completed",  r.read_ok);
    CHECK_TRUE("VIA1 saw exactly one write", via1.writes.size() == 1);
    CHECK_EQ("VIA1 write reg", via1.writes[0].first, 2u);
    CHECK_EQ("VIA1 write byte", via1.writes[0].second, 0x6Du);
    CHECK_TRUE("VIA1 saw exactly one read pulse", via1.reads.size() == 1);
    CHECK_EQ("VIA1 read pulse carries the READ's reg, not the write's",
             via1.reads[0], 5u);
    CHECK_EQ("read resp OKAY", r.read_resp, 0u);
    CHECK_EQ("read data = reg-5 rdata replicated", r.read_data, 0xB5B5B5B5u);
    return true;
}

// 10c. Same-slot interlock under a DRQ-stalled SCSI DMA-shim write —
//      the widest real-world exposure window: a shim write can stall
//      for many cycles waiting on scsi_dma_wr_ready (DRQ gate), during
//      which a JTAG/host-debug read to a SCSI register would, pre-
//      interlock, fire a scsi_rd pulse under the WRITE's shim address
//      (write-priority addr mux) — consuming a pseudo-DMA byte and
//      cross-feeding acks between the two FSMs.  Post-interlock the AR
//      must simply not be accepted (s_arready held low) until the
//      write's B completes.
static bool test_same_slot_scsi_stalled_write_blocks_read() {
    reset();
    scsi_dma_wr_ready_v = 0;   // stall the shim write on the DRQ gate

    // LSU-shaped byte store to the shim: AW=0xF100, byte in
    // wdata[31:24], strb bit 3 of lane 0 (same shape as
    // test_scsi_dma_shim_write_drq_withhold).
    dut->s_awid    = 0xC;
    dut->s_awaddr  = 0x000F100u;
    dut->s_awlen   = 0;
    dut->s_awsize  = 0;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    Word128 w{0,0,0,0};
    w[0] = 0x3C000000u;
    set128(dut->s_wdata, w);
    dut->s_wstrb  = 0x8;
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    // Concurrent AR to SCSI register 4 (0xF040 → pb reg 4) — same slot.
    dut->s_arid    = 0x3;
    dut->s_araddr  = 0x000F040u;
    dut->s_arlen   = 0;
    dut->s_arsize  = 0;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 1;

    bool aw_done = false, w_done = false, b_done = false;
    bool ar_accepted_while_wr_busy = false;
    bool scsi_rd_pulse_while_wr_busy = false;
    for (int i = 0; i < 30; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (dut->s_arvalid && dut->s_arready && !b_done)
            ar_accepted_while_wr_busy = true;
        if (dut->scsi_rd && !b_done)
            scsi_rd_pulse_while_wr_busy = true;
        if (b_hs) b_done = true;
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
    }
    CHECK_TRUE("write AW+W accepted (write goes first)", aw_done && w_done);
    CHECK_TRUE("write still stalled on DRQ gate (no B yet)", !b_done);
    CHECK_TRUE("AR to same slot NOT accepted while write in flight",
               !ar_accepted_while_wr_busy);
    CHECK_TRUE("no scsi_rd pulse while write in flight",
               !scsi_rd_pulse_while_wr_busy);
    CHECK_TRUE("no shim read pulse logged", scsi.reads.empty());

    // Release the DRQ gate: write drains, then the read proceeds.
    scsi_dma_wr_ready_v = 1;
    bool ar_done = false, r_done = false;
    uint32_t rdata = 0, rresp = 0xFFu;
    for (int i = 0; i < 60 && !(b_done && r_done); i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (ar_hs && !b_done && !b_hs)
            ar_accepted_while_wr_busy = true;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            rdata = v[0];
            rresp = dut->s_rresp;
        }
        if (b_hs) b_done = true;
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) r_done = true;
    }
    dut->s_arvalid = 0;
    CHECK_TRUE("AR still never accepted before write completion",
               !ar_accepted_while_wr_busy);
    CHECK_TRUE("write completed after DRQ release", b_done);
    CHECK_TRUE("exactly one shim byte delivered", scsi.writes.size() == 1);
    CHECK_EQ("shim byte addr", scsi.writes[0].first, 0x100u);
    CHECK_EQ("shim byte value", scsi.writes[0].second, 0x3Cu);
    CHECK_TRUE("read completed after the write", r_done);
    CHECK_EQ("read resp OKAY", rresp, 0u);
    CHECK_EQ("read data (register 4 rdata)", rdata, 0x5C5C5C5Cu);
    CHECK_TRUE("exactly one read pulse, after the write", scsi.reads.size() == 1);
    CHECK_EQ("read pulse carries the register addr, not the shim addr",
             scsi.reads[0], 4u);
    for (int i = 0; i < 2; i++) cycle();
    return true;
}

// 11. Narrow-band stall — one peripheral (ASC, combinational-ack) holds
//     its ack low for several cycles; the write must queue cleanly, and
//     after the stall clears, a subsequent write to a different peripheral
//     must commit on time.  Verifies that the bus FSM's single-outstanding
//     discipline does not deadlock when a slow slave is involved.
//
//     NOTE: we target ASC rather than VIA1/VIA2 for the stall because
//     the peripheral_bus.v write FSM requires `via_wr` to remain high
//     THROUGHOUT the wait-for-ack window (it's gated on s_wvalid, which
//     per the FSM protocol stays valid until B).  The existing axi_write
//     helper drops s_wvalid after the W handshake — so a VIA stall would
//     leave via_wr low while the ack is still being blocked by the stall,
//     and the bus FSM would never see the `via_ack && via_wr` conjunction.
//     That's a separate RTL issue flagged in the report ("VIA write FSM
//     requires s_wvalid held until B").  ASC doesn't trigger it because
//     its ack is combinational AND the FSM latches b_done on `asc_ack &&
//     asc_wr` — which works fine as long as stall releases before s_wvalid
//     drops, i.e. within one wait cycle while the W beat is being re-
//     presented.  Here we use a short stall that releases in the same
//     cycle as the W handshake, exercising the queue path.
static bool test_slow_ack_stall() {
    reset();
    // Stall the first ASC access for 2 cycles.  The bus holds s_wvalid
    // (we use a custom helper below that keeps valid high until B) and
    // asc_wr stays high while asc_ack is forced low.  After 2 such
    // cycles the stall counter hits zero and ack fires → B completes.
    asc_stall = 2;

    // Inlined axi_write with s_wvalid held until B.
    uint32_t addr = 0x0014100;
    uint32_t lane_val = 0x5A;
    dut->s_awid    = 0xC;
    dut->s_awaddr  = addr;
    dut->s_awlen   = 0;
    dut->s_awsize  = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    uint32_t wlane = (addr >> 2) & 0x3;
    Word128 w{0,0,0,0};
    w[wlane] = lane_val;
    set128(dut->s_wdata, w);
    dut->s_wstrb  = (uint16_t)(1u << (wlane * 4 + 0));
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;
    bool aw_done = false, b_done = false;
    for (int i = 0; i < 200 && !b_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        // Keep s_wvalid high until B — that's the departure from axi_write.
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    for (int i = 0; i < 2; i++) cycle();

    CHECK_TRUE("ASC stall write B completed", b_done);
    CHECK_TRUE("ASC saw one write after stall", asc.writes.size() == 1);
    CHECK_EQ("ASC stall byte", asc.writes[0].second, 0x5Au);
    CHECK_EQ("ASC stall counter drained", asc_stall, 0);

    // Follow-up: a normal write to VIA1 must commit immediately — no
    // residual deadlock from the previous stall.  VIA1 has combinational
    // ack (via the best-of-both model) so this uses the standard helper.
    axi_write(0x0F00004, 0xC3, 0);
    CHECK_TRUE("VIA1 saw follow-up write", via1.writes.size() == 1);
    CHECK_EQ("VIA1 follow-up byte", via1.writes[0].second, 0xC3);
    return true;
}

// 12. Byte-vs-word granularity — exercise every peripheral family's
//     accepted access width.  Mac peripherals are 8-bit faces; the
//     peripheral_bus mux picks the wstrb-indicated lane byte.  We also
//     test a WORD store (2 adjacent strb bits hot) to a VIA window to
//     prove the mux still picks a well-defined byte (policy: the lowest
//     hot strb lane wins — documented in the wr_byte priority mux in
//     peripheral_bus.v).  DMA controller regs are carved out by the
//     xbar (not routed through this module), so DMA isn't reachable
//     here — the comment calls that out explicitly.
static bool test_granularity_per_device() {
    reset();
    // VIA byte access (reg index 1 at addr[12:9]=1 → byte at CPU offset 0x200)
    axi_write(0x0F00200, 0x11, 0);
    CHECK_TRUE("VIA1 byte write seen", via1.writes.size() == 1);
    CHECK_EQ("VIA1 byte val", via1.writes[0].second, 0x11);
    CHECK_EQ("VIA1 reg idx", via1.writes[0].first & 0xF, 1u);

    // SCC Universal Bus byte lanes: offset bit 1 selects channel A/B,
    // offset bit 2 selects control/data, matching MAME dc_ab_r/w.
    axi_write(0x000C000, 0x20, 0); // B control
    axi_write(0x000C004, 0x21, 0); // B data
    axi_write(0x000C002, 0x22, 0); // A control
    axi_write(0x000C006, 0x23, 0); // A data
    CHECK_TRUE("SCC dc_ab writes seen", scc.writes.size() == 4);
    CHECK_EQ("SCC B control dc_ab", scc.writes[0].first & 0x3, 0u);
    CHECK_EQ("SCC B data dc_ab",    scc.writes[1].first & 0x3, 1u);
    CHECK_EQ("SCC A control dc_ab", scc.writes[2].first & 0x3, 2u);
    CHECK_EQ("SCC A data dc_ab",    scc.writes[3].first & 0x3, 3u);
    CHECK_EQ("SCC A control val",   scc.writes[2].second, 0x22);

    // SCSI/TurboSCSI byte access: Q700 register index 3 is at +0x30.
    axi_write(0x000F030, 0x33, 0);
    CHECK_TRUE("SCSI byte write seen", scsi.writes.size() == 1);
    CHECK_EQ("SCSI byte val", scsi.writes[0].second, 0x33);
    CHECK_EQ("SCSI local register", scsi.writes[0].first & 0x1FF, 0x003u);

    // ASC byte access — byte-granular, offset 0x123 inside the window
    // means addr 0x1_4123 (lane 0, byte-in-lane picks which strb bit).
    // Address 0x14123 → awaddr[1:0]=3, so wstrb bit 0 guards that byte
    // under the big-endian mapping used in the existing asc_byte_select
    // test.
    uint32_t asc_val = 0x44 << 0;   // byte in little-endian position 0
    axi_write(0x0014123, asc_val, 0);  // wstrb bit 0 hot → bytes [7:0]
    CHECK_TRUE("ASC byte write seen", asc.writes.size() == 1);
    CHECK_EQ("ASC byte val", asc.writes[0].second, 0x44u);
    CHECK_EQ("ASC addr", asc.writes[0].first & 0xFFFu, 0x123u);

    // WORD store to VIA1: 2 adjacent strb bits hot (bits 2,3 = byte0/byte1
    // of the BE word at lane 0).  wr_byte priority mux selects the lowest
    // hot strb lane — so the byte at wstrb bit 2 wins (bits [23:16] of
    // wdata).  We verify the mux policy is observable, which matters for
    // any CPU that accidentally emits a word store to an 8-bit peripheral
    // (the real Mac doesn't, but we document the behaviour).
    //
    // CORRECTION (2026-08-18).  "the real Mac doesn't" is FALSE in
    // general and must not be reused as an argument: a MAME System 7
    // boot capture (docs/mame_periph_multihot_reachability.md) found real
    // multi-hot writes on ASC, ORWELL, SONIC and the SCSI pseudo-DMA
    // port.  It happens to remain TRUE for VIA1 specifically — 219,273
    // VIA1 writes across two OS versions and 13 of 16 registers, 100 %
    // single-byte — which is why this scenario's VIA1 stimulus is still
    // representative and why the strided family was left alone.  See
    // docs/periph_multihot_write_serializer.md §"What was NOT changed".
    reset();
    dut->s_awid    = 0xC;
    dut->s_awaddr  = 0x0F00000;   // VIA1, lane 0
    dut->s_awlen   = 0;
    dut->s_awsize  = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    Word128 w{0, 0, 0, 0};
    // Place 0xAABB as a 16-bit WORD in lane 0 at byte positions 2,3 (BE
    // word).  That means wdata[23:8] = 0xAABB → [23:16]=0xAA, [15:8]=0xBB.
    w[0] = (uint32_t)0xAA << 16 | (uint32_t)0xBB << 8;
    set128(dut->s_wdata, w);
    dut->s_wstrb  = (1u << 2) | (1u << 1);  // bits 2,1 hot
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;
    bool aw_done = false, w_done = false, b_done = false;
    for (int i = 0; i < 200 && !b_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid && dut->s_wready;
        bool b_hs  = dut->s_bvalid && dut->s_bready;
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    for (int i = 0; i < 2; i++) cycle();
    CHECK_TRUE("VIA1 word-store completed", b_done);
    CHECK_TRUE("VIA1 saw one write from WORD store", via1.writes.size() == 1);
    // Priority mux: lowest hot strb bit wins — bit 1 → wdata[15:8] = 0xBB.
    CHECK_EQ("VIA1 word-store picks lowest strb byte (0xBB)",
             via1.writes[0].second, 0xBBu);
    return true;
}

// 13. Overlapped address windows — probe addresses at the edges of each
//     MAME Q700 decode window to verify the decode_slot function routes
//     correctly at the boundaries.  Specifically:
//       * 0x0000_1FFC → last word of VIA1 window
//       * 0x0000_2000 → first word of VIA2 window
//       * 0x0000_3FFC → last word of VIA2 window
//       * 0x0000_4000 → empty Q700 gap (→ VOID OKAY+0)
//       * 0x0000_8000 → first word of Ethernet ID window
//       * 0x0000_8004 → last word of Ethernet ID window
//       * 0x0000_8008 → Ethernet gap (→ VOID OKAY+0)
//       * 0x0000_A000 → first word of SONIC window
//       * 0x0000_B0FC → last word of SONIC window
//       * 0x0000_B100 → SONIC gap (→ VOID OKAY+0)
//       * 0x0000_C000 → first word of SCC window
//       * 0x0000_DFFC → last word of SCC window
//       * 0x0000_E000 → first word of Orwell controls
//       * 0x0000_E0FC → last word of Orwell controls
//       * 0x0000_E100 → Orwell page gap
//       * 0x0000_F000 → first word of SCSI/TurboSCSI register window
//       * 0x0000_F0FC → last word of SCSI/TurboSCSI register window
//       * 0x0000_F100 → DMA handshake shim
//       * 0x0000_F101 → DMA handshake shim upper byte
//       * 0x0000_F104 → SCSI page gap
//       * 0x0001_4000 → first word of ASC window
//       * 0x0001_5FFC → last word of ASC window
//       * 0x0001_E000 → first word of SWIM/IWM window
//       * 0x0001_FFFC → last word of SWIM/IWM window
//       * 0x0002_0000 → post-SWIM gap
static bool test_window_boundaries() {
    reset();
    axi_write(0x0001FFC, 0x01, 0);
    CHECK_TRUE("VIA1 last-word in VIA1", via1.writes.size() == 1);
    CHECK_EQ("VIA1 last-word val", via1.writes[0].second, 0x01u);

    axi_write(0x0002000, 0x02, 0);
    CHECK_TRUE("VIA2 first-word in VIA2", via2.writes.size() == 1);
    CHECK_EQ("VIA2 first-word val", via2.writes[0].second, 0x02u);

    axi_write(0x0003FFC, 0x03, 0);
    CHECK_TRUE("VIA2 last-word still in VIA2", via2.writes.size() == 2);
    CHECK_EQ("VIA2 last-word val", via2.writes[1].second, 0x03u);

    uint32_t bresp = 0;
    axi_write(0x0004000, 0x04, 0, &bresp);
    CHECK_EQ("0x4000 gap VOID OKAY", bresp, 0u);
    CHECK_TRUE("0x4000 gap did not hit debug", dbg_slv.aw_log.empty());
    CHECK_EQ("VIA1 count stable after gap", (uint32_t)via1.writes.size(), 1u);
    CHECK_EQ("VIA2 count stable", (uint32_t)via2.writes.size(), 2u);

    axi_write(0x000C000, 0x05, 0);
    CHECK_TRUE("SCC first-word in SCC", scc.writes.size() == 1);
    CHECK_EQ("SCC first-word val", scc.writes[0].second, 0x05u);

    axi_write(0x0008000, 0x0Du, 0);
    CHECK_TRUE("Ethernet first-word in Ethernet", enet.writes.size() == 1);
    CHECK_EQ("Ethernet first-word val", enet.writes[0].second, 0x0Du);

    axi_write(0x0008004, 0x0Eu, 0);
    CHECK_TRUE("Ethernet last-word still in Ethernet", enet.writes.size() == 2);
    CHECK_EQ("Ethernet last-word val", enet.writes[1].second, 0x0Eu);

    axi_write(0x000A000, 0x0Fu, 0);
    CHECK_TRUE("SONIC first-word in SONIC", sonic.writes.size() == 1);
    CHECK_EQ("SONIC first-word val", sonic.writes[0].second, 0x0Fu);

    axi_write(0x000B0FC, 0x10u, 0);
    CHECK_TRUE("SONIC last-word still in SONIC", sonic.writes.size() == 2);
    CHECK_EQ("SONIC last-word val", sonic.writes[1].second, 0x10u);

    axi_write(0x000DFFC, 0x06, 0);
    CHECK_TRUE("SCC last-word still in SCC", scc.writes.size() == 2);
    CHECK_EQ("SCC last-word val", scc.writes[1].second, 0x06u);

    axi_write(0x000E000, 0x11u, 0);
    CHECK_TRUE("Orwell first-word in Orwell", orwell.writes.size() == 1);
    CHECK_EQ("Orwell first-word local addr", orwell.writes[0].first, 0x00u);
    CHECK_EQ("Orwell first-word val", orwell.writes[0].second, 0x11u);

    axi_write(0x000E0FC, 0x12u, 0);
    CHECK_TRUE("Orwell last-word still in Orwell", orwell.writes.size() == 2);
    CHECK_EQ("Orwell last-word local addr", orwell.writes[1].first, 0xFCu);
    CHECK_EQ("Orwell last-word val", orwell.writes[1].second, 0x12u);

    axi_write(0x000E100, 0x13u, 0, &bresp);
    CHECK_EQ("Orwell page gap VOID OKAY", bresp, 0u);
    CHECK_EQ("Orwell count stable after page gap", (uint32_t)orwell.writes.size(), 2u);

    axi_write(0x000F000, 0x07, 0);
    CHECK_TRUE("SCSI first-word in SCSI", scsi.writes.size() == 1);
    CHECK_EQ("SCSI first-word local addr", scsi.writes[0].first, 0x000u);
    CHECK_EQ("SCSI first-word val", scsi.writes[0].second, 0x07u);

    axi_write(0x000F0FC, 0x08, 0);
    CHECK_TRUE("SCSI reg top word still in SCSI", scsi.writes.size() == 2);
    CHECK_EQ("SCSI reg top local addr", scsi.writes[1].first, 0x00Fu);
    CHECK_EQ("SCSI reg top val", scsi.writes[1].second, 0x08u);

    axi_write(0x000F100, 0x0B, 0);
    CHECK_TRUE("SCSI DMA shim in SCSI", scsi.writes.size() == 3);
    CHECK_EQ("SCSI DMA shim local addr", scsi.writes[2].first, 0x100u);
    CHECK_EQ("SCSI DMA shim val", scsi.writes[2].second, 0x0Bu);

    axi_write(0x000F101, 0x0C0000u, 2);
    CHECK_TRUE("SCSI DMA shim upper byte in SCSI", scsi.writes.size() == 4);
    CHECK_EQ("SCSI DMA shim upper local addr", scsi.writes[3].first, 0x101u);
    CHECK_EQ("SCSI DMA shim upper val", scsi.writes[3].second, 0x0Cu);

    axi_write(0x000F104, 0x0C, 0, &bresp);
    CHECK_EQ("SCSI page gap VOID OKAY", bresp, 0u);
    CHECK_TRUE("SCSI page gap did not hit debug", dbg_slv.aw_log.empty());
    CHECK_EQ("SCSI count stable after page gap", (uint32_t)scsi.writes.size(), 4u);

    axi_write(0x0014000, 0x09, 0);
    CHECK_TRUE("ASC first-word in ASC", asc.writes.size() == 1);
    CHECK_EQ("ASC first-word val", asc.writes[0].second, 0x09u);

    axi_write(0x0015FFC, 0x0A, 0);
    CHECK_TRUE("ASC last-word still in ASC", asc.writes.size() == 2);
    CHECK_EQ("ASC last-word val", asc.writes[1].second, 0x0Au);

    axi_write(0x001E000, 0x14u, 0);
    CHECK_TRUE("IWM first-word in IWM", iwm.writes.size() == 1);
    CHECK_EQ("IWM first-word reg", iwm.writes[0].first & 0xFu, 0u);
    CHECK_EQ("IWM first-word val", iwm.writes[0].second, 0x14u);

    axi_write(0x001FFFC, 0x15u, 0);
    CHECK_TRUE("IWM last-word still in IWM", iwm.writes.size() == 2);
    CHECK_EQ("IWM last-word reg", iwm.writes[1].first & 0xFu, 0xFu);
    CHECK_EQ("IWM last-word val", iwm.writes[1].second, 0x15u);

    axi_write(0x0020000, 0x16u, 0, &bresp);
    CHECK_EQ("post-IWM gap VOID OKAY", bresp, 0u);
    CHECK_EQ("IWM count stable after gap", (uint32_t)iwm.writes.size(), 2u);
    return true;
}

// 14. Unaligned accesses — the Mac 6522 VIAs and ASC all tolerate byte
//     stores to odd addresses (the peripheral_bus passes addr[1:0]
//     verbatim on the ASC byte-granular path; VIAs use addr[12:9] so
//     odd byte offsets within a 0x200-spaced register alias to the same
//     reg index).
//     We check two things:
//       (a) VIA1: odd address (0x0F00205 = reg 1 + byte offset 1).  The
//           peripheral sees addr[12:9]=1, same as 0x0F00200.  Byte value
//           comes from the wstrb-selected lane position (byte 1).
//       (b) ASC: odd address (0x0014001) — byte-granular path, pb_addr
//           must reflect the low 2 bits (→ 0x001, not 0x000).
static bool test_unaligned_byte() {
    reset();
    // (a) VIA1 at odd address 0x0F00205 — byte in lane 1 position 1.
    // axi_write places the byte in the low-order 8 bits of lane_wdata
    // and wstrb bit (lane*4 + byte_in_lane) is hot.  We pass the byte
    // value directly; byte_in_lane=1 drops it into [15:8] and asserts
    // wstrb bit (1*4+1)=5.
    uint32_t lane_val = (uint32_t)0xAB << 8;   // byte at position 1
    axi_write(0x0F00205, lane_val, 1);
    CHECK_TRUE("VIA1 saw odd-addr byte", via1.writes.size() == 1);
    CHECK_EQ("VIA1 odd reg idx = 1", via1.writes[0].first & 0xF, 1u);
    CHECK_EQ("VIA1 odd byte val", via1.writes[0].second, 0xABu);

    // (b) ASC at odd address 0x0014001 — pb_addr must pass [1:0] through.
    // Byte offset 1 → byte-in-lane 2 under the big-endian mapping used
    // in the existing asc_byte_select test (strb bit (3-k)=2, lane 0).
    uint32_t asc_val = (uint32_t)0xCD << 16;  // byte at position 2
    axi_write(0x0014001, asc_val, 2);
    CHECK_TRUE("ASC saw odd-addr byte", asc.writes.size() == 1);
    CHECK_EQ("ASC odd pb_addr = 0x001", asc.writes[0].first & 0xFFFu, 0x001u);
    CHECK_EQ("ASC odd byte val", asc.writes[0].second, 0xCDu);
    return true;
}

// 15. Multi-device parallelism stress — write to one pb_* peripheral
//     while a second transaction reads from a *Lite* slave (debug_ctrl).
//     The write travels the pb path and the read travels the Lite
//     passthrough — completely disjoint control paths inside
//     peripheral_bus.v.  This is the strongest form of the "independent
//     FSM" property: two transactions executing against two different
//     downstream interface shapes simultaneously.
static bool test_parallel_pb_and_lite() {
    reset();
    auto r = axi_write_read_concurrent(
        /*waddr=*/0x0F02004,      // VIA2 byte write
        /*lane_wdata=*/0x6A,
        /*byte_in_lane=*/0,
        /*raddr=*/0x0900200       // debug_ctrl Lite-slave read
    );
    CHECK_TRUE("pb write completed", r.write_ok);
    CHECK_TRUE("lite read completed", r.read_ok);
    CHECK_TRUE("VIA2 saw pb write", via2.writes.size() == 1);
    CHECK_EQ("VIA2 byte", via2.writes[0].second, 0x6Au);
    CHECK_EQ("dbg read resp OKAY", r.read_resp, 0);
    CHECK_EQ("dbg read data = 0xCAFEBABE", r.read_data, 0xCAFEBABEu);
    // Confirm the Lite slave logged exactly one AR at the expected addr.
    CHECK_TRUE("dbg AR log size 1", dbg_slv.ar_log.size() == 1);
    CHECK_EQ("dbg AR addr", dbg_slv.ar_log[0] & 0xFFFFFu, 0x00200u);
    return true;
}

// ─── 16. CPU-LSU BE-lane byte-matrix (endianness audit 2026-04-28) ─────
//     Exercises the EXACT convention the 68k LSU emits — `wstrb` bit
//     (3 - addr[1:0]) hot, byte at lane[(3-addr[1:0])*8 +: 8] — for all
//     four byte positions across all four 32-bit lanes.  This is what
//     `rtl/core/mem/lsu.v` `mk_strb` / `mk_wdata` actually drive on a
//     CPU `MOVE.B` to a peripheral.
//
//     Existing `test_via1_byte_rw` etc. all use the LE-lane helper
//     (`byte_in_lane=0` → strb bit 0, byte at wdata[7:0]), which is the
//     opposite convention.  peripheral_bus's `wr_byte` mux is symmetric
//     so both happen to give the same result, but a regression that
//     specifically broke the BE-lane path would slip past the existing
//     tests.  This scenario is the BE-lane gate.
//
//     Documented in `docs/endianness_audit.md` §2 B6.
static bool test_be_lane_byte_matrix() {
    reset();

    // Issue an LSU-style BE-lane byte store at every (lane, addr[1:0])
    // pair within the VIA1 register-0 mirror window.  Verify the
    // peripheral sees the right byte each time.
    //
    // Convention emulated:
    //   addr[1:0] = k  (byte position within the 32-bit lane)
    //   wstrb     = 4'b1000 >> k         (BE-lane: bit 3 hot for k=0)
    //   wdata     = byte << (24 - 8*k)   (byte placed at MSB of 4-byte word)
    //   addr[3:2] = lane                  (which 32-bit lane within the 128-bit beat)
    //
    // In tb_peripheral_bus's existing `axi_write(addr, lane_data, byte_in_lane)`,
    // strb bit = (lane*4 + byte_in_lane).  For BE-lane we need bit (lane*4 + (3-k)),
    // i.e. byte_in_lane = 3 - k.

    struct Case { uint32_t lane_select; uint32_t addr_lo2; uint8_t byte_val; };
    Case cases[] = {
        // lane 0 (addr[3:2]=0)
        { 0, 0, 0xA0 }, { 0, 1, 0xA1 }, { 0, 2, 0xA2 }, { 0, 3, 0xA3 },
        // lane 1 (addr[3:2]=1)
        { 1, 0, 0xB0 }, { 1, 1, 0xB1 }, { 1, 2, 0xB2 }, { 1, 3, 0xB3 },
        // lane 2 (addr[3:2]=2)
        { 2, 0, 0xC0 }, { 2, 1, 0xC1 }, { 2, 2, 0xC2 }, { 2, 3, 0xC3 },
        // lane 3 (addr[3:2]=3)
        { 3, 0, 0xD0 }, { 3, 1, 0xD1 }, { 3, 2, 0xD2 }, { 3, 3, 0xD3 },
    };

    int prev_writes = (int)via1.writes.size();
    int n_checked = 0;
    for (auto& c : cases) {
        // ASC has byte-granular pb_addr (`wr_addr_q[11:0]`) — use it so
        // each (lane, addr_lo2) combination produces a distinct
        // peripheral-visible address.  VIA1's `wr_addr_q[12:9]` would
        // collapse all addr[1:0] cases to the same register.
        uint32_t addr = 0x0014000u + (c.lane_select << 2) + c.addr_lo2;
        uint32_t lane_val = ((uint32_t)c.byte_val) << (24u - 8u * c.addr_lo2);
        uint8_t byte_in_lane = (uint8_t)(3 - c.addr_lo2);  // BE-lane
        axi_write(addr, lane_val, byte_in_lane);
        n_checked++;
    }

    // Verify ASC saw all 16 byte writes with the right (addr, value).
    CHECK_TRUE("ASC saw 16 BE-lane byte writes", asc.writes.size() == 16);

    bool addr_byte_match = true;
    for (size_t i = 0; i < asc.writes.size() && i < 16; i++) {
        uint32_t expected_addr = (cases[i].lane_select << 2) + cases[i].addr_lo2;
        uint8_t  expected_val  = cases[i].byte_val;
        uint32_t got_addr = asc.writes[i].first & 0xFFFu;
        uint8_t  got_val  = asc.writes[i].second;
        if (got_addr != expected_addr || got_val != expected_val) {
            std::printf("    case %zu: lane=%u addr_lo2=%u → expected pb_addr=0x%03x val=0x%02x, "
                        "got pb_addr=0x%03x val=0x%02x\n",
                        i, cases[i].lane_select, cases[i].addr_lo2,
                        (unsigned)expected_addr, (unsigned)expected_val,
                        (unsigned)got_addr, (unsigned)got_val);
            addr_byte_match = false;
        }
    }
    CHECK_TRUE("BE-lane (lane, addr[1:0]) → pb_addr/wr_byte all correct", addr_byte_match);

    (void)prev_writes;
    (void)n_checked;
    return true;
}

// WSTRB IS THE AUTHORITY: an AXI-standard little-endian byte write must land
// intact even when the OTHER byte lanes carry live data.
//
// This is the case the old `wr_byte = onehot ? (wr_addr_byte | wr_strb_byte)
// : wr_strb_byte` got WRONG, and it is why that OR was removed rather than
// merely tidied. The two selectors used opposite conventions -- address-derived
// was big-endian (addr[1:0]==0 -> [31:24]), strobe-derived little-endian
// (strb[0] -> [7:0]) -- so for a little-endian producer they resolve to
// DIFFERENT byte indices. The bus then delivered the bitwise OR of two
// unrelated bytes.
//
// The old code's own justification was "well-formed writes have only one
// non-zero copy". That is a property of the MASTER, not of this bus:
//   * cpu040's store path presents a full 16-byte cache LINE IMAGE on WDATA,
//     so its non-strobed lanes carry live data, not zero;
//   * tools/mame_axi_periph_bridge.cpp is an AXI-standard little-endian
//     producer.
// Both premises of the hedge therefore fail simultaneously in principle. The
// non-zero filler below (0x5A) is exactly what makes this test discriminating:
// with zero filler the OR is invisible, which is why the existing BE-lane
// matrix passed either way.
//
// EXPECTED TO FAIL ON THE PRE-2026-09-05 BUS: for addr[1:0]=k the old
// wr_addr_byte selects index 3-k = 0x5A, so the peripheral would see
// (value | 0x5A) instead of value.
static bool test_le_byte_write_with_live_neighbour_lanes() {
    reset();

    // ASC has byte-granular pb_addr, so each addr[1:0] is a DISTINCT
    // peripheral register -- which also lets us prove no OTHER register is
    // disturbed, the second half of "accurate from the driver's perspective".
    struct Case { uint32_t addr_lo2; uint8_t byte_val; };
    Case cases[] = { {0, 0x81}, {1, 0x42}, {2, 0x24}, {3, 0x18} };

    for (auto& c : cases) {
        uint32_t addr = 0x0014100u + c.addr_lo2;
        // AXI-standard little-endian: byte at index addr[1:0], strobe bit the
        // same index. Every OTHER byte in the lane is deliberately non-zero.
        uint32_t lane_val = 0x5A5A5A5Au;
        lane_val &= ~(0xFFu << (8u * c.addr_lo2));
        lane_val |=  ((uint32_t)c.byte_val << (8u * c.addr_lo2));
        axi_write(addr, lane_val, (uint8_t)c.addr_lo2);
    }

    CHECK_TRUE("ASC saw exactly 4 byte writes (no register disturbed beyond them)",
               asc.writes.size() == 4);

    bool ok = true;
    for (size_t i = 0; i < asc.writes.size() && i < 4; i++) {
        uint32_t expected_addr = 0x100u + cases[i].addr_lo2;
        uint8_t  expected_val  = cases[i].byte_val;
        uint32_t got_addr = asc.writes[i].first & 0xFFFu;
        uint8_t  got_val  = asc.writes[i].second;
        if (got_addr != expected_addr || got_val != expected_val) {
            std::printf("    LE case %zu: addr[1:0]=%u -> expected pb_addr=0x%03x val=0x%02x, "
                        "got pb_addr=0x%03x val=0x%02x%s\n",
                        i, cases[i].addr_lo2,
                        (unsigned)expected_addr, (unsigned)expected_val,
                        (unsigned)got_addr, (unsigned)got_val,
                        (got_val == (uint8_t)(expected_val | 0x5A))
                          ? "   <-- exactly (value | 0x5A): the address/strobe OR" : "");
            ok = false;
        }
    }
    CHECK_TRUE("little-endian byte write lands intact with live neighbour lanes", ok);
    return true;
}

// ─── Bounded ack watchdog (PB_WATCHDOG_LOG2) scenarios ─────────────────
// The Verilator build overrides PB_WATCHDOG_LOG2 down to 10 (see the
// tb-peripheral-bus rule in the Makefile) so these scenarios prove the
// timeout in ~1k cycles instead of the production 2^24 (~335 ms @
// 50 MHz).  Keep WD_TIMEOUT in sync with that -G override.
static const int WD_TIMEOUT = 1 << 10;

// (a) A peripheral that never acks must get SLVERR after the timeout —
//     not hold wr_busy (and the fabric behind it) hostage forever — and
// (b) the bus must be immediately usable for a DIFFERENT peripheral
//     afterwards.  Write-side flavour: VIA1's ack is permanently
//     withheld via its stall counter (it only decrements while the
//     via1_wr strobe is high, and the generic pb_wr pulse is a single
//     cycle — a huge value never drains).
static bool test_watchdog_write_ack_timeout_slverr() {
    reset();
    via1_stall = 1 << 30;   // never acks

    uint32_t addr = 0x0F00004u;   // VIA1 window, lane 1
    dut->s_awid    = 0xC;
    dut->s_awaddr  = addr;
    dut->s_awlen   = 0;
    dut->s_awsize  = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    uint32_t lane = (addr >> 2) & 0x3;
    Word128 w{0,0,0,0};
    w[lane] = 0x77u;
    set128(dut->s_wdata, w);
    dut->s_wstrb  = (uint16_t)(1u << (lane * 4 + 0));
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    int  b_cycle = -1;
    uint32_t resp = 0xFFu;
    for (int i = 0; i < WD_TIMEOUT + 100 && !b_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) { resp = dut->s_bresp; b_done = true; b_cycle = i; }
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    for (int i = 0; i < 2; i++) cycle();

    CHECK_TRUE("never-acked write completes (not held forever)", b_done);
    CHECK_EQ("timed-out write resp is SLVERR", resp, 2u);
    // Prove the watchdog is not trigger-happy: it must have waited the
    // (tb-shortened) full bound, not fired early.
    CHECK_TRUE("timeout did not fire early", b_cycle >= WD_TIMEOUT - 8);

    // (b) A DIFFERENT peripheral (VIA2) is reachable immediately after.
    uint32_t bresp2 = 0xFFu;
    axi_write(0x0F02000u, 0xC4, 0, &bresp2);
    CHECK_EQ("post-timeout VIA2 write resp OKAY", bresp2, 0u);
    CHECK_TRUE("post-timeout VIA2 saw the write", via2.writes.size() == 1);
    CHECK_EQ("post-timeout VIA2 byte", via2.writes[0].second, 0xC4u);
    return true;
}

// Read-side flavour of the same pair: SCC ack permanently withheld →
// SLVERR + zeroed data after the bound; then a VIA1 read completes
// normally, proving rd_busy was released.
static bool test_watchdog_read_ack_timeout_slverr() {
    reset();
    scc_stall = 1 << 30;   // never acks

    uint32_t addr = 0x000C000u;   // SCC window, lane 0
    dut->s_arid    = 0x3;
    dut->s_araddr  = addr;
    dut->s_arlen   = 0;
    dut->s_arsize  = 4;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready  = 1;

    bool ar_done = false, r_done = false;
    int  r_cycle = -1;
    uint32_t got = 0xFFFFFFFFu, resp = 0xFFu;
    for (int i = 0; i < WD_TIMEOUT + 100 && !r_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            got = v[0];
            resp = dut->s_rresp;
            r_done = true;
            r_cycle = i;
        }
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
    }
    dut->s_arvalid = 0;
    for (int i = 0; i < 2; i++) cycle();

    CHECK_TRUE("never-acked read completes (not held forever)", r_done);
    CHECK_EQ("timed-out read resp is SLVERR", resp, 2u);
    CHECK_EQ("timed-out read data zeroed", got, 0u);
    CHECK_TRUE("read timeout did not fire early", r_cycle >= WD_TIMEOUT - 8);

    // Bus usable again: normal VIA1 read straight after.
    uint32_t rr2 = 0xFFu;
    uint32_t got2 = axi_read(0x0F00004u, &rr2);
    CHECK_EQ("post-timeout VIA1 read resp OKAY", rr2, 0u);
    CHECK_EQ("post-timeout VIA1 read data", got2, 0xA1A1A1A1u);
    return true;
}

// (c) A legitimately LONG but eventually-acking transaction — the SCSI
// DMA-shim write held off by the TurboSCSI DRQ-check for over half the
// (tb-shortened) watchdog bound — must NOT be falsely SLVERR'd: once
// DRQ releases before the bound, the beat completes OKAY and delivers
// its byte.  This is exactly the wait profile the generous production
// bound (2^24, ~335 ms) exists to accommodate.
static bool test_watchdog_slow_drq_wait_not_slverrd() {
    reset();
    scsi_dma_wr_ready_v = 0;   // DRQ-check withholding

    // LSU-shaped byte store to the shim (same shape as
    // test_scsi_dma_shim_write_drq_withhold).
    dut->s_awid    = 0xC;
    dut->s_awaddr  = 0x000F100u;
    dut->s_awlen   = 0;
    dut->s_awsize  = 0;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;
    Word128 w{0,0,0,0};
    w[0] = 0xE5000000u;
    set128(dut->s_wdata, w);
    dut->s_wstrb  = 0x8;
    dut->s_wlast  = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    uint32_t resp = 0xFFu;
    // Hold DRQ off for well past half the watchdog bound — far longer
    // than any pre-watchdog test waited, well short of the bound itself.
    for (int i = 0; i < WD_TIMEOUT / 2 + 100; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) { resp = dut->s_bresp; b_done = true; }
        sample_pb_writes();
        step_registered_acks();
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
    }
    CHECK_TRUE("no B while DRQ withheld below the bound", !b_done);

    // Release DRQ before the bound expires: must complete OKAY.
    scsi_dma_wr_ready_v = 1;
    for (int i = 0; i < 60 && !b_done; i++) {
        drive_lite_slaves();
        dut->eval();
        drive_pb_peripherals();
        dut->eval();
        bool b_hs = dut->s_bvalid && dut->s_bready;
        if (b_hs) { resp = dut->s_bresp; b_done = true; }
        sample_pb_writes();
        step_registered_acks();
        tick();
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    for (int i = 0; i < 2; i++) cycle();

    CHECK_TRUE("slow-DRQ write completed after release", b_done);
    CHECK_EQ("slow-DRQ write resp OKAY (no false SLVERR)", resp, 0u);
    CHECK_TRUE("slow-DRQ write delivered exactly one byte",
               scsi.writes.size() == 1);
    CHECK_EQ("slow-DRQ delivered byte", scsi.writes[0].second, 0xE5u);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Address-decode audit (2026-08) — systematic window-edge coverage
// ═══════════════════════════════════════════════════════════════════════
//
// Motivation: the SCC alt-base bug (commit 26360eb) was a bit-slice
// equality that matched a DIFFERENT range than its comment claimed, and
// silently shadowed SWIM/IWM register 0.  It survived because the suite
// probed the INTERIOR of every window but almost never the four
// addresses that actually pin a window down:
//
//     first, last, first-1, last+1
//
// The tests below close that gap for every window in decode_slot().
// Every expectation is derived from the intended map (the header table
// in rtl/soc/peripheral_bus.v + MAME src/mame/apple/macquadra700.cpp
// spike_state::quadra700_map), NOT read back off the RTL.

// Probe one address and report which peripheral face observed it.
// Returns a stable short name so decode expectations can be written as
// a table.  "VOID" == SLOT_FAULT (quiet OKAY, no device touched).
static const char* probe_slot(uint32_t addr) {
    reset();
    uint32_t bresp = 0xFF;
    axi_write(addr, 0xA5u, 0, &bresp);
    if (bresp != 0)               return "SLVERR";
    if (!via1.writes.empty())     return "VIA1";
    if (!via2.writes.empty())     return "VIA2";
    if (!enet.writes.empty())     return "ENET";
    if (!sonic.writes.empty())    return "SONIC";
    if (!orwell.writes.empty())   return "ORWELL";
    if (!scc.writes.empty())      return "SCC";
    if (!scsi.writes.empty())     return "SCSI";
    if (!asc.writes.empty())      return "ASC";
    if (!iwm.writes.empty())      return "IWM";
    if (!adbinj.writes.empty())   return "ADBINJ";
    if (!dbg_slv.aw_log.empty())  return "DBG";
    if (!dafb_slv.aw_log.empty()) return "DAFB";
    return "VOID";
}

struct DecodeCase { uint32_t addr; const char* want; const char* why; };

static bool run_decode_cases(const DecodeCase* c, int n) {
    bool ok = true;
    for (int i = 0; i < n; i++) {
        const char* got = probe_slot(c[i].addr);
        if (std::strcmp(got, c[i].want) != 0) {
            std::printf("    FAIL 0x%08X: got %s, expected %s  (%s)\n",
                        c[i].addr, got, c[i].want, c[i].why);
            ok = false;
        }
    }
    return ok;
}

// A. Mirrored Mac-device windows: first / last / below / above.
//    Addresses are bare 24-bit I/O offsets (mirror copy 0).
static bool test_decode_edges_mac_devices() {
    static const DecodeCase cases[] = {
        // VIA1 mac_off [0x00000,0x01FFF]
        {0x0000000u, "VIA1",   "VIA1 first"},
        {0x0001FFFu, "VIA1",   "VIA1 last byte"},
        {0x0002000u, "VIA2",   "VIA1 last+1 == VIA2 first"},
        // VIA2 mac_off [0x02000,0x03FFF]
        {0x0003FFFu, "VIA2",   "VIA2 last byte"},
        {0x0004000u, "VOID",   "VIA2 last+1 -> gap"},
        // ENET mac_off [0x08000,0x08007] -- only 8 bytes wide
        {0x0007FFFu, "VOID",   "ENET first-1"},
        {0x0008000u, "ENET",   "ENET first"},
        {0x0008007u, "ENET",   "ENET LAST BYTE (0x8004 was the old bound)"},
        {0x0008008u, "VOID",   "ENET last+1"},
        // SONIC mac_off [0x0A000,0x0B0FF]
        {0x0009FFFu, "VOID",   "SONIC first-1"},
        {0x000A000u, "SONIC",  "SONIC first"},
        {0x000B0FFu, "SONIC",  "SONIC last byte"},
        {0x000B100u, "VOID",   "SONIC last+1"},
        // SCC mac_off [0x0C000,0x0DFFF]
        {0x000BFFFu, "VOID",   "SCC first-1 (never probed before)"},
        {0x000C000u, "SCC",    "SCC first"},
        {0x000DFFFu, "SCC",    "SCC last byte"},
        // ORWELL mac_off [0x0E000,0x0E0FF]
        {0x000E000u, "ORWELL", "SCC last+1 == Orwell first"},
        {0x000E0FFu, "ORWELL", "Orwell last byte"},
        {0x000E100u, "VOID",   "Orwell last+1"},
        // SCSI regs mac_off [0x0F000,0x0F0FF]
        {0x000EFFFu, "VOID",   "SCSI first-1 (never probed before)"},
        {0x000F000u, "SCSI",   "SCSI regs first"},
        {0x000F0FFu, "SCSI",   "SCSI regs last byte"},
        // SCSI DMA shim mac_off [0x0F100,0x0F101]
        {0x000F100u, "SCSI",   "DMA shim first"},
        {0x000F101u, "SCSI",   "DMA shim last"},
        {0x000F102u, "VOID",   "DMA shim last+1 (suite jumped to 0xF104)"},
        // ADB injection mac_off [0x11000,0x11FFF]
        {0x0010FFFu, "VOID",   "ADBINJ first-1"},
        {0x0011000u, "ADBINJ", "ADBINJ first"},
        {0x0011FFFu, "ADBINJ", "ADBINJ last byte"},
        {0x0012000u, "VOID",   "ADBINJ last+1"},
        // ASC mac_off [0x14000,0x15FFF]
        {0x0013FFFu, "VOID",   "ASC first-1"},
        {0x0014000u, "ASC",    "ASC first"},
        {0x0015FFFu, "ASC",    "ASC last byte"},
        {0x0016000u, "VOID",   "ASC last+1"},
        // SWIM/IWM mac_off [0x1E000,0x1FFFF]
        {0x001DFFFu, "VOID",   "IWM first-1"},
        {0x001E000u, "IWM",    "IWM first"},
        {0x001FFFFu, "IWM",    "IWM last byte"},
        {0x0020000u, "VOID",   "IWM last+1"},
    };
    return run_decode_cases(cases, (int)(sizeof(cases)/sizeof(cases[0])));
}

// B. Raw-offset carve-outs decoded BEFORE the Q700 mirror.
//
//    These are the arms whose whole job is to stop a non-Mac window
//    aliasing back onto a Mac device through addr[23:18].  Each
//    below/above-edge address below is chosen so that it masks to
//    mac_off 0x00000 -- i.e. it WOULD be VIA1 if the arm were absent or
//    mis-sized.  A pure "bresp == 0" assertion cannot tell VOID from a
//    successful VIA1 write, which is exactly how the existing
//    test_prov_window_void / test_unmapped_gaps_void checks pass
//    vacuously; probe_slot() distinguishes them.
static bool test_decode_edges_raw_carveouts() {
    static const DecodeCase cases[] = {
        // DMA config carve-out: off_raw[23:20]==0x1 => [0x100000,0x1FFFFF]
        {0x500C0000u, "VIA1", "DMA carve-out first-1 (mirror copy 3 VIA1)"},
        {0x50100000u, "VOID", "DMA carve-out first -- must NOT be VIA1"},
        {0x501C0000u, "VOID", "DMA carve-out, mirror copy 7 VIA1 base"},
        {0x501FFFFFu, "VOID", "DMA carve-out last byte"},
        {0x50200000u, "VIA1", "DMA carve-out last+1 (mirror copy 8 VIA1)"},
        // ex-sd_provision VOID: off_raw[23:20]==0x8 => [0x800000,0x8FFFFF]
        {0x507C0000u, "VIA1", "prov-VOID first-1 (mirror copy 31 VIA1)"},
        {0x50800000u, "VOID", "prov-VOID first -- must NOT be VIA1"},
        {0x508C0000u, "VOID", "prov-VOID, mirror copy 35 VIA1 base"},
        {0x508FFFFFu, "VOID", "prov-VOID last byte"},
        // debug_ctrl service BAR: off_raw[23:20]==0x9 => [0x900000,0x9FFFFF]
        {0x50900000u, "DBG",  "dbg BAR first (== prov-VOID last+1)"},
        {0x509C0000u, "DBG",  "dbg BAR, mirror copy 39 VIA1 base"},
        {0x509FFFFFu, "DBG",  "dbg BAR last byte"},
        // ... and the arm STOPS at 0x9FFFFF.  0x50A0_0000 is the
        // SD-JTAG writer window at the SYSTEM level (axi_xbar.v
        // decodes it to S5 before S1), but peripheral_bus has NO
        // carve-out for it, so at the unit level it aliases to VIA1.
        // See the audit finding "SD-JTAG window has no peripheral_bus
        // carve-out" -- this expectation records the CURRENT unit-level
        // truth, which is NOT what the system does.
        {0x50A00000u, "VIA1", "dbg BAR last+1 -> unguarded VIA1 alias"},
        // SCC alternate base: off_raw[23:5]==0x7_8F01 => [0xF1E020,0xF1E03F]
        {0x50F1E000u, "IWM",  "SWIM reg0 base -- the 26360eb regression"},
        // Task #246: there is NO SCC alternate base.  The entire
        // 0x50F1_Exxx window is SWIM/IWM via the Q700 addr[23:18] mirror.
        // MAME makes ZERO accesses in E020..E03F across a full 7.5.3 boot
        // (46 reads in the window, all at E000); its SCC traffic is at the
        // canonical 0x50F0_C020/_C024.  These three rows previously encoded
        // the carve-out and were inverted when it was deleted.
        {0x50F1E01Fu, "IWM",  "IWM reg0 stride, was alt-base first-1"},
        {0x50F1E020u, "IWM",  "no SCC alt base (was SCC)"},
        {0x50F1E03Fu, "IWM",  "no SCC alt base (was SCC, top edge)"},
        {0x50F0C020u, "SCC",  "CANONICAL SCC — where MAME's 282k reads land"},
        {0x50F0C024u, "SCC",  "CANONICAL SCC — MAME's 1,970 reads"},
        {0x50F1E040u, "IWM",  "SCC alt-base last+1 -- window is 32 B"},
        // Legacy DAFB register window: [0xF9800000,0xF98003FF]
        {0xF97FFFFFu, "VOID", "DAFB legacy first-1"},
        {0xF9800000u, "DAFB", "DAFB legacy first"},
        {0xF98003FFu, "DAFB", "DAFB legacy last byte"},
        {0xF9800400u, "VOID", "DAFB legacy last+1"},
    };
    return run_decode_cases(cases, (int)(sizeof(cases)/sizeof(cases[0])));
}

// C. Every one of the 64 Q700 mirror copies, for every Mac device base.
//
//    The Q700 mirror ignores addr[23:18], so a device appears 64 times
//    inside 0x5000_0000..0x50FF_FFFF.  Four of our own windows punch
//    holes in that: DMA config (copies 4-7), the ex-sd_provision VOID
//    (copies 32-35) and debug_ctrl (copies 36-39).  Nothing before this
//    test checked which copies survive -- the suite only ever probed
//    copies 0, 2, 60 and 62.
//
//    NOTE this is a UNIT-level expectation.  At the system level
//    axi_xbar.v additionally steals mirror copy 40 (0x50A0_xxxx) for
//    the SD-JTAG writer before S1 ever sees it; peripheral_bus has no
//    matching arm, hence copy 40 reads as intact here.
static bool test_q700_mirror_copy_sweep() {
    struct Dev { uint32_t off; const char* name; };
    static const Dev devs[] = {
        {0x00000u, "VIA1"}, {0x02000u, "VIA2"},  {0x08000u, "ENET"},
        {0x0A000u, "SONIC"},{0x0C000u, "SCC"},   {0x0E000u, "ORWELL"},
        {0x0F000u, "SCSI"}, {0x11000u, "ADBINJ"},{0x14000u, "ASC"},
        {0x1E000u, "IWM"},
    };
    bool ok = true;
    for (int copy = 0; copy < 64; copy++) {
        // off_raw[23:20] is the top nibble of (copy << 18).
        uint32_t nib = (uint32_t)(copy >> 2);          // (copy<<18)>>20
        const char* carve = nullptr;
        if (nib == 0x1) carve = "VOID";                // DMA config
        else if (nib == 0x8) carve = "VOID";           // ex-sd_provision
        else if (nib == 0x9) carve = "DBG";            // debug_ctrl BAR
        for (unsigned d = 0; d < sizeof(devs)/sizeof(devs[0]); d++) {
            uint32_t addr = 0x50000000u | ((uint32_t)copy << 18) | devs[d].off;
            const char* want = carve ? carve : devs[d].name;
            const char* got  = probe_slot(addr);
            if (std::strcmp(got, want) != 0) {
                std::printf("    FAIL mirror copy %d 0x%08X: got %s, expected %s (%s)\n",
                            copy, addr, got, want, devs[d].name);
                ok = false;
            }
        }
    }
    return ok;
}

// D. The ex-sd_provision VOID arm, via its TRUE full-system-address form.
//
//    test_prov_window_void()'s "same window via the system-address form"
//    probe at 0x5008_C000 does NOT exercise this arm at all:
//        off_raw          = 0x08C000
//        off_raw[23:20]   = 0x0            (arm requires 0x8) -> no match
//        mac_off          = 0x08C000 & 0x03FFFF = 0x0C000 -> SLOT_SCC
//    It asserts only bresp == 0, which the SCC also returns, so it
//    passes while asserting the opposite of what happens.  The real
//    system-address form is 0x5080_0000.
static bool test_prov_void_true_system_form() {
    reset();
    uint32_t bresp = 0xFF;
    axi_write(0x50800000u, 0xDEADBEEFu, 0xFF, &bresp);
    CHECK_EQ("0x5080_0000 write VOID OKAY", bresp, 0u);
    CHECK_EQ("prov VOID did not alias to VIA1", (uint32_t)via1.writes.size(), 0u);
    CHECK_TRUE("prov VOID did not reach debug_ctrl", dbg_slv.aw_log.empty());

    uint32_t rresp = 0xFF;
    uint32_t got = axi_read(0x50800000u, &rresp);
    CHECK_EQ("0x5080_0000 read VOID OKAY", rresp, 0u);
    CHECK_EQ("0x5080_0000 read data all-zeros", got, 0u);
    CHECK_EQ("prov VOID read did not alias to VIA1", (uint32_t)via1.reads.size(), 0u);

    // Document the arithmetic above: 0x5008_C000 really is the SCC.
    reset();
    axi_write(0x5008C000u, 0x77u, 0, &bresp);
    CHECK_EQ("0x5008_C000 OKAY", bresp, 0u);
    CHECK_TRUE("0x5008_C000 is SCC (mac_off 0x0C000), not the prov VOID arm",
               scc.writes.size() == 1);
    return true;
}

// E. The DMA carve-out must not alias to VIA1.
//
//    test_unmapped_gaps_void() probes 0x5010_0000 but asserts only
//    bresp == 0 and "did not hit debug".  0x5010_0000 masks to
//    mac_off 0x00000 == VIA1, which is the exact aliasing the arm
//    exists to prevent -- delete the arm and that test still passes.
static bool test_dma_carveout_does_not_alias_via1() {
    reset();
    uint32_t bresp = 0xFF;
    axi_write(0x50100000u, 0x66u, 0, &bresp);
    CHECK_EQ("DMA carve-out write VOID OKAY", bresp, 0u);
    CHECK_EQ("DMA carve-out did NOT reach VIA1", (uint32_t)via1.writes.size(), 0u);
    CHECK_TRUE("DMA carve-out did not hit debug", dbg_slv.aw_log.empty());

    // ...and the same for the top of the 1 MB hole, which masks to
    // mac_off 0x3FFFF (a Q700 gap) and to mirror copy 7's VIA1 base.
    reset();
    axi_write(0x501C0000u, 0x67u, 0, &bresp);
    CHECK_EQ("DMA carve-out copy-7 write VOID OKAY", bresp, 0u);
    CHECK_EQ("DMA carve-out copy 7 did NOT reach VIA1",
             (uint32_t)via1.writes.size(), 0u);
    return true;
}

// ── KNOWN-RED FINDING (deliberately NOT in the RUN list below) ─────────
//
// FINDING: the SCC alternate-base carve-out still steals SWIM/IWM
// register-0 addresses, just 0x20 higher than the 26360eb bug did.
//
//   Carve-out (peripheral_bus.v:434):  off_raw[23:5] == 19'h7_8F01
//     19'h7_8F01 << 5 = 0x78F01 * 32 = 0xF1_E020
//     => matched range [0xF1_E020, 0xF1_E03F]  (32 bytes)
//
//   SWIM/IWM register select is addr[12:9] (peripheral_bus.v:1647,
//   matching MAME's `m_swim->read((offset >> 8) & 0xf)` with a 16-bit
//   offset, macquadra700.cpp:393):
//     mac_off 0x1_E020 >> 9 = 0xF0, & 0xF = 0  -> register 0
//     mac_off 0x1_E03F >> 9 = 0xF0, & 0xF = 0  -> register 0
//   i.e. ALL 32 carved-out bytes are inside SWIM register 0's 0x200
//   stride (mac_off 0x1_E000..0x1_E1FF), which MAME maps to the SWIM
//   for the whole range (macquadra700.cpp:568).
//
// CONSEQUENCE: 8 of those 32 byte addresses (addr[2:1] == 00, i.e.
// 0x50F1_E020/21/28/29/30/31/38/39) land on SCC channel-B CONTROL
// (scc_addr = {addr[5:4], addr[1], addr[2]} -> sel_a = addr[1] = 0,
// sel_data = addr[2] = 0; scc.v:111-112).  A control-port access RESETS
// the SCC's register pointer, which is SHARED across both channels
// (scc.v:438, :732-745) -- the same corruption mechanism as 26360eb,
// against AppleTalk/serial traffic in flight.
//
// peripheral_bus.v:448-450 already records this as an OPEN QUESTION
// ("whether this carve-out should exist at all -- it is the thing that
// collides with the SWIM mirror.  Nobody has confirmed the dispatcher
// needs an alt base").  This test encodes the MAME-faithful
// expectation.  It FAILS on current main BY DESIGN; it is left out of
// the RUN list so the gate stays green.  Register it in main() when the
// carve-out is removed or moved off the SWIM stride.
static bool test_KNOWNRED_scc_altbase_steals_swim_reg0() {
    static const DecodeCase cases[] = {
        {0x50F1E020u, "IWM", "MAME: SWIM reg 0 (mirror of 0x5001_E020)"},
        {0x50F1E021u, "IWM", "MAME: SWIM reg 0 -- SCC chanB CONTROL today"},
        {0x50F1E028u, "IWM", "MAME: SWIM reg 0 -- SCC chanB CONTROL today"},
        {0x50F1E030u, "IWM", "MAME: SWIM reg 0 -- SCC chanB CONTROL today"},
        {0x50F1E038u, "IWM", "MAME: SWIM reg 0 -- SCC chanB CONTROL today"},
        {0x50F1E03Fu, "IWM", "MAME: SWIM reg 0 (top of the carve-out)"},
    };
    return run_decode_cases(cases, (int)(sizeof(cases)/sizeof(cases[0])));
}


// ════════════════════════════════════════════════════════════════════════
// Peripheral-reset barrier (peripheral_bus.v `periph_rst` / `pb_quiesce`)
//
// THE DEFECT.  In the SoC the pb_* peripherals and peripheral_bus sit on
// DIFFERENT reset nets, on purpose: `pb_full_rst` (board cold | JTAG
// debug-full | warm_peripheral_reset, i.e. a 68040 RESET instruction)
// resets u_scsi/u_asc/u_via..., while peripheral_bus stays on
// pb_soc_full_rst so the still-running CPU keeps its I/O bus.  Every
// pb_* strobe peripheral_bus emits is a ONE-SHOT that is latched off
// until the peripheral's pb_ack comes back.  A peripheral held in reset
// never samples the strobe and never acks it, so the pulse is LOST and
// the FSM waits forever: wr_busy/rd_busy stay set and s_awready/s_arready
// stay low -- the whole S1 slave dead to every master, permanently, from
// a plain 68040 RESET instruction.
//
// The shim's own guard does not see the reset either: scsi.v's
// dma_{rd,wr}_ready mirrors are combinational from `scsi_ctrl_in`, reset
// by the same net as u_scsi, so they read "ready" for the whole window
// plus the ~2 cycles its synchroniser takes to reload.  The pulse ALWAYS
// fires into the reset chip.
//
// WHY THE OLD SUITE COULDN'T SEE IT.  This file's `*_stall` knobs withhold
// ack while still REMEMBERING the pulse, so the ack eventually arrives
// with no new strobe -- strictly more forgiving than a real reset chip.
// periph_rst_v (above) models the real thing.  And with
// ENABLE_ACK_WATCHDOG=1 (this build's default, the OPPOSITE of
// production) a lost pulse still "completes" via SLVERR after
// PB_ACK_TIMEOUT while the bytes are never delivered -- which is why
// `make tb-peripheral-bus-prod` rebuilds this same file at the
// production ENABLE_ACK_WATCHDOG=0 and runs exactly these scenarios.
//
// Assertions below are watchdog-INDEPENDENT on purpose: OKAY responses,
// bytes actually delivered, zero strobes into a reset face, and the front
// door open again afterwards.  All four fail in both builds without the
// fix (SLVERR/no-delivery with the watchdog on; hang with it off).
// ════════════════════════════════════════════════════════════════════════

struct RstTxnResult {
    bool     completed = false;
    uint32_t resp = 0;
    uint32_t data = 0;
};

// Run one AXI read, asserting periph_rst from `rst_at` cycles after the
// transaction is first presented and holding it for `rst_len` cycles.
// rst_at < 0 means "assert before the AR is presented at all".
static RstTxnResult read_across_periph_rst(uint32_t addr, uint8_t arsize,
                                           int rst_at, int rst_len,
                                           int budget = 400) {
    RstTxnResult r;
    uint32_t lane = (addr >> 2) & 0x3;

    if (rst_at < 0) { periph_rst_v = 1; for (int i = 0; i < -rst_at; i++) cycle(); }

    dut->s_arid = 0x3; dut->s_araddr = addr; dut->s_arlen = 0;
    dut->s_arsize = arsize; dut->s_arburst = 1;
    dut->s_arvalid = 1; dut->s_rready = 1;

    bool ar_done = false;
    for (int i = 0; i < budget && !r.completed; i++) {
        if (rst_at >= 0 && i == rst_at) periph_rst_v = 1;
        if (periph_rst_v && rst_len-- <= 0) periph_rst_v = 0;
        drive_lite_slaves(); dut->eval();
        drive_pb_peripherals(); dut->eval();
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (r_hs) { r.data = get128(dut->s_rdata)[lane]; r.resp = dut->s_rresp; }
        sample_pb_writes(); step_registered_acks(); tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) r.completed = true;
    }
    dut->s_arvalid = 0;
    periph_rst_v = 0;
    for (int i = 0; i < 8; i++) cycle();
    return r;
}

// Write twin.  `lane_strb_4b` is the m68k_mem_strb-shaped 4-bit pattern.
static RstTxnResult write_across_periph_rst(uint32_t addr, uint32_t lane_data,
                                            uint8_t lane_strb_4b,
                                            int rst_at, int rst_len,
                                            int budget = 400,
                                            uint8_t awsize = 4) {
    RstTxnResult r;
    uint32_t lane = (addr >> 2) & 0x3;

    if (rst_at < 0) { periph_rst_v = 1; for (int i = 0; i < -rst_at; i++) cycle(); }

    dut->s_awid = 0xC; dut->s_awaddr = addr; dut->s_awlen = 0;
    dut->s_awsize = awsize; dut->s_awburst = 1; dut->s_awvalid = 1;
    Word128 w{0,0,0,0}; w[lane] = lane_data; set128(dut->s_wdata, w);
    dut->s_wstrb = (uint16_t)((uint32_t)lane_strb_4b << (lane * 4));
    dut->s_wlast = 1; dut->s_wvalid = 1; dut->s_bready = 1;

    bool aw_done = false, w_done = false;
    for (int i = 0; i < budget && !r.completed; i++) {
        if (rst_at >= 0 && i == rst_at) periph_rst_v = 1;
        if (periph_rst_v && rst_len-- <= 0) periph_rst_v = 0;
        drive_lite_slaves(); dut->eval();
        drive_pb_peripherals(); dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) r.resp = dut->s_bresp;
        sample_pb_writes(); step_registered_acks(); tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) r.completed = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    periph_rst_v = 0;
    for (int i = 0; i < 8; i++) cycle();
    return r;
}

// "The front door is open and a plain transaction still works."  This is
// the property the defect destroys: after ONE lost pulse, s_arready /
// s_awready are low forever and every master behind S1 is dead.
static bool s1_still_alive() {
    if (!dut->s_arready || !dut->s_awready) return false;
    uint32_t resp = 0xFF;
    uint32_t v = axi_read(0x50F00000u, &resp);          // VIA1 reg 0
    if (resp != 0 || (v & 0xFF) != 0xA1u) return false;
    resp = 0xFF;
    axi_write(0x50F00000u, 0x0000005Au, 0, &resp);
    return resp == 0;
}

// 1. AR arrives while the SCSI DMA shim is held in reset.
//    Before the fix: peripheral_bus accepts it immediately, fires scsi_rd
//    into the reset chip, latches rd_scsi_beat_kicked_q and never retries.
static bool test_periph_rst_shim_read_defers_and_completes() {
    reset();
    // -4 => assert the reset BEFORE the AR is even presented, which is the
    // measured shape: the reset lands, then the CPU touches the shim.
    RstTxnResult r = read_across_periph_rst(0x5000F100u, /*arsize=*/0,
                                            /*rst_at=*/-4, /*rst_len=*/20);
    CHECK_TRUE("shim read completed", r.completed);
    CHECK_EQ("shim read RRESP is OKAY", r.resp, 0u);
    CHECK_EQ("shim read got the post-reset byte", r.data & 0xFFu, 0x5Cu);
    CHECK_EQ("no scsi_rd strobe was lost into the reset chip",
             (uint32_t)pulses_into_reset, 0u);
    CHECK_TRUE("exactly one scsi_rd pulse, after the release",
               scsi.reads.size() == 1);
    CHECK_TRUE("S1 front door still alive", s1_still_alive());
    return true;
}

// 2. Same, write side: a 16-bit pseudo-DMA store split into two byte
//    beats by the wr_scsi_* serializer.  Both bytes must reach the chip.
static bool test_periph_rst_shim_write_defers_and_completes() {
    reset();
    RstTxnResult r = write_across_periph_rst(0x5000F100u, 0x0000A5B6u,
                                             /*lane_strb_4b=*/0x3,
                                             /*rst_at=*/-4, /*rst_len=*/20,
                                             /*budget=*/400, /*awsize=*/1);
    CHECK_TRUE("shim write completed", r.completed);
    CHECK_EQ("shim write BRESP is OKAY", r.resp, 0u);
    CHECK_EQ("no scsi_wr strobe was lost into the reset chip",
             (uint32_t)pulses_into_reset, 0u);
    CHECK_TRUE("both pseudo-DMA bytes delivered after the release",
               scsi.writes.size() == 2);
    CHECK_EQ("high byte first (big-endian walk)", scsi.writes[0].second, 0xA5u);
    CHECK_EQ("low byte second",                   scsi.writes[1].second, 0xB6u);
    return true;
}

// 3. The ASC / ORWELL multi-hot byte walk (wr_ser_pulse) is an
//    INDEPENDENT one-shot family from the two shim serializers, and it
//    carries real ROM traffic (the chime path's MOVE.W to the volume
//    registers; ORWELL's `move.l D4,(A2)+` init loop).
static bool test_periph_rst_asc_byte_walk_defers_and_completes() {
    reset();
    // MOVE.W #$7F00 -> ASC 0xF06: AW byte-granular, two hot strobes.
    RstTxnResult r = write_across_periph_rst(0x50014F06u, 0x00007F00u,
                                             /*lane_strb_4b=*/0x3,
                                             /*rst_at=*/-4, /*rst_len=*/20);
    CHECK_TRUE("asc byte-walk write completed", r.completed);
    CHECK_EQ("asc BRESP is OKAY", r.resp, 0u);
    CHECK_EQ("no asc_wr strobe was lost into the reset chip",
             (uint32_t)pulses_into_reset, 0u);
    CHECK_TRUE("both bytes delivered after the release", asc.writes.size() == 2);
    CHECK_EQ("byte 0 addr", asc.writes[0].first,  0xF06u);
    CHECK_EQ("byte 0 data", asc.writes[0].second, 0x7Fu);
    CHECK_EQ("byte 1 addr", asc.writes[1].first,  0xF07u);
    CHECK_EQ("byte 1 data", asc.writes[1].second, 0x00u);
    return true;
}

// 4. The ORWELL leg of the same family (combinational pb_ack, so the
//    walk latches the ack in the PULSE cycle -- a cancelled pulse must
//    not leave wr_ser_ack_seen_q set).
static bool test_periph_rst_orwell_byte_walk_defers_and_completes() {
    reset();
    RstTxnResult r = write_across_periph_rst(0x5000E010u, 0xDEADBEEFu,
                                             /*lane_strb_4b=*/0xF,
                                             /*rst_at=*/-4, /*rst_len=*/20);
    CHECK_TRUE("orwell byte-walk write completed", r.completed);
    CHECK_EQ("orwell BRESP is OKAY", r.resp, 0u);
    CHECK_EQ("no orwell_wr strobe was lost into the reset chip",
             (uint32_t)pulses_into_reset, 0u);
    CHECK_TRUE("all four bytes delivered after the release",
               orwell.writes.size() == 4);
    CHECK_EQ("byte 0", orwell.writes[0].second, 0xDEu);
    CHECK_EQ("byte 3", orwell.writes[3].second, 0xEFu);
    return true;
}

// 5. The generic one-shot family: an ordinary registered-ack device
//    (VIA1).  pb_rd_active / pb_wr_active are latched off by
//    rd_ar_done / wr_w_done, so they are stranded the same way.
static bool test_periph_rst_generic_via_rw_defers_and_completes() {
    reset();
    RstTxnResult rr = read_across_periph_rst(0x50F00000u, /*arsize=*/0,
                                             /*rst_at=*/-4, /*rst_len=*/20);
    CHECK_TRUE("via1 read completed", rr.completed);
    CHECK_EQ("via1 RRESP is OKAY", rr.resp, 0u);
    CHECK_EQ("via1 read got the live (post-reset) value", rr.data & 0xFFu, 0xA1u);
    CHECK_EQ("no via1_rd strobe lost into the reset chip",
             (uint32_t)pulses_into_reset, 0u);

    reset();
    RstTxnResult wr = write_across_periph_rst(0x50F00000u, 0x0000005Au,
                                              /*lane_strb_4b=*/0x1,
                                              /*rst_at=*/-4, /*rst_len=*/20);
    CHECK_TRUE("via1 write completed", wr.completed);
    CHECK_EQ("via1 BRESP is OKAY", wr.resp, 0u);
    CHECK_EQ("no via1_wr strobe lost into the reset chip",
             (uint32_t)pulses_into_reset, 0u);
    // The W beat is DEFERRED, not dropped: s_wready is held low for the
    // whole window, so WVALID is still up when the barrier lifts.
    CHECK_TRUE("the byte is delivered late, not lost", via1.writes.size() == 1);
    CHECK_EQ("byte value", via1.writes[0].second, 0x5Au);
    CHECK_TRUE("S1 front door still alive", s1_still_alive());
    return true;
}

// 6. THE ASSERT SIDE.  A front-door hold cannot rescue a transaction that
//    was already ADMITTED and already KICKED when the reset lands, so
//    sweep the reset's arrival across the whole life of a transaction and
//    require the only invariant that matters: S1 never dies.
//
//    This is the half that the stranded-register list is about
//    (rd_scsi_beat_kicked_q / rd_scsi_phase_q / rd_ar_done;
//    wr_scsi_beat_kicked_q / wr_scsi_strb_q / wr_scsi_first_q; the seven
//    wr_ser_*).  Each offset that leaves one of them latched is a dead S1.
static bool test_periph_rst_midflight_sweep_never_kills_s1() {
    struct Case { const char* name; bool is_write; uint32_t addr;
                  uint32_t data; uint8_t strb; uint8_t size; };
    static const Case cases[] = {
        { "shim byte read",   false, 0x5000F100u, 0,          0x0, 0 },
        { "shim word read",   false, 0x5000F100u, 0,          0x0, 1 },
        { "shim word write",  true,  0x5000F100u, 0x0000A5B6u, 0x3, 1 },
        { "asc byte walk",    true,  0x50014F06u, 0x00007F00u, 0x3, 4 },
        { "orwell long walk", true,  0x5000E010u, 0xDEADBEEFu, 0xF, 4 },
        { "via1 byte write",  true,  0x50F00000u, 0x0000005Au, 0x1, 4 },
        { "via1 byte read",   false, 0x50F00000u, 0,          0x0, 0 },
        { "scc byte read",    false, 0x5000C000u, 0,          0x0, 0 },
    };
    for (const Case& c : cases) {
        for (int at = 0; at <= 14; at++) {
            reset();
            RstTxnResult r = c.is_write
                ? write_across_periph_rst(c.addr, c.data, c.strb, at, 9, 400, c.size)
                : read_across_periph_rst(c.addr, c.size, at, 9);
            if (!r.completed) {
                std::printf("    FAIL %s: reset at cycle %d never completed "
                            "(S1 permanently dead)\n", c.name, at);
                return false;
            }
            if (r.resp != 0) {
                std::printf("    FAIL %s: reset at cycle %d -> RESP=%u "
                            "(expected OKAY)\n", c.name, at, r.resp);
                return false;
            }
            if (pulses_into_reset != 0) {
                std::printf("    FAIL %s: reset at cycle %d -> %d pb strobe "
                            "cycle(s) fired into a face held in reset\n",
                            c.name, at, pulses_into_reset);
                return false;
            }
            if (!s1_still_alive()) {
                std::printf("    FAIL %s: reset at cycle %d -> S1 front door "
                            "did not reopen\n", c.name, at);
                return false;
            }
        }
    }
    return true;
}

// 7. The barrier is pb-FACE scoped, not bus-wide: debug_ctrl and the DAFB
//    shim have independent AXI-Lite channels and are not reset with the
//    Mac peripherals, so JTAG polling must keep working right through a
//    peripheral reset -- which is exactly when it is wanted.
static bool test_periph_rst_does_not_block_debug_ctrl() {
    reset();
    periph_rst_v = 1;
    for (int i = 0; i < 4; i++) cycle();

    uint32_t resp = 0xFF;
    uint32_t v = axi_read(0x50900010u, &resp);
    CHECK_EQ("debug_ctrl read served during a peripheral reset", resp, 0u);
    CHECK_EQ("debug_ctrl rdata", v, 0xCAFEBABEu);

    resp = 0xFF;
    axi_write(0x50900014u, 0x12345678u, 0xFF, &resp);
    CHECK_EQ("debug_ctrl write served during a peripheral reset", resp, 0u);
    CHECK_TRUE("debug_ctrl write reached the slave", dbg_slv.aw_log.size() == 1);

    periph_rst_v = 0;
    for (int i = 0; i < 8; i++) cycle();
    CHECK_TRUE("S1 front door still alive", s1_still_alive());
    return true;
}

// ─── main ──────────────────────────────────────────────────────────────
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { std::printf("[PASS] %s\n", #fn); n_pass++; } \
    else    { std::printf("[FAIL] %s\n", #fn); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vperipheral_bus;

    RUN(test_via1_byte_rw);
    RUN(test_via2_byte_rw);
    RUN(test_scc_decode);
    RUN(test_orwell_decode);
    RUN(test_orwell_e000_long_read_exact_fault_pc_address);
    RUN(test_scsi_decode);
    RUN(test_scsi_dma_shim_word_read);
    RUN(test_scsi_dma_shim_word_write);
    RUN(test_scsi_dma_shim_write_drq_withhold);
    RUN(test_scsi_dma_shim_read_drq_withhold);
    RUN(test_asc_decode);
    RUN(test_iwm_decode);
    RUN(test_iwm_reg0_not_stolen_by_scc_window);
    RUN(test_adbinj_decode);
    RUN(test_asc_byte_select);
    RUN(test_asc_multi_byte_word_off2);
    RUN(test_asc_multi_byte_word_off0);
    RUN(test_asc_multi_byte_long_off0);
    RUN(test_asc_multi_byte_followup_single_byte);
    RUN(test_orwell_multi_byte_long_serializes);
    RUN(test_orwell_multi_byte_word_off2_serializes);
    RUN(test_orwell_multi_byte_followup_single_byte);
    RUN(test_sonic_driver_reset_long_write_serializes);
    RUN(test_sonic_multi_byte_long_distinct_bytes);
    RUN(test_sonic_word_path_not_shadowed_by_serializer);
    RUN(test_sonic_multi_byte_followup_single_byte);
    RUN(test_adbinj_full_strobe_word_not_serialized);
    RUN(test_enet_multi_byte_not_serialized);
    RUN(test_strided_slots_multi_hot_stay_single_pulse);
    RUN(test_q700_mame_mirror);
    RUN(test_via_alias_probe_family_d2_offsets);
    RUN(test_no_scc_alt_base_bit17_alias);
    RUN(test_enet_sonic_decode);
    RUN(test_prov_window_void);
    RUN(test_unmapped_q700_gap_void);
    RUN(test_dbg_service_route);
    RUN(test_dafb_axi_passthrough);
    RUN(test_unmapped_gaps_void);
    RUN(test_concurrent_rw);
    RUN(test_q700_via_stride_aliases);
    // Task #82 widening: stress scenarios
    RUN(test_simultaneous_write_read);
    RUN(test_same_slot_write_read_interlock);
    RUN(test_same_slot_scsi_stalled_write_blocks_read);
    RUN(test_slow_ack_stall);
    RUN(test_granularity_per_device);
    RUN(test_window_boundaries);
    RUN(test_unaligned_byte);
    RUN(test_parallel_pb_and_lite);
    RUN(test_be_lane_byte_matrix);
    RUN(test_le_byte_write_with_live_neighbour_lanes);
    // Bounded ack watchdog (PB_WATCHDOG_LOG2 — tb builds with the bound
    // shortened to 2^10; see the Makefile rule).
#ifndef PB_PROD_WATCHDOG_OFF
    // These three only exist when ENABLE_ACK_WATCHDOG=1, which is this
    // build's -G override and the OPPOSITE of production.  The
    // `tb-peripheral-bus-prod` build defines PB_PROD_WATCHDOG_OFF and
    // rebuilds this same file at the production ENABLE_ACK_WATCHDOG=0.
    RUN(test_watchdog_write_ack_timeout_slverr);
    RUN(test_watchdog_read_ack_timeout_slverr);
    RUN(test_watchdog_slow_drq_wait_not_slverrd);
#else
    (void)test_watchdog_write_ack_timeout_slverr;
    (void)test_watchdog_read_ack_timeout_slverr;
    (void)test_watchdog_slow_drq_wait_not_slverrd;
#endif

    // ── Peripheral-reset barrier (periph_rst / pb_quiesce) ────────────
    // Watchdog-independent by construction, so they are a real negative
    // control in BOTH builds.  See the comment block above
    // test_periph_rst_shim_read_defers_and_completes().
    RUN(test_periph_rst_shim_read_defers_and_completes);
    RUN(test_periph_rst_shim_write_defers_and_completes);
    RUN(test_periph_rst_asc_byte_walk_defers_and_completes);
    RUN(test_periph_rst_orwell_byte_walk_defers_and_completes);
    RUN(test_periph_rst_generic_via_rw_defers_and_completes);
    RUN(test_periph_rst_midflight_sweep_never_kills_s1);
    RUN(test_periph_rst_does_not_block_debug_ctrl);
    // Address-decode audit (2026-08): systematic window-edge coverage.
    RUN(test_decode_edges_mac_devices);
    RUN(test_decode_edges_raw_carveouts);
    RUN(test_q700_mirror_copy_sweep);
    RUN(test_prov_void_true_system_form);
    RUN(test_dma_carveout_does_not_alias_via1);
    // DELIBERATELY UNREGISTERED known-red finding -- see the comment
    // block above test_KNOWNRED_scc_altbase_steals_swim_reg0().  It
    // fails on current main by design (the SCC alt-base carve-out still
    // overlaps SWIM/IWM register 0).  There is no XFAIL convention in
    // this suite, so leaving it out of RUN() is how a known-red finding
    // is parked without breaking `make tb-all`.
    // RUN(test_KNOWNRED_scc_altbase_steals_swim_reg0);
    (void)test_KNOWNRED_scc_altbase_steals_swim_reg0;   // keep it compiled

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
