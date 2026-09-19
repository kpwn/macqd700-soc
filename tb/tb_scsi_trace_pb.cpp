// tb/tb_scsi_trace_pb.cpp — integration tb for the 53C96 trace ring as it
// is actually wired in a bitstream: real AXI4 -> real peripheral_bus.v ->
// real scsi.v, with rtl/soc/scsi_trace_ring.v snooping that port and
// rtl/soc/vhdd_ctrl.v presenting it to the host.  See tb_scsi_trace_pb.v
// for why this exists alongside the ring's own unit tb.
//
// THE METHOD, and why it is not "check the ring says what I expected".
// ────────────────────────────────────────────────────────────────────
// A trace ring is an INSTRUMENT.  The failure mode that matters is not
// "it records nothing" (obvious) but "it records something plausible
// that is not what the bus did" -- a tap on the wrong wire, a beat
// tracker that drops a beat when two peripherals' strobes collide, an
// off-by-one in the readout path.  A test written from the test's own
// expectations cannot see any of those: it agrees with the ring exactly
// where both are wrong.
//
// So this harness watches the SNOOPED BUS itself, cycle by cycle, and
// builds an independent golden model of what the ring must contain --
// beat tracking and poll filter reimplemented from
// rtl/soc/scsi_trace_ring.v's stated contract, driven by the wires, not
// by the script.  The tests then drive REALISTIC traffic (the Q700 ROM's
// select / READ(6) / 16-byte pseudo-DMA chunk drain, polling loops and
// all) and compare the host-visible dump against that model, entry for
// entry, in order.
//
// The model deliberately does NOT know about `cap_en`.  That is what
// makes --positive-control work: with capture disabled the traffic still
// happens, the model still says "these N entries must be here", and
// every comparison must fail.
//
// It also measures one property the model cannot assume: the ring's beat
// tracker holds ONE beat, so a request strobe that appears only during
// the cycle a previous beat is committing would be dropped.  The model
// counts those, and the tests assert the count is zero -- a measurement
// of the real peripheral_bus timing, not a hope about it.
//
// Build/run: make tb-scsi-trace-pb   (runs the positive control first)

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_trace_pb.h"

static Vtb_scsi_trace_pb *dut = nullptr;
static int n_fail = 0;
static int n_pass = 0;
// Bus-level failures inside the CSR helpers, counted separately from test
// failures so the positive-control threshold below stays meaningful.
static int cfg_errors = 0;
static bool positive_control = false;

using Word128 = std::array<uint32_t, 4>;
template <typename Port> static void set128(Port &p, const Word128 &v) {
    for (int i = 0; i < 4; i++) p[i] = v[i];
}
template <typename Port> static Word128 get128(Port &p) {
    Word128 v{};
    for (int i = 0; i < 4; i++) v[i] = p[i];
    return v;
}

static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

#define CHECK_TRUE(name, cond)                                                 \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("  FAIL %s\n", name);                                  \
            ok = false;                                                        \
        }                                                                      \
    } while (0)
#define CHECK_EQ(name, got, exp)                                               \
    do {                                                                       \
        uint32_t _g = (uint32_t)(got);                                         \
        uint32_t _e = (uint32_t)(exp);                                         \
        if (_g != _e) {                                                        \
            std::printf("  FAIL %s: got 0x%x, expected 0x%x\n", name, _g, _e); \
            ok = false;                                                        \
        }                                                                      \
    } while (0)

// ─── Golden model of the ring ───────────────────────────────────────────
// Reimplements rtl/soc/scsi_trace_ring.v's capture contract from its
// header comment, fed from the snooped bus.  Kept structurally separate
// from the RTL (no shared code, no shared constants beyond the entry
// field positions, which are the interface) so that a change to one does
// not silently drag the other with it.
struct Model {
    static constexpr int DEPTH = 4096;

    // Beat tracker
    bool pend = false;
    bool lat_wr = false, lat_dma = false;
    int lat_reg = 0;
    uint8_t lat_wdata = 0;

    // Poll filter shadow, indexed {dma, reg} exactly as the RTL does
    uint8_t last_val[32] = {0};
    bool last_vld[32] = {false};

    // Ring
    std::deque<uint32_t> ring;  // oldest first, capped at DEPTH
    uint64_t total = 0;         // total entries ever pushed (for wr_ptr)
    bool frozen = false;        // model-side freeze, set by the test

    // Beat-drop detector, see the header.  A request strobe that is
    // present while the tracker is busy and then VANISHES (or is replaced
    // by a different request) before the tracker frees up was never
    // latched, so its beat is missing from the trace with nothing to say
    // so.  `sig` identifies the request so a strobe replaced by a
    // different one is not mistaken for the same one waiting.
    bool unlatched = false;
    uint64_t unlatched_sig = 0;
    int drops = 0;

    void clear() {
        ring.clear();
        total = 0;
        frozen = false;
        for (int i = 0; i < 32; i++) last_vld[i] = false;
        unlatched = false;
        drops = 0;
    }

    void reset() {
        pend = false;
        clear();
    }

    // One pb_clk rising edge, with the settled pre-edge bus values.
    // Mirrors rtl/soc/scsi_trace_ring.v: latch and commit are both
    // decided from the PRE-edge value of `pend`, so they are mutually
    // exclusive.
    void edge(bool wr, bool rd, uint32_t addr, uint8_t wdata, uint8_t rdata,
              bool ack) {
        const bool req = wr || rd;
        const bool old_pend = pend;
        const bool latch_now = !old_pend && req;
        const uint64_t sig = ((uint64_t)wr << 40) | ((uint64_t)rd << 41) |
                             ((uint64_t)addr << 8) | wdata;

        if (unlatched && (!req || sig != unlatched_sig)) drops++;
        unlatched = req && !latch_now;
        unlatched_sig = sig;

        if (latch_now) {
            pend = true;
            lat_wr = wr;
            lat_dma = (addr == 0x100u) || (addr == 0x101u);
            lat_reg = (int)(addr & 0xfu);
            lat_wdata = wdata;
        } else if (old_pend && ack) {
            pend = false;
            const uint8_t data = lat_wr ? lat_wdata : rdata;
            const int idx = (lat_dma ? 16 : 0) | lat_reg;
            const bool keep =
                lat_wr || !last_vld[idx] || (last_val[idx] != data);
            if (lat_wr) {
                last_vld[idx] = false;
            } else {
                last_val[idx] = data;
                last_vld[idx] = true;
            }
            if (keep && !frozen) {
                uint32_t e = ((uint32_t)lat_wr << 31) |
                             ((uint32_t)lat_dma << 30) |
                             ((uint32_t)(lat_reg & 0xf) << 26) |
                             ((uint32_t)data << 18);
                ring.push_back(e);
                if ((int)ring.size() > DEPTH) ring.pop_front();
                total++;
            }
        }
    }

    uint32_t wrptr() const { return (uint32_t)(total % DEPTH); }
    bool wrapped() const { return total >= DEPTH; }
};

static Model model;

// ─── Mocked SD backing store (same shape as tb_pb_scsi.cpp's) ───────────
struct SdMock {
    std::vector<uint8_t> read_sector;
    uint32_t last_lba = 0;
    uint8_t last_cmd_type = 0;
    int gap = 0;
    int gap_ctr = 0;
    int delay = 0;
    int cnt = 0;
    int total = 0;
    enum class State { Idle, Reading, Done } st = State::Idle;
};
static SdMock sd_mock;

static void sd_mock_tick() {
    dut->sd_rd_valid = 0;
    dut->sd_done = 0;
    dut->sd_error = 0;
    dut->sd_wr_ready = 0;

    if (sd_mock.st == SdMock::State::Idle) {
        dut->sd_busy = 0;
        if (dut->sd_go) {
            sd_mock.last_cmd_type = dut->sd_cmd_type;
            sd_mock.last_lba = dut->sd_lba;
            sd_mock.cnt = 0;
            sd_mock.total = 512 * (dut->sd_block_count ? dut->sd_block_count : 1);
            sd_mock.delay = 4;
            sd_mock.gap_ctr = 0;
            sd_mock.st = SdMock::State::Reading;
            dut->sd_busy = 1;
        }
    } else if (sd_mock.st == SdMock::State::Reading) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) {
            --sd_mock.delay;
            return;
        }
        if (sd_mock.gap_ctr > 0) {
            --sd_mock.gap_ctr;
            return;
        }
        if (!dut->sd_rd_ready) return;
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
    } else if (sd_mock.st == SdMock::State::Done) {
        dut->sd_busy = 0;
        dut->sd_done = 1;
        sd_mock.st = SdMock::State::Idle;
    }
}

// ─── Two independent clocks on one fine-grained time base ──────────────
// pb_clk toggles every PB_HALF units, core_clk every RD_HALF.  The halves
// are coprime so the two never settle into a fixed phase relationship --
// the ring's control CDC and readout synchronisers are then genuinely
// crossed, which is the whole point of separating them.
static constexpr int PB_HALF = 2;
static constexpr int RD_HALF = 3;
static uint64_t tnow = 0;
static bool model_enabled = true;

static int level_at(uint64_t t, int half) { return (int)((t / half) & 1u); }

static void unit_step() {
    dut->eval();
    const int npb = level_at(tnow + 1, PB_HALF);
    const int nrd = level_at(tnow + 1, RD_HALF);
    const bool pb_rise = npb && !dut->clk;
    if (pb_rise) {
        sd_mock_tick();
        dut->eval();
        if (model_enabled)
            model.edge(dut->snoop_scsi_wr, dut->snoop_scsi_rd,
                       dut->snoop_scsi_addr, dut->snoop_scsi_wdata,
                       dut->snoop_scsi_rdata, dut->snoop_scsi_ack);
    }
    dut->clk = npb;
    dut->rd_clk = nrd;
    dut->eval();
    tnow++;
    main_time++;
}

// Advance to just after the next pb_clk rising edge.
static void pb_tick() {
    for (int i = 0; i < 4 * PB_HALF + 4; i++) {
        bool was = dut->clk;
        unit_step();
        if (!was && dut->clk) return;
    }
}
// Advance to just after the next core_clk rising edge.
static void rd_tick() {
    for (int i = 0; i < 4 * RD_HALF + 4; i++) {
        bool was = dut->rd_clk;
        unit_step();
        if (!was && dut->rd_clk) return;
    }
}
static void idle(int pb_cycles) {
    for (int i = 0; i < pb_cycles; i++) pb_tick();
}

static void reset_all() {
    dut->s_awid = 0; dut->s_awaddr = 0; dut->s_awlen = 0;
    dut->s_awsize = 0; dut->s_awburst = 0; dut->s_awvalid = 0;
    set128(dut->s_wdata, Word128{0, 0, 0, 0});
    dut->s_wstrb = 0; dut->s_wlast = 0; dut->s_wvalid = 0;
    dut->s_bready = 1;
    dut->s_arid = 0; dut->s_araddr = 0; dut->s_arlen = 0;
    dut->s_arsize = 0; dut->s_arburst = 0; dut->s_arvalid = 0;
    dut->s_rready = 1;
    dut->cfg_awaddr = 0; dut->cfg_awvalid = 0;
    dut->cfg_wdata = 0; dut->cfg_wstrb = 0; dut->cfg_wvalid = 0;
    dut->cfg_bready = 1;
    dut->cfg_araddr = 0; dut->cfg_arvalid = 0; dut->cfg_rready = 1;
    dut->scsi_ctrl_in = 0;
    dut->disk_num_lbas = 1048576;
    dut->cap_en = positive_control ? 0 : 1;
    sd_mock = SdMock();
    model.reset();
    model_enabled = false;      // nothing meaningful on the bus in reset
    dut->rst = 1;
    dut->rd_rst = 1;
    for (int i = 0; i < 12; i++) pb_tick();
    dut->rst = 0;
    dut->rd_rst = 0;
    for (int i = 0; i < 4; i++) pb_tick();
    model.reset();
    model_enabled = true;
}

// ─── AXI4 (Mac MMIO) helpers ────────────────────────────────────────────
struct AxiReadResult { uint32_t data; uint32_t resp; bool completed; };

static AxiReadResult axi_read_full(uint32_t addr, uint8_t arsize, int max_cycles) {
    dut->s_arid = 0x3;
    dut->s_araddr = addr;
    dut->s_arlen = 0;
    dut->s_arsize = arsize;
    dut->s_arburst = 1;
    dut->s_arvalid = 1;
    dut->s_rready = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    uint32_t got = 0, resp = 0;
    bool ar_done = false, r_done = false;
    for (int i = 0; i < max_cycles && !r_done; i++) {
        dut->eval();
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs = dut->s_rvalid && dut->s_rready;
        if (r_hs) {
            Word128 v = get128(dut->s_rdata);
            got = v[lane];
            resp = dut->s_rresp;
        }
        pb_tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) r_done = true;
    }
    dut->s_arvalid = 0;
    idle(2);
    return {got, resp, r_done};
}

static uint32_t axi_read(uint32_t addr, uint8_t arsize = 4, int max_cycles = 400) {
    return axi_read_full(addr, arsize, max_cycles).data;
}

static bool axi_write(uint32_t addr, uint8_t byte, int max_cycles = 400) {
    dut->s_awid = 0xC;
    dut->s_awaddr = addr;
    dut->s_awlen = 0;
    dut->s_awsize = 4;
    dut->s_awburst = 1;
    dut->s_awvalid = 1;

    uint32_t lane = (addr >> 2) & 0x3;
    Word128 w{0, 0, 0, 0};
    w[lane] = byte;
    set128(dut->s_wdata, w);
    dut->s_wstrb = (uint16_t)(1u << (lane * 4 + 0));
    dut->s_wlast = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    for (int i = 0; i < max_cycles && !b_done; i++) {
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs = dut->s_wvalid && dut->s_wready;
        bool b_hs = dut->s_bvalid && dut->s_bready;
        pb_tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs && !w_done) { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    idle(2);
    return b_done;
}

// Register N lives at raw offset 0xF000 + (N<<4); the pseudo-DMA aperture
// is 0xF100 (tb_pb_scsi.cpp, matching the DAFB TurboSCSI map).
static constexpr uint32_t SCSI_BASE = 0x000F000u;
static constexpr uint32_t SCSI_DMA_SHIM = 0x000F100u;

static void reg_w(uint8_t off, uint8_t v) {
    axi_write(SCSI_BASE | ((uint32_t)off << 4), v);
}
static uint8_t reg_r(uint8_t off) {
    return (uint8_t)axi_read(SCSI_BASE | ((uint32_t)off << 4), 4, 400);
}

// ─── AXI4-Lite (host/JTAG) helpers for the vhdd_ctrl CSR ───────────────
// Byte offsets are exactly the ones tools/jtag_repl.tcl uses.
static constexpr uint32_t VHDD_OFF_IDENT = 0x000;
static constexpr uint32_t VHDD_OFF_TRACE_CTRL = 0x020;
static constexpr uint32_t VHDD_OFF_TRACE_ADDR = 0x024;
static constexpr uint32_t VHDD_OFF_TRACE_DATA = 0x028;
static constexpr uint32_t VHDD_IDENT = 0x5D0D0001u;
static constexpr int VHDD_TRACE_DEPTH = 4096;

static uint32_t cfg_read(uint32_t off, int max_cycles = 200) {
    dut->cfg_araddr = off;
    dut->cfg_arvalid = 1;
    dut->cfg_rready = 1;
    uint32_t got = 0;
    bool ar_done = false, r_done = false;
    for (int i = 0; i < max_cycles && !r_done; i++) {
        dut->eval();
        bool ar_hs = dut->cfg_arvalid && dut->cfg_arready;
        bool r_hs = dut->cfg_rvalid && dut->cfg_rready;
        if (r_hs) got = dut->cfg_rdata;
        rd_tick();
        if (ar_hs && !ar_done) { dut->cfg_arvalid = 0; ar_done = true; }
        if (r_hs) r_done = true;
    }
    dut->cfg_arvalid = 0;
    if (!r_done) {
        std::printf("  FAIL cfg_read(0x%03x) never completed\n", off);
        cfg_errors++;
    }
    for (int i = 0; i < 2; i++) rd_tick();
    return got;
}

static void cfg_write(uint32_t off, uint32_t data, int max_cycles = 200) {
    dut->cfg_awaddr = off;
    dut->cfg_awvalid = 1;
    dut->cfg_wdata = data;
    dut->cfg_wstrb = 0xF;
    dut->cfg_wvalid = 1;
    dut->cfg_bready = 1;
    bool aw_done = false, w_done = false, b_done = false;
    for (int i = 0; i < max_cycles && !b_done; i++) {
        dut->eval();
        bool aw_hs = dut->cfg_awvalid && dut->cfg_awready;
        bool w_hs = dut->cfg_wvalid && dut->cfg_wready;
        bool b_hs = dut->cfg_bvalid && dut->cfg_bready;
        rd_tick();
        if (aw_hs && !aw_done) { dut->cfg_awvalid = 0; aw_done = true; }
        if (w_hs && !w_done) { dut->cfg_wvalid = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->cfg_awvalid = 0;
    dut->cfg_wvalid = 0;
    if (!b_done) {
        std::printf("  FAIL cfg_write(0x%03x) never completed\n", off);
        cfg_errors++;
    }
    for (int i = 0; i < 2; i++) rd_tick();
}

// The three fields tools/jtag_repl.tcl unpacks out of TRACE_CTRL.
struct TraceStatus { uint32_t wrptr; bool wrapped; bool frozen; };
static TraceStatus trace_status() {
    uint32_t c = cfg_read(VHDD_OFF_TRACE_CTRL);
    return {c & 0xFFFu, ((c >> 16) & 1u) != 0, ((c >> 17) & 1u) != 0};
}
// Exactly jtag_repl.tcl's scsi_trace_entry: write the index, then read
// the data.  No auto-increment -- see the comment on OFF_TRACE_DATA in
// rtl/soc/vhdd_ctrl.v for why that matters.
static uint32_t trace_entry(int idx) {
    cfg_write(VHDD_OFF_TRACE_ADDR, (uint32_t)idx);
    return cfg_read(VHDD_OFF_TRACE_DATA);
}

// Host-side freeze / re-arm, byte-for-byte what the REPL writes.
static void trace_freeze() {
    cfg_write(VHDD_OFF_TRACE_CTRL, 0x1);
    // Let the level cross into pb_clk and the flag cross back.
    idle(40);
}
static void trace_rearm() {
    cfg_write(VHDD_OFF_TRACE_CTRL, 0x2);
    idle(40);
}

// Read the ring back oldest-first, exactly as scsi_trace_dump does.
static std::vector<uint32_t> trace_dump(const TraceStatus &st) {
    int n = st.wrapped ? VHDD_TRACE_DEPTH : (int)st.wrptr;
    int start = st.wrapped ? (int)st.wrptr : 0;
    std::vector<uint32_t> out;
    out.reserve(n);
    for (int i = 0; i < n; i++)
        out.push_back(trace_entry((start + i) % VHDD_TRACE_DEPTH));
    return out;
}

// The timestamp field is free-running and deliberately not modelled.
static uint32_t strip_ts(uint32_t e) { return e & 0xFFFC0000u; }

static void describe(uint32_t e, char *buf, size_t n) {
    std::snprintf(buf, n, "%c%s%x=%02x", ((e >> 31) & 1) ? 'W' : 'R',
                  ((e >> 30) & 1) ? "dma:" : "reg", (e >> 26) & 0xf,
                  (e >> 18) & 0xff);
}

// Compare a host dump against the golden model, entry for entry.
static bool compare_dump(const std::vector<uint32_t> &dump, const char *what) {
    bool ok = true;
    std::vector<uint32_t> exp(model.ring.begin(), model.ring.end());
    char nm[160];
    std::snprintf(nm, sizeof nm, "%s: entry count", what);
    CHECK_EQ(nm, dump.size(), exp.size());
    size_t n = dump.size() < exp.size() ? dump.size() : exp.size();
    int shown = 0;
    for (size_t i = 0; i < n; i++) {
        if (strip_ts(dump[i]) != strip_ts(exp[i])) {
            if (shown < 8) {
                char g[64], e[64];
                describe(dump[i], g, sizeof g);
                describe(exp[i], e, sizeof e);
                std::printf("  FAIL %s: entry %zu is %s, bus said %s\n", what, i,
                            g, e);
                shown++;
            }
            ok = false;
        }
    }
    if (!ok && shown >= 8) std::printf("  ... (further mismatches suppressed)\n");
    return ok;
}

// ─── Realistic traffic: the Q700 ROM's select + READ(6) + chunk drain ──
// Lifted from tb_pb_scsi.cpp's test_lbtm_word_chunk_drain_matches_mame,
// which is itself checked against the MAME capture record numbers.  Using
// the ROM's real sequence (polling loops included) is the point: a made-up
// sequence would not exercise the poll filter, which is the ring's most
// load-bearing and most easily-wrong behaviour.
static bool drive_rom_read6_chunk(uint16_t *payload_out) {
    bool ok = true;
    sd_mock.read_sector.resize(512);
    for (int i = 0; i < 512; ++i)
        sd_mock.read_sector[i] = (uint8_t)(0x11 + i * 7);
    sd_mock.gap = 0;

    reg_w(0xC, 0x04);   // config3 = LBTM, as the ROM writes at 0x40899120
    reg_w(0x4, 0x00);   // bus_id = TARGET_ID
    reg_w(0x3, 0x01);   // FLUSH_FIFO
    reg_w(0x1, 0x00);
    reg_w(0x0, 0x01);
    reg_w(0x3, 0xC1);   // DMA | CD_SELECT, empty FIFO

    bool seq_ok = false;
    for (int i = 0; i < 64; ++i) {
        if (reg_r(0x6) & 7) { seq_ok = true; break; }
    }
    CHECK_TRUE("seq_step != 0 after empty-FIFO select", seq_ok);

    // CDB: READ(6), opcode 0x08, lba 0, 1 block.
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

    // Arm one 16-byte chunk exactly like the ROM (tcount=16, DMA|CI_XFER).
    reg_w(0x0, 0x10);
    reg_w(0x1, 0x00);
    reg_w(0x3, 0x90);

    dut->scsi_ctrl_in = 0x080;  // DAFB read DRQ-check on

    for (int i = 0; i < 4000 && (reg_r(0x7) & 0x1F) != 0x10; ++i) {}
    CHECK_EQ("chip staged the whole 16-byte chunk", reg_r(0x7) & 0x1F, 0x10);

    // Eight 16-bit pseudo-DMA beats -- the aperture traffic the 7.5.3
    // investigation actually needs to see.
    for (int beat = 0; beat < 8; ++beat) {
        AxiReadResult r = axi_read_full(SCSI_DMA_SHIM, /*arsize=word*/ 1, 9000);
        char nm[96];
        std::snprintf(nm, sizeof nm, "pseudo-DMA word beat %d completed", beat);
        CHECK_TRUE(nm, r.completed);
        std::snprintf(nm, sizeof nm, "pseudo-DMA word beat %d resp OKAY", beat);
        CHECK_EQ(nm, r.resp, 0);
        uint16_t got = (uint16_t)(r.data & 0xFFFF);
        uint16_t exp = (uint16_t)((sd_mock.read_sector[2 * beat] << 8) |
                                  sd_mock.read_sector[2 * beat + 1]);
        std::snprintf(nm, sizeof nm, "pseudo-DMA word beat %d payload", beat);
        CHECK_EQ(nm, got, exp);
        if (payload_out) payload_out[beat] = got;
    }
    dut->scsi_ctrl_in = 0;
    return ok;
}

// ─── Tests ─────────────────────────────────────────────────────────────

// (1) The host register map itself.  Runs first and does NOT depend on
// capture, so a failure here says "the CSR wiring is wrong", not "the
// ring is empty" -- two very different bugs that would otherwise present
// identically over JTAG.
static bool test_host_register_map() {
    reset_all();
    bool ok = true;
    CHECK_EQ("IDENT at +0x000", cfg_read(VHDD_OFF_IDENT), VHDD_IDENT);

    cfg_write(VHDD_OFF_TRACE_ADDR, 0x123);
    CHECK_EQ("TRACE_ADDR at +0x024 reads back", cfg_read(VHDD_OFF_TRACE_ADDR),
             0x123u);
    cfg_write(VHDD_OFF_TRACE_ADDR, 0x1FFF);
    CHECK_EQ("TRACE_ADDR truncates to the ring's 12 bits",
             cfg_read(VHDD_OFF_TRACE_ADDR), 0xFFFu);

    // TRACE_DATA must NOT auto-increment: the RAM read is registered, so
    // an auto-increment would hand back the previous index and every dump
    // would be silently off by one.
    cfg_write(VHDD_OFF_TRACE_ADDR, 7);
    uint32_t a = cfg_read(VHDD_OFF_TRACE_DATA);
    uint32_t b = cfg_read(VHDD_OFF_TRACE_DATA);
    CHECK_EQ("TRACE_ADDR unchanged by a TRACE_DATA read",
             cfg_read(VHDD_OFF_TRACE_ADDR), 7u);
    CHECK_EQ("TRACE_DATA does not auto-increment", a, b);

    TraceStatus st = trace_status();
    CHECK_TRUE("a fresh ring is not frozen", !st.frozen);
    CHECK_TRUE("a fresh ring has not wrapped", !st.wrapped);
    return ok;
}

// (2) THE test: realistic 53C96 traffic through the real fabric, dumped
// through the real host register map, compared against the bus.
static bool test_realistic_traffic_captured_in_order() {
    reset_all();
    bool ok = true;
    uint16_t payload[8] = {0};
    if (!drive_rom_read6_chunk(payload)) ok = false;
    idle(200);

    // The tracker is single-beat by construction; measure that the real
    // peripheral_bus timing never presents a request it cannot latch.
    CHECK_EQ("no bus beat was dropped by the ring's beat tracker",
             model.drops, 0);

    // Guard against a vacuous pass: if the model itself is thin, the
    // comparison below proves nothing.  The ROM sequence is ~20 register
    // writes plus the 16 pseudo-DMA byte beats plus whatever survives the
    // poll filter, so 30 is a floor with margin, not a fitted number.
    int dma_entries = 0;
    for (uint32_t e : model.ring)
        if ((e >> 30) & 1) dma_entries++;
    std::printf("    bus produced %zu ring entries (%d of them pseudo-DMA)\n",
                model.ring.size(), dma_entries);
    CHECK_TRUE("the bus really produced a substantial trace (>= 30 entries)",
               model.ring.size() >= 30);
    CHECK_TRUE("the bus really produced pseudo-DMA entries (>= 16)",
               dma_entries >= 16);

    model.frozen = true;   // the model stops where the host freezes
    trace_freeze();
    TraceStatus st = trace_status();
    CHECK_TRUE("freeze took (TRACE_CTRL.frozen == 1)", st.frozen);
    CHECK_EQ("wr_ptr matches the number of entries the bus produced",
             st.wrptr, model.wrptr());
    CHECK_EQ("wrapped flag matches the model", st.wrapped ? 1 : 0,
             model.wrapped() ? 1 : 0);

    std::vector<uint32_t> dump = trace_dump(st);
    if (!compare_dump(dump, "realistic traffic")) ok = false;

    // End-to-end value check on the traffic that matters most: the bytes
    // the ring recorded for the pseudo-DMA aperture must be the bytes the
    // CPU actually received over AXI.  This is the check that would catch
    // a tap on the wrong data wire, which no ordering check can.
    std::vector<uint8_t> dma_bytes;
    for (uint32_t e : dump)
        if (((e >> 30) & 1) && !((e >> 31) & 1))
            dma_bytes.push_back((uint8_t)((e >> 18) & 0xff));
    CHECK_TRUE("ring holds at least the 16 pseudo-DMA payload bytes",
               dma_bytes.size() >= 16);
    if (dma_bytes.size() >= 16) {
        // The 16-bit aperture is replayed as two byte beats, high byte
        // first (peripheral_bus's rd_scsi_phase_q split).
        size_t base = dma_bytes.size() - 16;
        for (int beat = 0; beat < 8; ++beat) {
            uint16_t got = (uint16_t)((dma_bytes[base + 2 * beat] << 8) |
                                      dma_bytes[base + 2 * beat + 1]);
            char nm[96];
            std::snprintf(nm, sizeof nm,
                          "recorded pseudo-DMA word %d == what the CPU read",
                          beat);
            CHECK_EQ(nm, got, payload[beat]);
        }
    }
    return ok;
}

// (3) Freeze really stops capture, and re-arm really restarts it -- both
// driven from the host side, across the CDC.
static bool test_freeze_holds_then_rearm_resumes() {
    reset_all();
    bool ok = true;

    reg_w(0x3, 0x01);
    reg_w(0x0, 0x11);
    reg_w(0x1, 0x22);
    idle(50);
    uint32_t before = model.wrptr();
    CHECK_TRUE("setup produced entries", before > 0);

    model.frozen = true;
    trace_freeze();
    TraceStatus st1 = trace_status();
    CHECK_TRUE("frozen flag set", st1.frozen);
    CHECK_EQ("wr_ptr at freeze", st1.wrptr, before);
    std::vector<uint32_t> dump1 = trace_dump(st1);
    if (!compare_dump(dump1, "captured before freeze")) ok = false;

    // Traffic while frozen must be invisible to the ring.
    for (int i = 0; i < 12; i++) reg_w(0x0, (uint8_t)(0x40 + i));
    idle(50);
    TraceStatus st2 = trace_status();
    CHECK_EQ("frozen ring did not advance", st2.wrptr, before);
    CHECK_TRUE("still frozen", st2.frozen);
    std::vector<uint32_t> dump2 = trace_dump(st2);
    if (!compare_dump(dump2, "unchanged while frozen")) ok = false;

    // Re-arm, then prove capture resumed from an empty ring.
    trace_rearm();
    model.clear();
    model.frozen = false;
    TraceStatus st3 = trace_status();
    CHECK_TRUE("re-arm cleared the frozen flag", !st3.frozen);
    CHECK_EQ("re-arm rewound wr_ptr", st3.wrptr, 0u);
    CHECK_TRUE("re-arm cleared wrapped", !st3.wrapped);

    for (int i = 0; i < 6; i++) reg_w(0x0, (uint8_t)(0x70 + i));
    idle(50);
    CHECK_TRUE("model recorded post-rearm traffic", model.ring.size() >= 6);
    model.frozen = true;
    trace_freeze();
    TraceStatus st4 = trace_status();
    CHECK_TRUE("second freeze took", st4.frozen);
    CHECK_EQ("post-rearm wr_ptr", st4.wrptr, model.wrptr());
    std::vector<uint32_t> dump4 = trace_dump(st4);
    if (!compare_dump(dump4, "captured after re-arm")) ok = false;
    return ok;
}

// (4) Wrap.  Pushed through the REAL bus, not by poking the ring: the
// oldest-first reconstruction the host does (start = wr_ptr when wrapped)
// is part of what is under test, and it is only exercised once the ring
// has actually lapped.
static bool test_wrap_through_the_real_bus() {
    reset_all();
    bool ok = true;

    // Writes are never filtered, so each one is exactly one entry.  4100
    // of them laps the 4096-entry ring and leaves it 4 entries in.
    const int N = VHDD_TRACE_DEPTH + 4;
    for (int i = 0; i < N; i++) reg_w(0x0, (uint8_t)(i & 0xff));
    idle(100);

    CHECK_EQ("no bus beat dropped during the wrap run", model.drops, 0);
    CHECK_TRUE("model says the ring lapped", model.wrapped());
    CHECK_EQ("model wr_ptr after the lap", model.wrptr(), 4u);

    model.frozen = true;
    trace_freeze();
    TraceStatus st = trace_status();
    CHECK_TRUE("wrapped flag set on hardware", st.wrapped);
    CHECK_EQ("wr_ptr after the lap", st.wrptr, model.wrptr());
    std::vector<uint32_t> dump = trace_dump(st);
    CHECK_EQ("a wrapped dump is a full ring", dump.size(),
             (uint32_t)VHDD_TRACE_DEPTH);
    if (!compare_dump(dump, "wrapped ring, oldest first")) ok = false;
    return ok;
}

// (5) The poll filter, through the real fabric.  This is the property the
// whole instrument rests on: a driver spinning on one register must not
// scroll the pre-wedge history out of the ring.
static bool test_poll_loop_does_not_scroll_history() {
    reset_all();
    bool ok = true;

    // A distinctive prologue we will insist survives.
    reg_w(0xC, 0x04);
    reg_w(0x4, 0x00);
    reg_w(0x3, 0x01);
    idle(20);
    size_t after_prologue = model.ring.size();
    CHECK_EQ("prologue is 3 writes", after_prologue, 3u);

    // Now spin on an idle status register the way a wedged driver does.
    for (int i = 0; i < 600; i++) (void)reg_r(0x4);
    idle(20);

    CHECK_TRUE("the poll loop added at most a handful of entries",
               model.ring.size() <= after_prologue + 4);

    model.frozen = true;
    trace_freeze();
    TraceStatus st = trace_status();
    CHECK_TRUE("600 identical polls did not lap the ring", !st.wrapped);
    CHECK_EQ("wr_ptr after the poll loop", st.wrptr, model.wrptr());
    std::vector<uint32_t> dump = trace_dump(st);
    if (!compare_dump(dump, "poll loop")) ok = false;

    // The prologue must still be the OLDEST thing in the ring.
    CHECK_TRUE("prologue survived the poll loop", dump.size() >= 3);
    if (dump.size() >= 3) {
        CHECK_EQ("oldest entry is still W reg C = 0x04", strip_ts(dump[0]),
                 (uint32_t)((1u << 31) | (0xCu << 26) | (0x04u << 18)));
    }
    return ok;
}

#define RUN(fn)                                                                \
    do {                                                                       \
        std::printf("=== %s\n", #fn);                                          \
        if (fn()) { n_pass++; std::printf("    PASS\n"); }                      \
        else { n_fail++; std::printf("    FAIL\n"); }                           \
    } while (0)

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; i++)
        if (!std::strcmp(argv[i], "--positive-control")) positive_control = true;

    dut = new Vtb_scsi_trace_pb;

    if (positive_control)
        std::printf("=== POSITIVE CONTROL: cap_en=0, every capture assertion "
                    "MUST fail ===\n");

    RUN(test_host_register_map);
    RUN(test_realistic_traffic_captured_in_order);
    RUN(test_freeze_holds_then_rearm_resumes);
    RUN(test_wrap_through_the_real_bus);
    RUN(test_poll_loop_does_not_scroll_history);

    delete dut;

    // Same exit convention as tb-scsi-trace-ring: the
    // positive-control run must exit 0 only if the assertions actually
    // failed, so a suite that has gone inert is caught.
    if (positive_control) {
        // test_host_register_map does not depend on capture and is
        // expected to keep passing; the four capture tests must not.
        if (n_fail < 4) {
            std::printf("POSITIVE CONTROL DID NOT FAIL (%d of 5 tests failed) "
                        "- the capture assertions are inert.\n", n_fail);
            return 1;
        }
        std::printf("positive control failed as required (%d tests) - "
                    "assertions are live\n", n_fail);
        return 0;
    }

    std::printf("%d passed, %d failed, %d CSR bus errors\n", n_pass, n_fail,
                cfg_errors);
    return (n_fail || cfg_errors) ? 1 : 0;
}
