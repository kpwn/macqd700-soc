// tb_pb_lane_shim.cpp — equivalence driver for the narrowed-payload S1
// (peripheral-bus) clock-domain-crossing.
//
// What this proves
// ────────────────
// `rtl/soc/axi_pb_s1_cdc.v` is a drop-in replacement for
// `axi_async_bridge #(.DATA_WIDTH(128))` on the xbar-S1 -> peripheral_bus
// path.  Internally it is
//     axi_pb_lane_narrow (128->32) -> axi_async_bridge #(32)
//                                  -> axi_pb_lane_widen (32->128)
// so only 32 payload bits actually cross core_clk -> pb_clk.  This
// testbench runs TWO structurally parallel chains (tb/tb_pb_lane_shim.v):
//
//   REF:  wide face -> axi_async_bridge #(128) -> pb_lane_slave_model
//   DUT:  wide face -> axi_pb_s1_cdc  #(128)   -> pb_lane_slave_model
//
// and asserts that, transaction for transaction, the DUT chain is
// behaviourally identical to the REF chain.
//
// `pb_lane_slave_model` (in the .v) is peripheral_bus.v's byte-lane
// policy distilled: lane = addr[3:2] (NOT the strobe), 4:1 wdata/wstrb
// muxes, and reads that drive the selected lane with the other three
// HARD ZERO — which is precisely what `axi_pb_lane_widen`'s 4-way
// OR-reduce depends on.
//
// Stimulus (identical stream into both chains, replayed from one
// `txs[]` list so the two can never diverge):
//   * single beat only (AWLEN = ARLEN = 0) — S1 is burst-free by
//     construction (axi_xbar's `is_lite_only_slv`), so multi-beat is
//     out of scope;
//   * random addresses across a 4 KB window, random lane (addr[3:2]),
//     random byte / halfword / word strobes WITHIN the addressed lane
//     (this is exactly how axi_narrow_to_wide places data — the strobed
//     lane always equals addr[3:2]), random IDs, random data;
//   * the three UNADDRESSED 128-bit lanes are filled with random
//     garbage on every write, so any accidental dependence on them
//     shows up immediately;
//   * random master-side pre-delays and random BREADY/RREADY stalls;
//   * the slave models add pseudo-random response latency of their own.
//
// Directed coverage on top of the randomized stream: all four lanes,
// each of the four single-byte strobes within each lane, halfword
// (0b0011 / 0b1100) and word (0b1111) strobes, back-to-back
// same-address write-then-read, and an all-zero-WSTRB write (must be a
// no-op on both chains).
//
// Checks
// ──────
//   * BRESP / BID equal between chains (and BID == the issued AWID);
//   * RRESP / RID equal between chains (and RID == the issued ARID);
//   * read data IN THE ADDRESSED LANE equal between chains, bit for bit,
//     and equal to a host-side golden model;
//   * DUT RDATA == {4{addressed lane word}} — the replication itself;
//   * REF RDATA has hard zeroes in the three unaddressed lanes (the
//     property axi_pb_lane_widen relies on);
//   * both slave models' captured write memories are byte-identical at
//     the end of the run, and match the golden model;
//   * AWSIZE/ARSIZE seen at the slave: pass-through for size <= 2,
//     clamped to 2 above it (see below).
//
// KNOWN INTENTIONAL DIFFERENCES (asserted, not flagged as failures)
// ─────────────────────────────────────────────────────────────────
//   1. Read lane replication.  REF returns zeroes in the three
//      unselected 128-bit lanes; DUT replicates the word into all four.
//      Every master on this path selects lane addr[3:2], so this is
//      unobservable.  Compare the ADDRESSED LANE, and separately assert
//      the replication.
//   2. AWSIZE/ARSIZE clamp.  axi_pb_lane_narrow clamps SIZE to 3'd2
//      because the CDC payload is 32 bits.  peripheral_bus only ever
//      tests `size == 3'd1`, and sizes 0/1/2 survive the clamp
//      untouched, so this too is unobservable downstream.  Asserted
//      explicitly via the slave models' *_dbg_last_a?size taps.
//
// PRECONDITION NOT COVERED HERE (see the report / axi_pb_lane_shim.v
// header): `axi_pb_lane_narrow` recovers the write lane from WSTRB with
// a LOWEST-set-lane priority encoder, not from AWADDR[3:2].  That is
// only equivalent while exactly ONE 32-bit lane is strobed per beat,
// which is what axi_narrow_to_wide (and therefore every master on S1)
// produces, and what this tb drives.  A beat that strobed two lanes —
// even if one of them IS the address lane — diverges: the old 128-bit
// path took the ADDRESS lane, the shim takes the LOWEST strobed lane.
// Verified by temporarily forcing WSTRB=0x0F0F on an AWADDR-lane-2
// write: REF stored lane 2's word, DUT stored lane 0's.  Nothing on S1
// generates such a beat today; there is no RTL assertion enforcing it.
//
// Clocking: s_clk 100 MHz (core), m_clk 50 MHz (pb) — 2:1, with the
// m_clk edges deliberately offset so they never coincide with an s_clk
// edge, keeping the async_fifo gray-pointer synchronisers honest.
//
// Style follows tb/tb_l2c_chain.cpp.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <vector>

#include <verilated.h>
#include "Vtb_pb_lane_shim.h"
using DutT = Vtb_pb_lane_shim;

static DutT*    dut       = nullptr;
static uint64_t core_cyc  = 0;
static int      n_pass    = 0;
static int      n_fail    = 0;

static std::mt19937 rng(0xB1A5'CDCu);

// Chain selectors.
static constexpr int REF = 0;
static constexpr int DUT = 1;
static const char* CH_NAME[2] = {"REF", "DUT"};

// Port accessor: both chains' ports are the same Verilator type, so the
// conditional expression is an lvalue and works for reads and writes.
#define PORT(c, name) ((c) == REF ? dut->ref_##name : dut->dut_##name)

static constexpr uint32_t MEM_WORDS = 1024;                 // 4 KB window
static constexpr uint32_t BASE      = 0x5000'0000u;
static constexpr uint32_t WIN_BYTES = MEM_WORDS * 4;

// ─── check helpers ────────────────────────────────────────────────────
static void check(bool ok, const char* what, const char* detail = nullptr) {
    if (ok) {
        n_pass++;
    } else {
        n_fail++;
        if (n_fail <= 40)
            printf("  FAIL: %s%s%s\n", what,
                   detail ? " — " : "", detail ? detail : "");
    }
}

// ─── transaction stream ───────────────────────────────────────────────
struct Tx {
    bool     is_write;
    uint32_t addr;        // byte address (BASE + offset, offset < 4 KB)
    uint8_t  id;          // 6 bits
    uint8_t  size;        // AxSIZE as issued
    uint8_t  strb4;       // strobes WITHIN the addressed lane
    uint32_t lane_data;   // the 32-bit word placed in the addressed lane
    uint32_t fill[4];     // full 128-bit W payload (garbage in other lanes)
    uint8_t  pre_delay;   // master-side idle cycles before asserting valid
};

static std::vector<Tx>       txs;
static std::vector<uint32_t> exp_rdata;   // per-tx expected read word
static std::vector<uint32_t> golden(MEM_WORDS, 0);

static inline uint32_t lane_of(uint32_t addr) { return (addr >> 2) & 3u; }
static inline uint32_t idx_of (uint32_t addr) { return (addr >> 2) & (MEM_WORDS - 1); }

static void push_write(uint32_t addr, uint8_t strb4, uint32_t data,
                       uint8_t size = 2) {
    Tx t{};
    t.is_write  = true;
    t.addr      = addr;
    t.id        = static_cast<uint8_t>(rng() & 0x3Fu);
    t.size      = size;
    t.strb4     = static_cast<uint8_t>(strb4 & 0xFu);
    t.lane_data = data;
    for (int l = 0; l < 4; l++) t.fill[l] = rng();      // garbage lanes
    t.fill[lane_of(addr)] = data;                       // real lane
    t.pre_delay = static_cast<uint8_t>(rng() % 4);
    txs.push_back(t);
}

static void push_read(uint32_t addr, uint8_t size = 2) {
    Tx t{};
    t.is_write  = false;
    t.addr      = addr;
    t.id        = static_cast<uint8_t>(rng() & 0x3Fu);
    t.size      = size;
    t.pre_delay = static_cast<uint8_t>(rng() % 4);
    txs.push_back(t);
}

// Replay the stream over a host-side shadow memory so every read has a
// precomputed expected value and the end-of-run memory image is known.
static void build_golden() {
    exp_rdata.assign(txs.size(), 0);
    for (size_t i = 0; i < txs.size(); i++) {
        const Tx& t = txs[i];
        uint32_t  w = idx_of(t.addr);
        if (t.is_write) {
            uint32_t cur = golden[w];
            for (int b = 0; b < 4; b++) {
                if (t.strb4 & (1u << b)) {
                    cur &= ~(0xFFu << (b * 8));
                    cur |= (t.lane_data & (0xFFu << (b * 8)));
                }
            }
            golden[w] = cur;
        } else {
            exp_rdata[i] = golden[w];
        }
    }
}

// ─── per-chain results ────────────────────────────────────────────────
struct Result {
    bool     done   = false;
    uint8_t  id     = 0;
    uint8_t  resp   = 0;
    uint32_t rdata[4] = {0, 0, 0, 0};
    uint8_t  slv_size = 0;   // AxSIZE observed at the slave model
};
static std::vector<Result> results[2];

// ─── master BFM state ─────────────────────────────────────────────────
struct Chain {
    size_t   idx      = 0;
    int      pre      = -1;   // -1 => not yet armed for this tx
    bool     aw_done  = false;
    bool     w_done   = false;
    bool     ar_done  = false;
    std::mt19937 srng;
    uint64_t last_progress = 0;
};
static Chain ch[2];
static bool  g_bfm_enable = false;   // held off until the CDC's reset
                                     // handshake has settled

static void drive_idle(int c) {
    PORT(c, awvalid) = 0;
    PORT(c, wvalid)  = 0;
    PORT(c, arvalid) = 0;
}

static void bfm_drive(int c) {
    Chain& s = ch[c];
    if (!g_bfm_enable) {
        drive_idle(c);
        PORT(c, bready) = 0;
        PORT(c, rready) = 0;
        return;
    }
    if (s.idx >= txs.size()) {
        drive_idle(c);
        PORT(c, bready) = 1;
        PORT(c, rready) = 1;
        return;
    }
    const Tx& t = txs[s.idx];

    // Random response-channel stalls.
    PORT(c, bready) = (s.srng() & 3u) != 0;
    PORT(c, rready) = (s.srng() & 3u) != 0;

    if (s.pre != 0) {                // not yet armed / still in the gap
        drive_idle(c);
        return;
    }

    PORT(c, awlen)   = 0;
    PORT(c, arlen)   = 0;
    PORT(c, awburst) = 1;            // INCR
    PORT(c, arburst) = 1;
    PORT(c, awsize)  = t.size;
    PORT(c, arsize)  = t.size;
    PORT(c, wlast)   = 1;

    if (t.is_write) {
        PORT(c, arvalid) = 0;
        PORT(c, awid)    = t.id;
        PORT(c, awaddr)  = t.addr;
        PORT(c, awvalid) = !s.aw_done;
        for (int l = 0; l < 4; l++) PORT(c, wdata)[l] = t.fill[l];
        PORT(c, wstrb)  = static_cast<uint16_t>(t.strb4 << (lane_of(t.addr) * 4));
        PORT(c, wvalid) = !s.w_done;
    } else {
        PORT(c, awvalid) = 0;
        PORT(c, wvalid)  = 0;
        PORT(c, arid)    = t.id;
        PORT(c, araddr)  = t.addr;
        PORT(c, arvalid) = !s.ar_done;
    }
}

// Pre-edge: observe handshakes with the drives that the RTL is about to
// latch, and advance the software state.  Writes no ports.
static void bfm_capture(int c) {
    Chain& s = ch[c];
    if (!g_bfm_enable) return;
    if (s.idx >= txs.size()) return;
    const Tx& t = txs[s.idx];

    if (s.pre < 0) s.pre = t.pre_delay;     // arm on first visit
    if (s.pre > 0) { s.pre--; return; }

    if (t.is_write) {
        if (PORT(c, awvalid) && PORT(c, awready)) { s.aw_done = true; s.last_progress = core_cyc; }
        if (PORT(c, wvalid)  && PORT(c, wready))  { s.w_done  = true; s.last_progress = core_cyc; }
        if (PORT(c, bvalid)  && PORT(c, bready)) {
            Result& r  = results[c][s.idx];
            r.done     = true;
            r.id       = PORT(c, bid);
            r.resp     = PORT(c, bresp);
            r.slv_size = PORT(c, dbg_last_awsize);
            s.idx++; s.pre = -1;
            s.aw_done = s.w_done = s.ar_done = false;
            s.last_progress = core_cyc;
        }
    } else {
        if (PORT(c, arvalid) && PORT(c, arready)) { s.ar_done = true; s.last_progress = core_cyc; }
        if (PORT(c, rvalid)  && PORT(c, rready)) {
            Result& r  = results[c][s.idx];
            r.done     = true;
            r.id       = PORT(c, rid);
            r.resp     = PORT(c, rresp);
            for (int l = 0; l < 4; l++) r.rdata[l] = PORT(c, rdata)[l];
            r.slv_size = PORT(c, dbg_last_arsize);
            s.idx++; s.pre = -1;
            s.aw_done = s.w_done = s.ar_done = false;
            s.last_progress = core_cyc;
        }
    }
}

// ─── clocking: 8 sub-steps per pb period ──────────────────────────────
//   s_clk (core, 100 MHz): posedge at sub 0 and sub 4
//   m_clk (pb,    50 MHz): posedge at sub 1, negedge at sub 5
// The half-sub offset keeps m_clk edges away from s_clk edges.
static int subphase = 0;

static void substep() {
    const bool s_pos = (subphase == 0 || subphase == 4);
    if (s_pos) { bfm_capture(REF); bfm_capture(DUT); }
    dut->s_clk = ((subphase % 4) < 2) ? 1 : 0;
    dut->m_clk = (subphase >= 1 && subphase < 5) ? 1 : 0;
    dut->eval();
    if (s_pos) {
        core_cyc++;
        bfm_drive(REF); bfm_drive(DUT);
        dut->eval();
    }
    subphase = (subphase + 1) & 7;
}

static void run_cycles(uint64_t n) { for (uint64_t i = 0; i < n * 8; i++) substep(); }

// ─── stimulus construction ────────────────────────────────────────────
static uint32_t g_next_block = 0;
// Hand out a fresh 16-byte-aligned block so directed cases never alias.
static uint32_t fresh_block() {
    uint32_t off = (g_next_block * 16) % WIN_BYTES;
    g_next_block++;
    return BASE + off;
}

static void build_stimulus(int n_random) {
    // ── D1: all four lanes, full-word strobe, write then read back ───
    for (uint32_t lane = 0; lane < 4; lane++) {
        uint32_t a = fresh_block() + lane * 4;
        push_write(a, 0xF, 0xDEADBE00u | lane);
        push_read(a);                       // back-to-back same address
    }

    // ── D2: each of the four byte strobes, in each lane ──────────────
    for (uint32_t lane = 0; lane < 4; lane++) {
        for (int b = 0; b < 4; b++) {
            uint32_t a = fresh_block() + lane * 4;
            push_write(a, 0xF, 0x00000000u);          // clear
            push_write(a, static_cast<uint8_t>(1u << b), 0xA1B2C3D4u);
            push_read(a);
        }
    }

    // ── D3: halfword + word strobes, in each lane ────────────────────
    const uint8_t multi[3] = {0x3, 0xC, 0xF};
    for (uint32_t lane = 0; lane < 4; lane++) {
        for (int m = 0; m < 3; m++) {
            uint32_t a = fresh_block() + lane * 4;
            push_write(a, 0xF, 0xFFFFFFFFu);
            push_write(a, multi[m], 0x12345678u);
            push_read(a);
        }
    }

    // ── D4: all-zero WSTRB write must be a no-op, in each lane ───────
    for (uint32_t lane = 0; lane < 4; lane++) {
        uint32_t a = fresh_block() + lane * 4;
        push_write(a, 0xF, 0xAAAAAAAAu);
        push_read(a);
        push_write(a, 0x0, 0x55555555u);    // no-op on both chains
        push_read(a);                       // must still read 0xAAAAAAAA
    }

    // ── D5: AxSIZE clamp coverage (0..4) ─────────────────────────────
    for (uint8_t sz = 0; sz <= 4; sz++) {
        uint32_t a = fresh_block() + (sz & 3u) * 4;
        push_write(a, 0xF, 0x5A5A0000u | sz, sz);
        push_read(a, sz);
    }

    // ── R: randomized single-beat stream ─────────────────────────────
    const uint8_t strb_pool[9] = {0x1, 0x2, 0x4, 0x8, 0x3, 0xC, 0xF, 0x6, 0x0};
    for (int i = 0; i < n_random; i++) {
        uint32_t off  = (rng() % (WIN_BYTES / 4)) * 4;   // word aligned
        uint32_t addr = BASE + off;
        uint8_t  sz   = static_cast<uint8_t>(rng() % 5);
        if (rng() & 1u) {
            push_write(addr, strb_pool[rng() % 9], rng(), sz);
        } else {
            push_read(addr, sz);
        }
    }
}

// ─── main ─────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new DutT;

    const int n_random = 5000;
    build_stimulus(n_random);
    build_golden();
    results[REF].assign(txs.size(), Result{});
    results[DUT].assign(txs.size(), Result{});

    printf("tb_pb_lane_shim: S1 narrowed-payload CDC equivalence\n");
    printf("  REF: axi_async_bridge #(128) -> pb_lane_slave_model\n");
    printf("  DUT: axi_pb_s1_cdc  #(128)   -> pb_lane_slave_model\n");
    printf("  transactions: %zu (%d randomized + directed), "
           "s_clk:m_clk = 2:1\n", txs.size(), n_random);

    ch[REF].srng.seed(0x5EED'0001u);
    ch[DUT].srng.seed(0x5EED'0002u);

    // ── reset both domains ───────────────────────────────────────────
    dut->s_rst = 1; dut->m_rst = 1;
    dut->dbg_idx = 0;
    drive_idle(REF); drive_idle(DUT);
    PORT(REF, bready) = 0; PORT(REF, rready) = 0;
    PORT(DUT, bready) = 0; PORT(DUT, rready) = 0;
    dut->eval();
    for (int i = 0; i < 40 * 8; i++) {
        dut->s_clk = ((subphase % 4) < 2) ? 1 : 0;
        dut->m_clk = (subphase >= 1 && subphase < 5) ? 1 : 0;
        dut->eval();
        subphase = (subphase + 1) & 7;
    }
    dut->s_rst = 0; dut->m_rst = 0;
    dut->eval();
    bfm_drive(REF); bfm_drive(DUT);
    dut->eval();

    // Let the bridge's coupled-reset handshake settle before traffic.
    run_cycles(64);
    g_bfm_enable = true;
    bfm_drive(REF); bfm_drive(DUT);
    dut->eval();

    // ── run ──────────────────────────────────────────────────────────
    const uint64_t CYCLE_CAP = 4'000'000;
    bool timed_out = false;
    while (ch[REF].idx < txs.size() || ch[DUT].idx < txs.size()) {
        run_cycles(1);
        if (core_cyc > CYCLE_CAP) { timed_out = true; break; }
    }

    if (timed_out) {
        printf("  FAIL: timeout after %llu core cycles "
               "(REF at tx %zu/%zu, DUT at tx %zu/%zu)\n",
               static_cast<unsigned long long>(core_cyc),
               ch[REF].idx, txs.size(), ch[DUT].idx, txs.size());
        n_fail++;
    }

    // ── per-transaction equivalence ──────────────────────────────────
    size_t n_wr = 0, n_rd = 0;
    char buf[256];
    for (size_t i = 0; i < txs.size(); i++) {
        const Tx& t = txs[i];
        const Result& r = results[REF][i];
        const Result& d = results[DUT][i];
        if (!r.done || !d.done) {
            snprintf(buf, sizeof buf,
                     "tx %zu never completed (ref=%d dut=%d)",
                     i, (int)r.done, (int)d.done);
            check(false, "completion", buf);
            continue;
        }

        // Response equivalence.
        if (r.id != d.id) {
            snprintf(buf, sizeof buf, "tx %zu %cID ref=0x%02x dut=0x%02x",
                     i, t.is_write ? 'B' : 'R', r.id, d.id);
            check(false, "id mismatch", buf);
        } else if (r.id != t.id) {
            snprintf(buf, sizeof buf, "tx %zu %cID=0x%02x issued=0x%02x",
                     i, t.is_write ? 'B' : 'R', r.id, t.id);
            check(false, "id not echoed", buf);
        } else {
            n_pass++;
        }
        if (r.resp != d.resp || r.resp != 0) {
            snprintf(buf, sizeof buf, "tx %zu %cRESP ref=%u dut=%u",
                     i, t.is_write ? 'B' : 'R', (unsigned)r.resp,
                     (unsigned)d.resp);
            check(false, "resp mismatch", buf);
        } else {
            n_pass++;
        }

        // AxSIZE: pass-through <= 2, clamped to 2 above (intentional).
        uint8_t want_dut = t.size > 2 ? 2 : t.size;
        if (r.slv_size != t.size || d.slv_size != want_dut) {
            snprintf(buf, sizeof buf,
                     "tx %zu size issued=%u ref_slv=%u dut_slv=%u (want %u)",
                     i, (unsigned)t.size, (unsigned)r.slv_size,
                     (unsigned)d.slv_size, (unsigned)want_dut);
            check(false, "AxSIZE", buf);
        } else {
            n_pass++;
        }

        if (t.is_write) { n_wr++; continue; }
        n_rd++;

        const uint32_t lane = lane_of(t.addr);
        const uint32_t rw   = r.rdata[lane];
        const uint32_t dw   = d.rdata[lane];

        // The addressed lane must match REF bit for bit, and the golden.
        if (rw != exp_rdata[i]) {
            snprintf(buf, sizeof buf,
                     "tx %zu addr=0x%08x lane=%u ref=0x%08x golden=0x%08x",
                     i, t.addr, lane, rw, exp_rdata[i]);
            check(false, "REF read data vs golden", buf);
        } else {
            n_pass++;
        }
        if (dw != rw) {
            snprintf(buf, sizeof buf,
                     "tx %zu addr=0x%08x lane=%u ref=0x%08x dut=0x%08x",
                     i, t.addr, lane, rw, dw);
            check(false, "addressed-lane read data", buf);
        } else {
            n_pass++;
        }

        // REF must hard-zero the unaddressed lanes (the property
        // axi_pb_lane_widen's OR-reduce depends on).
        bool ref_zeroed = true;
        for (uint32_t l = 0; l < 4; l++)
            if (l != lane && r.rdata[l] != 0) ref_zeroed = false;
        if (!ref_zeroed) {
            snprintf(buf, sizeof buf,
                     "tx %zu ref rdata=%08x_%08x_%08x_%08x lane=%u",
                     i, r.rdata[3], r.rdata[2], r.rdata[1], r.rdata[0], lane);
            check(false, "REF unaddressed lanes not zero", buf);
        } else {
            n_pass++;
        }

        // DUT replicates the word into all four lanes — verify it.
        bool replicated = true;
        for (uint32_t l = 0; l < 4; l++)
            if (d.rdata[l] != dw) replicated = false;
        if (!replicated) {
            snprintf(buf, sizeof buf,
                     "tx %zu dut rdata=%08x_%08x_%08x_%08x want {4{0x%08x}}",
                     i, d.rdata[3], d.rdata[2], d.rdata[1], d.rdata[0], dw);
            check(false, "DUT lane replication", buf);
        } else {
            n_pass++;
        }
    }

    // ── end-of-run memory equivalence ────────────────────────────────
    int mem_diff = 0, mem_golden_diff = 0;
    for (uint32_t w = 0; w < MEM_WORDS; w++) {
        dut->dbg_idx = static_cast<uint16_t>(w);
        dut->eval();
        uint32_t rv = dut->ref_dbg_data;
        uint32_t dv = dut->dut_dbg_data;
        if (rv != dv) {
            if (mem_diff < 8)
                printf("  FAIL: write memory word %u ref=0x%08x dut=0x%08x\n",
                       w, rv, dv);
            mem_diff++;
        }
        if (rv != golden[w]) {
            if (mem_golden_diff < 8)
                printf("  FAIL: write memory word %u ref=0x%08x golden=0x%08x\n",
                       w, rv, golden[w]);
            mem_golden_diff++;
        }
    }
    check(mem_diff == 0, "slave write memories identical (REF vs DUT)");
    check(mem_golden_diff == 0, "slave write memory matches golden model");

    printf("  summary: %zu transactions (%zu writes / %zu reads) in %llu "
           "core cycles; %u memory words compared; "
           "%d memory diffs REF-vs-DUT, %d vs golden\n",
           txs.size(), n_wr, n_rd,
           static_cast<unsigned long long>(core_cyc), MEM_WORDS,
           mem_diff, mem_golden_diff);
    (void)CH_NAME;

    printf("tb_pb_lane_shim: %d PASS / %d FAIL\n", n_pass, n_fail);
    dut->final();
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
