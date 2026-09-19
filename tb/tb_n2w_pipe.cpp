// tb_n2w_pipe.cpp — pipelined-master driver for axi_narrow_to_wide.
//
// Two jobs.
//
// 1. THE DEPTH CURVE.  Every other harness that drives this adapter is a
//    single-outstanding master, so none of them can price the difference
//    between "one transaction at a time" and "N in flight".  This one
//    keeps the narrow side saturated and reports narrow-side cycles per
//    32-bit word against a wide slave with a configurable, genuinely
//    multi-outstanding response latency.  Run the same source at
//    DEPTH = 1/2/4/8 (the Makefile builds all four) and the curve falls
//    out; the shipping WR_OUTSTANDING / RD_OUTSTANDING were chosen from
//    it and the numbers are reproduced in axi_narrow_to_wide.v's header.
//
// 2. THE DIRECTED MULTI-OUTSTANDING SCENARIOS.  Correctness properties
//    that only exist once more than one transaction can be in flight:
//
//      * several writes in flight, completing against a slave that
//        answers them at staggered times — B count, B order and the wide
//        payload of each must all be right;
//      * a read and a write in flight simultaneously, neither disturbing
//        the other's response;
//      * MIXED BURST LENGTHS back to back, which is the shape that would
//        expose partially-gathered beats from two transactions being
//        merged into one wide beat — the hazard the front end's
//        one-transaction-at-a-time rule exists to prevent;
//      * the abandonment watchdog firing with N transactions outstanding:
//        N SLVERR Bs for N writes, and per-transaction SLVERR R bursts
//        with RLAST at each transaction's own boundary rather than one
//        long run with a single RLAST (which would strand the master on
//        every read but the first).
//
//    At DEPTH = 1 the multi-outstanding scenarios are skipped and the
//    build instead pins that the historical single-outstanding contract
//    is exactly reproducible: n_awready stays low until B.
//
// Built via: make tb-n2w-pipe

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <array>
#include <deque>
#include <string>
#include <vector>
#include <verilated.h>
#include "Vtb_n2w_pipe.h"

#ifndef N2W_PIPE_DEPTH
#define N2W_PIPE_DEPTH 4
#endif
static const int kDepth = N2W_PIPE_DEPTH;

// Wide-side response latency, in cycles, from the accepting AW/AR
// handshake to the first B/R beat.  60 is what the shipping chain
// (n2w -> l2c -> async bridge -> MIG bridge -> DDR) measures and what
// tb_sd_boot_top's own latency model uses for its headline number, so
// the curve below is comparable to the boot pre-zero figures.
static const int kLat = 60;

static Vtb_n2w_pipe* dut = nullptr;
static int n_pass = 0, n_fail = 0;
static uint64_t now = 0;

using Word128 = std::array<uint32_t, 4>;

static void check(const char* name, bool ok, const std::string& detail = "") {
    if (ok) { n_pass++; printf("  PASS: %s\n", name); }
    else    { n_fail++; printf("  FAIL: %s — %s\n", name, detail.c_str()); }
}
static std::string dec(long long v) { return std::to_string(v); }
static std::string hex32(uint32_t v) {
    char b[16]; snprintf(b, sizeof b, "0x%08x", v); return b;
}

// ── Wide-slave model ─────────────────────────────────────────────────
// Multi-outstanding on purpose: the point of the DUT change is that it
// can now keep several transactions in flight, and a slave that
// serialized them would hide exactly the effect being measured.  AXI
// same-ID ordering is respected (both queues are FIFOs), which is what
// the DUT relies on to match responses positionally.
struct WrTxn {
    uint32_t addr; int len; int beats_seen; uint64_t due;
    std::vector<Word128> data; std::vector<uint32_t> strb;
};
struct RdTxn { uint32_t addr; int len; int size; uint64_t due; int beats_sent; };

static std::deque<WrTxn> wr_q;      // AW accepted, W not finished
static std::deque<WrTxn> b_q;       // W finished, B owed
static std::deque<RdTxn> rd_q;      // AR accepted, R owed
static bool     slave_stalled = false;   // freeze the wide side entirely
// Backpressure the wide W channel ONLY (AW/AR/B/R keep running).  This is
// what exercises the gather bank's park/transfer path: the output bank
// cannot be emptied, so a completed group has nowhere to go.
static bool     wide_w_stalled = false;
static uint64_t wide_b_count = 0, wide_r_beats = 0;
static std::vector<WrTxn> wr_done;       // completed writes, in B order

// Narrow-side logs.
struct BBeat { int resp; uint64_t t; };
struct RBeat { uint32_t data; int resp; int last; uint64_t t; };
static std::vector<BBeat> b_log;
static std::vector<RBeat> r_log;

static Word128 wide_wdata() {
    Word128 w{};
    // Verilator exposes a 128-bit port as a 4-word array.
    for (int i = 0; i < 4; i++) w[i] = dut->w_wdata[i];
    return w;
}

// Drive every wide-side input for this cycle, then evaluate.  Called
// once per cycle before the clock edge.
static void slave_pre() {
    if (slave_stalled) {
        dut->w_awready = 0; dut->w_wready = 0; dut->w_arready = 0;
        dut->w_bvalid  = 0; dut->w_rvalid  = 0;
        dut->eval();
        return;
    }
    dut->w_awready = 1;
    dut->w_wready  = wide_w_stalled ? 0 : 1;
    dut->w_arready = 1;

    // B: present the head of b_q once its latency has elapsed.
    if (!b_q.empty() && now >= b_q.front().due) {
        dut->w_bvalid = 1; dut->w_bresp = 0; dut->w_bid = 0;
    } else {
        dut->w_bvalid = 0;
    }

    // R: stream the head of rd_q once its latency has elapsed.
    if (!rd_q.empty() && now >= rd_q.front().due) {
        RdTxn& t = rd_q.front();
        for (int i = 0; i < 4; i++)
            dut->w_rdata[i] = t.addr + (uint32_t)(t.beats_sent * 16 + i * 4);
        dut->w_rresp = 0;
        dut->w_rlast = (t.beats_sent == t.len) ? 1 : 0;
        dut->w_rvalid = 1;
    } else {
        dut->w_rvalid = 0;
    }
    dut->eval();
}

// Observe the handshakes this cycle resolved to, then advance the clock.
static void slave_post_and_tick() {
    if (!slave_stalled) {
        if (dut->w_awvalid && dut->w_awready) {
            WrTxn t{}; t.addr = dut->w_awaddr; t.len = dut->w_awlen;
            t.beats_seen = 0; t.due = 0;
            wr_q.push_back(t);
        }
        if (dut->w_wvalid && dut->w_wready) {
            // W bursts arrive in AW order; the head of wr_q that still
            // owes beats is the one this belongs to.
            if (!wr_q.empty()) {
                WrTxn& t = wr_q.front();
                t.data.push_back(wide_wdata());
                t.strb.push_back(dut->w_wstrb);
                t.beats_seen++;
                if (dut->w_wlast) {
                    t.due = now + kLat;
                    b_q.push_back(t);
                    wr_q.pop_front();
                }
            }
        }
        if (dut->w_bvalid && dut->w_bready) {
            wide_b_count++;
            if (!b_q.empty()) { wr_done.push_back(b_q.front()); b_q.pop_front(); }
        }
        if (dut->w_arvalid && dut->w_arready) {
            RdTxn t{}; t.addr = dut->w_araddr; t.len = dut->w_arlen;
            t.size = dut->w_arsize; t.due = now + kLat; t.beats_sent = 0;
            rd_q.push_back(t);
        }
        if (dut->w_rvalid && dut->w_rready) {
            wide_r_beats++;
            if (!rd_q.empty()) {
                RdTxn& t = rd_q.front();
                t.beats_sent++;
                if (t.beats_sent > t.len) rd_q.pop_front();
            }
        }
    }
    // Narrow-side response logging.
    if (dut->n_bvalid && dut->n_bready)
        b_log.push_back({(int)dut->n_bresp, now});
    if (dut->n_rvalid && dut->n_rready)
        r_log.push_back({(uint32_t)dut->n_rdata, (int)dut->n_rresp,
                         (int)dut->n_rlast, now});

    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    now++;
}

static void cyc() { slave_pre(); slave_post_and_tick(); }
static void run(int n) { for (int i = 0; i < n; i++) cyc(); }

static void clear_state() {
    wr_q.clear(); b_q.clear(); rd_q.clear(); wr_done.clear();
    b_log.clear(); r_log.clear();
    wide_b_count = 0; wide_r_beats = 0;
    slave_stalled = false;
    wide_w_stalled = false;
}

static void reset() {
    clear_state();
    dut->rst = 1;
    dut->n_awvalid = 0; dut->n_wvalid = 0; dut->n_arvalid = 0;
    dut->n_bready  = 1; dut->n_rready  = 1;
    dut->n_awlen = 0; dut->n_arlen = 0; dut->n_arsize = 2;
    dut->n_wstrb = 0xF; dut->n_wlast = 1;
    dut->w_bresp = 0; dut->w_rresp = 0; dut->w_bid = 0; dut->w_rid = 0;
    run(4);
    dut->rst = 0;
    run(4);
    clear_state();
}

// ═══════════════════════════════════════════════════════════════════════
// Pipelined narrow master
// ═══════════════════════════════════════════════════════════════════════
//
// Presents AW as early as the adapter will take it and streams W beats
// behind it, never waiting for B.  This is the shape the D-cache
// writeback path and boot_fsm's RAM pre-zero pass both have available to
// them and could not use while the adapter was single-outstanding.
struct WrPerf { double cyc_per_word; uint64_t cycles; uint64_t words; };

static WrPerf stream_writes(uint32_t base, int awlen, int nburst) {
    const int beats = awlen + 1;
    const uint64_t t0 = now;
    int aw_issued = 0, w_burst = 0, w_beat = 0;
    uint64_t b_seen = 0;
    uint32_t pattern = 0x1000'0000u;
    int guard = 0;

    while ((int)b_seen < nburst) {
        // AW channel: offer the next burst whenever one is left.
        bool aw_off = (aw_issued < nburst);
        dut->n_awvalid = aw_off;
        dut->n_awaddr  = base + (uint32_t)aw_issued * (uint32_t)beats * 4u;
        dut->n_awlen   = (uint8_t)awlen;
        // W channel: stream beats for whichever burst is next in order.
        bool w_off = (w_burst < nburst);
        dut->n_wvalid = w_off;
        dut->n_wdata  = pattern;
        dut->n_wstrb  = 0xF;
        dut->n_wlast  = (w_beat + 1 == beats) ? 1 : 0;
        dut->n_bready = 1;

        slave_pre();
        const bool aw_hs = aw_off && dut->n_awready;
        const bool w_hs  = w_off  && dut->n_wready;
        const bool b_hs  = dut->n_bvalid && dut->n_bready;
        slave_post_and_tick();

        if (aw_hs) aw_issued++;
        if (w_hs) {
            pattern++;
            if (++w_beat == beats) { w_beat = 0; w_burst++; }
        }
        if (b_hs) b_seen++;
        if (++guard > 20'000'000) { printf("  HANG in stream_writes\n"); n_fail++; break; }
    }
    dut->n_awvalid = 0; dut->n_wvalid = 0;
    const uint64_t cycles = now - t0;
    const uint64_t words  = (uint64_t)nburst * (uint64_t)beats;
    return { words ? (double)cycles / (double)words : 0.0, cycles, words };
}

static WrPerf stream_reads(uint32_t base, int arlen, int nburst) {
    const int beats = arlen + 1;
    const uint64_t t0 = now;
    int ar_issued = 0;
    uint64_t beats_seen = 0;
    int guard = 0;

    while (beats_seen < (uint64_t)nburst * (uint64_t)beats) {
        bool ar_off = (ar_issued < nburst);
        dut->n_arvalid = ar_off;
        dut->n_araddr  = base + (uint32_t)ar_issued * (uint32_t)beats * 4u;
        dut->n_arlen   = (uint8_t)arlen;
        dut->n_arsize  = 2;
        dut->n_rready  = 1;

        slave_pre();
        const bool ar_hs = ar_off && dut->n_arready;
        const bool r_hs  = dut->n_rvalid && dut->n_rready;
        slave_post_and_tick();

        if (ar_hs) ar_issued++;
        if (r_hs)  beats_seen++;
        if (++guard > 20'000'000) { printf("  HANG in stream_reads\n"); n_fail++; break; }
    }
    dut->n_arvalid = 0;
    const uint64_t cycles = now - t0;
    const uint64_t words  = (uint64_t)nburst * (uint64_t)beats;
    return { words ? (double)cycles / (double)words : 0.0, cycles, words };
}

// ── Single-shot helpers for the directed scenarios ───────────────────
// Offer one AW (and, for a burst, its W beats) without ever waiting for
// B, so several can be stacked up.
static bool offer_aw(uint32_t addr, int len, int timeout = 32) {
    for (int c = 0; c < timeout; c++) {
        dut->n_awvalid = 1; dut->n_awaddr = addr; dut->n_awlen = (uint8_t)len;
        slave_pre();
        bool hs = dut->n_awready;
        slave_post_and_tick();
        if (hs) { dut->n_awvalid = 0; return true; }
    }
    dut->n_awvalid = 0;
    return false;
}
static bool offer_w(uint32_t data, bool last, int timeout = 32) {
    for (int c = 0; c < timeout; c++) {
        dut->n_wvalid = 1; dut->n_wdata = data; dut->n_wstrb = 0xF;
        dut->n_wlast = last;
        slave_pre();
        bool hs = dut->n_wready;
        slave_post_and_tick();
        if (hs) { dut->n_wvalid = 0; return true; }
    }
    dut->n_wvalid = 0;
    return false;
}
static bool offer_w_strb(uint32_t data, uint32_t strb, bool last,
                         int timeout = 32) {
    for (int c = 0; c < timeout; c++) {
        dut->n_wvalid = 1; dut->n_wdata = data; dut->n_wstrb = (uint8_t)strb;
        dut->n_wlast = last;
        slave_pre();
        bool hs = dut->n_wready;
        slave_post_and_tick();
        if (hs) { dut->n_wvalid = 0; return true; }
    }
    dut->n_wvalid = 0;
    return false;
}
static bool offer_ar(uint32_t addr, int len, int timeout = 32) {
    for (int c = 0; c < timeout; c++) {
        dut->n_arvalid = 1; dut->n_araddr = addr; dut->n_arlen = (uint8_t)len;
        dut->n_arsize = 2;
        slave_pre();
        bool hs = dut->n_arready;
        slave_post_and_tick();
        if (hs) { dut->n_arvalid = 0; return true; }
    }
    dut->n_arvalid = 0;
    return false;
}

// ═══════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_n2w_pipe;
    reset();

    printf("tb_n2w_pipe: axi_narrow_to_wide multi-outstanding "
           "(DEPTH=%d, wide-side latency=%d cycles)\n", kDepth, kLat);

    // ── Perf: the depth curve ────────────────────────────────────────
    printf("\n=== throughput (narrow-side cycles per 32-bit word) ===\n");
    struct { const char* name; int len; int n; bool rd; } perf[] = {
        { "single-beat writes",   0,  64, false },
        { "16-beat burst writes", 15, 32, false },
        { "64-beat burst writes", 63, 32, false },
        { "64-beat burst reads",  63, 32, true  },
    };
    for (auto& p : perf) {
        reset();
        WrPerf r = p.rd ? stream_reads(0x2000'0000u, p.len, p.n)
                        : stream_writes(0x1000'0000u, p.len, p.n);
        printf("  %-22s %8.2f cycles/word  (%llu cycles / %llu words)\n",
               p.name, r.cyc_per_word,
               (unsigned long long)r.cycles, (unsigned long long)r.words);
    }
    printf("\n");

    // ── Scenario A: several writes in flight ─────────────────────────
    // The wide slave answers on a 60-cycle timer, so with DEPTH > 1 all
    // of these are genuinely concurrent — the adapter must hold a B
    // credit per accepted AW and return them in order, one each.
    {
        reset();
        const int kN = (kDepth < 4) ? kDepth : 4;
        bool all_aw = true;
        int  max_inflight = 0;
        for (int i = 0; i < kN; i++) {
            all_aw &= offer_aw(0x3000'0000u + (uint32_t)i * 0x40u, 0);
            all_aw &= offer_w(0xA000'0000u + (uint32_t)i, true);
            dut->eval();
            if ((int)dut->dbg_wr_out_q > max_inflight)
                max_inflight = dut->dbg_wr_out_q;
        }
        check("multi-write: every AW accepted without waiting for a B", all_aw,
              "an AW was refused");
        if (kDepth > 1)
            check("multi-write: they really were concurrent",
                  max_inflight >= 2 && b_log.empty(),
                  "max wr_out_q=" + dec(max_inflight) +
                  " b_log=" + dec((long long)b_log.size()));
        else
            check("multi-write(depth 1): the port serializes, as it must",
                  max_inflight == 1, "max wr_out_q=" + dec(max_inflight));

        run(kLat * (kN + 2) + 64);
        check("multi-write: exactly one B per accepted write",
              (int)b_log.size() == kN,
              dec((long long)b_log.size()) + " B beats for " + dec(kN) + " writes");
        bool all_okay = true;
        for (auto& b : b_log) all_okay &= (b.resp == 0);
        check("multi-write: every B is OKAY", all_okay, "a B carried an error");
        check("multi-write: the wide side saw one write per narrow write",
              (int)wr_done.size() == kN,
              dec((long long)wr_done.size()) + " wide writes completed");
        bool payload_ok = ((int)wr_done.size() == kN);
        for (int i = 0; payload_ok && i < kN; i++) {
            // Each single-beat write lands in the lane its address picks,
            // and must carry ITS OWN data — not a neighbour's.
            const uint32_t want_addr = 0x3000'0000u + (uint32_t)i * 0x40u;
            const uint32_t want_data = 0xA000'0000u + (uint32_t)i;
            if (wr_done[i].addr != want_addr) payload_ok = false;
            else if (wr_done[i].data.size() != 1) payload_ok = false;
            else if (wr_done[i].data[0][(want_addr >> 2) & 3] != want_data)
                payload_ok = false;
        }
        check("multi-write: each completed write carries its own address+data",
              payload_ok, "a write's wide payload did not match its own AW");
        check("multi-write: the credit is fully returned afterwards",
              dut->dbg_wr_out_q == 0,
              "wr_out_q=" + dec(dut->dbg_wr_out_q));
    }

    // ── Scenario B: a read and a write in flight simultaneously ──────
    {
        reset();
        bool aw = offer_aw(0x4000'0000u, 0);
        bool w  = offer_w(0xBEEF'0001u, true);
        bool ar = offer_ar(0x5000'0010u, 0);   // lane 0 of the next 16 B
        dut->eval();
        const int wr_out = dut->dbg_wr_out_q;
        const int rd_out = dut->dbg_ar_out_q;
        check("rw-concurrent: both directions accepted", aw && w && ar,
              "aw=" + dec(aw) + " w=" + dec(w) + " ar=" + dec(ar));
        check("rw-concurrent: one write and one read are in flight together",
              wr_out == 1 && rd_out == 1,
              "wr_out=" + dec(wr_out) + " rd_out=" + dec(rd_out));

        run(kLat * 3 + 64);
        check("rw-concurrent: the write got exactly one OKAY B",
              b_log.size() == 1 && b_log[0].resp == 0,
              dec((long long)b_log.size()) + " B beats");
        check("rw-concurrent: the read got exactly one OKAY beat with RLAST",
              r_log.size() == 1 && r_log[0].resp == 0 && r_log[0].last == 1,
              dec((long long)r_log.size()) + " R beats");
        // The slave's read data is addr+lane*4, so lane 0 of 0x50000010.
        check("rw-concurrent: the read data is the read's, not the write's",
              r_log.size() == 1 && r_log[0].data == 0x5000'0010u,
              r_log.empty() ? "no R" : hex32(r_log[0].data));
        check("rw-concurrent: both credits fully returned",
              dut->dbg_wr_out_q == 0 && dut->dbg_ar_out_q == 0,
              "wr_out=" + dec(dut->dbg_wr_out_q) +
              " rd_out=" + dec(dut->dbg_ar_out_q));
    }

    // ── Scenario C: mixed burst lengths, back to back ────────────────
    // The width-conversion hazard: a burst that does not fill its last
    // wide beat, immediately followed by a different transaction.  If the
    // front end let go of the gather buffer before its final wide beat
    // was taken, the next transaction's first beats would be merged into
    // it — one wide beat carrying two transactions' data, with a wstrb
    // that is the union of both.  Each completed wide write is checked
    // for exactly its own beat count and its own data.
    if (kDepth > 1) {
        reset();
        // Burst A: 3 narrow beats starting at lane 3 -> spans 2 wide beats
        // and leaves 2 lanes of the second one empty.
        bool ok = offer_aw(0x6000'000Cu, 2);
        ok &= offer_w(0xC0DE'0000u, false);
        ok &= offer_w(0xC0DE'0001u, false);
        ok &= offer_w(0xC0DE'0002u, true);
        // Burst B: a single beat at lane 0 of the next line, offered
        // immediately behind it.
        ok &= offer_aw(0x6000'0040u, 0);
        ok &= offer_w(0xD0DE'0000u, true);
        check("mixed-burst: both transactions accepted back to back", ok,
              "a handshake was refused");
        run(kLat * 3 + 64);

        check("mixed-burst: two Bs, one per transaction",
              b_log.size() == 2, dec((long long)b_log.size()) + " B beats");
        check("mixed-burst: two wide writes completed",
              wr_done.size() == 2, dec((long long)wr_done.size()) + " wide writes");
        if (wr_done.size() == 2) {
            const WrTxn& A = wr_done[0];
            const WrTxn& B = wr_done[1];
            check("mixed-burst: A is a 2-beat wide burst at its 16 B base",
                  A.addr == 0x6000'0000u && A.len == 1 && A.data.size() == 2,
                  "addr=" + hex32(A.addr) + " len=" + dec(A.len) +
                  " beats=" + dec((long long)A.data.size()));
            check("mixed-burst: A's first wide beat strobes ONLY lane 3",
                  A.strb.size() == 2 && A.strb[0] == 0xF000u,
                  A.strb.empty() ? "no beats" : hex32(A.strb[0]));
            check("mixed-burst: A's second wide beat strobes ONLY lanes 0-1 "
                  "(B's data did not leak into it)",
                  A.strb.size() == 2 && A.strb[1] == 0x00FFu,
                  A.strb.size() < 2 ? "no second beat" : hex32(A.strb[1]));
            check("mixed-burst: A's data is A's, in order",
                  A.data.size() == 2 && A.data[0][3] == 0xC0DE'0000u &&
                  A.data[1][0] == 0xC0DE'0001u && A.data[1][1] == 0xC0DE'0002u,
                  "A payload mismatch");
            check("mixed-burst: B is its own single-beat write",
                  B.addr == 0x6000'0040u && B.data.size() == 1 &&
                  B.strb.size() == 1 && B.strb[0] == 0x000Fu &&
                  B.data[0][0] == 0xD0DE'0000u,
                  "addr=" + hex32(B.addr) + " beats=" +
                  dec((long long)B.data.size()) +
                  " strb=" + (B.strb.empty() ? "-" : hex32(B.strb[0])));
        }
    }

    // ── Scenario G: the gather handover costs no narrow cycle ────────
    //
    // THE PROPERTY DOUBLE BUFFERING EXISTS FOR.  With one gather bank the
    // cycle an assembled wide beat is handed to the wide side is a cycle
    // n_wready is low, so four narrow beats cost five cycles (1.25
    // cycles/word, and every burst write in the design paid it).  With
    // two banks the completing beat is written straight into the output
    // bank while the gather bank is freed in the same cycle, so a burst
    // against a ready wide side streams one narrow beat per cycle with no
    // gap at ANY wide-beat boundary.
    //
    // Deliberately a HANDSHAKE-LEVEL check, not a cycles/word one: the
    // throughput table above would move for a dozen unrelated reasons,
    // whereas "n_wvalid was high and n_wready was low, mid-burst, even
    // once" is exactly and only this bug.
    {
        reset();
        const uint32_t kBase  = 0xB000'0000u;
        const int      kBeats = 64;
        int  accepted = 0, stall_cycles = 0, first_stall_at = -1;
        bool started = false, aw_done = false;
        for (int c = 0; c < 512 && accepted < kBeats; c++) {
            dut->n_awvalid = aw_done ? 0 : 1;
            dut->n_awaddr  = kBase;
            dut->n_awlen   = (uint8_t)(kBeats - 1);
            dut->n_wvalid  = 1;
            dut->n_wdata   = 0xB0B0'0000u + (uint32_t)accepted;
            dut->n_wstrb   = 0xF;
            dut->n_wlast   = (accepted == kBeats - 1);
            slave_pre();
            const bool aw_hs = !aw_done && dut->n_awready;
            const bool w_hs  = dut->n_wready;
            if (started && !w_hs) {
                if (first_stall_at < 0) first_stall_at = accepted;
                stall_cycles++;
            }
            slave_post_and_tick();
            if (aw_hs) aw_done = true;
            if (w_hs) { accepted++; started = true; }
        }
        dut->n_awvalid = 0; dut->n_wvalid = 0;
        check("dblbuf: the whole 64-beat burst is handed over",
              accepted == kBeats, "accepted=" + dec(accepted));
        check("dblbuf: the narrow W channel never stalls mid-burst against a "
              "ready wide side",
              stall_cycles == 0,
              dec(stall_cycles) + " stall cycle(s), first after beat " +
              dec(first_stall_at) + " (one gather bank stalls at every "
              "4th beat)");

        run(kLat * 2 + 64);
        check("dblbuf: exactly one B for the burst", b_log.size() == 1,
              dec((long long)b_log.size()) + " B beats");
        check("dblbuf: the wide side saw one 16-beat wide burst",
              wr_done.size() == 1 && wr_done[0].data.size() == 16 &&
              wr_done[0].len == 15,
              wr_done.empty() ? "no wide write"
                              : "beats=" + dec((long long)wr_done[0].data.size()) +
                                " awlen=" + dec(wr_done[0].len));
        if (wr_done.size() == 1 && wr_done[0].data.size() == 16) {
            const WrTxn& A = wr_done[0];
            bool data_ok = true, strb_ok = true;
            for (size_t i = 0; i < A.data.size(); i++) {
                strb_ok &= (A.strb[i] == 0xFFFFu);
                for (int j = 0; j < 4; j++)
                    data_ok &= (A.data[i][j] ==
                                0xB0B0'0000u + (uint32_t)(i * 4 + (size_t)j));
            }
            check("dblbuf: every narrow beat landed in its own lane, in order",
                  data_ok, "lane payload mismatch");
            check("dblbuf: every wide beat is fully strobed, none unioned",
                  strb_ok, "a wide beat carried the wrong strobes");
        }
    }

    // ── Scenario H: a PARTIAL trailing group keeps its own strobes ────
    //
    // The strobe hazard double buffering makes easy to write.  A 6-beat
    // burst from lane 0 fills lanes 0-3 (wide beat 0) and then only lanes
    // 0-1 (wide beat 1).  If the gather bank's strobes are not zeroed when
    // a group leaves it, wide beat 1 inherits beat 0's lane-2/3 strobes
    // and writes two words the master never sent.  Per-beat DISTINCT
    // strobes so a lane mix-up is visible as well as a leak.
    {
        reset();
        const uint32_t kBase = 0xB100'0000u;      // 16 B aligned -> lane 0
        const uint32_t sb[6] = { 0x1, 0x2, 0x4, 0x8, 0x3, 0xC };
        bool ok = offer_aw(kBase, 5);
        for (int i = 0; i < 6; i++)
            ok &= offer_w_strb(0xB1B1'0000u + (uint32_t)i, sb[i], i == 5);
        check("dblbuf-partial: the 6-beat burst is accepted", ok,
              "a handshake was refused");
        run(kLat * 2 + 64);
        check("dblbuf-partial: two wide beats, one B",
              wr_done.size() == 1 && b_log.size() == 1 &&
              wr_done[0].data.size() == 2,
              wr_done.empty() ? "no wide write"
                              : "beats=" + dec((long long)wr_done[0].data.size()) +
                                " b=" + dec((long long)b_log.size()));
        if (wr_done.size() == 1 && wr_done[0].strb.size() == 2) {
            const WrTxn& A = wr_done[0];
            check("dblbuf-partial: the full group's strobes are exactly its own",
                  A.strb[0] == 0x8421u, hex32(A.strb[0]) + " != 0x00008421");
            check("dblbuf-partial: the trailing group strobes ONLY lanes 0-1 "
                  "(the previous group's strobes were cleared out of the "
                  "gather bank)",
                  A.strb[1] == 0x00C3u, hex32(A.strb[1]) + " != 0x000000c3");
            check("dblbuf-partial: both groups carry their own data",
                  A.data[0][0] == 0xB1B1'0000u && A.data[0][3] == 0xB1B1'0003u &&
                  A.data[1][0] == 0xB1B1'0004u && A.data[1][1] == 0xB1B1'0005u,
                  "payload mismatch");
        }
    }

    // ── Scenario I: the gather bank parks, then hands over ────────────
    //
    // The other half of double buffering: when the wide side will NOT
    // take the presented beat, a completing group has to park in the
    // gather bank, backpressure the narrow channel, and then move out
    // intact — with the right data, the right strobes and, if it is the
    // transaction's final group, the right WLAST.  A parked beat that
    // loses its WLAST hangs the transaction; one that loses its data
    // corrupts memory silently.
    //
    // 12 narrow beats against a wedged wide W channel: 4 bypass into the
    // output bank, 4 park in the gather bank, and the 9th must be
    // REFUSED.  Then the wide side wakes up and the remaining 4 stream.
    {
        reset();
        const uint32_t kBase  = 0xB200'0000u;
        const int      kBeats = 12;
        wide_w_stalled = true;
        check("dblbuf-park: AW accepted with the wide W channel wedged",
              offer_aw(kBase, kBeats - 1));

        int absorbed = 0;
        for (int c = 0; c < 64 && absorbed < kBeats; c++) {
            dut->n_wvalid = 1;
            dut->n_wdata  = 0xB2B2'0000u + (uint32_t)absorbed;
            dut->n_wstrb  = 0xF;
            dut->n_wlast  = (absorbed == kBeats - 1);
            slave_pre();
            const bool hs = dut->n_wready;
            slave_post_and_tick();
            if (hs) absorbed++;
        }
        dut->n_wvalid = 0;
        // Two 128-bit banks x four 32-bit lanes.  One bank would stop at
        // 4; three would take 12 and this scenario would never park.
        check("dblbuf-park: exactly two wide beats are buffered before the "
              "narrow channel backpressures",
              absorbed == 8, "absorbed=" + dec(absorbed) + " of 12");

        // Wake the wide side up and let the rest of the burst stream.
        wide_w_stalled = false;
        int total = absorbed;
        for (int c = 0; c < 128 && total < kBeats; c++) {
            dut->n_wvalid = 1;
            dut->n_wdata  = 0xB2B2'0000u + (uint32_t)total;
            dut->n_wstrb  = 0xF;
            dut->n_wlast  = (total == kBeats - 1);
            slave_pre();
            const bool hs = dut->n_wready;
            slave_post_and_tick();
            if (hs) total++;
        }
        dut->n_wvalid = 0;
        check("dblbuf-park: the burst finishes once the wide side wakes up",
              total == kBeats, "handed over " + dec(total) + " of " + dec(kBeats));
        run(kLat * 2 + 64);
        check("dblbuf-park: one B, three wide beats",
              b_log.size() == 1 && wr_done.size() == 1 &&
              wr_done[0].data.size() == 3,
              wr_done.empty() ? "no wide write"
                              : "beats=" + dec((long long)wr_done[0].data.size()) +
                                " b=" + dec((long long)b_log.size()));
        if (wr_done.size() == 1 && wr_done[0].data.size() == 3) {
            const WrTxn& A = wr_done[0];
            bool ok = true;
            for (size_t i = 0; i < 3; i++) {
                ok &= (A.strb[i] == 0xFFFFu);
                for (int j = 0; j < 4; j++)
                    ok &= (A.data[i][j] ==
                           0xB2B2'0000u + (uint32_t)(i * 4 + (size_t)j));
            }
            check("dblbuf-park: the parked beat came out with its own data "
                  "and strobes, in order", ok, "payload mismatch");
        }
    }

    // ── Scenario J: the transaction's FINAL group is the parked one ──
    //
    // Scenario I parks a MIDDLE group, so its parked beat carries
    // WLAST = 0 and a bug that drops the parked beat's WLAST is invisible
    // there — it agrees with the correct answer.  (It was: the first
    // version of scenario I passed a mutant that hardwired the transferred
    // beat's wo_last_q to 0.)  This scenario is the one that distinguishes
    // them: an 8-beat burst against a wedged wide W channel fills the
    // output bank with group 0 and parks group 1, which IS the last group.
    // Its WLAST has to survive the park and the transfer, or the wide side
    // never sees the burst end and nothing ever completes.
    {
        reset();
        const uint32_t kBase  = 0xB300'0000u;
        const int      kBeats = 8;
        wide_w_stalled = true;
        check("dblbuf-park-last: AW accepted with the wide W channel wedged",
              offer_aw(kBase, kBeats - 1));

        int absorbed = 0;
        for (int c = 0; c < 64 && absorbed < kBeats; c++) {
            dut->n_wvalid = 1;
            dut->n_wdata  = 0xB3B3'0000u + (uint32_t)absorbed;
            dut->n_wstrb  = 0xF;
            dut->n_wlast  = (absorbed == kBeats - 1);
            slave_pre();
            const bool hs = dut->n_wready;
            slave_post_and_tick();
            if (hs) absorbed++;
        }
        dut->n_wvalid = 0;
        check("dblbuf-park-last: the whole burst fits in the two banks",
              absorbed == kBeats, "absorbed=" + dec(absorbed));
        // The last group is now sitting in the gather bank with its WLAST.
        wide_w_stalled = false;
        run(kLat * 2 + 64);
        check("dblbuf-park-last: the parked FINAL beat keeps its WLAST, so the "
              "burst terminates and is answered",
              wr_done.size() == 1 && wr_done[0].data.size() == 2 &&
              b_log.size() == 1,
              wr_done.empty()
                  ? "the wide burst never terminated (no WLAST) — " +
                    dec((long long)b_log.size()) + " B beats"
                  : "beats=" + dec((long long)wr_done[0].data.size()) +
                    " b=" + dec((long long)b_log.size()));
        if (wr_done.size() == 1 && wr_done[0].data.size() == 2) {
            const WrTxn& A = wr_done[0];
            bool ok = true;
            for (size_t i = 0; i < 2; i++) {
                ok &= (A.strb[i] == 0xFFFFu);
                for (int j = 0; j < 4; j++)
                    ok &= (A.data[i][j] ==
                           0xB3B3'0000u + (uint32_t)(i * 4 + (size_t)j));
            }
            check("dblbuf-park-last: both beats carry their own payload", ok,
                  "payload mismatch");
        }
    }

    // ── Scenario D: watchdog fires with N writes outstanding ─────────
    // One timer covers the whole in-flight set (see the RTL comment on
    // why per-transaction timers would say nothing extra under a single
    // wide ID).  Every write the master handed over must still get its
    // own B.
    {
        reset();
        const int kN = (kDepth < 4) ? kDepth : 4;
        bool ok = true;
        for (int i = 0; i < kN; i++) {
            ok &= offer_aw(0x7000'0000u + (uint32_t)i * 0x40u, 0);
            ok &= offer_w(0xE000'0000u + (uint32_t)i, true);
        }
        check("watchdog-multi(write): N writes accepted", ok, "an AW was refused");
        dut->eval();
        check("watchdog-multi(write): N really are in flight",
              (int)dut->dbg_wr_out_q == kN,
              "wr_out_q=" + dec(dut->dbg_wr_out_q) + " expected " + dec(kN));

        // Now the wide side goes completely silent.  Nothing that has
        // already been handed over will ever be answered.
        slave_stalled = true;
        b_log.clear();
        run(512 * 4 + 256);
        slave_stalled = false;
        run(64);

        check("watchdog-multi(write): exactly N SLVERR Bs, one per write",
              (int)b_log.size() == kN,
              dec((long long)b_log.size()) + " B beats for " + dec(kN) + " writes");
        bool all_slverr = !b_log.empty();
        for (auto& b : b_log) all_slverr &= (b.resp == 2);
        check("watchdog-multi(write): every one of them is SLVERR",
              all_slverr, "a synthesized B was not SLVERR");
        check("watchdog-multi(write): the credit is fully released",
              dut->dbg_wr_out_q == 0 && dut->dbg_aw_err_q == 0,
              "wr_out_q=" + dec(dut->dbg_wr_out_q) +
              " err=" + dec(dut->dbg_aw_err_q));
        dut->eval();
        check("watchdog-multi(write): the port is usable again",
              dut->n_awready == 1, "n_awready still low");
    }

    // ── Scenario E: watchdog fires with N reads outstanding ──────────
    // The one that is easy to get wrong: the abandoned set must be
    // answered ONE TRANSACTION AT A TIME, with RLAST at each
    // transaction's own boundary.  Collapsing them into one long SLVERR
    // run with a single RLAST would strand the master on every read but
    // the first — the same hazard the RTL header documents for a single
    // beat on a multi-beat read, one level up.
    {
        reset();
        const int kN   = (kDepth < 3) ? kDepth : 3;
        const int kLen = 3;                    // 4 narrow beats each
        bool ok = true;
        for (int i = 0; i < kN; i++)
            ok &= offer_ar(0x8000'0000u + (uint32_t)i * 0x40u, kLen);
        check("watchdog-multi(read): N reads accepted", ok, "an AR was refused");
        dut->eval();
        check("watchdog-multi(read): N really are in flight",
              (int)dut->dbg_ar_out_q == kN,
              "ar_out_q=" + dec(dut->dbg_ar_out_q) + " expected " + dec(kN));

        slave_stalled = true;
        r_log.clear();
        run(512 * 4 + 256);
        slave_stalled = false;
        run(64);

        check("watchdog-multi(read): every beat of every read is delivered",
              (int)r_log.size() == kN * (kLen + 1),
              dec((long long)r_log.size()) + " R beats, expected " +
              dec(kN * (kLen + 1)));
        bool all_slverr = !r_log.empty();
        for (auto& r : r_log) all_slverr &= (r.resp == 2);
        check("watchdog-multi(read): every beat is SLVERR", all_slverr,
              "a synthesized R beat was not SLVERR");
        // RLAST must land on beat kLen, 2*kLen+1, ... — i.e. once per
        // transaction, at its own boundary, and nowhere else.
        bool rlast_ok = ((int)r_log.size() == kN * (kLen + 1));
        int  rlast_count = 0;
        for (size_t i = 0; rlast_ok && i < r_log.size(); i++) {
            const bool want_last = ((i + 1) % (kLen + 1) == 0);
            if ((r_log[i].last != 0) != want_last) rlast_ok = false;
            if (r_log[i].last) rlast_count++;
        }
        check("watchdog-multi(read): RLAST at EACH transaction's own boundary",
              rlast_ok && rlast_count == kN,
              "rlast_count=" + dec(rlast_count) + " expected " + dec(kN));
        check("watchdog-multi(read): the credit is fully released",
              dut->dbg_ar_out_q == 0 && dut->dbg_ar_err_mode_q == 0,
              "ar_out_q=" + dec(dut->dbg_ar_out_q) +
              " err_mode=" + dec(dut->dbg_ar_err_mode_q));
        dut->eval();
        check("watchdog-multi(read): the port is usable again",
              dut->n_arready == 1, "n_arready still low");
    }

    // ── Scenario F (depth 1 only): the historical contract ───────────
    // WR_OUTSTANDING / RD_OUTSTANDING = 1 must reproduce exactly what
    // this module did before the rework: no second transaction is
    // accepted until the first has completed end to end.
    if (kDepth == 1) {
        reset();
        check("depth1: first write accepted", offer_aw(0x9000'0000u, 0));
        check("depth1: its W beat accepted", offer_w(0x1234'5678u, true));
        bool awready_seen = false;
        for (int c = 0; c < kLat - 4; c++) {
            slave_pre();
            if (dut->n_awready) awready_seen = true;
            slave_post_and_tick();
        }
        check("depth1: n_awready stays low for the whole B round trip",
              !awready_seen, "n_awready came back early");
        run(kLat + 32);
        check("depth1: the B arrives and the port reopens",
              b_log.size() == 1 && dut->n_awready == 1,
              dec((long long)b_log.size()) + " B beats");

        reset();
        check("depth1: first read accepted", offer_ar(0xA000'0000u, 0));
        bool arready_seen = false;
        for (int c = 0; c < kLat - 4; c++) {
            slave_pre();
            if (dut->n_arready) arready_seen = true;
            slave_post_and_tick();
        }
        check("depth1: n_arready stays low until the read completes",
              !arready_seen, "n_arready came back early");
        run(kLat + 32);
        check("depth1: the R arrives and the port reopens",
              r_log.size() == 1 && dut->n_arready == 1,
              dec((long long)r_log.size()) + " R beats");
    }

    printf("\n%d checks: %d passed, %d failed\n", n_pass + n_fail, n_pass, n_fail);
    if (n_fail) { printf("FAILED\n"); return 1; }
    printf("All %d checks PASSED.\n", n_pass);
    return 0;
}
