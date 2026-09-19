// tb_vhdd_readahead.cpp — unit testbench for rtl/soc/vhdd_readahead.v
//
// WHAT IT PROVES
// ══════════════
// The module sits between a vhdd MASTER (scsi.v, via vhdd_mux) and a
// vhdd PROVIDER (vhdd_sd → sd_scsi_bridge → sd_ctrl → the card).  This
// bench replaces both with models, so it can do two things the SCSI-level
// benches cannot:
//
//  1. Give the provider a REALISTIC COST MODEL and measure blocks/sec.
//     The hardware measurement this module exists to fix is a RATIO:
//
//        per-command latency  69.7 ms   (R1 poll + data-token wait)
//        per-block streaming   0.263 ms (boot_fsm: 3800 sectors/s)
//        ratio ................ 265 : 1
//
//     The model below reproduces that ratio exactly — CMD_LATENCY_CYC =
//     265 * 512 with one byte per cycle — rather than the absolute
//     numbers, which would make the run take minutes for nothing.  Any
//     speedup this bench reports is therefore the speedup the ratio
//     predicts on hardware, not an artefact of the model's units.
//
//  2. Drive the COHERENCY cases directly.  A cache that serves one stale
//     block silently corrupts the user's disk image, so the write,
//     invalidate, straddle, seek, clamp and error paths are all driven
//     here at the contract level where they can be checked byte for byte.
//
// Build twice from the same source:
//   tb-vhdd-readahead      ENABLE=1   the cache
//   tb-vhdd-readahead-off  ENABLE=0   the identical netlist, built out —
//                                     this is the "before" number.
//
// The disk content is a deterministic function of (lba, offset) with a
// mutable overlay for writes, so every byte the master receives can be
// checked against what the provider would have returned at that instant.

#include <verilated.h>
#include "Vvhdd_readahead.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <map>
#include <vector>
#include <string>

// ── Provider cost model ──────────────────────────────────────────────
// See the header: 265:1 is the measured hardware ratio.
static const int      BYTE_CYCLES       = 1;
static const uint64_t CMD_LATENCY_CYC   = 265ULL * 512ULL;   // 135680

// Volume geometry.  Small enough to keep the model cheap, large enough
// that a "seek" is genuinely outside the cached runs.
static const uint32_t NUM_LBAS_DEFAULT  = 1048576;

static int  g_pass = 0, g_fail = 0;
static std::string g_case;

static void check(bool ok, const char* what) {
    if (ok) { g_pass++; }
    else    { g_fail++; printf("  [%s] FAIL %s\n", g_case.c_str(), what); }
}
static void checkf(bool ok, const char* fmt, ...) {
    if (ok) { g_pass++; return; }
    g_fail++;
    va_list ap; va_start(ap, fmt);
    printf("  [%s] FAIL ", g_case.c_str());
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

// ─────────────────────────────────────────────────────────────────────
// The world below the module: a vhdd provider with a per-command cost.
// ─────────────────────────────────────────────────────────────────────
struct Provider {
    Vvhdd_readahead* dut = nullptr;

    // Backing store.  Default content is f(lba, offset); `overlay` holds
    // whatever has been written, so a read after a write returns the new
    // bytes exactly as a real volume would.
    std::map<uint32_t, std::vector<uint8_t>> overlay;

    static uint8_t gen(uint32_t lba, int off) {
        return (uint8_t)((lba * 1103515245u + off * 12345u) >> 7);
    }
    uint8_t byte_at(uint32_t lba, int off) {
        auto it = overlay.find(lba);
        if (it != overlay.end()) return it->second[off];
        return gen(lba, off);
    }
    void write_byte(uint32_t lba, int off, uint8_t v) {
        auto it = overlay.find(lba);
        if (it == overlay.end()) {
            std::vector<uint8_t> blk(512);
            for (int i = 0; i < 512; i++) blk[i] = gen(lba, i);
            it = overlay.emplace(lba, std::move(blk)).first;
        }
        it->second[off] = v;
    }

    // ── state machine ────────────────────────────────────────────────
    enum St { IDLE, LATENCY, STREAM, WSTREAM, FINISH };
    St        st = IDLE;
    uint64_t  timer = 0;
    uint32_t  lba = 0;
    uint32_t  blocks = 0;
    bool      is_write = false;
    uint64_t  bytes_done = 0;
    int       byte_gap = 0;

    // ── fault injection ──────────────────────────────────────────────
    // Error after this many blocks have streamed (-1 = never).
    int       fail_after_blocks = -1;

    // ── observation ──────────────────────────────────────────────────
    uint32_t  commands = 0;          // provider requests issued
    uint64_t  blocks_requested = 0;  // summed over those requests
    uint32_t  last_block_count = 0;
    uint32_t  last_lba = 0;
    bool      last_multi = false;
    uint32_t  max_lba_touched = 0;
    uint32_t  num_lbas = NUM_LBAS_DEFAULT;

    // Bytes the provider received on a write, per LBA.
    std::vector<uint8_t> wbuf;

    void reset() { st = IDLE; timer = 0; bytes_done = 0; byte_gap = 0; }

    // Called once per posedge, BEFORE the DUT is evaluated for that edge,
    // with the DUT's outputs from the previous settle already visible.
    void drive() {
        dut->p_busy     = (st != IDLE);
        dut->p_done     = 0;
        // `error` is sticky and only meaningful with done (rtl/vhdd.vh);
        // keep it as-is between requests.
        dut->p_rd_valid = 0;
        dut->p_wr_ready = 0;
        dut->p_chk_ok   = 1;

        switch (st) {
        case IDLE:
            if (dut->p_req_go) {
                commands++;
                lba              = dut->p_req_lba;
                blocks           = dut->p_req_block_count;
                is_write         = dut->p_req_write;
                last_block_count = blocks;
                last_lba         = lba;
                last_multi       = dut->p_req_multi;
                blocks_requested += blocks;
                if (lba + blocks > max_lba_touched)
                    max_lba_touched = lba + blocks;
                bytes_done = 0;
                byte_gap   = 0;
                timer      = CMD_LATENCY_CYC;
                dut->p_error = 0;
                st = LATENCY;
                dut->p_busy = 1;
                wbuf.clear();
            }
            break;

        case LATENCY:
            if (timer) timer--;
            else       st = is_write ? WSTREAM : STREAM;
            break;

        case STREAM: {
            // REAL back-pressure: a byte is only issued when the consumer
            // says it has room (rtl/vhdd.vh rd_ready).
            if (!dut->p_rd_ready) break;
            if (byte_gap) { byte_gap--; break; }
            uint32_t blk = (uint32_t)(bytes_done / 512);
            if (fail_after_blocks >= 0 && (int)blk >= fail_after_blocks) {
                dut->p_error = 1;
                st = FINISH;
                break;
            }
            int off = (int)(bytes_done % 512);
            dut->p_rd_data  = byte_at(lba + blk, off);
            dut->p_rd_valid = 1;
            bytes_done++;
            byte_gap = BYTE_CYCLES - 1;
            if (bytes_done == (uint64_t)blocks * 512) st = FINISH;
            break;
        }

        case WSTREAM: {
            if (fail_after_blocks >= 0 &&
                bytes_done >= uint64_t(fail_after_blocks) * 512) {
                dut->p_error = 1;
                st = FINISH;
                break;
            }
            if (!dut->p_wr_avail) break;
            if (byte_gap) { byte_gap--; break; }
            dut->p_wr_ready = 1;      // pulse: "give me a byte"
            byte_gap = BYTE_CYCLES - 1;
            break;
        }

        case FINISH:
            dut->p_done = 1;
            dut->p_busy = 0;
            st = IDLE;
            break;
        }
    }

    // Called after the DUT has been evaluated for the edge, so we see the
    // data it presented in response to our wr_ready pulse.
    void sample(uint8_t wr_data) {
        if (st == WSTREAM && dut->p_wr_ready) {
            uint32_t blk = (uint32_t)(bytes_done / 512);
            int      off = (int)(bytes_done % 512);
            write_byte(lba + blk, off, wr_data);
            bytes_done++;
            if (bytes_done == (uint64_t)blocks * 512) st = FINISH;
        }
    }
};

// ─────────────────────────────────────────────────────────────────────
// The world above the module: a vhdd master.
// ─────────────────────────────────────────────────────────────────────
struct Harness {
    Vvhdd_readahead* dut;
    Provider         prov;
    uint64_t         cycles = 0;

    // Consumer pacing: accept a byte only every `rd_gap+1` cycles, or
    // never when `stall` is set (the bounded-response case).
    int  rd_gap   = 0;
    int  rd_count = 0;
    bool stall    = false;

    std::vector<uint8_t> rx;
    bool saw_done = false, saw_error = false;

    explicit Harness(Vvhdd_readahead* d) : dut(d) { prov.dut = d; }

    // Producer-paced write payload (see write_request).
    const std::vector<uint8_t>* wpayload = nullptr;
    size_t                      widx = 0;

    // One pb_clk cycle.
    //
    // ORDER MATTERS AND IT BIT THIS BENCH ONCE.  Several of the module's
    // outputs are COMBINATIONAL and qualified by the current state
    // (`p_req_go` is `st == ST_IDLE && ...`).  Sampling them after the
    // posedge -- when `st` has already moved on -- makes them vanish, and
    // the pass-through path looked dead when it was not.  So: settle with
    // the clock LOW, let the models look at and answer the outputs that
    // are genuinely present during THIS cycle, and only then take the
    // edge.  That is also what the real neighbours see: sd_scsi_bridge
    // latches pb_go at the posedge, using the value the wire held before
    // it.
    void tick() {
        dut->clk = 0; dut->eval();

        // ── consumer readiness for this cycle ────────────────────────
        bool ready;
        if (stall)            ready = false;
        else if (rd_gap == 0) ready = true;
        else                  ready = (rd_count == 0);
        dut->rd_ready = ready;
        dut->wr_avail = 1;
        dut->eval();

        prov.drive();
        dut->eval();

        // ── what the module is presenting THIS cycle ─────────────────
        if (dut->rd_valid) {
            rx.push_back(dut->rd_data);
            if (rd_gap) rd_count = rd_gap;
        } else if (rd_count) {
            rd_count--;
        }
        if (dut->done) { saw_done = true; saw_error = dut->error; }

        // The provider takes the byte the master is presenting on its own
        // wr_ready pulse; the master then advances to the next one.
        prov.sample(dut->wr_data);
        if (wpayload && dut->wr_ready) {
            if (widx + 1 < wpayload->size()) widx++;
            dut->wr_data = (*wpayload)[widx];
            dut->eval();
        }

        dut->clk = 1; dut->eval();
        cycles++;
    }

    void tickn(int n) { for (int i = 0; i < n; i++) tick(); }

    // Run until the provider is genuinely idle.  The module answers the
    // master and then keeps absorbing the TAIL of the run in the
    // background, so "the last request finished" is not the same thing as
    // "the card is free" -- and a test that wants to observe a COLD cache
    // has to wait for the difference.
    void drain(uint64_t budget = 40'000'000ULL) {
        uint64_t n = 0;
        while ((prov.st != Provider::IDLE || dut->busy) && n++ < budget) tick();
        tickn(4);
    }

    void reset() {
        dut->rst = 1;
        dut->inval = 0;
        dut->req_go = 0;
        dut->req_write = 0;
        dut->req_multi = 0;
        dut->req_lba = 0;
        dut->req_block_count = 0;
        dut->rd_ready = 1;
        dut->wr_valid = 0;
        dut->wr_data = 0;
        dut->wr_avail = 1;
        dut->num_lbas = prov.num_lbas;
        dut->chk_lba = 0;
        dut->chk_blocks = 0;
        dut->p_busy = 0; dut->p_done = 0; dut->p_error = 0;
        dut->p_rd_valid = 0; dut->p_rd_data = 0;
        dut->p_wr_ready = 0; dut->p_chk_ok = 1;
        prov.reset();
        tickn(8);
        dut->rst = 0;
        tickn(4);
    }

    // Issue one vhdd request and run until `done`.  Returns cycles taken
    // from the go pulse to the done pulse.
    uint64_t request(bool write, uint32_t lba, uint32_t blocks,
                     uint64_t budget = 40'000'000ULL) {
        // The contract says busy must be low before a new kick.
        uint64_t guard = 0;
        while (dut->busy && guard++ < budget) tick();

        rx.clear();
        saw_done = saw_error = false;

        dut->req_write       = write;
        dut->req_multi       = (blocks > 1);
        dut->req_lba         = lba;
        dut->req_block_count = blocks;
        dut->req_go          = 1;
        uint64_t t0 = cycles;
        tick();
        dut->req_go = 0;

        uint64_t n = 0;
        while (!saw_done && n++ < budget) tick();
        return cycles - t0;
    }

    // Data-out for writes: the master presents the byte the provider
    // asked for.  Producer-paced, exactly as scsi.v does it.
    // (Driven inline by write_request below.)
    uint64_t write_request(uint32_t lba, uint32_t blocks,
                           const std::vector<uint8_t>& payload) {
        uint64_t guard = 0;
        while (dut->busy && guard++ < 40'000'000ULL) tick();

        saw_done = saw_error = false;
        wpayload = &payload;
        widx     = 0;
        dut->wr_data = payload.empty() ? 0 : payload[0];

        dut->req_write       = 1;
        dut->req_multi       = (blocks > 1);
        dut->req_lba         = lba;
        dut->req_block_count = blocks;
        dut->req_go          = 1;
        dut->wr_valid        = 1;
        uint64_t t0 = cycles;
        tick();
        dut->req_go = 0;

        uint64_t n = 0;
        while (!saw_done && n++ < 40'000'000ULL) tick();
        dut->wr_valid = 0;
        wpayload = nullptr;
        return cycles - t0;
    }

    // Verify the received payload against what the volume holds now.
    void expect_blocks(uint32_t lba, uint32_t blocks, const char* what) {
        checkf(rx.size() == (size_t)blocks * 512,
               "%s: got %zu bytes, expected %u", what, rx.size(), blocks * 512);
        if (rx.size() != (size_t)blocks * 512) return;
        for (uint32_t b = 0; b < blocks; b++) {
            for (int o = 0; o < 512; o++) {
                uint8_t want = prov.byte_at(lba + b, o);
                uint8_t got  = rx[(size_t)b * 512 + o];
                if (want != got) {
                    checkf(false, "%s: lba %u byte %d: got %02x want %02x",
                           what, lba + b, o, got, want);
                    return;
                }
            }
        }
        g_pass++;
    }
};

// ─────────────────────────────────────────────────────────────────────
#define CASE(name) do { g_case = name; printf("-- %s\n", name); } while (0)

static bool RA_ON = true;   // set from the ENABLE parameter at runtime
static uint32_t RUN = 1;    // blocks per way, MEASURED not assumed

// Nothing in this bench may hardcode BLOCKS_PER_WAY.  The parameter is a
// build knob, and a bench that bakes today's value into its assertions
// manufactures convincing false failures the day someone turns it.  So:
// warm a cold run and walk forward until a request reaches the provider
// again; that distance IS the run length.
static uint32_t probe_run_len(Vvhdd_readahead* dut) {
    Harness h(dut);
    h.reset();
    uint32_t base = 50000;
    h.request(false, base, 1);
    uint32_t c = h.prov.commands;
    for (uint32_t k = 1; k <= 4096; k++) {
        h.request(false, base + k, 1);
        if (h.prov.commands != c) return k;
    }
    return 1;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vvhdd_readahead* dut = new Vvhdd_readahead;

    // Detect which build this is by behaviour, not by a #define: warm a
    // run and see whether the second read reaches the provider.
    {
        Harness h(dut);
        h.reset();
        h.request(false, 5000, 1);
        uint32_t c0 = h.prov.commands;
        h.request(false, 5001, 1);
        RA_ON = (h.prov.commands == c0);
    }
    RUN = RA_ON ? probe_run_len(dut) : 1;
    printf("=== vhdd_readahead tb — cache %s ===\n", RA_ON ? "ENABLED" : "BUILT OUT");
    printf("    measured read-ahead run length: %u blocks (%u KiB per way)\n",
           RUN, RUN / 2);
    printf("    provider model: %llu cycles per command + %d cycle(s) per byte\n",
           (unsigned long long)CMD_LATENCY_CYC, BYTE_CYCLES);

    // ═════════════════════════════════════════════════════════════════
    CASE("sequential_throughput");
    uint64_t seq_cycles = 0;
    uint32_t seq_cmds = 0;
    uint64_t seq_blocks_fetched = 0;
    const uint32_t SEQ_BLOCKS = 256;
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 100000;
        uint32_t c0 = h.prov.commands;
        uint64_t b0 = h.prov.blocks_requested;
        uint64_t t0 = h.cycles;
        for (uint32_t i = 0; i < SEQ_BLOCKS; i++) {
            h.request(false, base + i, 1);
            check(!h.saw_error, "sequential read reported an error");
            h.expect_blocks(base + i, 1, "sequential payload");
        }
        seq_cycles = h.cycles - t0;
        seq_cmds   = h.prov.commands - c0;
        seq_blocks_fetched = h.prov.blocks_requested - b0;
        checkf(h.prov.max_lba_touched <= base + SEQ_BLOCKS + 32,
               "read-ahead ran past the request window (max lba %u)",
               h.prov.max_lba_touched);
    }
    // Two numbers, and they answer different questions.
    //
    //  * MODEL cycles/block is what this bench actually measured.  Its
    //    absolute value is meaningless (the model streams a byte per
    //    cycle, ~26x faster than the real transport) but the RATIO
    //    between the two builds is exactly the amortisation factor.
    //
    //  * The HARDWARE PROJECTION re-costs the very same provider traffic
    //    -- commands issued and blocks fetched -- with the numbers
    //    measured on the board on 2026-09-15.  That is the blocks/sec
    //    this change is expected to produce, and it is derived from
    //    counters, not from a fit.
    const double HW_MS_PER_COMMAND = 69.7;   // R1 poll + data-token wait
    const double HW_MS_PER_BLOCK   = 0.263;  // boot_fsm: 3800 sectors/s
    double cyc_per_block = (double)seq_cycles / SEQ_BLOCKS;
    double hw_ms = seq_cmds * HW_MS_PER_COMMAND +
                   (double)seq_blocks_fetched * HW_MS_PER_BLOCK;
    double hw_bps = (double)SEQ_BLOCKS / (hw_ms / 1000.0);
    printf("   THROUGHPUT  %u sequential single-block reads\n", SEQ_BLOCKS);
    printf("   THROUGHPUT  provider commands issued  : %u\n", seq_cmds);
    printf("   THROUGHPUT  blocks fetched from card  : %llu\n",
           (unsigned long long)seq_blocks_fetched);
    printf("   THROUGHPUT  model cycles/block        : %.1f\n", cyc_per_block);
    printf("   THROUGHPUT  projected HW time         : %.0f ms\n", hw_ms);
    printf("   THROUGHPUT  projected HW blocks/s     : %.1f\n", hw_bps);
    printf("   THROUGHPUT  projected HW KB/s         : %.1f\n", hw_bps * 0.5);

    // ═════════════════════════════════════════════════════════════════
    CASE("hits_issue_no_command");
    if (RA_ON) {
        Harness h(dut);
        h.reset();
        uint32_t base = 200000;
        h.request(false, base, 1);
        uint32_t after_first = h.prov.commands;
        checkf(after_first == 1, "first read issued %u commands", after_first);
        for (uint32_t i = 1; i < RUN; i++) {
            h.request(false, base + i, 1);
            h.expect_blocks(base + i, 1, "warm-run payload");
        }
        checkf(h.prov.commands == after_first,
               "blocks 1..%u of the warmed run cost %u extra commands",
               RUN - 1, h.prov.commands - after_first);
        // One past the run: exactly one more command.
        h.request(false, base + RUN, 1);
        checkf(h.prov.commands == after_first + 1,
               "crossing the run boundary cost %u commands",
               h.prov.commands - after_first);
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("write_coherency");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 300000;
        // Warm a run that contains base..base+31.
        h.request(false, base, 1);
        h.expect_blocks(base, 1, "pre-write read");

        // Overwrite base+3 with a recognisable pattern.
        std::vector<uint8_t> payload(512);
        for (int i = 0; i < 512; i++) payload[i] = (uint8_t)(0xA0 ^ i);
        h.write_request(base + 3, 1, payload);
        check(!h.saw_error, "write reported an error");
        for (int i = 0; i < 512; i++)
            if (h.prov.byte_at(base + 3, i) != payload[i]) {
                check(false, "provider did not receive the written bytes");
                break;
            }
        g_pass++;

        // THE test: the very next read of the written block must return
        // the NEW bytes, not the copy the run captured before the write.
        h.request(false, base + 3, 1);
        h.expect_blocks(base + 3, 1, "post-write read of the written block");
        checkf(h.rx.size() == 512 && h.rx[0] == payload[0] && h.rx[511] == payload[511],
               "post-write read returned stale bytes (%02x..%02x, want %02x..%02x)",
               h.rx.size() ? h.rx[0] : 0, h.rx.size() ? h.rx[511] : 0,
               payload[0], payload[511]);

        // ...and so must its NEIGHBOURS, which were in the same run and
        // were not written.  (A cache that invalidated only the written
        // block would pass the line above and fail nothing here; a cache
        // that invalidated nothing fails both.)
        h.request(false, base + 4, 1);
        h.expect_blocks(base + 4, 1, "post-write read of a run neighbour");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("unrelated_write_preserves_both_cached_runs");
    {
        Harness h(dut);
        h.reset();
        const uint32_t a = 310000, b = 320000;
        h.request(false, a, 1); h.expect_blocks(a, 1, "warm A"); h.drain();
        h.request(false, b, 1); h.expect_blocks(b, 1, "warm B"); h.drain();
        std::vector<uint8_t> payload(512, 0x6d);
        h.write_request(330000, 1, payload);
        check(!h.saw_error, "unrelated write succeeds");
        uint32_t commands = h.prov.commands;
        h.request(false, a, 1); h.expect_blocks(a, 1, "A survives unrelated write");
        h.request(false, b, 1); h.expect_blocks(b, 1, "B survives unrelated write");
        if (RA_ON) check(h.prov.commands == commands,
                         "unrelated write must not cause either run to be refetched");
    }

    CASE("write_invalidates_only_overlapping_run");
    {
        Harness h(dut);
        h.reset();
        const uint32_t a = 340000, b = 350000;
        h.request(false, a, 1); h.drain();
        h.request(false, b, 1); h.drain();
        std::vector<uint8_t> payload(512, 0x96);
        h.write_request(a, 1, payload);
        uint32_t commands = h.prov.commands;
        h.request(false, b, 1); h.expect_blocks(b, 1, "untouched run survives");
        if (RA_ON) check(h.prov.commands == commands,
                         "overlapping write should retain the other way");
        h.request(false, a, 1);
        check(h.rx == payload, "overlapping way must return the written bytes");
        check(h.prov.commands > commands, "overlapping way was invalidated");
    }

    CASE("write_overlap_half_open_boundaries");
    {
        const uint32_t base = 360000;
        for (int offset : {-2, -1, 0, int(RUN) - 1, int(RUN), int(RUN) + 1}) {
            for (uint32_t count : {1u, 2u, RUN + 4}) {
                Harness h(dut);
                h.reset();
                h.request(false, base, 1); h.drain();
                const uint32_t write_lba = base + offset;
                const bool overlap = write_lba < base + RUN && write_lba + count > base;
                std::vector<uint8_t> payload(count * 512, 0x72);
                h.write_request(write_lba, count, payload);
                check(!h.saw_error, "boundary write succeeds");
                const uint32_t commands = h.prov.commands;
                h.request(false, base, 1);
                h.expect_blocks(base, 1, "boundary write/read contents");
                if (RA_ON)
                    check((h.prov.commands != commands) == overlap,
                          "half-open overlap exactly determines invalidation");
            }
        }
    }

    CASE("failed_partial_write_cannot_leave_stale_cache");
    {
        Harness h(dut);
        h.reset();
        const uint32_t base = 370000;
        h.request(false, base, 1); h.drain();
        h.prov.fail_after_blocks = 1;
        std::vector<uint8_t> payload(1024, 0xb7);
        h.write_request(base, 2, payload);
        check(h.saw_error, "partial write reports failure");
        h.prov.fail_after_blocks = -1;
        const uint32_t commands = h.prov.commands;
        h.request(false, base, 1);
        h.expect_blocks(base, 1, "partial write invalidates old bytes");
        check(h.prov.commands > commands, "failed write still invalidates cache");
    }

    CASE("unrelated_write_preserves_in_flight_fill");
    {
        Harness h(dut);
        h.reset();
        const uint32_t base = 380000;
        h.request(false, base, 1); // deliberately do not drain the tail
        std::vector<uint8_t> payload(512, 0x21);
        h.write_request(base + RUN + 1, 1, payload);
        check(!h.saw_error, "unrelated write following live fill succeeds");
        const uint32_t commands = h.prov.commands;
        h.request(false, base, 1);
        h.expect_blocks(base, 1, "in-flight fill survives unrelated write");
        if (RA_ON) check(h.prov.commands == commands,
                         "unrelated write must not poison a live fill");
    }

    CASE("write_multi_coherency");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 310000;
        h.request(false, base, 4);
        h.expect_blocks(base, 4, "pre-write multi read");
        std::vector<uint8_t> payload(512 * 3);
        for (size_t i = 0; i < payload.size(); i++) payload[i] = (uint8_t)(i * 7 + 1);
        h.write_request(base + 1, 3, payload);
        check(!h.saw_error, "multi write reported an error");
        h.request(false, base, 4);
        h.expect_blocks(base, 4, "post-multi-write read");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("straddling_read");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 400000;
        h.request(false, base, 1);                 // caches base..base+RUN-1
        uint32_t sl = (RUN >= 2) ? (RUN - 2) : 0;  // last two blocks of it
        h.request(false, base + sl, 4);            // runs off the end
        check(!h.saw_error, "straddling read reported an error");
        h.expect_blocks(base + sl, 4, "straddling payload");
        // And the head of the run must still read correctly afterwards.
        h.request(false, base, 2);
        h.expect_blocks(base, 2, "post-straddle head read");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("seek_keeps_the_other_stream");
    if (RA_ON) {
        Harness h(dut);
        h.reset();
        uint32_t a = 500000, b = 900000;
        h.request(false, a, 1);                    // way X
        h.drain();
        h.request(false, b, 1);                    // way Y (a seek)
        h.drain();
        uint32_t c = h.prov.commands;
        checkf(c == 2, "two cold streams cost %u commands", c);
        h.request(false, a + 1, 1);                // must still hit
        h.expect_blocks(a + 1, 1, "stream A after a seek");
        checkf(h.prov.commands == c, "stream A was evicted by the seek");
        h.request(false, b + 1, 1);
        h.expect_blocks(b + 1, 1, "stream B after returning to A");
        checkf(h.prov.commands == c, "stream B was evicted by the return to A");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("interleaved_streams");
    {
        Harness h(dut);
        h.reset();
        uint32_t a = 600000, b = 700000;
        uint32_t c0 = h.prov.commands;
        for (uint32_t i = 0; i < 24; i++) {
            h.request(false, a + i, 1);
            h.expect_blocks(a + i, 1, "interleaved A");
            h.request(false, b + i, 1);
            h.expect_blocks(b + i, 1, "interleaved B");
        }
        uint32_t used = h.prov.commands - c0;
        printf("   interleaved 2x24 single-block reads -> %u provider commands\n", used);
        // Two streams, each covering 24 blocks in runs of RUN.  Anything
        // more than that is thrash -- which is what ONE way would do here
        // (48 commands, one per request).
        uint32_t ideal = 2 * ((24 + RUN - 1) / RUN);
        if (RA_ON)
            checkf(used <= ideal,
                   "two interleaved streams thrashed (%u commands, ideal %u)",
                   used, ideal);
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("oversized_request_bypasses");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 800000;
        uint32_t c0 = h.prov.commands;
        uint32_t big = RUN + 8;                    // > BLOCKS_PER_WAY
        h.request(false, base, big);
        check(!h.saw_error, "oversized read reported an error");
        h.expect_blocks(base, big, "oversized payload");
        checkf(h.prov.commands == c0 + 1, "oversized read issued %u commands",
               h.prov.commands - c0);
        checkf(h.prov.last_block_count == big,
               "provider saw block_count %u, expected the master's own %u",
               h.prov.last_block_count, big);
        checkf(h.prov.last_lba == base, "provider saw lba %u, expected %u",
               h.prov.last_lba, base);
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("capacity_clamp");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = h.prov.num_lbas - 3;
        h.request(false, base, 2);
        check(!h.saw_error, "read near the end of the volume errored");
        h.expect_blocks(base, 2, "end-of-volume payload");
        checkf(h.prov.max_lba_touched <= h.prov.num_lbas,
               "read-ahead addressed past the end of the volume (%u > %u)",
               h.prov.max_lba_touched, h.prov.num_lbas);
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("error_after_the_requested_extent");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 820000;
        h.prov.fail_after_blocks = 5;              // run dies at block 5
        uint32_t c0 = h.prov.commands;
        h.request(false, base, 1);                 // master only wants block 0
        check(!h.saw_error, "a bad sector 5 blocks ahead failed a good read");
        h.expect_blocks(base, 1, "payload ahead of the bad sector");
        // The run must NOT have been KEPT.  Let the failed run finish
        // first: while it is still in flight the module may legitimately
        // serve blocks that have already landed out of it (they are
        // CRC-checked bytes from the card), and that is not caching.
        h.drain();
        h.prov.fail_after_blocks = -1;
        h.request(false, base + 1, 1);
        checkf(h.prov.commands > c0 + 1, "a failed run was cached anyway");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("error_before_the_requested_extent");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 830000;
        h.prov.fail_after_blocks = 0;              // nothing lands
        h.request(false, base, 1);
        check(h.saw_error, "a dead read was reported as success");
        h.prov.fail_after_blocks = -1;
        // And the volume is still usable afterwards.
        h.request(false, base, 1);
        check(!h.saw_error, "the provider stayed broken after one error");
        h.expect_blocks(base, 1, "payload after recovering from an error");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("inval_drops_the_cache");
    if (RA_ON) {
        Harness h(dut);
        h.reset();
        uint32_t base = 840000;
        h.request(false, base, 1);
        uint32_t c = h.prov.commands;
        h.request(false, base + 1, 1);
        checkf(h.prov.commands == c, "warm run did not hit");
        h.drain();
        dut->inval = 1; h.tickn(4); dut->inval = 0; h.tickn(4);
        h.request(false, base + 2, 1);
        checkf(h.prov.commands == c + 1, "inval did not drop the cache");
        h.expect_blocks(base + 2, 1, "payload after inval");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("inval_mid_fetch_is_not_cached");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 850000;
        // Kick a fetch, let it get into the latency, then invalidate.
        dut->req_write = 0; dut->req_multi = 0;
        dut->req_lba = base; dut->req_block_count = 1;
        dut->req_go = 1; h.tick(); dut->req_go = 0;
        h.rx.clear(); h.saw_done = h.saw_error = false;
        h.tickn(1000);
        dut->inval = 1; h.tickn(4); dut->inval = 0;
        uint64_t n = 0;
        while (!h.saw_done && n++ < 40'000'000ULL) h.tick();
        check(h.saw_done, "an invalidated fetch never completed");
        if (RA_ON) check(h.saw_error, "an invalidated fetch reported success");
        // Recovery: the next read works and is not served from the
        // poisoned run.
        h.request(false, base, 1);
        check(!h.saw_error, "the module did not recover from a mid-fetch inval");
        h.expect_blocks(base, 1, "payload after a mid-fetch inval");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("reset_drops_the_cache");
    if (RA_ON) {
        Harness h(dut);
        h.reset();
        uint32_t base = 860000;
        h.request(false, base, 1);
        h.drain();
        uint32_t c = h.prov.commands;
        dut->rst = 1; h.tickn(6); dut->rst = 0; h.tickn(4);
        h.request(false, base + 1, 1);
        checkf(h.prov.commands == c + 1, "rst did not drop the cache");
        h.expect_blocks(base + 1, 1, "payload after rst");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("medium_change_drops_the_cache");
    if (RA_ON) {
        Harness h(dut);
        h.reset();
        uint32_t base = 870000;
        h.request(false, base, 1);
        h.drain();
        uint32_t c = h.prov.commands;
        dut->num_lbas = h.prov.num_lbas - 1;       // a different volume
        h.prov.num_lbas -= 1;
        h.tickn(4);
        h.request(false, base + 1, 1);
        checkf(h.prov.commands == c + 1, "a medium change did not drop the cache");
        h.expect_blocks(base + 1, 1, "payload after a medium change");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("consumer_backpressure");
    {
        Harness h(dut);
        h.reset();
        uint32_t base = 880000;
        h.rd_gap = 7;                              // one byte per 8 cycles
        h.request(false, base, 2);
        check(!h.saw_error, "a back-pressured read errored");
        h.expect_blocks(base, 2, "back-pressured payload");
        h.rd_gap = 0;
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("bounded_response_under_a_dead_consumer");
    if (RA_ON) {
        // rtl/vhdd.vh: a provider must answer `done | error` even if
        // rd_ready never rises.  Only reachable in the SERVE phase (the
        // fetch inherits the real provider's bound, deliberately).
        Harness h(dut);
        h.reset();
        uint32_t base = 890000;
        h.request(false, base, 1);                 // warm, so SERVE is next
        h.stall = true;
        uint64_t t = h.request(false, base + 1, 1, 8'000'000ULL);
        h.stall = false;
        check(h.saw_done, "a dead consumer parked the module forever");
        check(h.saw_error, "a dead consumer was reported as success");
        printf("   serve watchdog fired after %llu cycles\n",
               (unsigned long long)t);
        h.tickn(16);
        h.request(false, base + 1, 1);
        check(!h.saw_error, "the module did not recover from a serve timeout");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("first_byte_is_not_delayed_by_the_run");
    {
        // THE REGRESSION FENCE FOR THE 2026-09-15 REWRITE.
        //
        // An earlier shape of this module filled the WHOLE run into BRAM
        // before serving the master a single byte.  It was simpler, the
        // amortisation was identical, and every test in this file passed
        // -- and it broke the machine, because scsi.v's stuck-supply
        // watchdog (VH_STUCK_TIMEOUT, rtl/mac/scsi.v) is BUSY-INDEPENDENT:
        // it counts contiguous cycles of "multi-block read, ring empty,
        // no byte arriving" in S_VH_WAIT_RD and completes the command as
        // CHECK CONDITION when it expires.  A quiet fill is exactly that
        // pattern.  Caught by tb-scsi-sd-e2e-ra, not here.
        //
        // So: on a MISS, the first byte must reach the master about as
        // soon as the card produces it -- NOT after the whole run has
        // landed.  Measured against the provider's own command latency.
        Harness h(dut);
        h.reset();
        uint32_t base = 895000;
        h.drain();

        uint64_t guard = 0;
        while (dut->busy && guard++ < 40'000'000ULL) h.tick();
        h.rx.clear(); h.saw_done = h.saw_error = false;
        dut->req_write = 0; dut->req_multi = 0;
        dut->req_lba = base; dut->req_block_count = 1;
        dut->req_go = 1;
        uint64_t t0 = h.cycles;
        h.tick();
        dut->req_go = 0;
        uint64_t n = 0;
        while (h.rx.empty() && n++ < 40'000'000ULL) h.tick();
        uint64_t first_byte = h.cycles - t0;
        while (!h.saw_done && n++ < 40'000'000ULL) h.tick();

        // One whole run of streaming past the command latency would be
        // RUN*512 cycles at BYTE_CYCLES=1.  Allow a generous slack for
        // the pipeline and still fail the fill-then-serve shape outright.
        uint64_t budget = CMD_LATENCY_CYC + 512;
        printf("   first byte after %llu cycles (command latency %llu, "
               "a whole %u-block run would add %u)\n",
               (unsigned long long)first_byte,
               (unsigned long long)CMD_LATENCY_CYC, RUN, RUN * 512);
        checkf(first_byte <= budget,
               "first byte took %llu cycles, budget %llu -- the master was "
               "starved for the length of the fill",
               (unsigned long long)first_byte, (unsigned long long)budget);
        h.expect_blocks(base, 1, "miss payload");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("sequential_read_during_the_tail_follows_the_stream");
    if (RA_ON) {
        // The request right after a miss arrives while the rest of the
        // run is still streaming in.  It must be answered out of the run
        // IN FLIGHT -- no new command, and correct bytes -- including the
        // blocks that have not landed yet when it is accepted.
        Harness h(dut);
        h.reset();
        uint32_t base = 897000;
        h.drain();
        uint32_t c0 = h.prov.commands;
        h.request(false, base, 1);              // miss: starts the run
        checkf(h.prov.commands == c0 + 1, "the miss did not issue a command");
        // No drain: the tail is still landing.
        for (uint32_t i = 1; i < RUN; i++) {
            h.request(false, base + i, 1);
            h.expect_blocks(base + i, 1, "in-flight-run payload");
        }
        checkf(h.prov.commands == c0 + 1,
               "reads during the tail cost %u extra commands",
               h.prov.commands - (c0 + 1));
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("write_during_the_tail_kills_the_in_flight_run");
    {
        // The nastiest coherency case there is: the cache is mid-run, the
        // OS writes a block the card is still delivering to us, and then
        // reads it back.  The in-flight run holds the PRE-write bytes.
        Harness h(dut);
        h.reset();
        uint32_t base = 898000;
        h.drain();
        h.request(false, base, 1);              // miss: run in flight
        std::vector<uint8_t> payload(512);
        for (int i = 0; i < 512; i++) payload[i] = (uint8_t)(0x5A ^ (i * 3));
        // Block 2 is inside the run and almost certainly not landed yet.
        h.write_request(base + 2, 1, payload);
        check(!h.saw_error, "the write errored");
        h.request(false, base + 2, 1);
        h.expect_blocks(base + 2, 1, "read-back after a write during the tail");
        checkf(h.rx.size() == 512 && h.rx[0] == payload[0] &&
               h.rx[511] == payload[511],
               "read-back served the PRE-write copy from the in-flight run");
        // The neighbours of the written block must be current too.
        h.request(false, base + 1, 1);
        h.expect_blocks(base + 1, 1, "neighbour after a write during the tail");
    }

    // ═════════════════════════════════════════════════════════════════
    CASE("chk_probe_passthrough");
    {
        Harness h(dut);
        h.reset();
        dut->chk_lba = 1234; dut->chk_blocks = 8;
        dut->p_chk_ok = 1; dut->eval();
        checkf(dut->p_chk_lba == 1234 && dut->p_chk_blocks == 8,
               "extent probe was not forwarded (%u/%u)",
               (unsigned)dut->p_chk_lba, (unsigned)dut->p_chk_blocks);
        checkf(dut->chk_ok == 1, "chk_ok was not returned");
        dut->p_chk_ok = 0; dut->eval();
        checkf(dut->chk_ok == 0, "chk_ok did not follow the provider");
        dut->p_chk_ok = 1; dut->eval();
    }

    dut->final();
    delete dut;

    printf("=== vhdd_readahead tb (%s): %d passed, %d failed ===\n",
           RA_ON ? "cache ENABLED" : "cache BUILT OUT", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
