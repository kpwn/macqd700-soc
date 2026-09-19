// tb_fb_reader_ddr_chain.cpp -- drives the REAL fb_reader.v through the
// T14 VRAM-in-DDR chain (tb/tb_fb_reader_ddr_chain.v). See that file's
// header for what this closes (tb-vram-ddr-chain only drove
// scanout_ddr_reader.v's port directly from a C++ BFM, never the real
// fb_reader.v pclk<->vram_clk CDC + credit machinery video_top.v uses).
//
// Scenario: write a pixel-exact checkerboard pattern for one "frame"
// (64x64 = 4096 pixels) via a raw CPU-shaped AXI write port straight into
// the carveout, then drive fb_reader's pclk-side request stream for that
// same 4096-pixel span and check every returned byte against the
// pattern, in order. Two independent clock nets (pclk, core_clk) at a
// matched rate to genuinely exercise fb_reader.v's real CDC FIFOs
// without needing exact 148.5/100 MHz frequency fidelity (already
// covered by tb_axi_async_bridge's own unit tb), plus sim_mig_backend's
// STALL_ENABLE=1 jitter.
//
// RESOLVED (T14 round 5, via T15's independent investigation --
// scratchpad/briefs/t15-report.md). Two earlier rounds misdiagnosed
// this, in sequence, both now RETRACTED:
//
//   - Round 3 saw 240/4096 mismatches confined to the first ~4 lines
//     and attributed it to a scanout_line_fetch.v ring cold-start bug.
//     That was an artifact of the ORIGINAL checkerboard golden pattern
//     here -- (x^y)&1 ? 0x80:0 | (px&0x3F) -- which depends only on x
//     and the PARITY of y, so every even line was byte-for-byte
//     IDENTICAL to every other even line (and likewise odd lines),
//     making a real bug and a benign aliasing coincidence
//     indistinguishable.
//   - Round 4 fixed the golden pattern (`v = (y*37+x)&0xFF`, alias-
//     free) and found the true picture was far worse -- 4080/4096
//     mismatches, present on every line -- then, via direct
//     instrumentation, ruled OUT scanout_line_fetch.v, axi_ro_
//     priority_mux2.v, and fb_reader.v as the cause (every address
//     issued was exactly correct; the corruption was already present
//     at v_rd_data, scanout_ddr_reader's raw output, before fb_reader's
//     own logic ever touched it), and tentatively pinned it on
//     axi_ddr4_mig_bridge.v's 256<->128-bit width-conversion logic
//     and/or sim_mig_backend.v's read burst FSM.
//
// T15 (an independent task authorized specifically for those two
// files) built minimal standalone repros, then the exact failing
// integration test, and reproduced the identical 4080/4096 signature --
// then traced it via direct `$display` instrumentation of BOTH
// axi_ddr4_mig_bridge.v and sim_mig_backend.v to the ACTUAL root cause:
// this file's OWN `cpu_w_drive_and_latch()` write-phase driver (which
// seeds the golden framebuffer data the test later reads back) held
// `cpu_wvalid` asserted, unconditionally, for the ENTIRE lifetime of
// each pending write -- never deasserting it after a successful
// `cpu_wvalid && cpu_wready` handshake. That's an AXI4 W-channel
// master-contract violation (VALID must deassert, or advance to a
// genuinely NEW beat, immediately after a completed handshake) --
// `axi_async_bridge`'s W FIFO has no way to distinguish an intentional
// new beat from a buggy master re-presenting the old one, so it happily
// accepted the SAME already-consumed beat again on every subsequent
// cycle WREADY was also high (before this driver's own B-gated
// advance to the next transaction caught up) -- desynchronizing the W
// FIFO from the AW FIFO, so unrelated LATER AWs paired with stale,
// much-earlier W data. Every RTL module in the entire chain --
// axi_ddr4_mig_bridge.v, sim_mig_backend.v, axi_async_bridge.v,
// axi_vram_priority_mux3.v, scanout_ddr_reader.v, scanout_line_fetch.v,
// fb_reader.v -- is EXONERATED; the round-3 and round-4 RTL-focused
// diagnoses are both retracted. Fixed here (round 5) by adding a
// `w_done` field to `PendingW` and gating `cpu_wvalid` on `!w_done`,
// set the cycle a real handshake completes -- exactly as T15 verified.
// Result: 4096/4096, 0 mismatches, zero RTL changes required anywhere.
// See t15-report.md for the full instrumentation trail and t14-report.md
// ROUND 5 for the closing reconciliation.

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <verilated.h>
#include "Vtb_fb_reader_ddr_chain.h"
using DutT = Vtb_fb_reader_ddr_chain;

static DutT* dut = nullptr;
static uint64_t sim_time = 0;

static constexpr uint32_t VRAM_APERTURE_BASE = 0xF9000000u; // unused here (raw carveout write, see header)
static constexpr uint32_t CARVEOUT_BASE = 0x46000000u;
static constexpr int FRAME_PIXELS = 4096; // 64x64
static std::vector<uint8_t> golden(FRAME_PIXELS, 0);

// ─── Clocking: pclk and core_clk toggle at a matched 1:1 rate on
//     independent clock nets (genuinely separate domains -- fb_reader.v's
//     CDC FIFOs still cross real clock-net boundaries -- without needing
//     exact 148.5/100 MHz frequency fidelity, which tb_axi_async_bridge's
//     own unit tb already covers per the T13 brief's "reuse existing tb
//     clocking idioms" precedent). mig_clk also matches (mirrors
//     tb_vram_ddr_chain.v's convention).
static void eval() { dut->eval(); }

void cpu_w_drive_and_latch();
void scan_drive_and_latch();

static void tick() {
    dut->pclk = 0; dut->core_clk = 0; dut->mig_clk = 0; eval();
    cpu_w_drive_and_latch();
    scan_drive_and_latch();
    eval();
    dut->pclk = 1; dut->core_clk = 1; dut->mig_clk = 1; eval();
    sim_time++;
    eval();
}

// ─── Raw CPU AXI write driver (single-beat writes, straight into the
//     carveout -- see tb_fb_reader_ddr_chain.v header for why no swap is
//     needed here). ─────────────────────────────────────────────────────
//
// T15/T14-round-4-retraction fix: `cpu_wvalid` used to stay asserted
// unconditionally for the whole lifetime of `w_cur` (only cleared once
// `do_write_beat`'s caller nulled it out after BRESP) -- an AXI4
// W-channel master-contract violation (VALID must deassert, or advance
// to a genuinely new beat, immediately after a VALID&&READY handshake;
// this driver kept re-presenting the SAME already-accepted beat every
// subsequent cycle WREADY was also high, since nothing here ever
// tracked "have I already sent my one beat"). axi_async_bridge's W FIFO
// has no way to distinguish an intentional new beat from a buggy master
// re-offering the old one, so it happily accepted the duplicate,
// desynchronizing the W FIFO from the AW FIFO -- unrelated LATER AWs
// then paired with stale, much-earlier W data. This was root-caused by
// T15 (scratchpad/briefs/t15-report.md) via direct instrumentation of
// axi_ddr4_mig_bridge.v/sim_mig_backend.v, which starkly EXONERATED
// both files and every RTL module in this chain -- T14's round-3
// "cold-start ring bug" and round-4 "bridge width-conversion" diagnoses
// are BOTH retracted; the entire corruption was this test driver
// mis-seeding its own golden data before ever reading it back.
struct PendingW { uint32_t id; bool aw_done = false; bool w_done = false; bool done = false; };
static PendingW* w_cur = nullptr;
static uint32_t w_next_id = 1;

void cpu_w_drive_and_latch() {
    if (w_cur) {
        if (!w_cur->aw_done) { dut->cpu_awvalid = 1; }
        else dut->cpu_awvalid = 0;
        dut->cpu_wvalid = w_cur->w_done ? 0 : 1;
    } else {
        dut->cpu_awvalid = 0;
        dut->cpu_wvalid = 0;
    }
    dut->cpu_bready = 1;

    if (w_cur) {
        if (!w_cur->aw_done && dut->cpu_awvalid && dut->cpu_awready) w_cur->aw_done = true;
        if (!w_cur->w_done && dut->cpu_wvalid && dut->cpu_wready) {
            // Single-beat write: this WAS the one beat we meant to send.
            // Deassert cpu_wvalid from the NEXT cycle onward so the
            // bridge never sees it re-presented.
            w_cur->w_done = true;
        }
    }
    if (dut->cpu_bvalid && dut->cpu_bready && w_cur) {
        w_cur->done = true;
    }
}

static bool do_write_beat(uint32_t addr, const uint8_t* bytes16) {
    PendingW pw; pw.id = (w_next_id++) & 0xF;
    w_cur = &pw;
    dut->cpu_awid = pw.id; dut->cpu_awaddr = addr;
    dut->cpu_awlen = 0; dut->cpu_awsize = 4; dut->cpu_awburst = 1;
    for (int w = 0; w < 4; w++) {
        uint32_t v = bytes16[w*4] | (bytes16[w*4+1]<<8) | (bytes16[w*4+2]<<16) | (bytes16[w*4+3]<<24);
        dut->cpu_wdata[w] = v;
    }
    dut->cpu_wstrb = 0xFFFF;
    dut->cpu_wlast = 1;
    int guard = 20000;
    while (!pw.done && guard-- > 0) tick();
    w_cur = nullptr;
    return pw.done;
}

// ─── fb_reader pclk-side scanout driver ────────────────────────────────
static std::vector<uint32_t> scan_pending;
static std::vector<uint8_t> scan_got;
// Full 4-byte group as returned, parallel to scan_got.  Used by
// check_wide_group_lanes() below to hold the widened port to its contract for
// 4-byte-ALIGNED requests -- the lanes the 24bpp direct-colour path consumes
// and which scan_got (top lane only) would never notice being wrong.
static std::vector<uint32_t> scan_wide;
static uint32_t scan_next = 0, scan_stop = 0;
static bool scan_active = false;
static bool scan_req_this_cycle = false;
static bool scan_seen_valid = false;

void scan_drive_and_latch() {
    // fb_reader.v's own contract (see its header): s_rd_ready is a
    // backpressure hint -- s_rd_en may be held high while s_rd_ready is
    // low, but s_rd_addr must stay stable until the request is actually
    // ACCEPTED (s_rd_en && s_rd_ready). An earlier draft advanced
    // scan_next / pushed into scan_pending whenever it PRESENTED a
    // request, without checking s_rd_ready -- on any cycle fb_reader's
    // own request FIFO was momentarily full, that silently desynced this
    // driver's index tracking from what fb_reader actually accepted,
    // producing a false "shifted" pixel-mismatch pattern that had
    // nothing to do with the DDR reader path itself.
    scan_req_this_cycle = scan_active && (scan_next < scan_stop) && (scan_pending.size() < 16);
    dut->s_rd_en = scan_req_this_cycle ? 1 : 0;
    dut->s_rd_addr = scan_next;
    if (scan_req_this_cycle && dut->s_rd_ready) {
        scan_pending.push_back(scan_next);
        scan_next++;
    }
    if (dut->s_rd_valid) {
        if (!scan_pending.empty()) {
            // The streaming port returns a 4-byte group; this driver is a
            // CONSUMER of the single byte at the requested address, which is
            // the always-valid top lane [31:24].
            scan_got.push_back((uint8_t)((dut->s_rd_data >> 24) & 0xFFu));
            scan_wide.push_back((uint32_t)dut->s_rd_data);
            scan_pending.erase(scan_pending.begin());
        }
        scan_seen_valid = true;
    }
}

// Streaming-port contract check for the lower three lanes.  This driver
// requests consecutive byte addresses 0..FRAME_PIXELS-1, so index i in
// scan_wide is the response for address i.  For every 4-byte-ALIGNED address
// the whole returned group must be golden[i..i+3], big-endian (byte at i in
// [31:24]).  Unaligned addresses are architecturally don't-care below the top
// lane and are deliberately NOT checked.  Returns the mismatch count.
static int check_wide_group_lanes(const char* name) {
    int bad = 0;
    for (int i = 0; i + 3 < (int)scan_wide.size() && i + 3 < FRAME_PIXELS; i++) {
        if (i & 3) continue;
        const uint32_t want = ((uint32_t)golden[i + 0] << 24)
                            | ((uint32_t)golden[i + 1] << 16)
                            | ((uint32_t)golden[i + 2] <<  8)
                            |  (uint32_t)golden[i + 3];
        if (scan_wide[i] != want) {
            if (bad < 16)
                printf("  WIDE-MISMATCH(%s) addr=%d got=%08x want=%08x\n",
                       name, i, scan_wide[i], want);
            bad++;
        }
    }
    return bad;
}

// Runs the full write-then-scan scenario once against a fresh DUT.
// `post_reset_settle_cycles` controls how long we wait after core_rst
// deasserts (and cal_done completes) before issuing the first write --
// 0 is the genuine "cold start" case: the very first CPU write and,
// later, the very first fb_reader request land essentially immediately
// after reset releases. Added in round 5 per the coordinator's ask, now
// that the underlying corruption (the test's own W-channel driver bug,
// see header) is fixed and this variant is meaningful to run: it was
// skipped in round 4 because the "cold start ring bug" hypothesis it
// would have tested was already disproven by that round's own evidence
// (corruption present on every line, not just the first few) -- with
// the real bug now fixed, this confirms the fix isn't fragile to
// how quickly traffic starts after reset.
static bool run_scenario(int post_reset_settle_cycles, const char* name) {
    dut = new DutT();
    golden.assign(FRAME_PIXELS, 0);
    scan_pending.clear();
    scan_got.clear();
    scan_wide.clear();
    scan_next = 0; scan_stop = 0;
    scan_active = false;
    scan_req_this_cycle = false;
    scan_seen_valid = false;
    w_cur = nullptr;
    w_next_id = 1;
    sim_time = 0;

    // Reset.
    dut->core_rst = 1; dut->mig_rst = 1; dut->pclk_rst = 0;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->cpu_bready = 1;
    dut->s_rd_en = 0;
    dut->pclk = 0; dut->core_clk = 0; dut->mig_clk = 0;
    eval();
    for (int i = 0; i < 10; i++) tick();
    dut->mig_rst = 0;
    int waited = 0;
    while (dut->cal_done == 0 && waited < 300) { tick(); waited++; }
    dut->core_rst = 0;
    for (int i = 0; i < post_reset_settle_cycles; i++) tick();

    // Write a per-(line,offset)-unique pattern (raw, matching VRAM-native
    // byte order -- see header). NOTE (T14 round 4): the original
    // checkerboard formula here -- (x^y)&1 ? 0x80:0 | (px&0x3F) -- only
    // depends on x and (x^y)&1, i.e. on x and the PARITY of y. That means
    // every even line is byte-for-byte IDENTICAL to every other even
    // line, and likewise for odd lines: golden(line=L, off) ==
    // golden(line=L+2, off) for ALL L. With NUM_LINE_BUF=4, a ring slot
    // holding STALE data from 2 (or 4, 6, ...) lines ago is
    // indistinguishable from correct data under that pattern -- a real
    // stale-slot bug and a same-parity aliasing coincidence read
    // identically. This was traced (via scanout_line_fetch.v BEATWR/HIT
    // instrumentation) to be the actual explanation for the round-3
    // "cold start ring bug" finding: slot rdata patterns exactly matched
    // OTHER same-parity lines' real, freshly-fetched data, not stale
    // leftover content -- multiplier below (37, coprime with 256) maps
    // each (line, off) pair for line in 0..63 to a distinct byte per
    // fixed off, so any line/slot confusion within the ring's lifetime
    // becomes a genuine, visible mismatch instead of a coincidental
    // match.
    bool ok = true;
    for (int off = 0; off < FRAME_PIXELS; off += 16) {
        uint8_t beat[16];
        for (int b = 0; b < 16; b++) {
            int px = off + b;
            int x = px % 64, y = px / 64;
            uint8_t v = (uint8_t)((y * 37 + x) & 0xFF);
            beat[b] = v;
            golden[px] = v;
        }
        ok &= do_write_beat(CARVEOUT_BASE + off, beat);
    }
    if (!ok) {
        printf("  [FAIL] checkerboard write phase (AXI write timeout)\n");
        delete dut;
        return false;
    }

    // Stream the same span back through the REAL fb_reader.
    scan_next = 0; scan_stop = FRAME_PIXELS;
    scan_active = true;
    int guard = FRAME_PIXELS * 400 + 20000;
    while (((int)scan_got.size() < FRAME_PIXELS || !scan_pending.empty()) && guard-- > 0) {
        if (scan_next >= scan_stop) scan_active = false;
        tick();
    }
    scan_active = false;

    bool complete = ((int)scan_got.size() == FRAME_PIXELS) && scan_pending.empty();
    int mismatches = 0;
    for (int i = 0; i < (int)scan_got.size() && i < FRAME_PIXELS; i++) {
        if (scan_got[i] != golden[i]) {
            if (mismatches < 24) {
                printf("  MISMATCH px=%d (line=%d off=%d) got=%02x want=%02x\n",
                       i, i / 64, i % 64, scan_got[i], golden[i]);
            }
            mismatches++;
        }
    }

    const int wide_bad = check_wide_group_lanes(name);

    bool pass = complete && (mismatches == 0) && (wide_bad == 0)
                && !dut->underflow_sticky;
    printf("  %s: %d/%d pixels streamed, %d mismatches, "
           "%d wide-group mismatches, underflow_sticky=%d\n",
           name, (int)scan_got.size(), FRAME_PIXELS, mismatches, wide_bad,
           (int)dut->underflow_sticky);
    printf("[%s] %s\n", pass ? "PASS" : "FAIL", name);

    delete dut;
    return pass;
}

// ═══════════════════════════════════════════════════════════════════════
// frame_wrap_repaint -- full-branch-review C1, real fb_reader.v version.
// scanout_line_fetch.v's ring had no frame-boundary flush: `hit` served
// a resident lb_line == req_line match FOREVER, so a CPU repaint
// between frames (fb_reader restarting its request stream at offset 0,
// the real vsync-equivalent) could serve frame 1's stale bytes through
// the REAL fb_reader.v consumer, not just the AXI-level BFM
// tb-vram-ddr-chain drives. Paints pattern A, streams it through
// fb_reader, repaints with a different pattern B over the SAME address
// range, restarts fb_reader's request stream at offset 0 again, and
// verifies frame 2 shows B, not A's residue.
// ═══════════════════════════════════════════════════════════════════════
static bool run_frame_wrap_scenario() {
    dut = new DutT();
    golden.assign(FRAME_PIXELS, 0);
    scan_pending.clear();
    scan_got.clear();
    scan_wide.clear();
    scan_next = 0; scan_stop = 0;
    scan_active = false;
    scan_req_this_cycle = false;
    scan_seen_valid = false;
    w_cur = nullptr;
    w_next_id = 1;
    sim_time = 0;

    dut->core_rst = 1; dut->mig_rst = 1; dut->pclk_rst = 0;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->cpu_bready = 1;
    dut->s_rd_en = 0;
    dut->pclk = 0; dut->core_clk = 0; dut->mig_clk = 0;
    eval();
    for (int i = 0; i < 10; i++) tick();
    dut->mig_rst = 0;
    int waited = 0;
    while (dut->cal_done == 0 && waited < 300) { tick(); waited++; }
    dut->core_rst = 0;
    for (int i = 0; i < 50; i++) tick();

    auto paint = [&](int seed_mul, int seed_add) -> bool {
        bool ok = true;
        for (int off = 0; off < FRAME_PIXELS; off += 16) {
            uint8_t beat[16];
            for (int b = 0; b < 16; b++) {
                int px = off + b;
                int x = px % 64, y = px / 64;
                uint8_t v = (uint8_t)(((y * seed_mul + x) + seed_add) & 0xFF);
                beat[b] = v;
                golden[px] = v;
            }
            ok &= do_write_beat(CARVEOUT_BASE + off, beat);
        }
        return ok;
    };
    auto stream_and_check = [&](const char* name) -> bool {
        scan_pending.clear();
        scan_got.clear();
        scan_wide.clear();
        scan_next = 0; scan_stop = FRAME_PIXELS;
        scan_active = true;
        int guard = FRAME_PIXELS * 400 + 20000;
        while (((int)scan_got.size() < FRAME_PIXELS || !scan_pending.empty()) && guard-- > 0) {
            if (scan_next >= scan_stop) scan_active = false;
            tick();
        }
        scan_active = false;
        bool complete = ((int)scan_got.size() == FRAME_PIXELS) && scan_pending.empty();
        int mismatches = 0;
        for (int i = 0; i < (int)scan_got.size() && i < FRAME_PIXELS; i++) {
            if (scan_got[i] != golden[i]) {
                if (mismatches < 16)
                    printf("  MISMATCH(%s) px=%d (line=%d off=%d) got=%02x want=%02x\n",
                           name, i, i / 64, i % 64, scan_got[i], golden[i]);
                mismatches++;
            }
        }
        const int wide_bad = check_wide_group_lanes(name);
        printf("  %s: %d/%d pixels streamed, %d mismatches, "
               "%d wide-group mismatches\n",
               name, (int)scan_got.size(), FRAME_PIXELS, mismatches, wide_bad);
        return complete && (mismatches == 0) && (wide_bad == 0);
    };

    bool ok = true;
    ok &= paint(37, 0);   // frame 1 pattern A (same formula as run_scenario's baseline)
    if (!ok) { printf("  [FAIL] frame1 paint\n"); delete dut; return false; }
    ok &= stream_and_check("frame_wrap_repaint_frame1");

    ok &= paint(41, 13);  // frame 2 repaint: DIFFERENT pattern, same address range
    if (!ok) { printf("  [FAIL] frame2 repaint\n"); delete dut; return false; }
    ok &= stream_and_check("frame_wrap_repaint_frame2");

    ok &= paint(53, 29);  // frame 3 -- proves the flush keeps working across repeated wraps
    if (!ok) { printf("  [FAIL] frame3 repaint\n"); delete dut; return false; }
    ok &= stream_and_check("frame_wrap_repaint_frame3");

    bool underflow = dut->underflow_sticky != 0;
    if (underflow) printf("  [FAIL] underflow_sticky asserted\n");
    delete dut;
    return ok && !underflow;
}

// ═══════════════════════════════════════════════════════════════════════
// pclk_only_reset_midstream -- task #194, the RESET-DOMAIN SKEW.
//
// WHAT PRODUCTION ACTUALLY WIRES (three different nets, verified by read):
//   fb_reader .resetn   <- video_top resetn_bank[2]   pclk,     MMCM LOCKED
//   fb_reader .vram_rst <- vram_rd_rst                core_clk, soc_full_rst
//   scanout_ddr_reader .rst <- vram_rd_rst            core_clk, soc_full_rst
//
// fb_reader turns its pclk reset into a vram_clk-domain reset internally
// (`p_rst_vram` -> `vram_domain_rst`), which clears in_flight_cnt, the
// request pipe and BOTH CDC FIFOs.  scanout_ddr_reader sees none of that:
// its 64-entry request queue keeps every {line,offset} it was ever handed.
//
// So an MMCM relock -- a JTAG bitstream reload, a ref-clock blip, cold boot
// -- zeroes one side of a two-sided accounting and leaves the other loaded.
// After it, fb_reader believes nothing is outstanding and starts issuing
// again from zero credits, while the reader is still going to answer the
// OLD queue entries first, IN ORDER.  Every response fb_reader hands back
// is then the answer to a request several positions stale.
//
// This scenario drives ONLY `pclk_rst`, mid-stream, with traffic genuinely
// in flight (both counters non-zero), then restarts the stream from pixel 0
// and compares against the same golden data the healthy scenarios use.
// Correct behaviour: the restart returns golden[0..N-1].  It also reports
// both counters across the event, so the skew is visible as data and not
// only as a mismatch total.
static bool run_pclk_only_reset_scenario() {
    dut = new DutT();
    golden.assign(FRAME_PIXELS, 0);
    scan_pending.clear(); scan_got.clear(); scan_wide.clear();
    scan_next = 0; scan_stop = 0;
    scan_active = false; scan_req_this_cycle = false; scan_seen_valid = false;
    w_cur = nullptr; w_next_id = 1; sim_time = 0;

    dut->core_rst = 1; dut->mig_rst = 1; dut->pclk_rst = 0;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->cpu_bready = 1;
    dut->s_rd_en = 0;
    dut->pclk = 0; dut->core_clk = 0; dut->mig_clk = 0;
    eval();
    for (int i = 0; i < 10; i++) tick();
    dut->mig_rst = 0;
    int waited = 0;
    while (dut->cal_done == 0 && waited < 300) { tick(); waited++; }
    dut->core_rst = 0;
    for (int i = 0; i < 50; i++) tick();

    bool ok = true;
    for (int off = 0; off < FRAME_PIXELS; off += 16) {
        uint8_t beat[16];
        for (int b = 0; b < 16; b++) {
            int px = off + b;
            int x = px % 64, y = px / 64;
            uint8_t v = (uint8_t)((y * 37 + x) & 0xFF);
            beat[b] = v;
            golden[px] = v;
        }
        ok &= do_write_beat(CARVEOUT_BASE + off, beat);
    }
    if (!ok) { printf("  [FAIL] pclk_only_reset write phase\n"); delete dut; return false; }

    // Phase 1: stream far enough that the pipe is genuinely loaded.
    scan_next = 0; scan_stop = FRAME_PIXELS; scan_active = true;
    int guard = 200000;
    while ((int)scan_got.size() < 512 && guard-- > 0) tick();
    scan_active = false;

    // Let requests keep flowing right up to the reset edge -- we WANT
    // in-flight traffic.  Sample both accountings on the cycle before.
    const unsigned q_before  = (unsigned)dut->dbg_q_count;
    const unsigned inf_before = (unsigned)dut->dbg_in_flight_cnt;

    // Phase 2: pclk-domain reset ONLY.  core_rst stays low throughout, so
    // scanout_ddr_reader is never told anything happened.
    dut->pclk_rst = 1;
    for (int i = 0; i < 20; i++) tick();
    const unsigned q_during = (unsigned)dut->dbg_q_count;
    dut->pclk_rst = 0;
    for (int i = 0; i < 50; i++) tick();

    const unsigned q_after   = (unsigned)dut->dbg_q_count;
    const unsigned inf_after = (unsigned)dut->dbg_in_flight_cnt;
    printf("  reset-skew accounting: q_count %u -> %u (during) -> %u,"
           "  in_flight_cnt %u -> %u\n",
           q_before, q_during, q_after, inf_before, inf_after);

    // Phase 3: restart the stream from scratch and check it.
    scan_pending.clear(); scan_got.clear(); scan_wide.clear();
    scan_next = 0; scan_stop = FRAME_PIXELS; scan_active = true;
    guard = FRAME_PIXELS * 400 + 20000;
    while (((int)scan_got.size() < FRAME_PIXELS || !scan_pending.empty()) && guard-- > 0) {
        if (scan_next >= scan_stop) scan_active = false;
        tick();
    }
    scan_active = false;

    bool complete = ((int)scan_got.size() == FRAME_PIXELS) && scan_pending.empty();
    int mismatches = 0;
    for (int i = 0; i < (int)scan_got.size() && i < FRAME_PIXELS; i++) {
        if (scan_got[i] != golden[i]) {
            if (mismatches < 12)
                printf("  MISMATCH px=%d got=%02x want=%02x\n", i, scan_got[i], golden[i]);
            mismatches++;
        }
    }
    bool pass = complete && (mismatches == 0);
    printf("  pclk_only_reset_midstream: %d/%d streamed, %d mismatches, complete=%d\n",
           (int)scan_got.size(), FRAME_PIXELS, mismatches, (int)complete);
    delete dut;
    return pass;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    int n_pass = 0, n_fail = 0;

    // Baseline: matches the original scenario's settle margin.
    if (run_scenario(50, "fb_reader_through_ddr_chain_pixel_exact")) n_pass++; else n_fail++;

    // Cold-start variant: essentially zero settle after core_rst
    // deasserts (and cal_done completes) -- the first CPU write and,
    // later, fb_reader's own first request land as soon as reset
    // structurally allows.
    if (run_scenario(0, "fb_reader_through_ddr_chain_cold_start")) n_pass++; else n_fail++;

    // Full-branch-review C1: two-frame repaint through the REAL
    // fb_reader.v consumer (not just the AXI-level BFM).
    {
        bool r = run_frame_wrap_scenario();
        printf("[%s] fb_reader_through_ddr_chain_frame_wrap_repaint\n", r ? "PASS" : "FAIL");
        if (r) n_pass++; else n_fail++;
    }

    // Task #194: the pclk-only (MMCM-relock-shaped) reset.
    {
        bool r = run_pclk_only_reset_scenario();
        printf("[%s] fb_reader_through_ddr_chain_pclk_only_reset_midstream\n",
               r ? "PASS" : "FAIL");
        if (r) n_pass++; else n_fail++;
    }

    printf("\ntb_fb_reader_ddr_chain: %d PASS / %d FAIL\n", n_pass, n_fail);
    return n_fail ? 1 : 0;
}
