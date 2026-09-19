// tb_vhdd_net.cpp — Ethernet-backed vHDD provider (transaction layer).
//
// The testbench plays BOTH ends: the vhdd master above (scsi.v's role) and
// the framer plus host daemon below.  What it is really checking is that the
// module survives the things a network does that a disk does not -- replies
// that never come, replies that come twice, replies that come late and stale
// -- without ever violating the two bounded-response obligations in
// rtl/vhdd.vh: never wedge `busy`, and never report success for data that did
// not arrive.
//
// A positive control runs first with the host disconnected, so that every
// data assertion is proven capable of failing before any of them is trusted.

#include "Vvhdd_net.h"
#include "verilated.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <map>

static Vvhdd_net *dut;
static uint64_t cycles = 0;
static int failures = 0;
static bool positive_control = false;

#define CHECK(cond, ...) do { if (!(cond)) { \
    std::printf("  FAIL: "); std::printf(__VA_ARGS__); \
    std::printf("   [%s:%d]\n", __FILE__, __LINE__); failures++; } } while (0)

// ── the simulated volume ─────────────────────────────────────────────
// Byte at (lba,off) is deterministic, so a wrong block is a wrong VALUE,
// not just a wrong length -- an off-by-one window would still deliver 512
// plausible bytes and only a content check catches it.
static uint8_t volume_byte(uint32_t lba, unsigned off) {
    return uint8_t((lba * 7u + off * 31u + (off >> 8) * 13u) & 0xff);
}

struct HostModel {
    // request capture
    bool     saw_request = false;
    uint8_t  op = 0;
    uint32_t lba = 0;
    uint16_t count = 0;
    uint16_t tag = 0;
    std::vector<uint8_t> write_payload;

    // behaviour knobs
    int  drop_next = 0;          // drop this many replies outright
    bool inject_stale = false;   // answer once with a wrong tag first
    bool stale_active = false;   // the stale answer is in flight now
    bool stale_payload = false;  // ...and its (wrong) payload is streaming
    uint8_t status = 0x00;
    bool connected = true;

    // observations
    int  requests_seen = 0;
    int  max_blocks_seen = 0;
    std::vector<std::pair<uint32_t,uint16_t>> request_log;
    std::map<uint32_t, std::vector<uint8_t>> written;

    // reply state machine
    bool     replying = false;
    bool     reply_hdr_done = false;
    unsigned reply_index = 0;
    std::vector<uint8_t> reply_payload;
    uint16_t reply_tag = 0;
    int      delay = 0;
};

static HostModel host;

static void drive_host() {
    // ── request channel ──────────────────────────────────────────────
    dut->net_req_ready = host.connected && !host.replying;
    if (dut->net_req_valid && dut->net_req_ready) {
        host.saw_request = true;
        host.op    = dut->net_req_op;
        host.lba   = dut->net_req_lba;
        host.count = dut->net_req_block_count;
        host.tag   = dut->net_req_tag;
        host.requests_seen++;
        if (host.count > host.max_blocks_seen) host.max_blocks_seen = host.count;
        host.request_log.push_back({host.lba, host.count});
        host.write_payload.clear();
        if (host.op == 0x00) {           // read: build the reply now
            host.reply_payload.clear();
            for (unsigned b = 0; b < host.count; b++)
                for (unsigned o = 0; o < 512; o++)
                    host.reply_payload.push_back(volume_byte(host.lba + b, o));
            host.replying = true;
            host.reply_hdr_done = false;
            host.reply_index = 0;
            host.reply_tag = host.tag;
            host.stale_active = host.inject_stale;
            host.inject_stale = false;
            host.delay = 3;
        }
    }

    // ── write payload ────────────────────────────────────────────────
    dut->net_req_payload_tready = host.connected && ((cycles % 7) != 3);
    if (dut->net_req_payload_tvalid && dut->net_req_payload_tready) {
        host.write_payload.push_back(dut->net_req_payload_tdata);
        if (dut->net_req_payload_tlast) {
            for (unsigned b = 0; b < host.count; b++) {
                std::vector<uint8_t> blk(host.write_payload.begin() + b * 512,
                                         host.write_payload.begin() + (b + 1) * 512);
                host.written[host.lba + b] = blk;
            }
            host.reply_payload.clear();
            host.replying = true;
            host.reply_hdr_done = false;
            host.reply_index = 0;
            host.reply_tag = host.tag;
            host.delay = 3;
        }
    }

    // ── reply channel ────────────────────────────────────────────────
    dut->net_reply_valid = 0;
    dut->net_reply_payload_tvalid = 0;
    if (host.replying && host.delay > 0) { host.delay--; return; }

    if (host.replying && !host.reply_hdr_done) {
        if (host.drop_next > 0) {         // silently lose this reply
            host.drop_next--;
            host.replying = false;
            return;
        }
        dut->net_reply_valid  = 1;
        dut->net_reply_tag    = host.stale_active ? uint16_t(host.reply_tag ^ 0x5a5a)
                                                  : host.reply_tag;
        dut->net_reply_status = host.status;
        dut->eval();
        if (dut->net_reply_ready) {
            if (host.stale_active) {
                // A stale reply must carry the WRONG DATA to be a real test.
                // A late duplicate of the correct answer proves nothing --
                // accepting it yields the same bytes, so a module with NO tag
                // check would still pass.  The case that matters is a reply
                // for an EARLIER window arriving while we wait for this one,
                // which if accepted delivers the wrong block under good status.
                host.reply_payload.clear();
                for (unsigned b = 0; b < host.count; b++)
                    for (unsigned o = 0; o < 512; o++)
                        host.reply_payload.push_back(volume_byte(host.lba + b + 9999, o));
                host.reply_hdr_done = true;
                host.stale_payload = true;
            } else if (host.op == 0x00 && host.status == 0) {
                host.reply_hdr_done = true;
            } else {
                host.replying = false;
            }
        }
        return;
    }

    if (host.replying && host.reply_hdr_done) {
        if (host.reply_index < host.reply_payload.size()) {
            dut->net_reply_payload_tvalid = 1;
            dut->net_reply_payload_tdata  = host.reply_payload[host.reply_index];
            dut->net_reply_payload_tlast  =
                (host.reply_index + 1 == host.reply_payload.size());
            dut->eval();
            if (dut->net_reply_payload_tready) {
                host.reply_index++;
                if (host.reply_index == host.reply_payload.size()) {
                    if (host.stale_payload) {
                        host.stale_payload = false;
                        host.stale_active  = false;
                        host.reply_hdr_done = false;
                        host.reply_index = 0;
                        host.reply_payload.clear();
                        for (unsigned b = 0; b < host.count; b++)
                            for (unsigned o = 0; o < 512; o++)
                                host.reply_payload.push_back(volume_byte(host.lba + b, o));
                        host.delay = 2;
                    } else {
                        host.replying = false;
                    }
                }
            }
        } else {
            host.replying = false;
        }
    }
}

static void tick() {
    drive_host();
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    cycles++;
}

static void reset_dut() {
    host = HostModel();
    if (positive_control) host.connected = false;
    dut->rst = 1;
    dut->req_go = 0; dut->req_write = 0; dut->req_multi = 0;
    dut->req_lba = 0; dut->req_block_count = 0;
    dut->rd_ready = 0; dut->wr_valid = 0; dut->wr_data = 0; dut->wr_avail = 1;
    dut->num_lbas = 1000000;
    dut->net_req_ready = 0; dut->net_req_payload_tready = 0;
    dut->net_reply_valid = 0; dut->net_reply_payload_tvalid = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    for (int i = 0; i < 4; i++) tick();
}

static void start(bool write, uint32_t lba, uint16_t blocks) {
    dut->req_write = write; dut->req_multi = blocks > 1;
    dut->req_lba = lba; dut->req_block_count = blocks;
    dut->req_go = 1; tick(); dut->req_go = 0;
}

// ── a read transfer, with the master pacing itself ───────────────────
static bool run_read(uint32_t lba, uint16_t blocks, std::vector<uint8_t> &out,
                     bool &err, unsigned limit = 400000, bool stall = false) {
    out.clear(); err = false;
    start(false, lba, blocks);
    for (unsigned n = 0; n < limit; n++) {
        // Exercise real back-pressure: refuse bytes on a duty cycle.
        dut->rd_ready = stall ? 0 : ((cycles % 4) != 1);
        dut->eval();
        if (dut->rd_valid && dut->rd_ready) out.push_back(dut->rd_data);
        tick();
        if (dut->done) { err = dut->error; return true; }
    }
    return false;
}

static bool run_write(uint32_t lba, uint16_t blocks, bool &err,
                      unsigned limit = 400000, bool starve = false) {
    err = false;
    unsigned produced = 0;
    const unsigned total = unsigned(blocks) * 512;
    start(true, lba, blocks);
    for (unsigned n = 0; n < limit; n++) {
        // wr_avail is real back-pressure; wr_data is producer-paced.
        dut->wr_avail = starve ? 0 : ((cycles % 5) != 2);
        dut->wr_valid = 1;
        dut->wr_data = (produced < total)
                       ? volume_byte(lba + produced / 512, produced % 512) : 0;
        dut->eval();
        if (dut->wr_ready) produced++;
        tick();
        if (dut->done) { err = dut->error; return true; }
    }
    return false;
}

static void expect_read(uint32_t lba, uint16_t blocks, const char *what) {
    std::vector<uint8_t> got; bool err = false;
    CHECK(run_read(lba, blocks, got, err), "%s: never completed", what);
    CHECK(!err, "%s: reported error", what);
    CHECK(got.size() == size_t(blocks) * 512, "%s: got %zu bytes, want %u",
          what, got.size(), unsigned(blocks) * 512);
    if (got.size() == size_t(blocks) * 512) {
        for (unsigned b = 0; b < blocks; b++)
            for (unsigned o = 0; o < 512; o++) {
                const uint8_t want = volume_byte(lba + b, o);
                if (got[b * 512 + o] != want) {
                    CHECK(false, "%s: block %u offset %u = %02x want %02x",
                          what, b, o, got[b * 512 + o], want);
                    return;
                }
            }
    }
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    positive_control = (argc > 1 && std::string(argv[1]) == "control");
    dut = new Vvhdd_net;

    if (positive_control)
        std::printf("=== POSITIVE CONTROL: host disconnected, data checks MUST fail ===\n");

    // ── extent probe ─────────────────────────────────────────────────
    reset_dut();
    dut->num_lbas = 1000;
    dut->chk_lba = 0;   dut->chk_blocks = 1000; dut->eval();
    CHECK(dut->chk_ok, "exact-capacity extent must be addressable");
    dut->chk_lba = 0;   dut->chk_blocks = 1001; dut->eval();
    CHECK(!dut->chk_ok, "over-capacity extent must fail closed");
    dut->chk_lba = 999; dut->chk_blocks = 1;    dut->eval();
    CHECK(dut->chk_ok, "last block must be addressable");
    dut->chk_lba = 0;   dut->chk_blocks = 0;    dut->eval();
    CHECK(!dut->chk_ok, "zero-length extent must fail closed");
    // The 33-bit sum matters: a 32-bit compare wraps and calls this fine.
    dut->chk_lba = 0xFFFFFF00; dut->chk_blocks = 0x1000; dut->eval();
    CHECK(!dut->chk_ok, "extent that wraps 32 bits must fail closed");
    dut->num_lbas = 1000000;

    // ── reads ────────────────────────────────────────────────────────
    reset_dut();
    expect_read(0, 1, "single-block read");

    reset_dut();
    expect_read(42, 2, "two-block read (one full window)");

    // 5 blocks is deliberately not a multiple of the window: it must split
    // 2+2+1, and the short tail is where a length bug hides.
    reset_dut();
    expect_read(100, 5, "five-block read (2+2+1 windows)");
    CHECK(host.max_blocks_seen <= 2, "never request more than 2 blocks (saw %d)",
          host.max_blocks_seen);
    CHECK(host.request_log.size() == 3, "5 blocks must be 3 windows, saw %zu",
          host.request_log.size());
    if (host.request_log.size() == 3) {
        CHECK(host.request_log[0] == std::make_pair(100u, uint16_t(2)), "window 0 wrong");
        CHECK(host.request_log[1] == std::make_pair(102u, uint16_t(2)), "window 1 wrong");
        CHECK(host.request_log[2] == std::make_pair(104u, uint16_t(1)), "window 2 wrong");
    }

    // ── writes ───────────────────────────────────────────────────────
    reset_dut();
    {
        bool err = false;
        CHECK(run_write(200, 3, err), "three-block write never completed");
        CHECK(!err, "three-block write reported error");
        CHECK(host.max_blocks_seen <= 2, "write requested >2 blocks (%d)",
              host.max_blocks_seen);
        for (unsigned b = 0; b < 3; b++) {
            auto it = host.written.find(200 + b);
            CHECK(it != host.written.end(), "write block %u never reached the host", b);
            if (it != host.written.end())
                for (unsigned o = 0; o < 512; o++)
                    if (it->second[o] != volume_byte(200 + b, o)) {
                        CHECK(false, "write block %u offset %u = %02x want %02x",
                              b, o, it->second[o], volume_byte(200 + b, o));
                        break;
                    }
        }
    }

    // ── a dropped reply must be retransmitted, not fatal ─────────────
    reset_dut();
    host.drop_next = 1;
    expect_read(300, 2, "read surviving one dropped reply");
    CHECK(host.requests_seen >= 2, "dropped reply must be retransmitted (saw %d)",
          host.requests_seen);

    // ── a late reply carrying a STALE tag must be ignored ────────────
    // This is the one that silently returns the WRONG BLOCK if tags are not
    // checked, so it is worth its own case rather than folding into the drop.
    reset_dut();
    host.inject_stale = true;
    expect_read(400, 2, "read ignoring a stale-tag reply");

    // ── exhausted retries must FAIL, not hang ────────────────────────
    reset_dut();
    host.drop_next = 1000;
    {
        std::vector<uint8_t> got; bool err = false;
        const uint64_t t0 = cycles;
        CHECK(run_read(500, 2, got, err), "exhausted retries must still complete");
        CHECK(err, "exhausted retries must report error");
        // Assert WHICH mechanism ended it.  Without this the stall watchdog
        // rescues the test and the retry budget is never actually exercised
        // -- a module that retried forever would still "pass" because the
        // watchdog eventually fired.  MAX_RETRIES=3 means exactly 4 attempts
        // (the original plus three retransmissions), and it must land well
        // inside STALL_TIMEOUT.
        CHECK(host.requests_seen == 4,
              "expected 4 attempts (1 + MAX_RETRIES), saw %d", host.requests_seen);
        CHECK(cycles - t0 < 20000,
              "retry budget must end the transfer before the stall watchdog "
              "(took %llu cycles)", (unsigned long long)(cycles - t0));
    }

    // ── a host error status must surface as an error ─────────────────
    reset_dut();
    host.status = 0x02;
    {
        std::vector<uint8_t> got; bool err = false;
        CHECK(run_read(600, 1, got, err), "error status must complete");
        CHECK(err, "error status must be reported as error");
    }

    // ── bounded response: a master that stalls forever still gets done ─
    // Without this the provider parks `busy` and the volume is dead until
    // reset -- strictly worse than an error the master could retry.
    reset_dut();
    {
        std::vector<uint8_t> got; bool err = false;
        CHECK(run_read(700, 2, got, err, 400000, /*stall=*/true),
              "a permanently stalled master must still be answered");
        CHECK(err, "stall timeout must report error");
    }

    if (positive_control) {
        if (failures == 0) {
            std::printf("POSITIVE CONTROL DID NOT FAIL - the assertions are inert.\n");
            return 1;
        }
        std::printf("positive control failed as required (%d checks) - assertions are live\n",
                    failures);
        return 0;
    }
    if (failures) { std::printf("FAILED: %d check(s)\n", failures); return 1; }
    std::printf("all vhdd_net tests passed\n");
    delete dut;
    return 0;
}
