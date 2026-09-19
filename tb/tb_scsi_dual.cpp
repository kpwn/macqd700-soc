// tb_scsi_dual.cpp — two-target SCSI + vhdd_mux routing testbench.
//
// Build/run via:   make tb-scsi-dual
//
// The DUT (tb/tb_scsi_dual.v) is one scsi.v answering to ID 0 and ID 1,
// a vhdd_mux, and two behavioural vhdd providers whose data signatures
// differ in EVERY byte:
//
//   provider A: block N, byte i == 0xA0 ^ (uint8)(N + i)
//   provider B: block N, byte i == 0xB0 ^ (uint8)(N + i)
//
// The initiator side is driven exactly the way tb_scsi.cpp drives it —
// the NCR 5380 register file through Selection / Command / Data /
// Status / Message-In — so a regression here is a regression in the same
// terms the single-target suite already speaks.
//
// Scenarios:
//   1. read6_id0_routes_to_provider_a  — all 512 bytes are A's signature.
//   2. read6_id1_routes_to_provider_b  — all 512 bytes are B's signature.
//   3. write6_id1_roundtrip_no_crosstalk
//                                      — WRITE(6) ID 1 LBA 3, read back,
//                                        and prove A's block 3 is
//                                        byte-for-byte untouched.
//   4. read6_id1_two_blocks            — 2-block READ(6) returns the
//                                        right blocks in the right order.
//   5. dev_en_01_hides_id1             — ID 1 absent, ID 0 still works.
//   6. dev_en_10_hides_id0             — mirror image.
//   7. capacity_routing_after_id1_selection — a read to ID 0 after a
//      transaction on ID 1 must be range-checked against A's capacity,
//      not B's (vhdd_mux routes m_num_lbas by dev_sel).

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_dual.h"

static Vtb_scsi_dual* dut      = nullptr;
static uint64_t       sim_time = 0;
static int            n_pass   = 0;
static int            n_fail   = 0;

// ── Clock helpers ──────────────────────────────────────────────────────
static void tick_n(int n = 1) {
    for (int i = 0; i < n; i++) {
        dut->clk = 1;
        dut->eval();
        dut->clk = 0;
        dut->eval();
        sim_time++;
    }
}

static void reset(uint8_t dev_en = 0x3) {
    dut->rst = 1;
    dut->dev_en = dev_en;
    dut->pb_addr = 0; dut->pb_wdata = 0; dut->pb_wr = 0; dut->pb_rd = 0;
    dut->probe_dev = 0; dut->probe_addr = 0;
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

// ── Bus helpers (same shape as tb_scsi.cpp) ────────────────────────────
static void bus_write(uint8_t addr, uint8_t data) {
    dut->pb_addr = addr & 0x7;
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

// Read a byte straight out of a provider's storage (harness probe port).
static uint8_t probe(int dev, uint32_t block, uint32_t off) {
    dut->probe_dev = dev ? 1 : 0;
    dut->probe_addr = (uint16_t)(block * 512u + off);
    dut->eval();
    return dut->probe_data & 0xFF;
}

// ── 5380 Initiator-Command bit constants ──────────────────────────────
static constexpr uint8_t IC_RST = 0x80;
static constexpr uint8_t IC_ACK = 0x10;
static constexpr uint8_t IC_SEL = 0x04;
static constexpr uint8_t IC_DB  = 0x01;

// Current SCSI Bus Status bits
static constexpr uint8_t SR_BSY = 0x40;
static constexpr uint8_t SR_REQ = 0x20;
static constexpr uint8_t SR_MSG = 0x10;
static constexpr uint8_t SR_CD  = 0x08;
static constexpr uint8_t SR_IO  = 0x04;

// Bus and Status bits
static constexpr uint8_t BS_IRQ = 0x10;
static constexpr uint8_t BS_DRQ = 0x40;

// ── Selection helpers ─────────────────────────────────────────────────
static bool do_select(uint8_t target_id, int timeout = 64) {
    bus_write(0, (uint8_t)(1 << target_id));
    bus_write(1, IC_SEL | IC_DB);
    for (int i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_BSY) return true;
        tick_n(1);
    }
    return false;
}

static void drop_sel() {
    bus_write(1, IC_DB);
    tick_n(2);
}

static bool send_cdb_byte(uint8_t b, int timeout = 64) {
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_REQ) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(0, b);
    bus_write(1, IC_ACK | IC_DB);
    for (i = 0; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_REQ)) break;
        tick_n(1);
    }
    if (i == timeout) return false;
    bus_write(1, IC_DB);
    tick_n(2);
    return true;
}

static bool recv_in_byte(uint8_t& out, int timeout = 4096) {
    int i = 0;
    for (; i < timeout; i++) {
        uint8_t s = bus_read(4);
        if (s & SR_REQ) break;
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

static bool push_out_byte(uint8_t b, int timeout = 4096) {
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

static bool recv_status_and_msg(uint8_t& status, uint8_t& msg) {
    if (!recv_in_byte(status)) return false;
    if (!recv_in_byte(msg))    return false;
    for (int i = 0; i < 64; i++) {
        uint8_t s = bus_read(4);
        if (!(s & SR_BSY)) return true;
        tick_n(1);
    }
    return false;
}

static bool send_cdb6(const uint8_t cdb[6], const char* tag) {
    for (int i = 0; i < 6; i++) {
        char lbl[64]; snprintf(lbl, sizeof(lbl), "%s cdb[%d]", tag, i);
        if (!send_cdb_byte(cdb[i])) {
            printf("  FAIL %s (t=%lu)\n", lbl, (unsigned long)sim_time);
            return false;
        }
    }
    return true;
}

// Expected byte of a provider's block N.
static uint8_t sig_byte(uint8_t sig, uint32_t block, uint32_t off) {
    return (uint8_t)(sig ^ (uint8_t)(block + off));
}

// ═══════════════════════════════════════════════════════════════════════
// Shared body: READ(6) `blocks` blocks from `lba` on `target_id`,
// returning all bytes read.
// ═══════════════════════════════════════════════════════════════════════
static bool do_read6(uint8_t target_id, uint32_t lba, uint8_t blocks,
                     std::vector<uint8_t>& out, const char* tag) {
    char lbl[80];
    snprintf(lbl, sizeof(lbl), "%s selection id=%u", tag, target_id);
    CHECK_TRUE(lbl, do_select(target_id));
    drop_sel();
    uint8_t cdb[6] = {0x08,
                      (uint8_t)((lba >> 16) & 0x1F),
                      (uint8_t)((lba >> 8) & 0xFF),
                      (uint8_t)(lba & 0xFF),
                      blocks, 0x00};
    if (!send_cdb6(cdb, tag)) return false;

    out.assign((size_t)blocks * 512u, 0);
    for (size_t i = 0; i < out.size(); i++) {
        if (!recv_in_byte(out[i])) {
            printf("  FAIL %s recv[%zu] (t=%lu)\n", tag, i,
                   (unsigned long)sim_time);
            return false;
        }
    }
    uint8_t status = 0xFF, msg = 0xFF;
    snprintf(lbl, sizeof(lbl), "%s status/msg", tag);
    CHECK_TRUE(lbl, recv_status_and_msg(status, msg));
    snprintf(lbl, sizeof(lbl), "%s status GOOD", tag);
    CHECK_EQ(lbl, status, 0x00);
    snprintf(lbl, sizeof(lbl), "%s msg COMPLETE", tag);
    CHECK_EQ(lbl, msg, 0x00);
    return true;
}

static bool do_write6(uint8_t target_id, uint32_t lba, uint8_t blocks,
                      const std::vector<uint8_t>& data, const char* tag) {
    char lbl[80];
    snprintf(lbl, sizeof(lbl), "%s selection id=%u", tag, target_id);
    CHECK_TRUE(lbl, do_select(target_id));
    drop_sel();
    uint8_t cdb[6] = {0x0A,
                      (uint8_t)((lba >> 16) & 0x1F),
                      (uint8_t)((lba >> 8) & 0xFF),
                      (uint8_t)(lba & 0xFF),
                      blocks, 0x00};
    if (!send_cdb6(cdb, tag)) return false;
    for (size_t i = 0; i < data.size(); i++) {
        if (!push_out_byte(data[i])) {
            printf("  FAIL %s push[%zu] (t=%lu)\n", tag, i,
                   (unsigned long)sim_time);
            return false;
        }
    }
    uint8_t status = 0xFF, msg = 0xFF;
    snprintf(lbl, sizeof(lbl), "%s status/msg", tag);
    CHECK_TRUE(lbl, recv_status_and_msg(status, msg));
    snprintf(lbl, sizeof(lbl), "%s status GOOD", tag);
    CHECK_EQ(lbl, status, 0x00);
    snprintf(lbl, sizeof(lbl), "%s msg COMPLETE", tag);
    CHECK_EQ(lbl, msg, 0x00);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 1 — READ(6) LBA 0 from ID 0 returns provider A's signature.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read6_id0_routes_to_provider_a() {
    reset(0x3);
    std::vector<uint8_t> got;
    if (!do_read6(0, 0, 1, got, "A/READ6")) return false;
    CHECK_EQ("A/READ6 length", got.size(), 512u);
    for (uint32_t i = 0; i < 512; i++) {
        if (got[i] != sig_byte(0xA0, 0, i)) {
            printf("  FAIL A/READ6 byte[%u]: got 0x%02x, expected 0x%02x "
                   "(provider B would be 0x%02x) (t=%lu)\n",
                   i, got[i], sig_byte(0xA0, 0, i), sig_byte(0xB0, 0, i),
                   (unsigned long)sim_time);
            return false;
        }
    }
    CHECK_EQ("A/READ6 dev_sel latched to A", dut->dev_sel, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2 — READ(6) LBA 0 from ID 1 returns provider B's signature.
// This is the assertion a "dev_sel stuck at 0" bug must break.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read6_id1_routes_to_provider_b() {
    reset(0x3);
    std::vector<uint8_t> got;
    if (!do_read6(1, 0, 1, got, "B/READ6")) return false;
    CHECK_EQ("B/READ6 length", got.size(), 512u);
    for (uint32_t i = 0; i < 512; i++) {
        if (got[i] != sig_byte(0xB0, 0, i)) {
            printf("  FAIL B/READ6 byte[%u]: got 0x%02x, expected 0x%02x "
                   "(provider A would be 0x%02x) (t=%lu)\n",
                   i, got[i], sig_byte(0xB0, 0, i), sig_byte(0xA0, 0, i),
                   (unsigned long)sim_time);
            return false;
        }
    }
    CHECK_EQ("B/READ6 dev_sel latched to B", dut->dev_sel, 1);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 3 — WRITE(6) to ID 1 LBA 3, read it back, and prove provider
// A's block 3 was not touched.  A's block is read through the harness
// probe port, i.e. out of A's own storage, not through a second SCSI
// round trip on the routing logic under test.
// ═══════════════════════════════════════════════════════════════════════
static bool test_write6_id1_roundtrip_no_crosstalk() {
    reset(0x3);

    // Pre-state of A's block 3, straight from A's memory.
    std::vector<uint8_t> a_before(512);
    for (uint32_t i = 0; i < 512; i++) a_before[i] = probe(0, 3, i);
    for (uint32_t i = 0; i < 512; i++) {
        if (a_before[i] != sig_byte(0xA0, 3, i)) {
            printf("  FAIL A pre-state byte[%u]: got 0x%02x, expected 0x%02x\n",
                   i, a_before[i], sig_byte(0xA0, 3, i));
            return false;
        }
    }

    std::vector<uint8_t> pattern(512);
    for (uint32_t i = 0; i < 512; i++)
        pattern[i] = (uint8_t)(0x37u + i * 7u);
    // The pattern must differ from BOTH signatures somewhere, or the
    // round trip proves nothing.
    bool differs_a = false, differs_b = false;
    for (uint32_t i = 0; i < 512; i++) {
        if (pattern[i] != sig_byte(0xA0, 3, i)) differs_a = true;
        if (pattern[i] != sig_byte(0xB0, 3, i)) differs_b = true;
    }
    CHECK_TRUE("write pattern differs from A's signature", differs_a);
    CHECK_TRUE("write pattern differs from B's signature", differs_b);

    if (!do_write6(1, 3, 1, pattern, "B/WRITE6")) return false;
    CHECK_EQ("B/WRITE6 dev_sel latched to B", dut->dev_sel, 1);

    std::vector<uint8_t> got;
    if (!do_read6(1, 3, 1, got, "B/READBACK")) return false;
    for (uint32_t i = 0; i < 512; i++) {
        if (got[i] != pattern[i]) {
            printf("  FAIL B/READBACK byte[%u]: got 0x%02x, expected 0x%02x "
                   "(t=%lu)\n", i, got[i], pattern[i],
                   (unsigned long)sim_time);
            return false;
        }
    }

    // Cross-talk: A's block 3 must be byte-for-byte what it was.
    for (uint32_t i = 0; i < 512; i++) {
        uint8_t now = probe(0, 3, i);
        if (now != a_before[i]) {
            printf("  FAIL crosstalk A block3 byte[%u]: got 0x%02x, "
                   "expected 0x%02x (write to B leaked into A) (t=%lu)\n",
                   i, now, a_before[i], (unsigned long)sim_time);
            return false;
        }
    }
    // ...and B's block 3 really did change in its own storage.
    for (uint32_t i = 0; i < 512; i++) {
        uint8_t now = probe(1, 3, i);
        if (now != pattern[i]) {
            printf("  FAIL B block3 storage byte[%u]: got 0x%02x, "
                   "expected 0x%02x (t=%lu)\n", i, now, pattern[i],
                   (unsigned long)sim_time);
            return false;
        }
    }
    // A's neighbouring blocks are untouched too.
    for (uint32_t i = 0; i < 512; i += 37) {
        CHECK_EQ("A block2 intact", probe(0, 2, i), sig_byte(0xA0, 2, i));
        CHECK_EQ("A block4 intact", probe(0, 4, i), sig_byte(0xA0, 4, i));
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 4 — multi-block READ(6) of 2 blocks from ID 1: right blocks,
// right order.
// ═══════════════════════════════════════════════════════════════════════
static bool test_read6_id1_two_blocks() {
    reset(0x3);
    std::vector<uint8_t> got;
    if (!do_read6(1, 5, 2, got, "B/READ6x2")) return false;
    CHECK_EQ("B/READ6x2 length", got.size(), 1024u);
    for (uint32_t blk = 0; blk < 2; blk++) {
        for (uint32_t i = 0; i < 512; i++) {
            uint8_t exp = sig_byte(0xB0, 5 + blk, i);
            uint8_t g   = got[blk * 512 + i];
            if (g != exp) {
                printf("  FAIL B/READ6x2 block %u byte[%u]: got 0x%02x, "
                       "expected 0x%02x (other block would be 0x%02x, "
                       "provider A would be 0x%02x) (t=%lu)\n",
                       blk, i, g, exp, sig_byte(0xB0, 5 + (1 - blk), i),
                       sig_byte(0xA0, 5 + blk, i),
                       (unsigned long)sim_time);
                return false;
            }
        }
    }
    // The two blocks are genuinely different data, so "right order" has
    // content: block 0 of the response must NOT equal block 1.
    bool blocks_differ = false;
    for (uint32_t i = 0; i < 512; i++)
        if (got[i] != got[512 + i]) { blocks_differ = true; break; }
    CHECK_TRUE("two returned blocks are distinct", blocks_differ);
    return true;
}


// ═══════════════════════════════════════════════════════════════════════
// Scenario 7 — capacity routing: after a transaction on ID 1, a read to
// ID 0 must be range-checked against PROVIDER A's capacity, not B's.
//
// Why this exists (2026-08-18 hardware investigation): on the FPGA the
// .ASYC00 disk driver takes CHECK CONDITION on READ(10) LBA 2314 x1 while
// vhdd-status reports the SD volume ENABLED with 4194304 blocks, retries
// 16x and returns ioErr(-36) -- with ZERO back-end activity, i.e. the
// command is rejected before any provider request. The only read paths in
// scsi.v that do that are the not-ready arms and `!vh_chk_ok`, and
// vhdd_mux.v:178 routes the capacity the range check sees:
//     assign m_num_lbas = dev_sel ? b_num_lbas : a_num_lbas;
// `vh_dev_sel` is latched at selection and HELD until the next successful
// selection, so a stale value would make a good read to A fail closed.
//
// Provider A is 64 blocks and B is 16 (see tb_scsi_dual.v), so LBA 32 is
// valid for A and out of range for B. Selecting B first, then reading A at
// LBA 32, fails iff the range check consulted B's capacity.
// ═══════════════════════════════════════════════════════════════════════
static bool test_capacity_routing_after_id1_selection() {
    reset(0x3);
    std::vector<uint8_t> got;

    // Touch ID 1 first so dev_sel is latched to B.
    if (!do_read6(1, 0, 1, got, "cap/B-first")) return false;
    CHECK_EQ("cap dev_sel latched to B", dut->dev_sel, 1);

    // Now read ID 0 at an LBA only A is big enough to hold.
    got.clear();
    if (!do_read6(0, 32, 1, got, "cap/A-LBA32")) return false;
    CHECK_EQ("cap A-LBA32 length", got.size(), 512u);
    CHECK_EQ("cap dev_sel back to A", dut->dev_sel, 0);
    for (uint32_t i = 0; i < 512; i++) {
        if (got[i] != sig_byte(0xA0, 32, i)) {
            printf("  FAIL cap/A-LBA32 byte[%u]: got 0x%02x expected 0x%02x (t=%lu)\n",
                   i, got[i], sig_byte(0xA0, 32, i), (unsigned long)sim_time);
            return false;
        }
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 5 — dev_en = 2'b01: ID 1 is off the bus.  Selecting it must
// produce exactly the absent-device outcome (never leaves BUS_FREE: no
// BSY, no REQ, no phase bits, no IRQ), while ID 0 still completes.
// ═══════════════════════════════════════════════════════════════════════
static bool test_dev_en_01_hides_id1() {
    reset(0x1);

    bus_write(0, 0x02);              // Output Data = ID 1 bit
    bus_write(1, IC_SEL | IC_DB);
    for (int i = 0; i < 64; i++) {
        uint8_t s = bus_read(4);
        CHECK_EQ("dev_en=01 no BSY on id1", s & SR_BSY, 0);
        CHECK_EQ("dev_en=01 no REQ on id1", s & SR_REQ, 0);
        CHECK_EQ("dev_en=01 no phase bits on id1",
                 s & (SR_MSG | SR_CD | SR_IO), 0);
        CHECK_EQ("dev_en=01 no IRQ/DRQ on id1",
                 bus_read(5) & (BS_IRQ | BS_DRQ), 0);
        CHECK_EQ("dev_en=01 irq wire low", dut->irq, 0);
        tick_n(1);
    }
    bus_write(1, IC_RST);
    tick_n(4);
    bus_write(1, 0x00);
    tick_n(4);

    // ID 0 still works, and still routes to provider A.
    std::vector<uint8_t> got;
    if (!do_read6(0, 0, 1, got, "dev_en=01 A/READ6")) return false;
    for (uint32_t i = 0; i < 512; i++) {
        if (got[i] != sig_byte(0xA0, 0, i)) {
            printf("  FAIL dev_en=01 A/READ6 byte[%u]: got 0x%02x, "
                   "expected 0x%02x (t=%lu)\n", i, got[i],
                   sig_byte(0xA0, 0, i), (unsigned long)sim_time);
            return false;
        }
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 6 — dev_en = 2'b10: mirror image.  ID 0 is off the bus, ID 1
// answers and routes to provider B.
// ═══════════════════════════════════════════════════════════════════════
static bool test_dev_en_10_hides_id0() {
    reset(0x2);

    bus_write(0, 0x01);              // Output Data = ID 0 bit
    bus_write(1, IC_SEL | IC_DB);
    for (int i = 0; i < 64; i++) {
        uint8_t s = bus_read(4);
        CHECK_EQ("dev_en=10 no BSY on id0", s & SR_BSY, 0);
        CHECK_EQ("dev_en=10 no REQ on id0", s & SR_REQ, 0);
        CHECK_EQ("dev_en=10 no phase bits on id0",
                 s & (SR_MSG | SR_CD | SR_IO), 0);
        CHECK_EQ("dev_en=10 no IRQ/DRQ on id0",
                 bus_read(5) & (BS_IRQ | BS_DRQ), 0);
        CHECK_EQ("dev_en=10 irq wire low", dut->irq, 0);
        tick_n(1);
    }
    bus_write(1, IC_RST);
    tick_n(4);
    bus_write(1, 0x00);
    tick_n(4);

    std::vector<uint8_t> got;
    if (!do_read6(1, 0, 1, got, "dev_en=10 B/READ6")) return false;
    for (uint32_t i = 0; i < 512; i++) {
        if (got[i] != sig_byte(0xB0, 0, i)) {
            printf("  FAIL dev_en=10 B/READ6 byte[%u]: got 0x%02x, "
                   "expected 0x%02x (t=%lu)\n", i, got[i],
                   sig_byte(0xB0, 0, i), (unsigned long)sim_time);
            return false;
        }
    }
    CHECK_EQ("dev_en=10 dev_sel latched to B", dut->dev_sel, 1);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
static void run(const char* name, bool (*fn)()) {
    printf("[RUN ] %s\n", name);
    bool ok = fn();
    if (ok) { printf("[PASS] %s\n", name); n_pass++; }
    else    { printf("[FAIL] %s\n", name); n_fail++; }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scsi_dual;

    run("read6_id0_routes_to_provider_a",   test_read6_id0_routes_to_provider_a);
    run("read6_id1_routes_to_provider_b",   test_read6_id1_routes_to_provider_b);
    run("write6_id1_roundtrip_no_crosstalk", test_write6_id1_roundtrip_no_crosstalk);
    run("read6_id1_two_blocks",             test_read6_id1_two_blocks);
    run("dev_en_01_hides_id1",              test_dev_en_01_hides_id1);
    run("dev_en_10_hides_id0",              test_dev_en_10_hides_id0);
    run("capacity_routing_after_id1_selection",
                                            test_capacity_routing_after_id1_selection);

    printf("%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    dut->final();
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
