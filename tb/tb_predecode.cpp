// tb_predecode.cpp — Verilator unit testbench for rtl/core/decode/predecode.v
//
// Exercises the 68040 instruction boundary scanner (predecoder) in isolation.
// predecode.v is a single-cycle registered combinational module that takes a
// 16-byte fetch buffer and reports the lengths / starts / validity of up to
// two instructions packed into that buffer.  Output is registered — results
// appear one cycle after fetch_valid rises.
//
// Scenarios (names are grep-visible):
//   1. reset_clears_buffer
//   2. simple_fetch_fills_buffer
//   3. variable_length_instruction_boundary_detection
//   4. branch_instruction_flagged
//   5. misaligned_address_handling
//   6. buffer_drain_on_redirect
//   7. back_to_back_branches
//   8. exception_opcode_detection
//   9. moveq_two_per_buffer
//  10. move_l_abs_consumes_two_ext_words
//  14. system_register_forms_consume_extension_words
//
// All scenarios use CHECK_EQ / CHECK_TRUE against the registered outputs
// (pd_inst0_len, pd_inst1_len, pd_inst1_off, pd_valid, pd_stall).  No
// internal-signal peeking.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <verilated.h>
#include "Vpredecode.h"

static Vpredecode* dut      = nullptr;
static uint64_t    sim_time = 0;
static int         n_pass   = 0;
static int         n_fail   = 0;

// ── Clock helpers ──────────────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void zero_inputs() {
    dut->fetch_valid = 0;
    dut->fetch_pc    = 0;
    // 128-bit fetch_buf — Verilator exposes as WData array [0..3] (32-bit words).
    dut->fetch_buf[0] = 0;
    dut->fetch_buf[1] = 0;
    dut->fetch_buf[2] = 0;
    dut->fetch_buf[3] = 0;
}

static void reset() {
    dut->rst = 1;
    zero_inputs();
    tick(); tick();
    dut->rst = 0;
    dut->eval();
}

// Drive a 16-byte fetch_buf from a C byte-array (big-endian wire layout).
// fetch_buf[127:120] = byte 0 (PC+0).  In Verilator's 32-bit-word packing
// for VlWide<4>, bit 127 sits in word[3], MSB.  byte 0 → word[3][31:24].
static void set_buf(const uint8_t bytes[16]) {
    for (int w = 0; w < 4; w++) {
        uint32_t v = 0;
        // word w covers bits [32*w+31 : 32*w].
        // word[3] → bits 127..96 → bytes 0..3.
        // word[2] → bits 95..64  → bytes 4..7.
        // word[1] → bits 63..32  → bytes 8..11.
        // word[0] → bits 31..0   → bytes 12..15.
        int base = (3 - w) * 4;
        v = ((uint32_t)bytes[base + 0] << 24) |
            ((uint32_t)bytes[base + 1] << 16) |
            ((uint32_t)bytes[base + 2] << 8)  |
            ((uint32_t)bytes[base + 3]);
        dut->fetch_buf[w] = v;
    }
}

// Drive fetch_valid for one cycle with a given buffer + PC, tick, then
// sample the registered outputs.
static void drive_fetch(uint32_t pc, const uint8_t bytes[16]) {
    set_buf(bytes);
    dut->fetch_pc    = pc;
    dut->fetch_valid = 1;
    tick();
    dut->fetch_valid = 0;
    dut->eval();
}

// ── Assertion helpers ──────────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    uint32_t g = (uint32_t)(got); \
    uint32_t e = (uint32_t)(exp); \
    if (g != e) { \
        printf("  FAIL %s: got %u (0x%x), expected %u (0x%x)\n", \
               name, g, g, e, e); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { printf("  FAIL %s: condition false\n", name); return false; } \
} while(0)

// ═══════════════════════════════════════════════════════════════════════
// Scenario 1 — reset clears the buffer; valid reports 0.
// ═══════════════════════════════════════════════════════════════════════
static bool test_reset_clears_buffer() {
    reset();
    CHECK_EQ("pd_valid @reset",   dut->pd_valid,    0);
    CHECK_EQ("pd_stall @reset",   dut->pd_stall,    0);
    CHECK_EQ("pd_inst0_len reset", dut->pd_inst0_len, 1);
    CHECK_EQ("pd_inst1_len reset", dut->pd_inst1_len, 0);
    CHECK_EQ("pd_inst1_off reset", dut->pd_inst1_off, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 2 — simple_fetch_fills_buffer
// MOVEQ #0, D0  = 0x7000 (1 word).  Second slot also a MOVEQ.
// ═══════════════════════════════════════════════════════════════════════
static bool test_simple_fetch_fills_buffer() {
    reset();
    uint8_t buf[16] = {
        0x70, 0x00,
        0x72, 0x01,
        0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
    };
    drive_fetch(0x40800000, buf);
    CHECK_EQ("pd_valid[0] after simple fetch", dut->pd_valid & 1, 1);
    CHECK_EQ("pd_inst0_len MOVEQ", dut->pd_inst0_len, 1);
    CHECK_EQ("pd_inst1_off MOVEQ pair", dut->pd_inst1_off, 2);
    CHECK_EQ("pd_inst1_len MOVEQ pair", dut->pd_inst1_len, 1);
    CHECK_EQ("pd_valid[1] MOVEQ pair", (dut->pd_valid >> 1) & 1, 1);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 3 — variable_length_instruction_boundary_detection.
//   inst0 = MOVE.L #imm32, D0 → opword 0x203C + 2 words imm = 3 words.
//   inst1 = NOP                → 1 word.
// ═══════════════════════════════════════════════════════════════════════
static bool test_variable_length_instruction_boundary_detection() {
    reset();
    uint8_t buf[16] = {
        0x20, 0x3C,
        0xDE, 0xAD, 0xBE, 0xEF,
        0x4E, 0x71,
        0, 0, 0, 0,  0, 0, 0, 0,
    };
    drive_fetch(0x40800000, buf);
    CHECK_EQ("MOVE.L #imm length = 3w", dut->pd_inst0_len, 3);
    CHECK_EQ("inst1 off after MOVE.L #imm", dut->pd_inst1_off, 6);
    CHECK_EQ("inst1 NOP len = 1w", dut->pd_inst1_len, 1);
    CHECK_EQ("both valid", dut->pd_valid, 3);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 4 — branch_instruction_flagged.
// BRA.B (1w), Bcc.W (2w), Bcc.L (3w).
// ═══════════════════════════════════════════════════════════════════════
static bool test_branch_instruction_flagged() {
    reset();
    uint8_t b1[16] = { 0x60, 0x04, 0x4E, 0x71,
                        0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0 };
    drive_fetch(0, b1);
    CHECK_EQ("BRA.B len=1", dut->pd_inst0_len, 1);
    CHECK_EQ("BRA.B inst1_off=2", dut->pd_inst1_off, 2);

    uint8_t b2[16] = { 0x67, 0x00, 0x00, 0x08, 0x4E, 0x71,
                        0, 0,  0, 0, 0, 0,  0, 0, 0, 0 };
    drive_fetch(0, b2);
    CHECK_EQ("BEQ.W len=2", dut->pd_inst0_len, 2);
    CHECK_EQ("BEQ.W inst1_off=4", dut->pd_inst1_off, 4);

    uint8_t b3[16] = { 0x67, 0xFF, 0x00, 0x00, 0x00, 0x08, 0x4E, 0x71,
                        0, 0, 0, 0,  0, 0, 0, 0 };
    drive_fetch(0, b3);
    CHECK_EQ("BEQ.L len=3", dut->pd_inst0_len, 3);
    CHECK_EQ("BEQ.L inst1_off=6", dut->pd_inst1_off, 6);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 5 — misaligned_address_handling.
// Odd-byte ifetch is illegal on 68040 but predecode should not fault.
// ═══════════════════════════════════════════════════════════════════════
static bool test_misaligned_address_handling() {
    reset();
    uint8_t buf[16] = {
        0x70, 0x00, 0x70, 0x01,
        0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
    };
    drive_fetch(0x40800001, buf);
    CHECK_EQ("pd_valid[0] on misaligned PC", dut->pd_valid & 1, 1);
    CHECK_EQ("inst0_len misaligned", dut->pd_inst0_len, 1);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 6 — buffer_drain_on_redirect.
// ═══════════════════════════════════════════════════════════════════════
static bool test_buffer_drain_on_redirect() {
    reset();
    uint8_t buf[16] = { 0x70, 0x00,
                        0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0 };
    drive_fetch(0x40800000, buf);
    CHECK_EQ("pd_valid after fill", dut->pd_valid & 1, 1);

    dut->fetch_valid = 0;
    tick();
    dut->eval();
    CHECK_EQ("pd_valid after drain (inst0)", dut->pd_valid & 1, 0);
    CHECK_EQ("pd_valid after drain (inst1)", (dut->pd_valid >> 1) & 1, 0);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 7 — back_to_back_branches.
// ═══════════════════════════════════════════════════════════════════════
static bool test_back_to_back_branches() {
    reset();
    uint8_t buf[16] = {
        0x60, 0x04,
        0x60, 0x02,
        0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
    };
    drive_fetch(0, buf);
    CHECK_EQ("inst0 BRA.B len=1", dut->pd_inst0_len, 1);
    CHECK_EQ("inst1 BRA.B len=1", dut->pd_inst1_len, 1);
    CHECK_EQ("inst1 off=2",       dut->pd_inst1_off, 2);
    CHECK_EQ("both valid",        dut->pd_valid,     3);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 8 — exception_opcode_detection.
// A-line (0xAxxx), ILLEGAL (0x4AFC), F-line (0xFxxx).
// ═══════════════════════════════════════════════════════════════════════
static bool test_exception_opcode_detection() {
    reset();
    uint8_t a[16] = { 0xA0, 0x00, 0x70, 0x00,
                       0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0 };
    drive_fetch(0, a);
    CHECK_EQ("A-line len=1", dut->pd_inst0_len, 1);
    CHECK_EQ("A-line inst1 off=2", dut->pd_inst1_off, 2);

    uint8_t i[16] = { 0x4A, 0xFC, 0x4E, 0x71,
                       0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0 };
    drive_fetch(0, i);
    CHECK_EQ("ILLEGAL len >=1", dut->pd_inst0_len >= 1, 1);
    CHECK_TRUE("ILLEGAL does not fault", dut->pd_valid & 1);

    uint8_t f[16] = { 0xF0, 0x00, 0x70, 0x00,
                       0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0 };
    drive_fetch(0, f);
    CHECK_TRUE("F-line does not fault", dut->pd_valid & 1);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 9 — moveq_two_per_buffer.
// ═══════════════════════════════════════════════════════════════════════
static bool test_moveq_two_per_buffer() {
    reset();
    uint8_t buf[16] = {
        0x70, 0x01, 0x72, 0x02, 0x74, 0x03, 0x76, 0x04,
        0x78, 0x05, 0x7A, 0x06, 0x7C, 0x07, 0x7E, 0x08,
    };
    drive_fetch(0, buf);
    CHECK_EQ("MOVEQ0 len=1", dut->pd_inst0_len, 1);
    CHECK_EQ("MOVEQ1 len=1", dut->pd_inst1_len, 1);
    CHECK_EQ("MOVEQ1 off=2", dut->pd_inst1_off, 2);
    CHECK_EQ("both valid", dut->pd_valid, 3);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 10 — move_l_abs_consumes_two_ext_words.
// MOVE.L D0, (xxx).L = 0x23C0 + 4 bytes abs address = 3 words.
// ═══════════════════════════════════════════════════════════════════════
static bool test_move_l_abs_consumes_two_ext_words() {
    reset();
    uint8_t buf[16] = {
        0x23, 0xC0,
        0x12, 0x34, 0x56, 0x78,
        0x4E, 0x71,
        0, 0, 0, 0,  0, 0, 0, 0,
    };
    drive_fetch(0, buf);
    CHECK_EQ("MOVE.L D0,(abs.L) len=3", dut->pd_inst0_len, 3);
    CHECK_EQ("inst1 off=6", dut->pd_inst1_off, 6);
    CHECK_EQ("inst1 NOP len=1", dut->pd_inst1_len, 1);
    CHECK_EQ("both valid", dut->pd_valid, 3);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 11 — full_format_mode6_counts_payload_words.
// ROM frontier: MOVE.L @($0ddc)@(-24),$1ef0.W is 5 words:
// op + full ext + bd.W + od.W + abs.W destination.
// ═══════════════════════════════════════════════════════════════════════
static bool test_full_format_mode6_counts_payload_words() {
    reset();
    uint8_t buf[16] = {
        0x21, 0xF0,
        0x81, 0xE2,
        0x0D, 0xDC,
        0xFF, 0xE8,
        0x1E, 0xF0,
        0x4E, 0x71,
        0, 0, 0, 0,
    };
    drive_fetch(0, buf);
    CHECK_EQ("MOVE.L full memind abs.W len=5", dut->pd_inst0_len, 5);
    CHECK_EQ("inst1 off after full memind", dut->pd_inst1_off, 10);
    CHECK_EQ("inst1 NOP len=1", dut->pd_inst1_len, 1);
    CHECK_EQ("both valid", dut->pd_valid, 3);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 12 — static_bit_full_format_counts_post_immediate_ea.
// ROM frontier: BTST #0,@($0ddc)@(-25) is 5 words:
// op + imm.W + full ext + bd.W + od.W.  The full extension word lives
// after the immediate word, not in extword0.
// ═══════════════════════════════════════════════════════════════════════
static bool test_static_bit_full_format_counts_post_immediate_ea() {
    reset();
    uint8_t buf[16] = {
        0x08, 0x30,
        0x00, 0x00,
        0x81, 0xE2,
        0x0D, 0xDC,
        0xFF, 0xE7,
        0x4E, 0x71,
        0, 0, 0, 0,
    };
    drive_fetch(0, buf);
    CHECK_EQ("BTST full memind len=5", dut->pd_inst0_len, 5);
    CHECK_EQ("inst1 off after BTST full memind", dut->pd_inst1_off, 10);
    CHECK_EQ("inst1 NOP len=1", dut->pd_inst1_len, 1);
    CHECK_EQ("both valid", dut->pd_valid, 3);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 13 — jsr_full_format_mode6_counts_payload_words.
// ROM frontier: JSR @($400,D2.W*4)@(0) is 3 words:
// op + full ext + bd.W.
// ═══════════════════════════════════════════════════════════════════════
static bool test_jsr_full_format_mode6_counts_payload_words() {
    reset();
    uint8_t buf[16] = {
        0x4E, 0xB0,
        0x25, 0xA1,
        0x04, 0x00,
        0x4E, 0x71,
        0, 0, 0, 0,  0, 0, 0, 0,
    };
    drive_fetch(0, buf);
    CHECK_EQ("JSR full memind len=3", dut->pd_inst0_len, 3);
    CHECK_EQ("inst1 off after JSR full memind", dut->pd_inst1_off, 6);
    CHECK_EQ("inst1 NOP len=1", dut->pd_inst1_len, 1);
    CHECK_EQ("both valid", dut->pd_valid, 3);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Scenario 14 — system_register_forms_consume_extension_words.
// ROM frontier: MOVE.W #$2700,SR is opword + imm16.  If predecode
// reports it as one word, $2700 is decoded as the next opcode.
// ═══════════════════════════════════════════════════════════════════════
static bool test_system_register_forms_consume_extension_words() {
    reset();
    uint8_t move_sr[16] = {
        0x46, 0xFC,
        0x27, 0x00,
        0x4E, 0x71,
        0, 0, 0, 0,  0, 0, 0, 0,  0, 0,
    };
    drive_fetch(0, move_sr);
    CHECK_EQ("MOVE.W #imm,SR len=2", dut->pd_inst0_len, 2);
    CHECK_EQ("NOP off after MOVE.W #imm,SR", dut->pd_inst1_off, 4);
    CHECK_EQ("NOP len after MOVE.W #imm,SR", dut->pd_inst1_len, 1);

    uint8_t move_ccr[16] = {
        0x44, 0xFC,
        0x00, 0x04,
        0x4E, 0x71,
        0, 0, 0, 0,  0, 0, 0, 0,  0, 0,
    };
    drive_fetch(0, move_ccr);
    CHECK_EQ("MOVE.W #imm,CCR len=2", dut->pd_inst0_len, 2);
    CHECK_EQ("NOP off after MOVE.W #imm,CCR", dut->pd_inst1_off, 4);
    CHECK_EQ("NOP len after MOVE.W #imm,CCR", dut->pd_inst1_len, 1);

    uint8_t stop_movec[16] = {
        0x4E, 0x72,
        0x27, 0x00,
        0x4E, 0x7A,
        0x08, 0x01,
        0x4E, 0x71,
        0, 0, 0, 0,  0, 0,
    };
    drive_fetch(0, stop_movec);
    CHECK_EQ("STOP #imm len=2", dut->pd_inst0_len, 2);
    CHECK_EQ("MOVEC off after STOP", dut->pd_inst1_off, 4);
    CHECK_EQ("MOVEC len=2", dut->pd_inst1_len, 2);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════
#define RUN(fn) do { \
    zero_inputs(); \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vpredecode;
    zero_inputs();

    RUN(test_reset_clears_buffer);
    RUN(test_simple_fetch_fills_buffer);
    RUN(test_variable_length_instruction_boundary_detection);
    RUN(test_branch_instruction_flagged);
    RUN(test_misaligned_address_handling);
    RUN(test_buffer_drain_on_redirect);
    RUN(test_back_to_back_branches);
    RUN(test_exception_opcode_detection);
    RUN(test_moveq_two_per_buffer);
    RUN(test_move_l_abs_consumes_two_ext_words);
    RUN(test_full_format_mode6_counts_payload_words);
    RUN(test_static_bit_full_format_counts_post_immediate_ea);
    RUN(test_jsr_full_format_mode6_counts_payload_words);
    RUN(test_system_register_forms_consume_extension_words);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
