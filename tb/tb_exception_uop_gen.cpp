// tb_exception_uop_gen.cpp — Verilator unit testbench for the
// pure-combinational helper functions in `rtl/core/exception_uop_gen.vh`.
//
// Validates that for each (vec, fmt, push_word_idx) the generated µop
// bundle matches the expected layout for a 68040 exception entry as
// defined by exception.v's existing `frame_word_data` /
// `frame_word_addr` / `format_vec_word` machinery.  This is the
// single source of truth that the upcoming µop injection FSM will
// rely on; passing tests here means the cutover to the µop path
// preserves frame layout bit-for-bit.
//
// The tb is purely combinational — no clock, no reset.  Each test
// case sets inputs, evaluates, and checks outputs.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "Vexception_uop_gen_wrap.h"
#include "verilated.h"

static int n_pass = 0;
static int n_fail = 0;

#define CHECK(name, cond) do {                                  \
    if (cond) { ++n_pass; std::printf("[PASS] %s\n", name); }  \
    else      { ++n_fail; std::printf("[FAIL] %s\n", name); }  \
} while (0)

#define CHECK_EQ(name, got, want) do {                          \
    uint64_t g = (uint64_t)(got), w = (uint64_t)(want);          \
    if (g == w) { ++n_pass; std::printf("[PASS] %s (0x%llx)\n", \
                                        name, (unsigned long long)g); } \
    else { ++n_fail; std::printf("[FAIL] %s: got 0x%llx, want 0x%llx\n", \
                                 name, (unsigned long long)g,    \
                                 (unsigned long long)w); }       \
} while (0)

// uop_pkg constants we cross-check against
static constexpr uint32_t UOP_LOAD  = 2;
static constexpr uint32_t UOP_STORE = 3;
static constexpr uint32_t SZ_WORD   = 1;
static constexpr uint32_t SZ_LONG   = 2;

static void eval(Vexception_uop_gen_wrap* dut) { dut->eval(); }

// Drive a complete state, evaluate, return.
static void set_state(Vexception_uop_gen_wrap* dut,
                      uint8_t  push_idx,
                      bool     is_fmt7,
                      bool     is_fmt2,
                      bool     is_m_irq,
                      uint16_t sr,
                      uint32_t fault_pc,
                      uint32_t fault_addr,
                      uint16_t format_vec_word,
                      uint16_t fmt7_ssw,
                      uint32_t a7_new,
                      uint32_t vbr,
                      uint8_t  vec,
                      uint32_t isp_new = 0,
                      uint32_t msp_new = 0) {
    dut->push_word_idx   = push_idx;
    dut->is_fmt7         = is_fmt7;
    dut->is_fmt2         = is_fmt2;
    dut->cur_is_m_irq    = is_m_irq;
    dut->cur_sr          = sr;
    dut->cur_fault_pc    = fault_pc;
    dut->cur_fault_addr  = fault_addr;
    dut->format_vec_word = format_vec_word;
    dut->fmt7_ssw        = fmt7_ssw;
    dut->cur_a7_new      = a7_new;
    dut->cur_vbr         = vbr;
    dut->cur_vec         = vec;
    dut->cur_isp_new     = isp_new;
    dut->cur_msp_new     = msp_new;
    eval(dut);
}

// Helper: lower 16 bits of o_uop_data
static uint16_t data_word(Vexception_uop_gen_wrap* dut) {
    return (uint16_t)(dut->o_uop_data & 0xFFFF);
}

// ─── Scenario 1: format-0 frame (vec 25 = level-1 IRQ) ──────────────
// Frame layout per m68k 040 (fmt 0):
//   SP+0 : SR
//   SP+2 : PC[31:16]
//   SP+4 : PC[15:0]
//   SP+6 : format/vec word (0x064 = vec 25*4)
// Plus the terminating LOAD µop reading from VBR + 25*4.
static void scenario_fmt0_irq25(Vexception_uop_gen_wrap* dut) {
    std::printf("\n=== Scenario 1: format-0, vec 25 IRQ entry ===\n");

    const uint32_t a7_new   = 0x0000FECE;
    const uint32_t fault_pc = 0x40847BF6;
    const uint32_t vbr      = 0x0000FECE;
    const uint8_t  vec      = 25;
    const uint16_t sr       = 0x2000;
    const uint16_t format_vec_word = (0 << 12) | (vec * 4);  // = 0x0064

    // push_count = 4 (format-0)
    set_state(dut, 0, false, false, false, sr, fault_pc, 0, format_vec_word,
              0, a7_new, vbr, vec);
    CHECK_EQ("fmt0 push_count", dut->o_push_count, 4);

    // idx 0: STORE.W [a7_new + 0] = sr
    CHECK_EQ("idx0 type==STORE",     dut->o_uop_type, UOP_STORE);
    CHECK_EQ("idx0 size==WORD",      dut->o_uop_size, SZ_WORD);
    CHECK_EQ("idx0 addr",            dut->o_uop_addr, a7_new + 0);
    CHECK_EQ("idx0 data == sr",      data_word(dut),  sr);
    CHECK_EQ("idx0 imm_is_data",     dut->o_imm_is_data, 1);
    CHECK_EQ("idx0 fc_override==5",  dut->o_fc_override, 5);
    CHECK_EQ("idx0 is_store",        dut->o_is_store, 1);
    CHECK_EQ("idx0 is_load",         dut->o_is_load,  0);
    CHECK_EQ("idx0 is_finalize",     dut->o_is_finalize, 0);

    // idx 1: STORE.W [a7_new + 2] = fault_pc[31:16]
    set_state(dut, 1, false, false, false, sr, fault_pc, 0, format_vec_word,
              0, a7_new, vbr, vec);
    CHECK_EQ("idx1 addr",            dut->o_uop_addr, a7_new + 2);
    CHECK_EQ("idx1 data == pc_hi",   data_word(dut),  (fault_pc >> 16) & 0xFFFF);

    // idx 2: STORE.W [a7_new + 4] = fault_pc[15:0]
    set_state(dut, 2, false, false, false, sr, fault_pc, 0, format_vec_word,
              0, a7_new, vbr, vec);
    CHECK_EQ("idx2 addr",            dut->o_uop_addr, a7_new + 4);
    CHECK_EQ("idx2 data == pc_lo",   data_word(dut),  fault_pc & 0xFFFF);

    // idx 3: STORE.W [a7_new + 6] = format_vec_word
    set_state(dut, 3, false, false, false, sr, fault_pc, 0, format_vec_word,
              0, a7_new, vbr, vec);
    CHECK_EQ("idx3 addr",            dut->o_uop_addr, a7_new + 6);
    CHECK_EQ("idx3 data == fmt/vec", data_word(dut),  format_vec_word);

    // idx 4: LOAD.L [vbr + 25*4]  ← terminating finalize µop
    set_state(dut, 4, false, false, false, sr, fault_pc, 0, format_vec_word,
              0, a7_new, vbr, vec);
    CHECK_EQ("idx4 type==LOAD",      dut->o_uop_type, UOP_LOAD);
    CHECK_EQ("idx4 size==LONG",      dut->o_uop_size, SZ_LONG);
    CHECK_EQ("idx4 addr == vbr+vec*4",
             dut->o_uop_addr, vbr + (uint32_t)vec * 4);
    CHECK_EQ("idx4 is_load",         dut->o_is_load,  1);
    CHECK_EQ("idx4 is_store",        dut->o_is_store, 0);
    CHECK_EQ("idx4 is_finalize",     dut->o_is_finalize, 1);
}

// ─── Scenario 2: format-2 frame (vec 6 = CHK) ────────────────────────
// 6 µops: idx 0..3 like fmt0; idx 4..5 carry fault_pc[31:16] and
// fault_pc[15:0] (NOT fault_addr — fmt-2 uses pc).
static void scenario_fmt2_chk(Vexception_uop_gen_wrap* dut) {
    std::printf("\n=== Scenario 2: format-2, vec 6 CHK ===\n");

    const uint32_t a7_new   = 0x0010FFE8;
    const uint32_t fault_pc = 0x40802000;
    const uint32_t fault_addr = 0xDEADBEEF;     // ignored by fmt-2 idx4/5
    const uint8_t  vec      = 6;
    const uint16_t format_vec_word = (2 << 12) | (vec * 4);  // = 0x2018
    const uint16_t sr       = 0x2700;

    set_state(dut, 0, false, true, false, sr, fault_pc, fault_addr,
              format_vec_word, 0, a7_new, 0, vec);
    CHECK_EQ("fmt2 push_count", dut->o_push_count, 6);

    // idx 4: data = fault_pc[31:16]  (NOT fault_addr)
    set_state(dut, 4, false, true, false, sr, fault_pc, fault_addr,
              format_vec_word, 0, a7_new, 0, vec);
    CHECK_EQ("fmt2 idx4 data == pc_hi",
             data_word(dut), (fault_pc >> 16) & 0xFFFF);
    CHECK_EQ("fmt2 idx4 addr == a7_new+8", dut->o_uop_addr, a7_new + 8);

    // idx 5: data = fault_pc[15:0]
    set_state(dut, 5, false, true, false, sr, fault_pc, fault_addr,
              format_vec_word, 0, a7_new, 0, vec);
    CHECK_EQ("fmt2 idx5 data == pc_lo",
             data_word(dut), fault_pc & 0xFFFF);

    // idx 6: LOAD finalize
    set_state(dut, 6, false, true, false, sr, fault_pc, fault_addr,
              format_vec_word, 0, a7_new, 0, vec);
    CHECK_EQ("fmt2 idx6 is_finalize", dut->o_is_finalize, 1);
    CHECK_EQ("fmt2 idx6 type == LOAD", dut->o_uop_type, UOP_LOAD);
}

// ─── Scenario 3: format-7 frame (vec 2 = bus error) ──────────────────
// 30 µops; idx 4..5 carry fault_addr[31:16] / fault_addr[15:0]
// (NOT fault_pc — fmt-7 swaps which field at idx4/5).  idx 6 is the
// SSW.
static void scenario_fmt7_buserr(Vexception_uop_gen_wrap* dut) {
    std::printf("\n=== Scenario 3: format-7, vec 2 bus error ===\n");

    const uint32_t a7_new     = 0x0010FF80;
    const uint32_t fault_pc   = 0x40803000;
    const uint32_t fault_addr = 0xCAFEBABE;
    const uint8_t  vec        = 2;
    const uint16_t format_vec_word = (7 << 12) | (vec * 4);  // = 0x7008
    const uint16_t fmt7_ssw    = 0x0500;        // probe + DATA FC=5

    set_state(dut, 0, true, false, false, 0, fault_pc, fault_addr,
              format_vec_word, fmt7_ssw, a7_new, 0, vec);
    CHECK_EQ("fmt7 push_count", dut->o_push_count, 30);

    // idx 4: data = fault_addr[31:16]
    set_state(dut, 4, true, false, false, 0, fault_pc, fault_addr,
              format_vec_word, fmt7_ssw, a7_new, 0, vec);
    CHECK_EQ("fmt7 idx4 data == addr_hi",
             data_word(dut), (fault_addr >> 16) & 0xFFFF);

    // idx 5: data = fault_addr[15:0]
    set_state(dut, 5, true, false, false, 0, fault_pc, fault_addr,
              format_vec_word, fmt7_ssw, a7_new, 0, vec);
    CHECK_EQ("fmt7 idx5 data == addr_lo",
             data_word(dut), fault_addr & 0xFFFF);

    // idx 6: data = fmt7_ssw
    set_state(dut, 6, true, false, false, 0, fault_pc, fault_addr,
              format_vec_word, fmt7_ssw, a7_new, 0, vec);
    CHECK_EQ("fmt7 idx6 data == ssw", data_word(dut), fmt7_ssw);

    // idx 30: finalize
    set_state(dut, 30, true, false, false, 0, fault_pc, fault_addr,
              format_vec_word, fmt7_ssw, a7_new, 0, vec);
    CHECK_EQ("fmt7 idx30 is_finalize", dut->o_is_finalize, 1);
}

// ─── Scenario 4: vector address arithmetic across vec range ──────────
// For each vec in {0, 1, 4, 25, 31, 64, 255}, the LOAD addr should
// be vbr + vec*4, regardless of fmt.
static void scenario_vec_addr_sweep(Vexception_uop_gen_wrap* dut) {
    std::printf("\n=== Scenario 4: vector address sweep ===\n");

    const uint32_t vbr = 0x0000FECE;
    const uint8_t  vecs[] = {0, 1, 4, 25, 31, 64, 255};
    char buf[64];

    for (uint8_t v : vecs) {
        // LOAD µop is at idx == push_count (= 4 for fmt-0)
        set_state(dut, 4, false, false, false, 0, 0, 0, 0, 0, 0, vbr, v);
        std::snprintf(buf, sizeof buf, "vec %u dispatch addr", v);
        CHECK_EQ(buf, dut->o_uop_addr, vbr + (uint32_t)v * 4);
    }
}

// ─── Scenario 5: a7_new arithmetic at offset 2 (unaligned SSP) ───────
// The Q700 ROM puts the VBR table on SSP at 0xFECE — unaligned by 2.
// Frame pushes go to a7_new+0/+2/+4/+6.  Our helper must produce
// these literal byte addresses (not round to alignment — LSU's own
// split logic handles 16-bit-store-at-unaligned).
static void scenario_unaligned_a7(Vexception_uop_gen_wrap* dut) {
    std::printf("\n=== Scenario 5: unaligned a7_new (SSP=0xFECE) ===\n");

    const uint32_t a7_new = 0x0000FECE;
    for (uint8_t i = 0; i < 4; ++i) {
        set_state(dut, i, false, false, false, 0, 0, 0, 0, 0, a7_new, 0, 0);
        char buf[64];
        std::snprintf(buf, sizeof buf, "fmt0 idx%u addr == a7+%u", i, i*2);
        CHECK_EQ(buf, dut->o_uop_addr, a7_new + (uint32_t)i * 2);
    }
}

// ─── Scenario 6: 68040 M=1 IRQ dual-frame stack split ────────────────
// PRM §8.5.1: Format $1 throwaway goes to MSP, then M clears and
// Format $0 main goes to ISP.  Codebase signal names here carry the
// already-subtracted per-stack frame bases.
static void scenario_m_irq_dual_frame_swap(Vexception_uop_gen_wrap* dut) {
    std::printf("\n=== Scenario 6: M=1 IRQ dual-frame stack split ===\n");

    const uint32_t fire_a7_before  = 0x0000FE00;  // ISP per PRM
    const uint32_t fire_msp_before = 0x0000FD00;  // MSP per PRM
    const uint32_t isp_new = fire_a7_before - 8;  // main frame base
    const uint32_t msp_new = fire_msp_before - 8; // throwaway frame base
    const uint8_t  vec = 25;
    const uint16_t sr = 0x3000;
    const uint32_t fault_pc = 0x40847BF6;
    const uint16_t format_vec_word = (0 << 12) | (vec * 4);

    set_state(dut, 0, false, false, true, sr, fault_pc, 0,
              format_vec_word, 0, isp_new, 0, vec, isp_new, msp_new);
    CHECK_EQ("m-irq push_count", dut->o_push_count, 8);

    for (uint8_t i = 0; i < 4; ++i) {
        set_state(dut, i, false, false, true, sr, fault_pc, 0,
                  format_vec_word, 0, isp_new, 0, vec, isp_new, msp_new);
        char buf[80];
        std::snprintf(buf, sizeof buf, "m-irq throwaway idx%u MSP addr", i);
        CHECK_EQ(buf, dut->o_uop_addr, msp_new + (uint32_t)i * 2);
    }

    for (uint8_t i = 4; i < 8; ++i) {
        set_state(dut, i, false, false, true, sr, fault_pc, 0,
                  format_vec_word, 0, isp_new, 0, vec, isp_new, msp_new);
        char buf[80];
        std::snprintf(buf, sizeof buf, "m-irq main idx%u ISP addr", i);
        CHECK_EQ(buf, dut->o_uop_addr, isp_new + (uint32_t)(i - 4) * 2);
    }

    set_state(dut, 3, false, false, true, sr, fault_pc, 0,
              format_vec_word, 0, isp_new, 0, vec, isp_new, msp_new);
    CHECK_EQ("m-irq throwaway format nibble", data_word(dut) >> 12, 1);

    set_state(dut, 7, false, false, true, sr, fault_pc, 0,
              format_vec_word, 0, isp_new, 0, vec, isp_new, msp_new);
    CHECK_EQ("m-irq main format nibble", data_word(dut) >> 12, 0);
    CHECK_EQ("m-irq inject_a7_new", dut->o_inject_a7_new, 0x0000FDF8);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto* dut = new Vexception_uop_gen_wrap;

    scenario_fmt0_irq25(dut);
    scenario_fmt2_chk(dut);
    scenario_fmt7_buserr(dut);
    scenario_vec_addr_sweep(dut);
    scenario_unaligned_a7(dut);
    scenario_m_irq_dual_frame_swap(dut);

    delete dut;

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    return n_fail ? 1 : 0;
}
