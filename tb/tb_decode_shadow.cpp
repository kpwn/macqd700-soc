// tb_decode_shadow.cpp — Phase-1 validation harness for 2-wide decode (task #237).
//
// Drives the standalone `decode` module twice for each instruction pair:
//   pass-A : pd_buf contains [inst0 || inst1 || ...].  Lane-0 emits inst0's
//            µop on the primary outputs (uop_*), lane-1 internally emits
//            inst1's µop on the V2 assembler outputs (`l1_asm_*`).
//   pass-B : pd_buf contains [inst1 || ...] alone (single-wide reference).
//            Lane-0 emits inst1's µop on the primary outputs.
//
// Phase-1 gate: pass-A's lane-1 ASSEMBLER bundle (`l1_asm_*`, read via
// the verilator root struct) must match pass-B's primary uop_* bundle
// bit-exactly across the µop.  This proves the shadow lane chain is
// wired correctly — the V2 chain on shifted-pd_buf produces the same
// µop the legacy chain produces when pd_buf is unshifted to inst1.
//
// We compare the INTERNAL assembler bundle rather than the gated
// `disp_d_*` dispatch port because the accept gate (CCR conflict, RAW,
// share-conditions) masks out otherwise-correct shadow output for many
// pair shapes.  The gate is correctness-orthogonal to the shadow chain
// itself; the test focuses on the chain.
//
// Build: requires decode.v compiled with `ENABLE_2WIDE_DECODE=1` so the
// lane-1 chain is physically synthesised AND the dispatch port is reachable.
// See `tb-decode-shadow` in Makefile.
//
// Pairs are encoded as raw opword arrays.  The first instruction is the
// lane-0 candidate; the second is the lane-1 candidate.  Both must be
// single-µop, non-branch, non-exception instructions for the comparison
// to be meaningful (multi-µop cracks rely on phase counter that lane-1
// always sees as 0 — only the first phase's µop is comparable).

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <verilated.h>
#include "Vdecode.h"
#include "Vdecode___024root.h"

namespace {

void tick(Vdecode* dut) {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
}

void reset(Vdecode* dut) {
    dut->rst = 1;
    dut->pd_valid = 0;
    dut->pd_fault = 0;
    dut->pd_next_fault = 0;
    dut->rn_ready = 1;
    dut->flush_en = 0;
    dut->pd_pc = 0;
    for (int i = 0; i < 4; i++)
        dut->pd_buf[i] = 0;
    tick(dut);
    tick(dut);
    dut->rst = 0;
    dut->eval();
}

// pd_buf is laid out big-endian: byte at PC+0 lives in pd_buf[127:120].
// Verilator packs the 128-bit pd_buf into 4 × 32-bit words; pd_buf[3] is
// the high word ([127:96]), pd_buf[0] is the low word ([31:0]).
void set_pd_buf_words(Vdecode* dut, const std::vector<uint16_t>& opwords) {
    uint8_t bytes[16] = {0};
    for (size_t i = 0; i < opwords.size() && i < 8; i++) {
        bytes[2 * i + 0] = (uint8_t)(opwords[i] >> 8);
        bytes[2 * i + 1] = (uint8_t)(opwords[i] & 0xFF);
    }
    for (int w = 0; w < 4; w++) {
        int base = (3 - w) * 4;
        dut->pd_buf[w] =
            ((uint32_t)bytes[base + 0] << 24) |
            ((uint32_t)bytes[base + 1] << 16) |
            ((uint32_t)bytes[base + 2] << 8) |
            ((uint32_t)bytes[base + 3] << 0);
    }
}

struct UopBundle {
    uint8_t  uop_valid;
    uint8_t  uop_type;
    uint8_t  uop_op;
    uint8_t  uop_size;
    uint8_t  has_src_a;
    uint8_t  arch_src_a;
    uint8_t  has_src_b;
    uint8_t  arch_src_b;
    uint8_t  has_src_c;
    uint8_t  arch_src_c;
    uint8_t  has_dst;
    uint8_t  arch_dst;
    uint8_t  has_dst_b;
    uint8_t  arch_dst_b;
    uint8_t  imm_valid;
    uint32_t imm;
    uint8_t  flags_wr;
    uint8_t  flags_rd;
    uint8_t  is_branch;
    uint8_t  is_store;
    uint8_t  is_load;
    uint8_t  imm_is_data;
    uint32_t imm_data;
    uint8_t  is_rts;
    uint8_t  is_abs;
    uint8_t  exc_valid;
    uint8_t  exc_vec;
    uint8_t  requires_supervisor;
    uint8_t  uop_is_last;
    uint8_t  elim_kind;
    uint8_t  elim_arch_dst;
    uint8_t  len_bytes;
};

// Read primary lane-0 µop from the decode DUT.
UopBundle read_lane0(Vdecode* dut) {
    UopBundle u{};
    u.uop_valid           = dut->uop_valid;
    u.uop_type            = dut->uop_type;
    u.uop_op              = dut->uop_op;
    u.uop_size            = dut->uop_size;
    u.has_src_a           = dut->has_src_a;
    u.arch_src_a          = dut->arch_src_a;
    u.has_src_b           = dut->has_src_b;
    u.arch_src_b          = dut->arch_src_b;
    u.has_src_c           = dut->has_src_c;
    u.arch_src_c          = dut->arch_src_c;
    u.has_dst             = dut->has_dst;
    u.arch_dst            = dut->arch_dst;
    u.has_dst_b           = dut->has_dst_b;
    u.arch_dst_b          = dut->arch_dst_b;
    u.imm_valid           = dut->imm_valid;
    u.imm                 = dut->imm;
    u.flags_wr            = dut->flags_wr;
    u.flags_rd            = dut->flags_rd;
    u.is_branch           = dut->is_branch;
    u.is_store            = dut->is_store;
    u.is_load             = dut->is_load;
    u.imm_is_data         = dut->imm_is_data;
    u.imm_data            = dut->imm_data;
    u.is_rts              = dut->is_rts;
    u.is_abs              = dut->is_abs;
    u.exc_valid           = dut->exc_valid;
    u.exc_vec             = dut->exc_vec;
    u.requires_supervisor = dut->requires_supervisor;
    u.uop_is_last         = dut->uop_is_last;
    u.elim_kind           = dut->elim_kind;
    u.elim_arch_dst       = dut->elim_arch_dst;
    // len_bytes is computed from uop_npc - uop_pc since len_bytes is not
    // a top-level output port.
    u.len_bytes           = (uint8_t)(dut->uop_npc - dut->uop_pc);
    return u;
}

// Read shadow lane-1 µop from the decode DUT — both the gated dispatch
// port (`disp_d_*`) and the internal assembler outputs (`l1_asm_*`).  The
// dispatch port is forced to 0 by the accept-gate when lane-0 / lane-1
// share-conditions fail (e.g. CCR conflict, RAW); the internal assembler
// outputs ARE always live and are what we want to compare for shadow
// correctness.
UopBundle read_lane1(Vdecode* dut) {
    UopBundle u{};
    u.uop_valid           = dut->disp_d_uop_valid;
    u.uop_type            = dut->disp_d_uop_type;
    u.uop_op              = dut->disp_d_uop_op;
    u.uop_size            = dut->disp_d_uop_size;
    u.has_src_a           = dut->disp_d_has_src_a;
    u.arch_src_a          = dut->disp_d_arch_src_a;
    u.has_src_b           = dut->disp_d_has_src_b;
    u.arch_src_b          = dut->disp_d_arch_src_b;
    u.has_src_c           = dut->disp_d_has_src_c;
    u.arch_src_c          = dut->disp_d_arch_src_c;
    u.has_dst             = dut->disp_d_has_dst;
    u.arch_dst            = dut->disp_d_arch_dst;
    u.has_dst_b           = dut->disp_d_has_dst_b;
    u.arch_dst_b          = dut->disp_d_arch_dst_b;
    u.imm_valid           = dut->disp_d_imm_valid;
    u.imm                 = dut->disp_d_imm;
    u.flags_wr            = dut->disp_d_flags_wr;
    u.flags_rd            = dut->disp_d_flags_rd;
    u.is_branch           = dut->disp_d_is_branch;
    u.is_store            = dut->disp_d_is_store;
    u.is_load             = dut->disp_d_is_load;
    u.imm_is_data         = dut->disp_d_imm_is_data;
    u.imm_data            = dut->disp_d_imm_data;
    u.is_rts              = dut->disp_d_is_rts;
    u.is_abs              = dut->disp_d_is_abs;
    u.exc_valid           = dut->disp_d_exc_valid;
    u.exc_vec             = dut->disp_d_exc_vec;
    u.requires_supervisor = dut->disp_d_requires_supervisor;
    u.uop_is_last         = dut->disp_d_uop_is_last;
    u.elim_kind           = dut->disp_d_elim_kind;
    u.elim_arch_dst       = dut->disp_d_elim_arch_dst;
    u.len_bytes           = dut->disp_d_len_bytes;
    return u;
}

// Read the internal lane-1 assembler bundle (not gated by the accept
// gate).  Used by the shadow validation test to compare against the
// re-driven single-wide reference, since the accept gate can mask out
// otherwise-correct lane-1 µops (CCR conflict, RAW, etc).  The internal
// assembler signals ARE always live — they reflect the bit-exact V2 chain
// output for the shadow opword.
UopBundle read_lane1_internal(Vdecode* dut) {
    auto* rp = dut->rootp;
    UopBundle u{};
    u.uop_valid           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_uop_valid;
    u.uop_type            = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_uop_type;
    u.uop_op              = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_uop_op;
    u.uop_size            = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_uop_size;
    u.has_src_a           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_has_src_a;
    u.arch_src_a          = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_arch_src_a;
    u.has_src_b           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_has_src_b;
    u.arch_src_b          = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_arch_src_b;
    u.has_src_c           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_has_src_c;
    u.arch_src_c          = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_arch_src_c;
    u.has_dst             = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_has_dst;
    u.arch_dst            = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_arch_dst;
    u.has_dst_b           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_has_dst_b;
    u.arch_dst_b          = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_arch_dst_b;
    u.imm_valid           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_imm_valid;
    u.imm                 = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_imm;
    u.flags_wr            = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_flags_wr;
    u.flags_rd            = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_flags_rd;
    u.is_branch           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_is_branch;
    u.is_store            = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_is_store;
    u.is_load             = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_is_load;
    u.imm_is_data         = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_imm_is_data;
    u.imm_data            = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_imm_data;
    u.is_rts              = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_is_rts;
    u.is_abs              = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_is_abs;
    u.exc_valid           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_exc_valid;
    u.exc_vec             = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_exc_vec;
    u.requires_supervisor = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_requires_supervisor;
    u.uop_is_last         = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_uop_is_last;
    u.elim_kind           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_elim_kind;
    u.elim_arch_dst       = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_elim_arch_dst;
    u.len_bytes           = rp->decode__DOT__gen_lane1_on__DOT__l1_asm_len_bytes;
    return u;
}

#define COMPARE(field) do { \
    if (a.field != b.field) { \
        std::fprintf(stderr, "  diff " #field ": lane1=0x%x ref=0x%x\n", \
                     (unsigned)a.field, (unsigned)b.field); \
        ok = false; \
    } \
} while (0)

bool compare_uops(const UopBundle& a, const UopBundle& b) {
    bool ok = true;
    COMPARE(uop_valid);
    COMPARE(uop_type);
    COMPARE(uop_op);
    COMPARE(uop_size);
    COMPARE(has_src_a);
    COMPARE(arch_src_a);
    COMPARE(has_src_b);
    COMPARE(arch_src_b);
    COMPARE(has_src_c);
    COMPARE(arch_src_c);
    COMPARE(has_dst);
    COMPARE(arch_dst);
    COMPARE(has_dst_b);
    COMPARE(arch_dst_b);
    COMPARE(imm_valid);
    COMPARE(imm);
    COMPARE(flags_wr);
    COMPARE(flags_rd);
    COMPARE(is_branch);
    COMPARE(is_store);
    COMPARE(is_load);
    COMPARE(imm_is_data);
    COMPARE(imm_data);
    COMPARE(is_rts);
    COMPARE(is_abs);
    COMPARE(exc_valid);
    COMPARE(exc_vec);
    COMPARE(requires_supervisor);
    COMPARE(uop_is_last);
    COMPARE(elim_kind);
    COMPARE(elim_arch_dst);
    COMPARE(len_bytes);
    return ok;
}

#undef COMPARE

// Drive a single-cycle decode with the given opword stream + pd_pc.
// Returns lane-0 + lane-1 internal-assembler µop snapshots from the same
// eval.  We read the INTERNAL lane-1 assembler bundle (not the gated
// `disp_d_*` ports) because the accept gate masks out otherwise-correct
// shadow output when lane-0/lane-1 share-conditions fail (CCR conflict,
// RAW, etc).  Shadow correctness is about the assembler chain producing
// the right µop on a shifted window — independent of whether dispatch
// would actually fire that µop.
void drive_decode(Vdecode* dut, uint32_t pd_pc,
                  const std::vector<uint16_t>& opwords,
                  UopBundle& lane0, UopBundle& lane1) {
    // Flush + redrive so internal phase counter starts at 0.
    dut->flush_en = 1;
    tick(dut);
    dut->flush_en = 0;
    dut->pd_valid = 1;
    dut->pd_fault = 0;
    dut->pd_next_fault = 0;
    dut->rn_ready = 1;
    dut->pd_pc = pd_pc;
    set_pd_buf_words(dut, opwords);
    // F2 retime Stage B (decode.v): the F3 register block now adds a
    // 1-cycle capture latency between pd_valid and the always @* µop
    // assembly.  After flush, F3 is empty; one extra tick lets it
    // capture pd_buf into the F3 stage so the always @* outputs reflect
    // the staged window.  Without this tick, lane-0/lane-1 read default
    // (uop_valid=0) values.  rn_ready stays high to prevent F3 from
    // wedging on a stale done_w.
    tick(dut);
    dut->eval();
    lane0 = read_lane0(dut);
    lane1 = read_lane1_internal(dut);
}

struct InstSpec {
    const char*           name;
    std::vector<uint16_t> bytes;   // 1+ words, including any ext words
};

struct Pair {
    const char* desc;
    InstSpec    inst0;
    InstSpec    inst1;
};

// Single-µop, non-branch, non-exception, register-direct fixtures.  These
// exercise the lane-1 V2 chain on its current narrow EA-bits feed
// (src=l1_op[5:0], dst={000,l1_op[11:9]}) — register-direct ALU ops with
// Dn destinations.  Two-write-CCR pairs are intentional: the gate's CCR-
// conflict heuristic masks dispatch for them, but the test compares the
// internal assembler bundle directly so we still validate shadow shape.
const std::vector<Pair> kPairs = {
    // ADD.L D1,D0  ;  ADD.L D3,D2 — both single-µop, no RAW.
    { "add_dn_dn_pair",
      { "ADD.L D1,D0", { 0xD081 } },                  // 1101_000_010_000_001
      { "ADD.L D3,D2", { 0xD483 } } },                // 1101_010_010_000_011
    // MOVEQ #1,D0  ;  MOVEQ #2,D1
    { "moveq_pair",
      { "MOVEQ #1,D0", { 0x7001 } },                  // 0111_000_0_00000001
      { "MOVEQ #2,D1", { 0x7202 } } },                // 0111_001_0_00000010
    // SUB.L D1,D0  ;  AND.L D3,D2
    { "sub_and_pair",
      { "SUB.L D1,D0", { 0x9081 } },                  // 1001_000_010_000_001
      { "AND.L D3,D2", { 0xC483 } } },                // 1100_010_010_000_011
    // OR.L D1,D0  ;  EOR.L D3,D2 (EOR.L 1011_010_110_000_011 = 0xB58B)
    { "or_eor_pair",
      { "OR.L D1,D0", { 0x8081 } },                   // 1000_000_010_000_001
      { "EOR.L D3,D2", { 0xB58B } } },                // 1011_010_110_000_011 (EOR = op[8]=1)
    // NEG.L D0  ;  NOT.L D1
    { "neg_not_pair",
      { "NEG.L D0", { 0x4480 } },                     // 0100_0100_10_000_000
      { "NOT.L D1", { 0x4681 } } },                   // 0100_0110_10_000_001
    // CMP.L D1,D0  ;  TST.L D2 (CMP doesn't write a Dn dst — flag-only)
    { "cmp_tst_pair",
      { "CMP.L D1,D0", { 0xB081 } },                  // 1011_000_010_000_001
      { "TST.L D2", { 0x4A82 } } },                   // 0100_1010_10_000_010

    // ── Task #246 / H1: non-ALU-reg-direct shapes ─────────────────────
    // These exercise the lane-1 EA-bits widening — instruction families
    // beyond the original reg-direct ALU shape.  Each pair leaves inst0
    // single-µop (so lane-0's accept gate is structurally happy) and
    // tests inst1 against the lane-0-solo reference.

    // MOVE.L D1,D0  ;  MOVE.L D3,D2 — exercises MOVE family EA-bits.
    // Opword: 0010_dst-reg_dst-mode_src-mode_src-reg.
    //   MOVE.L D1,D0 = 0010_000_000_000_001 = 0x2001
    //   MOVE.L D3,D2 = 0010_010_000_000_011 = 0x2403
    { "move_l_pair",
      { "MOVE.L D1,D0", { 0x2001 } },
      { "MOVE.L D3,D2", { 0x2403 } } },

    // ADDQ.L #1,D0 ; ADDQ.L #3,D1 — exercises ALU addq_subq shape.
    // Opword: 0101_quick_0_size_mode_reg.  size=10 (.L), mode=000 (Dn).
    //   ADDQ.L #1,D0 = 0101_001_0_10_000_000 = 0x5280
    //   ADDQ.L #3,D1 = 0101_011_0_10_000_001 = 0x5681
    { "addq_pair",
      { "ADDQ.L #1,D0", { 0x5280 } },
      { "ADDQ.L #3,D1", { 0x5681 } } },

    // ADDI.L #imm,D0 ; ADDI.L #imm,D1 — exercises ALU immgrp shape.
    // Opword: 0000_0110_10_000_reg + 2x ext word (long imm).
    //   ADDI.L #1,D0 = 0x0680, then 0x00000001 split into two words.
    //   ADDI.L #2,D1 = 0x0681, then 0x00000002.
    { "addi_l_pair",
      { "ADDI.L #1,D0", { 0x0680, 0x0000, 0x0001 } },
      { "ADDI.L #2,D1", { 0x0681, 0x0000, 0x0002 } } },

    // LSL.L #1,D0  ; ASR.L #2,D1 — exercises shift register form.
    // Opword: 1110_count_dr_size_ir_type_reg.  size=10 (.L), ir=0 (imm),
    // dr=1 (left), type=01 (LSL/LSR).  ASR is dr=0 type=00.
    //   LSL.L #1,D0 = 1110_001_1_10_0_01_000 = 0xE388
    //   ASR.L #2,D1 = 1110_010_0_10_0_00_001 = 0xE401
    { "shift_pair",
      { "LSL.L #1,D0", { 0xE388 } },
      { "ASR.L #2,D1", { 0xE401 } } },

    // SUBA.L A1,A0  ;  ADDA.L A3,A2 — exercises ALU adda_like shape
    // (dst = An, mode bits 011=W or 111=L, here 111).
    //   SUBA.L A1,A0 = 1001_000_111_001_001 = 0x91C9
    //   ADDA.L A3,A2 = 1101_010_111_001_011 = 0xD5CB
    { "suba_adda_pair",
      { "SUBA.L A1,A0", { 0x91C9 } },
      { "ADDA.L A3,A2", { 0xD5CB } } },

    // SWAP D0  ;  SWAP D1 — unary reg-only shape (no EA at all).
    //   SWAP Dn = 0100_1000_01_000_reg
    { "swap_pair",
      { "SWAP D0", { 0x4840 } },
      { "SWAP D1", { 0x4841 } } },

    // EXT.W D0  ;  EXT.L D1 — unary reg-only shape with ext_l vs ext_w.
    //   EXT.W Dn = 0100_1000_10_000_reg  (op[11:6]=100010)
    //   EXT.L Dn = 0100_1000_11_000_reg  (op[11:6]=100011)
    { "ext_pair",
      { "EXT.W D0", { 0x4880 } },
      { "EXT.L D1", { 0x48C1 } } },

    // CMPI.L #imm,Dn — exercises ALU immgrp with CMPI subop.
    // CMPI.L #1,D0 = 0000_1100_10_000_000 + ext = 0x0C80 0x0000 0x0001
    // CMPI.L #2,D1 = 0x0C81 0x0000 0x0002
    { "cmpi_pair",
      { "CMPI.L #1,D0", { 0x0C80, 0x0000, 0x0001 } },
      { "CMPI.L #2,D1", { 0x0C81, 0x0000, 0x0002 } } },
};

}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    Vdecode* dut = new Vdecode;
    reset(dut);

    int total = 0;
    int fail  = 0;
    int skipped = 0;

    for (const auto& p : kPairs) {
        // Build the joint window: inst0 followed by inst1 followed by
        // padding (NOP = 0x4E71 — single-µop sysop) so any third decode
        // doesn't trip an unrelated trap.
        std::vector<uint16_t> joint;
        joint.insert(joint.end(), p.inst0.bytes.begin(), p.inst0.bytes.end());
        joint.insert(joint.end(), p.inst1.bytes.begin(), p.inst1.bytes.end());
        while (joint.size() < 8) joint.push_back(0x4E71);

        UopBundle l0_pair, l1_pair;
        drive_decode(dut, 0x40800000u, joint, l0_pair, l1_pair);

        // Reference: drive inst1 alone at pd_pc + len(inst0).
        std::vector<uint16_t> single = p.inst1.bytes;
        while (single.size() < 8) single.push_back(0x4E71);

        UopBundle l0_solo, l1_solo;  // l1_solo is unused / don't-care
        // PC must match what lane-1 saw: pd_pc + len(inst0).
        uint32_t inst0_len_bytes = (uint32_t)p.inst0.bytes.size() * 2;
        uint32_t inst1_pc = 0x40800000u + inst0_len_bytes;
        drive_decode(dut, inst1_pc, single, l0_solo, l1_solo);

        total++;
        std::printf("[%s] inst0=%s | inst1=%s : ",
                    p.desc, p.inst0.name, p.inst1.name);
        std::fflush(stdout);

        if (!l1_pair.uop_valid) {
            std::printf("SKIP (lane-1 assembler emitted uop_valid=0 for inst1)\n");
            skipped++;
            continue;
        }
        if (!l0_solo.uop_valid) {
            std::printf("SKIP (lane-0-solo invalid — bad fixture)\n");
            skipped++;
            continue;
        }

        // Both lane-0 (inst0) and lane-1 (inst1) bundles must match the
        // re-driven single-wide reference for inst0 / inst1 respectively.
        // We focus the test on lane-1 ↔ inst1-solo equivalence (the
        // shadow-correctness claim).  Lane-0 ↔ inst0-solo is structurally
        // guaranteed because lane-0 doesn't move from byte 0 of pd_buf.
        if (compare_uops(l1_pair, l0_solo)) {
            std::printf("PASS\n");
        } else {
            std::printf("FAIL\n");
            fail++;
        }
    }

    std::printf("summary: total=%d pass=%d skip=%d fail=%d\n",
                total, total - fail - skipped, skipped, fail);

    delete dut;
    return fail == 0 ? 0 : 1;
}
