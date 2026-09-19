#!/usr/bin/env python3
"""Deterministic final-register comparison sweep against Musashi.

This driver is intentionally separate from the random fuzzer.  It emits
small one-case programs that exercise byte/word data-register writes plus a
handful of supported register-source, address-register, and postincrement
smoke cases, runs each program on both the RTL sim and Musashi, then diffs
final CPU register state.  The current completion path uses the existing PASS
sentinel store at 0xffff0000, so CCR is not compared by default because the
sentinel MOVE clobbers flags after the instruction under test.

Example:
    python3 tools/regstate/regstate_compare.py \
        --sim build/sim/Vmac_top \
        --musashi tb/models/musashi_run \
        --work build/regstate-smoke \
        --limit 32
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


LOAD_ADDR = 0x40800000
SENTINEL_ADDR = 0xFFFF0000
PASS_VALUE = 0xC0FFEE00
STOP_PC = LOAD_ADDR + 0x400

BASE_VALUES = [
    0x00000000,
    0xFFFFFFFF,
    0x12345678,
    0xA5A500FF,
    0x5A5AFF00,
    0x80007F80,
]
IMM8_VALUES = [0x00, 0x01, 0x7F, 0x80, 0xA5, 0xFF]
IMM16_VALUES = [0x0000, 0x0001, 0x7FFF, 0x8000, 0xA55A, 0xFFFF]
SRC_VALUES = [0x00000000, 0xFFFFFFFF, 0x13572468, 0xAA55CC33]
Q_VALUES = [1, 2, 7, 8]
SHIFT_COUNTS = [1, 4, 8]

INIT_D = [
    0x01020304,
    0x11223344,
    0x55667788,
    0x99AABBCC,
    0x0F1E2D3C,
    0xC3D2E1F0,
    0x89ABCDEF,
    0x76543210,
]

DEFAULT_COMPARE_KEYS = ["pass"] + [f"d{i}" for i in range(8)] + [f"a{i}" for i in range(7)]
IGNORED_KEYS = {"cycles", "committed", "last_pc", "a7", "ccr", "sr", "pc", "mem_writes"}
MEM_LINE_MASK = ~0x1F


@dataclass(frozen=True)
class Case:
    name: str
    family: str
    dst: int
    base: int
    body: tuple[str, ...]
    src_reg: int | None = None
    src_value: int | None = None
    dst_kind: str = "d"
    xfail: bool = False
    xfail_reason: str = ""


def run(cmd: list[str], cwd: Path | None = None, timeout: int | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def parse_u32(text: str) -> int:
    return int(text, 0) & 0xFFFFFFFF


def safe_name(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", s)


def choose_pass_reg(dst: int) -> int:
    return 7 if dst != 7 else 6


def choose_src_reg(dst: int) -> int:
    pass_reg = choose_pass_reg(dst)
    for reg in (6, 5, 4, 3):
        if reg != dst and reg != pass_reg:
            return reg
    raise AssertionError("unable to pick source register")


def add_case(cases: list[Case], family: str, dst: int, base: int, suffix: str,
             body: list[str], src_value: int | None = None,
             dst_kind: str = "d", xfail: bool = False,
             xfail_reason: str = "") -> None:
    src_reg = choose_src_reg(dst) if src_value is not None else None
    name = safe_name(f"{family}_{dst_kind}{dst}_b{base:08x}_{suffix}")
    cases.append(Case(name, family, dst, base, tuple(body), src_reg, src_value,
                      dst_kind, xfail, xfail_reason))


def generate_cases() -> list[Case]:
    cases: list[Case] = []

    for dst in range(8):
        for base in BASE_VALUES:
            for imm in IMM8_VALUES:
                add_case(cases, "move_b_imm", dst, base, f"i{imm:02x}",
                         [f"move.b  #0x{imm:02x}, %d{dst}"])
                add_case(cases, "andi_b_imm", dst, base, f"i{imm:02x}",
                         [f"andi.b  #0x{imm:02x}, %d{dst}"])
                add_case(cases, "ori_b_imm", dst, base, f"i{imm:02x}",
                         [f"ori.b   #0x{imm:02x}, %d{dst}"])
                add_case(cases, "eori_b_imm", dst, base, f"i{imm:02x}",
                         [f"eori.b  #0x{imm:02x}, %d{dst}"])

            for imm in IMM16_VALUES:
                add_case(cases, "move_w_imm", dst, base, f"i{imm:04x}",
                         [f"move.w  #0x{imm:04x}, %d{dst}"])
                add_case(cases, "andi_w_imm", dst, base, f"i{imm:04x}",
                         [f"andi.w  #0x{imm:04x}, %d{dst}"])
                add_case(cases, "ori_w_imm", dst, base, f"i{imm:04x}",
                         [f"ori.w   #0x{imm:04x}, %d{dst}"])
                add_case(cases, "eori_w_imm", dst, base, f"i{imm:04x}",
                         [f"eori.w  #0x{imm:04x}, %d{dst}"])

            for q in Q_VALUES:
                add_case(cases, "addq_b", dst, base, f"q{q}",
                         [f"addq.b  #{q}, %d{dst}"])
                add_case(cases, "subq_b", dst, base, f"q{q}",
                         [f"subq.b  #{q}, %d{dst}"])
                add_case(cases, "addq_w", dst, base, f"q{q}",
                         [f"addq.w  #{q}, %d{dst}"])
                add_case(cases, "subq_w", dst, base, f"q{q}",
                         [f"subq.w  #{q}, %d{dst}"])

            for op in ("clr", "not", "neg"):
                add_case(cases, f"{op}_b", dst, base, "u",
                         [f"{op}.b   %d{dst}"])
                add_case(cases, f"{op}_w", dst, base, "u",
                         [f"{op}.w   %d{dst}"])

            add_case(cases, "ext_w", dst, base, "u", [f"ext.w   %d{dst}"])
            add_case(cases, "ext_l", dst, base, "u", [f"ext.l   %d{dst}"])

            for op in ("asl", "asr", "lsl", "lsr", "rol", "ror"):
                for count in SHIFT_COUNTS:
                    add_case(cases, f"{op}_b", dst, base, f"c{count}",
                             [f"{op}.b   #{count}, %d{dst}"])
                    add_case(cases, f"{op}_w", dst, base, f"c{count}",
                             [f"{op}.w   #{count}, %d{dst}"])

            for src_value in SRC_VALUES:
                for op in ("move", "and", "or", "eor", "add", "sub"):
                    src = choose_src_reg(dst)
                    add_case(cases, f"{op}_b_reg", dst, base, f"s{src_value:08x}",
                             [f"{op}.b   %d{src}, %d{dst}"], src_value)
                    add_case(cases, f"{op}_w_reg", dst, base, f"s{src_value:08x}",
                             [f"{op}.w   %d{src}, %d{dst}"], src_value)

    # Focused regression slices for corners that are easier to validate
    # as end-of-test register state than inside the random fuzzer.
    byte_unary_cases = [
        ("clr", 0, 0x123456a5),
        ("not", 1, 0x89abcd10),
        ("neg", 2, 0x00000080),
    ]
    for op, dst, base in byte_unary_cases:
        add_case(cases, "byte_unary", dst, base, op,
                 [f"{op}.b   %d{dst}"])

    # Final-state smoke cases that exercise address-register sign-extension and
    # partial-register writeback on the same narrow families the ROM has hit.
    add_case(cases, "movea_w_imm", 4, 0x00000000, "w8001",
             ["movea.w #0x8001, %a4"], dst_kind="a")
    add_case(cases, "movea_postinc", 4, 0x00104000, "a0",
             [
                 "    move.l  #0x11223344, (%a0)",
                 "    movea.l (%a0)+, %a4",
                 "    cmpa.l  #0x00104004, %a0",
                 "    cmpa.l  #0x11223344, %a4",
             ], dst_kind="a")
    add_case(cases, "movea_postinc", 5, 0x00105000, "a1",
             [
                 "    move.l  #0x55667788, (%a1)",
                 "    movea.l (%a1)+, %a5",
                 "    cmpa.l  #0x00105004, %a1",
                 "    cmpa.l  #0x55667788, %a5",
             ], dst_kind="a")
    add_case(cases, "move_w_postinc", 0, 0xCAFEBABE, "w8001",
             [
                 "    move.w  #0x8001, (%a0)",
                 "    move.w  (%a0)+, %d0",
                 "    cmpa.l  #0x00100002, %a0",
             ])
    add_case(cases, "move_b_postinc", 1, 0x13572468, "b80",
             [
                 "    move.b  #0x80, (%a1)",
                 "    move.b  (%a1)+, %d1",
                 "    cmpa.l  #0x00101001, %a1",
             ])
    add_case(cases, "swap", 2, 0x11223344, "u",
             ["swap    %d2"])
    add_case(cases, "extb", 3, 0x12345680, "u",
             ["extb.l  %d3"])

    # Curated register-source slices for supported byte/word forms.  These
    # intentionally avoid broader EOR/SUB register families until those decode
    # paths stop producing mismatches/timeouts in this harness.
    move_reg_cases = [
        ("b", 0, 0x12345678, 0xAA55CC33),
        ("w", 1, 0xA5A500FF, 0x13572468),
    ]
    for size, dst, base, src_value in move_reg_cases:
        src = choose_src_reg(dst)
        add_case(cases, "move_reg_partial", dst, base, f"{size}_s{src_value:08x}",
                 [f"move.{size}  %d{src}, %d{dst}"], src_value)

    alu_reg_cases = [
        ("and", "b", 2, 0x12345678, 0xAA55CC33),
        ("and", "w", 3, 0xA5A500FF, 0x13572468),
        ("or", "b", 4, 0x80007F80, 0x13572468),
        ("or", "w", 5, 0x5A5AFF00, 0xAA55CC33),
        ("add", "b", 6, 0x80007F80, 0xFFFFFFFF),
        ("add", "w", 7, 0x5A5AFF00, 0xAA55CC33),
    ]
    for op, size, dst, base, src_value in alu_reg_cases:
        src = choose_src_reg(dst)
        add_case(cases, "alu_reg_bw_supported", dst, base,
                 f"{op}{size}_s{src_value:08x}",
                 [f"{op}.{size}   %d{src}, %d{dst}"], src_value)

    # Curated quick-op slices.  The broad generated addq/subq matrix above is
    # intentionally opt-in; these cases pin already-supported register forms
    # that the ROM path relies on, and keep memory-destination probes available
    # as an explicit triage slice.
    quick_reg_cases = [
        ("addq", "b", "d", 5, 0x123456FC, 4),
        ("subq", "b", "d", 1, 0x0BADBE00, 1),
        ("addq", "w", "d", 3, 0x007C00FC, 1),
        ("subq", "w", "d", 2, 0xABCD0003, 4),
        ("addq", "l", "d", 0, 0x00000000, 8),
        ("subq", "l", "d", 4, 0x00000008, 8),
        ("addq", "l", "a", 1, 0x00102000, 4),
        ("subq", "l", "a", 2, 0x00103008, 8),
    ]
    for op, size, kind, dst, base, q in quick_reg_cases:
        add_case(cases, "quick_reg_supported", dst, base,
                 f"{op}{size}_q{q}",
                 [f"{op}.{size}  #{q}, %{kind}{dst}"], dst_kind=kind)

    quick_mem_cases = [
        ("addq", "b", 0, "0x112233fc", "3(%a0)", "(%a0)", 4),
        ("subq", "b", 1, "0x44556600", "3(%a1)", "(%a1)", 1),
        ("addq", "w", 2, "0x7788ffff", "2(%a2)", "(%a2)", 1),
        ("subq", "w", 3, "0x99aa0003", "2(%a3)", "(%a3)", 4),
        ("addq", "l", 4, "0x00000000", "(%a4)", "(%a4)", 8),
        ("subq", "l", 5, "0x00000008", "(%a5)", "(%a5)", 8),
    ]
    for op, size, dst, initial, op_ea, load_ea, q in quick_mem_cases:
        add_case(cases, "quick_mem_supported", dst, 0x00000000,
                 f"{op}{size}_q{q}",
                 [
                     f"    move.l  #{initial}, (%a{dst})",
                     f"    {op}.{size}  #{q}, {op_ea}",
                     f"    move.l  {load_ea}, %d{dst}",
                 ])

    # Q700 ROM-frontier decode/addressing smoke.  These mirror exact opwords
    # from the directed ROM-frontier tests, then fold observable memory effects
    # back into final registers so the default parity gate stays write-log quiet.
    add_case(cases, "rom_frontier_movew_areg", 0, 0x12345678, "a7w8000",
             [
                 "    lea     0x00208000, %a7",
                 "    .word   0x300f",
             ])
    add_case(cases, "rom_frontier_movel_areg_postinc", 1, 0x00000000, "a0a7post",
             [
                 "    lea     0x00208100, %a7",
                 "    movea.l #0x80000004, %a0",
                 "    .word   0x2ec8",
                 "    move.l  0x00208100, %d1",
             ])
    add_case(cases, "rom_frontier_movem_pc_disp", 0, 0x00000000, "d0_d5",
             [
                 ".Lmovem_case:",
                 "    .word   0x4cfa, 0x003f",
                 "    .word   .Lmovem_table - (.Lmovem_case + 4)",
                 "    bra     .Lmovem_after",
                 "    .align  2",
                 ".Lmovem_table:",
                 "    .long   0x11111111",
                 "    .long   0x22222222",
                 "    .long   0x33333333",
                 "    .long   0x44444444",
                 "    .long   0x55555555",
                 "    .long   0x66666666",
                 ".Lmovem_after:",
             ])
    add_case(cases, "rom_frontier_lea_pc_indexed", 0, 0x00000020, "a0_alias",
             [
                 "    .word   0x41fb, 0x88f8",
             ], dst_kind="a")
    add_case(cases, "rom_frontier_lea_scaled_alias", 5, 0x00100000, "d1w4",
             [
                 "    move.l  #0x00000003, %d1",
                 "    .word   0x4bf5, 0x1400",
             ], dst_kind="a")
    add_case(cases, "rom_frontier_suba_mem", 1, 0x08000000, "a5_ind",
             [
                 "    lea     0x0010000c, %a5",
                 "    move.l  #0x00400000, %d0",
                 "    move.l  %d0, (%a5)",
                 "    suba.l  (%a5), %a1",
             ], dst_kind="a")
    add_case(cases, "rom_frontier_scc_indexed", 2, 0x00000000, "seq_d1l",
             [
                 "    lea     0x0010a000, %a0",
                 "    moveq   #5, %d1",
                 "    move.l  #0x11223344, 0x17(%a0)",
                 "    moveq   #0, %d6",
                 "    tst.l   %d6",
                 "    seq     0x12(%a0, %d1.l)",
                 "    move.l  0x17(%a0), %d2",
             ])
    add_case(cases, "rom_frontier_cmpb_indexed", 2, 0x00000000, "d2w",
             [
                 "    lea     .Lcmp_bytes, %a0",
                 "    move.l  #0x00010003, %d2",
                 "    moveq   #0x5a, %d1",
                 "    .word   0xb230, 0x2000",
                 "    beq     .Lcmp_ok",
                 "    moveq   #0x22, %d2",
                 "    bra     .Lcmp_done",
                 ".Lcmp_ok:",
                 "    moveq   #0x11, %d2",
                 ".Lcmp_done:",
                 "    bra     .Lcmp_after",
                 "    .align  2",
                 ".Lcmp_bytes:",
                 "    .byte   0x10, 0x20, 0x30, 0x5a",
                 ".Lcmp_after:",
             ])
    add_case(cases, "rom_frontier_notb_indexed", 2, 0x00000000, "d2w",
             [
                 "    lea     0x0010b000, %a0",
                 "    move.l  #0x112233a5, (%a0)",
                 "    move.l  #0x00010003, %d2",
                 "    .word   0x4630, 0x2000",
                 "    move.l  (%a0), %d2",
             ])
    add_case(cases, "rom_frontier_jmp_pc_indexed", 0, 0x00000000, "d3w",
             [
                 "    move.w  #(.Ljmp_target - (.Ljmp_case + 4)), %d3",
                 ".Ljmp_case:",
                 "    .word   0x4efb, 0x3002",
                 "    move.l  #0xdead0001, %d0",
                 "    bra     .Ljmp_after",
                 ".Ljmp_target:",
                 "    move.l  #0xc001d00d, %d0",
                 ".Ljmp_after:",
             ])
    add_case(cases, "rom_frontier_moveb_indexed_dst", 1, 0x00000000, "d3l",
             [
                 "    lea     0x00107000, %a3",
                 "    moveq   #4, %d3",
                 "    move.l  #0x00000080, %d0",
                 "    move.b  %d0, (2,%a3,%d3.l)",
                 "    move.b  6(%a3), %d1",
             ])
    add_case(cases, "rom_frontier_movel_full_memind", 0, 0x00000000, "a4bd_od",
             [
                 "    lea     0x0010f390, %a4",
                 "    lea     0x0010f380, %a5",
                 "    move.l  #0x0010f3b4, (%a5)",
                 "    lea     0x0010f3b0, %a5",
                 "    move.l  #0x89abcdef, (%a5)",
                 "    .word   0x2034, 0x8162, 0xfff0, 0xfffc",
             ])
    add_case(cases, "rom_frontier_movel_full_memind_no_outer", 1, 0x00000000, "a4bd_null",
             [
                 "    lea     0x0010f430, %a4",
                 "    lea     0x0010f420, %a5",
                 "    move.l  #0x0010f460, (%a5)",
                 "    lea     0x0010f460, %a5",
                 "    move.l  #0x00000039, (%a5)",
                 "    .word   0x2234, 0x8161, 0xfff0",
             ])
    add_case(cases, "rom_frontier_subq_mem_disp", 0, 0x00000000, "a4m16",
             [
                 "    lea     0x0010f410, %a4",
                 "    move.l  #0x00000020, -16(%a4)",
                 "    .word   0x59ac, 0xfff0",
                 "    move.l  -16(%a4), %d0",
             ])
    add_case(cases, "rom_frontier_movew_imm_disp", 2, 0x00000000, "a4m154",
             [
                 "    lea     0x0010f500, %a4",
                 "    .word   0x397c, 0x0001, 0xff66",
                 "    move.w  -154(%a4), %d2",
             ])
    add_case(cases, "rom_frontier_movel_indexed_mem_to_indexed", 0, 0x00000000, "a1d2w_a0d2w",
             [
                 "    lea     0x0010f600, %a1",
                 "    lea     0x0010f700, %a0",
                 "    moveq   #8, %d2",
                 "    move.l  #0x89abcdef, 8(%a1)",
                 "    move.l  #0x00000000, 8(%a0)",
                 "    .word   0x21b1, 0x2000, 0x2000",
                 "    move.l  8(%a0), %d0",
             ])

    # Stop-PC parity cases preserve final CCR/SR because they do not finish
    # with a sentinel MOVE.  They are intentionally few and deterministic so
    # the Makefile smoke can compare D/A/A7/PC/SR/CCR in CI.
    add_case(cases, "parity_trap_rte_ccr_restore", 4, 0x00000000, "trap0",
             [
                 "    lea     0x00018000, %a7",
                 "    move.l  #_trap_rte_restore_handler, 0x00000080",
                 "    moveq   #0, %d0",
                 "    trap    #0",
                 "    bra     _trap_rte_after",
                 "_trap_rte_restore_handler:",
                 "    moveq   #1, %d0",
                 "    rte",
                 "_trap_rte_after:",
             ],
             dst_kind="d")
    add_case(cases, "parity_user_stack_switch", 7, 0x00000000, "trap0",
             [
                 "    lea     0x00020000, %a7",
                 "    move.l  #_stack_switch_handler, 0x00000080",
                 "    andi.w  #0xDFFF, %sr",
                 "    lea     0x00008000, %a7",
                 "    trap    #0",
                 "    bra     _stack_switch_after",
                 "_stack_switch_handler:",
                 "    cmp.l   #0x0001fff8, %a7",
                 "    bne     stack_switch_fail",
                 "    move.l  4(%a7), %d0",
                 "    and.l   #0x00000fff, %d0",
                 "    cmp.l   #0x00000080, %d0",
                 "    bne     stack_switch_fail",
                 "    rte",
                 "stack_switch_fail:",
                 "    move.l  #0xbadf0004, %d6",
                 "    rte",
                 "_stack_switch_after:",
             ],
             dst_kind="a")

    add_case(cases, "parity_vector_frame_regs", 1, 0x00000000, "fmt0",
             [
                 "    lea     0x00019000, %a7",
                 "    move.l  #0x20044080, %d0",
                 "    move.l  %d0, (%a7)",
                 "    move.l  #0x007e002c, %d0",
                 "    move.l  %d0, 4(%a7)",
                 "    move.w  6(%a7), %d1",
                 "    and.w   #0x0fff, %d1",
                 "    cmp.w   #0x002c, %d1",
                 "    bne     frame_bad",
                 "    move.w  2(%a7), %d2",
                 "    cmp.w   #0x4080, %d2",
                 "    bne     frame_bad",
                 "    move.w  4(%a7), %d2",
                 "    cmp.w   #0x007e, %d2",
                 "    bne     frame_bad",
                 "    move.w  (%a7), %d3",
                 "    and.w   #0x2004, %d3",
                 "    move.l  #0x13579bdf, %d6",
                 "    cmp.w   #0x2004, %d3",
                 "    bne     frame_bad",
                 "    bra.w   _done",
                 "frame_bad:",
                 "    move.l  #0xbadf0001, %d6",
                 "    bra.w   _done",
             ])

    add_case(cases, "parity_rom_frontier", 2, 0x00000000, "probe",
             [
                 "    lea     0x0001a000, %a7",
                 "    lea     0x00112000, %a0",
                 "    move.l  #0x00112000, (%a7)",
                 "    moveq   #3, %d2",
                 "    move.l  #0x11223355, %d0",
                 "    move.l  %d0, (%a0)",
                 "    move.l  #0x00000055, %d1",
                 "    movea.l (%a7)+, %a0",
                 "    cmp.b   (0,%a0,%d2.w), %d1",
                 "    bne     rom_probe_bad",
                 "    not.b   %d1",
                 "    not.b   (0,%a0,%d2.w)",
                 "    move.l  (%a0), %d3",
                 "    move.l  #0x2468ace0, %d4",
                 "    cmp.l   #0x112233aa, %d3",
                 "    bne     rom_probe_bad",
                 "    bra.w   _done",
                 "rom_probe_bad:",
                 "    move.l  #0xbadf0002, %d4",
                 "    bra.w   _done",
             ])

    # Sentinel-completion adversarial cases.  These keep the ROM-frontier
    # and memory-write surfaces honest without depending on the stop-PC path.
    add_case(cases, "parity_word_odd3", 1, 0x00000000, "odd3",
             [
                 "    lea     0x00020000, %a7",
                 "    lea     0x00010003, %a0",
                 "    move.w  #0xBEEF, (%a0)",
                 "    move.w  (%a0), %d1",
                 "    cmp.l   #0x0000BEEF, %d1",
                 "    bne     word_odd3_fail",
                 "    bra     word_odd3_done",
                 "word_odd3_fail:",
                 "    move.l  #0xbadf0002, %d6",
                 "word_odd3_done:",
             ])
    add_case(cases, "parity_store_load_same_addr", 2, 0x00000000, "repeat",
             [
                 "    lea     0x00020000, %a7",
                 "    lea     0x00015000, %a2",
                 "    move.l  #0xCAFEBABE, (%a2)",
                 "    move.l  (%a2), %d0",
                 "    cmp.l   #0xCAFEBABE, %d0",
                 "    bne     store_load_same_addr_fail",
                 "    move.l  #0x12345678, (%a2)",
                 "    move.l  (%a2), %d1",
                 "    cmp.l   #0x12345678, %d1",
                 "    bne     store_load_same_addr_fail",
                 "    move.l  #0x00000000, (%a2)",
                 "    move.l  (%a2), %d2",
                 "    cmp.l   #0x00000000, %d2",
                 "    bne     store_load_same_addr_fail",
                 "    bra     store_load_same_addr_done",
                 "store_load_same_addr_fail:",
                 "    move.l  #0xbadf0003, %d6",
                 "store_load_same_addr_done:",
             ])
    add_case(cases, "parity_movem_an_in_list", 7, 0x00000000, "a7list",
             [
                 "    lea     0x00020000, %a7",
                 "    move.l  #0x1AAAAAA1, %a0",
                 "    move.l  #0x2BBBBBB2, %a1",
                 "    move.l  #0x3CCCCCC3, %a2",
                 "    movem.l %a0-%a2/%a7, -(%a7)",
                 "    move.l  (%a7)+, %d0",
                 "    move.l  (%a7)+, %d1",
                 "    move.l  (%a7)+, %d2",
                 "    move.l  (%a7)+, %d3",
             ], dst_kind="a", xfail=True,
             xfail_reason="MOVEM.L with A7 in the reglist still disagrees on the pushed/popped A7 slot")

    return cases


def render_program(case: Case, completion: str = "sentinel") -> str:
    pass_reg = choose_pass_reg(case.dst)
    lines: list[str] = [
        "    .text",
        "    .globl _start",
        "_start:",
    ]

    for reg, value in enumerate(INIT_D):
        lines.append(f"    move.l  #0x{value:08x}, %d{reg}")
    for reg in range(7):
        lines.append(f"    lea     0x{0x00100000 + reg * 0x1000:08x}, %a{reg}")

    if case.dst_kind == "a":
        dst_seed = f"    lea     0x{case.base:08x}, %a{case.dst}"
    else:
        dst_seed = f"    move.l  #0x{case.base:08x}, %d{case.dst}"

    lines.extend([
        f"    lea     0x{SENTINEL_ADDR:08x}, %a6",
        f"    move.l  #0x{PASS_VALUE:08x}, %d{pass_reg}",
        dst_seed,
    ])

    if case.src_reg is not None and case.src_value is not None:
        lines.append(f"    move.l  #0x{case.src_value:08x}, %d{case.src_reg}")

    for body_line in case.body:
        if body_line.endswith(":") or body_line.startswith("    "):
            lines.append(body_line)
        else:
            lines.append(f"    {body_line}")

    if completion == "stop-pc":
        lines.extend([
            "    bra.w   _done",
            f"    .org    0x{STOP_PC - LOAD_ADDR:04x}",
            "_done:",
            "    bra     _done",
            "",
        ])
    else:
        lines.extend([
            f"    move.l  %d{pass_reg}, (%a6)",
            "_halt:",
            "    bra     _halt",
            "",
        ])
    return "\n".join(lines)


def assemble(asm_src: Path, bin_out: Path, as_tool: str, ld_tool: str,
             objcopy_tool: str) -> tuple[bool, str]:
    obj = bin_out.with_suffix(".o")
    elf = bin_out.with_suffix(".elf")
    steps = [
        [as_tool, "-m68040", "-o", str(obj), str(asm_src)],
        [ld_tool, "-Ttext", f"0x{LOAD_ADDR:08x}", "-o", str(elf), str(obj)],
        [objcopy_tool, "-O", "binary", str(elf), str(bin_out)],
    ]
    for step in steps:
        p = run(step)
        if p.returncode != 0:
            return False, f"{step[0]} rc={p.returncode}: {p.stderr.strip()}"
    return True, ""


def parse_state(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            out[key] = value
    return out


def compare_states(rtl: dict[str, str], ref: dict[str, str],
                   compare_keys: list[str], include_memory: bool) -> list[str]:
    keys = set(compare_keys)
    if include_memory:
        keys |= {k for k in rtl if k.startswith("mem[")}
        keys |= {k for k in ref if k.startswith("mem[")}

    diffs: list[str] = []
    for key in sorted(keys):
        if key in IGNORED_KEYS and key not in compare_keys:
            continue
        if key.startswith("mem["):
            try:
                addr = int(key[len("mem[0x"):-1], 16)
            except ValueError:
                addr = 0
            if SENTINEL_ADDR <= addr < SENTINEL_ADDR + 0x10:
                continue
        rv = rtl.get(key)
        mv = ref.get(key)
        if rv != mv:
            diffs.append(f"{key}: rtl={rv!r} musashi={mv!r}")
    return diffs


def _mem_diff_summary(diffs: list[str]) -> str | None:
    mem_diffs = [d for d in diffs if d.startswith("mem[0x")]
    if not mem_diffs:
        return None

    addrs: list[int] = []
    presence_only = True
    stale_byte = True
    for diff in mem_diffs:
        key, rest = diff.split(":", 1)
        try:
            addrs.append(int(key[len("mem[0x"):-1], 16))
        except ValueError:
            pass
        rtl_missing = "rtl=None" in rest
        musashi_missing = "musashi=None" in rest
        if rtl_missing == musashi_missing:
            presence_only = False
        if not (("rtl='0xff'" in rest and musashi_missing) or
                ("musashi='0xff'" in rest and rtl_missing)):
            stale_byte = False

    lines = sorted({addr & MEM_LINE_MASK for addr in addrs})
    line_desc = ", ".join(f"0x{line:08x}" for line in lines[:4])
    if len(lines) > 4:
        line_desc += f", ... (+{len(lines) - 4} more)"
    kind = "stale-byte-line" if stale_byte else (
        "presence-only" if presence_only else "mixed-values"
    )
    return f"memory-write-log: {kind} bytes={len(mem_diffs)} lines={line_desc}"


def run_musashi(musashi: Path, bin_path: Path, out_state: Path,
                max_cycles: int, stop_pc: int | None = None) -> tuple[bool, str]:
    cmd = [
        str(musashi),
        "--bin", str(bin_path),
        "--load-addr", f"0x{LOAD_ADDR:08x}",
        "--sentinel", f"0x{SENTINEL_ADDR:08x}",
        "--max-cycles", str(max_cycles),
        "--out", str(out_state),
    ]
    if stop_pc is not None:
        cmd += ["--stop-pc", f"0x{stop_pc:08x}"]
    p = run(cmd)
    if p.returncode == 0:
        return True, ""
    return False, f"musashi rc={p.returncode}: {p.stderr.strip()}"


def run_rtl(sim: Path, test_name: str, bin_path: Path, out_state: Path,
            timeout_cycles: int, host_timeout: int | None,
            stop_pc: int | None = None) -> tuple[bool, str]:
    cmd = [
        str(sim),
        f"+test={test_name}",
        f"+bin={bin_path}",
        f"+binaddr={LOAD_ADDR:08x}",
        f"+timeout={timeout_cycles}",
        f"+dump_final_state={out_state}",
    ]
    if stop_pc is not None:
        cmd.append(f"+stop_pc={stop_pc:08x}")
    p = run(cmd, timeout=host_timeout)
    if p.returncode == 0:
        return True, ""
    tail = (p.stdout + p.stderr)[-800:]
    return False, f"rtl rc={p.returncode}: {tail.strip()}"


def filter_cases(cases: list[Case], args: argparse.Namespace) -> list[Case]:
    selected = cases
    if args.family:
        families = set(args.family)
        selected = [c for c in selected if c.family in families]
    if args.dst is not None:
        dsts = {parse_u32(d) for d in args.dst}
        selected = [c for c in selected if c.dst in dsts]
    if args.case:
        wanted = set(args.case)
        selected = [c for c in selected if c.name in wanted]
    if args.start_at:
        names = [c.name for c in selected]
        if args.start_at not in names:
            raise SystemExit(f"--start-at case not found after filtering: {args.start_at}")
        selected = selected[names.index(args.start_at):]
    if args.limit is not None:
        selected = selected[:args.limit]
    return selected


def ensure_tool(path: Path, label: str) -> None:
    if not path.exists():
        raise SystemExit(f"{label} not found: {path}")
    if not path.is_file():
        raise SystemExit(f"{label} is not a file: {path}")


def preserve_artifacts(paths: list[Path], fail_dir: Path) -> None:
    for path in paths:
        if path.exists() and path.is_file():
            shutil.copyfile(path, fail_dir / path.name)


def main() -> int:
    repo = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(
        description="Run deterministic final-register RTL vs Musashi comparisons."
    )
    ap.add_argument("--sim", default=str(repo / "build/sim/Vmac_top"),
                    help="Vmac_top simulator path")
    ap.add_argument("--musashi", default=str(repo / "tb/models/musashi_run"),
                    help="Musashi standalone runner path")
    ap.add_argument("--work", default=str(repo / "build/regstate-sweep"),
                    help="scratch/output directory")
    ap.add_argument("--limit", type=int, default=None,
                    help="run only the first N selected cases")
    ap.add_argument("--start-at", default=None,
                    help="start at a specific generated case name")
    ap.add_argument("--family", action="append", default=None,
                    help="run only this family; may be repeated")
    ap.add_argument("--dst", action="append", default=None,
                    help="run only this destination D register number; may be repeated")
    ap.add_argument("--case", action="append", default=None,
                    help="run only this exact generated case name; may be repeated")
    ap.add_argument("--list", action="store_true",
                    help="list selected cases without assembling or running")
    ap.add_argument("--keep-asm", action="store_true",
                    help="keep generated assembly and binaries; default also keeps failing cases")
    ap.add_argument("--compare-ccr", action="store_true",
                    help="also compare final CCR; usually reflects the sentinel store")
    ap.add_argument("--compare-sr", action="store_true",
                    help="also compare final SR; best used with --completion stop-pc")
    ap.add_argument("--compare-pc", action="store_true",
                    help="also compare final committed PC; best used with --completion stop-pc")
    ap.add_argument("--compare-a7", action="store_true",
                    help="also compare A7; use only for cases that initialise it explicitly")
    ap.add_argument("--include-memory", action="store_true",
                    help="also compare non-sentinel memory writes; opt-in because writeback-log noise is known")
    ap.add_argument("--completion", choices=("sentinel", "stop-pc"),
                    default="sentinel",
                    help="program completion mechanism; stop-pc preserves CCR/SR")
    ap.add_argument("--timeout", type=int, default=200000,
                    help="RTL cycle timeout")
    ap.add_argument("--host-timeout", type=int, default=20,
                    help="host seconds allowed per RTL case; 0 disables")
    ap.add_argument("--musashi-max", type=int, default=200000,
                    help="Musashi cycle budget")
    ap.add_argument("--max-diffs", type=int, default=20,
                    help="maximum mismatch cases to print in detail")
    ap.add_argument("--as", dest="as_tool", default="m68k-linux-gnu-as")
    ap.add_argument("--ld", dest="ld_tool", default="m68k-linux-gnu-ld")
    ap.add_argument("--objcopy", dest="objcopy_tool", default="m68k-linux-gnu-objcopy")
    args = ap.parse_args()

    all_cases = generate_cases()
    cases = filter_cases(all_cases, args)

    if args.list:
        for c in cases:
            if c.xfail:
                reason = f" [XFAIL: {c.xfail_reason}]" if c.xfail_reason else " [XFAIL]"
                print(f"{c.name}{reason}")
            else:
                print(c.name)
        print(f"selected={len(cases)} total={len(all_cases)}")
        return 0

    if not cases:
        print(f"regstate: no cases selected (total generated={len(all_cases)})")
        return 1

    sim = Path(args.sim).resolve()
    musashi = Path(args.musashi).resolve()
    ensure_tool(sim, "sim")
    ensure_tool(musashi, "musashi")

    for tool in (args.as_tool, args.ld_tool, args.objcopy_tool):
        if shutil.which(tool) is None:
            raise SystemExit(f"required tool not found in PATH: {tool}")

    work = Path(args.work).resolve()
    asm_dir = work / "asm"
    bin_dir = work / "bin"
    state_dir = work / "state"
    fail_dir = work / "fails"
    for d in (asm_dir, bin_dir, state_dir, fail_dir):
        d.mkdir(parents=True, exist_ok=True)

    compare_keys = list(DEFAULT_COMPARE_KEYS)
    if args.compare_ccr:
        compare_keys.append("ccr")
    if args.compare_sr:
        compare_keys.append("sr")
    if args.compare_pc:
        compare_keys.append("pc")
    if args.compare_a7:
        compare_keys.append("a7")
    stop_pc = STOP_PC if args.completion == "stop-pc" else None

    n_pass = n_mismatch = n_timeout = n_error = 0
    n_xfail = n_xpass = 0
    details: list[tuple[str, list[str]]] = []

    print(f"regstate: selected={len(cases)} total={len(all_cases)} work={work}")
    for idx, case in enumerate(cases, 1):
        asm_path = asm_dir / f"{case.name}.s"
        bin_path = bin_dir / f"{case.name}.bin"
        rtl_state_path = state_dir / f"{case.name}.rtl.txt"
        musashi_state_path = state_dir / f"{case.name}.musashi.txt"

        asm_path.write_text(render_program(case, args.completion))
        ok, msg = assemble(asm_path, bin_path, args.as_tool, args.ld_tool, args.objcopy_tool)
        if not ok:
            if case.xfail:
                n_xfail += 1
                if len(details) < args.max_diffs:
                    reason = f"XFAIL ({case.xfail_reason})" if case.xfail_reason else "XFAIL"
                    details.append((case.name, [f"{reason}", f"assemble: {msg}"]))
            else:
                n_error += 1
                details.append((case.name, [f"assemble: {msg}"]))
            preserve_artifacts([asm_path], fail_dir)
            sys.stdout.write("x" if case.xfail else "E")
            sys.stdout.flush()
            continue

        ok_m, msg_m = run_musashi(musashi, bin_path, musashi_state_path,
                                  args.musashi_max, stop_pc)
        ok_r, msg_r = run_rtl(
            sim, f"regstate_{case.name}", bin_path, rtl_state_path, args.timeout,
            None if args.host_timeout == 0 else args.host_timeout, stop_pc,
        )

        if not ok_m or not musashi_state_path.exists():
            if case.xfail:
                n_xfail += 1
                if len(details) < args.max_diffs:
                    reason = f"XFAIL ({case.xfail_reason})" if case.xfail_reason else "XFAIL"
                    details.append((case.name, [reason, msg_m or "musashi state not produced"]))
                sys.stdout.write("x")
            elif "rc=1" in msg_m:
                n_timeout += 1
                details.append((case.name, [msg_m or "musashi did not hit sentinel"]))
                sys.stdout.write("T")
            else:
                n_error += 1
                details.append((case.name, [msg_m or "musashi state not produced"]))
                sys.stdout.write("E")
            preserve_artifacts([
                asm_path, bin_path, rtl_state_path, musashi_state_path
            ], fail_dir)
            sys.stdout.flush()
            continue
        if not ok_r or not rtl_state_path.exists():
            if case.xfail:
                n_xfail += 1
                if len(details) < args.max_diffs:
                    reason = f"XFAIL ({case.xfail_reason})" if case.xfail_reason else "XFAIL"
                    details.append((case.name, [reason, msg_r or "rtl state not produced"]))
                sys.stdout.write("x")
            elif "TIMEOUT" in msg_r:
                n_timeout += 1
                details.append((case.name, [msg_r or "rtl did not hit sentinel"]))
                sys.stdout.write("T")
            else:
                n_error += 1
                details.append((case.name, [msg_r or "rtl state not produced"]))
                sys.stdout.write("E")
            preserve_artifacts([
                asm_path, bin_path, rtl_state_path, musashi_state_path
            ], fail_dir)
            sys.stdout.flush()
            continue

        rtl_state = parse_state(rtl_state_path)
        musashi_state = parse_state(musashi_state_path)
        diffs = compare_states(rtl_state, musashi_state, compare_keys, args.include_memory)
        if diffs:
            if case.xfail:
                n_xfail += 1
                if len(details) < args.max_diffs:
                    reason = f"XFAIL ({case.xfail_reason})" if case.xfail_reason else "XFAIL"
                    details.append((case.name, [reason, *diffs]))
            else:
                n_mismatch += 1
                if len(details) < args.max_diffs:
                    details.append((case.name, diffs))
            preserve_artifacts([
                asm_path, bin_path, rtl_state_path, musashi_state_path
            ], fail_dir)
            sys.stdout.write("x" if case.xfail else "M")
        else:
            if case.xfail:
                n_xpass += 1
                if len(details) < args.max_diffs:
                    reason = f"XPASS ({case.xfail_reason})" if case.xfail_reason else "XPASS"
                    details.append((case.name, [reason]))
                sys.stdout.write("X")
            else:
                n_pass += 1
                sys.stdout.write(".")
        sys.stdout.flush()

        if idx % 80 == 0:
            sys.stdout.write(f" {idx}/{len(cases)}\n")
            sys.stdout.flush()

        if not args.keep_asm and not diffs:
            for p in (asm_path, bin_path, bin_path.with_suffix(".o"), bin_path.with_suffix(".elf")):
                try:
                    p.unlink()
                except FileNotFoundError:
                    pass

    sys.stdout.write("\n")
    print(f"regstate: PASS={n_pass} XFAIL={n_xfail} XPASS={n_xpass} MISMATCH={n_mismatch} TIMEOUT={n_timeout} ERROR={n_error} selected={len(cases)}")
    for name, diffs in details:
        print(f"  {name}:")
        mem_summary = _mem_diff_summary(diffs)
        if mem_summary:
            print(f"    {mem_summary}")
        for diff in diffs[:12]:
            print(f"    {diff}")
        if len(diffs) > 12:
            print(f"    ... {len(diffs) - 12} more")
    return 0 if n_mismatch == 0 and n_timeout == 0 and n_error == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
