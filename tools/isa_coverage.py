#!/usr/bin/env python3
"""isa_coverage.py — Parse Verilator sim logs and report per-opcode coverage.

Usage:
    python3 tools/isa_coverage.py --logdir build/logs

The simulation is expected to emit lines like:
    [COV] OPCODE=4e75 COUNT=1234
when compiled with +define+COVERAGE.

Alternatively, this script can parse a disassembly listing and cross-check
against the ISA status table in docs/isa_status.md.
"""

import argparse
import os
import re
import sys
from pathlib import Path
from collections import defaultdict
from typing import Optional, Tuple

# ──────────────────────────────────────────────────────────────────────────────
# 68040 opcode group map (first nibble of opword → group name)
# ──────────────────────────────────────────────────────────────────────────────
OPCODE_GROUPS = {
    0x0: "Bit/Immediate",
    0x1: "MOVE.B",
    0x2: "MOVE.L / MOVEA.L",
    0x3: "MOVE.W / MOVEA.W",
    0x4: "Misc (LEA/NEG/CLR/JSR/JMP/MOVEM...)",
    0x5: "ADDQ/SUBQ/Scc/DBcc",
    0x6: "Bcc/BRA/BSR",
    0x7: "MOVEQ",
    0x8: "DIVU/DIVS/OR",
    0x9: "SUB/SUBA/SUBX",
    0xA: "A-line (Mac OS traps)",
    0xB: "CMP/EOR",
    0xC: "AND/MULS/MULU/EXG",
    0xD: "ADD/ADDA/ADDX",
    0xE: "Shift/Rotate",
    0xF: "FPU/MMU/CPU32",
}

def parse_coverage_log(logdir: Path) -> dict:
    """Parse [COV] lines from sim log files."""
    counts = defaultdict(int)
    for logfile in logdir.glob("*.log"):
        with open(logfile) as f:
            for line in f:
                m = re.match(r'\[COV\] OPCODE=([0-9a-fA-F]{4}) COUNT=(\d+)', line)
                if m:
                    opcode = int(m.group(1), 16)
                    count  = int(m.group(2))
                    counts[opcode] += count
    return counts

def print_coverage_report(counts: dict):
    """Print a coverage report grouped by opcode family."""
    print("=" * 60)
    print("m68k-ooo ISA Coverage Report")
    print("=" * 60)

    total_insts = sum(counts.values())
    print(f"Total instructions executed: {total_insts:,}")
    print()

    # Group by first nibble
    group_counts = defaultdict(int)
    for opcode, count in counts.items():
        group_counts[(opcode >> 12) & 0xF] += count

    print(f"{'Group':<6} {'Name':<42} {'Count':>10} {'%':>6}")
    print("-" * 68)
    for nibble in range(16):
        name  = OPCODE_GROUPS.get(nibble, f"Group {nibble:X}")
        count = group_counts.get(nibble, 0)
        pct   = (count / total_insts * 100) if total_insts > 0 else 0
        bar   = "█" * int(pct / 2)
        print(f"  {nibble:X}    {name:<42} {count:>10,} {pct:>5.1f}%  {bar}")

    print()
    # Opcodes never seen
    unseen_groups = [n for n in range(16) if group_counts.get(n, 0) == 0]
    if unseen_groups:
        print(f"Never executed: groups {', '.join(hex(n) for n in unseen_groups)}")
        if 0xA in unseen_groups:
            print("  ⚠ A-line traps never executed — Mac OS A-trap compatibility untested!")
        if 0xF in unseen_groups:
            print("  ⚠ FPU/MMU instructions never executed")
    print()

WIDTHED_MNEMONICS = {
    "move", "add", "sub", "and", "or", "eor", "cmp", "cmpi", "tst",
    "clr", "not", "neg", "negx",
}

FILENAME_SIZE_ALIASES = {
    "b": "B",
    "byte": "B",
    "w": "W",
    "word": "W",
    "l": "L",
    "long": "L",
}

PREFIX_SIZE_ALIASES = {
    "moveb": ("move", "B"),
    "movew": ("move", "W"),
    "movel": ("move", "L"),
}

def strip_asm_comment(line: str) -> str:
    """Return the instruction portion before the m68k asm comment marker."""
    return line.split("|", 1)[0]

def normalize_width_stem(stem: str) -> Tuple[str, Optional[str]]:
    """Collapse filename size spellings so sibling tests cluster together."""
    parts = stem.split("_")
    width = None

    if parts[0] in PREFIX_SIZE_ALIASES:
        base, width = PREFIX_SIZE_ALIASES[parts[0]]
        parts[0] = base

    normalized = []
    for part in parts:
        if part in FILENAME_SIZE_ALIASES:
            width = FILENAME_SIZE_ALIASES[part]
            continue
        normalized.append(part)

    return "_".join(normalized), width

def scan_asm_widths(asm_dir: Path) -> Tuple[dict, dict]:
    """Scan directed asm tests for explicit B/W/L test coverage."""
    filename_clusters = defaultdict(set)
    mnemonic_widths = defaultdict(set)

    for asm in sorted(asm_dir.glob("*.s")):
        cluster, filename_width = normalize_width_stem(asm.stem)
        if filename_width:
            filename_clusters[cluster].add(filename_width)

        text = asm.read_text(errors="replace")
        for line in text.splitlines():
            code = strip_asm_comment(line).lower()
            for mnemonic, width in re.findall(r"\b([a-z][a-z0-9]*)\s*\.\s*([bwl])\b", code):
                if mnemonic in WIDTHED_MNEMONICS:
                    mnemonic_widths[mnemonic].add(width.upper())

    return filename_clusters, mnemonic_widths

def print_asm_width_audit(asm_dir: Path):
    if not asm_dir.exists():
        print(f"ASM directory not found: {asm_dir}", file=sys.stderr)
        sys.exit(1)

    filename_clusters, mnemonic_widths = scan_asm_widths(asm_dir)
    all_widths = {"B", "W", "L"}

    print("Directed ASM width audit")
    print(f"ASM directory: {asm_dir}")
    print()

    print("Mnemonic families:")
    for mnemonic in sorted(WIDTHED_MNEMONICS):
        widths = mnemonic_widths.get(mnemonic, set())
        missing = sorted(all_widths - widths)
        seen = "".join(sorted(widths)) or "-"
        if missing:
            print(f"  {mnemonic:<5} seen={seen:<3} missing={''.join(missing)}")
        else:
            print(f"  {mnemonic:<5} seen={seen:<3} missing=-")

    print()
    print("Filename sibling clusters with missing widths:")
    any_missing = False
    for cluster, widths in sorted(filename_clusters.items()):
        if not widths or widths == all_widths:
            continue
        missing = sorted(all_widths - widths)
        any_missing = True
        print(f"  {cluster:<32} seen={''.join(sorted(widths)):<3} missing={''.join(missing)}")

    if not any_missing:
        print("  none")

def main():
    parser = argparse.ArgumentParser(description='m68k-ooo ISA coverage report')
    parser.add_argument('--logdir', type=Path, default=Path('build/logs'),
                        help='Directory containing simulation log files')
    parser.add_argument('--asm-width-audit', type=Path,
                        help='Scan tb/tests/asm for B/W/L directed-test gaps')
    args = parser.parse_args()

    if args.asm_width_audit:
        print_asm_width_audit(args.asm_width_audit)
        return

    if not args.logdir.exists():
        print(f"Log directory not found: {args.logdir}")
        print("Run 'make test' first to generate coverage data.")
        sys.exit(1)

    counts = parse_coverage_log(args.logdir)
    if not counts:
        print("No coverage data found. Recompile with +define+COVERAGE and re-run tests.")
        sys.exit(0)

    print_coverage_report(counts)

if __name__ == '__main__':
    main()
