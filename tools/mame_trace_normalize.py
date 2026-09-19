#!/usr/bin/env python3
"""mame_trace_normalize.py — convert a MAME macqd700 CPU trace to the
format emitted by tb_rom_boot.cpp: `<pc> <ir> <ccr>` one per line.

Task #101 bring-up tooling.  See docs/mame_integration.md §6 and
docs/rom_boot_bringup.md for how to capture the source trace.

Recognised MAME input shapes
────────────────────────────

1) Debugger `trace` default format — each line starts with the PC in
   hex, followed by whitespace and disassembly:

       00000028: 4efa 0060                jmp     $8a(pc)
       0000008a: 2b7c 0000 0004 0120      move.l  #$4,$120.w

   The ir field is the first 4 hex digits of the second column; ccr
   is unknown from this form (emitted as 00).

2) Debugger `trace` with scripted `tracelog` extras — see §9 cheat:

       trace cpu.tr,,,{tracelog "PC=%08X SR=%04X ", pc, sr}

   Produces lines like:

       PC=00000028 SR=2700  00000028: 4efa 0060    jmp $8a(pc)

   We parse PC= and SR= prefixes when present.  CCR = low 5 bits of SR
   (X,N,Z,V,C).

3) MAME 0.264 packaged debugger output with a `tracelog` prefix but
   no opcode bytes:

       PC=0000008C SR=2704 0000008C: move    #$2700, SR

   Use `--allow-missing-ir` for this form.  The emitted IR field is
   `0000`, so compare it with `tools/rom_trace_diff.py --pc-only`.

4) Hand-rolled format matching our own dump, passed through untouched.

The tool is tolerant of blank lines, debugger banners, and watchpoint
hits interleaved with the CPU trace.  Anything it can't parse as a
trace line is dropped silently (use --strict to fail instead).

Usage
─────

    ./tools/mame_trace_normalize.py input.tr -o normalized.tr
    ./tools/mame_trace_normalize.py input.tr           # stdout

With --limit N, only the first N parsed lines are emitted.
"""

from __future__ import annotations

import argparse
import re
import sys
from typing import Optional

# Match `PC=HHHHHHHH`, `SR=HHHH`, or `IR=HHHH` prefixes (MAME tracelog scripted
# form).  IR= is accepted for future debugger scripts that explicitly log the
# instruction word.
PC_PREFIX_RE = re.compile(r"PC=([0-9a-fA-F]{4,8})", re.ASCII)
SR_PREFIX_RE = re.compile(r"SR=([0-9a-fA-F]{1,4})", re.ASCII)
IR_PREFIX_RE = re.compile(r"IR=([0-9a-fA-F]{4})", re.ASCII)
# MAME debug `trace` line, possibly after tracelog prefixes.  Some packaged
# MAME builds omit opcode bytes when an action is present; group 2 is optional.
TRACE_PC_RE = re.compile(
    r"\b([0-9a-fA-F]{4,8})\s*:\s*(?:([0-9a-fA-F]{4})\b)?",
    re.ASCII,
)


def parse_line(ln: str, *, allow_missing_ir: bool = False) -> Optional[tuple[int, int, int]]:
    ln_stripped = ln.rstrip("\n")
    pc: Optional[int] = None
    ir: Optional[int] = None
    ccr = 0
    m_pc = PC_PREFIX_RE.search(ln_stripped)
    if m_pc:
        pc = int(m_pc.group(1), 16)
    m_sr = SR_PREFIX_RE.search(ln_stripped)
    if m_sr:
        sr = int(m_sr.group(1), 16)
        ccr = sr & 0x1F
    m_ir = IR_PREFIX_RE.search(ln_stripped)
    if m_ir:
        ir = int(m_ir.group(1), 16)

    # Trace line pattern (even if PC= was also present, prefer the explicit
    # prefix, but grab the IR from the trace line when present).
    m_def = TRACE_PC_RE.search(ln_stripped)
    if m_def:
        if pc is None:
            pc = int(m_def.group(1), 16)
        if ir is None and m_def.group(2) is not None:
            ir = int(m_def.group(2), 16)
    if pc is not None:
        if ir is None:
            if not allow_missing_ir:
                return None
            ir = 0
        return (pc, ir, ccr)
    return None


def _main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help="MAME trace file (or - for stdin)")
    ap.add_argument("-o", "--output", default="-",
                    help="output path (default stdout)")
    ap.add_argument("--limit", type=int, default=0,
                    help="emit at most N lines (0 = all)")
    ap.add_argument("--allow-missing-ir", action="store_true",
                    help="emit IR=0000 for MAME trace lines that only carry PC/SR")
    ap.add_argument("--strict", action="store_true",
                    help="fail on unparseable non-blank lines")
    args = ap.parse_args(argv)

    if args.input == "-":
        fin = sys.stdin
    else:
        fin = open(args.input, "r", encoding="utf-8", errors="replace")
    if args.output == "-":
        fout = sys.stdout
    else:
        fout = open(args.output, "w", encoding="utf-8")

    emitted = 0
    parsed = 0
    dropped = 0
    try:
        fout.write("# MAME macqd700 cold-boot trace, normalised\n")
        fout.write(f"# source: {args.input}\n")
        for ln in fin:
            if not ln.strip():
                continue
            if ln.lstrip().startswith("#"):
                continue
            res = parse_line(ln, allow_missing_ir=args.allow_missing_ir)
            if res is None:
                dropped += 1
                if args.strict:
                    sys.stderr.write(f"[strict] unparseable: {ln!r}\n")
                    return 2
                continue
            pc, ir, ccr = res
            fout.write(f"{pc:08x} {ir:04x} {ccr:02x}\n")
            parsed += 1
            emitted += 1
            if args.limit and emitted >= args.limit:
                break
    finally:
        if fin is not sys.stdin:
            fin.close()
        if fout is not sys.stdout:
            fout.close()

    sys.stderr.write(
        f"[mame_trace_normalize] emitted={emitted} "
        f"parsed={parsed} dropped={dropped}\n"
    )
    return 0


def main_args_for_test(argv: list[str]) -> int:
    return _main(argv)


def main() -> int:
    return _main()


if __name__ == "__main__":
    sys.exit(main())
