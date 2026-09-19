#!/usr/bin/env python3
"""Log repeated MAME PC hits with SP/SR/register state.

This is intentionally breakpoint-based (not a trace stream): it installs
debugger breakpoints at a small set of ROM PCs, logs the architectural state
on each hit, then resumes until the hit limit or time limit is reached.
"""

import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile


def parse_pc(text: str) -> int:
    value = int(text, 0)
    if not 0 <= value <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError(f"PC out of range: {text!r}")
    return value


def safe_mame_path(path: Path) -> str:
    text = str(path)
    if re.search(r"[\s,{};\n\r]", text):
        raise SystemExit(f"unsafe debugger path: {text!r}")
    return text


def safe_condition(text: str) -> str:
    if not text or re.search(r"[\n\r{},;]", text):
        raise argparse.ArgumentTypeError(
            f"unsafe MAME debugger condition: {text!r}")
    return text


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pc", action="append", type=parse_pc, required=True,
                    help="PC to breakpoint; may be repeated")
    ap.add_argument("--out", required=True,
                    help="output log path")
    ap.add_argument("--seconds", type=int, default=20)
    ap.add_argument("--video", default="none",
                    help="MAME video backend (debugger builds commonly need bgfx)")
    ap.add_argument("--cond", type=safe_condition, default="1",
                    help="MAME debugger breakpoint condition")
    ap.add_argument("--rompath", default="/tmp/mame_rompath")
    ap.add_argument("--hard", help="hard-disk image to attach")
    ap.add_argument("--ramsize", help="MAME RAM size, for example 8M")
    ap.add_argument("--nvram-directory",
                    help="isolated MAME NVRAM directory")
    ap.add_argument("--dasm-out",
                    help="optional disassembly file written at each hit")
    ap.add_argument("--dasm-address", type=parse_pc,
                    help="start address for --dasm-out")
    ap.add_argument("--dasm-length", type=parse_pc, default=0x100,
                    help="byte length for --dasm-out (default: 0x100)")
    args = ap.parse_args()

    if bool(args.dasm_out) != (args.dasm_address is not None):
        ap.error("--dasm-out and --dasm-address must be used together")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    mame_out = safe_mame_path(out)
    dasm_out = safe_mame_path(Path(args.dasm_out)) if args.dasm_out else None

    commands = []
    fmt = (
        'HIT PC=%08X SR=%04X SP=%08X '
        'D0=%08X D1=%08X D2=%08X D3=%08X D4=%08X D5=%08X D6=%08X D7=%08X '
        'A0=%08X A1=%08X A2=%08X A3=%08X A4=%08X A5=%08X A6=%08X A7=%08X '
        'STK0=%08X STK4=%08X\\n'
    )
    for pc in args.pc:
        action = (
            f'trace {mame_out},0,noloop;'
            f'tracelog "{fmt}",pc,sr,sp,d0,d1,d2,d3,d4,d5,d6,d7,'
            'a0,a1,a2,a3,a4,a5,a6,a7,'
            'd@(a7),d@(a7+4);'
            'trace off;'
        )
        if dasm_out:
            action += (
                f'dasm {dasm_out},0x{args.dasm_address:X},'
                f'0x{args.dasm_length:X};'
            )
        commands.append(
            f'bpset 0x{pc:X},{args.cond},'
            '{'
            f'{action}'
            'g'
            '}'
        )
    commands.append("g")
    script = "\n".join(commands) + "\n"

    with tempfile.NamedTemporaryFile(mode="w", suffix=".dbg", delete=False) as f:
        f.write(script)
        script_path = f.name

    env = os.environ.copy()
    if args.video == "none":
        env["QT_QPA_PLATFORM"] = "offscreen"
    cmd = [
        "mame",
        "-rompath", args.rompath,
        "-video", args.video,
        "-sound", "none",
        "-nothrottle",
        "-debug",
        "-debugscript", script_path,
        "-seconds_to_run", str(args.seconds),
        "macqd700",
    ]
    if args.hard:
        cmd.extend(["-hard", args.hard])
    if args.ramsize:
        cmd.extend(["-ramsize", args.ramsize])
    if args.nvram_directory:
        cmd.extend(["-nvram_directory", args.nvram_directory])
    print(f"[mame-pc-sp-log] script={script_path}")
    print(f"[mame-pc-sp-log] out={out}")
    print(f"[mame-pc-sp-log] pcs={','.join(f'0x{pc:08x}' for pc in args.pc)}")
    out.write_text("")
    return subprocess.call(cmd, env=env, stdin=subprocess.DEVNULL)


if __name__ == "__main__":
    raise SystemExit(main())
