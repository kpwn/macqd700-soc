#!/usr/bin/env python3
"""Generate a MAME debugger MMIO trace scaffold for Q700 peripheral windows.

This is an observational helper, not live co-simulation.  It checks the local
MAME binary, writes a debugger script with watchpoints for the windows we want
to bridge later, and emits a shell command for a short headless trace run.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


WINDOWS = [
    ("via1", 0x50000000, 0x00002000),
    ("via2", 0x50002000, 0x00002000),
    ("scc", 0x5000C000, 0x00002000),
    ("scsi_regs", 0x5000F000, 0x00000100),
    ("scsi_dma", 0x5000F100, 0x00000002),
    ("asc", 0x50014000, 0x00002000),
    ("swim_iwm", 0x5001E000, 0x00002000),
    ("dafb_regs", 0xF9800000, 0x00000400),
    ("vram_pixels", 0xF9000000, 0x00100000),
]


def run_text(argv: list[str]) -> str:
    proc = subprocess.run(
        argv,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return proc.stdout.strip()


def find_mame(explicit: str | None) -> str:
    if explicit:
        return explicit
    found = shutil.which("mame") or shutil.which("mame64") or "/usr/games/mame"
    if Path(found).exists() or shutil.which(found):
        return found
    raise SystemExit("could not find mame; pass --mame")


def write_debug_script(path: Path, trace_path: Path, seconds: float) -> None:
    lines = [
        f'trace {trace_path},,noloop,{{tracelog "PC=%08X SR=%04X D0=%08X A0=%08X A7=%08X\\n", pc, sr, d0, a0, a7}}',
    ]
    for name, base, size in WINDOWS:
        for mode, label in (("r", "R"), ("w", "W")):
            lines.append(
                "wpset "
                f"0x{base:08x},0x{size:08x},{mode},1,"
                "{"
                f'tracelog "MMIO {name} {label} %08X data=%08X pc=%08X\\n", '
                "wpaddr, wpdata, pc; g"
                "}"
            )
    # The command-line -seconds_to_run is the real limiter; this comment keeps
    # the script self-describing when copied into an interactive debugger.
    lines.append(f"# command line should include -seconds_to_run {seconds:g}")
    lines.append("g")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_runner(
    path: Path,
    mame: str,
    rompath: Path,
    nvram_dir: Path,
    debug_script: Path,
    seconds: float,
) -> None:
    cmd = [
        mame,
        "macqd700",
        "-rompath",
        str(rompath),
        "-nvram_directory",
        str(nvram_dir),
        "-debug",
        "-debugger",
        "qt",
        "-debugscript",
        str(debug_script),
        "-debuglog",
        "-video",
        "none",
        "-sound",
        "none",
        "-nothrottle",
        "-seconds_to_run",
        f"{seconds:g}",
    ]
    quoted = " ".join("'" + part.replace("'", "'\\''") + "'" for part in cmd)
    path.write_text(
        "#!/bin/sh\nset -eu\nexport QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-offscreen}\n"
        + quoted
        + "\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mame", help="MAME binary path")
    ap.add_argument("--out-dir", type=Path, default=Path("build/mame_mmio_trace"))
    ap.add_argument("--rompath", type=Path, default=Path("roms"))
    ap.add_argument("--seconds", type=float, default=0.2)
    args = ap.parse_args()

    mame = find_mame(args.mame)
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    nvram_dir = out_dir / "nvram"
    nvram_dir.mkdir(parents=True, exist_ok=True)

    version = run_text([mame, "-version"])
    media = run_text([mame, "macqd700", "-listmedia"])
    debug_script = out_dir / "q700_mmio_trace.dbg"
    trace_path = out_dir / "q700_mmio_trace.tr"
    runner = out_dir / "run_mame_mmio_trace.sh"

    write_debug_script(debug_script, trace_path, args.seconds)
    write_runner(runner, mame, args.rompath, nvram_dir, debug_script, args.seconds)

    print(f"mame={mame}")
    print(f"version={version}")
    print(f"debug_script={debug_script}")
    print(f"runner={runner}")
    print(f"trace={trace_path}")
    print("media:")
    print(media)
    print()
    print("Next command:")
    print(str(runner))
    print()
    print("Note: this only logs MAME MMIO; it does not return RTL read data to MAME.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
