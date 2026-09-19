#!/usr/bin/env python3
"""Run bounded RTL and Musashi Q700 ROM slices and compare their frontiers.

This orchestrates:

- `make tb-rom-boot` for the RTL slice
- `make musashi-rom-boot` for the Musashi slice
- `tools/rom_boot_stop_summary.py` for the RTL last-N summary
- `tools/rom_trace_diff.py` for the committed-instruction trace diff

All artifacts are written under a single run directory rooted in
`/dev/shm/m68k` by default.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence


DEFAULT_ROOT = Path("/dev/shm/m68k/rom_frontier_diff")
DEFAULT_ROM = Path("files/420dbff3.rom")
DEFAULT_STOP_PC = 0x408005B0
DEFAULT_MAX_INSTS = 2048
DEFAULT_LASTN_CYCLES = 256
DEFAULT_PERIPH_LIMIT = 128
DEFAULT_PERIPH_FILTER = "VIA1,VIA2,ADB,VBL,RTC,PRAM,ASC,SCSI,SCC,DAFB,VRAM"
DEFAULT_RTL_STOP_ON_EXC = "2,4,11"
DEFAULT_TRACE_CONTEXT = 16
DEFAULT_SAMPLE_EVERY = 0

RTL_PERIPH_BLOCK_START = "-------- peripheral model event summary --------"
RTL_PERIPH_BLOCK_END = "-----------------------------------------------"
MUSASHI_BUS_BLOCK_START = "-------- musashi-rom-boot bus activity --------"
MUSASHI_BUS_BLOCK_END = "-----------------------------------------------"

MUSASHI_STOP_RE = re.compile(
    r"^\[musashi-rom-boot\] stop reason=(?P<reason>.+?) committed=(?P<committed>\d+) "
    r"cycles=(?P<cycles>\d+) pc=0x(?P<pc>[0-9a-fA-F]+) sr=0x(?P<sr>[0-9a-fA-F]+)$"
)


@dataclass(frozen=True)
class RunPaths:
    root: Path
    rtl: Path
    musashi: Path
    compare: Path


@dataclass(frozen=True)
class MusashiSummary:
    stop_line: str
    reason: str
    committed: int
    cycles: int
    pc: int
    sr: int
    bus_block: list[str]


def fmt_pc(value: int) -> str:
    return f"0x{value:08x}"


def make_env() -> dict[str, str]:
    env = os.environ.copy()
    env["VERILATOR_THREADS"] = "4"
    env["VERILATOR_JOBS"] = "4"
    env["MAKEFLAGS"] = "-j1"
    return env


def timestamp_run_name() -> str:
    return datetime.utcnow().strftime("%Y%m%d-%H%M%S")


def build_run_paths(root: Path, run_name: str) -> RunPaths:
    run_root = root / run_name
    return RunPaths(
        root=run_root,
        rtl=run_root / "rtl",
        musashi=run_root / "musashi",
        compare=run_root / "compare",
    )


def ensure_run_dirs(paths: RunPaths) -> None:
    paths.rtl.mkdir(parents=True, exist_ok=True)
    paths.musashi.mkdir(parents=True, exist_ok=True)
    paths.compare.mkdir(parents=True, exist_ok=True)


def build_rtl_make_args(
    *,
    rom: Path,
    rtl_root: Path,
    stop_pc: int,
    max_insts: int,
    lastn_cycles: int,
    periph_limit: int,
    periph_filter: str,
    stop_on_exc: str,
    sample_every: int,
) -> list[str]:
    extra_parts = [
        f"+end_pc={fmt_pc(stop_pc)}",
        "+end_pc_hit=1",
        f"+stop_on_exc={stop_on_exc}",
        f"+max_insts={max_insts}",
        f"+lastn_trace={lastn_cycles}",
        f"+lastn_trace_path={rtl_root / 'rom_boot_frontier_lastn.log'}",
        f"+periph_event_log={rtl_root / 'rom_boot_frontier_periph.log'}",
        f"+periph_event_filter={periph_filter}",
        f"+periph_event_log_limit={periph_limit}",
        "+no_waves",
    ]
    if sample_every > 0:
        extra_parts.extend(
            [
                f"+arch_sample_every={sample_every}",
                f"+arch_sample_log={rtl_root / 'rtl_arch_sample.tsv'}",
            ]
        )
    extra = " ".join(extra_parts)
    return [
        "make",
        "tb-rom-boot",
        f"ROM={rom}",
        f"ROMBOOT_OUTPUT_ROOT={rtl_root}",
        f"ROMBOOT_EXTRA={extra}",
    ]


def build_musashi_make_args(
    *,
    rom: Path,
    musashi_root: Path,
    stop_pc: int,
    max_insts: int,
    periph_limit: int,
    sample_every: int,
) -> list[str]:
    extra_parts = [
        "--stop-pc",
        fmt_pc(stop_pc),
        "--stop-pc-hit",
        "1",
        "--allow-unshared-io",
        "--periph-log-limit",
        str(periph_limit),
    ]
    if sample_every > 0:
        extra_parts.extend(
            [
                "--sample-every",
                str(sample_every),
                "--sample-log",
                str(musashi_root / "musashi_rom_boot_sample.tsv"),
            ]
        )
    extra = " ".join(extra_parts)
    return [
        "make",
        "musashi-rom-boot",
        f"ROM={rom}",
        f"ROMBOOT_OUTPUT_ROOT={musashi_root}",
        f"MUSASHI_ROM_BOOT_MAX={max_insts}",
        f"MUSASHI_ROM_BOOT_ARGS={extra}",
    ]


def run_command(argv: Sequence[str], *, cwd: Path, env: dict[str, str], log_path: Path) -> int:
    with log_path.open("w", encoding="utf-8") as log_fp:
        proc = subprocess.run(
            list(argv),
            cwd=str(cwd),
            env=env,
            stdout=log_fp,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    return proc.returncode


def extract_block(lines: Iterable[str], start: str, end: str) -> list[str]:
    block: list[str] = []
    capture = False
    for line in lines:
        text = line.rstrip("\n")
        if text == start:
            block = [text]
            capture = True
            continue
        if capture:
            block.append(text)
            if text == end:
                return block
    return block


def read_lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8", errors="replace").splitlines()


def parse_musashi_summary(log_path: Path) -> MusashiSummary:
    lines = read_lines(log_path)
    stop_line = ""
    bus_block: list[str] = []
    for line in lines:
        if line.startswith("[musashi-rom-boot] stop reason="):
            stop_line = line
        if line == MUSASHI_BUS_BLOCK_START:
            bus_block = extract_block(lines, MUSASHI_BUS_BLOCK_START, MUSASHI_BUS_BLOCK_END)
    if not stop_line:
        raise ValueError(f"missing Musashi stop line in {log_path}")

    m = MUSASHI_STOP_RE.match(stop_line)
    if not m:
        raise ValueError(f"unparseable Musashi stop line: {stop_line}")

    return MusashiSummary(
        stop_line=stop_line,
        reason=m.group("reason"),
        committed=int(m.group("committed")),
        cycles=int(m.group("cycles")),
        pc=int(m.group("pc"), 16),
        sr=int(m.group("sr"), 16),
        bus_block=bus_block,
    )


def summarize_musashi(summary: MusashiSummary) -> str:
    return (
        f"reason={summary.reason} committed={summary.committed} cycles={summary.cycles} "
        f"pc={fmt_pc(summary.pc)} sr=0x{summary.sr:04x}"
    )


def last_nonempty_line(path: Path) -> str:
    for line in reversed(read_lines(path)):
        if line.strip():
            return line
    return ""


def extract_required_block(path: Path, start: str, end: str) -> list[str]:
    block = extract_block(read_lines(path), start, end)
    if not block:
        raise ValueError(f"missing block {start!r} in {path}")
    return block


def rtl_stop_summary_command(repo_root: Path, rtl_root: Path) -> list[str]:
    return [
        sys.executable,
        str(repo_root / "tools" / "rom_boot_stop_summary.py"),
        str(rtl_root / "rom_boot_frontier_lastn.log"),
        "--log",
        str(rtl_root / "run.log"),
        "--trace",
        str(rtl_root / "rom_boot_trace.log"),
        "--compact",
    ]


def trace_diff_command(
    repo_root: Path,
    ours: Path,
    theirs: Path,
    *,
    context: int,
    pc_only: bool = False,
) -> list[str]:
    cmd = [
        sys.executable,
        str(repo_root / "tools" / "rom_trace_diff.py"),
        str(ours),
        str(theirs),
        "--context",
        str(context),
        "--min-match",
        "1",
    ]
    if pc_only:
        cmd.append("--pc-only")
    return cmd


def arch_sample_compare_command(repo_root: Path, rtl_root: Path, musashi_root: Path) -> list[str]:
    return [
        sys.executable,
        str(repo_root / "tools" / "rom_arch_sample_compare.py"),
        str(rtl_root / "rtl_arch_sample.tsv"),
        str(musashi_root / "musashi_rom_boot_sample.tsv"),
    ]


def write_report(
    *,
    report_path: Path,
    paths: RunPaths,
    rtl_summary_path: Path,
    rtl_summary_line: str,
    rtl_periph_path: Path,
    musashi_summary_path: Path,
    musashi_summary_line: str,
    musashi_summary: MusashiSummary,
    musashi_periph_path: Path,
    trace_diff_path: Path,
    trace_diff_rc: int,
    sample_compare_path: Path | None,
    sample_compare_rc: int | None,
    mame_trace_path: Path | None,
    rtl_mame_diff_path: Path | None,
    rtl_mame_diff_rc: int | None,
    musashi_mame_diff_path: Path | None,
    musashi_mame_diff_rc: int | None,
    rtl_rc: int,
    musashi_rc: int,
) -> None:
    lines = [
        f"[rom-frontier-compare] root={paths.root}",
        f"[rom-frontier-compare] rtl-run-log={paths.rtl / 'run.log'}",
        f"[rom-frontier-compare] rtl-trace={paths.rtl / 'rom_boot_trace.log'}",
        f"[rom-frontier-compare] rtl-lastn={paths.rtl / 'rom_boot_frontier_lastn.log'}",
        f"[rom-frontier-compare] rtl-periph={paths.rtl / 'rom_boot_frontier_periph.log'}",
        f"[rom-frontier-compare] rtl-summary={rtl_summary_path}",
        f"[rom-frontier-compare] rtl-summary-line={rtl_summary_line}",
        f"[rom-frontier-compare] rtl-periph-summary={rtl_periph_path}",
        f"[rom-frontier-compare] musashi-run-log={paths.musashi / 'run.log'}",
        f"[rom-frontier-compare] musashi-trace={paths.musashi / 'musashi_rom_boot_trace.log'}",
        f"[rom-frontier-compare] musashi-periph={paths.musashi / 'musashi_rom_boot_periph.log'}",
        f"[rom-frontier-compare] musashi-summary={musashi_summary_path}",
        f"[rom-frontier-compare] musashi-summary-line={musashi_summary_line}",
        f"[rom-frontier-compare] musashi-periph-summary={musashi_periph_path}",
        f"[rom-frontier-compare] trace-diff-rc={trace_diff_rc}",
        f"[rom-frontier-compare] rtl-rc={rtl_rc} musashi-rc={musashi_rc}",
        f"[rom-frontier-compare] trace-diff={trace_diff_path}",
    ]
    if mame_trace_path is not None:
        lines.extend(
            [
                f"[rom-frontier-compare] mame-trace={mame_trace_path}",
                f"[rom-frontier-compare] rtl-mame-diff-rc={rtl_mame_diff_rc}",
                f"[rom-frontier-compare] rtl-mame-diff={rtl_mame_diff_path}",
                f"[rom-frontier-compare] musashi-mame-diff-rc={musashi_mame_diff_rc}",
                f"[rom-frontier-compare] musashi-mame-diff={musashi_mame_diff_path}",
            ]
        )
    if sample_compare_path is not None:
        lines.extend(
            [
                f"[rom-frontier-compare] rtl-sample={paths.rtl / 'rtl_arch_sample.tsv'}",
                f"[rom-frontier-compare] musashi-sample={paths.musashi / 'musashi_rom_boot_sample.tsv'}",
                f"[rom-frontier-compare] sample-compare-rc={sample_compare_rc}",
                f"[rom-frontier-compare] sample-compare={sample_compare_path}",
            ]
        )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument("--rom", type=Path, default=DEFAULT_ROM)
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    ap.add_argument("--run-name", default="")
    ap.add_argument("--stop-pc", type=lambda s: int(s, 0), default=DEFAULT_STOP_PC)
    ap.add_argument("--max-insts", type=int, default=DEFAULT_MAX_INSTS)
    ap.add_argument("--lastn-cycles", type=int, default=DEFAULT_LASTN_CYCLES)
    ap.add_argument("--periph-limit", type=int, default=DEFAULT_PERIPH_LIMIT)
    ap.add_argument("--periph-filter", default=DEFAULT_PERIPH_FILTER)
    ap.add_argument("--rtl-stop-on-exc", default=DEFAULT_RTL_STOP_ON_EXC)
    ap.add_argument("--trace-context", type=int, default=DEFAULT_TRACE_CONTEXT)
    ap.add_argument("--sample-every", type=int, default=DEFAULT_SAMPLE_EVERY)
    ap.add_argument("--mame-trace", type=Path,
                    help="optional normalized MAME trace to compare against RTL and Musashi")
    ap.add_argument("--mame-pc-only", action="store_true",
                    help="use PC-only trace diffs for optional MAME comparisons")
    args = ap.parse_args()

    run_name = args.run_name or timestamp_run_name()
    paths = build_run_paths(args.root, run_name)
    ensure_run_dirs(paths)

    env = make_env()
    rtl_cmd = build_rtl_make_args(
        rom=args.rom,
        rtl_root=paths.rtl,
        stop_pc=args.stop_pc,
        max_insts=args.max_insts,
        lastn_cycles=args.lastn_cycles,
        periph_limit=args.periph_limit,
        periph_filter=args.periph_filter,
        stop_on_exc=args.rtl_stop_on_exc,
        sample_every=args.sample_every,
    )
    musashi_cmd = build_musashi_make_args(
        rom=args.rom,
        musashi_root=paths.musashi,
        stop_pc=args.stop_pc,
        max_insts=args.max_insts,
        periph_limit=args.periph_limit,
        sample_every=args.sample_every,
    )

    rtl_rc = run_command(rtl_cmd, cwd=args.repo_root, env=env, log_path=paths.rtl / "run.log")
    musashi_rc = run_command(
        musashi_cmd,
        cwd=args.repo_root,
        env=env,
        log_path=paths.musashi / "run.log",
    )

    rtl_summary_path = paths.compare / "rtl_stop_summary.txt"
    with rtl_summary_path.open("w", encoding="utf-8") as rtl_summary_fp:
        rtl_summary_rc = subprocess.run(
            rtl_stop_summary_command(args.repo_root, paths.rtl),
            cwd=str(args.repo_root),
            env=env,
            stdout=rtl_summary_fp,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        ).returncode
    rtl_summary_line = last_nonempty_line(rtl_summary_path)

    rtl_periph_path = paths.compare / "rtl_periph_summary.txt"
    rtl_periph_path.write_text(
        "\n".join(
            extract_required_block(
                paths.rtl / "run.log",
                RTL_PERIPH_BLOCK_START,
                RTL_PERIPH_BLOCK_END,
            )
        )
        + "\n",
        encoding="utf-8",
    )

    musashi_summary = parse_musashi_summary(paths.musashi / "run.log")
    musashi_summary_path = paths.compare / "musashi_stop_summary.txt"
    musashi_summary_path.write_text(
        summarize_musashi(musashi_summary) + "\n" + "\n".join(musashi_summary.bus_block) + "\n",
        encoding="utf-8",
    )
    musashi_summary_line = summarize_musashi(musashi_summary)
    musashi_periph_path = paths.compare / "musashi_periph_summary.txt"
    musashi_periph_path.write_text(
        "\n".join(
            extract_required_block(
                paths.musashi / "run.log",
                MUSASHI_BUS_BLOCK_START,
                MUSASHI_BUS_BLOCK_END,
            )
        )
        + "\n",
        encoding="utf-8",
    )

    trace_diff_path = paths.compare / "trace_diff.txt"
    with trace_diff_path.open("w", encoding="utf-8") as trace_diff_fp:
        trace_diff_rc = subprocess.run(
            trace_diff_command(
                args.repo_root,
                paths.rtl / "rom_boot_trace.log",
                paths.musashi / "musashi_rom_boot_trace.log",
                context=args.trace_context,
            ),
            cwd=str(args.repo_root),
            env=env,
            stdout=trace_diff_fp,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        ).returncode

    rtl_mame_diff_path = None
    rtl_mame_diff_rc = None
    musashi_mame_diff_path = None
    musashi_mame_diff_rc = None
    if args.mame_trace is not None:
        rtl_mame_diff_path = paths.compare / "rtl_vs_mame_trace_diff.txt"
        with rtl_mame_diff_path.open("w", encoding="utf-8") as diff_fp:
            rtl_mame_diff_rc = subprocess.run(
                trace_diff_command(
                    args.repo_root,
                    paths.rtl / "rom_boot_trace.log",
                    args.mame_trace,
                    context=args.trace_context,
                    pc_only=args.mame_pc_only,
                ),
                cwd=str(args.repo_root),
                env=env,
                stdout=diff_fp,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            ).returncode
        musashi_mame_diff_path = paths.compare / "musashi_vs_mame_trace_diff.txt"
        with musashi_mame_diff_path.open("w", encoding="utf-8") as diff_fp:
            musashi_mame_diff_rc = subprocess.run(
                trace_diff_command(
                    args.repo_root,
                    paths.musashi / "musashi_rom_boot_trace.log",
                    args.mame_trace,
                    context=args.trace_context,
                    pc_only=args.mame_pc_only,
                ),
                cwd=str(args.repo_root),
                env=env,
                stdout=diff_fp,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            ).returncode

    sample_compare_path = None
    sample_compare_rc = None
    if args.sample_every > 0:
        sample_compare_path = paths.compare / "arch_sample_compare.txt"
        with sample_compare_path.open("w", encoding="utf-8") as sample_compare_fp:
            sample_compare_rc = subprocess.run(
                arch_sample_compare_command(args.repo_root, paths.rtl, paths.musashi),
                cwd=str(args.repo_root),
                env=env,
                stdout=sample_compare_fp,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            ).returncode

    report_path = paths.compare / "report.txt"
    write_report(
        report_path=report_path,
        paths=paths,
        rtl_summary_path=rtl_summary_path,
        rtl_summary_line=rtl_summary_line,
        rtl_periph_path=rtl_periph_path,
        musashi_summary_path=musashi_summary_path,
        musashi_summary_line=musashi_summary_line,
        musashi_summary=musashi_summary,
        musashi_periph_path=musashi_periph_path,
        trace_diff_path=trace_diff_path,
        trace_diff_rc=trace_diff_rc,
        sample_compare_path=sample_compare_path,
        sample_compare_rc=sample_compare_rc,
        mame_trace_path=args.mame_trace,
        rtl_mame_diff_path=rtl_mame_diff_path,
        rtl_mame_diff_rc=rtl_mame_diff_rc,
        musashi_mame_diff_path=musashi_mame_diff_path,
        musashi_mame_diff_rc=musashi_mame_diff_rc,
        rtl_rc=rtl_rc,
        musashi_rc=musashi_rc,
    )

    sys.stdout.write(report_path.read_text(encoding="utf-8"))
    return 0 if rtl_rc == 0 and musashi_rc == 0 and rtl_summary_rc == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
