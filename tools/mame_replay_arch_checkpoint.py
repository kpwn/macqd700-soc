#!/usr/bin/env python3
"""Replay a ROM boot architectural checkpoint inside MAME.

This is a pragmatic bridge between the RTL architectural checkpoint format and
MAME's debugger.  It materializes the checkpoint ROM/RAM into a private MAME
rompath, emits a debugger script that seeds CPU/MMU state, and can optionally
run MAME for a bounded number of instructions.

The replay is not a full machine snapshot: MAME devices keep their own state.
For the current ROM-frontier work this is still useful because it lets us ask
"does MAME's 68040/MMU execute forward from the same architectural CPU/RAM
state?" without committing to a DPI/co-sim integration.
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path


ARCH_FORMAT = "m68k-ooo-arch-checkpoint-v1"
DEFAULT_MACHINE = "macqd700"
DEFAULT_SEED_BREAK = 0x408025F4
DEFAULT_VISIBLE_RAM = 0x00400000
ADB_PIC_SIZE = 1024


def parse_int(text: str) -> int:
    return int(text, 0)


def kv_args(parts: list[str]) -> dict[str, str]:
    fields: dict[str, str] = {}
    for part in parts:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields


def fmt32(value: int) -> str:
    return f"{value & 0xFFFFFFFF:08x}"


def mame_hex(value: int, width: int = 8) -> str:
    return f"{value & ((1 << (width * 4)) - 1):0{width}x}"


@dataclass
class Segment:
    name: str
    base: int
    size: int
    encoding: str
    default: int = 0
    material_bytes: int = 0
    chunks: int = 0


@dataclass
class Checkpoint:
    path: Path
    format_name: str | None = None
    run: dict[str, str] = field(default_factory=dict)
    sr: int = 0
    regs: dict[str, int] = field(default_factory=dict)
    pc: dict[str, str] = field(default_factory=dict)
    control: dict[str, int] = field(default_factory=dict)
    mmu: dict[str, int] = field(default_factory=dict)
    replay: dict[str, str] = field(default_factory=dict)
    segments: dict[str, Segment] = field(default_factory=dict)

    @property
    def visible_ram(self) -> int:
        value = self.run.get("visible_ram")
        if value is None:
            return DEFAULT_VISIBLE_RAM
        return parse_int(value)

    @property
    def overlay(self) -> int | None:
        value = self.run.get("overlay")
        return None if value is None else parse_int(value)

    def replay_pc(self, source: str) -> int:
        if source == "next":
            if self.pc.get("next_valid") != "1" or "next" not in self.pc:
                raise ValueError("checkpoint has no valid pc.next")
            return parse_int(self.pc["next"])
        if source == "commit":
            if "commit_pc" not in self.pc:
                raise ValueError("checkpoint has no pc.commit_pc")
            return parse_int(self.pc["commit_pc"])
        raise ValueError(f"unknown PC source {source!r}")


def parse_checkpoint_metadata(path: Path) -> Checkpoint:
    cp = Checkpoint(path=path)

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("format "):
                cp.format_name = line.split(" ", 1)[1]
                continue
            if line.startswith("run "):
                cp.run = kv_args(shlex.split(line)[1:])
                continue
            if line.startswith("arch "):
                fields = kv_args(shlex.split(line)[1:])
                if "sr" in fields:
                    cp.sr = parse_int(fields["sr"])
                continue
            if line.startswith("reg "):
                fields = kv_args(shlex.split(line)[1:])
                for key, value in fields.items():
                    cp.regs[key] = parse_int(value)
                continue
            if line.startswith("pc "):
                cp.pc = kv_args(shlex.split(line)[1:])
                continue
            if line.startswith("control "):
                cp.control = {
                    key: parse_int(value)
                    for key, value in kv_args(shlex.split(line)[1:]).items()
                }
                continue
            if line.startswith("mmu "):
                cp.mmu = {
                    key: parse_int(value)
                    for key, value in kv_args(shlex.split(line)[1:]).items()
                }
                continue
            if line.startswith("replay "):
                cp.replay = kv_args(shlex.split(line)[1:])
                continue
            if line.startswith("segment "):
                fields = kv_args(shlex.split(line)[1:])
                name = fields["name"]
                cp.segments[name] = Segment(
                    name=name,
                    base=parse_int(fields["base"]),
                    size=parse_int(fields["size"]),
                    encoding=fields["encoding"],
                    default=parse_int(fields.get("default", "0")),
                    material_bytes=parse_int(fields.get("material_bytes", "0")),
                    chunks=parse_int(fields.get("chunks", "0")),
                )

    if cp.format_name != ARCH_FORMAT:
        raise ValueError(f"{path}: format {cp.format_name!r} is not {ARCH_FORMAT!r}")
    for name in ("q700-rom", "ram"):
        if name not in cp.segments:
            raise ValueError(f"{path}: missing {name} segment")
    missing_regs = [name for name in [f"d{i}" for i in range(8)] + [f"a{i}" for i in range(8)] if name not in cp.regs]
    if missing_regs:
        raise ValueError(f"{path}: missing registers: {','.join(missing_regs)}")
    return cp


def materialize_segment(cp: Checkpoint, name: str, output: Path, *, limit: int | None = None) -> int:
    segment = cp.segments[name]
    size = segment.size if limit is None else min(segment.size, limit)
    data = bytearray([segment.default & 0xFF]) * size
    current: str | None = None

    with cp.path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("segment "):
                fields = kv_args(shlex.split(line)[1:])
                current = fields.get("name")
                continue
            if line.startswith("endsegment "):
                current = None
                continue
            if current != name or not line.startswith("data "):
                continue

            fields = kv_args(shlex.split(line)[1:])
            off = parse_int(fields["off"])
            declared = parse_int(fields["bytes"])
            payload = bytes.fromhex(fields["hex"])
            if len(payload) != declared:
                raise ValueError(
                    f"{cp.path}: segment {name} off 0x{off:x} declared "
                    f"{declared} bytes but has {len(payload)}"
                )
            if off >= size:
                continue
            take = min(len(payload), size - off)
            data[off : off + take] = payload[:take]

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(data)
    return size


def ensure_adb_pic(dest: Path, source: Path | None) -> str:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if source and source.exists():
        shutil.copyfile(source, dest)
        return f"copied {source}"
    dest.write_bytes(bytes(ADB_PIC_SIZE))
    return "wrote zero-filled stub"


def stack_register_values(cp: Checkpoint) -> tuple[int, int, int]:
    a7 = cp.regs["a7"]
    usp = cp.control.get("usp", 0)
    ssp = cp.control.get("ssp", 0)
    isp = cp.control.get("isp", 0)
    msp = cp.control.get("msp", 0)

    # The checkpoint still exposes a historical SSP field.  MAME's 68040
    # debugger exposes ISP/MSP, so seed both from the best supervisor value and
    # then set A7 after SR to make the active stack exact.
    supervisor_seed = ssp or isp or msp or a7
    return usp, (isp or supervisor_seed), (msp or supervisor_seed)


def write_debug_script(
    cp: Checkpoint,
    output: Path,
    *,
    ram_path: Path,
    ram_size: int,
    trace_path: Path,
    pc_source: str,
    seed_break: int | None,
    steps: int,
    run_until: list[int],
) -> int:
    pc = cp.replay_pc(pc_source)
    usp, isp, msp = stack_register_values(cp)

    lines: list[str] = []
    if seed_break is not None:
        lines.append(f"bpset 0x{seed_break:08x}")
        lines.append("go")
        lines.append("bpclear 1")
    lines.append(
        'printf "SEED_BEFORE PC=%08X SR=%04X A7=%08X ISP=%08X MSP=%08X '
        'USP=%08X low0=%08X\\n", pc, sr, a7, isp, msp, usp, d@0'
    )
    lines.append(f"load {ram_path},0,0x{ram_size:x}")
    for reg in ("vbr", "cacr", "sfc", "dfc"):
        lines.append(f"{reg}={mame_hex(cp.control.get(reg, 0))}")
    for reg in ("tc", "itt0", "itt1", "dtt0", "dtt1", "urp", "srp"):
        lines.append(f"{reg}={mame_hex(cp.mmu.get(reg, 0))}")
    lines.append(f"usp={mame_hex(usp)}")
    lines.append(f"isp={mame_hex(isp)}")
    lines.append(f"msp={mame_hex(msp)}")
    lines.append(f"sr={mame_hex(cp.sr, 4)}")
    for reg in [f"d{i}" for i in range(8)] + [f"a{i}" for i in range(7)]:
        lines.append(f"{reg}={mame_hex(cp.regs[reg])}")
    lines.append(f"a7={mame_hex(cp.regs['a7'])}")
    lines.append(f"pc={mame_hex(pc)}")
    lines.append(
        'printf "SEED_AFTER PC=%08X SR=%04X D0=%08X D1=%08X D2=%08X '
        'A0=%08X A1=%08X A2=%08X A7=%08X ISP=%08X MSP=%08X VBR=%08X '
        'TC=%08X SRP=%08X low0=%08X vec4=%08X vec10=%08X slot5fc=%08X\\n", '
        "pc, sr, d0, d1, d2, a0, a1, a2, a7, isp, msp, vbr, tc, srp, "
        "d@0, d@10, d@28, d@5fc"
    )
    lines.append(
        f'trace {trace_path},,noloop,{{tracelog "PC=%08X SR=%04X D0=%08X '
        'D1=%08X D2=%08X A0=%08X A1=%08X A2=%08X A7=%08X VBR=%08X '
        'TC=%08X SRP=%08X OP=%04X ", pc, sr, d0, d1, d2, a0, a1, a2, '
        'a7, vbr, tc, srp, w@pc}'
    )
    for addr in run_until:
        lines.append(f"bpset 0x{addr:08x}")
    if run_until:
        lines.append("go")
    elif steps > 0:
        lines.append(f"step {steps}")
    lines.append("trace off")
    lines.append(
        'printf "AFTER_STEP PC=%08X SR=%04X D0=%08X D1=%08X D2=%08X '
        'A0=%08X A1=%08X A2=%08X A7=%08X ISP=%08X MSP=%08X VBR=%08X '
        'TC=%08X SRP=%08X low0=%08X vec4=%08X vec10=%08X\\n", pc, sr, '
        "d0, d1, d2, a0, a1, a2, a7, isp, msp, vbr, tc, srp, d@0, d@10, d@28"
    )
    lines.append("quit")

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return pc


def find_mame(explicit: str | None) -> str:
    if explicit:
        return explicit
    for candidate in ("mame", "/usr/games/mame"):
        resolved = shutil.which(candidate) if "/" not in candidate else candidate
        if resolved and Path(resolved).exists():
            return resolved
    raise FileNotFoundError("could not find mame; pass --mame")


def run_mame(
    *,
    mame: str,
    machine: str,
    rom_root: Path,
    debug_script: Path,
    out_dir: Path,
    timeout: int,
) -> int:
    env = os.environ.copy()
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    for subdir in ("cfg", "nvram", "inp", "sta", "snap", "diff"):
        (out_dir / subdir).mkdir(parents=True, exist_ok=True)

    cmd = [
        mame,
        machine,
        "-rompath",
        str(rom_root),
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
        "-skip_gameinfo",
        "-cfg_directory",
        str(out_dir / "cfg"),
        "-nvram_directory",
        str(out_dir / "nvram"),
        "-input_directory",
        str(out_dir / "inp"),
        "-state_directory",
        str(out_dir / "sta"),
        "-snapshot_directory",
        str(out_dir / "snap"),
        "-diff_directory",
        str(out_dir / "diff"),
    ]
    stdout = out_dir / "stdout.log"
    stderr = out_dir / "stderr.log"
    with stdout.open("w", encoding="utf-8") as out, stderr.open(
        "w", encoding="utf-8"
    ) as err:
        proc = subprocess.run(
            cmd, cwd=out_dir, env=env, stdout=out, stderr=err, timeout=timeout
        )
    return proc.returncode


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("checkpoint", type=Path, help="architectural checkpoint text file")
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=Path("/dev/shm/m68k/mame_arch_replay"),
        help="output directory for MAME rompath, RAM seed, debugger script, and logs",
    )
    ap.add_argument("--mame", help="MAME binary path")
    ap.add_argument("--machine", default=DEFAULT_MACHINE)
    ap.add_argument("--adb-pic", type=Path, default=Path("files/342s0440-b.bin"))
    ap.add_argument(
        "--ram-bytes",
        type=lambda text: int(text, 0),
        default=None,
        help="RAM bytes to materialize/load; default uses checkpoint visible_ram",
    )
    ap.add_argument(
        "--pc-source",
        choices=("next", "commit"),
        default="next",
        help="checkpoint PC field to seed into MAME",
    )
    ap.add_argument(
        "--seed-break",
        type=lambda text: int(text, 0),
        default=DEFAULT_SEED_BREAK,
        help="MAME PC breakpoint to reach before seeding state",
    )
    ap.add_argument(
        "--seed-immediate",
        action="store_true",
        help="seed as soon as the debugger starts instead of first reaching --seed-break",
    )
    ap.add_argument("--steps", type=int, default=40, help="instructions to step after seeding")
    ap.add_argument(
        "--run-until",
        action="append",
        type=lambda text: int(text, 0),
        default=[],
        help="after seeding, set a breakpoint at this PC and run instead of stepping; repeatable",
    )
    ap.add_argument("--timeout", type=int, default=60, help="MAME run timeout in seconds")
    ap.add_argument("--run", action="store_true", help="run MAME after preparing inputs")
    args = ap.parse_args(argv)

    cp = parse_checkpoint_metadata(args.checkpoint)
    out_dir = args.out_dir.resolve()
    rom_dir = out_dir / "roms" / args.machine
    ram_path = out_dir / "ram.bin"
    debug_script = out_dir / "replay.dbg"
    trace_path = out_dir / "step.tr"

    if cp.overlay not in (0, None) and not args.seed_immediate:
        sys.stderr.write(
            "[mame_replay_arch_checkpoint] warning: checkpoint overlay is not off; "
            "the default seed breakpoint assumes high ROM is already visible\n"
        )

    rom_size = materialize_segment(cp, "q700-rom", rom_dir / "420dbff3.rom")
    ram_size = args.ram_bytes if args.ram_bytes is not None else cp.visible_ram
    ram_size = materialize_segment(cp, "ram", ram_path, limit=ram_size)
    pic_status = ensure_adb_pic(rom_dir / "342s0440-b.bin", args.adb_pic)
    pc = write_debug_script(
        cp,
        debug_script,
        ram_path=ram_path,
        ram_size=ram_size,
        trace_path=trace_path,
        pc_source=args.pc_source,
        seed_break=None if args.seed_immediate else args.seed_break,
        steps=args.steps,
        run_until=args.run_until,
    )

    print(f"[mame_replay_arch_checkpoint] checkpoint={args.checkpoint}")
    print(f"[mame_replay_arch_checkpoint] out_dir={out_dir}")
    print(f"[mame_replay_arch_checkpoint] rom={rom_dir / '420dbff3.rom'} bytes={rom_size}")
    print(f"[mame_replay_arch_checkpoint] adb_pic={rom_dir / '342s0440-b.bin'} ({pic_status})")
    print(f"[mame_replay_arch_checkpoint] ram={ram_path} bytes={ram_size}")
    print(f"[mame_replay_arch_checkpoint] pc=0x{pc:08x} source={args.pc_source}")
    print(f"[mame_replay_arch_checkpoint] debugger_script={debug_script}")

    if not args.run:
        return 0

    mame = find_mame(args.mame)
    rc = run_mame(
        mame=mame,
        machine=args.machine,
        rom_root=out_dir / "roms",
        debug_script=debug_script,
        out_dir=out_dir,
        timeout=args.timeout,
    )
    print(f"[mame_replay_arch_checkpoint] mame_rc={rc}")
    print(f"[mame_replay_arch_checkpoint] debug_log={out_dir / 'debug.log'}")
    print(f"[mame_replay_arch_checkpoint] trace={trace_path}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
