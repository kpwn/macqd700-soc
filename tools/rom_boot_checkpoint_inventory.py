#!/usr/bin/env python3
"""Inventory and validate Q700 ROM boot Verilator checkpoints."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from struct import error as StructError
from struct import unpack


COMMIT_RE = re.compile(r"^q700\.(\d{8})\.vlt$")
CYCLE_RE = re.compile(r"^q700\.cyc(\d{8})\.vlt$")
FINAL_NAME = "q700.final.vlt"
VLTSAVE_HEADER = b"verilatorsave02\n"
VLTSAVE_TRAILER = b"vltsaved"
CHECKPOINT_MAGIC = 0x5142434B  # "QBCK"
CHECKPOINT_VERSION = 1
EXPECTED_MEM_SIZE = 0x04400010
REGENERATE_HINT = (
    "regenerate with MAKEFLAGS='-j1' make rom-boot-snapshots, then "
    "MAKEFLAGS='-j1' make rom-boot-deep-snapshots"
)
RESTORE_RE = re.compile(
    r"\[rom-boot\] checkpoint restored: (?P<path>.+) "
    r"\(committed=(?P<committed>\d+) pc=0x(?P<pc>[0-9a-fA-F]+)\)"
)
FINAL_COMMITTED_RE = re.compile(
    r"^\s*committed:\s+(?P<committed>\d+)\s*$", re.MULTILINE
)


@dataclass(frozen=True)
class ExpectedMetadata:
    committed: int
    pc: int
    sim_time: int | None = None


# Current known-good Q700 checkpoint corpus.  These values come from the
# checkpoint save/restore metadata emitted by tb_rom_boot.cpp, but are pinned
# here so stale files fail smoke instead of silently looking alive.
CURRENT_METADATA: dict[str, ExpectedMetadata] = {
    "q700.00000250.vlt": ExpectedMetadata(250, 0x40002F7C, 1454),
    "q700.00000500.vlt": ExpectedMetadata(500, 0x40802F7C, 2862),
    "q700.00001000.vlt": ExpectedMetadata(1000, 0x408047D8, 5016),
    "q700.00002000.vlt": ExpectedMetadata(2000, 0x4084B0A0, 9543),
    "q700.00005000.vlt": ExpectedMetadata(5000, 0x40847518, 19686),
    "q700.00010000.vlt": ExpectedMetadata(10000, 0x4084751C, 29686),
    "q700.00025000.vlt": ExpectedMetadata(25000, 0x4084751C, 59686),
    "q700.00050000.vlt": ExpectedMetadata(50000, 0x40847518, 109686),
    "q700.00100000.vlt": ExpectedMetadata(100000, 0x4084751C, 209686),
    "q700.00250000.vlt": ExpectedMetadata(250000, 0x4084751C, 509686),
    "q700.00500000.vlt": ExpectedMetadata(500000, 0x40847518, 1009686),
    "q700.01000000.vlt": ExpectedMetadata(1000000, 0x4084751C, 2009686),
    "q700.01600000.vlt": ExpectedMetadata(1600000, 0x4084751C, 3209686),
    "q700.final.vlt": ExpectedMetadata(1800000, 0x40847516, 3609686),
    "q700.cyc04000000.vlt": ExpectedMetadata(1995157, 0x40847516, 4000000),
    "q700.cyc08000000.vlt": ExpectedMetadata(3631355, 0x40807118, 8000000),
    "q700.cyc16000000.vlt": ExpectedMetadata(4989138, 0x4084BB9C, 16000000),
    "q700.cyc32000000.vlt": ExpectedMetadata(4989138, 0x4084BB9C, 32000000),
    "q700.cyc40000000.vlt": ExpectedMetadata(4989138, 0x4084BB9C, 40000000),
}


class MetadataError(Exception):
    pass


@dataclass(frozen=True)
class CheckpointMetadata:
    sim_time: int
    host_cycle: int
    overlay: bool
    last_pc: int
    last_same_pc_count: int
    mem_size: int


@dataclass(frozen=True)
class Checkpoint:
    path: Path
    kind: str
    point: int | None
    size: int
    metadata: CheckpointMetadata | None = None
    metadata_error: str | None = None


def parse_points(text: str) -> list[int]:
    if not text:
        return []
    points: list[int] = []
    for item in text.split(","):
        item = item.strip()
        if item:
            points.append(int(item, 0))
    return points


def _read_exact(f, size: int) -> bytes:
    data = f.read(size)
    if len(data) != size:
        raise MetadataError("checkpoint truncated while reading metadata")
    return data


def _read_u8(f) -> int:
    try:
        return unpack("<B", _read_exact(f, 1))[0]
    except StructError as exc:
        raise MetadataError(str(exc)) from exc


def _read_bool(f) -> bool:
    value = _read_u8(f)
    if value not in (0, 1):
        raise MetadataError(f"invalid serialized bool value {value}")
    return bool(value)


def _read_u32(f) -> int:
    try:
        return unpack("<I", _read_exact(f, 4))[0]
    except StructError as exc:
        raise MetadataError(str(exc)) from exc


def _read_u64(f) -> int:
    try:
        return unpack("<Q", _read_exact(f, 8))[0]
    except StructError as exc:
        raise MetadataError(str(exc)) from exc


def _skip_bytes(f, size: int, file_size: int) -> None:
    f.seek(size, 1)
    if f.tell() > file_size:
        raise MetadataError("checkpoint truncated while skipping metadata")


def _skip_map(f, file_size: int) -> None:
    count = _read_u32(f)
    if count > 100000:
        raise MetadataError(f"serialized map entry count is unreasonable: {count}")
    _skip_bytes(f, count * 12, file_size)


def read_checkpoint_metadata(path: Path) -> CheckpointMetadata:
    file_size = path.stat().st_size
    if file_size < len(VLTSAVE_HEADER) + len(VLTSAVE_TRAILER):
        raise MetadataError("checkpoint is too small to contain a Verilator save")

    with path.open("rb") as f:
        trailer_pos = file_size - len(VLTSAVE_TRAILER)
        f.seek(trailer_pos)
        if _read_exact(f, len(VLTSAVE_TRAILER)) != VLTSAVE_TRAILER:
            raise MetadataError("missing Verilator save trailer")

        f.seek(0)
        if _read_exact(f, len(VLTSAVE_HEADER)) != VLTSAVE_HEADER:
            raise MetadataError("missing Verilator save header")

        magic = _read_u32(f)
        version = _read_u32(f)
        if magic != CHECKPOINT_MAGIC or version != CHECKPOINT_VERSION:
            raise MetadataError(
                f"bad Q700 checkpoint header magic=0x{magic:08x} version={version}"
            )

        sim_time = _read_u64(f)
        host_cycle = _read_u64(f)
        overlay = _read_bool(f)

        _skip_bytes(f, 15, file_size)  # VIA1 byte registers.
        _read_bool(f)                  # via1.adb_shift_in_progress
        _read_u64(f)                   # via1.adb_shift_complete_cyc
        _read_u8(f)                    # via1.adb_last_byte
        _skip_bytes(f, 16, file_size)  # VIA2 register shadow.

        _skip_map(f, file_size)        # unmapped_write_count
        _skip_map(f, file_size)        # unmapped_read_count

        _read_u32(f)                   # if_pending
        _read_u32(f)                   # if_addr_q

        _read_bool(f)                  # ax.ar_outstanding
        _read_u32(f)                   # ax.ar_addr
        _read_u32(f)                   # ax.ar_delay
        _read_bool(f)                  # ax.aw_done
        _read_bool(f)                  # ax.w_done
        _read_u32(f)                   # ax.aw_addr
        _read_u32(f)                   # ax.w_data
        _read_u32(f)                   # ax.w_strb
        _read_u32(f)                   # ax.b_delay
        _read_bool(f)                  # ax.b_pending

        last_pc = _read_u32(f)
        last_same_pc_count = _read_u32(f)
        mem_size = _read_u32(f)
        if mem_size != EXPECTED_MEM_SIZE:
            raise MetadataError(
                f"checkpoint memory size {mem_size} != expected {EXPECTED_MEM_SIZE}"
            )
        if f.tell() + mem_size + len(VLTSAVE_TRAILER) > file_size:
            raise MetadataError("checkpoint truncated before serialized memory ends")

    return CheckpointMetadata(
        sim_time=sim_time,
        host_cycle=host_cycle,
        overlay=overlay,
        last_pc=last_pc,
        last_same_pc_count=last_same_pc_count,
        mem_size=mem_size,
    )


def classify(path: Path) -> Checkpoint:
    name = path.name
    metadata: CheckpointMetadata | None = None
    metadata_error: str | None = None
    try:
        metadata = read_checkpoint_metadata(path)
    except (OSError, MetadataError) as exc:
        metadata_error = str(exc)

    match = COMMIT_RE.match(name)
    if match:
        return Checkpoint(
            path, "commit", int(match.group(1)), path.stat().st_size,
            metadata, metadata_error
        )
    match = CYCLE_RE.match(name)
    if match:
        return Checkpoint(
            path, "cycle", int(match.group(1)), path.stat().st_size,
            metadata, metadata_error
        )
    if name == FINAL_NAME:
        return Checkpoint(path, "final", None, path.stat().st_size,
                          metadata, metadata_error)
    return Checkpoint(path, "unknown", None, path.stat().st_size,
                      metadata, metadata_error)


def inventory(directory: Path) -> list[Checkpoint]:
    kind_order = {"commit": 0, "cycle": 1, "final": 2, "unknown": 3}
    return sorted(
        (classify(path) for path in directory.glob("*.vlt")),
        key=lambda cp: (
            kind_order.get(cp.kind, 99),
            cp.point if cp.point is not None else -1,
            cp.path.name,
        ),
    )


def human_size(size: int) -> str:
    units = ["B", "KiB", "MiB", "GiB"]
    value = float(size)
    for unit in units:
        if value < 1024.0 or unit == units[-1]:
            if unit == "B":
                return f"{int(value)} {unit}"
            return f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{size} B"


def point_text(cp: Checkpoint) -> str:
    return "-" if cp.point is None else str(cp.point)


def metadata_pc_text(cp: Checkpoint) -> str:
    if cp.metadata is None:
        return "metadata-error" if cp.metadata_error else "-"
    return f"0x{cp.metadata.last_pc:08x}"


def metadata_cycle_text(cp: Checkpoint) -> str:
    if cp.metadata is None:
        return "-"
    return str(cp.metadata.sim_time)


def expected_for_checkpoint(cp: Checkpoint) -> ExpectedMetadata | None:
    expected = CURRENT_METADATA.get(cp.path.name)
    if expected is not None:
        return expected
    if cp.kind == "commit" and cp.point is not None and cp.metadata is not None:
        return ExpectedMetadata(cp.point, cp.metadata.last_pc)
    return None


def print_table(checkpoints: list[Checkpoint]) -> None:
    print("kind     point       cycle       pc          size       path")
    print("-------  ----------  ----------  ----------  ---------  ----")
    for cp in checkpoints:
        print(
            f"{cp.kind:<7}  {point_text(cp):>10}  "
            f"{metadata_cycle_text(cp):>10}  {metadata_pc_text(cp):>10}  "
            f"{human_size(cp.size):>9}  {cp.path}"
        )


def list_paths(checkpoints: list[Checkpoint]) -> None:
    for cp in checkpoints:
        print(cp.path)


def list_smoke_specs(checkpoints: list[Checkpoint]) -> list[str]:
    errors: list[str] = []
    for cp in checkpoints:
        expected = expected_for_checkpoint(cp)
        if expected is None:
            errors.append(
                f"no restore metadata baseline for {cp.path.name}; "
                "update CURRENT_METADATA after an intentional checkpoint rebaseline"
            )
            continue
        point = point_text(cp)
        print(
            f"{cp.path}\t{expected.committed}\t0x{expected.pc:08x}\t"
            f"{cp.kind}\t{point}"
        )
    return errors


def verify_restore_log(
    log_path: Path,
    snapshot: Path,
    expected_committed: int,
    expected_pc: int,
) -> list[str]:
    try:
        text = log_path.read_text(errors="replace")
    except OSError as exc:
        return [f"cannot read restore log {log_path}: {exc}"]

    matches = list(RESTORE_RE.finditer(text))
    if not matches:
        return [
            f"restore metadata line not found in {log_path}; "
            "the checkpoint may be dead or the harness output changed"
        ]
    if len(matches) > 1:
        return [f"multiple restore metadata lines found in {log_path}"]

    match = matches[0]
    restored_path = Path(match.group("path"))
    actual_committed = int(match.group("committed"))
    actual_pc = int(match.group("pc"), 16)
    errors: list[str] = []

    if restored_path.name != snapshot.name:
        errors.append(
            f"restore log snapshot mismatch: expected {snapshot.name}, "
            f"got {restored_path.name}"
        )
    if actual_committed != expected_committed:
        errors.append(
            f"{snapshot.name}: restored committed={actual_committed}, "
            f"expected {expected_committed}"
        )
    if actual_pc != expected_pc:
        errors.append(
            f"{snapshot.name}: restored pc=0x{actual_pc:08x}, "
            f"expected 0x{expected_pc:08x}"
        )

    final_commits = [
        int(m.group("committed")) for m in FINAL_COMMITTED_RE.finditer(text)
    ]
    if not final_commits:
        errors.append(f"final committed count not found in {log_path}")
    elif final_commits[-1] < actual_committed:
        errors.append(
            f"{snapshot.name}: final committed={final_commits[-1]} is below "
            f"restored committed={actual_committed}"
        )

    if not errors:
        print(
            f"restore metadata ok: {snapshot.name} "
            f"committed={actual_committed} pc=0x{actual_pc:08x}"
        )
    return errors


def validate(
    checkpoints: list[Checkpoint],
    expected_commit: list[int],
    expected_cycle: list[int],
    require_final: bool,
    forbid_unknown: bool,
    require_current_metadata: bool,
) -> list[str]:
    errors: list[str] = []
    commits = sorted(cp.point for cp in checkpoints if cp.kind == "commit")
    cycles = sorted(cp.point for cp in checkpoints if cp.kind == "cycle")
    finals = [cp for cp in checkpoints if cp.kind == "final"]
    unknown = [cp.path.name for cp in checkpoints if cp.kind == "unknown"]

    for cp in checkpoints:
        if cp.kind != "unknown" and cp.metadata_error:
            errors.append(
                f"unreadable checkpoint metadata for {cp.path.name}: "
                f"{cp.metadata_error}; {REGENERATE_HINT}"
            )
        if cp.kind == "cycle" and cp.metadata is not None and cp.point is not None:
            if cp.metadata.sim_time != cp.point:
                errors.append(
                    f"{cp.path.name}: filename cycle {cp.point} does not match "
                    f"checkpoint cycle {cp.metadata.sim_time}; {REGENERATE_HINT}"
                )

    if expected_commit:
        missing = sorted(set(expected_commit) - set(commits))
        extra = sorted(set(commits) - set(expected_commit))
        if missing:
            errors.append(
                "missing committed-uop checkpoints: "
                + ",".join(map(str, missing))
                + "; run MAKEFLAGS='-j1' make rom-boot-snapshots"
            )
        if extra:
            errors.append(
                "unexpected committed-uop checkpoints: " + ",".join(map(str, extra))
            )
    if expected_cycle:
        missing = sorted(set(expected_cycle) - set(cycles))
        extra = sorted(set(cycles) - set(expected_cycle))
        if missing:
            errors.append(
                "missing cycle checkpoints: "
                + ",".join(map(str, missing))
                + "; run MAKEFLAGS='-j1' make rom-boot-deep-snapshots"
            )
        if extra:
            errors.append(
                "unexpected cycle checkpoints: " + ",".join(map(str, extra))
            )
    if require_final and not finals:
        errors.append(
            f"missing {FINAL_NAME}; run MAKEFLAGS='-j1' make rom-boot-snapshots"
        )
    if len(finals) > 1:
        errors.append(f"multiple {FINAL_NAME} entries found")
    if forbid_unknown and unknown:
        errors.append("unknown checkpoint names: " + ",".join(sorted(unknown)))
    if not checkpoints:
        errors.append(f"no .vlt checkpoints found; {REGENERATE_HINT}")
    if require_current_metadata:
        for cp in checkpoints:
            if cp.kind == "unknown":
                continue
            expected = CURRENT_METADATA.get(cp.path.name)
            if expected is None:
                errors.append(
                    f"no current metadata baseline for {cp.path.name}; "
                    "update CURRENT_METADATA after an intentional checkpoint rebaseline"
                )
                continue
            if cp.kind == "commit" and cp.point != expected.committed:
                errors.append(
                    f"{cp.path.name}: committed filename point {cp.point} "
                    f"!= expected {expected.committed}"
                )
            if (
                cp.kind == "cycle"
                and expected.sim_time is not None
                and cp.point != expected.sim_time
            ):
                errors.append(
                    f"{cp.path.name}: cycle filename point {cp.point} "
                    f"!= expected {expected.sim_time}"
                )
            if cp.metadata is not None:
                if cp.metadata.last_pc != expected.pc:
                    errors.append(
                        f"{cp.path.name}: checkpoint metadata pc="
                        f"0x{cp.metadata.last_pc:08x}, expected 0x{expected.pc:08x}; "
                        f"{REGENERATE_HINT} or update CURRENT_METADATA after rebaseline"
                    )
                if (
                    expected.sim_time is not None
                    and cp.metadata.sim_time != expected.sim_time
                ):
                    errors.append(
                        f"{cp.path.name}: checkpoint metadata cycle="
                        f"{cp.metadata.sim_time}, expected {expected.sim_time}; {REGENERATE_HINT}"
                    )
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--expected-commit", default="")
    parser.add_argument("--expected-cycle", default="")
    parser.add_argument("--require-final", action="store_true")
    parser.add_argument("--forbid-unknown", action="store_true")
    parser.add_argument("--require-current-metadata", action="store_true")
    parser.add_argument("--list-paths", action="store_true")
    parser.add_argument("--list-smoke-specs", action="store_true")
    parser.add_argument("--verify-restore-log", type=Path)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--expected-committed")
    parser.add_argument("--expected-pc")
    args = parser.parse_args()

    if args.verify_restore_log:
        if (
            args.snapshot is None
            or args.expected_committed is None
            or args.expected_pc is None
        ):
            print(
                "--verify-restore-log requires --snapshot, --expected-committed, "
                "and --expected-pc",
                file=sys.stderr,
            )
            return 2
        verify_errors = verify_restore_log(
            args.verify_restore_log,
            args.snapshot,
            int(args.expected_committed, 0),
            int(args.expected_pc, 0),
        )
        if verify_errors:
            for error in verify_errors:
                print(f"ERROR: {error}", file=sys.stderr)
            return 1
        return 0

    if not args.directory.is_dir():
        print(
            f"checkpoint directory not found: {args.directory}; {REGENERATE_HINT}",
            file=sys.stderr,
        )
        return 2

    checkpoints = inventory(args.directory)
    errors = validate(
        checkpoints,
        parse_points(args.expected_commit),
        parse_points(args.expected_cycle),
        args.require_final,
        args.forbid_unknown,
        args.require_current_metadata,
    )

    if args.list_smoke_specs:
        errors.extend(list_smoke_specs(checkpoints))
    elif args.list_paths:
        list_paths(checkpoints)
    else:
        print_table(checkpoints)

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
