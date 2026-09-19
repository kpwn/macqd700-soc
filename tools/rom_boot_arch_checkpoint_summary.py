#!/usr/bin/env python3
"""Summarize and validate portable ROM boot architectural checkpoints.

The arch checkpoint format is plain text, Verilator-independent, and meant to
survive save/restore layout churn.  This tool parses that text format, prints a
concise summary of the replay-relevant fields, and can fail fast when required
fields are missing.

Typical usage:

    python3 tools/rom_boot_arch_checkpoint_summary.py \
        build/rom_boot_checkpoints/q700.arch_checkpoint.txt \
        --check-replayable

Use ``--compact`` for a single-line handoff summary.
"""

from __future__ import annotations

import argparse
import shlex
import sys
from dataclasses import dataclass, field
from pathlib import Path


ARCH_FORMAT = "m68k-ooo-arch-checkpoint-v1"
EXPECTED_REPLAY_SUPPORTS = {"debug_arch_load_v1", "debug_arch_load_v2"}
EXPECTED_IO_STATES = {"reset", "via1-v1"}
EXPECTED_SEGMENTS = {
    "q700-rom": {
        "base": 0x40000000,
        "max_size": 0x00100000,
        "encoding": "hex-full",
    },
    "ram": {
        "base": 0x00000000,
        "size": 0x04400000,
        "encoding": "sparse-hex",
        "default": 0x00,
    },
    "vram": {
        "base": 0xF9000000,
        "size": 0x00200000,
        "encoding": "sparse-hex",
        "default": 0x00,
    },
    "magic": {
        "base": 0xFFFF0000,
        "size": 0x00000010,
        "encoding": "sparse-hex",
        "default": 0x00,
    },
}
Q700_ROM_MIRROR_BASE = 0x40000000
Q700_ROM_MIRROR_LIMIT = 0x50000000
LOW_TRAP_TABLE_START = 0x00000400
LOW_TRAP_TABLE_END = 0x000007FF
FRONTIER_ROM_SOURCE_ADDRS = (
    0x408CA0E0,
    0x408CA3F0,
)


def parse_u32(text: str) -> int:
    return int(text, 0)


def parse_u64(text: str) -> int:
    return int(text, 0)


def fmt_u32(value: int | None) -> str:
    return "-" if value is None else f"0x{value:08x}"


def fmt_u64(value: int | None) -> str:
    return "-" if value is None else f"0x{value:016x}"


def kv_args(parts: list[str]) -> dict[str, str]:
    fields: dict[str, str] = {}
    for part in parts:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields


def canonical_q700_rom_addr(addr: int, rom_size: int) -> int | None:
    if rom_size <= 0:
        return None
    if addr < Q700_ROM_MIRROR_BASE or addr >= Q700_ROM_MIRROR_LIMIT:
        return None
    return Q700_ROM_MIRROR_BASE + ((addr - Q700_ROM_MIRROR_BASE) & (rom_size - 1))


@dataclass
class Segment:
    name: str
    base: int
    size: int
    encoding: str
    default_value: int | None = None
    chunk_bytes: int | None = None
    material_bytes: int | None = None
    chunks: int | None = None
    checksum_fnv1a64: int | None = None
    data_bytes: int = 0
    data_chunks: int = 0
    sample_bytes: dict[int, int] = field(default_factory=dict)


@dataclass
class ArchCheckpoint:
    path: Path
    format_name: str | None = None
    producer: str | None = None
    endianness: str | None = None
    run_cycle: int | None = None
    run_committed: int | None = None
    stop_reason_before_flush: str | None = None
    stop_reason: str | None = None
    overlay: int | None = None
    q700_descriptor_selected: int | None = None
    flush_attempted: int | None = None
    flush_completed: int | None = None
    flush_start_cycle: int | None = None
    flush_end_cycle: int | None = None
    flush_start_committed: int | None = None
    flush_end_committed: int | None = None
    quiesce: dict[str, int] = field(default_factory=dict)
    ccr: int | None = None
    sr: int | None = None
    regs: dict[str, int] = field(default_factory=dict)
    pc_next_valid: int | None = None
    pc_next: int | None = None
    pc_source: str | None = None
    pc_commit_pc: int | None = None
    pc_committed: int | None = None
    control: dict[str, int] = field(default_factory=dict)
    mmu: dict[str, int] = field(default_factory=dict)
    replay: dict[str, str] = field(default_factory=dict)
    has_io_via1: bool = False
    gaps: list[str] = field(default_factory=list)
    rom_patches: list[str] = field(default_factory=list)
    segments: list[Segment] = field(default_factory=list)
    saw_memory_begin: bool = False
    saw_memory_end: bool = False
    saw_end_format: bool = False

    def segment(self, name: str) -> Segment | None:
        for segment in self.segments:
            if segment.name == name:
                return segment
        return None


def parse_checkpoint(path: Path) -> ArchCheckpoint:
    cp = ArchCheckpoint(path=path)
    current_segment: Segment | None = None

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            if line.startswith("format "):
                cp.format_name = line.split(" ", 1)[1]
                continue
            if line.startswith("producer "):
                cp.producer = line.split(" ", 1)[1]
                continue
            if line.startswith("endianness "):
                cp.endianness = line.split(" ", 1)[1]
                continue
            if line.startswith("run "):
                fields = kv_args(shlex.split(line)[1:])
                if "cycle" in fields:
                    cp.run_cycle = parse_u64(fields["cycle"])
                if "committed" in fields:
                    cp.run_committed = parse_u32(fields["committed"])
                cp.stop_reason_before_flush = fields.get("stop_reason_before_flush")
                cp.stop_reason = fields.get("stop_reason")
                if "overlay" in fields:
                    cp.overlay = int(fields["overlay"], 0)
                if "q700_descriptor_selected" in fields:
                    cp.q700_descriptor_selected = int(
                        fields["q700_descriptor_selected"], 0
                    )
                continue
            if line.startswith("flush "):
                fields = kv_args(shlex.split(line)[1:])
                if "attempted" in fields:
                    cp.flush_attempted = int(fields["attempted"], 0)
                if "completed" in fields:
                    cp.flush_completed = int(fields["completed"], 0)
                if "start_cycle" in fields:
                    cp.flush_start_cycle = parse_u64(fields["start_cycle"])
                if "end_cycle" in fields:
                    cp.flush_end_cycle = parse_u64(fields["end_cycle"])
                if "start_committed" in fields:
                    cp.flush_start_committed = parse_u32(fields["start_committed"])
                if "end_committed" in fields:
                    cp.flush_end_committed = parse_u32(fields["end_committed"])
                continue
            if line.startswith("quiesce "):
                fields = kv_args(shlex.split(line)[1:])
                cp.quiesce = {key: int(value, 0) for key, value in fields.items()}
                continue
            if line.startswith("arch "):
                fields = kv_args(shlex.split(line)[1:])
                if "ccr" in fields:
                    cp.ccr = parse_u32(fields["ccr"])
                if "sr" in fields:
                    cp.sr = parse_u32(fields["sr"])
                continue
            if line.startswith("reg "):
                fields = kv_args(shlex.split(line)[1:])
                if fields:
                    reg_name = next(iter(fields))
                    cp.regs[reg_name] = parse_u32(fields[reg_name])
                continue
            if line.startswith("pc "):
                fields = kv_args(shlex.split(line)[1:])
                if "next_valid" in fields:
                    cp.pc_next_valid = int(fields["next_valid"], 0)
                if cp.pc_next_valid and "next" in fields:
                    cp.pc_next = parse_u32(fields["next"])
                cp.pc_source = fields.get("source")
                if "commit_pc" in fields:
                    cp.pc_commit_pc = parse_u32(fields["commit_pc"])
                if "committed" in fields:
                    cp.pc_committed = parse_u32(fields["committed"])
                continue
            if line.startswith("control "):
                fields = kv_args(shlex.split(line)[1:])
                cp.control = {
                    key: parse_u32(value)
                    for key, value in fields.items()
                    if key != "sr_source"
                }
                continue
            if line.startswith("mmu "):
                fields = kv_args(shlex.split(line)[1:])
                cp.mmu = {key: parse_u32(value) for key, value in fields.items()}
                continue
            if line.startswith("replay "):
                fields = kv_args(shlex.split(line)[1:])
                cp.replay = fields
                continue
            if line.startswith("io via1 "):
                cp.has_io_via1 = True
                continue
            if line.startswith("gap "):
                cp.gaps.append(line)
                continue
            if line.startswith("rom_patch "):
                cp.rom_patches.append(line)
                continue
            if line == "memory begin":
                cp.saw_memory_begin = True
                continue
            if line == "memory end":
                cp.saw_memory_end = True
                continue
            if line == f"end format={ARCH_FORMAT}":
                cp.saw_end_format = True
                continue
            if line.startswith("segment "):
                fields = kv_args(shlex.split(line)[1:])
                if not {"name", "base", "size", "encoding"} <= fields.keys():
                    continue
                current_segment = Segment(
                    name=fields["name"],
                    base=parse_u32(fields["base"]),
                    size=parse_u64(fields["size"]),
                    encoding=fields["encoding"],
                    default_value=parse_u32(fields["default"])
                    if "default" in fields
                    else None,
                    chunk_bytes=parse_u64(fields["chunk_bytes"])
                    if "chunk_bytes" in fields
                    else None,
                    material_bytes=parse_u64(fields["material_bytes"])
                    if "material_bytes" in fields
                    else None,
                    chunks=parse_u64(fields["chunks"]) if "chunks" in fields else None,
                    checksum_fnv1a64=parse_u64(fields["checksum_fnv1a64"])
                    if "checksum_fnv1a64" in fields
                    else None,
                )
                cp.segments.append(current_segment)
                continue
            if line == "endsegment" or line.startswith("endsegment "):
                current_segment = None
                continue
            if line.startswith("data "):
                if current_segment is not None:
                    fields = kv_args(shlex.split(line)[1:])
                    off = int(fields["off"], 0)
                    data_len = int(fields["bytes"], 0)
                    current_segment.data_bytes += data_len
                    current_segment.data_chunks += 1
                    if "hex" in fields:
                        data = bytes.fromhex(fields["hex"])
                        for i, value in enumerate(data):
                            addr = current_segment.base + off + i
                            if (
                                current_segment.name == "ram"
                                and LOW_TRAP_TABLE_START <= addr <= LOW_TRAP_TABLE_END
                            ):
                                current_segment.sample_bytes[addr] = value
                            if current_segment.name == "q700-rom":
                                for source_addr in FRONTIER_ROM_SOURCE_ADDRS:
                                    canonical_addr = canonical_q700_rom_addr(
                                        source_addr, current_segment.size
                                    )
                                    if (
                                        canonical_addr is not None
                                        and canonical_addr <= addr < canonical_addr + 4
                                    ):
                                        current_segment.sample_bytes[addr] = value
                continue

    return cp


def segment_sample_addr(segment: Segment, addr: int) -> int | None:
    if segment.name == "q700-rom":
        return canonical_q700_rom_addr(addr, segment.size)
    return addr


def segment_read_u32(segment: Segment | None, addr: int) -> int | None:
    if segment is None:
        return None
    read_addr = segment_sample_addr(segment, addr)
    if read_addr is None:
        return None
    if read_addr < segment.base or read_addr + 4 > segment.base + segment.size:
        return None
    value = 0
    for i in range(4):
        byte_addr = read_addr + i
        byte_value = segment.sample_bytes.get(byte_addr)
        if byte_value is None:
            byte_value = segment.default_value
        if byte_value is None:
            return None
        value = (value << 8) | (byte_value & 0xFF)
    return value


def aline_slot_info(cp: ArchCheckpoint) -> tuple[int, int | None] | None:
    d2 = cp.regs.get("d2")
    if d2 is None or d2 == 0:
        return None
    slot = 0x400 + ((d2 & 0xFFFF) * 4)
    if slot < 0x400 or slot > 0x7FC:
        return None
    return slot, segment_read_u32(cp.segment("ram"), slot)


def frontier_rom_source_info(cp: ArchCheckpoint) -> list[tuple[int, int | None]]:
    rom = cp.segment("q700-rom")
    if rom is None:
        return []
    return [
        (addr, segment_read_u32(rom, addr))
        for addr in FRONTIER_ROM_SOURCE_ADDRS
    ]


def looks_like_aline_frontier(cp: ArchCheckpoint) -> bool:
    reason_text = " ".join(
        text or ""
        for text in (cp.stop_reason_before_flush, cp.stop_reason, cp.pc_source)
    )
    return (
        cp.pc_next == 0xFFFFFFFF
        or cp.pc_commit_pc == 0x40809A04
        or "f-line" in reason_text
    )


def validate_replayable(cp: ArchCheckpoint) -> list[str]:
    errors: list[str] = []

    if cp.format_name != ARCH_FORMAT:
        errors.append(f"format={cp.format_name!r} expected {ARCH_FORMAT!r}")
    if cp.producer != "tb-rom-boot":
        errors.append(f"producer={cp.producer!r} expected 'tb-rom-boot'")
    if cp.endianness != "big":
        errors.append(f"endianness={cp.endianness!r} expected 'big'")
    if cp.run_cycle is None or cp.run_committed is None:
        errors.append("missing run cycle/committed fields")
    if cp.overlay not in (0, 1):
        errors.append(f"run.overlay={cp.overlay} expected 0 or 1")
    if cp.flush_attempted != 1 or cp.flush_completed != 1:
        errors.append(
            f"flush attempted={cp.flush_attempted} completed={cp.flush_completed} "
            "expected 1/1"
        )
    if cp.quiesce.get("rob_empty") != 1:
        errors.append("quiesce.rob_empty is not 1")
    for key in (
        "int_iq_count",
        "mem_iq_count",
        "lsu_busy",
        "exc_active",
        "cache_maint_wait",
        "dcache_flush_busy",
        "flush_queue_empty",
    ):
        if cp.quiesce.get(key) != 0 and key != "flush_queue_empty":
            errors.append(f"quiesce.{key} is not 0")
    if cp.quiesce.get("flush_queue_empty") != 1:
        errors.append("quiesce.flush_queue_empty is not 1")
    if cp.ccr is None or cp.sr is None:
        errors.append("missing arch ccr/sr fields")

    expected_regs = [f"d{i}" for i in range(8)] + [f"a{i}" for i in range(8)]
    missing_regs = [reg for reg in expected_regs if reg not in cp.regs]
    if missing_regs:
        errors.append("missing registers: " + ",".join(missing_regs))

    if cp.pc_next_valid != 1:
        errors.append(f"pc.next_valid={cp.pc_next_valid} expected 1")
    if cp.pc_next is None:
        errors.append("missing pc.next field")
    elif cp.pc_next in (0x00000000, 0xFFFFFFFF):
        errors.append(
            f"pc.next={fmt_u32(cp.pc_next)} is an open-bus/null replay target"
        )
    if cp.pc_committed is None:
        errors.append("missing pc.committed field")

    for key in ("vbr", "cacr", "sfc", "dfc", "usp", "ssp", "isp"):
        if key not in cp.control:
            errors.append(f"missing control.{key}")
    for key in ("tc", "itt0", "itt1", "dtt0", "dtt1", "urp", "srp"):
        if key not in cp.mmu:
            errors.append(f"missing mmu.{key}")

    if cp.replay.get("supported") not in EXPECTED_REPLAY_SUPPORTS:
        errors.append(
            f"replay.supported={cp.replay.get('supported')!r} expected "
            f"one of {sorted(EXPECTED_REPLAY_SUPPORTS)!r}"
        )
    if cp.replay.get("io_state") not in EXPECTED_IO_STATES:
        errors.append(
            f"replay.io_state={cp.replay.get('io_state')!r} expected "
            f"one of {sorted(EXPECTED_IO_STATES)!r}"
        )
    if cp.replay.get("supported") == "debug_arch_load_v2":
        if cp.replay.get("io_state") != "via1-v1":
            errors.append(
                "replay.supported='debug_arch_load_v2' requires "
                "replay.io_state='via1-v1'"
            )
        if not cp.has_io_via1:
            errors.append("replay.io_state='via1-v1' requires an io via1 line")
    for key in ("caches", "queues", "predictors"):
        if cp.replay.get(key) != "reset":
            errors.append(f"replay.{key}={cp.replay.get(key)!r} expected 'reset'")

    if not cp.saw_memory_begin or not cp.saw_memory_end:
        errors.append("missing memory begin/end markers")
    if not cp.saw_end_format:
        errors.append(f"missing end format={ARCH_FORMAT} marker")

    required_segments = set(EXPECTED_SEGMENTS)
    seen_segments = {segment.name for segment in cp.segments}
    missing_segments = sorted(required_segments - seen_segments)
    if missing_segments:
        errors.append("missing segments: " + ",".join(missing_segments))
    duplicate_segments = sorted(
        name
        for name in seen_segments
        if sum(1 for segment in cp.segments if segment.name == name) > 1
    )
    if duplicate_segments:
        errors.append("duplicate segments: " + ",".join(duplicate_segments))
    unsupported_segments = sorted(seen_segments - required_segments)
    if unsupported_segments:
        errors.append("unsupported segments: " + ",".join(unsupported_segments))

    for segment in cp.segments:
        expected = EXPECTED_SEGMENTS.get(segment.name)
        if expected is None:
            continue
        if segment.base != expected["base"]:
            errors.append(
                f"segment {segment.name} base={fmt_u32(segment.base)} "
                f"expected {fmt_u32(expected['base'])}"
            )
        if segment.encoding != expected["encoding"]:
            errors.append(
                f"segment {segment.name} encoding={segment.encoding!r} "
                f"expected {expected['encoding']!r}"
            )
        if "size" in expected and segment.size != expected["size"]:
            errors.append(
                f"segment {segment.name} size=0x{segment.size:x} "
                f"expected 0x{expected['size']:x}"
            )
        if "max_size" in expected:
            if segment.size == 0 or segment.size > expected["max_size"]:
                errors.append(
                    f"segment {segment.name} size=0x{segment.size:x} "
                    f"expected 1..0x{expected['max_size']:x}"
                )
        if "default" in expected and segment.default_value != expected["default"]:
            errors.append(
                f"segment {segment.name} default={segment.default_value!r} "
                f"expected 0x{expected['default']:02x}"
            )
        if segment.checksum_fnv1a64 is None:
            errors.append(f"segment {segment.name} missing checksum_fnv1a64")
        if segment.chunk_bytes is None or segment.chunk_bytes == 0:
            errors.append(f"segment {segment.name} missing/zero chunk_bytes")
        if segment.encoding == "hex-full" and segment.data_bytes != segment.size:
            errors.append(
                f"segment {segment.name} data bytes=0x{segment.data_bytes:x} "
                f"expected full size 0x{segment.size:x}"
            )
        if segment.encoding == "sparse-hex":
            if segment.material_bytes is None:
                errors.append(f"segment {segment.name} missing material_bytes")
            elif segment.material_bytes != segment.data_bytes:
                errors.append(
                    f"segment {segment.name} material_bytes=0x{segment.material_bytes:x} "
                    f"but data bytes=0x{segment.data_bytes:x}"
                )
            if segment.chunks is None:
                errors.append(f"segment {segment.name} missing chunks")
            elif segment.chunks != segment.data_chunks:
                errors.append(
                    f"segment {segment.name} chunks={segment.chunks} "
                    f"but data chunks={segment.data_chunks}"
                )
            if segment.data_bytes > segment.size:
                errors.append(
                    f"segment {segment.name} data bytes=0x{segment.data_bytes:x} "
                    f"exceeds size 0x{segment.size:x}"
                )

    slot_info = aline_slot_info(cp)
    if slot_info is not None and looks_like_aline_frontier(cp):
        slot, value = slot_info
        if value == 0xFFFFFFFF:
            errors.append(
                f"a-line slot from d2 points at {fmt_u32(slot)}="
                f"{fmt_u32(value)} (open bus)"
            )

    for addr, value in frontier_rom_source_info(cp):
        if value == 0xFFFFFFFF:
            errors.append(
                f"frontier ROM source sample {fmt_u32(addr)}="
                f"{fmt_u32(value)} in checkpoint ROM image"
            )

    return errors


def render_summary(cp: ArchCheckpoint, compact: bool) -> str:
    run_cycle = fmt_u64(cp.run_cycle)
    flush_delta = None
    if cp.flush_start_committed is not None and cp.flush_end_committed is not None:
        flush_delta = cp.flush_end_committed - cp.flush_start_committed

    q = cp.quiesce
    q_text = (
        "rob_empty={rob_empty} rob_head={rob_head} rob_tail={rob_tail} "
        "int_iq_count={int_iq_count} mem_iq_count={mem_iq_count} "
        "lsu_busy={lsu_busy} exc_active={exc_active} "
        "cache_maint_wait={cache_maint_wait} dcache_flush_busy={dcache_flush_busy} "
        "flush_queue_empty={flush_queue_empty}"
    ).format(
        rob_empty=q.get("rob_empty", "-"),
        rob_head=q.get("rob_head", "-"),
        rob_tail=q.get("rob_tail", "-"),
        int_iq_count=q.get("int_iq_count", "-"),
        mem_iq_count=q.get("mem_iq_count", "-"),
        lsu_busy=q.get("lsu_busy", "-"),
        exc_active=q.get("exc_active", "-"),
        cache_maint_wait=q.get("cache_maint_wait", "-"),
        dcache_flush_busy=q.get("dcache_flush_busy", "-"),
        flush_queue_empty=q.get("flush_queue_empty", "-"),
    )

    if compact:
        q700 = cp.segment("q700-rom")
        ram = cp.segment("ram")
        vram = cp.segment("vram")
        magic = cp.segment("magic")
        slot_info = aline_slot_info(cp)
        slot_text = ""
        if slot_info is not None:
            slot, value = slot_info
            slot_text = (
                f" aline_slot={fmt_u32(slot)} aline_value={fmt_u32(value)}"
            )
        rom_source_text = "".join(
            f" rom_src_{addr:08x}={fmt_u32(value)}"
            for addr, value in frontier_rom_source_info(cp)
            if value is not None
        )
        return (
            f"[rom-boot-arch-summary] path={cp.path} format={cp.format_name} "
            f"run_cycle={run_cycle} committed={cp.run_committed} "
            f"flush_completed={cp.flush_completed} flush_delta={flush_delta} "
            f"pc_next={fmt_u32(cp.pc_next)} ccr={fmt_u32(cp.ccr)} sr={fmt_u32(cp.sr)} "
            f"q700_rom={q700.size if q700 else '-'} ram={ram.size if ram else '-'} "
            f"vram={vram.size if vram else '-'} magic={magic.size if magic else '-'} "
            f"segments={len(cp.segments)} gaps={len(cp.gaps)}"
            f"{slot_text}{rom_source_text}"
        )

    lines = [
        f"[rom-boot-arch-summary] path={cp.path}",
        f"[rom-boot-arch-summary] format={cp.format_name} producer={cp.producer} "
        f"endianness={cp.endianness}",
        f"[rom-boot-arch-summary] run cycle={run_cycle} committed={cp.run_committed} "
        f"overlay={cp.overlay} q700_descriptor_selected={cp.q700_descriptor_selected}",
        f"[rom-boot-arch-summary] flush attempted={cp.flush_attempted} "
        f"completed={cp.flush_completed} start_committed={fmt_u32(cp.flush_start_committed)} "
        f"end_committed={fmt_u32(cp.flush_end_committed)} delta={flush_delta}",
        f"[rom-boot-arch-summary] quiesce {q_text}",
        f"[rom-boot-arch-summary] arch ccr={fmt_u32(cp.ccr)} sr={fmt_u32(cp.sr)}",
        "[rom-boot-arch-summary] regs "
        + " ".join(f"{reg}={fmt_u32(cp.regs.get(reg))}" for reg in (
            [f"d{i}" for i in range(8)] + [f"a{i}" for i in range(8)]
        )),
        f"[rom-boot-arch-summary] pc next_valid={cp.pc_next_valid} "
        f"next={fmt_u32(cp.pc_next)} commit_pc={fmt_u32(cp.pc_commit_pc)} "
        f"committed={fmt_u32(cp.pc_committed)} source={cp.pc_source}",
        "[rom-boot-arch-summary] control "
        + " ".join(f"{key}={fmt_u32(cp.control.get(key))}" for key in (
            "vbr", "cacr", "sfc", "dfc", "usp", "ssp", "isp"
        )),
        "[rom-boot-arch-summary] mmu "
        + " ".join(f"{key}={fmt_u32(cp.mmu.get(key))}" for key in (
            "tc", "itt0", "itt1", "dtt0", "dtt1", "urp", "srp"
        )),
        "[rom-boot-arch-summary] replay "
        + " ".join(f"{key}={cp.replay.get(key, '-')}" for key in (
            "supported", "io_state", "caches", "queues", "predictors"
        )),
        "[rom-boot-arch-summary] segments "
        + " ".join(
            f"{segment.name}:base={fmt_u32(segment.base)} size=0x{segment.size:x} "
            f"encoding={segment.encoding} bytes={segment.data_bytes}"
            for segment in cp.segments
        ),
    ]
    slot_info = aline_slot_info(cp)
    if slot_info is not None:
        slot, value = slot_info
        lines.append(
            f"[rom-boot-arch-summary] frontier aline_slot={fmt_u32(slot)} "
            f"aline_value={fmt_u32(value)} source=d2_scaled_lowmem"
        )
    rom_source_info = [
        (addr, value)
        for addr, value in frontier_rom_source_info(cp)
        if value is not None
    ]
    if rom_source_info:
        lines.append(
            "[rom-boot-arch-summary] frontier_rom_sources "
            + " ".join(
                f"{fmt_u32(addr)}={fmt_u32(value)}"
                for addr, value in rom_source_info
            )
        )
    if cp.gaps:
        lines.append(f"[rom-boot-arch-summary] gaps count={len(cp.gaps)}")
        lines.extend(f"[rom-boot-arch-summary] {gap}" for gap in cp.gaps)
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument(
        "--check-replayable",
        action="store_true",
        help="fail when replay-critical fields are missing or inconsistent",
    )
    ap.add_argument(
        "--compact",
        action="store_true",
        help="emit a single handoff line instead of the expanded report",
    )
    args = ap.parse_args()

    if not args.checkpoint.is_file():
        print(
            f"[rom-boot-arch-summary] checkpoint not found: {args.checkpoint}",
            file=sys.stderr,
        )
        return 2

    cp = parse_checkpoint(args.checkpoint)
    errors = validate_replayable(cp) if args.check_replayable else []
    print(render_summary(cp, args.compact))
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
