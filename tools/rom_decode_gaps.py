#!/usr/bin/env python3
"""Inventory Q700 ROM opwords that fall into the RTL decoder vec-4 fallback."""

import argparse
import csv
import os
import re
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


OBJ_LINE_RE = re.compile(
    r"^\s*([0-9a-fA-F]+):\s+((?:[0-9a-fA-F]{4}(?:\s+|$))+)\s*(.*)$"
)
TRACE_RE = re.compile(r"^\s*([0-9a-fA-F]{8})\s+([0-9a-fA-F]{4})\b")


SIZE_VARIANT_GROUPS = (
    ("add.b", "add.w", "add.l"),
    ("adda.w", "adda.l"),
    ("addi.b", "addi.w", "addi.l"),
    ("addq.b", "addq.w", "addq.l"),
    ("addx.b", "addx.w", "addx.l"),
    ("and.b", "and.w", "and.l"),
    ("andi.b", "andi.w", "andi.l"),
    ("asl.b", "asl.w", "asl.l"),
    ("asr.b", "asr.w", "asr.l"),
    ("cas.b", "cas.w", "cas.l"),
    ("chk.w", "chk.l"),
    ("clr.b", "clr.w", "clr.l"),
    ("cmp.b", "cmp.w", "cmp.l"),
    ("cmp2.b", "cmp2.w", "cmp2.l"),
    ("cmpa.w", "cmpa.l"),
    ("cmpi.b", "cmpi.w", "cmpi.l"),
    ("divs.w", "divs.l", "divsl.l"),
    ("divu.w", "divu.l", "divul.l"),
    ("eor.b", "eor.w", "eor.l"),
    ("eori.b", "eori.w", "eori.l"),
    ("lsl.b", "lsl.w", "lsl.l"),
    ("lsr.b", "lsr.w", "lsr.l"),
    ("move.b", "move.w", "move.l"),
    ("movea.w", "movea.l"),
    ("moves.b", "moves.w", "moves.l"),
    ("muls.w", "muls.l"),
    ("mulu.w", "mulu.l"),
    ("neg.b", "neg.w", "neg.l"),
    ("negx.b", "negx.w", "negx.l"),
    ("not.b", "not.w", "not.l"),
    ("or.b", "or.w", "or.l"),
    ("ori.b", "ori.w", "ori.l"),
    ("rol.b", "rol.w", "rol.l"),
    ("ror.b", "ror.w", "ror.l"),
    ("roxl.b", "roxl.w", "roxl.l"),
    ("roxr.b", "roxr.w", "roxr.l"),
    ("sub.b", "sub.w", "sub.l"),
    ("suba.w", "suba.l"),
    ("subi.b", "subi.w", "subi.l"),
    ("subq.b", "subq.w", "subq.l"),
    ("subx.b", "subx.w", "subx.l"),
    ("tst.b", "tst.w", "tst.l"),
)

SIZE_VARIANTS_BY_FAMILY = {
    family_name: frozenset(group)
    for group in SIZE_VARIANT_GROUPS
    for family_name in group
}


@dataclass(frozen=True)
class Inst:
    addr: int
    opword: int
    words: tuple[str, ...]
    text: str


def parse_u32(s: str) -> int:
    return int(s, 0)


def canonical_rom_addr(addr: int, base: int) -> int:
    if base == 0x40800000 and 0x40000000 <= addr < 0x40100000:
        return addr + 0x00800000
    return addr


def run_objdump(rom: Path, base: int, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "m68k-linux-gnu-objdump",
        "-D",
        "-b",
        "binary",
        "-m",
        "m68k:68040",
        f"--adjust-vma=0x{base:08x}",
        str(rom),
    ]
    with out.open("w") as f:
        subprocess.run(cmd, check=True, stdout=f)


def parse_objdump(path: Path) -> list[Inst]:
    insts: list[Inst] = []
    for line in path.read_text(errors="replace").splitlines():
        m = OBJ_LINE_RE.match(line)
        if not m:
            continue
        addr = int(m.group(1), 16)
        words = tuple(m.group(2).split())
        text = m.group(3).strip()
        if not text or text == "...":
            continue
        insts.append(Inst(addr=addr, opword=int(words[0], 16), words=words, text=text))
    return insts


def parse_trace(path: Path, base: int) -> Counter[tuple[int, int]]:
    counts: Counter[tuple[int, int]] = Counter()
    if not path:
        return counts
    for line in path.read_text(errors="replace").splitlines():
        m = TRACE_RE.match(line)
        if not m:
            continue
        pc = canonical_rom_addr(int(m.group(1), 16), base)
        op = int(m.group(2), 16)
        counts[(pc, op)] += 1
    return counts


def parse_family_filter(path: Path | None) -> set[str]:
    if path is None:
        return set()
    families: set[str] = set()
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        families.add(line.lower())
    return families


def expand_family_variants(families: set[str]) -> set[str]:
    expanded = set(families)
    for fam in families:
        expanded.update(SIZE_VARIANTS_BY_FAMILY.get(fam, ()))
    return expanded


def write_addr_file(insts: list[Inst], path: Path) -> None:
    with path.open("w") as f:
        for inst in insts:
            f.write(f"0x{inst.addr:08x}\n")


def run_probe(probe: Path, rom: Path, base: int, addr_file: Path) -> dict[int, dict[str, int]]:
    cmd = [
        str(probe),
        f"+rom={rom}",
        f"+base=0x{base:08x}",
        f"+addr_file={addr_file}",
    ]
    proc = subprocess.run(cmd, check=True, text=True, stdout=subprocess.PIPE)
    rows: dict[int, dict[str, int]] = {}
    for row in csv.DictReader(proc.stdout.splitlines()):
        addr = parse_u32(row["addr"])
        rows[addr] = {
            "opword": parse_u32(row["opword"]),
            "uop_valid": int(row["uop_valid"]),
            "uop_type": int(row["uop_type"]),
            "uop_op": int(row["uop_op"]),
            "exc_valid": int(row["exc_valid"]),
            "exc_vec": int(row["exc_vec"]),
            "requires_supervisor": int(row["requires_supervisor"]),
            "len_bytes": int(row["len_bytes"]),
            "pd_consumed": int(row["pd_consumed"]),
        }
    return rows


def mnemonic(text: str) -> str:
    return text.split()[0].lower() if text else ""


def family(text: str) -> str:
    mnem = mnemonic(text)
    if mnem.startswith("."):
        return mnem
    aliases = {
        "moveal": "movea.l",
        "moveaw": "movea.w",
        "cmpal": "cmpa.l",
        "cmpaw": "cmpa.w",
        "addal": "adda.l",
        "addaw": "adda.w",
        "subal": "suba.l",
        "subaw": "suba.w",
    }
    if mnem in aliases:
        return aliases[mnem]
    if len(mnem) > 1 and mnem[-1] in "bwl":
        return f"{mnem[:-1]}.{mnem[-1]}"
    if len(mnem) > 1 and mnem[-1] == "s" and mnem[:-1] in {
        "bra", "bsr", "bhi", "bls", "bcc", "bcs", "bne", "beq",
        "bvc", "bvs", "bpl", "bmi", "bge", "blt", "bgt", "ble",
    }:
        return f"{mnem[:-1]}.s"
    return mnem


def low_confidence_data(inst: Inst) -> bool:
    text = inst.text.lower()
    return text.startswith(".short") or text.startswith(".word")


def intentional_exception(inst: Inst) -> bool:
    if inst.opword == 0x4AFC:
        return True
    if (inst.opword >> 12) == 0xA:
        return True
    return False


def build_candidates(
    insts: list[Inst],
    probe_rows: dict[int, dict[str, int]],
    trace_counts: Counter[tuple[int, int]],
    family_filter: set[str],
) -> list[dict[str, object]]:
    candidates: list[dict[str, object]] = []
    for inst in insts:
        row = probe_rows.get(inst.addr)
        if not row:
            continue
        if not (row["exc_valid"] and row["exc_vec"] == 4):
            continue
        if intentional_exception(inst):
            continue
        fam = family(inst.text)
        if family_filter and fam not in family_filter:
            continue
        candidates.append(
            {
                "addr": inst.addr,
                "opword": inst.opword,
                "family": fam,
                "text": inst.text,
                "low_confidence": low_confidence_data(inst),
                "trace_count": trace_counts.get((inst.addr, inst.opword), 0),
                **row,
            }
        )
    return candidates


def write_candidates(path: Path, candidates: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "addr",
        "opword",
        "family",
        "text",
        "low_confidence",
        "trace_count",
        "uop_type",
        "uop_op",
        "len_bytes",
        "pd_consumed",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, delimiter="\t")
        w.writeheader()
        for c in candidates:
            row = dict(c)
            row["addr"] = f"0x{int(row['addr']):08x}"
            row["opword"] = f"0x{int(row['opword']):04x}"
            w.writerow({k: row[k] for k in fields})


def print_summary(
    insts: list[Inst],
    candidates: list[dict[str, object]],
    out: Path,
    family_filter: set[str],
    requested_family_filter: set[str],
) -> None:
    unique_static = len({i.opword for i in insts})
    filtered = [c for c in candidates if not c["low_confidence"]]
    traced = [c for c in filtered if c["trace_count"]]
    filtered_families = {str(c["family"]) for c in filtered}

    print(f"ROM objdump instructions parsed: {len(insts)}")
    print(f"Unique static opwords: {unique_static}")
    if family_filter:
        if requested_family_filter != family_filter:
            print(f"Requested family filter entries: {len(requested_family_filter)}")
            print(f"Expanded family filter entries: {len(family_filter)}")
        else:
            print(f"Family filter entries: {len(family_filter)}")
        print(f"Filtered families with vec-4 candidates: {len(filtered_families)}")
        missing = sorted(family_filter - filtered_families)
        if missing:
            print(f"Filter families with no current vec-4 candidates: {', '.join(missing)}")
    print(f"RTL vec-4 fallback candidates: {len(candidates)}")
    print(f"Filtered candidates excluding .short/.word: {len(filtered)}")
    print(f"Trace-observed filtered candidates: {len(traced)}")
    print(f"Candidate TSV: {out}")
    print()

    fam_counts = Counter(str(c["family"]) for c in filtered)
    print("Top static filtered families:")
    for fam, count in fam_counts.most_common(20):
        examples = [c for c in filtered if c["family"] == fam][:3]
        ex = "; ".join(
            f"0x{int(e['addr']):08x}: 0x{int(e['opword']):04x} {e['text']}"
            for e in examples
        )
        print(f"  {fam:<12} {count:5d}  {ex}")

    if traced:
        print()
        print("Trace-observed candidates:")
        for c in sorted(traced, key=lambda x: (-int(x["trace_count"]), int(x["addr"])))[:40]:
            print(
                f"  x{int(c['trace_count']):<6d} "
                f"0x{int(c['addr']):08x}: 0x{int(c['opword']):04x} {c['text']}"
            )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--rom", type=Path, default=Path("files/420dbff3.rom"))
    p.add_argument("--base", type=parse_u32, default=0x40800000)
    p.add_argument("--probe", type=Path, required=True)
    p.add_argument("--objdump", type=Path)
    p.add_argument("--trace", type=Path)
    p.add_argument(
        "--families",
        type=Path,
        help="Optional newline-separated normalized mnemonic families to report, e.g. romcodes.",
    )
    p.add_argument(
        "--expand-variants",
        action="store_true",
        help="Expand size-specific families to their B/W/L or W/L sibling set.",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=Path("/dev/shm/m68k-ooo/decode_gap/rom_decode_gaps.tsv"),
    )
    args = p.parse_args()

    if not args.probe.exists():
        print(f"decode probe not found: {args.probe}", file=sys.stderr)
        return 2

    objdump = args.objdump
    if objdump is None:
        root = Path(os.environ.get("ROM_DECODE_GAP_ROOT", "/dev/shm/m68k-ooo/decode_gap"))
        objdump = root / f"{args.rom.stem}.objdump"
    if not objdump.exists():
        run_objdump(args.rom, args.base, objdump)

    insts = parse_objdump(objdump)
    if not insts:
        print(f"no objdump instructions parsed from {objdump}", file=sys.stderr)
        return 2

    trace_counts = parse_trace(args.trace, args.base) if args.trace else Counter()
    requested_family_filter = parse_family_filter(args.families)
    family_filter = (
        expand_family_variants(requested_family_filter)
        if args.expand_variants
        else requested_family_filter
    )

    with tempfile.NamedTemporaryFile("w", delete=False, dir="/dev/shm", prefix="rom-decode-addrs-") as f:
        addr_path = Path(f.name)
    try:
        write_addr_file(insts, addr_path)
        probe_rows = run_probe(args.probe, args.rom, args.base, addr_path)
    finally:
        addr_path.unlink(missing_ok=True)

    candidates = build_candidates(insts, probe_rows, trace_counts, family_filter)
    write_candidates(args.out, candidates)
    print_summary(insts, candidates, args.out, family_filter, requested_family_filter)
    return 0


if __name__ == "__main__":
    sys.exit(main())
