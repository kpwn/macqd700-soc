#!/usr/bin/env python3
"""macsym — Mac OS / Q700 symbol maps for the m68k-ooo FPGA debugger.

We have no Mac OS symbol file.  What we DO have is a slowly growing map of
addresses that hours of debugging have identified ("this is the File System
dispatcher", "this is where the Device Manager writes ioResult").  This
module turns that map into two things:

1. A ``SymbolTable`` for host-side symbolization — used by ``gdbstub.py``'s
   ``monitor`` commands (backtrace, queue walkers, watchpoint reports) so
   they print ``_FSDispatch+0x12`` instead of ``0x0002E94A``.

2. A real **ELF symbol file** that stock GDB can load, so ``bt``, ``disas``,
   ``info symbol`` and ``break _FSDispatch`` work in the normal way::

       tools/macsym.py build tools/macsyms/macos_q700.syms -o build/macos.elf
       gdb-multiarch build/macos.elf
       (gdb) target remote :1234

   The ELF's code sections are ``SHT_NOBITS`` — they carry addresses and
   symbols but no bytes, so GDB disassembles by reading the *live target*
   rather than a stale file copy.  That is exactly what we want: the ROM and
   RAM on the board are the truth, the symbol file only names things.

Map file format (``*.syms``) — one entry per line::

    # comment
    0002E938  _FSDispatch              ; File System trap dispatcher
    0002E8A4  _vSyncWaitPoll   0010    ; optional third field = size in hex
    region    lowram  00000000 00100000

``region`` lines are optional; without them regions are inferred by
clustering symbols (a gap larger than ``--cluster-gap``, default 1 MiB,
starts a new region).  Names are emitted as ``STT_FUNC`` unless the entry
name ends in ``$d`` or the line carries ``:data``, in which case
``STT_OBJECT`` is used — Mac low-memory globals are data, not code.

Deliberately dependency-free: no binutils, no pyelftools.  The ELF writer
below is ~200 lines of struct-packing and is covered by
``tools/tests/test_macsym.py``, including a round-trip that asks real
``gdb-multiarch`` to resolve an address back to a name.
"""

from __future__ import annotations

import argparse
import bisect
import re
import struct
import sys
from pathlib import Path

# ── ELF constants (32-bit, big-endian, m68k) ────────────────────────────
ELFCLASS32 = 1
ELFDATA2MSB = 2
EV_CURRENT = 1
ET_EXEC = 2
EM_68K = 4

SHT_NULL = 0
SHT_PROGBITS = 1
SHT_SYMTAB = 2
SHT_STRTAB = 3
SHT_NOBITS = 8

SHF_WRITE = 0x1
SHF_ALLOC = 0x2
SHF_EXECINSTR = 0x4

STB_GLOBAL = 1
STT_OBJECT = 1
STT_FUNC = 2

ELF_HDR_SIZE = 52
SHDR_SIZE = 40
SYM_SIZE = 16

DEFAULT_CLUSTER_GAP = 1 << 20


class SymbolError(ValueError):
    """Raised on a malformed symbol map.  Never silently skipped — a symbol
    map that half-parses would put wrong names on right addresses, which is
    exactly the class of quiet lie this tooling must not tell."""


class Symbol:
    __slots__ = ("addr", "name", "size", "is_data", "comment")

    def __init__(self, addr: int, name: str, size: int = 0,
                 is_data: bool = False, comment: str = ""):
        self.addr = addr
        self.name = name
        self.size = size
        self.is_data = is_data
        self.comment = comment

    def __repr__(self):  # pragma: no cover - debugging aid
        return f"Symbol(0x{self.addr:08x}, {self.name!r}, size={self.size})"


class Region:
    __slots__ = ("name", "start", "end", "is_data")

    def __init__(self, name: str, start: int, end: int, is_data: bool = False):
        self.name = name
        self.start = start
        self.end = end
        self.is_data = is_data


# ── Map file parsing ────────────────────────────────────────────────────

_HEX = r"[0-9A-Fa-f]+"
_SYM_RE = re.compile(
    rf"^\s*(?P<addr>{_HEX})\s+(?P<name>[A-Za-z_.$][\w.$]*)"
    rf"(?:\s+(?P<size>{_HEX}))?\s*(?P<rest>.*)$"
)
_REGION_RE = re.compile(
    rf"^\s*region\s+(?P<name>[\w.$]+)\s+(?P<start>{_HEX})\s+(?P<end>{_HEX})"
    rf"\s*(?P<rest>.*)$"
)


def parse_map(text: str, source: str = "<string>"):
    """Parse a .syms map.  Returns (symbols, regions).

    Raises SymbolError with the offending line number on anything it cannot
    parse.  There is no "skip the weird line" path on purpose."""
    symbols: list[Symbol] = []
    regions: list[Region] = []
    seen: dict[str, int] = {}

    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0]
        # ';' starts a trailing comment, but only after the fields.
        stripped = line.strip()
        if not stripped:
            continue

        m = _REGION_RE.match(stripped)
        if m:
            start = int(m.group("start"), 16)
            end = int(m.group("end"), 16)
            if end <= start:
                raise SymbolError(
                    f"{source}:{lineno}: region {m.group('name')!r} has "
                    f"end 0x{end:x} <= start 0x{start:x}")
            regions.append(Region(m.group("name"), start, end,
                                  is_data=":data" in m.group("rest")))
            continue

        m = _SYM_RE.match(stripped)
        if not m:
            raise SymbolError(
                f"{source}:{lineno}: cannot parse {raw.strip()!r} — expected "
                f"'<hexaddr> <name> [<hexsize>] [; comment]' or "
                f"'region <name> <hexstart> <hexend>'")

        addr = int(m.group("addr"), 16)
        name = m.group("name")
        size = int(m.group("size"), 16) if m.group("size") else 0
        rest = m.group("rest")
        comment = rest.split(";", 1)[1].strip() if ";" in rest else ""
        is_data = ":data" in rest or name.endswith("$d")

        if name in seen:
            raise SymbolError(
                f"{source}:{lineno}: duplicate symbol {name!r} (first defined "
                f"on line {seen[name]}) — a duplicate name would make "
                f"'break {name}' ambiguous")
        seen[name] = lineno
        symbols.append(Symbol(addr, name, size, is_data, comment))

    return symbols, regions


def load_map_file(path: Path):
    return parse_map(path.read_text(), source=str(path))


def load_map_files(paths):
    """Load and merge several map files.  Duplicate names across files are an
    error, same as within one file."""
    all_syms: list[Symbol] = []
    all_regions: list[Region] = []
    seen: dict[str, str] = {}
    for p in paths:
        syms, regions = load_map_file(Path(p))
        for s in syms:
            if s.name in seen:
                raise SymbolError(
                    f"{p}: duplicate symbol {s.name!r} — already defined in "
                    f"{seen[s.name]}")
            seen[s.name] = str(p)
        all_syms.extend(syms)
        all_regions.extend(regions)
    return all_syms, all_regions


# ── Host-side symbolization ─────────────────────────────────────────────

class SymbolTable:
    """Address → name lookup for host-side pretty-printing.

    ``lookup()`` deliberately refuses to attribute an address to a symbol that
    is implausibly far away: an address 300 KiB past the nearest known symbol
    is *not* "that symbol + 0x4B000", it is unknown territory, and saying
    otherwise is the kind of confident-but-wrong output that costs hours."""

    #: Max distance past a symbol we are willing to call "sym+off" when the
    #: symbol has no explicit size.
    DEFAULT_MAX_OFFSET = 0x1000

    def __init__(self, symbols=(), max_offset: int | None = None):
        self._syms = sorted(symbols, key=lambda s: s.addr)
        self._addrs = [s.addr for s in self._syms]
        self._by_name = {s.name: s for s in self._syms}
        self.max_offset = (self.DEFAULT_MAX_OFFSET if max_offset is None
                           else max_offset)

    @classmethod
    def from_files(cls, paths, max_offset: int | None = None):
        syms, _regions = load_map_files(paths)
        return cls(syms, max_offset=max_offset)

    def __len__(self):
        return len(self._syms)

    def by_name(self, name: str):
        return self._by_name.get(name)

    def all_symbols(self):
        """Every symbol, ascending by address."""
        return list(self._syms)

    def lookup(self, addr: int):
        """Return (Symbol, offset) for the nearest symbol at or below `addr`,
        or None when there is no plausible match."""
        if not self._syms:
            return None
        i = bisect.bisect_right(self._addrs, addr) - 1
        if i < 0:
            return None
        sym = self._syms[i]
        off = addr - sym.addr
        limit = sym.size if sym.size else self.max_offset
        if off >= limit and off != 0:
            return None
        return sym, off

    def format(self, addr: int, width: int = 8) -> str:
        """``0x0002e94a <_FSDispatch+0x12>`` — or bare hex when unknown.

        Never invents a name; unknown addresses stay unadorned so the reader
        can tell "we don't know" from "we know"."""
        hit = self.lookup(addr)
        base = f"0x{addr:0{width}x}"
        if hit is None:
            return base
        sym, off = hit
        return f"{base} <{sym.name}>" if off == 0 else f"{base} <{sym.name}+0x{off:x}>"


# ── Region inference ────────────────────────────────────────────────────

def infer_regions(symbols, cluster_gap: int = DEFAULT_CLUSTER_GAP,
                  tail_pad: int = 0x100):
    """Group symbols into contiguous address regions.

    A region must span every symbol assigned to it, because an ELF symbol's
    st_shndx has to name a section that actually contains its address — GDB
    ignores (or mis-attributes) symbols that fall outside their section."""
    if not symbols:
        return []
    ordered = sorted(symbols, key=lambda s: s.addr)
    regions = []
    start = ordered[0].addr
    end = ordered[0].addr + max(ordered[0].size, 2)
    is_data = ordered[0].is_data
    for sym in ordered[1:]:
        if sym.addr - end > cluster_gap:
            regions.append(Region(f"seg{len(regions)}", start,
                                  end + tail_pad, is_data))
            start = sym.addr
            end = sym.addr + max(sym.size, 2)
            is_data = sym.is_data
        else:
            end = max(end, sym.addr + max(sym.size, 2))
            is_data = is_data and sym.is_data
    regions.append(Region(f"seg{len(regions)}", start, end + tail_pad, is_data))
    return regions


def assign_regions(symbols, regions):
    """Map each symbol to the index of the region containing it.

    Raises SymbolError if a symbol falls outside every region — silently
    dropping it would produce an ELF that resolves some addresses and not
    others with no indication why."""
    ordered = sorted(range(len(regions)), key=lambda i: regions[i].start)
    out = {}
    for sym in symbols:
        for i in ordered:
            r = regions[i]
            if r.start <= sym.addr < r.end:
                out[sym.name] = i
                break
        else:
            raise SymbolError(
                f"symbol {sym.name!r} at 0x{sym.addr:08x} is outside every "
                f"declared region — add a 'region' line covering it")
    return out


# ── ELF writer ──────────────────────────────────────────────────────────

class _StrTab:
    def __init__(self):
        self.buf = bytearray(b"\0")
        self._off = {"": 0}

    def add(self, s: str) -> int:
        if s in self._off:
            return self._off[s]
        off = len(self.buf)
        self.buf += s.encode("utf-8") + b"\0"
        self._off[s] = off
        return off


def build_elf(symbols, regions=None, entry: int = 0,
              cluster_gap: int = DEFAULT_CLUSTER_GAP) -> bytes:
    """Build an ELF32-BE m68k object carrying `symbols` as a symbol table.

    Layout: [ELF header][.symtab][.strtab][.shstrtab][section headers].
    Address-bearing sections are SHT_NOBITS so GDB reads code from the live
    target instead of from this file."""
    symbols = sorted(symbols, key=lambda s: s.addr)
    if regions:
        regions = sorted(regions, key=lambda r: r.start)
    else:
        regions = infer_regions(symbols, cluster_gap=cluster_gap)
    sym_region = assign_regions(symbols, regions)

    shstrtab = _StrTab()
    strtab = _StrTab()

    # Section indices: 0 = NULL, 1..n = regions, then .symtab, .strtab,
    # .shstrtab.
    n_regions = len(regions)
    idx_symtab = 1 + n_regions
    idx_strtab = idx_symtab + 1
    idx_shstrtab = idx_strtab + 1
    n_sections = idx_shstrtab + 1

    # --- symbol table -------------------------------------------------
    # Index 0 must be the null symbol.
    sym_entries = [b"\0" * SYM_SIZE]
    for sym in symbols:
        st_name = strtab.add(sym.name)
        st_info = (STB_GLOBAL << 4) | (STT_OBJECT if sym.is_data else STT_FUNC)
        shndx = 1 + sym_region[sym.name]
        sym_entries.append(struct.pack(
            ">IIIBBH", st_name, sym.addr, sym.size, st_info, 0, shndx))
    symtab_bytes = b"".join(sym_entries)

    # --- file layout --------------------------------------------------
    off = ELF_HDR_SIZE
    symtab_off = off
    off += len(symtab_bytes)
    strtab_off = off
    off += len(strtab.buf)

    # shstrtab content is finalized after we name every section, but its
    # offset is known once we fix the ordering.
    sec_names = [""]
    for r in regions:
        sec_names.append(f".text.{r.name}" if not r.is_data else f".data.{r.name}")
    sec_names += [".symtab", ".strtab", ".shstrtab"]
    for nm in sec_names:
        shstrtab.add(nm)
    shstrtab_off = off
    off += len(shstrtab.buf)

    # Section headers go last; align to 4.
    off = (off + 3) & ~3
    shoff = off

    # --- ELF header ---------------------------------------------------
    e_ident = bytes([0x7F]) + b"ELF" + bytes([ELFCLASS32, ELFDATA2MSB,
                                              EV_CURRENT, 0]) + b"\0" * 8
    ehdr = e_ident + struct.pack(
        ">HHIIIIIHHHHHH",
        ET_EXEC,            # e_type
        EM_68K,             # e_machine
        EV_CURRENT,         # e_version
        entry,              # e_entry
        0,                  # e_phoff (no program headers)
        shoff,              # e_shoff
        0,                  # e_flags
        ELF_HDR_SIZE,       # e_ehsize
        0, 0,               # e_phentsize, e_phnum
        SHDR_SIZE,          # e_shentsize
        n_sections,         # e_shnum
        idx_shstrtab,       # e_shstrndx
    )
    assert len(ehdr) == ELF_HDR_SIZE, len(ehdr)

    # --- section headers ----------------------------------------------
    def shdr(name, sh_type, flags, addr, offset, size, link=0, info=0,
             align=1, entsize=0):
        return struct.pack(">IIIIIIIIII", shstrtab.add(name), sh_type, flags,
                           addr, offset, size, link, info, align, entsize)

    shdrs = [b"\0" * SHDR_SIZE]
    for i, r in enumerate(regions):
        flags = SHF_ALLOC | (SHF_WRITE if r.is_data else SHF_EXECINSTR)
        shdrs.append(shdr(sec_names[1 + i], SHT_NOBITS, flags, r.start,
                          # NOBITS carries no file bytes; offset is nominal.
                          shstrtab_off + len(shstrtab.buf),
                          r.end - r.start, align=2))
    # sh_info for a symtab = index of the first non-local symbol.  Every
    # symbol we emit is GLOBAL, so that is 1 (just past the null entry).
    shdrs.append(shdr(".symtab", SHT_SYMTAB, 0, 0, symtab_off,
                      len(symtab_bytes), link=idx_strtab, info=1,
                      align=4, entsize=SYM_SIZE))
    shdrs.append(shdr(".strtab", SHT_STRTAB, 0, 0, strtab_off,
                      len(strtab.buf), align=1))
    shdrs.append(shdr(".shstrtab", SHT_STRTAB, 0, 0, shstrtab_off,
                      len(shstrtab.buf), align=1))
    assert len(shdrs) == n_sections

    out = bytearray()
    out += ehdr
    out += symtab_bytes
    out += strtab.buf
    out += shstrtab.buf
    while len(out) < shoff:
        out += b"\0"
    for s in shdrs:
        out += s
    return bytes(out)


# ── CLI ─────────────────────────────────────────────────────────────────

def _default_map_dir() -> Path:
    return Path(__file__).resolve().parent / "macsyms"


def _collect_maps(args) -> list[Path]:
    if args.maps:
        paths = [Path(p) for p in args.maps]
    else:
        d = _default_map_dir()
        paths = sorted(d.glob("*.syms"))
        if not paths:
            raise SystemExit(f"no .syms files found in {d}")
    for p in paths:
        if not p.exists():
            raise SystemExit(f"symbol map not found: {p}")
    return paths


def cmd_build(args) -> int:
    paths = _collect_maps(args)
    symbols, regions = load_map_files(paths)
    if not symbols:
        raise SystemExit("symbol maps contained no symbols")
    blob = build_elf(symbols, regions or None, cluster_gap=args.cluster_gap)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(blob)
    print(f"wrote {out} — {len(symbols)} symbols from "
          f"{', '.join(p.name for p in paths)}")
    return 0


def cmd_lookup(args) -> int:
    table = SymbolTable.from_files(_collect_maps(args))
    for a in args.addrs:
        addr = int(a, 0)
        print(table.format(addr))
    return 0


def cmd_list(args) -> int:
    symbols, _ = load_map_files(_collect_maps(args))
    for s in sorted(symbols, key=lambda s: s.addr):
        note = f"  ; {s.comment}" if s.comment else ""
        print(f"0x{s.addr:08x}  {s.name}{note}")
    print(f"({len(symbols)} symbols)", file=sys.stderr)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-m", "--map", dest="maps", action="append", default=None,
                    help="symbol map file; repeat for several "
                         "(default: every tools/macsyms/*.syms)")
    ap.add_argument("--cluster-gap", type=lambda s: int(s, 0),
                    default=DEFAULT_CLUSTER_GAP,
                    help="address gap that starts a new ELF section")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("build", help="emit an ELF symbol file for GDB")
    p.add_argument("-o", "--output", default="build/macos.elf")
    p.set_defaults(func=cmd_build)

    p = sub.add_parser("lookup", help="symbolize one or more addresses")
    p.add_argument("addrs", nargs="+")
    p.set_defaults(func=cmd_lookup)

    p = sub.add_parser("list", help="list all symbols, sorted by address")
    p.set_defaults(func=cmd_list)

    args = ap.parse_args(argv)
    try:
        return args.func(args)
    except SymbolError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
