#!/usr/bin/env python3
"""Tests for tools/macsym.py — the Mac OS symbol map / ELF generator.

Two things are being defended here:

1. **A malformed symbol map must be an error, not a partial load.**  Half a
   symbol file is worse than none: it puts confident wrong names on right
   addresses, and a name in a backtrace is trusted far more than a bare
   address.

2. **The generated ELF must be one stock GDB actually accepts.**  There is a
   real end-to-end check below that shells out to `gdb-multiarch` and asks it
   to resolve an address back to a name, because "the bytes look like an ELF"
   and "GDB can use it" are different claims.
"""

import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import macsym  # noqa: E402


GOOD_MAP = """\
# a comment
region lowram 00000100 00100000
region rom    40800000 40900000

0002E938  _FSDispatch     0040 ; File System trap dispatcher
0002E8A4  _vSyncWaitPoll
40806ECA  _SlotIntDispatch
000008CE  CrsrNew         0001 :data ; cursor flag
"""


class TestParsing(unittest.TestCase):
    def test_parses_symbols_regions_sizes_and_comments(self):
        syms, regions = macsym.parse_map(GOOD_MAP)
        self.assertEqual(len(syms), 4)
        self.assertEqual(len(regions), 2)
        by = {s.name: s for s in syms}
        self.assertEqual(by["_FSDispatch"].addr, 0x0002E938)
        self.assertEqual(by["_FSDispatch"].size, 0x40)
        self.assertEqual(by["_FSDispatch"].comment,
                         "File System trap dispatcher")
        self.assertEqual(by["_vSyncWaitPoll"].size, 0)
        self.assertTrue(by["CrsrNew"].is_data)
        self.assertFalse(by["_FSDispatch"].is_data)

    def test_addresses_are_hex_not_decimal(self):
        """Bare tokens are HEX.  Reading them as decimal is exactly the bug
        that once made `r 40800000` read 0x026E8F80."""
        syms, _ = macsym.parse_map("region r 0 1000\n0100 foo\n")
        self.assertEqual(syms[0].addr, 0x100)

    def test_duplicate_name_is_an_error(self):
        bad = "region r 0 10000\n0100 foo\n0200 foo\n"
        with self.assertRaises(macsym.SymbolError) as cm:
            macsym.parse_map(bad)
        self.assertIn("duplicate", str(cm.exception))

    def test_malformed_line_is_an_error_not_a_skip(self):
        bad = "region r 0 10000\n0100 foo\nthis is not a symbol line\n"
        with self.assertRaises(macsym.SymbolError) as cm:
            macsym.parse_map(bad)
        self.assertIn(":3:", str(cm.exception))

    def test_bad_region_bounds_rejected(self):
        with self.assertRaises(macsym.SymbolError):
            macsym.parse_map("region r 2000 1000\n")

    def test_symbol_outside_every_region_is_an_error(self):
        syms, regions = macsym.parse_map(
            "region r 00000000 00001000\n0002E938 _FSDispatch\n")
        with self.assertRaises(macsym.SymbolError) as cm:
            macsym.assign_regions(syms, regions)
        self.assertIn("outside every declared region", str(cm.exception))


class TestSymbolTable(unittest.TestCase):
    def setUp(self):
        syms, _ = macsym.parse_map(GOOD_MAP)
        self.t = macsym.SymbolTable(syms)

    def test_exact_and_offset_lookup(self):
        self.assertEqual(self.t.format(0x0002E938), "0x0002e938 <_FSDispatch>")
        self.assertEqual(self.t.format(0x0002E94A),
                         "0x0002e94a <_FSDispatch+0x12>")

    def test_address_far_past_a_symbol_is_not_attributed(self):
        """An address 40 MB past the last known symbol is unknown territory,
        not `_SlotIntDispatch+0x2800000`.  Refusing to name it is the whole
        point — a wrong name reads as fact."""
        self.assertEqual(self.t.format(0x43000000), "0x43000000")

    def test_sized_symbol_bounds_the_attribution(self):
        # _FSDispatch has size 0x40, so +0x40 is past its end.
        self.assertIsNone(self.t.lookup(0x0002E938 + 0x40))
        self.assertIsNotNone(self.t.lookup(0x0002E938 + 0x3F))

    def test_address_below_all_symbols_is_unknown(self):
        self.assertIsNone(self.t.lookup(0x10))

    def test_by_name(self):
        self.assertEqual(self.t.by_name("CrsrNew").addr, 0x8CE)
        self.assertIsNone(self.t.by_name("nope"))


class TestElf(unittest.TestCase):
    def setUp(self):
        syms, regions = macsym.parse_map(GOOD_MAP)
        self.blob = macsym.build_elf(syms, regions)

    def test_elf_header_is_big_endian_m68k(self):
        self.assertEqual(self.blob[:4], b"\x7fELF")
        self.assertEqual(self.blob[4], macsym.ELFCLASS32)
        self.assertEqual(self.blob[5], macsym.ELFDATA2MSB)
        e_type, e_machine = struct.unpack(">HH", self.blob[16:20])
        self.assertEqual(e_machine, macsym.EM_68K)
        self.assertEqual(e_type, macsym.ET_EXEC)

    def test_code_sections_are_nobits(self):
        """NOBITS is what makes GDB disassemble the LIVE board instead of a
        stale copy baked into the symbol file."""
        shoff, = struct.unpack(">I", self.blob[32:36])
        shentsize, shnum = struct.unpack(">HH", self.blob[46:50])
        kinds = []
        for i in range(shnum):
            off = shoff + i * shentsize
            sh_type, = struct.unpack(">I", self.blob[off + 4:off + 8])
            sh_addr, = struct.unpack(">I", self.blob[off + 12:off + 16])
            if sh_addr:
                kinds.append(sh_type)
        self.assertTrue(kinds)
        self.assertTrue(all(k == macsym.SHT_NOBITS for k in kinds))

    @unittest.skipUnless(shutil.which("gdb-multiarch"),
                         "gdb-multiarch not installed")
    def test_real_gdb_resolves_symbols(self):
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / "syms.elf"
            elf.write_bytes(self.blob)
            out = subprocess.run(
                ["gdb-multiarch", "-batch", "-nx", str(elf),
                 "-ex", "info symbol 0x2e94a",
                 "-ex", "info symbol 0x40806eca",
                 "-ex", "info symbol 0x8ce"],
                capture_output=True, text=True, timeout=60).stdout
        self.assertIn("_FSDispatch + 18", out)
        self.assertIn("_SlotIntDispatch", out)
        self.assertIn("CrsrNew", out)


class TestShippedMaps(unittest.TestCase):
    """The maps we actually ship must load, and must build an ELF GDB likes."""

    @classmethod
    def setUpClass(cls):
        cls.paths = sorted((REPO / "tools" / "macsyms").glob("*.syms"))

    def test_shipped_maps_exist_and_parse(self):
        self.assertTrue(self.paths, "no tools/macsyms/*.syms shipped")
        syms, regions = macsym.load_map_files(self.paths)
        self.assertGreater(len(syms), 20)
        macsym.assign_regions(syms, regions or macsym.infer_regions(syms))

    def test_owner_supplied_addresses_are_present(self):
        """These came from the project owner and are the seed of the map."""
        table = macsym.SymbolTable.from_files(self.paths)
        for addr in (0x0002E938, 0x0002E8A4, 0x0002CCCE, 0x0002CD1E,
                     0x0002CD26, 0x0002CD98, 0x0002CDAE, 0x0002EEDC,
                     0x00009C74, 0x00009C9E, 0x00009CBA, 0x00009CF2,
                     0x00009D2C, 0x00014716):
            self.assertIsNotNone(table.lookup(addr),
                                 f"0x{addr:08x} lost from the symbol maps")

    def test_queue_and_dce_symbols_present(self):
        """`monitor queue` / `monitor dce` resolve these by name and refuse to
        guess, so their absence would silently disable those commands."""
        table = macsym.SymbolTable.from_files(self.paths)
        for name in ("FSQHdr", "DTQueue", "VBLQueue",
                     "UTableBase", "UnitNtryCnt"):
            self.assertIsNotNone(table.by_name(name), f"{name} missing")

    def test_lowmem_globals_have_widths(self):
        """`monitor lomem` reads `size` bytes; a sizeless byte flag would be
        read 4 bytes wide and print a wrong value."""
        table = macsym.SymbolTable.from_files(self.paths)
        for name in ("CrsrNew", "DSErrCode", "Ticks"):
            s = table.by_name(name)
            self.assertIsNotNone(s, f"{name} missing")
            self.assertGreater(s.size, 0, f"{name} has no width")

    @unittest.skipUnless(shutil.which("gdb-multiarch"),
                         "gdb-multiarch not installed")
    def test_shipped_maps_build_an_elf_gdb_accepts(self):
        syms, regions = macsym.load_map_files(self.paths)
        blob = macsym.build_elf(syms, regions or None)
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / "macos.elf"
            elf.write_bytes(blob)
            r = subprocess.run(
                ["gdb-multiarch", "-batch", "-nx", str(elf),
                 "-ex", "info symbol 0x2e938"],
                capture_output=True, text=True, timeout=60)
        self.assertIn("_FSDispatch", r.stdout)


if __name__ == "__main__":
    unittest.main()
