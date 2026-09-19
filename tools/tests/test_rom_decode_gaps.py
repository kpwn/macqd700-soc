#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import rom_decode_gaps  # noqa: E402


class RomDecodeGapsTests(unittest.TestCase):
    def test_family_normalizes_objdump_size_aliases(self) -> None:
        self.assertEqual(rom_decode_gaps.family("movel %a0@,%d0"), "move.l")
        self.assertEqual(rom_decode_gaps.family("moveaw %pc@(4),%a0"), "movea.w")
        self.assertEqual(rom_decode_gaps.family("cmpal %a0,%a1"), "cmpa.l")
        self.assertEqual(rom_decode_gaps.family("subaw %d0,%a0"), "suba.w")

    def test_expand_family_variants_adds_size_siblings(self) -> None:
        expanded = rom_decode_gaps.expand_family_variants({"asl.w", "divul.l", "move.l"})

        self.assertIn("asl.b", expanded)
        self.assertIn("asl.w", expanded)
        self.assertIn("asl.l", expanded)
        self.assertIn("divu.w", expanded)
        self.assertIn("divu.l", expanded)
        self.assertIn("divul.l", expanded)
        self.assertIn("move.b", expanded)
        self.assertIn("move.w", expanded)
        self.assertIn("move.l", expanded)

    def test_expand_family_variants_preserves_unsized_families(self) -> None:
        expanded = rom_decode_gaps.expand_family_variants({"btst", "bfextu", "sne"})

        self.assertEqual(expanded, {"btst", "bfextu", "sne"})


if __name__ == "__main__":
    unittest.main()
