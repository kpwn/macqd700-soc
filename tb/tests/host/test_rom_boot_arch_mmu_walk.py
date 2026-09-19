"""Tests for the ROM boot arch-checkpoint MMU walk tool."""

import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


_TOOLS = Path(__file__).resolve().parents[3] / "tools"
_SCRIPT = _TOOLS / "rom_boot_arch_mmu_walk.py"


def _write_checkpoint(root: str) -> Path:
    path = Path(root) / "q700.arch_checkpoint.txt"
    path.write_text(
        textwrap.dedent(
            """
            format m68k-ooo-arch-checkpoint-v1
            mmu tc=0x0000c000 itt0=0x00000000 itt1=0x00000000 dtt0=0x00000000 dtt1=0x00000000 urp=0x00000000 srp=0x00001000
            memory begin
            segment name=ram base=0x00000000 size=0x04400000 encoding=sparse-hex default=0xff chunk_bytes=32 material_bytes=28 chunks=7 checksum_fnv1a64=0x0000000000000001
            data off=0x00001000 bytes=4 hex=0000200a
            data off=0x00001070 bytes=4 hex=0000200a
            data off=0x00001080 bytes=4 hex=0000200a
            data off=0x00002000 bytes=4 hex=0000300a
            data off=0x00003000 bytes=4 hex=00000039
            data off=0x00002080 bytes=4 hex=0000400a
            data off=0x00004018 bytes=4 hex=4080c039
            endsegment name=ram
            memory end
            end format=m68k-ooo-arch-checkpoint-v1
            """
        ).lstrip(),
        encoding="utf-8",
    )
    return path


def _run(*argv: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(_SCRIPT), *argv],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )


class ArchMmuWalkTests(unittest.TestCase):
    def test_alias_walk_reports_same_physical_vector_slot(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(tmp)

            proc = _run(str(checkpoint), "0x00000028", "0x38000028")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("va=0x00000028", proc.stdout)
            self.assertIn("va=0x38000028", proc.stdout)
            self.assertEqual(proc.stdout.count("result pa=0x00000028"), 2)

    def test_rom_walk_uses_fixed_8k_040_slices(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(tmp)

            proc = _run(str(checkpoint), "0x4080cfcc")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("l1_idx=0x20", proc.stdout)
            self.assertIn("l2_idx=0x20", proc.stdout)
            self.assertIn("leaf_idx=0x06", proc.stdout)
            self.assertIn("result pa=0x4080cfcc", proc.stdout)


if __name__ == "__main__":
    unittest.main()
