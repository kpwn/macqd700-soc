"""Tests for the MAME architectural-checkpoint replay helper."""

import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


_TOOLS = Path(__file__).resolve().parents[1]
_SCRIPT = _TOOLS / "mame_replay_arch_checkpoint.py"


def _write_checkpoint(root: Path) -> Path:
    checkpoint = root / "q700.arch_checkpoint.txt"
    regs = "\n".join(
        [f"reg d{i}=0x{i + 1:08x}" for i in range(8)]
        + [f"reg a{i}=0x{0x1000 + i:08x}" for i in range(8)]
    )
    checkpoint.write_text(
        textwrap.dedent(
            f"""
            format m68k-ooo-arch-checkpoint-v1
            producer tb-rom-boot
            endianness big
            run cycle=0x1 committed=0x2 stop_reason="unit" overlay=0 visible_ram=0x10 q700_descriptor_selected=1
            arch ccr=0x04 sr=0x2704 sr_source=unit
            {regs}
            pc next_valid=1 next=0x40801234 source=unit commit_pc=0x40801230 committed=0x2
            control vbr=0x00000000 cacr=0x80008000 sfc=0x00000000 dfc=0x00000000 usp=0x00000000 ssp=0x00002000 isp=0x00000000
            mmu tc=0x0000c000 itt0=0x00000000 itt1=0x00000000 dtt0=0x00000000 dtt1=0x00000000 urp=0x00000000 srp=0x003fdc00
            replay supported=debug_arch_load_v1 io_state=reset caches=reset queues=reset predictors=reset
            memory begin
            segment name=q700-rom base=0x40000000 size=0x4 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x1
            data off=0x0 bytes=4 hex=deadbeef
            endsegment name=q700-rom
            segment name=ram base=0x00000000 size=0x20 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=5 chunks=2 checksum_fnv1a64=0x2
            data off=0x0 bytes=4 hex=00010203
            data off=0x8 bytes=1 hex=ff
            endsegment name=ram
            memory end
            end format=m68k-ooo-arch-checkpoint-v1
            """
        ).lstrip(),
        encoding="utf-8",
    )
    return checkpoint


class MameReplayArchCheckpointTests(unittest.TestCase):
    def test_prepare_materializes_seed_files_and_debugger_script(self):
        with tempfile.TemporaryDirectory() as tmp_str:
            tmp = Path(tmp_str)
            checkpoint = _write_checkpoint(tmp)
            out_dir = tmp / "out"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(_SCRIPT),
                    str(checkpoint),
                    "--out-dir",
                    str(out_dir),
                    "--adb-pic",
                    str(tmp / "missing-pic.bin"),
                    "--run-until",
                    "0x408004be",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)

            rom = out_dir / "roms" / "macqd700" / "420dbff3.rom"
            adb = out_dir / "roms" / "macqd700" / "342s0440-b.bin"
            ram = out_dir / "ram.bin"
            dbg = out_dir / "replay.dbg"

            self.assertEqual(rom.read_bytes(), bytes.fromhex("deadbeef"))
            self.assertEqual(len(adb.read_bytes()), 1024)
            self.assertEqual(ram.read_bytes(), bytes.fromhex("0001020300000000ff00000000000000"))

            script = dbg.read_text(encoding="utf-8")
            self.assertIn("bpset 0x408025f4", script)
            self.assertIn("bpclear 1", script)
            self.assertIn("pc=40801234", script)
            self.assertIn("bpset 0x408004be", script)


if __name__ == "__main__":
    unittest.main()
