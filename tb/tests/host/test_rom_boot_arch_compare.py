"""Tests for the portable ROM boot arch checkpoint compare tool."""

import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


_TOOLS = Path(__file__).resolve().parents[3] / "tools"
_SCRIPT = _TOOLS / "rom_boot_arch_compare.py"


def _checkpoint_text(
    *,
    d3: str = "0x33333333",
    pc_next: str = "0x40801234",
    ram_checksum: str = "0x0000000000000002",
    ram_hex: str = "00010203",
) -> str:
    return f"""
        format m68k-ooo-arch-checkpoint-v1
        producer tb-rom-boot
        endianness big
        run cycle=0x0000000000001234 committed=0x00000042 stop_reason_before_flush="done" stop_reason="done" overlay=0 q700_descriptor_selected=1
        flush attempted=1 completed=1 start_cycle=0x0000000000001200 end_cycle=0x0000000000001234 start_committed=0x00000041 end_committed=0x00000042
        quiesce rob_empty=1 rob_head=3 rob_tail=3 int_iq_count=0 mem_iq_count=0 lsu_busy=0 exc_active=0 cache_maint_wait=0 dcache_flush_busy=0 flush_queue_empty=1
        arch ccr=0x1f sr=0x2700 sr_source=commit_arch_sr_15_5_plus_ccr_rat
        reg d0=0x00000000
        reg d1=0x11111111
        reg d2=0x22222222
        reg d3={d3}
        reg d4=0x44444444
        reg d5=0x55555555
        reg d6=0x66666666
        reg d7=0x77777777
        reg a0=0x88888888
        reg a1=0x99999999
        reg a2=0xaaaaaaaa
        reg a3=0xbbbbbbbb
        reg a4=0xcccccccc
        reg a5=0xdddddddd
        reg a6=0xeeeeeeee
        reg a7=0xffffffff
        pc next_valid=1 next={pc_next} source=last_retire_actual_next commit_pc=0x40801230 committed=0x00000042 fetch_dbg_pc=0x40801200 dbg_last_pc=0x408011fc
        control vbr=0x00000000 cacr=0x00000000 sfc=0x00000000 dfc=0x00000000 usp=0x00000000 ssp=0x00000000 isp=0x00000000
        mmu tc=0x00000000 itt0=0x00000000 itt1=0x00000000 dtt0=0x00000000 dtt1=0x00000000 urp=0x00000000 srp=0x00000000
        replay supported=debug_arch_load_v1 io_state=reset caches=reset queues=reset predictors=reset
        memory begin
        segment name=q700-rom base=0x40000000 size=0x4 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x0000000000000001
        data off=0 bytes=4 hex=DEADBEEF
        endsegment name=q700-rom
        segment name=ram base=0x00000000 size=0x04400000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64={ram_checksum}
        data off=0 bytes=4 hex={ram_hex}
        endsegment name=ram
        segment name=vram base=0xf9000000 size=0x00200000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=0 chunks=0 checksum_fnv1a64=0x0000000000000003
        endsegment name=vram
        segment name=magic base=0xffff0000 size=0x00000010 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64=0x0000000000000004
        data off=0 bytes=4 hex=c0ffee00
        endsegment name=magic
        memory end
        end format=m68k-ooo-arch-checkpoint-v1
    """


def _write(path: Path, text: str) -> None:
    path.write_text(textwrap.dedent(text).lstrip(), encoding="utf-8")


def _run(*argv: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(_SCRIPT), *argv],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )


class ArchCheckpointCompareTests(unittest.TestCase):
    def test_identical_checkpoints_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            left = Path(tmp) / "left.txt"
            right = Path(tmp) / "right.txt"
            text = _checkpoint_text()
            _write(left, text)
            _write(right, text)

            proc = _run(str(left), str(right), "--check-replayable")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("[rom-boot-arch-compare] PASS", proc.stdout)
            self.assertIn("pc_next=0x40801234", proc.stdout)

    def test_register_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            left = Path(tmp) / "left.txt"
            right = Path(tmp) / "right.txt"
            _write(left, _checkpoint_text())
            _write(right, _checkpoint_text(d3="0xdeadbeef"))

            proc = _run(str(left), str(right))
            self.assertEqual(proc.returncode, 1)
            self.assertIn("reg.d3", proc.stderr)

    def test_memory_checksum_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            left = Path(tmp) / "left.txt"
            right = Path(tmp) / "right.txt"
            _write(left, _checkpoint_text())
            _write(
                right,
                _checkpoint_text(
                    ram_checksum="0x00000000000000ff",
                    ram_hex="00010204",
                ),
            )

            proc = _run(str(left), str(right))
            self.assertEqual(proc.returncode, 1)
            self.assertIn("segment ram checksum_fnv1a64", proc.stderr)

    def test_replayability_check_is_optional_and_reports_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            left = Path(tmp) / "left.txt"
            right = Path(tmp) / "right.txt"
            _write(left, _checkpoint_text())
            _write(right, _checkpoint_text(pc_next="0xffffffff"))

            unchecked = _run(str(left), str(right))
            self.assertEqual(unchecked.returncode, 1)
            self.assertIn("pc.next", unchecked.stderr)

            checked = _run(str(left), str(right), "--check-replayable")
            self.assertEqual(checked.returncode, 1)
            self.assertIn("right replayability", checked.stderr)
            self.assertIn("pc.next=0xffffffff", checked.stderr)


if __name__ == "__main__":
    unittest.main()
