"""Tests for the portable ROM boot arch checkpoint summary tool."""

import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


_TOOLS = Path(__file__).resolve().parents[3] / "tools"
_SCRIPT = _TOOLS / "rom_boot_arch_checkpoint_summary.py"


def _write_checkpoint(root: str, text: str) -> Path:
    path = Path(root) / "q700.arch_checkpoint.txt"
    path.write_text(textwrap.dedent(text).lstrip(), encoding="utf-8")
    return path


def _run(*argv: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(_SCRIPT), *argv],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )


def _replayable_checkpoint_text(
    *,
    quiesce: str = (
        "quiesce rob_empty=1 rob_head=3 rob_tail=3 int_iq_count=0 "
        "mem_iq_count=0 lsu_busy=0 exc_active=0 cache_maint_wait=0 "
        "dcache_flush_busy=0 flush_queue_empty=1"
    ),
    replay: str = (
        "replay supported=debug_arch_load_v1 io_state=reset caches=reset "
        "queues=reset predictors=reset"
    ),
    include_end_format: bool = True,
) -> str:
    end_format = (
        "end format=m68k-ooo-arch-checkpoint-v1\n" if include_end_format else ""
    )
    return f"""
        format m68k-ooo-arch-checkpoint-v1
        producer tb-rom-boot
        endianness big
        run cycle=0x0000000000001234 committed=0x00000042 stop_reason_before_flush="done" stop_reason="done" overlay=0 q700_descriptor_selected=1
        flush attempted=1 completed=1 start_cycle=0x0000000000001200 end_cycle=0x0000000000001234 start_committed=0x00000041 end_committed=0x00000042
        {quiesce}
        arch ccr=0x1f sr=0x2700 sr_source=commit_arch_sr_15_5_plus_ccr_rat
        reg d0=0x00000000
        reg d1=0x11111111
        reg d2=0x22222222
        reg d3=0x33333333
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
        pc next_valid=1 next=0x40801234 source=last_retire_actual_next commit_pc=0x40801230 committed=0x00000042 fetch_dbg_pc=0x40801200 dbg_last_pc=0x408011fc
        control vbr=0x00000000 cacr=0x00000000 sfc=0x00000000 dfc=0x00000000 usp=0x00000000 ssp=0x00000000 isp=0x00000000
        mmu tc=0x00000000 itt0=0x00000000 itt1=0x00000000 dtt0=0x00000000 dtt1=0x00000000 urp=0x00000000 srp=0x00000000
        {replay}
        memory begin
        segment name=q700-rom base=0x40000000 size=0x4 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x0000000000000001
        data off=0 bytes=4 hex=DEADBEEF
        endsegment name=q700-rom
        segment name=ram base=0x00000000 size=0x04400000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64=0x0000000000000002
        data off=0 bytes=4 hex=00010203
        endsegment name=ram
        segment name=vram base=0xf9000000 size=0x00200000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=0 chunks=0 checksum_fnv1a64=0x0000000000000003
        endsegment name=vram
        segment name=magic base=0xffff0000 size=0x00000010 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64=0x0000000000000004
        data off=0 bytes=4 hex=c0ffee00
        endsegment name=magic
        memory end
        {end_format}
    """


class ArchCheckpointSummaryTests(unittest.TestCase):
    def test_compact_summary_and_check_replayable(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                """
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
                reg d3=0x33333333
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
                pc next_valid=1 next=0x40801234 source=last_retire_actual_next commit_pc=0x40801230 committed=0x00000042 fetch_dbg_pc=0x40801200 dbg_last_pc=0x408011fc
                control vbr=0x00000000 cacr=0x00000000 sfc=0x00000000 dfc=0x00000000 usp=0x00000000 ssp=0x00000000 isp=0x00000000
                mmu tc=0x00000000 itt0=0x00000000 itt1=0x00000000 dtt0=0x00000000 dtt1=0x00000000 urp=0x00000000 srp=0x00000000
                replay supported=debug_arch_load_v1 io_state=reset caches=reset queues=reset predictors=reset
                gap restore=debug_arch_load_is_simulation_only_until_fpga_debug_path_exists
                memory begin
                segment name=q700-rom base=0x40000000 size=0x4 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x0000000000000001
                data off=0 bytes=4 hex=DEADBEEF
                endsegment name=q700-rom
                segment name=ram base=0x00000000 size=0x04400000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64=0x0000000000000002
                data off=0 bytes=4 hex=00010203
                endsegment name=ram
                segment name=vram base=0xf9000000 size=0x00200000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=0 chunks=0 checksum_fnv1a64=0x0000000000000003
                endsegment name=vram
                segment name=magic base=0xffff0000 size=0x00000010 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64=0x0000000000000004
                data off=0 bytes=4 hex=c0ffee00
                endsegment name=magic
                memory end
                end format=m68k-ooo-arch-checkpoint-v1
                """,
            )

            proc = _run(str(checkpoint), "--check-replayable", "--compact")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("[rom-boot-arch-summary] path=", proc.stdout)
            self.assertIn("format=m68k-ooo-arch-checkpoint-v1", proc.stdout)
            self.assertIn("pc_next=0x40801234", proc.stdout)
            self.assertIn("q700_rom=4", proc.stdout)
            self.assertIn("ram=71303168", proc.stdout)
            self.assertIn("vram=2097152", proc.stdout)
            self.assertIn("magic=16", proc.stdout)
            self.assertIn("segments=4", proc.stdout)

    def test_missing_replay_fields_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                """
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
                reg d3=0x33333333
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
                pc next_valid=1 next=0x40801234 source=last_retire_actual_next commit_pc=0x40801230 committed=0x00000042 fetch_dbg_pc=0x40801200 dbg_last_pc=0x408011fc
                control vbr=0x00000000 cacr=0x00000000 sfc=0x00000000 dfc=0x00000000 usp=0x00000000 ssp=0x00000000 isp=0x00000000
                mmu tc=0x00000000 itt0=0x00000000 itt1=0x00000000 dtt0=0x00000000 dtt1=0x00000000 urp=0x00000000 srp=0x00000000
                replay supported=debug_arch_load_v1 io_state=reset caches=reset queues=reset predictors=reset
                memory begin
                segment name=q700-rom base=0x40000000 size=0x4 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x0000000000000001
                data off=0 bytes=4 hex=DEADBEEF
                endsegment name=q700-rom
                memory end
                end format=m68k-ooo-arch-checkpoint-v1
                """,
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 1)
            self.assertIn("missing segments: magic,ram,vram", proc.stderr)

    def test_io_state_must_be_reset_for_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                _replayable_checkpoint_text(
                    replay=(
                        "replay supported=debug_arch_load_v1 "
                        "io_state=captured caches=reset queues=reset "
                        "predictors=reset"
                    )
                ),
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 1)
            self.assertIn(
                "replay.io_state='captured' expected one of",
                proc.stderr,
            )

    def test_via1_v2_requires_io_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                _replayable_checkpoint_text(
                    replay=(
                        "replay supported=debug_arch_load_v2 "
                        "io_state=via1-v1 caches=reset queues=reset "
                        "predictors=reset"
                    )
                ),
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 1)
            self.assertIn(
                "replay.io_state='via1-v1' requires an io via1 line",
                proc.stderr,
            )

    def test_via1_v2_partial_io_restore_metadata_is_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                _replayable_checkpoint_text(
                    replay=(
                        "replay supported=debug_arch_load_v2 "
                        "io_state=via1-v1 io_restore=partial-via1-v1 "
                        "caches=reset queues=reset predictors=reset"
                    )
                ).replace(
                    "memory begin",
                    "\n".join(
                        [
                            "io via1 orb=0x80 ora=0x00 ddrb=0x00 ddra=0x00 "
                            "t1cl=0x00 t1ch=0x00 t1ll=0x00 t1lh=0x00 "
                            "t2cl=0x00 t2ch=0x00 sr=0xff acr=0x00 pcr=0x00 "
                            "ifr=0x00 ier=0x80 t1_running=0 t1_latch=0x0000 "
                            "t1_counter=0x0000 t2_running=0 t2_latch=0x0000 "
                            "t2_counter=0x0000 adb_shift_in_progress=0 "
                            "adb_shift_complete_cyc=0x0000000000000000 "
                            "adb_last_byte=0x00 adb_transaction_id=0x00000000 "
                            "rtc_data_line=1 via1_timer_div=0x0000000000000080 "
                            "via1_timer_div_ctr=0x0000000000000000",
                            "memory begin",
                        ]
                    ),
                ),
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_overlay_is_required_for_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                _replayable_checkpoint_text().replace(
                    " overlay=0 q700_descriptor_selected=1",
                    " q700_descriptor_selected=1",
                ),
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 1)
            self.assertIn("run.overlay=None expected 0 or 1", proc.stderr)

    def test_non_quiesced_checkpoint_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                _replayable_checkpoint_text(
                    quiesce=(
                        "quiesce rob_empty=0 rob_head=3 rob_tail=4 "
                        "int_iq_count=1 mem_iq_count=0 lsu_busy=0 "
                        "exc_active=0 cache_maint_wait=0 "
                        "dcache_flush_busy=1 flush_queue_empty=0"
                    )
                ),
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 1)
            self.assertIn("quiesce.rob_empty is not 1", proc.stderr)
            self.assertIn("quiesce.int_iq_count is not 0", proc.stderr)
            self.assertIn("quiesce.dcache_flush_busy is not 0", proc.stderr)
            self.assertIn("quiesce.flush_queue_empty is not 1", proc.stderr)

    def test_missing_end_marker_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                _replayable_checkpoint_text(include_end_format=False),
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 1)
            self.assertIn(
                "missing end format=m68k-ooo-arch-checkpoint-v1 marker",
                proc.stderr,
            )

    def test_segment_metadata_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                """
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
                reg d3=0x33333333
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
                pc next_valid=1 next=0x40801234 source=last_retire_actual_next commit_pc=0x40801230 committed=0x00000042 fetch_dbg_pc=0x40801200 dbg_last_pc=0x408011fc
                control vbr=0x00000000 cacr=0x00000000 sfc=0x00000000 dfc=0x00000000 usp=0x00000000 ssp=0x00000000 isp=0x00000000
                mmu tc=0x00000000 itt0=0x00000000 itt1=0x00000000 dtt0=0x00000000 dtt1=0x00000000 urp=0x00000000 srp=0x00000000
                replay supported=debug_arch_load_v1 io_state=reset caches=reset queues=reset predictors=reset
                memory begin
                segment name=q700-rom base=0x40000000 size=0x4 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x0000000000000001
                data off=0 bytes=4 hex=DEADBEEF
                endsegment name=q700-rom
                segment name=ram base=0x00000000 size=0x04400000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=5 chunks=1 checksum_fnv1a64=0x0000000000000002
                data off=0 bytes=4 hex=00010203
                endsegment name=ram
                segment name=vram base=0xf9000000 size=0x00200000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=0 chunks=0 checksum_fnv1a64=0x0000000000000003
                endsegment name=vram
                segment name=magic base=0xffff0000 size=0x00000010 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64=0x0000000000000004
                data off=0 bytes=4 hex=c0ffee00
                endsegment name=magic
                memory end
                end format=m68k-ooo-arch-checkpoint-v1
                """,
            )

            proc = _run(str(checkpoint), "--check-replayable")
            self.assertEqual(proc.returncode, 1)
            self.assertIn(
                "segment ram material_bytes=0x5 but data bytes=0x4",
                proc.stderr,
            )

    def test_frontier_open_bus_pc_and_aline_slot_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                """
                format m68k-ooo-arch-checkpoint-v1
                producer tb-rom-boot
                endianness big
                run cycle=0x00000000015623e1 committed=0x0072d185 stop_reason_before_flush="stop-exc class=f-line" stop_reason="stop-exc class=f-line" overlay=0 q700_descriptor_selected=1
                flush attempted=1 completed=1 start_cycle=0x0000000001562311 end_cycle=0x00000000015623e1 start_committed=0x0072d185 end_committed=0x0072d185
                quiesce rob_empty=1 rob_head=6 rob_tail=6 int_iq_count=0 mem_iq_count=0 lsu_busy=0 exc_active=0 cache_maint_wait=0 dcache_flush_busy=0 flush_queue_empty=1
                arch ccr=0x00 sr=0x2000 sr_source=commit_arch_sr_15_5_plus_ccr_rat
                reg d0=0x00000000
                reg d1=0x00000000
                reg d2=0x00000047
                reg d3=0x00000000
                reg d4=0x00000000
                reg d5=0x00000000
                reg d6=0x00000000
                reg d7=0x00000000
                reg a0=0x00000000
                reg a1=0x00000000
                reg a2=0x00000000
                reg a3=0x00000000
                reg a4=0x00000000
                reg a5=0x00000000
                reg a6=0x00000000
                reg a7=0x0017ffdc
                pc next_valid=1 next=0xffffffff source=last_retire_actual_next commit_pc=0x40809a04 committed=0x0072d185 fetch_dbg_pc=0x00000001 dbg_last_pc=0x40809a04
                control vbr=0x00000000 cacr=0x80008000 sfc=0x00000000 dfc=0x00000000 usp=0x00000000 ssp=0x0017ffdc isp=0x00000000
                mmu tc=0x0000c000 itt0=0xf900c060 itt1=0x807fc040 dtt0=0xf900c060 dtt1=0x807fc040 urp=0x00000000 srp=0x043ffa00
                replay supported=debug_arch_load_v1 io_state=reset caches=reset queues=reset predictors=reset
                memory begin
                segment name=q700-rom base=0x40000000 size=0x4 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x0000000000000001
                data off=0 bytes=4 hex=DEADBEEF
                endsegment name=q700-rom
                segment name=ram base=0x00000000 size=0x04400000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=4 chunks=1 checksum_fnv1a64=0x0000000000000002
                data off=0x51c bytes=4 hex=ffffffff
                endsegment name=ram
                segment name=vram base=0xf9000000 size=0x00200000 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=0 chunks=0 checksum_fnv1a64=0x0000000000000003
                endsegment name=vram
                segment name=magic base=0xffff0000 size=0x00000010 encoding=sparse-hex default=0x00 chunk_bytes=32 material_bytes=0 chunks=0 checksum_fnv1a64=0x0000000000000004
                endsegment name=magic
                memory end
                end format=m68k-ooo-arch-checkpoint-v1
                """,
            )

            proc = _run(str(checkpoint), "--check-replayable", "--compact")
            self.assertEqual(proc.returncode, 1)
            self.assertIn("aline_slot=0x0000051c", proc.stdout)
            self.assertIn("aline_value=0xffffffff", proc.stdout)
            self.assertIn(
                "pc.next=0xffffffff is an open-bus/null replay target",
                proc.stderr,
            )
            self.assertIn(
                "a-line slot from d2 points at 0x0000051c=0xffffffff",
                proc.stderr,
            )

    def test_frontier_rom_source_samples_follow_q700_mirror(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = _write_checkpoint(
                tmp,
                """
                format m68k-ooo-arch-checkpoint-v1
                producer tb-rom-boot
                endianness big
                run cycle=0x0000000000001234 committed=0x00000042 stop_reason_before_flush="done" stop_reason="done" overlay=0 q700_descriptor_selected=1
                flush attempted=1 completed=1 start_cycle=0x0000000000001200 end_cycle=0x0000000000001234 start_committed=0x00000041 end_committed=0x00000042
                quiesce rob_empty=1 rob_head=3 rob_tail=3 int_iq_count=0 mem_iq_count=0 lsu_busy=0 exc_active=0 cache_maint_wait=0 dcache_flush_busy=0 flush_queue_empty=1
                arch ccr=0x00 sr=0x2000 sr_source=commit_arch_sr_15_5_plus_ccr_rat
                pc next_valid=1 next=0x40809a9a source=last_retire_actual_next commit_pc=0x40809a96 committed=0x00000042 fetch_dbg_pc=0x40809aa4 dbg_last_pc=0x40809a96
                replay supported=debug_arch_load_v1 io_state=reset caches=reset queues=reset predictors=reset
                memory begin
                segment name=q700-rom base=0x40000000 size=0x00100000 encoding=hex-full chunk_bytes=32 checksum_fnv1a64=0x0000000000000001
                data off=0x000ca0e0 bytes=4 hex=80ff0004
                data off=0x000ca3f0 bytes=4 hex=050f918c
                endsegment name=q700-rom
                memory end
                end format=m68k-ooo-arch-checkpoint-v1
                """,
            )

            proc = _run(str(checkpoint), "--compact")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("rom_src_408ca0e0=0x80ff0004", proc.stdout)
            self.assertIn("rom_src_408ca3f0=0x050f918c", proc.stdout)


if __name__ == "__main__":
    unittest.main()
