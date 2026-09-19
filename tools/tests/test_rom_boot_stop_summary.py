#!/usr/bin/env python3
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import rom_boot_stop_summary  # noqa: E402


class RomBootStopSummaryTests(unittest.TestCase):
    def test_parses_hex_pc_without_prefix_as_hex(self) -> None:
        self.assertEqual(rom_boot_stop_summary.parse_u32("40800888"), 0x40800888)
        self.assertEqual(rom_boot_stop_summary.parse_u32("026e9278"), 0x026E9278)

    def test_compact_summary_uses_hex_pcs_from_lastn(self) -> None:
        text = (
            "# rom-boot last-N-cycle trace capacity=1 samples=1 "
            "reason=stuck-pc pc=0x40800888 repeats=8192 threshold=8192\n"
            "lastn[00000] sim=1 committed=2 dbg_last_pc=0x40800888 "
            "dbg_pc=0x40800888 rob_v=1 rob_c=1 rob_pc=0x40800888 "
            "rob_vec=0 commit_exc_wait=0 commit_take_exc=0 exc_state=0 "
            "exc_vec=0 exc_fault_pc=0x00000000 exc_a7=0x00000000\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "lastn.log"
            path.write_text(text, encoding="utf-8")
            header, samples = rom_boot_stop_summary.parse_trace(path)

        self.assertIsNotNone(header)
        self.assertEqual(len(samples), 1)
        rendered = rom_boot_stop_summary.render_compact_summary(
            header.reason if header else "unknown",
            samples,
            samples[0],
            samples[0],
        )
        self.assertIn("focus_dbg_pc=0x40800888", rendered)
        self.assertNotIn("0x026e9278", rendered)

    def test_parses_tail_ifetch_and_daxi_fields(self) -> None:
        text = (
            "# rom-boot last-N-cycle trace capacity=3 samples=3 "
            "reason=stop-ifetch-berr addr=0x50f04000\n"
            "lastn[00000] sim=10 committed=7 dbg_last_pc=0x4080002a "
            "dbg_pc=0x4080002e rob_v=0 rob_c=0 rob_pc=0x00000000 "
            "rob_vec=0 commit_exc_wait=0 commit_take_exc=0 exc_state=0 "
            "exc_vec=0 exc_fault_pc=0x00000000 exc_a7=0x00000000 "
            "daxi_ar=0/0/0x00000000 daxi_r=0/0/0 daxi_aw=0/0/0x00000000 "
            "daxi_w=0/0 daxi_b=0/0/0 if=0/0x40800020/0/0\n"
            "lastn[00001] sim=20 committed=8 dbg_last_pc=0x4080002e "
            "dbg_pc=0x40800032 rob_v=1 rob_c=0 rob_pc=0x40800032 "
            "rob_vec=0 commit_exc_wait=0 commit_take_exc=0 exc_state=0 "
            "exc_vec=0 exc_fault_pc=0x00000000 exc_a7=0x00000000 "
            "daxi_ar=1/0/0x50f04000 daxi_r=0/0/0 daxi_aw=1/1/0x0017ffe8 "
            "daxi_w=1/1 daxi_b=0/1/0 if=1/0x50f04000/0/1\n"
            "lastn[00002] sim=30 committed=8 dbg_last_pc=0x4080002e "
            "dbg_pc=0x40800032 rob_v=1 rob_c=0 rob_pc=0x40800032 "
            "rob_vec=0 commit_exc_wait=0 commit_take_exc=0 exc_state=0 "
            "exc_vec=0 exc_fault_pc=0x00000000 exc_a7=0x00000000 "
            "overlay=0/1 "
            "daxi_ar=1/0/0x50f04000 daxi_r=0/0/0 daxi_aw=0/1/0x0017ffe8 "
            "daxi_w=0/1 daxi_b=1/1/0 if=1/0x50f04000/0/1\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "lastn.log"
            path.write_text(text, encoding="utf-8")
            header, samples = rom_boot_stop_summary.parse_trace(path)

        self.assertIsNotNone(header)
        self.assertEqual(len(samples), 3)
        self.assertEqual(samples[-1].if_req, 1)
        self.assertEqual(samples[-1].if_addr, 0x50F04000)
        self.assertEqual(samples[-1].daxi_ar_addr, 0x50F04000)
        self.assertEqual(samples[-1].daxi_aw_addr, 0x0017FFE8)
        self.assertEqual(samples[-1].daxi_b_valid, 1)
        self.assertEqual(samples[-1].overlay_active, 0)
        self.assertEqual(samples[-1].via1_overlay_live, 1)

        rendered = rom_boot_stop_summary.render_compact_summary(
            header.reason if header else "unknown",
            samples,
            rom_boot_stop_summary.pick_focus(samples),
            samples[-1],
        )
        self.assertIn("tail_same_committed=2", rendered)
        self.assertIn("tail_if=1/0x50f04000/0/1", rendered)
        self.assertIn("tail_daxi_ar=1/0/0x50f04000", rendered)
        self.assertIn("tail_daxi_aw=0/1/0x0017ffe8", rendered)
        self.assertIn("tail_daxi_b=1/1/0", rendered)
        self.assertIn("tail_overlay=0/1", rendered)

        activity = rom_boot_stop_summary.render_tail_activity(samples)
        self.assertIn("same_dbg_pc=2", activity)
        self.assertIn("tail_overlay=0/1", activity)
        self.assertIn("last_active_if=1/0x50f04000/1", activity)

    def test_diagnoses_bus_vs_core_waits_from_lastn(self) -> None:
        text = (
            "# rom-boot last-N-cycle trace capacity=2 samples=2 "
            "reason=no-progress cycles=8 threshold=8\n"
            "lastn[00000] sim=10 committed=7 dbg_last_pc=0x4080002a "
            "dbg_pc=0x4080002e rob_v=0 rob_c=0 rob_pc=0x00000000 "
            "rob_vec=0 commit_exc_wait=0 commit_take_exc=0 exc_state=0 "
            "exc_vec=0 exc_fault_pc=0x00000000 exc_a7=0x00000000 "
            "dcache=0/0/0/0 lsu=0/0/0/0/0/0/0x00000000/0x00000000/"
            "0x00000000/0x00000000/0x00000000/0/0/0 "
            "dc=1/0/0x50f04000/0x00000000/0/0/0x00000000/0 "
            "dmmu=0/1/1/0/0/0/0/0/0/0/0/0/0/0/0x00000000/0x00000000 "
            "daxi_ar=1/0/0x50f04000 daxi_r=0/1/0 daxi_aw=0/1/0x00000000 "
            "daxi_w=0/1 daxi_b=0/1/0 if=0/0x4080002e/0/0\n"
            "lastn[00001] sim=11 committed=7 dbg_last_pc=0x4080002a "
            "dbg_pc=0x4080002e rob_v=0 rob_c=0 rob_pc=0x00000000 "
            "rob_vec=0 commit_exc_wait=0 commit_take_exc=0 exc_state=0 "
            "exc_vec=0 exc_fault_pc=0x00000000 exc_a7=0x00000000 "
            "dcache=0/0/0/0 lsu=0/0/0/0/0/0/0x00000000/0x00000000/"
            "0x00000000/0x00000000/0x00000000/0/0/0 "
            "dc=1/0/0x50f04000/0x00000000/0/0/0x00000000/0 "
            "dmmu=0/1/1/0/0/0/0/0/0/0/0/0/0/0/0x00000000/0x00000000 "
            "daxi_ar=1/0/0x50f04000 daxi_r=0/1/0 daxi_aw=0/1/0x00000000 "
            "daxi_w=0/1 daxi_b=0/1/0 if=0/0x4080002e/0/0\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "lastn.log"
            path.write_text(text, encoding="utf-8")
            header, samples = rom_boot_stop_summary.parse_trace(path)

        self.assertIsNotNone(header)
        diagnosis = rom_boot_stop_summary.diagnose_stall(
            samples,
            rom_boot_stop_summary.pick_focus(samples),
            samples[-1],
        )
        self.assertEqual(diagnosis.label, "bus-read-address")
        self.assertEqual(diagnosis.region, "q700-io")

        rendered = rom_boot_stop_summary.render_compact_summary(
            header.reason if header else "unknown",
            samples,
            rom_boot_stop_summary.pick_focus(samples),
            samples[-1],
        )
        self.assertIn("diagnosis=bus-read-address", rendered)
        self.assertIn("region=q700-io", rendered)


if __name__ == "__main__":
    unittest.main()
