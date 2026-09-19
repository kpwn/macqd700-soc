from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import rom_frontier_diff  # noqa: E402


class RomFrontierDiffTests(unittest.TestCase):
    def test_build_rtl_make_args_binds_comparison_knobs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            rtl_root = Path(tmp) / "rtl"
            args = rom_frontier_diff.build_rtl_make_args(
                rom=Path("files/420dbff3.rom"),
                rtl_root=rtl_root,
                stop_pc=0x408005B0,
                max_insts=128,
                lastn_cycles=64,
                periph_limit=8,
                periph_filter="VIA1,DAFB",
                stop_on_exc="2,4,11",
                sample_every=0,
            )

        self.assertEqual(args[0:2], ["make", "tb-rom-boot"])
        self.assertIn("ROM=files/420dbff3.rom", args)
        self.assertIn(f"ROMBOOT_OUTPUT_ROOT={rtl_root}", args)
        extra = next(arg for arg in args if arg.startswith("ROMBOOT_EXTRA="))
        self.assertIn("+end_pc=0x408005b0", extra)
        self.assertIn("+stop_on_exc=2,4,11", extra)
        self.assertIn("+max_insts=128", extra)
        self.assertIn("+lastn_trace=64", extra)
        self.assertIn("+periph_event_filter=VIA1,DAFB", extra)
        self.assertIn("+periph_event_log_limit=8", extra)

    def test_build_musashi_make_args_enables_unshared_io_override(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            musashi_root = Path(tmp) / "musashi"
            args = rom_frontier_diff.build_musashi_make_args(
                rom=Path("files/420dbff3.rom"),
                musashi_root=musashi_root,
                stop_pc=0x408005B0,
                max_insts=256,
                periph_limit=12,
                sample_every=0,
            )

        self.assertEqual(args[0:2], ["make", "musashi-rom-boot"])
        self.assertIn(f"ROMBOOT_OUTPUT_ROOT={musashi_root}", args)
        self.assertIn("MUSASHI_ROM_BOOT_MAX=256", args)
        extra = next(arg for arg in args if arg.startswith("MUSASHI_ROM_BOOT_ARGS="))
        self.assertIn("--stop-pc 0x408005b0", extra)
        self.assertIn("--stop-pc-hit 1", extra)
        self.assertIn("--allow-unshared-io", extra)
        self.assertIn("--periph-log-limit 12", extra)

    def test_sample_every_wires_both_arch_sample_logs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rtl_args = rom_frontier_diff.build_rtl_make_args(
                rom=Path("files/420dbff3.rom"),
                rtl_root=root / "rtl",
                stop_pc=0x408005B0,
                max_insts=128,
                lastn_cycles=64,
                periph_limit=8,
                periph_filter="VIA1",
                stop_on_exc="2,4,11",
                sample_every=16,
            )
            musashi_args = rom_frontier_diff.build_musashi_make_args(
                rom=Path("files/420dbff3.rom"),
                musashi_root=root / "musashi",
                stop_pc=0x408005B0,
                max_insts=128,
                periph_limit=8,
                sample_every=16,
            )

        rtl_extra = next(arg for arg in rtl_args if arg.startswith("ROMBOOT_EXTRA="))
        musashi_extra = next(
            arg for arg in musashi_args if arg.startswith("MUSASHI_ROM_BOOT_ARGS=")
        )

        self.assertIn("+arch_sample_every=16", rtl_extra)
        self.assertIn(f"+arch_sample_log={root / 'rtl' / 'rtl_arch_sample.tsv'}", rtl_extra)
        self.assertIn("--sample-every 16", musashi_extra)
        self.assertIn(
            f"--sample-log {root / 'musashi' / 'musashi_rom_boot_sample.tsv'}",
            musashi_extra,
        )

    def test_parse_musashi_summary_extracts_stop_line_and_bus_block(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "run.log"
            log.write_text(
                "\n".join(
                    [
                        "[musashi-rom-boot] stop reason=stop-pc pc=0x408005b0 hit=1 committed=12 cycles=34 pc=0x408005b0 sr=0x2700",
                        "",
                        "-------- musashi-rom-boot bus activity --------",
                        "[musashi-rom-boot]   via1 r=3 w=1",
                        "[musashi-rom-boot]   rom r=9 w=0",
                        "-----------------------------------------------",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = rom_frontier_diff.parse_musashi_summary(log)

        self.assertEqual(summary.reason, "stop-pc pc=0x408005b0 hit=1")
        self.assertEqual(summary.committed, 12)
        self.assertEqual(summary.cycles, 34)
        self.assertEqual(summary.pc, 0x408005B0)
        self.assertEqual(summary.sr, 0x2700)
        self.assertEqual(summary.bus_block[0], "-------- musashi-rom-boot bus activity --------")
        self.assertIn("via1 r=3 w=1", summary.bus_block[1])
        self.assertEqual(
            rom_frontier_diff.summarize_musashi(summary),
            "reason=stop-pc pc=0x408005b0 hit=1 committed=12 cycles=34 pc=0x408005b0 sr=0x2700",
        )

    def test_trace_diff_command_can_request_pc_only_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cmd = rom_frontier_diff.trace_diff_command(
                root,
                root / "rtl.tr",
                root / "mame.tr",
                context=8,
                pc_only=True,
            )

        self.assertIn(str(root / "rtl.tr"), cmd)
        self.assertIn(str(root / "mame.tr"), cmd)
        self.assertIn("--context", cmd)
        self.assertIn("8", cmd)
        self.assertIn("--pc-only", cmd)


if __name__ == "__main__":
    unittest.main()
