from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import rom_arch_sample_compare  # noqa: E402


HEADER = (
    "# columns: side committed cycles pc sr ccr "
    "d0 d1 d2 d3 d4 d5 d6 d7 "
    "a0 a1 a2 a3 a4 a5 a6 a7 usp isp sfc dfc vbr\n"
)


def row(side: str, committed: int, pc: str, d0: str = "0x00000000") -> str:
    regs = [d0] + ["0x00000000"] * 7 + ["0x00000000"] * 8
    tail = ["0x00000000"] * 5
    return (
        f"{side} {committed} {committed * 4} {pc} 0x2700 0x00 "
        + " ".join(regs + tail)
        + "\n"
    )


class RomArchSampleCompareTests(unittest.TestCase):
    def test_parse_and_pass_matching_samples(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            rtl = Path(tmp) / "rtl.tsv"
            musashi = Path(tmp) / "musashi.tsv"
            data = HEADER + row("rtl", 0, "0x4000002a") + row("rtl", 2, "0x40800090")
            rtl.write_text(data, encoding="utf-8")
            musashi.write_text(
                data.replace("rtl", "musashi"),
                encoding="utf-8",
            )

            diffs = rom_arch_sample_compare.compare_samples(
                rom_arch_sample_compare.parse_sample_log(rtl),
                rom_arch_sample_compare.parse_sample_log(musashi),
                ["pc", "sr", "d0"],
                max_diffs=10,
            )

        self.assertEqual(diffs, [])

    def test_reports_first_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            rtl = Path(tmp) / "rtl.tsv"
            musashi = Path(tmp) / "musashi.tsv"
            rtl.write_text(HEADER + row("rtl", 4, "0x40800090"), encoding="utf-8")
            musashi.write_text(
                HEADER + row("musashi", 4, "0x40800094"),
                encoding="utf-8",
            )

            diffs = rom_arch_sample_compare.compare_samples(
                rom_arch_sample_compare.parse_sample_log(rtl),
                rom_arch_sample_compare.parse_sample_log(musashi),
                ["pc"],
                max_diffs=10,
            )

        self.assertEqual(
            diffs,
            ["4: pc rtl=0x40800090 musashi=0x40800094"],
        )


if __name__ == "__main__":
    unittest.main()
