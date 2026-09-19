from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import mame_trace_normalize  # noqa: E402


class MameTraceNormalizeTests(unittest.TestCase):
    def test_parse_prefixed_trace_with_opcode(self) -> None:
        line = "PC=0000009C SR=2700 IR=4E70 0000009C: reset"

        self.assertEqual(
            mame_trace_normalize.parse_line(line),
            (0x0000009C, 0x4E70, 0x00),
        )

    def test_parse_packaged_mame_trace_without_opcode_when_allowed(self) -> None:
        line = "PC=0000008C SR=2704 0000008C: move    #$2700, SR"

        self.assertIsNone(mame_trace_normalize.parse_line(line))
        self.assertEqual(
            mame_trace_normalize.parse_line(line, allow_missing_ir=True),
            (0x0000008C, 0x0000, 0x04),
        )

    def test_cli_allow_missing_ir_emits_pc_only_compatible_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "mame.tr"
            dst = Path(tmp) / "norm.tr"
            src.write_text(
                "PC=0000008C SR=2704 0000008C: move    #$2700, SR\n",
                encoding="utf-8",
            )

            rc = mame_trace_normalize.main_args_for_test(
                [str(src), "-o", str(dst), "--allow-missing-ir"]
            )

            self.assertEqual(rc, 0)
            self.assertIn("0000008c 0000 04", dst.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
