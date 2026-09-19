from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "rom_trace_diff.py"


class RomTraceDiffTests(unittest.TestCase):
    def test_pc_only_ignores_ir_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ours = Path(tmp) / "ours.tr"
            theirs = Path(tmp) / "theirs.tr"
            ours.write_text("0000008c 46fc 04\n", encoding="utf-8")
            theirs.write_text("0000008c 0000 04\n", encoding="utf-8")

            proc = subprocess.run(
                [sys.executable, str(SCRIPT), str(ours), str(theirs), "--pc-only"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )

            self.assertEqual(proc.returncode, 0, proc.stdout)
            self.assertIn("FULL MATCH", proc.stdout)

    def test_default_mode_still_checks_ir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ours = Path(tmp) / "ours.tr"
            theirs = Path(tmp) / "theirs.tr"
            ours.write_text("0000008c 46fc 04\n", encoding="utf-8")
            theirs.write_text("0000008c 0000 04\n", encoding="utf-8")

            proc = subprocess.run(
                [sys.executable, str(SCRIPT), str(ours), str(theirs)],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )

            self.assertEqual(proc.returncode, 1)
            self.assertIn("FIRST DIVERGENCE", proc.stdout)

    def test_prefix_match_requires_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ours = Path(tmp) / "ours.tr"
            theirs = Path(tmp) / "theirs.tr"
            ours.write_text(
                "0000008c 46fc 04\n00000090 4dfa 00\n00000094 6000 00\n",
                encoding="utf-8",
            )
            theirs.write_text(
                "0000008c 46fc 04\n00000090 4dfa 00\n00000096 6000 00\n",
                encoding="utf-8",
            )

            strict = subprocess.run(
                [sys.executable, str(SCRIPT), str(ours), str(theirs), "--min-match", "2"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            prefix = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(ours),
                    str(theirs),
                    "--min-match",
                    "2",
                    "--allow-prefix-match",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )

            self.assertEqual(strict.returncode, 1, strict.stdout)
            self.assertEqual(prefix.returncode, 0, prefix.stdout)


if __name__ == "__main__":
    unittest.main()
