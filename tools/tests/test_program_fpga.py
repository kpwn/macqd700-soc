#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PROGRAM_FPGA = REPO_ROOT / "synth" / "program_fpga.sh"


class ProgramFpgaTests(unittest.TestCase):
    def run_program(self, *args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
        merged_env = os.environ.copy()
        merged_env["VIVADO"] = "/bin/echo"
        if env:
            merged_env.update(env)
        return subprocess.run(
            [str(PROGRAM_FPGA), *args],
            cwd=REPO_ROOT,
            env=merged_env,
            text=True,
            capture_output=True,
            check=False,
        )

    def touch(self, path: Path, text: str = "x") -> None:
        path.write_text(text)

    def test_requires_matching_ltx_for_debug_programming(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)
            bit = td_path / "fpga_top.bit"
            self.touch(bit)
            result = self.run_program("--bit", str(bit), "--ltx", str(td_path / "missing.ltx"))

        self.assertEqual(result.returncode, 1)
        self.assertIn("Debug-capable programming now requires a matching .ltx", result.stderr)

    def test_no_vio_allows_bitstream_only_programming(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)
            bit = td_path / "fpga_top.bit"
            self.touch(bit)
            result = self.run_program("--bit", str(bit), "--ltx", str(td_path / "missing.ltx"), "--no-vio")

        self.assertEqual(result.returncode, 0)
        self.assertIn("mode:       program only", result.stdout)

    def test_manifest_rejects_non_vio_build_without_no_vio(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)
            bit = td_path / "fpga_top.bit"
            ltx = td_path / "fpga_top.ltx"
            buildinfo = td_path / "fpga_top.buildinfo"
            self.touch(bit)
            self.touch(ltx)
            buildinfo.write_text(
                "enable_vio=0\n"
                "host_debug=none\n"
            )
            result = self.run_program("--bit", str(bit), "--ltx", str(ltx))

        self.assertEqual(result.returncode, 1)
        self.assertIn("build manifest says ENABLE_VIO was off", result.stderr)

    def test_manifest_allows_debug_capable_dry_run(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)
            bit = td_path / "fpga_top.bit"
            ltx = td_path / "fpga_top.ltx"
            buildinfo = td_path / "fpga_top.buildinfo"
            self.touch(bit)
            self.touch(ltx)
            buildinfo.write_text(
                "enable_vio=1\n"
                "host_debug=jtag_axi\n"
            )
            result = self.run_program("--bit", str(bit), "--ltx", str(ltx))

        self.assertEqual(result.returncode, 0)
        self.assertIn("mode:       program + VIO dashboard", result.stdout)
        self.assertIn(str(buildinfo), result.stdout)


if __name__ == "__main__":
    unittest.main()
