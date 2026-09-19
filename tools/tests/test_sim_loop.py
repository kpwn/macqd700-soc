#!/usr/bin/env python3
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import sim_loop  # noqa: E402


class SimLoopTests(unittest.TestCase):
    def test_build_invocation_uses_standard_env(self) -> None:
        invocation = sim_loop.build_invocation(Path("/tmp/repo"), ["test", "TEST=moveq"])

        self.assertEqual(invocation.command, ["make", "test", "TEST=moveq"])
        self.assertEqual(invocation.env_overrides["VERILATOR_THREADS"], "4")
        self.assertEqual(invocation.env_overrides["VERILATOR_JOBS"], "4")
        self.assertEqual(invocation.env_overrides["MAKEFLAGS"], "-j1")
        self.assertIn("MAKEFLAGS=-j1 make test TEST=moveq", invocation.display())

    def test_shm_adds_build_trace_and_tmp_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            invocation = sim_loop.build_invocation(
                Path("/work/m68k-ooo-worktree"),
                ["tb-rom-boot"],
                shm=True,
                shm_root=Path(tmp),
            )

            self.assertIn(f"BUILD_DIR={tmp}/m68k-ooo-worktree/build", invocation.command)
            self.assertIn(
                f"ROMBOOT_OUTPUT_ROOT={tmp}/m68k-ooo-worktree/romboot",
                invocation.command,
            )
            self.assertEqual(invocation.env_overrides["TMPDIR"], f"{tmp}/m68k-ooo-worktree/tmp")
            self.assertTrue(Path(tmp, "m68k-ooo-worktree", "tmp").is_dir())

    def test_shm_does_not_override_explicit_make_vars(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            invocation = sim_loop.build_invocation(
                Path("/work/repo"),
                ["test", "BUILD_DIR=/custom/build", "ROMBOOT_OUTPUT_ROOT=/custom/rom"],
                shm=True,
                shm_root=Path(tmp),
            )

            self.assertIn("BUILD_DIR=/custom/build", invocation.command)
            self.assertIn("ROMBOOT_OUTPUT_ROOT=/custom/rom", invocation.command)
            self.assertNotIn(f"BUILD_DIR={tmp}/repo/build", invocation.command)

    def test_shm_can_skip_directory_creation_for_dry_run(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            invocation = sim_loop.build_invocation(
                Path("/work/repo"),
                ["tb-rom-boot"],
                shm=True,
                shm_root=Path(tmp),
                create_shm_dirs=False,
            )

            self.assertIn(f"BUILD_DIR={tmp}/repo/build", invocation.command)
            self.assertFalse(Path(tmp, "repo").exists())

    def test_blocks_non_sim_targets_by_default(self) -> None:
        with self.assertRaises(sim_loop.UsageError):
            sim_loop.build_invocation(Path("/tmp/repo"), ["impl"])

        allowed = sim_loop.build_invocation(Path("/tmp/repo"), ["impl"], allow_non_sim=True)
        self.assertEqual(allowed.target, "impl")

    def test_extract_summary_from_make_output(self) -> None:
        lines = [
            "noise\n",
            "summary: PASS=3 DEFER=1 FAIL=0 SKIP=0\n",
            "[sim-loop] trailer\n",
        ]

        self.assertEqual(
            sim_loop.extract_summary_from_lines(lines),
            "summary: PASS=3 DEFER=1 FAIL=0 SKIP=0",
        )

    def test_record_and_load_invocation(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            state_dir = Path(tmp)
            invocation = sim_loop.build_invocation(
                Path("/tmp/repo"),
                ["tb-alu"],
                state_dir=state_dir,
                log_path=state_dir / "logs" / "run.log",
            )

            sim_loop.record_invocation(state_dir, "last-fail", invocation)
            loaded = sim_loop.load_invocation(state_dir, "last-fail")

            self.assertEqual(loaded.repo_root, Path("/tmp/repo"))
            self.assertEqual(loaded.command, ["make", "tb-alu"])
            self.assertTrue((state_dir / "last-fail.sh").exists())

    def test_render_rom_lastn_summary_from_log_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            lastn = root / "rom_boot_lastn.log"
            lastn.write_text(
                "# rom-boot last-N-cycle trace capacity=1 samples=1 "
                "reason=no-progress cycles=8 threshold=8\n"
                "lastn[00000] sim=1 committed=2 dbg_last_pc=0x40800888 "
                "dbg_pc=0x40800888 rob_v=1 rob_c=0 rob_pc=0x40800888 "
                "rob_vec=0 commit_exc_wait=0 commit_take_exc=0 exc_state=0 "
                "exc_vec=0 exc_fault_pc=0x00000000 exc_a7=0x00000000 "
                "daxi_ar=1/0/0x50f04000 daxi_r=0/1/0 daxi_aw=0/1/0x00000000 "
                "daxi_w=0/1 daxi_b=0/1/0 if=0/0x40800888/0/0\n",
                encoding="utf-8",
            )
            log = root / "run.log"
            log.write_text(
                f"[rom-boot] last-N trace at {lastn.name}\n",
                encoding="utf-8",
            )

            rendered = sim_loop.render_rom_lastn_summary(root, log)

        self.assertIsNotNone(rendered)
        self.assertIn("diagnosis=bus-read-address", rendered or "")
        self.assertIn("region=q700-io", rendered or "")


if __name__ == "__main__":
    unittest.main()
