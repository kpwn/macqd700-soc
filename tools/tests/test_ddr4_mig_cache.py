#!/usr/bin/env python3
from __future__ import annotations

import contextlib
import io
import tempfile
import unittest
from pathlib import Path

from tools import ddr4_mig_cache as cache


def write_manifest(out_dir: Path, mode: str) -> None:
    paths = cache.artifact_paths(out_dir)
    lines = [
        f"ip={cache.IP_NAME}",
        "part=xcku5p-ffvb676-2-i",
        f"mode={mode}",
        f"output_dir={out_dir.resolve()}",
        f"xci={paths['xci'].resolve()}",
        f"dcp={paths['dcp'].resolve()}",
    ]
    for key, value in cache.MANIFEST_EXPECTED.items():
        if key in {"ip", "part"}:
            continue
        lines.append(f"{key}={value}")
    paths["manifest"].write_text("\n".join(lines) + "\n", encoding="utf-8")


class Ddr4MigCacheTests(unittest.TestCase):
    def make_tree(self, mode: str) -> tuple[tempfile.TemporaryDirectory[str], Path, Path]:
        tmp = tempfile.TemporaryDirectory()
        root = Path(tmp.name)
        out_dir = root / "build" / "ddr4_mig"
        out_dir.mkdir(parents=True)
        generator = root / "synth" / "gen_ddr4_mig.tcl"
        generator.parent.mkdir()
        generator.write_text("set mode synth\n", encoding="utf-8")
        paths = cache.artifact_paths(out_dir)
        paths["xci"].write_text('{"ip_inst": {"component_reference": "ddr4"}}\n',
                                encoding="utf-8")
        if mode == "synth":
            paths["dcp"].write_bytes(b"synthetic dcp\n")
        write_manifest(out_dir, mode)
        return tmp, out_dir, generator

    def test_validate_cache_hits_after_stamp(self) -> None:
        tmp, out_dir, generator = self.make_tree("validate")
        with tmp:
            cache.stamp_cache(out_dir, generator, "validate")
            cache.check_cache(out_dir, generator, "validate")
            with self.assertRaises(cache.CacheMiss):
                cache.check_cache(out_dir, generator, "synth")

    def test_synth_cache_hits_for_synth_and_validate(self) -> None:
        tmp, out_dir, generator = self.make_tree("synth")
        with tmp:
            cache.stamp_cache(out_dir, generator, "synth")
            cache.check_cache(out_dir, generator, "synth")
            cache.check_cache(out_dir, generator, "validate")

    def test_generator_change_invalidates_cache(self) -> None:
        tmp, out_dir, generator = self.make_tree("synth")
        with tmp:
            cache.stamp_cache(out_dir, generator, "synth")
            generator.write_text("set mode validate\n", encoding="utf-8")
            with self.assertRaises(cache.CacheMiss):
                cache.check_cache(out_dir, generator, "synth")

    def test_missing_synth_dcp_invalidates_synth_cache(self) -> None:
        tmp, out_dir, generator = self.make_tree("synth")
        with tmp:
            cache.stamp_cache(out_dir, generator, "synth")
            cache.artifact_paths(out_dir)["dcp"].unlink()
            with self.assertRaises(cache.CacheMiss):
                cache.check_cache(out_dir, generator, "synth")
            cache.check_cache(out_dir, generator, "validate")

    def test_xci_change_invalidates_cache(self) -> None:
        tmp, out_dir, generator = self.make_tree("validate")
        with tmp:
            cache.stamp_cache(out_dir, generator, "validate")
            cache.artifact_paths(out_dir)["xci"].write_text("changed\n", encoding="utf-8")
            with self.assertRaises(cache.CacheMiss):
                cache.check_cache(out_dir, generator, "validate")

    def test_cli_cache_hit_is_one_line(self) -> None:
        tmp, out_dir, generator = self.make_tree("validate")
        with tmp:
            cache.stamp_cache(out_dir, generator, "validate")
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                rc = cache.main([
                    "check",
                    "--mode", "validate",
                    "--dir", str(out_dir),
                    "--generator", str(generator),
                ])
            self.assertEqual(rc, 0)
            lines = stdout.getvalue().splitlines()
            self.assertEqual(len(lines), 1)
            self.assertIn("DDR4 MIG cache hit (validate):", lines[0])
            self.assertIn("skipping Vivado", lines[0])


if __name__ == "__main__":
    unittest.main()
