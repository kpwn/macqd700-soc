#!/usr/bin/env python3
"""Board-free tests for tools/board_ab.py (OCR, screen classifier, REPL parsing,
and the whole harness replayed against recorded fixtures with --dry-run)."""
from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import board_ab  # noqa: E402
from PIL import Image  # noqa: E402

FX = ROOT / "tools" / "board_ab_fixtures"


class OcrTests(unittest.TestCase):
    def test_every_labelled_cell_reads_exactly(self) -> None:
        ok, n, fails = board_ab.selftest(str(FX / "labels.json"))
        self.assertEqual(fails, [])
        self.assertEqual(ok, n)
        self.assertGreater(n, 1000)

    def test_i2_matches_recorded_board_numbers(self) -> None:
        r = board_ab.read_benchmark(str(FX / "speedo_i2.jpg"))
        self.assertEqual(r["rows"]["Dhrystones"]["abs"], 120220.917)
        self.assertEqual(r["rows"]["Sieve"]["rating"], 13.701)
        self.assertEqual(r["average"], 8.110)
        self.assertTrue(board_ab.ocr_ok(r))
        self.assertTrue(r["complete"])
        self.assertFalse(r["alert"])

    def test_alert_frame_reads_abs_and_flags_alert(self) -> None:
        r = board_ab.read_benchmark(str(FX / "screen_alert.jpg"))
        self.assertTrue(r["alert"])
        self.assertTrue(r["complete_abs"])
        self.assertIsNone(r["rows"]["Towers"]["rating"])  # covered -> unread, never guessed

    def test_running_marker(self) -> None:
        r = board_ab.read_benchmark(str(FX / "screen_running.jpg"))
        self.assertEqual(r["running_row"], "IntMatrix")

    def test_1080p_capture_mode(self) -> None:
        src = Image.open(FX / "speedo_i2.jpg").convert("RGB").crop((320, 120, 960, 600))
        for rs in (Image.NEAREST, Image.BILINEAR):
            big = Image.new("RGB", (1920, 1080))
            big.paste(src.resize((960, 720), rs), (480, 180))
            buf = io.BytesIO()
            big.save(buf, "JPEG", quality=85)
            r = board_ab.read_benchmark(buf.getvalue())
            self.assertEqual(r["average"], 8.110)
            self.assertEqual(r["rows"]["Dhrystones"]["abs"], 120220.917)

    def test_checks_catch_a_misread_digit(self) -> None:
        r = board_ab.read_benchmark(str(FX / "speedo_i2.jpg"))
        r["rows"]["Puzzle"]["rating"] = 8.673          # 9 -> 8
        self.assertFalse(board_ab.check_values(r)["Puzzle"])
        self.assertFalse(board_ab.check_values(r)["Average"])

    def test_no_window(self) -> None:
        r = board_ab.read_benchmark(str(FX / "screen_finder.jpg"))
        self.assertFalse(r["window"])


class ParseTests(unittest.TestCase):
    def test_parse_perf(self) -> None:
        p = board_ab.parse_perf((FX / "perf_sample.txt").read_text())
        c = p["counters"]
        self.assertEqual(c["window_cycles"], 38_351_506_215)
        self.assertEqual(c["retired_macro_insts"], 5_128_236_571)
        self.assertEqual(c["stall_cycles_walk"], 3_254_695_374)
        self.assertEqual(p["n_counters"], 18)
        self.assertAlmostEqual(p["derived"]["ipc"], 0.1337, places=4)
        self.assertAlmostEqual(p["derived"]["branch_mispredicts_per_kinst"], 99.454, places=2)
        self.assertEqual(p["free_cycles"], 38_355_304_283)

    def test_parse_misc(self) -> None:
        self.assertEqual(board_ab.parse_build_id("> programmed x\n> jtag_repl: build_id = 0xDC31F130\n"),
                         "0xDC31F130")
        self.assertEqual(board_ab.parse_exc_count("> exc_count = 0x01f07745\n"), 0x01F07745)
        h = board_ab.parse_halt("> halt: reason=0x0 pc_live=0x4080dd54 manual=0 dbl_fault=1 effective=0")
        self.assertEqual(h["dbl_fault"], 1)
        self.assertEqual(board_ab.parse_exc_ring(
            "> exc[30] vec=0x02 pc=0x0 fa=0x40900000 handler=0x4080dbfe\n"
            "> exc[ 9] vec=0x0a pc=0x311f8 fa=0x0 handler=0x408099b0\n"), [2, 10])


class ReplTransportTests(unittest.TestCase):
    """Repl against a fake REPL: FIFO reader appending '> READY' to an out file."""

    def test_fifo_roundtrip_and_lock(self) -> None:
        import os
        import threading
        td = tempfile.TemporaryDirectory(prefix="board_ab_repl_")
        self.addCleanup(td.cleanup)
        d = Path(td.name)
        fifo, out, lock = d / "in", d / "out", d / "lock"
        os.mkfifo(fifo)
        out.write_text("old noise\n> READY\n")
        seen = []

        def serve():
            with open(fifo) as fh:          # blocks until the writer opens
                for line in fh:
                    seen.append(line.strip())
                    with open(out, "a") as o:
                        o.write(f"> exc_count = 0x0000002a\n> READY\n")
                    return
        th = threading.Thread(target=serve, daemon=True)
        th.start()
        import time
        time.sleep(0.2)
        r = board_ab.Repl(fifo=str(fifo), out=str(out), lock=str(lock), log=str(d / "log"))
        txt = r.cmd("exc-count", timeout=5)
        th.join(2)
        self.assertEqual(seen, ["exc-count"])
        self.assertEqual(board_ab.parse_exc_count(txt), 42)
        self.assertNotIn("old noise", txt)
        self.assertIn("board_ab", lock.read_text())


class DryRunTests(unittest.TestCase):
    def _run(self, *extra):
        td = tempfile.TemporaryDirectory(prefix="board_ab_test_")
        self.addCleanup(td.cleanup)
        tmp = td.name
        out = io.StringIO()
        with redirect_stdout(out):
            rc = board_ab.main(["run", "/nonexistent/fpga_top.local.bit", "/nonexistent/fpga_top.ltx",
                                "dry", "--dry-run", "--outdir", tmp, *extra])
        return rc, json.loads((Path(tmp) / "result.json").read_text()), out.getvalue(), tmp

    def test_full_cycle(self) -> None:
        rc, res, _out, tmp = self._run("--runs", "2", "--idle", "1")
        self.assertEqual(rc, 0)
        self.assertEqual(res["status"], "ok")
        self.assertEqual(res["build_id"], "0xDC31F130")
        self.assertEqual(len(res["runs"]), 2)
        for run in res["runs"]:
            self.assertTrue(run["ocr_ok"])
            self.assertTrue(run["alert_seen"])          # alert dismissed and re-read
            self.assertEqual(run["average"], 3.683)
            self.assertEqual(run["perf"]["clock_mhz"], 200.0)
            self.assertIn("ipc", run["perf"]["derived"])
            self.assertTrue((Path(tmp) / run["snapshot"]).exists())
        self.assertEqual(len(res["idle"]), 1)
        self.assertEqual(res["summary"]["rows"]["Sieve"]["abs_mean"], 0.202)

    def test_sadmac_fails_fast(self) -> None:
        rc, res, _o, _t = self._run("--dry-scenario", "sadmac")
        self.assertEqual(rc, 2)
        self.assertIn("sadmac", res["status"])

    def test_exception_storm_fails_fast(self) -> None:
        rc, res, _o, _t = self._run("--dry-scenario", "storm")
        self.assertEqual(rc, 2)
        self.assertIn("exception storm", res["status"])

    def test_compare_one_line(self) -> None:
        _rc, _res, _o, tmp = self._run("--runs", "1", "--idle", "0")
        ref = Path(tmp) / "ref.json"
        ocr = board_ab.read_benchmark(str(FX / "speedo_i2.jpg"))
        ref.write_text(json.dumps({"label": "i2", "rows": ocr["rows"], "average": ocr["average"]}))
        out = io.StringIO()
        with redirect_stdout(out):
            board_ab.main(["compare", str(ref), str(Path(tmp) / "result.json")])
        line = [l for l in out.getvalue().splitlines() if l.startswith("ONE-LINE")][0]
        self.assertIn("AVG -54.6%", line)     # 3.683 vs 8.110


if __name__ == "__main__":
    unittest.main()
