"""Smoke test the ASC WAV exporter against a tiny synthetic event log."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
import wave
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[3] / "tools" / "asc_wav_from_periph_log.py"


class AscWavFromPeriphLogTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="asc-wav-export-")
        self.root = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def _write_log(self, text: str) -> Path:
        path = self.root / "asc_events.log"
        path.write_text(text, encoding="utf-8")
        return path

    def _run_export(
        self, log: Path, wav: Path, *extra: str
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(SCRIPT), str(log), str(wav), *extra],
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )

    def test_exporter_turns_fifo_writes_into_wav(self):
        log = self._write_log(
            "\n".join(
                [
                    "# peripheral model event log",
                    "# fields: cycle committed pc category event addr value detail",
                    "cycle=10 committed=1 pc=0x40000000 category=ASC event=write "
                    "addr=0x50014000 value=0x00000080 detail=off=0x000,reg=FIFO_A",
                    "cycle=11 committed=2 pc=0x40000002 category=ASC event=write "
                    "addr=0x50014400 value=0x00000040 detail=off=0x400,reg=FIFO_B",
                    "cycle=12 committed=3 pc=0x40000004 category=ASC event=write "
                    "addr=0x50014803 value=0x00000080 detail=off=0x803,reg=FIFO_CONTROL",
                    "cycle=13 committed=4 pc=0x40000006 category=ASC event=write "
                    "addr=0x50014000 value=0x000000c0 detail=off=0x000,reg=FIFO_A",
                    "cycle=14 committed=5 pc=0x40000008 category=ASC event=write "
                    "addr=0x50014400 value=0x00000020 detail=off=0x400,reg=FIFO_B",
                    "",
                ]
            )
        )
        wav_path = self.root / "asc.wav"

        result = self._run_export(log, wav_path)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(wav_path.is_file())

        with wave.open(str(wav_path), "rb") as wf:
            self.assertEqual(wf.getnchannels(), 2)
            self.assertEqual(wf.getsampwidth(), 2)
            self.assertEqual(wf.getframerate(), 22222)
            self.assertEqual(wf.getnframes(), 1)
            self.assertEqual(wf.readframes(1), b"\x00@\x00\xa0")

    def test_exporter_accepts_clock_select_rate_alias(self):
        log = self._write_log(
            "\n".join(
                [
                    "# peripheral model event log",
                    "# fields: cycle committed pc category event addr value detail",
                    "cycle=20 committed=1 pc=0x40000000 category=ASC event=write "
                    "addr=0x50014807 value=0x00000003 detail=off=0x807,reg=CLOCK_SELECT",
                    "cycle=21 committed=2 pc=0x40000002 category=ASC event=write "
                    "addr=0x50014000 value=0x000000ff detail=off=0x000,reg=FIFO_A",
                    "",
                ]
            )
        )
        wav_path = self.root / "rate.wav"

        result = self._run_export(log, wav_path)
        self.assertEqual(result.returncode, 0, result.stderr)

        with wave.open(str(wav_path), "rb") as wf:
            self.assertEqual(wf.getnchannels(), 1)
            self.assertEqual(wf.getframerate(), 43478)
            self.assertEqual(wf.getnframes(), 1)
            self.assertEqual(wf.readframes(1), b"\x00\x7f")


if __name__ == "__main__":
    unittest.main()
