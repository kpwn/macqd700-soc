#!/usr/bin/env python3
"""Execute the README's dd/cmp commands on synthetic regular files only."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


README = Path(__file__).resolve().parents[1] / "README.md"


class ReadmeSdLayoutTest(unittest.TestCase):
    def commands(self):
        lines = README.read_text().splitlines()
        commands = [line.removeprefix("sudo ") for line in lines
                    if line.startswith(("sudo dd ", "sudo cmp "))]
        self.assertEqual(len(commands), 4, "expected two writes and two read-back checks")
        return commands

    def run_recipe(self, commands):
        with tempfile.TemporaryDirectory(prefix="soc-readme-sd-") as directory:
            root = Path(directory)
            rom = bytes(range(256)) * 4096  # synthetic 1 MiB, not firmware
            hdd = bytes(reversed(range(256))) * 14  # seven sectors
            card_size = 4194304 + len(hdd) + 512
            card = root / "card.img"
            card.write_bytes(b"\xa5" * card_size)
            (root / "rom.bin").write_bytes(rom)
            (root / "hdd.img").write_bytes(hdd)
            env = dict(os.environ, sd_device=str(card), rom_image=str(root / "rom.bin"),
                       hdd_image=str(root / "hdd.img"), hdd_bytes=str(len(hdd)))
            for command in commands:
                subprocess.run(["bash", "-eu", "-c", command], env=env,
                               check=True, capture_output=True)
            actual = card.read_bytes()
            self.assertEqual(len(actual), card_size, "must not truncate the card")
            self.assertEqual(actual[:1048576], rom)
            self.assertEqual(actual[1048576:4194304], b"\xa5" * (4194304 - 1048576),
                             "reserved region, including PRAM, must survive")
            self.assertEqual(actual[4194304:4194304 + len(hdd)], hdd)
            self.assertEqual(actual[-512:], b"\xa5" * 512)

    def test_documented_commands(self):
        self.run_recipe(self.commands())

    def test_wrong_hdd_offset_is_detected(self):
        commands = [line.replace("seek=8192", "seek=0") for line in self.commands()]
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_recipe(commands)


if __name__ == "__main__":
    unittest.main()
