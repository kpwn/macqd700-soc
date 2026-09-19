#!/usr/bin/env python3
import unittest
from check_publication import path_problem, SECRET


class PublicationCheckTest(unittest.TestCase):
    def test_generated_and_firmware_rejected(self):
        for name in ("logs/run.txt", "build/foo.cpp", "rtl/mac/adb_pic_fw.hex",
                     "files/342s0440-b.bin", "files/pmuv2.bin", "renamed.rom",
                     "release/image.bit", "unknown.bin"):
            with self.subTest(name=name):
                self.assertIsNotNone(path_problem(name))

    def test_sources_and_regression_fixtures_kept(self):
        for name in ("rtl/mac/adb_pic_modem.v", "files/ddr4.xdc",
                     "tb/fuzz_fails/example.bin", "tb/vectors/example.csv",
                     "vendor/rk5-eth/third_party/taxi/LICENSE"):
            with self.subTest(name=name):
                self.assertIsNone(path_problem(name))

    def test_secret_signature(self):
        self.assertIsNotNone(SECRET.search(b"-----BEGIN " + b"OPENSSH PRIVATE KEY-----"))
        self.assertIsNone(SECRET.search(b"a normal module comment"))


if __name__ == "__main__":
    unittest.main()
