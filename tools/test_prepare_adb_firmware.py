#!/usr/bin/env python3
"""Synthetic fixtures only: no Apple firmware bytes."""
import struct
import unittest

from prepare_adb_firmware import convert


class FirmwareConversionTest(unittest.TestCase):
    def test_word_order_and_padding(self):
        words = [(i * 13) & 0xFFF for i in range(512)]
        result = convert(struct.pack("<512H", *words))
        self.assertEqual(result.splitlines(), [f"{w:03x}" for w in words])
        self.assertTrue(result.endswith("\n"))

    def test_wrong_sizes(self):
        for size in (0, 512, 1023, 1025, 2048):
            with self.subTest(size=size), self.assertRaises(ValueError):
                convert(bytes(size))

    def test_invalid_word(self):
        with self.assertRaises(ValueError):
            convert(struct.pack("<512H", 0x1000, *([0] * 511)))


if __name__ == "__main__":
    unittest.main()
