#!/usr/bin/env python3
"""Synthetic packet/map tests. No firmware dumps or vendor executables."""
import copy
import hashlib
import json
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import unittest
import zipfile
from unittest.mock import patch as mock_patch

from adb_config import ConfigImage, FORMAT, FRAME_WORDS, insert_firmware, validate_map
from build_adb_patch_map import changed_bits, identify_locations, pattern_firmware
from patch_adb_bitstream import patch
from package_adb_patcher import package


def slow_crc(words, packets):
    """Independent bit-serial reference for the optimized runtime CRC."""
    crc = 0
    for register, begin, end in packets:
        for index in range(begin, end):
            value = words[index]
            if register == 0:
                words[index], crc = crc, 0
            elif register == 4 and value == 7:
                crc = 0
            else:
                value |= register << 32
                for bit in range(37):
                    crc = (crc >> 1) ^ (0x82F63B78 if (crc ^ (value >> bit)) & 1 else 0)


def fixture():
    # Three miniature frames and two CRC writes exercise frame boundaries,
    # every firmware bit, multiple CRC accumulators and a reversed bit map.
    words = [0x20000000, 0x30018001, 0x04A62093,
             0x30008001, 7, 0x30004000, 0x50000000 | (3 * FRAME_WORDS)]
    begin = len(words)
    words += [0] * (3 * FRAME_WORDS)
    words += [0x30000001, 0, 0x30002001, 0x01234567,
              0x30008001, 13, 0x30000001, 0, 0x20000000, 0xFFFFFFFF]
    image = ConfigImage(b"synthetic header" + bytes.fromhex("aa995566")
                        + struct.pack(f">{len(words)}I", *words))
    slow_crc(image.words, image.packets)
    locations = [word * 32 + bit for word in range(begin, begin + 3 * FRAME_WORDS)
                 if (word - begin) % FRAME_WORDS not in (45, 46, 47)
                 for bit in range(32)][:6144][::-1]
    data = image.bytes()
    mapping = {"format": FORMAT, "bit_sha256": hashlib.sha256(data).hexdigest(),
               "bit_locations": locations}
    return data, mapping


class ConfigTest(unittest.TestCase):
    def setUp(self):
        self.data, self.mapping = fixture()

    def test_zero_is_byte_identical(self):
        self.assertEqual(insert_firmware(self.data, self.mapping, bytes(1024)), self.data)

    def test_all_bits_and_non_pic_preservation(self):
        result = ConfigImage(insert_firmware(self.data, self.mapping, b"\xff\x0f" * 512))
        before = ConfigImage(self.data)
        self.assertEqual(changed_bits(before, result), set(self.mapping["bit_locations"]))
        result.crc()
        self.assertEqual(result.header, before.header)

    def test_word_endianness_boundaries_and_each_instruction_bit(self):
        for address in (0, 7, 8, 255, 256, 511):
            for bit in range(12):
                words = [0] * 512
                words[address] = 1 << bit
                result = ConfigImage(insert_firmware(
                    self.data, self.mapping, struct.pack("<512H", *words)))
                self.assertEqual(changed_bits(ConfigImage(self.data), result),
                                 {self.mapping["bit_locations"][address * 12 + bit]})

    def test_randomized_firmware_against_bit_serial_crc(self):
        rng = random.Random(0xADB040)
        for _ in range(20):
            firmware = [rng.randrange(4096) for _ in range(512)]
            expected = ConfigImage(self.data)
            for logical, location in enumerate(self.mapping["bit_locations"]):
                if firmware[logical // 12] & (1 << (logical % 12)):
                    expected.words[location // 32] ^= 1 << (location % 32)
            slow_crc(expected.words, expected.packets)
            self.assertEqual(insert_firmware(self.data, self.mapping,
                             struct.pack("<512H", *firmware)), expected.bytes())

    def test_hash_mismatch(self):
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            validate_map(self.data + b"\0\0\0\0", self.mapping)

    def test_invalid_maps(self):
        cases = (None, {}, {**self.mapping, "format": "future"},
                 {**self.mapping, "bit_locations": []},
                 {**self.mapping, "bit_locations": [0] * 6144},
                 {**self.mapping, "bit_locations": ["bad"] * 6144})
        for mapping in cases:
            with self.subTest(mapping_type=type(mapping)), self.assertRaises(ValueError):
                validate_map(self.data, mapping)

    def test_forbidden_offsets(self):
        image = ConfigImage(self.data)
        start, end = image.fdri
        for location in (-1, 0, end * 32, (start + 45) * 32,
                         (start + 46) * 32, (start + 47) * 32):
            mapping = copy.deepcopy(self.mapping)
            mapping["bit_locations"][0] = location
            with self.subTest(location=location), self.assertRaises(ValueError):
                validate_map(self.data, mapping)

    def test_nonblank_data_rejected_even_with_updated_hash(self):
        data = insert_firmware(self.data, self.mapping, b"\xff\x0f" * 512)
        mapping = {**self.mapping, "bit_sha256": hashlib.sha256(data).hexdigest()}
        with self.assertRaisesRegex(ValueError, "not blank"):
            validate_map(data, mapping)

    def test_bad_crc_rejected_even_with_updated_hash(self):
        image = ConfigImage(self.data)
        image.words[image.crc_indices[0]] ^= 1
        data = image.bytes()
        mapping = {**self.mapping, "bit_sha256": hashlib.sha256(data).hexdigest()}
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            validate_map(data, mapping)

    def test_malformed_and_unsupported_packets(self):
        for data in (b"", self.data[:-1], self.data[:50],
                     self.data.replace(b"\x30\x00\x40\x00", b"\x30\x01\x40\x00"),
                     self.data.replace(b"\x04\xa6\x20\x93", b"\x04\xa6\x20\x92"),
                     self.data.replace(b"\x50\x00\x01\x17", b"\x50\x00\x01\x16")):
            with self.subTest(length=len(data)), self.assertRaises(ValueError):
                ConfigImage(data)
        # Compression and encryption control writes must not be accepted even
        # before their payloads expose the unsupported format.
        for register, value in ((24, 0x1000), (5, 0x40)):
            packet = struct.pack(">II", 0x30000001 | register << 13, value)
            data = self.data.replace(bytes.fromhex("aa995566"), bytes.fromhex("aa995566") + packet)
            with self.assertRaises(ValueError):
                ConfigImage(data)

    def test_map_discovery_roundtrip(self):
        candidates = set(self.mapping["bit_locations"])
        patterns = [{location for logical, location in enumerate(self.mapping["bit_locations"])
                     if (logical + 1) & (1 << shift)} for shift in range(13)]
        self.assertEqual(identify_locations(candidates, patterns), self.mapping["bit_locations"])
        patterns[0].add(-1)
        with self.assertRaisesRegex(ValueError, "outside"):
            identify_locations(candidates, patterns)

    def test_discovery_rejects_ambiguous_and_ecc_changing_maps(self):
        candidates = set(self.mapping["bit_locations"])
        with self.assertRaises(ValueError):
            identify_locations(candidates | {-1}, [set()] * 13)
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            identify_locations(candidates, [candidates] * 13)

    def test_pattern_encoding(self):
        firmware = pattern_firmware(lambda index: index in (0, 11, 12, 6143))
        words = struct.unpack("<512H", firmware)
        self.assertEqual((words[0], words[1], words[-1]), (0x801, 1, 0x800))

    def test_file_patch_never_invokes_vendor_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bit, manifest, firmware, output = [root / name for name in
                                               ("base.bit", "map.json", "fw.bin", "out.bit")]
            bit.write_bytes(self.data)
            manifest.write_text(json.dumps(self.mapping))
            firmware.write_bytes(bytes(1024))
            with mock_patch("patch_adb_bitstream.subprocess.run", side_effect=AssertionError):
                patch(bit, None, manifest, firmware, output)
            self.assertEqual(output.read_bytes(), self.data)
            with self.assertRaisesRegex(ValueError, "new file"):
                patch(bit, None, manifest, firmware, output)
            self.assertEqual(output.read_bytes(), self.data)

    def test_bad_firmware_leaves_no_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bit, manifest, firmware, output = [root / name for name in
                                               ("base.bit", "map.json", "fw.bin", "out.bit")]
            bit.write_bytes(self.data)
            manifest.write_text(json.dumps(self.mapping))
            for contents in (b"", bytes(1023), bytes(1025), b"\0\x10" + bytes(1022)):
                firmware.write_bytes(contents)
                with self.assertRaises(ValueError):
                    patch(bit, None, manifest, firmware, output)
                self.assertFalse(output.exists())

    def test_standalone_zip_with_empty_path_and_no_site_packages(self):
        with tempfile.TemporaryDirectory(prefix="adb standalone ") as directory:
            root = Path(directory)
            archive = root / "adb-patcher.zip"
            package(archive)
            with zipfile.ZipFile(archive) as contents:
                self.assertEqual(set(contents.namelist()), {"adb-patcher/" + name for name in
                    ("patch_adb_bitstream.py", "adb_config.py", "prepare_adb_firmware.py",
                     "README.md", "LICENSE", "THIRD_PARTY_NOTICES.md")})
                contents.extractall(root)
            (root / "base.bit").write_bytes(self.data)
            (root / "map.json").write_text(json.dumps(self.mapping))
            (root / "fw.bin").write_bytes(bytes(1024))
            result = subprocess.run([
                sys.executable, "-S", "adb-patcher/patch_adb_bitstream.py", "patch",
                "--bit", "base.bit", "--manifest", "map.json", "--firmware", "fw.bin",
                "--output", "out.bit"], cwd=root, env={"PATH": ""},
                text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual((root / "out.bit").read_bytes(), self.data)


if __name__ == "__main__":
    unittest.main()
