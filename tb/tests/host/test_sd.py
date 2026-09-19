"""Tests for the SD-card path against MockDevice."""

import os
import tempfile
import unittest
import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[3] / 'tools'
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from m68kctl import MockDevice, SdCard
from m68kctl.provision import upload_rom, verify_rom


class SdSingleBlockTests(unittest.TestCase):
    def test_write_then_read_roundtrip(self):
        dev = MockDevice()
        sd = SdCard(dev)
        self.assertTrue(sd.card_ready)

        payload = bytes(range(256)) * 2   # 512 B
        sd.write_block(42, payload)
        got = sd.read_block(42)
        self.assertEqual(got, payload)

    def test_unwritten_block_reads_zero(self):
        dev = MockDevice()
        sd = SdCard(dev)
        self.assertEqual(sd.read_block(0), b'\x00' * 512)

    def test_wrong_size_write_rejected(self):
        dev = MockDevice()
        sd = SdCard(dev)
        with self.assertRaises(ValueError):
            sd.write_block(0, b'\x00' * 511)

    def test_card_version(self):
        dev = MockDevice()
        sd = SdCard(dev)
        self.assertEqual(sd.version, 0x5D50_0002)


class SdProvisionFlowTests(unittest.TestCase):
    def _make_image(self, n_blocks: int) -> str:
        fd, path = tempfile.mkstemp(suffix='.bin')
        try:
            os.write(fd, os.urandom(n_blocks * 512))
        finally:
            os.close(fd)
        return path

    def test_upload_then_verify_clean(self):
        img = self._make_image(8)
        try:
            dev = MockDevice()
            sd = SdCard(dev)
            n = upload_rom(sd, img)
            self.assertEqual(n, 8)
            bad = verify_rom(sd, img)
            self.assertEqual(bad, [])
        finally:
            os.unlink(img)

    def test_verify_detects_corruption(self):
        img = self._make_image(4)
        try:
            dev = MockDevice()
            sd = SdCard(dev)
            upload_rom(sd, img)
            # Corrupt one block on the fake card.
            dev.sd_blocks[1] = bytes(512)
            bad = verify_rom(sd, img)
            self.assertEqual(len(bad), 1)
            self.assertEqual(bad[0][0], 1)
        finally:
            os.unlink(img)

    def test_upload_short_tail_padded(self):
        # 512 + 200 bytes => 2 blocks (tail zero-padded)
        fd, path = tempfile.mkstemp(suffix='.bin')
        os.write(fd, b'\xAA' * 512 + b'\x55' * 200)
        os.close(fd)
        try:
            dev = MockDevice()
            sd = SdCard(dev)
            n = upload_rom(sd, path)
            self.assertEqual(n, 2)
            self.assertEqual(sd.read_block(0), b'\xAA' * 512)
            self.assertEqual(sd.read_block(1), b'\x55' * 200 + b'\x00' * 312)
        finally:
            os.unlink(path)


class SdOwnershipTests(unittest.TestCase):
    def test_ownership_toggles(self):
        dev = MockDevice()
        sd = SdCard(dev)
        self.assertFalse(sd.own_state)
        sd.request_ownership()
        self.assertTrue(sd.own_state)
        sd.release_ownership()
        self.assertFalse(sd.own_state)


if __name__ == '__main__':
    unittest.main()
