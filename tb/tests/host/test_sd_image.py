"""Tests for first-light raw SD image layout helpers."""

import tempfile
import unittest
import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[3] / 'tools'
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from m68kctl.sd_image import (  # noqa: E402
    RAW_SCSI_BASE_BYTE,
    RAW_SCSI_BASE_LBA,
    ROM_WINDOW_BYTES,
    ROM_WINDOW_LAST_LBA,
    SECTOR_SIZE,
    SdImageLayoutError,
    format_plan,
    plan_image,
    write_image,
)


class SdImageLayoutTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix='m68kctl-sdimg-')
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def _write(self, name: str, data: bytes) -> Path:
        path = self.tmp / name
        path.write_bytes(data)
        return path

    def _sparse(self, name: str, size: int) -> Path:
        path = self.tmp / name
        with path.open('wb') as f:
            f.truncate(size)
        return path

    def test_exact_rom_window_is_valid(self):
        rom = self._sparse('rom-4m.bin', ROM_WINDOW_BYTES)

        plan = plan_image(rom)

        self.assertEqual(plan.rom_sectors, RAW_SCSI_BASE_LBA)
        self.assertEqual(plan.raw_scsi_base_lba, ROM_WINDOW_LAST_LBA + 1)
        self.assertIn('overlap check     : PASS', format_plan(plan))

    def test_rom_one_byte_past_window_is_rejected(self):
        rom = self._sparse('rom-too-big.bin', ROM_WINDOW_BYTES + 1)

        with self.assertRaisesRegex(SdImageLayoutError, 'max ROM window'):
            plan_image(rom)

    def test_raw_hdd_image_must_be_sector_aligned(self):
        rom = self._write('rom.bin', b'rom')
        hdd = self._write('bad.hdv', b'x' * (SECTOR_SIZE + 1))

        with self.assertRaisesRegex(SdImageLayoutError, 'multiple of 512'):
            plan_image(rom, hdd)

    def test_write_image_places_hdd_after_reserved_rom_window(self):
        rom = self._write('rom.bin', b'ROMDATA')
        hdd_payload = bytes(range(256)) * 2
        hdd = self._write('disk.hdv', hdd_payload)
        out = self.tmp / 'sd.img'

        plan = plan_image(rom, hdd)
        written = write_image(plan, out)

        self.assertEqual(written, RAW_SCSI_BASE_BYTE + len(hdd_payload))
        with out.open('rb') as f:
            self.assertEqual(f.read(len(b'ROMDATA')), b'ROMDATA')
            f.seek(RAW_SCSI_BASE_BYTE - 1)
            self.assertEqual(f.read(1), b'\x00')
            self.assertEqual(f.read(len(hdd_payload)), hdd_payload)

    def test_existing_output_requires_overwrite(self):
        rom = self._write('rom.bin', b'rom')
        out = self._write('sd.img', b'exists')
        plan = plan_image(rom)

        with self.assertRaisesRegex(SdImageLayoutError, 'output exists'):
            write_image(plan, out)


if __name__ == '__main__':
    unittest.main()
