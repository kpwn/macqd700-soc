"""Tests for SystemBus region routing against MockDevice."""

import os
import json
import tempfile
import unittest
import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[3] / 'tools'
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from m68kctl import MockDevice, SystemBus
from m68kctl import regs
from m68kctl.provision import DumpRegion, dump_region, dump_regions, load_region, write_dump_manifest


class RegionRoutingTests(unittest.TestCase):
    def test_region_of_classifier(self):
        self.assertEqual(regs.region_of(0x00000000), 'ram')
        self.assertEqual(regs.region_of(0x0FFFFFFF), 'ram')
        self.assertEqual(regs.region_of(0x40000000), 'rom')
        self.assertEqual(regs.region_of(0x40FFFFFF), 'rom')
        self.assertEqual(regs.region_of(0x50000000), 'io')
        self.assertEqual(regs.region_of(0x60000000), 'fb')
        self.assertEqual(regs.region_of(0x70000000), 'unmapped')

    def test_ram_roundtrip(self):
        dev = MockDevice()
        bus = SystemBus(dev)
        bus.write_bytes(0x1000, b'hello world')
        self.assertEqual(bus.read_bytes(0x1000, 11), b'hello world')

    def test_word_roundtrip(self):
        dev = MockDevice()
        bus = SystemBus(dev)
        bus.write32(0x2000, 0xDEADBEEF)
        self.assertEqual(bus.read32(0x2000), 0xDEADBEEF)

    def test_regions_distinct(self):
        """Writes to different regions shouldn't collide."""
        dev = MockDevice()
        bus = SystemBus(dev)
        bus.write_bytes(0x00000100, b'RAM!')
        bus.write_bytes(0x40000100, b'ROM!')
        bus.write_bytes(0x60000100, b'FB!!')
        self.assertEqual(bus.read_bytes(0x00000100, 4), b'RAM!')
        self.assertEqual(bus.read_bytes(0x40000100, 4), b'ROM!')
        self.assertEqual(bus.read_bytes(0x60000100, 4), b'FB!!')

    def test_unwritten_region_zeros(self):
        dev = MockDevice()
        bus = SystemBus(dev)
        self.assertEqual(bus.read_bytes(0xAAAA0000, 32), b'\x00' * 32)


class DumpLoadTests(unittest.TestCase):
    def test_dump_load_roundtrip(self):
        payload = os.urandom(8192)
        fd, path = tempfile.mkstemp()
        os.close(fd)
        fd2, path2 = tempfile.mkstemp()
        os.close(fd2)
        try:
            # Stage the payload in a file.
            Path(path).write_bytes(payload)
            dev = MockDevice()
            bus = SystemBus(dev)
            load_region(bus, 0x40000000, path)
            dump_region(bus, 0x40000000, len(payload), path2)
            self.assertEqual(Path(path2).read_bytes(), payload)
        finally:
            os.unlink(path)
            os.unlink(path2)

    def test_dump_regions_manifest(self):
        payload_ram = b'RAMDUMP' * 256
        payload_rom = b'ROMDUMP' * 128
        with tempfile.TemporaryDirectory(prefix='m68kctl-bundle-') as tmp:
            tmpdir = Path(tmp)
            dev = MockDevice()
            bus = SystemBus(dev)
            bus.write_bytes(0x1000, payload_ram)
            bus.write_bytes(0x40002000, payload_rom)
            regions = [
                DumpRegion('ram', 0x1000, len(payload_ram), tmpdir / 'ram.bin'),
                DumpRegion('rom', 0x40002000, len(payload_rom), tmpdir / 'rom.bin'),
            ]
            manifest = dump_regions(bus, regions, chunk_bytes=4096)
            write_dump_manifest(tmpdir / 'manifest.json',
                                manifest=manifest,
                                source='test_dump_regions_manifest',
                                chunk_bytes=4096,
                                total_bytes=len(payload_ram) + len(payload_rom))

            self.assertEqual((tmpdir / 'ram.bin').read_bytes(), payload_ram)
            self.assertEqual((tmpdir / 'rom.bin').read_bytes(), payload_rom)
            data = json.loads((tmpdir / 'manifest.json').read_text())
            self.assertEqual(data['source'], 'test_dump_regions_manifest')
            self.assertEqual(len(data['regions']), 2)
            self.assertEqual(data['regions'][0]['name'], 'ram')
            self.assertEqual(data['regions'][1]['name'], 'rom')


if __name__ == '__main__':
    unittest.main()
