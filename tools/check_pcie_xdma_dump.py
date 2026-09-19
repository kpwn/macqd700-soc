#!/usr/bin/env python3
"""Smoke-test the repo-owned XDMA checkpoint dump path.

This validates the host-side bulk dump flow in isolation:

* chunked DMA reads stream directly to files;
* multiple RAM/ROM regions can be captured in one pass;
* the manifest matches the written files.

The check runs entirely against ``MockDevice`` and does not require Vivado
or a programmed FPGA.
"""

from __future__ import annotations

import argparse
import json
import tempfile
import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[0]
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from m68kctl import MockDevice, SystemBus
from m68kctl.provision import DumpRegion, dump_regions, write_dump_manifest


def _payload(seed: bytes, length: int) -> bytes:
    data = (seed * ((length + len(seed) - 1) // len(seed)))[:length]
    return data


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--chunk-bytes', type=int, default=1 << 22,
                    help='chunk size to validate (default: 4 MiB)')
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    docs = (repo_root / 'docs' / 'debug_pcie.md').read_text(encoding='utf-8')
    readme = (repo_root / 'tools' / 'm68kctl' / 'README.md').read_text(encoding='utf-8')
    if 'm68kctl checkpoint dump' not in docs or 'm68kctl checkpoint dump' not in readme:
        raise RuntimeError('checkpoint dump command is not documented')
    if 'pcie-checkpoint-dump-check' not in readme:
        raise RuntimeError('validation target is not documented')

    with tempfile.TemporaryDirectory(prefix='m68kctl-xdma-check-') as tmp:
        tmpdir = Path(tmp)
        dev = MockDevice()
        bus = SystemBus(dev)

        ram = _payload(b'RAMDUMP', 8192)
        rom = _payload(b'ROMDUMP', 4096)
        bus.write_bytes(0x0000_1000, ram)
        bus.write_bytes(0x4000_2000, rom)

        regions = [
            DumpRegion('ram', 0x0000_1000, len(ram), tmpdir / 'ram.bin'),
            DumpRegion('rom', 0x4000_2000, len(rom), tmpdir / 'rom.bin'),
        ]
        manifest = dump_regions(bus, regions, chunk_bytes=args.chunk_bytes)
        manifest_path = tmpdir / 'manifest.json'
        write_dump_manifest(
            manifest_path,
            manifest=manifest,
            source='check_pcie_xdma_dump.py',
            chunk_bytes=args.chunk_bytes,
            total_bytes=sum(region.length for region in regions),
        )

        if (tmpdir / 'ram.bin').read_bytes() != ram:
            raise RuntimeError('RAM dump did not round-trip')
        if (tmpdir / 'rom.bin').read_bytes() != rom:
            raise RuntimeError('ROM dump did not round-trip')
        loaded = json.loads(manifest_path.read_text(encoding='utf-8'))
        if loaded['source'] != 'check_pcie_xdma_dump.py':
            raise RuntimeError('unexpected manifest source')
        if len(loaded['regions']) != 2:
            raise RuntimeError('unexpected manifest region count')
        if loaded['regions'][0]['bytes_written'] != len(ram):
            raise RuntimeError('unexpected RAM byte count')
        if loaded['regions'][1]['bytes_written'] != len(rom):
            raise RuntimeError('unexpected ROM byte count')

    print('PASS: XDMA checkpoint dump smoke test')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
