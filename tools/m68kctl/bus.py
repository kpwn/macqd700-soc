"""bus.py — system-bus (BAR 0) accessor.

Wraps a ``Device`` with region-aware read/write helpers that decode a
32-bit system-bus address into one of RAM / ROM / FB / I-O (per
``docs/peripheral_arch.md`` §"Address space partition") and dispatches to
either bulk DMA (for cacheable regions) or — in a future revision —
serialised MMIO (for the I/O region, which should see one transaction
per register).

Today all four regions go through the same DMA path, because:

* The XDMA AXI-MM master on BAR 0 reaches every address the CPU's LSU
  does, including the I-O window — the serialisation is enforced by the
  GLUE fabric, not the host.
* The MockDevice already serialises by virtue of being single-threaded.

If/when we discover a bug where short MMIO reads in the I-O region
misbehave via the DMA path, the I-O dispatch can switch to ``mmio_read32``
against a bypass BAR — the dispatch table below is the single place to
change.
"""

from __future__ import annotations

import logging
import struct
from typing import Iterable, Optional

from . import regs
from .device import Device


log = logging.getLogger(__name__)


class SystemBus:
    """Host-side view of the CPU's system bus, via BAR 0 DMA channels."""

    def __init__(self, device: Device):
        self.dev = device

    # ─── 32-bit word helpers ─────────────────────────────────────
    def read32(self, addr: int) -> int:
        data = self.read_bytes(addr, 4)
        # Little-endian on the host side — the m68k is big-endian on the
        # wire, but XDMA passes raw bytes and the RTL already swaps lanes
        # for peripherals.  For ROM/RAM regions the data is stored in
        # host-endian order (tests can round-trip with a plain struct).
        return struct.unpack('<I', data)[0]

    def write32(self, addr: int, value: int) -> None:
        self.write_bytes(addr, struct.pack('<I', value & 0xFFFFFFFF))

    # ─── Bulk helpers ────────────────────────────────────────────
    def read_bytes(self, addr: int, length: int) -> bytes:
        region = regs.region_of(addr)
        if region == 'unmapped':
            log.warning('read_bytes: addr 0x%x (len %d) hits unmapped region',
                        addr, length)
        log.debug('bus read  %-4s 0x%08x +%d', region, addr, length)
        return self.dev.dma_read(addr, length)

    def iter_read_chunks(self, addr: int, length: int,
                         chunk: int = 1 << 22) -> Iterable[bytes]:
        """Yield ``length`` bytes from ``addr`` in fixed-size DMA chunks."""
        remaining = length
        offset = 0
        while remaining > 0:
            n = min(chunk, remaining)
            yield self.read_bytes(addr + offset, n)
            offset += n
            remaining -= n

    def write_bytes(self, addr: int, data: bytes) -> None:
        region = regs.region_of(addr)
        if region == 'unmapped':
            log.warning('write_bytes: addr 0x%x (len %d) hits unmapped region',
                        addr, len(data))
        if region == 'rom':
            # ROM region is read-only from the CPU's perspective but can be
            # programmed from the host — this is the "DMA the ROM image into
            # DDR4 from Python" flow in docs/debug_pcie.md §"Host-driven boot".
            log.info('bus write rom 0x%08x +%d (ROM provisioning)', addr, len(data))
        else:
            log.debug('bus write %-4s 0x%08x +%d', region, addr, len(data))
        self.dev.dma_write(addr, data)

    # ─── Convenience: chunked bulk to avoid huge kernel-side allocs ──
    def read_bytes_chunked(self, addr: int, length: int,
                           chunk: int = 1 << 22,
                           progress_cb: Optional[callable] = None) -> bytes:
        out = bytearray()
        done = 0
        for block in self.iter_read_chunks(addr, length, chunk):
            out.extend(block)
            done += len(block)
            if progress_cb:
                progress_cb(done, length)
        return bytes(out)

    def write_bytes_chunked(self, addr: int, data: bytes,
                            chunk: int = 1 << 22,
                            progress_cb: Optional[callable] = None) -> None:
        total = len(data)
        pos = 0
        while pos < total:
            n = min(chunk, total - pos)
            self.write_bytes(addr + pos, data[pos:pos + n])
            pos += n
            if progress_cb:
                progress_cb(pos, total)
