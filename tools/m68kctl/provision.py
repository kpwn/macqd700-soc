"""provision.py — high-level flows composed from SystemBus + SdCard.

The two headline flows:

* ``upload_rom(sd, path)``  — stream a 4 MB ROM image onto the SD card one
  block at a time, starting at LBA 0.  Pair with ``verify_rom``.
* ``dump_region(bus, addr, length, path)`` — bulk-dump a system-bus region
  to a file.  Used during bring-up to snapshot DDR4 or the framebuffer.
* ``load_region(bus, addr, path)``          — inverse.

All four accept an optional ``progress_cb(done, total)`` callable so the
CLI can render a progress bar; library callers who want a progress bar
can pass one of their own.
"""

from __future__ import annotations

import os
import hashlib
import json
from pathlib import Path
from dataclasses import dataclass
from typing import Callable, Optional

from . import regs
from .bus import SystemBus
from .sd import SdCard


ProgressCB = Optional[Callable[[int, int], None]]


@dataclass(frozen=True)
class DumpRegion:
    """One region in a checkpoint dump bundle."""

    name: str
    addr: int
    length: int
    path: Path


def _pad_block(chunk: bytes) -> bytes:
    """Zero-pad ``chunk`` up to a 512-byte block."""
    if len(chunk) == regs.SDP_BLOCK_SIZE:
        return chunk
    return chunk + b'\x00' * (regs.SDP_BLOCK_SIZE - len(chunk))


# ═════════════════════════════════════════════════════════════════════════
# SD-card ROM upload / verify
# ═════════════════════════════════════════════════════════════════════════
def upload_rom(sd: SdCard,
               image_path: str | os.PathLike,
               *,
               start_lba: int = 0,
               progress_cb: ProgressCB = None) -> int:
    """Upload ``image_path`` to the SD card one 512-byte block at a time.

    Returns the number of blocks written.  Short trailing chunks are zero-
    padded to a full block.  Caller owns error handling — any SdError
    propagates.
    """
    p = Path(image_path)
    size = p.stat().st_size
    blocks = (size + regs.SDP_BLOCK_SIZE - 1) // regs.SDP_BLOCK_SIZE
    written = 0
    with p.open('rb') as f:
        for i in range(blocks):
            chunk = f.read(regs.SDP_BLOCK_SIZE)
            if not chunk:
                break
            sd.write_block(start_lba + i, _pad_block(chunk))
            written += 1
            if progress_cb:
                progress_cb(written * regs.SDP_BLOCK_SIZE, size)
    return written


def verify_rom(sd: SdCard,
               image_path: str | os.PathLike,
               *,
               start_lba: int = 0,
               progress_cb: ProgressCB = None) -> list[tuple[int, bytes, bytes]]:
    """Read back the SD card and diff against ``image_path``.

    Returns a list of ``(lba, expected, actual)`` tuples for every
    mismatched block.  Empty list = clean verify.
    """
    p = Path(image_path)
    size = p.stat().st_size
    blocks = (size + regs.SDP_BLOCK_SIZE - 1) // regs.SDP_BLOCK_SIZE
    mismatches: list[tuple[int, bytes, bytes]] = []
    with p.open('rb') as f:
        for i in range(blocks):
            exp = _pad_block(f.read(regs.SDP_BLOCK_SIZE))
            got = sd.read_block(start_lba + i)
            if exp != got:
                mismatches.append((start_lba + i, exp, got))
            if progress_cb:
                progress_cb((i + 1) * regs.SDP_BLOCK_SIZE, size)
    return mismatches


# ═════════════════════════════════════════════════════════════════════════
# System-bus dump / load
# ═════════════════════════════════════════════════════════════════════════
def dump_region(bus: SystemBus,
                addr: int,
                length: int,
                path: str | os.PathLike,
                *,
                progress_cb: ProgressCB = None,
                chunk_bytes: int = 1 << 22) -> int:
    """Dump ``length`` bytes from ``addr`` to ``path``.

    The implementation streams one DMA chunk at a time so large ROM/RAM
    captures do not build the full image in memory before hitting disk.
    Returns the number of bytes written.
    """
    out_path = Path(path)
    written = 0
    with out_path.open('wb') as f:
        for block in bus.iter_read_chunks(addr, length, chunk_bytes):
            f.write(block)
            written += len(block)
            if progress_cb:
                progress_cb(written, length)
    return written


def dump_regions(bus: SystemBus,
                 regions: list[DumpRegion],
                 *,
                 progress_cb: ProgressCB = None,
                 chunk_bytes: int = 1 << 22) -> list[dict[str, object]]:
    """Stream a checkpoint bundle to disk and return manifest entries."""
    manifest: list[dict[str, object]] = []
    total = sum(region.length for region in regions)
    done = 0
    for region in regions:
        digest = hashlib.sha256()
        written = 0
        with region.path.open('wb') as f:
            for block in bus.iter_read_chunks(region.addr, region.length,
                                              chunk_bytes):
                f.write(block)
                digest.update(block)
                written += len(block)
                done += len(block)
                if progress_cb:
                    progress_cb(done, total)
        manifest.append({
            'name': region.name,
            'addr': region.addr,
            'length': region.length,
            'path': str(region.path),
            'bytes_written': written,
            'sha256': digest.hexdigest(),
        })
    return manifest


def write_dump_manifest(path: str | os.PathLike,
                        *,
                        manifest: list[dict[str, object]],
                        source: str,
                        chunk_bytes: int,
                        total_bytes: int) -> None:
    Path(path).write_text(json.dumps({
        'source': source,
        'chunk_bytes': chunk_bytes,
        'total_bytes': total_bytes,
        'regions': manifest,
    }, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def load_region(bus: SystemBus,
                addr: int,
                path: str | os.PathLike,
                *,
                progress_cb: ProgressCB = None,
                chunk_bytes: int = 1 << 22) -> int:
    """Load the contents of ``path`` into system-bus memory at ``addr``.

    Returns bytes loaded.
    """
    total = Path(path).stat().st_size
    loaded = 0
    with Path(path).open('rb') as f:
        while True:
            chunk = f.read(chunk_bytes)
            if not chunk:
                break
            bus.write_bytes(addr + loaded, chunk)
            loaded += len(chunk)
            if progress_cb:
                progress_cb(loaded, total)
    return loaded
