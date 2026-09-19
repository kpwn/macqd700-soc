"""Raw SD-card image layout helpers for first-light storage.

The hardware storage contract is deliberately positional:

* SD LBA 0..8191 is the 4 MiB ROM/provisioning window.
* SCSI disk LBA 0 maps to SD LBA 8192.
* No filesystem or partition parser is involved.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
from typing import Optional


SECTOR_SIZE = 512
ROM_WINDOW_BYTES = 4 * 1024 * 1024
ROM_WINDOW_LBAS = ROM_WINDOW_BYTES // SECTOR_SIZE
ROM_WINDOW_LAST_LBA = ROM_WINDOW_LBAS - 1
RAW_SCSI_BASE_LBA = ROM_WINDOW_LBAS
RAW_SCSI_BASE_BYTE = RAW_SCSI_BASE_LBA * SECTOR_SIZE
COPY_CHUNK_BYTES = 1024 * 1024


class SdImageLayoutError(RuntimeError):
    """Raised when a requested SD image would violate the raw layout."""


def sectors_for_size(size_bytes: int) -> int:
    """Return the number of 512-byte sectors needed for ``size_bytes``."""
    if size_bytes < 0:
        raise SdImageLayoutError(f'negative byte count: {size_bytes}')
    return (size_bytes + SECTOR_SIZE - 1) // SECTOR_SIZE


@dataclass(frozen=True)
class SdImagePlan:
    """Validated positional SD image plan."""

    rom_path: Path
    rom_size: int
    rom_sectors: int
    hdd_path: Optional[Path]
    hdd_size: int
    hdd_sectors: int

    @property
    def raw_scsi_base_lba(self) -> int:
        return RAW_SCSI_BASE_LBA

    @property
    def raw_scsi_base_byte(self) -> int:
        return RAW_SCSI_BASE_BYTE

    @property
    def image_size(self) -> int:
        return RAW_SCSI_BASE_BYTE + self.hdd_size

    def as_dict(self) -> dict[str, object]:
        return {
            'sector_size': SECTOR_SIZE,
            'rom_window_bytes': ROM_WINDOW_BYTES,
            'rom_window_lbas': ROM_WINDOW_LBAS,
            'rom_window_last_lba': ROM_WINDOW_LAST_LBA,
            'raw_scsi_base_lba': self.raw_scsi_base_lba,
            'raw_scsi_base_byte': self.raw_scsi_base_byte,
            'rom_path': str(self.rom_path),
            'rom_size': self.rom_size,
            'rom_sectors': self.rom_sectors,
            'hdd_path': str(self.hdd_path) if self.hdd_path else None,
            'hdd_size': self.hdd_size,
            'hdd_sectors': self.hdd_sectors,
            'image_size': self.image_size,
        }


def plan_image(rom_path: str | Path,
               hdd_path: str | Path | None = None) -> SdImagePlan:
    """Validate and return a first-light raw SD image plan."""
    rom = Path(rom_path)
    if not rom.is_file():
        raise SdImageLayoutError(f'ROM image not found: {rom}')
    rom_size = rom.stat().st_size
    rom_sectors = sectors_for_size(rom_size)

    if rom_size > ROM_WINDOW_BYTES:
        raise SdImageLayoutError(
            f'ROM image is {rom_size} bytes; max ROM window is '
            f'{ROM_WINDOW_BYTES} bytes ({ROM_WINDOW_LBAS} sectors)')
    if rom_sectors > RAW_SCSI_BASE_LBA:
        raise SdImageLayoutError(
            f'ROM image consumes {rom_sectors} sectors and would overlap '
            f'raw SCSI base LBA {RAW_SCSI_BASE_LBA}')
    if RAW_SCSI_BASE_LBA != ROM_WINDOW_LAST_LBA + 1:
        raise SdImageLayoutError(
            'internal layout error: raw SCSI base does not immediately '
            'follow the ROM window')

    hdd: Optional[Path] = None
    hdd_size = 0
    hdd_sectors = 0
    if hdd_path is not None:
        hdd = Path(hdd_path)
        if not hdd.is_file():
            raise SdImageLayoutError(f'raw HDD image not found: {hdd}')
        hdd_size = hdd.stat().st_size
        if hdd_size % SECTOR_SIZE != 0:
            raise SdImageLayoutError(
                f'raw HDD image is {hdd_size} bytes; size must be a '
                f'multiple of {SECTOR_SIZE} bytes')
        hdd_sectors = hdd_size // SECTOR_SIZE

    return SdImagePlan(
        rom_path=rom,
        rom_size=rom_size,
        rom_sectors=rom_sectors,
        hdd_path=hdd,
        hdd_size=hdd_size,
        hdd_sectors=hdd_sectors,
    )


def format_plan(plan: SdImagePlan) -> str:
    """Return a human-readable layout summary."""
    rom_last = plan.rom_sectors - 1 if plan.rom_sectors else 0
    lines = [
        'SD raw-block layout:',
        f'  sector size       : {SECTOR_SIZE} bytes',
        f'  ROM window        : LBA 0..{ROM_WINDOW_LAST_LBA} '
        f'({ROM_WINDOW_BYTES} bytes reserved)',
        f'  ROM image         : {plan.rom_path} ({plan.rom_size} bytes, '
        f'{plan.rom_sectors} sectors, LBA 0..{rom_last})',
        f'  raw SCSI HDD base : LBA {plan.raw_scsi_base_lba} '
        f'(byte offset 0x{plan.raw_scsi_base_byte:08x})',
        '  overlap check     : PASS (raw base is ROM window last LBA + 1)',
    ]
    if plan.hdd_path:
        hdd_last = (plan.raw_scsi_base_lba + plan.hdd_sectors - 1
                    if plan.hdd_sectors else plan.raw_scsi_base_lba)
        lines.append(
            f'  raw HDD image     : {plan.hdd_path} ({plan.hdd_size} bytes, '
            f'{plan.hdd_sectors} sectors, SD LBA {plan.raw_scsi_base_lba}'
            f'..{hdd_last})')
    else:
        lines.append('  raw HDD image     : none')
    lines.append(f'  output image size : {plan.image_size} bytes')
    return '\n'.join(lines)


def write_image(plan: SdImagePlan,
                output_path: str | Path,
                *,
                overwrite: bool = False) -> int:
    """Write a combined raw SD image and return its byte size."""
    out = Path(output_path)
    if out.exists() and not overwrite:
        raise SdImageLayoutError(f'output exists, pass --overwrite: {out}')

    with out.open('wb') as outf:
        with plan.rom_path.open('rb') as romf:
            shutil.copyfileobj(romf, outf, COPY_CHUNK_BYTES)
        pos = outf.tell()
        if pos > RAW_SCSI_BASE_BYTE:
            raise SdImageLayoutError(
                'internal layout error: ROM copy crossed raw SCSI base')
        if pos < RAW_SCSI_BASE_BYTE:
            outf.write(b'\x00' * (RAW_SCSI_BASE_BYTE - pos))
        if plan.hdd_path:
            with plan.hdd_path.open('rb') as hddf:
                shutil.copyfileobj(hddf, outf, COPY_CHUNK_BYTES)
    return out.stat().st_size
