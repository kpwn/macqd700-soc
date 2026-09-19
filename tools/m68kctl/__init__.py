"""m68kctl — host-side control + debug for the m68k-ooo FPGA target.

Primary entry points:

* :class:`m68kctl.device.XdmaDevice`  — real-FPGA backend over
  ``/dev/xdma0_*``
* :class:`m68kctl.device.MockDevice`  — dict-backed mock (works without FPGA)
* :class:`m68kctl.bus.SystemBus`      — BAR 0 DMA into RAM/ROM/FB/I-O
* :class:`m68kctl.cpu.CpuDebug`       — BAR 1 debug_ctrl wrapper
* :class:`m68kctl.sd.SdCard`          — BAR 1 sd_provision wrapper
* :mod:`m68kctl.provision`            — high-level upload / verify / dump flows

CLI: ``python -m m68kctl <subcmd>`` or (after `pip install -e tools/m68kctl`)
``m68kctl <subcmd>``.
"""

from .device import Device, MockDevice, XdmaDevice
from .bus import SystemBus
from .cpu import CommitRecord, CpuDebug, HaltDebugStatus
from .sd import SdCard, SdError, SdTimeout
from .provision import (
    DumpRegion,
    dump_region,
    dump_regions,
    load_region,
    upload_rom,
    verify_rom,
    write_dump_manifest,
)
from . import regs

__all__ = [
    'Device', 'MockDevice', 'XdmaDevice',
    'SystemBus',
    'CpuDebug', 'CommitRecord', 'HaltDebugStatus',
    'SdCard', 'SdError', 'SdTimeout',
    'DumpRegion', 'dump_region', 'dump_regions', 'load_region',
    'upload_rom', 'verify_rom', 'write_dump_manifest',
    'regs',
]

__version__ = '0.1.0'
