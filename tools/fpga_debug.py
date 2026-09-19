#!/usr/bin/env python3
"""fpga_debug.py — backwards-compatibility shim.

The host-side helper has been restructured into the ``m68kctl`` package
(see ``tools/m68kctl/``).  This file preserves the classic
``from fpga_debug import Fpga`` import for any legacy caller while the
real implementation lives in :mod:`m68kctl.cpu` / :mod:`m68kctl.device`.

Also remains directly runnable as a CLI — delegates to
``m68kctl.cli.main info``.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure `tools/` is on the import path so this file works whether invoked
# as `python3 tools/fpga_debug.py` or imported from elsewhere.
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from m68kctl.cpu import CpuDebug              # noqa: E402
from m68kctl.device import XdmaDevice         # noqa: E402
from m68kctl import regs as _regs             # noqa: E402


class Fpga(CpuDebug):
    """Legacy wrapper — opens ``/dev/xdma0_user`` and exposes CpuDebug methods.

    The original ``fpga_debug.Fpga`` class bundled device open + register
    access.  We preserve that API by subclassing :class:`CpuDebug` and
    grabbing an :class:`XdmaDevice` on construction.
    """

    # Class-level constants preserved for any caller that referenced them.
    BAR1_SIZE            = _regs.BAR1_SIZE
    VERSION_MAGIC        = _regs.DBG_VERSION_MAGIC
    OFF_VERSION          = _regs.OFF_DBG_VERSION
    OFF_BUILD_ID         = _regs.OFF_DBG_BUILD_ID
    OFF_CONTROL          = _regs.OFF_DBG_CONTROL
    OFF_STATUS           = _regs.OFF_DBG_STATUS
    OFF_PC               = _regs.OFF_DBG_PC
    OFF_LAST_PC          = _regs.OFF_DBG_LAST_PC
    OFF_REDIRECT_PC      = _regs.OFF_DBG_REDIRECT_PC
    OFF_REDIRECT_TRIGGER = _regs.OFF_DBG_REDIRECT_TRIGGER
    OFF_IRQ_INJECT       = _regs.OFF_DBG_IRQ_INJECT
    OFF_EXC_VEC          = _regs.OFF_DBG_EXC_VEC
    OFF_EXC_PC           = _regs.OFF_DBG_EXC_PC
    OFF_RESET_CAUSE      = _regs.OFF_DBG_RESET_CAUSE
    OFF_HALT_AFTER_LO    = _regs.OFF_DBG_HALT_AFTER_LO
    OFF_HALT_AFTER_HI    = _regs.OFF_DBG_HALT_AFTER_HI
    OFF_BREAK_PC         = _regs.OFF_DBG_BREAK_PC
    OFF_HALT_CTL         = _regs.OFF_DBG_HALT_CTL
    OFF_HALT_REASON      = _regs.OFF_DBG_HALT_REASON
    OFF_HALT_HIT_PC      = _regs.OFF_DBG_HALT_HIT_PC
    OFF_HALT_HIT_INST_LO = _regs.OFF_DBG_HALT_HIT_INST_LO
    OFF_HALT_HIT_INST_HI = _regs.OFF_DBG_HALT_HIT_INST_HI
    OFF_HALT_EXC_VEC     = _regs.OFF_DBG_HALT_EXC_VEC
    OFF_CYCLE_LO         = _regs.OFF_DBG_CYCLE_LO
    OFF_CYCLE_HI         = _regs.OFF_DBG_CYCLE_HI
    OFF_INST_LO          = _regs.OFF_DBG_INST_LO
    OFF_INST_HI          = _regs.OFF_DBG_INST_HI
    OFF_MISPRED_COUNT    = _regs.OFF_DBG_MISPRED_COUNT
    OFF_FLUSH_COUNT      = _regs.OFF_DBG_FLUSH_COUNT
    OFF_EXC_COUNT        = _regs.OFF_DBG_EXC_COUNT
    OFF_PC_TRACE_BASE    = _regs.OFF_DBG_PC_TRACE_BASE
    OFF_PC_TRACE_HEAD    = _regs.OFF_DBG_PC_TRACE_HEAD
    PC_TRACE_DEPTH       = _regs.DBG_PC_TRACE_DEPTH
    CTL_HALT_REQ         = _regs.CTL_HALT_REQ
    CTL_STEP_PULSE       = _regs.CTL_STEP_PULSE
    CTL_SOFT_RST         = _regs.CTL_SOFT_RST
    CTL_INIT_DONE_OVR    = _regs.CTL_INIT_DONE_OVR
    STS_HALTED           = _regs.STS_HALTED
    STS_EXC_PENDING      = _regs.STS_EXC_PENDING
    STS_INIT_DONE_SEEN   = _regs.STS_INIT_DONE_SEEN
    STS_CPU_RUNNING      = _regs.STS_CPU_RUNNING

    def __init__(self, dev: str = '/dev/xdma0_user', *, verify: bool = True):
        self._xdma = XdmaDevice(bar1_dev=dev)
        super().__init__(self._xdma, verify_magic=verify)

    # Legacy method names.
    def read32(self, off: int) -> int:
        return self._xdma.mmio_read32(1, off)

    def write32(self, off: int, val: int) -> None:
        self._xdma.mmio_write32(1, off, val)

    def read64(self, off_lo: int) -> int:
        lo = self.read32(off_lo)
        hi = self.read32(off_lo + 4)
        return (hi << 32) | lo

    def close(self) -> None:
        self._xdma.close()

    def __enter__(self) -> 'Fpga':
        return self

    def __exit__(self, *args) -> None:
        self.close()


def main(argv=None) -> int:
    """Delegate to m68kctl CLI (``m68kctl info``) — preserves the old
    ``python3 tools/fpga_debug.py`` smoke-test behaviour.
    """
    from m68kctl.cli import main as _main
    return _main(['info'] + (argv or []))


if __name__ == '__main__':
    sys.exit(main())
