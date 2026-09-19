"""sd.py — ``SdCard`` wrapper over the sd_provision register block.

Drives single-block reads (CMD17) and writes (CMD24) through the register
map documented in ``rtl/sys/sd_provision.v``.  One outstanding command at
a time, polled with a bounded timeout.  Multi-block batching (CMD18 /
CMD25) is deferred to task #22 sd-ctrl unification.
"""

from __future__ import annotations

import struct
import time
from typing import Optional

from . import regs
from .device import Device


class SdError(RuntimeError):
    """Any error from the sd_provision path — includes err_cause classification."""


class SdTimeout(SdError):
    """Poll timed out waiting for DONE."""


class SdCard:
    """Single-block SD provisioning over sd_provision."""

    POLL_TIMEOUT_S = 1.0
    POLL_INTERVAL_S = 0.001

    def __init__(self, device: Device):
        self.dev = device

    # ─── Basic read/write ────────────────────────────────────────
    def _r(self, off: int) -> int:
        return self.dev.mmio_read32(1, off)

    def _w(self, off: int, val: int) -> None:
        self.dev.mmio_write32(1, off, val)

    # ─── Identity + status ───────────────────────────────────────
    @property
    def version(self) -> int:
        return self._r(regs.OFF_SDP_VERSION)

    @property
    def status(self) -> int:
        return self._r(regs.OFF_SDP_STATUS)

    @property
    def busy(self) -> bool:
        return bool(self.status & regs.SDP_STATUS_BUSY)

    @property
    def done(self) -> bool:
        return bool(self.status & regs.SDP_STATUS_DONE)

    @property
    def error(self) -> bool:
        return bool(self.status & regs.SDP_STATUS_ERROR)

    @property
    def card_ready(self) -> bool:
        return bool(self.status & regs.SDP_STATUS_CARD_READY)

    @property
    def own_state(self) -> bool:
        return bool(self.status & regs.SDP_STATUS_OWN_STATE)

    @property
    def err_cause(self) -> int:
        return self._r(regs.OFF_SDP_ERR_CAUSE)

    @property
    def cmd_count(self) -> int:
        return self._r(regs.OFF_SDP_CMD_COUNT)

    def request_ownership(self) -> None:
        self._w(regs.OFF_SDP_OWN_REQ, 1)

    def release_ownership(self) -> None:
        self._w(regs.OFF_SDP_OWN_REQ, 0)

    def clear_error(self) -> None:
        # bit31 is the clear_error side-effect (see sd_provision.v register
        # map).  Writing it also drops any pending SDP_CMD action.
        self._w(regs.OFF_SDP_CMD, 0x8000_0000)

    # ─── Scratch-buffer helpers ──────────────────────────────────
    def _write_scratch(self, data: bytes) -> None:
        if len(data) != regs.SDP_BLOCK_SIZE:
            raise ValueError(f'SD block must be {regs.SDP_BLOCK_SIZE} B, got {len(data)}')
        for widx in range(128):
            w = struct.unpack_from('<I', data, widx * 4)[0]
            self._w(regs.OFF_SDP_BUF_BASE + widx * 4, w)

    def _read_scratch(self) -> bytes:
        buf = bytearray(regs.SDP_BLOCK_SIZE)
        for widx in range(128):
            w = self._r(regs.OFF_SDP_BUF_BASE + widx * 4)
            struct.pack_into('<I', buf, widx * 4, w)
        return bytes(buf)

    # ─── Command execution ───────────────────────────────────────
    def _poll_done(self) -> None:
        deadline = time.monotonic() + self.POLL_TIMEOUT_S
        while True:
            st = self.status
            if st & regs.SDP_STATUS_ERROR:
                cause = self.err_cause
                raise SdError(
                    f'sd_provision error: status=0x{st:08x}, '
                    f'err_cause={cause}')
            if st & regs.SDP_STATUS_DONE:
                return
            if time.monotonic() > deadline:
                raise SdTimeout(
                    f'sd_provision command timed out (status=0x{st:08x})')
            time.sleep(self.POLL_INTERVAL_S)

    def read_block(self, lba: int) -> bytes:
        """Single-block read (CMD17) at ``lba``.  Returns 512 B."""
        if self.busy:
            raise SdError('sd_provision is busy — cannot start new command')
        self._w(regs.OFF_SDP_LBA, lba & 0xFFFFFFFF)
        self._w(regs.OFF_SDP_CMD, regs.SDP_CMD_READ)
        self._poll_done()
        return self._read_scratch()

    def write_block(self, lba: int, data: bytes) -> None:
        """Single-block write (CMD24) at ``lba``.  ``data`` must be 512 B."""
        if self.busy:
            raise SdError('sd_provision is busy — cannot start new command')
        self._write_scratch(data)
        self._w(regs.OFF_SDP_LBA, lba & 0xFFFFFFFF)
        self._w(regs.OFF_SDP_CMD, regs.SDP_CMD_WRITE)
        self._poll_done()
