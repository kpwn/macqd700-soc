"""device.py — low-level XDMA channel wrapper for m68kctl.

Provides two backends that expose the same interface:

* ``XdmaDevice``  — real FPGA access via the Xilinx XDMA character devices
  (``/dev/xdma0_user`` for BAR-mapped MMIO, ``/dev/xdma0_h2c_0`` / ``_c2h_0``
  for bulk DMA).  Everything is opened lazily on first use so merely
  importing this module or constructing an XdmaDevice on a machine without
  an FPGA does not raise.
* ``MockDevice``   — dict-backed memory model used by the test suite and by
  ``m68kctl --mock`` for on-developer-machine exercises.  Writes stash into
  a dict; reads return the last-written value or 0.  Version registers are
  pre-populated with the correct magic so self-checks pass.  A tiny SD-card
  simulator (triggered on SDP_CMD writes) lets the full provisioning flow
  run end-to-end without real hardware.

Register offsets live in ``m68kctl.regs`` to keep the backends agnostic of
specific register meanings — but the MockDevice's SD-card sim imports them
so the mock can speak the right protocol.

Sub-agent ``host-infra``.  See ``docs/debug_pcie.md`` for the authoritative
register catalogue.
"""

from __future__ import annotations

import logging
import mmap
import os
import pickle
import struct
import threading
from typing import Any, Dict, Optional

from . import regs


# Environment variable consulted by ``MockDevice`` to persist its state
# across separate CLI invocations.  See the class docstring below for the
# on-disk format.
MOCK_STATE_ENV = 'M68KCTL_MOCK_STATE'


log = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────
# Abstract base — lets type-hints stay tidy across backends.
# ─────────────────────────────────────────────────────────────────────────
class Device:
    """Abstract MMIO+DMA backend interface."""

    bar1_size: int = 1 << 20        # 1 MB debug_ctrl + sd_provision window

    def mmio_read32(self, bar: int, offset: int) -> int:
        raise NotImplementedError

    def mmio_write32(self, bar: int, offset: int, value: int) -> None:
        raise NotImplementedError

    def dma_read(self, addr: int, length: int) -> bytes:
        raise NotImplementedError

    def dma_write(self, addr: int, data: bytes) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __enter__(self) -> 'Device':
        return self

    def __exit__(self, *args) -> None:
        self.close()


# ─────────────────────────────────────────────────────────────────────────
# Real XDMA backend
# ─────────────────────────────────────────────────────────────────────────
class XdmaDevice(Device):
    """Real-FPGA XDMA backend — opens ``/dev/xdma0_*`` on demand.

    The XDMA driver exposes a user BAR as a char device that can be
    ``mmap``'d.  Per ``docs/debug_pcie.md``:

        BAR 0 — AXI-MM master onto the CPU system bus (RAM/ROM/FB/I-O).
                Bulk access via ``/dev/xdma0_h2c_0`` (host→card) and
                ``/dev/xdma0_c2h_0`` (card→host); direct MMIO via
                ``/dev/xdma0_bypass`` (kernel may or may not expose this,
                so bulk is always the safer path).
        BAR 1 — AXI-Lite peripheral window, ``/dev/xdma0_user`` (1 MB).
                Covers the full debug_ctrl (0x00000-0x7FFFF) + the
                sd_provision (0x80000-0x8FFFF) register files.

    Each BAR is opened lazily on first use.  Close releases all handles.
    A single mutex guards the mmap'd region so multi-threaded CLI users
    don't hand out clobbered reads.
    """

    BAR1_DEV = '/dev/xdma0_user'
    H2C_DEV  = '/dev/xdma0_h2c_0'
    C2H_DEV  = '/dev/xdma0_c2h_0'

    def __init__(self,
                 bar1_dev: Optional[str] = None,
                 h2c_dev: Optional[str] = None,
                 c2h_dev: Optional[str] = None):
        self._bar1_dev = bar1_dev or self.BAR1_DEV
        self._h2c_dev  = h2c_dev  or self.H2C_DEV
        self._c2h_dev  = c2h_dev  or self.C2H_DEV

        self._bar1_fd: Optional[int] = None
        self._bar1_map: Optional[mmap.mmap] = None
        self._h2c_fd: Optional[int] = None
        self._c2h_fd: Optional[int] = None
        self._lock = threading.Lock()

    # ─── Lazy opens ──────────────────────────────────────────────
    def _ensure_bar1(self) -> mmap.mmap:
        if self._bar1_map is None:
            try:
                self._bar1_fd = os.open(self._bar1_dev, os.O_RDWR | os.O_SYNC)
            except FileNotFoundError as e:
                raise FileNotFoundError(
                    f"XDMA device {self._bar1_dev} not found. "
                    "Ensure the FPGA is programmed and the Xilinx XDMA "
                    "driver is loaded (modprobe xdma)."
                ) from e
            self._bar1_map = mmap.mmap(self._bar1_fd, self.bar1_size)
        return self._bar1_map

    def _ensure_h2c(self) -> int:
        if self._h2c_fd is None:
            self._h2c_fd = os.open(self._h2c_dev, os.O_WRONLY)
        return self._h2c_fd

    def _ensure_c2h(self) -> int:
        if self._c2h_fd is None:
            self._c2h_fd = os.open(self._c2h_dev, os.O_RDONLY)
        return self._c2h_fd

    # ─── MMIO ────────────────────────────────────────────────────
    def mmio_read32(self, bar: int, offset: int) -> int:
        if bar != 1:
            raise ValueError(f"XdmaDevice only supports MMIO on BAR 1 (got BAR {bar})")
        if offset & 0x3:
            raise ValueError(f"unaligned MMIO read: 0x{offset:x}")
        if offset < 0 or offset + 4 > self.bar1_size:
            raise ValueError(f"MMIO offset 0x{offset:x} out of BAR1 range")
        with self._lock:
            m = self._ensure_bar1()
            return struct.unpack_from('<I', m, offset)[0]

    def mmio_write32(self, bar: int, offset: int, value: int) -> None:
        if bar != 1:
            raise ValueError(f"XdmaDevice only supports MMIO on BAR 1 (got BAR {bar})")
        if offset & 0x3:
            raise ValueError(f"unaligned MMIO write: 0x{offset:x}")
        if offset < 0 or offset + 4 > self.bar1_size:
            raise ValueError(f"MMIO offset 0x{offset:x} out of BAR1 range")
        with self._lock:
            m = self._ensure_bar1()
            struct.pack_into('<I', m, offset, value & 0xFFFFFFFF)

    # ─── DMA ─────────────────────────────────────────────────────
    def dma_read(self, addr: int, length: int) -> bytes:
        if length < 0:
            raise ValueError(f"negative DMA length {length}")
        if length == 0:
            return b''
        fd = self._ensure_c2h()
        os.lseek(fd, addr, os.SEEK_SET)
        out = bytearray()
        while len(out) < length:
            chunk = os.read(fd, length - len(out))
            if not chunk:
                raise IOError(
                    f"short read from {self._c2h_dev}: "
                    f"got {len(out)} of {length} bytes")
            out.extend(chunk)
        return bytes(out)

    def dma_write(self, addr: int, data: bytes) -> None:
        if len(data) == 0:
            return
        fd = self._ensure_h2c()
        os.lseek(fd, addr, os.SEEK_SET)
        mv = memoryview(data)
        written = 0
        while written < len(mv):
            n = os.write(fd, mv[written:])
            if n == 0:
                raise IOError(
                    f"short write to {self._h2c_dev}: "
                    f"wrote {written} of {len(data)} bytes")
            written += n

    # ─── Close ───────────────────────────────────────────────────
    def close(self) -> None:
        with self._lock:
            if self._bar1_map is not None:
                try:    self._bar1_map.close()
                except Exception: pass
                self._bar1_map = None
            for attr in ('_bar1_fd', '_h2c_fd', '_c2h_fd'):
                fd = getattr(self, attr)
                if fd is not None:
                    try: os.close(fd)
                    except Exception: pass
                    setattr(self, attr, None)


# ─────────────────────────────────────────────────────────────────────────
# Mock backend — dict-backed memory model + fake SD card
# ─────────────────────────────────────────────────────────────────────────
class MockDevice(Device):
    """In-process mock of ``XdmaDevice``.  Intended uses:

    1. ``m68kctl --mock <subcmd>`` — lets the full CLI run on a machine
       with no FPGA; handy for sanity checks & demos.
    2. Unit tests in ``tb/tests/host/`` — every flow in ``provision.py``
       and the CLI is exercised against this mock.

    Behaviour:

    * MMIO reads return the last-written 32-bit value for the offset, or
      0 for an unwritten offset, except for:
        - BAR 1 0x00000 (DBG_VERSION) — returns ``0xDEB60001`` (magic)
        - BAR 1 0x80000 (SDP_VERSION) — returns ``0x5DP00001``
    * DMA-region addresses stash into a separate dict keyed by (addr, len)
      range; reads return the bytes last written (or zeros).
    * SD-card sim:
        - SDP_CMD=READ(0x01)  copies ``_sd_blocks[lba]`` → scratch buffer
        - SDP_CMD=WRITE(0x02) copies scratch buffer → ``_sd_blocks[lba]``
        - STATUS.done pulses for a couple of polls after each command.
      The backing store is ``self.sd_blocks`` (dict: lba → bytes(512)).
      Unwritten blocks read back as zeros.

    Persistence
    -----------
    The MockDevice is in-process only by default — every constructor
    produces a fresh blank device.  That works for library consumers
    (one Python process, one MockDevice) but breaks the CLI flow where
    each ``m68kctl --mock <subcmd>`` invocation spawns a new process:
    the second invocation cannot read what the first wrote.

    To fix that without changing library semantics, ``MockDevice``
    optionally persists a tiny snapshot of its state to a pickle file.
    Activation is opt-in, driven by one of:

    * explicit ``state_path=`` constructor argument (wins over env var);
    * ``M68KCTL_MOCK_STATE`` environment variable (the CLI sets this
      when ``--mock`` is passed).

    When active:
      * on construction, if the file exists, state is loaded;
      * on ``close()`` (and ``__del__`` / context-manager exit), the
        state is re-written to the same path.

    On-disk format is a plain ``dict`` — not a class — so adding new
    surfaces later doesn't break unpickling of older snapshots.  Keys:

        {
          'mmio':      {bar(int): {offset(int): value(int)}},
          'sd_blocks': {lba(int):  bytes(512)},
        }

    Unknown keys are silently ignored at load time; missing keys fall
    back to the default empty state.  Keeping the surface minimal also
    means the pickle file is trivially inspectable with ``python3 -c
    "import pickle, sys; print(pickle.load(open(sys.argv[1], 'rb')))"``.
    """

    def __init__(self, state_path: Optional[str] = None):
        # Per-BAR regfile: {offset: u32}
        self._mmio: Dict[int, Dict[int, int]] = {0: {}, 1: {}}
        # DMA memory: flat byte-addressable dict, sparse
        self._dma_mem: bytearray = bytearray()
        self._dma_base: int = 0    # first non-zero address seen (for info only)
        self._dma_pages: Dict[int, bytearray] = {}   # 4 KB pages

        # Pre-set version magics per the RTL spec.
        self._mmio[1][regs.OFF_DBG_VERSION] = regs.DBG_VERSION_MAGIC
        self._mmio[1][regs.OFF_DBG_BUILD_ID] = 0x600DF00D
        self._mmio[1][regs.OFF_SDP_VERSION] = regs.SDP_VERSION_MAGIC
        # card_ready bit, no error, not busy
        self._mmio[1][regs.OFF_SDP_STATUS] = regs.SDP_STATUS_CARD_READY

        # CPU counters that plausible tests may read
        self._cycles = 0
        self._insts = 0
        self._manual_halted = False
        self._auto_halted = False
        self._halted = False
        self._halt_after_enable = False
        self._break_pc_enable = False
        self._halt_exc_enable = False
        self._halt_after_inst = 0
        self._break_pc = 0
        self._halt_exc_vec = 4
        self._halt_after_latched = False
        self._break_pc_latched = False
        self._halt_exc_latched = False
        self._halt_hit_pc = 0
        self._halt_hit_inst = 0

        # PC-trace backing
        self._pc_trace = [0] * 1024
        self._pc_trace_head = 0

        # SD backing store
        self.sd_blocks: Dict[int, bytes] = {}

        # SD command state-machine: counts remaining polls until DONE
        # After a command is accepted we set this to ~3 so the host
        # polls STATUS.busy=1 then STATUS.done=1.
        self._sd_poll_countdown = 0

        self._lock = threading.Lock()

        # ── Persistence wiring ───────────────────────────────────
        # Explicit arg wins; otherwise consult env var.  ``None``
        # disables persistence entirely (original behaviour).
        if state_path is None:
            env_path = os.environ.get(MOCK_STATE_ENV)
            self._state_path: Optional[str] = env_path or None
        else:
            self._state_path = state_path
        # ``_state_loaded`` guards against double-save in __del__ after
        # close(), and also marks whether we ever managed to read from
        # disk (useful for tests).
        self._state_loaded: bool = False
        self._state_closed: bool = False
        if self._state_path is not None:
            self._load_state()

    # ─── MMIO ────────────────────────────────────────────────────
    def mmio_read32(self, bar: int, offset: int) -> int:
        if bar not in self._mmio:
            return 0
        with self._lock:
            # Dynamic reads — let the CPU/SD sim react to "time" passing.
            # Every MMIO read advances the model by one tick.
            if not self._halted:
                self._cycles += 1
            self._advance_sd()
            return self._dyn_read(bar, offset)

    def mmio_write32(self, bar: int, offset: int, value: int) -> None:
        if bar not in self._mmio:
            self._mmio[bar] = {}
        with self._lock:
            value &= 0xFFFFFFFF
            self._dyn_write(bar, offset, value)

    # ─── DMA ─────────────────────────────────────────────────────
    def dma_read(self, addr: int, length: int) -> bytes:
        out = bytearray(length)
        for i in range(length):
            page, off = divmod(addr + i, 4096)
            p = self._dma_pages.get(page)
            if p is not None:
                out[i] = p[off]
        return bytes(out)

    def dma_write(self, addr: int, data: bytes) -> None:
        for i, b in enumerate(data):
            page, off = divmod(addr + i, 4096)
            p = self._dma_pages.get(page)
            if p is None:
                p = bytearray(4096)
                self._dma_pages[page] = p
            p[off] = b

    # ─── Internal helpers ────────────────────────────────────────
    def _dyn_read(self, bar: int, offset: int) -> int:
        # PC-trace ring reads (BAR 1, 0x10000..0x10FFC).  HEAD lives at
        # 0x11000 so it no longer aliases trace slot 960.
        if bar == 1 and offset == regs.OFF_DBG_PC_TRACE_HEAD:
            return self._pc_trace_head
        if bar == 1 and (offset & 0xFF000) == (regs.OFF_DBG_PC_TRACE_BASE & 0xFF000):
            idx = ((offset - regs.OFF_DBG_PC_TRACE_BASE) >> 2) & (len(self._pc_trace) - 1)
            return self._pc_trace[idx]
        # Cycle/inst counters
        if bar == 1 and offset == regs.OFF_DBG_CYCLE_LO:
            return self._cycles & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_CYCLE_HI:
            return (self._cycles >> 32) & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_INST_LO:
            return self._insts & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_INST_HI:
            return (self._insts >> 32) & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_STATUS:
            st = 0
            if self._halted:
                st |= regs.STS_HALTED
            else:
                st |= regs.STS_CPU_RUNNING
            if self._auto_halted:
                st |= regs.STS_AUTO_HALT
            st |= regs.STS_INIT_DONE_SEEN
            return st
        if bar == 1 and offset == regs.OFF_DBG_HALT_AFTER_LO:
            return self._halt_after_inst & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_HALT_AFTER_HI:
            return (self._halt_after_inst >> 32) & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_BREAK_PC:
            return self._break_pc & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_HALT_EXC_VEC:
            return self._halt_exc_vec & 0xFF
        if bar == 1 and offset == regs.OFF_DBG_HALT_CTL:
            v = 0
            if self._halt_after_enable:
                v |= regs.HALT_AFTER_ENABLE
            if self._break_pc_enable:
                v |= regs.HALT_BREAK_PC_ENABLE
            if self._halt_exc_enable:
                v |= regs.HALT_EXC_ENABLE
            if self._auto_halted:
                v |= regs.HALT_AUTO_LATCHED
            if self._halt_after_latched:
                v |= regs.HALT_AFTER_LATCHED
            if self._break_pc_latched:
                v |= regs.HALT_BREAK_PC_LATCHED
            if self._halt_exc_latched:
                v |= regs.HALT_EXC_LATCHED
            return v
        if bar == 1 and offset == regs.OFF_DBG_HALT_REASON:
            v = 0
            if self._manual_halted:
                v |= 1 << 0
            if self._halt_after_latched:
                v |= 1 << 1
            if self._break_pc_latched:
                v |= 1 << 2
            if self._halted:
                v |= 1 << 3
            if self._halt_after_enable:
                v |= 1 << 4
            if self._break_pc_enable:
                v |= 1 << 5
            if self._halt_exc_latched:
                v |= 1 << 6
            if self._halt_exc_enable:
                v |= 1 << 7
            return v
        if bar == 1 and offset == regs.OFF_DBG_HALT_HIT_PC:
            return self._halt_hit_pc & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_HALT_HIT_INST_LO:
            return self._halt_hit_inst & 0xFFFFFFFF
        if bar == 1 and offset == regs.OFF_DBG_HALT_HIT_INST_HI:
            return (self._halt_hit_inst >> 32) & 0xFFFFFFFF
        # Default: last-written or 0
        return self._mmio[bar].get(offset, 0)

    def _dyn_write(self, bar: int, offset: int, value: int) -> None:
        # Control register bits.
        if bar == 1 and offset == regs.OFF_DBG_CONTROL:
            self._manual_halted = bool(value & regs.CTL_HALT_REQ)
            self._refresh_halt()
            # step_pulse: fake a single-inst advance
            if value & regs.CTL_STEP_PULSE:
                self._cycles += 1
                if self._manual_halted:
                    self._push_pc_trace(0x40800000)
                    self._insts += 1
                    self._halt_hit_pc = 0x40800000
                    self._halt_hit_inst = self._insts
                else:
                    self._retire_pc(0x40800000)
            if value & regs.CTL_SOFT_RST:
                self._cycles = 0
                self._insts = 0
                self._pc_trace = [0] * 1024
                self._pc_trace_head = 0
            self._mmio[bar][offset] = value
            return
        if bar == 1 and offset == regs.OFF_DBG_REDIRECT_PC:
            self._mmio[bar][offset] = value
            return
        if bar == 1 and offset == regs.OFF_DBG_REDIRECT_TRIGGER:
            # Simulate the redirect taking effect: push a commit event with
            # the REDIRECT_PC onto the PC trace.
            pc = self._mmio[bar].get(regs.OFF_DBG_REDIRECT_PC, 0)
            self._retire_pc(pc)
            return
        if bar == 1 and offset == regs.OFF_DBG_HALT_AFTER_LO:
            self._halt_after_inst = (
                (self._halt_after_inst & 0xFFFF_FFFF_0000_0000) |
                (value & 0xFFFFFFFF))
            self._mmio[bar][offset] = value
            return
        if bar == 1 and offset == regs.OFF_DBG_HALT_AFTER_HI:
            self._halt_after_inst = (
                ((value & 0xFFFFFFFF) << 32) |
                (self._halt_after_inst & 0xFFFFFFFF))
            self._mmio[bar][offset] = value
            return
        if bar == 1 and offset == regs.OFF_DBG_BREAK_PC:
            self._break_pc = value
            self._mmio[bar][offset] = value
            return
        if bar == 1 and offset == regs.OFF_DBG_HALT_EXC_VEC:
            self._halt_exc_vec = value & 0xFF
            self._mmio[bar][offset] = value & 0xFF
            return
        if bar == 1 and offset == regs.OFF_DBG_HALT_CTL:
            self._halt_after_enable = bool(value & regs.HALT_AFTER_ENABLE)
            self._break_pc_enable = bool(value & regs.HALT_BREAK_PC_ENABLE)
            self._halt_exc_enable = bool(value & regs.HALT_EXC_ENABLE)
            if value & regs.HALT_CLEAR_LATCH:
                self._auto_halted = False
                self._halt_after_latched = False
                self._break_pc_latched = False
                self._halt_exc_latched = False
            self._refresh_halt()
            self._mmio[bar][offset] = value
            return
        if bar == 1 and offset == regs.OFF_SDP_CMD:
            self._sd_cmd(value)
            return
        if bar == 1 and offset == regs.OFF_SDP_LBA:
            self._mmio[bar][offset] = value
            return
        if bar == 1 and offset == regs.OFF_SDP_OWN_REQ:
            self._mmio[bar][offset] = value & 1
            # Ownership grants are auto-follow in the mock.
            st = self._mmio[bar].get(regs.OFF_SDP_STATUS, 0)
            if value & 1:
                st |= regs.SDP_STATUS_OWN_STATE
            else:
                st &= ~regs.SDP_STATUS_OWN_STATE
            self._mmio[bar][regs.OFF_SDP_STATUS] = st
            return
        self._mmio[bar][offset] = value

    # ─── PC trace ────────────────────────────────────────────────
    def _push_pc_trace(self, pc: int) -> None:
        self._pc_trace[self._pc_trace_head] = pc & 0xFFFFFFFF
        self._pc_trace_head = (self._pc_trace_head + 1) & (len(self._pc_trace) - 1)

    def _refresh_halt(self) -> None:
        self._halted = self._manual_halted or self._auto_halted

    def _retire_pc(self, pc: int) -> None:
        if self._halted:
            return
        self._push_pc_trace(pc)
        self._insts += 1
        if (self._halt_after_enable and self._halt_after_inst != 0 and
                self._insts >= self._halt_after_inst):
            self._auto_halted = True
            self._halt_after_latched = True
            self._halt_hit_pc = pc & 0xFFFFFFFF
            self._halt_hit_inst = self._insts
        if self._break_pc_enable and ((pc & 0xFFFFFFFF) == self._break_pc):
            self._auto_halted = True
            self._break_pc_latched = True
            self._halt_hit_pc = pc & 0xFFFFFFFF
            self._halt_hit_inst = self._insts
        self._refresh_halt()

    def _exception_boundary(self, vec: int, pc: int) -> None:
        if self._halted:
            return
        self._push_pc_trace(pc)
        self._insts += 1
        if self._halt_exc_enable and ((vec & 0xFF) == self._halt_exc_vec):
            self._auto_halted = True
            self._halt_exc_latched = True
            self._halt_hit_pc = pc & 0xFFFFFFFF
            self._halt_hit_inst = self._insts
        self._refresh_halt()

    # ─── SD-card sim ─────────────────────────────────────────────
    def _sd_cmd(self, cmd_word: int) -> None:
        if cmd_word & 0x80000000:
            # clear-error
            st = self._mmio[1].get(regs.OFF_SDP_STATUS, 0)
            st &= ~regs.SDP_STATUS_ERROR
            self._mmio[1][regs.OFF_SDP_STATUS] = st
            self._mmio[1][regs.OFF_SDP_ERR_CAUSE] = 0
            return
        cmd = cmd_word & 0xFF
        lba = self._mmio[1].get(regs.OFF_SDP_LBA, 0)
        # Count accepted commands
        self._mmio[1][regs.OFF_SDP_CMD_COUNT] = \
            (self._mmio[1].get(regs.OFF_SDP_CMD_COUNT, 0) + 1) & 0xFFFFFFFF
        # Enter busy
        st = self._mmio[1].get(regs.OFF_SDP_STATUS, 0)
        st = (st & ~(regs.SDP_STATUS_DONE | regs.SDP_STATUS_ERROR)) | regs.SDP_STATUS_BUSY
        self._mmio[1][regs.OFF_SDP_STATUS] = st
        self._sd_poll_countdown = 2

        if cmd == regs.SDP_CMD_READ:
            blk = self.sd_blocks.get(lba, b'\x00' * 512)
            # stash into scratch buffer words (little-endian)
            for widx in range(128):
                w = int.from_bytes(blk[widx*4:widx*4+4], 'little')
                self._mmio[1][regs.OFF_SDP_BUF_BASE + widx*4] = w
        elif cmd == regs.SDP_CMD_WRITE:
            buf = bytearray(512)
            for widx in range(128):
                w = self._mmio[1].get(regs.OFF_SDP_BUF_BASE + widx*4, 0)
                buf[widx*4:widx*4+4] = w.to_bytes(4, 'little')
            self.sd_blocks[lba] = bytes(buf)
        else:
            # unknown cmd: error cause 6
            self._mmio[1][regs.OFF_SDP_STATUS] = \
                (st & ~regs.SDP_STATUS_BUSY) | regs.SDP_STATUS_ERROR
            self._mmio[1][regs.OFF_SDP_ERR_CAUSE] = 6
            self._sd_poll_countdown = 0

    def _advance_sd(self) -> None:
        if self._sd_poll_countdown <= 0:
            return
        self._sd_poll_countdown -= 1
        if self._sd_poll_countdown == 0:
            st = self._mmio[1].get(regs.OFF_SDP_STATUS, 0)
            st = (st & ~regs.SDP_STATUS_BUSY) | regs.SDP_STATUS_DONE
            self._mmio[1][regs.OFF_SDP_STATUS] = st

    # ─── Persistence (pickle snapshot) ───────────────────────────
    def _load_state(self) -> None:
        """Load mmio + sd_blocks from ``self._state_path`` if present.

        Missing file / empty file / corrupt pickle is non-fatal: we
        log a warning and start from the default empty state.  This
        matches the "first invocation" semantics documented in the
        class docstring — a user setting ``M68KCTL_MOCK_STATE`` to a
        fresh path shouldn't crash before any write has happened.
        """
        if self._state_path is None:
            return
        try:
            with open(self._state_path, 'rb') as f:
                snap = pickle.load(f)
        except FileNotFoundError:
            return
        except (EOFError, pickle.UnpicklingError, OSError) as e:
            log.warning('MockDevice: could not load state from %s: %s; '
                        'starting fresh', self._state_path, e)
            return
        if not isinstance(snap, dict):
            log.warning('MockDevice: state file %s is not a dict '
                        '(got %s); starting fresh',
                        self._state_path, type(snap).__name__)
            return
        mmio = snap.get('mmio')
        if isinstance(mmio, dict):
            for bar, regfile in mmio.items():
                if not isinstance(regfile, dict):
                    continue
                # Merge, not replace — keep the pre-seeded version
                # magics as the fallback if the snapshot lacks them.
                self._mmio.setdefault(int(bar), {}).update(
                    {int(k): int(v) & 0xFFFFFFFF for k, v in regfile.items()})
        sd_blocks = snap.get('sd_blocks')
        if isinstance(sd_blocks, dict):
            for lba, blk in sd_blocks.items():
                if isinstance(blk, (bytes, bytearray)) and len(blk) == 512:
                    self.sd_blocks[int(lba)] = bytes(blk)
        self._state_loaded = True

    def _save_state(self) -> None:
        """Atomically persist mmio + sd_blocks to ``self._state_path``.

        Writes to ``<path>.tmp`` then ``os.replace`` to avoid truncating
        the on-disk file if the process crashes mid-write.  Parent
        directory is created on demand — matches the CLI convention
        of picking ``$XDG_STATE_HOME/m68kctl/`` which may not exist.
        """
        if self._state_path is None:
            return
        snap: Dict[str, Any] = {
            'mmio': {bar: dict(regfile) for bar, regfile in self._mmio.items()},
            'sd_blocks': dict(self.sd_blocks),
        }
        try:
            parent = os.path.dirname(self._state_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            tmp = self._state_path + '.tmp'
            with open(tmp, 'wb') as f:
                pickle.dump(snap, f, protocol=pickle.HIGHEST_PROTOCOL)
            os.replace(tmp, self._state_path)
        except OSError as e:
            log.warning('MockDevice: could not save state to %s: %s',
                        self._state_path, e)

    def close(self) -> None:
        # Guard against re-entry — context-manager __exit__ and an
        # explicit user close() shouldn't double-write.
        if self._state_closed:
            return
        self._state_closed = True
        if self._state_path is not None:
            with self._lock:
                self._save_state()

    def __del__(self) -> None:
        # __del__ is best-effort: if the interpreter is shutting down
        # the modules we depend on may be None already, so swallow.
        try:
            self.close()
        except Exception:
            pass

    # ─── Debugging helpers (mock-only) ───────────────────────────
    def inject_pc_retire(self, pc: int) -> None:
        """Pretend the core just retired an instruction at ``pc`` — useful
        in tests to check PC-trace wraparound without a full CPU sim."""
        with self._lock:
            self._retire_pc(pc)

    def inject_exception_boundary(self, vec: int, pc: int) -> None:
        """Pretend the core took a precise exception boundary."""
        with self._lock:
            self._exception_boundary(vec, pc)


# ─────────────────────────────────────────────────────────────────────────
# Helpers for the CLI
# ─────────────────────────────────────────────────────────────────────────
def default_mock_state_path() -> str:
    """Return the CLI-default pickle path for ``MockDevice`` persistence.

    Follows the XDG Base Directory spec: ``$XDG_STATE_HOME/m68kctl/
    mock_state.pkl``, falling back to ``~/.local/state/m68kctl/
    mock_state.pkl`` when ``XDG_STATE_HOME`` is unset.
    """
    xdg = os.environ.get('XDG_STATE_HOME')
    if not xdg:
        xdg = os.path.join(os.path.expanduser('~'), '.local', 'state')
    return os.path.join(xdg, 'm68kctl', 'mock_state.pkl')


def reset_mock_state(state_path: Optional[str] = None) -> bool:
    """Delete the pickle snapshot at ``state_path`` if it exists.

    Used by ``m68kctl --mock --reset-state``.  Returns True if a file
    was actually removed, False if there was nothing to remove.
    """
    path = state_path or os.environ.get(MOCK_STATE_ENV) or default_mock_state_path()
    try:
        os.unlink(path)
        return True
    except FileNotFoundError:
        return False
