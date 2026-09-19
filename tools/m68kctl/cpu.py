"""cpu.py — ``CpuDebug`` wrapper over BAR 1 debug_ctrl.

Exposes the user-facing half of ``docs/debug_pcie.md`` — control knobs
(halt, resume, step, soft_rst, redirect), observability (PC, cycles,
insts, ipc, mispred count), the PC trace ring, and the TIER 2 arch
register snapshot + commit log.

Graceful degradation: when we ask for a register from a tier the current
bitstream does not implement, the BAR just returns 0 (the debug_ctrl
shell drops unmapped offsets silently per the RTL).  We detect that and
log a warning containing which tier the caller hit, rather than faking
the presence of data that would never be observable on real hardware.
A ``tier`` hint is inferred from ``DBG_BUILD_ID`` in a future revision;
for now we trust the magic word and log warnings lazily on first access.
"""

from __future__ import annotations

import logging
import struct
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional

from . import regs
from .device import Device


log = logging.getLogger(__name__)


@dataclass
class CommitRecord:
    """One entry from the TIER 2 commit-log ring — see debug_pcie.md §COMMIT_LOG."""
    pc: int
    cycle: int
    packed: int           # {arch_dst, has_dst, flags_wr, flags_val, uop_type}
    result: int
    phys_dst_old: int
    phys_dst_new: int
    src_a: int
    src_b: int

    @property
    def arch_dst(self) -> int:   return self.packed & 0x1F
    @property
    def has_dst(self) -> bool:   return bool(self.packed & (1 << 5))
    @property
    def uop_type(self) -> int:   return (self.packed >> 18) & 0xF


@dataclass
class HaltDebugStatus:
    """Programmable halt/breakpoint state from the TIER 1 debug block."""
    control: int
    reason: int
    hit_pc: int
    hit_inst: int
    exc_vec: int
    core040: bool = False
    break_enable_mask: int = 0
    exception_mask_nonzero: bool = False

    @property
    def halt_after_enabled(self) -> bool:
        return bool(self.control & regs.HALT_AFTER_ENABLE)

    @property
    def break_pc_enabled(self) -> bool:
        return (bool(self.break_enable_mask) if self.core040 else
                bool(self.control & regs.HALT_BREAK_PC_ENABLE))

    @property
    def halt_exc_enabled(self) -> bool:
        return (self.exception_mask_nonzero if self.core040 else
                bool(self.control & regs.HALT_EXC_ENABLE))

    @property
    def auto_latched(self) -> bool:
        return ((self.reason & 0x7) in (3, 5, 6) if self.core040 else
                bool(self.control & regs.HALT_AUTO_LATCHED))

    @property
    def halt_after_latched(self) -> bool:
        return ((self.reason & 0x7) == 3 if self.core040 else
                bool(self.control & regs.HALT_AFTER_LATCHED))

    @property
    def break_pc_latched(self) -> bool:
        return ((self.reason & 0x7) == 5 if self.core040 else
                bool(self.control & regs.HALT_BREAK_PC_LATCHED))

    @property
    def halt_exc_latched(self) -> bool:
        return ((self.reason & 0x7) == 6 if self.core040 else
                bool(self.control & regs.HALT_EXC_LATCHED))


class CpuDebug:
    """Control + observability over the debug_ctrl AXI-Lite window."""

    def __init__(self, device: Device, *, verify_magic: bool = True):
        self.dev = device
        self._warned_tier2: bool = False
        v = self.dev.mmio_read32(1, regs.OFF_DBG_VERSION)
        self._core040 = v == 0xDEB6_0100
        if verify_magic:
            if (v & 0xFFFF0000) != (regs.DBG_VERSION_MAGIC & 0xFFFF0000):
                raise RuntimeError(
                    f'bad DBG_VERSION magic 0x{v:08x}: '
                    'bitstream may not include debug_ctrl.v or is out of date')

    # ─── Primitives ──────────────────────────────────────────────
    def _r(self, off: int) -> int:
        return self.dev.mmio_read32(1, off)

    def _w(self, off: int, val: int) -> None:
        self.dev.mmio_write32(1, off, val)

    # ─── Identity + status ───────────────────────────────────────
    @property
    def version(self) -> int:      return self._r(regs.OFF_DBG_VERSION)
    @property
    def build_id(self) -> int:     return self._r(regs.OFF_DBG_BUILD_ID)
    @property
    def pc(self) -> int:           return self._r(regs.OFF_DBG_PC)
    @property
    def last_pc(self) -> int:      return self._r(regs.OFF_DBG_LAST_PC)
    @property
    def status(self) -> int:       return self._r(regs.OFF_DBG_STATUS)
    @property
    def halted(self) -> bool:      return bool(self.status & regs.STS_HALTED)
    @property
    def running(self) -> bool:     return bool(self.status & regs.STS_CPU_RUNNING)
    @property
    def init_done_seen(self) -> bool:
        return bool(self.status & regs.STS_INIT_DONE_SEEN)

    @property
    def cycles(self) -> int:
        lo = self._r(regs.OFF_DBG_CYCLE_LO)
        hi = self._r(regs.OFF_DBG_CYCLE_HI)
        return (hi << 32) | lo

    @property
    def insts(self) -> int:
        lo = self._r(regs.OFF_DBG_INST_LO)
        hi = self._r(regs.OFF_DBG_INST_HI)
        return (hi << 32) | lo

    @property
    def ipc(self) -> float:
        c = self.cycles
        return (self.insts / c) if c else 0.0

    @property
    def mispred_count(self) -> int:
        return self._r(regs.OFF_DBG_MISPRED_COUNT)

    @property
    def flush_count(self) -> int:
        return self._r(regs.OFF_DBG_FLUSH_COUNT)

    @property
    def exc_count(self) -> int:
        return self._r(regs.OFF_DBG_EXC_COUNT)

    @property
    def exc_vec(self) -> int:
        return self._r(regs.OFF_DBG_EXC_VEC) & 0xFF

    @property
    def exc_pc(self) -> int:
        return self._r(regs.OFF_DBG_EXC_PC)

    @property
    def reset_cause(self) -> int:
        return self._r(regs.OFF_DBG_RESET_CAUSE) & 0x3

    # ─── Control ─────────────────────────────────────────────────
    def halt(self) -> None:
        self._w(regs.OFF_DBG_CONTROL, regs.CTL_HALT_REQ)

    def resume(self) -> None:
        self.clear_auto_halt()
        self._w(regs.OFF_DBG_CONTROL, 0)

    def step(self) -> None:
        """Pulse step.  Preserves current halt state."""
        if not self.halted:
            raise RuntimeError('CPU must be halted before single-step')
        self._w(regs.OFF_DBG_CONTROL, regs.CTL_HALT_REQ | regs.CTL_STEP_PULSE)

    def soft_rst(self) -> None:
        self._w(regs.OFF_DBG_CONTROL, regs.CTL_SOFT_RST)

    def reset_halt(self) -> None:
        """Pulse CPU soft reset while leaving the JTAG manual halt latch set."""
        self._w(regs.OFF_DBG_CONTROL, regs.CTL_HALT_REQ | regs.CTL_SOFT_RST)

    def init_done(self, enable: bool = True) -> None:
        bits = regs.CTL_INIT_DONE_OVR if enable else 0
        if self.halted:
            bits |= regs.CTL_HALT_REQ
        self._w(regs.OFF_DBG_CONTROL, bits)

    def redirect(self, pc: int) -> None:
        self._w(regs.OFF_DBG_REDIRECT_PC, pc & 0xFFFFFFFF)
        self._w(regs.OFF_DBG_REDIRECT_TRIGGER, 1)

    def inject_irq(self, level: int) -> None:
        if not 1 <= level <= 7:
            raise ValueError(f'IRQ level {level} out of range 1..7')
        self._w(regs.OFF_DBG_IRQ_INJECT, level)

    def halt_status(self) -> HaltDebugStatus:
        lo = self._r(regs.OFF_DBG_HALT_HIT_INST_LO)
        hi = self._r(regs.OFF_DBG_HALT_HIT_INST_HI)
        bp_mask = self._r(regs.OFF_DBG_BREAK_PC_CTRL) & 0xF if self._core040 else 0
        exc_mask_nonzero = (any(self._r(regs.OFF_DBG_HALT_EXC_MASK0 + i * 4)
                                for i in range(8)) if self._core040 else False)
        return HaltDebugStatus(
            control=self._r(regs.OFF_DBG_HALT_CTL),
            reason=self._r(regs.OFF_DBG_HALT_REASON),
            hit_pc=self._r(regs.OFF_DBG_HALT_HIT_PC),
            hit_inst=(hi << 32) | lo,
            exc_vec=self._r(regs.OFF_DBG_EXC_VEC if self._core040 else
                            regs.OFF_DBG_HALT_EXC_VEC) & 0xFF,
            core040=self._core040,
            break_enable_mask=bp_mask,
            exception_mask_nonzero=exc_mask_nonzero,
        )

    def _halt_enable_bits(self) -> int:
        mask = regs.HALT_AFTER_ENABLE
        if not self._core040:
            mask |= regs.HALT_BREAK_PC_ENABLE | regs.HALT_EXC_ENABLE
        return self._r(regs.OFF_DBG_HALT_CTL) & mask

    def clear_auto_halt(self) -> None:
        """Clear latched halt-after/breakpoint/exception state, preserving enables."""
        self._w(regs.OFF_DBG_HALT_CTL,
                self._halt_enable_bits() | regs.HALT_CLEAR_LATCH)

    def set_halt_after(self, inst_count: int, *, enable: bool = True,
                       clear: bool = True) -> None:
        if inst_count < 0 or inst_count > 0xFFFF_FFFF_FFFF_FFFF:
            raise ValueError(f'instruction count out of range: {inst_count}')
        self._w(regs.OFF_DBG_HALT_AFTER_LO, inst_count & 0xFFFFFFFF)
        self._w(regs.OFF_DBG_HALT_AFTER_HI, (inst_count >> 32) & 0xFFFFFFFF)
        bits = self._halt_enable_bits()
        if enable:
            bits |= regs.HALT_AFTER_ENABLE
        else:
            bits &= ~regs.HALT_AFTER_ENABLE
        if clear:
            bits |= regs.HALT_CLEAR_LATCH
        self._w(regs.OFF_DBG_HALT_CTL, bits)

    def set_breakpoint(self, pc: int, *, enable: bool = True,
                       clear: bool = True, slot: int = 0) -> None:
        if slot < 0 or slot > 3:
            raise ValueError(f'breakpoint slot out of range: {slot}')
        bp_off = (regs.OFF_DBG_BREAK_PC if slot == 0 else
                  regs.OFF_DBG_BREAK_PC1 + (slot - 1) * 4)
        self._w(bp_off, pc & 0xFFFFFFFF)
        if self._core040:
            enables = self._r(regs.OFF_DBG_BREAK_PC_CTRL) & 0xF
            enables = ((enables | (1 << slot)) if enable else
                       (enables & ~(1 << slot)))
            self._w(regs.OFF_DBG_BREAK_PC_CTRL, enables)
        bits = self._halt_enable_bits()
        if not self._core040:
            if enable:
                bits |= regs.HALT_BREAK_PC_ENABLE
            else:
                bits &= ~regs.HALT_BREAK_PC_ENABLE
        if clear:
            bits |= regs.HALT_CLEAR_LATCH
        self._w(regs.OFF_DBG_HALT_CTL, bits)

    def set_halt_exception(self, vec: int = 4, *, enable: bool = True,
                           clear: bool = True) -> None:
        if vec < 0 or vec > 0xFF:
            raise ValueError(f'exception vector out of range: {vec}')
        if self._core040:
            lane_off = regs.OFF_DBG_HALT_EXC_MASK0 + ((vec >> 5) & 7) * 4
            lane = self._r(lane_off)
            lane = ((lane | (1 << (vec & 31))) if enable else
                    (lane & ~(1 << (vec & 31))))
            self._w(lane_off, lane)
        else:
            self._w(regs.OFF_DBG_HALT_EXC_VEC, vec & 0xFF)
        bits = self._halt_enable_bits()
        if not self._core040:
            if enable:
                bits |= regs.HALT_EXC_ENABLE
            else:
                bits &= ~regs.HALT_EXC_ENABLE
        if clear:
            bits |= regs.HALT_CLEAR_LATCH
        self._w(regs.OFF_DBG_HALT_CTL, bits)

    # ─── PC trace ────────────────────────────────────────────────
    def pc_trace(self) -> List[int]:
        """Return PC trace ring in chronological order (oldest → newest).

        Head points to the next slot to be written, so element [head] is
        the oldest one (or 0 if the ring hasn't wrapped yet).
        """
        head = self._r(regs.OFF_DBG_PC_TRACE_HEAD) & (regs.DBG_PC_TRACE_DEPTH - 1)
        # Read the whole ring as raw bytes — faster than 1024 MMIO reads.
        n = regs.DBG_PC_TRACE_DEPTH
        entries: List[int] = []
        for i in range(n):
            entries.append(self._r(regs.OFF_DBG_PC_TRACE_BASE + i * 4))
        return entries[head:] + entries[:head]

    # ─── TIER 2 — arch register snapshot ────────────────────────
    def regs(self) -> Dict[str, int]:
        """Return the architectural register file snapshot.

        Reads D0-D7/A0-A7 from the live snap-chain window
        (OFF_DBG_LIVE_D0 / OFF_DBG_LIVE_A0) which walks
        prf[crat[arch]] inside m68k_core.  VBR/SR/A7 also come from the
        live window.  Other status regs (USP/SSP/ISP/CACR) still come
        from the arch_shadow_* path because they have no live readback
        port today; they only reflect what the host last wrote via
        load_arch_state().  PC is the live `dbg_pc`.

        Stable only while the core is halted; racy under free-run.
        """
        out: Dict[str, int] = {}
        for i in range(8):
            out[f'D{i}'] = self._r(regs.OFF_DBG_LIVE_D0 + i * 4)
        for i in range(8):
            out[f'A{i}'] = self._r(regs.OFF_DBG_LIVE_A0 + i * 4)
        out['SR']  = self._r(regs.OFF_DBG_LIVE_SR) & 0xFFFF
        out['VBR'] = self._r(regs.OFF_DBG_LIVE_VBR)
        # Shadow-only (no live port today)
        for name, off in [
            ('USP',  regs.OFF_DBG_USP),
            ('SSP',  regs.OFF_DBG_SSP),
            ('ISP',  regs.OFF_DBG_ISP),
            ('CACR', regs.OFF_DBG_CACR),
        ]:
            out[name] = self._r(off)
        out['PC'] = self.pc
        # The m68k040 core advertises live architectural readback explicitly.
        # A zero-filled unmapped window must not be reported as a real dump.
        out['_tier'] = (2 if self._core040 and
                        (self._r(regs.OFF_DBG_FEATURES) & (1 << 10)) else 1)
        return out

    def load_arch_state(self, state: Mapping[str, int], *, apply: bool = True) -> int:
        """Write the halt-time architectural shadow state and optionally apply it.

        The core must already be halted.  Keys are D0-D7, A0-A7, PC, SR,
        VBR, USP, SSP, ISP, CACR, SFC, DFC, and MMU CR names ITT0/ITT1/
        DTT0/DTT1/TC/URP/SRP.  Returns ARCH_STATUS after the optional apply.
        """
        for i in range(8):
            if f'D{i}' in state:
                self._w(regs.OFF_DBG_D0 + i * 4, state[f'D{i}'])
            if f'A{i}' in state:
                self._w(regs.OFF_DBG_A0 + i * 4, state[f'A{i}'])

        for name, off in [
            ('USP', regs.OFF_DBG_USP),
            ('SSP', regs.OFF_DBG_SSP),
            ('ISP', regs.OFF_DBG_ISP),
            ('SR', regs.OFF_DBG_SR),
            ('VBR', regs.OFF_DBG_VBR),
            ('CACR', regs.OFF_DBG_CACR),
            ('TC', regs.OFF_DBG_TC),
            ('ITT0', regs.OFF_DBG_ITT0),
            ('ITT1', regs.OFF_DBG_ITT1),
            ('DTT0', regs.OFF_DBG_DTT0),
            ('DTT1', regs.OFF_DBG_DTT1),
            ('URP', regs.OFF_DBG_URP),
            ('SRP', regs.OFF_DBG_SRP),
            ('PC', regs.OFF_DBG_ARCH_PC),
            ('SFC', regs.OFF_DBG_SFC),
            ('DFC', regs.OFF_DBG_DFC),
        ]:
            if name in state:
                self._w(off, state[name])

        if apply:
            self._w(regs.OFF_DBG_ARCH_APPLY,
                    regs.ARCH_APPLY_CLEAR_STATUS | regs.ARCH_APPLY_START)
            for _ in range(128):
                status = self._r(regs.OFF_DBG_ARCH_STATUS)
                if not (status & regs.ARCH_STATUS_BUSY):
                    return status
        return self._r(regs.OFF_DBG_ARCH_STATUS)

    # ─── TIER 2 — commit log ring ───────────────────────────────
    def commit_log(self) -> List[CommitRecord]:
        """Return the 256-deep commit log in chronological order."""
        head = self._r(regs.OFF_DBG_COMMIT_LOG_HEAD) & (regs.DBG_COMMIT_LOG_DEPTH - 1)
        # Pull raw bytes from the ring.
        raw = bytearray(regs.DBG_COMMIT_LOG_DEPTH * regs.DBG_COMMIT_LOG_REC_SIZE)
        for i in range(len(raw) // 4):
            w = self._r(regs.OFF_DBG_COMMIT_LOG_BASE + i * 4)
            struct.pack_into('<I', raw, i * 4, w)

        all_nonzero = any(raw)
        if not all_nonzero and not self._warned_tier2:
            log.warning(
                'CpuDebug.commit_log(): ring all-zero — bitstream may only '
                'implement TIER 1; commit log not populated.')
            self._warned_tier2 = True

        out: List[CommitRecord] = []
        for i in range(regs.DBG_COMMIT_LOG_DEPTH):
            idx = (head + i) % regs.DBG_COMMIT_LOG_DEPTH
            off = idx * regs.DBG_COMMIT_LOG_REC_SIZE
            pc, cy, pkd, val, phd, src_a, src_b, _rsv = \
                struct.unpack_from('<8I', raw, off)
            out.append(CommitRecord(
                pc=pc, cycle=cy, packed=pkd, result=val,
                phys_dst_old=phd & 0x3F, phys_dst_new=(phd >> 6) & 0x3F,
                src_a=src_a, src_b=src_b))
        return out
