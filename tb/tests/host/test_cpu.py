"""Tests for CpuDebug against MockDevice."""

import unittest
import sys
from pathlib import Path

# Resolve tools/ for both pytest and plain `python -m unittest`.
_TOOLS = Path(__file__).resolve().parents[3] / 'tools'
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from m68kctl import MockDevice, CpuDebug
from m68kctl import regs


class CpuHaltResumeTests(unittest.TestCase):
    def test_cycle_counter_advances_then_halts(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        # Cycles start at 0, tick each MMIO read in the mock.
        c0 = cpu.cycles
        c1 = cpu.cycles
        c2 = cpu.cycles
        self.assertLess(c0, c1)
        self.assertLess(c1, c2)

        # Halt stops cycle advance.
        cpu.halt()
        # Let the halt take effect (a single read may still tick)
        _ = cpu.cycles
        frozen = cpu.cycles
        for _ in range(5):
            cpu.cycles
        self.assertEqual(cpu.cycles, frozen)

        # Resume advances again.
        cpu.resume()
        self.assertGreater(cpu.cycles, frozen)

    def test_halt_status_bit(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        self.assertFalse(cpu.halted)
        self.assertTrue(cpu.running)
        cpu.halt()
        self.assertTrue(cpu.halted)
        cpu.resume()
        self.assertFalse(cpu.halted)

    def test_version_magic(self):
        dev = MockDevice()
        cpu = CpuDebug(dev, verify_magic=True)
        self.assertEqual(cpu.version & 0xFFFF0000, 0xDEB60000)

    def test_step_advances_insts_only_while_halted(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        with self.assertRaises(RuntimeError):
            cpu.step()
        cpu.halt()
        i0 = cpu.insts
        cpu.step()
        # A step pulse bumps insts on the mock.
        self.assertGreater(cpu.insts, i0)


class PcTraceTests(unittest.TestCase):
    def test_trace_wraparound(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        # Inject 2048 retires; the ring is 1024 deep, so head wraps exactly
        # twice back to 0, and the oldest entry visible is retire #1024.
        for pc in range(2048):
            dev.inject_pc_retire(0x40800000 + pc * 2)
        trace = cpu.pc_trace()
        self.assertEqual(len(trace), regs.DBG_PC_TRACE_DEPTH)
        # head = 0 after 2048 retires; order is entries[0:]+entries[:0]
        # Slot 0 = retire 1024, slot 1023 = retire 2047.
        self.assertEqual(trace[0],  0x40800000 + 1024 * 2)
        self.assertEqual(trace[-1], 0x40800000 + 2047 * 2)

    def test_trace_partial_fill(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        # Only 10 retires — head is at 10, rest of the ring is zeros.
        for pc in range(10):
            dev.inject_pc_retire(0xCAFE0000 + pc)
        trace = cpu.pc_trace()
        self.assertEqual(trace[-10:], [0xCAFE0000 + i for i in range(10)])
        for i, v in enumerate(trace[:-10]):
            self.assertEqual(v, 0, f'slot {i} (reorder pos) = {v:#x}, expected 0')


class ProgrammableHaltTests(unittest.TestCase):
    @staticmethod
    def _core040():
        class Core040Device:
            def __init__(self):
                self.words = {regs.OFF_DBG_VERSION: 0xDEB60100}

            def mmio_read32(self, bar, offset):
                return self.words.get(offset, 0)

            def mmio_write32(self, bar, offset, value):
                self.words[offset] = value & 0xFFFFFFFF

        dev = Core040Device()
        return dev, CpuDebug(dev)

    def test_halt_after_latches_and_resume_clears(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        cpu.set_halt_after(3)
        for pc in [0x40800000, 0x40800002, 0x40800004]:
            dev.inject_pc_retire(pc)
        self.assertTrue(cpu.halted)
        hs = cpu.halt_status()
        self.assertTrue(hs.auto_latched)
        self.assertTrue(hs.halt_after_latched)
        self.assertEqual(hs.hit_pc, 0x40800004)
        self.assertEqual(hs.hit_inst, 3)
        cpu.set_halt_after(6)
        cpu.resume()
        self.assertFalse(cpu.halted)
        for pc in [0x40800006, 0x40800008, 0x4080000A]:
            dev.inject_pc_retire(pc)
        self.assertTrue(cpu.halted)

    def test_break_pc_latches(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        cpu.set_breakpoint(0x40800006)
        for pc in [0x40800000, 0x40800002, 0x40800006]:
            dev.inject_pc_retire(pc)
        self.assertTrue(cpu.halted)
        hs = cpu.halt_status()
        self.assertTrue(hs.break_pc_latched)
        self.assertEqual(hs.hit_pc, 0x40800006)

    def test_halt_exception_latches_illegal_by_default(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        cpu.set_halt_exception()
        dev.inject_exception_boundary(4, 0x4080000C)
        self.assertTrue(cpu.halted)
        hs = cpu.halt_status()
        self.assertTrue(hs.halt_exc_enabled)
        self.assertTrue(hs.halt_exc_latched)
        self.assertEqual(hs.exc_vec, 4)
        self.assertEqual(hs.hit_pc, 0x4080000C)

    def test_reset_halt_keeps_manual_hold_asserted(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        cpu.reset_halt()
        self.assertTrue(cpu.halted)
        dev.inject_pc_retire(0x40800000)
        self.assertEqual(cpu.insts, 0)

    def test_core040_breakpoint_programs_selected_slot_and_enable_mask(self):
        dev, cpu = self._core040()
        cpu.set_breakpoint(0x40801234, slot=2)
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_BREAK_PC2), 0x40801234)
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_BREAK_PC_CTRL), 0x4)
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_HALT_CTL),
                         regs.HALT_CLEAR_LATCH)
        self.assertTrue(cpu.halt_status().break_pc_enabled)

    def test_core040_exception_programs_mask_lane_not_reserved_vector(self):
        dev, cpu = self._core040()
        cpu.set_halt_exception(33)
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_HALT_EXC_MASK0 + 4), 0x2)
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_HALT_EXC_VEC), 0)
        self.assertTrue(cpu.halt_status().halt_exc_enabled)

    def test_core040_halt_latches_decode_primary_reason_code(self):
        dev, cpu = self._core040()
        dev.mmio_write32(1, regs.OFF_DBG_HALT_REASON, 5)
        hs = cpu.halt_status()
        self.assertTrue(hs.auto_latched)
        self.assertTrue(hs.break_pc_latched)
        self.assertFalse(hs.halt_after_latched)


class CpuRegsTests(unittest.TestCase):
    def test_regs_graceful_degrade_on_tier1_only(self):
        dev = MockDevice()
        cpu = CpuDebug(dev)
        r = cpu.regs()
        # TIER 2 reads zero on the mock; the _tier key reports 1.
        self.assertEqual(r.get('_tier'), 1)
        for reg in ['D0', 'D7', 'A0', 'A7', 'SR', 'VBR']:
            self.assertEqual(r[reg], 0)


if __name__ == '__main__':
    unittest.main()
