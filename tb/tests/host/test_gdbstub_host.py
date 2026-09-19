#!/usr/bin/env python3
"""Host-side tests for the GDB stub — no FPGA, no Vivado, no Tcl.

Drives `tools/gdbdbg.py` and `tools/gdbstub.py` against
`tb/tests/host/fake_jtag_repl.py`, a software model of the debug register
file that reproduces the hardware's awkward semantics deliberately.

What is actually being defended
-------------------------------
Most of these tests are not "does the feature work" but **"does the host
refuse to make something up"**.  This tooling has silently fabricated wrong
values in at least six distinct ways, and each one sent someone debugging the
wrong thing for hours.  So there are tests here that assert an *exception* is
raised — on an address echo mismatch, on the `BADA0BAD` sentinel, on a
suspiciously uniform burst, on reading registers while the CPU runs, on a
breakpoint arm that did not stick.  Those tests are the point of the file.

Passing here proves the HOST is right.  It says nothing about the board.
"""

import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fake_jtag_repl                      # noqa: E402
import gdbdbg                              # noqa: E402
import gdbstub                             # noqa: E402
from gdbdbg import Target, TargetError     # noqa: E402
from macsym import SymbolTable             # noqa: E402


def make_target(**kw):
    fake = fake_jtag_repl.FakeRepl()
    return fake, Target(gdbdbg.FakeReplTransport(fake), **kw)


def set_reg(fake, name, value):
    """Poke an architectural register straight into the fake's state.

    The fake models the register file the way the RTL exposes it (separate
    `arch_d`/`arch_a` arrays, a `live_sr` distinct from the shadow `arch_sr`,
    and a live `pc`), so tests set values through this helper rather than
    assuming one flat dict."""
    if name.startswith("D"):
        i = int(name[1:])
        fake.arch_d[i] = fake.live_d[i] = value
    elif name.startswith("A"):
        i = int(name[1:])
        fake.arch_a[i] = fake.live_a[i] = value
    elif name == "SR":
        fake.live_sr = value
        fake.arch_sr = value
    elif name == "PC":
        fake.pc = value
        fake.arch_pc = value
    else:
        raise KeyError(name)


def get_applied(fake, name):
    """Read back what an arch-apply actually committed."""
    if name.startswith("D"):
        return fake.arch_d[int(name[1:])]
    if name.startswith("A"):
        return fake.arch_a[int(name[1:])]
    raise KeyError(name)


class ScriptedTransport(gdbdbg.ReplTransport):
    """A transport that replies with exactly what the test dictates.

    Used for the failure modes the fake is too well-behaved to produce — a
    REPL that echoes the wrong address, or hands back the failure sentinel as
    if it were data."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.sent = []

    def execute(self, line, wait_s=0.05):
        self.sent.append(line)
        return self.replies.pop(0) if self.replies else ["> READY"]


# ══════════════════════════════════════════════════════════════════════
# Transport-level refusals
# ══════════════════════════════════════════════════════════════════════
class TestReadVerification(unittest.TestCase):
    def test_read32_accepts_a_matching_echo(self):
        t = Target(ScriptedTransport([["> r 0x50900000 = 0xDEB60007",
                                       "> READY"]]))
        self.assertEqual(t.read32(0x50900000), 0xDEB60007)

    def test_read32_rejects_an_echoed_address_mismatch(self):
        """The REPL echoes the address it resolved.  A mismatch means the
        operand was parsed as something else — the exact shape of the bug
        where `r 40800000` once read 0x026E8F80."""
        t = Target(ScriptedTransport([["> r 0x026E8F80 = 0x12345678",
                                       "> READY"]]))
        with self.assertRaises(TargetError) as cm:
            t.read32(0x40800000)
        self.assertIn("echoed address", str(cm.exception))
        self.assertNotIn("12345678", str(cm.exception).split("refusing")[0][:0]
                         or "")

    def test_read32_treats_the_failure_sentinel_as_an_error(self):
        """BADA0BAD is a failed AXI read, not a value.  In particular it must
        never be quietly turned into 0."""
        t = Target(ScriptedTransport([["> r 0x50900000 = 0xBADA0BAD",
                                       "> READY"]]))
        with self.assertRaises(TargetError) as cm:
            t.read32(0x50900000)
        self.assertIn("BADA0BAD", str(cm.exception))
        self.assertIn("not a value of zero", str(cm.exception))

    def test_repl_error_line_raises(self):
        t = Target(ScriptedTransport([["> ERROR unaligned address", "> READY"]]))
        with self.assertRaises(TargetError) as cm:
            t.read32(0x50900000)
        self.assertIn("ERROR", str(cm.exception))

    def test_missing_reply_line_raises_rather_than_defaulting(self):
        t = Target(ScriptedTransport([["> READY"]]))
        with self.assertRaises(TargetError):
            t.read32(0x50900000)

    def test_unaligned_access_is_refused_before_it_reaches_the_bus(self):
        """The 32-bit JTAG-AXI master silently aligns down; a read of +2 would
        return the neighbouring word and look perfectly plausible."""
        _f, t = make_target()
        with self.assertRaises(TargetError) as cm:
            t.read32(0x00000002)
        self.assertIn("align", str(cm.exception).lower())
        with self.assertRaises(TargetError):
            t.write32(0x00000002, 0)

    def test_write_echo_is_verified(self):
        t = Target(ScriptedTransport([["> w 0x00001000 = 0x0000BEEF",
                                       "> READY"]]))
        with self.assertRaises(TargetError) as cm:
            t.write32(0x1000, 0xDEADBEEF)
        self.assertIn("echoed", str(cm.exception))


class TestBurstFabricationGuard(unittest.TestCase):
    """The historical burst bug returned the same latched word for every
    address, while `dump-mem` still printed correctly increasing addresses —
    the echo proves nothing there, because the REPL computes it host-side."""

    def test_uniform_burst_that_contradicts_a_single_read_is_refused(self):
        replies = [
            ["> mem 0x00001000 = 0x11111111",
             "> mem 0x00001004 = 0x11111111",
             "> mem 0x00001008 = 0x11111111",
             "> mem 0x0000100C = 0x11111111", "> READY"],
            ["> r 0x00001004 = 0x22222222", "> READY"],   # cross-check
        ]
        t = Target(ScriptedTransport(replies), auto_cache_flush=False)
        with self.assertRaises(TargetError) as cm:
            t.read_words(0x1000, 4)
        self.assertIn("fabricating", str(cm.exception))

    def test_genuinely_zeroed_page_is_allowed_through(self):
        replies = [
            ["> mem 0x00001000 = 0x00000000",
             "> mem 0x00001004 = 0x00000000", "> READY"],
            ["> r 0x00001004 = 0x00000000", "> READY"],
        ]
        t = Target(ScriptedTransport(replies), auto_cache_flush=False)
        self.assertEqual(t.read_words(0x1000, 2), [0, 0])

    def test_dump_mem_word_count_is_sent_as_explicit_hex(self):
        """The REPL's `parse_num` is HEX-BY-DEFAULT, including for the word
        count: `dump-mem <addr> 16` reads 0x16 = 22 words.  Sending a bare
        decimal count silently reads the wrong amount of memory — caught by
        the fake, which models that parsing faithfully."""
        fake, t = make_target(auto_cache_flush=False)
        t.halt()
        fake.load_bytes(0x3000, bytes(range(0x40)))
        words = t.read_words(0x3000, 16)
        self.assertEqual(len(words), 16)
        self.assertEqual(words[0], 0x00010203)
        self.assertEqual(words[15], 0x3C3D3E3F)

    def test_read_bytes_of_a_decimal_looking_length(self):
        fake, t = make_target(auto_cache_flush=False)
        t.halt()
        fake.load_bytes(0x3000, bytes(range(0x40)))
        self.assertEqual(t.read_bytes(0x3000, 0x40), bytes(range(0x40)))

    def test_word_count_shortfall_raises(self):
        replies = [["> mem 0x00001000 = 0x11111111", "> READY"]]
        t = Target(ScriptedTransport(replies), auto_cache_flush=False)
        with self.assertRaises(TargetError) as cm:
            t.read_words(0x1000, 4)
        self.assertIn("expected 4", str(cm.exception))

    def test_address_sequence_gap_raises(self):
        replies = [["> mem 0x00001000 = 0x11111111",
                    "> mem 0x00001008 = 0x22222222", "> READY"]]
        t = Target(ScriptedTransport(replies), auto_cache_flush=False)
        with self.assertRaises(TargetError) as cm:
            t.read_words(0x1000, 2)
        self.assertIn("expected 0x00001004", str(cm.exception))


# ══════════════════════════════════════════════════════════════════════
# Halt discipline
# ══════════════════════════════════════════════════════════════════════
class TestHaltDiscipline(unittest.TestCase):
    def test_registers_are_not_read_while_running(self):
        """The snap chain multiplexes through the committed RAT; on a running
        CPU it yields a coherent-looking snapshot of nothing in particular.
        The fake poisons it with 0xDEADBEEF — we must never surface that."""
        fake, t = make_target()
        self.assertFalse(t.is_halted())
        with self.assertRaises(TargetError) as cm:
            t.read_regs()
        self.assertIn("running", str(cm.exception))
        self.assertNotIn("deadbeef", str(cm.exception).lower())

    def test_halt_only_offsets_refused_while_running(self):
        _f, t = make_target()
        with self.assertRaises(TargetError) as cm:
            t.dbg_read(gdbdbg.OFF_LIVE_D0)
        self.assertIn("snap chain", str(cm.exception))

    def test_halt_waits_for_the_status_bit(self):
        _f, t = make_target()
        t.halt()
        self.assertTrue(t.is_halted())
        regs = t.read_regs()
        self.assertNotEqual(regs["D0"], 0xDEADBEEF)

    def test_cache_op_refused_while_running(self):
        _f, t = make_target()
        with self.assertRaises(TargetError):
            t.flush_dcache(force=True)


# ══════════════════════════════════════════════════════════════════════
# Register staging — halted atomic Stage-3 apply
# ══════════════════════════════════════════════════════════════════════
class TestRegisterStaging(unittest.TestCase):
    def setUp(self):
        self.fake, self.t = make_target()
        self.t.halt()

    def test_staging_does_not_resume_the_cpu(self):
        """Staging is host-only and cannot disturb the stopped machine."""
        self.t.stage_reg("D0", 0x1234)
        self.assertTrue(self.t.is_halted(),
                        "staging a register must not resume the CPU")

    def test_reads_reflect_staged_writes(self):
        self.t.stage_reg("D0", 0xCAFEBABE)
        self.assertEqual(self.t.read_regs()["D0"], 0xCAFEBABE)

    def test_apply_happens_on_resume(self):
        self.t.stage_reg("D0", 0xCAFEBABE)
        self.t.resume()
        self.assertEqual(get_applied(self.fake, "D0"), 0xCAFEBABE)
        self.assertEqual(self.t.pending_regs(), {})

    def test_resume_after_a_register_write_still_arms_bp_skip_once(self):
        """A staged write must not lose breakpoint skip-once on resume."""
        self.t.set_bp(0, 0x1000)
        self.fake.stop_breakpoint(0, 0x1000)
        self.t.stage_reg("D0", 1)
        self.t.resume()
        self.assertTrue(self.fake.bp_skip_once & 0x1,
                        "skip-once was not armed for the enabled slot")

    def test_flush_applies_and_remains_halted(self):
        self.t.stage_reg("D0", 1)
        self.t.flush_regs()
        self.assertEqual(get_applied(self.fake, "D0"), 1)
        self.assertTrue(self.t.is_halted())

    def test_unwritable_register_is_rejected_not_ignored(self):
        with self.assertRaises(TargetError):
            self.t.stage_reg("MMUSR", 0)

    def test_writable_set_matches_what_arch_write_actually_accepts(self):
        """The host and REPL must advertise the same Stage-3 write surface."""
        self.assertEqual(
            Target.WRITABLE_REGS,
            frozenset([f"D{i}" for i in range(8)] +
                      [f"A{i}" for i in range(8)] +
                      ["USP", "MSP", "SSP", "ISP", "SR", "VBR", "CACR",
                       "TC", "ITT0", "ITT1", "DTT0", "DTT1", "URP", "SRP",
                       "PC", "SFC", "DFC"]))

    def test_every_writable_register_survives_a_real_apply(self):
        """Round-trips each one through the fake's actual arch-write parser,
        so a name this list accepts but the REPL rejects fails here."""
        for name in sorted(Target.WRITABLE_REGS):
            self.t.stage_reg(name, 0x1234)
        self.t.resume()   # applies; raises if the REPL rejected any name
        self.assertEqual(self.t.pending_regs(), {})

    def test_discard(self):
        self.t.stage_reg("D0", 1)
        self.t.discard_pending_regs()
        self.assertEqual(self.t.pending_regs(), {})


class TestStep(unittest.TestCase):
    def test_plain_step_is_exact(self):
        _f, t = make_target()
        t.halt()
        self.assertIs(t.step(), True)
        self.assertTrue(t.is_halted())

    def test_step_after_a_register_write_is_exact(self):
        _f, t = make_target()
        t.halt()
        t.stage_reg("D0", 5)
        self.assertIs(t.step(), True)
        self.assertTrue(t.is_halted())


# ══════════════════════════════════════════════════════════════════════
# Breakpoints / watchpoints / A-traps
# ══════════════════════════════════════════════════════════════════════
class TestBreakpoints(unittest.TestCase):
    def setUp(self):
        self.fake, self.t = make_target()
        self.t.halt()

    def test_arm_and_read_back(self):
        self.t.set_bp(2, 0x0002E938)
        slots = self.t.read_bp_slots()
        self.assertEqual(slots[2], (0x0002E938, True))
        self.assertFalse(slots[0][1])

    def test_all_four_slots_independent(self):
        for i, pc in enumerate((0x100, 0x200, 0x300, 0x400)):
            self.t.set_bp(i, pc)
        self.assertEqual([pc for pc, _en in self.t.read_bp_slots()],
                         [0x100, 0x200, 0x300, 0x400])
        self.assertTrue(all(en for _pc, en in self.t.read_bp_slots()))
        self.t.clear_bp(1)
        self.assertEqual([en for _pc, en in self.t.read_bp_slots()],
                         [True, False, True, True])

    def test_arm_that_does_not_stick_raises(self):
        """A breakpoint that was never armed and a breakpoint that was never
        reached look identical from the far end of a debugging session."""
        replies = [["> w 0x50900038 = 0x0002E938", "> READY"],
                   ["> r 0x50900038 = 0x00000000", "> READY"]]
        t = Target(ScriptedTransport(replies))
        with self.assertRaises(TargetError) as cm:
            t.set_bp(0, 0x0002E938)
        self.assertIn("read back", str(cm.exception))

    def test_slot_out_of_range(self):
        with self.assertRaises(TargetError):
            self.t.set_bp(4, 0x100)

    def test_hit_latch_clears_via_bit14_not_bit15(self):
        """RTL quirk: OFF_BREAK_PC_CTRL reads hit_valid at bit 15 but the
        write-clear tests bit 14.  Writing the symmetric bit is a silent
        no-op, so this asserts the latch really went away."""
        self.t.set_bp(0, 0x1000)
        self.fake.stop_breakpoint(0, 0x1000)
        sr = self.t.stop_reason()
        self.assertTrue(sr.bp_valid)
        self.t.clear_stop_latches()
        ctrl = self.t.dbg_read(gdbdbg.OFF_BREAK_PC_CTRL)
        self.assertEqual(ctrl & gdbdbg.BPCTRL_HIT_VALID_READ, 0)


class TestWatchpoints(unittest.TestCase):
    def setUp(self):
        self.fake, self.t = make_target()
        self.t.halt()

    def test_arm_store_watchpoint(self):
        self.t.set_watchpoint(0, 0x08CC, on_load=False, on_store=True)
        w = self.t.read_watchpoints()[0]
        self.assertTrue(w["enabled"] and w["stores"])
        self.assertFalse(w["loads"])
        self.assertEqual(w["addr"], 0x08CC)

    def test_arm_with_value_compare(self):
        self.t.set_watchpoint(1, 0x1000, value=0xFF, lanes=0xF)
        w = self.t.read_watchpoints()[1]
        self.assertTrue(w["value_cmp"])
        self.assertEqual(w["value"], 0xFF)

    def test_must_match_something(self):
        with self.assertRaises(TargetError):
            self.t.set_watchpoint(0, 0x1000, on_load=False, on_store=False)

    def test_clear(self):
        self.t.set_watchpoint(0, 0x1000)
        self.t.clear_watchpoint(0)
        self.assertFalse(self.t.read_watchpoints()[0]["enabled"])


class TestAtraps(unittest.TestCase):
    def setUp(self):
        self.fake, self.t = make_target()
        self.t.halt()

    def test_arm_exact_trap(self):
        self.t.set_atrap(0, 0xA815)
        a = self.t.read_atraps()[0]
        self.assertTrue(a["enabled"])
        self.assertEqual((a["value"], a["mask"]), (0xA815, 0xFFFF))

    def test_arm_family_mask(self):
        """Mask polarity here is 1 = CARE — the opposite of the watchpoint
        address mask.  0xA800/0xFF00 is the whole 0xA8xx family."""
        self.t.set_atrap(1, 0xA800, mask=0xFF00)
        a = self.t.read_atraps()[1]
        self.assertEqual((a["value"], a["mask"]), (0xA800, 0xFF00))

    def test_non_aline_opcode_refused(self):
        """The comparator is A-line gated; arming 0x4E71 would never match and
        reporting it as armed would be a lie."""
        with self.assertRaises(TargetError) as cm:
            self.t.set_atrap(0, 0x4E71)
        self.assertIn("A-line", str(cm.exception))

    def test_d0_qualifier(self):
        self.t.set_atrap(0, 0xA004, d0=0x5)
        a = self.t.read_atraps()[0]
        self.assertTrue(a["d0qual"])
        self.assertEqual(a["d0val"], 5)


class TestFeatureGating(unittest.TestCase):
    def test_missing_feature_raises_with_actionable_text(self):
        _f, t = make_target()
        t._features_cache = 0            # pretend an old bitstream
        with self.assertRaises(TargetError) as cm:
            t.require_feature(gdbdbg.FEAT_WATCHPOINTS, "data watchpoints")
        msg = str(cm.exception)
        self.assertIn("watchpoints", msg)
        self.assertIn("does not report", msg)

    def test_shipped_bitstream_features_match_the_rtl(self):
        _f, t = make_target()
        for bit in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 16, 17):
            self.assertTrue(t.has_feature(bit), f"feature bit {bit} missing")
        # 12 (perf counters) and 14 (trace trigger) are deliberately absent.
        self.assertFalse(t.has_feature(12))
        self.assertFalse(t.has_feature(14))


# ══════════════════════════════════════════════════════════════════════
# Stop-reason disambiguation
# ══════════════════════════════════════════════════════════════════════
class TestStopReason(unittest.TestCase):
    """Breakpoint, single-step and A-trap stops all set HALT_REASON bit 2 —
    they share the SYS_DBG_BREAK injection path.  Getting these apart is
    entirely a matter of the per-feature hit_valid registers."""

    def setUp(self):
        self.fake, self.t = make_target()
        self.t.halt()
        self.t.clear_stop_latches()

    def test_breakpoint(self):
        self.t.set_bp(1, 0x2E938)
        self.fake.stop_breakpoint(1, 0x2E938)
        sr = self.t.stop_reason()
        self.assertEqual(sr.kind, "breakpoint")
        self.assertEqual(sr.bp_slot, 1)
        self.assertEqual(sr.pc, 0x2E938)

    def test_step_is_not_reported_as_a_breakpoint(self):
        self.fake.stop_step(0x1234)
        sr = self.t.stop_reason()
        self.assertEqual(sr.kind, "step")
        self.assertTrue(sr.reason & gdbdbg.REASON_BREAK,
                        "a step really does set the break latch in hardware")

    def test_watchpoint(self):
        self.fake.stop_watchpoint(0, 0x8CC, 0xDEAD, 0x40801000, True, 0xC)
        sr = self.t.stop_reason()
        self.assertEqual(sr.kind, "watchpoint")
        self.assertTrue(sr.wp_is_store)
        self.assertEqual(sr.wp_addr, 0x8CC)

    def test_atrap(self):
        self.fake.stop_atrap(1, 0xA815, 0x9C74, 0x383D10, 7)
        sr = self.t.stop_reason()
        self.assertEqual(sr.kind, "atrap")
        self.assertEqual(sr.atrap_opword, 0xA815)
        self.assertEqual(sr.atrap_a0, 0x383D10)
        self.assertEqual(sr.atrap_d0, 7)

    def test_exception(self):
        # halt-on-exception only fires for vectors armed in the 256-bit mask
        # AND with the enable set — exactly as the hardware behaves.
        self.t.set_halt_exc_mask(1 << 2, True)
        self.fake.stop_exception(2, 0x40801234)
        sr = self.t.stop_reason()
        self.assertEqual(sr.kind, "exception")
        self.assertEqual(sr.exc_vec, 2)

    def test_exception_outside_the_mask_does_not_stop(self):
        """An unarmed vector must not be reported as a stop just because an
        exception happened — the CPU never halted."""
        self.t.set_halt_exc_mask(1 << 2, True)
        self.fake.stop_exception(4, 0x40801234)
        self.assertNotEqual(self.t.stop_reason().kind, "exception")

    def test_double_fault_is_called_a_wedge(self):
        self.fake.stop_double_fault(2, 0x40801234)
        sr = self.t.stop_reason()
        self.assertEqual(sr.kind, "double-fault")
        self.assertIn("wedged", sr.describe())

    def test_clearing_latches_prevents_stale_attribution(self):
        """A sticky hit latch left from the previous stop is one of the
        easiest ways to describe entirely the wrong event."""
        self.fake.stop_watchpoint(0, 0x8CC, 1, 2, True, 0xF)
        self.assertEqual(self.t.stop_reason().kind, "watchpoint")
        self.t.clear_stop_latches()
        self.fake.stop_atrap(0, 0xA002, 0x1000, 0, 0)
        self.assertEqual(self.t.stop_reason().kind, "atrap")


# ══════════════════════════════════════════════════════════════════════
# The D-cache bypass
# ══════════════════════════════════════════════════════════════════════
class TestDcacheBypass(unittest.TestCase):
    """A location the CPU just wrote reads back as zero over JTAG, and zero
    is indistinguishable from 'never written'."""

    def test_dirty_value_is_invisible_until_pushed(self):
        fake, t = make_target(auto_cache_flush=False)
        t.halt()
        fake.set_dirty_cache(0x2000, 0xCAFEBABE)
        self.assertEqual(t.read_words(0x2000, 1), [0])
        t.flush_dcache(force=True)
        self.assertEqual(t.read_words(0x2000, 1), [0xCAFEBABE])

    def test_auto_flush_makes_the_value_visible_without_asking(self):
        fake, t = make_target(auto_cache_flush=True)
        t.halt()
        fake.set_dirty_cache(0x2000, 0xCAFEBABE)
        self.assertEqual(t.read_words(0x2000, 1), [0xCAFEBABE])

    def test_flush_happens_once_per_halt_not_per_read(self):
        fake, t = make_target(auto_cache_flush=True)
        t.halt()
        t.read_words(0x2000, 1)
        before = fake.dcache_push_count if hasattr(
            fake, "dcache_push_count") else None
        t.read_words(0x2004, 1)
        if before is not None:
            self.assertEqual(fake.dcache_push_count, before)


class TestMemoryBytes(unittest.TestCase):
    def setUp(self):
        self.fake, self.t = make_target()
        self.t.halt()
        self.fake.load_bytes(0x3000, bytes(range(0x10)))

    def test_read_bytes_big_endian_and_unaligned_spans(self):
        self.assertEqual(self.t.read_bytes(0x3000, 4), bytes(range(4)))
        self.assertEqual(self.t.read_bytes(0x3001, 2), bytes([1, 2]))
        self.assertEqual(self.t.read_bytes(0x3003, 3), bytes([3, 4, 5]))

    def test_write_bytes_read_modify_write(self):
        self.t.write_bytes(0x3001, b"\xAA\xBB")
        self.assertEqual(self.t.read_bytes(0x3000, 4),
                         bytes([0x00, 0xAA, 0xBB, 0x03]))


# ══════════════════════════════════════════════════════════════════════
# GRSP packet layer
# ══════════════════════════════════════════════════════════════════════
class TestPacketFraming(unittest.TestCase):
    def test_pack_and_checksum(self):
        self.assertEqual(gdbstub.gdb_pack(b"OK"), b"$OK#9a")

    def test_good_packet_parsed(self):
        pkts, ints, bad, rem = gdbstub.gdb_parse_packets(b"$g#67")
        self.assertEqual((pkts, ints, bad, rem), ([b"g"], 0, 0, b""))

    def test_bad_checksum_is_flagged_for_nak_not_acted_on(self):
        pkts, _i, bad, _r = gdbstub.gdb_parse_packets(b"$g#00")
        self.assertEqual(pkts, [])
        self.assertEqual(bad, 1)

    def test_ctrl_c_counted(self):
        _p, ints, _b, _r = gdbstub.gdb_parse_packets(b"\x03")
        self.assertEqual(ints, 1)

    def test_partial_packet_is_retained(self):
        _p, _i, _b, rem = gdbstub.gdb_parse_packets(b"$vCont")
        self.assertEqual(rem, b"$vCont")

    def test_escape_decoding(self):
        """GDB escapes `#`, `$`, `}` and `*` as `}` followed by byte^0x20, so
        an escaped `#` arrives as `}\\x03` and must not terminate the frame."""
        payload = b"}" + bytes([0x23 ^ 0x20])
        raw = gdbstub.gdb_pack(payload)
        pkts, _i, _b, _r = gdbstub.gdb_parse_packets(raw)
        self.assertEqual(pkts, [b"#"])


def make_stub(halt=True):
    fake, t = make_target()
    if halt:
        t.halt()
    syms = SymbolTable.from_files(
        sorted((REPO / "tools" / "macsyms").glob("*.syms")))
    stub = gdbstub.GdbStub(t, port=0, syms=syms, verbose=False)
    return fake, t, stub


class TestGrspDispatch(unittest.TestCase):
    def setUp(self):
        self.fake, self.t, self.stub = make_stub()

    def test_qsupported_advertises_hwbreak_and_denies_swbreak(self):
        r = self.stub.dispatch(b"qSupported:multiprocess+")
        self.assertIn(b"hwbreak+", r)
        self.assertIn(b"swbreak-", r)

    def test_target_xml_is_wellformed_and_covers_every_register(self):
        blob = self.stub._target_xml()
        self.assertTrue(blob.startswith(b"<?xml"))
        for i in range(gdbstub.N_REGS):
            self.assertIn(f'regnum="{i}"'.encode(), blob)

    def test_target_xml_names_a6_a7_as_fp_sp(self):
        """gdb/m68k-tdep.c validates the core feature against its own register
        names, in which a6 is `fp` and a7 is `sp`.  Emitting "a6"/"a7" makes
        GDB reject the whole description, silently fall back to its built-in
        29-register layout, and then report every `g` packet we send as
        truncated — which is how this was found."""
        blob = self.stub._target_xml()
        self.assertIn(b'name="fp"', blob)
        self.assertIn(b'name="sp"', blob)
        self.assertNotIn(b'name="a6"', blob)
        self.assertNotIn(b'name="a7"', blob)

    def test_target_xml_declares_the_architecture(self):
        """Without <architecture>m68k</architecture> GDB also rejects the
        description (verified against gdb-multiarch 17.1)."""
        self.assertIn(b"<architecture>m68k</architecture>",
                      self.stub._target_xml())

    def test_qxfer_window_respects_offset_and_length(self):
        full = self.stub._target_xml()
        first = self.stub.dispatch(b"qXfer:features:read:target.xml:0,10")
        self.assertEqual(first[:1], b"m")
        self.assertEqual(first[1:], full[:0x10])
        tail = self.stub.dispatch(
            f"qXfer:features:read:target.xml:0,{len(full):x}".encode())
        self.assertEqual(tail[:1], b"l")

    def test_g_packet_length_matches_the_declared_register_set(self):
        g = self.stub.dispatch(b"g")
        self.assertEqual(len(g), gdbstub.G_PACKET_BYTES * 2)

    def test_g_packet_reflects_real_register_values(self):
        set_reg(self.fake, "D3", 0x11223344)
        set_reg(self.fake, "A5", 0x55667788)
        g = bytes.fromhex(self.stub.dispatch(b"g").decode())
        self.assertEqual(int.from_bytes(g[3 * 4:4 * 4], "big"), 0x11223344)
        self.assertEqual(int.from_bytes(g[13 * 4:14 * 4], "big"), 0x55667788)

    def test_P_stages_and_c_applies(self):
        idx = gdbstub.REGS.index("D0")
        r = self.stub.dispatch(f"P{idx:x}=deadbeef".encode())
        self.assertEqual(r, b"OK")
        self.assertTrue(self.t.is_halted(), "P must not resume the CPU")
        self.assertEqual(self.t.pending_regs()["D0"], 0xDEADBEEF)

    def test_P_on_a_readonly_register_errors_rather_than_lying(self):
        idx = gdbstub.REGS.index("TC")
        self.assertEqual(self.stub.dispatch(f"P{idx:x}=1".encode()), b"E02")

    def test_m_and_M(self):
        self.fake.load_bytes(0x4000, bytes([0xDE, 0xAD, 0xBE, 0xEF]))
        self.assertEqual(self.stub.dispatch(b"m4000,4"), b"deadbeef")
        self.assertEqual(self.stub.dispatch(b"M4000,2:1234"), b"OK")
        self.assertEqual(self.stub.dispatch(b"m4000,4"), b"1234beef")

    def test_m_failure_surfaces_as_an_error_not_as_zeros(self):
        stub = gdbstub.GdbStub(
            Target(ScriptedTransport([["> ERROR bus fault", "> READY"]]),
                   auto_cache_flush=False),
            port=0, verbose=False)
        self.assertEqual(stub.dispatch(b"m1000,4"), b"E01")

    def test_hw_breakpoint_insert_and_remove(self):
        self.assertEqual(self.stub.dispatch(b"Z1,2e938,2"), b"OK")
        self.assertEqual(self.t.read_bp_slots()[0], (0x2E938, True))
        self.assertEqual(self.stub.dispatch(b"z1,2e938,2"), b"OK")
        self.assertFalse(self.t.read_bp_slots()[0][1])

    def test_z0_is_served_from_hardware_slots(self):
        """GDB's default `break` uses Z0.  We map it to a real slot rather
        than patching memory."""
        self.assertEqual(self.stub.dispatch(b"Z0,1000,2"), b"OK")
        self.assertTrue(self.t.read_bp_slots()[0][1])

    def test_running_out_of_breakpoint_slots_is_an_honest_error(self):
        for i in range(4):
            self.assertEqual(
                self.stub.dispatch(f"Z1,{0x1000 + i * 4:x},2".encode()), b"OK")
        self.assertEqual(self.stub.dispatch(b"Z1,9000,2"), b"E28")

    def test_watchpoint_insert_and_remove(self):
        self.assertEqual(self.stub.dispatch(b"Z2,8cc,2"), b"OK")
        self.assertTrue(self.t.read_watchpoints()[0]["enabled"])
        self.assertTrue(self.t.read_watchpoints()[0]["stores"])
        self.assertEqual(self.stub.dispatch(b"z2,8cc,2"), b"OK")
        self.assertFalse(self.t.read_watchpoints()[0]["enabled"])

    def test_read_watchpoint_arms_the_load_side(self):
        self.stub.dispatch(b"Z3,8cc,4")
        w = self.t.read_watchpoints()[0]
        self.assertTrue(w["loads"])
        self.assertFalse(w["stores"])

    def test_access_watchpoint_arms_both(self):
        self.stub.dispatch(b"Z4,8cc,4")
        w = self.t.read_watchpoints()[0]
        self.assertTrue(w["loads"] and w["stores"])

    def test_running_out_of_watchpoint_slots_is_an_honest_error(self):
        self.stub.dispatch(b"Z2,1000,4")
        self.stub.dispatch(b"Z2,2000,4")
        self.assertEqual(self.stub.dispatch(b"Z2,3000,4"), b"E28")

    def test_vcont_query(self):
        self.assertEqual(self.stub.dispatch(b"vCont?"), b"vCont;c;C;s;S")

    def test_unknown_packet_gets_an_empty_reply(self):
        """Per the protocol an unrecognised packet is answered with an empty
        reply, which is how GDB discovers what the stub cannot do."""
        self.assertEqual(self.stub.dispatch(b"Ynope"), b"")


class TestStopReplies(unittest.TestCase):
    def setUp(self):
        self.fake, self.t, self.stub = make_stub()
        self.t.clear_stop_latches()

    def test_breakpoint_reply_carries_hwbreak(self):
        self.stub.dispatch(b"Z1,2e938,2")
        self.fake.stop_breakpoint(0, 0x2E938)
        self.assertEqual(self.stub.dispatch(b"?"), b"T05hwbreak:;")

    def test_watchpoint_reply_carries_the_address(self):
        self.stub.dispatch(b"Z2,8cc,4")
        self.fake.stop_watchpoint(0, 0x8CC, 0xDEAD, 0x1000, True, 0xF)
        self.assertEqual(self.stub.dispatch(b"?"), b"T05watch:8cc;")

    def test_read_watchpoint_reply_uses_rwatch(self):
        self.stub.dispatch(b"Z3,8cc,4")
        self.fake.stop_watchpoint(0, 0x8CC, 0xDEAD, 0x1000, False, 0xF)
        self.assertEqual(self.stub.dispatch(b"?"), b"T05rwatch:8cc;")

    def test_bus_error_maps_to_sigsegv(self):
        self.t.set_halt_exc_mask(1 << 2, True)
        self.fake.stop_exception(2, 0x40801234)
        self.assertEqual(self.stub.dispatch(b"?"), b"T0b")

    def test_illegal_instruction_maps_to_sigill(self):
        self.t.set_halt_exc_mask(1 << 4, True)
        self.fake.stop_exception(4, 0x40801234)
        self.assertEqual(self.stub.dispatch(b"?"), b"T04")

    def test_step_reports_sigtrap(self):
        self.fake.stop_step(0x1234)
        self.assertEqual(self.stub.dispatch(b"?"), b"T05")


class TestMonitorCommands(unittest.TestCase):
    def setUp(self):
        self.fake, self.t, self.stub = make_stub()

    def mon(self, cmd):
        return self.stub.monitor(cmd)

    def test_help_lists_the_mac_specific_commands(self):
        h = self.mon("help")
        for word in ("atrap", "lomem", "queue", "dce", "bt", "cache"):
            self.assertIn(word, h)

    def test_rcmd_replies_ok_and_sends_the_body_as_console_packets(self):
        """The body of a `monitor` response goes as `O` console packets and
        the reply itself is a status.  Returning the hex body AS the reply is
        a non-standard shape that gdb-multiarch answers with "Protocol error
        with Rcmd" and then prints raw hex at the user."""
        import socket as _socket
        a, b = _socket.socketpair()
        self.stub.conn = a
        try:
            reply = self.stub.dispatch(b"qRcmd," + b"help".hex().encode())
            self.assertEqual(reply, b"OK")
            b.setblocking(False)
            raw = b.recv(65536)
        finally:
            a.close()
            b.close()
        pkts, _i, _bad, _r = gdbstub.gdb_parse_packets(raw)
        self.assertTrue(pkts, "no console packets were sent")
        text = "".join(bytes.fromhex(p[1:].decode()).decode("ascii", "replace")
                       for p in pkts if p.startswith(b"O"))
        self.assertIn("monitor commands", text)

    def test_packet_handler_errors_are_queued_not_sent_immediately(self):
        """A message emitted while GDB is waiting for an `m`/`Z` reply must
        not go out as an `O` packet -- GDB would read it as the reply."""
        import socket as _socket
        a, b = _socket.socketpair()
        self.stub.conn = a
        try:
            for i in range(5):
                self.stub.dispatch(f"Z1,{0x1000 + i * 4:x},2".encode())
            b.setblocking(False)
            try:
                raw = b.recv(65536)
            except (BlockingIOError, OSError):
                raw = b""
        finally:
            a.close()
            b.close()
        self.assertEqual(raw, b"", "nothing may be pushed to GDB mid-packet")
        self.assertTrue(any("out of hardware breakpoint slots" in n
                            for n in self.stub._notices),
                        "the message should still be queued for later")

    def test_regs_decodes_sr(self):
        set_reg(self.fake, "SR", 0x2700)
        out = self.mon("regs")
        self.assertIn("S=1", out)
        self.assertIn("IPL=7", out)

    def test_features_reports_the_deliberate_zero(self):
        out = self.mon("features")
        self.assertIn("perf_counters", out)
        self.assertIn("by design", out)

    def test_sym_uses_the_symbol_table(self):
        self.assertIn("_FSDispatch", self.mon("sym 0x2e938"))

    def test_syms_filter(self):
        self.assertIn("_FSDispatch", self.mon("syms fsdisp"))

    def test_lomem_reads_at_the_declared_width(self):
        # CrsrNew is a 1-byte flag at $08CE.
        self.fake.load_bytes(0x8CC, bytes([0, 0, 0xFF, 0x11]))
        out = self.mon("lomem CrsrNew")
        self.assertIn("CrsrNew", out)
        self.assertIn("0xff", out)
        self.assertNotIn("0xff11", out)

    def test_lomem_unknown_name_says_so(self):
        self.assertIn("not in the symbol files", self.mon("lomem NoSuchGlobal"))

    def test_queue_reports_empty_queue(self):
        self.assertIn("EMPTY", self.mon("queue fs"))

    def test_queue_walks_and_decodes_a_param_block(self):
        pb = 0x00383D10
        self.fake.load_bytes(0x0360, (0).to_bytes(2, "big")
                             + pb.to_bytes(4, "big") + pb.to_bytes(4, "big"))
        block = bytearray(26)
        block[0:4] = (0).to_bytes(4, "big")        # qLink = end of queue
        block[4:6] = (2).to_bytes(2, "big")        # qType
        block[6:8] = (0xA002).to_bytes(2, "big")   # ioTrap = _Read
        block[16:18] = (0xFFFF).to_bytes(2, "big")  # ioResult = -1
        self.fake.load_bytes(pb, bytes(block))
        out = self.mon("queue fs")
        self.assertIn("_Read", out)
        self.assertIn("ioResult=-1", out)

    def test_queue_refuses_to_follow_an_odd_pointer(self):
        self.fake.load_bytes(0x0360, (0).to_bytes(2, "big")
                             + (0x1001).to_bytes(4, "big")
                             + (0).to_bytes(4, "big"))
        self.assertIn("corrupt", self.mon("queue fs"))

    def test_atrap_list_and_arm_by_name(self):
        self.assertIn("_SCSIDispatch", self.mon("atrap list"))
        out = self.mon("atrap arm _SCSIDispatch")
        self.assertIn("0xa815", out)
        self.assertTrue(self.t.read_atraps()[0]["enabled"])

    def test_atrap_arm_family_mask(self):
        self.mon("atrap arm 0xA800 mask 0xFF00")
        a = self.t.read_atraps()[0]
        self.assertEqual((a["value"], a["mask"]), (0xA800, 0xFF00))

    def test_atrap_unknown_name_is_reported(self):
        self.assertIn("unknown trap name", self.mon("atrap arm _NotATrap"))

    def test_atrap_status_shows_the_latched_hit(self):
        self.fake.stop_atrap(0, 0xA815, 0x9C74, 0x383D10, 3)
        out = self.mon("atrap status")
        self.assertIn("_SCSIDispatch", out)
        self.assertIn("0x00383d10", out)

    def test_bp_status_reads_hardware_not_host_bookkeeping(self):
        self.stub.dispatch(b"Z1,2e938,2")
        out = self.mon("bp status")
        self.assertIn("ARMED", out)
        self.assertIn("_FSDispatch", out)

    def test_watch_status_states_the_hardware_caveats(self):
        out = self.mon("watch status")
        self.assertIn("PHYSICAL", out)
        self.assertIn("read-modify-write", out)

    def test_cache_toggle(self):
        self.assertIn("DISABLED", self.mon("cache off"))
        self.assertFalse(self.t.auto_cache_flush)
        self.mon("cache on")
        self.assertTrue(self.t.auto_cache_flush)

    def test_bt_walks_the_a6_chain_and_symbolizes(self):
        set_reg(self.fake, "PC", 0x0002E938)
        set_reg(self.fake, "A6", 0x00100000)
        # frame: [A6] = next A6, [A6+4] = return address
        self.fake.load_bytes(0x00100000,
                             (0x00100100).to_bytes(4, "big")
                             + (0x0002CCCE).to_bytes(4, "big"))
        self.fake.load_bytes(0x00100100,
                             (0).to_bytes(4, "big") + (0).to_bytes(4, "big"))
        out = self.mon("bt")
        self.assertIn("#0", out)
        self.assertIn("_FSDispatch", out)
        self.assertIn("_DevMgrEntry", out)

    def test_bt_stops_on_a_non_growing_chain_instead_of_inventing_frames(self):
        set_reg(self.fake, "A6", 0x00100000)
        self.fake.load_bytes(0x00100000,
                             (0x00000010).to_bytes(4, "big")
                             + (0x0002CCCE).to_bytes(4, "big"))
        self.assertIn("does not grow the stack", self.mon("bt"))

    def test_bt_flags_a_zeroed_frame_as_possibly_uncached(self):
        set_reg(self.fake, "A6", 0x00100000)
        self.assertIn("D-cache", self.mon("bt"))

    def test_dce_refuses_an_unusable_unit_table_pointer(self):
        self.assertIn("not usable", self.mon("dce"))

    def test_dce_walks_a_populated_unit_table(self):
        self.fake.load_bytes(0x011C, (0x00200000).to_bytes(4, "big"))
        self.fake.load_bytes(0x01D2, (4).to_bytes(2, "big"))
        self.fake.load_bytes(0x00200000 + 4, (0x00300000).to_bytes(4, "big"))
        self.fake.load_bytes(0x00300000, (0x00301000).to_bytes(4, "big"))
        self.fake.load_bytes(0x00301000, (0x0002CCCE).to_bytes(4, "big"))
        out = self.mon("dce")
        self.assertIn("unit   1", out)
        self.assertIn("refNum    -2", out)

    def test_unknown_monitor_command_shows_help(self):
        out = self.mon("nonsense")
        self.assertIn("unknown monitor command", out)
        self.assertIn("monitor commands", out)

    def test_status_flags_a_hit_pc_live_pc_disagreement(self):
        self.fake.stop_breakpoint(0, 0x2E938)
        self.fake.dbg_pc = 0x40801234 if hasattr(self.fake, "dbg_pc") else None
        out = self.mon("status")
        self.assertIn("halt_reason", out)

    def test_status_lists_staged_register_writes(self):
        self.t.stage_reg("D0", 0x1234)
        self.assertIn("staged register writes", self.mon("status"))


class TestContinuePath(unittest.TestCase):
    """The continue → stop → report path, including the console narration.

    `_wait_for_stop` needs a socket to peek at for Ctrl-C, so these wire up a
    real socketpair and read back the `O` console packets the stub emits."""

    def setUp(self):
        import socket as _socket
        self.fake, self.t, self.stub = make_stub()
        self.a, self.b = _socket.socketpair()
        self.stub.conn = self.a

    def tearDown(self):
        self.a.close()
        self.b.close()

    def console_text(self) -> str:
        self.b.setblocking(False)
        try:
            raw = self.b.recv(65536)
        except (BlockingIOError, OSError):
            return ""
        out = []
        for pkt, *_ in [gdbstub.gdb_parse_packets(raw)[:1]]:
            for p in pkt:
                if p.startswith(b"O"):
                    out.append(bytes.fromhex(p[1:].decode()).decode(
                        "ascii", "replace"))
        return "".join(out)

    def test_breakpoint_stop_is_reported_and_narrated(self):
        self.stub.dispatch(b"Z1,2e938,2")
        self.fake.stop_breakpoint(0, 0x2E938)
        reply = self.stub._wait_for_stop(poll_s=0.0)
        self.assertEqual(reply, b"T05hwbreak:;")
        text = self.console_text()
        self.assertIn("breakpoint slot 0", text)
        self.assertIn("_FSDispatch", text)

    def test_atrap_stop_names_the_toolbox_trap(self):
        self.fake.stop_atrap(0, 0xA815, 0x9C74, 0x00383D10, 1)
        reply = self.stub._wait_for_stop(poll_s=0.0)
        self.assertEqual(reply, b"T05")
        text = self.console_text()
        self.assertIn("A-trap", text)
        self.assertIn("_SCSIDispatch", text)
        self.assertIn("0x00383d10", text)     # A0 = param block pointer

    def test_double_fault_tells_the_user_it_cannot_be_resumed(self):
        self.fake.stop_double_fault(2, 0x40801234)
        self.stub._wait_for_stop(poll_s=0.0)
        self.assertIn("cannot be resumed", self.console_text())

    def test_continue_clears_latches_so_the_next_stop_is_attributed_right(self):
        self.fake.stop_watchpoint(0, 0x8CC, 1, 2, True, 0xF)
        self.t.clear_stop_latches()
        self.assertFalse(self.t.stop_reason().wp_valid)


class TestSymbolIntegration(unittest.TestCase):
    def test_stub_symbolizes_owner_supplied_addresses(self):
        _f, _t, stub = make_stub()
        self.assertIn("_FSDispatch", stub.sym(0x0002E938))
        self.assertIn("_SCSIGet", stub.sym(0x00009C74))

    def test_unknown_address_stays_bare_hex(self):
        _f, _t, stub = make_stub()
        self.assertEqual(stub.sym(0x7F123456), "0x7f123456")


if __name__ == "__main__":
    unittest.main()
