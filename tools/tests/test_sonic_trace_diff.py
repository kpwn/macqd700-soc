#!/usr/bin/env python3
"""Unit tests for tools/sonic_trace_diff.py.

All inputs are hand-built CSV, never the output of a real capture: a test
whose fixture came from the tool it tests proves nothing.
"""
from __future__ import annotations

import importlib.util
import io
import os
import re
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DIFF_PY = REPO_ROOT / "tools" / "sonic_trace_diff.py"
CAPTURE_LUA = REPO_ROOT / "tools" / "mame_sonic_capture.lua"


def _load_module():
    spec = importlib.util.spec_from_file_location("sonic_trace_diff", DIFF_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


std = _load_module()

HEADER = "# columns: ns,rw,reg,data,lanes,flags\n"

# Register indices, spelled out so the fixtures read like the driver does.
CR, DCR, RCR, TCR, IMR, ISR = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
RSC, CRCT = 0x2b, 0x2c
CRDA = 0x0e


def ev(rw, reg, data, lanes=3, flags="-", ns=0):
    return "%d,%s,%02x,%04x,%x,%s" % (ns, rw, reg, data, lanes, flags)


class TraceFile:
    """Builds a capture file on disk from a list of event lines."""

    def __init__(self, tmpdir, name, lines, header=HEADER):
        self.path = Path(tmpdir) / name
        self.path.write_text(header + "\n".join(lines) + "\n")

    def __fspath__(self):
        return str(self.path)


# A plausible driver init sequence: software reset, configure, arm the
# receiver, enable interrupts, then a poll of ISR.
INIT = [
    ev("W", CR, 0x0080),            # software reset
    ev("W", DCR, 0x0026),           # data configuration
    ev("W", CR, 0x0000),            # leave reset
    ev("W", RCR, 0x2000),           # accept broadcast
    ev("W", IMR, 0x0446),           # PRXEN|PTXEN|RDEEN|LCDEN
    ev("R", IMR, 0x0446),           # driver reads its own mask back
    ev("W", CR, 0x0208),            # LCAM | RXEN
]


def run_diff(*argv):
    """-> (exit_code, stdout)."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        code = std.main(list(argv))
    return code, buf.getvalue()


class LoadTests(unittest.TestCase):
    def test_columns_header_is_honoured_in_any_order(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "t.csv"
            p.write_text("# columns: rw,lanes,reg,data\nW,3,04,0446\nR,1,05,0002\n")
            events, meta = std.load(str(p))
        self.assertTrue(meta["has_lanes"])
        self.assertEqual(events[0], ("W", 0x04, 0x0446, 3))
        self.assertEqual(events[1], ("R", 0x05, 0x0002, 1))

    def test_positional_fallback_without_header(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "t.csv"
            p.write_text("0,W,04,0446,3\n1,R,04,0446,3\n")
            events, meta = std.load(str(p))
        self.assertTrue(meta["has_lanes"])
        self.assertEqual([e.reg for e in events], [0x04, 0x04])

    def test_hardware_style_columns_with_a_timestamp_tail(self):
        # The JTAG ring dumper emits idx first and a timestamp last; neither
        # must be mistaken for the lanes column.
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "t.csv"
            p.write_text("# columns: idx,rw,reg,data,lanes,tstamp\n"
                         "0,W,04,0446,3,12345\n")
            events, meta = std.load(str(p))
        self.assertEqual(events[0], ("W", 0x04, 0x0446, 3))
        self.assertTrue(meta["has_lanes"])


class IdenticalStreamTests(unittest.TestCase):
    def test_identical_streams_report_no_divergence(self):
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", INIT)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 0, out)
        self.assertIn("NO DIVERGENCE", out)
        self.assertNotIn("FIRST DIVERGENCE", out)

    def test_identical_streams_are_still_clean_under_strict(self):
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", INIT)
            code, out = run_diff(os.fspath(mame), os.fspath(hw), "--strict")
        self.assertEqual(code, 0, out)
        self.assertIn("NO DIVERGENCE", out)


class WriteDivergenceTests(unittest.TestCase):
    def test_differing_write_value_is_caught_at_the_right_index(self):
        # HW writes a different interrupt mask -- index 4 of the stream.
        hw_lines = list(INIT)
        hw_lines[4] = ev("W", IMR, 0x0400)
        hw_lines[5] = ev("R", IMR, 0x0400)
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 1, out)
        self.assertIn("FIRST DIVERGENCE at event 4", out)
        self.assertIn("HW  : W IMR  =0400", out)
        self.assertIn("MAME: W IMR  =0446", out)

    def test_a_write_value_difference_is_never_tolerated_by_a_read_rule(self):
        # 'tally' is on by default but covers READS only.  A write to CRCT
        # that differs must still be a divergence.
        base = INIT + [ev("W", CRCT, 0xffff)]
        hw_lines = INIT + [ev("W", CRCT, 0x0000)]
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", base)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 1, out)
        self.assertIn("FIRST DIVERGENCE at event 7", out)

    def test_register_index_difference_is_a_divergence(self):
        hw_lines = list(INIT)
        hw_lines[3] = ev("W", TCR, 0x2000)
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw), "--align", "first")
        self.assertEqual(code, 1, out)
        self.assertIn("register index", out)

    def test_half_register_write_is_reported_as_a_lane_divergence(self):
        # MAME writes the whole 16-bit IMR; HW only drives the low half.
        hw_lines = list(INIT)
        hw_lines[4] = ev("W", IMR, 0x0046, lanes=1)
        hw_lines[5] = ev("R", IMR, 0x0046, lanes=1)
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 1, out)
        self.assertIn("byte lanes", out)

    def test_ignore_lanes_compares_only_the_touched_bytes(self):
        hw_lines = list(INIT)
        hw_lines[4] = ev("W", IMR, 0x0046, lanes=1)   # low half matches MAME's 0x46
        hw_lines[5] = ev("R", IMR, 0x0046, lanes=1)
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw), "--ignore-lanes")
        self.assertEqual(code, 0, out)
        self.assertIn("NO DIVERGENCE", out)


class BenignReadTests(unittest.TestCase):
    def test_tally_counter_read_difference_is_tolerated_and_reported(self):
        mame_lines = INIT + [ev("R", RSC, 0x0007)]
        hw_lines = INIT + [ev("R", RSC, 0x0000)]
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 0, out)
        self.assertIn("NO DIVERGENCE", out)
        # Tolerated, but never silent.
        self.assertIn("tolerated differences", out)
        self.assertIn("R RSC", out)
        self.assertIn("tally", out)

    def test_the_same_difference_is_a_divergence_under_strict(self):
        mame_lines = INIT + [ev("R", RSC, 0x0007)]
        hw_lines = INIT + [ev("R", RSC, 0x0000)]
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw), "--strict")
        self.assertEqual(code, 1, out)
        self.assertIn("FIRST DIVERGENCE at event 7", out)

    def test_receive_pointer_difference_is_NOT_tolerated_by_default(self):
        # rx-pointers is deliberately off by default: the RX path is the
        # thing under investigation.
        mame_lines = INIT + [ev("R", CRDA, 0x1000)]
        hw_lines = INIT + [ev("R", CRDA, 0x2000)]
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
            self.assertEqual(code, 1, out)
            code2, out2 = run_diff(os.fspath(mame), os.fspath(hw),
                                   "--tolerate", "rx-pointers")
        self.assertEqual(code2, 0, out2)
        self.assertIn("rx-pointers", out2)

    def test_tx_status_rule_does_not_hide_a_failed_transmit(self):
        # tx-status tolerates the error bits but explicitly NOT TCR_PTX.
        mame_lines = INIT + [ev("R", TCR, 0x0001)]   # PTX: transmitted OK
        hw_lines = INIT + [ev("R", TCR, 0x0040)]     # EXC: excessive collisions
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw),
                                 "--tolerate", "tx-status")
        self.assertEqual(code, 1, out)
        self.assertIn("differing bits 0001", out)

    def test_ad_hoc_tolerate_reg_accepts_a_named_bit(self):
        mame_lines = INIT + [ev("R", ISR, 0x0400)]
        hw_lines = INIT + [ev("R", ISR, 0x0000)]
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw),
                                 "--tolerate-reg", "05:0400")
        self.assertEqual(code, 0, out)
        self.assertIn("cli:05:0400", out)


class PollRepetitionTests(unittest.TestCase):
    def test_differing_poll_counts_do_not_cause_a_false_positive(self):
        mame_lines = INIT + [ev("R", ISR, 0x0000)] * 40 + [ev("R", ISR, 0x0200)]
        hw_lines = INIT + [ev("R", ISR, 0x0000)] * 3 + [ev("R", ISR, 0x0200)]
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 0, out)
        self.assertIn("NO DIVERGENCE", out)

    def test_interleaved_two_register_poll_loops_also_fold(self):
        cycle = [ev("R", ISR, 0x0000), ev("R", CR, 0x0008)]
        mame_lines = INIT + cycle * 25 + [ev("R", ISR, 0x0200)]
        hw_lines = INIT + cycle * 2 + [ev("R", ISR, 0x0200)]
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 0, out)
        self.assertIn("NO DIVERGENCE", out)

    def test_poll_folding_never_swallows_the_value_that_changed(self):
        # HW's poll never observes the completion bit.  Folding must not
        # make that look the same as MAME's poll, which does.
        mame_lines = INIT + [ev("R", ISR, 0x0000)] * 5 + [ev("R", ISR, 0x0200)]
        hw_lines = INIT + [ev("R", ISR, 0x0000)] * 900
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 1, out)
        self.assertIn("STOPS at event", out)

    def test_poll_folding_can_be_disabled(self):
        mame_lines = INIT + [ev("R", ISR, 0x0000)] * 4
        hw_lines = INIT + [ev("R", ISR, 0x0000)] * 2
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, _ = run_diff(os.fspath(mame), os.fspath(hw))
            self.assertEqual(code, 0)
            code2, out2 = run_diff(os.fspath(mame), os.fspath(hw), "--poll-window", "0")
        self.assertEqual(code2, 1, out2)
        self.assertIn("STOPS at event", out2)


class NormalisationTests(unittest.TestCase):
    def test_open_bus_only_accesses_are_dropped(self):
        hw_lines = list(INIT)
        hw_lines.insert(2, ev("W", CR, 0x0000, lanes=0, flags="L"))
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 0, out)
        self.assertIn("open-bus-only accesses dropped", out)

    def test_collapse_polls_is_a_pure_function_on_events(self):
        e = std.Event
        seq = [e("R", 5, 0, 3)] * 6 + [e("R", 5, 0x200, 3)]
        self.assertEqual(std.collapse_polls(seq, 4),
                         [e("R", 5, 0, 3), e("R", 5, 0x200, 3)])

    def test_writes_are_barriers_for_poll_folding(self):
        e = std.Event
        seq = [e("R", 5, 0, 3), e("W", 5, 1, 3), e("R", 5, 0, 3)]
        self.assertEqual(std.collapse_polls(seq, 4), seq)


class AlignmentTests(unittest.TestCase):
    def test_hardware_window_starting_mid_stream_still_aligns(self):
        # The HW ring is last-N-wins: it holds only the tail.  Slide mode
        # must find where that tail sits in the MAME trace.
        prelude = [ev("R", ISR, 0x0000), ev("R", CR, 0x0014), ev("W", CR, 0x0004)]
        mame_lines = prelude + INIT
        hw_lines = INIT[1:]                      # no CR<-RST anchor in the window
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw), "--align", "slide")
        self.assertEqual(code, 0, out)
        self.assertIn("best start offset 4", out)

    def test_epoch_alignment_picks_the_matching_reset_epoch(self):
        other = [ev("W", CR, 0x0080), ev("W", DCR, 0x0000), ev("W", CR, 0x0000)]
        mame_lines = other + INIT
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", mame_lines)
            hw = TraceFile(td, "hw.csv", INIT)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 0, out)
        self.assertIn("auto -> epoch", out)
        self.assertIn("best-matching epoch #1", out)


class ReadbackAuditTests(unittest.TestCase):
    def test_imr_readback_of_zero_is_called_out_on_hardware_only(self):
        hw_lines = list(INIT)
        hw_lines[5] = ev("R", IMR, 0x0000)   # the reported symptom
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", hw_lines)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 1, out)
        self.assertIn("HW    IMR   wrote=0446", out)
        self.assertIn("FIRST MISMATCH: wrote 0446, read 0000", out)
        self.assertIn("*** IMR: HW reads back a value it never wrote", out)

    def test_audit_is_quiet_when_both_sides_read_back_what_they_wrote(self):
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", INIT)
            _, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertIn("no register misbehaves on HW but not on MAME", out)


class EmptyInputTests(unittest.TestCase):
    def test_empty_hardware_trace_is_not_reported_as_agreement(self):
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", INIT)
            hw = TraceFile(td, "hw.csv", [])
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 2, out)
        self.assertIn("HW trace is EMPTY", out)
        self.assertNotIn("NO DIVERGENCE", out)

    def test_empty_mame_trace_says_which_knob_to_turn(self):
        with tempfile.TemporaryDirectory() as td:
            mame = TraceFile(td, "mame.csv", [])
            hw = TraceFile(td, "hw.csv", INIT)
            code, out = run_diff(os.fspath(mame), os.fspath(hw))
        self.assertEqual(code, 2, out)
        self.assertIn("MAME_SONIC_TRACE_MIRRORS=all", out)


class RegisterModelTests(unittest.TestCase):
    """The register numbering has to agree with the RTL, or nothing else means
    anything.  Both sides derive the index as (byte offset >> 2)."""

    def test_register_names_match_the_dp83932c_layout(self):
        self.assertEqual(std.reg_name(0x00), "CR")
        self.assertEqual(std.reg_name(0x04), "IMR")
        self.assertEqual(std.reg_name(0x05), "ISR")
        self.assertEqual(std.reg_name(0x28), "SR")
        self.assertEqual(std.reg_name(0x3f), "DCR2")
        self.assertEqual(std.reg_name(0x40), "PROM0")

    def test_regmask_table_matches_the_rtl(self):
        rtl = (REPO_ROOT / "rtl" / "mac" / "q700_eth_sonic.v").read_text()
        found = dict(
            (int(a, 16), int(b, 16))
            for a, b in re.findall(r"6'h([0-9A-Fa-f]{2}):\s*sonic_reg_mask\s*=\s*16'h([0-9A-Fa-f]{4})", rtl))
        self.assertTrue(found, "could not parse sonic_reg_mask() out of the RTL")
        for reg, mask in found.items():
            self.assertEqual(std.REGMASK[reg], mask,
                             "regmask disagreement at reg %02x (%s): "
                             "differ table %04x, RTL %04x"
                             % (reg, std.reg_name(reg), std.REGMASK[reg], mask))

    def test_lane_mask_matches_the_sonic_wstrb_encoding(self):
        # peripheral_bus.v: sonic_wstrb[1] gates D15..D8 (byte +2),
        #                   sonic_wstrb[0] gates D7..D0  (byte +3).
        self.assertEqual(std.lane_mask(0), 0x0000)
        self.assertEqual(std.lane_mask(1), 0x00ff)
        self.assertEqual(std.lane_mask(2), 0xff00)
        self.assertEqual(std.lane_mask(3), 0xffff)


class CaptureScriptTests(unittest.TestCase):
    def test_lua_capture_emits_a_columns_header_the_differ_understands(self):
        text = CAPTURE_LUA.read_text()
        self.assertIn("# columns: ns,rw,reg,data,lanes,flags", text)
        # The differ's own parser must accept exactly that header.
        roles = std.parse_columns("ns,rw,reg,data,lanes,flags".split(","), 6, "x")
        self.assertEqual(roles, [None, "rw", "reg", "data", "lanes", "flags"])

    def test_lua_capture_derives_the_register_index_as_offset_shr_2(self):
        text = CAPTURE_LUA.read_text()
        self.assertIn("(win >> 2) & 0x3f", text)

    def test_lua_capture_keeps_taps_and_the_stop_subscription_alive(self):
        text = CAPTURE_LUA.read_text()
        self.assertIn("_G.sonic_taps", text)
        self.assertIn("_G.sonic_stop_sub", text)


if __name__ == "__main__":
    unittest.main()
