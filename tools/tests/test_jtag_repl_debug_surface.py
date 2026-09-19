import os
import subprocess
import textwrap
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
REPL = ROOT / "tools" / "jtag_repl.tcl"


def run_tcl(body: str) -> str:
    script = f"source {{{REPL}}}\n" + textwrap.dedent(body)
    env = dict(os.environ, JTAG_REPL_LIBRARY_ONLY="1")
    result = subprocess.run(
        ["tclsh"], input=script, text=True, capture_output=True, env=env, check=False
    )
    if result.returncode:
        raise AssertionError(f"tcl failed ({result.returncode}):\n{result.stdout}\n{result.stderr}")
    return result.stdout


class JtagReplDebugSurfaceTest(unittest.TestCase):
    def test_complete_architectural_name_map(self):
        out = run_tcl(
            """
            foreach item {
                {D0 0x2000} {D7 0x201c} {A0 0x2020} {A7 0x203c}
                {USP 0x2040} {MSP 0x2044} {SSP 0x2044} {ISP 0x2048}
                {SR 0x204c} {VBR 0x2050} {CACR 0x2054} {TC 0x2058}
                {ITT0 0x205c} {ITT1 0x2060} {DTT0 0x2064} {DTT1 0x2068}
                {URP 0x206c} {SRP 0x2070} {PC 0x2074} {SFC 0x2080} {DFC 0x2084}
            } {
                lassign $item name expected
                if {[arch_reg_offset $name] != $expected} { error "$name offset mismatch" }
            }
            puts OK
            """
        )
        self.assertIn("OK", out)

    def test_cache_status_rejection_and_error_are_fatal(self):
        out = run_tcl(
            """
            proc halt_status_line {} { return halted }
            set ::fake_status 0x10
            proc dbg_rd {off} { return [format %08X $::fake_status] }
            if {![catch {cache_op_poll 0x210 rejected-op} msg] ||
                [string first REJECTED $msg] < 0} { error "rejection not surfaced: $msg" }
            set ::fake_status 0x22
            if {![catch {cache_op_poll 0x210 failed-op} msg] ||
                [string first "WRITEBACK ERROR" $msg] < 0} { error "error not surfaced: $msg" }
            set ::fake_status 0x02
            cache_op_poll 0x210 good-op
            puts OK
            """
        )
        self.assertIn("good-op busy=0 done=1", out)
        self.assertIn("OK", out)

    def test_coherent_write_orders_push_write_and_invalidations(self):
        out = run_tcl(
            """
            set ::ops {}
            proc require_core040_debug_epoch {args} { lappend ::ops epoch }
            proc require_effective_halt {args} { lappend ::ops halt }
            proc dcache-op {kind} { lappend ::ops dcache-$kind }
            proc icache-op {kind} { lappend ::ops icache-$kind }
            proc wr {addr data} { lappend ::ops [format "write-%08X-%08X" $addr $data] }
            coherent-w 0x1234 0xa5a5
            if {$::ops ne {epoch halt dcache-push write-00001234-0000A5A5 dcache-inv icache-inv}} {
                error "bad coherent write order: $::ops"
            }
            puts OK
            """
        )
        self.assertIn("OK", out)

    def test_feature_bit_19_is_versioned(self):
        out = run_tcl(
            """
            set ::fake_version 0xDEB60008
            proc dbg_version {} { return $::fake_version }
            if {[lindex [dbg_feature_names] 19] ne "icache_probe"} {
                error "legacy bit 19 was misdecoded"
            }
            set ::fake_version $::CORE040_DEBUG_VERSION
            if {[lindex [dbg_feature_names] 19] ne "arch_apply_stays_halted"} {
                error "core040 bit 19 was misdecoded"
            }
            puts OK
            """
        )
        self.assertIn("OK", out)

    def test_core040_effective_halt_and_break_reason_use_v2_layout(self):
        out = run_tcl(
            """
            proc dbg_is_core040_epoch {} { return 1 }
            set ::fake_status 1
            set ::fake_reason 5
            set ::fake_hit 0x40801234
            proc dbg_rd {off} {
                if {$off == $::OFF_STATUS} { return [format %08X $::fake_status] }
                if {$off == $::OFF_HALT_REASON} { return [format %08X $::fake_reason] }
                if {$off == $::OFF_HALT_HIT_PC} { return [format %08X $::fake_hit] }
                return 00000000
            }
            if {![effective_halt]} { error "core040 STATUS.halted was ignored" }
            if {![break_pc_reached 0x40801234]} { error "primary reason code 5 was not decoded" }
            set ::fake_status 0
            if {[effective_halt]} { error "core040 effective halt incorrectly used legacy reason bit 3" }
            set ::fake_reason 6
            if {[break_pc_reached 0x40801234]} { error "exception reason misdecoded as breakpoint" }
            puts OK
            """
        )
        self.assertIn("OK", out)

    def test_legacy_effective_halt_layout_is_preserved(self):
        out = run_tcl(
            """
            proc dbg_is_core040_epoch {} { return 0 }
            set ::fake_reason 0x0c
            proc dbg_rd {off} {
                if {$off == $::OFF_HALT_REASON} { return [format %08X $::fake_reason] }
                if {$off == $::OFF_HALT_HIT_PC} { return 00001234 }
                return 00000000
            }
            if {![effective_halt]} { error "legacy HALT_REASON.effective was broken" }
            if {![break_pc_reached 0x1234]} { error "legacy breakpoint latch was broken" }
            puts OK
            """
        )
        self.assertIn("OK", out)


if __name__ == "__main__":
    unittest.main()
