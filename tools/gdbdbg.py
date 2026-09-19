#!/usr/bin/env python3
"""gdbdbg — the target layer under `gdbstub.py`.

Everything here sits between the GDB Remote Serial Protocol and the JTAG-AXI
REPL: transport, the debug register map, and the verified accessors that the
stub is allowed to use.

Why this is a separate module from `gdbstub.py`
-----------------------------------------------
Because it is the part that must never lie, and it is therefore the part that
needs the most test coverage.  Keeping it free of socket and packet handling
lets `tb/tests/host/test_gdbstub_host.py` drive it directly against the
offline fake in `tb/tests/host/fake_jtag_repl.py`.

The one rule
------------
**Never return a plausible-but-wrong value.**  This tooling has silently
fabricated wrong data in at least six distinct ways, and every one cost hours
of debugging in the wrong place.  So, concretely, throughout this module:

* Every 32-bit read echoes the address it actually read; we compare that echo
  against the address we asked for and raise if they differ.
* The REPL's `BADA0BAD` failed-read sentinel is treated as an error, never as
  data.
* A `> ERROR ...` line anywhere in a command's output raises.
* Registers that the RTL only defines while the core is halted are read only
  while the core is halted — verified each time, not assumed.
* Where a value cannot be obtained, the caller gets an exception.  There is no
  `except: return 0` in this file, and there should never be one: a zero is
  indistinguishable from "this memory was never written", which is exactly the
  confusion the D-cache bypass already creates.
"""

from __future__ import annotations

import re
import time
from pathlib import Path

DBG_BASE = 0x50900000

# ── debug_ctrl register map ─────────────────────────────────────────────
# Offsets transcribed from rtl/core/debug/debug_ctrl.v.  Names match the
# RTL's own localparams so the two can be diffed by eye.
OFF_VERSION            = 0x000
OFF_BUILD_ID           = 0x004
OFF_CONTROL            = 0x008
OFF_STATUS             = 0x00C
OFF_PC                 = 0x010
OFF_LAST_PC            = 0x014
OFF_EXC_VEC            = 0x024
OFF_EXC_PC             = 0x028
OFF_HALT_AFTER_LO      = 0x030
OFF_HALT_AFTER_HI      = 0x034
OFF_BREAK_PC0          = 0x038
OFF_HALT_CTL           = 0x03C
OFF_HALT_REASON        = 0x040
OFF_HALT_HIT_PC        = 0x044
OFF_HALT_HIT_INST_LO   = 0x048
OFF_HALT_HIT_INST_HI   = 0x04C
OFF_EXC_FAULT_ADDR     = 0x054
OFF_HALT_EXC_MASK_0    = 0x060
OFF_BP_SKIP_ONCE       = 0x080
OFF_BREAK_PC1          = 0x084
OFF_BREAK_PC2          = 0x088
OFF_BREAK_PC3          = 0x08C
OFF_BREAK_PC_CTRL      = 0x090
OFF_DBL_FAULT_PC       = 0x094
OFF_DBL_FAULT_VEC      = 0x098
OFF_FEATURES           = 0x0A0
OFF_DBG_RESET_CTL      = 0x0A4
OFF_CAP_TRACE          = 0x0A8
OFF_WP0_ADDR           = 0x0B0
OFF_WP0_AMASK          = 0x0B4
OFF_WP0_VALUE          = 0x0B8
OFF_WP0_CTRL           = 0x0BC
OFF_WP1_ADDR           = 0x0C0
OFF_WP1_AMASK          = 0x0C4
OFF_WP1_VALUE          = 0x0C8
OFF_WP1_CTRL           = 0x0CC
OFF_WP_HIT             = 0x0D0
OFF_WP_HIT_ADDR        = 0x0D4
OFF_WP_HIT_DATA        = 0x0D8
OFF_WP_HIT_PC          = 0x0DC
OFF_AT0_CTRL           = 0x0E0
OFF_AT0_MATCH          = 0x0E4
OFF_AT0_D0VAL          = 0x0E8
OFF_AT1_CTRL           = 0x0EC
OFF_AT1_MATCH          = 0x0F0
OFF_AT1_D0VAL          = 0x0F4
OFF_AT_SKIP_ONCE       = 0x0F8
OFF_AT_HIT             = 0x0FC
OFF_AT_HIT_PC          = 0x100
OFF_AT_HIT_A0          = 0x104
OFF_AT_HIT_D0          = 0x108
# cpu040 A7-ODD halt lane (2026-09-09): CTL bit0 enable, [31:16] threshold
OFF_A7ODD_CTL          = 0x10C
OFF_A7ODD_PC0          = 0x110
OFF_A7ODD_PC1          = 0x114
OFF_A7ODD_PC2          = 0x118
OFF_A7ODD_VALUE        = 0x11C
OFF_A7ODD_COUNT        = 0x120
OFF_PCRANGE_CTL        = 0x124
OFF_PCRANGE_LO         = 0x128
OFF_PCRANGE_HI         = 0x12C
OFF_PCRANGE_PC0        = 0x130
OFF_PCRANGE_PC1        = 0x134
OFF_PCRANGE_PC2        = 0x138
OFF_PCRANGE_COUNT      = 0x13C
OFF_CYCLE_LO           = 0x1000
OFF_INST_LO            = 0x1008
OFF_INST_HI            = 0x100C
OFF_FLUSH_COUNT        = 0x1014
OFF_EXC_COUNT          = 0x1018
OFF_ARCH_D0            = 0x2000
OFF_ARCH_A0            = 0x2020
OFF_ARCH_USP           = 0x2040
OFF_ARCH_SSP           = 0x2044
OFF_ARCH_ISP           = 0x2048
OFF_ARCH_SR            = 0x204C
OFF_ARCH_VBR           = 0x2050
OFF_ARCH_CACR          = 0x2054
OFF_ARCH_TC            = 0x2058
OFF_ARCH_ITT0          = 0x205C
OFF_ARCH_ITT1          = 0x2060
OFF_ARCH_DTT0          = 0x2064
OFF_ARCH_DTT1          = 0x2068
OFF_ARCH_URP           = 0x206C
OFF_ARCH_SRP           = 0x2070
OFF_ARCH_PC            = 0x2074
OFF_ARCH_APPLY         = 0x2078
OFF_ARCH_STATUS        = 0x207C
OFF_ARCH_SFC           = 0x2080
OFF_ARCH_DFC           = 0x2084
OFF_LIVE_VBR           = 0x2100
OFF_LIVE_SR            = 0x2104
OFF_LIVE_A7            = 0x2108
OFF_LIVE_USP           = 0x210C
OFF_LIVE_D0            = 0x2110
OFF_LIVE_A0            = 0x2130
OFF_LIVE_MMU_TC        = 0x2160
OFF_LIVE_MMU_DTT0      = 0x2164
OFF_LIVE_MMU_DTT1      = 0x2168
OFF_LIVE_MMU_ITT0      = 0x216C
OFF_LIVE_MMU_ITT1      = 0x2170
OFF_LIVE_MMU_SRP       = 0x2174
OFF_LIVE_MMU_URP       = 0x2178
OFF_LIVE_SSP           = 0x217C
OFF_LIVE_ISP           = 0x2180
OFF_LIVE_CACR          = 0x2184
OFF_LIVE_SFC           = 0x2188
OFF_LIVE_DFC           = 0x218C
OFF_LIVE_PC            = 0x2190
OFF_LIVE_MMUSR         = 0x2194

BREAK_PC_OFFS = (OFF_BREAK_PC0, OFF_BREAK_PC1, OFF_BREAK_PC2, OFF_BREAK_PC3)
WP_OFFS = (
    (OFF_WP0_ADDR, OFF_WP0_AMASK, OFF_WP0_VALUE, OFF_WP0_CTRL),
    (OFF_WP1_ADDR, OFF_WP1_AMASK, OFF_WP1_VALUE, OFF_WP1_CTRL),
)
AT_OFFS = (
    (OFF_AT0_CTRL, OFF_AT0_MATCH, OFF_AT0_D0VAL),
    (OFF_AT1_CTRL, OFF_AT1_MATCH, OFF_AT1_D0VAL),
)

# OFF_CONTROL bits
CTRL_HALT_REQ      = 1 << 0
CTRL_STEP_PULSE    = 1 << 1
CTRL_INIT_DONE_OVR = 1 << 3
CTRL_COLD_RST_HOLD = 1 << 4
CTRL_COLD_RST_PULSE = 1 << 5

# OFF_STATUS bits
STAT_HALTED       = 1 << 0
STAT_EXC_PENDING  = 1 << 1
STAT_INIT_DONE    = 1 << 2
STAT_AUTO_LATCHED = 1 << 4

# OFF_HALT_CTL bits
HALT_AFTER_EN   = 1 << 0
HALT_BREAK_EN0  = 1 << 1
HALT_CLEAR      = 1 << 2      # write-1 pulse: clears every auto-halt latch
HALT_AUTO_LATCH = 1 << 3
HALT_AFTER_LATCH = 1 << 4
HALT_BREAK_LATCH = 1 << 5
HALT_EXC_EN     = 1 << 6
HALT_EXC_LATCH  = 1 << 7

# OFF_HALT_REASON bits.  NOTE: this is a bitmask of sticky latches, not a
# mutually-exclusive code, and breakpoint / single-step / A-trap stops ALL
# set bit 2 because they ride the same SYS_DBG_BREAK injection path.  The
# only way to tell them apart is the per-feature hit_valid registers.
REASON_MANUAL      = 1 << 0
REASON_HALT_AFTER  = 1 << 1
REASON_BREAK       = 1 << 2
REASON_HALTED      = 1 << 3
REASON_EXC         = 1 << 6
REASON_DBL_FAULT   = 1 << 8
REASON_WATCHPOINT  = 1 << 11
REASON_ATRAP       = 1 << 12

# OFF_BREAK_PC_CTRL.  The read layout puts hit_valid at bit 15 but the
# write-clear path tests bit 14 (debug_ctrl.v:1939-1944).  That asymmetry is
# real RTL, not a typo here — writing bit 15 to clear is a silent no-op.
BPCTRL_HIT_VALID_READ  = 1 << 15
BPCTRL_HIT_VALID_CLEAR = 1 << 14

WP_HIT_VALID    = 1 << 0
WP_HIT_SLOT     = 1 << 1
WP_HIT_IS_STORE = 1 << 2

AT_HIT_VALID   = 1 << 0
AT_HIT_SLOT    = 1 << 1
AT_HIT_BUSY    = 1 << 2

# WPn_CTRL bits
WPC_ENABLE  = 1 << 0
WPC_LOADS   = 1 << 1
WPC_STORES  = 1 << 2
WPC_VALUE   = 1 << 3

# ATn_CTRL bits
ATC_ENABLE = 1 << 0
ATC_D0QUAL = 1 << 1

FEATURE_BITS = {
    0:  "dbg_reset_domain",
    1:  "axi_ready_gated",
    2:  "cfg_wipe",
    3:  "cpu_reset_count",
    4:  "pc_trace",
    5:  "exc_ring",
    6:  "break_pc_multi",
    7:  "halt_exc_mask",
    8:  "fault_snap",
    9:  "rts_snap",
    10: "live_arch",
    11: "dcache_probe",
    12: "perf_counters",
    13: "watchpoints",
    14: "trace_trigger",
    15: "atrap_bp",
    16: "atrap_regcap",
    17: "atrap_d0qual",
    18: "mon_sense",
    19: "arch_apply_stays_halted",
    20: "arch_dirty_apply",
    21: "cache_maint_only",
    22: "macro_retire_count",
    23: "stop_status_v2",
    24: "branch_ring",
}
FEAT_WATCHPOINTS = 13
FEAT_ATRAP_BP = 15
FEAT_ATRAP_REGCAP = 16
FEAT_ATRAP_D0QUAL = 17
FEAT_BREAK_PC_MULTI = 6

BAD_READ_SENTINEL = "BADA0BAD"


class TargetError(RuntimeError):
    """Anything that means "I could not obtain a trustworthy value".

    Always carries enough context to act on: which command, which address,
    what came back."""


# ── Transport ───────────────────────────────────────────────────────────

class ReplTransport:
    """Interface: `execute(line) -> list[str]` returning the REPL's output
    lines for one command, with the trailing `> READY` handshake consumed."""

    def execute(self, line: str, wait_s: float = 0.05) -> list[str]:
        raise NotImplementedError

    def close(self):
        pass


class FifoRepl(ReplTransport):
    """Drives a live `jtag_repl.tcl` session over its FIFO / tail-file pair.

    See docs/debugging_the_mac_on_fpga.md for how to start the REPL.  The
    contract we rely on (tools/jtag_repl.tcl:247-253) is that every command
    terminates with a `> READY` line, so we poll for that rather than
    sleeping a fixed interval."""

    def __init__(self, fifo_in: Path, fifo_out: Path, timeout_s: float = 30.0):
        self.fifo_in = Path(fifo_in)
        self.fifo_out = Path(fifo_out)
        self.timeout_s = timeout_s
        if not self.fifo_in.exists():
            raise TargetError(
                f"REPL input FIFO {self.fifo_in} does not exist — start "
                f"tools/jtag_repl.tcl first (see "
                f"docs/debugging_the_mac_on_fpga.md)")
        # Consume anything already buffered so our first command does not
        # parse a previous session's output as its own reply.
        self._mark = self.fifo_out.stat().st_size if self.fifo_out.exists() else 0

    def execute(self, line: str, wait_s: float = 0.05) -> list[str]:
        with open(self.fifo_in, "w") as f:
            f.write(line + "\n")
            f.flush()
        deadline = time.time() + max(self.timeout_s, wait_s * 40)
        raw = b""
        while time.time() < deadline:
            if self.fifo_out.exists():
                with open(self.fifo_out, "rb") as f:
                    f.seek(self._mark)
                    raw = f.read()
                if b"> READY" in raw:
                    break
            time.sleep(0.005)
        else:
            raise TargetError(
                f"REPL command {line!r} produced no '> READY' handshake "
                f"within {self.timeout_s}s — is jtag_repl.tcl still alive? "
                f"(partial output: {raw[-200:]!r})")
        self._mark += len(raw)
        text = raw.replace(b"\x00", b"").decode("ascii", errors="replace")
        return [ln.rstrip() for ln in text.splitlines() if ln.strip()]


class FakeReplTransport(ReplTransport):
    """Offline transport backed by tb/tests/host/fake_jtag_repl.py.

    Lets the whole GDB stack be exercised — including from real GDB — with no
    FPGA and no Vivado.  Everything it reports is simulated; it proves host
    logic, never hardware behaviour."""

    def __init__(self, fake):
        self.fake = fake

    def execute(self, line: str, wait_s: float = 0.05) -> list[str]:
        return [ln.rstrip() for ln in self.fake.execute(line) if ln.strip()]


# ── The verified target ─────────────────────────────────────────────────

_RE_READ  = re.compile(r"^>\s*r\s+0x([0-9A-Fa-f]{1,8})\s*=\s*0x(\S+)\s*$")
_RE_WRITE = re.compile(r"^>\s*w\s+0x([0-9A-Fa-f]{1,8})\s*=\s*0x([0-9A-Fa-f]{1,8})\s*$")
_RE_MEM   = re.compile(r"^>\s*mem\s+0x([0-9A-Fa-f]{1,8})\s*=\s*0x(\S+)\s*$")
_RE_ARCH  = re.compile(r"^>\s*([A-Z][A-Z0-9]*)\s*=\s*0x([0-9A-Fa-f]+)(.*)$")
_RE_ERROR = re.compile(r"^>\s*ERROR\b(.*)$")


class Target:
    """Verified access to the CPU debug surface.

    Every method either returns a value the hardware actually reported, or
    raises TargetError.  Nothing in between."""

    #: Registers whose backing snap chain is documented as valid only while
    #: the core is halted (debug_ctrl.v:383-387).
    HALT_ONLY_OFFSETS = frozenset(
        [OFF_LIVE_D0 + i * 4 for i in range(8)] +
        [OFF_LIVE_A0 + i * 4 for i in range(8)]
    )

    def __init__(self, transport: ReplTransport, auto_cache_flush: bool = True):
        self.t = transport
        self.auto_cache_flush = auto_cache_flush
        #: Bumped on every resume; used to flush the D-cache at most once per
        #: halt so repeated memory reads do not pay for it every time.
        self._halt_epoch = 0
        self._cache_flushed_epoch = -1
        self._features_cache: int | None = None
        #: Register writes are buffered so a multi-register packet is applied
        #: atomically at one effective-halt point.
        self._pending_regs: dict[str, int] = {}
        self.warnings: list[str] = []

    # -- low level ------------------------------------------------------
    @staticmethod
    def _check_error(lines, what: str):
        for ln in lines:
            m = _RE_ERROR.match(ln)
            if m:
                raise TargetError(f"{what}: REPL reported ERROR:{m.group(1)}")

    @staticmethod
    def _parse_value(text: str, what: str) -> int:
        if BAD_READ_SENTINEL in text.upper():
            raise TargetError(
                f"{what}: AXI read failed — REPL returned the {BAD_READ_SENTINEL} "
                f"sentinel.  This is a real bus failure, not a value of zero.")
        try:
            return int(text, 16)
        except ValueError:
            raise TargetError(f"{what}: unparseable value {text!r}")

    def raw(self, line: str, wait_s: float = 0.05) -> list[str]:
        """Run an arbitrary REPL command.  Used by `monitor` pass-through."""
        lines = self.t.execute(line, wait_s=wait_s)
        return lines

    def read32(self, addr: int, what: str = "") -> int:
        """Read one 32-bit word, verifying the REPL echoed the address we
        asked for.  The echo check is what catches a mis-parsed operand
        (`r 40800000` once read 0x026E8F80 because the token went through
        `expr`) and any future variant of it."""
        if addr & 3:
            raise TargetError(
                f"read32(0x{addr:08x}) is not longword-aligned; the JTAG-AXI "
                f"master would silently align down and return a neighbouring "
                f"word")
        what = what or f"read32 0x{addr:08x}"
        lines = self.t.execute(f"r 0x{addr:08X}")
        self._check_error(lines, what)
        for ln in lines:
            m = _RE_READ.match(ln)
            if not m:
                continue
            echoed = int(m.group(1), 16)
            if echoed != addr:
                raise TargetError(
                    f"{what}: REPL echoed address 0x{echoed:08x} but we asked "
                    f"for 0x{addr:08x} — operand parsing or aliasing bug; "
                    f"refusing to use the value")
            return self._parse_value(m.group(2), what) & 0xFFFFFFFF
        raise TargetError(f"{what}: no '> r ... = ...' line in {lines!r}")

    def write32(self, addr: int, value: int, what: str = ""):
        if addr & 3:
            raise TargetError(f"write32(0x{addr:08x}) is not longword-aligned")
        value &= 0xFFFFFFFF
        what = what or f"write32 0x{addr:08x}"
        lines = self.t.execute(f"w 0x{addr:08X} 0x{value:08X}")
        self._check_error(lines, what)
        for ln in lines:
            m = _RE_WRITE.match(ln)
            if not m:
                continue
            echoed = int(m.group(1), 16)
            echoed_v = int(m.group(2), 16)
            if echoed != addr or echoed_v != value:
                raise TargetError(
                    f"{what}: REPL echoed w 0x{echoed:08x}=0x{echoed_v:08x}, "
                    f"expected 0x{addr:08x}=0x{value:08x}")
            return
        raise TargetError(f"{what}: no '> w ... = ...' line in {lines!r}")

    def write32_verified(self, addr: int, value: int, mask: int = 0xFFFFFFFF,
                         what: str = ""):
        """Write then read back and compare under `mask`.

        Used for every breakpoint / watchpoint / A-trap arm.  An arm that did
        not land is the difference between "the CPU never reached my
        breakpoint" and "my breakpoint was never armed", and those two lead to
        completely different (and one of them entirely wasted) investigations."""
        what = what or f"arm 0x{addr:08x}"
        self.write32(addr, value, what)
        back = self.read32(addr, what + " readback")
        if (back & mask) != (value & mask):
            raise TargetError(
                f"{what}: wrote 0x{value:08x} but read back 0x{back:08x} "
                f"(mask 0x{mask:08x}) — the debug register did not accept the "
                f"write; NOT reporting this as armed")
        return back

    def dbg_read(self, off: int, what: str = "") -> int:
        if off in self.HALT_ONLY_OFFSETS and not self.is_halted():
            raise TargetError(
                f"refusing to read debug offset 0x{off:03x} while the CPU is "
                f"running: it is served by the snap chain, which the RTL "
                f"documents as valid only while halted (debug_ctrl.v:383-387). "
                f"A value read now would be plausible and meaningless.")
        return self.read32(DBG_BASE + off, what or f"dbg[0x{off:03x}]")

    def dbg_write(self, off: int, value: int, what: str = ""):
        self.write32(DBG_BASE + off, value, what or f"dbg[0x{off:03x}]")

    # -- capability discovery -------------------------------------------
    def features(self, refresh: bool = False) -> int:
        if self._features_cache is None or refresh:
            self._features_cache = self.dbg_read(OFF_FEATURES, "OFF_FEATURES")
        return self._features_cache

    def has_feature(self, bit: int) -> bool:
        return bool(self.features() & (1 << bit))

    def require_feature(self, bit: int, what: str):
        if not self.has_feature(bit):
            name = FEATURE_BITS.get(bit, f"bit{bit}")
            raise TargetError(
                f"{what} needs debug feature '{name}' (OFF_FEATURES bit {bit}), "
                f"which this bitstream does not report. Features=0x"
                f"{self.features():08x}.  Rebuild/reflash the FPGA, or use a "
                f"different mechanism — do not assume it works.")

    def version(self) -> int:
        return self.dbg_read(OFF_VERSION, "OFF_VERSION")

    def build_id(self) -> int:
        return self.dbg_read(OFF_BUILD_ID, "OFF_BUILD_ID")

    # -- halt / run ------------------------------------------------------
    def status(self) -> int:
        return self.dbg_read(OFF_STATUS, "OFF_STATUS")

    def is_halted(self) -> bool:
        return bool(self.read32(DBG_BASE + OFF_STATUS, "OFF_STATUS") & STAT_HALTED)

    def halt(self, timeout_s: float = 5.0):
        """Assert the manual halt request and wait for it to actually take.

        Returns only once OFF_STATUS.halted is set; otherwise raises.  "I asked
        it to halt" is not the same as "it halted"."""
        self.dbg_write(OFF_CONTROL, CTRL_HALT_REQ, "halt request")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.is_halted():
                self._halt_epoch += 1
                return
            time.sleep(0.02)
        raise TargetError(
            f"CPU did not halt within {timeout_s}s of setting OFF_CONTROL bit0. "
            f"halt_reason=0x{self.dbg_read(OFF_HALT_REASON):08x} "
            f"status=0x{self.status():08x}")

    def resume(self):
        """Continue, using the REPL's validated `continue` sequence.

        We deliberately do NOT hand-roll this.  Resuming correctly means
        arming BP skip-once, pulsing OFF_HALT_CTL bit2 to drop every auto-halt
        latch (because `dbg_halt_req = ctrl_halt_req | auto_halt_latched_r`,
        debug_ctrl.v:889 — clearing OFF_CONTROL alone leaves the CPU halted),
        and only then releasing OFF_CONTROL.  `continue` in jtag_repl.tcl
        encodes that sequence and is the version validated on hardware.

        Breakpoint skip-once is armed before applying staged registers so this
        remains compatible with older debug hardware as well as Stage 3."""
        self._arm_bp_skip_once()
        self.flush_regs(resume_ok=True)
        lines = self.t.execute("continue", wait_s=0.2)
        self._check_error(lines, "continue")
        self._halt_epoch += 1

    def _arm_bp_skip_once(self):
        """Arm skip-once for every enabled BP slot, if we are stopped on one.

        Harmless to do twice (the REPL's `continue` may do it again); the bit
        auto-clears on the decode pass that consumes it, so later visits to
        the same PC still trip the breakpoint."""
        ctl = self.dbg_read(OFF_HALT_CTL, "OFF_HALT_CTL")
        if not (ctl & HALT_BREAK_LATCH):
            return
        enables = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL") & 0xF
        if enables:
            self.dbg_write(OFF_BP_SKIP_ONCE, enables, "arm bp skip-once")

    def step(self, timeout_s: float = 5.0) -> bool:
        """Single-step one macro-instruction.

        Returns True after the exact macro step completes.

        The exact path is injection-based: OFF_CONTROL bit 1 arms a
        decode-side break at the next macro, so the stop is commit-precise
        with no skid, and it works across A-line / TRAP / F-line and bus
        errors — stepping a trapping instruction lands on the handler's first
        instruction.

        Stage-3 apply remains effectively halted, so staged register writes are
        committed first and the ordinary exact one-macro step follows from that
        same clean stop-the-world point."""
        self.flush_regs()
        lines = self.t.execute("step", wait_s=0.2)
        self._check_error(lines, "step")
        self._await_halt(timeout_s, "single-step")
        return True

    def halt_at_next_macro(self):
        """Ask a *running* CPU to stop at the next decoded macro boundary.

        Writing OFF_CONTROL bit 1 while the core is running sets the
        decode-side step arm without having a halt to release, so the next
        first-of-macro µop is replaced with SYS_DBG_BREAK and commit stops
        there.  That gives a clean instruction boundary — it says nothing
        about how far the CPU travelled before the write landed."""
        self.dbg_write(OFF_CONTROL, CTRL_HALT_REQ | CTRL_STEP_PULSE,
                       "break at next macro")

    def _await_halt(self, timeout_s: float, what: str, extra: str = ""):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.is_halted():
                self._halt_epoch += 1
                return
            time.sleep(0.02)
        raise TargetError(
            f"{what} did not come back to a halt within {timeout_s}s"
            + (f" ({extra})" if extra else "")
            + f". halt_reason=0x{self.dbg_read(OFF_HALT_REASON):08x} "
              f"status=0x{self.status():08x}")

    # -- stop reason -----------------------------------------------------
    def stop_reason(self) -> "StopReason":
        """Read the complete stop description in one pass.

        Breakpoint, single-step and A-trap stops all set HALT_REASON bit 2
        (they share the SYS_DBG_BREAK injection path), so the bit alone is
        ambiguous.  Disambiguation is by the per-feature hit_valid registers,
        exactly as debug_ctrl.v computes them."""
        sr = StopReason()
        sr.status = self.status()
        sr.halted = bool(sr.status & STAT_HALTED)
        sr.reason = self.dbg_read(OFF_HALT_REASON, "OFF_HALT_REASON")
        sr.halt_hit_pc = self.dbg_read(OFF_HALT_HIT_PC, "OFF_HALT_HIT_PC")
        sr.live_pc = self.dbg_read(OFF_PC, "OFF_PC")

        if sr.reason & REASON_WATCHPOINT and self.has_feature(FEAT_WATCHPOINTS):
            hit = self.dbg_read(OFF_WP_HIT, "OFF_WP_HIT")
            if hit & WP_HIT_VALID:
                sr.wp_valid = True
                sr.wp_slot = 1 if hit & WP_HIT_SLOT else 0
                sr.wp_is_store = bool(hit & WP_HIT_IS_STORE)
                sr.wp_wstrb = (hit >> 12) & 0xF
                sr.wp_addr = self.dbg_read(OFF_WP_HIT_ADDR, "WP_HIT_ADDR")
                sr.wp_data = self.dbg_read(OFF_WP_HIT_DATA, "WP_HIT_DATA")
                sr.wp_pc = self.dbg_read(OFF_WP_HIT_PC, "WP_HIT_PC")

        if self.has_feature(FEAT_ATRAP_BP):
            hit = self.dbg_read(OFF_AT_HIT, "OFF_AT_HIT")
            if hit & AT_HIT_BUSY and not hit & AT_HIT_VALID:
                # The A0/D0 capture FSM is mid-flight.  Reporting now would
                # hand back an A0/D0 pair from a previous trap.
                sr.atrap_capture_busy = True
            if hit & AT_HIT_VALID:
                sr.atrap_valid = True
                sr.atrap_slot = 1 if hit & AT_HIT_SLOT else 0
                sr.atrap_opword = (hit >> 16) & 0xFFFF
                sr.atrap_pc = self.dbg_read(OFF_AT_HIT_PC, "AT_HIT_PC")
                if self.has_feature(FEAT_ATRAP_REGCAP):
                    sr.atrap_a0 = self.dbg_read(OFF_AT_HIT_A0, "AT_HIT_A0")
                    sr.atrap_d0 = self.dbg_read(OFF_AT_HIT_D0, "AT_HIT_D0")

        bpctrl = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL")
        sr.bp_enables = bpctrl & 0xF
        if bpctrl & BPCTRL_HIT_VALID_READ:
            sr.bp_valid = True
            sr.bp_slot = (bpctrl >> 8) & 0x3

        if sr.reason & REASON_EXC:
            sr.exc_valid = True
            sr.exc_vec = self.dbg_read(OFF_EXC_VEC, "OFF_EXC_VEC") & 0xFF
            sr.exc_pc = self.dbg_read(OFF_EXC_PC, "OFF_EXC_PC")
            sr.exc_fault_addr = self.dbg_read(OFF_EXC_FAULT_ADDR, "EXC_FAULT_ADDR")

        if sr.reason & REASON_DBL_FAULT:
            dfv = self.dbg_read(OFF_DBL_FAULT_VEC, "OFF_DBL_FAULT_VEC")
            sr.dbl_fault = True
            sr.dbl_fault_vec = dfv & 0xFF
            sr.dbl_fault_pc = self.dbg_read(OFF_DBL_FAULT_PC, "DBL_FAULT_PC")

        sr.halt_after = bool(sr.reason & REASON_HALT_AFTER)
        return sr

    def clear_stop_latches(self):
        """Clear the sticky hit reports so the NEXT stop cannot be confused
        with this one.  A stale hit_valid is one of the easiest ways to make a
        debugger describe the wrong event."""
        bpctrl = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL")
        if bpctrl & BPCTRL_HIT_VALID_READ:
            # RTL quirk: the clear is on bit 14 even though the status reads
            # back on bit 15 (debug_ctrl.v:1939-1944).  Writing bit 15 here
            # would be a silent no-op.
            self.dbg_write(OFF_BREAK_PC_CTRL,
                           (bpctrl & 0xF) | BPCTRL_HIT_VALID_CLEAR,
                           "clear break-pc hit latch")
        if self.has_feature(FEAT_WATCHPOINTS):
            if self.dbg_read(OFF_WP_HIT, "OFF_WP_HIT") & WP_HIT_VALID:
                self.dbg_write(OFF_WP_HIT, WP_HIT_VALID, "clear wp hit latch")
        if self.has_feature(FEAT_ATRAP_BP):
            if self.dbg_read(OFF_AT_HIT, "OFF_AT_HIT") & AT_HIT_VALID:
                self.dbg_write(OFF_AT_HIT, AT_HIT_VALID, "clear atrap hit latch")

    # -- architectural registers ----------------------------------------
    MMU_REGS = frozenset(
        ["TC", "ITT0", "ITT1", "DTT0", "DTT1", "SRP", "URP", "MMUSR"])

    def read_regs(self, include_mmu: bool = True) -> dict[str, int]:
        """Read the architectural register file.

        Requires the CPU to be halted, and says so rather than quietly using
        `live-arch force`: the snap chain multiplexes through the committed
        RAT, so on a running CPU it returns a coherent-looking snapshot of
        nothing in particular."""
        if not self.is_halted():
            raise TargetError(
                "refusing to read architectural registers while the CPU is "
                "running — the snap chain is only valid while halted, and a "
                "'live-arch force' read here has previously produced a "
                "confident wrong answer.  Halt first.")
        lines = self.t.execute("regs")
        self._check_error(lines, "regs")
        regs: dict[str, int] = {}
        for ln in lines:
            m = _RE_ARCH.match(ln)
            if not m:
                continue
            name, val, trailer = m.group(1), m.group(2), m.group(3)
            if BAD_READ_SENTINEL in val.upper():
                raise TargetError(f"live-arch: {name} read failed "
                                  f"({BAD_READ_SENTINEL})")
            regs[name] = int(val, 16) & 0xFFFFFFFF
            if "<<<" in trailer:
                # The REPL flags an SR with reserved bits set.  Surface it
                # rather than swallowing it — it once indicated a real ISA bug
                # (unmasked SR writes in commit.v), not a transport glitch.
                self.warnings.append(f"live-arch {name}:{trailer.strip()}")
        missing = [n for n in
                   [f"D{i}" for i in range(8)] + [f"A{i}" for i in range(8)] +
                   ["SR", "VBR", "PC"] if n not in regs]
        if missing:
            raise TargetError(
                f"live-arch did not report {missing} — got {sorted(regs)} "
                f"from {lines!r}")
        # `regs` is one complete, version-gated coherent dump. Do not issue a
        # second set of direct CSR reads; aside from needless JTAG traffic, that
        # used to let the two sources drift into subtly different views.
        if not include_mmu:
            for name in self.MMU_REGS:
                regs.pop(name, None)
        # Merge anything the host has staged but not yet applied, so a
        # read-after-write is self-consistent for the user.
        regs.update(self._pending_regs)
        return regs

    WRITABLE_REGS = frozenset(
        [f"D{i}" for i in range(8)] + [f"A{i}" for i in range(8)] +
        ["USP", "MSP", "SSP", "ISP", "SR", "VBR", "CACR", "TC",
         "ITT0", "ITT1", "DTT0", "DTT1", "URP", "SRP", "PC", "SFC", "DFC"])

    def stage_reg(self, name: str, value: int):
        """Buffer a register write.

        Writes accumulate until the next resume or step so a multi-register GDB
        packet becomes one atomic halted apply transaction."""
        if name not in self.WRITABLE_REGS:
            raise TargetError(
                f"register {name} is not writable through the arch-shadow "
                f"apply path (writable: {sorted(self.WRITABLE_REGS)})")
        self._pending_regs[name] = value & 0xFFFFFFFF

    def pending_regs(self) -> dict[str, int]:
        return dict(self._pending_regs)

    def flush_regs(self, resume_ok: bool = False):
        """Atomically apply staged writes while remaining effectively halted.

        ``resume_ok`` is retained for API compatibility with older callers; Stage 3
        hardware no longer needs or uses that unsafe permission.
        """
        if not self._pending_regs:
            return
        for name, value in sorted(self._pending_regs.items()):
            lines = self.t.execute(f"arch-write {name} 0x{value:08X}")
            self._check_error(lines, f"arch-write {name}")
        lines = self.t.execute("arch-apply", wait_s=0.2)
        self._check_error(lines, "arch-apply")
        self._pending_regs.clear()

    def discard_pending_regs(self):
        self._pending_regs.clear()

    # -- memory ----------------------------------------------------------
    def flush_dcache(self, force: bool = False):
        """Push dirty D-cache lines out to RAM so JTAG reads see them.

        JTAG-AXI reads bypass the CPU's write-back D-cache, so a location the
        CPU wrote recently reads back as whatever RAM last held — very often
        zero, which is indistinguishable from "never written".  This is the
        single most misleading behaviour in the whole stack, so we push once
        per halt before serving memory."""
        if not self.auto_cache_flush and not force:
            return
        if self._cache_flushed_epoch == self._halt_epoch and not force:
            return
        if not self.is_halted():
            raise TargetError(
                "cannot flush the D-cache while the CPU is running (the RTL "
                "only accepts cache ops while halted)")
        lines = self.t.execute("dcache-op push", wait_s=0.2)
        self._check_error(lines, "dcache-op push")
        self._cache_flushed_epoch = self._halt_epoch

    def invalidate_caches(self, icache: bool = True):
        """Drop cached copies so the CPU sees memory we just wrote.

        Order matters: the caller must have pushed dirty lines first, or an
        invalidate would discard the CPU's own un-written-back stores."""
        lines = self.t.execute("dcache-op inv", wait_s=0.2)
        self._check_error(lines, "dcache-op inv")
        if icache:
            lines = self.t.execute("icache-op inv", wait_s=0.2)
            self._check_error(lines, "icache-op inv")

    def read_words(self, addr: int, n_words: int) -> list[int]:
        """Read `n_words` consecutive longwords.

        Layered defence against the historical burst bug (a fixed-address
        burst returning the same latched word for every address).  The REPL's
        `rd_burst` has its own guard; we add an independent one here, because
        the echoed addresses in `dump-mem` output are computed host-side and
        therefore prove nothing about what the hardware actually fetched."""
        if addr & 3:
            raise TargetError(f"read_words: 0x{addr:08x} not longword-aligned")
        if n_words <= 0:
            return []
        if self.auto_cache_flush and self.is_halted():
            self.flush_dcache()
        if n_words == 1:
            return [self.read32(addr, f"mem 0x{addr:08x}")]

        # The word count goes through the REPL's `parse_num`, which is
        # HEX-BY-DEFAULT — `dump-mem <addr> 16` reads 0x16 = 22 words, not 16.
        # Send it 0x-prefixed so there is nothing to misread.
        lines = self.t.execute(f"dump-mem 0x{addr:08X} 0x{n_words:X}",
                               wait_s=0.1)
        self._check_error(lines, f"dump-mem 0x{addr:08x}")
        words: list[int] = []
        for ln in lines:
            m = _RE_MEM.match(ln)
            if not m:
                continue
            want = addr + len(words) * 4
            got = int(m.group(1), 16)
            if got != want:
                raise TargetError(
                    f"dump-mem 0x{addr:08x}: entry {len(words)} reports "
                    f"address 0x{got:08x}, expected 0x{want:08x}")
            words.append(self._parse_value(m.group(2), f"mem 0x{got:08x}"))
        if len(words) != n_words:
            raise TargetError(
                f"dump-mem 0x{addr:08x} {n_words}: got {len(words)} words, "
                f"expected {n_words}")
        if n_words > 1 and len(set(words)) == 1:
            # Every word identical.  Legitimate for a zeroed page, but it is
            # also the exact signature of the fixed-burst bug, so confirm the
            # second word independently before believing it.
            check = self.read32(addr + 4, "burst cross-check")
            if check != words[1]:
                raise TargetError(
                    f"dump-mem 0x{addr:08x} returned {n_words} identical words "
                    f"(0x{words[0]:08x}) but an independent single read of "
                    f"0x{addr + 4:08x} returned 0x{check:08x}.  The burst path "
                    f"is fabricating data — refusing to return it.")
        return words

    def read_bytes(self, addr: int, n: int) -> bytes:
        if n <= 0:
            return b""
        start = addr & ~3
        end = (addr + n + 3) & ~3
        words = self.read_words(start, (end - start) // 4)
        raw = b"".join(w.to_bytes(4, "big") for w in words)
        return raw[addr - start: addr - start + n]

    def write_bytes(self, addr: int, data: bytes, sync_caches: bool = True):
        """Read-modify-write a byte range, then re-synchronise the caches.

        Push before invalidate, always: pushing first means the invalidate
        cannot throw away a dirty line the CPU has not written back yet."""
        if not data:
            return
        if sync_caches and self.is_halted():
            self.flush_dcache()
        n = len(data)
        start = addr & ~3
        end = (addr + n + 3) & ~3
        n_words = (end - start) // 4
        if addr == start and n == n_words * 4:
            existing = bytearray(n_words * 4)   # fully overwritten anyway
        else:
            existing = bytearray(b"".join(
                w.to_bytes(4, "big") for w in self.read_words(start, n_words)))
        existing[addr - start: addr - start + n] = data
        for i in range(n_words):
            word = int.from_bytes(bytes(existing[i * 4:(i + 1) * 4]), "big")
            self.write32(start + i * 4, word, f"mem 0x{start + i * 4:08x}")
        if sync_caches and self.is_halted():
            self.invalidate_caches()
            self._cache_flushed_epoch = self._halt_epoch

    # -- breakpoints ------------------------------------------------------
    N_BP_SLOTS = 4

    def read_bp_slots(self) -> list[tuple[int, bool]]:
        ctrl = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL")
        out = []
        for i, off in enumerate(BREAK_PC_OFFS):
            pc = self.dbg_read(off, f"OFF_BREAK_PC{i}")
            out.append((pc, bool(ctrl & (1 << i))))
        return out

    def set_bp(self, slot: int, pc: int):
        """Arm hardware breakpoint `slot` at `pc`, verifying it landed.

        We write the debug registers directly rather than using the REPL's
        `break-pc` command, because that command *releases the halt and sleeps
        200 ms* as part of its job — appropriate for interactive bring-up,
        catastrophic for GDB, which inserts breakpoints while stopped and
        expects the target to stay exactly where it is."""
        if not 0 <= slot < self.N_BP_SLOTS:
            raise TargetError(f"breakpoint slot {slot} out of range 0..3")
        self.write32_verified(DBG_BASE + BREAK_PC_OFFS[slot], pc,
                              what=f"break_pc slot {slot}")
        ctrl = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL")
        want = (ctrl & 0xF) | (1 << slot)
        self.dbg_write(OFF_BREAK_PC_CTRL, want, f"enable bp slot {slot}")
        back = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL readback")
        if (back & 0xF) != want:
            raise TargetError(
                f"breakpoint slot {slot} enable did not stick: wrote "
                f"enables=0x{want:x}, read back 0x{back & 0xF:x}")

    def clear_bp(self, slot: int):
        ctrl = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL")
        want = (ctrl & 0xF) & ~(1 << slot)
        self.dbg_write(OFF_BREAK_PC_CTRL, want, f"disable bp slot {slot}")
        back = self.dbg_read(OFF_BREAK_PC_CTRL, "OFF_BREAK_PC_CTRL readback")
        if (back & 0xF) != want:
            raise TargetError(
                f"breakpoint slot {slot} disable did not stick "
                f"(enables now 0x{back & 0xF:x})")

    # -- watchpoints ------------------------------------------------------
    N_WP_SLOTS = 2

    def set_watchpoint(self, slot: int, addr: int, amask: int = 0,
                       on_load: bool = True, on_store: bool = True,
                       value: int | None = None, lanes: int = 0xF):
        """Arm data watchpoint `slot`.

        `amask` polarity is **1 = don't care** (the opposite of the A-trap
        match mask, where 1 = care; both conventions are the RTL's, and both
        are easy to get backwards).  Address compare is longword-granular:
        address bits [1:0] are always masked out by the hardware.

        Watchpoints match **physical** addresses, after MMU translation."""
        self.require_feature(FEAT_WATCHPOINTS, "data watchpoints")
        if not 0 <= slot < self.N_WP_SLOTS:
            raise TargetError(f"watchpoint slot {slot} out of range 0..1")
        if not (on_load or on_store):
            raise TargetError("watchpoint must match loads, stores or both")
        a_off, m_off, v_off, c_off = WP_OFFS[slot]
        ctrl = WPC_ENABLE
        ctrl |= WPC_LOADS if on_load else 0
        ctrl |= WPC_STORES if on_store else 0
        if value is not None:
            ctrl |= WPC_VALUE | ((lanes & 0xFF) << 8)
        self.write32_verified(DBG_BASE + a_off, addr, what=f"wp{slot} addr")
        self.write32_verified(DBG_BASE + m_off, amask, what=f"wp{slot} amask")
        if value is not None:
            self.write32_verified(DBG_BASE + v_off, value, what=f"wp{slot} value")
        self.write32_verified(DBG_BASE + c_off, ctrl, mask=0xFFFF,
                              what=f"wp{slot} ctrl")

    def clear_watchpoint(self, slot: int):
        self.require_feature(FEAT_WATCHPOINTS, "data watchpoints")
        _a, _m, _v, c_off = WP_OFFS[slot]
        self.write32_verified(DBG_BASE + c_off, 0, mask=0xFFFF,
                              what=f"wp{slot} disable")

    def read_watchpoints(self) -> list[dict]:
        out = []
        for slot, (a, m, v, c) in enumerate(WP_OFFS):
            ctrl = self.dbg_read(c, f"WP{slot}_CTRL")
            out.append({
                "slot": slot,
                "enabled": bool(ctrl & WPC_ENABLE),
                "loads": bool(ctrl & WPC_LOADS),
                "stores": bool(ctrl & WPC_STORES),
                "value_cmp": bool(ctrl & WPC_VALUE),
                "lanes": (ctrl >> 8) & 0xFF,
                "addr": self.dbg_read(a, f"WP{slot}_ADDR"),
                "amask": self.dbg_read(m, f"WP{slot}_AMASK"),
                "value": self.dbg_read(v, f"WP{slot}_VALUE"),
            })
        return out

    # -- A-trap breakpoints ----------------------------------------------
    N_AT_SLOTS = 2

    def set_atrap(self, slot: int, value: int, mask: int = 0xFFFF,
                  d0: int | None = None):
        """Arm an A-trap (Mac OS toolbox) breakpoint.

        `mask` polarity is **1 = CARE** — the opposite of the watchpoint
        address mask.  `0xA815, mask 0xFFFF` is exactly `_SCSIDispatch`;
        `0xA800, mask 0xFF00` is the whole `0xA8xx` toolbox family.

        The hardware additionally requires opword[15:12] == 0xA, so an
        all-don't-care mask degenerates to "every toolbox call", never to
        "every instruction"."""
        self.require_feature(FEAT_ATRAP_BP, "A-trap breakpoints")
        if not 0 <= slot < self.N_AT_SLOTS:
            raise TargetError(f"A-trap slot {slot} out of range 0..1")
        if (value & 0xF000) != 0xA000:
            raise TargetError(
                f"0x{value:04x} is not an A-line opcode (must be 0xAxxx) — the "
                f"hardware comparator would never match it")
        c_off, m_off, d_off = AT_OFFS[slot]
        packed = ((mask & 0xFFFF) << 16) | (value & 0xFFFF)
        ctrl = ATC_ENABLE
        if d0 is not None:
            self.require_feature(FEAT_ATRAP_D0QUAL, "A-trap D0 qualifier")
            ctrl |= ATC_D0QUAL
            self.write32_verified(DBG_BASE + d_off, d0, what=f"at{slot} d0val")
        # {mask,value} share one CSR so the slot is never live on a
        # half-written pattern; write it before enabling.
        self.write32_verified(DBG_BASE + m_off, packed, what=f"at{slot} match")
        self.write32_verified(DBG_BASE + c_off, ctrl, mask=0x3,
                              what=f"at{slot} ctrl")

    def clear_atrap(self, slot: int):
        self.require_feature(FEAT_ATRAP_BP, "A-trap breakpoints")
        c_off, _m, _d = AT_OFFS[slot]
        self.write32_verified(DBG_BASE + c_off, 0, mask=0x3,
                              what=f"at{slot} disable")

    def read_atraps(self) -> list[dict]:
        out = []
        for slot, (c, m, d) in enumerate(AT_OFFS):
            ctrl = self.dbg_read(c, f"AT{slot}_CTRL")
            packed = self.dbg_read(m, f"AT{slot}_MATCH")
            out.append({
                "slot": slot,
                "enabled": bool(ctrl & ATC_ENABLE),
                "d0qual": bool(ctrl & ATC_D0QUAL),
                "value": packed & 0xFFFF,
                "mask": (packed >> 16) & 0xFFFF,
                "d0val": self.dbg_read(d, f"AT{slot}_D0VAL"),
            })
        return out

    # -- halt-on-exception ------------------------------------------------
    def set_halt_exc_mask(self, mask256: int, enable: bool):
        for lane in range(8):
            self.dbg_write(OFF_HALT_EXC_MASK_0 + lane * 4,
                           (mask256 >> (lane * 32)) & 0xFFFFFFFF,
                           f"halt-exc mask lane {lane}")
        ctl = self.dbg_read(OFF_HALT_CTL, "OFF_HALT_CTL")
        bits = (ctl & (HALT_AFTER_EN | HALT_BREAK_EN0))
        if enable:
            bits |= HALT_EXC_EN
        self.dbg_write(OFF_HALT_CTL, bits, "halt-exc enable")

    def get_halt_exc_mask(self) -> int:
        mask = 0
        for lane in range(8):
            mask |= self.dbg_read(OFF_HALT_EXC_MASK_0 + lane * 4,
                                  f"halt-exc lane {lane}") << (lane * 32)
        return mask


class StopReason:
    """Everything the hardware can tell us about why it stopped."""

    def __init__(self):
        self.status = 0
        self.halted = False
        self.reason = 0
        self.halt_hit_pc = 0
        self.live_pc = 0
        self.halt_after = False
        self.bp_valid = False
        self.bp_slot = 0
        self.bp_enables = 0
        self.wp_valid = False
        self.wp_slot = 0
        self.wp_is_store = False
        self.wp_addr = 0
        self.wp_data = 0
        self.wp_pc = 0
        self.wp_wstrb = 0
        self.atrap_valid = False
        self.atrap_capture_busy = False
        self.atrap_slot = 0
        self.atrap_opword = 0
        self.atrap_pc = 0
        self.atrap_a0 = 0
        self.atrap_d0 = 0
        self.exc_valid = False
        self.exc_vec = 0
        self.exc_pc = 0
        self.exc_fault_addr = 0
        self.dbl_fault = False
        self.dbl_fault_vec = 0
        self.dbl_fault_pc = 0

    @property
    def pc(self) -> int:
        """The PC GDB should show.

        For any event that came through the precise break path, HALT_HIT_PC is
        the instruction *about to execute* — which is what a debugger means by
        "where you are".  For a plain manual halt there is no hit capture, so
        the live PC is the only thing on offer."""
        if self.reason & (REASON_BREAK | REASON_EXC | REASON_HALT_AFTER |
                          REASON_WATCHPOINT | REASON_DBL_FAULT):
            return self.halt_hit_pc
        return self.live_pc

    @property
    def kind(self) -> str:
        if self.dbl_fault:
            return "double-fault"
        if self.wp_valid:
            return "watchpoint"
        if self.atrap_valid:
            return "atrap"
        if self.bp_valid:
            return "breakpoint"
        if self.exc_valid:
            return "exception"
        if self.halt_after:
            return "halt-after-N"
        if self.reason & REASON_BREAK:
            # Break path fired but no slot claims it: that is a completed
            # single-step (or an A-trap whose capture has not resolved yet).
            return "step"
        if self.halted:
            return "manual"
        return "running"

    def describe(self, sym=None) -> str:
        def a(x):
            return sym.format(x) if sym else f"0x{x:08x}"
        k = self.kind
        if k == "watchpoint":
            what = "store to" if self.wp_is_store else "load from"
            return (f"watchpoint {self.wp_slot}: {what} {a(self.wp_addr)} "
                    f"data=0x{self.wp_data:08x} wstrb=0x{self.wp_wstrb:x} "
                    f"at {a(self.wp_pc)}")
        if k == "atrap":
            return (f"A-trap slot {self.atrap_slot}: opword 0x{self.atrap_opword:04x} "
                    f"at {a(self.atrap_pc)} A0=0x{self.atrap_a0:08x} "
                    f"D0=0x{self.atrap_d0:08x}")
        if k == "breakpoint":
            return f"breakpoint slot {self.bp_slot} at {a(self.pc)}"
        if k == "exception":
            return (f"exception vector {self.exc_vec} (0x{self.exc_vec:02x}) at "
                    f"{a(self.exc_pc)} fault_addr=0x{self.exc_fault_addr:08x}")
        if k == "double-fault":
            return (f"DOUBLE FAULT vector {self.dbl_fault_vec} at "
                    f"{a(self.dbl_fault_pc)} — the CPU is wedged, not stopped")
        if k == "halt-after-N":
            return f"halt-after-N reached at {a(self.pc)}"
        if k == "step":
            return f"single-step complete at {a(self.pc)}"
        if k == "manual":
            return f"halted by host request at {a(self.pc)}"
        return "CPU is running"
