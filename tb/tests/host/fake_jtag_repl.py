"""fake_jtag_repl.py — an OFFLINE FAKE of tools/jtag_repl.tcl.

THIS IS A FAKE.  It exists so a GDB stub (or any other host-side tool that
talks the jtag_repl.tcl line protocol) can be developed and exercised with
NO FPGA and NO Vivado attached.  It is a software model of:

  - the debug_ctrl.v register file (rtl/core/debug/debug_ctrl.v), reproduced
    offset-by-offset with its real bit layouts and its real, sometimes
    deliberately awkward, quirks (see "QUIRKS MODELED" below), and
  - a flat little-endian-indexed-by-byte / big-endian-content memory model
    with a scriptable "dirty D-cache" hazard.

Passing tests against this fake is NOT evidence of correctness on real
hardware.  It only proves a host tool's command sequences and register-bit
arithmetic match what tools/jtag_repl.tcl + debug_ctrl.v are DOCUMENTED and
READ to do at the time this file was written.  Real FPGA/Vivado validation
is still required before trusting anything this fake says about actual
silicon.  Treat a green run here the same way you'd treat a green unit test
for a device driver written against a datasheet: necessary, not sufficient.

Every command implemented here was written by reading the corresponding
arm of tools/jtag_repl.tcl's dispatcher (and, for register semantics, the
corresponding always-block in rtl/core/debug/debug_ctrl.v) and copying its
`puts` format strings verbatim — not by guessing a plausible-looking format.

QUIRKS MODELED (deliberately, because a naive gdbstub gets these wrong):

  - dbg_halt_req = ctrl_halt_req | auto_halt_latched.  Writing CONTROL
    bit0=0 does NOT resume a CPU that auto-halted (breakpoint / watchpoint
    / halt-on-exception / A-trap / double-fault / halt-after).  Resuming
    requires the HALT_CTL bit2 clear pulse.
  - Stage-3 ARCH_APPLY is an atomic dirty-register apply at an effective
    halt. Completion leaves the CPU halted; resume/step remain explicit.
  - BREAK_PC_CTRL: the READ layout puts hit_valid at bit 15, but the WRITE
    that is supposed to clear hit_valid tests bit 14, not bit 15.  Writing
    bit15=1 to "clear" it is a silent no-op.
  - HALT_AFTER_LO/HI store a DELTA, not an absolute target.  The target
    auto-rearms (current position + delta) every time the halt-after latch
    clears — it is not one-shot.
  - arch-write's <hex> operand is NOT run through the hex-by-default
    parse_num() that r/w/dump-mem use.  It goes through a bare Tcl `expr`,
    so an unprefixed token is DECIMAL ("arch-write D0 40" sets D0 to 40
    decimal, i.e. 0x28, not 0x40).  This asymmetry is real and is preserved
    here on purpose.
  - The LIVE_D0..D7/A0..A7 registers ride a shared snap chain that is only
    coherent while the core is halted.  Real hardware would return racy
    but plausible garbage when read on a running CPU; this FAKE instead
    returns the obviously-poisoned sentinel 0xDEADBEEF in that case, on
    purpose, so a stub bug (reading live registers without checking
    "effective halt" first) fails loudly in a test instead of silently
    shipping a plausible lie.
  - WP_HIT is sticky: the first watchpoint hit's detail (slot/addr/data/
    pc/wstrb/is_store) is latched and a second hit does NOT overwrite it,
    even though the generic halt/latch state (wp_latched, auto_halt_latched,
    HALT_HIT_PC) DOES update on every hit.  Forgetting to clear WP_HIT
    before continuing means the next distinct hit's detail is silently
    dropped.
  - Breakpoint, single-step, AND A-trap stops all set the SAME
    break_pc_latched bit (HALT_REASON bit 2).  Disambiguating "why did we
    halt" requires checking BREAK_PC_CTRL.hit_valid / AT_HIT.valid /
    WP_HIT.valid individually — none of which single-step sets.  Modeled
    exactly: stop_step() sets break_pc_latched but none of the hit_valid
    bits, on purpose.
  - Reading an unaligned address is a hard error (never silently aligned
    down) for r / w / dump-mem, mirroring require_aligned() in
    tools/jtag_repl.tcl.
  - A failed / unmodeled debug-register read returns the literal sentinel
    string "BADA0BAD", never a fabricated plausible value and never a
    silent 0.
  - dcache-op and icache-op SHARE one busy/done flip-flop pair in the real
    RTL (both OFF_DCACHE_OP and OFF_ICACHE_OP read the same
    dcache_op_busy_r/dcache_op_done_r registers).  Modeled faithfully:
    kicking one shows up in the other's status register too.
  - dcache-op is accepted only while the (fake) CPU is halted; a write
    while running is a silent reject (busy/done simply do not change).

SIMPLIFICATIONS (documented, not hidden):

  - No real pipeline, no real cycle-by-cycle drain latency: `halted` is
    computed synchronously as `ctrl_halt_req or auto_halt_latched` the
    instant either changes, instead of lagging by however many cycles the
    real core takes to drain in flight instructions.
  - `continue` and `step` do not execute real 68k code.  `step` advances a
    fixed, predictable (pc += 2, inst_count += 1) amount and immediately
    re-halts (that IS the contract of single-step).  `continue` resumes
    the CPU and, if resumed, also advances by that same fixed predictable
    amount so a test can observe forward progress; real forward progress
    for anything else (hitting a breakpoint, a watchpoint, an exception, a
    double fault, a halt-after target) is driven explicitly by a test via
    the stop_*() methods below — this fake does not simulate 68k
    execution, so it cannot know when a *real* CPU would trip one of those
    on its own.
  - LIVE_A7 is simplified to mirror arch_a[7]/live_a[7] directly rather
    than modeling the real SR.S/M-bit-dependent USP/SSP/ISP mux that picks
    which physical register the architectural A7 alias currently means.
  - PC trace ring, exception ring, dcache/wedge probe registers, ADB
    injection, SD-card registers, and everything else tools/jtag_repl.tcl
    implements beyond the ~40 registers and 10 commands this fake was
    asked to model are NOT implemented.  Any address inside the modeled
    debug_ctrl address window that isn't one of the offsets below reads
    back BADA0BAD, per the module contract ("never a fabricated 0").
  - atrap hit capture is NOT modeled with the same sticky "first hit wins"
    guard as WP_HIT — the RTL text available at the time this was written
    did not show that guard for the A-trap path, so this fake does not
    invent one.  If real hardware turns out to behave differently, treat
    this as a fake-vs-real divergence to fix, not a hidden assumption.

Usable in two ways:

  1. As a library: `from fake_jtag_repl import FakeRepl`, drive it with
     `FakeRepl().execute("r 0x50900000")` (returns a list of output lines,
     the last always "> READY", exactly like the real REPL's line
     protocol), and script hardware-like events with the stop_*() methods.

  2. As a standalone process implementing the same FIFO-based protocol
     described in CLAUDE.md for the real jtag_repl.tcl session:

         mkfifo /tmp/jtag_in
         python3 fake_jtag_repl.py --fifo-in /tmp/jtag_in --fifo-out /tmp/jtag_out &
         echo 'r 0x50900000' > /tmp/jtag_in
         tail -f /tmp/jtag_out

     so a human (or a gdbstub's own test harness) can drive the whole
     offline stack exactly the way they'd drive the real board.

  3. `python3 fake_jtag_repl.py --selftest` runs a small built-in
     self-check covering read/write, halt/resume, the quirks above, and
     the scriptable stop events, printing PASS/FAIL and exiting 0/1.
"""

import re
import sys

# ─────────────────────────────────────────────────────────────────────────
# Address map (relative offsets are exactly tools/jtag_repl.tcl's OFF_* /
# rtl/core/debug/debug_ctrl.v's localparams — same names, same values).
# ─────────────────────────────────────────────────────────────────────────

DBG_BASE = 0x50900000
# Generous window: real debug_ctrl.v's address space runs out past 0x13000
# (exception ring) plus 0x3000s (wedge probes).  Any offset inside this
# window that isn't one of the ones this fake implements returns BADA0BAD
# (never a fabricated 0); anything outside it is treated as plain memory.
DBG_WINDOW = 0x20000

BADA0BAD = "BADA0BAD"

OFF_VERSION = 0x000
OFF_BUILD_ID = 0x004
OFF_CONTROL = 0x008
OFF_STATUS = 0x00C
OFF_PC = 0x010
OFF_LAST_PC = 0x014
OFF_EXC_VEC = 0x024
OFF_EXC_PC = 0x028
OFF_EXC_FAULT_ADDR = 0x054        # faulting address for the last exception
OFF_HALT_AFTER_LO = 0x030
OFF_HALT_AFTER_HI = 0x034
OFF_BREAK_PC0 = 0x038
OFF_HALT_CTL = 0x03C
OFF_HALT_REASON = 0x040
OFF_HALT_HIT_PC = 0x044
OFF_HALT_HIT_INST_LO = 0x048
OFF_HALT_HIT_INST_HI = 0x04C
OFF_HALT_EXC_VEC = 0x050          # dead register: RW, comparator ignores it
OFF_HALT_EXC_MASK0 = 0x060        # .. + 0x1C, 8 lanes of 32 bits = 256 bits
OFF_PC_MISALIGNED_PC = 0x09C      # retired feature; always reads 0
OFF_BP_SKIP_ONCE = 0x080
OFF_BREAK_PC1 = 0x084
OFF_BREAK_PC2 = 0x088
OFF_BREAK_PC3 = 0x08C
OFF_BREAK_PC_CTRL = 0x090
OFF_DBL_FAULT_PC = 0x094
OFF_DBL_FAULT_VEC = 0x098
OFF_WP0_ADDR = 0x0B0
OFF_WP0_AMASK = 0x0B4
OFF_WP0_VALUE = 0x0B8
OFF_WP0_CTRL = 0x0BC
OFF_WP1_ADDR = 0x0C0
OFF_WP1_AMASK = 0x0C4
OFF_WP1_VALUE = 0x0C8
OFF_WP1_CTRL = 0x0CC
OFF_WP_HIT = 0x0D0
OFF_WP_HIT_ADDR = 0x0D4
OFF_WP_HIT_DATA = 0x0D8
OFF_WP_HIT_PC = 0x0DC
OFF_AT0_CTRL = 0x0E0
OFF_AT0_MATCH = 0x0E4
OFF_AT0_D0VAL = 0x0E8
OFF_AT1_CTRL = 0x0EC
OFF_AT1_MATCH = 0x0F0
OFF_AT1_D0VAL = 0x0F4
OFF_AT_SKIP_ONCE = 0x0F8
OFF_AT_HIT = 0x0FC
OFF_AT_HIT_PC = 0x100
OFF_AT_HIT_A0 = 0x104
OFF_AT_HIT_D0 = 0x108
OFF_FEATURES = 0x0A0
OFF_DBG_RESET_CTL = 0x0A4
OFF_CAP_TRACE = 0x0A8
OFF_DCACHE_OP = 0x210
OFF_ICACHE_OP = 0x214
OFF_CYCLE_LO = 0x1000
OFF_CYCLE_HI = 0x1004
OFF_INST_LO = 0x1008
OFF_INST_HI = 0x100C
OFF_MISPRED_COUNT = 0x1010        # always 0 -- no real source in this design
OFF_FLUSH_COUNT = 0x1014
OFF_EXC_COUNT = 0x1018
OFF_ARCH_D0 = 0x2000              # .. + 0x1C, D0..D7
OFF_ARCH_A0 = 0x2020              # .. + 0x1C, A0..A7
OFF_ARCH_USP = 0x2040
OFF_ARCH_SSP = 0x2044
OFF_ARCH_ISP = 0x2048
OFF_ARCH_SR = 0x204C
OFF_ARCH_VBR = 0x2050
OFF_ARCH_CACR = 0x2054
OFF_ARCH_TC = 0x2058
OFF_ARCH_ITT0 = 0x205C
OFF_ARCH_ITT1 = 0x2060
OFF_ARCH_DTT0 = 0x2064
OFF_ARCH_DTT1 = 0x2068
OFF_ARCH_URP = 0x206C
OFF_ARCH_SRP = 0x2070
OFF_ARCH_PC = 0x2074
OFF_ARCH_APPLY = 0x2078
OFF_ARCH_STATUS = 0x207C
OFF_ARCH_SFC = 0x2080
OFF_ARCH_DFC = 0x2084
OFF_LIVE_VBR = 0x2100
OFF_LIVE_SR = 0x2104
OFF_LIVE_A7 = 0x2108
OFF_LIVE_USP = 0x210C
OFF_LIVE_D0 = 0x2110              # .. + 0x1C, D0..D7 (snap-chain, poisoned)
OFF_LIVE_A0 = 0x2130              # .. + 0x1C, A0..A7 (snap-chain, poisoned)
OFF_LIVE_MMU_TC = 0x2160
OFF_LIVE_MMU_DTT0 = 0x2164
OFF_LIVE_MMU_DTT1 = 0x2168
OFF_LIVE_MMU_ITT0 = 0x216C
OFF_LIVE_MMU_ITT1 = 0x2170
OFF_LIVE_MMU_SRP = 0x2174
OFF_LIVE_MMU_URP = 0x2178
OFF_LIVE_SSP = 0x217C
OFF_LIVE_ISP = 0x2180
OFF_LIVE_CACR = 0x2184
OFF_LIVE_SFC = 0x2188
OFF_LIVE_DFC = 0x218C
OFF_LIVE_PC = 0x2190
OFF_LIVE_MMUSR = 0x2194

VERSION_VALUE = 0xDEB60007
CAP_TRACE_VALUE = 0x01000020
FEATURES_VALUE = sum(
    1 << b for b in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 16, 17)
)  # Legacy host-test surface; Stage-5 capability tests live with the core.


class FakeReplError(Exception):
    """Raised for anything the real REPL would report as `> ERROR ...`."""


class FakeRepl:
    """Software model of tools/jtag_repl.tcl + rtl/core/debug/debug_ctrl.v.

    Construct one, feed it command lines via execute(), and drive
    hardware-like events into it with the stop_*() methods and the memory
    helpers (load_bytes / read_bytes / set_dirty_cache).
    """

    # ── construction ────────────────────────────────────────────────────

    def __init__(self):
        # Flat memory model: word-aligned address -> 32-bit big-endian word.
        self.mem = {}
        # Addresses whose true value is staged here but not yet "pushed" to
        # self.mem -- JTAG (which bypasses the D-cache) reads these as 0
        # until a dcache-op push commits them.
        self.dirty_cache = {}

        self.build_id = 0x00000000

        # CONTROL-derived state.
        self.ctrl_halt_req = False
        self.ctrl_init_done_override = False
        self.ctrl_cold_reset_hold = False
        self.step_macro_arm = False

        # STATUS-derived state.
        self.halted = False
        self.init_done_latched = True
        self.exc_pending = False

        self.pc = 0
        self.last_pc = 0
        self.inst_count = 0
        self.cycle_count = 0

        self.exc_vec = 0
        self.exc_pc = 0
        self.exc_fault_addr = 0
        self.exc_count = 0
        self.flush_count = 0

        self.halt_after_enable = False
        self.halt_after_delta = 0
        self.halt_after_target = 0
        self.halt_after_latched = False

        self.break_pc = [0, 0, 0, 0]
        self.break_pc_enable_mask = 0
        self.break_pc_latched = False
        self.break_pc_hit_valid = False
        self.break_pc_hit_slot = 0
        self.bp_skip_once = 0

        self.auto_halt_latched = False
        self.halt_exc_enable = False
        self.halt_exc_latched = False
        self.halt_exc_vec = 0            # dead register: stored, never consulted
        self.halt_exc_mask = [0] * 8

        self.dbl_fault_latched = False
        self.dbl_fault_pc = 0
        self.dbl_fault_vec = 0

        self.halt_hit_pc = 0
        self.halt_hit_inst = 0

        self.wp_addr = [0, 0]
        self.wp_amask = [0, 0]
        self.wp_value = [0, 0]
        self.wp_ctrl = [0, 0]
        self.wp_latched = False
        self.wp_hit_valid = False
        self.wp_hit_slot = 0
        self.wp_hit_is_store = False
        self.wp_hit_wstrb = 0
        self.wp_hit_addr = 0
        self.wp_hit_data = 0
        self.wp_hit_pc = 0

        self.at_ctrl = [0, 0]
        self.at_match = [0, 0]
        self.at_d0val = [0, 0]
        self.at_skip_once = 0
        self.atrap_latched = False
        self.atrap_hit_valid = False
        self.atrap_hit_slot = 0
        self.atrap_cap_busy = False
        self.atrap_hit_opword = 0
        self.atrap_hit_pc = 0
        self.atrap_hit_a0 = 0
        self.atrap_hit_d0 = 0

        self.cpu_reset_count = 0

        # ARCH_* host-write shadow.
        self.arch_d = [0] * 8
        self.arch_a = [0] * 8
        self.arch_usp = 0
        self.arch_ssp = 0
        self.arch_isp = 0
        self.arch_sr = 0
        self.arch_vbr = 0
        self.arch_cacr = 0
        self.arch_mmu_tc = 0
        self.arch_mmu_itt0 = 0
        self.arch_mmu_itt1 = 0
        self.arch_mmu_dtt0 = 0
        self.arch_mmu_dtt1 = 0
        self.arch_mmu_urp = 0
        self.arch_mmu_srp = 0
        self.arch_pc = 0
        self.arch_sfc = 0
        self.arch_dfc = 0
        self.arch_apply_busy = False
        self.arch_apply_done = False
        self.arch_apply_rejected = False

        # LIVE_* readback (populated by arch-apply, or directly by a test).
        self.live_d = [0] * 8
        self.live_a = [0] * 8
        self.live_vbr = 0
        self.live_sr = 0
        self.live_a7 = 0
        self.live_usp = 0
        self.live_ssp = 0
        self.live_isp = 0
        self.live_cacr = 0
        self.live_sfc = 0
        self.live_dfc = 0
        self.live_mmusr = 0
        self.live_mmu_tc = 0
        self.live_mmu_dtt0 = 0
        self.live_mmu_dtt1 = 0
        self.live_mmu_itt0 = 0
        self.live_mmu_itt1 = 0
        self.live_mmu_srp = 0
        self.live_mmu_urp = 0

        # dcache-op / icache-op SHARE one busy/done pair in real hardware.
        self.dcache_op_busy = False
        self.dcache_op_done = False

        self.last_requested_break_pc = None  # unused (break-pc cmd not modeled)

    # ── parse_num / require_aligned (mirrors tools/jtag_repl.tcl exactly) ──

    @staticmethod
    def parse_num(tok, what="value"):
        """HEX BY DEFAULT, exactly like tools/jtag_repl.tcl's parse_num.

        0x1234 / 0X1234 -> hex.  #1234 -> hex.  d1234 / 1234d -> decimal.
        A bare token ("1234") is HEX ("0x1234"), NOT decimal -- that is the
        whole point of parse_num existing in the real REPL.
        """
        if tok is None or tok == "":
            raise FakeReplError("missing %s" % what)
        t = tok
        if t[:2] in ("0x", "0X"):
            body = t[2:]
            try:
                return int(body, 16)
            except ValueError:
                raise FakeReplError("bad hex %s: %s" % (what, tok))
        if t[:1] == "#":
            body = t[1:]
            try:
                return int(body, 16)
            except ValueError:
                raise FakeReplError("bad hex %s: %s" % (what, tok))
        if t[:1] in ("d", "D") and len(t) > 1:
            body = t[1:]
            if not FakeRepl._is_strict_decimal(body):
                raise FakeReplError("bad decimal %s: %s" % (what, tok))
            return int(body, 10)
        if t[-1:] in ("d", "D") and len(t) > 1:
            body = t[:-1]
            if not FakeRepl._is_strict_decimal(body):
                raise FakeReplError("bad decimal %s: %s" % (what, tok))
            return int(body, 10)
        try:
            return int(t, 16)
        except ValueError:
            raise FakeReplError("bad hex %s: %s" % (what, tok))

    @staticmethod
    def _is_strict_decimal(body):
        b = body[1:] if body[:1] in ("+", "-") else body
        return b != "" and b.isdigit()

    @staticmethod
    def require_aligned(addr, what="address"):
        if addr & 0x3:
            raise FakeReplError(
                "unaligned %s 0x%08X -- the JTAG-AXI master is 32-bit "
                "word-addressed and would silently read 0x%08X instead. "
                "Re-issue with an aligned address."
                % (what, addr, addr & ~0x3)
            )
        return addr

    @staticmethod
    def _tcl_expr_int(tok):
        """Bare-Tcl-`expr` numeric parse: arch-write's real (quirky) path.

        Unlike parse_num, a plain token here is DECIMAL, not hex.  This is
        a genuine asymmetry in tools/jtag_repl.tcl (arch-write's operand is
        parsed with `expr {[lindex $tokens 2]}}`, not parse_num) and is
        preserved here on purpose.
        """
        if tok is None or tok == "":
            raise FakeReplError("missing value")
        t = tok.strip()
        if t[:2].lower() == "0x":
            try:
                return int(t, 16)
            except ValueError:
                raise FakeReplError("bad value: %s" % tok)
        try:
            return int(t, 10)
        except ValueError:
            raise FakeReplError("bad value: %s" % tok)

    @staticmethod
    def _sr_check_valid(sr):
        bad = sr & ~0xF71F
        if bad == 0:
            return ""
        return (
            "INVALID: reserved SR bits 0x%04X set (impossible on a 68040 -- "
            "stale/mis-unpacked readback or pre-SR-mask bitstream; DO NOT "
            "TRUST)" % bad
        )

    # ── memory model ────────────────────────────────────────────────────

    def load_bytes(self, addr, data):
        """Store `data` (bytes-like) at byte address `addr`, big-endian."""
        for i, b in enumerate(bytearray(data)):
            a = addr + i
            word_addr = a & ~0x3
            shift = (3 - (a & 0x3)) * 8
            cur = self.mem.get(word_addr, 0)
            cur = (cur & ~(0xFF << shift)) | ((b & 0xFF) << shift)
            self.mem[word_addr] = cur

    def read_bytes(self, addr, n):
        """Read n bytes starting at byte address addr, big-endian."""
        out = bytearray()
        for i in range(n):
            a = addr + i
            word_addr = a & ~0x3
            shift = (3 - (a & 0x3)) * 8
            out.append((self._mem_read_word(word_addr) >> shift) & 0xFF)
        return bytes(out)

    def set_dirty_cache(self, addr, value):
        """Model "the CPU wrote `value` into the D-cache but hasn't pushed
        it to RAM yet".  JTAG (which bypasses the cache) reads `addr` as 0
        until a `dcache-op push` commits it -- the real "freshly written
        RAM reads as zero over JTAG" trap.
        """
        if addr & 0x3:
            raise ValueError("set_dirty_cache: addr must be word-aligned")
        self.dirty_cache[addr] = value & 0xFFFFFFFF

    def _mem_read_word(self, addr):
        if addr in self.dirty_cache:
            return 0
        return self.mem.get(addr, 0)

    def _mem_write_word(self, addr, value):
        self.mem[addr] = value & 0xFFFFFFFF

    def _dcache_push(self):
        for a, v in self.dirty_cache.items():
            self.mem[a] = v
        self.dirty_cache.clear()

    # ── scriptable hardware-like stop events ────────────────────────────

    def _recompute_halted(self):
        self.halted = bool(self.ctrl_halt_req or self.auto_halt_latched)

    def stop_breakpoint(self, slot, pc):
        """Simulate hitting PC breakpoint `slot` (0..3) at `pc`."""
        if slot not in (0, 1, 2, 3):
            raise ValueError("breakpoint slot must be 0..3")
        self.break_pc_latched = True
        self.auto_halt_latched = True
        self.halt_hit_pc = pc & 0xFFFFFFFF
        self.halt_hit_inst = self.inst_count
        self.break_pc_hit_valid = True
        self.break_pc_hit_slot = slot
        self._recompute_halted()

    def stop_watchpoint(self, slot, addr, data, pc, is_store, wstrb):
        """Simulate a data watchpoint hit on slot 0/1.

        Sticky: if a WP hit report is already latched (host hasn't cleared
        WP_HIT since the last one), the DETAIL fields are NOT overwritten
        -- only the generic halt state (wp_latched / auto_halt_latched /
        HALT_HIT_PC) updates.  This matches the real RTL's
        `if (dbg_wp_hit && !wp_hit_valid_r)` guard.
        """
        if slot not in (0, 1):
            raise ValueError("watchpoint slot must be 0 or 1")
        self.wp_latched = True
        self.auto_halt_latched = True
        self.halt_hit_pc = pc & 0xFFFFFFFF
        self.halt_hit_inst = self.inst_count
        if not self.wp_hit_valid:
            self.wp_hit_valid = True
            self.wp_hit_slot = slot
            self.wp_hit_is_store = bool(is_store)
            self.wp_hit_wstrb = wstrb & 0xF
            self.wp_hit_addr = addr & 0xFFFFFFFF
            self.wp_hit_data = data & 0xFFFFFFFF
            self.wp_hit_pc = pc & 0xFFFFFFFF
        self._recompute_halted()

    def stop_atrap(self, slot, opword, pc, a0, d0):
        """Simulate an A-trap breakpoint hit on slot 0/1.

        Sets BOTH break_pc_latched (the shared completion bit) and the
        dedicated atrap_latched bit (HALT_REASON bit 12), per the real
        RTL's dbg_auto_halt_event(reason[1]) + atrap_latched_r pairing.
        """
        if slot not in (0, 1):
            raise ValueError("atrap slot must be 0 or 1")
        self.break_pc_latched = True
        self.atrap_latched = True
        self.auto_halt_latched = True
        self.halt_hit_pc = pc & 0xFFFFFFFF
        self.halt_hit_inst = self.inst_count
        self.atrap_hit_valid = True
        self.atrap_hit_slot = slot
        self.atrap_hit_opword = opword & 0xFFFF
        self.atrap_hit_pc = pc & 0xFFFFFFFF
        self.atrap_hit_a0 = a0 & 0xFFFFFFFF
        self.atrap_hit_d0 = d0 & 0xFFFFFFFF
        self._recompute_halted()

    def stop_exception(self, vec, pc, fault_addr=0):
        """Simulate the core taking exception vector `vec` at `pc`.

        EXC_VEC/EXC_PC/EXC_COUNT update unconditionally (that capture is
        not gated).  Whether this ALSO halts the CPU depends on
        halt_exc_enable AND the corresponding bit of the 256-bit
        halt-on-exception mask being set -- exactly like real hardware.
        """
        self.exc_vec = vec & 0xFF
        self.exc_pc = pc & 0xFFFFFFFF
        self.exc_fault_addr = fault_addr & 0xFFFFFFFF
        self.exc_count = (self.exc_count + 1) & 0xFFFFFFFF
        lane, bit = (vec & 0xFF) // 32, (vec & 0xFF) % 32
        masked_in = bool(self.halt_exc_mask[lane] & (1 << bit))
        if self.halt_exc_enable and masked_in:
            self.halt_exc_latched = True
            self.auto_halt_latched = True
            self.halt_hit_pc = pc & 0xFFFFFFFF
            self.halt_hit_inst = self.inst_count
            self._recompute_halted()

    def stop_double_fault(self, vec, pc, fault_addr=0):
        """Simulate a double fault at `pc` taking vector `vec`.

        Unlike stop_exception, this always halts -- there is no
        enable/mask gate on the double-fault path in the real design.
        """
        self.exc_vec = vec & 0xFF
        self.exc_pc = pc & 0xFFFFFFFF
        self.exc_fault_addr = fault_addr & 0xFFFFFFFF
        self.exc_count = (self.exc_count + 1) & 0xFFFFFFFF
        self.dbl_fault_latched = True
        self.dbl_fault_pc = pc & 0xFFFFFFFF
        self.dbl_fault_vec = vec & 0xFF
        self.auto_halt_latched = True
        self.halt_hit_pc = pc & 0xFFFFFFFF
        self.halt_hit_inst = self.inst_count
        self._recompute_halted()

    def stop_halt_after(self):
        """Simulate the halt-after-N-instructions target firing right now
        (at the current self.pc / self.inst_count)."""
        self.auto_halt_latched = True
        self.halt_after_latched = True
        self.halt_hit_pc = self.pc & 0xFFFFFFFF
        self.halt_hit_inst = self.inst_count
        self._recompute_halted()

    def stop_step(self, pc):
        """Simulate a single-step completing at `pc`.

        Sets break_pc_latched (the shared completion bit) WITHOUT setting
        break_pc_hit_valid/AT_HIT.valid/WP_HIT.valid -- this is the
        deliberate ambiguity the module docstring calls out: a host can
        only tell a real breakpoint/atrap/watchpoint hit apart from a
        step-completion by checking those specific hit_valid bits.
        """
        self.break_pc_latched = True
        self.auto_halt_latched = True
        self.halt_hit_pc = pc & 0xFFFFFFFF
        self.halt_hit_inst = self.inst_count
        self._recompute_halted()

    def _check_halt_after_autofire(self):
        if (
            self.halt_after_enable
            and not self.halt_after_latched
            and self.inst_count >= self.halt_after_target
        ):
            self.stop_halt_after()

    def _halt_after_rearm_target(self):
        base = self.halt_hit_inst if self.halt_after_latched else self.inst_count
        return base + self.halt_after_delta

    # ── debug_ctrl.v register file: read side ───────────────────────────

    def _dbg_read(self, off):
        """Return the 32-bit value at debug-CSR offset `off`, or None if
        this offset is not modeled (caller surfaces BADA0BAD for that)."""
        if off == OFF_VERSION:
            return VERSION_VALUE
        if off == OFF_BUILD_ID:
            return self.build_id & 0xFFFFFFFF
        if off == OFF_CONTROL:
            return (
                ((1 if self.step_macro_arm else 0) << 7)
                | ((1 if self.ctrl_cold_reset_hold else 0) << 4)
                | ((1 if self.ctrl_init_done_override else 0) << 3)
                | (1 if self.ctrl_halt_req else 0)
            )
        if off == OFF_STATUS:
            return (
                ((1 if self.auto_halt_latched else 0) << 4)
                | ((0 if self.halted else 1) << 3)
                | ((1 if self.init_done_latched else 0) << 2)
                | ((1 if self.exc_pending else 0) << 1)
                | (1 if self.halted else 0)
            )
        if off == OFF_PC:
            return self.pc & 0xFFFFFFFF
        if off == OFF_LAST_PC:
            return self.last_pc & 0xFFFFFFFF
        if off == OFF_EXC_VEC:
            return self.exc_vec & 0xFF
        if off == OFF_EXC_PC:
            return self.exc_pc & 0xFFFFFFFF
        if off == OFF_EXC_FAULT_ADDR:
            return self.exc_fault_addr & 0xFFFFFFFF
        if off == OFF_HALT_AFTER_LO:
            return self.halt_after_delta & 0xFFFFFFFF
        if off == OFF_HALT_AFTER_HI:
            return (self.halt_after_delta >> 32) & 0xFFFFFFFF
        if off == OFF_BREAK_PC0:
            return self.break_pc[0] & 0xFFFFFFFF
        if off == OFF_BREAK_PC1:
            return self.break_pc[1] & 0xFFFFFFFF
        if off == OFF_BREAK_PC2:
            return self.break_pc[2] & 0xFFFFFFFF
        if off == OFF_BREAK_PC3:
            return self.break_pc[3] & 0xFFFFFFFF
        if off == OFF_HALT_CTL:
            return (
                (1 if self.halt_after_enable else 0)
                | ((1 if self.break_pc_enable_mask else 0) << 1)
                | ((1 if self.auto_halt_latched else 0) << 3)
                | ((1 if self.halt_after_latched else 0) << 4)
                | ((1 if self.break_pc_latched else 0) << 5)
                | ((1 if self.halt_exc_enable else 0) << 6)
                | ((1 if self.halt_exc_latched else 0) << 7)
            )
        if off == OFF_HALT_REASON:
            return (
                (1 if self.ctrl_halt_req else 0)
                | ((1 if self.halt_after_latched else 0) << 1)
                | ((1 if self.break_pc_latched else 0) << 2)
                | ((1 if self.halted else 0) << 3)
                | ((1 if self.halt_after_enable else 0) << 4)
                | ((1 if self.break_pc_enable_mask else 0) << 5)
                | ((1 if self.halt_exc_latched else 0) << 6)
                | ((1 if self.halt_exc_enable else 0) << 7)
                | ((1 if self.dbl_fault_latched else 0) << 8)
                | ((1 if self.wp_latched else 0) << 11)
                | ((1 if self.atrap_latched else 0) << 12)
            )
        if off == OFF_HALT_HIT_PC:
            return self.halt_hit_pc & 0xFFFFFFFF
        if off == OFF_HALT_HIT_INST_LO:
            return self.halt_hit_inst & 0xFFFFFFFF
        if off == OFF_HALT_HIT_INST_HI:
            return (self.halt_hit_inst >> 32) & 0xFFFFFFFF
        if off == OFF_HALT_EXC_VEC:
            return self.halt_exc_vec & 0xFF
        if off == OFF_PC_MISALIGNED_PC:
            return 0
        if (
            OFF_HALT_EXC_MASK0 <= off <= OFF_HALT_EXC_MASK0 + 0x1C
            and (off - OFF_HALT_EXC_MASK0) % 4 == 0
        ):
            return self.halt_exc_mask[(off - OFF_HALT_EXC_MASK0) // 4]
        if off == OFF_BP_SKIP_ONCE:
            return self.bp_skip_once & 0xF
        if off == OFF_BREAK_PC_CTRL:
            return (
                ((1 if self.break_pc_hit_valid else 0) << 15)
                | ((self.break_pc_hit_slot & 0x3) << 8)
                | (self.break_pc_enable_mask & 0xF)
            )
        if off == OFF_DBL_FAULT_PC:
            return self.dbl_fault_pc & 0xFFFFFFFF
        if off == OFF_DBL_FAULT_VEC:
            return ((1 if self.dbl_fault_latched else 0) << 8) | (
                self.dbl_fault_vec & 0xFF
            )
        if off == OFF_WP0_ADDR:
            return self.wp_addr[0] & 0xFFFFFFFF
        if off == OFF_WP0_AMASK:
            return self.wp_amask[0] & 0xFFFFFFFF
        if off == OFF_WP0_VALUE:
            return self.wp_value[0] & 0xFFFFFFFF
        if off == OFF_WP0_CTRL:
            return self.wp_ctrl[0] & 0xFFFF
        if off == OFF_WP1_ADDR:
            return self.wp_addr[1] & 0xFFFFFFFF
        if off == OFF_WP1_AMASK:
            return self.wp_amask[1] & 0xFFFFFFFF
        if off == OFF_WP1_VALUE:
            return self.wp_value[1] & 0xFFFFFFFF
        if off == OFF_WP1_CTRL:
            return self.wp_ctrl[1] & 0xFFFF
        if off == OFF_WP_HIT:
            return (
                ((self.wp_hit_wstrb & 0xF) << 12)
                | ((1 if self.wp_hit_is_store else 0) << 2)
                | ((self.wp_hit_slot & 0x1) << 1)
                | (1 if self.wp_hit_valid else 0)
            )
        if off == OFF_WP_HIT_ADDR:
            return self.wp_hit_addr & 0xFFFFFFFF
        if off == OFF_WP_HIT_DATA:
            return self.wp_hit_data & 0xFFFFFFFF
        if off == OFF_WP_HIT_PC:
            return self.wp_hit_pc & 0xFFFFFFFF
        if off == OFF_AT0_CTRL:
            return self.at_ctrl[0] & 0xFFFF
        if off == OFF_AT0_MATCH:
            return self.at_match[0] & 0xFFFFFFFF
        if off == OFF_AT0_D0VAL:
            return self.at_d0val[0] & 0xFFFFFFFF
        if off == OFF_AT1_CTRL:
            return self.at_ctrl[1] & 0xFFFF
        if off == OFF_AT1_MATCH:
            return self.at_match[1] & 0xFFFFFFFF
        if off == OFF_AT1_D0VAL:
            return self.at_d0val[1] & 0xFFFFFFFF
        if off == OFF_AT_SKIP_ONCE:
            return self.at_skip_once & 0x3
        if off == OFF_AT_HIT:
            return (
                ((self.atrap_hit_opword & 0xFFFF) << 16)
                | ((1 if self.atrap_cap_busy else 0) << 2)
                | ((self.atrap_hit_slot & 0x1) << 1)
                | (1 if self.atrap_hit_valid else 0)
            )
        if off == OFF_AT_HIT_PC:
            return self.atrap_hit_pc & 0xFFFFFFFF
        if off == OFF_AT_HIT_A0:
            return self.atrap_hit_a0 & 0xFFFFFFFF
        if off == OFF_AT_HIT_D0:
            return self.atrap_hit_d0 & 0xFFFFFFFF
        if off == OFF_FEATURES:
            return FEATURES_VALUE
        if off == OFF_DBG_RESET_CTL:
            return (self.cpu_reset_count & 0xFFFF) << 16
        if off == OFF_CAP_TRACE:
            return CAP_TRACE_VALUE
        if off == OFF_CYCLE_LO:
            return self.cycle_count & 0xFFFFFFFF
        if off == OFF_CYCLE_HI:
            return (self.cycle_count >> 32) & 0xFFFFFFFF
        if off == OFF_INST_LO:
            return self.inst_count & 0xFFFFFFFF
        if off == OFF_INST_HI:
            return (self.inst_count >> 32) & 0xFFFFFFFF
        if off == OFF_MISPRED_COUNT:
            return 0
        if off == OFF_FLUSH_COUNT:
            return self.flush_count & 0xFFFFFFFF
        if off == OFF_EXC_COUNT:
            return self.exc_count & 0xFFFFFFFF
        if OFF_ARCH_D0 <= off <= OFF_ARCH_D0 + 0x1C and (off - OFF_ARCH_D0) % 4 == 0:
            return self.arch_d[(off - OFF_ARCH_D0) // 4] & 0xFFFFFFFF
        if OFF_ARCH_A0 <= off <= OFF_ARCH_A0 + 0x1C and (off - OFF_ARCH_A0) % 4 == 0:
            return self.arch_a[(off - OFF_ARCH_A0) // 4] & 0xFFFFFFFF
        if off == OFF_ARCH_USP:
            return self.arch_usp & 0xFFFFFFFF
        if off == OFF_ARCH_SSP:
            return self.arch_ssp & 0xFFFFFFFF
        if off == OFF_ARCH_ISP:
            return self.arch_isp & 0xFFFFFFFF
        if off == OFF_ARCH_SR:
            return self.arch_sr & 0xFFFF
        if off == OFF_ARCH_VBR:
            return self.arch_vbr & 0xFFFFFFFF
        if off == OFF_ARCH_CACR:
            return self.arch_cacr & 0xFFFFFFFF
        if off == OFF_ARCH_TC:
            return self.arch_mmu_tc & 0xFFFFFFFF
        if off == OFF_ARCH_ITT0:
            return self.arch_mmu_itt0 & 0xFFFFFFFF
        if off == OFF_ARCH_ITT1:
            return self.arch_mmu_itt1 & 0xFFFFFFFF
        if off == OFF_ARCH_DTT0:
            return self.arch_mmu_dtt0 & 0xFFFFFFFF
        if off == OFF_ARCH_DTT1:
            return self.arch_mmu_dtt1 & 0xFFFFFFFF
        if off == OFF_ARCH_URP:
            return self.arch_mmu_urp & 0xFFFFFFFF
        if off == OFF_ARCH_SRP:
            return self.arch_mmu_srp & 0xFFFFFFFF
        if off == OFF_ARCH_PC:
            return self.arch_pc & 0xFFFFFFFF
        if off == OFF_ARCH_APPLY:
            return 1 if self.arch_apply_busy else 0
        if off == OFF_ARCH_STATUS:
            return (
                ((1 if self.arch_apply_rejected else 0) << 2)
                | ((1 if self.arch_apply_done else 0) << 1)
                | (1 if self.arch_apply_busy else 0)
            )
        if off == OFF_ARCH_SFC:
            return self.arch_sfc & 0x7
        if off == OFF_ARCH_DFC:
            return self.arch_dfc & 0x7
        if off == OFF_LIVE_VBR:
            return self.live_vbr & 0xFFFFFFFF
        if off == OFF_LIVE_SR:
            return self.live_sr & 0xFFFF
        if off == OFF_LIVE_A7:
            return self.live_a7 & 0xFFFFFFFF
        if off == OFF_LIVE_USP:
            return self.live_usp & 0xFFFFFFFF
        if off == OFF_LIVE_SSP:
            return self.live_ssp & 0xFFFFFFFF
        if off == OFF_LIVE_ISP:
            return self.live_isp & 0xFFFFFFFF
        if OFF_LIVE_D0 <= off <= OFF_LIVE_D0 + 0x1C and (off - OFF_LIVE_D0) % 4 == 0:
            idx = (off - OFF_LIVE_D0) // 4
            return self.live_d[idx] & 0xFFFFFFFF if self.halted else 0xDEADBEEF
        if OFF_LIVE_A0 <= off <= OFF_LIVE_A0 + 0x1C and (off - OFF_LIVE_A0) % 4 == 0:
            idx = (off - OFF_LIVE_A0) // 4
            return self.live_a[idx] & 0xFFFFFFFF if self.halted else 0xDEADBEEF
        if off == OFF_LIVE_MMU_TC:
            return self.live_mmu_tc & 0xFFFFFFFF
        if off == OFF_LIVE_MMU_DTT0:
            return self.live_mmu_dtt0 & 0xFFFFFFFF
        if off == OFF_LIVE_MMU_DTT1:
            return self.live_mmu_dtt1 & 0xFFFFFFFF
        if off == OFF_LIVE_MMU_ITT0:
            return self.live_mmu_itt0 & 0xFFFFFFFF
        if off == OFF_LIVE_MMU_ITT1:
            return self.live_mmu_itt1 & 0xFFFFFFFF
        if off == OFF_LIVE_MMU_SRP:
            return self.live_mmu_srp & 0xFFFFFFFF
        if off == OFF_LIVE_MMU_URP:
            return self.live_mmu_urp & 0xFFFFFFFF
        if off == OFF_LIVE_CACR:
            return self.live_cacr & 0xFFFFFFFF
        if off == OFF_LIVE_SFC:
            return self.live_sfc & 0x7
        if off == OFF_LIVE_DFC:
            return self.live_dfc & 0x7
        if off == OFF_LIVE_PC:
            return self.pc & 0xFFFFFFFF
        if off == OFF_LIVE_MMUSR:
            return self.live_mmusr & 0xFFFFFFFF
        if off in (OFF_DCACHE_OP, OFF_ICACHE_OP):
            return ((1 if self.dcache_op_done else 0) << 1) | (
                1 if self.dcache_op_busy else 0
            )
        return None

    # ── debug_ctrl.v register file: write side ──────────────────────────

    def _dbg_write(self, off, val):
        """Apply a 32-bit write to debug-CSR offset `off`.  Offsets that
        are read-only or unmodeled are a silent no-op (matches the real
        AXI slave: an OKAY write to an address the case statement doesn't
        match simply changes nothing)."""
        val &= 0xFFFFFFFF

        if off == OFF_CONTROL:
            new_halt_req = bool(val & 0x1)
            if (not new_halt_req) and self.halt_after_latched:
                self.auto_halt_latched = False
                self.halt_after_latched = False
                self.halt_after_target = self._halt_after_rearm_target()
                self.halt_hit_pc = 0
                self.halt_hit_inst = 0
            self.ctrl_halt_req = new_halt_req
            if val & 0x80:
                self.step_macro_arm = True
            elif val & 0x2:
                self.step_macro_arm = True
            self.ctrl_init_done_override = bool(val & 0x8)
            self.ctrl_cold_reset_hold = bool(val & 0x10)
            self._recompute_halted()
            return

        if off == OFF_HALT_AFTER_LO:
            self.halt_after_delta = (self.halt_after_delta & ~0xFFFFFFFF) | val
            self.halt_after_target = self.inst_count + self.halt_after_delta
            self.halt_after_latched = False
            self._recompute_halted()
            return
        if off == OFF_HALT_AFTER_HI:
            self.halt_after_delta = (self.halt_after_delta & 0xFFFFFFFF) | (
                val << 32
            )
            self.halt_after_target = self.inst_count + self.halt_after_delta
            self.halt_after_latched = False
            self._recompute_halted()
            return

        if off == OFF_BREAK_PC0:
            self.break_pc[0] = val
            return
        if off == OFF_BREAK_PC1:
            self.break_pc[1] = val
            return
        if off == OFF_BREAK_PC2:
            self.break_pc[2] = val
            return
        if off == OFF_BREAK_PC3:
            self.break_pc[3] = val
            return

        if off == OFF_HALT_CTL:
            self.halt_after_enable = bool(val & 0x1)
            # Real RTL: this write sets break_pc_enable_r[0] ONLY.
            if val & 0x2:
                self.break_pc_enable_mask |= 0x1
            else:
                self.break_pc_enable_mask &= ~0x1
            self.halt_exc_enable = bool(val & 0x40)
            if val & 0x4:  # clear pulse: wipes every auto-halt latch
                if self.halt_after_latched:
                    self.halt_after_target = self._halt_after_rearm_target()
                self.auto_halt_latched = False
                self.halt_after_latched = False
                self.break_pc_latched = False
                self.halt_exc_latched = False
                self.dbl_fault_latched = False
                self.dbl_fault_pc = 0
                self.dbl_fault_vec = 0
                self.wp_latched = False
                self.atrap_latched = False
                self.halt_hit_pc = 0
                self.halt_hit_inst = 0
            self._recompute_halted()
            return

        if off == OFF_HALT_EXC_VEC:
            self.halt_exc_vec = val & 0xFF  # dead: stored, never consulted
            return
        if (
            OFF_HALT_EXC_MASK0 <= off <= OFF_HALT_EXC_MASK0 + 0x1C
            and (off - OFF_HALT_EXC_MASK0) % 4 == 0
        ):
            self.halt_exc_mask[(off - OFF_HALT_EXC_MASK0) // 4] = val
            return

        if off == OFF_BP_SKIP_ONCE:
            self.bp_skip_once = val & 0xF
            return
        if off == OFF_BREAK_PC_CTRL:
            # QUIRK: enables live in the low byte; the hit-valid clear
            # tests bit 14, NOT bit 15 (bit 15 is where hit_valid READS).
            self.break_pc_enable_mask = val & 0xF
            if val & (1 << 14):
                self.break_pc_hit_valid = False
            return

        if off == OFF_WP0_ADDR:
            self.wp_addr[0] = val
            return
        if off == OFF_WP0_AMASK:
            self.wp_amask[0] = val
            return
        if off == OFF_WP0_VALUE:
            self.wp_value[0] = val
            return
        if off == OFF_WP0_CTRL:
            self.wp_ctrl[0] = val & 0xFFFF
            return
        if off == OFF_WP1_ADDR:
            self.wp_addr[1] = val
            return
        if off == OFF_WP1_AMASK:
            self.wp_amask[1] = val
            return
        if off == OFF_WP1_VALUE:
            self.wp_value[1] = val
            return
        if off == OFF_WP1_CTRL:
            self.wp_ctrl[1] = val & 0xFFFF
            return
        if off == OFF_WP_HIT:
            if val & 0x1:  # symmetric W1C (unlike BREAK_PC_CTRL)
                self.wp_hit_valid = False
            return

        if off == OFF_AT0_CTRL:
            self.at_ctrl[0] = val & 0xFFFF
            return
        if off == OFF_AT0_MATCH:
            self.at_match[0] = val
            return
        if off == OFF_AT0_D0VAL:
            self.at_d0val[0] = val
            return
        if off == OFF_AT1_CTRL:
            self.at_ctrl[1] = val & 0xFFFF
            return
        if off == OFF_AT1_MATCH:
            self.at_match[1] = val
            return
        if off == OFF_AT1_D0VAL:
            self.at_d0val[1] = val
            return
        if off == OFF_AT_SKIP_ONCE:
            self.at_skip_once = val & 0x3
            return
        if off == OFF_AT_HIT:
            if val & 0x1:
                self.atrap_hit_valid = False
            return

        if off == OFF_DBG_RESET_CTL:
            if val & 0x2:
                self.cpu_reset_count = 0
            # bit 0 (cfg_wipe) accepted, not further modeled: this fake has
            # no host-programmed-config-vs-defaults distinction to restore.
            return

        if OFF_ARCH_D0 <= off <= OFF_ARCH_D0 + 0x1C and (off - OFF_ARCH_D0) % 4 == 0:
            self.arch_d[(off - OFF_ARCH_D0) // 4] = val
            return
        if OFF_ARCH_A0 <= off <= OFF_ARCH_A0 + 0x1C and (off - OFF_ARCH_A0) % 4 == 0:
            self.arch_a[(off - OFF_ARCH_A0) // 4] = val
            return
        if off == OFF_ARCH_USP:
            self.arch_usp = val
            return
        if off == OFF_ARCH_SSP:
            self.arch_ssp = val
            return
        if off == OFF_ARCH_ISP:
            self.arch_isp = val
            return
        if off == OFF_ARCH_SR:
            self.arch_sr = val & 0xFFFF
            return
        if off == OFF_ARCH_VBR:
            self.arch_vbr = val
            return
        if off == OFF_ARCH_CACR:
            self.arch_cacr = val
            return
        if off == OFF_ARCH_TC:
            self.arch_mmu_tc = val
            return
        if off == OFF_ARCH_ITT0:
            self.arch_mmu_itt0 = val
            return
        if off == OFF_ARCH_ITT1:
            self.arch_mmu_itt1 = val
            return
        if off == OFF_ARCH_DTT0:
            self.arch_mmu_dtt0 = val
            return
        if off == OFF_ARCH_DTT1:
            self.arch_mmu_dtt1 = val
            return
        if off == OFF_ARCH_URP:
            self.arch_mmu_urp = val
            return
        if off == OFF_ARCH_SRP:
            self.arch_mmu_srp = val
            return
        if off == OFF_ARCH_PC:
            self.arch_pc = val
            return
        if off == OFF_ARCH_SFC:
            self.arch_sfc = val & 0x7
            return
        if off == OFF_ARCH_DFC:
            self.arch_dfc = val & 0x7
            return
        if off == OFF_ARCH_APPLY:
            self._arch_apply_write(val)
            return

        if off in (OFF_DCACHE_OP, OFF_ICACHE_OP):
            self._dcache_op_write(val, is_icache=(off == OFF_ICACHE_OP))
            return

        # Everything else (VERSION, BUILD_ID, STATUS, PC, EXC_VEC/PC,
        # HALT_REASON, HALT_HIT_*, DBL_FAULT_*, WP_HIT_*, AT_HIT_*,
        # FEATURES, CAP_TRACE, CYCLE/INST/EXC/FLUSH/MISPRED counters,
        # ARCH_STATUS, all LIVE_* readback) is read-only in the real
        # hardware too: silent no-op.
        return

    def _arch_apply_write(self, val):
        if val & 0x2:  # ARCH_APPLY_CLEAR_STATUS
            self.arch_apply_done = False
            self.arch_apply_rejected = False
        if val & 0x1:  # ARCH_APPLY_START
            self.arch_apply_done = False
            if self.arch_apply_busy or not self.halted:
                self.arch_apply_rejected = True
            else:
                self.arch_apply_rejected = False
                self.arch_apply_busy = True
                self._perform_arch_apply()
                self.arch_apply_busy = False
                self.arch_apply_done = True

    def _perform_arch_apply(self):
        self.live_d = list(self.arch_d)
        self.live_a = list(self.arch_a)
        self.live_sr = self.arch_sr & 0xFFFF
        self.live_vbr = self.arch_vbr & 0xFFFFFFFF
        self.live_cacr = self.arch_cacr & 0xFFFFFFFF
        self.live_mmu_tc = self.arch_mmu_tc & 0xFFFFFFFF
        self.live_mmu_itt0 = self.arch_mmu_itt0 & 0xFFFFFFFF
        self.live_mmu_itt1 = self.arch_mmu_itt1 & 0xFFFFFFFF
        self.live_mmu_dtt0 = self.arch_mmu_dtt0 & 0xFFFFFFFF
        self.live_mmu_dtt1 = self.arch_mmu_dtt1 & 0xFFFFFFFF
        self.live_mmu_urp = self.arch_mmu_urp & 0xFFFFFFFF
        self.live_mmu_srp = self.arch_mmu_srp & 0xFFFFFFFF
        self.live_usp = self.arch_usp & 0xFFFFFFFF
        self.live_ssp = self.arch_ssp & 0xFFFFFFFF
        self.live_isp = self.arch_isp & 0xFFFFFFFF
        self.live_sfc = self.arch_sfc & 0x7
        self.live_dfc = self.arch_dfc & 0x7
        # Simplification (see module docstring): mirror arch_a[7] directly
        # instead of modeling the real SR.S/M-dependent USP/SSP/ISP mux.
        self.live_a7 = self.arch_a[7] & 0xFFFFFFFF
        self.pc = self.arch_pc & 0xFFFFFFFF

        # Stage-3 apply preserves the effective halt and stop-reason latches.
        self._recompute_halted()

    def _dcache_op_write(self, val, is_icache):
        start = bool(val & 0x1)
        kind_push = bool((val >> 1) & 0x1)  # meaningful for dcache-op only
        if start and not self.dcache_op_busy and self.halted:
            if not is_icache:
                if kind_push:
                    self._dcache_push()
                else:
                    self.dirty_cache.clear()  # invalidate: discard, no writeback
            self.dcache_op_busy = False
            self.dcache_op_done = True
        # else: rejected while running, or already busy -- silent no-op.

    # ── raw AXI-style read/write used by r / w / dump-mem ───────────────

    def _raw_read(self, addr):
        if DBG_BASE <= addr < DBG_BASE + DBG_WINDOW:
            val = self._dbg_read(addr - DBG_BASE)
            if val is None:
                return BADA0BAD
            return "%08X" % (val & 0xFFFFFFFF)
        return "%08X" % self._mem_read_word(addr)

    def _raw_write(self, addr, data):
        if DBG_BASE <= addr < DBG_BASE + DBG_WINDOW:
            self._dbg_write(addr - DBG_BASE, data)
        else:
            self._mem_write_word(addr, data)

    # ── halt-status formatting (verbatim halt_status_line port) ─────────

    def _halt_status_line(self):
        hr = self._dbg_read(OFF_HALT_REASON)
        hc = self._dbg_read(OFF_HALT_CTL)
        hp = self._dbg_read(OFF_HALT_HIT_PC)
        mp = self._dbg_read(OFF_PC_MISALIGNED_PC)
        pc = self._dbg_read(OFF_PC)
        ev = self._dbg_read(OFF_EXC_VEC)
        ep = self._dbg_read(OFF_EXC_PC)
        ec = self._dbg_read(OFF_EXC_COUNT)

        manual = 1 if (hr & 0x01) else 0
        halt_after = 1 if (hr & 0x02) else 0
        break_pc = 1 if (hr & 0x04) else 0
        effective = 1 if (hr & 0x08) else 0
        exc_halt = 1 if (hr & 0x40) else 0
        dbl_fault = 1 if (hr & 0x100) else 0
        pc_misaligned = 1 if (hr & 0x200) else 0

        enables = "ha=%d bp=%d exc=%d pcmis=%d" % (
            1 if (hr & 0x10) else 0,
            1 if (hr & 0x20) else 0,
            1 if (hr & 0x80) else 0,
            1 if (hr & 0x400) else 0,
        )
        ctl_latches = "auto=%d ha=%d bp=%d exc=%d" % (
            1 if (hc & 0x08) else 0,
            1 if (hc & 0x10) else 0,
            1 if (hc & 0x20) else 0,
            1 if (hc & 0x80) else 0,
        )

        suffix = ""
        if self.last_requested_break_pc is not None:
            want = self.last_requested_break_pc & 0xFFFFFFFF
            if not break_pc or hp != want:
                suffix += " note=break-pc-not-reached expected=0x%08X" % want
        elif (hr & 0x20) and not break_pc:
            suffix += " note=break-pc-armed-no-hit"
        if not effective:
            suffix += " note=not-halted-live-arch-unsafe"
        if dbl_fault:
            dfpc = self._dbg_read(OFF_DBL_FAULT_PC)
            dfvi = self._dbg_read(OFF_DBL_FAULT_VEC)
            suffix += " DBL_FAULT pc=0x%08x vec=0x%02x" % (dfpc, dfvi & 0xFF)

        return (
            "halt: reason=0x%08X ctl=0x%08X hit=0x%08X pc_live=0x%08X "
            "manual=%d halt_after=%d break_pc=%d exc_halt=%d dbl_fault=%d "
            "effective=%d enables={%s} latches={%s} exc_vec=0x%08X "
            "exc_pc=0x%08X exc_count=0x%08X%s\n"
            "pc_misaligned=%d  misaligned_pc=0x%08X"
            % (
                hr, hc, hp, pc,
                manual, halt_after, break_pc, exc_halt, dbl_fault,
                effective, enables, ctl_latches, ev,
                ep, ec, suffix,
                pc_misaligned, mp,
            )
        )

    # ── command handlers ─────────────────────────────────────────────────

    @staticmethod
    def _tok(tokens, idx):
        return tokens[idx] if len(tokens) > idx else ""

    def _cmd_r(self, tokens):
        addr = self.require_aligned(self.parse_num(self._tok(tokens, 1), "address"))
        v = self._raw_read(addr)
        return ["> r 0x%08X = 0x%s" % (addr, v)]

    def _cmd_w(self, tokens):
        addr = self.require_aligned(self.parse_num(self._tok(tokens, 1), "address"))
        data = self.parse_num(self._tok(tokens, 2), "data")
        self._raw_write(addr, data)
        return ["> w 0x%08X = 0x%08X" % (addr, data & 0xFFFFFFFF)]

    def _cmd_halt(self, tokens):
        if len(tokens) > 2:
            raise FakeReplError("usage: halt [wait_ms]")
        if len(tokens) == 2:
            self.parse_num(tokens[1], "halt wait_ms")
        self.ctrl_halt_req = True
        self._recompute_halted()
        return [
            "> halt landed at a coherent macro boundary after 0ms",
            "> %s" % self._halt_status_line(),
        ]

    def _cmd_dump_mem(self, tokens):
        addr = self.require_aligned(self.parse_num(self._tok(tokens, 1), "address"))
        n = self.parse_num(self._tok(tokens, 2), "word count")
        out = []
        for i in range(n):
            a = addr + 4 * i
            v = self._raw_read(a)
            out.append("> mem 0x%08X = 0x%s" % (a, v))
        return out

    def _cmd_live_arch(self, tokens):
        force = len(tokens) > 1 and tokens[1] == "force"
        reason = self._dbg_read(OFF_HALT_REASON)
        if not force and not (reason & 0x08):
            return [
                "> ERROR live-arch requires effective halt; run halt-status, "
                "break-pc, advance, or use `live-arch force`",
                "> %s" % self._halt_status_line(),
            ]
        out = []
        for i in range(8):
            out.append("> D%d = 0x%08X" % (i, self._dbg_read(OFF_LIVE_D0 + i * 4)))
        for i in range(8):
            out.append("> A%d = 0x%08X" % (i, self._dbg_read(OFF_LIVE_A0 + i * 4)))
        sr = self._dbg_read(OFF_LIVE_SR)
        sr_err = self._sr_check_valid(sr)
        if sr_err == "":
            out.append("> SR  = 0x%08X" % sr)
        else:
            out.append("> SR  = 0x%08X  <<< %s" % (sr, sr_err))
        out.append("> VBR = 0x%08X" % self._dbg_read(OFF_LIVE_VBR))
        out.append("> A7  = 0x%08X" % self._dbg_read(OFF_LIVE_A7))
        out.append("> PC  = 0x%08X" % self._dbg_read(OFF_PC))
        return out

    def _cmd_regs(self, tokens):
        if not self.halted:
            raise FakeReplError("register dump requires an EFFECTIVE halt")
        out = []
        for i in range(8):
            out.append("> D%d = 0x%08X" %
                       (i, self._dbg_read(OFF_LIVE_D0 + i * 4)))
        for i in range(8):
            out.append("> A%d = 0x%08X" %
                       (i, self._dbg_read(OFF_LIVE_A0 + i * 4)))
        fields = (
            ("SR", OFF_LIVE_SR), ("VBR", OFF_LIVE_VBR),
            ("USP", OFF_LIVE_USP), ("MSP", OFF_LIVE_SSP),
            ("ISP", OFF_LIVE_ISP), ("PC", OFF_LIVE_PC),
            ("CACR", OFF_LIVE_CACR), ("SFC", OFF_LIVE_SFC),
            ("DFC", OFF_LIVE_DFC), ("TC", OFF_LIVE_MMU_TC),
            ("ITT0", OFF_LIVE_MMU_ITT0), ("ITT1", OFF_LIVE_MMU_ITT1),
            ("DTT0", OFF_LIVE_MMU_DTT0), ("DTT1", OFF_LIVE_MMU_DTT1),
            ("URP", OFF_LIVE_MMU_URP), ("SRP", OFF_LIVE_MMU_SRP),
            ("MMUSR", OFF_LIVE_MMUSR),
        )
        out.extend("> %s = 0x%08X" % (name, self._dbg_read(off))
                   for name, off in fields)
        return out

    def _cmd_arch_write(self, tokens):
        if len(tokens) < 3:
            raise FakeReplError("usage: arch-write <name> <hex>")
        name = tokens[1].upper()
        val = self._tcl_expr_int(tokens[2])  # QUIRK: decimal-default, not parse_num
        off = -1
        m = re.match(r"^D([0-7])$", name)
        if m:
            off = OFF_ARCH_D0 + int(m.group(1)) * 4
        m = re.match(r"^A([0-7])$", name)
        if m:
            off = OFF_ARCH_A0 + int(m.group(1)) * 4
        named = {
            "USP": OFF_ARCH_USP, "MSP": OFF_ARCH_SSP,
            "SSP": OFF_ARCH_SSP, "ISP": OFF_ARCH_ISP,
            "SR": OFF_ARCH_SR, "VBR": OFF_ARCH_VBR,
            "CACR": OFF_ARCH_CACR, "TC": OFF_ARCH_TC,
            "ITT0": OFF_ARCH_ITT0, "ITT1": OFF_ARCH_ITT1,
            "DTT0": OFF_ARCH_DTT0, "DTT1": OFF_ARCH_DTT1,
            "URP": OFF_ARCH_URP, "SRP": OFF_ARCH_SRP,
            "PC": OFF_ARCH_PC, "SFC": OFF_ARCH_SFC, "DFC": OFF_ARCH_DFC,
        }
        off = named.get(name, off)
        if off < 0:
            return ["> ERROR unknown arch reg: %s" % name]
        self._dbg_write(off, val)
        return ["> arch-write %s = 0x%08X" % (name, val & 0xFFFFFFFF)]

    def _cmd_arch_apply(self, tokens):
        self._dbg_write(OFF_ARCH_APPLY, 0x1 | 0x2)
        status = self._dbg_read(OFF_ARCH_STATUS)
        done = 1 if (status & 0x2) else 0
        rejected = 1 if (status & 0x4) else 0
        return ["> arch-apply status=0x%08X done=%d rejected=%d" % (status, done, rejected)]

    def _cmd_continue(self, tokens):
        hc = self._dbg_read(OFF_HALT_CTL)
        bp_latched = bool(hc & 0x20)
        en_slots = 0
        if bp_latched:
            en_slots = self._dbg_read(OFF_BREAK_PC_CTRL) & 0xF
            if en_slots:
                self._dbg_write(OFF_BP_SKIP_ONCE, en_slots)
        halt_enable_bits = self._dbg_read(OFF_HALT_CTL) & (0x1 | 0x2 | 0x40)
        self._dbg_write(OFF_HALT_CTL, halt_enable_bits | 0x4)
        self._dbg_write(OFF_CONTROL, 0x0)
        # Simple, predictable simulated-progress model (see module
        # docstring "SIMPLIFICATIONS"): one simulated retire happens if the
        # CPU actually resumed.
        if not self.halted:
            self.inst_count += 1
            self.cycle_count += 1
            self.last_pc = self.pc
            self.pc = (self.pc + 2) & 0xFFFFFFFF
            self._check_halt_after_autofire()
        return [
            "> continue done (skip_once_armed=%d slots=0x%x)"
            % (1 if bp_latched else 0, en_slots),
            "> %s" % self._halt_status_line(),
        ]

    def _cmd_step(self, tokens):
        st = self._dbg_read(OFF_STATUS)
        if not (st & 0x1):
            raise FakeReplError("step requires CPU halted")
        pc0 = self.pc
        hc = self._dbg_read(OFF_HALT_CTL)
        if hc & 0x20:
            en_slots = self._dbg_read(OFF_BREAK_PC_CTRL) & 0xF
            if en_slots:
                self._dbg_write(OFF_BP_SKIP_ONCE, en_slots)
        self._dbg_write(OFF_CONTROL, 0x1)
        bits = self._dbg_read(OFF_HALT_CTL) & (0x1 | 0x2 | 0x40)
        self._dbg_write(OFF_HALT_CTL, bits | 0x4)
        self._dbg_write(OFF_CONTROL, 0x1 | 0x2)
        self._dbg_write(OFF_CONTROL, 0x0)
        # Simulate exactly one retired macro, then re-halt -- that is the
        # entire point of single-step.
        self.inst_count += 1
        self.cycle_count += 1
        self.last_pc = pc0
        self.pc = (pc0 + 2) & 0xFFFFFFFF
        pc1 = self.pc
        self.stop_step(pc1)
        return [
            "> step pc_before=0x%08X pc_after=0x%08X" % (pc0 & 0xFFFFFFFF, pc1 & 0xFFFFFFFF),
            "> %s" % self._halt_status_line(),
        ]

    def _cmd_dcache_op(self, tokens):
        if len(tokens) < 2:
            raise FakeReplError("usage: dcache-op <inv|push>")
        k = tokens[1].lower()
        if k in ("inv", "invalidate"):
            op = 0
        elif k in ("push", "cpush"):
            op = 1
        else:
            raise FakeReplError("dcache-op kind must be inv or push")
        self._dbg_write(OFF_DCACHE_OP, 0x1 | (op << 1))
        status = self._dbg_read(OFF_DCACHE_OP)
        busy = status & 0x1
        done = (status >> 1) & 0x1
        return ["> dcache-op %s busy=%d done=%d status=0x%08X" % (k, busy, done, status)]

    def _cmd_icache_op(self, tokens):
        k = tokens[1].lower() if len(tokens) > 1 else "inv"
        if k not in ("inv", "invalidate"):
            raise FakeReplError("icache-op kind must be inv")
        self._dbg_write(OFF_ICACHE_OP, 0x1)
        status = self._dbg_read(OFF_ICACHE_OP)
        busy = status & 0x1
        done = (status >> 1) & 0x1
        return ["> icache-op inv busy=%d done=%d status=0x%08X" % (busy, done, status)]

    # ── top-level dispatcher ─────────────────────────────────────────────

    _COMMANDS = {
        "r": "_cmd_r",
        "w": "_cmd_w",
        "dump-mem": "_cmd_dump_mem",
        "arch": "_cmd_regs",
        "regs": "_cmd_regs",
        "reg-dump": "_cmd_regs",
        "halt": "_cmd_halt",
        "live-arch": "_cmd_live_arch",
        "arch-write": "_cmd_arch_write",
        "arch-apply": "_cmd_arch_apply",
        "continue": "_cmd_continue",
        "cont": "_cmd_continue",
        "c": "_cmd_continue",
        "step": "_cmd_step",
        "dcache-op": "_cmd_dcache_op",
        "icache-op": "_cmd_icache_op",
    }

    def _dispatch(self, cmd, tokens):
        if cmd == "halt-status":
            return ["> %s" % self._halt_status_line()]
        method_name = self._COMMANDS.get(cmd)
        if method_name is None:
            raise FakeReplError("unknown command: %s" % cmd)
        return getattr(self, method_name)(tokens)

    def execute(self, line):
        """Consume one REPL command line, return the output lines
        (including the trailing "> READY"), exactly like the real
        tools/jtag_repl.tcl session's line protocol."""
        line = (line or "").strip()
        if line == "":
            return ["> READY"]
        tokens = line.split()
        cmd = tokens[0]
        try:
            out = self._dispatch(cmd, tokens)
        except FakeReplError as e:
            return ["> ERROR %s" % e, "> READY"]
        return out + ["> READY"]


# ─────────────────────────────────────────────────────────────────────────
# main(): FIFO-based driver, so a human (or a gdbstub's own test harness)
# can drive this exactly like the real jtag_repl.tcl session described in
# CLAUDE.md:
#
#     mkfifo /tmp/jtag_in
#     python3 fake_jtag_repl.py --fifo-in /tmp/jtag_in --fifo-out /tmp/jtag_out &
#     echo 'r 0x50900000' > /tmp/jtag_in
#     tail -f /tmp/jtag_out
# ─────────────────────────────────────────────────────────────────────────


def _run_fifo(fifo_in, fifo_out):
    repl = FakeRepl()
    with open(fifo_out, "a", buffering=1) as fout:
        fout.write("> jtag_repl(FAKE): ready\n")
        fout.write("> READY\n")
        fout.flush()
        while True:
            with open(fifo_in, "r") as fin:
                for raw_line in fin:
                    line = raw_line.strip()
                    if line in ("q", "quit", "exit"):
                        fout.write("> bye\n")
                        fout.flush()
                        return 0
                    for out_line in repl.execute(line):
                        fout.write(out_line + "\n")
                    fout.flush()
            # A FIFO's reader sees EOF when every writer closes it; reopen
            # and keep waiting for the next writer, same as a long-lived
            # `nohup bash -c 'exec 7>fifo; sleep 99999'` keeper process.


def _selftest():
    failures = []

    def check(name, cond):
        if not cond:
            failures.append(name)
            print("FAIL: %s" % name)
        else:
            print("ok:   %s" % name)

    r = FakeRepl()

    # VERSION / BUILD_ID.
    out = r.execute("r 0x50900000")
    check("VERSION reads 0xDEB60007", out[0] == "> r 0x50900000 = 0xDEB60007")
    check("VERSION READY", out[-1] == "> READY")

    # parse_num hex-by-default (bare token == hex, not decimal).
    out_bare = r.execute("r 50900000")
    out_0x = r.execute("r 0x50900000")
    check("bare hex token == 0x-prefixed", out_bare[0] == out_0x[0])

    # unaligned read is a hard error.
    out = r.execute("r 0x50900001")
    check("unaligned read errors", out[0].startswith("> ERROR unaligned address"))

    # unmodeled-but-in-window offset -> BADA0BAD, never 0.
    out = r.execute("r 0x50910000")
    check("unmodeled offset -> BADA0BAD", out[0] == "> r 0x50910000 = 0xBADA0BAD")

    # arch-write DECIMAL-default quirk (NOT parse_num's hex-default).
    r.execute("arch-write D0 40")
    d0 = r.execute("r 0x50902000")[0]
    check("arch-write bare '40' == decimal 40 (0x28), not hex 0x40",
          d0 == "> r 0x50902000 = 0x00000028")

    # manual halt via raw CONTROL write.  STATUS bit2 (init_done_latched)
    # defaults True in this fake (it models an already-booted CPU), so the
    # expected value is 0x5 (bit0 halted | bit2 init_done), not bare 0x1.
    r.execute("w 0x50900008 0x1")
    st = r.execute("r 0x5090000C")[0]
    check("CONTROL bit0=1 halts (STATUS bit0)", st == "> r 0x5090000C = 0x00000005")
    hr = r.execute("r 0x50900040")[0]
    check("HALT_REASON reflects ctrl_halt_req", hr == "> r 0x50900040 = 0x00000009")

    # First-class halt waits for the effective architectural stop rather than
    # exposing the raw request-bit sequence to callers.
    r_halt = FakeRepl()
    out = r_halt.execute("halt 200")
    check("halt command reports coherent boundary", out[0].startswith("> halt landed at a coherent macro boundary"))
    check("halt command reaches effective halt", "effective=1" in out[1])

    # halt-status command sanity.
    out = r.execute("halt-status")
    check("halt-status starts with 'halt: '", out[0].startswith("> halt: reason="))
    check("halt-status ends with READY", out[-1] == "> READY")

    # live-arch poisoning while not halted vs real value while halted.
    # NOTE: "force" only bypasses the command's OWN "requires effective
    # halt" early-exit -- it does NOT bypass the snap-chain poisoning,
    # because that poisoning models a real hardware hazard (the snap
    # chain is only coherent while halted), not a command-level safety
    # check.  "force" gets you a read attempt, not a guarantee of a
    # coherent value -- same as real hardware would give you racy
    # garbage instead of a clean refusal.
    r2 = FakeRepl()
    r2.live_d[3] = 0x12345678
    out = r2.execute("live-arch force")
    check("live-arch force still shows poisoned D3 while not halted (snap chain hazard)",
          any(l == "> D3 = 0xDEADBEEF" for l in out))
    r2.ctrl_halt_req = True
    r2._recompute_halted()
    r2.live_d[3] = 0xCAFEBABE
    out = r2.execute("live-arch")
    check("live-arch (halted) shows the real D3", any(l == "> D3 = 0xCAFEBABE" for l in out))
    r2.ctrl_halt_req = False
    r2._recompute_halted()
    out = r2.execute("live-arch")
    check("live-arch requires effective halt without force",
          out[0].startswith("> ERROR live-arch requires effective halt"))
    check("live-arch error path still prints halt-status", out[1].startswith("> halt:"))

    # Stage-3 arch-apply preserves the effective halt.
    r3 = FakeRepl()
    r3.execute("w 0x50900008 0x1")  # manual halt
    r3.execute("arch-write PC 0x1000")
    check("arch-write uses hex-0x path fine too", r3.arch_pc == 0x1000)
    out = r3.execute("arch-apply")
    check("arch-apply reports done=1 rejected=0", "done=1 rejected=0" in out[0])
    check("arch-apply copied arch_pc into live pc", r3.pc == 0x1000)
    check("arch-apply leaves the CPU effectively halted", r3.halted is True)

    # arch-apply REJECTS when not halted.
    r4 = FakeRepl()
    out = r4.execute("arch-apply")
    check("arch-apply rejects when CPU not halted", "rejected=1" in out[0])

    # stop_breakpoint / stop_step disambiguation via break_pc_hit_valid.
    r5 = FakeRepl()
    r5.stop_breakpoint(2, 0x4000)
    hr = r5._dbg_read(OFF_HALT_REASON)
    ctrl = r5._dbg_read(OFF_BREAK_PC_CTRL)
    check("stop_breakpoint sets break_pc_latched", bool(hr & 0x4))
    check("stop_breakpoint sets BREAK_PC_CTRL.hit_valid", bool(ctrl & 0x8000))
    check("stop_breakpoint records slot 2", ((ctrl >> 8) & 0x3) == 2)

    r6 = FakeRepl()
    r6.stop_step(0x4004)
    hr6 = r6._dbg_read(OFF_HALT_REASON)
    ctrl6 = r6._dbg_read(OFF_BREAK_PC_CTRL)
    check("stop_step ALSO sets break_pc_latched (ambiguous by design)", bool(hr6 & 0x4))
    check("stop_step does NOT set BREAK_PC_CTRL.hit_valid (disambiguator)",
          not bool(ctrl6 & 0x8000))

    # BREAK_PC_CTRL bit15-vs-bit14 write-clear quirk.
    r7 = FakeRepl()
    r7.stop_breakpoint(0, 0x8000)
    r7.execute("w 0x50900090 0x8000")  # bit15 set: should be a no-op
    check("writing bit15 to BREAK_PC_CTRL does NOT clear hit_valid",
          r7.break_pc_hit_valid is True)
    r7.execute("w 0x50900090 0x4000")  # bit14 set: real clear
    check("writing bit14 to BREAK_PC_CTRL DOES clear hit_valid",
          r7.break_pc_hit_valid is False)

    # watchpoint sticky first-hit-wins.
    r8 = FakeRepl()
    r8.stop_watchpoint(0, 0x1000, 0xAAAAAAAA, 0x5000, True, 0xF)
    r8.stop_watchpoint(1, 0x2000, 0xBBBBBBBB, 0x5010, False, 0x3)
    check("wp sticky: slot stays the FIRST hit's slot", r8.wp_hit_slot == 0)
    check("wp sticky: addr stays the FIRST hit's addr", r8.wp_hit_addr == 0x1000)
    check("wp generic halt state still updates to the SECOND hit's PC",
          r8.halt_hit_pc == 0x5010)

    # dcache dirty-cache-over-JTAG hazard + dcache-op push.
    r9 = FakeRepl()
    r9.set_dirty_cache(0x00001000, 0xDEADFACE)
    out = r9.execute("r 0x00001000")
    check("dirty cache reads 0 over JTAG before push",
          out[0] == "> r 0x00001000 = 0x00000000")
    r9.execute("w 0x50900008 0x1")  # halt required for dcache-op
    r9.execute("dcache-op push")
    out = r9.execute("r 0x00001000")
    check("dcache-op push commits the dirty value",
          out[0] == "> r 0x00001000 = 0xDEADFACE")

    # dcache-op inv DISCARDS instead of committing.
    r10 = FakeRepl()
    r10.set_dirty_cache(0x00002000, 0x11223344)
    r10.execute("w 0x50900008 0x1")
    r10.execute("dcache-op inv")
    out = r10.execute("r 0x00002000")
    check("dcache-op inv discards the dirty value (still reads 0)",
          out[0] == "> r 0x00002000 = 0x00000000")

    # dcache-op rejected while running.
    r11 = FakeRepl()
    r11.set_dirty_cache(0x00003000, 0x55667788)
    out = r11.execute("dcache-op push")
    check("dcache-op while running is rejected (busy=0 done=0)",
          "busy=0 done=0" in out[0])
    check("dcache-op rejected -> value still not committed",
          r11.execute("r 0x00003000")[0] == "> r 0x00003000 = 0x00000000")

    # dcache-op / icache-op share one busy/done pair.
    r12 = FakeRepl()
    r12.execute("w 0x50900008 0x1")
    r12.execute("dcache-op inv")
    out = r12.execute("icache-op")
    check("icache-op sees dcache-op's busy/done (shared register)",
          "busy=0 done=1" in out[0])

    # step / continue progress.
    r13 = FakeRepl()
    r13.execute("w 0x50900008 0x1")
    pc_before = r13.pc
    ic_before = r13.inst_count
    r13.execute("step")
    check("step advances inst_count by 1", r13.inst_count == ic_before + 1)
    check("step advances pc", r13.pc == pc_before + 2)
    check("step re-halts", r13.halted is True)

    r14 = FakeRepl()
    ic_before = r14.inst_count
    r14.execute("continue")
    check("continue (never halted) still advances one simulated step",
          r14.inst_count == ic_before + 1)

    # halt-on-exception mask gating.
    r15 = FakeRepl()
    r15.execute("w 0x50900008 0x1")  # halt
    r15.execute("w 0x5090003C 0x40")  # HALT_CTL bit6 = halt_exc_enable
    r15.execute("w 0x50900060 0x00000004")  # mask lane 0 bit2 = vec 2
    r15.execute("w 0x5090003C 0x44")  # release clear pulse too (bit2), keep bit6
    r15.ctrl_halt_req = False
    r15._recompute_halted()
    r15.stop_exception(3, 0x2000)  # vec 3 NOT in mask -> should not halt
    check("exception outside mask does not halt", r15.halted is False)
    r15.stop_exception(2, 0x2004)  # vec 2 IS in mask -> should halt
    check("exception inside mask + enabled DOES halt", r15.halted is True)
    check("EXC_VEC/EXC_PC captured", r15.exc_vec == 2 and r15.exc_pc == 0x2004)

    # double fault always halts, no mask needed.
    r16 = FakeRepl()
    r16.stop_double_fault(2, 0x3000)
    check("double fault always halts", r16.halted is True)
    dfv = r16._dbg_read(OFF_DBL_FAULT_VEC)
    check("DBL_FAULT_VEC packs {latched,vec}", dfv == ((1 << 8) | 2))

    print()
    if failures:
        print("SELFTEST: %d FAILURE(S): %s" % (len(failures), ", ".join(failures)))
        return 1
    print("SELFTEST: ALL CHECKS PASSED")
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if "--selftest" in argv:
        return _selftest()

    fifo_in = None
    fifo_out = None
    i = 0
    while i < len(argv):
        if argv[i] == "--fifo-in" and i + 1 < len(argv):
            fifo_in = argv[i + 1]
            i += 2
        elif argv[i] == "--fifo-out" and i + 1 < len(argv):
            fifo_out = argv[i + 1]
            i += 2
        else:
            i += 1

    if not fifo_in or not fifo_out:
        sys.stderr.write(
            "usage: fake_jtag_repl.py --fifo-in <path> --fifo-out <path>\n"
            "       fake_jtag_repl.py --selftest\n"
        )
        return 2

    return _run_fifo(fifo_in, fifo_out)


if __name__ == "__main__":
    sys.exit(main())
