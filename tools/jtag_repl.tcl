## jtag_repl.tcl — long-running Vivado JTAG-AXI REPL.
##
## One Vivado spawn, one open_hw_target, then a stdin command loop.  Avoids
## the ~10s/call cost of spawning fresh Vivado for every axi-read/snapshot.
##
## Usage:
##   vivado -mode tcl -nojournal -nolog -source tools/jtag_repl.tcl \
##          -tclargs <bit> [<ltx>]
## Then write commands to stdin, one per line:
##   r <addr_hex>                       — read 32-bit word at AXI addr
##   w <addr_hex> <data_hex>            — write 32-bit word
##   pc                                 — DBG_PC
##   perf                               — zero-able performance counters: freeze, read a
##                                        formatted table (IPC, per-kilo-instruction
##                                        miss rates, stall breakdown), resume.  The
##                                        freeze is what makes the 64-bit CYCLE/INST
##                                        pairs read ATOMICALLY — OFF_CYCLE_LO/HI and
##                                        OFF_INST_LO/HI cannot be frozen and can tear.
##                                        `perf hold` leaves them frozen; `perf live`
##                                        reads without freezing (pairs may tear);
##                                        `perf freeze` / `perf resume` gate explicitly.
##                                        REFUSES on a bitstream with no counter block
##                                        rather than printing a fabricated table of
##                                        zeros, and renders any counter whose producer
##                                        plugin is absent as `--`, never as 0.
##   perf-clear [hold]                  — zero every windowed counter and start a fresh
##                                        window.  `hold` leaves them frozen so you can
##                                        arm the window before releasing a breakpoint
##                                        or a reset.  The measurement discipline is:
##                                        perf-clear, run the bounded path, perf.
##   halt-status                        — HALT_REASON / HALT_HIT_PC / EXC_VEC / EXC_PC / EXC_COUNT
##                                        plus PC-misalignment tripwire capture
##   halt [wait_ms]                     — request a clean macro-boundary halt and wait for
##                                        the effective-halt acknowledgement (default 200 ms)
##   halt-clear                         — clear auto-halt/break/exception latches, preserving enables
##   halt-release                       — clear latches + release manual/cold-reset hold
##   break-pc <pc> [wait_ms]            — arm PC breakpoint, release, wait, then status
##                                        Phase 3 (2026-05-11) precise-BP:
##                                        when SYS_DBG_BREAK retires, the halt
##                                        fires BEFORE any side effect of the
##                                        BP'd instruction applies (= arch
##                                        state is pre-instruction).  The
##                                        commit-side flush + redirect keep
##                                        PC pinned at break_pc.  Use `continue`
##                                        to resume — skip-once is auto-armed.
##                                        ARMING RACE: after a plain `reset`,
##                                        the CPU free-runs during the seconds
##                                        it takes you to issue `break-pc`, so
##                                        an early-boot target can be missed.
##                                        Use `reset-and-break-pc` (or `reset
##                                        hold` then `break-pc`) to arm while
##                                        the CPU is held.
##                                        On the m68k040 core this is a registered
##                                        frontend marker honored PRE-EFFECT at
##                                        the commit head, so it also catches an
##                                        instruction that would trap. Legacy
##                                        controllers remain retire-triggered.
##   break-pc off                       — disable PC breakpoint + clear stale latch
##   continue / cont / c                — resume from a precise-BP halt:
##                                        clear halt latches, release halt.
##                                        Skip-once was auto-armed when the
##                                        DBG µop fired, so the next decode
##                                        pass at break_pc skips DBG injection
##                                        and runs the real macro; subsequent
##                                        encounters (loop iterations, etc.)
##                                        re-arm.  State mutations via
##                                        arch-write/arch-apply between halt
##                                        and continue ARE honoured (re-fetch
##                                        renames over the mutated PRF).
##   irq-inject <level> [count] [delay_ms]
##                                      — inject autovector IRQ level 1..7;
##                                        repeat count times with delay.
##   dbg-caps                           — report DBG_VERSION + the OFF_FEATURES
##                                        capability bitmap + trace/ring depths
##                                        + the observed-CPU-reset count.  Use
##                                        this instead of assuming what a
##                                        bitstream supports; a bitstream that
##                                        predates the capability block says so
##                                        explicitly rather than reading as
##                                        "no features".
##   mon-sense                          — print the live DAFB monitor-sense
##                                        code (7 bits): bit 6 = "extended
##                                        monitor" flag, bits[5:0] = code.
##   mon-sense <hex>                    — set it.  Was video.v's compile-time
##                                        MONITOR_TYPE parameter, so trying a
##                                        code used to cost a ~50-min
##                                        bitstream.  Monitor sense is what
##                                        Mac OS uses to decide which display
##                                        is attached, hence which resolutions
##                                        and depths the Monitors control panel
##                                        offers.  The value SURVIVES a CPU
##                                        reset (it lives in the debug reset
##                                        domain, cleared only by POR /
##                                        dbg-cfg-wipe) — and a CPU RESET IS
##                                        REQUIRED for it to take effect,
##                                        because Mac OS samples the sense pins
##                                        only during DAFB init.  So:
##                                        `mon-sense 4A` then `reset`.
##                                        Requires OFF_FEATURES bit 18
##                                        (mon_sense); refuses rather than
##                                        reporting 0 (a legal code) on a
##                                        bitstream that lacks the CSR.
##   dbg-cfg-wipe                       — restore host-programmed debug config
##                                        (break-PCs, halt-exc mask, halt-after,
##                                        RAM window, arch shadow) to power-on
##                                        defaults.  Since the debug reset
##                                        domain landed, a CPU reset no longer
##                                        does this — which is the point — so
##                                        this is the explicit way back to a
##                                        known state.
##   reset-and-break-pc <pc> [slot] [wait_ms]
##                                      — hold the CPU in reset, arm a PC
##                                        breakpoint WHILE HELD, then release
##                                        into a cold boot.  This is how you
##                                        catch a ONE-SHOT EARLY-BOOT event:
##                                        before the debug reset domain landed
##                                        the arm was wiped by the reset (and
##                                        the arming write was swallowed by a
##                                        CSR block held in reset for the whole
##                                        MIG-cal + boot_fsm window), so those
##                                        events were unobservable.  Verifies
##                                        the arm survived and says so, because
##                                        a silent "no hit" is ambiguous
##                                        between "never reached" and "never
##                                        armed".
##                                        THIS IS THE RACE-FREE WAY to break on
##                                        an early-boot PC.  Plain `reset` then
##                                        `break-pc` has an ARMING RACE: the CPU
##                                        runs free for the seconds between the
##                                        two commands and can blow past the
##                                        target.  Arming while held closes it.
##                                        (`reset hold` + `break-pc` is the
##                                        manual equivalent — break-pc ends with
##                                        CONTROL=0, which releases the hold.)
##
##                                        2026-08-02: a report that this command
##                                        wedges the debug bridge did NOT
##                                        reproduce — 3/3 clean on bitstream
##                                        0xCDCC2329 (CPU running / halted /
##                                        unreachable PC).  A debug READ with the
##                                        SoC held in reset also returned
##                                        normally, and in RTL the cold-reset
##                                        pulse never reaches dbg_axi at all
##                                        (debug_ctrl runs on dbg_por_rst, and
##                                        the pulse ORs only into soc_full_rst,
##                                        not core_rst).  So the reset is NOT the
##                                        trigger, and this command is not
##                                        special — see below.
##
## KNOWN WEDGE MECHANISM (unfixed, affects ANY command that touches S1):
##   u_jtag_n2w takes axi_narrow_to_wide's default TIMEOUT_CYCLES = 0xFFFFF
##   (~5.24 ms @200 MHz), but xbar slave S1 — where VIA/SCC/SCSI live — may
##   legitimately stall far longer (peripheral_bus watchdog ~335 ms, xbar
##   WD_LOG2_S1=27 ~671 ms).  The adapter therefore gives up ~128x too
##   early.  When it does, the xbar's write watchdog is suppressed by
##   BVALID alone rather than the B handshake, so an unread B parks
##   sw_owned[1] high permanently: S1 goes write-dead for every master, the
##   next JTAG write's AW is never arb-latched, and run_hw_axi blocks with
##   NO output at all (a slave that merely fails to answer prints "rd ...
##   FAILED" / BADA0BAD within ~10 s instead — so silence is the tell).
##   Only a bitstream reload clears it, since only core_rst does.
##   It needs the CPU to be holding S1 in a slow peripheral access at that
##   instant, which is why it is rare and load-dependent rather than tied
##   to any one command.  Fixing it is a SoC RTL job (widen TIMEOUT_CYCLES
##   — note the parameter AND counter are 20-bit, and the module is shared
##   with the CPU's own data master — plus bound the xbar watchdog
##   suppression).  Until then, the recovery below is the mitigation.
##
## IF THE REPL EVER DOES WEDGE (any command, not just this one):
##   The command loop is single-threaded, so a blocked command makes every
##   other command — including `vio-hard-reset` — unreachable.  You must
##   kill and relaunch.  RELAUNCH WITH JTAG_REPL_NO_PROGRAM=1: it re-attaches
##   to the running FPGA WITHOUT reprogramming it, so the boot state under
##   investigation SURVIVES.  A plain relaunch reprograms the device and
##   destroys the reproduction you were chasing — that has cost a real
##   investigation ~30 minutes and a lost repro.  Recipe:
##     pkill -x vivado
##     nohup bash -c 'exec 7>/tmp/jtag_in; sleep 99999' >/dev/null 2>&1 & disown
##     JTAG_REPL_NO_PROGRAM=1 vivado -mode tcl -nojournal -nolog \
##       -source tools/jtag_repl.tcl -tclargs <bit> <ltx> \
##       < /tmp/jtag_in > /tmp/jtag_out 2>&1 &
##   Startup prints "attaching without reprogramming" so you can confirm it.
##
## NUMERIC OPERANDS ARE HEX BY DEFAULT.
##   `r 40800000` reads 0x40800000.  Previously the dispatcher ran operands
##   through Tcl `expr`, which parsed a bare 8-hex-digit token as DECIMAL
##   (40800000 -> 0x026E8F80) — silently, and plausibly.  Explicit forms:
##   0x1234 / #1234 = hex, d1234 / 1234d = decimal.  A token that is not a
##   valid number in the selected base is an ERROR, never a silent 0.
##   Unaligned addresses are REJECTED rather than being silently aligned
##   down by the 32-bit JTAG-AXI master.
##   In scripts use `rdx <addr>` (returns an INTEGER, raises on a failed
##   read) rather than `rd` (returns a bare hex display string, and the
##   BADA0BAD failure sentinel as if it were data).
##   adb-key <keycode> [down|up|press]  — enqueue a synthetic ADB keyboard
##                                        event into adb_keyboard.v's FIFO
##                                        (keycode 0..0x7F; default `press`
##                                        = down+up pair).  The PIC firmware's
##                                        normal TALK-R0 polling delivers it
##                                        to Mac OS like a real keystroke.
##   adb-mouse <dx> <dy> [down|up]      — inject relative mouse motion (signed
##                                        deltas, report saturates ±63) and an
##                                        optional button transition into
##                                        adb_mouse.v's accumulator.
##   adb-status                         — keyboard-FIFO / mouse-pending flags;
##                                        count draining to 0 proves the ADB
##                                        poll loop is consuming events.
##   ── Unified reset (canonical surface, docs/reset_story.md) ──────────
##   reset                              — fire one cold-reset pulse via
##                                        DBG_CONTROL bit 5; CPU resumes
##                                        once SD→DDR copy completes.
##   reset hold                         — set DBG_CONTROL.cold_reset_hold,
##                                        fire pulse, leave CPU held in
##                                        reset post-cold-boot.  Use
##                                        before staging halt-after /
##                                        break_pc to avoid the race
##                                        window.
##   reset release                      — clear DBG_CONTROL.cold_reset_hold;
##                                        no pulse — CPU resumes from
##                                        wherever it was last left held.
##   reset hold-status                  — read live cold_reset_hold bit.
##   reset-and-halt-after <N> [wait_ms] — reset hold + arm halt-after-N
##                                        + release.  Default wait 200ms.
##   vio-reset-and-halt-after <N> [wait_ms]
##                                      — workaround path: CPU hold via
##                                        DBG_CONTROL bit 4, reset pulse via
##                                        VIO bit 3, stage halt-after after
##                                        reset clear, release hold.  VIO
##                                        bit 3 feeds the SAME unified pulse
##                                        as the canonical `reset` trigger
##                                        (see reset_and_halt_exc's comment
##                                        below) -- boot_fsm's SD->DDR ROM
##                                        copy re-runs exactly as it does
##                                        under `reset`.  Use `vio-hard-reset`
##                                        instead if a wedged JTAG-AXI bridge
##                                        or the RAM pre-zero pass is what's
##                                        actually needed.
##   vio-reset-halt-exc <vec> [wait_ms] — same VIO reset workaround, then
##                                        halt on first exception vector.
##                                        (Was broken with "[Designutils
##                                        20-1474] hw_probe VIO value [8] has
##                                        [1] value characters, required [2]":
##                                        probe_out0/vio_boot_ctrl went 4 bits →
##                                        5 bits at probe_map=v20, and Vivado
##                                        requires EXACTLY ceil(WIDTH/4) hex
##                                        chars.  vio_set now queries the live
##                                        probe WIDTH and zero-pads to it, so it
##                                        survives the next width bump too.
##                                        NOT the 1-bit-alias problem — the
##                                        right probe was always selected.)
##   sweep <wait_ms> <N1> <N2> ...      — per-N: reset hold + arm + release
##                                        + wait + snap.  One Vivado
##                                        session.
##   advance <N> [wait_ms]              — advance N retired instructions from the CURRENT halted state
##                                        (no reset).  Iterative bring-up: halt → inspect → advance N.
##                                        REQUIRES an effective halt (HALT_REASON
##                                        bit 3) AND a non-zero inst-count, and
##                                        REFUSES otherwise.  `advance` is
##                                        relative: it arms halt-after at
##                                        (inst-count + N), so if the preceding
##                                        halt had not landed, inst-count read 0
##                                        and the target became a count already
##                                        in the past — nothing ever halted
##                                        again while a bisect ladder kept
##                                        printing confident "good" rows.
##                                        Prints result=LANDED / NOT-LANDED:
##                                        it verifies afterwards that the halt
##                                        actually reached the target, and never
##                                        returns quietly having done nothing.
##   step                               — atomic single-step one retired μop using DBG_CONTROL bit 1
##   inst-count                         — read the captured retired-instruction count at last halt
##   ── Legacy aliases (deprecated; one-release window — see docs/reset_story.md)
##   reset-halt-after <N> [wait_ms]     — alias of reset-and-halt-after.
##   full-reset                         — alias of `reset`.
##   full-reset-and-halt [N]            — alias of reset-and-halt-after.
##                                        Both legacy commands print a
##                                        one-line deprecation notice.
##   ────────────────────────────────────────────────────────────────────
##   arch                          — dump host-write arch shadow regs (NOT live; use `live-arch`)
##   live-arch [force]             — dump live D0-D7/A0-A7/SR/VBR/A7/PC via snap chain.
##                                    Requires an effective halt.  `force` still
##                                    bypasses that check, but a forced read of a
##                                    RUNNING CPU relabels EVERY line —
##                                    "UNRELIABLE A7 ?= 0x..." instead of
##                                    "A7 = 0x..." — so a caller grepping for
##                                    `A7 = ` matches nothing rather than
##                                    harvesting a stale register.  The snap
##                                    chain is read register-by-register while
##                                    the pipeline retires, so forced values are
##                                    stale, torn, or both.
##   wedge-status                  — decode core/LSU/dcache live wedge probe regs
##   dcache-probe <set> <way> [word=0]
##                                  — inspect one D-cache tag/flags/data word.
##                                    Reads the cache array directly and does
##                                    NOT require a halt — use this when you
##                                    need cache state on a running CPU.
##   icache-probe <set> <way> [word=0]
##                                  — inspect one I-cache tag/valid/data word.
##                                    set 0..63, way 0..3, word 0..3 (word 0
##                                    is the LOWEST-addressed longword of the
##                                    16 B line).  Unlike dcache-probe this is
##                                    a REQUEST/ACK probe: the I-cache data
##                                    arrays are BRAM and cannot afford a
##                                    second read port, so the probe borrows
##                                    the fetch read path on an idle cycle.
##                                    It therefore needs an idle fetch cycle
##                                    to retire.  Halted is the reliable case;
##                                    on a RUNNING CPU it usually still
##                                    completes but can time out, and a
##                                    timeout RAISES rather than printing a
##                                    line — "no sample" is never reported as
##                                    "cache empty".
##   icache-lookup <addr>          — THE ONE YOU WANT.  Computes set/tag from
##                                    <addr>, probes all 4 ways, reports which
##                                    way HITS (or MISS), dumps that line's
##                                    16 bytes as four `> mem` lines in the
##                                    same format `dump-mem` uses, then reads
##                                    the same 16 bytes from DDR and prints a
##                                    per-word SAME/DIFFER verdict.  A DIFFER
##                                    means the CPU is fetching an instruction
##                                    stream that memory no longer holds —
##                                    stale I-cache over freshly-written code.
##                                    Neither half is authoritative alone:
##                                    remember `r`/`dump-mem` read DDR, so run
##                                    `dcache-op push` at a halt first if the
##                                    fresh bytes might still be sitting dirty
##                                    in the D-cache.
##   dcache-op <inv|push>          — D-cache invalidate-all / push-all.
##   icache-op [inv]               — I-cache invalidate-all.
##                                    BOTH REQUIRE AN EFFECTIVE HALT and refuse
##                                    otherwise. New m68k040 hardware additionally
##                                    reports REJECTED and writeback ERROR; legacy
##                                    debug hardware can still silently drop a
##                                    running request, so the host halt gate remains.
##                                    This matters because JTAG r/dump-mem read DDR
##                                    while the D-cache is write-back.
##                                    A run that ends done=0 now RAISES rather
##                                    than printing a status line — done=0 means
##                                    the cache was not maintained at all.
##   fault-snap                    — decode sticky-latched wedge state captured
##                                    at the first vec=2 fault since last clear.
##                                    Shows NOT LATCHED if no fault has fired.
##   fault-snap-clear              — clear the sticky latch + re-arm the trigger
##   rts-snap                      — decode sticky-latched finalize-LOAD
##                                    data/address snapshot (2026-07 "MOVEA.L
##                                    (SP),SP wild jump" investigation).
##                                    Captured at the first exception/IRQ-entry
##                                    take_finalize since last clear.  Shows
##                                    NOT LATCHED if none has fired.
##   rts-snap-clear                — clear the rts-snap sticky latch + re-arm
##   regs / reg-dump               — complete coherent halted architectural dump
##   arch-write <name> <hex>       — stage any writable architectural register
##   arch-apply                    — apply staged shadow regs into core (CPU must be halted)
##   reg-set <name> <hex>          — stage+apply one register; remains halted
##   halt-exc-mask                 — dump 256-bit halt-on-exc mask (8 lanes) + enable
##   halt-exc-mask <vec> [off]     — set/clear bit for exception vector 0..255
##   halt-kind                     — decode OFF_HALT_KIND (0x140): WHICH fatal
##                                    condition halted the core.  OFF_HALT_REASON
##                                    only says FATAL(4); this says which of
##                                    DCACHE_DIAG / FS_XLATE / RESET_VECTOR /
##                                    ARBITER_WEDGE / WALKER_PORT_WEDGE it was.
##   wedge-why                     — ⭐ READ THIS FIRST when the machine is dead
##                                    after a `reset`.  Decodes OFF_HALT_KIND +
##                                    OFF_STALL_ARB (0x102C, which DIRECTION and
##                                    which OWNER held the grant) + OFF_STALL_ABSORB
##                                    (0x1030, the two AXI reset absorbers) and
##                                    prints a VERDICT naming where to look next.
##                                    It separates the two cases a bare
##                                    ARBITER_WEDGE cannot: "the core could not
##                                    issue an address because an absorber is still
##                                    waiting for a response the fabric abandoned"
##                                    vs "an address DID go out and nothing came
##                                    back".  Those need opposite fixes.
##                                    ⚠ halt-kind reads 0 for ~10 s after a reset
##                                    even when wedged (the D20 bound must expire),
##                                    so re-read rather than concluding healthy.
##   pc-range-halt <lo> <hi>|off|status — cpu040 PC-RANGE halt lane: halt the
##                                    instant a macro RETIRES with its PC inside
##                                    [lo,hi].  Catches a wild jump into memory the
##                                    OS never allocated (the filler-execution boot
##                                    defect), whose landing address differs every
##                                    boot so break-pc cannot match it.  Try
##                                    `pc-range-halt 0x00800000 0x03ffffff`.
##                                    Reports reason 7 like a7-odd-halt; the halt
##                                    line shows PCRANGE pc0/pc1/pc2 + count.
##   a7-odd-halt <thresh>|off|status — cpu040 A7-ODD halt lane (OFF_A7ODD_CTL 0x10C):
##                                    halt with reason 7 (A7_ODD) once the COMMITTED
##                                    A7 has stayed odd for <thresh> retired macros;
##                                    the halt line then shows the PCs at the
##                                    EVEN->ODD edge (pc0 = retiring that cycle,
##                                    pc1/pc2 = the two before) + the odd A7 value.
##   halt-exc-mask raw <lane> <hex>  — write 32-bit lane (vec range lane*32..lane*32+31)
##       All three forms keep HALT_CTL bit 6 (halt_exc_enable) in sync with the
##       lanes.  The halt needs BOTH; writing only the mask arms a vector that
##       can never fire (silently — the exception just keeps counting).  #235.
##   pc-trace / last-pcs [<N>]     — dump committed macro PCs, newest window
##   branch-ring / last-branches [<N>]
##                                  — dump retired branches with outcome/next-PC
##   exc-ring / last-exceptions [<N>]
##                                  — dump completed (vec,pc,fault_addr,handler_pc)
##                                    entries from the exception ring + per-PC tally.
##                                    Use this — not the single exc_pc latch — to tell
##                                    if a storm is one PC looping vs. many PCs faulting.
##                                    `exc` and `excring` are accepted aliases.
##   dump-mem <addr> <words>       — read N consecutive 32-bit words
##   coherent-r <addr>             — halted D-push then physical word read
##   coherent-dump <addr> <words>  — halted D-push then physical burst read
##   coherent-w <addr> <value>     — push, physical write, invalidate D and I
##   watch <slot> <addr> [r|w|rw] [mask <amask>] [value <v> [lanes <m>]]
##                                  — arm data watchpoint slot 0/1 on a
##                                    PHYSICAL address (post-MMU compare
##                                    point).  amask bits = address don't-
##                                    care (0xFFF = 4 KiB page).  Halts at
##                                    the next retire boundary after the
##                                    access; report via `watch status`.
##                                    Config survives CPU reset — arm it,
##                                    then `reset`, to catch one-shot
##                                    early-boot writes.  Requires
##                                    OFF_FEATURES bit 13 (watchpoints).
##                                    A read-modify-write instruction
##                                    (bset/bclr/bchg/tas on memory) is two
##                                    bus transactions: `w` reports the
##                                    write-back once, `r` reports the read
##                                    half once, `rw` reports both.
##                                    VALUE FILTER LANES: `lanes` is a
##                                    wstrb-style mask over the 32-bit data
##                                    bus, bit 3 = the byte at the LOWEST
##                                    address of the longword.  EVERY armed
##                                    lane must be written by the access and
##                                    compare equal, so `value 0x12` with
##                                    the default lanes 0xF never matches a
##                                    byte store — position the byte in its
##                                    lane instead: byte 0x12 at addr A
##                                    is `value 0x12<<(8*(3-(A&3)))` with
##                                    `lanes [expr {1<<(3-(A&3))}]`, e.g.
##                                    `value 0x12000000 lanes 0x8` for a
##                                    longword-aligned byte.
##   watch <slot> off              — disarm slot + clear the hit latch
##   watch status                  — dump both slots + the latched hit
##                                    (slot, load/store, addr, data, PC)
##   atrap <slot> <value> [mask <m>] [d0 <v>]
##                                  — arm A-trap breakpoint slot 0/1 on an
##                                    A-line opcode word (opword[15:12] must
##                                    be 0xA — a decode-time hardware gate,
##                                    not just a mask bit).  <value>/<mask>
##                                    are 16-bit; default mask 0xFFFF (exact
##                                    match on <value>).  MASK BIT 1 = CARE
##                                    (compare this bit) — opposite polarity
##                                    from `watch`'s amask, where 1 = ignore.
##                                    `d0 <v>` enables the per-slot D0
##                                    qualifier: only report/halt when
##                                    architectural D0 == <v>.  Halts BEFORE
##                                    the trap's side effects, through the
##                                    same path as break-pc: HALT_REASON
##                                    bit 12 = atrap latched, HALT_HIT_PC ==
##                                    ATRAP_HIT_PC.  Requires OFF_FEATURES
##                                    bit 15 (atrap_bp); `d0` additionally
##                                    requires bit 17 (atrap_d0qual).
##   atrap <slot> off               — disarm slot, clear its skip-once bit,
##                                    and clear the hit latch ONLY if that
##                                    slot owns it (an unrelated slot's real
##                                    hit is never dropped).
##   atrap status                   — dump both slots (enable/value/mask/
##                                    match-set/d0-qual) plus the latched
##                                    hit (slot, opword, PC, A0, D0 — A0/D0
##                                    require OFF_FEATURES bit 16,
##                                    atrap_regcap; printed as explicitly
##                                    unsupported rather than a fabricated 0
##                                    when absent).
##   atrap list                     — print well-known Mac OS A-trap
##                                    selectors (_Read/_Write/_Control/...,
##                                    _SCSIDispatch, _SysError, etc.) plus
##                                    the family-mask idiom (value=0xA800
##                                    mask=0xFF00 = the whole 0xA8xx
##                                    toolbox-trap family).
##   sd-write <lba> <file>         — write file to SD card over JTAG, one
##                                    512-byte sector at a time, padding the
##                                    final short sector with zeros.
##   sd-write-fast <lba> <file> [noverify]
##                                  — FAST bulk SD write.  Requires the
##                                    DEDICATED provisioning bitstream
##                                    (make sd-provision-impl →
##                                    build/sd_provision/sd_provision_top.bit,
##                                    load it with load-bit first).  Streams
##                                    1 KiB AXI bursts into the 128 KiB
##                                    staging BRAM, then one CMD25 per batch;
##                                    each batch is CRC32-verified end-to-end
##                                    (CMD18 read-back) unless "noverify".
##                                    ~10 MiB in minutes vs many hours for
##                                    sd-write.  Tip: JTAG_TCK_HZ=30000000.
##   sd-fast-status                 — decode the provisioning bitstream's
##                                    bulk-writer STATUS/IDENT/CAPS regs.
##   eth-status                     — decode the optional SONIC/Taxi telemetry
##                                    page at 0x5098_0000 (requires a build
##                                    made with ETH_DEBUG_ENABLE=1).
##   eth-clear                      — clear its saturating counters and sticky
##                                    first-error snapshot.
##   eth-promisc [on|off]           — read or toggle the runtime SONIC RX
##                                    destination-filter bypass.
##   sonic-trace [dump [<path>]]    — dump the SONIC register-access trace
##                                    ring (rtl/soc/sonic_trace_ring.v) as a
##                                    CSV whose first four columns match the
##                                    53C96 dump shape, so the same differ
##                                    style (tools/scsi96_trace_diff.py) works
##                                    against a MAME capture of this driver.
##                                    NON-DESTRUCTIVE: reads a recording in
##                                    the SoC, never the live chip.  Before
##                                    freezing, `dump` samples wr_ptr AND the
##                                    suppressed-poll total twice 1.5 s apart
##                                    and SAYS which of three states it is in:
##                                    still advancing / spinning on a constant
##                                    register / not touching the SONIC at all.
##   sonic-trace status|freeze|rearm — ring state (incl. the live filtered
##                                    count) / manual freeze / re-arm
##
## ── vHDD: the DDR-backed RAM-disk SCSI volume ───────────────────────────────
## Control block at 0x5010_0000 (AXI-Lite, IDENT 0x5D0D0001) + a plain AXI
## data aperture whose base is READ FROM RD_APERTURE, never hardcoded, so a
## future rebase of the aperture cannot silently corrupt an upload.  Volume
## LBA L byte i lives at aperture + L*512 + i, ascending.  There is no
## doorbell and no descriptor ring — upload/download is an AXI block copy.
##
## OPERAND BASE: <MB>, <byte-count> and <byte-offset> below are DECIMAL by
## default (prefix 0x for hex) — the OPPOSITE of address operands like
## `r`/`w`/`dump-mem`, which are hex by default.  "ramdisk-size 32" means
## 32 MB, not 0x32.  Every command echoes the resolved value.
##
##   scsi-trace [dump [<path>]]     — dump the 53C96 register-access trace
##                                    ring (rtl/soc/scsi_trace_ring.v) as a
##                                    CSV comparable with the MAME golden
##                                    capture; diff with
##                                    tools/scsi96_trace_diff.py.  NON-
##                                    DESTRUCTIVE: reads a recording in the
##                                    SoC, never the live chip (a reg-2 read
##                                    would pop the FIFO, a reg-5 read would
##                                    clear the pending IRQ).  Before
##                                    freezing, `dump` samples wr_ptr twice
##                                    1.5 s apart and SAYS whether the ring
##                                    is actually quiescent — a dump of a
##                                    still-advancing ring is not a wedge.
##   scsi-trace status|freeze|rearm — ring state / manual freeze / re-arm
##   vhdd-status                    — IDENT check (says so LOUDLY if this
##                                    bitstream has no vHDD block — the whole
##                                    0x5010_0000 window is a null slave in
##                                    builds without it, so every register
##                                    would read a plausible 0), then a
##                                    decoded summary: SD volume enable +
##                                    block count/MB, RAM-disk enable + size/
##                                    max, busy/error/FSM state, watchdog-fire
##                                    count, and the aperture base.
##   vhdd-net [<our-mac> <our-ip> <our-port> <dst-mac> <dst-ip> <dst-port>]
##                                 — read or set the Ethernet-backed volume's
##                                   endpoint.  No ARP: both ends are
##                                   configured, and the destination is the
##                                   NEXT HOP.  our-MAC of all zeroes means
##                                   ABSENT, which is the power-on state.
##   vhdd-wprot [on|off]           — read or set SD volume write-protect
##                                  (CTRL[2]).  When on, the SCSI target
##                                  refuses WRITE(6)/WRITE(10) with CHECK
##                                  CONDITION / DATA PROTECT and the card
##                                  is never written.  Use it to freeze a
##                                  known-good disk image while debugging.
##   vhdd-enable <sd|ram|both> <0|1>
##                                  — read-modify-write CTRL bit0 (SD volume)
##                                    / bit1 (RAM disk).  Prints CTRL before
##                                    and after, and flags a write that did
##                                    not take.
##                                    PERSISTENCE: CTRL sits on core_rst, not
##                                    soc_full_rst, so an enable SURVIVES the
##                                    debug-full-reset / `reset` used to
##                                    reboot the Mac from JTAG -- enable the
##                                    RAM disk, then reboot, and the machine
##                                    boots seeing it.  A power cycle or a
##                                    real platform reset (btn / vio-hard-
##                                    reset) restores the default (SD only).
##   ramdisk-size <MB>              — set RD_BLOCKS = MB*2048, read back, and
##                                    report what actually took effect (the
##                                    hardware CLAMPS to RD_MAX_BLOCKS).
##                                    Resizing while Mac OS has the volume
##                                    MOUNTED will confuse the OS — unmount or
##                                    resize before boot.
##   ramdisk-clear [<MB>]           — zero the RAM disk through the aperture.
##                                    Default extent = the current RD_BLOCKS;
##                                    <MB> zeroes only the first N MB.
##                                    Progress every ~1 MB, plus wall time and
##                                    effective KiB/s, plus a read-back check
##                                    of the first/middle/last word.
##   ramdisk-load <file> [<byte-offset>]
##                                  — upload a host file into the aperture at
##                                    <byte-offset> (default 0).  REFUSES if
##                                    the file would run past RD_BLOCKS*512.
##                                    Unaligned offsets and non-multiple-of-4
##                                    lengths are handled by read-modify-write
##                                    so bytes outside the file's extent are
##                                    preserved exactly.  Sampled read-back
##                                    every ~1 MB aborts on a mismatch rather
##                                    than reporting a false success.
##   ramdisk-save <file> <byte-count> [<byte-offset>]
##                                  — download <byte-count> bytes from the
##                                    aperture into a host file, in STRICTLY
##                                    ASCENDING address order (unlike the
##                                    historical dump-mem beat scramble; the
##                                    read path re-checks interior burst beats
##                                    against single-word reads and downgrades
##                                    to per-word reads, loudly, if they
##                                    disagree).
##
## Bulk transfers use 256-beat INCR AXI bursts — the same shape sd-write-fast
## uses to reach ~264 KiB/s, so budget ~2 min for 32 MB.  Burst word order is
## MEASURED once per session against a saved-and-restored 64-word probe, never
## assumed; a burst never crosses a 4 KiB AXI boundary.  Tip, as for
## sd-write-fast: JTAG_TCK_HZ=30000000.
##
## Like `r`/`dump-mem`, these go straight to DDR over the JTAG-AXI master and
## do NOT participate in the CPU's D-cache.  That is the right thing for a
## volume the CPU reaches through SCSI rather than by address, but do not use
## them to inspect memory the CPU has dirty in L1D.
##   dump-frame-pgm <path> <base> <stride> <bpp> <w> <h>
##                                  — dump packed framebuffer to binary PGM
##   load-bit <bit> [<ltx>]        — re-program FPGA + (optionally) attach LTX probes
##                                    (auto re-checks build_id after program).
##   build_id | build-id           — read live OFF_BUILD_ID and compare against
##                                    build/vivado/fpga_top.buildinfo (no-op
##                                    when the buildinfo file is absent).
##   refresh                       — refresh_hw_device (re-sample probes etc.)
##   bp-stats                      — branch-predictor hit/miss telemetry:
##                                    retired branches, mispredicts, hit rate.
##                                    NOTE these count EVERY retired branch;
##                                    the separate OFF_MISPRED_COUNT register
##                                    counts RTS-class RAS returns only, which
##                                    is a different (much smaller) population.
##   vio-set <hex>                 — direct VIO probe write (power user)
##   vio-read [filter]             — dump VIO INPUT probes (live signal values)
##   video-status                  — decode the coherent scan-out snapshot
##                                   (task #243): committed hres/vres/base/
##                                   stride, dafb_live (boot-splash
##                                   substitution gate), rd_en/valid/ready,
##                                   and the underflow sticky SPLIT into its
##                                   line-buffer and fb_reader halves.  All
##                                   fields come from ONE pclk edge, so they
##                                   can be correlated -- unlike reading
##                                   video_debug_hcount/rgb as separate
##                                   probes, which are sampled seconds apart.
##                                   ALSO answers "why is the screen black"
##                                   without reading RTL: the placement
##                                   admission verdict (reject_reason), the
##                                   sticky "rejected since the last commit"
##                                   bit, the tuple that was refused, and
##                                   fb_reader's req/rsp/miss counters whose
##                                   (req - rsp) is the live outstanding-
##                                   response count.  See
##                                   docs/video_path_review.md S4.2.
##   vio-hard-reset [hold_ms]      — VIO equivalent of a physical btn[3] press
##                                    (platform_reset_req/platform_resetn only,
##                                    NOT the raw fabric_gt_clr/BUFG_GT .CLR --
##                                    see fpga_top_clocks.vh). Use when the
##                                    CSR-based `reset` command has left the
##                                    JTAG-AXI bridge unresponsive; default
##                                    hold=50ms.  Unlike `reset` /
##                                    `vio-reset-and-halt-after` (both "warm":
##                                    boot_fsm re-runs the SD->DDR ROM copy
##                                    but the 256 MiB RAM pre-zero pass is
##                                    skipped, per fpga_top_boot_master.vh
##                                    boot_warm_q), this is a TRUE platform
##                                    reset -- it also clears boot_warm_q, so
##                                    the RAM zero pass runs too.  This is
##                                    the most complete VIO-driven reset:
##                                    genuine fresh-DRAM cold boot AND the
##                                    one proven to recover a wedged
##                                    JTAG-AXI bridge that `reset` itself
##                                    can't reach.
##   pram-save                     — save the LIVE 256-byte PRAM to the
##                                    reserved SD sector (LBA 8191, the last
##                                    sector of the 4 MiB ROM window; sectors
##                                    8176..8191 are the reserved system-
##                                    persistence block).  MANUAL ONLY —
##                                    nothing in the hardware ever does this
##                                    by itself, and the Mac cannot reach the
##                                    register file that starts it.  Warns if
##                                    the CPU is not halted (see pram-load).
##   pram-load                     — restore PRAM from that sector.  A sector
##                                    that fails magic/version/length/checksum,
##                                    is blank, or cannot be read is NOT
##                                    installed: PRAM is put back on rtc.v's
##                                    own post-reset defaults (the same image
##                                    `pram-clear` produces) and the reason is
##                                    printed.  The fallback is never silent.
##                                    Halt the CPU first if the Mac is doing
##                                    disk I/O: the RTL will not interrupt an
##                                    SD transfer already in flight, but a
##                                    SCSI command started DURING this one
##                                    stalls ~10 s on sd_ctrl's watchdog.
##   pram-dump                     — hex dump the live 256 PRAM bytes.  Reads
##                                    the RTC array only; touches no SD sector.
##   pram-clear [hold_ms]          — zap the RTC PRAM (Cmd-Opt-P-R equivalent).
##                                    PRAM is battery-backed: it deliberately
##                                    SURVIVES reset (and debug-full-reset), so
##                                    this is the only software-reachable way
##                                    back to the power-on image if a bad PRAM
##                                    image wedges the ROM boot.  Pulses
##                                    vio_boot_ctrl[4] 0->1->0 (RTL edge-detects
##                                    it into a one-shot; holding the bit does
##                                    NOT hold PRAM clear).  Other probe_out0
##                                    bits are preserved.  Needs a
##                                    probe_map=v20+ bitstream; default
##                                    hold=50ms.
##   q | quit | exit               — exit REPL
##
## Output is line-oriented and prefixed with `>` for results so stdin parsers
## can find them easily:
##   > r 0x50900028 = 0x408046AA
##   > halt: reason=0x18 hit=0x408046AA exc_vec=0x02 exc_pc=0x408046AA exc_count=0x16
##   > sweep_step N=1000 pc=0x4084aa08 hit=0x4084a96a reason=0x1a exc=2@0x408046aa
##   > READY        (after each command completes — synchronous handshake)
##   > ERROR <message>

# ── Init ─────────────────────────────────────────────────────────────────
set ::jtag_repl_library_only [expr {[info exists ::env(JTAG_REPL_LIBRARY_ONLY)] &&
                                    $::env(JTAG_REPL_LIBRARY_ONLY) eq "1"}]
if {!$::jtag_repl_library_only && [llength $::argv] < 1} {
    puts "ERROR: usage: jtag_repl.tcl <bit> \[<ltx>\]"
    exit 1
}
set BIT [expr {[llength $::argv] > 0 ? [lindex $::argv 0] : ""}]
set LTX [expr {[llength $::argv] > 1 ? [lindex $::argv 1] : ""}]

set DBG_BASE          0x50900000
set SDJ_BASE          0x50A00000
set SDJ_OFF_LBA       0x000
set SDJ_OFF_CTRL      0x004
set SDJ_OFF_STATUS    0x008
set SDJ_OFF_BUFPTR    0x00C
set SDJ_OFF_BUFDATA   0x010
set SDJ_OFF_IDENT     0x014
set SDJ_CTRL_GO       0x5D000001
set SDJ_IDENT         0x5D7A0001

# sd_bulk_writer (DEDICATED provisioning bitstream, rtl/soc/sd_provision_top.v).
# The bulk writer is the ONLY AXI slave in that bitstream; registers sit at
# absolute address 0x0, the staging BRAM window at +0x100000 (addr bit 20).
set SDP_REG_BASE      0x00000000
set SDP_STG_BASE      0x00100000
set SDP_OFF_LBA       0x000
set SDP_OFF_CTRL      0x004
set SDP_OFF_STATUS    0x008
set SDP_OFF_BLKCNT    0x00C
set SDP_OFF_CRC32     0x010
set SDP_OFF_IDENT     0x014
set SDP_OFF_CAPS      0x018
set SDP_CTRL_GO_WRITE  0x5DB00001
set SDP_CTRL_GO_VERIFY 0x5DB00002
set SDP_IDENT          0x5DB70001
# Burst word-order calibration result: "" (unknown), fwd (first word in the
# -data hex string lands at the lowest address) or rev (last word does).
set ::sdp_word_order  ""

# ── vHDD: the DDR-backed RAM-disk SCSI volume ───────────────────────────────
# Control block: AXI-Lite, reusing the previously-dead "DMA config" window.
# Everything else in 0x5010_0000..0x501F_FFFF is a NULL SLAVE (reads 0,
# swallows writes) so a stray access completes instead of wedging the fabric
# -- which is exactly why IDENT must be checked before believing any of the
# other registers: on a bitstream without the block they all read 0, and 0 is
# a legal-looking value for every one of them.
set VHDD_BASE          0x50100000
set VHDD_OFF_IDENT     0x000   ;# RO  0x5D0D0001
set VHDD_OFF_CTRL      0x004   ;# RW  bit0 = SD vol enable, bit1 = RAM-disk enable
set VHDD_OFF_RD_BLOCKS 0x008   ;# RW  RAM-disk size in 512-byte blocks (clamped to RD_MAX)
set VHDD_OFF_SD_BLOCKS 0x00C   ;# RO  SD volume block count as the platform computed it
set VHDD_OFF_STATUS    0x010   ;# RO  [0]sd_en [1]ram_en [2]busy [3]err [7:4]fsm [8]wprot [31:16]wdog
set VHDD_OFF_RD_MAX    0x014   ;# RO  compile-time max block count
set VHDD_OFF_RD_APER   0x018   ;# RO  AXI base of the RAM disk's data aperture
set VHDD_IDENT         0x5D0D0001
set ETH_DEBUG_BASE     0x50980000
set ETH_DEBUG_IDENT    0x45544801
set VHDD_CTRL_SD_EN    0x1
set VHDD_CTRL_SD_WPROT 0x4   ;# CTRL[2] — SD volume write-protect
set VHDD_CTRL_RAM_EN   0x2
# Burst word-order calibration for the DATA APERTURE (independent of
# ::sdp_word_order, which is calibrated against the provisioning bitstream's
# staging BRAM and is meaningless here): "" unknown, fwd, rev.
set ::vhdd_wr_order    ""
# Set to 1 once burst READS have been disproven for this session -- from then
# on the aperture is read one word at a time rather than trusting a burst
# whose beat order we could not confirm.
set ::vhdd_rd_slow     0

set OFF_VERSION       0x000
set OFF_BUILD_ID      0x004
set OFF_CONTROL       0x008
set OFF_STATUS        0x00C
# Capability / discovery block (debug_ctrl >= 0xDEB6_0006).  Older
# bitstreams return 0 here (unmapped offsets read 0 with an OKAY
# response), which the `dbg-caps` command below reports as "feature
# discovery not supported" rather than as "no features".
set OFF_FEATURES      0x0A0
set OFF_DBG_RESET_CTL 0x0A4
set OFF_CAP_TRACE     0x0A8
set OFF_CAP_TRACE2    0x0AC
set DRC_CFG_WIPE      0x1
set DRC_COUNT_CLEAR   0x2
# OFF_FEATURES bit names, in bit order.  Append-only — never renumber.
set CORE040_DEBUG_VERSION 0xDEB60100
set DBG_FEATURE_NAMES_LEGACY {
    dbg_reset_domain axi_ready_gated cfg_wipe cpu_reset_count
    pc_trace exc_ring break_pc_multi halt_exc_mask
    fault_snap rts_snap live_arch dcache_probe
    perf_counters watchpoints trace_trigger
    atrap_bp atrap_regcap atrap_d0qual
    mon_sense icache_probe
}
set DBG_FEATURE_NAMES_CORE040 {
    dbg_reset_domain axi_ready_gated cfg_wipe cpu_reset_count
    pc_trace exc_ring break_pc_multi halt_exc_mask
    fault_snap rts_snap live_arch dcache_probe
    perf_counters watchpoints trace_trigger
    atrap_bp atrap_regcap atrap_d0qual
    mon_sense arch_apply_stays_halted arch_dirty_apply cache_maint_only
    macro_retire_count stop_status_v2 branch_ring
}
set OFF_PC            0x010
set OFF_LAST_PC       0x014
set OFF_IRQ_INJECT    0x020
set OFF_EXC_VEC       0x024
set OFF_EXC_PC        0x028
set OFF_RESET_CAUSE   0x02C
set OFF_HALT_AFTER_LO   0x030
set OFF_HALT_AFTER_HI   0x034
set OFF_BREAK_PC        0x038
set OFF_HALT_CTL        0x03C
set OFF_HALT_REASON     0x040
set OFF_HALT_HIT_PC     0x044
set OFF_HALT_HIT_INST_LO 0x048
set OFF_HALT_HIT_INST_HI 0x04C
set OFF_HALT_EXC_VEC    0x050
set OFF_RAM_WINDOW_LG2  0x058   ;# DDR RAM-window size select, lg2 [22..30]
# DAFB monitor-sense code, 7 bits (debug_ctrl >= 0xDEB6_0008, OFF_FEATURES
# bit 18 = mon_sense).  bit 6 = "extended monitor" flag, bits[5:0] = code.
# Lives in the debug reset domain, so a written value survives a CPU reset.
set OFF_MON_SENSE       0x05C
set OFF_HALT_KIND       0x140   ;# cpu040 fatal-halt attribution (socket HaltReason)
set OFF_STALL_ARB       0x102C  ;# cpu040 AxiDMerge post-mortem: direction + owner at a wedge
set OFF_STALL_ABSORB    0x1030  ;# cpu040 AXI read-reset absorbers, {axi_i[31:16], axi_d[15:0]}
set OFF_PCRANGE_CTL     0x124   ;# cpu040 PC-RANGE halt lane: bit0 enable
set OFF_PCRANGE_LO      0x128
set OFF_PCRANGE_HI      0x12C
set OFF_PCRANGE_PC0     0x130
set OFF_PCRANGE_PC1     0x134
set OFF_PCRANGE_PC2     0x138
set OFF_PCRANGE_COUNT   0x13C
set OFF_A7ODD_CTL       0x10C   ;# cpu040 A7-ODD halt lane: bit0 enable, [31:16] threshold
set OFF_A7ODD_PC0       0x110
set OFF_A7ODD_PC1       0x114
set OFF_A7ODD_PC2       0x118
set OFF_A7ODD_VALUE     0x11C
set OFF_A7ODD_COUNT     0x120
set OFF_HALT_EXC_MASK   0x060   ;# 8 × 32-bit lanes, bit (lane*32+bit) = vec
set OFF_BP_SKIP_ONCE    0x080
set OFF_BREAK_PC1       0x084
set OFF_BREAK_PC2       0x088
set OFF_BREAK_PC3       0x08C
set OFF_BREAK_PC_CTRL   0x090
# Phase-3 data watchpoints (debug_ctrl >= 0xDEB6_0006 with OFF_FEATURES
# bit 13).  WPn_CTRL: bit0 enable, bit1 loads, bit2 stores, bit3 value
# compare, bits[15:8] value byte-lane mask (wstrb-style, bit 3+8 = the
# lowest-addressed byte of the longword).  Addresses are PHYSICAL (the
# compare point is post-MMU — the same address space `r`/`w` use).
set OFF_WP0_ADDR        0x0B0
set OFF_WP0_AMASK       0x0B4
set OFF_WP0_VALUE       0x0B8
set OFF_WP0_CTRL        0x0BC
set OFF_WP1_ADDR        0x0C0
set OFF_WP1_AMASK       0x0C4
set OFF_WP1_VALUE       0x0C8
set OFF_WP1_CTRL        0x0CC
set OFF_WP_HIT          0x0D0   ;# [0]=valid (W1C), [1]=slot, [2]=store, [15:12]=wstrb
set OFF_WP_HIT_ADDR     0x0D4
set OFF_WP_HIT_DATA     0x0D8
set OFF_WP_HIT_PC       0x0DC

# A-trap breakpoints (OFF_FEATURES bit 15 = atrap_bp, bit 16 = atrap_regcap
# for HIT_A0/HIT_D0, bit 17 = atrap_d0qual for the per-slot D0 qualifier).
# Two independent slots, each halting BEFORE the trapped instruction's side
# effects through the same halt path as break-pc (HALT_REASON bit 12 =
# atrap latched, HALT_HIT_PC == ATRAP_HIT_PC).  CTRL/MATCH/D0VAL are packed
# per-slot with a fixed 0xC stride, same layout idiom as WP0/WP1 above.
# MATCH packs {mask[15:0] in bits[31:16], value[15:0] in bits[15:0]} so
# arming is atomic; MASK BIT 1 = CARE (compare this bit) -- the OPPOSITE
# polarity from WPn_AMASK (1 = ignore there). Match rule: opword[15:12]==0xA
# AND ((opword ^ value) & mask) == 0.
set OFF_ATRAP0_CTRL      0x0E0
set OFF_ATRAP0_MATCH     0x0E4
set OFF_ATRAP0_D0VAL     0x0E8
set OFF_ATRAP1_CTRL      0x0EC
set OFF_ATRAP1_MATCH     0x0F0
set OFF_ATRAP1_D0VAL     0x0F4
set OFF_ATRAP_SKIP_ONCE  0x0F8   ;# bits[1:0] per-slot skip-once (diagnostic; HW auto-arms it)
set OFF_ATRAP_HIT        0x0FC   ;# [0]=hit_valid(W1C) [1]=slot [2]=capture_busy [31:16]=opword
set OFF_ATRAP_HIT_PC     0x100
set OFF_ATRAP_HIT_A0     0x104
set OFF_ATRAP_HIT_D0     0x108

# Task #33: double-fault halt state (captured at the dbl_fault edge).
set OFF_DBL_FAULT_PC    0x094
set OFF_DBL_FAULT_VEC   0x098   ;# [8]=latched, [7:0]=vector
set OFF_PC_MISALIGNED_PC 0x09C
set OFF_DCACHE_PROBE_SEL   0x200
set OFF_DCACHE_PROBE_TAG   0x204
set OFF_DCACHE_PROBE_FLAGS 0x208
set OFF_DCACHE_PROBE_DATA  0x20C
set OFF_DCACHE_OP          0x210
set OFF_ICACHE_OP          0x214
# I-cache probe (OFF_FEATURES bit 19 = icache_probe).  Field widths differ
# from the D-cache probe's because the geometries differ -- the I-cache is
# 64 sets x 16 B (4 words/line, 6-bit set), the D-cache 32 sets x 32 B
# (8 words/line, 5-bit set).  Do NOT copy the D-cache packing.
#   SEL   [5:0]=set  [7:6]=way  [9:8]=word   -- WRITING IT LAUNCHES A PROBE
#   FLAGS [0]=line valid  [1]=sample landed  [2]=probe still outstanding
#   TAG   [21:0] = addr[31:10] of the cached line
#   DATA  32-bit word `word` of the line (word 0 = LOWEST address)
set OFF_ICACHE_PROBE_SEL   0x218
set OFF_ICACHE_PROBE_TAG   0x21C
set OFF_ICACHE_PROBE_FLAGS 0x220
set OFF_ICACHE_PROBE_DATA  0x224
# I-cache geometry, from cpu/rtl/core/fetch/icache.v (4 KB, 4-way, 16 B
# lines -> 64 sets).  set = addr[9:4], tag = addr[31:10].
set IC_LINE_BYTES  16
set IC_NUM_SETS    64
set IC_NUM_WAYS    4
set OFF_EXC_COUNT     0x1018

# ---- windowed performance counters (core commit: feat/perf-counters) ----------
# The owner's request: "do we have any perf counters available? i would like a way
# to zero 'em, too, such that i can capture a precise path."
#
# Everything else in the 0x1000 block is free-running and read-only, so a window had
# to be a difference of two reads -- and OFF_CYCLE_LO/HI and OFF_INST_LO/HI can TEAR
# across a LO wrap, so even the difference is untrustworthy.  These can be ZEROED and
# FROZEN, which makes the 64-bit LO/HI pairs read atomically.
#
# OFF_MISPRED_COUNT and OFF_FLUSH_COUNT are the SAME offsets that have read zero since
# Stage 1 ("zero until a real producer exists").  They now have real producers and are
# part of this windowed set.  A bitstream OLDER than the core commit that added them
# still reads zero here -- which is why `perf` checks OFF_PERF_CTL first and REFUSES
# rather than printing a table of zeros.  A confident table of zeros is exactly the
# failure mode that made `wedge-status` refuse outright.
# The FREE-RUNNING telemetry pair, for the sanity line at the bottom of `perf`.
# These were NEVER DEFINED in this file before: the REPL talked about OFF_CYCLE_LO in a
# comment (as the canonical example of a declared-but-unimplemented register) and read
# the live retired-macro count by its raw absolute address 0x50901008, but no symbol
# existed.  `perf` referencing `$::OFF_INST_LO` would therefore have thrown
# `can't read "::OFF_INST_LO": no such variable` the first time it ran on the board.
# Caught by running the command against a modelled register block instead of assuming
# it worked -- which is the same discipline the RTL side of this change is held to.
set OFF_CYCLE_LO          0x1000
set OFF_CYCLE_HI          0x1004
set OFF_INST_LO           0x1008
set OFF_INST_HI           0x100C
set OFF_MISPRED_COUNT     0x1010
set OFF_FLUSH_COUNT       0x1014
set OFF_PERF_CTL          0x1080
set OFF_PERF_CYCLE_LO     0x1084
set OFF_PERF_CYCLE_HI     0x1088
set OFF_PERF_INST_LO      0x108C
set OFF_PERF_INST_HI      0x1090
set OFF_PERF_BRANCH       0x1094
set OFF_PERF_DC_MISS      0x1098
set OFF_PERF_IC_MISS      0x109C
set OFF_PERF_DTLB_WALK    0x10A0
set OFF_PERF_ITLB_WALK    0x10A4
set OFF_PERF_STALL_RETIRE 0x10A8
set OFF_PERF_STALL_DC     0x10AC
set OFF_PERF_STALL_WALK   0x10B0
# OFF_PERF_CTL write encodings.  Byte 0 carries both controls, so one write sets both.
set PERF_CTL_CLEAR_RUN 0x3
set PERF_CTL_FREEZE    0x0
set PERF_CTL_RUN       0x2
set PERF_CTL_CLEAR_HOLD 0x1
set OFF_PC_TRACE_BASE 0x10000
set OFF_PC_TRACE_HEAD 0x11000
# FALLBACK ONLY.  The real depth is a BUILD-TIME parameter -- see
# m68k_axi_wrapper.v's .PC_TRACE_DEPTH() override (currently 64; note
# debug_ctrl.v's own default is 256, so the two disagree by design).  The
# pc-trace command reads the actual value from OFF_CAP_TRACE and only falls
# back to this constant on bitstreams predating that capability register.
# Do NOT reason about ring depth from this line -- read OFF_CAP_TRACE.
set PC_TRACE_DEPTH    64       ;# fallback if OFF_CAP_TRACE is unavailable
set PC_TRACE_MASK     0x3F     ;# unused by pc-trace; kept for old callers

# Exception ring buffer — 32 entries × 16 bytes at 0x12000..0x121FC.
# Per entry: +0 vec, +4 pc, +8 fault_addr, +12 exc_count_at_event.
# HEAD at 0x13000 (next-slot-to-write).
set OFF_EXC_RING_BASE 0x12000
set OFF_EXC_RING_HEAD 0x13000
set EXC_RING_DEPTH    32
set EXC_RING_MASK     0x1F
set OFF_BRANCH_RING_BASE 0x14000
set OFF_BRANCH_RING_HEAD 0x15000
set BRANCH_RING_DEPTH    32
set BRANCH_RING_MASK     0x1F
set OFF_ARCH_D0       0x2000   ;# host-write shadow (NOT live CPU state)
set OFF_ARCH_A0       0x2020
set OFF_ARCH_USP      0x2040
set OFF_ARCH_SSP      0x2044   ;# master supervisor stack bank (MSP)
set OFF_ARCH_ISP      0x2048
set OFF_ARCH_SR       0x204C
set OFF_ARCH_VBR      0x2050
set OFF_ARCH_CACR     0x2054
set OFF_ARCH_TC       0x2058
set OFF_ARCH_ITT0     0x205C
set OFF_ARCH_ITT1     0x2060
set OFF_ARCH_DTT0     0x2064
set OFF_ARCH_DTT1     0x2068
set OFF_ARCH_URP      0x206C
set OFF_ARCH_SRP      0x2070
set OFF_ARCH_PC       0x2074
set OFF_ARCH_APPLY    0x2078
set OFF_ARCH_STATUS   0x207C
set OFF_ARCH_SFC      0x2080
set OFF_ARCH_DFC      0x2084
set OFF_LIVE_VBR      0x2100   ;# live arch readback (snap path → cRAT → PRF)
set OFF_LIVE_SR       0x2104
set OFF_LIVE_A7       0x2108
set OFF_LIVE_USP      0x210C
set OFF_LIVE_D0       0x2110   ;# +0x4 per reg, D0..D7
set OFF_LIVE_A0       0x2130   ;# +0x4 per reg, A0..A7
set OFF_LIVE_MMU_TC   0x2160   ;# live MMU CSR readback (CPU goes through MMU;
set OFF_LIVE_MMU_DTT0 0x2164   ;# JTAG-AXI does not — these expose what the
set OFF_LIVE_MMU_DTT1 0x2168   ;# CPU's translation pipeline sees).
set OFF_LIVE_MMU_ITT0 0x216C
set OFF_LIVE_MMU_ITT1 0x2170
set OFF_LIVE_MMU_SRP  0x2174
set OFF_LIVE_MMU_URP  0x2178
set OFF_LIVE_SSP      0x217C   ;# master supervisor stack bank (MSP)
set OFF_LIVE_ISP      0x2180
set OFF_LIVE_CACR     0x2184
set OFF_LIVE_SFC      0x2188
set OFF_LIVE_DFC      0x218C
set OFF_LIVE_PC       0x2190
set OFF_LIVE_MMUSR    0x2194
set OFF_WEDGE0        0x3000
set OFF_WEDGE1        0x3004
set OFF_WEDGE2        0x3008
set OFF_WEDGE3        0x300C
set OFF_FAULT_SNAP_VALID 0x3010
set OFF_FAULT_SNAP_W0    0x3014
set OFF_FAULT_SNAP_W1    0x3018
set OFF_FAULT_SNAP_W2    0x301C
set OFF_FAULT_SNAP_W3    0x3020
set OFF_FAULT_SNAP_CLEAR 0x3024
set OFF_RTS_SNAP_VALID 0x3028
set OFF_RTS_SNAP_W0    0x302C
set OFF_RTS_SNAP_W1    0x3030
set OFF_RTS_SNAP_W2    0x3034
set OFF_RTS_SNAP_W3    0x3038
set OFF_RTS_SNAP_W4    0x303C
set OFF_RTS_SNAP_CLEAR 0x3040

# ── ADB event injection (rtl/mac/adb_inject.v @ system 0x5001_1000) ──────
# Uses adb_inject.v's word-aligned alias register block (offsets
# 0x10..0x1C): the JTAG-AXI master issues only full-strobe word
# transactions, so the byte-granular offsets 0x00..0x05 are unreachable
# from here.  peripheral_bus delivers exactly one byte per word write
# (the wdata LSB) to the device models' injection queue — see the
# adb_inject.v register map and tb_peripheral_bus::test_adbinj_decode.
set ADBINJ_BASE     0x50011000
set ADBINJ_OFF_KBD  0x10   ;# W keycode (bit7 = release)   R kbd FIFO status
set ADBINJ_OFF_BTN  0x14   ;# W button state (bit0 = down) R mouse status
set ADBINJ_OFF_DX   0x18   ;# W signed 8-bit X delta (report saturates ±63)
set ADBINJ_OFF_DY   0x1C   ;# W signed 8-bit Y delta (report saturates ±63)

set CTL_HALT_REQ          0x01
set CTL_STEP_PULSE        0x02
set CTL_SOFT_RST          0x04   ;# DEPRECATED legacy — alias of CTL_COLD_RESET_PULSE
set CTL_INIT_DONE_OVR     0x08
set CTL_COLD_RESET_HOLD   0x10   ;# bit 4 — sticky CPU hold across unified reset
set CTL_COLD_RESET_PULSE  0x20   ;# bit 5 — canonical unified-reset trigger
set CTL_STEP_MACRO        0x80
set HALT_AFTER_EN  0x1
set HALT_BREAK_PC_EN 0x2
set HALT_CLEAR     0x4
set HALT_EXC_EN    0x40
set ARCH_APPLY_START         0x1
set ARCH_APPLY_CLEAR_STATUS  0x2
set ARCH_STATUS_BUSY         0x1
set ARCH_STATUS_DONE         0x2
set ARCH_STATUS_REJECTED     0x4
set ::last_requested_break_pc ""

# ── Hardware setup (once) ────────────────────────────────────────────────
if {!$::jtag_repl_library_only} {
puts "> jtag_repl: opening hw_manager"
open_hw_manager
connect_hw_server -url localhost:3121 -allow_non_jtag
set ::hw_tgt [lindex [get_hw_targets] 0]
# Bump TCK frequency on the JTAG target before opening it.  Most Xilinx
# USB-JTAG cables (Digilent / FT232H / SmartLynq) accept up to 30 MHz;
# SmartLynq Plus goes to 60 MHz.  Override at runtime with the env var
# JTAG_TCK_HZ if your cable can't sustain 30 MHz.
set ::jtag_tck_hz 15000000
if {[info exists ::env(JTAG_TCK_HZ)] && $::env(JTAG_TCK_HZ) ne ""} {
    set ::jtag_tck_hz [expr {int($::env(JTAG_TCK_HZ))}]
}
if {[catch {
    set_property PARAM.FREQUENCY $::jtag_tck_hz $::hw_tgt
    puts "> jtag_repl: set TCK=${::jtag_tck_hz} Hz on target"
} freq_err]} {
    puts "> jtag_repl: WARNING — could not set TCK to ${::jtag_tck_hz} Hz: $freq_err"
}
current_hw_target $::hw_tgt
open_hw_target
set ::hw_dev [lindex [get_hw_devices] 0]
current_hw_device $::hw_dev
puts "> jtag_repl: device = $::hw_dev"
if {[info exists ::env(JTAG_REPL_NO_PROGRAM)] && $::env(JTAG_REPL_NO_PROGRAM) eq "1"} {
    if {$LTX ne ""} {
        set_property PROBES.FILE $LTX $::hw_dev
    }
    refresh_hw_device $::hw_dev
    puts "> jtag_repl: WARNING: attaching without reprogramming (JTAG_REPL_NO_PROGRAM=1)"
} elseif {$BIT ne ""} {
    # Program before attaching probe metadata or refreshing.  The currently
    # loaded design may not match the new LTX, and refreshing that combination
    # can wedge Vivado in a debug-core/MIG refresh callback.
    set_property PROGRAM.FILE $BIT $::hw_dev
    program_hw_devices $::hw_dev
    if {$LTX ne ""} {
        set_property PROBES.FILE $LTX $::hw_dev
    }
    refresh_hw_device $::hw_dev
    puts "> jtag_repl: programmed $BIT"
} else {
    if {$LTX ne ""} {
        set_property PROBES.FILE $LTX $::hw_dev
    }
    refresh_hw_device $::hw_dev
}
set ::axi [lindex [get_hw_axis] 0]
puts "> jtag_repl: axi master = $::axi"
}

# ── ONE MASTER, TWO BUSSES (2026-09-12) ────────────────────────────────
# There is exactly ONE jtag_axi core, and there must stay exactly one.  A
# 2026-09-07 experiment instantiated a SECOND one to own the 0x5090_0000
# debug window; it was reverted (330ad925) because two BSCAN cores
# destabilised debug-hub enumeration -- `get_hw_axis` intermittently
# reported 1 core instead of 2 and the device dropped off the chain,
# producing two confidently wrong conclusions from misread zeros.
#
# The separation is now INSIDE the SoC instead: the single bridge lands on
# `axi_dbg_bus` (rtl/soc/axi_dbg_bus.v), which serves the 0x5090_0000 debug
# window from local core_clk slaves and masters the SoC crossbar for
# everything else.  So debug-register access no longer traverses the
# crossbar, the peripheral bus or the pb_clk CDC, and is not in the SoC
# reset domain.
#
# CAVEAT, still true and load-bearing: the Vivado jtag_axi master IP stalls
# on its OWN outstanding transaction and issues nothing further
# (`run_hw_axi` blocks -- see the note near the top of this file).  That is
# upstream of the SoC and no in-SoC topology can fix it.  So a `r
# 0x50F0F040` into a stalled peripheral can still block the next command,
# even though the debug window itself is now unblockable.  What changed is
# that a wedged peripheral no longer makes debug access IMPOSSIBLE -- only
# a transaction already parked in the master does, and that clears when the
# system transaction retires.
set ::axi_dbg $::axi
if {[llength [get_hw_axis]] > 1} {
    puts "> jtag_repl: WARNING: [llength [get_hw_axis]] hw_axi cores found."
    puts "> jtag_repl:  This bitstream predates 330ad925 (dual-BSCAN revert)."
    puts "> jtag_repl:  Using core 0; expect flaky enumeration."
} else {
    puts "> jtag_repl: single hw_axi core (correct) -- debug window is served"
    puts "> jtag_repl:  off the SoC bus by axi_dbg_bus, not through S1"
}

# Read OFF_BUILD_ID (debug_ctrl base + 0x4) and compare against the canonical
# `build_id=0x...` line in build/vivado/fpga_top.buildinfo.  Returns the live
# build_id string (e.g. "0x3B701463") or "" on error.  When `quiet` is 0,
# prints the live value and a WARNING on mismatch.
#
# Two callers:
#   - startup, right after AXI master attach
#   - `load-bit`, after re-programming the device (the new bitstream may
#     report a different build_id, so re-check)
#   - explicit `build_id` REPL command, for users debugging stale-bitstream
#     attaches at any point during a session.
proc check_build_id {{quiet 0}} {
    if {[catch {
        set build_id_hex [dbg_rd 0x4]
        scan $build_id_hex %x build_id_value
        set build_id [format "0x%08X" $build_id_value]
        if {!$quiet} {
            puts "> jtag_repl: build_id = $build_id"
        }

        # Prefer the manifest beside the bitstream actually loaded. Custom
        # VIVADO_IMPL_DIR builds are routine, and comparing them only against
        # build/vivado/fpga_top.buildinfo creates a convincing false stale-build
        # warning. Keep the historical path as a fallback for attach-only use.
        set buildinfo_file "build/vivado/fpga_top.buildinfo"
        if {[info exists ::BIT] && $::BIT ne ""} {
            set adjacent [file join [file dirname [file normalize $::BIT]] fpga_top.buildinfo]
            if {[file exists $adjacent]} { set buildinfo_file $adjacent }
        }
        if {[file exists $buildinfo_file]} {
            set buildinfo_fh [open $buildinfo_file r]
            set buildinfo [read $buildinfo_fh]
            close $buildinfo_fh
            if {[regexp -line {^build_id=(0x[0-9A-Fa-f]{8})$} $buildinfo -> expected_build_id]} {
                if {[string toupper $build_id] ne [string toupper $expected_build_id]} {
                    puts "> WARNING: bitstream build_id ($build_id) != buildinfo ($expected_build_id); bitstream may not be the one you just baked"
                } elseif {!$quiet} {
                    puts "> jtag_repl: build_id matches buildinfo ($expected_build_id)"
                }
            }
        }
        # NOT "return $build_id" here -- a `return` inside a `catch {...}`
        # script is intercepted as an exceptional (TCL_RETURN) completion
        # code, so `catch` treats it as a failure even on the success path:
        # $build_id_err below would end up holding the build_id STRING
        # itself, printed as "build_id read failed: 0xBADA0BAD" even when
        # the read genuinely succeeded. Falling through with a bare `set`
        # as the script's last statement makes it the script's real return
        # value on normal (TCL_OK) completion instead.
        set build_id
    } build_id_err]} {
        # CPU/debug fabric may not be ready yet; do not abort REPL startup.
        if {!$quiet} {
            puts "> jtag_repl: build_id read failed: $build_id_err"
        }
        return ""
    }
}

# VIO probe_out0 — used for umbrella full-reset (bit 3) plus boot-ctl bits.
# Bound on first use; nil if no VIO/LTX in this bitstream.
set ::vio_probe_out ""
proc vio_probe_out {} {
    if {$::vio_probe_out eq ""} {
        set vios [get_hw_vios -quiet]
        if {[llength $vios] == 0} {
            error "no VIO core (need ENABLE_VIO=1 bitstream + LTX)"
        }
        set vio [lindex $vios 0]
        # Match by NAME or PROBE_NAME property — Vivado returns probe handles
        # whose name varies by tool version; check both attributes.
        # The probe driving probe_out0 is named "vio_boot_ctrl" (the RTL
        # wire feeding the IP).  Look it up by NAME.
        set candidates [get_hw_probes -of_objects $vio -quiet]
        foreach p $candidates {
            if {[get_property NAME $p] eq "vio_boot_ctrl"} {
                set ::vio_probe_out $p
                break
            }
        }
        if {$::vio_probe_out eq ""} {
            # Fallback for a renamed/uniquified net.  It must NOT match a
            # 1-bit probe: since probe_out1 (vio_hard_reset) landed there
            # are TWO output probes here, get_hw_probes ordering is not
            # contractual, and binding this handle to the hard-reset line
            # would turn `vio_set 8` into a board reset.
            foreach p $candidates {
                if {[catch {set _ [get_property OUTPUT_VALUE $p]}] != 0} { continue }
                set w ""
                catch {set w [get_property WIDTH $p]}
                if {[string is integer -strict $w] && $w > 1} {
                    set ::vio_probe_out $p
                    break
                }
            }
        }
        if {$::vio_probe_out eq ""} {
            error "vio probe_out0 / vio_boot_ctrl not found"
        }
    }
    return $::vio_probe_out
}

# Vivado's `set_property OUTPUT_VALUE` on a HEX-radix probe requires the
# value STRING to carry EXACTLY ceil(WIDTH/4) hex characters.  Too few is a
# hard error, not a zero-extend:
#   [Designutils 20-1474] hw_probe VIO value [8] has [1] value characters,
#   required [2]
#
# That error was live on this bitstream.  probe_out0 (vio_boot_ctrl) went
# 4 bits -> 5 bits at probe_map=v20 (876de3c, which added the PRAM-zap bit),
# and every caller passing a single hex digit — `vio_set 8`, `vio_set 0`,
# and pram_clear_pulse's own `format %X` of a value < 16 — broke at once.
# The digit count is therefore queried from the LIVE probe, never hardcoded:
# the width has already changed once and can change again.
# ── Scan-out placement admission verdicts ────────────────────────────────
# Mirrors the REJ_* localparams in rtl/board/video_phy/mode_admit.v (stage 3,
# the single admission authority -- they moved there out of
# scanout_placement_sync.v when that module stopped owning the gates).
# Keep the two in step: the numbers are a wire protocol between the RTL and
# this decoder, and a silent divergence here is worse than no decode at all
# because it names the wrong gate confidently.
#
# The whole point of the channel (docs/video_path_review.md S4.2) is that a
# refused placement used to produce rd_en=0 forever with no reason code, no
# counter and no status bit -- diagnosing the black screen it caused meant
# hand-evaluating four inequalities against live register values.
proc video_reject_reason_name {code} {
    switch -- $code {
        0 { return "OK (admitted)" }
        1 { return "DEPTH_UNSUPPORTED - 16/24bpp; the scanner cannot render it, last good placement retained" }
        2 { return "STRIDE_SHORT - fb_stride is smaller than one visible row; the scanner would read into the next row" }
        3 { return "FRAME_OUT_OF_MEMORY - base + stride*(vres-1) + row_extent is past the framebuffer aperture" }
        4 { return "GEOMETRY_ZERO - admitted, but hres or vres is 0 so the active window is EMPTY and nothing can render" }
        default { return "UNKNOWN($code) - decoder is older than the bitstream; check mode_admit.v's REJ_* list" }
    }
}

proc vio_fmt_value {p v} {
    set w ""
    if {[catch {set w [get_property WIDTH $p]}] || ![string is integer -strict $w] || $w <= 0} {
        error "vio_set: cannot read WIDTH of probe [get_property NAME $p] — probe absent or unreadable on this bitstream (needs ENABLE_VIO=1 and a matching LTX)"
    }
    if {$w < 32} {
        set masked [expr {$v & ((1 << $w) - 1)}]
        if {$masked != $v} {
            puts "> WARNING vio_set: 0x[format %X $v] does not fit the ${w}-bit probe [get_property NAME $p]; truncated to 0x[format %X $masked]"
        }
        set v $masked
    }
    return [format %0*X [expr {($w + 3) / 4}] $v]
}

proc vio_set {hex_val} {
    set p [vio_probe_out]
    # Pin the radix: the digit-count arithmetic above assumes HEX, and a
    # stale session RADIX (BINARY/UNSIGNED) would silently change what
    # Vivado requires.
    catch {set_property RADIX HEX $p}
    if {![regexp {^(?:0[xX])?([0-9a-fA-F]+)$} $hex_val -> digits]} {
        error "vio_set: '$hex_val' is not a hex value"
    }
    set v 0
    scan $digits %x v
    set_property OUTPUT_VALUE [vio_fmt_value $p $v] $p
    commit_hw_vio [list $p]
}

# VIO probe_out1 — dedicated hard-reset control (2026-07-24), a VIO
# equivalent of a physical btn[3] press.  Deliberately wired downstream
# of debounce into platform_reset_req/platform_resetn ONLY, NOT into
# fabric_gt_clr's raw BUFG_GT .CLR -- see fpga_top_clocks.vh's
# btn3_resetn_db comment for why a JTAG-driven pulse must not touch that
# path (risk of killing dbg_hub's own clock mid-transaction, the same
# reason jtag_debug_full_reset_eff is excluded from it).  Use this when
# the CSR-based `reset` command (unified_reset, bits 4/5 on debug_ctrl)
# has left the JTAG-AXI bridge unresponsive -- that class of wedge has
# so far only recovered via a physical power-cycle; this gives a
# scriptable equivalent of the OTHER physical recovery button (btn[3])
# without leaving the chair.
set ::vio_probe_out1 ""
proc vio_probe_out1 {} {
    if {$::vio_probe_out1 eq ""} {
        set vios [get_hw_vios -quiet]
        if {[llength $vios] == 0} {
            error "no VIO core (need ENABLE_VIO=1 bitstream + LTX)"
        }
        set vio [lindex $vios 0]
        foreach p [get_hw_probes -of_objects $vio -quiet] {
            if {[get_property NAME $p] eq "vio_hard_reset"} {
                set ::vio_probe_out1 $p
                break
            }
        }
        if {$::vio_probe_out1 eq ""} {
            error "vio probe_out1 / vio_hard_reset not found -- bitstream predates this probe (rebuild needed)"
        }
    }
    return $::vio_probe_out1
}
# WHAT IT REACHES (2026-09-12): probe_out1 -> vio_hard_reset -> btn3_resetn_db
# -> platform_resetn -> soc_hard_rst_req, which now drives BOTH clk_rst
# instances (core + pb) AND the DDR4 MIG's sys_rst.  Before that the MIG was
# on `~btn[0]` alone, so the memory controller was the one block a VIO reset
# could not reach.  It deliberately does NOT touch fabric_gt_clr / BUFG_GT
# .CLR: pulsing GT CLR during an in-flight JTAG transaction can kill dbg_hub's
# clock, and a VIO write IS a JTAG transaction.
proc vio_hard_reset_pulse {{hold_ms 50}} {
    set p [vio_probe_out1]
    set_property OUTPUT_VALUE 1 $p
    commit_hw_vio [list $p]
    after $hold_ms
    set_property OUTPUT_VALUE 0 $p
    commit_hw_vio [list $p]
}

# ── PRAM zap — vio_boot_ctrl[4] (the RTL equivalent of Cmd-Opt-P-R) ──────
#
# Why this exists: PRAM is battery-backed as of the pram-survives-reset
# change — rtc.v deliberately never clears the array, not on `rst` and not
# on a debug-full-reset, so Mac OS settings persist across reboots the way
# they do on real silicon.  The flip side is that a PRAM image which wedges
# the ROM boot would otherwise only be recoverable by re-loading the
# bitstream.  This is that escape hatch.
#
# The probe bit is LEVEL driven, but fpga_top_clocks.vh turns its RISING
# EDGE into a fixed-length one-shot (then a 2-FF CDC into pb_clk), so all
# this proc has to produce is a clean 0 -> 1 -> 0 on probe_out0[4].
# Leaving the bit high does NOT hold PRAM clear — the pulse self-expires.
#
# Read-modify-write so the other probe_out0 bits (boot bypass/release,
# scc_uart_sel_b, debug_full_reset) survive the pulse untouched — unlike
# the blunt `vio_set` path, which rewrites the whole bus.
#
# NOTE: needs a bitstream whose probe_out0 is 5 bits wide (probe_map=v20 or
# later, synth/vivado.tcl).  On an older 4-bit bitstream bit4 is simply
# absent and the zap silently does nothing.
proc pram_clear_pulse {{hold_ms 50}} {
    set p [vio_probe_out]
    set cur 0
    catch {scan [get_property OUTPUT_VALUE $p] %x cur}
    if {![string is integer -strict $cur]} { set cur 0 }
    set base [expr {$cur & 0x0F}]
    # Force bit4 LOW first.  The RTL fires on the RISING edge, so if a
    # previous aborted run (or a stale probe state) left the bit high, a
    # plain set-then-clear would produce no edge at all and the zap would
    # silently do nothing.  Drive low -> settle -> high -> low so the edge
    # is guaranteed regardless of the starting state.
    # Width-padded via vio_fmt_value: a bare `format %X` of any value < 16
    # yields ONE hex character, which the 5-bit probe rejects outright
    # ([Designutils 20-1474]).  That made `pram-clear` fail on exactly the
    # common case where the other boot-ctrl bits happened to be clear.
    catch {set_property RADIX HEX $p}
    set_property OUTPUT_VALUE [vio_fmt_value $p $base] $p
    commit_hw_vio [list $p]
    after 10
    set_property OUTPUT_VALUE [vio_fmt_value $p [expr {$base | 0x10}]] $p
    commit_hw_vio [list $p]
    after $hold_ms
    set_property OUTPUT_VALUE [vio_fmt_value $p $base] $p
    commit_hw_vio [list $p]
    return $base
}

proc rd {addr {axi ""}} {
    if {$axi eq ""} { set axi $::axi }
    # Robustness: a failed JTAG-AXI txn (lost hw_target, MIG not
    # calibrated, etc.) can leave DATA empty.  An empty string
    # propagates into `scan %x` → "" → `expr {& ...}` and crashes
    # every caller with "can't use empty string as operand of &".
    # Always return a well-formed 8-hex-digit string; surface a
    # failed read as the all-hex sentinel BADA0BAD so the operator
    # sees it without the whole REPL falling over.
    if {[catch {
        create_hw_axi_txn -quiet -force _r $axi -type READ -address [format %08X $addr] -len 1
        run_hw_axi -quiet _r
        set v [get_property DATA [get_hw_axi_txns _r]]
    } err]} {
        # Surface the real error (e.g. "[Xicom 50-38] AXI TRANSACTION TIMED
        # OUT") instead of silently eating it -- callers still get the
        # well-formed BADA0BAD sentinel below so they never crash on an
        # empty DATA string, but a human watching jtag_out can now tell a
        # genuine 10s Xicom timeout apart from any other failure class
        # without re-running the raw create_hw_axi_txn/run_hw_axi sequence
        # by hand every time.
        puts "> jtag_repl: rd 0x[format %08X $addr] FAILED: $err"
        return "BADA0BAD"
    }
    if {$v eq "" || ![string is xdigit -strict $v]} {
        # This is the path that ACTUALLY fires for a Xicom-level timeout in
        # practice: create_hw_axi_txn/run_hw_axi use -quiet above, which
        # suppresses the Tcl-level exception the catch{} block above is
        # watching for, so a timed-out transaction completes "normally"
        # with an empty DATA property instead of throwing -- the catch{}
        # branch's error message never fires for this class of failure.
        # STATUS is queryable even under -quiet and is the only surviving
        # clue about why DATA came back empty.
        set txn_status ""
        catch {set txn_status [get_property STATUS [get_hw_axi_txns _r]]}
        puts "> jtag_repl: rd 0x[format %08X $addr] FAILED: empty/non-hex DATA (txn STATUS=$txn_status)"
        return "BADA0BAD"
    }
    return $v
}
proc wr {addr data {axi ""}} {
    if {$axi eq ""} { set axi $::axi }
    create_hw_axi_txn -quiet -force _w $axi -type WRITE -address [format %08X $addr] -data [format %08X $data]
    run_hw_axi -quiet _w
}

# ══════════════════════════════════════════════════════════════════════
# Correctness wrappers — never hand a fabricated value to a caller
# ══════════════════════════════════════════════════════════════════════
#
# Two silent-fabrication traps used to live here.  Both produced PLAUSIBLE
# wrong values, which is the worst possible failure mode for a debug tool:
#
#   1. `rd` returns a BARE 8-hex-digit string with no 0x prefix.  Any
#      caller that feeds it to Tcl's `expr` gets it parsed as DECIMAL.
#      "40800000" -> 40800000 -> 0x026E8F80.  Silently, plausibly wrong.
#
#   2. The command dispatcher parsed operands with `expr {[lindex ...]}`,
#      so a bare hex address typed at the REPL prompt (`r 40800000`, the
#      natural thing to type when every value you look at is hex) was ALSO
#      read as decimal.
#
# Use `rdx` wherever a NUMBER is wanted and `rd` only where the raw
# display string is wanted.  `rdx` refuses to return a value for a failed
# transaction rather than passing the BADA0BAD sentinel off as data.
proc rdx {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} {
        error "AXI read failed at [format 0x%08X $addr] (BADA0BAD sentinel) -- refusing to return a fabricated value"
    }
    return [expr {"0x$v"}]
}

# Same, but for a caller that wants to handle failure itself: returns the
# integer, or the empty string on a failed read.  Never returns 0xBADA0BAD
# as if it were data.
proc rdx_or_empty {addr} {
    set v [rd $addr]
    if {$v eq "BADA0BAD"} { return "" }
    return [expr {"0x$v"}]
}

# Parse a numeric operand typed at the REPL.
#
# HEX IS THE DEFAULT.  Every operand this REPL takes is an address, a
# register value, or a bit mask; none of them are naturally decimal, and a
# bare-hex-read-as-decimal is exactly the trap described above.  Explicit
# forms are still honoured:
#
#   0x1234 / 0X1234   hex
#   #1234             hex (alternate explicit prefix)
#   d1234 / 1234d     DECIMAL, when you really mean decimal
#
# Anything that is not a valid number in the selected base is an ERROR --
# never a silent 0.
proc parse_num {tok {what "value"}} {
    if {$tok eq ""} { error "missing $what" }
    set t $tok
    if {[string match "0x*" $t] || [string match "0X*" $t]} {
        set body [string range $t 2 end]
        if {![string is xdigit -strict $body]} { error "bad hex $what: $tok" }
        return [expr {"0x$body"}]
    }
    if {[string match "#*" $t]} {
        set body [string range $t 1 end]
        if {![string is xdigit -strict $body]} { error "bad hex $what: $tok" }
        return [expr {"0x$body"}]
    }
    if {[string match "d*" $t] || [string match "D*" $t]} {
        set body [string range $t 1 end]
        if {![string is integer -strict $body]} { error "bad decimal $what: $tok" }
        return [expr {$body}]
    }
    if {[string match "*d" $t] || [string match "*D" $t]} {
        set body [string range $t 0 end-1]
        if {![string is integer -strict $body]} { error "bad decimal $what: $tok" }
        return [expr {$body}]
    }
    if {![string is xdigit -strict $t]} { error "bad hex $what: $tok" }
    return [expr {"0x$t"}]
}

# Word-align check.  The JTAG-AXI master is 32-bit and word-addressed, so
# the Vivado hw_axi layer silently ALIGNS DOWN a byte address -- another
# way to get a plausible wrong answer, and one that cannot be fixed inside
# `rd` because the masking happens in the IP, not in this script.  Reject
# it at the command layer instead of letting it happen quietly.
proc require_aligned {addr {what "address"}} {
    if {$addr & 0x3} {
        error [format "unaligned %s 0x%08X -- the JTAG-AXI master is 32-bit word-addressed and would silently read 0x%08X instead. Re-issue with an aligned address." \
               $what $addr [expr {$addr & ~0x3}]]
    }
    return $addr
}

proc sdj_addr {off} { return [expr {$::SDJ_BASE + $off}] }
proc sd_write_file {start_lba path} {
    if {![file exists $path]} { error "sd-write: file not found: $path" }

    set ident_hex [rd [sdj_addr $::SDJ_OFF_IDENT]]
    scan $ident_hex %x ident
    if {$ident != $::SDJ_IDENT} {
        error "sd-write: SD JTAG writer not present at [format 0x%08X $::SDJ_BASE] (ident=0x$ident_hex)"
    }

    set fh [open $path "rb"]
    fconfigure $fh -translation binary -encoding binary
    set lba $start_lba
    set sectors 0
    while {![eof $fh]} {
        set chunk [read $fh 512]
        set n [string length $chunk]
        if {$n == 0} { break }
        if {$n < 512} {
            append chunk [string repeat "\x00" [expr {512 - $n}]]
        }

        binary scan $chunk c* bytes
        wr [sdj_addr $::SDJ_OFF_BUFPTR] 0
        foreach b $bytes {
            wr [sdj_addr $::SDJ_OFF_BUFDATA] [expr {$b & 0xFF}]
        }
        wr [sdj_addr $::SDJ_OFF_LBA] $lba
        wr [sdj_addr $::SDJ_OFF_CTRL] $::SDJ_CTRL_GO

        set done 0
        for {set poll 0} {$poll < 200000} {incr poll} {
            set st_hex [rd [sdj_addr $::SDJ_OFF_STATUS]]
            scan $st_hex %x st
            if {($st & 0x4) != 0} {
                close $fh
                error "sd-write: error at lba=$lba status=[format 0x%08X $st]"
            }
            if {(($st & 0x1) == 0) && (($st & 0x2) != 0)} {
                set done 1
                break
            }
        }
        if {!$done} {
            close $fh
            error "sd-write: timeout at lba=$lba"
        }

        incr sectors
        incr lba
        if {($sectors % 128) == 0} {
            puts "> sd-write progress: $sectors sectors"
            flush stdout
        }
    }
    close $fh
    return $sectors
}

# ══════════════════════════════════════════════════════════════════════════
# PRAM persistence — rtl/soc/pram_sd.v  (`pram-save` / `pram-load` /
# `pram-dump`).  Manual ONLY: nothing in the hardware ever starts one of
# these by itself, and the Mac cannot reach this register file at all.
# ══════════════════════════════════════════════════════════════════════════
#
# pram_sd shares the AXI_SD_JTAG_BASE slave window with sd_jtag_writer,
# split on address bit 8 (rtl/soc/axil_split2.v):
#     0x50A0_0000  sd_jtag_writer  (`sd-write`, unchanged)
#     0x50A0_0100  pram_sd         (here)
# Unlike sd_jtag_writer, pram_sd is NOT gated out of production bitstreams.
set PRAMSD_BASE         0x50A00100
set PRAMSD_OFF_IDENT    0x000
set PRAMSD_OFF_CTRL     0x004
set PRAMSD_OFF_STATUS   0x008
set PRAMSD_OFF_BUFPTR   0x00C
set PRAMSD_OFF_BUFDATA  0x010
set PRAMSD_OFF_LBA      0x014
set PRAMSD_OFF_CKREF    0x018
set PRAMSD_IDENT        0x50524D31
set PRAMSD_CMD_SAVE     0x50520001
set PRAMSD_CMD_LOAD     0x50520002
set PRAMSD_CMD_SNAP     0x50520003

proc pramsd_addr {off} { return [expr {$::PRAMSD_BASE + $off}] }

proc pramsd_result_name {code} {
    switch -- $code {
        0  { return "OK" }
        1  { return "BAD_MAGIC     (sector is not a PRAM image)" }
        2  { return "BAD_VERSION   (saved by a different format version)" }
        3  { return "BAD_LENGTH    (payload length field is not 256)" }
        4  { return "BAD_CHECKSUM  (image is corrupt)" }
        5  { return "BLANK         (sector is all zeros — never saved)" }
        6  { return "SD_ERROR      (card refused or failed the transfer)" }
        7  { return "ARB_TIMEOUT   (SD bus never went idle — SCSI busy?)" }
        8  { return "NOT_BOOTED    (boot_fsm has not released the card yet)" }
        9  { return "CDC_TIMEOUT   (RTC clock domain did not respond)" }
        10 { return "BAD_COMMAND   (REPL/bitstream mismatch)" }
        default { return "UNKNOWN($code)" }
    }
}

# Presence probe.  Loud, and it names the concrete reason rather than
# leaving the operator to guess — an absent register file reads as
# 0x00000000 or BADA0BAD, both of which are easy to misread as "worked".
proc pramsd_require_present {what} {
    set ident_hex [rd [pramsd_addr $::PRAMSD_OFF_IDENT]]
    if {![scan $ident_hex %x ident]} { set ident 0 }
    if {$ident != $::PRAMSD_IDENT} {
        error "$what: pram_sd NOT PRESENT at [format 0x%08X $::PRAMSD_BASE] (IDENT=0x$ident_hex, expected [format 0x%08X $::PRAMSD_IDENT]).\
\n>        This bitstream predates PRAM-to-SD persistence, or the JTAG-AXI\
\n>        link is down (BADA0BAD = failed AXI read).  Nothing was written."
    }
}

# The SD bus interlock in fpga_top_sd.vh guarantees we never preempt an
# SCSI transfer that is ALREADY in flight.  It cannot stop the Mac from
# starting a NEW one while we hold the bus — that one stalls on sd_ctrl's
# ~10 s watchdog and surfaces as a retryable SCSI error.  So: warn, loudly,
# and say exactly what the risk is.
proc pramsd_halt_warn {what} {
    if {[catch {set halted [effective_halt]}]} { set halted 0 }
    if {$halted} { return }
    puts "> WARNING: $what with the CPU RUNNING."
    puts ">        The hardware interlock will not interrupt an SD transfer that is"
    puts ">        already in flight, but a SCSI command the Mac issues DURING this"
    puts ">        operation will stall until sd_ctrl's ~10 s watchdog fires and then"
    puts ">        surface as a retryable SCSI error.  Halt the CPU first (`halt` /"
    puts ">        `break-pc <pc>`) if the machine is doing disk I/O."
    flush stdout
}

# Issue a command and poll to completion.  Bounded on BOTH sides: the RTL
# terminates every path itself, and this loop refuses to spin forever if
# the register file stops answering at all.
proc pramsd_run {cmd what} {
    wr [pramsd_addr $::PRAMSD_OFF_CTRL] $cmd
    for {set poll 0} {$poll < 20000} {incr poll} {
        set st_hex [rd [pramsd_addr $::PRAMSD_OFF_STATUS]]
        if {$st_hex eq "BADA0BAD"} {
            error "$what: AXI read of STATUS failed (BADA0BAD) — JTAG link down."
        }
        scan $st_hex %x st
        if {(($st & 0x1) == 0) && (($st & 0x2) != 0)} { return $st }
    }
    error "$what: TIMED OUT waiting for pram_sd to report done (last STATUS=$st_hex).\
\n>        pram_sd bounds every internal wait itself, so a hang here means the\
\n>        register file stopped responding, not that the operation is slow."
}

proc pramsd_report {st what} {
    set res      [expr {($st >> 4) & 0xF}]
    set defaults [expr {($st >> 3) & 0x1}]
    set sderr    [expr {($st >> 8) & 0xF}]
    set ckcalc   [expr {($st >> 16) & 0xFFFF}]
    if {$res == 0} {
        puts "> $what: OK   (status=[format 0x%08X $st] cksum=[format 0x%04X $ckcalc])"
        return 1
    }
    set ckref 0
    catch {scan [rd [pramsd_addr $::PRAMSD_OFF_CKREF]] %x ckref}
    puts "> $what: FAILED — [pramsd_result_name $res]"
    puts ">        status=[format 0x%08X $st] sd_err_cause=$sderr"
    puts ">        checksum computed=[format 0x%04X $ckcalc] stored-in-sector=[format 0x%04X [expr {$ckref & 0xFFFF}]]"
    if {$defaults} {
        puts ">        PRAM WAS RESET TO ITS POST-RESET DEFAULTS (rtc.v pram_reset_value,"
        puts ">        the same image `pram-clear` produces).  The saved sector was NOT"
        puts ">        installed and the previous live contents are GONE."
    } else {
        puts ">        PRAM was left untouched."
    }
    return 0
}

proc pram_save {} {
    pramsd_require_present "pram-save"
    pramsd_halt_warn "pram-save"
    set lba 0
    catch {scan [rd [pramsd_addr $::PRAMSD_OFF_LBA]] %x lba}
    puts "> pram-save: writing live PRAM to SD LBA $lba ..."
    flush stdout
    set st [pramsd_run $::PRAMSD_CMD_SAVE "pram-save"]
    return [pramsd_report $st "pram-save"]
}

proc pram_load {} {
    pramsd_require_present "pram-load"
    pramsd_halt_warn "pram-load"
    set lba 0
    catch {scan [rd [pramsd_addr $::PRAMSD_OFF_LBA]] %x lba}
    puts "> pram-load: reading SD LBA $lba into live PRAM ..."
    flush stdout
    set st [pramsd_run $::PRAMSD_CMD_LOAD "pram-load"]
    return [pramsd_report $st "pram-load"]
}

# Hex dump of the LIVE 256 PRAM bytes.  Touches no SD sector at all — the
# SNAP command only copies rtc.v's array into pram_sd's staging buffer.
proc pram_dump {} {
    pramsd_require_present "pram-dump"
    set st [pramsd_run $::PRAMSD_CMD_SNAP "pram-dump"]
    if {![pramsd_report $st "pram-dump"]} { return 0 }

    wr [pramsd_addr $::PRAMSD_OFF_BUFPTR] 0
    set bytes {}
    for {set i 0} {$i < 272} {incr i} {
        set v 0
        catch {scan [rd [pramsd_addr $::PRAMSD_OFF_BUFDATA]] %x v}
        lappend bytes [expr {$v & 0xFF}]
    }
    set magic ""
    foreach b [lrange $bytes 0 3] { append magic [format %c $b] }
    puts "> pram-dump: header magic='$magic' version=[expr {([lindex $bytes 4]<<8)|[lindex $bytes 5]}]\
 length=[expr {([lindex $bytes 6]<<8)|[lindex $bytes 7]}]\
 cksum=[format 0x%02X%02X [lindex $bytes 8] [lindex $bytes 9]]"
    for {set row 0} {$row < 256} {incr row 16} {
        set line [format "  %02X:" $row]
        set ascii ""
        for {set c 0} {$c < 16} {incr c} {
            set b [lindex $bytes [expr {16 + $row + $c}]]
            append line [format " %02X" $b]
            append ascii [expr {($b >= 32 && $b < 127) ? [format %c $b] : "."}]
        }
        puts "$line  |$ascii|"
    }
    flush stdout
    return 1
}

# ── sd-write-fast — bulk SD write via the DEDICATED provisioning bitstream ──
# (rtl/soc/sd_provision_top.v + sd_provision_core.v, make sd-provision-impl).
# Uses hw_axi burst transactions (CONFIG.M_HAS_BURST=1 on prov_jtag_axi)
# INCR burst instead of a single 32-bit word.  Flow per batch (up to
# CAPS.stage_sectors = 256 sectors = 128 KiB):
#   1. burst-write the batch into the staging BRAM (128 x 1 KiB txns);
#   2. poke LBA/BLKCNT, CTRL=GO_WRITE → hardware streams one CMD25;
#   3. poll STATUS, check the hardware CRC32 of the streamed bytes;
#   4. (default) CTRL=GO_VERIFY → hardware CMD18-reads the LBA range back
#      and the CRC32 register must reproduce the host-computed CRC.

proc sdp_addr {off} { return [expr {$::SDP_REG_BASE + $off}] }

proc sdp_rd {off} {
    set v [rd [sdp_addr $off]]
    if {$v eq "BADA0BAD"} { error "sd-write-fast: AXI read failed (off=$off)" }
    scan $v %x n
    return $n
}

proc sdp_err_name {cause} {
    switch -- $cause {
        0  { return "none" }
        1  { return "R1 rejected" }
        2  { return "R1 timeout (card silent)" }
        3  { return "data token timeout" }
        4  { return "data-response bad (write rejected)" }
        5  { return "busy timeout" }
        6  { return "unsupported cmd" }
        14 { return "bad BLKCNT" }
        15 { return "card not ready" }
        default { return "cause=$cause" }
    }
}

proc sdp_probe {} {
    set ident [sdp_rd $::SDP_OFF_IDENT]
    if {$ident != $::SDP_IDENT} {
        error "sd-write-fast: bulk writer not present (ident=[format 0x%08X $ident]).\n  Load the provisioning bitstream first:\n    make sd-provision-impl\n    load-bit build/sd_provision/sd_provision_top.bit"
    }
    set caps [sdp_rd $::SDP_OFF_CAPS]
    return [expr {$caps & 0xFFFF}]
}

proc sdp_wait_card_ready {} {
    # boot_fsm init (CMD0/8/55+41/58/6 + 1 dummy sector at 200 kHz SPI)
    # takes ~1-2 s after configuration.
    for {set i 0} {$i < 100} {incr i} {
        set st [sdp_rd $::SDP_OFF_STATUS]
        if {($st & 0x200) != 0} {
            error "sd-write-fast: SD card init FAILED (STATUS=[format 0x%08X $st]) — is a card inserted?"
        }
        if {($st & 0x100) != 0} { return }
        after 100
    }
    error "sd-write-fast: timed out waiting for card_ready (STATUS bit 8)"
}

# Issue one burst write of up to 256 x 32-bit words in a single hw_axi
# transaction.  $words is a list of integers; word i lands at addr + 4*i.
proc sdp_wrburst {addr words} {
    set n [llength $words]
    set hexes {}
    foreach w $words { lappend hexes [format %08X [expr {$w & 0xFFFFFFFF}]] }
    if {$::sdp_word_order eq "rev"} {
        set data [join [lreverse $hexes] ""]
    } else {
        set data [join $hexes ""]
    }
    create_hw_axi_txn -quiet -force _wb $::axi -type WRITE \
        -address [format %08X $addr] -len $n -data $data
    run_hw_axi -quiet _wb
}

# Determine whether the first word of the -data hex string lands at the
# lowest or highest address of a burst (Vivado versions differ) by writing
# a 4-word probe burst to the staging BRAM and reading word 0 back.
proc sdp_calibrate_word_order {} {
    if {$::sdp_word_order ne ""} { return }
    set probe [list 0x11AA22BB 0x33CC44DD 0x55EE66FF 0x77008811]
    set ::sdp_word_order "fwd"
    sdp_wrburst $::SDP_STG_BASE $probe
    set w0 [expr {[sdp_rd [expr {$::SDP_STG_BASE - $::SDP_REG_BASE}]]}]
    set w1 [sdp_rd [expr {$::SDP_STG_BASE - $::SDP_REG_BASE + 4}]]
    if {$w0 == [lindex $probe 0] && $w1 == [lindex $probe 1]} {
        set ::sdp_word_order "fwd"
    } elseif {$w0 == [lindex $probe 3] && $w1 == [lindex $probe 2]} {
        set ::sdp_word_order "rev"
    } else {
        set ::sdp_word_order ""
        error "sd-write-fast: burst word-order calibration failed (w0=[format 0x%08X $w0] w1=[format 0x%08X $w1])"
    }
    puts "> sd-write-fast: burst word order = $::sdp_word_order"
}

proc sdp_poll_done {what timeout_ms} {
    set waited 0
    while {1} {
        set st [sdp_rd $::SDP_OFF_STATUS]
        if {(($st & 0x1) == 0) && (($st & 0x2) != 0)} {
            if {($st & 0x4) != 0} {
                set cause [expr {($st >> 4) & 0xF}]
                error "sd-write-fast: $what error — [sdp_err_name $cause] (STATUS=[format 0x%08X $st])"
            }
            return
        }
        after 20
        incr waited 20
        if {$waited >= $timeout_ms} {
            error "sd-write-fast: $what timeout (STATUS=[format 0x%08X $st])"
        }
    }
}

# Recover a degraded JTAG-AXI path without losing the host-side transfer
# position.  Long provisioning runs have shown that after many burst
# transactions the AXI debug path can return changing, impossible CRC values;
# reprogramming the SAME provisioning image immediately restores exact reads.
# The caller still owns the source chunk and retries the same LBA afterwards.
proc sdp_reprogram_current {} {
    set bit [get_property PROGRAM.FILE $::hw_dev]
    if {$bit eq "" || ![file exists $bit]} {
        error "sd-write-fast: cannot recover JTAG AXI — current PROGRAM.FILE is missing: '$bit'"
    }
    puts "> sd-write-fast: reprogramming $bit to recover the JTAG-AXI path"
    flush stdout
    program_hw_devices $::hw_dev
    refresh_hw_device $::hw_dev
    set ::axi [lindex [get_hw_axis] 0]
    if {$::axi eq ""} {
        error "sd-write-fast: no JTAG AXI master after recovery reprogram"
    }
    set ::sdp_word_order ""
    sdp_probe
    sdp_wait_card_ready
    sdp_calibrate_word_order
}

proc sd_write_fast_file {start_lba path {do_verify 1}} {
    if {![file exists $path]} { error "sd-write-fast: file not found: $path" }
    set stage_sectors [sdp_probe]
    sdp_wait_card_ready
    sdp_calibrate_word_order

    set have_crc [expr {![catch {zlib crc32 ""}]}]
    if {!$have_crc} {
        puts "> sd-write-fast: WARNING — this Tcl has no zlib crc32; CRC checks skipped"
    }

    set fsize [file size $path]
    set total_sectors [expr {($fsize + 511) / 512}]
    set fh [open $path "rb"]
    fconfigure $fh -translation binary -encoding binary

    set lba $start_lba
    set sectors_done 0
    set t0 [clock milliseconds]
    set batch_bytes [expr {$stage_sectors * 512}]

    while {![eof $fh]} {
        set chunk [read $fh $batch_bytes]
        set n [string length $chunk]
        if {$n == 0} { break }
        set pad [expr {(512 - ($n % 512)) % 512}]
        if {$pad > 0} { append chunk [string repeat "\x00" $pad] }
        set nsec [expr {[string length $chunk] / 512}]

        binary scan $chunk i* words        ;# 32-bit little-endian words
        set nwords [llength $words]
        if {$have_crc} {
            set expect [expr {[zlib crc32 $chunk] & 0xFFFFFFFF}]
        }

        # A JTAG burst can very rarely corrupt staging data.  The stream CRC
        # is available only after CMD25, so recovery must rewrite the SAME LBA
        # batch before advancing.  Three bounded attempts avoid turning one
        # transient into a partially provisioned disk without hiding a
        # persistent cable/BRAM fault.
        set batch_ok 0
        set batch_why "unknown failure"
        set recoveries 0
        while {!$batch_ok && $recoveries <= 3} {
            for {set attempt 1} {$attempt <= 3 && !$batch_ok} {incr attempt} {
                # Preserve maximum throughput normally, but shorten JTAG AXI
                # bursts after a staging CRC failure.  Dense data on the real
                # cable has shown non-repeatable corruption with 256-beat bursts;
                # 128/64-beat retries trade speed for signal/bridge margin.
                set burst_words [expr {256 >> ($attempt - 1)}]
                # 1. Fill staging with bounded AXI bursts.
                for {set off 0} {$off < $nwords} {incr off $burst_words} {
                    set hi [expr {min($off + $burst_words - 1, $nwords - 1)}]
                    sdp_wrburst [expr {$::SDP_STG_BASE + 4 * $off}] \
                        [lrange $words $off $hi]
                }

                # 2. Kick the hardware CMD25 batch write.
                wr [sdp_addr $::SDP_OFF_LBA]    $lba
                wr [sdp_addr $::SDP_OFF_BLKCNT] $nsec
                wr [sdp_addr $::SDP_OFF_CTRL]   $::SDP_CTRL_GO_WRITE
                # A hardware error here (data token timeout, card busy
                # timeout) used to throw straight past this retry loop and
                # abandon the whole transfer.  Only CRC mismatches were
                # retried.  Treat an errored batch the same way -- the card
                # stalling longer than the controller's budget is exactly the
                # transient worth retrying, and sd_ctrl closes the transaction
                # cleanly on the error path before we come round again.
                if {[catch {sdp_poll_done "CMD25 write (lba=$lba n=$nsec)" 30000} werr]} {
                    set batch_why "write error: $werr"
                    puts "> sd-write-fast: WARNING $batch_why at lba=$lba — retrying batch ($attempt/3)"
                    flush stdout
                    continue
                }

                # 3. CRC of the byte stream the hardware sent to the card.
                if {$have_crc} {
                    set got [sdp_rd $::SDP_OFF_CRC32]
                    if {$got != $expect} {
                        set batch_why "write-stream CRC mismatch (hw=[format 0x%08X $got] host=[format 0x%08X $expect])"
                        puts "> sd-write-fast: WARNING $batch_why at lba=$lba after ${burst_words}-beat bursts — retrying batch ($attempt/3)"
                        flush stdout
                        continue
                    }
                }

                # 4. Read-back verify (CMD18 + hardware CRC32).
                if {$do_verify && $have_crc} {
                    wr [sdp_addr $::SDP_OFF_CTRL] $::SDP_CTRL_GO_VERIFY
                    if {[catch {sdp_poll_done "CMD18 verify (lba=$lba n=$nsec)" 30000} verr]} {
                        set batch_why "read-back error: $verr"
                        puts "> sd-write-fast: WARNING $batch_why at lba=$lba — retrying batch ($attempt/3)"
                        flush stdout
                        continue
                    }
                    set got [sdp_rd $::SDP_OFF_CRC32]
                    if {$got != $expect} {
                        set batch_why "read-back CRC mismatch (hw=[format 0x%08X $got] host=[format 0x%08X $expect])"
                        puts "> sd-write-fast: WARNING $batch_why at lba=$lba after ${burst_words}-beat bursts — retrying batch ($attempt/3)"
                        flush stdout
                        continue
                    }
                }
                set batch_ok 1
            }
            if {!$batch_ok && $recoveries < 3} {
                incr recoveries
                puts "> sd-write-fast: batch lba=$lba still bad after 3 attempts — JTAG-AXI recovery $recoveries/3"
                sdp_reprogram_current
            } else {
                break
            }
        }
        if {!$batch_ok} {
            close $fh
            error "sd-write-fast: batch failed after 3 attempts and $recoveries JTAG-AXI recoveries at lba=$lba — $batch_why"
        }

        incr sectors_done $nsec
        incr lba $nsec
        set dt [expr {[clock milliseconds] - $t0}]
        if {$dt > 0} {
            set rate [expr {double($sectors_done) * 512.0 / 1024.0 / ($dt / 1000.0)}]
            puts [format "> sd-write-fast: %d/%d sectors (%.0f KiB/s)" \
                      $sectors_done $total_sectors $rate]
            flush stdout
        }
    }
    close $fh
    return $sectors_done
}

# ── sd-verify — READ-ONLY scan of the card against a host image ──────────────
# Answers "do the card's contents still match the image we provisioned from?"
#
# WHY THIS EXISTS: the legacy `sd-write` path (sd_write_file, above) writes
# each sector and only polls a status bit — it NEVER reads anything back.  A
# card provisioned that way can hold silently corrupt sectors.  Crucially the
# CPU-side read CRC16 in sd_ctrl.v CANNOT detect this: the card computes its
# stored CRC over whatever bytes it actually received, so corrupt-but-
# consistent data reads back "clean" forever.  Only a comparison against the
# original host image can find it.
#
# This issues GO_VERIFY only (CMD18 read-back + hardware CRC32) and never
# GO_WRITE, so it cannot modify the card.  Staging BRAM is untouched except by
# sdp_calibrate_word_order, which is BRAM-local.
#
# Reports EVERY mismatching batch rather than stopping at the first, so one
# pass characterises the whole corruption pattern (isolated vs widespread).
proc sd_verify_file {start_lba path {max_sectors 0}} {
    if {![file exists $path]} { error "sd-verify: file not found: $path" }
    if {[catch {zlib crc32 ""}]} {
        error "sd-verify: this Tcl has no zlib crc32 — cannot verify"
    }
    set stage_sectors [sdp_probe]
    sdp_wait_card_ready
    sdp_calibrate_word_order

    set fsize [file size $path]
    set total_sectors [expr {($fsize + 511) / 512}]
    if {$max_sectors > 0 && $max_sectors < $total_sectors} {
        set total_sectors $max_sectors
    }
    set fh [open $path "rb"]
    fconfigure $fh -translation binary -encoding binary

    set lba $start_lba
    set done 0
    set bad 0
    set batch_bytes [expr {$stage_sectors * 512}]
    set t0 [clock milliseconds]

    while {$done < $total_sectors} {
        set want [expr {min($batch_bytes, ($total_sectors - $done) * 512)}]
        set chunk [read $fh $want]
        set n [string length $chunk]
        if {$n == 0} { break }
        set pad [expr {(512 - ($n % 512)) % 512}]
        if {$pad > 0} { append chunk [string repeat "\x00" $pad] }
        set nsec [expr {[string length $chunk] / 512}]

        wr [sdp_addr $::SDP_OFF_LBA]    $lba
        wr [sdp_addr $::SDP_OFF_BLKCNT] $nsec
        wr [sdp_addr $::SDP_OFF_CTRL]   $::SDP_CTRL_GO_VERIFY
        sdp_poll_done "CMD18 verify (lba=$lba n=$nsec)" 30000

        set expect [expr {[zlib crc32 $chunk] & 0xFFFFFFFF}]
        set got [sdp_rd $::SDP_OFF_CRC32]
        if {$got != $expect} {
            incr bad
            puts [format "> sd-verify: MISMATCH lba=%d..%d (card=0x%08X image=0x%08X)" \
                      $lba [expr {$lba + $nsec - 1}] $got $expect]
            flush stdout
        }

        incr done $nsec
        incr lba  $nsec
        set dt [expr {[clock milliseconds] - $t0}]
        if {$dt > 0 && ($done % (16 * $stage_sectors)) == 0} {
            puts [format "> sd-verify: %d/%d sectors, %d bad batches (%.0f KiB/s)" \
                      $done $total_sectors $bad \
                      [expr {double($done) * 512.0 / 1024.0 / ($dt / 1000.0)}]]
            flush stdout
        }
    }
    close $fh
    return [list $done $bad]
}

proc sd_fast_status {} {
    set ident [sdp_rd $::SDP_OFF_IDENT]
    if {$ident != $::SDP_IDENT} {
        return "bulk writer NOT present (ident=[format 0x%08X $ident]) — load build/sd_provision/sd_provision_top.bit"
    }
    set caps [sdp_rd $::SDP_OFF_CAPS]
    set st   [sdp_rd $::SDP_OFF_STATUS]
    set crc  [sdp_rd $::SDP_OFF_CRC32]
    set cause [expr {($st >> 4) & 0xF}]
    return [format "ident=OK caps: v%d stage_sectors=%d | busy=%d done=%d error=%d (%s) card_ready=%d init_error=%d | crc32=0x%08X" \
        [expr {($caps >> 16) & 0xFFFF}] [expr {$caps & 0xFFFF}] \
        [expr {$st & 1}] [expr {($st >> 1) & 1}] [expr {($st >> 2) & 1}] \
        [sdp_err_name $cause] \
        [expr {($st >> 8) & 1}] [expr {($st >> 9) & 1}] $crc]
}

proc dbg_rd {off}      { return [rd [expr {$::DBG_BASE + $off}] $::axi_dbg] }
proc dbg_wr {off data} { wr [expr {$::DBG_BASE + $off}] $data $::axi_dbg }

# Moved here (was a top-level call right after `axi master =` in earlier
# versions) because check_build_id calls dbg_rd, which isn't defined until
# the line above -- calling it any earlier threw "invalid command name
# dbg_rd" on every single REPL startup (caught, harmless, but noisy and
# masked the very first real diagnostic a user would want to see).
#
# JTAG_REPL_SKIP_BUILD_ID_CHECK=1 skips this -- temporary, for the
# first-read ILA capture experiment (2026-07-20) where the automatic
# build_id read would otherwise BE "read #1" before the operator can
# arm a trigger to observe it. Revert to always-on once that experiment
# is done; this env var is not meant to be a permanent feature.
if {![info exists ::env(JTAG_REPL_SKIP_BUILD_ID_CHECK)] || $::env(JTAG_REPL_SKIP_BUILD_ID_CHECK) ne "1"} {
    check_build_id
} else {
    puts "> jtag_repl: SKIPPING automatic build_id check (JTAG_REPL_SKIP_BUILD_ID_CHECK=1)"
}

# ══════════════════════════════════════════════════════════════════════
# rd_burst — bulk 32-bit reads in ONE JTAG round trip per burst
# ══════════════════════════════════════════════════════════════════════
# `dump-mem` used to issue one create_hw_axi_txn + run_hw_axi per word.
# Each of those is a full Vivado -> hw_server -> JTAG round trip, so 40
# longwords took ~30 s.  The JTAG-AXI master supports INCR bursts, so the
# same 40 words are one transaction.
#
# Returns a list of bare 8-hex-digit strings, one per word, in ASCENDING
# ADDRESS order.  On any failure the burst path is abandoned and the
# per-word path is used instead, so behaviour degrades to the old (slow,
# known-good) implementation rather than to wrong data.
#
# NOT HARDWARE-VALIDATED: written without board access.  The burst DATA
# property packs all beats into one hex string; which end of that string
# is the LOWEST address is a property of the hw_axi layer, so the result
# is self-checked against a single-word read of the first address before
# being trusted, and the whole burst is discarded if they disagree.
set ::rd_burst_ok 1
set ::rd_burst_order ""

# Pure helper (unit-tested): validate a 68040 SR value.  Bits 11 and 7:5
# are reserved-zero on the 68040 (implemented-bits mask 0xF71F, same as
# Musashi's CPU_SR_MASK).  A readback with any of them set is NOT a legal
# architectural state — it is a transport/unpack error or a pre-SR-mask
# bitstream — and must never be presented as trustworthy data (a live
# 0x3850 readback blocked a hardware verdict on ori.w #$0700,SR during
# the 0xD2DF5DD3 investigation).  Returns "" if valid, else a diagnosis.
proc sr_check_valid {sr} {
    set bad [expr {$sr & ~0xF71F}]
    if {$bad == 0} { return "" }
    return [format "INVALID: reserved SR bits 0x%04X set (impossible on a 68040 — stale/mis-unpacked readback or pre-SR-mask bitstream; DO NOT TRUST)" $bad]
}

# ══════════════════════════════════════════════════════════════════════
# A-trap breakpoint helpers (pure, unit-tested)
# ══════════════════════════════════════════════════════════════════════
# OFF_FEATURES bit 15 = atrap_bp, bit 16 = atrap_regcap (HIT_A0/HIT_D0 are
# real), bit 17 = atrap_d0qual (per-slot D0 qualifier).  See the `atrap`
# docstring near the top of this file for the command surface.

# Pure helper (unit-tested): the CTRL-register offset for a slot.  Slot 1's
# CTRL/MATCH/D0VAL sit exactly 0xC above slot 0's -- same fixed-stride
# layout idiom as the WP0/WP1 block.  Raises on anything but 0/1: a typo'd
# slot number silently operating on the wrong (or a nonexistent) slot is
# exactly the class of bug this whole feature exists to avoid.
proc atrap_ctrl_off {slot} {
    if {$slot != 0 && $slot != 1} { error "atrap: slot must be 0 or 1 (got '$slot')" }
    return [expr {$slot ? $::OFF_ATRAP1_CTRL : $::OFF_ATRAP0_CTRL}]
}

# Pure helper (unit-tested): validate an A-trap opcode <value>.  Must fit in
# 16 bits AND have opword[15:12] == 0xA -- the hardware match rule ANDs in
# that nibble unconditionally (see the OFF_ATRAP*_MATCH comment above), so a
# value with any other top nibble can NEVER fire.  A silently-never-firing
# breakpoint is exactly the failure mode this helper exists to prevent.
proc atrap_check_value {value} {
    if {$value < 0 || $value > 0xFFFF} {
        error [format "atrap: value 0x%X out of range -- must be a 16-bit opcode word (0..0xFFFF)" $value]
    }
    if {(($value >> 12) & 0xF) != 0xA} {
        error [format "atrap: value 0x%04X is not an A-line opcode (opword\[15:12\] must be 0xA) -- this breakpoint could never fire" $value]
    }
    return $value
}

# Pure helper (unit-tested): validate an A-trap <mask>.  16-bit only --
# OFF_ATRAP*_MATCH packs it into bits[31:16] of one register alongside the
# 16-bit value.
proc atrap_check_mask {mask} {
    if {$mask < 0 || $mask > 0xFFFF} {
        error [format "atrap: mask 0x%X out of range -- must be a 16-bit mask (0..0xFFFF)" $mask]
    }
    return $mask
}

# Pure helper (unit-tested): pack {value, mask} into the OFF_ATRAP*_MATCH
# register layout -- {mask[15:0] in bits[31:16], value[15:0] in bits[15:0]}.
# NOTE mask polarity here is the OPPOSITE of the WPn_AMASK watchpoint
# convention (there 1 = ignore; here 1 = CARE/compare this bit) -- do not
# copy that convention across.
proc atrap_pack_match {value mask} {
    return [expr {(($mask & 0xFFFF) << 16) | ($value & 0xFFFF)}]
}

# Pure helper (unit-tested): inverse of atrap_pack_match.  Returns a
# two-element list {value mask}.
proc atrap_unpack_match {packed} {
    set value [expr {$packed & 0xFFFF}]
    set mask  [expr {($packed >> 16) & 0xFFFF}]
    return [list $value $mask]
}

# Write one A-trap CSR and verify the readback matches before returning.
# This project has burned multi-hour investigations on host tools that
# silently trusted an unverified write, so every atrap arm/disarm write
# goes through this instead of a bare `wr`/`dbg_wr`.  Raises with BOTH the
# wanted and observed value on mismatch -- never returns having silently
# written the wrong thing.  Self-contained (only depends on `wr`, `rdx` and
# $::DBG_BASE) so it is unit-testable the same way dbg_rd/dbg_wr are used
# elsewhere, without needing dbg_wr's own (non-extractable, one-line) body.
proc atrap_wr_verify {off data {what "atrap register"}} {
    wr [expr {$::DBG_BASE + $off}] $data
    set want [expr {$data & 0xFFFFFFFF}]
    set got  [rdx [expr {$::DBG_BASE + $off}]]
    if {$got != $want} {
        error [format "%s write verify FAILED at offset 0x%03X: wrote 0x%08X, read back 0x%08X" \
                   $what $off $want $got]
    }
    return $got
}

proc rd_burst_slow {addr n} {
    set out {}
    for {set i 0} {$i < $n} {incr i} {
        lappend out [rd [expr {$addr + $i*4}]]
    }
    return $out
}

# Pure helper (unit-tested): 1 if every word in the list equals the first.
# A multi-beat burst whose words are ALL identical is the signature of a
# FIXED (non-incrementing) burst re-reading one address — hardware-proven:
# `dump-mem 0x0002e8a0 6` returned 0x0098205f six times while single-word
# reads of the same range differed.  Such a burst must never be returned
# as data without independent confirmation.
proc rd_burst_all_identical {words} {
    set w0 [lindex $words 0]
    foreach w $words {
        if {![string equal -nocase $w $w0]} { return 0 }
    }
    return 1
}

proc rd_burst {addr n} {
    if {$n <= 1 || !$::rd_burst_ok} { return [rd_burst_slow $addr $n] }

    set out {}
    set remaining $n
    set cur $addr
    while {$remaining > 0} {
        set beats [expr {$remaining > 256 ? 256 : $remaining}]
        set chunk ""
        if {[catch {
            # -burst INCR is EXPLICIT: relying on the IP default produced
            # a non-incrementing burst on hardware (same latched word for
            # every address in range — see rd_burst_all_identical above).
            create_hw_axi_txn -quiet -force _rb $::axi -type READ \
                -address [format %08X $cur] -len $beats -burst INCR
            run_hw_axi -quiet _rb
            set chunk [get_property DATA [get_hw_axi_txns _rb]]
        } err]} {
            puts "> jtag_repl: rd_burst failed ($err) -- falling back to per-word reads for this session"
            set ::rd_burst_ok 0
            return [rd_burst_slow $addr $n]
        }
        if {$chunk eq "" || ![string is xdigit -strict $chunk] ||
            [string length $chunk] != [expr {$beats * 8}]} {
            puts "> jtag_repl: rd_burst returned [string length $chunk] hex chars, expected [expr {$beats*8}] -- falling back to per-word reads for this session"
            set ::rd_burst_ok 0
            return [rd_burst_slow $addr $n]
        }
        # Split into 8-char words.
        set words {}
        for {set i 0} {$i < $beats} {incr i} {
            lappend words [string range $chunk [expr {$i*8}] [expr {$i*8+7}]]
        }
        # Determine (once) which end of the string is the lowest address by
        # cross-checking against a single-word read.  A guess here would be
        # exactly the class of silent fabrication this whole change is
        # about, so it is measured, not assumed.
        if {$::rd_burst_order eq ""} {
            set probe [rd $cur]
            if {$probe eq "BADA0BAD"} {
                set ::rd_burst_ok 0
                return [rd_burst_slow $addr $n]
            }
            if {[string equal -nocase $probe [lindex $words 0]]} {
                set ::rd_burst_order fwd
            } elseif {[string equal -nocase $probe [lindex $words end]]} {
                set ::rd_burst_order rev
            } else {
                puts "> jtag_repl: rd_burst word-order probe inconclusive (single-word read 0x$probe matches neither end of the burst) -- falling back to per-word reads for this session"
                set ::rd_burst_ok 0
                return [rd_burst_slow $addr $n]
            }
            puts "> jtag_repl: rd_burst word order = $::rd_burst_order"
        }
        if {$::rd_burst_order eq "rev"} { set words [lreverse $words] }
        # All-identical guard: if every beat returned the same word, the
        # single-word order probe above is inconclusive by construction
        # (both ends match), so it CANNOT catch a non-incrementing burst.
        # Cross-check the second address independently; on mismatch, fail
        # LOUDLY and fall back to per-word reads — never return a burst
        # that repeats one latched value across a varying range.
        if {$beats > 1 && [rd_burst_all_identical $words]} {
            set probe2 [rd [expr {$cur + 4}]]
            if {$probe2 eq "BADA0BAD" ||
                ![string equal -nocase $probe2 [lindex $words 1]]} {
                puts "> jtag_repl: rd_burst returned ONE word repeated ${beats}x but addr+4 reads 0x$probe2 -- burst path is NOT incrementing; falling back to per-word reads for this session"
                set ::rd_burst_ok 0
                return [rd_burst_slow $addr $n]
            }
        }
        set out [concat $out $words]
        set cur [expr {$cur + $beats*4}]
        set remaining [expr {$remaining - $beats}]
    }
    return $out
}

# ══════════════════════════════════════════════════════════════════════
# vHDD — DDR-backed RAM-disk SCSI volume
# ══════════════════════════════════════════════════════════════════════
#
# Two pieces of hardware surface, both reachable with the ordinary `r`/`w`
# AXI primitives:
#
#   1. A control block at VHDD_BASE (AXI-Lite).  Presence is proved by
#      IDENT == 0x5D0D0001 and NOTHING ELSE -- the surrounding window is a
#      null slave that reads 0 for every offset, and 0 is a plausible value
#      for CTRL / RD_BLOCKS / STATUS / RD_APERTURE alike.  Every command
#      below therefore probes IDENT first and refuses (or warns loudly)
#      rather than decoding fiction.
#
#   2. A plain AXI memory aperture whose base is READ FROM RD_APERTURE, never
#      hardcoded, so a future rebase of the aperture cannot silently send a
#      32 MB upload into whatever now lives at the old address.  Volume LBA L
#      byte i lives at aperture + L*512 + i, ascending: byte i at base+i.
#      There is no doorbell and no descriptor ring -- upload/download is an
#      AXI block copy.
#
# THROUGHPUT / BURSTING.  A per-word round trip over JTAG is unusably slow
# (a 32 MB upload would take hours), so the bulk paths use 256-beat AXI
# bursts, the same shape `sd-write-fast` uses to reach ~264 KiB/s.  Two
# hazards come with that, and both have bitten this file before:
#
#   * Vivado's hw_axi -data string may be consumed lowest-address-first or
#     highest-address-first depending on version.  MEASURED, never assumed
#     (vhdd_calibrate_wr_order), against a probe that is saved and restored
#     word-by-word so calibration is non-destructive.
#   * A burst that is not INCR re-writes/re-reads ONE address for every beat.
#     `-burst INCR` is passed explicitly; the read path additionally
#     spot-checks interior beats against single-word reads, because
#     rd_burst's own fwd/rev probe only inspects the two END words and so
#     cannot see an interior scramble (the historical `dump-mem` 0,4,3,2,1,5
#     bug, whose self-check was blind to exactly this).
#
# An AXI INCR burst may not cross a 4 KiB boundary; vhdd_burst_beats enforces
# that in addition to the 256-beat cap.

proc vhdd_addr {off} { return [expr {$::VHDD_BASE + $off}] }

# Read one control register as an integer.  Never returns the BADA0BAD
# sentinel as if it were data.
proc vhdd_rdreg {off} {
    set v [rdx_or_empty [vhdd_addr $off]]
    if {$v eq ""} {
        error [format "vhdd: AXI read FAILED at 0x%08X (control reg +0x%03X) -- refusing to return a fabricated value" [vhdd_addr $off] $off]
    }
    return $v
}

proc vhdd_wrreg {off data} { wr [vhdd_addr $off] [expr {$data & 0xFFFFFFFF}] }


# ──────────────────────────────────────────────────────────────────────────────
# net-vHDD endpoint  (vhdd_ctrl.v 0x040..0x058; docs/net_vhdd_design.md)
# ──────────────────────────────────────────────────────────────────────────────
# The Ethernet-backed volume does no ARP, so BOTH endpoints are configured
# rather than discovered, and the destination is the NEXT HOP -- point it at
# the gateway and UDP still routes off-segment.
#
# our-MAC of 00:00:00:00:00:00 means the block client is ABSENT: the MAC
# sharing demux excludes an all-zero address from both exact match and
# broadcast fan-out, which is what keeps an unconfigured board from claiming
# the SONIC's frames.  That is the power-on state, so this command is what
# brings the volume to life.
set VHDD_OFF_NET_MAC_LO     0x040
set VHDD_OFF_NET_MAC_HI     0x044
set VHDD_OFF_NET_IP         0x048
set VHDD_OFF_NET_DST_MAC_LO 0x04C
set VHDD_OFF_NET_DST_MAC_HI 0x050
set VHDD_OFF_NET_DST_IP     0x054
set VHDD_OFF_NET_PORTS      0x058

proc net_parse_mac {text} {
    set clean [string map {: "" - "" . ""} $text]
    if {![regexp {^[0-9a-fA-F]{12}$} $clean]} {
        error "bad MAC '$text' (want aa:bb:cc:dd:ee:ff)"
    }
    return [expr {"0x$clean"}]
}
proc net_parse_ip {text} {
    set parts [split $text .]
    if {[llength $parts] != 4} { error "bad IPv4 '$text'" }
    set v 0
    foreach o $parts {
        if {![string is integer -strict $o] || $o < 0 || $o > 255} {
            error "bad IPv4 octet '$o' in '$text'"
        }
        set v [expr {($v << 8) | $o}]
    }
    return $v
}
proc net_fmt_mac {v} {
    set out {}
    for {set i 5} {$i >= 0} {incr i -1} {
        lappend out [format %02X [expr {($v >> ($i*8)) & 0xFF}]]
    }
    return [join $out :]
}
proc net_fmt_ip {v} {
    return [format "%d.%d.%d.%d" [expr {($v>>24)&0xFF}] [expr {($v>>16)&0xFF}] \
                                 [expr {($v>>8)&0xFF}]  [expr {$v&0xFF}]]
}

proc vhdd_net_show {} {
    set mac [expr {([vhdd_rdreg $::VHDD_OFF_NET_MAC_HI] & 0xFFFF) << 32 |
                    [vhdd_rdreg $::VHDD_OFF_NET_MAC_LO]}]
    set dmac [expr {([vhdd_rdreg $::VHDD_OFF_NET_DST_MAC_HI] & 0xFFFF) << 32 |
                     [vhdd_rdreg $::VHDD_OFF_NET_DST_MAC_LO]}]
    set ip   [vhdd_rdreg $::VHDD_OFF_NET_IP]
    set dip  [vhdd_rdreg $::VHDD_OFF_NET_DST_IP]
    set pp   [vhdd_rdreg $::VHDD_OFF_NET_PORTS]
    puts [format "> vhdd-net: ours %s  %s:%d" [net_fmt_mac $mac] [net_fmt_ip $ip] \
              [expr {($pp >> 16) & 0xFFFF}]]
    puts [format "> vhdd-net: peer %s  %s:%d" [net_fmt_mac $dmac] [net_fmt_ip $dip] \
              [expr {$pp & 0xFFFF}]]
    if {$mac == 0} {
        puts "> vhdd-net: our MAC is zero -- the block client is ABSENT and will not receive"
    }
    puts "> vhdd-net: the host needs a static neighbour entry, or its replies are dropped:"
    puts [format "> vhdd-net:   ip neigh replace %s lladdr %s dev <iface> nud permanent" \
              [net_fmt_ip $ip] [net_fmt_mac $mac]]
}

proc vhdd_net_set {our_mac our_ip our_port dst_mac dst_ip dst_port} {
    set m  [net_parse_mac $our_mac]
    set dm [net_parse_mac $dst_mac]
    set i  [net_parse_ip  $our_ip]
    set di [net_parse_ip  $dst_ip]
    vhdd_wrreg $::VHDD_OFF_NET_IP         $i
    vhdd_wrreg $::VHDD_OFF_NET_DST_MAC_LO [expr {$dm & 0xFFFFFFFF}]
    vhdd_wrreg $::VHDD_OFF_NET_DST_MAC_HI [expr {($dm >> 32) & 0xFFFF}]
    vhdd_wrreg $::VHDD_OFF_NET_DST_IP     $di
    vhdd_wrreg $::VHDD_OFF_NET_PORTS \
        [expr {(($our_port & 0xFFFF) << 16) | ($dst_port & 0xFFFF)}]
    # our MAC LAST: it is the enable.  Writing it first would arm the demux
    # while the rest of the endpoint is still half-configured, so the client
    # would start claiming frames it cannot yet answer.
    vhdd_wrreg $::VHDD_OFF_NET_MAC_LO [expr {$m & 0xFFFFFFFF}]
    vhdd_wrreg $::VHDD_OFF_NET_MAC_HI [expr {($m >> 32) & 0xFFFF}]
    vhdd_net_show
}

# ──────────────────────────────────────────────────────────────────────────────
# SONIC register-access trace ring  (rtl/soc/sonic_trace_ring.v)
# ──────────────────────────────────────────────────────────────────────────────
# Readout registers live in the ETH_DEBUG page (eth_debug_regs.sv):
#   +0x80 TRACE_CTRL  W: bit0 freeze, bit1 clear+re-arm
#                     R: {frozen[17], wrapped[16], wr_ptr[11:0]}
#   +0x84 TRACE_ADDR  ring index for the next TRACE_DATA read
#   +0x88 TRACE_DATA  ring[TRACE_ADDR]  (does NOT auto-increment)
#   +0x8c TRACE_FILT  live suppressed-poll total (valid WITHOUT freezing)
#
# WHY THIS EXISTS: every remaining Ethernet question is a SEQUENCE question
# -- did the driver ever write IMR, or write it and have it cleared; what
# went into CAM entry 15; does CR_TXP stick set after ~8 transmits -- and a
# post-mortem register snapshot structurally cannot answer any of them.  The
# ring is a passive observer inside the SoC, so this command reads a
# RECORDING, never the live chip.
#
# The dump runs entirely inside this proc -- one Vivado JTAG-AXI transaction
# per access, no per-command REPL round trip -- so 4096 entries take tens of
# seconds rather than the ~68 minutes a one-command-per-entry loop would.
set ETH_OFF_TRACE_CTRL 0x80
set ETH_OFF_TRACE_ADDR 0x84
set ETH_OFF_TRACE_DATA 0x88
set ETH_OFF_TRACE_FILT 0x8c
set ETH_TRACE_DEPTH    4096

# Presence probe.  IDENT alone is not enough: an older ETH_DEBUG page has the
# same IDENT and answers 0 at 0x80..0x8c, which would decode as a perfectly
# plausible empty, never-frozen ring.  CAPS bit 7 is set only by a page built
# with the trace ring wired in.
proc sonic_trace_probe {} {
    eth_debug_probe
    set caps [eth_debug_rd 0x04]
    if {($caps & 0x80) == 0} {
        error [format "sonic-trace: CAPS=0x%08X has bit 7 clear -- this ETH_DEBUG page has no SONIC trace ring wired in.  Its 0x80..0x8c would read back 0, which decodes as an empty never-frozen ring; refusing to report that as a measurement.  Rebuild with rtl/soc/sonic_trace_ring.v instantiated (ETH_DEBUG_ENABLE=1)." $caps]
    }
    return $caps
}

# Returns {wr_ptr wrapped frozen}
proc sonic_trace_status {} {
    set c [eth_debug_rd $::ETH_OFF_TRACE_CTRL]
    return [list [expr {$c & 0xFFF}] [expr {($c >> 16) & 1}] [expr {($c >> 17) & 1}]]
}

proc sonic_trace_filtered {} {
    return [expr {[eth_debug_rd $::ETH_OFF_TRACE_FILT] & 0xFFFF}]
}

# Read one entry.  TRACE_DATA does NOT auto-increment: the RAM read is
# registered, so the host must write the address and then read the data.
proc sonic_trace_entry {idx} {
    eth_debug_wr $::ETH_OFF_TRACE_ADDR $idx
    set v [rdx_or_empty [expr {$::ETH_DEBUG_BASE + $::ETH_OFF_TRACE_DATA}]]
    if {$v eq ""} {
        error [format "sonic-trace: AXI read FAILED at entry %d -- refusing to fabricate a value" $idx]
    }
    return $v
}

# SONIC register names, for the trailing comment column only.  The four
# machine-parsed columns never depend on this table.
array set ::SONIC_REGNAME {
    0x00 CR   0x01 DCR  0x02 RCR  0x03 TCR  0x04 IMR  0x05 ISR
    0x06 UTDA 0x07 CTDA 0x08 TPS  0x09 TFC  0x0a TSA0 0x0b TSA1 0x0c TFS
    0x0d URDA 0x0e CRDA 0x0f CRBA0 0x10 CRBA1 0x11 RBWC0 0x12 RBWC1
    0x13 EOBC 0x14 URRA 0x15 RSA  0x16 REA  0x17 RRP  0x18 RWP
    0x19 TRBA0 0x1a TRBA1 0x1b TBWC0 0x1c TBWC1 0x1d ADDR0 0x1e ADDR1
    0x1f LLFA 0x20 TTDA 0x21 WT0  0x22 WT1  0x23 RSC0 0x24 RSC1
    0x25 CE   0x26 CDP  0x27 CDC  0x28 SR   0x29 WT0b 0x2a WT1b
    0x2b RSC  0x2c CRCT 0x2d FAET 0x2e MPT  0x2f MDT  0x3f DCR2
}
proc sonic_regname {r} {
    set k [format "0x%02x" $r]
    if {[info exists ::SONIC_REGNAME($k)]} { return $::SONIC_REGNAME($k) }
    return [format "r%02x" $r]
}

# Dumps the ring as a CSV whose first four columns are the same
# (idx, rw, reg, data) shape tools/scsi96_trace_diff.py already consumes, so
# the same differ style works against a MAME capture of this driver.
proc sonic_trace_dump {path} {
    sonic_trace_probe

    # POSITIVE CONTROL for "the ring is quiescent".  Read wr_ptr AND the
    # suppressed-poll total, wait, read both again.  Two separate facts:
    #   * wr_ptr moved      -> the driver is doing NEW things; freezing now
    #                          captures a moving window.
    #   * wr_ptr static but filtered climbing -> the driver is SPINNING on a
    #                          constant register.  That is the wedge, and the
    #                          pre-wedge window is intact.
    #   * both static       -> the driver has stopped touching the SONIC at
    #                          all.  A different diagnosis entirely.
    # A dump that could not tell those apart would prove nothing about which
    # state we are in.
    lassign [sonic_trace_status] p0 w0 f0
    set q0 [sonic_trace_filtered]
    after 1500
    lassign [sonic_trace_status] p1 w1 f1
    set q1 [sonic_trace_filtered]
    if {$f0} {
        puts "> sonic-trace: ring was ALREADY frozen (wr_ptr=$p0) -- dumping the existing capture"
    } elseif {$p0 != $p1} {
        puts "> sonic-trace: ***** ring is STILL ADVANCING (wr_ptr $p0 -> $p1 over 1.5 s)."
        puts "> sonic-trace:       The driver is issuing NEW register accesses, so this is"
        puts "> sonic-trace:       NOT a quiescent wedge.  Freezing anyway, but the window"
        puts "> sonic-trace:       is a moving one -- treat it accordingly."
    } elseif {$q0 != $q1} {
        puts "> sonic-trace: ring QUIESCENT (wr_ptr stable at $p0) but the poll filter is"
        puts "> sonic-trace:       still dropping reads (filtered $q0 -> $q1 over 1.5 s)."
        puts "> sonic-trace:       The driver IS still running and SPINNING on a constant"
        puts "> sonic-trace:       register -- consistent with a wedge, pre-wedge window intact."
    } else {
        puts "> sonic-trace: ring QUIESCENT and poll filter idle (wr_ptr=$p0 filtered=$q0"
        puts "> sonic-trace:       both stable over 1.5 s) -- the driver has stopped touching"
        puts "> sonic-trace:       the SONIC entirely, which is NOT the same as spinning on it."
    }

    # Freeze, then confirm the DUT agrees it is frozen before reading.
    eth_debug_wr $::ETH_OFF_TRACE_CTRL 0x1
    after 100
    lassign [sonic_trace_status] wp wrapped frozen
    if {!$frozen} {
        error "sonic-trace: freeze did not take (TRACE_CTRL.frozen still 0) -- refusing to dump a live ring"
    }
    set filt [sonic_trace_filtered]

    # Oldest-first ordering: if the ring wrapped, the oldest entry is the one
    # wr_ptr is about to overwrite.
    set depth $::ETH_TRACE_DEPTH
    if {$wrapped} { set n $depth; set start $wp } else { set n $wp; set start 0 }
    puts [format "> sonic-trace: wr_ptr=%d wrapped=%d filtered=%d -> dumping %d entries" \
              $wp $wrapped $filt $n]

    set fh [open $path w]
    puts $fh "# sonic_trace_ring dump — idx,rw,reg,data,strb,skipped,name"
    puts $fh "#   rw      R = CPU read, W = CPU write"
    puts $fh "#   reg     SONIC register index, hex (0x00..0x3f)"
    puts $fh "#   data    16-bit word transferred, hex"
    puts $fh "#   strb    byte strobes; writes carry the real sonic_wstrb,"
    puts $fh "#           reads always 3 (the SONIC returns the whole register)"
    puts $fh "#   skipped poll-filtered reads dropped just before this entry"
    puts $fh "#           (saturating; 127 means >= 127)"
    puts $fh [format "# wr_ptr=%d wrapped=%d depth=%d filtered_total=%d" \
                  $wp $wrapped $depth $filt]
    set t0 [clock milliseconds]
    for {set i 0} {$i < $n} {incr i} {
        set idx [expr {($start + $i) % $depth}]
        set e [sonic_trace_entry $idx]
        set rw   [expr {($e >> 31) & 1}]
        set strb [expr {($e >> 29) & 0x3}]
        set reg  [expr {($e >> 23) & 0x3F}]
        set data [expr {($e >> 7) & 0xFFFF}]
        set skip [expr {$e & 0x7F}]
        set rs   [expr {$rw ? "W" : "R"}]
        puts $fh [format "%d,%s,%02x,%04x,%d,%d,%s" \
                      $i $rs $reg $data $strb $skip [sonic_regname $reg]]
    }
    close $fh
    set dt [expr {([clock milliseconds] - $t0) / 1000.0}]
    puts [format "> sonic-trace: wrote %d entries to %s in %.1f s" $n $path $dt]
    puts "> sonic-trace: ring is still FROZEN — run `sonic-trace rearm` to resume capture"
}

# ──────────────────────────────────────────────────────────────────────────────
# 53C96 register-access trace ring  (rtl/soc/scsi_trace_ring.v)
# ──────────────────────────────────────────────────────────────────────────────
# ── Optional SONIC/Taxi telemetry page ──────────────────────────────────────
proc eth_debug_rd {off} {
    set v [rdx_or_empty [expr {$::ETH_DEBUG_BASE + $off}]]
    if {$v eq ""} {
        error [format "eth-status: AXI read failed at 0x%08X" [expr {$::ETH_DEBUG_BASE + $off}]]
    }
    return $v
}

proc eth_debug_wr {off data} {
    wr [expr {$::ETH_DEBUG_BASE + $off}] [expr {$data & 0xFFFFFFFF}]
}

proc eth_promisc_set {enable} {
    eth_debug_probe
    set v [expr {$enable ? 1 : 0}]
    eth_debug_wr 0x70 $v
    puts [format "> eth-promisc: %s" [expr {$v ? "enabled" : "disabled"}]]
}

proc eth_promisc_status {} {
    eth_debug_probe
    set v [expr {[eth_debug_rd 0x70] & 1}]
    puts [format "> eth-promisc: %s" [expr {$v ? "enabled" : "disabled"}]]
}

proc eth_debug_probe {} {
    set id [eth_debug_rd 0x00]
    if {$id != $::ETH_DEBUG_IDENT} {
        error [format "eth-status: IDENT=0x%08X, expected 0x%08X -- this bitstream was not built with ETH_DEBUG_ENABLE=1 (or does not contain the SONIC DMA datapath)" $id $::ETH_DEBUG_IDENT]
    }
    return $id
}

proc eth_debug_pair {value hi_name lo_name} {
    puts [format "> eth-status: %-19s %5d   %-19s %5d" \
              $hi_name [expr {($value >> 16) & 0xFFFF}] \
              $lo_name [expr {$value & 0xFFFF}]]
}

proc eth_status_report {} {
    set id [eth_debug_probe]
    set caps [eth_debug_rd 0x04]
    set st [eth_debug_rd 0x08]
    set cr_imr [eth_debug_rd 0x0C]
    set isr [eth_debug_rd 0x10]
    set txs [eth_debug_rd 0x14]
    set rxs [eth_debug_rd 0x18]
    set tx_desc [eth_debug_rd 0x1C]
    set rx_desc [eth_debug_rd 0x20]
    set lens [eth_debug_rd 0x24]
    set dma_addr [eth_debug_rd 0x28]
    set dma_meta [eth_debug_rd 0x2C]
    set err_info [eth_debug_rd 0x30]
    set err_addr [eth_debug_rd 0x34]

    puts [format "> eth-status: ident=0x%08X caps=0x%08X link_speed=%d irq=%d rx_enable=%d first_error=%d" \
              $id $caps [expr {$st & 3}] [expr {($st >> 2) & 1}] \
              [expr {($st >> 3) & 1}] [expr {($st >> 15) & 1}]]
    puts [format "> eth-status: SONIC CR=0x%04X IMR=0x%04X ISR=0x%04X  TX state=0x%X RX state=0x%X" \
              [expr {($cr_imr >> 16) & 0xFFFF}] [expr {$cr_imr & 0xFFFF}] \
              [expr {($isr >> 16) & 0xFFFF}] [expr {$txs & 0xF}] [expr {$rxs & 0x1F}]]
    # caps bit 6 gates the receive-admission state.  A frame the MAC validated
    # is dropped silently unless RCR or the CAM admits it, which presents as
    # "RX does not work" with no DMA activity to explain it.
    if {($caps & 0x40) != 0} {
        set f [eth_debug_rd 0x74]
        set rcr [expr {($f >> 16) & 0xFFFF}]
        set came [expr {$f & 0xFFFF}]
        set m_lo [eth_debug_rd 0x78]
        set m_hi [expr {[eth_debug_rd 0x7C] & 0xFFFF}]
        set terms {}
        if {($rcr & 0x1000) != 0} { lappend terms PRO }
        if {($rcr & 0x2000) != 0} { lappend terms BRD }
        if {($rcr & 0x0800) != 0} { lappend terms AMC }
        if {$came != 0}           { lappend terms CAM }
        if {[llength $terms] == 0} { set terms {NONE-all-frames-dropped} }
        puts [format "> eth-status: SONIC RCR=0x%04X cam_enable=0x%04X accepts=%s" \
                  $rcr $came [join $terms +]]
        # caps bit 8 adds the entry-select at 0x5c.  Without it only entry 0
        # is visible, and the driver loads the Mac's own MAC into entry 15 --
        # so on an older debug page this print says nothing about the address
        # unicast filtering actually compares against.  Dump every ENABLED
        # entry, and say plainly when none are.
        if {($caps & 0x100) != 0} {
            set saved [eth_debug_rd 0x5c]
            set shown 0
            for {set i 0} {$i < 16} {incr i} {
                if {($came & (1 << $i)) == 0} { continue }
                eth_debug_wr 0x5c $i
                set lo [eth_debug_rd 0x78]
                set hi [expr {[eth_debug_rd 0x7C] & 0xFFFF}]
                puts [format "> eth-status: CAM\[%2d\]=%02X:%02X:%02X:%02X:%02X:%02X (enabled)" \
                          $i [expr {($hi >> 8) & 0xFF}] [expr {$hi & 0xFF}] \
                          [expr {($lo >> 24) & 0xFF}] [expr {($lo >> 16) & 0xFF}] \
                          [expr {($lo >> 8) & 0xFF}] [expr {$lo & 0xFF}]]
                incr shown
            }
            eth_debug_wr 0x5c [expr {$saved & 0xF}]
            if {$shown == 0} {
                puts "> eth-status: CAM has no enabled entries -- only broadcast/multicast/promiscuous can be accepted"
            }
        } else {
            puts [format "> eth-status: CAM\[0\]=%02X:%02X:%02X:%02X:%02X:%02X (entry 0 only; this bitstream has no CAM select)" \
                      [expr {($m_hi >> 8) & 0xFF}] [expr {$m_hi & 0xFF}] \
                      [expr {($m_lo >> 24) & 0xFF}] [expr {($m_lo >> 16) & 0xFF}] \
                      [expr {($m_lo >> 8) & 0xFF}] [expr {$m_lo & 0xFF}]]
        }
    }
    # caps bit 5 gates the DCR readback; older debug pages leave 0x3c absent.
    if {($caps & 0x20) != 0} {
        set dcr [expr {[eth_debug_rd 0x3C] & 0xFFFF}]
        puts [format "> eth-status: SONIC DCR=0x%04X  descriptors=%s (DW=%d)" \
                  $dcr [expr {($dcr & 0x20) ? "32-bit" : "16-bit"}] \
                  [expr {($dcr >> 5) & 1}]]
    }
    puts [format "> eth-status: descriptors TX=0x%08X RX=0x%08X  last lengths TX=%d RX=%d  live RX=%d" \
              $tx_desc $rx_desc [expr {($lens >> 16) & 0xFFFF}] \
              [expr {$lens & 0xFFFF}] [expr {($rxs >> 5) & 0xFFFF}]]
    puts [format "> eth-status: last DMA addr=0x%08X meta=0x%08X  first error info=0x%08X addr=0x%08X" \
              $dma_addr $dma_meta $err_info $err_addr]
    eth_debug_pair [eth_debug_rd 0x40] tx_commands tx_frames
    eth_debug_pair [eth_debug_rd 0x44] tx_completions tx_errors
    eth_debug_pair [eth_debug_rd 0x48] rx_frames rx_axis_errors
    eth_debug_pair [eth_debug_rd 0x4C] rx_completions rx_errors
    eth_debug_pair [eth_debug_rd 0x50] dma_tx_requests dma_rx_requests
    eth_debug_pair [eth_debug_rd 0x54] dma_tx_responses dma_rx_responses
    eth_debug_pair [eth_debug_rd 0x58] dma_tx_errors dma_rx_errors
    eth_debug_pair [eth_debug_rd 0x60] mac_tx_underflow mac_tx_overflow
    eth_debug_pair [eth_debug_rd 0x64] mac_tx_bad mac_tx_good
    eth_debug_pair [eth_debug_rd 0x68] mac_rx_bad mac_rx_bad_fcs
    eth_debug_pair [eth_debug_rd 0x6C] mac_rx_overflow mac_rx_good
}

proc eth_debug_clear {} {
    eth_debug_probe
    wr [expr {$::ETH_DEBUG_BASE + 0x08}] 1
    puts "> eth-clear: counters and sticky first-error snapshot cleared"
}

# Dumps the ring as a CSV directly comparable with the MAME golden capture
# from tools/mame_scsi96_capture.lua, so tools/scsi96_trace_diff.py can find
# the first divergence.
#
# WHY THIS EXISTS: the live 53C96 cannot be read over JTAG without changing
# it (reg 2 read pops the FIFO, reg 5 read clears the pending IRQ).  The ring
# is a passive observer in the SoC, so this command reads a RECORDING, never
# the chip.
#
# The dump runs entirely inside this proc — one Vivado JTAG-AXI transaction
# per access, no per-command REPL round trip — so 4096 entries take tens of
# seconds rather than the ~68 minutes a one-command-per-entry loop would.
set VHDD_OFF_TRACE_CTRL 0x020
set VHDD_OFF_TRACE_ADDR 0x024
set VHDD_OFF_TRACE_DATA 0x028
set VHDD_TRACE_DEPTH    4096

proc scsi_trace_status {} {
    set c [vhdd_rdreg $::VHDD_OFF_TRACE_CTRL]
    return [list [expr {$c & 0xFFF}] [expr {($c >> 16) & 1}] [expr {($c >> 17) & 1}]]
}

# Read one entry.  TRACE_DATA does NOT auto-increment: the RAM read is
# registered, so the host must write the address and then read the data.
proc scsi_trace_entry {idx} {
    vhdd_wrreg $::VHDD_OFF_TRACE_ADDR $idx
    set v [rdx_or_empty [vhdd_addr $::VHDD_OFF_TRACE_DATA]]
    if {$v eq ""} {
        error [format "scsi-trace: AXI read FAILED at entry %d -- refusing to fabricate a value" $idx]
    }
    return $v
}

proc scsi_trace_dump {path} {
    vhdd_probe 1

    # POSITIVE CONTROL for "the ring is quiescent".  Read wr_ptr, wait, read
    # it again.  If it moved, the SCSI bus is still active and freezing now
    # would capture a moving window -- say so rather than silently dumping a
    # torn trace.  This is the check that distinguishes "wedged" from "still
    # working"; without it a dump proves nothing about which state we are in.
    lassign [scsi_trace_status] p0 w0 f0
    after 1500
    lassign [scsi_trace_status] p1 w1 f1
    if {$f0} {
        puts "> scsi-trace: ring was ALREADY frozen (wr_ptr=$p0) -- dumping the existing capture"
    } elseif {$p0 != $p1} {
        puts "> scsi-trace: ***** ring is STILL ADVANCING (wr_ptr $p0 -> $p1 over 1.5 s)."
        puts "> scsi-trace:       The SCSI bus is active, so this is NOT a quiescent wedge."
        puts "> scsi-trace:       Freezing anyway, but the window is a moving one -- treat it accordingly."
    } elseif {$p0 == 0 && $w0 == 0 && $f0 == 0 && $p1 == 0} {
        # 2026-09-06: the ring is NOT instantiated in the current SoC build.
        # fpga_top_peripherals.vh (~:1947) ties the readouts to constants:
        #   scsi_trace_{rd_data,wrptr,wrapped,frozen} = 0
        # so a stubbed-out ring is INDISTINGUISHABLE from an idle one by
        # value alone.  Reporting that as "QUIESCENT -- consistent with a
        # wedge" turned absence-of-HARDWARE into apparent absence-of-TRAFFIC
        # and produced a wrong published conclusion ("the CPU is not touching
        # the 53C96") while the CPU was in fact polling it hard.
        puts "> scsi-trace: readouts are ALL ZERO (wr_ptr/wrapped/frozen)."
        puts "> scsi-trace: ***** This is almost certainly a bitstream with NO trace ring"
        puts "> scsi-trace:       instantiated -- fpga_top_peripherals.vh ties these to 0."
        puts "> scsi-trace:       It is NOT evidence that the SCSI bus is idle, and NOT"
        puts "> scsi-trace:       evidence of a wedge.  Draw no conclusion from it."
        puts "> scsi-trace:       To get real tracing, re-instantiate scsi_trace_ring.v"
        puts "> scsi-trace:       (kept in-tree; tb-scsi-trace-ring still passes)."
    } else {
        puts "> scsi-trace: ring QUIESCENT (wr_ptr stable at $p0 over 1.5 s) -- consistent with a wedge"
        puts "> scsi-trace: (ring IS present: nonzero wrptr/wrapped/frozen seen at some point)"
    }

    # Freeze, then confirm the DUT agrees it is frozen before reading.
    vhdd_wrreg $::VHDD_OFF_TRACE_CTRL 0x1
    after 100
    lassign [scsi_trace_status] wp wrapped frozen
    if {!$frozen} {
        error "scsi-trace: freeze did not take (TRACE_CTRL.frozen still 0) -- refusing to dump a live ring"
    }

    # Oldest-first ordering: if the ring wrapped, the oldest entry is the one
    # wr_ptr is about to overwrite.
    set depth $::VHDD_TRACE_DEPTH
    if {$wrapped} { set n $depth; set start $wp } else { set n $wp; set start 0 }
    puts [format "> scsi-trace: wr_ptr=%d wrapped=%d -> dumping %d entries" $wp $wrapped $n]

    set fh [open $path w]
    puts $fh "# scsi_trace_ring dump — idx,rw,reg,byte,tstamp"
    puts $fh [format "# wr_ptr=%d wrapped=%d depth=%d" $wp $wrapped $depth]
    set t0 [clock milliseconds]
    for {set i 0} {$i < $n} {incr i} {
        set idx [expr {($start + $i) % $depth}]
        set e [scsi_trace_entry $idx]
        set rw   [expr {($e >> 31) & 1}]
        set dma  [expr {($e >> 30) & 1}]
        set reg  [expr {($e >> 26) & 0xF}]
        set byte [expr {($e >> 18) & 0xFF}]
        set ts   [expr {$e & 0x3FFFF}]
        set rs   [expr {$rw ? "W" : "R"}]
        set rn   [expr {$dma ? "d" : [format "%x" $reg]}]
        puts $fh [format "%d,%s,%s,%02x,%d" $i $rs $rn $byte $ts]
    }
    close $fh
    set dt [expr {([clock milliseconds] - $t0) / 1000.0}]
    puts [format "> scsi-trace: wrote %d entries to %s in %.1f s" $n $path $dt]
    puts "> scsi-trace: ring is still FROZEN — run `scsi-trace rearm` to resume capture"
}

# Presence probe.  fatal=1 -> error out; fatal=0 -> warn loudly, return the
# value actually read so the caller can still dump raw registers.
proc vhdd_probe {{fatal 1}} {
    set id [rdx_or_empty [vhdd_addr $::VHDD_OFF_IDENT]]
    if {$id eq ""} {
        error [format "vhdd: IDENT read FAILED (AXI transaction error) at 0x%08X" [vhdd_addr $::VHDD_OFF_IDENT]]
    }
    if {$id != $::VHDD_IDENT} {
        set msg [format "vhdd: IDENT = 0x%08X, expected 0x%08X -- THIS BITSTREAM HAS NO vHDD CONTROL BLOCK. The 0x%08X window is a null slave in builds without it: every offset reads 0 and every write is swallowed, so any decoded value would be fiction. Load a bitstream that instantiates the vHDD control block." \
                    $id $::VHDD_IDENT $::VHDD_BASE]
        if {$fatal} { error $msg }
        puts "> ***** WARNING $msg"
    }
    return $id
}

# Aperture base, READ FROM HARDWARE.  Sanity-checked so a null-slave 0 (or a
# garbage read that somehow got past the IDENT check) can never be used as a
# write target.
proc vhdd_aperture {} {
    vhdd_probe 1
    set a [vhdd_rdreg $::VHDD_OFF_RD_APER]
    if {$a == 0 || ($a & 0xFFF) != 0} {
        error [format "vhdd: RD_APERTURE reads 0x%08X, which is not a plausible 4 KiB-aligned aperture base -- refusing to touch it" $a]
    }
    return $a
}

# Configured RAM-disk extent in bytes (RD_BLOCKS * 512).
proc vhdd_disk_bytes {} {
    return [expr {wide([vhdd_rdreg $::VHDD_OFF_RD_BLOCKS]) * 512}]
}

proc vhdd_mb {bytes} { return [format "%.1f MB" [expr {double($bytes) / 1048576.0}]] }

# ── status ──────────────────────────────────────────────────────────────────
proc vhdd_status_report {} {
    set id [vhdd_probe 0]
    if {$id != $::VHDD_IDENT} {
        puts "> vhdd-status: raw register dump follows -- these values are NOT meaningful:"
        foreach {name off} [list \
                IDENT         $::VHDD_OFF_IDENT \
                CTRL          $::VHDD_OFF_CTRL \
                RD_BLOCKS     $::VHDD_OFF_RD_BLOCKS \
                SD_BLOCKS     $::VHDD_OFF_SD_BLOCKS \
                STATUS        $::VHDD_OFF_STATUS \
                RD_MAX_BLOCKS $::VHDD_OFF_RD_MAX \
                RD_APERTURE   $::VHDD_OFF_RD_APER] {
            puts [format "> vhdd-status:   %-13s (+0x%03X) = 0x%08X" $name $off [vhdd_rdreg $off]]
        }
        return
    }

    set ctrl  [vhdd_rdreg $::VHDD_OFF_CTRL]
    set rdblk [vhdd_rdreg $::VHDD_OFF_RD_BLOCKS]
    set sdblk [vhdd_rdreg $::VHDD_OFF_SD_BLOCKS]
    set st    [vhdd_rdreg $::VHDD_OFF_STATUS]
    set rdmax [vhdd_rdreg $::VHDD_OFF_RD_MAX]
    set aper  [vhdd_rdreg $::VHDD_OFF_RD_APER]

    set sd_en    [expr {$ctrl & $::VHDD_CTRL_SD_EN ? 1 : 0}]
    set ram_en   [expr {$ctrl & $::VHDD_CTRL_RAM_EN ? 1 : 0}]
    set st_sd    [expr {$st & 0x1}]
    set st_ram   [expr {($st >> 1) & 0x1}]
    set busy     [expr {($st >> 2) & 0x1}]
    set err      [expr {($st >> 3) & 0x1}]
    set fsm      [expr {($st >> 4) & 0xF}]
    set wdog     [expr {($st >> 16) & 0xFFFF}]

    puts [format "> vhdd-status: ident=0x%08X OK  ctrl=0x%08X  status=0x%08X" $id $ctrl $st]
    puts [format "> vhdd-status: SD volume : %-8s  %d blocks (%s)   status.sd_en=%d" \
              [expr {$sd_en ? "ENABLED" : "disabled"}] $sdblk [vhdd_mb [expr {wide($sdblk)*512}]] $st_sd]
    puts [format "> vhdd-status: RAM disk  : %-8s  %d blocks (%s)   max %d blocks (%s)   status.ram_en=%d" \
              [expr {$ram_en ? "ENABLED" : "disabled"}] $rdblk [vhdd_mb [expr {wide($rdblk)*512}]] \
              $rdmax [vhdd_mb [expr {wide($rdmax)*512}]] $st_ram]
    puts [format "> vhdd-status: RAM disk  : busy=%d error=%d fsm_state=0x%X watchdog_fires=%d%s" \
              $busy $err $fsm $wdog \
              [expr {$wdog == 0xFFFF ? " (SATURATED)" : ""}]]
    puts [format "> vhdd-status: aperture  : 0x%08X (read from RD_APERTURE -- never hardcoded)" $aper]
    if {$sd_en != $st_sd || $ram_en != $st_ram} {
        puts "> vhdd-status: ***** WARNING CTRL and STATUS disagree about which volumes are enabled -- a write to CTRL may not have taken effect"
    }
    if {$err} {
        puts "> vhdd-status: ***** WARNING RAM-disk last-request ERROR bit is set (STATUS bit 3)"
    }
    if {$wdog} {
        puts "> vhdd-status: ***** WARNING RAM-disk watchdog has fired $wdog time(s) -- the volume did not answer a request in bounded time"
    }
}

# ── CTRL[2]: SD volume write-protect ────────────────────────────────────────
# Locks the SD-backed volume: the SCSI target answers WRITE(6)/WRITE(10) with
# CHECK CONDITION / DATA PROTECT (sense key 7, ASC 0x27) and the card is never
# touched.  Reported properly rather than silently dropped, so the guest mounts
# the volume read-only instead of believing writes succeeded -- a silent drop
# would let its in-memory filesystem state diverge from the disk.
proc vhdd_wprot {{val ""}} {
    vhdd_probe 1
    set before [vhdd_rdreg $::VHDD_OFF_CTRL]
    if {$val eq ""} {
        puts [format "> vhdd-wprot: SD volume is %s   (CTRL=0x%08X)" \
                  [expr {($before & $::VHDD_CTRL_SD_WPROT) ? "WRITE-PROTECTED" : "writable"}] $before]
        return
    }
    if {$val ne "0" && $val ne "1" && $val ne "on" && $val ne "off"} {
        error "usage: vhdd-wprot \[on|off\]   (got '$val')"
    }
    set on [expr {($val eq "1" || $val eq "on") ? 1 : 0}]
    if {$on} {
        set want [expr {$before | $::VHDD_CTRL_SD_WPROT}]
    } else {
        set want [expr {$before & ~$::VHDD_CTRL_SD_WPROT}]
    }
    vhdd_wrreg $::VHDD_OFF_CTRL $want
    set after [vhdd_rdreg $::VHDD_OFF_CTRL]
    puts [format "> vhdd-wprot: SD volume -> %s   CTRL before=0x%08X wrote=0x%08X after=0x%08X" \
              [expr {$on ? "WRITE-PROTECTED" : "writable"}] $before $want $after]
    if {(($after & $::VHDD_CTRL_SD_WPROT) != 0) != ($on != 0)} {
        puts "> vhdd-wprot: ***** WARNING CTRL\[2\] did not read back as written -- the write did NOT take effect"
    }
}

# ── CTRL enable bits ────────────────────────────────────────────────────────
proc vhdd_set_enable {which val} {
    switch -- $which {
        sd   { set mask $::VHDD_CTRL_SD_EN }
        ram  { set mask $::VHDD_CTRL_RAM_EN }
        both { set mask [expr {$::VHDD_CTRL_SD_EN | $::VHDD_CTRL_RAM_EN}] }
        default { error "usage: vhdd-enable <sd|ram|both> <0|1>   (got volume '$which')" }
    }
    if {$val ne "0" && $val ne "1"} {
        error "usage: vhdd-enable <sd|ram|both> <0|1>   (got value '$val')"
    }
    vhdd_probe 1
    set before [vhdd_rdreg $::VHDD_OFF_CTRL]
    if {$val eq "1"} {
        set want [expr {$before | $mask}]
    } else {
        set want [expr {$before & ~$mask}]
    }
    vhdd_wrreg $::VHDD_OFF_CTRL $want
    set after [vhdd_rdreg $::VHDD_OFF_CTRL]
    puts [format "> vhdd-enable: %s -> %s   CTRL before=0x%08X wrote=0x%08X after=0x%08X" \
              $which [expr {$val eq "1" ? "ENABLED" : "disabled"}] $before $want $after]
    puts [format "> vhdd-enable: sd_en=%d ram_en=%d" \
              [expr {($after & $::VHDD_CTRL_SD_EN) ? 1 : 0}] \
              [expr {($after & $::VHDD_CTRL_RAM_EN) ? 1 : 0}]]
    if {($after & 0x3) != ($want & 0x3)} {
        puts [format "> vhdd-enable: ***** WARNING CTRL bits\[1:0\] read back as 0x%X, not the 0x%X written -- the write did NOT take effect" \
                  [expr {$after & 3}] [expr {$want & 3}]]
    }
}

# ── RD_BLOCKS ───────────────────────────────────────────────────────────────
proc vhdd_set_size_mb {mb} {
    if {$mb <= 0} { error "ramdisk-size: size must be >= 1 MB (got $mb)" }
    set blocks [expr {wide($mb) * 2048}]
    if {$blocks > 0xFFFFFFFF} {
        error "ramdisk-size: $mb MB = $blocks blocks does not fit in the 32-bit RD_BLOCKS register"
    }
    vhdd_probe 1
    set before [vhdd_rdreg $::VHDD_OFF_RD_BLOCKS]
    set maxb   [vhdd_rdreg $::VHDD_OFF_RD_MAX]
    if {$blocks > $maxb} {
        puts [format "> ramdisk-size: NOTE requested %d blocks exceeds RD_MAX_BLOCKS %d (%s) -- hardware will clamp" \
                  $blocks $maxb [vhdd_mb [expr {wide($maxb)*512}]]]
    }
    vhdd_wrreg $::VHDD_OFF_RD_BLOCKS $blocks
    set after [vhdd_rdreg $::VHDD_OFF_RD_BLOCKS]
    puts [format "> ramdisk-size: RD_BLOCKS before=%d (%s)  requested=%d (%d MB)  IN EFFECT=%d (%s)" \
              $before [vhdd_mb [expr {wide($before)*512}]] $blocks $mb \
              $after [vhdd_mb [expr {wide($after)*512}]]]
    if {$after != $blocks} {
        puts [format "> ramdisk-size: ***** the hardware did NOT take the requested value (clamped to RD_MAX_BLOCKS=%d, or the register is read-only in this build). The effective size is %d blocks (%s)." \
                  $maxb $after [vhdd_mb [expr {wide($after)*512}]]]
    }
    puts "> ramdisk-size: ***** WARNING changing the RAM-disk size while Mac OS has the volume MOUNTED will confuse the OS (its cached drive geometry and volume bitmap no longer match the device). Unmount/eject first, or resize before boot."
    return $after
}

# ── bulk transfer primitives ────────────────────────────────────────────────

# Beats for one burst starting at word-aligned $addr, wanting $want words:
# capped at 256 beats and never crossing a 4 KiB AXI boundary.
proc vhdd_burst_beats {addr want} {
    set beats $want
    if {$beats > 256} { set beats 256 }
    set to_boundary [expr {(4096 - ($addr & 0xFFF)) / 4}]
    if {$to_boundary < 1} { set to_boundary 1 }
    if {$beats > $to_boundary} { set beats $to_boundary }
    return $beats
}

# One burst write.  $words is a list of integers; word i lands at addr+4*i.
proc vhdd_wrburst {addr words} {
    set n [llength $words]
    if {$n == 0} { return }
    set hexes {}
    foreach w $words { lappend hexes [format %08X [expr {$w & 0xFFFFFFFF}]] }
    if {$::vhdd_wr_order eq "rev"} {
        set data [join [lreverse $hexes] ""]
    } else {
        set data [join $hexes ""]
    }
    create_hw_axi_txn -quiet -force _vhw $::axi -type WRITE \
        -address [format %08X $addr] -len $n -burst INCR -data $data
    run_hw_axi -quiet _vhw
}

# Measure (once per session) which end of the -data hex string is the LOWEST
# address of a burst write, and prove the burst actually INCREMENTS.
#
# NON-DESTRUCTIVE: the 64-word probe region is saved and restored one word at
# a time.  Single-word reads/writes are the only primitive whose ordering is
# not itself under test, so the restore cannot be corrupted by the very bug
# being measured.
proc vhdd_calibrate_wr_order {scratch} {
    if {$::vhdd_wr_order ne ""} { return }
    set nprobe 64
    set saved {}
    for {set i 0} {$i < $nprobe} {incr i} {
        lappend saved [rdx [expr {$scratch + 4*$i}]]
    }
    # Distinct, index-identifying pattern so a mismatch says WHICH beat moved.
    set probe {}
    for {set i 0} {$i < $nprobe} {incr i} {
        lappend probe [expr {0x5AD00000 | ($i << 8) | ((~$i) & 0xFF)}]
    }
    # Interior indices as well as the ends: checking only 0 and n-1 is exactly
    # what let the historical dump-mem beat scramble through undetected.
    set check {0 1 2 31 62 63}
    set found ""
    foreach ord {fwd rev} {
        set ::vhdd_wr_order $ord
        vhdd_wrburst $scratch $probe
        set ok 1
        foreach i $check {
            if {[rdx [expr {$scratch + 4*$i}]] != [lindex $probe $i]} { set ok 0 ; break }
        }
        if {$ok} { set found $ord ; break }
    }
    # Restore, word at a time, regardless of the outcome.
    for {set i 0} {$i < $nprobe} {incr i} {
        wr [expr {$scratch + 4*$i}] [expr {[lindex $saved $i] & 0xFFFFFFFF}]
    }
    set ::vhdd_wr_order $found
    if {$found eq ""} {
        error [format "vhdd: burst-write calibration FAILED at 0x%08X -- neither word order reproduced a 64-beat INCR probe at indices {%s}. The JTAG-AXI write-burst path is not usable for this aperture; do not trust any bulk write. (The probe region has been restored.)" \
                   $scratch $check]
    }
    puts "> vhdd: AXI burst-write word order = $::vhdd_wr_order (verified at beats $check of a 64-beat INCR probe)"
    flush stdout
}

# Confirm a burst read really returned STRICTLY ASCENDING addresses, by
# re-reading interior beats one word at a time.  rd_burst's own probe only
# looks at beat 0 and beat n-1, so it is blind to an interior permutation.
proc vhdd_burst_read_is_ascending {base hexes} {
    set n [llength $hexes]
    if {$n < 3} { return 1 }
    set idxs {}
    foreach i [list 1 [expr {$n/2}] [expr {$n-2}]] {
        if {$i > 0 && $i < $n-1 && [lsearch -exact $idxs $i] < 0} { lappend idxs $i }
    }
    foreach i $idxs {
        set a [expr {$base + 4*$i}]
        set probe [rd $a]
        if {$probe eq "BADA0BAD"} {
            puts [format "> vhdd: burst-read order check could not complete (single-word read of 0x%08X failed)" $a]
            return 0
        }
        if {![string equal -nocase $probe [lindex $hexes $i]]} {
            puts [format "> vhdd: burst-read ORDER CHECK FAILED at beat %d (0x%08X): burst says 0x%s, single-word read says 0x%s" \
                      $i $a [lindex $hexes $i] $probe]
            return 0
        }
    }
    return 1
}

# Write a list of integers to consecutive words starting at $addr.
proc vhdd_wr_words {addr words} {
    set n [llength $words]
    set i 0
    while {$i < $n} {
        set cur   [expr {$addr + 4*$i}]
        set beats [vhdd_burst_beats $cur [expr {$n - $i}]]
        vhdd_wrburst $cur [lrange $words $i [expr {$i + $beats - 1}]]
        incr i $beats
    }
}

# Read $n consecutive words starting at $addr; returns INTEGERS in strictly
# ascending address order.  The first burst of every call is order-checked
# (~3 extra single-word reads per call, i.e. per 256 KiB -- negligible);
# a failure downgrades the whole session to per-word reads, loudly.
proc vhdd_rd_words {addr n} {
    set out {}
    set i 0
    set checked 0
    while {$i < $n} {
        set cur   [expr {$addr + 4*$i}]
        set beats [vhdd_burst_beats $cur [expr {$n - $i}]]
        if {$::vhdd_rd_slow} {
            set hexes [rd_burst_slow $cur $beats]
        } else {
            set hexes [rd_burst $cur $beats]
            if {!$checked} {
                set checked 1
                if {![vhdd_burst_read_is_ascending $cur $hexes]} {
                    set ::vhdd_rd_slow 1
                    puts "> vhdd: ***** burst reads are NOT returning strictly ascending addresses -- falling back to per-word reads for the REST OF THIS SESSION (much slower, but correct).  Any burst-read data reported BEFORE this line should be treated as suspect."
                    flush stdout
                    set hexes [rd_burst_slow $cur $beats]
                }
            }
        }
        foreach h $hexes {
            if {$h eq "BADA0BAD"} {
                error [format "vhdd: AXI read failed inside a bulk read near 0x%08X -- refusing to emit a fabricated byte" $cur]
            }
            lappend out [expr {"0x$h"}]
        }
        incr i $beats
    }
    return $out
}

# Sampled read-back of data we just wrote: proves the offset arithmetic and
# the burst beat order on LIVE data, for ~3 reads per sampled chunk.
proc vhdd_verify_sample {addr words label} {
    set n [llength $words]
    if {$n == 0} { return }
    foreach i [list 0 [expr {$n/2}] [expr {$n-1}]] {
        set a    [expr {$addr + 4*$i}]
        set want [expr {[lindex $words $i] & 0xFFFFFFFF}]
        set got  [rdx_or_empty $a]
        if {$got eq ""} {
            error [format "%s: read-back sample at 0x%08X FAILED (AXI error) -- cannot confirm the transfer" $label $a]
        }
        if {($got & 0xFFFFFFFF) != $want} {
            error [format "%s: READ-BACK MISMATCH at 0x%08X -- wrote 0x%08X, reads 0x%08X. The transfer is not landing correctly; aborting instead of reporting a false success." \
                       $label $a $want $got]
        }
    }
}

proc vhdd_progress {label done total t0} {
    set dt [expr {[clock milliseconds] - $t0}]
    set secs [expr {$dt / 1000.0}]
    set rate 0.0
    if {$dt > 0} { set rate [expr {double($done) / 1024.0 / $secs}] }
    set pct 0.0
    if {$total > 0} { set pct [expr {100.0 * double($done) / double($total)}] }
    puts [format "> %s: %d/%d bytes (%.1f%%) %.1f s %.0f KiB/s" $label $done $total $pct $secs $rate]
    flush stdout
}

proc vhdd_summary {label bytes t0} {
    set dt [expr {[clock milliseconds] - $t0}]
    set secs [expr {$dt / 1000.0}]
    set rate 0.0
    if {$dt > 0} { set rate [expr {double($bytes) / 1024.0 / $secs}] }
    puts [format "> %s done: %d bytes (%s) in %.1f s (%.0f KiB/s)" $label $bytes [vhdd_mb $bytes] $secs $rate]
    flush stdout
}

# Read-modify-write a partial word so bytes outside [first..first+len-1] of
# the word at $wa are preserved exactly.
proc vhdd_rmw_partial {wa first chunk} {
    set k [string length $chunk]
    set old [binary format i [rdx $wa]]
    set new [string replace $old $first [expr {$first + $k - 1}] $chunk]
    binary scan $new iu nw
    wr $wa [expr {$nw & 0xFFFFFFFF}]
}

# ── ramdisk-clear ───────────────────────────────────────────────────────────
proc vhdd_ramdisk_clear {{mb 0}} {
    set aper [vhdd_aperture]
    set disk [vhdd_disk_bytes]
    if {$disk <= 0} {
        error "ramdisk-clear: RD_BLOCKS reads 0 -- the RAM disk has no extent to clear. Set one with `ramdisk-size <MB>`."
    }
    if {$mb > 0} {
        set bytes [expr {wide($mb) * 1048576}]
        if {$bytes > $disk} {
            error "ramdisk-clear: $mb MB exceeds the configured RAM disk ($disk bytes = [vhdd_mb $disk]). Grow it first with `ramdisk-size <MB>`."
        }
    } else {
        set bytes $disk
    }
    if {$bytes == 0} { puts "> ramdisk-clear: zero-length extent -- nothing to do" ; return 0 }

    vhdd_calibrate_wr_order $aper
    puts [format "> ramdisk-clear: zeroing %d bytes (%s) at aperture 0x%08X" $bytes [vhdd_mb $bytes] $aper]
    flush stdout

    set zeros {}
    for {set i 0} {$i < 256} {incr i} { lappend zeros 0 }
    set nwords [expr {$bytes / 4}]
    set t0 [clock milliseconds]
    set next_report 1048576
    set i 0
    while {$i < $nwords} {
        set cur   [expr {$aper + 4*$i}]
        set beats [vhdd_burst_beats $cur [expr {$nwords - $i}]]
        vhdd_wrburst $cur [lrange $zeros 0 [expr {$beats - 1}]]
        incr i $beats
        set done [expr {$i * 4}]
        if {$done >= $next_report} {
            vhdd_progress "ramdisk-clear" $done $bytes $t0
            set next_report [expr {$done + 1048576}]
        }
    }
    # Positive control: the extremes and the middle must actually read zero.
    foreach off [list 0 [expr {($bytes/2) & ~3}] [expr {$bytes - 4}]] {
        set v [rdx_or_empty [expr {$aper + $off}]]
        if {$v eq ""} { error "ramdisk-clear: verification read failed at offset $off" }
        if {$v != 0} {
            error [format "ramdisk-clear: VERIFY FAILED -- offset %d (0x%08X) reads 0x%08X, not 0" $off [expr {$aper+$off}] $v]
        }
    }
    vhdd_summary "ramdisk-clear" $bytes $t0
    return $bytes
}

# ── ramdisk-load ────────────────────────────────────────────────────────────
proc vhdd_ramdisk_load {path {boff 0}} {
    if {![file exists $path]}      { error "ramdisk-load: file not found: $path" }
    if {[file isdirectory $path]}  { error "ramdisk-load: $path is a directory, not a file" }
    if {![file readable $path]}    { error "ramdisk-load: file is not readable: $path" }
    if {$boff < 0}                 { error "ramdisk-load: byte offset must be >= 0 (got $boff)" }

    set fsize [file size $path]
    set aper  [vhdd_aperture]
    set disk  [vhdd_disk_bytes]

    if {$fsize == 0} {
        puts "> ramdisk-load: $path is EMPTY (0 bytes) -- nothing written"
        return 0
    }
    if {$boff + $fsize > $disk} {
        error [format "ramdisk-load: REFUSING -- %s is %d bytes at offset %d, ending at %d, which is past the RAM disk's configured end (%d bytes = %s, RD_BLOCKS=%d). Grow it with `ramdisk-size <MB>` or pick a smaller offset." \
                   $path $fsize $boff [expr {$boff + $fsize}] $disk [vhdd_mb $disk] [expr {$disk/512}]]
    }

    if {[catch {set fh [open $path "rb"]} err]} {
        error "ramdisk-load: cannot open $path: $err"
    }
    fconfigure $fh -translation binary -encoding binary

    vhdd_calibrate_wr_order $aper

    puts [format "> ramdisk-load: %s -> aperture 0x%08X + %d, %d bytes (%s)" \
              $path $aper $boff $fsize [vhdd_mb $fsize]]
    flush stdout

    set t0        [clock milliseconds]
    set pos       [expr {$aper + $boff}]
    set written   0
    set remaining $fsize
    set BATCH     [expr {256 * 1024}]

    if {[catch {
        # Leading partial word: preserve the bytes before $boff.
        set lead [expr {$pos & 3}]
        if {$lead != 0} {
            set k [expr {(4 - $lead) < $remaining ? (4 - $lead) : $remaining}]
            set chunk [read $fh $k]
            if {[string length $chunk] != $k} {
                error "ramdisk-load: short read from $path (got [string length $chunk] of $k bytes) -- file changed under us?"
            }
            vhdd_rmw_partial [expr {$pos & ~3}] $lead $chunk
            incr pos $k
            incr written $k
            set remaining [expr {$remaining - $k}]
        }

        # Word-aligned bulk body.
        set next_report 1048576
        set verify_at   0          ;# sample-verify the FIRST chunk, then ~1 MB apart
        while {$remaining >= 4} {
            set want [expr {$remaining < $BATCH ? $remaining : $BATCH}]
            set want [expr {$want - ($want % 4)}]
            set chunk [read $fh $want]
            set got [string length $chunk]
            if {$got != $want} {
                error "ramdisk-load: SHORT READ from $path -- got $got of $want bytes at file offset $written. Refusing to write a truncated image."
            }
            binary scan $chunk iu* words
            vhdd_wr_words $pos $words
            set chunk_addr $pos
            incr pos $got
            incr written $got
            set remaining [expr {$remaining - $got}]
            if {$written >= $next_report} {
                vhdd_progress "ramdisk-load" $written $fsize $t0
                set next_report [expr {$written + 1048576}]
            }
            if {$written >= $verify_at || $remaining < 4} {
                vhdd_verify_sample $chunk_addr $words "ramdisk-load"
                set verify_at [expr {$written + 1048576}]
            }
        }

        # Trailing partial word: preserve the bytes after the file's end.
        if {$remaining > 0} {
            set chunk [read $fh $remaining]
            if {[string length $chunk] != $remaining} {
                error "ramdisk-load: short read of the final $remaining bytes from $path"
            }
            vhdd_rmw_partial $pos 0 $chunk
            incr written $remaining
            set remaining 0
        }
    } err]} {
        close $fh
        error $err
    }
    close $fh
    vhdd_summary "ramdisk-load" $written $t0
    return $written
}

# ── ramdisk-save ────────────────────────────────────────────────────────────
proc vhdd_ramdisk_save {path count {boff 0}} {
    if {$count < 0} { error "ramdisk-save: byte count must be >= 0 (got $count)" }
    if {$boff  < 0} { error "ramdisk-save: byte offset must be >= 0 (got $boff)" }

    set aper [vhdd_aperture]
    set disk [vhdd_disk_bytes]

    if {$count == 0} {
        puts "> ramdisk-save: zero-length request -- nothing read, no file written"
        return 0
    }
    if {$boff + $count > $disk} {
        error [format "ramdisk-save: REFUSING -- %d bytes at offset %d ends at %d, past the RAM disk's configured end (%d bytes = %s, RD_BLOCKS=%d)." \
                   $count $boff [expr {$boff + $count}] $disk [vhdd_mb $disk] [expr {$disk/512}]]
    }
    if {[catch {set fh [open $path "wb"]} err]} {
        error "ramdisk-save: cannot create $path: $err"
    }
    fconfigure $fh -translation binary -encoding binary

    puts [format "> ramdisk-save: aperture 0x%08X + %d, %d bytes (%s) -> %s" \
              $aper $boff $count [vhdd_mb $count] $path]
    flush stdout

    # Cover [boff, boff+count) with whole words; trim the ends on the way out.
    set start_w [expr {$boff & ~3}]
    set end_w   [expr {(($boff + $count) + 3) & ~3}]
    set skip    [expr {$boff - $start_w}]
    set total_w [expr {($end_w - $start_w) / 4}]

    set t0       [clock milliseconds]
    set produced 0
    set wdone    0
    set wpos     $start_w
    set CHUNK_W  65536
    set next_report 1048576

    if {[catch {
        while {$wdone < $total_w} {
            set nw [expr {($total_w - $wdone) < $CHUNK_W ? ($total_w - $wdone) : $CHUNK_W}]
            # vhdd_rd_words returns ascending-address words; binary format i*
            # emits each word LSB-first, i.e. byte at base+0 first.  Result:
            # strictly ascending byte order out of the file.
            set words [vhdd_rd_words [expr {$aper + $wpos}] $nw]
            set bytes [binary format i* $words]
            if {$wdone == 0 && $skip > 0} { set bytes [string range $bytes $skip end] }
            set room [expr {$count - $produced}]
            if {[string length $bytes] > $room} {
                set bytes [string range $bytes 0 [expr {$room - 1}]]
            }
            puts -nonewline $fh $bytes
            incr produced [string length $bytes]
            incr wdone $nw
            incr wpos [expr {$nw * 4}]
            if {$produced >= $next_report || $wdone >= $total_w} {
                vhdd_progress "ramdisk-save" $produced $count $t0
                set next_report [expr {$produced + 1048576}]
            }
        }
    } err]} {
        close $fh
        error $err
    }
    close $fh
    if {$produced != $count} {
        error "ramdisk-save: wrote $produced bytes to $path but $count were requested -- internal offset arithmetic error, do NOT trust the file"
    }
    vhdd_summary "ramdisk-save" $produced $t0
    return $produced
}

# Size / byte-count operand parser.
#
# DELIBERATELY DECIMAL BY DEFAULT -- the opposite of parse_num, which every
# ADDRESS operand in this REPL uses.  "ramdisk-size 32" must mean 32 MB, not
# 0x32 = 50 MB, and a silently-wrong size is exactly the class of plausible
# fabrication this file keeps getting bitten by.  Prefix with 0x for hex.
# Callers echo the resolved value so a misparse is visible immediately.
proc parse_count {tok {what "count"}} {
    if {$tok eq ""} { error "missing $what" }
    if {[string match "0x*" $tok] || [string match "0X*" $tok]} {
        set body [string range $tok 2 end]
        if {![string is xdigit -strict $body]} { error "bad hex $what: $tok" }
        return [expr {"0x$body"}]
    }
    if {![string is digit -strict $tok]} {
        error "bad $what: '$tok' -- sizes and byte counts here are DECIMAL and non-negative (prefix 0x for hex)"
    }
    set n 0
    if {[scan $tok %d n] != 1} { error "bad decimal $what: $tok" }
    return $n
}

# ══════════════════════════════════════════════════════════════════════
# Capability discovery (debug_ctrl >= 0xDEB6_0006)
# ══════════════════════════════════════════════════════════════════════
# The host tool and the bitstream version independently -- `load-bit`
# swaps bitstreams mid-session, and branch builds carry different feature
# subsets.  Ask the bitstream what it has instead of guessing.
proc dbg_features {} {
    set v [rdx_or_empty [expr {$::DBG_BASE + $::OFF_FEATURES}]]
    if {$v eq ""} { return "" }
    return $v
}

proc dbg_version {} {
    set v [rdx_or_empty [expr {$::DBG_BASE + $::OFF_VERSION}]]
    if {$v eq ""} { return "" }
    return $v
}

proc dbg_is_core040_epoch {} {
    set v [dbg_version]
    return [expr {$v ne "" && $v == $::CORE040_DEBUG_VERSION}]
}

proc dbg_feature_names {} {
    if {[dbg_is_core040_epoch]} { return $::DBG_FEATURE_NAMES_CORE040 }
    return $::DBG_FEATURE_NAMES_LEGACY
}

proc require_core040_debug_epoch {what} {
    set v [dbg_version]
    if {$v eq ""} { error "$what: cannot read DBG_VERSION" }
    if {$v != $::CORE040_DEBUG_VERSION} {
        error [format "%s requires the m68k040 debug epoch 0x%08X; hardware reports 0x%08X" \
                   $what $::CORE040_DEBUG_VERSION $v]
    }
}

proc dbg_has_feature {name} {
    # Validate the NAME first, unconditionally.  If this ran after the
    # "features unreadable / all-zero" early-out, a typo'd feature name
    # would quietly answer "no" on exactly the bitstreams where the caller
    # most needs a hard error -- the same silent-plausible-answer failure
    # mode this whole change is about.
    set names [dbg_feature_names]
    set idx [lsearch -exact $names $name]
    if {$idx < 0} { error "unknown debug feature name: $name" }
    set feats [dbg_features]
    if {$feats eq "" || $feats == 0} { return 0 }
    return [expr {($feats >> $idx) & 1}]
}

# Human-readable rendering of a 7-bit DAFB monitor-sense code.  Standard
# codes (bit 6 clear) name a display per MAME dafb.cpp:202-216; extended
# codes (bit 6 set) carry three 2-bit fields the CPU probes one at a time,
# so they get shown as ext(bc,ac,ab) rather than a single display name.
proc ram_window_describe {lg2} {
    if {$lg2 < 22 || $lg2 > 30} { return "invalid" }
    set bytes [expr {1 << $lg2}]
    if {$bytes >= 1024*1024*1024} { return [format "%d GiB" [expr {$bytes / (1024*1024*1024)}]] }
    return [format "%d MiB" [expr {$bytes / (1024*1024)}]]
}

proc mon_sense_describe {code} {
    set code [expr {$code & 0x7F}]
    if {$code & 0x40} {
        # NOTE the escaped brackets: an unescaped \[extended monitor\] inside a
        # double-quoted Tcl string is COMMAND SUBSTITUTION, and this proc
        # threw "invalid command name extended" for every extended code
        # before they were escaped.
        return [format "0x%02X ext(bc=%d,ac=%d,ab=%d) \[extended monitor\]" \
                    $code [expr {($code >> 4) & 3}] [expr {($code >> 2) & 3}] \
                    [expr {$code & 3}]]
    }
    set names {
        "Mac 21\" Color Display"
        "Mac Portrait B&W 15\""
        "Mac RGB 12\" 512x384 (Rubik)"
        "Mac Two-Page B&W 21\""
        "(code 4 — unassigned in MAME's table)"
        "(code 5 — unassigned in MAME's table)"
        "Mac Hi-Res 12-14\" 640x480"
        "no monitor"
    }
    return [format "0x%02X %s" $code [lindex $names [expr {$code & 0x7}]]]
}

# ══ windowed performance counters ═══════════════════════════════════════════════
#
# WHY THIS REFUSES INSTEAD OF PRINTING ZEROS.  Four registers in this project have
# read zero and LIED: pc_live, exc_count, OFF_CYCLE_LO, and wedge-status (which now
# refuses outright because it reads v1 probes cpu040's mux never implemented).
# OFF_MISPRED_COUNT and OFF_FLUSH_COUNT were two more -- declared at Stage 1 and never
# driven.  A perf table of zeros on an older bitstream reads exactly like "this
# workload had no mispredicts and no cache misses", which is a conclusion, not a
# reading.  So: probe OFF_PERF_CTL first, and if the block is absent, say so and stop.
#
# The block ADVERTISES ITS OWN PRODUCERS in OFF_PERF_CTL[20:16].  A counter whose
# producer plugin is absent from the build reads zero and is rendered as "--", never
# as 0.  That distinction is the entire point of the presence bitmap.
proc perf_ctl_read {} {
    return [rdx_or_empty [expr {$::DBG_BASE + $::OFF_PERF_CTL}]]
}

# A 64-bit LO/HI pair.  Safe WITHOUT re-read retry only because the caller has already
# frozen the counters; a frozen pair cannot tear.  Callers that read while RUNNING get
# the same treatment OFF_INST already gets elsewhere and must not trust the top word.
proc perf_rd64 {lo_off hi_off} {
    set lo [rdx_or_empty [expr {$::DBG_BASE + $lo_off}]]
    set hi [rdx_or_empty [expr {$::DBG_BASE + $hi_off}]]
    if {$lo eq "" || $hi eq ""} { return "" }
    return [expr {($hi << 32) | $lo}]
}

proc perf_rd32 {off} {
    return [rdx_or_empty [expr {$::DBG_BASE + $off}]]
}

# Zero every windowed counter and start a fresh window.  `hold` leaves them FROZEN
# after the clear, for arming before a reset or a breakpoint release.
proc perf_clear {{hold 0}} {
    set ctl [perf_ctl_read]
    if {$ctl eq ""} {
        error "perf-clear: OFF_PERF_CTL is unreachable (JTAG-AXI read failed)"
    }
    if {[expr {($ctl >> 8) & 0xFF}] == 0} {
        error "perf-clear REFUSED: OFF_PERF_CTL reports 0 implemented counters --\
this bitstream has no windowed performance-counter block.  Writing the control\
register would be dropped with an OKAY response and you would believe the counters\
had been zeroed.  Rebuild from a core that carries feat/perf-counters."
    }
    if {$hold} {
        dbg_wr $::OFF_PERF_CTL $::PERF_CTL_CLEAR_HOLD
    } else {
        dbg_wr $::OFF_PERF_CTL $::PERF_CTL_CLEAR_RUN
    }
}

# Format an integer with thousands separators so a 10-digit cycle count is readable.
proc perf_commas {n} {
    set s [format %d $n]
    set neg ""
    if {[string index $s 0] eq "-"} { set neg "-"; set s [string range $s 1 end] }
    set out ""
    while {[string length $s] > 3} {
        set out ",[string range $s end-2 end]$out"
        set s [string range $s 0 end-3]
    }
    return "$neg$s$out"
}

# Read the whole windowed set and print a table.
#
# `freeze` (the default) stops the counters for the duration of the read, which is what
# makes the 64-bit pairs atomic, and RESUMES them afterwards unless the caller asked to
# stay frozen.  The handful of milliseconds a JTAG read pass takes are then NOT counted
# into the window -- which is correct: they are host time, not machine time.
# Render one counter cell.  An absent PRODUCER prints `--`, never `0`: a zero from a
# counter nobody drives is indistinguishable from a zero that was measured, and that
# ambiguity is how a dead probe retires a live hypothesis for free.
proc perf_cell {v present} {
    if {$v eq ""} { return "READ-FAILED" }
    if {!$present} { return "--" }
    return [perf_commas $v]
}

proc perf_report {{freeze 1} {resume 1}} {
    set ctl [perf_ctl_read]
    if {$ctl eq ""} {
        puts "> perf REFUSED: OFF_PERF_CTL is unreachable (JTAG-AXI read failed)."
        return
    }
    set ncnt [expr {($ctl >> 8) & 0xFF}]
    if {$ncnt == 0} {
        puts "> perf REFUSED: OFF_PERF_CTL reads [format 0x%08X $ctl] -- 0 implemented"
        puts ">       counters.  This bitstream predates the windowed performance-counter"
        puts ">       block.  OFF_MISPRED_COUNT / OFF_FLUSH_COUNT here are the ORIGINAL"
        puts ">       unsourced declarations and read 0 because nothing drives them, not"
        puts ">       because nothing happened.  A table would be fabricated; refusing."
        return
    }
    set was_running [expr {($ctl >> 1) & 1}]
    if {$freeze} { dbg_wr $::OFF_PERF_CTL $::PERF_CTL_FREEZE }

    set has_rob  [expr {($ctl >> 16) & 1}]
    set has_dc   [expr {($ctl >> 17) & 1}]
    set has_ic   [expr {($ctl >> 18) & 1}]
    set has_dtlb [expr {($ctl >> 19) & 1}]
    set has_itlb [expr {($ctl >> 20) & 1}]

    set cyc  [perf_rd64 $::OFF_PERF_CYCLE_LO $::OFF_PERF_CYCLE_HI]
    set inst [perf_rd64 $::OFF_PERF_INST_LO  $::OFF_PERF_INST_HI]
    set mis  [perf_rd32 $::OFF_MISPRED_COUNT]
    set flu  [perf_rd32 $::OFF_FLUSH_COUNT]
    set brn  [perf_rd32 $::OFF_PERF_BRANCH]
    set dcm  [perf_rd32 $::OFF_PERF_DC_MISS]
    set icm  [perf_rd32 $::OFF_PERF_IC_MISS]
    set dtw  [perf_rd32 $::OFF_PERF_DTLB_WALK]
    set itw  [perf_rd32 $::OFF_PERF_ITLB_WALK]
    set str  [perf_rd32 $::OFF_PERF_STALL_RETIRE]
    set sdc  [perf_rd32 $::OFF_PERF_STALL_DC]
    set swk  [perf_rd32 $::OFF_PERF_STALL_WALK]

    # Free-running companions, for the "is this window a sane slice of the run" check.
    set fr_inst [perf_rd64 $::OFF_INST_LO $::OFF_INST_HI]
    set fr_cyc  [perf_rd64 $::OFF_CYCLE_LO $::OFF_CYCLE_HI]

    if {$freeze && $resume} { dbg_wr $::OFF_PERF_CTL $::PERF_CTL_RUN }

    puts "> perf: OFF_PERF_CTL=[format 0x%08X $ctl] counters=$ncnt run=$was_running\
producers={rob:$has_rob dcache:$has_dc icache:$has_ic dtlb:$has_dtlb itlb:$has_itlb}"
    if {$freeze} {
        if {$resume} {
            puts ">       (counters were FROZEN for this read and resumed -- the read pass\
itself is not in the window)"
        } else {
            puts ">       (counters LEFT FROZEN -- 'perf resume' to continue the window)"
        }
    } else {
        puts ">       WARNING: read while RUNNING.  The 64-bit CYCLE/INST pairs can TEAR\
across a LO wrap.  Use plain 'perf' (which freezes) for a trustworthy pair."
    }

    puts [format ">   %-22s %18s   %s" "window cycles"      [perf_cell $cyc 1]      "core_clk, counted while RUN=1"]
    puts [format ">   %-22s %18s   %s" "retired macro-insts" [perf_cell $inst $has_rob] "0/1/2 per cycle"]
    puts [format ">   %-22s %18s   %s" "branch mispredicts"  [perf_cell $mis $has_rob]  "commit-time redirects (OFF_MISPRED_COUNT)"]
    puts [format ">   %-22s %18s   %s" "pipeline flushes"    [perf_cell $flu $has_rob]  "every whole-ROB squash (OFF_FLUSH_COUNT)"]
    puts [format ">   %-22s %18s   %s" "retired branches"    [perf_cell $brn $has_rob]  "BTB-eligible only -- see note below"]
    puts [format ">   %-22s %18s   %s" "D-cache load misses" [perf_cell $dcm $has_dc]   ""]
    puts [format ">   %-22s %18s   %s" "I-cache demand misses" [perf_cell $icm $has_ic] "prefetch fills NOT counted"]
    puts [format ">   %-22s %18s   %s" "DTLB table walks"    [perf_cell $dtw $has_dtlb] ""]
    puts [format ">   %-22s %18s   %s" "ITLB table walks"    [perf_cell $itw $has_itlb] ""]
    puts [format ">   %-22s %18s   %s" "stall cycles: retire" [perf_cell $str $has_rob] "ROB non-empty, nothing retired"]
    puts [format ">   %-22s %18s   %s" "stall cycles: dcache" [perf_cell $sdc $has_dc]  "D-cache busy"]
    puts [format ">   %-22s %18s   %s" "stall cycles: walk"   [perf_cell $swk [expr {$has_dtlb || $has_itlb}]] "either walker out of IDLE"]

    # ── derived ───────────────────────────────────────────────────────────────
    # Only from counters whose producers are PRESENT.  A ratio computed against an
    # absent counter's zero is a fabricated number, and fabricated numbers in this
    # project have already been used to rule out a root cause.
    if {$cyc ne "" && $cyc > 0 && $has_rob && $inst ne ""} {
        set ipc [expr {double($inst) / double($cyc)}]
        puts [format ">   derived: IPC = %.4f   (%s inst / %s cycles)" \
              $ipc [perf_commas $inst] [perf_commas $cyc]]
        if {$inst > 0} {
            set kinst [expr {double($inst) / 1000.0}]
            set line ">   derived: per-kilo-instruction:"
            if {$mis ne ""} { append line [format "  mispred=%.3f" [expr {$mis / $kinst}]] }
            if {$flu ne ""} { append line [format "  flush=%.3f" [expr {$flu / $kinst}]] }
            if {$has_dc && $dcm ne ""} { append line [format "  dc-miss=%.3f" [expr {$dcm / $kinst}]] }
            if {$has_ic && $icm ne ""} { append line [format "  ic-miss=%.3f" [expr {$icm / $kinst}]] }
            if {$has_dtlb && $dtw ne ""} { append line [format "  dtlb-walk=%.3f" [expr {$dtw / $kinst}]] }
            if {$has_itlb && $itw ne ""} { append line [format "  itlb-walk=%.3f" [expr {$itw / $kinst}]] }
            puts $line
        }
        if {$has_rob && $brn ne "" && $brn > 0 && $mis ne ""} {
            puts [format ">   derived: mispredict rate = %.2f%% of retired BTB-eligible branches" \
                  [expr {100.0 * $mis / $brn}]]
        }
        if {$str ne "" && $has_rob} {
            puts [format ">   derived: retire-stall = %.2f%% of window cycles" \
                  [expr {100.0 * $str / $cyc}]]
        }
    }
    if {$fr_inst ne "" && $fr_cyc ne ""} {
        puts [format ">   free-running (NOT windowed, never cleared): cycles=%s macros=%s" \
              [perf_commas $fr_cyc] [perf_commas $fr_inst]]
    }
    puts ">   NOTE 'retired branches' counts the BTB-UPDATE flow, i.e. BTB-eligible"
    puts ">        branches only, so it is a LOWER BOUND on all retired branches."
    puts ">        'branch mispredicts' has no such filter -- it is the commit-time"
    puts ">        redirect itself -- so the rate above is an UPPER bound.  Both are"
    puts ">        real measurements; neither is the other's exact denominator."
}

proc dbg_caps_report {} {
    set ver [dbg_version]
    if {$ver eq ""} {
        puts "> dbg-caps: JTAG-AXI read failed -- cannot reach the debug CSR block"
        return
    }
    puts "> dbg-caps: DBG_VERSION = [format 0x%08X $ver]"
    set feats [dbg_features]
    if {$feats eq "" || $feats == 0} {
        puts "> dbg-caps: OFF_FEATURES reads 0 -- this bitstream predates the"
        puts ">           capability block (debug_ctrl < 0xDEB6_0006).  Feature"
        puts ">           discovery is NOT supported here; do not infer that"
        puts ">           the features are absent, only that it cannot say."
        return
    }
    puts "> dbg-caps: OFF_FEATURES = [format 0x%08X $feats]"
    set i 0
    foreach nm [dbg_feature_names] {
        puts [format ">   bit %2d %-18s %s" $i $nm \
              [expr {(($feats >> $i) & 1) ? "yes" : "NO"}]]
        incr i
    }
    if {!(($feats >> 12) & 1)} {
        puts "> dbg-caps: NOTE perf_counters=NO.  On a build that predates"
        puts ">           feat/perf-counters that means OFF_MISPRED_COUNT and"
        puts ">           OFF_FLUSH_COUNT are UNSOURCED -- they read 0 because"
        puts ">           there is no counter, not because nothing happened."
        puts ">           On a NEWER build it can also mean the windowed block"
        puts ">           is present but one PRODUCER PLUGIN is absent, in which"
        puts ">           case the bit is withheld deliberately and the surviving"
        puts ">           counters are still real.  Run `perf` -- it reports the"
        puts ">           per-producer presence bitmap and renders any counter"
        puts ">           without a producer as `--`, never as 0."
    }
    set cap [rdx_or_empty [expr {$::DBG_BASE + $::OFF_CAP_TRACE}]]
    if {$cap ne "" && $cap != 0} {
        puts "> dbg-caps: PC trace depth = [expr {($cap >> 16) & 0xFFFF}], exc ring depth = [expr {$cap & 0xFFFF}]"
    }
    if {($feats >> 3) & 1} {
        set drc [rdx_or_empty [expr {$::DBG_BASE + $::OFF_DBG_RESET_CTL}]]
        if {$drc ne ""} {
            puts "> dbg-caps: CPU resets observed since power-on = [expr {($drc >> 16) & 0xFFFF}]"
        }
    }
    if {($feats >> 0) & 1} {
        puts "> dbg-caps: debug reset domain PRESENT -- break-PCs, halt-exc mask,"
        puts ">           halt-after target and the RAM window SURVIVE a CPU"
        puts ">           reset.  Arm them before the reset; no re-arm needed."
    } else {
        puts "> dbg-caps: debug reset domain ABSENT -- debug configuration is"
        puts ">           wiped by every CPU reset in this bitstream.  Arm"
        puts ">           breakpoints AFTER the reset completes."
    }
}

proc rebind_debug_cores {} {
    set ::axi [lindex [get_hw_axis] 0]
    # The DEDICATED debug master's handle goes stale across a
    # refresh/reprogram exactly like ::axi does.  Leaving it stale set
    # $::axi_dbg to null after every load-bit, so dbg_rd -- i.e.
    # halt-status, build-id, every register read -- failed with the
    # BADA0BAD sentinel while the system master worked fine.  Re-resolve it
    # here, with the same single-core fallback as at startup.
    set ::axi_dbg $::axi
    if {[llength [get_hw_axis]] > 1} { set ::axi_dbg [lindex [get_hw_axis] 1] }
    # BOTH cached probe handles go stale across a refresh/reprogram, not
    # just probe_out0.  Leaving vio_probe_out1 bound kept a dead handle for
    # the hard-reset line — the one probe you need working when everything
    # else has wedged.
    set ::vio_probe_out ""
    set ::vio_probe_out1 ""
    if {$::axi eq ""} {
        error "no JTAG AXI master found after refresh"
    }
}

proc write_frame_pgm {path base stride bpp width height} {
    set fh [open $path "wb"]
    fconfigure $fh -translation binary -encoding binary
    puts $fh "P5"
    puts $fh "$width $height"
    puts $fh "255"

    set bytes_per_row [expr {($width * $bpp + 7) / 8}]
    for {set y 0} {$y < $height} {incr y} {
        set row_hex ""
        set row_base [expr {$base + $y * $stride}]
        set byte_idx 0
        while {$byte_idx < $bytes_per_row} {
            set word_addr [expr {$row_base + ($byte_idx & ~3)}]
            set word [scan [rd $word_addr] %x]
            for {set lane 0} {$lane < 4 && $byte_idx < $bytes_per_row} {incr lane; incr byte_idx} {
                set shift [expr {(3 - $lane) * 8}]
                set byte [expr {($word >> $shift) & 0xff}]
                if {$bpp == 1} {
                    for {set bit 7} {$bit >= 0 && ([string length $row_hex] / 2) < $width} {incr bit -1} {
                        set pix [expr {(($byte >> $bit) & 1) ? 255 : 0}]
                        append row_hex [format %02x $pix]
                    }
                } elseif {$bpp == 2} {
                    for {set bit 6} {$bit >= 0 && ([string length $row_hex] / 2) < $width} {incr bit -2} {
                        set pix [expr {(($byte >> $bit) & 3) * 85}]
                        append row_hex [format %02x $pix]
                    }
                } elseif {$bpp == 4} {
                    foreach shift4 {4 0} {
                        if {([string length $row_hex] / 2) >= $width} { break }
                        set pix [expr {(($byte >> $shift4) & 15) * 17}]
                        append row_hex [format %02x $pix]
                    }
                } else {
                    append row_hex [format %02x $byte]
                }
            }
        }
        puts -nonewline $fh [binary format H* $row_hex]
        if {($y & 31) == 31} {
            puts "> dump-frame-pgm row=[expr {$y + 1}]/$height"
            flush stdout
        }
    }
    close $fh
}

# ── Unified reset primitives (docs/reset_story.md) ───────────────────────
# `unified_reset` is the canonical CPU+platform clean-slate.  It pulses
# DBG_CONTROL.cold_reset_pulse (bit 5) which fans into the existing
# 1024-cy stretcher → soc_full_rst.  Boot FSM re-runs the SD→DDR copy,
# every state-bearing module on `soc_full_rst` / `pb_full_rst` re-inits,
# AXI error sticky bits clear, ROM overlay re-arms.  After the pulse the
# CPU resumes (or stays held if cold_reset_hold is set).
proc unified_reset {{hold 0}} {
    if {$hold} {
        # Set hold bit FIRST so it survives the pulse.  Then fire pulse.
        dbg_wr $::OFF_CONTROL [expr {$::CTL_COLD_RESET_HOLD | $::CTL_COLD_RESET_PULSE}]
    } else {
        dbg_wr $::OFF_CONTROL $::CTL_COLD_RESET_PULSE
    }
    # Allow time for the 1024-cy stretcher + boot_fsm SD→DDR copy.
    after 800
}

proc unified_reset_release {} {
    # Clear cold_reset_hold (and any other CONTROL bits) — CPU resumes.
    dbg_wr $::OFF_CONTROL 0x0
}

proc unified_reset_hold_status {} {
    set v [dbg_rd $::OFF_CONTROL]
    set raw [scan $v %x]
    set held [expr {($raw & $::CTL_COLD_RESET_HOLD) ? 1 : 0}]
    return "cold_reset_hold=$held control=0x$v"
}

# `reset_and_halt_after` — set hold, fire reset, arm halt-after, release.
# Replaces the old `reset_halt_after` legacy contract: no contamination
# (peripherals/DDR ROM/AXI sticky/etc. all re-init), no race window
# between reset deassert and ctrl_halt_req re-write.
proc reset_and_halt_after {inst_count wait_ms} {
    # 1. Hold + pulse (CPU stays in reset across the pulse and beyond).
    unified_reset 1
    # 2. Stage halt-after counters + enable halt-after, clear latched.
    set lo [expr {$inst_count & 0xFFFFFFFF}]
    set hi [expr {($inst_count >> 32) & 0xFFFFFFFF}]
    dbg_wr $::OFF_HALT_AFTER_LO $lo
    dbg_wr $::OFF_HALT_AFTER_HI $hi
    dbg_wr $::OFF_HALT_CTL [expr {$::HALT_AFTER_EN | $::HALT_CLEAR}]
    # 3. Release hold — CPU starts fetching, halt-after armed.
    unified_reset_release
    after $wait_ms
}

# `reset_and_halt_exc` — umbrella unified-reset (boot_fsm re-runs the
# SD->DDR copy, peripherals re-init) + halt-on-exception.
#
# CORRECTED 2026-08-27 (vio-full-cold-reset investigation): this comment
# used to claim `vio_reset_halt_exc` (VIO probe_out0 bit 3) pulses
# "full_dbg_rst" and does NOT re-run boot_fsm, leaving the boot on stale
# DRAM.  That was never true against the RTL as far back as task #256
# (rtl/soc/fpga_top_clocks.vh, commit 642614fc, landed BEFORE this comment
# was even written) — verified again against current HEAD:
#   vio_boot_ctrl[3] (jtag_debug_full_reset) and DBG_CONTROL bit 5
#   (dbg_cold_reset_pulse, the canonical `reset` trigger) are OR'd into
#   the SAME `dbg_rst_src_level` (fpga_top_clocks.vh) and drive the IDENTICAL
#   `jtag_debug_full_reset_eff` pulse into both clk_rst instances, which
#   produces `soc_full_rst`.  `boot_fsm_rst = soc_full_rst || jtag_boot_bypass`
#   (jtag_boot_bypass is tied 0 on real hardware), so a VIO-bit-3 reset
#   re-arms boot_fsm's SD->DDR ROM copy exactly like `reset` does — an
#   independent tool (synth/ila_l2c_forced_reset_capture.tcl) already
#   relies on this ("Pulsing VIO debug-full-reset ... to force a fresh
#   boot_fsm run").  See docs/reset_story.md §1.1/§4.3 for the design intent
#   (VIO bit 3 was always meant to be an alternate trigger into the same
#   unified pulse, not a separate reset tree).
#
# The REAL distinction between `reset`/`vio_reset_halt_exc` (both "warm")
# and a true cold boot is the 256 MiB RAM pre-zero pass
# (rtl/soc/fpga_top_boot_master.vh `boot_warm_q`): it only runs when
# `core_rst_bank[6]` fires, i.e. board power-on / btn[3] / the SEPARATE
# `vio-hard-reset` command (VIO probe_out1) — NOT on either bit-3/bit-5
# pulse.  Skipping the zero pass is intentional (saves ~4 s per iteration);
# `vio-hard-reset` already exists as the "genuinely fresh DRAM, VIO-driven,
# also recovers a wedged JTAG-AXI bridge" option when that matters — see
# its own doc comment below. Use `vio-hard-reset` (not `vio_reset_halt_exc`)
# when the RAM-zero pass itself is what a repro needs.
#
# `reset_and_halt_exc` remains preferable to `vio_reset_halt_exc` for the
# reason `unified_reset` documents (JTAG-AXI CONTROL writes are less prone
# to leaving cold_reset_hold stuck than the VIO probe path historically
# was) — but NOT because it uniquely reproduces a fresh-DRAM boot; both
# commands do.
# Make OFF_HALT_CTL bit 6 (halt_exc_enable) agree with the mask lanes.
#
# debug_stop_manager.v:142-145 ANDs the enable with the per-vector mask, so a
# mask written without the enable arms a vector that can never halt.  The
# interactive `halt-exc-mask` command did exactly that until 2026-08-03 — see
# the long note at its implementation, and task #235.
#
# Enable when ANY lane is non-zero, disable when all lanes are clear, so
# `halt-exc-mask <vec> off` on the last armed vector genuinely disarms rather
# than leaving a live enable behind for the next halt to be misattributed to.
#
# Read-modify-write masked to the ENABLE bits (0/1/6).  A bare write would
# clear halt_after_enable, which debug_ctrl.v:2218 assigns unconditionally
# from bit 0 — the same footgun the RTL comment at 2219-2232 documents for
# break-PC.  Bit 2 (HALT_CLEAR) reads back 0 (debug_ctrl.v:1503), so the
# readback cannot accidentally re-acknowledge a halt.
proc halt_exc_enable_sync {} {
    set any 0
    for {set i 0} {$i < 8} {incr i} {
        if {[scan [dbg_rd [expr {$::OFF_HALT_EXC_MASK + $i*4}]] %x] != 0} {
            set any 1
            break
        }
    }
    # The m68k040 Stage-5 contract makes a non-zero mask itself authoritative;
    # HALT_CTL bit 6 is a legacy-controller enable and is RAZ/WI on this core.
    if {[dbg_is_core040_epoch]} { return $any }
    set cur [scan [dbg_rd $::OFF_HALT_CTL] %x]
    set en_bits [expr {$cur & ($::HALT_AFTER_EN | 0x2 | $::HALT_EXC_EN)}]
    set new [expr {$any ? ($en_bits | $::HALT_EXC_EN)
                        : ($en_bits & ~$::HALT_EXC_EN)}]
    if {$new != $en_bits} {
        dbg_wr $::OFF_HALT_CTL $new
    }
    return $any
}

proc reset_and_halt_exc {vec wait_ms} {
    set lane [expr {($vec >> 5) & 0x7}]
    set bit  [expr {$vec & 0x1f}]
    set mask [expr {1 << $bit}]
    # 1. Hold + pulse (CPU stays in reset across the unified-reset pulse).
    unified_reset 1
    # 2. Arm halt-on-exc mask for $vec + enable exc-halt, clear latched.
    for {set i 0} {$i < 8} {incr i} {
        dbg_wr [expr {$::OFF_HALT_EXC_MASK + $i*4}] 0
    }
    dbg_wr [expr {$::OFF_HALT_EXC_MASK + $lane*4}] $mask
    dbg_wr $::OFF_HALT_CTL [expr {$::HALT_EXC_EN | $::HALT_CLEAR}]
    # 3. Release hold — CPU cold-boots with the exc-halt armed.
    unified_reset_release
    after $wait_ms
    puts "> [halt_status_line]"
    puts "> reset-halt-exc vec=$vec lane=$lane mask=[format 0x%08X $mask] waited=${wait_ms}ms"
}

# `reset_and_break_pc` — arm a PC breakpoint BEFORE the CPU starts
# fetching, then cold-boot into it.
#
# This is the operation that was impossible before the debug reset domain
# landed: debug_ctrl ran on cpu_rst, which the SoC holds for the entire
# MIG-calibration + boot_fsm SD->DDR window, so (a) the arm was wiped by
# the reset and (b) the arming write itself was swallowed by a CSR block
# held in reset.  One-shot early-boot events were therefore unobservable.
#
# Requires a bitstream advertising OFF_FEATURES bit 0 (dbg_reset_domain);
# refuses to run otherwise rather than appearing to work and silently
# arming nothing.
proc reset_and_break_pc {pc {slot 0} {wait_ms 2000}} {
    if {![dbg_has_feature dbg_reset_domain]} {
        error "reset-and-break-pc: this bitstream has no debug reset domain (OFF_FEATURES bit 0 clear), so the arm would be wiped by the reset. Use `reset` then `break-pc` after the boot completes, or load a newer bitstream."
    }
    # Validate BEFORE asserting reset.  arm_break_pc rejects slot > 3, but
    # it does so from INSIDE the hold window below, and an error there used
    # to abort the proc before the release -- leaving the CPU held in reset
    # with every debug CSR reading 0x00000000.  From outside that is
    # indistinguishable from a wedged JTAG bridge, and it is very likely
    # what an earlier session misdiagnosed as exactly that.  (Tell them
    # apart with `build_id`: it lives outside the CPU reset domain, so if
    # build_id reads correctly while halt-status reads 0, the CPU is held
    # in reset -- recover with `reset release`, not a REPL relaunch.)
    #
    # The usage error that found this: `reset-and-break-pc <pc> <wait_ms>`
    # puts wait_ms in the SLOT position, because the signature is
    # {pc {slot 0} {wait_ms 2000}}.
    if {![string is integer -strict $slot] || $slot < 0 || $slot > 3} {
        error "reset-and-break-pc: slot must be 0..3, got '$slot' -- note the signature is `reset-and-break-pc <pc> \[slot\] \[wait_ms\]`, so a bare second argument is the SLOT, not the wait"
    }
    if {![string is integer -strict $wait_ms] || $wait_ms < 0} {
        error "reset-and-break-pc: wait_ms must be a non-negative integer, got '$wait_ms'"
    }
    # 1. Hold the CPU + fire the unified reset.
    unified_reset 1
    # 2. Arm while held.  With the debug reset domain this STICKS -- the
    #    CSR block is not in reset and the config half of the register file
    #    is not touched by the CPU-reset event.
    #
    #    Guarded: ANY failure between the hold above and the release below
    #    must still release, or the CPU is left in reset and the operator
    #    is handed a fake wedge.  Release first, then re-raise.
    if {[catch {arm_break_pc $pc $slot} armerr]} {
        catch {unified_reset_release}
        error "reset-and-break-pc: arm failed, CPU reset released so the board is usable: $armerr"
    }
    # 3. Release and wait for the breakpoint to fire.
    unified_reset_release
    after $wait_ms
    # 4. Verify the arm actually survived, and say so explicitly.  A silent
    #    "no hit" is ambiguous between "never reached" and "was never
    #    armed"; this disambiguates it.
    set mask [break_pc_enable_mask]
    if {!(($mask >> $slot) & 1)} {
        puts "> reset-and-break-pc: WARNING slot $slot enable did NOT survive the reset (mask=[format 0x%X $mask]) -- the arm was lost, so a no-hit result means nothing"
    } else {
        puts "> reset-and-break-pc: slot $slot still armed after reset (mask=[format 0x%X $mask])"
    }
    puts "> [halt_status_line]"
}

proc vio_reset_and_halt_after {inst_count wait_ms} {
    set lo [expr {$inst_count & 0xffffffff}]
    set hi [expr {($inst_count >> 32) & 0xffffffff}]

    # Hold CPU without using the JTAG-AXI cold-reset pulse.  The current
    # loaded bitstream cannot reliably clear cold_reset_hold while the
    # JTAG pulse path keeps soc_full_rst asserted; VIO reset does not have
    # that failure mode.
    dbg_wr $::OFF_CONTROL $::CTL_COLD_RESET_HOLD
    after 20
    vio_set 8
    after 100
    vio_set 0
    after 900

    # Stage halt-after while the CPU is still held, then release hold and
    # wait for the auto-halt.
    #
    # (Historical note: this used to carry the comment "counters_clear
    # wipes halt-after".  That was never live -- m68k_axi_wrapper tied
    # counters_clear to 1'b0 -- and it is doubly untrue now: since the
    # debug reset domain landed, counters_clear is the CPU-reset event and
    # explicitly does NOT clear host configuration such as the halt-after
    # target.  Staging before the release is kept because it closes the
    # race window, not because a wipe is expected.)
    dbg_wr $::OFF_HALT_AFTER_LO $lo
    dbg_wr $::OFF_HALT_AFTER_HI $hi
    dbg_wr $::OFF_HALT_CTL [expr {$::HALT_AFTER_EN | $::HALT_CLEAR}]
    dbg_wr $::OFF_CONTROL 0x0
    after $wait_ms

    puts "> [halt_status_line]"
    puts "> inst-count = 0x[dbg_rd $::OFF_HALT_HIT_INST_HI][dbg_rd $::OFF_HALT_HIT_INST_LO]"
}

proc vio_reset_halt_exc {vec wait_ms} {
    set lane [expr {($vec >> 5) & 0x7}]
    set bit  [expr {$vec & 0x1f}]
    set mask [expr {1 << $bit}]

    dbg_wr $::OFF_CONTROL $::CTL_COLD_RESET_HOLD
    after 20
    vio_set 8
    after 100
    vio_set 0
    after 900

    for {set i 0} {$i < 8} {incr i} {
        dbg_wr [expr {$::OFF_HALT_EXC_MASK + $i*4}] 0
    }
    dbg_wr [expr {$::OFF_HALT_EXC_MASK + $lane*4}] $mask
    dbg_wr $::OFF_HALT_CTL [expr {$::HALT_EXC_EN | $::HALT_CLEAR}]
    dbg_wr $::OFF_CONTROL 0x0
    after $wait_ms

    puts "> [halt_status_line]"
    puts "> halt-exc vec=$vec lane=$lane mask=[format 0x%08X $mask]"
}

# DEPRECATED: kept for one release per docs/reset_story.md §4.7.
proc reset_halt_after {inst_count wait_ms} {
    puts "> WARNING reset-halt-after is DEPRECATED; use `reset-and-halt-after` (canonical: `reset` family)"
    reset_and_halt_after $inst_count $wait_ms
}

# ── Effective-halt gate ─────────────────────────────────────────────────
# HALT_REASON bit 3 = EFFECTIVE halt: the CPU is genuinely stopped at a
# retire boundary.  Every other bit in that register is a *request* or a
# *latch* — "halt requested" is not "halt landed", and the gap between them
# is where this REPL has historically produced its most expensive wrong
# answers.  Reading core state (live-arch), mutating it (arch-apply), or
# running a cache maintenance walk while the CPU is still executing does
# not fail: it returns plausible, stale, wrong numbers.
#
# Single definition on purpose.  This bit test used to be open-coded at
# each call site, which is how some sites got it and others didn't.
proc effective_halt {} {
    if {[dbg_is_core040_epoch]} {
        return [expr {([scan [dbg_rd $::OFF_STATUS] %x] & 0x01) ? 1 : 0}]
    }
    return [expr {([scan [dbg_rd $::OFF_HALT_REASON] %x] & 0x08) ? 1 : 0}]
}

# Ask the core to stop, then wait for the *acknowledged* architectural halt.
# CONTROL.HALT_REQ is only a request: the Stage-5 stop manager waits until the
# current macro has committed, flushes younger/in-flight work, and asserts
# STATUS.halted only once the live register snapshot is coherent.  Never make
# callers infer completion from the request bit itself.
proc request_debug_halt {{wait_ms 200}} {
    if {$wait_ms < 0} { error "halt: wait_ms must be >= 0" }
    dbg_wr $::OFF_CONTROL $::CTL_HALT_REQ
    for {set elapsed 0} {$elapsed <= $wait_ms} {incr elapsed} {
        if {[effective_halt]} { return $elapsed }
        if {$elapsed < $wait_ms} { after 1 }
    }
    error "halt timeout after ${wait_ms}ms: halt requested but effective halt did not land\n> [halt_status_line]"
}

# Refuse `what` unless the CPU is effectively halted.  `hint` should name
# the concrete thing to do instead — a refusal that doesn't tell you the
# way forward just gets forced past.
proc require_effective_halt {what {hint ""}} {
    if {[effective_halt]} { return }
    set source [expr {[dbg_is_core040_epoch] ? {STATUS bit 0} : {HALT_REASON bit 3}}]
    set msg "$what requires an EFFECTIVE halt ($source) — the CPU is RUNNING, so this would not do what you think."
    if {$hint ne ""} { append msg "\n>        $hint" }
    append msg "\n>        [halt_status_line]"
    error $msg
}

proc arch_reg_offset {name} {
    set n [string toupper $name]
    if {[regexp {^D([0-7])$} $n -> i]} { return [expr {$::OFF_ARCH_D0 + $i*4}] }
    if {[regexp {^A([0-7])$} $n -> i]} { return [expr {$::OFF_ARCH_A0 + $i*4}] }
    foreach pair {
        {USP OFF_ARCH_USP} {MSP OFF_ARCH_SSP} {SSP OFF_ARCH_SSP} {ISP OFF_ARCH_ISP}
        {SR OFF_ARCH_SR} {VBR OFF_ARCH_VBR} {CACR OFF_ARCH_CACR} {TC OFF_ARCH_TC}
        {ITT0 OFF_ARCH_ITT0} {ITT1 OFF_ARCH_ITT1} {DTT0 OFF_ARCH_DTT0} {DTT1 OFF_ARCH_DTT1}
        {URP OFF_ARCH_URP} {SRP OFF_ARCH_SRP} {PC OFF_ARCH_PC} {SFC OFF_ARCH_SFC} {DFC OFF_ARCH_DFC}
    } {
        lassign $pair reg var
        if {$n eq $reg} { return [set ::$var] }
    }
    error "unknown architectural register '$name' (D0-D7, A0-A7, USP/MSP/ISP, SR, VBR, CACR, TC, ITT0/1, DTT0/1, URP/SRP, PC, SFC/DFC)"
}

proc arch_apply_wait {} {
    require_core040_debug_epoch "arch-apply"
    if {![dbg_has_feature arch_apply_stays_halted] || ![dbg_has_feature arch_dirty_apply]} {
        error "arch-apply: hardware does not advertise halted atomic dirty apply"
    }
    require_effective_halt "arch-apply"
    dbg_wr $::OFF_ARCH_APPLY [expr {$::ARCH_APPLY_CLEAR_STATUS | $::ARCH_APPLY_START}]
    set status 0
    for {set i 0} {$i < 10000} {incr i} {
        set status [scan [dbg_rd $::OFF_ARCH_STATUS] %x]
        if {($status & $::ARCH_STATUS_BUSY) == 0 &&
            ($status & ($::ARCH_STATUS_DONE | $::ARCH_STATUS_REJECTED)) != 0} { break }
        after 1
    }
    set done [expr {($status & $::ARCH_STATUS_DONE) != 0}]
    set rejected [expr {($status & $::ARCH_STATUS_REJECTED) != 0}]
    if {!$done || $rejected} {
        error "arch-apply failed: status=[format 0x%08X $status] done=$done rejected=$rejected"
    }
    if {![effective_halt]} { error "arch-apply completed but released effective halt" }
    return $status
}

proc dump_arch_registers {} {
    require_core040_debug_epoch "register dump"
    if {![dbg_has_feature live_arch]} {
        error "register dump: hardware does not advertise live_arch"
    }
    require_effective_halt "register dump"
    for {set i 0} {$i < 8} {incr i} {
        puts "> D$i = 0x[dbg_rd [expr {$::OFF_LIVE_D0 + $i*4}]]"
    }
    for {set i 0} {$i < 8} {incr i} {
        puts "> A$i = 0x[dbg_rd [expr {$::OFF_LIVE_A0 + $i*4}]]"
    }
    foreach item {
        {SR OFF_LIVE_SR} {VBR OFF_LIVE_VBR} {USP OFF_LIVE_USP} {MSP OFF_LIVE_SSP}
        {ISP OFF_LIVE_ISP} {PC OFF_LIVE_PC} {CACR OFF_LIVE_CACR} {SFC OFF_LIVE_SFC}
        {DFC OFF_LIVE_DFC} {TC OFF_LIVE_MMU_TC} {ITT0 OFF_LIVE_MMU_ITT0}
        {ITT1 OFF_LIVE_MMU_ITT1} {DTT0 OFF_LIVE_MMU_DTT0} {DTT1 OFF_LIVE_MMU_DTT1}
        {URP OFF_LIVE_MMU_URP} {SRP OFF_LIVE_MMU_SRP} {MMUSR OFF_LIVE_MMUSR}
    } {
        lassign $item name var
        puts "> $name = 0x[dbg_rd [set ::$var]]"
    }
}

proc halt_status_line {} {
    set hr [dbg_rd $::OFF_HALT_REASON]
    set hc [dbg_rd $::OFF_HALT_CTL]
    set hp [dbg_rd $::OFF_HALT_HIT_PC]
    set mp [dbg_rd $::OFF_PC_MISALIGNED_PC]
    set pc [dbg_rd $::OFF_PC]
    set ev [dbg_rd $::OFF_EXC_VEC]
    set ep [dbg_rd $::OFF_EXC_PC]
    set ec [dbg_rd $::OFF_EXC_COUNT]
    set st [dbg_rd $::OFF_STATUS]
    set hri [scan $hr %x]
    set hci [scan $hc %x]
    if {[dbg_is_core040_epoch]} {
        # Stage-5 primary codes: 0 none, 1 manual, 2 step, 3 halt-after,
        # 4 fatal, 5 precise PC breakpoint, 6 completed exception entry.
        set code [expr {$hri & 0x7}]
        set manual [expr {$code == 1}]
        set halt_after [expr {$code == 3}]
        set break_pc [expr {$code == 5}]
        set exc_halt [expr {$code == 6}]
        set effective [expr {([scan $st %x] & 0x1) != 0}]
        set dbl_fault 0
        set pc_misaligned 0
        set bp_mask [break_pc_enable_mask]
        set exc_en 0
        for {set i 0} {$i < 8} {incr i} {
            if {[scan [dbg_rd [expr {$::OFF_HALT_EXC_MASK + $i*4}]] %x] != 0} {
                set exc_en 1; break
            }
        }
        set enables [format "ha=%d bp=%d exc=%d pcmis=0" \
            [expr {$hci & 1}] [expr {$bp_mask != 0}] $exc_en]
        set ctl_latches [format "auto=%d ha=%d bp_skip=0x%X exc_pending=%d" \
            [expr {([scan $st %x] >> 4) & 1}] [expr {$hci & 1}] \
            [expr {[scan [dbg_rd $::OFF_BP_SKIP_ONCE] %x] & 0xF}] \
            [expr {([scan $st %x] >> 1) & 1}]]
    } else {
        set manual [expr {($hri & 0x01) ? 1 : 0}]
        set halt_after [expr {($hri & 0x02) ? 1 : 0}]
        set break_pc [expr {($hri & 0x04) ? 1 : 0}]
        set effective [expr {($hri & 0x08) ? 1 : 0}]
        set exc_halt [expr {($hri & 0x40) ? 1 : 0}]
        set dbl_fault [expr {($hri & 0x100) ? 1 : 0}]
        set pc_misaligned [expr {($hri & 0x200) ? 1 : 0}]
        set enables [format "ha=%d bp=%d exc=%d pcmis=%d" \
            [expr {($hri & 0x10) ? 1 : 0}] [expr {($hri & 0x20) ? 1 : 0}] \
            [expr {($hri & 0x80) ? 1 : 0}] [expr {($hri & 0x400) ? 1 : 0}]]
        set ctl_latches [format "auto=%d ha=%d bp=%d exc=%d" \
            [expr {($hci & 0x08) ? 1 : 0}] [expr {($hci & 0x10) ? 1 : 0}] \
            [expr {($hci & 0x20) ? 1 : 0}] [expr {($hci & 0x80) ? 1 : 0}]]
    }

    set suffix ""
    if {$::last_requested_break_pc ne ""} {
        set want [expr {$::last_requested_break_pc & 0xffffffff}]
        set got [scan $hp %x]
        if {!$break_pc || $got != $want} {
            append suffix " note=break-pc-not-reached expected=[format 0x%08X $want]"
        }
    } elseif {(([dbg_is_core040_epoch] && [break_pc_enable_mask] != 0) ||
                (![dbg_is_core040_epoch] && ($hri & 0x20))) && !$break_pc} {
        append suffix " note=break-pc-armed-no-hit"
    }
    if {!$effective} {
        append suffix " note=not-halted-live-arch-unsafe"
    }
    if {$dbl_fault} {
        # Read captured PC + vec.  OFF_DBL_FAULT_VEC packs {latched,vec}.
        set dfpc [dbg_rd $::OFF_DBL_FAULT_PC]
        set dfvi [dbg_rd $::OFF_DBL_FAULT_VEC]
        set dfvec [format "0x%02x" [expr {[scan $dfvi %x] & 0xff}]]
        append suffix " DBL_FAULT pc=0x$dfpc vec=$dfvec"
    }

    if {$hri == 7} {
        set prc [dbg_rd $::OFF_PCRANGE_COUNT]
        if {[scan $prc %x] != 0} {
            append suffix " PC_RANGE pc0=0x[dbg_rd $::OFF_PCRANGE_PC0] pc1=0x[dbg_rd $::OFF_PCRANGE_PC1]"
            append suffix " pc2=0x[dbg_rd $::OFF_PCRANGE_PC2] count=0x$prc"
        }
    }
    if {$hri == 7} {
        # cpu040 A7-ODD halt lane (RobPlugin a7OddLane, DebugHaltReasonCode.A7_ODD).
        append suffix " A7_ODD pc0=0x[dbg_rd $::OFF_A7ODD_PC0] pc1=0x[dbg_rd $::OFF_A7ODD_PC1]"
        append suffix " pc2=0x[dbg_rd $::OFF_A7ODD_PC2] a7=0x[dbg_rd $::OFF_A7ODD_VALUE]"
        append suffix " episodes=0x[dbg_rd $::OFF_A7ODD_COUNT]"
    }
    return "halt: reason=0x$hr ctl=0x$hc hit=0x$hp pc_live=0x$pc manual=$manual halt_after=$halt_after break_pc=$break_pc exc_halt=$exc_halt dbl_fault=$dbl_fault effective=$effective enables={$enables} latches={$ctl_latches} exc_vec=0x$ev exc_pc=0x$ep exc_count=0x$ec$suffix\npc_misaligned=$pc_misaligned  misaligned_pc=0x$mp"
}

# Is the CPU currently halted ON a precise PC breakpoint?
#
# EPOCH-CORRECT.  The Stage 5 core reports its halt source as a PRIMARY REASON
# CODE in OFF_HALT_REASON[2:0] (5 = precise PC breakpoint).  The legacy
# controller's HALT_CTL bit 5 (break_pc_latched) is RAZ on this core -- see the
# RAZ/WI note on HALT_CTL above -- so testing that bit silently returned 0 for
# EVERY core040 breakpoint halt.  `continue`/`step` then left skip_once unarmed,
# decode re-injected SYS_DBG_BREAK on the very next fetch of break_pc, and the
# breakpoint re-fired with NO FORWARD PROGRESS (observed on hardware
# 2026-08-22: a second breakpoint armed while stopped on the first never
# released the original halt).
proc bp_halt_latched {} {
    if {[dbg_is_core040_epoch]} {
        return [expr {([scan [dbg_rd $::OFF_HALT_REASON] %x] & 0x7) == 5}]
    }
    return [expr {([scan [dbg_rd $::OFF_HALT_CTL] %x] & 0x20) != 0}]
}

# Did the break-PC we last armed actually fire at `want`?
proc break_pc_reached {want} {
    set hri [scan [dbg_rd $::OFF_HALT_REASON] %x]
    if {[dbg_is_core040_epoch]} {
        if {($hri & 0x7) != 5} { return 0 }
    } elseif {!($hri & 0x04)} { return 0 }
    return [expr {[scan [dbg_rd $::OFF_HALT_HIT_PC] %x] == ($want & 0xffffffff)}]
}

# Explain a break-pc that never fired.
#
# The bare `note=break-pc-not-reached` reads as "that code never ran", which
# is frequently the OPPOSITE of the truth.  break-pc fires at RETIRE, and a
# faulting instruction never retires — it is squashed and the exception is
# taken instead.  So a breakpoint set on an instruction already observed to
# trap CANNOT fire, no matter how long you wait, and the tool has been
# reporting that impossibility as absence of execution.
proc break_pc_not_reached_note {want} {
    puts "> ------------------------------------------------------------"
    puts "> break-pc at [format 0x%08X $want] DID NOT FIRE."
    if {[dbg_is_core040_epoch]} {
        puts ">   This core's PC breakpoint is PRE-EFFECT at the commit head, so"
        puts ">   a faulting instruction is catchable before it raises its exception."
        puts ">   A no-hit result therefore means the armed PC did not reach the"
        puts ">   architectural head (or the slot/configuration was changed)."
    } else {
        puts ">   This does NOT prove the PC was never executed."
        puts ">   Legacy break-pc halts at RETIRE.  An instruction that TRAPS never"
        puts ">   retires, so a breakpoint on a faulting instruction cannot fire."
    }
    puts ">   If this PC is known or suspected to trap, catch it by VECTOR"
    puts ">   instead:   reset-halt-exc <vec>      (e.g. reset-halt-exc 2)"
    puts ">   and cross-check with `exc-ring` / `pc-trace`, which record"
    puts ">   PCs that faulted rather than PCs that retired."
    puts "> ------------------------------------------------------------"
}

proc halt_enable_bits {} {
    set ctl [scan [dbg_rd $::OFF_HALT_CTL] %x]
    if {[dbg_is_core040_epoch]} { return [expr {$ctl & $::HALT_AFTER_EN}] }
    return [expr {$ctl & ($::HALT_AFTER_EN | $::HALT_BREAK_PC_EN | $::HALT_EXC_EN)}]
}

proc break_pc_offset {slot} {
    switch -- $slot {
        0 { return $::OFF_BREAK_PC }
        1 { return $::OFF_BREAK_PC1 }
        2 { return $::OFF_BREAK_PC2 }
        3 { return $::OFF_BREAK_PC3 }
        default { error "break-pc slot must be 0..3" }
    }
}

proc break_pc_enable_mask {} {
    return [expr {[scan [dbg_rd $::OFF_BREAK_PC_CTRL] %x] & 0xF}]
}

proc write_break_pc_enable_mask {mask} {
    set ctrl [scan [dbg_rd $::OFF_BREAK_PC_CTRL] %x]
    if {$ctrl & 0x8000} {
        set clear_hit 0x4000
    } else {
        set clear_hit 0
    }
    dbg_wr $::OFF_BREAK_PC_CTRL [expr {$clear_hit | ($mask & 0xF)}]
}

# Reject an unprefixed PC operand.  `expr` parses a bare token as DECIMAL,
# so `break-pc 40899706` armed address 0x02705F5A instead of 0x40899706 —
# the breakpoint then never fired and the run was misread as a wedge
# (hit live, 2026-09-06).  Every other address operand here is hex; refuse
# rather than guess the base.  A leading +/- is tolerated so arithmetic-ish
# forms still error with the same clear message.
proc require_hex_pc {tok} {
    if {![string match -nocase "0x*" [string trimleft $tok "+-"]]} {
        error "PC operand must be hex with an explicit 0x prefix (got '$tok').\
A bare operand is parsed as DECIMAL and would arm the wrong address."
    }
}

proc next_free_break_pc_slot {} {
    set mask [break_pc_enable_mask]
    for {set slot 0} {$slot < 4} {incr slot} {
        if {(($mask >> $slot) & 1) == 0} { return $slot }
    }
    error "all 4 break-pc slots are enabled"
}

proc clear_auto_halt {{extra_enable_bits -1}} {
    if {$extra_enable_bits < 0} {
        set bits [halt_enable_bits]
    } else {
        set bits $extra_enable_bits
    }
    dbg_wr $::OFF_HALT_CTL [expr {$bits | $::HALT_CLEAR}]
}

# ── data-watchpoint helpers ────────────────────────────────────────────
# Encode WPn_CTRL from the human-facing arguments.  Kept as a proc (rather
# than inline in the `watch` dispatch) so `make tb-jtag-repl-host` can
# regression-test the encoding without a board.
#   kind    : r | w | rw
#   value   : "" for no value filter, else the compare value
#   lanes   : 4-bit wstrb-style lane mask, bit 3 = lowest-addressed byte
proc wp_ctrl_encode {kind value lanes} {
    if {$kind ni {r w rw}} { error "watch: kind must be r, w or rw" }
    set ctrl 1
    if {$kind in {r rw}} { set ctrl [expr {$ctrl | 0x2}] }
    if {$kind in {w rw}} { set ctrl [expr {$ctrl | 0x4}] }
    if {$value ne ""} {
        if {($lanes & 0xF) == 0} {
            error "watch: value filter needs a non-zero lane mask"
        }
        set ctrl [expr {$ctrl | 0x8 | (($lanes & 0xF) << 8)}]
    }
    return $ctrl
}

# The lane a single byte at physical address `addr` occupies on the 32-bit
# data bus (big-endian packing: byte at the lowest address is in bits
# [31:24] = lane 3).  Returned as {lane_mask shift_bits}.
proc wp_byte_lane {addr} {
    set lane [expr {3 - ($addr & 3)}]
    return [list [expr {1 << $lane}] [expr {8 * $lane}]]
}

# Hardware requires EVERY armed lane to be written by the access.  A value
# filter armed over lanes the access cannot carry silently never fires, so
# warn when the arm looks like the classic mistake (a small value left in
# the low byte with the default all-lanes mask).
proc wp_lane_warning {addr value lanes} {
    if {$value eq ""} { return "" }
    lassign [wp_byte_lane $addr] byte_lane byte_shift
    if {($lanes & 0xF) == 0xF && $value == ($value & 0xFF) && $value != 0} {
        return [format "watch: value 0x%X with lanes 0xF only matches a FULL longword store; for the single byte at 0x%08X use `value 0x%08X lanes 0x%X`" \
                    $value $addr [expr {($value & 0xFF) << $byte_shift}] $byte_lane]
    }
    return ""
}

proc release_debug_halt {} {
    clear_auto_halt
    dbg_wr $::OFF_CONTROL 0x0
}

# Clear the manual-halt request bit ONLY, preserving every other bit
# OFF_CONTROL currently holds -- most importantly CTL_COLD_RESET_HOLD.
#
# `break-pc` and `advance` both used to do a bare `dbg_wr $::OFF_CONTROL
# 0x0` here, copying `release_debug_halt`'s idiom without its INTENT:
# `release_debug_halt`/`halt-release` deliberately zero the whole register
# (its own status line prints "DBG_CONTROL=0" -- that's the documented,
# wanted behavior for a full release). `break-pc`/`advance` only ever
# meant "let the CPU run so it can reach the target" -- but a bare zero
# write silently also clears `cold_reset_hold` if a `reset hold` sequence
# is still staged, releasing the CPU mid-arm with no error or warning.
# Found 2026-08-27 while investigating a boot-order question: arming
# `break-pc` right after `reset hold` let the CPU run *before* `reset
# release` was ever issued, invalidating the ordering test that depended
# on the CPU staying held. Use this everywhere the intent is "resume from
# a manual halt," not "abandon any other staged CONTROL state too."
proc clear_halt_req_preserve_control {} {
    set cur 0x[dbg_rd $::OFF_CONTROL]
    dbg_wr $::OFF_CONTROL [expr {$cur & ~$::CTL_HALT_REQ}]
}

proc arm_break_pc {pc {slot 0}} {
    if {$slot < 0 || $slot > 3} { error "break-pc slot must be 0..3" }
    set bits $::HALT_BREAK_PC_EN
    # Clear THIS slot's skip-once bit before arming (read-modify-write,
    # preserving any other slot's bit -- same convention `atrap_disarm`
    # already uses for the sibling A-trap skip-once register).
    #
    # `breakSkipOnce` (DebugCtrlPlugin.scala) is a per-slot "swallow the
    # next match, no halt" latch that hardware sets unconditionally on
    # EVERY genuine breakpoint hit, and that lives in the debug-config
    # domain -- which is explicitly designed to survive a CPU/core reset
    # (including the one `reset-and-break-pc` itself fires) and is only
    # ever cleared by a full `cfgWipe` or by the match that consumes it.
    # A fresh arm on a slot that previously hit -- e.g. a prior
    # `reset-and-break-pc`/`break-pc` trial on the same slot, with no
    # intervening bitstream reload -- therefore inherits a stale set skip
    # bit and silently swallows the VERY NEXT genuine hit (no halt, no
    # error): a construction-guaranteed HIT/skip/HIT/skip toggle
    # indistinguishable from a 50% race unless this is cleared. Root-caused
    # 2026-09-03 (docs/BUG_calibration_word_misplaced_0d00.md Part 98);
    # this is the fix Part 98 recommended and left unimplemented.
    #
    # Deliberately NOT touched by `continue`/`step`, which explicitly SET
    # this same bit right before resuming past a currently-halted
    # breakpoint -- that is the one legitimate, intended use of a set skip
    # bit, and arming a *different*, fresh breakpoint must not disturb it.
    set so [expr {[scan [dbg_rd $::OFF_BP_SKIP_ONCE] %x] & 0xF}]
    set so_new [expr {$so & ~(1 << $slot)}]
    if {$so_new != $so} {
        dbg_wr $::OFF_BP_SKIP_ONCE $so_new
    }
    dbg_wr [break_pc_offset $slot] $pc
    write_break_pc_enable_mask [expr {[break_pc_enable_mask] | (1 << $slot)}]
    clear_auto_halt $bits
    set ::last_requested_break_pc [expr {$pc & 0xffffffff}]
}

proc disable_break_pc {{slot ""}} {
    if {$slot eq ""} {
        set mask 0
        set bits [expr {[halt_enable_bits] & ~$::HALT_BREAK_PC_EN}]
        set ::last_requested_break_pc ""
    } else {
        if {$slot < 0 || $slot > 3} { error "break-pc slot must be 0..3" }
        set mask [expr {[break_pc_enable_mask] & ~(1 << $slot)}]
        set bits [halt_enable_bits]
        if {$mask == 0} { set bits [expr {$bits & ~$::HALT_BREAK_PC_EN}] }
    }
    write_break_pc_enable_mask $mask
    clear_auto_halt $bits
}

proc list_break_pc {} {
    set mask [break_pc_enable_mask]
    for {set slot 0} {$slot < 4} {incr slot} {
        set pc [dbg_rd [break_pc_offset $slot]]
        set en [expr {($mask >> $slot) & 1}]
        puts "> break-pc slot=$slot enable=$en pc=0x$pc"
    }
}

# Arm one A-trap breakpoint slot.  Stages MATCH (and D0VAL, if the D0
# qualifier is requested) BEFORE writing CTRL's enable bit, so the slot is
# never observably half-armed; every write is readback-verified via
# atrap_wr_verify.  Clears the shared hit latch unconditionally on arm,
# same convention as `watch`.
proc atrap_arm {slot value mask d0qual d0val} {
    set ctrl_off  [atrap_ctrl_off $slot]
    set match_off [expr {$ctrl_off + 4}]
    set d0_off    [expr {$ctrl_off + 8}]
    set match_packed [atrap_pack_match $value $mask]
    atrap_wr_verify $match_off $match_packed "atrap${slot} MATCH"
    if {$d0qual} {
        atrap_wr_verify $d0_off $d0val "atrap${slot} D0VAL"
    }
    set ctrl [expr {0x1 | ($d0qual ? 0x2 : 0x0)}]
    atrap_wr_verify $ctrl_off $ctrl "atrap${slot} CTRL"
    dbg_wr $::OFF_ATRAP_HIT 1
    puts [format "> atrap%d armed: value=0x%04X mask=0x%04X match_set(value&mask)=0x%04X%s" \
              $slot $value $mask [expr {$value & $mask}] \
              [expr {$d0qual ? [format " d0qual=1 d0val=0x%08X" $d0val] : ""}]]
    if {$mask != 0xFFFF} {
        puts [format "> atrap%d note: mask 0x%04X is not exact -- this arms a FAMILY of opcodes (any opword where (opword ^ 0x%04X) & 0x%04X == 0), not just the single value given" \
                  $slot $mask $value $mask]
    }
}

# Disarm one A-trap breakpoint slot: CTRL -> 0 (verified), clear this
# slot's skip-once bit (preserving the other slot's), and clear the shared
# hit latch ONLY if this slot is the one that owns it -- clearing an
# unrelated slot's real hit on disarm would silently drop it.
proc atrap_disarm {slot} {
    set ctrl_off [atrap_ctrl_off $slot]
    atrap_wr_verify $ctrl_off 0 "atrap${slot} CTRL"
    set so [rdx [expr {$::DBG_BASE + $::OFF_ATRAP_SKIP_ONCE}]]
    set so_new [expr {$so & ~(1 << $slot)}]
    if {$so_new != $so} {
        atrap_wr_verify $::OFF_ATRAP_SKIP_ONCE $so_new "ATRAP_SKIP_ONCE"
    }
    set h [rdx [expr {$::DBG_BASE + $::OFF_ATRAP_HIT}]]
    if {($h & 0x1) && ((($h >> 1) & 0x1) == $slot)} {
        dbg_wr $::OFF_ATRAP_HIT 1
        puts "> atrap$slot disarmed (owned hit latch cleared)"
    } else {
        puts "> atrap$slot disarmed"
    }
}

# `atrap status` body: dump both slots + the latched hit.  A0/D0 are only
# printed when the bitstream advertises atrap_regcap -- an explicit "not
# supported" line, never a fabricated 0.
proc atrap_status_report {} {
    set has_regcap [dbg_has_feature atrap_regcap]
    set has_d0qual [dbg_has_feature atrap_d0qual]
    foreach slot {0 1} {
        set ctrl_off [atrap_ctrl_off $slot]
        set c  [rdx [expr {$::DBG_BASE + $ctrl_off}]]
        set m  [rdx [expr {$::DBG_BASE + $ctrl_off + 4}]]
        set dv [rdx [expr {$::DBG_BASE + $ctrl_off + 8}]]
        foreach {value mask} [atrap_unpack_match $m] break
        set en [expr {$c & 0x1}]
        set dq [expr {($c >> 1) & 0x1}]
        if {$dq} {
            set d0str [format " d0qual=1 d0val=0x%08X" $dv]
        } elseif {$has_d0qual} {
            set d0str " d0qual=0"
        } else {
            set d0str " d0qual=n/a(bitstream lacks atrap_d0qual)"
        }
        puts [format "> atrap%d en=%d value=0x%04X mask=0x%04X match_set(value&mask)=0x%04X%s" \
                  $slot $en $value $mask [expr {$value & $mask}] $d0str]
    }
    set so [rdx [expr {$::DBG_BASE + $::OFF_ATRAP_SKIP_ONCE}]]
    puts [format "> atrap skip_once=0x%X (bit0=slot0 bit1=slot1)" [expr {$so & 0x3}]]
    set h [rdx [expr {$::DBG_BASE + $::OFF_ATRAP_HIT}]]
    if {$h & 0x1} {
        set hit_slot [expr {($h >> 1) & 0x1}]
        set busy     [expr {($h >> 2) & 0x1}]
        set opword   [expr {($h >> 16) & 0xFFFF}]
        set pc [rdx [expr {$::DBG_BASE + $::OFF_ATRAP_HIT_PC}]]
        puts [format "> atrap HIT: slot=%d opword=0x%04X pc=0x%08X capture_busy=%d" \
                  $hit_slot $opword $pc $busy]
        if {$has_regcap} {
            set a0 [rdx [expr {$::DBG_BASE + $::OFF_ATRAP_HIT_A0}]]
            set d0 [rdx [expr {$::DBG_BASE + $::OFF_ATRAP_HIT_D0}]]
            puts [format ">           a0=0x%08X d0=0x%08X" $a0 $d0]
        } else {
            puts ">           a0/d0 not supported by this bitstream (OFF_FEATURES bit 16, atrap_regcap, is absent)"
        }
    } else {
        set busy [expr {($h >> 2) & 0x1}]
        if {$busy && ![dbg_is_core040_epoch]} {
            puts "> atrap HIT: capture in progress (capture_busy set, hit_valid not yet latched) -- re-read shortly"
        } else {
            puts "> atrap HIT: none latched"
        }
    }
}

# `atrap list` body: no hardware access, purely informational.
proc atrap_print_list {} {
    puts "> atrap list: well-known Mac OS A-trap selectors"
    foreach {name val} {
        _Open          0xA000
        _Close         0xA001
        _Read          0xA002
        _Write         0xA003
        _Control       0xA004
        _Status        0xA005
        _DTInstall     0xA082
        _InitGraf      0xA86E
        _WaitNextEvent 0xA860
        _Dequeue       0xA96E
        _Enqueue       0xA96F
        _SCSIDispatch  0xA815
        _SysError      0xA9C9
    } {
        puts [format ">   %-14s value=0x%04X mask=0xFFFF" $name $val]
    }
    puts "> atrap list: family-mask idiom -- value=0xA800 mask=0xFF00 arms the whole 0xA8xx toolbox-trap family (any opword where opword&0xFF00==0xA800)"
}

# Read the captured retired-instruction count at the last halt.  Combine
# the LO/HI 32-bit register pair into a 64-bit Tcl wide-int.  Returns 0
# if the CPU has never halted (count is 0 at reset).
proc current_halt_inst {} {
    set lo 0x[dbg_rd $::OFF_HALT_HIT_INST_LO]
    set hi 0x[dbg_rd $::OFF_HALT_HIT_INST_HI]
    return [expr {wide($hi) << 32 | wide($lo)}]
}

# Advance N retired instructions from the current halted state.  Sets
# halt-after to (current + N), re-arms it, releases manual halt, and
# waits for the auto-halt to fire.
#
# Use this for iterative bring-up: halt → inspect → advance 1000 → halt
# → inspect → advance 1000 → ... without the cost of a full reset.
#
# WHY THE PRECONDITIONS BELOW EXIST — this command used to poison every
# probe that followed it.  `advance` is relative: it reads the retired-
# instruction count at the last halt and arms halt-after at (count + N).
# If the preceding halt had not actually LANDED, that read returns 0, so
# the target became 0+N — a count the CPU passed long ago.  halt-after
# then never fires again for the rest of the session, while a bisect
# ladder keeps printing confident "good" rows.  Five bogus probes went by
# in one ladder before anyone noticed, and every row after the first bad
# one was fiction.  So: an unlanded halt, or an inst-count of 0, is a
# FAILED PROBE, not a data point.  Refuse rather than compute a target
# from it.
#
# Returns 1 if the advance landed at or past the target, 0 otherwise.
proc advance {n {wait_ms 200}} {
    if {$n <= 0} { error "advance: N must be >= 1 (got $n)" }
    require_effective_halt "advance" \
        "The count `advance` measures from is only valid at a landed halt.\n>        Establish one first with `reset-and-halt-after <n>` or `break-pc <pc>`."

    set cur [current_halt_inst]
    if {$cur == 0} {
        error "advance: inst-count reads 0 — the halt has NOT landed, so there is no\n>        valid base to advance from.  Arming halt-after at 0+$n would target a\n>        count the CPU passed long ago, and NOTHING WOULD EVER HALT AGAIN while\n>        every later probe kept returning stale-but-plausible values.\n>        Re-establish a real halt (`reset-and-halt-after <n>`) and retry.\n>        [halt_status_line]"
    }

    set tgt [expr {$cur + $n}]
    set lo  [expr {$tgt & 0xFFFFFFFF}]
    set hi  [expr {($tgt >> 32) & 0xFFFFFFFF}]
    dbg_wr $::OFF_HALT_AFTER_LO $lo
    dbg_wr $::OFF_HALT_AFTER_HI $hi
    # Enable halt-after + clear latched halt, preserving PC/exception break enables.
    clear_auto_halt [expr {[halt_enable_bits] | $::HALT_AFTER_EN}]
    # Release manual halt.  CPU runs until halt-after fires at $tgt.
    clear_halt_req_preserve_control
    after $wait_ms

    # Verify the halt landed where we asked.  Without this the caller
    # cannot tell "advanced N" from "released the CPU and lost it".
    set landed [effective_halt]
    set now    [current_halt_inst]
    if {$landed && $now >= $tgt} {
        puts "> advance: landed at inst-count=$now (target $tgt, base $cur)"
        return 1
    }
    puts "> ============================================================"
    puts "> advance DID NOT LAND — DO NOT TREAT WHAT FOLLOWS AS DATA"
    puts ">   base inst-count : $cur"
    puts ">   target          : $tgt  (base + $n)"
    puts ">   inst-count now  : $now"
    puts ">   effective halt  : $landed"
    if {!$landed} {
        puts ">   The CPU IS RUNNING FREE right now.  Either the target was not"
        puts ">   reached within ${wait_ms}ms (retry with a larger wait_ms), or"
        puts ">   halt-after will never fire for this target."
    } else {
        puts ">   Halted, but BEFORE the requested target — something else"
        puts ">   (break-pc / exception / watchpoint) stopped the CPU first."
    }
    puts ">   Any live-arch / register / memory probe issued now is NOT a"
    puts ">   measurement of the state you asked for."
    puts "> ============================================================"
    return 0
}

# Atomic single-macro step from the current halted state.  DBG_CONTROL bit
# 1 arms the RTL step latch; debug_stop_manager completes it only at a
# last_uop macro boundary, then latches an auto-halt.
# ⚠️ OPEN BUG (observed on hardware 2026-09-06, p149): `step` sometimes
# FREE-RUNS instead of stepping — the core does not re-halt and the poll
# below times out (or returns a pc1 many instructions later), which reads
# as "the core resumed" and silently destroys a captured halt state.
# Suspect the trailing `dbg_wr $::OFF_CONTROL 0x0` releasing CTL_HALT_REQ,
# but the re-halt contract lives across DebugCtrlPlugin (debugStepRequest
# is a self-clearing reg, :451/:459/:1016) and RobPlugin (:943/:1262/:1282)
# and was NOT established — do not "fix" this by keeping HALT_REQ asserted
# without first proving the contract in sim or on the board.
# Until then: do not rely on `step` to preserve a halt you care about;
# re-arm a break-pc instead.
proc step {} {
    set st [scan [dbg_rd $::OFF_STATUS] %x]
    if {($st & 0x1) == 0} {
        error "step requires CPU halted"
    }

    set pc0 [scan [dbg_rd $::OFF_PC] %x]

    # If we're halted on a precise BP (break_pc_latched in HALT_CTL bit
    # 5), the next fetch at break_pc would re-inject SYS_DBG_BREAK and
    # halt again before the macro retires.  Arm skip_once for every
    # enabled BP slot so decode skips the injection on the next pass.
    # Auto-arm in HW exists but doesn't reliably stick in the current
    # bitstream (observed 2026-05-14).
    if {[bp_halt_latched]} {
        set en_slots [expr {[scan [dbg_rd $::OFF_BREAK_PC_CTRL] %x] & 0xF}]
        if {$en_slots != 0} {
            dbg_wr $::OFF_BP_SKIP_ONCE $en_slots
        }
    }

    # NOTE: these three raw OFF_CONTROL writes each fully overwrite the
    # register, same as the bug `clear_halt_req_preserve_control` (see its
    # doc comment) was added to fix elsewhere -- if `reset hold` is staged
    # while `step` runs, cold_reset_hold drops on the FIRST write here, not
    # just the last. Not fixed in this pass: the step-pulse edge timing
    # needs to land exactly right, and threading reset-hold preservation
    # through three sequential raw writes risks a new timing bug for a
    # combination (single-stepping mid-reset-hold) nobody has hit yet.
    # Flagged for a dedicated follow-up if it ever does.
    dbg_wr $::OFF_CONTROL $::CTL_HALT_REQ
    clear_auto_halt
    dbg_wr $::OFF_CONTROL [expr {$::CTL_HALT_REQ | $::CTL_STEP_PULSE}]
    dbg_wr $::OFF_CONTROL 0x0

    for {set i 0} {$i < 200} {incr i} {
        after 1
        set st [scan [dbg_rd $::OFF_STATUS] %x]
        if {($st & 0x1) != 0} {
            set pc1 [scan [dbg_rd $::OFF_PC] %x]
            return [list $pc0 $pc1]
        }
    }

    set pc1 [scan [dbg_rd $::OFF_PC] %x]
    error [format "step timeout: pc_live stayed 0x%08X (now 0x%08X)" \
        [expr {$pc0 & 0xffffffff}] [expr {$pc1 & 0xffffffff}]]
}

proc lsu_state_name {s} {
    set names {
        0 IDLE 1 LD_WAIT 2 ST_BUF 3 ST_WAIT 4 LD_GAP 5 LD_WAIT2
        6 ST_GAP 7 ST_WAIT2 8 MMU_WAIT 9 SPLIT_MMU_WAIT
    }
    if {[dict exists $names $s]} { return [dict get $names $s] }
    return "S$s"
}

proc dcache_state_name {s} {
    set names {
        0 IDLE 1 LOOKUP 2 EVICT_RD 3 EVICT_AW 4 EVICT_B 5 FILL_AR
        6 FILL_R 7 COMPLETE 8 BY_LD_R 9 BY_ST_AW 10 BY_ST_B 11 FA_SCAN
        12 FA_RD 13 FA_AW 14 FA_B 15 FA_DONE 16 ML_LOOKUP 17 ML_DISPATCH
        18 ML_EVICT_RD 19 ML_EVICT_AW 20 ML_EVICT_B 21 ML_DONE
        22 MA_CINV 23 MA_DONE 24 FA_INV 25 FA_INV_DONE 26 HIT_RESP
        27 EVICT_WAIT 28 FA_WAIT 29 ML_EVICT_WAIT
    }
    if {[dict exists $names $s]} { return [dict get $names $s] }
    return "S$s"
}

proc fault_snap_status_line {} {
    set valid [scan [dbg_rd $::OFF_FAULT_SNAP_VALID] %x]
    if {($valid & 0x1) == 0} {
        return "fault-snap: NOT LATCHED (no vec=2 fault since last clear/reset)"
    }
    # Decode the latched copy of dbg_wedge_state (same packing as
    # wedge_status_line).
    set w0 [scan [dbg_rd $::OFF_FAULT_SNAP_W0] %x]
    set w1 [scan [dbg_rd $::OFF_FAULT_SNAP_W1] %x]
    set addr [dbg_rd $::OFF_FAULT_SNAP_W2]
    set pc [dbg_rd $::OFF_FAULT_SNAP_W3]

    set rob_hd [expr {($w0 >> 31) & 1}]
    set last [expr {($w0 >> 30) & 1}]
    set uop_t [expr {($w0 >> 26) & 0xf}]
    set uop_op [expr {($w0 >> 18) & 0xff}]
    set lsu_state [expr {($w0 >> 14) & 0xf}]
    set dcache_state [expr {$w1 & 0x1f}]

    set flags {}
    foreach {bit name} {
        13 lsu_split 12 lsu_busy 11 mem_iss 10 dc_req 9 dc_wr
        8 dc_rvalid 7 dc_bvalid 6 dmmu_req 5 dmmu_ready
        4 dmmu_walk 3 dmmu_fault 2 arvalid 1 arready 0 rvalid
    } {
        lappend flags "$name=[expr {($w0 >> $bit) & 1}]"
    }

    return [format "fault-snap LATCHED: rob_pc=0x%s lsu_addr=0x%s rob_hd=%d last=%d uop_t=0x%X uop_op=0x%02X lsu=%d:%s dcache=%d:%s flags={%s} raw={w0=0x%08X w1=0x%08X}" \
        $pc $addr $rob_hd $last $uop_t $uop_op \
        $lsu_state [lsu_state_name $lsu_state] \
        $dcache_state [dcache_state_name $dcache_state] \
        [join $flags " "] $w0 $w1]
}

proc rts_snap_status_line {} {
    set valid [scan [dbg_rd $::OFF_RTS_SNAP_VALID] %x]
    if {($valid & 0x1) == 0} {
        return "rts-snap: NOT LATCHED (no exception/IRQ-entry finalize since last clear/reset)"
    }
    # Decode the latched copy of dbg_rts_snap_state (see m68k_core.v's
    # `assign dbg_rts_snap_state = {...}` for the authoritative packing).
    set ea         [dbg_rd $::OFF_RTS_SNAP_W0]
    set dc_rdata   [dbg_rd $::OFF_RTS_SNAP_W1]
    set br_target  [dbg_rd $::OFF_RTS_SNAP_W2]
    set fault_pc   [dbg_rd $::OFF_RTS_SNAP_W3]
    set w4         [scan [dbg_rd $::OFF_RTS_SNAP_W4] %x]

    # Field layout matches m68k_core.v's `assign dbg_rts_snap_state = {...}`
    # packing for W4: bits[7:0]=tag, [15:8]=vec, [16]=was_split,
    # [17]=alloc_epoch, [19:18]=dc_rresp, [20]=is_irq.
    set tag        [expr {$w4 & 0xff}]
    set vec        [expr {($w4 >> 8) & 0xff}]
    set was_split  [expr {($w4 >> 16) & 1}]
    set epoch      [expr {($w4 >> 17) & 1}]
    set dc_rresp   [expr {($w4 >> 18) & 0x3}]
    set is_irq     [expr {($w4 >> 20) & 1}]

    set match [expr {[string equal $dc_rdata $br_target] ? "MATCH" : "MISMATCH"}]

    return [format "rts-snap LATCHED: fault_pc(resume)=0x%s vec=0x%02X irq=%d tag=%d ea=0x%s dc_rdata=0x%s rob_br_target=0x%s (%s) dc_rresp=%d was_split=%d alloc_epoch=%d" \
        $fault_pc $vec $is_irq $tag $ea $dc_rdata $br_target $match $dc_rresp $was_split $epoch]
}

# OFF_WEDGE0..3 ARE NOT IMPLEMENTED ON cpu040 — REFUSE RATHER THAN DECODE ZEROS.
#
# These four offsets are legacy v1 debug_ctrl probes.  They are declared in the
# new core's register map (cpu040 DebugRegMap.scala:172-175) but its read mux
# NEVER DECODES THEM (`is(DebugRegMap.OFF_WEDGE0)` appears zero times in
# DebugCtrlPlugin.scala), so on a cpu040 bitstream they read 0x00000000.
#
# That is far worse than a missing feature, because the decoder below turns
# all-zero into a confident, fully-populated line — "rob_hd=0 last=0 lsu=0:IDLE
# dcache=0:IDLE" with every flag clear — which reads exactly like "the machine is
# completely drained".  On 2026-09-05 that fabricated line was used as evidence
# that the ROB was empty and no MMU walk or bus transfer was in flight on a wedged
# board, and it was used to RULE OUT a candidate root cause.  Nothing had been
# measured at all.
#
# Same failure class as OFF_CYCLE_LO/HI (declared, never implemented, reads 0 on a
# healthy running CPU).  Refuse loudly instead.
proc wedge_status_line {} {
    if {[dbg_is_core040_epoch]} {
        return "wedge-status REFUSED: OFF_WEDGE0..3 are v1 debug_ctrl probes and are\
NOT implemented by cpu040's debug read mux — they return 0x00000000, which this\
decoder would render as a plausible 'everything is IDLE' line. NOTHING would be\
measured. Use halt-status / exc-ring / the live retired-macro counter at\
0x50901008 instead."
    }
    set w0 [scan [dbg_rd $::OFF_WEDGE0] %x]
    set w1 [scan [dbg_rd $::OFF_WEDGE1] %x]
    set addr [dbg_rd $::OFF_WEDGE2]
    set pc [dbg_rd $::OFF_WEDGE3]

    set rob_hd [expr {($w0 >> 31) & 1}]
    set last [expr {($w0 >> 30) & 1}]
    set uop_t [expr {($w0 >> 26) & 0xf}]
    set uop_op [expr {($w0 >> 18) & 0xff}]
    set lsu_state [expr {($w0 >> 14) & 0xf}]
    set dcache_state [expr {$w1 & 0x1f}]

    set flags {}
    foreach {bit name} {
        13 lsu_split 12 lsu_busy 11 mem_iss 10 dc_req 9 dc_wr
        8 dc_rvalid 7 dc_bvalid 6 dmmu_req 5 dmmu_ready
        4 dmmu_walk 3 dmmu_fault 2 arvalid 1 arready 0 rvalid
    } {
        lappend flags "$name=[expr {($w0 >> $bit) & 1}]"
    }

    return [format "wedge: rob_pc=0x%s lsu_addr=0x%s rob_hd=%d last=%d uop_t=0x%X uop_op=0x%02X lsu=%d:%s dcache=%d:%s flags={%s} raw={w0=0x%08X w1=0x%08X}" \
        $pc $addr $rob_hd $last $uop_t $uop_op \
        $lsu_state [lsu_state_name $lsu_state] \
        $dcache_state [dcache_state_name $dcache_state] \
        [join $flags " "] $w0 $w1]
}

proc dcache-probe {set way {word 0}} {
    if {$set < 0 || $set > 31} { error "dcache-probe set must be 0..31" }
    if {$way < 0 || $way > 3} { error "dcache-probe way must be 0..3" }
    if {$word < 0 || $word > 7} { error "dcache-probe word must be 0..7" }
    set sel [expr {($set & 0x1f) | (($way & 0x3) << 5) | (($word & 0x7) << 7)}]
    dbg_wr $::OFF_DCACHE_PROBE_SEL $sel
    after 1
    set tag   [scan [dbg_rd $::OFF_DCACHE_PROBE_TAG] %x]
    set flags [scan [dbg_rd $::OFF_DCACHE_PROBE_FLAGS] %x]
    set data  [scan [dbg_rd $::OFF_DCACHE_PROBE_DATA] %x]
    set valid [expr {$flags & 0x1}]
    set dirty [expr {($flags >> 1) & 0x1}]
    puts "> dcache-probe set=$set way=$way word=$word tag=[format 0x%06X $tag] valid=$valid dirty=$dirty data=[format 0x%08X $data]"
}

# ── I-cache probe ────────────────────────────────────────────────────────
#
# Request/ack, unlike dcache-probe.  Writing SEL launches the probe and
# clears the done bit; the cache services it on the first cycle it is idle
# and not fetching, then sets done.  Reading TAG/DATA before done=1 returns
# the PREVIOUS probe's answer -- which is a completely plausible cache line,
# and therefore exactly the kind of silently-wrong result
# docs/hw_debug_traps.md exists to stop.  So this proc polls done and
# ERRORS on timeout rather than returning whatever it happens to find.
#
# Returns a dict: set, way, word, valid, tag, data.
proc icache_probe_raw {set way word} {
    if {![string is integer -strict $set] || $set < 0 || $set > 63} {
        error "icache-probe set must be 0..63 (the I-cache has 64 sets)"
    }
    if {![string is integer -strict $way] || $way < 0 || $way > 3} {
        error "icache-probe way must be 0..3"
    }
    if {![string is integer -strict $word] || $word < 0 || $word > 3} {
        error "icache-probe word must be 0..3 (16 B line = 4 longwords)"
    }
    set sel [expr {($set & 0x3f) | (($way & 0x3) << 6) | (($word & 0x3) << 8)}]
    dbg_wr $::OFF_ICACHE_PROBE_SEL $sel

    set flags 0
    set landed 0
    for {set i 0} {$i < 200} {incr i} {
        set flags [scan [dbg_rd $::OFF_ICACHE_PROBE_FLAGS] %x]
        if {($flags >> 1) & 0x1} { set landed 1 ; break }
        after 1
    }
    if {!$landed} {
        error "icache-probe set=$set way=$way word=$word DID NOT COMPLETE (flags=[format 0x%08X $flags]).\n>        The probe needs an idle I-cache cycle with no fetch outstanding.\n>        On a RUNNING CPU that can starve indefinitely -- halt first\n>        (`break-pc <pc>`, `advance <n>`, or `reset-and-halt-after <n>`).\n>        NOTHING was sampled: do NOT read this as \"the line is not cached\"."
    }
    return [dict create \
        set   $set \
        way   $way \
        word  $word \
        valid [expr {$flags & 0x1}] \
        tag   [scan [dbg_rd $::OFF_ICACHE_PROBE_TAG] %x] \
        data  [scan [dbg_rd $::OFF_ICACHE_PROBE_DATA] %x]]
}

proc icache-probe {set way {word 0}} {
    set r [icache_probe_raw $set $way $word]
    puts "> icache-probe set=$set way=$way word=$word tag=[format 0x%06X [dict get $r tag]] valid=[dict get $r valid] data=[format 0x%08X [dict get $r data]]"
}

# Address decomposition, straight off the icache.v geometry constants.
proc ic_set_of {addr} {
    return [expr {($addr >> 4) & ($::IC_NUM_SETS - 1)}]
}
proc ic_tag_of {addr} {
    return [expr {($addr >> 10) & 0x3FFFFF}]
}
proc ic_line_of {addr} {
    return [expr {$addr & ~($::IC_LINE_BYTES - 1)}]
}

# `icache-lookup <addr>` -- "what would the I-cache supply for this address,
# and does it match RAM?"
#
# Prints the cached line as `> mem <addr> = 0x<word>` lines, in exactly the
# format and order `dump-mem` uses, so the two outputs diff directly.  Then
# reads the same 16 bytes over JTAG-AXI and prints a per-word verdict,
# because doing that comparison by eye at 3am is how you convince yourself
# of the wrong answer.
proc icache-lookup {addr} {
    set line [ic_line_of $addr]
    set idx  [ic_set_of  $line]
    set tag  [ic_tag_of  $line]

    puts "> icache-lookup addr=[format 0x%08X $addr] line=[format 0x%08X $line] set=$idx tag=[format 0x%06X $tag]"

    set hit -1
    set ways {}
    for {set w 0} {$w < $::IC_NUM_WAYS} {incr w} {
        set r [icache_probe_raw $idx $w 0]
        set v [dict get $r valid]
        set t [dict get $r tag]
        if {$v} { set mark "V" } else { set mark "-" }
        lappend ways "way$w:$mark tag=[format 0x%06X $t]"
        if {$v && $t == $tag} {
            if {$hit >= 0} {
                # Two ways claiming one tag is a cache bug in its own right;
                # say so rather than silently picking one.
                puts "> icache-lookup WARNING: ways $hit and $w both hold tag [format 0x%06X $tag] -- duplicate line, report this"
            }
            set hit $w
        }
    }
    puts "> icache-lookup ways: [join $ways {  }]"

    if {$hit < 0} {
        puts "> icache-lookup MISS -- no way of set $idx holds tag [format 0x%06X $tag]."
        puts ">        The next fetch of this address refills from memory, so a"
        puts ">        stale-I-cache explanation is RULED OUT for this address"
        puts ">        (as of this instant -- a running CPU can refill it again)."
        return
    }

    puts "> icache-lookup HIT way=$hit"
    set cached {}
    for {set k 0} {$k < 4} {incr k} {
        set r [icache_probe_raw $idx $hit $k]
        lappend cached [dict get $r data]
        puts "> mem [format 0x%08X [expr {$line + $k*4}]] = 0x[format %08X [dict get $r data]]"
    }

    # The same 16 bytes straight from DDR -- the actual comparison.
    set memw [rd_burst $line 4]
    set ndiff 0
    for {set k 0} {$k < 4} {incr k} {
        set c [lindex $cached $k]
        set m [scan [lindex $memw $k] %x]
        if {$c == $m} { set verdict "SAME" } else { set verdict "DIFFER" ; incr ndiff }
        puts "> icache-lookup [format 0x%08X [expr {$line + $k*4}]] cache=0x[format %08X $c] mem=0x[format %08X $m] $verdict"
    }
    if {$ndiff == 0} {
        puts "> icache-lookup line MATCHES memory (0/4 words differ)"
    } else {
        puts "> icache-lookup line is STALE vs memory ($ndiff/4 words differ)."
        puts ">        The CPU is fetching bytes DDR no longer holds.  Instruction"
        puts ">        boundaries in the cached stream can differ from the ones you"
        puts ">        computed off a memory dump -- which is exactly why a break-pc"
        puts ">        on an address derived from memory would never fire."
        puts ">        Before concluding: `dcache-op push` at a halt and re-run."
        puts ">        JTAG reads go to DDR, and the fresh bytes may still be sitting"
        puts ">        dirty in the write-back D-cache rather than genuinely written."
    }
}

# Kick a cache-maintenance op and poll its status register to completion.
# Shared by dcache-op/icache-op so the done=0 handling cannot drift apart.
#
# The `done` bit is the ONLY evidence the walk happened.  A run that ends
# with done=0 did NOT maintain the cache, and anything you read afterwards
# is exactly as stale as it was before — so this raises rather than
# printing a status line that reads like a result.
proc cache_op_poll {off label} {
    set status 0
    for {set i 0} {$i < 10000} {incr i} {
        set status [scan [dbg_rd $off] %x]
        if {$status & 0x32} { break }
        after 1
    }
    set busy [expr {$status & 0x1}]
    set done [expr {($status >> 1) & 0x1}]
    set rejected [expr {($status >> 4) & 0x1}]
    set op_error [expr {($status >> 5) & 0x1}]
    if {$rejected} {
        error "$label REJECTED (status=[format 0x%08X $status]). [halt_status_line]"
    }
    if {$op_error} {
        error "$label completed with a CACHE WRITEBACK ERROR (status=[format 0x%08X $status]); memory coherence is not proven"
    }
    if {!$done} {
        set extra ""
        if {$busy} {
            # busy=1 that never reaches done=1 is the STUCK-BUSY wedge:
            # debug_ctrl accepts the launch on dbg_halt_req (a request) while
            # dcache.v requires dbg_core_halt (the acknowledged halt, which
            # lags by a skid stage).  A pulse landing in that window is
            # dropped, and dcache_op_busy_r has no timeout and no clear path
            # — so the facility is dead for the rest of the session and every
            # later cache op is rejected before it starts.
            set extra "\n>        busy=1 with no completion = the STUCK-BUSY wedge: the launch was\n>        accepted by debug_ctrl but dropped by the cache FSM (halt-request vs\n>        acknowledged-halt skew).  dcache_op_busy_r has no timeout, so cache\n>        ops stay dead until the next CPU reset.  `reset` to recover."
        }
        error "$label DID NOT COMPLETE (done=0 busy=$busy status=[format 0x%08X $status]).\n>        The cache was NOT maintained.  Any memory you read now is as stale\n>        as it was before this command — do not treat it as a measurement.$extra\n>        [halt_status_line]"
    }
    puts "> $label busy=$busy done=$done status=[format 0x%08X $status]"
}

# Cache maintenance requires an EFFECTIVE halt. New m68k040 hardware also
# reports REJECTED and writeback ERROR explicitly; legacy hardware lacks those
# terminal bits, so coherent-* below refuses it instead of overclaiming truth.
set ::CACHE_OP_HALT_HINT "Halt first (`break-pc <pc>`, `advance <n>`, or `reset-and-halt-after <n>`),\n>        then retry.  To inspect cache contents WITHOUT halting, use\n>        `dcache-probe <set> <way> \[word\]`, which reads the cache array directly."

proc dcache-op {kind} {
    set k [string tolower $kind]
    if {$k eq "inv" || $k eq "invalidate"} {
        set op 0
    } elseif {$k eq "push" || $k eq "cpush"} {
        set op 1
    } else {
        error "dcache-op kind must be inv or push"
    }
    if {[dbg_is_core040_epoch]} {
        if {![dbg_has_feature cache_maint_only]} {
            error "dcache-op: this m68k040 build does not advertise cache_maint_only"
        }
    } elseif {![dbg_has_feature dcache_probe]} {
        error "dcache-op: legacy hardware does not advertise dcache_probe/cache operations"
    }
    require_effective_halt "dcache-op $k" $::CACHE_OP_HALT_HINT
    dbg_wr $::OFF_DCACHE_OP [expr {0x1 | ($op << 1)}]
    cache_op_poll $::OFF_DCACHE_OP "dcache-op $k"
}

proc icache-op {{kind "inv"}} {
    set k [string tolower $kind]
    if {$k ne "inv" && $k ne "invalidate"} {
        error "icache-op kind must be inv"
    }
    if {[dbg_is_core040_epoch]} {
        if {![dbg_has_feature cache_maint_only]} {
            error "icache-op: this m68k040 build does not advertise cache_maint_only"
        }
    } elseif {![dbg_has_feature icache_probe]} {
        error "icache-op: legacy hardware does not advertise icache_probe"
    }
    require_effective_halt "icache-op inv" $::CACHE_OP_HALT_HINT
    dbg_wr $::OFF_ICACHE_OP 0x1
    cache_op_poll $::OFF_ICACHE_OP "icache-op inv"
}

# Coherent physical-memory helpers for halted bring-up. JTAG-AXI remains the
# transport; the CPU-side cache walker establishes DDR as the shared truth.
proc coherent-r {addr} {
    require_core040_debug_epoch "coherent-r"
    require_effective_halt "coherent-r"
    dcache-op push
    return [rd $addr]
}

proc coherent-dump {addr words} {
    require_core040_debug_epoch "coherent-dump"
    require_effective_halt "coherent-dump"
    dcache-op push
    return [rd_burst $addr $words]
}

proc coherent-w {addr data} {
    require_core040_debug_epoch "coherent-w"
    require_effective_halt "coherent-w"
    dcache-op push
    wr $addr $data
    dcache-op inv
    icache-op inv
}

# ── ADB event-injection helpers (rtl/mac/adb_inject.v) ──────────────────
proc adbinj_wr {off byte} {
    wr [expr {$::ADBINJ_BASE + $off}] [expr {$byte & 0xFF}]
}
proc adbinj_rd {off} {
    set v [rd [expr {$::ADBINJ_BASE + $off}]]
    if {$v eq "BADA0BAD"} { error "adb: JTAG-AXI read failed at ADBINJ+[format 0x%02X $off]" }
    scan $v %x n
    return [expr {$n & 0xFF}]
}
proc adb_status_line {} {
    set k [adbinj_rd $::ADBINJ_OFF_KBD]
    set m [adbinj_rd $::ADBINJ_OFF_BTN]
    return [format "kbd fifo count=%d full=%d nonempty=%d | mouse event_pending=%d" \
                [expr {($k >> 1) & 0x3F}] [expr {($k >> 7) & 1}] \
                [expr {$k & 1}] [expr {$m & 1}]]
}

# ── REPL loop ────────────────────────────────────────────────────────────
if {$::jtag_repl_library_only} { return }
puts "> READY"
flush stdout

while {1} {
    if {[gets stdin line] < 0} { break }
    set line [string trim $line]
    if {$line eq ""} { puts "> READY"; flush stdout; continue }

    if {[catch {
        set tokens [split $line]
        set cmd [lindex $tokens 0]
        switch -- $cmd {
            r {
                # HEX-by-default operand parsing (see parse_num).  The
                # resolved address is echoed so a mis-typed operand is
                # visible immediately instead of yielding a plausible
                # value from somewhere else entirely.
                set addr [require_aligned [parse_num [lindex $tokens 1] "address"]]
                set v [rd $addr]
                puts "> r [format 0x%08X $addr] = 0x$v"
            }
            w {
                set addr [require_aligned [parse_num [lindex $tokens 1] "address"]]
                set data [parse_num [lindex $tokens 2] "data"]
                wr $addr $data
                puts "> w [format 0x%08X $addr] = [format 0x%08X $data]"
            }
            dbg-caps {
                dbg_caps_report
            }
            bp-stats {
                # 0x0101C / 0x01020 — see debug_ctrl.v OFF_BP_*.
                set br  [expr {0x[dbg_rd 0x0101C]}]
                set ms  [expr {0x[dbg_rd 0x01020]}]
                if {$br == 0} {
                    puts "> bp-stats: 0 retired branches -- either nothing has run yet, or this bitstream predates the counters (they read 0 in both cases; check dbg-caps/build_id)."
                } else {
                    set hit [expr {$br - $ms}]
                    puts [format "> bp-stats: branches=%d  mispred=%d  hits=%d  hit-rate=%.2f%%" \
                              $br $ms $hit [expr {100.0 * $hit / $br}]]
                }
                puts [format "> bp-stats: (RAS returns mispred, separate counter) = %d" \
                          [expr {0x[dbg_rd 0x01010]}]]
            }
            ram-window {
                # ram-window             — print the live RAM window size
                # ram-window <lg2|size>  — set it; accepts 22..30, or a plain
                #                          size like 4M / 64M / 1G
                #
                # Unlike mon-sense this needs NO feature gate: the CSR clamps
                # to [22..30], so 0 (what an unmapped offset reads back) is
                # NOT a legal value and a write-verify cannot be confused for
                # success on a bitstream that lacks the register.
                #
                # The value survives a CPU reset but NOT a full FPGA
                # reprogram, which restores the 26 (64 MiB) default.  Mac OS
                # sizes RAM once during early boot, so `reset` afterwards.
                set arg [lindex $tokens 1]
                if {$arg eq ""} {
                    set got [expr {[scan [dbg_rd $::OFF_RAM_WINDOW_LG2] %x] & 0x3F}]
                    puts [format "> ram-window = %d (%s)" $got [ram_window_describe $got]]
                } else {
                    # accept 4M / 64M / 1G as well as a bare lg2
                    set a [string toupper $arg]
                    if {[regexp {^([0-9]+)([MG])$} $a -> n unit]} {
                        set bytes [expr {$n * ($unit eq "G" ? 1024*1024*1024 : 1024*1024)}]
                        set want 0
                        for {set i 22} {$i <= 30} {incr i} {
                            if {(1 << $i) == $bytes} { set want $i ; break }
                        }
                        if {$want == 0} {
                            error "ram-window: $arg is not a power-of-two size in \[4M..1G\]"
                        }
                    } elseif {[string is integer -strict $arg]} {
                        # A bare lg2 is a SHIFT COUNT and is written in decimal --
                        # the read-back prints decimal, and the help says 22..30.
                        # parse_num defaults a bare token to HEX, so it turned 26
                        # into 0x26 = 38 and rejected every legal value with
                        # "lg2 must be in [22..30]".  `string is integer` still
                        # accepts an explicit 0x prefix, so both forms work.
                        set want [expr {$arg & 0x3F}]
                    } else {
                        set want [expr {[parse_num $arg "ram window lg2"] & 0x3F}]
                    }
                    if {$want < 22 || $want > 30} {
                        error "ram-window: lg2 must be in \[22..30\] = \[4 MiB..1 GiB\] (the CSR clamps, so an out-of-range write would silently land elsewhere)"
                    }
                    dbg_wr $::OFF_RAM_WINDOW_LG2 $want
                    set got [expr {[scan [dbg_rd $::OFF_RAM_WINDOW_LG2] %x] & 0x3F}]
                    if {$got != $want} {
                        error [format "ram-window: wrote %d but read back %d -- this bitstream has no RAM-window CSR, or the value was clamped" $want $got]
                    }
                    puts [format "> ram-window = %d (%s)" $got [ram_window_describe $got]]
                    puts "> ram-window: survives a CPU reset, NOT an FPGA reprogram (default 26)."
                    puts "> ram-window: Mac OS sizes RAM at early boot -- run `reset` to apply."
                }
            }
            mon-sense {
                # mon-sense              — print the live code
                # mon-sense <hex>        — set it (7 bits)
                #
                # Feature-gated on purpose: on a bitstream without the CSR
                # the unmapped offset reads 0 with an OKAY response, and 0
                # is itself a LEGAL sense code (Mac 21" Color Display).  A
                # bare read would therefore report a plausible wrong answer
                # rather than "not supported".
                if {![dbg_has_feature mon_sense]} {
                    error "mon-sense: this bitstream does not advertise the mon_sense feature (OFF_FEATURES bit 18); its DAFB monitor sense is still the compile-time video.v MONITOR_TYPE parameter"
                }
                set arg [lindex $tokens 1]
                if {$arg eq ""} {
                    set got [expr {[scan [dbg_rd $::OFF_MON_SENSE] %x] & 0x7F}]
                    puts "> mon-sense = [mon_sense_describe $got]"
                } else {
                    set want [expr {[parse_num $arg "monitor sense code"] & 0x7F}]
                    dbg_wr $::OFF_MON_SENSE $want
                    set got [expr {[scan [dbg_rd $::OFF_MON_SENSE] %x] & 0x7F}]
                    puts "> mon-sense = [mon_sense_describe $got]"
                    if {$got != $want} {
                        error [format "mon-sense: wrote 0x%02X but read back 0x%02X -- the CSR did not take" $want $got]
                    }
                    # Mac OS reads the DAFB sense pins once, during DAFB
                    # init, and caches the resulting display identity in the
                    # video driver / gDevice.  Changing the pins on a booted
                    # system therefore does NOT re-open the Monitors list:
                    # the CPU has to go back through DAFB init.
                    puts "> mon-sense: value survives a CPU reset (debug reset domain)."
                    puts "> mon-sense: Mac OS samples sense ONLY at DAFB init, so run"
                    puts ">            `reset` for the new code to take effect."
                }
            }
            watch {
                # watch <slot> <addr> [r|w|rw] [mask <amask>] [value <v> [lanes <m>]]
                # watch <slot> off
                # watch status
                if {![dbg_has_feature watchpoints]} {
                    error "watch: this bitstream does not advertise the watchpoints feature (OFF_FEATURES bit 13)"
                }
                set sub [lindex $tokens 1]
                if {$sub eq "status" || $sub eq ""} {
                    foreach slot {0 1} {
                        set base [expr {$slot ? $::OFF_WP1_ADDR : $::OFF_WP0_ADDR}]
                        set a [dbg_rd $base]
                        set m [dbg_rd [expr {$base + 4}]]
                        set v [dbg_rd [expr {$base + 8}]]
                        set c [scan [dbg_rd [expr {$base + 12}]] %x]
                        set kind ""
                        if {$c & 0x2} { append kind r }
                        if {$c & 0x4} { append kind w }
                        puts [format "> wp%d en=%d kind=%-2s addr=0x%s amask=0x%s value=0x%s val_en=%d lanes=0x%X" \
                                  $slot [expr {$c & 1}] $kind $a $m $v \
                                  [expr {($c >> 3) & 1}] [expr {($c >> 8) & 0xF}]]
                    }
                    set h [scan [dbg_rd $::OFF_WP_HIT] %x]
                    if {$h & 1} {
                        puts [format "> wp HIT: slot=%d %s addr=0x%s data=0x%s pc=0x%s wstrb=0x%X" \
                                  [expr {($h >> 1) & 1}] \
                                  [expr {($h >> 2) & 1 ? "STORE" : "LOAD"}] \
                                  [dbg_rd $::OFF_WP_HIT_ADDR] \
                                  [dbg_rd $::OFF_WP_HIT_DATA] \
                                  [dbg_rd $::OFF_WP_HIT_PC] \
                                  [expr {($h >> 12) & 0xF}]]
                    } else {
                        puts "> wp HIT: none latched"
                    }
                } else {
                    set slot [parse_num $sub "watchpoint slot"]
                    if {$slot != 0 && $slot != 1} { error "watch: slot must be 0 or 1" }
                    set base [expr {$slot ? $::OFF_WP1_ADDR : $::OFF_WP0_ADDR}]
                    if {[lindex $tokens 2] eq "off"} {
                        dbg_wr [expr {$base + 12}] 0
                        dbg_wr $::OFF_WP_HIT 1
                        puts "> wp$slot disarmed (hit latch cleared)"
                    } else {
                        set addr [parse_num [lindex $tokens 2] "watch address"]
                        set kind rw
                        set amask 0
                        set value ""
                        set lanes 0xF
                        set i 3
                        while {$i < [llength $tokens]} {
                            set t [lindex $tokens $i]
                            switch -- $t {
                                r - w - rw { set kind $t; incr i }
                                mask  { set amask [parse_num [lindex $tokens [expr {$i+1}]] "amask"]; incr i 2 }
                                value { set value [parse_num [lindex $tokens [expr {$i+1}]] "value"]; incr i 2 }
                                lanes { set lanes [parse_num [lindex $tokens [expr {$i+1}]] "lanes"]; incr i 2 }
                                default { error "watch: unknown option '$t'" }
                            }
                        }
                        set ctrl [wp_ctrl_encode $kind $value $lanes]
                        if {$value ne ""} {
                            set warn [wp_lane_warning $addr $value $lanes]
                            if {$warn ne ""} { puts "> WARNING: $warn" }
                            dbg_wr [expr {$base + 8}] $value
                        }
                        dbg_wr $base $addr
                        dbg_wr [expr {$base + 4}] $amask
                        dbg_wr [expr {$base + 12}] $ctrl
                        dbg_wr $::OFF_WP_HIT 1
                        puts [format "> wp%d armed: %s addr=0x%08X amask=0x%08X%s (halts at next retire boundary after the access; config survives CPU reset)" \
                                  $slot $kind $addr $amask \
                                  [expr {$value ne "" ? [format " value=0x%08X lanes=0x%X" $value $lanes] : ""}]]
                    }
                }
            }
            atrap {
                # atrap <slot> <value> [mask <m>] [d0 <v>]
                # atrap <slot> off
                # atrap status
                # atrap list
                set sub [lindex $tokens 1]
                if {$sub eq "list"} {
                    atrap_print_list
                } elseif {$sub eq "status" || $sub eq ""} {
                    if {![dbg_has_feature atrap_bp]} {
                        error "atrap: this bitstream does not advertise the atrap_bp feature (OFF_FEATURES bit 15)"
                    }
                    atrap_status_report
                } else {
                    if {![dbg_has_feature atrap_bp]} {
                        error "atrap: this bitstream does not advertise the atrap_bp feature (OFF_FEATURES bit 15)"
                    }
                    set slot [parse_num $sub "atrap slot"]
                    atrap_ctrl_off $slot  ;# raises loudly if slot isn't 0/1
                    if {[lindex $tokens 2] eq "off"} {
                        atrap_disarm $slot
                    } else {
                        set value [atrap_check_value [parse_num [lindex $tokens 2] "atrap value"]]
                        set mask 0xFFFF
                        set d0qual 0
                        set d0val 0
                        set i 3
                        while {$i < [llength $tokens]} {
                            set t [lindex $tokens $i]
                            switch -- $t {
                                mask {
                                    set mask [atrap_check_mask [parse_num [lindex $tokens [expr {$i+1}]] "atrap mask"]]
                                    incr i 2
                                }
                                d0 {
                                    if {![dbg_has_feature atrap_d0qual]} {
                                        error "atrap: this bitstream does not advertise the atrap_d0qual feature (OFF_FEATURES bit 17) -- d0 qualifier unavailable"
                                    }
                                    set d0val [parse_num [lindex $tokens [expr {$i+1}]] "atrap d0 value"]
                                    set d0qual 1
                                    incr i 2
                                }
                                default { error "atrap: unknown option '$t'" }
                            }
                        }
                        atrap_arm $slot $value $mask $d0qual $d0val
                    }
                }
            }
            dbg-cfg-wipe {
                # Restore host-programmed debug CONFIGURATION (break-PCs,
                # halt-exc mask, halt-after, RAM window, arch shadow) to
                # power-on defaults.  Needed because, since the debug reset
                # domain landed, a CPU reset no longer wipes them -- which
                # is the whole point, but it means the host needs an
                # explicit way back to a known state.
                if {![dbg_has_feature cfg_wipe]} {
                    error "dbg-cfg-wipe: this bitstream does not advertise the cfg_wipe feature (OFF_FEATURES bit 2). Reload a bitstream built from debug_ctrl >= 0xDEB6_0006."
                }
                dbg_wr $::OFF_DBG_RESET_CTL $::DRC_CFG_WIPE
                puts "> dbg-cfg-wipe: debug configuration restored to power-on defaults"
            }
            sd-write {
                if {[llength $tokens] != 3} { error "usage: sd-write <lba> <file>" }
                set lba [expr {[lindex $tokens 1]}]
                set path [lindex $tokens 2]
                set sectors [sd_write_file $lba $path]
                puts "> sd-write done: $sectors sectors from $path starting at [format 0x%08X $lba]"
            }
            sd-write-fast {
                if {[llength $tokens] < 3 || [llength $tokens] > 4} {
                    error "usage: sd-write-fast <lba> <file> [noverify]"
                }
                set lba [expr {[lindex $tokens 1]}]
                set path [lindex $tokens 2]
                set do_verify 1
                if {[llength $tokens] == 4} {
                    if {[lindex $tokens 3] eq "noverify"} {
                        set do_verify 0
                    } else {
                        error "usage: sd-write-fast <lba> <file> [noverify]"
                    }
                }
                set t0 [clock milliseconds]
                set sectors [sd_write_fast_file $lba $path $do_verify]
                set dt [expr {([clock milliseconds] - $t0) / 1000.0}]
                puts [format "> sd-write-fast done: %d sectors from %s at lba=%d in %.1f s (verify=%s)" \
                          $sectors $path $lba $dt [expr {$do_verify ? "on" : "off"}]]
            }
            sd-verify {
                if {[llength $tokens] < 3 || [llength $tokens] > 4} {
                    error "usage: sd-verify <lba> <file> \[max_sectors\]"
                }
                set lba [expr {[lindex $tokens 1]}]
                set path [lindex $tokens 2]
                set maxs 0
                if {[llength $tokens] == 4} { set maxs [expr {[lindex $tokens 3]}] }
                set t0 [clock milliseconds]
                lassign [sd_verify_file $lba $path $maxs] nsec nbad
                set dt [expr {([clock milliseconds] - $t0) / 1000.0}]
                puts [format "> sd-verify done: %d sectors from lba=%d in %.1f s — %d MISMATCHING batches" \
                          $nsec $lba $dt $nbad]
            }
            sd-fast-status {
                puts "> [sd_fast_status]"
            }
            eth-status {
                if {[llength $tokens] != 1} { error "usage: eth-status" }
                eth_status_report
            }
            eth-clear {
                if {[llength $tokens] != 1} { error "usage: eth-clear" }
                eth_debug_clear
            }
            eth-promisc {
                if {[llength $tokens] == 1} {
                    eth_promisc_status
                } elseif {[llength $tokens] == 2} {
                    set mode [string tolower [lindex $tokens 1]]
                    if {$mode ni {0 1 on off}} { error "usage: eth-promisc [on|off|1|0]" }
                    eth_promisc_set [expr {$mode in {1 on}}]
                } else {
                    error "usage: eth-promisc [on|off|1|0]"
                }
            }
            vhdd-status {
                if {[llength $tokens] != 1} { error "usage: vhdd-status" }
                vhdd_status_report
            }
            sonic-trace {
                set sub "dump"
                if {[llength $tokens] >= 2} { set sub [lindex $tokens 1] }
                switch -- $sub {
                    status {
                        sonic_trace_probe
                        lassign [sonic_trace_status] wp wrapped frozen
                        puts [format "> sonic-trace: wr_ptr=%d wrapped=%d frozen=%d filtered=%d" \
                                  $wp $wrapped $frozen [sonic_trace_filtered]]
                    }
                    rearm {
                        sonic_trace_probe
                        eth_debug_wr $::ETH_OFF_TRACE_CTRL 0x2
                        after 50
                        lassign [sonic_trace_status] wp wrapped frozen
                        puts [format "> sonic-trace: re-armed (wr_ptr=%d wrapped=%d frozen=%d filtered=%d)" \
                                  $wp $wrapped $frozen [sonic_trace_filtered]]
                    }
                    freeze {
                        sonic_trace_probe
                        eth_debug_wr $::ETH_OFF_TRACE_CTRL 0x1
                        after 50
                        lassign [sonic_trace_status] wp wrapped frozen
                        puts [format "> sonic-trace: frozen=%d wr_ptr=%d filtered=%d" \
                                  $frozen $wp [sonic_trace_filtered]]
                    }
                    dump {
                        set path "/tmp/sonic_trace.csv"
                        if {[llength $tokens] >= 3} { set path [lindex $tokens 2] }
                        sonic_trace_dump $path
                    }
                    default {
                        error "usage: sonic-trace \[status|freeze|rearm|dump \[<path>\]\]"
                    }
                }
            }
            scsi-trace {
                set sub "dump"
                if {[llength $tokens] >= 2} { set sub [lindex $tokens 1] }
                switch -- $sub {
                    status {
                        lassign [scsi_trace_status] wp wrapped frozen
                        puts [format "> scsi-trace: wr_ptr=%d wrapped=%d frozen=%d" \
                                  $wp $wrapped $frozen]
                    }
                    rearm {
                        vhdd_wrreg $::VHDD_OFF_TRACE_CTRL 0x2
                        after 50
                        lassign [scsi_trace_status] wp wrapped frozen
                        puts [format "> scsi-trace: re-armed (wr_ptr=%d wrapped=%d frozen=%d)" \
                                  $wp $wrapped $frozen]
                    }
                    freeze {
                        vhdd_wrreg $::VHDD_OFF_TRACE_CTRL 0x1
                        after 50
                        lassign [scsi_trace_status] wp wrapped frozen
                        puts [format "> scsi-trace: frozen=%d wr_ptr=%d" $frozen $wp]
                    }
                    dump {
                        set path "/tmp/scsi_trace.csv"
                        if {[llength $tokens] >= 3} { set path [lindex $tokens 2] }
                        scsi_trace_dump $path
                    }
                    default {
                        error "usage: scsi-trace \[status|freeze|rearm|dump \[<path>\]\]"
                    }
                }
            }
            vhdd-net {
                if {[llength $tokens] == 1} {
                    vhdd_net_show
                } elseif {[llength $tokens] == 7} {
                    vhdd_net_set [lindex $tokens 1] [lindex $tokens 2] \
                                 [lindex $tokens 3] [lindex $tokens 4] \
                                 [lindex $tokens 5] [lindex $tokens 6]
                } else {
                    error "usage: vhdd-net \[<our-mac> <our-ip> <our-port> <dst-mac> <dst-ip> <dst-port>\]"
                }
            }
            vhdd-wprot {
                if {[llength $tokens] > 2} { error "usage: vhdd-wprot \[on|off\]" }
                vhdd_wprot [expr {[llength $tokens] == 2 ? [lindex $tokens 1] : ""}]
            }
            vhdd-enable {
                if {[llength $tokens] != 3} { error "usage: vhdd-enable <sd|ram|both> <0|1>" }
                vhdd_set_enable [lindex $tokens 1] [lindex $tokens 2]
            }
            ramdisk-size {
                if {[llength $tokens] != 2} { error "usage: ramdisk-size <MB>   (decimal MB; prefix 0x for hex)" }
                set mb [parse_count [lindex $tokens 1] "size in MB"]
                vhdd_set_size_mb $mb
            }
            ramdisk-clear {
                if {[llength $tokens] > 2} { error "usage: ramdisk-clear \[<MB>\]" }
                set mb 0
                if {[llength $tokens] == 2} { set mb [parse_count [lindex $tokens 1] "size in MB"] }
                vhdd_ramdisk_clear $mb
            }
            ramdisk-load {
                if {[llength $tokens] < 2 || [llength $tokens] > 3} {
                    error "usage: ramdisk-load <file> \[<byte-offset>\]"
                }
                set path [lindex $tokens 1]
                set boff 0
                if {[llength $tokens] == 3} { set boff [parse_count [lindex $tokens 2] "byte offset"] }
                vhdd_ramdisk_load $path $boff
            }
            ramdisk-save {
                if {[llength $tokens] < 3 || [llength $tokens] > 4} {
                    error "usage: ramdisk-save <file> <byte-count> \[<byte-offset>\]"
                }
                set path  [lindex $tokens 1]
                set count [parse_count [lindex $tokens 2] "byte count"]
                set boff  0
                if {[llength $tokens] == 4} { set boff [parse_count [lindex $tokens 3] "byte offset"] }
                vhdd_ramdisk_save $path $count $boff
            }
            pc {
                set v [dbg_rd $::OFF_PC]
                puts "> pc = 0x$v"
            }
            halt-status {
                puts "> [halt_status_line]"
            }
            perf {
                # perf                  freeze, read, resume  (the normal case)
                # perf hold             freeze, read, STAY frozen
                # perf live             read WITHOUT freezing (pairs may tear)
                # perf clear            zero + start a fresh window
                # perf clear hold       zero + stay frozen (arm before a reset/BP release)
                # perf freeze / resume  stop / restart the window explicitly
                set sub ""
                if {[llength $tokens] > 1} { set sub [lindex $tokens 1] }
                switch -- $sub {
                    "" {
                        perf_report 1 1
                    }
                    hold {
                        perf_report 1 0
                    }
                    live {
                        perf_report 0 0
                    }
                    clear {
                        set hold 0
                        if {[llength $tokens] > 2 && [lindex $tokens 2] eq "hold"} { set hold 1 }
                        perf_clear $hold
                        if {$hold} {
                            puts "> perf: counters zeroed and left FROZEN -- 'perf resume' starts the window"
                        } else {
                            puts "> perf: counters zeroed, window RUNNING"
                        }
                    }
                    freeze {
                        dbg_wr $::OFF_PERF_CTL $::PERF_CTL_FREEZE
                        puts "> perf: counters FROZEN (reads are now atomic across the 64-bit pairs)"
                    }
                    resume {
                        dbg_wr $::OFF_PERF_CTL $::PERF_CTL_RUN
                        puts "> perf: counters RUNNING"
                    }
                    default {
                        error "usage: perf | perf hold | perf live | perf clear \[hold\] | perf freeze | perf resume"
                    }
                }
            }
            perf-clear {
                set hold 0
                if {[llength $tokens] > 1 && [lindex $tokens 1] eq "hold"} { set hold 1 }
                perf_clear $hold
                if {$hold} {
                    puts "> perf-clear: counters zeroed and left FROZEN -- 'perf resume' starts the window"
                } else {
                    puts "> perf-clear: counters zeroed, window RUNNING"
                }
            }
            halt {
                set wait_ms 200
                if {[llength $tokens] > 2} { error "usage: halt \[wait_ms\]" }
                if {[llength $tokens] == 2} {
                    set wait_ms [parse_count [lindex $tokens 1] "halt wait_ms"]
                }
                set elapsed [request_debug_halt $wait_ms]
                puts "> halt landed at a coherent macro boundary after ${elapsed}ms"
                puts "> [halt_status_line]"
            }
            halt-clear {
                clear_auto_halt
                set ::last_requested_break_pc ""
                puts "> halt-clear done (auto-halt latches cleared; enables preserved)"
                puts "> [halt_status_line]"
            }
            halt-release {
                release_debug_halt
                puts "> halt-release done (latches cleared; DBG_CONTROL=0)"
                puts "> [halt_status_line]"
            }
            break-pc {
                if {[llength $tokens] < 2} { error "usage: break-pc <pc> [wait_ms] | break-pc <slot> <pc> [wait_ms]" }
                set sub [lindex $tokens 1]
                if {$sub eq "off" || $sub eq "clear" || $sub eq "disable"} {
                    disable_break_pc
                    puts "> break-pc disabled"
                    puts "> [halt_status_line]"
                } else {
                    # 2026-09-06: a bare operand (`break-pc 40899706`) was fed
                    # straight to `expr`, which parses it as DECIMAL and then
                    # silently armed a completely unrelated address — the halt
                    # simply never fired and the run looked like a wedge.  Every
                    # other address operand in this REPL is hex, so REFUSE an
                    # unprefixed PC rather than guessing which base was meant.
                    if {[llength $tokens] >= 3 && [string is integer -strict $sub] && $sub >= 0 && $sub <= 3} {
                        set slot [expr {$sub}]
                        set pctok [lindex $tokens 2]
                        require_hex_pc $pctok
                        set bp [expr {$pctok}]
                        set wait_ms 200
                        if {[llength $tokens] > 3} { set wait_ms [expr {[lindex $tokens 3]}] }
                    } else {
                        set slot [next_free_break_pc_slot]
                        require_hex_pc $sub
                        set bp [expr {$sub}]
                        set wait_ms 200
                        if {[llength $tokens] > 2} { set wait_ms [expr {[lindex $tokens 2]}] }
                    }
                    arm_break_pc $bp $slot
                    clear_halt_req_preserve_control
                    after $wait_ms
                    puts "> break-pc slot=$slot target=[format 0x%08X $bp] waited=${wait_ms}ms"
                    puts "> [halt_status_line]"
                    if {![break_pc_reached $bp]} { break_pc_not_reached_note $bp }
                }
            }
            break-pc-list {
                list_break_pc
            }
            break-pc-disable -
            disable-break-pc {
                if {[llength $tokens] > 1} {
                    set slot [expr {[lindex $tokens 1]}]
                    disable_break_pc $slot
                    puts "> break-pc slot=$slot disabled"
                } else {
                    disable_break_pc
                    puts "> break-pc disabled"
                }
                puts "> [halt_status_line]"
            }
            irq-inject {
                set level [expr {[lindex $tokens 1]}]
                set count 1
                set delay_ms 10
                if {[llength $tokens] > 2} { set count [expr {[lindex $tokens 2]}] }
                if {[llength $tokens] > 3} { set delay_ms [expr {[lindex $tokens 3]}] }
                if {$level < 1 || $level > 7} {
                    puts "> ERROR irq-inject level must be 1..7"
                } elseif {$count < 1} {
                    puts "> ERROR irq-inject count must be >= 1"
                } else {
                    for {set i 0} {$i < $count} {incr i} {
                        dbg_wr $::OFF_IRQ_INJECT $level
                        if {$i + 1 < $count} { after $delay_ms }
                    }
                    puts "> irq-inject level=$level count=$count delay_ms=$delay_ms"
                    puts "> [halt_status_line]"
                }
            }
            adb-key {
                # adb-key <keycode> [down|up|press]  — enqueue an ADB
                # keyboard event (keycode 0..0x7F).  `press` (the
                # default) enqueues a down+up pair, which the keyboard
                # model reports as both bytes of one register-0 TALK.
                if {[llength $tokens] < 2 || [llength $tokens] > 3} {
                    error "usage: adb-key <keycode> \[down|up|press\]"
                }
                set kc [expr {[lindex $tokens 1]}]
                if {$kc < 0 || $kc > 0x7F} {
                    error "adb-key: keycode must be 0..0x7F (got $kc)"
                }
                set act press
                if {[llength $tokens] == 3} { set act [lindex $tokens 2] }
                switch -- $act {
                    down  { adbinj_wr $::ADBINJ_OFF_KBD $kc }
                    up    { adbinj_wr $::ADBINJ_OFF_KBD [expr {$kc | 0x80}] }
                    press {
                        adbinj_wr $::ADBINJ_OFF_KBD $kc
                        adbinj_wr $::ADBINJ_OFF_KBD [expr {$kc | 0x80}]
                    }
                    default { error "usage: adb-key <keycode> \[down|up|press\]" }
                }
                puts "> adb-key [format 0x%02X $kc] $act — [adb_status_line]"
            }
            adb-mouse {
                # adb-mouse <dx> <dy> [down|up] — inject a mouse motion
                # (and optionally a button transition).  Deltas are
                # signed 8-bit; the device model accumulates them and
                # saturates each TALK report at ±63.
                if {[llength $tokens] < 3 || [llength $tokens] > 4} {
                    error "usage: adb-mouse <dx> <dy> \[down|up\]"
                }
                set dx [expr {[lindex $tokens 1]}]
                set dy [expr {[lindex $tokens 2]}]
                if {$dx < -128 || $dx > 127 || $dy < -128 || $dy > 127} {
                    error "adb-mouse: deltas must be -128..127"
                }
                set btn ""
                if {[llength $tokens] == 4} {
                    set btn [lindex $tokens 3]
                    switch -- $btn {
                        down { adbinj_wr $::ADBINJ_OFF_BTN 1 }
                        up   { adbinj_wr $::ADBINJ_OFF_BTN 0 }
                        default { error "usage: adb-mouse <dx> <dy> \[down|up\]" }
                    }
                }
                adbinj_wr $::ADBINJ_OFF_DX $dx
                adbinj_wr $::ADBINJ_OFF_DY $dy
                set btn_note [expr {$btn eq "" ? "" : " btn=$btn"}]
                puts "> adb-mouse dx=$dx dy=$dy$btn_note — [adb_status_line]"
            }
            adb-status {
                # Read-only: keyboard FIFO + mouse pending flags.  The
                # kbd count draining to 0 after injection is live proof
                # the PIC firmware's TALK polling is consuming events.
                puts "> adb-status: [adb_status_line]"
            }
            reset {
                # Canonical unified reset (docs/reset_story.md §4.7).
                # Subcommands:
                #   reset                — pulse, do not hold.
                #   reset hold           — set hold, pulse, leave held.
                #   reset release        — clear hold (no pulse).
                #   reset hold-status    — read DBG_CONTROL.cold_reset_hold.
                set sub ""
                if {[llength $tokens] > 1} { set sub [lindex $tokens 1] }
                switch -- $sub {
                    "" {
                        unified_reset 0
                        puts "> reset done (pulse, no hold)"
                        puts "> [halt_status_line]"
                    }
                    hold {
                        unified_reset 1
                        puts "> reset done (pulse + hold; CPU held until `reset release`)"
                        puts "> [halt_status_line]"
                    }
                    release {
                        unified_reset_release
                        puts "> reset release done (cold_reset_hold cleared)"
                        puts "> [halt_status_line]"
                    }
                    hold-status -
                    status {
                        puts "> [unified_reset_hold_status]"
                    }
                    default {
                        puts "> ERROR unknown subcommand: reset $sub"
                        puts "> usage: reset \[hold|release|hold-status\]"
                    }
                }
            }
            reset-and-break-pc {
                if {[llength $tokens] < 2} {
                    error "usage: reset-and-break-pc <pc> \[slot\] \[wait_ms\]"
                }
                set bp [parse_num [lindex $tokens 1] "break PC"]
                set slot 0
                set wait_ms 2000
                if {[llength $tokens] > 2} { set slot [parse_num [lindex $tokens 2] "slot"] }
                if {[llength $tokens] > 3} { set wait_ms [parse_num [lindex $tokens 3] "wait_ms"] }
                reset_and_break_pc $bp $slot $wait_ms
                puts "> reset-and-break-pc pc=[format 0x%08X $bp] slot=$slot waited=${wait_ms}ms"
            }
            reset-and-halt-after {
                set n [expr {[lindex $tokens 1]}]
                set wait_ms 200
                if {[llength $tokens] > 2} { set wait_ms [expr {[lindex $tokens 2]}] }
                reset_and_halt_after $n $wait_ms
                puts "> reset-and-halt-after N=$n waited=${wait_ms}ms"
                puts "> [halt_status_line]"
            }
            vio-reset-and-halt-after {
                set n [expr {[lindex $tokens 1]}]
                set wait_ms 200
                if {[llength $tokens] > 2} { set wait_ms [expr {[lindex $tokens 2]}] }
                vio_reset_and_halt_after $n $wait_ms
                puts "> vio-reset-and-halt-after N=$n waited=${wait_ms}ms"
                puts "> [halt_status_line]"
            }
            vio-reset-halt-exc {
                set vec [expr {[lindex $tokens 1]}]
                set wait_ms 5000
                if {[llength $tokens] > 2} { set wait_ms [expr {[lindex $tokens 2]}] }
                vio_reset_halt_exc $vec $wait_ms
                puts "> vio-reset-halt-exc vec=$vec waited=${wait_ms}ms"
                puts "> [halt_status_line]"
            }
            reset-halt-exc {
                set vec [expr {[lindex $tokens 1]}]
                set wait_ms 8000
                if {[llength $tokens] > 2} { set wait_ms [expr {[lindex $tokens 2]}] }
                reset_and_halt_exc $vec $wait_ms
            }
            reset-halt-after {
                # DEPRECATED legacy alias — kept for one release.
                puts "> WARNING reset-halt-after is DEPRECATED; use `reset-and-halt-after` (canonical surface: `reset`)"
                set n [expr {[lindex $tokens 1]}]
                set wait_ms 200
                if {[llength $tokens] > 2} { set wait_ms [expr {[lindex $tokens 2]}] }
                reset_and_halt_after $n $wait_ms
                puts "> reset-halt-after N=$n waited=${wait_ms}ms"
                puts "> [halt_status_line]"
            }
            advance {
                # Advance N retired instructions from the CURRENT halted
                # state.  No reset.  Useful for iterative bring-up:
                # halt → inspect → advance 1000 → inspect → advance 1000 → ...
                set n [expr {[lindex $tokens 1]}]
                set wait_ms 200
                if {[llength $tokens] > 2} { set wait_ms [expr {[lindex $tokens 2]}] }
                set ok [advance $n $wait_ms]
                puts "> advance N=$n waited=${wait_ms}ms result=[expr {$ok ? "LANDED" : "NOT-LANDED"}]"
                puts "> [halt_status_line]"
            }
            step {
                set pcs [step]
                puts "> step pc_before=[format 0x%08X [expr {[lindex $pcs 0] & 0xffffffff}]] pc_after=[format 0x%08X [expr {[lindex $pcs 1] & 0xffffffff}]]"
                puts "> [halt_status_line]"
            }
            inst-count {
                # Read the captured retired-instruction count at the
                # last halt boundary.  Useful for diagnosing where in a
                # boot sequence we are.
                set n [current_halt_inst]
                puts "> inst-count = $n"
            }
            sweep {
                set wait_ms [expr {[lindex $tokens 1]}]
                set ns [lrange $tokens 2 end]
                foreach n $ns {
                    set ni [expr {$n}]
                    reset_and_halt_after $ni $wait_ms
                    set hr [dbg_rd $::OFF_HALT_REASON]
                    set hp [dbg_rd $::OFF_HALT_HIT_PC]
                    set pc [dbg_rd $::OFF_PC]
                    set ev [dbg_rd $::OFF_EXC_VEC]
                    set ep [dbg_rd $::OFF_EXC_PC]
                    set ec [dbg_rd $::OFF_EXC_COUNT]
                    # Task #22 bisection helpers: capture VBR + MMU state.
                    # These mark Mac OS arch-state transitions (VBR move
                    # to RAM, MMU enable, DTT reprogramming).
                    set vbr [dbg_rd $::OFF_LIVE_VBR]
                    set tc  [dbg_rd $::OFF_LIVE_MMU_TC]
                    set dt1 [dbg_rd $::OFF_LIVE_MMU_DTT1]
                    puts "> sweep N=$ni pc_live=0x$pc hit=0x$hp reason=0x$hr exc=$ev@0x$ep count=0x$ec vbr=0x$vbr tc=0x$tc dtt1=0x$dt1"
                    flush stdout
                }
            }
            arch -
            regs -
            reg-dump {
                dump_arch_registers
            }
            dump-mem {
                set addr [require_aligned [parse_num [lindex $tokens 1] "address"]]
                set n    [parse_num [lindex $tokens 2] "word count"]
                set words [rd_burst $addr $n]
                for {set i 0} {$i < $n} {incr i} {
                    puts "> mem [format 0x%08X [expr {$addr + $i*4}]] = 0x[lindex $words $i]"
                }
            }
            coherent-r {
                set addr [require_aligned [parse_num [lindex $tokens 1] "address"]]
                puts "> coherent mem [format 0x%08X $addr] = 0x[coherent-r $addr]"
            }
            coherent-dump {
                set addr [require_aligned [parse_num [lindex $tokens 1] "address"]]
                set n [parse_count [lindex $tokens 2] "word count"]
                set words [coherent-dump $addr $n]
                for {set i 0} {$i < $n} {incr i} {
                    puts "> coherent mem [format 0x%08X [expr {$addr + $i*4}]] = 0x[lindex $words $i]"
                }
            }
            coherent-w {
                set addr [require_aligned [parse_num [lindex $tokens 1] "address"]]
                set value [parse_num [lindex $tokens 2] "value"]
                coherent-w $addr $value
                puts "> coherent write [format 0x%08X $addr] = [format 0x%08X $value] (D/I invalidated)"
            }
            dump-frame-pgm {
                set path [lindex $tokens 1]
                set base [expr {[lindex $tokens 2]}]
                set stride [expr {[lindex $tokens 3]}]
                set bpp [expr {[lindex $tokens 4]}]
                set width [expr {[lindex $tokens 5]}]
                set height [expr {[lindex $tokens 6]}]
                write_frame_pgm $path $base $stride $bpp $width $height
                puts "> dump-frame-pgm wrote $path base=[format 0x%08X $base] stride=$stride bpp=$bpp size=${width}x${height}"
            }
            load-bit {
                # Loading a FOREIGN design (the SD provisioning bitstream) used
                # to hang here, twice in one session, and each hang cost a
                # 300 s timeout in sd_write_rom.sh / sd_os_swap.sh followed by
                # "provisioning load produced no success marker".  Two causes,
                # both fixed below:
                #
                #  1. PROBES.FILE is STICKY.  These callers invoke `load-bit
                #     $PROV` with no LTX, so the previous design's .ltx stayed
                #     applied and Vivado tried to bind fpga_top's probes to a
                #     provisioning design that has none of them.  Clear it when
                #     no LTX is given rather than inheriting a mismatched one.
                #
                #  2. The success marker was printed AFTER rebind_debug_cores
                #     and before check_build_id, so anything that stalled or
                #     errored while re-binding cores on a design without them
                #     swallowed the marker the caller was waiting for.  Print
                #     it as soon as the device is actually programmed, and make
                #     the debug-core rebinding non-fatal: a provisioning
                #     bitstream legitimately has no CPU debug block, and that
                #     must not abort the command or wedge the REPL.
                set bit [lindex $tokens 1]
                set_property PROGRAM.FILE $bit $::hw_dev
                program_hw_devices $::hw_dev
                if {[llength $tokens] > 2} {
                    set ltx [lindex $tokens 2]
                    set_property PROBES.FILE $ltx $::hw_dev
                } else {
                    set_property PROBES.FILE {} $::hw_dev
                }
                refresh_hw_device $::hw_dev
                set ::BIT $bit
                puts "> programmed $bit"
                if {[catch {rebind_debug_cores} rbErr]} {
                    puts "> load-bit: no CPU debug cores rebound ($rbErr) -- expected for a provisioning bitstream"
                    return
                }
                # Re-check build_id after every reload so an out-of-date
                # bitstream is caught immediately, not only at REPL startup.
                if {[catch {check_build_id} biErr]} {
                    puts "> load-bit: build_id unreadable ($biErr) -- expected for a provisioning bitstream"
                }
            }
            build_id -
            build-id {
                # Explicit build_id readback — useful when the user suspects
                # a stale-bitstream attach mid-session.  Reads OFF_BUILD_ID
                # (debug_ctrl base + 0x4) and compares against the canonical
                # `build_id=0x...` line in build/vivado/fpga_top.buildinfo.
                check_build_id
            }
            refresh {
                refresh_hw_device $::hw_dev
                rebind_debug_cores
                puts "> refreshed"
            }
            vio-set {
                set hv [lindex $tokens 1]
                vio_set $hv
                puts "> vio_set probe_out0 = 0x$hv"
            }
            vio-read {
                # Dump VIO *input* probes (live signal values).  Optional
                # arg is a substring filter on the probe name.
                #
                # Why this exists: the REPL could WRITE probe_out0/1 but had
                # no way to READ the ~23 probe_in bundles, so a whole class
                # of "is this signal even asserting?" question was
                # unanswerable from here and needed an ILA capture.  For the
                # 24bpp black-screen work the interesting ones are
                # vio_vram_read {vram_rd_valid, vram_rd_en} and vio_dafb_cfg
                # (dafb_fb_base_px / dafb_fb_stride_px as the SCANOUT sees
                # them, not the raw DAFB registers).
                #
                # CORRECTION 2026-08-05: this comment used to name
                # `vio_boot_video` as carrying fb_underflow_sticky.  There
                # is NO such probe -- the RTL declares the bundle but it was
                # never wired to the VIO.  Filtering for it returns nothing,
                # and a script that assumes a value gets a fabricated one.
                # For scan-out state use `video-status` (task #243), which
                # decodes a genuinely coherent one-clock-edge snapshot.
                set filt ""
                if {[llength $tokens] >= 2} { set filt [lindex $tokens 1] }
                set vios [get_hw_vios -quiet]
                if {[llength $vios] == 0} {
                    puts "> vio-read: no VIO core (need ENABLE_VIO=1 bitstream + LTX)"
                } else {
                    set vio [lindex $vios 0]
                    refresh_hw_vio $vio
                    set n 0
                    foreach p [get_hw_probes -of_objects $vio -quiet] {
                        set nm [get_property NAME $p]
                        if {$filt ne "" && [string first $filt $nm] < 0} { continue }
                        # OUTPUT_VALUE exists only on probe_out; skip those.
                        if {[catch {set v [get_property INPUT_VALUE $p]}]} { continue }
                        puts "> vio $nm = $v"
                        incr n
                    }
                    if {$n == 0} { puts "> vio-read: no input probes matched '$filt'" }
                }
            }
            video-status {
                # Task #243 — decode the coherent scan-out snapshot.
                #
                # Every field here is latched from ONE pclk edge in
                # video_top, so unlike reading video_debug_hcount /
                # video_debug_rgb as separate probes (each its own JTAG
                # transaction, seconds apart) these values are mutually
                # consistent and CAN be correlated.
                set vios [get_hw_vios -quiet]
                if {[llength $vios] == 0} {
                    puts "> video-status: no VIO core (need ENABLE_VIO=1 bitstream + LTX)"
                } else {
                    set vio [lindex $vios 0]
                    refresh_hw_vio $vio
                    # Select by the probe's declared WIDTH, not by the length
                    # of its value string: Vivado emits a 1-bit alias under
                    # the bare name alongside the real wide bus (see the
                    # collision guard in `vio-get`), and a value-length
                    # comparison picks whichever happens to print longer.
                    # Also force the radix — INPUT_VALUE is formatted per
                    # INPUT_VALUE_RADIX, so a probe left in BINARY (or one
                    # reading back X/U) silently produced a string that
                    # `expr {0x$snap}` rejected with a bareword error.
                    set snap ""   ; set snap_w 0
                    set place ""  ; set place_w 0
                    set fbrs ""   ; set fbrs_w 0
                    foreach p [get_hw_probes -of_objects $vio -quiet] {
                        set nm [get_property NAME $p]
                        set is_snap  [expr {[string first "video_dbg_snap"  $nm] >= 0}]
                        set is_place [expr {[string first "video_dbg_place" $nm] >= 0}]
                        # vio_fb_reader_stats: {miss_count, rsp_count, req_count}, 3 x 16
                        # bits (rtl/soc/fpga_top_debug_vio.vh).  It has existed
                        # since 2026-08-01 and was never displayed anywhere --
                        # docs/video_path_review.md S4.2.  req - rsp is the live
                        # outstanding-response count: small and stable means the
                        # byte-slip family is dead; drifting means that IS the
                        # bug, and the drift value is the horizontal displacement
                        # in SOURCE BYTES.
                        set is_fbrs  [expr {[string first "vio_fb_reader_stats" $nm] >= 0}]
                        if {!$is_snap && !$is_place && !$is_fbrs} { continue }
                        catch {set_property INPUT_VALUE_RADIX HEX $p}
                        if {[catch {set v [get_property INPUT_VALUE $p]}]} { continue }
                        set w 0
                        catch {set w [get_property WIDTH $p]}
                        if {$is_snap  && $w >= $snap_w}  { set snap  $v ; set snap_w  $w }
                        if {$is_place && $w >= $place_w} { set place $v ; set place_w $w }
                        if {$is_fbrs  && $w >= $fbrs_w}  { set fbrs  $v ; set fbrs_w  $w }
                    }
                    # Sanitise: strip an optional 0x and anything that is not
                    # a hex digit.  If what is left is empty the probe read
                    # back undefined (X/U) — say so instead of dying in expr.
                    set snap_raw  $snap
                    set place_raw $place
                    regsub -nocase {^0x} $snap  "" snap
                    regsub -nocase {^0x} $place "" place
                    regsub -nocase {^0x} $fbrs  "" fbrs
                    regsub -all {[^0-9a-fA-F]} $snap  "" snap
                    regsub -all {[^0-9a-fA-F]} $place "" place
                    regsub -all {[^0-9a-fA-F]} $fbrs  "" fbrs
                    if {$snap eq ""} {
                        puts "> video-status: video_dbg_snap unreadable (width=${snap_w} raw='${snap_raw}')"
                        puts "> video-status: (empty width=0 => pre-#243 bitstream; non-hex raw => probe reads X/U)"
                    } else {
                        puts "> video raw   : snap=0x$snap place=0x$place"
                        # NB: `expr {0x$snap}` does NOT work -- inside braces
                        # expr lexes the literal "0x" as a bareword BEFORE
                        # substituting $snap, and errors out.  Build the
                        # string first, then let expr convert the variable's
                        # value (Tcl bignum handles the full 96 bits).
                        set snap_hex "0x$snap"
                        set v [expr {$snap_hex}]
                        set hcount   [expr {($v >> 84) & 0xFFF}]
                        set vcount   [expr {($v >> 73) & 0x7FF}]
                        set de       [expr {($v >> 72) & 1}]
                        set live     [expr {($v >> 71) & 1}]
                        set rden     [expr {($v >> 70) & 1}]
                        set rdvalid  [expr {($v >> 69) & 1}]
                        set rdready  [expr {($v >> 68) & 1}]
                        set ufl_lb   [expr {($v >> 67) & 1}]
                        set ufl_fbr  [expr {($v >> 66) & 1}]
                        # [65:64] is the committed INTEGER scale N, encoded
                        # as N-1 so it still fits the two bits the old
                        # scale_sel enum used.  N display pixels per source
                        # pixel on both axes; there are no fractional ratios
                        # any more (docs/video_path_review.md S3).
                        set scale_n  [expr {(($v >> 64) & 0x3) + 1}]
                        # [9:6] is the LIVE verdict on the candidate
                        # placement -- valid every frame, not only after a
                        # refusal.  See reject_reason_name below.
                        set rej_live [expr {($v >> 6) & 0xF}]
                        set rgb      [expr {($v >> 40) & 0xFFFFFF}]
                        set bppsh    [expr {($v >> 37) & 0x7}]
                        set bytespx  [expr {($v >> 34) & 0x7}]
                        set hres     [expr {($v >> 22) & 0xFFF}]
                        set vres     [expr {($v >> 10) & 0xFFF}]
                        puts [format "> video raster: hcount=%d vcount=%d de=%d rgb=%06x" \
                                     $hcount $vcount $de $rgb]
                        puts [format "> video fetch : rd_en=%d rd_valid=%d rd_ready=%d underflow{linebuf=%d fb_reader=%d}" \
                                     $rden $rdvalid $rdready $ufl_lb $ufl_fbr]
                        puts [format "> video mode  : COMMITTED hres=%d vres=%d bpp_shift=%d bytes_per_px=%d scale=x%d (integer, %dx%d on screen)" \
                                     $hres $vres $bppsh $bytespx $scale_n \
                                     [expr {$hres * $scale_n}] [expr {$vres * $scale_n}]]
                        puts [format "> video admit : candidate verdict = %s" \
                                     [video_reject_reason_name $rej_live]]
                        if {$live} {
                            puts "> video source: dafb_live=1 (showing the DAFB framebuffer)"
                        } else {
                            puts "> video source: dafb_live=0 *** BOOT SPLASH SUBSTITUTED — the framebuffer is NOT being displayed ***"
                        }
                        if {$hres == 0 || $vres == 0} {
                            puts "> video WARNING: committed geometry is ${hres}x${vres} — the active window is EMPTY, nothing can render"
                        }
                    }
                    if {$place ne ""} {
                        set place_hex "0x$place"
                        set pv [expr {$place_hex}]
                        set base   [expr {($pv >> 32) & 0xFFFFFFFF}]
                        set stride [expr {$pv & 0xFFFFFFFF}]
                        puts [format "> video place : COMMITTED fb_base=0x%08x fb_stride=0x%08x (%d bytes/row)" \
                                     $base $stride $stride]
                        # The REJECTED tuple lives above bit 64 and only
                        # exists on a probe_map=v26+ bitstream (160-bit
                        # video_dbg_place). On an older one the field is simply
                        # absent, so gate on the probe WIDTH rather than
                        # printing a decode of bits that were never driven.
                        if {$place_w >= 160} {
                            set rej_sticky [expr {($pv >> 67) & 1}]
                            set rej_code   [expr {($pv >> 68) & 0xF}]
                            set rej_vres   [expr {($pv >> 72) & 0xFFF}]
                            set rej_hres   [expr {($pv >> 84) & 0xFFF}]
                            set rej_stride [expr {($pv >> 96) & 0xFFFFFFFF}]
                            set rej_base   [expr {($pv >> 128) & 0xFFFFFFFF}]
                            if {$rej_sticky} {
                                puts "> video admit : *** A PLACEMENT WAS REJECTED SINCE THE LAST COMMIT ***"
                                puts [format "> video admit : reason = %s" \
                                             [video_reject_reason_name $rej_code]]
                                puts [format "> video admit : refused tuple base=0x%08x stride=0x%08x (%d bytes/row) geometry=%dx%d" \
                                             $rej_base $rej_stride $rej_stride $rej_hres $rej_vres]
                            } else {
                                puts "> video admit : no placement rejected since the last commit"
                            }
                        } else {
                            puts "> video admit : (pre-v26 bitstream: video_dbg_place is ${place_w}b, no reject channel)"
                        }
                    }
                    # ── fb_reader request/response accounting ──────────
                    # req - rsp (mod 2^16) is the live outstanding count.
                    if {$fbrs eq ""} {
                        puts "> video fbrdr : vio_fb_reader_stats not present (need ENABLE_VIO=1 and the _1-suffixed probe)"
                    } else {
                        # Same two-step as the snap/place decodes above:
                        # `expr {0x$fbrs}` lexes "0x" as a bareword BEFORE
                        # substituting, and errors out.
                        set fbrs_hex "0x$fbrs"
                        set fv [expr {$fbrs_hex}]
                        set fb_req  [expr {$fv & 0xFFFF}]
                        set fb_rsp  [expr {($fv >> 16) & 0xFFFF}]
                        set fb_miss [expr {($fv >> 32) & 0xFFFF}]
                        set fb_out  [expr {($fb_req - $fb_rsp) & 0xFFFF}]
                        puts [format "> video fbrdr : req=%d rsp=%d miss=%d outstanding=(req-rsp)=%d" \
                                     $fb_req $fb_rsp $fb_miss $fb_out]
                        puts "> video fbrdr : sample this twice -- SMALL AND STABLE outstanding means the byte-slip family is dead; DRIFTING means that is the bug, and the drift is the horizontal displacement in SOURCE BYTES"
                    }
                }
            }
            vio-hard-reset {
                set hold_ms 50
                if {[llength $tokens] >= 2} { set hold_ms [lindex $tokens 1] }
                vio_hard_reset_pulse $hold_ms
                puts "> vio-hard-reset pulsed (hold=${hold_ms}ms) -- the WHOLE-DESIGN hard reset"
                puts "> vio-hard-reset: this now also resets the DDR4 MIG (soc_hard_rst_req,"
                puts "> vio-hard-reset:  fpga_top_clocks.vh).  The SoC holds ITSELF in reset until"
                puts "> vio-hard-reset:  re-calibration finishes -- allow ~200 ms before judging the"
                puts "> vio-hard-reset:  board dead, and expect boot_fsm to re-copy the ROM from SD."
                puts "> vio-hard-reset: core_clk does not come from mig_ui_clk, so JTAG stays up"
                puts "> vio-hard-reset:  through the re-cal.  fabric_gt_clr is deliberately NOT in"
                puts "> vio-hard-reset:  this path (it can kill dbg_hub's own clock)."
            }
            pram-clear {
                set hold_ms 50
                if {[llength $tokens] >= 2} { set hold_ms [lindex $tokens 1] }
                set base [pram_clear_pulse $hold_ms]
                puts "> pram-clear pulsed (hold=${hold_ms}ms) -- vio_boot_ctrl\[4\] 0->1->0, other probe_out0 bits preserved (0x[format %X $base])"
                puts "> pram-clear: RTC PRAM rewritten from its power-on image; needs probe_map=v20+ bitstream (5-bit probe_out0)"
            }
            pram-save { pram_save }
            pram-load { pram_load }
            pram-dump { pram_dump }
            vio-get {
                set name [lindex $tokens 1]
                set vios [get_hw_vios -quiet]
                if {[llength $vios] == 0} { error "no VIO core" }
                set vio [lindex $vios 0]
                refresh_hw_vio $vio
                set hit ""
                foreach p [get_hw_probes -of_objects $vio -quiet] {
                    if {[get_property NAME $p] eq $name} { set hit $p; break }
                }
                if {$hit eq ""} { puts "> ERROR: probe '$name' not found"; } else {
                    # Collision guard: Vivado synthesis can leave a net named
                    # EXACTLY "$name" (some unrelated, often single-bit,
                    # aliased/optimized net) while the real, wider bus you
                    # actually declared in RTL gets uniquified to "${name}_1",
                    # "${name}_2", etc. -- a bare NAME-equality lookup then
                    # silently returns the wrong, unrelated probe with no
                    # error (confirmed 2026-07-19: vio_rst_bundle vs
                    # vio_rst_bundle_1 on the jtagaxi-ila build). Warn loudly
                    # whenever a same-prefixed, wider sibling exists so this
                    # doesn't waste another investigation silently.
                    set hit_width [get_property WIDTH $hit]
                    foreach p [get_hw_probes -of_objects $vio -quiet] {
                        set pname [get_property NAME $p]
                        if {[regexp "^${name}_\[0-9\]+\$" $pname]} {
                            set pw [get_property WIDTH $p]
                            if {$pw > $hit_width} {
                                puts "> WARNING: '$name' is only ${hit_width}b wide, but '$pname' (${pw}b) also exists -- likely a Vivado net-name collision; '$pname' is probably the signal you actually want (try: vio-get $pname)"
                            }
                        }
                    }
                    set v [get_property INPUT_VALUE $hit]
                    puts "> vio_get $name = 0x$v"
                }
            }
            probes {
                set vios [get_hw_vios -quiet]
                puts "> vio_count = [llength $vios]"
                foreach v $vios {
                    puts "> vio: $v"
                    foreach p [get_hw_probes -of_objects $v -quiet] {
                        set nm [get_property NAME $p]
                        set pn ""
                        catch {set pn [get_property PROBE_NAME $p]}
                        puts ">   probe NAME=$nm PROBE_NAME=$pn"
                    }
                }
            }
            full-reset {
                # DEPRECATED legacy alias — routes through the unified
                # reset (cold_reset_pulse via JTAG-AXI, NOT VIO bit 3).
                # Kept for one release per docs/reset_story.md §4.7.
                puts "> WARNING full-reset is DEPRECATED; use `reset` (canonical unified reset)"
                unified_reset 0
                puts "> full-reset done"
                puts "> [halt_status_line]"
            }
            full-reset-and-halt {
                # DEPRECATED legacy alias — kept for one release.
                puts "> WARNING full-reset-and-halt is DEPRECATED; use `reset-and-halt-after N` or `reset hold`"
                set n 0
                if {[llength $tokens] > 1} { set n [expr {[lindex $tokens 1]}] }
                if {$n > 0} {
                    reset_and_halt_after $n 200
                } else {
                    # No N — leave CPU halted post-reset via cold_reset_hold.
                    unified_reset 1
                }
                puts "> full-reset-and-halt N=$n"
                puts "> [halt_status_line]"
            }
            pc-trace -
            last-pcs {
                if {![dbg_has_feature pc_trace]} {
                    error "pc-trace: hardware does not advertise pc_trace"
                }
                # Dump up to N most recent PCs from the trace ring.
                # Token 1 (optional): N to dump (default = full depth).
                #
                # The depth is a BUILD-TIME parameter, not a constant: it
                # comes from m68k_axi_wrapper.v's .PC_TRACE_DEPTH() override
                # (currently 64; debug_ctrl.v's own default is 256).  Ask the
                # bitstream via OFF_CAP_TRACE rather than trusting the local
                # $::PC_TRACE_DEPTH, which is only a fallback for bitstreams
                # predating that capability register.
                #
                # This is not hypothetical tidiness: on 2026-08-02 the
                # hardcoded 64 next to debug_ctrl.v's default of 256 led to a
                # wrong conclusion that the REPL was under-reading the ring
                # 4x and mis-ordering the dump.  Reading the real value makes
                # the tool self-describing and makes a depth change take
                # effect without touching this file.
                set depth $::PC_TRACE_DEPTH
                set cap [rdx_or_empty [expr {$::DBG_BASE + $::OFF_CAP_TRACE}]]
                if {$cap ne "" && $cap != 0} {
                    set d [expr {($cap >> 16) & 0xFFFF}]
                    # Ring indexing needs a power-of-two depth for the mask.
                    if {$d > 0 && ($d & ($d - 1)) == 0} { set depth $d }
                }
                set mask [expr {$depth - 1}]
                if {$depth != $::PC_TRACE_DEPTH} {
                    puts "> pc-trace: bitstream reports depth $depth (local default $::PC_TRACE_DEPTH)"
                }
                set n $depth
                if {[llength $tokens] > 1} { set n [expr {[lindex $tokens 1]}] }
                if {$n > $depth} {
                    puts "> pc-trace: requested $n entries but the ring is only $depth deep -- dumping $depth"
                    set n $depth
                }
                set head [expr {[scan [dbg_rd $::OFF_PC_TRACE_HEAD] %x] & $mask}]
                puts "> pc-trace head=$head depth=$depth dumping $n entries (oldest→newest)"
                for {set k $n} {$k > 0} {incr k -1} {
                    set idx [expr {($head - $k) & $mask}]
                    set v [dbg_rd [expr {$::OFF_PC_TRACE_BASE + $idx*4}]]
                    puts "> trace\[[format %4d $idx]\] pc=0x$v"
                }
            }
            exc-ring -
            exc -
            excring -
            last-exceptions {
                if {![dbg_has_feature exc_ring]} {
                    error "exc-ring: hardware does not advertise exc_ring"
                }
                # Dump up to N most recent exceptions from the ring.
                # New-core entries end in installed handler PC. Legacy entries end
                # in the historical exception count; select by DBG_VERSION.
                # Default N = ring depth (32).  Output is newest-first so
                # you can read top-down to see "what just happened".
                set n $::EXC_RING_DEPTH
                if {[llength $tokens] > 1} { set n [expr {[lindex $tokens 1]}] }
                if {$n > $::EXC_RING_DEPTH} { set n $::EXC_RING_DEPTH }
                set head [expr {[scan [dbg_rd $::OFF_EXC_RING_HEAD] %x] & $::EXC_RING_MASK}]
                puts "> exc-ring head=$head — last $n entries (newest→oldest)"
                # Tally vec/pc distribution for the visible window so the
                # "is it one PC looping?" question is answered in one shot.
                array unset pc_tally
                for {set k 1} {$k <= $n} {incr k} {
                    set idx [expr {($head - $k) & $::EXC_RING_MASK}]
                    set base [expr {$::OFF_EXC_RING_BASE + $idx*16}]
                    set vec [scan [dbg_rd [expr {$base + 0}]] %x]
                    set pc  [dbg_rd [expr {$base + 4}]]
                    set fa  [dbg_rd [expr {$base + 8}]]
                    set tail [dbg_rd [expr {$base + 12}]]
                    if {[dbg_is_core040_epoch]} {
                        puts "> exc\[[format %2d $idx]\] vec=0x[format %02x $vec] pc=0x$pc fa=0x$fa handler=0x$tail"
                    } else {
                        puts "> exc\[[format %2d $idx]\] vec=0x[format %02x $vec] pc=0x$pc fa=0x$fa count=0x$tail"
                    }
                    if {[info exists pc_tally($pc)]} {
                        incr pc_tally($pc)
                    } else {
                        set pc_tally($pc) 1
                    }
                }
                puts "> exc-ring tally (pc → count in window):"
                foreach pc [lsort [array names pc_tally]] {
                    puts ">   0x$pc × $pc_tally($pc)"
                }
            }
            branch-ring -
            last-branches {
                if {![dbg_has_feature branch_ring]} {
                    error "branch-ring: hardware does not advertise branch_ring"
                }
                set depth $::BRANCH_RING_DEPTH
                set cap [scan [dbg_rd $::OFF_CAP_TRACE2] %x]
                if {($cap & 0xffff) != 0} { set depth [expr {$cap & 0xffff}] }
                set n $depth
                if {[llength $tokens] > 1} { set n [parse_count [lindex $tokens 1] "entry count"] }
                if {$n > $depth} { set n $depth }
                set mask [expr {$depth - 1}]
                set head [expr {[scan [dbg_rd $::OFF_BRANCH_RING_HEAD] %x] & $mask}]
                puts "> branch-ring head=$head depth=$depth — last $n entries (newest→oldest)"
                for {set k 1} {$k <= $n} {incr k} {
                    set idx [expr {($head - $k) & $mask}]
                    set base [expr {$::OFF_BRANCH_RING_BASE + $idx*16}]
                    set pc [dbg_rd $base]
                    set next [dbg_rd [expr {$base + 4}]]
                    set meta [scan [dbg_rd [expr {$base + 8}]] %x]
                    puts "> branch\[[format %2d $idx]\] pc=0x$pc next=0x$next taken=[expr {$meta & 1}] mispredict=[expr {($meta >> 1) & 1}] type=[expr {($meta >> 2) & 3}]"
                }
            }
            live-arch {
                # Live arch register readback via the snap chain (cRAT→PRF).
                # Stable only when CPU is halted; racy under free-run.
                set force 0
                if {[llength $tokens] > 1 && [lindex $tokens 1] eq "force"} {
                    set force 1
                }
                set eff [effective_halt]
                if {!$force && !$eff} {
                    puts "> ERROR live-arch requires effective halt; run halt-status, break-pc, advance, or use `live-arch force`"
                    puts "> [halt_status_line]"
                } else {
                    # `force` stays supported — it has legitimate uses — but a
                    # forced read of a RUNNING CPU walks the snap chain while
                    # the pipeline mutates underneath it.  The values come back
                    # stale, torn, or both, and they look exactly like good
                    # ones.  `force` is used habitually, so marking the header
                    # only is not enough: EVERY line is re-tokenised so that a
                    # caller grepping `A7 = ` matches NOTHING here rather than
                    # silently harvesting a stale register.
                    set unsafe [expr {!$eff}]
                    if {$unsafe} {
                        puts "> ############################################################"
                        puts "> # live-arch force WITHOUT an effective halt — CPU IS RUNNING"
                        puts "> # Every value below is UNRELIABLE (stale and/or torn: the"
                        puts "> # snap chain is read register-by-register while the pipeline"
                        puts "> # keeps retiring).  These are NOT a measurement of any single"
                        puts "> # point in time.  Note the `?=` — it is deliberate, so these"
                        puts "> # lines cannot be mistaken for `NAME = value` output."
                        puts "> ############################################################"
                        set eq "?="
                        set tag "UNRELIABLE "
                    } else {
                        set eq "="
                        set tag ""
                    }
                    for {set i 0} {$i < 8} {incr i} {
                        set v [dbg_rd [expr {$::OFF_LIVE_D0 + $i*4}]]
                        puts "> ${tag}D$i $eq 0x$v"
                    }
                    for {set i 0} {$i < 8} {incr i} {
                        set v [dbg_rd [expr {$::OFF_LIVE_A0 + $i*4}]]
                        puts "> ${tag}A$i $eq 0x$v"
                    }
                    set sr_hex [dbg_rd $::OFF_LIVE_SR]
                    set sr_err [sr_check_valid [scan $sr_hex %x]]
                    if {$sr_err eq ""} {
                        puts "> ${tag}SR  $eq 0x$sr_hex"
                    } else {
                        puts "> ${tag}SR  $eq 0x$sr_hex  <<< $sr_err"
                    }
                    puts "> ${tag}VBR $eq 0x[dbg_rd $::OFF_LIVE_VBR]"
                    puts "> ${tag}A7  $eq 0x[dbg_rd $::OFF_LIVE_A7]"
                    puts "> ${tag}PC  $eq 0x[dbg_rd $::OFF_PC]"
                    if {$unsafe} {
                        puts "> # ^ all UNRELIABLE — CPU was not halted.  [halt_status_line]"
                    }
                }
            }
            fault-snap {
                puts "> [fault_snap_status_line]"
            }
            rts-snap {
                puts "> [rts_snap_status_line]"
            }
            live-mmu {
                # Dump the MMU CSRs the CPU's translation pipeline sees.
                # Useful for "did the walker fault?" diagnosis on a wedge:
                # if TC bit 15 (E) is 1 AND DTT0/DTT1 don't cover the
                # faulting VA, the walker traverses SRP/URP page tables —
                # any missing PTE → vec=2.  JTAG-AXI bypasses MMU so the
                # xbar response alone won't show this fault path.
                set tc   [dbg_rd $::OFF_LIVE_MMU_TC]
                set dtt0 [dbg_rd $::OFF_LIVE_MMU_DTT0]
                set dtt1 [dbg_rd $::OFF_LIVE_MMU_DTT1]
                set itt0 [dbg_rd $::OFF_LIVE_MMU_ITT0]
                set itt1 [dbg_rd $::OFF_LIVE_MMU_ITT1]
                set srp  [dbg_rd $::OFF_LIVE_MMU_SRP]
                set urp  [dbg_rd $::OFF_LIVE_MMU_URP]
                set tci  [scan $tc %x]
                set tc_e [expr {($tci >> 15) & 1}]
                puts "> live-mmu: TC=0x$tc (E=$tc_e)"
                puts "> live-mmu: DTT0=0x$dtt0 DTT1=0x$dtt1"
                puts "> live-mmu: ITT0=0x$itt0 ITT1=0x$itt1"
                puts "> live-mmu: SRP=0x$srp  URP=0x$urp"
            }
            fault-snap-clear {
                dbg_wr $::OFF_FAULT_SNAP_CLEAR 0x1
                puts "> fault-snap cleared"
            }
            rts-snap-clear {
                dbg_wr $::OFF_RTS_SNAP_CLEAR 0x1
                puts "> rts-snap cleared"
            }
            continue -
            cont -
            c {
                # Phase 3 of 2026-05-11 debug-improvements: precise-BP
                # continue.  In theory debug_ctrl auto-arms
                # break_pc_skip_once_r=1 on dbg_break_uop_fire.  Observed
                # 2026-05-14: on the current bitstream the auto-arm bit
                # reads back as 0 by the time the host issues `continue`
                # — either the consume-on-decode raced ahead, or the
                # auto-arm path lost the slot index.  Either way, the
                # host must explicitly arm skip_once before releasing
                # halt, otherwise decode re-injects SYS_DBG_BREAK on the
                # very next fetch of break_pc and the BP fires again
                # without forward progress (single-step also fails for
                # the same reason).
                #
                # Arm skip_once for every currently-enabled BP slot.
                # The bit auto-clears on the decode pass that consumes
                # it, so subsequent visits to break_pc still trip the
                # BP as expected.
                # Only arm skip_once when we're actually halted on a
                # precise BP.  Other halt sources (halt-after-N,
                # halt-on-exc, manual) don't need the skip.  This MUST go
                # through bp_halt_latched: on the Stage 5 core the halt
                # source is a primary reason code, and the legacy
                # HALT_CTL bit 5 this used to read is RAZ there.
                set bp_latched [bp_halt_latched]
                set en_slots 0
                if {$bp_latched} {
                    set en_slots [expr {[scan [dbg_rd $::OFF_BREAK_PC_CTRL] %x] & 0xF}]
                    if {$en_slots != 0} {
                        dbg_wr $::OFF_BP_SKIP_ONCE $en_slots
                    }
                }
                # Halt-after auto-advance is owned by RTL: clearing the
                # latched halt reloads the next target from the saved delta.
                # Do not rewrite HALT_AFTER here; host restaging masks RTL
                # regressions and can perturb the count on fixed bitstreams.
                clear_auto_halt
                clear_halt_req_preserve_control
                after 100
                puts "> continue done (skip_once_armed=$bp_latched slots=0x[format %x $en_slots])"
                puts "> [halt_status_line]"
            }
            wedge-status {
                puts "> [wedge_status_line]"
            }
            dcache-probe {
                set set_idx [expr {[lindex $tokens 1]}]
                set way_idx [expr {[lindex $tokens 2]}]
                set word_idx 0
                if {[llength $tokens] > 3} { set word_idx [expr {[lindex $tokens 3]}] }
                dcache-probe $set_idx $way_idx $word_idx
            }
            icache-probe {
                set set_idx [parse_num [lindex $tokens 1] "set index"]
                set way_idx [parse_num [lindex $tokens 2] "way index"]
                set word_idx 0
                if {[llength $tokens] > 3} {
                    set word_idx [parse_num [lindex $tokens 3] "word index"]
                }
                icache-probe $set_idx $way_idx $word_idx
            }
            icache-lookup {
                icache-lookup [parse_num [lindex $tokens 1] "address"]
            }
            dcache-op {
                dcache-op [lindex $tokens 1]
            }
            icache-op {
                if {[llength $tokens] > 1} {
                    icache-op [lindex $tokens 1]
                } else {
                    icache-op
                }
            }
            arch-write {
                # Write a single arch shadow reg, no apply.
                # Usage: arch-write <name> <hex_val>
                #   name = D0..D7 / A0..A7 / SR / VBR / PC
                set name [string toupper [lindex $tokens 1]]
                set val  [expr {[lindex $tokens 2]}]
                require_effective_halt "arch-write"
                set off [arch_reg_offset $name]
                dbg_wr $off $val
                puts "> arch-write $name = [format 0x%08X $val] (staged, not yet applied)"
            }
            arch-apply {
                set status [arch_apply_wait]
                puts "> arch-apply status=[format 0x%08X $status] done=1 rejected=0 effective_halt=1"
            }
            reg-set {
                if {[llength $tokens] != 3} { error "usage: reg-set <name> <value>" }
                require_effective_halt "reg-set"
                set name [string toupper [lindex $tokens 1]]
                set val [parse_num [lindex $tokens 2] "register value"]
                dbg_wr [arch_reg_offset $name] $val
                arch_apply_wait
                puts "> reg-set $name = [format 0x%08X $val] applied; CPU remains effectively halted"
            }
            halt-exc-mask {
                # Write or read the 256-bit halt-on-exc mask (8 × 32-bit lanes).
                # Usage:
                #   halt-exc-mask                    — dump all 8 lanes + enable
                #   halt-exc-mask <vec>              — set single bit (vec 0..255)
                #   halt-exc-mask <vec> off          — clear single bit
                #   halt-exc-mask raw <lane> <hex>   — write entire 32-bit lane
                #
                # Legacy debug_stop_manager gates the mask with HALT_CTL bit 6.
                # The m68k040 Stage-5 contract makes the mask itself authoritative;
                # halt_exc_enable_sync handles that epoch distinction.
                #
                #   dbg_halt_exc_hit_now = dbg_boundary_event &&
                #       dbg_halt_exc_enable &&
                #       (dbg_boundary_kind == DBG_BOUNDARY_EXC) &&
                #       dbg_halt_exc_mask[dbg_boundary_exc_vec];
                #
                # Until 2026-08-03 this command wrote ONLY the mask lane, so
                # `halt-exc-mask 11` armed a vector that could never halt: the
                # exception fired ~178/s, exc_count climbed, and the CPU ran
                # straight through it.  That is the worst kind of instrument
                # failure — a green-looking arm that measures nothing — and it
                # cost a full day of "the breakpoint never hits" (task #235).
                # Setting the bit here makes arming mean what it says.
                #
                # RMW rather than a bare write, and masked to the ENABLE bits
                # (0/1/6): OFF_HALT_CTL's write block assigns halt_after_enable
                # from bit 0 unconditionally (debug_ctrl.v:2218), so the bare
                # `dbg_wr HALT_CTL $HALT_EXC_EN` used by the reset-* helpers
                # silently DISARMS an armed halt-after.  Bit 2 (HALT_CLEAR)
                # reads back as 0 (debug_ctrl.v:1503), so the readback can
                # never accidentally re-acknowledge a halt.
                if {[llength $tokens] == 1} {
                    for {set lane 0} {$lane < 8} {incr lane} {
                        set v [dbg_rd [expr {$::OFF_HALT_EXC_MASK + $lane*4}]]
                        puts "> halt-exc-mask lane$lane (vec [expr {$lane*32}]..[expr {$lane*32+31}]) = 0x$v"
                    }
                    if {[dbg_is_core040_epoch]} {
                        set en 0
                        for {set lane 0} {$lane < 8} {incr lane} {
                            if {[scan [dbg_rd [expr {$::OFF_HALT_EXC_MASK + $lane*4}]] %x] != 0} {
                                set en 1; break
                            }
                        }
                        puts "> halt-exc-mask active (non-zero mask) = $en"
                    } else {
                        # Report the legacy enable too — its absence makes an armed
                        # mask silently inert on debug_stop_manager hardware.
                        set hc [scan [dbg_rd $::OFF_HALT_CTL] %x]
                        set en [expr {($hc & $::HALT_EXC_EN) ? 1 : 0}]
                        puts "> halt-exc-mask enable (HALT_CTL bit6) = $en[expr {$en ? {} : {   <-- ARMED VECTORS CANNOT HALT}}]"
                    }
                } elseif {[lindex $tokens 1] eq "raw"} {
                    set lane [expr {[lindex $tokens 2]}]
                    set hex  [expr {[lindex $tokens 3]}]
                    dbg_wr [expr {$::OFF_HALT_EXC_MASK + $lane*4}] $hex
                    halt_exc_enable_sync
                    puts "> halt-exc-mask lane$lane <- [format 0x%08X $hex]"
                } else {
                    set vec  [expr {[lindex $tokens 1]}]
                    set off_b [expr {$vec & 0x1F}]
                    set lane  [expr {($vec >> 5) & 0x7}]
                    set lane_addr [expr {$::OFF_HALT_EXC_MASK + $lane*4}]
                    set cur [scan [dbg_rd $lane_addr] %x]
                    if {[llength $tokens] > 2 && [lindex $tokens 2] eq "off"} {
                        set new [expr {$cur & ~(1 << $off_b)}]
                        puts "> halt-exc-mask vec=$vec OFF (lane$lane bit$off_b)"
                    } else {
                        set new [expr {$cur | (1 << $off_b)}]
                        puts "> halt-exc-mask vec=$vec ON  (lane$lane bit$off_b)"
                    }
                    dbg_wr $lane_addr $new
                    halt_exc_enable_sync
                }
            }
            wedge-why {
                # ONE command that answers "the machine is dead after a `reset` -- why?"
                #
                # An ARBITER_WEDGE halt says the CPU's merge arbiter held a grant with
                # no progress. It does NOT say whether the core could not even issue an
                # address (a reset absorber still waiting for a response the fabric
                # abandoned) or whether an address DID go out and the fabric never
                # answered. Those need opposite fixes, and until 2026-09-17 neither was
                # readable on silicon -- which is precisely what made the board tests
                # inconclusive. Read this BEFORE theorising.
                set hk  [scan [dbg_rd $::OFF_HALT_KIND] %x]
                set arb [scan [dbg_rd $::OFF_STALL_ARB] %x]
                set abs [scan [dbg_rd $::OFF_STALL_ABSORB] %x]
                set hnames [list "NONE" "DCACHE_DIAG" "FS_XLATE" "RESET_VECTOR" \
                                 "ARBITER_WEDGE" "WALKER_PORT_WEDGE" "reserved(6)" "reserved(7)"]
                puts [format "> halt-kind      = 0x%08X -> %s" $hk [lindex $hnames [expr {$hk & 7}]]]
                if {($hk & 7) == 0} {
                    puts "> NOTE: halt-kind reads 0 for up to ~10 s after a reset even when the"
                    puts ">       arbiter IS wedged -- the D20 bounded-grant watchdog has to expire."
                    puts ">       Re-read before concluding the machine is healthy."
                }

                set onames [list "DCACHE" "ITLB" "DTLB" "RESETVEC"]
                puts [format "> OFF_STALL_ARB  = 0x%08X" $arb]
                puts [format ">   read : busy=%d arTaken=%d owner=%s req=0x%X wedge=%d" \
                        [expr {$arb & 1}] [expr {($arb >> 1) & 1}] \
                        [lindex $onames [expr {($arb >> 2) & 3}]] \
                        [expr {($arb >> 4) & 0xF}] [expr {($arb >> 8) & 1}]]
                puts [format ">   write: busy=%d awTaken=%d owner=%s req=0x%X wedge=%d" \
                        [expr {($arb >> 16) & 1}] [expr {($arb >> 17) & 1}] \
                        [lindex $onames [expr {($arb >> 18) & 3}]] \
                        [expr {($arb >> 20) & 0xF}] [expr {($arb >> 24) & 1}]]

                puts [format "> OFF_STALL_ABSORB = 0x%08X" $abs]
                foreach {name shift} [list axi_d 0 axi_i 16] {
                    set w    [expr {($abs >> $shift) & 0xFFFF}]
                    set outs [expr {$w & 0x1F}]
                    set absg [expr {($w >> 5) & 1}]
                    set strv [expr {($w >> 6) & 1}]
                    set arf  [expr {($w >> 7) & 0xF}]
                    set rf   [expr {($w >> 11) & 0xF}]
                    set blk  [expr {($w >> 15) & 1}]
                    puts [format ">   %s: outstanding=%d absorbing=%d starved=%d arFires=%d rLastFires=%d blockingCore=%d" \
                            $name $outs $absg $strv $arf $rf $blk]
                    # The verdict. Each branch names the NEXT place to look, because a
                    # bit dump that still needs interpreting is how the last two board
                    # tests ended up inconclusive.
                    if {$absg && !$strv} {
                        puts ">     VERDICT: the absorber is STILL WAITING for a response the fabric owes."
                        puts ">              Re-read in ~10 s: if `starved` then goes to 1 the fabric"
                        puts ">              ABANDONED it (hunt the abort path in axi_xbar.v); if it stays"
                        puts ">              0 the escape itself is not counting -- check cpu_rst is not"
                        puts ">              stuck asserted, which freezes the bound by design."
                    } elseif {$strv} {
                        puts ">     VERDICT: a response really WAS abandoned -- the escape fired and"
                        puts ">              released the core. If the machine is STILL dead, the wedge"
                        puts ">              is DOWNSTREAM of this absorber, not in it."
                    } elseif {$arf > 0 && $rf == 0} {
                        puts ">     VERDICT: the fabric took an address and never answered it."
                        puts ">              This is the 'slave never responds' class -- hunt the LOST"
                        puts ">              RESPONSE in the xbar/L2C/MIG path, NOT in the core."
                    } elseif {$arf == 0} {
                        puts ">     VERDICT: the core never got an address out on this master at all."
                        if {$blk} {
                            puts ">              ...and it IS asking while the absorber refuses. Absorber."
                        } else {
                            puts ">              ...and it is not even asking. Look upstream, in the core."
                        }
                    } else {
                        puts ">     VERDICT: this master looks healthy (addresses out, responses back)."
                    }
                }
                puts "> (counts are saturating at 15 and are scoped to SINCE THE LAST RESET)"
                puts {>}
                puts {> NEXT, on the FABRIC side -- run `vio-read boot` and decode vio_boot_diag:}
                puts {>   bit 31    = xbar_s1_slot_busy   (a slot still outstanding to S1)}
                puts {>   bits 30:25 = xbar_slv_poisoned[5:0], i.e. S0..S5 -> bits 25..30}
                puts {>}
                puts {>   poisoned bit 28 (S3/VRAM) set  -> the S3 quarantine timed out. Under}
                puts {>        VRAM_IN_DDR, S3 and S0 SHARE one DDR port, so an S3 read stall}
                puts {>        takes every DDR read with it. The read quarantine sinks only ONE}
                puts {>        burst while the read side has no per-slave owner lock, so two}
                puts {>        masters reading VRAM at once can leave a stale burst behind.}
                puts {>   poisoned = 0x00 and bit 31 set -> an S1 slot stranded OUTSIDE the flush}
                puts {>        window (the pb-side CDC leaves reset a few pb_clk later than the}
                puts {>        core side; with ENABLE_WD=0 nothing releases a transaction lost}
                puts {>        in that gap).}
                puts {>   poisoned = 0x00 and bit 31 clear -> points at S0/DDR. A slot-0 read to}
                puts {>        DDR has NO abort path at all: S0 is not in the flush domain and}
                puts {>        the slot-0 owner-gone term is S1-only.}
            }
            halt-kind {
                set k [dbg_rd $::OFF_HALT_KIND]
                set ki [expr {[scan $k %x] & 0x7}]
                set names [list "NONE (no fatal condition latched)" \
                                "DCACHE_DIAG (D-cache diagnostic fault)" \
                                "FS_XLATE (FSAVE/FRESTORE frame translation fault)" \
                                "RESET_VECTOR (reset-vector FSM halt)" \
                                "ARBITER_WEDGE (AxiDMerge arbiter wedge)" \
                                "WALKER_PORT_WEDGE (walker/D-cache merge wedge)" \
                                "reserved(6)" "reserved(7)"]
                puts "> halt-kind = 0x$k -> [lindex $names $ki]"
                set hr [dbg_rd $::OFF_HALT_REASON]
                puts "> (OFF_HALT_REASON = 0x$hr; 4 = FATAL, which is what makes this register meaningful)"
            }
            pc-range-halt {
                # cpu040 PC-RANGE halt lane.  Enable lives in the debug reset
                # domain, so an arm survives `reset`.
                set a [lindex $tokens 1]
                if {$a eq "" || $a eq "status"} {
                    puts "> pc-range-halt ctl=0x[dbg_rd $::OFF_PCRANGE_CTL] lo=0x[dbg_rd $::OFF_PCRANGE_LO] hi=0x[dbg_rd $::OFF_PCRANGE_HI] pc0=0x[dbg_rd $::OFF_PCRANGE_PC0] pc1=0x[dbg_rd $::OFF_PCRANGE_PC1] pc2=0x[dbg_rd $::OFF_PCRANGE_PC2] count=0x[dbg_rd $::OFF_PCRANGE_COUNT]"
                } elseif {$a eq "off"} {
                    dbg_wr $::OFF_PCRANGE_CTL 0
                    puts "> pc-range-halt OFF (ctl=0x[dbg_rd $::OFF_PCRANGE_CTL])"
                } else {
                    set lo [parse_num $a "lo"]
                    set hi [parse_num [lindex $tokens 2] "hi"]
                    if {$hi < $lo} { error "pc-range-halt: hi < lo" }
                    dbg_wr $::OFF_PCRANGE_LO $lo
                    dbg_wr $::OFF_PCRANGE_HI $hi
                    dbg_wr $::OFF_PCRANGE_CTL 1
                    puts "> pc-range-halt ARMED [format 0x%08X $lo]..[format 0x%08X $hi] (ctl=0x[dbg_rd $::OFF_PCRANGE_CTL])"
                }
            }
            a7-odd-halt {
                # cpu040 A7-ODD halt lane.  a7-odd-halt <thresh> arms it (halt with
                # reason 7 once the committed A7 stays odd for <thresh> retired
                # macros); a7-odd-halt off disarms; a7-odd-halt status dumps the
                # lane registers.  The enable lives in the debug reset domain, so
                # it survives `reset` / reset-and-halt-after like halt-exc-mask.
                set a [lindex $tokens 1]
                if {$a eq "" || $a eq "status"} {
                    puts "> a7-odd-halt ctl=0x[dbg_rd $::OFF_A7ODD_CTL] pc0=0x[dbg_rd $::OFF_A7ODD_PC0] pc1=0x[dbg_rd $::OFF_A7ODD_PC1] pc2=0x[dbg_rd $::OFF_A7ODD_PC2] a7=0x[dbg_rd $::OFF_A7ODD_VALUE] episodes=0x[dbg_rd $::OFF_A7ODD_COUNT]"
                } elseif {$a eq "off"} {
                    dbg_wr $::OFF_A7ODD_CTL 0
                    puts "> a7-odd-halt OFF (ctl=0x[dbg_rd $::OFF_A7ODD_CTL])"
                } else {
                    set th [expr {$a}]
                    if {$th < 1 || $th > 65535} { error "a7-odd-halt: thresh must be 1..65535" }
                    dbg_wr $::OFF_A7ODD_CTL [expr {($th << 16) | 1}]
                    puts "> a7-odd-halt ARMED thresh=$th (ctl=0x[dbg_rd $::OFF_A7ODD_CTL])"
                }
            }
            exc-count {
                set v [dbg_rd $::OFF_EXC_COUNT]
                puts "> exc_count = 0x$v"
            }
            tcl {
                # Generic raw-Tcl passthrough — runs in the same Vivado
                # session that already holds hw_target open, so native
                # Vivado commands (get_hw_ilas, set_property
                # TRIGGER_COMPARE_VALUE, run_hw_ila, wait_on_hw_ila,
                # upload_hw_ila_data, display_hw_ila_data, ...) work
                # directly without a second, contending JTAG connection.
                # Added 2026-07-14 for the v7 hw_ila probe recipes in
                # docs/ila_a7_drift_probes.md and
                # rtl/soc/fpga_top_debug_vio.vh (v7 probe-map comment) --
                # this REPL has no per-investigation ILA logic of its
                # own; it just exposes Vivado's own ILA Tcl API.
                set rest [string range $line [expr {[string length $cmd] + 1}] end]
                set result [uplevel #0 $rest]
                puts "> tcl: $result"
            }
            q -
            quit -
            exit {
                puts "> bye"
                flush stdout
                exit 0
            }
            default {
                puts "> ERROR unknown cmd: $cmd"
            }
        }
    } err]} {
        puts "> ERROR $err"
    }
    puts "> READY"
    flush stdout
}
