# BUG: `MOVEM.L (An)+` with a non-longword-aligned base DEADLOCKS the core

**Status**: **FIXED and hardware-verified.** Identified on HW 2026-08-26 via
JTAG precise-PC breakpoint + `reg-set` controlled-variable experiment (below).
Root-caused, bisected, and fixed in cpu040 commit `e7ee618a` ("split-load ring
`alignedSendHeld` used a positional proxy that wedges on ring wrap") — see
that commit's message for the full mechanism, and `LsEuSplitRingSpec`'s
targeted regression test. Re-verified on real hardware the same day (SoC
`8b37000`, cpu040 `e7ee618a`, bitstream build_id `0x8B370005`): the identical
controlled experiment at **both** documented wedge sites below now runs
straight through, followed by 77s of sustained retirement, and `halt` lands
at a coherent macro boundary in 0ms where it previously could never land.
**Severity was**: CRITICAL — hard boot blocker. The Q700 ROM could not get
past early boot; the core wedged permanently with nothing retiring and no bus
traffic.
**Originally affected build**: SoC `cb66e37`/`e1a93de`, cpu040 submodule
`4698963d` (branch `fmax-postroute-200mhz-closure`), bitstream build_id
`0xCB66E373`.

## Post-fix status (2026-08-26)

Both documented wedge sites (`0x40815646` / ISP `0x0017FFBA`, and
`0x4080c5b8` / ISP `0x0017FFC2`) now execute straight through on `continue`.
The board proceeds far past this point into real Mac ROM Toolbox execution
(proper A-line trap dispatch, deep ROM handler code at `0x4084xxxx`).

### Boot-outcome triage (2026-08-26, second session)

**The two follow-on anomalies were diagnosed. Neither is this deadlock, and
neither is a single condition — boot is NONDETERMINISTIC.** Three distinct
terminal states were observed across four boots of the *identical* bitstream
(build_id `0x8B370005`, verified clean each time, ROM shadow re-streamed by
`boot_fsm` on every cold reset; `r 0x40800000` = `0x420dbff3` confirms the
image is intact). This matches `tools/boot_outcome.sh`, which already
classifies these as *intermittent* outcomes — its recorded 2026-08-18
baseline had ~1 boot in 3 reach the Shutdown dialog while others parked "in
the ROM serial monitor at `0x4084a8xx`".

#### Outcome A — the `0x4084a840`–`0x4084afca` loop: a LEGITIMATE wait, but on a failure path

**2026-08-26, later same day: a second, independent route INTO this exact
loop was hardware-proven** — a genuine DIVU-by-zero (vector 5) at ROM
`0x40843a90` (`DIVU.W $0D00.W,D0`), caused by low-memory calibration word
`0x00000D00` reading back `0x0000`, routed through the shared early-boot
fault trampoline into this same monitor loop. This is a *different* path
than the SCSI/`c96_phase_bits()`/D7-bit-26 lead documented later in this
section — the DIVU causal chain does not pass through that gating code at
all. Full writeup, including a hardware-narrowed (but not yet fully
proven) hypothesis for why `0x00000D00` is zero, plus a newly-found,
separate, reproducible boot hang ("Outcome D", a Memory-Manager heap-zone
linked-list walk stuck on a corrupted pointer at `0x40806774`-`0x40806786`):
see `docs/BUG_calibration_word_misplaced_0d00.md`.

This is **not a bug in the loop**. It is the Q700 ROM's **serial diagnostic /
operator monitor REPL**, polling the SCC for a command that never arrives.
Fully decoded from `files/420dbff3.rom` and confirmed against `branch-ring`,
which shows a clean, exactly-13-instruction cycle with `mispredict=0`:

```
0x4084a840  lea   %pc@(0x4084a848),%fp
0x4084a844  jmp   %pc@(0x4084af9c)      ; call rom_scc_rx_poll
0x4084af9c  movew #0x8000,%d0           ; default = "no character"
0x4084afa0  btst  #17,%d7               ; monitor-I/O-active latch — SET on HW
0x4084afa4  beqs  0x4084afca            ; not taken
0x4084afa6  btst  #0,%a3@(2)            ; SCC chan-A RR0 bit 0 = Rx Char Available
0x4084afac  beqs  0x4084afca            ; TAKEN — no character
0x4084afca  jmp   %fp@                  ; return, D0 = 0x8000
0x4084a848  tstw  %d0
0x4084a84a  bmiw  0x4084a966
0x4084a966  btst  #16,%d7
0x4084a96a  beqw  0x4084aa08
0x4084aa08  braw  0x4084a840             ; loop closed
```

`regs` at a halt in this loop: `A3 = 0x50F0C020` (SCC chan A, = ROM machine
descriptor `+0x0C`), so `%a3@(2)` is **`0x50F0C022` = chan-A RR0**, and
`%a3@(6)` = `0x50F0C026` = chan-A data. `D7 = 0x00020000` — bit 17 set, so
unlike the sim path in `BUG_macsbug_repl_unreached.md`, on **real hardware
the monitor's SCC I/O body does execute** and the banner was printed. It is
simply waiting for an operator keystroke on the serial console.

**Re-confirmed on a later build, exact same PCs (2026-08-26, third session).** A
fresh SoC-integration build worktree (`build-2026-08-26-fixes`, bitstream
build_id `0x4339FA79`, carrying today's TTR/`mmuEnable` fix `ceebdee7` +
StoreQueue cross-page forward-hazard fix `7700c3a8` + SQ forward-comparator
FMax fold `9e0af36f`, on top of this same MOVEM fix `e7ee618a`) landed on a
cold boot at `PC = 0x4084a848` — **the literal same instruction address**
inside this loop, not merely the same routine. `halt-status` / `regs` /
`coherent-dump 0x4084a838 20` / `pc-trace 32` / `branch-ring 16` were all
re-captured live and match this section byte-for-byte:

* Disassembly at `0x4084a838`–`0x4084a884` decodes to the identical
  `lea`/`jmp`/`tst.w`/`bmi.w`/`btst`/`beq.w`/`bra.w` sequence transcribed
  above, confirming `A6 = 0x4084a848` (the `lea %pc@(0x4084a848),%a6` result)
  and the `jmp %pc@(0x4084af9c)` call target, both exactly as documented.
* `regs`: `A3 = 0x50F0C020`, `A5 = 0x4084a840`, `A6 = PC = 0x4084a848`,
  `D7 = 0x00020000` (bit 17 set, bit 26 clear), `VBR = 0x0040091a` (2 mod 4),
  `ITT0 = ITT1 = DTT0 = DTT1 = 0x00000000`, `TC = 0x0000c000` (MMU on),
  `CACR = 0x00008000` (I-cache only) — all identical to the outcome-A
  snapshot already on record below. `D0 = 0x05a08000`: the low word is
  `0x8000`, i.e. `rom_scc_rx_poll`'s "no character" default is intact
  (the high word `0x05a0` is stale from the earlier `UnivROMFlags` load, not
  part of this routine's contract).
* `pc-trace 32` dumped a perfect repeating cycle of the same 13 PCs listed
  above (`...a840→a844→af9c→afa0→afa4→afa6→afac→afca→a848→a84a→a966→a96a→
  aa08→` repeat), confirming this is the identical tight polling spin, not a
  slow crawl or a different loop that merely shares an address.
* `branch-ring 16` shows `mispredict=0` on every entry except one single
  `mispredict=1` at the `beq.w 0x4084a96a→0x4084aa08` edge (plausibly a
  cold-start misprediction before the ring's steady state locks in) —
  consistent with "clean, exactly-13-instruction cycle" and not a new
  anomaly.
* `exc-ring` head was static with only pre-existing vec `0x0a`/`0x05` entries
  from earlier in boot (handler `0x408099b0`, `0x408026f6`), i.e. zero new
  exceptions fired while parked here — same "pure polling wait, not an
  exception-retry loop" signature as the first observation.

**Conclusion: this is not a new bug.** It is outcome A, re-observed
verbatim (same PCs, same registers, same loop shape) on a build that
carries three additional, unrelated fixes landed since the first
observation. Those fixes did not change this outcome's occurrence, which is
expected — outcome A's own root-cause lead (the SCSI `c96_phase_bits()` gap,
below) is untouched by any of them. Per the brief's investigation
methodology, no new `docs/BUG_*.md` was created for this; this section was
updated in place instead. The board was `halt-release`d after data capture
per the standing "don't leave it needlessly halted" convention — this
outcome does not need a follow-up live session, since the mechanism is
already fully understood.

**Why we are in the monitor at all** is the real bug. `D7 = 0x00020000` means
**bit 26 is CLEAR**. There is exactly one site in the whole ROM that sets it
(`0x40846d5a: bset #26,%d7`), and `0x40849b08: btst #26,%d7 / bne` routes a
clear bit through `0x40849b24 → braw 0x4084a7e6` = `rom_monitor`. The
gating chain at `0x40846cbe`..`0x40846d3a` is:

* `A2 = *(A0+8) = 0x50F00000` (VIA1, loaded at `0x40846c8e`). The ROM clears
  DDRA bit 0 (`bclr #0,%a2@(1536)` = VIA1 reg 3) to make PA0 an input, then
  `btst #0,%a2@(7680)` (VIA1 reg 15, `vBufA`). **PA0 == 0 sets D7[26]
  immediately.** Our `via1_pa_in` is strapped `8'hC1`, so PA0 = 1 and this
  fast path is skipped — deliberately, per the comment at
  `rtl/soc/fpga_top_peripherals.vh:1189`, which *already documents this exact
  failure*: "Earlier 0xC0 attempts diverged at the d7 bit 26 latch
  (0x40846d5a) and dropped the boot into the operator prompt at 0x4084a82e."
* With PA0 = 1 the ROM instead needs a `'SCBI'` descriptor lookup, then the
  RAM probe at `0x408470ba`, then a peripheral-ID check. `D0[8] = 0` (the
  Q700 feature bitmap is `0x05a0_183f`) selects the branch at `0x40846d20`:
  `A3 = *(A0+0x60) = 0x50F0F000` (the 53C96), `moveb %a3@(64),%d3` reads the
  **SCSI status register** (offset `0x40` = reg 4, `rtl/mac/scsi.v:2122`),
  and requires `(status & 7) == 4` — i.e. **SCSI phase bits == `3'b100`**.
* `rtl/mac/scsi.v:1962` `c96_phase_bits()` can only ever return `000, 001,
  010, 011, 110, 111`. **`3'b100` is not producible**, so this path can never
  pass either.

That is a concrete, localised lead for outcome A, but it needs confirmation
by breaking at `0x40846cd4` / `0x40846d3a` on a boot that actually lands in
the monitor; it is **not** yet proven to be the taken path (there are two
other `braw 0x4084a7e6` entries, at `0x40849b96` and `0x40849bcc`, that are
not gated on D7[26]).

##### 2026-08-27, follow-up session: SCSI phase-bits lead DISPROVEN as the
##### taken path; real root cause is a different, unresolved boot-stage
##### control-flow divergence (`$0DB0` sentinel never gets stamped)

**The SCSI `c96_phase_bits()` lead above is CONFIRMED NOT TAKEN**, live on
real cpu040 hardware. Using the project's single-slot `reset hold` /
`break-pc` / `reset release` pattern (never the two-slot interactive form,
per this section's own tooling-bug note above) across three independent
fresh boots:

* `break-pc 0x40846d3a` (the SCSI-gate decision point) — **never fired**.
  `halt-status` polling showed PC going straight from ~`0x4084730e` to the
  `0x4084a840` loop without ever visiting it.
* `break-pc 0x40849b96` (first documented ungated `braw 0x4084a7e6` entry)
  — **never fired** either.
* `break-pc 0x40849bcc` (second documented ungated entry) — **also never
  fired**.

So none of the three previously-hypothesized entry points is what actually
happens. `rtl/mac/scsi.v`'s `c96_phase_bits()` gap is real (confirmed: `3'b100`
truly is unproducible, and is a correctly-modeled SCSI-spec reserved/undefined
phase encoding, not a bug — see the byte-lane note below) but **irrelevant to
this boot hang**, since that code path is never executed at all. No RTL
change was made to `scsi.v`.

**Byte-lane/endianness hypothesis also checked and ruled out** (raised as an
alternate theory given a real big-endian instruction-fetch bug, commit
`b0f0ae9`, was fixed 6 days before this session): `rtl/soc/peripheral_bus.v:1514`
(`rd_data_q <= {4{pb_rd_byte(rd_slot_q)}}`) replicates the single relevant
byte across all 4 AXI lanes for every standard MMIO slot (VIA1/VIA2/SCC/SCSI
non-DMA registers) — confirmed live via `coherent-dump` (bypasses the LSU
entirely): VIA1 `vBufA` read back `0xe1e1e1e1`, another VIA1 register read
back `0x28282828`. A CPU-side byte-lane-selection bug would be
undetectable for this whole class of access since every lane already holds
the identical value; a real lane bug, if any, would have to show up on
non-replicated data (DRAM, or the SCSI pseudo-DMA shim) which isn't in play
here.

**The real entry mechanism, hardware-confirmed**: breaking at the
convergence PC `0x4084a7e6` (common to all 3 known static call sites) DID
fire. `pc-trace` immediately after showed arrival via straight-line
fallthrough from `0x4084a7d4` (a VBR-relocation preamble: allocates a
256-byte scratch vector table on the stack, relocates 64 vector-table
longwords by a computed delta, `movec`s into VBR) — **not** via any of the
3 known branches. An exhaustive byte/opcode-level scan of the whole ROM
(every `Bcc`/`BRA`/`BSR`/`JMP`/`JSR`/`LEA` encoding, absolute and
PC-relative, plus a raw 4-byte literal-pointer scan) found **zero** static
references to `0x4084a7d4` anywhere — it is reached only via a
runtime-computed indexed jump (`0x4080246e: lea 0x48366,%a0` /
`0x40802474: jmp %pc@(...,%a0:l)`), which resolves (verified both by
address arithmetic and live hardware) to a **fixed**, register-independent
target of `0x4084a7d4` every time it executes.

Working backward one more level (`pc-trace` + a corrected re-disassembly —
the naive linear objdump of `0x40802300`+ was **misaligned/decoding
garbage-as-code**; re-disassembling from the actual live entry point
`0x40802310` fixed it, the same "objdump desyncs through data" pitfall this
whole investigation keeps hitting) found the actual gate:

```
0x40802316: cmpi.l  #0x5A932BC7, ($0DB0).w
0x4080231e: bne.w   0x4080246e        ; mismatch -> (via the fixed computed
                                       ;  jump chain above) -> monitor loop
```

**Confirmed live**: `coherent-dump 0x00000da0 8` while halted just past this
check read `$0DB0 = 0xFFFFFFFF` (never written), not the expected
`0x5A932BC7`. The magic constant appears exactly 4 times in the ROM: once
as this compare's immediate, once at the very end of the ROM image
(`0x408ffffa`, almost certainly the ROM's own signature/footer), and once
at `0x40800c7e` — `movel #0x5A932BC7, $0DB0.w`, the actual writer. That
writer sits at the tail of a small subroutine (`~0x40800bf0`-`0x40800c9e`)
that issues a sequence of Device-Manager-style calls (`moveq #22/27/2/6/5,d0`
+ trap `$A06E`, each followed by `bne -> abort`) culminating in trap
`$A071` and then the `$0DB0` stamp — i.e. **`$0DB0 = 0x5A932BC7` is a "the
boot device driver opened and verified successfully" sentinel**, and
`0x4080231e`'s check is literally "do we have a verified boot device? if
not, give up into the operator monitor."

**Bisection result (real hardware vs. MAME, "did the driver-open sequence
even run" question)**:

* Real hardware: `break-pc 0x40800bf0` (the driver-open subroutine's own
  entry) **never fired**, confirmed on two independent fresh boots — the
  code that would even attempt to open/verify the boot device is never
  reached at all this boot. This is stronger than "the check outran the
  write" (which would still visit both PCs, just reordered) — it is a
  genuine control-flow divergence upstream, not a close race within shared
  code.
* MAME (healthy boot): `0x40800bf0` hit at cyc=133,680,342, the `$0DB0`
  stamp write (`0x40800c7e`) at cyc=133,795,153 — both complete with large
  margin before the `0x40802736`-region retry loop (clocked separately at
  cyc≈198,833,892) that leads to the `0x40802316` check. So on a healthy
  timeline this is not a close race either; the writer simply always runs
  first, well before the checker.

**Update, same session: the static caller chain WAS found**, via a much
cheaper technique than static idiom-matching — MAME's own debugger
expression syntax (`d@<addr>` reads a dword; note `dword(addr)` is **not**
valid syntax in this MAME build and fails silently-ish with "unknown
symbol", a real tooling gotcha worth remembering) can read the top of stack
at a `bpset` hit. Since a `bsr`/`jsr` pushes its return address before
entering the callee, breaking at the callee's entry and reading `d@a7`
gives the exact call site for free, no idiom-guessing needed. This
immediately found what the 138-computed-jump static scan missed — the real
caller is a **plain, direct `bsrw`**, which that scan's Bcc/BRA pattern
(bug in the scan itself: it only matched opcode high byte `0x60`, i.e. BRA
condition code 0000, and missed `0x61xx` BSR and every other `0x6Xxx` Bcc
variant) simply never covered:

```
0x4080102e: moveq #21,d0
0x40801030: trap  $A06E                  ; Device-Manager-style call, csCode=21
0x40801032: beqs  0x4080103a             ; success (Z=1) -> proceed
0x40801034: bsrw  0x40800f40 / bra 0x4080104c   ; FAILURE path -> $0DB0 stays unstamped
0x4080103a: bsrw  0x40800bf0             ; SUCCESS path -> the driver-open-and-stamp sequence
0x4080103e: bnew  0x4080102e             ; (0x40800bf0 failing loops back to retry csCode 21)
```

So the immediate gate on `0x40800bf0` is a single, concrete Device-Manager
call: **does `trap $A06E` with `D0=21` return success (Z=1)?**

**But real hardware shows the divergence is even further upstream than
that.** `break-pc 0x40801032` (right after this exact trap, on a fresh
boot) **never fired** — cpu040 doesn't even reach the point of making the
csCode-21 call. The enclosing routine itself, entry `0x40801000` (found the same way: MAME
shows it's called via `0x4080114c: bsrw 0x40801000`, unconditionally in
the immediate local context; return address into `0x40801000`'s own
caller is `0x40801150`), is **also never reached** on real cpu040
hardware, confirmed as its own independent `break-pc 0x40801000` miss on a
separate fresh boot. MAME's
healthy boot reaches all of these normally (`0x40801000` hit at
cyc=133,664,708, `0x40800bf0` at cyc=133,680,342 — consistent, nested,
in-order), confirming this is real cpu040-specific divergence, not dead
ROM code.

One further level was attempted (guessing the enclosing function around
`0x40801060`, which also writes into the same `$0DA0`-region table via
trap `$A051` — consistent with `$0DA0`/`$0DA4` being populated while
`$0DA8`+ stays blank, i.e. this whole `$0DA0`-`$0DBC` span is one
sequential, multi-step low-memory init block that gets partway through
and stops) but the MAME evidence didn't fit (`0x40801060` hit at
cyc=179,088,625 — *later* than `0x40800bf0`, and with an unrelated-looking
parameter, A1=VIA1 base) — that guess was wrong, `0x40801060` is a
different, unrelated shared subroutine that happens to sit nearby. The
true caller of the `0x4080114c` call site (and hence the actual fork
point) was not found this session — it needs locating the real `link`
prologue that contains `0x4080114c`, not yet done.

A parallel check for a shared selector with the `machine-descriptor-slot4-fix`
bug (same session, `docs/BUG_calibration_word_misplaced_0d00.md` Part 14 —
that bug's own ID=4-vs-ID=9 selector was also never located) found the two
mechanisms share the same generic descriptor-lookup trampoline
(`0x408470ba`, whose own computed jump lands at `0x40802f18`, in the same
code region as this gate) but use **different** backing tables
(`0x408031c4` here vs. the `0x40803234`-stride-`0xA4` table for the slot4
bug) — structurally related via shared machinery, not confirmed to be
literally the same selector value.

**Fix status: none applied.** Per explicit direction this session: do NOT
fake/force-write the `$0DB0` sentinel directly (risks masking a genuine
downstream driver-open failure or papering over a real SCSI response gap
that would just resurface later, e.g. once Mac OS itself tries to use a
boot volume that was never really opened). The intended fix shape, once the
dispatch point is found, is to correct whichever branch/selector is
misrouting so the real `$A071`/`$A06E` driver-open sequence actually runs
(mirroring `via-alias-corruption-fix`'s approach of forcing a real CCR/branch
outcome rather than faking device data) — not yet designed, pending
locating the actual dispatch mechanism into `0x40800bf0`.

##### Further same-session work: v1 differential (decisive), a retracted
##### false lead, and where this stands at session end

**v1 differential — decisive against the SoC-decode-fidelity explanation.**
Warm-switched the physical board from cpu040 to v1 (`tcl boot_hw_device
$::hw_dev` from inside the cpu040 REPL — SPI-flash boot, no bitstream
rewrite — then quit and relaunched v1's own `tools/jtag_repl.tcl` from
`/home/qwertyoruiop/macqd700-soc` with `JTAG_REPL_NO_PROGRAM=1`; v1's
command set differs slightly — `r <addr>` not `coherent-dump`, `live-arch
force` not `regs`). On a **fresh v1 reset**, `break-pc 0x40800bf0` (the
driver-open subroutine cpu040 never reaches) fired immediately;
`live-arch force` confirmed `PC=0x40800BF0`. On v1 already running near
Finder, `r 0xdb0` read `0x5A932BC7` — correct. v1 shares the identical
SoC/SCSI RTL/ROM/SD-card image and is not blocked here, so this rules out
`via-alias-corruption-fix`'s bug class (residual open-bus/DECERR SoC
decode gaps) as the explanation for this specific hang — it is a genuine
cpu040-only divergence.

**Retracted: an interrupt-dispatch-chain reframing.** Mid-session, reading
`A6=0x40804182` at a `0x4080114c` breakpoint was mistaken for this ROM's
`lea pc@(x),%fp / jmp shared_routine` return-register convention (real
and confirmed correct elsewhere, e.g. this section's own monitor-entry
preamble analysis) and chased into an unrelated VBR/vector-table
initializer at `0x408025f0`. Checked and disproven two ways: (1) a MAME
watchpoint on `$0120` (thought to be a handler-chain head) showed its
only real write is `0x4080263e: clrl 0x120`, never written again before
being read — stays NULL all boot, not a live signal; (2) the dispatcher
body this pointed at (`0x40802650`, entry/`jsr a1@`/abort-path all
breakpointed) got **zero hits across a full 60-second healthy MAME
boot-to-Finder run** — it's not invoked at all on a healthy boot. The
`0x40802690`-`0x408027be` region this section spent the most time on
earlier is **not**, after all, what gates `0x4080114c` — it's unrelated
in-flight code that happens to sit nearby.

**The real immediate caller chain**, found via MAME's `d@<addr>`
stack-read trick (reliable here because these are genuine `bsr`s with
real pushed return addresses): `0x4080114c: bsrw 0x40801000` is inside a
function entered at `0x408010f0` (an initial read found `0x408010f4`,
4 bytes in — another linear-decode-alignment miss), itself called via
`0x40800284: bsrw 0x408010f0`, which sits in a block of **completely
unconditional, straight-line early-boot code** (`0x40800260`-`0x4080029e`,
no branches at all) that also calls the unrelated `0x40801060` sibling
routine my earlier guess had mistaken for the ancestor. **What calls
*that* block, and whether cpu040 ever reaches it, was not determined this
session** — a full MAME instruction trace attempt hit a real scaling wall
on several giant repeated ROM self-test/checksum loops between the RAM
test and this code's cycle range (tens of MB of trace text per attempt,
never escaping a single loop pass), and further one-level-at-a-time
caller hunts ran out of session budget.

**Two live hypotheses remain** for what actually blocks cpu040, neither
confirmed nor eliminated: (1) boot-race timing, the same *class* as the
`0x0D00` calibration-word fix — plausible, was the leading hypothesis
walking in, but the specific mechanism (what real-time dependency, where
the race happens) was not identified this session, since the lead that
would have supported a concrete version of it (`$0120`) was retracted
above; (2) some other, uncharacterized cpu040-specific divergence — a
parallel wide-lens synthesis pass raised a branch-misprediction/
frontend-recovery hypothesis (cpu040's NaxRiscv-derived BTB+predictor),
checked via live `branch-ring`/`pc-trace` at the nearest reachable
downstream PC (`0x40802690`): only ordinary cold-BTB-miss mispredictions
were found, all resolving to the *correct* target — no direct evidence,
but not a clean negative either, since the capture couldn't be taken AT
the actual fork point (the skip is an absence, not a redirect to a known
wrong PC, so there's no "moment of the skip" to break on). A separate
parallel RTL-audit pass found no evidence for a branch-misprediction/
recovery bug either, but that doesn't rule out other core-side
explanations.

**Recommended next step**: a real-hardware ILA capture spanning the boot
window around this divergence (roughly the `0x40800260`-`0x40801150` PC
range, or a data trigger on the `$0DB0` write/read) is very likely the
right tool now — this project has a proven ILA pipeline from earlier this
session (the Chipscope 16-213 tie-off fix / packed-probe-mapping work),
and it would show the actual PC sequence directly instead of requiring
precise advance knowledge of where to plant a breakpoint, which is what
has made manual bisection here so slow and false-lead-prone.

See `docs/BUG_calibration_word_misplaced_0d00.md` Part 15 for the same
material as a standalone executive summary with explicit confirmed/
ruled-out/open triage.

See `docs/BUG_calibration_word_misplaced_0d00.md` Part 15 for the
cross-reference and full session log.

##### 2026-09-03 addendum: "the SCSI `c96_phase_bits()` gap" label retired —
##### re-confirmed spec-correct (not a bug) and re-confirmed off the taken
##### path; no RTL fix made or warranted; see Part 104 for full detail

A follow-up session dispatched specifically to pin down and, if possible,
fix the `c96_phase_bits()`/`3'b100` lead above concluded **there is nothing
to fix**. Two independent findings, both new this session and not in the
disproof above:

1. **`3'b100` is a genuinely reserved SCSI phase code, not an
   implementation gap.** Cross-checked against MAME's own generic SCSI-bus
   core (`nscsi_bus.h`'s `S_PHASE_*` enum, the base every MAME SCSI device
   including `ncr53c90_device`/53C96 builds on): only six `{MSG,C/D,I/O}`
   combinations are ever defined (`000/001/010/011/110/111`); `100`/`101`
   have no defined constant anywhere in MAME's SCSI core, because real
   SCSI protocol always asserts C/D alongside MSG — `MSG=1,C/D=0` cannot
   occur on real bus wiring. `rtl/mac/scsi.v:1959-1976`'s `c96_phase_bits()`
   implements exactly this same six-entry table and is explicitly commented
   as tracking MAME's `ncr53c90_device::status_r()` line-for-line. The ROM
   boot-gate's `(status & 7) == 4` check could never pass against real
   53C96 silicon either — this was never a real target to hit.
2. **This document's own disproof above still stands and was not
   re-litigated with new hardware time** (Part 14/15's repeated,
   independent, real-`break-pc` misses on all three monitor-entry sites
   already constitute strong, reproduced-across-sessions evidence; a
   redundant fourth hardware run was judged an unjustified use of JTAG-board
   time given finding 1 above already makes the outcome certain regardless).

This also means the actual reason the 5th outcome above still recurs on
current builds (see `docs/BUG_calibration_word_misplaced_0d00.md` Parts 91/
97/98, which use "the `scsi.v c96_phase_bits()` gap" as loose shorthand for
"the `0x4084a840` park point") is **mislabeled** in those later entries —
that shorthand contradicts this section's own Part-15-era disproof and
should not be repeated. The real still-open blocker is the `$0DB0`
driver-open-sequence divergence documented above and tracked at length in
the calibration doc from Part 15 onward; the `0x4084a840` loop itself,
once reached via that gate, is expected ROM operator-monitor behavior, not
a further bug. Full write-up, MAME source citations, and disposition:
`docs/BUG_calibration_word_misplaced_0d00.md` Part 104.

#### Outcome B — the "wild jump + bus-error storm": REPRODUCED, and already documented

This **does** reproduce (seen on a fresh, clean, just-programmed boot), and it
is **not new**. It is the same failure this repo diagnosed on 2026-05-06 in
`docs/diag-bus-fault-51001c00.md` + `docs/diag-buserror-frame-format.md`,
now recurring on cpu040:

| | 2026-05-06 (legacy core) | 2026-08-26 (cpu040 `e7ee618a`) |
|---|---|---|
| stuck PC | `0x408046AA` | `0x408046AA` |
| SR | `0x2710` | `0x2710` |
| D7 | `0x08000000` | `0x08000000` |
| garbage JMP target | `0xEBD20004` | `0xE0390004` |

(The prior session's one-off `0x39000620` is the same family — a garbage
`JMP` target, differing per boot.)

`0x408046AA` is `tstb %a2@(0,%d2:l)` with `A2 = 0x50F01C00` (VIA1 IER) and
`D2 = 0x00100000` — the ROM's **VIA1 address-alias probe**, called from
`0x40803174`. `exc-ring` saturates with `vec=0x02 pc=fa=handler=0xE0390004`
and `pc-trace` freezes (zero retirement) while the exception ring keeps
advancing — a pure fault storm.

**Causality, corrected:** the `0xE0390004` values found in low memory
(`0x20`–`0x3C`) are *not* a corrupted vector table. Decoding the bytes at
`0x20` gives `27 10 | e0 39 00 04 | 70 08` = **SR `0x2710`, PC `0xE0390004`,
format/vector word `0x7008` → format 7, vector offset 8 = vector 2**. That is
a textbook 68040 format-$7 access-error frame. The storm's runaway SP is
*writing frames over low memory*; the garbage is the consequence, not the
cause.

Per `diag-buserror-frame-format.md` the frame layout itself is correct and
the root cause is upstream: **A6 loaded with garbage from RAM**, plus an odd
`A7`/`VBR` (the ROM legitimately runs an unaligned VBR — see
`tb/tests/asm/unaligned_long_vector_dispatch.s`; do **not** mask VBR). That
doc's leading hypothesis is *"misaligned-LONG load reconstruction in the
LSU"* — **the same family as the bug this document fixed.** The MOVEM fix
(`e7ee618a`) closed the split-ring wedge but evidently not this whole class.
Corroborating: on the outcome-A boot, `regs` reported `VBR = 0x0040091a`
(2 mod 4) with `ITT0/ITT1/DTT0/DTT1` all zero and the MMU on, whereas the
healthy boot (outcome C) reported `VBR = 0x00000000`.

##### 2026-08-27 update: caught live with `reset-halt-exc 2` — REVISES the root-cause picture

Board: same worktree, bitstream build_id `0x4339FA79`. Armed
`reset-halt-exc 2 8000` from a fresh cold boot (after a precautionary full
`load-bit` reprogram — see the tooling note at the end of this subsection),
which halts *before* any wild-jump/frame-corruption cascade has a chance to
develop, at the moment vector 2 is first taken. This changes the
conclusion in two concrete ways.

**1. This is a genuine MMU/ATC access fault, not a bus-protocol (SLVERR/xbar)
error.** The stacked format-7 frame's SSW, read directly off the stack
(`ISP = 0x0017fd85`, odd, byte-decoded from `coherent-dump`): `SR=0x2700`,
`PC=0x4080315c` (a *second*, different faulting PC — see point 2), fmt/vec
`0x7008` (format 7, vector 2), `fault_addr=0x5000e000`, `SSW=0x0505`. Per
`cpu040/src/main/scala/m68k040/exception/ExceptionUnit.scala:1335-1351`, SSW
bit 10 (`0x0400`) is `atcBit`, driven by `entryFaultAtc` — "1 for an
MMU/ATC-detected translation fault, 0 for a plain physical bus error
(SLVERR/DECERR with zero MMU involvement)" (that file's own comment).
`0x0505 & 0x0400 = 0x0400` → **ATC = 1**. So cpu040's own RTL classifies
this fault as an MMU/ATC translation fault, not a bus response error — the
2026-05-06 legacy-core diagnosis (`docs/diag-bus-fault-51001c00.md`, which
found the xbar returns clean OKAY+0 for this exact address and concluded
"no sim repro") and the "misaligned-LONG load reconstruction in the LSU"
hypothesis in `diag-buserror-frame-format.md` are **both narrowed out** as
the mechanism for cpu040: this is not a wrong bus response and not a
misaligned-load data-integrity bug, it's a real MMU walk failing.

Live regs at the very first vector-2 halt (`exc_pc=0x408046aa`, the same
VIA1-alias-probe PC as the 2026-05-06/2026-08-26 table above): `TC =
0x0000c000` (bit 15 `E`=1 → **MMU translation is ON**), `ITT0 = 0xf900c060`,
`ITT1 = 0x807fc040`, `DTT0 = 0xf900c060`, `DTT1 = 0x807fc040`, `SRP =
0x03fffa00`. Decoding the TTRs (base byte / don't-care-mask byte):
`DTT0`/`ITT0` = base `0xf9`, mask `0x00` → matches only `0xF9000000`-
`0xF9FFFFFF` (DAFB framebuffer). `DTT1`/`ITT1` = base `0x80`, mask `0x7f` →
matches only addresses with the top bit set, `0x80000000`-`0xFFFFFFFF`.
**Neither TTR covers `0x50xxxxxx`/`0x51xxxxxx` (the Q700 I/O window).** This
contradicts the legacy-core capture at the *same* ROM PC, which reported
`TC.E=0` (MMU fully off) at this point — a genuine, confirmed
core-to-core divergence: cpu040 reaches this exact VIA1 probe with the MMU
already enabled and only DAFB/high-half TTRs installed, where legacy
reaches it pre-MMU-enable. `docs/BUG_wrong_video_driver_rbv_vs_dafb.md`
records a MAME trace showing `DTT1 = 0x500FC040` (base `0x50` mask `0x0f`,
covering `0x50000000`-`0x5FFFFFFF`, i.e. exactly the missing I/O coverage)
already loaded "well before DAFB init" — so on real hardware/MAME the I/O
TTR is expected to be live by this point. With no TTR match and paging on,
the access falls through to a real page-table walk via `SRP=0x03fffa00`,
which evidently does not have a valid mapping for the I/O region either,
producing the ATC=1 access fault. **Open question, not yet resolved**: is
cpu040 reaching this probe too early (before the ROM's DTT1-install code
has run — an instruction-count/timing divergence), or via a different
control-flow path than legacy/MAME take (e.g. re-entered after the
Outcome-A DIVU-by-zero excursion, on a branch that assumes DTT1 is already
set up)? Determining which needs a instruction-level trace of *which* ROM
code sets DTT1 and whether cpu040 executes it before or after this probe —
not chased further this session.

**2. It is a genuine storm, but a small deterministic 2-PC cycle, not one
repeating instruction.** `continue`-ing past the first vector-2 halt lands
on a *second*, different vector-2 halt almost immediately, and the pattern
repeats indefinitely. `exc-ring` after 4 more `continue`s:

```
exc[24] vec=0x02 pc=0x4080315c fa=0x5000e000 handler=0x40846a80
exc[23] vec=0x02 pc=0x408046aa fa=0x51001c00 handler=0x40846a80
exc[22] vec=0x02 pc=0x4080315c fa=0x5000e000 handler=0x40846a80
exc[21] vec=0x02 pc=0x408046aa fa=0x51001c00 handler=0x40846a80
exc[20] vec=0x02 pc=0x4080315c fa=0x5000e000 handler=0x40846a80
exc[19] vec=0x02 pc=0x408046aa fa=0x51001c00 handler=0x40846a80
```

Both faulting addresses decode to real, distinct Q700 I/O devices per
`rtl/soc/peripheral_bus.v`'s slot decode: `0x51001c00` → VIA1 (the
already-known alias probe), `0x5000e000` → mac-offset `0x00E000`, which
`peripheral_bus.v:475-476` routes to `SLOT_ORWELL` ("Orwell controls", a
second, distinct Q700 ASIC probe). Both share exception handler entry
`0x40846a80` and neither ever resolves — the handler evidently returns (or
retries) straight back into whichever ROM caller made the failing probe,
which immediately retries the other one, forever. This is a genuine,
non-recoverable fault storm, just not literally "the same instruction
looping" — it's two I/O hardware-detection probes ping-ponging, both
unreachable for the same underlying reason (point 1).

**Tooling/board-state note**: before this capture, five consecutive fresh
`reset`s (with and without `reset-halt-exc` armed) all produced a
`DebugHaltReasonCode.FATAL` halt (`RobPlugin.scala`'s sticky `coreHalted`,
fed by `dc.diagFault || exc.fsXlateFault || arbWedge || rvHalt` in
`FullCoreSynth.scala:408`) with `PC=0`, all architectural registers reading
0, and `inst-count=0` — i.e. zero retirement, consistent with either the
D-side merge arbiter's real hardware bounded-grant watchdog (`arbWedge`,
`AxiDMerge.scala`'s `wdog`/`wedge`, NOT simulation-only — a genuine
`grantTimeout`-cycle counter) or `ResetVectorFsm`'s own `haltPulse` (a
non-OKAY response to the CPU's very first read, the 16-byte reset-vector
fetch at physical `0x0`, per `ResetVectorFsm.scala:96-106`) firing before
a single instruction ever retired. `dump-mem 0x40800000 1` still read the
correct ROM shadow (`0x420dbff3`) throughout, so this was not a DRAM/
ROM-shadow problem. A full `load-bit` reprogram (same bitstream, forcing a
fresh MIG calibration) immediately cleared it — the very next `reset`
booted normally and reached this section's Outcome-B capture. **Not
chased further and not attributed to a specific one of the three
`coreHaltedIn` producers** — flagged here because it fully blocked
progress for ~5 reset cycles and is worth a fresh pair of eyes if it
recurs, but it did not reproduce after the reprogram and this session did
not have budget to pursue it as a fifth cataloged outcome.

#### Outcome C — deepest boot: healthy Mac OS, then a permanent VIA2 hang

The furthest the machine got. `VBR = 0`, `SR = 0x2200` (IPL 2), `TC = 0xc000`
(MMU on), `CACR = 0x80008000` (both caches on), `A5 = 0x03feaa00`, A-line
traps dispatching correctly from many distinct ROM PCs. It then spins
forever in RAM at `0x0000a8c0` on a two-phase **VIA2 `vBufB` bit 6**
handshake that our RTL can never satisfy, because
`rtl/soc/fpga_top_peripherals.vh:794` ties `via2_pb_in` to the constant
`8'hCF`.

**Tracked separately in `docs/BUG_via2_pb6_constant_boot_hang.md`** — that is
the crisp, fully-measured new bug out of this session and the recommended
next fix.

### Recommended next step

Fix outcome C first (`BUG_via2_pb6_constant_boot_hang.md`): it is the
deepest boot, the bug is precisely localised to one line, and it has a
proven in-repo precedent (the `via2_pa_in` fix directly above it in the same
file).

Outcome B (**updated 2026-08-27**, superseding the sim-repro recommendation
below): a live `reset-halt-exc 2` capture now has ATC=1 SSW proof this is a
genuine MMU/ATC translation fault, not a bus-protocol error — the
2026-05-06 xbar/sim analysis in `diag-bus-fault-51001c00.md` and the
misaligned-LSU-load hypothesis in `diag-buserror-frame-format.md` are ruled
out as cpu040's mechanism (see the 2026-08-27 update above). The next step
is narrowing *why* `DTT1` doesn't yet cover `0x50xxxxxx` when cpu040
reaches the VIA1-alias probe (`0x408046aa`) — either a boot-order/timing
divergence from legacy/MAME (this probe runs before the ROM's DTT1-install
code on cpu040 but not on them) or a real DTT1-programming bug. That needs
an instruction-level trace of the DTT1-installing `MOVEC` (which ROM PC,
and whether it precedes or follows `0x408046aa` on cpu040 vs. the MAME
trace in `docs/BUG_wrong_video_driver_rbv_vs_dafb.md`) — not yet done, no
RTL fix attempted.

##### 2026-08-27, same-day follow-up: DTT1-ordering test attempted, inconclusive; MOVEC serialization independently confirmed clean; ROM PC located; connects to a separate, larger finding

Three concrete sub-results from a same-day follow-up pass:

1. **The specific ROM DTT0/DTT1-install site is located.** A generic,
   multi-caller MMU-setup subroutine at `0x40803f30`-`0x40803f86` installs
   ITT0/DTT0/ITT1/DTT1/SRP/TC from a caller-supplied parameter block (does
   not by itself prove which value it installs when). A SECOND, specific
   site with literal constants exists at `0x40804072`-`0x40804082`:
   `movel #0x807FC040,D0 / movec D0,DTT0` then
   `movel #0x500FC040,D0 / movec D0,DTT1` — the second constant
   (`0x500FC040`, base `0x50` mask `0x0F`, covering `0x50000000`-
   `0x5FFFFFFF`) is exactly the I/O-covering value the MAME reference trace
   in `docs/BUG_wrong_video_driver_rbv_vs_dafb.md` shows already loaded by
   the equivalent point on a healthy boot. This confirms `0x40804082` is
   the exact `MOVEC` this open question is about.
2. **MOVEC-to-DTT1 serialization independently audited and confirmed
   clean** (separate RTL trace, not board work): the ROB/IQ/decode-rename
   skid buffers flush for the whole sysOp episode the moment a `MOVEC`
   reaches the ROB head, any in-flight DTLB/ITLB walker miss is poisoned
   rather than written back, and fetch resumes from the post-`MOVEC` PC
   only after the new register value is already committed
   (`cpu040/src/main/scala/m68k040/cache/DtlbPlugin.scala:116-121,162-164`,
   `cpu040/src/main/scala/m68k040/exception/ExceptionUnit.scala:1831-1913,
   2162-2204,2401-2405`, `cpu040/src/main/scala/m68k040/rob/RobPlugin.scala:
   952,965,989-991,2443-2451`, `cpu040/src/main/scala/m68k040/top/
   FullCoreSynth.scala:81-87,354,364`,
   `cpu040/src/main/scala/m68k040/mmu/MmuControl.scala:154-162`). **No RTL
   path exists for a stale `DTT1` read to survive to commit.** So IF
   `DTT1` genuinely isn't programmed yet at `0x408046aa`, it is not a
   pipeline/serialization hazard — it is that `0x40804082` genuinely
   hasn't executed yet in program order on this boot path, which narrows
   the open question to pure control-flow/timing, not an MMU-write
   correctness bug.
3. **A direct double-breakpoint ordering test (arm slot 0 at
   `0x408046aa`, slot 1 at `0x40804082`, both before a single `reset
   release`, to see which fires first) was attempted and is
   INCONCLUSIVE** — invalidated by a genuine tooling bug found in the
   process (documented in full below), not by anything about the CPU.
   **Not retried this session** after time was redirected to the
   higher-priority finding in point 4.
4. **A separate, same-day, live-hardware finding
   (`docs/BUG_calibration_word_misplaced_0d00.md` Part 6) offers a
   plausible unifying explanation that doesn't require either a DTT1-
   programming bug or a mysterious boot-order divergence**: cpu040 was
   proven, via direct `pc-trace` retirement history, to run at least one
   early ROM POST busy-wait loop fast enough to exhaust its own
   fixed-iteration-count safety cap before a real-time VIA1 hardware timer
   it depends on ever fires — a "cpu040 is simply faster than whatever
   reference speed this ROM code assumes" class of issue, not an
   interrupt-controller or MMU RTL bug. If other early-boot ROM code
   between the calibration routine (`0x40800800`) and the DTT1-install
   site (`0x40804082`) has SIMILAR timing dependencies (e.g. a delay loop
   gated on real hardware state that cpu040 also blows through
   unexpectedly fast, or reads a corrupted calibration value downstream
   and takes a different branch as a result), that could plausibly explain
   *why* `0x408046aa` is reached before `0x40804082` on cpu040 specifically
   — without needing a bug in the DTT1 `MOVEC` path itself. **This
   connection is plausible, not confirmed** — no trace was done linking
   the calibration routine's outcome to which branch the ROM takes before
   reaching either `0x408046aa` or `0x40804082`.

**Tooling bug found**: the interactive `break-pc <slot> <pc> [wait_ms]`
command (as opposed to the reset-integrated `reset-and-break-pc`) ends with
an unconditional `dbg_wr $::OFF_CONTROL 0x0` (`tools/jtag_repl.tcl:4663`).
`OFF_CONTROL` (offset `0x008`) is the SAME register that carries
`CTL_COLD_RESET_HOLD` (bit 4, `tools/jtag_repl.tcl:891`) — the bit `reset
hold` sets and `reset release` clears. So arming a SECOND breakpoint via
plain `break-pc <slot> <pc>` after a `reset hold` **silently clears the
hold as a side effect**, releasing the CPU mid-arming-sequence. This is
the exact "bare register write clobbers an unrelated control bit sharing
the same offset" footgun class this project's own comments elsewhere
already warn about for `OFF_HALT_CTL` (see `halt_exc_enable_sync`'s
comment in the same file). **Symptom this produces**: a `reset hold` +
`break-pc <slot0> <pcA>` + `break-pc <slot1> <pcB>` + `reset release`
sequence intended to arm two breakpoints before a single controlled release
instead releases early (after the FIRST `break-pc` call), making any
"which fires first" ordering conclusion from that specific sequence
unreliable. Confirmed via `CTL_COLD_RESET_HOLD = 0x10` at
`tools/jtag_repl.tcl:891` sharing `OFF_CONTROL = 0x008` with the
unconditional clear at `tools/jtag_repl.tcl:4663`. **Not fixed** (tooling,
out of this investigation's scope) — flagged for whoever next needs a
genuine multi-breakpoint pre-boot arm: use `reset hold` +
`break-pc <slot> <pc> <wait_ms=0>` calls is NOT currently safe for more
than one slot; a `reset-and-break-pc`-style primitive that arms N slots
inside ONE hold/release window (never calling the plain interactive
`break-pc` path in between) would be needed instead.

### Tooling notes from this session

* **`vio-reset-halt-exc` killed the REPL outright** (Vivado process gone,
  FIFO orphaned). Use `reset hold` + `halt-exc-mask` + `reset release`
  instead.
* **`tools/jt.sh` exists specifically to flock-serialize `/tmp/jtag_in`** and
  logs every command to `/tmp/jtag_cmd.log`. Writing to the FIFO directly
  bypasses that. Two REPL instances from different sessions sharing
  `/tmp/jtag_in` corrupted each other's commands here; worse, the second
  launcher did `rm -f /tmp/jtag_in; mkfifo /tmp/jtag_in`, which **orphaned
  the first REPL onto a deleted inode** so it could never receive another
  command (it was still holding the JTAG target). If the REPL stops
  responding, check `ls -l /proc/<pid>/fd/0` for `(deleted)`.
* A cold reset pulse **does** re-run the `boot_fsm` SD→DDR ROM copy
  (`unified_reset`, `jtag_repl.tcl`), so the ROM shadow is restored by
  `reset` — the earlier "debug reset doesn't reload the ROM shadow" concern
  did not reproduce here.

## One-line summary

`MOVEM.L (An)+, <reglist>` executed with `An` **not** longword-aligned
(`An & 3 != 0`) permanently deadlocks the core. The identical instruction at the
identical PC completes normally if `An` is 4-aligned.

## Proof — controlled single-variable experiment

Run on live hardware through `tools/jtag_repl.tcl`:

1. `reset hold` -> `break-pc 0 0x40815646` -> `reset release`
   (breakpoints survive CPU reset via the debug reset domain).
2. Breakpoint lands pre-effect: `reason=0x5`, `effective=1`, `PC=0x40815646`.
   The instruction there is `4CDF 0D36` = `MOVEM.L (SP)+,D1/D2/D4/D5/A0/A2/A3`.
3. `regs` reports `A7 = ISP = 0x0017FFBA` -> `0x17FFBA & 3 == 2`, so **every one
   of the 7 longword loads is a misaligned / split access**.
4. `continue` -> **PERMANENT DEADLOCK** (signature below).
5. Repeat 1-3, then `reg-set A7 0x0017FFB8` (4-aligned) and `continue`
   -> the **same MOVEM at the same PC executes and the CPU runs on**
   (commit-PC ring head advances, L2 counters advance).

Only the low 2 bits of the base address differ between the deadlock and the
non-deadlock case.

## Deadlock signature (all measured, not inferred)

* `pc-trace` ring head **frozen** -> zero macro retirement.
* `vio_l2c_stats` (hit/miss counters) **byte-identical frozen** -> the CPU issues
  **zero** bus transactions. (Video scan-out traffic bypasses L2, so a frozen L2
  counter is specifically a CPU-quiet indicator; `ddr_dbg_r_cnt` keeps racing.)
* L2 **MSHR occupancy = 0** -> nothing outstanding; the core is not waiting on
  memory or AXI. (The L2 is a **SoC** block, `rtl/soc/fpga_top_ddr.vh` under
  `L2C_ENABLE`, sitting between the CPU and DDR — not a cpu040-internal cache.)
* `live-arch force` architectural registers **byte-identical across minutes**.

**DO NOT trust `wedge-status` on this core.** `OFF_WEDGE0..3` are declared in
`cpu040/src/main/scala/m68k040/debug/DebugRegMap.scala:172-175` but
`DebugCtrlPlugin.scala` contains **zero** `WEDGE` references, and empirically the
command reads all-zero in *every* state — including while the CPU is
demonstrably executing at full speed. Its `lsu_state_name`/`dcache_state_name`
tables in `jtag_repl.tcl` (~3850-3870) decode an enum vocabulary belonging to the
**legacy Verilog `cpu/` core**, not cpu040. An early draft of this investigation
cited "ROB empty / LSU IDLE / D-cache IDLE" from it; that reading was
**withdrawn** — it is an unimplemented register returning 0, not a measurement.
This is a third tooling bug (see below).
* CPU is **not** halted (`effective=0`, `reason=0`, `DBG_CONTROL=0`).

It is a fully quiescent deadlock, not a livelock and not a bus hang.

## `halt` legitimately cannot land — this is correct behaviour, not a tool bug

`request_debug_halt` waits for STATUS bit 0 (effective halt), which the Stage-5
stop manager asserts only **after the current macro commits**. In this deadlock
no macro will ever commit, so `halt` can never land. The observed
`ERROR halt timeout ... halt requested but effective halt did not land` is
therefore a *correct report of a genuine hang*.

**The working way in** is a precise PC breakpoint (`break-pc`), which is a
registered frontend marker honoured pre-effect at the commit head. It fires
*before* the offending MOVEM executes, giving a clean effective halt with
coherent architectural state. Arm it with `reset hold` / `break-pc` /
`reset release` so there is no arming race.

## Ruled out

* **Memory / stack corruption** — ruled out. `dcache-op push` + `coherent-dump`
  at `A7` shows the exact values the MOVEM is about to pop are all sane, and the
  RTS return address (`0x4080C5E4`) is a valid ROM instruction. No RAM-test
  bit-pattern (`0x6DB6DB6D`) anywhere on the stack.
* **Waiting on a bus/AXI transaction** — ruled out by L2 MSHR occupancy = 0 and
  frozen L2 counters.
* **A debug-tooling artifact** — ruled out. Reproduces on a completely clean
  boot with all breakpoints disarmed and `DBG_CONTROL=0`.
* **The DDR4 MIG WPWS pulse-width timing violation** in this bitstream — not
  implicated: DDR reads work throughout (JTAG `dump-mem`, video scan-out), and
  the core is provably not waiting on memory.
* **The store direction** — `MOVEM.L <list>,-(An)` with a misaligned `An`
  appears to work: the matching prologue pushed these 7 longwords with SP going
  `0x17FFD6 -> 0x17FFBA`, both 2 mod 4. So the bug looks specific to the
  misaligned **LOAD** path. (Inferred from the stack layout, not directly
  single-stepped.)

## Reproduction record

Four independent boots. Both PCs that were ever observed wedged are
`MOVEM.L (SP)+` (opcode `0x4CDF`), and both had a 2-mod-4 stack pointer:

| PC | encoding | reglist | A7 at wedge |
|---|---|---|---|
| `0x4080c5b8` | `4CDF 0326` | D1/D2/D5/A0/A1 (5) | `0x0017FFC2` |
| `0x40815646` | `4CDF 0D36` | D1/D2/D4/D5/A0/A2/A3 (7) | `0x0017FFBA` |

A clean boot with zero debug interference deterministically wedges at
`0x40815646`.

## Prime suspects (recent LSU/split-load work on this branch)

Not yet confirmed by source analysis:

* `03b8ab0e` perf(lsu): *split loads pipeline through the aligned ring instead of
  draining it + a serial FSM* — directly restructures the misaligned/split LOAD
  path. Top suspect.
* `f81a47d4`, `fd445138`, `19afd142`, `1618fab7`, `f5f9fe13`, `435e9efb` — other
  recent LSU/D-cache changes on the same branch.

A source-level review of `03b8ab0e` found **no** unbounded wait or missing exit
condition by static reading — the serialization point is `alignedSendHeld`
(`LsEuPlugin.scala:813`), `alignedCanEnqSplit` (:827) needs only 2 free ring
slots, a slot-A fault retires slot B unsent via `alignedRspAbortsPair` (:1399),
and `alignedRspTerminal` (:1409) gates busy across both sub-accesses. Every
branch terminates. So the defect is **not** yet localised to a line; it needs the
simulation repro below to bisect.

Also checked and found clean: the MOVEM FSM itself gates only on
`queue.io.push.ready` and nothing from the LS EU/D-cache/ROB
(`DecodeStage.scala:2392-2409, 2664`), so a MOVEM-FSM-internal deadlock would
require `push.ready` stuck low forever. The two Aug-26 area folds (`2edd810e`
MicroOpQueue LUTRAM, `4698963d` IqContext split) show no static deadlock
mechanism and both ship always-on `GenerationFlags.simulation` shadow-structure
equivalence asserts.

## Verification gap — CONFIRMED

**No simulation test covers `MOVEM` with a non-longword-aligned base.** Verified:
every MOVEM case in `MovemDecodeSpec.scala` and `ExecuteLockStepSpec.scala`
(`movem-l-postinc-load`, `movem-l-control-sparse`, `movem-w-signext`,
`movem-l-single`, `movem-l-prologue-epilogue`, `movem-postinc-base-in-list`,
`movem-stall-resume`, `movem-ucode-s1`) uses a 4-byte-aligned literal base
(0x300c/0x3000/0x4000 family). `LsEuSplitRingSpec.scala` (new in `03b8ab0e`)
tests split loads issued **directly**, never through the MOVEM FSM's uop stream.
**MOVEM x misaligned-base x split-ring is untested.**

**Compounding process gap**: `make test-fast` is defined in `build.sbt:21-22` as
`testOnly * -- -l m68k040.SlowTest -l m68k040.VerilatorTest -l m68k040.BoardTest`
— it **excludes every `VerilatorTest`-tagged suite**, and the entire
`ExecuteLockStepSpec` (the real lock-step suite) is `VerilatorTest`-tagged. The
"329/329 test-fast" validation claimed by both Aug-26 area commits therefore does
not include lock-step execution at all.

A directed matrix (MOVEM load **and** store, every reglist size, base address
0/1/2/3 mod 4) should be added and run under a target that actually includes
`VerilatorTest`.

## Suggested next steps

1. Reproduce in simulation: add a lock-step case mirroring `movem-l-postinc-load`
   but with a base `== 2 (mod 4)`, e.g.
   `move.l #0x300e,%a0 ; movem.l (%a0)+,%d3/%d4/%d5` — exercising `03b8ab0e`'s
   split ring *through* the MOVEM FSM, which nothing currently does. Expect a
   hang.
2. Bisect the branch's LSU commits against that test, starting at `03b8ab0e`.
3. Add the misaligned-MOVEM matrix to the regression suite before re-closing, and
   run it under a target that includes `VerilatorTest` (not `test-fast`).
4. Separately: implement or remove `wedge-status` on core040, and feature-gate
   `icache-probe`. Both currently emit confident, wrong-looking output.

## Tooling bugs found while investigating (separate, low severity)

1. **`break-pc` parses its address operand as DECIMAL.** The dispatcher uses a
   bare `[expr {$sub}]` instead of `parse_num`, so `break-pc 40815646` silently
   armed `0x026ECC1E` and reported "DID NOT FIRE". This is exactly the
   silent-wrong-answer class the script's own header warns about and mandates
   `parse_num` for. Same issue for its `wait_ms`, the `break-pc <slot> <pc>`
   form, and `irq-inject`. Workaround: always write `0x`-prefixed addresses.
2. **`icache-probe` / `icache-lookup` are not feature-gated on core040.**
   `icache_probe` does not exist in `DBG_FEATURE_NAMES_CORE040` (that list ends
   at bit 24 `branch_ring`), so the probe registers are tied off and the command
   *always* fails with "DID NOT COMPLETE ... the probe needs an idle I-cache
   cycle with no fetch outstanding". That message actively invites the wrong
   conclusion ("instruction fetch is spinning") — it did, mid-investigation, and
   the error persists even at a confirmed effective halt. It should refuse with
   "not supported by this bitstream" the way `mon-sense` does.
3. **`wedge-status` is unimplemented on core040 but prints a confident decode.**
   `OFF_WEDGE0..3` exist in `DebugRegMap.scala:172-175`; `DebugCtrlPlugin.scala`
   has no `WEDGE` read case, so all four words read 0 forever. `wedge_status_line`
   then decodes those zeros into `rob_hd=0 rob_pc=0x00000000 lsu=0:IDLE
   dcache=0:IDLE` — which reads exactly like a real, meaningful "pipeline is
   completely idle" measurement. It is not. Worse, its state-name tables belong to
   the legacy Verilog `cpu/` core's FSMs, not cpu040's. This actively produced a
   wrong intermediate conclusion during this investigation. It should either be
   implemented or made to refuse on this core.
4. **Observed once, did not reproduce**: after an exception storm, the
   `halt-exc-mask` lanes read back `0xdb6d0000` (lanes 0 and 3) and `0x01060000`
   (lane 1) without ever having been written this session. Reads were stable and
   deterministic, so it was stored state, not read corruption. Cleared cleanly
   and did **not** re-appear across a subsequent clean boot. Flagged for
   awareness only; a debug-CSR write-aliasing bug would be serious if confirmed.
