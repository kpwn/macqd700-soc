# >>> READ PART 5 FIRST (2026-09-03). THIS DOCUMENT'S TITLE AND ITS ENTIRE
# >>> "MISSING RING-CLOSURE / WRAPAROUND SENTINEL / EMPTY TABLE" FRAMING ARE
# >>> WRONG AND ARE RETRACTED. The structure is the Slot Manager's Slot
# >>> Resource Table; the loop at `0x4080677c` is a **predecessor search**,
# >>> not a ring walk; there is no wraparound sentinel and none is missing;
# >>> a `next` link of `0x00000000` in the last block is **normal**; and
# >>> cpu040's table is **populated (8 records), not empty**. The real defect
# >>> is upstream: cpu040 enumerates ~8 of the 76 ROM-resident sResources.
# >>> Full reference derivation in `BUG_calibration_word_misplaced_0d00.md`
# >>> Part 114. Everything below Part 4 predates that and should be read as
# >>> historical. The mechanical description of the *hang itself* (a walk
# >>> that never terminates and runs off into unmapped space) is still
# >>> accurate; only its cause and its structural interpretation change.

# FIXED, hardware-confirmed 2026-08-27 (Part 4): a static ROM patch (`zonewalk-empty-table-fix`, not an RTL change), verified on real cpu040 silicon -- boot now proceeds past this hang entirely. NOT a self-resolving side effect of the calibration-loop fix as Part 3 had hoped -- needed its own dedicated fix. See Part 4 for the fix, verification chain, and deployment status (not yet merged to any mainline branch). Root cause: a live, repeatedly-populated ROM record/table walk had no NULL check and free-ran into open bus, forever, because cpu040 reached it before the table had ever received its first real entry (real 68040 timing never saw it empty, per Part 3's MAME reference proof)

**Status**: Root-caused with a byte-exact live trace on real hardware,
now with MAME reference confirmation (Part 3, 2026-08-27) that the exact
same walk is called repeatedly and closes correctly on real 68040 timing
— cpu040 hits it with a genuinely empty table (wraparound sentinel ==
base, zero entries ever added) at the exact PC real hardware always sees
non-empty. A third independent reproduction (Part 2, 2026-08-27) found
the walked structure is very likely NOT a classic Memory Manager heap
zone (see Part 2) — the mechanical bug (no NULL/wraparound check, walks
into open bus, loops forever) is unchanged, but "heap-zone-block" in the
title/body below should be read as "the structure this loop walks", not
confirmed heap semantics. First flagged (not investigated) in
`docs/BUG_calibration_word_misplaced_0d00.md` Part 3 ("Outcome D") on
2026-08-26. This document is the dedicated follow-up investigation, per
that document's own "recommend a dedicated follow-up investigation" note.
**No RTL fix attempted — leading hypothesis is this resolves as a side
effect of fixing the calibration-loop interrupt-recognition bug, not a
separately-needed fix; see Part 3.**

## Bottom line

The Q700 ROM's Memory Manager walks a heap zone's block list starting from
low-memory global `GLOBAL[0x0D24]` (a real, correctly-initialized zone-base
pointer — confirmed sane on hardware, see below). The walking loop expects
a **circular** list: it keeps following each block's "next" field and
stops only when that chase leads back to the zone's own base address
(a wraparound-to-start check). It has **no defensive check for a `NULL`
("next" = 0) terminator.**

On this hardware/ROM-image/cpu040 combination, the zone's first (and only
reachable) block has its "next" field set to `0x00000000` instead of a
value that would satisfy the wraparound check. The loop faithfully follows
that `0` "next" pointer, applies its constant 192-byte (`0xC0`) block-size
stride, and walks off into low-memory ROM globals space (`0x000000C0`
onward) and then into genuinely unmapped, open-bus address space
(`0xD25...`, `0xB6DB...`), where it enters a **stable, deterministic
3-address cycle that provably never reaches the wraparound condition** —
an infinite loop, hardware-confirmed to survive 40+ seconds of continuous
polling and to reproduce identically after a full FPGA reprogram.

**Every hop in the observed chain was independently verified byte-for-byte
against a direct JTAG memory read.** The CPU's misaligned-load
reconstruction (the LSU) is doing exactly the right arithmetic at every
step — this is **not** a repeat of the split-ring/misaligned-load bug class
that dominated the rest of this session's findings. The bug is a **data**
problem (a missing/incorrect ring-closure link in the zone's own block
list), not a CPU logic-error.

## Reproduction

Board: `/home/qwertyoruiop/macqd700-soc-worktrees/build-2026-08-26-fixes`,
bitstream build_id `0x4339FA79`, `CPU=m68k040`, ROM `420dbff3`.

`reset-and-break-pc 0x4080677c 0 12000` (arm a precise PC breakpoint at the
loop's compare instruction, then cold-boot into it) landed cleanly on this
boot attempt — one of at least two independent hits across this and the
prior session (`docs/BUG_calibration_word_misplaced_0d00.md` reports its
own, earlier, independent capture with a different, garbage-looking `A1`
value at the moment it happened to sample). Static decode of the loop
(`m68k-linux-gnu-objdump`, confirmed against live disassembly earlier this
session):

```
40806774: moveal  0xd24,%a1        ; a1 = GLOBAL[0x0D24]           (once, entry)
40806778: lea     %a1@(192),%a1    ; a1 += 0xC0                     <- loop top
4080677c: cmpal   %a1@(2),%a2      ; compare LONG[a1+2] to a2 (wraparound sentinel)
40806780: beqs    0x40806788       ; done if equal — list closed the ring
40806782: moveal  %a1@(2),%a1      ; a1 = LONG[a1+2]  ("next" link, OVERWRITES a1)
40806786: bras    0x40806778       ; loop
```

192 bytes is a plausible Memory Manager zone-block header stride; `A1@(2)`
(offset +2 from a longword-aligned block base) matches a classic 68k
struct layout where a leading size/flags word precedes a 4-byte pointer
field — `A1@(2)` is therefore a **misaligned** (2-mod-4) longword access on
every iteration, which is why this was initially suspected to be another
instance of this session's dominant misaligned-load bug family. It is not
— see "Ruled out" below.

## Live capture at the breakpoint (first hit, pre-effect)

```
D0=0x00000000  D1=0x0000a06e  D2=0x4080006e  D3=0x00000060
D4=0x00000040  D5=0x0000c000  D6=0x00000000  D7=0x00040010
A0=0x000087e0  A1=0x000088a0  A2=0x000087e0  A3=0x000088b0
A4=0x40809ae6  A5=0x03feaa00  A6=0x0017ff12  A7=0x0017fe9b
SR=0x00002704  VBR=0x00000000  TC=0x0000c000  ISP=0x0017fe9b
SRP=0x03feea00  ITT0/1=0  DTT0/1=0
```

This is a **healthy-looking** boot state — `VBR=0`, `TC.E=1` (MMU on),
`A5=0x03feaa00` — matching the register signature already on record for
Outcome C (the deepest, healthiest boot) in
`docs/BUG_movem_misaligned_postinc_deadlock.md`. Outcome D is reached from
otherwise-normal boot progress, not from a corrupted/degenerate state.

`A0 = A2 = 0x000087e0` — the zone base and the loop's wraparound sentinel
agree, as expected. `A1 = 0x000088a0 = 0x000087e0 + 0xC0` — the very first
block after the zone header, exactly where the `lea` puts it. Nothing here
is wrong yet.

## The chain, traced hop-by-hop and independently verified

Repeated `continue` (each one re-hits the same breakpoint one loop
iteration later) shows `A1` at the breakpoint settle into a **perfectly
periodic 3-value cycle**, observed for 5 full repeats (15 consecutive
hits) with zero drift:

```
A1 = 0xb6db6e76
A1 = 0x000000c0
A1 = 0xd2524140
A1 = 0xb6db6e76   (repeats)
```

Each transition is `A1_next = LONG[A1_prev + 2] + 0xC0` (the loop body:
fetch "next" from `@(2)`, then add the 192-byte stride at the next
iteration's `lea`). Reconstructing what `LONG[A1_prev + 2]` must have been
at each step, and cross-checking against a **direct JTAG `coherent-dump`
read of the same address** (which goes over a separate debug AXI master,
bypassing the CPU's own LSU/D-cache path — an independent measurement):

| `A1_prev` | fetch address (`+2`) | value needed to explain the chain | direct JTAG read | match? |
|---|---|---|---|---|
| `0x000088a0` (first real block) | `0x000088a2` | `0x00000000` | bytes at `0x88A0`=`ffff0000`, `0x88A4`=`00000f27` → misaligned LONG@`0x88A2` = `0x00000000` | **exact match** |
| `0x000000c0` | `0x000000c2` | `0xd2524080` | bytes at `0xC0`=`4088d252`, `0xC4`=`40802704` → misaligned LONG@`0xC2` = `0xd2524080` | **exact match** |
| `0xd2524140` | `0xd2524142` | `0xb6db6db6` | address is far outside any mapped SoC region (RAM/ROM/IO); not independently re-read, but `0xb6db...` repeating-nibble pattern matches the DDR PRBS/training-pattern open-bus residue already documented in `BUG_calibration_word_misplaced_0d00.md`'s "untouched cell" caveat | consistent, not independently re-verified |

**The CPU's misaligned-longword reconstruction is byte-exact at every
directly-checked step.** This rules out an LSU/split-load data-integrity
bug as the cause of THIS hang (see "Ruled out").

## Root cause, precisely localised

1. `GLOBAL[0x0D24] = 0x000087E0` is correct and sane (confirmed live,
   matches the zone base / wraparound sentinel `A2` exactly). This global
   is not corrupted.
2. The zone's first block, at `0x000087E0 + 0xC0 = 0x000088A0`, has its
   "next" field (`@(2)`, i.e. address `0x88A2`) genuinely containing
   `0x00000000` — **verified via a direct, LSU-independent JTAG memory
   read**, not inferred.
3. The loop has **no check for `next == 0`**. It only checks for
   "wrapped back to the zone base" (`cmpal %a1@(2),%a2` — compare the
   fetched next-field to `A2`, the base). A `next` of `0` is neither a
   valid link nor the wraparound sentinel, so the loop treats it as an
   ordinary (if unusual) block pointer and keeps going: `a1 := 0;
   a1 += 0xC0` → `a1 = 0x000000C0`.
4. `0x000000C0` lands in **low-memory ROM/OS globals space** (the same
   general neighborhood as the `0x0D00`-`0x0D24` cluster investigated in
   `BUG_calibration_word_misplaced_0d00.md`), not heap-block storage. The
   loop reads whatever unrelated data happens to live at `0xC2`
   (`0xd2524080`) and misinterprets it as a "next" pointer.
5. That garbage pointer (`0xd2524140` after the stride) is a wild address
   with no backing SoC region. The read at `+2` from it lands on open bus,
   which this SoC's fall-through decode answers with a fixed residual
   pattern (`0xb6db...`) rather than faulting — matching the project's
   established "OKAY + fixed/zero data" open-bus convention
   (`rtl/soc/axil_null_slave.v`, `docs/diag-bus-fault-51001c00.md`) and
   the DDR PRBS-residue observation from `BUG_calibration_word_misplaced_0d00.md`.
6. That residue happens to close a **stable 3-hop cycle** back through
   `0x000000C0` — the loop can never reach the `A2` wraparound value from
   inside this cycle, so it runs forever. No exception ever fires (every
   address in the cycle answers with OKAY, never SLVERR/DECERR), so
   nothing halts the core on its own — this is a genuine silent, permanent
   application-level hang, invisible to any bus-error/exception-based
   detection.

**The proximate architectural anomaly is item 2/3**: a well-formed classic
Mac OS heap zone's blocks form a closed ring (the last block's "next"
points back to the zone header/base, which is exactly what this loop's own
termination check expects), not a NULL-terminated list. A `next` of `0` at
the first block means **this zone was never correctly closed into a ring**
— something upstream, during zone creation/initialization, left the block
list open instead of self-linking or linking back to the header. This
document does not trace that far upstream (open item below); it stops at
the first hardware-verified anomaly.

## Ruled out

* **LSU misaligned-load reconstruction bug** (this session's dominant bug
  family — the MOVEM split-ring deadlock this repo fixed earlier the same
  day). Explicitly checked and ruled out for this hang: both real (mapped)
  hops in the chain (`0x88A2`, `0xC2`) were independently re-read via a
  JTAG path that bypasses the CPU's LSU entirely, and the CPU's own
  misaligned-longword value at each hop matched the direct read
  byte-for-byte. The LSU is doing the right arithmetic here.
* **Stale/leftover DRAM from a prior debug session.** `BUG_calibration_word_misplaced_0d00.md`
  already ruled this out for its own (earlier, independent) Outcome-D
  capture by reproducing after a full `load-bit` FPGA reprogram. This
  session's capture is a second, independent reproduction (different
  session, board freshly reprogrammed earlier the same session for an
  unrelated reason — see the Outcome-B tooling note in
  `BUG_movem_misaligned_postinc_deadlock.md`), consistent with a
  deterministic, reproducible condition rather than session-local RAM
  residue.
* **A bus/exception storm** (the Outcome-B mechanism). Confirmed
  structurally distinct: zero exceptions fire anywhere in this chain
  (every address in the 3-cycle answers OKAY); this is a pure,
  silent application-level infinite loop, not a fault storm.

## Open, not investigated this session

* **Why is the zone never closed into a ring in the first place?** This
  needs tracing the zone-creation/initialization code (likely near where
  `GLOBAL[0x0D24]` itself is first written, and/or wherever the first
  block's header is built) to see whether it ever attempts to set the
  first block's "next" field to a ring-closing value at all, and if so,
  why that write didn't land. Given `0x0D24` sits in the same
  under-investigated low-memory-globals neighborhood as `0x0D00`/`0x0D02`
  (both confirmed in `BUG_calibration_word_misplaced_0d00.md` to be
  written via computed/indexed addressing invisible to a static literal-
  address grep), the same "correct value, wrong computed address" bug
  family flagged there is a plausible, but **unconfirmed**, candidate
  cause here too — this document does not claim that connection as proven,
  only as worth checking first.
* Whether this zone is *supposed to* have more than one real block at
  this point in boot (i.e. whether "only one block, immediately
  terminating" is itself already a symptom of something failing upstream,
  as opposed to a legitimately tiny/fresh zone that the ROM's own code
  should still be able to close into a 1-block ring but fails to).
* Whether the ROM's own loop (missing the `next == 0` defensive check) is
  simply relying on an invariant classic Mac OS always upholds on real
  hardware (every zone is always ring-closed, so a bare wraparound check
  is normally sufficient and this is not "a ROM bug" so much as "the ROM
  correctly assumes properly-initialized memory, which cpu040 boot
  doesn't yet provide") — this reads as the most likely framing given the
  evidence, but is not independently confirmed against a real-hardware or
  MAME reference trace of the same zone-creation code path.

## Cross-references

* `docs/BUG_calibration_word_misplaced_0d00.md` — first flagged this hang
  (Part 3), same general low-memory-globals neighborhood as its own
  primary `0x0D00`/`0x0D02` investigation; its "recommend a dedicated
  follow-up investigation" is what this document answers.
* `docs/BUG_movem_misaligned_postinc_deadlock.md` — Outcome B (the
  bus/MMU-fault storm) is a structurally different hang reached from the
  same general boot neighborhood; see that document's 2026-08-27 update
  for the sibling investigation done in the same session as this one.

## Board state left behind

Board left **running** (not halted): `break-pc off` then `continue` after
the final capture. The core is not stuck at the debug layer — it is
executing the confirmed silent infinite loop at `0x40806778`-`0x4080677c`
forever, exactly as this document describes, which is the expected/correct
state to leave it in per this session's "don't leave it needlessly halted"
convention.

## Part 2 (2026-08-27, later session): the "heap zone" framing is probably
## wrong — this looks like a fixed-size device/dispatch record table, not
## variable Memory Manager blocks; the "off-by-2 misdirected write" theory
## is NOT supported by the data

Re-hit this exact hang naturally during the ILA-capture investigation in
`BUG_calibration_word_misplaced_0d00.md` Part 9 (same `pc_live=0x4080677c`,
confirmed byte-for-byte against this doc's own documented breakpoint PC —
a third independent reproduction, on yet another bitstream build
(`build_id=0x77C93EAD`), months of unrelated core changes later). Took the
opportunity to `halt`/`coherent-dump` a much wider range
(`0x87E0`-`0x88E4`, 144 longwords) than the original investigation's
narrow hop-by-hop reads, to test the "open, not investigated" question
about whether a ring-closing write landed at the wrong (aligned) address
— mirroring the `0x0D00`/`0x0D02` bug class.

**That specific hypothesis is NOT supported.** No `0x000087E0` (the zone
base / would-be ring-closing value) appears anywhere in the dumped range,
aligned or not. Instead, two new observations:

1. The `0xC0`-byte "zone header" region (`0x87E0`-`0x889C`) is **not**
   a single header structure — it's **exactly 8 repeating 24-byte
   records** (`192 / 24 = 8`, clean), each shaped
   `{0xff010000, 0x00000000, 0x00010000, 0x00000000, 0x408f5aXX,
   0x00000000}`, where the 5th longword's low bits vary
   (`0x408f5af6`, `0x408f5ac4`, `0x408f5a90`, `0x408f5a90`, `0x408f5ac4`,
   `0x408f5af6`, `0x408f5b16`, ...) — code pointers into a narrow
   ~0x86-byte ROM range (`0x408f5a90`-`0x408f5b16`). This is much more
   consistent with a **fixed-size dispatch/device-record table**
   (something like a Slot Manager sResource list, unit/driver table, or
   similar early-boot enumeration structure) than a classic Mac OS
   variable-length Memory Manager heap zone. `GLOBAL[0x0D24]` is
   plausibly this table's base pointer, not `ApplZone`/`SysZone` (neither
   of which lives at `0x0D24` in the real Inside Macintosh low-memory
   map anyway — that mapping was always an assumption, never confirmed
   against ROM symbols).
2. The bytes at `0x88A0` (the "first block" the walk chokes on) start
   `0xFFFF` — a classic uninitialized/free-fill bit pattern, not a
   plausible legitimate header value that just landed 2 bytes off. This
   looks like memory that was simply **never written by anything**, not
   memory that received a correct value at the wrong address.

**Revised leading hypothesis**: this fixed 8-entry table is populated at
ROM init, and a *further* stage — real device/driver registration that
would append real, ring-closed entries starting at `0x88A0` — either runs
too late, gets skipped, or never executes on cpu040, echoing this whole
investigation's recurring theme (`BUG_calibration_word_misplaced_0d00.md`
Parts 5-8) of cpu040 reaching a check/walk before some slower upstream
step has completed. Static disassembly around the `GLOBAL[0x0D24]` write
site (`40806774: moveal 0xd24,%a1` is the walk's own read; the write is
at `408060b2: movel %a1,0xd24`, part of a larger A-trap-heavy
(`0xa06e`, selector-dispatched via D0 with values 19/40/44/47 seen
nearby) routine) is consistent with initializing this fixed table but
was not fully traced forward to find where/whether real entries ever get
appended after it — that's the concrete next step, most likely via a
MAME reference trace of the same PC region (mirroring the successful
Part 7 technique from the calibration-word investigation) to see what
real 68040 timing does differently here, rather than further blind static
disassembly of unfamiliar ROM-internal trap dispatch code.

### Status: reframed, still not root-caused past this point

The "no NULL check" mechanical description (Bottom line, above) remains
accurate and unchanged. What's revised is the *interpretation* of what
kind of structure is being walked and why it's empty — not a classic
heap zone with a missing ring-closure write, but more likely a
fixed-capacity table whose "extension" region was simply never populated
in cpu040's boot, for reasons not yet traced. The "off-by-2 misdirected
write" theory this document's "Open" section speculatively raised is now
actively disfavored by real data, not just unconfirmed.

## Part 3 (2026-08-27, same session): MAME reference proves this is a real,
## repeatedly-called, always-correctly-closing routine on real 68040 timing
## — cpu040 hits it BEFORE any entry has ever been added, matching Part
## 7/8's "cpu040 races ahead of ROM-assumed background timing" theme

Following Part 2's reframing, cross-referenced the exact same loop
(`0x40806774`/`0x4080677c`/`0x40806788`) against the project's
verified-good MAME Quadra 700 reference (`~/mame_q700_good/run.sh`,
boots this exact ROM to Finder), using the same `-debug -debuglog
-debugscript` technique that settled the calibration-word bug's root
cause in `BUG_calibration_word_misplaced_0d00.md` Part 7.

**Result: the loop is called at least 10 times across a 45-second boot
to Finder, and closes cleanly (reaches `0x40806788`) every single
time**, most after only 1-9 real hops. Two things fall straight out of
the trace log:

```
ZONEWALK-ENTRY pc=40806774 base=00004FE0 cyc=120114629
ZONEWALK-COMPARE ... a1=000050A0 a2=00005730 next=000050B0 ...
  ... (7 more hops, each landing on a real, valid "next" link) ...
ZONEWALK-COMPARE ... a1=00005720 a2=00005730 next=00005730 ...
ZONEWALK-EXIT-RING-CLOSED pc=40806788 a1=00005720 cyc=120114953
ZONEWALK-ENTRY pc=40806774 base=00004FE0 cyc=120133218
  ... (a2=00005660 this time — SHRUNK from 00005730) ...
```

1. **`a2` (the wraparound-sentinel / list-tail target) shrinks on every
   successive call** (`0x5730 → 0x5660 → ... → 0x5250 → 0x5180 →
   0x50B0`), while `base` stays fixed at `0x4FE0`. This is a real,
   live, growing/shrinking ring structure being repeatedly walked and
   re-verified over the course of boot — consistent with genuine
   ongoing table/resource population, not a one-shot static init.
2. **On every single MAME call, `a2 != base`** — by the time this
   routine is ever reached on real 68040 timing, at least one real
   entry has already been linked in. Compare this to cpu040's own
   captured state at the exact same breakpoint (see "Live capture at
   the breakpoint" above): `A0 = A2 = 0x000087E0` — **on cpu040, the
   sentinel exactly equals the base**, meaning **zero entries had ever
   been added** by the time cpu040 reached this code. cpu040 doesn't
   have a broken ring-closure algorithm — it has an **empty table that
   real hardware never sees empty**, because real hardware always gets
   here after at least one prior population pass has already run.

**This is the same shape of bug as the calibration-loop finding** (Parts
5-8 of `BUG_calibration_word_misplaced_0d00.md`): cpu040 reaching a ROM
checkpoint before some other, slower, real-time-paced or ordering-
dependent process has had a chance to do its part — not a data-integrity
or LSU-arithmetic bug (already ruled out in Part 1, reconfirmed here).
Given this and the calibration bug share that exact signature, and given
this hang is only reached in the first place via the calibration bug's
own fault-trampoline detour, **the leading hypothesis is now that this
is a secondary symptom of the same underlying instability, not an
independently-caused hang** — fixing whatever makes cpu040 outrun
ROM-assumed timing in the calibration loop (the interrupt-recognition-
during-tight-loop bug currently being chased via ILA in
`BUG_calibration_word_misplaced_0d00.md` Part 9) may resolve this hang
as a side effect, without needing a separate fix. Not proven (this
document does not trace what the "missing prior population pass" is or
confirm it's timing-gated rather than ordering-gated), but a strong,
well-evidenced lead — the MAME reference cross-check technique that
settled Part 7 for the sibling bug works identically here and gives an
equally clean, decisive answer.

### Status update: root cause of "why is the table empty" narrowed, not
### fully closed

Still open: WHAT specifically is supposed to add the first entry before
this walk runs, and whether cpu040 reaches the walk too early because of
lost real time (an interrupt-recognition-during-tight-loop-style stall
elsewhere) or a strict ordering violation (some earlier step being
skipped rather than merely delayed). Given the strong thematic link,
recommend treating this as downstream of the calibration-loop
investigation rather than opening a fully separate RTL-fix effort for
it — re-check this hang after any fix to the interrupt-recognition bug,
before investing further dedicated effort here.

## Part 4 (2026-08-27, same day) — FIXED, ROM patch, CONFIRMED ON REAL
## HARDWARE (not a self-resolving side effect after all)

Part 3's hope that fixing the calibration-loop bug (see
`docs/BUG_calibration_word_misplaced_0d00.md` Part 11) would also
resolve this hang as a side effect did NOT pan out — real cpu040
hardware, freshly booted with the calibration-fix ROM patch applied,
still reproduced this exact hang (`pc=0x4080677c`) immediately
downstream. So this bug got its own dedicated fix, same toolkit as the
calibration fix: a static ROM patch (`zonewalk-empty-table-fix` /
`calibration-and-zonewalk-fix`, added to `cpu/tb/models/rom_patch_sets.h`,
cpu submodule commit `85c2c535` on branch `feat/calibration-fix-rom-patch`,
NOT YET MERGED).

**Root cause of the write mechanism, further narrowed via MAME**: the
"next" field the walk reads is populated by a single write site at
`0x40806168: movel %a1,%a3@(2)` (a SlotManager-trap sub-handler that
stamps a self-referential marker into each 192-byte slot). Across a
full 41s MAME boot-to-Finder run this fires **9 separate times**,
roughly 130,000-150,000 MAME cycles apart, each preceded by ~240
further SlotManager sub-dispatches and genuine Memory-Manager work — a
slow, incremental, per-record process, not a single early burst. **No
hardware real-time wait (VIA/SCC poll) was found in the inter-write
windows**, so — unlike the calibration bug — this is NOT confirmed to
be the same "loses a fixed-duration hardware timer race" shape; whether
cpu040 races this population loop on raw speed, or simply arrives at
the walk via a shorter/different code path (most plausibly downstream
of the calibration-loop's own fault-trampoline detour, which is a
`cpu040-only` code path real hardware never takes at all), is left
genuinely open.

**Given that ambiguity, the fix does not try to force/skip an upstream
delay** (there may not even be one) — it instead gives the walk loop
the missing `next == 0` termination check the "Root cause, precisely
localised" section above already identified as the actual defect,
which is correct regardless of *why* the table is empty when reached.
Implementation: the 6-byte loop tail (`moveal A1@(2),A1` / `bras
0x40806778`) is redirected via `bra.l` to a small patch body relocated
into an already-provably-unreachable-on-this-config ROM helper
(`0x475e8`-`0x47607`, independently reconfirmed via live MAME coverage
tracing across a 50s boot) that adds the check, then either takes the
existing safe "not found" return (`0x4080679e`) or follows the real
link and loops back, exactly reproducing the original instruction's
behavior for the populated case.

**Verified**: static disassembly of the patched bytes matches the
intended opcodes exactly. Checksum (`0x420DB506` standalone /
`0x420D7B15` combined with `calibration-fix`) independently re-derived
via a full Python re-sum, both self-consistent. A synthetic A/B
reproduction — poking a slot's `next` field to `0x00000000`
immediately before the compare via the MAME debugger, replicating
cpu040's exact observed failure state — hangs the **unpatched** ROM
forever (MAME's own reported emulation speed visibly collapses) and is
handled cleanly by the **patched** ROM within 46 cycles of the poke,
zero extra loop iterations. All 9 natural zonewalk calls in a full
MAME boot-to-Finder run resolve byte-for-byte identically with the
patch applied — inert for the normal case, as intended.

**Confirmed on real cpu040 hardware** (same session): wrote the
combined-fix ROM to the physical SD card, booted cpu040 fresh with its
existing bitstream (no rebuild needed) — reached PAST this hang
entirely, `pc=0x408046aa`, genuinely new territory never reached in
any earlier attempt this whole investigation. The fix works on real
silicon, not just MAME.

### What's next

Past this hang, cpu040 hit a THIRD, distinct issue almost immediately
(a wild jump to an unmapped address, `pc=0x39000620`) — under
investigation as of this writing, tracked in
`docs/BUG_calibration_word_misplaced_0d00.md`'s ledger since it was
found in the same real-hardware session, not yet given its own
dedicated document. This bug (the zone-walk hang) is CLOSED as far as
this document is concerned.

### Deployment note

Same caveat as the calibration fix: this patch currently lives ONLY on
the test SD card and the unmerged `feat/calibration-fix-rom-patch`
branch — not applied to the canonical `files/420dbff3.rom` or any
default build/provisioning path yet.

## Part 5 (2026-09-03) — REFRAMED FROM A COMPLETE MAME REFERENCE DERIVATION:
## THIS IS THE SLOT RESOURCE TABLE, THE LOOP IS A PREDECESSOR SEARCH, AND THE
## TABLE IS POPULATED-BUT-SHORT (8 of 76), NOT EMPTY

Full derivation, evidence, MAME instruction indices and reproduction recipe
live in `docs/BUG_calibration_word_misplaced_0d00.md` **Part 114**. Summary of
what this document got wrong, so nobody re-derives from the wrong base:

1. **"The loop expects a circular list and stops when the chase leads back to
   the zone's own base (a wraparound-to-start check)" — WRONG.**
   `cmpa.l (2,a1),a2` is a **predecessor search**. `A2` is not a wraparound
   sentinel; it is *the block being freed*, loaded from `[$0CBC]-16` at
   `0x4080673C`. The loop terminates when it finds the block whose next-link
   points at `A2`. In MAME it exits 9 times out of 9, after 9, 8, 7, ... 1
   iterations (45 total — matching the trace exactly).

2. **"The zone's first block has its next field set to `0x00000000` instead of
   a value that would satisfy the wraparound check" — WRONG, that is normal.**
   A live MAME dump of the final SRT state shows the last block in the chain
   with `ff ff` at `+0xC0` and `00 00 00 00` at `+0xC2`. A NULL next-link in
   the tail block is the correct steady state. Nothing "failed to close a
   ring", because there is no ring.

3. **"`A0 = A2 = 0x000087E0` … meaning zero entries had ever been added" —
   WRONG inference.** `A2 == head` is what you observe when the record count
   has just decremented **to 0**, i.e. after entries were added and all of
   them removed. An SRT that never had an entry cannot reach this code at all
   (the delete path bails at `0x4080671E` when the record lookup fails, which
   is upstream of the count decrement). Part 2's own hardware dump proves
   population: the eight 24-byte records carry payload pointers
   `0x408F5A90 / 5AC4 / 5AF6 / 5B1A`, which are the ROM sResource-list
   pointers for directory entries 3–6 (sRsrcIDs `04`, `06`, `08`, `0D`), and
   `_NewPtrSysClear` zeroes a fresh block, so those can only have been written
   by the append path.

4. **"192-byte heap-zone block stride" — the real layout** is: a 198-byte
   (`0xC6`) `_NewPtrSysClear` block holding **8 × 24-byte sResource records**,
   a `$FFFF` end-marker word at `+0xC0`, and the next-block longword link at
   `+0xC2`. Part 2's "8 repeating 24-byte records" observation was right and
   is now explained. `$FF01` in a record's first word means "free record";
   `$FFFF` means "end of block".

5. **`0xB6DB6DB6` is not open-bus / DDR PRBS residue.** A `$0D24` watchpoint
   catches the ROM's own RAM diagnostic writing `B6DB6DB6 / 6DB6DB6D /
   DB6DB6DB` through low memory at `pc=0x4084728E` and `pc=0x40847306`. MAME's
   heap contains the same pattern. This document's hop-by-hop table used
   "matches the DDR PRBS residue" as corroboration; that corroboration is
   void. (The *hop values themselves*, verified byte-for-byte against direct
   JTAG reads, still stand — as does the "not an LSU misaligned-load bug"
   conclusion.)

6. **What is actually broken.** MAME enumerates **76** sResources into the SRT
   from a ROM-resident directory at `0x408F58F0` (77 longwords, `ID=$FF`
   terminated) and the count bottoms out at 7, so the head block is never
   freed and the degenerate case never arises. cpu040 enumerates ~**8**, the
   first prune pass deletes exactly those eight, the count reaches 0, and the
   walk is asked to find the predecessor of the head block — which does not
   exist. **The defect is in the enumeration, not in the walk**, and it sits
   at MAME instructions **17,377,367 → 17,565,864** (`totalcycles`
   116,048,596 → 117,371,213).

7. **The Part 4 ROM patch is re-classified.** Adding a `next == 0` check to
   the walk is a symptom guard for a ROM path that is only degenerate because
   the table is short; it does not address the enumeration truncation, and per
   the project's standing rule (ROM patches are legitimate only for genuine
   real-time speed races — and Part 111 has since killed the "cpu040 is too
   fast" hypothesis for this blocker) it should not be treated as the fix.
   It remains useful only as a way to boot past this point for further
   downstream investigation.

**Next step**, cheapest first: break at `0x4080615A` on cpu040 (the
"allocate another block" dispatch, reached only when a block fills). Never
reached ⇒ enumeration truncated at ≤ 8 records and the block allocator is
exonerated; then break at `0x40806198` and read `D3` / `D5` / `D0` to identify
which of the five enumeration exits fired.
