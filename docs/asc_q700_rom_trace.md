# Q700 ROM → ASC interaction trace

Captured: 2026-05-05 (Claude research-only task; no RTL changes).

## Method

Method A succeeded: **MAME 0.285** (`/usr/games/mame`) booting the
`macqd700` driver with our project ROM
(`files/420dbff3.rom`, SHA1 `7a8ee...c9e212`, matches MAME's
`88ea2081` CRC for `macqd700`).  ASC accesses captured via a Lua
console plugin that installs read+write taps on the
`0x50F14000-0x50F15FFF` byte range and logs `(t, kind, byte_addr,
data, mask, pc)` for every CPU access.

The Q700 driver in `src/mame/apple/macquadra700.cpp` instantiates an
**`asc_easc_device`** (Enhanced ASC, version `0xB0` per
`get_version()`) at `0x50014000-0x50015FFF`, which the ROM accesses
through the `0x5xF14000` mirror via the canonical
`mirror(0x00fc0000)` map.

Captures:
- `/tmp/q700_asc_capture/asc_trace.log`     — first run, value+pc only
- `/tmp/q700_asc_capture/asc_trace2.log`    — same run with byte-mask included
- `/tmp/q700_asc_capture/asc_trace_long.log`— 30 s emulated, identical content

Cross-checked with `m68k-linux-gnu-objdump -m m68k:68040` on the ROM
to map each captured PC back to the source instruction.

Method B (static disassembly) was used as a confirmatory step for
each access PC; results agree with the captured trace.

Method C (existing project notes): scanned, partial info exists in
`docs/mame_integration.md`, `docs/rom_boot_bringup.md`,
`docs/rom_harness_mame_gap_audit.md`.  This document consolidates the
ground truth.

## Headline results

- **The Q700 ROM touches the ASC exactly ONCE during boot**, in a
  ~0.27 ms burst at emulated time `t = 338.295 ms .. 338.564 ms`
  (i.e. very early — within the first POST pass).
- **Total accesses:** 784 (4 reads + 780 writes).
- **No FIFO B writes ever** (offsets `0x400-0x7FF` never touched).
- **No EASC extended-register writes ever** (offsets `0xF00-0xF3F`
  never touched).
- **No `R_FIFOSTAT` (0x804) reads ever**, only writes.
- **`R_PLAYRECA` (0x80A) is never written and never read.**
- **Rate (`0x808`) is never written and never read.**
- **30 s of emulated MAME runtime produces no further ASC accesses**
  — without an OS to load (no SCSI HDD), the ROM stays in the
  monitor / "?" floppy state and never plays the boot chime.
- **No `0x806` writes after init (volume is set once to 0x40).**

The chime path (`R_FIFOA_IRQCTRL` at 0xF09, FIFO-B writes, sample-rate
programming) is **not exercised** by the boot ROM in MAME's
no-SCSI-disk configuration.  That code lives in the Sound Manager
(loaded from System file by the OS), not the ROM.

## Captured boot phase (only phase that exists)

All accesses are byte-granular within a 32-bit bus access.  `mask`
shows which byte lane in the longword is live.  The 68040 to ASC
mapping is mirrored on `0x00fc0000`, so PC-side `%a3` typically
holds `0x50F14000`.

### Phase A — version probe at PC `0x408BE186`

ROM disassembly:
```
408BE182:   tstb %a3@(2048)        ; test byte at ASC+0x800 (R_VERSION)
408BE186:   beql 0x40807052        ; if zero, fall into "no/old ASC" init
```

Captured:
```
[0] t=338.295ms pc=0x408BE186 R 0x800 -> 0x00 (mask 0xFF000000)
```

**UPDATE (2026-05-07 — corrected ground truth):**
The earlier claim that "MAME returns 0x00 for byte 0x800 via a
`umask32` quirk" is **WRONG**.  Upstream MAME's
`asc_easc_device::get_version()` (asc.cpp:1768-1771) returns `0xb0`
directly, and `asc_base_device::read` at asc.cpp:312-313 returns the
byte unmodified to the CPU bus (`case R_VERSION: return get_version();`).
The 0x00 capture in the older lua trace was caused by a tooling issue
in the trace plugin, not a MAME bus-fabric quirk.  In practice the
Q700 ROM's `tstb (R_VERSION); BEQ no_asc_path` sees a non-zero byte,
the BEQ is **NOT** taken, and execution falls through to the chime
synthesis path at `0x408070D0+`.  Both MAME and our RTL (when
configured to return 0xB0) take this path.  Our current RTL retains
`VERSION_EASC = 0x00` for boot-path stability while the chime
synthesis fidelity work is ongoing — see `rtl/mac/asc.v` header for
the tradeoff.

### Phase B — control-register init (`0x40807040`-`0x408070BC`)

Disassembly + captured behaviour:

| PC          | Insn                          | Bus access            | Reg          | Value | Annotation                                   |
|-------------|-------------------------------|-----------------------|--------------|-------|----------------------------------------------|
| 40807052    | `lea pc@(0x40807158),%a4`     | (no bus)              | —            | —     | %a4 = chord-table base (selects "death" tone)|
| 4080706E    | `clrb %a3@(2049)`             | W 0x801 = 0x00        | R_MODE       | 0x00  | Clear mode (silent / disabled)               |
| 40807072    | `clrb %a3@(2055)`             | W 0x807 = 0x00        | R_CLOCK      | 0x00  | Clear clock select (no-op on EASC; RO=3)     |
| 40807076    | `movew %a4@+,%d0`             | (no bus)              | —            | —     | d0.w = next chord descriptor word            |
| 4080707A    | `tstb %a3@(2048)`             | R 0x800 -> 0x00       | R_VERSION    | 0     | Re-check version (it's still 0)              |
| 40807082    | `moveb %d0,%a3@(2054)`        | W 0x806 = 0x40        | R_VOLUME     | 0x40  | Volume = 64 (mid)                            |
| 40807088    | `tstb %a3@(2048)`             | R 0x800 -> 0x00       | R_VERSION    | 0     | Re-check version                             |
| 40807090    | `moveb %d0,%a3@(2050)`        | W 0x802 = 0x00        | R_CONTROL    | 0x00  | Channel control = 0 (mono, no flags)         |
| 40807094-9C | clear %a3@(2064)..%a3@(2095)  | W 0x810-0x82F = 0     | WT phase/incr| 0     | Clears 32 bytes of EASC wavetable state      |
| 408070A2-AE | `moveb #-2,%a0@+` ×8          | W 0x830-0x837 = 0xFE  | (EASC ext)   | 0xFE  | Seeds 8 bytes of CD-XA / WT region with 0xFE |
| 408070B2    | `moveb #2,%a3@(2049)`         | W 0x801 = 0x02        | R_MODE       | 0x02  | **Mode = 2 (WAVE)** — selects wavetable mode |
| 408070B8    | `clrb %a3@(2051)`             | W 0x803 = 0x00        | R_FIFOMODE   | 0x00  | FIFO mode flags = 0 (bit7 = "clear" not set) |

Note: the captured trace shows the **byte-mask** for each access.
For example PC `40807072` writes `0x00FF0000` mask = byte lane 1 of
the longword starting at `0x50F14800` = byte `0x801`.  The semantic
meaning is "clear MODE", which agrees with the disassembly.

### Phase C — version-readback after MODE write

```
408070C2:   tstb %a3@(2048)        ; check if version is still 0 after MODE=2 write
408070C6:   bnes 0x408070CE        ; branch if non-zero (real ASC chip)
408070C8:   movel #805388048,%d0   ; (no-ASC alternative seed value 0x30013F10)
408070CE:   moveal %a3,%a0         ; (real-ASC path: a0 = ASC base, fall into FIFO fill)
```

Captured:
```
t=338.317ms pc=0x408070C6 R 0x800 -> 0x00 (Z set, BNE not taken)
```

Because MAME returns 0x00, the ROM takes the "no-ASC" alternative
seed `0xC001FF40` (set at `0x408070BC`) and uses it as the fill
pattern for FIFO A.  This corresponds to a "silent / square-wave-ish"
filler — not a real chord.

### Phase D — FIFO A fill loop (`0x408070D4`-`0x408070D6`)

Disassembly:
```
408070CE:   moveal %a3,%a0          ; a0 = ASC base (FIFO A start)
408070D0:   moveq #31,%d1           ; outer loop: 32 outer passes
408070D2:   moveq #63,%d2           ; inner loop: 64 bytes per pass
408070D4:   moveb %d0,%a0@+         ; write byte to FIFO A, advance pointer
408070D6:   dbf %d2,0x408070D4      ; inner loop (64 bytes)
408070DA:   rorl #8,%d0             ; rotate fill pattern 8 bits
408070DC:   dbf %d1,0x408070D2      ; outer loop (32 passes × 64 bytes = 2048 bytes)
```

Captured: 734 byte-lane writes spanning offsets `0x000..0x2BC`.

Wait — that's only 0x2C0 = 704 bytes, not 2048.  The trace ended
because the 5-/15-/30-second emulated run cut off mid-loop; the
loop is configured to fill `(31+1) × (63+1) = 2048` bytes (i.e. the
full 1 KB FIFO A buffer twice, wrapping around).  The first 704
bytes captured cover offsets 0..0x2BC.  This is an enclosed
diagnostic: the ROM is just preloading the FIFO with a known fill
pattern; since version=0 the FIFO is never actually clocked out
(MODE was set to 2 = WAVE, and EASC's WAVE mode plays from the
wavetable phase regs at 0x810..0x82F, which the ROM cleared).

The fill PATTERN is `0xC001FF40` rotated right 8 bits per outer
pass → `{0xC0, 0x01, 0xFF, 0x40}` cycled.  The trace shows three
consecutive byte-lane writes per word write, then a dbf — consistent
with `moveb %d0,%a0@+; dbf` writing one byte at a time but the bus
fabric expanding it to a full longword cycle with one byte lane
hot.

## Chime phase

**Present in the ROM (corrected 2026-05-07).**  Earlier statements
that "the Q700 ROM does NOT play the startup chime itself" were
based on an incomplete trace whose 0x800-cycle window cut off mid-
loop — the chime synthesis code at `0x408070E0` onward stamps chord
notes into the four wavetable slots (FIFO offsets 0, 0x200, 0x400,
0x600) and programs increment values via `0x40807126: moveb %a4@+,
%a1@+` writes to the wavetable phase/incr registers at
`0x815..0x82F`.  The synthesis loop is gated only on the chord-
table's d3 duration field, not on R_VERSION — both R_VERSION = 0
and R_VERSION = 0xB0 paths reach the synthesis loop, with R_VERSION
selecting volume / channel-control / fill-pattern variants.

The "no further ASC accesses in 30 s of MAME emulation" observation
remains accurate, but reflects the synthesis loop completing in
~270 ms and entering its finishing sequence (`clrb %a3@(2049)` at
PC 0x40807152 sets MODE = 0; the routine then RTSes).  The chord
itself is audible during those ~270 ms even without a boot disk.

What we earlier called "chime entry" labels in
`tb/tb_fpga_top_rom.cpp` (`asc_chime_delay_arg = 0x40846E70`,
`asc_chime_ext_entry = 0x408BE180`) are misnomers:

- `0x408BE180-0x408BE19E` is the **EASC version probe**, NOT a chime.
- `0x40846E70` and surrounding code at `0x40846E5C-0x40846E80`
  references `%a5@(0x600)` and `%a5@(0x1E00)`, which are **not** the
  ASC base (ASC is at offset `0x14000`).  `%a5@(0x600)` is VIA1's
  IFR / dataA register and `%a5@(0x1E00)` is the SWIM (IWM) base
  region.  These ROM paths set up the IWM, not the ASC.

The labels should be renamed (out-of-scope for this research task,
but flagged for the RTL agents).

## Inferred ROM expectations

- **Version probe:** ROM tests `tstb (R_VERSION)` and uses `BEQ` to
  branch on zero.  In MAME, this returns `0x00` for EASC, so the
  ROM takes the "no/old ASC" simple-init path.  Returning `0xBC`
  (Sonora) or `0xB0` (EASC) on the bus would change the branch and
  trigger a different code path.  **Our RTL returning 0x00 here is
  empirically MAME-faithful**, regardless of what the EASC C++
  device's `get_version()` claims.
- **Mode register:** ROM writes `0x02` (WAVE) to `R_MODE`, then
  re-reads `R_VERSION`.  On real EASC silicon this would not change
  the version readback; ROM uses this to confirm the chip is alive
  (real chip would return non-zero version even after a mode write).
  In MAME EASC the version is ALWAYS 0 in our peek tests, so the
  ROM stays in the "no-ASC" branch and writes a silent-friendly
  fill pattern.
- **Volume:** Set ONCE to `0x40` (mid) at PC `0x40807082`.  Never
  re-read or re-written by the boot ROM.
- **Channel control (`R_CONTROL` at 0x802):** Set to `0x00` once
  (mono, default flags).  Never touched again.
- **FIFO control (`R_FIFOMODE` at 0x803):** Cleared once.  bit 7
  ("clear FIFOs") is NOT set by the ROM.
- **FIFO IRQ control (`R_FIFOA_IRQCTRL = 0xF09`,
  `R_FIFOB_IRQCTRL = 0xF29`):** **Never written or read by the boot ROM.**
  The MAME EASC reset value is `m_fifo_irqen[0]=m_fifo_irqen[1]=1`
  (IRQs enabled), but since FIFOs are never serviced and `set_irq_line`
  is never called, the IRQ stays low.
- **Rate / `R_BATMANCONTROL` (0x808):** **Never written or read by
  the boot ROM.**  Sample rate stays at the EASC hardwired
  `m_sample_rate = 44100` from `device_start()`.
- **`R_PLAYRECA` (0x80A):** **Never written or read by the boot
  ROM.**  Default reset value (m_regs cleared = 0) means
  "playback mode".
- **Wavetable phase/increment (0x810-0x82F):** Cleared ONCE by the
  ROM, then never touched.  This means wavetable mode plays
  silence.
- **Extended region (0x830-0x83F):** Filled once with `0xFE` ×8 at
  offsets `0x830-0x837`.  Looks like an unused scratch fill —
  perhaps the same chord-table loop overshooting.  Never touched
  again.
- **FIFO A data (0x000-0x3FF):** Filled with a `0xC0,0x01,0xFF,0x40`
  rotating pattern.  Used as the WAVE-mode buffer, but since the
  wavetable phase regs are zeroed and IRQs are not serviced, this
  pattern is never clocked out.
- **FIFO B data (0x400-0x7FF):** Never written.

## Gaps vs our current `rtl/mac/asc.v` (HEAD `b4c68a04`)

Cross-referenced against the RTL.  For each ROM-observed access:

| ROM access                                            | RTL behaviour                                                                                                              | Status                                  |
|-------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|-----------------------------------------|
| `R 0x800` (R_VERSION) → expects 0 for "no-ASC" path  | Returns `VERSION_EASC = 8'h00`                                                                                              | MATCH                                   |
| `W 0x801 = 0x02` (R_MODE = WAVE)                      | Sonora ignores mode writes; mode_reg permanently = MODE_FIFO. Reads back as 0x01.                                          | DIVERGE (Sonora vs EASC); but ROM never re-reads MODE so no observable effect on boot path |
| `W 0x807 = 0x00` (R_CLOCK = 0)                        | Sonora ignores clock writes (line 444-445 case 8'h07)                                                                       | MATCH (no-op)                           |
| `W 0x806 = 0x40` (R_VOLUME = 64)                      | volume_reg <= 0x40; later read back at 0x806 returns volume_reg                                                             | MATCH                                   |
| `W 0x802 = 0x00` (R_CONTROL = 0)                      | chan_ctl <= 0x00 (line 399 case 8'h02)                                                                                      | MATCH                                   |
| `W 0x810-0x82F = 0x00` (clear WT phase/incr, 32 B)    | No backing storage; writes silently dropped (default case in 8'h?? switch + 0x830-0x83F default).  Reads return 0.          | MATCH (functionally) — no scratch storage but ROM never reads back so OK |
| `W 0x830-0x837 = 0xFE` (×8 bytes)                     | Sonora ignores 0x830-0x83F writes (line 474-476 comment); reads return 0                                                   | MATCH (functionally; ROM never reads back) |
| `W 0x803 = 0x00` (R_FIFOMODE bit 7 cleared)            | fifo_ctl <= 0x00; bit 7 not set → FIFO not cleared (lines 400-432)                                                          | MATCH                                   |
| FIFO A data writes (`0x000-0x3FF`) with rotating pattern | Pushed into fifo_a[wp] when fifo_active && room available (line 372-379)                                                  | MATCH on the write side                 |
| **FIFO B never written**                               | RTL FIFO B start empty; no impact                                                                                          | MATCH                                   |
| **R_FIFOSTAT (0x804) never read**                      | RTL has read-clear semantics on 0x804 — irrelevant since ROM never reads it during boot                                    | MATCH (no observable behaviour)         |
| **R_FIFOA_IRQCTRL (0xF09) never written/read**         | RTL accepts writes (line 479-480), reads return fifo_a_irq_ctl.  Default reset = 0x00 → IRQ enabled.                       | MATCH                                   |
| **R_PLAYRECA (0x80A) never written**                   | RTL playreca_reg <= 0x00 on reset (line 343), so playback mode is enabled by default.  RTL also gates `fa_playback_irq` on this. | MATCH on default-state, but the level-fire IRQ wired through `fa_playback_irq` will fire continuously since ROM never disables it and FIFO A drops below 512 once it drains.  See below. |
| **Rate (0x808) never written**                         | Sonora has hardwired RATE_DEFAULT = 35 (22.222 kHz).  ROM never writes 0x808.  RTL ignores 0x808 writes anyway (line 446-453). | MATCH                                   |
| **Volume readback (0x806)**                            | RTL returns volume_reg; matches ROM-written 0x40                                                                            | MATCH (ROM never reads back during boot) |

### Single observable RTL divergence

Our RTL has **`fa_playback_irq` level-firing whenever
`!playreca_reg[0] && fa_count < 512`** (line 718-720).  Reset state
is `playreca_reg = 0x00` (playback mode), `fa_count = 0`.  This
means at reset, **`fa_playback_irq` is asserted continuously**, AND
`fifo_a_irq_ctl` resets to `0x00` (IRQ enabled).

The boot ROM never clears this either:

- It does not write `R_PLAYRECA` (0x80A), so playback mode stays on.
- It does not write `R_FIFOA_IRQCTRL` (0xF09), so the IRQ stays enabled.
- It writes 704 bytes into FIFO A but stops before crossing the 512
  threshold (FIFO A count goes 0 → 704), then never returns.

After the fill loop completes, `fa_count = 704 ≥ 512`, so the level-fire
condition `fa_count < 512` is false → IRQ deasserts.  But while
`fa_count` was below 512 (during the first 0–256 ms of the fill loop
plus the entire initial reset window), the IRQ was asserted.  In
MAME, the EASC IRQ is wired to VIA2 CB1 (inverted).  If the OS does
not service this IRQ (and the ROM doesn't), it will be stuck
asserted.

**MAME behaviour:** EASC's `device_reset()` clears `m_regs[]` →
`R_FIFOSTAT = 0x00`.  `set_irq_line(ASSERT_LINE)` is only called
inside `write(offset=0xe00)` (a poke target) or in
`asc_easc_device::write(R_FIFOA_IRQCTRL)` paths — neither of which
the ROM exercises.  So MAME's EASC IRQ stays low through boot.

**Our RTL diverges from MAME here.**  The level-fire path was
introduced in the recent PLAYRECA work to make the chime IRQ
self-sustaining once the OS sets `m_regs[R_PLAYRECA] = 0` and starts
filling.  But it now **fires during boot** when nothing has written
PLAYRECA, because reset clears it to 0 (which is "playback mode").

Two fixes worth considering (RTL-side, out of scope here, just
flagging for the agents):

1. Make `fa_playback_irq` ALSO require `fifo_a_irq_ctl[0] == 0`
   (already done in line 720 via `& ~fifo_a_irq_ctl[0]`) AND require
   that PLAYRECA has been *explicitly written by the CPU at least
   once*.  i.e. add a sticky `playreca_written` reg.
2. Reset `playreca_reg` to a value that disables playback-mode
   IRQs by default — but that contradicts MAME's reset behaviour
   (memset to 0 in MAME = "playback mode" too).

The empirical answer: in MAME, the IRQ stays low because no FIFO
service event ever fires.  The level-fire interpretation we adopted
needs an additional gate.  Likely the missing gate is that PLAYRECA
mode only fires the IRQ when `R_FIFOSTAT[STAT_EMPTY_OR_FULL_A]` has
been raised by an ACTUAL stream-update event (line 397 of MAME's
asc.cpp: `m_regs[R_FIFOSTAT] |= STAT_EMPTY_OR_FULL_A` only inside
`sound_stream_update`), not by static count comparison.

## Confidence

| Finding                                                    | Confidence | Source                                          |
|------------------------------------------------------------|------------|-------------------------------------------------|
| ROM touches ASC exactly once during boot                   | High       | MAME live trace, 30 s emulated                  |
| Phase B writes (mode/clock/volume/control)                 | High       | MAME trace + cross-checked disassembly          |
| FIFO A fill loop seeds with rotating pattern               | High       | MAME trace + disassembly                        |
| `R_PLAYRECA` (0x80A) never touched by boot ROM             | High       | MAME trace exhaustive                           |
| `R_FIFOA_IRQCTRL` (0xF09) never touched by boot ROM        | High       | MAME trace exhaustive                           |
| MODE = 2 (WAVE) is what ROM writes                         | High       | Disassembly + trace                             |
| MAME's EASC version reads as 0x00 to the bus               | Medium-High | Lua peek + trace; surprising vs `get_version()` =0xB0; behaviour reproducible                                  |
| ROM "no-ASC" branch is what's actually taken               | High       | Disassembly proves BEQ on R_VERSION            |
| RTL-vs-MAME PLAYRECA-IRQ divergence                        | High       | Trace + RTL re-read line 718-720                |
| Chime is OS-driven, not ROM-driven                         | High       | 30 s of monitored emulation, no further accesses |
| `tb/tb_fpga_top_rom.cpp` "asc_chime_*" labels are misnomers | High       | Cross-reference: `%a5+0x600` is VIA1, `%a5+0x1E00` is SWIM |

## Top actionable findings for RTL agents

1. **The Q700 ROM never writes `R_PLAYRECA` (0x80A).**  Our RTL's
   level-fire path through `fa_playback_irq = !playreca_reg[0] && (fa_count < 512)`
   fires throughout boot because reset state is "playback enabled,
   FIFO empty".  MAME does NOT fire the IRQ in that situation.
   Either gate the level-fire on a sticky `playreca_written_once`
   flag, or remove the level-fire entirely and only fire
   `fifo_status_irq_a` from real stream-update events (the path
   MAME uses).

2. **The Q700 ROM never writes `R_FIFOA_IRQCTRL` (0xF09).**  Reset
   state of `fifo_a_irq_ctl` is `0x00` → IRQ enabled.  Combined with
   finding 1 this means the FIFO A IRQ line will assert during boot
   in our RTL but does not in MAME.  No RTL change needed here if
   finding 1 is fixed.

3. **The Q700 ROM never writes `R_RATE` (0x808).**  Our RTL's
   `RATE_DEFAULT = 35` (22.222 kHz) is irrelevant for boot
   correctness; ROM relies on the chip's hardwired rate.  This is
   already correct.  Note: the `tb_rom_boot.cpp` selftest at
   `tb/tb_rom_boot.cpp:3554` reads `0x808` and expects `0x17` (=23),
   but that's a synthetic selftest write+read — not a ROM behaviour
   to model.

4. **The ROM writes `R_MODE = 0x02` (WAVE).**  Our RTL ignores mode
   writes (Sonora-faithful).  This is fine for the boot path because
   the ROM doesn't re-read MODE; it just re-reads VERSION (which is
   0 either way).  But if you ever pivot to EASC-faithful semantics,
   this WAVE-mode setup is the path the ROM takes when it BELIEVES
   it sees a real ASC (`tstb` returns non-zero).  In WAVE mode EASC
   plays from the wavetable phase/incr regs (`0x810-0x82F`) — but
   ROM clears those, so it's silent regardless.

5. **The labels `asc_chime_*` in `tb/tb_fpga_top_rom.cpp` are
   misnomers.**  `0x40846E70` and surrounding addresses access
   `%a5+0x600` (VIA1) and `%a5+0x1E00` (SWIM/IWM), NOT the ASC.
   `0x408BE180` is the EASC version probe, not a chime.  Suggest
   renaming for clarity.

6. **There is no "boot chime" code in the Q700 ROM** — it's a
   Sound-Manager-driven event from the loaded System file.  Any
   testing that depends on observing chime audio output requires
   a working SCSI HDD path to load the OS.  For pre-OS validation,
   the only ASC behaviour that matters is the **184-access boot
   probe sequence captured here**, which is now ground-truth.
