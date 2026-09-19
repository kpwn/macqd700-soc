# BUG: boot hang polling VIA2 `vBufB` bit 6 — **NOT a `via2_pb_in` bug**

**Status**: ROOT CAUSE RE-IDENTIFIED 2026-08-26. The original diagnosis in
this file (below, kept for the record) blamed `via2_pb_in = 8'hCF`. That is
**wrong**, and the "fix" it proposed (deriving PB6 from a real signal) would
have been a fabricated toggle on a bit that must not toggle. `8'hCF` is
**correct and must stay**.

The real bug is one layer up: **cpu040 loads and runs the wrong built-in
video driver** — `.Display_Video_Apple_RBV` (the Macintosh IIci/IIsi
"RAM-Based Video" driver) instead of `.Display_Video_Apple_DAFB` (the
Quadra 700's actual driver). The RBV driver polls its chip's byte-spaced
pseudo-VIA at `base+2`; on a Q700 `base` is a *real* 6522 VIA2 with
512-byte register stride, so `base+2` aliases onto VIA2 ORB and the poll
degenerates into "wait for VIA2 PB6 to clear", which nothing can ever
satisfy.

**Severity**: HIGH, unchanged — still the terminal blocker on the deepest
boot outcome. Only the target of the fix changes.

Tracking note: do **not** open a `via2_pb_in` change off this document.

> **SUPERSEDED FOR THE ONGOING INVESTIGATION (2026-08-26, second pass).**
> The driver-selection mechanism has since been fully mapped and the divergence
> measured on both sides — see **`docs/BUG_wrong_video_driver_rbv_vs_dafb.md`**.
> Headlines: the board selects sResource spID `0x81` (RBV) with
> `dCtlDevBase = 0`, the healthy machine selects `0xDD` (DAFB) with
> `dCtlDevBase = 0xF9001420`; there is **no slot-`$F` declaration ROM to
> implement** (the Q700's built-in video is a *slot-0* pseudo-slot whose decl
> ROM lives inside the Mac ROM at `0x408F58F0`); and the SoC's DAFB register
> window is **alive and correct** on the board. **§1.5 below is factually
> wrong** — see the correction inline there.

---

## 1. Why the original `via2_pb_in` diagnosis is disproven

### 1.1 MAME's own Quadra 700 model uses the identical constant — and boots

`src/mame/apple/macquadra700.cpp` (MAME 0.285, the behavioural reference
this project cites everywhere else):

```cpp
u8 quadrax00_state::via2_in_a()
{
	return 0x80 | m_nubus_irq_state;
}

u8 quadrax00_state::via2_in_b()
{
	return 0xcf;        // indicate no NuBus transaction error
}
```

`macii.cpp:563` does the same (`return 0xcf;`). A hardwired `0xCF` on VIA2
Port B **is** the faithful model, and `mame_q700_good/run.sh` boots this
exact ROM + disk to the Finder with it. The stub is not a placeholder that
was "left behind" next to `via2_pa_in` — it matches ground truth.

(The Port A precedent really was a bug, because MAME derives Port A from
live `m_nubus_irq_state`. MAME derives Port B from nothing. The two are not
the same class of problem.)

### 1.2 A healthy Quadra 700 never executes the hanging code

The RAM code at `0x0000a8b8` is a relocated copy of ROM `0x408fbd98`
(delta `0x408F14E0` — see §2.1). Breakpoints were set on that routine and
its siblings in `files/420dbff3.rom` and MAME was run for **120 emulated
seconds** (well past the Finder):

| ROM address | what it is | hit in a healthy boot? |
|---|---|---|
| `0x40809BE0` | VIA2 interrupt dispatcher (**control**) | **HIT** |
| `0x408fbd7c` | RBV driver: raise IPL, then the PB6 wait-set/wait-clear | never |
| `0x408fbd98` | the exact poll loop we hang in | never |
| `0x408fbdb0` | RBV driver: `move.b d1,16(a0)` (pseudo-VIA `+0x10`) | never |
| `0x408fc576` | V8 driver: same `+2` bit-6 poll | never |
| `0x408fdca8` | Apollo driver: same `+2` bit-6 poll | never |

The control breakpoint firing is what makes the negatives meaningful — an
earlier attempt with `-debugger none` silently armed nothing and produced
a false "zero hits" for everything including the control. Always keep a
known-hit control breakpoint in these runs.

### 1.3 The address the ROM polls is correct on both machines

Healthy MAME RAM at t = 5/15/30/60 s:

```
$01D4 (VIA1)        = 0x50F00000
$0CEC (VIA2)        = 0x50F02000     <-- identical to our board
$0C00 (SCSIBase)    = 0x50F0F000
$0CC0 (ASCBase)     = 0x50F14000
$0DD8 (UnivInfoPtr) = 0x4080390C
```

Our board's `A0 = 0x50F02002` is exactly `[$0CEC] + 2`. So the low-memory
global, the decode in `glue.v`, and the `+2` byte offset are all *right*.
Nothing about the VIA2 address path is broken.

### 1.4 `via2_pb_in` is not CPU-target-conditional

`rtl/soc/fpga_top_peripherals.vh` contains no `CPU_M68K`/`CPU_M68K040`
conditionals at all (the only `ifdef`s in the file are `PB_S1_WIDE_CDC` and
`ENABLE_DDR_RAMDISK`). v1 and cpu040 synthesise **byte-identical** VIA2
wiring, and `tb/tb_cold_boot.v:1811` uses the same `8'hCF`. So "v1 boots
this SoC fine" cannot be explained by a per-target difference in this
signal — it can only mean v1 never reaches this code, which §2 explains.

### 1.5 It is not a stale cached MMIO read either — **THE TTR CLAIM BELOW IS WRONG**

> **CORRECTION (2026-08-26, second pass).** The bullet below asserts
> "`DTT0=DTT1=ITT0=ITT1=0`" as MAME ground truth "at the same point in boot".
> That snapshot was taken **after** the ROM had already finished with the TTRs.
> Sampled across a healthy boot the TTRs are very much live earlier:
> at `t = 1..4 s` the ROM runs with **`TC = 0` (MMU DISABLED)** and
> `DTT0 = 0x807FC040` (covers `0x80000000..0xFFFFFFFF`, cache-inhibited),
> `DTT1 = 0x500FC040` (covers `0x50000000..0x5FFFFFFF` — the whole VIA/SCC/
> SCSI window — cache-inhibited); inside the DAFB PrimaryInit it is
> `TC = 0xC000` with `DTT0 = 0xF900C060` (covers `0xF9xxxxxx`,
> cache-inhibited); only from `t = 5 s` on are they all zero.
> **So the "C3" TTR gap (TTRs ignored while the MMU is disabled,
> `DtlbPlugin.scala:246-252`) IS squarely on this boot path**, and it is the
> current leading root-cause candidate. Full analysis:
> `docs/BUG_wrong_video_driver_rbv_vs_dafb.md` §4-§5.
> The second bullet below (PB6 is a constant, so caching cannot explain *this
> particular spin loop*) still stands.


Hypothesised: VIA2 not marked cache-inhibited, so the D-cache serves a
frozen snapshot of `vBufB`. Ruled out as the *cause of this loop*:

* MAME ground truth at the same point in boot — `TC=0xC000`,
  **`DTT0=DTT1=ITT0=ITT1=0`**, `SRP=0x7FCC00`, `CACR=0x80008000`. Mac OS on
  a Q700 leaves **all four transparent-translation registers disabled**, so
  the "C3" TTR gap (TTRs ignored while the MMU is *disabled*,
  `DtlbPlugin.scala:249` forcing `CacheMode.WRITETHROUGH`) is not on this
  path at all. With the MMU on, cpu040 *does* consult DTT0/DTT1
  (`DtlbPlugin.scala:137-139`, task #194) and does honour
  `tlbEntry.cacheMode` from the walker (`DtlbPlugin.scala:263`).
* More decisively: **PB6 is a constant `1` in the RTL regardless.** A live,
  uncached re-read every poll returns exactly the same `1`. Caching is not
  needed to explain the hang, and removing it would not end the hang.

(This does not *prove* VIA2 is correctly inhibited on our board — it proves
cache behaviour is not what makes this loop spin. If a separate MMIO
staleness question needs answering, it needs its own investigation.)

---

## 2. The actual root cause: wrong video driver loaded

### 2.1 The hanging code belongs to `.Display_Video_Apple_RBV`

`files/420dbff3.rom` carries the whole family of built-in-video drivers as
Pascal-named blocks:

| ROM address of name string | driver |
|---|---|
| `0x408fb713` | `.Display_Video_Apple_RBV`  |
| `0x408fbeb3` | `.Display_Video_Apple_V8`   |
| `0x408fc781` | `.Display_Video_Apple_DAFB` ← the Q700's driver |
| `0x408fd6bf` | `.Display_Video_Apple_TIM`  |
| `0x408fd9ed` | `.Display_Video_Apple_Apollo` |
| `0x408fdd19` | `.Display_Video_Apple_DBLite` |

`0x408fbd7c`–`0x408fbdae` sits inside the **RBV** block. Disassembled:

```
408fbd7c: 48e7 8080       movem.l d0/a0,-(sp)
408fbd80: 40e7            move.w  sr,-(sp)
408fbd82: 7007            moveq   #7,d0
408fbd84: c017            and.b   (sp),d0          ; current IPL
408fbd86: 5500            subq.b  #2,d0
408fbd88: 6c08            bge.s   408fbd92
408fbd8a: 007c 0200       ori.w   #$0200,sr        ; force IPL = 2
408fbd8e: 027c faff       andi.w  #$faff,sr
408fbd92: 2078 0cec       movea.l ($0CEC).w,a0     ; VIA2 / pseudo-VIA base
408fbd96: 5448            addq.w  #2,a0            ; base + 2
408fbd98: 1010            move.b  (a0),d0          ; phase 1: wait bit6 == 1
408fbd9a: 0800 0006       btst    #6,d0
408fbd9e: 67f8            beq.s   408fbd98
408fbda0: 1010            move.b  (a0),d0          ; phase 2: wait bit6 == 0
408fbda2: 0800 0006       btst    #6,d0
408fbda6: 66f8            bne.s   408fbda0         ; <-- our board spins here
408fbda8: 46df            move.w  (sp)+,sr
408fbdaa: 4cdf 0101       movem.l (sp)+,d0/a0
408fbdae: 4e75            rts
```

Its four in-driver callers are `0x408fb8b4`, `0x408fb9dc`, `0x408fbb3e`
(this routine) and `0x408fb860`/`0x408fb8d2` (the `+0x10` sibling) — all
inside the RBV block, all reached by short `bsr`. It is a VBL/scan-line
sync helper for RBV, nothing to do with a 6522.

**The byte offsets prove the chip it was written for is not a 6522.** The
RBV driver touches its base at `+0`, `+2`, `+7`, `+0x0A`, `+0x0B`, `+0x10`,
and does `lea 0(a0),a0; clr.b (a0)+` fifteen times (`0x40809f78`) — a
**byte-spaced** register block, exactly the RBV/V8/VASP/Sonora pseudo-VIA.
A real Mac VIA has a 512-byte register stride, so on a Q700 every one of
those offsets collapses onto register 0 (ORB). The ROM's genuine Q700 VIA2
code, by contrast, uses proper strides — `btst #1,514(a1)` (`0x202` = ORA),
`and.b 6659(a1),d0` (`0x1A03` = IFR), `move.b #127,7168(a0)` (`0x1C00` =
IER). Both conventions read the same `$0CEC` global, because on an
RBV-class machine `$0CEC` points at the pseudo-VIA and on a Q700 it points
at a real VIA2.

The relocation delta pins it: RAM `0xa8b8` ↔ ROM `0x408fbd98` ⇒ delta
`0x408F14E0`, so the RBV block (`0x408fb713`–`0x408fbeb3`, ~1952 bytes)
lands at roughly RAM `0xA1E0`–`0xA9C0`, and PC `0xa8c0` is inside it.

### 2.2 A healthy Q700 loads only the DAFB driver

Searching MAME's healthy 8 MB RAM images at t = 30 s and t = 60 s for each
driver name:

```
.Display_Video_Apple_RBV      ABSENT
.Display_Video_Apple_V8       ABSENT
.Display_Video_Apple_DAFB     present at RAM 0x6A83
.Display_Video_Apple_TIM      ABSENT
.Display_Video_Apple_Apollo   ABSENT
.Display_Video_Apple_DBLite   ABSENT
RBV PB6 poll-loop bytes       ABSENT from RAM entirely
```

So on real/reference hardware exactly one video driver is instantiated into
the system heap, and it is DAFB. Our board instantiated **RBV** and jumped
into it. That is the bug.

### 2.2a CONFIRMED ON THE LIVE BOARD (2026-08-26)

The board was still parked at the hang (`pc_live=0x0000a8c2`), so this was
verified directly, no rebuild needed — `tools/jt.sh` takes an flock, so it
is safe alongside the peer REPL session; `halt` → dumps → `halt-release`
returned the board to the identical spin.

The §2.1 relocation delta predicted the RBV driver's name string at RAM
`0xA233`. Read back:

```
0x0000A230 = 0x04ba192e     <- 0xA232 = 0x19 (Pascal len 25), 0xA233 = '.'
0x0000A234 = 0x44697370     "Disp"
0x0000A238 = 0x6c61795f     "lay_"
0x0000A23C = 0x56696465     "Vide"
0x0000A240 = 0x6f5f4170     "o_Ap"
0x0000A244 = 0x706c655f     "ple_"
0x0000A248 = 0x52425631     "RBV1"
```

**`.Display_Video_Apple_RBV1`, at exactly the predicted address.** And at
`0x0000A298`: `20780cec 10280010` = `movea.l $0CEC,a0` / `move.b 16(a0),d0`
— the byte-spaced pseudo-VIA `+0x10` read, live in RAM.

Meanwhile RAM `0x00006A80..0x6A9F` — where the healthy machine keeps
`.Display_Video_Apple_DAFB` — is **all zeros** on our board.

**The machine identification is NOT the problem.** Every box-ID global
matches the healthy Q700 exactly:

| global | healthy Q700 | our board | verdict |
|---|---|---|---|
| `$0DD0` `UnivROMFlags` | `0x05A0183F` | `0x05A0183F` | match |
| `$0DD4` | `0x00000900` | `0x00000900` | match |
| `$0DD8` `UnivInfoPtr` | `0x4080390C` | `0x4080390C` | **match** |
| `$0CB0` `MMUFlags/Type/32bit/Fluff` | `0x00050010` | `0x00050010` | match |
| `$0C2C` `NMIFlag/VidType/VidMode/SCSIPoll` | `0x00FFFF00` | `0x00FFFF00` | match |
| `$0CEC` `VIA2` | `0x50F02000` | `0x50F02000` | match |
| `$0DDC` | `0x007FFFD4` | `0x03FFFFD4` | RAM-size, expected |
| `$0CB4` `MMUTbl` | `0x007FFFA6` | `0x007FFCEC` | RAM-size, expected |

(the last two differ only because MAME was run `-ramsize 8M` while the
board has 64 MB; they are not divergences.)

So the ROM correctly identified a Quadra 700 and loaded the correct
ProductInfo record, and *still* instantiated the IIci/IIsi video driver.
That rules out box misidentification and points the remaining search at the
**video-driver selection** itself.

### 2.3 Why this is consistent with "v1 boots this same SoC fine"

It is not that v1 has a correct PB6 and cpu040 a broken one — the RTL is
identical (§1.4). It is that **v1 never enters the RBV driver**, because it
never mis-selects it. Everything downstream of the mis-selection (the
aliased `+2` poll, the permanent PB6) is a *symptom*.

---

## 3. What to investigate next (this is where the real fix lives)

Steps 1 and 2 of the original plan are **done** — see §2.2a. The driver
identity is confirmed as RBV on hardware, and the box identification is
confirmed *correct*. So the remaining question is narrow:

> The ROM knows it is a Quadra 700 (`UnivInfoPtr = 0x4080390C`, matching
> the healthy machine byte-for-byte) and still instantiates
> `.Display_Video_Apple_RBV1` instead of `.Display_Video_Apple_DAFB`.
> What does the driver-selection path read that differs on our SoC?

Next steps:

1. **Trace the selection in MAME.** Breakpoint the point where the healthy
   machine commits to the DAFB driver — the `.Display_Video_Apple_DAFB`
   name string lands at RAM `0x6A83` on the healthy boot, so a MAME
   *watchpoint* on a write to `0x6A83` catches the instant of selection and
   its call stack. Use the §"Reproducing the MAME reference runs" recipe
   (with a control breakpoint; never `-debugger none`).
2. **Suspect the built-in-video sResource / declaration-ROM path.** The
   Q700's video is virtual NuBus slot `$F`. Whatever the Slot Manager reads
   there to pick the video family is the prime candidate — most likely a
   DAFB chip-ID/revision register that `rtl/mac/video.v` does not model, so
   the ROM falls through to the RBV family.
3. **Re-check the v1 comparison against this narrowed question.** `video.v`
   is shared between the two CPU targets, so if v1 truly loads DAFB on the
   same SoC, whatever the selection reads is being answered *differently*
   under cpu040 — e.g. an MMIO read ordering/size difference, or the
   already-tracked DAFB-adjacent issues in
   `docs/BUG_low_ram_corruption_post_dafb.md`. Confirm on a v1 boot which
   driver name is resident (search RAM for `.Display_Video_Apple_`), since
   that single fact separates "cpu040-specific" from "SoC-wide, and v1 just
   never calls the routine".

---

## 4. `via2_pb_in` — leave it alone

`rtl/soc/fpga_top_peripherals.vh:793-794` stays as-is. A defensive comment
citing the MAME source and this document has been added so the constant is
not "fixed" again.

Getting this wrong in the other direction is the dangerous case: inventing
a PB6 toggle would let phase 2 pass **spuriously**, driving the RBV driver
deeper into a machine it was never written for — writes at `+0x0A`, `+0x0B`,
`+0x10` that all alias onto VIA2 ORB, whose real bits are the DFAC audio
serial lines (`PB0` latch, `PB3` data, `PB4` clock) and `PB7` → VIA1 CA1.
That would silently corrupt the audio codec state and the 60.15 Hz VIA1
CA1 chain instead of stopping visibly. A permanent hang on a bit that
cannot move is strictly the safer failure.

---

## 5. Original (superseded) diagnosis — kept for the record

Everything below was the 2026-08-26 first-pass write-up. Its *measurements*
are all still valid and useful; only its conclusion is wrong.

### One-line summary (superseded)

RAM-resident Mac OS code at `0x0000a8c0` runs a two-phase handshake on
**VIA2 `vBufB` bit 6** (wait-for-set, then wait-for-clear).
`rtl/soc/fpga_top_peripherals.vh:794` ties `via2_pb_in = 8'hCF`, a
**constant**, so PB6 reads `1` forever. Phase 1 passes instantly; **phase 2
can never complete.**

### Measured evidence (live hardware, all captured at an effective halt)

The poll loop, read back coherently (`coherent-dump 0x0000a8b8`):

```
0000a8b8:  1010        moveb  %a0@,%d0        ; A0 = 0x50F02002
0000a8ba:  0800 0006   btst   #6,%d0
0000a8be:  67f8        beqs   0x0000a8b8      ; phase 1: spin until PB6 == 1   (passes instantly)
0000a8c0:  1010        moveb  %a0@,%d0
0000a8c2:  0800 0006   btst   #6,%d0
0000a8c6:  66f8        bnes   0x0000a8c0      ; phase 2: spin until PB6 == 0   <-- STUCK HERE FOREVER
```

`pc-trace` shows nothing but `0xa8c0 / 0xa8c2 / 0xa8c6` for the whole
32-entry ring. The core is **retiring normally** — this is a live spin, not
a wedge.

Architectural state at the halt (`regs`) — note how healthy it is:

```
PC  = 0x0000a8c2      A0  = 0x50f02002      SR   = 0x00002200  (S=1, IPL=2)
VBR = 0x00000000      A5  = 0x03feaa00      TC   = 0x0000c000  (MMU on, 8K)
SRP = 0x03feea00      A7  = 0x004008ea      CACR = 0x80008000  (I+D caches on)
```

VBR is 0 and 4-aligned, the MMU is enabled with a real SRP, both caches are
on, and A-line traps were dispatching correctly from many distinct ROM PCs
(`exc-ring`: `vec=0x0a` from `0x4080b674`, `0x40827964`, `0x40830298`, ...
all to handler `0x408099b0`). This is a genuinely deep, correct boot.

### The address decodes to VIA2 vBufB — verified, not assumed

`A0 = 0x50F02002`. Through `rtl/mac/glue.v`:

* `in_io_window` = (`addr[31:24] == 0x50`) → true
* `io_off = addr[23:0] & ~Q700_IO_MIRROR_MASK` = `0xF02002 & 0x03FFFF` = **`0x002002`**
* `io_via2_hit` = (`io_off >= 0x2000 && io_off < 0x4000`) → **true** (`glue.v:214`)
* VIA register index = `(0x2002 - 0x2000) >> 9` = **0** → `vBufB` (ORB/IRB)

(§1.3 above independently confirms this matches a healthy Q700 exactly.)

### PB6 is an INPUT, so the read returns `pb_in[6]` — verified on hardware

`via2.v:516`-style port read is `orb_rd = (orb & ddrb) | (pb_in & ~ddrb)`,
so which side wins depends on DDRB. Read live over JTAG:

| register | addr | value | meaning |
|---|---|---|---|
| `vDirB` (reg 2) | `0x50F02400` | `0x99` | `1001_1001` → **bit 6 = 0 = INPUT** |
| `vIFR` (reg 13) | `0x50F03A00` | `0x40` | T1 flag set, bit 7 (IRQ summary) clear |
| `vIER` (reg 14) | `0x50F03C00` | `0x80` | enable mask = `0x00` → **all VIA2 IRQs masked** |

(byte-wide peripheral; the JTAG 32-bit read replicates the byte across all
four lanes, hence `0x99999999` etc.)

So PB6 is an input → the read returns `via2_pb_in[6]`, which is
`8'hCF`[6] = **`1`**, permanently.

Note in passing that `vDirB = 0x99` is exactly the *Q700* Port B output
configuration — PB0/PB3/PB4 (DFAC latch/data/clock) plus PB7 (→ VIA1 CA1),
matching `quadrax00_state::via2_out_b()`. The VIA2 itself is being driven
correctly by the OS; only the code doing the *polling* is foreign.

### Nothing can break the loop

After `halt-release`, re-sampled 20 s later: `pc_live` still `0x0000a8c0`
and the **`exc-ring` head is unchanged (22)** — i.e. *zero* exceptions and
zero interrupts fired in that window, consistent with `vIER = 0x80` (all
VIA2 sources masked). There is no interrupt path that could flip the bit or
kick the OS out. **The hang is structural and permanent.**

### Root cause (superseded — see §1/§2)

`rtl/soc/fpga_top_peripherals.vh:794`:

```verilog
    // Q700/MAME reset straps: no slot IRQs, TM0A/TM1A high.
    wire [7:0]  via2_pb_in = 8'hCF;
```

~~This is a *reset strap value* that was never made dynamic.~~ It matches
MAME's `via2_in_b()` exactly and is correct. See §1.1.

## Reproduction

1. Program `build/vivado/fpga_top.bit` from the `movem-fix-build` worktree
   (build_id `0x8B370005`) and let the board boot undisturbed.
2. `halt-status` repeatedly. On the "deep boot" outcome (see the triage in
   `BUG_movem_misaligned_postinc_deadlock.md`) `pc_live` parks on
   `0x0000a8c0 / 0x0000a8c2`.
3. `halt` → `regs` → `coherent-dump 0x0000a8b8 8` reproduces everything above.
4. `coherent-dump 0x0000A233` to confirm the RBV driver identity (§3.1).

Note the boot outcome is **nondeterministic** (three distinct outcomes
observed on the identical bitstream — see the triage doc), so this may take
more than one boot to land on.

## Reproducing the MAME reference runs

```
mame macqd700 -rompath  <mame_q700_good>/roms \
     -cfg_directory <...>/cfg -nvram_directory <...>/nv -diff_directory <...>/df \
     -video none -sound none -nothrottle -ramsize 8M \
     -hard <mame_q700_good>/hd753.chd \
     -debug -debugscript probe.dbg -seconds_to_run 120
```

with `probe.dbg` setting `bpset <addr>,1,{trace f,0,noloop;tracelog "...";trace off;g}`
per address. **Do not pass `-debugger none`** — it silently arms nothing and
every breakpoint reports a false negative. Always include one breakpoint
you know fires (`0x40809BE0`) as a control.

## Related

* `docs/BUG_movem_misaligned_postinc_deadlock.md` — the "Post-fix status"
  section carries the full three-outcome boot triage this was found in.
* `rtl/soc/fpga_top_peripherals.vh:766-799` — the Port A comment block. Note
  that Port A's fix was legitimate (MAME derives Port A from live state);
  Port B is *not* the same case.
