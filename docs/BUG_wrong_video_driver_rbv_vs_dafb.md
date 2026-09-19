# BUG: cpu040 boots the RBV video driver instead of DAFB — selection mechanism mapped, root cause narrowed

**Status**: OPEN. Mechanism fully mapped and the divergence measured on both
sides. **STALE as of 2026-08-30 (see `docs/BUG_calibration_word_misplaced_0d00.md`
Parts 36/39/40/41 for current status)**: §5's TTR/`mmuEnable` candidate below was
DISPROVEN on real hardware (Part 36, `ceebdee7` shipped and live, no behavior
change) and independently RE-DISPROVEN by a directly causal capture (Part 40:
DTT0/DTT1/ITT0/ITT1 all already zero at the exact commit-write instruction that
causes this bug). Part 40 caught the actual divergence red-handed with a real
`break-pc` capture at the generic-installer commit write (`0x4080b9cc`): the
gate/commit pipeline itself is bit-for-bit correct and matches MAME; the bug is
in an earlier, still-unlocated candidate-selection/staging step that puts an
RBV-class sResource's data where DAFB's should be. **Part 41 locates and maps
that candidate-selection step end-to-end**: a `csCode 21` retry loop
(`0x40801000`-`0x4080104c`) walks video-class declaration-ROM candidates one at
a time through a shared 6-check hardware-presence probe (`0x40800bf0`-
`0x40800c9e`); real hardware caught cpu040 constructing a fully-formed,
correctly-tagged RBV candidate (spSlot=0x00, spID=0x81, the numerically-first
RBV1 entry) right at that probe's own entry point — proving forward boot
progress past the 2026-08-27 baseline where this code was never reached at all.
The probe's own pass/fail OUTCOME for that candidate was not captured live
despite three real-hardware attempts. **Still no fix shipped** — the search
space is now one specific ~30-instruction routine with 6 identified internal
checks, down from "somewhere upstream, unlocated" — see Part 41 §9 for ranked
next steps.

**Parent**: `docs/BUG_via2_pb6_constant_boot_hang.md`. That document proved the
VIA2 PB6 hang is a *symptom* of the wrong video driver. This document takes
over from its §3 "what to investigate next" and **corrects its §1.5**.

**Severity**: HIGH — terminal blocker on the deepest cpu040 boot outcome.

---

## 0. What changed vs the parent document

The parent's leading hypothesis was "the SoC does not model the slot-`$F`
declaration ROM / DAFB chip ID that the Slot Manager needs". **That is wrong
on two counts**, both now measured:

1. There is no slot-`$F` declaration ROM to model. The Quadra 700's built-in
   video is a **slot-0 pseudo-slot** whose declaration ROM lives **inside the
   Mac ROM image itself** (`files/420dbff3.rom`), not on any bus. MAME's own
   `macquadra700.cpp` maps nothing at `0xFF000000` either, and it boots.
2. The SoC's DAFB register window is **present and answering correctly** on the
   live board (§3.2).

The user has separately confirmed that **v1 boots this same SoC and selects
DAFB correctly**, so the shared SoC RTL is not at fault. The divergence is
cpu040-side.

---

## 1. How the ROM actually selects a built-in video driver

Reverse-engineered from `files/420dbff3.rom` (Q700 ROM `420dbff3`, 1 MiB at
`0x40800000`, mirrored throughout `0x4xxxxxxx`).

### 1.1 The in-ROM declaration ROM

A real NuBus format block sits at the very top of the ROM image:

```
0x408FFFEC: 00 FF 59 04   DirectoryOffset  (-0xA6FC as a signed 24-bit offset)
0x408FFFF0: 00 00 A7 10   Length = 0xA710
0x408FFFF4: B1 9F D7 40   CRC
0x408FFFF8: 01            RevisionLevel
0x408FFFF9: 01            Format
0x408FFFFA: 5A 93 2B C7   TestPattern
0x408FFFFE: 00            Reserved
0x408FFFFF: 0F            ByteLanes (all four lanes)
```

⇒ **sResource directory at `0x408F58F0`**, declaration ROM spanning
`0x408F58F0..0x408FFFFF`.

(`0x5A932BC7` appears 4× in the ROM; the other three are code immediates —
`0x40800C7C` `move.l #$5A932BC7,($0DB0).w`, `0x4080230A` `cmpi.l`, and
`0x40805F42` `cmpi.l #$5A932BC7,14(a2)`, the Slot Manager's own format-block
validator.)

### 1.2 What is in it — 76 sResources

Board sResources (spID `0x01`..`0x15`): `Macintosh II`, `IIx`, `IIcx`,
`SE/30 Built-In Video`, `IIci Built-In Video`, `IIfx`, and
`Macintosh A`…`G Built-In Video`.

Video sResources, keyed **by spID**, all `cat=0003 typ=0001 drvSW=0001`:

| driver | `drvHW` | spIDs |
|---|---|---|
| `Display_Video_Apple_RBV1` | `0x0018` | `81 82 86 87 89 8A 8E 8F 91 92 96 97 9A 9E 9F` |
| `Display_Video_Apple_V8` | `0x001A` | `A2 A3 A6 AA AB AE B2 B6 BA` |
| `Display_Video_Apple_DAFB` | `0x001C` | `C0`–`D7`, `D9 DB DD DF EA EB ED EF` |
| `Display_Video_Apple_TIM` / `Apollo` / `DBLite` | | `E0` / `E1` / `E2` |

plus `CPU_68030_\_MacIIFamily` (`F0`), `CPU_68040` (`F1`),
`Network_Ethernet_Apple_Sonic` (`FD`).

**`0x81` is the numerically first video sResource in the directory** — i.e. the
one you land on if a search fails and the code falls back to "first match".

### 1.3 The selection is committed by the video sResource's PrimaryInit

Traced live in MAME (recipes in §6):

* At **t≈4 s** the Slot Manager runs the DAFB sResource's `sPrimaryInit`,
  relocated to RAM at `~0x00005F1C`. It programs the whole DAFB: CLUT
  (`+0x300..+0x3F0`), base/stride/config (`+0x000..+0x010`), SWATCH timing
  (`+0x100`, `+0x124..+0x168`), RAMDAC (`+0x200`, `+0x220`), then runs the
  **extended monitor-sense handshake** — repeated `W +0x1C` (drive sense
  lines) / `R +0x1C` (read them back), interleaved with 4× `R +0x2C` delay
  reads. Registers at the time: `A3 = 0xF9800000`, `A4 = 0x4080390C`
  (`UnivInfoPtr`).
* At **t≈5 s** the driver is installed. ROM `0x4080B9C6..0x4080B9E4`:

  ```
  4080b9bc: subaw  #56,%sp            ; 56-byte spBlock on the stack
  4080b9c0: moveal %sp,%a4
  4080b9c2: bsrw   0x4080b708         ; fill spBlock from the install descriptor
  4080b9c6: moveb  %a4@(49),%a1@(40)  ; spSlot -> DCE+0x28  (dCtlSlot)
  4080b9cc: moveb  %a4@(50),%a1@(41)  ; spID   -> DCE+0x29  (dCtlSlotId)
  4080b9d2: moveb  %a4@(51),%a1@(50)  ; spExtDev -> DCE+0x32
  4080b9d8: clrl   %a1@(42)           ; dCtlDevBase = 0
  4080b9dc: moveal %a4,%a0
  4080b9de: moveq  #27,%d0
  4080b9e0: a06e                      ; _SlotManager sel 27 (find device base)
  4080b9e2: bnes   0x4080b9e8         ;   on FAILURE dCtlDevBase stays 0
  4080b9e4: movel  %a0@,%a1@(42)      ; dCtlDevBase = result
  ```

So the observable, decisive state is the video driver's **DCE**:
`dCtlSlotId` = the chosen spID, `dCtlDevBase` = the resolved device base.

---

## 2. The measured divergence

Both machines put the built-in video driver at **unit 48, refnum −49
(`dCtlRefNum = 0xFFCF`), `dCtlSlot = 0x00`** (slot-0 pseudo-slot).

| | healthy MAME (Finder at t=30 s) | cpu040 board (parked at the hang) |
|---|---|---|
| unit-table handle | `0x00002128` | `0x00002124` |
| DCE | `0x00005B30` | `0x00009A40` |
| `dCtlSlotId` (spID) | **`0xDD`** → `Display_Video_Apple_DAFB` | **`0x81`** → `Display_Video_Apple_RBV1` |
| `dCtlDevBase` | **`0xF9001420`** | **`0x00000000`** |
| `$0824 ScrnBase` | `0xF900D428` | **`0x00000000`** |
| `$08A8 DeviceList` | `0x0000211C` | **`0x00000000`** |

Board DCE raw (`coherent-dump 0x00009A40`):

```
0x9A40 = 0x00002128   dCtlDriver (handle -> 0x0000A220, name at 0xA232)
0x9A44 = 0x4c600000   dCtlFlags = 0x4C60
0x9A54 = 0x00002120   dCtlStorage
0x9A58 = 0xffcf0000   dCtlRefNum = -49
0x9A68 = 0x00810000   dCtlSlot = 0x00, dCtlSlotId = 0x81   <-- RBV
0x9A6C = 0x00000000   dCtlDevBase = 0                       <-- sel-27 FAILED
```

Board register state at the hang (`regs`): `PC = 0x0000a8c2`,
`A1 = 0x80009a40` (that very DCE), `A4 = 0x40809ae6` (ROM), `SR = 0x2208`,
`TC = 0xc000`. i.e. **the board is hung inside the video driver's `Open`,
called with the RBV DCE** — the last step of the chain above.

So: the ROM did not *fail* to select a video driver. It selected the
numerically-first video sResource in the declaration ROM (`0x81`, RBV) and then
could not resolve a device base for it.

---

## 3. Ruled out, with measurements

### 3.1 Box identification — correct (already in the parent doc)

`UnivInfoPtr = 0x4080390C` on both. Decoding that ProductInfo record confirms
it is the right one:

```
+0x00 0xFFFFFC5C  DecoderInfo  -> 0x40803568  (contains 0x50F02000 = VIA2,
                                  matching low-mem $0CEC on both machines)
+0x04 0x000002AC  RamInfo
+0x08 0x0000033C  VideoInfo    -> 0x40803C48  (video bases 0xF9001000 /
                                  0xF9000E00 / 0xF9001020 / 0xF9001400)
+0x0C 0x0000041C  NuBusInfo
+0x18 0x05A0183F  -> low-mem $0DD0 (matches board)
+0x1C 0x00000900  -> low-mem $0DD4 (matches board)
```

### 3.2 The DAFB register window on the board — ALIVE and CORRECT

```
tools/jt.sh "dump-mem 0xF980002C 1"  ->  0x00000200   ==  DAFB_VERSION_BITS
tools/jt.sh "mon-sense"              ->  0x06 Mac Hi-Res 12-14" 640x480
```

`0x200` is `rtl/mac/video.v:343 DAFB_VERSION_BITS`, OR'd unconditionally into
every `REG_DAFB_TEST` (`+0x2C`) read — so a `0x200` readback proves the shim is
reachable, out of reset, and answering. The monitor-sense CSR
(`cpu_mon_sense`, `OFF_MON_SENSE 0x0005C`) holds its POR value `0x06`, the
same value `cpu_stub` drives, so sense is not misconfigured either.

> **GOTCHA that cost time here**: `dump-mem <addr> <n>` with `n > 1` issues a
> *burst*. Per `rtl/soc/axi_xbar.v:157-166`, **S4 (DAFB) and S1/S2/S5 are
> "lite-only"** — a burst decoded onto them is *never forwarded* and is
> answered locally with `SLVERR` on every beat. A burst dump of the DAFB window
> therefore returns **all zeros and looks exactly like a dead slave**. Always
> read DAFB registers one word at a time.

### 3.3 The DAFB PrimaryInit never ran on the board

```
tools/jt.sh "dump-mem 0xF9800100 2"  ->  0x00000000 0x00000000   (SWATCH)
tools/jt.sh "dump-mem 0xF9800220 1"  ->  0x00000000              (RAMDAC PBCTRL)
```

On the healthy machine the PrimaryInit writes `+0x100 = 0x0FF2`, `+0x220 =
0x80` then `0x06`, and the full `+0x124..+0x168` timing block. On the board
every one of them is still at reset. **The DAFB sResource's PrimaryInit was
never executed**, which is exactly why spID stayed on the first video
sResource.

### 3.4 Misaligned-stack hypothesis — NOT supported (negative result)

The coordinator's leading suspicion was a third instance of the
misaligned-load-reconstruction family (`e7ee618a`, and the still-open Outcome-B
case). It was tested and **did not hold up**:

* MAME does report an **odd** supervisor stack inside the DAFB PrimaryInit —
  `SP = ISP = A7 = 0x0017FEC7`, `SR = 0x2700`, `A6 = 0x0017FF12`.
* But read/write taps over `0x0017FEA0..0x0017FF1F` and over the whole
  `0x00170000..0x0017FFFF` stack page, armed from the PrimaryInit's first
  SWATCH write onward, recorded **zero** accesses — misaligned or otherwise.

So although this path does run on an odd `A7`, it does not actually perform
misaligned multi-byte stack traffic there. Recorded so the lead is not
re-opened without new evidence.

---

## 4. **Correction to `BUG_via2_pb6_constant_boot_hang.md` §1.5**

That section dismissed the TTR/cache-inhibition question with:

> "MAME ground truth at the same point in boot — `TC=0xC000`,
> **`DTT0=DTT1=ITT0=ITT1=0`** … so the 'C3' TTR gap … is not on this path at
> all."

**That snapshot was taken at the wrong phase of boot.** The TTRs are only zero
*after* the ROM has finished with them. Sampled across a healthy MAME boot:

| t | `TC` | `DTT0` | `DTT1` | PC |
|---|---|---|---|---|
| 1 s | `0x00000000` | `0x807FC040` | `0x500FC040` | `0x408472A4` |
| 2 s | `0x00000000` | `0x807FC040` | `0x500FC040` | `0x40847334` |
| 3 s | `0x00000000` | `0x807FC040` | `0x500FC040` | `0x4084731E` |
| 4 s | `0x00000000` | `0x807FC040` | `0x500FC040` | `0x40847AC0` |
| ~4 s (inside DAFB PrimaryInit) | `0x0000C000` | `0xF900C060` | `0x807FC040` | `0x00005FAE` |
| 5 s | `0x0000C000` | `0` | `0` | `0x40898AE2` |
| ≥6 s | `0x0000C000` | `0` | `0` | — |

Decoded (MC68040 UM §3.1.2, TTR = base[31:24] / mask[23:16] / E[15] / S[14:13]
/ CM[6:5]):

* `0x807FC040` — base `0x80`, mask `0x7F` ⇒ covers **`0x80000000..0xFFFFFFFF`**,
  E=1, S=`1x` (both), **CM=`10` = cache-inhibited, serialized**.
* `0x500FC040` — base `0x50`, mask `0x0F` ⇒ covers **`0x50000000..0x5FFFFFFF`**,
  E=1, **CM=`10` = cache-inhibited** — i.e. the whole VIA/SCC/SCSI/ASC/SWIM
  peripheral window.
* `0xF900C060` — base `0xF9`, mask `0x00` ⇒ covers exactly **`0xF9xxxxxx`**
  (DAFB VRAM + registers), E=1, **CM=`11` = cache-inhibited, nonserialized**.

So for the whole `TC=0` early-ROM phase, the Q700 ROM makes **every MMIO region
cache-inhibited using the TTRs alone, with the MMU disabled**.

---

## 5. Leading root-cause candidate (concrete, not yet proven)

**cpu040 ignores the TTRs whenever the MMU is disabled**, and forces every data
access to `WRITETHROUGH`:

`cpu040/src/main/scala/m68k040/mmu/DtlbPlugin.scala:246-252`

```scala
when(!mmuEnable) {
  rspValid             := True
  rspPayload.ppn       := _req.payload.vpn
  rspPayload.cacheMode := CacheMode.WRITETHROUGH   // <-- TTRs never consulted
  ...
```

The DTT0/DTT1 match immediately below it is gated `mmuEnable && TtMatch.hit(...)`
(`DtlbPlugin.scala:137-139`), and the I-side is the same
(`ItlbPlugin.scala:124-126`). This is the repo's already-known "C3 TTR gap".

Why that matters *here*, and why it is cpu040-specific:

1. `WRITETHROUGH` is **cacheable** in this D-cache —
   `DcachePlugin.scala:521` `val ldS1Cacheable = ldS1Cmode =/= CacheMode.INHIBITED`.
   Only `INHIBITED` skips line allocation (`DcachePlugin.scala:335`) and takes
   the single-beat `MmioCover` sub-transaction path.
2. So with `TC=0`, a load from `0x50F02000` (VIA) or `0xF9800000` (DAFB)
   **allocates a cache line** and issues a **multi-beat refill burst** on
   `axi_d`.
3. `rtl/soc/axi_xbar.v:157-166`: S1 (peripheral bus/GLUE), S2 (DMA cfg),
   **S4 (DAFB shim)** and S5 are single-beat-only. "A burst (`awlen`/`arlen`
   > 0) decoded onto a lite-only slave is **NOT forwarded** … reads are
   answered … with every beat `RRESP=SLVERR`."
4. ⇒ during the entire `TC=0` phase, cpu040's MMIO reads are (a) served from a
   stale cache line, and/or (b) turned into bus errors. v1's LSU does not have
   this shape, which is consistent with v1 booting the same SoC fine.

This also offers a single explanation for the **non-deterministic three-outcome
boot** documented in `BUG_movem_misaligned_postinc_deadlock.md` — an
intermittently-wrong early-boot MMIO path.

### What is still missing for proof

The DAFB PrimaryInit itself runs at `TC=0xC000` with `DTT0=0xF900C060`, and
cpu040 *does* honour DTT0/DTT1 in that state (`TtMatch.hit` and
`TtMatch.cacheMode` were both read and are correct: `0xC060` ⇒ E=1, S=`10`,
CM=`11` ⇒ `INHIBITED`). So the gap does **not** directly break the PrimaryInit
itself; it breaks the `TC=0` phase that precedes it — the phase in which the
ROM does its earliest hardware probing and its declaration-ROM CRC
(`0x40806074`, `0x40847518`; the whole `0xA710`-byte block).

**The unproven link is: which specific `TC=0`-phase MMIO access determines that
the DAFB sResource is kept, and does it read wrong under cpu040?**

---

## 6. Next steps, in priority order

1. **Sim proof of the gap (blocked this session on the machine's JVM/RAM
   budget — a Vivado implementation run was live).** Add a focused spec:
   MMU disabled, `DTT0 = 0x807FC040`, load from `0xF9800000` ⇒ assert the
   D-cache issues a *single-beat, non-allocating* access, not a line refill.
   This is a self-contained, testable statement of §5 and is worth doing
   independent of this bug.
2. **Fix the gap**: consult DTT0/DTT1 in `DtlbPlugin` (and ITT0/ITT1 in
   `ItlbPlugin`) even when `mmuEnable` is low, per MC68040 UM §3.1.2 (TTRs are
   *always* active; `TC[E]` gates only the table walk). Keep the identity
   translation; change only `cacheMode`. Note this deliberately changes
   MMU-disabled behaviour that many existing tests pin — the comment at
   `DtlbPlugin.scala:137-139` says the gating exists exactly to keep those
   "bit-for-bit unchanged", so expect test churn and re-baseline consciously.
3. **Hardware A/B**: arm a data watchpoint on the DAFB window and reboot —
   `reset hold` → `watch 0 0xF9800100 rw` → `reset release`. Watchpoint config
   survives CPU reset. If it never fires, the DAFB PrimaryInit truly never
   starts; if it fires and then the selection still lands on `0x81`, the
   divergence is inside the PrimaryInit. (Not done here: the board was parked
   at the reproduced hang and the boot outcome is non-deterministic, so a reset
   risks losing the repro without a guaranteed replacement.)
4. Only after 1–3: consider whether anything in `rtl/mac/video.v` needs to
   change. On present evidence **it does not** — see §3.2.

---

## 7. Reproduction recipes

### 7.1 Board (`tools/jt.sh`, which flocks — safe alongside a peer REPL)

```
tools/jt.sh "halt"
tools/jt.sh "coherent-dump 0x00000118 8"      # $011C UTableBase
tools/jt.sh "coherent-dump 0x00009410 64"     # unit table; entry 48 = video
tools/jt.sh "coherent-dump 0x00002120 8"      # handle -> DCE
tools/jt.sh "coherent-dump 0x00009A40 14"     # the DCE: +0x29 = spID
tools/jt.sh "dump-mem 0xF980002C 1"           # ONE word at a time (see §3.2)
tools/jt.sh "mon-sense"
tools/jt.sh "halt-release"
```

### 7.2 MAME reference — Lua taps, not `-debugscript`

`-debug` with an offscreen Qt debugger hangs headless, and `-debugger none`
silently arms nothing (the parent doc's gotcha). **Use `-autoboot_script` with
Lua memory taps instead** — no debugger needed, output goes straight to stdout:

```lua
local sp = manager.machine.devices[":maincpu"].spaces["program"]
sp:install_read_tap (start, endaddr, "name", function(offset, data, mask) ... end)
sp:install_write_tap(start, endaddr, "name", function(offset, data, mask) ... end)
```

Notes learned the hard way:

* `endaddr` must have its low bits **set** (`0x408F5A23`, not `0x408F5A20`).
* Read low-memory globals with `sp:readv_u32` (virtual); `read_u32` is the raw
  space. `read_log_*` does not exist in 0.285 — the translated accessors are
  `readv_*` / `writev_*`.
* Handles carry Memory-Manager flag bits in the top byte: mask master pointers
  with `& 0x00FFFFFF` before dereferencing.
* The declaration ROM is CRC'd twice at boot; both loops swamp any tap over
  ROM. Filter `pc & 0xFFFFFFE0` ∈ {`0x40847500`, `0x40806040`, `0x40806060`,
  `0x40806080`} — but note `0x40806C7A` is the byte-lane *reader*, so filtering
  it hides every legitimate declaration-ROM access too.

Runner: `/home/qwertyoruiop/mame_q700_good/run.sh -autoboot_script <x.lua>
-seconds_to_run <n>` (Finder at t≈30 s, ~400 % speed).

---

## 8. Related

* `docs/BUG_via2_pb6_constant_boot_hang.md` — the symptom, and the §1.5
  correction above.
* `docs/BUG_movem_misaligned_postinc_deadlock.md` — the three-outcome boot
  triage; §5 here proposes a common cause for its non-determinism.
* `docs/BUG_low_ram_corruption_post_dafb.md` — an older boot (legacy core) that
  *did* reach DAFB checkerboard rendering, i.e. independent evidence that the
  SoC's DAFB path works when the CPU drives it correctly.
* `rtl/soc/axi_xbar.v:140-166` — the lite-only-slave burst rule.
* `rtl/mac/video.v:230-260, 408-466, 1290-1300, 343` — monitor sense and the
  version register.
