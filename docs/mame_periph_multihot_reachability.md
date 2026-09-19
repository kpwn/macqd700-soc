# Multi-hot-strobe peripheral writes — MAME System 7 reachability measurement

Date: 2026-08-18

Answers **SI11 / M1 / SI-OPEN-3** of
`docs/superpowers/specs/2026-08-18-v1-shared-infra-fixes-design.md` §2.7: *does real
Mac ROM or System 7 driver code ever issue a multi-hot-strobe (WORD/LONG) write to the
peripheral slots `rtl/soc/peripheral_bus.v` decodes?*

Everything below is measured on this host. No claim here is inherited from an existing
in-tree assertion; §2.7.1 of the spec lists three such assertions and all three are
unsourced. Two independent instruments were used and they agree exactly.

**Headline:** the answer is **yes for three slots and no for the rest**, and the split does
not fall where the repository assumed.

| | |
|---|---|
| **Multi-hot writes DO occur** | `ORWELL` (LONG, ROM), `SONIC` (LONG, System-7 driver), `ASC` (LONG, System-7 Sound Manager), `SCSI` pseudo-DMA shim (WORD, ROM + System-7 driver) |
| **Multi-hot writes DO NOT occur** | `VIA1`, `VIA2`, `SCC`, `IWM`/SWIM, `SCSI` **register** page — 494,785 writes observed, zero multi-hot |
| **No traffic at all** | `ENET`, `ADBINJ` |

Of the four positives, two (`ASC`, `SCSI`-DMA) are the already-serialized paths the spec's
SI9 says to leave alone; the two that land on **unfixed** slots are **`ORWELL` and `SONIC`**,
both in the Fix A (byte-granular) family. **No hit landed on a Fix B (strided) slot, and no
hit landed on SCC.**

---

## 1. Instruments

### 1.1 New tool — `tools/mame_periph_strobe_capture.lua`

The existing VIA1 golden-trace tool could not be used, exactly as the spec predicted: its
lane loop `break`s after the first hot byte lane, so it cannot represent a multi-lane
access at all. A new tool was written rather than patched, because the old one's output
format is a lockstep golden and must not change.

The new tool installs **one** program-space write tap, classifies every write with a Lua
transcription of `peripheral_bus.v`'s own `decode_slot()` (including the
`addr[23:18]` Q700 mirror mask), and records for each write the **full 32-bit `mem_mask`**,
the hot-byte-lane popcount, the data, and the m68k PC. Nothing is dropped and nothing
breaks out of a lane loop. It emits:

* a per-slot × hot-lane-count histogram,
* a per-slot `mem_mask` histogram,
* a per-slot **device-register** coverage table, using `peripheral_bus.v`'s own
  device-local address expressions (`addr[12:9]` for VIA/IWM, `{addr[5:4],addr[1],addr[2]}`
  for SCC, `addr[7:4]` for SCSI regs, …) so a negative result can be qualified by how much
  of each device the workload actually touched,
* every multi-hot row in full, with the writer PC.

`tools/mame_periph_activity.lua` (also new) is an optional post-boot activity driver:
continuous ADB mouse motion, desktop clicks, and four floppy insert/eject cycles, so the
capture covers live driver code rather than only the ROM cold path.

### 1.2 Independent cross-check — MAME debugger watchpoints

Taps are a MAME memory-system feature; to avoid trusting one mechanism, the same question
was asked a second way, through the CPU-side debugger:

```
wpset 50000000,1000000,w,wpsize>8,{printf "MH %08X sz=%d pc=%08X d=%08X",wpaddr,wpsize,pc,wpdata;g}
wpset f9000000,1000000,w,wpsize>8,{printf "CTL %08X sz=%d",wpaddr,wpsize;g}
```

`wpsize` is in **bits** (a byte write reports `sz=8`), which is the *architectural* access
width reported by the CPU, independent of any mask/handler decomposition. The second
watchpoint is a positive control over the DAFB/VRAM aperture.

### 1.3 Positive controls (three, all fired)

A negative result is only worth as much as the proof that the instrument could have shown a
positive. Three independent positives fired in the same sessions:

| Control | Result |
|---|---|
| DAFB/VRAM aperture — QuickDraw LONG stores | 474,643 (tap) / 483,052 (watchpoint) 32-bit writes |
| SCSI pseudo-DMA shim — the `move.w` documented by commit `eba8aa2` | 41,939 16-bit writes, ROM **and** RAM driver PCs |
| ASC FIFOs — the multi-byte ASC family documented by commit `9468831` | 3,984 32-bit writes from the 7.5.3 Sound Manager |

---

## 2. Instrument defects found (both real, both affect existing tools)

### 2.1 An unreferenced tap handle is garbage-collected and the capture silently truncates

`install_read_tap` / `install_write_tap` return a `memory_passthrough_handler`. **If Lua
collects that object, MAME removes the tap.** There is no error and no warning — the trace
just stops, at a point determined by when the incremental collector happened to run.

Measured directly, MAME 0.285 / `macqd700`, identical 8 s ROM boot, one narrow write tap,
counting VIA1 writes:

| variant | VIA1 writes |
|---|---:|
| handle stored in a global | **17,248** |
| handle dropped, then `collectgarbage("collect")` | **0** |

This is not a range-width effect: narrow (`0x5000_0000..0x50FF_FFFF`), two-narrow, and
whole-space taps all report exactly 17,248 when the handle is held. The first revision of
the new tool did not hold the handle and reported 2,212 VIA1 writes for that same scenario —
an arbitrary prefix. That mis-measurement was caught only because the numbers were
cross-checked against a second instrument; it is recorded here because it nearly produced a
confidently wrong report.

**Every existing capture tool in `tools/` has this defect**: `mame_via1_capture.lua`,
`mame_axi_capture.lua`, `mame_iwm_capture.lua`, `mame_asc_capture.lua`,
`mame_dafb_capture.lua`. (`mame_scsi96_capture.lua` is the exception — it accumulates its
handles into a `taps[]` table and is therefore safe.) Warning comments were added to the
three the spec cites; **the tools themselves were not changed**, because lengthening a
golden trace changes lockstep vectors and that is the owner's call.

**Consequence for the spec.** §2.7.3's IWM/SWIM evidence
(`tb/vectors/swim_mame_q700_753.csv`, 869 events, "zero duplicate timestamps") was produced
by `mame_iwm_capture.lua` and is therefore **possibly a truncated prefix** — weaker than the
spec already rated it. It is superseded by §4 below, which is a retained-handle measurement.

### 2.2 `tools/mame_via1_capture.lua` is structurally blind to multi-lane accesses

Confirmed as described in the spec (`:133-142`, `:154-160`). Its conclusion happens to be
**true for VIA1** — see §4 — but it is **false for other slots in the same aperture**
(`ORWELL`, `SONIC`, `ASC`, SCSI-DMA all take multi-lane writes), so the lane loop must not be
copied to any other device.

---

## 3. Capture methodology

Host MAME `0.285` (`/usr/games/mame`), machine `macqd700`, ROM `files/420dbff3.rom`
(+ `342s0440-b.bin` for `adbmodem`), **unpatched** — no `mame-fastdiag` / `chime-skip`
patch stack, since the question is about the real boot path.

Disk images (raw, renamed to `.hdv`; MAME's `harddisk` device rejects `.hda`):

* `HD0-OpenRetroSCSI-7.0.1-500M.hda` — **the configuration v1 is deployed booting**
* `HD0-OpenRetroSCSI-7.5.3.hda` — second data point, AppleTalk active by default

```
MAME_STROBE_OUT=<csv> \
MAME_STROBE_ACTIVITY=tools/mame_periph_activity.lua \
MAME_ACTIVITY_FLOPPY=<blank 1.44M img> MAME_ACTIVITY_START=25 \
xvfb-run -a mame -rompath <rp> macqd700 -hard <disk>.hdv \
  -video bgfx -window -sound none -nothrottle -skip_gameinfo \
  -autoboot_delay 0 -seconds_to_run 130 \
  -autoboot_script tools/mame_periph_strobe_capture.lua
```

`-video bgfx` under `xvfb-run` is required: `-video none` runs, but `-video bgfx` without an
X server aborts BGFX init. Both OSes reached the Finder (snapshots retained during the runs;
the 7.0.1 desktop shows the mounted *OpenRetroSCSI 7.0.1* volume window, and the floppy
insert produces the Finder's *"This is not a Macintosh disk"* dialog — i.e. the `.Sony`
driver really ran).

Runs used for the verdict:

| run | OS | emulated | instrument | activity |
|---|---|---:|---|---|
| `c701` | 7.0.1 | 130 s | tap | boot → Finder, mouse, 4 floppy cycles |
| `c753` | 7.5.3 | 130 s | tap | boot → Finder, mouse, 4 floppy cycles |
| `wp701` | 7.0.1 | 130 s | debugger watchpoint | same |
| `wp753` | 7.5.3 | 130 s | debugger watchpoint | same |
| `t3` | 7.0.1 | 30 s | tap | bare boot, no activity |

---

## 4. Result — per-slot write histogram

Aggregate of `c701` + `c753` (260 emulated seconds, two OS versions, 1,695,109 peripheral
writes classified (794,292 + 900,817)). "hot" = number of asserted byte lanes in the beat.

| slot | family (spec §2.3) | writes | hot=1 | **hot=2** | **hot=4** | verdict |
|---|---|---:|---:|---:|---:|---|
| `VIA1` | B — strided | 219,273 | 219,273 | 0 | 0 | **not reached** |
| `VIA2` | B — strided | 75,064 | 75,064 | 0 | 0 | **not reached** |
| `IWM`/SWIM | B — strided | 2,534 | 2,534 | 0 | 0 | **not reached** |
| `SCSI` regs | B — strided | 139,408 | 139,408 | 0 | 0 | **not reached** |
| `SCC` | §2.6 special | 58,506 | 58,506 | 0 | 0 | **not reached** |
| `ENET` | A — byte-granular | **0** | 0 | 0 | 0 | no traffic at all |
| `ADBINJ` | A — byte-granular | **0** | 0 | 0 | 0 | not a Mac device (§5) |
| `ORWELL` | A — byte-granular | 186 | 4 | 0 | **182** | **REACHED** |
| `SONIC` | A — byte-granular | 8 | 0 | 0 | **8** | **REACHED** |
| `ASC` | A — already serialized | 279,524 | 275,540 | 0 | **3,984** | reached (SI9, no change) |
| `SCSI` DMA shim | already serialized | 43,339 | 1,400 | **41,939** | 0 | reached (SI9, no change) |

No `hot=3` beat was observed anywhere. No write was ever decoded to a `FAULT_*` window,
i.e. the Lua decode and `peripheral_bus.v` agreed on every one of the 1.69 M writes.

### 4.1 Independent watchpoint confirmation

The debugger (architectural `wpsize`, no taps involved) found multi-byte writes at exactly
these addresses and nowhere else:

| run | address | `wpsize` | count | slot |
|---|---|---:|---:|---|
| `wp701` | `50F4F100` | 16 | 1,536 | SCSI DMA shim |
| `wp701` | `5000E000..5000E0B8` | 32 | 91 | ORWELL |
| `wp701` | `50F0A000`,`50F0A010`,`50F0A094` | 32 | 4 | SONIC |
| `wp753` | `50F4F100` + `50F0F100` | 16 | 37,344 + 3,059 = 40,403 | SCSI DMA shim |
| `wp753` | `50F14000`,`50F14400` | 32 | 3,984 | ASC FIFO A/B |
| `wp753` | `5000E000..5000E0B8` | 32 | 91 | ORWELL |
| `wp753` | `50F0A000`,`50F0A010`,`50F0A094` | 32 | 4 | SONIC |

Every count matches the tap's `hot=2`/`hot=4` column exactly (1,536 / 40,403 / 3,984 / 91 /
4). Two mechanisms, two OS versions, identical answer.

Note `50F0F100` is the pseudo-DMA port reached through a *different* Q700 mirror than
`50F4F100`; it is **not** the SCSI register page (`…0F000..0F0FF`), which took zero
multi-byte writes.

### 4.2 Register coverage behind the negatives

A "zero multi-hot" result is only as strong as how much of each device was exercised.
Distinct device-register indices written (index = `peripheral_bus.v`'s own expression):

| slot | registers written | note |
|---|---|---|
| `VIA1` | 13 of 16 (`0,2,3,4,5,8,9,10,11,12,13,14,15`) | ORB/DDR/timers/SR/ACR/PCR/IFR/IER all hit |
| `VIA2` | 10 of 16 (`0,2,3,4,5,11,12,13,14,15`) | |
| `SCSI` regs | 11 of 16 (`0,1,2,3,4,5,7,8,9,11,12`) | 53C96 command/target/xfer path |
| `IWM`/SWIM | 7 of 16 (`2,3,4,5,6,7,15`) | `.Sony` read + probe path |
| `SCC` | 2 (`8`,`9`) | = offsets `0x20`/`0x24`, i.e. exactly where all real SCC traffic goes, corroborating `peripheral_bus.v:430-436` |

The SCC negative is therefore narrow in register span but complete in traffic span: 58,506
writes, all of them to the two registers the OS actually uses.

---

## 5. Per-family verdict, with evidence

### VIA1 — NOT REACHED
219,273 writes across two OS versions and 13 of 16 registers, **100 % `mem_mask` =
`0xFF000000`**, watchpoint `sz=8` for every one. The in-tree assumption is, for this one
slot, correct — but it was correct by luck, not by measurement, and the instrument that
"supported" it could not have contradicted it.

### VIA2 — NOT REACHED
75,064 writes, 100 % single-byte. Two masks appear (`0xFF000000`, `0x000000FF`) — both
one-hot; the second is simply a byte at an odd offset within the 32-bit beat.

### IWM / SWIM — NOT REACHED
2,534 writes, 100 % single-byte, over 7 registers, with the floppy driver demonstrably
live (four insert/eject cycles; the Finder's "not a Macintosh disk" dialog is the receipt).
This **supersedes** the spec §2.7.3 evidence, which came from a tool with the §2.1 truncation
defect. Caveat: no *formatted* floppy and no format/write operation was performed, so the
SWIM write-data path is not covered.

### SCSI (register space) — NOT REACHED
139,408 writes, 100 % single-byte, 11 of 16 registers, under a real boot **and** a real
OS disk-write path (the pseudo-DMA shim carried 41,939 word writes in the same sessions, so
the driver was genuinely transferring). The register page and the DMA port behave
differently and the capture separates them cleanly.

### SCC — NOT REACHED
58,506 writes, 100 % single-byte, in both 7.0.1 and 7.5.3 (the latter with AppleTalk
active). **This is the outcome that keeps `SI-OPEN-2` deferrable**: the spec's pre-committed
table says a hit on SCC would force it to be answered before merge. There is no hit.

### ENET — NO TRAFFIC
Zero writes of any width to `mac_off 0x8000..0x8007` in any run. Consistent with that window
being the Ethernet address PROM (read-only). Latent-only by absence of traffic, not by
observed byte-only behaviour — a weaker negative than the four above, and it should be
labelled that way.

### ADBINJ — NOT REACHABLE BY CONSTRUCTION
Zero writes. This is this project's own host-injection port; MAME's `macqd700` has nothing
at `mac_off 0x11000`, and no Mac ROM or System 7 code can address it. Confirms spec §2.7.3.

### ORWELL — **REACHED**, LONG writes, from the ROM
91 writes with `mem_mask = 0xFFFFFFFF` per boot, over ~40 distinct register offsets
(`0x00`–`0x88`, plus `0xA0`–`0xB8`), identical in 7.0.1 and 7.5.3 (it is ROM boot code, so it
does not vary with the OS). Three distinct ROM sites, all disassembled from
`files/420dbff3.rom` via `unidasm -arch m68040 -basepc 0x40800000`:

```
40804852: 283c 124f 0810   move.l  #$124f0810, D4
40804858: 761f             moveq   #$1f, D3
40804870: 45d3             lea     (A3), A2          ; A3 = ORWELL base 0x5000_E000
40804872: 24c4             move.l  D4, (A2)+         ; <-- 32 LONG writes, 0x0E000..0x0E07C
40804874: e28c             lsr.l   #1, D4
40804876: 51cb fffa        dbra    D3, $40804872
4080487c: 24c4             move.l  D4, (A2)+
4080487e: 2743 00a0        move.l  D3, ($a0,A3)      ; ... through ($b4,A3)

4084bb20: 2a68 0054        movea.l ($54,A0), A5      ; A5 = ORWELL base
4084bb2e: 20c3             move.l  D3, (A0)+         ; <-- 18 LONG writes (dbra D2 = 0x11)
4084bb36: 2b42 00a0        move.l  D2, ($a0,A5)

4084bd48: 2ac1             move.l  D1, (A5)+         ; <-- 6 LONG writes
```

Captured rows (from `c701.csv`), showing the walking-pattern longwords:

```
seq,slot,addr,mask,hot,size,data,pc
…,ORWELL,5000e000,ffffffff,4,4,124f0810,40804872
…,ORWELL,5000e004,ffffffff,4,4,09278408,40804872
…,ORWELL,5000e008,ffffffff,4,4,0493c204,40804872
…,ORWELL,5000e000,ffffffff,4,4,00030810,4084bb2e
…,ORWELL,5000e0a0,ffffffff,4,4,0000ffff,4084bb36
```

**Materiality:** `ORWELL` is a Fix A slot, so Fix A changes behaviour here from "1 byte
written" to "4 bytes written". Today's downstream is `rtl/mac/orwell_stub.v`, which
acknowledges and discards every write (`rdata = 0`, `ack = cs && (rd||wr)`), so the live
behavioural delta on the current RTL is **nil** — the fix turns one discarded pulse into four
discarded pulses. This is a genuine hit that carries no regression risk *while the stub
stands*, and it becomes load-bearing the moment `orwell_stub.v` grows real state.

### SONIC — **REACHED**, LONG writes, from System 7 driver code
4 writes with `mem_mask = 0xFFFFFFFF`, at `0x50F0A000`, `0x50F0A010`, `0x50F0A094`, identical
in both OS versions. The writer is **RAM-resident driver code**, not ROM — the SONIC reset
sequence, reached via the Slot Manager. Live disassembly at the moment of the write:

```
000054E0: 2F0A           move.l  A2, -(A7)
000054E2: 2478 0DD8      movea.l $dd8.w, A2
000054E6: D5D2           adda.l  (A2), A2
000054E8: 246A 005C      movea.l ($5c,A2), A2      ; A2 = 0x50F0A000 (SONIC cmd reg)
000054EC: 7004           moveq   #$4, D0
000054EE: 2480           move.l  D0, (A2)          ; <-- LONG write, sz=32
000054F0: 2012           move.l  (A2), D0
000054F2: 0800 0003      btst    #$3, D0
000054F6: 66F8           bne     $54f0              ; poll for reset complete
```

`wpsize=32` confirms the architectural width. This is the exact residual the spec's §2.4
identifies: SONIC's existing word serializer is entered only on `wr_size_q == 3'd1 &&
!wr_addr_q[0]`, and a LONG beat misses that predicate and falls to the generic single-pulse
path — so today three of the four bytes are dropped on a 16-bit part's command register.
The spec called SONIC "the worst prior"; it is the correct call.

**Coverage caveat, stated rather than glossed:** MAME's `macqd700` has no network backend
attached, so the Ethernet driver never enters its packet send/receive path. The SONIC verdict
is already POSITIVE so this cannot change it, but the observed *count* (4) is a floor, not a
characterisation of steady-state SONIC traffic.

### ASC and the SCSI DMA shim — reached, already serialized (SI9)
Recorded for completeness and as instrument validation. ASC: 3,984 LONG writes to FIFO A
(`0x50F14000`) and FIFO B (`0x50F14400`), data `0x9B9B9B9B`, from 7.5.3 Sound Manager code at
PC `0x001994A0`–`0x001994B4`. SCSI DMA shim: 41,939 WORD writes at `…F100`, from the ROM's
unrolled blind-transfer loop —

```
40899522: 4efb 2442        jmp     ($42,PC,D2.w*4)
40899526: 335a 0100        move.w  (A2)+, ($100,A1)   ; A1 = 0x50F4F000
4089952a: 335a 0100        move.w  (A2)+, ($100,A1)   ; ... x16, computed-jump entry
```

— and from the System's own RAM copy of the same loop at PC `0x0002A06A`–`0x0002A0A6` and
`0x8009A3F2`. Both are independent, first-hand reconfirmations of the two findings
(`9468831`, `eba8aa2`) that the spec cites as the precedent for distrusting the "the real Mac
doesn't do this" claim.

---

## 6. Filling in the spec's pre-committed consequence table (§2.7.5)

| Outcome row | Fires? | Consequence, per the pre-commitment |
|---|---|---|
| Zero hits on every slot | **No** | — |
| Hits on a **byte-granular** slot (Fix A) | **YES — `ORWELL` (182 LONG) and `SONIC` (8 LONG)** | Live behaviour changes from "one byte written" to "N bytes written". **Hardware gate becomes mandatory-blocking.** For SONIC this is the case that plausibly *fixes* an existing latent misbehaviour (3 of 4 bytes currently dropped on the reset command register of a 16-bit part). For ORWELL the downstream is a write-discarding stub, so the delta is nil today. |
| Hits on a **strided** slot (Fix B) | **No** — 436,279 writes to VIA1/VIA2/IWM/SCSI-regs, zero multi-hot | The spec's "highest-risk outcome" does **not** fire. Fix B is latent-only on this evidence. |
| Hits on **SCC** | **No** — 58,506 writes, zero multi-hot | `SI-OPEN-2` is **not** forced before merge; SI8's conservative treatment stands. |

M1 is therefore **complete and answered**. M2 (the generalized `peripheral_bus.v` canary
through the full-RTL ROM boot) is still worth doing as a regression gate, and this result
predicts what it should find: multi-hot on ORWELL only (the ROM path), since SONIC/ASC's
multi-hot writers are OS-resident code the RTL ROM boot never reaches.

---

## 7. Limits of this measurement

1. **MAME is not silicon.** It is a behavioural model of the driver *stimulus*, which is what
   the question is about (what the CPU emits), but a device that MAME models incompletely
   could in principle be poked differently by a driver that got different read-backs.
2. **No network backend** → SONIC's steady-state path is uncovered (§5).
3. **No floppy format / write** → SWIM's write-data path is uncovered (§5).
4. **No Control Panel / Chooser interaction.** Driving System 7 menus needs absolute mouse
   positioning; only mouse motion and desktop clicks were injected. PRAM writes,
   AppleTalk enable/disable, and monitor-depth changes are therefore not covered.
5. **Unaligned multi-byte stores are invisible as such.** A `move.w` to an odd address is
   split by the memory system into two one-hot beats. This is not a false negative for the
   RTL question — `peripheral_bus.v` would likewise see two one-hot AXI beats — but the
   capture cannot report "the CPU executed a word store" in that case.
6. **Run-to-run counts vary** (the disk image and PRAM are mutated by each boot). The
   *multi-hot* counts were stable to the unit across every run; the single-byte totals were
   not, and should be read as magnitudes.

## 8. Artifacts

* `tools/mame_periph_strobe_capture.lua` — new, the instrument.
* `tools/mame_periph_activity.lua` — new, the post-boot activity driver.
* `tools/mame_via1_capture.lua`, `tools/mame_iwm_capture.lua`,
  `tools/mame_axi_capture.lua` — warning comments added (§2); **no behavioural change**.
