# Quadra 700 ROM Boot Path to First-Light

Scout: what Q700 ROM does between cold-reset and happy/sad mac. Method: MAME
v0.264 `macqd700` + Lua memory taps (`-video none -sound none`), static
disasm of `files/420dbff3.rom`. Raw logs in `/tmp/mame_scout/`.

**Caveat:** this scout was captured before the repo gained the real 1 KiB ADB
PIC ROM (`342s0440-b.bin`), so its MAME run used a zero-filled placeholder and
spun in the VIA1-IFR ADB-ready poll (§4). Path through DAFB init captured
end-to-end; real framebuffer draw was not reached in 120 s with the stubbed
PIC. Re-run the MAME scout with the committed PIC before treating the
ADB-sensitive portion as golden.

**Repo-state note (2026-04-19, main `8480775`):** this document is still the
best ROM/MAME scout for first-light order, but the current RTL is well past
several older blockers. Current sim has the DAFB register shim, CPU->VRAM
aperture path, HDMI scaler geometry hooks, Q700 VIA stride decode in
`glue.v`, and wider ROM-path decode coverage. The latest integrated
`tb-rom-boot` still does **not** reach the first VRAM pixel writes, but it now
runs past the relocated checksum loop and through the Q700 memory-sizing
service code. The current 40M-cycle checkpoint frontier is
`last_pc=0x4084bb92`, `next_fetch=0x00000011`, with a pending vector-11 path
from `fault_pc=0xffffffff` while D-cache flush is active. Treat that as the
active ROM bring-up problem, not the old `0x40847516` checksum-loop cap.
For fast visible-output scouting, `tb-rom-boot` now also supports
`+rom_patch=chime-delay`, which NOPs the ASC chime's inner delay loop at
`0x40807118` without changing the ROM file.  A cold
`+rom_patch=diag-loops,chime-delay` run reaches the Sad Mac diagnostic entry
at `0x40849afa`, before any DAFB/VRAM writes.

**Current no-rootfs oracle (2026-04-25):** a stock-ROM MAME `macqd700` run
with the real `342s0440-b.bin` ADB PIC reaches the no-boot-disk polling loop
at `0x40898e3e` when the ADB init wait at `0x4080a8e6` is patched at runtime
after checksum has passed.  For this bring-up, that disk-prompt loop is the
accepted spinning-floppy milestone; a root filesystem is not required.

## 1. PC trace & first VRAM write

Cold-reset entry `0x0000002A`: `jmp pc@(0x8c)`. Early path (low-space ROM
shadow while overlay is active):

| PC (low) | What |
|----------|------|
| 0x002A   | reset entry → 0x008C |
| 0x005C   | read CACR, clear caches |
| 0x006E   | `pmove %a2@,%tc` (F-line) |
| 0x0072   | `pflusha` (F-line) |
| 0x0076   | `pmove %a0@,%crp` (F-line) |
| 0x007A   | `pmove %a0@(8),%tc` (F-line) |
| 0x0084   | `movec %d1,%cacr` — cache enable |
| 0x0094   | `braw 0x4052` — main cold-boot |
| 0x46B0.. | VIA1 init (0x50F01C00 btst/movew) |
| 0x480A.. | VIA2 init |
| 0x5B1C.. | **first DAFB register write** → F9800200 |
| 0x5CB4.. | VRAM-size probe writes `"32MEG"/"1mg1"/…` ASCII to F9xFFFFC |
| 0x5D4C   | `F980001C ← 7` — VRAM-present latch |
| 0x6334+  | VRAM pattern-test (AA/55 at 0xF9000000–0xF9002xxx) |
| 0x4080A8E6 | **ADB-IFR spin** (stubbed PIC never clears) |

First VRAM-aperture write: PC **0x4089914E**, `F9800024 ← 0x1EC`. Pure pixel
writes at F9000000–F90FFFFF start around PC 0x6334 (RAM-pattern VRAM test)
— roughly a few thousand instructions post-reset.

## 2. Opcode histogram (boot window, 8 000-sample hot-PC set)

Top hot instructions until ADB wedge (all ✅ in decode except noted):
- `66f8` bnes (4629); `082b` btst #imm,(d16,An) (3140); `51cc/51ca` dbf (71);
  `3018` movew (A0)+,D0 (8); `d280` addl (7); `45ea` lea (6); `48ea`
  moveml D0-D5,(d16,A2) ✅ .L (5); `0c43` cmpiw (5); `4a41` tstw (5);
  `241a` movel (A2)+,D2 (3); **`b592` eorl D2,(A2) (3) — 🚧 mem-EA RMW**.

From reset-path disasm, before PC sampler activates:
- `4e7a/4e7b` MOVEC ✅; **`4e70` RESET ❌** (PC 0x9C); `a71e/a087` A-line ✅;
  **`f012/f000/f010/f028` PMOVE/PFLUSHA** (F-line on 68040).

Top 10 decode gaps on boot path (strictly blocking: 1, 2, 3):

1. **RESET** (0x4E70) at PC 0x9C — decode as NOP suffices.
2. **PMOVE/PFLUSHA** — 4 hits 0x6E..0x7A; vec-11 path OK if handler RTE clean.
3. **Memory-EA EOR/OR/AND.L RMW** — direct/postinc/predec/d16/abs forms covered;
   indexed EOR memory destination remains a wider follow-up.
4. MOVEM.W — RAM-fill only; deferrable. 5. Scc mem-EA. 6. CPUSH/CINV
   (MOVEC CACR bits ✅; real cache NYI). 7. TAS/CAS/CAS2. 8. BFxxx mem-EA.
   9. MOVES. 10. MULU.L SZ=1 dual-dst. Items 4-10 deferrable.

## 3. Peripheral touches (reset → ADB wedge)

From `periph3.log` over ~120 s: VIA1 R=422 719 (ADB-IFR poll), W=702; VIA2
R=116 W=123 (one-shot init); ASC R=60 004 W=122 128 (clear+silence); DAFB
F9800xxx W~400 (CRTC/CLUT/PLL); SCC 0/0; SCSI 0/0.

**Only VIA1 + VIA2 + ASC + DAFB exercised before first light.** SCC + SCSI
can stay stubbed. ASC writes must not BERR.

### ASC WAV export

Captured ASC event logs can be turned into a playable WAV artifact without
touching the RTL. The exporter reads the existing `tb_rom_boot.cpp`
peripheral model event log and reconstructs PCM from the observed FIFO
writes plus the latest ASC rate/volume/control writes.

Example:
```bash
make tb-rom-boot ROMBOOT_EXTRA="+rom_patch=diag-loops,chime-delay,timer-delay +max_insts=12000000 +periph_event_log=/dev/shm/m68k-ooo/rom_boot_asc_events.log +periph_event_filter=ASC +periph_event_log_limit=200000 +no_waves"
make rom-boot-asc-wav ROMBOOT_ASC_EVENT_LOG=/dev/shm/m68k-ooo/rom_boot_asc_events.log \
    ROMBOOT_ASC_WAV=/dev/shm/m68k-ooo/rom_boot_asc.wav
```

The reconstruction is intentionally approximate. It preserves the FIFO write
stream and the latest rate/volume settings, but it does not re-simulate the
chip cycle-by-cycle. If the capture never reaches ASC FIFO writes, the
exporter exits instead of fabricating audio.

## 4. Exceptions taken before logo

Vector-space reads in `excp3.log` are ROM copying IVT ROM→RAM (PC inside
init, not dispatch). Real traps: **F-line (vec 11) × 4** at PC 0x6E-0x7A
(PMOVE/PFLUSHA) — our vec-11 path ✅; **A-line (vec 10)** fires once Toolbox
begins (post-ADB). No BUSERR, ADDRERR, PRIV pre-logo.

**Task #134 gating:** `0x4AFC` ILLEGAL (vec-4) NOT exercised pre-logo.
A-line/F-line are what ROM fires. Flip is safe.

## 5. Happy vs sad path

POST stages: (1) cache clear ✅; (2) PMOVE MMU init — F-line, handler no-op;
(3) VIA1+RTC probe → sad if silent; (4) **RAM sizing** via BERR probe — our
DDR stub returns 0 for [0, 0x04000000] → "full RAM"; (5) ROM checksum vs
0x420DBFF3 → passes (intact image); (6) **VRAM sizing** — writes
"32MEG"/"1mg1"/"M5.1"/"K512" ASCII, reads back; (7) DAFB CRTC/PLL/CLUT →
scanner active; (8) ADB query (stubbed-wedge, does NOT block logo).

**Happy fires if 1-7 pass.** Sad fires on {RAM BERR, ROM checksum, VRAM
readback} failure.

**Recommendation: aim for happy.** The current RTL already has VRAM aperture
storage/readback, DAFB register echo/sense/status shims, and a first-light
DAFB base/stride path into `video_top`'s scanner. The remaining video risk is
that BPP is still treated as a raw latch, CLUT values do not colour-map
indexed pixels, and VBL/status timing is deterministic rather than scanner
driven. The first sim-logo milestone should still include a VRAM dump
artifact rather than relying only on HDMI-path assumptions.

## 6. Logo draw primitive

ADB wedge blocks live capture. From System 7 sources + ROM around
0x40899xxx: bitmap is 32×32 mono at fixed ROM offset. Primitive: tight
`move.l (A0)+,(A1)+` loop, ~1024 stores to `F9000000+offset`. DAFB aperture
is non-cacheable (TT0) — LSU must bypass D-cache for F9000000-FA000000;
`rtl/mac/glue.v` passthrough already. Only opcode gating is
`move.l (An)+,(Am)+` plus CCR sequencing — both ✅.

## 7. Live punch list: "what's in the way of first light"

| # | Item | Class | Current state |
|---|------|-------|---------------|
| 1 | Resolve the `0x4084bb92` memory-probe frontier | core/runtime | Current 16M/32M/40M checkpoints all stop with vector-11/open-bus state from `fault_pc=0xffffffff` and active D-cache flush |
| 2 | Continue ROM decode/core chase after memory sizing completes | core/opcode | Recent service-code gaps fixed `JMP (d8,PC,D3.W)`, `MOVEA.L (A7)+,A0`, indexed `CMP.B`, and `NOT.B` forms |
| 3 | Mem-EA EOR/OR/AND.L RMW for VRAM tests | opcode | AND/OR RMW forms have coverage; EOR now covers direct/postinc/predec/d16/abs memory destinations for B/W/L, with indexed memory destination still pending |
| 4 | MOVEM.W and wider RAM-test coverage | opcode | Still deferrable until it appears on committed path |
| 5 | VRAM aperture `0xF9000000..0xF90FFFFF` | peripheral | Wired through xbar/VRAM path; keep regression tests |
| 6 | DAFB register echo/sense/status | peripheral | Minimal shim landed in `rtl/mac/video.v` |
| 7 | DAFB base/stride -> scanner | video | Wired through DAFB live latches, `video_top` config snapshotting, `fb_reader`, and scaler; BPP remains raw/observational for this 8bpp first-light path |
| 8 | DAFB CLUT -> indexed RGB | video | **Open**; 8bpp scan-out is greyscale fallback |
| 9 | DAFB VBL/status/VIA tie-in | video/IRQ | **Open**; status currently reads deterministic zero |
| 10 | Sim logo artifact | testing | **Open**; dump VRAM as PBM/PGM/PPM once writes appear |

**First-light attack order:** debug the `0x4084bb92` memory-sizing/open-bus
frontier from the saved checkpoints, keep advancing the ROM in sim until
writes to `0xF9000000..0xF90FFFFF` appear, then immediately add a VRAM dump
test. In parallel, plan the DAFB live-state work (base/stride/BPP first, CLUT
second, VBL third) so the HDMI path can render whatever the ROM wrote.

## Appendix: raw logs

`/tmp/mame_scout/vram_writes.log`, `periph_access.log`, `pc3.log`,
`excp3.log`, `q700_full.dis`. MAME:
`mame macqd700 -rompath … -autoboot_script trace3.lua -autoboot_delay 0
-video none -sound none -nothrottle -seconds_to_run 120`. `342s0440-b.bin`
stubbed zeros (warning; boot proceeds).
