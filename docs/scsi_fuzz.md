# SCSI 53C96 differential fuzzing — MAME `ncr53c90` as golden reference

`tools/fuzz/scsi_fuzz.py` applies the project's Musashi methodology to the
NCR 53C96 SCSI controller model in `rtl/mac/scsi.v` (TURBOSCSI_C96=1):
both sides execute the **same generated register-access script** and the
settled architectural state is compared at defined sync points.

```
make fuzz-scsi                    # 50 seeds, full profile
make fuzz-scsi N=200 START=100    # more seeds
make fuzz-scsi-replay SEED=17     # replay one seed
make fuzz-scsi DISK_IMAGE=x.img   # serve a real disk image on both sides
make fuzz-scsi-direct             # same seeds, scsi.v ALONE as the DUT
make fuzz-scsi-direct-replay SEED=17
tools/fuzz/scsi_fuzz.py --gen-only --seed 17     # print the script
tools/fuzz/scsi_fuzz.py --n 50 --profile clean   # driver-shaped subset
tools/fuzz/scsi_fuzz.py --n 50 --mask-cmd        # drop cmd= from the diff
```

`fuzz-scsi` and `fuzz-scsi-direct` run the SAME scripts against two
different DUT shapes — see "How each side is driven" below.  When a
divergence appears in `fuzz-scsi`, re-run that seed with
`fuzz-scsi-direct-replay`: if it survives there the chip model owns it,
if it disappears the fabric does.

Requirements: `mame` (0.264+; developed against 0.285) and `chdman` on
PATH, and the Q700 ROM set (default `~/mame_q700_good/roms`; override
with `MAME_Q700_ROMS` or `--mame-roms`).

## How each side is driven

* **RTL** — `tb/tb_scsi_fuzz.cpp`, a scriptable Verilator harness, built
  by `make tb-scsi-fuzz-harness` with `--public-flat-rw` so sync records
  can snapshot `c96_*` internals without popping the FIFO.  The SD mock
  serves a deterministic pattern disk (or `--disk-image`), with an
  in-memory write overlay.  ONE source file, TWO DUT shapes:
    * **default (`make fuzz-scsi`)** — top is `tb/tb_pb_scsi.v`:
      **real `peripheral_bus.v`** + real `scsi.v` (TARGET_ID=6,
      TURBOSCSI_C96=1) + `vhdd_sd`, driven by genuine **AXI4**
      transactions (`arsize`/`wstrb` carry the host access width).  This
      is what puts the pseudo-DMA aperture's word-splitting serializers
      (`rd_scsi_phase_q` / `wr_scsi_strb_q`) and the
      `scsi_dma16_lo_beat` DRQ-grant carry inside the fuzzer's cone.
      `PB_WATCHDOG_LOG2=13` so a genuinely withheld beat resolves to a
      reportable SLVERR instead of hanging the run.
    * **`make fuzz-scsi-direct`** — top is `tb/tb_scsi_vhdd_sd.v`
      (scsi.v + vhdd_sd), the historical shape, driving scsi.v's `pb_*`
      face directly.  The harness drives `pb_dma16_lo_beat` itself so
      16-bit ops still reach `c96_dma16_hi_granted`.  Kept for
      **attribution**, not for coverage.
* **MAME** — `tools/mame_scsi96_fuzz.lua` inside a `macqd700` boot:
  the CPU is parked on a `bra .` at frame 5 (SR=0x2700), registers are
  driven through the DAFB TurboSCSI aperture at `0x50f0f000`
  (`reg = offset>>4`, full side effects), pseudo-DMA beats are byte OR
  16-bit accesses at `+0x100` (`read_u8`/`write_u8` → `dma_r`/`dma_w`;
  `read_u16`/`write_u16` → `dma16_swap_r`/`dma16_swap_w`, the entry
  points the Q700 ROM actually uses), the DAFB TurboSCSI control word
  is poked at `0xf9800024` so the DRQ-check bits (7 = read, 8 = write)
  can be armed, time is stepped with a frame-done coroutine, and
  internal state (FIFO contents, irq, drq, config) is read
  NON-destructively through save-state items
  (`devices[":scsi:7:ncr53c96"].items` + `emu.item`).
  The hard disk sits at `:scsi:6:harddisk`, backed by a CHD built from
  the same deterministic pattern (or the `--disk-image` file).
  Both sides run all scripts of a batch sequentially against a live
  chip; every script begins with a normalization preamble.

## Script ops (shared, byte-identical logs)

```
W r v | RC r | RU r | CTRL v
DR n | DRU n | DW n b..            paced,  8-bit aperture
DR16 n | DRU16 n | DW16 n w..      paced, 16-bit aperture
DRB n | DRUB n | DWB n b..         blind,  8-bit aperture
DRB16 n | DRUB16 n | DWB16 n w..   blind, 16-bit aperture
GAP c | SDGAP c | SETTLE | SYNC id | END
```

`GAP`/`SDGAP` are RTL-only sub-frame timing perturbation (MAME cannot
step time below a frame; the comparison is settled-state, so this
asymmetry is sound).  `SETTLE` = bounded quiesce (RTL: cycle budget
computed from the programmed select timeout with irq early-exit; MAME:
4 frames ≈ 67 ms, which bounds every timer the generator can arm).

### The pseudo-DMA aperture ops

**Width is not cosmetic.**  The DAFB aperture is a 16-bit port
(`macquadra700.cpp:565` maps `0x5000f100..0x5000f101`), and the width of
the host access selects a *different chip entry point*:

| host access | MAME path | behaviour |
|---|---|---|
| byte | `dma_r()` / `dma_w()` | one FIFO slot |
| word | `dma16_swap_r()` / `dma16_swap_w()` | **atomic pair**, one DRQ check |

`dma16_r` (`ncr53c90.cpp:1325-1349`) pops both bytes under a *single*
`fifo_pos` test and calls `decrement_tcounter(2)` / `check_drq()` /
`step()` ONCE; at `fifo_pos < 2` it returns `dma_r() | 0xff00`, i.e. one
pop with 0xff in the other half.  `dma16_w` (`:1352-1358`) degrades to a
**single** `dma_w` of the first byte when `fifo_pos > 14 || tcounter == 1`
— it drops the second byte.  None of that is reachable from two byte
accesses, which is why byte-only ops proved nothing about the path the
ROM actually uses.  Word payloads are logged/parsed as 4 hex digits,
high byte first (= the byte at the lower m68k address).

**`CTRL v`** writes the DAFB TurboSCSI control word (RTL: `scsi_ctrl_in`;
MAME: the u32 at `0xf9800024`, `dafb.cpp:487`).  Bit 7 = DRQ Check Read,
bit 8 = DRQ Check Write.  Reset default 0 = a blind aperture, which is
what made every hold-off path in `scsi.v` and `peripheral_bus.v` dead
code for the whole fuzz run before this op existed.  `normalize()` emits
`CTRL 000` FIRST — the control word lives outside the 53C96, so neither
a chip reset nor a bus reset clears it, and without that a block that
armed the check would leak into the next block *and the next seed*.

**Paced vs blind.**  Paced ops (`DR`, `DW`, `DR16`, `DW16`) wait for the
chip's DRQ before each beat and give up if it never rises, so "how many
bytes moved" is itself compared.  Blind ops (`DRB…`) do not wait — they
reproduce the ROM's real 16-byte chunk drain, eight back-to-back
`move.w` with no DRQ poll at all.  Under LBTM (`config3` bit 2) the
BUSMD_1 DMA_IN formula is `fifo_pos > 1`, so a *paced byte* drain stalls
at the odd occupancy and can never reach `fifo_pos == 0`; only a 16-bit
pop, which skips that occupancy, gets there.  That combination —
16-bit + blind + LBTM — is the ROM's drain, and it was unreachable in
every one of the six blind spots below.

**Hold-off contract** (identical in both executors: `beat_allowed()` in
`tb/tb_scsi_fuzz.cpp` and in `tools/mame_scsi96_fuzz.lua`): if the DAFB
check bit for this direction is armed and DRQ is low, the access is *not
issued* and the op stops there.  That is what both sides really do —
MAME rewinds the instruction and returns 0xffff without touching the
chip (`dafb.cpp:1003-1009`); the SoC withholds the `pb` pulse until the
fabric's ack watchdog raises a bus error.  Stopping and logging how far
we got keeps the sides comparable while making the DRQ decision a
**per-beat compared quantity**, not something only visible at SYNC
granularity.

Every DMA record carries a trailing **`to=`** flag.  `to=1` means the
access was issued and never terminated (PB build: `peripheral_bus`
answered SLVERR from its ack watchdog).  MAME can never produce that, so
`to=1` is by construction a divergence — deliberately, because an
aperture beat that never terminates *is* the Sad Mac.

## What is compared at a SYNC

Internal snapshot first (mutation-free), then bus reads in fixed order:

```
SYNC id fifo=<pos>:<bytes> irq drq cfg=<busid:selto:syncper:syncoff:clkconv:cfg1:cfg2:cfg3>
        cmd=<echo:qdepth> tcount stat(4) seq(6) flags(7) tclo(0) tchi(1) istat(5)
```

The `istat` read is deliberately last: it is destructive (clears int
state, pops the command queue) **identically on both sides**, which is
sound because SYNCs only happen after SETTLE.

### Deliberately excluded (differ masks)

* `RU` values and `DRU`/`DRU16`/`DRUB`/`DRUB16` payload+length —
  device-dependent data (INQUIRY strings, sense bytes, mode pages:
  properties of vhdd-vs-nscsi_harddisk, not of the 53C96).  The
  trailing `to=` flag on those records is NOT masked: a beat that never
  terminates is a defect whatever the payload was going to be.
* `cmd=` (command echo + queue depth) is **compared by default since
  2026-08-19** — it used to be masked, which hid command-queue
  retirement, a load-bearing wedge mechanism (blind spot #5 below).  It
  is still noisy, so the runner reports it in a SECOND tier: a seed
  whose only divergence is `cmd=` is listed as "cmd=-only" with its
  artifacts under `fails_cmd/`, and does not shadow the first
  aperture/FIFO/DRQ divergence further down the same log.  `--mask-cmd`
  drops it from the diff entirely when bisecting an unrelated
  regression.
* The normalization preamble's istatus drains are `RU`: after a
  disruptor interrupted a select mid-arbitration, the wreckage istatus
  is timing-dependent (measured 0x18 vs 0x80); the SYNC that follows
  compares the converged state.
* Read payloads for READ(6)/READ(10)/READ CAPACITY and DW/readback
  data ARE compared (identical disk on both sides).  Generator writes
  stay inside an LBA window that random reads avoid, so backend flush
  policy differences cannot poison unrelated reads.

### Deliberate real-chip divergences (expected fuzz failures)

MAME is the golden reference for this fuzzer, but it is not the last
word: where MAME's model and the real 53C96 demonstrably disagree AND
the difference is load-bearing for boot, this project follows the real
chip (owner decision 2026-09-06; precedent: the RESET-instruction
decision).  Divergences in this table are EXPECTED and must NOT be
"fixed" by restoring MAME parity — read the linked rationale first.

* **Non-DMA DATA IN `CI_XFER` (0x10) over FIFO residue: one byte +
  I_BUS, never a free-run.**  MAME's completion test
  (ncr53c90.cpp:650, `fifo_pos == 1` after the push) never matches a
  receive armed behind residue, so MAME free-runs pulling target bytes
  until the FIFO fills, with no interrupt.  A real 53C96 moves one
  byte and interrupts regardless.  The MAME-faithful model turned a
  stolen interrupt-context istatus read into the permanent 7.5.3 boot
  wedge at ROM `0x40899704` (measured on silicon with the p143 trace
  ring, 2026-09-06); the real-chip form costs the same race one benign
  retry.  The RTL additionally completes an armed non-DMA DATA IN
  transfer whose backing supply has PERMANENTLY drained (provider
  idle, ring empty) — the trailing transfer — which MAME never
  reaches, because its target model always has the byte.  Coverage:
  `make tb-scsi-c96-nondma-trailing` (both park shapes, positive
  controls).  Any fuzz seed arming 0x10 in DATA IN with residue (the
  2026-08-19 "seed 13 family") now diverges by design: RTL shows
  `istat=0x10`/fifo=residue+1 where MAME shows `istat=00`/fifo
  free-running.  Site: the divergence box at the non-DMA CI_XFER
  completion hook in `rtl/mac/scsi.v`.

### Known comparison hazards

* Scripts in a batch share a live chip; a real divergence in seed k can
  cascade into k+1.  The runner re-verifies every failure standalone
  and reports cascades separately.
* `0x46` (SELECT_ATN3) while disconnected passes MAME's validity check
  but has no `start_command` case — mame0285 `ncr53c90.cpp:1057`
  `fatalerror`s (the golden model crashes).  The generator never emits
  it; the RTL answers I_ILLEGAL (documented divergence-by-necessity).

## Generator profiles

* `full` (default) — the whole stimulus space: valid transactions
  (TUR/REQUEST SENSE/INQUIRY/MODE SENSE/READ CAPACITY/READ 6+10/WRITE
  10, DMA + non-DMA, IDENTIFY permutations incl. wrong LUNs, all select
  forms incl. DMA variants, junk opcodes, truncated/over-long CDBs,
  wrong tcounts incl. 0 (=65536), partial/over drains), junk register
  bursts, orphan commands, FIFO over/underfill, mid-flight chip/bus
  resets and destructive reads, write-then-readback verification.
* `clean` — the driver-shaped subset (bare-CDB CD_SELECT, IDENTIFY+CDB
  SELECT_ATN, exact lengths and counts).  Useful to bisect a broad
  regression.

### Baseline — and why the number moved

The seed space **changed meaning** on 2026-08-19 when the aperture
widening landed (16-bit ops, the DRQ-check, blind drains, and
`peripheral_bus.v` in the DUT).  A seed number identifies a different
script before and after, so pre-widening rates are NOT comparable and
pre-widening seed lists no longer identify the classes they used to.

| when | gate | rate | failing seeds |
|---|---|---|---|
| 2026-08-19, pre-widening | seeds 0-59, `cmd=` masked, scsi.v-only DUT | 56/60 | 9 31 53 58 |
| 2026-08-19, post-widening | seeds 0-59, **core** state, `peripheral_bus`+scsi.v DUT | **48/60** | 0 5 9 16 30 31 38 41 43 45 48 54 |
| 2026-08-19, post-widening | seeds 0-59, core **plus `cmd=`** | **24/60** | the above + 24 cmd=-only seeds |

The post-widening run is reported in two tiers (see the `cmd=` note
under "Deliberately excluded"): a **core** rate over aperture payloads,
FIFO, DRQ, tcount and status/seq/flags/istat, and a stricter rate that
also compares `cmd=`.  The classes behind the residual divergences are
in the A-series table below; 11 of the 12 core failures survive on the
direct (scsi.v-only) DUT, so the chip model owns them.

The core count going 56 -> 48 is the widening working, not a regression:
the stimulus now reaches the 16-bit aperture, the DRQ hold-off and the
fabric's word-splitting FSM, none of which any earlier seed could touch.

The earlier "144/200 over seeds 0-199" figure was measured against the
pre-widening generator and is retained only as history.

## Findings (each verified against MAME 0.285 with minimal scripts)

Fixed in `rtl/mac/scsi.v` during this work (see git history for the
per-finding diffs):

| # | Finding | Status |
|---|---------|--------|
| F1 | Select paths self-decoded the whole FIFO as a CDB.  Real chip: the TARGET pulls bytes REQ-by-REQ and decides the CDB length (nscsi group table 6/10/10/1/1/12/1/32); surplus stays in the FIFO; ATN forms send the FIRST byte as the message whatever it is; seq_step settles 1/2/3/4 per MAME `DISC_SEL_*` | fixed |
| F2 | SELECT_ATN_STOP executed the CDB instead of halting after one message byte (seq=2, I_FUNCTION\|I_BUS, MSG_OUT, FIFO retained); continuation via Transfer Information message drain then CDB feed | fixed |
| F3 | Mode-aware command validity was missing (53c90a `check_valid_command`): initiator commands at bus-free / select commands while connected are I_ILLEGAL; CD_RESELECT with nothing to reconnect waits silently forever | fixed |
| F4 | DMA commands with tcount=0 transfer 65536 bytes (counter wraps; TC0 on 1→0 only) — RTL treated it as zero-length | fixed |
| F6 | Completed DMA-out left 16 stale bytes in the FIFO (double-booked push) | fixed |
| F8 | Command register is a 2-deep queue: slot 0 retires on a nonzero istatus read (`command_pop_and_chain`), a third write is dropped with S_GROSS_ERROR, RESET forms bypass the queue | fixed |
| — | CI_COMPLETE appends status+msg to the existing FIFO (phase-sensitive: STATUS pulls status+msg, MSG_IN pulls msg, out-phases pull one 0x00 with I_BUS); CI_MSG_ACCEPT leaves the FIFO alone and I_DISCONNECTs only when it releases a MSG_IN-parked connection | fixed |
| — | IDENTIFY LUN is latched per connection; a wrong LUN CHECK-CONDITIONs each command AFTER its CDB is consumed (INQUIRY excepted) — not rejected at select time | fixed |
| — | INQUIRY supplies exactly the allocation length, zero-padded past 36 | fixed |
| F11 | `dma_dir`/`dma_command` survive completion; DRQ is a LATCHED `check_drq()` result recomputed only at FIFO/tcounter ops (not at `dma_set`), so stale directions keep DRQ observable; blind aperture reads pop the FIFO (+ tcounter decrement outside DATA IN, clamped at TC0), blind writes push (+ decrement when the last valid command was DMA-form) | fixed |
| F12 | A DMA Transfer Information armed in STATUS pulls the status byte into the FIFO and parks in MSG_IN, staying armed | superseded by F16/F18 |
| F13 | COMPLETION clears DRQ: `function_complete()` / `function_bus_complete()` / `bus_complete()` all run `dma_set(DMA_NONE)` + `check_drq()` (ncr53c90.cpp:772-799), forcing the latched DRQ low the moment any command completes.  The RTL left `c96_drq_stale` latched high after e.g. a completed DMA-form select (seed 4) — the DRQ half of the MacBench "SCSI Information" hang.  Fixed: every completion site clears the latch; DIR_IN additionally evaluates the BUSMD_1 live formula (see F17) | fixed |
| F14 | Post-ATN_STOP DMA message drain wedge: `c96_stop_feed` was gated on the bare `pb_dma_shim` ADDRESS DECODE, so a bus idling with `pb_addr` parked at 0x100 after the last beat held the drain off forever — chip parked in MSG_OUT, no interrupt, while MAME sends every staged byte (stale FIFO + dma_w pushes) until empty and completes with I_BUS (seed 0 + 17 same-signature seeds).  Fixed: gate on the actual access strobes | fixed |
| F15 | FIFO residue semantics: MAME `fifo_pop()` is a memmove — slots at/beyond the new occupancy KEEP their contents, so blind `dma_r` of an empty FIFO repeats the last-popped byte; `device_reset()` (CM_RESET) memsets the CONTENTS, not just the count; CM_RESET_BUS touches neither.  The RTL's shift-all-with-0x00-fill, bulk CDB consumption and reset-count-only left different garbage (seeds 9/31/75/94).  Fixed: memmove-exact pops (incl. the select paths' byte-by-byte residue), CM_RESET memset, CM_RESET_BUS retention | fixed |
| F16 | The STATUS/MSG_IN receive of a DMA CI_XFER is DEFERRED, not instant: MAME's `recv_byte()` rides a `delay_cycles()` timer that fires only when emulated time advances, so a burst of blind `dma_w` beats issued right after the arm lands in the FIFO FIRST and the received byte is DROPPED at push time if they filled it (seed 5: MAME keeps 7 dma_w bytes and no status byte).  Fixed: `c96_xfr_recv_*` quiesce-timer deferral (fires after 512 DMA-port-idle cycles) + push-if-room | fixed |
| F17 | DRQ for DMA_IN is the 53C94 BUSMD_1 formula (macquadra700 sets BUSMD_1): async `fifo_pos > ((config3.2 \|\| !TC0) ? 1 : 0)` — i.e. DRQ needs TWO staged bytes until TC0 ("save last remaining byte for the processor") and falls synchronously with the pop (the RTL's event-latched recompute was one beat late — seed 5 took 17 beats where MAME takes 16).  DMA_OUT is `!TC0 && fifo_pos < 15` | fixed |
| F18 | `INIT_XFR_WAIT_REQ` condition ORDER (ncr53c90.cpp:646-658): the TC0 completion is checked FIRST and never touches the command queue; only the PHASE-CHANGE completion zeroes it (`command_pos = 0`, echo retained — the next `command_w` then dispatches immediately instead of queueing).  Both paths then hold I_BUS until `!(dma_command && drq)` (`INIT_XFR_BUS_COMPLETE`).  Seed 4 (no TC0: a queued FLUSH must run, clearing FIFO + zeroing tcounter) vs seed 5 (TC0: the FLUSH stays queued and a third write gets S_GROSS_ERROR) | fixed |
| F19 | CI_COMPLETE dispatched while ALREADY in MSG_IN (status consumed out of band) receives the message with ACK RELEASED (`INIT_CPT_RECV_BYTE_ACK`), so the target finishes COMMAND COMPLETE and disconnects: istatus = I_DISCONNECT + bus free — NOT the I_FUNCTION + parked connection of the normal STATUS-entry path (whose msg byte is held un-ACKed) | fixed |

Open (residual full-profile divergence classes, each with saved
artifacts under `build/fuzz_scsi/fails/`):

**R1 is CLOSED (2026-08-19)**: the DMA-in path is now staged through the
real 16-deep `c96_fifo` — accepts push (stalling at 16, which is what
keeps TC0 timing honest for tcount > 16), every host pop path (DMA
aperture, reg-2) pops the same FIFO, offset 7 is the live occupancy,
DRQ is the BUSMD_1 formula (F17), and chunk completion is MAME's
`TC0 && !drq` verbatim.  Validated three ways: (a) the full tb battery
incl. `tb-scsi-c96-read6` 28/28 and `tb-scsi-c96-sm43-chunk` 291/291
(the 7.5.3 chunk flows that boot real hardware); (b) two byte-identical
DIRECTED differentials vs MAME 0.285 — the complete 32-chunk 7.5.3
drain flow and a paced short-chunk over-read — both line-for-line
identical to MAME except the default-excluded `cmd=` echo; (c) the
fuzz sweep.  Three tb expectations were recalibrated WITH differential
evidence (they encoded the pre-R1 shim): the last chunk of a block
reads STAT 0x13 (target releases DATA IN on the final chip ACK), the
BUSMD_1 threshold holds DRQ low with one pre-TC0 staged byte (the
processor collects it through reg 2), and a short chunk's I_BUS is
held until the FIFO drains below the DRQ threshold.

| # | Class | Assessment |
|---|-------|------------|
| R2 | FIFO overflow drop-ordering: when pushes race select-consumption at saturation, the two sides keep a different 16-byte window | corner, RTL gap |
| R3 | `stat` phase bits at idle read 0 on RTL vs 03/06 on MAME after a chip reset — or even a chip reset PLUS a bus reset (measured seed 196) — interrupted a select mid-arbitration or an ATN_STOP halt: MAME's nscsi target stays wedged driving its latched ctrl lines and even ignores the RST pulse; a real bus (and this RTL) goes bus-free.  ~11 seeds/200 | MAME artifact (probable) |
| R4 | Residual `drq`/`data` deltas in compounded-junk states (blind pops of stale FIFO content, dir-tracking micro-differences through exotic abort chains) | mixed |
| R5 | Non-identify message bytes: nscsi ignores unknown messages; some MAME flows still answer with target-side behavior (MSG_IN excursions) the shortcut backend does not model | RTL gap, junk-only |
| R6 | COMMAND-phase DMA CI_XFER (0x90 while connected/cmd_wait) with STALE FIFO residue: MAME runs the CDB feed through the real FIFO — stale bytes + dma_w pushes mix, sends pop the head with handshake pacing (first send instant at dispatch, later sends only as emulated time advances) — while the RTL feeds the DMA-port bytes directly as CDB bytes and never sends the stale residue.  Junk-only: the ROM's deferred-select tail flow arrives here with an EMPTY FIFO and is deliberately untouched (converging this needs cycle-exact send/pop interleaving emulation — high risk against the boot-validated deferred-select and ATN_STOP-continuation paths for a stimulus no driver produces).  **Seed numbers dropped 2026-08-19: the widened generator changed what every seed means, so the old 58/77/97/143/189 list no longer identifies this class.  Re-derive from `build/fuzz_scsi/fails/` after a run.** | RTL gap, junk-only |
| R7 | Blind-pop garbage-value deltas: each side pops a differently-ordered FIFO residue (F15 memmove semantics) after compounded junk.  **Now reached far more often** — the `DRB`/`DRB16` blind ops pop residue directly instead of only stumbling into it through paced drains, and the 2026-08-19 60-seed run has this as the first divergence on several seeds.  Still junk-only: every instance measured so far starts from a FIFO the ROM never leaves dirty.  Old seed list (9/39/94) is stale for the same reason as R6. | mixed, junk-only |
| R8 | `seq_step` settled values through queued/repeated select chains: c96_sel_seq_final accounting diverges from MAME's DISC_SEL_WAIT_REQ rule when selects stack in the command queue.  Old seed list (80/89/155/157) is stale post-widening. | RTL gap, untriaged |

### A-series — surfaced by the 2026-08-19 aperture widening

New classes, i.e. ones no pre-widening seed could reach.  "Direct"
records whether the divergence survives on `make fuzz-scsi-direct`
(scsi.v alone): **yes** = the chip model owns it, **no** = it only
appears with `peripheral_bus.v` in the DUT.

| # | Class | Direct | Assessment |
|---|-------|--------|------------|
| A1 | **`dma16_r` FIFO-underflow fill.**  MAME `ncr53c90.cpp:1327-1329`: a 16-bit aperture read with `fifo_pos < 2` performs ONE `dma_r()` and returns `0xff` in the other half, so `dma16_swap_r` yields `<popped>:0xff`.  The SoC has no 16-bit read path at all — `peripheral_bus.v` splits the word into two byte pops, so the low half is a second real/residue byte and reads `<popped>:<residue>`.  Signature: `DRB16 1 got=1 data=0000` (RTL) vs `data=00ff` (MAME), and the same 0xff pattern repeated across a long over-read. | yes | **RTL gap, real.**  Q700 reachability is real but narrow: the ROM's chunk drain reads exactly the staged count, so it meets `fifo_pos == 2` on the last word, not `< 2`.  An over-read (short/error tail chunk) reaches it.  Fixing it needs a genuine 16-bit entry point rather than a split pair — an RTL design decision, not a patch. |
| A2 | **DRQ disagreement at identical FIFO/tcount.**  A SYNC where `fifo=`, `cfg=`, `tcount=` and the FIFO CONTENTS all match but `drq` (and consequently `stat`/`istat`) differ. | yes | **RTL gap, high interest.**  DRQ is what the entire pseudo-DMA handshake keys on, and it is the family both the MacBench hang (F13) and the Sad Mac (the split-beat grant) came from.  Now visible per-beat rather than only at SYNC, because a held-off beat truncates the record's `got=`. |
| A3 | **DMA-OUT staged-FIFO content after a CI_XFER dispatched in COMMAND phase over stale residue.**  Both sides end at `fifo=16` with a common CDB prefix and different tails. | no | **R6 family, junk-only**, amplified by the PB build's slower per-beat cadence (each aperture beat is an AXI transaction, ~10 cycles, vs 1 on the direct face), which reorders MAME's send/pop interleave against ours.  No driver produces this shape; see R6. |
| A4 | **Blind-pop residue values** (`DRB`/`DRB16` of an empty or stale FIFO). | yes | **R7 family, junk-only**, but now reached routinely instead of by accident — the blind ops pop residue directly.  F15 memmove semantics; the ROM never leaves the FIFO dirty here. |
| A5 | **Write-side split-beat DRQ re-check — a 16-bit pseudo-DMA WRITE that never terminates.**  `peripheral_bus.v:1722` drives `scsi_dma16_lo_beat` from `rd_scsi_dma16_lo` **only**, so the second byte of a split 16-bit aperture WRITE gets no grant carry and re-runs the DRQ check.  With the DAFB write check armed (`CTRL` bit 8) the low beat is withheld and the ack watchdog answers SLVERR: `DW16 ff put=ff to=1` where MAME completes the whole 512-byte push. | no | **Real, and it is the exact structural mirror of the Sad Mac `0F 02`** — same missing grant carry, other direction.  On the board a withheld aperture beat is a bus error, not a stall.  Predicted from the code before it was observed; a directed probe could not force it (this SoC's DATA_OUT sink drains the FIFO as fast as the host fills it, so `fifo_pos` never reaches the `!TC0 && fifo_pos < 15` threshold on its own) — the fuzzer found the state that does. |

**A5 minimal reproducer** (deterministic; `to=1` on the PB DUT, clean on
the direct DUT and on MAME):

```
… normalize …
W 2 80  +  WRITE(10) CDB (lba 0x0300, 1 block)  +  W 3 42   ; SELECT_ATN
SETTLE
W 2 00 / W 2 01 / W 2 00      ; THREE stale bytes left in the FIFO
CTRL 180                      ; DRQ-check reads AND writes
W 0 00 / W 1 02               ; tcount = 512
W 3 90                        ; DMA | CI_XFER (out)
DW16 ff <255 words>           ; -> RTL: put=ff to=1   MAME: put=ff to=0
DW 2 33 98                    ; -> RTL: put=0         MAME: put=2
```

The three stale bytes are load-bearing: they shift the word boundary so
a low half lands on a cycle where DRQ is low.  Without them the same
script completes on all three.  Full artifacts: seed 45 of the
2026-08-19 run, `build/fuzz_scsi/fails/seed_00000045/`, log line 22.

### `tb-scsi-c96-mame-chunk`'s 32-byte scenario

That tb (commit `48e4258`) is left RED on the 32-byte chunk — a chunk
LARGER than the 16-deep FIFO, so the chip must refill mid-drain — for
want of differential evidence.  The blind ops do **not** settle it, and
it is worth being explicit about why rather than leaving the impression
that they do: a blind burst longer than one FIFO load races the target's
refill against MAME's frozen clock, so the two executors would be
compared on timekeeping (see "Harness artifacts" below).

What the widening *does* give it is the **paced** equivalent: an armed
32-byte chunk drained with `DR 20` / `DR16 10`, which is comparable
because each beat yields to MAME's scheduler.  That covers the tb's
load-bearing claims about a >FIFO chunk (TC0 only reachable AFTER the
drain, tcounter parked at `chunk_bytes - 16` pre-drain).  It is thin
coverage — 2 of the first 60 seeds — so if that scenario is being
settled deliberately, widen `block_rom_chunk_drain`'s chunk choice
rather than relying on the default weights.

### Harness artifacts to expect (and what was done about them)

Blind ops trade one blind spot for a new hazard, because MAME's clock is
frozen inside a Lua op burst while the RTL's is not:

* A blind burst issued while the transfer is **still streaming** compares
  which executor refilled first, not the chip.  The generator therefore
  arms exactly one FIFO load, settles to TC0 before a blind drain, caps
  a blind burst at 16 bytes, and settles between the halves of a
  mixed-width drain.  Each of those was added after a measured false
  positive, not defensively.
* A divergence that appears on the PB build but not the direct build is
  **not automatically a fabric defect** — the PB build simply spends
  ~10× more cycles per aperture beat.  Always check the direct build
  (`make fuzz-scsi-direct-replay SEED=<n>`) before blaming
  `peripheral_bus.v`, and prefer a directed `tb-pb-scsi` scenario to
  settle it.

False-positive sources to keep in mind when triaging: cross-script
cascades (auto-rechecked), the MAME stale-phase artifact (R3), and
anything involving `RU`/`DRU`-masked data that leaks into counts.

## Reproducing and triaging a failure

```
python3 tools/fuzz/scsi_fuzz.py --seed <N>          # standalone recheck
ls build/fuzz_scsi/fails/seed_<N>/                  # script + both logs
ls build/fuzz_scsi/fails_cmd/seed_<N>/              # cmd=-only divergences
make fuzz-scsi-direct-replay SEED=<N>               # attribution
```
The report names the first divergent log line; the script is the exact
input sequence.  Hand-minimize by deleting blocks from the script and
re-running both sides (`tb_scsi_fuzz --dir`, and MAME with
`SCSI_FUZZ_DIR` as in `tools/mame_scsi96_fuzz.lua`'s header).

**Triage order that actually converges:**

1. Is the divergence `cmd=`-only?  The runner already told you — it is
   under `fails_cmd/`, not `fails/`.  That is command-queue retirement
   micro-timing, not aperture state.
2. Does it survive on the DIRECT DUT?  `make fuzz-scsi-direct-replay
   SEED=<n>`, or run `build/scsi_fuzz/Vtb_scsi_vhdd_sd --dir <one-script
   dir>` and diff against the stored `.mame.log`.  Survives → the chip
   model owns it.  Disappears → either `peripheral_bus.v` owns it, **or**
   it is the PB build's ~10×-longer per-beat cadence letting the target
   stage another byte while MAME's clock is frozen.  Distinguish those
   two with a directed `tb-pb-scsi` scenario; do not guess.
   One script per process — the stored MAME logs come from the runner's
   solo re-verification, so sharing a live chip with other scripts
   compares against the wrong state.
3. Does the record carry `to=1`?  Then an aperture beat never
   terminated; that is a bus error on the board and outranks everything
   else in the log.
4. Only then read the SYNC deltas.

## Fuzzer self-validation

Re-injecting the motivating IDENTIFY-as-opcode bug (reverting
`c96_sel_strip` in a scratch copy of scsi.v) is flagged by the clean
profile within 8 seeds with the exact historical signature
(SELECT_ATN + IDENTIFY + WRITE(10) → STATUS/CHECK CONDITION instead of
DATA_OUT), while the same seeds pass on the pristine RTL.

### The Sad Mac 0F02 bug, re-injected (2026-08-19)

The point of the aperture widening is that the pre-widening fuzzer was
*structurally* unable to see the bug that reached hardware.  That claim
is now backed by experiment rather than by reading.

Injection: a scratch copy of `peripheral_bus.v` with

```verilog
assign scsi_dma16_lo_beat = 1'b0;   // pre-f63207b behaviour
```

i.e. the low half of a split 16-bit aperture read re-runs its own DRQ
check instead of inheriting the grant — exactly the pre-fix code.  The
widened harness was rebuilt against it and both executors were run over
the same generated scripts.

Result over 8 seeds that contain a DRQ-checked 16-bit drain
(6 10 11 12 13 15 16 18):

| build | `to=1` records (aperture beat never terminated) | seeds diverging |
|---|---|---|
| pristine | 0 | — |
| bug re-injected | 8 | 6 12 13 15 16 18 |

and the record shape is the hardware signature verbatim — the **last**
word of the chunk stranded:

```
DRB16 8 got=8 data=7f8c99a6b3c0cddae7f4010e1b280000 to=1
```

`to=1` is `peripheral_bus`'s ack watchdog answering SLVERR, which on the
board is the bus error that painted the Sad Mac `0F 02` at
fa=`0x50f4f100`.  93 of the first 200 seeds contain a DRQ-checked 16-bit
aperture read, so this is routine coverage, not a lucky seed.

The pre-widening fuzzer scores 0 on the same experiment for two
independent reasons, either of which alone is fatal: its script language
had no 16-bit op at all, and `scsi_ctrl_in` was hardcoded to 0 so the
DRQ check that the bug lived in was never armed.

Reproduce with `tools/fuzz/` plus a scratch RTL tree; the injection is a
one-line edit and the comparison needs no MAME run (the pristine build's
agreement with MAME on those lines is established by the normal fuzz
run).

## Known blind spots — history, and what is closed

Recorded 2026-08-19 after the Sad Mac `0F 02` bus error at ROM PC
`0x4089931c` reached hardware with `make fuzz-scsi` green at its 56/60
baseline (46/60 earlier the same day, before three agents closed ten
seeds).  That bug was a **16-bit pseudo-DMA aperture** divergence: MAME
DRQ-checks once per host access and then pops both bytes atomically
(`dafb.cpp:1000-1010` -> `ncr53c94_device::dma16_r`, `ncr53c90.cpp:1325-1350`),
while `peripheral_bus.v` split one `move.w` into two independently
DRQ-gated byte beats.  Under LBTM (`config3` bit 2, which the Q700 ROM
sets at PC `0x40899120`) the odd intermediate FIFO occupancy has DRQ low,
so the second beat never acked.

The fuzzer could not have caught it, for **six independent reasons** —
each verified in the source, not assumed (#1-#4 found while chasing the
boot bug; #5-#6 while chasing the hang that followed it).  #1-#5 were
closed the same day by the widening below; #6 is characterised and left
open with a stated reason.

| # | Blind spot | Status |
|---|---|---|
| 1 | **Access width.**  Both executors drove the aperture with BYTE accesses only (`tb_scsi_fuzz.cpp:186`, `mame_scsi96_fuzz.lua:47`).  The script language had no 16-bit op, so `dma16_r`/`dma16_w` — the path the ROM uses — was unreachable. | **CLOSED** — `DR16`/`DRU16`/`DW16` + blind `DRB16`/`DRUB16`/`DWB16`.  MAME side `read_u16`/`write_u16` at the aperture (`dma16_swap_r`/`dma16_swap_w`); RTL side a genuine AXI `arsize`/2-strobe beat through `peripheral_bus.v`.  The generator hits both MAME degradation corners on purpose: `dma16_r` at `fifo_pos < 2` and `dma16_w` at `fifo_pos > 14 \|\| tcounter == 1`. |
| 2 | **The DRQ-check was disabled.**  `tb_scsi_fuzz.cpp:363` set `scsi_ctrl_in = 0` for the whole run, so every hold-off path in `scsi.v` and `peripheral_bus.v` was dead code.  The Sad Mac bug lived there. | **CLOSED** — `CTRL <v>` op drives `scsi_ctrl_in` (RTL) and the real DAFB register at `0xf9800024` (MAME).  Both the read-check bit (7) and the write-check bit (8) are exercised; `normalize()` resets it to 000 so it cannot leak between blocks or seeds. |
| 3 | **`peripheral_bus.v` was not in the DUT.**  The fuzz top was `scsi.v` + `vhdd_sd`; the word-splitting FSM that WAS the bug was never instantiated. | **CLOSED** — the default fuzz top is now `tb/tb_pb_scsi.v` (real `peripheral_bus.v` + real `scsi.v` + `vhdd_sd`, AXI4-driven).  The old shape survives as `make fuzz-scsi-direct` for attribution. |
| 4 | **`DR`/`DW` were DRQ-paced**, so under LBTM they stopped at `fifo_pos == 1` and could not reproduce the ROM's blind 8×`move.w` drain. | **CLOSED** — the `DRB…`/`DWB…` family issues beats with no DRQ poll.  Because MAME advances zero emulated time inside a Lua op burst, a blind burst is only comparable as far as one already-staged FIFO load, so the generator settles first and caps a blind burst at 16 bytes.  That is the ROM's own shape (MAME trace record #1612 shows `fifo=16` + TC0 *before* the first DMAR of #1613), not a workaround. |
| 5 | **`cmd=` was differ-masked** unless `--strict`, so command-queue retirement was invisible by default — and MAME's `command_w` *drops* a command with `S_GROSS_ERROR` at `command_pos == 2` and *queues without starting* at 1, which is exactly the shape of a wedge. | **CLOSED** — compared by default; `--mask-cmd` opts out.  Because the field is genuinely noisy, the runner reports it in a second tier: a seed whose *only* divergence is `cmd=` is listed separately (`fails_cmd/`) and never shadows a real aperture/FIFO/DRQ divergence further down the same log. |
| 6 | **`SETTLE` brackets every command**, so latched-vs-live races (e.g. `xfr_phase` captured at dispatch while the target is mid-transition) are structurally unreachable in a generated script. | **OPEN** — see below. |

### Why #6 is still open (measured, not assumed)

The obstruction is the MAME executor's time model, and it is sharper
than "MAME cannot step below a frame":

* Every Lua `write_u8`/`read_u8` in this harness executes at a **single
  emulated instant**.  A burst of ten register writes advances the
  emulated clock by exactly zero, while the RTL side advances one AXI
  transaction (~10 pb cycles) per write.  So an unsettled comparison
  does not measure a latched-vs-live race in the DUT — it measures which
  executor got to advance time, which is harness noise with the same
  signature as the bug we would be hunting.  This is not hypothetical:
  the first widened run produced 14 seeds whose first divergence was a
  blind read where MAME had an empty FIFO and the RTL a full one, purely
  from that asymmetry.
* The only deterministic time step available to a `register_frame_done`
  driven coroutine is `coroutine.yield()` = one frame (~16.6 ms), four
  orders of magnitude coarser than the races in question.
* MAME 0.285 *does* expose a finer primitive — `emu.wait(seconds)`
  (`luaengine.cpp:835`), which is `sol::yielding` and registers the
  thread in the engine's `m_waiting_tasks`.  It is unusable **as this
  executor is currently structured**: our coroutine is resumed by our
  own `emu.register_frame_done` callback, so after an `emu.wait` yield
  both the engine and our callback would resume the same thread and one
  of the two resumes hits a non-suspended coroutine.

The concrete path to closing #6 is therefore a self-contained follow-up,
not a tweak: convert the executor to an **engine-managed** coroutine —
drop the `register_frame_done` driver, start the coroutine once, and
express *both* `SETTLE` and a new matched `WAIT <us>` op as `emu.wait`.
`WAIT` would then advance both sides by the same emulated interval
(RTL: `us × f_pb` cycles) with neither side quiescing, which is what
makes a latched-vs-live disagreement expressible. Until that refactor
lands, an unsettled SYNC in this harness would generate false
divergences faster than real ones, so the generator does not emit one.

`GAP` remains the RTL-only half of that knob (perturb the DUT's timing
without the golden model moving), and it is sound only because every
comparison point is settled.
