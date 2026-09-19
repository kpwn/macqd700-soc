# 2026-09-04 — p128 vs p123: branch-delta verdict, and how far p128 actually boots

Two tracks. Track A asks *why* the `0x4084BECE` wedge reproduces on the p123
bitstream but not on p128. Track B asks *how far p128 gets and what stops it
next*. Track B produced the more valuable result.

Nothing here claims the wedge is fixed. p128 working remains a data point about
one bitstream.

---

## Bitstream provenance (established before anything else)

| | p123 | p128 |
|---|---|---|
| SoC commit | `4ffbb6f3` (`feat/m68k040ooo-socket-integration`) | `96670339` (`debug/lseu-psrca-ila-probes`) |
| SoC RTL ancestor | `1cee143c` | `1cee143c` |
| cpu040 **recorded** submodule pointer | `7d74ba33` | `7d74ba33` |
| cpu040 **actually built** | `7d74ba33` | **`1532cd2d`** |
| `ENABLE_ILA` | **0** | **1** |
| `fpga_top.ltx` contains | `vio` only | full `debug_ila` core, 63 probes |
| build_id | `0x1e0fb901` | `0xd9eb79fa` |
| bitstream | `…/m68k040ooo-integration/build/vivado_p123_100mhz/fpga_top.bit` | `…/lseu-psrca-ila/build/vivado_p127_lseu_ila/fpga_top.bit` |

**Self-retraction.** My first pass reported "the `cpu040` submodule pointer is
byte-identical, so there is zero core delta". That was wrong. The *recorded*
pointer is identical because the ILA branch never bumped it; the p128 build
consumed the cpu040 **working tree**, which is on core branch
`debug/lseu-psrca-ila-probes` at `1532cd2d`. Always read the submodule worktree
HEAD, not `git ls-tree`.

Both `buildinfo` files are otherwise identical (same part, `full_impl`,
`real_mig`, `l2c_enable=1`, `target_freq_mhz=100`, `boot_rom_sectors=2048`).
`ENABLE_ILA` is *not* recorded in `buildinfo` — the `.ltx` contents are the
witness.

---

## Track A — candidate (3), the RTL delta: **VERDICT — real delta, but purely observational**

### SoC side (`1cee143c` → `96670339`)

Files touched, all probe/debug/build plumbing:

* `synth/vivado.tcl`, `Makefile` — `general.maxThreads` made overridable via
  `VIVADO_MAX_THREADS`. Build-host knob, no design effect.
* `synth/debug_ila.tcl` — ILA IP probe count 57 → 63, six new 32-bit probes.
* `rtl/soc/fpga_top_debug_vio.vh` — six new `MARK_DEBUG` wires → `probe57..62`.
* `rtl/soc/fpga_top_debug_ctrl.vh` — the large hunk. Rounds 4-10's
  `dbg040_*` port connections **removed** and their wires tied to 0; round 11's
  six connections added. Reason recorded in the file: rounds 4-10 referenced
  cpu040 ports that do not exist on `fmax-closure-fanout`/`7d74ba33`, so
  `ILA_ENABLE + CPU_M68K040` had been unbuildable since round 4 — which is
  exactly why p123 had to be built with `ENABLE_ILA=0`.

No functional SoC logic changed.

### Core side (`7d74ba33` → `1532cd2d`, 8 commits)

Synthesizable files only:

* **`LsEuPlugin.scala` (+52)** — five new combinational taps (`dbgLsS1Base`,
  `dbgLsS1Va`, `dbgLsS1Pc`, `dbgLsS0Base`, `dbgLsPack`) off existing flops and
  off `rdBase.data`, which already fans out to `base0`. No new state, no
  consumer in the datapath.
* **`IssueQueuePlugin.scala` (+75)** — two new sticky debug flops
  (`dbgIqColdMismatchSticky`, `dbgIqCollisionSticky`) with no datapath consumer,
  plus one structural edit worth checking carefully: for the LS port (`k == 3`)
  the call `coldRead(piped.payload)` is replaced by an inlined body. `coldRead`
  is `def coldRead(h) = Mux(h.coldWay, coldWay1.readAsync(h.robId), coldWay0.readAsync(h.robId))`
  (line 233-234); the inlined form is `w0 = coldWay0.readAsync(robId)`,
  `w1 = coldWay1.readAsync(robId)`, `cold = Mux(coldWay, w1, w0)`. **Semantically
  identical** — same two `readAsync` instantiations, same Mux polarity, no third
  read port added.
* **`RobPlugin.scala` (+16)** — a `macroLast` field added to `CommitObs`. The
  whole `commitObs` block sits inside `GenerationFlags.simulation`, so it is
  `null` in every synth/GenVerilog build. **Zero synthesis impact.**
* **`M68kCore.scala` (+30) / `SocketTop.scala` (+15)** — six new `out` ports each,
  pure plumbing.

Remaining commits are docs and lock-step tests (`ExecuteLockStepSpec`,
`WhiteboxCapture`, `FuzzLockStepSpec`, `MiHangTraceSpec`).

### Verdict

**There is a source delta between p123 and p128, contrary to the recorded
submodule pointer, but it contains no functional logic change.** Every hunk is
a debug tap, a new `out` port, a sim-only field, or a semantically identical
inlining.

**Important caveat, and it limits the whole experiment:** the delta is *not*
physically neutral. p128 additionally instantiates a 63-probe, 4096-deep ILA
plus `MARK_DEBUG` attributes that pin nets and forbid optimisation, and the new
taps add fanout to `psrcAValid`/`psrcA`/`robId`/`pushPort`. Candidates (2)
"probe insertion changed fanout/placement" and (3) "RTL delta" are therefore
**not separable between these two bitstreams** — they differ by the presence of
the entire ILA, not by a probe or two.

### Timing, with domains named

`timing_summary.rpt`'s design-level WNS belongs to the **`dbg_hub` JTAG clock**
and is misleading. The **CPU clock** margins are p128 **+0.245 ns**, p123
**+1.035 ns**; both report 0 failing setup and 0 failing hold endpoints. p128
has *less* CPU-clock margin yet works, which is inconsistent with simple setup
marginality — as is the wedge reproducing identically at 100, 25 and 12.5 MHz.

### The separating build — **QUEUED, NOT FINISHED**

Worktree `/home/qwertyoruiop/macqd700-soc-worktrees/p123-reseed`, branch
`debug/p123-reseed-placement`, SoC `4ffbb6f3` + cpu040 `7d74ba33` +
`ENABLE_ILA=0` — byte-for-byte the p123 source.

**Vivado ML's `place_design` has no `-seed` option** (unlike the old ISE/PAR
flow). The supported lever for drawing a different placement from an identical
netlist is the placer *directive*, which `synth/vivado.tcl:2195` already exposes
as `PLACE_DIRECTIVE` for exactly this A/B purpose. Default on this branch is
`AltSpreadLogic_high`; this run draws `Explore`. An identical rebuild would be
pointless — Vivado is deterministic given identical inputs and would reproduce
p123 bit-for-bit.

Launcher: `run_reseed_build.sh` in that worktree. It polls rather than nesting a
`flock`, because `make impl` takes the lock **non-blockingly**
(`flock -n -E 75`, Makefile:2939) and an outer `flock` on the same file would
deadlock against it.

At the time of writing, another agent's `full_impl` (pid 1508716, ~56 min in,
in `route_design` Phase 3.1 timing verification) still holds
`/var/tmp/m68k-ooo-vivado.lock`. **The reseeded result is not available and no
conclusion about candidate (1) placement lottery can be drawn yet.**

---

## Track B — how far p128 actually boots

Method: attach with `JTAG_REPL_NO_PROGRAM=1` (preserves board state), then
`vio-hard-reset` and sample `pc_live` on a timestamped loop with **no halting**
until the trajectory was established.

**Self-retraction.** My first attempt halted the running board and then
`halt-release`d it. That perturbed it: `pc_live` went to `0x00000000` and
`inst-count` froze at `783843195` across 30 s. Every quantitative result below
comes from the *subsequent* clean cold-reset run.

### Which ROM is on the card — verified on silicon

Read through the live ROM window rather than by `sd-verify` (which would
reprogram the FPGA with the provisioning bitstream):

| ROM offset | address | read | meaning |
|---|---|---|---|
| `0x00000` | `0x40800000` | `0x420d8602` | `calibration-fix` checksum |
| `0x00888` | `0x40800888` | `0x303ce799` | `calibration-fix` **present** (stock is `51C8FFFE`) |
| `0x02FD4` | `0x40802FD4` | `0x4efa16d2` | via-alias-corruption **absent** (stock) |
| `0x06782` | `0x40806780` | `0x67062269` | zonewalk **absent** (stock `2269`) |
| `0x03234` | `0x40803234` | `0x00000000` | machine-descriptor-slot4 **absent** (stock) |
| `0x98ADC` | `0x40898ADC` | `0xe38a2602` | scsi-open-delay **absent** (stock `E38A`) |
| `0x00270` | `0x40800270` | `0x41f90000` | bsrw-shim **absent** (stock `41F9`) |
| `0x03174` | `0x40803174` | `0x4efa1534` | io-oob-alias **absent** (stock) |

**The card carries `calibration-fix` and nothing else — the one ever-approved
patch. The baseline is valid.** This agrees with the documentary record
(Part 120 and Part 125's restore), and is now confirmed independently on
hardware. Host-side copy: `build/roms_patched/rom_calibonly_restore.rom`,
md5 `1be5903b45b4c1dc012ec103e42ea5d0`.

### Boot trajectory (cold reset → t ≈ 40 s)

| t (s) | `pc_live` | what |
|---|---|---|
| 0.3 – 10.1 | `0x408472f8` … `0x40847332` | ROM RAM test loop — **past the `0x4084BECE` wedge** |
| 10.4 | `0x4080607c` | (`4A04 67FA` = `tst.b d4; beq.s -6`) |
| 10.6 – 11.5 | `0x4080a8e6` ×4 | **ADB Manager init busy-wait**, `btst #5,0x015d(a3); bne.s 0x4080a8e6` — already documented at `docs/rom_boot_bringup.md:278-279` |
| 11.7 | `0x40809bbc` | `0x4E73` = **RTE**, preceded at `0x40809bb8` by `0x4CDF0F0F` = `movem.l (a7)+,d0-d3/a0-a3` — a handler epilogue |
| 12.0 → | `0x001809f2` … `0x002fa062` | control leaves ROM for low RAM; PC sweeps **monotonically upward with wraps** across ≈1.5 MiB, every sample ≡ 2 (mod 4) |
| ≈ 11 min | `0x00300018` / `0x0030001e` | terminal state |

The terminal PC pair reproduced across two independent boots (it was also where
the pre-reset instance had come to rest), so the endpoint is deterministic.

### The terminal state is an exception **livelock**, not a hang

`exc-ring` at the endpoint (`exc_count` reads 0 and is not to be trusted; the
ring is):

```
exc[29] vec=0x02 pc=0x50300000 fa=0x50300000 handler=0x00300000
exc[28] vec=0x04 pc=0x0030001e fa=0x00000000 handler=0x50300000
exc[27] vec=0x02 pc=0x50300000 fa=0x50300000 handler=0x00300000
exc[26] vec=0x04 pc=0x0030001e fa=0x00000000 handler=0x50300000
tally: 0x0030001e × 16, 0x50300000 × 16
```

A closed two-exception cycle: **illegal instruction (vec 4) at `0x0030001e`
→ garbage handler `0x50300000` → access fault (vec 2) there → garbage handler
`0x00300000` → executes to `0x0030001e` → repeat, forever.** `dbl_fault=0`
because each exception does vector successfully; it is a livelock, not a double
fault. `inst-count` at the endpoint = **632,102,773** on the fresh boot
(783,843,195 on the earlier longer-running instance) — hundreds of millions of
retirements, so the core is running at full speed the whole time.

Architectural state at the endpoint, **byte-identical across two independent
cold boots** except `A7`/`ISP`, `D0` and `PC`:

```
D2=0x00100000 D3=D4=0x0000ffff D5=0x0000ffc3 D7=0x08000000
A0=0x4080360c A1=0x408031b0 A2=0x50f01c00 A4=0x000088b0 A6=0x40800000
SR=0x00002710  VBR=0x00400662  CACR=0x00008000  TC=0x0000c000
ITT0=DTT0=0xf900c060  ITT1=DTT1=0x807fc040  SRP=0x03fffa00
A7 = ISP = 0x79a75056 (run 1) / 0xb03dd79a (run 2)   <- runs away: every
                                                        exception pushes a frame
```

### Root mechanism — MEASURED

`VBR = 0x00400662`. A plain JTAG-AXI read of the vector table it points at gave:

```
0x00400660 = 00000028   0x00400664 = 6d020028   0x00400668 = 00000030
0x0040066c = 0000001a   0x00400670 = 40885030   0x00400674 = 00000030
```

Decoded at `VBR+0x08` and `VBR+0x10` (byte offsets `0x0040066A` / `0x00400672`)
this yields **`0x00300000`** and **`0x50300000`** — *exactly* the two handlers
the exc-ring shows the core fetching. Two independent values matching exactly
is not chance: the core's vector fetch consumed precisely this memory image.

Then `coherent-dump 0x00400640 32` — which does a **halted D-cache push** before
reading — returned entirely different data at the same addresses, and a
**plain re-read afterwards agreed with it**, proving the push physically changed
DRAM:

```
before push: 00000028 6d020028 00000030 0000001a 40885030 00000030 0000c1d0
after  push: 00000000 20004084 6c064084 6a804084 6a904084 6a964084 6a9c4084
```

Laid out as bytes from `0x00400662`, the pushed image is a perfectly ordinary
vector table of ROM stubs 6 bytes apart:

```
VBR+0x08 (0x0040066A) -> 40 84 6a 80 = 0x40846a80   vec 2, access fault
VBR+0x10 (0x00400672) -> 40 84 6a 96 = 0x40846a96   vec 4, illegal instruction
VBR+0x14              -> 0x40846a9c, then 6aa2, 6aa8, 6aae …
```

**So `VBR` is not corrupt and the vector table is correct.** The ROM built a
valid table; those stores were sitting **dirty in the D-cache and had never been
written back**; the core's exception vector fetch read **stale DRAM** and got
garbage handlers.

`CACR = 0x00008000` → bit 31 `DE` = **0 (data cache disabled)**, bit 15 `IE` = 1.
Dirty D-cache lines existed anyway, and only an explicit CPUSH made the writes
visible.

### Where the RTL asymmetry is (hypothesis, **not** proven)

`ExceptionUnit.scala:786` already folds `CACR.DE` for the exception sequencer —
and its own comment describes this exact failure mode, including *"Same hole for
a VBR vector the program rewrites after an earlier exception already pulled that
line in"*:

```scala
val excCacheMode = Mux(ss.cacr(31), CacheMode.WRITETHROUGH, CacheMode.INHIBITED)
```

It is consumed at `:808` (`stoCmodeReg`) and `:859` (`ldoCmodeReg`), and is
**identical in `7d74ba33`** — so the exception path is fixed.

The **program** store path is where the fold appears to be missing.
`dcacheEnabled` (`= ss.cacr(31)`, `RobPlugin.scala:2200`) has only two consumers
in `LsEuPlugin.scala`: the load command at `:2912` and `txEffectiveCmode` at
`:2434`. The store-queue allocation does **not** go through either:

```scala
LsEuPlugin.scala:1083   sq.io.alloc.payload.cacheMode := p3Ctx.cmode   // raw page mode
LsEuPlugin.scala:1634   dst.cmode  := p4Ctx.xlate.cmode                // not DE-folded
LsEuPlugin.scala:1656   dstA.cmode := p4Ctx.xlate.cmode
LsEuPlugin.scala:1664   dstB.cmode := p4Ctx.xlate.cmodeB
LsEuPlugin.scala:2468   txOut.cmode := Mux(txSecond, txCmodeA, txEffectiveCmode)
                                            // ^ split second-half arm not folded
```

That is consistent with everything measured: with `DE=0`, a store to a page the
MMU marks COPYBACK still allocates and dirties a line, invisible to DRAM and to
the (correctly INHIBITED) exception vector fetch until a CPUSH.

**I did not run a simulation to prove which of these paths carried the ROM's
vector-table stores.** This is a hypothesis consistent with the measurement, not
a demonstrated root cause.

### Prior-art check

Per the standing rule, `docs/` was grepped before deriving anything:
`0x00300018`, `0x0030001e`, `0x50300000`, `0x40809bbc`, `0x4080607c`,
`0x00400662` — **no hits**, this endpoint is new. `0x4080a8e6` **is** documented
(`docs/rom_boot_bringup.md:278`, `docs/rom_logo_path.md:33`) as the ADB Manager
init busy-wait.

---

## Explicitly not run

* The reseeded p123 build — queued behind another agent's `full_impl`, not
  started. **No conclusion on candidate (1).**
* No ILA capture. p128's ILA was never armed or triggered in this session;
  the `psrcA`/IQ-cold probes were not read.
* No simulation of the store-path `CACR.DE` hypothesis.
* No `sd-verify` of the card (it reprograms the FPGA); ROM identity was
  established by reading the live ROM window instead.
* The `0x00180000…0x00300000` sweep phase between the RTE and the terminal
  livelock was characterised only by PC sampling; I did not determine what
  executes there.
