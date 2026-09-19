# Gameplan — m68k-ooo end-to-end development plan

This is the master sequencing doc. Where we are, what's next, the order to do it
in, and what success looks like at each milestone.

Companion docs:
- [core_gaps.md](core_gaps.md) — concrete list of what's missing in the core
- [peripheral_arch.md](peripheral_arch.md) — platform/peripheral architecture
- [optimisation_roadmap.md](optimisation_roadmap.md) — phase-4+ performance work

---

## Where we are (snapshot, end of phase-1 bring-up)

**Core**
- 32-entry ROB, 48 physical regs, 2 CDB, 8-entry IQs, single-issue LSU.
- ~30 instructions decoded; ALU has ~60 op encodings live.
- BTB phase-1 wired (decode-time predict, commit-time train).
- DBcc + BSR + RTS landed (BSR cracks to PUSH+BRA; RTS is single-uop).
- cRAT shadow + flush rollback works (RAT recovers correctly on mispredict).
- Multi-uop crack mechanism in decode (used by BSR; ready to be used by MOVEM).
- ~25/50 directed tests passing on `main`. Functional shake-out is the bottleneck.

**Platform**
- All Mac peripherals are stub modules.
- `wip/ddr-peripheral-bringup` has substantial `glue.v` work + AXI peripheral bus
  + DDR MIG wrapper + KU5P XDC, but isn't merged.
- HDMI bringup (separate Vivado project) verified working at 1080p60 on board.
- SD card bringup project (`sd-hdmi-bringup/`) scaffolded, not yet built.

**Build / sim**
- Verilator 5.042. 50 directed asm tests, ~25 passing.
- Build target works on macOS via /tmp/m68k-ooo-build (rsync workflow).
- No FPGA bitstream produced yet for the full core (HDMI standalone only).

---

## Phase 1.5 — close the bring-up gap (1–2 weeks of agent work)

**Goal:** all 50 directed tests pass, full integer ISA decoded, CCR renamed,
multi-outstanding loads merged. This is the "core is a credible OoO 68040"
milestone.

| Item | Owner | Status |
|---|---|---|
| CCR rename (CCR-PRF, widened CDB, commit reads ccr_prf[rob.ccr_pdst]) | primary agent | planned (microarch.md) |
| Decode coverage: shifts, rotates, bit ops, EOR, NOT, NEGX | sonnet sub-agent | partial in ALU, needs decode |
| Multi-outstanding-load LSU merge from `wip/ddr-peripheral-bringup` | primary | rebase-and-reimplement session |
| Functional tests for unvalidated ISA (shifts, X-flag chains, CMPI corners) | sonnet | listed in next_iteration.md |
| RAS module + integration | sonnet then primary | `ras.v` standalone, then if_stage wire |
| MULS/DIVS routing through new mul_div.v | sonnet then primary | mul_div.v exists as stub |
| LINK / UNLK decode | sonnet | next_iteration.md side quest C |

**Exit criterion:** `make test` passes 50/50. `make synth` returns WNS ≥ 0 ns
@ 200 MHz on the existing core (no peripherals yet).

---

## Phase 2 — Mac ROM cold-start (3–4 weeks)

**Goal:** the real Quadra ROM image runs from reset to either "happy Mac"
(boot disk found) or "sad Mac" (deterministic failure code displayed). This
is a brutal milestone — the ROM exercises ~80% of the architecture.

### What the ROM does in its first 50,000 instructions

1. Resets supervisor SP from vector 0, jumps to vector 1 (reset PC)
2. Reads VIA1 ORB to determine memory configuration (overlay bit, SIMM probe)
3. RAM sizing pattern: writes/reads test patterns at increasing addresses
4. ROM checksum: walks the entire ROM image, validates 32-bit sum
5. Initializes the system stack at top of RAM
6. Probes for ADB (keyboard) on VIA1
7. Probes for SCSI devices via NCR 5380 register polling
8. Searches for a boot block on each SCSI device
9. Loads boot block, jumps to it

### What we need built before this milestone

**Core additions (see core_gaps.md for full list):**
- A-line / F-line trap path (Mac OS Toolbox is 100% A-line traps)
- Supervisor mode + USP/SSP/ISP separation
- MOVEC for VBR, CACR, ITT0/1, DTT0/1 (ROM sets these early)
- CPUSH / CINV (cache flush — ROM uses to switch out of overlay mode)
- MMU stub with ITT0/DTT0 transparent translation (covers ROM and I/O ranges)
- Address-error and bus-error vectors
- Privileged-instruction trap
- TRAP #n vectors (Mac OS uses TRAP #15 for debugger entry)

**Peripheral additions:**
- GLUE address decoder + AXI→pb bridge
- VIA1 minimal: ORB overlay bit, Timer 1, IFR/IER, irq line
- VIA2 stub (returns 0xFF on ORB; sinks writes)
- SCSI minimal: NCR 5380 register file, RESET phase, Selection that returns
  "no device" on all IDs except 0
- SCC stub (returns 0 on read; sinks writes)
- ROM loader path: for sim, flat MemModel + ROM image binary
- Reset sequencer with `init_done` pin (already designed)

### Sub-milestones inside phase 2

| # | Milestone | Indicator |
|---|---|---|
| 2.1 | First instruction commits from ROM @ 0x40800000 | `dbg_committed > 0` |
| 2.2 | RAM sizing loop completes | execution leaves the test-pattern PC range |
| 2.3 | ROM checksum passes | execution reaches the post-checksum PC |
| 2.4 | First A-line trap dispatched cleanly | exception entry to vector 0x28 with correct stack frame |
| 2.5 | First VIA1 register read returns plausible value | observed at AXI write back to D-reg |
| 2.6 | First SCSI command issued | NCR 5380 Initiator Command Register written with selection bits |
| 2.7 | Sad Mac displayed (or happy mac if boot disk found) | video framebuffer contains expected pattern |

**Exit criterion:** the ROM executes deterministically through 2.7. Whether it
finds a boot disk or fails gracefully matters less than the path being
debuggable end-to-end.

---

## Phase 3 — boot to System 7 desktop (2–3 weeks after phase 2)

**Goal:** Mac OS loaded from SD-backed SCSI disk, runs through INIT loading,
draws the desktop, accepts keyboard input.

### What we need built

**Core additions:**
- Real D-cache (4KB 4-way write-back) — the boot path is cache-sensitive
- Real I-cache (4KB 4-way) — boot is cache-friendly but instruction footprint matters
- MMU with page-table walker + 64-entry ATC
- Interrupt levels 1–7 with autovector dispatch
- All A-line trap dispatch (RTE back to user mode)
- Self-modifying code support (I-cache invalidate on store to instruction line)
- FPU bring-up (at minimum: FNOP, FMOVE, FADD, FSUB, FMUL — INITs probe FPU)

**Peripheral additions:**
- Full VIA1: ADB over SR shift register, Timer 1 + 2, RTC
- Full VIA2: NuBus interrupt aggregation (we have no NuBus, just stub the slots)
- SCSI: full NCR 5380 phase machine (Selection, Command, Data In/Out, Status,
  Message), backed by SD card sectors (LBA + 8192 bias for ROM area)
- SD card SPI controller (init + multi-block read/write)
- Video framebuffer at 0x60000000 or wherever Mac OS expects it
- Keyboard via ADB (deferred: can stub initially)

### Sub-milestones

| # | Milestone |
|---|---|
| 3.1 | SCSI Selection phase identifies disk on ID 0 |
| 3.2 | First disk sector read, returned to ROM |
| 3.3 | Boot block loaded and executed (System 7 boot loader) |
| 3.4 | "Welcome to Macintosh" screen drawn |
| 3.5 | INIT chain executes (no INITs installed initially — should fly through) |
| 3.6 | Desktop drawn, mouse cursor visible |
| 3.7 | Keyboard input accepted (move cursor with arrow keys via ADB) |

**Exit criterion:** can interact with the desktop. This is the project's
"functional MVP" — the world's first OoO 68040 booting real Mac OS.

---

## Phase 4 — performance push (3–4 weeks)

**Goal:** hit the targets in CLAUDE.md: 200 MHz Fmax post-route, sustained
IPC ≥ 1.5 on Mac OS workload, ~10× Quadra 840AV throughput.

See [optimisation_roadmap.md](optimisation_roadmap.md) for the menu of
techniques. The high-impact ones:

1. **Wide front-end** — 2-way decode + dispatch, 2-way predecoder, RAT
   dual-port. Required to get past IPC 1.0.
2. **Bigger structures** — ROB to 64, IQs to 16, PRF to 96. Removes most
   structural stalls; cost is mostly BRAM.
3. **Two-level branch predictor** — gshare with 4K entries replaces bimodal.
4. **Memory disambiguation / load-store forwarding** — speculative loads
   past unresolved stores, with squash on alias detection.
5. **Move elimination** — MOVE Dn,Dm at rename: just point the destination
   RAT entry at the source physical reg; zero ALU ops.
6. **Zeroing idiom** — CLR Dn / SUB Dn,Dn / MOVEQ #0,Dn: rename to a
   pinned phys-zero register; zero ALU ops.
7. **Larger caches** — 16KB I+D (4× 68040 spec) to fit Mac OS hot loops.

**Exit criterion:** Quadra-ROM Drystone or DhrystoneMac ≥ 10× real Quadra 840AV
on identical Mac OS image.

---

## Phase 5 — beyond the MVP (open-ended)

This is the "what to push if it's all working and you want to keep going" list.
See [optimisation_roadmap.md](optimisation_roadmap.md) for full details.

Themes:
- **Microarchitectural exotica**: trace cache slice, value prediction on hot
  loads, indirect branch predictor, memory renaming on stack accesses.
- **System-level**: L2 cache (real SRAM-backed) — **note (2026-07-16):
  `docs/memhier.md` tracks this as its own internal "phase 4" (i.e.
  right after L1s land), a different, more granular numbering scheme
  than this doc's project-wide Phase 1-5; treat `memhier.md` as
  authoritative for L2 sequencing/design, this line as a pointer only.
  L2 is now front-of-MIG (all crossbar masters, not just CPU L1
  misses) and gated on real DDR4 hardware calibration in addition to
  real L1s — see `memhier.md`'s L2 section for the full design.  Also
  now the mechanism behind a confirmed (not speculative) future
  16/24bpp QuickDraw push via DDR4-backed VRAM — see `memhier.md`'s
  "16/24bpp QuickDraw" note.  SMT (run two Mac OSs at once
  on one core — Apple did this with the Macintosh II's MMU but never with HW
  threading).
- **ISA stretch**: 68060-class additions (FPU instruction set expansion,
  branch hints, MOVE16) without breaking 68040 ABI.
- **Real-hardware**: tape out as ASIC (28nm cost-down), sell as a
  retrocomputing accelerator, or as a Tang Mega 138K-Pro standalone product.
- **Mac OS contributions back**: write FPGA-aware drivers, optimised System
  enabler patches, faster QuickDraw paths.

---

## Risk register (read this before scheduling)

| Risk | Severity | Mitigation |
|---|---|---|
| Quadra ROM checksum fails due to subtle CCR/MMU bug | HIGH | Build checksum-only test harness in phase 2.3; iterate on tiny ROM region first |
| MMU walker timing breaks at 200 MHz | MED | Pipeline the walker into 3 stages, accept a 2-cycle TLB miss |
| LSU multi-outstanding-load merge regresses BSR/RTS | MED | Tested by `dbcc_loop` and `bsr_chain` benches before merge lands |
| VIA1 timing wrong → ROM hangs in autodetect | HIGH | Capture cycle-accurate Quadra trace from MAME, compare |
| SCSI phase FSM doesn't match what NCR driver expects | HIGH | Mirror reference open-source SCSI SD card adapter (SCSI2SD project) |
| HDMI pixel clock + DDR4 + CPU clock domain crossings cause Fmax loss | MED | All CDC via async FIFOs; lint with Vivado CDC report |
| FPU coverage too sparse for INIT chain | LOW | Many INITs detect FPU via FNOP; full FPU is phase-3 anyway |
| Cache coherence: DMA write to RAM not seen by CPU | HIGH | Implement CPUSH/CINV correctly; consider hardware snooping if SCSI DMA matters |
| Self-modifying code (Mac OS does this for stub patching) | MED | I-cache invalidate on D-cache write to same line |

See [core_gaps.md](core_gaps.md) for the detailed correctness gaps these
arise from.

---

## Parallelisation strategy (continuing from next_iteration.md)

The next_iteration.md doc already tabulates phase-1.5 parallel work. For
phases 2–3 the natural splits are:

- **Primary agent**: core changes (CCR rename, MMU, exception path, supervisor
  mode). Touches the hot files (rob, commit, alu, m68k_core).
- **Sub-agent A**: peripheral implementation (VIA1, VIA2, SCSI, SCC). Self-
  contained in `rtl/mac/`. Can run in parallel with anything else.
- **Sub-agent B**: tests (each new instruction or feature gets a directed
  test). Pure additions to `tb/tests/asm/`.
- **Sub-agent C**: tooling (decode coverage report, Mac ROM trace tool, FST
  visualisation helpers). Standalone scripts in `tools/`.
- **Remote FPGA agent**: synth/PnR iteration on the latest core, timing
  closure, real-hardware bring-up.

Every phase ends with a primary-agent integration session that lands all the
sub-agent work in dependency order. The cheat-sheet table format from
next_iteration.md scales to keep file ownership clear.

---

## Cadence

Estimated calendar time assuming continuous agent work:

```
Phase 1.5  ┃▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░  ~2 weeks
Phase 2    ┃░░░▓▓▓▓▓▓▓░░░░░░░░░░░░░░░░░  ~4 weeks
Phase 3    ┃░░░░░░░░░░▓▓▓▓▓▓░░░░░░░░░░░  ~3 weeks
Phase 4    ┃░░░░░░░░░░░░░░░░▓▓▓▓▓▓▓░░░░  ~4 weeks
Phase 5    ┃░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓  open
```

**Hard dependencies**: phase 2 needs phase 1.5 done, phase 3 needs phase 2
through milestone 2.7, phase 4 needs phase 3 to expose IPC measurements on
real workloads. The phases cannot be parallelised end-to-end — but the
sub-agent work within each phase can.

---

## Ideas captured from live work (append-only)

Items that popped up while executing a phase that are worth keeping but
aren't yet big enough to hoist into a milestone.

- **Sub-agent self-verification doesn't hold** (observed phase 1.5).
  Every sub-agent this session reported "Bash denied" at task completion
  and asked the user to run the verification.  Files DID land in the
  parent worktree anyway.  The orchestrator must run lint + targeted
  tests for every sub-agent before deciding to merge — never trust the
  agent's self-report.  (Historical lesson; the orchestrator pattern
  has since been retired in favour of the Task tool / TaskList.)

- **`isolation: worktree` is unreliable**.  Of four sub-agents spawned
  with isolation, three wrote directly into the parent worktree and one
  (mul_div) had its isolated worktree cleaned up with its diff lost.
  Fleet plan should always use named agent branches (`agent/<name>`) and
  have the orchestrator manage the worktrees explicitly.  Don't rely on
  the Agent tool's isolation flag for either insulation or preservation.

- **"Test triage" is a recurring workflow** worth scripting.  When a
  sub-agent writes N tests and M fail, classifying "agent test bug" vs
  "real core bug" is manual today.  Idea: a helper that assembles each
  failing test, runs it, and prints the CMP+BNE decision point and the
  CCR / Dn state at that point so a human (or an orchestrator) can
  spot-check whether the expected value in the test makes sense.
  Companion to tools/decode_check.py.

- **mul_div design spec landed in an agent's final message** without
  the implementation.  Preserved in the agent's task-output file for
  reuse: 34-cycle restoring shift-subtract, port shape mirrors `alu.v`,
  divide-by-zero → 1-cycle completion with `exc_out=1` (vector 5),
  DIVS overflow → `V=1`, raw quotient as result, no exception.  Next
  mul-div agent should be handed this spec as its starting prompt
  rather than re-designing.

- **RAS design notes from the standalone module** (`ras.v`, committed
  standalone this phase).  `spec_depth` combinational (not a counter),
  `flush_en` gates speculative push/pop but not commit-pointer advance,
  overflow wraps silently (Intel-style), push+pop same cycle = pop-
  then-push with net-zero `spec_top` movement.  Worth remembering
  during the `if_stage` integration session.

- **Commit-time BPU training pollution from RTS / JMP(An) is measurable**
  (`bsr_rts_basic` 106→109 cycles after BTB landed).  Phase-2 BPU work
  should filter training by uop op — only direct branches (BRA, BSR's
  BRA phase, Bcc, DBcc) train the BTB; RTS and JMP(An) should train
  RAS (already planned) or a dedicated Indirect Branch Target
  predictor.
