# Q700 ROM cold-boot bring-up log (task #101)

Status: **PATCHED Q700 FRONTIER REACHES LATE VECTOR/PERIPHERAL DELAY CODE** —
the ROM byte descriptor patches at `0x2f88/0x2f94` are removed, the Q700
descriptor-table entry is accepted by the ROM's own probe logic, and the fresh
cold path reaches the relocated checksum loop by 5k committed uops.  The
current checkpoint corpus is rebuilt from the current Verilated model and is
valid through 40M absolute cycles.  The current no-patch checkpoint frontier is
still useful history around the memory-sizing probe/exception path at
`0x4084bb9c`; the current patched frontier with `diag-loops,chime-delay` is the
non-exception `DBF` delay loop at `0x40800888` after the IWM/SWIM,
CMPI-absolute, vector-hook MOVE, and ADDA.W memory-source decode fixes.

This doc is the bring-up journal.  Update it when follow-ups land.

### 2026-04-21 architectural replay frontier

The current fast iteration path is architectural replay from a portable
checkpoint under `/dev/shm/m68k`, not exact Verilator checkpoint restore.  One
important harness fix from this pass: mapped RAM-like regions now default to
`0x00` while unmapped/open-bus reads still return `0xff`.  The earlier
exception-vector crash was traced to old `default=0xff` RAM state: lowmem byte
`0x1efc == 0xff` made the ROM choose a parameter block that zeroed the
`0x38000020..0x3800002f` alias, which is mapped back onto low exception
vectors.  Fresh zero-default cold runs should not need that poke; old portable
checkpoints that encode `default=0xff` either need recapture or an explicit
`0x1efc=0` artifact before replay.

The latest patched architectural replay used:

```bash
VERILATOR_THREADS=4 VERILATOR_JOBS=4 build/rom_boot/Vmac_top \
  +rom=files/420dbff3.rom +trace=/dev/null \
  +rom_patch=diag-loops,chime-delay,timer-delay,scc-delay \
  +arch_replay=/dev/shm/m68k/rom_replay_1efc0/q700.1efc0.txt \
  +max_insts=20000000 +timeout=20000000 \
  +stop_on_exc=2,4,11 +stuck_pc_threshold=8192 +no_waves
```

That run now clears the recent MOVE/SUB decode frontiers and the ROM-needed
offset-zero memory bitfield copy helper.  The current stop is later, in copied
low-RAM code:

```text
reason:     stop-exc class=f-line vec=11
committed:  7930249
fault_pc:   0x000024b8
last_pc:    0x000024b4
next_fetch: 0x000024e0
```

The implemented memory bitfield support is intentionally narrow: it covers
`BFEXTU (An){0:Dw},Dn`, dynamic `BFINS Dn,(An){0:Dw}` for runtime widths 24
and 32, and static `BFINS Dn,(An){0:32}`.  It is enough for the Q700 ROM copy
helper at `0x4080ca2c..0x4080ca3a`, not a full memory bitfield RMW
implementation.  See `loose_ends.md` before widening this area.

The replay still shows SCSI polling activity and no DAFB register or VRAM
aperture traffic at this frontier.  Next local investigation should disassemble
the low-RAM code around `0x24ae..0x24e0` and determine whether the F-line is
caused by copied-code corruption, unsupported decode, or a stale replay state.

---

## 1. Harness overview

- **Source**: `tb/tb_rom_boot.cpp` (standalone, does not touch `tb/tb_top.cpp`).
- **Build**: `make tb-rom-boot`.  Full Verilator rebuild of `mac_top.v`
  with the ROM-boot main-cpp in place of `tb_top.cpp`; lands in
  `$(BUILD_DIR)/rom_boot/Vmac_top` so it doesn't stomp the main
  `$(BUILD_DIR)/sim/Vmac_top` artifact.
- **ROM**: `files/420dbff3.rom` — default.  Quadra 700 Universal ROM,
  1 MB, stored-checksum `0x420dbff3`, reset-vector entry `0x0000002a`,
  SHA1 `7a8ee468d16e64f2ad10cb8d1a45e6f07cc9e212`.  Override with
  `make tb-rom-boot ROM=<path>` — e.g. to point at a keeper LC 630
  dump for chipset-divergence study.  The harness parses the
  first 8 header bytes on load and prints a warning if the
  checksum or entry offset don't match Q700; peripheral stubs are
  Q700-specific (discrete VIA1+VIA2, NCR 5380, Z80 SCC, ASC,
  DAFB) so a non-Q700 ROM will stall in early probe code.
- **Trace out**: `build/sim/rom_boot_trace.log` — one committed uop per
  line, `<pc> <ir> <ccr>`.  ROM-runner trace, last-N, and probe outputs now
  default to a low-risk scratch root: the Makefile prefers
  `/dev/shm/m68k-ooo` when it has at least 512 MB free and falls back to
  `build/sim` otherwise.  Override with `ROMBOOT_OUTPUT_ROOT=<path>` if you
  want a different scratch location.
- **Last-N cycle trace**: disabled by default.  Add
  `ROMBOOT_EXTRA="+lastn_trace=64"` to dump the final 64 per-cycle samples
  to stderr at termination, or add
  `+lastn_trace_path=/dev/shm/m68k-ooo/rom_boot_lastn_trace.log` (or the
  fallback `build/sim/...` path) to write them to a file.  Each sample
  captures the current committed count, last/current PC,
  ROB-head PC/vector/complete state, commit exception gates, exception
  sequencer state, flush queue, d-cache flush walker, DAXI handshakes, and
  IF request state.  This is intended for fast exception/flush frontier
  debugging without enabling waves or growing the committed-uop trace.
  For the current late-frontier stop workflow, use:
  ```
  make rom-boot-stop-summary
  ```
  The helper runs `tb-rom-boot` with `+stop_on_rom_faults`, the late-frontier
  `+rom_patch=diag-loops,chime-delay` pair, a 4096-sample last-N ring, and
  the usual commit trace, then writes a compact report to
  `$(ROMBOOT_OUTPUT_ROOT)/rom_boot_stop_summary_report.log` and prints it on
  stdout.  For CI/handoff validation, use:
  ```
  make rom-boot-stop-summary-smoke
  make rom-boot-stop-exc-smoke
  ```
  To swap in a custom exception list or patch set, override
  `ROMBOOT_STOP_SUMMARY_STOP` or `ROMBOOT_STOP_SUMMARY_EXTRA`, for example:
  ```
  make rom-boot-stop-summary ROMBOOT_STOP_SUMMARY_STOP='+stop_on_exc=4,11'
  make rom-boot-stop-summary ROMBOOT_STOP_SUMMARY_EXTRA='+rom_patch=diag-loops'
  ```
  The compact report is produced by `tools/rom_boot_stop_summary.py --compact`
  so the same helper is readable in logs and easy to grep in CI.
- **Fast fault stops and bounded windows**: selected exception-vector stops
  are opt-in with `+stop_on_exc=<list>`, `+stop_on_illegal`, or
  `+stop_on_rom_faults` (`2,4,11`).  Add `+stop_on_ifetch_berr` when you want
  the harness to stop as soon as the instruction fetch port asks for an
  unmapped 16-byte line instead of letting open-bus bytes decode later.
  `+stop_on_exc` now triggers on the precise exception-entry boundary after
  the handler PC is known, rather than on the transient live exception state.
  Add `+fault_dump_dir=$(ROMBOOT_OUTPUT_ROOT)/fault` to write small code/data
  windows around the stop point; `+fault_dump_bytes=<n>` defaults to 128 and
  is capped at 4096.  The dump is off by default and writes a manifest plus
  hex files under the requested directory, which is normally under
  `/dev/shm/m68k-ooo` through `ROMBOOT_OUTPUT_ROOT`.
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+stop_on_illegal +stop_on_ifetch_berr +fault_dump_dir=/dev/shm/m68k-ooo/fault +no_waves"
  make rom-boot-fault-knob-smoke
  ```
- **ROM outcome stops**: `+stop_on_rom_outcome` arms both terminal ROM
  outcome hooks: the Sad Mac diagnostic entry at `0x40849afa` and the
  no-rootfs disk-prompt polling loop at `0x40898e3e`.  Use
  `+stop_on_sad_mac` or `+stop_on_disk_prompt` independently when you only
  want one side.  `+disk_prompt_hit=<n>` defaults to 32 so a transient sample
  does not look like the spinning-floppy milestone; lowering it is useful for
  focused replay tests.  The Sad Mac stop includes the diagnostic code from
  `D6`, and the disk-prompt stop includes the hit count and live argument
  registers.  For the current no-rootfs bring-up, the disk-prompt loop is the
  "happy enough" milestone.
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+stop_on_rom_outcome +disk_prompt_hit=32 +no_waves"
  make rom-boot-outcome-smoke
  ```
- **Terminal watchdog thresholds**: the harness still defaults to the
  historical watchdogs, but the thresholds are now explicit and opt-out:
  `+stuck_pc_threshold=8192` stops after 8192 repeated commits at the same
  PC, and `+no_progress_cycles=500000` stops after 500000 cycles with no
  internal progress-signature movement.  Set either value to `0` to disable
  that watchdog for a debug run.  The final state prints the active values and
  the current same-PC repeat counter, which makes frontier logs cheaper to
  classify without reading the C++ harness.
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+stuck_pc_threshold=128 +no_progress_cycles=20000 +lastn_trace=128 +no_waves"
  make rom-boot-watchdog-smoke
  ```
  Stop summaries name the common ROM-fault classes, e.g.
  `stop-exc class=bus-error source=... vec=2` or
  `stop-exc class=illegal-instruction source=... vec=4`, so these known bad
  terminals are grep-friendly without changing the default ROM run.
- **Trace enders**: use the `+end_on_*` variants when a trap/fetch fault/hung
  loop is the expected terminal point for a frontier script rather than a
  debug stop.  `+end_on_exc=<list>`, `+end_on_illegal`, and
  `+end_on_rom_faults` mirror the stop-on-exception controls but report
  `end-exc ...` in the final state and last-N header.  `+end_on_ifetch_berr`
  reports `end-ifetch-berr ...` for unmapped instruction fetches.  The stuck
  PC and no-progress watchdogs remain on by default, but their budgets are now
  configurable with `+stuck_pc_threshold=<n>` and `+no_progress_cycles=<n>`;
  `0` disables the corresponding watchdog.  Add `+end_on_stuck_pc` or
  `+end_on_no_progress` to classify those as intentional enders.
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+end_on_rom_faults +end_on_ifetch_berr +end_on_stuck_pc +stuck_pc_threshold=16 +lastn_trace=128 +no_waves"
  make rom-boot-trace-enders-smoke
  ```
- **Data watchpoints**: disabled by default.  Add `+data_watch=<spec>` to
  log matching data-side reads/writes without turning on a broad trace.
  The spec is comma-separated and accepts exact byte addresses, inclusive
  ranges, or base+length forms: `0x386,0x372,0x3ee`,
  `0x408000f0-0x40800110`, `0x408000f0..0x40800110`, or
  `0x408000f0+0x20`.  A watched byte matches the enclosing 32-bit DAXI beat,
  so `+data_watch=0x386` reports a read at aligned address `0x384`.
  Use `+data_watch_log=/dev/shm/m68k-ooo/rom_boot_data_watch.log` and
  `+data_watch_limit=<n>` to keep output bounded; the limit defaults to
  1024 lines, and `0` keeps only the end summary.  Each detail line includes
  cycle, committed count, best-available core PCs, operation, address, size,
  strobe, data, AXI response, bus owner, and harness source class.  For the
  current low-pointer frontier:
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+data_watch=0x386,0x372,0x3ee,0x408000f0+0x40 +data_watch_log=/dev/shm/m68k-ooo/rom_boot_data_watch.log +data_watch_limit=4096 +stop_on_rom_faults +no_waves"
  ```
  To validate the facility without a deep ROM run:
  ```
  make rom-boot-data-watch-smoke
  ```
- **Cache frontier event log**: opt in with `+cache_event_log=<path>` and
  `+cache_event_log_limit=<n>` when you need to tell whether the ROM is
  issuing cache-maintenance requests, the D-cache is snooping writes, or the
  low-memory table is being touched only through the cache.  The log records
  cycle, committed count, current/last PC, cache-maint request fields
  (`req`, `inv`, `scope`, `caches`, `addr`), the visible busy/done state, and
  D-cache snoop / low-memory write events.  Every run also ends with a
  fallback snapshot of `0x00000400..0x000005ff` so you can compare the
  dispatch-table area even when the write never reaches the external DAXI
  path.
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+cache_event_log=/dev/shm/m68k-ooo/rom_boot_cache_event.log +cache_event_log_limit=256 +data_watch=0x00000400+0x200 +data_watch_log=/dev/shm/m68k-ooo/rom_boot_data_watch.log +data_watch_limit=4096 +stop_on_exc=11 +rom_patch=diag-loops,chime-delay,timer-delay +no_waves"
  ```
  For a cheap harness sanity check, use:
  ```
  make rom-boot-cache-event-smoke
  ```
  If the internal cache-maint signals are not enough to settle the question,
  use the end-of-run low-memory snapshot together with a committed-PC stop
  such as `+end_pc=0x40809a04` or the existing `+stop_on_exc=11` workflow.
- **Display watch summary**: every ROM harness run prints a compact
  `rom-boot display activity` block at exit.  It separately summarizes DAFB
  register traffic over `0xF9800000..0xF98003FF` and pixel/VRAM aperture
  traffic over `0xF9000000..0xF91FFFFF`, with read/write counts plus first
  and last PC/address/value/cycle when present.  This is intentionally
  separate from generic peripheral totals so early framebuffer writes cannot
  obscure display register setup, and zero-count display activity is still
  visible in frontier logs.  For detailed event logs, use the peripheral event
  logger with only the display categories:
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+periph_event_log=/dev/shm/m68k-ooo/rom_boot_display_events.log +periph_event_filter=DAFB,VRAM +periph_event_log_limit=4096 +no_waves"
  ```
  To validate the display watch without a deep ROM run:
  ```
  make rom-boot-display-watch-smoke
  ```
- **End breakpoints**: use `+end_pc=<pc>[,<pc>...]` for intentional
  terminal stops on a macro-instruction retire boundary.  `+end_pc` and
  `+stop_pc` now trigger from the core's boundary event stream, not raw
  retired-uop `dbg_last_pc`, so cracked instructions stop only after their
  final uop retires.  The final reason remains `end-breakpoint pc=...`,
  which keeps scripted frontier runs easy to distinguish from ad-hoc debug
  stops.  `+end_pc_hit=<n>` waits for the Nth hit, defaulting to 1.  Prefer
  this for known-good ROM milestones:
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+end_pc=0x4084bb9c +lastn_trace=128 +no_waves"
  make rom-boot-end-breakpoint-smoke
  ```
  Keep checkpoint capture for fault-region binary searches or resume-only
  debug; normal milestone runs should use an end breakpoint plus the cheap
  trace/probe logs.
- **Harness-local ROM patches**: default runs remain byte-for-byte ROM
  execution.  For late-frontier iteration, the harness accepts opt-in named
  patches after loading the ROM into its private buffer and reapplies them
  after `+restore_state=` because exact checkpoints serialize the ROM/RAM
  memory image.  `+rom_patch=clean-fastdiag` is the contract-preserving fast
  diagnostic set; `diag-loops` and `fastdiag` are aliases for it.  It leaves
  the checksum and RAM diagnostic helper entries/returns intact, patches only
  internal loop-back/status instructions, and preserves success-visible `D6`
  behavior by accepting bounded samples instead of returning around helper
  setup.  `+rom_patch=checksum-fast` and `+rom_patch=meminit-fast` can be used
  separately for narrower runs.  `+rom_patch=rtc-pram-mame-state` is a 4 MB
  Q700 MAME-derived shortcut for the RTC/PRAM sizing loop at `0x4084721c`: it
  restores the register/SR state MAME reaches at `0x40847280` and branches to
  that RAM-fill helper.  `+rom_patch=ramtest-mame-state` is a narrower
  MAME-derived alternative for the forward RAM helper at `0x40847280`: it
  returns with the MAME-observed register state and host-fills the helper's RAM
  range with the matching diagnostic pattern.  `+rom_patch=alias-probe-mame-state`
  skips the later RAM lane/alias probe at `0x4084bb74` by restoring the
  MAME-observed low-memory side effect, installing the post-probe 4 MB
  register/SR state, and continuing at `0x4084bc38`.  `+rom_patch=ram-list-sentinel-fast`
  replaces the RAM descriptor-list sentinel compare at `0x40846ed2` with an
  equivalent `A0+1` zero test while leaving the ROM's existing `BEQ.S` in
  place.  `+rom_patch=via-timer-mame-state` skips the VIA timer interrupt
  diagnostic waits at `0x40847bf6` and `0x40847caa` by restoring the
  MAME-observed validation counters, preserving the ROM's follow-on checks.
  This is a harness shortcut for the current missing VIA-IRQ-to-`cpu_ipl_ext`
  path, not a substitute for wiring the real interrupt path.  `+rom_patch=mame-fastdiag`
  combines these MAME-derived shortcuts with `checksum-fast` and `meminit-fast`
  for frontier iteration.  `+rom_patch=adb-init-wait` is a MAME-scout-only
  skip for the ADB manager init busy wait at `0x4080a8e6`
  (`btst #5,0x015d(a3); bne.s 0x4080a8e6`).  `+rom_patch=mame-firstlight`
  bundles `mame-fastdiag`, `adb-init-wait`, `chime-delay`, `timer-delay`, and
  `scc-delay` for bounded first-light scouting, but it is not a hardware-valid
  success criterion.  The MAME disk-prompt oracle uses the stock ROM image and
  applies only the ADB init wait patch at runtime after the ROM checksum path
  has passed; pre-patching that byte range changes the checksum behavior and is
  not the oracle.
  The old whole-helper `JMP (A6)`
  behavior is
  still available only as `+rom_patch=checksum-unsafe` or
  `+rom_patch=diag-loops-unsafe`; do not use those for hardware-valid ROM
  frontiers because they skip helper side effects and have caused bad
  heap/low-vector state, including the `0x38000000` alias wipe path.
  `+rom_patch=chime-delay` NOPs only the ASC chime inner `DBF` delay at
  `0x40807118` while leaving the helper's register setup and sample writes
  intact, and `+rom_patch=timer-delay` NOPs the late VIA timer `DBF` loop at
  `0x40800888`.  This does not mutate `files/420dbff3.rom`; every patched byte
  is logged with old/new values.  Use these only to skip known-deep diagnostics
  while chasing later decode/peripheral faults, and rerun no-patch before
  treating a frontier as hardware-valid.  If resuming from a checkpoint, prefer
  one captured with the same patch set already active; exact checkpoints can
  contain stale front-end/I-cache state from code fetched before a late patch
  was requested.
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+rom_patch=clean-fastdiag +end_pc=0x40846e3c +lastn_trace=128 +no_waves"
  make tb-rom-boot ROMBOOT_EXTRA="+rom_patch=clean-fastdiag,chime-delay +end_pc=0x40846ef2 +lastn_trace=128 +no_waves"
  make tb-rom-boot ROMBOOT_EXTRA="+rom_patch=diag-loops,chime-delay,timer-delay +end_pc=0x408005b0 +lastn_trace=1024 +no_waves"
  make rom-boot-fastdiag-smoke
  make rom-boot-chime-delay-smoke
  make rom-boot-timer-delay-smoke
  ```
- **Latest timer-delay frontier**: after adding full-format memory-indirect
  source to absolute-destination MOVE.{B,W,L} and static
  BTST/BCHG/BCLR/BSET through the same full-format source EA, the
  `diag-loops,chime-delay,timer-delay` smoke now uses `+end_pc=0x408005b0`.
  That proves the exact ROM `21f0 81e2 0ddc ffe8 1ef0` MOVE commits and the
  adjacent `0830 0000 81e2 0ddc ffe7` BTST is reached without decode trapping.
  Display evidence remains one DAFB register write and no VRAM aperture
  traffic.
- **Current deep timer-delay stop**: after fixing the D-cache dirty-evict
  refill beat, adding full-format indexed `LEA` mode-6 decode, widening
  full-format memory-indirect `JSR` for index-suppressed forms, adding
  full-format no-indirect indexed `MOVE.{B,W,L}` source-to-`(d16,An)`, and
  cracking `RTD #disp16`, the patched run crosses the old `0x408099e0` and
  `0x408099ec` illegal-instruction stops.  The low A-line slot at
  `0x0000051c` is populated with `0x40809a6e`; the ROM executes
  `40809a5e: 43f0 05a0 0400` as `lea @(0x400,D0.W*4),A1`, executes
  `40885000: 4eb0 81e1 06f4` as `jsr @(0x6f4)@(0)`, and reads
  `0x000006f4 -> 0x40885030`.  The current stop is now:
  `reason=stop-exc class=f-line vec=11 pc=0xffffffff
  dbg_last_pc=0x408099ec` at `cycles=22521968`, `committed=7548976`.
  The last decoded ROM instruction is `408099ec: 4e74 0004` (`rtd #4`);
  it branches through the stack return slot that the preceding
  `408099e0: 2f70 25a0 0e00 0008` full-format indexed `MOVE.L` updates.
  The immediate next question is why that dispatch return slot/table value is
  `0xffffffff` rather than a valid ROM/RAM target.  Display evidence remains
  one DAFB register write and no VRAM aperture traffic.
- **Current VIA-timer-state frontier**: `+rom_patch=via-timer-mame-state`
  recreates MAME's validation counters for the later VIA timer diagnostic at
  `0x40847bf6` / `0x40847caa`, covering the current harness gap where the VIA
  stub logs T1/IFR activity but does not drive `cpu_ipl_ext`.  With
  `diag-loops,chime-delay,timer-delay,via-timer-mame-state`, the ROM reaches
  the first DAFB register write at `0x40899166` and the SCSI polling path, then
  fails on the first Toolbox A-line trap:
  `fault_pc=0x40800502` (`_SetApplLimit`) takes vector 10, reads the low vector
  slot at `0x00000028`, gets handler `0x00000000`, and then faults executing
  low memory.  This is a vector-table/exception-path frontier, not another
  decode crack.  The captured last-N trace is
  `/dev/shm/m68k/rom_lowpc_after_timerdelay.lastn`.
- **Current frontier checkpoint**: for repeated work from the
  `0x408005b0` timer-delay frontier, use the narrow checkpoint target instead
  of regenerating the whole run by hand:
  ```
  make rom-boot-frontier-checkpoint
  make rom-boot-frontier-restore-smoke
  ```
  The default file is
  `build/rom_boot_checkpoints/frontier/q700.timer_delay_408005b0.vlt`.
  It is an exact Verilator model snapshot, not a durable architectural save
  format, so rebuild it after RTL/harness changes that alter the generated
  model layout or when changing the patch set.  Do not use this as the main
  decode-iteration shortcut when RTL is changing.
- **Portable architectural checkpoints**: `+arch_checkpoint=<path>` writes the
  durable, text, Verilator-independent ROM-boot checkpoint format.  The file
  starts with `format m68k-ooo-arch-checkpoint-v1` and includes:
  D0-D7, A0-A7 from the committed RAT/PRF, CCR and SR, the last observed
  commit-side architectural next PC, VBR/CACR/SFC/DFC, USP/SSP/ISP, MMU CRs
  (TC, URP, SRP, ITT0/1, DTT0/1), the patched Q700 ROM image, and sparse
  RAM/VRAM/magic memory segments.  Before writing memory, the harness asserts
  the existing debug D-cache flush hook and records whether it completed plus
  the committed-count delta while the flush clocked.  It also writes a
  `quiesce ...` line with ROB/IQ/LSU/cache-flush occupancy so a future replay
  loader can reject captures that are not at a clean architectural boundary.
  ```
  make rom-boot-arch-checkpoint
  make rom-boot-arch-checkpoint-smoke
  make rom-boot-arch-replay-smoke
  make rom-boot-arch-roundtrip-smoke
  make rom-boot-arch-endpc-replay-smoke
  make rom-boot-arch-aline-frontier-replay
  ```
  `make rom-boot-arch-checkpoint-summary` prints a compact summary of an
  existing checkpoint and checks the replay-critical fields directly, which
  keeps the workflow usable even when Verilator save compatibility is not.
  Default output goes under `$(ROMBOOT_OUTPUT_ROOT)`, usually
  `/dev/shm/m68k-ooo`, with trace output set to `/dev/null` for the Make
  targets.  `+arch_replay=<path>` cold-resets the Verilated model, loads the
  ROM/RAM/VRAM/magic memory segments from the text checkpoint, injects the
  committed D/A registers, CCR/SR, control registers, MMU CRs, and redirects
  fetch to the saved `pc next=...`.

  For a one-off capture at a frontier, pass normal ROM-runner stop controls
  through `ROMBOOT_ARCH_CHECKPOINT_EXTRA`.  For example:
  ```
  make rom-boot-arch-checkpoint \
      ROMBOOT_ARCH_CHECKPOINT=/dev/shm/m68k-ooo/q700.pre_frontier.txt \
      ROMBOOT_ARCH_CHECKPOINT_MAX=7600000 \
      ROMBOOT_ARCH_CHECKPOINT_TIMEOUT=26000000 \
      ROMBOOT_ARCH_CHECKPOINT_EXTRA='+rom_patch=diag-loops,chime-delay,timer-delay +end_pc=0x40809a96'
  ```
  To resume from a saved portable checkpoint without using a smoke target:
  ```
  make rom-boot-arch-resume \
      ROMBOOT_ARCH_RESUME=/dev/shm/m68k-ooo/q700.pre_frontier.txt \
      ROMBOOT_ARCH_RESUME_MAX=7524750
  ```
  `+max_insts` and `+timeout` are absolute harness counters, not deltas from
  the replay checkpoint.  When chaining replay segments, read the previous
  checkpoint log's `committed=` / `cycles=` values and set the next
  `ROMBOOT_ARCH_CHECKPOINT_MAX` / `ROMBOOT_ARCH_CHECKPOINT_TIMEOUT` above
  those values by the desired increment.  Setting `ROMBOOT_ARCH_CHECKPOINT_MAX`
  to `5000000` while replaying a checkpoint already past 100M commits will
  stop immediately and write a nearly identical checkpoint.

  `make rom-boot-arch-roundtrip-smoke` is the bounded regression for the
  end-to-end flow.  It captures a short seed checkpoint, runs the ROM
  continuously to a later committed limit, replays from the seed to that same
  limit with IO/peripherals reset, writes a final checkpoint from each path,
  and compares CPU architectural state plus ROM/RAM/VRAM/magic segment
  checksums with `tools/rom_boot_arch_compare.py`.

  Architectural replay deliberately reinitializes most IO/peripheral model
  state: VIA/ADB/RTC/SCSI/SCC/ASC/SWIM/DAFB model internals, outstanding
  fetch/data handshakes, caches, queues, and predictors restart
  empty/default.  The summary checker and replay loader both require the
  checkpoint to declare a supported replay contract and both require the Q700
  ROM, RAM, VRAM, and sim-magic memory segments.  The v2 contract carries
  `io_state=via1-v1 io_restore=partial-via1-v1` to make the limitation
  explicit: only the recorded VIA1/RTC-line/Q700-descriptor subset is
  restored, while the rest of IO/peripheral state is reset.  The loader
  rejects unsafe seeds before asserting the debug architectural load path:
  missing flush/quiesce/replay declarations, an incomplete memory section, a
  non-reset replay contract, non-empty ROB/IQs, active
  LSU/exception/cache-flush state, a non-empty flush queue, or
  `pc next=0x00000000/0xffffffff` all abort replay.  The remaining limitations
  are explicit in each checkpoint as `gap ...` lines: the debug D-cache flush
  is not a core halt, so the machine may retire additional uops during flush;
  peripheral model state is intentionally only partially restored; and the
  debug architectural load path is a simulation hook, not an FPGA resume
  mechanism.  Resume must use the saved commit-side `pc next=...`, not
  `dbg_pc`, because the front-end can be far ahead of the last retired
  instruction.

  Restored state is limited to the sparse ROM/RAM/VRAM/magic memory image plus
  D0-D7, A0-A7, CCR/SR, VBR/CACR/SFC/DFC, USP/SSP/ISP, MMU TC/URP/SRP/ITT/DTT
  registers, committed count metadata, the replay PC, and the small recorded
  VIA1/RTC-line/Q700-descriptor subset.  A replay seed is useful for CPU/RAM
  frontier iteration across RTL rebuilds but is not expected to reproduce
  IO-sensitive timing or device histories bit-for-bit.

  For pre-fault iteration, prefer capturing at a known committed end PC rather
  than letting the run fall into the final exception and checkpointing that
  state.  The smoke target above uses the existing `+end_pc` stop and then
  replays the resulting portable checkpoint with IO reset/default.  For a deep
  frontier, override the PC and budgets, for example:
  ```
  make rom-boot-arch-endpc-replay-smoke \
      ROMBOOT_ARCH_ENDPC=0x40809a00 \
      ROMBOOT_ARCH_ENDPC_CAPTURE_MAX=7600000 \
      ROMBOOT_ARCH_ENDPC_CAPTURE_TIMEOUT=26000000
  ```
  If the summary reports `pc next=0xffffffff`, `quiesce rob_empty=0`, or
  `dcache_flush_busy=1`, the checkpoint was captured too late or while the
  debug flush advanced into the fault path.  Move the end PC earlier and
  recapture; do not treat that file as a clean architectural replay seed.
  For the current A-line table-builder frontier, use the preset target:
  ```
  make rom-boot-arch-aline-frontier-replay
  ```
  It captures at `ROMBOOT_ARCH_ALINE_FRONTIER_PC` (default `0x40809a96`, the
  first committed pre-dispatch/table-builder entry seen in `atrace_deep`;
  overrideable with `ROMBOOT_ARCH_ALINE_FRONTIER_PC_HIT=<n>`), validates the
  portable checkpoint, then immediately replays it with IO reset/default.  The
  target also records low table, builder-region, and ROM-source data watches
  (`0x400..0x7ff`, `0x51c`, `0x13c0..0x14e0`,
  `0x408ca0e0..0x408ca3ff`) plus a cache-event log, so a failed validation
  still leaves enough context to choose an earlier PC/hit.  If this target
  fails validation after capture, the current debug-flush-based checkpoint path
  cannot safely land at that frontier; recapture at an earlier committed PC
  instead of replaying a post-builder or post-dispatch file.

  On the 2026-04-20 frontier, the preset captured a replayable CPU+RAM
  artifact at `/dev/shm/m68k-ooo/q700.arch_aline_frontier.txt`:
  `committed=7517611`, `pc_next=0x40809aa4`, `flush_delta=3`,
  `rom_src_408ca0e0=0x80ff0004`, and
  `rom_src_408ca3f0=0x050f918c`.  Replaying it with IO reset/default to the
  absolute committed limit `7524750` did not reproduce the cold-run F-line
  dispatch; it reached `max-insts-reached` at `last_pc=0x408026a2`.  That means
  architectural replay is good enough to avoid the 7.5M committed cold run for
  CPU/RAM iteration near the table-builder entry, but not a deterministic
  reproduction of IO-sensitive ROM progress after DAFB/SCSI/VIA state has been
  reset.
- **Exact checkpoints**: `tb-rom-boot` is built with Verilator
  `--savable` and can save/restore the full generated model plus the C++
  harness state outside RTL: memory model, VIA/ADB shadows, outstanding
  fetch/AXI transactions, overlay, counters, and watchdog state.  These
  snapshots are large (~69 MB today) because they include the external
  68 MB memory model, but they let long ROM runs resume from a known-good
  point without replaying from reset. The harness state includes the
  VIA/ADB shadows and the deterministic RTC/PRAM side-channel state.
  For focused event-log coverage, the harness also has directed smoke
  targets for VIA1 timer traffic, RTC/PRAM, VIA1 SR/ADB shadowing, and
  the TurboSCSI window:
  ```
  make tb-rom-boot-via1-t1-smoke
  make tb-rom-boot-rtc-smoke
  make tb-rom-boot-adb-smoke
  make tb-rom-boot-scsi-smoke
  ```
  The ADB smoke now covers the MAME 6522 SR read-clear behavior as well as
  empty-bus readback: MAME `mame0287` `src/devices/machine/6522via.cpp`
  clears `INT_SR` on SR read at lines 764-778 and on SR write at lines
  961-974.  The harness logs `ADB.sr_irq_clear` when an SR read consumes a
  pending shift-complete flag.
  `+max_insts=` remains an absolute
  committed-uop cap after restore; use a tiny cap such as
  `ROMBOOT_RESUME_MAX=1` when the goal is only to prove a snapshot can
  retire one more uop. `+timeout=` is a per-run cycle budget and starts
  at the restore point.
  ```
  make tb-rom-boot ROMBOOT_EXTRA="+max_insts=200000 +save_state=/tmp/q700.vlt"
  make tb-rom-boot ROMBOOT_EXTRA="+restore_state=/tmp/q700.vlt +max_insts=400000"
  make tb-rom-boot ROMBOOT_EXTRA="+checkpoint_prefix=/tmp/q700 +checkpoint_every=250000 +max_insts=2000000"
  make rom-boot-snapshots
  make rom-boot-deep-snapshots
  make tb-rom-boot-resume SNAPSHOT=build/rom_boot_checkpoints/q700.00050000.vlt
  make rom-boot-frontier-restore-smoke
  make rom-boot-checkpoint-inventory
  make rom-boot-snapshot-smoke
  make rom-boot-snapshot-corrupt-smoke
  ```
- **Deep cycle checkpoints**: `make rom-boot-deep-snapshots` restores
  `build/rom_boot_checkpoints/q700.final.vlt` and writes
  `q700.cyc04000000.vlt`, `q700.cyc08000000.vlt`,
  `q700.cyc16000000.vlt`, `q700.cyc32000000.vlt`, and
  `q700.cyc40000000.vlt`.  These are absolute sim-cycle points, not
  offsets from the restore point.  `ROMBOOT_DEEP_STOP_CYCLE` defaults to
  `40000000`, so a successful run exits with `cycle-budget-exhausted`.
- **Checkpoint inventory guardrail**: `make rom-boot-checkpoint-inventory`
  classifies the corpus by filename and fails if the expected committed-uop
  points, expected absolute-cycle points, `q700.final.vlt`, or naming scheme
  are stale or mixed.  Committed-uop snapshots use `q700.NNNNNNNN.vlt`;
  deep absolute-cycle snapshots use `q700.cycNNNNNNNN.vlt`.  The smoke target
  runs this inventory first, then restores the validated list in sorted order.
  `make rom-boot-snapshot-corrupt-smoke` is the negative guard: it writes a
  temporary bad `.vlt` under `build/sim` and checks that restore rejects it
  without contaminating the checkpoint corpus.
- **Overlay**: implemented in the TB, not in RTL.  `overlay=1` at reset;
  instruction fetches in the first 256 KB alias to the ROM image.  The legacy
  harness keeps data-side accesses in that low window pointed at RAM so the
  ROM can install its early `VBR=0` exception vectors before clearing
  overlay.  `+strict_overlay` instead matches the RTL glue path more closely:
  it aliases data-side low-memory accesses to ROM too and disables the old
  high-ROM-fetch auto-clear shortcut, so overlay stays asserted until VIA1
  ORB[3] is written to 0 (observed as a write through the Q700 VIA1 slave at
  `0x50F00000 + (0<<9) = 0x50F00000`, byte-lane decode).
- **Reset PC**: `tb-rom-boot` elaborates `mac_top` with
  `-GRESET_PC=32'h4000_002A`, so the CPU fetches the real Q700
  reset-vector entry directly from the native ROM window.  The old
  synthetic bootstrap trampoline at `0x40800000` is retired.

### Peripheral stubs (minimum-to-not-hang)

The TB intercepts AXI accesses in the MAME Q700 I/O mirror window
`0x50000000..0x50FFFFFF`.  Device decode canonicalizes addresses with
MAME's `0x00FC0000` mirror mask, so `0x50F0C000` and `0x5000C000`
hit the same SCC stub.  `0x5100xxxx` is outside this mirror and remains
unmapped/open-bus in the harness.
All stubs live in `tb_rom_boot.cpp` — do **not** confuse these with
the real-RTL peripherals in `rtl/mac/` (those are exercised by
tb-via1/tb-via2/tb-scsi; this harness never instantiates them because
`mac_top.v` still terminates its AXI master directly at the testbench
in the sim flow).

| Region | Canonical base | Common Q700 alias | Behaviour |
|---|---|---|---|
| VIA1 | `0x50000000` | `0x50F00000` | Reg file, 512-byte stride, ORB[3]=1 at reset, tracks IFR/IER write-clear/set |
| VIA2 | `0x50002000` | `0x50F02000` | Zero-fill, ORB reports no slot IRQs, IFR=0 |
| Ethernet ID | `0x50008000` | `0x50F08000` | Q700 MAC PROM bytes/checksum |
| SONIC | `0x5000A000` | `0x50F0A000` | DP83932 reset/config register block; descriptor DMA still pending |
| SCC | `0x5000C000` | `0x50F0C000` | Register-file reads return 0, writes swallowed |
| Orwell controls | `0x5000E000` | `0x50F0E000` | Reset-value stub |
| DAFB TurboSCSI | `0x5000F000..0x5000F0FF`, `0x5000F100..0x5000F101` | `0x50F0F000`, `0x50F0F100` | NCR53C96 reads now expose a deterministic phase/register stub with raw-block intent; DMA shim writes are still swallowed and logged |
| ASC/EASC | `0x50014000` | `0x50F14000` | Reads return 0 |
| SWIM/IWM | `0x5001E000` | `0x50F1E000` | Probe-safe no-media IWM-mode stub; phase/control latch state is visible, status/handshake respond deterministically |

Most writes are swallowed.  The intentional side effects are VIA1 overlay
control and the small amount of IWM/SWIM latch state needed by the ROM probe.
`0x50F04000` is deliberately **not** mapped as SCC for Q700: canonicalizing it
gives `0x50004000`, which MAME's Q700 map leaves empty.  If the ROM polls that
address, the harness selected the wrong ROM hardware descriptor.

### Termination conditions

- **sentinel-PASS/FAIL**: write 0xC0FFEE00 / other to 0xFFFF0000 — same
  magic as `tb_top.cpp`.  ROM never uses this address, but kept for parity.
- **stuck-pc**: same PC committed `+stuck_pc_threshold` times in a row
  (default 8192, `0` disables).
- **max-insts**: `+max_insts=N` (default 5000).
- **cycle-budget**: `+timeout=N` cycles (default 5 M).
- **no-progress**: `+no_progress_cycles` cycles without a new commit or
  observable internal movement (default 500 K, `0` disables).  D-cache flush
  walks and exception-entry progress now keep the watchdog alive instead of
  being misreported as a front-end stall.
- **end-class terminal reasons**: `+end_on_exc`, `+end_on_ifetch_berr`,
  `+end_on_stuck_pc`, and `+end_on_no_progress` keep the same stop mechanics
  but use `end-*` reason strings for expected frontier termini.

The harness always exits 0 on clean termination — this is a
diagnostic tool, not a pass/fail gate.  The `tb-rom-boot` target
therefore always "passes" unless the binary itself aborts.

`make tb-rom-boot-periph-events` is the bounded smoke for peripheral
observability.  It checks both category counts and the event-breakdown lines
for the early Q700 ROM request classes that must be visible before deeper
bring-up: VIA1/VIA2 reads and writes, ADB PCR/ACR setup, RTC select/clock/
deselect activity, and VBL IER transitions.  SCSI, SCC, ASC, DAFB, and VRAM
are still required as watched categories even when the current 70k-instruction
frontier has not reached live traffic there yet.

---

## 2. Current run summary (checkpoint corpus)

`make rom-boot-snapshots` still captures the committed-uop corpus around
the checksum loop:

    $ make rom-boot-snapshots
    ...
    [rom-boot] checkpoint saved: build/rom_boot_checkpoints/q700.01600000.vlt
    reason:     max-insts-reached
    cycles:     3609686
    committed:  1800000
    last_pc:    0x40847516
    next_fetch: 0x40847516
    overlay:    1 (still asserted on the current no-patch path)
    d3 = 0x0006dccc

The live path is the ROM checksum loop:

    40847512  5983    subql #4,%d3
    40847514  2818    movel %a0@+,%d4
    40847516  3018    movew %a0@+,%d0
    40847518  d280    addl %d0,%d1
    4084751a  5583    subql #2,%d3
    4084751c  66f8    bnes 0x40847516

`make rom-boot-deep-snapshots` then resumes from `q700.final.vlt` and captures
the deeper absolute-cycle corpus:

| Snapshot | Committed uops | PC at save |
|---|---:|---|
| `q700.cyc04000000.vlt` | 1,995,157 | `0x40847516` |
| `q700.cyc08000000.vlt` | 3,631,355 | `0x40807118` |
| `q700.cyc16000000.vlt` | 4,989,138 | `0x4084bb9c` |
| `q700.cyc32000000.vlt` | 4,989,138 | `0x4084bb9c` |
| `q700.cyc40000000.vlt` | 4,989,138 | `0x4084bb9c` |

Current deep-stop state:

    reason:     cycle-budget-exhausted
    cycles:     40000000
    committed:  4989138
    last_pc:    0x4084bb9c
    next_fetch: 0x00000011
    ccr:        0x00 (X=0 N=0 Z=0 V=0 C=0)
    overlay:    1
    a0:         0x04000000
    a1:         0x08000000
    a7:         0x406f1c58
    q700_descriptor_selected: 0 after restore
        (marker field is not serialized; the cold run selected Q700)

Recent fresh deep runs exposed these decode gaps after the checksum loop:

    40846e64  002d 0007 0600  ori.b #7,0x600(%a5)
    4084bada  363b 3006       move.w (6,PC,D3.W),D3
    4084bade  4efb 3002       jmp (2,PC,D3.W)
    4084bb8e  205f            movea.l (%a7)+,%a0
    4084bc16  b230 2000       cmp.b (0,A0,D2.W),D1
    4084bc1c  4601            not.b D1
    4084bc1e  4630 2000       not.b (0,A0,D2.W)

These are now decoded with directed regressions.  The 40M stop after the
current-model checkpoint rebaseline was past the indexed CMP/NOT probe body and
sat in exception handling with vector 11 pending from `fault_pc=0xffffffff`
while the D-cache flush gate was active.

The ROM harness now distinguishes unbacked low RAM from open-bus device probes:
addresses in the low-memory decode window above the 64 MB `MemModel` backing
store return AXI `SLVERR`, while unmapped Q700 device probes such as
`0x51001c00` still return `0xffffffff/OKAY`.  A resume from `q700.final.vlt`
with `ROMBOOT_RESUME_MAX=5100000` now stops at:

    cycles:     33609848
    committed:  4989138
    last_pc:    0x4084bb9c
    next_fetch: 0x00000015
    d0/a0:      0x04000000
    top probes: rd 0x04000000 x2, rd 0x51001c00 x18

This confirms the RAM-size boundary is now visible to the core.  The remaining
frontier is exception/vector-table handling after that probe; the live state
still turns into the vector-11 `fault_pc=0xffffffff` path while D-cache flush
is active.

The saved pre-fix 40M checkpoints serialized the wrong CPU/register state:
`a0` points at the non-Q700 descriptor `0x408032d8`, whose service loop uses
`a3=0x50f04000`.  MAME validation says that is not a Q700 SCC alias; the
Q700 descriptor at `0x40803568` uses `0x50f0c020`.

Pre-fix 40M top probes were:

    rd 0x50f04000 x280443
    wr 0x50f04000 x24
    rd 0x50f81c00 x20
    rd 0x51001c00 x20
    rd 0x5000e000 x10

A fresh 2k-instruction run after canonicalizing the MAME mirror map removes
the valid Q700 aliases from the unmapped tally:

    rd 0x51001c00 x8
    rd 0x58000000 x3
    wr 0xffffff00 x1

The old pre-descriptor deep files are gone.  The current corpus includes fresh
16M/32M/40M checkpoints and passes inventory under the current filename
scheme.

Current no-patch descriptor-selection state:

    $ make rom-boot-descriptor-smoke
    ...
    [rom-boot] Q700 descriptor table entry accepted (entry=0x4000390c feature=0xc1e800c7)
    reason:     max-insts-reached
    committed:  5000
    last_pc:    0x40847518
    next_fetch: 0x40847518
    a1:         0x4080390c
    a3:         0x50f02000
    q700_descriptor_selected=1 entry=0x4000390c feature=0xc1e800c7

The ROM byte patches at `0x2f88/0x2f94` are removed.  The TB VIA1 model now
uses 6522-style DDRA/DDRB-gated port reads plus the Q700/MAME Port-A config
pins, so the ROM's table scan selects the Q700 descriptor-table entry
naturally.  This required fixing byte/word partial-register semantics for
register-destination MOVE.B/W, shifts/rotates, and logical ops; the old
`0x4000481a` / `a0=0x40003568` / `d0=0x07a3181f` stop-state was only an
intermediate milestone before those fixes.

`make rom-boot-descriptor-smoke` now gates this state by running the same cold
path, asserting `q700_descriptor_selected=1`, and grepping the trace for
`40847516` to prove the relocated checksum loop was reached.  The next task is
to continue from the fresh 16M/32M/40M checkpoints at the `0x4084bb9c`
memory-probe/exception frontier; keep `0x50f04000` unmapped unless fresh MAME
trace evidence proves otherwise.

---

## 3. Historical root cause: `CMP.L -(An), Dn` decoded as NOP

`rtl/core/decode/decode.v` (@ the 4'b1011 block, roughly line 2400):
the CMP/CMPA decode only accepts `CMP.L Dm,Dn` and the CMPA.L variants.
Memory-source forms — notably `CMP.L -(An),Dn` (`addressing mode 100`)
— are not decoded, so the predecoder sees a zero-size uop and decode
emits `UOP_NOP`.  This was an early bring-up blocker and is now fixed;
the notes below are retained as historical context only.

- A0 does not pre-decrement → the loop condition `CMP.L -(A0),D0` never
  compares the ROM signature at `0x3DE8` against D0.
- TST.L (A0) reads [0x3DEC] = the stored LEA base (ROM image value
  `0x0000_3DEC`), which is non-zero, so `Z=0`, and BNE branches back
  to 0x2E0C every iteration.
- Loop is the classic "walk backwards looking for the boot-table
  sentinel 0x0000_3DEC = the pointer's own value" — found in the
  Universal ROM's config-area probe.

### Verification

Instruction word `0xB0A0` decodes cleanly via Musashi:
`CMP.L (A0)-, D0`.  Our `tools/fuzz/fuzz.py --replay` would confirm
the Musashi semantics; not run in this bring-up (sim-first principle
applies — the divergence is obvious from the loop shape alone).

---

## 4. Historical follow-up tasks

This section records early bring-up tasks.  Several are already fixed;
the active next-task list lives in the Task tool (`TaskList`) and
`docs/uarch_proposals.md`.

Ordered by what unlocks the next milestone.  "unblocks N" = next
likely divergence-point once that task lands, estimated from ROM
content post-0x2e0e.

### 4.1 `rom-boot-decode-cmp-mem: CMP/CMPA memory-source forms`

Add decode paths to `rtl/core/decode/decode.v` 4'b1011 block:

- `CMP.{B,W,L} <ea>, Dn` — EA modes 2-6 (indirect, postinc, predec,
  disp16, indexed, PC-relative).  Cracks to: AGU/LOAD (EA → side effect
  write-back on mode 3/4; value onto CDB as src_a) + UOP_INT/ALU_CMP
  (src_a = loaded, src_b = Dn, flags-only).
- `CMPA.{W,L} <ea>, An` — same EA modes.  Same crack.
- `CMPI.{B,W,L} #imm, <ea>` — immediate + EA (read-modify with flags).
- `CMPM.{B,W,L} (Ay)+, (Ax)+` — if not already present.

Unblocks: the ROM config-area probe at 0x2E0C should exit within
~4 iterations (finds its own pointer at 0x3DE8).  Expected next
~50-200 instructions cover a Checksum/CRC compute loop — likely
exercises more memory-source ALU ops (ADD.L (An)+, Dn etc.).  Those
same memory-source patterns will gate progress until cracked.  This
is a **decode-iv** follow-up (see `tasks/decode-iv-*`) and likely
wants to be scoped to a dedicated agent.

### 4.2 `rom-boot-ram-sizing-bus-error: allow bus error on unmapped RAM reads`

**Status: SUPERSEDED by §4.2a — uniform open-bus per MAME-canonical (2026-04-28).**

Original status (kept for history): the ROM harness was updated to treat the
installed RAM size as the 64 MB actually backed by `MemModel`; addresses from
`0x04000000` through the rest of the low RAM decode window returned AXI
`SLVERR`, while unmapped Q700 device probes such as `0x51001c00` returned
`0xffffffff/OKAY`.  This was the "RAM-sizing-via-bus-error" strategy and it
introduced the vector-11 `fault_pc=0xffffffff` path noted below as a
follow-on issue.

That strategy diverged from MAME.  See §4.2a for the canonical replacement.

### 4.2a `mame-canonical: uniform open-bus everywhere (no RAM SLVERR)` (2026-04-28)

MAME's `macquadra700.cpp` (`install_ram` at line ~520) installs RAM only in
`0x00..install_size-1` with `memory_mirror = memory_end & ~memory_end = 0`
(no aliasing).  Past install_size up to `0x40000000`, and any I/O gap, are
**unmapped**.  The driver makes no `space.set_unmap_value(...)` call, so
MAME's default applies: **unmapped reads return `0xFFFFFFFF`/OKAY, writes
are silently dropped.  Bus errors are not raised on unmapped addresses.**

The Q700 ROM's RAM-sizing routine at `0x408046aa` (the SIMM-detect A3-read
loop) does NOT rely on bus error to find the RAM boundary; it relies on
either RAM aliasing (the routine writes a marker pattern and looks for it
to wrap) or open-bus reads (the routine sees `0xFFFFFFFF` past the install
boundary and treats that as "no RAM here").  MAME picks the latter.

**Canonical policy for our fabric (matching MAME):**

| Region                                | Response                       |
|---------------------------------------|--------------------------------|
| `0x00000000 .. install_size-1` (RAM)  | normal R/W                     |
| `install_size .. 0x3FFFFFFF` (RAM-OOR)| **`0xFFFFFFFF` + OKAY** (open bus) |
| `0x40000000 .. 0x4FFFFFFF`            | ROM mirror (1 MB × 256-mirror) |
| `0x50000000 .. 0x5FFFFFFF`            | I/O regions w/ gaps            |
| Any I/O gap (peripheral_bus SLOT_VOID)| **`0xFFFFFFFF` + OKAY** (open bus) |
| `0xF9000000 .. 0xF91FFFFF`            | VRAM (**fixed 2 MB** on Q700)  |
| `0xF9800000 .. 0xF98003FF`            | DAFB regs                      |

**RAM size is variable; VRAM size is fixed.**

* **RAM:** parameterized (`RAM_SIZE_BYTES` or equivalent build/runtime
  knob).  Q700 ships 4 MB default and can be expanded via SIMMs to
  4M / 8M / 20M / 36M / 68M (per MAME `set_extra_options` line ~852).
  The fabric's open-bus boundary at `install_size..0x3FFFFFFF` MUST
  follow the configured RAM size, not be hardcoded — otherwise the
  ROM's RAM-sizing routine sees the wrong boundary on builds that
  use a different RAM size than the fabric was hardcoded for.
* **VRAM:** **2 MB hardcoded** (matches MAME `map(0xf9000000,
  0xf91fffff)`).  Not parameterized.  Q700 has fixed-size VRAM; the
  ROM's display-detect path expects the canonical 2 MB layout.
  Current fabric is 512 KB (per 2026-04-28 JTAG bisect) — needs a
  4× upsize to silicon spec.

Implementation note: the SLVERR-on-unmapped-RAM logic in any harness or
fabric code (tb sim model, axi_xbar, axi_ddr4_mig_bridge) should be removed
or gated.  All unmapped reads — RAM-OOR or I/O gap — return open bus.

The pre-existing vector-11 `fault_pc=0xffffffff` path that prompted §4.2's
SLVERR strategy was a symptom of *something else* (MMU walker / dcache
flush interaction); switching to uniform open-bus removes the symptom we
were attacking and exposes the actual underlying issue cleanly.

### 4.2b `mame-canonical: xbar local-response → OKAY + 0xFFFFFFFF, VRAM = 2 MB` (2026-04-28)

Implementation landed (this branch).  Three logical changes:

1. **xbar local-response policy flipped from DECERR → OKAY + 0xFFFFFFFF.**
   `rtl/sys/axi_xbar.v`'s `XBAR_SLV_NONE` decode path used to drive
   `AXI_RESP_DECERR` with `rdata = 0`.  It now drives `AXI_RESP_OKAY`
   with `rdata = {DATA_WIDTH{1'b1}}`, mirroring the `peripheral_bus.v`
   SLOT_VOID semantics added in main `205aaba` / `3640fdd`.  The one
   remaining non-OKAY policy on the M0 (CPU) write path is ROM-mirror
   writes (mapped read-only region) → `BRESP=SLVERR`; M1 (XDMA host)
   and M2 (BOOT FSM) keep their `is_rom_loader_w` exemption for ROM
   provisioning.  All other unmapped CPU traffic — RAM-OOR
   (`RAM_SIZE..0x3FFFFFFF`), gaps between FB and ROM, sim-magic, the
   2 MB-past-VRAM zone — sees uniform open-bus.

2. **RAM-install size knob: `dbg_ram_window_lg2` (debug CSR, default 26
   = 64 MiB).**  The xbar's existing runtime selector clamps lg2 ∈
   [22..30] (4 MiB..1 GiB) and routes
   `[install_size..0x3FFFFFFF]` to `XBAR_SLV_NONE` when
   `RAM_ALIAS_MODE=0` (default since main `f90cb9c`).  Override
   pathways:
     * **Sim**: drive `dbg_ram_window_lg2` from a `tb-axi-xbar`-style
       harness or via the `Vfpga_top` debug-CSR write port.
     * **Bitstream**: write the debug-CSR through VIO/JTAG-AXI; see
       `rtl/core/debug/debug_ctrl.v` and `tb/tb_debug_ctrl.cpp`.
   The Q700 ROM SIMM-detect routine reads 0xFFFFFFFF past the
   install boundary and treats that as the "device absent" sentinel,
   so configuring lg2=22 simulates a 4 MB SIMM; lg2=26 simulates 64
   MiB.  Today's tb-fpga-top-rom sim leaves lg2 at the 64 MB default.

3. **VRAM resized from 1 MB → 2 MB silicon-spec.**
   `rtl/sys/vram.v` gained a `VRAM_BYTES` parameter (default 0x200000)
   that decouples the addressable aperture from the active framebuffer
   region (`FB_WIDTH_PX × FB_HEIGHT_PX × BPP/8`).  The framebuffer
   occupies a contiguous prefix; remaining VRAM bytes are still real
   RAM and follow normal R/W.  `AXI_VRAM_SIZE` in `axi_defs.vh`
   updated to `0x0020_0000`.  Future resolution changes (1024×768×8 →
   1152×870×8 → 1024×768×16) reuse the same 2 MB aperture without
   re-sizing the URAM bank.  KU5P URAM cost: 64 URAMs (50% of 128
   total), up from 16 at the 768 KB bank size.

**Diagnostic — unpatched `make tb-fpga-top-rom FPGA_TOP_ROM_MAX_INSTS=200000
FPGA_TOP_ROM_TIMEOUT=10000000`:**

| Build | final PC | Note |
|-------|---------:|------|
| Pre-fix (main @ `f94aa56`) | `0x4084751e` | inside ROM checksum loop |
| Post-fix (this branch)     | `0x4084751c` | inside ROM checksum loop |

Both PCs lie in the documented ROM checksum verification loop
(`0x40847510..0x4084751c`, see §2 above) — the run hits `max_insts=200000`
mid-loop in both cases.  The 2-byte offset between pre/post is just where
in the loop body the budget hit.  Neither run trapped to MacsBug; the
`[simm-probe]` (formerly `[busfault]`) printf is a one-shot register dump
when PC reaches the SIMM-detect routine at `0x408046aa`, NOT an actual
exception.  Verifying with `+probe` shows zero `[probe-exc]` events with
either build.

The fix unblocks the SIMM-detect probe at `0x50f01c00 + D2(=0x100000) =
0x51001c00`: previously the SLVERR/DECERR cascade derailed the routine;
now the probe returns 0xFFFFFFFF/OKAY and execution proceeds normally
into the legitimate ROM checksum work.  To bypass the checksum (~524k
iterations on a 1 MB ROM) and observe the next code phase, run with
`FPGA_TOP_ROM_PATCH=checksum`.

### 4.3 `rom-boot-mmu-integration-wire: wire mmu-walker into LSU/if_stage (#99)`

Phase-A MMU walker landed at `rtl/core/mem/mmu_walker.v` (task #74)
but isn't wired.  Q700 ROM sets up transparent-translation via
MOVEC into ITT0/DTT0 (we see this at 0x4072-0x4082) and expects those
to be honoured on subsequent accesses.  Not a divergence today
because all our accesses pass through unchanged, but we'll hit this
when the ROM enables MMU proper (after 0x2e00+ block validates).

### 4.4 `rom-boot-rtc-advance: VIA1 RTC seconds must advance per probe`

Landed: the ROM-boot VIA1 stub now has an active RTC/PRAM side-channel
behind PB0/PB1/PB2.  The same pin monitor that logs MAME-style falling-edge
traffic now decodes commands, drives PB0 during read data bits, stores PRAM
bytes, and returns the seconds counter low byte first for commands
`0x81/0x85/0x89/0x8d`.  The harness seconds value advances deterministically
every 50,000 host cycles so ROM polling can observe time without depending
on wall-clock or random state.

Use `make tb-rom-boot-rtc-smoke` for the directed harness selftest.  The
remaining production step is wiring the synthesizable `rtl/mac/rtc.v` into
the real peripheral fabric path rather than relying on the ROM-boot C++
model.

### 4.5 `rom-boot-via1-ifr-tim1: T1 IFR pending bit for VBL-setup probe`

After self-test, the ROM writes T1C-H to start the VBL timer and
polls IFR bit 6 for wraparound.  Stub returns 0 always → ROM hangs
in poll.  Need either: T1 countdown simulation in the TB's VIA1 stub,
or wire the real `rtl/mac/via1.v` into the ROM-boot harness's AXI
decoder.

### 4.6 `rom-boot-overlay-clear: verify overlay-off transition`

This needs a fresh audit.  Older harness paths reported that the Q700 ROM
cleared overlay at cycle 1254 through VIA1 ORB, but the current no-patch
descriptor run still reports `overlay: 1` at the 1.8M checksum-loop snapshot
and at the current deep stop.  The harness now separates instruction-fetch
overlay from data-side low RAM, matching the ROM's early `MOVEC VBR=0` vector
table setup, but that alone does not move the `0x4084bb9c` RAM-probe frontier.
Do not rely on the older `overlay: 0` notes until the VIA1 ORB trace and
snapshot serialization are rechecked.

### 4.7 `rom-boot-mame-trace: capture MAME macqd700 reference trace`

Per `docs/mame_integration.md` §6, we need a golden trace for
`tools/rom_trace_diff.py` to be useful.  Local status on 2026-04-21:
`/usr/games/mame` is installed at version `0.264`, and `macqd700` runs
headless if the Qt debugger is forced offscreen:

```bash
QT_QPA_PLATFORM=offscreen /usr/games/mame macqd700 \
  -rompath /dev/shm/m68k/mame_roms \
  -noreadconfig -skip_gameinfo \
  -debug -debugger qt -debugscript /dev/shm/m68k/mame_trace/trace.dbg \
  -video none -sound none -seconds_to_run 1 -nothrottle
```

The repo has the correct `420dbff3.rom` and the 1 KiB ADB PIC ROM
`342s0440-b.bin`, so stock `macqd700` MAME runs no longer need a zero-filled
PIC placeholder.  Older traces labelled `adb-pic-stubbed` are still useful
only for early CPU/control-flow comparison before ADB-sensitive behavior.

MAME `0.264` debugger traces with the current script emit PC/SR plus
disassembly text, not raw opcode words:

```text
PC=0000008C SR=2704 0000008C: move    #$2700, SR
```

Normalize that form with `--allow-missing-ir` and compare with
`rom_trace_diff.py --pc-only`:

```bash
tools/mame_trace_normalize.py /dev/shm/m68k/mame_trace/cpu.tr \
  -o /dev/shm/m68k/mame_trace/cpu.norm.tr \
  --allow-missing-ir

tools/rom_trace_diff.py build/sim/rom_boot_trace.log \
  /dev/shm/m68k/mame_trace/cpu.norm.tr \
  --pc-only --context 20
```

`make rom-frontier-diff` can now include a normalized MAME trace as an
optional third lane:

```bash
make rom-frontier-diff \
  ROM_FRONTIER_DIFF_MAME_TRACE=/dev/shm/m68k/mame_trace/cpu.norm.tr \
  ROM_FRONTIER_DIFF_MAME_PC_ONLY=1
```

For a committed stock artifact, capture a short pinned MAME trace with the real
ADB PIC ROM, record the MAME version and both ROM SHA1s, then check in the
normalized trace and metadata under `tb/traces/`.

---

## 4a. Progression after task #102 (rom-boot-decode-cmp-mem)

> Landing: CMP.L `<ea>,Dn` memory-source forms + CMPI.L memory dest +
> CMPM.L added to `rtl/core/decode/decode.v`'s `4'b1011` / `4'b0000`
> blocks; see task #102.

Rerun post-landing:

    $ make tb-rom-boot
    ...
    [rom-boot] cycles=5000-insts budget reached
    [rom-boot] last_pc=0x00002f60  next_fetch=0x00002f60

**The 0x2e0c loop exits after the first iteration** (A0 predec to
0x00003DE8, load = 0x00003DEC, CMP equals D0, Z=1, BEQ taken to
0x2e1a).  Execution continues through:

  - 0x2e1a: `SUBQ.B #8, D0`
  - 0x2e1c: `MOVEC` (yet another control-reg op)
  - 0x2e20..0x2e26: short `MOVEQ` / `LEA` / `JMP (d16,PC)` to 0x2f18
  - 0x2f18..0x2f20: `BSET.L #7,D7` / `MOVEA.L A7,A5` / `TST.L D2` /
    `BEQ .+52` — the ROM is now probing for something at 0x2f52
  - 0x2f52..0x2f60: `MOVE.L D6,D0` / `LEA (d16,PC),A1` /
    `ADDA.L (A1)+,A0` / `MOVEA.L (d16,A0),A2` / `JMP (d8,A0,A2.L)`

The last committed PC at the 5000-inst cap is 0x00002f60 — a `JMP
(d8,A0,A2.L)`.  Two decode gaps gate progress here:

  1. `ADDA.L (An)+,An` (and other memory-source modes) was silently
     emitted as UOP_NOP — only register/#imm sources were handled
     in the 4'b1101 block.  Because of this A0 never picked up the
     real ROM pointer, and the subsequent `MOVEA.L (d16,A0),A2`
     loaded garbage (A2 ≈ 0xfdc06000 instead of a valid code addr),
     and the JMP target was bogus.
  2. `JMP (d8,An,Xn)` (mode 6) was also UOP_NOP — only `JMP (An)`
     (mode 2), `JMP (xxx).L`, and `JMP (d16,PC)` were handled.

## 4b. Progression after task rom-boot-decode-adda-jmp-idx

> Landing: ADDA.L `<ea>,An` memory-source forms (modes 010, 011, 100,
> 101, 111/000, 111/001, 111/010) + `JMP (d16,An)` (mode 5) +
> `JMP (d8,An,Xn.L*1)` (mode 6, brief extension, Xn.L, scale ×1)
> added to `rtl/core/decode/decode.v`.  Multi-µop cracks use TMP1
> staging, mirroring the CMP.L memory-source pattern.  Xn.W, full
> extension, and scale != 1 are deferred as future work (emit UOP_NOP
> — same pre-landing behaviour).

Rerun post-landing:

    $ make tb-rom-boot
    ...
    [rom-boot] committed=108  last_pc=0x000047c6  next_fetch=0x00600012

**Progression: 49 → 108 committed, last PC moved 0x00002f60 →
0x000047c6.**  The JMP at 0x2f60 now resolves to A0+A2 = 0x3162 (a
valid ROM dispatch), and execution continues through:

  - 0x3162..0x3170: short MOVEA/LEA/JMP chain to 0x46aa
  - 0x46aa..0x46d4: A3-read loop / short MOVEA / JMP (A6) back to
    0x3178 then back into the A1-pointer walk at 0x2f58
  - 0x2f58..0x2f6e: `MOVEA.L A1,A0` / `ADDA.L (A1)+,A0` /
    `MOVEA.L (d16,A0),A2` / `JMP (d8,A0,A2.L)` — second iteration
    of the dispatch (different target this time)
  - 0x47ae..0x47c6: byte-level decode of some dispatched handler,
    halting on `AND.B (d16,A0),D1` (opword 0xC228) — the next
    decode gap.

### Next decode gap: `AND.B <ea>,Dn` memory-source

The halt at 0x47c6 is `AND.B (d16,A0),D1` (opword `0xC228 + d16`).
The 4'b1100 block currently only decodes `AND.L Dn,Dm` (reg-reg) and
ABCD.  The full AND family (AND.B/.W/.L with all EA modes) is
missing, as is `AND <ea>,Dn`, `AND Dn,<ea>`, and `OR <ea>,Dn` / 
`OR Dn,<ea>` (same shape in 4'b1000).  This is the next
`rom-boot-decode-and-or-mem` follow-up.

Follow-ups #4.2..#4.7 remain unchanged.

---

## 4c. Progression after task #127 (rom-boot-decode-and-or-mem)

> Landing: AND/OR.{B,W,L} memory-source (mode 010/011/100/101 +
> 111/000/001/010) and memory-dest RMW (mode 010/011/100/101 +
> 111/000/001) added to the `4'b1100` / `4'b1000` blocks of
> `rtl/core/decode/decode.v`.  Plus .B/.W widening of the reg-reg
> forms (`AND.{B,W} Dm,Dn` / `OR.{B,W} Dm,Dn`).  All cracks use the
> existing CMP.L / ADDA.L TMP1 staging pattern:
>
> - mem-src: LOAD TMP1 ← (ea); ALU_{AND,OR} Dn = Dn op TMP1 (NZVC).
> - mem-dst: LOAD TMP1 ← (ea); TMP1 = TMP1 op Dn (NZVC); STORE TMP1 → (ea).
>
> Retrospective update: this landing originally inherited the old ALU
> byte/word truncation behavior.  The current core preserves upper Dn bytes
> for byte/word register-destination logical ops, matching the 68k PRM and
> the ROM descriptor-probe sequence.
>
> Directed coverage: `tb/tests/asm/and_or_mem_basic.s` (11 scenarios,
> long only — MOVE.B priming path is not decoded).  Fuzz widening:
> `emit_and_mem_*` / `emit_or_mem_*` in `tools/fuzz/gen_program.py`.

Rerun post-landing:

    $ make tb-rom-boot
    ...
    [rom-boot] committed=98  last_pc=0x000047b4  next_fetch=0x000047d8

**Result: the AND.B at 0x47c6 DOES decode** — the DEBUG dispatch log
shows the crack emits its LOAD+ALU_AND pair at PC 0x47c6 (with d16 =
0xfffffff4 = -12 correctly applied).  Directed test
`and_or_mem_basic` PASSes (11 scenarios, 111 committed), fuzz
N=200 posts 0 MISMATCH.  Lint clean.

**But committed count / last-PC are UNCHANGED from the pre-landing
baseline (98 / 0x000047b4).**  This is **not** caused by the AND/OR
decode — the fetch front-end had already stalled before reaching
0x47c6 on this main commit.  The stall sits around the BEQ.S +0x44
at PC 0x47b4 (opword 0x6744) fall-through path, where fetch reaches
0x47d8 (past my new AND.B decode, PC 0x47c6) but no new retirement
happens.  Pre-landing the same halt was at committed=98 last_pc=
0x000047b4 too — so the §4b doc's "0x000047c6" post-baseline
appears to reflect a slightly earlier state (perhaps before one of
the post-ADDA intervening commits).

### Next follow-up: ROM front-end stall at PC 0x47b4 BEQ

Recommended new task (separate agent, different track): figure out
why the front-end is stuck at PC 0x47b4 (BEQ.S) post-ADDA-landing.
Hypothesis: the BEQ speculatively goes fall-through into a long run
of undecoded NOPs (MOVE.B (d16,An),Dn etc. at 0x47ba/47c0/47ca/…);
those NOPs don't commit, the BEQ can't retire until its flag tag
resolves, and something in the iq_mem bitmap blocks retirement.
Decode work to unblock: add MOVE.B mem-source + MOVEA.L (d16,An),An
+ ASL/ASR.B #imm,Dn, likely in a `rom-boot-decode-moveb-movea-idx`
followup.  With those landed, the AND.B decode this task adds
SHOULD be directly visible in the commit-count delta.

Follow-ups #4.2..#4.7 remain unchanged.

---

## 4d. Progression after task rom-boot-decode-moveb-movea-idx

> Landing: `MOVE.B (d16,An),Dn` mem-source crack (LOAD TMP1 + ALU_MOV
> byte-size) + byte/word shift variants on the 4'b1110 block (widen
> the existing `op[7:6]==2'b10` gate to `op[7:6]!=2'b11`, keep the
> `{1'b0, op[7:6]}` size encoding that was already in-flight for the
> MUL/DIV cracks).  `MOVEA.L (d16,An),An` was already decoded at
> line 1568-1578 of `decode.v` — confirmed no new path needed.
>
> Directed coverage: `tb/tests/asm/moveb_shift_bw_basic.s` — 8
> scenarios covering positive / negative / zero MOVE.B with flag
> checks, byte ASL/ASR, word LSL/LSR in the rom-boot shape, and a
> register-count ASL.W.  Fuzz widening: `emit_shift_bw_imm` and
> `emit_moveb_mem_src_disp` added to `tools/fuzz/gen_program.py`.
> Retrospective update: the ALU now narrows byte/word shifts/rotates and
> register-destination MOVE.B/W with a merge-into-old-Dn path, so upper Dn
> bytes are preserved.  `moveb_shift_bw_basic` was widened to check full
> register values and the ROM's VIA-style byte-probe sequence; the fuzz
> weights can now be raised family-by-family with the Musashi end-state
> comparator instead of relying only on random fuzz.
>
> Hypothesis from §4c was wrong in detail but right on effect.  The
> BEQ.S at 0x47b4 was NOT blocked by iq_mem bitmap pressure — it was
> blocked because `op_mode_lo == 3'b101` MOVE.B was UOP_NOP with
> `len_bytes = 4'd2`, consuming only 2 of the 4-byte inst (leaving
> the 16-bit extension word at 0x47bc to decode as a bogus opword).
> The NOP + mis-aligned fetch pushed enough junk into iq_int /
> iq_mem that the pipe wedged.  With the 4-byte crack in place,
> dispatch cleanly emits LOAD+ALU_MOV and fetch moves on.

Rerun post-landing:

    $ make tb-rom-boot
    ...
    [rom-boot] cycles=10855 committed=5000  last_pc=0x00002f80  next_fetch=0x00002f88
    [rom-boot] reason: max-insts-reached

**Progression: committed 98 → 5000 (5000-cap hit), last PC moved
0x000047b4 → 0x00002f80.**  Execution now races through:

  - 0x47b4..0x47fa: byte-extract / byte-reassemble block that
    builds a 16-bit value from two ROM-table byte reads (the
    LSL.W/LSR.W #8 pattern the task brief anticipated).  Most
    MOVE.B Dm,Dn / MOVE.B Dn,(d16,An) ops in this block still NOP
    silently — tracked as a `rom-boot-decode-moveb-rest` follow-up.
  - 0x2f6e..0x2f7c: second iteration of the ADDA-driven dispatch
    from §4b, this time landing at 0x2f7c.
  - 0x2f7c..0x2f88: **new halt loop** — a ROM-config-table walker:

        0x2f7c  2018         MOVE.L (A0)+, D0      ; post-inc (d)
        0x2f7e  67d2         BEQ.S  -0x2c → 0x2f52
        0x2f80  43f0 08fc    LEA    (d8,A0,Xn.L*1), A1
        0x2f84  b429 0013    CMP.B  (0x13,A1), D2
        0x2f88  66f2         BNE.S  -0xe  → 0x2f7c

### Next decode gaps (for a `rom-boot-decode-movel-postinc-cmp-b` follow-up)

The 0x2f7c loop body needs three ops we don't decode:
  1. `MOVE.L (An)+, Dn`  (mode 011, post-inc).  Likely 2-µop crack:
     LOAD + increment of An (TMP1-free single-µop is possible if
     the LSU broadcasts the post-inc An on CDB, mirroring RTS).
  2. `LEA (d8,An,Xn.L*1), Am`  (mode 6, brief extension, scale=1).
     The ADDA-landing task (§4b) already did `JMP (d8,An,Xn.L*1)`
     with the same addressing mode; LEA is the same AGU ea, but
     writes An instead of branching.
  3. `CMP.B (d16,An), Dn`  (byte mem-source).  Same pattern as
     AND.B mem-source (§4c), different ALU op.  2-µop crack:
     LOAD TMP1 + ALU_CMP.

With those three landed, the loop body executes and the BEQ.S at
0x2f7e can terminate (Z=1 on table-sentinel).  Expected next halt:
somewhere past 0x2f52, still inside Universal ROM early bringup.

Follow-ups #4.2..#4.7 remain unchanged.

---

## 4e. Progression after task rom-boot-decode-movel-postinc-cmp-b

> Landing: `MOVE.L (An)+,Dn` (mode 011 postinc) + `LEA (d8,An,Xn.L*1),Am`
> (mode 6, brief extension, Xn.L, scale ×1) added to `rtl/core/decode/
> decode.v`'s 4'b0010 and 4'b0100 blocks.  `CMP.B (d16,An),Dn` was
> verified to ALREADY be decoded (task #102's op[8]==0 && op[7:6]!=2'b11
> && op_mode_lo==3'b101 branch handles byte-size via {1'b0, op[7:6]}
> size encoding — no new code needed; the task brief's §4d listing of
> three gaps was defensive inventory).  Cracks:
>
>   * MOVE.L (An)+,Dn — 2 µops: P0 UOP_LOAD Dn←(An) with flags_wr=NZVC
>     (uses Task #47's LSU→CCR-CDB broadcast path for long mem-source);
>     P1 UOP_INT/ALU_ADD An = An + 4, flags_wr=0 (ADDA-style).  Simpler
>     than the 3-µop TMP1 pattern the brief suggested — long-size loads
>     already plumb CCR metadata through iq_mem post-#47.
>   * LEA (d8,An,Xn.L*1),Am — 2 µops: P0 TMP1 = An + sx8(disp8);
>     P1 Am = TMP1 + Xn (src_b = Xn).  Mirrors the JMP (d8,An,Xn.L*1)
>     crack (§4b) minus the final BR_JMP.
>
> Directed coverage: `tb/tests/asm/rom_boot_loop_exit.s` — 8 scenarios
> covering MOVE.L (An)+ with value / Z-flag / N-flag checks, LEA
> indexed (negative disp, D/A bit, zero-index), CMP.B (d16,An) match
> and mismatch, and the full integrated ROM-loop walker shape.  Fuzz
> widening: `emit_movel_mem_src_postinc` + `emit_lea_indexed` in
> `tools/fuzz/gen_program.py` at weight 0 (matches the brief's
> "don't break fuzz" directive — can be raised once the next ROM-boot
> pass proves no peripheral divergence).

Rerun post-landing:

    $ make tb-rom-boot
    ...
    [rom-boot] cycles=502884 committed=584  last_pc=0x00002f88
    [rom-boot] reason: no-progress (front-end stalled)

**Progression: committed 5000 (cap) → 584, last PC moved
0x00002f80 → 0x00002f88.**  The 0x2f7c..0x2f88 loop now executes
cleanly for ~28 iterations, exits the BNE at 0x2f88 multiple times,
dispatches the fall-through block at 0x2f8a..0x2f94, and loops back
to 0x2f7c on the BNE at 0x2f94.  A0 progresses from 0x00003DEC →
0x000031F4 (valid ROM-table walking); A1 takes real values from the
LEA; D0 picks up real table entries from MOVE.L (An)+.

### Next halt: front-end stall at PC 0x2f88

At cycle 502884 fetch has stalled at next_fetch=0x2f80 but the
last-committed PC is 0x2f88 — classic "back-end drained, fetch
wedged" pattern.  The trace shows the loop is cleanly processed
for 28 iterations, then something blocks new fetches while the
ROB drains.  Not caused by our decode additions (every µop in the
loop dispatches and commits).  Most likely candidates: (a) an
iq_mem bitmap accounting bug on the MOVE.L (An)+ postinc cracks
when 28 outstanding postinc stores aren't properly tracked; (b) a
BPU or RAS side-effect where 0x2f88 BNE's BTB target decays
mid-run.  This is a **separate follow-up** — out of scope for the
task brief's "STOP and file a follow-up if new halt past 0x2f88
exposes a different issue."  File as `rom-boot-2f88-fetch-stall`.

Follow-ups #4.2..#4.7 remain unchanged.

---

## 5. Tooling summary

- `tb/tb_rom_boot.cpp` — harness.
- `tools/mame_trace_normalize.py` — MAME tracelog → our format.
- `tools/rom_trace_diff.py` — first-divergence detector.

### Typical workflow (once a MAME trace exists)

    # our trace
    make tb-rom-boot
    # MAME trace → normalise
    ./tools/mame_trace_normalize.py raw_mame_trace.tr \
        -o tb/traces/macqd700_cold.tr --limit 10000
    # Current tb-rom-boot resets directly to 0x4000002A, so only strip
    # trace headers before feeding rom_trace_diff.
    grep -v '^#' build/sim/rom_boot_trace.log > /tmp/ours.tr
    ./tools/rom_trace_diff.py /tmp/ours.tr \
        tb/traces/macqd700_cold.tr --context 20

Today diff against MAME isn't possible because the reference trace
isn't captured yet (see §4.7).

### MAME capture recipe (copy-paste)

    git clone --depth 1 --branch mame0287 https://github.com/mamedev/mame.git /tmp/mame
    cd /tmp/mame
    make SUBTARGET=macoracle SOURCES=src/mame/apple REGENIE=1 -j4
    mkdir -p roms/macqd700
    cp /path/to/m68k-ooo/files/420dbff3.rom roms/macqd700/420dbff3.rom
    cat > trace.dbg <<'EOF'
    trace cpu.tr,,,{tracelog "PC=%08X SR=%04X ", pc, sr}
    go
    EOF
    ./mame_macoracle macqd700 -debug -debugscript trace.dbg \
        -rompath roms -nothrottle -seconds_to_run 0.2
    # Output: cpu.tr  (then normalise per above)

Record the MAME binary commit + our ROM hash when checking in the
reference trace; the combination is the oracle identity.

---

## 6. File-conflict / cross-worktree notes

- **glue-real** worktree (`agent-ad26ac19`, commit 39a21f4) adds a
  proper Q700 address decoder in `rtl/mac/glue.v` + wires it into
  `rtl/mac_top.v` as an observational witness.  We did **not** pull
  that in — the ROM-boot harness does all decode in the TB to avoid
  stepping on an unmerged RTL change.  When glue-real merges, the TB
  peripheral stubs should be re-evaluated for whether to delegate
  to the real VIA1/VIA2 modules.
- **bus-error-fmt2** worktree owns `tb/tb_top.cpp`, `rtl/core/mem/lsu.v`,
  `rtl/core/m68k_core.v`, `rtl/core/exception.v`.  We touched **none**
  of those.  When it lands, §4.2 above (bus-error on unmapped RAM)
  becomes straightforward.

---

## 7. Honesty ledger

- The harness ran the ROM end-to-end without crashing.  Good.
- We executed ~200 committed ROM instructions before hitting the
  CMP-memory-source blocker.  Good.
- We did **not** get to the overlay-clear, RAM-size probe, or IFR-poll
  phases.  Follow-ups §4.1 through §4.3 are required before those
  phases come into view.
- No MAME trace was captured — §4.7 is a real blocker for meaningful
  divergence analysis on complex ROM sequences.  The best we can do
  in this agent's scope is the symbolic analysis above, which is good
  enough to justify the §4.1 task.
- CCR values in the trace may lag by one commit (the dump reads
  `ccr_prf[crat_tag]` at the tick that `dbg_committed` increments —
  same-cycle race with the CCR-RAT commit path).  This is cosmetic;
  PC + IR are definitive.
