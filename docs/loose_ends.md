# Loose Ends

## 2026-04-22: Patched ROM frontier results are not canonical

The A-line/F-line frontier seen through patched ROM runs is suspect.  We have
seen patch side effects masquerade as core/frontier bugs, especially around RAM
geometry, low exception vectors, and overlay teardown.

Current policy:

- Stock ROM runs are the default for frontier and stop-summary commands.
- `+rom_patch=...` is still allowed for bounded experiments, but the result is
  not a source of truth until the same milestone reproduces without patches.
- Any new ROM patch needs a narrow smoke that proves it preserves the relevant
  helper call/return contract and does not perturb RAM sizing or exception
  vector setup.

Specific shortcut in flight:

- `+rom_patch=rtc-pram-mame-state` is a MAME-derived 4 MB Q700 shortcut for
  the early RTC/PRAM sizing loop starting at `0x4084721c`.  It recreates the
  MAME register/SR state observed at the later RAM-fill helper entry
  `0x40847280`, then branches there.
- `+rom_patch=ramtest-mame-state` is a MAME-derived acceleration for the
  early forward RAM pattern helper at `0x40847280`.  MAME's Q700 4 MB run
  exits that helper at `0x4084737c` with `D0-D2=0`, `D3-D5` holding the
  `6d/b6/db` pattern words, `D6=0`, `A2=A1-4`, and `SR=0x2714`.
- The patch replaces the helper entry with that return-state sequence and the
  harness host-fills the helper's `A0..A1` RAM range with the matching
  `6d b6 db` pattern when the patched entry retires.
- `+rom_patch=alias-probe-mame-state` skips the later RAM lane/alias probe at
  `0x4084bb74` by restoring the MAME-observed low-memory side effect, setting
  the post-probe 4 MB register/SR state, and jumping to `0x4084bc38`.
- `+rom_patch=ram-list-sentinel-fast` replaces the RAM descriptor-list
  sentinel compare at `0x40846ed2` with an equivalent `A0+1` zero test while
  leaving the ROM's existing `BEQ.S` in place.  This is a shortcut around the
  patched-frontier CMPA/branch interaction; keep the directed regression
  `cmpa_word_imm_sentinel_branch` in the decode/adversarial set.
- `+rom_patch=via-timer-mame-state` skips the later VIA timer interrupt
  diagnostic waits at `0x40847bf6` and `0x40847caa` by restoring the
  MAME-observed validation counters (`D3=10,D4=0xa5,D5=1` for the first wait,
  then `D3=10,D4=1,D5=1` for the second).  This covers a harness wiring gap:
  the C++ ROM boot VIA stub logs T1/IFR activity, but that interrupt is not
  currently driven into the core as `cpu_ipl_ext`.
- `+rom_patch=mame-fastdiag` combines the MAME-derived PRAM/RAM helper
  shortcuts with `checksum-fast`, `meminit-fast`, and
  `alias-probe-mame-state`/`ram-list-sentinel-fast`/`via-timer-mame-state`
  for ROM frontier iteration.
- Treat these as iteration shortcuts only.  They are useful for getting to the
  next ROM frontier quickly, but the same frontier still needs confirmation
  with stock ROM plus `+q700_ram=4M` or a MAME-side trace.
- The current timer-skipped frontier reaches DAFB register traffic and SCSI
  polling, then takes the first Toolbox A-line trap at `0x40800502`
  (`_SetApplLimit`).  The secondary low-PC illegal is caused by vector 10
  dispatch reading handler `0x00000000` from low vector slot `0x00000028`.
  Treat this as an exception/vector-table fidelity issue until proven
  otherwise.

## 2026-04-22: VRAM/JTAG framebuffer bring-up is not a pure black screen anymore

Hardware context:

- Loaded bitstream:
  `/home/qwertyoruiop/m68k-ooo-worktrees/vivado-100mhz-timing-20260422/build/vivado/fpga_top.bit`.
- Build knobs: real MIG, VIO, JTAG AXI, `CORE_CLK_DIVIDE=1`,
  `CORE_CLK_HZ=100000000`, `HDMI_TEST_PATTERN=0`, `VIDEO_SMOKE=1`.
- Post-route timing met: WNS `+0.036 ns`, WHS `+0.021 ns`.
- HDMI pclk/local path had already shown the rainbow test pattern with `HDMI_TEST_PATTERN=1`.
- The current test uses `HDMI_TEST_PATTERN=0` and JTAG AXI writes into the
  VRAM aperture.

What was tried:

- Programmed the FPGA and took VIO/JTAG snapshots. DDR calibration completed,
  HDMI MMCM/I2C completed, scanout counters moved, the ROM preload completed,
  and the core later showed nonzero commits.
- Ran `tools/jtag_bringup_tui.py video-poke --go --hold-cpu 0x100 0x61e 48 96`.
- A follow-up JTAG worker repeated smaller pokes and confirmed VRAM readback:
  `0xF9000100 -> 0x09090909`, `0xF9000000 -> 0x09090909`, and
  `0xF9002400 -> 0x0A0A0A0A` for the stride-1152 row-8 case.
- AXI error visibility stayed clear (`axi_error seen=0 sticky=0 raw=0x0000`).
- The user observed a blue line at the top of the HDMI display while the JTAG
  poke was running.
- The final snapshot showed many VRAM writes and nonzero DAFB state.
- Sampled VIO still showed `vram_rd_en=0`, `vram_rd_valid=0`, and
  `vram_rd_addr=0x007ff`; this does not prove scanout never reads VRAM, only
  that the current sparse VIO sampling did not catch active reads.
- The generated LTX did not expose `probe_in17` / fb_reader stats, so request,
  response, and miss counts were unavailable on this programmed bitstream.

Important correction:

- The current JTAG AXI master goes through xbar M1 and can reach the
  `0xF900_0000..0xF90F_FFFF` VRAM aperture.
- This branch fixes the DAFB blind spot by routing
  `0xF980_0000..0xF980_03FF` through xbar S4 into the live DAFB shim.
  On bitstreams built from this branch, JTAG writes to
  `0xF980_0008/+0x0c/+0x10` should program the same base/stride/BPP latches
  the CPU uses.

Likely interpretation:

- We have reached a more useful hardware frontier than the old all-black state:
  DDR calibrates, ROM preload completes, the core runs, the ROM appears to
  reach some DAFB/VRAM path, and the monitor can show at least a line sourced
  from the framebuffer path.
- The remaining issue is likely one or more of:
  bad ROM-programmed DAFB base/stride/BPP due to CPU/DAFB write semantics,
  scanout fetching only a narrow band, line-buffer priming/restart behavior,
  or stale bitstreams that predate the shared DAFB AXI path.

Next useful fixes:

- Short term: rebuild/program a framebuffer bitstream from this branch and
  verify JTAG writes to `0xF980_0008/+0x0c/+0x10` are visible in the live DAFB
  VIO config fields.
- Keep using JTAG to verify raw VRAM read/write coherence at `0xF900_0000`.
- Add VIO/ILA probes for line-buffer/scaler request state, fb_reader
  request/response FIFO levels, `sc_rd_en`, `sc_rd_valid`, `fb_vram_rd_en`,
  and underflow/drop counters.
- Re-run a framebuffer bitstream after DAFB is JTAG-programmable so a host
  poke can distinguish scanout bugs from ROM DAFB programming bugs.

## 2026-04-27: monitor_gate_d7 (`0x40849b08`) trips on a real RTL gap, not a patch shortfall

Symptom.  `make tb-fpga-top-rom FPGA_TOP_ROM_PATCH=mame-fastdiag,chime-skip
+fail_monitor` runs deterministically to `t=11_928_560 retired=371_003`,
hits PC `0x40849b08` (the `btst #26,%d7 / bne 0x40849b28` test that gates
entry into `rom_monitor` at `0x4084a7e6`), and exits rc=3.  Layering
`via-timer-mame-state` on top is a no-op (the bytes are already in their
patched state from `mame-fastdiag` — the harness logs `old=X new=X` for
every byte) and produces a byte-identical retire trajectory.  The fail-PC
is therefore not a patch-set calibration issue; it is a real divergence
from MAME-equivalent state.

What `0x40849b08` actually tests.  Continuation `(%pc@(0x40849b08)),%fp`
followed by `jmp %pc@(0x408470ba)` then `btst #26,%d7`.  D7 is the ROM's
boot-diagnostic result vector — each bit is the pass/fail of one
diagnostic.  Bit 26 clear → branch through `0x40849b24 → 0x4084a7e6`
(rom_monitor, the sad-Mac handler).

What sets D7[26].  The subroutine at `0x408470ba` is the producer.  First
instruction `oriw #1792,%sr` raises IPL to mask interrupts, then sets up a
pattern-write / aliasing-compare loop using `magic = 'Shel' = 0x53686c`,
`d2 = 0x40000` (256 KB stride), with `bset #1,%a2@(1)` and `bset #2,%a2@(1)`
recording per-bank pass flags.  This is a **RAM SIMM-size / aliasing
probe**: it writes a sentinel + 0xFFFFFFFF, compares at a stride to detect
back-of-bank aliasing, and uses the result to populate the per-size flags
that aggregate into D7[26].

This is not the VIA timer interrupt path that `via-timer-mame-state`
papers over.  This is RAM-test verify, run after the host-fill of 4 MB
through `ramtest-mame-state`.  The fast-fill writes 4 MB into DDR
through a host backdoor (`tb_fpga_top_rom.cpp:451-466`,
`maybe_apply_ramtest_mame_state_fastfill`) and brute-invalidates the
entire D-cache, but the verify pass walks RAM through the real LSU / MMU
/ AXI.  The lockstep audit (`docs/.../mame_lockstep_audit_2026-04-27`)
flags this directly: "writes 4 MiB into DDR through a host backdoor and
brute-invalidates the entire D-cache.  This bypasses LSU, MMU, AXI —
exactly the paths a real bug would live on."

Investigation hypotheses (most likely first).
1. **Cache coherence between host-side DDR writes and RTL D-cache lookup**.
   `invalidate_dcache_for_host_ram_patch` (`tb_fpga_top_rom.cpp:370`) brute-
   invalidates the D-cache, but the verify pass then re-reads the RAM
   range through the LSU.  If the cache invalidate doesn't cover an
   aliased path or evict a stale dirty line cleanly, the verify reads
   stale data → aliasing miscompare → wrong D7 bit.
2. **MMU / TLB state mismatch** on the post-fast-fill verify.  ITT/DTT
   passthrough is supposed to map the test region directly, but if the
   verify path crosses a 4 KB boundary or hits an uncached I/O alias
   the brute-invalidate didn't reach, the read returns differently than
   the write.
3. **AXI write-merging / WSTRB lane aliasing on the DDR model**.  The
   tb's big-endian lane mapping (`CLAUDE.md:619-624`) is correct for the
   DAXI master, but the host-fill path writes via `ddr_write_phys_byte`
   (post-merge from worktree).  Need to verify byte-equivalence between
   host-fill output and what the LSU subsequently reads.
4. **Genuine RAM-size descriptor wrong**.  Q700 expects 4 MB; if any
   peripheral or the boot descriptor reports a different size to the
   diag, the aliasing test runs against a wrong stride and misfires.
5. **Last resort**: a real RTL bug in LSU/D-cache/MMU.  Less likely given
   the rest of the boot survives, but possible at the
   pattern-write/aliasing-compare access pattern this diag uses.

Where to start.
1. Reproduce: `make tb-fpga-top-rom FPGA_TOP_ROM_PATCH=mame-fastdiag,chime-
   skip FPGA_TOP_ROM_EXTRA="+fail_monitor +probe"`.  `+probe` (line 441)
   logs the pre-monitor PC history with full reg + SR + CCR snapshot.
2. Disassemble the path from where D7 is last written before the gate;
   find the diag step that should have set bit 26 and didn't.
3. Compare the live RTL trace against a MAME `macqd700` run of the same
   ROM at the same PC — the Tier-2 bridge
   (`tools/mame_axi_periph_bridge.cpp` + `tools/mame_q700_rtl_overlay.py`,
   weakness #5 in lockstep audit) emits `mame-mmio-divergence` events
   that are exactly the right channel for this comparison.  Plumb the
   socket into `tb_fpga_top_rom.cpp:660` so divergences fail loud
   instead of allowing the sim to wander into the monitor.
4. If the divergence is at LSU-level (cache/MMU/AXI), narrow with the
   existing `tb-lsu` / `tb-dcache` / `tb-dcache-burst` / `tb-mmu` unit
   tbs and add a directed test that reproduces the access pattern.

Acceptance criteria.
- `make tb-fpga-top-rom FPGA_TOP_ROM_PATCH=mame-fastdiag,chime-skip
  FPGA_TOP_ROM_EXTRA="+fail_monitor"` reaches PC ≥ `0x40849b28` (the
  good-path branch target after the gate) **without** layering
  `monitor-skip` / `sad-mac-boot-skip`.
- The same is true with **`FPGA_TOP_ROM_PATCH=chime-skip` only** (i.e.
  no `mame-fastdiag` umbrella, no MAME-state pre-fills).  This is the
  hardware-valid bar: the RTL completes the RAM diagnostic on its own.
- A directed unit test reproduces the failing access pattern in
  isolation (`tb-lsu` / `tb-dcache-burst`) so future regressions in this
  area surface fast.

Anti-patterns (do not).
- Do **not** add a new `monitor-skip` / `sad-mac-boot-skip` patch to the
  smoke preset.  That just papers over the divergence at a higher PC and
  the next agent will trip it again at the next monitor entry.
- Do **not** widen the host-side `maybe_apply_ramtest_mame_state_fastfill`
  to cover more RAM regions.  The audit already calls this out as
  cache-coherence-bug-masking.  The fix is to either (a) make the host
  fast-fill genuinely transparent to LSU/MMU/D-cache, or (b) replace it
  with an in-ROM patch that issues `move.l #...,(a0)+` so the same path
  the verify uses is the one that filled.
- Do **not** treat this as a regression introduced by the recent
  reset-fix wave (`4a5968d…aa74e3b`).  The byte-identical retire
  trajectory across narrow + wider patch sets rules out reset-arrival
  drift.
