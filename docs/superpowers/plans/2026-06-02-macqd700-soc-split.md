# macqd700-soc Platform Split — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Carve the Quadra-700 platform out of `m68k-ooo` into a new
history-preserving repo `~/macqd700-soc/`, behind a dual-AXI (128/256) + IRQ
CPU socket, organized into machine/board/soc layers, with the m68k integrated
as a git submodule.

**Architecture:** 3 layers — CPU repo (m68k-ooo + `m68k_axi_wrapper`) ↔ AXI
socket ↔ macqd700-soc (`rtl/mac` machine, `rtl/board` KU5P BSP, `rtl/soc`
integration). SoC builds standalone against a `cpu_stub`; the real CPU mounts at
`cpu/` as a submodule. Rich CPU-specific debug relocates to the CPU repo.

**Tech Stack:** Verilog-2005, Verilator (lint/sim), Vivado 2025.2 (synth), git
filter-repo, git submodules.

**Spec:** `docs/superpowers/specs/2026-06-02-macqd700-soc-split-design.md`

**Conventions used below:**
- `M68K=/home/qwertyoruiop/m68k-ooo` (source/CPU repo), `SOC=/home/qwertyoruiop/macqd700-soc` (new).
- "Elaborate" = `verilator --lint-only --top-module fpga_top` (the `lint-fpga-top` recipe).
- Commit after every task. Never `git push` (local-only per project norm).

---

## Phase 0 — Safety net & baseline

### Task 0.1: Tag baseline + record file inventory
**Files:** none (git + scratch).

- [ ] **Step 1:** Tag the pre-split point in `m68k-ooo`.
```bash
cd $M68K && git tag pre-soc-split-$(git rev-parse --short HEAD) HEAD
git tag | grep pre-soc-split
```
- [ ] **Step 2:** Snapshot the exact platform file inventory (used to verify nothing is lost).
```bash
cd $M68K
{ ls rtl/fpga_top*.v rtl/fpga_top*.vh rtl/mac_top.v;
  find rtl/mac rtl/sys rtl/vendor -type f; } | sort > /tmp/platform_inventory.txt
wc -l /tmp/platform_inventory.txt   # expect 74
```
- [ ] **Step 3:** Confirm `git filter-repo` is available (install path if not).
```bash
git filter-repo --version || pip install --user git-filter-repo
```
- [ ] **Step 4:** Record the boot-sim baseline milestone for the later parity check.
```bash
cd $M68K
# Build the ROM-boot sim and note the milestone it reaches (DAFB render / retire count).
make tb-fpga-top-rom 2>&1 | tail -20 | tee /tmp/boot_baseline.txt
```
- [ ] **Step 5:** Commit the captured inventory into the spec dir as provenance.
```bash
cp /tmp/platform_inventory.txt $M68K/docs/superpowers/specs/2026-06-02-platform-inventory.txt
cd $M68K && git add docs/superpowers/specs/2026-06-02-platform-inventory.txt
git commit -m "docs: pre-split platform file inventory (74 files)"
```

---

## Phase 1 — Create `macqd700-soc` (history-preserving)

### Task 1.1: filter-repo the platform paths into a new repo
**Files:** new repo at `$SOC`.

- [ ] **Step 1:** Make a fresh clone to filter (filter-repo rewrites history; never run on the working repo).
```bash
git clone $M68K /tmp/soc-filter && cd /tmp/soc-filter
```
- [ ] **Step 2:** Keep ONLY platform paths (CPU `rtl/core`, Musashi, fuzz, CPU tbs/docs are dropped here).
```bash
cd /tmp/soc-filter
git filter-repo --force \
  --path rtl/mac/ --path rtl/sys/ --path rtl/vendor/ --path rtl/mac_top.v \
  --path-glob 'rtl/fpga_top*' \
  --path synth/ --path files/ \
  --path tb/ --path tools/ --path docs/ --path Makefile --path CLAUDE.md
```
- [ ] **Step 3:** Verify all 74 platform RTL files survived.
```bash
cd /tmp/soc-filter
{ ls rtl/fpga_top*.v rtl/fpga_top*.vh rtl/mac_top.v;
  find rtl/mac rtl/sys rtl/vendor -type f; } | sort > /tmp/soc_inventory.txt
diff /tmp/platform_inventory.txt /tmp/soc_inventory.txt && echo "INVENTORY MATCH"
```
Expected: `INVENTORY MATCH` (no diff).
- [ ] **Step 4:** Move into place + reinit as standalone repo origin.
```bash
mv /tmp/soc-filter $SOC
cd $SOC && git log --oneline | wc -l   # history preserved (nonzero, large)
```
- [ ] **Step 5:** Commit a README marking origin.
```bash
cd $SOC
printf '# macqd700-soc\n\nMac Quadra 700 SoC platform, split from m68k-ooo @ %s.\nCPU integrates via the AXI socket (rtl/soc/cpu_socket.vh) as a git submodule at cpu/.\n' "$(cd $M68K && git rev-parse --short pre-soc-split-* 2>/dev/null | head -1)" > README.md
git add README.md && git commit -m "docs: macqd700-soc origin README"
```

### Task 1.2: Prune CPU-only leftovers from the SoC tree
**Files:** delete CPU-only tb/tools/docs that rode along.

- [ ] **Step 1:** Identify CPU-only files that filter-repo kept (Musashi, fuzz, CPU unit tbs, ISA docs).
```bash
cd $SOC
ls tb/models 2>/dev/null; ls tools/fuzz 2>/dev/null
ls tb/tb_alu.cpp tb/tb_rob.cpp tb/tb_rat.cpp tb/tb_iq_int.cpp 2>/dev/null
ls docs/isa_status.md docs/uarch_decisions.md docs/decode_coverage_matrix.md 2>/dev/null
```
- [ ] **Step 2:** Remove them (they remain in `m68k-ooo`; history kept here too).
```bash
cd $SOC
git rm -r --quiet tb/models tools/fuzz 2>/dev/null || true
git rm --quiet tb/tb_alu.cpp tb/tb_rob.cpp tb/tb_rat.cpp tb/tb_iq_int.cpp tb/tb_mul_div.cpp tb/tb_commit.cpp tb/tb_lsu.cpp tb/tb_dcache.cpp tb/tb_icache.cpp tb/tb_bpu.cpp tb/tb_ras.cpp tb/tb_decode.cpp tb/tb_exception.cpp 2>/dev/null || true
git rm --quiet docs/isa_status.md docs/uarch_decisions.md docs/uarch_proposals.md docs/decode_coverage_matrix.md docs/bench_baseline.md docs/fuzz_deep_policy.md 2>/dev/null || true
```
- [ ] **Step 3:** Commit.
```bash
cd $SOC && git commit -q -m "chore: drop CPU-only tb/tools/docs (live in m68k-ooo)" && echo done
```

---

## Phase 2 — Reorganize into machine / board / soc

### Task 2.1: Create the layer directories and move files (history-preserving `git mv`)
**Files:** `$SOC/rtl/{mac,board,soc}/`.

- [ ] **Step 1:** Create `rtl/board/` and move KU5P BSP files (per spec §5).
```bash
cd $SOC && mkdir -p rtl/board
git mv rtl/sys/ddr_ctrl.v rtl/sys/axi_ddr4_mig_bridge.v rtl/sys/sim_mig_backend.v \
       rtl/sys/clk_rst.v rtl/sys/reset_debounce.v \
       rtl/sys/sd_ctrl.v rtl/sys/sd_spi.v rtl/sys/sd_spi_mux.v rtl/sys/sd_jtag_writer.v \
       rtl/sys/vram.v rtl/sys/async_fifo.v rtl/sys/pulse_cdc.v rtl/sys/uart_byte_bridge.v \
       rtl/sys/audio_i2s.v rtl/sys/audio_pwm.v rtl/sys/audio_hdmi_bridge.v \
       rtl/board/
git mv rtl/vendor rtl/board/vendor
git mv rtl/mac/video rtl/board/video_phy   # HDMI PHY (mmcm/vtg/scaler/linebuf/i2c/...)
```
- [ ] **Step 2:** Move SoC integration + fabric to `rtl/soc/`.
```bash
cd $SOC && mkdir -p rtl/soc
git mv rtl/fpga_top.v rtl/mac_top.v rtl/soc/
git mv rtl/fpga_top_boot_master.vh rtl/fpga_top_clocks.vh rtl/fpga_top_cpu.vh \
       rtl/fpga_top_ddr.vh rtl/fpga_top_debug_ctrl.vh rtl/fpga_top_debug_host.vh \
       rtl/fpga_top_debug_vio.vh rtl/fpga_top_dma.vh rtl/fpga_top_peripherals.vh \
       rtl/fpga_top_sd.vh rtl/fpga_top_video.vh rtl/fpga_top_xbar.vh rtl/soc/
git mv rtl/sys/axi_xbar.v rtl/sys/axi_async_bridge.v rtl/sys/axil_async_bridge.v \
       rtl/sys/axi_narrow_to_wide.v rtl/sys/axi_n64_to_wide.v rtl/sys/axi_wide_to_axilite.v \
       rtl/sys/axi_defs.vh rtl/sys/peripheral_bus.v rtl/sys/dma_ctrl.v rtl/sys/boot_fsm.v \
       rtl/sys/if_to_axi.v \
       rtl/soc/
rmdir rtl/sys 2>/dev/null || true
```
- [ ] **Step 3:** `rtl/mac/` now holds only machine logic — verify.
```bash
cd $SOC && ls rtl/mac/
# expect: via1 via2 scsi scc asc rtc adb_* pic16c5x iwm_stub orwell_stub q700_eth_sonic glue irq_agg video.v
```
- [ ] **Step 4:** Fix include paths / `-I` dirs so elaboration still resolves (Verilator `-I` and Vivado read_verilog dirs now point at rtl/mac, rtl/board, rtl/board/vendor, rtl/board/video_phy, rtl/soc). Update the Makefile + synth/vivado.tcl source lists.
```bash
cd $SOC
grep -rln "rtl/sys\|rtl/vendor\|rtl/mac/video" Makefile synth/ | sort -u
# Edit each to the new paths (rtl/board, rtl/board/vendor, rtl/board/video_phy, rtl/soc).
```
- [ ] **Step 5:** Commit the reorg.
```bash
cd $SOC && git add -A && git commit -q -m "refactor: organize into rtl/mac (machine) / rtl/board (KU5P BSP) / rtl/soc (integration)" && echo done
```

### Task 2.2: Document the machine↔board BSP contract
**Files:** Create `$SOC/docs/bsp_contract.md`.

- [ ] **Step 1:** Write `docs/bsp_contract.md` describing the interfaces `rtl/board` exposes and `rtl/mac`+`rtl/soc` consume: AXI-memory slave, framebuffer/scanout (fb-read + timing params), audio sample stream, clk/rst, SD/boot, JTAG-AXI/VIO debug transport. (Content per spec §4.)
- [ ] **Step 2:** Commit.
```bash
cd $SOC && git add docs/bsp_contract.md && git commit -q -m "docs: machine<->board BSP contract" && echo done
```

---

## Phase 3 — CPU socket + stub; SoC builds standalone

### Task 3.1: Define the CPU socket interface
**Files:** Create `$SOC/rtl/soc/cpu_socket.vh`.

- [ ] **Step 1:** Write the socket port macro / parameter header. Concrete contract:
```verilog
// cpu_socket.vh — CPU<->SoC contract. AXI_DW in {128,256}.
// Instruction master (read-only): axi_i_{ar*,r*}
// Data master (read+write):       axi_d_{aw*,w*,b*,ar*,r*}
// IRQ: cpu_ipl[2:0] in, cpu_ipl_ack out.  Control: rst, halt_req, halt_ack.
// Debug (minimal generic): dbg_axi_* (JTAG-AXI window) + dbg_halt_req/ack + dbg_pc_tap[31:0]/dbg_retire.
`define CPU_SOCKET_PARAMS parameter AXI_DW = 128, parameter AXI_AW = 32, parameter AXI_IW = 4
```
List the exact AXI channel signals for both masters at `AXI_DW` width (mirror the existing `daxi_*` channel set, widened; instruction master is read-only).
- [ ] **Step 2:** Lint-parse the header in isolation (a tiny wrapper module that just includes it and declares the ports).
```bash
cd $SOC && verilator --lint-only --cc -Irtl/soc rtl/soc/cpu_socket.vh 2>&1 | tail -5 || true
```
Expected: parses (no syntax error).
- [ ] **Step 3:** Commit.
```bash
cd $SOC && git add rtl/soc/cpu_socket.vh && git commit -q -m "soc: define CPU socket interface (dual AXI 128/256 + IRQ + minimal debug)" && echo done
```

### Task 3.2: Black-box CPU stub
**Files:** Create `$SOC/rtl/soc/cpu_stub.v`.

- [ ] **Step 1:** Write `cpu_stub.v` — a module presenting the socket that drives all masters idle (arvalid/awvalid/wvalid=0, ready inputs ignored), `ipl_ack=0`, `halt_ack=halt_req`. Pure tie-off so `fpga_top` elaborates with no CPU.
- [ ] **Step 2:** Commit.
```bash
cd $SOC && git add rtl/soc/cpu_stub.v && git commit -q -m "soc: black-box CPU stub (idle masters) for standalone build" && echo done
```

### Task 3.3: Retarget `fpga_top_cpu.vh` to the socket
**Files:** Modify `$SOC/rtl/soc/fpga_top_cpu.vh`.

- [ ] **Step 1:** Replace the `m68k_core #(...) u_cpu (...)` instantiation with `cpu_stub` bound to the socket signals. Remove the ~120 CPU-specific `dbg_ila_*`/`dbg_*` wires that fed `debug_ctrl` (those move to the CPU repo); keep only the minimal-debug subset the socket defines.
- [ ] **Step 2:** Where `debug_ctrl`/ILA consumed the removed `dbg_*`, gate or stub those consumers to the socket's minimal-debug subset (so `fpga_top` still elaborates). Track removed-probe consumers in `rtl/soc/fpga_top_debug_*.vh`.
- [ ] **Step 3:** Elaborate `fpga_top` on the stub.
```bash
cd $SOC && make lint-fpga-top 2>&1 | grep -iE "%Error|Verilation Report" | tail -5
```
Expected: 0 `%Error`.
- [ ] **Step 4:** Commit.
```bash
cd $SOC && git add -A && git commit -q -m "soc: bind fpga_top to cpu_socket/cpu_stub; drop CPU-specific debug surface" && echo done
```

### Task 3.4: SoC standalone sanity (platform tbs)
**Files:** none (verification).

- [ ] **Step 1:** Run a CPU-independent platform unit tb to confirm the reorg didn't break peripheral RTL.
```bash
cd $SOC && make tb-via1 2>&1 | tail -5
```
Expected: PASS.
- [ ] **Step 2:** Run one more (SCSI or VRAM).
```bash
cd $SOC && make tb-scsi 2>&1 | tail -5 || make tb-vram 2>&1 | tail -5
```
Expected: PASS.

---

## Phase 4 — CPU-repo side: socket adapter + debug relocation

### Task 4.1: `m68k_axi_wrapper` presenting the socket
**Files (in `$M68K`):** Create `rtl/core/m68k_axi_wrapper.v`.

- [ ] **Step 1:** Write a wrapper instantiating `m68k_core` + the existing `if_to_axi.v` (copied/kept CPU-side: `if_*` → AXI instruction master) + `axi_narrow_to_wide` (daxi 32 → `AXI_DW`). Expose the `cpu_socket.vh` port set. Param `AXI_DW` ∈ {128,256}.
- [ ] **Step 2:** Add a Verilator lint target for the wrapper.
```bash
cd $M68K && verilator --lint-only --cc -Irtl/core -Irtl/core/decode --top-module m68k_axi_wrapper \
  rtl/core/m68k_axi_wrapper.v $(find rtl/core -name '*.v') 2>&1 | grep -iE "%Error" | tail -5
```
Expected: 0 `%Error` (debug ports may be `-Wno-PINMISSING`).
- [ ] **Step 2b:** Add a directed tb proving the 32→`AXI_DW` data-master widening preserves byte-lane/store-merge semantics (a store of each byte offset reads back correctly through the widened path). Run it; expect PASS. (Mitigates spec §10 width-adaptation risk.)
- [ ] **Step 3:** Commit.
```bash
cd $M68K && git add rtl/core/m68k_axi_wrapper.v tb/tb_axi_widen.cpp && git commit -q -m "core: m68k_axi_wrapper presenting the dual-AXI CPU socket + widen tb" && echo done
```

### Task 4.2: Relocate rich debug into the wrapper
**Files (in `$M68K`):** keep `debug_ctrl` + ILA probe map wired to `m68k_axi_wrapper`; expose only the socket's minimal-debug subset upward.

- [ ] **Step 1:** Move the CPU-specific `debug_ctrl` consumption (the `dbg_ila_*`/arch-capture/ROB-PRF surface) under `m68k_axi_wrapper`, exposing the socket minimal-debug (JTAG-AXI window + halt + PC/retire tap) at the wrapper boundary.
- [ ] **Step 2:** Lint the wrapper with debug included.
```bash
cd $M68K && verilator --lint-only --cc --top-module m68k_axi_wrapper ... 2>&1 | grep -iE "%Error" | tail
```
Expected: 0 `%Error`.
- [ ] **Step 3:** Confirm CPU regression unaffected (debug is non-functional).
```bash
cd $M68K && make test 2>&1 | tail -3
```
Expected: same PASS count as baseline (no NEW failures vs the pre-split `make test`).
- [ ] **Step 4:** Commit.
```bash
cd $M68K && git add -A && git commit -q -m "core: relocate rich debug_ctrl/ILA under m68k_axi_wrapper" && echo done
```

---

## Phase 5 — Integrate via submodule + parity

### Task 5.1: Mount the CPU as a submodule
**Files (in `$SOC`):** `.gitmodules`, `cpu/`.

- [ ] **Step 1:** Add the m68k repo as a submodule at `cpu/`, pinned to the wrapper commit.
```bash
cd $SOC && git submodule add $M68K cpu && (cd cpu && git checkout split/macqd700-soc) && git add .gitmodules cpu && git commit -q -m "soc: add m68k CPU submodule at cpu/"
```
- [ ] **Step 2:** Add a build flag `CPU=stub|m68k` to the SoC Makefile that selects `cpu_stub` vs `cpu/rtl/core/m68k_axi_wrapper` and adds the submodule's `-I` dirs.
- [ ] **Step 3:** Elaborate `fpga_top` with the real CPU.
```bash
cd $SOC && make lint-fpga-top CPU=m68k 2>&1 | grep -iE "%Error|Verilation Report" | tail
```
Expected: 0 `%Error`.
- [ ] **Step 4:** Commit.
```bash
cd $SOC && git add -A && git commit -q -m "soc: CPU=stub|m68k build select; elaborate with m68k submodule" && echo done
```

### Task 5.2: Boot-sim parity vs baseline
**Files:** none (verification).

- [ ] **Step 1:** Build + run the ROM-boot sim with the m68k submodule.
```bash
cd $SOC && make tb-fpga-top-rom CPU=m68k 2>&1 | tail -20 | tee /tmp/boot_split.txt
```
- [ ] **Step 2:** Diff the milestone against the Phase-0 baseline.
```bash
diff <(grep -oE "retire|DAFB|milestone|PASS|FAIL" /tmp/boot_baseline.txt) \
     <(grep -oE "retire|DAFB|milestone|PASS|FAIL" /tmp/boot_split.txt) && echo "BOOT PARITY OK"
```
Expected: reaches the same milestone (DAFB render) as baseline. If not, debug the seam (width/IRQ/boot-load) before proceeding.

---

## Phase 6 — Clean up `m68k-ooo`

### Task 6.1: Remove moved platform files from the CPU repo
**Files (in `$M68K`):** delete `rtl/mac`, `rtl/sys`, `rtl/vendor`, `rtl/fpga_top*`, `rtl/mac_top.v`, platform `synth/`/tb/tools/docs.

- [ ] **Step 1:** Delete the now-moved platform files (kept in history + in `$SOC`).
```bash
cd $M68K
git rm -r --quiet rtl/mac rtl/sys rtl/vendor rtl/mac_top.v
git rm --quiet rtl/fpga_top*.v rtl/fpga_top*.vh
# Keep if_to_axi/axi_narrow_to_wide only if still referenced by m68k_axi_wrapper; else they moved.
```
- [ ] **Step 2:** Reduce the Makefile to CPU targets (`test`, `fuzz`, `fuzz-deep`, `lint-core`, `decode-check`); drop synth/impl/tb-platform/jtag targets (now in `$SOC`).
- [ ] **Step 3:** Confirm the CPU repo still lints + tests.
```bash
cd $M68K && make lint-core 2>&1 | tail -3 && make test 2>&1 | tail -3
```
Expected: lint clean; `make test` same PASS count as baseline.
- [ ] **Step 4:** Update `CLAUDE.md` — note the platform now lives in `macqd700-soc`; this repo is the CPU + AXI socket adapter. Update the directory map.
- [ ] **Step 5:** Commit.
```bash
cd $M68K && git add -A && git commit -q -m "chore: remove platform (now in macqd700-soc); reduce to CPU + socket adapter" && echo done
```

### Task 6.2: Final cross-repo sanity
- [ ] **Step 1:** `$SOC`: `lint-fpga-top CPU=stub` and `CPU=m68k` both 0-error; one platform tb passes.
- [ ] **Step 2:** `$M68K`: `make test` + `make fuzz N=200` at baseline.
- [ ] **Step 3:** Update memory: platform split landed; record both repo roots + the socket contract location.

---

## Validation / acceptance (from spec §9)
- `macqd700-soc` elaborates `fpga_top` standalone on `cpu_stub` (0 errors).
- With m68k submodule: ROM-boot sim reaches the pre-split milestone (DAFB render); platform tb subset passes.
- `m68k-ooo` `make test`/`fuzz` unchanged.
- Inventory diff (Task 1.1 Step 3) showed every platform file accounted for.

## Notes
- The SP-drift wedge + BCHG-CCR regression are CPU-side; they ride along in
  `m68k-ooo` unaffected. The ILA debug scaffolding on `fix/stale-cdb-bypass-sp-drift`
  should be rebased/landed onto the CPU repo's debug relocation as a follow-up.
- `git push` is never invoked (local-only project norm); the user pushes if/when desired.
