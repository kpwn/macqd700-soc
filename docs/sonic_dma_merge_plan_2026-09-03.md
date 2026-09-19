# DMA Engine + SONIC Ethernet → cpu040 SoC: Merge Plan

**Target:** `/home/qwertyoruiop/macqd700-soc-worktrees/m68k040ooo-integration`, branch `feat/m68k040ooo-socket-integration`
**Source:** `main` @ `76766ef`
**Written:** 2026-09-03 · **Status:** ready to execute after the owner decisions in §2 are answered

---

## State of the merge

This is not a cross-repo port — it is an in-repo `git merge main`. The "target repo" is a **git worktree of `/home/qwertyoruiop/macqd700-soc/.git`** (`git rev-parse --git-common-dir` → `/home/qwertyoruiop/macqd700-soc/.git`), merge-base `c4fd97f`, main 65 commits ahead of it, the integration branch 156. `git merge-tree --write-tree HEAD main` produces **exactly six conflicts** — `Makefile`, `cpu` (submodule), `rtl/soc/fpga_top.v`, `rtl/soc/fpga_top_peripherals.vh`, `synth/vivado.tcl`, `tb/tests/host/test_jtag_repl_helpers.tcl` — every one of which is a small union or a one-line pick, and I have read all six hunks. That single fact deletes most of the work three of the five investigations planned: the `.sv` source glob, `check_synth_sources.py`'s `.sv` blindness, the Verilator Xilinx primitive stubs, the entire `synth/vivado.tcl` ethernet block, `tools/taxi_filelist.py`, `synth/ethernet_rgmii{,_post}.{xdc,tcl}`, `synth/eth_debug_cdc.tcl`, the `fpga_top.v` PHY ports, `fpga_top_ethernet.vh`, and **every one of the twelve DMA/SONIC `tb-*` targets** all arrive automatically as clean auto-merges. What the merge does *not* do for free is three things: it silently drops two Makefile lines onto deleted files if you resolve toward main (`vram_cpu_byteswap.v`, `tb-dma-integration`); it makes `dma_engine` a **live 128-bit AXI master on xbar M3 in every build, including `ETH_ENABLE=0`**, which nobody's prior scoping said out loud; and it lands a **CPU-visible SONIC register-map change** (register N moves from byte offset 2N to 4N) into a tree whose boot campaign is mid-investigation. The real remaining risk is not mechanical. It is (a) an unsettled L1-coherency question that gates the *packet engine*, not the merge, and (b) ~4.4k LUT of ethernet dropped onto a design sitting at 82.28% LUT with a documented 33-LUT-flips-route_design precedent.

---

## 1. Corrections to prior analysis — what I overrode and why

| Claim | Source | Verdict |
|---|---|---|
| "TWO REPOS", merge = cross-repo file copy | task framing; areas *dma-engine-integration*, *ethernet-plumbing*, *soc-divergence* | **Overridden.** Verified worktree; `git rev-parse --git-common-dir` → `macqd700-soc/.git`. Their per-file copy prescriptions are superseded wholesale by `git merge main`; their *substantive* findings are retained below. |
| "Makefile globs only `*.v` → BUILD_BREAKS" | *dma-engine-integration*, *soc-divergence* | **True of HEAD today, moot under the merge.** Merged `Makefile:2327` already reads `find $(RTL_DIR) \( -name '*.v' -o -name '*.sv' \)`; merged `:2163` fixes `lint MODULE=` the same way. No prep commit needed. |
| "`tools/check_synth_sources.py` is `.v`-blind → must be fixed" | *dma-engine-integration* verifier, *soc-divergence* | **Real regression, fixed by the merge.** Merged copy carries `\.(?:v|sv)\b` and both rglobs. `ALLOWLIST` has no stale `vram_cpu_byteswap.v` entry — verified. |
| "Restore MMCME4_BASE / IDELAYCTRL / IDELAYE3 to `verilator_xilinx_stubs.v`" | *soc-divergence* | **No action.** Merged stubs already carry them at `:197`, `:223`, `:229` (main-side addition). |
| "Resolve `FRAMEBUFFER_PIXEL_RTL` by taking main's three added lines" | *sonic-rtl-delta* | **Overridden — this breaks `make tb-framebuffer-pixel`.** `rtl/soc/vram_cpu_byteswap.v` is absent from the merged tree (HEAD deleted it, main never touched it). Merged `tb/tb_framebuffer_pixel.v` has no `u_cpu_byteswap` instance — only comments at `:84` and `:487` saying it "used to". Take **only** `mode_admit.v` + `place_plan.v`. |
| "Resolve `ALL_TBS` by taking main's superset" | *sonic-rtl-delta* | **Overridden — this breaks `make test-all`.** Main's side lists `tb-dma-integration`, whose rule HEAD deleted and main never restored; no `^tb-dma-integration:` rule exists in the merged Makefile. Take main's superset **minus** `tb-dma-integration`. |
| "`dma_irq_w` gains a real consumer"; "reconcile the live `dma_ctrl` stub" | review doc §3c | **Both false**, confirmed independently three times. Merged `fpga_top_dma.vh:615` still says `assign dma_irq_w = 1'b0;`. `dma_ctrl` is instantiated nowhere in either branch; `vhdd_ctrl` owns S2 in both. |
| "§1 fabric work is a hard prerequisite for this merge" | review doc §3a | **Overridden.** `axi_xbar.v` has zero commits on either side since the base and is md5-identical (`da2a8eb7a74ffcd79f52305da7219fba`). Ethernet works on v1 against this exact fabric. Performance, not prerequisite — see §7. |
| "The ICMP-only image is independently landable without DMA" | *ethernet-plumbing* | **Moot under the merge** (the engine lands regardless), and its prescribed tie-off arm was wrong anyway — it assigns `dma_client_req_*` nets that don't exist in HEAD, under `` `default_nettype none `` (`fpga_top.v:146`). |
| Fix (b'): drive cpu040's D-cache flush from PFLUSHA/MOVEC-TC "to restore the condition Ethernet works under" | *l1-coherency-driver* | **Overridden, verifier wins.** v1's trigger is a *page-table-walker* mitigation by its own comment (`cpu/rtl/core/m68k_core_flush.vh:335-344`), not a DMA one; nothing orders an OS PFLUSH between an RX descriptor write and the ISR's read of it. It restores a coincidence, not a mechanism. |
| "Add a second requester into cpu040's `DcachePlugin` maint walker" | *l1-coherency-driver* | **Overridden — already built.** `cpu040/src/main/scala/m68k040/top/FullCoreSynth.scala:472-501` drives `dc.maintCmd` with `scope := 3 // all`, its own quiesce (`:488`), completion (`:489-490`) and a collision assert (`:506-507`), fed from `DebugCtrlPlugin` at `:595` under `debugStage >= 3`. Zero `DcachePlugin` lines need to change. |
| "The driver does no cache maintenance ⟹ the hazard is live and merge-blocking" | *l1-coherency-driver* | **Downgraded to UNSETTLED.** The DRVR-body audit is genuinely strong (I accept it), but the ROM-resident `_MemoryDispatch` sel-4/sel-5 handlers the driver *calls* were never audited, and that is exactly where Apple's 68040 DMA-buffer handling would live. |
| Line anchors: SONIC instance `:668`, `via2_pa_in` `:941`, boot-throttle block `:621-670` | task brief; *ethernet-plumbing*; *soc-divergence* | **All wrong.** Verified: instance `:617`, `.sonic_irq` `:637`, `via2_pa_in` `:890`, `sonic_irq_pb` decl `:569`. `cpu_boot_throttle` **does not exist on HEAD at all** — commit `58726ee` removed the plumbing. Do not go looking for it. |

---

## 2. Owner decisions — answer these before Stage 1

| # | Decision | Options | Recommendation |
|---|---|---|---|
| **D1** | `cpu` submodule pointer | base `3b02130`; HEAD `80df95bb`; main `441946e6`. `git merge-tree` refuses to guess. | **MERGE THEM — do NOT pick either side.** See the correction immediately below this table; the original "take main's" recommendation would have silently destroyed the entire boot-fix campaign. |
| **D2** | Full `git merge main`, or scoped cherry-pick of the ~30 eth/DMA commits? | Full merge: 6 small conflicts, but also lands an 8-commit video-pipeline rewrite and net-vHDD, untested against cpu040. Scoped: no unrelated surface, but the eth commits interleave with video commits in `Makefile`, `vivado.tcl`, `fpga_top_peripherals.vh`. | **Full merge.** It is dramatically the cheaper mechanical path and the untested surface is bounded: net-vHDD is fully behind `` `ifdef ENABLE_NET_VHDD `` (merged periph.vh `2352-2481`), which is defined nowhere but one `lint-configs` arm. The video rewrite is the real exposure — see §8. |
| **D3** | Is `ETH_ENABLE=1` (the physical link) in scope? | (a) Register block + engine only, `ETH_ENABLE=0`. (b) + ICMP responder image. (c) Full SONIC NIC. | **Stage it: (a) → (b) → (c).** (a) is the merge itself. (b) proves pins/MMCM/IDELAY/Taxi/CDC with zero memory traffic. (c) is the only one gated on D4. |
| **D4** | **L1 coherency (the big one).** cpu040 has a copyback L1D, no snoop port anywhere in `cpu_socket.vh`/`fpga_top_cpu.vh`, and the SONIC driver's DRVR bodies contain zero `CPUSH`/`CINV`/`MOVEC`. | (i) Audit the ROM's `_MemoryDispatch` sel-4/sel-5 handlers first (cheap, static, decisive, nobody has done it). (ii) Run the decisive v1 experiment. (iii) Map the rings non-cacheable. (iv) SoC-side flush via the existing debug maint port. (v) Ship and see. | **(i) first, unconditionally** — it is hours of static work with the same method already used successfully on the driver bodies, and it may close the question outright. (iii) is **not implementable as written**: the rings are a runtime `_NewPtrSysClear` System-heap allocation with no fixed base, and cpu040 derives cacheability only from MMU CM bits. If (i) shows the hazard is live, (iv) is the cheapest real fix — the requester already exists. **Do not adopt (b')**. This decision gates **Stage 5 only**; Stages 1–4 are safe without it. |
| **D5** | SONIC register spacing 2N → 4N lands in the default build and changes what the ROM's Ethernet probe sees, mid-boot-campaign (`0x4084a840` monitor loop / `scsi.v c96_phase_bits()`). | Land now; or hold the merge until the boot investigation clears. | **Land now, but gate it explicitly** with `tb-cold-boot` + `tb-fpga-top-rom` + `mame-platform-lockstep` **before** the boot team next takes a reading, and tell them it moved. Main's mapping is the correct one (16-bit register on D15..D0 of a four-byte CPU slot — `tb/tb_q700_eth_sonic.cpp:108-119` encodes the contract directly); the target's `sonic_addr[6:1]` is the latent bug. |
| **D6** | `VIDEO_SMOKE` after the merge | HEAD 0, main 1 — a real conflict hunk (`fpga_top.v:210-220`). | **Keep 0.** Main deliberately set 1 and the reason is not in the diff; one line of confirmation from the video owner, not an assumption. |
| **D7** | `/home/qwertyoruiop/rk5-eth` — an **unversioned** external dependency (`git log` there: not a git repository) that every `ETH_ENABLE` bitstream elaborates. Merged default `ETH_RK5_DIR ?= $(PROJ_ROOT)/../rk5-eth` resolves to `.../macqd700-soc-worktrees/rk5-eth`, **which does not exist**. | Vendor it; submodule it; hardcode absolute; require the env var. | **Submodule or vendor before any `ETH_ENABLE=1` bitstream ships.** Until then pass `ETH_RK5_DIR=/home/qwertyoruiop/rk5-eth` explicitly — note `lint-eth-link` **silently SKIPs** on a missing dir (merged `Makefile:2439-2440`), so a wrong path reads as a pass. |
| **D8** | ~4.4k LUT of NIC onto 82.28% LUT (`build/vivado_genfix_full/reports/utilization_synth.rpt:35`, 178,519/216,960). | Land now; or land after a LUT-reduction pass. | **Land Stages 1–2 now** (engine alone is 2,289 LUT / 710 FF / 7 RAMB36 / 1 RAMB18, measured at v1 `utilization_route.rpt:168`), and treat Stage 5 as impl-gated. **Keep `ETH_DEBUG_ENABLE=0` permanently** — same precedent as the de-instantiated `scsi_trace_ring` (merged periph.vh `2021-2022`: at ~83% LUT / congestion level 6, a measured **33-LUT** delta flipped `route_design` from clean to 346 residual overlaps). |
| **D9** | Main also brings SCSI write-protect (`vhdd_ctrl` CTRL[2] + `scsi.v .wprot`) and the `SCSI_MAX_LBAS` 2 GiB−32 KiB fix, entangled in the same files. | Take; reject; split out. | **Take both** (they auto-merge, and the capacity fix is a hardware-confirmed TattleTech bug), but call them out in the merge commit message so they aren't silently inherited. |

> ### CORRECTION to D1 (2026-09-03, verified before execution)
>
> **The plan's original D1 recommendation — "take main's `441946e`" — is wrong
> and would have destroyed work.** It was reached without the commits being
> present locally: `441946e` did not exist in the `cpu` clone at all, which is
> the actual reason `git merge-tree` reported "commits not present". Any
> `git log` range against it silently returned nothing, which reads as
> "main's pointer adds nothing" when the truth is "git could not see it".
>
> After `git fetch` in `cpu/`, the two pointers are **genuinely divergent**
> from base `3b021305`, and each carries work the other does not:
>
> | pointer | commits over base | what they are |
> |---|---|---|
> | HEAD `80df95bb` | **9** | the entire ROM-patch campaign — `calibration-fix`, `zonewalk-empty-table-fix`, `io-oob-alias-probe-fix`, `via-alias-corruption-fix`, `machine-descriptor-slot4-fix`, `bsrw-collision-timing-shim` (+ far variants), `scsi-open-delay-scale-fix`, `diagnostic-loop-skip` |
> | main `441946e6` | **12** | real v1-core work — dyadic FPU (`FADD/FSUB/FMUL/FDIV/FCMP`, `FMOVE.D`, `.X`/`.D` memory sources), `DIVL` decode, `iq_fp` wakeup/select split with speculative wakeup, unary-mem indexed length fix, SD write-protect panel controls |
>
> Taking main's drops all nine ROM patches — the hardware-confirmed boot fixes
> this branch's whole boot campaign rests on, and the table this repo symlinks
> as `tb/models/rom_patch_sets.h`. Taking HEAD's drops main's FPU and decode work.
>
> **Correct resolution: merge the two branches inside the `cpu` submodule and
> point the gitlink at the merge.** Verified mergeable —
> `git merge-tree --write-tree 80df95bb 441946e6` produces a **clean tree**
> (`5d2fd132edffd4873c1da5735c76bd66c08edc4d`), zero conflicts.
>
> Because `cpu` is on the shared `feat/calibration-fix-rom-patch` branch that
> the boot campaign is actively working on, this merge should be made
> deliberately by its owner rather than as a side effect of the SoC merge.
>
> **General lesson, worth carrying:** a submodule pointer comparison is
> meaningless until the objects are actually fetched. `git log A..B` on an
> absent `B` fails open — it looks like "no difference".

---

## 3. The merge, mechanically

Merged tree: `cb452816cf9b92964cb6227f16392b99fc1904e9` (reproducible via `git merge-tree --write-tree HEAD main`; a prior run on a different git recorded `fdd5c38` — the conflict *set* is identical either way).

### The six conflicts and their resolutions

| File | Hunk | Resolution |
|---|---|---|
| `cpu` (submodule) | `3b02130` / `80df95b` / `441946e` | **D1.** `git merge-tree` cannot resolve submodules; record explicitly. |
| `rtl/soc/fpga_top.v` | merged `:210-220`. HEAD `VIDEO_SMOKE = 0`; main `VIDEO_SMOKE = 1` **plus** a 3-line comment and `parameter ETH_ICMP_RESPONDER = 1,`. Hunk contains nothing else. | **Union:** keep `VIDEO_SMOKE = 0`, add main's comment + `ETH_ICMP_RESPONDER`. Severity note: `ETH_ICMP_RESPONDER`'s live consumers (`fpga_top_dma.vh:287`, `fpga_top_peripherals.vh` `.PACKET_ENGINE(...)`, `fpga_top_ethernet.vh:88`) are **all** inside `` `ifdef ETH_ENABLE ``, so omitting it breaks `ETH_ENABLE=1` builds only, not the default. Main's PHY port block (`:272` onward) and the `` `include "fpga_top_ethernet.vh" `` auto-merge cleanly. |
| `rtl/soc/fpga_top_peripherals.vh` | merged `:648`. **Single trivial hunk**: HEAD adds a `═══` comment separator above `q700_eth_sonic u_q700_eth_sonic (`; main replaces the bare instantiation with a `#(.PACKET_ENGINE(...))` header. Zero semantic overlap. | **Keep both**: HEAD's separator line, then main's parameterized header. The entire ~90-port list below it already auto-merged to main's 16-bit-bus version, as did `pb_sonic_addr[5:0]` / `wdata[15:0]` / `wstrb[1:0]` / `rdata[15:0]`. |
| `synth/vivado.tcl` | Two hunks. (a) merged `:501-507`, `read_all_rtl`'s `global` line: HEAD `... cpu_m68k040 cpu_dir cpu_m68k040_v`, main `... cpu_dir eth_enable eth_rk5_dir`. (b) merged `:2364-2377`, buildinfo. | **Union both.** (a) → `global use_sim_model cpu_m68k cpu_m68k040 cpu_dir cpu_m68k040_v eth_enable eth_rk5_dir`. **Taking either side alone is a build break** — main's drops the cpu040 read path; HEAD's throws `can't read "eth_enable"` inside `read_all_rtl`. (b) → keep HEAD's `l2c_enable_effective` / `vram_in_ddr_effective` / `cpu=$cpu_sel`, append main's three `eth_*` lines. |
| `Makefile` | Three hunks, detailed below. | See table. |
| `tb/tests/host/test_jtag_repl_helpers.tcl` | merged `:61-71`. HEAD adds `bp_halt_latched`; main adds `video_reject_reason_name`. | **Union** the helper-name list. |

### The three Makefile hunks — two are traps

| Merged line | Hunk | Correct resolution |
|---|---|---|
| `1534` | `FRAMEBUFFER_PIXEL_RTL`. **HEAD's side is empty**; main's adds `vram_cpu_byteswap.v` + `mode_admit.v` + `place_plan.v`. | **Take `mode_admit.v` and `place_plan.v` only.** `rtl/soc/vram_cpu_byteswap.v` is not in the merged tree (verified `git ls-tree`), and the merged tb no longer instantiates it. Taking main wholesale → `No rule to make target rtl/soc/vram_cpu_byteswap.v`. |
| `2341` | `lint-fpga-top`. HEAD adds the cpu040 `M68kSocketTop.v` regeneration prerequisite `$(if $(filter m68k040,$(CPU)),$(CPU_M68K040_V))`; main adds `$(VERILATOR_EXTRA_DEFINES)` to the recipe. | **Union.** Either side alone breaks a build config. |
| `8062` | `ALL_TBS`. Main adds `tb-dma-integration tb-dma-engine tb-dma-l2c` and `tb-pram-sd-autoload`; HEAD had removed `tb-dma-integration`. | **Main's superset MINUS `tb-dma-integration`.** No such rule survives; `tb/tb_dma_integration.cpp`, `tb_dma_integration_top.v` and `rtl/soc/axi_n64_to_wide.v` are all absent from the merged tree. Taking main wholesale → `make test-all` dies. |

**The trap class, stated once:** HEAD deleted four files since the base (`axi_n64_to_wide.v`, `vram_cpu_byteswap.v`, `tb_dma_integration.cpp`, `tb_dma_integration_top.v`); main touched **none** of them, so git deletes them silently and without a conflict, while main-side *references* to them survive inside conflict hunks. I grepped the whole merged tree: the only live reference is `Makefile:1536`. Everything else is comments (`dma_engine.sv:12`, `fpga_top_dma.vh:11`/`:258`, `fpga_top_video.vh:332`, three tb comments).

### What arrives for free — do not hand-port any of this

Verified present in the merged tree: `rtl/soc/dma_engine.sv`, `rtl/soc/eth_debug_regs.sv`, `rtl/soc/sonic_trace_ring.v`, `rtl/soc/fpga_top_ethernet.vh`, `rtl/board/q700_eth_link.sv`, `rtl/mac/q700_sonic_{tx,rx,cdc,rx_cdc}.sv`, `rtl/board/{net_block_framer,vhdd_net}.sv`, `tb/tb_dma_l2c.sv`, `tools/taxi_filelist.py`, `synth/ethernet_rgmii.xdc`, `synth/ethernet_rgmii_post.tcl`, `synth/eth_debug_cdc.tcl`; the `.sv` glob and `.sv`-aware `lint MODULE=`; the two-extension `check_synth_sources.py`; `read_taxi_filelist` (merged `vivado.tcl:480`), all seven `-sv` reads (`:548-551`, `:672-673`, `:681-682`), the Taxi/ICMP/link block (`:693-696`), `read_xdc ethernet_rgmii.xdc` (`:1053`), the `ETH_*` `parse_bool_env` block (`:209-215`); and **every** DMA/SONIC make target — `tb-dma-engine` (`720`), `-swap` (`736`), `tb-dma-l2c` (`781`), `tb-q700-sonic-rx` (`5301`), `-tx` (`5359`), `tb-q700-eth-sonic` (`5395`), `-engine` (`5427`), `tb-q700-eth-stream-share` (`5201`), `tb-sonic-trace-ring` (`4425`), `tb-eth-debug-regs` (`5414`), `lint-eth-link` (`2438`), `eth-icmp-dry-run` (`3476`).

---

## 4. What the default (`ETH_ENABLE=0`) build actually becomes

Three things change in every build, not just ethernet builds. Say this out loud in the merge commit.

1. **`dma_engine` becomes a live 128-bit AXI master on xbar M3.** Merged `fpga_top_dma.vh` instantiates it at `:243`/`:262` under `` `ifndef ENABLE_DDR_RAMDISK `` **only** — the `` `ifdef ETH_ENABLE `` block starts *after* it, at `:286`. The target's 18 `assign m3_*` tie-offs are gone (HEAD had zero commits on this file; main's version wins outright). Client lanes are constant-zeroed under `ETH_ENABLE=0`, so it can never issue a transaction — but it is elaborated, placed, routed, and it costs 2,289 LUT / 710 FF / 7 RAMB36 / 1 RAMB18. `dma_irq_w` stays `1'b0` (`:615`).
2. **The SONIC register window moves.** Register N: byte offset 2N → 4N. Every SONIC register changes CPU address. Aliasing changes from every 128 bytes to every 256; note main's decode ignores address bit 1, so 4N and 4N+2 alias to the same register — "one register per 4-byte slot, both halves aliased".
3. **The SONIC register block goes from a 322-line probe stub to the real 674-line block** with all seven datasheet fixes, a native 16-bit `peripheral_bus` face, and `PACKET_ENGINE=0`. The stub's fake TX completion (`TCR_PTX`/`ISR_TXDN` on `CR_TXP`) is gone.

**On the SONIC file conflict specifically — there is nothing to lose.** `rtl/mac/q700_eth_sonic.v` on HEAD is **byte-identical to the merge-base blob** and `git log c4fd97f..HEAD -- rtl/mac/q700_eth_sonic.v` is **empty** (independently reproduced). Same for `peripheral_bus.v`, `fpga_top_dma.vh`, `dma_ctrl.v`: zero integration-side commits. The target's SONIC is not a cpu040 variant, it is the frozen ancestor. **No cpu040-side fix can be lost, provably** — and this is why the *file* needs no reconciliation at all, only the one instantiation hunk in `fpga_top_peripherals.vh` above. `fpga_top_peripherals.vh` is the one file both sides touched (HEAD 3 commits, main 7); HEAD's three are `58726ee` (removed the boot-throttle plumbing), `d47c5c4`, `ded744e` (the VIA2 PB6 documentation block) — and git left exactly one conflict between them, which I read in full.

---

## 5. Hard blockers vs correctness vs performance

**BUILD-BREAKING (must fix to compile/run)** — all three are conflict resolutions, not new work:

| Item | Where |
|---|---|
| `FRAMEBUFFER_PIXEL_RTL` byteswap line | merged `Makefile:1534` |
| `ALL_TBS` `tb-dma-integration` | merged `Makefile:8062` |
| `read_all_rtl` `global` line — either side alone breaks | merged `vivado.tcl:501` |
| (`ETH_ENABLE=1` only) missing `ETH_ICMP_RESPONDER`; missing/wrong `ETH_RK5_DIR` | `fpga_top.v:210-220`; `Makefile:2436`, `:2989` |

**CORRECTNESS (compiles, may be wrong):**

| Item | Status |
|---|---|
| **L1 coherency for SONIC descriptor rings and frame buffers** | **UNSETTLED.** Gates Stage 5 only. See D4 and §9. |
| **`BYTE_SWAP32(1)` under cpu040** | The parameter's own rationale comment (`fpga_top_dma.vh:258`) points at a file the target deleted. The cpu040 socket reaches the same SoC-facing convention by a *different* mechanism — a pure wire permutation, socket design decision D1 (`docs/superpowers/specs/2026-08-18-axi-socket-adapter-design.md:49`, §2 `:150-184`). It should be correct. **`tb-dma-engine-swap` cannot prove it** — that test only proves the RTL implements the permutation it was parameterised for, not that `1` is right for this SoC's lanes. A wrong value here is silent descriptor/payload corruption, not a crash. Needs a real round-trip (Stage 2). |
| **SONIC decode 2N→4N** | Correct (main), latent bug (target). But it is a driver-visible change landing under a live boot investigation. D5. |
| **`tb-dma-l2c` against the integration branch's `l2c_ctrl.v`** | `l2c_ctrl.v` has 5 HEAD-only commits main has never seen, including the `pipe_id_haz_c` fetch/LSU ID-namespace fix (`a28d5f7`, now `l2c_ctrl.v:653-654`, matching on the `(id, is_fetch)` pair). DMA IDs are non-fetch and numerically distinct, so no collision is expected — but this pairing has never been run. **Highest-value new coverage in the whole merge.** |
| **`tb_dma_l2c.sv` leaves `l2c`'s `f_axi_*` unconnected** | Merged `l2c.v:140-144` (`f_axi_arid/araddr/arlen/arsize/arburst/arvalid`, `f_axi_rready`) and `:178`/`:193` (`dbg_fetch_snap`, `dbg_fetch_id_snap`) are HEAD-only additions (4 commits); `tb/tb_dma_l2c.sv` is main-only and predates them. The recipe uses `-Wno-fatal`, so it will build with them floating. **Add explicit `.f_axi_arvalid(1'b0)`, zeroed AR fields, `.f_axi_rready(1'b0)` and empty dbg connections** rather than relying on Verilator's default. Small, mandatory. |

**PERFORMANCE-ONLY (explicitly *not* prerequisites):** fabric read-side concurrency (§7); the engine's 8-ID / 16-deep queue machinery being inert against a single-outstanding xbar (`axi_xbar.v:772-773`, `:3208`, `:1628-1632` — unchanged from v1, so not a regression); the engine's missing AXI 4 KB-boundary burst split (harmless here — the xbar's live AW/AR straddle assertions at `:3002-3035` are the real tripwire, and 64-byte max bursts vs MB-aligned decode regions never trip them); ~4.4k LUT of NIC (D8).

**COSMETIC:** `axi_xbar.v` carries **two** stale comments claiming M3 never asserts — `:1006-1008` ("Slot 3 is permanently idle... dead since `mr_arvalid[3]` never asserts", immediately above `assign mr_midx[3] = XBAR_M_DMA;`) and `:1134-1137`. Both are contradicted by `:1059` `assign mr_arvalid[3] = m3_arvalid;`, and both become actively misleading once M3 is live. The code is safe — the real protection is the `is_cpu_master_idx()` filter at `:1163`/`:1167-1168`, and the ROM overlay never touches M3 (`:1222` assigns `mw_awaddr_eff = mw_awaddr` unconditionally; `:1225-1227` gates `apply_cpu_overlay` on `XBAR_M_CPU || XBAR_M_CPUI`). Fix both comments, and fix them in `macqd700-soc` too so the two copies stay md5-identical — that property has diagnostic value.

---

## 6. Stages

Every stage below builds and is gated. Where a stage cannot be split, I say why.

### Stage 0 — Pre-merge (no code change)
- Answer **D1, D2, D6**. Record the `cpu` decision with evidence from `git log` inside the submodule.
- Capture a clean baseline: `make lint-fpga-top CPU=m68k040`, `make lint-configs`, `python3 tools/check_synth_sources.py`, `make tb-all`, `make tb-fpga-top-rom`, `make tb-cold-boot`, `make mame-platform-lockstep`. **Any of these failing before the merge must be recorded**, or the first post-merge failure is unattributable.
- Note the current full-core FMax on `fmax-closure-fanout` (~197–201 MHz) and the 82.28% LUT figure as the deltas to beat.

### Stage 1 — `git merge main`, `ETH_ENABLE=0` — **the minimum viable first merge**
**This is one atomic commit and cannot be staged internally.** It is a single `git merge`; the six conflicts are interdependent (`fpga_top_ethernet.vh` declares `sonic_loopback`/`sonic_rx_drain`/`sonic_*_pb`, whose only drivers are in `fpga_top_dma.vh` and whose only consumers are in `fpga_top_peripherals.vh`, all under `` `default_nettype none ``), and no partial resolution elaborates.

Content: the merge, resolved per §3, at default defines.
Deliverable: corrected SONIC register block; `dma_engine` live on M3 with zero clients; twelve new test targets; the whole ethernet build system in-tree but off.

**Gates, in this order** (cheapest first — `check-synth-sources` catches an `rtl/` file missing from `vivado.tcl` among the ~15 the merge adds, and it is a hard prerequisite of both `synth` and `impl` at merged `Makefile:2929`/`:2936`):

```
python3 tools/check_synth_sources.py      # or: make check-synth-sources
make lint-fpga-top CPU=m68k040            # gates Makefile hunk 2 + the SpinalHDL regen chain
make lint-configs                         # `ifdef asymmetry across every shipping config
make tb-framebuffer-pixel                 # gates Makefile hunk 1
make tb-peripheral-bus                    # the 16-bit native SONIC bus rewrite
make tb-q700-eth-sonic                    # register block, PACKET_ENGINE=0
make tb-q700-eth-sonic-engine             # register block, PACKET_ENGINE=1
make tb-dma-engine                        # 128/256/512-bit
make tb-dma-engine-mut                    # RED gate: drop-last-byte mutant must FAIL
make tb-dma-l2c                           # KEY new coverage: engine vs HEAD's l2c_ctrl.v
make tb-axi-xbar                          # M3 is a live master now
make tb-l2c                               # narrow-beat + gathered-write paths the engine uses
make tb-vhdd-ctrl && make tb-axil-null-slave   # S2 is untouched by this merge; prove it
make tb-fpga-top-rom                      # whole-SoC elaboration + ROM boot with M3 live
make tb-cold-boot                         # regression guard on the SONIC decode change
make mame-platform-lockstep               # SONIC label — the only check of the new device
                                          #   face against MAME's own dp83932c model
make test-all                             # gates Makefile hunk 3
```

**Exit criteria:** all green; `check-synth-sources` reports no new unlisted `rtl/` file; the merge commit message names D5 (decode moved), D9 (SCSI wprot + capacity fix inherited), and the M3 change.

**Caveat on `mame-platform-lockstep`:** it validates the register/address face only. `tb/mame_axi_periph_top.v` instantiates `q700_eth_sonic` with no `PACKET_ENGINE` parameter (defaults 0) and omits the TX/RX cmd/done ports; the build passes `-Wno-UNDRIVEN -Wno-PINCONNECTEMPTY` and `--x-assign fast`, so those inputs are silently 0. Real coverage, narrower than it sounds.

### Stage 2 — Prove the byte-lane convention + the implementation gate
Content: no RTL change if all goes well. A directed test that a descriptor written by cpu040 through the L1D/L2C path is read back byte-identically by `dma_engine` with `BYTE_SWAP32(1)`, and vice versa — the round trip `tb-dma-engine-swap` structurally cannot cover. Plus the standing project rule.

**Gates:**
```
make tb-dma-engine-swap && make tb-dma-engine-swap-mut   # necessary, not sufficient
<directed cpu040 round-trip test>                        # new; see D-note below
full-core OOC synth gate on fmax-closure-fanout          # report FMax vs ~197-201 MHz
make CPU=m68k040 vivado-dry-run
make impl                                                # check utilization_synth.rpt CLB LUT %
                                                          #   vs 82.28%, and congestion/DRC reports
```
**Exit criteria:** FMax and LUT deltas recorded and accepted; byte order proven end-to-end, not inferred. v1's routed build already shows `u_dma_engine`'s own critical path touching `m3_wready → w_beat[6]` (v1 `timing_place.rpt:22003-22005`) — budget for a real timing check, not a formality.

Also land here, as one small commit: the `tb_dma_l2c.sv` `f_axi_*` tie-offs and the two `axi_xbar.v` comment corrections (`:1006-1008`, `:1134-1137`).

### Stage 3 — Coherency decision (D4). **Gate only, no code.**
The one cheap decisive step nobody has taken: audit the ROM-resident `_MemoryDispatch` selector-4 (`LockMemoryContiguous`, called at `0x408FE164`) and selector-5 (`GetPhysical`, called at `0x408FE3D6`) handlers, by the same static method already used successfully on the two DRVR bodies. Corroborating and cheaper still: v1 exports `mmu_dcache_flush_req` to JTAG as `dbg_wedge_state` bit 43 (`cpu/rtl/core/m68k_core.v:509-521`) — poll `wedge-status` during a sustained ping and see whether the OS pflushes during steady-state networking at all.

This stage produces a written answer, not a commit. It blocks Stage 5 and nothing else.

### Stage 4 — `ETH_ENABLE=1 ETH_ICMP_RESPONDER=1` — physical link bring-up
Content: no RTL change; a build configuration plus `ETH_RK5_DIR`. The ICMP responder never touches system memory, so it is safe regardless of D4. Proves the pins, MMCM, IDELAY, Taxi read path, RGMII constraints and the CDC Tcl in isolation. All 12 RGMII pins (K22 L24 L25 K25 K26 K23 M25 L23 L22 L20 K20 M26) are unclaimed by any target XDC, and `fpga_top.xdc`/`fpga_top_real_mig.xdc` are byte-identical between the branches — clean drop, no pin negotiation.

**Gates:**
```
make lint-eth-link ETH_RK5_DIR=/home/qwertyoruiop/rk5-eth   # all four endpoint configs; SKIPs
                                                             #   silently on a bad path — check output
make ETH_ENABLE=1 ETH_ICMP_RESPONDER=1 ETH_RK5_DIR=... vivado-dry-run
make tb-q700-eth-stream-share && make tb-q700-eth-stream-share-mut
make ETH_ENABLE=1 ETH_ICMP_RESPONDER=1 ETH_RK5_DIR=... impl
<hardware: ping the responder>
```
**Watch:** `lint-eth-link` runs with `-Wno-UNDRIVEN -Wno-PINMISSING`, so it will **not** catch a seam left without a driver — the exact failure mode memorialized at `fpga_top_dma.vh:306-314` for `core_done_pint`. Run one extra lint pass with `UNDRIVEN` enabled as a one-off. Also confirm `ethernet_rgmii_post.tcl` and the four Taxi scoped-constraint Tcls actually source post-synth; without them the IDELAYE3/MMCM overrides and the 8 ns `set_max_delay -datapath_only` on the 125 MHz→fabric completion toggle are absent, and you get a build that closes on paper with an unconstrained CDC in it.

### Stage 5 — `ETH_ENABLE=1 ETH_ICMP_RESPONDER=0` — the full SONIC NIC
**Blocked on D4.** This is the first stage where DMA touches memory the CPU reads.

**Gates:**
```
make tb-q700-sonic-rx && make tb-q700-sonic-rx-mut && make tb-q700-sonic-rx-cam-mut
make tb-q700-sonic-tx && make tb-q700-sonic-tx-mut
make tb-q700-eth-sonic-engine
make ETH_ENABLE=1 ETH_ICMP_RESPONDER=0 ETH_DEBUG_ENABLE=0 ETH_RK5_DIR=... impl
<hardware: MacTCP bring-up, bidirectional ping, sustained transfer>
```
**Exit criteria:** sustained bidirectional traffic without RX corruption, over a run long enough to hit the stall regime described in §8 — not a single ping.

### Stage 6 — `ETH_DEBUG_ENABLE=1` (debug-only, deferred)
`eth_debug_regs.sv` and `sonic_trace_ring.v` **already land in-tree at Stage 1**, gated off. This stage is about *enabling* them, and it costs ~621 LUT / ~978 FF / 4 RAMB36 on a design where 33 LUTs once flipped `route_design`. It also re-splits the debug AXI-Lite BAR on bit 19 (`axil_split2 #(.ADDR_W(20), .SEL_BIT(19))`), replacing HEAD's deliberately-simplified straight `dbg_core_*` pass-through, in a subsystem where the integration branch has diverged heavily (`fpga_top_debug_ctrl.vh` and `fpga_top_debug_vio.vh` are both hundreds of lines apart). **Recommendation: keep it at 0 permanently**, matching the `scsi_trace_ring` precedent. Still run `make tb-sonic-trace-ring` and `make tb-eth-debug-regs` in `test-all` so the tree stays honest — exactly as the target already does for `tb-scsi-trace-ring`. Do not land the telemetry page without the matching `tools/jtag_repl.tcl` `sonic-trace`/`ETH_DEBUG_BASE` host commands, or it is inert from the host's point of view.

---

## 7. The performance track — fabric read concurrency (**not** a prerequisite)

This session measured **4.88x** available read-side concurrency being left on the table by the crossbar's strictly single-outstanding per-master slot FSMs (`axi_xbar.v:772-773` `NW=NR=4`; `:3208` every `req_rd_*[mi]` gated on `rs_state[mi]==RS_IDLE`; `:1628-1632` states the write-side equivalent). The engine's 8-ID / 16-deep-queue machinery is therefore inert: at most 1 read + 1 write is ever live.

**Honest framing of the trade-off, for the owner to sequence:**

- It is **not** a correctness prerequisite. `axi_xbar.v` is byte-identical between the branches (zero commits either side since the base), and ethernet works on v1 against this exact fabric today. The review doc's §3a claim that the fabric fix "is a hard prerequisite for this merge" is **overstated** and I am overriding it.
- It **will** make the NIC slower, and — this is the part that matters — it recreates the long DMA-stall windows that produced v1's real RX data-loss bug. v1 first papered over that with `QUEUE_DEPTH=32` (`dc29f38`), then root-caused it properly (`76766ef`: the RX read pipeline free-ran during a DMA stall) and reverted the depth to 16 (`fpga_top_dma.vh:256`). The fix is in the merged RTL. But cpu040 changes the *shape* of stalls — different L2C occupancy, a different D-side traffic profile, five HEAD-only `l2c_ctrl.v` commits — so "the bug is fixed" is a claim about v1's stall distribution, not cpu040's.
- **Recommendation:** do not block Stages 1–4 on it. Schedule it **between Stage 4 and Stage 5**, or accept it as known debt going into Stage 5 with the explicit expectation that Stage 5's sustained-traffic hardware gate is where a residual free-run bug would surface. Either is defensible; silently making it a prerequisite is not.

---

## 8. What could go wrong — grounded in v1's own history

1. **RX data loss under DMA stall, again.** v1 hit this for real: RX BRAM pipeline free-ran while the DMA engine was stalled (`76766ef`), after `dc29f38` had masked it with `QUEUE_DEPTH=32`. The fix is in the merged RTL and `tb-q700-sonic-rx` covers it. **cpu040 makes stalls different, not absent** — bigger L1D, larger speculative window, a different L2C front-door occupancy pattern, and §7's concurrency ceiling still in place. Expect this class to reappear with a different stall distribution, and gate Stage 5 on *sustained* traffic, not a ping.
2. **Silent byte-lane corruption.** `BYTE_SWAP32(1)` is inherited from v1's `axi_narrow_to_wide` convention; cpu040 reaches the same convention by a different route. If it is wrong, descriptors and payload are corrupt with no crash and no assertion — the same failure mode `tb-dma-engine-swap` was written for on v1. Do not skip Stage 2.
3. **Intermittent coherency failures.** If D4 resolves against us, the failure shape is the worst possible for bring-up: v1's 4 KB / 32 B-line L1D loses lines to capacity misses fast; cpu040's 8 KB / 16 B-line copyback L1D holds them longer. That means cpu040 could fail *intermittently* rather than deterministically, and it would present as **exactly the RX-corruption signature of `dc29f38`/`76766ef`** — inviting a re-diagnosis of a bug that is already fixed. Settle D4 before Stage 5, or you will burn a week chasing the wrong ghost.
4. **The video surface.** D2's full-merge path lands 8 video-pipeline commits (`scanout_display.v`, `scanout_placement_sync.v`, `scanout_fetch.v`, `video.v`, `fpga_top_video.vh` — all files the integration branch has never touched — plus new `mode_admit.v`, `place_plan.v`, `compositor.v`, `upscale.v`, `pixel_unpack.v`, `mode_decode.v`). These **auto-merge silently** and land untested against cpu040. `tb-framebuffer-pixel` and the video tbs are the only guard.
5. **Boot-signal confusion.** The SONIC decode change alters what the ROM's Ethernet probe sees, during a boot sequence currently parked at a live investigation. If the boot team takes a reading after this merge without being told, a shifted symptom will be attributed to the wrong cause. This project's own memory has a standing lesson about re-deriving a hung PC that was already documented — apply it in reverse: announce the change.
6. **`ENABLE_DDR_RAMDISK` latent asymmetry.** The entire engine block **and** its `ETH_ENABLE=0` tie-off arm live inside `` `ifndef ENABLE_DDR_RAMDISK `` (merged `fpga_top_dma.vh:225`–`:616`). Defining that macro leaves every `sonic_*_pb` input of `u_q700_eth_sonic` undriven. It is defined nowhere in the target today, so this is latent, not broken — but it comes across verbatim and should be recorded.
7. **The unversioned MAC.** `/home/qwertyoruiop/rk5-eth` is not a git repository, yet every `ETH_ENABLE` bitstream elaborates its Taxi sources and `rtl/icmp_echo_responder.sv`. No pin, no hash, no submodule. A change to that tree silently changes the bitstream. At minimum, record its content hash in buildinfo alongside `eth_icmp_responder`. See D7.

---

## 9. Unmeasured and unsettled — do not paper over these

- **L1 coherency (D4).** What is *proven*: both DRVR bodies (`.ENET` shell at `0x408D8FF0`, len `0x28C`; Eclipse/Spike hardware driver at `0x408FDFF0..0x408FF68A`, len `0x169A`) contain **zero** `0xF4xx` (CPUSH/CINV), zero `0x4E7A/0x4E7B` (MOVEC), and no `_HWPriv` ($A198) — verified two independent ways, twice. What is *proven* structurally: `cpu_socket.vh`/`fpga_top_cpu.vh` contain no snoop, coherency or invalidate port; there is no hardware path from the fabric into cpu040's L1. What is **not** established: that the ROM's `_MemoryDispatch` sel-4/sel-5 handlers do nothing either (never audited); and that the rings are copyback at *runtime* — the ROM's `movec %d0,%tc` with D0=0 at `0x4080406E` turns the MMU **off**, under which cpu040's default is WRITETHROUGH (`DtlbPlugin.scala:287`), and the only copyback evidence is a single MAME-traced ROM **boot-time** page descriptor (`docs/q700_pt_walker_dcache_incoherence.md:30`, `0x003FE039`, CM=01), not a descriptor covering a System-heap allocation at driver runtime. Reasonable inference; not a demonstration. **"Ethernet works on v1" does not transfer** — v1's core fires a full D-cache writeback+invalidate on MOVEC-TC/PFLUSHA (`cpu/rtl/core/m68k_core_flush.vh:358-363`) that cpu040 deliberately does not (`ExceptionUnit.scala:393-396`). That asymmetry is real and proven. Whether it is *why* v1 works is a hypothesis, and the one experiment that would settle it (gate v1's trigger off, rebuild, ping) has not been run.
- **A separate, pre-existing cpu040 bug this uncovered — file it independently.** cpu040's page-table walker has its own AXI master and bypasses the L1D (`DtlbPlugin.scala:70-71`, `TableWalker.scala:26`), exactly as v1's does. `docs/q700_pt_walker_dcache_incoherence.md` documents that bug class producing a hardware vec=2 at `PC=0x4080010E` on v1, and v1 fixed it with the `m68k_core_flush.vh:358` trigger. **cpu040 has the bypass and not the mitigation.** This is independent of the merge, is the correct justification for a v1-parity flush trigger (DMA coherency is not), and should get its own ticket and its own gate.
- **`tb-dma-l2c` against HEAD's `l2c_ctrl.v` has never run.** No collision is expected. Nobody has looked.
- **The engine's ID plumbing** is checked only by non-`SYNTHESIS` `$error` assertions in `dma_engine.sv:212-220` (request length outside 1..64 bytes; any B/R whose ID has bits above `[2:0]` set). Keep them — they are the cheapest available proof the xbar is replaying `rs_mid` (`axi_xbar.v:3504-3507`) rather than an unmodified `XID` (`:3752`).
- **`.rsp_ready(4'hf)`:** completions are presented for exactly one cycle and clients must consume unconditionally. `q700_sonic_tx`/`rx` leave `.dma_rsp_ready()` unconnected and rely on this. Any future client on lanes 2/3 must honour the same contract or responses vanish silently.
- **No FMax or LUT number for the merged design exists yet.** Every figure in this plan is either v1-measured (`utilization_route.rpt:168`, `utilization_synth.rpt`) or target-measured pre-merge (`build/vivado_genfix_full/reports/utilization_synth.rpt:35`). Stage 2 produces the first real one. Do not quote a post-merge FMax until it does.