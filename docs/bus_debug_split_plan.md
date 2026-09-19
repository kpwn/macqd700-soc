# Bus + debug architecture refactor — plan

Owner direction, 2026-09-12: *"the axi should probably be simplified and moved to
the reset domain for soc full reset via vio, and only slaves that should be
available to the cpu sitting on it; the debug infra should move to a different
axi bus that can also master the main bus (using the master currently used by
jtag)"* and *"have a vio reset for the whole design"*.

## Why — the defect this removes

The debug window reaches the CPU through **S1**, the peripheral bus
(`pb_dbg_*` in `fpga_top_peripherals.vh`). S1 is **deliberately excluded** from
the crossbar's flush domain:

```verilog
function is_flush_domain_slv;
    is_flush_domain_slv = (slv == `XBAR_SLV_DMA)  || (slv == `XBAR_SLV_VRAM) ||
                          (slv == `XBAR_SLV_DAFB) || (slv == `XBAR_SLV_SDJTAG);
endfunction
```

so S0 (DDR) and S1 (IO) stay alive across `soc_full_rst`. Consequence, measured
on hardware: a JTAG `reset` **is** a `dbg_wr` to DBG_CONTROL *through S1*. The
reset it causes swallows that write's BRESP, and the S1 write slot has no
release path at all — `ws_flush_abort` cannot fire for S1, leaving only the
watchdog. Instrumentation confirmed it: `xbar_s1_slot_busy` reads **1** after a
wedging reset and **0** while running, with `xbar_slv_poisoned = 0x00` in both
(so the poison theory, and the "freeze the watchdog" fix built on it, were both
wrong — see `a93451a6`).

**The reset strands its own write.** That is a topology problem, not a logic
bug, and `a93451a6` is a stopgap for the current topology.

## Target topology

Owner correction, 2026-09-12: the pre-existing `axi_dbg_split.v` approach is
**rejected**. It splits one JTAG master into two parallel paths, which means
exposing **two AXI-JTAG interfaces**. The design has exactly one `hw_axi` core
today (the REPL reports "single hw_axi core -- debug shares the system master"),
and two would make the debug surface worse, not better.

The shape is **hierarchical, not split**: one JTAG bridge lands on the debug
bus; the debug bus can MASTER the SoC bus.

```
 JTAG-AXI            <-- exactly ONE bridge, one hw_axi core
     |
     v
 DEBUG BUS           <-- its own reset domain; survives soc_full_rst
     |-- debug_ctrl registers (halt/step/breakpoint/exception)
     |-- SD/JTAG writer
     |-- reset + VIO control
     |
     +--[master]--> SoC BUS      <-- ONE reset domain: soc_full_rst (VIO)
                       |-- DDR
                       |-- peripheral bus (Mac I/O)
                       |-- VRAM
                       +-- DAFB
                            ^
              CPU LSU ------+
              CPU IF  ------+
              boot FSM -----+
```

An access from the host is decoded on the debug bus: debug-window addresses are
served locally; everything else is forwarded through the debug->SoC master port.

### Properties this buys

1. **One JTAG interface.** The host keeps a single `hw_axi` core and a single
   address space; nothing about the REPL's addressing changes.
2. **The SoC bus resets as one unit.** No slave sits outside the flush domain,
   so no transaction can be outstanding across a reset. The stranding class
   disappears rather than being patched.
3. **Debug is upstream of the reset, not downstream of it.** Resetting the SoC
   cannot kill the bridge, because the bridge is not behind anything being
   reset. This is the property the current topology lacks and cannot be given
   by any amount of watchdog tuning.
4. **Debug still reaches DDR and peripherals**, as a *master* on the SoC bus.
   Being the master, its in-flight work can be abandoned cleanly on a SoC reset
   -- it is the one issuing, so it can be told.
5. **Only CPU-visible slaves on the SoC bus.** The SD/JTAG writer and the debug
   registers move off it; neither is CPU-addressable in normal operation.

### Simplifications it unlocks

The per-slave sticky poison latches (`slv_w_poisoned`/`slv_r_poisoned`), the
split flush domain, `is_flush_domain_slv`, and much of the watchdog machinery
exist *because* slaves had different reset lifetimes. With one domain most of
that becomes dead weight. Measured evidence that the poison path never fires in
practice: `xbar_slv_poisoned = 0x00` both running and after a wedging reset.

`axi_dbg_split.v` should be **deleted**, not instantiated.

## Work items

| # | item | risk |
|---|---|---|
| 1 | Create the debug bus; move debug_ctrl + SD/JTAG writer onto it; land the single JTAG bridge there | medium — touches `fpga_top_peripherals.vh`, `fpga_top_debug_*.vh` |
| 2 | Add the debug->SoC master port (takes the seat `XBAR_M_XDMA` holds today) | medium |
| 3 | Put the whole main xbar in `soc_full_rst`; delete the flush-domain carve-out | **high** — the fabric everything depends on |
| 4 | Whole-design VIO reset | low |
| 5 | Delete the now-dead poison/watchdog machinery, and `axi_dbg_split.v` | low, but only after 3 lands |

Order matters: 1 and 2 are prerequisites for 3 (debug must be off the main bus
before the main bus can be reset wholesale), and 5 must follow 3.

## Gates

`make lint` (15 configs) after each item; `make tb-axi-xbar` (401 checks) after
3 and 5; a full-core synth gate before any bitstream claim. The crossbar tb
already covers the abandonment paths (scenarios 28/29/30/32/33/34, 42c, 43),
which is the behaviour item 3 changes most.

## Correction to this plan — `axi_dbg_split.v` was NOT the rejected thing

Written 2026-09-12, after reading the history the plan summarised second-hand.

The claim above that `axi_dbg_split.v` "splits one JTAG master into two parallel
paths, which means exposing **two AXI-JTAG interfaces**" is **wrong**, and the
instruction to delete it was wrong with it.

`axi_dbg_split.v` is a 1-slave x 2-master address-decoding interconnect: ONE
JTAG master in, a local debug port and a system master port out. That IS "two
busses, the debug one able to master the SoC one, and just one axi jtag bridge".
`abc43e51` instantiated it with exactly one bridge.

The two-BSCAN topology the owner is remembering came from a LATER commit,
`52166283`, which added a genuinely separate `debug_jtag_axi` IP instance. That
is what `330ad925` reverted, and its reasons (flaky `get_hw_axis` enumeration,
device dropping off the chain) are properties of the second BSCAN core, not of
the decode module. The revert message says so itself and explicitly kept the
module "in-tree, out of the datapath... the module is sound".

So the module is **reused, not deleted** — renamed to `rtl/soc/axi_dbg_bus.v`
because "split" is what caused the misreading. It carries a hard-won fix
(`8983ff84`: per-BRANCH independence, after a shared busy latch reintroduced the
head-of-line blocking it exists to remove — caught on hardware, p152) and a
14/14 unit tb. Re-deriving that from scratch would have been a way to rediscover
the same bug.

## Status

**Item 1 + 2 — LANDED.** The debug bus exists and masters the SoC bus.

* `rtl/soc/axi_dbg_bus.v` (was `axi_dbg_split.v`, `git mv`) — one JTAG bridge on
  its slave port, 0x5090_0000 served locally, everything else mastered onto the
  SoC crossbar through the seat `XBAR_M_XDMA` already held. `core_rst`, never
  `soc_full_rst`.
* The 0x5090_0000 window no longer reaches the CPU debug CSRs through S1 ->
  `peripheral_bus` -> `axil_async_bridge`. That whole core->pb->core detour is
  deleted for JTAG hosts; both ends were already on `core_clk`.
* `peripheral_bus`'s `dbg_*` window answers OKAY+0 (it is also that module's
  default decode fallback, so it must answer something; OKAY+0 is the same
  open-bus semantic `SLOT_FAULT` already uses).
* The PCIe/XDMA and no-host arms keep the old path verbatim, so no
  configuration regresses.
* **The SD/JTAG writer (S5) did NOT move**, though item 1 lists it. Moving it
  off the SoC bus means removing a slave from `axi_xbar.v` — whose slave-side
  logic is hand-written per-slave branches, not arrays indexed by `N_SLAVES`
  (the module's own header says those parameters are "documentation-only"), and
  whose tb covers the S5 routing. That is item-3-class surgery for an
  architectural tidy, with no measured defect behind it: S5's bridge already
  resets on `soc_full_rst_bank[4]` and is already in the flush domain, so it
  carries none of the stranding risk S1 did. Left for whoever does the S0
  quarantine, since they will be in that file anyway.

  Gates: `make lint` all configs; `make tb-axi-dbg-bus` 18/18; `make tb-axi-xbar`
  401/0.

  **One hole this opened, and closed (`62b272d8`).** The debug bus's local
  slaves are AXI-Lite: one AW, one W, one B. Hand them a burst and they take the
  AW and then see W beats they have no AW for. While the window was reached
  through the crossbar, a burst was handled *there* instead — mangled for short
  ones (`axi_narrow_to_wide` packs <= 4 narrow beats into one wide beat, which
  S1 accepts and writes one register from), SLVERR for longer ones
  (`is_lite_only_slv()` includes `XBAR_SLV_IO`). Either way it got a response.
  A burst in the window is now routed down the *system* branch, restoring that
  exactly — one term on each decode, no new state, no new response path. Four
  new tb checks, asserting both directions so neither half passes vacuously.

**Lint hole found and closed.** No `lint-configs` row elaborated the combination
every board bitstream actually ships — `JTAG_AXI_ENABLE` **with**
`DISABLE_SD_JTAG_WRITER`. `ila` has the first without the second; `nosdjtag*`
have the second without a host master. Since all of this work lives inside
`ifdef JTAG_AXI_ENABLE`, that hole had to be closed before touching it. New
`ship` row; core config count 11 -> 12.

**Item 3 — LANDED IN PART, and the plan was over-optimistic about the rest.**

What landed: **S1 is in the SoC reset domain and in the crossbar's flush
domain.** The S1 CDC moves to `soc_full_rst_bank[1]` / `pb_soc_full_rst_bank[3]`,
`peripheral_bus` to `pb_soc_full_rst_bank[3]`, and `is_flush_domain_slv()` gains
`XBAR_SLV_IO`. So an in-flight S1 transaction at reset is flush-aborted with a
clean local SLVERR, exactly the way S2/S4/S5 already were, instead of being
abandoned with no release path.

This is the *general* form of the measured defect, and item 1 only removed the
debug path's contribution to it. The CPU can strand the S1 slot the same way:
`ENABLE_WD = 0` (owner directive, 2026-09-06) means the watchdog is not a
backstop, so "ride out the reset with the watchdog frozen" — the old S1 contract
— has **no release path at all**. `dbg_s1_slot_busy = 1` after a reset is that,
and S1 is then write-dead for every master until `core_rst`.

Note the pb-side reset is `pb_soc_full_rst_bank`, **not** `pb_full_rst_bank`:
the latter also carries `warm_peripheral_reset` (a 68040 `RESET` instruction),
which does not reset the CPU and does not raise `slv_flush`. Resetting the S1
front door under a still-running CPU would create a new stranding class.

### What did NOT land, and why the plan is wrong about it

The plan says item 3 is "put the whole main xbar in `soc_full_rst`". **That
cannot be done as written.** `ddr_ctrl` (and `u_l2c` above it) sit on
`core_rst_bank[4]` deliberately, to skip the ~100 ms DDR re-calibration a
`soc_full_rst` would force. If the crossbar moved to `soc_full_rst_bank[4]`
while they did not, the crossbar would forget an in-flight S0 transaction that
the MIG has **not** forgotten, and the late B/R would arrive at an idle slot.

That is precisely the hazard `S3_BACKEND_SURVIVES_FLUSH` exists to quarantine —
the crossbar already documents it, for S3-into-DDR under `VRAM_IN_DDR` — except
no such quarantine is written for S0. Doing the move without it would trade one
stranding class for a worse one, on the path the machine boots from.

So "the SoC bus resets as one unit" needs **a late-response quarantine for the
S0/MIG boundary first**. That is the real remaining work, and it is a bigger
job than the plan's one-line "high" risk suggests. Everything else — S1, S2, S3,
S4, S5 — is now in the reset domain and in the flush domain; S0 alone is not.

  Gates: `make lint` 12/12 core + 4/4 eth-link; `make tb-axi-xbar` 401/0 (the
  count is unchanged: `scenario_36` was **inverted** from "S1 must NEVER be
  flush-aborted" to "S1 IS flush-aborted", check-for-check, because its stated
  premise — S1's bridge is kept alive across `soc_full_rst` — is exactly what
  this item removed).

### `461df73b` (`a93451a6`) is KEPT, not reverted

The brief asked whether to revert it once debug leaves the SoC bus. **No.** Its
*stated* purpose was wrong — it scoped the abort to `mi == 0`, and the stranded
debug write is on slot 1 (`XBAR_M_XDMA`), so it could not have fixed what its
message claims, which matches the hardware result.

But the code it added is not dead after item 3. It fires on
`m0_abandon || (m0_wsel_q != cpu_held_in_reset)` — and `cpu_held_in_reset` has
sources that **never** raise `slv_flush`: `dbg_cold_reset_hold` (DBG_CONTROL
bit 4, i.e. a plain REPL `reset hold`), `!boot_rom_ready`, and
`!cpu_rst_settle_done`. A slot-0 S1 write stranded by a host-driven
`reset hold`, with no `soc_full_rst` anywhere, is released **only** by that
term. Removing it would reopen a real case in exchange for tidiness.

**Item 4 — LANDED, and it was not where the plan thought.**

The plan rates this "low" risk and treats it as adding something. It was
neither: a VIO reset already existed (`vio_hard_reset`, probe_out1 →
`btn3_resetn_db` → `platform_resetn` → both `clk_rst` instances). What was
missing is one line, and it is the interesting one:

**The DDR4 MIG's `sys_rst` was tied to `~btn[0]` and nothing else.** Not
`cpu_resetn`, not `btn[3]`, not the VIO. The memory controller — the block a
wedged fabric most often ends up waiting on — was the only thing in the design
that could be reset *only* by physically pressing a button on the board. "A VIO
reset for the whole design" was not achievable, and reading the reset tree would
not have shown it, because the MIG instance lives in the `ifndef SIM_MODEL` arm
that `make lint` never elaborates.

Now `soc_hard_rst_req` (`fpga_top_clocks.vh`) is the single named whole-design
reset request — board pin, `btn[3]`, VIO — and it reaches the MIG too. It is
safe in both directions without new sequencing: resetting the MIG drops
`c0_init_calib_complete` → `ddr_cal_done` (a direct assign, not a sticky latch)
→ `platform_init_done` → `clk_rst`'s `rst_req`, so the SoC holds *itself* in
reset until re-calibration finishes; and `core_clk` comes from the fabric
`BUFG_GT` path rather than `mig_ui_clk`, so JTAG and `dbg_hub` stay up through
the re-cal. `fabric_gt_clr` stays out of it, as its own comment requires.

**Second lint hole found and closed: `make lint-realmig`.** Nothing in the repo
elaborated the real-hardware DDR arm — every `lint-configs` row passes
`-DSIM_MODEL`. Only Vivado saw it, ~5 minutes into a 40-minute run. This is the
same trap the Makefile already documents three times over (`VIO_ENABLE`,
`DISABLE_SD_JTAG_WRITER`, `ILA_ENABLE`), and it is why the MIG reset gap could
sit there unseen. The DDR4 stub already existed; the only thing missing was a
`STARTUPE3` stub. Config count: 12 core + 1 realmig + 4 eth-link = **17**.

**Both new lint rows have a negative control** — a green row that elaborates
nothing is exactly the failure mode this project keeps hitting:

| injected error | `default`/`vio`/`l2c`/`nosdjtag`/`scsitrace` | `ila` | `ship` | `realmig` |
|---|---|---|---|---|
| undefined signal inside `ifdef JTAG_AXI_ENABLE` | PASS (blind) | **FAIL** | **FAIL** | PASS (blind) |
| undefined signal inside `ifndef SIM_MODEL` (MIG) | PASS (blind) | PASS (blind) | PASS (blind) | **FAIL** |

Each row fails on the arm it exists to cover, and the pre-existing rows are
confirmed blind to both — which is the point.

**Item 5 — NOT started, and it is blocked on more than the plan thinks.**

The plan gates item 5 on item 3 "landing". The real gate is the S0 quarantine.
With the crossbar still on `core_rst` for S0's sake, `slv_flush` is **not** dead
code — it is the *only* thing that releases an S1/S2/S3/S4/S5 slot across a
reset the crossbar itself does not take. Deleting it would reintroduce the exact
defect this whole campaign removed.

The poison machinery is a different matter: with `ENABLE_WD = 0` both fire terms
are already constant-false, so synthesis strips the counters and the SLVERR arms
today. What remains is the latches and their clear-on-flush. Removing that is
~nothing in area, on a 4000-line load-bearing module, with real regression risk
and tb scenarios (30, 32, 33, 34) that exercise it under an `ENABLE_WD` override.
Not worth doing on its own.

`axi_dbg_split.v` is gone in the sense that matters: it is `axi_dbg_bus.v`, in
the datapath, in `synth/vivado.tcl`, and off `check_synth_sources.py`'s
allowlist. Deleting it outright would have been the wrong call — see the
correction above.

## What is left, honestly

1. **A late-response quarantine at the S0/MIG boundary.** This is the real
   remaining work and the blocker for both the rest of item 3 and for item 5.
   The `S3_BACKEND_SURVIVES_FLUSH` idiom is the model to copy.
2. **Give the PCIe/XDMA host a debug bus of its own.** Its 128-bit master does
   not pass through `axi_dbg_bus`, so it still reaches debug through S1, and it
   now carries a named reset-event mismatch (see the ⚠️ block in
   `fpga_top_peripherals.vh`). Unreachable in any built configuration.
3. **Hardware confirmation.** Nothing here has been on the board. The one
   measurement to take first: `vio_boot_diag` bit31 (`xbar_s1_slot_busy`) after
   a JTAG `reset`. It read 1 on every instrumented build so far; it should read
   0 now, and the debug bridge should survive the reset.
