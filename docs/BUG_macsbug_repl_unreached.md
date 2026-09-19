# BUG: MacsBug REPL not reached on Q700 ROM boot — D0/D7 bit 17 gates SCC I/O

**Status:** OPEN.  Infrastructure in place to deliver bytes to the SCC RX
FIFO, but the Q700 ROM's MacsBug / monitor REPL is gated on `D0[17]` /
`D7[17]` which never get set on our sim path.  See "Next steps" below.

## Symptom

Triggering MacsBug (NMI press = board `btn[1]`) does **not** result in
any host-visible serial output beyond the lone 0x80 byte that the
ROM's pre-NMI SCC self-test emits.  The Q700 ROM is supposed to print
a banner (e.g. `MacsBug version ...\r\n>`) on the SCC chan-A TX line in
response to the NMI.  We get one TX byte from boot-time SCC self-test,
nothing after that.

End-to-end test:

```bash
make tb-scc-uart-loopback                                       \
    SCC_LOOPBACK_PATCH=mame-fastdiag,chime-skip                  \
    SCC_LOOPBACK_MAX_INSTS=600000                                \
    SCC_LOOPBACK_TIMEOUT=20000000                                \
    SCC_RX_INJECT='G\\r' SCC_RX_INJECT_FAST=1                    \
    SCC_TX_MAX=64
```

Result: `cpu_tx_bytes=1, uart_tx_bytes=1` (the 0x80 self-test byte from
seq 8237, before MacsBug is even reached).  No additional TX bytes
after the NMI fires.

## Investigation (chain stages, in order)

The chain is:

```
NMI press → CPU vec 31 → MacsBug entry handler → REPL banner print
                                                  ↓
                                          SCC TX FIFO writes
                                                  ↓
                                       SCC TX serialiser
                                                  ↓
                                      uart_byte_bridge
                                                  ↓
                                     uart_rtl_0_txd pin
```

Stage-by-stage findings during this investigation:

**Stage 1 — NMI / programmer's switch fire**: BROKEN before this PR.
The testbench actively suppresses btn[1] at boot to avoid an NMI storm
(see `tb_fpga_top_rom.cpp:1141` — debounce is 2_000_000 cycles in HW
but not gated for SIM_MODEL, so without the suppression, NMI fires on
the first cycle out of soc_full_rst).  No mechanism existed to fire a
single deliberate NMI later.  **FIXED in this PR**: added
`+nmi_at_inst=<N>` and `+nmi_at_pc=<hex>` plusargs to the harness.
Confirmed working — at retired=100000+16 the CPU lands inside MacsBug
range PCs (e.g. 0x40846c02, then routes through 0x4084a812 ..
0x4084a82e).

**Stage 2 — SCC RX FIFO injection**: WAS BROKEN.  The legacy
`+scc_rx_inject_fast` path drove `u_scc_board_uart.rx_valid` for one
sys_clk = ¼ pb_clk; the SCC's pb_clk-domain sample window misses
this most of the time.  The bit-serial path (`+scc_rx_inject_fast=0`)
held each bit for 50 sys_clk = 12.5 pb_clk while the bridge's
hard-coded BAUD=115200 expects 434 pb_clk per bit — a 35× baud
mismatch that misframes the start-bit detector.  **FIXED in this PR**:
`+scc_rx_inject_fast=1` now uses a state-machined pulse on the
bridge's `rx_valid` that holds for ~1 pb_clk (4 ticks) and waits for
the SCC's `rxlvl_a` to bump as the handshake.  Confirmed working —
the injected `G\r` lands in `rxf_a0..1` and `rxlvl_a=2` survives.

**Stage 3 — MacsBug RE-INITS the SCC after entry**: ENCOUNTERED.
Between the NMI fire and the REPL run-loop, the Q700 ROM's MacsBug
entry path issues `WR9 = 0xc0` (hardware reset, AXI seq 25263 in our
trace), reprograms `WR3=0xc1, WR5=0xea, WR4=0x4c` etc.  Any RX byte
injected before that re-init is wiped.  **FIXED in this PR**: the
testbench now waits for the `WR5=0xea` signature ("init complete")
before queuing the inject.  Confirmed working — bytes survive past
init.

**Stage 4 — D0[17] / D7[17] gate the actual SCC I/O code path**:
**THIS IS WHERE WE'RE STUCK**.  The Q700 ROM's serial-I/O routines
are gated on bit 17 of D0 and D7:

  * `0x4084af9c rom_scc_rx_poll` body: `BTST #17, %D7 / BEQ → return` —
    if D7[17]=0, the routine is a no-op (returns immediately, never
    reads the SCC RX FIFO).
  * `0x4084ae3a rom_monitor_desc`: `BTST #17, %D0 / BEQ → skip` — gates
    the descriptor read.
  * `0x4084ae56 rom_monitor_banner`: `BTST #17, %D0 / BEQ → skip` —
    gates the banner-print routine that writes "Welcome to MacsBug"
    bytes to the SCC TX FIFO.
  * `0x4084ae7a` does `BSET #17, %D7` AFTER printing the banner — so
    D7[17] is the "banner-printed → I/O active" latch.

In our sim, **D0[17] is `0` post-NMI, so the banner-print routine is
short-circuited**.  Without the banner, no bytes are pushed to the
SCC TX FIFO.  Without TX, the host sees nothing.

D0[17] is set conditionally at `0x40803012 BSET #17, %D0`, gated on
the BNE at `0x40803010` which evaluates the result of the JSR at
`0x4080300c → 0x4080477a`.  The 0x4080477a routine is some hardware
feature-detect — it's testing whether the platform claims to have a
monitor / serial-output capability.  Whatever it tests fails on our
sim, so D0[17] never gets set.

This is consistent with what the `[probe]` log shows: PC trace post-NMI
goes through `rom_monitor_*` PCs (rom_monitor_entry, rom_monitor_body,
rom_monitor_prepare, rom_monitor_after_desc, rom_monitor_poll,
rom_monitor_rx_result, rom_monitor_loop) — but each one short-circuits
back via `BTST #17, %D0 / BEQ`.

## Smallest reproducer

```bash
# Build
make build/fpga_top_rom/Vfpga_top

# Run
build/fpga_top_rom/Vfpga_top \
    +rom=files/420dbff3.rom \
    +max_insts=600000 +timeout=20000000 \
    +rom_patch=mame-fastdiag,chime-skip \
    +nmi_at_inst=100000 \
    '+scc_rx_inject=G\r' +scc_rx_inject_fast \
    +scc_tx_log=scc_tx.log

# Expected output (currently observed):
#   cpu_tx_bytes=1, uart_tx_bytes=1   (only the boot-time 0x80 byte)
#
# Desired output:
#   cpu_tx_bytes ≥ 2 (a banner char and/or '>' prompt after NMI fire)
```

To reproduce the deeper issue (RX byte landing in FIFO but never read):

```bash
build/fpga_top_rom/Vfpga_top                                  \
    +rom=files/420dbff3.rom                                    \
    +max_insts=600000 +timeout=20000000                        \
    +rom_patch=mame-fastdiag,chime-skip                        \
    +nmi_at_inst=100000                                        \
    '+scc_rx_inject=G\r' +scc_rx_inject_fast 2>&1              \
  | grep -E '\[scc-rx\]|\[nmi\]'
```

Look for `[scc-rx] fast-inject byte[0] consumed (post-lvl=1)` etc. —
the bytes ARE in the SCC chan-A FIFO.  Then the AXI lockstep log
(`+axi_lockstep_log=axi.log`) shows the CPU never reads `0x50f0c026`
(chan-A data port) after seq 25286.  It's stuck reading RR0
(`0x50f0c022`) → `0x44` which has bit 0 = 0 (RX-empty masked), but
that's because the rom_scc_rx_poll body short-circuits before the
RR0 / RR8 path runs.

## Next step (next fix point)

### Update 2026-05-03 — investigation refined

A second-pass investigation (agent task `m68k-fpga-halt-bisect`,
`tb/models/rom_patch_sets.h::add_rom_macsbug_feature_bit17_patches`)
found that the original BUG analysis was **incomplete**:

* The dispatcher at `0x40802fde..0x40803012` (with the `BSET #17,%D0`
  at `0x40803012`) is **never executed** in any sim run.  That whole
  block is the "machine UNKNOWN" path — taken only when the matched
  descriptor's feature bitmap at `*A1+24` is zero (`beqs 0x40802f98`
  at `0x40802f42`).  The Q700 descriptor at `0x4080390c` has a
  non-zero feature bitmap at `+0x18`, so the flow goes
  `0x40802f3e → 0x40802f48 → 0x40803dba → 0x40803050 → return`,
  bypassing the dispatcher entirely.

* So `D0` at MacsBug entry is just whatever `*(A1+24)` had — for
  Q700 that's `0x05a0_183f`.  Bit 17 is **clear** in the canonical
  Q700 ROM image (file offset `0x3925 = 0xa0`, would need `0xa2`).

* Setting bit 17 in the Q700 descriptor (file off `0x3925: 0xa0 → 0xa2`,
  patch `macsbug-feature-bit17`) DOES drive D0[17]=1 at MacsBug entry.
  Verified by sim: with the patch the CPU reaches
  `pc=0x4084afa4 (rom_scc_rx_*)` at `retired=50000` (vs `294814`
  without the patch — the long boot SCC self-test poll dominates the
  delta).  The SCC chan-A RX FIFO injection chain works: 'G\r'
  arrives, bytes are consumed, `rxlvl_a` advances.

* **BUT** bit 17 also routes the SCC self-test and banner-print A3
  base from `*(A0+0x0c) = 0x50f0_c020` (chan B, 6-byte stride to
  chan-A control/data via A3+2/+6) to `*(A0+0x44) = 0x50f1_e020`,
  which on the Q700 memory map decodes to **SWIM/IWM** (`io_off &
  ~0xFC0000 = 0x1E020`, in `0x1E000..0x1FFFF` range).  That's the
  floppy controller, not a SCC alias.  So with bit 17 set, banner-
  print writes go to SWIM and never reach the host UART bridge.
  `cpu_tx_bytes` stays at 1 (the boot 0x80 byte) post-patch.

* The bit-17 path is for a **different machine variant** (post-Q700
  with a different SCC controller location) — not for Q700 itself.

So the original BUG note's option (b) ("fix the upstream feature-detect
that gates D0[17]") was based on a misreading.  The real fix must
either:

1. **Force D0[17]+D7[17] high directly** (option (a), still the right
   choice for sim-side bring-up).  The new patch
   `macsbug-feature-bit17` sets the descriptor bit but routes I/O to
   SWIM — keep it as an opt-in diagnostic ("does anything reach the
   peripheral fabric at all?") rather than a default.
2. **Patch the Q700 descriptor's `*(A0+0x44)` to point at a real SCC
   alias**, e.g. `0x50f0_c020`, AND set bit 17.  This routes banner
   I/O to chan A as on real hardware.  The descriptor lives in ROM
   at file offset `0x35ac` (4 bytes); a single 4-byte patch.
3. **Skip the boot SCC self-test** by NOPing the polling DBF at
   `0x408478b4..0x408478ba` (and the corresponding TX-empty poll at
   `0x40847886..0x4084788c`).  This gets us through boot faster but
   doesn't address the real "MacsBug REPL output reaches host" goal.

Recommend (2) as the next step — provides a proper SCC channel for
the bit-17 monitor I/O path and aligns with the canonical "MacsBug
debugs through chan A" expectation.

The opt-in patch `macsbug-feature-bit17` (bit 17 only, no peripheral
re-route) is landed as a building block.  Use:

```bash
make tb-fpga-top-rom FPGA_TOP_ROM_PATCH=mame-fastdiag,chime-skip,macsbug-feature-bit17
```

to reach MacsBug entry at retired=50000 instead of 295000.

### Original next-step list (still valid as fallbacks)

1. **Force D0[17] / D7[17] high after MacsBug entry** in the
   testbench.  Add `+force_monitor_active=1` plusarg that pokes the
   committed arch register state once retired count hits a threshold.
   Single 32-bit poke into PRF + RAT.  Should unblock the banner print
   and the rom_scc_rx_poll body.  Estimated ~2-3 hours including a
   regression test.  This is a SIM-ONLY hack — on real
   HW the monitor self-tests its hardware first, and the bit gets
   set legitimately.

2. **Find and fix the upstream feature-detect that gates D0[17]**.
   The JSR at `0x4080300c → 0x4080477a` is reading something from a
   peripheral or memory location that returns the "wrong" answer in
   sim.  **REFINED ABOVE: this dispatcher is never reached for Q700;
   the descriptor's feature bitmap is read directly.  See above.**

## What this PR does land

In `tb/tb_fpga_top_rom.cpp`:

* `+nmi_at_inst=<N>` and `+nmi_at_pc=<hex>` plusargs that fire one
  programmer's-switch press after the given retired-count or PC.
  Sized matching the `u_btn1_db` debounce poke pattern from boot —
  produces a clean NMI rising edge into irq_agg.
* `+nmi_pulse_cycles=<N>` to override the press width (default 4
  sys_clk).
* `+scc_rx_inject_fast=1` rewritten to use a state-machined pulse
  on the bridge's `rx_valid` line with handshake on `rxlvl_a` —
  100% reliable byte delivery to the SCC chan-A RX FIFO regardless
  of clock-domain timing.
* The post-MacsBug RX inject now waits for the `WR5=0xea` SCC
  re-init signature before injecting (fallback: 1M-sim_time
  timeout) so injected bytes don't get clobbered by MacsBug's own
  hardware reset.

These three pieces are the substrate the next investigation needs.
Without them, you can't even tell whether the SCC end-to-end byte
path works after MacsBug entry — now you can: the bytes land,
they survive, but the CPU never reads them because of (4) above.

## Files touched

* `tb/tb_fpga_top_rom.cpp`  — NMI fire, direct-FIFO inject, post-init gate.
* `docs/BUG_macsbug_repl_unreached.md` (this file).

No RTL changes.  No regression risk — the new plusargs are off by
default; the rewritten `+scc_rx_inject_fast=1` path is gated by the
existing `+scc_rx_inject_fast` flag and only activates after
MacsBug-entry detection.

## Repro / verification commands tried during investigation

| Command | Result |
|---|---|
| `make tb-scc-uart-loopback SCC_LOOPBACK_PATCH=mame-fastdiag,chime-skip` (no NMI) | 1 TX byte (0x80 self-test only) |
| `+nmi_at_inst=100000` (NMI fires, no inject) | NMI taken, MacsBug PCs visited, still 1 TX byte |
| `+nmi_at_inst=100000 +scc_rx_inject='G\r'` (legacy bridge-pulse) | RX bytes never land in FIFO (clock-domain miss) |
| `+nmi_at_inst=100000 +scc_rx_inject='G\r' +scc_rx_inject_fast=1` (new direct path, no init-gate) | RX bytes land but get wiped by MacsBug's own SCC reset |
| `+nmi_at_inst=100000 +scc_rx_inject='G\r' +scc_rx_inject_fast=1` (new direct path, WITH init-gate) | RX bytes land cleanly, persist forever, but CPU never reads them — D7[17]=0 short-circuits rom_scc_rx_poll |

The last row is the current state.  Adding (1) above — the
D0[17]/D7[17] forced-set — is the next thing to try.

## Tooling gaps noticed

* The `[probe] pc[N]=<pc>` ring-buffer dump on stop is great for
  post-mortem PC traces but doesn't include arch state — adding
  `D0/D7/SR` snapshots at each probe-PC hit would have shown the
  D0[17]=0 problem in seconds rather than 30 minutes.  Suggest
  extending `record_pc_history` to capture D0/D7 alongside PC.
* `+axi_lockstep_log` capture caps at `+axi_lockstep_max=N` events;
  when N is too small for a long run, the cap silently truncates the
  log without a warning.  Print the truncation seq + retired count.
* No way to plot `rxlvl_a` / `rxf_a*` / `WR3` / `WR5` over time
  without recompiling with custom prints.  An `+scc_state_dump_every=N`
  plusarg would have shortened this investigation by ~20 minutes.
* The `macsbug_pc_kind()` mapping in `tb_fpga_top_rom.cpp:643` is
  too generous — `0x40849b00..0x4084b000` is labeled "macsbug_range"
  but the Q700 ROM dwells in those PCs during normal boot init.
  Using these as an automatic MacsBug-entry trigger is a false
  positive.  Suggest tightening to `0x4084af9c..0x4084afca` (the
  actual rom_scc_rx_* range) AND requiring D7[17]=1.

## Cross-reference

* `docs/scc_hw_runbook.md` — the HW-side runbook this BUG is the
  sim-side companion to.  It says "The Q700 ROM lands in MacsBug's
  RX poll within ~6M instructions of POR" — that may be true on real
  HW where the feature-detect at 0x4080477a returns whatever it
  needs, but is *not* true in our sim where the feature detect fails
  and D0[17]/D7[17] stay 0.
* `rtl/mac/scc.v:783` (chan-A external RX delivery) — works
  correctly; not the bug.
* `rtl/sys/uart_byte_bridge.v` — works correctly at the parameterised
  baud; not the bug.
* `rtl/fpga_top_peripherals.vh:404-420` (uart_byte_bridge instance)
  + `:828-846` (SCC instance with chan-A wiring) — both correct.
