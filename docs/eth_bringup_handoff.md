# Ethernet bring-up handoff — DMA engine, SONIC, GbE

Executable handoff. Read in this order:

1. `~/rk5-eth/docs/SONIC_HANDOFF.md` — the proven PHY/MAC baseline and the
   MAC↔SONIC AXI-stream contract. **Owns the board-level details; do not
   change them.**
2. `docs/sonic_integration_plan.md` — the SoC-side decisions and why.
3. This document — what to actually do, in order, and how to know each step
   worked.

---

## 0. Where things stand

| piece | state | where |
|---|---|---|
| PHY + MAC + link | **proven on hardware** — 1000BASE-T, WNS +0.715, 200/200 ICMP, 0% loss | `~/rk5-eth` |
| ICMP echo responder | proven; keep as a physical-link regression image | `~/rk5-eth/rtl/icmp_echo_responder.sv` |
| SONIC register file | packet-engine command/completion seam implemented; legacy probe behavior remains the default | `rtl/mac/q700_eth_sonic.v` |
| SONIC TX | descriptor fetch, pipelined payload DMA, Taxi stream, completion-gated writeback implemented | `rtl/mac/q700_sonic_tx.sv` |
| SONIC RX | DMA-backed CAM load, RRA fetch, filtering/capture, pipelined payload DMA, RDA/link update and IRQ implemented | `rtl/mac/q700_sonic_rx.sv` |
| SONIC register tests | legacy and real-engine modes both 4/4 green, including MAME/Q700 physical lane layout | `make tb-q700-eth-sonic tb-q700-eth-sonic-engine` |
| SONIC IRQ | wired — VIA2 port A bit 0, slot $9 | `rtl/soc/fpga_top_peripherals.vh` |
| generic DMA engine | **implemented and on xbar M3** — 4 clients, bidirectional, byte-granular, width-swept | `rtl/soc/dma_engine.sv` |
| MAC in the SoC build | Taxi MAC selectable between proven ICMP responder and SONIC TX endpoint; frame CDC included | `rtl/board/q700_eth_link.sv` |

Both descriptor datapaths and CAM-programmed filtering are implemented and
structurally connected. Hardware driver bring-up is next; RDE/RBE stop and
recovery are also implemented.

The Q700 register mapping follows MAME's `umask32(0x0000ffff)`: each SONIC
register occupies a four-byte slot and is connected only at physical offsets
`register*4+2` (high byte) and `register*4+3` (low byte).  Upper-half accesses
are acknowledged as bus cycles but return `0xff` and do not mutate registers.
This is important for the System 7 driver's LONG writes: the peripheral-bus
serializer emits all four bytes, and only the final connected halfword reaches
the SONIC.

---

## 1. Architecture, decided

**The DMA engine is the single L2C master. SONIC is a consumer of it.**

```
        SONIC packet adapter ──┐
   (future) block/LAN client ──┼──> DMA ENGINE ──> single L2C master port
                        ...  ──┘
```

Consequences that shape everything below:

- **One attachment point**, not one per client — this is what makes the
  crossbar-seat problem tractable (`sonic_integration_plan.md` §2.2).
- **Coherent by construction.** L2 is the single point of coherency and this
  SoC has **no snooping**; CPUSH/CINV never reach L2. Routing through L2C
  removes the entire stale-descriptor / invisible-frame class and any
  dependency on what the real Mac driver does about cache maintenance.
- Clients get `{guest_addr, length, direction, tag}` + a stream. **Never an
  AXI port.** Descriptor *formats* stay client-side policy — SONIC's
  CDP/TXP/RRA/RDA layout is one client's encoding, not something the engine
  learns.

---

## 2. Phase order, with a definition of done for each

### Phase 1 — get the MAC into the SoC build, unchanged

Import the pinned `third_party/taxi` source lists and Taxi's constraint Tcl
into the SoC flow. **Do not copy a prebuilt bitstream** — the MAC must be
synthesised alongside the real logic so the CDC and timing are analysed
together. Keep the ICMP responder selectable by parameter or build target.

**Done when:** the SoC bitstream contains the MAC + responder, and the same
`ping -c 200` test from the rk5-eth handoff passes against the *SoC* image.
That proves the physical path survived the move before any new logic exists.

Expect this to be the fiddliest phase (SystemVerilog sources, a third clock
domain at 125 MHz, extra constraint files) and the least interesting. Do it
first anyway — everything after it is debuggable only if this is solid.

### Phase 2 — the DMA engine, standalone

Build and verify it with **no SONIC involved**: a synthetic client driving
`{addr, len, dir, tag}` against L2C in sim.

Non-negotiables:

- **Per-transaction AXI IDs.** `l2c_ctrl`'s Critical-3 gate blocks a second
  live op per ID: measured **55.40 vs 8.38 cyc/op**, a 6.6× cliff, *worse than
  single-outstanding*. 1 GbE needs 125 MB/s; unique IDs give 191 MB/s,
  constant ID gives 29. **Every existing master on this SoC uses a constant ID,
  so there is no in-tree example to copy.**
- **Parameterizable 128/256/512-bit AXI datapath.** The SoC instance is 128-bit
  to match L2C directly; wider targets use the same engine without a narrow
  internal datapath or a peripheral-specific widening shim.
- **Byte-granular 1..64-byte transactions.** Requests may be shorter than a
  cache line and may cross an AXI-beat boundary. Aligned 64-byte operations are
  the fast path because they exactly match one L2C line.
- **64 B-aligned full-line writes** wherever possible: L2C's gather path
  (`00f079c`) installs a line with **zero fill beats**. `stream_write_64B`
  measures **1264 MB/s** vs **191** for a random-miss stream. Address
  discipline only — the single largest perf lever available.
- **Pipeline, do not serialise.** Five one-at-a-time FSMs were found and fixed
  in L2C/fabric this session (6.8×, 7.7×, 2.97×…). The anti-pattern: a state
  that waits for a response before accepting the next request. Retire on the
  response and count the rest — `l2c_victim.v` and `l2c_bypass.v` are the
  in-tree templates.
- **AXI4 has no WID**: two clients' write bursts may never interleave. Use
  `l2c.v`'s owner-lock pattern — lock the port to one owner while it has
  writes outstanding, but let that owner run many in flight. Do not serialise
  clients to dodge it.

**Current verification:** `make tb-dma-engine` sweeps 128/256/512-bit AXI and
checks mixed short, unaligned, boundary-crossing and aligned-line reads/writes
from four clients at one accepted request per cycle. `make tb-dma-l2c` runs the
same traffic through the real L2C and reports cycles per 128-bit word.
`make tb-dma-engine-mut` proves the test rejects a named drop-last-byte mutant.

### Phase 3 — SONIC gets packet/DMA ports

Extend `q700_eth_sonic.v` with explicit packet and DMA-side ports. **Do not
bury Ethernet logic in the byte-oriented MMIO decode** — its own header says
so, and `make tb-q700-eth-sonic` is currently the only thing protecting the
register semantics. Keep it green throughout.

**Current state:** complete for TX and RX. `q700_sonic_cdc.sv` and
`q700_sonic_rx_cdc.sv` provide acknowledged PB↔core mailboxes; the register
block holds command bits until the corresponding engine operation completes.
The old immediate TX-completion behavior is retained only when
`PACKET_ENGINE=0`.

### Phase 4 — TX, then RX

**TX first** (simpler: we control when it happens). Fetch the transmit
descriptor and payload from Mac RAM via the engine, stream to Taxi, set TX
status and `ISR_TXDN` **only after the MAC completion is observed**.

**Current TX verification:** `make tb-q700-sonic-tx` covers both 16-bit and
32-bit SONIC descriptors, two unaligned fragments (70 + 5 bytes), consecutive
64/6/5-byte DMA request acceptance, deliberately out-of-order DMA responses,
AXI-stream backpressure, exact frame ordering, and status writeback only after
MAC completion. DMA client lane 0 is connected in the SoC when
`ETH_ENABLE=1`, `ETH_ICMP_RESPONDER=0`, and the legacy DDR RAM-disk owner is
disabled. The adapter streams descriptor payload only; Taxi owns padding and
FCS insertion (`cfg_tx_pad_en=1`). Consequently the SONIC `CRCI` descriptor
control is not used to append or pass a software-supplied FCS on this endpoint.
The test also sends a second, single-chunk frame using the EOL-tagged CTDA
loaded from the first descriptor's link value. This covers both the real SONIC
CTDA restart convention and the synchronous packet-BRAM prime cycle needed by
short frames such as ARP. `make tb-q700-sonic-tx-mut` proves the test rejects
the former link-field-address behavior and a missing BRAM-prime cycle.

**Then RX.** Accept a complete valid frame, apply SONIC receive filtering and
CAM state, write descriptor/status and payload through the engine, then raise
the masked interrupt. An asserted RX `tuser` is a receive error and **must
not** be DMA'd to guest RAM.

**Current RX verification:** the pinned Taxi RX pipeline validates and removes
the wire FCS. SONIC buffer contents and byte counts include it, so the adapter
reconstructs Ethernet CRC32 from the validated payload stream and appends the
four little-endian FCS bytes before DMA. `make tb-q700-sonic-rx` runs both 16-
and 32-bit descriptors. It proves a 125-byte payload becomes a 129-byte
FCS-inclusive SONIC frame and consecutive 64+64+1-byte DMA writes, including
the case where the FCS crosses a 64-byte packet-BRAM boundary. All payload
responses retire before the RDA is committed, and descriptor fields are
correct. It also covers request
backpressure, non-EOL completion, foreign-unicast/runt/Taxi-error rejection,
DMA loading of multiple CAM entries in both descriptor widths, CE-mask
selection, and stop/recovery after both descriptor EOL (`RDE`) and resource
exhaustion (`RBE`). `CR_LCAM` remains asserted until the CAM table and CE word
have actually returned through DMA, at which point CDP/CDC/CE and `ISR_LCD`
complete atomically. The named early-RDA mutant is rejected by
`make tb-q700-sonic-rx-mut`; `make tb-q700-sonic-rx-cam-mut` independently
proves the test rejects a missing CAP-word byte swap and missing FCS
reconstruction; it also rejects the former two-chunk-only BRAM lookahead that
repeated chunk 1 in place of chunk 2. The register test also
proves `CR_RRRA` remains asserted through the RRA DMA read and clears only when
the fetched CRBA/RBWC/RRP values are committed back to the register file.
TX and RX completion bits are merged atomically with a simultaneous ISR
write-one-clear, so coincident completions cannot overwrite one another.

**Implementation status (2026-08-21):** the canonical Ethernet-enabled build
now completes with the real m68k core and real MIG using `CPU=m68k`,
`ETH_ENABLE=1`, `ETH_ICMP_RESPONDER=0`, and `ALLOW_STALE_CPU=1`. Vivado routes
the design legally with zero failed, unrouted, or partially routed nets; final
DRC and bitgen both complete with zero errors. The debug-capable artifacts are
`build/vivado/fpga_top.bit`, `fpga_top.ltx`, and `fpga_top.buildinfo` (build ID
`0x1b1d1de0`). `make verify-fpga-debug-artifacts` accepts the set. Hold timing
closes (`WHS +0.009 ns`, zero failing endpoints), but 100 MHz setup timing has
a CPU-side miss. Post-route fanout optimization improved it from `WNS -0.101
ns` to `-0.078 ns`; retiming and replication did not improve further. The
reported critical paths are in existing m68k ALU/decode/issue/FPU logic, not
the DMA or Ethernet datapaths. Do not treat or load this negative-slack image
as the release candidate: a new timing-closed 100 MHz build is required.

The original generic DMA mapped to 12,609 LUTs and was the largest avoidable
source of the utilization increase. The compact implementation replaces wide
unaligned barrel shifters with aligned full-width bursts plus byte-sized
unaligned bursts, and uses one shared completion holding slot. Standalone
Vivado 2025.2 synthesis of the fully active four-client 128-bit configuration
is now **2,765 LUTs** (2,129 logic + 636 LUTRAM) and 673 flip-flops. The
128/256/512 tests, L2 integration test, throughput checks, and drop-byte mutant
all pass. Integrated synthesis and 100 MHz implementation are pending the
machine-wide Vivado build mutex.

A repository-wide variable-shift audit found no second arbitrary wide barrel
shifter comparable to the removed DMA alignment network. The compact DMA's
remaining 512-bit read placement is bounded to four positions for aligned
128-bit beats; its byte-granular fallback has 64 byte positions and is already
included in the 2,765-LUT measurement. The next-largest candidates are the
L2C hit/install byte strobes, but those are only 64-bit, four-position
quadrant placers. L2C round-robin rotates are 8-bit; DDR/VRAM AXI size decodes
are registered 32-bit one-hot shifts; the boot and video shifts are narrow and
off the reported critical paths. AXI-crossbar burst-size shifts occur only in
non-synthesis assertions. Do not replace bounded shifts with equivalent case
muxes without a before/after synthesis result.

The mapped design infers block RAM for all three wide stores: DMA write queue
`16x512`, SONIC RX packet store `32x512`, and SONIC TX packet store `48x512`.
The earlier illegal second input buffer on the MIG-owned `sys_clk_p/n` pins is
gone: `q700_eth_link.sv` takes the already buffered SoC utility clock instead
(100 MHz in real-MIG builds, 200 MHz in `SIM_MODEL`) and retunes its MMCM
divider while preserving the proven 125/312.5 MHz outputs. Synthesis also
exposed and led to removal of a genuine multi-driver on the RX capture staging
register; that register now has one clocked owner. The remaining gate is the
real classic-Mac path: boot this image, enable/configure MacTCP on the SONIC
interface, then prove ARP and bidirectional ICMP.

An optional JTAG-readable SONIC/Taxi telemetry page is now available for that
hardware gate. Build with `ETH_DEBUG_ENABLE=1` (in addition to
`ETH_ENABLE=1 ETH_ICMP_RESPONDER=0`) and run `eth-status` in
`tools/jtag_repl.tcl`; `eth-clear` resets only its counters/sticky snapshot.
It reports SONIC CR/IMR/ISR, TX/RX states and descriptor addresses, frame and
DMA progress/error counters, link/IRQ/RX-enable state, Taxi FIFO/FCS events,
and the first captured error. The page is absent by default and lives at
`0x5098_0000`; see `docs/eth_jtag_debug.md` for the exact ABI.

### Phase 4a — byte lanes: guest order is NOT AXI-invariant

**This SoC's memory does not use byte-invariant AXI lanes.**  The 68k LSU puts
the lowest-address byte of a 32-bit datum in `wdata[31:24]`, so a raw AXI beat
holds every 32-bit group byte-reversed relative to guest byte order.
`rtl/soc/vram_cpu_byteswap.v` documents the convention and corrects it on the
VRAM hop; the JTAG-AXI master shows the same thing directly (a raw `r
0x40800000` returns the ROM checksum `0x420dbff3`, i.e. lane bits `[31:24]`
hold ROM byte 0).

The DMA engine's clients speak **guest** byte order -- byte *i* of a request or
response is the byte at `guest_addr+i`.  `dma_engine.sv` therefore takes a
`BYTE_SWAP32` parameter and reverses W/R payload plus W strobes within every
32-bit lane; the SoC instantiates it with `BYTE_SWAP32(1)`
(`rtl/soc/fpga_top_dma.vh`).  Byte-granular beats replicate the data byte
across all lanes, so for those only the strobe permutation matters.

**This was the first hardware failure the real driver hit.**  Every SONIC
transmit fetched its descriptor header, read a reversed fragment-count word,
failed the `MAX_FRAGMENTS` sanity check and aborted before streaming a byte;
with no `TXDN` the driver timed out, gave up, and set `RXDIS`, which is why
nothing arrived either.  The signature in `eth-status` is
`tx_commands == tx_completions == tx_errors == dma_tx_requests` with
`tx_frames 0` and `dma_tx_errors 0` -- exactly one DMA per command, all
succeeding, all rejected by the client.

Note what did *not* catch it: `tb-q700-sonic-tx`, `tb-q700-sonic-rx` and
`tb-dma-engine` were all green, because each supplied its own memory model in
the same byte-invariant convention the RTL assumed.  A unit test cannot find a
disagreement about a convention when it inherits the convention from the thing
under test.  `tb-dma-engine-swap` now runs the whole engine suite against a
permuted memory model, and `tb-dma-engine-swap-mut` RED-verifies it by pointing
that model at an unswapped engine.

**Done when:** the descriptor-level Verilator tests remain green *and* the
real classic-Mac path passes: the SONIC driver initializes, MacTCP is enabled
and configured on Ethernet, host/Mac ARP resolves, and ICMP succeeds in both
directions. The FPGA-side synthetic responder is not evidence for this gate.

---

## 3. Constraints you cannot design around

| constraint | value |
|---|---|
| **URAM** | **100% used** — not an option, at all |
| CLB LUTs | 85.5% in the pre-compaction routed SONIC image; compact rebuild pending |
| Block RAM | 24.8% — **this is where FIFOs and descriptors go** |
| DSP48E2 | 1.48% — nearly untouched; right home for checksum/CRC and address arithmetic |
| congestion | level 5–6 |
| DDR total | **2 GiB** (`MT40A512M16LY-075` ×2, `AxiAddressWidth 31`) |
| clock domains | 125 MHz Taxi ↔ 100 MHz fabric ↔ 50 MHz `pb_clk` |

The **register/DMA** clock boundary is easy to miss: the SONIC register block
lives in `pb_clk` today, so if the engine lives in `fabric_clk100`, command
and status handoff crosses domains as well as the packet path.

---

## 4. Test ladder

1. `make test` in `~/rk5-eth` — byte-exact ARP/ICMP unit test.
2. Build the current SD provisioning helper with `make sd-provision-impl`.
   Do not reuse an old provisioning bitstream: the July 15 artifact predates
   the SD command-frame and write-session fixes and fails read-back verification
   on the board.
3. `make tb-q700-eth-sonic` after **every** register-side change.
4. Descriptor-level Verilator test before touching hardware.
5. Use `tools/boot_clean.sh` with `MAIN`/`LTX` set to the Ethernet artifacts;
   it must restore and verify the golden disk before loading the SoC image.
   Then exercise the actual classic Mac software path: the SONIC driver must
   initialize, MacTCP must enable and configure the interface, ARP resolution
   must succeed, and the Mac must answer bidirectional ICMP. Only then move on
   to bidirectional UDP and deliberate bad-FCS / oversize /
   RX-buffer-exhaustion cases. A PHY link or FPGA-side synthetic responder does
   not satisfy this gate.

**Every new test RED-verified against a named mutant.** Nine times this
session an agent's first test version silently passed the mutant it was
written to catch — twice because a new file was auto-found by Verilator's
`-I`/`-y` and so was **not a make prerequisite; nothing rebuilt**. If you add
a file, add it to the Makefile lists and prove a mutant fails.

---

## 5. Open questions worth answering before coding

- **Real DDR round-trip latency is still unmeasured.** It is the input to
  Little's Law for the engine's outstanding depth. `docs/soc_bus_review.md`
  Phase 0 has a JTAG pointer-chase procedure needing **no rebuild**.
- **Does the real Mac SONIC driver do cache maintenance?** Unnecessary now
  that we route through L2C, but the answer tells us whether a non-coherent
  path was ever viable, and it is cheap to read from the driver.

---

## 6. Later, deliberately deferred

A **network flavour of `boot_fsm`** — fetch ROM, PRAM and the HDD image over
UDP into DDR while the CPU is held in reset, then release it. See
`sonic_integration_plan.md` §4a. It is a *better* answer to HDD-over-LAN than
caching, because it removes network latency from the SCSI critical path
entirely (Q700 SCSI is CPU-driven pseudo-DMA — a round trip inside a data
phase presents as a hang, not a slow disk). Its ceiling is the 2 GiB DDR
budget, which must also hold Mac RAM, VRAM and the ROM; the cheapest useful
first step is network ROM/PRAM plus an optional image override, keeping SD as
the backing store.
