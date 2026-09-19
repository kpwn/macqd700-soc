# SONIC / GbE integration plan

Companion to `~/rk5-eth/docs/SONIC_HANDOFF.md`, which owns the proven PHY/MAC
baseline (RTL8211F RGMII, Taxi MAC, WNS +0.715 ns, 200/200 ICMP) and the
MAC↔SONIC AXI-stream contract. **Read that first.** This document covers only
the SoC-side decisions it leaves open, and the constraints this repo imposes
on them.

## 0. Decisions taken

1. **SONIC DMA routes through L2C.** Coherent by construction.
2. **A DMA engine sits on top** of the SONIC register block.
3. **Speculative but shaping:** the FPGA may also serve the HDD over LAN.
   Treat that as a design input now, not a retrofit later — see §4.

---

## 1. Why routing through L2C was the right call

SONIC really is a bus master — this is worth stating because the Q700's *other*
mastering-looking peripheral is not. MAME's `macquadra700.cpp:776`:

```cpp
DP83932C(config, m_sonic, 40_MHz_XTAL / 2);
m_sonic->set_bus(m_maincpu, 0);        // the CPU's own address space
```

and `dp83932c.h` exposes `read_bus_word()` / `write_bus_word()` / `read_rra()`.
It fetches descriptors and moves payloads itself. (Contrast SCSI, which is
**pseudo-DMA** — CPU-driven through the TurboSCSI handshake. Do not
generalise from it.)

Real SONIC sat on a bus whose caches saw its traffic. Ours does not:
`docs/l2c_spec.md` §1 makes L2 the **single point of coherency**, with **no
snooping**, and CPUSH/CINV never reach L2. Left on a private path, TX would
fetch stale descriptors and RX would deposit frames the CPU cannot see.
Routing through L2C removes the whole class, and removes any dependency on
what the real Mac driver happens to do about cache maintenance.

---

## 2. The constraints that decision inherits

### 2.1 The same-ID cliff — the biggest performance trap

Measured on this L2C (see `docs/v2_interconnect_handoff.md` §2):

```
miss_read   8 outstanding, UNIQUE IDs :  8.38 cyc/op  (191 MB/s)
miss_read   8 outstanding, ONE ID     : 55.40 cyc/op  ( 29 MB/s)
```

`l2c_ctrl`'s Critical-3 `id_busy_c` gate blocks a second live op per AXI ID, so
a master presenting **one constant ID gets no benefit from the 8 MSHRs at
all** — 6.6× worse, and *worse than a single outstanding request*.

**The DMA engine must present distinct AXI IDs per outstanding transaction**,
or it will measure as a broken NIC and the cause will not be obvious. Every
existing master on this SoC presents one constant ID (`if_to_axi.v:16`,
`axi_narrow_to_wide.v:72`, the debug host, `vhdd_ddr.v`), so there is no
in-tree example to copy — this is new-code discipline, not a pattern to follow.

At 1 GbE line rate (125 MB/s) against 191 MB/s of achievable miss bandwidth,
the margin is real but not generous. At 29 MB/s it does not close.

### 2.2 There is no free crossbar seat

`axi_xbar.v` is `N_MASTERS = 3`. Seat 0 is already **shared** between CPU-data
and the boot writer via a plain 2:1 mux, which is only legal because reset
sequencing makes them mutually exclusive by construction. **SONIC has no such
exclusivity** — it masters while the CPU runs, which is the entire point.

So one of:
- widen the crossbar (seats cost **routing, not LUTs** —
  `docs/soc_v2_interconnect_shape.md` §5.2 — and congestion is already 5–6);
- attach to L2C on a path that does not traverse the crossbar, as the
  victim/bypass writeback path does (`fpga_top_ddr.vh` shows L2C's master side
  reaching DDR via `axi_vram_priority_mux3` → async bridge → MIG bridge, no
  crossbar involved);
- share an existing seat with genuine arbitration rather than a mux.

The second is the most promising and the least explored.

### 2.3 Resource budget, post-route from the last full build

| resource | used |
|---|---|
| CLB LUTs | **76.9%** |
| **URAM** | **100%** — none available |
| Block RAM | 24.8% |
| CLB registers | 26.5% |
| DSP48E2 | **1.48%** (27 of 1824) |
| congestion | level **5–6** |

Frame FIFOs and descriptor storage must come from **BRAM**. Nothing may want
URAM. DSP is nearly untouched and is the right home for any checksum/CRC
offload — `CLAUDE.md` already makes DSP-for-arithmetic repo policy, and the
DSP48E2's A/B/M/P pipeline registers are free, which suits a latency-tolerant
packet path.

### 2.4 Three clock domains

125 MHz `logic_clk` (Taxi) ↔ 100 MHz `fabric_clk100` (L2C/DMA) ↔ 50 MHz
`pb_clk` (SONIC registers). The handoff already mandates an async frame FIFO
per direction at the first boundary. The **register/DMA** boundary is a second
crossing and is easy to overlook: the register block lives in `pb_clk` today,
and if the DMA engine lives in `fabric_clk100` then command/status handoff
crosses domains too.

---

## 3. Shape of the DMA engine

**The DMA engine is THE L2C master. SONIC is a consumer of it.** Not a
SONIC-internal engine, not one master per client. Goal: **maximum throughput,
minimum area.**

That inverts the ownership the handoff implies and it resolves §2.2 for free:
**one attachment point, not one per client.** SONIC, a future
network-block-device client, and anything else that needs guest RAM all arrive
through the same port, so the crossbar/L2C question is answered once.

```
        SONIC packet adapter ──┐
   (future) block/LAN client ──┼──> DMA ENGINE ──> single L2C master port
              (future) other ──┘     (descriptors,
                                      SG, ordering)
```

### Client contract — keep it narrow

Clients do **not** get an AXI port. They get a request/completion pair:
`{guest_addr, length, direction, tag}` in, `{tag, status}` out, plus a data
stream. Anything wider duplicates arbitration and address logic per client,
which is exactly the area we are trying not to spend.

Descriptor *formats* stay client-side policy. SONIC's CDP/TXP/RRA/RDA layout
is one client's encoding; the engine moves bytes between guest addresses and
streams and does not learn it. `q700_eth_sonic.v`'s own header already
anticipates this split:

> *"The packet side should connect behind that descriptor engine as an
> AXI-stream producer/consumer."*

### For maximum performance

1. **Per-transaction AXI IDs — non-negotiable** (§2.1). One constant ID caps
   the engine at 29 MB/s against 191 MB/s available. 1 GbE needs 125 MB/s.
   **This single decision is the difference between a working NIC and a
   broken-looking one.**
2. **128-bit datapath**, matching L2C's `DATA_WIDTH`. Do not build a 32-bit
   engine and widen later — `axi_narrow_to_wide` exists precisely because
   something did that, and it cost two rounds of work this session to get it
   to 1.04 cycles/word.
3. **Emit 64 B-aligned full-line writes wherever possible.** L2C has a
   full-line-write gather path (`00f079c`) that installs a line with **zero
   fill beats**. Measured: `stream_write_64B` runs **1264 MB/s** against
   `hit_read_64B_burst`'s 1594 and a random-miss stream's 191. An RX DMA that
   lands frames on line boundaries hits that path; one that dribbles unaligned
   bursts pays a fill per line. This is the largest single perf lever
   available to the engine and it costs only address discipline.
4. **Size outstanding depth from measured latency, not a guess.** Little's Law
   against the L2 miss round trip — which this repo has **never measured**
   (§6). Note every depth in L2C is 8 (MSHRs, victim slots, bypass slots, MIG
   reads) and they were each argued to 8 *because the next thing downstream
   was 8*. Match that set or raise it deliberately.
5. **Pipeline, do not serialise.** Five separate one-at-a-time FSMs were found
   and fixed in L2C and the fabric this session, worth 6.8×, 7.7×, 2.97× and
   more. The pattern to avoid: a state that waits for a response before
   accepting the next request. Retire on the response, count the rest —
   `l2c_victim.v` and `l2c_bypass.v` are the in-tree templates.

### For minimum area

- **Descriptor and reorder storage in BRAM** (24.8% used), never flops. LUTs
  are at 76.9% and URAM is at **100%** — URAM is not an option at all.
  `878d002` and `386ea29` both moved payload arrays flops→LUTRAM/BRAM and
  measured the saving; follow that.
- **Address arithmetic into DSP48E2** — 1.48% used, and `base + index × stride`
  is an exact `(A+D)×B+C` fit for the pre-adder. Its A/B/M/P pipeline registers
  are free, which suits a latency-tolerant path. `CLAUDE.md` already makes this
  repo policy; the video path ignored it for years and paid in LUTs.
- **One arbiter, one address path, one outstanding table.** The whole reason
  the engine is shared is to pay for these once.
- **AXI4 has no WID** — two clients' write bursts may never interleave on the
  wire. `l2c.v`'s owner-lock is the pattern: lock the port to one owner while
  it has writes outstanding, but let that owner run many in flight. Do not
  solve it by serialising clients.

## 4. HDD over LAN — what it implies *now*

Treating this as a design input rather than a later retrofit changes two
things:

**It justifies the generic engine (§3).** A network-backed block device needs
the same scatter-gather DMA into guest RAM that SONIC needs. Building it
SONIC-shaped and generalising later means rewriting the descriptor path.

**It collides with SCSI's pseudo-DMA model, and that is the real risk.** The
Q700 SCSI path is **CPU-driven**: the CPU spins on the TurboSCSI handshake and
moves every byte itself. There is no descriptor engine to hide latency behind.
So a SCSI data phase backed by a network round trip stalls the CPU for the
whole round trip — and at Mac-driver timescales that presents as the machine
hanging, not as a slow disk. `docs/handoff_2026_08_20.md` §2 has a worked
example of how badly a livelock inside a driver loop presents from the outside.

If HDD-over-LAN is pursued, the block path needs to be **asynchronous to the
SCSI data phase**: a local cache or read-ahead buffer in DDR that the SCSI
target serves from at bus speed, with network fills happening off the critical
path. Design that in from the start; retrofitting it into a synchronous
handshake is much harder than building it that way.

Note also that DDR is already the RAM-disk backing (`vhdd_ddr.v`) and that
`3bf03b3` removed the reason `ENABLE_DDR_RAMDISK` was disabled — so the
buffering substrate exists and is newly usable.

---

## 4a. Later: a network flavour of `boot_fsm` (idea, not scheduled)

Sketched 2026-08-20, deliberately deferred. Recorded because it changes what
"HDD over LAN" should mean and is a **better answer than §4's read-ahead
cache**.

`boot_fsm` already stages artifacts from SD into DDR while holding the CPU in
reset, then releases it. A network flavour would keep that shape and change
only the **source**: fetch ROM, PRAM and the HDD image over UDP into DDR, then
release the CPU exactly as today.

Why this is the better shape: it **removes network latency from the SCSI
critical path entirely.** §4's problem was that Q700 SCSI is CPU-driven
pseudo-DMA, so a network round trip inside a data phase stalls the CPU and
presents as a hang. If the image is already in DDR before the CPU runs, no
data phase ever touches the network. The runtime path is unchanged — the SCSI
target serves from DDR at bus speed, which `vhdd_ddr.v` already does and which
`3bf03b3` newly unblocked.

It also composes with everything else here: the same L2C-mastering DMA engine
(§3) does the DDR staging, and the same MAC/stream plumbing carries the
transfer. The only genuinely new piece is a minimal UDP/IP (or raw-Ethernet)
requester, and it runs entirely before the CPU is released — no driver, no
interrupts, no coherency question, no reentrancy.

**The one hard constraint: DDR is 2 GiB total** (`MT40A512M16LY-075` ×2 via
`DDR4_DataWidth 32`, `AxiAddressWidth 31`), and it must simultaneously hold
Mac RAM (up to 128 MB), VRAM (2 MB), the 1 MB ROM, and the staged image. The
SCSI target currently advertises just under 2 GiB (`SCSI_MAX_LBAS` = 4194240
after `c0a506d`), so **a full-capacity image cannot be staged.** Realistic
options:

- ship a small image (a few hundred MB) sized to fit alongside everything else,
  and advertise a capacity that matches it rather than the 2 GiB clamp;
- stage boot-critical blocks and fetch the remainder on demand into a DDR
  cache — which reintroduces §4's latency problem, but only for cold blocks
  and only after boot, which is a far weaker version of it;
- keep SD as the backing store and use the network flavour only for ROM/PRAM
  plus an optional image *override*, which is the cheapest useful subset.

The third is probably the right first step: it is small, it makes netboot real,
and it does not require answering the capacity question at all.

## 5. Suggested order

1. **Settle the master-port attachment** (§2.2). It gates everything else and
   is the piece with no obvious answer.
2. Bring the Taxi MAC into the SoC build per the handoff's *Build ownership*
   section — pinned source lists and its constraint Tcl, not a copied
   bitstream. Keep the ICMP responder selectable as a physical-link regression
   image.
3. **Generic DMA engine** with per-transaction AXI IDs, verified against L2C
   in sim before any SONIC logic depends on it. A descriptor-level Verilator
   test (handoff step 4) is the right gate.
4. TX path, then RX path, per the handoff's order.
5. Re-measure. `make tb-q700-eth-sonic` must stay green throughout — it is the
   only thing currently protecting the register semantics.

## 6. Open questions worth answering before coding

- **Does the real Mac SONIC driver do cache maintenance?** Routing through L2C
  makes it unnecessary, but knowing the answer tells us whether a
  non-coherent path was ever viable, and it is cheap to check in the driver
  disassembly.
- **What outstanding depth does the DMA engine need?** Little's Law against
  measured L2 miss latency. `docs/v2_interconnect_handoff.md` records that the
  real DDR round-trip latency is **still unmeasured** — that is the input, and
  `docs/soc_bus_review.md` Phase 0 has a JTAG procedure needing no rebuild.
- **Where does the 125 MHz Taxi domain get its clock in the SoC build**, and
  does adding it perturb the existing MMCM/PLL budget?
