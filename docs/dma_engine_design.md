# DMA Engine Architecture — Peripheral Master Hub, Raw-DDR + Coherent Service Modes

Status: **design document, no RTL** (v2 — supersedes the disk-DMA-only v1
draft in this file's history).  Written 2026-08-19 against `15e4650` plus the
working-tree RAM-disk removal.  Every claim about current behaviour carries a
`file:line` cite; anything not verified from source is marked *estimate* or
*unverified*.

## 0. What this engine is (revised mission)

Per the maintainer's direction, this is **not** "a DMA engine for disk".  It
is the **single shared master seat through which peripherals — and JTAG bulk
traffic — reach memory**, with two hard-split service modes:

1. **Raw DDR** — private backing store for a peripheral that needs DDR *for
   itself* (RAM-disk volume, a future device's private buffers).  **No
   coherency, at all, by definition**: the region is not SoC-observable and
   is never expected to be consistent with anything the CPU sees.  That is
   what makes it cheap — no L2 involvement, no front-door serialization, no
   `CPUSH`/`CINV` contract.
2. **AXI system-bus view** — anything **observable from the SoC always goes
   through the main AXI bus and through L2C**.  Not a per-client
   optimisation to trade away: if the CPU can see it, it went through L2
   (with L2 as the point of coherency, and a per-descriptor *non-allocating*
   hint — §6.3 — which is an **allocation policy, not a coherency bypass**).

N peripherals cost **one** xbar master seat (plus one non-seat tap for the
raw path), not N seats.  The first client is the DDR RAM disk, whose
standalone implementation has just been compiled out to free area and its
seat (§1.1); JTAG bulk transfer is the second client, in both modes (§7).

---

## 1. Hard resource context

| Quantity | Value | Source |
|---|---|---|
| CLB LUTs | 188,226 / 216,960 = **86.76 %** (84.26 % as logic) | `build/vivado/reports/utilization_synth.rpt` |
| URAM288 | **64 / 64 = 100 %** (all l2c data arrays) | same report; `docs/l2c_spec.md` §3 |
| Router initial congestion | **Level 6** (South/Long); contributors `u_cpu` (75 %), `u_rob`, `u_l2c/g_active.u_ctrl` | `build/vivado/reports/congestion_route.rpt` §2 |
| fabric_clk100 closure | recent completed routes: WNS **+0.047** (`timing_20260818_085855`), **+0.533** (`_160913`), **+0.114** (`_222419`), **+0.188 ns** (`timing_20260819_023113`); build in flight: −0.606 at postRoute-physopt | `synth/timing_reports/*.rpt`, Intra Clock Table |
| Fabric clock | 100 MHz | timing reports, clock table |

A 33-LUT netlist delta has flipped route_design from clean to 346 residual
overlaps (per the brief).  Every option below is scored by LUT delta and by
whether it touches a congestion-report module.  **`rtl/soc/l2c*.v` is owned
by another agent right now** — anything needed inside l2c is *specified
here*, not edited (§6.3).

### 1.1 Freed by the RAM-disk removal (working tree, `ENABLE_DDR_RAMDISK` off)

The DDR RAM disk (`vhdd_ddr`) has been compiled out and its xbar seat tied
off, explicitly "ready for the next master to claim"
(`rtl/soc/fpga_top_dma.vh`, M3 tie-off block: "Freeing this seat is one of
the reasons for removing the RAM disk — the xbar read fan-in was 4/4 full").
Coordinator-measured post-route costs freed:

- `u_vhdd_ddr` 665 LUT / 339 FF, `u_vhdd_ddr_cdc` 672 LUT / 402 FF —
  **1,337 LUT freed now** (`rtl/soc/fpga_top_peripherals.vh`, removal
  comment block).
- `g_active.u_bypass` (`l2c_bypass`) 213 LUT / 320 FF — the RAM disk was
  its only bulk consumer, **but this saving is pending, not landed**: the
  window is still instantiated and enabled (`rtl/soc/fpga_top_ddr.vh:148`,
  `.BYP_WIN_EN(1'b1)`), and the xbar still decodes the `0x7000_0000`
  aperture onto S0.  Disabling it is a parameter/decode change at the
  integration sites (not an l2c source edit) and is sequenced in §12 —
  it must land **together with** removing the aperture decode (§13 F6).
- **The M3 master seat** — previously the only free read-fan-in slot did
  not exist (reads were 4/4: M0 LSU, M1 debug, M2 IF, M3 vhdd —
  `rtl/soc/axi_xbar.v:71-84, 772-773`).  M3's fan-in tag is literally
  `XBAR_M_DMA` (`rtl/soc/axi_defs.vh:251-252`).

Verified green after removal (coordinator): lint clean, tb-sd-boot 11/11,
tb-scsi 41/41, tb-scsi-dual 7/7.

---

## 2. Ground truth — the fabric the hub plugs into

(Condensed from v1; all cites re-verified.)

### 2.1 L2 cache

`rtl/soc/l2c.v` + submodules, spec `docs/l2c_spec.md`:

- 2 MB, 64 B lines, 8-way, 4096 sets (`docs/l2c_spec.md` §3).  **One** AXI4
  slave port (128-bit, ID width 6) from xbar S0; one master port toward DDR
  (`rtl/soc/l2c.v:40-84`).  No snoop, no probe port, no maintenance CSRs.
- Front door: **one op at a time**, per-16 B-beat lookup, measured hit
  round-trip 5.1–5.3 cycles (`docs/l2c_spec.md` §2, §4, §10).
- MSHR: 8 entries, 4-deep replay FIFOs, 4-beat 64 B fills; **8 concurrent
  misses in 66 cycles vs 204 sequential** (measured, §5/§9).  A new accept
  is blocked while its AXI ID is live in MSHR/bypass — **single-ID masters
  get zero miss overlap** (§9 "AXI4 response ordering").
- Every write it accepts is write-allocate/write-back; there is **no
  non-allocating request path today** (§4 miss path allocates
  unconditionally; confirmed against `rtl/soc/l2c_ctrl.v` S_LOOKUP →
  MSHR-alloc flow).  Non-INCR bursts are SLVERR'd; any `AxSIZE` is accepted
  (`rtl/soc/l2c_ctrl.v:25-35, 137-139`).
- Bypass window (`rtl/soc/l2c_bypass.v`): address-disjoint pass-through
  that skips the tag arrays entirely; **single-outstanding and single-beat**
  (front door re-fires per beat, `l2c_bypass.v:49, 62-68`; master side
  ARLEN=0 only, `:122-124`) ⇒ ~16 B per DDR round trip ≈ **~40 MB/s**
  ceiling — *this is why the RAM disk was slow*, and the raw path must not
  reinvent it (§5.2).  It also has a documented unfixed mid-W-burst/stray-B
  reset gap (`docs/l2c_spec.md` §9 Important-A closing note).

### 2.2 Crossbar, masters, DDR path

- `rtl/soc/axi_xbar.v`: 128-bit AXI4.  Live masters after the removal:
  **M0** CPU LSU/boot mux, **M1** host debug ("host debug (XDMA /
  JTAG-to-AXI)", `rtl/soc/fpga_top_xbar.vh:312`), **M2** CPU IF (read-only),
  **M3 free (tied off)**.  Hand-written 4-slot RR fan-in the project
  declined to resize (`axi_xbar.v:71-84`).  Per-slave watchdog 2^18 cycles
  (`:348, 1412-1439`).
- DDR flattening + runtime RAM alias mask applied in the xbar
  (`rtl/soc/axi_defs.vh:176-214, 179-183`).
- Downstream of l2c: `axi_vram_priority_mux3` merges {l2c master port, S3
  VRAM lane, scanout reader}; scanout wins idle AR, quota
  `MAX_SCAN_AHEAD=4`, and at most `MAX_BULK_AHEAD=2` non-scan bursts
  outstanding downstream (`rtl/soc/axi_vram_priority_mux3.v:39-57,
  95-105`).  Scanout margin at 1024×768×24bpp: demand 189 MB/s vs ~267 MB/s
  fill ≈ 1.4×, built on an *assumed* ~40-cycle DDR RTT
  (`rtl/soc/scanout_ddr_reader.v:49-77`).
- MIG: 256-bit UI @ 333.25 MHz, DDR4-2400 ×32 DQ ⇒ ~9.6 GB/s peak; bridge
  is cut-through, 8 outstanding reads / 4 writes, single-ID in-order
  (`rtl/board/ddr_ctrl.v:41-52`, `rtl/board/axi_ddr4_mig_bridge.v` header,
  `rtl/soc/fpga_top_ddr.vh:207-239`).  Fabric ceiling: 128 b × 100 MHz =
  **1.6 GB/s**.

### 2.3 CPU L1 (what no hardware below the CPU can see)

`cpu/rtl/core/mem/dcache.v`: 4 KB, 4-way, **32 B lines** (`:65, :274`),
write-back, **no inbound snoop port** (only the outbound D→I SMC export,
`:111-124`).  `CPUSH` here = writeback, **leave valid+clean**
(`:195-197`); `CINV` = drop.  L1 writebacks are ordinary M0 AXI writes that
L2 absorbs — "CPUSH costs L1→L2 only" already holds for the CPU side
(`docs/l2c_spec.md` §1 "Why CPUSH/CINV never reach L2").

### 2.4 Reusable pieces

- `rtl/soc/dma_ctrl.v` (852 lines, unit-tested, `docs/dma_ctrl.md`): 4
  channels, 64-bit master, ≤16-beat INCR, 32 B chained descriptors, level
  IRQ, pause/abort, per-channel error latch.  Measured cost of its removed
  instance: **2,024 LUT / 814 FF** (`rtl/soc/fpga_top_dma.vh` history
  block).  IRQ plumbing to `irq_agg.rsvd_irq6` (IPL 6) still exists, fed 0
  (`fpga_top_dma.vh`, `rtl/soc/fpga_top_peripherals.vh:2339`).
- `rtl/vhdd.vh`: the SCSI backing-store client contract (request/extent
  probe/byte-stream with credit) — the template for the hub's client ports.
- `rtl/soc/axi_n64_to_wide.v`: 64→128 half-lane shim (no beat folding,
  half bandwidth, `:11-15`).  Calibration point for shim cost: the
  same-shaped `u_jtag_n2w` measures 323 LUT / 518 FF (coordinator-measured).
- JTAG-AXI path cost (coordinator-measured, post-route): `u_debug_jtag_axi`
  674 LUT / 1,679 FF + `u_jtag_n2w` 323/518 + `u_dbg_pb_to_core` 340/357 ≈
  **1,337 LUT** plus the M1 seat.  (`u_debug` 4,635 LUT and `u_dbg_vio`
  2,683 LUT are the CSR block and VIO and stay regardless.)

---

## 3. The service model, stated precisely

This section is normative; everything else derives from it.

**Raw-DDR mode.**  A raw client owns a **disjoint region of flattened DDR**
(a carve-out like `AXI_VRAM_DDR_CARVEOUT_BASE` or `AXI_DDR_RAMDISK_OFFSET`,
`rtl/soc/axi_defs.vh:173-214`).  The engine clamps every raw access into the
client's configured `{base, limit}` in hardware (precedent: `vhdd_ddr`'s
`MAX_LBAS` clamp, `rtl/soc/axi_xbar.v:107-121` M3 note).  The region is
**not SoC-observable**: no xbar decode routes it, the CPU cannot address it,
L2 can never hold a line of it.  Consequently the raw path carries **zero
coherency machinery and zero ordering guarantees against the system bus** —
by design, not omission.  Its only contract is AXI-legal bursts and the
engine's completion signalling.

**System-bus mode.**  Everything SoC-observable goes through the main AXI
bus and through L2C.  There is no fast incoherent path for shared data.
The coherency story is L2-as-PoC plus the `CPUSH`/`CINV` L1 contract (§6.1).

**The non-allocating bit (system-bus mode only) is an allocation policy,
not a coherency bypass.**  Maintainer's words: *"even if non-allocating bit
will probably be settable to let accesses bypass cache if line is not
resident."*  Precisely:

- Line **resident** → the access **hits and is fully coherent**: a read
  returns L2's (possibly dirty) data; a write updates the resident line.
  The tag probe is never skipped.
- Line **not resident** → do not allocate: the access goes to DDR without
  pulling the line into L2.  What is saved is the fill, the eviction, and
  the pollution — **not the probe**.

Do not conflate this with `l2c_bypass`: the window mechanism skips the tag
arrays entirely and is only sound because its addresses are
**disjoint-by-construction** from anything cacheable (`l2c_bypass.v`
elaboration `$fatal`, `:168-183`; the Round-4 mask-bug post-mortem
`:144-164` shows what overlap does — real corruption).  A non-allocating
access targets *cacheable, shared* addresses and therefore must pay the tag
lookup every time.  Conflating the two produces a coherency bug that only
appears under load (§13 F10).

---

## 4. Hub architecture — N clients, one seat

```
   clients (K ports)                       ENGINE (core_clk)                    fabric
  ┌──────────────────┐   ┌──────────────────────────────────────────┐
  │ RAM-disk provider│──►│ client ports: req/cmpl + data FIFO,      │   sys port (128b AXI4)
  │  (vhdd contract, │   │   per-client static config:              │──► xbar M3 seat ─► S0 ─► l2c
  │   pb_clk, 1 CDC) │   │   {mode, base/limit clamp, burst cap,    │        (coherent view)
  │ JTAG bulk (raw)  │──►│    QoS class, noalloc default}           │
  │ JTAG bulk (sys)  │──►│ channel FSMs + descriptor engine         │   raw port (128b AXI4)
  │ future: NVMe,GbE │──►│   (dma_ctrl.v lineage)                   │──► 2:1 tap at the l2c_m ─►
  │ CPU-programmed   │   │ inter-client arbiter: burst-granular RR  │    axi_vram_priority_mux3
  │  descriptors(S2) │   │   + 2 QoS classes + per-chan watchdog    │    "l2c" input ─► MIG
  └──────────────────┘   └──────────────────────────────────────────┘        (raw view)
```

### 4.1 Client model

A *client* is a device-side consumer bound to an engine channel.  Two
binding styles, both already proven in-tree:

- **Streaming client** (RAM disk, future NVMe/GbE data): a port shaped like
  the `vhdd` contract (`rtl/vhdd.vh` — request `{lba/addr, count, dir}`,
  completion `{done, error}`, byte/beat stream with credit).  The engine
  owns the AXI side; the client never sees the bus.
- **Descriptor client** (CPU driver, JTAG bulk): transfers programmed as
  `dma_ctrl`-style descriptors/registers through the S2 config window
  (`docs/dma_ctrl.md` §2), or through the JTAG aperture mapping (§7.2).

Per-client **static configuration** (parameters or cfg registers, set once):
`mode` (RAW / SYS), `region {base, limit}` (RAW: hardware clamp; SYS:
optional restriction), `burst cap` (log2 beats, ≤16), `QoS class`
(latency-sensitive / streaming), `noalloc default` (SYS only, §6.3).
Per-descriptor fields can lower (never raise) burst cap and toggle
`noalloc`.

### 4.2 What is shared vs per-mode

**Shared** (one instance total): descriptor engine + channel FSMs, channel
register file + IRQ logic, client data FIFOs' arbitration, the single
pb→core CDC for pb-domain clients (today `vhdd_ddr` + its private CDC cost
1,337 LUT for *one* client; the hub amortizes one CDC across all pb
clients).

**Per-mode** (two master ports):

- **Sys port** → xbar **M3** (the freed seat).  The xbar applies address
  decode, DDR flattening, and the runtime RAM alias mask **for free**
  (§2.2) — no duplicated flatten logic, no ID-space surgery at l2c (S0
  ID width stays 6).
- **Raw port** → a new **2:1 arbiter inserted at the `l2c_m_* →
  axi_vram_priority_mux3` seam** in `rtl/soc/fpga_top_ddr.vh` (a `.vh`
  integration site, not an l2c source file).  The mux3 keeps exactly 3
  sources, so **the scanout guarantee is preserved structurally** — raw
  bursts are just more "bulk" under the existing `MAX_SCAN_AHEAD=4` /
  `MAX_BULK_AHEAD=2` bounds.  Raw burst length is capped (8–16 beats,
  Phase-0 measurement decides, §15) so a scan request never waits behind
  more than 2 × cap beats of bulk data.  Response routing: downstream is
  single-ID in-order (`axi_ddr4_mig_bridge.v` header), so an
  accepted-order route FIFO suffices (same idiom as mux3's own 8-entry
  AR FIFO, `axi_vram_priority_mux3.v:70-76`) — no ID widening anywhere.

Clamps: the raw port hard-clamps every address into the issuing client's
region; a violating descriptor completes with error, no bus activity.  An
elaboration-time check asserts every raw region is disjoint from the l2c
cacheable span *and* from every other client's region (same doctrine as
`l2c_bypass.v:168-183`, relocated into the engine).

### 4.3 Inter-client arbitration and QoS

- **Burst-granular round-robin** (the `dma_ctrl` precedent: a channel owns
  the master only for one burst; worst hold = one burst ≤ cap beats,
  `docs/dma_ctrl.md` §1.1/§1.2).
- **Two QoS classes**: between bursts, any pending latency-sensitive
  channel is picked before streaming channels (RR within class).  Bound: a
  latency-sensitive request waits ≤ 1 in-flight burst + (#peer LS channels)
  bursts.  A streaming client cannot starve anyone: it only ever holds one
  burst.  Cost: a class bit and a two-level picker over ≤8 channels —
  small (*estimate* <100 LUT).
- **Per-channel unconditional watchdog** (the `vhdd_ddr`/`sd_ctrl`
  bounded-response doctrine, `rtl/soc/vhdd_ddr.v:48-77`): armed per
  request, counts every cycle unconditionally, applied after the state
  case.  On expiry: error completion + AXI-legal quiesce (pad W with
  zero-strobe beats to WLAST, sink the B).  This is what keeps one wedged
  *client* from wedging the shared engine (§13 F11).
- **Never start an AXI burst you cannot finish**: writes issue AW only
  when the burst's data is fully staged in the client FIFO; reads accept R
  only into guaranteed buffer space (the `vhdd_ddr` blkbuf decoupling
  rule, `vhdd_ddr.v:29-43`).  This makes client stalls invisible to the
  bus and is a hard engine invariant in both modes.

---

## 5. Attach points — revised verdicts

### 5.1 System-bus (coherent) mode: xbar M3 seat — **selected**

v1 of this document recommended a 2:1 merge at the S0→l2c seam ("Option B")
because the read fan-in was full.  The RAM-disk removal freed M3, which
flips the verdict:

| | v1 Option B (S0-seam merge) | **M3 seat (now selected)** |
|---|---|---|
| New fabric logic | ~0.6–1.2k LUT arbiter + route logic (*estimate*) | ~0 (rewire tied-off slot; shim `axi_n64_to_wide` ≈ 323-LUT class in Phase 2a, dropped at 128-bit) |
| l2c impact | ID_WIDTH 6→7 parameter, +1 bit through MSHR/CAM | **none** |
| Address handling | duplicate flatten + alias mask at the seam | **free** (xbar does it) |
| Fairness | new arbiter policy to define | existing 4-slot RR fan-in |
| Congestion-report modules touched | l2c_ctrl (ID width) | none |

Coherency is identical either way: every M3 transaction lands on S0 and
passes the one front door (`docs/l2c_spec.md` §1 invariant 2).  The seat's
fan-in tag is already `XBAR_M_DMA` and, being tagged like the old DMA
master, its ROM-window writes are dropped-with-OKAY like the CPU's — the
conservative default (`axi_xbar.v:107-121`).

### 5.2 Raw-DDR mode: downstream 2:1 tap — **selected**; l2c-bypass rejected

The maintainer settled it — *"raw DDR does not need L2C; only axi bus view
will"* — and the numbers agree:

| | l2c bypass window (deleted consumer) | **downstream 2:1 tap (selected)** |
|---|---|---|
| Bandwidth | **~40 MB/s** (single-beat, single-outstanding engine — `l2c_bypass.v:49, 122-124`; the measured reason the RAM disk was slow) | burst-capable, multi-outstanding up to `MAX_BULK_AHEAD=2`; ~270–800 MB/s (*estimates*, §9) |
| CPU cost | every bypass beat occupies the single front-door lookup slot (`docs/l2c_spec.md` §4) — raw streaming *slows the CPU* | zero front-door occupancy |
| Correctness debt | unfixed mid-W-burst/stray-B reset gap (`docs/l2c_spec.md` §9 Important-A note) | new, small, reviewed-idiom arbiter (grant-lock + reset-hold per mux3) |
| Editability now | inside `l2c*.v` — **owned by another agent** | `.vh` seam + new file |
| Area | +213 LUT kept | +0.4–0.8k LUT (*estimate*), and the 213-LUT bypass frees once `BYP_WIN_EN→0` |

Rejected likewise (unchanged from v1, briefly): a **new 4th mux3 source**
(re-derives the hard-realtime scan bound — never do this); a **tag-probe
snoop port** (replicates the entire front-door hazard fabric inside the
congestion-critical, other-agent-owned l2c for bandwidth nothing needs);
**L1 snooping** (architecturally moot — the 68040 contract already requires
`CPUSH`/`CINV`, and `u_cpu` is the congestion epicentre).

### 5.3 The xbar seats after this design

M0 = CPU LSU/boot, M1 = **host debug, kept** (§7.4), M2 = CPU IF, M3 =
**engine sys port**.  Zero free seats, zero picker changes.  On the
coordinator's "two free seats vs shrink the picker" question: **neither** —
M1 must stay for debug-path independence (§7.4), M3 is consumed by the
engine, and shrinking the hand-written picker is the same
load-bearing-arbiter rewrite the project declined in the other direction
(`axi_xbar.v:71-84`) for a return of a few hundred LUT (*estimate*).  Not
worth the route risk at +0.05-ns margins.

---

## 6. Coherency, per mode

### 6.1 System-bus mode — the four cases (unchanged in substance from v1)

| Case | What happens | Who handles it |
|---|---|---|
| Engine read, line dirty in **L2** | Front-door hit returns dirty data (`docs/l2c_spec.md` §4; invariant 2) | **Hardware** |
| Engine read, line dirty in **L1 only** | Invisible below the CPU (no L1 snoop port, `dcache.v:187-206`). Driver `CPUSH`es the range; dirty L1 lines land in L2 as ordinary M0 writes; the engine read then hits L2. **L1→L2 only — Goal 1's payoff.** | **Software** (`CPUSH` first) |
| Engine write, line **clean in L2** | Write hit, line dirtied in place | **HW** for L2; **SW** `CINV` for any L1 copy |
| Engine write, line **dirty in L2** | Write hit, bytes merge under strobes | **HW** for L2; **SW** `CINV` for any L1 copy |

Driver contract (sharpened for this dcache): `CPUSH` sources before
device-bound DMA and descriptors before `start`; **`CINV`, not `CPUSH`**,
on DMA-write destinations — this dcache's `CPUSH` leaves the line
valid+clean (`dcache.v:195-197`), so a post-DMA `CPUSH` yields stale reads;
`CINV` *before* starting a device→memory DMA into a CPU-dirtied buffer
(else later L1 eviction clobbers the DMA data); 32 B-aligned interior only,
head/tail fragments bounced by PIO (§13 F1).

### 6.2 Raw mode — deliberately none

No coherency, no probes, no cross-bus ordering.  Correctness rests on two
hardware-enforced facts: the per-client region clamp (§4.2) and the removal
of any system-bus decode of raw regions (§12 step R2, §13 F6).  A raw
client's data becomes SoC-observable only by the engine *copying* it into
system-bus space via a sys-mode transfer — which then takes the coherent
path like anything else.

### 6.3 The non-allocating bit — front-door delta, specified for the l2c owner

l2c today has **no non-allocating request notion** (§2.1).  This section is
the requirement handed to the l2c owners; we do not edit `l2c*.v`.

**Interface**: one sideband bit per request channel at the s_axi boundary
(`s_axi_ar_noalloc`, `s_axi_aw_noalloc` — the port list carries no
AxCACHE/AxUSER today, `rtl/soc/l2c.v:52-66`, so this is a new wire pair).
Carrying it from the engine through the xbar M3 fan-in to S0 adds one bit
to the hand-written fan-in payload — mechanical but real (*estimate*
50–150 LUT xbar-side).  An address-alias window ("no-alloc view of RAM")
was considered to avoid the xbar plumbing and rejected: every alias base in
reach is either probed by the Q700 ROM (superslots `0x9`–`0xE`) or already
meaningful, and address-encoding a *policy* invites exactly the
raw/no-alloc conflation §3 warns against.

**Semantics** (normative; "as today" = existing behaviour per
`docs/l2c_spec.md` §4–§6):

| Request | Lookup result | Behaviour |
|---|---|---|
| any, noalloc | **tag hit** | exactly as today — coherent hit, dirty data served/merged.  The probe is never skipped. |
| any, noalloc | miss, **MSHR match** (line in flight) | merge as secondary into that entry, as today — the line is arriving regardless; joining it is the coherent choice. |
| any, noalloc | miss, **victim-buffer hit** | stall until drain, as today (Critical-2 hazard rules unchanged). |
| **read**, noalloc | clean miss | issue the fill-shaped read but **do not install**: reuse the MSHR entry machinery with a `no_install` flag — respond to the requester from the entry's assembly buffer, then free; no way claimed, no PLRU touch, no victim eviction, no tag write. |
| **write**, noalloc | clean miss | **forward the write burst to the master port with its original byte strobes.**  No read-modify-write exists or is needed at any level: DDR honours per-byte strobes end-to-end (the MIG bridge forwards wstrb, `axi_ddr4_mig_bridge.v` narrow-traffic support), and RMW is only ever required when *installing* into a cache line — which noalloc, by definition, does not do.  Track the B; same-ID accept gating applies as today. |

What the bit saves: the fill (for writes), the eviction, the pollution.
What it never skips: the tag probe, the MSHR CAM, the victim-buffer hazard,
the install-history retry — i.e., all of coherency.

**Cost** (*estimates*, for the l2c owners to refine): `no_install` flag ×8
MSHR entries + install-path gating ≈ tens of LUT; the write-forward path is
the larger piece (an AW/W/B engine on the master-port arbiter, burst-level,
reusing the victim-writeback engine's idioms) ≈ 150–400 LUT inside
`l2c_ctrl`/`l2c_mshr` — congestion-contributor territory, hence **Phase 3**
(§12), landed by its owners at a phase boundary.

**Policy defaults** (resolves the v1 write-policy question as a per-transfer
choice):

- Default **allocate** — a disk sector landing in a File Manager buffer is
  read by the CPU microseconds later; allocation is the point.
- Drivers set **noalloc** for bulk streaming: transfer length above ~⅛ of
  L2 (≈256 KB, tunable) or known-streaming ops (disk-image copy, backup).
- **JTAG sys-view bulk reads default noalloc** (§7.3): a debugger dumping
  64 MB must not evict the working set it is trying to observe.
- RAM-disk **volume** traffic: not applicable — raw mode (§10).  The
  *sys-side* half of a RAM-disk-to-RAM transfer follows the length rule.

Until Phase 3 lands, everything allocates (v1 behaviour); the descriptor
bit is reserved from day one so drivers can set it before the hardware
honours it.

---

## 7. JTAG as a client — both views, and the independence boundary

### 7.1 What the user asked for

JTAG issues in **either mode, selected per transaction/window**: a **raw
DDR view** (bulk host transfers: `ramdisk-load`/`-save`, ROM staging,
disk-image blasting — today slow 256 KiB-batch single-beat traffic) and an
**AXI system-bus view** (what the CPU sees, through the coherent path).

### 7.2 Mode selection: address apertures — argued

Selection mechanisms considered: per-command mode registers (stateful —
a crashed script leaves the next session in the wrong mode; racy for
interleaved tooling) vs **address apertures** (stateless, self-describing,
and the existing `r`/`w`/`dump-mem` tooling keeps working **unchanged**
against the system view because the system view *is* today's address map).
Apertures win.  Concretely:

- **System view** = the existing map, via the existing M1 master —
  unchanged (and already coherent at L2, §7.3).
- **Raw view** = a dedicated aperture decoded on the *debug side* (in the
  `u_debug_jtag_axi`/`fpga_top_debug_host.vh` integration, before the
  xbar): accesses inside it are steered to an engine client port as
  `{raw, addr − aperture_base + window_base}` with a host-programmable
  `window_base` CSR into flattened-DDR space, engine-clamped per §4.2.
  Placement of the aperture is a detail (it must not collide with the Q700
  map; a high window such as `0x8xxx_xxxx`, unused by ROM/NuBus probing
  per the `axi_defs.vh:85-93` slot-space analysis, is the candidate) —
  *to be pinned at implementation time*.
- Bulk raw transfers use engine descriptors + real INCR bursts rather than
  per-word transactions; expected to be dramatically faster than today's
  path, though the JTAG serial link itself remains the physical floor
  (*unverified — measure, §15*).

### 7.3 The coherent-view payoff, and its precise limit (tooling contract)

Because M1 already lands on S0 and therefore passes through l2c when
`L2C_ENABLE` is set (§2.2), the system view — today's or the engine's — is
**coherent at L2**: a JTAG read sees data dirty in L2 without any cache
dance.  The `dcache-op push` folklore shrinks to a stated contract:

- **Visible without any host action**: everything that has reached L2 —
  all CPU stores whose lines have been written back or pushed out of L1,
  all engine sys-mode writes, all boot/debug writes.
- **Still invisible**: data dirty in **L1 only** — the 68040 does not
  snoop (architectural, §2.3).  Bounded staleness: L1-D is 4 KB total, so
  at most 4 KB of the freshest CPU stores can be hidden at any instant.
- **Host must still**: run `dcache-op push` (the debug-ctrl
  `flush_all_req` walker, `dcache.v:189-192, 217`) before reading data
  the CPU may have dirtied *recently* and not yet evicted — e.g. lowmem
  globals mid-session; and `dcache-op`-invalidate (or halt first) before
  *writing* memory the CPU may hold in L1, since a JTAG sys-view write
  updates L2 but not a stale L1 copy.
- These rules are identical for M1 and for the engine's sys view — moving
  bulk to the engine changes speed and pollution (noalloc, §6.3), not
  visibility.

### 7.4 The independence boundary — debug must not depend on the thing being debugged

Load-bearing precedent: the CSR `reset` command can wedge `dbg_axi` into
`0xBADA0BAD` sentinel reads, and recovery works **only because VIO rides
BSCAN independently of the dbg_axi path**
(`.claude/skills/m68k-jtag-wedge-recovery`, `docs/reset_story.md`).  The
same property must hold against a wedged *engine*.  Boundary, drawn
explicitly:

**Rides the engine** (bulk, latency-tolerant, only meaningful on a working
machine): `dump-mem`, `ramdisk-load`/`-save`, ROM/disk-image staging, any
multi-kilobyte transfer — both views.

**Never touches the engine** (the diagnose-a-wedged-machine set):
halt/reset control, `halt-status`, `dbg-caps`, `pc-trace`, exception-ring
reads, `dcache-op`, single `r`/`w` accesses — all stay on **M1 + the debug
CSR block + VIO**, whose wiring this design does not modify.  Keeping
single-word `r`/`w` on M1 (not the engine) means basic memory inspection
survives an engine wedge.

**Reachability when the engine is wedged** (the failure-mode statement the
coordinator asked for):

| Path | Engine wedged | Notes |
|---|---|---|
| VIO / `vio-hard-reset` (BSCAN) | **works** | independent of dbg_axi *and* engine — the recovery of last resort, unchanged |
| M1: debug CSRs (halt/reset/status/dcache-op/pc-trace) | **works** | requires xbar+debug block alive; engine not in path |
| M1: single `r`/`w`, small `dump-mem` fallback | **works** (slow) | S0/l2c path; engine not in path |
| M1 → S2: engine config registers | **works** | cfg register file must never gate on data-path progress — `dma_ctrl` already satisfies this (`docs/dma_ctrl.md` §2, §3 pause/abort); used to abort/reset the wedged channel |
| Engine bulk (both views) | dead until channel abort / `core_rst_bank[5]`-region reset | bounded by the per-channel watchdog (§4.3), which converts a wedge into an error completion |

Consequence for §5.3: **M1 is not freed.**  The engine gains JTAG as a
*bulk* client for speed and cache-policy control; the 1,337-LUT M1 path is
the price of debug independence and stays.  (If a future crunch demands the
area, the correct move is slimming `u_jtag_n2w` to the control subset —
not routing control through the engine.)

---

## 8. Ordering between modes — stated, not implied

Raw and sys are **different physical paths with independent queues** (raw:
tap below l2c; sys: through xbar+l2c).  The hardware provides **no
ordering between in-flight operations on different paths**, and — because
raw regions are disjoint from everything sys-addressable — no *data*
overlap can make that unordered-ness visible except through the engine
itself.  The one ordering token that exists:

> **A channel's completion (`done` after all its Bs/RLASTs are retired —
> the `dma_ctrl` FSM already defines `done` this way, `docs/dma_ctrl.md`
> §3) is the only cross-mode, cross-path ordering guarantee.**

Host/driver rule: to make a raw-mode result observable via the sys view
(or vice versa), (1) wait for the producing channel's `done`, then (2)
issue the consuming transfer.  Concretely for tooling: `ramdisk-save` must
poll the raw channel's completion before reading any of the data back
through the system view; interleaving without the token reads garbage
non-deterministically (§13 F12).  There is nothing subtler available and
nothing subtler should be promised.

---

## 9. Throughput, with numbers

Ceilings (cites in §2.2): fabric 1.6 GB/s; DDR/MIG ≈ 9.6/10.7 GB/s — DRAM
is never the binding term.

**Sys (coherent) mode** — bound by the l2c front door:

- Hit path: 16 B / 5.2-cycle measured round trip ≈ **~310 MB/s** at
  128-bit beats; ~155 MB/s through the Phase-2a 64-bit shim (per-*beat*
  cost).
- Miss path: the MSHR machinery sustains 512 B / 66 cycles ≈ **~775 MB/s**
  (measured) — but only across ≥2 AXI IDs; a single-ID miss stream
  serializes at one 64 B line per fill RTT ≈ **~110–130 MB/s** (*derived*).
  ⇒ the engine's sys port issues **64 B-aligned 4-beat bursts with 2–4
  rotating IDs**, never overlapping two in-flight ops on one line across
  IDs (cross-ID order is unconstrained, `docs/l2c_spec.md` §9; §13 F9).
- 512 B sector, buffer not resident: ≈0.7–1.2 µs (rotated-ID misses) to
  ≈1.7 µs (all-hit) — **~300–700 MB/s** (*derived*).
- Non-allocating (Phase 3) removes fill+evict from the write path —
  relevant to master-port occupancy and pollution, not to the front-door
  accept rate that binds here.

**Raw mode** — bound by burst shape × outstanding cap:

- Single 8-beat (128 B) burst, ~40-cycle RTT (*assumed*, §2.2):
  128/(40+8) ≈ 2.7 B/cy ≈ **267 MB/s**; with the full `MAX_BULK_AHEAD=2`
  pipelining ≈ **~500 MB/s**; 16-beat bursts ≈ **~450–700 MB/s**
  (*estimates*).  Against the deleted alternatives: bypass-window path
  **~40 MB/s**; and raw transfers cost the CPU nothing (no front-door
  slots).
- Scanout stays protected by construction (§4.2); whether 16- or 32-beat
  caps erode the 1.4× scan margin is precisely the Phase-0 measurement
  (§15).

**Context**: every current producer is far slower (SD/SPI ≈3 MB/s pacing,
`vhdd_ddr.v:96-100`; pseudo-DMA PIO order 2–10 MB/s, *estimate*, via the
53C96 handshake window `rtl/mac/glue.v:37-38, 219-225`).  The engine's
ceilings matter for the NVMe/GbE-class future, which is when Phase 3
earns consideration.

---

## 10. Client zero — the RAM disk, re-implemented on the engine

What was deleted: `vhdd_ddr` (private 512 B blkbuf + own AXI master + own
CDC + M3 seat + l2c bypass dependency).  What replaces it: a thin provider
speaking the unchanged `vhdd` contract to `scsi.v` (`rtl/vhdd.vh` — scsi.v
does not change), bound to an engine **raw** channel:

- **Config**: client entry `{mode=RAW, region = ramdisk carve-out
  (`AXI_DDR_RAMDISK_OFFSET` 0x5000_0000, ≤256 MB, `axi_defs.vh:194-214`),
  burst cap = 8–16 beats, QoS = latency-sensitive}` (SCSI request latency
  matters; volume traffic is short bursts).
- **READ(6)/(10)**: provider computes `addr = region_base + lba×512`,
  requests `len = n×512` from the engine; engine streams beats into the
  client FIFO; provider paces bytes to scsi.v per the contract's
  credit/handshake timing (`vhdd_ddr.v:109-116` — that half of vhdd_ddr's
  design carries over verbatim, including `wr_avail` gating and the ≥2-cycle
  byte handshake).  WRITE mirrors it.
- **Bounded response**: the provider keeps the vhdd watchdog obligation
  (`rtl/vhdd.vh` "bounded response"; `vhdd_ddr.v:48-77`); the engine's
  per-channel watchdog backs it.
- **Volume access from the host**: `ramdisk-load`/`-save` become JTAG
  **raw-view** transfers into the same carve-out (§7.2) — replacing the
  old CPU-visible `0x7000_0000` aperture, whose decode is removed (§12
  R2).  The volume is thereby fully *not SoC-observable*: exactly the raw
  model (§3).  Cross-client raw access to one region (JTAG + RAM-disk
  provider) is legal — raw regions may be shared *between engine clients*;
  ordering between them is the §8 completion token, enforced by tooling
  (don't `ramdisk-load` while the Mac has the volume mounted — same rule
  as today).
- **What it recovers**: RAM-disk function at raw-path speed (~hundreds of
  MB/s to the SCSI provider vs the old ~40 MB/s bypass ceiling — the SCSI
  byte-stream pacing then dominates), for the cost of a client port +
  channel instead of 1,337 LUT of private master+CDC.

---

## 11. Descriptors, config, interrupts

- **Descriptor model**: `dma_ctrl`'s 32 B chained descriptors carry over
  (`docs/dma_ctrl.md` §2.4).  Additions: `flags` bits for `NOALLOC`
  (§6.3, reserved from day one), `MODE` is *not* a descriptor field —
  mode is a property of the channel/client binding (§4.1), so a corrupt
  descriptor cannot flip a channel into the wrong address space.
  Descriptors are fetched through the **sys** port (they are
  SoC-observable state); drivers `CPUSH` the 32 B-aligned descriptor line
  before `start`.
- **Config window**: S2 (`0x5010_0000`) still decodes and is currently
  occupied by `vhdd_ctrl`, which null-responds outside its 8 registers
  (`fpga_top_dma.vh` history block).  Split with `rtl/soc/axil_split2.v`
  so the engine's register bank returns at its documented offsets — zero
  xbar changes.
- **Interrupts**: both wirings, both nearly free — (1) `irq_agg.rsvd_irq6`
  (CPU IPL 6): plumbing exists end-to-end, currently fed 0
  (`fpga_top_dma.vh`, `fpga_top_peripherals.vh:2339`) — bring-up/bare-metal;
  (2) **VIA2 NuBus slot IRQ** for Mac OS: PA5..PA1 (slots $E..$A) are
  idle/no-device (`rtl/mac/via2.v:7-11, 89-93`,
  `fpga_top_peripherals.vh:731-741`) — drive slot $E's sense line
  (synchronized, active-low) from the engine IRQ; a driver installs a Slot
  Manager handler (`SIntInstall`), the period-correct convention.

---

## 12. Re-cost and phased plan

Arithmetic against the freed area (all engine-side figures *estimates*
except `dma_ctrl`'s measured 2,024 LUT):

| Item | LUT |
|---|---|
| Freed now (vhdd_ddr + CDC) | **−1,337** |
| Freed at R2 (bypass disable, `BYP_WIN_EN→0` + decode removal) | **−213** |
| Engine core (`dma_ctrl` lineage, 4 ch) | +2,024 (measured, prior instance) |
| Client-port glue ×3 (RAM-disk, JTAG-raw, JTAG-sys) + QoS/clamps | +450–800 |
| Raw tap (2:1 + route FIFO at the l2c_m seam) | +400–800 |
| Sys attach: M3 rewire + `axi_n64_to_wide`-class shim (Phase 2a) | +~350 |
| JTAG raw-aperture decode + window CSR | +100–300 |
| **Net through Phase 2a** | **≈ +1.6k – +2.7k gross, ≈ +0.1k – +1.2k net** |

versus v1's ≈ +3k with no offsets.  Still nonzero on an 86.76 % device —
every phase lands only with a full route-validation window at a phase
boundary, and the fallback on a failed close is to hold, not to shave the
watchdogs/clamps.

**Phase 0 — measure (no Vivado, no RTL commitment).**  Two Verilator
experiments, §15.

**Phase 1 — raw path + client zero (smallest worthwhile step).**
Engine core + raw tap + RAM-disk provider client + JTAG raw aperture.
Zero l2c involvement, zero coherency machinery, no l2c-owner dependency.
Sequenced within the phase: **R1** land engine+tap with the carve-out
clamp; **R2** *atomically* remove the `0x7000_0000` xbar decode **and**
set `BYP_WIN_EN→0` (`fpga_top_ddr.vh:148`) — the aperture and the window
must die together (§13 F6); **R3** retool `ramdisk-load`/`-save` onto the
raw view.  Measurable: SCSI RAM-disk tests green again
(`ENABLE_DDR_RAMDISK` path retired), host-side transfer rate vs the old
path, scan margin unchanged under load (tb-vram-ddr-chain).

**Phase 2 — coherent mode.**  (a) Sys port onto M3 through the 64-bit
shim; descriptor engine + S2 split + IRQs; JTAG-sys bulk client;
directed coherency tb scenarios (DMA-read-sees-L2-dirty,
DMA-write-then-CPU-read-after-CINV, engine-wedge-reachability).  (b)
128-bit datapath + 2–4 rotating IDs (per §9); Mac OS driver / virtual
SCSI HBA (software long pole); VIA2 slot-$E IRQ.

**Phase 3 — l2c-owner work, only if Phase 0/2 measurements demand.**
Non-allocating bit per §6.3 spec (sideband through xbar fan-in + MSHR
`no_install` + write-forward path); PLRU insertion hint; front-door
pipelining (must preserve the §9 Critical-7 retry bound,
`docs/l2c_spec.md`).

---

## 13. What could go wrong — intermittent-corruption catalogue

(F1–F9 carried from v1 where still applicable, restated tersely; F6 and
F10–F13 are new/reshaped for the two-mode hub.)

- **F1 — partial L1 line at buffer edges** (sys mode).  32 B L1 lines vs
  arbitrarily-aligned File Manager buffers: `CINV` on a boundary line
  drops neighbouring dirty bytes; skipping it leaves stale bytes.  **DMA
  the 32 B-aligned interior only; PIO-bounce head/tail** — driver
  contract, enforced by the HBA shim, because it passes every aligned
  test and corrupts in the field.
- **F2 — `CPUSH` where `CINV` is required**: this dcache's `CPUSH` leaves
  lines valid+clean (`dcache.v:195-197`) → stale reads after DMA writes,
  only when the buffer happened to be L1-resident.  Contract wording + a
  directed test.
- **F3 — stale L1 dirty line evicted after the engine wrote L2** (missed
  pre-DMA `CINV`): lost update timed by unrelated CPU cache pressure.
  Contract: `CINV` *before* device→memory DMA; buffer off-limits while
  the channel runs.
- **F4 — ID discipline at M3**: the sys port's IDs live inside M3's XID
  space (xbar-tagged), so cross-master collision is structural non-issue;
  *within* the port, rotation must respect F9.
- **F6 — aperture/window must change together** (Phase 1 R2).  If the
  `0x7000_0000` decode outlives `BYP_WIN_EN`, CPU/JTAG access to the
  RAM-disk region starts **allocating in L2** (the tag covers all 32 bits,
  `docs/l2c_spec.md` §3) while the raw port writes the same DRAM behind
  l2c's back — stale lines and eviction clobbers, the exact corruption
  class of the Round-4 bypass bug (`l2c_bypass.v:144-164`).  Conversely,
  window-off + decode-on with no raw path yet = same hazard.  **One
  commit, both edits, plus an elaboration check in the engine that its
  raw regions are disjoint from the l2c cacheable span.**
- **F7 — reset with engine mid-burst**: the raw tap needs the mux3's
  `aw_open`-style reset-hold + bounded drain idioms
  (`axi_vram_priority_mux3.v` reset rules); the engine's
  never-start-unfinishable-bursts invariant (§4.3) keeps reset drains
  short.  The l2c bypass stray-B gap (`docs/l2c_spec.md` §9 Important-A
  note) stops mattering once R2 disables the window — one debt item
  *retired* by this design.
- **F8 — a wedged engine must not be a fabric hazard**: cfg registers
  never gate on data-path progress; per-channel watchdogs quiesce
  AXI-legally (§4.3); debug control is out-of-path entirely (§7.4 table).
- **F9 — cross-ID same-line overlap** (sys mode): l2c's cross-ID
  completion order is unconstrained (`docs/l2c_spec.md` §9); the engine
  never overlaps two in-flight ops on one 64 B line across IDs.
- **F10 — noalloc conflated with bypass** (Phase 3): a "skip the probe"
  implementation reads clean DDR under a dirty L2 line — silent, and only
  under the streaming workloads that set the bit.  The §6.3 table is the
  contract; the directed test is a noalloc read/write against a line made
  dirty via the CPU path.
- **F11 — one client wedging the shared hub**: a stalled client FIFO
  must never stall an in-flight AXI burst (§4.3 staging invariant) —
  otherwise one dead peripheral takes down RAM-disk *and* JTAG bulk.
  Watchdog completes the request with error; channel isolation is the
  whole point of burst-granular arbitration.
- **F12 — cross-mode interleaving without the completion token** (§8):
  host reads back, via the sys view, data whose raw-mode write has not
  `done`'d — nondeterministic garbage.  Tooling rule: poll `done` first;
  the REPL helpers should encode it, not document it.
- **F13 — raw-view region misprogramming from JTAG**: `window_base` CSR
  pointing raw bursts at cacheable DDR would corrupt behind L2 (F6 class).
  The engine clamp (§4.2) rejects it; a *deliberate* provisioning unlock
  (e.g. ROM staging into cacheable space while the CPU is reset-held and
  l2c freshly reset-walked) is possible but must be an explicit,
  logged CSR bit, defaulting locked — cold-boot staging via boot_fsm/M1
  already exists, so Phase 1 ships **without** the unlock.

---

## 14. Constrained by area/timing vs by architecture

**Area/timing (may become viable later):** non-allocating bit (l2c-owner
work, §6.3); PLRU insertion hints; front-door pipelining; 128-bit engine
datapath (Phase 2b); >2 raw-port outstanding (would need MAX_BULK_AHEAD
re-derivation against scan); shrinking the xbar picker; slimming the M1
path to a control-only subset.

**Architecture (will not change):** the 68040 L1 never snoops — the
`CPUSH`/`CINV` contract and F1's bounce rule are permanent, and at most
4 KB of freshest CPU stores are invisible to any coherent observer; the
raw mode is *defined* as incoherent and unordered — no future work makes
raw data SoC-consistent except copying it through the sys view; everything
SoC-observable serializes through the one l2c front door — coherent
bandwidth is front-door bandwidth; AXI same-ID ordering + l2c's accept
gate make ID rotation the only route to miss overlap; scanout's
hard-realtime bound lives in the mux3 burst quota — no new source may be
added to it; debug-path independence (VIO/BSCAN + M1 control) is
load-bearing and the engine must never enter that path.

---

## 15. Recommendation and the measurement to take first

**Build the hub as specified**: shared engine (`dma_ctrl` lineage) with
per-client mode binding; **raw port** as a 2:1 tap at the
`l2c_m → axi_vram_priority_mux3` seam with hardware region clamps; **sys
port** on the freed **M3 seat** (coherent at L2 by construction, no l2c
edits); RAM disk as client zero on the raw path; JTAG bulk as a client in
both views behind stateless address apertures, with **all debug control
remaining on M1/VIO**; `NOALLOC` reserved in descriptors now, implemented
in l2c by its owners in Phase 3 per the §6.3 spec.  Net cost through the
coherent phase ≈ **+0.1k–1.2k LUT** after the freed area (*estimates*),
touching no congestion-report module before Phase 3.

**Single most valuable measurement** (Phase 0a, pure Verilator, gates
Phase 1): extend `tb-vram-ddr-chain`'s concurrent-streaming scenario with
a raw-port traffic source at the l2c_m seam — sweep burst length
{8, 16, 32 beats} × outstanding {1, 2} against active scanout at
1024×768×24bpp with `STALL_ENABLE=1` jitter, and record (a) scan-side
underflow margin and (b) raw throughput.  This pins the raw burst cap with
the scan guarantee *demonstrated* rather than argued, and replaces the
~40-cycle-RTT assumption every §9 number inherits.  Second (Phase 0b,
gates Phase 2): the v1 front-door experiment — DMA-shaped 64 B bursts
through `tb_l2c`, sweeping 1→8 rotating IDs against CPU-shaped traffic —
to confirm the ~310 MB/s hit / ~775 MB/s miss picture before any coherent
plumbing lands.
