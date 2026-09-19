# dma_ctrl — programmable multi-channel AXI-master DMA engine

Phase-3.5 foundation for virtual peripherals (NVMe-as-SCSI, 1GbE-as-
NuBus, USB-as-ADB, etc.) per `docs/clocking.md` §13 and the virtual-
peripherals umbrella plan.  The engine pushes bulk data between main
memory and peripherals, or between two memory regions, without
stalling the CPU fetch/issue pipeline.

File: `rtl/sys/dma_ctrl.v`  (≤ 800 lines, Verilog-2005, lint-clean).
Unit testbench: `tb/tb_dma_ctrl.cpp` — 10 scenarios, `make tb-dma-ctrl`.

---

## 1. Scope + design choices

### 1.1 Locked-in choices

| Decision | Value | Rationale |
|---|---|---|
| Channels (`N_CH`) | **4** | Phase-3.5 need (one each for NVMe-ish SCSI, 1GbE RX/TX rings, audio playback).  `N_CH=8` is a one-line parameter change; leaves headroom for H1. |
| AXI master width (`DATA_WIDTH`) | **64 bits** | Matches the task spec; smaller than the 128-bit system xbar to keep the arithmetic unit narrow.  Integration through `axi_narrow_to_wide`-class adapter (see §4). |
| Burst shape | INCR up to 16 beats | 16 beats × 8 B = 128 B per burst — KU5P-friendly and well below AXI4's 256-beat cap.  Channel FSM cuts transfers at 4 KB boundaries (AXI4 rule). |
| Arbitration | Channel-round-robin | Simple fairness; starvation is impossible with 4 requesters.  Bus-phase holding: once a channel wins AR, it retains the master until the read burst's RLAST (then the arbiter re-picks for the following AW). |
| Descriptor size | **32 bytes**, fetched as 4 × 64-bit beats | A nice power-of-two. |
| Alignment | long-aligned (4-byte) only (P0) | byte-level misalignment → P2. |
| IRQ line | level, OR of per-channel `(done|error) & irq_en` | Feeds `irq_agg.v` on one of the free IRQ slots (recommend `rsvd_irq6` — see §5). |
| Cache coherence | software-managed, `CPUSH`/`CINV` around DMA | Consistent with 68040 convention + Mac OS driver expectation.  No hardware snooping at phase 3.5 (see `docs/clocking.md` §13.3). |

### 1.2 Not in scope

- Scatter-gather with multiple tuples per descriptor — we achieve the
  same effect via a descriptor CHAIN (one tuple per descriptor).
- 2-D / strided transfers — P2.
- Byte-level misalignment — P2 (current: `CH_SRC`/`CH_DST`/`CH_LEN`
  must be long-aligned).
- Preemption — a mid-burst channel owns the master until the burst
  completes.  Channels round-robin between bursts, so the longest
  starvation is one burst (≤ 128 B / ≈ 20 cycles at 200 MHz).
- Encryption / hashing side-channels — H1+.

---

## 2. Register map (AXI-Lite slave, 20-bit byte address)

### 2.1 Global

| Offset | Name | Access | Fields |
|---|---|---|---|
| `0x000` | `DMA_CTRL`    | R/W  | `[0]` global_enable; `[1]` irq_enable |
| `0x004` | `DMA_STATUS`  | R/O  | `[3:0]` channel_busy; `[11:8]` channel_done; `[19:16]` channel_error; `[24]` global_irq |
| `0x008` | `DMA_IRQ_CLR` | W1C  | write `1<<n` to clear `CH[n]`'s sticky done/error |

### 2.2 Per-channel block — 4-bit slot per channel at `0x100 + ch*0x40`

| Offset | Name | Access | Fields |
|---|---|---|---|
| `0x00` | `CH_CTRL`     | R/W | `[0]` enable; `[1]` start (W1S); `[2]` desc_mode; `[3]` irq_en; `[7:4]` burst_beats_log2 |
| `0x04` | `CH_STATUS`   | R/O | `[0]` busy; `[1]` done; `[2]` error; `[3]` desc_active |
| `0x08` | `CH_SRC`      | R/W | source address (byte, long-aligned) |
| `0x0C` | `CH_DST`      | R/W | destination address (byte, long-aligned) |
| `0x10` | `CH_LEN`      | R/W | remaining bytes (auto-decrement during transfer) |
| `0x14` | `CH_DESC_PTR` | R/W | physical address of first descriptor |
| `0x18` | `CH_NEXT_DESC`| R/O | live next-descriptor pointer (from current descriptor) |
| `0x1C` | reserved      | —   | — |

### 2.3 CH_CTRL.burst_beats_log2 encoding

`0` → 1 beat;  `1` → 2;  `2` → 4 (DEFAULT);  `3` → 8;  `4` → 16.  Values
5..15 clamp to 16.  Per-channel; bigger bursts trade latency against
per-beat efficiency.

### 2.4 Descriptor layout (32 bytes, long-aligned)

| Byte offset | Size | Field |
|---|---|---|
| 0  | 4 | `src` |
| 4  | 4 | `dst` |
| 8  | 4 | `length` (bytes) |
| 12 | 4 | `flags`  (bit 0 = `last_of_chain`, bit 1 = `irq_on_complete` — reserved, currently gated only by `CH_CTRL.irq_en`) |
| 16 | 4 | `next_ptr`  (0 = end of chain) |
| 20 | 12 | reserved |

Descriptors are fetched as 4 × 64-bit AXI beats.

---

## 3. FSM (per channel)

```
  IDLE ─ start ─► (desc_mode?) ────────────────────────────────────
                     │yes                                          │
                     ▼                                             │
                DESC_AR ──►  DESC_R ─ rlast ─┐                     │
                                             ▼                     │
                                           READY ◄─────────────────┘
                                             │  ch_len==0 → SEG_DONE
                                             ▼  else plan seg_bytes+seg_beats
                                          RD_AR ── AR handshake ─► RD_R
                                                                    │ rlast
                                                                    ▼
                                                                 WR_AW ── AW ─► WR_W
                                                                                │ wlast
                                                                                ▼
                                                                               WR_B
                                                                                │ bvalid
                                                                                ▼
                                                                           SEG_DONE
  DONE ◄── SEG_DONE (last seg & no chain)
  ERROR ◄── any RRESP/BRESP != OKAY
  PAUSED ◄── !g_enable OR CH_CTRL.enable going low (hit between bursts)
```

Key invariants:

- An AR that's accepted at the same posedge as a pause request **must**
  transition to the R state (not PAUSED).  Otherwise the slave holds a
  pending burst that we never collect.  Enforced in `S_DESC_AR` and
  `S_RD_AR` by checking AR-accept first, pause-check second.
- All data-phase bursts (RD_R, WR_W, WR_B) complete atomically — the
  FSM does not pause mid-burst.
- `CH_LEN` decrements by `seg_bytes` on each `SEG_DONE`.  When it
  reaches 0 AND there is no further descriptor in the chain, channel
  enters `DONE`.

---

## 4. Integration

### 4.1 Phase-3.5 — UNCONNECTED in mac_top.v

For this task, `dma_ctrl` lives standalone.  The `tb-dma-ctrl` unit
testbench exercises it against a memory-model AXI slave.  `mac_top.v`
does **not** instantiate `dma_ctrl` yet.  Reason: the current 2M×2S
(soon 3M×3S per task #19 / 4M×3S per `docs/clocking.md` §13.7) system
xbar has no spare master port.

### 4.2 Phase-3.5+ — DMA integrated through 5M × 3S xbar (task #116 LANDED)

Status table (`xbar-retune-3s + dma-ctrl-wire`):

| Item                                               | Status |
|----------------------------------------------------|--------|
| Xbar retuned to 5 masters × 3 slaves               | ✓ wired |
| dma_ctrl master → `axi_n64_to_wide` → xbar M4      | ✓ wired |
| dma_ctrl cfg AXI-Lite ← `axi_wide_to_axilite` ← S2 | ✓ wired |
| Config window carved at `0x5010_0000..0x501F_FFFF` | ✓       |
| `dma_ctrl.irq` → `irq_agg.rsvd_irq6` (level 6)     | ✓ IRQ L6 |
| `rtl/fpga_top.v` integration                       | ✓       |
| `tb-dma-integration` end-to-end scenarios          | ✓       |
| `tb-axi-xbar` extended to 5M×3S + DMA scenarios    | ✓       |

Concrete wiring that landed:

1. **Xbar retune (`rtl/sys/axi_xbar.v`)** widened to 5 masters + 3 slaves.
   M4 = DMA (read+write); S2 = DMA config.  DMA window is a 1 MB hole
   inside the 16 MB I/O range — xbar's `decode_slv` checks DMA before
   IO so `0x5010_0000` routes to S2 while the rest of `0x5xxx_xxxx`
   routes to S1/peripheral_bus.
2. **64→128 master adapter (`rtl/sys/axi_n64_to_wide.v`)** — narrow-burst
   shim forwarding each 64-bit beat as a half-lane transfer on the
   128-bit xbar side; preserves `awlen`/`arlen` unchanged; shifts
   wstrb to the correct 64-bit lane per beat by address[3].
3. **128→AXI-Lite cfg bridge (`rtl/sys/axi_wide_to_axilite.v`)** — xbar
   S2 is 128-bit AXI4; dma_ctrl's `cfg_*` is 32-bit AXI-Lite (20-bit
   byte address).  Bridge handles single-beat lane slicing
   (`awaddr[3:2]` → 32-bit lane) and terminates B/R responses.
4. **IRQ (`rtl/fpga_top.v`)** — `dma_ctrl.irq` feeds `irq_agg.rsvd_irq6`,
   CPU IPL level 6 (above Sound DMA, below NMI) per §5 recommendation.
5. **Mac OS driver side** — still phase-5 software work (INIT + virtual
   SCSI HBA); unchanged from original plan.

### 4.2b 2026-07-16 — M4 master port STUBBED (crossbar master-count reduction)

Status as of this session: `dma_ctrl`'s data-movement AXI4 master (M4
above) still had **zero live consumers** — nothing in the design has
ever triggered a channel start.  As part of a broader crossbar
master-count reduction (5M → 3M: CPU LSU merged with boot FSM behind a
`cpu_held_in_reset` mux, CPU IF renumbered M2), M4 was removed from
`axi_xbar.v` entirely rather than kept as a permanently-idle port:

| Item                                                | Status |
|------------------------------------------------------|--------|
| M4 port on `axi_xbar.v`                               | REMOVED |
| `axi_n64_to_wide` bridge (`rtl/soc/fpga_top_dma.vh`)  | REMOVED |
| `dma_ctrl.m_axi_*` (narrow master side)               | tied to a permanently-idle AXI slave (never ready, never valid) |
| `dma_ctrl.cfg_*` (AXI-Lite config, S2)                | **unchanged, still live** — any master can still program descriptors |
| `dma_ctrl.v` itself                                   | **untouched**, still unit-tested via `tb-dma-ctrl` |
| `tb-axi-xbar` M4 scenarios (12/14/15)                 | `#if 0`'d out (not deleted — see file), historical reference for reviving M4 |

If `dma_ctrl` is ever configured and started with nothing consuming this
port, its AW/AR channel will simply sit with valid asserted and no
grant forever — a benign hang confined to the (currently unused) DMA
engine, not a fabric-wide hazard.

**To revive**: re-add the M4 port to `axi_xbar.v` (mirror the removed
write/read fan-in slot-3 wiring, still present as commented-out-in-spirit
tie-offs — see the file's inline comments), restore the
`axi_n64_to_wide` bridge in `fpga_top_dma.vh`, and un-guard the
`#if 0` blocks in `tb/tb_axi_xbar.cpp`.

### 4.3 Cache-coherence contract for drivers

Per `docs/clocking.md` §13.3 and the 68040 manual: our D-cache is
write-back with software-managed coherence.  Drivers must:

- `CPUSH` (write-back) every line in the **source** region before
  starting a memory-to-peripheral DMA so the DMA reads current data.
- `CINV` (invalidate) every line in the **destination** region after
  completing a peripheral-to-memory DMA so the CPU sees fresh data.
- Intra-memory DMA (mem→mem) needs both.
- This is the standard Mac OS SCSI-DMA / NuBus-DMA convention; zero
  custom code.

---

## 5. IRQ wiring recommendation

`dma_ctrl.irq` is a level line, held high while any channel has
`(done | error)` set AND its per-channel `irq_en` AND the global
`irq_enable`.  Cleared by writing the channel's bit to
`DMA_IRQ_CLR`.

Recommended IRQ-aggregator slot: **level 6** (currently tied off in
`irq_agg.v` as `rsvd_irq6`).  This keeps levels 1–5 for the classic
Mac peripherals:

| Level | Vec | Source |
|---|---|---|
| 1 | 25 | VIA1 (60 Hz VBL, ADB, RTC, sound command) |
| 2 | 26 | VIA2 (NuBus slot IRQ aggregate) |
| 3 | 27 | SCSI (NCR 5380 end-of-command) |
| 4 | 28 | SCC (serial RX/TX ready) |
| 5 | 29 | Sound DMA done |
| 6 | 30 | **DMA engine (proposed)** |
| 7 | 31 | NMI |

Level 6 lets the CPU mask DMA IRQs independently of the sound path
and is the highest-priority maskable IRQ — appropriate for fast
virtual-peripheral completions.

---

## 6. Performance & cost estimates (not-yet-synthesised)

- Channels: 4 × per-channel state = ~200 FF.
- Buffer: 4 channels × 16 beats × 64 bits = 4096 FF (or small BRAM if
  we re-target).  Unclear which synth tool picks — both are OK.
- Descriptor buffer: 4 channels × 4 beats × 64 bits = 1024 FF.
- Register bank + FSMs + arbiter: ~500 FF.
- Total ≈ 6 K FF + minor LUT logic; negligible vs the core.
- Fmax: single-always sequential block.  Longest path is the
  segment-planner function (a few chained comparisons and subtracts)
  — well under 5 ns at KU5P.  No DSP or BRAM in the critical path.

---

## 7. Testing

`make tb-dma-ctrl` runs 10 scenarios (`tb/tb_dma_ctrl.cpp`):

1. Single 512 B memcpy — baseline.
2. Burst boundary — src spanning a 4 KB boundary; FSM cracks the
   transfer into sub-4K bursts.
3. Two channels concurrent — interleaved RR arbitration.
4. Descriptor chain — 3 descriptors chained, single "go".
5. AXI SLVERR on read — channel flags error, IRQ fires, IRQ_CLR
   drops the line.
6. Back-pressure — slave holds ready low for 50 cycles mid-transfer;
   transfer still completes.
7. Global `DMA_CTRL.global_enable` drop mid-transfer — channels
   pause cleanly at a burst boundary; re-enable resumes.
8. Channel `CH_CTRL.enable` drop during descriptor chain — channel
   halts cleanly.
9. Tiny 8-byte (single-beat) transfer.
10. Config register readback.

All 10 pass.  See `tb/tb_dma_ctrl.cpp` for the exact scenario code.

---

## 8. Follow-up tasks

- **`dma-ctrl-wire`** (new) — integrate `dma_ctrl.v` as a 3rd/4th xbar
  master, wire its AXI-Lite config into peripheral_bus, wire its IRQ
  into irq_agg.  **Depends on `#19 axi-xbar-vram` retune.**
- **`dma-sg-2d`** (new, P2) — extend descriptor to support 2-D
  strided transfers (stride + num_rows fields in the reserved word).
  Useful for framebuffer blitter and packetised network RX.
- **`dma-byte-align`** (new, P2) — lift the long-alignment
  restriction.  Complication: sub-beat byte-lane masking of the first
  and last beats.  Doable but not required for the current virtual-
  peripheral targets.
- **`dma-snoop`** (new, H2) — hardware snoop of DMA writes against
  the D-cache tag array; removes the software CPUSH/CINV requirement.
  Depends on a real coherence protocol upgrade.

---

## 9. Quick reference

```c
/* Software programming pattern — direct (no descriptor chain) */
static void dma_memcpy(uint32_t ch, uint32_t src, uint32_t dst, uint32_t len) {
    /* 1. CPUSH source, CINV dest — driver duty */
    __asm__("cpusha dc"); /* simplified */
    /* 2. Program */
    mmio_write(DMA_BASE + 0x000, 0x1);          /* DMA_CTRL: global_enable */
    mmio_write(DMA_BASE + 0x100 + ch*0x40 + 0x08, src);
    mmio_write(DMA_BASE + 0x100 + ch*0x40 + 0x0C, dst);
    mmio_write(DMA_BASE + 0x100 + ch*0x40 + 0x10, len);
    /* 3. Kick: enable | start | burst=16 beats */
    mmio_write(DMA_BASE + 0x100 + ch*0x40 + 0x00, (1u<<0)|(1u<<1)|(4u<<4));
    /* 4. Poll status */
    while ((mmio_read(DMA_BASE + 0x100 + ch*0x40 + 0x04) & 0x2) == 0) ;
    /* 5. Clear IRQ */
    mmio_write(DMA_BASE + 0x008, 1u << ch);
}
```
