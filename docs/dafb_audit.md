# DAFB Register-Set Audit — Q700 Mac-Logo Boot Path

**Date:** 2026-04-21
**Scope:** `rtl/mac/video.v` + `rtl/mac/video/*.v` vs. what the Quadra 700
Universal ROM (`files/420dbff3.rom`) touches during boot-to-logo.
**Result:** We now implement the DAFB **front-door shim** plus the first
live framebuffer-placement hooks, but not a complete DAFB.  The old
"dead register window" blocker is gone: `rtl/mac/video.v` ACKs and stores
the ROM's `0xF9800000..0xF98003FF` register traffic, with deterministic
status/sense readbacks.  The `+0x20` status register now exposes a
synthetic sticky vblank bit that is armed only after base, stride, and raw
depth selector have all been programmed; write-one-to-clear acknowledges it.
The ROM's base/stride/BPP writes at `+0x08`, `+0x0C`, and `+0x10` now latch
into exported scanout state, and
`tb-dafb-scanout` proves a DAFB-programmed `0x100` VRAM base plus
`0x61E` stride can steer scaler reads to CPU-written bytes that emerge as
8bpp indexed pixels through the low-depth CLUT.  `fpga_top` now routes
the same register window through xbar S4 so CPU and host AXI masters feed
the same live latches into `video_top`; `tb-scanout-placement-sync` checks that
the pclk-domain snapshot does not change mid-frame and that zero stride falls
back to the source width.
Remaining visible-logo gaps are CLUT/timing/VBL fidelity, exact source-window
selection for the ROM's programmed mode, and the ROM reaching the paint path.
**Data source for "what the ROM does":**
`/tmp/mame_scout/vram_writes.log` (MAME `macqd700` capture of every
write into the `0xF9xxxxxx` window during ROM init), cross-checked
against `/tmp/mame_scout/q700_full.dis` and `rtl/mac/glue.v`.
MAME `src/mame/apple/dafb.cpp` was **not** available on this host
(`find / -name "dafb*.cpp"` returned nothing; the Debian `mame-data`
package ships ROM metadata only), so the real-DAFB column is
inferred from the ROM's write pattern + Apple DAFB documentation +
MAME trace semantics, not from authoritative MAME source.

---

## 1. Our DAFB register inventory

`rtl/mac/video.v` is a simple AXI4-lite DAFB register shim:

- 256 longwords, selected by `addr[9:2]`, covering the ROM-touched
  `0xF9800000..0xF98003FF` range.
- Writes ACK in one cycle and merge by `WSTRB`.
- Reads return last-written values, except:
  - `+0x20` IRQ/status returns sticky synthetic vblank in bit 0 and an
    IRQ-enable-gated pending indication in bit 1.
  - `+0x200` / `+0x220` monitor-sense reads return `0x7`.
- The shim exports live base/stride/BPP latches for the scanner path.
  It still has no IRQ output, no full RAMDAC model, and no timing
  programmability.

The `rtl/mac/video/` subtree is the HDMI-out chain
(`video_top.v` / `vtg.v` / `scaler.v` / `fb_reader.v` /
`mmcm_hdmi.v` / `i2c_init.v` / `vram_smoke.v`).  It free-runs on `pclk`,
reads a linear framebuffer from URAM-backed `vram`, and scans out a
parameterized source window to a parameterized HDMI mode.  The production
`fpga_top` build uses an 8bpp 1024x768 source letterboxed into 1920x1080p60;
DAFB base/stride writes update the scanout address placement at runtime,
but not the scaler geometry or HDMI timing parameters.

`tb/tb_dafb.cpp` now explicitly exercises the boot-time control latches
at `+0x08`, `+0x0C`, and `+0x10`, the 16-entry CLUT window at
`+0x300 + n*0x10`, and the existing status / sense paths, so regressions
in the ROM's first-light register traffic are caught in unit sim before
ROM boot.  `tb/tb_dafb_scanout.cpp` then wires those latches into a small
scaler instance and proves ROM-style VRAM bytes at `base + y*stride + x`
can appear at scanout through the programmed CLUT.  `tb-vram-scaler-firstlight`
keeps the DAFB shim in the VRAM/core clock domain while the scanner runs in
pclk, so the preflight now exercises the same register-to-scanout CDC split
used by production `video_top`; it also logs and checks the first real VRAM
AXI byte write and the first scanout-side VRAM read.

`rtl/mac/glue.v` still classifies the full `0xF9xxxxxx` range as video
for observability, but `cs_video` is now asserted only for the DAFB
register window `0xF9800000..0xF9800FFF`.  The `0xF9000000..0xF90FFFFF`
pixel aperture is routed through the xbar/VRAM path instead.
`tb/tb_rom_boot.cpp` leaves the register window to the in-RTL shim and
treats the VRAM aperture as RAM-like storage for boot bring-up.

Net inventory:

| Offset | Width | R/W | Effect on scanner |
|--------|-------|-----|-------------------|
| `0x000..0x3FF` | 32-bit | R/W | Stored/echoed; `+0x08/+0x0C/+0x10` export live scanout placement |
| `0x020` | 32-bit | R/W1C | Bit 0 is sticky synthetic vblank status; bit 1 mirrors bit 0 through IRQ enable bit 0; armed by non-zero base, stride, and BPP |
| `0x200`, `0x220` | 32-bit | R | Hard-wired monitor sense `0x7` |
| `0x300..0x3F0` | 32-bit | R/W | Low-nibble CLUT entries feed the 8bpp scaler palette |

---

## 2. Q700 DAFB register set — what the ROM actually touches

The ROM boot trace (`PC=0x5B1C…0x6198`, all in the ROM-overlay
mirror) writes the following **unique** DAFB register offsets in the
`0xF9800000` base window.  Grouped by functional block:

**Core control (0xF9800000 + 0x00..0x24)** — display enable, base addr,
mode:

    F9800000  .L  writes: 0x00000008               -- "MON_ID" / display-sense read-back? (also written 0x08 during init)
    F9800004  .L  writes: 0x00000000
    F9800008  .L  writes: 0x00000100               -- framebuffer base offset (in VRAM, 0x100)
    F980000C  .L  writes: 0x00000000, 0x0000061E   -- framebuffer row stride / base (1566 = 1024*8bpp/8 + pad? see §4)
    F9800010  .L  writes: 0x00000000, 0x00000030   -- depth / BPP select (0x30 = 8bpp on real DAFB)
    F980001C  .L  writes: 0x00000007               -- interrupt enable / clear
    F9800020  .L  writes: 0x00, 0x02, 0x03, 0x07   -- VBL IRQ status / ack (polled 5× in a row at PC 5BE0..5C4C)
    F9800024  .L  writes: 0x000001EC               -- written first, before any other DAFB reg (PC 4089914E)

**Timing block (0xF9800100 + 0x00..0x68)** — pixel-clock PLL +
horizontal/vertical timing generator load:

    F9800100  .L  writes: 0x00000001, 0x00000FF2   -- PLL M/N / clock-select (first programs 0001, then 0xFF2 = final)
    F9800104  .L  writes: 0x00000000               -- PLL aux / dot-clock divisor
    F9800124  .L  writes: 0x0000026E, 0x0000031E   -- h_total
    F9800128  .L  writes: 0x00000190, 0x000001B0   -- h_active  (0x190 = 400?  see §4)
    F980012C  .L  writes: 0x00000020, 0x00000030   -- h_front_porch
    F9800130  .L  writes: 0x0000031F, 0x0000035F   -- h_sync_end
    F9800134  .L  writes: 0x0000003F, 0x0000005F   -- h_back_porch
    F9800138  .L  writes: 0x0000004B, 0x0000006B   -- v_total
    F980013C  .L  writes: 0x0000006F, 0x00000083   -- v_active
    F9800140  .L  writes: 0x00000088, 0x00000098   -- v_front_porch
    F9800144  .L  writes: 0x00000308, 0x00000318   -- v_sync_end
    F9800148  .L  writes: 0x0000031E, 0x0000035E   -- v_back_porch
    F980014C  .L  writes: 0x0000041A               -- blanking / composite-sync
    F9800150  .L  writes: 0x00000418               -- burst / colour-key
    F9800154  .L  writes: 0x00000002, 0x00000004   -- misc timing
    F9800158  .L  writes: 0x00000007, 0x00000009
    F980015C  .L  writes: 0x00000044, 0x00000052
    F9800160  .L  writes: 0x00000404, 0x00000412
    F9800164  .L  writes: 0x00000408, 0x00000416
    F9800168  .L  writes: 0x0000002A, 0x0000003A

Two distinct value-sets visible across each timing register ⇒ ROM
first programs one mode (probably 640×480), reprograms to the Apple
13"/14"/16" sensed mode, then enables the scanner.

**Monitor-ID / sense (0xF9800200, 0xF9800220)** — 3-bit
monitor-sense lines that identify the attached display; probed
multiple times before and after mode programming:

    F9800200  .L  writes: 0x00, 0x01              -- sense drive enable
    F9800220  .L  writes: 0x00, 0x06, 0x80        -- sense tri-state / sense data latch

**CLUT / palette (0xF9800300 + 0x00,0x10,0x20,…,0xF0)** — 16-entry
strided palette register set (RAMDAC index+RGB).  Writes at PCs 5B78
and 6184; second pass updates a different value set:

    F9800300  .L  writes: 0x0E, 0x0F
    F9800310  .L  writes: 0x01, 0x0F
    F9800320  .L  writes: 0x00, 0x01
    F9800330  .L  writes: 0x00
    F9800340  .L  writes: 0x05, 0x09
    F9800350  .L  writes: 0x01, 0x03
    F9800360  .L  writes: 0x00
    F9800370  .L  writes: 0x00
    F9800380  .L  writes: 0x00
    F9800390  .L  writes: 0x02, 0x03
    F98003A0  .L  writes: 0x05
    F98003B0  .L  writes: 0x06
    F98003C0  .L  writes: 0x04
    F98003D0  .L  writes: 0x01
    F98003E0  .L  writes: 0x00
    F98003F0  .L  writes: 0x00

Stride = 0x10 per entry, 16 entries → fits "low-depth CLUT" or a
cursor colour cache; values are all small integers (≤ 0x0F), which
is consistent with a 4bpp indexed mode being set up before the
8bpp mode lands.

All writes are `.L`-sized full-word (`mask=FFFFFFFF`), aligned to
4.  No DAFB reads were captured in the periph logs (`periph*.log`
size 0) — we do not yet know the full read-back contract, but the
ROM polls `F9800020` as a VBL status register (5 back-to-back
reads with tightly-spaced PCs).

VRAM (not DAFB register) writes are at `0xF9001xxx..0xF91Fxxxx`,
i.e. **VRAM lives at `0xF9000000 + 0x100` base offset** — matches
the `F9800008 ← 0x100` configuration.

---

## 3. Gap table (register-by-register, mac-logo impact)

| Offset | Inferred name | Real-DAFB effect | Our model today | Mac-logo impact |
|---|---|---|---|---|
| `0xF9800000` | Board-ID / reset | Resets scanner FSM | Stored/echoed | **M** - no scanner reset side effect |
| `0xF9800004` | ? | zero-init | Stored/echoed | S |
| `0xF9800008` | FB base offset | sets VRAM base of visible framebuffer | Latched/exported; production scanout consumes via CDC snapshot | **S** - exact ROM source window still static |
| `0xF980000C` | Row stride | pixels-per-line stride | Latched/exported; production scanout consumes via CDC snapshot | **S** - exact ROM source window still static |
| `0xF9800010` | BPP / depth | 0x30 = 8bpp, 0x20 = 4bpp, 0x10 = 2bpp, 0x00 = 1bpp | Latched/exported raw; scanout remains elaborated as 8bpp for first light | **M** - no runtime format switch yet |
| `0xF980001C` | IRQ enable | arms VBL IRQ line | Stored/echoed; bit 0 gates status bit 1 | **M** - no external IRQ side effect |
| `0xF9800020` | IRQ status / ack | W1C of VBL-pending | Sticky synthetic vblank in bit 0, enable-gated pending in bit 1, lane-valid W1C ack | **M** - poll-visible, but not scanner/VIA-driven |
| `0xF9800024` | ?? (first write) | unclear - possibly VRAM config / PLL init | Stored/echoed | **M** - first ROM DAFB write has no hardware effect |
| `0xF9800100` | PLL control | programs pixel clock M/N | Stored only | **M** - clock not runtime-programmable |
| `0xF9800104` | PLL aux | dot-clock divisor | Stored only | M |
| `0xF9800124..168` (18 regs) | H/V timing + blanking | full programmable VTG for Apple 13"/14"/16"/21" modes | Stored only; HDMI geometry is parameterized but static | **L** - scanner does not honour Mac-mode timings |
| `0xF9800200` | Monitor-sense drive | drives sense pins low to measure ID | Reads return `0x7` | **S** - enough for deterministic monitor detect, not electrically faithful |
| `0xF9800220` | Monitor-sense data | tri-state + sample | Reads return `0x7` | **S** - same |
| `0xF9800300..3F0` (16 regs) | CLUT / palette index+RGB | loads RAMDAC LUT | Low-nibble indexed palette feeds 8bpp scanout | **S/M** - enough for first colours, not full RAMDAC format |

The register access blocker is fixed. **The remaining gaps are live
effects from the stored registers.**

---

## 4. Framebuffer-format audit

- **Q700 ROM configures:** BPP ≈ 0x30 (likely 8bpp indexed) per
  `F9800010 ← 0x30`; framebuffer base ≈ `0x100` inside VRAM per
  `F9800008 ← 0x100`; row stride ≈ `0x61E` (1566) per
  `F980000C ← 0x61E`.
- **Our scanner expects:** the production `fpga_top` instantiates
  `video_top`/`vram` at 8bpp to fit the KU5P URAM budget.  The raw
  `F9800010` value is logged and exported as `fb_bpp_reg`, but scanout is
  still elaborated as the 8bpp first-light path.  `scaler.v` treats the
  byte as an indexed colour and maps `idx[3:0]` through the live DAFB CLUT
  exported by `rtl/mac/video.v`.  This is still a first-light subset of the
  RAMDAC format, not a full DAFB palette model.
- **Source dimensions:** our scaler is parameterised 1024×768; the
  ROM's first mode programming looks closer to 640×480 (H_active
  0x190 = 400 ≠ 640, so this is in DAFB-internal timing units, not
  pixels — exact units unknown without MAME source).  Our timing
  block accepts parameter overrides (post-`76fd86b`) but not runtime
  DAFB register programming.
- **Verdict:** the ROM sets up an indexed-colour framebuffer and the
  hardware scanner now consumes 8bpp bytes from the DAFB-programmed base and
  stride.  The low-nibble CLUT path is live for first colours, but the
  scanner dimensions and pixel depth remain elaboration-time constants rather
  than runtime DAFB timing/BPP state.

---

## 5. Interrupt audit

- **DAFB VBL** on real Q700 → `VIA1` CA1 pin → VIA1 IFR bit 1 →
  autovector level 1.  `irq_agg.v` documents "level 1 (vec 25) —
  VIA1 (60 Hz VBL …)" so the aggregation *level* is modelled, but
  the VBL *edge* in our sim comes from the VIA1 Timer 1
  under-flow, **not** from a DAFB scanner output.  There is no
  DAFB → VIA1 CA1 wire in the current RTL.
- **DAFB internal IRQ** — `0xF980001C` / `0xF9800020` look like a
  dedicated DAFB interrupt enable+ack pair.  On real hardware
  this can be routed to a slot IRQ or the VIA1 CA1 line; we route
  neither.  ROM writes `0x07` to `001C`; our shim makes `0020` a
  deterministic cycle-driven sticky bit with W1C ack and exposes bit 1
  as `pending & irq_enable[0]`.  It is useful for ROM-visible polling and
  proving the enable/status handshake, but it is not yet tied to real
  scanner vblank or VIA1.
- **No dedicated DAFB IRQ input on `irq_agg`.**  Would need either
  (a) a DAFB→VIA1 CA1 tie-in so the existing Timer-1 VBL story
  becomes scanner-driven, or (b) a new slot-style IRQ aggregated
  through VIA2.

---

## 6. Punch list — top 5 gaps by mac-logo impact

1. **(M) Runtime pixel-depth decoding** —
   `F9800008/00C/010` are writable/exported, production `video_top`
   consumes the base/stride path through a frame-boundary CDC snapshot, and
   the first-light tests prove `0x100` base / `0x61E` stride.  The remaining
   work is decoding the raw DAFB depth selector into 1/2/4/8/16/24/32bpp
   pixel fetch/expand behavior instead of using the elaborated 8bpp path.
   **Effort: M.**
2. **(L) Full CLUT/RAMDAC register file** — 256-
   entry palette RAM at offsets `F9800300 + n*0x10` (with alias
   for the full 256 entries — ROM only writes 16 at boot but
   QuickDraw extends later) and faithful RAMDAC packing.  The current
   low-nibble, 16-entry CLUT is enough for first-light indexed colours, not
   complete DAFB palette behavior.  **Effort: M.**
3. **(M) Scanner-driven VBL/status model** — make `F980001C/0020`,
   scanner frame timing, and VIA1 CA1 agree on frame status.  The current
   status bit is deterministic and W1C, but cycle-driven inside the shim.
   **Effort: S.**
4. **(M) Programmable H/V timing regs** (`F9800124..168`) — accept
   the writes and use them to drive a Mac-mode VTG that feeds the
   scaler's source window (separate from the HDMI 1920×1080
   output VTG).  Lower priority than 1–3 because our scaler can
   still letterbox a 1024×768 or 640×480 source; but the ROM does
   reprogram timings, so ignoring the writes is technically fine
   for logo-only, but the source-window size has to be right.
   **Effort: L.**
5. **(M) DAFB VBL IRQ → VIA1 CA1 tie-in** — drive a 60 Hz edge
   from the scanner's vsync rising edge into VIA1's CA1 input, so
   the ROM's existing VIA1-driven VBL path fires from real
   scan-out frames rather than from a Timer-1 stub.  Needed
   if later ROM/OS code needs frame-synchronous IRQs.  **Effort: S.**

---

## References

- `rtl/mac/video.v`, `rtl/mac/video/video_top.v` (register shim + HDMI chain)
- `rtl/mac/video/scaler.v` (geometry hook + 8bpp greyscale expansion)
- `rtl/mac/glue.v` (DAFB address window decoder)
- `tb/tb_rom_boot.cpp` (DAFB register window left to RTL shim; VRAM aperture RAM-like)
- `/tmp/mame_scout/vram_writes.log` (201-line ROM DAFB write trace)
- `/tmp/mame_scout/q700_full.dis` (full Q700 ROM disassembly)
- `rtl/mac/irq_agg.v` (IRQ level table; DAFB VBL is implicit via VIA1)
- `docs/peripheral_arch.md` (address-map authority)
- `docs/video_smoke.md` (HDMI-chain bring-up context, separate from DAFB)
