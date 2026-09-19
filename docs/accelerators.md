# Hardware accelerators — long-term plan

Post-MVP accelerators that plug into the platform to beat the 68040
ISA's software performance by orders of magnitude on specific
workloads.  These are **phase 5+** — scheduled after the core boots
real Mac OS, the Fmax push is done, and the core IPC goals are met.

Companion docs:
- [`gameplan.md`](gameplan.md) §"Phase 5" — the umbrella
- [`peripheral_arch.md`](peripheral_arch.md) — how the platform hosts
  arbitrary peripherals today
- [`memhier.md`](memhier.md) — L1D coherence constraints each
  accelerator has to respect

---

## Integration pattern (all accelerators follow this)

```
CPU LSU ─► axi-xbar ─► peripheral bus ─► <accel>.v  (MMIO registers)
                            │
                            └─► <accel>.v also has an axi-xbar
                                MASTER port (extra master!) for
                                streaming data to/from DDR or VRAM
                                without CPU mediation
```

Programming model:
1. CPU writes control + source/dest pointers to the accel's MMIO
   register block via peripheral bus.
2. CPU sets the `GO` bit.
3. Accel masters DDR / VRAM through its own axi-xbar port, streams
   data through its datapath, writes results back.
4. Accel asserts an interrupt (VIA2 slot or a dedicated IRQ line) on
   completion — or CPU polls the status register, whichever's
   better for the use case.

Coherence: accels that write DDR RAM must trigger an L1D
invalidation of affected lines (CPU software does `CINV` before
reading results; or accel exposes a "snoop-notify" port into L1D).
Accels that write VRAM URAM bypass L1D entirely (VRAM is
write-combining / uncached from the CPU side anyway).

Every accel is **optional at synth time** via an `ACCEL_*` localparam
in `rtl/mac_top.v`.  Turned off = zero area cost.  Turned on = the
accel instantiates + the axi-xbar grows another master slot.

---

## Tier 1 — serious accelerators (definite phase-5 scope)

### QuickDraw accelerator (`rtl/accel/qd.v`)

**What it does.**  Hardware 2D raster engine.  Mac OS's QuickDraw API
was CPU-bound on every 68040 — every menu paint, window move, scroll,
icon draw is a stream of `CopyBits` / `FillRect` / `PaintPattern`
calls.  This accel turns those into one MMIO-programmed command + a
DMA burst.

Operations:
- `FillRect(rect, pattern)` — 8×8 pattern tiled into an arbitrary
  rectangle, with transfer mode (copy/xor/or/and/not)
- `CopyBits(srcRect, dstRect, mode)` — arbitrary stride, arbitrary
  alignment, source/dest pixel-depth combinations (1/2/4/8/16/24/32
  bpp), transfer mode, optional mask region
- `FillOval` / `FillRgn` — rasterise an oval or region to the
  framebuffer (optional; CopyBits covers 80% of real Mac OS usage)

Sizing: ~5-10K LUTs, 2-4 DSPs (for stride multipliers), 1-2 BRAMs
(for the 8×8 pattern cache).  Fmax: trivially 200 MHz — all straight
streaming, no feedback paths.

Integration: needs an axi-xbar master port (reads source, writes
dest); writes to VRAM URAM directly, reads from DDR for
off-screen bitmaps.  When dest is in DDR (off-screen GWorld), L1D
coherence must be respected — easiest is for the driver (Mac OS INIT
we write) to `CPUSH` the GWorld line range before calling the accel.

Killer app: every window drag.  Measurable speedup on real System 7
workloads >10× vs software QuickDraw on the OoO core.

Effort: 2-3 agent-sessions.  Register map + FSM + per-mode datapath
pipeline.  Biggest risk is the transfer-mode mux width (QuickDraw has
16 modes; software combines two source/dest pixels via a LUT —
easily done in hardware).

### JPEG decoder (`rtl/accel/jpeg_dec.v`)

**What it does.**  Baseline JPEG (8×8 DCT + Huffman + dequant + colour
convert + upsample).  Decodes pixel stream straight into VRAM.

Why it matters: PDF viewers, web image loading (if we ever get
MacTCP up), QuickTime still-frame decoding, disk-thumbnail rendering.
Real 68040 does ~1-2 MP/s software-JPEG; HW does 30-50 MP/s.

Operations:
- `Decode(jpeg_stream, output_rect)` — feeds a JPEG byte stream from
  DDR or SD, produces RGB pixels at dest.
- Optional: progressive JPEG (not worth it for phase 5), YUV 4:2:0 vs
  4:4:4 support (required for any real image).

Sizing: ~8-15K LUTs, 64 DSPs (for the 8×8 IDCT — one DSP per
multiply in the fast-forward DCT algorithm).  1-2 BRAMs for
Huffman tables, 1 BRAM for quantisation tables.  Fmax: 200 MHz with
a 4-stage pipeline.

Integration: axi-xbar master (reads stream from DDR, writes pixels
to VRAM).  No CPU in the loop after `GO`.  Interrupt on completion
or error.

Effort: 4-6 agent-sessions.  JPEG is finicky (Huffman parsing edge
cases, MCU re-ordering, restart markers).  Reference: open-source
"JPEGant" + Xilinx JPEG decoder IP datasheets.

### DMA engine (`rtl/accel/dma.v`)

**What it does.**  On-chip memcpy.  Issues AXI reads + writes through
axi-xbar at bus saturation, no CPU involvement.

Why it matters:
- System 7 INIT loading = a few hundred KB of PEF code copied from
  disk-cached ROM into RAM.  Today every byte goes through the CPU
  LSU.
- CFM / shared library fixups.
- Framebuffer copies when apps double-buffer by hand (pre-GWorld).

Operations:
- `Copy(src, dst, length, stride_src, stride_dst)` — source and dest
  can be any of {DDR RAM, DDR ROM, VRAM URAM}.  Stride parameters
  let it do 2D blits (though QuickDraw accel is better for that).
- `Fill(dst, value, length)` — single-value fill.
- `Scatter` / `Gather` (optional): list-of-descriptors mode, for
  INIT chains that want to queue many copies.

Sizing: ~1-2K LUTs, 0 DSPs.  Fmax: trivial at 200 MHz.

Integration: axi-xbar master.  Interrupt on completion.

Effort: 1 agent-session.  The simplest accel; mostly an AXI-master
FSM with strided-address counter.

---

## Tier 2 — wacky accelerators (phase 5 if bored / for fun)

### AES-256 block cipher (`rtl/accel/aes256.v`)

**What it does.**  ECB / CBC / CTR mode AES-256.  Fully pipelined: one
128-bit block per cycle at steady state.

Why it's wacky: the real 68040 predates AES by a decade.  No
realistic Mac OS workload hits it.  But:
- Encrypted disk images are a thing (DMG) and Mac OS Classic never
  had an accel; 68k implementations are painful.
- Demonstrates that the core can host arbitrary workloads.
- "Fastest AES-on-68040-ISA-ever" is a claim worth making.

Sizing: ~3K LUTs + 16 BRAM18s (for the AES S-box lookup, or
~4K more LUTs if we roll distributed-RAM s-boxes).  Fmax 200 MHz
trivially.  Throughput: ~3 GB/s pipelined (16 B/cycle × 200 MHz).

Integration: MMIO registers for key, IV, mode; streaming block
interface via axi-xbar.

Effort: 1-2 agent-sessions.  AES is heavily referenced; Xilinx has
an app note, and open IP exists (OpenCores AES, Cryptopp reference).

### ChaCha20 stream cipher (`rtl/accel/chacha20.v`)

**What it does.**  RFC 8439 ChaCha20 + Poly1305 AEAD.  One 64-byte
keystream block every ~22 cycles at 200 MHz.

Why it's wacky: same reason as AES.  ChaCha20 is just even
more modern.  Nice to pair with AES for "the Mac with the fastest
crypto suite no user will ever need".

Sizing: ~2K LUTs, <1 BRAM.  ChaCha is ridiculously FPGA-friendly —
the round function is 4× parallel add-rotate-XOR with no data-
dependent branches.

Effort: 1 agent-session.  Very small design.

### SHA-256 hash (`rtl/accel/sha256.v`)

**What it does.**  SHA-256 message digest.  One 512-bit block every
~65 cycles.

Why include: rounds out the crypto suite.  Useful for HMAC if we
ever combine with AES/ChaCha for authenticated crypto.

Sizing: ~2K LUTs, 0 BRAMs.  Fmax 200 MHz easy.

Effort: 1 agent-session.

### zlib / deflate (`rtl/accel/deflate.v` + `.../inflate.v`)

**What it does.**  Hardware gzip decode + (optional) encode.

Why: Stuffit, .sit.hqx, .zip archive extraction on Classic Mac took
forever.  HW deflate is ~50× faster than a 40 MHz 68040.

Sizing: inflate = ~3K LUTs + 2 BRAMs (for the 32KB sliding window).
deflate = ~6K LUTs + 4 BRAMs (LZ77 match finder is the expensive
part).

Effort: 2-3 agent-sessions for inflate; deflate is a separate
ticket — 4-5 sessions because the match-finding heuristics are
tuneable trade-offs.

### CORDIC transcendentals (`rtl/accel/cordic.v`)

**What it does.**  Sin/cos/atan2/exp/log in CORDIC mode.  Makes the
FPU fast on transcendentals.

Why: the 68881/2/040 FPU transcendentals are microcoded sequences
that take 100-200 cycles each.  A CORDIC block does them in 18-22
cycles of pipelined 32-bit datapath.

Sizing: ~1.5K LUTs + 1 DSP.  Fmax 200 MHz.

Integration: not a separate accel — hoist into `fpu_trig.v` (already
a stub).  So this is really an FPU-upgrade ticket disguised as an
accel.

Effort: 1-2 agent-sessions; covered by the phase-5 FPU completion
anyway.

### Polygon rasteriser (`rtl/accel/rast.v`)

**What it does.**  2D polygon rasterisation (lines, triangles,
textured triangles, alpha-blend).  Basically a small GPU.

Why this is really wacky: Bungie's early Mac games (Pathways Into
Darkness, Marathon, Myth) were software 3D on 68k.  HW rasteriser
would make them fly.

Sizing: ~10-15K LUTs, 4-8 DSPs (for interpolators), 1-2 BRAMs
(line buffer).  Fmax probably 200 MHz with careful pipelining.

Effort: 4-6 agent-sessions.  Biggest of the wacky tier.  Probably
never happens — by the time we'd want this, the platform is done
and we're doing retrocomputing demos instead.

### Sound mixer (`rtl/accel/sndmixer.v`)

**What it does.**  N-channel PCM mixer → ASC/AWACS output.  Frees
the CPU from Mac OS `SndDoCommand` software synthesis.

Why: Classic Mac audio was entirely CPU-driven; games (Marathon!)
spent a surprising fraction of frame time doing 8-bit µ-law mixing.

Sizing: ~2K LUTs + per-channel BRAM.  Integrates with `rtl/mac/asc.v`.

Effort: 2 agent-sessions.

---

## Scheduling table

| Accel            | Tier | Effort (sessions) | Phase | Depends on |
|------------------|:----:|:-----------------:|:-----:|------------|
| DMA              | 1    | 1                 | 5.1   | axi-xbar + mac_top wired |
| QuickDraw        | 1    | 2-3               | 5.2   | DMA, VRAM, L1D snoop |
| JPEG             | 1    | 4-6               | 5.3   | DMA, VRAM |
| AES-256          | 2    | 1-2               | 5.4a  | DMA |
| ChaCha20         | 2    | 1                 | 5.4b  | DMA |
| SHA-256          | 2    | 1                 | 5.4c  | DMA |
| zlib inflate     | 2    | 2-3               | 5.5   | DMA |
| CORDIC trig      | 2    | 1-2               | phase-5 FPU | fpu_top bringup |
| Sound mixer      | 2    | 2                 | 5.6   | ASC real impl |
| Polygon rasteriser | 2  | 4-6               | 5.7   | VRAM, FP mul |

---

## Why now (writing this doc)

This is pure planning — none of these are buildable before phase 5
starts.  Capturing the list early lets future sessions resource-plan:
- FPU completion (phase 4) should include CORDIC as a deliberate
  sub-goal, not an accident.
- axi-xbar was designed with 4 master slots from day one; phase-5
  accels will expand that to 6-8, so the retune-for-VRAM ticket
  (`axi-xbar-vram`) should think about headroom.
- The `accel/` directory layout gets reserved in `rtl/` now so
  phase-5 agents land in a predictable location.
- Interrupt routing (VIA2 slot pins vs dedicated IRQ pins) is a
  decision to make once, during phase-2 VIA2 work, so each phase-5
  accel doesn't re-invent its own interrupt path.

---

## Open questions (capture now, answer later)

1. **Interrupt wiring**: dedicated IRQ pins per accel, or VIA2 slot
   interrupts, or a new IRQ aggregator?  VIA2 has 7 slot lines; we'll
   run out if many accels are on.  Likely answer: a small
   `irq_mux.v` that multiplexes accel interrupts onto whichever VIA2
   slot Mac OS expects.

2. **Driver integration**: do we write Mac OS INITs that patch the
   QuickDraw / crypto / compression traps to call the accels
   directly?  Or write a "slot manager"-style stub that presents as
   a NuBus card?  INIT patching is easier; NuBus is more authentic.

3. **Accelerator Fmax vs core Fmax**: if an accel needs a faster
   clock (unlikely but possible — polygon rasteriser might want
   250 MHz for fill rate), do we add a second clock domain or
   downclock to 200 MHz?  Two clock domains are a pain; 200 MHz is
   probably enough for everything listed here.

4. **Memory bandwidth ceiling**: DDR4 × 128-bit × 200 MHz gives
   ~3 GB/s effective.  QuickDraw + JPEG + scan-out + CPU will
   contend.  At what point do we add an L2 or go to wider DDR?
   Probably phase-6 concern.
