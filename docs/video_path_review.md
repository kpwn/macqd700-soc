# Video path review — why the screen is black, and what to build instead

Written 2026-08-20 against the contract:

> **Take any resolution / bit depth / LUT from DAFB, render it as a centred,
> integer-upscaled picture on a 1080p HDMI stream off the HDMI module.**

That sentence is the right specification. The rest of this document is about
the fact that **no module in the current design owns it**, and that every bug
we have seen — the black screen, the rightward drift, the first-pixels
glitch, the 832×624 failure — is downstream of that.

---

## 1. The live bug, root-caused

`video-status` on hardware, in the failing mode:

```
video fetch : rd_en=0 rd_valid=0 rd_ready=1 underflow{linebuf=0 fb_reader=0}
video mode  : COMMITTED hres=1664 vres=624 bpp_shift=3 scale_sel=0
video place : COMMITTED fb_base=0x00000e00 fb_stride=0x00000340 (832 bytes/row)
```

**`hres` is exactly 2× too large.** At `bpp_shift=3` (1 byte/px), 832 pixels
is 832 bytes — which is precisely the stride the DAFB reports. The stride is
right; `hres` is wrong.

`rtl/mac/video.v:648`:

```verilog
assign hres = r_config[3] ? ((hres_raw >> swatch_clockdiv_log2) - 12'd23)
                          : (hres_raw << swatch_clockdiv_log2);
```

The file's own comment documents the intended case: *"raw = 416, clockdiv = 2
→ hres = 832"*. We are getting 1664, so either `hres_raw` is 832 where the
model expects 416, or `swatch_clockdiv_log2` is 2 where it should be 1.
**That single error is the whole failure**, and it cascades twice:

**Cascade 1 — the fetcher refuses to fetch.** `scanout_fetch.v:190`:

```verilog
wire fetch_stride_sane = (fetch_stride_px > row_last_off);
wire fetch_placement_valid = fetch_stride_sane && fetch_frame_in_mem;
```

With `hres=1664`, `row_last_off` is 1663 and the stride is 832. `832 > 1663`
is false, so `fetch_placement_valid` drops, `rd_en` stays 0 forever, and the
screen is black. **The gate is correct** — it is stopping the scanner from
reading past each row into the next row's data. It is doing its job on bad
input.

**Cascade 2 — the scaler picks the wrong ratio.**
`scanout_placement_sync.v:181`:

```verilog
wire [1:0] scale_pick = fits_2x ? 2'd1 : fits_3_2 ? 2'd2 : 2'd0;
```

- True mode 832×624: 2× = 1664×1248 (1248 > 1080, no) → 3:2 = 1248×936 → **fits**
- Buggy 1664×624: 2× = 3328×1248 (no) → 3:2 = 2496×936 (2496 > 1920, no) → **falls to 1×**

So the observed `scale_sel=0` is not an independent bug; it is the same bug
seen through a second lens. **Fix `hres` and both cascades resolve.** That is
the one change to make first.

---

## 2. The structural problem — the contract has no owner

The path is:

```
DAFB regs → mode arithmetic (video.v)
          → placement sync   (admission gates + scale pick)
          → fetch            (more admission gates)
          → linebuf          (slot credits, dual-clock)
          → display          (scale + centre)
          → HDMI (1080p fixed)
```

**Every stage re-derives geometry and applies its own admission test**, and
the tests do not agree on units:

| gate | file | test | units |
|---|---|---|---|
| `fetch_stride_sane` | `scanout_fetch.v:190` | `stride_px > row_last_off` | strict `>` |
| `stable_stride_sane` | `scanout_placement_sync.v:281` | `stride >= min_stride_bytes` | `>=` |
| `fetch_frame_in_mem` | `scanout_fetch.v:191` | `req_addr_limit < FB_PIXEL_LIMIT` | bytes, despite the name |
| `stable_placement_in_range` | `scanout_placement_sync.v:280` | `frame_last_addr < FB_BYTE_LIMIT` | bytes |

Two of these compare the *same physical quantity* with different operators
(`>` vs `>=`), and the naming has already caused confusion — the in-tree
comment at `:192` records that `FB_MAX_PIXELS`/`FB_PIXEL_LIMIT` "read as a
pixel count while every value compared against it was a byte count", and had
to be renamed. **That is the same class of error as the live `hres` bug.**

The 832×624 mode previously passed `fetch_stride_sane` with **one byte of
margin** (3328 > 3327). A contract that is satisfied by one byte is not
satisfied; it is coincidence.

### Failure is silent, and that is the expensive part

When a gate rejects a placement, the result is `rd_en=0` — indefinitely,
with **no reason code, no counter, no status bit**. The screen goes black and
the only way to find out why is to read RTL and hand-evaluate four
inequalities against live register values, which is exactly what diagnosing
this bug required.

This is also why the sim agent could not reproduce the glitch: it tested the
640×480 placement, where every gate passes. **The failing mode never reaches
the code it was probing.**

---

## 3. Scaling policy — DECIDED: integer nearest-neighbour, sharp

**Policy, settled 2026-08-20:**

> **N = min(floor(DST_W / hres), floor(DST_H / vres)), clamped to >= 1.**
> Nearest-neighbour replication. No fractional ratios, no filtering.
> Sharp and smaller beats full-screen and soft.

Applied to the Q700 mode set at 1080p:

| source | N | rendered | letterbox |
|---|---|---|---|
| 512×384 | **2** | 1024×768 | 896×312 |
| 640×480 | **2** | 1280×960 | 640×120 |
| **832×624** | **1** | 832×624 | 1088×456 |
| 1024×768 | **1** | 1024×768 | 896×312 |
| 1152×870 | **1** | 1152×870 | 768×210 |

**`SCALE_3_2` is deleted.** It was added to make 832×624 fill more of the
frame, and it is the direct cause of an artifact the code itself documents:
its 2,1,2,1 line-doubling aliases the 1bpp 50%-dither desktop into fine
striping, which is why the solid menu bar looked right while the desktop did
not. A ratio that renders some source pixels as two display pixels and its
neighbours as one is worse than a smaller, exact image.

That reasoning generalises, and it is why **rational scaling is rejected even
though it is affordable**:

- *Rational without filtering* is strictly worse than integer — it is the 3:2
  artifact at every ratio.
- *Rational with filtering* (bilinear ≈ 12 DSPs at pixel rate, 4-tap
  polyphase ≈ 24, against 1797 free — genuinely cheap in silicon) fills the
  frame but softens pixel-exact content. For a 1bpp dithered Mac desktop that
  trade is the wrong way round.

So the constraint that shaped the old ladder is gone, and we are choosing
integer anyway — on aesthetics, not cost. Worth recording explicitly so
nobody "fixes" the letterbox later by reintroducing a fractional ratio.

**Consequence: the MMCM/AL9134 question is closed too.** Driving a
non-1080p output timing was only interesting as a way to make 832×624
integer-scale to full screen. With sharp-and-smaller as the policy, 1080p
stays fixed and `DST_W`/`DST_H` remain plain parameters.

---

## 4. What to build

### 4.1 A real pipeline: many small machines, each doing one job

The contract must not be implemented as a smear of geometry knowledge across
the SoC. It should be a **pipeline of simple machines with typed contracts
between them**, where each stage does one job well and no stage re-derives
what an upstream stage already decided.

#### What is wrong today: every module is a jack of all trades

| module | lines | jobs it currently holds |
|---|---|---|
| `video.v` | 1348 | DAFB register model **+ geometry arithmetic + depth decode + LUT write port** |
| `scanout_display.v` | 520 | scaling **+ centring + CLUT lookup + pixel unpack + boot logo substitution** |
| `scanout_placement_sync.v` | 524 | admission tests **+ scale policy + frame-boundary commit + range checks** |
| `scanout_fetch.v` | 385 | address generation **+ its own admission tests + prefetch policy** |
| `linebuf_scanout.v` | 315 | dual-clock handoff **+ slot credits + instantiates fetch and display** |

Geometry knowledge appears in **at least four** of these, in two different
unit systems, with two different comparison operators for the same physical
quantity. That is the actual disease; the black screen is a symptom.

#### The decomposition

Each arrow is a **typed, unit-explicit contract**. Names carry `_px` or
`_bytes`. Nothing downstream of stage 4 knows what a "resolution" is.

```
 1  dafb_regs        Mac-facing register file. Reads/writes, side effects,
                     LUT write port. Emits a RAW register snapshot.
                     Knows: the Mac bus.  Knows nothing about pixels.
        │  raw DAFB register snapshot
        ▼
 2  mode_decode      Pure function: registers → canonical mode descriptor
                     {src_w_px, src_h_px, bpp_shift, bytes_per_row,
                      fb_base_bytes, lut_depth}.
                     Owns MAME fidelity — clockdiv, convolution, the 512
                     fixup, the AC842 depth table. THE ONLY PLACE THAT
                     ARITHMETIC LIVES.  Exhaustively testable against the
                     monitor-sense table.
        │  mode_desc
        ▼
 3  mode_admit       Pure predicate: mode_desc + memory limits →
                     {renderable, reject_reason[3:0]}.
                     Owns EVERY inequality in the design. One operator per
                     comparison, one unit system, one place to be wrong.
        │  mode_desc + renderable + reason
        ▼
 4  place_plan       Pure function: valid mode_desc + DST_W/DST_H →
                     {scale_n, active_w_px, active_h_px,
                      border_x_px, border_y_px}.
                     Owns the SCALING POLICY and nothing else:
                     scale_n = min(DST_W/hres, DST_H/vres), clamp >= 1.
                     Integer only (S3).  A comparison ladder over the few
                     legal N -- no divider, no DSP.  Swapping 1080p for
                     another timing is a parameter change here and touches
                     no other stage.
        │  placement
        ▼
 5  frame_commit     Control only, zero arithmetic: latches a placement on a
                     frame boundary, holds it stable for the frame, signals
                     changes. Owns the "when", never the "what".
        │  committed placement
        ▼
 6  row_addrgen      placement → stream of fetch descriptors
                     {row_index, byte_offset, byte_length}.
                     Owns stride walking and prefetch depth. Knows nothing
                     about AXI.
        │  fetch_desc (ready/valid)
        ▼
 7  fetch_engine     descriptors → AXI → row payloads. Owns outstanding
                     bookkeeping, ordering, credits, error handling.
                     Knows nothing about geometry — cannot, by construction.
        │  row payload (ready/valid)
        ▼
 8  line_store       Slot storage + credits + the ONE dual-clock crossing.
                     Owns the CDC and nothing else.
        │  packed row bytes @ pclk
        ▼
 9  pixel_unpack     packed bytes + bpp_shift → palette indices.
                     Owns bit-depth unpacking only. 1/2/4/8bpp and direct.
        │  index stream
        ▼
10  clut             index → RGB. Owns the palette memory and its write port.
        │  rgb stream
        ▼
11  upscale          rgb + scale_num/den → replicated rgb. Owns replication
                     only. Has no idea what a border is.
        │  rgb stream
        ▼
12  compositor       active region + borders → full DST frame. Owns centring
                     only. Also the natural home for the boot logo, which is
                     currently tangled into the scaler.
        │  rgb + de/hs/vs
        ▼
13  vtg / phy_out    Timing generation and the AL9134 pins.
```

#### The rules that make it hold

1. **Arithmetic lives in exactly one stage each.** Mode arithmetic in 2,
   inequalities in 3, scaling policy in 4. If a value is needed downstream it
   is *carried*, never recomputed. This alone would have made the `hres` bug
   a one-line fix in one file instead of a four-file investigation.
2. **Units are in the signal names**, and stage 3 carries a Verilator
   assertion that its byte-domain and pixel-domain derivations agree. The
   in-tree rename of `FB_MAX_PIXELS` → `FB_MAX_BYTES` happened *because* this
   rule did not exist.
3. **Pure stages are pure.** Stages 2, 3, 4, 9, 10, 11 are combinational
   functions of their inputs — trivially unit-testable with a table, no
   testbench harness, no clock.
4. **One CDC, in one module** (stage 8). Today the fetch/display clock
   relationship is implicit in `linebuf_scanout` while it *also* instantiates
   both sides.
5. **Backpressure is explicit ready/valid** at every arrow, so a stall
   anywhere is visible and attributable rather than presenting as `rd_en=0`
   forever.
6. **Only stage 3 can say no**, and it must say *why*. No other stage
   silently declines to do its job.

#### Migration, without a big-bang rewrite

The existing code already contains the seeds — this is refactoring toward a
shape that is half-present, not inventing one:

- `scanout_placement_sync.v` already has a reusable admission **function**
  over an arbitrary tuple, plus a Verilator equivalence assertion that fires
  if the function and the inline wires disagree. **That is stage 3 in
  embryo.** Promote the function to a module, delete the inline wires, and
  the assertion becomes the migration's own regression test.
- `scale_pick` and the `border_x`/`active_w` arithmetic already sit together
  and are already pure. **That is stage 4**; it needs extracting, not writing.
- `mode_decode` is a contiguous block in `video.v` around `:637-660` with no
  state — it can be lifted out mechanically and given the exhaustive
  monitor-sense table it has never had.

Do those three extractions first. They are mechanical, individually
verifiable, and they remove geometry knowledge from three of the four places
it currently hides.

#### The property that makes all of this cheap: a generous latency BUDGET, a hard DEADLINE

Be precise about this, because the loose version of it is wrong. This path
**is hard real-time** — every output line has a deadline and missing one is a
visible glitch. What is generous is the *latency budget*, not the deadline:

```
frame @ 60 fps                    16.67 ms
output line @ 1080p60             14.81 us
a 1 ms constant pipeline delay  =  ~67 output lines  =  6% of one frame
```

**A bounded, constant ~1 ms of delay costs nothing** — the raster is 67 lines
behind the fetch and no one can tell. What costs everything is *jitter*: a
single line that arrives late. So the design rule is not "latency does not
matter"; it is:

> **Spend latency freely and constantly. Never spend it variably.**

Two consequences the current code does not exploit.

**Consequence 1 — register everything, freely.** Every arrow in the
decomposition above can carry as many pipeline stages as it likes; ~67 output
lines of budget is an enormous number of register stages. There is no IPC
cost, no branch penalty, no cycle to claw back. So each machine should
be built for **Fmax and clarity, never for latency**: register its inputs,
register its outputs, never reach across a stage boundary combinationally.
This matters concretely — the design currently sits at a true
`fabric_clk100` WNS of −0.064 ns and congestion level 5–6
(`docs/v2_interconnect_handoff.md` §6), and the video path runs through the
same fabric. A deeply-pipelined video path is *free* in the only currency
that is scarce here.

**Consequence 2 — buffer depth is the design variable, and it is cheap.**
Deep buffering is what converts an unbounded latency problem into a bounded
throughput one. The arithmetic at 1080p60:

```
output line       2200 px @ 148.5 MHz          = 14.81 us  = 1481 core clocks
832x624 at 3:2    624 src rows -> 936 dst lines
source row consumed every 1.5 output lines     = 22.2 us   = 2222 core clocks
one source row    832 B = 52 x 16 B beats
                                       duty    = ~2.3%
```

**The bandwidth is trivial — about 37 MB/s against a 1.6 GB/s fabric.** The
entire risk is *blackout*: an interval where the fetch path gets nothing
back. Sources, in order of increasing nastiness:

| source | scale |
|---|---|
| DDR4 refresh (`tRFC`) | ~350 ns ≈ 35 core clocks — **1.6% of one source-row budget** |
| bank conflict / row miss | tens of cycles |
| arbitration loss to CPU traffic | **unbounded**, and this is the real one |

Refresh is a non-event at this duty cycle. The unbounded term is arbitration:
scanout shares DDR with the CPU through `axi_vram_priority_mux3`, and after
today's L2C work the CPU can present **8 outstanding fills + 8 victim
writebacks + 8 bypass transactions** where it previously presented one. We
measured zero underflow, but zero-underflow-today is not a bound.

So size the line store to survive worst-case starvation, not typical load.
**N buffered source rows tolerates N × 22.2 µs of total DDR blackout**; four
rows is ~89 µs, which no plausible arbitration stall reaches. And the
resource is the one we actually have: **BRAM is at 24.8% (119 of 480)**,
against URAM at 100% and LUTs at 76.9%. Line buffering is exactly the thing
to spend BRAM on.

Three design rules follow, and they replace the current prefetch policy:

1. **Prefetch by whole source rows, as far ahead as the buffer allows** —
   not by a fixed `PREFETCH_DEPTH=5` ring-line window that is blind to
   stride. §4.5 measures that window costing ~13× read amplification and a
   cold miss at *every* source-row start; a row-granular prefetcher with
   depth to spare makes both disappear.
2. **Never let a stall propagate to the raster.** With explicit ready/valid
   and a deep store, a fetch stall consumes buffer margin instead of
   producing a glitch — and the margin consumed is a *measurable number*, so
   it becomes a high-water-mark counter rather than an artifact someone
   reports by eye.
3. **Make the high-water mark a first-class output.** Report minimum buffer
   occupancy per frame in `video-status`. That number is the entire safety
   margin of the video path, and today it is neither measured nor known.

#### Push every adder and multiplier into DSP, across the whole pipeline

**27 of 1824 DSP48E2 are used — 1.48%. The video path uses none.** That is
1797 idle hard macros next to a LUT fabric at **76.9%** with routing
congestion at **level 5–6**. Every arithmetic operator left in LUTs is
spending the scarce resource to avoid using the abundant one.

This is repo policy, not a new idea — `CLAUDE.md` already says *"DSP58E2
blocks used for multiply, shifts, and adder trees wherever it helps timing."*
The video path has simply never followed it.

**Two properties make DSP the right default here, not just an option:**

1. **The internal pipeline registers are free.** A DSP48E2 has A/B input
   regs, an M register after the multiplier, and a P register on the output —
   up to four stages that cost nothing and run past 500 MHz. §4.1's latency
   budget is ~67 output lines; spending four register stages per arithmetic
   node is invisible. **A latency-tolerant pipeline is exactly the workload
   DSP pipeline registers were designed for.**
2. **It moves arithmetic off the congested fabric.** Each operator relocated
   is LUTs returned and wires removed from windows already at congestion 5–6.

**Where, concretely.** The DSP48E2 computes `(A + D) × B + C` with a
pre-adder and a 48-bit accumulator, and its ALU also does add/sub/compare
with pattern detect — so plain adders and comparators belong there too, not
only multiplies.

| stage | expression | shape |
|---|---|---|
| **6 row_addrgen** | `fb_base + row × stride + row_off` | **exact `(A+D)×B+C` fit** — the hot one, evaluated per row |
| **3 mode_admit** | `base + stride × last_row + row_last_off` vs limit | same multiply, then ALU compare + pattern detect |
| 4 place_plan | `active_w = hres × N`, `border = (DST − active) >> 1` | multiply + ALU subtract |
| 2 mode_decode | `hres_raw << clockdiv`, `(vfp>>1) − (val>>1)` | ALU |
| 11 upscale | replication counters | ALU accumulate |
| 12 compositor | `x`/`y` against `border_x`/`border_y` | ALU compare / pattern detect |

Stages 3 and 6 are the prize: **both compute `base + stride × row + offset`**,
today as inferred multipliers sitting inside the fetch and admission cones —
the same cones that produce the `rd_en=0` decision. One DSP each removes the
multiply *and* deepens the pipeline for free.

Note this does **not** revive fractional scaling. §3 is settled on
aesthetics, not cost. Integer replication still needs no multiplier in the
pixel path — but every *address* and *admission* computation feeding it
should be in a DSP.

**Rule for the rebuild: if a stage computes anything wider than a mux, it
should be asking why it is not a DSP.**

### 4.2 Make the failure loud

`video-status` must answer "why is the screen black" without reading RTL:

- print `reject_reason` and the tuple that was rejected
- print `vio_fb_reader_stats` — **it already exists** at
  `rtl/soc/fpga_top_debug_vio.vh:84` (`{miss_count, rsp_count, req_count}`)
  and is simply never displayed. `(req_count − rsp_count)` is the exact live
  outstanding-response count; small and stable means the byte-slip family is
  dead, drifting means that *is* the bug and its value is the horizontal
  displacement in source bytes.
- a sticky "placement rejected since last commit" bit

### 4.3 Fix the `hres` derivation and pin it

Root-cause the 2×: read `hres_raw`, `r_config[3]` and `swatch_clockdiv_log2`
live and compare against MAME's `dafb.cpp` for the same monitor sense. Then
add a directed tb over **every Q700 monitor-sense code**, asserting the
committed (hres, vres, bpp, stride) tuple matches MAME. That table is small,
enumerable and currently untested.

### 4.4 Close the coverage holes

- **`make tb-video` is RED on main and has been** — proven against pristine
  HEAD, so nobody is watching it. Fix or delete; a permanently-red gate is
  worse than no gate.
- **No scanout tb runs at 832×624.** Every scanout scenario uses geometries
  where the gates pass. Add the failing mode as a scenario; it would have
  caught this before hardware.
- **`axi_vram_priority_mux3`'s `s3_*` lane is tied off in every scanout tb**
  (`tb_scanout_ddr_frames.v` hardwires `s3_awvalid/s3_arvalid = 0`). Three-
  source arbitration — CPU VRAM-aperture writes against live scanout — has
  never been exercised.

### 4.5 The prefetcher is stride-blind (measured, not speculative)

At the live geometry each source row is 80 bytes inside one 128 B ring line,
and the next row sits at `req_line+8` — outside the `PREFETCH_DEPTH=5`
window. Measured **8.3 bursts (1,058 B) of DDR traffic per 80 B row, ~13×
amplification**, and every source-row start is a cold miss that stalls the
whole request queue for a full DDR round trip. Absorbed today (0 underflow),
but it is the least-margin structure at exactly the place the artifact
appears, and it will not survive a higher-bandwidth mode.

---

## 5. Ordering

0. **Adopt the decomposition as the target shape** (§4.1) before writing any
   more video RTL. Every fix below should move code *toward* it, not add
   another special case to a module that already holds four jobs.
1. **Fix `hres`** (§4.3). One bug, two cascades, black screen gone.
2. **Reason codes + `fb_reader` stats in `video-status`** (§4.2). Cheap, and
   it makes everything after this diagnosable in seconds instead of hours.
3. **832×624 scanout scenario** (§4.4). Locks the fix in.
4. **Single geometry authority** (§4.1). The structural fix.
5. **Delete `SCALE_3_2` and implement the integer ladder** (§3). Small,
   and it removes a documented artifact rather than adding a feature.
6. **Prefetcher stride awareness** (§4.5).

Steps 1–3 are a day and remove the symptom. Step 4 is what stops the next
mode from doing this again, which is what "close these bugs once and for all"
actually requires.

---

## 6. What is NOT wrong

Recorded so it is not re-investigated:

- **Not a fetch underrun.** `underflow{linebuf=0 fb_reader=0}` on hardware.
- **Not display-pipeline misalignment.** The latency ladder
  (addr→`line_rd_addr`→`pix_data_pipe`→`pix_data_pipe2`→`clut_rdata`→`rgb`)
  is 5 stages, and `de`/`hs`/`vs`/`border_pipe`/`pix_valid_pipe`/`x_src_lo_pipe`
  are all exactly 5. Hand-verified.
- **Not a line-buffer BRAM read/write collision.** Structurally unreachable:
  `line_valid[slot]` is cleared at `row_start` and only re-set when the row's
  last byte lands, so a slot being written is never `disp_ready`. Verified
  with a mutant that collapses the ring's one-row separation — it surfaces as
  underflow, never as a consumed-read collision.
- **Not the resync discard window.** Required window is 256–512 pclk against
  a 4096 budget (~10×), and in steady state the window is never entered:
  `outstanding` is 0 at every end-of-visible-frame resync.
- **Not queue lapping.** `q_peak = 48/64` at every geometry, capped by
  `MAX_INFLIGHT=48`.
- **Not the output stage.** Parallel RGB/DE/HS/VS to an external AL9134, no
  in-FPGA TMDS or data-island logic — a 2-deep register. Nothing between the
  scaler and the pins can corrupt a line's first pixels.
