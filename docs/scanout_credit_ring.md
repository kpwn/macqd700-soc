# Scanout credit ring — design notes

Covers `rtl/board/video_phy/linebuf_scanout.v` (wrapper), `scanout_fetch.v`
(producer) and `scanout_display.v` (consumer). The RTL headers point here
rather than carrying this; keep the two in step.

## Why this was rewritten (2026-08-01)

`linebuf_scanout.v` used to be a single 1,260-line module that coordinated its
fetch walk with **nine** interacting flags — `prefetch_active`,
`restart_pending`, `placement_pending`, `placement_restart_pending`,
`start_pending`, `awaiting_frame_start`, `frame_active`, `display_primed`,
`restart_stall_frames` — plus `resync_discard`. Four of those were different
flavours of "pending", and **ten** terms gated a single request.

Every bug found on hardware was an interaction *between* those flags, never a
logic error inside one:

* `awaiting_frame_start` suppressed the `line_tag[req_slot] < y_src` term in
  `req_slot_reusable`, which was the only thing that ever released a ring slot.
  With all 64 slots valid the fetcher parked at source row 63 forever — 64 rows
  at the top of the screen, black below.
* the `af8fe56` response-desync watchdog force-fired a re-arm one cycle *after*
  `sof`, and that re-arm re-armed `awaiting_frame_start` — the very flag it
  needed clear. The mitigation fed the condition it was written to break.
* fixing that re-arm (`540f68f`) then exposed the vblank race the flag existed
  to prevent: the fetcher racing a whole frame into the 64-entry ring during
  blanking, leaving only the last 64 rows resident when the display started.

Measured on hardware with `CPU=stub` — a static framebuffer, so all
frame-to-frame variation is scanout — that was **six distinct frames out of
ten**, alternating between a full-height render with the top ~80 rows torn and
one where only the bottom two bands rendered.

## The replacement

### 1. Credit-paced ring

The producer holds credits. The consumer returns one **unconditionally** as it
advances past each source row. A row is fetched iff a credit is available.
Both failure modes become impossible by construction rather than guarded
against:

* It cannot race a frame ahead during vblank. At a frame boundary it holds
  exactly `LINE_COUNT` credits, fills the ring, and stops. The bound is
  arithmetic.
* It cannot park. Credits carry no `line_tag < y_src` comparison for any flag
  to suppress.

The credit counter **saturates at `LINE_COUNT`** — the ring cannot hold more,
and the saturation is what keeps it from wrapping.

The credit is derived from the **advance event**, not from a change in `y_src`.
That distinction is load-bearing: `y_src` also changes when the frame wraps
(`vres-1 → 0` at `sof`), and a wrap pulse lands a cycle or two *after* the
resync has reloaded the counter — by which time the fetcher has already spent
one, so the pulse is not absorbed by the saturation. The fetcher then sits
exactly one ring position too far ahead and clobbers the slot the display is
reading, every row, for the whole frame. Measured during bring-up as *only the
last `LINE_COUNT` source rows rendering* — i.e. the same picture as the
hardware bug this rewrite exists to kill.

It is also delayed two cycles, matching the `line_rd_addr → pix_data_pipe` read
depth, so a slot is only released after the last read of that row is addressed.

### 2. The frame boundary is an unconditional hard resync

There is **no wait on the walk draining**. That unbounded wait was the root of
the original permanent wedge: one lost fetch response made `outstanding_empty`
false forever, so the walk finished the current frame, cleared
`prefetch_active`, and was never re-armed.

Instead the resync latches how many responses are still owed (`stale_count`)
and discards exactly that many. **The fetch port is in-order**, so no tagging
is needed — verified at all three levels:

* `fb_reader.v` — a Gray-pointer async FIFO in each direction; its header
  states "s_rd_data/s_rd_valid are a registered **ordered** pclk-domain
  response stream".
* `vram.v` — a fixed-latency URAM pipeline.
* `scanout_ddr_reader.v` — "every `rd_en` is accepted and produces exactly one
  later `rd_valid` pulse, **IN ORDER**, no drops"; every accepted request is
  pushed onto an in-order FIFO and the drain logic only ever looks at the head.

Two details of the discard window matter:

* **No new requests are issued while it is open.** If a response was genuinely
  lost (`fb_reader` drops one when its response FIFO overflows), `stale_count`
  can never reach zero on its own; without the holdoff the window would eat one
  response of the *new* walk and re-desync it every frame, forever.
* **The window is also time-bounded** (`RESYNC_DISCARD_CYCLES`, 4096 pclk >
  `fb_reader`'s 256+256-deep FIFO pair). That is what closes it in the
  lost-response case.

Consequence: a lost response corrupts the frame it happened in and the very
next frame is clean. **Recovery is one frame with no watchdog**, which is why
`af8fe56`'s watchdog is deleted rather than ported.

`outstanding` is **clamped at zero**. An unmatched response — `fb_reader` can
emit one across a one-sided reset, and the reset domains really are split
(`video_top` wires `fb_reader`'s `vram_rst` to the core-clock bank while the
scanout resets off the pclk bank) — would otherwise wrap the counter to its
maximum. The next resync latches that as `stale_count` and the fetcher sits in
the discard window swallowing everything until the timeout, which at unit-tb
geometries outlasts a whole frame. Measured as `tb-dafb-scanout`'s first frame
rendering black with the FSM parked in `S_FILL`.

### 3. Where the resync fires — NOT at (0,0)

The steady-state resync fires at the **end of the visible frame**, i.e. the
start of vertical blanking. On the real VTG, `(0,0)` is the *first active
pixel*, not a blanking cycle, so clearing the ring there wipes it exactly as
the display starts reading it. With a letterbox border that is invisible (the
first real source row is not needed for another ~60 display lines); at a
geometry with no border it blacks out the top source row outright — measured in
`tb-framebuffer-pixel` scenarios F/G/H (64×64 into 64×64) as 64 mismatched
pixels, all of source row 0.

Top-of-frame is still used, for two narrow jobs:

* **cold start** — `resync_pulse` fires at end-of-visible-frame, so on the very
  first frame after reset there has not been one yet. Armed only when no walk
  is armed (`state == S_IDLE`) *and* the ring is empty. Both terms matter: "the
  ring holds nothing" is not "no walk is armed" — a walk whose first responses
  have not landed yet also has an empty ring, and re-arming that throws its
  in-flight batch into the discard window.
* **servicing a placement change** — see below.

The top-of-frame window is **two cycles** (`sof || sof_d`).
`scanout_placement_sync` commits the new base/stride/depth *on* the frame
boundary, so the committed value is only visible the cycle after `sof`. A
one-cycle window latches the old placement and then has no boundary left to
correct it — measured as `tb-vram-scaler-firstlight`'s prime pass fetching
nothing at all, because its whole priming sequence contains exactly one `sof`.

### 4. One FSM

`S_IDLE / S_RESYNC / S_FILL / S_RUN`. A restart is a **state**, not four
booleans racing each other. A placement change is an **input** to it, serviced
at a frame boundary, never mid-frame: tearing the ring down mid-frame throws
the whole in-flight request batch into the discard window and stalls the
fetcher for as many cycles as there were requests in flight (190 in
`tb-dafb-scanout` — five display rows there). `can_request` is held off in the
meantime, since the latched placement is known-stale.

Reset and resync share one always-block: **a reset is a resync with the
accounting zeroed**, so the two cannot drift apart.

`can_request` is now four terms, not ten.

### 5. Underflow reporting

`line_underflow_sticky` is a sticky health signal, so it must not latch on the
unavoidable transient after a reset or a placement change. A frame reports
underflow iff the **previous frame delivered at least one pixel**. That is
strictly stronger than the old `display_primed`, which armed on "source row 0
happens to be resident at `sof`" — with the ring now cleared at a frame
boundary that would never arm at all, silently disabling the very thing it
gates. The new rule also reports a wholly blank frame, which `display_primed`
could not.

`tb-scanout-frames`' `8bpp-starved` scenario is the positive control: every
other scenario asserts the signal stays clear, which is worthless unless
something proves it can fire.

## Preserved from before the rewrite

* `c57fc08`'s `fetch_row_y_last` / `req_addr_limit` derived from the **runtime
  `vres`**, not the elaborated `SRC_H`. Using `SRC_H` overstated the frame
  footprint (640×480 at 24bpp: 768 rows instead of 480, +60%) and pushed a
  placement that genuinely fits the 2 MiB aperture past `fetch_frame_in_mem` —
  a measured hardware black screen.
* The runtime `hres`/`bpp_shift`-derived `row_last_idx_in`, and the matching
  `min_stride_bytes` term in `scanout_placement_sync.v`, so the two gates
  cannot disagree.
* The CLUT as a true dual-clock dual-port BRAM with no synchroniser: a
  mid-frame palette write lands immediately, which is authentic RAMDAC
  behaviour. All indexed depths index the same 256-entry table, matching MAME's
  `dafb_base::screen_update()`.
* The 4-byte wide fetch group, the wide/narrow 24bpp split, the three-byte-plane
  line buffer (a BRAM-cost decision: 48 RAMB36 instead of 64), and 128 B ring
  lines.
* All depths: 1, 2, 4, 8 and 24bpp. **1bpp is the depth Mac OS boots in.**

## Gating

`tb-scanout-frames` is the multi-frame gate, and the only one that can see any
of this: a re-arm bug is invisible inside a single frame walk, which is how the
original survived the rest of the video tb suite. Its scenarios cover all three
shipping depths, a dropped response at a named point in a named frame (with
every later frame gated STRICT — the one-frame-recovery proof), a permanently
faulting port, a deep/slow port, and the underflow positive control.

**A new module must be wired in two places**: the Makefile *and*
`synth/vivado.tcl`'s explicit `read_verilog` list. Only `cpu/rtl/core` is
globbed there; a module missing from the TCL passes lint, `lint-configs` and
every tb, then dies in synthesis with `[Synth 8-439]`.
