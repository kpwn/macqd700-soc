# Video smoke mode — de-risking the HDMI chain without the CPU

> **Scope:** platform/peripheral bring-up.  Enables a reset-time VRAM preload
> that paints a hardcoded bitmap into the URAM framebuffer without requiring
> the CPU or ROM boot to run first.

## Why this exists

Task "show a happy/sad mac on HDMI" depends on three independent things:

1. **ROM boot must progress far enough to hit DAFB init** — still being chased
   via decode-gap fills.  See `docs/core_gaps.md`.
2. **The HDMI output path must physically work on the FPGA** — clk-gen →
   MMCM → VTG → AL9134 I²C bring-up → actual display output.
3. **The VRAM scan-out path must cross clocks safely** — core-clock URAM
   read side through `fb_reader` into the pclk scaler path, with the
   VRAM-side clock wired to the actual core clock used by `fpga_top`.

`HDMI_TEST_PATTERN=1` is the current first-board default and validates (2)
without touching VRAM.  The `tb-video-pattern` sim target now also checks that
the AL9134 reset/I2C sequence is monotonic while the bars are running.  The
`tb-video-checkerboard` target exercises the same bypass with a coarse
checkerboard for easier eye-ball alignment checks.  The `VIDEO_SMOKE=1` path
validates the reset-time VRAM preload separately.  Seeing bars or a
checkerboard on HDMI proves the AL9134/pixel-clock/pinout path, not the full
framebuffer scan-out.
Normal scan-out now passes `core_clk/core_rst` explicitly into `video_top`'s
`fb_reader` bridge, so the VRAM side follows the selected `CORE_CLK_DIVIDE`
instead of assuming the undivided board clock.

## What it does

When `VIDEO_SMOKE=1` is passed to `fpga_top`, the `vram_smoke` module is
instantiated and walks through every word of VRAM at reset release,
writing a **SMPTE-style 8-bar colour pattern** through the vram AXI
slave port:

    WHITE | YELLOW | CYAN | GREEN | MAGENTA | RED | BLUE | BLACK

At `FB_WIDTH_PX = 1024`, each bar is 128 px wide.  The write phase takes
~600 k core cycles, roughly 3-6 ms across the current 100-200 MHz bring-up
range.  After the last word, `smoke_done` latches high and the writer parks
permanently.

When `HDMI_TEST_PATTERN=1` (default), `video_top` emits full-frame pclk-domain
colour bars directly and suppresses VRAM read requests.  Set it to 0 for the
full VRAM -> fb_reader -> scaler path after the pclk-local bars and VRAM smoke
preload have both passed.

The matching unit test checks both halves of that contract: test-pattern mode
must keep VRAM reads at zero, and the HDMI reset/I2C handoff must rise once and
stay high.

When `VIDEO_SMOKE=0`, the `vram_smoke` instance is NOT
generated; the xbar S3 VRAM aperture owns `u_vram` from reset.

## How to enable on the FPGA

Two options.

### Option 1: Vivado generic override

`synth/vivado.tcl` forwards these environment variables into
`fpga_top` generics:

```bash
TARGET_FREQ_MHZ=50 CORE_CLK_DIVIDE=4 CORE_CLK_HZ=50_000_000 \
VIDEO_SMOKE=1 HDMI_TEST_PATTERN=1 \
make clock-report
```

Use the same environment on the PM-scheduled `make impl` run.  Keep
`HDMI_TEST_PATTERN=1` for first board HDMI proof; switching it to `0`
exercises normal VRAM scan-out from the URAM framebuffer through the CDC bridge.

### Option 2: Edit the default

In `rtl/fpga_top.v`:

```verilog
module fpga_top #(
    parameter VIDEO_SMOKE = 1,
    parameter HDMI_TEST_PATTERN = 1
) (
```

Remember to revert before landing a non-smoke bitstream.

### Building the bitstream

Once the generic is set:

```bash
make impl                    # full place + route + bitstream
# (or TARGET_FREQ_MHZ=100 make impl for 100 MHz bring-up round)
```

`make impl` holds the Vivado mutex (`/var/tmp/m68k-ooo-vivado.lock`) —
only one synth/impl at a time on the shared machine.

Flash `build/synth/m68k_top.bit` to the KU5P board.  On reset release:

- LED[0] lit briefly = `core_rst` (normal).
- LED[3] = `hdmi_mmcm_locked` — goes high within ~100 µs of power-on.
- HDMI output shows 8 vertical colour bars within ~10 ms of reset when
  `HDMI_TEST_PATTERN=1`.  The CPU is free to do whatever it wants (crash on a
  decode gap, spin, etc.) — the display pattern does not depend on it.

## How to swap the bitmap

Two places to edit `rtl/mac/video/vram_smoke.v`:

1. **Colour palette** — `bar_rgb` function (line ~115).  8 entries.
   Change any of them to render different bar colours.  RGB888.
2. **Pixel-packing rule** — `pixel_data` function (line ~135).  Today
   it splits x-coordinate by `BAR_W_PX = FB_WIDTH_PX / 8`.  Replace
   with any per-pixel computation that uses `x_rel` and `pix` —
   e.g. a gradient, a checkerboard, or a coordinate-to-index lookup
   into a 512×342 1 bpp happy-mac / sad-mac bitmap.

For a real icon bitmap, the cleanest approach is to:

1. Pre-render the icon as a flat array of per-pixel values in Python
   (e.g. from PIL).
2. Encode it as a Verilog `case(pix)` ladder in a separate function,
   or a `$readmemh` from a `.mem` file at elaboration (simulation-
   only; URAM on KU5P does NOT support INIT attributes — see
   caveats).
3. Fall back to the bars computation when the pixel is outside the
   icon's bounding box.

Keep the module ≤300 lines per the project style guide.

## Caveats

- **URAM cannot be preloaded via INIT attrs** on Xilinx tools; that's
  why we use an AXI-writer FSM instead of a `$readmemh`-in-`initial`
  trick.  The FSM synthesises cleanly (~200 LUT / 50 FF).
- **A simultaneous CPU write to VRAM during the smoke phase is stalled.**
  `fpga_top` muxes the smoke writer onto the VRAM AXI slave while
  `smoke_done=0`; xbar S3 writes wait with `awready/wready=0` until the smoke
  pass completes.
- **The smoke pattern is BPP-aware.**  At `FB_BPP=8`, the bar index is
  packed into the top 3 bits of the byte so the scaler's greyscale
  expansion produces 8 distinct luminance levels.  The current `fpga_top`
  default is 8 bpp to fit the KU5P URAM budget.
- **Smoke mode does NOT exercise DDR.**  VRAM is URAM on the KU5P and
  is not in the DDR path.  If you want a smoke signal that crosses
  DDR, build a separate "DRAM smoke" that walks the DDR controller
  — this module is video-chain only.
- **`HDMI_TEST_PATTERN=1` does not exercise VRAM scan-out.**  That is
  deliberate for first board bring-up.  Use `HDMI_TEST_PATTERN=0` only when
  you want the display to depend on VRAM contents and the pclk/core-clock CDC.
- **Normal scan-out is line-buffered.**  `video_top` now uses
  `linebuf_scanout` for `HDMI_TEST_PATTERN=0`: source pixels are fetched once
  into rolling pclk-side source-line buffers, then reused for the 1024x768 to
  1080p nearest-neighbour repeats.  The VRAM stream must return one ordered
  response for each accepted read, which matches `rtl/sys/vram.v`.
- **Line-buffer depth is first-pass conservative.**  The default production
  path keeps 64 source lines of headroom so a slower core/VRAM clock can read
  ahead during vblank and repeated output rows.  Startup may blank until the
  first needed source line is present; steady-state underflow is reported via
  `fb_underflow_sticky`.

## How to verify with sim before synth

The unit tb exercises the full preload+readback path:

```bash
VERILATOR_THREADS=4 VERILATOR_JOBS=4 MAKEFLAGS='-j1' \
  make firstlight-framebuffer-preflight

make tb-video-smoke         # build + run tb
make tb-video-pattern       # build + run direct HDMI pattern tb
make video-smoke            # same, then print the PPM path

# Inspect the captured frame:
eog  build/video_smoke/frame.ppm        # GNOME
feh  build/video_smoke/frame.ppm        # minimal
display build/video_smoke/frame.ppm     # ImageMagick

# Convert to PNG for a PR / sharing:
convert build/video_smoke/frame.ppm smoke.png
```

The tb uses a shrunken 128 × 48 × 8 bpp configuration so a full write-
and-read completes in < 2 k cycles.  The scaler and MMCM are NOT in
scope for this tb — they're exercised separately by `tb-video`.

## File manifest

| File | Role |
|---|---|
| `rtl/mac/video/vram_smoke.v` | AXI-master writer (SMPTE bars) |
| `rtl/fpga_top.v` | `VIDEO_SMOKE` / `HDMI_TEST_PATTERN` parameters |
| `tb/tb_video_smoke.v` | Verilator wrapper |
| `tb/tb_video_smoke.cpp` | C++ harness (checks pattern, dumps PPM) |
| `Makefile` | `tb-video-smoke`, `tb-video-pattern`, and user-facing `video-smoke` targets |
| `docs/video_smoke.md` | This file |

## Follow-ups worth filing

- Wire `smoke_done` to a spare LED so a board without an HDMI monitor
  can still confirm the writer completed.
- Replace the bar pattern with a pre-rendered happy-mac icon so the
  smoke is even more obviously "Mac".
