# Framebuffer First-Light Preflight

Scope: first board validation for the path
`CPU/synthetic AXI writes -> VRAM -> fb_reader CDC -> scaler -> HDMI pins`.

## Required local preflight

Use the serialized make policy and 4 Verilator threads:

```bash
VERILATOR_THREADS=4 VERILATOR_JOBS=4 MAKEFLAGS='-j1' \
  make firstlight-framebuffer-preflight
```

The target runs:

| Target | Coverage |
|---|---|
| `tb-video-pattern` | AL9134 reset/I2C, sync/DE/data pins, direct colour bars, no VRAM reads |
| `tb-video-checkerboard` | Same HDMI bypass path with coarse alignment checkerboard |
| `tb-vram-scaler-firstlight` | DAFB base/stride writes, real VRAM AXI byte writes, first VRAM write/read logging, fb_reader CDC, scaler pixels, underflow sticky clear |

`tb-vram-scaler-firstlight` intentionally uses independent `pclk` and
`vram_clk` ticks with varied phase. It writes one checkerboard, scans it while
a background AXI writer fills an offscreen framebuffer, then relocates DAFB
base/stride and verifies the new framebuffer scans out exactly. The test fails
if `fb_reader` ever reports underflow. It also checks that first VRAM AXI
activity is the programmed framebuffer byte at the DAFB base and that the first
scanout-side VRAM read requests the same base address.

## Hardware sequence

First prove the transmitter path without depending on the framebuffer:

```bash
TARGET_FREQ_MHZ=50 CORE_CLK_DIVIDE=4 CORE_CLK_HZ=50_000_000 \
VIDEO_SMOKE=1 HDMI_TEST_PATTERN=1 \
make clock-report
```

Use the same environment for the bounded implementation run when the shared
Vivado slot is available. This should show pclk-local bars or checkerboard
even if the CPU never reaches ROM DAFB init.

After HDMI-local first light, switch to normal scanout:

```bash
TARGET_FREQ_MHZ=50 CORE_CLK_DIVIDE=4 CORE_CLK_HZ=50_000_000 \
VIDEO_SMOKE=1 HDMI_TEST_PATTERN=0 \
make clock-report
```

With `VIDEO_SMOKE=1`, the reset-time synthetic AXI writer fills VRAM before
normal scanout uses `VRAM -> fb_reader -> scaler`. On hardware, watch
`fb_underflow_sticky` through VIO or a temporary LED before trusting the image.

## 50 MHz risk to clear before declaring ROM framebuffer ready

The line-buffered scanout path fetches each 1024x768 source pixel once per
frame and reuses buffered pixels for the 1080p scale. At 60 Hz that is about
47.2 million source reads/second before FIFO/CDC bubbles and any concurrent
CPU or smoke-writer VRAM traffic. A 50 MHz VRAM/core clock is therefore close
to the practical limit even though the line buffer removed the older
one-read-per-output-pixel requirement. Treat `fb_reader_miss_count` and
`fb_underflow_sticky` as first-light gates before trusting ROM-driven DAFB
output.

Therefore the safe first hardware order is:

1. `HDMI_TEST_PATTERN=1`: prove HDMI clocks, pins, sync, DE, and AL9134 init.
2. `VIDEO_SMOKE=1 HDMI_TEST_PATTERN=0`: prove the real framebuffer path and
   check `fb_underflow_sticky`.
3. Only after underflow is clear, use ROM-driven DAFB writes as the framebuffer
   source of truth.
