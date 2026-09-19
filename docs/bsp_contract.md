# BSP Contract — machine ↔ board seam

This repo is layered into three directories:

- **`rtl/mac`** — the Quadra-700 *machine*: VIA1/2, SCSI, SCC, ASC, RTC,
  ADB, glue/IRQ aggregation, and the DAFB display-controller register shim
  (`video.v`). Pure Mac-architecture logic; no KU5P/board primitives.
- **`rtl/board`** — the KU5P *board support package*: clocking/reset,
  DDR4 MIG wrapper, SD transport, VRAM, audio output PHYs, CDC primitives,
  HDMI video PHY (`video_phy/`), and the Xilinx IP stub (`vendor/`).
- **`rtl/soc`** — *integration + fabric*: `fpga_top.v` (+ its `.vh` body
  slices), the AXI crossbar/bridges/adapters, peripheral bus, DMA, boot FSM,
  and the SD↔SCSI / VRAM-byteswap glue.

The board layer **exposes** physical-resource interfaces; the machine
(`rtl/mac`) and the integration layer (`rtl/soc`) **consume** them. The six
seams below define that contract. Signal/register details are factual to the
moved RTL headers; consult each module's header comment for the full port list.

## 1. Main memory — AXI4 slave backed by DDR4

- **Board exposes:** `rtl/board/ddr_ctrl.v` — a single AXI4 slave port
  (128-bit data, 6-bit ID, 32-bit address; full AW/W/B/AR/R) shaped to match
  `axi_xbar.v` S0. In `SIM_MODEL` it is BRAM-backed; on real hardware it
  bridges the 128-bit DDR AXI contract to the MIG's 256-bit UI AXI
  (`rtl/board/axi_ddr4_mig_bridge.v`; physical MIG pins instantiated at
  `fpga_top`). Status: `ddr_cal_done` (= `init_calib_complete`).
- **Consumed by:** `rtl/soc` fabric (`axi_xbar.v`, narrow→wide adapters) routes
  CPU and DMA traffic into this port; this slave *is* the machine's RAM
  (mapped at `0x00000000` after boot overlay clears).

## 2. Framebuffer / scanout

- **Machine drives the pixels:** `rtl/mac/video.v` (DAFB shim) holds the
  ROM-programmed framebuffer base/stride/CLUT and display geometry — the
  "what to show / where it lives in VRAM" parameters.
- **Board turns them into HDMI:** `rtl/board/video_phy/` (`video_top.v` =
  mmcm_hdmi + vtg + scaler + fb_reader + i2c_init). It runs the 148.5 MHz
  pixel clock and 1080p60 timing, samples the core-domain base/stride into the
  pclk domain (double-agreement), and issues VRAM reads via `fb_reader.v`
  (pclk↔vram_clk CDC) against `rtl/board/vram.v`'s streaming read port.
  Output: AL9134 RGB/DE/HS/VS + pclk + I2C init.
- **Seam direction:** machine → (geometry/base/stride/CLUT) → board PHY;
  board PHY → (rd_en/rd_addr) → board VRAM → (rd_data/rd_valid) → board PHY.

## 3. Audio sample stream

- **Machine produces samples:** `rtl/mac/asc.v` emits a core-clock 16-bit
  stereo stream (`audio_sample_out[15:0]` + `audio_sample_valid`).
- **Board renders it:** the stream is consumed by one of the board audio
  PHYs — `rtl/board/audio_i2s.v` (external-codec I2S), `audio_pwm.v` (Σ-Δ PWM),
  or `audio_hdmi_bridge.v` (IEC-60958 sub-frames into the HDMI data island).
- **Seam direction:** machine (ASC, core clock) → ready/valid sample
  handshake → board audio output PHY.

## 4. Clocks / reset

- **Board exposes:** `rtl/board/clk_rst.v` takes the 200 MHz board reference
  + active-high external reset + `init_done` (DDR-cal gate) and produces the
  SoC core clock plus a synchronous active-high reset that deasserts only
  after `DEST_SYNC_FF` clean edges once reset and init are both satisfied; it
  also folds in the JTAG/VIO debug-full-reset request. `rtl/board/reset_debounce.v`
  synchronizes + debounces the mechanical `cpu_resetn`/button pins into one
  clean transition. HDMI pixel/MMCM clocking lives in `video_phy/mmcm_hdmi.v`.
- **Consumed by:** `rtl/soc` `fpga_top` distributes core/pb clocks and the
  synchronized resets to the CPU socket, peripheral bus, and machine modules.

## 5. SD / boot

- **Board exposes the SD transport:** `rtl/board/sd_ctrl.v` (SPI-mode CMD17/18/
  24/25/12 data engine) + `sd_spi.v` / `sd_spi_mux.v` (SPI master/mux).
  `rtl/board/sd_jtag_writer.v` is an AXI-Lite aperture for host-driven sector
  writes (gated on `boot_done`).
- **Integration owns the load:** `rtl/soc/boot_fsm.v` runs SD init
  (CMD0/8/55+41/58/6), delegates bulk reads to `sd_ctrl`, and acts as an
  AXI4-MM write master streaming sectors into DDR (via the narrow→wide path)
  **before the CPU is released**. `rtl/soc/sd_scsi_lba_mapper.v` +
  `sd_scsi_bridge.v` map/CDC the machine SCSI disk path onto the same SD media.
- **Seam direction:** board SD ctrl/SPI ↔ soc boot_fsm (master) → DDR slave;
  CPU held in reset until ROM/disk image is resident.

## 6. Debug transport

- **Board exposes:** the JTAG-AXI master and VIO/ILA debug cores (Xilinx
  primitives, gated by `JTAG_AXI_ENABLE` / `VIO_ENABLE` / `ILA_ENABLE`),
  wired in at `fpga_top` (`rtl/soc/fpga_top_debug_vio.vh`,
  `fpga_top_debug_host.vh`).
- **Consumed by:** `rtl/soc` routes the JTAG-AXI master onto the fabric to
  reach the debug-control register window (`fpga_top_debug_ctrl.vh`, whose
  `debug_ctrl` module body lives in the CPU repo) and peripheral/memory space;
  VIO/ILA probes observe core and board signals for bring-up.

---

**Note on the CPU socket:** `rtl/soc/fpga_top` instantiates `m68k_core` and
`debug_ctrl`, both of which live in the separate CPU repository and are
mounted later (Phase 3/4 of the split). Their absence is the expected
elaboration signal when linting `fpga_top` standalone in this repo.
