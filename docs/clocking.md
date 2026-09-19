# Clock domain story — m68k-ooo

Planning doc.  Companion to `gameplan.md`, `peripheral_arch.md`, and
`hardware_roadmap.md`.  Today every module lives on the single `clk`
— fine for Verilator, disastrous for hardware.  This doc lays out
the domain map for H0 (KCU116 + FMC mezzanine) and beyond.

---

## 1. The principle

**One fast clock for the core.  Slow clocks for the world.  Async
FIFOs at every crossing; clock-enable pulses where phase-aligned.**

Rule of thumb for every signal: if it leaves the CPU domain, it
either (a) goes through a properly-synthesised async FIFO (for
data), (b) goes through a 2-flop synchroniser (for control), or
(c) rides on a clock-enable pulse that is valid in both domains
(for sub-multiples of the CPU clock only).

No combinational paths cross domains.  No exceptions.

---

## 2. Domain inventory (target state for H0/H1)

| Domain | Freq | Source | What lives there |
|---|---|---|---|
| **`clk_core`** | 50 → 200 MHz | BUFG/BUFGCE_DIV from sys_clk_p 200 MHz | CPU core (if_stage, decode, RAT, ROB, issue queues, ALU, mul_div, LSU, commit, exception, MMU stub, L1I, L1D, CDB net), AXI fabric (axi_xbar), all CPU-facing AXI masters |
| **`clk_pb`** | 50 MHz | SIM: divided from 200 MHz sys clock; real MIG: BUFG_GT divide from 100 MHz fabric reference | Peripheral bus Mac MMIO island (VIA1, VIA2, ENET/SONIC, SCSI register side, SCC, ASC, SWIM/IWM) behind the xbar S1 async bridge. Debug/provisioning are legacy service BARs bridged back to `clk_core`; DAFB registers are xbar S4/core-domain. |
| **`clk_phi2`** | 783.36 kHz for Q700 | NCO enable off clk_pb | 6522 timer counters (already wired through `phi2_tick`), ADB bit cells.  MAME derives the Q700 VIA clock from C7M/10: 31.3344 MHz / 4 / 10. |
| **`clk_scsi`** | 10 MHz | independent MMCM output, async boundary | NCR 5380 phase FSM, REQ/ACK handshakes.  In sim we collapse to `clk_core` but cross-document it as its own domain. |
| **`clk_adb`** | 32 kHz | /N enable off clk_phi2 | ADB physical-layer framing (separate `rtl/mac/adb_phy.v` once it exists) |
| **`clk_rtc`** | 1 Hz | /1M enable off clk_phi2 | RTC seconds counter (already parameterised in rtl/mac/rtc.v via `SEC_DIV`) |
| **`clk_hdmi_pix`** | 74.25 MHz (720p60) or 148.5 MHz (1080p60) | dedicated MMCM | HDMI TMDS PHY + scanner + VTG + i2c_init |
| **`clk_hdmi_tmds`** | 5 × pixel | MMCM output (serdes) | TMDS serialiser |
| **`clk_ddr_ui`** | 300 MHz | Xilinx MIG UI clock, async to clk_core | MIG user interface (AXI slave); we stay on clk_core on OUR side, MIG owns the CDC |
| **`clk_spi_sd`** | 25-50 MHz | /N div or MMCM | SD SPI bit shifter (sd_spi.v currently runs at clk_core with SPI_DIV; keep that pattern) |
| **`clk_eth_rgmii`** | 125 MHz | external PHY RX / MMCM TX | 1 GbE MAC (phase 3+ — no task yet) |
| **`clk_usb`** | 60 MHz | USB PHY ULPI | USB 2.0 host (phase 3+) |
| **`clk_audio`** | 11.29 MHz (256 × 44.1 kHz) | dedicated MMCM | audio DAC / ADC serial clock (ASC internal) |

---

## 3. Current State

Verilator sim still keeps most Mac logic single-domain for speed.  The
real `fpga_top.v` now routes the 200 MHz board clock through
`clk_rst.v`, which handles the core clock buffer/divider and the reset
release sequence.  Default is still a 200 MHz passthrough; the first
board-speed test uses `CORE_CLK_DIVIDE=4` for a 50 MHz core.

Build guardrail: `TARGET_FREQ_MHZ` in `synth/vivado.tcl` is a timing
budget, not a request to create a new board clock.  Use
`make clock-report TARGET_FREQ_MHZ=50 CORE_CLK_DIVIDE=4` when you want
Vivado to spell out the active clocks.  The report should show the fixed
200 MHz board input, the active core clock, and any MMCM-derived clocks
such as HDMI `pclk` separately.

The important exceptions are the HDMI path, the Mac peripheral island,
and CDC utilities:

1. `fpga_top.v` derives `pb_clk` as a fixed 50 MHz domain.  SIM_MODEL
   divides the 200 MHz system clock by four; the real-MIG shell derives
   it from the 100 MHz fabric reference through `BUFG_GT`.
2. xbar S1 crosses `clk_core` -> `clk_pb` through `axi_async_bridge`
   before `peripheral_bus`, so VIA/SCC/SCSI/ASC/SONIC/SWIM register
   traffic no longer runs on the core clock.
3. `fpga_top.v` derives `phi2_tick` from explicit `PB_CLK_HZ` and
   `VIA_PHI2_HZ` generics, defaulting to the Q700-compatible
   783.36 kHz target rather than the old fixed /100 placeholder.
4. `video_top.v` owns a dedicated HDMI pixel MMCM and all VTG/scaler/I2C
   logic downstream runs in `pclk`.
5. `rtl/sys/async_fifo.v`, `rtl/sys/axi_async_bridge.v`, and the
   AXI-Lite `axil_async_bridge.v` wrapper exist with unit tests; they
   are no longer future work items.
6. The first-board default enables `HDMI_TEST_PATTERN=1`, which emits
   pclk-domain colour bars and suppresses VRAM reads.  `video_top` now
   has bounded request/response FIFOs for the VRAM read protocol, and
   `fpga_top` passes the selected `core_clk/core_rst` into that bridge as
   the VRAM-side clock/reset.  With `CORE_CLK_DIVIDE=4`, normal scan-out
   therefore crosses from the 50 MHz core/VRAM domain into `pclk`.  Keep
   the direct test pattern as the first HDMI pinout smoke; disable it only
   when the test goal is normal VRAM scan-out from ROM/smoke framebuffer
   contents.
7. SCSI REQ/ACK will glitch at 200 MHz with no async handling.  Real
   SCSI lines are 5V single-ended and slow — we MUST buffer them through
   input synchronisers + de-glitch filters.
8. ADB, RTC already ride clock enables (`phi2_tick` subdivides for
   RTC; ADB not implemented yet) — these pattern are correct.

---

## 4. Design policy — 3 patterns

### Pattern A: Clock-enable (preferred where sub-multiple)

- Everything stays on `clk_core`.
- A `phi2_tick` (783.36 kHz Q700 target) / `adb_tick` (32 kHz) /
  `rtc_tick` (1 Hz) pulse is generated from clock-enable logic on
  `clk_core`; `fpga_top.v` uses an NCO for `phi2_tick`.
- Modules that need slow behaviour wait for the tick and gate their
  state advance on it.
- **Pros**: no async FIFO, no 2-flop sync, Verilator happy.
- **Cons**: only works for clean integer ratios AND where the slow
  domain doesn't have its own clock input.  Not suitable for RGMII,
  HDMI pixel, DDR UI, or any externally-sourced clock.
- **Who uses it today**: VIA1 timers, RTC.  Will extend to: VIA2
  timers, ASC sample counter.

### Pattern B: True async CDC via FIFO

- Source-domain writer pushes into an async FIFO.
- Destination-domain reader pops.
- Vivado IP `xilinx_axi_clock_converter` for AXI crossings; existing
  `rtl/sys/async_fifo.v` for simple valid/ready crossings.
- **Who needs it**: AXI-Lite between `clk_core` (fabric) and `clk_pb`
  (peripherals).  AXI4 between `clk_core` and MIG `clk_ddr_ui`.  HDMI
  framebuffer reader pulling from VRAM/clk_core into scanner/clk_pix.
- **Implementation plan**: one `axi_async_clock_converter` instance per
  AXI boundary; one `async_fifo` per valid/ready handshake.

### Pattern C: 2-flop synchroniser for single-bit control

- Flop-flop synchroniser on the destination domain.
- Use `(* ASYNC_REG = "TRUE" *)` on the two flops for Vivado CDC-aware
  timing ignore.
- Suitable for: resets, one-shot strobes, level-sensitive status.
- Who needs it: `rst` trees entering slow domains, IRQ lines leaving
  slow peripherals into `clk_core`'s IRQ aggregator.

---

## 5. Proposed domain refactor plan (phase by phase)

### H0 sim (today + near-term)
- Stay single-domain in Verilator.  Keep the phi2_tick / rtc_tick
  enable-based pattern.  NO CDC work required for sim correctness.
- Document in each `.v` header "synth: this module belongs on
  `clk_<domain>`; sim runs on `clk_core`".

### H0 FMC (first bitstream)
- Keep `clk_rst.v` as the board clock / reset owner.  It now handles the
  core clock passthrough/divide and reset release.  The first board test
  should run with `CORE_CLK_DIVIDE=4` and `CORE_CLK_HZ=50_000_000` so we
  can validate the platform at 50 MHz before chasing a faster core.
- Keep `video_top`'s HDMI pixel MMCM for the AL9134 path.
- Use the existing `async_fifo.v` and `axi_async_bridge.v` modules for
  CDC; instantiate Xilinx clock-converter IP only if the hand-rolled
  bridge fails timing or protocol coverage.
- Keep peripheral_bus + its Mac-device slaves on `clk_pb` through the
  xbar S1 async bridge.  DAFB stays on xbar S4/core-domain so display
  register traffic is not coupled to the pb island.
- Hold `cpu_rst` for 50 ms, derived from `CORE_CLK_HZ`, after both
  `soc_full_rst` and the core-synchronized `pb_rst` are low.  This gives
  VIA/SCC/SCSI/ASC/SONIC/SWIM reset state time to settle before the 68k
  can issue the reset-vector fetch or any early ROM MMIO.
- Keep HDMI scan-out on `pclk`.  `video_top`'s VRAM-side clock/reset are
  explicit inputs from `fpga_top` so full scan-out can be tested with
  `HDMI_TEST_PATTERN=0` after the pclk-local bars are proven.
  Rerun the two-clock video tests before changing the first-board generic
  mix.  `HDMI_TEST_PATTERN=1` remains the clean HDMI pinout smoke, while
  `HDMI_TEST_PATTERN=0` exercises the normal core-clock VRAM to pclk
  scanner bridge.
- DDR4 MIG handles its own `clk_ddr_ui` CDC internally.
- Real MIG bring-up on the ALINX AN9134 / KU5P board still needs the
  exact DDR4 component part / speed bin, SPD timing data, and a
  generated `mig_ddr4` IP artifact before we can boot from DDR with
  confidence.  Until those inputs are captured, stay on `ddr_pincheck`
  and the stub guardrail.
- SCSI REQ/ACK + serial PHYs: add input synchronisers at the FMC
  boundary (Verilog `always @(posedge clk_pb) begin sync1 <= pad;
  sync2 <= sync1; end`); the module state still runs on `clk_pb`.

### H1 custom PCB
- Every peripheral that has its own natural frequency gets its own
  clock domain, generated by a dedicated MMCM output (KU5P has 4
  MMCMs, plenty).
- Ethernet PHY clock domain (RGMII), USB PHY domain.
- Real audio clock for the ASC DAC path.

### H2 boxed product
- No new domains.  Tighten power by clock-gating idle peripherals;
  this is about when we push Fmax of `clk_core` to its absolute max,
  not about adding domains.

---

## 6. Verilator sim modelling

Running multiple clocks in Verilator is possible but costs sim perf
(2-4×).  Our current approach is correct: single-clock sim with
enable pulses.  For future bring-up of real CDC infrastructure:

- Provide a `tb-cdc` unit testbench that DOES model two clocks in
  Verilator (unusual for us, but essential for CDC verification).
- The async_fifo.v module gets its own tb with a fast writer + slow
  reader (or vice versa) to verify data integrity across the
  crossing.

Sim-first policy still applies: validate each CDC instance in a unit
tb before trusting it in the full mac_top.

---

## 7. Real-hardware CDC hazard list (specific to Mac peripherals)

| Peripheral | Hazard | Mitigation |
|---|---|---|
| SCSI REQ/ACK | 5V SE, millisecond-scale glitchy edges | 3-stage input synchroniser + de-glitch counter (N cycles at clk_pb) |
| ADB | 200 µs bit cell; pull-up open-collector | Input synchroniser + edge-detect FSM on clk_adb |
| LocalTalk / SCC | RS-422 differential; receiver spec'd for 50 µs rise | 26LS32 transceiver + synchroniser |
| Floppy (phase 3+) | MFM / GCR, 2 µs bit cell | dedicated 8× oversample clock; hold recovered clock on clk_pb |
| HDMI hotplug | 5 V GPIO, async | 2-flop synchroniser into clk_pb |
| USB D+/D- | 12 Mbit/s or 480 Mbit/s | use PHY; synchronise only the ULPI status bits |
| Board reset | pushbutton, bouncy | de-bounce counter on clk_pb, synchronise out to every domain |

---

## 8. IRQ aggregator considerations

irq_agg.v collects IRQ lines from VIA1/VIA2/SCSI/SCC/ASC and produces
`cpu_ipl[2:0]` for the CPU core.  Today all on clk_core.

When peripherals move to clk_pb:
- Each IRQ line (level-sensitive per 68040 autovector semantics) must
  cross clk_pb → clk_core via 2-flop sync.
- Latency: 2 clk_core cycles of uncertainty on each IRQ edge.  Fine —
  Mac OS IRQ handler entry is ~50 cycles anyway.
- Spurious pulses at domain boundary: none, because lines are
  level-sensitive.  If we ever add edge-triggered IRQs (VIA SR
  attention for instance), flag-on-edge-hold-until-ack in the source
  domain before crossing.

---

## 9. Reset

- `rst` is currently global synchronous-active-high on `clk_core`.
- For H0 hardware: generate `rst_pb` / `rst_pix` / `rst_ddr` — each
  synchronised to the target domain's clock but all released N cycles
  after the base `rst_sys` drops.
- Pattern: `clk_rst.v` (existing) already produces `rst`.  Extend it
  to produce per-domain resets, released in sequence:
  1. `rst_core` releases first (CPU, fabric).
  2. `rst_pb` releases after ~16 clk_pb cycles of extra hold (gives
     peripherals time to settle before core starts touching them).
  3. `rst_pix` releases after HDMI MMCM is locked.
  4. `rst_ddr` released by MIG's init_calib_complete.

---

## 10. Action items / sequencing

### Immediate
- Keep first board tests in `HDMI_TEST_PATTERN=1` unless specifically
  validating a `core_clk` to `pclk` VRAM scan-out build.
- Audit and wire the divided `core_clk` into the VRAM side of
  `fb_reader` before claiming the normal VRAM-to-HDMI path is
  hardware-ready at `CORE_CLK_DIVIDE=4`.
- Add a one-line "synth: belongs on `clk_<domain>`" comment to the
  header of each peripheral `.v` so future edits do not assume
  clk_core forever.

### H0 bitstream prep (when we're ready for first synth-close)
- **Task: `video-vram-cdc`** — complete; `video_top` now receives
  explicit `core_clk/core_rst` for the VRAM-side read bridge and the
  two-clock video CDC tests pass under Verilator.
- **Task: `clk_rst` implementation** — replace the stub with MMCM
  generation for core/peripheral clocks and reset release sequencing.
- **Task: `peripheral_bus` onto clk_pb** — wire an async bridge at
  its upstream; migrate every peripheral slave.  1-2 agent-days,
  touches a lot of files but mostly wrapper code.

### H1 (post-first-bitstream)
- **Task: MIG integration** — DDR4 UI clock; pull out the behavioural
  SIM_MODEL for synth path.

### H2 and beyond
- Individual peripheral clocks as needed for real-world interfaces
  (Ethernet, USB, audio).

---

## 11. Why we didn't do this first

Sim is single-domain by policy; clock-enable pulses give us 100% of
the BEHAVIOUR correctness at 0% of the CDC risk.  The CDC
infrastructure only pays off when we synthesise.  Per the SIM-FIRST
policy and the sequence in `gameplan.md` phase 3, CDC work fits in
the H0 FMC bring-up prep window, NOT now.

If agents ever need to synthesise + see real timing closure on a
multi-clock design, that agent gets the CDC infrastructure as a
prerequisite.  Today no agent needs that.

---

## 12. Open questions

1. **Do we want `clk_pb` = 50 MHz or the more Mac-compatible 16 MHz?**
   50 MHz is the current implementation default; 16 MHz matches the
   Quadra's real peripheral clock more closely.  No strong preference
   yet — keep 50 MHz unless a real peripheral forces 16 MHz exactly.
2. **Do peripherals on `clk_pb` need to be Mac-cycle-accurate?**  No
   for most; yes for floppy (GCR bit cell has sub-µs precision).
   Revisit at floppy time.
3. **When does SCSI get its own clock domain vs shared `clk_pb`?**
   Probably never — SCSI async REQ/ACK is slow enough that `clk_pb`
   at 50 MHz oversamples it 10×.  Keep on clk_pb.

---

## 13. Virtual peripherals — the long-term phase

The fundamental asymmetry: Mac OS drivers assume 1991-era I/O speeds
(SCSI-1 at 5-10 MB/s, LocalTalk at 230 kbit/s, slot Ethernet at 10
Mbit/s max).  The physical interfaces on our target board are 1993-
era at one end (ADB, RS-422) and 2020-era at the other (USB-C,
Gigabit, NVMe, HDMI).  The obvious implementation — bit-accurate
emulation of slow Mac peripherals — throws away that speed.

**The win: virtual peripherals present a Mac-compatible register
interface to Mac OS while servicing the actual data over modern I/O.**
Mac OS thinks it's talking to a NuBus Asanté Ethernet card; in reality
the packets cross Gigabit Ethernet.  Mac OS thinks it's reading a
SCSI-1 hard disk; the blocks come off NVMe at 3 GB/s.

This is a phase-5+ initiative but the architectural hooks must exist
earlier.  Below is the long-term plan, NOT a phase-3 deliverable.

### 13.1 What gets virtualised (priority order)

| Real hardware | Mac OS sees it as | Speed-up |
|---|---|---|
| **M.2 NVMe** (internal SSD or add-in) | A very fast SCSI hard disk | 5 MB/s → 3 GB/s (600×) |
| **1 GbE RJ-45** | A NuBus Asanté or 3Com Ethernet card | 10 Mbit/s → 1 Gbit/s (100×) |
| **USB-C / USB 3** | Multiple NuBus/ADB devices (storage, input, audio) | bulk transfers → tens of MB/s |
| **microSD card** | Secondary SCSI device (backup, ROM images) | — |
| **HDMI audio out** | Apple Sound Chip output path + ASC samples | 22 kHz → 48 kHz native |
| **USB webcam** | A virtual QuickTake camera with an INIT that exposes the image API | the whole "1990s Mac sees a webcam" joke |

### 13.2 Required infrastructure (lands incrementally)

1. **DMA engine — `rtl/sys/dma_ctrl.v`** (new module, phase-3 prerequisite).
   - AXI master on `clk_core`.
   - Scatter-gather descriptor list in main memory.
   - Per-channel source/dest/len/cfg.
   - Sends completion IRQ to CPU via irq_agg.
   - Cache coherence: DMA writes to RAM lines that the L1D may cache.
     Software-managed: driver must `CPUSH` before DMA start, `CINV` after.
     This is the standard 68040 model; already why task #75 CPUSH/CINV
     drive matters.
   - Use cases: NVMe block → Mac-RAM staging; Ethernet packet →
     Mac-RAM ring buffer; audio sample → ASC DAC; framebuffer →
     HDMI (though the HDMI scanner already pulls VRAM directly).

2. **Mac OS INIT-based drivers ("extensions")** — written in 68k asm
   or Think C, compiled against classic Toolbox.  Each virtual
   peripheral ships with one.  Lives in `System Folder > Extensions`.
   - NVMe-backed-SCSI INIT: patches the SCSI Manager to intercept
     reads to our device-ID, routes them through the DMA engine.
     Mac OS sees a "very fast SCSI disk" — no driver-stack rewrite
     required.  This is the same pattern as BlueSCSI / SCSI2SD.
   - GbE NuBus-card INIT: declares itself as a slot card with the
     Apple Shared Ethernet Driver hooks, but backed by our 1 GbE
     MAC.  Mac OS uses MacTCP / AppleShare / whatever — the whole
     stack works unchanged.
   - USB-storage-as-SCSI INIT: USB Mass Storage Class → a virtual
     SCSI device on our bus.
   - Audio-line-in INIT: USB audio → Sound Manager input queue.

3. **Host-side control plane (`tools/m68kctl` v2)**
   - Let the Linux/MacOS host drive the virtual peripherals when
     the FPGA is in debug mode (e.g., send a "pretend network
     interrupt" via PCIe XDMA so the Mac OS driver can be debugged
     without real packets).
   - Provision the virtual disk image (same as today's SD
     provision, but backed by NVMe for the shipping product).

4. **Extensibility contract**
   - A well-defined "virtual peripheral slot" interface — an AXI
     slave region + an IRQ line.  New virtual peripherals drop in
     without changing mac_top.
   - The INIT-extension side gets a matching "Declaration ROM"
     at a known slot-space address so the Slot Manager probes
     us correctly.  This is the NuBus trick that made add-in
     cards work on real Macs.

### 13.3 DMA coherence story (important, easy to get wrong)

- D-cache is write-back with software-managed coherence (current
  design, see `docs/core_gaps.md` §5).  Mac OS drivers always
  CPUSH around DMA.  If we follow the same convention, our virtual
  peripherals behave identically to real NuBus cards.
- DMA engine must be an AXI master — NOT go through the L1D cache
  (that's point-in-time coherent, not a good fit for bulk transfers).
- Write target must be addressable by the DMA engine's AXI master.
  "Mac RAM" lives in our DDR4 region (0x0000_0000 + overlay off);
  DMA writes land there, CPU side invalidates cache after completion.
- Alternatives (future):
  - **Hardware-snooping cache**: L1D snoops AXI writes, invalidates
    matching lines.  Adds a comparator per line.  68040 has MI/MA
    pins for exactly this.  Phase 5.
  - **ACE/CHI coherence**: way overkill.  Not recommended.

### 13.4 Int extension + ROM footprint

Classic Mac INITs live in the System Folder as files with file-type
'INIT'.  At boot, the OS loads each INIT into a reserved part of
low memory, runs its entry point, which typically installs some
Trap patches (or slot-card declarations).

For our virtual peripherals to "just work":
- We need a way to ship our INITs WITH the hardware — boot disk
  image includes them, OR user manually installs from a floppy we
  hand them on a floppy ... actually we don't ship a floppy.  Ship
  an HFS disk image on the SD card with the INITs pre-installed.
- Version management: every firmware rev ships with matching INIT
  versions; we publish compatibility matrices.
- ROM ID detection: INITs probe for our virtual peripherals via
  slot-space reads; bail gracefully if firmware is too old.

This is a software engineering effort in its own right (writing
classic-Mac drivers).  Estimated ~1 agent-month per virtual
peripheral INIT.  Not urgent; not even urgent for H2 launch
(we can ship with SCSI2SD-style BlueSCSI semantics via real
disk-image-on-SD and call it done).

### 13.5 Phase plan for virtual peripherals

| Phase | Deliverable | Agent-days |
|---|---|---|
| 3.5 | `dma_ctrl.v` + unit tb + integration with L1D CPUSH/CINV path | 2-3 |
| 4.x | Virtual SCSI-over-SD upgrade (the disk image is already SD-backed; just add DMA for bulk transfer) | 2 |
| 5.1 | NVMe host controller + disk-image-on-NVMe | 5-8 |
| 5.2 | NVMe-backed virtual SCSI INIT (classic Mac asm driver) | ~5 (includes bring-up on real hardware) |
| 5.3 | 1 GbE MAC (Xilinx Tri-Mode Ethernet MAC IP or open-source alternative) | 3-4 |
| 5.4 | NuBus Asanté-compatible virtual Ethernet card + INIT | 8-10 |
| 5.5 | USB 2.0 host PHY + Mass Storage Class → virtual SCSI | 10+ |
| 5.6 | USB HID input → ADB input bridge | 3-4 |

### 13.6 The "extensions" mental model

A key part of the product story: **upgrades arrive as new INITs**.
Ship the hardware; post new drivers on GitHub for MultiFinder
compatibility, faster AppleShare, better USB compatibility, etc.
Nerds can install them from a floppy-image download.  The hardware
never has to ship a firmware update; the classic Mac ROM is frozen
but the system is extensible.

This is ACTUALLY how Mac OS was designed to work — extensibility
via drivers + INITs in the System Folder.  We just exploit it.

### 13.7 What to note NOW

No RTL today needs to change for this.  But two forward-looking
constraints:

1. **Keep the DMA door open.**  The axi_xbar is now 5M × 5S with a DMA
   master slot and explicit DMA-config, VRAM, and DAFB slave ports.  Future
   accelerator work should extend that fabric deliberately instead of
   sneaking in CPU-local intercepts.
2. **AXI IDs.**  Our current AXI lacks AxID / RID entropy — when
   DMA lands it must be able to run transactions concurrently with
   CPU loads without serialising.  Plan for AxID widening (task
   #37's old `wip-rebase` direction; we left it out-of-scope then
   but the reasoning was "no DMA, not worth it").  Revisit when
   DMA lands.

### 13.8 Referenced tasks to update / file

- Update `#19 axi-xbar-vram` scope: 4M × 3S, reserve master slot
  for DMA engine.
- New `dma-ctrl` task: `rtl/sys/dma_ctrl.v` + tb-dma-ctrl.  Phase 3.5.
- New `nvme-host` task: PCIe Gen3 NVMe host controller.  Phase 5.1.
- New `1gbe-mac` task: Ethernet MAC.  Phase 5.3.
- New `virtual-peripherals-init` umbrella: shipping Mac-OS-side
  classic-asm drivers for each virtual peripheral.  Phase 5 and
  beyond.  This is a software product-engineering task, not RTL.

---

## 14. CDC primitives: rtl/sys/async_fifo.v + rtl/sys/axi_async_bridge.v

The two hand-rolled primitives that implement patterns A/B/C above are:

### `rtl/sys/async_fifo.v` — Pattern B atomic building block

Gray-coded dual-clock FIFO; classic Cummings (SNUG-2002) construction.
Parametric payload `WIDTH` (default 32) and `DEPTH_LOG2` (default 4).
Small depths (`DEPTH_LOG2 ≤ 5`) infer distributed LUTRAM; larger depths
infer RAMB18/36 SDP.  Both sides carry independent synchronous resets
(`wrst` on the writer clock, `rrst` on the reader clock).  Water-mark
outputs (`wr_almost_full`, `rd_almost_empty`) at parametric thresholds.

Callers: any valid/ready handshake crossing a clock boundary inside
the CPU or between the CPU and a peripheral.  See the AXI bridge
below for the heaviest client.  Unit tb: `make tb-async-fifo`
(11 scenarios covering reset, fill/drain, wraparound, water-marks,
and both slow-→-fast and fast-→-slow clock ratios).

### `rtl/sys/axi_async_bridge.v` — Pattern B for full AXI4 buses

Full AXI4 master↔slave clock-domain crossing built from five
`async_fifo` instances (one per AXI channel: AW, W, B, AR, R).
Parameterised data/addr/ID widths (default 64 / 32 / 4) and
per-channel depths (AW/AR/B depth 4, W/R depth 16 by default).

Burst transactions pass through unmodified (AWLEN/ARLEN/WLAST/RLAST
are carried in the channel payloads).  Backpressure propagates in both
directions via per-channel full/empty signals.  Unit tb:
`make tb-axi-async-bridge` (8 scenarios: 1-beat read, 1-beat write,
8-beat burst read, 8-beat burst write, interleaved read+write,
slave-side backpressure, slow-→-fast and fast-→-slow clock ratios).

### Where these instantiate in H0+

- `clk_core` (50-200 MHz) ↔ `clk_pb` (50 MHz): one `axi_async_bridge`
  at the peripheral-bus root (between `axi_xbar` I/O slave port and
  `peripheral_bus`), so every VIA/SCC/SCSI/ASC access crosses the
  boundary through a single CDC primitive.  `axil_async_bridge` wraps
  the same primitive for the legacy debug/provisioning service BARs
  that are still decoded under S1 but implemented in `clk_core`.
- `clk_core` ↔ `clk_ddr_ui` (300 MHz, MIG-owned): MIG provides its
  own CDC on the AXI side, so we do NOT insert our bridge here —
  this is called out for H0 bring-up.  The `axi_async_bridge` is the
  fallback if we ever synthesise without MIG.
- `clk_core` ↔ `clk_hdmi_pix`: HDMI framebuffer reader uses a narrow
  `async_fifo` (one line-buffer's worth) — a full AXI bridge is
  overkill since the pixel read stream is valid/ready only.

Pattern-C single-bit synchronisers (the `(* ASYNC_REG = "TRUE" *)`
2-flop cell) remain inline in the modules that need them — no
central module for that one-liner.
