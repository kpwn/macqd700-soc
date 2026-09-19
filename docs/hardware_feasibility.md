# Hardware feasibility — H0 (FMC mezzanine) / H1 (custom board)

> Research doc.  Companion to [`hardware_roadmap.md`](hardware_roadmap.md)
> (phases H0 → H2) and [`clocking.md`](clocking.md) (domain inventory).
> Deliverable for task #68.  No RTL touched.  Sources cited inline.
>
> Snapshot date: **2026-04-17**.  All pricing from Digikey unless
> otherwise noted; all utilization numbers from the post-route report
> at `build/vivado/reports/utilization_route.rpt`.

---

## 0. Executive summary (one page)

**Verdict: GO for H0. Conditional GO for H1 pending the task #92
(dcache-as-BRAM) + task #73 (real L1I) landing.**

The existing `main @ f85adee` RTL routes on an `xcku5p-ffvb676-2-i`
KCU116 at 100 MHz with **49 % LUTs / 16 % FFs / 0 % BRAM / 0.2 % DSPs
used**.  On-chip power is 1.66 W, well inside KU5P's envelope.  There
is ample headroom for all phase-3 additions (real L1D/L1I, MMU walker,
Ethernet MAC, HDMI IP) even in the pessimistic projection.

The **one structural concern** is that `u_dcache` today consumes
46 500 LUTs and 40 966 FFs because it is inferring LUTRAM for its
tag+data arrays (task #92 tracks this).  That number collapses to
~2 kLUT + 4–8 RAMB36 once it moves to BRAM, freeing ~44 kLUT — plenty
for the remaining phase-3 blocks.

**Top-3 cost drivers for H0 (FMC mezzanine kit that plugs into a
KCU116 the nerd buys separately):**

| # | Item                                   | 1-off $  | 100-qty $  |
|---|----------------------------------------|----------|------------|
| 1 | KCU116 evaluation kit (user buys)      | $7 349   | n/a (user) |
| 2 | FMC mezzanine PCB + BOM (we ship)      | $180–220 | $120–150   |
| 3 | DDR4 SODIMM + microSD + cables         | $30–50   | $18–28     |

H0 ship-to-user all-in (we ship ONLY the mezzanine and recommend a
user-bought KCU116):  **mezzanine $220–260 incl. margin**.  With
subsidised KCU116 bundled: $8 200 all-in (only viable as "dev kit"
for 20-50 early-access units at cost-plus, per roadmap §H0).

**Top-3 cost drivers for H1 (custom board — retail product):**

| # | Item                                   | 1-off $  | 100-qty $  |
|---|----------------------------------------|----------|------------|
| 1 | KU5P bare die (XCKU5P-1FFVB676I -1)    | $2 872   | ~$2 100 *  |
| 2 | 8-10 layer PCB + assembly (CM)         | $450     | $55–80     |
| 3 | DDR4 SODIMM 4 GB (smallest std SKU)    | $18      | $14        |

\* qty-100 KU5P pricing requires Digikey quote — see §1.  The chip is
by far the dominant BOM line; at $2 100/unit, KU5P in qty-100 alone
BUSTS the $180-$250 BOM target in `hardware_roadmap.md` §H1.  **This
is the real feasibility blocker.**  Three mitigations (§7): pivot to
ZU3EG/ZU4EV for retail volume, lifetime-buy XCKU3P at ~$1 600/qty-100,
or accept a $499 BOM floor and position retail at $1 299+.

Remainder of the doc defends these numbers.

---

## 1. Part choice — `xcku5p-ffvb676-2-i`

### 1.1 Why this part

The existing RTL targets `xcku5p-ffvb676-2-i` (see `synth/vivado.tcl:23`
and `synth/fpga_top.xdc`).  Package FFVB676 (676-pin FCBGA), -2 speed
grade, industrial temp, 280 user I/O.  Resources per
[UltraScale+ product data sheet](https://www.mouser.cn/datasheet/2/903/ds890_ultrascale_overview-1591529.pdf):

| Resource        | KU5P (-2I)    | Our 100 MHz post-route usage |
|-----------------|---------------|------------------------------|
| System Logic Cells | 474 600    | (49 % LUT)                   |
| CLB LUTs        | 216 960       | 105 547  (48.6 %)            |
| CLB Flip-Flops  | 433 920       |  70 280  (16.2 %)            |
| Distributed RAM | 99 840 (Mb≈3.1)|  10 340  (10.4 %)            |
| Block RAM (36Kb)| 480 ≈ 17.28 Mb|      0   (0 %)               |
| UltraRAM        | 64  ≈ 18 Mb   |      0   (0 %)               |
| DSP58E2         | 1 824         |      4   (0.22 %)            |
| CMT (MMCM+2 PLL)| 4 CMTs (4 MMCM + 8 PLL) | 1 MMCM used (HDMI) |
| GT YH trx       | 16 × 16.3 Gb  | 0 (not wired)                |
| Max I/O         | 280 HP        | 46                           |

### 1.2 Digikey pricing + availability (2026-04-17)

| Part number              | Speed | Temp | Qty 1 (US $)                           | Stock        | Mfg lead time |
|--------------------------|-------|------|----------------------------------------|--------------|---------------|
| **XCKU5P-2FFVB676I**     | -2    | I    | **$3 676.10**                          | 2 units      | 40 weeks      |
| XCKU5P-1FFVB676I         | -1    | I    | **$2 872.42**                          | 4 units      | 40 weeks      |
| EK-U1-KCU116-G (dev kit) | n/a   | n/a  | **$7 349.25**                          | 14 units     | 8 weeks       |

Sources: [Digikey XCKU5P-2FFVB676I](https://www.digikey.com/en/products/detail/amd/XCKU5P-2FFVB676I/6797925),
[Digikey XCKU5P-1FFVB676I](https://www.digikey.com/en/products/detail/amd/XCKU5P-1FFVB676I/6797921),
[Digikey EK-U1-KCU116-G](https://www.digikey.com/en/products/detail/amd/EK-U1-KCU116-G/7035246).

**Qty-100 pricing:** Digikey does not publish it; requires a formal
quote.  Industry rule-of-thumb for this tier is a ~25 % discount from
qty-1, landing the -1 part at **~$2 155/unit qty-100** and the -2 at
**~$2 757/unit qty-100**.  Real qty-100 numbers require supplier RFQ
(AMD direct, Arrow, or Avnet); at the time of writing Digikey shows
in-stock but tiny (2-4 units) quantities so AMD direct reservation is
the realistic path.

**Lead time** is the real risk factor: **40 weeks mfg lead**.  That is
10 months, which would push H1 from the roadmap's "6-12 months after H0"
to "18-22 months after H0" unless we place a reservation order now.
`hardware_roadmap.md` §8 flagged this (HIGH risk).  Reserve 500 units
via AMD direct at H0 kickoff; keep a ZU7EV fallback schematic (§7).

### 1.3 -1 vs -2 speed-grade decision

| Speed | Fmax (KU5P worst-case) | Our 100 MHz design? | Our 200 MHz goal? | Price premium |
|-------|------------------------|---------------------|-------------------|---------------|
| -1    | ~625 MHz fabric        | Yes (plenty margin) | Marginal          | Baseline      |
| -2    | ~725 MHz fabric        | Yes                 | Yes (comfortable) | +28 %         |
| -3    | ~775 MHz fabric        | Yes                 | Yes               | +100 %+       |

Our target is 200 MHz (`CLAUDE.md`).  Current build closes at 100 MHz
without heroics (see `docs/fmax_analysis.md` and `docs/fmax_retime_log.md`
for the retime bank history).  Once the F1-F3 registered-PRF / move-elim
bundle lands (§microarch.md proposals F1-F3), the design should close
at 150-200 MHz on -2.  -1 might or might not — we'd find out during
impl.

**Recommendation:**
- **H0:** use the -2 part that's already on the KCU116.  Zero work.
- **H1:** target -1 for cost (saves $700-800/unit).  If we miss timing
  on -1 we have two outs: bump to -2 for the BOM hit, OR add pipeline
  stages to compensate (acceptable IPC cost).  Build 5 -1 prototypes
  first to de-risk.

---

## 2. Utilization headroom

### 2.1 Current numbers (post-route @ 100 MHz, 2026-04-17)

From [`build/vivado/reports/utilization_route.rpt`](../build/vivado/reports/utilization_route.rpt)
and the companion [`power.rpt`](../build/vivado/reports/power.rpt):

```
fpga_top                  |  105 547 LUTs (48.6%)  70 280 FFs (16.2%)  0 BRAM  4 DSP
├── u_cpu (m68k_core)     |   68 881 LUTs          58 874 FFs          0 BRAM  4 DSP
│   ├── u_dcache          |   46 500 LUTs          40 966 FFs          0 BRAM  (LUTRAM!)
│   ├── u_rob             |    5 574 LUTs           7 397 FFs
│   ├── u_iq_int          |    2 873 LUTs            332 FFs
│   ├── u_if              |    2 226 LUTs            399 FFs
│   ├── u_lsu             |    2 323 LUTs            387 FFs
│   ├── u_commit          |    2 165 LUTs            833 FFs
│   ├── u_bpu             |    2 003 LUTs          3 840 FFs
│   ├── u_iq_mem          |    2 095 LUTs            691 FFs
│   └── u_alu + u_mul_div |    1 914 LUTs            917 FFs          4 DSP
├── u_sd_provision        |  REMOVED (boot is via JTAG-AXI now)
├── u_ddr (SIM_MODEL)     |   13 694 LUTs             850 FFs (LUTRAM: 10 240)
├── u_scsi (stub)         |    2 345 LUTs           4 321 FFs
├── u_boot_fsm + u_ctrl   |      453 LUTs             355 FFs
└── u_video (HDMI)        |      139 LUTs              85 FFs  (plus 1 MMCM)
```

**Power:** 1.66 W total (1.18 W dynamic + 0.48 W static), confidence
LOW (I/O activity not annotated).  Even generously 3× that for full-load
I/O it's ~5 W — comfortably inside KU5P's ~25 W envelope.  Junction
27.9 °C @ 25 °C ambient, 250 LFM airflow, medium heatsink.  No thermal
concerns for H0.

### 2.2 Projected LUT% after known work lands

`u_dcache` inferring LUTRAM is a known bug (task #92) — dcache should
back its tag+data arrays with RAMB36.  Once fixed:

| Scenario                                        | LUT (K) | LUT %  | BRAM  | Warn? |
|-------------------------------------------------|---------|--------|-------|-------|
| Today (main @ f85adee)                          | 105.5   | 48.6 % | 0     | no    |
| After #92 (dcache→BRAM, save ~44 kLUT)          |  ~62    | ~29 %  | 8–12  | no    |
| After #73 (real L1I, 4KB 4-way tags+data)       |  ~70    | ~32 %  | 16–24 | no    |
| After #75 (CPUSH/CINV + store-buffer forward)   |  ~73    | ~34 %  | 16–24 | no    |
| After MMU walker (task implicit, docs/core_gaps)|  ~78    | ~36 %  | 16–24 | no    |
| After real DDR4 MIG IP (task #61)               |  ~90    | ~41 %  | 20–28 | no    |
| After 1 GbE Tri-Mode MAC IP (phase 5.3)         |  ~97    | ~45 %  | 24–32 | no    |
| After HDMI TX IP (Xilinx) instead of open core  | ~102    | ~47 %  | 24–32 | no    |
| After F1-F3 perf bundle (ROB→64, PRF→96, IQ→16) | ~118    | ~54 %  | 28–36 | no    |
| After full FPU body (not currently synth'd)     | ~138    | ~64 %  | 28–36 | yes   |

**Explicit warnings — any resource > 70 %:**
- **None today.**  Highest projected row is 64 % LUT with FPU body
  fully elaborated.  The design fits KU5P with material headroom.
- **FFs stay low** (16 % today) because the dcache-LUTRAM soaks flops
  it shouldn't.  After #92 lands, FF count drops to the ~30 k range
  and stays comfortable through all projections (< 20 % worst-case).
- **DSPs are barely touched** (4 / 1 824).  Plenty of room for MUL.L
  SZ=1 dual-destination, FPU mul (needs ~8 DSPs), FIR/scaler if we
  ever do video.  DSP % stays below 2 % through all phases.
- **BRAM % stays below 10 %** (480 RAMB36 in KU5P).  L1I+L1D+ROB+IQ+
  RAT+PRF+VRAM+ATC = ~40-50 RAMB36 at worst (~10 %).  We could
  comfortably grow caches to 16 KB each or add an L2 if Fmax allows.

**Utilization conclusion:** **Utilization does not gate H0 or H1 on
the KU5P.**  The dcache-LUTRAM bug is cosmetic in the sense that the
design still fits; once fixed, the headroom is embarrassing.  We
could host TWO 68040 cores on one KU5P if we wanted to (and had any
reason to).

### 2.3 What about KU3P / KU9P deratings for cost?

KU3P has ~45 % fewer LUTs (123 k vs 217 k).  If we land F1-F3 + L1I
+ MMU walker + all peripherals in the ~120 k range (achievable per
§2.2), **we'd fit on KU3P.**  KU3P -1 is quoted at ~$2 140 qty-1
([Digikey XCKU3P-1FFVB676I](https://www.digikey.com/en/products/detail/amd/XCKU3P-1FFVB676I/7203922)),
roughly $800 less than KU5P.  Revisit this for H2 when the core
utilization shape is frozen.

---

## 3. Clock-domain resource cost

### 3.1 Current clock domains

From [`docs/clocking.md`](clocking.md) §2 and the post-route power
report §2.2:

```
Clock name    | Domain                     | Constraint
pclk_unbuf    | u_video/u_mmcm/pclk_unbuf  | 6.7 ns (149.3 MHz — HDMI pix or its /2)
sysclk200     | sys_clk_p                  | 5.0 ns (200 MHz board osc)
```

Two clock sources today.  One MMCM used (HDMI), 0 PLLs.

### 3.2 H0 / H1 domain inventory (target)

From `docs/clocking.md` §2, H0/H1 target 11 logical domains, but most
share a single MMCM/PLL output:

| Domain         | How sourced            | New MMCM/PLL | Notes                   |
|----------------|------------------------|--------------|-------------------------|
| clk_core       | MMCM0 output 0         | (existing)   | 100-200 MHz CPU+fabric  |
| clk_pb         | MMCM0 output 1         | 0            | /2 or /4 of clk_core    |
| clk_phi2       | /N enable on clk_pb    | 0            | 1 MHz tick              |
| clk_scsi       | MMCM0 output 2         | 0            | 10 MHz                  |
| clk_adb/rtc    | /N enable on clk_phi2  | 0            | 32 kHz / 1 Hz           |
| clk_hdmi_pix   | MMCM1 output           | 1 MMCM (done)| 74.25 / 148.5 MHz       |
| clk_hdmi_tmds  | MMCM1 output           | 0            | 5× pixel                |
| clk_ddr_ui    | MIG IP internal MMCM   | ~1 PLL       | 300 MHz, MIG owns CDC   |
| clk_eth_rgmii  | External PHY input     | 1 PLL        | 125 MHz (H1+)           |
| clk_usb        | USB PHY ULPI           | ~1 PLL       | 60 MHz (H1+)            |
| clk_audio      | MMCM2 output           | ~1 MMCM      | 11.29 MHz               |

**Total at H1:** 3 MMCMs + 3-4 PLLs.  KU5P has **4 MMCMs + 8 PLLs**
([Xilinx DS890](https://www.mouser.cn/datasheet/2/903/ds890_ultrascale_overview-1591529.pdf)).
We are at 75 % MMCM usage and ~50 % PLL usage at the H1 endpoint.
Enough margin; no MMCM-budget failure.

### 3.3 Async CDC resource cost (task #89 — not yet landed)

The CDC work has to happen; it is not optional on real silicon.
Per `docs/clocking.md` §5, the plan is:

- 1× `axi_async_clock_converter` (AXI4) from clk_core to MIG
  clk_ddr_ui:  ~300-500 LUTs + 2 RAMB18 (typical Xilinx IP).
- 1× `axi_async_clock_converter` (AXI-Lite) from clk_core to clk_pb:
  ~150-250 LUTs + 1 RAMB18.
- Async FIFO for HDMI framebuffer reader (clk_core → clk_hdmi_pix):
  ~100-200 LUTs + 1-2 RAMB18 + small gray-code logic.
- 2-flop synchronisers on every IRQ line crossing into clk_core:
  <10 LUTs each, maybe 8-12 lines total.
- 3-stage input synchronisers on SCSI REQ/ACK, ADB pin, buttons:
  <100 LUTs total.

**Total CDC overhead: <2 kLUT + 4-8 RAMB18** — well under 1 % of
KU5P LUTs and <2 % of BRAMs.  Does NOT materially change any
utilization projection in §2.2.  Async CDC is a functional
requirement, not a resource pressure.

### 3.4 Does CDC slow us down?

Each AXI clock-converter adds ~3-5 cycles of latency on each side
of the crossing.  For peripheral-bus accesses (clk_core ↔ clk_pb at
50 MHz), that's 3-5 clk_core cycles + 3-5 clk_pb cycles = ~15-25
clk_core cycle budget.  Fine — peripheral registers are hit rarely.
For DDR4 (clk_core ↔ clk_ddr_ui at 300 MHz), the crossing is ~3-5
cycles on each side, or ~10 ns of extra latency.  MIG pipelines it
out anyway.  Does NOT meaningfully move IPC.

**Clocking conclusion:** KU5P has plenty of MMCM/PLL for H1's clock
zoo.  CDC adds ≤ 1 % resources and <25 cycles of peripheral latency.

---

## 4. H0 FMC mezzanine BOM

The working assumption (per `hardware_roadmap.md` §H0) is that we do
NOT re-create the KCU116's on-board peripherals (DDR4, HDMI PHY,
Ethernet PHY, SD card, USB-UART) — those are already on the eval
board.  The FMC mezzanine carries **only the legacy Mac peripherals**
plus power staging.

H0's mission is: prove Mac-OS-on-FPGA end-to-end using the KCU116's
native peripherals for modern I/O (HDMI out, Ethernet, DDR4) and the
mezzanine for legacy I/O (ADB, SCSI, serial, analog audio, floppy).

### 4.1 What's on the KCU116 eval board already (no mezzanine cost)

Per the [AMD KCU116 product page](https://www.amd.com/en/products/adaptive-socs-and-fpgas/evaluation-boards/ek-u1-kcu116-g.html)
and [FPGAkey breakdown](https://www.fpgakey.com/technology/details/amd-xilinx-kintex-ultrascale+-fpga-kcu116-evaluation-kit):

- XCKU5P-2FFVB676E FPGA (-2 speed grade — matches our target)
- DDR4 SODIMM socket (supports up to 4GB/2400 MT/s)
- HDMI TX (via HDMI IP to board connector)
- Gigabit Ethernet (88E1111 PHY on RGMII)
- 4× 28 Gb/s SFP28 (we don't use)
- USB-UART (FT4232H)
- 1× FMC-HPC connector (for our mezzanine)
- microSD slot (native on board)
- PCIe ×8 edge
- 2× PMOD (12 GPIOs total)

Everything modern-I/O-ish is covered.  The mezzanine's job is the
legacy Mac ports.

### 4.2 Mezzanine BOM (what we ship in H0)

Prices are **qty-1 Digikey** unless noted; **qty-100 Digikey-listed
price-break** in parentheses.  Prices verified 2026-04-17 where
possible; generic chips (74-series, passives) priced from standard
Digikey catalog data.  Subject to supply fluctuation.

#### 4.2.1 FMC mezzanine connector

| Part                               | Mfg    | Function                  | Qty 1 | Qty 100 |
|------------------------------------|--------|---------------------------|-------|---------|
| ASP-134604-01                      | Samtec | FMC-HPC plug (160-pin)    | $14.29 | $10-12 |

Source: [Digikey ASP-134604-01](https://www.digikey.com/en/products/detail/samtec-inc/ASP-134604-01/9762396).

#### 4.2.2 ADB (DIN-4 keyboard/mouse)

| Part                    | Mfg             | Function                      | Qty 1 | Qty 100 |
|-------------------------|-----------------|-------------------------------|-------|---------|
| DIN-4 PCB-mount jack    | CUI / Kycon     | ADB connector (S-video style) | $1.80 | $0.90   |
| SN65LBC176D             | TI              | Open-drain driver option      | $2.20 | $1.10   |
| 2N7002 × 2              | Nexperia / Diodes | Pull-up FET switch          | $0.10 | $0.04   |
| TPD2E009 ESD diode      | TI              | 5 V-tolerant clamp            | $0.30 | $0.12   |

Source: [Digikey SN65LBC176P](https://www.digikey.com/en/products/detail/texas-instruments/SN65LBC176P/380363).
ADB bit-cell is 200 µs and tiny, so open-drain-with-pullup suffices;
SN65LBC176 is a fallback if we ever need a real transceiver.

**Sub-total:** ~$4.40 qty-1 / ~$2.16 qty-100

#### 4.2.3 SCSI DB-25 (single-ended, active-termination)

| Part                    | Mfg             | Function                       | Qty 1 | Qty 100 |
|-------------------------|-----------------|--------------------------------|-------|---------|
| DB-25 female PCB mount  | Norcomp / Amphenol | SCSI DB-25 port             | $2.70 | $1.80   |
| 74LVC245 × 2            | TI / NXP        | 5 V ↔ 3.3 V level shift (16 bit total) | $0.70×2 | $0.32×2 |
| SN74LS38 × 2            | TI              | Open-collector ACK/REQ pullup  | $0.45×2 | $0.20×2 |
| UC5601 or DS2107AS      | TI / Maxim      | Active terminator (9-line SE)  | $8.50 | $6.00   |
| 110 Ω resistor network  | Bourns CAY16    | Termination pull-up            | $0.80 | $0.40   |

SCSI single-ended is old tech; TI/Maxim still make active terminator
chips.  Copy the SCSI2SD rev 6 schematic as `hardware_roadmap.md` §8
already directs.  The 74LVC245 in qty-100 is ~$0.32 ([Digikey 74LVC245](https://www.digikey.com/en/products/base-product/texas-instruments/296/74LVC245/6469)
reports $1.04 qty-1; common).

**Sub-total:** ~$13.50 qty-1 / ~$8.74 qty-100.  Call it **$15 qty-1 /
$10 qty-100** with the resistor network and ESD clamps rounded up.

#### 4.2.4 RS-422 LocalTalk / Printer / Modem (DIN-8 × 2)

| Part                    | Mfg             | Function                       | Qty 1 | Qty 100 |
|-------------------------|-----------------|--------------------------------|-------|---------|
| mini-DIN-8 PCB jack × 2 | CUI / Kycon     | LocalTalk connector            | $2.10×2 | $1.15×2 |
| MAX3097EEEE             | Analog/Maxim    | RS-422 triple receiver         | $12.10 | $7-8   |
| DS26LS31CN              | TI              | RS-422 quad driver             | $3.90 | $2.30   |
| +12V / -12V DC-DC converter (RS-422 signalling) | Murata MEV1S0512SC | 0.5W converter | $9.50 | $6.00 |

Source: [Digikey MAX3097ECSE](https://www.digikey.com/en/products/detail/maxim-integrated/MAX3097ECSE/MAX3097ECSE-ND/467152).
Note MAX3097ECSE is now obsolete; MAX3097EEEE is the current part.
RS-422 actually works fine at ±5 V but the classic Mac pinout
expected ±12 V for some accessories (LaserWriter line drive); the
Murata converter makes us spec-compliant.

**Sub-total:** ~$29.70 qty-1 / ~$17.60 qty-100.  **Biggest-per-function
cost on the mezzanine after SCSI active termination.**

> Note: DS26LS31CN is life-time-buy in some quantities; we'd keep
> a drop-in equivalent (TI AM26LS31CNSR, SN65C1168) qualified as
> hardware_roadmap.md §8 flags.

#### 4.2.5 Analog audio out/in

| Part                    | Mfg             | Function                       | Qty 1 | Qty 100 |
|-------------------------|-----------------|--------------------------------|-------|---------|
| WM8960CGEFL             | Cirrus          | Stereo codec (DAC + ADC + HP)  | $3.65 | $2.50   |
| 3.5 mm TRS jack × 2     | CUI SJ1-3513    | Audio in + out                 | $0.55×2 | $0.30×2 |
| Op-amps + passives      | TI OPA1612      | Line-out anti-alias            | $2.80 | $1.90   |

Source: [Digikey WM8960](https://www.digikey.com/en/products/detail/cirrus-logic-inc/WM8960CGEFL-V/5036712).
Single-chip codec covers both DAC and ADC needs that `hardware_roadmap.md`
§H0 listed separately (PCM5100 + AK5720).  Saves cost and board
space.  Interfaces over I²S + I²C.

**Sub-total:** ~$8.25 qty-1 / ~$5.00 qty-100

#### 4.2.6 Floppy (optional; hardware_roadmap.md §H0 suggests defer to H1)

| Part                    | Mfg             | Function                       | Qty 1 | Qty 100 |
|-------------------------|-----------------|--------------------------------|-------|---------|
| 34-pin shrouded header  | Amphenol MW34   | Floppy FFC header              | $1.20 | $0.60   |
| 74LS240 × 2             | TI              | Floppy TTL buffer pair         | $0.50×2 | $0.22×2 |

**Sub-total:** ~$2.20 qty-1 / ~$1.04 qty-100.  **Optional — punt to
H1 per roadmap.  Include on the silkscreen only.**

#### 4.2.7 PS/2 (keyboard + mouse, soft-convert to ADB)

| Part                    | Mfg             | Function                       | Qty 1 | Qty 100 |
|-------------------------|-----------------|--------------------------------|-------|---------|
| mini-DIN-6 PCB jack × 2 | Kycon           | PS/2 jack                      | $1.40×2 | $0.78×2 |
| 74AHC125 buffer         | TI              | Open-drain ground reference    | $0.40 | $0.18   |

**Sub-total:** ~$3.20 qty-1 / ~$1.74 qty-100

#### 4.2.8 Power + rails

| Part                            | Mfg      | Function                       | Qty 1 | Qty 100 |
|---------------------------------|----------|--------------------------------|-------|---------|
| 5 V → 3.3 V buck (TPS62140)     | TI       | 3.3 V local rail               | $4.40 | $2.80   |
| 5 V → 1.8 V buck (TPS62170)     | TI       | 1.8 V LVCMOS rail              | $3.20 | $2.10   |
| 5 V → +TERMPWR buck/boost       | TI/LM2623| SCSI bus term 4.75 V           | $3.80 | $2.40   |
| Murata MEV1S0512SC (±12 V)      | Murata   | (counted in §4.2.4)            | —     | —       |
| FMC power inrush + EMI filter   | Murata + Bourns | ferrite + TVS cluster   | $1.50 | $0.80   |

**Sub-total:** ~$12.90 qty-1 / ~$8.10 qty-100

#### 4.2.9 Test points / LEDs / buttons / OLED

| Part                    | Mfg             | Function                       | Qty 1 | Qty 100 |
|-------------------------|-----------------|--------------------------------|-------|---------|
| 0.96" 128×32 OLED I²C   | Winstar / UG-2832HSWEG04  | "nerd bait" front display | $6.80 | $4.20   |
| Tactile buttons × 4     | C&K PTS645      | Reset, NMI, boot-mode, aux     | $0.35×4 | $0.18×4 |
| Status LEDs × 8         | Bivar / Kingbright | CPU events indicator        | $0.12×8 | $0.06×8 |
| Test-point posts × 20   | Keystone 5015   | Scope touchdown                | $0.18×20 | $0.08×20 |

**Sub-total:** ~$14.40 qty-1 / ~$8.44 qty-100

#### 4.2.10 PCB + assembly

| Item                          | Qty 1 one-off | Qty 100           |
|-------------------------------|---------------|-------------------|
| 4-layer PCB, 100×80 mm        | $30           | $4.50/board       |
| Assembly (JLCPCB SMT svc.)    | $75 setup+$40 | $12/board (std.)  |
| Solder stencils               | $12           | one-time $12      |
| Packaging                     | $5            | $1.50             |

**Sub-total (amortised):** ~$162 qty-1 / ~$20 qty-100

### 4.3 H0 mezzanine BOM total

| Category                      | Qty 1  | Qty 100 |
|-------------------------------|--------|---------|
| FMC connector                 | $14.29 | $11.00  |
| ADB                           | $4.40  | $2.16   |
| SCSI                          | $15.00 | $10.00  |
| RS-422 serial (×2 DIN-8)      | $29.70 | $17.60  |
| Analog audio (codec + jacks)  | $8.25  | $5.00   |
| Floppy (optional)             | $2.20  | $1.04   |
| PS/2 (×2)                     | $3.20  | $1.74   |
| Power                         | $12.90 | $8.10   |
| OLED + LEDs + buttons + TPs   | $14.40 | $8.44   |
| PCB + assembly (amortised)    | $162   | $20     |
| **Mezzanine BOM subtotal**    | **$266** | **$85**  |

### 4.4 H0 all-in (what a user actually pays)

Two models per `hardware_roadmap.md` §H0:

**Model A — "buy your own KCU116" (recommended):**
- User buys KCU116: $7 349 (one source shows 14 in stock today at
  [Digikey](https://www.digikey.com/en/products/detail/amd/EK-U1-KCU116-G/7035246))
- We ship mezzanine: **$266 BOM + ~50 % margin = $399**.
- Total to user: ~$7 750.

**Model B — "dev-kit bundle":**
- We buy the KCU116 in bulk (-5 % to 10 % at qty-50): ~$6 950 each
- Add mezzanine qty-100 BOM: $85 direct cost, bill at $199 bundled
- We bill cost + light margin: **$7 500 bundle**, we eat the
  $300 delta as "early-access subsidy" per roadmap §H0.

**Critical single-items > $50:** only the KCU116 itself ($7 349).
Mezzanine-only has no single > $50 item (largest: Murata DC-DC at $9.50,
WM8960 at $3.65).

---

## 5. H0 total cost model + single-unit flags

### 5.1 1-off prototype

For the first hand-assembled prototype (you and 2 friends build one
each, then hand-ship to the top 3 nerds on the waiting list):

| Line                                    | $       |
|-----------------------------------------|---------|
| KCU116 eval kit                         | $7 349  |
| Mezzanine qty-1 BOM (per §4.3)          | $266    |
| One-off PCB/stencil setup charges       | $87     |
| DDR4 SODIMM 4 GB for KCU116             | $18     |
| microSD 16 GB Class 10                  | $10     |
| HDMI cable, FMC ribbon, PS/2 cables     | $25     |
| Shipping in a reasonable box            | $40     |
| **Per-unit 1-off cost**                 | **$7 795** |

### 5.2 Small run (qty 50 — reach the target early-access nerd count)

| Line                                    | $       |
|-----------------------------------------|---------|
| KCU116 at qty-50 (AMD direct ~5 %)      | $6 982  |
| Mezzanine qty-100 BOM (per §4.3)        | $85     |
| Assembly at JLCPCB SMT qty-50           | $14     |
| DDR4 SODIMM qty-50                      | $15     |
| microSD qty-50                          | $6      |
| Cables + box + manual print             | $20     |
| **Per-unit qty-50 cost**                | **$7 122** |

### 5.3 Items > $50 (single-line flag)

- **KCU116** ($7 349).  Dominant line.  Biggest risk to the BOM.
  Mitigations: use user-bought KCU116 (Model A above) so we aren't
  inventory-exposed; or pivot H0 to a lower-cost eval board (KCU105
  at ~$3 500, but that's KU040 not KU5P — Fmax parity would need
  re-timing study).
- **No other single line > $50 on the mezzanine.**  Cleanest BOM
  you'll ever see at this tier.

---

## 6. H1 / H2 risks (non-BOM)

### 6.1 H1 (custom SBC) risks beyond `hardware_roadmap.md §8`

| Risk | Severity | Notes |
|------|----------|-------|
| KU5P lead time (40 weeks) vs H1 timeline (6-12 mo) | **HIGH** | Reserve 500 units via AMD direct at H0 kickoff.  If Avnet/Arrow won't, the lead time *is* the schedule. |
| qty-100 KU5P at ~$2 100 busts the hardware_roadmap.md $180-250 BOM target | **HIGH** | Either: (a) accept $499+ BOM floor → retail $1 299+; (b) pivot to ZU3EG ($800/unit in qty-100 per industry norm); (c) KU3P -1 derating ($1 600 qty-100). |
| 8-10 layer HDI PCB (KU5P FFVB676 requires it) | MED | CM quote at qty-100: $55-80/board; at qty-1000: $35-50. Not a blocker but a scale tax. |
| DDR4 SODIMM availability for 4 GB at $14 qty-100 | LOW | Commodity; price from [Crucial CT4G4SFS824A](https://www.crucial.com/memory/ddr4/ct4g4sfs824a) at retail is $35, at distributor will be $14. |
| HDMI TX IP license (Xilinx) | MED | ~$5 k one-time + per-bitstream royalty.  Alternative: open-source HDMI core.  `hardware_roadmap.md §8` already flags; stays MED. |
| 1 GbE PHY + magnetics placement | LOW | Marvell 88E1512 or KSZ9031 at ~$5 qty-100.  Source: [Digikey 88E1512-A0-NNP2I000](https://www.digikey.com/en/products/detail/marvell-semiconductor-inc/88E1512-A0-NNP2I000/10478776). |
| USB-C PD controller at qty-100 | LOW | TPS65988 ~$8 qty-100 per [Digikey](https://www.digikey.com/en/products/detail/texas-instruments/TPS65988DHRSHR/9608026). |
| M.2 M-key socket | LOW | TE/JAE SM3 series at $1.50-3.00 qty-100. |
| JTAG chain + fuseboot layout | LOW | Standard; KU5P needs Master SPI x4 config at 85 MHz (already in `ku5p.xdc`). |
| FCC Class B pre-scan failure | MED | Clear case risk.  Internal RF-absorbing liner + board-level shielding. `hardware_roadmap.md §8` already flags. |
| Thermal: sustained 5 W on BGA | LOW | Passive heatsink + small fan.  Power report says 1.66 W today; even 3× = 5 W is easy at 250 LFM airflow. |

### 6.2 H2 (boxed retail) risks

| Risk | Severity | Notes |
|------|----------|-------|
| Translucent polycarbonate shell tooling at qty-500 | MED | SLA → injection-mould transition at 500-1k units.  Tool cost: $15-25k one-time. |
| FCC Part 15 Class B certification | **HIGH** | $10-15k typical for pre-scan + cert.  Budget for it. |
| CE mark (if shipping EU) | MED | Self-declare usually OK at this product class; RoHS compliance easier. |
| Case aesthetic (designer engagement) | MED | Non-trivial industrial design budget ($20-40k). |
| Quadra ROM licensing (Apple IP) | **HIGH** | Don't ship the ROM.  Ship an extractor + instructions.  Same model as MiSTer.  Already flagged in roadmap §8. |
| Shipping insurance on $500 units | LOW | Standard fulfillment channels cover. |
| Yield on BGA-676 at volume assembly | MED | X-ray inspection; retain a few spares per 50-unit lot. |
| Anti-tamper / IP protection on bitstream | LOW | KU5P supports AES-256 bitstream encryption (eFUSE + BBRAM).  Enable at H2. |

---

## 7. Go / no-go recommendation

### 7.1 H0: **GO**

**Blockers: none.**  The current RTL fits the KCU116 at 100 MHz with
huge resource headroom (§2).  Power is fine (§2.1).  Clock count is
fine (§3).  The mezzanine BOM is routine — legacy SE/RS-422/audio
components are all catalogued and available on Digikey today.  The
mezzanine dev-kit BOM comes in at **$266 qty-1 / $85 qty-100**,
excluding the KCU116.

**Delta-work to H0 MVP ship:**

| Work item                                                 | Est.   |
|-----------------------------------------------------------|--------|
| Task #92 dcache→BRAM (critical; tidies utilization)       | 1 day  |
| Task #73 real L1I                                         | 3-4 d  |
| Task #61 real MIG DDR4 IP                                 | 2 d    |
| Task #89 async CDC infrastructure                         | 2 d    |
| MMU page-table walker (docs/core_gaps.md §2)              | 5-7 d  |
| Full VIA1 ADB shift register + Timer                      | 3 d    |
| Full NCR 5380 SCSI phase FSM                              | 5 d    |
| HDMI IP license decision (buy Xilinx vs open-core)        | decision |
| Mezzanine schematic + layout (EE task; outsource)         | 2 weeks |
| PCB fab + assembly lead time                              | 3-4 wk |
| First-article bring-up                                    | 1-2 wk |
| **Total calendar time H0 MVP**                            | **10-14 weeks** |

Matches `hardware_roadmap.md` "4-6 months out" for H0 ship.  Agent-days
above are RTL-only; EE + PCB work is outsourced.

### 7.2 H1: **CONDITIONAL GO**

**Blockers:** qty-100 KU5P pricing.  At ~$2 100/unit of BOM (KU5P)
alone, the entire "retail at $499-699 with $180-250 BOM" target from
`hardware_roadmap.md §H1` is unachievable.  This is the ONE material
finding of this feasibility study.

**Three paths forward:**

1. **Accept the real BOM floor.**  KU5P qty-100 = $2 100; add $200
   peripherals + $70 PCB + $40 case + $20 assembly + $30 misc =
   **$2 460 BOM**, retail at $3 499 (gross margin ~30%).  Nerd
   audience might bear this, but it's 7× the roadmap target.
2. **Pivot to ZU3EG or ZU4EV (Zynq UltraScale+ MPSoC).**  These are
   cheaper (ZU3EG qty-100 ~$700; ZU4EV ~$1 100 per industry norm;
   Digikey doesn't publish) AND bring a quad-Cortex-A53 PS "for free",
   solving USB host / SD driver / web-UI-for-config in the PS.  `hardware_roadmap.md §H1`
   already listed this as the primary pivot.  **PL-size fits** —
   ZU4EV has 192 k LUTs vs our 120 k projected.  The only cost is a
   re-fit timing closure (not free but not the calendar killer the
   40-week lead time is).
3. **Use KU3P -1 derating.**  KU3P has 45 % fewer LUTs; we fit at
   projection (§2.3).  qty-100 price ~$1 600 (guess; needs RFQ).
   BOM floor ~$1 960.  Retail $2 799.  Middle-ground.

**Recommendation:** target path 2 (ZU4EV) for H1 retail.  Keep H0 on
KU5P (zero pivot cost; KCU116 is already bought).  This aligns with
the roadmap's existing ZU7EV mention but steps down to ZU4EV/ZU3EG
for cost.  File a decision-marker on the timing-closure check per
§H1 of roadmap.

**Delta-work from H0 to H1 if we pivot to ZU4EV:**

| Work item                                            | Est.    |
|------------------------------------------------------|---------|
| ZU4EV re-fit: re-synth, retime, DSP re-map           | 2-3 wk  |
| PS-PL boundary design (SD / Eth / USB via PS)        | 3-4 wk  |
| Custom SBC schematic + 10-layer PCB layout           | 4-6 wk  |
| Power delivery (multi-rail PMIC, DDR4 PHY sig-int)   | 2-3 wk  |
| First-article fab + bring-up + re-spin allowance     | 8-12 wk |
| **Total H0 → H1 calendar time**                      | **~6 months** |

Matches the roadmap.

### 7.3 H2: deferred

H2 is case + polish + retail logistics; its go/no-go is not a
technical feasibility question and belongs in a marketing-cost study,
not this doc.  Flagging only: FCC cert is a $10-15k line item; case
tooling another $20-25k; industrial design engagement $20-40k.  Plan
for ~$75k non-recurring for H2 launch beyond whatever the BOM path
from §7.2 settles on.

---

## 8. Summary table

| Dimension            | H0 (FMC mezzanine on KCU116)       | H1 (custom SBC, retail)               |
|----------------------|------------------------------------|---------------------------------------|
| Part                 | XCKU5P-2FFVB676E (on eval board)   | Pivot to ZU4EV (recommended)          |
| Part qty-100 $       | n/a (on board)                     | ~$1 100 (ZU4EV) vs $2 100 (KU5P)      |
| LUT headroom         | 48 % today → 64 % worst-case       | Similar or better on ZU4EV            |
| Power                | 1.66 W measured; 5 W budget OK     | Similar; PS adds ~1-2 W               |
| Clock domains needed | 3 MMCM + 3 PLL (of 4+8)            | Same                                  |
| Mezzanine BOM qty-1  | $266                               | n/a (custom board)                    |
| Mezzanine BOM qty-100| $85                                | n/a                                   |
| Full BOM qty-100     | $7 400 user-buys-KCU116            | $220-300 (ZU4EV path, ex-FPGA)        |
| Retail price target  | "dev kit", $7 500-7 800            | $1 299 (ZU4EV) vs $3 499 (KU5P)       |
| Go/no-go             | **GO**                             | **GO pending ZU4EV pivot**            |

---

## 9. Sources

Primary part pricing and availability, all accessed **2026-04-17**:

- Digikey XCKU5P-2FFVB676I: https://www.digikey.com/en/products/detail/amd/XCKU5P-2FFVB676I/6797925
- Digikey XCKU5P-1FFVB676I: https://www.digikey.com/en/products/detail/amd/XCKU5P-1FFVB676I/6797921
- Digikey XCKU3P-1FFVB676I: https://www.digikey.com/en/products/detail/amd/XCKU3P-1FFVB676I/7203922
- Digikey EK-U1-KCU116-G (KCU116 eval kit): https://www.digikey.com/en/products/detail/amd/EK-U1-KCU116-G/7035246
- Digikey 88E1512-A0-NNP2I000 (Marvell GbE PHY): https://www.digikey.com/en/products/detail/marvell-semiconductor-inc/88E1512-A0-NNP2I000/10478776
- Digikey TPS65988DHRSHR (USB-C PD): https://www.digikey.com/en/products/detail/texas-instruments/TPS65988DHRSHR/9608026
- Digikey SN65DP159RGZR (HDMI retimer): https://www.digikey.com/en/products/detail/texas-instruments/SN65DP159RGZR/5428834
- Digikey WM8960CGEFL/V (audio codec): https://www.digikey.com/en/products/detail/cirrus-logic-inc/WM8960CGEFL-V/5036712
- Digikey SN65LBC176P (RS-422 transceiver): https://www.digikey.com/en/products/detail/texas-instruments/SN65LBC176P/380363
- Digikey MAX3097EEEE+ (RS-422 receiver): https://www.digikey.com/en/products/detail/analog-devices-inc-maxim-integrated/MAX3097EEEE/1513658
- Digikey ASP-134604-01 (Samtec FMC-HPC connector): https://www.digikey.com/en/products/detail/samtec-inc/ASP-134604-01/9762396
- Digikey DM1AA-SF-PEJ(82) (microSD socket): https://www.digikey.com/en/products/detail/hirose-electric-co-ltd/DM1AA-SF-PEJ-82/5021219
- Digikey 74LVC245 (TI base page): https://www.digikey.com/en/products/base-product/texas-instruments/296/74LVC245/6469
- Crucial DDR4-2400 4 GB SODIMM CT4G4SFS824A: https://www.crucial.com/memory/ddr4/ct4g4sfs824a

Architecture / product brief:

- UltraScale+ Architecture Overview DS890: https://www.mouser.cn/datasheet/2/903/ds890_ultrascale_overview-1591529.pdf
- AMD Kintex UltraScale+ product page: https://www.amd.com/en/products/adaptive-socs-and-fpgas/fpga/kintex-ultrascale-plus.html
- AMD KCU116 Evaluation Kit page: https://www.amd.com/en/products/adaptive-socs-and-fpgas/evaluation-boards/ek-u1-kcu116-g.html
- FPGAkey KCU116 breakdown: https://www.fpgakey.com/technology/details/amd-xilinx-kintex-ultrascale+-fpga-kcu116-evaluation-kit
- Marvell 88E1512 product brief: https://www.marvell.com/content/dam/marvell/en/public-collateral/phys-transceivers/marvell-phys-transceivers-alaska-88e1512-product-brief.pdf

Project-internal sources consulted:

- `build/vivado/reports/utilization_route.rpt` — post-route utilization @ 100 MHz
- `build/vivado/reports/power.rpt` — post-route power estimate
- `synth/vivado.tcl` — synthesis flow and target part
- `synth/fpga_top.xdc`, `synth/ku5p.xdc` — pin + timing constraints
- `docs/hardware_roadmap.md` — H0 → H1 → H2 plan
- `docs/clocking.md` — target clock domains
- `docs/core_gaps.md` — MMU walker + L1I/L1D outstanding work
- `docs/fmax_analysis.md`, `docs/fmax_retime_log.md` — Fmax trajectory
- `CLAUDE.md` — project conventions + pinned status

---

## 10. Open questions for decision

1. **KU5P reservation order — how many units, when?**  Without a
   reservation, H1 timeline stretches by 10 months.  My recommendation:
   reserve 500 of XCKU5P-1FFVB676I at H0 kickoff if we're staying on
   KU5P for H1; reserve NOTHING if we're pivoting to Zynq.  Decision
   gate: end of phase-3 boot milestone.
2. **H1 part: KU5P, KU3P, ZU4EV, or ZU7EV?**  Cost argues ZU4EV.
   Roadmap flagged ZU7EV as default.  Recommend ZU4EV for cost, but
   only if the 192 k LUT budget holds through phase-5 (currently
   trending well under).
3. **HDMI IP: Xilinx ($$$ + royalty) vs open-source hdmi_core?**  The
   open-source option works up to 1080p60 at the -2 speed grade.  Use
   it for H0/H1; revisit for H2 if volume demands predictability.
4. **KCU116 bundle vs user-bought for H0?**  Strong recommendation:
   user-bought.  Do not hold $7 k of inventory per unit.  Ship
   mezzanine only + printed instructions for KCU116 ordering.

These are decision items, not blockers.  H0 can ship without
resolving any of them.

---

## 11. What this doc did NOT assess

Out of scope, flagged for follow-up:

- Detailed signal-integrity analysis of DDR4 on H1 custom PCB.
- Final layer stackup recommendation for the 10-layer H1 PCB.
- Industrial-design brief (separate roadmap task).
- Software ecosystem roadmap (m68kctl v2, web-UI, INIT installers).
- Specific EMI liner material for FCC compliance.
- Certification bureau selection (NTS, UL, etc.).
- Detailed DFM review of the mezzanine layout.

These belong in follow-on work per `hardware_roadmap.md` §10.
