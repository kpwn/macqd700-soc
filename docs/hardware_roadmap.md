# Hardware roadmap — from FMC mezzanine to shippable box

Planning doc (no implementation yet).  Companion to [`gameplan.md`](gameplan.md)
(software/RTL phases), [`optimisation_roadmap.md`](optimisation_roadmap.md),
[`uarch_proposals.md`](uarch_proposals.md), and [`peripheral_arch.md`](peripheral_arch.md).

---

## 1. Product vision

**One sentence:** a 68040-compatible Mac on an FPGA, running unmodified
Mac OS 6/7/8 at 10× Quadra speed, that plugs into a real keyboard from
1987 AND into USB-C, HDMI, and Gigabit Ethernet.

**Audience:** nerds.  Not "nerds" the marketing persona — actual people
who own a Quadra 840AV, a Performa 6200, a pile of ADB mice, a SCSI
Zip drive, and can't decide between TenFourFox and System 7.5.5 for
their daily driver.  The product exists to make them say "holy
shit" when it boots.

**What wows this audience:**

1. It's an actual, faster 68040.  Not emulated in software on a Pi — a
   silicon OoO CPU doing what no real 68040 chip ever did.
2. It speaks ADB and SCSI for real.  Plug in your 1989 Extended Keyboard
   II; it works.  Plug in a SCSI2SD; it works.
3. It also speaks USB-C, Gigabit Ethernet, and HDMI.  So you can use it
   as your daily machine without a VGA monitor from eBay.
4. It's hackable.  FPGA reflashable from host.  Every pipeline event
   you care about visible on LEDs and/or an OLED.  The case is clear.
5. It's honest.  Case ships with a schematic printed on the bottom.  BOM
   on the GitHub page.  Everything publishable is published.

**Explicit non-goals:**

- Not a universal retrocomputing platform.  MiSTer already does that.
  This is 68k-Mac-maximalist.
- Not a museum replica.  We are NOT shipping a Quadra cosmetic clone.
  New aesthetic, honours the lineage, doesn't cosplay.
- Not cheap.  $400–$800 range accepted.  Nerds who buy 68k Macs off
  eBay for $300 with a bad cap leak will pay this for one that
  outperforms 10× the real thing.

---

## 2. Phase progression

Four hardware phases, each a coherent deliverable.  Software/RTL work
(phases 1–4 in `gameplan.md`) gates the order.

### H0 — Dev kit on existing eval board (now)

**Carrier:** AMD/Xilinx **KCU116 Evaluation Kit** (XCKU5P, same part as
our targeting).  Already has: DDR4 SO-DIMM, HDMI TX, Ethernet PHY,
USB-UART, PCIe edge, FMC-HPC connector, SD card slot, 200 MHz clock.

**Mezzanine:** **FMC-HPC "Mac legacy I/O" card** — custom, but one-layer
two-sided PCB, no fancy RF, single day of layout for a tolerable-quality
engineer.  Carries:

- **ADB**: 1× DIN-4 jack, open-drain driver with pull-up, 5 V tolerant
  input buffer, optional isolation MOSFET so it can present the 200-mA
  pull-up a 1989 keyboard expects.
- **RS-422 LocalTalk/printer/modem**: 2× DIN-8 jacks with a 26LS32/34
  transceiver pair.  Can run LocalTalk, an Apple ImageWriter, or a
  classic modem.
- **SCSI DB-25**: 1× DB-25 with active-termination.  FPGA drives the
  single-ended 5 V signals through 74-series buffers.  Can host any
  SCSI-1/2 peripheral including SCSI2SD, Zip/Jaz, external hard disks,
  and Apple's Scanner.
- **Audio**: 1× stereo 3.5 mm out (PCM5100 DAC), 1× 3.5 mm mic in (AK5720
  ADC).  Same Apple-PlainTalk impedance as the 660AV.
- **Floppy** (stretch): 1× shrouded 34-pin header for an external 1.44 MB
  Superdrive.  If we ship this on H0, we can boot from a real floppy.
  Honestly, skip for H0 and add in H1.
- **PS/2**: 2× mini-DIN (keyboard + mouse) — because plenty of nerds
  have PS/2 lying around and the protocol is cheap.  Soft-converts to
  ADB in firmware.
- **Test points + LEDs + a physical NMI button + power/reset/interrupt
  switches**.  Nerd-bait.
- **Power**: takes 5 V + 12 V from the FMC connector.  SCSI +TERMPWR rail
  generated locally through a small buck-boost.

**Form factor:** the KCU116 + FMC mezzanine sits on a 3D-printed tray;
not a finished case.  Ships to maybe 20-50 early-access nerds as "dev
kit" priced at cost-plus (probably $1200–$1500 all-in because the
KCU116 is ~$3000 MSRP; we subsidise).  Alternatively: ship JUST the
FMC mezzanine + BOM + instructions, and let people buy their own
KCU116 (or KCU105 with deratings).  That's the honest path; keeps
inventory risk low.

**Deliverable signals**: boots System 7 off SD, mouse works, keyboard
works, HDMI output looks right, Ethernet via some MacTCP driver we
write.

**First board test:** run the platform at 50 MHz core speed first
(`CORE_CLK_DIVIDE=4`, `CORE_CLK_HZ=50_000_000`) before trying to
close a faster build.  That gives us a known-good clock/reset path and
keeps the MIG work separate from frequency chasing.

**DDR readiness checklist:** the first real-MIG bitstream still needs the
known-good `~/FPGA/pcie_test` contract to stay green:
`make ddr-pcie-test-check` for pins/reset/MIG geometry and
`make tb-axi-ddr4-mig-bridge` for the repo 128-bit/6-ID/32-address to
pcie_test 256-bit/1-ID/31-address AXI shim.  The bridge is intentionally
standalone until the MIG UI clock-domain and burst policy are settled; until
then, treat `make ddr-pincheck` as the only DDR hardware smoke test.

### H1 — Custom single-board (6–12 months)

Once H0 is stable across 20+ users, commit to a custom PCB.  Two
form-factor candidates:

**Option A: set-top-box ("the shoebox")** — 8×5×2 inches.  Mac-mini-ish.
Desktop-friendly.  HDMI/USB/Ethernet on the back, ADB/serial/audio on
the front-or-sides.  Power brick (no internal PSU → fewer FCC headaches).

**Option B: half-height PCIe card** — plugs into a modern PC, uses the
host as a "power + screen + storage" substrate while the FPGA runs
Mac OS in its own window via a PCIe-shared framebuffer.  Nerds love
this because it's an actual Mac that lives in their gaming rig.  But
it's niche; it's not "the" product.  Consider as a SKU, not the flagship.

**Flagship: Option A.**

**SoC-grade Zynq pivot vs KU5P staying:**

KCU116 stays for H0.  For H1, consider moving to **Zynq UltraScale+
MPSoC** (e.g., ZU7EV).  Reasons:
- Built-in Arm Cortex-A53 ×4 for host-side tasks (SD driver, Ethernet
  stack, web UI for config) — saves us writing a bare-metal host.
- Built-in hard PCIe, USB-3, DisplayPort, SATA.
- Comparable PL (programmable logic) to KU5P in the ZU7EV/ZU9EG part.
- Downside: different die, re-fit timing; licensing cost per part.

**Decision marker for H1 kickoff:** run 1× benchmark on the current
KU5P design that confirms we can fit the core + caches + peripherals
comfortably under 60% utilisation.  If yes, staying on KU5P saves
~6 months.  If no, pivot to Zynq ZU7EV for more LUTs + the Arm
processing system.

**Expansion:** ONE M.2 M-key slot for NVMe (for disk images / SD
replacement) and ONE mini-PCIe for WiFi/BT/something.  Keep it.

**BOM target:** $180–$250 in qty 100.  Retail at $499–$699.

### H2 — Full boxed product with case (12–24 months)

**The case is the headline.**

Design brief for industrial design:
- **Translucent or clear polycarbonate** main shell.  The FPGA is visible.
  Think late-90s iMac but sharper.  LED strips inside illuminate the
  internals — not RGB gamer trash; warm-white, with per-LED tie to
  actual hardware events (more on this in §4).
- **Vintage port cluster on one side** — ADB, DB-25 SCSI, two DIN-8
  serial, stereo mini-jacks.  Labelled with small icons styled after
  the period's System 7 dialog graphics.
- **Modern port cluster on the other side** — HDMI, USB-C ×2, RJ-45
  Gigabit, microSD, USB-A ×2 for keyboards, power-in (USB-C PD or
  barrel jack).
- **Front face** — small OLED (128×32), physical reset button, physical
  NMI button, physical "programmer's switch" (the original
  interrupt/reset nub from the 68k Mac era — people will freak out).
  A discrete power LED and a "HDD activity" LED that's actually driven
  by our L1D miss rate.
- **Top:** no vents needed (KU5P TDP ~10 W in our config) — add a small
  passive finned heatsink inside, visible through the top, branded.

**Case material**: translucent polycarbonate + CNC'd aluminum base for
rigidity + heat spread.  Not 3D-printed — we're past that phase.

**Kit option:** ship the PCB + all the case parts as a kit for $100 off
retail.  Nerds assemble.  Also ships with a signed schematic.  Kit
SKU exists because assembling is half the fun and builds community.

**Branded accessories:** an official matching ADB keyboard (reshelled
Cherry MX board with DIN-4 cable) — stretch goal, probably post-launch.

### H3 — Beyond

Expansion card slot (custom FPGA-to-FPGA protocol on a 1× PCIe edge)
so the community can build add-on daughterboards.  Think of it as
"our NuBus".  First-party daughterboards: faster Ethernet (10 GbE
via our own SFP+ board), video out (multiple HDMI for multi-monitor
Mac OS), music (JMAX-ADAT digital audio I/O).

---

## 3. Port inventory — legacy + modern, rationalised

### Legacy — all worth the board space

| Port | Protocol | FPGA logic cost | Why it ships |
|---|---|---|---|
| **ADB (DIN-4 × 1)** | 125 kbps open-drain bidir | <500 LUT | Keyboards + mice; this is the identity port |
| **SCSI (DB-25 × 1)** | 5 V SE, async + sync (≤10 MB/s) | ~3 kLUT | Real peripherals incl. SCSI2SD, tape, CD, scanner |
| **Serial / LocalTalk (DIN-8 × 2)** | RS-422 230.4 kbps | <1 kLUT each | Printer, modem, LocalTalk, Pi-fronted ADB bridge |
| **Analog audio out (3.5 mm)** | DAC-fed line-out | PCM5100 ext. | Nerd requirement |
| **Analog audio in (3.5 mm)** | ADC-fed line/mic | AK5720 ext. | PlainTalk nostalgia + podcast input |
| **Floppy (34-pin shrouded header)** | GCR 500 kbps | ~2 kLUT | Real disks; FluxEngine-compat |

### Modern — all worth the board space

| Port | Protocol | FPGA logic cost | Why it ships |
|---|---|---|---|
| **HDMI out ×1 (type-A)** | TMDS 1080p60 | ~5 kLUT + TMDS IO | Actual usable display |
| **USB-C ×2** | USB 2.0 high-speed host (Zynq PS path) | external PHY | Modern keyboards + mice + storage |
| **USB-A ×2** | USB 2.0 host | external PHY | Legacy USB keyboards still exist |
| **Gigabit Ethernet (RJ-45)** | RGMII to 88E1111 PHY | ~3 kLUT + MAC IP | Networking without a NuBus card |
| **microSD slot** | SDIO 50 MHz | ~1 kLUT | Storage for disk images |
| **M.2 M-key** | PCIe Gen3 ×2 to NVMe | SoC-integrated | Fast virtual disk; future-proof |
| **DisplayPort or 2nd HDMI** | 1.4 | ~5 kLUT | Multi-monitor Mac OS support |

### Debug — not end-user but needs board space

| Port | Use |
|---|---|
| **JTAG (10-pin 0.1" header)** | Bitstream loading, chipscope |
| **PCIe ×4 (edge, optional on H1 set-top)** | Host-side debug (m68kctl / XDMA) |
| **UART over USB-C (USB-UART chip)** | Serial console for bring-up |

### Conspicuously NOT shipping

- **NuBus** — no one has NuBus cards anyway, and emulating slot access
  in software Toolbox is easier than shipping the physical connector.
- **Apple Desktop Video (AAUI / AV)** — niche; use HDMI.
- **PDS (processor direct slot)** — model-specific; not worth it.
- **S-Video / composite out** — Use an external HDMI→composite box if
  you want it on a CRT.  Not shipping a CVBS DAC.
- **DIN-8 external HDD** — subsumed by SCSI DB-25.
- **Stereo RCA pair** — we have 3.5 mm; buy an adapter.

---

## 4. The "nerd wow" features — implementation plan

### Translucent case + functional LEDs

The LED bar inside is not RGB eye-candy.  Each LED is tied to a
pipeline event:

- LED 1: fetch active  (on = front-end not stalled)
- LED 2: LSU busy  (on = memory in flight)
- LED 3: branch mispredict  (blinks on `flush_en` with a 50 ms latch)
- LED 4: D-cache miss  (on = any in-flight load with >1 cycle latency, once real L1D lands)
- LED 5: IRQ active  (on = SR.I > 0)
- LED 6: supervisor mode  (on = SR.S)
- LED 7: sync-exception-in-progress
- LED 8: IPL indicator  (dim = 0, bright = 7)

Wired in `rtl/mac/debug_leds.v` (new module — not implemented yet).
Host tool `m68kctl leds` can force patterns for LARP purposes.

### The OLED boot display

128×32 I²C OLED on the front.  During boot:

```
 [ m68k-ooo v1.0       ]
 [ cpu 197 MHz         ]
 [ ram 128 MB          ]
 [ rom Quadra 840AV    ]
```

Post-boot cycles through live stats: uptime, IPC sliding-window, packets/sec,
most-recent trap vector seen, SD card temperature.  Total added cost:
~$3 (display) + ~2 kLUT (I²C driver + text framebuffer) + ~200 LOC of
Zynq PS firmware.

### Physical buttons — "programmer's switch" revival

- **Reset** — asserts RESET# to everything.
- **NMI** — fires vector 7 or vector 31 depending on SR.  The original
  Mac's "interrupt" button that dropped you into MacsBug.  Shipping
  this is the ACTUAL "hello fellow nerds" moment.
- **Boot mode switch** — 3-position rotary: Normal / Sad-Mac-Debug /
  Flash-Recovery.

### HDMI boot splash

Before RAM sizing completes and the ROM kicks in, show a colour test
pattern on HDMI.  Could be an animated bouncing Claude Code splash
("made with agents" tongue-in-cheek).  Comes from a tiny PL-side
framebuffer before the CPU has RAM.

### Bottom-panel schematic + signature

The case's bottom aluminum plate is engraved with:
- Block diagram of the FPGA design.
- The list of contributors (Claude 4.7 and friends).
- The BOM.
- A QR code to the GitHub repo.

### "Clear mode" firmware toggle

Hold the NMI button during boot → FPGA front-panel displays the OoO
pipeline graphically on HDMI while the CPU runs.  You can literally
watch instruction retirement.  Absurd.  Worth it.

---

## 5. FPGA platform decisions

| Phase | Part | Why |
|---|---|---|
| H0 | KCU116 (XCKU5P) | Existing target; minimal pivot |
| H1 | KU5P direct OR ZU7EV (Zynq MPSoC) | Decision pending utilization check |
| H2 | same as H1, possibly a smaller derating | Cost reduction for retail volume |

**Key decision H0 → H1**: do we want an Arm subsystem?

PRO Arm:
- No bare-metal C host stack to write for SD/USB/Ethernet.
- Can run Linux for board-management / web UI / OTA updates.
- Standard PCIe/USB3/DP/SATA hard blocks save LUTs.

CON Arm:
- $$ per part.
- Timing closure harder across PS-PL boundary.
- Adds complexity for pure "CPU on FPGA" purists.

**Recommendation:** pivot to ZU7EV for H1 IF the H0 timing closure on
KU5P fails the 100 MHz gate.  Otherwise stay on KU5P; use a small
RP2040 or ESP32-S3 as the board-management coprocessor (handles LEDs,
OLED, fan, power sequencing).  Keeps the "no ARM" purity intact.

### External components needed regardless of FPGA

- **DDR4 SO-DIMM socket**  — 128 MB or 256 MB, user-upgradeable; nerds
  want the socket.  (Alt: soldered for cost, but the socket is part
  of the brand — "you can upgrade RAM on your Quadra".)
- **SPI flash × 2** — one for bitstream, one for Mac ROM image.
- **HDMI redriver** (e.g., TI SN75DP130) — 2× downstream HDMI outs.
- **Ethernet PHY** (Marvell 88E1111 or similar RGMII).
- **USB-C switching** (TPS65988 or similar) — PD power in + data.
- **Audio CODEC** (PCM5100 DAC + AK5720 ADC, or a single combined part
  like the WM8960).
- **ADB transceiver** — SN65LBC176D or discrete FET pull-up.
- **SCSI transceivers** — 2× 74LVC2244 + 2× SN75LBC176 for single-ended.
- **Level shifters for floppy and DIN-8 serial.**
- **Board-management MCU** (RP2040 recommended; $1 part).

### Power

~10 W peak on the FPGA, ~5 W on peripherals, ~5 W on DDR.  Call it
25 W total budget.  Single USB-C PD input at 20 V × 2 A.  Barrel jack
as fallback.  No internal PSU, no FCC headaches, all the compliance
on the brick.

---

## 6. Software ecosystem

This isn't hardware per se but it's what actually makes the box wow.

### `m68kctl` host tool (already prototyped)

- `m68kctl flash bitstream` — reload FPGA.
- `m68kctl flash rom <quadra_700.rom>` — swap CPU ROM (default
  target; `files/420dbff3.rom` is the committed Q700 Universal ROM).
- `m68kctl sd upload <disk.hdv>` — push a disk image.
- `m68kctl trace start/stop/dump` — PC-trace from debug_ctrl.
- `m68kctl led set <pattern>` — show off.
- `m68kctl fpu selftest` — when we land it.

### Web UI served by the board

Tiny HTTP server on the board-management MCU (or Zynq PS) listens on
port 80 over USB-C (RNDIS/ECM) or Ethernet:
- Shows uptime, IPC, temp.
- Upload/download disk images.
- Swap ROM image.
- Screenshots of HDMI output (for remote debugging).
- Console access to a serial-over-HTTP MacsBug.

### OS support image

We ship an SD card with:
- A pre-built HDV disk image of System 7.5.5 with our "welcome" package.
- An INIT installer that enables fast-mode cache policies, the driver
  for our virtual NVMe SCSI HBA, the ADB-over-USB bridge service, and
  the LED-control CDEV (so Mac OS software can blink our LEDs).
- A MacTCP config for our Ethernet.
- A sample folder with MacPaint, HyperCard, Dark Castle, whatever is
  freely redistributable.

---

## 7. Production + distribution model

**H0 dev-kit delivery:** direct shipping from a small CM (Circuit Hub,
or a friend-of-the-founder PCB house).  20-50 units.

**H1 first custom run:** 100-250 units.  Small US-based CM or
Shenzhen prototype house.  Hand-test + ship.  Retail direct only,
no Amazon.

**H2 retail:** 500-1000 unit runs.  Crowdfunding-style pre-orders
(Crowd Supply is the natural fit for this audience — they sell
MiSTer, ClockworkPi GameShell-likes, Open Book, and similar nerd
hardware).  Actual retail CM partnership.

**Kit SKU:** ships with PCB + tested bitstream on flash + all
components in antistatic.  Assembly docs as a printed 48-page
book (nerds love paper books for this stuff).

**Community:**
- Full schematic + gerbers on GitHub (MIT or CERN-OHL-P license).
- RTL stays on GitHub with the same pace as today.
- Discord server.  Bi-weekly "office hours" video call.
- Owners manual as an HTML "Classic Mac System 7" styled site.

---

## 8. Risks + open questions

| Risk | Severity | Mitigation |
|---|---|---|
| SCSI DB-25 is fiddly, needs 5 V-tolerant SE termination done RIGHT | MED | Copy the topology from SCSI2SD rev 6 exactly.  Build 5 prototypes. |
| ADB protocol timing is tight; bit-banging from PL must respect 200 µs bit cells | LOW | Plenty of open RTL reference (Emile Rétrocampus, Amiga Buffee) |
| HDMI clocking on KU5P needs Xilinx HDMI TX IP license ($$) | MED | Use open-source `hdmi-rs` or `hdmi_core` from OpenCores initially; buy the IP if sales grow |
| LocalTalk/RS-422 drivers are hard to source; some Maxim parts are life-time-buy | MED | Design with drop-in 26C31/26C32 equivalents; keep schematic generic |
| KU5P supply chain availability (AMD/Xilinx lead times 20+ weeks) | HIGH | Reserve 500 units via AMD direct; have a ZU7EV fallback schematic ready |
| FCC Class B compliance for a clear-plastic case (emissions leakage) | MED | Internal RF-absorbing liner behind the clear; pre-scan at NTS |
| Cost of translucent polycarbonate shell at low volume | MED | Start with clear 3D-printed SLA → move to IM only at H2 volume |
| CPUSH/CINV Mac OS expectations for real L1D with coherence-to-DMA | HIGH | Software-managed flush via Mac OS driver (no snoop in H1; add in H2+) |
| SCSI is single-initiator; host-side tools might not coexist cleanly | LOW | Board-management MCU owns SCSI bus; Mac OS is the only initiator |
| Floppy drive disposition — nobody has one | LOW | Stretch goal; skip H0, ship as accessory |

### Open questions needing user decision (eventually)

1. **Zynq or no?** — affects every downstream decision.  Decide after
   H0 timing closure.
2. **One HDMI or two?** — two doubles the "wow" (multi-monitor Mac OS)
   but costs board space and a 2nd HDMI IP license.  Defer to H1 brief.
3. **Include a real floppy port or not?** — strong nerd signal, real
   BOM + placement cost.  My vote: yes, but only on H2 and as an
   optional daughterboard on H1.
4. **Price point?** — $499 positions against a Raspberry Pi 5 + retro
   hat combo; $699 signals "real" product.  My vote: $599 for H1
   direct; $799 retail at H2.
5. **Kit vs finished** — kit-only at H1 (select nerd audience) vs kit
   + finished at H2 (broader but still nerd-flavoured).  Default yes
   to both.
6. **Ethernet over USB-C (RNDIS) as fallback if no RJ-45?** — saves
   board space if we drop RJ-45.  My vote: ship both; the RJ-45 is
   part of the identity.
7. **Licensing of the Quadra ROM image** — we cannot ship the Apple ROM.
   Users will have to supply their own (from a real 68040 Mac).  The
   installer instructions handle this with dignity.  Same model as
   MiSTer and most retro platforms.

---

## 9. Phase sequencing

```
                  NOW ────────────────── 6-12 mo ──────── 12-24 mo ──── 24+ mo
RTL/software:     [phase 2 closing] → phase 3 (System 7 boot) → phase 4 perf → phase 5
Hardware H0:      [FMC mezzanine design] ship to 20-50 early users
Hardware H1:                     [PCB bring-up] → retail at Crowd Supply
Hardware H2:                                    [case + final polish] → retail
```

Hardware is gated on software milestones, not the other way around:

- **H0** can ship as soon as the sim reliably boots System 7 on SD — i.e.,
  phase 3 of `gameplan.md`.  Call it 4-6 months out from today.
- **H1** needs phases 3 + 4 done (real caches, Mac OS performance
  respectable).  ~9-12 months out.
- **H2** needs H1 in the wild with customer feedback + case-design
  iteration time.  ~18-24 months out.

---

## 10. Immediate next steps (no implementation, just follow-ups)

1. **Hardware feasibility agent** — spawn a research agent to:
   - Validate the KU5P utilisation projection for the full SoC + MMU
     walker + real L1D + Ethernet MAC + HDMI IP.  If over 80% util at
     100 MHz, signal the Zynq ZU7EV pivot decision.
   - Catalog the real KU5P supply situation (lead time, reelable qty).
   - Draft a single-page H0 BOM cost model for the FMC mezzanine.
   Deliverable: `docs/hardware_feasibility.md`.

2. **Legacy PHY reference-designs collection agent** — compile known-
   working open-source reference schematics for each legacy peripheral:
   ADB, SCSI, LocalTalk, floppy, audio.  Deliverable:
   `docs/legacy_refs.md`.  Inform H0 mezzanine layout when an actual
   EE does the work.

3. **Industrial-design brief** — user-facing task: write up 1 page of
   "the box should look like X" for a third-party industrial designer.
   Reference images (clear iMac G3 internals, Teenage Engineering
   form factors, MiSTer cases).

4. **Host software audit** — the existing `m68kctl` needs a v2 path
   for the eventual board-management scheme (USB-CDC or TCP/IP, not
   just XDMA).  Non-urgent; bank for phase-3 software work.

5. **Pick the name.** — "m68k-ooo" is great for engineers, terrible as
   a product name.  Candidate shortlist:
   - "Quarta" (gestures at Quadra without ripping it off)
   - "Mackbit" (portable + keyboard vibes)
   - "System 10" (cheeky — 6/7/8/9 → 10)
   - "Lisa" (already taken, legally fraught)
   - "Helvetica" (typeface nod; already used in other products)
   - "Macintosh Ultra" (Apple still owns it; don't)

   Decision gate: phase-3 boot milestone is the natural "we have a
   product" reveal moment.  Land name before then.

---

## 11. What this doc is NOT

- A schematic.  Not drawing one before H0 software milestones land.
- A BOM.  Cost modelling comes when the feasibility agent runs.
- An industrial design spec.  A designer gets briefed; this doc is
  inputs to that brief.
- A promise.  Phase 3 could expose a core bug that resets the
  schedule.  The case is a reward for shipping software; ship
  software first.

---

## 12. Summary for the orchestrator

Hardware is a deferred-execution roadmap.  The FPGA design sequencing
in `gameplan.md` is the critical path; hardware phases H0/H1/H2 layer
on top.  The near-term "do nothing / notice a lot" directive stands:
keep the RTL evolving, defer all PCB/case work until phase-3 boots a
disk.  The doc exists so that when phase 3 lands, we have six months
of HW planning already banked.
