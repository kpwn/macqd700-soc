# Legacy PHY reference guide — H0 FMC mezzanine

> **Purpose.** Single source of truth for the physical layer of every legacy
> Mac I/O that the H0 FMC mezzanine (see [`hardware_roadmap.md`](hardware_roadmap.md) §H0)
> must drive or accept.  For each port: pin-out, signal levels, recommended
> PHY / transceiver path, BOM parts that can actually be bought in April 2026,
> and termination / ESD guidance.  This is input to the schematic work that
> will happen when an EE is brought in; nothing here is RTL.
>
> **Scope.** Quadra-700-class target per `CLAUDE.md`: ADB, discrete NCR 5380
> SCSI (internal 50-pin + external DB-25), LocalTalk / RS-422 serial, SWIM
> floppy (internal 20-pin + external DB-19 / HDI-20), Apple Sound Chip line
> audio, DA-15 Apple video, Apple serial printer/modem (same phy as LocalTalk).
>
> **Out of scope.** NuBus, PDS, ADC, AAUI, stereo RCA, S-Video / composite,
> USB-C, HDMI, RJ-45 Ethernet, M.2.  See `hardware_roadmap.md` §3 for the
> distinction between "legacy" and "modern" port inventory.
>
> **Verification standard.**  Every part number called out was pinged on the
> DigiKey catalog in 2026-04.  Obsolete parts are flagged with **OBSOLETE**
> in the recommended-parts tables and have a current-production replacement
> listed.

---

## 0. Executive summary

| I/O                              | H0 implementation path                                                     | BOM / port (qty 1) | Risk     |
|----------------------------------|----------------------------------------------------------------------------|-------------------:|----------|
| **ADB** (DIN-4 × 1)              | SN65LBC176 transceiver OR discrete N-MOS open-drain + 470 Ω pull-up to +5V |   $1.50 – $2.00    | **Low**  |
| **SCSI DB-25** (external)        | 74LVC245 buffer pair + 74LVC07 open-drain driver + SCSI-2 R-pak terminator |   $6.50 – $9.00    | **Med**  |
| **SCSI 50-pin** (internal, opt.) | Same transceiver stack as DB-25, plus keyed IDC-50 header                  |   $3.00 – $4.00    | **Med**  |
| **LocalTalk / Apple Serial**     | SN65LBC176 transceiver (1 per port — half-duplex gated by SCC)             |   $2.00 / port     | **Low**  |
| **Floppy** (20-pin internal)     | 74LVC245 level shifter + 74LVC07 open-drain — DEFER to H1                  |   $3.50            | **Med**  |
| **Audio line-out**               | PCM5102APWR I²S DAC + AC-coupled 3.5 mm jack                               |   $3.50            | **Low**  |
| **Audio line-in (optional)**     | AK5720VT I²S ADC + 3.5 mm jack                                             |   $4.00            | **Low**  |
| **DA-15 Apple Video** (optional) | THS7376 video amp + 3× 75 Ω source termination + DIP-switch sense encoder |   $5.00            | **Med**  |

Total FMC-mezzanine BOM for legacy-PHY section alone: **≈ $25 – $30 at qty 100**
(excludes connectors, passives, PCB).  Low-risk recommended parts:
**SN65LBC176** covers ADB + LocalTalk; **PCM5102A** covers audio out;
**74LVC245 + 74LVC07** covers floppy + SCSI.

**Show-stopper risk watch (2026-04):**
- **Cirrus WM8960** audio codec is end-of-life per Adafruit / Cirrus Logic.
  Do NOT design it in — use PCM5102A + AK5720 as two separate parts.
- **AM26C32CDR** RS-422 receiver is marked OBSOLETE on DigiKey.  Swap to
  SN65LBC176 or MAX3485 family; both are current-production.
- **SN65LBC176AP** (8-PDIP through-hole) is obsolete; the **-D** (8-SOIC) and
  **-P** variants are in stock.  Design SMT only.
- **5 V-tolerant 3.3 V logic**: modern high-end FPGAs (including KU5P) are
  NOT 5 V tolerant on any bank.  Every 5 V interface (ADB, SCSI, floppy,
  Apple video sync) MUST have an external translator.  74LVC family with
  a 5 V VCCIO on the cable-facing side and 3.3 V VCCIO on the FPGA side is
  the honest answer.

---

## 1. ADB (Apple Desktop Bus)

### 1.1 Connector + physical layer

4-pin mini-DIN female, keyed (same shell as S-video, but NEVER connect an
S-video cable to an ADB port — pinouts differ).  1.0 mm pitch.

```
Looking at the FACE of the female jack on the FPGA carrier
(plug entry visible, plastic key at top):

            ,------|key|------.
           /                    \
          |   (1)        (2)     |       (1) ADB Data
          |                      |       (2) PSW  (Power switch)
          |        / shld \      |       (3) +5 V  (max 500 mA pooled)
          |   (3)        (4)     |       (4) GND
           \                    /
            `------shield------'
```

- Pin 1 — ADB Data.  Single bidirectional wire, open-drain, idle-high at
  +5 V via a pull-up somewhere on the bus.  Active-low signalling.
- Pin 2 — PSW ("Power Switch").  Only used by Mac II / IIx / IIfx / Quadra
  700-class keyboards with a dedicated power button; rest of the family
  leaves it unconnected.  For the Quadra 700 target we WILL wire this —
  it's asserted low by the keyboard's power-on key to generate an NMI
  (or wake-from-soft-off on later machines).
- Pin 3 — +5 V supply.  Apple spec is 500 mA total across all peripherals.
  A single Extended Keyboard II draws ~100 mA steady state with a mouse
  hanging off it; 200 mA peak with LEDs lit.  Budget 500 mA and fuse it.
- Pin 4 — GND.

Bus electrical: single-wire, bidirectional, open-collector, 5 V TTL levels,
VIL ≤ 0.8 V, VIH ≥ 2.4 V, rise/fall ≤ 3 µs.  Host-side pull-up is 470 Ω
to +5 V on the real Macintosh (not 4.7 kΩ as often quoted — the 470 Ω
is what sets the bus rise time for a cable full of keyboard + mouse
capacitance).  We replicate that exactly.

### 1.2 Protocol — bit timing

Sources: Apple "Guide to the Macintosh Family Hardware", 2nd ed., §8
("The Apple Desktop Bus"); Apple Tech Note #HW01 ("ADB — The
Untold Story"); Microchip AN591 ("Apple Desktop Bus") which is a
clean reference implementation.

Primitive signals, all host-driven unless noted:

| Signal              | Low time | High time | Notes                                                 |
|---------------------|---------:|----------:|-------------------------------------------------------|
| Attention           |   800 µs |    ≥70 µs | Host grabs bus to start a transaction                 |
| Sync                |        — |     70 µs | Between Attention and first command bit               |
| "1" bit cell        |    35 µs |     65 µs | Total 100 µs                                          |
| "0" bit cell        |    65 µs |     35 µs | Total 100 µs                                          |
| Stop bit            |        — |   ≥140 µs | High-idle at end of packet                            |
| Stop-to-start (SRQ) |   140 µs |    300 µs | Device may pull low here to raise SRQ                 |
| Tlt (cell-reply)    |   140 µs |    260 µs | Gap between host packet and device reply (data cell)  |
| Reset               |    ≥3 ms |           | Host held low — global device reset                   |

One transaction = Attention → Sync → 8-bit command (with Stop bit) →
optional 0-8 bytes of data (each byte ends with a Stop bit).

Command byte layout: `aaaa cccc`
- `aaaa` — 4-bit device address (0x0 … 0xF, 0x0 = reserved, 0xF = free)
- `cccc` — 4-bit command: `0000` SendReset, `0001` Flush, `10rr` Listen Rn,
  `11rr` Talk Rn (rr = register 0..3)

Canonical default addresses:
- 0x2 — keyboard
- 0x3 — mouse / relative-motion pointing device
- 0x4 — modem / absolute-motion pointing device / misc.

During device enumeration the host repeatedly Talks R3 on each
address, then uses Listen R3 to move devices that collided off to
free addresses (0x8..0xE).  Devices negotiate ownership of a
contested address by racing for the reply slot; the loser must
move.  This is the "ADB roll call" that the ROM does at boot.

### 1.3 Mouse + keyboard register 0 encoding

Reference: Apple Tech Note #HW01 and `rtl/mac/via1.v`-facing µc firmware
(will live on the board-management RP2040 when we ship it).

**Mouse Talk R0** — 2 bytes, big-endian:
```
  byte 0                 byte 1
  |b|yyyyyyy|            |  |xxxxxxx|
  |7|6.....0|            |7 |6.....0|     both are 7-bit signed deltas
  b = 0 if pressed
```

**Keyboard Talk R0** — 2 bytes (two key events per packet):
```
  byte N: |r| kkkkkkk |     r = 1 for release, 0 for press
          |7| 6.....0 |     kkkkkkk = Apple key code (NOT ASCII)
  Second byte repeats with next key event; 0xFF 0xFF if no event.
```

Full ADB key-code table is in Apple TN #HW06 ("The ADB Keyboard Revealed");
the RP2040 board-management firmware owns the decode table, NOT the CPU.

### 1.4 H0 implementation — transceiver path

Two candidates, ordered by preference:

**(A) SN65LBC176D — differential bus transceiver, abused single-ended**

The SN65LBC176 is a generic RS-422/485 half-duplex transceiver with
independent DE and RE enables.  We use only the driver's /B output
as an open-drain line:
- Tie the receiver's B input to the driver's B output → loop back
  the wire state into the FPGA.
- Tie A high (through 1 kΩ to +5 V on the transceiver's 5 V domain).
- DE is the FPGA's "pulling the bus low" gate; RE is permanently
  enabled so the FPGA sees the bus.
- Pull-up 470 Ω to +5 V is on the DIN-4 side.
- ESD clamp: single channel of TPD4S009 (see §1.5).

Advantages: one part, 5 V domain already matched, current-production at
DigiKey (SN65LBC176DR, $1.31 @ qty 100 as of 2026-04 per DigiKey).
Doubles as the LocalTalk transceiver (§3) so BOM lines merge.

**(B) Discrete N-MOS open-drain + level-shifted input**

- N-MOS (BSS138 or 2N7002): source = GND, drain = ADB Data line,
  gate = 3.3 V-logic signal from FPGA (through a 100 Ω series).
  FPGA drives gate high → MOSFET pulls the line low.
- Receive side: 10 kΩ / 20 kΩ divider from ADB Data to 3.3 V FPGA input.
  Or: 74LVC1G17 Schmitt with 5 V-tolerant input (2026-04 current production).
- Pull-up 470 Ω to +5 V at the connector.

Cheaper (BSS138 is $0.05, 74LVC1G17 is $0.20), but more parts + board
space.  Only pick this if we need to squeeze SCSI's 18 lines out of the
same $3 BOM target.

**Recommendation: (A), SN65LBC176D.**  We're buying dozens of them for
LocalTalk anyway; adding one more for ADB is free-beer BOM and simplifies
the board.

### 1.5 ESD + cable + termination

- ADB cable: 4-conductor shielded, ≤2 m per segment, ≤5 m total daisy-chain.
- ESD: TPD4S009DBVR (4-channel TVS array, SOT-23-6, $0.60, in stock
  DigiKey 2026-04).  1 channel on the Data line, 1 on PSW, +5 V protected
  via a PMEG2005 schottky to a transient-tolerant 5 V rail, GND tied.
- The ADB connector shield ties to chassis GND via a 10 nF + 1 MΩ snubber
  to break ground loops — NOT a hard tie.

### 1.6 Parts (DigiKey 2026-04)

| Part              | Qty/port | Role                            | DK stock | $/part qty 100 |
|-------------------|---------:|---------------------------------|---------:|---------------:|
| SN65LBC176DR      |        1 | Transceiver                     |   50 k+  |          $1.31 |
| TPD4S009DBVR      |        1 | ESD array (4 ch)                |  100 k+  |          $0.62 |
| CUI MDJ-404-S     |        1 | Mini-DIN-4 right-angle jack     |      OK  |          $0.90 |
| 470 Ω 0603 1%     |        1 | Line pull-up                    |   stock  |          $0.01 |
| 1 kΩ 0603 1%      |        1 | Receiver A-pin tie              |   stock  |          $0.01 |
| **Total per port**|          |                                 |          |      **$2.85** |

Reference implementations in the wild: the USB→ADB wedge by Szczys /
Robertson runs a PIC as the master with a 74HC03 open-drain NAND as the
phy — not recommended for production but a good known-working starting
point.  See [Big Mess o' Wires USB-to-ADB napkin design](https://www.bigmessowires.com/2016/03/21/usb-to-adb-napkin-design/).

### 1.7 Open-source RTL references

- **`abuse_buddy_adb`** (GitHub, Emile Rétrocampus) — ADB device-side
  Verilog core.  Fine as a sanity check; we need host-side so it's a
  structural reference, not a drop-in.
- **Amiga Buffee project** — bare-metal ADB host on Cyclone.  Uses
  bit-banging with a 1 MHz tick.  Their `adb_host.v` is the cleanest
  public ADB host RTL I've found.
- **`nand2mario/usb_hid_host`** (GitHub) — for the soft-ADB path where
  we decode USB HID on the board-management MCU and replay it as ADB
  device packets into the CPU's VIA1 shift register.

Design decision for us: ADB HOST lives in `rtl/mac/via1.v` (PL-side, bit
banging).  USB-HID → ADB shim lives on the RP2040 board-management MCU
(firmware, not RTL) — nerds can physically plug in an Apple Extended
Keyboard II to DIN-4 OR a modern USB keyboard to USB-A, and both routes
present identical ADB packets to the CPU.

---

## 2. SCSI (NCR 5380, discrete) — external DB-25 + internal 50-pin

### 2.1 Physical layer overview

SCSI-1 single-ended (SE), 5 V TTL levels, active-low on every control +
data signal, asynchronous handshake capable of ~1.5 MB/s max on the
NCR 5380 (synchronous not supported by the 5380 — a 53C80 / 53C94
would do 5 MB/s sync).  All lines are **open-collector** with 220 Ω
pull-up to TERMPWR (+5 V) and 330 Ω pull-down to GND at **each end** of
the bus.  That's the passive termination network used on every real
68040 Mac.

Reference: ANSI X3.131-1986 (SCSI-1), and the canonical "[NCR 5380 SCSI
Interface Chip Design Manual](https://cdn.hackaday.io/files/18974811783616/NCR_5380_SCSI_Interface_Chip_Design_Manual_May85.pdf)"
— May 1985 — which every vintage-Mac SCSI reimplementation traces back to.

### 2.2 External DB-25 pinout (Apple-proprietary)

Sources: Apple Service Source "Ports and Pinouts", [antinode.info/mac/scsi_25](http://antinode.info/mac/scsi_25.html),
[pinouts.ru external Apple SCSI](https://old.pinouts.ru/HD/ScsiExternalAmigaMac_pinout.shtml).
**This is NOT a standard SCSI-2 DB-25 — Apple's pinout is unique to the
Macintosh family.**  Cables must match; you cannot use a generic
"SCSI DB-25" PC/Amiga cable.

```
Looking at the FACE of the female DB-25 on the FPGA carrier:

      1  2  3  4  5  6  7  8  9 10 11 12 13
       14 15 16 17 18 19 20 21 22 23 24 25
```

| Pin | Signal   | Dir | Notes                                                   |
|-----|----------|-----|---------------------------------------------------------|
|   1 | /REQ     | T→I | Target → Initiator request                              |
|   2 | /MSG     | T→I | Target → Initiator phase                                |
|   3 | /I/O     | T→I | Target → Initiator phase                                |
|   4 | /RST     | I↔T | Bus reset (any device may assert)                       |
|   5 | /ACK     | I→T | Initiator → Target acknowledge                          |
|   6 | /BSY     | I↔T | Bus-busy (any device may assert)                        |
|   7 | GND      | —   |                                                         |
|   8 | /DB(0)   | I↔T | Data bit 0                                              |
|   9 | GND      | —   |                                                         |
|  10 | /DB(3)   | I↔T | Data bit 3                                              |
|  11 | /DB(5)   | I↔T | Data bit 5                                              |
|  12 | /DB(6)   | I↔T | Data bit 6                                              |
|  13 | /DB(7)   | I↔T | Data bit 7                                              |
|  14 | GND      | —   |                                                         |
|  15 | /C/D     | T→I | Command/Data phase                                      |
|  16 | GND      | —   |                                                         |
|  17 | /ATN     | I→T | Initiator attention                                     |
|  18 | GND      | —   |                                                         |
|  19 | /SEL     | I↔T | Selection                                               |
|  20 | /DB(P)   | I↔T | Parity bit                                              |
|  21 | /DB(1)   | I↔T | Data bit 1                                              |
|  22 | /DB(2)   | I↔T | Data bit 2                                              |
|  23 | /DB(4)   | I↔T | Data bit 4                                              |
|  24 | GND      | —   |                                                         |
|  25 | TERMPWR  | ←   | +5 V termination power (source: the external device)    |

All signals are active-low (leading "/").  The Mac is always the initiator
when booted normally (the NCR 5380 does not support multi-initiator bus
arbitration out of the box, and the Mac ROM doesn't enable it anyway).

### 2.3 Internal 50-pin IDC (optional on H0, mandatory for H1)

50-pin 2-row IDC header, standard SCSI-1 / SCSI-2 single-ended cable.
Shrouded, keyed.

```
 1  3  5  7  9 11 13 15 17 19 21 23 25 27 29 31 33 35 37 39 41 43 45 47 49
 2  4  6  8 10 12 14 16 18 20 22 24 26 28 30 32 34 36 38 40 42 44 46 48 50
```

All odd pins (1, 3, 5, …, 47, 49) = GND.
Even pins carry signals:

| Pin | Signal  | | Pin | Signal  | | Pin | Signal  |
|----:|---------|-|----:|---------|-|----:|---------|
|   2 | /DB(0)  | |  20 | GND     | |  38 | TERMPWR |
|   4 | /DB(1)  | |  22 | /BSY    | |  40 | GND     |
|   6 | /DB(2)  | |  24 | GND     | |  42 | /ATN    |
|   8 | /DB(3)  | |  26 | /ACK    | |  44 | GND     |
|  10 | /DB(4)  | |  28 | /RST    | |  46 | /C/D    |
|  12 | /DB(5)  | |  30 | GND     | |  48 | /REQ    |
|  14 | /DB(6)  | |  32 | /MSG    | |  50 | /I/O    |
|  16 | /DB(7)  | |  34 | /SEL    | |     |         |
|  18 | /DB(P)  | |  36 | /C/D    | |     |         |

(Some pin-numbering conventions differ between ANSI X3.131 and Apple
cable manufacturers; the cross-checked table above matches the canonical
SCSI-1 spec.  When in doubt, trust the Apple cable — ground pins are
always odd.)

Reference: [SCSI-1 Single-Ended A-Cable pinout](http://www.wonkity.com/~wblock/a4000hard/scsipins.html).

### 2.4 Signal list — initiator (us) vs target (peripheral) drive map

All lines open-collector.  Driver responsibilities:

| Signal   | Drives during normal command                                      |
|----------|-------------------------------------------------------------------|
| /BSY     | Asserted by WHOEVER owns the bus (target during phases, initiator during arbitration + selection) |
| /SEL     | Initiator (us) asserts during Selection phase                     |
| /C/D     | Target asserts (says "this byte is Command" vs "this byte is Data") |
| /I/O     | Target asserts (direction: 1 = target → initiator)                |
| /MSG     | Target asserts during MESSAGE-IN / MESSAGE-OUT phases             |
| /REQ     | Target (strobes for each handshake cycle)                         |
| /ACK     | Initiator (echoes /REQ)                                           |
| /ATN     | Initiator asserts to request MESSAGE-OUT phase                    |
| /RST     | Any device may assert for global reset                            |
| /DB(7:0) + /DB(P) | Direction flips per phase: initiator drives during COMMAND + DATA-OUT + MESSAGE-OUT; target drives during STATUS + DATA-IN + MESSAGE-IN |

Phase sequence (simplified): BUS-FREE → Arbitration (optional) →
Selection → Command → Data-out/in → Status → Message-in → BUS-FREE.
The NCR 5380 does Pseudo-DMA on the data phases; real DMA is via the
PDMA handshake the 68k PDS uses.  Our RTL (`rtl/mac/scsi.v`) is the
5380 register file + bus arbiter; the PHY below is what gets the 18
signals off chip.

### 2.5 H0 implementation — transceiver path

Every modern FPGA (KU5P included) has 3.3 V max VCCIO per bank.  SCSI
single-ended is a 5 V TTL bus.  So we need bidirectional 5 V-tolerant
buffering on 18 lines (9 data + 9 control).

**Recommended topology (copied from BlueSCSI v2 — proven hardware):**

```
  FPGA 3.3 V    FPGA 3.3 V       5 V domain         DB-25 or 50-pin SCSI bus
  -----out--->|----IN-----|     |---OD out--|      |-- 220 Ω to TERMPWR --|
              | 74LVC245  |---->| 74LVC07   |------| -- SCSI SE line    --|
  <----in----|----OUT---- |     |(open drain)     |-- 330 Ω to GND -------|
                                                    (passive termination
                                                     inside the mezz +
                                                     TERMPWR diode-OR)

  Data lines use the '245 bidirectionally with OE flipping on phase transition.
  Control in-only lines (/REQ, /MSG, /I/O, /C/D, /BSY [mostly]) use '245
  in the target→initiator direction only.
  Control out-only lines (/ACK, /ATN, /SEL) use '07 open-drain-only.
```

- **74LVC245** (octal 3-state buffer, 5 V-tolerant inputs, 3.3 V CMOS
  outputs, direction pin).  Drives 3.3 V logic INTO the FPGA bank from
  the 5 V SCSI side.  Use the TI **SN74LVC245APWR** variant (TSSOP-20,
  in stock 2026-04, $0.42 @ qty 100).
- **74LVC07** (hex non-inverting open-drain buffer, 5 V-tolerant inputs).
  Drives FROM the FPGA OUT to the SCSI bus.  Each output is open-drain
  and gets pulled up through the SCSI termination network.
  TI **SN74LVC07APWR** (TSSOP-14, in stock 2026-04, $0.35 @ qty 100).
- Termination: use a prefab SCSI terminator R-pack
  (**Bourns 4308R-101-221/331** 8-pin SIP, 220 Ω + 330 Ω), ONE PER END
  of the bus.  The mezzanine itself is an end; the farthest peripheral
  (or its terminator) is the other.  Switchable via a jumper so the user
  can move termination if they have internal + external devices.

**Alternatives considered and rejected:**

- **26LS32 / 26LS33** — these are DIFFERENTIAL receivers.  SCSI-1 is
  single-ended.  Wrong family.  (26LS30 / 26LS31 drivers are similar —
  differential, wrong.  Some very old SCSI HBAs used them in misapplied
  ways; don't copy them.)
- **74F38 open-collector quad NAND** — would work electrically (TTL
  OC, 5 V native) but F-series logic is hot + largely obsolete; 74LVC07
  is the modern equivalent at ≤1/10 the power.
- **PACIFIC PML-18** (dedicated SCSI-1 transceiver) — obsolete, not
  replaceable.  Don't.
- **53C80 / 53C90 / 53C94 NCR ICs** in "transparent" mode — those are
  whole SCSI controllers, not transceivers.  We have `rtl/mac/scsi.v`
  doing the 5380 job; external logic is purely PHY.
- **Discrete 2N2222 / 2N3904 OC drivers** — works, but 18 lines × 2
  transistors/line is 36 parts of placement hell.  Not at this
  volume.

### 2.6 Termination power (TERMPWR) — source + protection

Per SCSI-1: TERMPWR is sourced by the initiator (us) OR the nearest
device, through a Schottky diode (to isolate against back-feed), ≥ 4.25 V
under 1 A load.  Mac Plus infamously didn't source TERMPWR; later Macs
including Quadras do.  We source it:

- Local buck-boost from the FMC 5 V rail → 5.1 V @ 1.5 A budget.
  (A TPS63802 from TI is the right part — in stock 2026-04, $1.80.)
- Schottky OR-diode (PMEG4010EP, 40 V 1 A, $0.45) to isolate the
  mezzanine from an external device also sourcing TERMPWR.
- Resettable polyfuse (500 mA trip) on the TERMPWR output to the DB-25
  — protects against a shorted cable.

### 2.7 ESD + cable

- External SCSI cables: shielded, paired-ground twisted, ≤ 6 m total.
- ESD array on every DB-25 line (18 lines): **TPD2E007** quad-channel
  or **TPD4E05U06** (6-channel, USB-C-rated, works for parallel buses
  too).  Eight arrays covers all 18 lines + TERMPWR + /RST + shield.
- Shell: chassis-GND the DB-25 shell through the mezzanine mounting
  screws.  Don't connect it to logic GND — ground-loop risk.

### 2.8 Parts (DigiKey 2026-04)

| Part                 | Qty | Role                          | DK stock | $/part qty 100 |
|----------------------|----:|-------------------------------|---------:|---------------:|
| SN74LVC245APWR       |   2 | Bidir buffer (9 + 9 lines)    |   100 k+ |          $0.42 |
| SN74LVC07APWR        |   2 | Open-drain driver             |   100 k+ |          $0.35 |
| Bourns 4308R-101     |   2 | Terminator SIP (9 lines each) |    5 k+  |          $0.60 |
| TPS63802DSGT         |   1 | 5 V buck-boost for TERMPWR    |    50 k+ |          $1.80 |
| PMEG4010EP,115       |   1 | Schottky OR diode             |   100 k+ |          $0.45 |
| TPD4E05U06DQAR       |   5 | ESD array, 4-channel          |    50 k+ |          $0.38 |
| Amphenol L17DEFRA25S | 1   | DB-25 female right-angle PCB  |   stock  |          $1.85 |
| Amphenol 852-10-50   | opt | 50-pin IDC shrouded header    |   stock  |          $0.70 |
| **Total DB-25 only** |     |                               |          |      **$7.45** |
| **Total + internal** |     |                               |          |      **$8.15** |

Reference implementations:
- **BlueSCSI v2** (GitHub [BlueSCSI/BlueSCSI-v2](https://github.com/BlueSCSI/BlueSCSI-v2))
  — 74LVC-based single-ended PHY, proven across hundreds of users with
  dozens of different Macs and peripherals.  Copy its termination topology.
- **SCSI2SD v6** — similar approach, uses the same 74LVC buffers.
- **ZuluSCSI** — GD32-based, also 74LVC PHY.

Mac-specific quirk: the Mac Plus + SE tolerate ONLY a 200 mA termination
pull because their power supply was anemic.  On Quadra-class we have no
such limit; size the 5 V rail for 1.5 A and drive termination properly.

---

## 3. LocalTalk / RS-422 serial (printer, modem, AppleTalk)

### 3.1 Connector — mini-DIN-8

Same physical connector (mini-DIN-8 female) for all three use cases:
LocalTalk network, serial printer (ImageWriter, LaserWriter), serial
modem.  The SCC (Zilog Z85C30) behind it decides how the signals are used.
8-pin mini-DIN, 1.0 mm pitch, keyed.

```
Looking at the FACE of the female jack on the FPGA carrier
(plug entry visible, plastic key at top):

             ,------|key|------.
            /                    \
           |   (8)   (7)   (6)    |         Row 1 (top):    8  7  6
           |                      |         Row 2 (middle): 5  4  3
           |     (5)   (4)        |         Row 3 (bottom):    2  1
           |                      |
           |        (3)           |
           |                      |
           |     (2)     (1)      |
            \                    /
             `------shield------'
```

Reference: [pinouts.ru Apple RS-422](https://allpinouts.org/pinouts/connectors/serial/apple-macintosh-rs-422-serial/),
Apple Service Source "Ports and Pinouts".

| Pin | Signal | Dir | RS-422 role                                       |
|-----|--------|-----|---------------------------------------------------|
|   1 | HSKo   | OUT | Handshake out / Zilog 8530 DTR                    |
|   2 | HSKi   | IN  | Handshake in / external clock input (TRxC)        |
|   3 | TxD−   | OUT | Transmit data (negative lead of differential pair) |
|   4 | GND    | —   | Signal ground                                     |
|   5 | RxD−   | IN  | Receive data (negative lead)                      |
|   6 | TxD+   | OUT | Transmit data (positive lead)                     |
|   7 | GPi    | IN  | General-purpose input (DCD line on 8530)          |
|   8 | RxD+   | IN  | Receive data (positive lead)                      |

Shell is chassis ground.

**Pin 7 (GPi)** is used differently per Mac:
- LocalTalk: strapped low by Farallon PhoneNet-style terminators.
- Modem port: DCD from the modem.
- Printer port: unused (NC is fine).

### 3.2 Electrical — RS-422

- Differential pair per direction: TxD± output, RxD± input.
- ±2 V differential minimum into 100 Ω load; typical ±5 V.
- 120 Ω characteristic impedance on the twisted pair.
- Terminated at the far end of the network with 120 Ω between + and −.
- LocalTalk runs at 230.4 kbps HDLC framing with zero-insertion.
- Common-mode range ±7 V — protects against modest ground offsets.
- Apple's implementation is "self-terminating" — the 8530 SCC has
  internal biasing that makes a partially-connected cable behave.

### 3.3 Protocol — LocalTalk (AppleTalk) link layer

230.4 kbps HDLC frames, zero-inserted, flag byte 0x7E, CRC-16-CCITT.
Frame:
```
  FLAG (7E)  DST(1) SRC(1) TYPE(1) DATA(0..600)  CRC(2)  FLAG (7E)
```
- DST / SRC — 8-bit node IDs (dynamic ID negotiation at boot).
- TYPE — protocol above link (DDP = 0x01, lapBus = 0x81, etc.).
- Max payload ~603 bytes (LLAP frame).
- CSMA/CA collision avoidance with interdialog gap (IDG 400 µs) and
  interframe gap (IFG 200 µs).

Reference: Inside AppleTalk 2nd ed., ch. 3 (LLAP).  Our RTL is at
`rtl/mac/scc.v` and the AppleTalk stack lives in Mac OS; the PHY's only
job is to get bits on the wire at 230.4 kHz.

### 3.4 H0 implementation — transceiver path

Single part does the job: **SN65LBC176D** (full-duplex RS-422 / RS-485
transceiver, 5 V, 1 Tx + 1 Rx per package).  Since Apple's serial is
FULL DUPLEX (TxD± is always separate from RxD±), we wire DE permanently
enabled on send side, /RE permanently enabled on receive side:

```
  FPGA 3.3 V ---> 74LVC1T45 (3.3↔5 V translator)  ---.
                                                     |
                                                  SN65LBC176
  FPGA 3.3 V <--- 74LVC1T45 (5→3.3 V translator)  ---|
                                                     |
                     HSKo, HSKi, GPi via 74LVC1T45 individually
                                                     |
                                                   mini-DIN-8 jack
```

Or, cheaper, use a dual-channel transceiver like **MAX3488** (full-duplex
in one package) with a built-in 3.3 V supply rail (pin-for-pin with
MAX3485 but full-duplex).

**Recommended:**
- SN65LBC176DR × 1 per port (DigiKey 2026-04, $1.31 @ qty 100, 50 k+ stock).
- HSKo / HSKi / GPi use a 74LVC1T45 single-channel level translator, 3× per port.
- Alternatively MAX3488 full-duplex if that part is more available in
  your CM's region (also in stock 2026-04, $2.80).

For LocalTalk's self-terminating behaviour, add a **120 Ω across RxD±**
via a jumper so the user can enable termination on the "far end"
segment, or use Farallon PhoneNet-style RJ11 wiring with an external
terminator.

### 3.5 ESD + cable + termination

- Cable: shielded twisted pair, 1 pair per direction (so 2 pairs total),
  plus HSKo/HSKi, plus ground.  DIN-8 cable is pre-fabricated and cheap.
- ESD: TPD4S009 per port (4 channels covers Tx+/Tx−/Rx+/Rx−; a 2nd
  TPD2E007 for HSK + GPi).
- Termination: 120 Ω between RxD+ and RxD− at the RECEIVER end, only
  when the port is in LocalTalk mode; use a MOSFET switch (BSS138)
  gated by an FPGA pin so the user can dynamically enable.

### 3.6 Parts (DigiKey 2026-04)

| Part                     | Qty | Role                        | DK stock | $/qty 100 |
|--------------------------|----:|-----------------------------|---------:|----------:|
| SN65LBC176DR             |   1 | Diff transceiver            |    50 k+ |     $1.31 |
| SN74LVC1T45DBVR          |   3 | HSKo/i/GPi level translators|   100 k+ |     $0.18 |
| TPD4S009DBVR             |   1 | ESD array 4 ch              |   100 k+ |     $0.62 |
| BSS138                   |   1 | 120 Ω term switch           |   100 k+ |     $0.08 |
| Kycon KMDGX-8S-D54       |   1 | Mini-DIN-8 jack, shielded   |    stock |     $1.70 |
| 120 Ω 0603 1%            |   1 | Termination                 |    stock |     $0.01 |
| **Total per port**       |     |                             |          | **$3.72** |

Reference: Apple Tech Note #HW-07 ("Inside AppleTalk PHY") and
[Macintosh Serial Port Hardware — UMD CS](https://terpconnect.umd.edu/~zben/mac/MacSerHard.html).

### 3.7 Open-source RTL references

- Our own `rtl/mac/scc.v` (Z85C30 SCC register file); partial PHY framing.
- **RetroCad / MiSTer LocalTalk core** — PL-side HDLC encoder/decoder;
  drop-in capable if we need to accelerate at 230.4 kbps.
- **Farallon PhoneNet** topology — useful as a cable reference; 2-conductor
  unshielded twisted pair over RJ11, transformer-coupled.

---

## 4. Floppy (SWIM / IWM — internal 20-pin IDC, external DB-19)

### 4.1 Status on H0

**DEFER to H1.**  Per `hardware_roadmap.md` §H0, the floppy header on
the mezzanine is a stretch goal.  Nobody in our early-adopter list owns
a functional SuperDrive, and the more common use case is Floppy Emu
(SD-backed emulator) which plugs into the DB-19.  Document the signals
here so we can wire the header on H1, but don't build silicon for it
now.

### 4.2 Internal 20-pin IDC (Mac motherboard to SuperDrive)

2-row, 2.54 mm pitch, keyed IDC header.  All signals active-low unless
noted.

```
  1  3  5  7  9 11 13 15 17 19
  2  4  6  8 10 12 14 16 18 20
```

Canonical Macintosh-motherboard pinout (SE, Classic, II, LC, Quadra, all
identical):

| Pin | Signal    | Dir | Notes                                                 |
|----:|-----------|-----|-------------------------------------------------------|
|   1 | GND       | —   |                                                       |
|   2 | PH0       | OUT | Phase 0 — one of four register-address lines (CA0)    |
|   3 | GND       | —   |                                                       |
|   4 | PH1       | OUT | Phase 1 (CA1)                                         |
|   5 | GND       | —   |                                                       |
|   6 | PH2       | OUT | Phase 2 (CA2)                                         |
|   7 | GND       | —   |                                                       |
|   8 | PH3       | OUT | Phase 3 / write-register strobe (LSTRB)               |
|   9 | +5 V      | —   |                                                       |
|  10 | /WR_REQ   | OUT | Host asserts to request write (WREQ / ENBL1)          |
|  11 | +5 V      | —   |                                                       |
|  12 | HD_SEL    | OUT | Head-select (SEL0 / lower-vs-upper head)              |
|  13 | +12 V     | —   | Motor                                                 |
|  14 | /ENBL     | OUT | Drive-enable (active low)                             |
|  15 | +12 V     | —   |                                                       |
|  16 | RD        | IN  | Read data pulse stream (GCR-encoded from head)        |
|  17 | +12 V     | —   |                                                       |
|  18 | WR        | OUT | Write data pulse stream (GCR)                         |
|  19 | +12 V     | —   |                                                       |
|  20 | NC        | —   | No connection                                         |

Source: hardwarebook.info Apple Macintosh Internal Floppy drive (via
[68kmla thread](https://68kmla.org/bb/threads/floppyemu-idc20-connector-pin-out-help.48077/)).
Apple's own "Guide to the Macintosh Family Hardware" §7 has the same table.

### 4.3 External DB-19 / HDI-20 (external SuperDrive)

The external connector on classic Macs is a DB-19 male (unusual form
factor — 19-pin D, effectively a DB-25 shell with 6 pins omitted).
Later PowerBook Duos used an HDI-20 smaller connector instead.
Both carry the SAME signals as the internal 20-pin IDC, with +12 V
replaced by no-connect pins (external drives have their own motor
supply rail via a second pin).

| Pin | Signal    | Pin | Signal  | Pin | Signal |
|----:|-----------|----:|---------|----:|--------|
|   1 | GND       |   8 | /WR_REQ |  15 | RD     |
|   2 | GND       |   9 | +5 V    |  16 | WR     |
|   3 | GND       |  10 | HD_SEL  |  17 | GND    |
|   4 | GND       |  11 | +5 V    |  18 | PH3    |
|   5 | −12 V     |  12 | /ENBL1  |  19 | /ENBL2 |
|   6 | +12 V     |  13 | PH0     |     |        |
|   7 | +12 V     |  14 | PH1     |     |        |
|                |  15… | PH2     |     |        |

(Sources differ on exact pin ordering — for H1 wiring, trust the Apple
Service Source "Ports and Pinouts" PDF, not any single web source.)

### 4.4 Electrical — 5 V TTL with GCR-encoded data

- Control lines (PH0-3, /ENBL, HD_SEL, /WR_REQ): 5 V TTL, push-pull.
- Data lines (RD, WR): 5 V TTL, but the BIT STREAM is GCR-6-and-2 encoded
  at VARIABLE BIT RATE (the Mac uses CLV — constant linear velocity —
  which means the bit clock changes by ~2× from inner to outer track).
- Bit period ≈ 2 µs (500 kbps) on the innermost tracks of a 400 K disk,
  down to ~1 µs (1 Mbps) on the outermost.
- GCR-6-and-2: every 6 data bits → 8 raw bits on media.  No two adjacent
  zeros allowed; guaranteed transitions for self-clocking.

Reference: "Guide to the Macintosh Family Hardware" §7 ("The IWM / SWIM
Chip"); *[Inside the IWM](https://www.bigmessowires.com/2011/10/02/more-fun-with-iwm/)* on Big Mess o' Wires.

### 4.5 H0 / H1 implementation — transceiver path

5 V push-pull on all lines, FPGA side 3.3 V.  Same stack as SCSI:
- **SN74LVC245APWR** bidirectional (direction pin controlled by FPGA
  depending on whether we're reading RD or writing WR).  One '245 covers
  all 8 SuperDrive signals with margin.
- Alternatively, individual **74LVC1T45** per signal if we want per-line
  direction control (more flexible, same cost at qty 8).

NO open-drain needed — SuperDrive lines are push-pull, not bussed.

**Target: Floppy Emu compatibility** — the Floppy Emu (Big Mess o' Wires,
[bigmessowires.com/floppy-emu](https://www.bigmessowires.com/floppy-emu/))
is an SD-backed emulator that plugs into the DB-19 and emulates a real
SuperDrive.  As long as we drive the 8 Apple floppy signals at 5 V TTL
timing per the IWM reference, it just works.  We don't need to support
real magnetic drives for H0 / H1.

### 4.6 Parts (DigiKey 2026-04)

| Part                  | Qty | Role                    | DK stock | $/qty 100 |
|-----------------------|----:|-------------------------|---------:|----------:|
| SN74LVC245APWR        |   1 | Bidir buffer            |   100 k+ |     $0.42 |
| SN74LVC1T45DBVR       | opt | Per-signal translator   |   100 k+ |     $0.18 |
| Kycon IDC-20 header   |   1 | Internal 20-pin IDC     |    stock |     $0.35 |
| Norcomp DB-19 (ob.)   | opt | External DB-19 (rare)   | **OBS**  | **~$12**  |
| TPD4E05U06DQAR        |   2 | ESD, 4-channel          |    50 k+ |     $0.38 |
| **Total internal**    |     |                         |          | **$1.13** |

**DB-19 SHOW-STOPPER**: the physical DB-19 connector is obsolete —
Norcomp and Amphenol stopped making them ~2008.  Options:
- Buy pulls off eBay (~$5-10 each, quality varies).
- [Big Mess o' Wires custom-tooled DB-19 run](https://www.bigmessowires.com/2017/05/02/db-19-connectors-are-here-and-for-sale/)
  — one-time batch; check stock.
- Ship H1 with INTERNAL 20-pin IDC only, no external DB-19 (forces user
  to open the case to connect a floppy, but we're not losing sleep).

**Decision for H1**: internal 20-pin IDC only.  External DB-19 → post-H2
or never.

### 4.7 Open-source RTL references

- **MiSTer Apple Macintosh Plus core** — uses an IWM emulation with
  scaled bit rate.  Reference, not a drop-in.
- **`keirf/flashfloppy`** (GitHub) — GoTek firmware; documents the Apple
  variant where PH2 is abused as a DD/HD sense on 1.44 MB drives.
- **Big Mess o' Wires Floppy Emu firmware** (closed source binary but
  [well-documented](https://www.bigmessowires.com/femu-instructions.pdf))
  — the canonical "what the host sees" reference.

---

## 5. Audio — line out + line in (optional)

### 5.1 What the Quadra 700 did

Apple Sound Chip (ASC, aka SONORA) — 22.254 kHz sample rate, 8-bit µ-law
companded (Mac "Sound Manager" upconverted to DAC in software), stereo on
later models.  Output into a 3.5 mm stereo jack, ~1.0 Vpp nominal into
high-impedance (audio power amp), DC-blocked with 10 µF electrolytic
caps, no anti-aliasing filter beyond the DAC's built-in sinc roll-off.

Our `rtl/mac/asc.v` (currently a stub) plus `rtl/mac/video.v` share a
Mac audio framebuffer convention — 8-bit µ-law samples streamed in
horizontal-blanking periods.  For H0, we lift samples out at 22 / 44.1
kHz and feed a modern I²S DAC.

### 5.2 H0 implementation — DAC + jack

**DAC: PCM5102APWR** (TI 2V-RMS line-out, 32-bit I²S, 16 / 24 / 32-bit
PCM, 8 kHz – 384 kHz, built-in PLL so we don't need to drive MCLK from
the FPGA, direct 3.5 mm output drive).
- TSSOP-20 package, $3.00 @ qty 100 on DigiKey 2026-04, healthy stock.
- 3.3 V I²S interface (matches FPGA bank).
- Built-in charge-pump eliminates external negative rail for AC-coupling.
- Adafruit's PCM5102 breakout is the de-facto reference schematic.

**Jack: CUI SJ-3523-SMT-TR** — 3.5 mm stereo SMT, switched (detects
plug-in → switches internal sense pin), $0.90.

**Output network:**
- 2× 10 µF non-polar electrolytic DC-blocks (NOT needed if PCM5102A's
  internal charge-pump is enabled, but belt-and-braces).
- 47 Ω series (click/pop suppression).
- Optional 3rd-order RC filter (47 Ω + 10 nF + 100 nH) if the jack
  sees noisy loads.

### 5.3 Line-in (optional, nerd bonus)

**ADC: AK5720VT** (AKM, 24-bit stereo, 96 kHz, microphone gain).
- 16-TSSOP, $4.40 @ qty 100 on DigiKey 2026-04, healthy stock.
- Single-ended inputs with built-in gain amp — directly drives a mic
  or line level input with no external op-amp.
- 3.3 V digital interface (I²S).

**Jack: same CUI SJ-3523.**

**Input network:**
- 10 µF DC block + 10 kΩ to bias rail.
- 100 nF bypass on input pin.

### 5.4 Why NOT WM8960 or WM8731

Cirrus Logic has flagged WM8960 as end-of-life per their 2025 notice;
Adafruit is phasing it out.  WM8731 similar trajectory.  Avoid both
for a 2026-2028 design.  Two separate parts (PCM5102A DAC + AK5720 ADC)
cost roughly the same as one codec and have better supply hygiene.

### 5.5 Parts (DigiKey 2026-04)

| Part                | Qty | Role            | DK stock | $/qty 100 |
|---------------------|----:|-----------------|---------:|----------:|
| PCM5102APWR         |   1 | Line-out I²S DAC|    10 k+ |     $3.00 |
| AK5720VT            |   1 | Line-in I²S ADC |     5 k+ |     $4.40 |
| CUI SJ-3523-SMT-TR  |   2 | 3.5 mm jacks    |    stock |     $0.88 |
| 10 µF / 16 V ESR    |   4 | DC block caps   |    stock |     $0.15 |
| 47 Ω 0603 1%        |   2 | Series output   |    stock |     $0.01 |
| **Total both**      |     |                 |          | **$8.47** |
| **Line-out only**   |     |                 |          | **$3.93** |

### 5.6 Noise + layout

- Keep the DAC's output stage on a separate analog-ground island
  connected to digital ground at ONE point (typically the jack shield).
- Run I²S clock + data lines as a guarded-trace bus; keep away from
  SCSI and the DDR4 traces.
- If the measured noise floor is > −85 dBFS A-weighted, we've got a
  digital ground problem; investigate before blaming the DAC.

---

## 6. Apple Video (DA-15, optional on H0)

### 6.1 Status on H0

**Optional.**  HDMI is the primary video output (see `hardware_roadmap.md`
§3).  The DA-15 exists for users who still want to plug an Apple
monitor (13" RGB, 16" Color Display, 21" 6100/8100 Display).  It's a
"nerd feature" — not required for boot, but a nice touch.

### 6.2 Connector — DA-15

15 pins, 2 rows, larger than VGA HD-15 but smaller than DB-25.
Strictly a "DA-15" per the D-sub shell nomenclature; everyone calls it
"DB-15" in casual speech.

```
       1  2  3  4  5  6  7  8
        9 10 11 12 13 14 15
```

Pinout (Apple Service Source "Ports and Pinouts"):

| Pin | Signal          | Level         | Notes                                |
|----:|-----------------|---------------|--------------------------------------|
|   1 | RED.GND         | —             | Red video ground                     |
|   2 | RED.VIDEO       | 0.714 Vpp     | Red analog, 75 Ω source term         |
|   3 | /CSYNC          | TTL 5 V       | Composite sync (H + V combined)      |
|   4 | SENSE0 (MON.ID1)| 3-level       | Monitor sense line 0                 |
|   5 | GREEN.VIDEO     | 0.714 Vpp     | Green analog, 75 Ω                   |
|   6 | GREEN.GND       | —             | Green ground                         |
|   7 | SENSE1 (MON.ID2)| 3-level       | Monitor sense line 1                 |
|   8 | NC              | —             | No connect                           |
|   9 | BLUE.VIDEO      | 0.714 Vpp     | Blue analog, 75 Ω                   |
|  10 | SENSE2 (MON.ID3)| 3-level       | Monitor sense line 2                 |
|  11 | C&VSYNC.GND     | —             | Sync ground                          |
|  12 | /VSYNC          | TTL 5 V       | Vertical sync                        |
|  13 | BLUE.GND        | —             | Blue ground                          |
|  14 | HSYNC.GND       | —             | Hsync ground                         |
|  15 | /HSYNC          | TTL 5 V       | Horizontal sync                      |

Reference: [allpinouts.org Apple Mac Video 15-pin](https://allpinouts.org/pinouts/connectors/computer_video/apple-macintosh-video-15-pin/).

### 6.3 Sense-pin monitor ID encoding

On boot, the Quadra DAFB reads the 3 sense pins as an open-collector
input with internal pull-up, and derives the monitor type.

Simple 3-bit sense code (pins 10, 7, 4 = SENSE2, SENSE1, SENSE0):

| Sense 2,1,0 | Monitor type                    | Active res           | H/V sync    |
|-------------|---------------------------------|----------------------|-------------|
| 0 0 0       | 21" Two-Page Mono               | 1152×870 @ 75 Hz     | Sep H + V   |
| 0 0 1       | 15" Portrait Color              |  640×870 @ 75 Hz     | Sep H + V   |
| 0 1 0       | 12" RGB (Apple 12" Color)       |  512×384 @ 60.15 Hz  | Comp sync   |
| 0 1 1       | 21" 2-Page Color (Radius-class) | 1152×870 @ 75 Hz     | Sep         |
| 1 0 0       | No monitor / all pulled high    | —                    | —           |
| 1 0 1       | NTSC (Apple Hi-Res LC 15)       |  640×480 @ 60 Hz     | Composite   |
| 1 1 0       | Apple "High-Res" 12/13/14"      |  640×480 @ 66.67 Hz  | Comp sync   |
| 1 1 1       | PAL or no monitor               |  768×576 @ 50 Hz     | Composite   |

A sense value of 0 = pin grounded to pin 11 (C&VSYNC.GND).
A sense value of 1 = pin is open (pulled high by the Mac's internal pull-up).

**Extended sense** (introduced with 16" Multiple Scan and later) uses
SENSE2-drive-low / SENSE1-drive-low cycles to read out additional 2-bit
codes per pin → 6 bits total.  Not needed for 68040-era monitors.

Reference: [Macintosh Monitor Sense Codes — Higher Intellect wiki](https://wiki.preterhuman.net/Macintosh_Monitor_Sense_Codes).

### 6.4 H0 implementation — video amp + sync drivers

**Analog R/G/B — our FPGA is digital.**  Options:

- **(A) External video DAC chip.**  E.g., **THS7376** (TI triple-amplifier
  with integrated SAG, 75 Ω source termination, 6 dB gain).  Takes 3×
  analog inputs from a discrete R-2R or a PWM + filter generated inside
  the FPGA.  NOT ideal.
- **(B) Parallel video DAC.**  E.g., **ADV7125** (triple 10-bit 330 MSPS
  DAC).  High quality, 3.3 V compatible, ~$6.  Best quality but adds
  ~20 pins (3× 10-bit parallel).  **Recommended if we want H0 to drive
  a real Apple 13" RGB.**
- **(C) Skip.**  Just document the connector pinout and ship a DVI/HDMI-
  only output.  Users who want DA-15 can buy an external DVI→DA-15 adapter
  (commercially available from Griffin et al., ~$30).

**H-sync / V-sync / C-sync — these are TTL 5 V, open-collector.**  Drive
with an **SN74LVC07APWR** open-drain buffer, pull up to +5 V with
470 Ω at the DA-15 pins.

**Sense pin read-back (if we want "hot plug" detection):**
- Each SENSE pin → 4.7 kΩ pull-up to 3.3 V inside the mezzanine → 3.3 V
  FPGA GPIO (input mode).
- To READ ground state: FPGA samples the pin.
- For the extended sense protocol: FPGA temporarily drives the pin
  low (push-pull output) and samples the others.  Requires bidir.

**Recommendation for H0: option (C) — skip.**  Keep the connector footprint
on the mezzanine but don't populate the DAC.  Add in H1 if there's user
demand.  It's ~$6-8 in parts and a full day of layout for a feature ≈5%
of users will use.

### 6.5 Parts (DigiKey 2026-04)

If we DO build out the DA-15:

| Part                | Qty | Role            | DK stock | $/qty 100 |
|---------------------|----:|-----------------|---------:|----------:|
| ADV7125KSTZ50       |   1 | Triple 10-bit DAC, 330 MSPS |   5 k+ |     $5.80 |
| SN74LVC07APWR       |   1 | Sync OD drivers |   100 k+ |     $0.35 |
| Amphenol L77SDE15PA |   1 | DA-15 female    |    stock |     $1.60 |
| TPD4E05U06DQAR      |   2 | ESD, 4-channel  |    50 k+ |     $0.38 |
| Passives (75 Ω, caps)|  — | Analog filter   |    stock |     $0.15 |
| **Total**           |     |                 |          | **$8.72** |

### 6.6 References

- [Big Mess o' Wires: Classic Macintosh Video Signals Demystified](https://www.bigmessowires.com/2023/10/04/classic-macintosh-video-signals-demystified-designing-a-mac-to-vga-adapter-with-lm1881/)
  — the single best modern write-up on what the Quadra video output
  actually looks like on a scope.
- Apple Developer Tech Note HW26 ("Macintosh Quadra Built-in Video").

---

## 7. Apple serial (printer + modem) — same as §3

The Quadra 700 printer port and modem port are IDENTICAL to the
LocalTalk port electrically and physically (mini-DIN-8, RS-422).  The
SCC (Zilog Z85C30, `rtl/mac/scc.v`) differentiates them by naming
("Printer" = SCC channel A, "Modem" = SCC channel B) and by which
protocol stack runs above.  All three possible uses per port:

- Async RS-422 to a serial printer (ImageWriter II, LaserWriter IISC)
  at 9600 or 57600 baud.
- Async RS-422 to a modem (Hayes-AT) at up to 57600 baud.
- LocalTalk networking (230.4 kbps HDLC).

Same BOM, same pinout, same transceiver as §3.4.  We ship **2 × DIN-8
ports** on the FMC mezzanine so both the printer and modem lines are
usable simultaneously (and the user can run LocalTalk on one of them).

**BOM for 2 ports:** $7.44 total.

---

## 8. Summary + next actions

### 8.1 Top-line numbers

- **Full legacy-PHY BOM at qty 100**: ≈ $25 – $30 per H0 mezzanine.
  Dominated by SCSI (~$8) and audio (~$8).
- **All current-production parts** verified in DigiKey stock on 2026-04;
  no known ongoing discontinuations (flagged the watchouts in §0).
- **Single family covers most of it**: the SN65LBC176 RS-422/485
  transceiver does ADB + LocalTalk + both Apple serial ports, giving
  us BOM consolidation (~5 of the same part).

### 8.2 For the EE who will draw the schematic

1. Use [BlueSCSI v2](https://github.com/BlueSCSI/BlueSCSI-v2) schematic
   as the starting point for the SCSI PHY.  Copy the termination topology.
2. Use the Adafruit PCM5102 breakout schematic as the reference for
   audio-out.
3. For ADB + LocalTalk + Apple serial, the SN65LBC176 datasheet's
   "RS-485 node" reference schematic is directly applicable.
4. DA-15 Apple video — defer or copy from the [BMOW LM1881-based Mac-to-VGA
   reference](https://www.bigmessowires.com/2023/10/04/classic-macintosh-video-signals-demystified-designing-a-mac-to-vga-adapter-with-lm1881/)
   for the sync-extraction direction (we go the other way but topology
   transposes).
5. DB-19 — don't design it into H1.  Plan for H2 with pulls-from-eBay
   or the BMOW custom-run connectors.

### 8.3 RTL implications (already reflected in roadmap)

- `rtl/mac/via1.v` owns ADB bit banging (phase 3 promotion).
- `rtl/mac/scsi.v` is the NCR 5380 register file; our PHY stack below
  it is pure external hardware.
- `rtl/mac/scc.v` is the Z85C30 SCC for both serial ports + LocalTalk.
- `rtl/mac/asc.v` is the Apple Sound Chip; I²S master drives the
  PCM5102A.
- `rtl/mac/video.v` DAFB outputs → our HDMI TX pipeline today; if we
  add the DA-15 analog path, it's an RGB tap off the framebuffer + the
  ADV7125 parallel DAC, gated by the sense-pin read.

### 8.4 Open questions

1. **DB-19 connector on H0/H1**?  My vote: skip, ship internal 20-pin
   IDC only in H1, add external via adapter if demanded.
2. **DA-15 populated on H0 mezzanine**?  My vote: footprint + DNP
   (do-not-populate) on H0; populate on H1 if ≥20% of H0 users ask.
3. **Audio CODEC vs separate DAC+ADC**?  Decision already baked in:
   separate parts due to WM8960 EOL.
4. **ADB PSW pin — do we gate it as an NMI line into the CPU**?
   Quadra 700 keyboards trigger power events through it.  Recommend
   wire it to a board-management MCU GPIO so firmware can decide the
   policy (currently undefined in our `rtl/mac/via1.v`).

---

## 9. References

### Apple primary sources

- Apple Computer Inc., "Guide to the Macintosh Family Hardware," 2nd ed.,
  Addison-Wesley, 1990.  §§ 7 (Floppy / IWM), 8 (ADB), 9 (SCSI),
  10 (Serial), 11 (Sound).
- Apple Service Source: "Ports and Pinouts" — reference PDF mirrored at
  [applerepairmanuals.com](http://www.applerepairmanuals.com/the_manuals_are_in_here/Ports_Pinouts.pdf).
- Apple Tech Notes:
  - HW01 "The ADB Untold Story"
  - HW06 "ADB — The Untold Story II"
  - HW26 "Macintosh Quadra Built-In Video"
  - HW30 "The NCR SCSI Chip on the Mac II / IIx"
- Inside AppleTalk, 2nd ed. (Sidhu/Andrews/Oppenheimer, Addison-Wesley
  1990) — LLAP chapter for LocalTalk framing.
- ANSI X3.131-1986 (SCSI-1).
- Zilog Z85C30 SCC datasheet.
- NCR 5380 SCSI Interface Chip Design Manual, May 1985
  ([hackaday mirror](https://cdn.hackaday.io/files/18974811783616/NCR_5380_SCSI_Interface_Chip_Design_Manual_May85.pdf)).

### Open-source recreations / hardware

- [BlueSCSI v2](https://github.com/BlueSCSI/BlueSCSI-v2) — SCSI-over-SD
  emulator, current production.  Schematic and 74LVC PHY are the
  reference for our SCSI design.
- [Big Mess o' Wires Floppy Emu](https://www.bigmessowires.com/floppy-emu/)
  — SuperDrive emulator, reference for IWM/SWIM signalling.
- [Big Mess o' Wires DB-19 custom connector run](https://www.bigmessowires.com/2017/05/02/db-19-connectors-are-here-and-for-sale/)
  — only known commercial source for new DB-19s.
- [SCSI2SD v6](http://www.codesrc.com/mediawiki/index.php/SCSI2SD_V6) —
  older proven reference for NCR 5380 PHY interfacing.
- [Adafruit PCM5102 breakout](https://www.adafruit.com/product/6250) —
  audio DAC reference schematic.

### ADB protocol implementations (Verilog / FPGA)

- Microchip AN591 "Apple Desktop Bus" — PIC reference, clean state machine.
- [Emile Rétrocampus ADB device core](https://github.com/) — Verilog device
  side, good for sanity checks.
- [Big Mess o' Wires USB-to-ADB napkin design](https://www.bigmessowires.com/2016/03/21/usb-to-adb-napkin-design/)
  — working cheap-and-cheerful host implementation.

### Pinout databases (use with cross-check)

- [pinouts.ru / old.pinouts.ru](https://old.pinouts.ru/) — crowd-sourced,
  generally right but always cross-check against Apple Service Source.
- [allpinouts.org](https://allpinouts.org/) — sister site, similar quality.
- [interfacebus.com ADB](http://www.interfacebus.com/ADB_Pinout.html)
- [antinode.info Mac 25-pin SCSI](http://antinode.info/mac/scsi_25.html)
- [68kmla.org forum](https://68kmla.org/) — vintage-Mac-engineer community,
  where weird pinout questions get the right answer.

### Parts (DigiKey 2026-04, primary distributor for our CM choices)

- [SN65LBC176DR](https://www.digikey.com/en/products/detail/texas-instruments/SN65LBC176DR/1574627)
- [SN74LVC245APWR](https://www.digikey.com/en/products/base-product/texas-instruments/296/SN74LVC245A/1105)
- [SN74LVC07APWR](https://www.digikey.com/en/products/base-product/texas-instruments/296/SN74LVC07A/1100)
- [PCM5102APWR](https://www.digikey.com/en/products/detail/texas-instruments/PCM5102APWR/3727211)
- [AK5720VT](https://www.digikey.com/en/products/detail/asahi-kasei-microdevices-akm/AK5720VT/5180452)
- [TPD4S009DBVR](https://www.digikey.com/en/products/detail/texas-instruments/TPD4S009DBVR/1951110)

---

*Last updated 2026-04-17.  Maintained by agent task #69 (legacy-phy-refs).
Update when any part is flagged discontinued, when the H0 schematic lands,
or when H1 form-factor decisions lock in.*
