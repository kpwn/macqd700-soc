# DP83932C (SONIC) software-observable behaviour audit

**Status: AUDIT ONLY.** No RTL was changed in producing this document.

## What this is

A lookup table for the software-visible behaviour of the SONIC register
interface, comparing three sources for every claim:

| Column | Source | Citation form |
|---|---|---|
| **DS** | `~/DP83932C-20-1.PDF`, extracted to `/tmp/sonic_ds.txt` (`pdftotext -layout`, 4606 lines) | `ds:NNNN` |
| **MAME** | `src/devices/machine/dp83932c.cpp` / `.h` — the project's golden peripheral model | `mame.cpp:NN` / `mame.h:NN` |
| **RTL** | `rtl/mac/q700_eth_sonic.v` (register file), `rtl/mac/q700_sonic_tx.sv`, `rtl/mac/q700_sonic_rx.sv` | `sonic.v:NN`, `tx.sv:NN`, `rx.sv:NN` |

Regenerate the datasheet text with:

```
pdftotext -layout ~/DP83932C-20-1.PDF /tmp/sonic_ds.txt
```

Lines 1573–2269 (all of §4.3.1–4.3.7) come out single-column and clean.
Interleave appears elsewhere (notably ds:320–345 and ds:1130–1270); those
passages are quoted raw where used.

**Where DS and MAME disagree, this document says so and does not pick a
winner.** MAME is the golden model because it demonstrably runs the real
Macintosh driver, and it already carries a `// FIXME: documented byte/word
order doesn't match emulation` on CAM loading (mame.cpp:501) — so
documented-vs-emulated divergence is known to exist here.

Go straight to [§10 Divergence register](#10-divergence-register) for the
ranked findings.

---

## 1. Access model

### 1.1 Register addressing

The SONIC sits on D15..D0 of the Q700's 32-bit bus. `peripheral_bus.v`
turns each four-byte CPU slot into one native 16-bit register transaction.

| Property | Value | Cite |
|---|---|---|
| Window | `0x5000_A000 .. 0x5000_B0FF` (register block aliases every `0x100`) | `peripheral_bus.v:470` |
| Register index | `addr[7:2]` — registers are **4 bytes apart** | `peripheral_bus.v:1610` |
| Connected half | physical bytes **+2/+3** of each slot; +0/+1 are open bus | `peripheral_bus.v:1607-1609` |
| 32-bit read | `{16'hFFFF, sonic_rdata}` — value in the low half | `peripheral_bus.v:1386` |
| 16-bit read at +0 | `0xFFFFFFFF` (open bus) | `peripheral_bus.v:1390` |
| 16-bit read at +2 | `{sonic_rdata, sonic_rdata}` | `peripheral_bus.v:1389` |
| Byte write to +0/+1 | no strobes asserted | `peripheral_bus.v:1628` |
| Register file index space | 64 entries (`0x00`–`0x3F`) | `sonic.v:139` |

`ds:1478` gives the register-address field as `RA<5:0>`, i.e. 64 registers —
consistent.

### 1.2 Read semantics — global

**Reads have no side effects anywhere, in any of the three models.**

- MAME: `u16 reg_r(offs_t offset) { return m_reg[offset]; }` — a bare array
  read (mame.h:24).
- RTL: `assign sonic_rdata = (sonic_cs && sonic_rd) ? sonic_word : 16'hFFFF;`
  (sonic.v:239).
- DS documents no read-triggered action for any user register.

Two datasheet read-back quirks are **not** modelled by either implementation
— see D-16 and D-21 in §10:

| Quirk | DS |
|---|---|
| CAP0/1/2 return valid data **only in reset mode**; "invalid data" otherwise | `ds:1525-1526` |
| Tally counters invert on **write** but *not* on read | `ds:1530-1531` |
| TPS reads back **inverted** relative to what was written (internal-use reg) | `ds:1558` |

### 1.3 Interrupt assertion rule

| | Rule | Cite |
|---|---|---|
| **DS** | "Enabling the corresponding bits in the IMR allows bits in this register to produce an interrupt." Never stated as a formula; `INT = \|(ISR & IMR)` is the only reasonable reading. ISR bits set **regardless** of IMR — IMR gates only the pin. | `ds:2104-2106` |
| **MAME** | `bool const int_state = bool(m_reg[ISR] & m_reg[IMR]);` | `mame.cpp:525` |
| **RTL** | `assign sonic_irq = \|(sonic_reg[REG_ISR] & sonic_reg[REG_IMR]);` | `sonic.v:240` |
| **Verdict** | **Match.** | |

**Polarity.** `ds:2700`: "Interrupt (INT): This signal is active high when
BMODE = 0." The Q700 is BMODE=1 (Motorola/big-endian), so INT is active
**low** on real silicon. The SoC inverts on the way into VIA2 PA0:
`assign via2_pa_in = {1'b1, ~dafb_irq_pb_sync, 5'h1F, ~sonic_irq_pb};`
(`fpga_top_peripherals.vh:933`). **Correct.** (MAME lists "interrupts active
low" as an unfinished TODO at mame.cpp:13 — the RTL is *ahead* of MAME here.)

---

## 2. Reset scope — the load-bearing distinction

The datasheet defines exactly two resets and states plainly that they are not
interchangeable:

> "The SONIC has two reset modes; a hardware reset and a software reset... The
> two reset modes are not interchangeable since each mode performs a different
> function." — `ds:3368-3373`

> "A software reset immediately terminates DMA operations and future
> interrupts. The chip is put into an idle state where registers can be
> accessed, but the SONIC will not be active in any other way. **The registers
> are affected by a software reset as shown in Table 5-4 (only the Command
> Register is changed).**" — `ds:3459-3464`

**Power-on reset is not a third scope.** `ds:3404-3416` says only that the
part must be hardware-reset after power-on. The single "power-up" phrasing in
the register chapter is CR bit 4 STP, `ds:1625`. So in this document
*hardware reset* == *power-on reset* == the RTL's `rst` input.

### 2.1 Reset table

DS column from TABLE 5-4 (`ds:3374-3402`) and the per-register prose.

| Register | DS hardware reset | DS software reset | MAME HW (`device_reset`) | RTL HW (`sonic_reset_value`) | RTL SW (`sonic_enter_reset`) | Verdict |
|---|---|---|---|---|---|---|
| CR | `0x0094` (bits 7,4,2 set, rest cleared) `ds:1584`, `ds:3380` | bits 9,8,1,0 cleared; 7,2 set; **all others unaffected** `ds:1584-1585` | `CR_RST\|CR_STP\|CR_RXDIS` mame.cpp:98 | `CR_RST\|CR_STP\|CR_RXDIS` = `0x0094` sonic.v:167 | `CR & ~(LCAM\|RRRA\|TXP\|HTX) \| RST \| RXDIS` sonic.v:281-283 | **Match, all three.** |
| DCR | **only bits 15 and 13 cleared; all others unaffected** `ds:1669-1670`, `ds:3397-3399` | unaffected `ds:1670` | `&= ~(EXBUS\|LBR)` mame.cpp:99 | **zeroed** sonic.v:171 | untouched | **RTL diverges (D-11).** MAME matches DS. |
| RCR | LB1,LB0,BRD cleared `ds:3401`; per-bit prose also clears RNT `ds:1833` | unaffected `ds:1800` | `&= ~(RNT\|BRD\|LB)` mame.cpp:100 | **zeroed** sonic.v:171 | untouched | **RTL diverges (D-12).** MAME matches DS. |
| TCR | bits 8,0 set, bit 1 cleared = `0x0101` `ds:1905`, `ds:3389` | unaffected `ds:1905` | `\|= NCRS\|PTX; &= ~(TPC\|BCM)` mame.cpp:101-102 | `TCR_NCRS\|TCR_PTX` = `0x0101` sonic.v:168 | untouched | **Match.** |
| IMR | all cleared `ds:2015` | unchanged `ds:3385` | `= 0` mame.cpp:103 | `0x0000` sonic.v:171 | untouched sonic.v:284-291 | **Match. Confirms fix #1.** |
| ISR | cleared `ds:2107` | **unaffected** `ds:2107`, `ds:3387` | `= 0` mame.cpp:104 | `0x0000` | untouched | **Match. Confirms fix #1.** |
| EOBC | `0x02F8` `ds:3392` | unchanged | `0x02f8` mame.cpp:105 | `0x02F8` sonic.v:169 | untouched | **Match.** |
| CE | `0x0000` `ds:3395` | unchanged `ds:3395` | `= 0` mame.cpp:106 | `0x0000` | untouched | **Match. Confirms fix #1.** |
| RSC | `0x0000` `ds:3393` | unchanged `ds:3393` | `= 0` mame.cpp:107 | `0x0000` | untouched | **Match. Confirms fix #1.** |
| DCR2 | all 0 except EXPO[3:0] unknown `ds:2212-2213` | **not affected** `ds:2214` | `= 0` mame.cpp:108 | `0x0000` | untouched | **Match. Confirms fix #1.** |
| SR | — | — | `= 6` (device_start) mame.cpp:93 | `0x0006` sonic.v:170 | untouched | **Match.** |

### 2.2 Confirmation of known fix #1 — software reset touches only CR

**The datasheet agrees, emphatically and in five independent places.**

`ds:1584-1585` gives the CR software-reset mask bit-for-bit:

> "During software reset bits 9, 8, 1, and 0 are cleared and bits 7 and 2 are
> set to a '1'; **all others are unaffected**."

`sonic_enter_reset()` at sonic.v:281-283 computes exactly
`CR & ~(LCAM|RRRA|TXP|HTX) | RST | RXDIS` — clearing bits 9,8,1,0 and setting
7,2. **Bit-exact match to the datasheet**, and identical to MAME
(mame.cpp:248-249).

Per-register confirmation that the other registers must survive:

- ISR — "cleared by a hardware reset and **unaffected by a software reset**" `ds:2107`
- TCR — "This register is **unaffected by a software reset**" `ds:1905`
- RCR — "This register is **unaffected by a software reset**" `ds:1800`
- DCR — "**All bits are unaffected by a software reset**" `ds:1670`
- DCR2 — "A software reset will **not affect any bits** in this register" `ds:2214`
- IMR / CE / RSC — "unchanged" in TABLE 5-4 `ds:3385`, `ds:3395`, `ds:3393`

The prior behaviour (clearing IMR on the software path) contradicted
`ds:3385` directly and silently disarmed every interrupt the moment a driver
reset the chip after programming it. **Fix confirmed correct against the
datasheet.** Regression-locked at `tb/tb_q700_eth_sonic.cpp:290-297`.

One residual: `ds:1585` says RXEN (bit 3) is *unaffected*, but TABLE 5-4's
software-reset values (`0094h`/`00A4h`, `ds:3380`) both show bit 3 = 0. RTL
and MAME both follow the prose and leave RXEN set. That is internally
consistent, but it is what makes **D-5** (§10) bite.

---
## 3. Command Register — CR (RA `0x00`)

### 3.1 Write mask and global rules

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write mask | bits 9,8,7,5,4,3,2,1,0 → `0x03BF` `ds:1587-1589` | `0x03bf` mame.cpp:57 | `0x03BF` sonic.v:180 | **Match** |
| Write-0 | "With the exception of RST, writing a '0' to any bit has no effect" `ds:1577-1578` — CR is **set-only** | OR-in: `m_reg[offset] \|= data & regmask` mame.cpp:263 | OR-in: `cr_next = CR \| (data & writable)` sonic.v:466 | **Match** |
| Self-clear | "For all bits, except for the RST bit, the SONIC resets the bit after the command is completed" `ds:1577` | in `command()` mame.cpp:304-357 | in the CR arm + engine writeback sonic.v:467-494, 581-604 | **Match** |
| Gate while RST set | "Before any commands can be issued, the RST bit must first be reset to '0'... **two writes** are required" `ds:1578-1580` | all CR writes gated mame.cpp:233-241 | 16-bit path gated sonic.v:460-464; **byte path is not** sonic.v:374-390 | **D-18** |

### 3.2 Per-bit command semantics

| Bit | Name | DS behaviour | MAME | RTL | Verdict |
|---|---|---|---|---|---|
| 9 | LCAM | Loads CAM from CDP. Self-clears; sets ISR.LCD. `ds:1606-1608` | mame.cpp:350-354, 519-520 | sonic.v:549-551 issue; cleared on `rx_done` sonic.v:590-597 | **Match** |
| 8 | RRRA | Reads next RRA descriptor at RRP. Self-clears. `ds:1611-1614` | mame.cpp:344-348, 482-483 | sonic.v:552-554; cleared sonic.v:599-603 | **Match** |
| 7 | RST | Software reset — see §2. Not self-clearing; the one bit where write-0 acts. `ds:1615-1618` | mame.cpp:233-250 | sonic.v:316-320, 460-464 | **Match** |
| 5 | ST | Start timer. **"Setting this bit resets STP."** `ds:1622` | `CR &= ~CR_STP` mame.cpp:338-342 | `if (data[5]) cr_next &= ~CR_STP` sonic.v:471 | **Match** (bit unused — no timer, D-13) |
| 4 | STP | Stop timer, **resets ST**. `ds:1623-1625` | `CR &= ~CR_ST` mame.cpp:332-336 | sonic.v:470 | **Match** (D-13) |
| 3 | RXEN | Receiver enable. **"Setting this bit resets the RXDIS bit."** `ds:1637-1638` | `CR &= ~CR_RXDIS` mame.cpp:326-330 | sonic.v:469 | **Match** |
| 2 | RXDIS | Receiver disable. **"The RXEN bit is reset when the receiver is disabled."** `ds:1643-1644`. Exception: when RXEN *and* RXDIS are both set, "RXDIS could be cleared by writing zero to it" `ds:1645-1646` | `CR &= ~CR_RXEN` mame.cpp:320-324; write-0 exception not modelled | sonic.v:468; write-0 exception not modelled | **Match** to MAME; **D-24** vs DS |
| 1 | TXP | Transmit. SONIC clears it on (1) EOL detected, (2) HTX taken effect, (3) transmit abort (EXC/EXD/FU/BCM). `ds:1648-1655` | mame.cpp:312-318; suppressed if already set mame.cpp:254-262 | sonic.v:478-493; suppressed by `(CR & CR_TXP)==0 && !tx_cmd_valid` sonic.v:481 | **Match** on issue; abort conditions never arise (D-15) |
| 0 | HTX | **"Setting this bit halts the transmit command after the current transmission has completed. TXP is reset after transmission has halted. The CTDA register points to the last descriptor transmitted. The SONIC samples this bit after writing to the TXpkt.status field."** `ds:1656-1659`. Also "If the halt transmit command is issued... the CTDA register is not loaded" `ds:1224-1227` | Implemented: `if (!(m_reg[CR] & CR_HTX))` gates the CTDA reload and the next-packet chain mame.cpp:434-455 | **CR bit only.** `q700_sonic_tx.sv` has **no halt port at all**; the engine walks to EOL regardless | **D-2 — DIVERGENCE** |

### 3.3 Confirmation of known fix #2 — TXP/HTX cross-clear

| Direction | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| **HTX clears TXP** | **Yes, explicit:** "TXP is reset after transmission has halted" `ds:1656-1657` | mame.cpp:306-310 | sonic.v:467 (`if (data[0]) cr_next &= ~CR_TXP`) | **Confirmed by DS.** |
| **TXP clears HTX** | **Not stated anywhere.** The datasheet never says TXP affects HTX. Only the blanket self-clear rule `ds:1577` covers HTX. | mame.cpp:316 (`m_reg[CR] &= ~CR_HTX`) | sonic.v:479 (`cr_next &= ~CR_HTX`) | **DS silent — RTL follows MAME.** Deliberate, see **D-26**. |

So half of fix #2 is datasheet-backed and half is MAME-only. The MAME-only half
is defensible (a stuck HTX would otherwise wedge every future transmit, and
`ds:1577` says command bits self-clear once complete), but it should be
recorded as a conscious choice to follow the golden model past the document.

**One subtlety in the RTL.** sonic.v:479 clears HTX unconditionally whenever
TXP is written, *including* when the TXP command itself is suppressed because a
transmit is already in flight (sonic.v:481). MAME clears HTX inside `command()`,
which only runs for the TXP bit that survived the suppression at mame.cpp:261 —
so MAME would leave HTX set in that case. Narrow, but a real behavioural delta
on the dynamic-TDA-append path that mame.cpp:256-260 exists to support.

### 3.4 Confirmation of known fix #3 — CPU/engine write race on CR

Not a datasheet topic; a pure RTL-correctness fix. Present and correct:

- `cpu_wr_cr` (sonic.v:267) tells the engine writeback to stand down.
- `cr_engine_clear` (sonic.v:268-272) folds the engine's pending TXP/LCAM/RRRA
  clears into the CPU's own nonblocking assignment, so the CPU write carries
  them rather than being overwritten by a later statement in the same
  `always` block.
- Applied on all three CR write paths: sonic.v:318, sonic.v:355, sonic.v:494.
- Engine side defers: sonic.v:582, 594, 600.

The comment at sonic.v:259-266 states the failure mode correctly — a swallowed
`CR_TXP` is a wedge, not a lost status bit.

**Residual: the same protection does not extend to any other register.** See
**D-19**.

---

## 4. Configuration and status registers

### 4.1 DCR (RA `0x01`) — Data Configuration

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write mask | bits 15,13..0 (bit 14 = 0) → `0xBFFF` `ds:1673-1675` | `0xbfff` mame.cpp:57 | `0xBFFF` sonic.v:181 | **Match** |
| Writable only in reset mode | **"This register must only be accessed when the SONIC is in reset mode"** `ds:1670-1671`; "Writing to these registers while not in reset mode **does not alter the registers**" `ds:1528-1529` | **Not enforced** — `// TODO: can only write during reset: DCR, DCR2` mame.cpp:228 | **Not enforced** — falls to the `default` arm sonic.v:507-511 | **D-17** (shared with MAME) |
| HW reset | **only bits 15 and 13 cleared; all others unaffected** `ds:1669-1670` | `&= ~(EXBUS\|LBR)` mame.cpp:99 | **zeroed** sonic.v:171 | **D-11** |
| SW reset | unaffected `ds:1670` | untouched | untouched sonic.v:284-291 | **Match** |
| USR1,0 (bits 9,8) | **pin-sampled at hardware reset, not software-written** `ds:1742-1746` | `// TODO: sample USR1,0` mame.cpp:99 | not modelled | **D-17** (shared) |

**Only bit 5 (DW, data width) is functionally consumed by the RTL** — it selects
16- vs 32-bit descriptor stride: `wide_desc <= start_dcr[5]` (tx.sv:231),
`wide_desc <= cfg_dcr[5]` (rx.sv:304). All other DCR bits are stored and read
back but drive nothing (FIFO thresholds, wait states, block mode, bus retry and
the programmable outputs have no analogue in this SoC). That is appropriate —
they describe a bus this design does not have.

### 4.2 RCR (RA `0x02`) — Receive Control

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write mask | bits 15–9 r/w, bits 8–0 read-only status → `0xFE00` `ds:1802-1804` | `0xfe00` mame.cpp:57 | `0xFE00` sonic.v:182 | **Match** |
| Engine also writes it | bits 8–0 written per received packet; "all bits in the RCR are written into the RXpkt.status field" `ds:1797-1799` | mame.cpp:128, 141-150, 551-561 | rx.sv:393-396 → `done_rcr` → sonic.v:611 | **Match** |
| Status clear | "Bits 8–6 and 3–0 are cleared at the reception of the next packet" `ds:1798-1799` (**bits 5 CRS / 4 COL have no stated rule**) | `&= ~(MC\|BC\|LPKT\|CRCR\|FAER\|LBK\|PRX)` mame.cpp:128 | `status_rcr <= rcr & 0xFE00` rebuilt per frame rx.sv:349, 393 | **Match** |
| HW reset | LB1,LB0,BRD cleared `ds:3401`; per-bit prose also clears RNT `ds:1833` | `&= ~(RNT\|BRD\|LB)` mame.cpp:100 | **zeroed** sonic.v:171 | **D-11** |
| SW reset | **unaffected** `ds:1800` | untouched | untouched | **Match** |

**Filter bit implementation status:**

| Bit | Name | DS | MAME | RTL | Verdict |
|---|---|---|---|---|---|
| 15 | ERR — accept CRC-error / collision packets | `ds:1827-1829` | mame.cpp:140-144, sets RCR_CRCR | **ignored** — `frame_bad` always drops rx.sv:384 | **D-10** |
| 14 | RNT — accept runt (<64 byte) packets | `ds:1830-1833` | mame.cpp:134 | rx.sv:384 `(rda_len<64)&&((rcr&RCR_RNT)==0)` | **Match** |
| 13 | BRD — accept broadcast | `ds:1835-1838` | mame.cpp:546 | rx.sv:156 | **Match** |
| 12 | PRO — physical promiscuous | `ds:1840-1843` | mame.cpp:536-541 | rx.sv:155 | **Match** |
| 11 | AMC — accept all multicast (broadcast is a subset, accepted regardless of BRD) | `ds:1844-1847` | mame.cpp:546, 556 | rx.sv:156-157 | **Match** |
| 10,9 | LB1,LB0 — loopback control | `ds:1857-1866` | `set_loopback()` mame.cpp:270 | **no loopback path**; only the LBK *status* bit is faked rx.sv:394 | **D-12** |

Status bits 8–0 (MC, BC, LPKT, CRS, COL, CRCR, FAER, LBK, PRX) are produced at
rx.sv:393-396. **CRS(5), COL(4), CRCR(3), FAER(2) are never set** by the RTL —
there is no collision or carrier model, and CRC errors drop the frame (D-10).

### 4.3 TCR (RA `0x03`) — Transmit Control

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write mask | bits 15–12 r/w (PINT, POWC, CRCI, EXDIS), bits 11–0 read-only → `0xF000` `ds:1907-1909` | `0xf000` mame.cpp:57 | `0xF000` sonic.v:183 | **Match** |
| **Engine also writes it** | "At the beginning of transmission, bits 15, 14, 13 and 12 **from the TXpkt.config field are loaded into the TCR**" `ds:1899-1900`, `ds:1217`. At the end, bits 10–0 hold status. | `m_reg[TCR] = read_bus_word(...) & TCR_TPC` mame.cpp:369, then `\|= TCR_PTX` mame.cpp:427 — **config half preserved** | `done_tcr <= (descriptor_tcr \| TCR_PTX) & 16'h07ff` tx.sv:349 — `descriptor_tcr` is masked to `0xF000` (tx.sv:250), so `& 0x07FF` **erases it**; TCR always reads back `0x0001` | **D-3 — DIVERGENCE** |
| HW reset | bits 8,0 set, bit 1 cleared = `0x0101` `ds:1905` | mame.cpp:101-102 | `0x0101` sonic.v:168 | **Match** |
| SW reset | **unaffected** `ds:1905` | untouched | untouched | **Match** |

**Per-bit configuration half:**

| Bit | Name | DS | MAME | RTL | Verdict |
|---|---|---|---|---|---|
| 15 | PINT | "The SONIC will issue an interrupt... **immediately after reading a TDA** and detecting that PINT is set in the TXpkt.config field." **"PINT in the TCR must be cleared before it is set again in order to have the interrupt issued for another packet."** `ds:1930-1936` | Edge-detected: `if ((m_reg[TCR] & TCR_PINT) && !(tcr & TCR_PINT))` mame.cpp:376 — exactly the alternation rule | `done_pint <= descriptor_tcr[15] && !failed` tx.sv:380 — **fires on every** PINT descriptor, no edge gate. Also fires at *end* of transmission, not on TDA read. | **D-4 — DIVERGENCE** |
| 14 | POWC | out-of-window collision timer start point `ds:1938-1941` | stored only | stored only | benign (no collision model) |
| 13 | **CRCI** | **"0: transmit packet with 4-byte FCS field. 1: transmit packet without 4-byte FCS field."** `ds:1942-1944` | `if (!(m_reg[TCR] & TCR_CRCI))` gates the FCS append mame.cpp:400-407 | **Captured but never used.** `descriptor_tcr` keeps bit 13 (tx.sv:250) but nothing reads it; the MAC always appends an FCS. | **D-6 — DIVERGENCE (known, open)** |
| 12 | EXDIS | excessive-deferral timer disable `ds:1945-1947` | stored only | stored only | benign |

**Status half (bits 10–0)** — all produced by the engine on real silicon.
The RTL writes a hardcoded `TCR_PTX` (`0x0001`) into `TXpkt.status`
(tx.sv:189-192) and reports `0x0001` in `done_tcr`. For a clean, collision-free
transmit that is the **correct** value (`ds:1230-1238`: status = `{NC4..NC0,
TCR[10:0]}`, so zero collisions + PTX). The divergence is only that no error
condition can ever be reported — see **D-15** (BCM) and D-22.

#### CRCI in full — what the datasheet actually says

The complete §4.3.4 description is three lines (`ds:1942-1944`):

```
13   CRCI: CRC INHIBIT
     0: transmit packet with 4-byte FCS field.
     1: transmit packet without 4-byte FCS field.
```

That is **all** of it. The only other substantive statements are:

1. **Purpose** (`ds:337-345`, column-interleaved — left column is the prose):
   > "For bridging or switched ethernet applications the CRC Generator can be
   > inhibited by setting bit 13 in the Transmit Control Register (Section
   > 4.3.4). This feature is used when **an ethernet segment has already
   > received a packet with a CRC appended and needs to forward it** to another
   > ethernet segment."

2. **Interaction with PMB** (`ds:1987-1988`):
   > "Note 2: If CRC has been inhibited for transmissions (CRCI is set), this
   > bit will always be low. This is true regardless of Frame Alignment or
   > Source Address mismatch errors."

3. **The TCR config half is loaded from `TXpkt.config` per descriptor**, so CRCI
   is a **per-packet** property, not a global mode (`ds:1206-1207`, `ds:1217`).

**On the byte-count question the datasheet is silent.** There is no sentence
anywhere stating whether `TXpkt.pkt_size` (TPS) / `TXpkt.frag_size` (TFS)
include the driver-supplied FCS when CRCI is set. The only definitions are
`ds:1146-1147` ("TXpkt.pkt_size: This field contains the byte count of the
entire packet") and `ds:1157-1159` ("TXpkt.frag_size: This field contains the
byte count of the packet fragment").

**Inference (flagged as inference, not text):** the hardware FCS is appended
*after* the DMA'd fragment data, and the documented use case is forwarding a
packet that already carries a CRC in the buffer. Therefore, with CRCI set, the
driver's FCS must lie **inside the fragment data** and must be **counted in
both TFS and TPS**. MAME implements exactly this: it DMAs `TFS` bytes per
fragment and simply skips the 4-byte append (mame.cpp:395-407), so whatever the
driver placed in the buffer — FCS included — goes on the wire verbatim. This is
the behaviour to implement.

**Implementation note for whoever closes D-6:** the RTL's TX path does not
generate the FCS itself — the Taxi MAC appends it downstream of
`tx_axis_t*`. Honouring CRCI therefore requires a *new signal* out of
`q700_sonic_tx` telling the MAC not to append, not merely a change inside the
descriptor parser. There is no such port today.

### 4.4 IMR (RA `0x04`) — Interrupt Mask

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write mask | bits 14–0 (bit 15 must be 0) → `0x7FFF` `ds:2017-2019` | `0x7fff` mame.cpp:57 | `0x7FFF` sonic.v:184 | **Match** |
| Semantics | "Writing a '1' to the bit enables the corresponding interrupt" `ds:2014-2015` | plain masked store + `update_interrupts()` mame.cpp:273-276 | plain masked store sonic.v:400-403 | **Match** |
| Read | no side effect | array read | array read | **Match** |
| HW reset | all cleared `ds:2015` | `= 0` mame.cpp:103 | `0x0000` | **Match** |
| SW reset | **unchanged** `ds:3385` | untouched mame.cpp:248-249 | untouched sonic.v:284-291 | **Match — fix #1 confirmed** |

Bit assignments (`ds:2017-2019`, all confirmed identical in mame.h:163-180): 14 BREN,
13 HBLEN, 12 LCDEN, 11 PINTEN, 10 PRXEN, 9 PTXEN, 8 TXEREN, 7 TCEN, 6 RDEEN,
5 RBEEN, 4 RBAEEN, 3 CRCEN, 2 FAEEN, 1 MPEN, 0 RFOEN. The RTL never names IMR
bits individually — it only ANDs the whole register against ISR (sonic.v:240),
which is correct and mask-agnostic.

### 4.5 ISR (RA `0x05`) — Interrupt Status

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write mask | bits 14–0 → `0x7FFF` `ds:2109-2111` | `0x7fff` mame.cpp:57 | `0x7FFF` sonic.v:185 | **Match** |
| Write semantics | **write-1-to-clear**: "A bit is cleared by writing '1' to it. Writing a '0' to any bit has no effect." `ds:2106` | `m_reg[offset] &= ~(data & regmask)` mame.cpp:283 | `isr_bus_clear` (sonic.v:248-251) applied at sonic.v:621-623 | **Match** |
| Read | no side effect | array read | array read | **Match** |
| Set by | the engine, **regardless of IMR** `ds:2104-2106` | throughout | `tx_isr_event` / `rx_isr_event` sonic.v:254-257 | **Match** |
| HW reset | cleared `ds:2107` | `= 0` mame.cpp:104 | `0x0000` | **Match** |
| SW reset | **unaffected** `ds:2107` | untouched | untouched | **Match — fix #1 confirmed** |

The W1C merge at sonic.v:621-623 is written so a same-cycle engine event and a
CPU clear compose rather than race:
`ISR <= (ISR & ~isr_bus_clear) | tx_isr_event | rx_isr_event`. Correct — a
newly-raised bit survives a clear of a *different* bit in the same cycle.

**Per-bit encoding — this is where the RTL has a hard bug.**

| Bit | Mask | DS name `ds:2109-2110` | MAME `mame.h:181-198` | RTL | Verdict |
|---|---|---|---|---|---|
| 14 | `0x4000` | BR | `ISR_BR` | not modelled | D-22 |
| 13 | `0x2000` | HBL | `ISR_HBL` | not modelled | D-22 |
| 12 | `0x1000` | LCD | `ISR_LCD = 0x1000` | `ISR_LCD = 16'h1000` sonic.v:125 | **Match** |
| 11 | `0x0800` | PINT | `ISR_PINT = 0x0800` | `ISR_PINT = 16'h0800` sonic.v:120 | **Match** |
| 10 | `0x0400` | **PKTRX** | `ISR_PKTRX = 0x0400` | `ISR_PKTRX = 16'h0400` **rx.sv:70** | **Match** (RX engine) |
| 10 | `0x0400` | **PKTRX** | — | **`ISR_TXER = 16'h0400`** sonic.v:122 | **D-1 — WRONG BIT** |
| 9 | `0x0200` | TXDN | `ISR_TXDN = 0x0200` | `ISR_TXDN = 16'h0200` sonic.v:121 | **Match** |
| 8 | `0x0100` | **TXER** | `ISR_TXER = 0x0100` | **absent** | **D-1** |
| 7 | `0x0080` | TC | `ISR_TC` | not modelled | D-13 |
| 6 | `0x0040` | RDE | `ISR_RDE = 0x0040` | sonic.v:124, rx.sv:70 | **Match** |
| 5 | `0x0020` | RBE | `ISR_RBE = 0x0020` | sonic.v:123, rx.sv:69 | **Match** |
| 4 | `0x0010` | RBAE | `ISR_RBAE = 0x0010` | rx.sv:69 | **Match** |
| 3..1 | | CRC, FAE, MP | present | not modelled | D-14 |
| 0 | `0x0001` | RFO | `ISR_RFO` | not modelled | D-22 |

**The register file and its own RX engine assign contradictory meanings to bit
10.** `sonic.v:122` calls `0x0400` "TXER"; `rx.sv:70` calls the same bit
"PKTRX". The datasheet (`ds:2109-2110`) and MAME (`mame.h:191, 193`) both say
bit 10 is PKTRX and bit 8 is TXER. See **D-1**.

**Documented clear-side effects:**

| Bit | DS side effect of clearing | MAME | RTL |
|---|---|---|---|
| RBE (5) | "resetting this bit causes the SONIC to read the next resource descriptor pointed to by the RRP... similar to issuing the Read RRA command" `ds:2185-2187` | `read_rra()` on RBE clear mame.cpp:280-281 | `rx_recovery_pending[0]` → `cfg_op` bit 0 sonic.v:499-501, rx.sv:313-315 | **Match** |
| RDE (6) | **no documented side effect**; recovery is the lazy link re-read at reception `ds:1066-1075` | none; MAME re-reads CRDA from LLFA lazily mame.cpp:120-126 | `rx_recovery_pending[1]` → an explicit reload op sonic.v:499-501, rx.sv:316-317; **and** the lazy `S_EOL_RETRY` path rx.sv:386-389, 458-469 | **Deliberate superset** — D-27 |
| BR (14) | "Before the SONIC will continue any DMA operations, BR must be cleared" `ds:2136-2138` | not modelled | not modelled | D-22 |

### 4.6 DCR2 (RA `0x3F`) — Data Configuration 2

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write mask | bits 15–12 EXPO, 4 PH, 2 PCM, 1 PCNM, 0 RJCM → `0xF017` `ds:2216-2218` | `0xf017` mame.cpp:65 | `0xF017` sonic.v:228 | **Match** |
| Writable only in reset mode | "This register should only be written to when the SONIC is in software reset" `ds:2214-2215`, `ds:1528-1529` | not enforced mame.cpp:228 | not enforced sonic.v:423-427 | **D-17** (shared) |
| HW reset | all 0 except EXPO[3:0] "unknown until written to" `ds:2212-2213` | `= 0` mame.cpp:108 | `0x0000` | **Match** (both make EXPO deterministic — harmless) |
| SW reset | **"will not affect any bits"** `ds:2214` | untouched | untouched | **Match — fix #1 confirmed** |
| RJCM / PCM / PCNM | reject-on-CAM-match and packet-compress bridge modes `ds:2241-2264` | stored only | stored only | benign — no Mac driver sets them |

---
## 5. Transmit registers

`ds:2271-2313` describes only UTDA and CTDA. TPS/TFC/TSA0/TSA1/TFS/TTDA appear
only in **TABLE 4-2, "Internal Use Registers (Users should not write to these
registers)"** (`ds:1533-1557`) and in the §3.5 load list (`ds:1217-1223`).

| Reg | RA | Mask (DS / MAME / RTL) | Who writes it | Read semantics | Reset | Verdict |
|---|---|---|---|---|---|---|
| **UTDA** | 06 | `0xFFFF` / `0xffff` mame.cpp:57 / `0xFFFF` sonic.v:186 | **SW only.** A`<31:16>` of the TDA `ds:2282` | plain | **"unaffected by a hardware or software reset"** `ds:2292-2293` | mask **Match**; reset **D-11** |
| **CTDA** | 07 | `0xFFFF` / `0xffff` / `0xFFFF` sonic.v:187 | **Both.** SW at init `ds:2299-2301`; engine does `CTDA ← TXpkt.link` `ds:1224`, **only after all fragments transmitted, and *not* if HTX was issued** `ds:1225-1227` | plain; **bit 0 is the EOL flag, not an address bit** `ds:2296-2297` | unaffected `ds:2312-2313` | mask **Match**; writeback **Match** (tx.sv:367 → sonic.v:584); HTX exception **D-2**; reset **D-11** |
| **TPS** | 08 | `0xFFFF` / `0xffff` / `0xFFFF` sonic.v:188 | Engine: `TPS ← TXpkt.pkt_size` `ds:1218` | **DS: "The data that is read from these registers is the inversion of what has been written"** `ds:1558` | — | writeback **Match** (tx.sv:251 → sonic.v:586); read inversion **D-21** (shared with MAME) |
| **TFC** | 09 | `0xFFFF` all three, sonic.v:189 | Engine: `TFC ← TXpkt.frag_count` `ds:1219` | plain | — | **Match** (tx.sv:252 → sonic.v:587) |
| **TSA0/TSA1** | 0A/0B | `0xFFFF` all three, sonic.v:190-191 | Engine: `TSA0/1 ← TXpkt.frag_ptr0/1` per fragment `ds:1220-1222` | plain | — | MAME updates them mame.cpp:387-388; **RTL never does** — **D-20** |
| **TFS** | 0C | `0xFFFF` all three, sonic.v:192 | Engine: `TFS ← TXpkt.frag_size` `ds:1223` | **DS: written value is "shifted once in 16-bit mode and shifted twice in 32-bit mode"** `ds:1559` | — | MAME updates mame.cpp:389; **RTL never does** — **D-20**; shift **D-21** |
| **TTDA** | 20 | `0xFFFF` all three, sonic.v:212 | Engine keeps a copy of CTDA for collision retry `ds:1247-1248` | plain | — | RTL sets it at TXP issue (sonic.v:345, 485) — **Match** in spirit |

**Transmit sequence, three-way:**

| Step | DS | MAME | RTL |
|---|---|---|---|
| Load TCR/TPS/TFC from TDA words 1,2,3 | `ds:1217-1219` | mame.cpp:369-371 | tx.sv:250-253 |
| Read fragments (ptr0, ptr1, size) × TFC | `ds:1220-1223`, "6, 3, or 2 accesses" `ds:1256-1261` | mame.cpp:384-397 | tx.sv:262-283 (3 words per fragment at `descriptor_addr + (4 + i*3)*word_bytes`, tx.sv:172) |
| Append FCS unless CRCI | `ds:1942-1944` | mame.cpp:400-407 | **absent** — **D-6** |
| Write `TXpkt.status` | "At the end of packet transmission, status is written into the TXpkt.status field" `ds:1237-1238` | mame.cpp:431 | tx.sv:186-196 (ST_STATUS_REQ) |
| **Then** read `TXpkt.link` | "The SONIC **then** reads the TXpkt.link field" `ds:1238-1239` | mame.cpp:437 | tx.sv:197-202 (ST_LINK_REQ) — **correct order** |
| EOL=1 → ISR.TXDN + clear CR.TXP | `ds:1239-1244`, `ds:2152-2156` | mame.cpp:440-446 | tx.sv:369-370 → `done_valid`; sonic.v:255, 583 |
| EOL=0 → next descriptor | `ds:1241-1243` | mame.cpp:448-452 | tx.sv:372-373 (loops internally, no ISR per packet) |
| HTX sampled after status write | `ds:1658-1659` | mame.cpp:434 | **absent** — **D-2** |

The internal EOL loop means the RTL raises ISR.TXDN exactly once per TXP
command, at end of list — which matches both DS and MAME.

---

## 6. Receive registers

Scope statement, `ds:2316-2321`: **"A software reset has no effect on these
registers and a hardware reset only affects the EOBC and RSC registers. The
receive registers must be initialized prior to issuing the receive command."**

| Reg | RA | Mask (DS / MAME / RTL) | Who writes it | Reset (DS) | Verdict |
|---|---|---|---|---|---|
| **URDA** | 0D | `0xFFFF` / `0xffff` / `0xFFFF` sonic.v:193 | SW only; A`<31:16>` `ds:2323` | unaffected `ds:2328-2329` | mask **Match**; reset **D-11** |
| **CRDA** | 0E | `0xFFFF` all three, sonic.v:194 | Both; **bit 0 = EOL** `ds:2333, 2340-2341` | unaffected `ds:2341-2342` | mask **Match**; writeback rx.sv:445 → sonic.v:612 **Match** (mame.cpp:194) |
| **CRBA0/1** | 0F/10 | `0xFFFF` all three, sonic.v:195-196 | Engine, from `RXrsrc.buff_ptr0/1` `ds:1012-1013`; advanced per packet | — | **Match** (rx.sv:327, 435 → sonic.v:613; mame.cpp:464-465, 171-172) |
| **RBWC0/1** | 11/12 | `0xFFFF` all three, sonic.v:197-198 | Engine, from `RXrsrc.buff_wc0/1`; decremented per word `ds:1080-1082` | — | **Match** (rx.sv:141, 435 → sonic.v:614; mame.cpp:176-179) |
| **EOBC** | 13 | `0xFFFF` all three, sonic.v:199 | SW; static during operation `ds:2342-2347` | **`0x02F8`** HW, unchanged SW `ds:3384` | mask **Match**; reset value **Match** (sonic.v:169, mame.cpp:105). DS also says the SONIC **holds the LSB low in 32-bit mode** `ds:1019-1020` — **D-23** (shared) |
| **URRA** | 14 | `0xFFFF` all three, sonic.v:200 | SW; A`<31:16>` for **both** RRA and CDA `ds:2289-2292, 1360-1364` | unaffected `ds:2316-2318` | **Match** (used for both: rx.sv:227, 256, 260) |
| **RSA** | 15 | **15-bit, "LSB... always reads back as a 0"** `ds:2298-2300` / `0xfffe` / `0xFFFE` sonic.v:201 | SW only | unaffected | **Match** |
| **REA** | 16 | same LSB rule `ds:2312-2315` / `0xfffe` / `0xFFFE` sonic.v:202 | SW only. **One-past-the-last descriptor** `ds:952-955` | unaffected | **Match** (wrap `rrp==rea → rsa`: rx.sv:329, mame.cpp:476-477) |
| **RRP** | 17 | same LSB rule `ds:2319-2322` / `0xfffe` / `0xFFFE` sonic.v:203 | **Both.** Engine advances 4 words / 4 long words per RRA read, auto-wraps `ds:813-818` | unaffected | **Match** (rx.sv:329 → sonic.v:615; mame.cpp:473) |
| **RWP** | 18 | same LSB rule `ds:2326-2329` / `0xfffe` / `0xFFFE` sonic.v:204 | **SW only** `ds:2329-2331` | unaffected | **Match**. Never written back by the engine, so **D-19 does not endanger the resource-replenish path** |
| **TRBA0/1** | 19/1A | `0xFFFF` all three, sonic.v:205-206 | Engine; saved copy of CRBA for runt/error recovery `ds:1102-1105` | — | **Match** (rx.sv:397 → sonic.v:616-617; mame.cpp:156-157) |
| **TBWC0/1** | 1B/1C | `0xFFFF` all three, sonic.v:207-208 | Engine; restores RBWC `ds:1105-1107` | — | **Match** (rx.sv:397 → sonic.v:617-618; mame.cpp:158-159) |
| **LLFA** | 1F | `0xFFFF` all three, sonic.v:211 | Engine: address of the current `RXpkt.link` field | **DS gives LLFA no prose at all** — TABLE 4-2 lists it with description "none" `ds:1554` | RTL/MAME agree: `LLFA = CRDA + 5*width` (rx.sv:440, mame.cpp:193) |
| **RSC** | 2B | `0xFFFF` all three, sonic.v:223 | **Both.** `[15:8]` RBA sequence, `[7:0]` packet sequence, both modulo-256 `ds:2352-2354` | `0x0000` HW; **"or by writing zero to it. A software reset has no affect."** `ds:2350-2352` | **Match** — increment split identical to MAME: packet+1 (rx.sv:517, mame.cpp:209) vs RBA+0x100 with packet reset (rx.sv:514, mame.cpp:485) |

**RRA / resource-exhaustion behaviour:**

| Behaviour | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| RRP==RWP → ISR.RBE | `ds:1138-1144`; **"comparison is made after the complete RRA descriptor has been read"** `ds:966-969` | mame.cpp:479-480 | rx.sv:337-340 (after the RRA response) | **Match** |
| RBE set on the *second-to-last* buffer | `ds:1140-1144` | mame.cpp:479 | rx.sv:337 | **Match** |
| Reception stops until RBE cleared | `ds:2182-2184` | mame.cpp:116 | `resource_blocked` gates `rx_axis_tready` rx.sv:198-199 | **Match** |
| Clearing RBE re-reads the RRA | `ds:2185-2187` | mame.cpp:280-281 | sonic.v:499-501 → rx.sv:313-315 | **Match** |
| RRRA command self-clears | `ds:1016-1019` | mame.cpp:482-483 | sonic.v:599-603 | **Match** |
| Buffer overrun → ISR.RBAE, no RDA written | `ds:2189-2192` | `// TODO` mame.cpp:185 | rx.sv:390-391 (`(rbwc<<1) < rda_len` → RBAE, no RDA) | **RTL ahead of MAME** |

---

## 7. CAM registers and the CAM access model

Scope, `ds:2359-2363`: **"These registers, except for the CAM Enable register,
are unaffected by a hardware or software reset."**

### 7.1 Register table

| Reg | RA | DS | MAME mask | RTL mask | Verdict |
|---|---|---|---|---|---|
| **CEP** | 21 R/W | **4-bit**; "SONIC uses the least significant 4-bits... 0h points to the first CAM entry and Fh to the last" `ds:2371-2375` | `0x000f` mame.cpp:62 | `0x000F` sonic.v:213 | **Match** |
| **CAP2** | 22 **R** | read-only; CAM bits `<47:32>` `ds:2381-2383` | `0x0000` mame.cpp:62 | `0x0000` sonic.v:214 | mask **Match**; **never populated — D-9** |
| **CAP1** | 23 **R** | read-only; CAM bits `<31:16>` | `0x0000` | `0x0000` sonic.v:215 | **D-9** |
| **CAP0** | 24 **R** | read-only; CAM bits `<15:0>` | `0x0000` | `0x0000` sonic.v:216 | **D-9** |
| **CE** | 25 R/W | 16-bit enable mask, one bit per CAM entry `ds:2412-2417`. **"can only be written to when the SONIC is in reset mode"** `ds:1527` | `0xffff` mame.cpp:62 | `0xFFFF` sonic.v:217 | mask **Match**; reset-mode gate **D-17**; HW reset `0x0000` / SW unchanged `ds:2417-2419` — **Match** |
| **CDP** | 26 R/W | **15-bit, "LSB is unused and always reads back as 0"** `ds:2424-2426` | `0xfffe` mame.cpp:62 | `0xFFFE` sonic.v:218 | **Match** |
| **CDC** | 27 R/W | **5-bit** `ds:2376-2377, 1346-1348` | `0x001f` mame.cpp:62 | `0x001F` sonic.v:219 | **Match** |

### 7.2 The CAM read-back model — unimplemented in both

`ds:2391-2401`, verbatim:

> "To read a CAM entry, the user first **places the SONIC in software reset**
> (set the RST bit in the Command register), programs the **CEP** register to
> select one of sixteen CAM entries, then reads **CAP2, CAP1, and CAP0** to
> obtain the complete 48-bit entry. The user can not write to the CAM entries
> directly. Instead, the user programs the CAM descriptor area in system
> memory... then issues the Load CAM command... This causes the SONIC to read
> the descriptors from memory and loads the corresponding CAM entry through
> CAP2-0."

Reinforced by TABLE 4-1 Note 1 (`ds:1525-1526`): CAP2/1/0 "can only be read
when the SONIC is in reset mode... The SONIC gives **invalid data** when these
registers are read in non-reset mode."

- **MAME:** CAP0/1/2 have write mask `0x0000` and are never assigned. They read
  back as 0 forever.
- **RTL:** identical — mask `0x0000` (sonic.v:214-216), never written by the
  engine. The CAM itself lives in `cam_table[0:15]` inside the RX engine
  (rx.sv:93) and is only observable through a **debug** mux
  (`dbg_cam_index` → `dbg_cam_entry`, rx.sv:53-59, 206), which is a JTAG tap,
  not the architectural port.

The RTL's own comment at rx.sv:56-58 already records this: *"The chip's own
readback path (write CEP, read CAP0/1/2) is not modelled here or in MAME."*

**Ranking note:** MAME runs the real Macintosh driver successfully with
CAP0/1/2 permanently zero. That is strong empirical evidence the Apple driver
never reads the CAM back. See **D-9** — real, but low-priority.

### 7.3 CAM address byte/word order — the documented-vs-emulated split

**What the datasheet says**, `ds:2381-2389` verbatim:

> "The CAP2 register is used to access the upper bits (`<47:32>`), CAP1 the
> middle bits (`<31:16>`) and CAP0 the lower bits (`<15:0>`) of the CAM entry.
> Given the physical address 60:50:40:30:20:10, which is made up of 6 octets or
> bytes, where 10h is the least significant byte and 60h is the most
> significant byte (**60h would be the first byte received from the network**
> and 10h would be the last), **CAP0 would be loaded with 2010h, CAP1 with
> 4030h and CAP2 with 6050h.**"

So per the datasheet, for a MAC that appears on the wire as `60 50 40 30 20 10`:

| Port | Value | Contains wire bytes |
|---|---|---|
| CAP2 | `0x6050` | 0 and 1 (**first** on the wire) |
| CAP1 | `0x4030` | 2 and 3 |
| CAP0 | `0x2010` | 4 and 5 (**last** on the wire) |

**What MAME does** (mame.cpp:496-509), reading CDA fields in ascending address
order and calling them `cep, cap0, cap1, cap2`:

```
m_cam[cep] = (u64(swapendian_int16(cap0)) << 32)
           | (u64(swapendian_int16(cap1)) << 16)
           | (u64(swapendian_int16(cap2)) << 0);
```

compared against `get_u48be(buf)` (mame.cpp:543) — a big-endian read of the
first six wire bytes, so bits `<47:40>` are wire byte 0. Therefore MAME places
**wire byte 0 in the low byte of the first CDA address word**, i.e. the exact
mirror of the documented layout on both axes (port order *and* intra-word byte
order). mame.cpp:501 marks this precisely:

```
// FIXME: documented byte/word order doesn't match emulation
```

**What the RTL does** (rx.sv:164-174):

```
rsp_cam_cap0 = desc_word(rsp,1,wide);   // first CDA word after the entry pointer
...
rsp_cam_mac = {cap0[7:0],cap0[15:8], cap1[7:0],cap1[15:8], cap2[7:0],cap2[15:8]};
```

and `dest_mac[47:40]` is loaded with the **first** byte off the wire
(rx.sv:348), descending thereafter (rx.sv:363). So `rsp_cam_mac[47:40] =
cap0[7:0]` — **bit-for-bit identical to MAME.**

| | Wire byte 0 comes from |
|---|---|
| **DS** `ds:2381-2389` | CAP2 high byte (last CDA word, high half) |
| **MAME** mame.cpp:506-509, 543 | first CDA word, **low** half |
| **RTL** rx.sv:172-174, 348 | first CDA word, **low** half |

**Verdict: DS and MAME disagree; the RTL follows MAME, and that is correct.**
MAME's ordering is the one that demonstrably matches the real Apple driver's
CDA layout. Recorded as **D-8 — deliberate**.

One caveat carried from the source extraction: the CDA *field order* itself
(whether the three address words appear as cap0,cap1,cap2 or cap2,cap1,cap0 in
memory) lived only in **Figure 4-2**, which is a bitmap (art tag
`TL/F/10492 – 22`, `ds:1369`) and did not survive `pdftotext`. The prose
(`ds:1345-1350`) says only "the remaining fields are for the three CAM Address
Ports". So the datasheet text cannot independently arbitrate; the ports-vs-wire
mapping quoted above is the only verbatim ordering statement available.

### 7.4 LCAM sequence and the CDP end state

| Step | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Per descriptor: 4 fields (entry ptr, 3 address words), 4 memory accesses | `ds:1343-1347`, `ds:2967-2972` | mame.cpp:496-499 | rx.sv:256-257 (`word_bytes*4` at `{urra, cam_cdp}`) | **Match** |
| CDA addressed by **URRA**:CDP, same 64k page as the RRA | `ds:1339-1341, 1360-1364` | `EA(m_reg[URRA], m_reg[CDP])` mame.cpp:494 | `{urra, cam_cdp}` rx.sv:256 | **Match** |
| CEP taken from the descriptor's first field, low 4 bits | `ds:1344-1347`, `ds:2371-2375` | `read_bus_word(cdp) & 0xf` mame.cpp:496 | `rsp_cam_pointer[3:0]` rx.sv:163 | **Match** |
| CDC decrements per descriptor; LCAM ends at zero | `ds:2376-2389` | `m_reg[CDC]--` mame.cpp:512 | `cam_count` rx.sv:491-494 | **Match** |
| CDP advances 4 words / 4 long words per descriptor | `ds:2371-2373` | `m_reg[CDP] += 4*width` mame.cpp:511 | `cam_cdp <= cam_cdp + 4*word_bytes` rx.sv:490 | **Match** |
| CE read from the field **after** the last descriptor | `ds:1351-1353` | `read_bus_word(EA(URRA, CDP))` mame.cpp:516 | rx.sv:259-261, 502-503 | **Match** |
| **CDP end state** | **"the CDP register points to the next location after the CAM enable field"** `ds:1357-1359`; **"After the Load Command completes, this register points to the next location after the CAM Descriptor Area"** `ds:2372-2375` | leaves CDP **at** the enable field — mame.cpp:511 never runs again after the loop, and 516 does not advance | leaves CDP **at** the enable field — `done_cdp <= cam_cdp` rx.sv:505 | **D-7 — DS says PAST, both say AT** |
| CDC == 0 at completion | `ds:1358-1359` | `while(m_reg[CDC])` exits at 0 | `done_cdc <= 0` rx.sv:505 | **Match** |
| LCAM self-clears, ISR.LCD set | `ds:1358-1359` | mame.cpp:519-520 | rx.sv:506 → sonic.v:590-597 | **Match** |
| CAM DMA defers until any transmit/receive in progress finishes | `ds:1355-1356` | implicit (synchronous) | cfg accepted only in `S_IDLE`/`S_READY` rx.sv:190 | **Match** |
| LCAM + TXP simultaneously → **"The SONIC will lock up"** | `ds:1609`, `ds:1654-1655` | not modelled | not modelled | **D-25** — both are more forgiving than silicon (benign) |

**Confirmation of known finding #6.** The datasheet states the CDP end position
**twice, independently, in two different sections**, and both say *past* the
enable field. MAME and the RTL both leave it *at* the field. The RTL matches
MAME. This only matters for a driver that reuses CDP without reloading it — and
`ds:1348-1350` explicitly warns "This register must be reloaded each time a new
Load CAM command is issued", so a conforming driver always rewrites CDP. **Real
divergence, low impact, deliberate (matches MAME).**

---

## 8. Tally counters, timer, revision

### 8.1 Tally counters — CRCT (2C), FAET (2D), MPT (2E)

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Write is **inverted** | "The data written to these registers is inverted before being latched... if a value of FFFFh is written... they will contain and read back the value 0000h. **Data is not inverted during a read operation.** The Tally registers, therefore, are cleared by writing all '1's'" `ds:2399-2406`, `ds:1530-1531` | `m_reg[offset] = ~data` mame.cpp:287-292 | `sonic_reg[idx] <= ~data` sonic.v:505-506 (and byte form sonic.v:414-419) | **Match, all three** |
| Read is straight | `ds:2401-2402` | plain array read | plain array read | **Match** |
| Incremented by | CRCT: CRC error on an accepted packet, **not** if it also had a frame-alignment error `ds:2407-2413`. FAET: FAE on an accepted packet `ds:2414-2418`. MPT: no resources / FIFO overrun / valid packet with receiver disabled `ds:2420-2423` | `// TODO: tally counters` mame.cpp:17 | never incremented | **D-14** (shared) |
| Rollover → ISR CRC/FAE/MP | `ds:2193-2198` | not implemented | not implemented | **D-14** |
| Reset | **"A software or hardware reset does not clear the tally counters"** `ds:2404-2406`; halted while RST set `ds:2397-2398` | `device_reset()` leaves them | **zeroed by hardware reset** (default arm of `sonic_reset_value`, sonic.v:171) | **D-16** — moot while D-14 stands |

### 8.2 General-purpose watchdog timer — WT0 (29), WT1 (2A)

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| 32-bit down-counter, `WT1:WT0`, clocked at TXC/2 | `ds:2435-2454` | `// TODO: watchdog timers` mame.cpp:18 | masks `0xFFFF` only (sonic.v:221-222); **no counter** | **D-13** (shared) |
| Controlled by CR.ST / CR.STP | `ds:1620-1625` | CR bits toggle, no timer | CR bits toggle correctly (sonic.v:470-471), no timer | **D-13** |
| Rollover `0000_0000h → FFFF_FFFFh` → ISR.TC (bit 7) | `ds:2174` | not set | not set | **D-13** |
| Reset | **"A hardware or software reset halts, but does not clear, the General Purpose timer"** `ds:2451` | n/a | registers zeroed on HW reset | **D-11/D-16** — moot |

### 8.3 Silicon Revision — SR (RA `0x28`)

| Property | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Read-only | `ds:1524`, `ds:2458` | mask `0x0000` mame.cpp:63 | mask `0x0000` sonic.v:220 | **Match** |
| Value | **"The value of the DP83932CVF revision register is 6h"** `ds:2458-2462` | `m_reg[SR] = 6` mame.cpp:93 | `0x0006` sonic.v:170 | **Match, all three** |
| Reset | absent from TABLE 5-4 | set in `device_start` only, survives `device_reset` | re-applied by `sonic_reset_value` sonic.v:170 | **Match** in effect |

Regression-locked at `tb/tb_q700_eth_sonic.cpp:217`.

---

## 9. Descriptor formats

### 9.1 32-bit mode word placement

The datasheet states this four times, identically, once per structure:

| Structure | Statement | Cite |
|---|---|---|
| RRA | "in 32-bit mode the upper word (`D<31:16>`) is not used by the SONIC. **This area may be used for other purposes since the SONIC never writes into the RRA.**" | `ds:793-796` |
| RDA | "In 32-bit mode, the upper word, `D<31:16>`, is not used. **This unused area in memory should not be used for other purposes, since the SONIC may still write into these locations.**" | `ds:904-907` |
| TDA | "in 32-bit mode the upper word, `D<31:16>`, is not used" | `ds:1176-1177` |
| CDA | "In 32-bit mode the upper word, `D<31:16>`, is not used" | `ds:1343-1344` |

So **every 16-bit descriptor field occupies one 32-bit slot, value on
`D<15:0>`, upper half unused.** On the Q700's big-endian (BMODE=1) bus, byte 0
of a long word sits on `D<31:24>` (`ds:353-357`), so `D<15:0>` is the
**higher-addressed half** — the field lands at byte offset **+2** within each
4-byte slot.

> **Caveat, carried honestly:** the datasheet's byte-ordering diagrams
> (`ds:322-357`) are scoped explicitly and only to the RBA and TBA — "the byte
> orientation for received and transmitted data **in the RBA and TBA**". No
> sentence ties descriptor-field placement to BMODE. The `+2` conclusion is a
> **derivation** from the two facts above, not a quotation. It does match MAME
> and the real driver.

| | Implementation | Cite |
|---|---|---|
| **MAME** | `read_dword`/`write_dword` on a big-endian address space; field stride `width = (DCR & DCR_DW) ? 4 : 2` | mame.cpp:361, 460, 490, 580-591 |
| **RTL TX** | `byte_index = word_index*(wide?4:2) + (wide?2:0)` — **offset +2 in wide mode** | tx.sv:118 |
| **RTL RX** | same, read (`desc_word`) and write (`pack_word`) | rx.sv:111, 119 |
| **Verdict** | **Match, all three.** | |

Alignment requirements (`ds:782-787`): descriptor fields on word boundaries
(A0=0) in 16-bit mode, long-word boundaries (A1,A0=0,0) in 32-bit mode; the RBA
likewise; **TBA fragments "may be aligned on any arbitrary byte boundary"** —
which the RTL honours (`fragment_addr` is used unmasked, tx.sv:180).

### 9.2 TDA — `TXpkt`

Field order confirmed by the DMA access counts (`ds:1256-1261`: "6, 3, or 2
accesses") and the register load list (`ds:1217-1224`).

| Idx | Field | Written by | RTL | MAME |
|---|---|---|---|---|
| 0 | `TXpkt.status` | **SONIC**, at end of transmission `ds:1138-1140, 1235-1238` | tx.sv:186-196 (`descriptor_addr + 0`) | mame.cpp:431 |
| 1 | `TXpkt.config` | SW → TCR bits 15–12 `ds:1141-1145` | tx.sv:250 (`& 0xF000`) | mame.cpp:369 (`& TCR_TPC`) |
| 2 | `TXpkt.pkt_size` | SW → TPS `ds:1146-1147` | tx.sv:251 | mame.cpp:370 |
| 3 | `TXpkt.frag_count` | SW → TFC `ds:1148-1150` | tx.sv:252-253 | mame.cpp:371 |
| 4+3n | `TXpkt.frag_ptr0` | SW → TSA0; **any byte alignment** `ds:1151-1156` | tx.sv:264-265 | mame.cpp:387 |
| 5+3n | `TXpkt.frag_ptr1` | SW → TSA1 | tx.sv:264 | mame.cpp:388 |
| 6+3n | `TXpkt.frag_size` | SW → TFS; **min 1 byte** `ds:1157-1159` | tx.sv:266 | mame.cpp:389 |
| last | `TXpkt.link` | SW; **bit 0 = EOL** `ds:1160-1163` | tx.sv:198, 367-373 | mame.cpp:437-440 |

`TXpkt.status` content (`ds:1230-1238`, Figure 3-14): `{NC4..NC0, TCR[10:0]}`
— collision count in bits 15–11, TCR status in 10–0. RTL writes a constant
`0x0001` (tx.sv:189-192) = zero collisions + PTX. Correct for a clean transmit;
no error case can be produced (D-15).

`TXpkt.config` (`ds:1212-1221`, Figure 3-13): only bits 15–12 defined, 11–0
"don't care" — matching the `0xF000` mask everywhere.

Dynamic append rule, `ds:1288-1291`: "**The last TXpkt.link field must point to
the next location where a descriptor will be added.**" This is what MAME's
TXP-re-issue suppression (mame.cpp:256-262) and the RTL's equivalent
(tx.sv/sonic.v:481) exist to support.

### 9.3 RDA — `RXpkt`

> **Datasheet self-contradiction.** `ds:900` says the SONIC writes **6 words**
> of status; `ds:1041` says it "first writes **5 words**"; `ds:1061-1062` and
> `ds:2969` both give the descriptor as **7 words total**. Only the 5-word
> reading is self-consistent (5 written + 1 link read + 1 in_use written = 7).
> **Both MAME and the RTL implement 5.**

| Idx | Field | Written by | DS | RTL | MAME |
|---|---|---|---|---|---|
| 0 | `RXpkt.status` | **SONIC** — the whole RCR, config bits included | `ds:924-930` | `status_rcr` rx.sv:237, 393-396 | mame.cpp:188 |
| 1 | `RXpkt.byte_count` | **SONIC** — "from the start of Destination Address to the **end of FCS**" | `ds:941-942` | `rda_len` rx.sv:238 (includes the 4 re-appended FCS bytes, rx.sv:377-382) | mame.cpp:189 |
| 2 | `RXpkt.pkt_ptr0` | **SONIC** — CRBA0 at packet start | `ds:944-947` | `trba[15:0]` rx.sv:238 | mame.cpp:190 |
| 3 | `RXpkt.pkt_ptr1` | **SONIC** — CRBA1 | `ds:944-947` | `trba[31:16]` rx.sv:239 | mame.cpp:191 |
| 4 | `RXpkt.seq_no` | **SONIC** — `[15:8]` RBA seq, `[7:0]` packet seq | `ds:948-950`, `ds:899-903` | `rsc` rx.sv:239 | mame.cpp:192 |
| 5 | `RXpkt.link` | **System**; bit 0 = EOL | `ds:905-908` | read at `crda + 5*word_bytes` rx.sv:243 | mame.cpp:194 |
| 6 | `RXpkt.in_use` | **Both** — see below | `ds:909-917` | written at `llfa + word_bytes` = `crda + 6*word_bytes` rx.sv:247 | mame.cpp:203 |

**`in_use` handshake**, `ds:909-917` verbatim: *"When the system avails a
descriptor to the SONIC, it writes a non-zero value into this field. The SONIC,
in turn, sets this field to all '0's' when it has finished processing the
descriptor... If, however, the SONIC has reached the last descriptor in the
list, it maintains ownership of the descriptor until the system has appended
additional descriptors."* The written value is **`0x0000`** (`ds:1134-1135`).

| Rule | DS | MAME | RTL | Verdict |
|---|---|---|---|---|
| Writes `0x0000` to `in_use` when done | `ds:1134-1135` | mame.cpp:203 | rx.sv:246-250 (`dma_req_wdata` defaults to 0, rx.sv:223) | **Match** |
| On EOL=1: **does not** write `in_use`, sets ISR.RDE | `ds:1041-1062` | mame.cpp:197-203 | rx.sv:447-454 (skips `S_CLEAR_REQ`) | **Match** |
| LLFA = address of the current link field | — (`ds:1554` gives LLFA no prose) | `CRDA + 5*width` mame.cpp:193 | `crda + 5*word_bytes` rx.sv:440 | **Match** |
| At next reception, re-read the same link field once to see if the system reopened the ring | `ds:1066-1075`, `ds:2985-2990` | mame.cpp:120-126 | `S_EOL_RETRY`/`S_EOL_WAIT` rx.sv:386-389, 458-469 | **Match** |
| Runt/error frame → restore CRBA/RBWC from TRBA/TBWC, no RDA write | `ds:1096-1102` | mame.cpp:134-144 (returns before any write) | rx.sv:384-385 returns to `S_READY`; `crba`/`rbwc` only advance at rx.sv:435 | **Match** |

`RXpkt.status` layout (`ds:932-940`, Figure 3-6) is the RCR bit layout verbatim
— confirming that config bits 15–9 travel into the descriptor.

### 9.4 RRA — `RXrsrc`

Field order, `ds:980-987` verbatim: *"four fields: (1) RXrsrc.buff_ptr0,
(2) RXrsrc.buff_ptr1, (3) RXrsrc.buff_wc0, and (4) RXrsrc.buff_wc1... The '0'
and '1' in the descriptors denote the least and most significant portions."*

| Idx | Field | → register | RTL | MAME |
|---|---|---|---|---|
| 0 | `RXrsrc.buff_ptr0` | CRBA0 | rx.sv:327, 331 | mame.cpp:464 |
| 1 | `RXrsrc.buff_ptr1` | CRBA1 | rx.sv:327, 332 | mame.cpp:465 |
| 2 | `RXrsrc.buff_wc0` | RBWC0 | rx.sv:328, 333 | mame.cpp:466 |
| 3 | `RXrsrc.buff_wc1` | RBWC1 | rx.sv:328, 334 | mame.cpp:467 |

- Word count is **in 16-bit words**, not bytes (`ds:782-790`). RTL: `rbwc_after
  = rbwc - ((rda_len+1)>>1)` (rx.sv:141) — **Match** (mame.cpp:176).
- **"The SONIC never writes into the RRA"** (`ds:795-796`) — RTL only ever
  issues reads at `{urra, rrp}` (rx.sv:226-229). **Match.**
- Circular queue delimited by RSA/REA, advanced by RRP, replenished at RWP
  (`ds:797-818`). REA is one-past-the-last descriptor (`ds:952-955`). RTL wrap:
  `rrp+4*word_bytes == rea ? rsa : rrp+4*word_bytes` (rx.sv:329) — **Match**
  (mame.cpp:473-477).
- LPKT: set when RBWC ≤ EOBC (`ds:1871-1874`). RTL: `(rbwc_after < eobc)`
  (rx.sv:396); MAME: `(rbwc < m_reg[EOBC])` (mame.cpp:181). Both use strict
  `<` where the datasheet says "less than or equal to" — a **shared one-count
  edge difference**, folded into D-23.

### 9.5 CDA

Per-descriptor: 4 × 16-bit fields — entry pointer then three CAM address words
— repeated CDC times, followed by **one trailing field holding the CAM-enable
mask** (`ds:1343-1353`). Addressed by URRA:CDP. See §7.3 for the byte/word
order split and §7.4 for the CDP end state.

---
## 10. Divergence register

**27 divergences catalogued.** Ranked by whether the Macintosh driver would
plausibly hit them. Every row carries three citations.

Legend — **Status**: `FIXED` (already corrected, verified here against the
datasheet) · `OPEN` (real gap, not yet addressed) · `DELIBERATE` (we knowingly
follow MAME against the datasheet, or knowingly simplify) · `SHARED` (MAME has
the same gap; matching it is safe).

### 10.0 Confirmation of the five previously-known items

| # | Item | Does the datasheet agree with the fix? |
|---|---|---|
| **K1** | Software reset must touch **only CR**; IMR/ISR/CE/RSC/DCR2 are power-on-only | **YES, emphatically.** `ds:1584-1585` gives the CR software-reset mask bit-for-bit ("bits 9, 8, 1, and 0 are cleared and bits 7 and 2 are set... **all others are unaffected**") and `sonic.v:281-283` computes exactly that. Reinforced per-register at `ds:2107` (ISR), `ds:1905` (TCR), `ds:1800` (RCR), `ds:1670` (DCR), `ds:2214` (DCR2), `ds:3385/3393/3395` (IMR/RSC/CE), and globally at `ds:3459-3464` ("only the Command Register is changed"). **Fix correct.** |
| **K2** | TXP must clear HTX **and** HTX must clear TXP | **HALF.** HTX→TXP is explicit (`ds:1656-1657` "TXP is reset after transmission has halted"). **TXP→HTX is stated nowhere in the datasheet** — that direction follows MAME (mame.cpp:316) only. Defensible via the blanket self-clear rule `ds:1577`, but record it as a choice. See **D-26**. Also note the narrow delta at §3.3: sonic.v:479 clears HTX even when the TXP command is suppressed; MAME would not. |
| **K3** | CPU write to CR racing an engine writeback | Not a datasheet topic. Fix present and correct (`cpu_wr_cr` sonic.v:267, `cr_engine_clear` sonic.v:268-272, applied at sonic.v:318/355/494 and deferred at sonic.v:582/594/600). **The protection does not extend to any other register — see D-19.** |
| **K4** | Software reset must cancel in-flight **TX** work as well as RX | **YES** — `ds:3459-3462`: "A software reset **immediately terminates DMA operations**... the SONIC will not be active in any other way." `sonic_enter_reset` clears `tx_cmd_valid` (sonic.v:308) and the RX bookkeeping (sonic.v:300-303). **But the fix is incomplete:** it cancels *queued* work only. A TX engine already running keeps walking the list (no reset/abort port on `q700_sonic_tx`), and the RX datapath stays live because `rx_enabled` ignores CR_RST. See **D-2** and **D-5** — these are the remaining halves of K4. |
| **K5** | TCR_CRCI entirely unimplemented in the TX path | **Confirmed OPEN.** Full documentation at §4.3. `ds:1942-1944` is the entire specification: "1: transmit packet without 4-byte FCS field." Byte-count treatment is **not stated anywhere in the datasheet**; the bridging use case (`ds:337-345`) implies the driver's FCS sits inside the fragment data and is counted in TFS/TPS, which is what MAME implements (mame.cpp:395-407). See **D-6**. |
| **K6** | After LCAM, CDP should point PAST the CAM-enable field | **Confirmed — the datasheet says PAST, twice, independently:** `ds:1357-1359` ("the CDP register points to the next location after the CAM enable field") and `ds:2372-2375` ("this register points to the next location after the CAM Descriptor Area"). MAME (mame.cpp:511-516) and the RTL (rx.sv:490, 505) both leave it **at** the field. See **D-7**. |

### 10.1 Ranked — most likely to be hit first

| # | Rank | Divergence | DS | MAME | RTL | Status |
|---|---|---|---|---|---|---|
| **D-1** | **1** | **`ISR_TXER` is encoded `0x0400` — that is `PKTRX`. TXER is bit 8 (`0x0100`).** A transmit error raises the *packet received* interrupt. The register file and its own RX engine disagree about bit 10: `sonic.v:122` calls it TXER, `rx.sv:70` calls it PKTRX. | bit 10 = PKTRX, bit 8 = TXER `ds:2109-2110` | `ISR_TXER = 0x0100`, `ISR_PKTRX = 0x0400` mame.h:191,193 | `ISR_TXER = 16'h0400` **sonic.v:122**, consumed at sonic.v:255 | **OPEN — NEW** |
| **D-2** | **2** | **HTX does not halt the transmit engine.** Writing HTX clears `CR.TXP` in the register file, but `q700_sonic_tx.sv` has **no halt/abort port at all** — the engine keeps fetching descriptors, DMAing fragments and transmitting to end-of-list. CTDA is also reloaded, which the datasheet forbids after HTX. | "halts the transmit command after the current transmission has completed... The SONIC samples this bit after writing to the TXpkt.status field" `ds:1656-1659`; "If the halt transmit command is issued... the CTDA register is **not** loaded" `ds:1224-1227` | implemented — `if (!(m_reg[CR] & CR_HTX))` gates both the CTDA reload and the next-packet chain mame.cpp:434-455 | CR bit only: sonic.v:467. No port on tx.sv (grep for halt/htx/abort returns nothing) | **OPEN — NEW** |
| **D-3** | **3** | **A software reset does not gate the RX datapath.** `rx_enabled` is derived from `CR_RXEN` alone; `CR_RST` is never consulted. Because software reset leaves RXEN untouched (correctly, per `ds:1585`), a reset issued while the receiver was enabled leaves the RX engine accepting frames and DMAing them against the stale CRBA/CRDA the driver has already disowned. This is the remaining half of K4 and the same failure class as the "65535+ RX DMA ops at a garbage CRDA" symptom the earlier fix chased. | "A software reset **immediately terminates DMA operations**... the SONIC will not be active in any other way" `ds:3459-3462` | same hole in principle — `recv_start_cb` checks only `CR_RXEN` mame.cpp:116 — but MAME's writes are harmless emulation-side | `assign rx_enabled = (sonic_reg[REG_CR] & CR_RXEN) != 0;` **sonic.v:247**; consumed as the only gate at rx.sv:198-199 | **OPEN — NEW** |
| **D-4** | **4** | **Loopback (RCR LB1/LB0) is unimplemented.** No loopback datapath exists; only the `LBK` *status* bit is synthesised (rx.sv:394). If the Apple driver runs a loopback self-test at open, it passes under MAME and fails here. **Worth checking against the actual driver before ranking lower.** | four modes: none / MAC / ENDEC / Transceiver `ds:1857-1866`; "For proper loopback operation, the CAM Address registers and Receive Control register must be initialized to accept the Destination address of the loopback packet" | `set_loopback(bool(m_reg[RCR] & RCR_LB))` mame.cpp:270 — routed through MAME's network layer | `RCR_LB` declared rx.sv:65, used only to fake `RCR_LBK` at rx.sv:394 | **OPEN — NEW** |
| **D-5** | **5** | **TCR loses its configuration half after every transmit.** `done_tcr = (descriptor_tcr \| TCR_PTX) & 16'h07ff` — but `descriptor_tcr` is masked to `0xF000` (tx.sv:250), so the `& 0x07FF` erases it. TCR reads back `0x0001` forever; PINT/POWC/CRCI/EXDIS never appear. | "bits 15, 14, 13 and 12 from the TXpkt.config field are loaded into the TCR" `ds:1899-1900`, `ds:1217`; TCR bits 15–12 are r/w `ds:1907-1909` | `m_reg[TCR] = read_bus_word(...) & TCR_TPC` mame.cpp:369, `\|= TCR_PTX` mame.cpp:427 — config half preserved | **tx.sv:349** → sonic.v:585 | **OPEN — NEW** |
| **D-6** | **6** | **PINT fires on every descriptor that requests it, with no alternation gate**, and fires at *end* of transmission rather than on TDA read. Root cause is shared with D-5: with the TCR config half always zero, no edge can ever be detected. A driver that leaves PINT set in consecutive TDAs gets an interrupt per packet instead of one. | "issue an interrupt immediately **after reading a TDA**"; "**PINT in the Transmit Control Register must be cleared before it is set again** in order to have the interrupt issued for another packet" `ds:1930-1936` | edge-detected: `if ((m_reg[TCR] & TCR_PINT) && !(tcr & TCR_PINT))` mame.cpp:376 | `done_pint <= descriptor_tcr[15] && !failed` **tx.sv:380** | **OPEN — NEW** |
| **D-7** | **7** | **`RCR_ERR` is ignored; `CRCR`/`FAER` are never reported.** Any frame the MAC flags bad (`rx_axis_tuser`) is dropped unconditionally, even with ERR set. | "1: Accept packets with CRC errors and ignore collisions" `ds:1827-1829`; CRCR/FAER status `ds:1879-1885` | `if (m_reg[RCR] & RCR_ERR) m_reg[RCR] \|= RCR_CRCR; else return -1;` mame.cpp:140-144 | `if(frame_bad \|\| ...)` drops **rx.sv:384**; ERR never tested | **OPEN — NEW** |
| **D-8** | **8** | **`TCR_CRCI` (0x2000) unimplemented — the MAC always appends an FCS.** The bit is captured into `descriptor_tcr` (tx.sv:250) but nothing consumes it. Note the fix is not local to the descriptor parser: the FCS is appended by the downstream Taxi MAC, so honouring CRCI needs a **new port** out of `q700_sonic_tx`. A Mac networking driver is unlikely to set CRCI (it is a bridging/forwarding feature, `ds:337-345`), which is why this ranks below the items above despite being a whole missing feature. | `ds:1942-1944` (complete text quoted at §4.3) | `if (!(m_reg[TCR] & TCR_CRCI))` gates the append mame.cpp:400-407 | no consumer; `tb/tb_q700_sonic_tx.cpp:120` sets CRCI in a TDA but asserts nothing about FCS | **OPEN — KNOWN (K5)** |
| **D-9** | **9** | **CAM read-back through CEP → CAP2/CAP1/CAP0 is unimplemented** — the ports read `0x0000` forever. The CAM is reachable only via a JTAG debug mux (rx.sv:53-59, 206). | full procedure at `ds:2391-2401`; validity restricted to reset mode `ds:1525-1526` | identical gap — mask `0x0000`, never assigned mame.cpp:62 | mask `0x0000` **sonic.v:214-216** | **OPEN — NEW, but SHARED.** MAME runs the real driver with these permanently zero, which is strong evidence the Apple driver never reads the CAM back. |
| **D-10** | **10** | **Hardware reset over-clears registers the datasheet declares unaffected.** `sonic_reset_value()` zeroes all 64 entries. The datasheet says HW reset touches **only** DCR bits 15/13, RCR bits LB1/LB0/BRD(/RNT), CR, IMR, ISR, TCR, EOBC, CE, RSC — and explicitly that UTDA, CTDA, URDA, CRDA, URRA, RSA, REA, RRP, RWP, CEP, CAP, CDP, CDC and the tally counters are "unaffected by a hardware or software reset". | `ds:1669-1670` (DCR), `ds:1833/1838/1861`+`ds:3401` (RCR), `ds:2292-2293, 2312-2313, 2328-2329, 2341-2342, 2316-2318` (pointers), `ds:2359-2363` (CAM), `ds:2404-2406` (tally) | matches the datasheet: `DCR &= ~(EXBUS\|LBR)` mame.cpp:99, `RCR &= ~(RNT\|BRD\|LB)` mame.cpp:100, everything else untouched by `device_reset` | `for (i=0;i<64;i=i+1) sonic_reg[i] <= sonic_reset_value(i)` **sonic.v:520-521** | **OPEN — NEW.** Invisible on a cold boot (all-zero either way); observable only on a warm hardware reset, after which the driver reprograms anyway. |
| **D-11** | **11** | **`BCM` (byte count mismatch) is never checked.** `TXpkt.pkt_size` is read into TPS but never compared against the sum of `TXpkt.frag_size`. A malformed TDA transmits silently instead of aborting. | "set when the SONIC detects that the TXpkt.pkt_size field is not equal to the sum of the TXpkt.frag_size field(s). **Transmission is aborted**" `ds:1996-1999`; also clears CR.TXP `ds:1651-1653` and raises ISR.TXER `ds:2166-2172` | `// byte count mismatch` TODO mame.cpp:14 | TPS stored (tx.sv:251), never compared | **OPEN — SHARED** |
| **D-12** | 12 | **CDP is left pointing AT the CAM-enable field**, not past it. | "next location **after** the CAM enable field" `ds:1357-1359`; "next location after the CAM Descriptor Area" `ds:2372-2375` | leaves it at the field mame.cpp:511-516 | `done_cdp <= cam_cdp` **rx.sv:505** | **DELIBERATE (matches MAME).** Harmless for a conforming driver: `ds:1348-1350` requires CDP be reloaded before every LCAM. **KNOWN (K6)** |
| **D-13** | 13 | **CDA MAC byte/word order is the mirror of the documented layout** — wire byte 0 comes from the low half of the *first* CDA address word, where the datasheet puts it in the high half of CAP2 (the *last*). | `ds:2381-2389` (full quote at §7.3) | same as RTL; explicitly flagged `// FIXME: documented byte/word order doesn't match emulation` mame.cpp:501 | `rsp_cam_mac` **rx.sv:172-174** vs `dest_mac` **rx.sv:348** | **DELIBERATE (matches MAME).** MAME's order is the one that works with the real Apple driver. **Do not "fix" toward the datasheet.** |
| **D-14** | 14 | **`TXP` clearing `HTX` is not in the datasheet.** Only HTX→TXP is documented. | `ds:1656-1657` documents HTX→TXP only; nothing documents TXP→HTX | mame.cpp:316 | sonic.v:479 | **DELIBERATE (matches MAME).** Part of **K2**. Sub-item: sonic.v:479 clears HTX even when the TXP command is suppressed by an in-flight transmit; MAME does not (mame.cpp:261 nulls TXP before `command()` runs). |
| **D-15** | 15 | **Clearing `ISR.RDE` triggers an explicit descriptor reload.** The datasheet documents a clear-side-effect only for RBE. | RBE side effect at `ds:2185-2187`; **no** RDE equivalent — recovery is the lazy link re-read `ds:1066-1075` | no RDE side effect; lazy CRDA reload from LLFA at reception mame.cpp:120-126 | `rx_recovery_pending[1]` sonic.v:499-501 → rx.sv:316-317, **plus** the lazy `S_EOL_RETRY` path rx.sv:458-469 | **DELIBERATE superset.** Both mechanisms present; strictly more forgiving. rx.sv:191-197 documents why. |
| **D-16** | 16 | **DCR, DCR2 and CE are writable outside reset mode.** | DCR/DCR2: "Writing to these registers while not in reset mode **does not alter the registers**" `ds:1528-1529`, `ds:1670-1671`, `ds:2214-2215`. CE: "can only be written to when the SONIC is in reset mode" `ds:1527` | `// TODO: can only write during reset: DCR, DCR2` mame.cpp:228; CE mask `0xffff` mame.cpp:62 | `default` write arm sonic.v:507-511; CE mask `0xFFFF` sonic.v:217 | **SHARED.** Benign — drivers program these while in reset anyway, per `ds:1669-1670`. |
| **D-17** | 17 | **CR byte-write to the high half bypasses the RST gate.** A single-byte write to offset `+2` reaches sonic.v:374-390, which has no `CR_RST` check (the 16-bit path at sonic.v:460 does), so LCAM/RRRA can be set while the chip is in software reset. The command issue path is separately gated (sonic.v:546), so the bit merely sits pending until reset exits. | "Before any commands can be issued, the RST bit must first be reset to '0'... **two writes** are required" `ds:1578-1580` | all CR writes gated mame.cpp:233 | **sonic.v:374-390** | **OPEN — NEW.** Low: the Q700 driver writes CR as a 16-bit word. |
| **D-18** | 18 | **A CPU write to a non-CR register loses to an engine writeback in the same cycle.** Only CR (sonic.v:582/594/600) and RCR (sonic.v:610) are protected; CRDA, CRBA0/1, RBWC0/1, RRP, RSC, LLFA, TRBA0/1, TBWC0/1, TCR, TPS, TFC and CTDA are assigned later in the same `always` block and win outright. | n/a (RTL correctness) | n/a (MAME is not cycle-accurate) | sonic.v:581-620 vs sonic.v:532-533 | **OPEN — NEW.** Low in practice: **RWP and URDA/URRA — the registers a driver touches at runtime — are never written back by the engine**, and the rest move only while the receiver is stopped. Same bug class as K3, one register wider. |
| **D-19** | 19 | **`TSA0`/`TSA1`/`TFS` are never updated during transmit.** | "TSA0 ← TXpkt.frag_ptr0, TSA1 ← TXpkt.frag_ptr1, TFS ← TXpkt.frag_size" `ds:1220-1223` | updates all three per fragment mame.cpp:387-389 | `done_*` carries only ctda/tcr/tps/tfc (tx.sv:53-56) | **OPEN — NEW.** TABLE 4-2 marks these "Internal Use... Users should not write to these registers" (`ds:1533`), so no driver should be reading them either. |
| **D-20** | 20 | **Watchdog timer unimplemented** — WT0/WT1 store but do not count; CR.ST/CR.STP toggle correctly but drive nothing; ISR.TC (bit 7) is never set. | `ds:2435-2454`, `ds:1620-1625`, `ds:2174` | `// TODO: watchdog timers` mame.cpp:18 | masks only, sonic.v:221-222 | **SHARED** |
| **D-21** | 21 | **Tally counters never increment**; ISR CRC/FAE/MP rollover bits never set. The inverted-write behaviour *is* correct in all three. | `ds:2399-2423`, `ds:2193-2198` | `// TODO: tally counters` mame.cpp:17 | inverted write sonic.v:505-506; no counter | **SHARED** |
| **D-22** | 22 | **`ISR.RFO` (0), `ISR.HBL` (13), `ISR.BR` (14) are never set**, and clearing BR has no DMA-release side effect. | `ds:2199-2202`, `ds:2140-2142`, `ds:2134-2139` | not modelled | not modelled | **SHARED.** No FIFO / heartbeat / bus-retry model exists in this SoC. |
| **D-23** | 23 | **`LPKT` uses strict `<` where the datasheet says "less than or equal to"**, and **EOBC's LSB is not forced low in 32-bit mode**. Both are one-count edge effects on the last-packet-in-RBA flag. | "when its Remaining Buffer Word Count register is **less than or equal to** the End Of Buffer Count register" `ds:1871-1874`; "in 32-bit mode, the SONIC holds the LSB always low so that it properly compares with the RBWC0,1 registers" `ds:1019-1020` | `if (rbwc < m_reg[EOBC])` mame.cpp:181; EOBC mask `0xffff` | `(rbwc_after < eobc)` **rx.sv:396**; EOBC mask `0xFFFF` sonic.v:199 | **SHARED** |
| **D-24** | 24 | **Tally counters are cleared by hardware reset.** | "A software or hardware reset **does not clear** the tally counters" `ds:2404-2406` | `device_reset` leaves them | zeroed by the default arm of `sonic_reset_value` sonic.v:171 | **OPEN — NEW.** Moot while D-21 stands. Subsumed by D-10. |
| **D-25** | 25 | **`TPS` read-inversion and `TFS` write-shift are not modelled.** | TPS: "The data that is read from these registers is the inversion of what has been written" `ds:1558`. TFS: "shifted once in 16-bit mode and shifted twice in 32-bit mode" `ds:1559` | plain masked stores mame.cpp:294-300 | plain masked stores sonic.v:507-511 | **SHARED.** Both are TABLE 4-2 internal-use registers. |
| **D-26** | 26 | **`RXDIS` cannot be cleared by writing zero** when both RXEN and RXDIS are set. | "When both RXEN and RXDIS are set, RXDIS **could be cleared by writing zero to it**" `ds:1645-1646` — an explicit exception to the write-0-has-no-effect rule at `ds:1577` | not modelled | not modelled (CR is strictly set-only, sonic.v:466) | **SHARED** |
| **D-27** | 27 | **The LCAM+TXP mutual-lockup hazard is not modelled** — both implementations execute happily where silicon hangs. | "The SONIC will lock up if both bits are set simultaneously" `ds:1609`, `ds:1654-1655` | not modelled | not modelled | **SHARED, benign** — more forgiving than hardware, never less. |

### 10.2 Summary by status

| Status | Count | Numbers |
|---|---|---|
| **OPEN — new this audit** | **13** | D-1, D-2, D-3, D-4, D-5, D-6, D-7, D-9, D-10, D-17, D-18, D-19, D-24 *(D-9 is also SHARED — MAME has the same gap)* |
| **OPEN — previously known** | 1 | D-8 (CRCI, = K5) |
| **DELIBERATE** — follow MAME against the datasheet | 4 | D-12 (= K6), D-13, D-14 (= K2), D-15 |
| **SHARED with MAME** — safe to match | 9 | D-11, D-16, D-20, D-21, D-22, D-23, D-25, D-26, D-27 |
| **FIXED and datasheet-confirmed** | 4 | K1, K2 (half), K3, K4 (partially — see D-2, D-3) |

### 10.3 Suggested order of work

1. **D-1** — one-line constant. `ISR_TXER` must be `16'h0100`. Confirmed
   against all three sources; the RTL currently contradicts *itself*
   (sonic.v:122 vs rx.sv:70).
2. **D-2 + D-3 together** — they are one architectural gap: *chip-level control
   bits do not reach the DMA engines.* HTX needs an abort port on
   `q700_sonic_tx`; `rx_enabled` needs `&& !(CR & CR_RST)`; and a software
   reset should abort a running TX engine, not just cancel a queued command.
   This closes the remaining half of K4 and is the same failure class as the
   stale-pointer RX DMA storm already seen on hardware.
3. **D-5 + D-6 together** — preserve the TCR config half through transmit
   completion (`done_tcr` should be `descriptor_tcr | (status & 0x07FF)`), then
   PINT edge-detection falls out for free.
4. **D-4** — check the Apple driver for a loopback self-test before deciding
   how much of `RCR_LB` to build.
5. **D-7**, then **D-8 (CRCI)**, then the rest.

### 10.4 Non-divergences worth recording

Things that look wrong but are correct:

- **Interrupt polarity.** `sonic_irq` is active-high internally and inverted
  into VIA2 PA0 (`fpga_top_peripherals.vh:933`). `ds:2700` says INT is active
  high only when BMODE=0; the Q700 is BMODE=1, so active-low is right. MAME
  still lists this as an unfinished TODO (mame.cpp:13) — **the RTL is ahead of
  MAME here.**
- **`RXpkt.byte_count` includes the FCS.** `ds:941-942` says "from the start of
  Destination Address to the end of FCS". The Taxi MAC strips the FCS after
  validating it, so the RX engine regenerates and re-appends it (rx.sv:377-382)
  before counting. Correct, and the reason `rda_len` looks four bytes long.
- **`RBAE` handling.** rx.sv:390-391 raises `ISR.RBAE` and skips the RDA write
  on buffer overrun, exactly as `ds:2189-2192` requires. MAME has this as a
  `// TODO` (mame.cpp:185) — **RTL ahead of MAME.**
- **32-bit descriptor fields at byte offset +2.** tx.sv:118 / rx.sv:111,119.
  Correct for a big-endian bus (§9.1).
- **Write masks.** All 30 non-trivial masks in `sonic_reg_mask()`
  (sonic.v:176-232) match MAME's `regmask[]` (mame.cpp:55-66) exactly, and every
  one that the datasheet specifies independently (CR `0x03BF`, DCR `0xBFFF`,
  RCR `0xFE00`, TCR `0xF000`, IMR/ISR `0x7FFF`, DCR2 `0xF017`, CEP `0x000F`,
  CDC `0x001F`, CDP/RSA/REA/RRP/RWP `0xFFFE`, CAP/SR/MDT `0x0000`) matches the
  datasheet too.
- **Bring-up filter bypasses are tied off.** `ACCEPT_ALL(1'b0)`
  (`fpga_top_dma.vh:410`) and `eth_promisc_enable = 1'b0`
  (`fpga_top_dma.vh:336`), so the architectural CAM/broadcast/multicast filter
  is live in the current build despite commit `594a3af`'s title.

---

## 11. Method notes and limitations

- Figures 3-3, 3-5, 3-8, 3-9, 3-12 and 4-2 are **bitmap artwork** and did not
  survive `pdftotext`. Descriptor field orders in §9 are reconstructed from
  prose plus the DMA access counts at `ds:1256-1261` and `ds:2967-2972`, which
  independently confirm them. **The CDA field order is the one thing the prose
  cannot arbitrate** — see §7.3.
- The datasheet contradicts itself on the RDA status write: 6 words at
  `ds:900` vs 5 at `ds:1041`, against a 7-word total at `ds:1061-1062` and
  `ds:2969`. Only 5 is self-consistent, and both implementations use 5.
- The datasheet contradicts itself on RCR hardware reset: `ds:1833` clears RNT,
  TABLE 5-4's footnote `ds:3401-3402` omits it.
- The datasheet contradicts itself on CR software reset bit 3: `ds:1585` says
  RXEN is unaffected, TABLE 5-4 `ds:3380` shows it as 0. Both implementations
  follow the prose. This is what makes **D-3** reachable.
- OCR mangles: `<` `>` → `k` `l`, `=` → `e`, `←` → `w`, `≤` → `s`, `≥` → `t`.
  Also `51.2 ms` (`ds:1979`) and `6.4 ms` (`ds:2141`) are microseconds.
- **Power-on reset and hardware reset are the same event** in this datasheet
  (`ds:3404-3416`); there is no third scope. The RTL's `rst` input is that
  event.
