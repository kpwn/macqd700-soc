# SONIC (DP83932C) wire-observable behaviour audit

**Companion:** `docs/sonic_behaviour_audit.md` covers the *software-observable*
side (the register/descriptor interface).  This document covers the other side
of the chip.

**Scope:** everything visible at the MAC/PHY boundary — FCS, padding, preamble,
CSMA/CD, loopback, error handling, duplex — *not* the register/descriptor
interface.  **Audit only; no RTL was changed.**

## Sources

| Tag | Source |
|-----|--------|
| `DS:n` | `~/DP83932C-20-1.PDF` → `pdftotext -layout` → `/tmp/sonic_ds_wire.txt` (4606 lines) |
| `MAME:n` | `/tmp/mame-map-check.pVwuOD/src/devices/machine/dp83932c.cpp` (+ `dp83932c.h` for bit values) |
| `TX:n` | `rtl/mac/q700_sonic_tx.sv` |
| `RX:n` | `rtl/mac/q700_sonic_rx.sv` |
| `LINK:n` | `rtl/board/q700_eth_link.sv` |
| `TAXI` | `/tmp/rk5-taxi/src/eth/rtl/*` (the Taxi tree this build instantiates) |

### Where MAME is NOT a reference

MAME models transmit as `send(buf, length, 4)` into a network backend
(`MAME:417`) and receive as `recv_start_cb(buf, length)` (`MAME:113`).  There is
no preamble, no serialiser, no carrier, no collision domain, no FIFO.  Its own
header lists the gaps: `MAME:12-21` TODOs include *byte count mismatch*, *tally
counters*, *watchdog timers*, and *loopback modes*.  For CSMA/CD, preamble/SFD,
FIFO underrun/overrun, tally counters and the three loopback levels, **MAME's
silence is not agreement** — the datasheet is the only authority.

### The Taxi boundary (verify, don't assume)

`LINK:465-483` instantiates `taxi_eth_mac_1g_rgmii_fifo` with:

```
.cfg_tx_pad_en(1'b1), .cfg_tx_min_pkt_len(8'd59), .cfg_tx_max_pkt_len(16'd1517),
.cfg_tx_ifg(8'd12),   .cfg_rx_max_pkt_len(16'd1517)
```

`59` and `1517` are **not off-by-one bugs** — Taxi's own defaults are
`8'd60-1` and `16'd1518-1` (`TAXI taxi_eth_mac_1g_rgmii.sv:160-163`), i.e. these
are `N-1` encodings meaning *pad to 60 data bytes* and *cap at 1518 total*.
Padding is done by `taxi_axis_pad` before the FCS stage
(`TAXI taxi_eth_mac_1g.sv:212-243`), so 60 data bytes + 4 FCS = 64 on the wire.
Receive-side FCS strip and error flagging are in
`TAXI taxi_axis_gmii_rx.sv:358-407` (4-stage delay line strips FCS; `tuser=1`
on RX_ER, bad FCS, or over-length).

One Taxi behaviour is **not determinable from the sources present**: whether the
RGMII TX path can be told to suppress FCS insertion per frame.  No `cfg_tx_fcs*`
port exists on `taxi_eth_mac_1g_rgmii_fifo`, and `taxi_axis_gmii_tx.sv` inserts
FCS unconditionally, but I did not exhaustively read every Taxi variant.  A
CRCI fix must confirm this rather than assume it.

---

## 1. Lookup table

Verdict key — **CLEAN** = exactly one layer does it correctly; **BOTH** =
double-applied (bug); **NEITHER** = nobody does it (bug); **EXTRA** = our stack
does something the silicon does not; **N/A-FD** = cannot occur on a
full-duplex link and zero is the correct report.

### 1.1 Frame Check Sequence

| # | Behaviour | Datasheet says | MAME does | Taxi does | We do | Verdict |
|---|---|---|---|---|---|---|
| **W1** | **TX FCS when `TCR_CRCI`=1** | `DS:1942-1944` CRCI 1 ⇒ "transmit packet **without** 4-byte FCS field"; `DS:1987` PMB forced low when CRCI set | `MAME:400-407` `if (!(TCR & TCR_CRCI))` append CRC, `length += 4` | inserts FCS unconditionally (`TAXI taxi_axis_gmii_tx.sv`) | `TX:250` latches `descriptor_tcr = word & 16'hf000` (CRCI **is** captured, bit 13) but it is consumed only at `TX:380` (PINT, bit 15) and `TX:349` where `& 16'h07ff` discards every config bit. No path to the MAC. | **BOTH** ⚠ *known* |
| **W2** | TX FCS, normal path (CRCI=0) | `DS:559-561` SONIC generates and appends FCS | `MAME:402-406` | inserts | nothing | **CLEAN** |
| **W3** | RX FCS delivered into the RBA | `DS:562-563` "(The CRC is passed through to buffer memory during reception.)"; `DS:941-942` byte count is "from the start of Destination Address **to the end of FCS**" | `MAME:166-167` writes the whole `buf` (backend buffer carries 4 FCS bytes); `MAME:189` writes `length` incl. FCS | **strips** FCS, exposes only `tuser` / `rx_error_bad_fcs` (`LINK:462,478`) | `RX:125-137` reflected CRC32 (poly `0xEDB88320`, init `0xFFFFFFFF`); `RX:352,368` `fcs_shift <= ~crc`; `RX:377-382` `S_FCS_APPEND` emits 4 bytes low-byte-first, incrementing `frame_len` | **CLEAN shape, masking risk** — see §2.3 |
| **W4** | RX byte count includes FCS | `DS:941-942` | `MAME:189` | n/a | `RX:140` `rda_len = frame_len`, which has the 4 appended bytes; `RX:237-239` writes it to `RXpkt.byte_count` | **CLEAN** |

### 1.2 Padding and length

| # | Behaviour | Datasheet says | MAME does | Taxi does | We do | Verdict |
|---|---|---|---|---|---|---|
| **W5** | TX pad to 60 data bytes | `DS:638-643` **"The SONIC does not append pad bytes for short packets during transmission"**; `DS:652-655` Note 3 — the *driver* pads by lengthening `TXpkt.pkt_size`/`frag_size`; `DS:604` conformance table marks "Pad Length Generation" as *User Driver Software* | nothing — `MAME:395-396` copies exactly `TFS` bytes | **pads** to 60 (`LINK:480-481`, `TAXI taxi_eth_mac_1g.sv:212-243`) | nothing | **EXTRA** |
| **W6** | Pad in loopback | as W5 (no pad) | no pad | **bypassed** — loopback never reaches the MAC (`LINK:139-143`, `LINK:240-244`) | no pad | **CLEAN** (matches silicon; and therefore *inconsistent with our own wire path*) |
| **W7** | Runt filter, < 64 bytes incl. FCS | `DS:1830-1832` `RCR_RNT` accepts "packets less than 64 bytes"; `DS:1889` PRX excludes length errors | `MAME:134` `(length < 64) && !(RCR & RCR_RNT)` → reject; `length` includes FCS | no minimum-length check on RX | `RX:384` `(rda_len<64) && ((rcr&RCR_RNT)==0)` → drop; `rda_len` includes the appended FCS | **CLEAN** |
| **W8** | Oversize receive | `DS:639` "nor check for oversize packets during reception"; `DS:646-648` "up to 64k bytes in length"; `DS:599`+Note 1 — max frame size is the *driver's* job via `RXpkt.byte_count` | no ceiling (`MAME:113-134` checks only the floor) | drops > 1518: `cfg_rx_max_pkt_len` (`LINK:482`) → `tuser` (`TAXI taxi_axis_gmii_rx.sv:391-394`) → killed by `DROP_BAD_FRAME` in `rx_cdc_fifo` (`LINK:639-643`) | no check | **EXTRA** (silent drop, no status bit) |
| **W9** | Oversize transmit | `DS:646-648` up to 64 KB | no ceiling (`MAME:380` `u8 buf[1520]`, FIXME at `MAME:379`) | caps at 1518 (`LINK:481`) | caps at 3072 — `MAX_CHUNKS=48` × 64 B (`TX:12`, `TX:178-179`), overflow → `failed` at `TX:295-299` | **EXTRA** ×2 |
| **W10** | `TCR_BCM` — `pkt_size` vs Σ`frag_size` | `DS:1996-1999` set when `TXpkt.pkt_size != Σ TXpkt.frag_size`; **transmission is aborted**; `DS:2000-2005` PTX excludes BCM; `DS:2166-2172` ISR_TXER | **not implemented** (`MAME:14` TODO) | n/a | `TX:251` reads TPS into `done_tps`, `TX:266` reads each `frag_len` — **never compared** | **NEITHER** |

### 1.3 Preamble, SFD, inter-frame gap

| # | Behaviour | Datasheet says | MAME does | Taxi does | We do | Verdict |
|---|---|---|---|---|---|---|
| **W11** | Preamble + SFD generation | `DS:559-561` SONIC generates and appends preamble + SFD; `DS:324-331` 7-byte preamble generator, always sent in full | not modelled | emits `ETH_PRE`/`ETH_SFD` | nothing | **CLEAN** |
| **W12** | Preamble + SFD strip | `DS:561` "stripped during reception"; `DS:552-561` 62-bit preamble, SFD = two consecutive 1s | not modelled | strips (`TAXI taxi_axis_gmii_rx.sv:318-326`) | nothing | **CLEAN** |
| **W13** | Inter-frame gap | `DS:280-283` IFG timer 9.6 µs (= 96 bit times at 10 Mb) | not modelled | `cfg_tx_ifg(8'd12)` = 12 byte times = 96 bit times (`LINK:481`) | nothing | **CLEAN in bit-times**; 100× shorter in wall-clock (see W26) |

### 1.4 CSMA/CD — TXpkt.status

`DS:1224-1237` (Figure 3-14) defines `TXpkt.status` as `NC[4:0]` (collision
count) in bits 15-11 plus `TCR[10:0]`.  **Our transmit engine writes the literal
constant `TCR_PTX` (`0x0001`) into the descriptor** (`TX:186-196`), and reports
`done_tcr = (descriptor_tcr | TCR_PTX) & 16'h07ff` = `0x0001` (`TX:349`).  Every
row below is therefore permanently zero.  MAME is equivalent —
`MAME:427-431` writes `m_reg[TCR] & TCR_TPS` where `TCR_TPS = 0x07ff` and only
`TCR_PTX` was ever OR'd in; its collision TODO is at `MAME:425`.

| # | Bit | Datasheet meaning | What a driver may infer | We report | Verdict |
|---|---|---|---|---|---|
| **W14** | `NC[4:0]` bits 15-11 | `DS:1227-1228` number of collisions this transmission; `DS:530-532` feeds the 802.3 single/multiple/collision-frame statistics | link congestion | always 0 | **N/A-FD** — 0 is correct |
| **W15** | `DEF` (9) | `DS:1961-1963` deferred on first attempt; reset on subsequent collisions | medium was busy | always 0 | **N/A-FD** |
| **W16** | `EXD` (10) | `DS:1949-1951` deferring for 3.2 ms; aborts transmission if `EXDIS`=0 | cable/hub fault | always 0 | **N/A-FD** |
| **W17** | `EXC` (6) | `DS:1976-1977` 16 collisions; transmission aborted; also forces BCM (`DS:1998-1999`) | attempt limit reached | always 0 | **N/A-FD** |
| **W18** | `OWC` (5) | `DS:1978-1981` late collision after one slot time (51.2 µs), window start selected by `POWC` | segment too long / duplex mismatch | always 0 | **N/A-FD** |
| **W19** | `NCRS` (8) | `DS:1964-1968` set at start of preamble, cleared if CRS ever seen ⇒ **stays set means no carrier**. `DS:1969` "NCRS will remain set in MAC loopback as long as there is no activity on the RX±" | **cable unplugged / transceiver dead** | always 0 (= "carrier was present") | **NEITHER**, but the *safe* direction |
| **W20** | `CRSL` (7) | `DS:1971-1974` CRS went low or was absent during transmission. `DS:1974` **"CRSL will always be set in MAC loopback"** | marginal link | always 0 | **NEITHER** — a self-test written to expect `CRSL` in MAC loopback will mis-diagnose |
| **W21** | `PMB` (3) | `DS:1983-1985` set when the self-received frame has a bad CRC/frame-alignment error **or the source address is not in the CAM** (`DS:612-614`, `DS:1207-1209`). `DS:1991` always zero in all three loopback modes | driver forgot to load its own MAC into the CAM | always 0 | **NEITHER** (MAME agrees) |
| **W22** | `FU` (2) | `DS:1993-1995` FIFO underrun from bus latency; transmission aborted | host memory too slow | always 0. Taxi's `tx_error_underflow` (`LINK:460,476`) goes **only** to `debug_event_toggle` (`LINK:490-501`), a JTAG telemetry XOR | **NEITHER** |
| **W23** | `ISR_TXER` (bit 8) | `DS:2166-2172` set on BCM \| EXC \| FU \| EXD | "a transmit failed" | never set (all four sources are dead) | follows W10/W17/W16/W22 |
| **W24** | `ISR_HBL` (bit 13) | `DS:2140-2142` transceiver failed to send the collision heartbeat within 6.4 µs of IFG | 10BASE-T transceiver SQE test failed | never set | **N/A-FD** — correct; there is no such transceiver |

**Hang analysis.** None of W14-W24 is a bit a normal driver spins on: they are
all *error-occurred* flags read once after a transmit completes.  The only bit a
driver polls in a loop is `PTX`, which we do set (`TX:186-196`, and the
completion is raised even in loopback via `sonic_loopback_cpl`,
`LINK:142-143`, `LINK:668-675`).  So the CSMA/CD gap is a **diagnostics** gap,
not a hang.

### 1.5 Loopback

| # | Behaviour | Datasheet says | MAME does | We do | Verdict |
|---|---|---|---|---|---|
| **W25** | `RCR_LB1/LB0` levels | `DS:1857-1866`: `01`=MAC, `10`=ENDEC, `11`=transceiver. `DS:485-489` MAC loopback never reaches the ENDEC. `DS:454-465` ENDEC loopback: not transmitted from the chip, CD± ignored, **full CSMA/CD still followed**. `DS:467-476` transceiver loopback: **data IS transmitted from the chip** and is affected by network activity | `MAME:268-271` `set_loopback(bool(RCR & RCR_LB))` — one boolean; `MAME:19` TODO "loopback modes" | `fpga_top_dma.vh:325` `sonic_loopback = (rx_dbg_rcr & 16'h0600) != 0` — one boolean. `LINK:139-143` diverts SONIC TX straight into SONIC RX; `LINK:240-244` muxes it; the frame never reaches the wire | **NEITHER** for `LB=11` (transceiver loopback should still transmit). Matches MAME |
| **W26** | `RCR_LBK` status | `DS:1886-1887` "successfully received a loopback packet" | `MAME:149-150` sets `LBK` whenever `RCR_LB` is set | `RX:393-394` `(((rcr&RCR_LB)!=0)?RCR_LBK:0)` | **CLEAN** (matches MAME) |
| **W27** | "one packet queued" limit | `DS:507-512` in **MAC loopback only**, one packet at a time, because the transmit MAC generates no IFG and the receive MAC cannot update status | not modelled | no limit — loopback is a straight byte stream, back-to-back frames work | **EXTRA permissiveness**, harmless |
| **W28** | Address filtering during loopback | `DS:477-480` looped frames "are filtered by the address recognition logic and buffered to memory if accepted"; `DS:497-500` the procedure requires loading the CAM with the destination address | filter runs (`MAME:131`) | filter runs — loopback enters at `S_READY` and passes `S_FILTER` (`RX:383-401`) | **CLEAN** |
| **W29** | Loopback vs wire pad asymmetry | — | — | wire path pads (W5), loopback path does not (W6) | see §2.4 |

### 1.6 Receive error handling

| # | Behaviour | Datasheet says | MAME does | Taxi does | We do | Verdict |
|---|---|---|---|---|---|---|
| **W30** | `RCR_ERR` (bit 15) | `DS:1827-1829` 0 = reject frames with CRC errors or collisions; 1 = **accept** them | `MAME:137-144` implements exactly this: bad residue + `RCR_ERR` ⇒ set `RCR_CRCR` and keep the frame; otherwise drop | flags bad FCS via `tuser` (`TAXI taxi_axis_gmii_rx.sv:400-407`) | **never decoded.** `RX:155-157` `filter_accept` has no `RCR_ERR` term | **NEITHER** |
| **W31** | `RCR_CRCR` (3) / `RCR_FAER` (2) | `DS:1879-1885`; FAER only set when *both* alignment and CRC errors occur | `MAME:141` sets `CRCR` | — | never set. `RX:393-396` builds `status_rcr` from `PRX\|LBK\|BC\|MC\|LPKT` only | **NEITHER** |
| **W32** | Bad frames reaching the engine at all | — | reach `recv_start_cb` | **killed upstream**: `rx_cdc_fifo` has `DROP_BAD_FRAME(1'b1)` (`LINK:639-643`), which drops on `tlast` when `tuser` is set (`TAXI taxi_axis_fifo.sv:288`) | `RX:348,366` compute `frame_bad` from `rx_axis_tuser` and drop at `RX:384` — **dead logic in the fpga_top build**, since no `tuser` frame ever survives the CDC FIFO | **NEITHER** |
| **W33** | `CRCT` / `FAET` / `MPT` tally counters | `DS:541-546` Table 1-1: FCS errors → `CRCT` + `RCR.CRC`; alignment → `FAET` + `RCR.FAE`; frames lost to internal MAC receive error → `MPT` + `ISR.RFO`. `DS:2035-2037` `CRCEN`/`FAEEN`/`MPEN` interrupt enables exist for their rollovers | **not implemented** (`MAME:16` TODO) | — | never increment | **NEITHER** (MAME agrees) |
| **W34** | `ISR_RFO` + `MPT` on internal loss | `DS:2130` receive FIFO overrun; `DS:545-546` pairs it with `MPT` | not modelled | `rx_fifo_overflow`, `rx_fifo_bad_frame`, `rx_error_bad_frame` (`LINK:462-463, 478-479`) | wired **only** to `debug_event_toggle` (`LINK:490-501`). Frames lost to `DROP_BAD_FRAME`/`DROP_OVERSIZE_FRAME` (`LINK:639-643`) or to `rx_drain` (`LINK:232,242`, `fpga_top_dma.vh:328`) are invisible to the guest | **NEITHER** |
| **W35** | `RCR_CRS` (5) | `DS:1875-1876` "Set when CRS is active. Indicates the presence of network activity" — set for essentially every received frame on real silicon | never set (`MAME:128` clears bits 8-6 and 3-0, which does not include CRS, and no path sets it) | — | never set (`RX:393-396`) | **NEITHER** (MAME agrees — and the Mac driver works in MAME, so nothing depends on it) |
| **W36** | `RCR_COL` (4) | `DS:1877-1878` collision during reception; `DS:1889` PRX excludes collisions | never set | — | never set | **N/A-FD** — 0 is correct |
| **W37** | Status-bit clear scope | `DS:1798-1799` "Bits 8-6 and 3-0 are cleared at the reception of the next packet" (5 and 4 are **not** in that set) | `MAME:128` clears exactly `MC\|BC\|LPKT\|CRCR\|FAER\|LBK\|PRX` = bits 8,7,6,3,2,1,0 — faithful | — | `RX:349` `status_rcr <= rcr & 16'hfe00` clears **all** status bits including 5 and 4 | **CLEAN in effect** — CRS/COL are never set anywhere, so the wider clear is unobservable |

### 1.7 Duplex, speed, and the second address filter

| # | Behaviour | Datasheet says | We do | Verdict |
|---|---|---|---|---|
| **W38** | Media | `DS:552-561` 10 Mb Manchester, half duplex, CSMA/CD; `DS:287-293` truncated binary exponential backoff; `DS:346-353` 4-byte jam generator; `DS:1244-1247` retransmit up to 15 times | 1 Gb full duplex RGMII (`LINK:465-483`, `link_speed` at `LINK:362`) | **context** — the root cause of every `N/A-FD` above |
| **W39** | Frame arrival rate | implied 10 Mb ceiling | 1 Gb; the guest's descriptor ring can be overrun ~100× faster than on silicon. Mitigations already present: `rx_drain` (`LINK:374-380`, `fpga_top_dma.vh:328`), 4096-deep MAC frame FIFOs, and `ISR_RDE`/`ISR_RBE` (`RX:449,475,340`) | **context** — not a wire divergence, but the reason the RDE/RBE paths are load-bearing here in a way they are not on silicon |
| **W40** | `blk_mac` demux ahead of the SONIC | `DS:1840-1843` `RCR_PRO` promiscuous mode "Enable all Physical Address packets to be accepted" | `LINK:281-307`: a frame whose destination exactly equals `blk_mac` is routed to the block client **only** (`route_sonic <= 1'b0`, `LINK:302`). The SONIC never sees it — not even in promiscuous mode. Inert today: `blk_mac` is all-zero unless `ENABLE_NET_VHDD` (`fpga_top_ethernet.vh:61`, `LINK:100`), and `blk_present=0` collapses routing to the pre-sharing behaviour | **EXTRA** — a second address filter in front of the CAM |

---

## 2. Analysis of the non-obvious cases

### 2.1 W1 — the CRCI double-FCS (known)

`descriptor_tcr` **does** capture CRCI: `TX:250` masks with `16'hf000`, which
keeps bits 15-12 = `PINT`, `POWC`, `CRCI`, `EXDIS` — exactly the four config bits
the datasheet says are loaded from `TXpkt.config` (`DS:1206-1209`,
Figure 3-13 at `DS:1212-1221`).  Bit 15 is then used for `done_pint` (`TX:380`),
and `TX:349` masks the register down to `16'h07ff` for the status write-back,
which discards all four.  So CRCI is read, carried, and thrown away.

The wire consequence with CRCI=1: the driver's own 4 FCS bytes sit at the end of
the last fragment, Taxi's pad stage may insert zeros **after** them if the frame
is under 60 bytes (`LINK:480-481`), and Taxi then computes a real FCS over the
whole thing.  The peer sees a frame 4 (or more) bytes too long whose payload
tail is a stale CRC.

### 2.2 W10 — BCM is the only actionable transmit-status bit

Every other zero in §1.4 is either physically impossible on this link
(W14-W18, W24) or a "no fault detected" report that errs safe (W19-W22).
BCM is different: it is a **descriptor-consistency check**, not a media
condition, and it is fully reproducible here.  We already have both operands —
`done_tps` (`TX:251`) and the per-fragment `fragment_len[]` (`TX:266`).
Without it, a driver that miscomputes `TXpkt.pkt_size` gets a silently
wrong-length frame on the wire instead of an abort plus `ISR_TXER`.

### 2.3 W3 — the reconstructed FCS is correct, and that is the problem

The reconstruction itself is right: reflected CRC32 with poly `0xEDB88320` and
init `0xFFFFFFFF` (`RX:125-137`), final complement (`RX:352`, `RX:368`), emitted
low byte first (`RX:379`).  That is the standard Ethernet FCS, and the resulting
`RXpkt.byte_count` correctly includes it (`DS:941-942`).

But the CRC is computed over **the bytes we received from the fabric**, not the
bytes that arrived on the wire.  Taxi validates the wire FCS and then discards
it; from that point the frame crosses `rx_cdc_fifo` (`LINK:639-643`), the
destination-capture/replay path in `q700_eth_stream_share` (`LINK:281-339`), a
byte-to-chunk staging register (`RX:212-220`) and a BRAM (`RX:97`) before the
FCS is regenerated over whatever came out.  Any corruption introduced in that
chain is **re-blessed with a self-consistent FCS**.  A guest that re-validates
the FCS in the RBA — a normal driver sanity check, and a normal bring-up
technique — can never fail.

This is a divergence in *observability*, not in specified behaviour.  Silicon
hands over the real received FCS; we hand over a receipt for our own work.

### 2.4 W29 — loopback and wire disagree with each other

Because loopback bypasses the MAC entirely (`LINK:139-143`, `LINK:240-244`):

* a sub-60-byte frame in **loopback** stays short, gets 4 reconstructed FCS
  bytes, lands under 64, and is dropped by the runt filter (`RX:384`) unless the
  driver set `RCR_RNT`.  **This matches silicon and MAME.**
* the same frame on the **wire** is padded to 64 by Taxi (`LINK:480-481`) and
  goes out as a legal frame.  **This does not match silicon** (`DS:638-643`).

So the loopback path is the faithful one and the wire path is the divergent one.
A driver that self-tests with a short frame in loopback, concludes "short frames
are rejected", and then relies on that when transmitting is fine; a driver that
validates padding behaviour through loopback and assumes the wire matches is not.

### 2.5 W32 — `frame_bad` is dead logic in the shipped configuration

`RX:348` and `RX:366` accumulate `rx_axis_tuser` into `frame_bad`, and `RX:384`
drops the frame on it.  But `rx_cdc_fifo` is instantiated with
`DROP_BAD_FRAME(1'b1)` (`LINK:639-643`), and Taxi drops on `tlast` whenever
`tuser` matches the bad-frame mask (`TAXI taxi_axis_fifo.sv:288`).  No frame with
`tuser` set can reach `q700_sonic_rx` in an `fpga_top` build.  The path is still
live for the standalone unit testbenches, which drive `rx_axis_*` directly — so
this is not dead code to delete, it is a **hook with nothing connected to it**.
Any future `RCR_ERR` implementation must move the drop decision out of the CDC
FIFO, not just add a term to `filter_accept`.

---

## 3. Divergences ranked by real-driver impact

Ranked by: could this affect a real Macintosh SONIC driver on a modern
full-duplex 1 G link?

| Rank | # | Divergence | Why it ranks here |
|---|---|---|---|
| 1 | **W1** | `TCR_CRCI` ignored ⇒ double FCS | The only **BOTH**. Every frame a driver sends with CRCI set goes out ≥ 4 bytes too long with a stale CRC as payload — corrupt on the wire, dropped by every peer. Conditional on a driver using CRCI, but total when it does. *(known)* |
| 2 | **W34** | Internal frame loss reported nowhere — no `ISR_RFO`, no `MPT` | Happens on a live link under load, today, with no driver bug required. Frames vanish between Taxi and the ring and the guest has **zero** indication: no interrupt, no counter, no descriptor. Turns an intermittent hardware problem into an undebuggable one from inside Mac OS. |
| 3 | **W3** | RX FCS regenerated rather than passed through | Behaviour is per-spec, but it makes every frame we deliver self-consistent by construction. Any corruption in the CDC/replay/BRAM chain is invisible to guest-side and host-side FCS checks alike — it removes the one end-to-end integrity check the architecture had. |
| 4 | **W8**, **W9** | Taxi enforces a 1518-byte ceiling the SONIC does not (RX and TX); our TX engine adds a 3072-byte one | Silicon accepts to 64 KB (`DS:646-648`) and explicitly leaves the ceiling to the driver (`DS:599` Note 1). Over-length frames are dropped silently with no ISR bit. Harmless for EtherTalk/IP at a 1500 MTU; the *silent* part is what makes it worth fixing. |
| 5 | **W10** | `TCR_BCM` never checked | A driver descriptor bug (`pkt_size` ≠ Σ`frag_size`) produces a silently wrong-length frame on the wire instead of an abort + `ISR_TXER`. Both operands are already in the RTL; this is the cheapest real fix on the list. |
| 6 | **W30**, **W31**, **W33** | `RCR_ERR` / `CRCR` / `FAER` / tally counters unimplemented | A driver diagnostic that enables `RCR_ERR` to count CRC errors reads zero forever and concludes the link is perfect. MAME implements `ERR`→`CRCR` (`MAME:137-144`), so this is a divergence from the golden model too. Needs W32 addressed first. |
| 7 | **W40** | `blk_mac` demux filters ahead of the CAM, bypassing `RCR_PRO` | Only live under `ENABLE_NET_VHDD`. Promiscuous mode is specified to see all physical-address frames (`DS:1840-1843`); here one address is structurally invisible. A packet sniffer in the guest would show a hole. |
| 8 | **W5**, **W29** | Taxi pads; silicon does not; loopback and wire therefore disagree | Peer-visible only, and in the benign direction (we emit legal frames where silicon emits runts). Ranks low but is a real "our two paths differ" hazard for anyone validating through loopback. |
| 9 | **W25** | `LB1/LB0` collapsed to one boolean | `LB=11` (transceiver loopback) is specified to still transmit on the wire (`DS:467-472`). Matches MAME, needs an external transceiver to matter, no known driver use. |
| 10 | **W20**, **W19** | `CRSL`/`NCRS` never set, including MAC loopback where `DS:1974` says `CRSL` is **always** set | A loopback self-test written against the datasheet expecting `CRSL` mis-diagnoses. Errs in the safe direction (we report "no fault"). |
| 11 | **W21**, **W35** | `PMB` and `RCR_CRS` never set | Both are also absent in MAME, and the Mac driver demonstrably works there. Documented for completeness. |
| 12 | **W22**, **W23** | `FU` / `ISR_TXER` never set | Taxi's `tx_error_underflow` exists (`LINK:460`) and could feed `FU`. Underflow should not occur with the current 4096-deep frame FIFO. |
| — | W14-W18, W24, W36 | CSMA/CD status bits and `ISR_HBL` | **Not bugs.** Collisions, deferral, backoff and the SQE heartbeat cannot occur on a full-duplex link; zero is the correct report. No driver spins on any of them — they are read-once error flags. Listed so nobody "fixes" them. |
| — | W2, W4, W6, W7, W11, W12, W13, W26, W28, W37 | FCS/pad/preamble/SFD/IFG/runt/LBK/filter/clear-scope | **Verified clean** — exactly one layer does each, correctly. |

## 4. Summary

* **16 divergences** (W1, W3*, W5, W8, W9, W10, W19, W20, W21, W22, W25, W30/31, W33, W34, W35, W40), of which **15 are new** beyond the already-known CRCI bug.
  *W3 is a divergence in observability rather than in specified behaviour.*
* **1 "BOTH"** (double-application): W1, the known CRCI case.  No other
  double-application exists — FCS, padding, preamble, SFD, IFG and the runt
  filter each have exactly one owner.
* **11 "NEITHER"**: W10, W19, W20, W21, W22, W23, W30, W31, W33, W34, W35.
* **4 "EXTRA"** (our stack does what silicon does not): W5 (pad), W8 (RX
  ceiling), W9 (TX ceiling), W40 (second address filter).
* **10 verified clean**, and **7 correctly-zero** half-duplex/SQE bits that
  should be left alone.
