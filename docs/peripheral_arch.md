# Peripheral / platform architecture

Decisions captured during the architecture discussion that produced
`sd-hdmi-bringup/`. This is the master spec for the I/O side that
`mac_top.v`, `glue.v`, and the `rtl/mac/` peripherals implement.

---

## Address space partition

```
0x0000_0000 ─┬─ RAM (DDR4, up to 128MB)
             │   D-cache cacheable, write-back
0x0FFF_FFFF ─┘
0x4000_0000 ─┬─ ROM (loaded into DDR4 at boot)
             │   D-cache cacheable, read-only (writes drop / bus-error)
             │   I-cache cacheable
0x40FF_FFFF ─┘
0x5000_0000 ─┬─ I/O (peripheral registers)
             │   Uncacheable, serialised, slow
             │   ┌──────────────┬─────────────────┐
             │   │ +0x000_0000  │ SCC (Z85C30)    │
             │   │ +0x000_1000  │ SCSI (NCR 5380) │
             │   │ +0x000_2000  │ ASC / AWACS     │
             │   │ +0x000_8000  │ SCC (alternate) │
             │   │ +0x010_0000  │ Sound DMA       │
             │   │ +0x0F0_0000  │ VIA1 (6522)     │
             │   │ +0x0F0_2000  │ VIA2 (6522)     │
             │   │ +0x0F0_4000  │ ADB             │
             │   └──────────────┴─────────────────┘
0x50FF_FFFF ─┘
0x6000_0000 ─── Video framebuffer (Mac OS may map here)
             ├── Possibly D-cacheable (write-combining ideal but skip)
0xFFFF_0000 ─── sim-only sentinel range (TB only, never on FPGA)
```

**Decode bits**: 8-bit address-prefix decode is sufficient.
- `addr[31:28] == 4'h0` → RAM
- `addr[31:28] == 4'h4` → ROM
- `addr[31:28] == 4'h5` → I/O
- `addr[31:28] == 4'h6` → video framebuffer

Done in `glue.v` from the AXI ARADDR/AWADDR.

---

## Three subordinate classes

The CPU's dual-LSU (when integrated) emits memory ops in one of three
classes, distinguished by address-bit decode at AGU output:

```
CPU dual-LSU
    │
    ├─ port0 ─┐                          ┌─► RAM (DDR4 via MIG, banked)
    │         ├─► AXI crossbar ─► D-cache┤
    ├─ port1 ─┘                          └─► ROM (DDR4-resident, R/O)
    │
    └─ I/O port (serialised) ──────────────► GLUE ──► peripheral bus
```

**Why this split:**
- **RAM** is the IPC-critical path — must be dual-issue, banked, pipelined.
- **ROM** is read-only with no side effects — fully cacheable, parallel reads
  fine.
- **I/O** has side effects (read clears IFR, etc.) — must be serialised to
  preserve program order.

**LSU detection of I/O addresses**: at AGU output, check `addr[31:28] == 4'h5`.
If either of the two simultaneous LSU ops is I/O, that op gets the I/O port
solo and the other LSU op stalls one cycle. I/O is rare and slow anyway —
this stall is invisible.

---

## AXI → peripheral bus bridge (inside GLUE)

Reduce AXI4-Lite to a minimal synchronous handshake at the GLUE boundary.
Peripherals speak this much simpler protocol:

```verilog
// Peripheral bus (one outstanding transaction at a time)
output [23:0] pb_addr,    // byte address within I/O window (24 bits = 16MB I/O)
output [31:0] pb_wdata,
output  [3:0] pb_wstrb,
output        pb_wr,      // pulse high for one cycle to start a write
output        pb_rd,      // pulse high for one cycle to start a read
input  [31:0] pb_rdata,   // valid when pb_ack is high
input         pb_ack      // hold low for multi-cycle peripherals
```

GLUE's responsibilities:
1. Decode address class (RAM/ROM/IO/video) from AXI ARADDR/AWADDR.
2. Route RAM/ROM transactions to the DDR4 MIG AXI port.
3. For I/O: convert AXI to pb_* handshake. Hold AXI read/write channel
   open while pb_ack is low.
4. Decode I/O sub-region from `pb_addr[23:12]` (4KB peripheral slots).
5. Drive chip-select to the matching peripheral.
6. Mux pb_rdata / pb_ack from the selected peripheral back to AXI.

This is ~200 lines of Verilog, mostly mux. The wip branch already has
`axi_periph_bus.v` with similar functionality — needs review but the
shape is correct.

---

## Reset sequencer

Owned by `clk_rst.v`. Holds CPU reset until everything downstream is ready.

```verilog
module clk_rst (
    input  wire clk_in,
    input  wire rst_in,
    input  wire init_done,   // ANDed externally: ddr_cal & rom_loaded & i2c_done
    output wire clk_out,
    output wire rst_out
);
    // rst_out = !(pll_locked & init_done), through a 4-cycle synchroniser
endmodule
```

**`init_done` sources** (all driven outside this module — clk_rst doesn't care):
- `pll_locked` (from MMCM)
- `ddr_cal_done` (from MIG)
- `rom_loaded` (from boot SD FSM)
- `hdmi_i2c_done` (from i2c_init module already proven in hdmi-bringup)

**In sim** (`tb_top.cpp`): tie `init_done = 1` from cycle 8 onwards. The
boot sequencer is synthesis-only; sim uses a flat MemModel preloaded with
ROM image.

**On FPGA**: the boot sequencer uses the SPI master to read ROM sectors
from SD into DDR4, then asserts `rom_loaded`. Estimated cold-boot time:
under 0.3 seconds after SD init for the current 1 MiB Q700 ROM
(2048 sectors at 25 MHz SPI).

---

## SD card dual-use

One SD card serves two distinct roles:

```
┌─────────────────────────────────────┐  sector 0
│  boot/provisioning window          │
│  first 1 MiB = Q700 ROM sectors    │
│  0..2047 copied by boot_fsm        │
│  total reserved: 4 MiB / 8192 sec  │
├─────────────────────────────────────┤  sector 8192
│  SCSI disk image (rest of card)    │
└─────────────────────────────────────┘
```

**The SPI master is shared, time-multiplexed:**

| Phase | Owner | Action |
|---|---|---|
| Power-on | Boot FSM (in `clk_rst.v`-adjacent module) | Reads sectors 0..2047, writes the 1 MiB Q700 ROM to DDR4 at 0x40000000 |
| CPU running | `scsi.v` (NCR 5380 emulation) | Reads/writes sectors 8192+, presents as SCSI ID 0 disk |

The handoff is one-way: once `cpu_running` (= `init_done` synchroniser
output) goes high, the boot FSM never touches SPI again. A 1-bit ownership
mux on the SPI master input ports.

For the bring-up project (`sd-hdmi-bringup/`), only the boot FSM exists
and it loads a 480×270 RGB565 image instead of ROM. That validates the SD
SPI path before we layer SCSI on top.

---

## VIA1 minimum viable

Required for ROM cold-start. The 6522 has 16 registers at A4=0..15
(word-spaced from VIA1 base):

| A4 | Register | Reset value | What ROM does |
|---|---|---|---|
| 0  | ORB | stored 0x80, reads 0x08 at reset with external PB low | reads bit 3 (live overlay latch); PB[7:5] follow the normal DDR mux; clears overlay after init |
| 1  | ORA | 0x00 | minimal — sound, ADB SR config |
| 2  | DDRB | 0x00 | sets bit 3 to output to clear overlay |
| 3  | DDRA | 0x00 | configures ADB pin direction |
| 4  | T1CL | 0x00 | low byte of timer 1 (write does not start) |
| 5  | T1CH | 0x00 | high byte of timer 1 (write starts countdown) |
| 6  | T1LL | 0x00 | timer 1 latch low (T1 reload value) |
| 7  | T1LH | 0x00 | timer 1 latch high |
| 8  | T2CL | 0x00 | timer 2 low |
| 9  | T2CH | 0x00 | timer 2 high (write starts) |
| 10 | SR | 0x00 | shift register (ADB serial) |
| 11 | ACR | 0x00 | aux control: T1 mode, SR mode, latching |
| 12 | PCR | 0x00 | peripheral control: CA1/CA2/CB1/CB2 modes |
| 13 | IFR | 0x00 | interrupt flag register; bit 7 is derived from `(IFR & IER)` |
| 14 | IER | stored 0x00, reads 0x80 | interrupt enable (write-bit-clear/set per bit) |
| 15 | ORA-NH | 0x00 | ORA without handshake |

**Phase-2 minimum implementation:**
- All 16 registers as a register file (read-back of last write for most).
- ORB bit 3: backed by the live overlay latch. Reads 1 at reset while
  DDRB[3] is still input, then follows ORB[3] after the ROM drives
  DDRB[3]=1 and clears it. The ROM-overlay-mode aliasing of ROM at
  0x000000 is handled separately in glue.v's address decoder, which
  checks this flop.
- ORB bits 7:5: normal 6522 DDR-muxed port bits. MAME's Q700 model does not
  synthesize RAM-size pins here; forcing them high changes the ROM's
  memory-sizing path.
- Timer 1: 16-bit countdown at VIA clock (1 MHz, derived from 200/200=1 MHz
  divider). When reaches 0: assert IFR bit 6, reload from T1L if continuous
  mode (ACR bit 6 = 1).
- IFR / IER: standard 6522 behaviour. IFR bit 7 = OR of (IFR & IER) bits 0-6.
- IRQ output line: drives the 68040 IRQ pin level 1 (via2 also drives 1, ORed).

**Estimated size**: 200 lines of Verilog.

**Phase-3 full-boot upgrade (task #76, `agent/via1-full`):**

The minimum-viable VIA1 above boots the ROM cold-start path, but Mac OS
proper needs four more pieces of 6522 behaviour:

| Feature | IFR bit | Source | Notes |
|---|---|---|---|
| Timer 1 free-run / one-shot | 6 | phi2_tick | Already covered — used for 60 Hz or general SW timers. |
| Timer 2 one-shot | 5 | phi2_tick | For sub-VBL timeouts. |
| SR shift register + attention | 2 | ADB transceiver | Byte-level TX/RX interface. |
| 60 Hz VBL tick | 1 (CA1) | phi2 /16666 | Primary Mac-OS scheduling tick. |

- **SR (ADB)**: VIA1 exposes `adb_rx_byte` / `adb_rx_valid` / `adb_rx_ready`
  inputs and `adb_tx_byte` / `adb_tx_valid` outputs.  The bit-level 100-
  µs ADB framing lives in a separate `rtl/mac/adb_phy.v` (deferred to
  the ADB phy agent).  Writing SR in TX mode (ACR[4:2] ∈ {110,111})
  queues a TX pulse that fires at the next `phi2_tick`; ACR[4:2] = 011
  (shift-in under external clock) lets the transceiver hand over a
  framed byte.  Either direction sets IFR bit 2 (SR-attention).
  Reading or writing SR clears IFR bit 2.
- **60 Hz VBL**: a parametric 32-bit divide of `phi2_tick`
  (`VBL_PHI2_DIV`, default 16666 for 1 MHz phi2 → 60.00 Hz) raises IFR
  bit 1 (CA1) every VBL period.  Reading ORA clears IFR[CA1];
  ORB / ORA-NH do not.
- **RTC**: PB0/PB1/PB2 fan out to `rtc.v` (off-chip real-time-clock +
  256-byte PRAM).  Mac OS shifts an 8-bit command (bit 7 = R/W, bits
  6..2 = reg) then 8 bits of data, MSB-first, via bit-banging rtcClk.
  Command, address, and write-data bits sample the physical host data
  line; if VIA1 releases PB0, the pull-up is sampled as a `1`.
  The "Read Seconds" sequence (commands 0x81, 0x85, 0x89, 0x8D) returns
  the 4 bytes of the 32-bit seconds counter, with register aliases 4..7
  mapping back to the same bytes.  Normal PRAM commands expose register
  windows 8..11 and 16..31; extended PRAM commands use `(cmd & 0x78) ==
  0x38` plus a following address byte to reach all 256 bytes.  The rtc.v
  module ticks its seconds counter off `phi2_tick` with a configurable
  `SEC_DIV` (1_000_000 at 1 MHz phi2 = real-time).
- **IRQ aggregation**: `irq_o = |((IFR & IER) & 7'h7F)` — the summary
  bit (IFR[7]) is excluded.

**Extended port list (agent/via1-full):**

```verilog
module via1 (
    input  clk, rst, phi2_tick,
    input  [3:0] pb_addr,  input  [7:0] pb_wdata,
    input  pb_wr, pb_rd,   output [7:0] pb_rdata,  output pb_ack,
    input  [7:0] pa_in,    input  [7:0] pb_in,
    output [7:0] pa_out, pa_mask, pb_out, pb_mask,
    output       overlay_bit,

    // ADB transceiver (byte-level)
    input  [7:0] adb_rx_byte,  input  adb_rx_valid,
    output       adb_rx_ready,
    output [7:0] adb_tx_byte,  output adb_tx_valid,

    // RTC side-channel (PB0 data, PB1 clk, PB2 enb)
    output       rtc_enb, rtc_clk, rtc_data_o, rtc_data_oe,
    input        rtc_data_i,

    output       irq
);
```

**Sim tests**: `make tb-via1` covers reset, overlay/PB DDR-mux readback,
T1/T2, SR RX/TX, empty-bus ADB completion, VBL, and IRQ aggregation.
`make tb-rtc` covers the seconds counter, read-seconds byte order, command
sampling including pull-up one bits, write-protect/test control registers,
normal PRAM, and extended PRAM.  The ROM harness smokes (`make
tb-rom-boot-via1-t1-smoke`,
`make tb-rom-boot-adb-smoke`, `make tb-rom-boot-rtc-smoke`) exercise the
lightweight VIA1/ADB/RTC shadow model used during real-ROM polling runs.
That shadow keeps explicit Timer 2 one-shot state, logs ADB transaction ids
plus SR mode/ACR/PCR context, and records RTC command direction and
extended-register decode in the peripheral event summary.

---

## VIA2 minimum viable

ROM probes VIA2 for slot-interrupt configuration and additional sense bits.
Phase 2 stub:

- All 16 registers writable but no functional behaviour.
- ORB and ORA return 0xFF on read (or specific reset values matching the
  Quadra schematic — can refine when ROM trace shows what it expects).
- IFR returns 0 (no interrupts pending).
- No timer behaviour.

~80 lines.

---

## SCSI controller (NCR 5380)

8 registers at offsets 0..7 from SCSI base (0x5000_1000).  Byte-wide on
real hardware; on AXI we read/write the full 32-bit word with byte lane
select.  Peripheral-bus-level address is `pb_addr[2:0]`.

### Register layout (as implemented in `rtl/mac/scsi.v`)

| Reg | Read (initiator sees)                    | Write (initiator drives)        |
|----:|------------------------------------------|---------------------------------|
| 0   | Current SCSI Data (cur_data — bus echo)  | Output Data (r_output_data)     |
| 1   | Initiator Command (read-back)            | Initiator Command (r_init_cmd)  |
| 2   | Mode (read-back)                         | Mode (r_mode)                   |
| 3   | Target Command (read-back)               | Target Command (r_tgt_cmd)      |
| 4   | Current SCSI Bus Status (bus_status)     | Select Enable (r_sel_enable)    |
| 5   | Bus and Status (bus_and_stat)            | Start DMA Send (r_dma_send)     |
| 6   | Input Data (= cur_data)                  | Start DMA Target Receive        |
| 7   | Reset Interrupt (reading clears IRQ)     | Start DMA Initiator Receive     |

Bit layouts — positive logic (1 = signal asserted on the SCSI bus):

```
reg 1 Initiator Command:
  [7] RST  [6] AIP/TEST  [5] LA/DIFF  [4] ACK
  [3] BSY  [2] SEL       [1] ATN      [0] DATA_BUS
reg 3 Target Command:
  [3] REQ  [2] MSG  [1] C_D  [0] I_O
reg 4 Current SCSI Bus Status (read-only):
  [7] RST  [6] BSY  [5] REQ  [4] MSG
  [3] C_D  [2] I_O  [1] SEL  [0] DBP
reg 5 Bus and Status (read-only):
  [7] END_DMA     [6] DRQ       [5] PARITY_ERR  [4] IRQ
  [3] PHASE_MATCH [2] BUSY_ERR  [1] ATN         [0] ACK
```

`rtl/mac/scsi.v` uses bit 7 as a completion latch for the command tail
path: it asserts when a command reaches STATUS / MSG_IN / DISCONNECT and
stays high until the initiator reads register 7 (Reset Interrupt).

### Phase FSM (task #77, fully implemented in `rtl/mac/scsi.v`)

Target-side state machine — Mac is the initiator; we emulate a single
disk at `TARGET_ID = 0`.  Any other selection ID keeps us in BUS_FREE
(no-device semantics).

```
          ┌──────────────┐
     ┌───►│   BUS_FREE   │◄────────────────────────────────┐
     │    └──────┬───────┘                                 │
     │           │ SEL asserted + output-data[ID]=1         │
     │           ▼                                          │
     │    ┌──────────────┐                                 │
     │    │   S_SELECT   │  (we assert BSY)                │
     │    └──────┬───────┘                                 │
     │           │ SEL dropped                              │
     │           ▼                                          │
     │    ┌──────────────┐                                 │
     │    │   S_COMMAND  │  REQ/ACK CDB bytes (6 or 10)    │
     │    └──────┬───────┘                                 │
     │           │ CDB complete                             │
     │           ▼                                          │
     │    ┌──────────────┐                                 │
     │    │  S_CMD_EXEC  │  decode opcode                  │
     │    └──┬────┬────┬─┘                                 │
     │       │    │    │                                    │
     │       │    │    └─► S_DATA_OUT ──► S_SD_WAIT_WR ──┐ │
     │       │    │             ▲                         │ │
     │       │    │             │ (more blocks)           │ │
     │       │    │                                       │ │
     │       │    └─► S_SD_WAIT_RD ──► S_DATA_IN ────────┤ │
     │       │               ▲              │             │ │
     │       │               └──────────────┘ (more blks) │ │
     │       │                                            │ │
     │       ▼                                            ▼ │
     │    ┌──────────────┐                                  │
     │    │   S_STATUS   │  REQ/ACK status byte; IRQ on     │
     │    └──────┬───────┘                                  │
     │           │                                          │
     │           ▼                                          │
     │    ┌──────────────┐                                  │
     │    │   S_MSG_IN   │  REQ/ACK command-complete byte  │
     │    └──────┬───────┘                                  │
     │           │                                          │
     │           ▼                                          │
     │    ┌──────────────┐                                  │
     └────┤ S_DISCONNECT │──────────────────────────────────┘
          └──────────────┘
 (RST asserted at any state → unconditional return to BUS_FREE)
```

### Supported CDB opcodes

| Opcode | Command             | Notes                                  |
|-------:|---------------------|----------------------------------------|
| 0x00   | TEST UNIT READY     | zero-data, GOOD status                 |
| 0x03   | REQUEST SENSE       | 18 B fixed-format sense                |
| 0x08   | READ 6              | N × 512 B from backing store           |
| 0x0A   | WRITE 6             | N × 512 B to backing store             |
| 0x12   | INQUIRY             | 36 B vendor/product/rev                |
| 0x15   | MODE SELECT 6       | payload drained, GOOD status           |
| 0x1A   | MODE SENSE 6        | 4 B header (no pages)                  |
| 0x1B   | START STOP UNIT     | GOOD status (we don't spin down)       |
| 0x25   | READ CAPACITY 10    | last LBA + 512-byte block size         |
| 0x28   | READ 10             | N × 512 B from backing store           |
| 0x2A   | WRITE 10            | N × 512 B to backing store             |
| other  | —                   | CHECK CONDITION + ILLEGAL REQUEST      |

### Backing-store interface

`scsi.v` exposes an active-high `drq` output that follows Bus-and-Status
bit 6 during DATA IN / DATA OUT REQ phases.  `fpga_top.v` inverts that
line onto VIA2 PA6 because the Quadra 700 VIA2 sees SCSI DRQ as an
active-low latch input.  PA7 is wired similarly from the active-high SCSI
IRQ latch.

`scsi.v` also exposes a **virtual-HDD** request interface on module ports
— the `vh_*` contract documented in `rtl/vhdd.vh`: a live addressability
probe (`vh_chk_lba / vh_chk_blocks / vh_chk_ok`), a request
(`vh_req_write / vh_req_multi / vh_req_lba / vh_req_block_count /
vh_req_go`), completion (`vh_busy / vh_done / vh_error`) and the
byte-stream handshake (`vh_rd_valid / vh_rd_data / vh_rd_ready` and
`vh_wr_ready / vh_wr_data`).  It is backing-store agnostic: 512-byte
blocks and a volume-relative LBA, nothing else.

    scsi.v ──vhdd──► vhdd_sd.v  ──► sd_scsi_bridge ──► sd_ctrl ──► sd_spi
                   (──► vhdd_ddr.v — future, DDR-backed volume)

`rtl/soc/vhdd_sd.v` is the SD-card provider.  It owns everything
SD-shaped: it re-codes `{req_write, req_multi}` into the SD command class
(CMD17/18/24/25) and biases the volume above the reserved 4 MiB
boot/provisioning window, so `sd_sector = volume_lba + RESERVED_LBAS`
(8192).  `fpga_top_peripherals.vh` drives that parameter and the capacity
clamp from one `SD_RESERVED_LBAS` localparam so the two cannot drift.
`vhdd_sd` is purely combinational — it adds no cycle on the request or
data path.

In the unit testbenches (`tb_scsi.cpp` and friends) a C++ mock fulfills
the handshake, with configurable `inject_error` to exercise the CHECK
CONDITION path.  Those tbs mock an **SD card**, so they build against
`tb/tb_scsi_vhdd_sd.v`, which composes `scsi` + `vhdd_sd` and re-exposes
the `sd_*` interface.

Verification note: there is no focused platform simulation today that
drives an integrated `fpga_top` SCSI data phase and observes VIA2 PA6.
`lint-fpga-top` covers the structural netlist path; dynamic behavior is
covered one level down by the SCSI and VIA2 unit tests.

### IRQ behaviour

`irq` is level-sensitive (per 5380).  Asserted on entry to STATUS and
held through MSG_IN and DISCONNECT.  Cleared when the initiator reads
register 7 (Reset Interrupt) — standard 5380 convention.

NCR 5380 docs are well-known (Linux kernel, MAME, SCSI2SD project).  Use
SCSI2SD as reference for SD-backed SCSI emulation.

---

## SCC stub

Mac ROM reads SCC at boot to detect serial port presence and then polls RR0
while checking for monitor/debug serial input.  The standalone RTL is a
register-aware Z85C30 subset.  The ROM-boot harness keeps a smaller
register-aware SCC shadow: RR0 returns idle-ready status (`0x6c`, TX empty
with no RX byte), RR2/WR2 vector writes are shared, control-port pointer
selection is honored, data reads return no attached serial byte, and writes
are accepted/drained.  Peripheral event logs classify SCC register/data
reads and writes separately and include channel/port/register detail, so ROM
frontier runs show exactly which SCC register was touched without wedging on
common RX/TX polls.

---

## Sound + Video

**Sound (ASC / AWACS)**: ASC is now a ROM-safe SONORA FIFO model, not an
audio-output pipeline. Modeled: version/mode/channel/rate/volume registers,
FIFO writes and read-pointer advance, half-empty IRQ latching, sticky FIFO
overflow/underflow status, FIFO clear, and the per-channel IRQ mask bits the
ROM polls while setting up the startup chime. Intentionally stubbed: the real
wavetable path, codec/DAC behavior, and any board-level audio plumbing
outside the 16-bit `audio_sample_out` simulation hook. AWACS remains future
work.

**Video framebuffer**: phase 3 uses the HDMI bringup as the video output.
Mac OS draws into a framebuffer at a fixed address; we point that address
at a region of DDR4. The HDMI scan-out DMA reads from that region at
pixel clock rate.

Resolution choice for phase 3: probably 832×624 (Mac II 16" display) at
8 bits/pixel = ~520KB framebuffer. Fits easily in DDR4. Scan-out DMA at
148.5 MHz reads ~37 MB/s; DDR4 has plenty of bandwidth.

Eventually: 640×480 24-bit, 1024×768, 1920×1080. Mac OS Color QuickDraw
handles arbitrary resolutions via the slot manager.

---

## What's already on the wip branch

`wip/ddr-peripheral-bringup` contains advance work on the platform side.
Reusable as-is or with minor adaptation:

- `rtl/axi_periph_bus.v` — 235 lines of AXI peripheral routing
- `rtl/axi_pw_sm.v` — 282 lines of AXI write state machine
- `rtl/mac/glue.v` — 186 lines (vs current 5-line stub)
- `rtl/mac/via_core.v` + `via_timer.v` — VIA decomposition
- `rtl/mac/scc_channel.v` — SCC channel implementation
- `rtl/sys/ddr_ctrl.v` — DDR MIG wrapper
- `rtl/sys/clk_rst.v` — clock + reset (already with init_done concept)
- `synth/base.xdc` — KU5P board constraints

**Integration plan** (after phase 1.5 closes):
1. Branch from `main` post-CCR-rename
2. Cherry-pick the peripheral-side changes (most don't touch core)
3. Resolve the LSU split conflict (the hard part — see wip_branches.md)
4. Test against the existing 50 directed tests + the upcoming Mac ROM test

---

## Cross-references

- See [core_gaps.md](core_gaps.md) §5 (cache subsystem) and §7 (bus interface)
  for the core-side requirements that interact with this peripheral plan.
- See [gameplan.md](gameplan.md) phase 2 for the milestone gating order.
