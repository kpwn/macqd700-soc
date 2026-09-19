# SONIC/Taxi JTAG telemetry

The Ethernet telemetry page is an opt-in, low-area alternative to an ILA or
wide VIO probe set. Build the real SONIC datapath with:

```sh
make impl CPU=m68k ETH_ENABLE=1 ETH_ICMP_RESPONDER=0 ETH_DEBUG_ENABLE=1
```

`ETH_DEBUG_ENABLE` defaults to `0`. The Vivado flow rejects it unless the
SONIC DMA datapath is selected. With the flag off, the CSR slave and debug
split are absent; the otherwise-unused observation outputs are pruned.

The page is visible through JTAG AXI at `0x5098_0000`, the upper half of the
existing `0x5090_0000` debug-service aperture. The normal CPU debug registers
remain in its lower half. In `tools/jtag_repl.tcl` use:

```text
eth-status
eth-clear
```

`eth-status` first checks the ABI identifier and refuses to decode an old or
non-debug bitstream.  When capability bit 5 is present it also prints the
SONIC `DCR` and the descriptor width the TX/RX engines derive from its `DW`
bit -- the one configuration input that decides how every descriptor in the
ring is parsed, and which is otherwise invisible from JTAG because the
peripheral bus is not reachable from the debug AXI master. `eth-clear` clears all saturating counters and the sticky
first-error snapshot; it does not reset SONIC, Taxi, or the DMA engine.

## CSR map

| Offset | Contents |
|---:|---|
| `0x00` | identifier `0x45544801` (`ETH`, ABI 1) |
| `0x04` | capability/version bitmap |
| `0x08` | link, IRQ, RX enable, engine states, first-error-valid; write bit 0 to clear telemetry |
| `0x0c` | SONIC `CR` (upper 16), `IMR` (lower 16) |
| `0x10` | SONIC `ISR` (upper 16) and status bits |
| `0x14`, `0x18` | TX/RX state and current frame length |
| `0x1c`, `0x20` | current TX/RX descriptor addresses |
| `0x24` | most recent completed TX/RX frame lengths |
| `0x28`, `0x2c` | most recently accepted DMA address and metadata |
| `0x30`, `0x34` | first-error type/details and associated address |
| `0x38` | Taxi event toggle synchronization state |
| `0x3c` | SONIC `DCR` (lower 16).  Bit 5 is `DW`: set = 32-bit descriptors.  Present only when capability bit 5 is set. |
| `0x74` | RX admission: `RCR` (upper 16), CAM enable mask (lower 16).  Capability bit 6. |
| `0x78`, `0x7c` | CAM entry 0, low 32 / high 16 bits of the 48-bit address. |
| `0x40`–`0x58` | 16-bit saturating SONIC frame/completion and DMA request/response/error counters |
| `0x60`–`0x6c` | 16-bit saturating Taxi event counters |
| `0x70` | runtime RX destination-filter bypass (`eth-promisc`) |
| `0x80` | trace ring CTRL — W: bit 0 freeze, bit 1 clear+re-arm; R: `{frozen[17], wrapped[16], wr_ptr[11:0]}`.  Capability bit 7. |
| `0x84` | trace ring index for the next `0x88` read |
| `0x88` | `ring[0x84]`.  Does **not** auto-increment: the RAM read is registered, so an auto-increment would return the previous index's word and every dump would be silently off by one. |
| `0x8c` | live suppressed-poll total.  Valid **without** freezing. |

`eth-status` also decodes which receive-admission terms are live
(`PRO`/`BRD`/`AMC`/`CAM`).  A validated frame that matches none of them is
dropped before any DMA is issued, so the failure presents as "RX does not
work" with every DMA counter at zero and nothing in the error snapshot.

Taxi events are, in bit order 0 through 7: TX underflow, TX FIFO overflow,
TX bad frame, TX good frame, RX bad frame (MAC or FIFO), RX bad FCS, RX FIFO
overflow, and RX good frame. They cross from the 125 MHz MAC domain as toggle
bits so single-cycle event pulses remain observable at the 100 MHz core.

## Register-access trace ring

The counters above are a snapshot.  They cannot answer a **sequence**
question, and every remaining Ethernet question is one: did the driver ever
write `IMR`, or write it and have something clear it?  What did it actually
put in CAM entry 15?  Does `CR_TXP` stick set after a handful of transmits and
silently block every later one?  A snapshot cannot tell "never set" from "set
then cleared".

`rtl/soc/sonic_trace_ring.v` records the register-access stream itself.  It is
a pure observer on the `peripheral_bus` <-> `q700_eth_sonic` port — every port
on it is an input, and it never issues an access of its own — so it can run
through a whole boot without perturbing what it measures.  It is instantiated
only under `ETH_DEBUG_ENABLE`, next to `eth_debug_regs` in
`rtl/soc/fpga_top_dma.vh`.

```text
sonic-trace status          # wr_ptr / wrapped / frozen / live filtered count
sonic-trace dump [<path>]   # default /tmp/sonic_trace.csv
sonic-trace freeze
sonic-trace rearm
```

The dump's first four columns are the same `idx,rw,reg,data` shape the 53C96
ring emits, so `tools/scsi96_trace_diff.py`'s parser reads it unchanged and
the same differ style works against a MAME capture of this driver.  Two extra
columns follow: `strb` (the byte strobes) and `skipped` (see below), plus a
register-name comment column.

**The byte strobes are not decoration.**  This driver does half-register
writes and `q700_eth_sonic` decodes `CR`'s two halves as different commands
(low byte through `sonic_command_low()`, high byte for `RRRA`/`LCAM`), so a
trace without them could not tell "wrote `0x0002` to CR (TXP)" from "wrote
`0x00` to CR's high half".  Reads always record `3`: the SONIC returns the
whole register and the CPU's read granularity is resolved inside
`peripheral_bus.v`, so it is genuinely not observable at this port.

### The poll filter

A wedged driver spins on `ISR` (with `IMR` = 0 no interrupt can assert, so the
service path degenerates into re-reading it) and on `CR` (waiting for `CR_TXP`
to self-clear).  A plain last-N-wins ring would be overwritten by that spin
before anyone could read it out, destroying exactly the history that matters.

So: **writes are always recorded** (they are the commands, and a repeated
identical command is itself the signal); **a read is recorded only if its
16-bit value differs from the last value read from that same register**, with
a private shadow per register index; and **a write invalidates that
register's read shadow**, so the read-back after a write — the only place the
trace shows what a write-masked, command-decoded or write-1-to-clear register
actually did — is never dropped.

Value identity is a sound proxy here, more so than it was for the 53C96:
SONIC register reads have no side effects, so two equal consecutive reads
provably carry no state transition between them.

Consequence: during a wedge the ring **stops advancing on its own** and
preserves the pre-wedge window indefinitely, with no watchdog and no trigger
logic.  A re-arm wipes the read shadows, so every capture window opens with a
baseline entry per register and is self-contained.

What the filter must not throw away is whether the driver is still spinning at
all, since "hammering ISR and never seeing a bit set" and "stopped touching
the SONIC entirely" are different diagnoses that both look like silence.  So
suppressed reads are counted twice: per entry as `skipped` (saturating 0..127,
"the driver spun this deep immediately before this event"), and live in the
`0x8c` total.  `sonic-trace dump` samples `wr_ptr` **and** that total 1.5 s
apart before freezing and reports which of the three states it found: still
advancing, spinning on a constant register, or not touching the SONIC at all.

`make tb-sonic-trace-ring` is the unit tb; it runs a positive control first
(`pb_ack` held low so the DUT cannot commit an entry) and requires every
capture assertion to fail there.

The snapshot is designed to answer whether the driver configured SONIC,
whether descriptors and DMA are advancing, whether frames reach Taxi in each
direction, and where the first error occurred.

The Taxi-event mask that arms the sticky first-error snapshot covers bits 0,
1, 2, 4, 5 and 6.  It deliberately excludes bit 3 (TX **good** frame) and bit 7
(RX **good** frame).  An earlier mask included bit 3, so every successful
transmit armed the snapshot and a healthy link permanently reported
`first_error=1  info=0x10000008`. It is not a cycle trace; use a
temporary ILA only when ordering inside one transaction must be reconstructed.
