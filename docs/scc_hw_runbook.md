# SCC HW bring-up runbook

This is the step-by-step guide to driving MacsBug interactively from a host
serial terminal once the Q700 ROM is running on the FPGA.  See
[`docs/scc_lockstep.md`](scc_lockstep.md) for the MAME-vs-RTL TX byte-stream
lockstep framework that validates SCC TX correctness against MAME's
`macqd700` driver; this doc is the HW-side counterpart.

## Hardware prerequisites

- Bitstream built from `main` (or the agent worktree) on the KU5P board.
- Board UART pin (`uart_rtl_0_txd` / `uart_rtl_0_rxd`) wired to a
  USB-serial adapter on the host.  The bitstream's
  `uart_byte_bridge` is parameterised with `BAUD = 115200` over a
  50 MHz `pb_clk`, giving 8N1 framing.

## Host-side serial settings

| Knob | Value |
|---|---|
| Baud | `115200` |
| Data bits | 8 |
| Parity | none |
| Stop bits | 1 |
| Flow control | none |
| Line ending | CR (`\r`) — MacsBug accepts it as Enter |

## SCC channel selection

The bitstream wires ONE board UART pin pair to BOTH SCC channels via a
runtime VIO mux (`probe_out0[2]`, signal `jtag_scc_uart_sel_b`).  Default
is **chan A** (the channel MacsBug polls on the Q700 ROM's
`rom_scc_rx_poll` path):

| `vio-set` arg | `probe_out0[2]` | UART → | SCC channel |
|---|---|---|---|
| `0` (default) | 0 | A | A (`rx_a_valid`) |
| `4` | 1 | B | B (`rx_b_valid`) |

Don't forget to clear other VIO bits when toggling.

## Step-by-step

### 1. Open the host serial port

```
$ picocom -b 115200 --omap crcrlf --imap lfcrlf /dev/ttyUSB0
```

(`--omap crcrlf` translates a host CR into CR/LF on the wire and back the
other way; pick whichever flag set your terminal needs to deliver
`\r` when you press Enter.  GNU `screen` users: `screen /dev/ttyUSB0 115200`.)

### 2. Reset the CPU and watch it land in MacsBug

In a second terminal, drive the JTAG REPL:

```
# (one-time: arm the REPL — see CLAUDE.md > JTAG / Hardware Bring-up)
$ /tmp/jcmd.sh full-reset-and-halt 0
$ /tmp/jcmd.sh halt-status        # confirm pc_live is in 0x4084af9c..0x4084afca
$ /tmp/jcmd.sh "vio-set 9"        # toggle debug_full_reset (bit3) + force_cpu_reset (bit0)
$ /tmp/jcmd.sh "vio-set 0"        # release reset
$ /tmp/jcmd.sh halt-status        # again confirm pc_live in rom_scc_rx_*
```

The Q700 ROM lands in MacsBug's RX poll within ~6M instructions of POR.
`halt-status` should print a `pc_live` in the range
`0x4084af9c..0x4084afca` once the diagnostic chime + RAM walk are done.

### 3. Drive the host serial

In the picocom window, type:

```
G<Enter>
```

(That's the letter `G` followed by Enter.  `G` is MacsBug's "Go" command —
resumes the CPU from the trap that dropped us into MacsBug.)

Expected: MacsBug responds with the byte stream captured under
`make tb-scc-uart-loopback` (see [`docs/scc_lockstep.md`](scc_lockstep.md)
"known-good baseline" once that doc lands the HW capture).  In short, you
should see at minimum:

- The character `G` echoed back.
- A CR/LF.
- Either MacsBug resuming silently (if there was no MacsBug-displayed
  prompt to begin with) or a fresh `>` prompt.

### 4. If nothing comes back

Order of suspicion (sim-first, hardware-last):

1. Confirm sim-side correctness first — run `make tb-scc-uart-loopback`
   on the same checkout.  If the sim loopback does not capture a TX byte
   from the same RX inject, the bug is in RTL and HW won't fare any
   better; fix and re-bitstream.
2. Confirm `vio-set 0` has cleared the SCC reset (`pb_full_rst_bank[2]`).
   A stuck reset pins TX FIFO and RX FIFO empty.
3. Confirm CPU is actually in `rom_scc_rx_*`.  If `halt-status` shows
   `pc_live` outside that range, the CPU never entered MacsBug — fix
   the upstream boot probe issue, not the SCC.
4. Wire a logic analyzer / scope to `uart_rtl_0_rxd` and confirm the
   start bit reaches the bridge.  If it doesn't, the issue is the
   board-UART pin mapping, not the SCC.
5. With the scope, sample `pb_scc_addr / pb_scc_rd / pb_scc_rdata` on
   the JTAG AXI master — confirm the CPU is hitting the SCC RR0 / RR8
   addresses.  Compare to `docs/scc_lockstep.md` MMIO trace.

## Sim-side validation (same code path)

Before you trust the HW path, run the sim end-to-end:

```
$ MAKEFLAGS="-j1" VERILATOR_THREADS=4 VERILATOR_JOBS=4 \
  make tb-scc-uart-loopback
```

This boots the Q700 ROM through the full `fpga_top` RTL, waits for
MacsBug entry, injects `"G\r"` on the SCC's external RX byte interface,
and captures the first 64 TX bytes from BOTH the CPU side
(`scc_uart_tx_valid` / `scc_uart_tx_data`) and the post-bridge UART pin
(`uart_rtl_0_txd`).  The captures are written to:

- `build/fpga_top_rom/scc_tx.log`        — CPU-side TX bytes
- `build/fpga_top_rom/scc_tx_uart.log`   — UART-pin TX bytes

Both files use the format:

```
<retired_count> <sim_time> 0x<hex_byte> '<ascii>'
```

Latency from RX inject completion to the first CPU-side TX byte is
printed as `[scc] first_cpu_tx ... latency_cycles=...` — that's a
useful regression metric for SCC IRQ / poll-loop responsiveness.

## Known sim-vs-HW differences

- The sim runs at ~50 MHz `pb_clk` exactly (parameter `PB_CLK_HZ` =
  50000000); the HW runs at the same number from `clk_rst.v`.  A real
  HW UART byte at 115200 baud is 86.8 µs (10 bits × 8.68 µs); a sim
  byte at the bridge's BAUD parameter is the same.
- The bit-serial sim path uses a higher artificial baud (1 Mbaud
  default — see `SCC_TICKS_PER_BIT` in `tb/tb_fpga_top_rom.cpp`) for
  iteration speed.  The bridge is asynchronous so any baud the bridge
  can sample correctly works.
- The byte-fast sim path bypasses the bridge entirely and pulses the
  SCC's external `rx_valid/rx_data` byte interface directly.  This
  is NOT a valid HW path — the HW path always goes through the
  bridge — but it's a useful smoke that the SCC-internal RX FIFO and
  RR0/RR8 register paths are correct.
