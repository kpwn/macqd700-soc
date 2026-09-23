# SoC architecture

The SoC implements the Quadra 700 machine around the out-of-order 68040 in
the pinned `cpu040` submodule. `CPU=stub` is an integration-test option, not
an alternative description of the production CPU.

## Boundaries

| Location | Responsibility |
|---|---|
| `cpu040/` | CPU pipeline, MMU, L1 caches and CPU debug state |
| `rtl/mac/` | Macintosh-visible peripherals and register behavior |
| `rtl/soc/` | CPU integration, AXI fabric, L2, boot and storage glue |
| `rtl/board/` | KU5P clocks, DDR4, SD transport, video and audio interfaces |

The top level is [fpga_top.v](../rtl/soc/fpga_top.v); its `.vh` files divide
the wiring by subsystem. [cpu_socket.vh](../rtl/soc/cpu_socket.vh) defines
the CPU boundary: 32-bit physical addresses, 4-bit AXI IDs, 128-bit data
traffic and a separate 256-bit instruction-fetch port. CPU implementation
details belong in the CPU repository.

## Memory paths

RTC parameter RAM uses block RAM with a bounded explicit-clear sweep; see
[PRAM storage](pram-storage.md) for reset, serial-port and SD-persistence rules.

With L2 and DDR-backed VRAM enabled:

```text
CPU data / boot / host debug -> AXI crossbar S0 -> L2 ----+
CPU instruction fetch --------------------------> L2   |
                                                       +-> VRAM/DDR arbiter
AXI crossbar S3 -> VRAM pixel adapter ------------------+       |
Video scanout -----------------------------------------+       v
                                                        CDC -> MIG -> DDR4
```

The direct fetch port avoids narrowing instruction traffic through the
128-bit crossbar. The data-side CPU and boot loader share a master port;
the CPU remains held during boot loading. The host debug path has its own
master port. Ethernet DMA integration is wired in
[fpga_top_ethernet.vh](../rtl/soc/fpga_top_ethernet.vh).

The crossbar's slave destinations are:

| Port | Destination |
|---|---|
| S0 | Cacheable DDR-backed memory through L2 when enabled |
| S1 | Macintosh peripheral bus |
| S2 | DMA configuration registers |
| S3 | VRAM pixel aperture |
| S4 | DAFB display registers |
| S5 | SD/debug service aperture, where instantiated |

Address decode and DDR backing offsets are defined in
[axi_defs.vh](../rtl/soc/axi_defs.vh), not by the diagram. In particular,
the ROM has a 1 MiB mirror period, VRAM is at `0xF9000000`, and DAFB
registers are at `0xF9800000`. The RAM decode window is not a statement of
the amount of RAM reported to Mac OS. ROM folding must agree on the
crossbar and direct instruction-fetch paths.

[fpga_top_ddr.vh](../rtl/soc/fpga_top_ddr.vh) connects L2, the independent
VRAM/scanout lanes and the DDR controller. The board-side
[MIG bridge](../rtl/board/axi_ddr4_mig_bridge.v) adapts the fabric's
128-bit data / 6-bit IDs to the MIG's 256-bit data / 1-bit ID interface.
Its default outstanding budgets are eight reads and four writes. See the
[fabric contract](fabric_concurrency_contract.md) for ordering requirements.

## Macintosh I/O

The peripheral bus hosts VIA1/2, SCC, SCSI, sound, floppy, RTC and ADB
logic. Address aliases and side effects are implemented by
[glue.v](../rtl/mac/glue.v) and the individual `rtl/mac` modules. Multi-hot
peripheral writes are serialized by the integration fabric.

SCSI presents a 53C96-compatible controller backed by SD storage. Its
pseudo-DMA register transfers are not a DMA engine transferring disk data
directly into RAM. The shipped storage configuration uses CMD24 writes;
the experimental staged CMD25 path is disabled by default.

The ADB modem executes a PIC program in one BRAM. Releases leave it empty;
users [insert their own firmware](adb_firmware_bitstream.md). The Quadra
boot ROM and disk image instead reside on SD. DAFB controls framebuffer
geometry and color state; the board video path produces the physical output.

## Clocks, boot and reset

The release's `.buildinfo` records its actual configuration. The 200 MHz
build has a 200 MHz core/fabric clock and a 50 MHz peripheral clock;
MIG, video and Ethernet have their own clocking. The PIC fetches on the
peripheral clock's falling edge and executes on its rising edge, so those
paths retain their half-cycle constraints.

[fpga_top_clocks.vh](../rtl/soc/fpga_top_clocks.vh) defines clock and reset
distribution. AXI bridges and explicit synchronizers cross domains; a
reset is not permission to discard an already accepted transaction.
The peripheral S1 path stays alive across `soc_full_rst` so outstanding
peripheral transactions can complete. Cache and DDR recovery must preserve
response ownership until accepted operations have drained.

The boot FSM waits for memory readiness, loads the SD-resident ROM into
DDR, and then releases the CPU. Boot readiness, reset state and debugger
halt state are distinct. A debugger halt does not reset the machine.
