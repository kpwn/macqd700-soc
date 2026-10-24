# TQTT SD performance requirement

The TQTT bring-up is not complete until the SD-backed Mac disk has been
measured and tuned for the fastest reliable mode of the board as built.
Measure both raw card transfers and Mac-visible reads/writes; report latency,
sustained throughput, CRC/data-integrity results, and the exact card/image.

## Electrical ceiling

The board schematic (page 4, J8) wires CLK, CMD, and DAT0-DAT3 to FPGA bank
84. J8 VDD, the line pull-ups, and bank 84 VCCO are fixed at 3.3 V. Native
four-bit SD High Speed at 50 MHz has a 25 MB/s raw bus ceiling; actual card
throughput will be lower. UHS-I signaling requires a 1.8 V I/O switch that
this board does not provide. UHS-II's additional contacts and SD Express's
PCIe interface are absent. Do not claim those modes from the card label.

## Current implementation and measured problem

The full Mac uses one-bit SPI. At a 200 MHz core clock the SCSI transport's
`HS_HALF=4` produces a 25 MHz SD clock, or 3.125 MB/s raw bus capacity.
The boot transfer also uses 25 MHz. The 50 MHz SCSI setting had produced CRC
failures on the earlier board/card path, so the current RTL conservatively
scales the divider; TQTT's 50 MHz reliability has not been established.

Earlier same-card measurements in `rtl/soc/vhdd_readahead.v` were about
13.6 single-block SCSI reads/s against 3,800 sectors/s for a long CMD18
transfer. The roughly 70 ms per-command gap dominated the raw clock rate.
The current full image has a two-way, 32-block adaptive read-ahead cache,
but its actual TQTT Mac-visible throughput has not been measured. Do not use
the cache's simulated projections as a hardware result.

MacBench's disk suite has historically failed on this system, including
before TQTT. Its error 5050 is not a TQTT regression signal. Use a separately
validated Mac-visible disk workload for throughput and integrity. A Finder
file duplication on the current TQTT image also reported a disk error; read
the card back and check the volume before assigning a cause to that failure.

## Work sequence

1. Establish a reproducible baseline on the current TQTT 200 MHz image:
   raw sequential and random SD read/write throughput, command latency,
   read-ahead hit/miss behavior, and a validated Mac-visible disk workload. Include
   SD error/retry counters and a host-side sector comparison after writes.
2. Build a standalone TQTT 200 MHz SPI probe at 25 and 50 MHz. Use long
   sequential and randomized reads, CRC16 on every block, and repeated
   resets. Only use 50 MHz in the Mac image if this passes on the actual card.
   The first experiment is `make fpga-tqtt-sdmin-50mhz-bitstream`; its
   `build/vivado_sdmin_tqtt_50` image runs the SD engine at 200 MHz and
   offers 25 MHz fast / 50 MHz negotiated HS. In `jtag_repl_sdmin.tcl`,
   set `ctrl 5` then `ctrl 4` to assert/release soft reset while requesting
   HS after CMD6. Read sector count, retry/error flags, per-block CRC, and
   whole-image CRC. Compare the latter against a fresh 25 MHz run on the
   same card, rather than assuming a host ROM file matches the card's
   current contents. The old 100 MHz image remains the 25 MHz reference.
3. Implement native four-bit SD initialization and transport for the same
   block-level boot and SCSI consumers. Native mode must start at boot:
   a card initialized into SPI mode cannot switch to native mode without
   resetting its power. Preserve bounded requests, back-pressure,
   CRC/checking, error propagation, and write coherency.
4. Test 25 MHz and 50 MHz four-bit modes separately. Verify long reads,
   writes, random LBAs, card removal/reset recovery, and full Mac boot.
   Measure transfer and command overhead at each mode; retain the fastest
   mode that passes integrity checks.
5. Re-run Mac-visible benchmarks and storage integrity checks, and record
   the selected mode, achieved throughput, and any remaining software or
   SCSI bottleneck. Keep RK's validated SPI path selectable.

The [SD Association bus-speed table](https://www.sdcard.org/developers/sd-standard-overview/bus-speed-default-speed-high-speed-uhs-sd-express/)
defines the 12.5 MB/s default-speed and 25 MB/s High Speed ceilings; those
figures are bus rates, not application throughput.
