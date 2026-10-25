# TQTT SD performance requirement

The TQTT bring-up is not complete until the SD-backed Mac disk has been
measured and tuned for the fastest reliable mode of the board as built.
Measure both raw card transfers and Mac-visible reads/writes; report latency,
sustained throughput, CRC/data-integrity results, and the exact card/image.

## Electrical ceiling

The board schematic (page 4, J8) wires CLK, CMD, and DAT0-DAT3 to FPGA bank
84. J8 VDD, the line pull-ups, and bank 84 VCCO are fixed at 3.3 V. Native
SD pins are CLK AE15, CMD AD15, DAT0 AF13, DAT1 AE13, DAT2 AF15, and
DAT3 AF14; J8 has 4.7 kOhm pull-ups on CMD and every data line.
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

On 2026-10-24, the TQTT provisioning image read the card with repeated CRC32
checks. The card's Apple partition map describes a 2,252,896-block image,
matching `/home/qwertyoruiop/shd.hda`, rather than the 500 MB 7.5.3 reference.
A read-only comparison of disk blocks 0..95410 against `shd.hda` found 10
changed 256-block batches; sector dumps narrowed those to 102 changed blocks.
The card re-booted Mac OS afterward. Those changes may be normal filesystem
updates and do not establish corruption. The Finder copy failure still needs
an isolated write-path reproduction and HFS consistency check.

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
   `tools/tqtt_sd_25_50_integrity.tcl` automates repeated read-only 25/50 MHz
   loads of the 2048-sector boot window on the 200 MHz probe. It requires the
   matching `.bit` and `.ltx`, checks that CMD6 MODE=1 actually selected the
   High Speed function in the card's 64-byte status, rejects
   whole-run retries, and compares CRC32 across runs. Passing this first
   window does not qualify the full card or writes; extend the probe to long
   and randomized LBAs before changing the Mac's SD clock.

   On 2026-10-25 the TQTT card passed two complete runs per mode: 2048 sectors,
   zero whole-run retries, matching final block CRC16, and the same full-window
   CRC32 `0xFF52A648` at 25 MHz and at negotiated 50 MHz. The High Speed mode
   flag was set only in the 50 MHz runs. Each run took about 12.4–12.9 seconds
   including card initialization and JTAG polling, so that elapsed value is
   not a data-transfer throughput measurement. VIO readback gave inconsistent
   intermediate snapshots during active transfer; the script accepts only
   two matching completed snapshots and times out stalled runs.

   ```sh
   make fpga-tqtt-sdmin-50mhz-bitstream
   vivado -nojournal -nolog -mode batch \
     -source tools/tqtt_sd_25_50_integrity.tcl -tclargs \
     build/vivado_sdmin_tqtt_50/fpga_top_sdmin.bit \
     build/vivado_sdmin_tqtt_50/fpga_top_sdmin.ltx 5
   ```
3. Implement native four-bit SD initialization and transport for the same
   block-level boot and SCSI consumers. Native mode must start at boot:
   a card initialized into SPI mode cannot switch to native mode without
   resetting its power. Preserve bounded requests, back-pressure,
   CRC/checking, error propagation, and write coherency.
   [LiteSDCard](https://github.com/enjoy-digital/litesdcard) is a BSD-2-Clause
   candidate for this transport: release 2024.12 generates a standalone
   32-bit Wishbone control/DMA Verilog core with one CMD and four DAT lines,
   plus CRC handling. An isolated generator run produced a 246 KB core and
   CSR map.
   Integration still needs a block-command bridge for the existing boot and
   SCSI consumers and direct TQTT IO constraints. The generated core has not
   been integrated or tested here.

   A standalone native probe is now in progress on branch
   `codex/tqtt-native-sd-20261025`. It vendors the generated LiteSDCard
   2024.12 core and places a VIO-controlled Wishbone master in front of its
   CSRs and a 16 KiB local RAM on its DMA bus. `make
   fpga-tqtt-sd-native-probe-bitstream` builds it. The read-only host script
   `tools/tqtt_sd_native_read_probe.tcl` initializes a card in native mode,
   selects four-bit bus width, optionally switches to 50 MHz after checking
   CMD6 status, then reads up to 32 sectors with CRC32 and hardware DMA
   cycle timing. This is a probe, not yet the full Mac storage transport.
   The corrected Vivado route on 2026-10-25 met the 100 MHz fabric constraint
   with WNS +2.897 ns and WHS +0.017 ns, and bitstream DRC had zero errors.
   The artifact is `build/tqtt_sd_native_probe/tqtt_sd_native_probe.bit`,
   SHA-256 `60edf295fc267c204b0b83fb52a0bf3f0875928219dafed4dda888df4d2104d9`.
   No native-mode card measurement has been made.
   The card must be physically power-cycled after any SPI-mode image before
   native-mode hardware testing; FPGA reconfiguration alone does not reset
   the card's protocol selection.
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
