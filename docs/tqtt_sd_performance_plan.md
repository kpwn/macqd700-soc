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
   The standalone probes now use the generated core with direct TQTT pins.
   An experimental full SoC variant connects native boot and SCSI. PRAM and
   JTAG card provisioning still need native transports; no native card
   transfer has been measured on hardware.

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
   the card's protocol selection. To avoid removing a card while Mac OS is
   using it, first load the native probe into FPGA SRAM, then reseat the
   card. Run the host script with `NATIVE_SD_NO_PROGRAM=1` to attach to that
   already loaded probe; it will initialize the reseated card and perform
   read-only transfers. After native-mode testing, power-cycle the card
   again before booting the current SPI-mode Mac image; reloading the FPGA
   alone does not return a native-mode card to SPI mode.

   Before any card power cycle, the loaded probe passed a card-independent
   hardware smoke test: `NATIVE_SD_NO_PROGRAM=1 NATIVE_SD_SMOKE_ONLY=1`
   exercised a VIO-to-Wishbone scratch write/read roundtrip on the FPGA.
   This verifies the debug/control path only, not native SD signaling.
   After reseating the card while the native probe is loaded, run the
   read-only comparison matrix with
   `NATIVE_SD_CARD_POWER_CYCLED=1 tools/tqtt_sd_native_matrix.sh`.
   It saves the card CID/CSD and individual Vivado logs, reads 32 sectors
   each at LBAs 0, 8192, and 95410 at both 25 and negotiated 50 MHz, and
   requires each same-LBA CRC32 pair to match. The reported DMA cycles
   measure transfer time in the 100 MHz probe, excluding JTAG startup.

   For full-Mac integration, TQTT already has a 100 MHz `video_ref_clk`
   from its fabric MMCM at both 100 and 200 MHz CPU settings. That can clock
   the native controller without changing its generated timing parameters.
   The existing `sd_scsi_bridge` control/reset handshake can cross the
   existing 50 MHz peripheral bus into this 100 MHz storage domain, but its
   single-byte toggle data path cannot carry native bus bursts. The
   `tb-sd-bridge-pulse` measurement duplicates data at four core cycles per
   byte (the four-bit 50 MHz bus rate); its first lossless tested spacing is
   six cycles. A separate Gray-pointer `sd_async_byte_fifo` now passes
   12,288 ordered bytes at the four-cycle rate, including reader stalls and
   an interrupted-transfer reset (`make tb-sd-native-fifo`).
   `sd_scsi_native_bridge` now wraps the established request/reset crossing
   with read and write FIFOs. Its focused test checks ordered one- and
   32-sector reads and writes, read completion only after the final byte
   reaches the SCSI side, FIFO backpressure, partial-read error propagation,
   and a core-side reset abort (`make tb-sd-native-bridge`). The experimental
   TQTT full SoC variant connects this wrapper to SCSI and the shared card
   backend. The optional native `boot_fsm` source uses a second FIFO CDC and
   the existing AXI ROM writer to copy the ROM into DDR.
   A one-outstanding `sd_native_csr_master` now provides bounded Wishbone
   CSR transactions for the hardware command sequencer. Its test
   covers reads, writes, bus errors, timeouts, and reset cancellation
   (`make tb-sd-native-csr`). `sd_native_command` builds on it to write a
   command argument/type/send sequence, poll the card event with a bound,
   and return all 128 response bits. Its test covers normal completion,
   card errors, event timeout, and CSR error (`make tb-sd-native-command`).
   `sd_native_init` sequences native 3.3 V SDHC/SDXC initialization through
   four-bit default-speed mode and decodes CSD v2 capacity. Its unit test
   exercises OCR retries and voltage, CSD, and CSR failures
   (`make tb-sd-native-init`). The experimental full SoC variant instantiates
   these modules through `sd_native_backend`. `sd_native_init_controller`
   connects the init FSM, command engine, and CSR master to one LiteSDCard
   Wishbone port; its
   integration test checks the real CSR transaction order and failure paths
   (`make tb-sd-native-init-controller`). The small TQTT hardware image
   `make fpga-tqtt-sd-native-autoinit-probe-bitstream` routed at 100 MHz with
   WNS +6.541 ns, WHS +0.011 ns, and zero DRC errors on 2026-10-25. Its
   VIO reports card capacity, RCA, init phase, and error, but it has not yet
   run against the physical card. The existing manual read probe remains
   loaded for the card's first native transfer test. DMA block commands
   are now also sequenced by `sd_native_block_engine`, with a 16 KiB local
   DMA RAM and byte-stream interfaces for one- or 32-sector reads/writes.
   Its behavioral LiteSDCard test checks DMA register programming, byte
   order, full transfer counts, CMD17/18/24/25 and CMD12 encodings, and
   completion (`make tb-sd-native-block`). A standalone read-only image now
   connects this engine to the hardware init controller and LiteSDCard core:
   `make fpga-tqtt-sd-native-engine-probe-bitstream`. On 2026-10-25 its
   100 MHz route had WNS +5.001 ns, WHS +0.010 ns, and zero bitstream DRC
   errors. The artifact is
   `build/tqtt_sd_native_engine_probe/tqtt_sd_native_engine_probe.bit`,
   SHA-256 `dafee1f71f20cc50040c10b2ec2e0b9a0fc79140d1ba140f485067d16aa504c9`.
   After physically reseating the card, use the read-only
   `tools/tqtt_sd_native_engine_read.tcl` with
   `NATIVE_SD_CARD_POWER_CYCLED=1` to check card initialization, 1..32
   sectors, CRC32, and DMA timing. After the first read succeeds,
   `NATIVE_SD_CARD_POWER_CYCLED=1 tools/tqtt_sd_native_engine_matrix.sh`
   repeats eight read-only cases across the card, checks each CRC32 pair,
   and records the engine's DMA cycles and MB/s. The original engine probe
   was loaded in FPGA SRAM on 2026-10-25; Vivado detected its VIO and its
   automatic init reported an error before the card was reseated. This is
   not a native card transfer result. It was reloaded after the audio
   hardware test using
   `tools/tqtt_sd_native_engine_load.tcl`; this loader issues no card
   command and leaves the SPI flash untouched. The read script pulses the
   probe's soft reset after the card power cycle so initialization can be
   retried. The current source factors
   that same init, block RAM, and LiteSDCard wiring into `sd_native_backend`
   for reuse in the full Mac. The refactored probe also routed at 100 MHz
   (WNS +5.519 ns, WHS +0.022 ns, DRC zero errors) in the separate
   `build/tqtt_sd_native_engine_probe_backend` output directory, SHA-256
   `88d70d4bf4fe48dc38e69cc79df65e755d791e2fe8887ffadf988f8a781f40a2`.
   It is not the image loaded in SRAM. The experimental full SoC variant uses
   the same backend source, without physical card validation yet.
   `boot_fsm` now has an optional `TQTT_NATIVE_SD` byte-stream source that
   retains its existing AXI ROM writer and RAM/VRAM zeroing. It requests the
   ROM in at most 32-sector chunks, back-pressures the byte stream when its
   AXI word queue fills, and refuses to boot on native initialization or
   capacity failure. `make tb-sd-boot-native` checks a 40-sector two-chunk
   copy under AXI response delay, all 5,120 output words, byte-stream stalls,
   and both failure guards. The existing SPI boot suite still passes 12/12.
   The experimental full SoC variant connects native boot across the 100 MHz
   storage-to-core CDC and gives it exclusive access to the backend during
   ROM load. `sd_native_owner_mux` now handles the boot-to-SCSI
   ownership change and rejects requests from the inactive side; its focused
   test checks both byte-stream directions and in-flight owner retention
   (`make tb-sd-native-owner`). `sd_native_scsi_chunker` now translates SCSI
   requests longer than the backend's 32-sector RAM window into consecutive
   commands while keeping one SCSI completion. Its test covers 65-sector
   reads and writes, a later-chunk failure, and invalid requests
   (`make tb-sd-native-chunker`). The block engine also bounds idle time while
   filling its write RAM or draining a completed read, so an interrupted
   producer or consumer reports an error instead of holding SCSI busy forever
   (`make tb-sd-native-block`). A multi-block data error now makes a bounded
   CMD12 stop attempt before the error is returned; the same test checks
   both read and write error paths. An experimental full-SoC TQTT source
   variant now selects native pins and connects the backend through boot
   and SCSI FIFO CDCs with `TQTT_NATIVE_SD=1`. A combined 100 MHz TQTT
   implementation with HDMI audio, automatic fan control, Ethernet, reduced
   CPU debug, and native SD completed on 2026-10-25. The routed checkpoint is
   `/var/tmp/tqtt_native_easc_100/vivado/checkpoints/route.dcp` (SHA-256
   `c4bdbe9c32556bebcaa22bf662bb7c5971dd3345f451a86b79f6d93f9a42697b`).
   Its blank bitstream is `/var/tmp/tqtt_native_easc_100/vivado/fpga_top.blank.bit`
   (SHA-256 `36cc7206e8f1e6b791403a29dab15cd0b281e5731a6e36f3ee188c1f9ebc30cd`).
   Final WNS/WHS/WPWS were +0.005/+0.005/+0.053 ns with zero failing endpoints
   and zero bitstream DRC errors. The source RTL was staging commit `7540ebe`
   with CPU commit `4736fadf`; the later build-flow correction changed no RTL.
   A local ADB-patched image is at `fpga_top.local.bit` in the same directory
   (SHA-256 `c535770b58f16cb06a169bbbfe2a1136e9438fbbbdd8ce9bc386ce6d35fef458`).
   The original 25 MHz route checkpoint was inadvertently replaced while
   using it as the incremental reference for the 50 MHz build; it was
   regenerated from its saved place checkpoint with identical timing and
   zero DRC errors. The earlier blank/local bitstreams are retained under
   `/var/tmp/tqtt_native_easc_100/pre_recovery_artifacts/`. The current
   bitstream has not been programmed. It still needs native-card
   read/write integrity tests and 200 MHz timing before replacing the SPI
   path.
   The full-top Vivado RTL dry-run passed with HDMI audio, fan control, and
   Ethernet selected on 2026-10-25. The SD J8 constraints are separate
   `tqtt_sd_spi.xdc` and `tqtt_sd_native.xdc` files selected by Vivado Tcl;
   Vivado XDC rejected a conditional in one file. The native CDC data RAM
   write is separate from asynchronous pointer resets to permit RAM
   inference; `make tb-sd-native-fifo tb-sd-native-bridge` still pass.
   The SPI-only PRAM persistence and JTAG card writer are terminated in
   this variant until their native equivalents exist; the default SPI
   variant still has them. A reset guard defers controller reset while
   an on-card command is active (`make tb-sd-native-reset`). The negotiated
   50 MHz CMD6 switch now has an opt-in `TQTT_NATIVE_SD_HS=1` implementation.
   After four-bit initialization at 25 MHz, `sd_native_hs_switch` sends CMD6
   with a 64-byte DMA transfer, verifies the support and selected-function
   bytes, and only then writes the divider for 50 MHz. A card declining High
   Speed stays at 25 MHz. `make tb-sd-native-hs` checks confirmed selection,
   fallback, missing status data, command/CSR errors, and timeout. Full-top
   native+HDMI-audio+fan+Ethernet Vivado RTL elaboration passed with the
   option on. A full 100 MHz CPU / negotiated 50 MHz SD candidate then routed
   on 2026-10-25 from staging commit `faa4e3a` and CPU commit `4736fadf`:
   `/var/tmp/tqtt_native_hs_easc_100/vivado/checkpoints/route.dcp` (SHA-256
   `75de7a8960b62f09c087d612caf69444a1caeaa5479bef398ec4fa572ec5e074`),
   blank bitstream SHA-256
   `263f93135f43c6efb4a0cbc427c55bcde334dc1f7207e057285a387e2601c133`,
   and local ADB-patched bitstream SHA-256
   `cf3faf6af073e087e07e97f5e4a0ccdb81387f8c97ecc049a2a912b35aa96a27`.
   Final WNS/WHS/WPWS were +0.020/+0.005/+0.053 ns, with zero failing
   endpoints and zero bitstream DRC errors. No 50 MHz card test or board load
   exists yet. The default remains 25 MHz until physical card integrity
   tests pass. A later source change appends the actual negotiated High Speed
   state as bit 187 of `vio_scsi_sd`, preserving all older bit positions;
   `tools/scsi_sense_probe.sh` decodes it. That observability change passed
   TQTT native-SD Verilator lint in both modes and TQTT Vivado RTL
   elaboration. The bitstreams listed above predate that VIO change, so
   they cannot report the new bit without a rebuild.
   PRAM and JTAG provisioning also need native-mode paths before the SPI
   transport can be replaced in a full image. Keep the RK SPI board path.
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
