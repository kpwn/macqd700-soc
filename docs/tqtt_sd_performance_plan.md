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
throughput will be lower. Standards-compliant UHS-I signaling requires a
1.8 V I/O switch that this board does not provide. A 100 MHz clock at 3.3 V
is a possible experimental overclock, not a negotiated UHS mode. It requires
a faster FPGA SD clock domain and read-only CRC testing across this card
before any write test. UHS-II's additional contacts and SD Express's PCIe
interface are absent. Do not claim those modes from the card label.

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
   An experimental full SoC variant connects native boot, SCSI, and PRAM.
   JTAG bulk card provisioning still needs a native transport; no native card
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
   To inspect the loaded engine probe without resetting it or sending a card
   command, run `tools/tqtt_sd_native_engine_status.tcl` with its matching
   `.ltx`. On 2026-10-25 the probe was still loaded, but repeated status
   samples alternated between `0x70` and `0x00` and card capacity stayed
   zero. Those samples do not establish a reliable native card result.
   Physical card reseating and the guarded read matrix remain pending.
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
   repeats nine read-only cases across the card, checks each CRC32 pair,
   and records the engine's DMA cycles and MB/s. LBA 90000 is an additional
   32-sector anchor within the previously audited `shd.hda` region.
   The same read-only engine probe can also be built with
   `make TQTT_NATIVE_SD_HS=1 fpga-tqtt-sd-native-engine-probe-bitstream`.
   Its separate `build/tqtt_sd_native_engine_probe_hs` output negotiates
   CMD6 High Speed and reports the confirmed selection in status bit 10.
   Point the matrix at that build with
   `NATIVE_SD_ENGINE_BUILD_DIR=build/tqtt_sd_native_engine_probe_hs` and set
   `NATIVE_SD_EXPECT_HS=1` to reject a silent fallback to 25 MHz. Load the
   matching bitstream and LTX before the matrix. The physical card must be
   power-cycled before the first native test; neither build step does so.
   Compare the two `summary.tsv` files with
   `python3 tools/tqtt_sd_native_matrix_compare.py <25-summary.tsv> <50-summary.tsv>`.
   Every same-range CRC must agree across speeds; the report gives the
   32-sector throughput ratio. Add `--image /home/qwertyoruiop/shd.hda` to
   annotate host-image CRC matches. Image differences are advisory because
   Mac OS can update the disk after provisioning; cross-speed differences
   are a transfer-integrity failure. The reported DMA MB/s excludes JTAG
   startup and is not Mac-visible filesystem throughput.
   The High Speed probe build completed on 2026-10-25 with WNS/WHS/WPWS
   +5.127/+0.019/+4.458 ns, zero failing timing endpoints, and zero
   bitstream DRC errors. Its bitstream SHA-256 is
   `86970fc789eced4cda252b5a75682bfdb8a2ffa6e6ee4c0148cd535f9d3854f0`.
   This is a build result, not a physical card speed or integrity result.
   The original engine probe
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
   A native PRAM transport now shares the block engine with boot and SCSI,
   retaining the SPI build's LBA 8191 sector format, checksum, boot autoload,
   and `0x50A00100` JTAG commands. `make tb-pram-sd-native` checks a full
   SAVE/LOAD sector round trip, blank-sector fallback, and saved-sector
   autoload before the boot hold releases; the SPI PRAM tests and native
   owner-mux test also pass. Native PRAM still needs hardware validation.
   The JTAG bulk card writer remains disabled in this variant; the default SPI variant
   retains it. A reset guard defers controller reset while
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
   elaboration. The bitstreams listed above predate that VIO change. A
   VIO-enabled combined 100 MHz CPU / negotiated 50 MHz SD image from staging
   commit `21d30b2` and CPU commit `4736fadf` completed on 2026-10-25:
   `/var/tmp/tqtt_native_hs_vio_100/vivado/checkpoints/route.dcp` (SHA-256
   `dcab6a3e90798f073c0e0f9fd4cf1e6788da3583fcdd1f7ff99b187823b3563c`),
   blank bitstream SHA-256
   `d549277326fb5df302e9ba5d50c1d350b7481bf9e688c49d515348493398a812`,
   and local ADB-patched bitstream SHA-256
   `7b8adf235219955b5757146a20a9ccb2a5cf08f218e39af19eb1e25da5a85a8f`.
   Final WNS/WHS/WPWS were +0.010/+0.003/+0.053 ns with zero pulse-width
   failures and zero bitstream DRC errors. Incremental routing reported four
   incomplete global-clock guidance trees, but completed routing and passed
   final timing and DRC. This image has not been loaded; card power cycling,
   native integrity tests, and High Speed negotiation readback remain pending.
   A full 100 MHz TQTT image with native PRAM, negotiated High Speed SD,
   HDMI audio, fan control, Ethernet, and reduced CPU debug completed from
   staging commit `a415ee0` and CPU commit `4736fadf` on 2026-10-25.
   Its routed checkpoint is
   `/var/tmp/tqtt_native_pram_hs_100/vivado/checkpoints/route.dcp`
   (SHA-256 `d1c28c9254c98ac5baeb40c447c285db07379cdaa811e7b05c9e9f91e469a1b6`),
   blank bitstream SHA-256
   `fcf4dfaced948b346e0b2fe3dff3d1cad086559e369422d0c752af89d7537cf5`,
   and local ADB-patched bitstream SHA-256
   `0878531c94f59982e4a56bef3f7bab2965ed5eb0edacd27bae16fdc51395c663`.
   Final WNS/WHS/WPWS were +0.021/+0.003/+0.236 ns, with zero failing
   pulse-width endpoints and zero bitstream DRC errors. Incremental route
   reported eleven incomplete global-clock guidance trees but completed
   routing and passed final timing and DRC. This image has not been loaded;
   native card integrity, actual High Speed selection, and PRAM persistence
   all still need hardware checks. Keep the RK SPI path.
   A separate native-mode TQTT JTAG provisioner is now built at
   `build/sd_provision_tqtt_native/sd_provision_top.bit` (SHA-256
   `e268eb9972404f1b21b12f12bc04ff0e1bc946c97b7224eaa864439d4b7c4255`).
   It retains the existing JTAG commands and uses 32-sector native batches.
   Post-route WNS/WHS/WPWS are +5.804/+0.018/+4.458 ns; bitstream DRC has
   zero errors. The SPI provisioning regression and native writer round trips
   pass at both 2 sectors and the maximum 32-sector batch, with independent
   expected CRC32 values. The native image has not been loaded or used to
   write a card; physical read-only integrity checks remain the prerequisite.
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

## Samsung EVO card check, 2026-10-25

After reseating the Samsung EVO card, the read-only manual native probe
initialized it at 390 kHz through ACMD41, then selected four-bit mode.
It reports 122,138,624 sectors and RCA `0x59b4`. A 32-sector read at 25 MHz
returned CRC32 `0x3f0eb0b8`. The CMD6 switch response in LiteSDCard DMA word
order is `word3=0x00000380`, `word4=0x00000001`: the card supports High Speed
and selected function 1. The old decoder used incorrect byte lanes, so the
manual script and `sd_native_hs_switch` now check word3 bits 15:8 and word4
bits 3:0. A 32-sector read at 50 MHz returned the same CRC32. The 50 MHz
manual-probe log is `/var/tmp/tqtt_native_50_corrected.log`.

The automatic initializer previously raised the clock to 25 MHz before
ACMD41 was ready. It now stays at 390 kHz until the card is ready and inserts
a 4096-cycle gap between commands. With this change, the automatic block
engine reads one sector at 25 MHz with CRC32 `0x1df5ba99`, matching the
manual probe. The first automatic multi-block attempts reached DMA, then
timed out on CMD12. A 4096-cycle pause between DMA completion and CMD12
fixed this: automatic 32-sector reads at both 25 and 50 MHz return CRC32
`0x3f0eb0b8` for LBA 0 and `0x9ff83835` for LBA 8192. At LBA 8192, the
50 MHz automatic read took 158,317 system cycles for 16 KiB, or 10.35 MB/s;
two 25 MHz runs took 199,113 and 187,969 cycles, or 8.23 and 8.72 MB/s.
LBA 0 was much slower at both speeds, around 2.2-2.3 MB/s. These are
end-to-end block-engine measurements and show card/location latency is
material. Logs are under `/var/tmp/tqtt_sd_stopgap_*.log`.

Some JTAG VIO samples are incoherent even after successful card reads: the
reported capacity becomes zero and RCA/CRC fields shift into adjacent probe
values. The read script now checks fixed capacity, RCA, init phase, and
negotiated speed before accepting a result. The 50 MHz automatic read-only
matrix passed twice at LBA 0, 8192, 90000, 95410, 250000, 500000,
1000000, and 2000000 (32 sectors each, plus one sector at LBA 0); CRCs
matched on every repeat. Results are in
`/var/tmp/tqtt_sd_stopgap_matrix50/summary.tsv`. The 32-sector results range
from 1.97 to 11.18 MB/s depending on LBA and run. Those variations need
investigation before calling the card's performance stable. Write/readback
and full-Mac tests remain. Do not write the card or load the full native Mac
image until those gates pass.

The manual probe's cycle counter includes JTAG-driven command and polling
delays; its 3.18 MiB/s figure at 50 MHz is not a sustained card bandwidth
measurement. Use the automatic engine's cycle counter for raw block-path
throughput and Mac benchmarks for user-visible throughput.

## Byte-lane correction, 2026-10-25

The first full native-SD Mac image (`13b25de`, build ID `0x13B25DE3`)
initialized the card and copied 2048 ROM sectors, but the CPU wedged on its
first reset-vector read. JTAG read `0x02860D42` at ROM base, whereas the
working SPI image read `0x420D8602`. The saved ROM image also begins with
bytes `42 0D 86 02`. The old one-sector native probe CRC `0x1df5ba99`
matches CRC32 of the saved ROM's first 512 bytes **after reversing every
four-byte group**; the original-order CRC is `0x575b1d17`. Likewise, the
old 32-sector native CRC `0x3f0eb0b8` matches the reversed-byte stream, not
the original-order CRC `0x777639f1`. Thus the earlier repeated CRC matrix
proved repeatability but did not prove correct byte order or data integrity.

LiteSDCard DMA places the first card byte in the low byte lane of each
Wishbone word. `sd_native_block_engine` now emits read bytes from lanes
7:0, 15:8, 23:16, then 31:24 and packs write bytes into those same lanes.
The directed block-engine simulation uses that bus convention and passes
one- and 32-sector reads and writes. Hardware validation remains pending:
the corrected probe must produce `0x575b1d17` and `0x777639f1` at LBA 0,
then the full Mac must boot without the reset-vector wedge. No native-SD
card writes are authorized by these read-only results.

The next full native image reuses the otherwise zero `vio_boot_crc` probe
for four saturating read handshake counts after CPU reset: CPU-D AR, CPU-D R,
DDR-port AR, DDR-port R (one byte each, high to low). If the reset-vector
wedge persists, these counts identify whether the response disappeared
before or after the DDR port without another synthesis cycle.

The corrected full TQTT image from commit `7ad1dfa` is at
`/var/tmp/tqtt_native_bytes_fixed_full_100/vivado/fpga_top.local.bit`
(local ADB firmware; SHA-256
`a70b8ed4c104e228e1923421030c6128f4c804f316cfb2ae258249c1f24c3e90`).
It uses a 100 MHz CPU, 50 MHz four-bit native SD, HDMI audio, automatic fan,
and Ethernet. Final route timing is WNS +0.015 ns, WHS +0.010 ns, WPWS
+0.073 ns with zero failing endpoints; bitstream DRC reported zero errors.
JTAG loaded it as build `0x7AD1DFA7` on 2026-10-25. The first native init
failed after switching from a SPI-mode image; a physical card power cycle
and hard reset resolved that initialization failure.

After the Samsung EVO was reseated and the board hard-reset, native High
Speed initialization and all 2048 ROM sectors completed. The ROM's first
two vectors and seven words spread through its 1 MiB image matched the
saved SPI/reference ROM. The CPU reached low-RAM Mac OS code. At the later
stall, SCSI reported 1273 completions, zero SD errors, and zero CHECK
CONDITIONs; the 53C96 trace ended after 426 consecutive pseudo-DMA data-port
writes. The CPU stopped at PC `0x000C04AA` with `AxiDMerge` waiting for a
D-cache write response and the crossbar's S1 peripheral slot still busy.
The C96 was in DATA OUT with `REQ=0`, four blocks in the active SCSI request,
and no backend busy indication. This supports a pseudo-DMA back-pressure
deadlock; the exact missing transition is not yet measured. The trace is
`/var/tmp/tqtt_native_bytes_fixed_scsi_wedge.csv`.

The next diagnostic image appends `dbg_write_flow` to the upper 32 bits of
`vio_scsi_c96` in native-SD builds, leaving its original low 84 bits intact.
The new fields are `[31:22]` write-ring occupancy, `[21:12]` bytes left in
the current sector, then flags `[11:0]`: multi-write, backend kicked,
request-go, backend busy, write-ready, target REQ, C96 DRQ, shim-write
withhold, backend done, backend error, range-check OK, request is write.

The diagnostic image from commit `9005062` is
`/var/tmp/tqtt_native_scsi_flow_100/vivado/fpga_top.local.bit` (local ADB
firmware; SHA-256
`30728d22b3bb504d74ecec8cc754ecdea90510ec4d2696d02d8690d583ef4e3a`).
Its final WNS/WHS/WPWS are +0.004/+0.005/+0.210 ns, with zero failing
endpoints and zero bitstream DRC errors. JTAG loaded it as build
`0x90050622`. Two hard-reset boots reached a different failure before the
four-block write: MacsBug reported a bus error at ROM PC `0x4081B820`
(`_vStdExit+0x56`). The faulting `CMPA.L (A2)+,A1` uses
`A2=0x02C80000`. The CPU MMU's pointer descriptor for that virtual 256 KiB
region is `0x00000018` (nonresident); the preceding region is mapped and a
JTAG physical read at `0x02C80000` succeeds. Thus this immediate bus error
is a virtual-memory mapping fault. Its cause may still be earlier data
corruption; this observation does not exonerate the SD path. The ROM also
produces some expected bus errors during boot, so halt-on-first-vector-2
alone is not a sufficient diagnosis. The capture-server still is
`/var/tmp/tqtt_snapshot_probe.jpg`.

At the MacsBug halt, SCSI was idle after 1271 completed backend commands,
zero SD errors, and zero CHECK CONDITIONs. The trace
`/var/tmp/tqtt_native_macsbug_scsi.csv` records the most recent pseudo-DMA
reads. Its read filter omits repeated values, so raw trace chunk lengths
are shorter than 512 bytes. The filtered data for five consecutive sectors
is an exact ordered subsequence of the reference `shd.hda` sectors
11735..11739 (missing only repeated-value events), including a 501-byte
trace for sector 11739 with its 11 repeated bytes omitted. This validates
those final sectors' byte order and values as delivered by SCSI, but does
not prove the entire disk read stream or explain the memory fault.

For a simpler long-term data path, reuse the native block engine's 16 KiB
BRAM as a dual-port SCSI/SD chunk buffer. The SCSI side fills an entire
bounded write chunk before the card command; reads fill from SD before the
Mac drains the chunk. A narrow cross-clock command/completion handshake and
one authoritative byte count can replace the 512-byte SCSI ring plus the
separate 128-byte native CDC FIFO. Start with serialized chunks for
correctness; add a second buffer only if measured Mac-visible throughput
justifies overlap. This preserves the Mac-facing 53C96 protocol and keeps
bulk data off the shared DDR routing fabric.

## Confirmed live write corruption, 2026-10-25

The corrected native provisioner compared disk sectors 0..95410 with
`/home/qwertyoruiop/shd.hda`. It found 102 changed sectors in 15 localized
ranges. Each pre-repair range was saved under
`/var/tmp/tqtt_card_pre_repair_20261025/`; all 15 ranges were restored and
immediately verified. A subsequent full scan of 95,411 sectors reported
zero mismatches (`/var/tmp/tqtt_native_diff_after_repair_95411.log`). A
four-sector scratch write at raw LBA 3,000,000 also read back exactly and
its original bytes were restored, proving that the provisioner's native
block writer can write correctly.

Rebooting the Mac from that clean baseline reproduced the pseudo-DMA write
stall. The 53C96 trace records a WRITE(10) to disk LBA 46700 for eight
blocks. The host supplied six complete sectors and 426 bytes of the
seventh; it was held with 86 bytes left while the backend was idle and the
512-byte SCSI ring full. Reading the card afterward showed all eight target
sectors had changed, with the first four exactly matching the prior corrupt
content. Thus the Mac's native write path corrupted a verified-clean volume
during this boot; the earlier MacsBug mapping fault alone did not explain
the damage. Trace: `/var/tmp/tqtt_native_clean_disk_write_wedge.csv`.

`scsi.v` delayed `vh_wr_avail` by one pb_clk cycle to align a staged byte
with credit in the legacy SPI bridge. The native bridge instead pushes to
its asynchronous write FIFO on *every* pb_clk with credit high. At ring
count zero, the registered credit remained high for one more cycle, so the
native bridge could take stale bytes. The new native-bridge 53C96 end-to-end
test reproduces this before the fix: three multi-block writes with producer
stalls deliver corrupt bytes to the card model, and the SCSI ring records
7..9 empty-ring drains. Ending credit immediately at count zero removes
the underflows but still corrupts bytes after each stall. At ring count one,
simultaneous producer push and FIFO drain preloads the *next* byte from the
same RAM address being written at that edge, so it reads stale data. Holding
one byte in reserve while producer and consumer overlap fixes all 27
native-bridge end-to-end cases, including the three stall cases. The credit
rising edge remains delayed to align the staged byte. Hardware retest and
post-boot card comparison are required before treating the volume as safe.

After the clean-baseline reproduction, a full scan found five changed
32-sector batches. A per-sector comparison localized these to 15 sectors:
98, 106, 2145..2146, 2240, 2243, 46700..46707, and 46840. Each 32-sector
batch is backed up under `/var/tmp/tqtt_card_post_clean_boot_20261025/`.
The 15 changed sectors were restored from the reference image and each
immediately verified. A second full-volume comparison of 95,411 sectors
reported zero mismatches
(`/var/tmp/tqtt_native_diff_after_credit_repair_95411.log`).

The fixed image from commit `de0c3f8` is
`/var/tmp/tqtt_native_credit_reserve2_100/vivado/fpga_top.local.bit`
(local ADB firmware; SHA-256
`481d006cb099ff1429e435c33e154c86feb6de4ddbbb3693f5a8176eef7eff5c`).
It keeps the same 100 MHz CPU, 50 MHz four-bit SD, HDMI audio, fan control,
and Ethernet configuration. Final WNS/WHS/WPWS are +0.012/+0.001/0.000 ns,
with zero failing endpoints and zero bitstream DRC errors. JTAG loaded it
with VIO probes as build ID `0xDE0C3F8D`. The Mac reached the System 7
desktop and its unclean-shutdown dialog, passing the prior LBA 46700 write
wedge. At the desktop the SCSI probe read 2077 completions, zero SD errors,
zero CHECK CONDITIONs, and an idle backend.

The native provisioner then compared all 95,411 disk sectors to the clean
source image. Eight 32-sector batches differed. Every changed batch was
read twice and saved under
`/var/tmp/tqtt_native_credit_reserve2_100/card_post_boot/`; the full scan
is `/var/tmp/tqtt_native_credit_reserve2_100/card_after_boot_diff.log`.
The changed sectors were overlaid on a copy of the clean image, which
mounted as an HFS volume with a current modification date and a readable
System Folder and Finder. These sectors include normal Mac OS writeback;
they were preserved on the card. This proves a successful boot and
navigable volume after native writes, but does not by itself prove every
changed byte is correct. The byte-exact regression and native block-engine
test supply the complementary data-path evidence.

Finder file duplication is still a reproducible hardware failure on this
image. Duplicating `About System 7.5` (2,336-byte data fork and 19,696-byte
resource fork), `Finder` (503,994-byte data fork), and `MacTCP DNR`
(3,659-byte data fork) each raises Finder's "couldn't be written, because a
disk error occurred" dialog. Creating a new folder succeeds. During the
`MacTCP DNR` failure, the SCSI trace
`/var/tmp/tqtt_mactcp_dup_scsi_trace.csv` captured READ(10) and WRITE(10)
of one sector at LBA 1,142,795, no REQUEST SENSE, and the live probe showed
2,782 completions, zero SD errors, and zero CHECK CONDITIONs. The trace
ring filters repeated read values, so its 430 recorded data-port reads do
**not** imply a short 512-byte transfer. `exc-ring` showed no bus-error
exception and `fault-snap` was not latched.

A control boot in MAME used a reflink copy of the reconstructed HFS image,
dismissed the same unclean-shutdown dialog, selected `About System 7.5`,
and sent Command-D. Its screen capture at 100 emulated seconds shows the
new `About System 7.5 copy 2`; `hdir` confirms matching 2,336/19,696-byte
fork lengths on the resulting disk. The control script is
`/var/tmp/tqtt_mame_control/control.lua`, its trace is
`/var/tmp/tqtt_mame_control/scsi_dup.csv`, and the result frame is
`/var/tmp/tqtt_mame_control/snap/macqd700/0004.png`. Thus the HFS image and
System 7 software can perform this copy; the FPGA's Mac-visible data path
remains unqualified. MAME's DAFB data-in DMA is not visible as CPU pseudo-DMA
register reads in that trace, so the MAME and FPGA payload read counts are
not directly comparable.
