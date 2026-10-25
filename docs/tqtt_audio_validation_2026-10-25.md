# TQTT audio validation, 2026-10-25

The 100 MHz TQTT image built from `codex/tqtt-audio-original-asc-20261025`
(`594a2b8`) plays consecutive Mac OS 7.5.3 Sound control panel alerts over HDMI.
It was programmed into FPGA SRAM only. The verified 200 MHz SPI flash image was
not changed.

## Reproduction and evidence

All captures use the USB HDMI capture card's ALSA `hw:1,0` input, 48 kHz,
signed 16-bit stereo. Images came from the running MJPEG server's snapshot
endpoint. The captures are local artifacts and contain no firmware image.

| Image | Sound panel result | Evidence |
| --- | --- | --- |
| Flashed 200 MHz baseline `0xB9BE30AF` | Clicking Basil leaves the Sound Manager waiting at PC `0x000862f6`; HDMI capture is digital zero | `/var/tmp/tqtt_audio_flash_baseline_bendme.wav` |
| 100 MHz 44.1 kHz FIFO trial `0x6930EAC4` | Same wait at PC `0x000862f6`; Basil capture is digital zero in both channels | `/var/tmp/tqtt_audio_fifo_basil.wav`, `/var/tmp/tqtt_audio_fifo_basil.jpg` |
| 100 MHz original ASC trial `0x594A2B8F` | Basil plays about 2.50 s of nonzero stereo PCM; the PC subsequently leaves the wait loop. Bend Me can then be selected and plays another distinct about 2.32 s stereo burst | `/var/tmp/tqtt_audio_original_basil.wav`, `/var/tmp/tqtt_audio_original_bendme.wav`, `/var/tmp/tqtt_audio_original_bendme.jpg` |

The passing image implements the original ASC version-zero FIFO status, mode,
and IRQ behavior. Its final routed timing is WNS `+0.023 ns`, WHS `+0.010 ns`,
and WPWS `+0.053 ns`; all user constraints are met and bitstream DRC has zero
errors. `make tb-asc-original` and `make tb-asc` pass. The CPU worktree's
`make SBT='/home/qwertyoruiop/sbt/bin/sbt --mem 6000' test-fast` gate passed
with 441 succeeded, zero failed, one canceled, and two ignored.

## Startup chime investigation corrected

The passing version-zero image produces a 0.292 s startup burst in
`/var/tmp/tqtt_audio_original_boot.wav`. The earlier 0.60 s MAME Q700
reference is **not a like-for-like comparison**. A live MAME register trace
(`/var/tmp/audio_mame_baseline_asc.tsv`) confirms that Q700 detects EASC
version `0xB0`, configures SRC with `F08/F28=0x82`, and feeds 4-bit CD-XA
data into both FIFOs. The version-zero RTL takes the separate original ASC
wavetable path. The prior CPU speed hypothesis and proposed ROM delay patch
therefore do not explain the difference. A local MAME ROM patch increasing
the version-zero chime delay table values left the first two seconds of the
EASC WAV byte-identical.

The EASC CD-XA path is now implemented on `codex/tqtt-audio-easc-20261025`.
A volatile 100 MHz TQTT image built from that branch passed routed timing
(WNS `+0.023 ns`, WHS `+0.010 ns`, WPWS `0.000 ns`, zero failing endpoints)
and bitstream DRC (zero errors). The ADB-patched test image is
`/var/tmp/tqtt_audio_easc_100/vivado/fpga_top.local.bit`; its JTAG build ID
is `0x22D1A7D9`. `make tb-asc-easc`, `make tb-asc-original`, and
`make tb-asc` pass. SPI flash was not changed.

On hardware, the EASC image reached Finder. The startup chime in
`/var/tmp/tqtt_audio_easc_boot.wav` has nonzero stereo content from 21.54 to
22.92 s of the capture (1.38 s). The live MAME EASC baseline
`/var/tmp/audio_mame_baseline.wav` has a first chime envelope from 0.33 to
1.69 s (1.36 s). Their gain-normalized averaged 4K-point FFT power spectra
have cosine similarity 0.99931. The prominent bins at 58.6, 70.3, 257.8,
and 328.1 Hz agree. The earlier `/var/tmp/mame_q700_boot_audio.wav` 0.62 s
clip has a very different, DC-dominated spectrum and is not a valid chime
reference for this EASC trial. The cause of its difference from the newer
MAME run is not established.

The Sound control panel opened on the EASC image. Basil produced 2.51 s of
nonzero stereo HDMI PCM in `/var/tmp/tqtt_audio_easc_basil.wav`; Bend Me then
produced a distinct 2.34 s stereo burst in
`/var/tmp/tqtt_audio_easc_bendme.wav`. The panel remained responsive and
advanced its selection. Screenshots are
`/var/tmp/tqtt_audio_easc_basil_done.jpg` and
`/var/tmp/tqtt_audio_easc_bendme_done.jpg`. The previous version-zero branch
remains an independently validated fallback.
