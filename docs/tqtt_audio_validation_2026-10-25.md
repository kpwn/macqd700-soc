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

## Startup chime still needs work

The same passing image produces a startup burst from 14.772 to 15.064 s in
`/var/tmp/tqtt_audio_original_boot.wav` (about 0.292 s). A local MAME Q700
reference capture, `/var/tmp/mame_q700_boot_audio.wav`, has appreciable
musical content for roughly 0.60 s. Both contain pitch components near
261, 522, and 695 Hz, so the ASC sample cadence appears close. The FPGA
wavetable envelope decays to a flat level by about 0.20 s while MAME's decays
over roughly 0.60 s. This points to the ROM's CPU-paced chime loop running
faster on the FPGA; it does not establish a faulty audio sample divider.

The ROM writes four wavetable bytes and then delays in the `DBF D4` loop at
`0x40807118` before eventually writing `R_MODE=0` at `0x40807152`.
Disassembly shows `D4` loaded from table `D0`; the version-dependent table
values are 39 at ROM offset `0x71b0` and 43 at `0x71c8`. Increasing these
local ROM table values could stretch the chime without changing steady-state
audio. Such a test would need ROM checksum compensation and must stay local to
the user's ROM dump. The existing simulation-only `chime-delay` patch in
`tb/models/rom_patch_sets.h` removes the delay and is not a chime fix.

A boot-only CPU throttle limited to the chime routine is another possible
approach, but it would change CPU/debug clocking or execution control and
needs a separate design review. Neither approach has been implemented or
validated. The passing Sound panel branch leaves the chime gap open.
