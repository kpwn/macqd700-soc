# MAME-vs-RTL DAFB register lockstep

Byte-for-byte parity between MAME's `dafb_device` (in `macqd700`) and our
`rtl/mac/video.v` shim plus the TurboSCSI register window in
`rtl/mac/scsi.v` / `rtl/sys/peripheral_bus.v`.  Used to localise the
first DAFB-related transaction where the RTL falls out of register-level
parity with MAME, register-by-register, byte-by-byte.

This is the DAFB sister of `docs/axi_lockstep.md` — same capture/diff
pattern, narrower address filter so VIA / SCC / IWM noise doesn't
dominate the trace.

## Why this lockstep

The DAFB shim has historically been a "stored register file plus a few
hand-coded read-back tweaks".  As MAME's reference `dafb_device` evolved,
several gaps opened:

* `0xF980_0000..0xF980_002F` — extra control-register read-backs MAME
  computes from internal state (FB base/stride encoding, monitor sense
  inverse, SCSI mirrors, version/test register).
* `0xF980_0100..0xF980_01FF` — Swatch timing / VBL block; MAME treats
  this as a 12-bit-wide field with sequence-detected mode-switch
  semantics.
* `0xF980_0200..0xF980_02FF` — AC842 RAMDAC byte registers; the Q700
  ROM only writes the palette address + PCBR, so full CLUT support is
  optional for the boot path.
* `0xF980_0300..0xF980_03FF` — DP8531 clock generator; also poked
  during init.
* `0x5000_F100..0x5000_F101` — TurboSCSI 16-bit DMA shim.

The lockstep capture turns each of these into "first divergent event,
±5 lines context", which is much cheaper to chase than a screenful of
register reads.

Address filter — what's in scope:

| Range                              | In scope? | Reason                                  |
|------------------------------------|-----------|-----------------------------------------|
| `0xF980_0000..0xF980_03FF`         | YES       | DAFB register file (dafb_r/w + swatch_r/w + ramdac_r/w + clockgen_r/w) |
| `0xF900_0000..0xF91F_FFFF`         | optional  | VRAM aperture (`MAME_DAFB_INCLUDE_VRAM=1` / `+dafb_lockstep_include_vram` to enable) |
| `0x5000_F000..0x5000_F0FF` + mirror| YES       | TurboSCSI register window               |
| `0x5000_F100..0x5000_F101` + mirror| YES       | TurboSCSI 16-bit DMA handshake          |

The MAME ROM uses the bitwise-OR mirror family for the
`0x5000_xxxx`-range accesses (`0x50F0_F0xx`, `0x50F4_F1xx`, …); the diff
tool strips `0x00FC_0000` from MAME-side addresses inside the
`0x5000_xxxx` window before comparing, so a captured `0x50F0_F0XX`
event aligns with the canonicalised RTL `0x5000_F0XX` event.

## How the captures work

* **MAME side** (`tools/mame_dafb_capture.lua`, loaded with
  `-autoboot_script`):

  1. Optional ROM patches read from `$MAME_DAFB_PATCH_FILE` are
     applied to MAME's in-memory `:bootrom` region — the on-disk ROM
     is untouched so the MAME checksum still passes.  The wrapping
     `make tb-dafb-lockstep` target writes the patch file before
     launching MAME.  Default patch set is **empty** — the standard
     `mame-fastdiag` stack jumps past the DAFB init, so the unpatched
     ROM is the one that exercises the DAFB.
  2. Two read+write taps are installed:
     `0xF900_0000..0xF980_03FF` (DAFB regs + VRAM) and
     `0x5000_0000..0x50FF_FFFF` (TurboSCSI mirror window).  The
     callback filters to the apertures listed above and emits one CSV
     line per byte / half-word / longword access, decoding the
     handler-level `(offset, mask)` pair into architectural-size
     events.
  3. Output goes to `$MAME_DAFB_TRACE_OUT` in CSV form:
     `<seq>,<R|W>,<addr_hex>,<size_bytes>,<data_hex>`.

* **RTL side** (`tb/tb_fpga_top_rom.cpp::main` with
  `+dafb_lockstep_log=<path>`):

  1. The same per-cycle M0 / M3 AXI sniffer that drives the AXI
     lockstep capture also emits to the DAFB log when the address
     passes the DAFB / TurboSCSI scope check.  No new sniffing logic
     — the capture function (`axi_emit`) forks to whichever of the
     two log handles is open, with the matching scope filter.
  2. CSV output matches MAME's format exactly so
     `tools/dafb_lockstep_diff.py` can byte-diff.

* **Diff** (`tools/dafb_lockstep_diff.py`):
  reads both CSVs, canonicalises mirror bits in the `0x5000_xxxx`
  window, walks them in lock-step on `(rw, canon_addr, size, data)`
  tuples (the `<seq>` column is reindexed in load order), exits
  non-zero on the first divergence with a ±5-event context window
  printed for each side.

## Run

```bash
# 1. Sanity-check MAME's macqd700 ROM set (one-time, same as iwm-lockstep)
mkdir -p /tmp/mame_iwm/roms/{macqd700,adbmodem}
cp files/420dbff3.rom               /tmp/mame_iwm/roms/macqd700/
cp /home/qwertyoruiop/342s0440-b.bin /tmp/mame_iwm/roms/adbmodem/342s0440-b.bin
mame -rompath /tmp/mame_iwm/roms -verifyroms macqd700

# 2. End-to-end (default patch_set = "" — unpatched ROM):
make tb-dafb-lockstep

# 3. Tune coverage / patch set (if needed):
make tb-dafb-lockstep \
     DAFB_LOCKSTEP_PATCH=chime-skip \
     DAFB_LOCKSTEP_MAX=16000 \
     DAFB_LOCKSTEP_SECONDS=6
```

Outputs (under `build/dafb_lockstep/`):

| File                  | Purpose                                          |
|-----------------------|--------------------------------------------------|
| `dafb_mame.csv`       | MAME-side capture                                |
| `dafb_rtl.csv`        | RTL-side capture                                 |
| `patches.txt`         | applied byte-patch list (empty by default)       |
| `mame.log`, `rtl.log` | run logs                                         |

## Variables

| Variable                       | Default               | Purpose                                              |
|--------------------------------|-----------------------|------------------------------------------------------|
| `DAFB_LOCKSTEP_PATCH`          | (empty)               | ROM patch set (`tools/dump_rom_patches` keys)        |
| `DAFB_LOCKSTEP_MAX`            | 8000                  | event cap on each side                               |
| `DAFB_LOCKSTEP_TIMEOUT`        | 600000000             | RTL `+timeout=` cycles                               |
| `DAFB_LOCKSTEP_MAX_INSTS`      | 200000000             | RTL `+max_insts=` cap                                |
| `DAFB_LOCKSTEP_SECONDS`        | 4                     | MAME `-seconds_to_run` window                        |
| `DAFB_LOCKSTEP_INCLUDE_VRAM`   | 0                     | include VRAM aperture writes too (very noisy)        |
| `MAME_ROM_PATH`                | `/tmp/mame_iwm/roms`  | rompath shared with iwm-lockstep / axi-lockstep      |

## Known exemptions

* **MAME hardcodes `dafb_w` data to `data & 0xFFF`** for the core
  control / Swatch / clockgen blocks (12-bit wide registers).  The
  RTL stores the full 32-bit value the CPU wrote.  When the ROM reads
  it back, the diff is on the upper bits only — both sides return the
  programmed 12 bits, but our shim's read mux for offsets that aren't
  among MAME's named cases will return whatever the CPU wrote.  Where
  MAME applies an explicit mask in `dafb_r` (e.g. `+0x00`, `+0x04`,
  `+0x08`, `+0x2C`), we match it.
* **VRAM aperture is opt-in** (`+dafb_lockstep_include_vram`).
  The lockstep gate is the register boundary, not the pixel storage
  — including VRAM dominates the CSV with thousands of routine
  background-fill writes.
* **`mame-fastdiag` patch stack disables this capture** — that patch
  jumps past the DAFB init.  Run with the unpatched ROM (default) or
  a narrower patch set such as `chime-skip` to capture the full DAFB
  init sequence.

## Q700 ROM DAFB write trace (informational)

The unpatched Q700 ROM hits ~30+ unique DAFB register offsets in the
first 4 s of MAME-time during the boot probe.  A summary of the touched
offsets (collected pre-this-doc by `docs/dafb_audit.md` §2):

```
+0x00  control       baseA writeback (bits 20-9 of fb base)
+0x04  baseB         (bits 8-5 of fb base)
+0x08  stride        framebuffer row stride / 4
+0x0C  timing_ctrl
+0x10  config        BPP / convolution / interlace selectors
+0x14  block_ctrl
+0x1C  monitor sense drive (write) / inverse-sense (read)
+0x20–0x28  TurboSCSI ctrl[0..1] mirrors + status
+0x2C  test/version  (read returns dafb_version<<9 | test[8:0])
+0x100  swatch mode
+0x104  swatch IRQ enable / VBL arm
+0x108  swatch IRQ status
+0x10C  swatch cursor IRQ ack
+0x114  swatch VBL ack
+0x118  swatch cursor IRQ line
+0x11C  swatch animation IRQ line
+0x120  swatch test stash
+0x124..0x148  swatch H/V timing params
+0x200  RAMDAC palette address
+0x210  RAMDAC palette data (3-stage R/G/B)
+0x220  AC842 PCBR pixel-bus control (mode select)
+0x303, 0x313, …, 0x3F3  DP8531 clock-generator regs (writes only,
                          last-write-only effect via reg 15)
+0x300..0x3F0  16-entry low-depth CLUT (legacy)
```

## Future work

* **Bring the diff fully clean.** Current run on the unpatched ROM:
  see the `RESULT` block written to `build/dafb_lockstep/rtl.log` and
  the divergence summary printed by the diff tool.  Track outstanding
  divergence in `docs/mmio_coverage_audit.md` §4.
* **Wire into rom-boot smoke.** Like the AXI lockstep, the DAFB diff
  could become a per-merge gate once the diff is clean — capturing
  any new RTL change that breaks DAFB parity.
* **Drive longer runs** by letting MAME advance past the first frame
  so the diff covers the steady-state poll loop too (frame N+1
  status reads after the cursor IRQ).
