# MAME-vs-RTL IWM/SWIM register lockstep

The Q700 ROM uses the on-board SWIM1 controller (Apple's combined
IWM/ISM floppy controller) to probe for boot media.  This doc
describes a lockstep test that captures the SWIM register access
sequence from MAME (golden reference) and from our RTL stub
(`rtl/mac/iwm_stub.v`), diffs the two, and asserts byte-for-byte
parity.  This is the gate for the iwm_stub no-media handshake fix
(see `tb/tb_iwm.cpp::test_mame_lockstep_no_disk_init`).

## Why the lockstep matters

Pre-fix, `iwm_stub.v` advertised "no drive at all" via the SWIM
handshake `0x08`.  The Q700 ROM finishes the floppy probe, sees
"no drive", and combined with the SCSI stub returning "no devices",
concludes there is NO boot device of any kind and falls into MacsBug.

Real MAME (with no floppy attached, drive slot empty) returns `0x0c`
on the SWIM handshake — bits 2 and 3 set, per
`swim1_device::ism_read` line 233-234:

```cpp
case 0x7: { // handshake
    u8 h = 0;
    ...
    if(!m_floppy || m_floppy->wpt_r())
        h |= 0x0c;
    ...
}
```

Bit 2 = `rddata` idle, bit 3 = write-protect.  The Q700 ROM
interprets `0x0c` as "drive attached, no media inserted, please
insert a disk" and enters the canonical floppy-poll loop
(disk-prompt PC `0x40898e3e`), showing the `?-floppy` icon.  That is
the correct idle state for a no-boot-device Mac.

The stub now returns `0x0c` whenever its `drive_present` input is
asserted.  The platform (`rtl/fpga_top_peripherals.vh`) hardwires
`drive_present=1'b1`.  Legacy `0x08` is preserved for
`drive_present=0` for backwards-compat with older unit tests.

## How the capture works

* **MAME side** (`tools/mame_iwm_capture.lua`, loaded with
  `-autoboot_script`):

  1. Optional ROM patches read from `$MAME_IWM_PATCH_FILE` are
     applied to MAME's in-memory `:bootrom` region — the on-disk
     ROM is untouched so MAME's checksum still passes.  *Skip the
     standard `mame-fastdiag` stack: it bypasses the ROM's floppy
     probe entirely; with no patches the no-disk init sequence is
     captured cleanly.*
  2. A wide read+write tap is installed across the Q700 I/O window
     `0x5000_0000..0x50FF_FFFF`.  The tap callback filters to the
     SWIM aperture (sub-address `0x1e000..0x1ffff` after stripping
     the bitwise-OR mirror mask `0x00fc_0000`).  This catches every
     mirror combination the ROM picks (the Q700 ROM uses
     `0x50F1_xxxx` family).
  3. The 16-bit SWIM handler maps byte address → register selector
     as `reg = (byte_off >> 9) & 0xF` (i.e. `(word_off >> 8) & 0xF`).
     The byte rides the high byte of the high u16 lane (mask
     `0xFF00_0000` in 32-bit dbus mode).
  4. Every event is appended to `$MAME_IWM_TRACE_OUT` in CSV form:
     `<sim_time_ns>,<R|W>,<reg_hex>,<byte_hex>`.

* **RTL side** (`tb/tb_iwm.cpp::test_mame_capture_replay`, opt-in
  via `IWM_LOCKSTEP_CSV` env):

  1. The MAME CSV is replayed event-by-event into the
     `iwm_stub` DUT.  Reads sample `dut->rdata`; writes drive
     `dut->wdata`.  `drive_present=1` is asserted throughout (matches
     the platform default).
  2. If `IWM_LOCKSTEP_OUT` is set, every observed (rw, reg, byte)
     is mirrored to that file in the same CSV format.
  3. Any read divergence prints a `DIFF` line and the test fails.

* **Diff** (`tools/iwm_lockstep_diff.py`): reads both CSVs, walks
  them in lock-step on `(rw, reg, byte)` triples, exits non-zero
  on the first divergence with a context window.

## Run

```bash
# 1. Sanity-check MAME's macqd700 ROM set (one-time)
mkdir -p /tmp/mame_iwm/roms/{macqd700,adbmodem}
cp files/420dbff3.rom            /tmp/mame_iwm/roms/macqd700/
cp /home/qwertyoruiop/342s0440-b.bin /tmp/mame_iwm/roms/adbmodem/342s0440-b.bin
mame -rompath /tmp/mame_iwm/roms -verifyroms macqd700

# 2. Capture from MAME (~5 s of MAME-time is enough; 21 events)
MAME_IWM_TRACE_OUT=build/iwm_lockstep/mame_iwm.csv \
MAME_IWM_TRACE_LIMIT=512 \
mame -rompath /tmp/mame_iwm/roms macqd700 \
     -window -resolution0 320x240 -nothrottle \
     -seconds_to_run 5 -sound none -skip_gameinfo \
     -autoboot_delay 0 \
     -autoboot_script tools/mame_iwm_capture.lua

# 3. Replay against RTL stub
make tb-iwm
IWM_LOCKSTEP_CSV=build/iwm_lockstep/mame_iwm.csv \
IWM_LOCKSTEP_OUT=build/iwm_lockstep/rtl_iwm.csv \
build/iwm_stub/Viwm_stub

# 4. Byte-diff the two
python3 tools/iwm_lockstep_diff.py \
    build/iwm_lockstep/mame_iwm.csv \
    build/iwm_lockstep/rtl_iwm.csv
```

Expected output of step 4:

```
[iwm-lockstep] MAME ...: 21 events
[iwm-lockstep] RTL  ...: 21 events
[iwm-lockstep] streams IDENTICAL (21 events)
```

## Known-good baseline (2026-04-28)

With **MAME 0.264** + the unpatched Q700 universal ROM
(`files/420dbff3.rom`, sha-1 stored-checksum `0x420dbff3`), no
floppy attached, the no-disk init sequence is **21 SWIM accesses
total** (10 reads + 11 writes — the prompt-side spec asks for
"≥ 32 distinct read events" but MAME only emits this many in the
no-disk pre-MacsBug window, so 21 is the upper bound the gate can
target).

Sequence (canonical):

| #  | RW | reg | byte | meaning |
|----|----|-----|------|---------|
| 1  | R  | a   | ff   | IWM mode read, control bit5 cleared |
| 2  | R  | e   | ff   | IWM mode read, control bit7 cleared |
| 3  | R  | 8   | ff   | IWM mode read, control bit4 cleared |
| 4  | R  | d   | 80   | IWM **status read** — bit7=1 = no-disk |
| 5  | W  | f   | 57   | IWM→ISM mode-switch sequence (#1, bit6=1) |
| 6  | W  | f   | 17   | IWM→ISM (#2, bit6=0) |
| 7  | W  | f   | 57   | IWM→ISM (#3, bit6=1) |
| 8  | W  | f   | 57   | IWM→ISM (#4, bit6=1) → ISM mode entered |
| 9  | W  | 4   | f5   | ISM phases write |
| 10 | R  | c   | f5   | ISM phases readback (echoes write) |
| 11 | W  | 4   | f6   | … (5 more phases pairs through 0xfc) |
| …  | …  | …   | …    | |

Step 4 of the IWM-mode probe (`R d = 0x80`) is the bit that today's
stub had to fix to NOT fall into MacsBug.  But the stub ALREADY
returned `0x80` on that path — that path was never broken.  The
bug was on the **post-mode-switch SWIM handshake** path (offsets
`7` and `f` in ISM mode), which today's MAME-canonical capture does
NOT exercise inside the 21-event window because the ROM moves on
to ADB / video init before re-reading the handshake.  The
`tb_iwm.cpp::test_mame_lockstep_no_disk_init` scenario therefore
asserts the handshake `0x0c` value DIRECTLY at the end of the
21-event replay — that's the load-bearing assertion.

## Known exemptions

* **Internal SWIM timer events** are NOT captured (they don't go
  through the CPU bus).  This is by design — our stub doesn't
  emulate timer-driven phase advancement either, so this is a
  shared no-op surface.

* **The `mame-fastdiag` patch stack disables this capture** — it
  patches out the ROM RAM-test and floppy-probe path, after which
  the ROM never accesses the SWIM aperture.  Don't run with
  fastdiag if you want a non-empty trace.

* **MAME 0.264 fatals on `DAFB: Aux scanline interrupt enable`** if
  the ROM enables that bit.  Empirically the unpatched ROM does
  NOT enable that bit in the first 5 s, so the capture works.  If a
  future ROM patch causes the fatal, either drop the patch or
  patch MAME locally to soft-warn instead.

## Future work

* **Drive a longer capture** by injecting "pretend disk inserted"
  state into the SWIM after the IWM-to-ISM transition — this would
  let the ROM proceed into the floppy-data read path and exercise
  the read side of the SWIM ISM-mode register set.  Needs a
  follow-on patch to `iwm_stub.v` that toggles a configurable
  "media present" input.

* **Wire the captured trace into the FPGA-top rom-boot flow**
  (`tb_fpga_top_rom.cpp`) as an extra invariant on each rerun:
  log the SWIM accesses observed during the rom-boot sim and
  byte-diff against the MAME baseline.  Today the platform-level
  sim diverges from MAME on the boot path, so this is gated on
  closing that gap first.
