# MAME-vs-RTL CPU-side AXI lockstep

CPU-bus byte-for-byte parity between MAME's macqd700 and our RTL
fpga_top, for everything **except** DDR (`0x0000_0000..0x3FFF_FFFF`)
and ROM (`0x4000_0000..0x40FF_FFFF`).  Used to localise the first
peripheral / VRAM / DAFB transaction where the boot path divergences
visible at the PC level (e.g. PM-identified back-jump from
`0x4084b3bc` to `0x4080428e`) translate into a different bus
operation.

## Why this lockstep

Higher-level checks (PC trace, retired-uop count, register dumps)
prove that the CPUs are not executing the same code stream — but they
don't pinpoint *where* the divergence enters the system.  The CPU-bus
lockstep does: the first transaction that doesn't match is, by
definition, the first place the CPUs see different bytes from the
peripheral fabric, which is the proximate cause of the path split.

Address filter — what's in scope:

| Range                              | In scope? | Reason                                  |
|------------------------------------|-----------|-----------------------------------------|
| `0x0000_0000..0x3FFF_FFFF`         | NO        | DDR / RAM — too noisy to diff usefully  |
| `0x4000_0000..0x40FF_FFFF`         | NO        | ROM — fetch-only, not where divergence enters |
| `0x4100_0000..0x4FFF_FFFF`         | YES       | ROM mirror window (defensive)           |
| `0x5000_0000..0x5FFF_FFFF`         | YES       | Q700 I/O peripherals (VIA1/2, SCC, SCSI, IWM, ASC, ENET, SONIC, ORWELL) |
| `0xF900_0000..0xF91F_FFFF`         | YES       | DAFB pixel aperture (VRAM)              |
| `0xF980_0000..0xF980_03FF`         | YES       | DAFB live registers                     |
| `0x6000_0000..0x607F_FFFF`         | YES       | scratch FB region                       |
| anything else                      | YES       | catches stray probes (NuBus, etc.)      |

## How the captures work

* **MAME side** (`tools/mame_axi_capture.lua`, loaded with
  `-autoboot_script`):

  1. Optional ROM patches read from `$MAME_AXI_PATCH_FILE` are applied
     to MAME's in-memory `:bootrom` region — the on-disk ROM is
     untouched so MAME's checksum still passes.  The wrapping target
     `tb-axi-lockstep` populates this file from
     `tools/dump_rom_patches`.
  2. Wide read+write taps are installed across
     `0x4000_0000..0xFFFF_FFFF`.  The callback drops in-DDR + in-ROM
     addresses and emits one CSV line per byte / half-word / longword
     access.  The mask-driven decoder splits multi-byte handler
     accesses into the architectural-size events the CPU emitted
     (1 / 2 / 4 bytes).
  3. Output goes to `$MAME_AXI_TRACE_OUT` in CSV form:
     `<seq>,<R|W>,<addr_hex>,<size_bytes>,<data_hex>`.

* **RTL side** (`tb/tb_fpga_top_rom.cpp::main` with
  `+axi_lockstep_log=<path>`):

  1. Per-cycle sniffing of the M0 narrow data path
     (`fpga_top.core_daxi_*`) and the M3 wide instruction-fetch path
     (`fpga_top.ifa_*`).  The narrow side is byte-granular with
     4-bit wstrb so byte-address + size are recovered without lane
     gymnastics.  The wide IF side is sliced by `addr[3:2]`.
  2. AR / AW handshakes latch the address; R / W+B handshakes emit
     the line.  Same address filter as MAME.
  3. CSV output matches MAME's format exactly so
     `tools/axi_lockstep_diff.py` can byte-diff.

* **Diff** (`tools/axi_lockstep_diff.py`):
  reads both CSVs, walks them in lock-step on `(rw, addr, size, data)`
  tuples (the `<seq>` column is reindexed in load order), exits
  non-zero on the first divergence with a ±5-event context window
  printed for each side.

## Run

```bash
# 1. Sanity-check MAME's macqd700 ROM set (one-time)
mkdir -p /tmp/mame_iwm/roms/{macqd700,adbmodem}
cp files/420dbff3.rom               /tmp/mame_iwm/roms/macqd700/
cp /home/qwertyoruiop/342s0440-b.bin /tmp/mame_iwm/roms/adbmodem/342s0440-b.bin
mame -rompath /tmp/mame_iwm/roms -verifyroms macqd700

# 2. End-to-end (default patch_set = "mame-fastdiag,chime-skip"):
make tb-axi-lockstep

# 3. With overrides:
make tb-axi-lockstep \
    AXI_LOCKSTEP_PATCH=mame-fastdiag,chime-skip \
    AXI_LOCKSTEP_MAX=10000 \
    AXI_LOCKSTEP_TIMEOUT=800000000
```

Outputs land in `build/axi_lockstep/`:

```
axi_mame.csv     — MAME-side capture
axi_rtl.csv      — RTL-side capture
patches.txt      — byte-patch list applied on both sides
mame.log         — MAME stdout/stderr
rtl.log          — RTL sim stdout/stderr
```

## Address filter in code

Mirror-identical filters guard both captures:

```c
// tb/tb_fpga_top_rom.cpp
if (addr < 0x40000000u) return false;  // DDR
if (addr < 0x50000000u) return false;  // ROM + ROM-mirror window
return true;
```

```lua
-- tools/mame_axi_capture.lua
if addr < 0x40000000 then return false end  -- DDR
if addr < 0x50000000 then return false end  -- ROM + ROM-mirror window
return true
```

If MAME emits a peripheral access that the RTL filter would drop (or
vice versa), the diff lines up incorrectly.  Keep the two filters in
sync.

## Known-good baseline (2026-04-28)

This lockstep is **divergence-finding by design** — the streams
are *expected* to diverge until the underlying RTL bug is fixed.
Today's PM-identified divergence (CPU back-jumps from `0x4084b3bc`
to `0x4080428e` on RTL while MAME proceeds forward to
`0x40898e3e` floppy-prompt) surfaces as an early divergence
(seq < 100) — see the `tb-axi-lockstep` invocation reports.

The "known-good" criterion is: after a fix landings, the
divergence point moves later (higher seq) or eliminates entirely.
First-light (Mac logo render) requires the diff to walk the full
boot trace without a divergence inside the I/O-discovery / VIA1
keyboard-poll / DAFB-init window.

## Known exemptions

* **AXI burst metadata is not captured.**  The diff focuses on the
  CPU-visible data side: address, direction, size, data.  ARLEN /
  ARSIZE / ARBURST handshake-side details would only matter if the
  divergence were inside the burst protocol itself, which is unlikely
  for single-beat peripheral accesses.

* **Internal write-back / cache behaviours** that don't reach the AXI
  bus are not captured.  This is intentional — those sit on the CPU
  side of the L1D, so a coherence bug there would surface as a
  *delayed* divergence at the next bus access (a write that should
  have reached the peripheral but was held up in the store buffer).

* **MAME 0.264 has been observed to fatal on `DAFB: Aux scanline
  interrupt enable`** if the ROM enables that bit.  Empirically the
  fastdiag-patched ROM does NOT enable that bit in the first 4 s, so
  the capture works.  If a future patch causes the fatal, drop the
  patch or extend `tools/dump_rom_patches.cpp` to NOP the offending
  ROM write.

## Future work

* **Wire into ROM-boot CI** — once the streams converge, add a
  `make ci-axi-lockstep` target that fails on any divergence inside
  the first-light window.  Today the divergence is expected, so the
  target is diagnostic-only.

* **Coalesce contiguous transactions** — the RTL side emits one event
  per AXI transaction; MAME emits one per CPU memory access.  When a
  CPU does a `MOVEM.L` writing 8 longwords, RTL emits 8 events while
  MAME's mask-decoder also emits 8.  No coalescing is needed today,
  but a future LSU optimization that fuses adjacent stores into a
  burst would need this layer to match.
