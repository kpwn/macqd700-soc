# MAME-vs-RTL VIA1 register-file byte parity

Byte-for-byte parity between MAME's `via6522_device` (driven by the Q700
ROM through `quadrax00_state::via_r/via_w`) and our `rtl/mac/via1.v` at
the peripheral_bus VIA1 master face.  The lockstep complements the
broader CPU-side AXI lockstep (`docs/axi_lockstep.md`) by letting us
isolate VIA1 register-file regressions from CPU / decoding regressions.

## Why this lockstep

VIA1 is the boot-critical timer + IRQ + ADB / RTC source.  The Q700 ROM
polls IFR for VBL, manipulates IER for masking, and reads/writes ORA /
ORB / DDR / PCR / ACR during initialisation.  A divergence in any of
these registers shows up as a CPU-side AXI mismatch, but the AXI
lockstep can't separate "CPU did the wrong access" from "VIA1 returned
the wrong byte".  This lockstep does: the input is identical (both
captured downstream of the CPU's MMIO load), so any divergence in the
CSV is a VIA1 RTL bug.

## Address-map basics

```
0x5000_0000..0x5000_1FFF   VIA1 (Q700)        (MAME, mirror 0x00fc_0000)
```

In MAME the VIA1 handler is u16-wide (`u16 via_r(offs_t offset)`); MAME
calls the handler with a WORD offset, then `offset >>= 8; offset &=
0x0f` selects the 6522 register.  Translated to byte addresses inside
the 0x2000-byte aperture, register `n` lives at offset `(n << 9)`.  Our
`peripheral_bus.v` decodes `addr[12:9]` into `via1_addr[3:0]` — a
matching mapping.

The byte rides EITHER:
- the high byte of the high u16 lane (mask `0xff00_0000`, lane 3) when
  the CPU touches the upper half of the 32-bit word, or
- the high byte of the low u16 lane (mask `0x0000_ff00`, lane 1) when
  the CPU touches the lower half of the 32-bit word.

The Q700 ROM consistently issues byte-wide MMIO accesses to VIA1, so
exactly one byte is hot at a time.

## How the captures work

* **MAME side** (`tools/mame_via1_capture.lua`, loaded with
  `-autoboot_script`):

  1. Optional ROM patches read from `$MAME_VIA1_PATCH_FILE` are applied
     to MAME's in-memory `:bootrom` region — same approach as the IWM
     and AXI captures.
  2. A wide read+write tap is installed across
     `0x5000_0000..0x50FF_FFFF`.  The callback strips the Q700 mirror
     mask (`0x00fc_0000`) and filters to the VIA1 sub-aperture
     (`0x0000..0x1FFF`).
  3. For each masked byte lane the script computes the VIA1 register
     selector as `(mac_byte_off >> 9) & 0xF` and emits one CSV line.
     One event per architectural CPU access — duplicated lanes inside
     the same `via_r/via_w` call are skipped because the CPU only
     touched one byte.
  4. Output goes to `$MAME_VIA1_TRACE_OUT` in CSV form:
     `<seq>,<R|W>,<reg_hex>,<byte_hex>`.

* **RTL side** (`tb/tb_fpga_top_rom.cpp::main` with
  `+via1_lockstep_log=<path>`):

  1. Per-cycle sniffing of the peripheral_bus VIA1 master face
     (`fpga_top.pb_via1_*`).  pb_via1_wr / pb_via1_rd are 1-cycle
     pulses on the pb_clk; we detect the rising edge to emit exactly
     one event per pulse (sys_clk runs faster than pb_clk so the same
     pulse is observed across multiple `tick()`s).
  2. Writes emit on the rising edge — pb_via1_wdata and pb_via1_addr
     are valid same cycle.
  3. Reads latch the addr on the rising edge of pb_via1_rd; pb_via1_rdata
     is registered (`via1.v`'s read mux runs off pb_addr which holds
     while pb_rd is high), and we sample pb_via1_rdata on the falling
     edge of pb_via1_rd to capture the response.
  4. CSV output matches MAME's format exactly so
     `tools/via1_lockstep_diff.py` can byte-diff.

* **Diff** (`tools/via1_lockstep_diff.py`): reads both CSVs, walks them
  in lock-step on `(rw, reg, byte)` triples, exits non-zero on the first
  divergence with a ±5-event context window.  Register names (ORB / ORA
  / DDRB / DDRA / T1CL / T1CH / T1LL / T1LH / T2CL / T2CH / SR / ACR /
  PCR / IFR / IER / ORA-NH) are printed inline so the report reads as
  6522 register accesses, not raw indices.

## Run

```bash
# Sanity-check MAME's macqd700 ROM set (one-time, shared with iwm-lockstep)
mkdir -p /tmp/mame_iwm/roms/{macqd700,adbmodem}
cp files/420dbff3.rom            /tmp/mame_iwm/roms/macqd700/
cp /home/qwertyoruiop/342s0440-b.bin /tmp/mame_iwm/roms/adbmodem/342s0440-b.bin
mame -rompath /tmp/mame_iwm/roms -verifyroms macqd700

# End-to-end (default patch_set = "mame-fastdiag,chime-skip"):
make tb-via1-lockstep

# With overrides:
make tb-via1-lockstep \
    VIA1_LOCKSTEP_PATCH=mame-fastdiag,chime-skip \
    VIA1_LOCKSTEP_MAX=4096 \
    VIA1_LOCKSTEP_SECONDS=4
```

Outputs land in `build/via1_lockstep/`:

```
mame_via1.csv    — MAME-side capture
rtl_via1.csv     — RTL-side capture
patches.txt      — byte-patch list applied on both sides
mame.log         — MAME stdout/stderr
rtl.log          — RTL sim stdout/stderr
mame_nvram/      — throwaway per-run MAME nvram dir (see below)
```

## NVRAM determinism

MAME persists the RTC PRAM image to `$HOME/.mame/nvram/macqd700/rtc`
and reloads it on the next boot.  A stale image makes MAME take the
"PRAM valid" boot fast-path while the RTL — whose `rtc.v` resets PRAM
to zero (`mame_state_mode`) — takes the "PRAM invalid → reinit" path.
The streams then diverge for environmental reasons, not RTL bugs (the
first such divergence observed was an extended-PRAM read of byte 0xF9
returning 0x01 from a populated nvram vs 0x00 from zero-fill).

The make target therefore points `mame -nvram_directory` at a
freshly-wiped `$(VIA1_LOCKSTEP_DIR)/mame_nvram` on every run, so MAME
always boots from power-on-default (zero) PRAM — the same starting
state the RTL has.  Do not run the lockstep against `$HOME/.mame`.

## What a clean diff looks like

Once VIA1 is at byte parity (the goal of this lockstep), the diff
output ends with:

```
[via1-lockstep] streams IDENTICAL (NN events)
```

Until then, the diff reports the first divergence + ±5-event windows
on each side.  The first divergence is by construction the first place
the RTL VIA1 register file deviates from the canonical 6522 model.

## Known exemptions

* **Internal VIA1 timer events are NOT captured.**  T1 / T2 phi2-tick
  countdowns happen inside the VIA, not on the CPU bus.  Only register
  accesses go through pb_via1_*, so timer-driven IFR transitions
  surface as the CPU reading the new IFR value (visible) rather than
  as a separate timer event (not visible — by design).

* **CA1 / CB1 / CA2 / CB2 edge events are not emitted as separate
  events** — they're observable through the IFR read path, which IS
  captured.  A divergence on the VBL → CA1 → IFR[1] chain manifests as
  a different IFR byte returned to the CPU.

* **Same-cycle write-then-read aliasing** (CPU writes ORB then reads
  ORB on the very next pb_clk) lands as two separate CSV events — they
  do NOT collapse.  Both sides produce the same two events, so this is
  a non-issue for the diff.

## Future work

* **Tie into ROM-boot CI** — once MAME / RTL converge, add the lockstep
  to `make ci-fast` so any VIA1 regression fails CI.
* **Coalesce repeated IFR-poll traces** — the ROM hot-loops on IFR
  reads while waiting for VBL.  A coalescer would collapse identical
  consecutive `R IFR=0xXX` events into a count, making divergences
  near the poll easier to read.
