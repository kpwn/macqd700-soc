# MAME integration — reference + trace-oracle plan

Research companion to `peripheral_arch.md` and `gameplan.md` phase 2/3.
Nothing in this doc touches RTL, tests, or build scripts — it is a
reference for coupling our sim to MAME as a behavioural oracle and
trace-replay golden, now that the target ROM + chipset decision is
closed.

---

## 1. Executive summary

1. **Target chipset: Quadra 700** (discrete VIA1 + VIA2 + NCR 5380 +
   Z80 SCC + ASC (SONORA variant) + DAFB + MEMCjr). See
   `CLAUDE.md` §Mac Hardware Integration for the full chipset
   rationale.
2. **Committed ROM**: `files/420dbff3.rom` — 1 MB Quadra 700 Universal
   ROM, SHA1 `7a8ee468d16e64f2ad10cb8d1a45e6f07cc9e212`, stored-checksum
   `0x420dbff3`, reset-vector entry `0x0000002a`. Default for
   `make tb-rom-boot` (see `docs/rom_boot_bringup.md`). MAME machine
   shortname: **`macqd700`**.
3. **MAME value proposition**: read the BSD-licensed device models at
   `src/devices/machine/*.cpp` and `src/mame/apple/*.cpp` as a
   behavioural spec when the datasheet/PRM is ambiguous; boot the
   same ROM in MAME and trace-replay it against our Verilator sim to
   catch divergence early.
4. **Tier plan**:
   - **Tier 1** (read-only MAME source reference): active from day 1.
     No MAME build needed — GitHub raw URL is sufficient. Covers
     ~60 % of peripheral bring-up bugs.
   - **Tier 2** (trace capture from MAME, replay against our sim):
     3–4 agent-days to stand up. Primary tool for ROM cold-boot
     progression and phase-3 System 7 boot divergence analysis.
     Already in use — see `docs/rom_boot_bringup.md` §4.7.
   - **Tier 3** (DPI co-sim of MAME devices into Verilator): not
     worth it. MAME devices are welded to `device_t` + `devcb` +
     `machine_config` infrastructure that would cost 2–4 agent-weeks
     to sever. Tier 2 covers the same need in days.

The rest of this doc is the evidence backing those conclusions plus
the recipes to execute them.

---

## 2. MAME build + install on this host

### 2.1 Host capability
- **OS**: Ubuntu 24.04.3 LTS (Noble), kernel 6.8.0-107-generic.
- **CPU**: 32 cores.
- **RAM**: 62 GiB (8 GiB swap).
- **Disk**: 657 GiB free on `/`.

MAME compiles fine here; full build ~20–30 min wall-clock, single
subtarget build ~5–10 min. Disk footprint: ~5 GB of source + ~4 GB
of build artefacts, ~180 MB final `mame` binary (full) or ~30–60 MB
for a subtarget.

### 2.2 Repository & pinning
- URL: `https://github.com/mamedev/mame`
- Latest stable tag at time of writing: **`mame0287`** (MAME 0.287,
  released 2026-03-30). That's the recommended pin for
  reproducibility. Quote this tag in any agent task spawning a MAME
  build.
- `master` follows 0.287+ and moves daily; avoid it unless the pin
  is being deliberately refreshed.

Minimum recommended MAME version for Mac Quadra work: **0.266 or
later** (MAME got rid of the legacy `ncr5380n.cpp` in 0.240-ish and
reorganised the Apple driver tree around 0.262; older MAMEs have a
monolithic `mac.cpp` that isn't worth reading). 0.287 is strictly
better than 0.266 for our reference purposes.

### 2.3 Packages (Ubuntu 24.04)

From MAME's own `docs/source/initialsetup/compilingmame.rst`:

    sudo apt-get install \
        git build-essential python3 \
        libsdl2-dev libsdl2-ttf-dev \
        libfontconfig-dev libpulse-dev \
        qt6-base-dev qt6-base-dev-tools qtchooser

For headless tracing we don't actually need Qt or pulse/SDL audio,
but the MAME build system still tests for them. The smallest working
subset in practice on 24.04 is:

    sudo apt-get install git build-essential python3 \
        libsdl2-dev libsdl2-ttf-dev libfontconfig-dev

(omitting Qt forces the build to use the native text-mode debugger,
which is what we want for scripted traces anyway).

GCC 13.2 (Ubuntu 24.04 default) is ≥ 11 per MAME's C++20 requirement;
no compiler upgrade needed.

### 2.4 Subtarget build: only the Apple-Mac drivers

MAME's minimal-build mechanism is `SUBTARGET=<name> SOURCES=<list>`:

    cd /tmp
    git clone --depth 1 --branch mame0287 https://github.com/mamedev/mame.git
    cd mame
    make SUBTARGET=macoracle \
         SOURCES=src/mame/apple \
         TOOLS=0 \
         REGENIE=1 \
         -j8

Notes:
- `SOURCES=src/mame/apple` pulls in the entire `apple/` driver folder
  (Quadra, LC, II, IIci, PowerBook, etc). That is ~40 drivers, but
  they share devices and the per-driver cost is tiny. Narrower
  `SOURCES=src/mame/apple/macquadra700.cpp` would compile faster
  (~3–5 min on this box) at the cost of needing to list every .cpp
  the driver pulls in as a dependency.
- `REGENIE=1` regenerates project files; required whenever `SOURCES`
  changes or the MAME tree moves commits.
- Output binary: `mame_macoracle` (or `mame_macoracle64`) in the MAME
  root. ~40–60 MB stripped.
- Without `SUBTARGET`, a full `make -j32` on this host builds in
  ~25–30 min. Subtarget save is real but not dramatic.

**Recommended agent invocation** (safe reproduce-from-scratch):

    git clone --depth 1 --branch mame0287 https://github.com/mamedev/mame.git /tmp/mame
    cd /tmp/mame
    make SUBTARGET=macoracle SOURCES=src/mame/apple REGENIE=1 -j4

Expect a slower but less disruptive build with `-j4`, ~4 GB build
directory, and a ~50 MB final binary.

### 2.5 If MAME is infeasible to build

Fallback: **GitHub raw-file read** of `src/devices/machine/*.cpp` and
`src/mame/apple/*.cpp`. All of MAME's device models are BSD-3 licensed
and are single-file-readable — you do not need a working MAME binary
to use them as behavioural reference for Tier 1. This doc's §4 works
100 % from GitHub without a local MAME checkout.

Tier 2 (trace capture + replay) DOES need a working MAME binary,
because that's the oracle that generates the trace. Tier 3 needs a
build tree. So: if the build fails on this box (unlikely, but
possible if SDL2 has ABI drift), document the failure, fall back to
Tier 1 only, defer Tier 2 until a build host is sorted.

---

## 3. MAME source files mapped to our peripherals

All paths are relative to MAME repo root, confirmed present on
`mamedev/mame` master as of April 2026 via the GitHub tree API.

### 3.1 Quadra 700 chipset files (our target)

The Quadra 700 is the first-generation 68040 Mac and keeps the
discrete-chip architecture of the Mac II family rather than the
later IOSB/djMEMC fusion. This is the exact chipset `peripheral_arch.md`
targets.

| Our module | MAME path | What it models |
|---|---|---|
| `rtl/mac/via1.v` | `src/devices/machine/6522via.cpp` (+`.h`) | Full 6522 VIA: T1/T2 timers, SR shift register, CA1/CA2/CB1/CB2 handshakes, IFR/IER with bit-7 summary. MAME uses the **same 16 register offsets** we already have in `peripheral_arch.md`. Quadra 700 VIA1 is a real discrete 6522, not a pseudo-VIA. |
| `rtl/mac/via2.v` | `src/devices/machine/6522via.cpp` | Quadra 700 VIA2 is **also** a real discrete 6522 (unlike the later IOSB-integrated pseudoVIA). Slot interrupts, SCSI IRQ/DRQ gating. |
| `rtl/mac/scsi.v` | `src/devices/machine/ncr5380.cpp` (+`.h`) | Current NCR 5380 implementation. Pat Mackinlay rewrite (the `n` variant was deleted around 0.240). Models NCR 5380, NCR 53C80, Sony CXD1180, DP8490. **Initiator + target, arbitration, phases, DMA.** |
| `rtl/mac/scsi.v` wrapper | `src/mame/apple/macscsi.cpp` (+`.h`) | `mac_scsi_helper_device` — Apple's pseudo-DMA glue sitting between the 68040 bus and the NCR 5380. Handles the FIFO + timeout mode bits the Mac ROM expects. **This is the piece our `scsi.v` boundary most closely resembles**. |
| `rtl/mac/scc.v` | `src/devices/machine/z80scc.cpp` (+`.h`) | Zilog Z8530/Z85C30 SCC. Full channel pair, MR0–MR15, WR0–WR15. Heavy model. For phase-2 stub we need almost none of it; for phase-3 serial we'll lean on it hard. |
| `rtl/mac/asc.v` | `src/devices/sound/asc.cpp` + `.h` | Apple Sound Chip. 7 `asc_type` enum variants. Quadra 700 uses the **SONORA** variant; registers at `R_VERSION=0x800`...`R_TEST=0x80f`. Our `asc.v` base is `0x50014000` to match Q700 DAFB-era map. |
| `rtl/mac/video.v` (DAFB) | `src/mame/apple/dafb.cpp` + `.h` | **DAFB = "Display Adapter for Framebuffer"** — the Q700/Q900/Q950 video block. Standalone chip on the Q700, later absorbed into djMEMC. This is the exact video block `peripheral_arch.md` targets. |
| `rtl/sys/ddr_ctrl.v` RAM/ROM decode | Q700 has no integrated memory controller; address decode lives in GLU(e). | For Q700 the RAM decoder is simpler than djMEMC — straight RAM at `0x00000000`, ROM at `0x40000000`, no 1 GB aperture trickery. |
| `rtl/mac/rtc.v` (planned) | `src/mame/apple/macrtc.cpp` + `.h` | Apple 343-0042 RTC + PRAM. Bit-banged serial via CE/CLK/DATA pins (matches our `peripheral_arch.md` RTC signal list). `ce_w` / `clk_w` / `data_r` / `data_w` public API. NVRAM-backed 256-byte PRAM. |
| `rtl/mac/glue.v` (address decoder + IRQ aggregator) | `src/mame/apple/macquadra700.cpp` | The Q700 driver's `memory_map` + `irq_map` is the spec for our GLUE. No single ASIC (no IOSB) — the decoder lives in the driver file directly. |
| ADB — VIA1 SR side | `src/mame/apple/macadb.cpp` + `.h` | `macadb_device` — ADB transceiver living on VIA1's shift register, with `adb_data_callback / adb_irq_callback / adb_power_callback / adb_akd_callback`. Quadra 700 uses the **direct VIA-SR ADB path** (no adbmodem/Cuda intermediary); this matches our planned `adb_phy.v`. |
| MMU / ATC reference | `src/devices/cpu/m68000/m68kmmu.h` | **68851 / 68030 / 68040 PMMU** implementation in one file. Contains the exact MMU-SR bit layout, descriptor flag encodings, and TC register field constants we need for our MMU stub (`M68K_MMU_SR_*`, `M68K_MMU_DF_*`, `M68K_MMU_TC_*`). Our `docs/core_gaps.md` §MMU should cite this as the golden field layout. |

### 3.2 Optional alternate-ROM chipsets (comparison testing only)

These are NOT our target and require NO action. They are listed here
only as reference points if we ever want to compare how a different
Apple chipset exercises the same peripheral devices (e.g. VIA1 timer
semantics). The committed Q700 ROM remains authoritative.

| File | Machine | ROM | Notes |
|---|---|---|---|
| `src/mame/apple/macquadra800.cpp` | Quadra 800, Centris 610/650, Quadra 610/650 | `f1acad13.rom` (CRC `4e70e3c0`, 1 MB) — Q800 Version 23F2 | IOSB + djMEMC fusion chipsets, NCR 5380 behind pseudo-DMA helper, adbmodem intermediary on ADB path. Useful for comparing how our VIA1/SCSI would behave on a post-Q700 chipset if we ever widen compatibility. |
| `src/mame/apple/macquadra630.cpp` | Quadra 630, LC/Performa 580 | `06684214.bin` (CRC `1735e7a5`) — 68LC040 | Historical only. The original committed ROM in this repo (removed in commit `616fb32` as wrong chipset). F108 + PrimeTime II + Valkyrie + IDE + Cuda — none of which matches our peripheral RTL. |

### 3.3 Chipset-specific component files (for context only)

| File | Relevance |
|---|---|
| `src/mame/apple/dafb.cpp` + `.h` | **RELEVANT** — Q700 video block. |
| `src/mame/apple/iosb.cpp` + `.h` | Not our target (Q800-era IOSB ASIC). |
| `src/mame/apple/djmemc.cpp` + `.h` | Not our target (Q800-era memory controller). |
| `src/mame/apple/f108.cpp` + `.h` | Not our target (Q630 memory controller). |
| `src/mame/apple/valkyrie.cpp` + `.h` | Not our target (Q630 video). |
| `src/mame/apple/cuda.cpp` + `.h` | Not our target (Q630-and-later ADB µcontroller). Q700 uses direct VIA-SR ADB. |
| `src/mame/apple/egret.cpp` + `.h` | Not our target (IIsi/LC III/IIvx pre-Cuda ADB µcontroller). |
| `src/mame/apple/adbmodem.cpp` + `.h` | Not our target (Q800 adbmodem intermediary). Q700 has no adbmodem. |
| `src/mame/apple/rbv.cpp` + `.h` | Not our target (IIci/IIsi pseudo-VIA). |
| `src/mame/apple/sonora.cpp` + `.h` | Not our target (LC III Sonora I/O ASIC — distinct from the ASC-SONORA *sound-chip variant* we use). |
| `src/mame/apple/mactoolbox.cpp` + `.h` | **MAYBE USEFUL** — custom m68k disassembler that names A-line traps (`_Open`, `_Read`, etc). If we ever disassemble ROM in our own tools this is the dispatch table to steal. |

### 3.4 What each MAME file gives us that docs don't

Programmer's Reference and NCR datasheets tell you the registers
exist. MAME tells you **which bits the Mac ROM actually writes first,
and in what order**. The two deltas we care about:

- **VIA1 T1 latch vs counter timing**: real 6522 has a subtle
  write-high-starts-timer behaviour that Atari/Commodore code exploits
  but Mac ROM doesn't care about. MAME's `6522via.cpp::execute_task`
  shows which paths actually fire on a Mac run. When we find a test
  that hangs on a timer poll, diffing our T1 transitions against
  MAME's log tells us in 5 minutes whether our counter reload semantic
  is right.
- **NCR 5380 arbitration + phase-match bit**: the BSY→SEL handshake
  is the classic "works on 90 % of implementations" edge case. MAME's
  `ncr5380.cpp::state_loop` has a ~400-line FSM that names every
  transition. We already have a Target-FSM; MAME's is the Initiator
  side we need to bounce off of.
- **VIA1 ADB SR attention edge**: the "shift-register-interrupt-clears-
  on-SR-read" vs "on-any-write" distinction. MAME `6522via.cpp`'s
  `shift_in / shift_out` has this correct and is the fastest way to
  resolve a test failure rooted in SR IFR clearing.
- **VIA1 CA1 ack path**: VBL/CA1 is acknowledged by the handshake ORA
  access, not by ORB / ORA-NH. If ROM stalls in a VBL poll, that is the
  first place to check before blaming the timer.
- **IFR bit-7 summary**: real 6522 is `IFR[7] = |(IFR[6:0] & IER[6:0])`.
  Our `peripheral_arch.md` says this; MAME confirms it (`m_ifr =
  (m_ifr & 0x7f) | ((m_ifr & m_ier & 0x7f) ? 0x80 : 0)` in
  `6522via.cpp`). If our `via1.v` IFR summary doesn't match, the ROM
  will enter an IRQ-check loop that never resolves — MAME tells us
  that instantly.

---

## 4. Tier 1 — peripheral behavioural reference (active from day 1)

**Goal**: no MAME runtime. The MAME source tree is read-only
reference material the sub-agents doing VIA1/VIA2/SCSI/SCC/RTC
consult when the datasheet is ambiguous or when a test fails in a
way the PRM doesn't explain.

### 4.1 Workflow per peripheral

Concrete recipe: when implementing or debugging a peripheral module,
spawn a sub-agent with this instruction shape:

> Implement VIA1 T1 timer per `peripheral_arch.md` §VIA1. Test
> `tb_via1_t1_continuous.cpp` is failing: timer reloads but no
> IRQ fires on second wrap. Before writing RTL, read
> `https://raw.githubusercontent.com/mamedev/mame/mame0287/src/devices/machine/6522via.cpp`
> lines covering `t1_tick()` and `counter_to_timer()`. Find the exact
> cycle the IRQ bit is set on reload vs on expiry. Compare to our
> `via1.v::t1_countdown`. Report the discrepancy; then patch.

The agent does NOT need a MAME build to do this — the GitHub raw URL
is the only dependency. `gh api` works if authenticated, but raw
HTTPS `curl` or `WebFetch` is sufficient and was used to build §3 of
this doc.

### 4.2 Per-peripheral edge cases MAME clarifies

What to look at in each file when stuck:

**VIA1 / VIA2 (`6522via.cpp`)**
- `void via6522_device::write(offs_t offset, u8 data)` — which writes
  start T1, which writes reset IFR bits, how PCR edges interact
  with CA1/CB1 latch. Our `via1.v` must match the flag-clear order
  or the ROM will spin in IFR-poll.
- `void via6522_device::execute_task()` — T1/T2 counter countdown
  logic. Phase-2 blind spots: T1 one-shot vs free-run transition
  (ACR bit 6), T2 in PB6 pulse-count mode (we don't use it but the
  ROM probes it during autoconfig).
- `shift_in()` / `shift_out()` — ADB SR clocking. When our SR
  generates the wrong edge count, the ADB transceiver timeouts.

**NCR 5380 (`ncr5380.cpp`)**
- `state_loop()` — the target-side FSM. Our `scsi.v` is target-side
  too, so this is the most direct parallel. Compare phase transitions
  BSY→SEL, SEL→CMD, data in/out → status.
- `read_wrapper` in `mac_scsi_helper_device` (`src/mame/apple/macscsi.cpp`)
  — Apple's pseudo-DMA over the 5380. This sits between the 68040
  bus and the 5380 core and implements the "hold ACK until FIFO
  drain" semantic that real Mac ROM uses. Our scsi.v doesn't yet
  model pseudo-DMA; when phase-3 needs disk throughput, this file is
  the reference.
- `arbitration_decide()` / `selection_check()` — arbitration priority
  and bus-free detection. Our current `scsi.v` bypasses arbitration
  (single target, ID 0 hardcoded). When we add a second ID for
  CD-ROM support this file is the spec.

**SCC (`z80scc.cpp`)**
- `do_sccreg_wr0()` / `do_sccreg_wrN()` — the 16 × 2 write-register
  decode. Phase-2 stub doesn't need any of this; phase-3 does when
  AppleTalk probes channel B.
- Baud-rate-generator tick. For now: unused.

**ASC (`asc.cpp`)**
- `read(offs_t offset)` / `write(offs_t offset, u8 data)` — the 16
  control registers at `0x800-0x80f`. Q700 uses the `SONORA` enum
  variant (512-byte FIFO per channel). The IRQF ("FIFO half-empty")
  bit timing is something the OS polls heavily.
- `sound_stream_update` — the sample-generation loop. We don't need
  to emulate this body; only the register-side behaviour matters.

**macrtc (`src/mame/apple/macrtc.cpp`)**
- `ce_w / clk_w / data_r / data_w` — the 4-wire bit-bang interface
  used by VIA1's PB0/PB1/PB2. `m_bit_count`, `m_data_dir`, `m_cmd`
  state machine matches 343-0042 datasheet exactly. `peripheral_arch.md`
  says "commands 0x81, 0x85, 0x89, 0x8D" for read-seconds; MAME shows
  the PRAM-read and write-protect dance too (commands 0x31-0x3D for
  PRAM, 0x35/0x34 for write-protect).

**ADB path (`macadb.cpp`)**
- Phase-2 doesn't need ADB. Phase-3 does. `macadb_device::service`
  drives the `poll 2 ms` loop that the Mac ROM kernel wants to see.
- Q700 uses the direct VIA1-SR ADB path (no adbmodem). Our `adb_phy.v`
  replaces the ADB transceiver entirely; MAME's `macadb_device` is
  the behavioural spec.

**MMU (`m68kmmu.h`)**
- `M68K_MMU_SR_*` / `M68K_MMU_DF_*` / `M68K_MMU_TC_*` field constants
  — adopt these verbatim in our MMU module to avoid PRM
  reinterpretation errors.
- `pmmu_translate_address_and_rw()` — reference implementation of
  the full 68040 page-table walk. Not what we'll implement (we'll
  pipeline ours), but the **ordering of table-walk + descriptor-
  modified write-back + bus-error raise** is what matters.

### 4.3 When Tier 1 is not enough

Tier 1 fails you when:
- The ROM writes a sequence of register pokes in an order the
  PRM/NCR datasheet doesn't specify, and you don't know whether your
  order is wrong or the ROM's expectation is quirky.
- A multi-peripheral dance (IRQ assertion from VIA1 → CPU exception
  → MOVEC to mask → ACK to VIA1) fails and you can't tell which
  peripheral is guilty.

For those: Tier 2.

### 4.4 Cost + payoff estimate

- **Cost**: ~0 setup (no MAME build). Per-use cost: one agent
  hopping between two GitHub raw files. ~5–15 min per peripheral
  question.
- **Payoff**: catches maybe 60 % of the peripheral-bring-up bugs. The
  remaining 40 % need Tier 2.

---

## 5. Tier 2 — MAME trace oracle, replayed against Verilator

**Goal**: MAME is the golden. We boot our Q700 ROM in MAME with full
debugger tracing, capture a few seconds of execution + bus activity,
and write a harness that streams this trace into our Verilator
`mac_top` and diverges at the first mismatch.

**Status**: active. See `docs/rom_boot_bringup.md` §4.7 for the
current bringup script. `tb/tb_rom_boot.cpp` is the consumer side.

### 5.1 What MAME can log

From `docs/source/debugger/execution.rst` and `debugger/watchpoint.rst`:

**CPU instruction trace** — every PC, optionally every register state:

    trace maclog.tr,,,{tracelog "PC=%08X SR=%04X D0=%08X D1=%08X A0=%08X A7=%08X ", pc, sr, d0, d1, a0, a7}

(MAME's m68k core exposes `pc`, `sr`, `d0..d7`, `a0..a7`, plus
`fpcr/fpsr/fpiar` and `vbr/msp/isp/usp` as debugger expressions.)

**Memory-access trace** — watchpoint covering the entire address
space, no condition, action that prints the access + continues:

    wpset 0,0xFFFFFFFF,rw,1,{ printf "%s %08X data=%08X pc=%08X\n", wpdata == 0 ? "R" : "W", wpaddr, wpdata, pc; g }

(There is a subtler form using `wpi`/`wpd` to distinguish program vs
data space. For the 68040 Mac both are the same physical space, so
`wpset` on the program space is sufficient.)

**I/O region-specific watchpoint** — narrow to 0x50000000-0x50FFFFFF
and we get just the peripheral-register trace (which is smaller and
easier to diff against our `pb_addr`/`pb_wdata` stream):

    wpset 0x50000000,0x01000000,rw,1,{ printf "IO %s %08X=%08X pc=%08X\n", wpdata == 0 ? "R" : "W", wpaddr, wpdata, pc; g }

**IRQ assertion trace** — harder. MAME logs this via `-log` (general
error log) plus device-level `LOG_*` macros. For VIA1 IRQ lines
the macro is `LOG_GENERAL` in `6522via.cpp`, off by default. To
enable we recompile with `VERBOSE` set, OR we add a breakpoint on
the `via_irq` callback function address. The cleaner route:

    bpset irq_assert,1,{ printf "IRQ level=%d pc=%08X\n", irq, pc; g }

where `irq_assert` is the address of the `via1_irq` handler (found
via MAME debugger `print via1.irq_handler`).

**Combining into a single trace file** — debugger commands take an
optional `<action>` clause; chain with `;`. MAME also supports
`-debugscript <file>` on the command line to fire these commands at
startup:

    cat > trace.dbg <<'EOF'
    trace maclog.tr
    wpset 0x50000000,0x01000000,rw,1,{ tracelog "IO %c %08X=%08X pc=%08X\n", wpdata == 0 ? 'R' : 'W', wpaddr, wpdata, pc; g }
    go
    EOF
    mame_macoracle macqd700 -debug -debugscript trace.dbg -rompath roms -nothrottle -seconds_to_run 10 -str 10

This boots `macqd700` with our ROM (placed in `roms/macqd700/420dbff3.rom`)
headless, runs for 10 s of emulated time, and produces `maclog.tr`.
10 s of emulated Mac time ≈ 80 M cycles @ 8 MHz-equivalent bus, which
at ~100-byte-per-cycle text trace = ~8 GB. That's too big. Narrower
traces:
- CPU trace only, no memory wpset: ~1 GB per 10 s.
- I/O-only wpset (address 0x50000000 range), no CPU trace: ~5 MB
  per 10 s (the ROM only hits peripherals ~100× per millisecond).
- Capture 0.2 s instead: ~20 MB CPU trace, ~100 KB I/O trace. Plenty
  for initial divergence analysis.

### 5.2 Replay against our Verilator sim

The trace file is ASCII. A replay harness is a small C++ (or Python)
wrapper around `tb_top.cpp` / `tb_rom_boot.cpp` that, instead of
letting the DUT run freely, advances one commit at a time and checks:

1. Committed PC matches the next MAME `trace` line's PC.
2. Any AXI read/write in the last window matches the next MAME
   `wpset` line (same address, same direction, same data).
3. If either check fails: print last N MAME lines, dump last N
   committed ops from our `rob.v` (via the existing `CORE_DEBUG`
   hooks), diverge.

Rough shape of the replay harness:

    class MameTrace {
      vector<TraceLine> lines;
      size_t pos = 0;
      bool advance_pc(uint32_t committed_pc);
      bool check_bus(uint32_t addr, uint32_t data, bool is_write);
    };

    int main() {
      load_rom(ROM_PATH);
      MameTrace ref("maclog.tr");
      while (!done) {
        tick();
        if (commit_valid) {
          if (!ref.advance_pc(committed_pc)) fail();
        }
        if (axi_ar_fire || axi_aw_fire) {
          if (!ref.check_bus(...)) fail();
        }
      }
    }

### 5.3 Divergence handling

First divergence is almost always the most useful. Typical failure
modes and what they tell us:

- **PC drifts at a branch**: our BPU predicted wrong and didn't
  redirect fast enough, OR decode emitted wrong branch target, OR
  CCR was wrong so condition evaluated differently. The MAME trace
  line before divergence shows which Bcc condition and SR value.
- **Bus access address mismatch**: our AGU computed wrong EA.
  Usually indexed-mode index scaling or extension-word handling.
- **Bus access data mismatch on read**: our memory model is wrong
  (ROM aliasing? DDR ordering? write-buffer forwarding missed?).
- **Bus access data mismatch on write**: our execution is right but
  our ALU wrote the wrong value — back to the CCR rename or flag
  propagation code.
- **IRQ-level divergence**: a peripheral IFR bit wasn't set when it
  should have been. This is where Tier 2 shines over Tier 1 — MAME
  says "IRQ should have asserted at PC=0x40803F20 after this VIA1
  T1 write", and the next 10 instructions in our trace tell us
  whether our VIA1 is computing the expiry too late or never at all.

### 5.4 Determinism

MAME is deterministic by construction per emulated clock cycle,
**provided NVRAM + RTC are pinned**. Our agent run must:
- Use `-nvram_directory /dev/null` or a fixed empty dir to defeat
  PRAM state from previous runs.
- Use `-rtc` implementation from macrtc, which supports a fixed
  seconds counter for debug. (Alternatively: patch macrtc to start at
  a canned date; MAME devs have a patch for this but it's not
  upstream.)
- Use `-seconds_to_run` + `-nothrottle` so wall-clock doesn't matter.

### 5.5 Cost + payoff

- **Cost** (assuming MAME builds):
  - Day 1: build MAME, drop our ROM into `roms/macqd700/`, capture
    first 0.5 s trace, skim the trace file structure.
  - Day 2: write the replay harness (~400 LoC of C++).
  - Day 3: first divergence, understand what it tells us, iterate.
  - Day 4: productionise — trace becomes part of CI or a commit-gate
    regression.
  - Total: 3–4 agent-days for the initial harness. After that, each
    divergence-hunt session is 1–3 hours.
- **Payoff**: converts "this test fails and I don't know what reality
  looks like" into "this test fails at PC X with SR Y, here's the
  correct behaviour from MAME". Very high leverage on phase-2 ROM
  boot and phase-3 System 7 boot.

### 5.6 Caveats

- MAME's m68k core is also not a perfect 68040 — it's Musashi-derived
  internally (Karl Stenerud's implementation, the same Musashi we
  already cite as golden in `CLAUDE.md`). So MAME + our Musashi
  oracle **share a bug universe**. If MAME is wrong and Musashi is
  wrong in the same way (MMU corner cases, some FPU ops), the trace
  oracle can't tell us. For the ROM cold-start path the shared
  Musashi/MAME bugs are essentially zero relevance — the boot code
  uses documented, heavily-exercised instructions.
- MAME traces are large. Plan for storage + gzip.
- MAME's NCR 5380 model is good but not perfect — arbitration timing
  especially has been tweaked over the years. Don't trust SCSI trace
  divergence until the trace is against a known-good MAME commit.

### 5.7 Lockstep MMIO against RTL peripherals

The patched Q700 overlay can now run MAME's normal peripheral models
and the Verilated `peripheral_bus` endpoint on the same MMIO stream.
This is not DPI extraction of MAME devices; it keeps the normal MAME
machine running and mirrors each wrapped Q700 access to the RTL bridge.

Modes:
- Native trace only: set `MAME_RTL_MMIO_TRACE=/path/to/native.log`
  without `MAME_RTL_BRIDGE_SOCKET`. The overlay calls MAME's normal
  handlers and logs `mame-mmio mode=mame ...`.
- RTL-forwarding trace: set both `MAME_RTL_BRIDGE_SOCKET` and
  `MAME_RTL_MMIO_TRACE`. The overlay forwards reads/writes to RTL and
  logs `mode=rtl`.
- Lockstep compare: also set `MAME_RTL_LOCKSTEP=1`. Writes are sent to
  both MAME and RTL. Reads call both sides, log both values, and emit
  `mame-mmio-divergence ...` if the normalized read data differs.
  `MAME_RTL_READ_SOURCE=mame` returns MAME's read data to the emulated
  CPU so the reference boot can keep running while RTL divergences are
  collected. Omit it to return RTL data. `MAME_RTL_LOCKSTEP_FATAL=1`
  stops on the first mismatch; `MAME_RTL_LOCKSTEP_VERBOSE=1` mirrors
  mismatches to stderr.
- Lockstep reports normalized read mismatches as
  `mame-mmio-divergence ...`. `MAME_RTL_ACCEPT_VALIDATED_TIMING=1`
  enables a small allowlist for proven timing-only fields and logs those
  as `mame-mmio-accepted-divergence ...`; currently this is limited to
  VIA2 ORB PB7 when ACR routes Timer 1 onto PB7 and PB7 is the only
  differing bit. Leave it unset while hunting new device mismatches.

Current wrapped windows:
- VIA1 `0x50000000..0x50001fff`
- VIA2 `0x50002000..0x50003fff`
- ENET `0x50008000..0x50008007`
- SONIC `0x5000a000..0x5000b0ff`
- SCC `0x5000c000..0x5000dfff`
- ORWELL `0x5000e000..0x5000e0ff`
- SCSI `0x5000f000..0x5000f0ff`
- SCSI DMA `0x5000f100..0x5000f101`
- ASC `0x50014000..0x50015fff`
- SWIM `0x5001e000..0x5001ffff`
- VRAM aperture `0xf9000000..0xf91fffff`
- DAFB registers `0xf9800000..0xf98003ff`

When the bridge is disabled, or when lockstep is running with
`MAME_RTL_READ_SOURCE=mame`, wrapped windows must still execute MAME's real
device model. SONIC follows that rule too: the MAME side uses
`dp83932c_device::reg_r/reg_w`, while the RTL side now uses
`q700_eth_sonic.v` for the Ethernet PROM and the SONIC reset/config register
block. Descriptor DMA and packet movement are still future RTL work; that
path should terminate in the planned FPGAtaxi AXI-stream integration rather
than another harness-only model. Returning a fake `0xffff` from the MAME side
holds the ROM in the Ethernet probe path and prevents the no-disk display
loop from being reached.

DAFB in the bridge is the real RTL `video.v` register shim, not the old
generic AXI stub. The bridge selftest writes/reads framebuffer base and
stride and checks the monitor-sense readback. This validates the MMIO
front door.

The VRAM aperture is bridge-owned mmap-backed memory, enabled with
`--vram-shm PATH`. MAME CPU reads/writes at `0xf9000000..0xf91fffff`
are forwarded to the bridge and committed as big-endian bytes in that
2 MiB file. This is a Verilator/testbench sharing mechanism, not a
replacement for hardware VRAM: production RTL still uses `rtl/sys/vram.v`.
The point is to let MAME populate the same framebuffer bytes that an RTL
scanout/screenshot harness can sample at a chosen timeout or frame
milestone.

For visual RTL verification of the DAFB/CLUT/scaler path, use the
standalone scanout wrapper:

```
DAFB_SCANOUT_PPM_DIR=build/dafb_scanout/frames make tb-dafb-scanout
```

The generated PPM frames are produced from the real `video.v` CLUT state,
the live DAFB base/stride/BPP outputs, `fb_reader`, `linebuf_scanout`,
and the scaler path. The current frame source is a deterministic
testbench VRAM pattern. The next bridge step for full MAME screenshots is
to forward the `0xf9000000..0xf91fffff` VRAM aperture into an RTL VRAM
instance, then trigger this same frame dump at timeout or at a chosen
MMIO/cycle milestone.

Example:

```
make mame-axi-periph-bridge
python3 tools/mame_q700_rtl_overlay.py /tmp/mame
make -C /tmp/mame SUBTARGET=macrtl SOURCES=src/mame/apple REGENIE=1 TOOLS=0 -j4

rm -f /tmp/mame-axi-periph-bridge.sock
build/mame_axi_periph_bridge/Vmame_axi_periph_top \
    --socket /tmp/mame-axi-periph-bridge.sock --max-cpu-advance 32768 \
    --vram-shm build/mame_runs/q700_vram.bin

MAME_RTL_BRIDGE_SOCKET=/tmp/mame-axi-periph-bridge.sock \
MAME_RTL_LOCKSTEP=1 \
MAME_RTL_READ_SOURCE=mame \
MAME_RTL_BRIDGE_LABELS=VIA1,VIA2,SCSI,SCSI\ DMA,SWIM,DAFB,VRAM \
MAME_RTL_TRACE_LABELS=VIA1,VIA2,SCSI,SCSI\ DMA,SWIM,DAFB,VRAM \
MAME_RTL_MMIO_TRACE=build/mame_runs/lockstep_mmio_trace.log \
    /tmp/mame/macrtl macqd700 -rompath /tmp/mame-roms \
    -bench 1 -watchdog 5 -skip_gameinfo -noreadconfig \
    -video none -sound none

make mame-vram-scaler-dump
build/mame_vram_scaler_dump/Vtb_mame_vram_scaler_dump \
    +vram_image=build/mame_runs/q700_vram.bin \
    +ppm=build/mame_runs/q700_scaler.ppm \
    +vram_bpp=1 +vram_stride=1024 +vram_base=0x1000
```

For a fast oracle snapshot without paying the socket cost for every VRAM
write, run patched MAME natively to the no-disk loop and dump the CPU-visible
DAFB aperture with Lua:

```
rm -rf build/mame_runs/native_vram_dump
mkdir -p build/mame_runs/native_vram_dump/nvram
MAME_RTL_VRAM_DUMP=build/mame_runs/native_vram_dump/vram.bin \
    /tmp/mame/macrtl macqd700 -rompath /tmp/mame-roms \
    -nvram_directory build/mame_runs/native_vram_dump/nvram \
    -nothrottle -video none -sound none -skip_gameinfo -noreadconfig \
    -autoboot_delay 60 -autoboot_script tools/mame_dump_vram.lua

build/mame_vram_scaler_dump/Vtb_mame_vram_scaler_dump \
    +vram_image=build/mame_runs/native_vram_dump/vram.bin \
    +ppm=build/mame_runs/native_vram_dump/scaler.ppm \
    +vram_bpp=1 +vram_stride=1024 +vram_base=0x1000
```

The `0x1000` base is the visible framebuffer base programmed through the
DAFB base registers for the current Q700 no-disk loop. Dumping from raw base
zero is still useful for checking early fill traffic, but it is not the
visible insert-floppy frame.

The larger `--max-cpu-advance` is needed for VIA timer lockstep: the ROM
uses timer periods longer than 4096 68040 cycles, and clipping those gaps
turns real timer matches into artificial IFR phase divergences.  Include
VIA2 whenever VIA1 CA1 is under test; on the Q700, MAME drives VIA1 CA1
from VIA2 PB7.

To fast-forward to display traffic while keeping the CPU-visible machine
state native, add `MAME_RTL_BRIDGE_LABELS=display`. This forwards only
DAFB and VRAM accesses to the RTL bridge. All other wrapped windows still
run through MAME's normal device models and are omitted from lockstep
comparison. This is a targeted display/VRAM exerciser, not a full-platform
co-sim result.

For exact filtering, `MAME_RTL_BRIDGE_LABELS` also accepts comma-separated
trace labels such as `DAFB,VRAM,SCC`. The default and `all` preserve the
full wrapped-window bridge behavior.

For long probes, `MAME_RTL_TRACE_LABELS` uses the same syntax but only
controls trace-file logging. For example, pair
`MAME_RTL_BRIDGE_LABELS=display` with `MAME_RTL_TRACE_LABELS=display` to
produce a compact DAFB/VRAM-only trace while native MAME peripherals keep
the machine moving.

For interrupt validation, add `MAME_RTL_IRQ_TRACE=1` with
`MAME_RTL_MMIO_TRACE` enabled. The bridge stamps each MMIO response with
an RTL IRQ bitmap:

| Bit | Meaning |
| --- | --- |
| 0 | VIA1 IRQ |
| 1 | VIA2 IRQ |
| 2 | SCC IRQ |
| 3 | raw SCSI IRQ before VIA2 board glue |
| 4 | raw ASC/EASC IRQ before VIA2 board glue |
| 5 | raw IWM/SWIM IRQ |

The overlay logs `mame-irq` lines containing MAME's CPU-facing IRQ class
bitmap (`VIA1/VIA2/SCC`) and the RTL bitmap. Treat this as an event-order
check, not a cycle-exact check: VIA timers, ASC FIFO service, and stream
update scheduling can differ in phase. The Q700-accurate topology is ASC
IRQ -> VIA2 CB1 and SCSI IRQ -> VIA2 CB2/PA status; SCC is the direct
CPU IPL4 source. A passing IRQ validation means the same CPU-facing class
asserts and clears around the same MMIO frontier, while raw ASC/SCSI bits
may lead VIA2 by a small, justified phase window.

After a run, summarize and validate the IRQ trace with a snapshot lag
window:

```
python3 tools/analyze_mame_irq_trace.py build/mame_runs/lockstep_mmio_trace.log --lag 64
```

This reports per-class high counts and fails if raw ASC/SCSI assertions do
not reach RTL VIA2, or if MAME CPU-facing VIA1/VIA2/SCC assertions and
clears do not have a matching RTL class event within the allowed MMIO
snapshot lag.

For two separate native/RTL traces, `tools/compare_mame_mmio_traces.py`
reports the first normalized divergence:

```
python3 tools/compare_mame_mmio_traces.py \
    build/mame_runs/native_mmio_trace.log \
    build/mame_runs/rtl_mmio_trace.log
```

---

## 6. Tier 3 — DPI co-sim of MAME peripheral classes (NOT recommended)

**Goal considered, rejected**: link MAME's `6522via.cpp`,
`ncr5380.cpp`, `z80scc.cpp`, `asc.cpp`, `macrtc.cpp` into a
Verilator DPI shim so our `mac_top_tb` calls the MAME C++ models on
every peripheral register access.

### 6.1 Why it sounds appealing

- Zero-coverage peripherals become fully correct peripherals instantly.
- We could rip out our VIA1/VIA2/SCSI/SCC Verilog and just co-sim
  against the MAME C++ until we're ready to implement the RTL. Kind
  of like a Fast Model for the Mac platform.
- The RTL we write is validated directly against the oracle at every
  register access, not at a distant point downstream.

### 6.2 Why it doesn't work in practice

MAME devices are wedded to the MAME infrastructure in ways that make
standalone extraction very expensive:

1. **`device_t` + `machine_t` dependency**: every device has
   `device_start() / device_reset()` lifecycle methods called by
   `running_machine`. To use `via6522_device` standalone you either
   instantiate a full `running_machine` (drags in `src/emu/`, ~50 MB
   of code) or you stub out ~20 virtual methods (feasible but tedious
   per device).
2. **`devcb` callback framework**: IRQ lines, port handlers, clock
   inputs all go through `devcb_write_line` / `devcb_read8` template
   machinery. Rewriting these into plain function pointers is a
   per-device port effort.
3. **`attotime` + `emu_timer` scheduling**: MAME devices use a global
   scheduler for timer callbacks (VIA T1 tick, NCR DMA advance). You
   either simulate the scheduler or manually call `execute_run` with
   cycle counts. Either way, you're shimming the emulator's core loop.
4. **`machine_config`**: devices assume they're instantiated via
   MAME's config builder. Bypassing it means providing mock parent
   devices.

Realistic extraction cost:
- Minimal VIA6522 standalone shim (no timers, just register file +
  IFR/IER): ~1 agent-day. Useful as a reference but isn't what we
  need.
- Full VIA6522 with timers: ~3 agent-days. Timers need the scheduler.
- NCR 5380 + nscsi bus: ~1 agent-week (nscsi_bus, nscsi_device_
  interface, target-device registry).
- Five peripherals total: ~3–4 agent-weeks for the extraction layer
  plus DPI plumbing.

### 6.3 Why Tier 2 beats Tier 3

Tier 2 runs MAME as a full emulator and captures its outputs. It does
not care how MAME's internal machinery is wired — a trace file is
opaque. This has the same oracle value as Tier 3 (both check our
behaviour against MAME's behaviour on the same ROM) for ~10 % the
effort.

The one scenario where Tier 3 wins: if we wanted to co-simulate our
CPU against MAME's **peripherals** without ever letting MAME see the
CPU — e.g. we're just running MAME's VIA1 model as a verif-IP behind
a DPI wall. That's a real verif pattern. But the trace-replay
approach covers it at much lower cost.

### 6.4 Recommendation

**Skip Tier 3.** If we ever hit a scenario where Tier 2 isn't enough
— specifically, if we need interactive stepping of MAME peripherals
in lockstep with our sim (not batch replay) — revisit.

---

## 7. Quick-reference cheatsheet

**MAME repo + pin**: `https://github.com/mamedev/mame` @ `mame0287`
(March 2026 stable).

**Apt packages (Ubuntu 24.04)**:
```
sudo apt-get install git build-essential python3 \
    libsdl2-dev libsdl2-ttf-dev libfontconfig-dev
```

**Subtarget build**:
```
git clone --depth 1 --branch mame0287 https://github.com/mamedev/mame.git /tmp/mame
cd /tmp/mame
make SUBTARGET=macoracle SOURCES=src/mame/apple REGENIE=1 -j4
```
Slower than the old 16-thread recipe, ~4 GB build dir, ~50 MB binary.

**Machine to boot**: `macqd700` (Quadra 700) — requires
`420dbff3.rom` (stored-checksum `0x420dbff3`, 1 MB) in
`roms/macqd700/`. The committed `files/420dbff3.rom` is the exact
match; copy it into place with:

    mkdir -p roms/macqd700
    cp /path/to/m68k-ooo/files/420dbff3.rom roms/macqd700/420dbff3.rom

**Debugger trace (`trace.dbg`)**:
```
trace cpu.tr,,,{tracelog "PC=%08X SR=%04X D0=%08X A7=%08X ", pc, sr, d0, a7}
wpset 0x50000000,0x01000000,rw,1,{ tracelog "IO %c %08X=%08X pc=%08X\n", wpdata == 0 ? 'R' : 'W', wpaddr, wpdata, pc; g }
go
```

**Invocation**:
```
./mame_macoracle macqd700 -debug -debugscript trace.dbg \
    -rompath roms -nothrottle -seconds_to_run 0.5
```

**Key MAME files** (paths under `mamedev/mame` master):
- `src/devices/machine/6522via.cpp` / `.h` — VIA1 + VIA2 (Q700 uses
  two real discrete 6522s).
- `src/devices/machine/ncr5380.cpp` / `.h` — SCSI.
- `src/devices/machine/z80scc.cpp` / `.h` — SCC.
- `src/devices/sound/asc.cpp` / `.h` — Apple Sound Chip (Q700 =
  SONORA variant).
- `src/mame/apple/macrtc.cpp` / `.h` — RTC + PRAM.
- `src/mame/apple/macscsi.cpp` / `.h` — Apple pseudo-DMA wrapper
  around NCR 5380.
- `src/mame/apple/macquadra700.cpp` — our Q700 driver. Address
  decoder + IRQ aggregator ≈ our `glue.v`; the `memory_map` /
  `irq_map` tables here are the spec.
- `src/mame/apple/dafb.cpp` / `.h` — Q700 video block.
- `src/mame/apple/macadb.cpp` / `.h` — ADB transceiver. Phase-3.
- `src/devices/cpu/m68000/m68kmmu.h` — 68040 PMMU field layouts.

**Alternate-ROM references** (optional, not our target):
- `src/mame/apple/macquadra800.cpp` — Q800/Centris 650 driver;
  `f1acad13.rom` (CRC `4e70e3c0`) if you want to compare how a
  later IOSB+djMEMC chipset exercises the same devices.

---

## 8. Open questions (for the next research pass)

1. Is there a canonical pinned `-seconds_to_run` + `-debugscript`
   incantation that MAME devs use for regression tracing? If yes,
   adopt it verbatim instead of rolling our own.
2. Can MAME's SCSI `nscsi_bus` hold a backing-store disk image at a
   known checksum? We want the trace + disk image to be a single
   pinned artifact, not "whatever NVRAM MAME happened to save last
   run".
3. Does MAME's m68040 core model the ATC / page-table walker
   identically to real 68040 silicon? The commentary in `m68kmmu.h`
   notes "PMMU implementation for 68851/68030/68040" — same code
   path for all three, which is suspicious. Verify before leaning
   on MAME for MMU-behaviour oracling in phase 3.
4. Is there a pre-made Mac System 7 disk image (in MAME's
   `software_list` format) that works with `macqd700`? That would
   give us a reproducible phase-3 boot target without needing to
   construct our own SCSI-HDD image.

These are "ask during the next research pass" items; none block
phase-2 work.
