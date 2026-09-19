# Core gaps — what's missing to run real Mac OS

Companion to [gameplan.md](gameplan.md). This is the concrete inventory of
things the core does not yet do, organised by category and prioritised by
when they're needed.

Status legend: ❌ not started · 🚧 partial · ⚠️ correctness risk · ✅ done

---

## 1. Instruction coverage gaps

### High-priority (needed for ROM cold-start)

| Instruction | Status | Why ROM needs it |
|---|---|---|
| MOVEC src,creg / MOVEC creg,src | ❌ | Sets VBR (vector base), CACR, ITT0/1, DTT0/1 within first ~20 instructions |
| MOVES src,dst / MOVES dst,src | ❌ | Move between address spaces (FC select). Used by MMU init |
| CPUSH (Bc/Dc/Ic) #cl, (An) | ❌ | Cache flush. Required to switch out of overlay mode |
| CINV (Bc/Dc/Ic) #cl, (An) | ❌ | Cache invalidate. Required after RAM probes |
| TRAP #n | ❌ | Mac OS Toolbox uses TRAP #15 for debugger entry |
| RTE | ❌ | Return from exception — pops format/PC/SR. Used by every interrupt handler |
| STOP #imm | 🚧 | Currently translated to NOP. Real STOP halts CPU pending IRQ |
| RESET | ❌ | Drives bus RESET line. Mac ROM uses it to reset peripherals during init |
| BSR (with longer disp), BCC long disp | 🚧 | 16-bit forms work; some ROM code uses 32-bit form (68020+) |
| MOVEM <list>,(An)+ / MOVEM -(An),<list> | ❌ | Function prologue/epilogue throughout the ROM. Long instruction (4+2N bytes) |
| LINK An, #-disp | ❌ | C-style frame setup; ROM uses for some routines |
| UNLK An | ❌ | Frame teardown |
| Scc <ea> | ❌ | Set byte conditional. Used by ROM for boolean flags |
| TRAPV | ❌ | Trap if V; rarely used but not zero |
| CHK / CHK2 | ❌ | Bounds check. Compiler-generated; Pascal-heavy ROM uses it |

### Medium-priority (needed during boot, not at cold start)

| Instruction | Status | When it shows up |
|---|---|---|
| BFTST/BFCHG/BFSET/BFCLR/BFINS/BFEXTS/BFEXTU/BFFFO | ❌ | Bitmap drawing in QuickDraw |
| TAS | ❌ | Synchronisation in driver init |
| CAS / CAS2 | ❌ | Atomic ops in some MultiFinder paths |
| PACK / UNPK | ❌ | BCD conversion (rare) |
| MOVEP | ❌ | Used by ADB driver to talk to VIA |
| ABCD / SBCD / NBCD | ❌ | BCD arithmetic; rare but Toolbox uses for Date/Time format |

### FPU coverage (phase 3+)

| Instruction | Status | Notes |
|---|---|---|
| FNOP | ❌ | INITs use this to detect FPU presence |
| FMOVE / FMOVECR | ❌ | Constant load; ROM tests with FMOVECR |
| FADD / FSUB / FMUL / FDIV | ❌ | Basic arithmetic |
| FSQRT / FABS / FNEG / FNOP | ❌ | Unary ops |
| FCMP / FTST | ❌ | Compare → FPCC update |
| FBcc / FScc / FDBcc / FTRAPcc | ❌ | FPU branches (use FPCC, not CCR) |
| Transcendentals (FSIN, FCOS, etc.) | ❌ | 68040 implements via FPSP package — many trap to software |
| FMOVEM | ❌ | FP register save/restore |

**Critical insight on the 68040 FPU:** the real 68040 chip implements only
a subset of FPU instructions in hardware (FADD, FSUB, FMUL, FDIV, FSQRT,
basic moves). All transcendentals (FSIN/FCOS/FETOX/FLOGN/etc.) generate an
F-line trap and Apple's FPSP (Floating Point Software Package) handles them
in software. **We can do the same** — implement the core FPU set in HW and
let Mac OS install FPSP for the rest. This dramatically simplifies phase 3.

---

## 2. Exception handling — the boot-blocker category

The 68040 has 256 exception vectors. Each must be precisely dispatchable.
The current core has zero exception infrastructure.

### Exception machinery to build

- **VBR register** (Vector Base Register): MOVEC writes to it, exception
  vectors look up `[VBR + vector_num × 4]` in supervisor data space.
- **Stack frame builder**: pushes frame format word, faulted PC, SR onto
  current stack. Format depends on exception type (4-word, 6-word,
  bus-error, etc.).
- **Privilege transition**: switches to supervisor mode (sets SR.S), if
  user→sup transition, also switches stack pointer (USP→SSP/ISP).
- **Mask interrupts**: sets SR.I to the level of the taken interrupt.
- **Squash + redirect**: like a branch mispredict but with arch state
  rollback (RAT to cRAT, free in-flight phys regs, flush ROB tail).

### Vectors that matter most

| # | Vector | Trigger | Frequency in Mac OS |
|---|---|---|---|
| 0 | Reset SP | reset (loaded from ROM) | once |
| 1 | Reset PC | reset (loaded from ROM) | once |
| 2 | Bus error | AXI BRESP/RRESP non-OKAY, missing TLB walk | rare |
| 3 | Address error | misaligned word/long access | rare in well-formed code |
| 4 | Illegal instruction | unrecognized opword | rare (Mac OS uses A-line for traps, not illegal) |
| 5 | Zero divide | DIVS/DIVU with divisor 0 | rare |
| 6 | CHK / CHK2 | bounds check fail | rare |
| 7 | TRAPV / FTRAPcc | overflow trap | rare |
| 8 | Privilege violation | supervisor inst in user mode | ⚠️ critical for ROM that drops to user briefly |
| 9 | Trace | T-bit set in SR | debugger only |
| 10 | A-line trap (1010) | opword[15:12] = 1010 | **EVERY MAC OS TOOLBOX CALL** |
| 11 | F-line trap (1111) | opword[15:12] = 1111 | every unimplemented FPU op |
| 24–31 | Spurious / autovector / lvl 1–7 IRQ | external IRQ pin | constant (60Hz tick from VIA1) |
| 32–47 | TRAP #0–15 | TRAP #n | TRAP #15 = MacsBug entry |
| 48–63 | FPU-related (FBSUN, FOPERR, etc.) | FPU exception | per-FPU op |
| 64–255 | User-defined | OS use | varies |

**Vec-4 ILLEGAL default-case design (post-task #134).**  Decode treats
every unmatched opword as an ILLEGAL instruction per 68040 PRM §8.4 —
the default arm of the top-level opword casez emits `UOP_SYS` with
`exc_valid=1, exc_vec=8'd4` instead of the pre-#134 silent `UOP_NOP`
fallback.  The previous behaviour masked decode gaps (e.g. the #131
MOVE.B byte-length bug slipped past CI because unhandled byte-size
moves quietly retired as NOPs, leaving a zero destination instead of
a faulting PC pointing at the offending instruction).  Unit tbs now
observe a vec-4 trap at the exact PC of any undecoded op: the ROM-boot
harness halts with a precise PC+opword signature, `tb-exception`'s
`test_illegal_opcode_trap` scenario drives the Format-0 frame end-to-
end, and `tb/tests/asm/illegal_opcode_trap.s` pairs with the
pre-existing `exc_illegal.s` to cover both the canonical 0x4AFC
trigger and the generalised default-case path.  `commit.v` prints
`[COMMIT-ILLEGAL cyc=X] vec=V pc=P opword=O` under `CORE_DEBUG` on
every vec-4 / A-line (vec-10) / F-line (vec-11) retirement so a
fresh gap is diagnosable in one grep.

**A-line trap is the most performance-critical exception in Mac OS.** Every
QuickDraw call, every File Manager call, every Memory Manager call goes
through an A-line trap. The hot path must be:

1. Detect opword[15:12] == 1010 in decode
2. Squash everything younger in ROB
3. Build 4-word stack frame (format $0)
4. Jump to `[VBR + 0x28]`
5. Resume

Cost target: ~12 cycles entry, ~10 cycles RTE exit. Every cycle saved here
multiplies across the entire boot path.

---

## 3. Supervisor mode

Currently nonexistent. Mac OS spends ~70% of cycles in supervisor mode
(Toolbox + kernel). This is required for everything in phase 2.

### What needs building

- **SR (Status Register)** as architectural state. Currently we only have CCR
  (low byte of SR). Add: T (trace), S (supervisor), M (master/interrupt
  switch), I[2:0] (interrupt mask).
- **USP / SSP / ISP** separation. Three stack pointers; A7 is selected by
  current SR.S and SR.M:
  - User mode: A7 = USP
  - Supervisor + M=0: A7 = ISP (interrupt stack)
  - Supervisor + M=1: A7 = SSP (master stack)
  - The unselected stack pointers are stored in shadow regs, accessed via
    MOVEC USP / MOVE USP,An / MOVE An,USP.
- **Privilege check** in decode for supervisor-only instructions:
  STOP, RESET, RTE, MOVEC, MOVES, MOVE to/from SR (read SR is supervisor
  on 68010+), CPUSH/CINV.
- **MOVE to/from SR** — sets/reads the entire SR including supervisor bits.
- **OR/AND/EOR #imm,SR** — bit manipulation of SR (supervisor only).
- **OR/AND/EOR #imm,CCR** — bit manipulation of CCR (user mode OK).
- **Renaming the SR**: only CCR (low byte) is renamed via the upcoming
  CCR-PRF. The supervisor bits (T, S, M, I) are committed in-order in
  `commit.v` since they affect dispatch (privilege check) and exception
  handling, and rarely change.

---

## 4. MMU / address translation

Required for any non-trivial Mac OS path. The 68040 has on-chip MMU; software
configures it via MOVEC + PMOVE-equivalent instructions.

### Components

- **TC (Translation Control register)**: enables MMU, sets page size (4KB
  or 8KB), root pointer location.
- **ITT0 / ITT1 / DTT0 / DTT1** (Transparent Translation registers):
  4 windows that bypass page tables. ROM uses these to map ROM and I/O
  ranges 1:1 without setting up page tables.
- **URP / SRP** (User / Supervisor Root Pointer): top-level page table base.
- **64-entry ATC** (Address Translation Cache): the 68040 TLB.
- **Page table walker**: 3-level walk on ATC miss. Hardware-driven (the
  68040 walks tables in hardware; the page format is fixed).
- **PFLUSH / PTEST**: cache management for ATC.

### Bring-up order

1. **ITT0/DTT0 only**: cover ROM (0x40000000–0x40FFFFFF) and I/O
   (0x50000000–0x50FFFFFF) as 1:1 transparent. RAM (0x00000000) might
   also be 1:1 initially.
2. **Page table walker**: required when Mac OS sets up its own page tables
   to remap RAM, set protection, etc. This happens in the System file
   setup, not in the ROM.
3. **Full ATC + walker**: phase 3.

**Risk**: the 68040 page table format is rigid. Software (Mac OS) builds
trees in a specific format and the hardware walker must match exactly.
Reference: 68040 User's Manual chapter 3.

---

## 5. Cache subsystem

Currently: nothing. Reads go straight to AXI; writes too. This is fine for
the bring-up sim but a non-starter for FPGA Fmax and for Mac OS performance.

### I-cache (4KB, 4-way, 32B lines)

- BRAM-backed: 1 RAMB36 per way × 4 ways = 4 BRAMs for data, plus tags.
- Refill on miss: read 32B from AXI (8 beats × 32-bit), write to all-of-way.
- Critical-word-first: deliver the requested word to fetch immediately,
  fill the rest in background.
- LRU: pseudo-LRU with 3 bits per set (4-way).
- Invalidate: CINV instruction line, CINV instruction page, CINV all,
  CPUSH instruction (degenerate — I-cache is read-only, CPUSH = CINV).
- **Snoop on D-cache write** (self-modifying code): on D-cache store to
  address that hits I-cache, invalidate the I-cache line. Mac OS uses SMC
  in driver patching paths.

### D-cache (4KB, 4-way, 32B lines, write-back)

- BRAM-backed similarly.
- Write-back, write-allocate: hits write the cache; misses fill then write.
- MESI bits per line: M (modified), E (exclusive), S (shared), I (invalid).
  Single-core for now → only M and I matter; S/E for future SMP.
- Refill on miss: same path as I-cache.
- Writeback on eviction of M line: write 32B back to AXI.
- **CPUSH** (clean): writes back M lines to memory, leaves them clean.
- **CINV** (invalidate): drops lines without writeback (dangerous if M).
- **CPUSH+CINV combo**: writeback then invalidate. ROM uses to ensure RAM
  visible to DMA peripherals.

### Cache coherence with peripherals

**This is the biggest correctness risk in phase 3.**

SCSI DMA writes to RAM. The CPU has cached copies of those RAM lines. If
the CPU reads from a cached line that DMA just wrote, it gets stale data.

Three approaches, in increasing complexity:
1. **Software-managed**: Mac OS does CPUSH/CINV around DMA. The 68040 driver
   model assumes this. **Sufficient for booting.**
2. **Hardware snooping**: peripherals broadcast addresses on a snoop bus,
   D-cache invalidates matching lines. 68040 supports MI/MA pins for this
   on real hardware. **Phase 4+.**
3. **Cache coherent interconnect**: AXI4 + ACE protocol. **Way overkill.**

Phase-3 plan: assume software-managed. Validate by stress-testing CPUSH
correctness with a directed test that mimics DMA-then-read.

---

## 6. Self-modifying code

Mac OS does this in several places:
- A-line trap dispatcher patches itself when a TRAP word is "registered"
- INIT chains modify Toolbox vectors (which point to code patches)
- Driver loaders modify their own trampolines

**Mechanism:** any D-cache write that hits an I-cache line must invalidate
the I-cache line. This is a snoop from D-cache to I-cache, single-direction
(I-cache never writes).

**Implementation cost:** one comparator per I-cache way per D-cache write.
Single-cycle snoop. Trivial in BRAM.

---

## 7. Bus interface details

Currently: AXI4 master, single transaction in flight (LSU is single-issue),
no bursts, 32-bit data.

### Phase 2/3 needs

- **Bursts** for cache refill: 8-beat INCR8 transactions for 32B lines.
- **Multi-outstanding loads** (already on wip branch): N_LD=4 slots with
  ARID tagging.
- **Read-and-write-strobe**: D-cache writeback uses bursts with WSTRB.
- **AXI exclusive access** (AXLOCK) — for TAS / CAS atomics.

### Address error vs bus error

- **Address error**: `request misaligned` (word at odd address, long at
  non-4-aligned). Detected at AGU output, before AXI. Vector 3.
- **Bus error**: AXI BRESP/RRESP returns SLVERR or DECERR. Detected on
  response. Vector 2. The 68040 generates a complex stack frame for bus
  error (12-word format $7) to allow OS recovery.

---

## 8. Interrupts

Currently: zero interrupt path. Phase 2 needs minimal interrupt support
because VIA1 Timer 1 fires at 60 Hz and ROM enables it.

### Components

- **IRQ pins**: 3 input wires `irq_level[2:0]` representing the priority
  level of pending interrupts (1–7; 0 = no interrupt).
- **IRQ acknowledge cycle**: 68040 issues an IACK transaction. For
  autovectored peripherals (VIA, SCSI), the vector number is computed as
  24 + level. For vectored peripherals (some NuBus devices), the device
  drives the vector.
- **Mask comparison**: pending IRQ level must be > SR.I.
- **Atomic SR update on entry**: SR.I gets set to taken IRQ level
  (prevents nested same-level interrupts).
- **Stack frame**: 4-word format $0 for autovector.

### Mac OS interrupt sources at boot

| Level | Source | Frequency |
|---|---|---|
| 1 | VIA1 (timer, ADB) | 60 Hz from Timer 1 |
| 2 | VIA2 (slot interrupts) | rare during boot |
| 3 | SCC (serial) | rare during boot |
| 4 | Sound DMA | not until audio init |
| 5 | NMI / interrupt switch | manual only |
| 6 | NuBus / video | depends on slot config |
| 7 | NMI | manual only |

Phase 2 only needs level 1 (VIA1 Timer 1). Everything else can be deferred.

---

## 9. Misc subtle gotchas

- **Read-modify-write atomicity**: TAS, CAS, CAS2 must be atomic against
  bus accesses. AXLOCK on AXI handles this for sim. On FPGA, peripherals
  may not honor lock — need to check.
- **PC-relative + indirect addressing**: `(d8,PC,Xn)` and `([bd,PC,Xn],od)`.
  PC sample is the PC of the extension word. We currently support a few
  PC-relative forms but not the full extension word format.
- **Indirect with index**: `(d8,An,Xn.SIZE*SCALE)` and full extension word
  `([bd,An,Xn],od)`. Compiler-generated for array access. Decode complexity.
- **MOVEM register list ordering**: predecrement form reverses the register
  list (D7 first, A0 last). Easy to get wrong.
- **SP wrap-around on PUSH/POP**: A7 is 32-bit, no special wrap; but Mac OS
  sometimes uses a guard word at top of stack.
- **PC sampled into stored frame is "incremented PC" or "current PC"
  depending on exception type**. The 68040 manual table 8-2 specifies which
  PC value goes in the stack frame. Get this wrong → RTE returns to wrong
  address.

---

## 10. Summary: priority order for phase 2

1. **MOVEC + VBR** (5 lines decode + 1 reg in commit) — unblocks vector setup
2. **Exception entry skeleton** (squash + RAT rollback + push frame + jump to vec) — unblocks every other exception
3. **A-line trap** (single new decode case + exception entry) — unblocks all Toolbox
4. **Privilege checking + supervisor mode** — unblocks ROM running its supervisor code
5. **CPUSH/CINV** (mostly NOP if no cache; real flush after phase 3 cache lands)
6. **MMU ITT0/DTT0** (transparent translation; 5 LUTs, no walker yet)
7. **MOVEM crack** (uses existing multi-uop mechanism from BSR)
8. **RTE** (counterpart to exception entry)
9. **TRAP #n** (sub-case of exception entry)
10. **Interrupt input pin + level 1 dispatch** — unblocks VIA1 60Hz

After these the ROM can run cold-start through milestone 2.7 with high
probability.
