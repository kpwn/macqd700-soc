# Bug B — A-trap dispatch divergence (Q700 boot)

**Status (2026-05-04):** root cause not yet pinned to a specific RTL bug. All four directed tests landed this session pass on RTL and on Musashi golden-ref. The bug appears HW-specific (not reproducible in Verilator sim).

## TL;DR

- HW boot diverges inside the BlockMove A-trap dispatcher at ROM `0x408099B0`, between cap[226] (A-line at `0x4081BFDE`) and cap[227] (FIRST F-line at `0x253C`) — a 403-instruction window.
- The dispatcher's exit RTS pops the wrong return PC and lands in ROM zero-pad at `0x40885036..0x408853FF`. Zero-pad walker eventually hits real code at `0x40885414..0x40885418` (`addq.w #4, sp; rts`) which pops `0x252E` from the stack (BlockMove A0 source pointer) — that lands at low-RAM data which decodes as F-line (`0xFFFF`) and triggers FPSP recursion at `0x4088D244` (FSF, unsupported in our partial FPU body) — infinite recursion at 212 bytes/iter — eventually bus-error storm at `0x408026F0`.
- Sim (Verilator + `tb-fpga-top-rom`) doesn't reach this point — diverges earlier in a peripheral poll at `0x40847AC0..0x408478B4` (~65k iterations) and never executes ~10.97M instructions to where the bug fires.
- All BlockMove dispatcher *building blocks* tested in isolation work correctly — see test results table below.

## Directed-test results (this session)

Every test runs identically on our RTL (Verilator) and Musashi (golden ref):

| # | Test | RTL | Musashi | Coverage |
|---|------|-----|---------|----------|
| 1 | `atrap_overlay_unaligned_long_store.s` | PASS | (RTL match) | Replicates the dispatcher's `MOVE.L A2, 20(SP)` rts-trick at 2-byte-aligned 4-byte boundary. 4 iterations + back-to-back stress. |
| 2 | `jmp_pc_d8_dn_wscale2_table.s` | PASS | PASS | Replicates BlockMove handler's `4EFB 0284` brief-format `JMP (d8,PC,Dn.W*2)` jump-table dispatch. 16-test sweep including high-bits truncation, sign-ext, negative .W, 8-shot back-to-back hazard test. |
| 3 | `jsr_preindexed_memind_atrap_table.s` | PASS | PASS | Replicates the dispatcher's `4EB0 25A1 0400 = JSR ([0x400 + D2.W*4])` mem-indirect preindexed full-format JSR. 14-test sweep with same .W / hazard coverage. |
| 4 | `atrap_full_dispatch_replay.s` | PASS | PASS | Full ROM dispatcher replay — VBR-relocated A-line entry, 4-reg saves, unaligned overlay, mem-indirect JSR to fake handler doing 8× `MOVE.L (A0)+,(A1)+` + SUB+BGE inner loop, dispatcher unwind, addqw + RTS-trick. 4 iterations. Most ROM-faithful test possible without booting full ROM. |

**Implication:** the bug is NOT in any single instruction or addressing-mode crack. The dispatcher's individual primitives all behave correctly in our RTL. The bug must be in a HW-specific interaction not reproducible in Verilator: **strongest remaining candidates**

1. **IRQ delivery during handler** — boot non-determinism (same N gives different states across cold boots) is consistent with VIA1 IRQs arriving at sensitive cycles inside the dispatcher. Sim has no real peripheral IRQ generation.
2. **DDR/cache timing** — sim uses simplified DDR/cache models. Real KU5P has variable-latency MIG plus L1 pass-through wrappers; a stale-read from store-buffer or cache-coherence corner could cause the dispatcher's RTS to read pre-overlay bytes.
3. **MMU/TLB interaction** — under boot the relocated VBR + supervisor-mode page mappings could expose a TLB invalidation race.

## Symptom

Q700 ROM boot under our 100 MHz JTAG bitstream reaches a bus-error storm at PC=`0x408026F0` with A7=`0xFFF8002E`.  Storm is downstream of an F-line recursion at `0x4088D244` (FSF instruction in FPSP), which is downstream of the FIRST F-line firing at PC=`0x253c` (low-RAM data executed as code), which is downstream of an A-trap dispatch chain that landed in ROM zero-pad at `0x40885036..0x408853FF`.

MAME reaches the disk-prompt cleanly using the same ROM image (SHA1 `7a8ee468d16e64f2ad10cb8d1a45e6f07cc9e212`, 1 MB, stored-checksum `0x420dbff3`).  130 successful BlockMove A-trap dispatches at `0x4081BFDE` complete normally in MAME — never lands in zero-pad, never hits F-line.

## Cascade (downstream → upstream)

1. Bus-error storm at 0x408026F0, A7=0xFFF8002E (off the bottom of the SSP) — caused by FPSP recursion exhausting stack.
2. FPSP at `0x4088DB1E` is the F-line vec-11 handler.  Body checks frame format byte 0; if not `0x40` (UNIMP), branches to `0x4088D244` which is **FScc.F (xxx).L** — opcode `0xF27F` `0x0000` `0x00005380`.  Our FPU body does not decode FSF → re-fires F-line → infinite recursion at 212 bytes per iteration.
3. First F-line fires at PC=`0x253c`.  The bytes at `0x253c` are `0xFF 0xFF` (low-RAM data left from MAME-style RAM-test pattern), opcode 1111-prefix → F-line.
4. CPU got to `0x253c` because the function ending at `0x40885418 RTS` popped `0x0000252E` from the stack as a return PC.  `0x40885414..0x40885418` is real code: `moveq #0,d0; addq.w #4,sp; rts`.  The `addq.w #4,sp` skips an SR slot, RTS pops a 4-byte return PC.  In our boot that slot held `0x0000252E` — the BlockMove SOURCE pointer A0, not a return PC.
5. CPU reached `0x40885418` by walking forward through ROM zero-pad `0x40885036..0x408853FF` (970 bytes of `0x00` bytes interpreted as `ori.b #0,d0` instructions, 4-byte stride).  Loop pattern (0x4088521E → 0x40885122) seen in the JTAG ring suggests a VIA1 IRQ fires periodically and RTEs back to 0x40885122 (re-entering zero-pad), churning until it hits the real code at `0x40885400`.
6. CPU first lands in zero-pad immediately after the BlockMove A-trap handler exit.  Between cap[226] (A-line at `0x4081BFDE`, INS=`0xa76ca4`, A7=`0x17FEFE`) and cap[227] (first F-line at `0x253c`, INS=`0xa76e37`, A7=`0x17FEE2`), 403 instructions execute.  EXC_COUNT increases by exactly 1.  No second exception fires before the F-line — so the BlockMove handler exited *directly* into zero-pad without intermediate exception cleanup.

## What the trap chain looks like

### BlockMove call site (`0x4081BFD0`)

```
4081BFD0  48E7 80C0      MOVEM.L  D0/A0-A1, -(SP)        ← push 12 bytes
4081BFD4  C189            EXG      D0, A1
4081BFD6  9088            SUB.L    A0, D0
4081BFD8  C0B8 031A      AND.L    ($31A).W, D0
4081BFDC  D3C8            ADDA.L   A0, A1
4081BFDE  A02E            _BlockMove                     ← A-line, vec 10
4081BFE0  4CDF 0301      MOVEM.L  (SP)+, D0/A0-A1
4081BFE4  4E75            RTS                            ← returns to 0x4081BFAE
```

### A-trap dispatcher (low ROM, `0x408099B0`)

```
408099B0  2F0A            MOVE.L   A2, -(SP)              ; SP -= 4
408099B2  2F02            MOVE.L   D2, -(SP)              ; SP -= 4 (now S_entry-8)
408099B4  246F 000A      MOVEA.L  10(SP), A2             ; A2 = saved PC = 0x4081BFDE
408099B8  341A            MOVE.W   (A2)+, D2              ; D2 = 0xA02E, A2 = 0x4081BFE0
408099BA  0C42 A800      CMPI.W   #$A800, D2
408099BE  6530            BCS.S    0x408099F0             ; A02E < A800 → take

408099F0  2F01            MOVE.L   D1, -(SP)              ; SP -= 4
408099F2  2F09            MOVE.L   A1, -(SP)              ; SP -= 4 (now S_entry-16)
408099F4  3202            MOVE.W   D2, D1
408099F6  2F4A 0014      MOVE.L   A2, 20(SP)             ; ★ overlay 0x4081BFE0 at SP+20
408099FA  0242 0100      ANDI.W   #$0100, D2
408099FE  6620            BNE.S   ...
40809A00  1401            MOVE.B   D1, D2
40809A02  2F08            MOVE.L   A0, -(SP)              ; SP -= 4
40809A04  4EB0 25A1 0400 JSR      ([0x400, A1, D2.W*2])  ; JSR via dispatch table
40809A0A  205F            MOVEA.L  (SP)+, A0
40809A0C  225F            MOVEA.L  (SP)+, A1
40809A0E  221F            MOVE.L   (SP)+, D1
40809A10  241F            MOVE.L   (SP)+, D2
40809A12  245F            MOVEA.L  (SP)+, A2              ; SP back at S_entry
40809A14  4A40            TST.W    D0
40809A16  584F            ADDQ.W   #4, A7                 ; SP = S_entry+4
40809A18  4E75            RTS                            ; reads bytes at S_entry+4..7
```

When SP=S_entry-16, `MOVE.L A2, 20(SP)` writes 4 bytes of A2 at **byte address `S_entry+4`** — which is **2-byte aligned, NOT 4-byte aligned** (S_entry itself was at `0x17FEE6` for the diverging dispatch).  This overlay overwrites the LOW WORD of the saved-PC slot AND the format-word slot with the advanced PC.  The exit `RTS` then reads those same 4 bytes and jumps to the advanced PC.

### Stack snapshot at break_pc=0x40885122 (first hit, EXC_COUNT=0x13B, A7=0x17FEE6)

```
A7+0  0x17FEE6  word 0x2700  ← SR (correct)
A7+2  0x17FEE8  word 0x4081  ← PC_high (correct, top of saved 0x4081BFDE)
A7+4  0x17FEEA  word 0x4081  ← OVERLAY high half — replaced PC_low (was 0xBFDE)
A7+6  0x17FEEC  word 0xBFE0  ← OVERLAY low half — replaced fmt word (was 0x028A)
```

The overlay actually took effect — bytes `[0x40,0x81,0xBF,0xE0] = 0x4081BFE0` are at `0x17FEEA..0x17FEED` as designed.  RTS at `0x40809A18` SHOULD pop those bytes and jump to `0x4081BFE0`.

But the CPU is at `0x40885122` (deep in zero-pad walker), A7 still equals S_entry, and EXC_COUNT shows no new exception.  **The dispatcher's exit RTS did NOT take the CPU back to `0x4081BFE0`.**

### Register state at the diverging halt

```
D0=0x00000000  D1=0x000026DA  D2=0x00000020  D3=0x53455244  ('DERS')
D4=0x00000003  D5=0x00000011  D6=0x00000001  D7=0x00040010
A0=0x000026C0  A1=0x000026CC  A2=0x4081BFE0  A3=0x0000252E
A4=0x0000221C  A5=0x003FAC00  A6=0x0017FF78  A7=0x0017FEE2
EXC_VEC=0x0A   EXC_PC=0x4081BFDE  LIVE_VBR=0x00000000  LIVE_SR=0x2700
```

Register A2 holds the correct advanced PC (`0x4081BFE0`).  D1's low half = `0x26DA` is plausible (BlockMove pointer arithmetic).  At cap[227] in the *previous* bitstream, D1 read `0x2480B6DB` — the `B6DB` bytes are the post-RAM-test scrubber pattern, suggesting an LSU read of uninitialized space at some point in the handler (not consistently across boots).

## Boot non-determinism

JTAG `halt-after-N` on the same N value across separate cold boots produces wildly different CPU states:

```
N=10965401  EXC=0x0176 (deep recursion, A7=0x17CE12)
N=10965405  EXC=0x013B (clean zero-pad walker, A7=0x17FEE6)
N=10965410  EXC=0x0168 (deep recursion, A7=0x17D9AA)
N=10965415  EXC=0x0140 (deep recursion, A7=0x17FB92)
```

Caused by DDR4 calibration timing variance and IRQ delivery jitter.  The bug fires at slightly different INS counts each boot.  Implication: JTAG halt-after-N samples parallel-universe boots, not one trajectory.  Single-boot trajectory tracing requires a DIFFERENT mechanism (sim, or break_pc on the divergent path).

## Directed-test results (2026-05-04)

Three directed tests written to mimic the dispatcher's individual building blocks; each runs on both Verilator (our RTL) and Musashi (golden ref):

| Test | Path | RTL | Musashi |
|------|------|-----|---------|
| Unaligned 32-bit overlay (S1) | `tb/tests/asm/atrap_overlay_unaligned_long_store.s` | PASS | PASS |
| JMP (d8,PC,Dn.W*2) jump table (S2) | `tb/tests/asm/jmp_pc_d8_dn_wscale2_table.s` | PASS | PASS |
| JSR ([0x400+Dn.W*4]) mem-indirect preindexed (S4) | `tb/tests/asm/jsr_preindexed_memind_atrap_table.s` | PASS | PASS |

S1, S2, S4 individually work — both in isolation and in 8-iteration back-to-back stress (no TMP1/TMP2 hazard).  D-register .W truncation, sign-extension, and scale shifts behave correctly across the value sweeps tested.

Implication: the BlockMove handler's individual building blocks all work.  The bug must be in an *interaction* not covered by the isolated tests.  Candidates remaining:

- **IRQ delivery during handler**: VIA1 timer IRQs fire during boot at level 1.  If an IRQ arrives at a specific cycle within the dispatcher (e.g., between the unaligned overlay store and the dispatcher's RTS), exception entry might mis-handle the SP and corrupt the saved-PC slot.  Boot non-determinism is consistent with this — same N-value gives different states.
- **MOVE.L (A0)+,(A1)+ rapid loop**: 8 of these in a row at `0x4080CB00..0x4080CB0E`, with the inner copy of 420 bytes.  If our LSU has a corner with rapid postinc loads + stores (e.g., pipeline draining issues, store-buffer ordering), the BlockMove could land outside its destination region and overwrite the saved-PC slot ON THE STACK.  A0=`0x252e`, A1=`0x2536`, count=`0x1a4` → dest range `0x2536..0x26da`.  Stack at `0x17FEEA` is FAR from this range.  But A0/A1 are LOW MEMORY (system globals) — a wrap or a postinc-overflow could push A1 toward the supervisor stack at `0x17xxxx`.
- **Specific interaction between unaligned store + adjacent reads**: the dispatcher's `MOVE.L A2, 20(SP)` is followed by reads from the same stack region.  If the LSU's store-buffer forwarding misses on the unaligned-split second beat, a subsequent read of the overlay slot could see stale bytes.  But our snapshot says the overlay landed correctly.

## Next directed-test candidates

- `atrap_full_dispatch_replay.s` — replay the EXACT instruction sequence of the ROM dispatcher (0x408099B0..0x40809A18) plus a fake handler doing the bulk MOVE.L + JMP-indexed pattern.  Drive it 100×.  This is the most ROM-faithful test we can write without booting the real ROM.
- `atrap_dispatch_with_irq.s` — same as above but with a periodic VIA1-style IRQ injected by the testbench.  Validate handler chain robustness against arbitrary IRQ arrival.
- `lsu_postinc_long_chain.s` — eight `MOVE.L (A0)+, (A1)+` in a row, followed by a SUB and BGE backward branch (mimics the BlockMove inner loop).  Many iterations to expose any postinc / store-buffer drain bug.

## Suspect mechanisms (pick-and-test order)

### S1.  Unaligned 32-bit STORE on 2-byte boundary

The dispatcher's `MOVE.L A2, 20(SP)` writes 4 bytes at `0x17FEEA` (2-byte aligned, crosses 4-byte boundary).  Our LSU has a split-store path (`rtl/core/mem/lsu.v:280-310`).  If the split path commits one half but not the other, or commits in wrong order, the overlay would be partially or wrongly applied.

**Counter-evidence:** the JTAG stack snapshot shows the bytes `[0x40,0x81,0xBF,0xE0]` are correctly placed at `0x17FEEA..0x17FEED`.  The overlay ran cleanly in the iteration we sampled.  The split-store appears to commit correctly here.

**Test to write:** `tb/tests/asm/atrap_overlay_unaligned_long_store.s` — replicate the dispatcher exit pattern (push exception-style frame at odd offset, do `MOVE.L An, 20(SP)` overlay, `ADDQ.W #4,SP; RTS`) and assert RTS lands at the overlay value.  Add many iterations to expose race / split-MMU-wait corners.

### S2.  JMP (d8,PC,Xn.W*2) brief-format indexed PC

The BlockMove handler at `0x4080CA10..0x4080CB18` uses `JMP (d8,PC,Dn.W*2)` jump-tables: opcodes `4EFB 0284` (at 0x4080CB14) and `4EFB 029C` (at 0x4080CAFC).  Both target base = `0x4080CA9A` + `D0.W * 2`.

`decode_0100.vh:524-705` implements a 5-µop crack for the scale-≠1 case:
```
P0      TMP1 = pd_pc + 2 + sx8(disp)
P1      TMP2 = sx16(D0.W)              [ALU_EXT]
P2..    TMP2 += TMP2 (one doubling for scale=01 = ×2)
P3      TMP1 += TMP2
P4      BR_JMP TMP1
```

If TMP1/TMP2 has a hazard with the prior crack (in the BlockMove handler's loop body, multiple `MOVE.L (A0)+, (A1)+` instructions retire just before the JMP), the JMP target could be a stale TMP1.  Or — sign-extension of D0.W is bit 11 = 0 → ALU_EXT.  If our ALU_EXT µop on the wrong-size mask returned full D0 instead of sign-ext-of-low-word, and D0 had high-bits set from the BlockMove byte count, the offset would explode and the JMP target could land in `0x40885xxx`.

**Test to write:** `tb/tests/asm/jmp_pc_d8_dn_wscale2_jump_table.s` — exact replica of the BlockMove handler shape: bulk-copy loop, JMP indexed PC into a small jump table at known PC offset, with D0 covering a sweep of values (positive, negative, with high bits set, near-zero).  Assert each JMP lands at the expected table entry.

### S3.  Format word collision in the dispatcher path

Live-FPGA plus MAME evidence confirms vec 10 must stay format-0.  The
high-ROM Toolbox dispatcher at `0x408099b0` overwrites the saved-PC slot
and exits with `addq #4,sp; rts`; a format-2 frame leaks four bytes and
loops back into the A-line opcode at `0x40803e08`.  The low-RAM A05D
failure therefore has a different root cause, not vec-10 frame size.

## Two compounding bugs, fix order

- **Fix 1 (root):** find which suspect mechanism (S1, S2, S3, or other) makes the BlockMove handler land in zero-pad.  Without this, the FPU emulation gap stays latent.
- **Fix 2 (downstream):** add FSF (FScc.F, opcode `0xF27F`) decode + handler — the FPSP entry path uses it, and any genuine F-line miss will recurse without it.  Track on task #11.

## Reference data

- Exception trace: `/tmp/exc_trace_long.log` (250 captures, includes cap[225..249] showing the bridge from healthy A-traps to F-line storm).
- JTAG ring at first hit of zero-pad walker: `/tmp/halt_zpw_entry.log` (full register and stack window).
- JTAG ring at first F-line under fresh bitstream: `/tmp/halt_first_fline.log`.
- Halt-after-N sweeps: `/tmp/sweep1.log`..`/tmp/sweep5.log` (locate the divergence to N≈10,965,400 ± 100 in the new bitstream).
- MAME A-line dispatch trace: `/tmp/mame_aline.log` (238 MB, 130 hits at `PC=4081BFDE`, all `frame6=28`).
- ROM SHA1: `7a8ee468d16e64f2ad10cb8d1a45e6f07cc9e212` (Q700 universal `420dbff3.rom`).

## Tools used

- `/tmp/halt_at_n_full_ring.tcl` — halt at retired-µop count N + dump 1024-entry ring + stack window.
- `/tmp/halt_at_pc.tcl` — halt at first hit of break_pc + dump state + 16-long stack window above and 8 longs below.
- `.claude/skills/m68k-fpga-halt-bisect/halt_after_n.tcl` — multi-N sweep helper.
- `.claude/skills/m68k-q700-rom-sim/SKILL.md` — sim-side run procedure (sim diverges earlier than HW; not useful here).
