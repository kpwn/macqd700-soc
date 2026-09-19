# Core / µarch track

> Read `docs/agent_policy.md` first. This brief adds core-track scope.

## Scope

The out-of-order pipeline. Everything from fetch to commit that implements
the 68040 ISA contract.

## Files you own

```
rtl/core/fetch/{if_stage.v, bpu.v, ras.v, icache.v, predecode.v}
rtl/core/decode/{decode.v, uop_pkg.v}
rtl/core/rename/{rat.v, ccr_rat.v, rob.v}
rtl/core/issue/{iq_int.v, iq_fp.v, iq_mem.v}
rtl/core/execute/{alu.v, agu.v, mul_div.v, ccu.v, fpu/*.v}
rtl/core/mem/{lsu.v, dcache.v, mmu.v, mmu_atc.v, mmu_walker.v}
rtl/core/{commit.v, exception.v, m68k_core.v}
tb/{tb_alu, tb_mul_div, tb_rob, tb_rat, tb_bpu, tb_lsu, tb_dcache,
    tb_icache, tb_if_stage, tb_mmu, tb_mmu_walker}.cpp
tb/tests/asm/*.s (directed ISA tests)
tools/fuzz/gen_program.py (extend when you add ISA)
```

Do NOT touch `rtl/mac/*` (peripheral), `rtl/sys/*` (platform),
`tb/tb_top.cpp` (coordination hotspot), `mac_top.v` / `fpga_top.v`
(platform).

## Pipeline at a glance

```
fetch → predecode → decode → rename → dispatch →
  iq_int (2 ports) → ALU × 2 → CDB ──┐
  iq_mem (1 port)  → LSU → CDB ─────┤→ ROB → commit → retirement
```

- Fetch width 4B (16B prefetch buffer), decode width 2 µops/cycle.
- ROB 32 entries, 48 phys int regs, 24 phys FP regs.
- iq_int 8 entries, iq_mem 8 (4LD+4ST nominal, single-port LSU today).
- 2 CDBs (ALU + LSU).
- CCR rename via `ccr_rat.v` (16 slots).

Target: 200 MHz Fmax, IPC ≥ 1.0 after Phase-C of `docs/ipc_roadmap.md`.

## Key design decisions (don't regress these)

1. **Branch dispatch via iq_int.** `UOP_BRANCH` goes through the integer
   issue queue; the ALU resolves direction and target. `iq_int` stores
   `e_is_branch` and `alu.v` disambiguates `BR_BRA` (op 0, collides with
   `ALU_ADD` encoding) via the `is_branch_in` flag.

2. **Bcc condition in `flags_rd[3:0]`.** Decode packs the cc code into
   `flags_rd`; ALU reads it for `BR_BCC`.

3. **CCR rename.** CCR is treated like any renamed int reg. Flag-writing
   µops allocate a CCR phys tag; flag-reading µops carry
   `e_ccr_src_tag` and wake on the same CDB broadcast net. RTE restores
   the saved CCR through the CCR-RAT writeback path. Any new µop that
   writes or reads CCR must plumb the tag — cannot bypass.

4. **Store commit discipline.** Stores hit `cmpl_en` immediately on
   issue (ROB marks done), then park in `S_ST_BUF` until
   `commit_store_en` fires at retire. Only then does the AXI write
   start. Flushes discard un-retired stores cleanly. New atomic ops
   (TAS/CAS) must slot into this model.

5. **RAT rollback on flush.** Speculative `ratmap` is backed by
   `crat[0:ARCH_INT_REGS-1]` shadow. `flush_en` overwrites `ratmap ←
   crat`. `committed_busy[47:0]` bitmap tracks committed phys;
   `free_bm` rebuilt from `~committed_busy` on flush. Same-cycle
   commit-then-flush handled by mirroring the pending commit into the
   rollback result combinationally.

6. **DBcc is single-µop.** `UOP_BRANCH` / `BR_DBCC`, single ROB entry,
   no crack. Don't refactor.

7. **BSR is 2-µop crack** (STORE ret_pc to -(A7), then BRA target).
   RTS is single-µop (LOAD (A7), post-inc A7, broadcast new A7 on CDB).

8. **BTB + bimodal at decode-time**, 64 entries. Training is
   commit-time. Indirect branches (RTS, JMP An) rely on BTB counter
   decay — accepted tradeoff.

9. **PHYS_ZERO_TAG (phys 16)** never freed. Used for zero-idiom elim
   and decode cracks that need a known-zero source.

10. **MMU walker Phase-A landed, Phase-B not wired.** `mmu.v +
    mmu_atc.v + mmu_walker.v` are self-contained. LSU / if_stage / exc
    integration is task #99 (blocked on #85 landed — now unblocked).
    Walker is ITT/DTT passthrough until #99 wires the fault path.

## uop_pkg.v opcode allocation

ALU op slots 0..47 are standard (ADD, SUB, AND, OR, …). Recent
additions:

```
42-47  MUL.L / DIV.L (MULUL, MULSL, DIVUL_Q/R, DIVSL_Q/R)
48-50  BCD: BCD_ADD, BCD_SUB, BCD_NEG (decode-iv-c)
51-54  Scc / TRAPcc / CHK2 / CMP2 (decode-iv-a)
55-58  BFTST / BFCHG / BFCLR / BFSET (decode-iv-b)
59     BF_PACK (decode-iv-d dynamic-both-BF crack)
```

When adding a new ALU op: take the next free slot, add a comment block
explaining semantics, update the tracks/core.md table above, and add at
least one directed test + fuzz widening per the policy.

## Common pitfalls

- **Opcode-slot collision.** Concurrent decode-adding agents once
  collided at slots 48-51. Always grep `uop_pkg.v` for the next-free
  slot BEFORE editing. Union-merge on conflicts.
- **ARCH_INT_REGS bump coupling.** Changing arch reg count requires
  updating `rat.v` (alloc/free masks, committed_busy init),
  `ccr_rat.v` if CCR arch index changes, `tb/tb_rat.cpp` baseline
  expectations.
- **CCR rename write-path omission.** Forgetting to allocate a CCR tag
  on a new flag-writing µop causes readers to wait forever for a tag
  that never fires. Always plumb `flags_wr` and confirm
  `ccr_rat.alloc_en` fires.
- **Dispatch-gate `src_b_rdy |= imm_valid` short-circuit.** This is at
  `m68k_core.v:~905`. If your new µop needs a real `src_b` Dn read
  AND also has `imm_valid=1`, set `imm_valid=0` and carry the constant
  through a non-imm channel, OR crack into 2 µops with TMP1 staging.
  Cost decode-iv-b an hour of triage.
- **Reset PC / RESET_PC mismatch.** tb loads binaries at
  `0x40800000`; m68k_core resets to that address. If you add a new
  harness, match this.
- **dcache read latency.** BRAM-backed dcache (post-#92) has
  synchronous read — 1-cycle BRAM access. lsu.v already handles it.
  Don't add combinational read paths.
- **I-cache read latency.** BRAM-backed icache (post-#73 + #107)
  has 1-2 cycle read with victim-slot soft-hit for backward branches.
  if_stage.v has the prefetch + soft-hit logic.

## Testing rigour for core work

Every ISA addition gets:

1. A directed `.s` test covering the corner you almost got wrong.
2. A fuzz widening in `gen_program.py` (so Musashi cross-checks on
   random sequences).
3. A commit to bench_baseline.md if the change is IPC-visible.

Every µarch change gets:

1. A unit-tb scenario if the affected module has one.
2. A cycle-count delta on the `bench_*.s` suite (honest numbers).
3. A fuzz run (200 seeds) to catch regressions Musashi can detect.

## References

- `docs/microarch.md` — full pipeline spec.
- `docs/isa_status.md` — authoritative ISA inventory.
- `docs/bench_baseline.md` — pinned cycle counts.
- `docs/ipc_roadmap.md` — 4-phase plan to IPC ≥ 1.0.
- `docs/uarch_proposals.md` — older Fmax + IPC proposals.
- `docs/mmu_walker.md` — MMU Phase-A spec + Phase-B hand-off.
