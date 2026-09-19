# 68040 Stack-Pointer Model Audit (USP / ISP / MSP)

Sub-agent audit, 2026-05-03. Findings drive task #10.

## Headline bugs

1. **Slot semantics inverted from PRM**: our "SSP" slot plays the role
   of architectural **ISP** (M=0 supervisor stack); our "ISP" slot
   plays the role of architectural **MSP** (M=1 supervisor stack).
   Per `m68k_core_execute.vh:142-145` selector:
   ```
   S=0       → PHYS_USP_TAG
   S=1, M=0  → PHYS_SSP_TAG    (PRM says ISP)
   S=1, M=1  → PHYS_ISP_TAG    (PRM says MSP)
   ```

2. **No real MSP** — `CR_MSP` (MOVEC reg 0x803) and `CR_ISP` (0x804)
   both alias onto the SSP shadow + slot 1 (`commit.v:2218-2228`,
   2262, 2283).  A guest that programs MSP and ISP separately at boot
   sees them collapse to one register.

3. **RTE never honors saved-SR M-bit at restore**: `take_rte_finalize`
   (`commit.v:1689-1800`) and legacy `exc_done_is_rte`
   (`commit.v:2511-2554`) always write the post-pop supervisor A7
   back to **SSP slot** unconditionally, regardless of the target's
   `saved_sr_w[12]`.  Direct violation of the user's directive
   ("sp should be set in the target mode on rte").

4. **IRQ entry never clears M-bit**: per 68040 PRM §8.4.1, IRQ entry
   forces M=0 to switch onto ISP.  We don't.  `arch_sr[12]` only
   moves on `SYS_ANDI/ORI/EORI/MOVE_SR` and the RTE restore.

5. **MOVE-to-SR / ANDI-to-SR ignores M transitions**: `commit.v:2373-2383`
   detects S=1→S=0 and swaps in USP, but never checks for M-bit
   transitions.  A `MOVE #imm,SR` that flips M alone leaves A7
   pointing at the wrong stack.

6. **IRQ-entry completion always writes SSP slot**: `take_finalize`
   (`commit.v:1664-1671`) and `exc_done` (`commit.v:2586-2589`) both
   pin `sp_slot_write_sel <= 2'd1` regardless of caller M.

## Mac OS bootability impact

Today the wedge is upstream of MSP touch — Q700 ROM cold boot runs
S=1 / M=0 throughout, and the sole supervisor stack survives.  But:

| Phase                              | Symptom                                                |
|------------------------------------|--------------------------------------------------------|
| ROM late init / boot blocks        | First MOVEC MSP corrupts ISP → next IRQ return uses    |
|                                    | garbage A7 → sad-mac.                                  |
| System 7 Process Manager init      | Hard crash on first task dispatch.                     |
| System 7 cooperative multitask     | Each context switch corrupts both stacks.              |
| Mac OS 8.x                         | Same + more aggressive MSP usage in nanokernel.        |

**System 7's Process Manager will not survive without proper MSP** —
this is the next architectural cliff after the current bisect frontier.

## Recommended fix

Smallest correct change:
- Add fourth reserved PRF slot `PHYS_MSP_TAG = 7'd22` (PHYS_INT_REGS=96
  has headroom).
- Rename slots so names match PRM: `SSP` → `ISP`, `ISP` → `MSP`.
- Widen `sp_slot_write_sel` semantics to encode 4 cases
  (00=USP, 01=ISP, 10=MSP per the new naming).

### File-by-file change list

`rtl/core/decode/uop_pkg.v`
```
`define PHYS_USP_TAG    7'd19
`define PHYS_ISP_TAG    7'd20    // was PHYS_SSP_TAG; M=0 supervisor
`define PHYS_MSP_TAG    7'd21    // was PHYS_ISP_TAG; M=1 supervisor
```

`rtl/core/m68k_core_execute.vh`
- Add MSP slot to reset/JTAG-arch-load clear loop.
- Read mux: `S=0→USP : (M=0 ? ISP : MSP)`.
- 4-case write decoder.

`rtl/core/commit.v`
- New `reg [31:0] msp;`.
- Helper: `active_sp_sel({arch_sr[13], arch_sr[12]})`.
- **(a) IRQ entry**: clear `arch_sr[12]` on entry; user→sup loads from
  ISP (not SSP).
- **(b) Sync exc entry**: `prf_sp_slot_val` read is correct after
  rename; user→sup paths load from ISP.
- **(c) IRQ-entry completion**: replace pinned `sel<=2'd1` with
  `sel<=active_sp_sel`, update right shadow reg.
- **(d) RTE restore**: 4-arm pick on `{saved_S, saved_M}` for the
  target slot — addresses the user's "target mode on RTE" directive.
- **(e) MOVE-to-SR / ANDI-to-SR**: detect M-transitions, do
  slot-swap symmetrical to existing S-transition path.
- **(f) MOVEC MSP**: stop aliasing onto SSP shadow; use dedicated
  `msp` reg + new MSP slot.
- **(g) Debug ports**: `DBG_CTRL_MSP = 4'd10`; new `OFF_DBG_MSP`.

`rtl/core/exception.v`: no changes (slot selection is commit-side).

### Estimated scope

~80 lines net across 4 files.  Highest-risk site is RTE restore's
mode-aware swap (saved_M may differ from current_M, requiring two
simultaneous slot operations correctly ordered with CCR-restore +
a7_writeback_en pulse).

## Status (2026-05-03 rolling)

- **Bug #2 (MOVEC slot inversion) — FIXED** at commit `cea6cd43`.
  Swapped CR_MSP/CR_ISP slot+shadow targets in commit.v.  Direct
  regression test at `tb/tests/asm/movec_msp_isp_distinct.s` (PASS
  at 294 cycles — pre-fix it returned 0xBBBB0000 from CR_ISP read,
  proving the inversion).
- **Bugs #3-#6** (RTE M-aware restore, IRQ entry M-clear,
  MOVE-to-SR M-transition swap, IRQ-entry-completion slot pin) —
  **still open**.  Each lands in commit.v in a follow-up; same
  surgical pattern as bug #2 but each touches an exception-side
  state-machine arm instead of MOVEC.

## Test plan (remaining)

- MSP-stress directed test: distinct MOVEC values into MSP/ISP, MOVE-
  to-SR M-bit flips, verify A7 follows.  (Slot-distinction half
  shipped via movec_msp_isp_distinct.s; mode-flip half pending.)
- RTE mode-target test: synthetic frame with saved-SR M ≠ current M;
  verify both A7 and inactive slots read out correctly via JTAG.
- IRQ-entry-clears-M test: set M=1, raise VIA1 IRQ, verify handler
  sees M=0 + ISP active + MSP intact.
- Fuzz seed bias: extend `gen_program.py` to occasionally emit MOVEC
  MSP/ISP and M-bit toggles; Musashi cross-check catches slot
  aliasing immediately.
