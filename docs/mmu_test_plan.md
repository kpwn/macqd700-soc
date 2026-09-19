# MMU walker + ATC stress test plan (task #100)

> **Scope.**  Companion to `docs/mmu_walker.md`.  Designs the ≥30-scenario
> stress test suite extending `tb/tb_mmu_walker.cpp` beyond the original
> 17 correctness-tick scenarios.  Objective: exercise every interesting
> corner of the freshly-landed MMU walker + ATC (`rtl/core/mem/mmu.v` +
> `mmu_atc.v` + `mmu_walker.v`) under load, contention, and adversarial
> inputs.
>
> **Status.**  All BFM-level scenarios (categories 1–8, 10) land with
> this ticket and run under `make tb-mmu-walker`.  `agent/mmu-walker-tests`
> adds an 18-scenario boot-critical widening suite that can run alone as
> `make tb-mmu-walker-boot` and is included in `make tb-mmu-walker`.
> Category 9 (RTE-resume) is staged as three pre-authored `.s` tests in
> `tb/tests/asm/mmu_*.s`, deferred via `tb/tests/deferred.txt` with a
> pointer at task #99 (Phase-B wiring) as the blocker.

## 1. Scenario inventory (≥30 BFM scenarios + 3 asm stage-B tests)

Scenario IDs prefixed `S1`–`S10` per category; asm tests `A1`–`A3`.
`CHECK` column lists the primary invariant; `MAME-xref` cites the
Musashi/MAME routine we cross-checked where applicable (see Appendix).

### Category 1 — ATC thrashing (>64 distinct entries)

| ID  | Name (tb fn)                      | Setup                                                                              | Stimulus                                                         | Expected outcome                                                                                                                                         | MAME-xref         |
|-----|-----------------------------------|-------------------------------------------------------------------------------------|------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------|
| S1.1| `test_atc_thrash_128_pages`       | 4K pages, 3-lvl; install 128 distinct VA→PA mappings at 4 KB stride                 | Walk all 128 pages sequentially twice                            | Round 1: 128 walker-driven fills, 0 ATC hits.  Round 2: only VAs whose set is still resident can hit; >=64 walker fills must still occur                 | —                 |
| S1.2| `test_atc_lru_pressure`           | 4K, 3-lvl; fill A..Z (26 entries)                                                   | Walk A..Z, then walk A again                                     | A must still hit (≤64 entries total — no set pressure)                                                                                                   | —                 |
| S1.3| `test_atc_every_set_touched`      | 4K, 3-lvl; pick VAs whose `VA[15:12]` spans all 16 set indexes                      | Walk all 16 unique-set VAs                                       | `atc_valid_count == 16` post-burst; every set has exactly one entry                                                                                      | —                 |
| S1.4| `test_atc_4way_conflict_miss`     | 4K, 3-lvl; 5 VAs all mapping to the same set (varying only the tag bits)            | Walk 5 in order; poison backing memory for all 5; walk all 5 again | After the 5th, oldest of {1,2,3,4} must have evicted (tree-PLRU victim); exactly 4 survive.  Re-walk: one must fault (the evicted tag)                 | `m68kmmu.h` L39 (4-way scheme is not specified by PRM — "LRU-ish") |

### Category 2 — Back-to-back walker throughput

| ID  | Name                              | Setup                                                  | Stimulus                                                     | Expected                                                                                               | MAME-xref |
|-----|-----------------------------------|--------------------------------------------------------|--------------------------------------------------------------|--------------------------------------------------------------------------------------------------------|-----------|
| S2.1| `test_back_to_back_8_walks`       | 4K, 3-lvl; 8 distinct VAs, each a fresh walk          | Fire 8 requests sequentially, measure total cycle count    | Average ≤15 cyc/walk (worst-case per spec ≤14); no deadlock                                          | —         |
| S2.2| `test_walker_single_in_flight`    | 4K, 3-lvl                                             | `req_valid` asserted while walker busy on prev request      | Second request must NOT start until first completes (`resp_ready=0` holds second off)                 | —         |
| S2.3| `test_mixed_2lvl_3lvl_walks`      | Reconfigure TC mid-test between 2-lvl and 3-lvl      | 4 alternating 2-lvl / 3-lvl walks                            | All succeed; no state leak between modes                                                               | —         |

### Category 3 — PFLUSH correctness under load

| ID  | Name                              | Setup                                           | Stimulus                                                                      | Expected                                                               | MAME-xref                       |
|-----|-----------------------------------|--------------------------------------------------|-------------------------------------------------------------------------------|------------------------------------------------------------------------|---------------------------------|
| S3.1| `test_pflush_va_during_walk`      | Start walk for VA=A; 2 cycles later fire PFLUSH-by-VA on DIFFERENT VA=B      | Walk for A finishes, ATC fills with A; B's (absent) entry unaffected           | A resident, B never existed ⇒ no-op pflush                           | —                               |
| S3.2| `test_pflush_va_matches_inflight` | Same as above but PFLUSH VA = the VA walker is resolving                     | Walk completes; ATC invalidates the just-filled entry (order depends on timing) | Post-sequence re-walk for that VA must re-fill (miss), not hit stale | Guidance from 68040UM §6.2.3 |
| S3.3| `test_pflush_asid_mixed_entries`  | Populate ATC with 2 user + 2 supervisor entries, then PFLUSH-by-asid         | Stub treats asid-flush as PFLUSHA                                              | All 4 entries invalidated                                              | Musashi stubs PFLUSH (L209)    |
| S3.4| `test_pflusha_midwalk`            | Kick walker; immediately pulse PFLUSHA while walker busy                     | Walker completes and fills an entry; the PFLUSHA had already cleared valid-bits | Post-sequence `atc_valid_count == 1` (only the fill that landed after PFLUSHA) | —                               |

### Category 4 — Page-boundary straddling

| ID  | Name                                | Setup                                                           | Stimulus                                                          | Expected                                                             | MAME-xref |
|-----|-------------------------------------|------------------------------------------------------------------|-------------------------------------------------------------------|----------------------------------------------------------------------|-----------|
| S4.1| `test_cross_page_long_split`        | 4K; two adjacent pages with different PFNs                      | Translate VA = pageN+0xFFE (halfway through 4B read) then pageN+1 | Both resolve to different PFNs; 2 walker runs on first pass          | —         |
| S4.2| `test_movem_like_burst_across_page` | Translate 10 consecutive VAs spanning a page boundary           | Walker runs exactly twice (one per page)                          | 10 req's → 2 walks + 8 ATC hits; no spurious re-walks                | —         |
| S4.3| `test_ifetch_cross_page_simulated`  | 4K; two adjacent pages; switch `is_instruction=1`               | 2 translations either side of the boundary                        | Both OK; no distinction from D-side in phase-A                       | —         |

### Category 5 — Page-size switching (TC.P 4K ↔ 8K)

| ID  | Name                            | Setup                                                          | Stimulus                                                     | Expected                                                                                                   | MAME-xref |
|-----|---------------------------------|-----------------------------------------------------------------|--------------------------------------------------------------|------------------------------------------------------------------------------------------------------------|-----------|
| S5.1| `test_pagesize_switch_4k_to_8k` | Fill ATC with a 4K entry; switch TC.P to 8K                    | Re-request same VA                                           | After switch, ATC lookup uses 8K tag geometry ⇒ miss (cannot match on different vpn_lsb) ⇒ walker re-runs | —         |
| S5.2| `test_pagesize_consecutive`     | Walk one 4K VA; reconfigure TC.P=1 + 8K page tables; walk 8K VA | Both resolve with correct PFN + offset widths               | 4K PA low 12 bits from VA; 8K PA low 13 bits from VA                                                       | —         |
| S5.3| `test_pagesize_switch_pflushall`| Same as S5.1 but PFLUSHA between modes                         | Re-walks cleanly in 8K mode                                  | `atc_valid_count` drops to 0 post-PFLUSHA; 8K walk populates 1 entry                                       | —         |

### Category 6 — Root-pointer swap (URP/SRP reload)

| ID  | Name                        | Setup                                                          | Stimulus                                                   | Expected                                                                                      | MAME-xref |
|-----|-----------------------------|-----------------------------------------------------------------|------------------------------------------------------------|-----------------------------------------------------------------------------------------------|-----------|
| S6.1| `test_srp_swap_reflects`    | Fill ATC with supervisor entry; rewrite SRP to a different L1 table | PFLUSHA explicitly (HW does not auto-inval on SRP write) | New SRP's mapping walks successfully; old mapping unreachable                                 | PRM §6.5.2 (SRP) |
| S6.2| `test_urp_change_no_inval`  | Populate user entry; change URP without PFLUSHA (intent: broken SW) | Re-request user VA                                         | ATC hit on stale entry (walker did NOT auto-flush) — documents the "PFLUSH after URP" contract | —         |
| S6.3| `test_srp_no_affect_user`   | Populate mixed user+sup entries; rewrite SRP                   | User entry still hits                                      | User ATC entry unaffected by SRP change (only supervisor walks source SRP)                    | —         |

### Category 7 — Supervisor/user FC crossings

| ID  | Name                            | Setup                                                                        | Stimulus                                                 | Expected                                                | MAME-xref |
|-----|---------------------------------|-------------------------------------------------------------------------------|----------------------------------------------------------|---------------------------------------------------------|-----------|
| S7.1| `test_sup_only_user_access`     | 4K page with S=1 in leaf                                                      | Translate with `supervisor=0`                            | Fault code 4                                             | Walker L82 (fault-code 4) |
| S7.2| `test_sup_only_sup_access`      | Same page                                                                    | Translate with `supervisor=1`                            | Success                                                  | —         |
| S7.3| `test_sup_access_user_page`     | 4K page with S=0                                                             | Translate with `supervisor=1`                            | Success                                                  | —         |
| S7.4| `test_toggle_fc_same_va`        | Install ATC entries at same VA for BOTH FCs                                  | Alternate FC, translate same VA                          | Two distinct ATC entries (`sup_tag` differs) — PAs distinct (map them to different PFNs) | —   |

### Category 8 — Exception-during-walk

| ID  | Name                              | Setup                                                          | Stimulus                                                 | Expected                                                                | MAME-xref |
|-----|-----------------------------------|-----------------------------------------------------------------|----------------------------------------------------------|-------------------------------------------------------------------------|-----------|
| S8.1| `test_fault_then_recover`         | L1 PTE type=00 → fault code 1                                  | After fault, fix backing memory, re-request same VA      | Second walk succeeds (walker returned to IDLE)                          | Walker L48/78/117 (`case 0`) |
| S8.2| `test_fault_then_different_va`    | Fault on VA=A; next request VA=B (valid)                        | B walk must complete normally                            | Walker not stuck; no stale last_fault leaked to B's result              | —         |
| S8.3| `test_wp_fault_no_write_leak`     | WP page; issue write                                            | Fault code 3; backing memory unchanged (no spurious AXI writes) | Page descriptor byte untouched; no M-bit set                            | —         |

### Category 10 — Contention / slow-slave

| ID   | Name                          | Setup                                      | Stimulus                                      | Expected                                                     | MAME-xref |
|------|-------------------------------|---------------------------------------------|-----------------------------------------------|--------------------------------------------------------------|-----------|
| S10.1| `test_slow_axi_slave`         | R-channel injects 10-cycle extra delay     | Standard 3-lvl walk                           | Success; total latency scales by the delay                   | —         |
| S10.2| `test_axi_backpressure_ar`    | `ar_ready` held 0 for 5 cycles on every AR | Standard 3-lvl walk                           | Walker tolerates stall; completes correctly                   | —         |
| S10.3| `test_axi_backpressure_b`    | `b_valid` held 0 for many cycles on UM-writeback | U-bit-clear walk needing writeback           | Walker blocks on B, then completes                            | —         |

### Category 9 (deferred asm tests, Phase-B gated on task #99)

| ID | File                                     | Blocker          | Intent                                                                                                       |
|----|------------------------------------------|------------------|--------------------------------------------------------------------------------------------------------------|
| A1 | `tb/tests/asm/mmu_pagefault_rte_basic.s` | task #99 (Phase-B LSU wiring) | User traps on invalid page, handler replaces PTE, RTE, re-execute completes               |
| A2 | `tb/tests/asm/mmu_wp_fix_rte.s`          | task #99 (Phase-B LSU wiring) | Supervisor traps on WP violation, handler clears WP, RTE, continues                       |
| A3 | `tb/tests/asm/mmu_nested_fault.s`        | task #99 (Phase-B LSU wiring) | Handler for page fault itself touches an unmapped page → double fault handling per 68040 |

## 2. Scenario count by category

| Category                 | Count |
|--------------------------|------:|
| 1 ATC thrashing          |     4 |
| 2 Back-to-back walker    |     3 |
| 3 PFLUSH under load      |     4 |
| 4 Page-boundary straddle |     3 |
| 5 Page-size switching    |     3 |
| 6 Root-pointer swap      |     3 |
| 7 FC crossings           |     4 |
| 8 Exception-during-walk  |     3 |
| 10 Contention / slow bus |     3 |
| 11 Boot-critical widening |    18 |
| **BFM-level stress subtotal** | **30** |
| **Current tb-mmu-walker total** | **68** |
| 9 RTE-resume asm (deferred) | 3 |

## 3. Invariants the scenarios guard

1. Walker never violates "one walk at a time" — `w_busy` must stay high for
   the entire sub-flow and drop exactly when `done_ok` or `done_fault`
   pulses.
2. ATC fills happen only on `done_ok`, never on `done_fault` (a faulting
   walk leaves no shadow entry — critical for bsd/Mac OS's "trap then
   fix PTE then RTE" flow).
3. U-bit / M-bit writebacks happen AFTER the walk resolves, never on a
   faulting walk.  Category 8.3 guards this.
4. PFLUSH-by-VA never evicts entries outside its set (the pf_set-localized
   hit detect).  Category 3.1/3.2 guards this.
5. Page-size switch DOES NOT match partially against old entries — the
   `vpn_lsb` change causes different set-indices so a stale entry can't
   accidentally match a new-geometry probe (Category 5).  This is the
   subtle "looked right, was actually broken" case because set-index only
   shifts by 1 bit — an adversary could construct VAs that alias across
   modes.  S5.1 uses such a VA deliberately.
6. Supervisor tag acts as a first-class key — two entries at same VA with
   different `sup_tag` coexist.  Category 7.4.

## 4. Corner cases that forced design changes or nearly did

(The "captured near-misses" from the implementation conversation, per
CLAUDE.md sub-policy.)

- **S3.2 PFLUSH-by-VA same-cycle-as-walker-done.**  The registered ATC
  fill (mmu.v `atc_fill_en <= 1'b1` on `w_done_ok`) means PFLUSH-by-VA
  fired in the same cycle as `done_ok` races with the fill-write.  Both
  write the ATC on the NEXT posedge.  Current RTL resolves this by the
  `pflush_all` taking precedence inside `mmu_atc.v`'s always block; for
  `pflush_va_en` vs `fill_en` the order is `pflush` then `fill`, so the
  fill wins.  That matches the PRM (PFLUSH fires BEFORE the pending walk
  completes — the new entry is considered fresh).  S3.2 asserts the
  resulting post-sequence behaviour.

- **S1.4 4-way conflict miss + tree-PLRU victim.**  Tree-PLRU's
  victim-selection depends on 3 state bits per set.  After a fresh boot
  (all PLRU = 0), the victim is way 3 for first eviction.  Writing the
  test without knowing the exact PLRU state would produce flaky output;
  the test instead touches each way in a known order so the PLRU tree
  ends up in a known shape, then records which specific VA gets evicted.

- **S5.1 4K→8K page-size switch, same VA.**  If the test used a VA whose
  upper bits are identical in both geometries (i.e. only the low ≤12 bits
  differ), the set/tag could alias dangerously across modes.  We pick a
  VA where `VA[15:12] != VA[16:13]` so the new geometry resolves to a
  DIFFERENT set-index — forcing a miss.  Test docstring explains.

- **S8.2 walker return-to-IDLE after fault.**  The initial draft forgot
  to check that `resp_ready` returns high post-fault.  An early RTL
  review caught that `last_fault` is registered and could remain asserted
  longer than 1 cycle if the caller doesn't deassert `req_valid`.  S8.2
  exercises exactly this: fault, deassert/reassert `req_valid` for a
  DIFFERENT VA, confirm cleanly.

## 5. Run + regression

- Focused boot-critical target: `make tb-mmu-walker-boot` (Verilator
  standalone — no dependency on the full core).  Expected:
  **18/18 scenarios pass**.
- Full walker target: `make tb-mmu-walker`.  Expected: 20 base/regression
  + 18 boot-critical + 30 stress = **68/68 scenarios pass**.
- `make test` suite PASS count unchanged (these are BFM-level, not
  integrated).

## 6. Boot-critical widening addendum

The focused `tb-mmu-walker-boot` target covers gaps that are critical to
early Mac OS / ROM page-table bring-up but were either implicit in the
stress suite or not directly asserted:

| ID | Area | Tests |
|----|------|-------|
| B1 | Table traversal | fixed 040 slices despite legacy TC fields, URP/SRP root selection, unaligned root-pointer masking |
| B2 | Pointer attributes | pointer-level WP accumulation, L1/L2 U writeback after success, pending U writebacks suppressed after faults |
| B3 | Descriptor faults | pointer DT=11, leaf DT=10/11, L1/leaf AXI read-response errors |
| B4 | Protection | supervisor-only ATC fills do not authorize user-mode access |
| B5 | Transparent bypass | DTT pass-through and DTT WP faults do not start the walker or fill ATC |
| B6 | Request/AXI handshake | request VA/write latching while busy, AW-before-W and W-before-AW writeback ordering |

## Appendix — MAME / Musashi reference consultation

MAME's `src/devices/cpu/m68000/m68kmmu.h` is built around Musashi's PMMU
code (same file tree — Musashi and MAME share this translation
implementation).  Read at `/tmp/musashi-src/m68kmmu.h` (321 lines).

### What we confirmed

1. **Invalid pointer/page descriptor handling** (Musashi `case 0:` at
   lines 48, 78, 117, 152): the reference implementation simply calls
   `fatalerror()` on invalid descriptors — it does NOT specify a clean
   fault-return protocol because Musashi's PMMU is a skeleton.  The PRM
   (M68000PRM §6.4.3) dictates a bus-error exception (vec 2).  Our walker
   returns `fault_code = 1` for invalid pointer, `= 2` for invalid page,
   and leaves vector-mapping to Phase-B — consistent with the PRM and
   more testable than Musashi's skeleton.

2. **PFLUSH semantics** (Musashi stub at line 209): Musashi intentionally
   leaves PFLUSH as an `fprintf("unhandled")` and relies on the caller to
   flush its own ATC.  The PRM (§6.5) distinguishes PFLUSHA / PFLUSHN /
   PFLUSH (by effective address) / PFLUSHS — all invalidate ATC entries.
   Our RTL implements PFLUSHA + PFLUSH-by-VA; PFLUSH-by-ASID is a stub
   treated as PFLUSHA (acknowledged divergence — see §2 docs/mmu_walker.md).
   Scenarios S3.1–S3.4 cover this.

3. **Used / Modified bit writeback ordering** (walker spec vs PRM §6.4.4):
   the PRM specifies U/M set AFTER successful resolution.  A faulting
   walk leaves U/M untouched.  Our walker does this correctly (see
   mmu_walker.v §5.3 summary in docs/mmu_walker.md).  S8.3 verifies this
   — Musashi's skeleton does not implement U/M so the only cross-check
   is against the PRM directly.

### Where we diverge from MAME/Musashi

| Behaviour                    | MAME/Musashi                                   | Our RTL                                                                                              | Why                                                                                                                              |
|-------------------------------|-------------------------------------------------|------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------|
| PFLUSH-by-VA                 | Not implemented (stub)                          | Single-cycle set-indexed invalidate                                                                  | Skeleton in ref — PRM §6.5 is authoritative.  We track the PRM.                                                                  |
| Indirect descriptor support  | 8-byte descriptor case implemented              | Fault code 5 (unsupported)                                                                           | Phase-A deliberate simplification (see mmu_walker.md §1.3.2).  Phase-B ticket tracks adding it once we see real usage.            |
| ASID tag                     | No ASID (68040 has no HW ASID)                  | 1-bit supervisor-as-ASID scaffolding                                                                 | Forward-compat only.  Our `pflush_asid_req` is currently aliased to PFLUSHA per ticket scope.                                    |
| Early-termination descriptor | Musashi implements "mode=1" early-term (L99)    | Not implemented — all resident leaves are terminal page descriptors                                  | Phase-A simplification.  Early-term is 68851-era feature rarely used on 68040 (M68040UM confirms; 68040 supports it but A/UX etc rarely use it). Phase-B ticket can add if a real workload demands. |

### Scenarios where we explicitly consulted MAME

- **S8.1** (fault recovery): MAME's `fatalerror()` means no test coverage
  there; we cross-reference the PRM instead.
- **S3.3** (PFLUSH-by-ASID): confirmed Musashi does nothing here → our
  "treat as PFLUSHA" is the safe superset.
- **S7.1–S7.4** (supervisor/user tagging): Musashi does not tag ATC by
  FC[2] (no ATC at all); the PRM specifies S-bit stored with each
  descriptor.  Our `sup_tag` field correctly implements the "per-FC
  separate entries" behaviour S7.4 exercises.

No BUG_*.md was filed against the walker during this task — all
scenarios pass as expected.
