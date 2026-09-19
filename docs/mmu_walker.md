# MMU walker + ATC — Phase-A spec and Phase-B hand-off

> **Scope.**  This document describes the Phase-A landing of task #74
> (`mmu-walker + 64-entry ATC`).  Phase-A is fully self-contained inside
> `rtl/core/mem/{mmu,mmu_atc,mmu_walker}.v` plus its unit testbench
> (`tb/tb_mmu_walker.cpp`).  It does **not** wire into LSU, if_stage,
> or commit — those edits are deferred to a follow-up ticket
> `mmu-walker-wire: LSU + if_stage + exception.v integration` (Phase-B).

## 1. What landed in Phase-A

### 1.1 New RTL

| File | Lines | Purpose |
|------|------:|---------|
| `rtl/core/mem/mmu_atc.v`    | ~280 | 64-entry (16 sets × 4 ways) tree-PLRU ATC with PFLUSH variants |
| `rtl/core/mem/mmu_walker.v` | ~370 | 2- or 3-level 68040 page-table walker FSM with U/M-bit writeback |
| `rtl/core/mem/mmu.v`        | ~400 | Top-level orchestrator (ITT/DTT + ATC + walker + PFLUSH + PTEST) |

### 1.2 Preserved legacy surface

The phase-2 MMU stub's port set is 100% preserved:
`va_in / is_instruction / is_write / supervisor / pa_out / fault /
fault_vec` and the full MOVEC-driven CR bundle (`wr_en / wr_cr /
wr_val / itt0..srp`).  **Every caller of the phase-2 stub continues
to compile without edits**; all 21 legacy tb-mmu scenarios pass unchanged.

### 1.3 Phase-A additions (new ports on `mmu.v`)

#### 1.3.1 Request handshake

```
input  wire        req_valid
output wire        resp_ready
```

When `req_valid` is low, `pa_out` / `fault` continue to reflect
combinationally whatever the phase-2 stub would have produced — safe
default for any caller that hasn't yet opted in.  When `req_valid` is
high, the MMU is authoritative: `resp_ready` goes low while the walker
is running and returns high with a valid `pa_out` / `fault` when the
walk completes or the ATC hits.

#### 1.3.2 Extended fault reporting

```
output wire [31:0] fault_addr_out
output wire [2:0]  fault_code_out
```

`fault_code_out` encoding (kept in sync with `mmu_walker.v` locals):

| Code | Cause                                         |
|-----:|-----------------------------------------------|
|  0   | none                                          |
|  1   | invalid pointer descriptor                    |
|  2   | invalid page descriptor                       |
|  3   | write-protect violation                       |
|  4   | supervisor-only access from user mode         |
|  5   | indirect descriptor (unsupported in Phase-A)  |
|  7   | ITT/DTT write-protect                         |

`fault_vec` is preserved at `8'd8` for legacy-caller compatibility.
Phase-B will re-map this to `8'd2` (bus error) for walker faults and
push a format-2 exception frame.

#### 1.3.3 Walker-side AXI4 master

```
output [31:0] w_ar_addr ; output w_ar_valid ; input  w_ar_ready
input  [31:0] w_r_data  ; input  [1:0] w_r_resp ; input w_r_valid
                                                    output w_r_ready
output [31:0] w_aw_addr ; output w_aw_valid ; input  w_aw_ready
output [31:0] w_w_data  ; output [3:0] w_w_strb ; output w_w_valid
                                                    input  w_w_ready
input  [1:0]  w_b_resp  ; input  w_b_valid ; output w_b_ready
```

In Phase-A the unit testbench drives these directly from a std::map-backed
AXI slave BFM.  Phase-B will stitch them into the existing `axi-xbar`.

#### 1.3.4 PFLUSH (invalidation)

```
input  wire        pflush_all_req    // PFLUSHA
input  wire        pflush_va_req     // PFLUSH-by-VA (PFLUSHN)
input  wire        pflush_asid_req   // stub — treated as PFLUSHA
input  wire [31:0] pflush_addr
input  wire        pflush_sup
```

`pflush_all_req` invalidates every ATC entry in one cycle.
`pflush_va_req` invalidates entries whose tag matches `pflush_addr`
+ `pflush_sup`.  `pflush_asid_req` is scaffolded for a future ASID
extension; today it's a no-op alias for `pflush_all_req`.

#### 1.3.5 PTEST (probe)

```
input  wire        ptest_req
input  wire [31:0] ptest_va
input  wire        ptest_is_write
input  wire        ptest_sup
output wire        ptest_done
output wire [31:0] ptest_pa
output wire        ptest_wp
output wire        ptest_sup_only
output wire        ptest_modified
output wire        ptest_cache_inh
output wire        ptest_ttr
output wire        ptest_hit          // 1 = resident
output wire        ptest_fault
output wire [2:0]  ptest_fault_code
```

Raise `ptest_req` for one cycle.  TTR and MMU-disabled probes return
through a one-cycle fast path; table probes run the walker.  `ptest_done`
pulses high for one cycle when the result lands.  Commit packs the
status bits into MMUSR for MOVEC `%mmusr` readback.

#### 1.3.6 Debug

```
output wire [6:0]  atc_valid_count
```

Combinational popcount of the 64 ATC valid bits, handy for unit tests
and future performance counters.

## 2. Descriptor format

Pointer descriptor (32 bits):

| Bits   | Field               |
|-------:|---------------------|
| `[31:4]` | next-level table phys addr (4 B aligned) |
| `[3]`    | U   — used; walker sets on first touch |
| `[2]`    | WP  — accumulated into page WP on success |
| `[1:0]`  | DT : 10 resident 4-byte table descriptor; 00 invalid; 11 unsupported |

Page descriptor (32 bits):

| Bits            | Field |
|----------------:|-------|
| `[31:pg_bits]`  | PFN  (`pg_bits` = 12 for 4 KB, 13 for 8 KB) |
| `[7]`           | S  (supervisor only) |
| `[6]`           | CI (cache inhibit) |
| `[4]`           | M  (modified; walker sets on write) |
| `[3]`           | U  (used; walker sets on any reference) |
| `[2]`           | WP (per-page write-protect) |
| `[1:0]`         | DT: 01 resident page; 00 invalid; 10/11 unsupported |

## 3. Translation control (TC) encoding used

`mmu.v` reads TC via MOVEC and derives:

| TC bits | Meaning |
|--------:|--------|
| `[15]`  | E — MMU enable |
| `[14]`  | P — 0 = 4 KB pages, 1 = 8 KB pages |
| `[19:16]` | IS — 0 ⇒ 3-level walk (L1 → L2 → leaf); nonzero ⇒ 2-level (L2 → leaf) |
| `[11:8]`  | TIA — L1 index bits (tests use 7) |
| `[7:4]`   | TIB — L2 index bits (tests use 7 for 4 KB; 6 for 8 KB) |

The real 68040 TC layout is richer; Phase-A uses the subset above so
we can exercise 4 KB + 8 KB and 2-/3-level walks independently.
Phase-B can refine the bit mapping without changing the walker FSM.

## 4. ATC design

### 4.1 Geometry

- 64 entries total = 16 sets × 4 ways, 4-way set-associative.
- Set index: `VA[vpn_lsb+3 : vpn_lsb]` (4 bits).
- `vpn_lsb = 12` for 4 KB, `13` for 8 KB — driven by `page_size_8k`.
- Tag: VPN (up to 20 bits for 4 KB) + supervisor bit.
  ASID is scaffolded (1-bit `sup_tag`) but not yet a full ASID;
  hardware ASIDs don't exist on real 68040 so this is
  forward-compatibility.

### 4.2 Replacement

Tree pseudo-LRU, 3 bits per set.  Same scheme as `dcache.v`:
- `plru[2]` : top bit — 0 picks {ways 2,3} as victim side, 1 picks {0,1}.
- `plru[1]` : picks between ways 0/1 within their side.
- `plru[0]` : picks between ways 2/3 within their side.

Fill prefers any invalid way first, else the PLRU victim.  Probe
hits also update PLRU toward the hit way.

### 4.3 Lookup latency

Single-cycle combinational.  `hit_out / hit_pa / hit_wp / hit_sup_only
/ hit_modified / hit_cache_inh` all reflect the current `probe_va`
+ `probe_sup` inputs.

## 5. Walker FSM

### 5.1 States (4-bit, 10 legal states)

```
S_IDLE
  ├─ start? → if three_level: S_L1_AR
  │              else:        S_L2_AR
S_L1_AR → S_L1_R  → if bad PTE: DONE_FAULT
                  → S_L2_AR
S_L2_AR → S_L2_R  → if bad PTE: DONE_FAULT
                  → S_L3_AR
S_L3_AR → S_L3_R  → WP/SUP/invalid? → DONE_FAULT
                  → any U/M bits need writeback? S_UPDATE_AW
                  → else DONE_OK
S_UPDATE_AW → S_UPDATE_B → cycle up_phase (L1 → L2 → leaf → DONE_OK)
S_DONE → S_IDLE (1 cycle to clear busy)
```

### 5.2 Latency

Best case (2-level walk, U/M already set, no writebacks): 4 AXI
single-beat round-trips ≈ 6 clocks.  Worst case (3-level walk + U
writeback at each level + M writeback at leaf): ~14 clocks.

### 5.3 U/M bit writeback

When a pointer or page descriptor is read with U=0, the walker
queues a single-beat AW+W at the descriptor's address with the U
bit set (bit 3 for both pointer and page descriptors).  Writes are applied
AFTER the successful walk, so a faulting walk leaves the in-memory
U/M bits untouched.

Page descriptors additionally get M=1 set on the leaf writeback when
the originating access was a write (`lat_is_write == 1`; M is bit 4).

## 6. Unit-testbench coverage (68 scenarios)

From `tb/tb_mmu_walker.cpp`, `make tb-mmu-walker` now runs 68
deterministic Verilator scenarios.  The original bring-up coverage
includes:

1.  `test_4k_3lvl_miss_then_hit` — cold walk + ATC hit on second request.
2.  `test_8k_3lvl_walk` — 8 KB pages, 3-level walk.
3.  `test_2lvl_4k_walk` — 2-level walk (TC.IS nonzero).
4.  `test_atc_hit_after_fill` — poison L1 after fill, confirm hit bypasses walker.
5.  `test_invalid_pointer` — L1 PTE type=00 → fault code 1.
6.  `test_invalid_page` — leaf PTE type=00 → fault code 2.
7.  `test_wp_violation` — write to WP page → fault code 3 (read OK).
8.  `test_supervisor_only_user_fault` — S=1 page in user mode → fault code 4.
9.  `test_pflush_all_clears_atc` — PFLUSHA invalidates every entry.
10. `test_pflush_va_only_one` — PFLUSH-by-VA leaves other entries intact.
11. `test_pflush_asid_stub` — PFLUSH-by-ASID behaves as PFLUSHA.
12. `test_ptest_probe_hits` — PTEST returns PA + hit.
13. `test_modified_bit_set_on_write` — walker writes back M=1 on first write.
14. `test_used_bit_set` — walker writes back U=1 on first reference.
15. `test_cross_page_access` — adjacent pages with different PFNs.
16. `test_itt_passthrough_still_works` — phase-2 ITT path unaffected.
17. `test_mmu_disabled_passthrough` — TC.E=0 is still pure pass-through.

The boot-critical widening suite added by `agent/mmu-walker-tests` runs
independently with `make tb-mmu-walker-boot` and is also included in
`make tb-mmu-walker`:

1.  `test_boot_fixed_040_slices_ignore_legacy_tc_fields` — odd legacy
    TC fields still walk the fixed 68040 descriptor geometry.
2.  `test_boot_urp_srp_select_distinct_roots` — user and supervisor
    walks source URP/SRP independently and create distinct ATC entries.
3.  `test_boot_unaligned_root_pointer_masking` — low root-pointer bits
    are ignored consistently with descriptor alignment.
4.  `test_pointer_wp_accumulates_to_atc` — pointer-level WP propagates
    into the ATC and faults a later write hit.
5.  `test_l2_pointer_wp_cold_write_fault_no_fill` — cold writes through
    pointer WP fault without filling ATC or writing U/M bits.
6.  `test_clean_read_does_not_set_modified` — reads do not set M.
7.  `test_pointer_used_bits_set_after_success` — L1/L2 U bits are
    written only after a successful walk.
8.  `test_fault_does_not_write_pending_pointer_used` — faulting walks
    do not leak pending pointer U updates.
9.  `test_unsupported_pointer_descriptor_faults` — pointer DT=11 faults
    as invalid pointer.
10. `test_indirect_leaf_descriptor_faults` — leaf DT=10/11 returns
    fault code 5.
11. `test_l1_axi_error_faults_invalid_pointer` — L1 read response errors
    surface as invalid-pointer faults.
12. `test_leaf_axi_error_faults_invalid_page` — leaf read response
    errors surface as invalid-page faults and do no writebacks.
13. `test_sup_fill_does_not_authorize_user` — a supervisor ATC fill for
    an S page does not authorize a user-mode access to the same VA.
14. `test_dtt_bypass_invalid_tables_no_walk` — transparent DTT hits
    bypass the walker and ATC even with empty/invalid tables.
15. `test_dtt_write_protect_fault_no_walk` — DTT WP faults report code 7
    without starting the walker.
16. `test_req_va_write_latched_while_busy` — VA/write inputs are latched
    when a walk starts and mutations while busy do not start a second
    walk until `req_valid` drops.
17. `test_axi_write_address_before_data` — U-bit writeback completes
    when AW handshakes before W.
18. `test_axi_write_data_before_address` — U-bit writeback completes
    when W handshakes before AW.

Run:

```
make tb-mmu-walker-boot
make tb-mmu-walker
```

Expected output: `18/18 scenarios passed.` for the focused target and
`68/68 scenarios passed.` for the full walker target.

## 7. Phase-B wiring plan

### 7.1 LSU (`rtl/core/mem/lsu.v`) — D-side

1.  Replace current (stub) `ea → pa` pass-through with a real MMU
    handshake.  New LSU state: `S_MMU_WAIT` after EA compute.
2.  Drive `mmu.req_valid = 1`, `mmu.va_in = ea`, `mmu.supervisor = SR.S`,
    `mmu.is_write = uop_is_store`.  Wait for `mmu.resp_ready`.
3.  On resp: if `mmu.fault` → latch `(fault_addr_out, fault_code_out)`,
    raise `cmpl_exc` on the completion channel with vector 8'd2 and
    format-2 bus-error frame.  If OK → route `mmu.pa_out` into
    the existing cache / AXI-bypass fanout.
4.  Handshake ordering: the LSU must keep `req_valid` asserted until
    it sees `resp_ready = 1` — this matches the existing cache
    handshake and lets the walker interleave with stalls.

### 7.2 if_stage (`rtl/core/fetch/if_stage.v`) — I-side

Instantiate a **second** `mmu` (or add a second read port — the
walker is shared by registering a small 2-entry arb in mmu.v's
`req_pending` logic).  Drive `req_valid`, `is_instruction=1`,
`is_write=0`, `va_in=pc`.  On fault, raise a fetch-side
bus-error exception through the existing fault-PC plumbing.

> **Note.**  The current core already instantiates 3 MMU copies
> (`u_mmu / u_immu / u_dmmu`).  That pattern holds — each copy will
> carry its own ATC (small enough that duplicated state is cheaper
> than arbitration for Phase-B).

### 7.3 exception.v — fault dispatch

Format-2 frame already supported by the exception sequencer (the
`bus-error-fmt2` agent is landing that).  Phase-B adds:

1.  Map `mmu.fault_code_out` to the correct bus-error cause bits in
    the SSW (Special Status Word) pushed on the stack.
2.  For vec 2, arrange that the faulting VA (`fault_addr_out`) is
    pushed at the standard offset in the frame.

### 7.4 commit.v — MOVEC + PFLUSH + PTEST

1.  Decode `PFLUSH`, `PFLUSHA`, `PFLUSHN`, `PFLUSHS` at the existing
    MOVEC decode site.  Drive `mmu.pflush_all_req / pflush_va_req /
    pflush_asid_req` accordingly.
2.  Decode `PTESTR`, `PTESTW`.  Drive `mmu.ptest_req`, wait for
    `mmu.ptest_done`, and stash the result into MMUSR.

## 8. What Phase-A deliberately does NOT do

- Does NOT dispatch a real bus-error vector on walker fault.  That
  needs commit + exception.v changes (Phase-B).
- Does NOT feed its AXI master through the crossbar.  The top-level
  instantiations in `m68k_core.v` wire the new AXI-master ports to
  ground (via defaults); the walker is never exercised through the
  real memory system yet.
- Does NOT handle ATC-write-hit modify-bit refresh at the LSU
  interface (the `set_mod_en` port exists but is tied 0 at Phase-A).
  Phase-B will raise `set_mod_en` from LSU when a write hits an ATC
  entry whose M bit is 0, triggering an immediate re-walk to set M=1.

## 9. File map

```
rtl/core/mem/mmu.v           ← top, orchestrates ATC + walker + TTRs
rtl/core/mem/mmu_atc.v       ← 64-entry TLB with tree-PLRU
rtl/core/mem/mmu_walker.v    ← 2-/3-level walker FSM, AXI master
tb/tb_mmu_walker.cpp         ← 17-scenario unit tb (std::map mem model)
docs/mmu_walker.md           ← this document
```
