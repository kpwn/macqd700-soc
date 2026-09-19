# L2 System Cache (`l2c`) — Spec v1

Status: integrated between xbar S0 and the shared DDR path when
`L2C_ENABLE=1`, with VRAM/scanout joining downstream through
`axi_vram_priority_mux3`. The module is unit-tested by
`tb/tb_l2c.{v,cpp}` and exercised through the real CDC/MIG chain by
`tb-l2c-chain` and `tb-vram-ddr-chain`.

## 1. Role and coherency model

The L2 is the **single point of coherency** for all traffic that reaches the
DDR-backed windows of the system (RAM, ROM mirror, legacy FB — everything
the AXI crossbar (`rtl/soc/axi_xbar.v`) currently flattens onto its S0 port
via `ddr_flatten()`, see `rtl/soc/axi_defs.vh`). Once integrated, L2 sits:

```
 xbar S0 (128b AXI4, flattened DDR address space)
        |
      [ L2C ]   <-- this module
        |
 axi_async_bridge  ->  MIG  ->  DDR4
   (or SIM_MODEL ddr_ctrl directly, in sim)
```

All masters that can reach S0 today (CPU LSU/boot-FSM merge on M0, host
debug on M1, CPU instruction fetch on M2) converge onto the *same* L2
instance before DRAM, so any two masters racing on the same cache line see
a single serializing point — L2's tag/MSHR/victim-buffer control is that
point of coherency, not DRAM itself.

### Why CPUSH/CINV never reach L2

CPUSH/CINV are 68040 **L1-local** cache-management instructions (per
`CLAUDE.md` — "Mac OS uses CPUSH/CINV during boot" — these operate on the
CPU's own L1 D-cache/I-cache state, which is entirely inside `cpu/`). The
CPU's L1 has no wired-OR "flush to L2" side channel; when the L1 writes
back a dirty line, that writeback is issued as an **ordinary AXI write
burst** on the CPU's data-master port (M0), indistinguishable at the xbar
from any other store. L2 therefore never needs a CPUSH/CINV-shaped command
of its own — dirty L1 data simply *arrives* as normal write traffic, is
looked up, and merges into L2's own write-allocate/write-back policy like
any other store. This is why L2 has zero CSRs/opcodes for cache
maintenance in v1 (see `docs/agent_policy.md`/brief — "no CSRs/stats in
v1").

### Integration update (T16 — "decode-level VRAM lane")

T14 (VRAM-in-DDR migration) briefly used invariant 1's bypass-window
mechanism for a real purpose: the 2 MB VRAM pixel aperture, once moved off
URAM onto a DDR4 carve-out, was routed through `xbar S0 -> l2c`, with a
bypass window configured over the carve-out so l2c never installed VRAM
lines in its tag/data arrays. **T16 replaces that with a stronger, cheaper
mechanism: enforcement moved to the address DECODE itself.**
`axi_xbar.v`'s S3 slave decode (`decode_slv()`) routes the VRAM aperture to
a genuine, separate S3 slave port — unconditionally, regardless of whether
VRAM is backed by URAM or by the DDR4 carve-out — so VRAM-aperture traffic
**never reaches `xbar S0`, and therefore never reaches l2c's slave port, at
all**. There is nothing for l2c to bypass because there is no path for that
traffic to take through l2c in the first place. **L2 is the sole client of
the cacheable decode**: `xbar S0` is the only xbar slave port l2c's slave
port ever sees, and (per invariant 2, unchanged below) every transaction
that reaches it goes through the tag pipeline — S0's traffic is now, by
construction, exactly l2c's own cacheable span (RAM/ROM mirror/legacy FB),
never anything else.

This is a strictly stronger form of the SAME theorem T14's bypass window
proved (no L2 copy of the VRAM window can exist): T14's version relied on
address-range comparison logic *inside* l2c/l2c_bypass.v being configured
correctly at every call site; T16's version is structural — the traffic
simply has no wire path to l2c's slave port, so there is no comparison to
get wrong. See `rtl/soc/axi_vram_priority_mux3.v` (the VRAM-aperture CPU
lane's arbiter, downstream of xbar S3, merged with l2c's own master port
before reaching DRAM) for the mechanism that replaces T14's bypass-window
plumbing at `fpga_top_ddr.vh`'s integration site. `l2c.v`'s own bypass-
window feature (this section, invariant 1) is **untouched RTL** — it
remains fully implemented and unit-tested (`tb-l2c`/`tb-l2c-bypass-all`)
but its one window slot is back to its module-default **disabled** state,
reserved for a *future* window that genuinely can't be separated at the
address-decode level the way VRAM could (e.g. an NVMe descriptor-ring
window, whose traffic might need to interleave with cacheable addresses in
ways a static decode split can't express) — not deleted, just no longer
wired to a live carve-out by any current integration.

### Three invariants

1. **Address-based never-allocate.** A fixed, parameterized list of
   `{base, mask}` bypass windows (default, and — per the integration
   update above — current: one *disabled* window) is checked before any
   tag lookup. A request whose address matches an enabled window is
   forwarded straight through to the master port and is *never* installed
   in L2's tag/data arrays. This list is a compile-time/parameter decision,
   not runtime-programmable in v1 (no CSRs). Reserved for a future
   non-decode-separable window (e.g. NVMe descriptor rings) — see the
   integration update above for why the VRAM-in-DDR carve-out that
   originally motivated sizing this window no longer uses it.

   **Mask convention** (fixed post-Round-4, see `l2c_bypass.v` window-match
   comment for the full writeup): `WIN_MASK` bits set to `1` are the
   window's FIXED (tag) bits — they must equal `WIN_BASE`'s own bits at
   those positions for a hit. Bits set to `0` are the in-window OFFSET,
   don't-cares for the match. A contiguous-high-bits mask like
   `0xFFF0_0000` therefore selects a 1 MB window (`2**popcount(~mask)`),
   matched as `(addr & mask) == (base & mask)`. A prior inverted-sense bug
   (`(addr & ~mask) == (base & ~mask)`, comparing OFFSET bits instead of
   FIXED bits) survived three review rounds because it happened to still
   pass most directed traffic; it broke both invariants above (an
   in-window address whose offset didn't match the base's own took the
   cache path; a cacheable address that happened to share the window's
   offset bits took the bypass path — the latter is real corruption, not
   just misclassification, since a dirty line's eviction writeback can
   land on and clobber a later bypass write to the same DRAM address).
2. **No path around L2 for cacheable windows.** Every AXI4 transaction that
   lands on the slave port and does **not** match a bypass window goes
   through the tag pipeline — there is no secondary "fast path" to the
   master port for cacheable addresses. This is what makes L2 the point of
   coherency: two masters can never race directly on DRAM for the same
   cacheable line.
3. **Full reset semantics.** On reset, the *entire* tag/state array is
   walked and cleared before the slave port accepts any transaction (see
   §6). Dirty lines present at reset time are **not** written back — they
   are simply dropped. This is an explicit design decision (brief: "Dirty
   lines are intentionally lost on reset — RAM after reset reflects only
   what reached DRAM").

### Precondition: bypass/cacheable address disjointness

Bypass windows and the cacheable RAM/ROM/FB space must never overlap. This
is asserted in RTL (`l2c_bypass.v`, wrapped in `// synthesis translate_off`
/ `// synthesis translate_on`) against the `RAM_BASE/RAM_SIZE`-shaped
cacheable-span parameters passed to `l2c.v`. If this precondition holds,
bypass traffic and cached traffic can never target the same line, so there
is no need to order one against the other for coherency — only for shared
physical AXI-master-port arbitration (see §5).

## 2. Interfaces

- **AXI4 slave port** (`s_axi_*`): 128-bit data (`AXI_DATA_WIDTH=128`,
  matches `rtl/soc/fpga_top_xbar.vh` S0 wires), 32-bit address, ID width
  parameter `ID_WIDTH` (default **6**, matching what the xbar's S0 port
  presents today — `axi_xbar`'s `XID_WIDTH = ID_WIDTH(4) + 2`, see
  `s0_awid[5:0]` etc. in `fpga_top_xbar.vh`).
- **AXI4 master port** (`m_axi_*`): same widths, toward the DDR path
  (`axi_async_bridge` / `ddr_ctrl`, whichever the integration task wires
  it to). Single `clk` domain, synchronous active-high `rst`.

Both ports are full AXI4 (AW/W/B/AR/R, `AxLEN`/`AxSIZE`/`AxBURST` present)
so bursts are supported end-to-end; v1 processes slave-side bursts one
INCR beat at a time internally (see §4) — each beat is independently
looked up, which is only a behavioural simplification if a burst crosses a
line (64 B) boundary at a non-16-byte-aligned start address, a shape none
of today's masters (CPU LSU width-adapted 128-bit single-beat ops, MSHR's
own 4×16B fills, boot FSM single-beat writes) currently produce.

## 3. Geometry

| Parameter        | Value | Notes                                    |
|-------------------|-------|-------------------------------------------|
| Capacity           | 2 MB  | data array total                          |
| Line size          | 64 B  | = 4 × 128-bit AXI beats                   |
| Associativity      | 8-way |                                            |
| Sets               | 4096  | 2 MB / 64 B / 8-way                        |
| Set index bits     | 12    | `addr[17:6]`                              |
| Line offset bits   | 6     | `addr[5:0]`                                |
| Tag bits           | 14    | `addr[31:18]` — `32 - 12 - 6`               |

14 tag bits + 12 set bits + 6 offset bits = 32, i.e. the tag is wide enough
to address the *entire* 32-bit AXI address space without truncation — no
extra restriction is needed to "fit" the actual cacheable span. The
cacheable span itself (what the brief calls "the actual cacheable address
span served by S0") is, per `rtl/soc/axi_defs.vh` + `axi_xbar.v`'s
`ddr_flatten()`:

- RAM: `AXI_DDR_RAM_OFFSET` (`0x0000_0000`) .. `+AXI_RAM_SIZE` (1 GiB decode
  window; actual populated RAM is runtime-sized smaller via
  `dbg_ram_window_lg2`, but the xbar's raw decode window is 1 GiB).
- ROM mirror: `AXI_DDR_ROM_OFFSET` (`0x4000_0000`) .. `+AXI_ROM_SIZE` (4 MiB).
- Legacy FB: `AXI_DDR_FB_OFFSET` (`0x4040_0000`) .. `+AXI_FB_SIZE` (8 MiB
  default).

All three ranges arrive on the *same* xbar S0 port (confirmed by
`ddr_flatten()` handling `RAM_BASE`/`RAM_ALIAS_BASE`/`ROM_MIRROR_BASE`/
`FB_BASE` uniformly before presenting `s0_awaddr`/`s0_araddr`), so **ROM
reads do arrive through this same port** — L2 caches them as ordinary
clean lines. ROM is read-only from the CPU's perspective (the xbar drops
writes to the ROM window before they reach S0 — see `axi_xbar.v`'s
local-response policy comment), so an L2 line backed by ROM can be dirtied
only in the pathological case of a non-CPU master (e.g. host debug via M1)
writing into the ROM mirror; L2 does not special-case this — it treats any
write it receives as an ordinary write-allocate/write-back regardless of
which DDR-backed range it targets, exactly matching real hardware.

**Write-allocate has one exception, added 2026-08-19: a write burst that
covers whole 64 B lines with every byte strobe set allocates WITHOUT
fetching them.**  Shape test, from the AW header only: INCR, `awsize` =
16 B, 64 B-aligned base, `(awlen+1)` a multiple of 4, and the first beat
already fully strobed.  Such a burst's beats are gathered four at a time
at `l2c_ctrl`'s front door and the assembled line is installed valid+dirty
in one tag+data write — the fill it would have needed is pure waste,
because every byte of the fetched line would be overwritten.  Anything
else (unaligned, narrow `awsize`, a partial line, any beat with a partial
strobe, a bypass-window address) takes the unchanged fetch-allocate-merge
path.  This is ALLOCATE-without-fetch, not write-through: L2 stays the
single holder of the newest copy, so no reader can observe a state it
could not observe before.  Measurement and rationale:
`docs/l2c_perf.md` section 11.

Data storage: `(* ram_style = "ultra" *)`, one URAM array **per way**
(8 independent arrays, each 4096 × 512 bits), registered (2-cycle) read,
one write port — the multi-way BRAM-inference rule from
`docs/tracks/platform.md` ("4-way-associative caches → 4 independent
per-way BRAMs, not one 2D array") applied at 8-way.

Tag+state storage: BRAM, one array per way (8 × 4096 × {vsec[3:0],
dsec[3:0], tag[13:0]} = 22 bits/entry), registered read, one write port.
PLRU tree state (7 bits/set for 8-way) lives in its own small array, same
access pattern.

**Valid and dirty are SECTORED** — four bits each, one per 128-bit
quadrant of the 64 B line, which is exactly one 68040 L1D line
(`docs/l2c_perf.md` §12):

- `vsec[q]` — quadrant q holds real data. A line may be partially valid:
  a write that supplies every byte of a quadrant installs it with **no
  fill at all**, leaving the other three invalid.
- `dsec[q]` — quadrant q was modified since it was fetched. Only these
  are written back on eviction.
- Invariants, both relied on elsewhere: `dsec[q] ⇒ vsec[q]`, and a valid
  quadrant holds good data in **all 16** of its bytes (a partial-strobe
  write can only reach a quadrant that is already valid; a full-strobe
  write supplies every byte itself). The second is what lets
  `l2c_victim.v` send a full-strobe beat for any dirty quadrant.
- A line whose tag matches but whose requested quadrant is invalid fills
  **into that same way** — never into a second way for the same tag. Way
  selection therefore keys on `tag_match_c` (tag match AND any quadrant
  valid), while the hit response keys on `way_hit_c` (tag match AND the
  requested quadrant valid).

Entry width 16 → 22 bits moves the tag arrays from 2 to 3 RAMB36 per way,
i.e. 17 → 25 RAMB36 for `u_tags`. No URAM change (the data array is
untouched), and BRAM is the one resource this design has spare.

**URAM budget note (Important-8):** each of the 8 per-way data arrays
(4096 × 512 bits = 2 Mb) maps to 8 × URAM288 blocks (URAM288 is
288 Kb, 4096 × 72b native), so L2's data storage alone needs **64 URAM288
blocks**. KU5P ships far fewer than 64 + the ~57 blocks the 2 MB
Q700-silicon-matched VRAM array (`rtl/mac/video.v`) already reserves —
the two budgets do **not** both fit on one KU5P device simultaneously.
This is a known, already-tracked bitstream-cutover blocker: **VRAM must
move off URAM onto the DDR4 path before L2C can be enabled in the same
bitstream as the existing VRAM implementation.** This is a resource-
budget fact about the target device, not an L2C RTL defect — flagged here
so it isn't rediscovered at `make impl` time. The per-way write logic
itself (`l2c_data.v`) must originate from a **single** `always` block per
way with a `for`-loop over bytes for the byte-enable write (not one
process per byte-lane, which is what the v1-draft code did) — URAM288
genuinely supports per-byte write enables, but UG901 requires all writes
to one inferred memory to come from a single process for the tool to
infer URAM at all; the multi-process shape silently fails inference.

## 4. Pipeline

```
       ┌─────────┐    ┌──────────┐    ┌────────────┐
 AXI ->│  accept │ -> │ tag+data │ -> │  resolve   │ -> AXI resp
 AW/AR │ (S_IDLE)│    │  read    │    │ (S_LOOKUP) │    (R/B, arbitrated)
 W     └─────────┘    │(1 cycle  │    └─────┬──────┘
                       │ BRAM     │          │
                       │ latency) │     hit  │  miss
                       └──────────┘   ┌──────┴───────┐
                                      data R/W      MSHR alloc/merge
                                      (fast path)   -> AXI master fill
                                                     -> victim writeback
                                                     -> replay response
```

- **Accept (S_IDLE):** one new op/cycle, arbitrated between a pending AR and
  a pending AW+W-collected write. This *is* round-robin as of the round-2
  fix round (`rw_favor`, flips on every accept) — the v1-draft code gave
  reads fixed absolute priority (`read_sel = ar_have`), which a continuous
  AR stream could starve writes under forever; see §11 item 4. Bypass-window
  addresses are diverted straight to `l2c_bypass` here — they never enter
  the tag pipeline (so they don't cost a lookup slot). A new accept is also
  held off while its AXI ID has a live op still in MSHR or the bypass
  engine (§9, Critical-3 fix) — same-ID response ordering.
- **Tag+data read:** address presented this cycle, all 8 ways' tag+data
  arrays are read in parallel (independent per-way arrays, no port
  conflict), valid the following cycle (matches the registered-read BRAM
  template).
- **Resolve (S_LOOKUP):** 8-way tag compare; victim-buffer hazard check;
  MSHR CAM check (both small combinational compares, 2/8 entries).
  - Victim-buffer address match -> stall (re-read) until that victim entry
    retires (drains to DRAM and frees) — the safe, simple hazard
    resolution: a line mid-drain is *not* re-installed speculatively; a
    later access to it is treated as a fresh miss/allocate once the drain
    completes.
  - MSHR match (line already being fetched) -> secondary miss: enqueue
    into that MSHR entry's replay FIFO (depth 4); MSHR-full-for-this-line
    backpressures (`s_axi_*ready` deasserted) rather than dropping.
  - Tag hit -> fast path: read/write the hit way's data array directly,
    update PLRU (MRU touch), drive the R or B response (arbitrated skid
    register, 1 deep). A write hit only drives a B response on the
    burst's *last* beat (`req_need`, mirroring how AXI expects exactly one
    BRESP per write transaction, not one per beat) — earlier beats of a
    multi-beat cached write retire silently (§9, Critical-1 fix).
  - Miss (requested quadrant not valid), no MSHR match, and the request
    is a WRITE that supplies every byte of the quadrants it touches ->
    **no fill is needed at all**. Pick the way (the tag-matching one if
    the line is already resident, else a PLRU victim, evicted first if it
    has dirty quadrants), and write data + `vsec` + `dsec` on one clock
    edge. This is the generalisation of §11's full-line case; the AW-shape
    gather survives only as a throughput shortcut (one tag-pipeline
    request per line instead of four).
  - Miss, no MSHR match, and a fill IS needed -> allocate a free MSHR
    entry. If the line's tag already matches a way, fill into THAT way and
    evict nothing; otherwise prefer an invalid way in the set, else the
    PLRU victim, and if that victim has any dirty quadrant push its line +
    dirty mask + reconstructed address (`{tag,set,6'b0}`) to the 2-entry
    victim buffer (stall if both slots busy) and invalidate the tag
    immediately (so nothing else can hit it mid-fill). Hand the whole op
    off to `l2c_mshr`, which owns the rest of this miss's lifecycle
    independently of the front door.
  - A WRITE onto a quadrant the cache already owns, on a line that has a
    live MSHR entry, **retries** rather than merging: `l2c_mshr` merges
    into fetched data, which is stale for a quadrant the cache owns. The
    fill needs no front-door progress, so the retry always terminates.

v1 accepts one op through the lookup pipeline at a time (no overlapped
issue) — hit latency is ~4-6 cycles as specified; throughput is
correctness-first, not maximised, for this task. Deeper front-door
pipelining is a documented follow-up (see §8).

## 5. MSHR semantics

8 entries. Per entry: `{valid, set[11:0], tag[13:0], way[2:0], state,
replay_fifo[4]}`. States: `ALLOC -> (EVICT_COPY) -> ISSUED_RD -> FILLING
-> INSTALL -> REPLAY -> free`.

- **Allocation:** on a primary miss, if no free entry, back-pressure
  (`s_axi_*ready` low) — misses are never dropped.
- **Same-line secondary merge:** a second request whose `{set,tag}`
  matches an already-allocated (non-free) MSHR entry is *not* a new
  allocation — it is pushed onto that entry's `replay_fifo` (id, is_write,
  wdata/wstrb, response routing) and serviced once the line installs. If
  the replay FIFO for that entry is full, back-pressure (never drop).
- **Fill issue:** all 8 entries may have fill ARs outstanding. Each entry
  owns its issued/done/error state, beat counter, and 512-bit assembly
  buffer; its table index is carried in the low AXI RID bits. The issue
  scheduler can accept one new AR per cycle, and returning beats are
  demultiplexed by RID independently of the single-port install/replay
  scheduler. Fills are 4-beat 128-bit INCR bursts (`ARLEN=3, ARSIZE=4
  (16B), ARBURST=INCR`), matching the line size exactly. The production
  DDR path currently returns bursts in accepted-AR order, but the MSHR's
  RID demultiplexing is also tested with complete bursts returned in
  reverse order and with 32 rounds of shuffled eight-fill completion.
- **Completion service is round-robin, not fixed-priority:** the "which
  completed entry installs/replays next" scan rotates its starting point past
  whichever entry it just picked (a rotate-then-lowest-index encode using
  `l2c_pri8`), rather than always scanning from entry 0. A fixed
  lowest-index scan was the v1-draft design and **failed under bring-up**
  load testing: under sustained concurrent refill (the randomized
  scoreboard's continuous 8-deep issue window), a freed low-index entry
  gets immediately reallocated by the (also lowest-index) free-slot
  picker, so a fixed-priority scan re-selects that same refreshed low
  index forever and starves every higher-index entry permanently — a
  directed test that issues one fixed batch and drains it to completion
  never exercises this (nothing new competes for the freed slot), only
  sustained load does. Confirmed via direct trace during bring-up
  (entries 2-7 activated exactly once each while 0/1 cycled hundreds of
  times); round-robin fixes it.
- **Fill may proceed before victim drains:** the victim's dirty data is
  copied out of the data array into the victim buffer *before* the fill
  is issued (a same-cycle local copy, not gated on DRAM), so the new
  line's read burst does not wait for the writeback to complete on DRAM.
- **Install:** on the fill's `RLAST`, the line is written into the
  allocated `(set, way)` data array (full-line strobe — nothing valid
  there before, so no clobber risk, see Critical-6 below) and the tag
  entry is marked valid. If any fill beat returned a non-OKAY `RRESP`,
  the install is skipped entirely and every primary/replay requester for
  that entry instead gets a SLVERR response (`rsp_resp`) — no garbage
  line is ever installed on a DRAM-side error (Minor-16 fix).
- **Replay:** the primary op, then each queued secondary op, are serviced
  in FIFO order against the now-resident line, each driving a normal R or
  B response through the arbitrated response path. Each replay write uses
  a **quadrant-only** byte strobe (its own 16 B, not the whole 64 B line)
  — an earlier version wrote the whole `line_reg` snapshot on every replay
  step, which would silently clobber an unrelated hit-write that landed
  on this same now-resident line between install and that replay step
  (its data never touched `line_reg`, so the full-line write would revert
  it to stale data) — see Critical-6 in §9. The MSHR entry frees once the
  replay FIFO drains.
- **Reset drain:** on `rst`, the table enters `S_DRAIN` and accepts/discards
  R beats instead of returning directly to SCAN with `m_rready` low. The
  MSHR reset input remains asserted for the full 4096-set tag-clear walk,
  followed by the local 128-cycle drain window. This composition matters:
  the 200-263-cycle DDR model exposed that a fixed 128-cycle sink could
  expire while the front door was still closed; a delayed pre-reset fill
  then remained at `RVALID` until a post-reset MSHR reused its RID and
  consumed stale data as a new fill. Holding drain through the tag walk
  prevents RID reuse while pre-reset read debt can still arrive. The SoC's
  async bridge stale sink remains the definitive downstream debt tracker.

## 6. Victim / writeback buffer

`VICTIM_SLOTS` entries, default 8: metadata `{valid, addr[31:0],
dsec[3:0]}` in flops, the 512-bit payload in a flat `(* ram_style =
"distributed" *)` array (`v_pay`). A dirty eviction pushes one entry
(stalls if all are occupied) and is drained by an AXI-master write engine
that keeps **up to `VICTIM_SLOTS` writebacks outstanding at once**: the
sequencer presents an AW, streams that transaction's W beats, and moves
straight on to the next entry — `BRESP` is tracked by count, not waited
on. Three pointers order it: `wptr` (push), `dptr` (AW/W, advances at
WLAST), `bptr` (retire, advances at B), with `bptr <= dptr <= wptr`. All
writebacks share one AWID, so AXI4's same-ID rule is what makes in-order
retirement at `bptr` correct. A transaction's AW is never presented
before the previous one's W burst has finished, so at most one write can
ever be "AW accepted, W incomplete" — which is what keeps the
reset-recovery story (§9, IMPORTANT-A) to one filler burst plus a count
of BRESPs still owed. The burst covers the
**contiguous span of dirty quadrants** — `awaddr = line base + first×16`,
`awlen = last − first`, so 1 to 4 beats of 128 bits rather than always 4.
A quadrant inside the span that is not dirty goes out with `wstrb=0`: it
may never have been fetched, and writing the array's contents for it
would be silent corruption. The engine is arbitrated against the bypass
engine for the one
physical AXI-master AW/W/B sub-port (and, symmetrically, MSHR-fill is
arbitrated against bypass for the AR/R sub-port; the response arbiter
back at the slave port arbitrates mshr/hit/bypass onto R/B). **Bypass
wins every one of these three arbitrations** — this is the opposite of
the v1 draft (which reasoned "bypass is rare, give core traffic
priority") and was corrected during bring-up: because a
bypass-classified request occupies the *single* front-door lookup slot
(§4) until `l2c_bypass` accepts it, and `l2c_bypass` cannot return to
IDLE (and therefore cannot accept the next request) until its *own*
pending response drains, fixed core-traffic-wins priority let sustained
mshr/hit/victim traffic starve a bypass response indefinitely — which
wedges the *entire* front door (a bypass-classified head-of-line request
can neither dispatch as bypass, since `byp_req_ready` never rises, nor
fall back to the cacheable path). Confirmed via the 20k-op randomized
scoreboard: it failed reproducibly under sustained mixed traffic with
core-priority arbitration and passed cleanly both with bypass traffic
disabled and after switching to bypass-wins priority everywhere. Bypass
is rare (single-beat, ~5% of traffic in the testbench mix) so giving it
priority costs negligible core throughput. An entry frees once its
`BRESP` is accepted — **never** at AW-accept or WLAST: a presented or
accepted write is not durable, and the entry has to stay visible to the
victim-buffer hazard check below until it is.

The AW/W/B arbiter locks the write port to **one owner** while that owner
has any write outstanding, because AXI4 has no WID and two sources'
W bursts may therefore never interleave. It no longer holds the grant to
`BRESP`, which is what used to serialize the port to one transaction at a
time; §13 of `docs/l2c_perf.md` has the measurement. Bypass keeps
priority, and a saturated victim engine cannot starve it: new victim AWs
are withheld the moment `by_awvalid` rises (never mid-presentation), so
the outstanding count drains and bypass acquires the port.

All three of these arbiters (both AXI-master sub-port muxes in `l2c.v`,
plus the response mux described just above) **lock their winner the
moment anything is first presented** (`VALID` asserted) and hold it until
that transfer's handshake actually fires, rather than re-evaluating
priority live every cycle a grant hadn't yet been marked busy. The
v1-draft code recomputed the live priority every cycle prior to the
*handshake* completing (not prior to presentation), so a higher-priority
source asserting its own `VALID` while a lower-priority transfer was
already `VALID`-but-not-yet-`READY` could swap the presented payload out
from under it — an AXI4 stability violation (Critical-4/Critical-5 in
§9).

**Victim-buffer hazard:** while an entry is resident (valid, whether
draining has started or not), its address is visible to the front door's
per-lookup hazard check (§4) — any new access to that line stalls until
the entry retires rather than being served speculatively from the
(already-evicted) tag/data arrays or racing the writeback. The compare is
**line-aligned** (`addr[31:6]`): the hazard query arrives as a full
per-beat address, which may carry a nonzero offset within the line, while
the buffered entry's address is always line-aligned (`{tag,set,6'b0}`) —
an earlier version compared the two addresses directly, so a query whose
low 6 bits happened to be nonzero missed every hazard, letting an access
at a nonzero offset of a just-evicted dirty line fall through to a
fresh-miss refetch from DRAM while the actual dirty data was still sitting
unwritten in the buffer (stale-data escape, Critical-2 in §9).

Both slot allocation (push) and drain selection alternate their
preference (`push_favor`/`drain_favor`) rather than always favoring slot
0 — the v1-draft code preferred slot 0 for both, which could starve slot
1 under sustained eviction pressure (Important-12 in §9).

## 7. Reset FSM (`l2c_reset.v`)

BRAM/URAM have no bulk-clear primitive, so on `rst` a walking FSM sweeps
`set = 0 .. 4095`, driving a synchronous "invalidate all 8 ways of this
set" write each cycle into `l2c_tags`' 8 per-way arrays (independent write
ports, no conflict) and clearing that set's PLRU state. The slave port's
`s_axi_awready`/`s_axi_arready` are held low for the whole walk (~4096
cycles). This is far below the xbar S0 watchdog bound (2^18 cycles, see
`docs/tracks/platform.md`), so a full reset never trips the watchdog on
its own. Dirty lines present at reset are dropped (§1, invariant 3) — the
victim buffer and MSHR table are also synchronously cleared, so no
in-flight writeback survives a reset.

## 8. Bringup safety: `L2_BYPASS_ALL`

Parameter, default 0. When set to 1, `l2c.v` wires the slave port straight
through to the master port with **plain combinational `assign` pass-
through** (AW/W/B and AR/R each pass through unchanged in the same cycle,
zero added latency) — not a registered passthrough as an earlier draft of
this doc claimed; `clk`/`rst` aren't even used by the cache machinery in
this branch (it isn't instantiated at all). This gives the integration
task a known-good escape hatch: land the module in the address path with
`L2_BYPASS_ALL=1` first, confirm zero behavioural change, then flip it off
once confidence is established.

### Per-window bypass: pipelined, one id at a time (`l2c_bypass.v`)

> **Superseded 2026-08-20.** This section used to read "single-outstanding
> precondition" and declared that floor a hard v1 property. It is gone.
> `l2c_bypass` is now a `BYPASS_SLOTS`-deep in-order queue (default 8) and
> the follow-up this section asked for — "any such integration needs its
> own multi-outstanding bypass engine" — is what landed. Measurements and
> the depth argument are in `docs/l2c_perf.md` §14.

Separately from `L2_BYPASS_ALL`, each address-matched bypass *window*
(§1 invariant 1) is serviced by one queue that accepts up to
`BYPASS_SLOTS` beats and keeps them in flight together. The ordering
guarantees are **exactly** what the old one-at-a-time engine gave, and
they come from two rules rather than from serialization:

- **One id at a time.** `req_ready` refuses a request whose AXI id differs
  from the id already queued, so every outstanding transaction carries the
  same `ARID`/`AWID` and AXI4 obliges the slave to answer them in
  acceptance order. That is what makes the engine's purely positional
  bookkeeping legal — the same argument `l2c_victim` makes for its single
  `wb_id`. It costs nothing on real traffic: `l2c_ctrl` decomposes one
  burst into per-beat requests that all carry that burst's own id.
- **No read/write overlap.** AXI orders nothing between the R and B
  channels, so a read must never be in flight alongside a write; a
  direction change drains first (`dir_ok_c`). Back-to-back accesses to the
  same bypass address therefore still resolve in front-door acceptance
  order, read or write. Real bypass traffic (block storage) is
  unidirectional per burst, so this costs one drain per direction change.

Consequences for consumers:

- **VRAM-in-DDR carve-out** (the window this mechanism was originally
  sized for) was already fine and is unaffected.
- **Streaming block storage** — the DDR RAM-disk at
  `AXI_RAMDISK_L2_BYP_BASE` — is the case that motivated the rewrite. A
  512 B transfer used to cost 32 serialized DRAM round trips; it now costs
  `ceil(32 / BYPASS_SLOTS)` of them.
- **Depth is bounded by the DDR path, not by this engine.**
  `axi_ddr4_mig_bridge` accepts `RMAX_OUTSTANDING = 8` reads and
  `WMAX_OUTSTANDING = 4` writes (`docs/ddr4_mig_bridge_contract.md`), so
  past 8 the extra slots only move the queue. Raise `BYPASS_SLOTS` and the
  bridge's caps together or not at all.
- **Scanout / video-refresh bandwidth** through a bypass window is now
  bounded by `BYPASS_SLOTS x line_size / round_trip_latency` rather than
  by one round trip per beat, but it is still not a substitute for a
  dedicated port — the front door accepts at most one beat per cycle and
  that beat competes with CPU traffic for `l2c_ctrl`'s single `ar_have`
  tracker.
- Also document this precondition wherever a **new** bypass window is
  configured (`l2c_bypass.v`'s `m_arsize`/`m_awsize` are additionally
  hardcoded to the full 16 B AXI-bus width regardless of the original
  request's size — correct only for a plain memory carve-out with no
  read/write side effects, see §9 Minor-17).

## 9. Known v1 simplifications / follow-ups, and round-2 correctness fixes

### AXI4 response ordering (updated)

**Same-ID** ordering *is* enforced as of the round-2 fix round: a new
front-door accept is held off whenever its AXI ID matches an op still
live in MSHR or in the bypass engine (`id_busy_c`/`idq_busy`, Critical-3
below) — this also transparently serializes same-ID beats of a
line-crossing burst (a later beat can't even enter the front-door pipe
until an earlier same-ID miss has fully drained), so per-ID R/B response
order matches per-ID issue order, matching AXI4 §A5.3. This is
conservative rather than optimal: MSHR itself already keeps ops that
merge into the *same* entry in issue order, so blocking a fresh accept
on any live same-ID op (rather than only on ops that would race a
*different* entry) costs a little concurrency but never correctness.
**Cross-ID** ordering remains unconstrained by design — different-ID
transactions may complete out of order (this is normal/desirable AXI4
behavior), and v1 still does not implement (nor does AXI4 require) a
global in-order completion queue across IDs.

### Other v1 simplifications

- Front door: one lookup in flight at a time (no overlapped issue).
- MSHR: 8-deep miss tracking and up to 8 outstanding fill bursts. Install
  and replay remain one-entry-at-a-time because the tag/data arrays expose
  one physical write port. MEASURED (`tb-l2c-chain`,
  `concurrent_fill_overlap`): 8 concurrent misses complete in 66 cycles
  versus 204 sequential, a 3.09x speedup through the real CDC/MIG chain.
- Shared VRAM/scanout DDR read mux: up to 8 accepted reads are routed by a
  source FIFO, but at most 2 non-scan bursts may be outstanding. Because
  downstream responses are accepted-order, this is the hard fabric
  queueing bound ahead of a newly arriving scan request. Scanout does not
  allocate L2 lines; VRAM-aperture CPU traffic remains structurally outside
  L2 as described in the T16 integration update. The bound assumes the MIG
  itself makes forward progress; AXI does not specify an absolute response
  deadline for a slave that stalls forever.
- Only full-width (16 B) INCR bursts are supported by this datapath's
  quadrant-indexed addressing; a request with any other `AxBURST` or
  `AxSIZE` is rejected with SLVERR (one response per burst, array/MSHR/
  bypass entirely untouched) rather than mis-decoded (Important-10).
- No CSRs/perf counters (matches brief).
- Bypass-window list is parameter-fixed (not runtime CSR-programmable).
- Bypass windows accept one AXI id at a time and drain on a read/write
  direction change (§8). Both are ordering rules, not throughput floors —
  the one-at-a-time *transaction* limit was removed 2026-08-20.

These are candidates for `docs/uarch_proposals.md`-style follow-up once
the integration task lands and real traffic patterns are measured.

### Round-2 correctness fixes (architectural review)

A round-2 correctness-focused architectural review found 7 Critical + 7
Important + 3 Minor issues, all fixed in this revision; the round-1 tb
passed 12/12 despite these because its driver was structurally blind to
every class below (unique IDs only, `rready`/`bready` hardwired to 1,
full strobes only, single-beat randoms only, unmatched B/R silently
dropped, no payload-stability checking) — the tb was widened alongside
the RTL fixes (§10).

**Critical:**

1. Multi-beat cached write emitted one BRESP per beat instead of one per
   burst — fixed by gating the hit-write response on `req_need` (last
   beat only), mirroring how the MSHR replay path already worked. See §4.
2. Victim-buffer hazard check compared a full per-beat address against a
   line-aligned buffered address, missing every query at a nonzero line
   offset and allowing a stale-data escape. Fixed: line-aligned compare
   (`addr[31:6]`) on both sides. See §6.
3. Same-ID response reordering: a same-ID hit could respond before an
   older same-ID miss still resolving in MSHR, and line-crossing burst
   beats could reorder across a miss/hit boundary. Fixed via `id_busy_c`
   front-door accept gating. See "AXI4 response ordering" above.
4. `m_axi` master-port arbiters (AR/R and AW/W/B) could swap which source
   was being presented mid-transfer, before the target's `READY` fired —
   an AXI stability violation, and for AW/W specifically could route a
   write-data burst to the wrong AW header entirely. Fixed with a
   locked-grant pattern (`*_grant_live_c`), see §6.
5. Same disease, response side: the `s_axi` R/B response mux could swap
   payload mid-presentation when a higher-priority response arrived while
   a lower-priority one was valid-but-not-ready. Fixed the same way
   (`r_lock`/`b_lock`), see §6.
6. MSHR replay writes (`S_SWR`) used a full-line byte strobe, so a replay
   step could silently overwrite an unrelated hit-write that landed on
   the same now-resident line between install and that replay step.
   Fixed: quadrant-only strobe for replay writes. See §5.
7. Tag array (1-cycle latency) and data array (2-cycle latency) read at
   different speeds; an MSHR install landing during a lookup's own
   accept or wait cycle could write the arrays too late to be reflected
   in that lookup's resolve, producing a hit against pre-install (stale)
   data. Fixed: a 2-deep install history bounces the lookup back for one
   retry cycle on a same-set collision within that window (derived
   edge-by-edge from the arrays' exact NBA timing). See `l2c_ctrl.v`.

   **Retry bound (MINOR-B, independently re-verified):** the retry is
   NOT open-ended. Each retry costs exactly one extra S_WAIT cycle, and
   because MSHR is a SINGLE-active-install source (§5 -- one install/replay
   scheduler, one entry installing at a time), consecutive installs to the SAME set
   are separated by at least the S_SCAN→S_AR→S_FILL(4 beats)→S_INSTALL
   pipeline latency, an **≥8-cycle inter-entry gap** even in the
   tightest realistic case. Since the hazard window is only 2 cycles
   wide, a colliding request can be forced to retry at most once per
   *install*, and installs to its own set can't arrive back-to-back
   faster than that gap allows -- worst case is bounded at **~12 cycles**
   of front-door stall for any single colliding request (a handful of
   retries against a burst of same-set installs, never unbounded
   livelock). Front-door pipelining work MUST preserve this: either keep
   MSHR single-active-install, or re-derive this bound against whatever
   replaces it.

**Important:** 8 (URAM inference coding pattern + budget note, see §3),
9 (post-reset drain of an in-flight fill, see §5), 10 (illegal-burst
SLVERR reject path, see above), 11 (read/write front-door fairness, see
§4), 12 (victim slot-1 starvation, see §6), 13 (tb widening, see §10),
14 (bypass ordering rules, see §8 — the single-outstanding half of this
item was retired 2026-08-20).

**Important-A (found by re-review, landed in a follow-up commit on top
of the round-2 fixes above):** the post-reset drain described under
Important-9 only ever covered the READ side (MSHR's in-flight fill).
Reset with a victim WRITEBACK mid-flight (`l2c_victim.v`) was not
drained at all -- an AW already ACCEPTED by DRAM with its W burst only
partially sent left DRAM waiting for the remaining beats forever, and a
BRESP still outstanding when reset hit would arrive later and get
silently mis-consumed as some UNRELATED, LATER writeback's own
completion (a perpetual off-by-one B association -- confirmed by direct
reviewer probe). Fixed with the RTL quiesce approach (not a spec-only
precondition): `l2c_victim.v` gained `S_RSTDRAINW` (completes an
abandoned W burst with wstrb=0 filler beats -- DRAM's own beat-count
must still reach exactly the promised length) and `S_RSTSINKB` (sinks
the one BRESP DRAM still owes before resuming normal accounting);
`stray_b_owed` is the only state in that module that deliberately does
NOT clear on `rst` (bus-protocol hygiene, not cache state -- same
category as the tag-clear walker that also runs post-reset). `l2c.v`'s
AW/W/B arbiter correspondingly keeps a VICTIM-locked grant held across
reset (bypass, which has no such drain fix, still releases normally --
a separately-flagged gap, see below). A related bug was found and fixed
during THIS fix's own bring-up: every master-port output driven by the
new states must ALSO be gated by `!rst` uniformly (not just the new
states specifically) -- on the exact cycle `rst` first asserts, `st`
itself hasn't transitioned yet (that's an NBA), so an ungated `(st==S_W)`
term stayed live for one extra cycle with real (non-filler) data,
desyncing DRAM's beat count by exactly one and hanging the drain
permanently (DRAM had already closed the burst one beat early; the
drain FSM never got the WREADY it was still waiting for). Composes
correctly with the R-side S_DRAIN (independent arbiters, no shared
state) -- verified by a directed reset-during-simultaneous-fill-and-
writeback scenario. **Known gap, not fixed in this round:** `l2c_bypass.v`
has the identical mid-W-burst/stray-B vulnerability (its own writes are
always single-beat, so only the "stray B" half applies, not the
multi-beat W-completion half) and no drain fix -- low risk today since
bypass is disabled by default (`WIN_EN=0`), but MUST be addressed before
any bypass window carrying real write traffic goes live.

**Minor:** 15 (this doc's drift from as-built behavior — all fixed
throughout this revision — plus the disjointness assert upgraded from
`$display` to `$fatal`), 16 (fill/writeback SLVERR propagation instead of
silent-OKAY-plus-stale-install, see §5/§6), 17 (bypass `ARSIZE`/`AWSIZE`
memory-carveout-only precondition documented, see §8).

**CRITICAL (found by final-gate review, survived rounds 1-3, fixed in
round 4):** `l2c_bypass.v`'s window-match logic used the INVERTED mask
sense — `(match_addr & ~mask) == (base & ~mask)`, comparing the in-window
OFFSET bits — while the module's own disjointness `$fatal` assert (window
size = `~WIN_MASK+1`) and every `WIN_MASK` value ever configured (e.g.
`0xFFF0_0000` = 1 MB window, top 12 bits fixed) treat the mask as FIXED
HIGH BITS. As built, an in-window address whose *offset* bits happened to
differ from the base's own took the CACHE path (breaking invariant 1 —
S1: a future VRAM-in-DDR carve-out would thrash L2 instead of bypassing
it), while a cacheable address that happened to share the window's
*offset* bits took the BYPASS path instead (breaking invariant 2: a dirty
cached line's eviction writeback can land on and clobber a later bypass
write to the same physical DRAM address — real data corruption, not just
a misclassification). Fixed: `(match_addr & mask) == (base & mask)`; the
mask convention is now also documented at the `WIN_BASE`/`WIN_MASK`
parameter declaration and in §1 above. **Consequence:** under the old
decode, nearly all traffic aimed at the (disabled-by-default) bypass
window actually exercised the cache path instead — so this fix gives the
bypass engine, its arbiters, and the same-ID active-request check their
first real soak under the full tb suite (§10, Round-4).

**Important-A (v2, found by final-gate review on the round-3 fix):** the
round-3 reset grant-hold (`l2c.v`'s AW/W/B arbiter, keyed off `aw_busy`)
over-approximated. `aw_busy` sets the moment an AW is first PRESENTED
(`by_awvalid || vw_awvalid`), not once it's actually ACCEPTED — so a
reset while the victim sat in `S_AW` unaccepted (routine downstream
backpressure on `m_axi_awready`, nothing unusual) held the grant for a
transaction DRAM never actually committed to. `l2c_victim.v` itself
already handles this correctly (its reset `case` falls through to
`default: st <= S_IDLE` for `S_AW`, no drain queued, no `stray_b_owed`
bump) — but with the old `aw_busy`-keyed hold, `l2c.v`'s arbiter kept
waiting for a B that would never arrive, since nothing was ever open.
Probe-confirmed: a post-reset bypass write times out, and with a second
bypass op queued behind it, permanent head-of-line deadlock (near-certain
in a scanout config, where bypass and victim writebacks interleave
routinely). Fixed by tracking a new `aw_open` register — set on a genuine
AW handshake (`m_axi_awvalid && m_axi_awready` while granted to victim),
cleared on the matching B handshake — and holding the reset grant on
`aw_open && !aw_grant` instead of `aw_busy && !aw_grant`. `aw_busy` itself
is unchanged and still locks on PRESENTATION for the normal (non-reset)
arbitration path — that's the separate Critical-4 mechanism and must not
be weakened. New directed scenario: reset during `S_AW` pending-
unaccepted, then a bypass write completes promptly (no HOL stall). See
§10, Round-4.

**Spec note (protocol-checker relevant, not a bug):** during the
`S_RSTDRAINW` window, forcing `m_wvalid` low mid-presentation on the
reset-assert edge (the one-cycle gating fix from round 3, §9
Important-A) is, strictly, a technical AXI stability violation —
`m_wvalid` drops while the beat it was presenting hasn't been accepted.
It is unavoidable under this module's present-nothing-during-`rst`
policy, and the drain re-presents the exact same burst position on the
next cycle once `rst` deasserts, so no data is lost or reordered. A
future protocol-checker run against this module should expect and
tolerate exactly this one-cycle exception at the reset-assert edge.

## 10. Verification results (T10 bring-up)

`tb/tb_l2c.{v,cpp}` (see the Makefile's `tb-l2c` / `tb-l2c-bypass-all`
and `tb-l2c-stress` stanzas) exercises every scenario in the brief: read miss+fill, read hit,
write miss+merge, write hit dirty, dirty eviction writeback ordering,
victim-buffer hazard, MSHR merge, per-line replay saturation, MSHR-full
backpressure, ARREADY hold/release, hot-hit progress under eight blocked
cold fills, eight pipelined fills with reverse and shuffled RID returns,
fill-error suppression/retry, concurrent same-set dirty evictions, bypass
read/write ordering, reset walk, back-to-back mixed bursts, an
L2_BYPASS_ALL pass-through equivalence check, and a 20,000-op randomized
scoreboard against a host-side golden flat-memory model (final read-back
verification through the DUT, no RTL backdoor). The memory model uses an
independently-seeded, reproducible 200+random(0..63)-cycle first-response
latency, 0-3 cycle intra-burst gaps, and randomized READY/backpressure. The
random scoreboard keeps up to 24 operations (often 80-100 16-byte chunks)
in flight and mixes a hot working set, sequential streams, same-set pressure,
random addresses, 1/2/3/4/8-beat bursts, partial writes, and bypass accesses.
The default 20,000-op run plus three independent 50,000-op seeds pass and reach
all eight MSHRs. Measured
average hit round-trip latency: ~5.1-5.3 cycles across runs, inside the
~4-6 cycle target.

Three real RTL bugs were found and fixed during bring-up (all still
present in the delivered RTL as fixes, documented inline at their fix
sites and summarized in §5/§6 above):

1. **MSHR SCAN starvation** (§5) — fixed lowest-index scan + fixed
   lowest-index free-slot allocation combined to permanently starve
   higher-index entries under sustained concurrent load. Fixed with a
   round-robin scan pointer.
2. **Bypass priority inversion** (§6) — bypass was originally lowest
   priority on all three shared-resource arbitrations (both AXI-master
   sub-ports and the response mux), which is exactly backwards: bypass
   occupies the single front-door slot until serviced, so starving its
   response wedges the whole module, not just bypass traffic. Fixed by
   giving bypass top priority everywhere (cheap, since it's rare and
   single-beat).
3. **l2c_bypass response-ID corruption** — the bypass engine stored the
   *outbound-tagged* master ID (with the bypass marker bit forced) into
   the register also used to source `rsp_id`, so responses were echoed
   back with the wrong ID. Fixed by storing the untagged original
   requester ID separately from the tag applied only to the outbound
   `m_awid`/`m_arid`.

One correctness bug was found and fixed in the testbench itself, not the
RTL: `tb_l2c.cpp`'s handshake-detection originally re-read
`valid`/`ready` signals *after* raising `clk`, racing against DUT
registers that update on that same edge (a signal like `s_axi_awready`,
driven combinationally from a register that also updates on the
accepting edge, can appear to have "already dropped" by the time it's
re-checked post-edge). Fixed by detecting all handshake firings on the
settled *pre-edge* snapshot instead.

### Round-2 verification results

`make tb-l2c` now reports **18 PASS / 0 FAIL** (up from round-1's 12), and
`make tb-l2c-bypass-all` reports **3 PASS / 0 FAIL** (up from 1) — the
increase is the widened tb (Important-13) plus a new directed scenario
per Critical finding: `multi_beat_write_hit_single_bresp` (Critical-1),
`illegal_burst_slverr` (Important-10), `same_id_ordering` (Critical-3),
`mshr_replay_partial_write` (Critical-6), plus `victim_buffer_hazard`
widened to also probe a nonzero line offset (Critical-2), and two new
always-on generic checks appended to every run:
`unmatched_response_protocol_check` (any B/R that doesn't match a live
outstanding tracking entry now FAILS the run, not just a debug-gated
print) and `axi_payload_stability_check` (asserts AW/AR/R/B payload
doesn't change while VALID is held with READY low, on both the DUT's
master port and its slave-side responses — this is what actually
exercises the Critical-4/5 locked-arbiter fixes, since it needs a real
VALID-but-not-READY window to trigger; `s_axi_bready`/`s_axi_rready` are
now randomized ~85% instead of hardwired to 1 specifically to create
that window). The randomized scoreboard's op generator now also mixes in
multi-beat bursts (~20% of ops, 2-4 beats) and partial write strobes
(~33% of writes) instead of single-beat/full-strobe only. Determinism
re-checked (same binary, two runs, byte-identical stdout).

Four real RTL bugs were found and fixed during round-2 bring-up beyond
the 7 Critical / 7 Important / 3 Minor findings the review already named
explicitly (see §9) — these four were only *caused by* implementing
those fixes, not present in the round-1 baseline, and were only caught
because the tb was widened at the same time (backpressure + payload-
stability assertions + the unconditional unmatched-response check, per
project policy: fix the RTL and widen the tb together):

1. **Response-mux lock same-cycle race** (`l2c.v` `r_lock`/`b_lock`):
   the Critical-5 lock's own accept-clears-the-lock check only ran in the
   branch where the lock was *already* set from a prior cycle; a
   freshly-picked response that happened to be accepted on the SAME cycle
   it was first picked (common with `s_axi_*ready` mostly high) left the
   lock set with nothing left to legitimately hold, producing one phantom
   extra beat of stale, already-consumed payload the following cycle —
   observed as a spurious duplicate B with a since-reused ID, silently
   corrupting an unrelated later write. Fixed by checking the accept
   condition with priority over (not alongside) the lock-in condition, so
   it applies whether the lock was already held or is being set this same
   cycle.
2. **`s_wready` missing `id_busy_c`**: added everywhere else `do_accept_c`
   is gated, but not in `s_wready`'s own formula — so the requester could
   see a valid W-beat handshake the front door had not actually captured
   (blocked internally on `id_busy_c`), silently corrupting whichever
   later write happened to be presented once the beat was *actually*
   captured. Fixed by adding `!id_busy_c` to `s_wready`.
3. **`s_wready` stale `!ar_have` proxy**: `s_wready`'s formula still
   hard-required no read pending at all (`!ar_have`), which was only
   equivalent to "this write is the one selected" back when reads had
   fixed absolute priority; Important-11's `rw_favor` fairness fix made
   it possible for `write_sel` to win a tie *while* `ar_have` was still
   1, silently reintroducing the same false-ready class of bug as (2).
   Fixed by deriving `s_wready` directly from `write_sel` instead of the
   stale `aw_have`/`ar_have` proxy, so it tracks the real arbitration
   outcome as it evolves.
4. **Testbench same-ID map collision** (`tb_l2c.cpp`, not RTL): the
   driver's own `r_awaiting` was a single-entry-per-id map; a *new* AR
   header for an id can legitimately be accepted (buffered) while an
   *older* transaction under the same id is still unresolved and blocked
   on `id_busy_c` at the dispatch stage — the RTL correctly serializes
   *dispatch and response* per Critical-3, but AXI never promised the
   header-accept step itself would wait, so the testbench's single-entry
   map let the newer arrival's tracking struct silently clobber the
   older one's before its data had even arrived. Fixed by making
   `r_awaiting` a per-id FIFO (`std::deque`), which is also the more
   accurate model of AXI4's actual same-ID completion-order guarantee.

### Round-3 (Important-A) verification results

`make tb-l2c` now reports **21 PASS / 0 FAIL** (up from round-2's 18) —
three new directed scenarios: `reset_mid_w_burst` (a writeback's AW is
accepted but only 2 of 4 W beats sent when reset hits — the master port
must not be left wedged waiting on the abandoned burst),
`reset_b_pending_next_writeback_correct` (the EXP D reproduction — reset
with a writeback's BRESP still outstanding, then a SECOND, fully
independent eviction must still complete correctly), and
`reset_compose_fill_and_writeback` (reset while an MSHR fill and a
victim writeback are both mid-flight simultaneously, proving the R-side
S_DRAIN and the new W/B drain compose without interference). Determinism
and lint re-checked; both clean.

One more real RTL bug was found and fixed during THIS round's own
bring-up (see the Important-A entry in §9 for the full writeup): every
master-port output driven by `S_RSTDRAINW`/`S_RSTSINKB` needed `!rst`
gating applied UNIFORMLY across all their terms, not just the new-state
terms — the ungated `(st==S_W)` term stayed live for one extra cycle on
the exact edge `rst` first asserts (before the state NBA takes effect),
leaking one uncounted real-data beat that desynced DRAM's beat count by
exactly one and hung the drain permanently. Caught by a directed
scenario, not the randomized run — this class of one-cycle reset-edge
bug is inherently a "you have to specifically construct the race"
finding, reinforcing why Important-A asked for dedicated directed tests
rather than trusting the existing randomized coverage to stumble onto
it.

**Minor-C (testing, not yet hardened):** `victim_buffer_hazard`'s
nonzero-offset probe and `mshr_replay_partial_write` both rely on the
mem model's *default* latency distribution (`mslv.lat()`, 10-60 cycles,
`gate(80)`) to land their read/write race naturally inside the relevant
hazard window — they are not deterministically pinned to that window.
In practice this has proven reliable at the fixed seed this suite runs
at, but a future latency profile change (or reseeding) could silently
stop exercising the intended race while still reporting PASS. The
IMPORTANT-A scenarios above sidestep this by using `g_dbg_aw_accepts`/
`g_dbg_w_accepts` beat-counters plus (for scenarios that need it)
`g_mem_force_ready` to make the race deterministic instead of
latency-dependent — the same technique should eventually be extended to
Critical-2/Critical-6's own scenarios (a master-port backpressure/
force-timing hook, generalized beyond the write-side-only hook added
this round) so they stop depending on the mem model's random timing.
Documented here as a known limitation, not implemented in this round.

### Round-4 (final-gate) verification results

Two probe-confirmed findings fixed this round (§9): the CRITICAL
`l2c_bypass.v` mask-sense inversion and the Important-A v2 `aw_open`
reset-hold fix in `l2c.v`. Both new directed scenarios added and green,
and — per the coordinator's explicit instruction — the full suite
(including the 20k-op scoreboard) was re-run in full rather than spot-
checked, since the mask fix changes which traffic actually reaches the
bypass path for the first time. See the commit's `t10-report.md`
Round-4 entry for the exact PASS counts and any new bugs the re-soak
surfaced.
