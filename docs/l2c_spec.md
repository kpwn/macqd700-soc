# L2 system cache

The SoC L2 is implemented by [l2c.v](../rtl/soc/l2c.v) and connected in
[fpga_top_ddr.vh](../rtl/soc/fpga_top_ddr.vh) when `L2C_ENABLE` is set.
It is separate from the CPU's L1 instruction and data caches.

## Organization and interfaces

- 2 MiB capacity: 4,096 sets, eight ways, 64-byte lines.
- URAM-backed line storage; four 128-bit quadrants per line.
- A 128-bit AXI read/write port for crossbar S0, with 6-bit IDs.
- A dedicated 256-bit read-only instruction-fetch port, with 4-bit IDs.
- A dedicated 128-bit read-only D-side refill port (`dh_axi_*`, "the hot
  door"), with 2-bit IDs, gated by `DH_PORT_EN` and **off by default**.
- Its write half (`dw_axi_*`, AW/W/B, 128-bit, 2-bit IDs, INCR) for
  cacheable writes, gated by `DW_PORT_EN` (requires `DH_PORT_EN`) and
  **off by default**.
- A shared 128-bit downstream AXI interface to the DDR path.
- Eight miss-status entries; default victim-buffer depth eight and
  per-miss secondary replay depth four.

`l2c_ctrl.v` owns lookup, admission and response sequencing. `l2c_mshr.v`
tracks refills and merged operations; `l2c_victim.v` holds dirty evictions;
`l2c_bypass.v` handles configured nonallocating windows. Fetch responses
reassemble internal quadrants into the CPU's native 256-bit beats.

## The hot/cold memory split (P6, `DH_PORT_EN`)

`DH_PORT_EN` adds a third front door and is the SoC half of the hot/cold
split. It is one parameter and it gates four things together: the `dh_axi_*`
port, a two-bit front-door source tag in place of the one-bit fetch flag,
WRAP bursts on the two hot doors, and fixed hot-over-cold priority at S0.
With it 0 every added expression reduces to the one that shipped before it
and every added register loses its consumers, so the OFF netlist is
unchanged; `make tb-l2c` is that configuration and its output is the
regression baseline for the claim.

**Routing.** Cacheable D-side refill *reads* take the hot door. Everything
else the D side does — writethrough and cache-inhibited loads, every store,
L1D copyback writebacks — stays on the cold path through the crossbar, and
so does DMA. The cold path is in-order and one-outstanding per master, which
is correct for cache-inhibited accesses (they are ordered barriers on a
68040) and is the shape a native 68040 bus adapter wants. The demux key is
the MMU cache mode, never an address decode. With `DW_PORT_EN` the cacheable
writes (copyback writebacks, full-line stores) move to the hot door's write
half instead; writethrough and inhibited stores stay cold.

**The hot door has no address decode, so it needs a window guard.** A
cacheable access sent down the cold path is merely slow; an access marked
cacheable that is *not memory* and takes the hot path is **wrong**, because
the DDR side serves `addr mod DRAM size`. That is how the I side once
executed NuBus-slot filler. `ifetch_window_guard.v` is therefore instantiated
in front of `dh_axi_*` exactly as it is in front of `f_axi_*`, at 128 bits
and 2 ID bits, and its `fault_count` is the counter to read if a boot
misbehaves. Do not treat the guard as optional: with it removed, a hot read
of a bypass-window address trips `l2c_ctrl`'s non-LSU-source assertion in
simulation and would be served from aliased DRAM on hardware.

**Priority has a bounded fairness backstop, and that is not optional
either.** `DH_COLD_FAIR_N` (default 64) forces the cold path to win one
arbitration after that many consecutive cycles of being offered and refused.
With `DH_COLD_FAIR_N=0` — pure fixed priority — a saturated hot stream takes
3,000 of 3,000 cycles and the cold path gets none, and two pre-existing
scenarios fail: `fetch_id0_does_not_starve_lsu_bypass_id0` and
`dual_source_response_association_stress`. `make tb-l2c-dh-nofair` is kept as
that negative control. The flag is a register, so the door's select cone sees
a flop rather than a comparator.

**WRAP.** Both hot doors accept INCR and WRAP; the cold path stays INCR-only.
WRAP is what lets a 64-byte line fill be issued demanded-sector-first as one
burst. The per-beat address is the base's bits above the wrap block combined
with the linear address's bits inside it, where the block mask is
`((arlen+1) << arsize) - 1`; a WRAP whose `arlen+1` is not a power of two is
not a legal AXI burst and is answered SLVERR rather than given a nonsense
mask.

**Source identity.** `SRC_BITS` widens the MSHR's internal `{source, id}` tag
from one bit to two. An inconsistent `(DH_PORT_EN, SRC_BITS)` pair would alias
the D source onto the LSU source — the same numeric ID on two ports would
become one transaction and responses would leave on the wrong bus — so it is
an elaboration error, not a comment.

**What this does NOT do.** It adds no L2 bandwidth and is not meant to: the
hit path already moves 16 B/cycle against a core that asks for well under
1 B/cycle. Its value is that it removes the crossbar's one-outstanding cap
from the D refill path, which is what makes an L1D MSHR file worth building.
Banking the L2 by set was considered and rejected for the same reason.

**Core-side prerequisites.** The port cannot be enabled end to end until the
CPU socket carries `dha_*` and the core enforces (a) an inhibited access as a
full barrier launched at ROB head under `dQuiesce`, and (b) a per-line
writeback-address gate that holds an `axi_dh` AR for line X while X is in the
L1D writeback buffer awaiting its B. The fabric contributes no cross-path
ordering between the doors and does not need to; what it does guarantee is
that once a cold write has been *accepted* at the front door, a subsequently
accepted hot read of the same set observes it, and that a line whose dirty
copy is still in the victim buffer is never answered from DRAM.

## The hot door's write half (P6W, `DW_PORT_EN`)

**Why.** With the read half built, every cacheable *write* — L1D dirty
writebacks and coalesced full-line stores — still crossed the crossbar,
which allows one outstanding write per master. l2c's own share of that is
7 cycles per 16 B (measured below, AW to B at an L2 hit); the crossbar's
hops come on top. That caps any copy at under ~2 B/cycle regardless of how
many read MSHRs the L1D has.

**What.** `DW_PORT_EN` gives the hot door AW/W/B. Inside `l2c_ctrl` it is a
fifth accept candidate with its **own header tracker and its own W
channel**, tagged `SRCV_DH`, in the hot class. Because each source's W beats
arrive on their own channel and are consumed at S0 against their own
tracker, no W-owner lock is needed (the objection that deferred "a hot port
read+write" in the P6 design assumed one shared W channel). It never uses
the full-line gather: a full-strobe 16 B beat is already a no-fetch
install. Hot write IDs live in the DH source's *write* ID space — the
native ID with bit `ID_WIDTH-1` set — so a hot read ID k and a hot write
ID k are distinct `{src,id}` tags in the MSHR ordering CAM (AXI read and
write IDs are independent; aliasing them would make a writeback's B wait
on an unrelated refill). With `DW_PORT_EN=0` every added expression is the
P6 one; `make tb-l2c` and `make tb-l2c-dh` are byte-identical to the tree
before P6W (whole output file, diffed).

**Arbitration.** Within the hot class, hot writes and hot reads alternate on
`dw_favor` (fetch vs dh reads still alternate on `hot_favor`), so neither
half of the hot door starves the other. The cold path keeps its
`DH_COLD_FAIR_N` backstop, and gains a second, cheaper one:
`hot_haz_q`, a register meaning "the hot class held the door last cycle and
`set_haz_c` refused its beat". A sequential write stream to the quadrants
of one line is refused by the array read-after-write interlock on most
cycles; under pure hot > cold priority those cycles were simply lost
(measured: cold got 46 of 3000 cycles while the door idled 60%). Yielding
the next arbitration to a waiting cold beat raised that to 900 of 3000 with
no loss on the hot stream (1199 hot-write accepts either way).

**What B means — the ordering contract.** B is produced only after the write
was accepted at S0: at the resolve stage for a hit or no-fetch install, or
by the MSHR after install for a partial-strobe miss. Every read accepted at
S0 after that, on any door, observes it. AW/W/B carry **no** ordering
against `dh_axi` AR/R, so the core must hold a hot read of line X while any
write of X, on either door, lacks its B — the same writeback-address gate
the read half already requires, extended to cover writes issued on this
port. `dw_read_after_b_observes_write` checks both halves: a read launched
on the B handshake is never stale (240 probes over hits, evicting
installs, MSHR fills and same-set churn, with W lagging AW by up to 24
cycles), and the same writes with the read launched *alongside* the write
come back stale 159 times in 160 — so the gate is load-bearing and the test
can see a stale read. A posted-B mutant (B at AW accept) makes 203 of the
240 gated probes stale. `make tb-l2c-dw-mutants` runs 14 single-mechanism
mutants of the write door: 13 are caught (by data, ordering or fairness
checks, or by an RTL assertion) and one — a hot write follower allowed to
overlap a same-line read leader — is an argued equivalent mutant that must
survive (see the script for why).

**The write guard** (`dh_write_window_guard.v`, in front of `dw_axi_*` as
`ifetch_window_guard` is in front of `dh_axi_*`). RAM window: forwarded.
ROM mirror: dropped with OKAY — the crossbar's policy for a CPU-LSU write
to ROM, so a copyback-dirtied ROM line behaves as on the cold path.
Anything else: DECERR, data discarded, counted. A locally answered burst
starts only when nothing forwarded is outstanding, which keeps per-ID B
order. On `abandon` (= `cpu_rst`) the guard refuses new AWs, completes any
burst the dead core left mid-W with WSTRB=0 no-op beats, finishes a local
drain and swallows every B owed to the dead core, until all of it has
retired — the write-side twin of the crossbar's `m0_pad_w`.

**Throughput** (`make tb-l2c-dw`, L2 hits, deterministic memory):

| traffic | cyc/op | B/cycle |
|---|---:|---:|
| cold s_axi, 1 outstanding, 16 B (l2c's share of today's path) | 7.00 | 2.29 |
| hot door, window 1, 16 B | 7.00 | 2.29 |
| hot door, window 2, 16 B | 3.51 | 4.56 |
| hot door, window 4 or 8, 16 B sequential | 2.51 | 6.37 |
| hot door, window 8, 16 B striding 64 B | 1.01 | 15.78 |
| hot door, window 2, 64 B bursts | 10.06 | 6.37 |
| hot door, window 2, with a saturated hot-read stream | 3.51 | 4.56 (reads meanwhile 11.4) |
| memcpy at L2: 4 hot reads + 2 hot writes in flight | 3.52 | 4.54 copied |

The single-outstanding latency is identical on both doors; the gain is
concurrency. Sequential 16 B writes and 64 B bursts stop at 2.5 cycles per
16 B because consecutive quadrants of one line may not overlap in the
lookup pipeline (`set_haz_c`); a gather on this port (as the cold path has)
would lift 64 B bursts to ~16 B/cycle and is the lever for 64 B sectors —
not built.

### Latent: the hot read port ignores the reset ROM overlay

After a cold or warm SoC reset the crossbar aliases CPU **reads** of
`[0, AXI_ROM_SIZE)` = `[0, 4 MiB)` onto the ROM (`apply_cpu_overlay`,
`axi_xbar.v`), so the 68040 reset vectors come from ROM. The alias stays
armed until the first CPU data read of the ROM mirror (`cpu_rom_read_seen`,
committed once no CPU read is in flight) and is re-armed by
`cpu_overlay_reset` = `soc_full_rst | warm_peripheral_reset`. Writes are never
aliased. With `L2C_ENABLE` only master M0 (`axi_d`) can disarm it, because
instruction fetch reaches l2c directly and M2 is tied off.

`dh_axi` (and `f_axi`) also reach l2c directly and apply **no** overlay. Both
guards pass `[0, 4 MiB)` as RAM. So a hot read of a low address while the
overlay is armed returns **DDR RAM**, where the cold path would return
**ROM**: wrong data, not a fault. The write half is consistent (the overlay
never applies to writes).

It is unreachable today because it needs **all** of the following at once:
1. `effective_cpu_overlay_active` = 1, i.e. no CPU data read of the ROM mirror
   has completed since the last cold/warm reset;
2. `CACR.DE` = 1, with the line's cache mode not inhibited. With `TC.E`=0
   every address defaults to WRITETHROUGH, which is cacheable, so the
   routing key sends the miss hot;
3. a data read of `[0, 4 MiB)` that misses the L1D (a refill or MSHR
   allocation, i.e. the hot door).

`CACR` resets to 0, so (2) is false from every core reset until the ROM turns
the cache on, and the ROM reads its own tables through the mirror long
before that. What would make it **reachable**:
- a reset path that re-arms the overlay **without** resetting the core's
  `CACR` (a `warm_peripheral_reset` that leaves the CPU running, or a
  debug flow that pulses `cpu_overlay_reset` alone);
- a core change that sends the reset-vector reads, or any read before the
  first ROM data read, down the cacheable/MSHR path. For example, a
  `ResetVectorFsm` load given `ooOk`/a cache mode instead of the inhibited
  legacy FSM;
- boot code on a non-Q700 machine where the VIA1 overlay input is live and
  software re-asserts the overlay with the data cache on;
- the same holds for `f_axi`: an instruction fetch below 4 MiB with the
  overlay armed.

**Fixed (2026-09-30), loud instead of wrong.** `ifetch_window_guard` has an
`OVERLAY_GUARD` parameter and an `overlay_active` input (=
`xbar_overlay_effective`). With it set, a read of `[0, OVERLAY_SIZE)` =
`[0, 4 MiB)` while the overlay is armed is out of window, so it gets DECERR.
- **Hot read port (`u_dh_guard`):** `OVERLAY_GUARD=1` always. The port is new
  and off by default.
- **Fetch port (`u_ifetch_guard`):** has the same mismatch, but that guard is
  in the shipping build, so the term there is opt-in (`IFETCH_OVERLAY_GUARD`).
  Turning it on changes what a shipping bitstream answers. The Q700 boot
  path never fetches below 4 MiB with the overlay armed, because the reset PC
  is in the ROM mirror.
- **Write port:** no term. The crossbar's overlay applies to reads only (CPU
  writes to low memory always reach real RAM), so the write door already
  matches the cold path. A DECERR there would be a new mismatch, not a fix.

`dh_read_below_4mib_with_overlay_armed_is_decerr` (`tb-l2c-dw`) fails with
the term removed: the read returns DDR RAM with OKAY. That removed-term
mutant is `dh_guard_ignores_overlay` in `make tb-l2c-dw-mutants`.

### Next ceiling, priced (not built): a gather on the hot write port

**The wall.** Consecutive writes to one 64 B line cannot overlap in the
lookup pipeline. The resolve stage read-modify-writes the tag entry's
valid/dirty sector bits (`tw_vsec`/`tw_dsec`) from its own array snapshot, so
a follower resolving on a stale snapshot would drop its leader's bits. So
`set_haz_c` blocks write/write in a set, and a sequential 16 B write stream
or a 4-beat 64 B burst on the hot port runs at 2.51 cycles per 16 B
(**6.37 B/cycle**). Letting same-line writes overlap would mean forwarding
the leader's sector bits into the resolve cone, which is this design's
tightest path, so that option is rejected. The cold path already has the
fix: `flw_*` gathers a 64 B-aligned, whole-line INCR burst into one 512 b
register and dispatches it as a single `req_full` install, taking 4 W beats
at one per cycle. Its measured row is `stream_write_64B` at 5.06 cycles per
64 B line (**12.6 B/cycle**), with the one-cycle G_FULL-to-next-gather
turnaround accounting for the gap to 16.

**This only pays with 64 B writes.** Today's 16 B L1D writebacks are
separate single-beat bursts, which a gather cannot combine. The lever
arrives with 64 B sectors (`awlen`=3 writebacks) or 64 B-coalesced stores,
and until then 6.37 B/cycle is about 3x what the core can generate.

**Option A (recommended): share the existing gather between the two
write sources, owner-locked.**
- A 1-bit gather owner and source tag (`flw_src`) are set at arm.
- A 128 b 2:1 mux selects `s_wdata`/`dw_wdata`, plus a 16 b one for strobes,
  in front of `flw_data`/`flw_strb`. It sits on a register D-input, off the
  accept cone.
- There is an arm condition per source; `dw_aw_*` gains the `aw_flw`-style
  shape bit.
- The raw-beat term of `write_avail`, and the hot one, change from "gst ==
  G_IDLE" to "gst == G_IDLE or the gather belongs to the other source". A
  line gathering for one source then never blocks the other source's raw
  beats. That is the only arbitration change.
- The dispatch reuses `cur_*`/`a_line`/`req_full` unchanged, except that
  `cur_src`/`cur_id` take `flw_src`/`flw_id` on a gathered dispatch.
- The contention cost: while one source holds the gather, the other
  source's full lines fall back to raw beats (correct, 2.5 cycles per 16 B)
  for at most one line.
- **Estimate: ~150-250 LUT** (the 128 b mux packs about 2 bits per LUT6_2,
  ~64-128 LUT, plus ~40 LUT of arm/owner/select and ~30 LUT of dispatch-tag
  muxing) **and ~15 FF**.
- Expected hot 64 B bursts: 5.06 cycles per line, 12.6 B/cycle (2.0x).
- A side benefit: `dw_wready` for a gathered beat stops depending on
  `do_accept_c`, since the gather intake bypasses the accept cone. That
  removes the timing concern noted for the write port on its most important
  traffic.

**Option B: a second, private gather for the hot port.** 512 b data + 64 b
strobes + ~40 b control = **~620 FF**. It needs a 512 b 2:1 mux into
`a_line` (~256-512 LUT) and a replay-quadrant arm into `cur_wdata`/`cur_wstrb`
(~150 LUT). **~450-700 LUT** in total, and a second `gst` term in the accept
cone. It buys the same 12.6 B/cycle with no cross-source contention. That
is not worth ~620 FF while cold full-line writes are rare (the DMA and
boot loader).

**Tests it would need.**
- Hot 64 B bursts interleaved with cold 64 B bursts, both sources gathering.
- A partial-strobe hot line forcing G_RPLY.
- A CPU reset mid-gather: the guard pads, and the gather must complete a
  padded line as G_RPLY with zero strobes, not as G_FULL.
- A dw throughput row reaching at least 10 B/cycle on 64 B bursts.
- Mutants: the other source's beat written into an owned gather; `flw_src`
  not applied to the dispatch, so the B is misrouted.

## Addressing and coherency

The crossbar flattens RAM, ROM and legacy framebuffer addresses before S0.
The direct fetch port has its own ROM-mirror folding because it does not
pass through the crossbar. Both paths must identify the same backing line.

L2 serializes accesses that reach it; this does not make CPU L1 contents
coherent with arbitrary host or DMA writes. CPU cache maintenance remains
an L1 concern. Dirty L1 writebacks arrive as ordinary AXI writes, not as
special L2 flush commands.

With `VRAM_IN_DDR`, the pixel aperture and scanout join the DDR path
downstream of L2. They do not allocate cache lines. Keep cached and bypass
addresses disjoint: exposing one physical location through both paths
would allow stale cache lines to hide direct writes.

## Ordering and progress

Front-door header staging allows another header to be accepted while a
burst is decomposed. Admission, miss merging and replay must preserve
source identity, byte strobes, beat count and response ordering.

Fetch IDs and crossbar IDs are different namespaces. Hazard checks must
include the source; the same numeric ID on different ports is not the
same transaction. A refill may become visible only with valid data and
the corresponding tag/state update.

Lines can be partially valid. A refill must preserve already resident dirty
quadrants and install only the missing data; merging into a fetched copy
must not overwrite a newer write to a valid quadrant.

The MSHR install interface is a registered one-cycle valid pulse, accepted
by the array-write arbitration without a ready handshake. Its data comes
from the active-line register: primary/replay byte merges update that
register on the same edge that asserts install-valid, and it stays unchanged
through the consuming edge. Data outside a valid install is unspecified.
There is no separate install-data register and no added cycle. Tag, dirty
state and byte strobes remain registered alongside the valid pulse.

AXI arbiters hold payloads stable from VALID assertion through acceptance.
The write path cannot interleave different owners' W bursts: AXI4 has no
WID. Victim writeback completion and refill ordering must prevent an old
dirty eviction from overwriting newer data at the same address.

Lookup completion factors outcome readiness separately from the common
live-request, skew and victim-buffer guards. This lets the late victim CAM
result qualify completion directly rather than traverse each action's
qualification before reaching the shared array enable. No pipeline stage,
acceptance rule or response cycle changes. The individual action wires keep
their original conditions; `make check-l2-completion` exhaustively compares
the completion decision with their original combination over 65,536 two-state
input combinations. This is a logical equivalence check, not a timing claim.

The MSHR lookup exports both its binary index and a one-hot selected entry.
Both choose the same lowest-index match; no match produces a zero mask.
The one-hot result is formed directly from the match vector, not by decoding
the binary index. Same-ID merge ordering masks only this selected entry;
all other entries, replay slots and the bypass owner retain their existing
ordering checks. Preserve priority even for a multi-match input rather than
assuming such an input cannot occur. No state, cycle or acceptance rule changes.

See the [fabric concurrency contract](fabric_concurrency_contract.md) for
the shared interface requirements. Capacity changes must retain correctness
under full miss, replay and victim queues, not merely under a one-request
testbench.

## Reset and bypass

Reset invalidates cache state through the reset machinery; requests remain
blocked while initialization is incomplete. Accepted downstream operations
still need a completion/drain policy. Do not reuse a response ID or release
a reset generation while old traffic can be mistaken for a new refill.
The integration's reset/recovery wiring is part of this contract.

`L2_BYPASS_ALL` is a build-time bypass option. Per-window bypass is a
different mechanism and must not overlap the cacheable range. Neither is
a substitute for CPU L1 maintenance.

## Verification

The Makefile includes `tb-l2c`, `tb-l2c-chain`, `tb-l2c-wstream` and the
VRAM/DDR chain tests. `tb-l2c` builds the hot door OFF and `tb-l2c-dh` builds
it ON (`tb-l2c-dh-stress` runs the randomized scoreboard with it on,
`tb-l2c-dh-nofair` is the fairness negative control described above).
`tb-l2c-dw` builds both halves of the hot door ON; its scoreboard sends a
third of the cacheable traffic through the hot read and write doors
(`tb-l2c-dw-stress`: 5 seeds x 100k ops), and `tb-l2c-dw-mutants` rebuilds it
once per single-mechanism mutant of the write door and requires every one
to fail. The
hot door's scenarios cover the door itself, INCR and WRAP beat order, the
window guard refusing a non-memory address, per-source arbitration counts,
AXI-ID-namespace independence from the LSU port, hot/cold same-set
interleaving against the array read-after-write interlock, and a read of a
line whose dirty copy is pinned in the victim buffer with the DDR write
channel held shut. Each of those was confirmed to FAIL on a deliberately
broken copy of the RTL before being trusted. Unit tests and integrated CDC/MIG tests cover different
boundaries; passing one does not establish the other. Test queued traffic,
same-line conflicts, partial writes, backpressure and reset with operations
in flight. Current suite limitations are recorded in the main README.
