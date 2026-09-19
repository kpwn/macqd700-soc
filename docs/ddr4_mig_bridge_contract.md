# DDR4 pcie_test MIG Bridge Contract

This note records the non-Vivado contract between the repo memory fabric and
the known-good DDR4 MIG generated in `~/FPGA/pcie_test`.

## Known-good MIG facts

`tools/check_ddr4_pcie_test.py` extracts these from the working Vivado
project:

| Item | pcie_test MIG |
|---|---|
| AXI data width | 256 bits |
| AXI address width | 31 bits |
| AXI ID width | 1 bit |
| UI clock | 333.25 MHz |
| DDR4 data width | 32 bits |
| DDR4 part | MT40A512M16LY-075 |
| DDR4 memory period | 750 ps |
| DDR4 input clock period | 5000 ps |
| Reset topology | active-low board reset inverted into MIG `sys_rst`; `c0_ddr4_ui_clk` drives `proc_sys_reset` and AXI |

The repo DDR-facing xbar contract remains 128-bit data, 6-bit ID, and
32-bit address.

## Shim in tree

`rtl/board/axi_ddr4_mig_bridge.v` bridges the repo-side 128-bit/6-bit-ID/
32-bit-address AXI4 contract to the pcie_test MIG's 256-bit/1-bit-ID/
31-bit-address AXI4 contract.  It is wired into both DDR paths in
`rtl/board/ddr_ctrl.v`: the `SIM_MIG_BRIDGE` sim variant (against the
behavioural `sim_mig_backend.v`) and the real-hardware `USE_REAL_MIG` path
(against the actual MIG UI, crossed through `axi_async_bridge`).

The bridge accepts only the traffic the repo fabric ever emits:

| Repo-side request | Bridge behavior |
|---|---|
| aligned 128-bit beat/burst, `size=4`, `burst=INCR`, `addr[31]=0`, `addr[3:0]=0` | forwarded, packed 2 repo beats per 256-bit MIG beat |
| narrow single beat, `len=0`, `size∈{0,1,2}`, `burst=INCR`, `addr[31]=0` | forwarded as one (possibly WSTRB-partial) 256-bit MIG beat |
| `addr[4]=0` on a beat | uses the lower 128-bit data/strobe half of its MIG beat |
| `addr[4]=1` on a beat, or the first beat of a burst starting there | uses the upper 128-bit half; if it has no partner (lone edge beat) the other half is zero-filled |
| any high address (`addr[31]`), misaligned beat, non-INCR burst, unsupported size, or malformed `WLAST` placement | completed locally with `SLVERR`, never forwarded to the MIG |

Repo response IDs are preserved locally (a small per-descriptor field, see
below) because the pcie_test MIG only has a 1-bit ID.

## Multi-outstanding / cut-through contract (T11)

The bridge pipelines both directions instead of serializing one repo-side
transaction end-to-end per direction:

### Reads — up to `RMAX_OUTSTANDING` (default 8) in flight

- AR acceptance (`s_arready`) is gated only by descriptor-queue occupancy
  (a FIFO of accepted-but-not-yet-drained reads), not by whether earlier
  reads have finished returning data.
- AR issuance to the MIG (`m_arvalid`) is a second, independent FIFO walk:
  it issues as fast as `m_arready` allows, constrained only by (a) the
  same-64B-line hazard check (below) and (b) the requirement that **read
  data returns strictly in the order AR commands were accepted**. The
  bridge always drives `m_arid=0`, so this is simply the plain AXI4
  same-ID in-order-response rule — the protocol itself, not any
  MIG-specific reordering behaviour, is what guarantees it.
  `docs/ddr4_mig_generation.md` is silent on internal MIG scheduling, so
  the design deliberately does not lean on any such assumption. This
  bridge's drain-side bookkeeping tracks only its own issue order (no
  MIG-side tags), so if a future downstream device ever violated the
  same-ID ordering rule it is required to follow, beats would be
  mis-associated — flagged here for anyone re-targeting the bridge at a
  device with a different ID-width contract that could paper over an
  actual violation.
- Data return uses a 2-deep intake skid between the MIG `m_r*` channel and
  the repo-side drain. This is what removes the old single-outstanding
  design's "drops a cycle on every lower-half hold" bug: `m_rready` no
  longer has to fall just because a held upper-half beat is draining to
  `s_r` — the skid independently accepts the next MIG beat as long as it
  has room. A same-cycle "bypass" path also lets the very first beat of an
  otherwise-idle read reach `s_rdata` the same cycle `m_rvalid` asserts
  (no forced 1-cycle registration latency), while still registering into
  the hold register in case a second (upper-half) repo beat is needed from
  it.
- A locally-rejected (contract-illegal) read always completes as **one**
  truncated `SLVERR` beat with `RLAST` asserted, regardless of the
  requested burst length — matching the original single-outstanding
  bridge's fast-reject shortcut. `axi_narrow_to_wide` and the xbar never
  issue illegal multi-beat reads in practice, so this is not a general
  multi-beat `SLVERR` burst.

### Writes — up to `WMAX_OUTSTANDING` (default 4) in flight

- AW acceptance is gated only by descriptor-queue occupancy, independent
  of whether the current burst's W data has even started arriving.
- AW issuance to the MIG is decoupled from W-data streaming: it issues as
  soon as `m_awready` allows, in FIFO order.
- W beats are packed into 256-bit MIG beats and pushed into a 2-deep
  *output* skid toward the MIG **as each MIG beat's worth of data
  completes** (cut-through) — not buffered for the whole burst before the
  first byte reaches the MIG, which is what capped the old design under
  20% utilization on long bursts.
- **Ordering window (data vs. command):** the bridge follows the
  conservative rule "data no later than command" — a buffered MIG beat is
  never presented on `m_wvalid` until its owning AW has been issued (or is
  issuing on the exact same cycle; simultaneous is fine, since AW and its
  first W beat routinely arrive from the repo side on the same cycle and
  forcing an artificial 1-cycle gap there would regress ordinary,
  uncontended write latency). The pcie_test MIG UI docs are silent on
  whether a narrower window (e.g. data slightly ahead of command) is also
  legal; this bridge does not attempt it. If a future MIG generation
  documents a wider legal window, this is the constraint to revisit.
- **B response timing:** `s_bvalid` fires once the burst's **last** W beat
  has been accepted by the MIG (`m_wvalid && m_wready` on the final MIG
  beat) — matching how a native `app_wdf_rdy` UI reports write completion
  — rather than waiting on a real downstream AXI B handshake.  `m_bready`
  is tied high (the bridge always sinks the real B channel so the
  downstream slave is never stalled), but the real `m_bresp` is **not**
  re-propagated upstream. **This is a deliberate trade-off documented
  here**: if a real HW MIG ever reports a genuine write error via `bresp`,
  it is invisible to the repo-side master under this contract. Locally-
  rejected (contract-illegal) writes are unaffected — those still get a
  real `SLVERR` (they never reach the MIG at all).
- **Malformed-burst caveat:** a `WLAST`-placement mismatch detected only
  partway through a long burst may already have forwarded a well-formed
  prefix to the MIG before the mismatch is seen — cut-through cannot roll
  back beats already pushed, unlike the old store-and-forward design
  (which buffered the entire burst locally first). The repo-side response
  is still `SLVERR` either way; only the "nothing ever reached the MIG"
  guarantee is weaker for this one malformed-input case. The beat that
  first reveals the mismatch is itself never pushed — only an
  already-pushed *prior* prefix, **or a descriptor's AW being *committed*
  — issued, or irrevocably committed to issuing via sticky
  `aw_presenting`**, makes a descriptor obligated to complete its MIG
  burst. **"Committed" is precisely scoped**: it requires the descriptor
  to be accept-legal (`cur_wdata_ok`, i.e. the AW itself was never
  rejected — misaligned address, wrong size, wrong `AWBURST`, etc.) and
  it is *never* true on the descriptor's own same-cycle accept-and-stream
  ("bypass") cycle, even though a stale array read could otherwise appear
  to say so on that exact cycle (a brand-new descriptor's AW cannot
  possibly have issued yet — see the RTL header note on `wq_aw_committed`
  for the two ways a looser definition previously leaked: an
  accept-rejected descriptor's AW-issue *skip* path being misread as a
  real issuance, and the same-index array read during the bypass cycle
  returning the *previous* occupant of that physical ring slot's
  issued-flag instead of "not yet issued" for the new one). With that
  scoping, a single-beat burst whose only beat has the wrong `WLAST`, on
  a descriptor that is either accept-rejected or whose AW has not yet
  been issued or started presenting, never touches the MIG at all (pure
  local `SLVERR`, same as an address-rejected write) — regardless of
  whether some *other*, unrelated descriptor's AW happens to be
  in-flight or already retired. Because AW issuance is eager and
  independent of W-data streaming, the mismatch can just as easily
  surface *before* any beat is ever pushed on a **legitimately
  committed** descriptor — e.g. an early `WLAST` inside what would have
  been the first MIG-beat pair, with the AW already accepted by the MIG
  or mid-presentation — as after a well-formed prefix. Either way the
  bridge still completes the MIG burst it already committed to: AW is
  never skipped once issued or presenting (a sticky `aw_presenting`
  register holds `AWVALID` across the handshake exactly like the
  read-side `ar_presenting`, so it can never retract without a handshake
  per AXI4 A3.2.1), and any beats still owed beyond the malformed point —
  up to and including the *entire* originally promised `AWLEN` count, if
  zero real beats were ever pushed — are padded with `wstrb=0` filler. A
  real MIG requires exact command/data framing and would otherwise wedge
  its address counter (and, via the hazard-tracking queue that only pops
  on the real MIG `B` response, permanently block any subsequent
  overlapping read) for every subsequent transfer on a short-changed
  burst — the flip side of the same care: an *uncommitted* descriptor
  must never receive orphan padding either, since command-less `W` beats
  desync a real MIG's write FIFO just as badly. The slot is not recycled
  until the MIG has fully drained (real data + any padding).

### Ordering guarantees

- **Same-ID / per-direction ordering**: preserved. Each direction (reads,
  writes) is a strict FIFO from acceptance through drain — descriptors are
  never reordered relative to each other within a direction.
- **Write-then-read to the same 64B line (RAW)**: protected. A queued
  read's AR issuance is held back from the MIG for as long as any
  outstanding write whose 64B-granule line range (`addr[31:6]`) overlaps
  it has not received its **real `m_bvalid`** — tracked in a dedicated
  `bq_` queue (pushed at real AW issuance, popped at real B), deliberately
  *not* the same signal that gates the early upstream B response above.
  Local skid acceptance (what the early B-ack is keyed on) is not
  something either sim backend or `docs/ddr4_mig_generation.md`
  substantiates as the point a subsequent read is guaranteed to observe
  the write, so the hazard uses the strictly stronger real-B signal
  instead. `bq_` is deliberately decoupled from the write descriptor ring
  (whose slots retire — become reusable by a later, unrelated write — on
  local skid-drain, independent of real-B timing): tracking hazard state
  directly on the ring would let a slot recycle before its real B
  arrived, corrupting the hazard state for whatever write lands in that
  physical slot next. `bq_` never overflows: AW issuance is itself
  backpressured by `bq_` occupancy.
- **Read-then-write to the same 64B line (WAR)**: **not** separately
  guarded. A later-issued write is not held back from racing ahead of an
  earlier, still-outstanding read to the same line. This is an explicit,
  documented non-goal of the T11 rework, not an oversight — flagged here
  for anyone building traffic patterns that depend on it. (The repo
  fabric's actual masters — CPU LSU, boot FSM, DMA — do not currently rely
  on this ordering; if that changes, revisit.)

## Testing

```bash
make ddr-pcie-test-check       # known-good MIG facts vs. pcie_test project
make tb-axi-ddr4-mig-bridge    # bridge unit tb (Verilator, no CPU needed)
```

`tb/tb_axi_ddr4_mig_bridge.cpp` scenarios, beyond the original per-field
contract checks (address/size/burst/WLAST rejection, half-select packing,
narrow-store forwarding):

- `eight_pipelined_reads_speedup` — 8 back-to-back reads against a fixed-
  latency pipelined MIG model, measuring completion cycles against a
  naive-serial (8× isolated round trip) baseline and printing the speedup
  factor.
- `interleaved_read_write_streams` — mixed writes/reads over a small
  revisited address pool, scoreboarded.
- `write_read_hazard_returns_new_data` — explicit same-line write-then-
  read, confirming the RAW hazard check holds the read back and it
  observes the fresh value.
- `rlast_beat_count_integrity_len{0,3,255}` — `RLAST` lands on exactly the
  last beat and beat count matches the requested `ARLEN`, at the AXI4
  burst-length extremes.
- `pipelined_writes_multi_outstanding` — 4 (`WMAX_OUTSTANDING`) bare AW
  commands queued while `m_awready` is low and no W data has been sent
  yet, proving the command queue is genuinely 4 deep and decoupled from
  data streaming; a 5th AW is confirmed refused; each burst's full data
  is then streamed in AXI4 order and all 4 drain correctly.
- `malformed_burst_completes_with_padding` — a legitimate pushed pair
  followed by an early-`WLAST` beat still yields exactly the promised MIG
  beat count (real pair + `wstrb=0` padding), a correct local `SLVERR`,
  and no wedge for a following unrelated write.
- `backpressure_sweep_stall{0,15,40,70}` — mixed read/write sequence under
  0/15/40/70% MIG-side random ready/valid stall.
- `random_scoreboard_10k_mixed_ops` — ~10k randomized single-beat
  read/write ops, genuinely concurrent (up to 4 writes / 6 reads truly
  outstanding at once, not spun to completion one at a time), under 20%
  random backpressure against a byte-addressed golden model.

`rtl/board/sim_mig_backend.v` additionally supports `STALL_ENABLE`/
`STALL_SEED` parameters (default off, zero behavioural change) for command-
side random backpressure at the `tb-fpga-top-rom-mig` integration level.

## Remaining integration decisions

- Clock crossing: the pcie_test MIG AXI runs on `c0_ddr4_ui_clk`
  (333.25 MHz). The real-hardware path crosses through `axi_async_bridge`
  before this shim (see `rtl/board/ddr_ctrl.v`); the bridge itself lives
  entirely in the MIG UI clock domain and has no CDC logic of its own.
- Timing constraints: keep every core-to-MIG UI clock path visible until
  it is proven to pass only through `axi_async_bridge`/`async_fifo`
  synchronizers or vendor CDC IP. Direct paths are first-hardware
  blockers, not false-path candidates. The multi-outstanding rework adds
  descriptor-queue arrays and two small (2-deep) skid buffers, all single-
  clock-domain (MIG UI clock) — no new CDC surface.
- Address capacity: the known-good MIG is 31-bit addressed. Any future map
  that needs bit 31 set must be flattened differently before reaching this
  bridge.
- Reordering assumption: the whole read-side pipelining design leans on
  read data returning strictly in AR-acceptance order (see above). Since
  the bridge always drives a single `m_arid=0`, this is the plain AXI4
  same-ID rule, not a pcie_test-MIG-specific behaviour — any AXI4-
  compliant device is required to honour it. Re-verify only if a future
  target ever presents multiple IDs to this bridge (it currently never
  does) or is not itself AXI4-compliant.
