# CPU040 hot-read door integration validation (2026-10-01)

## Actual CPU plus L2 dependent chase (2026-10-02)

The corrected paired full-top ROM run uses frozen SoC `446abae` and identical
generated CPU inputs from `63171ff62b253d10cba982b60aca36814bf95225`. Both
Verilator builds and all four ROM runs passed. Only
`L2_DH_R_FALL_THROUGH` changes between arms. CPU options include NB4/ring8,
hot read door, D4 ordering, P1 early loads (An and Dn), P3/probe forwarding,
probe-miss staging, early response, eager/preselected AR, dynamic release,
speculative wakeup and LS-OoO; direct refill is OFF. Exact generation flags
and file hashes are in the input manifest below.

The dedicated `+macro_pc_dump_path` samples actual retiring instruction PCs
before each edge, including both retirement lanes. The parser requires its
versioned header. This corrects the earlier `e3896db` experiment's use of
legacy `traceVec.pc` resume/next PCs. Those earlier marker-window measurements
are superseded; their independently checked 16/15-cycle AXI request spacing
was correct. The legacy raw artifacts remain at
`/tmp/codex-soc-chase-e3896db/` for audit.

| Hot R fall-through | L1 marker cycles / 128 hops | L2 marker cycles / 4096 hops | Consecutive L1 / L2 load-retirement spacing | Accepted hot AR to first RVALID / transfer |
| --- | ---: | ---: | --- | --- |
| OFF | 646 | 65541 | 5 / 16 cycles | 6 / 6 cycles for all 4096 requests |
| ON | 646 | 61445 | 5 / 15 cycles | 5 / 5 cycles for all 4096 requests |

Marker-window cycles/hop are 5.046875 for L1, 16.001221 for L2 OFF, and
15.001221 for L2 ON. They include fixed marker overhead. The separate
retirement-spacing histograms contain 127 L1 intervals and 4095 L2 intervals
per arm, all exactly the values in the table. In both L2 arms, returned data
to that load's retirement is exactly four cycles (4096 samples), and returned
data to the next accepted hot AR is exactly ten cycles (4095 samples).
These are actual observed endpoints, not inferred issue timestamps.

Both measured L1 windows have zero hot reads and zero DDR reads. Both
measured L2 windows have exactly 4096 hot reads/transfers and zero DDR reads.
Every run reaches its post-check pass PC. The parser checks each measured L2
request address, in order, against all four laps of the generated Sattolo
ring, rejecting short repeated cycles, reordered accesses, missing requests,
and wrong read shape. Both L2 address sequences have SHA-256
`79510e223956378f424e61e694b9e5cdc8b0f9eeab84c894e33140b65516ade1`.
Host validation passes 4/4 tests, including legacy-trace rejection and
corrupted-sequence controls.

The end-to-end saving is exactly 4096 cycles for 4096 hops. Steady L1 stays
at five cycles; the ON L2-over-L1 difference is **ten cycles**, not the
requested six. This is actual CPU/guard/L2 RTL simulation with behavioral
downstream memory, not board timing or a whole-program IPC result. It proves
the response bypass's cycle saving, not mapped area or routed timing. The
option remains OFF by default. The synthetic reset ROM does not establish
an OS boot.

Artifacts under `/tmp/codex-soc-chase-446abae/`:

- `input-manifest.json`: SoC/CPU provenance and frozen input hashes, all
  rechecked after both builds and simulations.
- `results.json`: commands, exit codes and original corrected-parser results.
- `retirement-validation-results.json`: parser `92a9abd` revision/hash,
  input-manifest and raw-trace hashes, all four rechecked measurements, and
  per-load retirement/AXI interval histograms.
- `ft{0,1}-{2,64}k.{pc,bus,log}`: true macro retirement, AXI and simulator data.

The sections below retain earlier unit/integration experiments and their
original narrower scopes.

Scope: SoC branch `codex/bandwidth-soc` after `3ad9018`, generated CPU from
`m68k-core-040-ooo` commit `5577d139`. The CPU was generated in isolated
agent-53 with `CPU_DCACHE_NONBLOCKING=1`, `CPU_INHIBITED_FULL_BARRIER=1`,
`CPU_AXI_DH=1`. The SoC's `CPU_AXI_DH` knob enables the **read** hot door
(`L2C_DH_PORT`); neither the CPU socket nor this knob enables axi_dh AW/W/B or
`L2C_DW_PORT`. The L2 hot-write unit tests are not CPU/SoC write integration.

## Build and functional evidence

| Check | Result | What it establishes |
| --- | --- | --- |
| Generated CPU OFF and ON, `make lint-fpga-top CPU=m68k040` with matching `CPU_AXI_DH` | Both passed full `fpga_top` Verilator lint (194 modules); ON repeated after explicit NB/D4 Make guards | Actual generated socket elaborates with the SoC guard and L2 port in both configurations. ON top-level CPU has all 13 expected AR/R pins, 2-bit IDs and 128-bit R data. |
| `check-cpu-dh` with OFF netlist/ON SoC and ON netlist/OFF SoC | Both rejected | Both mismatch directions fail before lint. The same check is now an order-only prerequisite of the CPU-bearing ROM simulation builds and a prerequisite of the other full-top lint configurations. |
| Invalid ON build configurations | Rejected at Make parse time | Non-CPU040 socket, missing non-blocking L1D, and missing D4 inhibited-access barrier each fail explicitly. Direct `vivado.tcl` has the same guards; Vivado was not run. |
| `make tb-l2c-dh` | 78 pass, 0 fail | The test wrapper routes hot AR/R through `ifetch_window_guard` into L2. Directed checks include window refusal, dirty-victim interlock, 192 hot accepts versus 96 cold writes with 457 set-hazard refusals, and independent source-qualified IDs. Randomized scoreboard reached eight occupied L2 MSHRs in aggregate. |
| CPU `InhibitedFullBarrierSpec` ON cases A and C | 1 pass each, 0 fail | Passive monitor saw 17 inhibited writes with walker traffic in A and 32 inhibited reads with 370 younger-held cycles in C; both reported zero bus/command before/after violations. These are focused CPU checks, not an end-to-end SoC boot. |

The routing is visible at [the CPU bind](../rtl/soc/fpga_top_debug_ctrl.vh),
[hot read guard and L2 bind](../rtl/soc/fpga_top_ddr.vh), and
[the corresponding unit-test composition](../tb/tb_l2c.v). In-window guard
traffic passes through without a register stage; out-of-window traffic gets a
local DECERR. The ON `fpga_top` lint elaborates the actual CPU/guard/L2
instances, while `check-cpu-dh` catches absent generated CPU ports because
general lint suppresses `PINMISSING`. CPU D4 is the required ordering fence between
cacheable hot reads and inhibited cold accesses. L2 itself does not provide a
cross-door architectural barrier.

## Read concurrency and its ceiling

The CPU has four non-blocking L1D MSHRs and uses four distinct native 2-bit
hot AR IDs (`ShippingCoreConfig.dcacheMshrs`, `DcachePlugin` AR issue at CPU
commit `5577d139`). The SoC removes the crossbar's one-outstanding cold-data
hop. Its hot header door has an active slot and one pending header
([`l2c_ctrl.v`](../rtl/soc/l2c_ctrl.v)); accepted beats share one rigid
three-stage lookup pipeline with fetch and cold traffic. The shared
[`l2c_mshr`](../rtl/soc/l2c_mshr.v) has eight entries, can issue fill ARs on
consecutive clocks, and tracks each by its MSHR-index AXI ID. The
[`l2c.v` read-route queue](../rtl/soc/l2c.v) is 16 entries at default
`BYPASS_SLOTS=8` and its registered AR slice does not wait for RLAST before
accepting another request. The real DDR bridge caps accepted read descriptors
at eight ([`axi_ddr4_mig_bridge.v`](../rtl/board/axi_ddr4_mig_bridge.v)).
Thus four CPU refills can coexist downstream, subject to the **shared** eight
L2 MSHRs and eight DDR descriptors; fetch/cold misses and same-line merge or
same-ID ordering can reduce free capacity. One lookup pipeline, one DDR port,
and ordered downstream response bursts still determine service rate. Four
L1D MSHRs do not mean four independent DRAM data channels. The unit
scoreboard's occupancy of eight is aggregate requester traffic, not a
measurement of four CPU misses on a board.

## Raw ring storage comparison

Generated `M68kSocketTop.v` twice from the same CPU commit with
`LS_OOO_ISSUE=1`, NB=1, D4=1, hot=1, four D-cache MSHRs. Only
`CPU_LS_LOAD_RING_DEPTH` changed, 4 to 8. Summing the widths of named
`LsEuPlugin` register declarations for the aligned descriptor, response,
FIFO and inhibited park arrays (excluding simulation-only `_string` fields):

| State group | Ring 4 | Ring 8 | Delta |
| --- | ---: | ---: | ---: |
| Aligned descriptors, response data/line/flags, FIFO tags and pointers | 1,129 bits | 2,247 bits | +1,118 bits |
| Inhibited `parkMem`, `parkValid`, `parkSaw` | 440 bits | 880 bits | +440 bits |
| **Selected raw register widths** | **1,569 bits** | **3,127 bits** | **+1,558 bits** |

Each added aligned slot carries a 108-bit descriptor, 32-bit result, 128-bit
line and six one-bit flags (274 bits); four slots account for 1,096 of the
1,118-bit aligned delta. The remaining 22 bits are FIFO/pointer growth.
Each added park slot adds a 108-bit descriptor and two flags. These are
pre-synthesis widths for selected named registers, not a LUT, FF or timing
estimate: RID widening, mux/CAM/control logic, register pruning and physical
implementation are excluded.

## Hot-hit latency question

The existing `tb-l2c-dh` parity row measures **10.0 cycles per completed
64-byte warm line**, hot and cold alike, with one burst outstanding. That
includes four 16-byte beats and testbench issue/retire bookkeeping; it is
not the latency from hot AR acceptance to first R. The guard passes an
in-window AR/R combinationally ([`ifetch_window_guard.v`](../rtl/soc/ifetch_window_guard.v)).
Inside L2, the AR is captured into the hot active/pending header, then a beat
passes the shared A→Q→S2 lookup registers; a hit registers `hit_rsp_*`, and
the hot R path passes a two-entry **registered-output** skid
([`l2c_ctrl.v`](../rtl/soc/l2c_ctrl.v), [`l2c.v`](../rtl/soc/l2c.v)).
Arbitration, same-set hazards and response backpressure add variable waits.
The new `dh_first_beat_latency` check timestamps accepted hot AR, first
RVALID, and first R transfer with RREADY held high. It warms 32 disjoint
lines, checks every returned byte, and rejects downstream DDR reads during
measurement. The results from `make tb-l2c-dh` (79 pass, 0 fail) are:

| Warm hit | Isolated AR→first RVALID / transfer | Mixed cold-write AR→first RVALID / transfer | Cold write accepted before first RVALID |
| --- | --- | --- | --- |
| 1 beat | 6 / 6 / 6 cycles (min/median/max) | 6 / 6 / 6 cycles | 32 / 32 samples |
| 4 beats | 6 / 6 / 6 cycles | 6 / 6 / 6 cycles | 0 / 32 samples |

The four-beat mixed arm still checks that all 32 cold writes enter the shared
front door and finish with OKAY. Hot-burst priority defers their acceptance
until after first RVALID, so that arm does not establish within-window write
contention. The one-beat arm does. The measured six cycles encompass hot
header capture, shared S0→A→Q→S2 lookup, registered hit response and
registered-output R skid. They start at **accepted hot AR**, excluding CPU
miss discovery, AR launch, refill handling, and dependent-instruction wakeup.
Thus they do not prove the requested L2-over-L1 ≤6-cycle load-to-use goal;
the 10.0-cycle parity row is completed-line throughput, a different endpoint.

A concrete next latency experiment is a fall-through variant of the **hot R
skid only**: when empty, forward the already-registered `hit_rsp_*` beat to
`dh_axi_r*` in the same cycle, retaining both skid entries for a refused
beat and keeping the upstream `s_ready` independent of `dh_axi_rready`.
That could remove one output register cycle on an uncontended hit. It risks
an extra combinational route to the CPU, AXI payload instability or duplicate
beats during an empty-to-stalled transition, and a timing regression. The
first-beat timestamp experiment above supplies the baseline for this proposal;
the remaining CPU-side stages and registered timing boundaries need measurement.
The proposal is falsified if the output skid is not a material first-beat
component, or if backpressure tests or paired post-route timing regress.
It needs an architecture-spec update and focused ready/valid, reset and
same-ID tests before implementation.

No Vivado, board, or full CPU+SoC boot was run in this validation. The hot
write half remains a separate CPU socket and routing integration task.

## Empty hot-R bypass experiment (2026-10-01, default OFF)

Isolated branch `codex/hot-return` selects the experiment in spec commit
`62e754b` and implements it in `40eacb5`. No board top enables the new
`DH_R_FALL_THROUGH` parameter. Paired `make tb-l2c-dh` runs with
`L2C_DH_FALL_THROUGH=0/1` each pass 79 tests, zero failures. For all four
first-beat rows above (32 samples each), accepted AR to first RVALID and
first transfer changes from exactly **6 to 5 cycles**. Checked bytes, no-DDR
warm-hit checks, same-ID ordering and AXI payload stability pass. The mixed
one-beat row still observes a cold write accepted inside all 32 measured
windows; the four-beat row still does not, so it is not evidence of
within-window cold-write contention.

The standalone FIFO scoreboard passes in both modes over more than 20,000
cycles. It verifies data order, every refused payload, full-buffer behavior,
reset discard count, one-beat-per-cycle unstalled admission, and upstream
ready independence from downstream ready. ON exercised 2,832 empty bypass
transfers and 4,103 full cycles. A single-mechanism mutant that discards an
empty bypassed beat when downstream refuses it compiled successfully and
failed at cycle 4 with `refused response changed payload`.

Exact commands, source hash and result codes:
`/tmp/codex-l2-hot-return-results.json`; paired logs:
`/tmp/codex-l2-hot-return-{fifo-off,fifo-on,l2-off,l2-on}.log`.
Mutant source, compile/run logs and result:
`/tmp/codex-l2-hot-return-mutant/`. The implementation adds no payload or
valid registers, but adds an output mux/control cone; raw state equality
is not mapped area or Fmax evidence. End-to-end CPU load-use and routed
SoC timing remain unmeasured, and board defaults remain unchanged.

Separately, SoC commit `6086cf4` corrects the VIO v28 simulation stub's
missing 32-bit `probe_in15`. Its clean snapshot with CPU `46413280`'s actual
dynamic-release ON socket passes basic and shipping full-top lint; see
`/tmp/codex-soc-hot-integration-6086cf4/{manifest,results}.json`. That lint
predates the response-bypass RTL and must not be cited as its integration
gate. No live board or owner worktree was modified.

SoC `9419a10` now passes paired shipping full-top lint with
`L2_DH_R_FALL_THROUGH=0/1` and the same actual generated CPU `46413280`
artifact in both arms. VIO, JTAG AXI, disabled SD JTAG writer, hot read door,
NB and D4 selections are held identical. A separate negative configuration
with the bypass selected but no hot door fails specifically at the
`L2_DH_R_FALL_THROUGH_requires_L2C_DH_PORT` elaboration guard. Inputs,
commands and results: `/tmp/codex-soc-hot-integration-9419a10/`.
This closes the response-bypass binding check under the SoC's existing lint
warning suppressions, not end-to-end functional or timing acceptance. No
board default, synthesis setting or live board state changed.
