# CPU040 hot-read door integration validation (2026-10-01)

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
