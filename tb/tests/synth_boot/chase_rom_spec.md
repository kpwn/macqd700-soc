# CPU040 ROM pointer-chase measurement

Generate separate 2 KiB and 64 KiB ROM images, each linked at 0x40000000 with a
64-byte-stride Sattolo pointer ring starting at 0x40010000. The seed and
permutation match `BoardMembenchSimSpec.kChaseRing`. Reset enters supervisor
mode with interrupts masked. Software sets CACR=0x80008000, enables TC.E with
TC=0x8000, and installs cacheable transparent mappings: the bench's
ITT0=0x00FFC000/DTT0=0x00FFC020. The 0xFF transparent mask matches every
top-byte value, including the ROM's 0x40xxxxxx address; ITT1/DTT1 are disabled.
No page-table lookup is intended.

One subroutine contains a start marker, four `movea.l (%a0),%a0` load PCs, a
counted branch, and an end marker. It runs first for one lap (warm instruction
and data caches), then for four measured laps. The parser selects the **second**
start/end marker occurrence. It requires exactly `4*nodes` retired load PCs in
that window, with each of the four PCs seen `nodes` times, and checks both
window ordering and positive elapsed clock cycles. The image checks A0 returns
to the ring base after each invocation and checks an order-sensitive rotate/XOR
checksum of every stored pointer after the measured window. A pass PC is
reached only after these checks; any fail PC or missing pass PC rejects a trace.
The checksum catches a collapsed ring even if four laps happen to end at base.

The ROM testbench's legacy `+pc_dump_path` uses `RobPlugin.traceVec.pc`, which
is the resume/next PC rather than the retired instruction PC. It must never
define the chase markers. A dedicated `+macro_pc_dump_path` samples the existing
`debugMacroRetirePc_{0,1}_{valid,payload}` signals before the rising edge and
labels events with the corresponding post-edge `sim_time+2`, lane 0 before
lane 1. It writes `# cpu040-macro-retire-v1 ticks-per-cycle=2` followed by
`<ordinal> <tick> <instruction-pc>` rows. The parser requires that exact
header and rejects legacy traces even when their numeric rows happen to match
marker values. Its cycles are `(end_tick-start_tick)/2`. The new dump defaults
to a 30000-row cap, independently of the legacy dump, and is CPU040-only.
This is a dependent-load latency test, not an instruction-count proxy.

The checked-in generated files are `chase_generated/chase_{2,64}k.{s,elf,bin,json}`.
Regenerate with `python3 chase_rom.py generate --size 2048 --out-dir
chase_generated` (and `--size 65536`). Run the full-top ROM test with the
corresponding `.bin` and `+macro_pc_dump_path=<trace>`; the 30000-row cap covers
the 64 KiB checksum and pass PC. Parse
with `python3 chase_rom.py parse --meta <json> --rom <bin> --trace <trace>`.
The parser rejects truncated/failed runs instead of reporting a partial CPI.
It also reports the histogram of consecutive measured load-retirement
intervals, with exactly `measured_hops - 1` samples. This separates steady
retirement spacing from fixed start/end marker overhead; both metrics must
remain visible. Retirement spacing is not an issue timestamp or an IPC
measurement of an independent-load workload.

An optional `--bus-trace <path>` consumes accepted CPU-facing hot-door
`t AR id addr len size burst`, asserted `t RV id resp last ready`, and accepted
L2-master `t DDR_AR addr len` rows. Timestamps share the PC trace's two ticks
per system clock and denote the post-edge time for a pre-edge bus sample. A
same-timestamp event belongs to the interval ending at that marker, so window
membership is `start < t <= end`. The parser tracks outstanding bursts FIFO
per AXI ID, counts only ready R beats, tolerates repeated stalled RVALID,
requires a correct final RLAST and RESP=OKAY, and rejects orphan/incomplete
bursts. It reports AR-to-first-RVALID and AR-to-first-transfer cycle histograms
for requests accepted in the measured interval, plus warm and measured hot
AR/R-transfer and DDR_AR counts. No RDATA payload is logged. The ROM checks
the final pointer and stored table checksum; these alone do not check every
intermediate returned pointer. `--acceptance warmL2` therefore also requires
exactly one single-beat 16-byte INCR hot read per measured hop and checks the
complete ordered AR address sequence against the generated Sattolo ring.
Every returned pointer except the last determines the following request
address; the ROM's final-pointer comparison checks the last. The parser
reports a SHA-256 digest of that sequence instead of thousands of addresses.
A shortened/repeated cycle must fail even if its request count and final
pointer match. The gate also requires the bus trace, the 64 KiB image, at
least one warm hot AR, and no measured DDR_AR (the warm lap has populated
L2). This acceptance is specific
to the P1/P3 CPU path exercised by this ROM and does not establish other
request classes.

For that gate, append `--bus-trace <bus_trace> --acceptance warmL2` to the
parser command above. The bus writer records asserted RVALID every cycle,
including a held beat, so `hot_r_valid_samples` can exceed
`hot_r_transfers`; only transfers advance the per-ID burst count.

The previous full-SoC 2 KiB/64 KiB marker figures (5.0 and 15.9998
cycles/hop in the OFF snapshot) were derived from the legacy resume-PC dump
and remain **pending remeasurement** with the
dedicated macro-PC tap. The independently checked bus recurrence remains
valid: 4096 ordered Sattolo hot reads, zero measured DDR reads, and a 16-cycle
OFF / 15-cycle ON hot-AR recurrence in the 64 KiB case. Do not label that bus
recurrence as CPU issue-to-issue or retirement latency.

## Independent four-chain warm-L2 experiment (2026-10-02)

A separate `parallel_chase_rom.py` benchmark keeps the single-chain benchmark
unchanged. It partitions the same 64 KiB, 64-byte-stride table into four
16 KiB Sattolo cycles, addressed by A0..A3. Each loop issues one simple
`MOVEA.L (An),An` per chain, with D7 controlling the loop and A7 retained as
stack. Warmup traverses every chain once; measurement traverses each four
times (4096 total pointer loads). Both invocations check all four final
pointers; a whole-table checksum follows measurement. Translation/cache setup
and true macro-retirement markers match the existing chase ROM.

Acceptance uses the existing strict macro trace parser, ROM hash, and AXI
burst checker. Requests from different chains may interleave in any order;
within each chain, every accepted measured address must equal its next
expected pointer and advance that chain's Sattolo state. Require exactly
1024 reads per chain, 4096 total single-beat 16-byte hot reads/responses,
zero measured DDR reads, final closure of all chains, and guest PASS. No
fixed global request order is assumed. Reject unknown/out-of-range/misaligned
addresses, skipped/repeated chain steps, missing transactions, or wrong ROM.
Report aggregate cycles/bytes per cycle and maximum outstanding measured hot
reads; aggregate retirement spacing is not dependent load-to-use latency.
This ROM provides actual CPU/guard/L2 simulation evidence, not mapped area,
routed timing, board execution, or proof of general application IPC.


Phase-accounting refinement: the first guest passed, but two next-traversal
requests appeared before the warm-end retirement marker, and the measured
retirement window contained 4095 rather than 4096 reads. Retirement markers
are not issue fences, especially across a predicted loop boundary. Therefore
classify each chain's full accepted request prefix: 256 warm then 1024 measured
steps. Check every step against the generated cycle, match responses by AXI ID,
and require all four 1280-step prefixes and measured responses complete by the
measured retirement end. Count preissued measured requests explicitly. Do not
force marker-window traffic to equal architectural load count or call an
aggregate retirement interval dependent latency. Keep zero DDR reads in the
measured retirement window, report phase request-to-response latency and
marker-window traffic separately, and validate guest final pointers/checksum.
Requests after a chain's complete prefix are not attributed to its measured
phase. This is needed for speculation/cleanup; the full bus parser still
checks their AXI integrity. The raw first run remains preserved at
`/tmp/codex-soc-parallel-chase-57672e3` as a rejected measurement.
