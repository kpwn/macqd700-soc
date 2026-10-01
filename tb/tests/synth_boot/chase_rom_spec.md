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

The ROM testbench's `+pc_dump_path` provides true retire order from both ROB
lanes. Its `sim_time` is two ticks per system clock, so parser cycles are
`(end_time-start_time)/2`. Raw trace and JSON metadata are kept with each
binary. This is a dependent-load latency test, not an instruction-count or
cache-miss-count proxy. No runtime change to the full-top testbench is made here.

The checked-in generated files are `chase_generated/chase_{2,64}k.{s,elf,bin,json}`.
Regenerate with `python3 chase_rom.py generate --size 2048 --out-dir
chase_generated` (and `--size 65536`). Run the full-top ROM test with the
corresponding `.bin`, `+pc_dump_path=<trace>`, and `+pc_dump_max=30000`; the
default 10000-entry cap can truncate the 64 KiB checksum and pass PC. Parse
with `python3 chase_rom.py parse --meta <json> --rom <bin> --trace <trace>`.
The parser rejects truncated/failed runs instead of reporting a partial CPI.

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
AR/R-transfer and DDR_AR counts. No bus payload is checked; the ROM's
post-window checksum and pass PC validate data. `--acceptance warmL2` requires
the bus trace, the 64 KiB image, at least one warm and measured hot AR, and no
measured DDR_AR (the warm lap has populated L2). This acceptance is specific
to the P1/P3 CPU path exercised by this ROM and does not establish other
request classes.

For that gate, append `--bus-trace <bus_trace> --acceptance warmL2` to the
parser command above. The bus writer records asserted RVALID every cycle,
including a held beat, so `hot_r_valid_samples` can exceed
`hot_r_transfers`; only transfers advance the per-ID burst count.
