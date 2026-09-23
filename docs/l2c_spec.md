# L2 system cache

The SoC L2 is implemented by [l2c.v](../rtl/soc/l2c.v) and connected in
[fpga_top_ddr.vh](../rtl/soc/fpga_top_ddr.vh) when `L2C_ENABLE` is set.
It is separate from the CPU's L1 instruction and data caches.

## Organization and interfaces

- 2 MiB capacity: 4,096 sets, eight ways, 64-byte lines.
- URAM-backed line storage; four 128-bit quadrants per line.
- A 128-bit AXI read/write port for crossbar S0, with 6-bit IDs.
- A dedicated 256-bit read-only instruction-fetch port, with 4-bit IDs.
- A shared 128-bit downstream AXI interface to the DDR path.
- Eight miss-status entries; default victim-buffer depth eight and
  per-miss secondary replay depth four.

`l2c_ctrl.v` owns lookup, admission and response sequencing. `l2c_mshr.v`
tracks refills and merged operations; `l2c_victim.v` holds dirty evictions;
`l2c_bypass.v` handles configured nonallocating windows. Fetch responses
reassemble internal quadrants into the CPU's native 256-bit beats.

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
VRAM/DDR chain tests. Unit tests and integrated CDC/MIG tests cover different
boundaries; passing one does not establish the other. Test queued traffic,
same-line conflicts, partial writes, backpressure and reset with operations
in flight. Current suite limitations are recorded in the main README.
