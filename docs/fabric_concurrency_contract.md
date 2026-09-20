# Fabric concurrency contract

These are interface requirements and performance targets, not a claim that
every current path meets every target. The clause numbers are retained for
references in RTL and tests. Build-specific measurements belong with their
test or release results, not in this contract.

The CPU socket is defined by [cpu_socket.vh](../rtl/soc/cpu_socket.vh).
Integration is described in [SoC architecture](architecture.md) and
[L2 system cache](l2c_spec.md).

| Clause | Requirement |
|---|---|
| C1 | A read master capable of N outstanding misses must expose all N concurrently to the fabric; the target is N ≥ 4. |
| C2 | Concurrent reads from one master use distinct ARIDs. An ID is not reused while that master's prior read remains in flight. |
| C3 | The crossbar target is at least four outstanding reads per master port. |
| C4 | Each L2 header input stages the next header while the current burst decomposes. |
| C5 | Same-ID gating blocks the actual ordering hazard, not unrelated work sharing a numeric ID. |
| C6 | The line-fill throughput target is the L2's internal quadrant-processing floor, without additional admission bubbles. |
| C7 | The DDR path below L2 supports at least eight outstanding reads and four outstanding writes. |
| C8 | The VRAM lane mux must not reduce bulk-fill concurrency below the crossbar's per-master budget. |
| C9 | Preserve write-side streaming throughput; this contract does not require new write concurrency. |
| C10 | A write master may reuse an AWID. B responses remain ordered per ID; masters must not depend on out-of-order B. |
| C11 | AXI READY outputs must not depend combinationally on their own acceptance/hazard cone. Add header staging to buy depth. |
| C12 | Socket masters use 4-bit IDs. The crossbar adds a 2-bit slot tag, producing 6-bit IDs at its downstream interface. |
| C13 | CPU table-walk descriptor reads and U/M updates go through `DcacheService`, not a private AXI port that bypasses L1D. |

## Ordering and ownership

Reads may complete across different IDs out of order where the interface
permits it. Track their owners through buffering, width conversion and CDC;
never infer ownership from which requester happens to be presenting now.

AXI4 has no WID. The write path must retain an owner for each W burst and
must not interleave W beats from different owners. Raising read concurrency
does not authorize changing this write policy. Hold payloads stable whenever
VALID is asserted and READY is low.

Native fetch and crossbar traffic use separate ID namespaces at L2. Hazard
and response routing checks must carry that source distinction.

## Coherency and reset

L2 does not snoop CPU L1 caches. A DMA/host coherency scheme needs explicit
cache-maintenance or mapping policy; increasing queue depth cannot provide
coherency. Likewise, CPU table-walk visibility is a CPU cache-service
requirement, not something an AXI merge alone can fix.

Reset must account for accepted transactions through every bridge. Check
in-flight responses, CDC queues, generation/ownership state and peripheral
operations before allowing new requests to reuse old identifiers.

## Validation

Exercise multiple IDs under backpressure, same-line hazards, full queues,
and reset with traffic outstanding. Measure end-to-end throughput as well
as each interface's acceptance capacity. Do not infer full-system IPC or
board timing closure from an isolated bridge benchmark.
