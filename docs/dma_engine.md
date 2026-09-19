# Shared DMA engine contract

`rtl/soc/dma_engine.sv` is a peripheral-neutral, bidirectional AXI4 master.
It arbitrates a parameterized set of packed request/response lanes onto one
memory-system attachment point.

## Client request

Each client owns one slice of the packed ports. A request transfers when both
`req_valid[client]` and `req_ready[client]` are high.

| field | meaning |
|---|---|
| `req_addr` | byte address of the first byte |
| `req_len` | 1..64 bytes |
| `req_write` | 0 = memory read, 1 = memory write |
| `req_wdata` | write bytes, with byte 0 in bits `[7:0]` |
| `req_tag` | client-defined value returned unchanged on completion |

The arbiter accepts at most one aggregate request per cycle and uses round-robin
selection when multiple clients assert `req_valid`. Separate read and write
queues let ingress continue while AXI responses are outstanding. A full queue
can accept a replacement request in the same cycle its head retires.

## Completion

`rsp_valid[client] && rsp_ready[client]` retires a completion. `rsp_tag`,
`rsp_len`, and `rsp_write` identify the original request. `rsp_status` is the
AXI `RRESP`/`BRESP`; read data uses the same byte-zero-at-`[7:0]` convention.
Bytes above `rsp_len` are zero. The engine has one shared response holding slot;
the payload and metadata are broadcast as wires and only the selected client
gets `rsp_valid`. A client that stalls its response therefore backpressures
later completions. This avoids replicating a 512-bit holding register per
client while preserving full throughput for clients that keep `rsp_ready`
asserted.

## AXI behavior

- `DATA_WIDTH` supports 128, 256, or 512 bits.
- Requests are byte granular and may be unaligned or cross a bus-beat boundary.
  Bus-aligned requests use full-width AXI beats and exact final-beat strobes.
  Unaligned requests use byte-sized AXI INCR bursts, avoiding a wide barrel
  shifter while retaining exact byte semantics.
- An aligned 64-byte request maps to one L2C line and uses 4, 2, or 1 AXI beats
  at 128, 256, or 512 bits respectively.
- Eight IDs are shared by reads and writes and are not reused until the matching
  final `R` beat or `B` response retires.
- Read addresses can issue every cycle while IDs are available. Write addresses
  issue ahead into an ordered write-data schedule; write data never interleaves
  across bursts because AXI4 has no `WID`.

The client contract currently has no per-request cache-allocation hint. In the
normal SoC address range, read misses allocate in L2C and partial write misses
may fill before allocating. An aligned full 64-byte write uses L2C's
allocate-without-fetch path, but still leaves the new dirty line resident.
L2C's separate never-allocate engine is selected by configured address windows,
not by this DMA master, and its only normal window is disabled when the legacy
DDR RAM disk is disabled. A future general no-allocate request bit would require
a coherent L2 hit/miss path with victim and concurrent-fill hazards; merely
adding AXI `AxCACHE` outputs to this module would not be sufficient.

The normal SoC instance uses a 128-bit datapath and occupies xbar M3, which
reaches DDR through xbar S0 and L2C. The optional legacy
`ENABLE_DDR_RAMDISK` build still assigns M3 directly to `vhdd_ddr`; migrating
that provider behind a DMA client adapter is deferred. Wider configurations are
intended for targets whose cache or fabric front door is correspondingly wider.
Client lane 0 is currently assigned to the SONIC packet adapter when the real
Ethernet endpoint is selected; lane 1 is its receive adapter. No SONIC or
Ethernet descriptor semantics exist inside `dma_engine`; other peripheral
clients use the identical lane contract.

The four-client, 128-bit standalone synthesis is 2,765 LUTs (2,129 logic plus
636 distributed RAM), 673 flip-flops, seven RAMB36 and one RAMB18 in Vivado
2025.2. This is the resource ceiling configuration used to guard the generic
engine; constant inactive lanes in a particular SoC may optimize lower.

## Verification

- `make lint-dma-engine`
- `make tb-dma-engine` — 128/256/512 width sweep, mixed clients, short and
  unaligned transfers, line fast path, ID lifetime, and ingress throughput.
- `make tb-dma-l2c` — the same contract through the real L2C, with measured
  cycles per 128-bit word.
- `make tb-dma-engine-mut` — proves the test rejects a drop-last-byte mutant.
