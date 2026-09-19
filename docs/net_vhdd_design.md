# Ethernet-backed vHDD — design

Status: **RTL landed up to the seam; not yet instantiated.**  The decisions
below were settled in discussion and are now implemented; the open questions
at the end are still genuinely open.

Built and tested:

| piece | where | state |
|---|---|---|
| framing (Ethernet/IPv4/UDP + protocol) | `rtl/board/net_block_framer.sv` | done, `tb-net-block-framer` |
| MAC sharing with the SONIC | `q700_eth_stream_share` in `rtl/board/q700_eth_link.sv` | done, `tb-q700-eth-stream-share` |
| transaction layer (windowing, tags, retransmit, ping-pong) | `rtl/board/vhdd_net.sv` | done, `tb-vhdd-net` |
| endpoint CSR (JTAG-loadable MAC/IP/port) | `rtl/soc/vhdd_ctrl.v` 0x040-0x058, `vhdd-net` in the REPL | done, `tb-vhdd-ctrl` |
| host daemon | `tools/net_vhdd_server.py` | done, 28 tests |

**The mux needs no change, and there is a slot already waiting.**  An earlier
revision of this section claimed a third provider meant widening
`vhdd_mux`'s 1-bit `dev_sel` and the `vh_dev_sel` latch in `scsi.v`.  That was
wrong.  `vhdd_ddr`, the DDR-backed RAM disk that used to be provider B on SCSI
ID 1, was **compiled out on 2026-08-19** (`ENABLE_DDR_RAMDISK`, see the note at
`fpga_top_peripherals.vh:2154`).  Its slot is vacant, its `vhb_*` wires still
exist and are still wired to the mux, and `dev_en[1]` is forced low so
`dev_sel` cannot go high with nothing behind it.  So the net volume drops onto
the existing B face and answers on ID 1, with **no change to `scsi.v` and no
change to the mux** — removing the only step that could have regressed the SD
volume.

It is also a better tenant than the RAM disk was.  That note freed the slot for
two reasons: ~1.5k LUT on a design at ~86.8% utilisation, and — worth more —
xbar master seat M3, on a read fan-in that was 4/4 FULL and is being held for a
larger CPU core.  `vhdd_net` reclaims neither: its backing store is reached
over Ethernet through the stream share, not over AXI, so it needs no master
port and no L2C bypass window.

**Integration is done** (`ENABLE_NET_VHDD`, off by default).  `vhdd_net` and
`net_block_framer` sit on `pb_clk` beside `vhdd_sd`; `q700_eth_link` carries
the `blk_clk` ↔ `core_clk` frame-FIFO crossing; the endpoint CSR feeds them;
`dev_en[1]` is unmasked when the provider is built; and `lint-eth-link` gained
a `sonic+netvhdd` configuration, which is the only place this path can be
linted at all (`lint-fpga-top` has no taxi sources and cannot elaborate any
ethernet build).

**What remains is hardware bring-up, not RTL:**

1. Host-side `ip neigh replace` in the daemon's setup, per **No ARP in RTL**
   below.  Skipping it looks exactly like a broken receive path: requests
   visible in tcpdump, replies never arriving.
2. Build with `ENABLE_NET_VHDD`, then `vhdd-net` to configure the endpoint and
   `vhdd-enable` to bring ID 1 live.  Nothing happens until both: our-MAC
   resets to zero and an all-zero block MAC means "absent" to the demux.
3. Measure the cold start (see **Reads, writes, and the cold start**).  The
   disconnect/reselect-vs-stall question is explicitly marked "measure it; do
   not assume", and it is still unmeasured.

### Cold start: RESOLVED — stall, and do not build disconnect

The three options above (disconnect/reselect, readahead, stall) are settled in
favour of **stall**, and the reason is not "simplest, let's try it":

This bus serviced period SCSI disks.  A contemporary drive had a 15–30 ms seek
plus ~8 ms of average rotational latency at 3600 RPM, so the Mac's driver was
*written* to sit through tens of milliseconds for a single block.  A
sub-millisecond LAN round trip is an order of magnitude inside what it already
tolerates — and `vhdd_sd` has stalled the same driver the same way for every
CMD17 since day one.  The "measure it; do not assume" caveat is discharged by
the hardware the driver was designed against.

So **disconnect/reselect is not needed and is not being built.**  It remains
architecturally correct and `scsi.v` still has the machinery (DISCONNECT
phase, RESELECTED status bit, drives its own ID during reselection) if a
future backing store is slow enough to need it.

Two things that follow, and one earlier claim withdrawn:

- **The retransmit timeout is tuned for packet loss, not for driver patience.**
  Those are different questions and only the second was ever in doubt.  The
  module default is 40 ms at `pb_clk`, ~200× a LAN round trip — correct, but it
  turns one dropped frame into a 40 ms stall.  The instantiation overrides it
  to 4 ms (still ~20× RTT, so it cannot fire on a merely slow reply), making
  the full four-attempt budget ~16 ms worst case: one legacy seek.
- **Withdrawn:** an earlier revision claimed disconnect was also needed to stop
  the slow network volume blocking the fast SD one.  That is real at the bus
  level but almost certainly invisible above it — classic Mac OS's SCSI
  Manager is largely synchronous, so the driver is not going to touch ID 0
  while it is blocked on ID 1 regardless of who holds the bus.  Freeing the
  bus buys nothing a synchronous initiator can spend.

## What this is

A third virtual-HDD provider whose backing store is a host over Ethernet,
serving reads and writes, sitting behind the existing `rtl/vhdd.vh`
contract alongside `vhdd_sd` and `vhdd_ddr`.

`vhdd_mux` already routes one SCSI master to two target IDs backed by
different providers, selected by `vh_dev_sel` latched at selection.  So
this is a new provider, **not** a new SCSI path — `scsi.v` and the mux
are untouched.

## The constraint everything follows from

Q700 SCSI is CPU-driven pseudo-DMA.  A network round trip *inside* a data
phase presents as a hang, not a slow disk.  So:

> the network transaction must complete before the data phase opens.

## Buffer: BRAM, ping-pong, small

Sizing is set by **refill latency × drain rate**, not by transfer size:

    ~1 ms round trip × ~2 MB/s pseudo-DMA drain  ≈  2 KB  ≈  4 sectors

So ~4 sectors per side, 8-16 total (4-8 KB) — one BRAM tile.  The network
refills roughly 100x faster than the Mac drains, so in steady state the
double buffer never runs dry.

BRAM not DDR, deliberately: this is a *staging buffer*, not a cache.  It
does not hold a working set.  Going through DDR would pull in the DMA
engine, AXI, the 32-bit byte-lane convention, crossbar contention and L2C
coherency — the entire bug surface that cost us a day.  Precedent exists:
the SD provisioning path already streams into a 128 KiB staging BRAM, and
the SONIC engines infer BRAM for their packet stores.

DDR would only earn its place for genuine caching (resident working set)
or large write-back batching.  Neither is v1, and both are additive since
the provider hides behind `rtl/vhdd.vh`.

**Transfers stream; they are not buffered whole.**  The protocol ceiling
is 256 blocks for READ(6) (length 0) and 65535 for READ(10) — up to 32 MB.
The buffer is a sliding window that refills behind the drain.

## Transport: UDP

Raw EtherType would save 28 bytes/frame (IP 20 + UDP 8).  On a 512-byte
sector that is ~5% of wire bytes on a link already ~100x faster than the
consumer — irrelevant.

UDP buys a host daemon that is an ordinary socket: no raw socket, no
CAP_NET_RAW, no BPF, no root, and it routes.  Worth 28 bytes.

RTL cost is one IP header checksum (16-bit one's complement over 20 bytes,
nearly all fields constant, so mostly precomputable).  **The UDP checksum
is optional in IPv4 — transmit zero and skip it.**

Framing: request `{magic, op, lba:32, count, tag}`, reply
`{magic, tag, status, payload}`.  Keep frames under the MTU; do not
fragment.  Magic is `0x4E424844` ("NBHD"); `op` and `status` are 8-bit,
`0x00` = read / success, `0x01` = write.

**Every frame carries at most 2 blocks, in BOTH directions.**  1500-byte IPv4 MTU minus the
IPv4, UDP and protocol headers leaves room for 1024 bytes of payload; three
blocks would overflow it.  A read *request* carries no payload, so a large
`req_block_count` would parse — but the REPLY could not fit, so the host
daemon refuses those too (`STATUS_TOO_MANY_BLOCKS`).  The transaction layer
must therefore never issue a request of more than 2 blocks in either
direction.  That is a constraint on the windowing state machine, not a
limit on what SCSI may ask for: `req_block_count` is 16 bits and READ(10)
may legitimately ask for 65535.

## No ARP in RTL

The block service has its own **locally-administered** MAC (bit 1 of the
first octet set, so it cannot collide with a real OUI) and its own IP.
The destination MAC is hardcoded and JTAG-loadable via a small CSR block,
like the `vhdd` CTRL bits.

That destination is the **next hop**, which is exactly what ARP would have
resolved — point it at the gateway and UDP still routes off-segment.

We are always the initiator, so no inbound ARP is needed for us to send.
**But the host kernel still resolves our IP to build its replies.**  With
no ARP responder on our side it will query, get silence, and drop every
reply — requests visible in tcpdump, replies never arriving, looking
exactly like a broken receive path.

Fix, chosen: a static neighbour entry on the host, as part of the daemon's
setup —

    ip neigh replace <fpga-ip> lladdr <fpga-mac> dev <iface> nud permanent

If replies ever stop after a host reboot or NIC change, a stale or missing
neighbour entry is the first thing to check.

Upgrade path if the board must work on a host we do not administer: a
minimal ARP *responder* (answer requests for our IP, never send any).
`icmp_echo_responder` already implements this and is hardware-proven
(200/200 ICMP, 0% loss), so it is a copy, not new logic.  The *requesting*
side — where most ARP complexity lives — stays absent either way.

## Sharing the MAC with the SONIC

Note `ICMP_RESPONDER` is a compile-time **alternative** to the SONIC path
(`generate if/else`), not a concurrent client — there is no existing
arbiter to reuse.

- **RX demux** on destination MAC: exact match on the block MAC → block
  path, everything else → SONIC.
- **Broadcast** is copied to both.  The SONIC applies its own filter
  anyway, and the block engine only acts on traffic for its own IP.
- **TX arbitration must be frame-granular.**  Two AXI-stream frames must
  never interleave on the shared MAC: grant for a whole frame, `tlast` to
  `tlast`, then re-arbitrate.  Getting this wrong corrupts both directions
  in a way that reads as random packet loss.

~~Caveat: with the SONIC in promiscuous mode it sees block-service
frames too.~~  **Wrong, now that the demux exists.**  A frame matching
`blk_mac` goes only to the block client, so a promiscuous SONIC does not
see it and `rx_frames` is not inflated.

Two things this section originally failed to say, both found while
implementing it:

- **The arbiter owes completion ATTRIBUTION, not just non-interleaving.**
  The MAC emits one completion per transmitted frame and the SONIC path
  turns that into a `TCR_PTX` descriptor writeback.  With two clients, a
  block frame's completion would falsely complete a SONIC descriptor.  A
  per-frame owner bit travels with the data through the CDC so completions
  are routed back to whoever sent the frame.
- **The absent client is a stall hazard.**  Before the block engine exists
  its `tready` is strapped, so fanning a broadcast to it would hold the
  frame forever and starve the SONIC of the entire wire.  An all-zero
  `blk_mac` therefore means "absent" and is excluded from both exact match
  and broadcast fan-out.

Broadcast backpressure is lock-step fan-out with independent per-consumer
acceptance: one elastic beat carries a pending bit per consumer, whichever
accepts first clears its bit while the byte stays valid for the other.  No
byte is duplicated or dropped, and a genuinely slow consumer may
backpressure the wire — the MAC's frame FIFO absorbs a bounded stall.

## Reads, writes, and the cold start

- **Read hit** — serve from the buffer at full rate.
- **Read miss** — fetch *before* entering DATA IN.
- **Write** — accept into the buffer, complete the SCSI command, flush
  asynchronously (write-back).  Honour SYNCHRONIZE CACHE (0x35) as a real
  barrier, and flush on idle.

Buffer size does **not** help the **cold start**: the first run of a
transfer still needs a full round trip before the data phase can open.
Three ways to cover it, and they are orthogonal to capacity:

1. **Disconnect/reselect** — architecturally correct.  `scsi.v` already has
   a DISCONNECT phase, a RESELECTED status bit, and drives its own ID
   during reselection.  Depends on the driver permitting disconnect via
   IDENTIFY — **check a live capture before designing around it.**
2. **Readahead** — converts most misses to hits; shrinks the problem
   rather than removing it.
3. **Stall before the data phase** — simplest, viable *if* the driver's
   patience is milliseconds.  **Measure it; do not assume.**

## Reliability

Raw Ethernet is lossy: tags, a bounded retransmit timeout, retry on
transient failure rather than abort (the lesson from `sd-write-fast`,
where a single stalled read-back aborted a 35-minute transfer).  A dropped
reply must never wedge a SCSI transaction — it should surface as a
retryable SCSI error, the way `sd_ctrl`'s watchdog does.

## Boot chicken-and-egg

The Mac boots from a SCSI volume, so a net-backed *boot* volume needs the
link and host daemon live before the CPU is even released.  First cut:
keep SD as the boot volume on target *a* and put the network volume on
target *b*, which `vhdd_mux` already supports.  Full network boot is the
`boot_fsm`-over-network idea in `eth_bringup_handoff.md` §6 — later, and
separate.

## Open questions

- Does the Q700 driver permit disconnect (IDENTIFY)?  Decides item 1 above.
- What is the driver's actual tolerance for a delayed data phase?  Decides
  whether item 3 alone is enough.
- ~~Does `vhdd_sd` serve a multi-block request itself, or does `scsi.v`
  iterate block by block?~~  **RESOLVED, and the harder way.**  `rtl/vhdd.vh`
  passes `req_multi` and `req_block_count[15:0]` *to the provider*: a
  multi-block request arrives as ONE request and the provider must stream
  it.  So the network provider does need a windowing state machine; it does
  not inherit per-block granularity.

## Staging

Read-only, target *b*, stall-before-data-phase, no cache.  Smallest thing
that proves transport, framing and latency tolerance end to end.  Measure
the driver's patience.  Then readahead, then write-back, then
disconnect/reselect if the measurement demands it.
