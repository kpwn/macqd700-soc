// vhdd.vh — the virtual-HDD backing-store contract.
//
// WHAT THIS IS
// ════════════
// A *convention*, not a module.  `vhdd` names the seam between a SCSI
// disk target (the master, rtl/mac/scsi.v) and whatever actually stores
// the blocks (the provider).  The master knows only about 512-byte
// blocks, an LBA, and a byte stream; it knows nothing about SD cards,
// SPI, CMD17/CMD18, DDR, or partition layouts.
//
//   scsi.v  ──vhdd──►  vhdd_sd.v   (rtl/soc/vhdd_sd.v — SD-card volume)
//                  ►  provider B   (SCSI ID 1; vhdd_net under
//                                   ENABLE_NET_VHDD.  vhdd_ddr, the
//                                   DDR-backed volume, held this slot
//                                   until it was deleted 2026-09-10)
//
// WHY A HEADER AND NOT A `vhdd.v` PASS-THROUGH MODULE
// ═══════════════════════════════════════════════════
// There is exactly one provider today.  A `vhdd.v` wrapper between the
// master and `vhdd_sd` would be ~30 port declarations forwarding wires
// one-for-one, with no logic of its own — a second copy of the port list
// to keep in sync and an extra level of hierarchy in fpga_top and in
// every testbench, bought for nothing.  When a second provider lands,
// the *selection* between providers is real logic and a real module can
// earn its place then; scsi.v will not have to change, because it
// already speaks this contract and nothing else.
//
// THE PORTS
// ═════════
// Names below are the canonical contract names.  The master prefixes
// them `vh_` on its own port list (`vh_req_go`, `vh_busy`, ...); the
// provider uses them bare.  Everything is synchronous to the master's
// clock domain — the provider is responsible for any CDC below it.
//
//   ── configuration ──────────────────────────────────────────────────
//   num_lbas        [31:0]  volume capacity, in VHDD_BLOCK_BYTES blocks.
//                           Runtime input (a card/volume swap must not
//                           need a rebuild), driven by the platform to
//                           BOTH ends of the seam.
//
//   ── extent probe (master → provider, combinational, no side effect) ─
//   chk_lba         [31:0]  first block of the extent under consideration
//   chk_blocks      [23:0]  length of that extent, in blocks
//   chk_ok                  1 == that extent is addressable on this
//                           volume.  Fails closed on a zero-length
//                           extent, on capacity overflow, and on any
//                           provider-internal address overflow (e.g. a
//                           reserved-window bias pushing the mapped
//                           address past 2^32).  Evaluated every cycle,
//                           including before the first req_go — the
//                           master gates its transfer on it.
//
//   ── request (master → provider) ────────────────────────────────────
//   req_write               0 = read from volume, 1 = write to volume
//   req_multi               0 = single block, 1 = multi-block stream
//   req_lba         [31:0]  first block, volume-relative (NOT biased —
//                           any reserved window belongs to the provider)
//   req_block_count [15:0]  blocks this request will move (>= 1)
//   req_go                  1-cycle pulse.  req_write/req_multi/req_lba/
//                           req_block_count are already valid on the
//                           cycle req_go is high and stay stable until
//                           the next req_go.
//
//   ── completion (provider → master) ─────────────────────────────────
//   busy                    provider is working on a request
//   done                    1-cycle pulse at end of request
//   error                   sticky; qualify with `done`
//
//   ── read data (provider → master) ──────────────────────────────────
//   rd_valid                1-cycle per byte
//   rd_data         [7:0]
//   rd_ready                REAL back-pressure.  Holding it low pauses
//                           the byte stream at the source.  The master
//                           drives it low when its ring is within
//                           16 bytes of full, and only while a multi-
//                           block read is actually live — an abandoned
//                           stream must be allowed to drain into the
//                           void rather than park `busy` forever.
//                           Pausing does NOT stop the provider's
//                           bounded-response clock (see load-bearing
//                           behaviour 2 below): a master that holds
//                           rd_ready low long enough will get
//                           `done | error` rather than silence.
//
//   ── write data (master → provider) ─────────────────────────────────
//   wr_ready                1-cycle when the provider wants a byte
//   wr_valid                INFORMATIONAL ONLY.  It means "the master is
//                           sitting in its write-drain state"; it is not
//                           a handshake qualifier and the provider must
//                           not gate on it.  Producer-paced: the master
//                           presents wr_data in response to wr_ready.
//                           (Kept as-is deliberately: renaming or
//                           re-purposing it would churn every provider
//                           and every tb for nothing.  wr_avail below is
//                           the signal with teeth.)
//   wr_data         [7:0]
//   wr_avail                REAL back-pressure, the exact mirror of
//                           rd_ready.  1 == "the master has at least one
//                           byte ready RIGHT NOW".  A provider MUST NOT
//                           take a byte while this is low; holding it
//                           low pauses the stream at the consumer.
//                           The master drives it low only while it is
//                           actually co-running the ring with the
//                           initiator (a multi-block write, blocks
//                           2..N) and the ring is empty; in every other
//                           state — single-block writes, MODE SELECT,
//                           and the buffered TAIL of a multi-block
//                           write — the whole remaining payload is
//                           already in sec_buf and this reads 1.
//                           Providers that predate it, and masters that
//                           always have a byte, tie it to 1'b1 and are
//                           bit-for-bit unaffected.
//                           Pausing does NOT stop the provider's
//                           bounded-response clock (load-bearing
//                           behaviour 2 below): a provider that is
//                           starved long enough must still answer with
//                           `done | error`.
//
//                           WHY IT EXISTS (2026-08-08).  Without it the
//                           SD provider's sd_ctrl issued the next SPI
//                           byte unconditionally, so any producer stall
//                           longer than the ring's residue — on
//                           hardware, an interrupt pre-empting the
//                           Mac's pseudo-DMA loop — wrote the PREVIOUS
//                           block's stale bytes to the card at the
//                           correct LBA with GOOD status, and the
//                           master's ring counter clamped the resulting
//                           underflow at 0 instead of flagging it.  The
//                           read side had the identical defect until
//                           2026-07-15 (see rd_ready).  Same disease,
//                           same cure, other direction.
//
// BLOCK SIZE
// ══════════
// VHDD_BLOCK_BYTES is part of the contract, not an incidental constant:
// the master's sector ring, its byte counters and its high-water mark
// are all sized from it, and a SCSI block is 512 bytes anyway.  A
// provider that cannot serve 512-byte blocks does not implement this
// contract.  (The master's ring pointers are 9 bits wide and wrap on
// their own; that too assumes 512.)
//
// TWO BEHAVIOURS THAT ARE LOAD-BEARING
// ════════════════════════════════════
//  1. A multi-block READ enters the data phase as soon as ONE byte is
//     buffered and drains concurrently with the fill; a single-block
//     read waits for `done`.  See the comment at S_VH_WAIT_RD in
//     scsi.v — entering at a full block left the ring with zero
//     headroom and corrupted the first bytes of every multi-block read.
//  2. BOUNDED RESPONSE IS THE PROVIDER'S OBLIGATION.  A provider MUST
//     assert `done` within a bounded number of cycles of `req_go`,
//     whatever happens below it — a silent card, a stalled `rd_ready`,
//     a dead transport.  If it cannot complete the request it must
//     answer anyway, with `error` set.  Holding `busy` high is NOT a
//     licence to stall: it only tells the master that the answer has
//     not arrived yet, it does not extend the deadline.
//
//     Why the obligation lives here and not in the master: the master's
//     own VH_WAIT_TIMEOUT counts only while `busy` is low (it exists to
//     catch a provider that never even starts), so a provider that
//     parks with `busy` high is invisible to it.  There is no watchdog
//     above it either — an unbounded provider therefore holds the SCSI
//     target in S_VH_WAIT_RD/S_VH_WAIT_WR forever, which holds the
//     initiator's pseudo-DMA handshake forever, which hangs the CPU on
//     a peripheral-bus access that can never be acked.  One provider
//     that never answers wedges the whole machine.
//
//     A watchdog somewhere upstream is not a substitute: it can only
//     turn the hang into a bus error the OS has no recovery path for.
//     A provider that answers `done | error` gets turned into a SCSI
//     CHECK CONDITION, which the ROM and the OS both already retry.
//
//     The SD provider implements this as ONE global per-request
//     watchdog in sd_ctrl.v (REQ_WDOG_*), armed at `go`, incremented
//     unconditionally — not gated on transport progress, consumer
//     readiness, or anything else — and cleared only when the request
//     terminates.  Every per-phase timeout in that module is gated on
//     some event, which is precisely why they all froze together in the
//     deadlock this clause used to bless.  A future provider must
//     provide its own equivalent; it does not inherit one.

`ifndef VHDD_VH
`define VHDD_VH

// Block size of the virtual HDD, in bytes.  Fixed at 512 (see above).
`define VHDD_BLOCK_BYTES 512

`endif // VHDD_VH
