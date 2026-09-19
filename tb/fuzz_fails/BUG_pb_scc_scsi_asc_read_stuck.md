# BUG: peripheral_bus read to combinational-ack pb_* slaves stalls forever

## Summary
In `rtl/sys/peripheral_bus.v` the pb-path read FSM sets `rd_ar_done <= 1`
and checks `!rd_r_done && pb_rd_ack && rd_ar_done` in the SAME clock-edge
always block.  Because non-blocking assignments read the previous value
of `rd_ar_done`, the condition is never satisfied on the cycle the ack
first appears — and the very next cycle `pb_rd_active` falls (because
`rd_ar_done` is now 1), dropping the combinational-ack slave's ack back
to 0.

## Affected peripherals
SCC, SCSI (combinational-ack implementations) and ASC — their
`pb_ack = pb_rd | pb_wr` assignment means ack is only high during the
one cycle pb_rd is asserted.  VIA1/VIA2 ack is registered so they track
the NEXT cycle's ack, which happens to line up with `rd_ar_done=1`.

Note: SCSI is registered too (rtl/mac/scsi.v line 99) so it actually
works; SCC and ASC (assign) are the stuck ones.  On hw the combinational
ack path is the one most at risk.

## Location
`rtl/sys/peripheral_bus.v`, read-side always block, the pb default case
(circa line 486).

## Consequence
A CPU read of the SCC or ASC I/O window never returns — the AXI R beat
is never issued; the CPU's LSU waits on dc_rvalid forever.  In practice
this would hang the Mac ROM boot the first time it probes the SCC.

## Repro (isolated)
See `tb/tb_peripheral_bus.cpp` scenarios `test_scc_decode`,
`test_scsi_decode`, `test_asc_decode` — the read-back check times out
on the 200-cycle AXI read loop.  The write half of each scenario
succeeds.

## Suggested fix
One of:
- Register pb_ack in `scc.v` / `asc.v` (to match via1/via2 pattern).
- Change peripheral_bus.v read FSM so `rd_ar_done <= 1` happens on a
  SEPARATE cycle from the first ack-poll, OR use a blocking assignment
  pattern that lets the ack-poll see the same-cycle rd_ar_done transition.
- Extend the pb_rd_active window: hold pb_rd high until the ack
  handshake completes (currently drops after rd_ar_done<=1).
