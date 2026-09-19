| exc_double_fault_halt.s — exception during exception entry -> halt.
|
| Goal: provoke a fault during the act of taking a fault.  Per 68040
| UM §8.4.5.4 ("Multiple Exceptions"), if a bus / address error occurs
| while pushing the exception frame to SSP (e.g. because SSP itself
| points at an invalid address), the CPU latches a "double bus error"
| and HALTs.  This is the architectural "give up" signal.
|
| Construction:
|   1. Install a TRAP #0 handler at vec 32.
|   2. Set SSP to an address that is invalid for writes (we choose
|      0xFFFFFFE0 — high enough that any writeback faults if no DTT
|      covers it; below the sentinel page so the sentinel write at
|      0xFFFF0000 still works once we eventually halt).
|   3. Execute TRAP #0 — sequencer attempts to push 8-byte frame to
|      [0xFFFFFFE0..0xFFFFFFE7].  AXI returns SLVERR (no slave at
|      that range).  The CPU's exception sequencer takes a SECOND
|      bus error trying to dispatch vec-2 (because vec-2's frame
|      push ALSO targets the same SSP, recursively faults).
|   4. Halt.  Testbench detects HALT signal externally.
|
| EXPECTED OUTCOME: today the core does not implement the double-
| bus-error halt; instead it loops forever (re-raising vec-2 every
| frame push).  The testbench timeout fires and the test is marked
| TIMEOUT.  This is a coverage gap that the test reveals.
|
| When the double-fault halt lands, cpu_halted is asserted; the
| harness detects it and writes 0xC0FFEE00 to PASS_SENT on the CPU's
| behalf (sim/test infrastructure only — JTAG can also see cpu_halted
| and read out architectural state for post-mortem inspection).
|
| Note: the testbench finishes on ANY write to PASS_SENT, so we must
| NOT pre-write a marker there.
|
| PASS sentinel: 0xC0FFEE00 (written by harness on HALT detect).
| FAIL sentinels:
|   0xDEAD0D01 — control returned to mainline (no fault propagated)
|   0xDEAD0D02 — handler ran (push didn't fault as expected)
|   0xDEAD0D03 — bus-error handler ran (chain wasn't a true double fault)

    .text
    .org 0

    .equ PASS_SENT,  0xFFFF0000
    .equ BAD_SSP,    0xAAAA0000       | unmapped AXI region (DECERR per exc_bus_error.s)

_start:
    lea     0x00010000, %a7
    move.l  #_handler, 0x00000080      | vec 32 (TRAP #0)
    move.l  #_buserr,  0x00000008      | vec 2 (bus error)

    | Move SSP to the bad region.  Any subsequent frame push will
    | take a bus error, and that bus error's own frame push also
    | targets BAD_SSP — recursive fault → cpu_halted.
    lea     BAD_SSP, %a7

    | Drain the pipeline so the SSP shadow update lands before TRAP.
    | The a7_mirror_pending mechanism in commit.v takes one cycle to
    | propagate the new A7 into the SP-cache slot the exception
    | sequencer reads.
    nop
    nop
    nop
    nop

    | Trigger.
    trap    #0

    | Should not reach.
    move.l  #0xDEAD0D01, %d0
    lea     PASS_SENT, %a1
    move.l  %d0, (%a1)
_halt_fail:
    bra     _halt_fail

_handler:
    | If we somehow get here (SSP push succeeded — fault didn't fire
    | as expected), tag and halt.
    move.l  #0xDEAD0D02, %d0
    lea     PASS_SENT, %a1
    move.l  %d0, (%a1)
_halt_h:
    bra     _halt_h

_buserr:
    | If we get here once, it means the FIRST frame push faulted but
    | the bus-error vector dispatch did NOT itself fault (SSP somehow
    | recovered).  The test wanted a recursive fault — log and halt.
    move.l  #0xDEAD0D03, %d0
    lea     PASS_SENT, %a1
    move.l  %d0, (%a1)
_halt_be:
    bra     _halt_be
