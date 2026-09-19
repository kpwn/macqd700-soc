| fpu_fsave_null_frame.s — verify FSAVE writes a valid 68040 frame.
|
| HISTORICAL NOTE: This test originally asserted FSAVE writes a NULL
| frame (4 bytes of 0x00).  After the IDLE-frame upgrade (commit
| date 2026-05-04), FSAVE now writes the 68040 IDLE frame
| (version/type byte 0x41) so OS code that does the optimisation
|   if ((frame[0] & 0xf) == 0) skip_restore;
| correctly takes the restore path.  This test was updated in place
| to track the new behaviour rather than retired since the round-trip
| FSAVE/FRESTORE plumbing it exercises is still valuable.
|
| The Q700 FPSP at 0x4088db40 inspects the saved frame's format byte
| and branches on it.  Format 0x00 (NULL) → FPSP exits via the "no FPU
| state pending" path; version/type 0x41 (IDLE) → FPSP exits cleanly.
|
| PASS sentinel: 0xC0FFEE00.
| FAIL sentinels:
|   0xDEAD0F41 — frame word != 0x41000000 (IDLE format violation)

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000

_start:
    lea     0x00010000, %a7

    | Pre-fill the 4 frame bytes with a non-zero value so a no-op store
    | can't accidentally pass the check.
    move.l  #0xDEADBEEF, -4(%a7)

    | FSAVE -(SP).
    .short  0xF327

    | Read the frame.  After FSAVE, A7 -= 4, so 0(%a7) is the format
    | long.  For the 68040 IDLE frame, version/type=0x41 — entire
    | long is 0x41000000.
    move.l  (%a7), %d0
    cmp.l   #0x41000000, %d0
    bne     _fail

    | FRESTORE (SP)+ to consume the frame back.
    .short  0xF35F

    | PASS.
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d2
    move.l  %d2, (%a1)
_halt:
    bra     _halt

_fail:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0F41, %d2
    move.l  %d2, (%a1)
_hf:
    bra     _hf
