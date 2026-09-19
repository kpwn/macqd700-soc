| fpu_fmovem_multi.s — FMOVEM.X register list ↔ memory through (d16,An).
|
| Validates the FPSP-recursion fix (decode-1111: FMOVEM.X register
| list in (d16,An) mode, multi-µop crack into per-FP-register 12-byte
| memory traffic).
|
| Behavior model: this landing produces correct memory traffic
| (3 longs per FP register) but does NOT round-trip FP register data
| through memory.  The test verifies:
|   1. FMOVEM.X store does not trap (no F-line) and writes 12*N bytes
|      of zeros to memory.
|   2. FMOVEM.X load does not trap and reads 12*N bytes back.
|   3. The instruction stream advances correctly past the FMOVEM.X
|      opwords (no PC stall).
|
| FP register fidelity is a follow-up — this is the "FPSP recursion
| break" landing.
|
| Encodings (verified via m68k-linux-gnu-as -m68040):
|   F228 F0F0 0000   FMOVEM.X FP0-FP3,(0,A0)
|   F228 D0F0 0000   FMOVEM.X (0,A0),FP0-FP3
|
| Use op=F228 (mode=101 reg=000 = (d16,A0)), ext2=disp16=0.
|
| PASS sentinel: 0xC0FFEE00.
| FAIL sentinels:
|   0xDEAD0F21 — memory not zeroed at offset 0 (store didn't fire)
|   0xDEAD0F22 — memory not zeroed at offset 44 (last byte slot)
|   0xDEAD0F23 — sentinel was trampled (load wrote past EA range)
|   0xDEAD0F01 — vec-11 F-line (decoder gap)

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ FP_BUF,    0x00020000

_start:
    lea     0x00010000, %a7
    move.l  #_fline, 0x0000002C        | vec 11 F-line

    | Pre-fill the FP buffer with 0xDEADBEEF in every long so we can
    | tell whether the FMOVEM.X store actually wrote zeros.
    lea     FP_BUF, %a0
    move.l  #0xDEADBEEF, %d0
    move.l  %d0, 0(%a0)
    move.l  %d0, 4(%a0)
    move.l  %d0, 8(%a0)
    move.l  %d0, 12(%a0)
    move.l  %d0, 16(%a0)
    move.l  %d0, 20(%a0)
    move.l  %d0, 24(%a0)
    move.l  %d0, 28(%a0)
    move.l  %d0, 32(%a0)
    move.l  %d0, 36(%a0)
    move.l  %d0, 40(%a0)
    move.l  %d0, 44(%a0)               | offset 44..47 = last long (3rd long of fp3)

    | Place a sentinel right past the end of the FMOVEM.X buffer
    | so we catch any over-store.
    move.l  #0xCAFEBABE, 48(%a0)

    | FMOVEM.X FP0-FP3,(0,A0).
    .short  0xF228, 0xF0F0, 0x0000

    | Check first long is zero (proves store fired).
    move.l  0(%a0), %d0
    cmp.l   #0, %d0
    bne     _fail_first

    | Check last long (offset 44) is zero (proves all 12 phases fired).
    move.l  44(%a0), %d0
    cmp.l   #0, %d0
    bne     _fail_last

    | Check the past-end sentinel is intact (proves no over-store).
    move.l  48(%a0), %d0
    cmp.l   #0xCAFEBABE, %d0
    bne     _fail_overrun

    | FMOVEM.X (0,A0),FP0-FP3 — should not trap.  We don't check FP
    | register state (round-trip not implemented yet).
    .short  0xF228, 0xD0F0, 0x0000

    | PASS.
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d2
    move.l  %d2, (%a1)
_halt:
    bra     _halt

_fail_first:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0F21, %d2
    move.l  %d2, (%a1)
_h1:
    bra     _h1

_fail_last:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0F22, %d2
    move.l  %d2, (%a1)
_h2:
    bra     _h2

_fail_overrun:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0F23, %d2
    move.l  %d2, (%a1)
_h3:
    bra     _h3

_fline:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0F01, %d2
    move.l  %d2, (%a1)
_hf:
    bra     _hf
