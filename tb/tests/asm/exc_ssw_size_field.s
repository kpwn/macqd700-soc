| exc_ssw_size_field.s — verify 68040 Format-7 SSW SIZE / RW / FC fields
| are populated correctly on a STORE bus error.
|
| Construction:
|   1. Install vec-2 (bus error) handler.
|   2. Issue MOVE.W to unmapped 0xAAAA0000 → SLVERR → vec 2.
|   3. Handler reads SSW at A7+12 (low half of long at +12 — the SSW
|      lives at byte offset +14..+15 of the format-7 frame in big-endian).
|      Layout: word at A7+12 is high half (status word), word at A7+14
|      is low half — but note word ordering varies; we read the SSW as
|      the WORD at A7+12 (the documented offset of SSW in the frame
|      layout per 68040 UM §8.3.7).
|   4. Verify:
|       a. RW bit (8) = 0 (write)
|       b. ATC bit (10) = 1 (LSU set in_mmu=1 for vec 2)
|       c. SIZE field (bits 6:5) = 10 (word) for our MOVE.W
|       d. FC field (bits 2:0) = 5 (supervisor data — boot mode)
|
| PASS: 0xC0FFEE00.
| FAIL sentinels:
|   0xDEAD0501 — RW != 0 (write should clear bit 8)
|   0xDEAD0502 — ATC != 1 (LSU should set in_mmu)
|   0xDEAD0503 — SIZE != word
|   0xDEAD0504 — FC != 5

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000

_start:
    lea     0x00010000, %a7
    move.l  #_handler, 0x00000008   | vec 2

    | Trigger a WORD store to an unmapped address.
    lea     0xAAAA0000, %a0
    move.w  #0x1234, (%a0)           | SLVERR → vec 2 with SIZE=word

    | Unreachable.
_fail:
    lea     PASS_SENT, %a1
    move.l  #0xDEADFA17, %d1
    move.l  %d1, (%a1)
_halt_fail:
    bra     _halt_fail

_handler:
    | Format-7 frame layout (per 68040 UM §8.3.7):
    |   A7+0  : SR
    |   A7+2  : PC[31:16]
    |   A7+4  : PC[15:0]
    |   A7+6  : format/vec word (nibble = 7)
    |   A7+8  : EA[31:16]
    |   A7+10 : EA[15:0]
    |   A7+12 : SSW (special status word)
    |   A7+14 : reserved
    |   ...
    | Read the SSW at A7+12.  Our exception sequencer pushes SSW at
    | frame_word_data idx=6 → byte offset 12 within the frame.
    move.w  12(%a7), %d0
    move.w  %d0, %d7                 | save raw SSW for debug

    | Check (a) RW bit (bit 8) = 0 (write).
    btst    #8, %d0
    bne     _fail_rw                 | bit set → was reported as read

    | Check (b) ATC bit (bit 10) = 1 (LSU sets in_mmu=1).
    btst    #10, %d0
    beq     _fail_atc

    | Check (c) SIZE field (bits 6:5) = 2'b10 for word.
    move.w  %d7, %d1
    lsr.w   #5, %d1
    andi.w  #3, %d1
    cmp.w   #2, %d1
    bne     _fail_size

    | Check (d) FC field (bits 2:0) = 5 (supervisor data).
    move.w  %d7, %d1
    andi.w  #7, %d1
    cmp.w   #5, %d1
    bne     _fail_fc

    | All checks passed.
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d2
    move.l  %d2, (%a1)
_halt:
    bra     _halt

_fail_rw:
    move.l  #0xDEAD0501, %d2
    bra     _do_fail
_fail_atc:
    move.l  #0xDEAD0502, %d2
    bra     _do_fail
_fail_size:
    move.l  #0xDEAD0503, %d2
    bra     _do_fail
_fail_fc:
    move.l  #0xDEAD0504, %d2
_do_fail:
    lea     PASS_SENT, %a1
    move.l  %d2, (%a1)
_halt_f:
    bra     _halt_f
